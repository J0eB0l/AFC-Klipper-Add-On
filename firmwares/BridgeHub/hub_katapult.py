# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""
Flash a BoxTurtleUSB (an AFC-Lite running firmwares/BoxTurtleUSB) that is
plugged into a BridgeHub, over the hub's TCP port, with Katapult's flashtool.

Usage: python3 hub_katapult.py <hub ip>[:<port>] <firmware.bin>
       [--unit NAME [--key-file PATH] | --key KEY] [--flashtool PATH]

A hub that holds a link key wants it first: --unit reads the key AFC saved
for that unit (AFC.var.hub next to AFC.var), --key takes it as typed.

Stop Klipper first: the hub serves one TCP client at a time, and AFC would
take the port back.
"""
import argparse
import errno
import hashlib
import hmac
import json
import os
import pty
import select
import socket
import subprocess
import sys
import time
import tty
from typing import List, Optional, Tuple

DEFAULT_FLASHTOOL = os.path.expanduser("~/katapult/scripts/flashtool.py")
DEFAULT_KEY_FILE = os.path.expanduser("~/printer_data/config/AFC/AFC.var.hub")
HUB_PORT = 4001
CONNECT_TIMEOUT = 5.0
GREETING_WAIT = 1.0
AUTH_WAIT = 3.0
REPLY_WAIT = 2.0
BOOT_WAIT = 4.0
INFO_TRIES = 3


def log(msg: str) -> None:
    """
    Print one progress line.

    :param msg: text to print
    """
    print(f"hub: {msg}", flush=True)


def parse_target(spec: str) -> Tuple[str, int]:
    """
    Split `<host>[:<port>]` into its parts, the port being the hub's (4001)
    when none is given. A bare IPv6 address takes the default port;
    brackets make a port plain ([::1]:4001).

    :param spec: the hub's address and TCP port
    :return Tuple[str, int]: host and port
    """
    port = str(HUB_PORT)
    bad = False
    if spec.startswith("["):
        host, _, tail = spec[1:].partition("]")
        if tail:
            bad = not tail.startswith(":")
            port = tail[1:]
    elif spec.count(":") == 1:
        host, _, port = spec.partition(":")
    else:
        host = spec
    if bad or not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise ValueError(f"{spec} is not <host>[:<port>]")
    return host, int(port)


def saved_key(path: str, unit: str) -> Optional[str]:
    """
    The link key AFC made for a unit's hub and saved in its key file.

    :param path: AFC's key file (AFC.var.hub)
    :param unit: the AFC unit's name, such as Turtle_1
    :return Optional[str]: the key, or None (logged) when there is none
    """
    try:
        with open(path) as f:
            keys = json.load(f)
    except (OSError, ValueError) as e:
        log(f"cannot read {path}: {e}")
        return None
    key = keys.get(unit) if isinstance(keys, dict) else None
    if not isinstance(key, str) or not key:
        log(f"{path} has no link key for {unit}")
        return None
    return key


def read_lines(sock: socket.socket, secs: float) -> List[dict]:
    """
    Read the unit's JSON lines for a while and print each.

    :param sock: connection to the hub
    :param secs: how long to read
    :return List[dict]: the JSON objects read
    """
    buf = b""
    found: List[dict] = []
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ready, _, _ = select.select([sock], [], [], 0.05)
        if not ready:
            continue
        data = sock.recv(4096)
        if not data:
            raise ConnectionError("the hub closed the connection")
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").strip()
            try:
                obj = json.loads(text)
            except ValueError:
                continue
            if isinstance(obj, dict):
                log(f"< {text}")
                found.append(obj)
    return found


def read_line(sock: socket.socket, secs: float) -> Optional[dict]:
    """
    Read one line, a byte at a time so nothing after it leaves the socket.

    :param sock: connection to the hub
    :param secs: how long to wait for the whole line
    :return Optional[dict]: the line's JSON object, or None (no line in
        time, or not an object)
    """
    buf = b""
    end = time.monotonic() + secs
    while time.monotonic() < end:
        ready, _, _ = select.select([sock], [], [], 0.05)
        if not ready:
            continue
        byte = sock.recv(1)
        if not byte:
            raise ConnectionError("the hub closed the connection")
        if byte != b"\n":
            buf += byte
            continue
        try:
            obj = json.loads(buf.decode(errors="replace"))
        except ValueError:
            return None
        return obj if isinstance(obj, dict) else None
    return None


def hub_auth(sock: socket.socket, key: Optional[str]) -> bool:
    """
    Pass the hub's greeting, which comes before any of the unit's bytes: a
    hub that holds a link key sends a challenge, answered with the first 32
    bytes of HMAC-SHA512(key, nonce); an open hub, or a plain TCP serial
    server that greets with nothing, needs no key.

    :param sock: connection to the hub
    :param key: the hub's link key, or None
    :return bool: True when the unit may be talked to
    """
    greeting = read_line(sock, GREETING_WAIT)
    if greeting is None or greeting.get("evt") != "auth":
        return True
    if greeting.get("open"):
        if key:
            log("the hub has no link key, so the key given is not used")
        return True
    if "nonce" not in greeting:
        log(f"the hub sent an unknown greeting {greeting}")
        return False
    if not key:
        log("the hub holds a link key: pass --unit <AFC unit name> or --key <key>")
        return False
    try:
        nonce = bytes.fromhex(str(greeting["nonce"]))
    except ValueError:
        log("the hub sent a bad challenge")
        return False
    mac = hmac.new(key.encode(), nonce, hashlib.sha512).digest()[:32]
    # Compact, as the hub reads it.
    line = json.dumps({"cmd": "auth", "mac": mac.hex()}, separators=(",", ":"))
    sock.sendall(line.encode() + b"\n")
    reply = read_line(sock, AUTH_WAIT)
    if reply is None:
        log("no answer from the hub to the link key")
        return False
    if reply.get("ok") != 1:
        log("the hub refused the link key")
        return False
    log("the hub took the link key")
    return True


def ask(sock: socket.socket, rid: int, cmd: str, secs: float) -> Optional[dict]:
    """
    Send one request to the unit and wait for its reply.

    :param sock: connection to the hub
    :param rid: request id, echoed in the reply
    :param cmd: the command
    :param secs: how long to wait
    :return Optional[dict]: the reply, or None if none came
    """
    sock.sendall(b"\n" + json.dumps({"id": rid, "cmd": cmd}).encode() + b"\n")
    for obj in read_lines(sock, secs):
        if obj.get("id") == rid:
            return obj
    return None


def open_tty() -> Tuple[int, str]:
    """
    Make a raw pseudo terminal for flashtool. Only the master side is kept
    open, since flashtool refuses a tty that another process holds.

    :return Tuple[int, str]: the master's fd and the tty's path
    """
    master, slave = pty.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    os.close(slave)
    return master, path


def relay(sock: socket.socket, master: int, proc: subprocess.Popen) -> int:
    """
    Carry bytes between the tty and the hub until flashtool exits.

    :param sock: connection to the hub
    :param master: the tty's master fd
    :param proc: the running flashtool
    :return int: flashtool's exit code
    """
    while proc.poll() is None:
        ready, _, _ = select.select([master, sock], [], [], 0.2)
        if master in ready:
            try:
                data = os.read(master, 4096)
            except OSError as e:
                if e.errno != errno.EIO:
                    raise
                # Nothing has the tty open right now.
                data = b""
                time.sleep(0.02)
            if data:
                sock.sendall(data)
        if sock in ready:
            data = sock.recv(4096)
            if not data:
                log("the hub closed the connection during the flash")
                proc.kill()
                break
            os.write(master, data)
    return proc.wait()


def flash(sock: socket.socket, flashtool: str, firmware: str) -> int:
    """
    Ask the unit into Katapult, flash it through the hub, then ask the new
    build for its info, which also tells its boot guard that a host is there.

    :param sock: connection to the hub
    :param flashtool: path of Katapult's flashtool.py
    :param firmware: path of the firmware .bin
    :return int: 0 when flashed and answering, else non-zero
    """
    if ask(sock, 1, "bootloader", REPLY_WAIT) is None:
        log("no reply to bootloader (the unit may already be in Katapult)")
    else:
        log("the unit is restarting into Katapult")
    read_lines(sock, BOOT_WAIT)
    master, path = open_tty()
    log(f"flashtool on {path}")
    try:
        rc = relay(sock, master, subprocess.Popen(
            [sys.executable, flashtool, "-d", path, "-f", firmware]))
    finally:
        os.close(master)
    log(f"flashtool exit {rc}")
    if rc != 0:
        return rc
    for rid in range(2, 2 + INFO_TRIES):
        time.sleep(BOOT_WAIT / 2)
        if ask(sock, rid, "info", REPLY_WAIT) is not None:
            log("the new build is running")
            return 0
    log("no reply from the new build")
    return 2


def main(argv: Optional[List[str]] = None) -> int:
    """
    Command line entry point.

    :param argv: arguments, without the program name
    :return int: exit code
    """
    parser = argparse.ArgumentParser(
        description="Flash a BoxTurtleUSB through a BridgeHub with Katapult.")
    parser.add_argument("target", help="the hub's <ip>[:<port>]")
    parser.add_argument("firmware", help="the firmware .bin")
    parser.add_argument("--flashtool", default=DEFAULT_FLASHTOOL,
                        help="Katapult's flashtool.py")
    which = parser.add_mutually_exclusive_group()
    which.add_argument("--unit", help="the AFC unit whose saved link key to use")
    which.add_argument("--key", help="the hub's link key")
    parser.add_argument("--key-file", default=DEFAULT_KEY_FILE,
                        help="where AFC saves link keys (with --unit)")
    args = parser.parse_args(argv)
    try:
        host, port = parse_target(args.target)
    except ValueError as e:
        parser.error(str(e))
    for path in (args.firmware, args.flashtool):
        if not os.path.isfile(path):
            log(f"{path} not found")
            return 1
    key = args.key
    if args.unit:
        key = saved_key(args.key_file, args.unit)
        if key is None:
            return 1
    try:
        sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT)
    except OSError as e:
        log(f"cannot connect to {host}:{port}: {e}")
        return 1
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    log(f"connected to {host}:{port}")
    try:
        if not hub_auth(sock, key):
            return 1
        return flash(sock, args.flashtool, args.firmware)
    except ConnectionError as e:
        log(str(e))
        return 1
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
