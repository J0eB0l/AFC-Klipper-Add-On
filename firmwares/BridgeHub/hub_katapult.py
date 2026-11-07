# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""
Flash a BoxTurtleUSB (an AFC-Lite running firmwares/BoxTurtleUSB) that is
plugged into a BridgeHub, over the hub's TCP port, with Katapult's flashtool.

Usage: python3 hub_katapult.py <hub ip>:<port> <firmware.bin> [--flashtool PATH]

Stop Klipper first: the hub serves one TCP client at a time, and AFC would
take the port back.
"""
import argparse
import errno
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
CONNECT_TIMEOUT = 5.0
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
    Split `<host>:<port>` into its parts.

    :param spec: the hub's address and TCP port
    :return Tuple[str, int]: host and port
    """
    host, sep, port = spec.rpartition(":")
    if not sep or not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise ValueError(f"{spec} is not <host>:<port>")
    return host.strip("[]"), int(port)


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
    parser.add_argument("target", help="the hub's <ip>:<port>")
    parser.add_argument("firmware", help="the firmware .bin")
    parser.add_argument("--flashtool", default=DEFAULT_FLASHTOOL,
                        help="Katapult's flashtool.py")
    args = parser.parse_args(argv)
    try:
        host, port = parse_target(args.target)
    except ValueError as e:
        parser.error(str(e))
    for path in (args.firmware, args.flashtool):
        if not os.path.isfile(path):
            log(f"{path} not found")
            return 1
    try:
        sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT)
    except OSError as e:
        log(f"cannot connect to {host}:{port}: {e}")
        return 1
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    log(f"connected to {host}:{port}")
    try:
        return flash(sock, args.flashtool, args.firmware)
    except ConnectionError as e:
        log(str(e))
        return 1
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
