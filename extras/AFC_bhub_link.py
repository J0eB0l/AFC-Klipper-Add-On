# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024-2026 Armored Turtle
#
# A unit on a BridgeHub (firmwares/BridgeHub), or on anything that relays a
# unit's serial bytes the same way: a tcp:// serial_port and the hub's link
# key. Shared by every unit type that can sit on a hub, so they all connect
# one way.
#
# The hub greets each connection before any of the unit's bytes:
#   keyed: {"evt":"auth","nonce":"<32 hex>"}, answered with
#          {"cmd":"auth","mac":"<64 hex>"}, the first 32 bytes of
#          HMAC-SHA512(key, nonce)
#   open:  {"evt":"auth","open":1}, claimed with {"cmd":"linkkey","key":"<key>"}
# and replies {"evt":"auth","ok":1} (or ok 0 and closes). AFC makes a unit's
# key the first time it meets an open hub and keeps it in <VarFile>.hub; a
# tcp_key in the unit's section is used instead. A server that sends no
# greeting (ser2net) is used as it is.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import hashlib
import hmac
import ipaddress
import json
import os
import secrets
import select
import socket
import threading
import time

from typing import Any, Callable, Dict, Optional, Tuple

# serial_port prefix for a unit on a hub or another raw TCP serial server.
TCP_SCHEME = "tcp://"
# A BridgeHub's port, taken when serial_port names none.
HUB_PORT = 4001
# Longest link key a BridgeHub takes.
LINK_KEY_MAX = 64
# A hub greets a connection at once; a server that hasn't by then (ser2net)
# doesn't. The hub's answer to a key takes a flash write.
GREETING_TIME = 1.0
AUTH_REPLY_TIME = 2.0
CONNECT_TIME = 3.0
# Longest line the hub sends during the exchange.
AUTH_LINE_MAX = 160
# The key file is shared by every unit; ACE units save to it off the reactor.
_KEY_FILE_LOCK = threading.Lock()

BOOT_HINT = "hold the hub's BOOT button for 10 s to clear it"
NO_KEY_FOR_HUB = f"the hub holds a link key this printer doesn't have; {BOOT_HINT}"
BAD_CHALLENGE = "the hub sent a bad challenge"
NO_KEY_TO_GIVE = "no link key to give the hub"
REFUSED = f"the hub refused the link key; {BOOT_HINT}"
NO_ANSWER = "no answer from the hub to the link key"


class HubLinkError(Exception):
    """The hub's link key exchange failed; the message says why."""


def parse_tcp(spec: str) -> Tuple[str, int]:
    """
    Splits a tcp://<ip>[:<port>] serial_port into its address and port, the
    port being a BridgeHub's (4001) when none is given. The address must be
    an IP: a name lookup would block Klipper's reactor. An IPv6 address that
    reads as a whole address takes the default port; brackets make the port
    plain ([::1]:4001).

    :param spec: The serial_port value
    :return tuple: (address, port)
    :raises ValueError: When the address or the port is bad
    """
    rest = spec[len(TCP_SCHEME):].rstrip("/")
    if rest.startswith("["):
        host, _, tail = rest[1:].partition("]")
        sep, port = tail[:1], tail[1:]
        bad = tail and sep != ":"
    elif rest.count(":") == 1:
        host, sep, port = rest.partition(":")
        bad = False
    else:
        host, sep, port, bad = rest, "", "", False
        try:
            ipaddress.ip_address(rest)
        except ValueError:
            host, sep, port = rest.rpartition(":")
    try:
        ipaddress.ip_address(host)
        number = int(port) if sep else HUB_PORT
    except ValueError:
        number = 0
    if bad or not 0 < number < 65536:
        error_string = f"serial_port {spec} is not tcp://<ip address>[:<port>]"
        raise ValueError(error_string)
    return host, number


def check_link_key(key: str) -> bool:
    """
    A link key a BridgeHub takes: 1 to 64 printable characters with no
    space, quote or backslash.

    :param key: The key
    :return bool: True if the hub takes it
    """
    return (0 < len(key) <= LINK_KEY_MAX
            and all("!" <= c <= "~" and c not in "\\\"" for c in key))


def bad_tcp_key(key: Optional[str]) -> Optional[str]:
    """
    Checks a unit's tcp_key option.

    :param key: The option's value, or None when unset
    :return str: Why it can't be used, or None when it can
    """
    if key is None or check_link_key(key):
        return None
    return (f"tcp_key must be 1 to {LINK_KEY_MAX} characters "
            f"with no spaces, quotes or backslashes")


def link_key_file(var_file: str) -> str:
    """
    :param var_file: AFC's VarFile setting
    :return str: Where the link keys AFC made for hubs are kept, next to
        AFC's other saved state
    """
    return os.path.expanduser(f"{var_file}.hub")


def stored_link_key(var_file: str, name: str, tcp_key: Optional[str], make: bool,
                    logger: Any) -> Optional[str]:
    """
    The key for a unit's hub: tcp_key when set, else the one AFC saved for
    the unit, else (with make) a new one, saved first so a hub never holds a
    key AFC lost.

    :param var_file: AFC's VarFile setting
    :param name: The unit's name
    :param tcp_key: The unit's tcp_key, or None
    :param make: Make and save a key when none is saved
    :param logger: Where a failed save is reported
    :return str: The key, or None (none saved, or it could not be saved)
    """
    if tcp_key:
        return tcp_key
    with _KEY_FILE_LOCK:
        return _saved_or_new_key(link_key_file(var_file), name, make, logger)


def _saved_or_new_key(path: str, name: str, make: bool, logger: Any) -> Optional[str]:
    """
    stored_link_key's file work, under its lock.

    :param path: The key file
    :param name: The unit's name
    :param make: Make and save a key when none is saved
    :param logger: Where a failed save is reported
    :return str: The key, or None
    """
    try:
        with open(path) as f:
            keys = json.load(f)
    except (OSError, ValueError):
        keys = {}
    if not isinstance(keys, dict):
        keys = {}
    key = keys.get(name)
    if isinstance(key, str) and check_link_key(key):
        return key
    if not make:
        return None
    key = secrets.token_hex(16)
    keys[name] = key
    tmp = f"{path}.tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(keys, f, indent=2)
        os.replace(tmp, path)
    except OSError as e:
        logger.warning(f"{name}: cannot save the hub's link key in {path}: {e}")
        return None
    return key


def auth_line(msg: Dict[str, Any]) -> bytes:
    """
    :param msg: A flat dict of strings
    :return bytes: The compact JSON line the hub reads (it takes a claim
        only in exactly this form)
    """
    return (json.dumps(msg, separators=(",", ":")) + "\n").encode()


def hub_answer(greeting: Dict[str, Any],
               link_key: Callable[[bool], Optional[str]]) -> Tuple[bytes, bool]:
    """
    What to send a hub that greeted: the answer to its challenge, or this
    unit's key for an open hub.

    :param greeting: The hub's greeting line
    :param link_key: Gets the unit's key; its argument says to make one
    :return tuple: (the line to send, True when it claims an open hub)
    :raises HubLinkError: When there is nothing to send
    """
    if "nonce" in greeting:
        key = link_key(False)
        if not key:
            raise HubLinkError(NO_KEY_FOR_HUB)
        try:
            nonce = bytes.fromhex(str(greeting["nonce"]))
        except ValueError:
            raise HubLinkError(BAD_CHALLENGE)
        mac = hmac.new(key.encode(), nonce, hashlib.sha512).digest()[:32]
        return auth_line({"cmd": "auth", "mac": mac.hex()}), False
    if greeting.get("open"):
        key = link_key(True)
        if not key:
            raise HubLinkError(NO_KEY_TO_GIVE)
        return auth_line({"cmd": "linkkey", "key": key}), True
    raise HubLinkError(f"the hub sent an unknown greeting {greeting}")


def hub_verdict(reply: Optional[Dict[str, Any]]) -> None:
    """
    Checks the hub's reply to the answer or the claim.

    :param reply: The reply line, or None when none came
    :raises HubLinkError: When the hub said no, or nothing
    """
    if reply is None:
        raise HubLinkError(NO_ANSWER)
    if reply.get("ok") != 1:
        raise HubLinkError(REFUSED)


def is_auth(msg: Any) -> bool:
    """
    :param msg: A parsed line
    :return bool: True for one of the hub's link key lines
    """
    return isinstance(msg, dict) and msg.get("evt") == "auth"


class HubSocket:
    """
    A connection to a unit on a hub, past the link key exchange, read and
    written like the pyserial port a unit's code uses: read() never blocks,
    write() gives up after 0.5 s like pyserial's write_timeout.
    """
    WRITE_TIMEOUT = 0.5

    def __init__(self, sock: socket.socket, port: str, rest: bytes = b"") -> None:
        """
        :param sock: The connected socket
        :param port: The serial_port it was opened as, for errors
        :param rest: Unit bytes that came in with the hub's last line
        """
        self.sock = sock
        self.port = port
        self._rest = rest
        sock.setblocking(False)

    def fileno(self) -> int:
        """
        :return int: The socket's fd, for the reactor
        """
        return self.sock.fileno()

    def read(self, size: int = 1) -> bytes:
        """
        :param size: Most bytes to return
        :return bytes: What has arrived, b"" for nothing yet
        :raises OSError: When the hub closed the connection
        """
        if self._rest:
            data, self._rest = self._rest[:size], self._rest[size:]
            return data
        try:
            data = self.sock.recv(size)
        except (BlockingIOError, InterruptedError):
            return b""
        if not data:
            raise OSError(f"{self.port}: the hub closed the connection (disconnected)")
        return data

    def write(self, data: bytes) -> int:
        """
        :param data: Bytes for the unit
        :return int: How many were sent (all of them)
        :raises OSError: When the hub takes nothing for 0.5 s
        """
        view = memoryview(data)
        deadline = time.monotonic() + self.WRITE_TIMEOUT
        while view:
            try:
                view = view[self.sock.send(view):]
                continue
            except (BlockingIOError, InterruptedError):
                pass
            left = deadline - time.monotonic()
            if left <= 0 or not select.select([], [self.sock], [], left)[1]:
                raise OSError(f"{self.port}: write timeout")
        return len(data)

    def flush(self) -> None:
        """
        Nothing to do: write() sends at once.
        """

    def reset_input_buffer(self) -> None:
        """
        Drops what has arrived and not been read.
        """
        self._rest = b""
        while True:
            try:
                if not self.sock.recv(4096):
                    return
            except OSError:
                return

    def reset_output_buffer(self) -> None:
        """
        Nothing to do: nothing is held back.
        """

    def close(self) -> None:
        """
        Closes the connection.
        """
        self.sock.close()


def _read_auth(sock: socket.socket, buf: bytearray, timeout: float) -> Optional[Dict[str, Any]]:
    """
    Waits for the hub's next line.

    :param sock: The connection
    :param buf: Bytes read and not used yet; a hub's line is taken out of
        it, anything else (the unit's own bytes) stays
    :param timeout: Seconds to wait
    :return dict: The line, or None when none came (or it was not a hub's)
    :raises HubLinkError: When the connection closed
    """
    end = time.monotonic() + timeout
    while b"\n" not in buf:
        left = end - time.monotonic()
        if left <= 0 or len(buf) > AUTH_LINE_MAX:
            return None
        if not select.select([sock], [], [], left)[0]:
            return None
        data = sock.recv(4096)
        if not data:
            raise HubLinkError("the hub closed the connection")
        buf += data
    line, _, rest = bytes(buf).partition(b"\n")
    try:
        msg = json.loads(line)
    except ValueError:
        return None
    if not is_auth(msg):
        return None
    buf[:] = rest
    return msg


def open_hub(spec: str, link_key: Optional[Callable[[bool], Optional[str]]], name: str,
             logger: Any, tcp_key: Optional[str] = None) -> HubSocket:
    """
    Connects to a unit on a hub and passes the link key exchange. Blocks
    (for up to CONNECT_TIME + GREETING_TIME + AUTH_REPLY_TIME), so it runs
    off the reactor.

    :param spec: The tcp:// serial_port
    :param link_key: Gets the unit's key (its argument says to make one),
        or None when the unit has no way to
    :param name: The unit's name, for the log
    :param logger: Takes info and warning lines
    :param tcp_key: The unit's tcp_key, only to say it went unused
    :return HubSocket: The connection, ready for the unit's bytes
    :raises OSError: When the connect fails
    :raises HubLinkError: When the exchange fails
    """
    host, port = parse_tcp(spec)
    sock = socket.create_connection((host, port), timeout=CONNECT_TIME)
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        buf = bytearray()
        greeting = _read_auth(sock, buf, GREETING_TIME)
        if greeting is None:
            if tcp_key:
                logger.warning(f"{name}: {spec} asked for no link key, so tcp_key is not used")
            return HubSocket(sock, spec, bytes(buf))
        line, claimed = hub_answer(greeting, link_key or (lambda make: None))
        sock.sendall(line)
        hub_verdict(_read_auth(sock, buf, AUTH_REPLY_TIME))
        if claimed:
            logger.info(f"{name}: link key stored on the hub at {spec}")
        return HubSocket(sock, spec, bytes(buf))
    except BaseException:
        sock.close()
        raise
