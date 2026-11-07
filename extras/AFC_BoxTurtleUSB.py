# Armored Turtle Automated Filament Control
#
# BoxTurtle on its own USB firmware (boxturtle_fw, protocol boxturtle-1).
#
# The AFC-Lite runs the lanes, espoolers, TurtleNeck and LEDs itself and
# talks JSON lines over USB, the way an ACE does, so unplugging it never
# shuts Klipper down. Each lane gets a TurtleDrive where a Klipper stepper
# would be, so AFC's own BoxTurtle load, unload, calibration and prep code
# runs unchanged with the moves going to the board. The TurtleNeck switches
# live on the board too; TurtleNeck stands in for the AFC_buffer.
#
# An external buffer works too (an AFC_buffer on a Klipper MCU, such as an
# analog PSF): name it with `buffer` on the unit, a lane or the extruder
# (with `turtleneck: False`). The board then follows the extruder position
# scaled by that buffer's multiplier, and the buffer does the ramming and
# fault checks like it does for a stepper lane.
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import errno
import fcntl
import glob
import ipaddress
import json
import os
import select
import socket
import termios
import traceback

from configparser import Error as error
from typing import TYPE_CHECKING, Any, Callable, Dict, Optional, Tuple

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLaneState, SpeedMode
except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_BoxTurtle import afcBoxTurtle
except: raise error(ERROR_STR.format(import_lib="AFC_BoxTurtle", trace=traceback.format_exc()))

PROTO = "boxturtle-1"
PORT_GLOB = "/dev/serial/by-id/usb-AFC_BoxTurtle_*-if00"
# serial_port prefix for a board on a raw TCP serial server.
TCP_SCHEME = "tcp://"
# The AFC-Lite's LED headers, RGB1 to RGB4, and the color orders the board
# drives.
LED_HEADERS = 4
LED_ORDERS = ("GRB", "GRBW", "RGB", "RGBW")
NUM_LANES = 4
# Longest line the board reads, newline included; it drops longer ones.
LINE_MAX = 255
# Room for any request id when sizing a line before it gets its id.
ID_ROOM = 9999999

# Firmware fault codes that need AFC to act. Job failures come back to the
# caller that started the job, and a prep runout comes as a switch event.
ASSIST_FAULTS = ("clog", "not_feeding")
TMC_FAULTS = ("tmc_uart", "tmc_ot", "tmc_short", "tmc_reset", "tmc_uv",
              "tmc_unknown")


def encode_line(msg: Dict[str, Any]) -> bytes:
    """
    Encodes one request as a JSON line the firmware's parser reads. Floats
    are written with 3 decimals, since the parser takes no exponents.

    :param msg: Flat dict of str, int, float or bool values
    :return bytes: The line, ending in a newline
    """
    parts = []
    for key, value in msg.items():
        if isinstance(value, bool):
            text = "true" if value else "false"
        elif isinstance(value, int):
            text = str(value)
        elif isinstance(value, float):
            text = f"{value:.3f}"
        else:
            text = json.dumps(str(value))
        parts.append(f'"{key}":{text}')
    return ("{" + ",".join(parts) + "}\n").encode()


def split_fields(cmd: str, fields: Dict[str, Any], keep: tuple = ()) -> list:
    """
    Splits one request's fields over as many lines as it takes to keep
    each under LINE_MAX. Fields named in keep go on every line.

    :param cmd: Firmware command
    :param fields: Fields to send
    :param keep: Field names every line needs (such as lane)
    :return list: One dict of fields per line
    """
    base = {k: fields[k] for k in keep if k in fields}
    parts = []
    part = dict(base)
    for key, value in fields.items():
        if key in base:
            continue
        trial = dict(part, **{key: value})
        if (len(encode_line({"id": ID_ROOM, "cmd": cmd, **trial})) > LINE_MAX
                and len(part) > len(base)):
            parts.append(part)
            trial = dict(base, **{key: value})
        part = trial
    parts.append(part)
    return parts


def parse_tcp(spec: str) -> Tuple[str, int]:
    """
    Splits a tcp://<ip>:<port> serial_port into its address and port. The
    address must be an IP: a name lookup would block Klipper's reactor.

    :param spec: The serial_port value
    :return tuple: (address, port)
    :raises ValueError: When the address or the port is missing or bad
    """
    host, sep, port = spec[len(TCP_SCHEME):].rstrip("/").rpartition(":")
    if host.startswith("[") and host.endswith("]"):
        host = host[1:-1]
    try:
        ipaddress.ip_address(host)
        number = int(port)
    except ValueError:
        number = 0
    if not sep or not 0 < number < 65536:
        error_string = f"serial_port {spec} is not tcp://<ip address>:<port>"
        raise ValueError(error_string)
    return host, number


def parse_turtle_led(spec: str, counts: Dict[int, int]) -> list:
    """
    Reads a lane's turtle_led: one or more RGB<header>:<first>[-<last>]
    ranges, comma separated, with LEDs counted from 1 on each header.

    :param spec: The option value
    :param counts: LEDs on each header, by header number
    :return list: (header, first, last) for each range
    :raises ValueError: When a range is malformed or past its header's LEDs
    """
    ranges = []
    for part in (p.strip() for p in spec.split(",")):
        head, sep, span = part.upper().partition(":")
        first, _, last = span.partition("-")
        try:
            header = int(head[3:]) if head.startswith("RGB") else 0
            first_led = int(first)
            last_led = int(last) if last else first_led
        except ValueError:
            header = 0
        if not sep or header not in counts or not 1 <= first_led <= last_led:
            error_string = (f"turtle_led {part} is not "
                            f"RGB<1-{LED_HEADERS}>:<first>[-<last>]")
            raise ValueError(error_string)
        if last_led > counts[header]:
            error_string = (f"turtle_led {part} is past the {counts[header]} LEDs "
                            f"of rgb{header}_chain_count")
            raise ValueError(error_string)
        ranges.append((header, first_led, last_led))
    return ranges


def led_hex(color) -> Optional[str]:
    """
    Turns an AFC LED color ("r,g,b,w" string or list, 0..1 each) into the
    firmware's RRGGBBWW hex.

    :param color: AFC color string or list of floats
    :return str: Hex color, or None if the color can't be read
    """
    try:
        if isinstance(color, str):
            values = [float(v) for v in color.split(",")]
        else:
            values = [float(v) for v in color]
    except (TypeError, ValueError):
        return None
    values = (values + [0.0, 0.0, 0.0, 0.0])[:4]
    return "".join(f"{max(0, min(255, int(round(v * 255)))):02X}" for v in values)


class TurtleLink:
    """
    The USB link: opens the board's tty (or connects to a raw TCP serial
    server) without blocking, reads JSON lines in a reactor fd handler,
    matches replies to requests by id, and keeps the board's link-loss timer
    fed with pings. Reconnects on its own, looking often for a while after a
    drop, since a USB drop is usually back within a second.
    """
    PING_TIME = 0.5
    RX_TIMEOUT = 3.0
    CONNECT_POLL_TIME = 0.05
    RETRY_TIME = 2.0
    FAST_RETRY_TIME = 0.1
    FAST_RETRY_WINDOW = 10.0

    def __init__(self, unit: "afcBoxTurtleUSB", port: str, uid: Optional[str]):
        """
        :param unit: The unit that owns this link
        :param port: tty path, "auto" to find the board by its USB name, or
            tcp://<ip>:<port> for a board on a raw TCP serial server
        :param uid: Chip serial to pick one board when several are plugged in
        """
        self.unit = unit
        self.reactor = unit.reactor
        self.logger = unit.logger
        self.port_cfg = port
        self.uid = uid
        self.fd: Optional[int] = None
        self.sock: Optional[socket.socket] = None
        self.connecting = False
        self.fd_handle = None
        self.port: Optional[str] = None
        self.connected = False
        self.info: Dict[str, Any] = {}
        self.last_error = ""
        self._rx = b""
        self._tx = b""
        self._next_id = 1
        self._pending: Dict[int, Any] = {}
        self._waiters: list = []
        self._last_rx = 0.0
        self._dropped_at: Optional[float] = None
        self.timer = self.reactor.register_timer(self._timer_cb)

    def start(self) -> None:
        """
        Starts trying to reach the board.
        """
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def stop(self) -> None:
        """
        Closes the link for good (Klipper shutting down).
        """
        self.reactor.update_timer(self.timer, self.reactor.NEVER)
        self.close("klipper stopping", notify=False)

    def find_port(self) -> Optional[str]:
        """
        Finds the board's tty.

        :return str: The tty path (or the tcp:// address, which is always
            tried), or None when the board is not plugged in
        """
        if self.port_cfg.startswith(TCP_SCHEME):
            return self.port_cfg
        if self.port_cfg != "auto":
            return self.port_cfg if os.path.exists(self.port_cfg) else None
        ports = sorted(glob.glob(PORT_GLOB))
        if self.uid:
            ports = [p for p in ports if self.uid in p]
        return ports[0] if ports else None

    def _timer_cb(self, eventtime: float) -> float:
        """
        Opens the port when the board shows up, pings it while connected and
        drops a link that went quiet.

        :param eventtime: Reactor time
        :return float: Next wake time
        """
        if self.fd is None:
            port = self.find_port()
            if port is not None:
                self.open(port)
            if self.connecting:
                return eventtime + self.CONNECT_POLL_TIME
            return eventtime + self.retry_time(eventtime)
        if self.connecting:
            if self._connect_pending(eventtime):
                return eventtime + self.CONNECT_POLL_TIME
            if self.fd is None:
                return eventtime + self.retry_time(eventtime)
            return eventtime + self.PING_TIME
        if eventtime - self._last_rx > self.RX_TIMEOUT:
            self.close("no reply for 3 s")
            return eventtime + self.retry_time(eventtime)
        if self.connected:
            self.send("ping")
        return eventtime + self.PING_TIME

    def retry_time(self, eventtime: float) -> float:
        """
        How long until the next look for the board.

        :param eventtime: Reactor time
        :return float: FAST_RETRY_TIME for a while after a drop, else RETRY_TIME
        """
        if (self._dropped_at is not None
                and eventtime - self._dropped_at < self.FAST_RETRY_WINDOW):
            return self.FAST_RETRY_TIME
        return self.RETRY_TIME

    def open(self, port: str) -> bool:
        """
        Opens the tty raw, non-blocking and exclusive (TIOCEXCL, so other
        programs that probe serial ports can't steal replies), then starts
        the handshake.

        :param port: tty path, or tcp://<ip>:<port>
        :return bool: True when the port opened (or the connect started)
        """
        if port.startswith(TCP_SCHEME):
            return self._open_tcp(port)
        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            self._warn_once(f"{self.unit.name}: cannot open {port}: {e}")
            return False
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            # Any speed but 1200, which asks the board into its bootloader.
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except (OSError, termios.error) as e:
            os.close(fd)
            msg = f"{self.unit.name}: cannot set up {port}: {e}"
            self.logger.warning(msg)
            self.last_error = msg
            return False
        self._attach(fd, port)
        self.reactor.register_callback(self._handshake)
        return True

    def _open_tcp(self, port: str) -> bool:
        """
        Starts a non-blocking connect to a raw TCP serial server (ser2net in
        raw mode, or a USB host box that serves the board's bytes as they
        are); the timer finishes it and starts the handshake. Raw TCP
        carries no baud rate, so the server opens the board at its own
        speed, which must never be 1200 (the bootloader).

        :param port: tcp://<ip>:<port>
        :return bool: True when the connect started
        """
        host, number = parse_tcp(port)
        family = socket.AF_INET6 if ":" in host else socket.AF_INET
        try:
            sock = socket.socket(family, socket.SOCK_STREAM)
        except OSError as e:
            self._warn_once(f"{self.unit.name}: cannot open {port}: {e}")
            return False
        sock.setblocking(False)
        # Short JSON lines: send each one now, not when a segment fills.
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        err = sock.connect_ex((host, number))
        if err not in (0, errno.EINPROGRESS):
            sock.close()
            self._warn_once(f"{self.unit.name}: cannot connect to {port}: "
                            f"{os.strerror(err)}")
            return False
        self.sock = sock
        self.connecting = True
        self._attach(sock.fileno(), port)
        return True

    def _connect_pending(self, eventtime: float) -> bool:
        """
        Checks a TCP connect in progress. A finished one starts the
        handshake; a failed or timed out one closes the link.

        :param eventtime: Reactor time
        :return bool: True while the connect is still in progress
        """
        if not select.select([], [self.fd], [], 0)[1]:
            if eventtime - self._last_rx > self.RX_TIMEOUT:
                self.close("connect timed out")
                return False
            return True
        err = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if err:
            self.close(f"cannot connect: {os.strerror(err)}")
            return False
        self.connecting = False
        self._last_rx = eventtime
        self.reactor.register_callback(self._handshake)
        return False

    def _attach(self, fd: int, port: str) -> None:
        """
        Takes an opened port into use and starts reading it.

        :param fd: File descriptor of the tty or socket
        :param port: What was opened, for the log
        """
        self.fd = fd
        self.port = port
        self._rx = self._tx = b""
        self._last_rx = self.reactor.monotonic()
        self.fd_handle = self.reactor.register_fd(fd, self._read_cb)

    def _warn_once(self, msg: str) -> None:
        """
        Logs a warning unless it is the last one logged, so a board that
        stays away doesn't fill the log.

        :param msg: The warning
        """
        if msg != self.last_error:
            self.logger.warning(msg)
            self.last_error = msg

    def _handshake(self, eventtime: float) -> None:
        """
        Ends any half line in the board's buffer, asks for info and checks
        the protocol, then lets the unit push its settings.

        :param eventtime: Reactor time
        """
        self._write(b"\n")
        reply = self.request("info", timeout=2.0)
        if not reply.get("ok"):
            self.close(f"no answer to info ({reply.get('err')})")
            return
        if reply.get("proto") != PROTO:
            msg = (f"{self.unit.name}: board at {self.port} speaks "
                   f"{reply.get('proto')}, not {PROTO}; flash the matching firmware")
            self.close(msg)
            return
        self.info = reply
        self.connected = True
        self.last_error = ""
        if not self.unit.handle_link_up(reply):
            self.close("settings were refused")

    def close(self, reason: str, notify: bool = True) -> None:
        """
        Closes the port and fails everything still waiting on the board.

        :param reason: Why, for the log
        :param notify: False to skip telling the unit (shutdown)
        """
        if self.fd is None:
            return
        was_connected = self.connected
        if notify and reason != self.last_error:
            self.logger.warning(f"{self.unit.name}: closed {self.port}: {reason}")
        self.last_error = reason
        if self.fd_handle is not None:
            self.reactor.unregister_fd(self.fd_handle)
            self.fd_handle = None
        try:
            if self.sock is not None:
                self.sock.close()
            else:
                os.close(self.fd)
        except OSError:
            pass
        self.fd = None
        self.sock = None
        self.connecting = False
        self.connected = False
        for completion in self._pending.values():
            completion.complete({"ok": False, "err": "disconnected"})
        self._pending = {}
        waiters, self._waiters = self._waiters, []
        for _, completion in waiters:
            completion.complete({"ev": "link_down"})
        if notify and was_connected:
            # Look for the board again at once.
            self._dropped_at = self.reactor.monotonic()
            self.reactor.update_timer(self.timer, self.reactor.NOW)
        if notify:
            self.unit.handle_link_down(reason, was_connected)

    def _read_cb(self, eventtime: float) -> None:
        """
        Reactor fd handler; never blocks. A raw tty with VMIN 0 reads b""
        when it has nothing, so only a read after select says ready means
        the board is gone (pyserial does the same). A socket that turns
        readable while connecting has finished its connect, one way or the
        other.

        :param eventtime: Reactor time
        """
        if self.connecting:
            self._connect_pending(eventtime)
            return
        try:
            if not select.select([self.fd], [], [], 0)[0]:
                return
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return
        except OSError as e:
            self.close(f"read failed: {e}")
            return
        if not data:
            self.close("port closed")
            return
        self._last_rx = eventtime
        self._rx += data
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            self._handle_line(line.strip(), eventtime)
        if len(self._rx) > 4096:
            self._rx = b""
        if self._tx:
            self._write(b"")

    def _handle_line(self, line: bytes, eventtime: float) -> None:
        """
        Routes one line from the board: replies to their request, events to
        any waiter and then to the unit.

        :param line: One line without its newline
        :param eventtime: Reactor time
        """
        if not line.startswith(b"{"):
            return
        try:
            msg = json.loads(line)
        except ValueError:
            return
        if "ev" in msg:
            for waiter in list(self._waiters):
                predicate, completion = waiter
                if predicate(msg):
                    self._waiters.remove(waiter)
                    completion.complete(msg)
            self.unit.handle_event(msg, eventtime)
            return
        completion = self._pending.pop(msg.get("id"), None)
        if completion is not None:
            completion.complete(msg)

    def _write(self, data: bytes) -> None:
        """
        Writes to the tty, keeping what the kernel didn't take for later.

        :param data: Bytes to add to the outgoing buffer
        """
        if self.fd is None:
            return
        self._tx += data
        try:
            while self._tx:
                count = os.write(self.fd, self._tx)
                self._tx = self._tx[count:]
        except BlockingIOError:
            pass
        except OSError as e:
            self.close(f"write failed: {e}")

    def send(self, cmd: str, **fields) -> None:
        """
        Sends a request without waiting for its reply.

        :param cmd: Firmware command
        :param fields: Command fields
        """
        if self.fd is None:
            return
        rid = self._next_id
        self._next_id += 1
        line = encode_line({"id": rid, "cmd": cmd, **fields})
        if self._too_long(cmd, line):
            return
        self._write(line)

    def _too_long(self, cmd: str, line: bytes) -> bool:
        """
        The board drops a line over LINE_MAX, so it is never sent.

        :param cmd: Firmware command
        :param line: The encoded line
        :return bool: True when the line is too long (logged)
        """
        if len(line) <= LINE_MAX:
            return False
        self.logger.error(f"{self.unit.name}: {cmd} line is {len(line)} bytes, "
                          f"over the board's {LINE_MAX}; not sent")
        return True

    def request(self, cmd: str, timeout: float = 1.0, **fields) -> Dict[str, Any]:
        """
        Sends a request and waits for its reply. Only call from a context
        that can wait (gcode or a reactor callback), never from the fd handler.

        :param cmd: Firmware command
        :param timeout: Seconds to wait for the reply
        :param fields: Command fields
        :return dict: The reply, or {"ok": False, "err": ...}
        """
        if self.fd is None:
            return {"ok": False, "err": "not connected"}
        rid = self._next_id
        self._next_id += 1
        line = encode_line({"id": rid, "cmd": cmd, **fields})
        if self._too_long(cmd, line):
            return {"ok": False, "err": "line too long"}
        completion = self.reactor.completion()
        self._pending[rid] = completion
        self._write(line)
        reply = completion.wait(self.reactor.monotonic() + timeout, None)
        self._pending.pop(rid, None)
        if reply is None:
            return {"ok": False, "err": "timeout"}
        return reply

    def wait_up(self, timeout: float) -> bool:
        """
        Waits for the link to be up (handshake done), such as after a drop.

        :param timeout: Seconds to wait
        :return bool: True when connected
        """
        end = self.reactor.monotonic() + timeout
        while not self.connected:
            now = self.reactor.monotonic()
            if now >= end:
                return False
            self.reactor.pause(now + 0.05)
        return True

    def expect(self, predicate: Callable[[Dict[str, Any]], bool]):
        """
        Registers a wait for an event before the request that causes it is
        sent, so a fast event can't slip past.

        :param predicate: Returns True for the event to wait for
        :return: Completion that gets the event, or {"ev": "link_down"}
        """
        completion = self.reactor.completion()
        if self.fd is None:
            completion.complete({"ev": "link_down"})
        else:
            self._waiters.append((predicate, completion))
        return completion

    def wait(self, completion, timeout: float) -> Optional[Dict[str, Any]]:
        """
        Waits for an event registered with expect.

        :param completion: From expect
        :param timeout: Seconds to wait
        :return dict: The event, or None on timeout
        """
        msg = completion.wait(self.reactor.monotonic() + timeout, None)
        if msg is None:
            self._waiters = [w for w in self._waiters if w[1] is not completion]
        return msg


class TurtleNeck:
    """
    Stands in for an AFC_buffer: the TurtleNeck switches are read by the
    board, and the board runs the feed assist and its clog checks. Enabling
    the buffer switches the lane's assist to TurtleNeck mode. The state reads
    like AFC_buffer's: it holds until the other switch changes, with no
    Neutral.
    """

    def __init__(self, unit: "afcBoxTurtleUSB"):
        """
        :param unit: The unit whose board reads the TurtleNeck
        """
        self.unit = unit
        self.name = f"{unit.name}_TurtleNeck"
        self.type = "TurtleNeck"
        self.lanes: Dict[str, Any] = {}
        self.advance_pin = None
        self.advance_state = False
        self.trailing_state = False
        self.last_state = "Unknown"
        self.enable = False
        self.current_lane = None
        self.fault_detection = True

    def enable_buffer(self, lane=None) -> None:
        """
        Called by the lane once it is loaded to the toolhead.

        :param lane: Lane now feeding the toolhead
        """
        if lane is None:
            lane = self.current_lane
        self.enable = True
        self.current_lane = lane
        if lane is not None:
            self.unit.buffer_changed(lane, True)

    def disable_buffer(self) -> None:
        """
        Called by the lane before an unload.
        """
        lane = self.current_lane
        self.enable = False
        if lane is not None:
            self.unit.buffer_changed(lane, False)

    def set_switch(self, name: str, value: bool) -> None:
        """
        Copies one board switch event. Like AFC_buffer, the state names where
        the buffer heads next: any change of the advance switch gives
        Trailing (feeding less, so it compresses), any change of the trailing
        switch gives Advancing.

        :param name: "tn_adv" or "tn_trl"
        :param value: True while the switch is pressed
        """
        if name == "tn_adv":
            self.advance_state = bool(value)
            self.last_state = "Trailing"
        else:
            self.trailing_state = bool(value)
            self.last_state = "Advancing"

    def sync_switches(self, advance: bool, trailing: bool) -> None:
        """
        Copies both switches from a board status (connect, end of a move). A
        pressed switch sets the state as its event would; with neither
        pressed the last state holds.

        :param advance: True while the advance switch is pressed
        :param trailing: True while the trailing switch is pressed
        """
        self.advance_state = bool(advance)
        self.trailing_state = bool(trailing)
        if self.advance_state:
            self.last_state = "Trailing"
        elif self.trailing_state:
            self.last_state = "Advancing"

    def buffer_status(self) -> str:
        """
        :return str: The TurtleNeck state like AFC_buffer reports it
        """
        return self.last_state

    def disable_fault_sensitivity(self) -> None:
        """
        Turns the board's clog check off (first layers after a load).
        """
        self.fault_detection = False
        self.unit.send_fault_mm(0.0)

    def restore_fault_sensitivity(self) -> None:
        """
        Turns the board's clog check back on.
        """
        self.fault_detection = True
        self.unit.send_fault_mm(self.unit.assist_fault_mm)

    def update_filament_error_pos(self, *args, **kwargs) -> None:
        """
        The board measures the clog distance itself.
        """
        return

    def enable_advance_latch(self) -> None:
        """
        Not used: the board reads the switch directly.
        """
        return

    def clear_advance_latch(self) -> None:
        """
        Not used: the board reads the switch directly.
        """
        return

    def get_status(self, eventtime=None) -> Dict[str, Any]:
        """
        :return dict: TurtleNeck state for the unit's status
        """
        return {"state": self.buffer_status(),
                "advance": self.advance_state,
                "trailing": self.trailing_state,
                "enabled": self.enable,
                "lane": self.current_lane.name if self.current_lane else None,
                "fault_detection": self.fault_detection}


class TurtleRotation:
    """
    What an AFC_buffer reads from a lane's extruder_stepper: the rotation
    distance with its multiplier applied. The board does the scaling.
    """

    def __init__(self, drive: "TurtleDrive"):
        """
        :param drive: The lane's TurtleDrive
        """
        self.drive = drive
        self.stepper = self

    def get_rotation_distance(self) -> tuple:
        """
        :return tuple: (rotation distance / multiplier, steps per rotation)
        """
        unit = self.drive.unit
        s = unit.lane_settings.get(self.drive.name, {})
        base = s.get('rotation_distance', unit.rotation_distance)
        steps = s.get('microsteps', unit.microsteps) * unit.full_steps
        multiplier = self.drive.multiplier or 1.0
        return base / multiplier, steps


class TurtleDrive:
    """
    Stands in for a lane's Klipper drive stepper. AFC_lane calls move,
    home_to, sync_to_extruder and the rest on it; each goes to the board.
    """

    def __init__(self, unit: "afcBoxTurtleUSB", lane: AFCLane):
        """
        :param unit: The unit whose board drives this lane
        :param lane: The lane
        """
        self.unit = unit
        self.lane = lane
        self.name = lane.name
        self.extruder_stepper = None
        # AFC's software tool_start endstop has no stepper to attach to.
        self._endstops: Dict[str, Any] = {"tool_start": None}
        self.assist_move = lane.assist_move
        self.multiplier = 1.0
        self.rotation = TurtleRotation(self)

    def move(self, distance: float, speed: float, accel: float,
             assist_active: bool = False) -> None:
        """
        A plain lane move.

        :param distance: mm, + toward the toolhead
        :param speed: mm/s
        :param accel: mm/s^2
        :param assist_active: Run the espooler like AFC does on long moves
        """
        self.unit.fw_move(self.lane, distance, speed, accel, assist_active)

    def home_to(self, endstop_spec, distance: float, speed: float, accel: float,
                triggered: bool = True, check_trigger: bool = True,
                assist_active: bool = True) -> tuple:
        """
        Moves until a switch reaches a state, like AFC_stepper.home_to.

        :param endstop_spec: AFC endstop name (load, hub, buffer, tool...)
        :param distance: Most to move, mm
        :param speed: mm/s
        :param accel: mm/s^2
        :param triggered: Stop when the switch triggers (False: when it clears)
        :param check_trigger: A move that never reaches the state fails
        :param assist_active: Run the espooler
        :return tuple: (homed, distance moved, error)
        """
        return self.unit.fw_home(self.lane, endstop_spec, distance, speed, accel,
                                 triggered, check_trigger, assist_active)

    def do_enable(self, enable: bool) -> None:
        """
        :param enable: Driver on or off (off is ignored while the lane runs)
        """
        self.unit.link.send("enable", lane=self.lane.index, on=bool(enable))

    def sync_to_extruder(self, update_current: bool = True,
                         th_extruder_name: Optional[str] = None) -> None:
        """
        The lane follows the extruder: TurtleNeck assist when the buffer is
        on, plain position following otherwise.

        :param update_current: Switch to print current
        :param th_extruder_name: Unused, the lane's extruder is used
        """
        if update_current:
            self.set_print_current()
        self.unit.set_sync(self.lane, True)

    def unsync_to_extruder(self, update_current: bool = True) -> None:
        """
        :param update_current: Switch back to load current
        """
        self.unit.set_sync(self.lane, False)
        if update_current:
            self.set_load_current()

    def set_load_current(self) -> None:
        """
        Load current is run_current.
        """
        self.unit.set_current(self.lane, False)

    def set_print_current(self) -> None:
        """
        Print current is print_current, when set.
        """
        self.unit.set_current(self.lane, True)

    def update_rotation_distance(self, multiplier: float) -> None:
        """
        An external buffer's multiplier (>1 feeds more). The position sent
        to the board in ext mode is scaled by it; the TurtleNeck's own
        multiplier is applied on the board.

        :param multiplier: Rotation distance multiplier
        """
        self.multiplier = float(multiplier) if multiplier else 1.0


class afcBoxTurtleUSB(afcBoxTurtle):
    """
    BoxTurtle whose AFC-Lite runs boxturtle_fw over USB.
    """
    E_STREAM_TIME = 0.1
    CONNECT_WAIT = 8.0
    # A USB drop: how long a move, job or print waits for the board to come
    # back, and how many drops one move or job gets through.
    RECONNECT_WAIT = 5.0
    DROP_TRIES = 3

    def __init__(self, config):
        super().__init__(config)
        # Fluidd and Mainsail find a unit as AFC_<type without underscores>.
        self.type = config.get('type', 'BoxTurtleUSB')
        self.serial_port = config.get('serial_port', 'auto')
        if self.serial_port.startswith(TCP_SCHEME):
            try:
                parse_tcp(self.serial_port)
            except ValueError as e:
                raise error(str(e))
        self.serial_uid = config.get('serial_uid', None)
        self.link_timeout = config.getfloat('link_timeout', 2.0, minval=0.5)

        # Lane drivers (each lane's [AFC_lane] section can override these)
        self.rotation_distance = config.getfloat('rotation_distance', 4.65, above=0.)
        self.microsteps = config.getint('microsteps', 16, minval=1, maxval=256)
        self.full_steps = config.getint('full_steps_per_rotation', 200, minval=1)
        self.invert_dir = config.getboolean('invert_dir', True)
        self.run_current = config.getfloat('run_current', 0.8, above=0., maxval=2.)
        self.hold_current = config.getfloat('hold_current', None, above=0., maxval=2.)
        self.print_current = config.getfloat('print_current', None, above=0., maxval=2.)
        self.sense_resistor = config.getfloat('sense_resistor', 0.110, above=0.)
        self.stealthchop = config.getboolean('stealthchop', False)
        self.driver_sgthrs = config.getint('driver_sgthrs', 0, minval=0, maxval=255)

        # TurtleNeck on the board (AFC_buffer's TurtleNeck options). With
        # turtleneck False and no buffer here, lanes use their extruder's.
        self.use_turtleneck = config.getboolean('turtleneck', True)
        self.multiplier_high = config.getfloat('multiplier_high', 1.15, minval=1.)
        self.multiplier_low = config.getfloat('multiplier_low', 0.85, minval=0., maxval=1.)
        sensitivity = config.getfloat('filament_error_sensitivity', 3.0, minval=0., maxval=10.)
        self.assist_fault_mm = 0.0 if sensitivity == 0 else (11 - sensitivity) * 10
        self.assist_fallback_speed = config.getfloat('assist_fallback_speed', 30., above=0.)
        self.assist_max_speed = config.getfloat('assist_max_speed', 80., above=0.)

        # Self-recovering jobs on the board
        self.job_tries = config.getint('job_tries', 2, minval=0, maxval=10)
        self.job_backoff_mm = config.getfloat('job_backoff_mm', 20., minval=0.)
        self.job_snag_backoff_mm = config.getfloat('job_snag_backoff_mm', 30., minval=0.)
        self.job_repush_mm = config.getfloat('job_repush_mm', 10., minval=0.)

        # Espooler PWM cycle (AFC's cycle_time for the motor pins)
        self.espool_cycle_time = config.getfloat('espool_cycle_time', 0.004, above=0.)

        # Switches: None keeps the board's default (printer 1's wiring)
        self.switch_invert = {}
        for key, names in (('prep_invert', ['prep1', 'prep2', 'prep3', 'prep4']),
                           ('load_invert', ['load1', 'load2', 'load3', 'load4']),
                           ('hub_invert', ['hub']),
                           ('tn_advance_invert', ['tn_adv']),
                           ('tn_trailing_invert', ['tn_trl'])):
            value = config.getboolean(key, None)
            if value is not None:
                for name in names:
                    self.switch_invert[name] = value
        self.switch_debounce_ms = config.getint('switch_debounce_ms', None, minval=0, maxval=1000)

        # LEDs on the board's RGB1-RGB4 headers. The defaults are the stock
        # BoxTurtle wiring: one GRBW LED per lane on RGB1.
        self.led_chains: Dict[int, tuple] = {}
        for header in range(1, LED_HEADERS + 1):
            count = config.getint(f'rgb{header}_chain_count', 4 if header == 1 else 0,
                                  minval=0, maxval=96)
            order = config.get(f'rgb{header}_color_order',
                               'GRBW' if header == 1 else 'GRB').upper()
            if order not in LED_ORDERS:
                error_string = (f"rgb{header}_color_order {order} is not one of "
                                f"{', '.join(LED_ORDERS)}")
                raise error(error_string)
            self.led_chains[header] = (count, order)

        self.lane_settings = self._read_lane_settings(config)
        self.link = TurtleLink(self, self.serial_port, self.serial_uid)
        self.sync = {}       # lane name -> synced to its extruder
        self.buffer_on = {}  # lane name -> TurtleNeck buffer enabled
        self.assist_mode = {}  # lane name -> mode last sent (tn, follow, ext)
        self.e_last = {}     # lane name -> last extruder position read
        self.e_sent = {}     # lane name -> last position sent (scaled in ext)
        self.switches: Dict[str, int] = {}
        self.led_colors: Dict[str, Any] = {}
        self.ever_connected = False
        self.mcu = None
        self._pulse_tick = 0
        self._seq = 0
        self.moving = set()  # lanes running a move or job for AFC
        self.e_timer = self.reactor.register_timer(self._e_stream_cb)

        self.printer.register_event_handler("klippy:shutdown", self.link.stop)
        self.printer.register_event_handler("klippy:disconnect", self.link.stop)
        self.gcode.register_mux_command('AFC_TURTLE_STATUS', 'UNIT', self.name,
                                        self.cmd_AFC_TURTLE_STATUS,
                                        desc=self.cmd_AFC_TURTLE_STATUS_help)
        self.gcode.register_mux_command('AFC_TURTLE_SEND', 'UNIT', self.name,
                                        self.cmd_AFC_TURTLE_SEND,
                                        desc=self.cmd_AFC_TURTLE_SEND_help)
        self.gcode.register_mux_command('AFC_TURTLE_BOOTLOADER', 'UNIT', self.name,
                                        self.cmd_AFC_TURTLE_BOOTLOADER,
                                        desc=self.cmd_AFC_TURTLE_BOOTLOADER_help)

    def _read_lane_settings(self, config) -> Dict[str, Dict[str, Any]]:
        """
        Reads the driver and LED options from each [AFC_lane] section that
        belongs to this unit, with the unit's values as defaults.

        :param config: This unit's config section
        :return dict: lane name -> settings
        """
        settings = {}
        for section in config.get_prefix_sections('AFC_lane '):
            unit = section.get('unit', '')
            if unit.split(':')[0] != self.name:
                continue
            name = section.get_name().split()[-1]
            settings[name] = {
                'rotation_distance': section.getfloat('rotation_distance', self.rotation_distance, above=0.),
                'microsteps': section.getint('microsteps', self.microsteps, minval=1, maxval=256),
                'invert_dir': section.getboolean('invert_dir', self.invert_dir),
                'run_current': section.getfloat('run_current', self.run_current, above=0., maxval=2.),
                'hold_current': section.getfloat('hold_current', self.hold_current, above=0., maxval=2.),
                'print_current': section.getfloat('print_current', self.print_current, above=0., maxval=2.),
                'turtle_led': self._lane_leds(name, unit, section.get('turtle_led', None)),
            }
        return settings

    def _lane_leds(self, name: str, unit: str, spec: Optional[str]) -> list:
        """
        A lane's LED ranges. Without turtle_led, lane N gets LED N on RGB1
        (the stock wiring) when RGB1 has that many.

        :param name: Lane name, for errors
        :param unit: The lane's unit option (<unit>:<lane number>)
        :param spec: Its turtle_led, or None
        :return list: (header, first, last) for each range
        """
        counts = {header: chain[0] for header, chain in self.led_chains.items()}
        if spec is None:
            index = unit.partition(':')[2]
            if index.isdigit() and 1 <= int(index) <= counts[1]:
                return [(1, int(index), int(index))]
            return []
        try:
            return parse_turtle_led(spec, counts)
        except ValueError as e:
            error_string = f"{name}: {e}"
            raise error(error_string)

    def handle_connect(self):
        """
        Hands the lanes a TurtleNeck (unless a buffer is named) and a
        TurtleDrive each, then starts looking for the board. The TurtleNeck
        is also a printer object so the UIs can show its state.
        """
        if self.buffer_name is None and self.use_turtleneck:
            self.buffer_obj = TurtleNeck(self)
            # Fluidd and Mainsail read a lane's buffer as "AFC_buffer <name>".
            key = f"AFC_buffer {self.buffer_obj.name}"
            if self.printer.lookup_object(key, None) is None:
                self.printer.add_object(key, self.buffer_obj)
        self.mcu = self.printer.lookup_object('mcu')
        super().handle_connect()
        for lane in self.lanes.values():
            drive = TurtleDrive(self, lane)
            lane.drive_stepper = drive
            # AFC_buffer applies its multiplier through extruder_stepper.
            lane.extruder_stepper = drive.rotation
            # No load pin: AFC_lane starts it True; the board says otherwise.
            lane._load_state = False
            self.sync[lane.name] = False
            self.buffer_on[lane.name] = False
        self.link.start()

    def lane_by_index(self, index: int) -> Optional[AFCLane]:
        """
        :param index: Board lane number, 1-4
        :return AFCLane: The lane on that drive, or None
        """
        for lane in self.lanes.values():
            if lane.index == index:
                return lane
        return None

    def _lane_cfg(self, lane: AFCLane) -> Dict[str, Any]:
        """
        :param lane: Lane to describe
        :return dict: lane_cfg fields for this lane
        """
        s = self.lane_settings.get(lane.name, {})
        rot = s.get('rotation_distance', self.rotation_distance)
        micro = s.get('microsteps', self.microsteps)
        run = s.get('run_current', self.run_current)
        hold = s.get('hold_current') or run
        delta_mm, pulse_ms = self.pulse_settings(lane)
        return {"lane": lane.index,
                "steps_per_mm": float(self.full_steps * micro / rot),
                "dir_invert": bool(s.get('invert_dir', self.invert_dir)),
                "run_current": float(run),
                "hold_current": float(hold),
                "sense_resistor": float(self.sense_resistor),
                "microsteps": int(micro),
                "stealthchop": bool(self.stealthchop),
                "sgthrs": int(self.driver_sgthrs),
                "espool_pulse_mm": float(delta_mm),
                "espool_pulse_ms": int(pulse_ms)}

    def pulse_settings(self, lane: AFCLane) -> tuple:
        """
        AFC's print assist pulse for this lane's spool: every delta_movement
        mm, kick_start_time plus the cruise time AFC works out from the
        weight; none at or over enable_assist_weight.

        :param lane: Lane to work out
        :return tuple: (pulse distance mm, pulse length ms)
        """
        espooler = lane.espooler
        values = espooler.espooler_values
        try:
            delta = float(values.delta_movement)
        except (TypeError, AttributeError):
            return 0.0, 0
        if (not espooler.enable_assist
                or espooler.enable_assist_weight is None
                or lane.weight >= espooler.enable_assist_weight):
            return delta, 0
        try:
            cruise = values.calculate_cruise_time(lane.weight)
            pulse_ms = int(round((values.kick_start_time + cruise) * 1000))
        except (TypeError, ZeroDivisionError):
            return delta, 0
        return delta, max(0, min(pulse_ms, 5000))

    def handle_link_up(self, info: Dict[str, Any]) -> bool:
        """
        The board answered: sends every setting, then reads the switches.

        :param info: The board's info reply
        :return bool: False if the board refused a setting
        """
        link = self.link
        self.logger.info(f"{self.name}: BoxTurtle firmware {info.get('fw')} "
                         f"on {link.port} (reset {info.get('reset')})")
        kick = float(self.kick_start_time * self.scaling)
        cfg = {"link_timeout_ms": int(self.link_timeout * 1000),
               "assist_high": float(self.multiplier_high),
               "assist_low": float(self.multiplier_low),
               "assist_fault_mm": float(self.assist_fault_mm),
               "assist_fallback_speed": float(self.assist_fallback_speed),
               "assist_max_speed": float(self.assist_max_speed),
               "espool_kick_ms": int(round(kick * 1000)),
               "espool_brake_ms": int(round(min(self.n20_break_delay_time, 1.0) * 1000)),
               "espool_cycle_us": int(round(self.espool_cycle_time * 1e6)),
               "job_tries": int(self.job_tries),
               "job_backoff_mm": float(self.job_backoff_mm),
               "job_snag_backoff_mm": float(self.job_snag_backoff_mm),
               "job_repush_mm": float(self.job_repush_mm)}
        requests = [("cfg", part) for part in split_fields("cfg", cfg)]
        for lane in self.lanes.values():
            requests += [("lane_cfg", part) for part in
                         split_fields("lane_cfg", self._lane_cfg(lane), keep=("lane",))]
        for name, invert in self.switch_invert.items():
            fields = {"sw": name, "invert": invert}
            if self.switch_debounce_ms is not None:
                fields["debounce_ms"] = self.switch_debounce_ms
            requests.append(("sw_cfg", fields))
        for header, (count, order) in self.led_chains.items():
            # A header with no LEDs is sent too, so one a restart turned off goes dark.
            requests.append(("led_cfg", {"strip": header, "count": count, "order": order}))
        bad = []
        for cmd, fields in requests:
            reply = link.request(cmd, **fields)
            if not reply.get("ok"):
                bad.append(f"{cmd} ({reply.get('err')})")
        if bad:
            self.logger.error(f"{self.name}: board refused settings: {', '.join(bad)}")
            return False
        status = link.request("status")
        if not status.get("ok"):
            return False
        self.switches = dict(status.get("sw", {}))
        self._apply_switches()
        first = not self.ever_connected
        self.ever_connected = True
        if not first:
            self._resume_after_reconnect()
        return True

    def _apply_switches(self) -> None:
        """
        Copies the board's switch states into AFC without running AFC's
        insert or runout handling (used at connect).
        """
        sw = self.switches
        for lane in self.lanes.values():
            lane.prep_state = bool(sw.get(f"prep{lane.index}", 0))
            lane._load_state = bool(sw.get(f"load{lane.index}", 0))
        hub = self._hub()
        if hub is not None and hasattr(hub, 'switch_pin_callback'):
            hub.switch_pin_callback(self.reactor.monotonic(), int(sw.get("hub", 0)))
        if isinstance(self.buffer_obj, TurtleNeck):
            self.buffer_obj.sync_switches(sw.get("tn_adv", 0), sw.get("tn_trl", 0))

    def _hub(self):
        """
        :return: This unit's hub object, or None
        """
        if self.hub_obj is not None:
            return self.hub_obj
        for lane in self.lanes.values():
            hub = getattr(lane, 'hub_obj', None)
            if hub is not None and hasattr(hub, 'switch_pin_callback'):
                return hub
        return None

    def _resume_after_reconnect(self) -> None:
        """
        After a replug the board has stopped every motor: turns the assist
        back on for synced lanes and refreshes the LEDs.
        """
        for lane in self.lanes.values():
            # A lane in a move or job gets its assist back when that ends.
            if self.sync.get(lane.name) and lane.name not in self.moving:
                self._send_assist(lane)
            if lane.name in self.led_colors:
                self.board_led(lane, self.led_colors[lane.name])
        self._update_e_timer()

    def handle_link_down(self, reason: str, was_connected: bool) -> None:
        """
        The board went away. Pauses a print that was feeding from it.

        :param reason: Why the link closed
        :param was_connected: The board had answered before
        """
        if not was_connected:
            return
        self.reactor.register_callback(lambda et, r=reason: self._link_lost(r),
                                       self.reactor.monotonic() + self.RECONNECT_WAIT)

    def _link_lost(self, reason: str) -> None:
        """
        RECONNECT_WAIT after a drop: pauses a print that was feeding from
        this unit if the board is still gone (runs outside the fd handler,
        since a pause runs gcode).

        :param reason: Why the link closed
        """
        if self.link.connected:
            self.logger.info(f"{self.name}: BoxTurtle USB link dropped ({reason}) "
                             f"and came back")
            return
        msg = f"{self.name}: BoxTurtle USB link lost ({reason})"
        current = self.afc.current
        if (current in self.lanes
                and self.afc.function.is_printing()):
            self.afc.error.AFC_error(f"{msg}, pausing. Plug it back in and resume.",
                                     pause=True)

    # ── board events ──────────────────────────────────────────────

    def handle_event(self, msg: Dict[str, Any], eventtime: float) -> None:
        """
        Called from the fd handler for every event; the work runs in a
        reactor callback so it can wait.

        :param msg: The event
        :param eventtime: Reactor time
        """
        ev = msg.get("ev")
        if ev == "sw":
            name, value = msg.get("sw"), int(msg.get("v", 0))
            self.switches[name] = value
            self.reactor.register_callback(
                lambda et, n=name, v=value: self.switch_changed(n, v, et))
        elif ev == "fault":
            self.reactor.register_callback(
                lambda et, m=msg: self.fault(m, et))
        elif ev == "job" and msg.get("state") == "recovering":
            self.logger.info(f"{self.name} lane {msg.get('lane')}: {msg.get('job')} "
                             f"retry {msg.get('try')} ({msg.get('why')})")
        elif ev == "log":
            self.logger.info(f"{self.name}: {msg.get('msg')}")

    def switch_changed(self, name: str, value: int, eventtime: float) -> None:
        """
        A switch on the board changed: runs AFC's handling for it.

        :param name: Switch name (prep1, load2, hub, tn_adv...)
        :param value: 1 triggered, 0 clear
        :param eventtime: Reactor time
        """
        if name.startswith("prep") or name.startswith("load"):
            lane = self.lane_by_index(int(name[4:]))
            if lane is None:
                return
            if name.startswith("load"):
                lane.load_callback(eventtime, bool(value))
            elif value:
                lane.prep_callback(eventtime, 1)
            else:
                self.prep_removed(lane, eventtime)
        elif name == "hub":
            hub = self._hub()
            if hub is None:
                return
            hub.switch_pin_callback(eventtime, value)
            current = self.afc.lanes.get(self.afc.current) if self.afc.current else None
            if (not value
                    and current is not None
                    and current.name in self.lanes
                    and getattr(hub, 'enable_runout', False)):
                current.handle_hub_runout(sensor=hub.name)
        elif name in ("tn_adv", "tn_trl") and isinstance(self.buffer_obj, TurtleNeck):
            self.buffer_obj.set_switch(name, value)

    def prep_removed(self, lane: AFCLane, eventtime: float) -> None:
        """
        A lane's prep switch opened. Mid print on the lane in use this is a
        runout (AFC_lane.handle_prep_runout's own branch needs a Klipper
        filament sensor, which this lane doesn't have); otherwise it is AFC's
        normal removal handling.

        :param lane: The lane
        :param eventtime: Reactor time
        """
        lane.prep_state = False
        if (self.printer.state_message == 'Printer is ready'
                and lane._afc_prep_done
                and lane.status != AFCLaneState.TOOL_UNLOADING
                and lane.name == self.afc.current
                and self.afc.function.is_printing()
                and lane.raw_load_state
                and lane.status != AFCLaneState.EJECTING):
            if lane.runout_lane is not None:
                lane._perform_infinite_runout()
            else:
                lane._perform_pause_runout()
            return
        lane.handle_prep_runout(eventtime, False)

    def fault(self, msg: Dict[str, Any], eventtime: float) -> None:
        """
        A fault the board found by itself.

        :param msg: The fault event (code, lane, msg)
        :param eventtime: Reactor time
        """
        code = msg.get("code")
        lane = self.lane_by_index(int(msg.get("lane", 0) or 0))
        text = f"{self.name}: {msg.get('msg')} ({code})"
        if lane is not None:
            text = f"{lane.name}: {msg.get('msg')} ({code})"
        printing_on_lane = (lane is not None
                            and lane.name == self.afc.current
                            and self.afc.function.is_printing())
        if code in ASSIST_FAULTS:
            if printing_on_lane:
                self.afc.error.AFC_error(text, pause=True)
            else:
                self.logger.warning(text)
            if lane is not None and self.sync.get(lane.name):
                # The board stopped the assist; it starts again on resume
                # when the extruder moves.
                self._send_assist(lane)
        elif code in TMC_FAULTS:
            self.afc.error.AFC_error(text, pause=printing_on_lane)
        elif code == "link_lost":
            self.logger.warning(text)
        elif code in ("tmc_otpw", "tmc_open", "tmc_ok"):
            self.logger.info(text)

    # ── moves ──────────────────────────────────────────────────────

    def _espool_fields(self, lane: AFCLane, speed: float, assist: bool) -> Dict[str, Any]:
        """
        Espooler duty for one move, sized like AFC_lane.assist_move.

        :param lane: Lane that moves
        :param speed: Move speed, mm/s
        :param assist: AFC would run the espooler on this move
        :return dict: espool_fwd, espool_rwd, espool_kick fields
        """
        if not assist:
            return {"espool_fwd": 0.0, "espool_rwd": 0.0, "espool_kick": False}
        try:
            fwd = min(1.0, float(lane.calculate_pwm_value(speed)))
            rwd = min(1.0, float(lane.calculate_pwm_value(speed, True)))
        except (TypeError, ZeroDivisionError, AttributeError):
            fwd = rwd = 0.5
        kick = bool(getattr(lane.espooler, 'enable_kick_start', True))
        return {"espool_fwd": fwd, "espool_rwd": rwd, "espool_kick": kick}

    def _report(self, lane: AFCLane, text: str) -> None:
        """
        Reports a move the board could not do.

        :param lane: Lane that was moving
        :param text: What went wrong
        """
        msg = f"{lane.name}: {text}"
        self.afc.error.AFC_error(msg, pause=self.afc.function.in_print())

    def fw_move(self, lane: AFCLane, distance: float, speed: float, accel: float,
                assist: bool, until: Optional[str] = None, until_v: int = 1) -> tuple:
        """
        Runs one move on the board and waits for it to end.

        :param lane: Lane to move
        :param distance: mm, + toward the toolhead
        :param speed: mm/s
        :param accel: mm/s^2
        :param assist: Run the espooler
        :param until: Board switch that ends the move early
        :param until_v: State of that switch that ends it
        :return tuple: (end reason, mm moved)
        """
        if not distance:
            return "done", 0.0
        speed = float(speed or lane.short_moves_speed)
        accel = float(accel or lane.short_moves_accel)
        fields = {"lane": lane.index, "mm": float(distance), "speed": speed,
                  "accel": accel, "hold": True}
        if until:
            fields["until"] = until
            fields["until_v"] = int(until_v)
        fields.update(self._espool_fields(lane, speed, assist))
        self.wait_toolhead()
        self.moving.add(lane.name)
        synced = self._pause_assist(lane)
        try:
            return self._run_move(lane, fields, distance, speed, accel)
        finally:
            self.moving.discard(lane.name)
            self._resume_assist(lane, synced)

    def wait_toolhead(self) -> None:
        """
        Waits for the moves already queued on the toolhead to finish. A lane
        on a Klipper stepper moves in the toolhead's timeline, after them;
        the board moves as soon as it is told, so AFC's own sequences (a
        tool_stn extrude, then a lane move) would otherwise overlap.
        """
        if self.printer.state_message != 'Printer is ready' or self.mcu is None:
            return
        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            return
        end = toolhead.get_last_move_time()
        while True:
            now = self.reactor.monotonic()
            left = end - self.mcu.estimated_print_time(now)
            if left <= 0:
                return
            self.reactor.pause(now + min(left, 0.1))

    def _next_seq(self) -> int:
        """
        :return int: A new id for a move or job, echoed in its end event
        """
        self._seq += 1
        return self._seq

    def _after_drop(self, lane: AFCLane, ev: str, seq: int, timeout: float,
                    stop: bool = False) -> Optional[Dict[str, Any]]:
        """
        The USB link dropped while the board ran a move or job for lane.
        Waits for the board to come back, then for that move's or job's end
        event; the board sends it again on replay in case it ended while
        the link was down.

        :param lane: The lane
        :param ev: move_done or job_done
        :param seq: The move's or job's id
        :param timeout: Seconds to wait for the end event
        :param stop: Stop the lane first (a move only the host can end)
        :return dict: The end event, or None if the board stayed gone or never ended it
        """
        for _ in range(self.DROP_TRIES):
            if not self.link.wait_up(self.RECONNECT_WAIT):
                return None
            done = self.link.expect(lambda m, n=lane.index: m.get("ev") == ev
                                    and m.get("lane") == n and m.get("seq") == seq)
            if stop:
                self.link.send("stop", lane=lane.index)
            self.link.request("replay", lane=lane.index)
            msg = self.link.wait(done, timeout)
            if msg is None or msg.get("ev") != "link_down":
                return msg
        return None

    def _pause_assist(self, lane: AFCLane) -> bool:
        """
        The board runs one thing per lane: a move takes over from the
        assist, which comes back after it.

        :param lane: Lane about to move
        :return bool: The lane was synced
        """
        synced = bool(self.sync.get(lane.name, False))
        if synced:
            self.link.request("assist", lane=lane.index, on=False)
        return synced

    def _resume_assist(self, lane: AFCLane, synced: bool) -> None:
        """
        Turns the assist back on after a move, if the lane is still synced.

        :param lane: Lane that moved
        :param synced: What _pause_assist returned
        """
        if synced and self.sync.get(lane.name):
            self._send_assist(lane)

    def _start_move(self, lane: AFCLane, fields: Dict[str, Any]) -> tuple:
        """
        Sends a move, retrying while the lane is still slowing from its
        assist.

        :param lane: Lane to move
        :param fields: move request fields
        :return tuple: (move_done completion, or None when refused)
        """
        for _ in range(20):
            done = self.link.expect(lambda m, n=lane.index: m.get("ev") == "move_done"
                                    and m.get("lane") == n)
            reply = self.link.request("move", **fields)
            if reply.get("ok"):
                return done
            self.link.wait(done, 0.0)
            if reply.get("err") != "lane busy":
                break
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        self._report(lane, f"BoxTurtle refused a move ({reply.get('err')})")
        return None

    def _run_move(self, lane: AFCLane, fields: Dict[str, Any], distance: float,
                  speed: float, accel: float) -> tuple:
        """
        Sends a move and waits for move_done. A USB drop does not end it:
        once the board is back this gets the move's end, and a move the
        board stopped on the drop goes on with what is left.

        :param lane: Lane to move
        :param fields: move request fields
        :param distance: mm
        :param speed: mm/s
        :param accel: mm/s^2
        :return tuple: (end reason, mm moved)
        """
        timeout = abs(distance) / speed + speed / accel + 10.0
        moved = 0.0
        rest = float(distance)
        for _ in range(self.DROP_TRIES + 1):
            seq = self._next_seq()
            done = self._start_move(lane, dict(fields, mm=rest, seq=seq))
            if done is None:
                return "fault", moved
            msg = self.link.wait(done, timeout)
            if msg is not None and msg.get("ev") == "link_down":
                msg = self._after_drop(lane, "move_done", seq, timeout)
                if msg is None:
                    self._report(lane, "BoxTurtle disconnected during a move")
                    return "fault", moved
                self.logger.info(f"{lane.name}: BoxTurtle link came back during a move")
            if msg is None:
                self.link.send("stop", lane=lane.index)
                self._report(lane, f"BoxTurtle move of {distance:.1f} mm did not finish")
                return "fault", moved
            moved += float(msg.get("mm", 0.0))
            if msg.get("end") != "link_lost":
                self._settle_switches()
                return msg.get("end", "done"), moved
            rest = float(distance) - moved
            if abs(rest) < 0.01 or (rest > 0) != (distance > 0):
                self._settle_switches()
                return ("no_trigger" if fields.get("until") else "done"), moved
            self.logger.info(f"{lane.name}: BoxTurtle stopped on a link drop, "
                             f"moving the last {rest:.1f} mm")
        self._report(lane, "BoxTurtle link kept dropping during a move")
        return "fault", moved

    def _settle_switches(self) -> None:
        """
        A move or job can end before AFC has run the switch change that
        ended it, and AFC reads the switches right after. Copies the board's
        switch states into AFC.
        """
        status = self.link.request("status")
        sw = status.get("sw")
        if not status.get("ok") or not isinstance(sw, dict):
            return
        self.switches.update(sw)
        self._apply_switches()

    def _fw_move_poll(self, lane: AFCLane, distance: float, speed: float, accel: float,
                      assist: bool, predicate: Callable[[], bool], want: bool) -> tuple:
        """
        A move that ends on a Klipper-side sensor (a toolhead switch): the
        host watches it and stops the lane.

        :param lane: Lane to move
        :param distance: Most to move, mm
        :param speed: mm/s
        :param accel: mm/s^2
        :param assist: Run the espooler
        :param predicate: Reads the sensor
        :param want: Sensor state that ends the move
        :return tuple: (end reason, mm moved)
        """
        if bool(predicate()) == want:
            return "triggered", 0.0
        speed = float(speed or lane.short_moves_speed)
        accel = float(accel or lane.short_moves_accel)
        fields = {"lane": lane.index, "mm": float(distance), "speed": speed,
                  "accel": accel, "hold": True}
        fields.update(self._espool_fields(lane, speed, assist))
        self.wait_toolhead()
        self.moving.add(lane.name)
        synced = self._pause_assist(lane)
        try:
            return self._poll_move(lane, fields, distance, speed, accel,
                                   predicate, want)
        finally:
            self.moving.discard(lane.name)
            self._resume_assist(lane, synced)

    def _poll_move(self, lane: AFCLane, fields: Dict[str, Any], distance: float,
                   speed: float, accel: float, predicate: Callable[[], bool],
                   want: bool) -> tuple:
        """
        Runs the move for _fw_move_poll, stopping it when the sensor reads
        the wanted state. After a USB drop the lane is stopped, and unless
        the sensor got there meanwhile the move goes on with what is left.

        :param lane: Lane to move
        :param fields: move request fields
        :param distance: Most to move, mm
        :param speed: mm/s
        :param accel: mm/s^2
        :param predicate: Reads the sensor
        :param want: Sensor state that ends the move
        :return tuple: (end reason, mm moved)
        """
        moved = 0.0
        rest = float(distance)
        for _ in range(self.DROP_TRIES + 1):
            seq = self._next_seq()
            done = self._start_move(lane, dict(fields, mm=rest, seq=seq))
            if done is None:
                return "fault", moved
            hit = False
            end_time = self.reactor.monotonic() + abs(rest) / speed + speed / accel + 10.0
            while not done.test():
                if bool(predicate()) == want:
                    hit = True
                    self.link.send("stop", lane=lane.index)
                    break
                now = self.reactor.monotonic()
                if now > end_time:
                    self.link.send("stop", lane=lane.index)
                    break
                done.wait(now + 0.01, None)
            msg = self.link.wait(done, 2.0)
            dropped = msg is not None and msg.get("ev") == "link_down"
            if dropped:
                msg = self._after_drop(lane, "move_done", seq, 2.0, stop=True)
            if msg is None:
                self.link.send("stop", lane=lane.index)
                self._report(lane, "BoxTurtle move to a Klipper-side sensor did not finish")
                return "fault", moved
            moved += float(msg.get("mm", 0.0))
            if hit or bool(predicate()) == want:
                self._settle_switches()
                return "triggered", moved
            if not dropped and msg.get("end") != "link_lost":
                self._settle_switches()
                return msg.get("end", "done"), moved
            rest = float(distance) - moved
            if abs(rest) < 0.01 or (rest > 0) != (distance > 0):
                self._settle_switches()
                return "done", moved
            self.logger.info(f"{lane.name}: BoxTurtle link dropped during a move to a "
                             f"Klipper-side sensor, moving the last {rest:.1f} mm")
        self._report(lane, "BoxTurtle link kept dropping during a move")
        return "fault", moved

    def endstop_switch(self, lane: AFCLane, spec) -> Optional[str]:
        """
        Board switch for an AFC endstop name.

        :param lane: Lane that homes
        :param spec: AFC endstop name
        :return str: Board switch name, or None for a Klipper-side sensor
        """
        name = str(spec) if spec is not None else ""
        if name in ("load", f"{lane.name}_load"):
            return "load"
        if name in ("prep", f"{lane.name}_prep"):
            return "prep"
        if name in ("hub", f"{lane.name}_hub"):
            return "hub"
        if isinstance(lane.buffer_obj, TurtleNeck):
            if name in ("buffer", "buffer_advance", f"{lane.name}_buffer"):
                return "tn_adv"
            if name == "buffer_trailing":
                return "tn_trl"
        return None

    def _sensor_predicate(self, lane: AFCLane, spec) -> Optional[Callable[[], bool]]:
        """
        Reader for a Klipper-side sensor AFC homes to.

        :param lane: Lane that homes
        :param spec: AFC endstop name
        :return: Function returning the sensor state, or None if unknown
        """
        name = str(spec) if spec is not None else ""
        extruder = lane.extruder_obj
        if name in ("tool", "tool_start", f"{lane.name}_tool_start"):
            return lane.get_toolhead_pre_sensor_state
        if name == "tool_end":
            return lambda: bool(extruder.tool_end_state)
        if name in ("buffer", "buffer_advance") and lane.buffer_obj is not None:
            return lambda: bool(lane.buffer_obj.advance_state)
        if name == "buffer_trailing" and lane.buffer_obj is not None:
            return lambda: bool(lane.buffer_obj.trailing_state)
        return None

    def fw_home(self, lane: AFCLane, spec, distance: float, speed: float, accel: float,
                triggered: bool, check_trigger: bool, assist: bool) -> tuple:
        """
        AFC_stepper.home_to on the board.

        :return tuple: (homed, distance moved, error)
        """
        switch = self.endstop_switch(lane, spec)
        if switch is not None:
            end, moved = self.fw_move(lane, distance, speed, accel, assist,
                                      until=switch, until_v=1 if triggered else 0)
        else:
            predicate = self._sensor_predicate(lane, spec)
            if predicate is None:
                self._report(lane, f"BoxTurtle cannot home to endstop {spec}")
                return False, 0.0, True
            end, moved = self._fw_move_poll(lane, distance, speed, accel, assist,
                                            predicate, bool(triggered))
        if end == "triggered":
            return True, abs(moved), False
        if end in ("done", "no_trigger"):
            if check_trigger:
                return False, abs(distance), False
            return True, abs(moved), False
        return False, abs(moved), True

    # ── sync and assist ──────────────────────────────────────────

    def set_sync(self, lane: AFCLane, on: bool) -> None:
        """
        AFC synced or unsynced the lane to its extruder.

        :param lane: The lane
        :param on: Synced
        """
        # A Klipper stepper changes sync in the toolhead's timeline.
        self.wait_toolhead()
        self.sync[lane.name] = on
        self.e_last.pop(lane.name, None)
        self.e_sent.pop(lane.name, None)
        if on:
            self._send_assist(lane)
        else:
            self.assist_mode.pop(lane.name, None)
            self.link.send("assist", lane=lane.index, on=False)
        self._update_e_timer()

    def buffer_changed(self, lane: AFCLane, on: bool) -> None:
        """
        The lane's TurtleNeck buffer was enabled or disabled.

        :param lane: The lane
        :param on: Enabled
        """
        if lane.name not in self.buffer_on:
            return
        self.buffer_on[lane.name] = on
        if self.sync.get(lane.name):
            if on:
                self._send_pulses(lane)
            self._send_assist(lane)

    def external_buffer_on(self, lane: AFCLane) -> bool:
        """
        :param lane: The lane
        :return bool: An external buffer is enabled for this lane
        """
        buffer = lane.buffer_obj
        return (buffer is not None
                and not isinstance(buffer, TurtleNeck)
                and bool(getattr(buffer, 'enable', False))
                and getattr(buffer, 'current_lane', None) is lane)

    def _wanted_mode(self, lane: AFCLane) -> str:
        """
        :param lane: A synced lane
        :return str: tn (TurtleNeck on), ext (external buffer on) or follow
        """
        if self.buffer_on.get(lane.name):
            return "tn"
        if self.external_buffer_on(lane):
            return "ext"
        return "follow"

    def _send_assist(self, lane: AFCLane) -> None:
        """
        Starts (or switches) the lane's assist: TurtleNeck mode while the
        TurtleNeck is on, ext mode while an external buffer is on, follow
        mode while only synced.

        :param lane: The lane
        """
        mode = self._wanted_mode(lane)
        if mode == "ext":
            self._send_pulses(lane)
        speed = getattr(lane.extruder_obj, 'tool_load_speed', 25) or 25
        fields = {"lane": lane.index, "on": True, "mode": mode}
        fields.update(self._espool_fields(lane, float(speed),
                                          bool(getattr(lane, 'assisted_unload', True))))
        self.assist_mode[lane.name] = mode
        self.link.send("assist", **fields)

    def _send_pulses(self, lane: AFCLane) -> None:
        """
        Sends the lane's print assist pulse from its spool weight now.

        :param lane: The lane
        """
        delta_mm, pulse_ms = self.pulse_settings(lane)
        self.link.send("lane_cfg", lane=lane.index, espool_pulse_mm=float(delta_mm),
                       espool_pulse_ms=int(pulse_ms))

    def send_fault_mm(self, mm: float) -> None:
        """
        :param mm: Clog check distance, 0 = off
        """
        self.link.send("cfg", assist_fault_mm=float(mm))

    def set_current(self, lane: AFCLane, printing: bool) -> None:
        """
        Switches the lane between run_current and print_current.

        :param lane: The lane
        :param printing: Use print_current
        """
        s = self.lane_settings.get(lane.name, {})
        current = s.get('print_current') if printing else None
        if printing and current is None:
            return
        if current is None:
            current = s.get('run_current', self.run_current)
        self.link.send("lane_cfg", lane=lane.index, run_current=float(current))

    def _update_e_timer(self) -> None:
        """
        Streams the extruder position while any lane follows it.
        """
        active = any(self.sync.values()) and self.link.connected
        self.reactor.update_timer(self.e_timer,
                                  self.reactor.NOW if active else self.reactor.NEVER)

    def lane_position(self, lane: AFCLane, e_pos: float) -> float:
        """
        The position the board's lane follows. In ext mode the extruder's
        moves are scaled by the external buffer's multiplier as they come,
        so a new multiplier never makes the lane jump.

        :param lane: A synced lane
        :param e_pos: The extruder position now
        :return float: Position to send
        """
        last = self.e_last.get(lane.name)
        sent = self.e_sent.get(lane.name)
        self.e_last[lane.name] = e_pos
        if last is None or sent is None:
            pos = e_pos
        elif self.assist_mode.get(lane.name) == "ext":
            pos = sent + (e_pos - last) * lane.drive_stepper.multiplier
        else:
            pos = sent + (e_pos - last)
        self.e_sent[lane.name] = pos
        return pos

    def _e_stream_cb(self, eventtime: float) -> float:
        """
        Sends each synced lane the position to follow, switches its mode
        when its buffer is turned on or off, and refreshes the weight-based
        pulse every 30 s.

        :param eventtime: Reactor time
        :return float: Next wake time
        """
        if not any(self.sync.values()) or not self.link.connected:
            return self.reactor.NEVER
        print_time = self.mcu.estimated_print_time(eventtime)
        positions = {}
        for lane in self.lanes.values():
            if not self.sync.get(lane.name):
                continue
            if self._wanted_mode(lane) != self.assist_mode.get(lane.name):
                self._send_assist(lane)
            extruder = getattr(lane.extruder_obj, 'toolhead_extruder', None)
            if extruder is None:
                continue
            if id(extruder) not in positions:
                positions[id(extruder)] = float(extruder.find_past_position(print_time))
            pos = self.lane_position(lane, positions[id(extruder)])
            self.link.send("e", lane=lane.index, pos=float(pos))
        self._pulse_tick += 1
        if self._pulse_tick >= int(30 / self.E_STREAM_TIME):
            self._pulse_tick = 0
            for lane in self.lanes.values():
                if (self.sync.get(lane.name)
                        and self.assist_mode.get(lane.name) in ("tn", "ext")):
                    self._send_pulses(lane)
        return eventtime + self.E_STREAM_TIME

    # ── AFC unit hooks ─────────────────────────────────────────────

    def _wait_for_board(self) -> bool:
        """
        Waits a little for the first connect (PREP runs right after start).

        :return bool: True when the board is connected
        """
        end = self.reactor.monotonic() + self.CONNECT_WAIT
        while not self.link.connected and self.reactor.monotonic() < end:
            self.reactor.pause(self.reactor.monotonic() + 0.2)
        return self.link.connected

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement) -> bool:
        """
        PREP's lane check, after the board is up.
        """
        if not self._wait_for_board():
            msg = (f"<span class=error--text>{self.name}: BoxTurtle not found on USB"
                   f"{' (' + self.link.last_error + ')' if self.link.last_error else ''}</span>")
            self.logger.info(f"{cur_lane.name} {msg}")
            self.lane_fault(cur_lane)
            cur_lane.set_afc_prep_done()
            return False
        return super().system_Test(cur_lane, delay, assignTcmd, enable_movement)

    def prep_load(self, lane: AFCLane):
        """
        A spool was inserted: the board's feed job pulls it to the load
        switch, retrying on its own before AFC hears about a problem.
        """
        if lane.raw_load_state:
            return
        speed, accel = lane.short_moves_speed, lane.short_moves_accel
        result = self.fw_job(lane, "feed", until="load",
                             mm=float(self.short_move_dis * self.MAX_NUM_MOVES),
                             speed=float(speed), accel=float(accel), watch="prep",
                             hold=True, **self._espool_fields(lane, speed, False))
        if not result.get("ok") and lane.prep_state:
            msg = (f' FAILED TO LOAD, CHECK FILAMENT AT TRIGGER ({result.get("msg") or result.get("code")})'
                   '\n||==>--||----||------||\nTRG   LOAD   HUB    TOOL')
            self.afc.error.AFC_error(msg, False)
            self.lane_fault(lane)
            lane.status = AFCLaneState.NONE

    def eject_lane(self, lane: AFCLane):
        """
        Ejects a spool with the board's unload job: back past the load
        switch, retrying a stuck retract, then extruder_clear_dis more.
        """
        try:
            mm = float(lane.dist_hub) + 100.0
            speed, accel = lane.get_speed_accel(SpeedMode.DIST_HUB)
            result = self.fw_job(lane, "unload", until="load", mm=mm, speed=float(speed),
                                 accel=float(accel), extra_mm=float(lane.extruder_clear_dis),
                                 **self._espool_fields(lane, speed, True))
            if not result.get("ok"):
                self.afc.error.AFC_error(f' Failed to eject {lane.name} '
                                         f'({result.get("msg") or result.get("code")})', False)
        finally:
            lane.do_enable(False)

    def fw_job(self, lane: AFCLane, job: str, **fields) -> Dict[str, Any]:
        """
        Runs a self-recovering job on the board and waits for job_done. A
        USB drop does not end it: once the board is back this gets the job's
        end, and a job the board stopped on the drop runs again.

        :param lane: Lane to run it on
        :param job: feed, unload or verify
        :param fields: Job fields
        :return dict: The job_done event, or {"ok": False, ...}
        """
        speed = float(fields.get("speed", 50.0)) or 50.0
        tries = int(fields.get("tries", self.job_tries)) + 1
        timeout = tries * (2 * float(fields.get("mm", 100.0)) / speed + 10.0)
        self.wait_toolhead()
        self.moving.add(lane.name)
        try:
            for _ in range(self.DROP_TRIES + 1):
                seq = self._next_seq()
                done = self.link.expect(lambda m, n=lane.index, q=seq: m.get("ev") == "job_done"
                                        and m.get("lane") == n and m.get("seq") == q)
                reply = self.link.request("job", job=job, lane=lane.index, seq=seq, **fields)
                if not reply.get("ok"):
                    self.link.wait(done, 0.0)
                    return {"ok": False, "code": reply.get("err")}
                msg = self.link.wait(done, timeout)
                if msg is not None and msg.get("ev") == "link_down":
                    msg = self._after_drop(lane, "job_done", seq, timeout)
                    if msg is None:
                        return {"ok": False, "code": "link_down"}
                    self.logger.info(f"{lane.name}: BoxTurtle link came back during the {job} job")
                if msg is None:
                    self.link.send("stop", lane=lane.index)
                    return {"ok": False, "code": "timeout"}
                if msg.get("code") != "link_lost":
                    self._settle_switches()
                    return msg
                self.logger.info(f"{lane.name}: BoxTurtle stopped the {job} job on a "
                                 f"link drop, running it again")
            return {"ok": False, "code": "link_lost"}
        finally:
            self.moving.discard(lane.name)

    def _trigger_led_state(self, lane=None, extruder=None, static_color=None,
                           extruder_static_color=None, effect_suffix=None) -> None:
        """
        AFC's lane LED states, also shown on the board's own LEDs.
        """
        super()._trigger_led_state(lane, extruder, static_color,
                                   extruder_static_color, effect_suffix)
        if lane is not None and static_color is not None:
            self.board_led(lane, static_color)

    def board_led(self, lane: AFCLane, color) -> None:
        """
        Sets a lane's LEDs on the board (turtle_led in its [AFC_lane]).

        :param lane: The lane
        :param color: AFC color
        """
        self.led_colors[lane.name] = color
        hex_color = led_hex(color)
        if hex_color is None:
            return
        for header, first, last in self.lane_settings.get(lane.name, {}).get('turtle_led', []):
            self.link.send("led", strip=header, i=first, n=last - first + 1,
                           color=hex_color)

    def get_status(self, eventtime=None):
        """
        Unit status plus the board's link state.
        """
        response = super().get_status(eventtime)
        response["turtle"] = {
            "connected": self.link.connected,
            "port": self.link.port,
            "firmware": self.link.info.get("fw"),
            "uid": self.link.info.get("uid"),
            "reset": self.link.info.get("reset"),
            "error": self.link.last_error,
            "turtleneck": self.buffer_obj.get_status() if isinstance(self.buffer_obj, TurtleNeck) else None,
            "assist": dict(self.assist_mode),
        }
        return response

    # ── gcode ──────────────────────────────────────────────────────

    cmd_AFC_TURTLE_STATUS_help = "Show the BoxTurtle board's link and switch states"
    def cmd_AFC_TURTLE_STATUS(self, gcmd):
        """
        Shows the USB link, firmware and the board's live status.

        Usage
        -----
        `AFC_TURTLE_STATUS UNIT=<unit>`
        """
        if not self.link.connected:
            gcmd.respond_info(f"{self.name}: not connected ({self.link.last_error or 'no board found'})")
            return
        status = self.link.request("status")
        info = self.link.info
        gcmd.respond_info(f"{self.name}: firmware {info.get('fw')} on {self.link.port}\n"
                          f"switches: {status.get('sw')}\nlanes: {status.get('lanes')}")

    cmd_AFC_TURTLE_SEND_help = "Send one raw JSON request to the BoxTurtle board"
    def cmd_AFC_TURTLE_SEND(self, gcmd):
        """
        Bench tool: sends a raw request and shows the reply.

        Usage
        -----
        `AFC_TURTLE_SEND UNIT=<unit> CMD=<cmd> [ARGS=<json object>]`

        Example
        -----
        `AFC_TURTLE_SEND UNIT=Turtle_1 CMD=tmc_read ARGS='{"lane":1,"reg":111}'`
        """
        cmd = gcmd.get('CMD')
        args = gcmd.get('ARGS', '{}')
        try:
            fields = json.loads(args)
        except ValueError as e:
            error_string = f"ARGS is not JSON: {e}"
            raise gcmd.error(error_string)
        gcmd.respond_info(str(self.link.request(cmd, timeout=3.0, **fields)))

    cmd_AFC_TURTLE_BOOTLOADER_help = "Restart the BoxTurtle board into its bootloader"
    def cmd_AFC_TURTLE_BOOTLOADER(self, gcmd):
        """
        Restarts the board into Katapult for a flash; refused while printing.

        Usage
        -----
        `AFC_TURTLE_BOOTLOADER UNIT=<unit>`
        """
        if self.afc.function.is_printing():
            error_string = "Cannot restart the BoxTurtle into its bootloader while printing"
            raise gcmd.error(error_string)
        gcmd.respond_info(str(self.link.request("bootloader")))


def load_config_prefix(config):
    return afcBoxTurtleUSB(config)
