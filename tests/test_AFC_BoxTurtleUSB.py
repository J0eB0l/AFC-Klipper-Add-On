"""
Unit tests for extras/AFC_BoxTurtleUSB.py

Covers:
  - encode_line, led_hex
  - TurtleLink over a real pty, with a fake board answering on the master,
    and over a real loopback TCP connection (tcp:// serial_port), with and
    without a BridgeHub's link key exchange
  - TurtleNeck, TurtleRotation, TurtleDrive
  - afcBoxTurtleUSB with a fake link: settings, sync and assist modes (tn,
    follow, ext for an external buffer), moves, homing, switches, faults,
    jobs, LEDs, status and gcode
"""

from __future__ import annotations

import errno
import fcntl
import hashlib
import hmac
import json
import os
import select
import socket
import termios
from typing import Any, Dict, List, Optional
from unittest.mock import MagicMock, patch

import pytest

from extras import AFC_BoxTurtleUSB as mod
from extras import AFC_bhub_link as hub_link
from extras.AFC_bhub_link import check_link_key
from extras.AFC_BoxTurtle import afcBoxTurtle
from extras.AFC_BoxTurtleUSB import (
    LINE_MAX,
    PROTO,
    TurtleDrive,
    TurtleLink,
    TurtleNeck,
    TurtleRotation,
    afcBoxTurtleUSB,
    encode_line,
    led_hex,
    load_config_prefix,
    parse_turtle_led,
    split_fields,
)
from extras.AFC_lane import AFCLaneState
from extras.AFC_unit import afcUnit
from tests.conftest import MockConfig, MockGCodeCommand, MockLogger, MockPrinter


# ── reactor fakes ─────────────────────────────────────────────────────────────

class FakeCompletion:
    """Reactor completion: wait() runs the fake reactor until it completes."""

    def __init__(self, reactor: "FakeReactor") -> None:
        self.reactor = reactor
        self.done = False
        self.result: Any = None

    def test(self) -> bool:
        return self.done

    def complete(self, value: Any) -> None:
        self.done = True
        self.result = value

    def wait(self, waketime: Optional[float] = None, waketime_result: Any = None) -> Any:
        while not self.done:
            if waketime is not None and self.reactor.now >= waketime:
                return waketime_result
            self.reactor.step()
        return self.result


class FakeReactor:
    """Time moves only when something waits; fds are polled with select."""
    NOW = 0.0
    NEVER = 9_999_999_999.0

    def __init__(self) -> None:
        self.now = 100.0
        self.timers: List[Dict[str, Any]] = []
        self.fds: Dict[int, Any] = {}
        self.callbacks: List[Any] = []
        self.callback_times: List[Any] = []
        self.pumps: List[Any] = []

    def monotonic(self) -> float:
        return self.now

    def register_timer(self, callback, waketime=None):
        timer = {"cb": callback, "when": self.NEVER if waketime is None else waketime}
        self.timers.append(timer)
        return timer

    def update_timer(self, timer, waketime) -> None:
        timer["when"] = waketime

    def register_fd(self, fd, callback):
        self.fds[fd] = callback
        return fd

    def unregister_fd(self, handle) -> None:
        self.fds.pop(handle, None)

    def register_callback(self, callback, waketime=None) -> None:
        self.callbacks.append(callback)
        self.callback_times.append(waketime)

    def completion(self) -> FakeCompletion:
        return FakeCompletion(self)

    def mutex(self, is_locked=False):
        return MagicMock()

    def step(self, dt: float = 0.005) -> None:
        for pump in list(self.pumps):
            pump()
        for fd, callback in list(self.fds.items()):
            if fd in self.fds and select.select([fd], [], [], 0)[0]:
                callback(self.now)
        self.now += dt

    def pause(self, until: float) -> float:
        while self.now < until:
            self.step()
        return self.now

    def run_callbacks(self) -> None:
        while self.callbacks:
            self.callbacks.pop(0)(self.now)


# ── link fakes ────────────────────────────────────────────────────────────────

class LinkUnit:
    """What TurtleLink needs from its unit."""

    def __init__(self, reactor: FakeReactor) -> None:
        self.name = "Turtle_1"
        self.reactor = reactor
        self.logger = MockLogger()
        self.link_up: List[Dict[str, Any]] = []
        self.link_down: List[tuple] = []
        self.events: List[Dict[str, Any]] = []
        self.up_result = True
        self.tcp_key: Optional[str] = None
        self.saved_key: Optional[str] = None
        self.new_key: Optional[str] = "ab" * 16
        self.key_calls: List[bool] = []

    def link_key(self, make):
        self.key_calls.append(make)
        if self.tcp_key or self.saved_key:
            return self.tcp_key or self.saved_key
        if make and self.new_key:
            self.saved_key = self.new_key
        return self.saved_key

    def handle_link_up(self, info):
        self.link_up.append(info)
        return self.up_result

    def handle_link_down(self, reason, was_connected):
        self.link_down.append((reason, was_connected))

    def handle_event(self, msg, eventtime):
        self.events.append(msg)


class PtyBoard:
    """The board end of a pty: reads request lines, writes replies."""

    def __init__(self, master: int, reactor: FakeReactor) -> None:
        self.master = master
        os.set_blocking(master, False)
        self.received: List[Dict[str, Any]] = []
        self.raw: List[bytes] = []
        self.buf = b""
        self.info = {"ok": True, "proto": PROTO, "fw": "fw1",
                     "uid": "UID1", "reset": "power"}
        self.handler = self.default
        reactor.pumps.append(self.service)

    def default(self, msg):
        if msg.get("cmd") == "info":
            return [dict(self.info, id=msg["id"])]
        return [{"id": msg.get("id"), "ok": True}]

    def service(self, wait: float = 0.0) -> None:
        """
        Reads what the link wrote. A pty hands data across asynchronously,
        so a test that checks right after a write drains with a wait.
        """
        while select.select([self.master], [], [], wait)[0]:
            try:
                data = os.read(self.master, 4096)
            except (BlockingIOError, OSError):
                break
            if not data:
                break
            self.buf += data
        self._lines()

    def drain(self) -> None:
        self.service(wait=0.05)

    def _lines(self) -> None:
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            if not line.strip():
                continue
            msg = json.loads(line)
            self.raw.append(line)
            self.received.append(msg)
            for reply in self.handler(msg) or []:
                self.write(reply)

    def write(self, obj) -> None:
        data = obj if isinstance(obj, bytes) else (json.dumps(obj) + "\n").encode()
        os.write(self.master, data)


@pytest.fixture
def pty():
    master, slave = os.openpty()
    path = os.ttyname(slave)
    reactor = FakeReactor()
    unit = LinkUnit(reactor)
    link = TurtleLink(unit, path, None)
    board = PtyBoard(master, reactor)
    yield link, board, unit, reactor, path
    link.close("test done", notify=False)
    for fd in (master, slave):
        try:
            os.close(fd)
        except OSError:
            pass


def wait_readable(link: TurtleLink) -> None:
    """Waits for bytes the board wrote to reach the link's end of the pty."""
    assert select.select([link.fd], [], [], 1.0)[0]


def connect(link: TurtleLink, reactor: FakeReactor, path: str) -> None:
    """Opens the pty and runs the handshake."""
    assert link.open(path)
    reactor.run_callbacks()
    assert link.connected


@pytest.fixture
def tcp():
    """A link to a loopback listener; accept() hands back the board end."""
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    url = f"tcp://127.0.0.1:{server.getsockname()[1]}"
    reactor = FakeReactor()
    unit = LinkUnit(reactor)
    link = TurtleLink(unit, url, None)
    ends = []

    def accept() -> PtyBoard:
        end, _ = server.accept()
        ends.append(end)
        board = PtyBoard(end.fileno(), reactor)
        board.sock = end
        return board

    yield link, unit, reactor, url, accept
    link.close("test done", notify=False)
    for sock in ends + [server]:
        sock.close()


def refused_url() -> str:
    """A loopback address nothing listens on."""
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return f"tcp://127.0.0.1:{port}"


def wait_connected(link: TurtleLink) -> None:
    """Waits for the link's connect to finish, one way or the other."""
    assert select.select([], [link.fd], [], 1.0)[1]


# The firmware's host test (firmwares/BridgeHub/test) checks the same answer.
HUB_KEY = "0123456789abcdef0123456789abcdef"
HUB_NONCE = bytes(range(16))
HUB_MAC = "84aa390f3f68d40d3b8f2b5ee2ad3517c13e31649284a43e867a8fa5974500bc"


def make_hub(board: PtyBoard, key: Optional[str]) -> PtyBoard:
    """
    Makes the board end act as a BridgeHub that holds key (None: an open
    hub): it greets, then checks the answer or takes the claim.
    """
    board.hub_key = key
    unit_handler = board.handler

    def handler(msg):
        if msg.get("cmd") == "auth":
            want = hmac.new(board.hub_key.encode(), HUB_NONCE, hashlib.sha512).hexdigest()
            return [{"evt": "auth", "ok": int(msg.get("mac") == want[:64])}]
        if msg.get("cmd") == "linkkey":
            board.hub_key = msg["key"]
            return [{"evt": "auth", "ok": 1}]
        return unit_handler(msg)

    board.handler = handler
    board.write({"evt": "auth", "nonce": HUB_NONCE.hex()} if key else {"evt": "auth", "open": 1})
    return board


def tcp_handshake(link: TurtleLink, reactor: FakeReactor, url: str, accept,
                  key: Optional[str] = None, hub: bool = True) -> PtyBoard:
    """Connects to the listener, which answers as a hub (or not), and runs the handshake."""
    assert link.open(url)
    board = accept()
    if hub:
        make_hub(board, key)
    wait_connected(link)
    assert link._connect_pending(101.0) is False
    reactor.run_callbacks()
    return board


# ── unit fakes ────────────────────────────────────────────────────────────────

class FakeLink:
    """Records what the unit sends; replies and events are scripted."""

    def __init__(self, reactor: FakeReactor) -> None:
        self.reactor = reactor
        self.connected = True
        self.port = "/dev/serial/by-id/usb-AFC_BoxTurtle_UID1-if00"
        self.info = {"fw": "fw1", "uid": "UID1", "reset": "power"}
        self.last_error = ""
        self.sent: List[tuple] = []
        self.requests: List[tuple] = []
        self.replies: Dict[str, Any] = {}
        self.events: List[Any] = []
        self.waiters: List[tuple] = []
        self.started = False
        self.up_results: List[bool] = []
        self.up_waits: List[float] = []

    def start(self) -> None:
        self.started = True

    def wait_up(self, timeout):
        self.up_waits.append(timeout)
        if self.up_results:
            return self.up_results.pop(0)
        return self.connected

    def send(self, cmd, **fields) -> None:
        self.sent.append((cmd, fields))

    def request(self, cmd, timeout=1.0, **fields):
        self.requests.append((cmd, fields))
        reply = self.replies.get(cmd, {"ok": True})
        if isinstance(reply, list):
            reply = reply.pop(0) if reply else {"ok": True}
        return reply

    def expect(self, predicate):
        completion = FakeCompletion(self.reactor)
        self.waiters.append((predicate, completion))
        return completion

    def wait(self, completion, timeout):
        if not completion.done and self.events and timeout > 0:
            event = self.events.pop(0)
            if event is None:
                return None
            completion.complete(event)
        if completion.done:
            return completion.result
        return completion.wait(self.reactor.now + timeout, None)


class FakeToolhead:
    """Toolhead whose queued moves end at last_move_time (print time)."""

    def __init__(self) -> None:
        self.last_move_time = 0.0
        self.calls = 0

    def get_last_move_time(self) -> float:
        self.calls += 1
        return self.last_move_time


class UnitConfig(MockConfig):
    """MockConfig with the [AFC_lane] sections the unit reads."""

    def __init__(self, name, printer, values, sections=()):
        super().__init__(name=name, printer=printer, values=values)
        self.sections = list(sections)

    def get_prefix_sections(self, prefix):
        return [s for s in self.sections if s.get_name().startswith(prefix)]


class TurtlePrinter(MockPrinter):
    """MockPrinter with Klipper's add_object."""

    def add_object(self, name, obj):
        """
        :param name: Printer object name
        :param obj: The object
        """
        if name in self._objects:
            raise Exception(f"Printer object '{name}' already created")
        self._objects[name] = obj


def make_unit(values=None, lane_sections=(), fake_link=True):
    printer = TurtlePrinter()
    reactor = FakeReactor()
    printer._reactor = printer.reactor = reactor
    sections = [MockConfig(name=f"AFC_lane {n}", printer=printer, values=v)
                for n, v in lane_sections]
    config = UnitConfig("AFC_BoxTurtleUSB Turtle_1", printer, values or {}, sections)
    unit = afcBoxTurtleUSB(config)
    unit.afc.function.is_printing = MagicMock(return_value=False)
    unit.afc.function.in_print = MagicMock(return_value=False)
    if fake_link:
        unit.link = FakeLink(reactor)
    unit.mcu = MagicMock()
    unit.mcu.estimated_print_time = lambda eventtime: eventtime - 1.0
    printer._objects["toolhead"] = FakeToolhead()
    return unit


def make_lane(unit, name="lane8", index=1, buffer=None, add=True):
    lane = MagicMock()
    lane.name = name
    lane.index = index
    lane.buffer_obj = buffer
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.tool_load_speed = 25
    lane.extruder_obj.tool_end_state = False
    lane.assisted_unload = True
    lane.short_moves_speed = 50.0
    lane.short_moves_accel = 400.0
    lane.calculate_pwm_value = MagicMock(side_effect=lambda speed, rev=False: 0.4 if rev else 0.6)
    lane.espooler.enable_kick_start = True
    lane.espooler.enable_assist = True
    lane.espooler.enable_assist_weight = 100.0
    lane.espooler.espooler_values.delta_movement = 150.0
    lane.espooler.espooler_values.kick_start_time = 0.07
    lane.espooler.espooler_values.calculate_cruise_time = MagicMock(return_value=0.5)
    lane.weight = 50.0
    lane.dist_hub = 180.0
    lane.extruder_clear_dis = 50.0
    lane.get_speed_accel = MagicMock(return_value=(60.0, 300.0))
    lane.raw_load_state = False
    lane.prep_state = True
    lane.drive_stepper = TurtleDrive(unit, lane)
    if add:
        unit.lanes[name] = lane
        unit.sync[name] = False
        unit.buffer_on[name] = False
    return lane


def ext_buffer(lane=None, enable=True):
    buffer = MagicMock()
    buffer.enable = enable
    buffer.current_lane = lane
    return buffer


# ── encode_line ───────────────────────────────────────────────────────────────

class TestEncodeLine:
    def test_types(self):
        line = encode_line({"id": 3, "cmd": "move", "mm": 1.5, "hold": True,
                            "watch": False, "until": "load"})
        assert line == b'{"id":3,"cmd":"move","mm":1.500,"hold":true,"watch":false,"until":"load"}\n'

    def test_float_never_uses_an_exponent(self):
        assert encode_line({"pos": 1e-7}) == b'{"pos":0.000}\n'

    def test_other_values_become_strings(self):
        assert encode_line({"x": None}) == b'{"x":"None"}\n'


# ── split_fields ──────────────────────────────────────────────────────────────

# Ten fields of 48 bytes each (',"k0":"' + 40 x + '"'). With the 25 byte
# '{"id":9999999,"cmd":"cfg"' and the 2 byte '}\n', four fit (219), five do
# not (267).
TEN = {f"k{i}": "x" * 40 for i in range(10)}


class TestSplitFields:
    def test_fits_one_line(self):
        assert split_fields("cfg", {"a": 1, "b": 2.5}) == [{"a": 1, "b": 2.5}]

    def test_splits_before_the_field_that_overflows(self):
        parts = split_fields("cfg", TEN)
        keys = [list(part) for part in parts]
        assert keys == [["k0", "k1", "k2", "k3"], ["k4", "k5", "k6", "k7"], ["k8", "k9"]]
        assert all(part[k] == "x" * 40 for part in parts for k in part)

    def test_keep_goes_on_every_line(self):
        # '{"id":9999999,"cmd":"lane_cfg","lane":2' is 39 bytes: still four per line
        parts = split_fields("lane_cfg", {"lane": 2, **TEN}, keep=("lane",))
        keys = [list(part) for part in parts]
        assert keys == [["lane", "k0", "k1", "k2", "k3"], ["lane", "k4", "k5", "k6", "k7"],
                        ["lane", "k8", "k9"]]
        assert [part["lane"] for part in parts] == [2, 2, 2]

    def test_keep_field_missing(self):
        assert split_fields("lane_cfg", {"a": 1}, keep=("lane",)) == [{"a": 1}]

    def test_oversized_field_goes_alone(self):
        big = "x" * 300
        assert split_fields("cfg", {"big": big, "a": 1}) == [{"big": big}, {"a": 1}]

    def test_nothing_to_send(self):
        assert split_fields("cfg", {}) == [{}]


# ── led_hex ───────────────────────────────────────────────────────────────────

class TestLedHex:
    def test_string(self):
        assert led_hex("1,0.5,0,0") == "FF800000"

    def test_list_and_padding(self):
        assert led_hex([0, 1, 0]) == "00FF0000"

    def test_clamped(self):
        assert led_hex("2,-1,0,1") == "FF0000FF"

    def test_bad_string(self):
        assert led_hex("red") is None

    def test_bad_type(self):
        assert led_hex(None) is None


# ── TurtleLink ────────────────────────────────────────────────────────────────

class TestParseTurtleLed:
    COUNTS = {1: 4, 2: 64, 3: 0, 4: 0}

    @pytest.mark.parametrize("spec, expected", [
        ("RGB1:1", [(1, 1, 1)]),
        ("RGB2:25-32", [(2, 25, 32)]),
        ("rgb2:25-32, RGB1:3", [(2, 25, 32), (1, 3, 3)]),
        ("RGB1:4-4", [(1, 4, 4)]),
    ])
    def test_good(self, spec, expected):
        assert parse_turtle_led(spec, self.COUNTS) == expected

    @pytest.mark.parametrize("spec", [
        "25-32", "RGB5:1", "RGB:1", "LED1:1", "RGB1:", "RGB1:0", "RGB1:3-2",
        "RGB1:a", "RGB1:1-x", "",
    ])
    def test_malformed(self, spec):
        with pytest.raises(ValueError) as e:
            parse_turtle_led(spec, self.COUNTS)
        assert str(e.value) == f"turtle_led {spec} is not RGB<1-4>:<first>[-<last>]"

    @pytest.mark.parametrize("spec, part, count, header", [
        ("RGB1:5", "RGB1:5", 4, 1),
        ("RGB1:1, RGB2:60-65", "RGB2:60-65", 64, 2),
        ("RGB3:1", "RGB3:1", 0, 3),
    ])
    def test_past_the_header(self, spec, part, count, header):
        with pytest.raises(ValueError) as e:
            parse_turtle_led(spec, self.COUNTS)
        assert str(e.value) == (f"turtle_led {part} is past the {count} LEDs "
                                f"of rgb{header}_chain_count")


class TestTurtleLinkInit:
    def test_fields(self, pty):
        link, _, unit, reactor, path = pty
        assert link.port_cfg == path
        assert link.uid is None
        assert link.fd is None
        assert link.connected is False
        assert reactor.timers[0]["cb"] == link._timer_cb


class TestTurtleLinkStart:
    def test_wakes_timer_now(self, pty):
        link, _, _, reactor, _ = pty
        link.start()
        assert link.timer["when"] == reactor.NOW


class TestTurtleLinkStop:
    def test_closes_without_telling_the_unit(self, pty):
        link, _, unit, reactor, path = pty
        connect(link, reactor, path)
        link.stop()
        assert link.timer["when"] == reactor.NEVER
        assert link.fd is None
        assert link.last_error == "klipper stopping"
        assert unit.link_down == []
        assert unit.logger.messages == []


class TestTurtleLinkFindPort:
    def test_fixed_path_exists(self, pty):
        link, _, _, _, path = pty
        assert link.find_port() == path

    def test_fixed_path_missing(self, pty):
        link, *_ = pty
        link.port_cfg = "/dev/nope-turtle"
        assert link.find_port() is None

    def test_auto_takes_first(self, pty):
        link, *_ = pty
        link.port_cfg = "auto"
        with patch.object(mod.glob, "glob", return_value=["/b-if00", "/a-if00"]) as g:
            assert link.find_port() == "/a-if00"
        g.assert_called_once_with(mod.PORT_GLOB)

    def test_auto_filters_by_uid(self, pty):
        link, *_ = pty
        link.port_cfg = "auto"
        link.uid = "B51"
        with patch.object(mod.glob, "glob", return_value=["/x_A11-if00", "/x_B51-if00"]):
            assert link.find_port() == "/x_B51-if00"

    def test_auto_none(self, pty):
        link, *_ = pty
        link.port_cfg = "auto"
        with patch.object(mod.glob, "glob", return_value=[]):
            assert link.find_port() is None

    def test_tcp_always_tried(self, tcp):
        link, _, _, url, _ = tcp
        assert link.find_port() == url


class TestTurtleLinkTimerCb:
    def test_no_board(self, pty):
        link, *_ = pty
        link.port_cfg = "/dev/nope-turtle"
        assert link._timer_cb(200.0) == 200.0 + TurtleLink.RETRY_TIME
        assert link.fd is None

    def test_opens_board(self, pty):
        link, _, _, _, path = pty
        assert link._timer_cb(200.0) == 200.0 + TurtleLink.RETRY_TIME
        assert link.port == path
        assert link.fd is not None

    def test_quiet_link_closes(self, pty):
        link, _, unit, reactor, path = pty
        connect(link, reactor, path)
        unit.logger.messages.clear()
        now = link._last_rx + TurtleLink.RX_TIMEOUT + 0.1
        # Dropped just now: look again soon.
        assert link._timer_cb(now) == now + TurtleLink.FAST_RETRY_TIME
        assert link.fd is None
        assert unit.logger.messages == [("warning", f"Turtle_1: closed {path}: no reply for 3 s")]
        assert unit.link_down == [("no reply for 3 s", True)]

    def test_pings_while_connected(self, pty):
        link, board, _, reactor, path = pty
        connect(link, reactor, path)
        now = link._last_rx + 0.1
        assert link._timer_cb(now) == now + TurtleLink.PING_TIME
        board.drain()
        assert board.received[-1]["cmd"] == "ping"

    def test_no_ping_before_handshake(self, pty):
        link, board, _, _, path = pty
        assert link.open(path)
        now = link._last_rx + 0.1
        assert link._timer_cb(now) == now + TurtleLink.PING_TIME
        board.drain()
        assert board.received == []


class TestTurtleLinkTimerCbTcp:
    def test_starts_connect(self, tcp):
        link, _, _, url, _ = tcp
        assert link._timer_cb(200.0) == 200.0 + TurtleLink.CONNECT_POLL_TIME
        assert link.port == url
        assert link.connecting is True

    def test_connected_starts_handshake(self, tcp):
        link, _, reactor, _, accept = tcp
        assert link.open(link.port_cfg)
        accept()
        wait_connected(link)
        now = link._last_rx + 0.1
        assert link._timer_cb(now) == now + TurtleLink.PING_TIME
        assert link.connecting is False
        assert link._last_rx == now
        assert reactor.callbacks == [link._handshake]

    def test_still_connecting(self, tcp):
        link, unit, reactor, _, _ = tcp
        assert link.open(link.port_cfg)
        now = link._last_rx + 0.1
        with patch.object(mod.select, "select", return_value=([], [], [])):
            assert link._timer_cb(now) == now + TurtleLink.CONNECT_POLL_TIME
        assert link.connecting is True
        assert reactor.callbacks == []
        assert unit.logger.messages == []

    def test_connect_times_out(self, tcp):
        link, unit, _, url, _ = tcp
        assert link.open(link.port_cfg)
        now = link._last_rx + TurtleLink.RX_TIMEOUT + 0.1
        with patch.object(mod.select, "select", return_value=([], [], [])):
            assert link._timer_cb(now) == now + TurtleLink.RETRY_TIME
        assert link.fd is None and link.sock is None
        assert link.connecting is False
        assert unit.logger.messages == [("warning", f"Turtle_1: closed {url}: connect timed out")]
        assert unit.link_down == [("connect timed out", False)]

    def test_connect_refused(self, tcp):
        link, unit, reactor, _, _ = tcp
        url = refused_url()
        assert link.open(url)
        wait_connected(link)
        now = link._last_rx + 0.1
        assert link._timer_cb(now) == now + TurtleLink.RETRY_TIME
        assert link.fd is None
        assert reactor.callbacks == []
        assert unit.logger.messages == [
            ("warning", f"Turtle_1: closed {url}: cannot connect: Connection refused")]
        assert unit.link_down == [("cannot connect: Connection refused", False)]


class TestTurtleLinkRetryTime:
    def test_never_dropped(self, pty):
        link, *_ = pty
        assert link.retry_time(200.0) == TurtleLink.RETRY_TIME

    def test_soon_after_a_drop(self, pty):
        link, *_ = pty
        link._dropped_at = 195.0
        assert link.retry_time(200.0) == TurtleLink.FAST_RETRY_TIME

    def test_long_after_a_drop(self, pty):
        link, *_ = pty
        link._dropped_at = 200.0 - TurtleLink.FAST_RETRY_WINDOW
        assert link.retry_time(200.0) == TurtleLink.RETRY_TIME


class TestTurtleLinkOpen:
    def test_opens_raw_exclusive(self, pty):
        link, _, _, reactor, path = pty
        with patch.object(mod.fcntl, "ioctl", wraps=fcntl.ioctl) as ioctl:
            assert link.open(path) is True
        assert ioctl.call_args[0][1] == termios.TIOCEXCL
        attrs = termios.tcgetattr(link.fd)
        assert attrs[3] == 0
        assert attrs[4] == attrs[5] == termios.B115200
        assert link.port == path
        assert link.fd in reactor.fds
        assert reactor.callbacks == [link._handshake]

    def test_missing_port_warns_once(self, pty):
        link, _, unit, _, _ = pty
        assert link.open("/dev/nope-turtle") is False
        assert link.open("/dev/nope-turtle") is False
        msg = ("Turtle_1: cannot open /dev/nope-turtle: [Errno 2] No such file or "
               "directory: '/dev/nope-turtle'")
        assert unit.logger.messages == [("warning", msg)]
        assert link.last_error == msg

    def test_not_a_tty(self, pty):
        link, _, unit, _, _ = pty
        assert link.open("/dev/null") is False
        msg = "Turtle_1: cannot set up /dev/null: [Errno 25] Inappropriate ioctl for device"
        assert unit.logger.messages == [("warning", msg)]
        assert link.last_error == msg
        assert link.fd is None


class TestTurtleLinkOpenTcp:
    def test_starts_connect_without_handshake(self, tcp):
        link, _, reactor, url, _ = tcp
        assert link.open(url) is True
        assert link.connecting is True
        assert link.fd == link.sock.fileno()
        assert link.sock.getblocking() is False
        assert link.sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY) != 0
        assert link.port == url
        assert link._last_rx == 100.0
        assert link.fd in reactor.fds
        assert reactor.callbacks == []

    def test_talks_to_the_board(self, tcp):
        link, unit, reactor, url, accept = tcp
        assert link.open(url)
        board = accept()
        wait_connected(link)
        assert link._connect_pending(101.0) is False
        reactor.run_callbacks()
        assert link.connected is True
        assert board.received[0]["cmd"] == "info"
        assert unit.link_up == [link.info]

    def test_connect_error_warns_once(self, tcp):
        link, unit, _, url, _ = tcp
        with patch.object(mod.socket.socket, "connect_ex", return_value=errno.ENETUNREACH):
            assert link.open(url) is False
            assert link.open(url) is False
        msg = f"Turtle_1: cannot connect to {url}: Network is unreachable"
        assert unit.logger.messages == [("warning", msg)]
        assert link.last_error == msg
        assert link.fd is None and link.sock is None
        assert link.connecting is False

    def test_ipv6_address(self, tcp):
        link, _, reactor, _, _ = tcp
        sock = MagicMock()
        sock.connect_ex.return_value = errno.EINPROGRESS
        sock.fileno.return_value = 99
        with patch.object(mod.socket, "socket", return_value=sock) as make:
            assert link.open("tcp://[fe80::1]:4002") is True
        make.assert_called_once_with(socket.AF_INET6, socket.SOCK_STREAM)
        sock.connect_ex.assert_called_once_with(("fe80::1", 4002))
        assert link.fd == 99 and 99 in reactor.fds

    def test_no_socket(self, tcp):
        link, unit, _, url, _ = tcp
        with patch.object(mod.socket, "socket", side_effect=OSError("no sockets")):
            assert link.open(url) is False
        assert unit.logger.messages == [("warning", f"Turtle_1: cannot open {url}: no sockets")]
        assert link.fd is None


class TestTurtleLinkConnectPending:
    def test_read_while_connecting_finishes_the_connect(self, tcp):
        link, unit, _, _, _ = tcp
        url = refused_url()
        assert link.open(url)
        assert select.select([link.fd], [], [], 1.0)[0]
        link._read_cb(101.0)
        assert link.fd is None
        assert unit.logger.messages == [
            ("warning", f"Turtle_1: closed {url}: cannot connect: Connection refused")]


class TestTurtleLinkHandshake:
    def test_connects(self, pty):
        link, board, unit, reactor, path = pty
        connect(link, reactor, path)
        assert board.received[0]["cmd"] == "info"
        assert link.info["fw"] == "fw1"
        assert link.last_error == ""
        assert unit.link_up == [link.info]

    def test_wrong_protocol(self, pty):
        link, board, unit, reactor, path = pty
        board.info["proto"] = "boxturtle-0"
        assert link.open(path)
        reactor.run_callbacks()
        msg = (f"Turtle_1: board at {path} speaks boxturtle-0, not {PROTO}; "
               "flash the matching firmware")
        assert link.connected is False
        assert link.fd is None
        assert unit.logger.messages == [("warning", f"Turtle_1: closed {path}: {msg}")]
        assert unit.link_down == [(msg, False)]

    def test_no_answer(self, pty):
        link, board, unit, reactor, path = pty
        board.handler = lambda msg: []
        assert link.open(path)
        reactor.run_callbacks()
        assert link.fd is None
        assert unit.logger.messages == [
            ("warning", f"Turtle_1: closed {path}: no answer to info (timeout)")]

    def test_settings_refused(self, pty):
        link, _, unit, reactor, path = pty
        unit.up_result = False
        assert link.open(path)
        reactor.run_callbacks()
        assert link.connected is False
        assert unit.link_down == [("settings were refused", True)]


class TestTurtleLinkHubAuth:
    BOOT = "hold the hub's BOOT button for 10 s to clear it"

    def test_answers_the_challenge(self, tcp):
        link, unit, reactor, url, accept = tcp
        unit.saved_key = HUB_KEY
        board = tcp_handshake(link, reactor, url, accept, key=HUB_KEY)
        assert link.connected is True
        # Byte for byte what the hub's firmware reads (no spaces).
        assert board.raw[0] == f'{{"cmd":"auth","mac":"{HUB_MAC}"}}'.encode()
        assert board.received[1]["cmd"] == "info"
        assert unit.key_calls == [False]
        assert unit.logger.messages == []
        assert unit.link_up == [link.info]

    def test_tcp_key_answers_the_challenge(self, tcp):
        link, unit, reactor, url, accept = tcp
        unit.tcp_key = HUB_KEY
        board = tcp_handshake(link, reactor, url, accept, key=HUB_KEY)
        assert link.connected is True
        assert board.received[0] == {"cmd": "auth", "mac": HUB_MAC}
        assert unit.logger.messages == []

    def test_claims_an_open_hub(self, tcp):
        link, unit, reactor, url, accept = tcp
        board = tcp_handshake(link, reactor, url, accept)
        assert link.connected is True
        # The hub's firmware takes a claim only in exactly this form.
        assert board.raw[0] == b'{"cmd":"linkkey","key":"' + b"ab" * 16 + b'"}'
        assert board.received[1]["cmd"] == "info"
        assert board.hub_key == "ab" * 16
        assert unit.key_calls == [True]
        assert unit.logger.messages == [
            ("info", f"Turtle_1: link key stored on the hub at {url}")]

    def test_open_hub_with_no_key_to_give(self, tcp):
        link, unit, reactor, url, accept = tcp
        unit.new_key = None
        board = tcp_handshake(link, reactor, url, accept)
        board.drain()
        assert link.fd is None
        assert board.received == []
        assert unit.link_down == [("no link key to give the hub", False)]
        assert unit.logger.messages == [
            ("warning", f"Turtle_1: closed {url}: no link key to give the hub")]

    def test_keyed_hub_this_printer_has_no_key_for(self, tcp):
        link, unit, reactor, url, accept = tcp
        board = tcp_handshake(link, reactor, url, accept, key=HUB_KEY)
        board.drain()
        reason = f"the hub holds a link key this printer doesn't have; {self.BOOT}"
        assert link.fd is None
        assert board.received == []
        assert unit.key_calls == [False]
        assert unit.link_down == [(reason, False)]
        assert unit.logger.messages == [("warning", f"Turtle_1: closed {url}: {reason}")]

    def test_wrong_key_refused(self, tcp):
        link, unit, reactor, url, accept = tcp
        unit.saved_key = "not-the-hubs-key"
        board = tcp_handshake(link, reactor, url, accept, key=HUB_KEY)
        board.drain()
        reason = f"the hub refused the link key; {self.BOOT}"
        assert link.fd is None
        assert [m["cmd"] for m in board.received] == ["auth"]
        assert unit.link_down == [(reason, False)]
        assert unit.logger.messages == [("warning", f"Turtle_1: closed {url}: {reason}")]

    def test_no_answer_to_the_key(self, tcp):
        link, unit, reactor, url, accept = tcp
        unit.saved_key = HUB_KEY
        assert link.open(url)
        board = make_hub(accept(), HUB_KEY)
        board.handler = lambda msg: []
        wait_connected(link)
        link._connect_pending(101.0)
        start = reactor.now
        reactor.run_callbacks()
        assert reactor.now >= start + TurtleLink.AUTH_REPLY_TIME
        assert link.fd is None
        assert unit.logger.messages == [
            ("warning", f"Turtle_1: closed {url}: no answer from the hub to the link key")]

    def test_bad_challenge(self, tcp):
        link, unit, reactor, url, accept = tcp
        unit.saved_key = HUB_KEY
        assert link.open(url)
        accept().write({"evt": "auth", "nonce": "zz"})
        wait_connected(link)
        link._connect_pending(101.0)
        reactor.run_callbacks()
        assert link.fd is None
        assert unit.logger.messages == [
            ("warning", f"Turtle_1: closed {url}: the hub sent a bad challenge")]

    def test_unknown_greeting(self, tcp):
        link, unit, reactor, url, accept = tcp
        assert link.open(url)
        accept().write({"evt": "auth", "open": 0})
        wait_connected(link)
        link._connect_pending(101.0)
        reactor.run_callbacks()
        reason = "the hub sent an unknown greeting {'evt': 'auth', 'open': 0}"
        assert link.fd is None
        assert unit.key_calls == []
        assert unit.logger.messages == [("warning", f"Turtle_1: closed {url}: {reason}")]

    def test_no_greeting(self, tcp):
        link, unit, reactor, url, accept = tcp
        start = reactor.now
        board = tcp_handshake(link, reactor, url, accept, hub=False)
        assert reactor.now >= start + TurtleLink.GREETING_TIME
        assert link.connected is True
        assert board.received[0]["cmd"] == "info"
        assert unit.key_calls == []
        assert unit.logger.messages == []

    def test_no_greeting_with_tcp_key_warns_once(self, tcp):
        link, unit, reactor, url, accept = tcp
        unit.tcp_key = HUB_KEY
        tcp_handshake(link, reactor, url, accept, hub=False)
        assert link.connected is True
        link._hub_auth()
        msg = f"Turtle_1: {url} asked for no link key, so tcp_key is not used"
        assert unit.logger.messages == [("warning", msg)]
        assert unit.key_calls == []

    def test_hangs_up_before_the_greeting(self, tcp):
        link, unit, reactor, url, accept = tcp
        assert link.open(url)
        board = accept()
        board.sock.shutdown(socket.SHUT_RDWR)
        wait_connected(link)
        link._connect_pending(101.0)
        reactor.run_callbacks()
        assert link.fd is None
        assert unit.key_calls == []
        assert unit.link_down == [("port closed", False)]

    def test_hangs_up_after_the_answer(self, tcp):
        link, unit, reactor, url, accept = tcp
        unit.saved_key = HUB_KEY
        assert link.open(url)
        board = make_hub(accept(), HUB_KEY)
        board.handler = lambda msg: board.sock.shutdown(socket.SHUT_RDWR)
        wait_connected(link)
        link._connect_pending(101.0)
        reactor.run_callbacks()
        assert link.fd is None
        assert [m["cmd"] for m in board.received] == ["auth"]
        assert unit.link_down == [("port closed", False)]

    def test_serial_port_has_no_hub(self, pty):
        link, board, unit, reactor, path = pty
        connect(link, reactor, path)
        assert board.received[0]["cmd"] == "info"
        assert unit.key_calls == []

    def test_auth_line_already_read(self, pty):
        link, _, _, reactor, path = pty
        assert link.open(path)
        link._handle_line(b'{"evt":"auth","ok":1}', 1.0)
        start = reactor.now
        assert link._auth_line(1.0) == {"evt": "auth", "ok": 1}
        assert reactor.now == start
        assert link._auth_wait is None

    def test_auth_line_on_a_closed_link(self, pty):
        link, _, _, reactor, _ = pty
        start = reactor.now
        assert link._auth_line(1.0) is None
        assert reactor.now == start

    def test_open_drops_old_auth_lines(self, pty):
        link, _, _, _, path = pty
        link._auth_lines = [{"evt": "auth", "ok": 1}]
        assert link.open(path)
        assert link._auth_lines == []


class TestTurtleLinkClose:
    def test_not_open(self, pty):
        link, _, unit, _, _ = pty
        link.close("nothing")
        assert link.last_error == ""
        assert unit.link_down == []

    def test_fails_waiting_requests_and_events(self, pty):
        link, _, unit, reactor, path = pty
        connect(link, reactor, path)
        pending = reactor.completion()
        link._pending[99] = pending
        waiter = link.expect(lambda m: True)
        link.close("unplugged")
        assert pending.result == {"ok": False, "err": "disconnected"}
        assert waiter.result == {"ev": "link_down"}
        assert link._pending == {} and link._waiters == []
        assert link.connected is False
        assert link.fd_handle is None
        assert unit.link_down == [("unplugged", True)]
        # A drop of a working link: look for the board again at once.
        assert link._dropped_at == reactor.now
        assert link.timer["when"] == reactor.NOW

    def test_quiet_close_of_a_working_link(self, pty):
        link, _, _, reactor, path = pty
        connect(link, reactor, path)
        link.timer["when"] = reactor.NEVER
        link.close("klipper stopping", notify=False)
        assert link._dropped_at is None
        assert link.timer["when"] == reactor.NEVER

    def test_same_reason_logged_once(self, pty):
        link, _, unit, _, path = pty
        link.last_error = "unplugged"
        assert link.open(path)
        link.close("unplugged")
        assert unit.logger.messages == []
        assert unit.link_down == [("unplugged", False)]
        # Never answered: no fast retry.
        assert link._dropped_at is None

    def test_no_fd_handler(self, pty):
        link, _, _, reactor, path = pty
        assert link.open(path)
        fd = link.fd
        link.fd_handle = None
        link.close("x", notify=False)
        assert fd in reactor.fds
        assert link.fd is None

    def test_closes_the_socket(self, tcp):
        link, _, reactor, url, _ = tcp
        assert link.open(url)
        sock = link.sock
        link.close("x", notify=False)
        assert sock.fileno() == -1
        assert link.sock is None and link.fd is None
        assert link.connecting is False
        assert reactor.fds == {}

    def test_os_close_error_ignored(self, pty):
        link, _, _, _, path = pty
        assert link.open(path)
        fd = link.fd
        with patch.object(mod.os, "close", side_effect=OSError("gone")):
            link.close("x", notify=False)
        os.close(fd)
        assert link.fd is None

    def test_ends_a_wait_for_the_hub(self, pty):
        link, _, _, reactor, path = pty
        assert link.open(path)
        link._auth_wait = waiting = reactor.completion()
        link.close("x", notify=False)
        assert waiting.done is True and waiting.result is None


class TestTurtleLinkReadCb:
    def test_nothing_to_read(self, pty):
        link, _, unit, _, path = pty
        assert link.open(path)
        link._read_cb(101.0)
        assert link.fd is not None
        assert link._last_rx == 100.0

    def test_ready_but_would_block(self, pty):
        link, board, unit, _, path = pty
        assert link.open(path)
        board.write(b"x")
        wait_readable(link)
        with patch.object(mod.os, "read", side_effect=BlockingIOError()):
            link._read_cb(101.0)
        assert link.fd is not None
        assert link._last_rx == 100.0

    def test_read_error_closes(self, pty):
        link, board, unit, _, path = pty
        assert link.open(path)
        board.write(b"x")
        wait_readable(link)
        with patch.object(mod.os, "read", side_effect=OSError("boom")):
            link._read_cb(101.0)
        assert link.fd is None
        assert unit.logger.messages == [("warning", f"Turtle_1: closed {path}: read failed: boom")]

    def test_eof_closes(self, pty):
        link, board, unit, _, path = pty
        assert link.open(path)
        board.write(b"x")
        wait_readable(link)
        with patch.object(mod.os, "read", return_value=b""):
            link._read_cb(101.0)
        assert link.last_error == "port closed"

    def test_lines_and_overflow(self, pty):
        link, board, unit, _, path = pty
        assert link.open(path)
        board.write(b"x")
        wait_readable(link)
        data = [b'{"ev":"log","msg":"hi"}\n' + b"x" * 4096, b"x" * 4096]
        with patch.object(mod.os, "read", side_effect=data):
            link._read_cb(101.0)
            assert link._rx == b"x" * 4096
            link._read_cb(101.0)
        assert unit.events == [{"ev": "log", "msg": "hi"}]
        assert link._rx == b""
        assert link._last_rx == 101.0

    def test_flushes_pending_tx(self, pty):
        link, board, _, _, path = pty
        assert link.open(path)
        board.write(b"\n")
        wait_readable(link)
        link._tx = b'{"cmd":"ping"}\n'
        link._read_cb(101.0)
        board.drain()
        assert link._tx == b""
        assert board.received == [{"cmd": "ping"}]


class TestTurtleLinkHandleLine:
    def test_junk_ignored(self, pty):
        link, _, unit, _, _ = pty
        link._handle_line(b"AT", 1.0)
        link._handle_line(b"{bad", 1.0)
        assert unit.events == []

    def test_event_to_matching_waiter_and_unit(self, pty):
        link, _, unit, reactor, path = pty
        assert link.open(path)
        hit = link.expect(lambda m: m.get("lane") == 2)
        miss = link.expect(lambda m: m.get("lane") == 3)
        link._handle_line(b'{"ev":"move_done","lane":2}', 1.0)
        assert hit.result == {"ev": "move_done", "lane": 2}
        assert miss.done is False
        assert link._waiters == [(link._waiters[0][0], miss)]
        assert unit.events == [{"ev": "move_done", "lane": 2}]

    def test_hub_auth_line(self, pty):
        link, _, unit, reactor, path = pty
        assert link.open(path)
        waiter = link.expect(lambda m: True)
        link._auth_wait = waiting = reactor.completion()
        link._handle_line(b'{"evt":"auth","ok":1}', 1.0)
        assert waiting.result == {"evt": "auth", "ok": 1}
        assert link._auth_lines == [{"evt": "auth", "ok": 1}]
        assert waiter.done is False
        assert unit.events == []

    def test_reply_by_id(self, pty):
        link, *_ = pty
        reactor = link.reactor
        completion = reactor.completion()
        link._pending[5] = completion
        link._handle_line(b'{"id":7,"ok":true}', 1.0)
        assert completion.done is False
        link._handle_line(b'{"id":5,"ok":true}', 1.0)
        assert completion.result == {"id": 5, "ok": True}
        assert link._pending == {}


class TestTurtleLinkWrite:
    def test_not_open(self, pty):
        link, *_ = pty
        link._write(b"abc")
        assert link._tx == b""

    def test_keeps_what_the_kernel_did_not_take(self, pty):
        link, _, _, _, path = pty
        assert link.open(path)
        with patch.object(mod.os, "write", side_effect=[2, BlockingIOError()]):
            link._write(b"abcdef")
        assert link._tx == b"cdef"

    def test_write_error_closes(self, pty):
        link, _, unit, _, path = pty
        assert link.open(path)
        with patch.object(mod.os, "write", side_effect=OSError("io")):
            link._write(b"abc")
        assert link.fd is None
        assert link.last_error == "write failed: io"


class TestTurtleLinkSend:
    def test_not_open(self, pty):
        link, *_ = pty
        link.send("ping")
        assert link._next_id == 1

    def test_numbers_each_line(self, pty):
        link, board, _, _, path = pty
        assert link.open(path)
        link.send("ping")
        link.send("e", lane=2, pos=1.25)
        board.drain()
        assert board.received == [{"id": 1, "cmd": "ping"},
                                  {"id": 2, "cmd": "e", "lane": 2, "pos": 1.25}]
        assert link._next_id == 3

    def test_too_long_not_sent(self, pty):
        link, board, unit, _, path = pty
        assert link.open(path)
        link.send("cfg", big="x" * 300)
        link.send("ping")
        board.drain()
        assert board.received == [{"id": 2, "cmd": "ping"}]
        # '{"id":1,"cmd":"cfg","big":"' (27) + 300 + '"}\n' (3)
        assert unit.logger.messages == [
            ("error", "Turtle_1: cfg line is 330 bytes, over the board's 255; not sent")]


class TestTurtleLinkRequest:
    def test_not_open(self, pty):
        link, *_ = pty
        assert link.request("status") == {"ok": False, "err": "not connected"}

    def test_reply(self, pty):
        link, _, _, reactor, path = pty
        connect(link, reactor, path)
        assert link.request("status") == {"id": 2, "ok": True}
        assert link._pending == {}

    def test_timeout(self, pty):
        link, board, _, reactor, path = pty
        connect(link, reactor, path)
        board.handler = lambda msg: []
        start = reactor.now
        assert link.request("status", timeout=0.2) == {"ok": False, "err": "timeout"}
        assert reactor.now >= start + 0.2
        assert link._pending == {}

    def test_too_long(self, pty):
        link, board, unit, reactor, path = pty
        connect(link, reactor, path)
        board.drain()
        sent = list(board.received)
        unit.logger.messages.clear()
        assert link.request("cfg", big="x" * 300) == {"ok": False, "err": "line too long"}
        assert link._pending == {}
        assert link._next_id == 3
        assert board.received == sent
        # '{"id":2,"cmd":"cfg","big":"' (27) + 300 + '"}\n' (3)
        assert unit.logger.messages == [
            ("error", "Turtle_1: cfg line is 330 bytes, over the board's 255; not sent")]


class TestTurtleLinkTooLong:
    def test_at_the_limit(self, pty):
        link, _, unit, _, _ = pty
        assert link._too_long("cfg", b"x" * LINE_MAX) is False
        assert unit.logger.messages == []

    def test_over_the_limit(self, pty):
        link, _, unit, _, _ = pty
        assert link._too_long("led", b"x" * 256) is True
        assert unit.logger.messages == [
            ("error", "Turtle_1: led line is 256 bytes, over the board's 255; not sent")]


class TestTurtleLinkExpect:
    def test_not_open(self, pty):
        link, *_ = pty
        assert link.expect(lambda m: True).result == {"ev": "link_down"}
        assert link._waiters == []

    def test_registers(self, pty):
        link, _, _, _, path = pty
        assert link.open(path)
        completion = link.expect(lambda m: True)
        assert link._waiters[0][1] is completion


class TestTurtleLinkWait:
    def test_event(self, pty):
        link, board, _, reactor, path = pty
        assert link.open(path)
        completion = link.expect(lambda m: m.get("ev") == "job_done")
        board.write({"ev": "job_done", "lane": 1})
        assert link.wait(completion, 1.0) == {"ev": "job_done", "lane": 1}

    def test_timeout_drops_the_waiter(self, pty):
        link, _, _, _, path = pty
        assert link.open(path)
        completion = link.expect(lambda m: False)
        assert link.wait(completion, 0.05) is None
        assert link._waiters == []


class TestTurtleLinkWaitUp:
    def test_already_up(self, pty):
        link, _, _, reactor, path = pty
        connect(link, reactor, path)
        start = reactor.now
        assert link.wait_up(1.0) is True
        assert reactor.now == start

    def test_comes_up(self, pty):
        link, _, _, reactor, _ = pty
        start = reactor.now

        def up():
            if reactor.now >= start + 0.2:
                link.connected = True
        reactor.pumps.append(up)
        assert link.wait_up(1.0) is True
        assert start + 0.2 <= reactor.now < start + 0.3

    def test_stays_down(self, pty):
        link, _, _, reactor, _ = pty
        start = reactor.now
        assert link.wait_up(0.5) is False
        assert reactor.now == pytest.approx(start + 0.5, abs=0.06)


# ── TurtleNeck ────────────────────────────────────────────────────────────────

@pytest.fixture
def neck():
    unit = MagicMock()
    unit.name = "Turtle_1"
    unit.assist_fault_mm = 80.0
    return TurtleNeck(unit), unit


class TestTurtleNeckInit:
    def test_fields(self, neck):
        tn, _ = neck
        assert tn.name == "Turtle_1_TurtleNeck"
        assert tn.type == "TurtleNeck"
        assert tn.advance_pin is None
        assert (tn.advance_state, tn.trailing_state, tn.enable) == (False, False, False)
        assert tn.fault_detection is True


class TestTurtleNeckEnableBuffer:
    def test_with_lane(self, neck):
        tn, unit = neck
        lane = MagicMock()
        tn.enable_buffer(lane)
        assert tn.enable is True and tn.current_lane is lane
        unit.buffer_changed.assert_called_once_with(lane, True)

    def test_keeps_current_lane(self, neck):
        tn, unit = neck
        lane = MagicMock()
        tn.current_lane = lane
        tn.enable_buffer()
        unit.buffer_changed.assert_called_once_with(lane, True)

    def test_no_lane(self, neck):
        tn, unit = neck
        tn.enable_buffer()
        assert tn.enable is True
        unit.buffer_changed.assert_not_called()


class TestTurtleNeckDisableBuffer:
    def test_with_lane(self, neck):
        tn, unit = neck
        lane = MagicMock()
        tn.enable = True
        tn.current_lane = lane
        tn.disable_buffer()
        assert tn.enable is False
        unit.buffer_changed.assert_called_once_with(lane, False)

    def test_no_lane(self, neck):
        tn, unit = neck
        tn.enable = True
        tn.disable_buffer()
        assert tn.enable is False
        unit.buffer_changed.assert_not_called()


class TestTurtleNeckBufferStatus:
    def test_unknown_until_a_switch(self, neck):
        tn, _ = neck
        assert tn.buffer_status() == "Unknown"

    @pytest.mark.parametrize("name,value,state", [("tn_adv", 1, "Trailing"),
                                                  ("tn_adv", 0, "Trailing"),
                                                  ("tn_trl", 1, "Advancing"),
                                                  ("tn_trl", 0, "Advancing")])
    def test_set_switch(self, neck, name, value, state):
        tn, _ = neck
        tn.set_switch(name, value)
        assert tn.buffer_status() == state
        assert tn.advance_state is (name == "tn_adv" and bool(value))
        assert tn.trailing_state is (name == "tn_trl" and bool(value))

    def test_holds_with_both_open(self, neck):
        tn, _ = neck
        tn.set_switch("tn_adv", 1)
        tn.set_switch("tn_adv", 0)
        assert tn.buffer_status() == "Trailing"
        tn.set_switch("tn_trl", 1)
        tn.set_switch("tn_trl", 0)
        assert tn.buffer_status() == "Advancing"
        assert (tn.advance_state, tn.trailing_state) == (False, False)

    @pytest.mark.parametrize("adv,trl,state", [(1, 0, "Trailing"),
                                               (1, 1, "Trailing"),
                                               (0, 1, "Advancing"),
                                               (0, 0, "Advancing")])
    def test_sync_switches(self, neck, adv, trl, state):
        tn, _ = neck
        tn.last_state = "Advancing"
        tn.sync_switches(adv, trl)
        assert tn.buffer_status() == state
        assert (tn.advance_state, tn.trailing_state) == (bool(adv), bool(trl))


class TestTurtleNeckFaultSensitivity:
    def test_disable(self, neck):
        tn, unit = neck
        tn.disable_fault_sensitivity()
        assert tn.fault_detection is False
        unit.send_fault_mm.assert_called_once_with(0.0)

    def test_restore(self, neck):
        tn, unit = neck
        tn.fault_detection = False
        tn.restore_fault_sensitivity()
        assert tn.fault_detection is True
        unit.send_fault_mm.assert_called_once_with(80.0)


class TestTurtleNeckNoOps:
    def test_board_does_these(self, neck):
        tn, unit = neck
        assert tn.update_filament_error_pos(1, x=2) is None
        assert tn.enable_advance_latch() is None
        assert tn.clear_advance_latch() is None
        assert unit.mock_calls == []


class TestTurtleNeckGetStatus:
    def test_with_lane(self, neck):
        tn, _ = neck
        lane = MagicMock()
        lane.name = "lane8"
        tn.current_lane = lane
        tn.set_switch("tn_trl", 1)
        assert tn.get_status() == {"state": "Advancing", "advance": False, "trailing": True,
                                   "enabled": False, "lane": "lane8", "fault_detection": True}

    def test_no_lane(self, neck):
        tn, _ = neck
        assert tn.get_status()["lane"] is None


# ── TurtleRotation and TurtleDrive ────────────────────────────────────────────

class TestTurtleRotationGetRotationDistance:
    def test_lane_settings_and_multiplier(self):
        unit = make_unit(lane_sections=[("lane8", {"unit": "Turtle_1:1",
                                                   "rotation_distance": 5.0,
                                                   "microsteps": 32})])
        lane = make_lane(unit)
        lane.drive_stepper.multiplier = 1.25
        assert lane.drive_stepper.rotation.get_rotation_distance() == (4.0, 6400)

    def test_unit_defaults(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.drive_stepper.multiplier = 0
        rotation = TurtleRotation(lane.drive_stepper)
        assert rotation.stepper is rotation
        assert rotation.get_rotation_distance() == (4.65, 3200)


class TestTurtleDriveInit:
    def test_fields(self):
        unit = make_unit()
        lane = make_lane(unit)
        drive = lane.drive_stepper
        assert drive.name == "lane8"
        assert drive.extruder_stepper is None
        assert drive._endstops == {"tool_start": None}
        assert drive.assist_move is lane.assist_move
        assert drive.multiplier == 1.0
        assert drive.rotation.drive is drive


class TestTurtleDriveMove:
    def test_delegates(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_move") as fw_move:
            lane.drive_stepper.move(10, 50, 400, True)
        fw_move.assert_called_once_with(lane, 10, 50, 400, True)


class TestTurtleDriveHomeTo:
    def test_delegates(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_home", return_value=(True, 5.0, False)) as fw_home:
            assert lane.drive_stepper.home_to("hub", 100, 50, 400, False, True, False) == (True, 5.0, False)
        fw_home.assert_called_once_with(lane, "hub", 100, 50, 400, False, True, False)


class TestTurtleDriveDoEnable:
    def test_sends(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.drive_stepper.do_enable(1)
        assert unit.link.sent == [("enable", {"lane": 1, "on": True})]


class TestTurtleDriveSyncToExtruder:
    def test_with_current(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "set_current") as cur, patch.object(unit, "set_sync") as sync:
            lane.drive_stepper.sync_to_extruder()
        cur.assert_called_once_with(lane, True)
        sync.assert_called_once_with(lane, True)

    def test_without_current(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "set_current") as cur, patch.object(unit, "set_sync"):
            lane.drive_stepper.sync_to_extruder(False)
        cur.assert_not_called()


class TestTurtleDriveUnsyncToExtruder:
    def test_with_current(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "set_current") as cur, patch.object(unit, "set_sync") as sync:
            lane.drive_stepper.unsync_to_extruder()
        sync.assert_called_once_with(lane, False)
        cur.assert_called_once_with(lane, False)

    def test_without_current(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "set_current") as cur, patch.object(unit, "set_sync"):
            lane.drive_stepper.unsync_to_extruder(False)
        cur.assert_not_called()


class TestTurtleDriveCurrents:
    def test_load_and_print(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "set_current") as cur:
            lane.drive_stepper.set_load_current()
            lane.drive_stepper.set_print_current()
        assert cur.call_args_list == [((lane, False),), ((lane, True),)]


class TestTurtleDriveUpdateRotationDistance:
    def test_stores(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.drive_stepper.update_rotation_distance(1.15)
        assert lane.drive_stepper.multiplier == 1.15

    def test_zero_is_one(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.drive_stepper.update_rotation_distance(0)
        assert lane.drive_stepper.multiplier == 1.0


# ── afcBoxTurtleUSB construction ──────────────────────────────────────────────

class TestInit:
    def test_defaults(self):
        unit = make_unit(fake_link=False)
        assert unit.type == "BoxTurtleUSB"
        assert unit.serial_port == "auto"
        assert unit.serial_uid is None
        assert unit.link_timeout == 2.0
        assert (unit.rotation_distance, unit.microsteps, unit.full_steps) == (4.65, 16, 200)
        assert unit.invert_dir is True
        assert unit.use_turtleneck is True
        assert (unit.multiplier_high, unit.multiplier_low) == (1.05, 0.95)
        assert unit.assist_fault_mm == 0.0
        assert unit.espool_cycle_time == 0.1
        assert unit.switch_invert == {}
        assert unit.switch_debounce_ms is None
        assert unit.led_chains == {1: (4, "GRBW"), 2: (0, "GRB"), 3: (0, "GRB"), 4: (0, "GRB")}
        assert isinstance(unit.link, TurtleLink)
        assert unit.link.port_cfg == "auto"
        assert unit.sync == {} and unit.buffer_on == {} and unit.assist_mode == {}
        assert unit.e_last == {} and unit.e_sent == {}
        handlers = unit.printer._event_handlers
        assert handlers["klippy:shutdown"] == [unit.link.stop]
        assert handlers["klippy:disconnect"] == [unit.link.stop]
        assert unit.reactor.timers[-1]["cb"] == unit._e_stream_cb

    def test_options(self):
        unit = make_unit({"serial_port": "/dev/ttyACM3", "filament_error_sensitivity": 0,
                          "turtleneck": False, "prep_invert": True, "tn_trailing_invert": False,
                          "switch_debounce_ms": 20, "rgb1_chain_count": 0,
                          "rgb2_chain_count": 64, "rgb4_color_order": "rgbw"})
        assert unit.serial_port == "/dev/ttyACM3"
        assert unit.assist_fault_mm == 0.0
        assert unit.use_turtleneck is False
        assert unit.switch_invert == {"prep1": True, "prep2": True, "prep3": True,
                                      "prep4": True, "tn_trl": False}
        assert unit.switch_debounce_ms == 20
        assert unit.led_chains == {1: (0, "GRBW"), 2: (64, "GRB"), 3: (0, "GRB"), 4: (0, "RGBW")}

    def test_bad_color_order(self):
        with pytest.raises(mod.error) as e:
            make_unit({"rgb3_color_order": "BGR"})
        assert str(e.value) == "rgb3_color_order BGR is not one of GRB, GRBW, RGB, RGBW"

    def test_tcp_port(self):
        unit = make_unit({"serial_port": "tcp://192.168.1.50:4001"}, fake_link=False)
        assert unit.serial_port == "tcp://192.168.1.50:4001"
        assert unit.link.port_cfg == "tcp://192.168.1.50:4001"

    def test_bad_tcp_port(self):
        with pytest.raises(mod.error) as e:
            make_unit({"serial_port": "tcp://hub.local:4001"})
        assert str(e.value) == "serial_port tcp://hub.local:4001 is not tcp://<ip address>[:<port>]"

    def test_tcp_without_port(self):
        unit = make_unit({"serial_port": "tcp://192.168.1.50"}, fake_link=False)
        assert unit.link.port_cfg == "tcp://192.168.1.50"
        assert unit.tcp_key is None

    def test_tcp_key(self):
        unit = make_unit({"tcp_key": HUB_KEY})
        assert unit.tcp_key == HUB_KEY

    @pytest.mark.parametrize("key", ["has space", "x" * 65, 'q"uote'])
    def test_bad_tcp_key(self, key):
        with pytest.raises(mod.error) as e:
            make_unit({"tcp_key": key})
        assert str(e.value) == ("tcp_key must be 1 to 64 characters with no spaces, "
                                "quotes or backslashes")

    def test_sensitivity_sets_fault_distance(self):
        unit = make_unit({"filament_error_sensitivity": 8})
        assert unit.assist_fault_mm == 30.0


class TestLinkKey:
    @pytest.fixture
    def unit(self, tmp_path):
        unit = make_unit()
        unit.afc.VarFile = str(tmp_path / "AFC.var")
        return unit

    def saved(self, unit):
        with open(unit.link_key_file()) as f:
            return json.load(f)

    def test_file_next_to_var_file(self, unit, tmp_path):
        assert unit.link_key_file() == str(tmp_path / "AFC.var.hub")

    def test_file_expands_home(self, unit):
        unit.afc.VarFile = "~/printer_data/config/AFC/AFC.var"
        assert unit.link_key_file() == os.path.expanduser(
            "~/printer_data/config/AFC/AFC.var.hub")

    def test_tcp_key_wins(self, unit):
        unit.tcp_key = HUB_KEY
        assert unit.link_key(make=True) == HUB_KEY
        assert not os.path.exists(unit.link_key_file())

    def test_none_saved(self, unit):
        assert unit.link_key(make=False) is None
        assert not os.path.exists(unit.link_key_file())

    def test_makes_and_saves(self, unit):
        with open(unit.link_key_file(), "w") as f:
            json.dump({"Turtle_2": "other"}, f)
        with patch.object(hub_link.secrets, "token_hex", return_value=HUB_KEY) as token:
            assert unit.link_key(make=True) == HUB_KEY
        token.assert_called_once_with(16)
        assert self.saved(unit) == {"Turtle_2": "other", "Turtle_1": HUB_KEY}
        assert not os.path.exists(f"{unit.link_key_file()}.tmp")
        assert unit.link_key(make=False) == HUB_KEY

    def test_made_key_is_one_the_hub_takes(self, unit):
        key = unit.link_key(make=True)
        assert check_link_key(key) and len(key) == 32

    def test_reads_the_saved_key(self, unit):
        with open(unit.link_key_file(), "w") as f:
            json.dump({"Turtle_1": HUB_KEY}, f)
        with patch.object(hub_link.secrets, "token_hex") as token:
            assert unit.link_key(make=True) == HUB_KEY
        token.assert_not_called()

    @pytest.mark.parametrize("text", ["not json", "[1, 2]", '{"Turtle_1": 5}',
                                      '{"Turtle_1": "has space"}'])
    def test_unusable_file_makes_a_new_key(self, unit, text):
        with open(unit.link_key_file(), "w") as f:
            f.write(text)
        with patch.object(hub_link.secrets, "token_hex", return_value=HUB_KEY):
            assert unit.link_key(make=True) == HUB_KEY
        assert self.saved(unit)["Turtle_1"] == HUB_KEY

    def test_cannot_save(self, unit, tmp_path):
        unit.afc.VarFile = str(tmp_path / "missing" / "AFC.var")
        path = unit.link_key_file()
        assert unit.link_key(make=True) is None
        assert unit.logger.messages[-1] == (
            "warning", f"Turtle_1: cannot save the hub's link key in {path}: "
                       f"[Errno 2] No such file or directory: '{path}.tmp'")


class TestReadLaneSettings:
    def test_only_this_units_lanes(self):
        unit = make_unit({"rgb2_chain_count": 64}, lane_sections=[
            ("lane8", {"unit": "Turtle_1:1", "rotation_distance": 4.8,
                       "turtle_led": "RGB2:25-32"}),
            ("lane1", {"unit": "Other:1"})])
        assert list(unit.lane_settings) == ["lane8"]
        assert unit.lane_settings["lane8"] == {
            "rotation_distance": 4.8, "microsteps": 16, "invert_dir": True,
            "run_current": 0.8, "hold_current": None, "print_current": None,
            "turtle_led": [(2, 25, 32)]}


class TestLaneLeds:
    def _leds(self, lane_values, unit_values=None):
        unit = make_unit(unit_values, lane_sections=[("lane9", lane_values)])
        return unit.lane_settings["lane9"]["turtle_led"]

    def test_stock_default_is_rgb1(self):
        assert self._leds({"unit": "Turtle_1:3"}) == [(1, 3, 3)]

    @pytest.mark.parametrize("unit_values", [{"rgb1_chain_count": 2}, {"rgb1_chain_count": 0}])
    def test_no_default_past_rgb1(self, unit_values):
        assert self._leds({"unit": "Turtle_1:3"}, unit_values) == []

    def test_no_default_without_a_lane_number(self):
        assert self._leds({"unit": "Turtle_1"}) == []

    def test_bad_spec_names_the_lane(self):
        with pytest.raises(mod.error) as e:
            self._leds({"unit": "Turtle_1:2", "turtle_led": "25-32"})
        assert str(e.value) == "lane9: turtle_led 25-32 is not RGB<1-4>:<first>[-<last>]"


class TestHandleConnect:
    def _connect(self, unit):
        lane = make_lane(unit, add=False)
        unit.lanes["lane8"] = lane
        with patch.object(afcBoxTurtle, "handle_connect") as base:
            unit.handle_connect()
        base.assert_called_once_with()
        return lane

    def test_turtleneck_and_drives(self):
        unit = make_unit()
        lane = self._connect(unit)
        assert isinstance(unit.buffer_obj, TurtleNeck)
        assert unit.printer.lookup_object("AFC_buffer Turtle_1_TurtleNeck") is unit.buffer_obj
        assert isinstance(lane.drive_stepper, TurtleDrive)
        assert lane.extruder_stepper is lane.drive_stepper.rotation
        assert lane._load_state is False
        assert unit.sync == {"lane8": False} and unit.buffer_on == {"lane8": False}
        assert unit.mcu is not None
        assert unit.link.started is True

    def test_turtleneck_name_taken(self):
        unit = make_unit()
        other = object()
        unit.printer._objects["AFC_buffer Turtle_1_TurtleNeck"] = other
        self._connect(unit)
        assert isinstance(unit.buffer_obj, TurtleNeck)
        assert unit.printer.lookup_object("AFC_buffer Turtle_1_TurtleNeck") is other

    def test_turtleneck_off(self):
        unit = make_unit({"turtleneck": False})
        self._connect(unit)
        assert unit.buffer_obj is None
        assert unit.printer.lookup_object("AFC_buffer Turtle_1_TurtleNeck") is None

    def test_named_buffer(self):
        unit = make_unit({"buffer": "PSF_buffer3"})
        self._connect(unit)
        assert not isinstance(unit.buffer_obj, TurtleNeck)


class TestLaneByIndex:
    def test_found_and_missing(self):
        unit = make_unit()
        lane = make_lane(unit, index=2)
        assert unit.lane_by_index(2) is lane
        assert unit.lane_by_index(3) is None


class TestLaneCfg:
    def test_fields(self):
        unit = make_unit(lane_sections=[("lane8", {"unit": "Turtle_1:1", "rotation_distance": 5.0,
                                                   "invert_dir": False, "run_current": 0.6,
                                                   "hold_current": 0.3})])
        lane = make_lane(unit)
        cfg = unit._lane_cfg(lane)
        assert cfg == {"lane": 1, "steps_per_mm": 640.0, "dir_invert": False,
                       "run_current": 0.6, "hold_current": 0.3, "sense_resistor": 0.11,
                       "microsteps": 16, "stealthchop": False, "sgthrs": 0,
                       "espool_pulse_mm": 150.0, "espool_pulse_ms": 570}

    def test_hold_defaults_to_run(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._lane_cfg(lane)["hold_current"] == 0.8


class TestPulseSettings:
    def test_from_weight(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit.pulse_settings(lane) == (150.0, 570)
        lane.espooler.espooler_values.calculate_cruise_time.assert_called_once_with(50.0)

    def test_no_values(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.espooler.espooler_values = None
        assert unit.pulse_settings(lane) == (0.0, 0)

    def test_bad_delta(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.espooler.espooler_values.delta_movement = None
        assert unit.pulse_settings(lane) == (0.0, 0)

    def test_assist_off(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.espooler.enable_assist = False
        assert unit.pulse_settings(lane) == (150.0, 0)

    def test_no_weight_limit(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.espooler.enable_assist_weight = None
        assert unit.pulse_settings(lane) == (150.0, 0)

    def test_heavy_spool(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.weight = 100.0
        assert unit.pulse_settings(lane) == (150.0, 0)

    def test_cruise_error(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.espooler.espooler_values.calculate_cruise_time.side_effect = ZeroDivisionError
        assert unit.pulse_settings(lane) == (150.0, 0)

    def test_capped(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.espooler.espooler_values.calculate_cruise_time.return_value = 9.0
        assert unit.pulse_settings(lane) == (150.0, 5000)

    def test_never_negative(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.espooler.espooler_values.calculate_cruise_time.return_value = -1.0
        assert unit.pulse_settings(lane) == (150.0, 0)


# ── link up and down ──────────────────────────────────────────────────────────

class TestHandleLinkUp:
    def _unit(self, **values):
        unit = make_unit(values)
        lane = make_lane(unit)
        unit.link.replies["status"] = {"ok": True, "sw": {"prep1": 1, "load1": 0}}
        return unit, lane

    def test_sends_every_setting(self):
        unit, lane = self._unit(prep_invert=True, switch_debounce_ms=15, rgb2_chain_count=64)
        assert unit.handle_link_up({"fw": "fw1", "reset": "power"}) is True
        cmds = [c for c, _ in unit.link.requests]
        assert cmds == ["cfg", "cfg", "lane_cfg", "sw_cfg", "sw_cfg", "sw_cfg", "sw_cfg",
                        "led_cfg", "led_cfg", "led_cfg", "led_cfg", "status"]
        # Through job_tries the first line is 251 bytes with a 7 digit id;
        # job_backoff_mm (24 more) would make it 275.
        assert unit.link.requests[0][1] == {
            "link_timeout_ms": 2000, "assist_high": 1.05, "assist_low": 0.95,
            "assist_fault_mm": 0.0, "assist_fallback_speed": 30.0,
            "assist_max_speed": 80.0, "espool_kick_ms": 70, "espool_brake_ms": 200,
            "espool_cycle_us": 100000, "job_tries": 2}
        assert unit.link.requests[1][1] == {"job_backoff_mm": 20.0, "job_snag_backoff_mm": 30.0,
                                            "job_repush_mm": 10.0}
        assert unit.link.requests[3][1] == {"sw": "prep1", "invert": True, "debounce_ms": 15}
        # Every header, an unused one with no LEDs so it goes dark.
        assert [f for c, f in unit.link.requests if c == "led_cfg"] == [
            {"strip": 1, "count": 4, "order": "GRBW"}, {"strip": 2, "count": 64, "order": "GRB"},
            {"strip": 3, "count": 0, "order": "GRB"}, {"strip": 4, "count": 0, "order": "GRB"}]
        assert unit.switches == {"prep1": 1, "load1": 0}
        assert lane.prep_state is True and lane._load_state is False
        assert unit.ever_connected is True
        assert unit.logger.messages == [
            ("info", "Turtle_1: BoxTurtle firmware fw1 on "
                     "/dev/serial/by-id/usb-AFC_BoxTurtle_UID1-if00 (reset power)")]

    def test_no_debounce_key(self):
        unit, _ = self._unit(hub_invert=False)
        unit.handle_link_up({})
        assert unit.link.requests[3][1] == {"sw": "hub", "invert": False}

    def test_refused(self):
        unit, _ = self._unit()
        unit.link.replies["cfg"] = {"ok": False, "err": "bad key"}
        assert unit.handle_link_up({"fw": "fw1", "reset": "power"}) is False
        assert unit.logger.messages == [
            ("info", "Turtle_1: BoxTurtle firmware fw1 on "
                     "/dev/serial/by-id/usb-AFC_BoxTurtle_UID1-if00 (reset power)"),
            ("error", "Turtle_1: board refused settings: cfg (bad key), cfg (bad key)")]
        assert "status" not in [c for c, _ in unit.link.requests]
        assert unit.ever_connected is False

    def test_status_failed(self):
        unit, _ = self._unit()
        unit.link.replies["status"] = {"ok": False, "err": "timeout"}
        assert unit.handle_link_up({}) is False
        assert unit.ever_connected is False

    def test_reconnect_resumes(self):
        unit, _ = self._unit()
        unit.ever_connected = True
        with patch.object(unit, "_resume_after_reconnect") as resume:
            unit.handle_link_up({})
        resume.assert_called_once_with()

    def test_first_connect_does_not_resume(self):
        unit, _ = self._unit()
        with patch.object(unit, "_resume_after_reconnect") as resume:
            unit.handle_link_up({})
        resume.assert_not_called()


class TestApplySwitches:
    def test_lanes_hub_and_turtleneck(self):
        unit = make_unit()
        lane = make_lane(unit)
        hub = MagicMock()
        unit.hub_obj = hub
        unit.buffer_obj = TurtleNeck(unit)
        unit.switches = {"prep1": 0, "load1": 1, "hub": 1, "tn_adv": 1, "tn_trl": 0}
        unit._apply_switches()
        assert lane.prep_state is False and lane._load_state is True
        hub.switch_pin_callback.assert_called_once_with(100.0, 1)
        assert unit.buffer_obj.advance_state is True
        assert unit.buffer_obj.trailing_state is False
        assert unit.buffer_obj.buffer_status() == "Trailing"

    def test_hub_without_callback(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.hub_obj = object()
        unit.switches = {"prep1": 1}
        unit._apply_switches()
        assert lane.prep_state is True

    def test_no_hub_and_external_buffer(self):
        unit = make_unit()
        make_lane(unit).hub_obj = None
        buffer = ext_buffer()
        buffer.advance_state = "untouched"
        unit.buffer_obj = buffer
        unit.switches = {"tn_adv": 1}
        unit._apply_switches()
        assert buffer.advance_state == "untouched"


class TestHub:
    def test_unit_hub(self):
        unit = make_unit()
        unit.hub_obj = "hub"
        assert unit._hub() == "hub"

    def test_lane_hub(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._hub() is lane.hub_obj

    def test_none(self):
        unit = make_unit()
        make_lane(unit).hub_obj = None
        assert unit._hub() is None

    def test_lane_hub_without_callback(self):
        unit = make_unit()
        make_lane(unit).hub_obj = object()
        assert unit._hub() is None


class TestResumeAfterReconnect:
    def test_rearms_and_relights(self):
        unit = make_unit()
        synced = make_lane(unit)
        idle = make_lane(unit, "lane9", 2)
        unit.sync["lane8"] = True
        unit.led_colors["lane9"] = "0,1,0,0"
        with patch.object(unit, "_send_assist") as assist, \
                patch.object(unit, "board_led") as led, \
                patch.object(unit, "_update_e_timer") as timer:
            unit._resume_after_reconnect()
        assist.assert_called_once_with(synced)
        led.assert_called_once_with(idle, "0,1,0,0")
        timer.assert_called_once_with()

    def test_lane_in_a_move_waits(self):
        unit = make_unit()
        make_lane(unit)
        unit.sync["lane8"] = True
        unit.moving.add("lane8")
        with patch.object(unit, "_send_assist") as assist, \
                patch.object(unit, "_update_e_timer"):
            unit._resume_after_reconnect()
        assist.assert_not_called()


class TestHandleLinkDown:
    def test_never_connected(self):
        unit = make_unit()
        unit.handle_link_down("gone", False)
        assert unit.reactor.callbacks == []

    def test_connected(self):
        unit = make_unit()
        with patch.object(unit, "_link_lost") as lost:
            unit.handle_link_down("gone", True)
            assert unit.reactor.callback_times == [unit.reactor.now + unit.RECONNECT_WAIT]
            unit.reactor.run_callbacks()
        lost.assert_called_once_with("gone")


class TestLinkLost:
    def test_came_back(self):
        unit = make_unit()
        make_lane(unit)
        unit.afc.current = "lane8"
        unit.afc.function.is_printing.return_value = True
        unit._link_lost("port closed")
        unit.afc.error.AFC_error.assert_not_called()
        assert unit.logger.messages[-1] == (
            "info", "Turtle_1: BoxTurtle USB link dropped (port closed) and came back")

    def test_printing_on_this_unit(self):
        unit = make_unit()
        make_lane(unit)
        unit.link.connected = False
        unit.afc.current = "lane8"
        unit.afc.function.is_printing.return_value = True
        unit._link_lost("port closed")
        unit.afc.error.AFC_error.assert_called_once_with(
            "Turtle_1: BoxTurtle USB link lost (port closed), pausing. Plug it back in and resume.",
            pause=True)

    def test_other_unit(self):
        unit = make_unit()
        make_lane(unit)
        unit.link.connected = False
        unit.afc.current = "lane1"
        unit.afc.function.is_printing.return_value = True
        unit._link_lost("port closed")
        unit.afc.error.AFC_error.assert_not_called()

    def test_not_printing(self):
        unit = make_unit()
        make_lane(unit)
        unit.link.connected = False
        unit.afc.current = "lane8"
        unit._link_lost("port closed")
        unit.afc.error.AFC_error.assert_not_called()


# ── board events ──────────────────────────────────────────────────────────────

class TestHandleEvent:
    def test_switch(self):
        unit = make_unit()
        with patch.object(unit, "switch_changed") as changed:
            unit.handle_event({"ev": "sw", "sw": "load1", "v": 1}, 5.0)
            assert unit.switches == {"load1": 1}
            unit.reactor.run_callbacks()
        changed.assert_called_once_with("load1", 1, 100.0)

    def test_fault(self):
        unit = make_unit()
        msg = {"ev": "fault", "code": "clog", "lane": 1}
        with patch.object(unit, "fault") as fault:
            unit.handle_event(msg, 5.0)
            unit.reactor.run_callbacks()
        fault.assert_called_once_with(msg, 100.0)

    def test_job_recovering(self):
        unit = make_unit()
        unit.handle_event({"ev": "job", "state": "recovering", "lane": 2, "job": "feed",
                           "try": 1, "why": "snag"}, 5.0)
        assert unit.logger.messages == [("info", "Turtle_1 lane 2: feed retry 1 (snag)")]

    def test_job_other_state(self):
        unit = make_unit()
        unit.handle_event({"ev": "job", "state": "running"}, 5.0)
        assert unit.logger.messages == []

    def test_log(self):
        unit = make_unit()
        unit.handle_event({"ev": "log", "msg": "tmc ok"}, 5.0)
        assert unit.logger.messages == [("info", "Turtle_1: tmc ok")]

    def test_unknown(self):
        unit = make_unit()
        unit.handle_event({"ev": "move_done"}, 5.0)
        assert unit.reactor.callbacks == [] and unit.logger.messages == []


class TestSwitchChanged:
    def test_load(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.switch_changed("load1", 1, 5.0)
        lane.load_callback.assert_called_once_with(5.0, True)

    def test_prep_inserted(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.switch_changed("prep1", 1, 5.0)
        lane.prep_callback.assert_called_once_with(5.0, 1)

    def test_prep_removed(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "prep_removed") as removed:
            unit.switch_changed("prep1", 0, 5.0)
        removed.assert_called_once_with(lane, 5.0)

    def test_unknown_lane(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.switch_changed("load3", 1, 5.0)
        lane.load_callback.assert_not_called()

    def _hub_unit(self, current="lane8", runout=True, printing_lane=True):
        unit = make_unit()
        lane = make_lane(unit)
        hub = MagicMock()
        hub.name = "Turtle_1"
        hub.enable_runout = runout
        unit.hub_obj = hub
        unit.afc.current = current
        unit.afc.lanes = {"lane8": lane} if printing_lane else {}
        return unit, lane, hub

    def test_hub_runout(self):
        unit, lane, hub = self._hub_unit()
        unit.switch_changed("hub", 0, 5.0)
        hub.switch_pin_callback.assert_called_once_with(5.0, 0)
        lane.handle_hub_runout.assert_called_once_with(sensor="Turtle_1")

    def test_hub_triggered_is_not_runout(self):
        unit, lane, _ = self._hub_unit()
        unit.switch_changed("hub", 1, 5.0)
        lane.handle_hub_runout.assert_not_called()

    def test_hub_no_current(self):
        unit, lane, _ = self._hub_unit(current=None)
        unit.switch_changed("hub", 0, 5.0)
        lane.handle_hub_runout.assert_not_called()

    def test_hub_current_on_other_unit(self):
        unit, lane, _ = self._hub_unit()
        other = MagicMock()
        other.name = "lane1"
        unit.afc.lanes = {"lane8": other}
        unit.switch_changed("hub", 0, 5.0)
        other.handle_hub_runout.assert_not_called()

    def test_hub_runout_off(self):
        unit, lane, _ = self._hub_unit(runout=False)
        unit.switch_changed("hub", 0, 5.0)
        lane.handle_hub_runout.assert_not_called()

    def test_no_hub(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.hub_obj = None
        unit.switch_changed("hub", 0, 5.0)
        lane.handle_hub_runout.assert_not_called()

    def test_turtleneck(self):
        unit = make_unit()
        unit.buffer_obj = TurtleNeck(unit)
        unit.switch_changed("tn_adv", 1, 5.0)
        assert unit.buffer_obj.buffer_status() == "Trailing"
        unit.switch_changed("tn_trl", 1, 5.0)
        assert unit.buffer_obj.advance_state is True
        assert unit.buffer_obj.trailing_state is True
        assert unit.buffer_obj.buffer_status() == "Advancing"

    def test_turtleneck_ignored_with_external_buffer(self):
        unit = make_unit()
        buffer = ext_buffer()
        buffer.advance_state = False
        unit.buffer_obj = buffer
        unit.switch_changed("tn_adv", 1, 5.0)
        assert buffer.advance_state is False


class TestPrepRemoved:
    def _lane(self, unit, **attrs):
        lane = make_lane(unit)
        lane._afc_prep_done = True
        lane.status = AFCLaneState.TOOLED
        lane.raw_load_state = True
        lane.runout_lane = None
        for key, value in attrs.items():
            setattr(lane, key, value)
        unit.afc.current = "lane8"
        unit.afc.function.is_printing.return_value = True
        return lane

    def test_runout_to_next_lane(self):
        unit = make_unit()
        lane = self._lane(unit, runout_lane="lane9")
        unit.prep_removed(lane, 5.0)
        assert lane.prep_state is False
        lane._perform_infinite_runout.assert_called_once_with()
        lane.handle_prep_runout.assert_not_called()

    def test_runout_pause(self):
        unit = make_unit()
        lane = self._lane(unit)
        unit.prep_removed(lane, 5.0)
        lane._perform_pause_runout.assert_called_once_with()
        lane._perform_infinite_runout.assert_not_called()

    @pytest.mark.parametrize("change", [
        {"printer_state": "Printer is starting"},
        {"_afc_prep_done": False},
        {"status": AFCLaneState.TOOL_UNLOADING},
        {"current": "lane9"},
        {"printing": False},
        {"raw_load_state": False},
        {"status": AFCLaneState.EJECTING},
    ])
    def test_otherwise_normal_removal(self, change):
        unit = make_unit()
        lane = self._lane(unit)
        if "printer_state" in change:
            unit.printer.state_message = change["printer_state"]
        if "current" in change:
            unit.afc.current = change["current"]
        if "printing" in change:
            unit.afc.function.is_printing.return_value = False
        for key in ("_afc_prep_done", "status", "raw_load_state"):
            if key in change:
                setattr(lane, key, change[key])
        unit.prep_removed(lane, 5.0)
        lane.handle_prep_runout.assert_called_once_with(5.0, False)
        lane._perform_pause_runout.assert_not_called()


class TestFault:
    def test_assist_fault_while_printing(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.sync["lane8"] = True
        unit.afc.current = "lane8"
        unit.afc.function.is_printing.return_value = True
        with patch.object(unit, "_send_assist") as assist:
            unit.fault({"code": "clog", "lane": 1, "msg": "TurtleNeck stayed full"}, 5.0)
        unit.afc.error.AFC_error.assert_called_once_with(
            "lane8: TurtleNeck stayed full (clog)", pause=True)
        assist.assert_called_once_with(lane)

    def test_assist_fault_printing_unsynced_lane(self):
        unit = make_unit()
        make_lane(unit)
        unit.afc.current = "lane8"
        unit.afc.function.is_printing.return_value = True
        with patch.object(unit, "_send_assist") as assist:
            unit.fault({"code": "clog", "lane": 1, "msg": "full"}, 5.0)
        unit.afc.error.AFC_error.assert_called_once_with("lane8: full (clog)", pause=True)
        assist.assert_not_called()

    def test_assist_fault_idle(self):
        unit = make_unit()
        make_lane(unit)
        with patch.object(unit, "_send_assist") as assist:
            unit.fault({"code": "not_feeding", "lane": 1, "msg": "pulled"}, 5.0)
        assert unit.logger.messages == [("warning", "lane8: pulled (not_feeding)")]
        unit.afc.error.AFC_error.assert_not_called()
        assist.assert_not_called()

    def test_assist_fault_no_lane(self):
        unit = make_unit()
        with patch.object(unit, "_send_assist") as assist:
            unit.fault({"code": "clog", "msg": "x"}, 5.0)
        assert unit.logger.messages == [("warning", "Turtle_1: x (clog)")]
        assist.assert_not_called()

    def test_tmc_fault(self):
        unit = make_unit()
        make_lane(unit)
        unit.afc.current = "lane8"
        unit.afc.function.is_printing.return_value = True
        unit.fault({"code": "tmc_short", "lane": 1, "msg": "short"}, 5.0)
        unit.afc.error.AFC_error.assert_called_once_with("lane8: short (tmc_short)", pause=True)

    def test_tmc_fault_other_lane_printing(self):
        unit = make_unit()
        make_lane(unit)
        unit.afc.current = "lane9"
        unit.afc.function.is_printing.return_value = True
        unit.fault({"code": "tmc_ot", "lane": 1, "msg": "hot"}, 5.0)
        unit.afc.error.AFC_error.assert_called_once_with("lane8: hot (tmc_ot)", pause=False)

    def test_link_lost(self):
        unit = make_unit()
        unit.fault({"code": "link_lost", "msg": "no host"}, 5.0)
        assert unit.logger.messages == [("warning", "Turtle_1: no host (link_lost)")]

    def test_warning(self):
        unit = make_unit()
        unit.fault({"code": "tmc_otpw", "lane": 0, "msg": "warm"}, 5.0)
        assert unit.logger.messages == [("info", "Turtle_1: warm (tmc_otpw)")]

    def test_unknown_code(self):
        unit = make_unit()
        unit.fault({"code": "cancelled", "msg": "x"}, 5.0)
        assert unit.logger.messages == []
        unit.afc.error.AFC_error.assert_not_called()


# ── moves ─────────────────────────────────────────────────────────────────────

class TestEspoolFields:
    def test_no_assist(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._espool_fields(lane, 50, False) == {
            "espool_fwd": 0.0, "espool_rwd": 0.0, "espool_kick": False}

    def test_from_afc(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._espool_fields(lane, 50, True) == {
            "espool_fwd": 0.6, "espool_rwd": 0.4, "espool_kick": True}
        assert lane.calculate_pwm_value.call_args_list == [((50,),), ((50, True),)]

    def test_capped_and_no_kick(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.calculate_pwm_value.side_effect = lambda speed, rev=False: 1.7
        lane.espooler.enable_kick_start = False
        assert unit._espool_fields(lane, 50, True) == {
            "espool_fwd": 1.0, "espool_rwd": 1.0, "espool_kick": False}

    def test_math_error(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.calculate_pwm_value.side_effect = ZeroDivisionError
        assert unit._espool_fields(lane, 50, True)["espool_fwd"] == 0.5


class TestReport:
    @pytest.mark.parametrize("in_print", [True, False])
    def test_pause_follows_print(self, in_print):
        unit = make_unit()
        lane = make_lane(unit)
        unit.afc.function.in_print.return_value = in_print
        unit._report(lane, "stuck")
        unit.afc.error.AFC_error.assert_called_once_with("lane8: stuck", pause=in_print)


class TestFwMove:
    def test_zero(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit.fw_move(lane, 0, 50, 400, False) == ("done", 0.0)
        assert unit.link.requests == []

    def test_move_with_until(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "move_done", "lane": 1, "end": "triggered", "mm": 42.5}]
        assert unit.fw_move(lane, 100, 0, 0, False, until="hub", until_v=0) == ("triggered", 42.5)
        assert unit.link.requests == [("move", {
            "lane": 1, "mm": 100.0, "speed": 50.0, "accel": 400.0, "hold": True,
            "until": "hub", "until_v": 0, "espool_fwd": 0.0, "espool_rwd": 0.0,
            "espool_kick": False, "seq": 1}), ("status", {})]
        assert unit.printer.lookup_object("toolhead").calls == 1
        assert unit.moving == set()

    def test_synced_lane_pauses_assist(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.sync["lane8"] = True
        unit.link.events = [{"ev": "move_done", "lane": 1, "end": "done", "mm": 10}]
        with patch.object(unit, "_send_assist") as assist:
            assert unit.fw_move(lane, 10, 30, 200, True) == ("done", 10.0)
        assert unit.link.requests == [
            ("assist", {"lane": 1, "on": False}),
            ("move", {"lane": 1, "mm": 10.0, "speed": 30.0, "accel": 200.0, "hold": True,
                      "espool_fwd": 0.6, "espool_rwd": 0.4, "espool_kick": True, "seq": 1}),
            ("status", {})]
        assist.assert_called_once_with(lane)

    def test_unsynced_during_move_stays_off(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.sync["lane8"] = True

        def unsync(timeout_unused=None):
            unit.sync["lane8"] = False
            return {"ev": "move_done", "lane": 1, "end": "done", "mm": 10}

        unit.link.events = [unsync()]
        with patch.object(unit, "_send_assist") as assist:
            unit.fw_move(lane, 10, 50, 400, True)
        assist.assert_not_called()


class TestSettleSwitches:
    def test_copies_the_board_switches(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.switches = {"prep1": 1, "load1": 0, "hub": 0}
        unit.link.replies["status"] = {"ok": True, "sw": {"load1": 1, "hub": 1}}
        with patch.object(unit, "_apply_switches") as apply:
            unit._settle_switches()
        assert unit.switches == {"prep1": 1, "load1": 1, "hub": 1}
        apply.assert_called_once_with()
        assert unit.link.requests == [("status", {})]

    def test_refused(self):
        unit = make_unit()
        make_lane(unit)
        unit.switches = {"load1": 0}
        unit.link.replies["status"] = {"ok": False, "err": "timeout", "sw": {"load1": 1}}
        with patch.object(unit, "_apply_switches") as apply:
            unit._settle_switches()
        assert unit.switches == {"load1": 0}
        apply.assert_not_called()

    def test_no_switches_in_reply(self):
        unit = make_unit()
        make_lane(unit)
        unit.switches = {"load1": 0}
        unit.link.replies["status"] = {"ok": True}
        with patch.object(unit, "_apply_switches") as apply:
            unit._settle_switches()
        assert unit.switches == {"load1": 0}
        apply.assert_not_called()


class TestPauseAssist:
    def test_synced(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.sync["lane8"] = True
        assert unit._pause_assist(lane) is True
        assert unit.link.requests == [("assist", {"lane": 1, "on": False})]

    def test_not_synced(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._pause_assist(lane) is False
        assert unit.link.requests == []


class TestResumeAssist:
    @pytest.mark.parametrize("was,now,called", [(True, True, True), (True, False, False),
                                                (False, True, False)])
    def test_cases(self, was, now, called):
        unit = make_unit()
        lane = make_lane(unit)
        unit.sync["lane8"] = now
        with patch.object(unit, "_send_assist") as assist:
            unit._resume_assist(lane, was)
        assert assist.called is called


class TestStartMove:
    def test_ok(self):
        unit = make_unit()
        lane = make_lane(unit)
        done = unit._start_move(lane, {"lane": 1})
        assert done is unit.link.waiters[0][1]
        predicate = unit.link.waiters[0][0]
        assert predicate({"ev": "move_done", "lane": 1}) is True
        assert predicate({"ev": "move_done", "lane": 2}) is False
        assert predicate({"ev": "job_done", "lane": 1}) is False

    def test_busy_then_ok(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.replies["move"] = [{"ok": False, "err": "lane busy"}, {"ok": True}]
        assert unit._start_move(lane, {"lane": 1}) is not None
        assert len(unit.link.requests) == 2
        assert unit.reactor.now >= 100.05

    def test_refused(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.replies["move"] = {"ok": False, "err": "bad lane"}
        assert unit._start_move(lane, {"lane": 1}) is None
        assert len(unit.link.requests) == 1
        unit.afc.error.AFC_error.assert_called_once_with(
            "lane8: BoxTurtle refused a move (bad lane)", pause=False)

    def test_always_busy(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.replies["move"] = [{"ok": False, "err": "lane busy"}] * 20
        assert unit._start_move(lane, {"lane": 1}) is None
        assert len(unit.link.requests) == 20
        unit.afc.error.AFC_error.assert_called_once_with(
            "lane8: BoxTurtle refused a move (lane busy)", pause=False)


class TestRunMove:
    def test_refused(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.replies["move"] = {"ok": False, "err": "bad"}
        assert unit._run_move(lane, {}, 10, 50, 400) == ("fault", 0.0)

    def test_timeout(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._run_move(lane, {}, 10, 50, 400) == ("fault", 0.0)
        # 10 mm at 50 mm/s, 50/400 s to speed up, plus 10 s slack
        assert unit.reactor.now == pytest.approx(100.0 + 0.2 + 0.125 + 10.0, abs=0.01)
        assert unit.link.sent == [("stop", {"lane": 1})]
        unit.afc.error.AFC_error.assert_called_once_with(
            "lane8: BoxTurtle move of 10.0 mm did not finish", pause=False)

    def test_link_down(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "link_down"}]
        assert unit._run_move(lane, {}, 10, 50, 400) == ("fault", 0.0)
        unit.afc.error.AFC_error.assert_called_once_with(
            "lane8: BoxTurtle disconnected during a move", pause=False)

    def test_defaults(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "move_done", "lane": 1}]
        assert unit._run_move(lane, {}, 10, 50, 400) == ("done", 0.0)

    def test_link_stays_down(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "link_down"}]
        unit.link.up_results = [False]
        assert unit._run_move(lane, {}, 10, 50, 400) == ("fault", 0.0)
        assert unit.link.requests == [("move", {"mm": 10.0, "seq": 1})]
        unit.afc.error.AFC_error.assert_called_once_with(
            "lane8: BoxTurtle disconnected during a move", pause=False)

    def test_link_back_during_the_move(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.logger.messages.clear()
        unit.link.events = [{"ev": "link_down"},
                            {"ev": "move_done", "lane": 1, "seq": 1, "end": "done", "mm": 10}]
        assert unit._run_move(lane, {}, 10, 50, 400) == ("done", 10.0)
        assert unit.link.requests == [("move", {"mm": 10.0, "seq": 1}),
                                      ("replay", {"lane": 1}), ("status", {})]
        assert unit.logger.messages == [("info", "lane8: BoxTurtle link came back during a move")]

    def test_stopped_on_a_drop_moves_the_rest(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.logger.messages.clear()
        unit.link.events = [{"ev": "move_done", "lane": 1, "seq": 1, "end": "link_lost", "mm": 4},
                            {"ev": "move_done", "lane": 1, "seq": 2, "end": "triggered", "mm": 5}]
        assert unit._run_move(lane, {"until": "hub"}, 10, 50, 400) == ("triggered", 9.0)
        assert unit.link.requests == [("move", {"until": "hub", "mm": 10.0, "seq": 1}),
                                      ("move", {"until": "hub", "mm": 6.0, "seq": 2}),
                                      ("status", {})]
        assert unit.logger.messages == [
            ("info", "lane8: BoxTurtle stopped on a link drop, moving the last 6.0 mm")]

    @pytest.mark.parametrize("fields,end", [({}, "done"), ({"until": "hub"}, "no_trigger")])
    @pytest.mark.parametrize("distance,mm", [(10, 10), (10, 10.004), (-10, -10.5)])
    def test_stopped_on_a_drop_with_nothing_left(self, fields, end, distance, mm):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "move_done", "lane": 1, "seq": 1, "end": "link_lost", "mm": mm}]
        assert unit._run_move(lane, fields, distance, 50, 400) == (end, mm)
        assert unit.link.requests[-1] == ("status", {})
        assert len(unit.link.requests) == 2

    def test_keeps_dropping(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "move_done", "lane": 1, "seq": n, "end": "link_lost", "mm": 1}
                            for n in range(1, 5)]
        assert unit._run_move(lane, {}, 10, 50, 400) == ("fault", 4.0)
        assert [r[1]["mm"] for r in unit.link.requests] == [10.0, 9.0, 8.0, 7.0]
        unit.afc.error.AFC_error.assert_called_once_with(
            "lane8: BoxTurtle link kept dropping during a move", pause=False)


class TestFwMovePoll:
    def test_already_there(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._fw_move_poll(lane, 50, 25, 400, False, lambda: True, True) == ("triggered", 0.0)
        assert unit.link.requests == []

    def test_given_speed(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "_poll_move", return_value=("done", 5.0)) as poll:
            unit._fw_move_poll(lane, 50, 30, 200, False, lambda: False, True)
        assert poll.call_args[0][1]["speed"] == 30.0
        assert poll.call_args[0][1]["accel"] == 200.0
        assert poll.call_args[0][3:5] == (30.0, 200.0)

    def test_fields_and_assist_pause(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.sync["lane8"] = True
        with patch.object(unit, "_poll_move", return_value=("done", 5.0)) as poll, \
                patch.object(unit, "_send_assist") as assist:
            assert unit._fw_move_poll(lane, 50, 0, 0, False, lambda: False, True) == ("done", 5.0)
        fields = poll.call_args[0][1]
        assert fields == {"lane": 1, "mm": 50.0, "speed": 50.0, "accel": 400.0, "hold": True,
                          "espool_fwd": 0.0, "espool_rwd": 0.0, "espool_kick": False}
        assert unit.link.requests == [("assist", {"lane": 1, "on": False})]
        assist.assert_called_once_with(lane)


class TestPollMove:
    def _complete_at(self, unit, when, event):
        def pump():
            if unit.reactor.now >= when and unit.link.waiters:
                completion = unit.link.waiters[-1][1]
                if not completion.done:
                    completion.complete(event)
        unit.reactor.pumps.append(pump)

    def test_refused(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.replies["move"] = {"ok": False, "err": "bad"}
        assert unit._poll_move(lane, {}, 50, 25, 400, lambda: False, True) == ("fault", 0.0)

    def test_sensor_hit_stops_lane(self):
        unit = make_unit()
        lane = make_lane(unit)
        reactor = unit.reactor
        self._complete_at(unit, 100.06, {"ev": "move_done", "lane": 1, "end": "stopped", "mm": 1.5})
        result = unit._poll_move(lane, {}, 50, 25, 400, lambda: reactor.now >= 100.03, True)
        assert result == ("triggered", 1.5)
        assert unit.link.sent == [("stop", {"lane": 1})]

    def test_move_ends_without_sensor(self):
        unit = make_unit()
        lane = make_lane(unit)
        self._complete_at(unit, 100.03, {"ev": "move_done", "lane": 1, "end": "done", "mm": 50})
        assert unit._poll_move(lane, {}, 50, 25, 400, lambda: False, True) == ("done", 50.0)
        assert unit.link.sent == []

    def test_sensor_reads_true_after_the_move(self):
        unit = make_unit()
        lane = make_lane(unit)
        reactor = unit.reactor
        self._complete_at(unit, 100.03, {"ev": "move_done", "lane": 1, "mm": 50})
        assert unit._poll_move(lane, {}, 50, 25, 400,
                               lambda: reactor.now >= 100.035, True) == ("triggered", 50.0)

    def test_overrun_stops(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "move_done", "lane": 1, "end": "stopped", "mm": 0.1}]
        start = unit.reactor.now
        assert unit._poll_move(lane, {}, 0.1, 100, 1000, lambda: False, True) == ("stopped", 0.1)
        assert unit.link.sent == [("stop", {"lane": 1})]
        assert unit.reactor.now > start + 10.0

    def test_no_move_done(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [None]
        with patch.object(unit, "_report") as report:
            assert unit._poll_move(lane, {}, 0.1, 100, 1000, lambda: False, True) == ("fault", 0.0)
        assert unit.link.sent == [("stop", {"lane": 1}), ("stop", {"lane": 1})]
        report.assert_called_once_with(lane, "BoxTurtle move to a Klipper-side sensor did not finish")

    def test_link_down(self):
        unit = make_unit()
        lane = make_lane(unit)
        self._complete_at(unit, 100.02, {"ev": "link_down"})
        with patch.object(unit, "_report") as report:
            assert unit._poll_move(lane, {}, 50, 25, 400, lambda: False, True) == ("fault", 0.0)
        report.assert_called_once()


class TestPollMoveDrops:
    """_poll_move after a USB drop. Each new waiter gets its own scripted end."""

    def _script(self, unit, ends):
        """
        ends: (time, event) per waiter, in the order the waiters are made.
        """
        def pump():
            for i, (predicate_unused, completion) in enumerate(unit.link.waiters):
                if i < len(ends) and not completion.done and unit.reactor.now >= ends[i][0]:
                    completion.complete(ends[i][1])
        unit.reactor.pumps.append(pump)

    def test_sensor_reached_while_down(self):
        unit = make_unit()
        lane = make_lane(unit)
        reactor = unit.reactor
        self._script(unit, [(100.02, {"ev": "link_down"}),
                            (100.05, {"ev": "move_done", "lane": 1, "seq": 1,
                                      "end": "stopped", "mm": 3})])
        result = unit._poll_move(lane, {}, 50, 25, 400, lambda: reactor.now >= 100.04, True)
        assert result == ("triggered", 3.0)
        assert unit.link.sent == [("stop", {"lane": 1})]
        assert unit.link.requests == [("move", {"mm": 50.0, "seq": 1}),
                                      ("replay", {"lane": 1}), ("status", {})]

    def test_moves_the_rest(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.logger.messages.clear()
        self._script(unit, [(100.02, {"ev": "link_down"}),
                            (100.03, {"ev": "move_done", "lane": 1, "seq": 1,
                                      "end": "stopped", "mm": 20}),
                            (100.05, {"ev": "move_done", "lane": 1, "seq": 2,
                                      "end": "done", "mm": 30})])
        assert unit._poll_move(lane, {}, 50, 25, 400, lambda: False, True) == ("done", 50.0)
        assert unit.link.requests == [("move", {"mm": 50.0, "seq": 1}),
                                      ("replay", {"lane": 1}),
                                      ("move", {"mm": 30.0, "seq": 2}), ("status", {})]
        assert unit.logger.messages == [
            ("info", "lane8: BoxTurtle link dropped during a move to a Klipper-side "
                     "sensor, moving the last 30.0 mm")]

    def test_board_stopped_it_on_a_drop(self):
        unit = make_unit()
        lane = make_lane(unit)
        self._script(unit, [(100.02, {"ev": "move_done", "lane": 1, "seq": 1,
                                      "end": "link_lost", "mm": 10}),
                            (100.04, {"ev": "move_done", "lane": 1, "seq": 2,
                                      "end": "done", "mm": 40})])
        assert unit._poll_move(lane, {}, 50, 25, 400, lambda: False, True) == ("done", 50.0)
        assert unit.link.sent == []
        assert [r[1] for r in unit.link.requests[:2]] == [{"mm": 50.0, "seq": 1},
                                                           {"mm": 40.0, "seq": 2}]

    def test_nothing_left(self):
        unit = make_unit()
        lane = make_lane(unit)
        self._script(unit, [(100.02, {"ev": "link_down"}),
                            (100.03, {"ev": "move_done", "lane": 1, "seq": 1,
                                      "end": "done", "mm": 50})])
        assert unit._poll_move(lane, {}, 50, 25, 400, lambda: False, True) == ("done", 50.0)
        assert unit.link.requests[-1] == ("status", {})

    def test_link_stays_down(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.up_results = [False]
        self._script(unit, [(100.02, {"ev": "link_down"})])
        with patch.object(unit, "_report") as report:
            assert unit._poll_move(lane, {}, 50, 25, 400, lambda: False, True) == ("fault", 0.0)
        assert unit.link.sent == [("stop", {"lane": 1})]
        report.assert_called_once_with(lane, "BoxTurtle move to a Klipper-side sensor did not finish")

    def test_keeps_dropping(self):
        unit = make_unit()
        lane = make_lane(unit)
        self._script(unit, [(100.02, {"ev": "move_done", "lane": 1, "seq": n,
                                      "end": "link_lost", "mm": 1}) for n in range(1, 5)])
        with patch.object(unit, "_report") as report:
            assert unit._poll_move(lane, {}, 50, 25, 400, lambda: False, True) == ("fault", 4.0)
        report.assert_called_once_with(lane, "BoxTurtle link kept dropping during a move")


class TestAfterDrop:
    def test_board_stays_gone(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.up_results = [False]
        assert unit._after_drop(lane, "move_done", 3, 5.0) is None
        assert unit.link.up_waits == [unit.RECONNECT_WAIT]
        assert unit.link.requests == []

    def test_end_event(self):
        unit = make_unit()
        lane = make_lane(unit)
        done = {"ev": "move_done", "lane": 1, "seq": 3, "end": "done"}
        unit.link.events = [done]
        assert unit._after_drop(lane, "move_done", 3, 5.0) == done
        assert unit.link.requests == [("replay", {"lane": 1})]
        assert unit.link.sent == []
        predicate = unit.link.waiters[0][0]
        assert predicate({"ev": "move_done", "lane": 1, "seq": 3}) is True
        assert predicate({"ev": "move_done", "lane": 1, "seq": 2}) is False
        assert predicate({"ev": "move_done", "lane": 2, "seq": 3}) is False
        assert predicate({"ev": "job_done", "lane": 1, "seq": 3}) is False

    def test_stop_first(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "move_done", "lane": 1, "seq": 3}]
        unit._after_drop(lane, "move_done", 3, 5.0, stop=True)
        assert unit.link.sent == [("stop", {"lane": 1})]

    def test_drops_again(self):
        unit = make_unit()
        lane = make_lane(unit)
        done = {"ev": "job_done", "lane": 1, "seq": 3, "ok": True}
        unit.link.events = [{"ev": "link_down"}, done]
        assert unit._after_drop(lane, "job_done", 3, 5.0) == done
        assert unit.link.requests == [("replay", {"lane": 1}), ("replay", {"lane": 1})]
        assert unit.link.up_waits == [unit.RECONNECT_WAIT] * 2

    def test_keeps_dropping(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "link_down"}] * unit.DROP_TRIES
        assert unit._after_drop(lane, "move_done", 3, 5.0) is None
        assert len(unit.link.requests) == unit.DROP_TRIES

    def test_never_ends(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._after_drop(lane, "move_done", 3, 5.0) is None
        assert unit.reactor.now == pytest.approx(105.0, abs=0.01)


class TestNextSeq:
    def test_counts_up(self):
        unit = make_unit()
        assert [unit._next_seq() for _ in range(3)] == [1, 2, 3]


class TestWaitToolhead:
    def test_waits_for_queued_moves(self):
        unit = make_unit()
        # print time is monotonic - 1, so moves queued to 99.25 end at 100.25
        unit.printer.lookup_object("toolhead").last_move_time = 99.25
        unit.wait_toolhead()
        assert unit.reactor.now == pytest.approx(100.25, abs=0.01)

    def test_nothing_queued(self):
        unit = make_unit()
        unit.printer.lookup_object("toolhead").last_move_time = 50.0
        unit.wait_toolhead()
        assert unit.reactor.now == 100.0

    def test_printer_not_ready(self):
        unit = make_unit()
        unit.printer.state_message = "Printer is not ready"
        unit.wait_toolhead()
        assert unit.printer.lookup_object("toolhead").calls == 0

    def test_no_mcu(self):
        unit = make_unit()
        unit.mcu = None
        unit.wait_toolhead()
        assert unit.printer.lookup_object("toolhead").calls == 0

    def test_no_toolhead(self):
        unit = make_unit()
        with patch.object(unit.printer, "lookup_object", return_value=None) as lookup:
            unit.wait_toolhead()
        lookup.assert_called_once_with("toolhead", None)
        assert unit.reactor.now == 100.0


class TestEndstopSwitch:
    @pytest.mark.parametrize("spec,switch", [("load", "load"), ("lane8_load", "load"),
                                             ("prep", "prep"), ("lane8_prep", "prep"),
                                             ("hub", "hub"), ("lane8_hub", "hub")])
    def test_board_switches(self, spec, switch):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit.endstop_switch(lane, spec) == switch

    @pytest.mark.parametrize("spec,switch", [("buffer", "tn_adv"), ("buffer_advance", "tn_adv"),
                                             ("lane8_buffer", "tn_adv"),
                                             ("buffer_trailing", "tn_trl"), ("tool", None)])
    def test_turtleneck(self, spec, switch):
        unit = make_unit()
        lane = make_lane(unit, buffer=TurtleNeck(unit))
        assert unit.endstop_switch(lane, spec) == switch

    def test_external_buffer_is_klipper_side(self):
        unit = make_unit()
        lane = make_lane(unit, buffer=ext_buffer())
        assert unit.endstop_switch(lane, "buffer") is None

    def test_none(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit.endstop_switch(lane, None) is None


class TestSensorPredicate:
    @pytest.mark.parametrize("spec", ["tool", "tool_start", "lane8_tool_start"])
    def test_tool_start(self, spec):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._sensor_predicate(lane, spec) == lane.get_toolhead_pre_sensor_state

    def test_tool_end(self):
        unit = make_unit()
        lane = make_lane(unit)
        predicate = unit._sensor_predicate(lane, "tool_end")
        assert predicate() is False
        lane.extruder_obj.tool_end_state = 1
        assert predicate() is True

    @pytest.mark.parametrize("spec,attr", [("buffer", "advance_state"),
                                           ("buffer_advance", "advance_state"),
                                           ("buffer_trailing", "trailing_state")])
    def test_buffer(self, spec, attr):
        unit = make_unit()
        buffer = ext_buffer()
        buffer.advance_state = buffer.trailing_state = False
        lane = make_lane(unit, buffer=buffer)
        predicate = unit._sensor_predicate(lane, spec)
        assert predicate() is False
        setattr(buffer, attr, True)
        assert predicate() is True

    @pytest.mark.parametrize("spec", ["buffer", "buffer_trailing", "spam", None])
    def test_unknown(self, spec):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._sensor_predicate(lane, spec) is None


class TestFwHome:
    def test_board_switch(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_move", return_value=("triggered", -12.0)) as move:
            assert unit.fw_home(lane, "hub", -100, 50, 400, False, True, True) == (True, 12.0, False)
        move.assert_called_once_with(lane, -100, 50, 400, True, until="hub", until_v=0)

    def test_board_switch_triggered(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_move", return_value=("triggered", 5.0)) as move:
            unit.fw_home(lane, "load", 100, 50, 400, True, True, False)
        assert move.call_args[1] == {"until": "load", "until_v": 1}

    def test_klipper_sensor(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "_fw_move_poll", return_value=("triggered", 30.0)) as poll:
            assert unit.fw_home(lane, "tool", 100, 50, 400, 1, True, True) == (True, 30.0, False)
        assert poll.call_args[0][:5] == (lane, 100, 50, 400, True)
        assert poll.call_args[0][5] == lane.get_toolhead_pre_sensor_state
        assert poll.call_args[0][6] is True

    def test_unknown_endstop(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "_report") as report:
            assert unit.fw_home(lane, "spam", 100, 50, 400, True, True, True) == (False, 0.0, True)
        report.assert_called_once_with(lane, "BoxTurtle cannot home to endstop spam")

    @pytest.mark.parametrize("end", ["done", "no_trigger"])
    def test_not_reached_checked(self, end):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_move", return_value=(end, 80.0)):
            assert unit.fw_home(lane, "hub", -100, 50, 400, True, True, True) == (False, 100, False)

    def test_not_reached_unchecked(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_move", return_value=("done", -80.0)):
            assert unit.fw_home(lane, "hub", -100, 50, 400, True, False, True) == (True, 80.0, False)

    def test_fault(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_move", return_value=("fault", -3.0)):
            assert unit.fw_home(lane, "hub", -100, 50, 400, True, True, True) == (False, 3.0, True)


# ── sync and assist ───────────────────────────────────────────────────────────

class TestSetSync:
    def test_on(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.e_last["lane8"] = unit.e_sent["lane8"] = 5.0
        with patch.object(unit, "_send_assist") as assist, \
                patch.object(unit, "_update_e_timer") as timer:
            unit.set_sync(lane, True)
        assert unit.sync["lane8"] is True
        assert "lane8" not in unit.e_last and "lane8" not in unit.e_sent
        assist.assert_called_once_with(lane)
        timer.assert_called_once_with()

    def test_off(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.assist_mode["lane8"] = "tn"
        with patch.object(unit, "_update_e_timer"):
            unit.set_sync(lane, False)
        assert unit.sync["lane8"] is False
        assert unit.assist_mode == {}
        assert unit.link.sent == [("assist", {"lane": 1, "on": False})]

    def test_waits_for_the_toolhead(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.printer.lookup_object("toolhead").last_move_time = 99.5
        with patch.object(unit, "_update_e_timer"):
            unit.set_sync(lane, False)
        assert unit.reactor.now == pytest.approx(100.5, abs=0.01)


class TestBufferChanged:
    def test_unknown_lane(self):
        unit = make_unit()
        lane = make_lane(unit, add=False)
        unit.buffer_changed(lane, True)
        assert unit.buffer_on == {}

    def test_not_synced(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "_send_assist") as assist:
            unit.buffer_changed(lane, True)
        assert unit.buffer_on["lane8"] is True
        assist.assert_not_called()

    def test_synced_on(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.sync["lane8"] = True
        with patch.object(unit, "_send_assist") as assist, \
                patch.object(unit, "_send_pulses") as pulses:
            unit.buffer_changed(lane, True)
        pulses.assert_called_once_with(lane)
        assist.assert_called_once_with(lane)

    def test_synced_off(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.sync["lane8"] = True
        unit.buffer_on["lane8"] = True
        with patch.object(unit, "_send_assist") as assist, \
                patch.object(unit, "_send_pulses") as pulses:
            unit.buffer_changed(lane, False)
        assert unit.buffer_on["lane8"] is False
        pulses.assert_not_called()
        assist.assert_called_once_with(lane)


class TestExternalBufferOn:
    def test_on_for_this_lane(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.buffer_obj = ext_buffer(lane)
        assert unit.external_buffer_on(lane) is True

    def test_no_buffer(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit.external_buffer_on(lane) is False

    def test_turtleneck_is_not_external(self):
        unit = make_unit()
        lane = make_lane(unit)
        neck = TurtleNeck(unit)
        neck.enable, neck.current_lane = True, lane
        lane.buffer_obj = neck
        assert unit.external_buffer_on(lane) is False

    def test_disabled(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.buffer_obj = ext_buffer(lane, enable=False)
        assert unit.external_buffer_on(lane) is False

    def test_other_lane(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.buffer_obj = ext_buffer(MagicMock())
        assert unit.external_buffer_on(lane) is False


class TestWantedMode:
    def test_tn(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.buffer_on["lane8"] = True
        lane.buffer_obj = ext_buffer(lane)
        assert unit._wanted_mode(lane) == "tn"

    def test_ext(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.buffer_obj = ext_buffer(lane)
        assert unit._wanted_mode(lane) == "ext"

    def test_follow(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit._wanted_mode(lane) == "follow"


class TestSendAssist:
    def test_follow(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.extruder_obj.tool_load_speed = 30
        unit._send_assist(lane)
        assert unit.link.sent == [("assist", {"lane": 1, "on": True, "mode": "follow",
                                              "espool_fwd": 0.6, "espool_rwd": 0.4,
                                              "espool_kick": True})]
        assert unit.assist_mode == {"lane8": "follow"}
        assert lane.calculate_pwm_value.call_args_list[0] == ((30.0,),)

    def test_ext_sends_pulses_first(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.buffer_obj = ext_buffer(lane)
        lane.assisted_unload = False
        lane.extruder_obj.tool_load_speed = None
        unit._send_assist(lane)
        assert lane.calculate_pwm_value.call_count == 0
        assert unit.link.sent == [
            ("lane_cfg", {"lane": 1, "espool_pulse_mm": 150.0, "espool_pulse_ms": 570}),
            ("assist", {"lane": 1, "on": True, "mode": "ext", "espool_fwd": 0.0,
                        "espool_rwd": 0.0, "espool_kick": False})]
        assert unit.assist_mode == {"lane8": "ext"}

    def test_tn(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.buffer_on["lane8"] = True
        unit._send_assist(lane)
        assert [c for c, _ in unit.link.sent] == ["assist"]
        assert unit.link.sent[0][1]["mode"] == "tn"


class TestSendPulses:
    def test_sends(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit._send_pulses(lane)
        assert unit.link.sent == [("lane_cfg", {"lane": 1, "espool_pulse_mm": 150.0,
                                                "espool_pulse_ms": 570})]


class TestSendFaultMm:
    def test_sends(self):
        unit = make_unit()
        unit.send_fault_mm(0.0)
        assert unit.link.sent == [("cfg", {"assist_fault_mm": 0.0})]


class TestSetCurrent:
    def test_print_current(self):
        unit = make_unit(lane_sections=[("lane8", {"unit": "Turtle_1:1", "print_current": 0.5})])
        lane = make_lane(unit)
        unit.set_current(lane, True)
        assert unit.link.sent == [("lane_cfg", {"lane": 1, "run_current": 0.5})]

    def test_no_print_current(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.set_current(lane, True)
        assert unit.link.sent == []

    def test_run_current(self):
        unit = make_unit(lane_sections=[("lane8", {"unit": "Turtle_1:1", "run_current": 0.7,
                                                   "print_current": 0.5})])
        lane = make_lane(unit)
        unit.set_current(lane, False)
        assert unit.link.sent == [("lane_cfg", {"lane": 1, "run_current": 0.7})]

    def test_unit_run_current(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.lane_settings = {}
        unit.set_current(lane, False)
        assert unit.link.sent == [("lane_cfg", {"lane": 1, "run_current": 0.8})]


class TestUpdateETimer:
    def test_active(self):
        unit = make_unit()
        unit.sync["lane8"] = True
        unit._update_e_timer()
        assert unit.e_timer["when"] == unit.reactor.NOW

    def test_nothing_synced(self):
        unit = make_unit()
        unit.sync["lane8"] = False
        unit.e_timer["when"] = 5.0
        unit._update_e_timer()
        assert unit.e_timer["when"] == unit.reactor.NEVER

    def test_not_connected(self):
        unit = make_unit()
        unit.sync["lane8"] = True
        unit.link.connected = False
        unit._update_e_timer()
        assert unit.e_timer["when"] == unit.reactor.NEVER


class TestLanePosition:
    def test_first_sample(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit.lane_position(lane, 120.0) == 120.0
        assert unit.e_last["lane8"] == unit.e_sent["lane8"] == 120.0

    def test_follow_is_unscaled(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.assist_mode["lane8"] = "follow"
        lane.drive_stepper.multiplier = 1.5
        unit.lane_position(lane, 100.0)
        assert unit.lane_position(lane, 110.0) == 110.0

    def test_ext_scales_each_move(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.assist_mode["lane8"] = "ext"
        unit.lane_position(lane, 100.0)
        lane.drive_stepper.multiplier = 0.5
        assert unit.lane_position(lane, 110.0) == 105.0
        lane.drive_stepper.multiplier = 2.0
        assert unit.lane_position(lane, 111.0) == 107.0
        assert unit.e_last["lane8"] == 111.0

    def test_sent_missing_rebaselines(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.e_last["lane8"] = 50.0
        assert unit.lane_position(lane, 60.0) == 60.0


class TestEStreamCb:
    def _unit(self):
        unit = make_unit()
        extruder = MagicMock()
        extruder.find_past_position = MagicMock(return_value=200.0)
        a = make_lane(unit)
        b = make_lane(unit, "lane9", 2)
        for lane in (a, b):
            lane.extruder_obj.toolhead_extruder = extruder
            unit.sync[lane.name] = True
            unit.assist_mode[lane.name] = "follow"
        return unit, a, b, extruder

    def test_idle(self):
        unit = make_unit()
        assert unit._e_stream_cb(100.0) == unit.reactor.NEVER

    def test_not_connected(self):
        unit, *_ = self._unit()
        unit.link.connected = False
        assert unit._e_stream_cb(100.0) == unit.reactor.NEVER

    def test_sends_each_lane(self):
        unit, a, b, extruder = self._unit()
        assert unit._e_stream_cb(100.0) == pytest.approx(100.1)
        assert unit.link.sent == [("e", {"lane": 1, "pos": 200.0}),
                                  ("e", {"lane": 2, "pos": 200.0})]
        extruder.find_past_position.assert_called_once_with(99.0)

    def test_skips_lanes_without_extruder_and_unsynced(self):
        unit, a, b, _ = self._unit()
        a.extruder_obj.toolhead_extruder = None
        unit.sync["lane9"] = False
        unit._e_stream_cb(100.0)
        assert unit.link.sent == []

    def test_switches_mode_when_buffer_turns_on(self):
        unit, a, b, _ = self._unit()
        a.buffer_obj = ext_buffer(a)
        unit._e_stream_cb(100.0)
        assert unit.assist_mode == {"lane8": "ext", "lane9": "follow"}
        assert [c for c, _ in unit.link.sent] == ["lane_cfg", "assist", "e", "e"]

    def test_pulses_every_30_s(self):
        unit, a, b, _ = self._unit()
        unit.assist_mode["lane8"] = "tn"
        unit.buffer_on["lane8"] = True
        unit._pulse_tick = 299
        unit._e_stream_cb(100.0)
        assert unit._pulse_tick == 0
        assert unit.link.sent[-1] == ("lane_cfg", {"lane": 1, "espool_pulse_mm": 150.0,
                                                   "espool_pulse_ms": 570})
        assert [c for c, _ in unit.link.sent].count("lane_cfg") == 1

    def test_pulse_tick_counts(self):
        unit, *_ = self._unit()
        unit._e_stream_cb(100.0)
        assert unit._pulse_tick == 1

    def test_pulse_skips_unsynced(self):
        unit, a, b, _ = self._unit()
        unit.assist_mode["lane8"] = "ext"
        a.buffer_obj = ext_buffer(a)
        unit.sync["lane8"] = False
        unit._pulse_tick = 299
        unit._e_stream_cb(100.0)
        assert "lane_cfg" not in [c for c, _ in unit.link.sent]


# ── AFC unit hooks ────────────────────────────────────────────────────────────

class TestWaitForBoard:
    def test_connected(self):
        unit = make_unit()
        assert unit._wait_for_board() is True
        assert unit.reactor.now == 100.0

    def test_gives_up(self):
        unit = make_unit()
        unit.link.connected = False
        assert unit._wait_for_board() is False
        assert unit.reactor.now >= 100.0 + unit.CONNECT_WAIT


class TestSystemTest:
    def test_no_board(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.last_error = "cannot open"
        with patch.object(unit, "_wait_for_board", return_value=False), \
                patch.object(unit, "lane_fault") as fault, \
                patch.object(afcBoxTurtle, "system_Test") as base:
            assert unit.system_Test(lane, 0, True, True) is False
        assert unit.logger.messages == [
            ("info", "lane8 <span class=error--text>Turtle_1: BoxTurtle not found on USB "
                     "(cannot open)</span>")]
        fault.assert_called_once_with(lane)
        lane.set_afc_prep_done.assert_called_once_with()
        base.assert_not_called()

    def test_no_board_no_error(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "_wait_for_board", return_value=False), \
                patch.object(unit, "lane_fault"):
            unit.system_Test(lane, 0, True, True)
        assert unit.logger.messages == [
            ("info", "lane8 <span class=error--text>Turtle_1: BoxTurtle not found on USB</span>")]

    def test_board(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(afcBoxTurtle, "system_Test", return_value=True) as base:
            assert unit.system_Test(lane, 0, True, False) is True
        base.assert_called_once_with(lane, 0, True, False)


class TestPrepLoad:
    def test_already_loaded(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.raw_load_state = True
        with patch.object(unit, "fw_job") as job:
            unit.prep_load(lane)
        job.assert_not_called()

    def test_ok(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_job", return_value={"ok": True}) as job:
            unit.prep_load(lane)
        job.assert_called_once_with(lane, "feed", until="load", mm=400.0, speed=50.0,
                                    accel=400.0, watch="prep", hold=True, espool_fwd=0.0,
                                    espool_rwd=0.0, espool_kick=False)
        unit.afc.error.AFC_error.assert_not_called()

    def test_failed(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.status = AFCLaneState.LOADED
        with patch.object(unit, "fw_job", return_value={"ok": False, "code": "snag"}), \
                patch.object(unit, "lane_fault") as fault:
            unit.prep_load(lane)
        msg = (" FAILED TO LOAD, CHECK FILAMENT AT TRIGGER (snag)"
               "\n||==>--||----||------||\nTRG   LOAD   HUB    TOOL")
        unit.afc.error.AFC_error.assert_called_once_with(msg, False)
        fault.assert_called_once_with(lane)
        assert lane.status == AFCLaneState.NONE

    def test_failed_message_wins(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_job", return_value={"ok": False, "code": "snag",
                                                        "msg": "stuck at 30 mm"}), \
                patch.object(unit, "lane_fault"):
            unit.prep_load(lane)
        assert "(stuck at 30 mm)" in unit.afc.error.AFC_error.call_args[0][0]

    def test_failed_after_removal(self):
        unit = make_unit()
        lane = make_lane(unit)
        lane.prep_state = False
        lane.status = AFCLaneState.LOADED
        with patch.object(unit, "fw_job", return_value={"ok": False, "code": "cancelled"}):
            unit.prep_load(lane)
        unit.afc.error.AFC_error.assert_not_called()
        assert lane.status == AFCLaneState.LOADED


class TestEjectLane:
    def test_ok(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_job", return_value={"ok": True}) as job:
            unit.eject_lane(lane)
        job.assert_called_once_with(lane, "unload", until="load", mm=280.0, speed=60.0,
                                    accel=300.0, extra_mm=50.0, espool_fwd=0.6,
                                    espool_rwd=0.4, espool_kick=True)
        lane.do_enable.assert_called_once_with(False)
        unit.afc.error.AFC_error.assert_not_called()

    def test_failed(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_job", return_value={"ok": False, "code": "not_clear"}):
            unit.eject_lane(lane)
        unit.afc.error.AFC_error.assert_called_once_with(" Failed to eject lane8 (not_clear)", False)
        lane.do_enable.assert_called_once_with(False)

    def test_disables_even_on_error(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(unit, "fw_job", side_effect=RuntimeError("x")):
            with pytest.raises(RuntimeError):
                unit.eject_lane(lane)
        lane.do_enable.assert_called_once_with(False)


class TestFwJob:
    def test_refused(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.replies["job"] = {"ok": False, "err": "lane busy"}
        assert unit.fw_job(lane, "feed", mm=100.0) == {"ok": False, "code": "lane busy"}

    def test_done(self):
        unit = make_unit()
        lane = make_lane(unit)
        done = {"ev": "job_done", "lane": 1, "ok": True}
        unit.link.events = [done]
        assert unit.fw_job(lane, "feed", mm=100.0, speed=50.0) == done
        assert unit.link.requests == [("job", {"job": "feed", "lane": 1, "seq": 1,
                                               "mm": 100.0, "speed": 50.0}), ("status", {})]
        predicate = unit.link.waiters[0][0]
        assert predicate({"ev": "job_done", "lane": 1, "seq": 1}) is True
        assert predicate({"ev": "job_done", "lane": 1, "seq": 2}) is False
        assert predicate({"ev": "job_done", "lane": 1}) is False
        assert predicate({"ev": "job_done", "lane": 2, "seq": 1}) is False
        assert predicate({"ev": "move_done", "lane": 1, "seq": 1}) is False
        assert unit.printer.lookup_object("toolhead").calls == 1
        assert unit.moving == set()

    def test_switches_current_when_it_returns(self):
        # The job ended on the load switch before AFC ran its change.
        unit = make_unit()
        lane = make_lane(unit)
        lane._load_state = False
        unit.link.events = [{"ev": "job_done", "lane": 1, "ok": True}]
        unit.link.replies["status"] = {"ok": True, "sw": {"prep1": 1, "load1": 1}}
        unit.fw_job(lane, "feed", until="load", mm=100.0)
        assert lane._load_state is True

    def test_timeout(self):
        unit = make_unit()
        lane = make_lane(unit)
        assert unit.fw_job(lane, "feed", speed=0) == {"ok": False, "code": "timeout"}
        assert unit.link.sent == [("stop", {"lane": 1})]
        # 3 tries of 200 mm at the 50 mm/s fallback plus 10 s each
        assert unit.reactor.now == pytest.approx(100.0 + 3 * 14.0, abs=0.01)

    def test_timeout_from_job_fields(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.fw_job(lane, "feed", speed=100.0, mm=50.0, tries=0)
        assert unit.reactor.now == pytest.approx(100.0 + 1.0 + 10.0, abs=0.01)

    def test_link_down(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "link_down"}]
        unit.link.up_results = [False]
        assert unit.fw_job(lane, "unload") == {"ok": False, "code": "link_down"}
        assert unit.moving == set()

    def test_link_back(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.logger.messages.clear()
        done = {"ev": "job_done", "lane": 1, "seq": 1, "ok": True}
        unit.link.events = [{"ev": "link_down"}, done]
        assert unit.fw_job(lane, "feed", mm=100.0) == done
        assert unit.link.requests == [("job", {"job": "feed", "lane": 1, "seq": 1, "mm": 100.0}),
                                      ("replay", {"lane": 1}), ("status", {})]
        assert unit.logger.messages == [
            ("info", "lane8: BoxTurtle link came back during the feed job")]

    def test_stopped_on_a_drop_runs_again(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.logger.messages.clear()
        done = {"ev": "job_done", "lane": 1, "seq": 2, "ok": True}
        unit.link.events = [{"ev": "job_done", "lane": 1, "seq": 1, "ok": False,
                             "code": "link_lost"}, done]
        assert unit.fw_job(lane, "unload", mm=80.0) == done
        assert unit.link.requests == [
            ("job", {"job": "unload", "lane": 1, "seq": 1, "mm": 80.0}),
            ("job", {"job": "unload", "lane": 1, "seq": 2, "mm": 80.0}), ("status", {})]
        assert unit.logger.messages == [
            ("info", "lane8: BoxTurtle stopped the unload job on a link drop, running it again")]

    def test_keeps_dropping(self):
        unit = make_unit()
        lane = make_lane(unit)
        unit.link.events = [{"ev": "job_done", "lane": 1, "seq": n, "ok": False,
                             "code": "link_lost"} for n in range(1, 5)]
        assert unit.fw_job(lane, "feed") == {"ok": False, "code": "link_lost"}
        assert [r[1]["seq"] for r in unit.link.requests] == [1, 2, 3, 4]
        assert unit.moving == set()

    def test_lane_is_moving_during_the_job(self):
        unit = make_unit()
        lane = make_lane(unit)
        seen = []
        wait = unit.link.wait

        def spy(completion, timeout):
            seen.append(set(unit.moving))
            return wait(completion, timeout)

        unit.link.events = [{"ev": "job_done", "lane": 1, "seq": 1, "ok": True}]
        with patch.object(unit.link, "wait", side_effect=spy):
            unit.fw_job(lane, "feed")
        assert seen == [{"lane8"}]
        assert unit.moving == set()


class TestTriggerLedState:
    def test_static_color_goes_to_board(self):
        unit = make_unit()
        lane = make_lane(unit)
        with patch.object(afcUnit, "_trigger_led_state") as base, \
                patch.object(unit, "board_led") as led:
            unit._trigger_led_state(lane, static_color="0,1,0,0")
        base.assert_called_once_with(lane, None, "0,1,0,0", None, None)
        led.assert_called_once_with(lane, "0,1,0,0")

    @pytest.mark.parametrize("with_lane,color", [(False, "0,1,0,0"), (True, None)])
    def test_needs_lane_and_color(self, with_lane, color):
        unit = make_unit()
        lane = make_lane(unit) if with_lane else None
        with patch.object(afcUnit, "_trigger_led_state"), \
                patch.object(unit, "board_led") as led:
            unit._trigger_led_state(lane, static_color=color)
        led.assert_not_called()


class TestBoardLed:
    def _unit(self, led="RGB2:25-32"):
        lane_values = {"unit": "Turtle_1:1"}
        if led is not None:
            lane_values["turtle_led"] = led
        unit = make_unit({"rgb2_chain_count": 64}, lane_sections=[("lane8", lane_values)])
        return unit, make_lane(unit)

    def test_range(self):
        unit, lane = self._unit()
        unit.board_led(lane, "1,0,0,0")
        assert unit.led_colors == {"lane8": "1,0,0,0"}
        assert unit.link.sent == [("led", {"strip": 2, "i": 25, "n": 8, "color": "FF000000"})]

    def test_single(self):
        unit, lane = self._unit(led="RGB2:7")
        unit.board_led(lane, "0,0,1,0")
        assert unit.link.sent == [("led", {"strip": 2, "i": 7, "n": 1, "color": "0000FF00"})]

    def test_several_ranges(self):
        unit, lane = self._unit(led="RGB1:1, RGB2:25-26")
        unit.board_led(lane, "0,1,0,0")
        assert unit.link.sent == [("led", {"strip": 1, "i": 1, "n": 1, "color": "00FF0000"}),
                                  ("led", {"strip": 2, "i": 25, "n": 2, "color": "00FF0000"})]

    def test_stock_default(self):
        unit, lane = self._unit(led=None)
        unit.board_led(lane, "0,0,0,1")
        assert unit.link.sent == [("led", {"strip": 1, "i": 1, "n": 1, "color": "000000FF"})]

    def test_bad_color_not_sent(self):
        unit, lane = self._unit()
        unit.board_led(lane, "red")
        assert unit.led_colors == {"lane8": "red"}
        assert unit.link.sent == []

    def test_lane_without_leds(self):
        unit, _ = self._unit()
        other = make_lane(unit, name="lane9", index=2)
        unit.board_led(other, "1,0,0,0")
        assert unit.led_colors == {"lane9": "1,0,0,0"}
        assert unit.link.sent == []


class TestGetStatus:
    def test_turtleneck(self):
        unit = make_unit()
        unit.buffer_obj = TurtleNeck(unit)
        unit.link.last_error = "x"
        unit.assist_mode["lane8"] = "tn"
        with patch.object(afcBoxTurtle, "get_status", return_value={"lanes": []}):
            status = unit.get_status()
        assert status == {"lanes": [], "turtle": {
            "connected": True, "port": unit.link.port, "firmware": "fw1", "uid": "UID1",
            "reset": "power", "error": "x", "turtleneck": unit.buffer_obj.get_status(),
            "assist": {"lane8": "tn"}}}

    def test_external_buffer(self):
        unit = make_unit()
        unit.buffer_obj = ext_buffer()
        with patch.object(afcBoxTurtle, "get_status", return_value={}):
            assert unit.get_status()["turtle"]["turtleneck"] is None


# ── gcode ─────────────────────────────────────────────────────────────────────

class TestCmdAfcTurtleStatus:
    def test_not_connected(self):
        unit = make_unit()
        unit.link.connected = False
        gcmd = MagicMock()
        unit.cmd_AFC_TURTLE_STATUS(gcmd)
        gcmd.respond_info.assert_called_once_with("Turtle_1: not connected (no board found)")

    def test_not_connected_with_error(self):
        unit = make_unit()
        unit.link.connected = False
        unit.link.last_error = "busy"
        gcmd = MagicMock()
        unit.cmd_AFC_TURTLE_STATUS(gcmd)
        gcmd.respond_info.assert_called_once_with("Turtle_1: not connected (busy)")

    def test_connected(self):
        unit = make_unit()
        unit.link.replies["status"] = {"ok": True, "sw": {"hub": 1}, "lanes": [{"pos": 1}]}
        gcmd = MagicMock()
        unit.cmd_AFC_TURTLE_STATUS(gcmd)
        gcmd.respond_info.assert_called_once_with(
            f"Turtle_1: firmware fw1 on {unit.link.port}\nswitches: {{'hub': 1}}\nlanes: [{{'pos': 1}}]")


class TestCmdAfcTurtleSend:
    def test_sends(self):
        unit = make_unit()
        unit.link.replies["tmc_read"] = {"ok": True, "val": 5}
        gcmd = MockGCodeCommand({"CMD": "tmc_read", "ARGS": '{"lane": 1, "reg": 111}'})
        gcmd.respond_info = MagicMock()
        unit.cmd_AFC_TURTLE_SEND(gcmd)
        assert unit.link.requests == [("tmc_read", {"lane": 1, "reg": 111})]
        gcmd.respond_info.assert_called_once_with("{'ok': True, 'val': 5}")

    def test_no_args(self):
        unit = make_unit()
        gcmd = MockGCodeCommand({"CMD": "ping"})
        gcmd.respond_info = MagicMock()
        unit.cmd_AFC_TURTLE_SEND(gcmd)
        assert unit.link.requests == [("ping", {})]

    def test_bad_json(self):
        unit = make_unit()
        gcmd = MagicMock()
        gcmd.get = MagicMock(side_effect=lambda key, default=None: {"CMD": "x", "ARGS": "{"}.get(key, default))
        gcmd.error = ValueError
        with pytest.raises(ValueError) as err:
            unit.cmd_AFC_TURTLE_SEND(gcmd)
        assert str(err.value).startswith("ARGS is not JSON: ")
        assert unit.link.requests == []


class TestCmdAfcTurtleBootloader:
    def test_refused_while_printing(self):
        unit = make_unit()
        unit.afc.function.is_printing.return_value = True
        gcmd = MagicMock()
        gcmd.error = ValueError
        with pytest.raises(ValueError, match="Cannot restart the BoxTurtle into its bootloader while printing"):
            unit.cmd_AFC_TURTLE_BOOTLOADER(gcmd)
        assert unit.link.requests == []

    def test_restarts(self):
        unit = make_unit()
        gcmd = MagicMock()
        unit.cmd_AFC_TURTLE_BOOTLOADER(gcmd)
        assert unit.link.requests == [("bootloader", {})]
        gcmd.respond_info.assert_called_once_with("{'ok': True}")


class TestLoadConfigPrefix:
    def test_builds_unit(self):
        printer = MockPrinter()
        printer._reactor = printer.reactor = FakeReactor()
        config = UnitConfig("AFC_BoxTurtleUSB Turtle_2", printer, {})
        unit = load_config_prefix(config)
        assert isinstance(unit, afcBoxTurtleUSB)
        assert unit.name == "Turtle_2"
