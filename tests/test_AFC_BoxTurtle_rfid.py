"""Unit tests for extras/AFC_BoxTurtle_rfid.py."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import pytest

from extras import AFC_BoxTurtle_rfid as bt_rfid_mod, AFC_RFID as afc_rfid_mod
from extras.AFC_BoxTurtle_rfid import _SerialRegLink, AFC_BoxTurtle_rfid, AFC_BoxTurtle_rfid_reader
from extras.AFC_lane import AssistActive, SpeedMode
from extras.AFC_rfid_write import StageError
from tests.bambu_helpers import (
    BambuConfig,
    BambuLogger,
    BambuPrinter,
    FakeCompletion,
    FakeGcmd,
    FakeReactor,
    FakeSerial,
    LogLine,
    make_bt_bridge_serial,
    Recorder,
)


#: The serial path every coordinator here is configured with.
BT_RFID_SERIAL = "/dev/serial/by-id/test-rfid-pico"


#: gcmd.error() raises this.
BT_RFID_CMD_ERROR = BambuPrinter.command_error


#: One spool turn on the configured 200mm spool, worked out by hand: pi * 200.
BT_RFID_TURN_MM = 628.3185307179587


#: The wall-clock time AFC_RFID stamps on every tag-read record here.
BT_RFID_WALL_TIME = 1700000000.0


def bt_rfid_logged(logger: BambuLogger) -> List[LogLine]:
    """
    The logger's (level, message) lines, once its side records are seen empty.

    AFC's logger takes one ready-made message: an extra argument to info,
    debug or error lands in console_only, file_only or tracebacks instead.

    :param logger: the logger the module logs to
    :return List[LogLine]: every line logged, in order
    """
    assert logger.console_only == []
    assert logger.file_only == []
    assert logger.tracebacks == []
    assert logger.stack_names == []
    return logger.messages


def bt_rfid_read_out(lane: str, uid: Optional[str]) -> LogLine:
    """
    The console read-out of a tag that decodes to a bare UID on 1.75mm.

    :param lane: the lane the tag was applied to
    :param uid: the tag's UID, or None when it carried none
    :return LogLine: the gcode console line
    """
    text = f"BT RFID: read spool on {lane}\n  Diameter: 1.75mm"
    if uid:
        text += f"\n  Tag UID: {uid}"
    return ("respond_info", text)


def bt_rfid_record(uid: Optional[str]) -> Dict[str, Any]:
    """
    The last-read record such a tag leaves for get_status.

    :param uid: the tag's UID, or None when it carried none
    :return dict: the record, stamped BT_RFID_WALL_TIME
    """
    record: Dict[str, Any] = {"is_dual_color": False, "diameter": 1.75,
                              "decoded": True, "scan_time": BT_RFID_WALL_TIME}
    if uid:
        record["uid"] = uid
    return record


class BtRfidCompletion(FakeCompletion):
    """A reactor completion whose wait() runs the reactor until it completes,
    the way Klipper's does while a worker thread finishes."""

    def __init__(self, reactor: "BtRfidReactor") -> None:
        """:param reactor: the reactor whose async callbacks wait() runs"""
        super().__init__()
        self._reactor = reactor

    def wait(self, waketime: float = 0.0, waketime_result: Any = None) -> Any:
        """:return Any: the completed result, or waketime_result after 10s"""
        deadline = time.monotonic() + 10.0
        while not self.done and time.monotonic() < deadline:
            self._reactor.run_callbacks()
            time.sleep(0.0005)
        return self.result if self.done else waketime_result


class BtRfidReactor(FakeReactor):
    """FakeReactor with thread-safe async callbacks and waiting completions."""

    def __init__(self, now: float = 100.0) -> None:
        """:param now: the starting time"""
        super().__init__(now=now)
        self._async_lock = threading.Lock()

    def register_async_callback(self, callback: Callable[[float], Any],
                                waketime: float = FakeReactor.NOW) -> None:
        """:param callback: run by the next run_callbacks, on the caller's thread"""
        with self._async_lock:
            self.async_callbacks.append(callback)

    def run_callbacks(self, until: Optional[float] = None) -> int:
        """:return int: how many async callbacks ran"""
        if until is not None:
            self.now = max(self.now, float(until))
        with self._async_lock:
            pending, self.async_callbacks = self.async_callbacks, []
        for callback in pending:
            callback(self.now)
        return len(pending)

    def completion(self) -> BtRfidCompletion:
        """:return BtRfidCompletion: a completion that runs this reactor"""
        return BtRfidCompletion(self)


class BtRfidPico(FakeSerial):
    """The rfid_bridge Pico's port. Each register op is answered from its
    bus's script ("=92", "=", "!" or "" for silence); the last entry repeats.
    With ``hold`` every readline waits for that event first."""

    def __init__(self, answers: Optional[Dict[int, List[str]]] = None,
                 hold: Optional[threading.Event] = None) -> None:
        """
        :param answers: bus -> replies, in order
        :param hold: an event readline waits on before answering
        """
        super().__init__()
        self.answers = answers if answers is not None else {0: ["=92"], 1: ["=92"]}
        self.hold = hold

    def write(self, data: bytes) -> int:
        """:return int: bytes written; the reply is queued for readline"""
        count = super().write(data)
        bus = int(data.decode()[1])
        script = self.answers.get(bus, [""])
        reply = script.pop(0) if len(script) > 1 else script[0]
        if reply:
            self.lines.append(f"{reply}\n".encode())
        return count

    def readline(self) -> bytes:
        """:return bytes: the queued reply, or b"" (a timeout)"""
        if self.hold is not None:
            self.hold.wait(5.0)
        return super().readline()

    def ops(self) -> List[str]:
        """:return List[str]: the ops written so far, without newlines"""
        return [line.decode().strip() for line in self.written]


class BtRfidWallClock:
    """``time`` as AFC_RFID reads it: a wall clock stopped at one instant."""

    def __init__(self, now: float) -> None:
        """:param now: what time() reports"""
        self.now = now

    def time(self) -> float:
        """:return float: the stopped wall-clock time"""
        return self.now


class BtRfidSerialModule:
    """pyserial as the module imports it: Serial() opens the fake Pico, or
    fails like an unplugged one when there is no port."""

    def __init__(self, port: Optional[BtRfidPico]) -> None:
        """:param port: what Serial() opens; None raises"""
        self.port = port
        self.opened: List[Tuple[tuple, dict]] = []

    def Serial(self, *args: Any, **kwargs: Any) -> BtRfidPico:
        """:return BtRfidPico: the port"""
        self.opened.append((args, kwargs))
        if self.port is None:
            raise OSError("could not open port")
        return self.port


class BtRfidField:
    """
    The shared ``read_tag`` stack, as the antenna sees it.

    A call without ``is_excluded`` is the parked-tag probe, answered from
    ``parked``, or with ``parked_tag`` verbatim when that is set. A read
    answers ``uid`` while ``present()``: by ``script`` (one bool per read) or
    while ``lane`` sits in ``[lo, hi]``. A read honours the excluder the way
    read_tag does: an excluded tag is passed over.
    """

    def __init__(self, uid: str = "DEADBEEF") -> None:
        """:param uid: the UID of the tag that answers a read"""
        self.uid = uid
        self.parked: Optional[str] = None
        self.parked_tag: Optional[dict] = None
        self.lane: Optional["BtRfidLane"] = None
        self.lo = math.inf
        self.hi = math.inf
        self.script: Optional[List[bool]] = None
        self.raises: Optional[Exception] = None
        self.on_probe: Optional[Callable[[], None]] = None
        self.calls: List[Tuple[str, bool, Any]] = []
        self._lock = threading.Lock()

    def appear(self, lane: "BtRfidLane", after_mm: float,
               until_mm: float = math.inf,
               start: Optional[float] = None) -> None:
        """
        Put the tag in the field once ``lane`` has fed ``after_mm``.

        :param lane: the lane whose spool carries the tag
        :param after_mm: feed from ``start`` at which it answers
        :param until_mm: feed from ``start`` past which it no longer answers
        :param start: where the feed is counted from; the lane's tip if None
        """
        origin = lane.pos if start is None else start
        self.lane = lane
        self.lo = origin + after_mm
        self.hi = origin + until_mm
        lane.after_feed = self.hold_for_poller

    def present(self) -> bool:
        """:return bool: whether the tag answers this read"""
        if self.script is not None:
            return self.script.pop(0) if self.script else False
        return self.lane is not None and self.lo <= self.lane.pos <= self.hi

    def hold_for_poller(self) -> None:
        """After a feed puts the tag in the field, wait for the sweep's poller
        to read it, as the time a real move takes always allows."""
        if not self.present():
            return
        for thread in threading.enumerate():
            if thread.name == "afc_bt_rfid_sw":
                thread.join(5.0)

    def __call__(self, link: Any, **kwargs: Any) -> Optional[dict]:
        """
        :param link: the reader's register link
        :return Optional[dict]: the tag read, or None
        """
        probe = "is_excluded" not in kwargs
        excluder = kwargs.get("is_excluded")
        with self._lock:
            self.calls.append((threading.current_thread().name, probe, excluder))
        if self.raises is not None:
            raise self.raises
        if probe:
            if self.on_probe is not None:
                self.on_probe()
            if self.parked_tag is not None:
                return self.parked_tag
            return {"uid": self.parked} if self.parked else None
        if not self.present():
            return None
        if excluder is not None and excluder(self.uid):
            return None
        return {"uid": self.uid, "sak": 8}

    def reads(self) -> List[str]:
        """:return List[str]: thread names of the reads (not the probes)"""
        return [name for name, probe, _ex in self.calls if not probe]

    def probes(self) -> List[str]:
        """:return List[str]: thread names of the parked-tag probes"""
        return [name for name, probe, _ex in self.calls if probe]


class BtRfidHub:
    """An AFC hub: its name, whether its switch is virtual, and its state,
    fixed or read from where a watched lane's tip is."""

    def __init__(self, name: str = "Turtle_1", virtual: bool = False,
                 state: bool = False) -> None:
        """
        :param name: the hub's name
        :param virtual: True when the switch is every lane's load sensor
        :param state: the switch state, while no lane is watched
        """
        self.name = name
        self._virtual = virtual
        self._state = state
        self.watched: Optional["BtRfidLane"] = None
        self.at = math.inf
        self.hub_clear_move_dis = 35.0

    def watch(self, lane: "BtRfidLane", at: float) -> None:
        """
        :param lane: the lane whose tip trips the switch
        :param at: the position at which it trips
        """
        self.watched, self.at = lane, at

    @property
    def state(self) -> bool:
        """:return bool: whether the switch reads filament"""
        if self.watched is not None:
            return self.watched.pos >= self.at
        return self._state

    @state.setter
    def state(self, value: bool) -> None:
        self._state = value

    def is_virtual_pin(self) -> bool:
        """:return bool: whether the switch is virtual"""
        return self._virtual


class BtRfidUnitObj:
    """The lane's unit, as the insert cycle uses it: prep_load homes forward
    onto the load switch, prep_post_load feeds dist_hub and marks it staged."""

    def __init__(self) -> None:
        self.calls: List[str] = []

    def prep_load(self, lane: "BtRfidLane") -> None:
        """:param lane: the lane to home onto its load switch"""
        self.calls.append("prep_load")
        lane.move_to(400.0, None, endstop=lane.load_es, use_homing=True)
        for _ in range(40):
            if lane.raw_load_state:
                break
            lane.move(lane.short_move_dis, 500.0, 400)

    def prep_post_load(self, lane: "BtRfidLane") -> None:
        """:param lane: the lane to stage at the hub"""
        self.calls.append("prep_post_load")
        if lane.loaded_to_hub or not lane.raw_load_state:
            return
        lane.move(lane.dist_hub, 100.0, 400, False)
        lane.loaded_to_hub = True


class BtRfidLane:
    """
    An AFC lane whose load switch tracks the filament tip.

    ``pos`` is the tip; the load switch reads filament above ``load_at``.
    ``slip`` is the fraction of each feed the gears lose. ``move_to`` models
    AFC's homing: a forward home stops when the switch is made, a reverse one
    when it releases. ``fail_after`` makes every plain move after that many raise.
    """

    def __init__(self, name: str, *, pos: float = 5.0, slip: float = 0.0,
                 dist_hub: Optional[float] = None,
                 hub: Optional[BtRfidHub] = None, staged: bool = False,
                 tool_loaded: bool = False) -> None:
        """
        :param name: the lane's name
        :param pos: where the tip starts; 5mm is just past the load switch
        :param slip: fraction of each feed that is lost
        :param dist_hub: the calibrated load switch to hub distance
        :param hub: the lane's hub
        :param staged: loaded_to_hub
        :param tool_loaded: loaded in the toolhead
        """
        self.name = name
        self.long_moves_accel = 400
        self.long_moves_speed = 100.0
        self.short_move_dis = 10.0
        self.short_moves_speed = 25.0
        self.load_es = "load"
        self.pos = pos
        self.load_at = 0.0
        self.slip = slip
        self.prep_state = True
        self.dist_hub = dist_hub
        self.hub_obj = hub
        self.loaded_to_hub = staged
        self.load_to_hub = True
        self.tool_loaded = tool_loaded
        self.unit_obj = BtRfidUnitObj()
        self.moves: List[Tuple[float, float, float, Any]] = []
        self.homing_moves: List[Tuple[float, Any, Any, bool, Any]] = []
        self.feed_starts: List[Tuple[float, bool]] = []
        self.after_feed: Optional[Callable[[], None]] = None
        self.fail_after: Optional[int] = None
        self.fault = "stepper gone"

    @property
    def raw_load_state(self) -> bool:
        """:return bool: whether the load switch reads filament"""
        return self.pos > self.load_at

    def is_direct_hub(self) -> bool:
        """:return bool: never direct-to-hub"""
        return False

    def _check_fault(self) -> None:
        """Raise ``fault`` once ``fail_after`` plain moves have been made."""
        if self.fail_after is not None and len(self.moves) >= self.fail_after:
            raise RuntimeError(self.fault)

    def move(self, distance: float, speed: float, accel: float,
             assist_active: Any = False) -> None:
        """
        :param distance: signed mm
        :param speed: mm/s
        :param accel: mm/s^2
        :param assist_active: whether the espooler runs
        """
        self._check_fault()
        if distance > 0:
            self.feed_starts.append((self.pos, self.loaded_to_hub))
        self.moves.append((distance, speed, accel, assist_active))
        self.pos += distance * (1.0 - self.slip) if distance > 0 else distance
        if distance > 0 and self.after_feed is not None:
            self.after_feed()

    def move_to(self, distance: float, speed_mode: Any, endstop: Any = None,
                assist_active: Any = None,
                use_homing: bool = True) -> Tuple[bool, float, None]:
        """
        :param distance: signed mm
        :param speed_mode: AFC's speed mode
        :param endstop: the endstop to home to
        :param assist_active: the espooler mode
        :param use_homing: home on the endstop
        :return tuple: (homed, distance, None)
        """
        self.homing_moves.append(
            (distance, speed_mode, endstop, use_homing, assist_active))
        if distance > 0:
            if self.raw_load_state:
                return True, 0.0, None
            self.pos = min(self.load_at + 0.5, self.pos + distance)
        else:
            self.pos = max(self.load_at - 0.5, self.pos + distance)
        return True, abs(distance), None

    def feeds(self) -> List[float]:
        """:return List[float]: the feed moves up to the first retract"""
        out = []
        for distance, _speed, _accel, _assist in self.moves:
            if distance < 0:
                break
            out.append(distance)
        return out

    def retracts(self) -> List[Tuple[float, float, Any]]:
        """:return list: (distance, speed, assist) of every retract"""
        return [(d, s, a) for d, s, _acc, a in self.moves if d < 0]


class BtRfidIdle:
    """idle_timeout: its state, or a get_status that raises."""

    def __init__(self, state: str, raises: bool = False) -> None:
        """
        :param state: "Ready", "Idle" or "Printing"
        :param raises: get_status raises instead
        """
        self.state = state
        self.raises = raises

    def get_status(self, eventtime: float) -> Dict[str, str]:
        """:return dict: the idle_timeout status"""
        if self.raises:
            raise KeyError("state")
        return {"state": self.state}


@dataclass
class BtRfidRig:
    """A built coordinator and the fakes around it."""

    printer: BambuPrinter
    unit: AFC_BoxTurtle_rfid
    field: BtRfidField
    port: BtRfidPico
    serial: BtRfidSerialModule
    readers: Dict[str, AFC_BoxTurtle_rfid_reader]

    @property
    def logger(self) -> BambuLogger:
        """:return BambuLogger: AFC's logger, which the module logs to"""
        return self.printer.afc.logger

    @property
    def console(self) -> List[LogLine]:
        """:return List[LogLine]: the gcode console, where a tag's read-out goes"""
        return self.printer.gcode.messages

    def logged(self) -> List[LogLine]:
        """:return List[LogLine]: AFC's logger lines, see bt_rfid_logged"""
        return bt_rfid_logged(self.logger)


BtRfidReaderSpec = Tuple[str, int, Tuple[str, ...]]


def build_bt_rfid(monkeypatch: pytest.MonkeyPatch, *,
                  readers: Sequence[BtRfidReaderSpec] = (
                      ("reader0", 0, ("lane8", "lane9")),),
                  lanes: Sequence[BtRfidLane] = (),
                  options: Optional[Dict[str, Any]] = None,
                  ready: bool = True, online: bool = True,
                  connected: bool = True, homing: bool = True,
                  idle: Optional[BtRfidIdle] = None,
                  port: Optional[BtRfidPico] = None) -> BtRfidRig:
    """
    Build ``[AFC_BoxTurtle_rfid]`` and its reader sections the way klippy does.

    The sweep knobs default to 20mm/s, 20mm chunks and a 75mm sister roll.

    :param monkeypatch: patches read_tag and pyserial in the module, and
        AFC_RFID's wall clock
    :param readers: (name, bus, lanes) per reader section
    :param lanes: AFC lanes to register
    :param options: coordinator options over the defaults here
    :param ready: run klippy:ready's handler
    :param online: give every reader the version its connect probe reads
    :param connected: the bridge port is open, as the connect timer leaves it
    :param homing: AFC's homing_enabled
    :param idle: the idle_timeout object; a Ready one when None
    :param port: the Pico's port; one answering 0x92 on both buses when None
    :return BtRfidRig: the coordinator and its fakes
    """
    printer = BambuPrinter()
    reactor = BtRfidReactor(now=100.0)
    printer.reactor = printer._reactor = printer.afc.reactor = reactor
    printer.afc.homing_enabled = homing
    printer.add_object("idle_timeout", idle or BtRfidIdle("Ready"))
    values: Dict[str, Any] = {"serial": BT_RFID_SERIAL,
                              "tag_sweep_speed": 20.0,
                              "tag_sweep_step_mm": 20.0,
                              "sibling_tag_adjust_dist": 75.0}
    values.update(options or {})
    unit = AFC_BoxTurtle_rfid(BambuConfig("AFC_BoxTurtle_rfid", printer, values))
    printer.add_object("AFC_BoxTurtle_rfid", unit)
    built: Dict[str, AFC_BoxTurtle_rfid_reader] = {}
    for name, bus, served in readers:
        section = f"AFC_BoxTurtle_rfid {name}"
        built[name] = AFC_BoxTurtle_rfid_reader(BambuConfig(
            section, printer, {"bus": bus, "lanes": ", ".join(served)}))
        printer.add_object(section, built[name])
    for lane in lanes:
        printer.afc.lanes[lane.name] = lane
    field = BtRfidField()
    monkeypatch.setattr(bt_rfid_mod, "read_tag", field)
    monkeypatch.setattr(afc_rfid_mod, "time", BtRfidWallClock(BT_RFID_WALL_TIME))
    port = port or BtRfidPico()
    serial = BtRfidSerialModule(port)
    monkeypatch.setattr(bt_rfid_mod, "serial", serial)
    if ready:
        unit._handle_ready()
        if connected:
            unit.bridge._ser = port
        if online:
            for rdr in built.values():
                rdr.version = 0x92
    return BtRfidRig(printer, unit, field, port, serial, built)


def bt_rfid_hub_rig(monkeypatch: pytest.MonkeyPatch, *, staged: bool = False,
                    busy: bool = False, dist_hub: float = 150.0,
                    homing: bool = True) -> Tuple[BtRfidRig, BtRfidLane]:
    """
    lane9 on reader0's antenna and on hub Turtle_1, with lane8 as its sister.

    :param monkeypatch: for build_bt_rfid
    :param staged: lane9 is staged at the hub, dist_hub past its load switch
    :param busy: lane8 is loaded to the toolhead through the same hub
    :param dist_hub: lane9's dist_hub
    :param homing: AFC's homing_enabled
    :return tuple: (rig, lane9)
    """
    hub = BtRfidHub()
    lane = BtRfidLane("lane9", dist_hub=dist_hub, hub=hub, staged=staged)
    if staged:
        lane.pos += dist_hub
    lanes = [lane]
    if busy:
        lanes.append(BtRfidLane("lane8", dist_hub=150.0, hub=hub,
                                tool_loaded=True))
        hub.state = True
    return build_bt_rfid(monkeypatch, lanes=lanes, homing=homing), lane


def bt_rfid_sister_rig(monkeypatch: pytest.MonkeyPatch, *,
                       parked: Optional[str] = "CAFE1234",
                       sib_pos: float = 200.0, dist_hub: float = 200.0,
                       hub_at: Optional[float] = None, slip: float = 0.0,
                       homing: bool = True
                       ) -> Tuple[BtRfidRig, BtRfidLane, BtRfidLane]:
    """
    lane8 being read and lane9, its sister on the shared antenna, staged at
    her hub: dist_hub past her load switch, short of the hub switch.

    :param monkeypatch: for build_bt_rfid
    :param parked: the UID the parked-tag probe sees, or None for a clear coil
    :param sib_pos: where the sister's tip is
    :param dist_hub: the sister's dist_hub
    :param hub_at: where the sister's tip trips her hub; dist_hub + 40 if None
    :param slip: the sister's feed slip
    :param homing: AFC's homing_enabled
    :return tuple: (rig, lane8, lane9)
    """
    hub = BtRfidHub()
    lane = BtRfidLane("lane8")
    sib = BtRfidLane("lane9", pos=sib_pos, slip=slip, dist_hub=dist_hub,
                     hub=hub, staged=True)
    hub.watch(sib, dist_hub + 40.0 if hub_at is None else hub_at)
    rig = build_bt_rfid(monkeypatch, lanes=[lane, sib], homing=homing)
    rig.field.parked = parked
    return rig, lane, sib


class TestBridgeSerialRequest:
    """request() returns a value for everything the stream can carry and
    never raises into Klipper."""

    CONNECTED = ("info", "BT RFID: bridge connected on /dev/test")

    def test_a_read_reply_returns_its_payload(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"=92\n"], logger=logger)
        assert bridge.request("r0 37") == "92"
        assert bridge._ser.written == [b"r0 37\n"]
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_a_write_ack_returns_empty_string(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"=\n"], logger=logger)
        assert bridge.request("w0 2A 8D") == ""
        assert bridge._ser.written == [b"w0 2A 8D\n"]
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_json_events_on_the_stream_are_skipped(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial(
            [b'{"evt":"hello","fw":"RFID-0.2"}\n', b"=92\n"], logger=logger)
        assert bridge.request("r0 37") == "92"
        assert bridge._ser.lines == []
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_a_reply_past_eight_other_lines_is_none(self):
        logger = BambuLogger()
        events = [b'{"evt":"tick"}\n'] * 8
        bridge = make_bt_bridge_serial(events + [b"=92\n"], logger=logger)
        assert bridge.request("r0 37") is None
        assert bridge._ser.lines == [b"=92\n"]
        assert bridge.connected() is True
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_a_nak_is_none(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"!\n", b"=92\n"], logger=logger)
        assert bridge.request("r0 37") is None
        assert bridge._ser.lines == [b"=92\n"]
        assert bridge.connected() is True
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_a_timeout_is_none(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([], logger=logger)
        assert bridge.request("r0 37") is None
        assert bridge._ser.written == [b"r0 37\n"]
        assert bridge.connected() is True
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_no_port_is_none_not_an_exception(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial(logger=logger, connect=False)
        assert bridge.request("r0 37") is None
        assert bridge._ser is None
        assert bt_rfid_logged(logger) == []

    def test_a_dying_port_drops_the_connection(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"=92\n"], logger=logger)
        port = bridge._ser
        port.write = Recorder(raises=OSError("gone"))
        assert bridge.request("r0 37") is None
        assert bridge._ser is None
        assert port.closed is True
        assert bt_rfid_logged(logger) == [
            self.CONNECTED,
            ("warning", "BT RFID: bridge dropped on /dev/test, retrying in the "
                        "background")]


class TestSerialRegLinkRegRead:
    """reg_read speaks "r<bus> <reg>" in hex and treats every kind of silence
    as the reader not answering. It logs nothing itself."""

    CONNECTED = ("info", "BT RFID: bridge connected on /dev/test")

    def test_reg_read_sends_r_line_and_parses_hex(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"=92\n"], logger=logger)
        assert _SerialRegLink(bridge, 0).reg_read(0x37) == 146
        assert bridge._ser.written == [b"r0 37\n"]
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_the_bus_number_rides_in_the_line(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"=1A\n"], logger=logger)
        assert _SerialRegLink(bridge, 1).reg_read(0x0A) == 26
        assert bridge._ser.written == [b"r1 0A\n"]
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_no_answer_raises_for_the_read_stack_to_catch(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([], logger=logger)
        with pytest.raises(OSError) as err:
            _SerialRegLink(bridge, 0).reg_read(0x37)
        assert str(err.value) == "rfid bridge bus 0: no answer reading reg 0x37"
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_an_empty_reply_is_no_answer_for_a_read(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"=\n"], logger=logger)
        with pytest.raises(OSError) as err:
            _SerialRegLink(bridge, 1).reg_read(0x2A)
        assert str(err.value) == "rfid bridge bus 1: no answer reading reg 0x2A"
        assert bt_rfid_logged(logger) == [self.CONNECTED]


class TestSerialRegLinkRegWrite:
    """reg_write speaks "w<bus> <reg> <val>"; only a missing ack fails it. It
    logs nothing itself."""

    CONNECTED = ("info", "BT RFID: bridge connected on /dev/test")

    def test_reg_write_sends_w_line(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"=\n"], logger=logger)
        assert _SerialRegLink(bridge, 0).reg_write(0x2A, 0x8D) is None
        assert bridge._ser.written == [b"w0 2A 8D\n"]
        assert bt_rfid_logged(logger) == [self.CONNECTED]

    def test_a_failed_write_raises_too(self):
        logger = BambuLogger()
        bridge = make_bt_bridge_serial([b"!\n"], logger=logger)
        with pytest.raises(OSError) as err:
            _SerialRegLink(bridge, 1).reg_write(0x2A, 0x8D)
        assert str(err.value) == "rfid bridge bus 1: no answer writing reg 0x2A"
        assert bridge._ser.written == [b"w1 2A 8D\n"]
        assert bt_rfid_logged(logger) == [self.CONNECTED]


class TestAFCBoxTurtlerfidInit:
    """The coordinator takes AFC's logger at construction and touches no
    hardware."""

    def test_the_module_takes_afcs_logger_at_construction(self, monkeypatch):
        printer = BambuPrinter()
        afc = printer.afc
        # AFC is reachable through load_object only, as when this section is
        # reached before AFC's: a lookup_object would come back empty.
        printer._afc = None
        printer._objects["AFC"] = afc
        serial = BtRfidSerialModule(BtRfidPico())
        monkeypatch.setattr(bt_rfid_mod, "serial", serial)
        unit = AFC_BoxTurtle_rfid(BambuConfig("AFC_BoxTurtle_rfid", printer,
                                              {"serial": BT_RFID_SERIAL}))
        assert printer.lookup_object("AFC") is None
        assert unit.logger is afc.logger
        assert unit.bridge.logger is afc.logger
        assert unit.afc is None
        assert unit.bridge.port == BT_RFID_SERIAL
        assert unit.bridge.connected() is False
        assert serial.opened == []
        assert bt_rfid_logged(afc.logger) == []

    def test_the_commands_and_handlers_are_registered(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, ready=False)
        unit, printer = rig.unit, rig.printer
        handlers = printer.gcode.ready_gcode_handlers
        assert handlers == {"AFC_BT_RFID_READ": unit.cmd_AFC_BT_RFID_READ,
                            "AFC_BT_RFID_STATUS": unit.cmd_AFC_BT_RFID_STATUS,
                            "AFC_BT_RFID_STAGE": unit.cmd_AFC_BT_RFID_STAGE}
        assert printer._event_handlers["klippy:ready"] == [unit._handle_ready]
        assert printer._event_handlers["afc:lane_prep_loaded"] == [
            unit._on_lane_prep_loaded]
        assert rig.logged() == []

    def test_the_section_knobs_and_keys_are_parsed(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, ready=False, options={
            "bambu_master_key": " 00112233445566778899aabbccddeeff ",
            "creality_key": "", "creality_encryption_key": "0102",
            "auto_spoolman_create": True, "spool_diameter_mm": 180.0,
            "tag_sweep_revs": 3.0, "read_timeout_s": 4.5,
            "retract_after_read": False, "tag_retract_speed": 60.0,
            "scan_on_insert": False, "sibling_tag_adjust": False})
        unit = rig.unit
        assert unit.serial_port == BT_RFID_SERIAL
        assert unit.bambu_master_key == bytes(range(0, 256, 17))
        assert unit.creality_key is None
        assert unit.creality_encryption_key == b"\x01\x02"
        assert unit.auto_create is True
        assert unit.spool_diameter_mm == 180.0
        assert unit.tag_sweep_revs == 3.0
        assert unit.tag_sweep_speed == 20.0
        assert unit.tag_sweep_step_mm == 20.0
        assert unit.read_timeout_s == 4.5
        assert unit.retract_after_read is False
        assert unit.tag_retract_speed == 60.0
        assert unit.scan_on_insert is False
        assert unit.sibling_tag_adjust is False
        assert unit.sibling_tag_adjust_dist == 75.0
        assert rig.logged() == []

    def test_unset_knobs_take_their_defaults(self, monkeypatch):
        printer = BambuPrinter()
        monkeypatch.setattr(bt_rfid_mod, "serial", BtRfidSerialModule(BtRfidPico()))
        unit = AFC_BoxTurtle_rfid(BambuConfig("AFC_BoxTurtle_rfid", printer,
                                              {"serial": BT_RFID_SERIAL}))
        assert (unit.bambu_master_key, unit.creality_key,
                unit.creality_encryption_key) == (None, None, None)
        assert unit.auto_create is False
        assert unit.spool_diameter_mm == 200.0
        assert unit.tag_sweep_revs == 2.0
        assert unit.tag_sweep_speed == 100.0
        assert unit.tag_sweep_step_mm == 0.0
        assert unit.read_timeout_s == 6.0
        assert unit.retract_after_read is True
        assert unit.tag_retract_speed == 0.0
        assert unit.scan_on_insert is True
        assert unit.sibling_tag_adjust is True
        assert unit.sibling_tag_adjust_dist == 45.0
        assert (unit._sweeping, unit._probing, unit._baseline_uid) == (
            False, False, None)
        assert (unit._readers, unit._reader_by_lane, unit._last_uid_by_lane) == (
            [], {}, {})
        assert unit.log_prefix == "BT RFID"
        assert bt_rfid_logged(printer.afc.logger) == []


class TestAFCBoxTurtlerfidWriteExcluder:
    """A staged write passes over the sister lane's known tag."""

    def test_the_write_passes_over_the_sister_lanes_known_tag(self, monkeypatch):
        lane8, lane9 = BtRfidLane("lane8"), BtRfidLane("lane9")
        rig = build_bt_rfid(monkeypatch, lanes=[lane8, lane9])
        rig.unit._last_uid_by_lane["lane8"] = "5157e12"
        excl = rig.unit._write_excluder((lane9, 0.0, False, None))
        assert excl("5157E12") is True
        assert excl("04ab") is False
        assert rig.logged() == []

    def test_no_known_sister_tag_means_no_excluder(self, monkeypatch):
        lane9 = BtRfidLane("lane9")
        rig = build_bt_rfid(monkeypatch, lanes=[lane9])
        rig.unit._last_uid_by_lane["lane9"] = "04ab"
        assert rig.unit._write_excluder((lane9, 0.0, False, None)) is None
        assert rig.logged() == []

    def test_a_lane_no_reader_serves_has_no_excluder(self, monkeypatch):
        lane12 = BtRfidLane("lane12")
        rig = build_bt_rfid(monkeypatch, lanes=[lane12])
        rig.unit._last_uid_by_lane["lane8"] = "5157e12"
        assert rig.unit._write_excluder((lane12, 0.0, False, None)) is None
        assert rig.logged() == []


class TestAFCBoxTurtlerfidApplyWrittenTag:
    """A written tag is applied like a scan and its UID noted."""

    def test_the_written_tag_is_applied_and_noted(self, monkeypatch):
        lane9 = BtRfidLane("lane9")
        rig = build_bt_rfid(monkeypatch, lanes=[lane9])
        rig.unit.apply_written_tag("lane9", {"uid": "04AB", "filament": {}})
        assert rig.console == [bt_rfid_read_out("lane9", "04AB")]
        assert lane9.weight == 1000
        assert rig.unit.last_reads_status() == {"lane9": bt_rfid_record("04AB")}
        assert rig.unit._last_uid_by_lane == {"lane9": "04ab"}
        assert rig.logged() == []

    def test_a_tag_without_a_uid_is_noted_as_blank(self, monkeypatch):
        lane9 = BtRfidLane("lane9")
        rig = build_bt_rfid(monkeypatch, lanes=[lane9])
        rig.unit.apply_written_tag("lane9", {"uid": None})
        assert rig.console == [bt_rfid_read_out("lane9", None)]
        assert rig.unit.last_reads_status() == {"lane9": bt_rfid_record(None)}
        assert rig.unit._last_uid_by_lane == {"lane9": ""}
        assert rig.logged() == []

    def test_a_name_that_is_not_an_afc_lane_raises(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        with pytest.raises(RuntimeError) as err:
            rig.unit.apply_written_tag("lane12", {"uid": "04AB"})
        assert str(err.value) == "lane12 is not an AFC lane"
        assert rig.console == []
        assert rig.unit.last_reads_status() == {}
        assert rig.unit._last_uid_by_lane == {}
        assert rig.logged() == []


class TestAFCBoxTurtlerfidStageForWrite:
    """Write staging spins the tag onto the antenna the way Scan Tag does: a
    staged lane comes back to its load switch, the sister's parked tag is
    rolled off, a busy hub bounds the sweep, and a tool-loaded lane is
    refused."""

    NOT_FOUND = ("info", "AFC_BT_RFID: staged lane9 for write: tag NOT found "
                         "after 1257mm (re-centred 0mm); holding for the write.")

    def test_write_staging_brings_a_staged_lane_back_and_restages_it(
            self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, staged=True)
        unit = rig.unit
        token = unit._stage_for_write("lane9")
        assert token == (lane, pytest.approx(2 * BT_RFID_TURN_MM), True, None)
        # Homed back first: the sweep began on the load switch edge, unstaged.
        assert lane.homing_moves[0][:4] == (-170.0, SpeedMode.LONG, "load", True)
        assert lane.feed_starts[0] == (0.5, False)
        assert sum(lane.feeds()) == pytest.approx(2 * BT_RFID_TURN_MM)
        assert unit._sweeping is True
        assert rig.logged() == [self.NOT_FOUND]
        unit._unstage_after_write(token)
        assert lane.loaded_to_hub is True
        assert lane.unit_obj.calls == ["prep_load", "prep_load", "prep_post_load"]
        assert lane.pos == pytest.approx(0.5 + 150.0)
        assert unit._sweeping is False
        assert rig.logged() == [self.NOT_FOUND]

    def test_write_staging_rolls_the_sister_tag_off_and_back(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        unit = rig.unit
        token = unit._stage_for_write("lane8")
        rolled = [(-10.0, 100.0, 400, True)] * 7 + [(-5.0, 100.0, 400, True)]
        assert sib.moves == rolled
        assert token[3] == (sib, 75.0, 100.0)
        # The parked-tag probe ran before lane8 fed anything.
        assert rig.field.calls[0][1] is True
        unit._unstage_after_write(token)
        assert sib.moves == rolled + [(75.0, 100.0, 400, False),
                                      (200.0, 100.0, 400, False)]
        assert sib.loaded_to_hub is True
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag CAFE1234 is parked on reader0's antenna "
                     "while lane8 is read, clearing lane9 off it."),
            ("info", "AFC_BT_RFID: rolled lane9 back 75mm to clear its tag off "
                     "reader0's antenna while lane8 is read."),
            ("info", "AFC_BT_RFID: staged lane8 for write: tag NOT found after "
                     "1257mm (re-centred 0mm); holding for the write."),
            ("info", "AFC_BT_RFID: re-staged lane9 the way a load does: homed to "
                     "its load switch, then fed dist_hub (200mm). The hub is "
                     "untouched.")]

    def test_write_staging_refuses_a_lane_in_the_toolhead(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        lane.tool_loaded = True
        with pytest.raises(StageError) as err:
            rig.unit._stage_for_write("lane9")
        assert str(err.value) == ("lane9 is loaded in the toolhead. The write "
                                  "turns the spool by moving its filament, so "
                                  "unload it first.")
        assert lane.moves == [] and lane.homing_moves == []
        assert rig.unit._sweeping is False
        assert rig.logged() == []

    def test_write_staging_refuses_a_staged_lane_without_homing(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, staged=True, homing=False)
        with pytest.raises(StageError) as err:
            rig.unit._stage_for_write("lane9")
        assert str(err.value) == ("lane9 is staged at the hub and homing is off, "
                                  "so there is no load switch to bring it back "
                                  "to. Eject it and insert it again to write it.")
        assert lane.moves == [] and lane.homing_moves == []
        assert rig.unit._sweeping is False
        assert rig.logged() == []

    def test_an_unstaged_lane_without_homing_is_swept(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, homing=False)
        token = rig.unit._stage_for_write("lane9")
        assert token == (lane, pytest.approx(2 * BT_RFID_TURN_MM), False, None)
        assert lane.homing_moves == []
        assert rig.logged() == [self.NOT_FOUND]

    def test_write_staging_on_a_busy_hub_gets_the_room_from_the_load_switch(
            self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, staged=True, busy=True)
        unit = rig.unit
        token = unit._stage_for_write("lane9")
        # 150mm dist_hub less the 25mm margin, counted from the load switch.
        assert lane.feeds() == [20.0] * 6 + [5.0]
        assert token[1] == 125.0
        unit._unstage_after_write(token)
        assert lane.loaded_to_hub is True
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: stage-for-write limited: hub Turtle_1 is in "
                     "use (lane8 is loaded through it), so the sweep stops 25mm "
                     "short of it."),
            ("info", "AFC_BT_RFID: staged lane9 for write: tag NOT found after "
                     "125mm (re-centred 0mm); holding for the write.")]

    def test_write_staging_refuses_when_there_is_no_room(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, busy=True, dist_hub=20.0)
        with pytest.raises(StageError) as err:
            rig.unit._stage_for_write("lane9")
        assert str(err.value) == ("hub Turtle_1 is in use (lane8 is loaded through "
                                  "it), so the sweep stops 25mm short of it, and "
                                  "lane9's dist_hub leaves no room before it. "
                                  "Unload that lane first to write this one.")
        assert lane.moves == []
        assert rig.unit._sweeping is False
        assert rig.logged() == []

    def test_a_failed_sweep_releases_the_guard_without_restaging(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, staged=True)
        lane.fail_after, lane.fault = 0, "stepper fault"
        with pytest.raises(RuntimeError) as err:
            rig.unit._stage_for_write("lane9")
        assert str(err.value) == "stepper fault"
        assert rig.unit._sweeping is False
        assert lane.loaded_to_hub is False
        assert lane.unit_obj.calls == ["prep_load"]
        assert rig.logged() == []

    def test_a_failed_sweep_still_puts_the_sister_back(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        lane.fail_after = 0
        with pytest.raises(RuntimeError) as err:
            rig.unit._stage_for_write("lane8")
        assert str(err.value) == "stepper gone"
        assert lane.moves == []
        assert sib.moves == ([(-10.0, 100.0, 400, True)] * 7
                             + [(-5.0, 100.0, 400, True)]
                             + [(75.0, 100.0, 400, False),
                                (200.0, 100.0, 400, False)])
        assert sib.loaded_to_hub is True
        assert rig.unit._sweeping is False
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag CAFE1234 is parked on reader0's antenna "
                     "while lane8 is read, clearing lane9 off it."),
            ("info", "AFC_BT_RFID: rolled lane9 back 75mm to clear its tag off "
                     "reader0's antenna while lane8 is read."),
            ("info", "AFC_BT_RFID: re-staged lane9 the way a load does: homed to "
                     "its load switch, then fed dist_hub (200mm). The hub is "
                     "untouched.")]

    def test_no_configured_step_sizes_the_chunk_from_the_lane(self, monkeypatch):
        lane = BtRfidLane("lane9")
        lane.long_moves_accel = 250
        rig = build_bt_rfid(monkeypatch, lanes=[lane], options={
            "tag_sweep_step_mm": 0.0, "tag_sweep_speed": 100.0})
        token = rig.unit._stage_for_write("lane9")
        # (100^2 / 250) / 0.4 = 100mm chunks over the 1256.6mm bound.
        assert lane.feeds() == [100.0] * 12 + [pytest.approx(56.637061)]
        assert token == (lane, pytest.approx(1256.637061), False, None)
        assert rig.logged() == [self.NOT_FOUND]

    def test_a_staged_lane_that_will_not_come_back_is_a_stage_error(
            self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, staged=True)
        # prep_load's forward home never makes the switch.
        lane.unit_obj.prep_load = Recorder()
        with pytest.raises(StageError) as err:
            rig.unit._stage_for_write("lane9")
        assert str(err.value) == ("lane9 did not come back onto its load switch. "
                                  "Check the spool; its next load will re-home it")
        assert rig.unit._sweeping is False
        assert lane.unit_obj.prep_load.calls == [((lane,), {})]
        assert lane.loaded_to_hub is False
        assert lane.moves == []
        assert rig.field.calls == []
        assert rig.logged() == []

    def test_write_staging_refuses_a_lane_on_the_other_reader(self, monkeypatch):
        lane = BtRfidLane("lane9")
        rig = build_bt_rfid(monkeypatch, lanes=[lane], readers=(
            ("reader0", 0, ("lane8", "lane9")),
            ("reader1", 1, ("lane10", "lane11"))))
        with pytest.raises(StageError) as err:
            rig.unit._stage_for_write("lane9", rig.readers["reader1"])
        assert str(err.value) == "lane9 is not on reader1; write it with READER=bt:reader0"
        assert lane.moves == []
        assert rig.logged() == []

    def test_the_reader_it_was_sent_to_is_accepted(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        token = rig.unit._stage_for_write("lane9", rig.readers["reader0"])
        assert token[0] is lane
        assert rig.logged() == [self.NOT_FOUND]

    def test_an_unknown_lane_or_offline_reader_is_skipped(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        unit = rig.unit
        lane12 = BtRfidLane("lane12")            # an AFC lane no reader serves
        rig.printer.afc.lanes["lane12"] = lane12
        # Each of the three conditions alone skips it.
        assert unit._stage_for_write("lane8") is None
        assert unit._stage_for_write("lane12") is None
        rig.readers["reader0"].version = None
        assert unit._stage_for_write("lane9") is None
        assert lane.moves == [] and lane12.moves == []
        assert unit._sweeping is False
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: stage-for-write skipped lane8 (lane=False, "
                     "rdr=True, ver=146)"),
            ("info", "AFC_BT_RFID: stage-for-write skipped lane12 (lane=True, "
                     "rdr=False, ver=None)"),
            ("info", "AFC_BT_RFID: stage-for-write skipped lane9 (lane=True, "
                     "rdr=True, ver=None)")]

    def test_a_target_for_a_lane_no_reader_serves_is_skipped_not_refused(
            self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        assert rig.unit._stage_for_write("lane12", rig.readers["reader0"]) is None
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: stage-for-write skipped lane12 (lane=False, "
                     "rdr=False, ver=None)")]

    def test_a_sweep_already_running_is_not_joined(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        rig.unit._sweeping = True
        assert rig.unit._stage_for_write("lane9") is None
        assert lane.moves == []
        assert rig.unit._sweeping is True
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: stage-for-write busy, skipping lane9")]

    def test_write_staging_settles_on_a_found_tag(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        rig.field.appear(lane, after_mm=10.0, until_mm=100.0)
        token = rig.unit._stage_for_write("lane9")
        # Found after the first 20mm chunk, then eased back: three 3mm steps
        # hold, the fourth loses it and is undone, so 20 - 9 = 11mm.
        assert lane.moves == [(20.0, 20.0, 400, False)] + [
            (-3.0, 25.0, 400, True)] * 4 + [(3.0, 25.0, 400, True)]
        assert token == (lane, 11.0, False, None)
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: staged lane9 for write: tag found DEADBEEF "
                     "after 11mm (re-centred -9mm); holding for the write.")]
        rig.unit._unstage_after_write(token)
        assert rig.unit._sweeping is False

    @staticmethod
    def _read_once_then_gone(rig: BtRfidRig, lane: BtRfidLane) -> None:
        """
        Make the tag drop out of the field for good once it has been read.

        :param rig: the rig whose field holds the tag
        :param lane: the lane being swept
        """

        def gone_once_read() -> None:
            present = rig.field.present()
            rig.field.hold_for_poller()
            if present:
                rig.field.lane = None

        lane.after_feed = gone_once_read

    def test_a_tag_that_never_rereads_keeps_the_search(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        rig.field.appear(lane, after_mm=50.0)
        self._read_once_then_gone(rig, lane)
        token = rig.unit._stage_for_write("lane9")
        # Read on the third 20mm chunk, then never again: the settle searches
        # back the 20mm chunk plus 25mm, fifteen 3mm steps, so 60 - 45 = 15mm.
        assert lane.moves == [(20.0, 20.0, 400, False)] * 3 + [
            (-3.0, 25.0, 400, True)] * 15
        assert token == (lane, 15.0, False, None)
        assert rig.unit._sweeping is True
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: staged lane9 for write: tag found DEADBEEF "
                     "after 15mm (re-centred -45mm); holding for the write.")]
        rig.unit._unstage_after_write(token)
        assert lane.homing_moves == [
            (-35.0, SpeedMode.LONG, "load", True, AssistActive.YES),
            (400.0, None, "load", True, None)]
        assert lane.pos == 0.5
        assert rig.unit._sweeping is False

    def test_a_tag_read_on_the_first_chunk_still_comes_back_to_the_switch(
            self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        rig.field.appear(lane, after_mm=10.0)
        self._read_once_then_gone(rig, lane)
        token = rig.unit._stage_for_write("lane9")
        # The 45mm search leaves 20 - 45 = -25mm, the tip 20mm behind its
        # load switch, so the restore homes it forward onto the switch.
        assert lane.moves == [(20.0, 20.0, 400, False)] + [
            (-3.0, 25.0, 400, True)] * 15
        assert token == (lane, -25.0, False, None)
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: staged lane9 for write: tag found DEADBEEF "
                     "after -25mm (re-centred -45mm); holding for the write.")]
        assert lane.raw_load_state is False
        rig.unit._unstage_after_write(token)
        assert lane.unit_obj.calls == ["prep_load"]
        assert lane.homing_moves == [(400.0, None, "load", True, None)]
        assert lane.pos == 0.5
        assert rig.unit._sweeping is False

    def test_a_tag_settled_behind_the_start_is_homed_back_onto_the_switch(
            self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        rig.field.appear(lane, after_mm=10.0)

        def settle_reads() -> None:
            # Once the sweep has read it, the settle's reads: lost at first,
            # found on the seventh 3mm step back, then held for all six eases.
            present = rig.field.present()
            rig.field.hold_for_poller()
            if present:
                rig.field.script = [False] * 7 + [True] * 7
                lane.after_feed = None

        lane.after_feed = settle_reads
        token = rig.unit._stage_for_write("lane9")
        # 20mm fed, 21mm searched and 18mm eased back: 20 - 39 = -19mm.
        assert lane.moves == [(20.0, 20.0, 400, False)] + [
            (-3.0, 25.0, 400, True)] * 13
        assert token == (lane, -19.0, False, None)
        assert rig.field.script == []
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: staged lane9 for write: tag found DEADBEEF "
                     "after -19mm (re-centred -39mm); holding for the write.")]
        assert lane.raw_load_state is False
        rig.unit._unstage_after_write(token)
        assert lane.unit_obj.calls == ["prep_load"]
        assert lane.homing_moves == [(400.0, None, "load", True, None)]
        assert lane.pos == 0.5
        assert rig.unit._sweeping is False


class TestAFCBoxTurtlerfidSettleOnTag:
    """After a sweep stops up to a chunk past the tag, ease it back to
    mid-field so a write couples: 3mm retracts at the lane's short speed."""

    @staticmethod
    def _settle(monkeypatch: pytest.MonkeyPatch, reads: List[bool],
                back_limit: float = 30.0) -> Tuple[float, BtRfidLane, BtRfidRig]:
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        rig.field.script = list(reads)
        moved = rig.unit._settle_on_tag(lane, "lane9", rig.readers["reader0"],
                                        back_limit)
        return moved, lane, rig

    def test_a_tag_still_in_range_is_eased_to_mid_field(self, monkeypatch):
        # Reads at once, holds two more steps, drops on the third.
        moved, lane, rig = self._settle(monkeypatch, [True, True, True, False])
        assert lane.moves == [(-3.0, 25.0, 400, True)] * 3 + [(3.0, 25.0, 400, True)]
        assert moved == -6.0
        assert rig.field.script == []
        assert rig.logged() == []

    def test_an_overshot_tag_is_backed_onto_first(self, monkeypatch):
        moved, lane, rig = self._settle(monkeypatch, [False, False, True, False])
        assert lane.moves == [(-3.0, 25.0, 400, True)] * 3 + [(3.0, 25.0, 400, True)]
        assert moved == -6.0
        assert rig.logged() == []

    def test_a_tag_that_never_rereads_is_left_where_the_search_stopped(
            self, monkeypatch):
        # The search stops at the limit and reports it, for the restore.
        moved, lane, rig = self._settle(monkeypatch, [False] * 5, back_limit=9.0)
        assert lane.moves == [(-3.0, 25.0, 400, True)] * 3
        assert moved == -9.0
        assert rig.field.script == [False]
        assert rig.logged() == []

    def test_no_search_room_moves_nothing(self, monkeypatch):
        moved, lane, rig = self._settle(monkeypatch, [False, False], back_limit=0.0)
        assert lane.moves == []
        assert moved == 0.0
        assert rig.field.script == [False]
        assert rig.logged() == []

    def test_a_read_that_always_holds_stops_after_six_steps(self, monkeypatch):
        moved, lane, rig = self._settle(monkeypatch, [True] * 8)
        assert lane.moves == [(-3.0, 25.0, 400, True)] * 6
        assert moved == -18.0
        assert rig.field.script == [True]
        assert rig.logged() == []

    def test_a_lane_without_a_short_speed_eases_at_the_sweep_speed(
            self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        lane.short_moves_speed = None
        rig.field.script = [True, False]
        moved = rig.unit._settle_on_tag(lane, "lane9", rig.readers["reader0"], 30.0)
        assert lane.moves == [(-3.0, 20.0, 400, True), (3.0, 20.0, 400, True)]
        assert moved == 0.0
        assert rig.logged() == []


class TestAFCBoxTurtlerfidUnstageAfterWrite:
    """After the write the filament goes back where staging found it and the
    sweep guard is released."""

    def test_write_staging_leaves_an_unstaged_lane_unstaged(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        rig.unit._unstage_after_write(rig.unit._stage_for_write("lane9"))
        assert lane.loaded_to_hub is False
        assert lane.unit_obj.calls == ["prep_load"]
        # Homed back, then forward onto the load switch edge.
        assert lane.homing_moves[0][0] == pytest.approx(-(2 * BT_RFID_TURN_MM + 20.0))
        assert lane.pos == 0.5
        assert rig.unit._sweeping is False
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: staged lane9 for write: tag NOT found after "
                     "1257mm (re-centred 0mm); holding for the write.")]

    def test_no_token_only_releases_the_guard(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        rig.unit._sweeping = True
        rig.unit._unstage_after_write(None)
        assert rig.unit._sweeping is False
        assert lane.moves == [] and lane.homing_moves == []
        assert rig.logged() == []

    def test_the_configured_retract_speed_drives_the_return(self, monkeypatch):
        hub = BtRfidHub()
        lane = BtRfidLane("lane9", pos=105.0, hub=hub, dist_hub=150.0)
        rig = build_bt_rfid(monkeypatch, lanes=[lane], homing=False,
                            options={"tag_retract_speed": 60.0})
        rig.unit._sweeping = True
        rig.unit._unstage_after_write((lane, 100.0, False, None))
        # Without homing: the bulk at the retract speed, the 30mm tail in
        # short steps, then a creep back onto the switch.
        assert lane.moves == ([(-70.0, 60.0, 400, True)]
                              + [(-10.0, 25.0, 400, True)] * 4
                              + [(10.0, 25.0, 400, False)])
        assert lane.pos == 5.0
        assert rig.unit._sweeping is False
        assert rig.logged() == []

    def test_the_lane_long_move_speed_drives_the_return_by_default(
            self, monkeypatch):
        lane = BtRfidLane("lane9", pos=105.0)
        rig = build_bt_rfid(monkeypatch, lanes=[lane], homing=False)
        rig.unit._sweeping = True
        rig.unit._unstage_after_write((lane, 100.0, False, None))
        # No retract speed configured: the lane's 100mm/s, not the 20mm/s sweep.
        assert lane.moves == ([(-70.0, 100.0, 400, True)]
                              + [(-10.0, 25.0, 400, True)] * 4
                              + [(10.0, 25.0, 400, False)])
        assert lane.pos == 5.0
        assert rig.unit._sweeping is False
        assert rig.logged() == []

    def test_a_lane_left_off_its_switch_is_not_restaged(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        lane.pos = 105.0
        lane.unit_obj.prep_load = Recorder()     # the forward home never lands
        rig.unit._sweeping = True
        rig.unit._unstage_after_write((lane, 100.0, True, None))
        assert lane.homing_moves == [
            (-120.0, SpeedMode.LONG, "load", True, AssistActive.YES)]
        assert lane.loaded_to_hub is False
        assert lane.unit_obj.calls == []
        assert rig.unit._sweeping is False
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: lane9 is not on its load switch after the "
                        "scan, so it was not re-staged. Its next load will re-home "
                        "it.")]

    def test_with_no_speeds_set_the_return_runs_at_the_sweep_speed(self, monkeypatch):
        lane = BtRfidLane("lane9", pos=105.0)
        lane.long_moves_speed = None
        rig = build_bt_rfid(monkeypatch, lanes=[lane], homing=False)
        rig.unit._unstage_after_write((lane, 100.0, False, None))
        assert lane.moves[0] == (-70.0, 20.0, 400, True)
        assert rig.logged() == []

    def test_a_restore_failure_still_releases_the_guard(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, homing=False)
        lane.pos, lane.fail_after = 105.0, 0
        rig.unit._sweeping = True
        with pytest.raises(RuntimeError) as err:
            rig.unit._unstage_after_write((lane, 100.0, True, None))
        assert str(err.value) == "stepper gone"
        assert rig.unit._sweeping is False
        assert lane.unit_obj.calls == []
        assert rig.logged() == []


class TestAFCBoxTurtlerfidHandleReady:
    """klippy:ready collects the readers, wires their links, registers them as
    write targets and starts the connect timer, touching no hardware."""

    TWO_READERS = (("reader0", 0, ("lane8", "lane9")),
                   ("reader1", 1, ("lane10", "lane11")))

    class _BareAfc:
        """An AFC object with no logger."""

    class _SharedKeys:
        """``[AFC_rfid_keys]``: the brand keys every reader shares."""

        def __init__(self, bambu: bytes, creality: bytes, enc: bytes) -> None:
            self.bambu_master_key = bambu
            self.creality_key = creality
            self.creality_encryption_key = enc

    def test_each_lane_resolves_to_its_reader(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, readers=self.TWO_READERS, ready=False)
        rig.unit._handle_ready()
        r0, r1 = rig.readers["reader0"], rig.readers["reader1"]
        assert rig.unit._readers == [r0, r1]
        assert rig.unit._reader_by_lane == {"lane8": r0, "lane9": r0,
                                            "lane10": r1, "lane11": r1}
        assert (r0.link.bridge, r0.link.bus) == (rig.unit.bridge, 0)
        assert (r1.link.bridge, r1.link.bus) == (rig.unit.bridge, 1)
        assert rig.unit.afc is rig.printer.afc
        assert rig.logged() == []

    def test_ready_touches_no_hardware_and_starts_the_timer(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, ready=False)
        rig.unit._handle_ready()
        assert rig.serial.opened == []
        assert rig.unit.bridge.connected() is False
        assert rig.port.written == []
        timers = rig.printer.reactor.timers
        assert [(t.callback, t.waketime) for t in timers] == [
            (rig.unit._connect_timer, 101.0)]
        assert isinstance(rig.readers["reader0"].link, _SerialRegLink)
        assert rig.readers["reader0"].version is None
        assert rig.logged() == []

    def test_every_reader_is_offered_to_the_tag_writer(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, readers=self.TWO_READERS, ready=False)
        rig.unit._handle_ready()
        registry = rig.printer._afc_rfid_write_registry
        assert [(t.name, t.label, t.unit) for t in registry.values()] == [
            ("bt:reader0", "BoxTurtle reader0 (bus 0, lanes lane8, lane9)",
             rig.unit),
            ("bt:reader1", "BoxTurtle reader1 (bus 1, lanes lane10, lane11)",
             rig.unit)]
        entry0, entry1 = registry["bt:reader0"], registry["bt:reader1"]
        r0 = rig.readers["reader0"]
        assert (entry0.threaded, entry1.threaded) == (True, True)
        assert entry0.unstage == rig.unit._unstage_after_write
        assert entry0.exclude == rig.unit._write_excluder
        assert (entry0.serves("lane9"), entry0.serves("lane10")) == (True, False)
        assert (entry1.serves("lane10"), entry1.serves("lane9")) == (True, False)
        # The link is fetched per call: an unplugged bridge reads as offline.
        assert entry0.open_link() is None
        rig.unit.bridge._ser = rig.port
        assert entry0.open_link() is r0.link
        rig.unit.bridge._ser = None
        assert entry0.open_link() is None
        rig.unit.bridge._ser = rig.port
        link0, r0.link = r0.link, None
        assert entry0.open_link() is None
        r0.link = link0
        # Each entry stages for its own reader.
        with pytest.raises(StageError) as err:
            entry1.stage("lane8")
        assert str(err.value) == "lane8 is not on reader1; write it with READER=bt:reader0"
        assert entry0.stage("lane8") is None
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: stage-for-write skipped lane8 (lane=False, "
                     "rdr=True, ver=None)")]

    def test_the_bridge_still_gets_handed_the_logger(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, ready=False)
        rig.unit.bridge.logger = BambuLogger()
        rig.unit._handle_ready()
        assert rig.unit.bridge.logger is rig.printer.afc.logger
        assert rig.logged() == []

    def test_ready_does_not_disturb_the_logger(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, ready=False)
        before = rig.unit.logger
        rig.printer._afc = None                  # AFC's lookup comes back empty
        rig.unit._handle_ready()
        assert rig.unit.afc is None
        assert rig.unit.logger is before
        assert rig.unit.bridge.logger is before
        assert rig.logged() == []

    def test_an_afc_without_a_logger_does_not_clear_ours(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, ready=False)
        before = rig.unit.logger
        bare_afc = self._BareAfc()
        rig.printer._afc = bare_afc
        rig.unit._handle_ready()
        assert rig.unit.afc is bare_afc
        assert rig.unit.logger is before
        assert rig.logged() == []

    def test_shared_keys_fill_only_the_unset_ones(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, ready=False,
                            options={"creality_key": "0a0b"})
        rig.printer.add_object("AFC_rfid_keys",
                               self._SharedKeys(b"bambu", b"shared", b"enc"))
        rig.unit._handle_ready()
        assert rig.unit.bambu_master_key == b"bambu"
        assert rig.unit.creality_key == b"\x0a\x0b"
        assert rig.unit.creality_encryption_key == b"enc"
        assert rig.logged() == []


class TestAFCBoxTurtlerfidGetStatus:
    """get_status lists the lane map, which is how the display offers Scan
    Tag, and each reader's online state."""

    def test_get_status_names_the_reader_for_each_lane(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, readers=(
            ("reader0", 0, ("lane8", "lane9")),
            ("reader1", 1, ("lane10", "lane11"))))
        rig.readers["reader1"].version = None
        assert rig.unit.get_status() == {
            "lane_slot_map": {"lane8": "reader0", "lane9": "reader0",
                              "lane10": "reader1", "lane11": "reader1"},
            "readers": {"reader0": True, "reader1": False}}
        assert rig.logged() == []

    def test_before_ready_there_is_nothing_to_list(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, ready=False)
        assert rig.unit.get_status(100.0) == {"lane_slot_map": {}, "readers": {}}
        assert rig.logged() == []


class TestAFCBoxTurtlerfidConnectTimer:
    """The connect timer owns the port: each tick starts one probe thread and
    returns. The probe opens the bridge and reads VersionReg of every reader
    not yet answering; its results are latched back on the reactor."""

    CONNECTED = ("info", f"BT RFID: bridge connected on {BT_RFID_SERIAL}")
    VERSION0 = ("info", "BT RFID reader0: version reg 0x92 (bus 0, lanes lane8, lane9)")
    NO_READER0 = ("warning", "BT RFID reader0: bridge is up but bus 0 has no reader "
                             "(rfid bridge bus 0: no answer reading reg 0x37), "
                             "check wiring")

    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch,
             answers: Optional[Dict[int, List[str]]] = None,
             readers: Sequence[BtRfidReaderSpec] = (
                 ("reader0", 0, ("lane8", "lane9")),),
             hold: Optional[threading.Event] = None) -> BtRfidRig:
        return build_bt_rfid(monkeypatch, readers=readers, connected=False,
                             online=False, port=BtRfidPico(answers, hold))

    @staticmethod
    def _wait_probe(rig: BtRfidRig) -> None:
        deadline = time.monotonic() + 5.0
        while rig.unit._probing and time.monotonic() < deadline:
            time.sleep(0.002)
        assert rig.unit._probing is False, "the probe thread never finished"

    def _tick(self, rig: BtRfidRig, at: float) -> float:
        """Fire the timer due at ``at``, let its probe finish and latch it.

        :return float: the timer's next wake time
        """
        reactor = rig.printer.reactor
        assert reactor.run_timers(until=at) == 1
        self._wait_probe(rig)
        reactor.run_callbacks()
        return reactor.timers[0].waketime

    def test_a_fresh_connect_probes_the_version(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92"]})
        assert self._tick(rig, 101.0) == 106.0
        assert rig.serial.opened == [((BT_RFID_SERIAL, 115200),
                                      {"timeout": 0.25, "write_timeout": 0.25})]
        assert rig.port.ops() == ["r0 37"]
        assert rig.readers["reader0"].version == 0x92
        assert rig.logged() == [self.CONNECTED, self.VERSION0]

    def test_a_fresh_connect_probes_even_during_a_sweep(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92"]})
        rig.unit._sweeping = True
        self._tick(rig, 101.0)
        assert rig.port.ops() == ["r0 37"]
        assert rig.readers["reader0"].version == 0x92
        assert rig.logged() == [self.CONNECTED, self.VERSION0]

    def test_an_absent_bridge_is_a_quiet_retry(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92"]})
        rig.serial.port = None
        rig.readers["reader0"].version = 0x92
        assert self._tick(rig, 101.0) == 106.0
        assert rig.port.written == []
        assert rig.readers["reader0"].version is None
        assert rig.unit.bridge.connected() is False
        assert rig.logged() == []

    def test_a_host_without_pyserial_is_a_quiet_retry(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92"]})
        monkeypatch.setattr(bt_rfid_mod, "serial", None)
        assert self._tick(rig, 101.0) == 106.0
        assert rig.unit.bridge.connected() is False
        assert rig.port.written == []
        assert rig.logged() == []

    def test_a_probe_failure_marks_the_reader_offline(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["!"]})
        self._tick(rig, 101.0)
        assert rig.port.ops() == ["r0 37"]
        assert rig.readers["reader0"].version is None
        assert rig.logged() == [self.CONNECTED, self.NO_READER0]

    def test_a_reader_that_missed_the_first_probe_comes_back(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["!", "=92"]})
        self._tick(rig, 101.0)
        assert rig.readers["reader0"].version is None
        self._tick(rig, 106.0)
        assert rig.readers["reader0"].version == 0x92
        assert rig.port.ops() == ["r0 37", "r0 37"]
        assert rig.logged() == [self.CONNECTED, self.NO_READER0,
                                       self.VERSION0]

    def test_a_healthy_reader_is_not_re_read(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92"]})
        for at in (101.0, 106.0, 111.0):
            self._tick(rig, at)
        assert rig.port.ops() == ["r0 37"]
        assert rig.logged() == [self.CONNECTED, self.VERSION0]

    def test_nothing_is_probed_during_a_sweep(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92"]})
        self._tick(rig, 101.0)
        rig.readers["reader0"].version = None      # now offline
        rig.unit._sweeping = True
        self._tick(rig, 106.0)
        assert rig.port.ops() == ["r0 37"]
        assert rig.readers["reader0"].version is None
        rig.unit._sweeping = False
        self._tick(rig, 111.0)
        assert rig.port.ops() == ["r0 37", "r0 37"]
        assert rig.readers["reader0"].version == 0x92
        assert rig.logged() == [self.CONNECTED, self.VERSION0, self.VERSION0]

    def test_an_absent_reader_does_not_warn_every_tick(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["!"]})
        for i in range(6):
            self._tick(rig, 101.0 + 5.0 * i)
        assert rig.port.ops() == ["r0 37"] * 6
        assert rig.logged() == [self.CONNECTED, self.NO_READER0]

    def test_recovery_is_announced_once(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["!", "=92"]})
        for at in (101.0, 106.0, 111.0, 116.0):
            self._tick(rig, at)
        assert rig.logged() == [self.CONNECTED, self.NO_READER0,
                                       self.VERSION0]

    def test_a_failed_retry_of_an_offline_reader_is_quiet(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92", "!"]})
        self._tick(rig, 101.0)
        rig.readers["reader0"].version = None     # offline, so retried
        self._tick(rig, 106.0)
        self._tick(rig, 111.0)
        assert rig.port.ops() == ["r0 37"] * 3
        assert rig.logged() == [self.CONNECTED, self.VERSION0]

    def test_a_reconnect_rechecks_and_warns_for_a_reader_that_went_quiet(
            self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92", "!"]})
        self._tick(rig, 101.0)
        rig.unit.bridge._ser = None              # the port dropped
        self._tick(rig, 106.0)
        assert rig.readers["reader0"].version is None
        assert rig.port.ops() == ["r0 37", "r0 37"]
        assert rig.logged() == [self.CONNECTED, self.VERSION0,
                                       self.CONNECTED, self.NO_READER0]

    def test_a_reconnect_re_announces_a_reader_that_still_answers(self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92"]})
        self._tick(rig, 101.0)
        rig.unit.bridge._ser = None              # the port dropped
        self._tick(rig, 106.0)
        assert rig.port.ops() == ["r0 37", "r0 37"]
        assert rig.readers["reader0"].version == 0x92
        assert rig.logged() == [self.CONNECTED, self.VERSION0,
                                       self.CONNECTED, self.VERSION0]

    def test_one_dead_reader_does_not_stop_the_other_being_retried(
            self, monkeypatch):
        rig = self._rig(monkeypatch, {0: ["=92"], 1: ["!"]}, readers=(
            ("reader0", 0, ("lane8", "lane9")),
            ("reader1", 1, ("lane10", "lane11"))))
        self._tick(rig, 101.0)
        self._tick(rig, 106.0)
        assert rig.readers["reader0"].version == 0x92
        assert rig.readers["reader1"].version is None
        assert rig.port.ops() == ["r0 37", "r1 37", "r1 37"]
        assert rig.logged() == [
            self.CONNECTED, self.VERSION0,
            ("warning", "BT RFID reader1: bridge is up but bus 1 has no reader "
                        "(rfid bridge bus 1: no answer reading reg 0x37), "
                        "check wiring")]

    def test_the_tick_itself_does_no_port_io(self, monkeypatch):
        hold = threading.Event()
        rig = self._rig(monkeypatch, {0: ["=92"]}, hold=hold)
        reactor = rig.printer.reactor
        try:
            assert reactor.run_timers(until=101.0) == 1
            # Returned while the probe is still blocked on the port.
            assert rig.unit._probing is True
            assert reactor.timers[0].waketime == 106.0
            assert rig.readers["reader0"].version is None
            assert "afc_bt_rfid_probe" in [t.name for t in threading.enumerate()]
        finally:
            hold.set()
        self._wait_probe(rig)
        reactor.run_callbacks()
        assert rig.readers["reader0"].version == 0x92
        assert rig.logged() == [self.CONNECTED, self.VERSION0]

    def test_only_one_probe_thread_at_a_time(self, monkeypatch):
        hold = threading.Event()
        rig = self._rig(monkeypatch, {0: ["=92"]}, hold=hold)
        reactor = rig.printer.reactor
        try:
            for i in range(4):
                assert reactor.run_timers(until=101.0 + 5.0 * i) == 1
            assert reactor.timers[0].waketime == 121.0
        finally:
            hold.set()
        self._wait_probe(rig)
        reactor.run_callbacks()
        assert rig.port.ops() == ["r0 37"]
        assert rig.serial.opened == [((BT_RFID_SERIAL, 115200),
                                      {"timeout": 0.25, "write_timeout": 0.25})]
        assert rig.logged() == [self.CONNECTED, self.VERSION0]

    def test_the_flag_clears_even_when_the_probe_explodes(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.readers["reader0"].version = 0x92
        connect = Recorder(raises=RuntimeError("boom"))
        rig.unit.bridge.connect = connect
        self._tick(rig, 101.0)
        assert rig.readers["reader0"].version is None
        self._tick(rig, 106.0)                   # not wedged: it probes again
        assert connect.call_count == 2
        assert rig.logged() == []


class TestAFCBoxTurtlerfidExcluderFor:
    """Each antenna sees both of its lanes' spools, so the partner lane's
    last-known UID, and a parked baseline, are passed over."""

    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch) -> BtRfidRig:
        return build_bt_rfid(monkeypatch, readers=(
            ("reader0", 0, ("lane8", "lane9")),
            ("reader1", 1, ("lane10", "lane11"))))

    def test_no_known_siblings_means_no_excluder(self, monkeypatch):
        rig = self._rig(monkeypatch)
        assert rig.unit._excluder_for("lane8", rig.readers["reader0"]) is None
        assert rig.logged() == []

    def test_the_partners_uid_is_excluded(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit._last_uid_by_lane["lane9"] = "aabbccdd"
        ex = rig.unit._excluder_for("lane8", rig.readers["reader0"])
        assert ex("AABBCCDD") is True
        assert ex("04a1b2c3") is False
        assert rig.logged() == []

    def test_a_partner_uid_on_file_in_upper_case_still_matches(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit._last_uid_by_lane["lane9"] = "AABBCCDD"
        ex = rig.unit._excluder_for("lane8", rig.readers["reader0"])
        assert ex("aabbccdd") is True
        assert rig.logged() == []

    def test_the_lanes_own_uid_is_not_excluded(self, monkeypatch):
        # Re-reading the same lane must see its own tag again.
        rig = self._rig(monkeypatch)
        rig.unit._last_uid_by_lane["lane8"] = "aabbccdd"
        assert rig.unit._excluder_for("lane8", rig.readers["reader0"]) is None
        assert rig.logged() == []

    def test_the_other_readers_lanes_do_not_leak_in(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit._last_uid_by_lane["lane10"] = "11223344"
        assert rig.unit._excluder_for("lane8", rig.readers["reader0"]) is None
        assert rig.logged() == []

    def test_the_baseline_survives_a_lane_with_no_uid_history(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit._baseline_uid = "CAFE1234"
        ex = rig.unit._excluder_for("lane8", rig.readers["reader0"])
        assert ex("cafe1234") is True
        assert ex("aabbccdd") is False
        assert rig.unit._baseline_uid == "CAFE1234"
        assert rig.logged() == []


class TestAFCBoxTurtlerfidParkedTag:
    """The parked-tag probe reads whatever is on the coil, unfiltered."""

    def test_the_probe_is_not_excluded(self, monkeypatch):
        # The sister's UID is exactly what is being looked for, so the probe
        # passes no excluder even though one would exclude it.
        rig = build_bt_rfid(monkeypatch)
        rig.unit._last_uid_by_lane["lane9"] = "cafe1234"
        rig.field.parked = "CAFE1234"
        assert rig.unit._parked_tag(rig.readers["reader0"]) == "CAFE1234"
        main = threading.current_thread().name
        assert rig.field.calls == [(main, True, None)]
        assert rig.logged() == []

    def test_a_clear_antenna_is_none(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        assert rig.unit._parked_tag(rig.readers["reader0"]) is None
        assert len(rig.field.calls) == 1
        assert rig.logged() == []

    def test_a_tag_without_a_uid_is_none(self, monkeypatch):
        # A tag answered, but with a blank UID: no parked tag to clear for.
        rig = build_bt_rfid(monkeypatch)
        rig.field.parked_tag = {"uid": "", "sak": 8}
        assert rig.unit._parked_tag(rig.readers["reader0"]) is None
        rig.field.parked_tag = {"sak": 8}
        assert rig.unit._parked_tag(rig.readers["reader0"]) is None
        main = threading.current_thread().name
        assert rig.field.calls == [(main, True, None), (main, True, None)]
        assert rig.logged() == []

    def test_a_failing_probe_is_no_collision(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        rig.field.parked = "CAFE1234"
        rig.field.raises = OSError("bus gone")
        assert rig.unit._parked_tag(rig.readers["reader0"]) is None
        assert rig.logged() == []

    def test_an_offline_bridge_or_unwired_reader_is_not_probed(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, connected=False)
        rig.field.parked = "CAFE1234"
        assert rig.unit._parked_tag(rig.readers["reader0"]) is None
        rig.unit.bridge._ser = rig.port
        rig.readers["reader0"].link = None
        assert rig.unit._parked_tag(rig.readers["reader0"]) is None
        assert rig.field.calls == []
        assert rig.logged() == []


class TestAFCBoxTurtlerfidParkedTagOnce:
    """The probe is a second of serial round-trips, so it runs on a worker
    thread while the reactor waits."""

    def test_the_probe_does_not_run_on_the_reactor(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        rig.field.parked = "CAFE1234"
        assert rig.unit._parked_tag_once(rig.readers["reader0"]) == "CAFE1234"
        assert rig.field.probes() == ["afc_bt_rfid_rd"]
        assert threading.current_thread().name != "afc_bt_rfid_rd"
        assert rig.printer.reactor.async_callbacks == []
        assert rig.logged() == []

    def test_a_clear_antenna_comes_back_as_none(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        assert rig.unit._parked_tag_once(rig.readers["reader0"]) is None
        assert rig.field.probes() == ["afc_bt_rfid_rd"]
        assert rig.logged() == []


class TestAFCBoxTurtlerfidRequireLaneReader:
    """LANE= must name a lane some reader serves."""

    def test_an_unmapped_lane_is_a_gcode_error(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        gcmd = FakeGcmd({"LANE": "lane12"})
        with pytest.raises(BT_RFID_CMD_ERROR) as err:
            rig.unit._require_lane_reader(gcmd)
        assert str(err.value) == ("AFC_BT_RFID: no reader serves lane lane12. "
                                  "Configured: ['lane8', 'lane9']")
        assert gcmd.messages == []
        assert rig.logged() == []

    def test_with_no_readers_it_says_none(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, readers=())
        gcmd = FakeGcmd({"LANE": "lane8"})
        with pytest.raises(BT_RFID_CMD_ERROR) as err:
            rig.unit._require_lane_reader(gcmd)
        assert str(err.value) == ("AFC_BT_RFID: no reader serves lane lane8. "
                                  "Configured: none")
        assert gcmd.messages == []
        assert rig.logged() == []

    def test_a_mapped_lane_returns_its_reader(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        gcmd = FakeGcmd({"LANE": "lane9"})
        assert rig.unit._require_lane_reader(gcmd) == ("lane9", rig.readers["reader0"])
        assert gcmd.messages == []
        assert rig.logged() == []


class TestAFCBoxTurtlerfidCmdAFCBTRFIDRead:
    """AFC_BT_RFID_READ: one read with no motion, applied when it answers."""

    NOTHING = ("respond_info",
               "AFC_BT_RFID_READ: nothing readable in reader0's field for lane8. "
               "If the tag rides the spool, use AFC_BT_RFID_STAGE to spin it past "
               "the antenna.")

    def test_no_tag_says_so_and_points_at_stage(self, monkeypatch):
        lane8 = BtRfidLane("lane8")
        rig = build_bt_rfid(monkeypatch, lanes=[lane8])
        gcmd = FakeGcmd({"LANE": "lane8"})
        rig.unit.cmd_AFC_BT_RFID_READ(gcmd)
        assert gcmd.messages == [self.NOTHING]
        assert rig.console == []
        assert rig.unit.last_reads_status() == {}
        assert rig.unit._last_uid_by_lane == {}
        assert rig.field.reads() == ["afc_bt_rfid_rd"]
        assert lane8.moves == []
        assert rig.logged() == []

    def test_an_offline_bridge_is_not_read(self, monkeypatch):
        # The reader's link is wired; only the bridge port is closed.
        lane8 = BtRfidLane("lane8")
        rig = build_bt_rfid(monkeypatch, lanes=[lane8], connected=False)
        rig.field.script = [True]
        gcmd = FakeGcmd({"LANE": "lane8"})
        rig.unit.cmd_AFC_BT_RFID_READ(gcmd)
        assert isinstance(rig.readers["reader0"].link, _SerialRegLink)
        assert rig.field.calls == []
        assert gcmd.messages == [self.NOTHING]
        assert rig.console == []
        assert rig.unit._last_uid_by_lane == {}
        assert rig.logged() == []

    def test_a_tag_is_applied_and_its_uid_remembered(self, monkeypatch):
        lane8 = BtRfidLane("lane8")
        rig = build_bt_rfid(monkeypatch, lanes=[lane8])
        rig.field.uid, rig.field.script = "04A1B2C3", [True]
        gcmd = FakeGcmd({"LANE": "lane8"})
        rig.unit.cmd_AFC_BT_RFID_READ(gcmd)
        assert rig.console == [bt_rfid_read_out("lane8", "04A1B2C3")]
        assert lane8.weight == 1000
        assert rig.unit.last_reads_status() == {"lane8": bt_rfid_record("04A1B2C3")}
        assert rig.unit._last_uid_by_lane == {"lane8": "04a1b2c3"}
        assert gcmd.messages == []
        assert lane8.moves == []
        assert rig.logged() == []

    def test_the_sister_lanes_known_tag_is_passed_over(self, monkeypatch):
        lane8 = BtRfidLane("lane8")
        rig = build_bt_rfid(monkeypatch, lanes=[lane8])
        rig.unit._last_uid_by_lane["lane9"] = "deadbeef"
        rig.field.uid, rig.field.script = "DEADBEEF", [True]
        gcmd = FakeGcmd({"LANE": "lane8"})
        rig.unit.cmd_AFC_BT_RFID_READ(gcmd)
        assert gcmd.messages == [self.NOTHING]
        assert rig.console == []
        assert rig.unit.last_reads_status() == {}
        assert rig.unit._last_uid_by_lane == {"lane9": "deadbeef"}
        assert rig.field.script == []
        assert rig.field.reads() == ["afc_bt_rfid_rd"]
        assert rig.field.calls[0][2]("DEADBEEF") is True
        assert rig.logged() == []

    def test_a_failed_read_is_logged_and_reads_as_no_tag(self, monkeypatch):
        lane8 = BtRfidLane("lane8")
        rig = build_bt_rfid(monkeypatch, lanes=[lane8])
        rig.field.raises = OSError("bus gone")
        gcmd = FakeGcmd({"LANE": "lane8"})
        rig.unit.cmd_AFC_BT_RFID_READ(gcmd)
        assert rig.logged() == [
            ("warning", "BT RFID reader0: read failed: bus gone")]
        assert gcmd.messages == [self.NOTHING]
        assert rig.console == []

    def test_a_tag_on_a_lane_afc_does_not_know_is_not_applied(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        rig.field.uid, rig.field.script = "04A1B2C3", [True]
        gcmd = FakeGcmd({"LANE": "lane9"})
        rig.unit.cmd_AFC_BT_RFID_READ(gcmd)
        assert rig.console == []
        assert rig.unit.last_reads_status() == {}
        assert rig.unit._last_uid_by_lane == {"lane9": "04a1b2c3"}
        assert gcmd.messages == [(
            "respond_info",
            "AFC_BT_RFID: read uid 04A1B2C3 but lane lane9 is not an AFC lane, "
            "nothing applied.")]
        assert rig.logged() == []


class TestAFCBoxTurtlerfidCmdAFCBTRFIDStatus:
    """AFC_BT_RFID_STATUS reports the bridge, each reader and the last reads."""

    def test_reports_readers_and_last_reads(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        rig.unit._last_uid_by_lane["lane9"] = "cafe1234"
        rig.unit._last_uid_by_lane["lane8"] = "04a1b2c3"
        gcmd = FakeGcmd()
        rig.unit.cmd_AFC_BT_RFID_STATUS(gcmd)
        assert gcmd.messages == [(
            "respond_info",
            "AFC_BT_RFID status\n"
            "bridge: connected (/dev/serial/by-id/test-rfid-pico)\n"
            "reader0: version reg 0x92  lanes: lane8, lane9\n"
            "lane8: last uid 04a1b2c3\n"
            "lane9: last uid cafe1234")]
        assert rig.logged() == []

    def test_offline_reader_says_offline(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, connected=False, online=False)
        gcmd = FakeGcmd()
        rig.unit.cmd_AFC_BT_RFID_STATUS(gcmd)
        assert gcmd.messages == [(
            "respond_info",
            "AFC_BT_RFID status\n"
            "bridge: OFFLINE (/dev/serial/by-id/test-rfid-pico)\n"
            "reader0: offline (retrying)  lanes: lane8, lane9")]
        assert rig.logged() == []

    def test_no_readers_configured_says_how_to(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch, readers=())
        gcmd = FakeGcmd()
        rig.unit.cmd_AFC_BT_RFID_STATUS(gcmd)
        assert gcmd.messages == [(
            "respond_info",
            "AFC_BT_RFID: no reader sections configured "
            "([AFC_BoxTurtle_rfid <name>] with bus + lanes).")]
        assert rig.logged() == []


class TestAFCBoxTurtlerfidOnLanePrepLoaded:
    """The insert scan: once prep_load has homed the tip onto the load switch,
    sweep for the tag, put the filament back and apply what was read. A
    failed scan never costs the insert."""

    class _Nameless:
        """A lane object with no name."""

    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch,
             tag_after: Optional[float] = 40.0,
             served: Tuple[str, ...] = ("lane8",),
             options: Optional[Dict[str, Any]] = None
             ) -> Tuple[BtRfidRig, BtRfidLane]:
        lane = BtRfidLane("lane8")
        rig = build_bt_rfid(monkeypatch, readers=(("reader0", 0, served),),
                            lanes=[lane], homing=False, options=options)
        rig.field.uid = "AA"
        if tag_after is not None:
            rig.field.appear(lane, tag_after)
        return rig, lane

    READ = ("info", "AFC_BT_RFID: lane8 tag AA read after 40mm at 20mm/s in 20mm "
                    "chunks; filament restored.")
    READ_OUT = bt_rfid_read_out("lane8", "AA")

    def test_an_insert_scans_and_applies_the_tag(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.unit._on_lane_prep_loaded(lane)
        assert rig.console == [self.READ_OUT]
        assert lane.weight == 1000
        assert rig.unit.last_reads_status() == {"lane8": bt_rfid_record("AA")}
        assert rig.unit._last_uid_by_lane == {"lane8": "aa"}
        assert lane.feeds() == [20.0, 20.0]
        assert rig.unit._sweeping is False
        assert rig.logged() == [self.READ]

    def test_the_filament_is_put_back_where_it_started(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.unit._on_lane_prep_loaded(lane)
        # 10mm bulk, the 30mm tail in short steps, then a creep back on.
        assert [m[0] for m in lane.moves] == [20.0, 20.0, -10.0, -10.0, -10.0,
                                              -10.0, -10.0, 10.0]
        assert lane.pos == 5.0
        assert rig.console == [self.READ_OUT]
        assert rig.logged() == [self.READ]

    def test_the_return_runs_the_espooler(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.unit._on_lane_prep_loaded(lane)
        assert lane.retracts() == [(-10.0, 100.0, True)] + [(-10.0, 25.0, True)] * 4
        assert lane.moves[-1] == (10.0, 25.0, 400, False)
        assert rig.console == [self.READ_OUT]
        assert rig.logged() == [self.READ]

    def test_a_configured_retract_speed_drives_the_return(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, options={"tag_retract_speed": 60.0})
        rig.unit._on_lane_prep_loaded(lane)
        assert lane.retracts() == [(-10.0, 60.0, True)] + [(-10.0, 25.0, True)] * 4
        assert lane.pos == 5.0
        assert rig.console == [self.READ_OUT]
        assert rig.logged() == [self.READ]

    def test_no_configured_step_sizes_the_chunk_from_the_lane(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, options={
            "tag_sweep_step_mm": 0.0, "tag_sweep_speed": 100.0})
        lane.long_moves_accel = 250
        rig.unit._on_lane_prep_loaded(lane)
        # (100^2 / 250) / 0.4 = 100mm: one chunk carries the tag past 40mm.
        assert lane.moves == ([(100.0, 100.0, 250, False),
                               (-70.0, 100.0, 250, True)]
                              + [(-10.0, 25.0, 250, True)] * 4
                              + [(10.0, 25.0, 250, False)])
        assert rig.console == [self.READ_OUT]
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: lane8 tag AA read after 100mm at 100mm/s in "
                     "100mm chunks; filament restored.")]

    def test_a_lane_no_reader_claims_is_left_alone(self, monkeypatch):
        # Half a BoxTurtle can have readers; the other half must not move.
        rig, lane = self._rig(monkeypatch, served=("lane9",))
        rig.unit._on_lane_prep_loaded(lane)
        assert lane.moves == []
        assert rig.console == []
        assert rig.field.calls == []
        assert rig.logged() == []

    def test_scan_on_insert_false_disables_it(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, options={"scan_on_insert": False})
        rig.unit._on_lane_prep_loaded(lane)
        assert lane.moves == []
        assert rig.field.calls == []
        assert rig.console == []
        assert rig.logged() == []

    def test_an_offline_reader_moves_no_filament(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.readers["reader0"].version = None
        rig.unit._on_lane_prep_loaded(lane)
        assert lane.moves == []
        assert rig.unit._sweeping is False
        assert rig.console == []
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: lane8 inserted but reader0 is offline, "
                        "skipping the tag scan. Check AFC_BT_RFID_STATUS.")]

    def test_it_does_not_join_a_sweep_already_running(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.unit._sweeping = True
        rig.unit._on_lane_prep_loaded(lane)
        assert lane.moves == []
        assert rig.unit._sweeping is True
        assert rig.console == []
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: lane8 inserted while a sweep was already "
                        "running, not scanning this one.")]

    def test_no_tag_still_restores_the_filament(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        rig.unit._on_lane_prep_loaded(lane)
        assert rig.console == []
        assert sum(lane.feeds()) == pytest.approx(2 * BT_RFID_TURN_MM)
        assert lane.pos == pytest.approx(5.0)
        assert rig.unit._last_uid_by_lane == {}
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: no tag on lane8 after 1257mm (2.0 turns); "
                     "filament restored.")]

    def test_a_failure_never_costs_the_insert(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        lane.fail_after = 0
        rig.unit._on_lane_prep_loaded(lane)            # must not raise
        assert rig.unit._sweeping is False
        assert rig.console == []
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: tag scan failed for lane8 (stepper gone), the "
                        "insert carries on without it.")]

    def test_a_lane_with_no_name_is_ignored(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.unit._on_lane_prep_loaded(self._Nameless())
        assert lane.moves == []
        assert rig.field.calls == []
        assert rig.console == []
        assert rig.logged() == []

    def test_every_read_in_a_sweep_is_off_the_reactor(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        rig.unit._on_lane_prep_loaded(lane)
        reads = rig.field.reads()
        assert set(reads) <= {"afc_bt_rfid_sw", "afc_bt_rfid_rd"}
        assert reads[-1] == "afc_bt_rfid_rd"           # the closing read
        assert rig.console == []
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: no tag on lane8 after 1257mm (2.0 turns); "
                     "filament restored.")]

    def test_the_insert_scan_is_bounded_by_a_busy_hub(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, busy=True)
        rig.unit._on_lane_prep_loaded(lane)
        # 150mm dist_hub less the 25mm margin.
        assert lane.feeds() == [20.0] * 6 + [5.0]
        assert lane.pos == 0.5                          # homed back on
        assert rig.console == []
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: lane9 inserted while hub Turtle_1 is in use "
                        "(lane8 is loaded through it), so the sweep stops 25mm "
                        "short of it; scanning only 125mm."),
            ("info", "AFC_BT_RFID: no tag on lane9 after 125mm (0.2 turns); "
                     "filament restored.")]

    def test_the_insert_scan_skips_when_there_is_no_room(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, busy=True, dist_hub=20.0)
        rig.unit._on_lane_prep_loaded(lane)
        assert lane.moves == []
        assert rig.field.calls == []
        assert rig.unit._sweeping is False
        assert rig.console == []
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: lane9 inserted while hub Turtle_1 is in use "
                        "(lane8 is loaded through it), so the sweep stops 25mm "
                        "short of it; not scanning. AFC_BT_RFID_STAGE can scan it "
                        "once the hub is clear.")]

    def test_the_guard_is_held_while_the_scan_runs(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        seen: List[bool] = []
        rig.field.on_probe = lambda: seen.append(rig.unit._sweeping)
        rig.unit._on_lane_prep_loaded(lane)
        assert seen == [True]
        assert rig.unit._sweeping is False
        assert rig.console == []
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag CAFE1234 is parked on reader0's antenna "
                     "while lane8 is read, clearing lane9 off it."),
            ("info", "AFC_BT_RFID: rolled lane9 back 75mm to clear its tag off "
                     "reader0's antenna while lane8 is read."),
            ("info", "AFC_BT_RFID: no tag on lane8 after 1257mm (2.0 turns); "
                     "filament restored."),
            ("info", "AFC_BT_RFID: re-staged lane9 the way a load does: homed to "
                     "its load switch, then fed dist_hub (200mm). The hub is "
                     "untouched.")]

    def test_the_sister_is_cleared_and_put_back_around_the_scan(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        rig.unit._on_lane_prep_loaded(lane)
        assert [m[0] for m in sib.moves] == [-10.0] * 7 + [-5.0, 75.0, 200.0]
        assert sib.loaded_to_hub is True
        assert rig.unit._sweeping is False
        assert rig.console == []
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag CAFE1234 is parked on reader0's antenna "
                     "while lane8 is read, clearing lane9 off it."),
            ("info", "AFC_BT_RFID: rolled lane9 back 75mm to clear its tag off "
                     "reader0's antenna while lane8 is read."),
            ("info", "AFC_BT_RFID: no tag on lane8 after 1257mm (2.0 turns); "
                     "filament restored."),
            ("info", "AFC_BT_RFID: re-staged lane9 the way a load does: homed to "
                     "its load switch, then fed dist_hub (200mm). The hub is "
                     "untouched.")]


class TestAFCBoxTurtlerfidClearSibling:
    """Two lanes share one antenna, so the sister's parked tag can be read as
    this lane's. When the probe sees a tag the sister owns, she is rolled back
    off the coil, or, when a gate forbids moving her, her tag is excluded."""

    PARKED = ("info", "AFC_BT_RFID: tag CAFE1234 is parked on reader0's antenna "
                      "while lane8 is read, clearing lane9 off it.")
    ROLLED = ("info", "AFC_BT_RFID: rolled lane9 back 75mm to clear its tag off "
                      "reader0's antenna while lane8 is read.")
    ROLL_BACK = [(-10.0, 100.0, 400, True)] * 7 + [(-5.0, 100.0, 400, True)]
    UNSEATED = ("info", "AFC_BT_RFID: tag CAFE1234 on reader0's antenna, but lane9 "
                        "holds no seated spool: it is lane8's own, reading it.")

    @staticmethod
    def _clear(rig: BtRfidRig) -> Optional[tuple]:
        return rig.unit._clear_sibling("lane8", rig.readers["reader0"])

    def test_a_clear_antenna_means_the_sister_is_not_touched(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, parked=None)
        assert self._clear(rig) is None
        assert sib.moves == [] and sib.homing_moves == []
        assert rig.field.probes() == ["afc_bt_rfid_rd"]
        assert rig.unit._baseline_uid is None
        assert rig.logged() == []

    def test_this_lanes_own_parked_tag_is_not_a_collision(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, parked="7BF0AFFF")
        rig.unit._last_uid_by_lane = {"lane8": "7bf0afff"}
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.unit._baseline_uid is None
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag 7BF0AFFF on reader0's antenna is lane8's "
                     "own, no collision, leaving lane9 alone.")]

    def test_the_sisters_known_tag_still_is_one(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        rig.unit._last_uid_by_lane = {"lane8": "7bf0afff", "lane9": "cafe1234"}
        assert self._clear(rig) == (sib, 75.0, 100.0)
        assert sib.moves == self.ROLL_BACK
        assert rig.logged() == [self.PARKED, self.ROLLED]

    def test_an_unknown_parked_tag_is_treated_as_a_collision(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, parked="DEADBEEF")
        assert self._clear(rig) == (sib, 75.0, 100.0)
        assert sib.moves == self.ROLL_BACK
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag DEADBEEF is parked on reader0's antenna "
                     "while lane8 is read, clearing lane9 off it."),
            self.ROLLED]

    def test_attribution_is_case_insensitive(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, parked="7bf0afff")
        rig.unit._last_uid_by_lane = {"lane8": "7BF0AFFF"}
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag 7bf0afff on reader0's antenna is lane8's "
                     "own, no collision, leaving lane9 alone.")]

    def test_a_parked_tag_is_what_triggers_the_clear(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        assert self._clear(rig) == (sib, 75.0, 100.0)
        assert sib.pos == 125.0
        assert rig.logged() == [self.PARKED, self.ROLLED]

    def test_the_probe_runs_before_any_move_is_made(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        seen: List[list] = []
        rig.field.on_probe = lambda: seen.append(list(sib.moves))
        self._clear(rig)
        assert seen == [[]]
        assert sib.moves == self.ROLL_BACK
        assert rig.logged() == [self.PARKED, self.ROLLED]

    def test_an_unmovable_sisters_tag_is_excluded_instead(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        sib.tool_loaded = True                    # a gate that refuses to move
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.unit._baseline_uid == "CAFE1234"
        ex = rig.unit._excluder_for("lane8", rig.readers["reader0"])
        assert ex("cafe1234") is True
        assert rig.logged() == [self.PARKED]

    def test_an_empty_sister_never_gets_this_lanes_tag_excluded(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, parked="04782696DD2A81")
        sib.prep_state = False
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.unit._baseline_uid is None
        assert rig.unit._excluder_for("lane8", rig.readers["reader0"]) is None
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag 04782696DD2A81 on reader0's antenna, but "
                     "lane9 holds no seated spool: it is lane8's own, reading "
                     "it.")]

    def test_a_sister_off_her_load_switch_excludes_nothing(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, sib_pos=-50.0,
                                            parked="04782696DD2A81")
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.unit._baseline_uid is None
        assert rig.unit._excluder_for("lane8", rig.readers["reader0"]) is None
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag 04782696DD2A81 on reader0's antenna, but "
                     "lane9 holds no seated spool: it is lane8's own, reading "
                     "it.")]

    def test_a_cleared_sister_needs_no_baseline(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        assert self._clear(rig) == (sib, 75.0, 100.0)
        assert rig.unit._baseline_uid is None
        assert rig.unit._excluder_for("lane8", rig.readers["reader0"]) is None
        assert rig.logged() == [self.PARKED, self.ROLLED]

    def test_a_clear_antenna_leaves_no_baseline_behind(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, parked=None)
        rig.unit._baseline_uid = "CAFE1234"       # left by an earlier scan
        assert self._clear(rig) is None
        assert rig.unit._baseline_uid is None
        assert rig.logged() == []

    def test_an_idle_hub_staged_sister_is_rolled_back(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        assert self._clear(rig) == (sib, 75.0, 100.0)
        assert sum(m[0] for m in sib.moves) == -75.0
        assert sib.moves == self.ROLL_BACK
        assert sib.homing_moves == []
        assert rig.logged() == [self.PARKED, self.ROLLED]

    def test_a_badly_staged_sister_stops_at_its_load_switch(self, monkeypatch):
        # loaded_to_hub is remembered, not measured: the switch has the last
        # word, so the roll-back distance is a maximum.
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, sib_pos=5.0)
        assert self._clear(rig) == (sib, 10.0, 100.0)
        assert sib.moves == [(-10.0, 100.0, 400, True)]
        assert sib.pos == -5.0
        assert rig.unit._baseline_uid is None
        assert rig.logged() == [
            self.PARKED,
            ("info", "AFC_BT_RFID: rolled lane9 back 10mm to clear its tag off "
                     "reader0's antenna while lane8 is read (stopped early at "
                     "its load switch).")]

    def test_a_sister_already_off_its_load_switch_is_not_touched(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch, sib_pos=-50.0)
        assert self._clear(rig) is None
        assert sib.moves == []
        assert sib.pos == -50.0
        assert rig.logged() == [self.UNSEATED]

    def test_a_tool_loaded_sister_is_never_moved(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        sib.tool_loaded = True
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.logged() == [self.PARKED]

    def test_a_sister_not_staged_at_the_hub_is_never_moved(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        sib.loaded_to_hub = False
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.unit._baseline_uid == "CAFE1234"
        assert rig.logged() == [self.PARKED]

    def test_an_empty_sister_lane_is_left_alone(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        sib.prep_state = False
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.unit._baseline_uid is None
        assert rig.logged() == [self.UNSEATED]

    def test_nothing_moves_during_a_print(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        rig.printer.set_print_state("printing")
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.unit._baseline_uid == "CAFE1234"
        assert rig.logged() == [self.PARKED]

    def test_a_print_state_that_cannot_be_read_does_not_block(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        rig.printer.afc.function.is_printing = Recorder(raises=KeyError("state"))
        assert self._clear(rig) == (sib, 75.0, 100.0)
        assert sib.moves == self.ROLL_BACK
        assert rig.logged() == [self.PARKED, self.ROLLED]

    def test_the_feature_can_be_turned_off(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        rig.unit.sibling_tag_adjust = False
        assert self._clear(rig) is None
        assert sib.moves == []
        assert rig.field.calls == []
        assert rig.logged() == []

    def test_a_reader_with_one_lane_has_no_sister(self, monkeypatch):
        lane = BtRfidLane("lane8")
        rig = build_bt_rfid(monkeypatch, readers=(("reader0", 0, ("lane8",)),),
                            lanes=[lane])
        rig.field.parked = "CAFE1234"
        assert self._clear(rig) is None
        assert rig.field.calls == []
        assert rig.logged() == []

    def test_a_sister_without_speeds_rolls_at_the_fallbacks(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        sib.long_moves_speed = None
        sib.short_move_dis = None
        assert self._clear(rig) == (sib, 75.0, 20.0)
        assert sib.moves == [(-10.0, 20.0, 400, True)] * 7 + [(-5.0, 20.0, 400, True)]
        assert rig.logged() == [self.PARKED, self.ROLLED]

    def test_a_clear_failure_does_not_stop_the_read(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        sib.fail_after = 0
        assert self._clear(rig) is None
        assert rig.unit._baseline_uid == "CAFE1234"
        assert rig.logged() == [
            self.PARKED,
            ("warning", "AFC_BT_RFID: could not roll lane9 back off the shared "
                        "antenna (stepper gone); reading lane8 anyway.")]

    def test_a_failure_part_way_returns_what_was_moved(self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        sib.fail_after = 2
        assert self._clear(rig) == (sib, 20.0, 100.0)
        assert sib.pos == 180.0
        assert rig.logged() == [
            self.PARKED,
            ("warning", "AFC_BT_RFID: could not roll lane9 back off the shared "
                        "antenna (stepper gone); reading lane8 anyway.")]


class TestAFCBoxTurtlerfidRestoreSibling:
    """The rolled-back sister goes back where a normal insert leaves her: the
    give-back, a creep onto her load switch if it lands short, then the same
    two moves a load makes (home onto the switch, feed dist_hub)."""

    RESTAGED = ("info", "AFC_BT_RFID: re-staged lane9 the way a load does: homed "
                        "to its load switch, then fed dist_hub (200mm). The hub is "
                        "untouched.")

    @staticmethod
    def _rolled(monkeypatch: pytest.MonkeyPatch, **kw: Any
                ) -> Tuple[BtRfidRig, BtRfidLane, tuple]:
        """The sister after a 75mm roll-back at 100mm/s, and its token."""
        rig, _lane, sib = bt_rfid_sister_rig(monkeypatch, **kw)
        sib.pos -= 75.0
        return rig, sib, (sib, 75.0, 100.0)

    def test_and_put_back_afterwards(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        rig.unit._restore_sibling(token)
        assert sib.pos == 200.5                  # load switch edge + dist_hub
        assert sib.hub_obj.state is False
        assert sib.loaded_to_hub is True
        assert rig.logged() == [self.RESTAGED]

    def test_a_hub_that_never_clears_still_terminates(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch, hub_at=-1e9)
        rig.unit._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0, 200.0]
        assert len(sib.homing_moves) == 2
        assert rig.logged() == [self.RESTAGED]

    def test_the_restage_never_drives_filament_at_the_hub(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        rig.unit._restore_sibling(token)
        assert sib.hub_obj.state is False
        assert sib.pos <= sib.hub_obj.at
        assert [m[2] for m in sib.homing_moves] == ["load", "load"]
        assert rig.logged() == [self.RESTAGED]

    def test_the_retracts_run_the_espooler(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        rig.unit._restore_sibling(token)
        assert [(m[0], m[4]) for m in sib.homing_moves if m[0] < 0] == [
            (-200.0, AssistActive.YES)]
        assert sib.moves[0] == (75.0, 100.0, 400, False)
        assert rig.logged() == [self.RESTAGED]

    def test_it_re_stages_the_way_a_load_does(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        rig.unit._restore_sibling(token)
        assert sib.homing_moves == [
            (-200.0, SpeedMode.LONG, "load", True, AssistActive.YES),
            (400.0, None, "load", True, None)]
        assert sib.unit_obj.calls == ["prep_load", "prep_post_load"]
        assert sib.moves[-1] == (200.0, 100.0, 400, False)
        assert sib.loaded_to_hub is True
        assert rig.logged() == [self.RESTAGED]

    @pytest.mark.parametrize("slip", [0.0, 0.2, 0.5])
    def test_it_lands_exactly_where_a_fresh_insert_would(self, monkeypatch, slip):
        rig, sib, token = self._rolled(monkeypatch, slip=slip)
        rig.unit._restore_sibling(token)
        # The last move is prep_post_load's dist_hub feed from the switch edge.
        assert sib.moves[-1][0] == 200.0
        assert sib.pos - 200.0 * (1.0 - slip) == pytest.approx(0.5)
        assert sib.loaded_to_hub is True
        assert sib.hub_obj.state is False
        assert rig.logged() == [self.RESTAGED]

    def test_nothing_is_inserted_between_the_two_proven_moves(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        rig.unit._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0, 200.0]
        assert rig.logged() == [self.RESTAGED]

    def test_without_homing_it_re_feeds_and_clears_the_flag(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch, homing=False)
        rig.unit._restore_sibling(token)
        assert sib.homing_moves == []
        assert sib.moves == [(75.0, 100.0, 400, False)]
        assert sib.loaded_to_hub is False
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: homing is off, so lane9 was only re-fed 75mm "
                        "rather than re-staged. Its next load will put it back at "
                        "the hub.")]

    def test_a_lane_configured_not_to_stage_is_left_where_it_lands(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.load_to_hub = False
        rig.unit._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0]
        assert sib.homing_moves == []
        assert sib.loaded_to_hub is True
        assert rig.logged() == []

    def test_a_lane_with_no_hub_object_is_left_after_the_refeed(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.hub_obj = None
        rig.unit._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0]
        assert sib.homing_moves == []
        assert rig.logged() == []

    def test_a_restore_failure_is_logged_not_raised(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.fail_after = 0
        rig.unit._restore_sibling(token)           # must not raise
        assert sib.loaded_to_hub is False
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: could not restore lane9 after the read "
                        "(stepper gone); its next load will re-home it.")]

    def test_it_says_so_when_the_sister_cannot_be_recovered(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.pos = -500.0                         # came out of the lane entirely
        rig.unit._restore_sibling(token)
        # The give-back, then int(75 / 10) + 4 creeps of 10mm.
        assert sib.moves == [(75.0, 100.0, 400, False)] + [
            (10.0, 25.0, 400, False)] * 11
        assert sib.loaded_to_hub is False
        assert sib.homing_moves == []
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: fed lane9 a further 110mm to put it back on its "
                     "load switch."),
            ("warning", "AFC_BT_RFID: lane9 is NOT back on its load switch after "
                        "the read; its filament may have come out of the lane. "
                        "Re-seat it and check the spool.")]

    def test_it_homes_the_sister_back_onto_the_switch(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.pos = -100.0                         # the give-back lands short
        rig.unit._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0, 10.0, 10.0, 10.0, 200.0]
        assert rig.unit._seated(sib) is True
        assert sib.loaded_to_hub is True
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: fed lane9 a further 30mm to put it back on its "
                     "load switch."),
            self.RESTAGED]

    def test_a_sister_the_homing_moves_cannot_seat_is_not_staged(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.unit_obj.prep_load = Recorder()     # the forward home never lands
        rig.unit._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0]
        assert sib.loaded_to_hub is False
        assert sib.unit_obj.calls == []
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: lane9 would not come back to its load switch, "
                        "so it is not staged. Check the spool. Its next load will "
                        "re-home it.")]

    def test_a_sister_with_no_unit_is_left_after_the_refeed(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.unit_obj = None
        rig.unit._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0]
        assert sib.homing_moves == []
        assert sib.loaded_to_hub is True
        assert rig.logged() == []

    def test_a_direct_hub_sister_is_left_after_the_refeed(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.is_direct_hub = lambda: True
        rig.unit._restore_sibling(token)
        assert sib.moves == [(75.0, 100.0, 400, False)]
        assert sib.homing_moves == []
        assert sib.unit_obj.calls == []
        assert sib.loaded_to_hub is True
        assert rig.logged() == []

    def test_a_sister_with_no_dist_hub_is_left_after_the_refeed(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.dist_hub = 0.0
        rig.unit._restore_sibling(token)
        assert sib.moves == [(75.0, 100.0, 400, False)]
        assert sib.homing_moves == []
        assert sib.unit_obj.calls == []
        assert sib.loaded_to_hub is True
        assert rig.logged() == []

    def test_a_unit_without_prep_post_load_is_left_after_the_refeed(
            self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.unit_obj.prep_post_load = None
        rig.unit._restore_sibling(token)
        assert sib.moves == [(75.0, 100.0, 400, False)]
        assert sib.homing_moves == []
        assert sib.unit_obj.calls == []
        assert sib.loaded_to_hub is True
        assert rig.logged() == []

    def test_a_unit_without_prep_load_is_left_after_the_refeed(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        sib.unit_obj.prep_load = None
        rig.unit._restore_sibling(token)
        assert sib.moves == [(75.0, 100.0, 400, False)]
        assert sib.homing_moves == []
        assert sib.unit_obj.calls == []
        assert sib.loaded_to_hub is True
        assert rig.logged() == []

    def test_none_token_restores_nothing(self, monkeypatch):
        rig, sib, token = self._rolled(monkeypatch)
        rig.unit._restore_sibling(None)
        assert sib.moves == [] and sib.homing_moves == []
        assert rig.logged() == []


class TestAFCBoxTurtlerfidHomeBackToLoad:
    """The retract home: AFC's homing move to the load endstop, reversed, at
    long-move speed, with the espooler forced on."""

    def test_a_reverse_home_alone_would_leave_it_off_the_switch(self, monkeypatch):
        # A reverse home stops where the switch releases, so on its own it
        # leaves the tip behind the switch.
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        rig.unit._home_back_to_load(sib, 200.0)
        assert sib.homing_moves == [
            (-200.0, SpeedMode.LONG, "load", True, AssistActive.YES)]
        assert sib.raw_load_state is False
        assert sib.moves == []
        assert rig.logged() == []

    def test_the_forward_home_does_not_fight_the_spool(self, monkeypatch):
        # Only the retract is driven here, with the espooler on: the forward
        # home is prep_load's, which feeds a free-wheeling spool.
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        rig.unit._home_back_to_load(lane, 37.5)
        assert lane.homing_moves == [
            (-37.5, SpeedMode.LONG, "load", True, AssistActive.YES)]
        assert lane.unit_obj.calls == []
        assert lane.moves == []
        assert rig.logged() == []


class TestAFCBoxTurtlerfidRestore:
    """The restore puts the tip back on the load sensor the sweep started
    from. Without homing it gives back the bulk in one move, steps the tail
    while checking the sensor, then creeps forward if it went past."""

    class _LoadStateOnlyLane(BtRfidLane):
        """A lane with load_state but no raw_load_state."""

        @property
        def raw_load_state(self) -> Optional[bool]:
            """:return Optional[bool]: never reported"""
            return None

        @property
        def load_state(self) -> bool:
            """:return bool: whether the load switch reads filament"""
            return self.pos > self.load_at

    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch, lane: BtRfidLane,
             homing: bool = False) -> BtRfidRig:
        return build_bt_rfid(monkeypatch, readers=(("reader0", 0, ("lane8",)),),
                             lanes=[lane], homing=homing)

    STEPPED = ([(-370.0, 100.0, 400, True)] + [(-10.0, 25.0, 400, True)] * 4
               + [(10.0, 25.0, 400, False)])

    def test_a_slipping_feed_still_ends_at_the_sensor(self, monkeypatch):
        # 20% of a 400mm feed never happened, so the tip is at 5 + 320.
        lane = BtRfidLane("lane8", pos=325.0, slip=0.2)
        rig = self._rig(monkeypatch, lane)
        # The 370mm bulk overshoots the switch; six slipping 8mm creeps
        # bring it back on: 370 - 60 given back.
        assert rig.unit._restore(lane, 400.0, 100.0) == 310.0
        assert lane.moves == [(-370.0, 100.0, 400, True)] + [
            (10.0, 25.0, 400, False)] * 6
        assert lane.raw_load_state is True
        assert lane.pos == pytest.approx(3.0)
        assert rig.logged() == []

    def test_a_clean_feed_also_ends_at_the_sensor(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        rig = self._rig(monkeypatch, lane)
        assert rig.unit._restore(lane, 400.0, 100.0) == 400.0
        assert lane.pos == 5.0
        assert rig.logged() == []

    def test_the_bulk_is_one_move_and_only_the_tail_is_stepped(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        rig = self._rig(monkeypatch, lane)
        rig.unit._restore(lane, 400.0, 100.0)
        assert lane.moves == self.STEPPED
        assert rig.logged() == []

    def test_the_tail_steps_bound_how_far_it_can_overshoot(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        rig = self._rig(monkeypatch, lane)
        rig.unit._restore(lane, 400.0, 100.0)
        assert min(m[0] for m in lane.moves[1:]) == -10.0
        assert 0.0 < lane.pos <= 10.0
        assert rig.logged() == []

    def test_a_short_feed_is_all_tail(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=25.0)
        rig = self._rig(monkeypatch, lane)
        assert rig.unit._restore(lane, 20.0, 100.0) == 20.0
        assert lane.moves == [(-10.0, 25.0, 400, True)] * 3 + [
            (10.0, 25.0, 400, False)]
        assert rig.logged() == []

    def test_nothing_on_the_sensor_falls_back_to_the_measured_distance(
            self, monkeypatch):
        lane = BtRfidLane("lane8", pos=-50.0)
        rig = self._rig(monkeypatch, lane, homing=True)
        assert rig.unit._restore(lane, 120.0, 100.0) == 120.0
        assert lane.moves == [(-120.0, 100.0, 400, True)]
        assert lane.homing_moves == []
        assert rig.logged() == []

    def test_it_homes_on_load_not_prep(self, monkeypatch):
        # Prep stays true throughout; only the load switch stops the restore.
        lane = BtRfidLane("lane8", pos=405.0)
        rig = self._rig(monkeypatch, lane)
        rig.unit._restore(lane, 400.0, 100.0)
        assert lane.prep_state is True
        assert lane.raw_load_state is True
        assert lane.pos == 5.0
        assert rig.logged() == []

    def test_a_lane_with_only_load_state_still_homes(self, monkeypatch):
        lane = self._LoadStateOnlyLane("lane8", pos=405.0)
        rig = self._rig(monkeypatch, lane)
        assert rig.unit._restore(lane, 400.0, 100.0) == 400.0
        assert lane.moves == self.STEPPED
        assert lane.load_state is True
        assert rig.logged() == []

    def test_nothing_fed_moves_nothing(self, monkeypatch):
        lane = BtRfidLane("lane8")
        rig = self._rig(monkeypatch, lane)
        assert rig.unit._restore(lane, 0.0, 100.0) == 0.0
        assert lane.moves == [] and lane.homing_moves == []
        assert rig.logged() == []

    def test_a_net_retract_still_on_the_switch_moves_nothing(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=5.0)
        rig = self._rig(monkeypatch, lane, homing=True)
        assert rig.unit._restore(lane, -3.0, 100.0) == 0.0
        assert lane.moves == [] and lane.homing_moves == []
        assert lane.unit_obj.calls == []
        assert rig.logged() == []

    def test_a_tip_behind_the_switch_is_homed_forward_onto_it(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=-15.0)
        rig = self._rig(monkeypatch, lane, homing=True)
        assert rig.unit._restore(lane, -20.0, 100.0) == -20.0
        assert lane.unit_obj.calls == ["prep_load"]
        assert lane.homing_moves == [(400.0, None, "load", True, None)]
        assert lane.moves == []
        assert lane.pos == 0.5
        assert rig.logged() == []

    def test_a_tip_behind_the_switch_without_homing_is_stepped_onto_it(
            self, monkeypatch):
        lane = BtRfidLane("lane8", pos=-15.0)
        rig = self._rig(monkeypatch, lane)
        assert rig.unit._restore(lane, -20.0, 100.0) == -20.0
        assert lane.moves == [(10.0, 25.0, 400, False)] * 2
        assert lane.homing_moves == [] and lane.unit_obj.calls == []
        assert lane.pos == 5.0
        assert rig.logged() == []

    def test_stepping_onto_the_switch_without_a_short_speed_uses_the_given_one(
            self, monkeypatch):
        lane = BtRfidLane("lane8", pos=-5.0)
        lane.short_moves_speed = None
        lane.short_move_dis = None
        rig = self._rig(monkeypatch, lane)
        assert rig.unit._restore(lane, -5.0, 100.0) == -10.0
        assert lane.moves == [(10.0, 100.0, 400, False)]
        assert lane.pos == 5.0
        assert rig.logged() == []

    def test_stepping_onto_the_switch_gives_up_after_its_bound(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=-15.0, slip=1.0)    # feeds go nowhere
        rig = self._rig(monkeypatch, lane)
        # int(20 / 10) + 4 steps, then it stops with the switch still open.
        assert rig.unit._restore(lane, -20.0, 100.0) == -60.0
        assert lane.moves == [(10.0, 25.0, 400, False)] * 6
        assert lane.raw_load_state is False
        assert rig.logged() == []

    def test_the_return_runs_the_espooler_but_the_creep_does_not(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        rig = self._rig(monkeypatch, lane)
        rig.unit._restore(lane, 400.0, 100.0)
        assert [(m[0] < 0, m[3]) for m in lane.moves] == [(True, True)] * 5 + [
            (False, False)]
        assert rig.logged() == []

    def test_a_prep_switch_that_never_releases_still_terminates(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        lane.load_at = -1e9                      # always reads filament
        rig = self._rig(monkeypatch, lane)
        # The tail steps stop once fed + tail (430mm) has been given back.
        assert rig.unit._restore(lane, 400.0, 100.0) == 430.0
        assert lane.moves == [(-370.0, 100.0, 400, True)] + [
            (-10.0, 25.0, 400, True)] * 6
        assert rig.logged() == []

    def test_a_creep_that_cannot_reseat_gives_up_after_its_bound(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0, slip=1.0)    # feeds go nowhere
        rig = self._rig(monkeypatch, lane)
        # The tail overshoots to -5; int(30 / 10) + 4 creeps cannot bring it back.
        assert rig.unit._restore(lane, 400.0, 100.0) == 410.0 - 70.0
        assert lane.moves[5:] == [(10.0, 25.0, 400, False)] * 7
        assert lane.raw_load_state is False
        assert rig.logged() == []

    def test_with_homing_it_homes_back_then_onto_the_switch(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        rig = self._rig(monkeypatch, lane, homing=True)
        assert rig.unit._restore(lane, 400.0, 100.0) == 400.0
        assert lane.homing_moves == [
            (-420.0, SpeedMode.LONG, "load", True, AssistActive.YES),
            (400.0, None, "load", True, None)]
        assert lane.moves == []
        assert lane.pos == 0.5
        assert rig.logged() == []

    def test_homing_without_a_unit_to_home_forward_steps_instead(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        lane.unit_obj = None
        rig = self._rig(monkeypatch, lane, homing=True)
        assert rig.unit._restore(lane, 400.0, 100.0) == 400.0
        assert lane.homing_moves == []
        assert lane.moves == self.STEPPED
        assert rig.logged() == []

    def test_homing_on_a_lane_without_move_to_steps_instead(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        lane.move_to = None
        rig = self._rig(monkeypatch, lane, homing=True)
        assert rig.unit._restore(lane, 400.0, 100.0) == 400.0
        assert lane.homing_moves == []
        assert lane.unit_obj.calls == []
        assert lane.moves == self.STEPPED
        assert lane.pos == 5.0
        assert rig.logged() == []

    def test_longer_short_moves_lengthen_the_tail(self, monkeypatch):
        lane = BtRfidLane("lane8", pos=405.0)
        lane.short_move_dis = 15.0
        rig = self._rig(monkeypatch, lane)
        # Three 15mm steps (45mm) beat the 30mm floor: a 355mm bulk, then the
        # tail in 15mm steps and one creep back onto the switch.
        assert rig.unit._restore(lane, 400.0, 100.0) == 400.0
        assert lane.moves == ([(-355.0, 100.0, 400, True)]
                              + [(-15.0, 25.0, 400, True)] * 4
                              + [(15.0, 25.0, 400, False)])
        assert lane.pos == 5.0
        assert rig.logged() == []


class TestAFCBoxTurtlerfidSweepRoom:
    """A sweep must never drive the tip into a hub another lane is using."""

    CUT_LOADED = ("hub Turtle_1 is in use (lane8 is loaded through it), so the "
                  "sweep stops 25mm short of it")

    def test_a_real_hub_switch_reading_filament_counts_as_busy(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        lane.hub_obj.state = True
        assert rig.unit._sweep_room(lane, 1000.0) == (
            125.0, "hub Turtle_1 is in use (its switch reads filament), so the "
                   "sweep stops 25mm short of it")
        assert rig.logged() == []

    def test_a_virtual_hub_switch_is_not_evidence(self, monkeypatch):
        # A virtual hub's state is every lane's load sensor, this one's included.
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        lane.hub_obj.state = True
        lane.hub_obj._virtual = True
        assert rig.unit._sweep_room(lane, 1000.0) == (1000.0, None)
        assert rig.logged() == []

    def test_a_staged_start_leaves_no_room_before_a_busy_hub(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, staged=True, busy=True)
        assert rig.unit._sweep_room(lane, 1000.0) == (0.0, self.CUT_LOADED)
        assert rig.logged() == []

    def test_a_staged_lane_judged_from_its_switch_gets_the_room(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, staged=True, busy=True)
        assert rig.unit._sweep_room(lane, 1000.0, staged=False) == (
            125.0, self.CUT_LOADED)
        assert rig.logged() == []

    def test_enough_room_before_a_busy_hub_cuts_nothing(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, busy=True, dist_hub=1100.0)
        assert rig.unit._sweep_room(lane, 1000.0) == (1000.0, None)
        assert rig.logged() == []

    def test_a_hub_that_cannot_say_if_it_is_virtual_is_not_evidence(
            self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        lane.hub_obj.state = True
        lane.hub_obj.is_virtual_pin = Recorder(raises=RuntimeError("no pin"))
        assert rig.unit._sweep_room(lane, 1000.0) == (1000.0, None)
        assert lane.hub_obj.is_virtual_pin.call_count == 1
        assert rig.logged() == []

    def test_a_clear_hub_cuts_nothing(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch, dist_hub=20.0)
        assert rig.unit._sweep_room(lane, 1000.0) == (1000.0, None)
        assert rig.logged() == []

    def test_a_lane_loaded_through_another_hub_is_not_evidence(self, monkeypatch):
        hub = BtRfidHub()
        lane = BtRfidLane("lane9", dist_hub=150.0, hub=hub)
        other = BtRfidLane("lane8", dist_hub=150.0, hub=BtRfidHub("Turtle_2"),
                           tool_loaded=True)
        rig = build_bt_rfid(monkeypatch, lanes=[lane, other])
        assert hub.state is False
        assert rig.unit._sweep_room(lane, 1000.0) == (1000.0, None)
        assert rig.logged() == []

    def test_the_lane_itself_being_loaded_is_not_evidence(self, monkeypatch):
        rig, lane = bt_rfid_hub_rig(monkeypatch)
        lane.tool_loaded = True
        assert list(rig.printer.afc.lanes) == ["lane9"]
        assert rig.unit._sweep_room(lane, 1000.0) == (1000.0, None)
        assert rig.logged() == []

    def test_a_lane_on_the_hub_but_not_loaded_is_not_evidence(self, monkeypatch):
        hub = BtRfidHub()
        lane = BtRfidLane("lane9", dist_hub=150.0, hub=hub)
        other = BtRfidLane("lane8", dist_hub=150.0, hub=hub)
        rig = build_bt_rfid(monkeypatch, lanes=[lane, other])
        assert hub.state is False
        assert rig.unit._sweep_room(lane, 1000.0) == (1000.0, None)
        assert rig.logged() == []


class TestAFCBoxTurtlerfidLaneMove:
    """_lane_move issues one lane move, with the espooler only when asked."""

    class _AssistLane(BtRfidLane):
        """A lane that decides its own assist mode."""

        def __init__(self, name: str, result: Any = False,
                     raises: Optional[Exception] = None) -> None:
            super().__init__(name)
            self.get_active_assist = Recorder(result=result, raises=raises)

    def test_a_blind_retract_would_have_pulled_it_out(self, monkeypatch):
        # A slipping feed is short of what was asked, so retracting the
        # commanded distance walks the tip out past the load switch.
        lane = BtRfidLane("lane8", slip=0.2)
        rig = build_bt_rfid(monkeypatch, lanes=[lane])
        rig.unit._lane_move(lane, 400.0, 100.0, assist=False)
        rig.unit._lane_move(lane, -400.0, 100.0, assist=True)
        assert lane.moves == [(400.0, 100.0, 400, False),
                              (-400.0, 100.0, 400, True)]
        assert lane.raw_load_state is False
        assert rig.logged() == []

    def test_the_lane_decides_the_assist_for_a_retract(self, monkeypatch):
        lane = self._AssistLane("lane8", result=False)
        rig = build_bt_rfid(monkeypatch, lanes=[lane])
        rig.unit._lane_move(lane, -50.0, 60.0, assist=True)
        assert lane.get_active_assist.calls == [((-50.0, AssistActive.YES), {})]
        assert lane.moves == [(-50.0, 60.0, 400, False)]
        assert rig.logged() == []

    def test_an_assist_lookup_that_fails_runs_the_espooler(self, monkeypatch):
        lane = self._AssistLane("lane8", raises=ValueError("no spooler"))
        rig = build_bt_rfid(monkeypatch, lanes=[lane])
        rig.unit._lane_move(lane, -50.0, 60.0, assist=True)
        assert lane.moves == [(-50.0, 60.0, 400, True)]
        assert rig.logged() == []

    def test_a_feed_never_asks_for_assist(self, monkeypatch):
        lane = self._AssistLane("lane8", result=True)
        lane.long_moves_accel = 250
        rig = build_bt_rfid(monkeypatch, lanes=[lane])
        rig.unit._lane_move(lane, 50.0, 60.0, assist=False)
        assert lane.get_active_assist.calls == []
        assert lane.moves == [(50.0, 60.0, 250, False)]
        assert rig.logged() == []

    def test_a_lane_without_an_accel_moves_at_400(self, monkeypatch):
        lane = BtRfidLane("lane8")
        del lane.long_moves_accel
        rig = build_bt_rfid(monkeypatch, lanes=[lane])
        rig.unit._lane_move(lane, 50.0, 60.0, assist=False)
        assert lane.moves == [(50.0, 60.0, 400, False)]
        assert rig.logged() == []


class TestAFCBoxTurtlerfidSweepStepFor:
    """A chunk is a whole move, so it must be long enough to cruise: its ramps
    (v^2 / a together) take 40% of it, bounded to 20..150mm."""

    @staticmethod
    def _lane(accel: Optional[float] = 250.0) -> BtRfidLane:
        lane = BtRfidLane("lane8")
        lane.long_moves_accel = accel
        return lane

    @staticmethod
    def _profile(step: float, speed: float, accel: float) -> Tuple[float, float]:
        """(peak velocity, cruise distance) of one trapezoidal chunk."""
        ramp = speed * speed / (2.0 * accel)
        if 2.0 * ramp <= step:
            return speed, step - 2.0 * ramp
        return (accel * step) ** 0.5, 0.0

    def test_the_old_fixed_chunk_never_reached_speed(self, monkeypatch):
        # At 100mm/s and 250mm/s^2 the ramps alone take 40mm, so the old 20mm
        # chunk never cruised; the chosen one is 40 / 0.4 = 100mm.
        rig = build_bt_rfid(monkeypatch)
        step = rig.unit._sweep_step_for(self._lane(), 100.0)
        assert step == 100.0
        # Full speed, with 100 - 40 = 60mm of it at cruise.
        assert self._profile(step, 100.0, 250.0) == (100.0, 60.0)
        assert rig.logged() == []

    @pytest.mark.parametrize("speed, expected", [
        (40.0, 20.0), (60.0, 36.0), (100.0, 100.0), (150.0, 150.0)])
    def test_the_chosen_chunk_actually_cruises(self, monkeypatch, speed, expected):
        rig = build_bt_rfid(monkeypatch)
        step = rig.unit._sweep_step_for(self._lane(), speed)
        assert step == pytest.approx(expected)
        peak, cruise = self._profile(step, speed, 250.0)
        assert peak == pytest.approx(speed)
        assert cruise > 0.0
        assert rig.logged() == []

    def test_a_slower_sweep_needs_a_shorter_chunk(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        assert rig.unit._sweep_step_for(self._lane(), 60.0) == pytest.approx(36.0)
        assert rig.unit._sweep_step_for(self._lane(), 100.0) == 100.0
        assert rig.logged() == []

    def test_higher_acceleration_needs_a_shorter_chunk(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        assert rig.unit._sweep_step_for(self._lane(1000.0), 100.0) == 25.0
        assert rig.unit._sweep_step_for(self._lane(250.0), 100.0) == 100.0
        assert rig.logged() == []

    def test_bounded_at_both_ends(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        assert rig.unit._sweep_step_for(self._lane(1e6), 1.0) == 20.0
        assert rig.unit._sweep_step_for(self._lane(10.0), 400.0) == 150.0
        assert rig.logged() == []

    def test_a_lane_without_an_accel_still_gets_a_chunk(self, monkeypatch):
        rig = build_bt_rfid(monkeypatch)
        # 400mm/s^2 is assumed: 100^2 / 400 = 25mm of ramps, / 0.4.
        assert rig.unit._sweep_step_for(self._lane(None), 100.0) == 62.5
        assert rig.logged() == []


class TestAFCBoxTurtlerfidCmdAFCBTRFIDStage:
    """AFC_BT_RFID_STAGE homes on the tag: feed while polling, stop when it
    answers, give up after a bound in spool turns, then put the filament back.
    It refuses while anything else moves filament."""

    FULL_PASS = ("That is a full pass of the spool, so the tag never enters this "
                 "antenna's field; move the reader rather than sweeping further.")
    ANNOUNCE = ("respond_info",
                "AFC_BT_RFID_STAGE: homing on lane8's tag: up to 1257mm (2.0 turns "
                "of a 200mm spool) at 20mm/s, polling reader0 throughout. Coming "
                "back at 100mm/s.")
    TAG60 = ("respond_info", "AFC_BT_RFID_STAGE: tag answered after 60mm of feed "
                             "(filament restored).")
    TAG60_KEPT = ("respond_info", "AFC_BT_RFID_STAGE: tag answered after 60mm of "
                                  "feed.")
    TAG20 = ("respond_info", "AFC_BT_RFID_STAGE: tag answered after 20mm of feed "
                             "(filament restored).")
    TAG120 = ("respond_info", "AFC_BT_RFID_STAGE: tag answered after 120mm of feed "
                              "(filament restored).")
    TAG120_RESTAGED = ("respond_info", "AFC_BT_RFID_STAGE: tag answered after 120mm "
                                       "of feed (filament restored and re-staged at "
                                       "the hub).")
    CUT_LOADED = ("hub Turtle_1 is in use (lane8 is loaded through it), so the "
                  "sweep stops 25mm short of it")
    STAGED_TOO = (" lane9 comes back to its load switch first and is re-staged at "
                  "the hub afterwards.")
    READ8 = bt_rfid_read_out("lane8", "DEADBEEF")
    READ9 = bt_rfid_read_out("lane9", "DEADBEEF")

    @staticmethod
    def _announce(lane: str, up_to: str, turns: str, speed: str, back: str,
                  extra: str = "") -> Tuple[str, str]:
        return ("respond_info",
                f"AFC_BT_RFID_STAGE: homing on {lane}'s tag: up to {up_to}mm "
                f"({turns} turns of a 200mm spool) at {speed}mm/s, polling reader0 "
                f"throughout. Coming back at {back}mm/s.{extra}")

    @staticmethod
    def _short(turns: str) -> str:
        return (f"That is only {turns} of a turn, so the tag may simply not have "
                f"come round yet; raise REVS before suspecting the mounting.")

    @staticmethod
    def _no_tag(mm: str, verdict: str,
                restored: str = " (filament restored)") -> Tuple[str, str]:
        return ("respond_info",
                f"AFC_BT_RFID_STAGE: no tag in {mm}mm of sweep on reader0{restored}. "
                f"{verdict}")

    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch, tag_after: Optional[float] = 60.0,
             served: Tuple[str, ...] = ("lane8", "lane9"),
             idle: Optional[BtRfidIdle] = None
             ) -> Tuple[BtRfidRig, BtRfidLane]:
        lane = BtRfidLane("lane8")
        rig = build_bt_rfid(monkeypatch, readers=(("reader0", 0, served),),
                            lanes=[lane], homing=False, idle=idle)
        if tag_after is not None:
            rig.field.appear(lane, tag_after)
        return rig, lane

    @staticmethod
    def _hub_rig(monkeypatch: pytest.MonkeyPatch, tag_after: Optional[float] = None,
                 **kw: Any) -> Tuple[BtRfidRig, BtRfidLane]:
        rig, lane = bt_rfid_hub_rig(monkeypatch, **kw)
        if tag_after is not None:
            # Counted from the load switch edge, where every scan starts.
            rig.field.appear(lane, tag_after, start=0.5)
        return rig, lane

    @staticmethod
    def _stage(rig: BtRfidRig, **params: Any) -> FakeGcmd:
        gcmd = FakeGcmd(dict({"LANE": "lane8"}, **params))
        rig.unit.cmd_AFC_BT_RFID_STAGE(gcmd)
        return gcmd

    @staticmethod
    def _refused(rig: BtRfidRig, **params: Any) -> Tuple[str, FakeGcmd]:
        """:return tuple: (the g-code error's message, the command)"""
        gcmd = FakeGcmd(dict({"LANE": "lane8"}, **params))
        with pytest.raises(BT_RFID_CMD_ERROR) as err:
            rig.unit.cmd_AFC_BT_RFID_STAGE(gcmd)
        return str(err.value), gcmd

    # ── the sweep ──

    def test_revs_convert_to_a_circumference_bounded_sweep(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        gcmd = self._stage(rig)
        # 2 turns of a 200mm spool: 2 * pi * 200, the last chunk partial.
        assert sum(lane.feeds()) == pytest.approx(2 * BT_RFID_TURN_MM)
        assert lane.feeds()[-1] == pytest.approx(2 * BT_RFID_TURN_MM - 62 * 20.0)
        assert rig.console == []
        assert gcmd.messages == [self.ANNOUNCE, self._no_tag("1257", self.FULL_PASS)]
        assert rig.logged() == []

    def test_revs_sets_the_bound_in_turns(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        gcmd = self._stage(rig, REVS=1.0)
        assert sum(lane.feeds()) == pytest.approx(BT_RFID_TURN_MM)
        assert gcmd.messages == [self._announce("lane8", "628", "1.0", "20", "100"),
                                 self._no_tag("628", self.FULL_PASS)]
        assert rig.console == []
        assert rig.logged() == []

    def test_it_stops_as_soon_as_the_tag_answers(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig)
        assert lane.feeds() == [20.0, 20.0, 20.0]       # not the 1257mm bound
        assert rig.console == [self.READ8]
        assert lane.weight == 1000
        assert rig.unit.last_reads_status() == {"lane8": bt_rfid_record("DEADBEEF")}
        assert rig.unit._last_uid_by_lane == {"lane8": "deadbeef"}
        assert rig.unit._sweeping is False
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60]
        assert rig.logged() == []

    def test_the_filament_ends_up_where_it_started(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig)
        assert lane.pos == 5.0
        assert lane.raw_load_state is True
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_retract_zero_leaves_the_filament_where_it_stopped(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig, RETRACT=0)
        assert lane.moves == [(20.0, 20.0, 400, False)] * 3
        assert lane.pos == 65.0
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60_KEPT]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_retract_after_read_off_is_the_default_for_retract(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.unit.retract_after_read = False
        gcmd = self._stage(rig)
        assert lane.retracts() == []
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60_KEPT]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_an_explicit_advance_overrides_revs(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        gcmd = self._stage(rig, ADVANCE=100.0)
        assert lane.feeds() == [20.0] * 5
        assert gcmd.messages == [self._announce("lane8", "100", "0.2", "20", "100"),
                                 self._no_tag("100", self._short("0.2"))]
        assert rig.console == []
        assert rig.logged() == []

    def test_a_full_pass_with_no_answer_blames_the_mounting(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        gcmd = self._stage(rig, ADVANCE=700.0)          # more than a 628mm turn
        assert rig.console == []
        assert gcmd.messages == [self._announce("lane8", "700", "1.1", "20", "100"),
                                 self._no_tag("700", self.FULL_PASS)]
        assert rig.logged() == []

    def test_a_short_sweep_blames_the_distance_not_the_mounting(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        gcmd = self._stage(rig, ADVANCE=600.0)          # just short of a turn
        assert gcmd.messages == [self._announce("lane8", "600", "1.0", "20", "100"),
                                 self._no_tag("600", self._short("1.0"))]
        assert rig.console == []
        assert rig.logged() == []

    def test_a_hit_reports_the_distance_and_nothing_more(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig)
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_an_offline_reader_moves_no_filament(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.readers["reader0"].version = None
        message, gcmd = self._refused(rig)
        assert message == ("AFC_BT_RFID_STAGE: reader0 is offline: no point moving "
                           "filament at a reader that cannot answer. Check the "
                           "bridge with AFC_BT_RFID_STATUS.")
        assert lane.moves == []
        assert rig.field.calls == []
        assert gcmd.messages == []
        assert rig.console == []
        assert rig.logged() == []

    def test_a_reader_lane_that_is_not_an_afc_lane_is_refused(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        message, gcmd = self._refused(rig, LANE="lane9")
        assert message == "AFC_BT_RFID_STAGE: lane9 is not an AFC lane"
        assert gcmd.messages == []
        assert rig.console == []
        assert rig.logged() == []

    def test_the_feed_leaves_the_espooler_alone(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig)
        # Sweep chunks and the closing creep feed unassisted; only retracts assist.
        assert lane.moves == ([(20.0, 20.0, 400, False)] * 3
                              + [(-30.0, 100.0, 400, True)]
                              + [(-10.0, 25.0, 400, True)] * 4
                              + [(10.0, 25.0, 400, False)])
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_the_restoring_retract_runs_the_espooler(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig)
        assert [a for _d, _s, a in lane.retracts()] == [True] * 5
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_the_bulk_of_the_return_is_one_fast_move(self, monkeypatch):
        # 60mm fed: a 30mm bulk in one move, then the 30mm tail stepped.
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig, STEP=20.0)
        assert [d for d, _s, _a in lane.retracts()] == [-30.0, -10.0, -10.0,
                                                        -10.0, -10.0]
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_the_return_uses_the_lane_speed_not_the_sweep_speed(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig, SPEED=20.0)
        assert [m[1] for m in lane.moves[:3]] == [20.0] * 3
        # The bulk at the lane's long-move speed, the tail at its short one.
        assert [s for _d, s, _a in lane.retracts()] == [100.0] + [25.0] * 4
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_retract_speed_can_be_given_explicitly(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig, SPEED=20.0, RETRACT_SPEED=75.0)
        assert lane.retracts()[0] == (-30.0, 75.0, True)
        assert gcmd.messages == [self._announce("lane8", "1257", "2.0", "20", "75"),
                                 self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_a_lane_with_no_long_move_speed_falls_back_to_the_sweep(
            self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None,
                                       served=("lane8",))
        lane.long_moves_speed = None
        rig.readers["reader0"].link = None       # nothing can answer
        gcmd = self._stage(rig, SPEED=17.0)
        assert lane.retracts()[0] == (pytest.approx(-(2 * BT_RFID_TURN_MM - 30.0)),
                                      17.0, True)
        assert rig.field.calls == []
        assert gcmd.messages == [self._announce("lane8", "1257", "2.0", "17", "17"),
                                 self._no_tag("1257", self.FULL_PASS)]
        assert rig.console == []
        assert rig.logged() == []

    def test_the_announcement_names_the_return_speed(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        gcmd = self._stage(rig, SPEED=20.0)
        assert gcmd.messages == [
            ("respond_info", "AFC_BT_RFID_STAGE: homing on lane8's tag: up to 1257mm "
                             "(2.0 turns of a 200mm spool) at 20mm/s, polling reader0 "
                             "throughout. Coming back at 100mm/s."),
            self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_a_partial_last_chunk_does_not_overshoot_the_bound(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        gcmd = self._stage(rig, ADVANCE=50.0, STEP=20.0)
        assert lane.feeds() == [20.0, 20.0, 10.0]
        assert gcmd.messages == [self._announce("lane8", "50", "0.1", "20", "100"),
                                 self._no_tag("50", self._short("0.1"))]
        assert rig.console == []
        assert rig.logged() == []

    def test_no_step_sizes_the_chunk_from_the_lane(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        lane.long_moves_accel = 250
        rig.unit.tag_sweep_step_mm = 0.0
        # (100^2 / 250) / 0.4 = 100mm chunks.
        gcmd = self._stage(rig, ADVANCE=250.0, SPEED=100.0)
        assert lane.feeds() == [100.0, 100.0, 50.0]
        assert gcmd.messages == [self._announce("lane8", "250", "0.4", "100", "100"),
                                 self._no_tag("250", self._short("0.4"))]
        assert rig.console == []
        assert rig.logged() == []

    def test_the_sister_is_rolled_off_and_put_back_around_the_sweep(
            self, monkeypatch):
        rig, lane, sib = bt_rfid_sister_rig(monkeypatch)
        gcmd = self._stage(rig, ADVANCE=40.0, STEP=20.0)
        assert sib.moves == ([(-10.0, 100.0, 400, True)] * 7
                             + [(-5.0, 100.0, 400, True)]
                             + [(75.0, 100.0, 400, False),
                                (200.0, 100.0, 400, False)])
        assert sib.loaded_to_hub is True
        assert lane.feeds() == [20.0, 20.0]
        assert rig.console == []
        assert rig.unit._sweeping is False
        assert gcmd.messages == [self._announce("lane8", "40", "0.1", "20", "100"),
                                 self._no_tag("40", self._short("0.1"))]
        assert rig.logged() == [
            ("info", "AFC_BT_RFID: tag CAFE1234 is parked on reader0's antenna "
                     "while lane8 is read, clearing lane9 off it."),
            ("info", "AFC_BT_RFID: rolled lane9 back 75mm to clear its tag off "
                     "reader0's antenna while lane8 is read."),
            ("info", "AFC_BT_RFID: re-staged lane9 the way a load does: homed to "
                     "its load switch, then fed dist_hub (200mm). The hub is "
                     "untouched.")]

    # ── refusing while something else moves ──

    def test_it_refuses_while_the_toolhead_is_moving(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, idle=BtRfidIdle("Printing"))
        message, gcmd = self._refused(rig)
        assert message == (
            "AFC_BT_RFID_STAGE: the printer is moving something already "
            "(idle_timeout says Printing): a TD-1 capture, a toolchange or a print. "
            "Two filament moves at once corrupt step generation and shut down every "
            "MCU, so this waits rather than joining in. Try again once it is idle.")
        assert lane.moves == []
        assert gcmd.messages == []
        assert rig.unit._sweeping is False
        assert rig.console == []
        assert rig.logged() == []

    def test_it_runs_when_the_printer_is_idle(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=20.0,
                                       idle=BtRfidIdle("Idle"))
        gcmd = self._stage(rig)
        assert rig.console == [self.READ8]
        assert gcmd.messages == [self.ANNOUNCE, self.TAG20]
        assert rig.logged() == []

    def test_a_second_sweep_cannot_start_inside_the_first(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        rig.unit._sweeping = True
        message, gcmd = self._refused(rig)
        assert message == ("AFC_BT_RFID_STAGE: a sweep is already running. Wait for "
                           "it to finish or restart the firmware.")
        assert lane.moves == []
        assert rig.unit._sweeping is True
        assert gcmd.messages == []
        assert rig.console == []
        assert rig.logged() == []

    def test_the_flag_is_cleared_after_a_normal_sweep(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        seen: List[bool] = []
        rig.field.on_probe = lambda: seen.append(rig.unit._sweeping)
        rig.printer.afc.lanes["lane9"] = BtRfidLane("lane9")
        gcmd = self._stage(rig)
        assert seen == [True]                   # held during the sweep
        assert rig.unit._sweeping is False
        assert gcmd.messages == [self.ANNOUNCE, self.TAG60]
        assert rig.console == [self.READ8]
        assert rig.logged() == []

    def test_the_flag_is_cleared_when_the_sweep_raises(self, monkeypatch):
        rig, lane = self._rig(monkeypatch)
        lane.fail_after = 0
        gcmd = FakeGcmd({"LANE": "lane8"})
        with pytest.raises(RuntimeError) as err:
            rig.unit.cmd_AFC_BT_RFID_STAGE(gcmd)
        assert str(err.value) == "stepper gone"
        assert rig.unit._sweeping is False
        assert gcmd.messages == [self.ANNOUNCE]
        assert rig.console == []
        assert rig.logged() == []

    def test_no_idle_timeout_object_does_not_block_the_command(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=20.0)
        rig.printer._objects["idle_timeout"] = None
        gcmd = self._stage(rig)
        assert rig.console == [self.READ8]
        assert gcmd.messages == [self.ANNOUNCE, self.TAG20]
        assert rig.logged() == []

    def test_an_idle_timeout_that_raises_does_not_block_the_command(
            self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=20.0,
                                       idle=BtRfidIdle("Printing", raises=True))
        gcmd = self._stage(rig)
        assert rig.console == [self.READ8]
        assert gcmd.messages == [self.ANNOUNCE, self.TAG20]
        assert rig.logged() == []

    # ── worker threads ──

    def test_the_closing_read_runs_on_a_worker_thread(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=None)
        gcmd = self._stage(rig, ADVANCE=40.0, STEP=20.0)
        reads = rig.field.reads()
        assert reads[-1] == "afc_bt_rfid_rd"
        assert set(reads) <= {"afc_bt_rfid_sw", "afc_bt_rfid_rd"}
        assert gcmd.messages == [self._announce("lane8", "40", "0.1", "20", "100"),
                                 self._no_tag("40", self._short("0.1"))]
        assert rig.console == []
        assert rig.logged() == []

    def test_the_read_and_sweep_threads_are_named(self, monkeypatch):
        rig, lane = self._rig(monkeypatch, tag_after=20.0)
        gcmd = self._stage(rig, ADVANCE=40.0, STEP=20.0)
        # The sweep's poller reads the tag.
        assert set(rig.field.reads()) == {"afc_bt_rfid_sw"}
        assert gcmd.messages == [self._announce("lane8", "40", "0.1", "20", "100"),
                                 self.TAG20]
        assert rig.console == [self.READ8]
        # With the tag gone, the closing read runs as well.
        rig.field.lane = None
        again = self._stage(rig, ADVANCE=40.0, STEP=20.0)
        reads = rig.field.reads()
        assert reads[-1] == "afc_bt_rfid_rd"
        assert set(reads) == {"afc_bt_rfid_sw", "afc_bt_rfid_rd"}
        assert rig.console == [self.READ8]
        assert again.messages == [self._announce("lane8", "40", "0.1", "20", "100"),
                                  self._no_tag("40", self._short("0.1"))]
        assert rig.logged() == []

    # ── a lane staged at the hub, or a hub in use ──

    def test_a_lane_in_the_toolhead_is_refused(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch)
        lane.tool_loaded = True
        message, gcmd = self._refused(rig, LANE="lane9")
        assert message == ("AFC_BT_RFID_STAGE: lane9 is loaded in the toolhead. The "
                           "scan turns the spool by moving its filament, so unload "
                           "it first.")
        assert lane.moves == [] and lane.homing_moves == []
        assert gcmd.messages == []
        assert rig.console == []
        assert rig.logged() == []

    def test_a_staged_lane_without_homing_is_refused(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, staged=True, homing=False)
        message, gcmd = self._refused(rig, LANE="lane9")
        assert message == ("AFC_BT_RFID_STAGE: lane9 is staged at the hub and homing "
                           "is off, so there is no load switch to bring it back to "
                           "for the scan. Eject it and insert it again to scan it.")
        assert lane.moves == [] and lane.homing_moves == []
        assert gcmd.messages == []
        assert rig.console == []
        assert rig.logged() == []

    def test_no_room_before_a_busy_hub_is_refused(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, busy=True, dist_hub=20.0)
        message, gcmd = self._refused(rig, LANE="lane9")
        assert message == (f"AFC_BT_RFID_STAGE: {self.CUT_LOADED}, and lane9's "
                           "dist_hub leaves no room before it. Unload that lane "
                           "first to scan this one.")
        assert lane.moves == []
        assert gcmd.messages == []
        assert rig.console == []
        assert rig.logged() == []

    def test_a_staged_lane_is_scanned_from_its_load_switch_and_restaged(
            self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, tag_after=120.0, staged=True)
        gcmd = self._stage(rig, LANE="lane9")
        # The sweep started on the load switch edge, not at the hub.
        assert lane.feed_starts[0] == (0.5, False)
        # Six sweep chunks, then prep_post_load's dist_hub feed.
        assert lane.moves == [(20.0, 20.0, 400, False)] * 6 + [
            (150.0, 100.0, 400, False)]
        assert lane.loaded_to_hub is True
        assert lane.unit_obj.calls == ["prep_load", "prep_load", "prep_post_load"]
        assert lane.pos == 0.5 + 150.0
        assert rig.console == [self.READ9]
        assert rig.unit.last_reads_status() == {"lane9": bt_rfid_record("DEADBEEF")}
        assert rig.unit._sweeping is False
        assert gcmd.messages == [
            self._announce("lane9", "1257", "2.0", "20", "100", self.STAGED_TOO),
            self.TAG120_RESTAGED]
        assert rig.logged() == []

    def test_a_staged_lane_is_restaged_when_no_tag_answers(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, staged=True)
        gcmd = self._stage(rig, LANE="lane9")
        assert lane.loaded_to_hub is True
        assert rig.console == []
        assert gcmd.messages == [
            self._announce("lane9", "1257", "2.0", "20", "100", self.STAGED_TOO),
            self._no_tag("1257", self.FULL_PASS,
                         " (filament restored and re-staged at the hub)")]
        assert rig.logged() == []

    def test_retract_off_is_ignored_on_a_staged_lane(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, tag_after=120.0, staged=True)
        gcmd = self._stage(rig, LANE="lane9", RETRACT=0)
        assert lane.loaded_to_hub is True
        assert lane.homing_moves[-2][0] == -(120.0 + 20.0)    # the restore ran
        assert gcmd.messages == [
            self._announce("lane9", "1257", "2.0", "20", "100", self.STAGED_TOO),
            self.TAG120_RESTAGED]
        assert rig.console == [self.READ9]
        assert rig.logged() == []

    def test_a_restage_that_does_not_take_is_reported(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, tag_after=120.0, staged=True)
        lane.unit_obj.prep_post_load = Recorder()          # leaves it unstaged
        gcmd = self._stage(rig, LANE="lane9")
        # prep_post_load leaves it unstaged without a word, so the scan logs the
        # warning the reply points at.
        assert lane.loaded_to_hub is False
        assert lane.unit_obj.prep_post_load.calls == [((lane,), {})]
        assert gcmd.messages == [
            self._announce("lane9", "1257", "2.0", "20", "100", self.STAGED_TOO),
            ("respond_info", "AFC_BT_RFID_STAGE: tag answered after 120mm of feed "
                             "(filament restored, but it is not staged at the hub; "
                             "see the warning).")]
        assert rig.console == [self.READ9]
        assert rig.logged() == [
            ("warning", "AFC_BT_RFID: lane9 is back on its load switch, but "
                        "re-staging it at the hub did not take, so it was left at "
                        "the load switch.")]

    def test_a_staged_lane_that_will_not_come_back_is_refused(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, staged=True)
        lane.unit_obj.prep_load = Recorder()               # never makes the switch
        message, gcmd = self._refused(rig, LANE="lane9")
        assert message == ("AFC_BT_RFID_STAGE: lane9 did not come back onto its load "
                           "switch. Check the spool; its next load will re-home it")
        assert rig.unit._sweeping is False
        assert lane.loaded_to_hub is False
        assert lane.unit_obj.prep_load.calls == [((lane,), {})]
        assert lane.moves == []
        assert gcmd.messages == [
            self._announce("lane9", "1257", "2.0", "20", "100", self.STAGED_TOO)]
        assert rig.console == []
        assert rig.logged() == []

    def test_an_unstaged_lane_is_not_staged_by_the_scan(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, tag_after=120.0)
        gcmd = self._stage(rig, LANE="lane9")
        assert lane.loaded_to_hub is False
        assert lane.unit_obj.calls == ["prep_load"]
        assert lane.pos == 0.5                         # back on its load switch
        assert gcmd.messages == [
            self._announce("lane9", "1257", "2.0", "20", "100"), self.TAG120]
        assert rig.console == [self.READ9]
        assert rig.logged() == []

    def test_a_busy_hub_stops_the_sweep_short_of_it(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, tag_after=120.0, busy=True)
        gcmd = self._stage(rig, LANE="lane9")
        assert lane.feeds() == [20.0] * 6
        assert gcmd.messages == [
            self._announce("lane9", "125", "0.2", "20", "100",
                           f" Limited: {self.CUT_LOADED}."),
            self.TAG120]
        assert rig.console == [self.READ9]
        assert rig.logged() == []

    def test_a_cut_sweep_with_no_tag_blames_the_busy_hub(self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, busy=True)
        gcmd = self._stage(rig, LANE="lane9")
        assert lane.feeds() == [20.0] * 6 + [5.0]
        assert gcmd.messages == [
            self._announce("lane9", "125", "0.2", "20", "100",
                           f" Limited: {self.CUT_LOADED}."),
            self._no_tag("125", f"Only 0.2 of a turn fitted because "
                                f"{self.CUT_LOADED}. Unload that lane to give the "
                                f"scan a full turn.")]
        assert rig.console == []
        assert rig.logged() == []

    def test_a_staged_lane_on_a_busy_hub_gets_the_room_from_its_load_switch(
            self, monkeypatch):
        rig, lane = self._hub_rig(monkeypatch, staged=True, busy=True)
        gcmd = self._stage(rig, LANE="lane9")
        # 125mm swept from the load switch, then prep_post_load's dist_hub.
        assert lane.moves == [(20.0, 20.0, 400, False)] * 6 + [
            (5.0, 20.0, 400, False), (150.0, 100.0, 400, False)]
        assert lane.feed_starts[0] == (0.5, False)
        assert lane.loaded_to_hub is True
        assert gcmd.messages == [
            self._announce("lane9", "125", "0.2", "20", "100",
                           f"{self.STAGED_TOO} Limited: {self.CUT_LOADED}."),
            self._no_tag("125", f"Only 0.2 of a turn fitted because "
                                f"{self.CUT_LOADED}. Unload that lane to give the "
                                f"scan a full turn.",
                         " (filament restored and re-staged at the hub)")]
        assert rig.console == []
        assert rig.logged() == []
