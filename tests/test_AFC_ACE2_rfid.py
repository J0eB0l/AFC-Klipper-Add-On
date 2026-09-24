"""Unit tests for extras/AFC_ACE2_rfid.py."""

from __future__ import annotations

from typing import Any, Callable, Dict, Iterable, List, Optional, Set, Tuple, Union

import pytest

from extras.AFC_ACE2 import afcACE2
from extras.AFC_ACE2_rfid import AFC_ACE2_RFID, load_config
import extras.AFC_ACE2_rfid as ace2_rfid_module
from extras.AFC_rfid_readers import decode_btt
from extras.AFC_rfid_write import StageError
from tests.ace_helpers import (
    ace_status,
    AceConfig,
    AcePrinter,
    DEFAULT,
    FakeAce2Connection,
    FakeTransport,
    LaneSpec,
    make_ace2_link,
    make_ace2_reg_link,
    make_ace2_rfid,
    make_ace2_unit,
    make_fake_ace_connection,
    make_gcmd,
    Recorder,
)


CommandError = AcePrinter.command_error


class Ace2RfidConnection(FakeAce2Connection):
    """FakeAce2Connection whose fire-and-forget sends can be watched or made
    to fail: async_hook(method, params) runs after each recorded async send
    and an exception it returns is raised, as a wedged serial write would."""
    async_hook: Optional[Callable[[str, Dict[str, Any]], Optional[BaseException]]] = None

    def send_command_async(self, method: str, params: Optional[Dict[str, Any]] = None) -> None:
        """
        Record the send as the scripted transport does, then run async_hook.

        :param method: method name
        :param params: params dict
        """
        super().send_command_async(method, params)
        hook = self.async_hook
        if hook is not None:
            error = hook(method, dict(params or {}))
            if error is not None:
                raise error


def ace2_rfid_rig(values: Optional[Dict[str, Any]] = None, *,
                  lanes: Iterable[Union[str, LaneSpec]] = (),
                  link: bool = True,
                  bind: bool = True, settle: bool = True,
                  **unit_options: Any) -> Tuple[AFC_ACE2_RFID, afcACE2]:
    """
    Build an afcACE2 with real lanes on an Ace2RfidConnection, and an
    AFC_ACE2_RFID bound to it, both through their real __init__.

    :param values: [AFC_ACE2_rfid] options
    :param lanes: the unit's lanes (names or LaneSpecs); prepped slots read "ready"
    :param link: give the unit a link; False leaves unit._ace None
    :param bind: run the reader's klippy:ready handler
    :param settle: run the bind-time identify retry while it is due
    :param unit_options: other make_ace2_unit keywords
    :return tuple: (reader, unit)
    """
    printer = AcePrinter()
    lanes = list(lanes)
    states = ["empty"] * 4
    for index, spec in enumerate(lanes):
        if isinstance(spec, LaneSpec) and spec.prep:
            states[index if spec.slot is None else spec.slot] = "ready"
    conn = None
    if link:
        conn = make_fake_ace_connection(ace2=True, printer=printer, info=DEFAULT,
                                        status=ace_status(*states), cls=Ace2RfidConnection)
    unit = make_ace2_unit(lanes=lanes, printer=printer, connection=conn, **unit_options)
    reader = make_ace2_rfid(values, ace2=unit, bind=bind, settle=settle)
    return reader, unit


class Ace2RfidMotion:
    """
    Moves filament on a unit's scripted link by the reactor clock, as an ACE
    does: a feed or unwind starts start_delay seconds after it is sent, runs
    at min(speed, 90) mm/s and reads "busy" in get_status while it runs; a
    stop_feed_filament ends it where it is. pos(slot) is the filament's
    position in mm from where the test started it (feeds positive). blips are
    busy/idle answers given to the next status reads before the clock model;
    start_blips are put there as each move is sent (a unit still finishing
    its last command). A slot in empty_slots reads "empty" with the unit idle
    (spool pulled).
    """

    def __init__(self, unit: afcACE2, start_delay: float = 0.1) -> None:
        """
        :param unit: the unit whose Ace2RfidConnection is driven
        :param start_delay: seconds a move takes to start
        """
        self.reactor = unit.printer.reactor
        self.start_delay = start_delay
        self.base: Dict[int, float] = {}
        self.move: Optional[Tuple[int, int, float, float, float]] = None
        self.blips: List[bool] = []
        self.start_blips: List[bool] = []
        self.empty_slots: Set[int] = set()
        conn = unit._ace
        conn.set_reply("feed_filament", lambda params: self._start(params, 1))
        conn.set_reply("unwind_filament", lambda params: self._start(params, -1))
        conn.set_reply("get_status", self._status)
        conn.async_hook = self._on_async

    def _travel(self, now: float) -> Tuple[Optional[int], float, bool]:
        """
        :param now: reactor time
        :return tuple: (slot moving or None, signed mm done so far, still running)
        """
        if self.move is None:
            return None, 0.0, False
        slot, sign, length, speed, t_start = self.move
        run = length / speed
        if now >= t_start + run:
            return slot, sign * length, False
        return slot, sign * max(0.0, now - t_start) * speed, now >= t_start

    def pos(self, slot: int) -> float:
        """
        :param slot: physical slot
        :return float: the slot's filament position now, mm
        """
        moving, done, _ = self._travel(self.reactor.monotonic())
        return self.base.get(slot, 0.0) + (done if moving == slot else 0.0)

    def _settle(self) -> None:
        """Fold the current move into its slot's base position and end it."""
        slot, done, _ = self._travel(self.reactor.monotonic())
        if slot is not None:
            self.base[slot] = self.base.get(slot, 0.0) + done
        self.move = None

    def _start(self, params: Dict[str, Any], sign: int) -> Dict[str, Any]:
        """
        :param params: the feed/unwind params the link sent
        :param sign: 1 feeds, -1 unwinds
        :return dict: the empty reply the ACE gives
        """
        self._settle()
        speed = min(float(params["speed"]), 90.0)
        self.move = (int(params["index"]), sign, float(params["length"]), speed,
                     self.reactor.monotonic() + self.start_delay)
        self.blips = list(self.start_blips)
        return {}

    def _on_async(self, method: str, params: Dict[str, Any]) -> Optional[BaseException]:
        """
        :param method: async method sent
        :param params: its params
        :return Optional[BaseException]: never fails the send
        """
        if method == "stop_feed_filament":
            self._settle()
        return None

    def _status(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        :param params: get_status params (none)
        :return dict: the unit status now
        """
        slots = ["empty" if s in self.empty_slots else "ready" for s in range(4)]
        if self.empty_slots:
            return ace_status(*slots)
        if self.blips:
            return ace_status(*slots, status="busy" if self.blips.pop(0) else "ready")
        _, _, running = self._travel(self.reactor.monotonic())
        return ace_status(*slots, status="busy" if running else "ready")


class Ace2RfidTagScript:
    """
    Stands in for read_tag in extras.AFC_ACE2_rfid: each call answers with the
    next result (the last repeats). An exception instance is raised; a
    callable is called with the call's keywords and its result used. Records
    each link's reg index in .slots and each call's keywords in .calls.
    """

    def __init__(self, monkeypatch: pytest.MonkeyPatch, *results: Any) -> None:
        """
        :param monkeypatch: the test's monkeypatch
        :param results: answers in order
        """
        self.results = list(results)
        self.slots: List[int] = []
        self.calls: List[Dict[str, Any]] = []
        monkeypatch.setattr(ace2_rfid_module, "read_tag", self)

    def __call__(self, link: Any, **kwargs: Any) -> Any:
        """
        :param link: the register link read through
        :return Any: the scripted answer
        """
        self.slots.append(link.slot)
        self.calls.append(kwargs)
        result = self.results.pop(0) if len(self.results) > 1 else self.results[0]
        if callable(result):
            result = result(**kwargs)
        if isinstance(result, BaseException):
            raise result
        return result


#: Options for a host-decode reader that hands identify back after a read.
ACE2_RFID_RESTORING = {"stage_read": False, "probe_settle": 0.0,
                       "skip_factory_autostage": False}


#: A fully decoded tag, as read_tag returns one.
ACE2_RFID_PLA = {"uid": "aa", "tag_type": "MifareClassic1k", "filament": {"type": "PLA"}}


def ace2_rfid_hold_prompt(where: str) -> List[Tuple[str, str]]:
    """
    The "hold the spool" dialog a scan pops on first sight of a tag.

    :param where: the lane or slot label
    :return list: its console lines
    """
    return [("raw", "// action:prompt_begin RFID Scan"),
            ("raw", f"// action:prompt_text Tag detected on {where}, hold the spool at the "
                    f"reader until the read completes…"),
            ("raw", "// action:prompt_show")]


#: The popup title of a scan on the scanner lane scan_lane.
ACE2_RFID_SCAN_TITLE = "Spool Scanned on scan_lane"


def ace2_rfid_scan_popup(title: str, lines: List[str]) -> List[Tuple[str, str]]:
    """
    The console summary and dialog _notify_scan shows for a scanned spool,
    for a scan on a lane (TestAFCACE2RFIDNotifyScan writes them out in full).

    :param title: the popup title, e.g. "Spool Scanned on lane1"
    :param lines: the summary lines
    :return list: its console lines
    """
    body = "".join(f"\n  {line}" for line in lines)
    return ([("info", f"{title}:{body}"), ("raw", f"// action:prompt_begin {title}")]
            + [("raw", f"// action:prompt_text {line}") for line in lines]
            + [("raw", "// action:prompt_footer_button OK|RESPOND TYPE=command "
                       "MSG=action:prompt_end|info"),
               ("raw", "// action:prompt_show")])


#: A decoded tag's slot_info, the fields _spool_details and _notify_scan read.
ACE2_RFID_SLOT_INFO = {"brand": "Bambu", "material": "PLA", "color_hex": "#112233",
                       "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60,
                       "weight_g": 1000, "uid": "aa"}


def ace2_rfid_sent(unit: afcACE2) -> List[Tuple[str, Dict[str, Any]]]:
    """
    :param unit: the unit
    :return list: what its link sent, status polls left out
    """
    return [c for c in unit._ace.commands if c[0] != "get_status"]


#: The log lines of a host-decode probe teardown that keeps identify off.
ACE2_RFID_TEARDOWN = [("info", "ACE2 RFID teardown: reader_power(off)..."),
                      ("info", "ACE2 RFID teardown: reader_power(off) done"),
                      ("info", "ACE2 RFID teardown: identify NOT restored (config)")]


def ace2_rfid_read_out(lane: str, uid: str) -> Tuple[str, str]:
    """
    :param lane: lane name
    :param uid: tag UID
    :return tuple: the console read-out applying a decoded PLA tag prints
    """
    return ("info", f"ACE2 RFID: read spool on {lane}\n  Name: PLA\n  Material: PLA\n"
                    f"  Diameter: 1.75mm\n  Tag UID: {uid}")


def ace2_rfid_assist_off(*slots: int) -> List[Tuple[str, str]]:
    """
    :param slots: the slots whose feed assist a stage read stops
    :return list: the log lines it writes
    """
    return [("info", f"ACE2 RFID: feed assist stopped on slot {s}") for s in slots]


def ace2_rfid_sister_rig(values: Optional[Dict[str, Any]] = None,
                         sibling: Optional[Dict[str, Any]] = None,
                         motion: bool = True, **unit_options: Any
                         ) -> Tuple[AFC_ACE2_RFID, afcACE2]:
    """
    Reader 1 with lane2 on slot 2 and its sibling lane3 on slot 3, movable by
    default (prepped, staged at the hub, not in the tool).

    :param values: [AFC_ACE2_rfid] options over the lane map
    :param sibling: LaneSpec fields for lane3 over the movable default
    :param motion: run moves on the link by the reactor clock (Ace2RfidMotion)
    :param unit_options: other make_ace2_unit keywords
    :return tuple: reader and unit
    """
    spec = dict({"prep": True, "load": True}, **(sibling or {}))
    reader, unit = ace2_rfid_rig(
        dict({"lane_slot_map": "lane2:2, lane3:3"}, **(values or {})),
        lanes=[LaneSpec("lane2", slot=2, prep=True), LaneSpec("lane3", slot=3, **spec)],
        **unit_options)
    if motion:
        Ace2RfidMotion(unit)
    return reader, unit


def ace2_rfid_moves(unit: afcACE2) -> List[Tuple[str, Dict[str, Any]]]:
    """
    :param unit: the unit
    :return list: the feed and unwind commands its link sent
    """
    return [c for c in unit._ace.commands if c[0] in ("unwind_filament", "feed_filament")]


#: Options for a rescan rig: lane1 and lane2 share reader 0, identify handed back.
ACE2_RFID_RESCAN_VALUES = {"lane_slot_map": "lane1:0, lane2:1", "skip_factory_autostage": False,
                           "stage_scan_speed": 25.0, "stage_read_hold_attempts": 1}


def ace2_rfid_rescan_rig(lane1: Optional[Dict[str, Any]] = None,
                         values: Optional[Dict[str, Any]] = None, **unit_options: Any
                         ) -> Tuple[AFC_ACE2_RFID, afcACE2, Ace2RfidMotion]:
    """
    lane1 (slot 0, 1250mm staged) and lane2 (slot 1) share reader 0; both are
    prepped and staged at the hub.

    :param lane1: LaneSpec fields for lane1 over that default
    :param values: options over ACE2_RFID_RESCAN_VALUES
    :param unit_options: other make_ace2_unit keywords
    :return tuple: reader, unit and its motion
    """
    spec = dict({"prep": True, "load": True, "values": {"dist_hub": 1250}}, **(lane1 or {}))
    reader, unit = ace2_rfid_rig(dict(ACE2_RFID_RESCAN_VALUES, **(values or {})),
                                 lanes=[LaneSpec("lane1", slot=0, **spec),
                                        LaneSpec("lane2", slot=1, prep=True, load=True)],
                                 **unit_options)
    return reader, unit, Ace2RfidMotion(unit)


def ace2_rfid_rescan_tag(monkeypatch: pytest.MonkeyPatch, motion: Ace2RfidMotion,
                         tag_at: Optional[float], uid: str = "beef",
                         filament: Optional[Dict[str, Any]] = DEFAULT) -> Ace2RfidTagScript:
    """
    Put lane1's tag tag_at mm into the 500mm scan feed: the field shows uid
    within 15mm of it while that feed runs, and a read at rest answers within
    6mm of it.

    :param monkeypatch: the test's monkeypatch
    :param motion: the unit's motion
    :param tag_at: mm into the scan feed, None for no tag
    :param uid: the UID the field shows
    :param filament: what the read at rest decodes; DEFAULT is PLA, None a blank tag
    :return Ace2RfidTagScript: the reads at rest
    """
    spot = None if tag_at is None else tag_at - 500.0

    def near(span: float) -> bool:
        return spot is not None and abs(motion.pos(0) - spot) <= span

    class FieldScript:
        def __init__(self, mfrc: Any) -> None:
            self.mfrc = mfrc

        def activate(self) -> Tuple[Optional[bytes], Optional[int]]:
            move = motion.move
            scanning = move is not None and move[1] == 1 and move[2] == 500.0
            return (bytes.fromhex(uid), 0x08) if scanning and near(15.0) else (None, None)

    monkeypatch.setattr(ace2_rfid_module, "MifareClassic", FieldScript)
    tag = {"uid": "beef", "tag_type": "MifareClassic1k",
           "filament": {"type": "PLA"} if filament is DEFAULT else filament}
    return Ace2RfidTagScript(monkeypatch, lambda **kwargs: tag if near(6.0) else None)


def ace2_rfid_rescan_logs(*steps: str) -> List[Tuple[str, str]]:
    """
    :param steps: the rescan's own step lines
    :return list: the log of a lane1 rescan that ran its moves with them
    """
    return (ace2_rfid_assist_off(0, 1)
            + [("info", f"ACE2 RFID rescan lane1: {step}") for step in steps]
            + [("info", "ACE2 RFID: feed assist restored on slot 0"),
               ("info", "ACE2 RFID: feed assist restored on slot 1")])


class TestAce2LinkBuildFrame:
    def test_frame_crc_matches_firmware_algo(self):
        link = make_ace2_link()
        assert link.build_frame(0x06, b"") == bytes.fromhex("ffaa01 0000 0600 f157 fe")
        # only the sequence byte moves on the next frame
        assert link.build_frame(0x06, b"") == bytes.fromhex("ffaa02 0000 0600 f157 fe")
        assert link._seq == 2


class TestAce2LinkRegRead:
    def test_returns_parsed_value_and_encodes_arg(self):
        reply = bytes.fromhex("ffaa01 0000 5002 0842 08c7 fe")
        link = make_ace2_link(1, transport=FakeTransport(reply))
        assert link.reg_read(0x0A) == 0x42
        # arg (1 << 16) | 0x0A = 0x1000A, varint 8a 80 04
        assert link._tx.frames == [bytes.fromhex("ffaa01 0000 5004 088a8004 5b41 fe")]


class TestAce2LinkRegWrite:
    def test_encodes_reg_and_val_into_arg(self):
        link = make_ace2_link(2)
        assert link.reg_write(0x0A, 0x99) is None
        # arg (2 << 16) | (0x0A << 8) | 0x99 = 0x20A99, varint 99 95 08
        assert link._tx.frames == [bytes.fromhex("ffaa01 0000 5104 08999508 c40e fe")]


class TestAce2LinkReaderPower:
    def test_reader_power_frame_encodes_cmd_0x52(self):
        link = make_ace2_link(1)
        link.reader_power(True)
        link.reader_power(False)
        # arg (1 << 16) | on: 0x10001 -> 81 80 04, 0x10000 -> 80 80 04
        assert link._tx.frames == [bytes.fromhex("ffaa01 0000 5204 08818004 ab60 fe"),
                                   bytes.fromhex("ffaa02 0000 5204 08808004 773a fe")]


class TestAce2LinkParseField1:
    def test_bad_preamble_raises(self):
        with pytest.raises(IOError, match="^bad ACE2 response preamble$"):
            make_ace2_link()._parse_field1(b"\x00\x00\x00\x00\x00\x00\x00")

    def test_empty_raises(self):
        with pytest.raises(IOError, match="^bad ACE2 response preamble$"):
            make_ace2_link()._parse_field1(b"")

    def test_field1_value_decoded(self):
        link = make_ace2_link()
        assert link._parse_field1(bytes.fromhex("ffaa01 0000 5002 0807 a1d2 fe")) == 7
        # a value past one byte keeps only its low byte (0x1FF -> 0xFF)
        assert link._parse_field1(bytes.fromhex("ffaa01 0000 5003 08ff03 b928 fe")) == 0xFF

    def test_non_field1_payload_returns_zero(self):
        link = make_ace2_link()
        assert link._parse_field1(bytes.fromhex("ffaa01 0000 5002 0a01 2784 fe")) == 0

    def test_short_payload_returns_zero(self):
        link = make_ace2_link()
        assert link._parse_field1(bytes.fromhex("ffaa01 0000 5000 d6d0 fe")) == 0
        # the field-1 tag alone, with no value byte after it
        assert link._parse_field1(bytes.fromhex("ffaa01 0000 5001 08 fb26 fe")) == 0


class TestAce2RegLinkRegRead:
    def test_masks_value_and_encodes_arg(self):
        unit = make_ace2_unit()
        unit._ace.set_reply("mfrc522_reg_read", {"val": 0x1FF})
        link = make_ace2_reg_link(1, ace2=unit)
        assert link.reg_read(0x0A) == 0xFF
        assert unit._ace.commands == [("mfrc522_reg_read", {"arg": 0x1000A})]
        assert unit._ace.requests[-1][3] == 2.0          # the per-register timeout

    def test_none_response_reads_zero(self):
        unit = make_ace2_unit()
        unit._ace.set_reply("mfrc522_reg_read", None)
        assert make_ace2_reg_link(0, ace2=unit).reg_read(0x05) == 0
        assert unit._ace.commands == [("mfrc522_reg_read", {"arg": 0x05})]

    def test_missing_serial_raises_ioerror(self):
        unit = make_ace2_unit(connection=None)
        with pytest.raises(IOError, match="^ACE2 serial not connected$"):
            make_ace2_reg_link(0, ace2=unit).reg_read(0x01)


class TestAce2RegLinkRegWrite:
    def test_encodes_arg(self):
        unit = make_ace2_unit()
        make_ace2_reg_link(2, ace2=unit).reg_write(0x0A, 0x99)
        assert unit._ace.commands == [("mfrc522_reg_write", {"arg": 0x20A99})]


class TestAce2RegLinkReaderPower:
    def test_reader_power_arg_encoding(self):
        unit = make_ace2_unit()
        link = make_ace2_reg_link(1, ace2=unit)          # reader index 1
        link.reader_power(True)
        link.reader_power(False)
        assert unit._ace.commands == [("mfrc522_reader_power", {"arg": 0x10001}),
                                      ("mfrc522_reader_power", {"arg": 0x10000})]


class TestAce2RegLinkSetRfidEnable:
    def test_set_rfid_enable_uses_physical_slot(self):
        unit = make_ace2_unit()
        make_ace2_reg_link(0, ace2=unit).set_rfid_enable(2, False)
        assert unit._ace.async_commands == [("set_rfid_enable", {"index": 2, "enable": False})]


class TestAFCACE2RFIDInit:
    def test_scanner_lanes_config_parsed(self):
        reader = make_ace2_rfid({"scanner_lanes": "lane1, , lane2", "scan_seconds": 45,
                                 "scan_interval": 0.5}, ace2=None, bind=False)
        assert reader._scanner_lanes == {"lane1", "lane2"}
        assert reader.scan_seconds == 45.0
        assert reader.scan_interval == 0.5

    def test_scan_seconds_defaults_to_30(self):
        reader = make_ace2_rfid({}, ace2=None, bind=False)
        assert reader.scan_seconds == 30.0

class TestAFCACE2RFIDOnReady:
    def test_binds_ace2_and_marks_scanner(self, monkeypatch):
        unit = make_ace2_unit(lanes=["scan1"])
        printer = unit.printer
        reader = make_ace2_rfid({"scanner_lanes": "scan1", "ace2_object": "AFC_ACE2 Ace2_1"},
                                ace2=unit, bind=False)
        monkeypatch.setattr(ace2_rfid_module, "resolve_rfid_keys",
                            lambda pr, b, c, d: (b"\xaa", c, d))
        assert reader.afc is None and reader.ace2 is None
        reader._on_ready()
        assert reader.afc is printer.afc
        assert reader.ace2 is unit                       # found by its configured name
        assert reader.bambu_master_key == b"\xaa"
        assert printer.afc.lanes["scan1"].spool_scanner is True
        assert printer.logger.messages == [("info", "ACE2 RFID bound to Ace2_1")]
        assert printer.reactor.pending == []             # identify waits for the connect

    def test_warns_when_ace2_missing(self, monkeypatch):
        reader = make_ace2_rfid({"scanner_lanes": "scan1"}, ace2=None, bind=False)
        printer = reader.printer
        del printer.objects["AFC"]                       # no AFC either: no lanes to mark
        monkeypatch.setattr(ace2_rfid_module, "resolve_rfid_keys",
                            lambda pr, b, c, d: (b, c, d))
        reader._on_ready()
        assert reader.afc is None
        assert reader.ace2 is None
        assert printer.logger.messages == [("warning", "ACE2 object not found; RFID disabled")]
        assert printer.reactor.pending == []

    def test_discovers_named_object_no_callback_when_not_skipping(self, monkeypatch):
        unit = make_ace2_unit(name="Ace2_9")
        printer = unit.printer
        reader = make_ace2_rfid({"skip_factory_autostage": False}, ace2=unit, bind=False)
        monkeypatch.setattr(ace2_rfid_module, "resolve_rfid_keys", None)
        reader._on_ready()
        assert reader.ace2 is unit                       # found as "AFC_ACE2 Ace2_9"
        assert reader.bambu_master_key is None
        assert printer.logger.messages == [("info", "ACE2 RFID bound to Ace2_9")]
        assert printer.reactor.pending == []             # skip off: no retry


class TestAFCACE2RFIDOnAceConnected:
    @pytest.mark.parametrize("reconnect", [False, True])
    def test_a_late_or_reset_unit_gets_identify_off_again(self, reconnect):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True})
        printer = unit.printer
        printer.reactor.pending.clear()
        unit._ace.commands.clear()
        reader._identify_disable_tries = 7
        printer.send_event("afc_ace:connected", unit, reconnect)
        assert reader._identify_disable_tries == 0
        printer.reactor.run_callbacks()
        assert unit._ace.commands == [("set_rfid_enable", {"index": s, "enable": False})
                                      for s in range(4)]
        assert reader._identify_retry_pending is False

    def test_waits_for_a_retry_already_queued(self):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True})
        printer = unit.printer
        printer.reactor.pending.clear()
        reader._identify_retry_pending = True
        reader._on_ace_connected(unit, False)
        assert printer.reactor.pending == []

    def test_ignores_another_unit_and_factory_autostage(self):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True})
        printer = unit.printer
        printer.reactor.pending.clear()
        reader._on_ace_connected(object(), False)
        assert printer.reactor.pending == []
        reader.skip_factory_autostage = False
        reader._on_ace_connected(unit, False)
        assert printer.reactor.pending == []


class TestAFCACE2RFIDRetryDisableIdentify:
    def test_retry_disable_identify_when_connected(self):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True})
        printer = unit.printer
        reader._retry_disable_identify(100.0)
        assert unit._ace.commands == [("set_rfid_enable", {"index": 0, "enable": False}),
                                      ("set_rfid_enable", {"index": 1, "enable": False}),
                                      ("set_rfid_enable", {"index": 2, "enable": False}),
                                      ("set_rfid_enable", {"index": 3, "enable": False})]
        assert printer.logger.messages == [
            ("info", "ACE2 RFID: factory identify disabled on slot 0"),
            ("info", "ACE2 RFID: factory identify disabled on slot 1"),
            ("info", "ACE2 RFID: factory identify disabled on slot 2"),
            ("info", "ACE2 RFID: factory identify disabled on slot 3")]
        assert reader._identify_disable_tries == 0
        assert printer.reactor.pending == []

    def test_retry_disable_identify_gives_up_when_never_ready(self):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True,
                                      "identify_disable_max_tries": 3,
                                      "identify_disable_retry": 0.05},
                                     link=False, settle=False)
        printer = unit.printer
        printer.reactor.pending.clear()
        reader._retry_disable_identify(100.0)
        assert reader._identify_disable_tries == 1
        assert [(e.callback, e.waketime) for e in printer.reactor.pending] == [
            (reader._retry_disable_identify, 100.05)]
        assert printer.logger.messages == []
        printer.reactor.advance(1.0)                     # the queued retries run
        assert reader._identify_disable_tries == 3
        assert printer.logger.messages == [
            ("warning", "ACE2 RFID: ACE serial not ready after 3 tries; factory identify not "
                        "set (autostage may run on insert)")]
        assert printer.reactor.pending == []

    def test_retry_disable_identify_handles_missing_ace2(self):
        reader = make_ace2_rfid({"skip_factory_autostage": True,
                                 "identify_disable_max_tries": 1}, ace2=None)
        reader._retry_disable_identify(100.0)
        assert reader._identify_disable_tries == 1
        assert reader.printer.logger.messages == [
            ("warning", "ACE2 RFID: ACE serial not ready after 1 tries; factory identify not "
                        "set (autostage may run on insert)")]
        assert reader.printer.reactor.pending == []

    def test_retry_waits_while_the_link_is_down(self):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True}, settle=False)
        printer = unit.printer
        printer.reactor.pending.clear()
        unit._ace.connected = False
        reader._retry_disable_identify(100.0)
        assert unit._ace.commands == []
        assert reader._identify_disable_tries == 1
        assert [(e.callback, e.waketime) for e in printer.reactor.pending] == [
            (reader._retry_disable_identify, 100.5)]
        assert unit.printer.logger.messages == []


class TestAFCACE2RFIDDisableFactoryIdentify:
    def test_disable_factory_identify_all_slots(self):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": False,
                                      "lane_slot_map": "lane1:0"})
        reader._disable_factory_identify()               # no slots: all of them
        assert unit._ace.commands == [("set_rfid_enable", {"index": 0, "enable": False}),
                                      ("set_rfid_enable", {"index": 1, "enable": False}),
                                      ("set_rfid_enable", {"index": 2, "enable": False}),
                                      ("set_rfid_enable", {"index": 3, "enable": False})]
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: factory identify disabled on slot 0"),
            ("info", "ACE2 RFID: factory identify disabled on slot 1"),
            ("info", "ACE2 RFID: factory identify disabled on slot 2"),
            ("info", "ACE2 RFID: factory identify disabled on slot 3")]

    def test_noop_without_ace2(self):
        reader, unit = ace2_rfid_rig()
        reader.ace2 = None
        reader._disable_factory_identify()
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_logs_when_enable_call_fails(self):
        reader, unit = ace2_rfid_rig(link=False)         # the link raises IOError
        reader._disable_factory_identify([0])
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: could not disable factory identify on slot 0 yet (serial not "
                     "ready?)")]

    def test_logs_success_per_slot(self):
        reader, unit = ace2_rfid_rig()
        reader._disable_factory_identify([1])
        assert unit._ace.commands == [("set_rfid_enable", {"index": 1, "enable": False})]
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: factory identify disabled on slot 1")]


class TestAFCACE2RFIDSlotForLane:
    def test_slot_for_lane_prefers_ace_map_over_stale_config(self):
        reader, unit = ace2_rfid_rig({"lane_slot_map": "lane3:1, ghost:3"},
                                     lanes=["lane1", "lane2", "lane3"])
        assert reader._slot_for_lane("lane3") == 2       # the ACE's own map wins
        assert reader._slot_for_lane("ghost") == 3       # config fills the rest
        assert reader._slot_for_lane("nowhere") is None


class TestAFCACE2RFIDMap:
    @staticmethod
    def _btt_image(rgb: Tuple[int, int, int]) -> bytes:
        """
        A 1024-byte MIFARE Classic image in BTT's BQ Tech layout (block N at
        byte N * 16), PET (CEP), 1.75 mm, 1000 g, 200-240 C hotend, 60 C bed.

        :param rgb: colour code bytes
        :return bytes: the image
        """
        image = bytearray(1024)

        def put(block: int, off: int, raw: bytes) -> None:
            image[block * 16 + off:block * 16 + off + len(raw)] = raw

        put(1, 0, (1000).to_bytes(2, "little"))          # tag_version
        put(1, 2, b"BQ Tech")
        put(2, 0, b"20240812_162600")
        put(4, 0, b"PET")
        put(5, 0, b"PET (CEP)")
        put(6, 0, b"IP243ZCXV67")
        put(8, 0, bytes(rgb))
        put(10, 0, (1750).to_bytes(2, "little"))         # diameter, um
        put(17, 0, (1000).to_bytes(2, "little"))         # weight, g
        put(18, 8, (60).to_bytes(2, "little"))           # bed max
        put(18, 10, (200).to_bytes(2, "little"))         # hotend min
        put(18, 12, (240).to_bytes(2, "little"))         # hotend max
        put(20, 0, (60).to_bytes(2, "little"))           # bed
        return bytes(image)

    def test_map_builds_multi_color_when_dual(self):
        reader = make_ace2_rfid(ace2=None, bind=False)
        tag = {"uid": "aa", "tag_type": "MifareClassic1k",
               "filament": {"type": "PLA", "manufacturer": "Bambu",
                            "color_argb": 0xFFE94B3C,
                            "colors_argb": [0xFFE94B3C, 0xFF112233]}}
        info = reader._map(tag)
        assert info["color_hex"] == "e94b3c"
        assert info["multi_color"] == ["e94b3c", "112233"]
        assert info["is_dual_color"] is True
        assert info["uid"] == "aa"

    def test_map_single_color_not_dual(self):
        reader = make_ace2_rfid(ace2=None, bind=False)
        tag = {"uid": "aa", "tag_type": "MifareClassic1k",
               "filament": {"type": "PLA", "manufacturer": "Bambu",
                            "color_argb": 0xFFE94B3C, "colors_argb": [0xFFE94B3C]}}
        info = reader._map(tag)
        assert info["multi_color"] == ["e94b3c"]
        assert info["is_dual_color"] is False

    def test_decode_btt_maps_to_slot_info_single_color(self):
        reader = make_ace2_rfid(ace2=None, bind=False)
        tag = {"uid": "aabbccdd", "tag_type": "MifareClassic1k",
               "filament": decode_btt(self._btt_image((0xC0, 0xFF, 0xEE)))}
        info = reader._map(tag)
        assert info["brand"] == "BQ Tech"
        assert info["material"] == "PET"
        assert info["color_hex"] == "c0ffee"
        assert info["multi_color"] == ["c0ffee"]
        assert info["is_dual_color"] is False
        assert info["uid"] == "aabbccdd"

class TestAFCACE2RFIDGetStatus:
    def test_shape_when_empty(self):
        reader = make_ace2_rfid({"lane_slot_map": "lane1:0"}, ace2=None, bind=False)
        assert reader.get_status() == {"lane_slot_map": {"lane1": 0}, "last_reads": {}}

    def test_last_reads_after_record(self):
        reader = make_ace2_rfid({"lane_slot_map": "lane1:0"}, ace2=None, bind=False)
        reader.record_tag_read("lane1", {"material": "PLA", "uid": "AABB"})
        status = reader.get_status()
        assert status["lane_slot_map"] == {"lane1": 0}
        assert list(status["last_reads"]) == ["lane1"]
        record = status["last_reads"]["lane1"]
        assert record["material"] == "PLA"
        assert record["uid"] == "AABB"
        assert record["decoded"] is True


class TestAFCACE2RFIDNewLink:
    def test_link_reaches_the_slots_reader(self):
        reader, unit = ace2_rfid_rig({"read_reg_timeout": 0.5})
        link = reader._new_link(1, power_index=0)
        assert (link.slot, link.power_index, link._reg_timeout) == (1, 0, 0.5)
        assert link._ace2 is unit
        link.reg_read(0x37)
        assert unit._ace.commands == [("mfrc522_reg_read", {"arg": 0x10037})]
        assert unit.printer.logger.messages == []

    def test_power_index_defaults_to_the_reader(self):
        reader, unit = ace2_rfid_rig()
        assert reader._new_link(3).power_index == 3


class TestAFCACE2RFIDProbeUid:
    @staticmethod
    def _field(monkeypatch, answer):
        class Field:
            def __init__(self, reader):
                pass

            def activate(self):
                if isinstance(answer, BaseException):
                    raise answer
                return answer
        monkeypatch.setattr(ace2_rfid_module, "MifareClassic", Field)

    def test_a_tag_in_the_field_gives_its_uid(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        self._field(monkeypatch, (bytes.fromhex("50f5533f"), 0x08))
        assert reader._probe_uid(reader._new_link(0)) == "50f5533f"

    def test_an_empty_field_gives_none(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        self._field(monkeypatch, (None, None))
        assert reader._probe_uid(reader._new_link(0)) is None

    def test_a_reader_error_gives_none(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        self._field(monkeypatch, IOError("serial wedged"))
        assert reader._probe_uid(reader._new_link(0)) is None


class TestAFCACE2RFIDReadSlot:
    @staticmethod
    def _sequence(pair: int, on_arg: int, off_arg: int,
                  restore: bool = True) -> List[Tuple[str, Any]]:
        """
        The commands a managed read sends: identify off on the pair, reader on,
        reader off, then identify back on when restore.

        :param pair: the reader pair's first slot
        :param on_arg: the reader-power arg that turns the reader on
        :param off_arg: the reader-power arg that turns it off
        :param restore: identify handed back afterwards
        :return list: (method, params) in wire order
        """
        sent = [("set_rfid_enable", {"index": pair, "enable": False}),
                ("set_rfid_enable", {"index": pair + 1, "enable": False}),
                ("mfrc522_reader_power", {"arg": on_arg}),
                ("mfrc522_reader_power", {"arg": off_arg})]
        if restore:
            sent += [("set_rfid_enable", {"index": pair, "enable": True}),
                     ("set_rfid_enable", {"index": pair + 1, "enable": True})]
        return sent

    def test_read_slot_power_sequence_order(self, monkeypatch):
        reader, unit = ace2_rfid_rig(ACE2_RFID_RESTORING)
        tags = Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        assert reader.read_slot(0) is ACE2_RFID_PLA
        assert unit._ace.commands == [
            ("set_rfid_enable", {"index": 0, "enable": False}),
            ("set_rfid_enable", {"index": 1, "enable": False}),
            ("mfrc522_reader_power", {"arg": 1}),
            ("mfrc522_reader_power", {"arg": 0}),
            ("set_rfid_enable", {"index": 0, "enable": True}),
            ("set_rfid_enable", {"index": 1, "enable": True})]
        assert tags.slots == [0]
        assert reader._slot_uid == {0: "aa"}
        assert unit.printer.logger.messages == []

    @pytest.mark.parametrize("phys_slot,reader_idx,on_arg,off_arg", [
        (0, 0, 0x00001, 0x00000), (1, 0, 0x00001, 0x00000),
        (2, 1, 0x10001, 0x10000), (3, 1, 0x10001, 0x10000)])
    def test_read_slot_reader_index_mapping(self, monkeypatch, phys_slot, reader_idx,
                                            on_arg, off_arg):
        reader, unit = ace2_rfid_rig(ACE2_RFID_RESTORING)
        tags = Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        reader.read_slot(phys_slot)
        # two readers cover four slots: reg r/w and power both use the pair index
        assert unit._ace.commands == self._sequence(reader_idx * 2, on_arg, off_arg)
        assert tags.slots == [reader_idx]
        assert unit.printer.logger.messages == []

    def test_read_slot_per_slot_reg_index_optin(self, monkeypatch):
        reader, unit = ace2_rfid_rig(dict(ACE2_RFID_RESTORING, reader_reg_per_slot=True))
        tags = Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        reader.read_slot(3)
        assert tags.slots == [3]                         # reg index is the slot itself
        assert unit._ace.commands == self._sequence(2, 0x10001, 0x10000)   # power per pair
        assert unit.printer.logger.messages == []

    def test_read_slot_reg_override_sets_chip_and_power(self, monkeypatch):
        reader, unit = ace2_rfid_rig(ACE2_RFID_RESTORING)
        tags = Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        reader.read_slot(2, reg_slot=3)                  # probe reader index 3 explicitly
        assert tags.slots == [3]
        assert unit._ace.commands == self._sequence(2, 0x30001, 0x30000)   # same reader
        assert unit.printer.logger.messages == []

    def test_read_slot_disables_identify_but_skips_restore_when_autostage_skipped(
            self, monkeypatch):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True})
        assert reader.probe_restore_identify is False    # keep identify off after the read
        Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        reader.read_slot(2)
        assert unit._ace.commands == self._sequence(2, 0x10001, 0x10000, restore=False)
        assert unit.printer.logger.messages == []

    def test_read_slot_restores_power_on_read_failure(self, monkeypatch):
        reader, unit = ace2_rfid_rig(ACE2_RFID_RESTORING)
        Ace2RfidTagScript(monkeypatch, RuntimeError("read blew up"))
        with pytest.raises(RuntimeError, match="^read blew up$"):
            reader.read_slot(0)
        # reader still powered off and identify restored on both slots
        assert unit._ace.commands == self._sequence(0, 0x1, 0x0)
        assert reader._slot_uid == {}
        assert unit.printer.logger.messages == []

    def test_read_slot_manage_power_false_skips_sequence(self, monkeypatch):
        reader, unit = ace2_rfid_rig(ACE2_RFID_RESTORING)
        tags = Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        assert reader.read_slot(0, manage_power=False) is ACE2_RFID_PLA
        assert unit._ace.commands == []
        assert tags.slots == [0]
        assert tags.calls[0]["is_excluded"] is None      # no sibling exclusion either
        assert reader._slot_uid == {}
        assert unit.printer.logger.messages == []

    def test_missing_ace2_raises(self):
        reader, unit = ace2_rfid_rig()
        reader.ace2 = None
        with pytest.raises(CommandError, match="^ACE2 not available$"):
            reader.read_slot(0)
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_sibling_match_warns(self, monkeypatch):
        reader, unit = ace2_rfid_rig({"lane_slot_map": "laneA:0, laneB:1"},
                                     lanes=[LaneSpec("laneA", prep=True),
                                            LaneSpec("laneB", prep=True)])
        reader._slot_uid[1] = "beef"                     # sibling slot 1 already read beef
        tags = Ace2RfidTagScript(monkeypatch, {"uid": "beef", "tag_type": "MifareClassic1k",
                                               "filament": {"type": "PLA"}})
        assert reader.read_slot(0)["uid"] == "beef"
        assert tags.calls[0]["is_excluded"]("beef") is True   # asked to halt it
        assert reader._slot_uid == {0: "beef", 1: "beef"}
        assert unit.printer.logger.messages == [
            ("warning", "ACE2 RFID: slot 0 read matches shared sibling slot 1's tag (uid=beef), "
                        "check the spool is in slot 0, not 1")]

    def test_power_off_exception_logged(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        unit._ace.set_reply("mfrc522_reader_power", lambda params: (
            {} if params["arg"] & 1 else RuntimeError("power off wedged")))
        Ace2RfidTagScript(monkeypatch, {"uid": "aa", "tag_type": "MifareClassic1k",
                                        "filament": None})
        assert reader.read_slot(0)["uid"] == "aa"
        logger = unit.printer.logger
        assert logger.messages == [("error", "ACE2 RFID: reader power-off failed")]
        assert logger.calls[0][2]["traceback"].splitlines()[-1] == (
            "RuntimeError: power off wedged")

    def test_reenable_exception_logged(self, monkeypatch):
        reader, unit = ace2_rfid_rig({"restore_identify": True,
                                      "skip_factory_autostage": False})
        unit._ace.async_hook = lambda method, params: (
            RuntimeError("enable wedged") if params.get("enable") else None)
        Ace2RfidTagScript(monkeypatch, {"uid": "aa", "tag_type": "MifareClassic1k",
                                        "filament": None})
        reader.read_slot(0)
        assert unit.printer.logger.messages == [
            ("error", "ACE2 RFID: re-enable identify failed"),
            ("error", "ACE2 RFID: re-enable identify failed")]
        assert unit._ace.commands == self._sequence(0, 0x1, 0x0)


class TestAFCACE2RFIDReadLane:
    def test_missing_slot_raises(self):
        reader, unit = ace2_rfid_rig()
        with pytest.raises(CommandError) as err:
            reader.read_lane("ghost")
        assert str(err.value) == "lane 'ghost' has no ACE2 reader slot (set lane_slot_map)"
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_no_tag_logs(self, monkeypatch):
        reader, unit = ace2_rfid_rig({"lane_slot_map": "laneA:0"})
        Ace2RfidTagScript(monkeypatch, None)
        assert reader.read_lane("laneA") is None
        assert unit.printer.logger.messages == [("info", "ACE2 RFID: no tag read on slot 0")]

    def test_applies_to_lane(self, monkeypatch):
        reader, unit = ace2_rfid_rig(lanes=[LaneSpec("laneA", prep=True)])
        tag = {"uid": "aa", "tag_type": "MifareClassic1k",
               "filament": {"type": "PLA", "manufacturer": "Bambu", "color_argb": 0xFF112233}}
        Ace2RfidTagScript(monkeypatch, tag)
        lane = unit.printer.afc.lanes["laneA"]
        assert reader.read_lane("laneA") is tag
        assert lane.material == "PLA"
        assert lane.color == "#112233"
        assert reader.last_reads_status()["laneA"]["uid"] == "aa"
        assert unit.printer.logger.messages == []
        assert unit.printer.gcode.messages == [
            ("info", "ACE2 RFID: read spool on laneA\n  Name: Bambu PLA\n  Brand: Bambu\n"
                     "  Material: PLA\n  Color: #112233\n  Diameter: 1.75mm\n  Tag UID: aa")]

    def test_without_afc_the_tag_is_returned_unapplied(self, monkeypatch):
        reader, unit = ace2_rfid_rig(lanes=[LaneSpec("laneA", prep=True)])
        Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        reader.afc = None
        assert reader.read_lane("laneA") is ACE2_RFID_PLA
        assert unit.printer.afc.lanes["laneA"].material is None
        assert reader.last_reads_status() == {}
        assert unit.printer.gcode.messages == []
        assert unit.printer.logger.messages == []


class TestAFCACE2RFIDScanSlot:
    SCAN = {"lane_slot_map": "scan_lane:0", "scan_interval": 0.05, "scanner_confirm_reads": 1}

    def test_scan_slot_reads_until_tag_then_powers_off(self, monkeypatch):
        reader, unit = ace2_rfid_rig(self.SCAN)
        found = {"uid": "beef", "filament": {"type": "PLA"}}
        tags = Ace2RfidTagScript(monkeypatch, None, None, found)
        assert reader._scan_slot(0, duration=5.0) is found
        assert tags.slots == [0, 0, 0]                   # two misses, then the decode
        assert unit._ace.commands == [
            ("set_rfid_enable", {"index": 0, "enable": False}),
            ("set_rfid_enable", {"index": 1, "enable": False}),
            ("mfrc522_reader_power", {"arg": 0}),        # barrier for the async disable
            ("mfrc522_reader_power", {"arg": 1}),        # powered on to read
            ("mfrc522_reader_power", {"arg": 0})]        # and off when done
        assert reader._slot_uid == {0: "beef"}
        assert unit.printer.gcode.messages == ace2_rfid_hold_prompt("slot 0")
        assert unit.printer.logger.messages == []

    def test_scan_prefers_full_decode_over_uid_only(self, monkeypatch):
        reader, unit = ace2_rfid_rig(self.SCAN)
        full = {"uid": "beef", "filament": {"type": "PLA", "manufacturer": "Snapmaker"}}
        tags = Ace2RfidTagScript(monkeypatch, {"uid": "beef", "filament": None}, full)
        assert reader._scan_slot(0, duration=5.0) is full   # not the UID-only read
        assert len(tags.slots) == 2
        assert unit.printer.gcode.messages == ace2_rfid_hold_prompt("slot 0")   # shown once
        assert unit.printer.logger.messages == []

    def test_scan_confirms_same_uid_over_consecutive_reads(self, monkeypatch):
        # With confirm=2 only the same UID decoded twice in a row wins; a
        # different UID in between resets the streak.
        reader, unit = ace2_rfid_rig(dict(self.SCAN, scanner_confirm_reads=2))
        good = {"uid": "beef", "filament": {"type": "PLA"}}
        stray = {"uid": "cafe", "filament": {"type": "ABS"}}
        tags = Ace2RfidTagScript(monkeypatch, stray, good, stray, good, good)
        assert reader._scan_slot(0, duration=10.0) is good
        assert len(tags.slots) == 5
        assert reader._slot_uid == {0: "beef"}
        assert unit.printer.gcode.messages == ace2_rfid_hold_prompt("slot 0")
        assert unit.printer.logger.messages == []

    def test_scan_returns_none_without_full_decode(self, monkeypatch):
        # Only ever a UID (the decode never completes): not trusted.
        reader, unit = ace2_rfid_rig(self.SCAN)
        Ace2RfidTagScript(monkeypatch, {"uid": "beef", "filament": None})
        assert reader._scan_slot(0, duration=2.0) is None
        assert reader._slot_uid == {}
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID scan: no full decode; saw UID(s) beef (uid-only, no decode)")]
        assert unit.printer.gcode.messages == ace2_rfid_hold_prompt("slot 0") + [
            ("raw", "// action:prompt_end")]             # the dialog is closed again

    def test_scan_reads_only_the_presented_tag(self, monkeypatch):
        # No pre-read of the neighbour slot: only the presented tag is cached.
        reader, unit = ace2_rfid_rig(self.SCAN)
        Ace2RfidTagScript(monkeypatch, {"uid": "new", "filament": {"type": "PLA"}})
        assert reader._scan_slot(0, duration=5.0)["uid"] == "new"
        assert reader._slot_uid == {0: "new"}
        assert unit.printer.gcode.messages == ace2_rfid_hold_prompt("slot 0")
        assert unit.printer.logger.messages == []

    def test_scanner_triggers_sister_retract(self, monkeypatch):
        # A sibling tag dominating the shared antenna is retracted out of the
        # way, then fed back when the scan ends.
        reader, unit = ace2_rfid_rig(
            {"lane_slot_map": "lane2:2, lane3:3", "skip_factory_autostage": False,
             "scan_interval": 0.05, "scanner_confirm_reads": 1},
            lanes=[LaneSpec("lane2", slot=2, prep=True),
                   LaneSpec("lane3", slot=3, prep=True, load=True)])
        motion = Ace2RfidMotion(unit)

        def sibling_only(seen: Optional[List[Any]] = None, **kwargs: Any) -> None:
            seen.append(("cafe", 0x08, True))            # halted, own tag absent
            return None

        Ace2RfidTagScript(monkeypatch, sibling_only)
        assert reader._scan_slot(2, duration=2.0) is None
        moves = [c for c in unit._ace.commands if c[0] in ("unwind_filament", "feed_filament")]
        assert moves == [
            ("unwind_filament", {"index": 3, "length": 75.0, "speed": 100.0, "mode": "normal"}),
            ("feed_filament", {"index": 3, "length": 75.0, "speed": 100.0})]
        assert motion.pos(3) == 0.0                      # sibling back where it was
        assert reader._sister_retracted is None
        assert reader._sister_hint_shown is False
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: retracted sister slot 3 by 75mm to clear its tag off the shared "
                     "antenna"),
            ("info", "ACE2 RFID scan: no full decode; saw UID(s) cafe"),
            ("info", "ACE2 RFID: restored sister slot 3 (+75mm) after stage read")]
        assert unit.printer.gcode.messages == []
        assert unit._ace.async_commands[-2:] == [
            ("set_rfid_enable", {"index": 2, "enable": True}),
            ("set_rfid_enable", {"index": 3, "enable": True})]


class TestAFCACE2RFIDScanLane:
    SCAN = {"lane_slot_map": "scan_lane:0", "scanner_lanes": "scan_lane",
            "scan_interval": 0.05, "scanner_confirm_reads": 1}
    TAG = {"uid": "abcd", "tag_type": "MifareClassic1k", "filament": {"type": "PLA"}}
    #: The summary of TAG with no Spoolman match.
    LINES = ["Name: PLA", "Material: PLA", "Diameter: 1.75mm"]

    def _rig(self, values: Optional[Dict[str, Any]] = None
             ) -> Tuple[AFC_ACE2_RFID, afcACE2]:
        """
        :param values: options over SCAN
        :return tuple: reader and unit, with scan_lane on slot 0 and Spoolman on
        """
        reader, unit = ace2_rfid_rig(dict(self.SCAN, **(values or {})),
                                     lanes=[LaneSpec("scan_lane", prep=True)])
        unit.printer.afc.spoolman = object()
        return reader, unit

    def test_scan_lane_stages_next_spool_id(self, monkeypatch):
        reader, unit = self._rig()
        afc = unit.printer.afc
        Ace2RfidTagScript(monkeypatch, self.TAG)
        sync = Recorder()
        monkeypatch.setattr(ace2_rfid_module, "sync_rfid_to_spoolman", sync)
        assert reader.scan_lane("scan_lane", 5) is self.TAG
        assert len(sync.calls) == 1
        args, kwargs = sync.calls[0]
        assert args[0] is afc
        assert args[1] is afc.lanes["scan_lane"]
        assert args[2]["uid"] == "abcd" and args[2]["material"] == "PLA"
        assert args[3] is unit.printer.logger
        assert args[4] == "ACE2 RFID scan"
        assert kwargs == {"allow_create": False, "set_next": True}   # staged, not applied
        assert afc.lanes["scan_lane"].material is None   # the scanner lane keeps nothing
        assert reader.last_reads_status()["scan_lane"]["uid"] == "abcd"
        assert unit.printer.logger.messages == []
        assert unit.printer.gcode.messages == (
            ace2_rfid_hold_prompt("scan_lane")
            + ace2_rfid_scan_popup(ACE2_RFID_SCAN_TITLE, self.LINES))

    def test_scan_emits_popup_notification(self, monkeypatch):
        reader, unit = self._rig()
        printer = unit.printer
        printer.afc.spool.next_spool_id = 120
        Ace2RfidTagScript(monkeypatch, self.TAG)
        monkeypatch.setattr(ace2_rfid_module, "sync_rfid_to_spoolman", Recorder())
        reader.scan_lane("scan_lane", 5)
        assert printer.gcode.messages == (
            ace2_rfid_hold_prompt("scan_lane")
            + ace2_rfid_scan_popup(ACE2_RFID_SCAN_TITLE, self.LINES + ["Spoolman ID: 120"]))
        printer.gcode.messages.clear()
        printer.reactor.advance(10.0)                    # the popup dismisses itself
        assert printer.gcode.messages == [("raw", "// action:prompt_end")]
        assert unit.printer.logger.messages == []

    def test_scan_lane_no_tag_no_sync(self, monkeypatch):
        reader, unit = self._rig()
        Ace2RfidTagScript(monkeypatch, None)
        sync = Recorder()
        monkeypatch.setattr(ace2_rfid_module, "sync_rfid_to_spoolman", sync)
        assert reader.scan_lane("scan_lane", 5) is None
        assert sync.calls == []                          # nothing staged
        assert reader.last_reads_status() == {}
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID scan: no tag on lane scan_lane in 5s")]
        assert unit.printer.gcode.messages == []

    def test_scan_lane_missing_slot_raises(self):
        reader, unit = ace2_rfid_rig({"scanner_lanes": "nolane"})
        with pytest.raises(CommandError) as err:
            reader.scan_lane("nolane", 5)
        assert str(err.value) == "lane 'nolane' has no ACE2 reader slot (set lane_slot_map)"
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_missing_ace2_raises(self):
        reader, unit = self._rig()
        reader.ace2 = None
        with pytest.raises(CommandError, match="^ACE2 not available$"):
            reader.scan_lane("scan_lane")
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_no_tag_logs_and_returns_none(self, monkeypatch):
        reader, unit = self._rig({"scan_seconds": 30.0})
        Ace2RfidTagScript(monkeypatch, None)
        assert reader.scan_lane("scan_lane") is None     # SECONDS defaults to scan_seconds
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID scan: no tag on lane scan_lane in 30s")]

    def test_spoolman_sync_exception_logged(self, monkeypatch):
        reader, unit = self._rig()
        unit.printer.afc.spool.next_spool_id = 1
        Ace2RfidTagScript(monkeypatch, self.TAG)
        monkeypatch.setattr(ace2_rfid_module, "get_auto_spoolman_create",
                            Recorder(raises=RuntimeError("x")))
        monkeypatch.setattr(ace2_rfid_module, "sync_rfid_to_spoolman",
                            Recorder(raises=RuntimeError("sync boom")))
        assert reader.scan_lane("scan_lane", 5) is self.TAG
        assert unit.printer.logger.messages == [
            ("warning", "ACE2 RFID scan Spoolman sync failed: sync boom")]
        assert unit.printer.gcode.messages == (
            ace2_rfid_hold_prompt("scan_lane")
            + ace2_rfid_scan_popup(ACE2_RFID_SCAN_TITLE, self.LINES + ["Spoolman ID: 1"]))


class TestAFCACE2RFIDSpoolDetails:
    #: What the tag alone gives, written out by hand.
    BASE = {"brand": "Bambu", "material": "PLA", "color": "112233", "diameter": 1.75,
            "ext": 220, "bed": 60, "weight": 1000, "name": ""}

    @staticmethod
    def _wire(reader: AFC_ACE2_RFID, monkeypatch: pytest.MonkeyPatch,
              get_spool: Recorder) -> Recorder:
        """
        Give AFC a moonraker and point the module's SpoolmanClient at get_spool.

        :param reader: the reader
        :param monkeypatch: the test's monkeypatch
        :param get_spool: answers SpoolmanClient(moonraker).get_spool(spool_id)
        :return Recorder: the SpoolmanClient constructor's calls
        """
        reader.afc.moonraker = object()
        client = Recorder(result=type("Client", (), {"get_spool": staticmethod(get_spool)}))
        monkeypatch.setattr(ace2_rfid_module, "SpoolmanClient", client)
        return client

    def test_base_when_no_spool_id(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        get_spool = Recorder(result={"filament": {"name": "never"}})
        self._wire(reader, monkeypatch, get_spool)
        assert reader._spool_details(None, ACE2_RFID_SLOT_INFO) == self.BASE
        assert get_spool.calls == []
        assert unit.printer.logger.messages == []

    def test_base_when_moonraker_missing(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        client = self._wire(reader, monkeypatch, Recorder(result={}))
        reader.afc.moonraker = None
        assert reader._spool_details(5, ACE2_RFID_SLOT_INFO) == self.BASE
        assert client.calls == []
        assert unit.printer.logger.messages == []

    def test_base_when_get_spool_raises(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        get_spool = Recorder(raises=RuntimeError("x"))
        self._wire(reader, monkeypatch, get_spool)
        assert reader._spool_details(5, ACE2_RFID_SLOT_INFO) == self.BASE
        assert get_spool.calls == [((5,), {})]
        assert unit.printer.logger.messages == []

    def test_base_when_spool_not_dict(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        get_spool = Recorder(result=None)
        self._wire(reader, monkeypatch, get_spool)
        assert reader._spool_details(5, ACE2_RFID_SLOT_INFO) == self.BASE
        assert get_spool.calls == [((5,), {})]
        assert unit.printer.logger.messages == []

    def test_enriched_from_spoolman(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        spool = {"remaining_weight": 750,
                 "filament": {"name": "Galaxy", "material": "PETG", "color_hex": "#ff0000",
                              "diameter": 1.75, "settings_extruder_temp": 240,
                              "settings_bed_temp": 70, "vendor": {"name": "Polymaker"}}}
        client = self._wire(reader, monkeypatch, Recorder(result=spool))
        assert reader._spool_details(5, ACE2_RFID_SLOT_INFO) == {
            "brand": "Polymaker", "material": "PETG", "color": "ff0000", "diameter": 1.75,
            "ext": 240, "bed": 70, "weight": 750, "name": "Galaxy"}   # remaining preferred
        assert client.calls == [((reader.afc.moonraker,), {})]
        assert unit.printer.logger.messages == []

    def test_weight_falls_back_when_no_remaining(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        spool = {"remaining_weight": None, "filament": {"weight": 500, "vendor": {}}}
        self._wire(reader, monkeypatch, Recorder(result=spool))
        assert reader._spool_details(5, ACE2_RFID_SLOT_INFO) == dict(self.BASE, weight=500)
        assert unit.printer.logger.messages == []


class TestAFCACE2RFIDNotifyScan:
    def test_full_details_popup(self):
        reader, unit = ace2_rfid_rig()
        printer = unit.printer
        reader._notify_scan(ACE2_RFID_SLOT_INFO, "lane1", 120)
        assert printer.gcode.messages == [
            ("info", "Spool Scanned on lane1:\n  Name: Bambu PLA\n  Brand: Bambu\n"
                     "  Material: PLA\n  Color: #112233\n  Diameter: 1.75mm\n"
                     "  Nozzle temp: 220°C\n  Bed temp: 60°C\n  Remaining: 1000g\n"
                     "  Spoolman ID: 120"),
            ("raw", "// action:prompt_begin Spool Scanned on lane1"),
            ("raw", "// action:prompt_text Name: Bambu PLA"),
            ("raw", "// action:prompt_text Brand: Bambu"),
            ("raw", "// action:prompt_text Material: PLA"),
            ("raw", "// action:prompt_text Color: #112233"),
            ("raw", "// action:prompt_text Diameter: 1.75mm"),
            ("raw", "// action:prompt_text Nozzle temp: 220°C"),
            ("raw", "// action:prompt_text Bed temp: 60°C"),
            ("raw", "// action:prompt_text Remaining: 1000g"),
            ("raw", "// action:prompt_text Spoolman ID: 120"),
            ("raw", "// action:prompt_footer_button OK|RESPOND TYPE=command "
                    "MSG=action:prompt_end|info"),
            ("raw", "// action:prompt_show")]
        assert [e.waketime for e in printer.reactor.pending] == [110.0]
        printer.gcode.messages.clear()
        printer.reactor.advance(10.0)                    # auto-dismiss
        assert printer.gcode.messages == [("raw", "// action:prompt_end")]
        assert printer.logger.messages == []

    def test_uid_fallback_when_no_fields(self):
        reader, unit = ace2_rfid_rig()
        reader._notify_scan({"uid": "deadbeef"}, "", None)
        assert unit.printer.gcode.messages == [
            ("info", "Spool Scanned:\n  uid: deadbeef"),
            ("raw", "// action:prompt_begin Spool Scanned"),
            ("raw", "// action:prompt_text uid: deadbeef"),
            ("raw", "// action:prompt_footer_button OK|RESPOND TYPE=command "
                    "MSG=action:prompt_end|info"),
            ("raw", "// action:prompt_show")]
        assert unit.printer.logger.messages == []

    def test_notification_error_logged(self):
        reader, unit = ace2_rfid_rig()
        reader._notify_scan(dict(ACE2_RFID_SLOT_INFO, weight_g="heavy"), "lane1", 5)
        assert unit.printer.logger.messages == [
            ("warning", "ACE2 RFID scan: notification error: could not convert string to "
                        "float: 'heavy'")]
        assert unit.printer.gcode.messages == []
        assert unit.printer.reactor.pending == []        # no dismiss queued either


class TestAFCACE2RFIDOnPostInsert:
    INSERT = dict(ACE2_RFID_RESTORING, read_on_insert=True, read_on_insert_attempts=1,
                  read_on_insert_delay=0.0)

    def _rig(self, values: Optional[Dict[str, Any]] = None
             ) -> Tuple[AFC_ACE2_RFID, afcACE2]:
        """
        :param values: options over INSERT
        :return tuple: reader and unit with lane1 (slot 0) and laneX (slot 1)
        """
        return ace2_rfid_rig(dict(self.INSERT, lane_slot_map="lane1:0", **(values or {})),
                             lanes=[LaneSpec("lane1", prep=True), LaneSpec("laneX", prep=True)])

    def test_auto_read_on_insert_reads_mapped_lane(self, monkeypatch):
        reader, unit = self._rig()
        printer = unit.printer
        tags = Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        reader._on_post_insert(printer.afc.lanes["lane1"])
        assert tags.slots == []                          # deferred onto the reactor
        assert len(printer.reactor.pending) == 1
        printer.reactor.run_callbacks()
        assert tags.slots == [0]
        assert printer.afc.lanes["lane1"].material == "PLA"
        assert printer.logger.messages == []
        assert printer.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n  Name: PLA\n  Material: PLA\n"
                     "  Diameter: 1.75mm\n  Tag UID: aa")]

    def test_auto_read_skips_unmapped_lane(self, monkeypatch):
        reader, unit = self._rig()
        reader._on_post_insert(unit.printer.afc.lanes["laneX"])
        assert unit.printer.reactor.pending == []
        assert unit.printer.logger.messages == []

    def test_auto_read_disabled_by_config(self):
        reader, unit = self._rig({"read_on_insert": False})
        reader._on_post_insert(unit.printer.afc.lanes["lane1"])
        assert unit.printer.reactor.pending == []
        assert unit.printer.logger.messages == []

    def test_disable_rfid_post_insert_no_read(self):
        reader, unit = self._rig({"disable_rfid": True})
        reader._on_post_insert(unit.printer.afc.lanes["lane1"])
        assert unit.printer.reactor.pending == []
        assert unit.printer.logger.messages == []

    def test_post_insert_skipped_when_stage_read(self):
        reader, unit = self._rig({"stage_read": True})   # the stage probe reads instead
        reader._on_post_insert(unit.printer.afc.lanes["lane1"])
        assert unit.printer.reactor.pending == []
        assert unit.printer.logger.messages == []

    def test_auto_read_retries_until_tag(self, monkeypatch):
        reader, unit = self._rig({"read_on_insert_attempts": 3})
        printer = unit.printer
        tags = Ace2RfidTagScript(monkeypatch, None, {"uid": "bb", "tag_type": "x",
                                                     "filament": None}, ACE2_RFID_PLA)
        reader._on_post_insert(printer.afc.lanes["lane1"])
        printer.reactor.run_callbacks()
        assert tags.slots == [0, 0]                      # stopped on the first tag
        assert printer.logger.messages == [("info", "ACE2 RFID: no tag read on slot 0")]
        assert printer.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n  Diameter: 1.75mm\n  Tag UID: bb")]


class TestAFCACE2RFIDAutoRead:
    def test_command_error_logged_and_returns(self, monkeypatch):
        reader, unit = ace2_rfid_rig({"read_on_insert_attempts": 3})
        tags = Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        reader._auto_read("ghost")
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID auto-read ghost: lane 'ghost' has no ACE2 reader slot (set "
                     "lane_slot_map)")]                  # once: no retry after a user error
        assert tags.slots == []

    def test_retries_then_reports_no_tag(self, monkeypatch):
        reader, unit = ace2_rfid_rig(dict(ACE2_RFID_RESTORING, lane_slot_map="lane1:0",
                                          read_on_insert_attempts=2,
                                          read_on_insert_delay=0.1))
        printer = unit.printer
        tags = Ace2RfidTagScript(monkeypatch, RuntimeError("glitch"))
        reader._auto_read("lane1")
        assert tags.slots == [0, 0]
        assert printer.logger.messages == [
            ("error", "ACE2 RFID auto-read failed for lane1"),
            ("error", "ACE2 RFID auto-read failed for lane1"),
            ("info", "ACE2 RFID auto-read lane1: no tag after 2 attempts")]
        assert printer.logger.calls[0][2]["traceback"].splitlines()[-1] == (
            "RuntimeError: glitch")
        # each read settles 0.2s; the 0.1s retry delay runs between the two only
        assert printer.reactor.now == pytest.approx(100.5)

    def test_no_delay_between_attempts_when_delay_is_zero(self, monkeypatch):
        reader, unit = ace2_rfid_rig(dict(ACE2_RFID_RESTORING, lane_slot_map="lane1:0",
                                          read_on_insert_attempts=2,
                                          read_on_insert_delay=0.0))
        Ace2RfidTagScript(monkeypatch, None)
        reader._auto_read("lane1")
        assert unit.printer.reactor.now == pytest.approx(100.4)
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: no tag read on slot 0"),
            ("info", "ACE2 RFID: no tag read on slot 0"),
            ("info", "ACE2 RFID auto-read lane1: no tag after 2 attempts")]

    def test_success_stops_without_logging(self, monkeypatch):
        reader, unit = ace2_rfid_rig({"lane_slot_map": "lane1:0", "read_on_insert_attempts": 3})
        tags = Ace2RfidTagScript(monkeypatch, ACE2_RFID_PLA)
        reader._auto_read("lane1")
        assert tags.slots == [0]
        assert unit.printer.logger.messages == []


class TestAFCACE2RFIDRfidEnabled:
    def test_rfid_enabled_uses_configured_map_as_allowlist(self):
        reader, unit = ace2_rfid_rig({"lane_slot_map": "lane1:1"}, lanes=["lane0", "lane1"])
        assert reader._rfid_enabled("lane1") is True
        assert reader._rfid_enabled("lane0") is False    # on the ACE, but not allowed

    def test_rfid_enabled_auto_from_ace_when_no_map(self):
        reader, unit = ace2_rfid_rig(lanes=["lane0", "lane1"])
        assert reader._rfid_enabled("lane0") is True
        assert reader._rfid_enabled("laneX") is False
        reader.ace2 = None                               # no unit, no map: nothing enabled
        assert reader._rfid_enabled("lane0") is False


class TestAFCACE2RFIDLaneAtSlot:
    def test_none_when_no_afc(self):
        reader, unit = ace2_rfid_rig(lanes=[LaneSpec("laneA", slot=2)])
        reader.afc = None
        assert reader._lane_at_slot(2) is None

    def test_finds_lane_via_ace_map(self):
        reader, unit = ace2_rfid_rig(lanes=[LaneSpec("laneA", slot=2)])
        assert reader._lane_at_slot(2) is unit.printer.afc.lanes["laneA"]

    def test_none_when_slot_unmapped(self):
        reader, unit = ace2_rfid_rig(lanes=[LaneSpec("laneA", slot=2)])
        assert reader._lane_at_slot(7) is None


class TestAFCACE2RFIDIsSiblingTag:
    @staticmethod
    def _reader(sibling_present: bool = True,
                session_uid: Optional[str] = None) -> AFC_ACE2_RFID:
        """
        Reader 1 serves slots 2 (lane2, active) and 3 (lane3, the sibling).

        :param sibling_present: lane3 holds a spool
        :param session_uid: the UID this session read on slot 3
        :return AFC_ACE2_RFID: the reader
        """
        reader, unit = ace2_rfid_rig(
            dict(ACE2_RFID_RESTORING, lane_slot_map="lane2:2, lane3:3"),
            lanes=[LaneSpec("lane2", slot=2), LaneSpec("lane3", slot=3, prep=sibling_present)])
        if session_uid is not None:
            reader._slot_uid[3] = session_uid
        return reader

    def test_is_sibling_tag_by_session_uid(self):
        assert self._reader(session_uid="beef")._is_sibling_tag(2, 3, "beef") is True

    def test_is_sibling_tag_false_without_a_session_read(self):
        assert self._reader()._is_sibling_tag(2, 3, "4cb2dea6") is False

    def test_is_sibling_tag_false_when_sibling_absent(self):
        # the spool left the sibling slot, so its old tag no longer collides
        reader = self._reader(sibling_present=False, session_uid="4cb2dea6")
        assert reader._is_sibling_tag(2, 3, "4cb2dea6") is False

    def test_is_sibling_tag_false_when_dedup_disabled(self):
        reader = self._reader(session_uid="4cb2dea6")
        reader.shared_reader_dedup = False
        assert reader._is_sibling_tag(2, 3, "4cb2dea6") is False

    def test_is_sibling_tag_false_on_empty_uid(self):
        reader = self._reader(session_uid="4cb2dea6")
        assert reader._is_sibling_tag(2, 3, "") is False
        assert reader._is_sibling_tag(2, 3, None) is False

    def test_is_sibling_tag_false_when_no_sibling(self):
        reader = self._reader(session_uid="4cb2dea6")
        assert reader._is_sibling_tag(2, None, "4cb2dea6") is False


class TestAFCACE2RFIDStageProbeBegin:
    STAGE = {"stage_read": True, "probe_settle": 0.0, "skip_factory_autostage": True}

    @staticmethod
    def _field(monkeypatch: pytest.MonkeyPatch, uids: List[Optional[str]]) -> None:
        """
        Script the tag that answers each stage-scan poll (MifareClassic.activate):
        one UID per poll, None or an empty queue for no tag in the field.

        :param monkeypatch: the test's monkeypatch
        :param uids: UID hex per poll
        """
        queue = list(uids)

        class FieldScript:
            def __init__(self, mfrc: Any) -> None:
                self.mfrc = mfrc

            def activate(self) -> Tuple[Optional[bytes], Optional[int]]:
                uid = queue.pop(0) if queue else None
                return (None, None) if uid is None else (bytes.fromhex(uid), 0x08)

        monkeypatch.setattr(ace2_rfid_module, "MifareClassic", FieldScript)

    @staticmethod
    def _decoded(uid: str = "beef") -> Dict[str, Any]:
        """
        :param uid: tag UID
        :return dict: a fully decoded PLA tag
        """
        return {"uid": uid, "tag_type": "MifareClassic1k", "filament": {"type": "PLA"}}

    #: The console hint to move lane3's spool by hand when it blocks lane2's read.
    HINT = ("info", "AFC ACE2: couldn't read lane2's RFID, lane lane3's spool is on the shared "
                    "reader blocking it. Manually move lane lane3's spool a little and re-stage "
                    "lane2, or set lane2's spool id by hand.")

    def _rig(self, values: Optional[Dict[str, Any]] = None,
             lane_values: Optional[Dict[str, Any]] = None, **unit_options: Any
             ) -> Tuple[AFC_ACE2_RFID, afcACE2, Ace2RfidMotion]:
        """
        :param values: options over STAGE
        :param lane_values: extra [AFC_lane lane1] options
        :param unit_options: other make_ace2_unit keywords
        :return tuple: reader, unit and its motion, with lane1 on slot 0
        """
        reader, unit = ace2_rfid_rig(
            dict(self.STAGE, lane_slot_map="lane1:0", **(values or {})),
            lanes=[LaneSpec("lane1", prep=True, values=dict(lane_values or {}))],
            **unit_options)
        return reader, unit, Ace2RfidMotion(unit)

    def _sister_rig(self, sibling: Optional[Dict[str, Any]] = None,
                    values: Optional[Dict[str, Any]] = None, **unit_options: Any
                    ) -> Tuple[AFC_ACE2_RFID, afcACE2, Ace2RfidMotion]:
        """
        Reader 1: lane2 on slot 2 is staged, lane3 on slot 3 is its sibling,
        movable by default (prepped, at the hub, not in the tool) and known
        this session by its parked tag "cafe".

        :param sibling: LaneSpec fields for lane3 over the movable default
        :param values: options over STAGE
        :param unit_options: other make_ace2_unit keywords
        :return tuple: reader, unit and its motion
        """
        spec = dict({"prep": True, "load": True}, **(sibling or {}))
        reader, unit = ace2_rfid_rig(
            dict(self.STAGE, lane_slot_map="lane2:2, lane3:3", **(values or {})),
            lanes=[LaneSpec("lane2", slot=2, prep=True), LaneSpec("lane3", slot=3, **spec)],
            **unit_options)
        reader._slot_uid = {3: "cafe"}
        return reader, unit, Ace2RfidMotion(unit)

    def test_stage_scan_reads_and_stops_on_detect(self, monkeypatch):
        reader, unit, motion = self._rig()
        lane = unit.printer.afc.lanes["lane1"]
        self._field(monkeypatch, ["beef"])               # tag seen on the first poll
        Ace2RfidTagScript(monkeypatch, self._decoded("beef"))
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(lane, ctx)
        # one 0.1s poll at 25 mm/s before the stop
        assert ctx == {"active": True, "done": True, "initial": 500.0,
                       "fed": pytest.approx(2.5)}
        assert reader._slot_uid == {0: "beef"}
        assert reader._probe is None                     # reader released for the feeder
        assert lane.material == "PLA"
        assert ace2_rfid_sent(unit) == [
            ("set_rfid_enable", {"index": 0, "enable": False}),
            ("set_rfid_enable", {"index": 1, "enable": False}),
            ("mfrc522_reader_power", {"arg": 0}),
            ("stop_feed_assist", {"index": 0}),
            ("stop_feed_assist", {"index": 1}),
            ("mfrc522_reader_power", {"arg": 1}),
            ("feed_filament", {"index": 0, "length": 560.0, "speed": 25.0}),   # 500 + dist_hub
            ("stop_feed_filament", {"index": 0}),        # stopped on detect
            ("mfrc522_reader_power", {"arg": 0})]
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1) + ACE2_RFID_TEARDOWN + [
            ("info", "ACE2 RFID: read lane1 during staging, uid=beef type=MifareClassic1k PLA")]
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane1", "beef")]

    def test_stage_begin_disables_identify_with_barrier(self):
        reader, unit, motion = self._rig({"stage_scan_dist": 0.0})   # no scan window
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert ctx == {"active": True, "done": False, "initial": 500.0, "fed": 0.0}
        assert ace2_rfid_sent(unit) == [
            ("set_rfid_enable", {"index": 0, "enable": False}),
            ("set_rfid_enable", {"index": 1, "enable": False}),
            ("mfrc522_reader_power", {"arg": 0}),        # synchronous barrier
            ("stop_feed_assist", {"index": 0}),
            ("stop_feed_assist", {"index": 1})]
        assert reader._probe["slot"] == 0 and reader._probe["sibling"] == 1
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1)

    def test_stage_begin_no_initial_when_factory_stages(self):
        reader, unit, motion = self._rig({"stage_scan_dist": 0.0,
                                          "skip_factory_autostage": False})
        ctx: Dict[str, Any] = {"active": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert ctx == {"active": True, "initial": 0.0, "fed": 0.0}   # factory does the load
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1)

    def test_disable_rfid_stage_begin_plain_feed_with_initial(self):
        # No reads, no reader touched, but the initial load still reaches the hub.
        reader, unit, motion = self._rig({"disable_rfid": True})
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert ctx == {"active": False, "done": False, "initial": 500.0}
        assert unit._ace.commands == []
        assert reader._probe is None
        assert unit.printer.logger.messages == []

    def test_disable_rfid_stage_begin_no_initial_when_factory_stages(self):
        reader, unit, motion = self._rig({"disable_rfid": True,
                                          "skip_factory_autostage": False})
        ctx: Dict[str, Any] = {"active": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert ctx == {"active": False, "initial": 0.0}
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_stage_scan_no_tag_releases_reader(self, monkeypatch):
        reader, unit, motion = self._rig()
        self._field(monkeypatch, [])                     # never a tag
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert ctx == {"active": True, "done": False, "initial": 500.0, "fed": 560.0}
        assert ace2_rfid_sent(unit) == [
            ("set_rfid_enable", {"index": 0, "enable": False}),
            ("set_rfid_enable", {"index": 1, "enable": False}),
            ("mfrc522_reader_power", {"arg": 0}),
            ("stop_feed_assist", {"index": 0}),
            ("stop_feed_assist", {"index": 1}),
            ("mfrc522_reader_power", {"arg": 1}),
            ("feed_filament", {"index": 0, "length": 560.0, "speed": 25.0}),
            ("mfrc522_reader_power", {"arg": 0})]        # released, nothing to stop
        assert motion.pos(0) == 560.0                    # the whole window was fed
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1) + [
            ("info", "ACE2 RFID: stage scan slot 0, no tag seen in 560mm; staging without RFID")]
        assert unit.printer.gcode.messages == []

    def test_stage_scan_no_tag_applies_afc_defaults(self, monkeypatch):
        # The case seen on printer 1: an untagged spool staged blank.
        reader, unit, motion = self._rig()
        unit.printer.afc.default_material_type = "PLA"
        unit.printer.afc.lanes["lane1"].material = ""
        self._field(monkeypatch, [])                     # never a tag
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert unit.printer.afc.lanes["lane1"].material == "PLA"
        assert unit.printer.logger.messages[-1] == (
            "info", "ACE2 RFID: lane1 has no readable tag; AFC defaults applied "
                    "(material PLA, color -)")

    def test_stage_scan_suppresses_and_restores_feed_assist(self, monkeypatch):
        reader, unit, motion = self._rig(feed_assist_active=[0, 1])
        lane = unit.printer.afc.lanes["lane1"]
        self._field(monkeypatch, [])
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(lane, ctx)
        assert unit._assist_suppressed == {0, 1}         # stopped and held off on both
        assert unit._feed_assist_active == set()
        assert reader._assist_slots == (0, 1)
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1) + [
            ("info", "ACE2 RFID: stage scan slot 0, no tag seen in 560mm; staging without RFID")]
        assert unit.printer.gcode.messages == []
        unit.printer.logger.clear()
        reader._stage_probe_end(lane, ctx)               # end of staging restores it
        assert unit._assist_suppressed == set()
        assert reader._assist_slots is None
        assert unit.printer.logger.messages == ACE2_RFID_TEARDOWN + [
            ("info", "ACE2 RFID: feed assist restored on slot 0"),
            ("info", "ACE2 RFID: feed assist restored on slot 1")]

    def test_stage_scan_dedups_sibling_then_reads_own(self, monkeypatch):
        reader, unit = ace2_rfid_rig(
            dict(self.STAGE, lane_slot_map="lane3:2, lane4:3"),
            lanes=[LaneSpec("lane3", slot=2, prep=True), LaneSpec("lane4", slot=3, prep=True)])
        Ace2RfidMotion(unit)
        reader._slot_uid = {3: "cafe"}                   # the sibling slot read "cafe"
        self._field(monkeypatch, ["cafe", "beef"])       # sibling first, then our tag
        tags = Ace2RfidTagScript(monkeypatch, self._decoded("beef"))
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane3"], ctx)
        assert ctx["done"] is True
        assert reader._slot_uid == {2: "beef", 3: "cafe"}   # ours, not the sibling's
        assert tags.calls[0]["is_excluded"]("cafe") is True
        assert unit.printer.logger.messages == ace2_rfid_assist_off(2, 3) + [
            ("info", "ACE2 RFID: stage scan slot 2, tag cafe parked on the shared reader "
                     "(sibling slot 3); clearing it before the read")] + ACE2_RFID_TEARDOWN + [
            ("info", "ACE2 RFID: read lane3 during staging, uid=beef type=MifareClassic1k PLA")]
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane3", "beef")]

    def test_stage_scan_aborts_on_removal(self, monkeypatch):
        reader, unit, motion = self._rig()
        motion.empty_slots = {0}                         # spool pulled mid-scan
        self._field(monkeypatch, [])
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        # 0.1s poll plus two 0.15s confirmation polls at 25 mm/s
        assert ctx == {"active": True, "done": False, "initial": 500.0, "removed": True,
                       "fed": pytest.approx(10.0)}
        assert ace2_rfid_sent(unit)[-2:] == [("stop_feed_filament", {"index": 0}),
                                             ("mfrc522_reader_power", {"arg": 0})]
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1)

    def test_stage_scan_recenters_when_parked_read_misses(self, monkeypatch):
        reader, unit, motion = self._rig()
        self._field(monkeypatch, ["beef"])
        # the three reads at rest miss; one after a re-center retract succeeds
        Ace2RfidTagScript(monkeypatch, None, None, None, self._decoded("beef"))
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert ctx["done"] is True
        assert ctx["fed"] == 0.0                         # 2.5mm fed less the 3mm retract
        assert ace2_rfid_sent(unit)[-3:] == [
            ("stop_feed_filament", {"index": 0}),
            ("unwind_filament", {"index": 0, "length": 3.0, "speed": 100.0, "mode": "normal"}),
            ("mfrc522_reader_power", {"arg": 0})]
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1) + [
            ("info", "ACE2 RFID: re-centered tag after 3mm retract")] + ACE2_RFID_TEARDOWN + [
            ("info", "ACE2 RFID: read lane1 during staging, uid=beef type=MifareClassic1k PLA")]
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane1", "beef")]

    def test_stage_scan_uid_only_not_accepted(self, monkeypatch):
        reader, unit, motion = self._rig({"stage_recenter_max": 6.0})
        self._field(monkeypatch, ["aa"])
        Ace2RfidTagScript(monkeypatch, {"uid": "aa", "tag_type": "MifareClassic1k",
                                        "filament": None})
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert ctx == {"active": True, "done": False, "initial": 500.0, "fed": 0.0}
        assert reader._slot_uid == {}
        unwinds = [c for c in unit._ace.commands if c[0] == "unwind_filament"]
        assert len(unwinds) == 2                         # two 3mm steps up to the 6mm cap
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1) + [
            ("info", "ACE2 RFID: stage scan slot 0 detected uid=aa but the tag did not decode, "
                     "staging without RFID")]

    def test_stage_probe_skips_unconfigured_lane(self):
        reader, unit = ace2_rfid_rig(dict(self.STAGE, lane_slot_map="lane1:0"),
                                     lanes=["lane1", "laneX"])
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["laneX"], ctx)
        assert ctx == {"active": False, "done": False}
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_stage_probe_disabled_leaves_ctx_inactive(self):
        reader, unit = ace2_rfid_rig(dict(self.STAGE, lane_slot_map="lane1:0",
                                          stage_read=False), lanes=["lane1"])
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert ctx == {"active": False, "done": False}
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_sister_retract_clears_then_reads_and_restores(self, monkeypatch):
        reader, unit, motion = self._sister_rig()        # movable sibling
        lane2 = unit.printer.afc.lanes["lane2"]
        self._field(monkeypatch, ["cafe", "beef"])       # sibling dominates, then our tag
        Ace2RfidTagScript(monkeypatch, self._decoded("beef"))
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(lane2, ctx)
        assert ctx["done"] is True
        assert reader._slot_uid == {2: "beef", 3: "cafe"}
        assert reader._sister_retracted == (3, 75.0, 100.0)
        assert motion.pos(3) == -75.0                    # sibling wound off the antenna
        assert unit.printer.logger.messages == ace2_rfid_assist_off(2, 3) + [
            ("info", "ACE2 RFID: stage scan slot 2, tag cafe parked on the shared reader "
                     "(sibling slot 3); clearing it before the read"),
            ("info", "ACE2 RFID: retracted sister slot 3 by 75mm to clear its tag off the "
                     "shared antenna")] + ACE2_RFID_TEARDOWN + [
            ("info", "ACE2 RFID: read lane2 during staging, uid=beef type=MifareClassic1k PLA")]
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane2", "beef")]
        unit.printer.logger.clear()
        reader._stage_probe_end(lane2, ctx)              # the sibling is fed back
        assert reader._sister_retracted is None
        assert motion.pos(3) == 0.0
        assert [c for c in unit._ace.commands if c[0] == "feed_filament"][-1] == (
            "feed_filament", {"index": 3, "length": 75.0, "speed": 100.0})
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: restored sister slot 3 (+75mm) after stage read"),
            ("info", "ACE2 RFID: feed assist restored on slot 2"),
            ("info", "ACE2 RFID: feed assist restored on slot 3")]

    def test_sister_retract_only_moves_once_per_stage(self, monkeypatch):
        reader, unit, motion = self._sister_rig()
        self._field(monkeypatch, ["cafe", "cafe", "cafe"])
        Ace2RfidTagScript(monkeypatch, None)
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane2"], ctx)
        unwinds = [c for c in unit._ace.commands if c[0] == "unwind_filament"]
        assert unwinds == [("unwind_filament", {"index": 3, "length": 75.0, "speed": 100.0,
                                                "mode": "normal"})]   # once per stage
        assert unit.printer.logger.messages == ace2_rfid_assist_off(2, 3) + [
            ("info", "ACE2 RFID: stage scan slot 2, tag cafe parked on the shared reader "
                     "(sibling slot 3); clearing it before the read"),
            ("info", "ACE2 RFID: retracted sister slot 3 by 75mm to clear its tag off the "
                     "shared antenna"),
            ("info", "ACE2 RFID: stage scan slot 2 saw sibling slot 3's tag (uid=cafe), "
                     "clearing it and continuing"),
            ("info", "ACE2 RFID: stage scan slot 2 saw sibling slot 3's tag (uid=cafe), "
                     "clearing it and continuing"),
            ("info", "ACE2 RFID: stage scan slot 2, no tag seen in 560mm; staging without RFID")]
        assert unit.printer.gcode.messages == []         # it moved, so no hint

    def _check_sister_left_alone(self, monkeypatch: pytest.MonkeyPatch,
                                 sibling: Dict[str, Any], **unit_options: Any) -> None:
        """
        Stage lane2 with only the sibling's parked tag in the field and check
        the sibling is not moved and the user is told once to move it.

        :param monkeypatch: the test's monkeypatch
        :param sibling: LaneSpec fields for lane3 over the movable default
        :param unit_options: other make_ace2_unit keywords
        """
        reader, unit, motion = self._sister_rig(sibling, **unit_options)
        self._field(monkeypatch, ["cafe", "cafe"])
        Ace2RfidTagScript(monkeypatch, None)
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane2"], ctx)
        assert [c for c in unit._ace.commands if c[0] == "unwind_filament"] == []
        assert motion.pos(3) == 0.0
        assert reader._sister_retracted is None
        assert reader._sister_hint_shown is True
        assert unit.printer.gcode.messages == [self.HINT]
        assert unit.printer.logger.messages == ace2_rfid_assist_off(2, 3) + [
            ("info", "ACE2 RFID: stage scan slot 2, tag cafe parked on the shared reader "
                     "(sibling slot 3); clearing it before the read"),
            ("info", "ACE2 RFID: stage scan slot 2, no tag seen in 560mm; staging without RFID")]

    def test_sister_not_moved_when_tool_loaded_but_user_told(self, monkeypatch):
        self._check_sister_left_alone(monkeypatch, {"tool_loaded": True})

    def test_sister_not_moved_when_printing_but_user_told(self, monkeypatch):
        self._check_sister_left_alone(monkeypatch, {}, printing=True)

    def test_sister_not_moved_when_not_hub_staged(self, monkeypatch):
        self._check_sister_left_alone(monkeypatch, {"load": False})

    def test_auto_tag_adjust_false_never_moves_but_tells_user(self, monkeypatch):
        reader, unit, motion = self._sister_rig(values={"auto_tag_adjust": False})
        self._field(monkeypatch, ["cafe", "cafe", "cafe"])
        Ace2RfidTagScript(monkeypatch, None)
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane2"], ctx)
        assert [c for c in unit._ace.commands if c[0] == "unwind_filament"] == []
        assert unit.printer.gcode.messages == [self.HINT]   # once
        assert unit.printer.logger.messages == ace2_rfid_assist_off(2, 3) + [
            ("info", "ACE2 RFID: stage scan slot 2, tag cafe parked on the shared reader "
                     "(sibling slot 3); clearing it before the read"),
            ("info", "ACE2 RFID: stage scan slot 2, no tag seen in 560mm; staging without RFID")]

    def test_stage_scan_never_assigns_unmovable_sibling_parked_tag(self, monkeypatch):
        # A sibling's parked tag is never read as this lane's, even unknown this
        # session and unmovable: the pre-spin baseline excludes it.
        reader, unit, motion = self._sister_rig({"tool_loaded": True})
        reader._slot_uid = {}                            # sibling UID unknown
        self._field(monkeypatch, ["cafe", "cafe", "cafe", "cafe"])   # only the sibling
        tags = Ace2RfidTagScript(monkeypatch, self._decoded("cafe"))
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane2"], ctx)
        assert ctx["done"] is False                      # sibling tag not assigned
        assert reader._slot_uid == {}
        assert tags.slots == []                          # never even read at rest
        assert unit.printer.gcode.messages == [self.HINT]
        assert unit.printer.logger.messages == ace2_rfid_assist_off(2, 3) + [
            ("info", "ACE2 RFID: stage scan slot 2, tag cafe parked on the shared reader "
                     "(sibling slot 3); clearing it before the read"),
            ("info", "ACE2 RFID: stage scan slot 2, no tag seen in 560mm; staging without RFID")]

    def test_stage_scan_reads_own_tag_past_unmovable_sibling(self, monkeypatch):
        reader, unit, motion = self._sister_rig({"tool_loaded": True})
        reader._slot_uid = {}
        self._field(monkeypatch, ["cafe", "cafe", "beef"])   # sibling, then our own
        tags = Ace2RfidTagScript(monkeypatch, self._decoded("beef"))
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane2"], ctx)
        assert ctx["done"] is True
        assert reader._slot_uid == {2: "beef"}
        assert tags.calls[0]["is_excluded"]("cafe") is True   # the baseline is excluded
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane2", "beef")]   # no hint
        assert unit.printer.logger.messages == ace2_rfid_assist_off(2, 3) + [
            ("info", "ACE2 RFID: stage scan slot 2, tag cafe parked on the shared reader "
                     "(sibling slot 3); clearing it before the read")] + ACE2_RFID_TEARDOWN + [
            ("info", "ACE2 RFID: read lane2 during staging, uid=beef type=MifareClassic1k PLA")]

    def test_stage_scan_without_afc_applies_to_the_staged_lane(self, monkeypatch):
        reader, unit, motion = self._rig()
        lane = unit.printer.afc.lanes["lane1"]
        reader.afc = None                                # no AFC to look the lane up in
        self._field(monkeypatch, ["beef"])
        Ace2RfidTagScript(monkeypatch, self._decoded("beef"))
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(lane, ctx)
        assert ctx["done"] is True
        assert lane.material == "PLA"
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane1", "beef")]
        assert unit.printer.logger.messages == ace2_rfid_assist_off(0, 1) + ACE2_RFID_TEARDOWN + [
            ("info", "ACE2 RFID: read lane1 during staging, uid=beef type=MifareClassic1k PLA")]

    def test_a_feed_assist_that_will_not_stop_is_logged(self):
        reader, unit, motion = self._rig({"stage_scan_dist": 0.0})
        unit._ace.set_reply("stop_feed_assist", lambda params: (
            RuntimeError("assist wedged") if params["index"] == 1 else {}))
        ctx: Dict[str, Any] = {"active": False, "done": False}
        reader._stage_probe_begin(unit.printer.afc.lanes["lane1"], ctx)
        assert unit._assist_suppressed == {0, 1}         # held off on both all the same
        assert reader._assist_slots == (0, 1)
        logger = unit.printer.logger
        assert logger.messages == [("info", "ACE2 RFID: feed assist stopped on slot 0"),
                                   ("error", "ACE2 RFID: failed to stop feed assist on slot 1")]
        assert logger.calls[1][2]["traceback"].splitlines()[-1] == (
            "RuntimeError: assist wedged")

class TestAFCACE2RFIDIsPrinting:
    def test_true_when_printing(self):
        reader, unit = ace2_rfid_rig(printing=True)
        assert reader._is_printing() is True
        unit.printer.afc.function.printing = False
        assert reader._is_printing() is False

    def test_false_on_exception(self):
        reader, unit = ace2_rfid_rig(printing=True)
        unit.printer.afc.function.raise_on_is_printing = RuntimeError("no print state")
        assert reader._is_printing() is False
        reader.afc = None                                # no AFC at all
        assert reader._is_printing() is False


class TestAFCACE2RFIDMaybeRetractSister:
    @staticmethod
    def _check_blocked(reader: AFC_ACE2_RFID, unit: afcACE2, sibling: Optional[int]) -> None:
        """
        :param reader: the reader
        :param unit: its unit
        :param sibling: the sibling slot asked for
        """
        assert reader._maybe_retract_sister(sibling) == "blocked"
        assert ace2_rfid_moves(unit) == []
        assert reader._sister_retracted is None
        assert unit.printer.logger.messages == []

    def test_none_sibling_blocked(self):
        reader, unit = ace2_rfid_sister_rig()
        self._check_blocked(reader, unit, None)

    def test_already_retracted_returns_retracted(self):
        reader, unit = ace2_rfid_sister_rig()
        reader._sister_retracted = (3, 75.0, 80.0)
        assert reader._maybe_retract_sister(3) == "retracted"
        assert ace2_rfid_moves(unit) == []               # not moved a second time
        assert reader._sister_retracted == (3, 75.0, 80.0)
        assert unit.printer.logger.messages == []

    def test_feature_off_blocked(self):
        reader, unit = ace2_rfid_sister_rig({"auto_tag_adjust": False})
        self._check_blocked(reader, unit, 3)

    def test_no_unit_blocked(self, monkeypatch):
        reader, unit = ace2_rfid_sister_rig()
        is_printing = Recorder(result=False)
        monkeypatch.setattr(unit.printer.afc.function, "is_printing", is_printing)
        reader.ace2 = None                               # feature on, but no ACE to move
        self._check_blocked(reader, unit, 3)
        assert is_printing.calls == []                   # stopped before the print check

    def test_no_link_blocked(self):
        reader, unit = ace2_rfid_sister_rig()
        conn = unit._ace
        unit._ace = None                                 # the link dropped
        assert reader._maybe_retract_sister(3) == "blocked"
        assert [c for c in conn.commands if c[0] in ("unwind_filament", "feed_filament")] == []
        assert reader._sister_retracted is None
        assert unit.printer.logger.messages == []

    def test_no_lane_at_slot_blocked(self):
        reader, unit = ace2_rfid_sister_rig()
        self._check_blocked(reader, unit, 1)             # no lane on slot 1

    def test_tool_loaded_blocked(self):
        reader, unit = ace2_rfid_sister_rig(sibling={"tool_loaded": True})
        self._check_blocked(reader, unit, 3)

    def test_not_hub_staged_blocked(self):
        reader, unit = ace2_rfid_sister_rig(sibling={"load": False})
        self._check_blocked(reader, unit, 3)

    def test_printing_blocked(self):
        reader, unit = ace2_rfid_sister_rig(printing=True)
        self._check_blocked(reader, unit, 3)

    def test_success_retracts_and_logs(self):
        reader, unit = ace2_rfid_sister_rig({"auto_tag_adjust_dist": 60.0})
        unit.feed_speed = 80.0
        assert reader._maybe_retract_sister(3) == "retracted"
        assert reader._sister_retracted == (3, 60.0, 80.0)
        assert ace2_rfid_moves(unit) == [
            ("unwind_filament", {"index": 3, "length": 60.0, "speed": 80.0, "mode": "normal"})]
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: retracted sister slot 3 by 60mm to clear its tag off the "
                     "shared antenna")]

    def test_unwind_error_blocked_and_logged(self):
        reader, unit = ace2_rfid_sister_rig(motion=False)
        unit._ace.set_reply("unwind_filament", RuntimeError("unwind wedged"))
        assert reader._maybe_retract_sister(3) == "blocked"
        assert reader._sister_retracted is None
        logger = unit.printer.logger
        assert logger.messages == [("error", "ACE2 RFID: sister retract on slot 3 failed")]
        assert logger.calls[0][2]["traceback"].splitlines()[-1] == (
            "RuntimeError: unwind wedged")


class TestAFCACE2RFIDRestoreSister:
    def test_noop_without_retract(self):
        reader, unit = ace2_rfid_sister_rig()
        reader._sister_retracted = None
        reader._restore_sister()
        assert reader._sister_retracted is None
        assert ace2_rfid_moves(unit) == []
        assert unit.printer.logger.messages == []

    def test_refeeds_and_logs(self):
        reader, unit = ace2_rfid_sister_rig()
        reader._sister_retracted = (3, 60.0, 80.0)
        reader._restore_sister()
        assert reader._sister_retracted is None
        assert ace2_rfid_moves(unit) == [
            ("feed_filament", {"index": 3, "length": 60.0, "speed": 80.0})]
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: restored sister slot 3 (+60mm) after stage read")]


class TestAFCACE2RFIDHandleSisterDomination:
    HINT = ("info", "AFC ACE2: the tag on lane lane3 is sitting on the shared reader and "
                    "blocking this read. Give that spool about a quarter turn by hand to move "
                    "its tag off the reader, then it should read.")

    def test_blocked_shows_hint_once(self):
        reader, unit = ace2_rfid_sister_rig({"auto_tag_adjust": False})
        reader._handle_sister_domination(3)
        assert reader._sister_hint_shown is True
        assert unit.printer.gcode.messages == [self.HINT]
        reader._handle_sister_domination(3)              # already shown: no repeat
        assert unit.printer.gcode.messages == [self.HINT]
        assert ace2_rfid_moves(unit) == []
        assert unit.printer.logger.messages == []

    def test_retracted_shows_no_hint(self):
        reader, unit = ace2_rfid_sister_rig()
        reader._handle_sister_domination(3)              # movable: retracted instead
        assert reader._sister_hint_shown is False
        assert reader._sister_retracted == (3, 75.0, 100.0)
        assert unit.printer.gcode.messages == []
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID: retracted sister slot 3 by 75mm to clear its tag off the "
                     "shared antenna")]


class TestAFCACE2RFIDReaderPowerOff:
    def test_powers_the_reader_off(self):
        reader, unit = ace2_rfid_rig()
        reader._reader_power_off(make_ace2_reg_link(1, ace2=unit, power_index=1))
        assert unit._ace.commands == [("mfrc522_reader_power", {"arg": 0x10000})]
        assert unit.printer.logger.messages == []

    def test_exception_logged(self):
        reader, unit = ace2_rfid_rig()
        unit._ace.set_reply("mfrc522_reader_power", RuntimeError("power off wedged"))
        reader._reader_power_off(make_ace2_reg_link(1, ace2=unit, power_index=1))
        logger = unit.printer.logger
        assert logger.messages == [
            ("error", "ACE2 RFID: stage read reader power-off failed")]
        assert logger.calls[0][2]["traceback"].splitlines()[-1] == (
            "RuntimeError: power off wedged")


class TestAFCACE2RFIDSafeProbeTeardown:
    @staticmethod
    def _probe(reader: AFC_ACE2_RFID, unit: afcACE2, pair: int = 0) -> None:
        """
        Hand the reader of a slot pair to the probe, as a stage read does.

        :param reader: the reader
        :param unit: its unit
        :param pair: the reader pair (0 serves slots 0-1, 1 serves slots 2-3)
        """
        link = make_ace2_reg_link(pair, ace2=unit, power_index=pair)
        reader._probe = {"link": link, "shared": (pair * 2, pair * 2 + 1)}

    def test_stage_teardown_leaves_identify_off_when_configured(self):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True})   # settles 0.2s
        assert reader.probe_restore_identify is False
        self._probe(reader, unit)
        reader._safe_probe_teardown()
        assert reader._probe is None
        assert unit._ace.commands == [("mfrc522_reader_power", {"arg": 0})]
        assert unit.printer.reactor.now == pytest.approx(100.2)
        assert unit.printer.logger.messages == ACE2_RFID_TEARDOWN

    def test_stage_teardown_restores_identify_when_configured(self):
        reader, unit = ace2_rfid_rig({"skip_factory_autostage": True,
                                      "restore_identify": True})
        self._probe(reader, unit)
        reader._safe_probe_teardown()
        assert unit._ace.commands == [("mfrc522_reader_power", {"arg": 0}),
                                      ("set_rfid_enable", {"index": 0, "enable": True}),
                                      ("set_rfid_enable", {"index": 1, "enable": True})]
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID teardown: reader_power(off)..."),
            ("info", "ACE2 RFID teardown: reader_power(off) done"),
            ("info", "ACE2 RFID teardown: set_rfid_enable(0,on)..."),
            ("info", "ACE2 RFID teardown: set_rfid_enable(0,on) done"),
            ("info", "ACE2 RFID teardown: set_rfid_enable(1,on)..."),
            ("info", "ACE2 RFID teardown: set_rfid_enable(1,on) done")]

    def test_noop_without_probe(self):
        reader, unit = ace2_rfid_rig()
        reader._probe = None
        reader._safe_probe_teardown()
        assert unit._ace.commands == []
        assert unit.printer.reactor.now == 100.0         # no settle either
        assert unit.printer.logger.messages == []

    def test_power_off_and_restore_identify(self):
        reader, unit = ace2_rfid_rig({"probe_settle": 0.0, "restore_identify": True})
        self._probe(reader, unit, pair=1)
        reader._safe_probe_teardown()
        assert reader._probe is None
        assert unit.printer.reactor.now == 100.0
        assert unit._ace.commands == [("mfrc522_reader_power", {"arg": 0x10000}),
                                      ("set_rfid_enable", {"index": 2, "enable": True}),
                                      ("set_rfid_enable", {"index": 3, "enable": True})]
        assert unit.printer.logger.messages == [
            ("info", "ACE2 RFID teardown: reader_power(off)..."),
            ("info", "ACE2 RFID teardown: reader_power(off) done"),
            ("info", "ACE2 RFID teardown: set_rfid_enable(2,on)..."),
            ("info", "ACE2 RFID teardown: set_rfid_enable(2,on) done"),
            ("info", "ACE2 RFID teardown: set_rfid_enable(3,on)..."),
            ("info", "ACE2 RFID teardown: set_rfid_enable(3,on) done")]

    def test_power_off_error_and_identify_not_restored(self):
        reader, unit = ace2_rfid_rig({"probe_settle": 0.0, "restore_identify": False,
                                      "skip_factory_autostage": True})
        unit._ace.set_reply("mfrc522_reader_power", RuntimeError("wedge"))
        self._probe(reader, unit)
        reader._safe_probe_teardown()
        assert reader._probe is None
        logger = unit.printer.logger
        assert logger.messages == [
            ("info", "ACE2 RFID teardown: reader_power(off)..."),
            ("error", "ACE2 RFID: probe power-off failed"),
            ("info", "ACE2 RFID teardown: identify NOT restored (config)")]
        assert logger.calls[1][2]["traceback"].splitlines()[-1] == "RuntimeError: wedge"
        assert unit._ace.commands == [("mfrc522_reader_power", {"arg": 0})]


class TestAFCACE2RFIDCmdACERFIDRead:
    def test_cmd_read_error_does_not_propagate(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        Ace2RfidTagScript(monkeypatch, RuntimeError("serial timed out"))
        gcmd = make_gcmd(SLOT=0)
        reader.cmd_ACE_RFID_READ(gcmd)                   # a reader glitch is not raised
        assert gcmd.messages == [("info", "ACE2 RFID: read error: serial timed out")]
        logger = unit.printer.logger
        assert logger.messages == [("error", "ACE2 RFID read failed")]
        assert logger.calls[0][2]["traceback"].splitlines()[-1] == (
            "RuntimeError: serial timed out")

    def test_cmd_read_reports_no_tag(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        Ace2RfidTagScript(monkeypatch, None)
        gcmd = make_gcmd(SLOT=0)
        reader.cmd_ACE_RFID_READ(gcmd)
        assert gcmd.messages == [("info", "ACE2 RFID: no tag found")]
        assert unit.printer.logger.messages == []

    def test_success_response_with_color(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        tags = Ace2RfidTagScript(monkeypatch, {
            "uid": "deadbeef", "tag_type": "MifareClassic1k",
            "filament": {"manufacturer": "Bambu", "type": "PLA", "color_argb": 0xFF112233}})
        gcmd = make_gcmd(SLOT=2, REG=3)                  # REG= picks the reader index
        reader.cmd_ACE_RFID_READ(gcmd)
        assert tags.slots == [3]
        assert gcmd.messages == [
            ("info", "ACE2 RFID: uid=deadbeef type=MifareClassic1k brand=Bambu material=PLA "
                     "color=112233")]
        assert unit.printer.logger.messages == []

    def test_success_response_without_color(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        tags = Ace2RfidTagScript(monkeypatch, {
            "uid": "aa", "tag_type": "MifareClassic1k",
            "filament": {"manufacturer": "", "type": "", "color_argb": None}})
        gcmd = make_gcmd(SLOT=2)                         # no REG: slot 2's own reader
        reader.cmd_ACE_RFID_READ(gcmd)
        assert tags.slots == [1]
        assert gcmd.messages == [
            ("info", "ACE2 RFID: uid=aa type=MifareClassic1k brand= material= color=")]
        assert unit.printer.logger.messages == []

    def test_lane_reads_through_its_slot(self, monkeypatch):
        reader, unit = ace2_rfid_rig({"lane_slot_map": "laneA:3"})
        tags = Ace2RfidTagScript(monkeypatch, None)
        gcmd = make_gcmd(LANE="laneA", SLOT=0)           # LANE wins over SLOT
        reader.cmd_ACE_RFID_READ(gcmd)
        assert tags.slots == [1]
        assert gcmd.messages == [("info", "ACE2 RFID: no tag found")]
        assert unit.printer.logger.messages == [("info", "ACE2 RFID: no tag read on slot 3")]


class TestAFCACE2RFIDCmdACERFIDBlocks:
    def test_bad_blocks_string(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        tags = Ace2RfidTagScript(monkeypatch, None)
        gcmd = make_gcmd(SLOT=0, BLOCKS="x,y")
        reader.cmd_ACE_RFID_BLOCKS(gcmd)
        assert gcmd.messages == [("info", "ACE2 RFID DUMP: bad BLOCKS='x,y'")]
        assert tags.slots == []                          # nothing read
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_lane_without_slot_raises(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        tags = Ace2RfidTagScript(monkeypatch, None)
        gcmd = make_gcmd(LANE="nope", BLOCKS="5,16")
        with pytest.raises(CommandError) as err:
            reader.cmd_ACE_RFID_BLOCKS(gcmd)
        assert str(err.value) == "lane 'nope' has no ACE2 reader slot (set lane_slot_map)"
        assert tags.slots == []
        assert gcmd.messages == []
        assert unit.printer.logger.messages == []

    def test_no_tag_found(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        tags = Ace2RfidTagScript(monkeypatch, None)
        gcmd = make_gcmd(SLOT=0, BLOCKS="5,16")
        reader.cmd_ACE_RFID_BLOCKS(gcmd)
        assert tags.calls[0]["dump_blocks"] == (5, 16)
        assert gcmd.messages == [("info", "ACE2 RFID DUMP: no tag found")]
        assert unit.printer.logger.messages == []

    def test_read_error_swallowed(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        Ace2RfidTagScript(monkeypatch, RuntimeError("wedge"))
        gcmd = make_gcmd(SLOT=0, BLOCKS="5,16")
        reader.cmd_ACE_RFID_BLOCKS(gcmd)
        assert gcmd.messages == [("info", "ACE2 RFID DUMP: error: wedge")]
        logger = unit.printer.logger
        assert logger.messages == [("error", "ACE2 RFID dump failed")]
        assert logger.calls[0][2]["traceback"].splitlines()[-1] == "RuntimeError: wedge"

    def test_dump_reports_dual_color(self, monkeypatch):
        reader, unit = ace2_rfid_rig()
        tags = Ace2RfidTagScript(monkeypatch, {
            "uid": "aabb", "tag_type": "MifareClassic1k",
            # block 16: format 0x0201, count 2, second colour as A, B, G, R
            "raw_blocks": {5: "112233ff", 16: "01020200ff665544"},
            "filament": {"colors_argb": [0xFF112233, 0xFF445566]}})
        gcmd = make_gcmd(SLOT=1)                         # BLOCKS defaults to 5,16
        reader.cmd_ACE_RFID_BLOCKS(gcmd)
        assert tags.calls[0]["dump_blocks"] == (5, 16)
        assert gcmd.messages == [("info", "\n".join([
            "ACE2 RFID DUMP: uid=aabb type=MifareClassic1k",
            "  block5 primary -> #112233 (a=ff)",
            "  block16 fmt=0201 color_count=2 second(ABGR bytes)=ff665544 -> #445566",
            "  VERDICT: DUAL-COLOR (count=2)",
            "  raw block 5: 112233ff",
            "  raw block 16: 01020200ff665544",
            "  decoded colors: #112233, #445566"]))]
        assert unit.printer.logger.messages == []

    def test_dump_reports_single_color_on_a_lane(self, monkeypatch):
        reader, unit = ace2_rfid_rig({"lane_slot_map": "laneA:2"})
        tags = Ace2RfidTagScript(monkeypatch, {
            "uid": "cc", "tag_type": "MifareClassic1k",
            "raw_blocks": {5: "0a0b0c", 16: "0000010000000000"}, "filament": None})
        gcmd = make_gcmd(LANE="laneA", BLOCKS="5, 16,,4")
        reader.cmd_ACE_RFID_BLOCKS(gcmd)
        assert tags.slots == [1]                         # slot 2's reader
        assert tags.calls[0]["dump_blocks"] == (5, 16, 4)
        # block 5 too short for a colour; block 4 was never read
        assert gcmd.messages == [("info", "\n".join([
            "ACE2 RFID DUMP: uid=cc type=MifareClassic1k",
            "  block16 fmt=0000 color_count=1 second(ABGR bytes)=00000000 -> #000000",
            "  VERDICT: single color (count=1)",
            "  raw block 5: 0a0b0c",
            "  raw block 16: 0000010000000000",
            "  raw block 4: (not read)"]))]
        assert unit.printer.logger.messages == []


class TestAFCACE2RFIDRescanLane:
    #: The console line every rescan that gets moving starts with.
    START = ("info", "ACE2 RFID: winding lane1 back 500mm and feeding it past the reader...")
    #: The answer when no tag came past the reader.
    NO_TAG = ("info", "ACE2 RFID: no tag read on lane1 (none came past the reader); the lane is "
                      "back where it was")

    @staticmethod
    def _moves(unit: afcACE2) -> List[Tuple[str, float, float]]:
        """
        :param unit: the unit
        :return list: (method, length, speed) of each feed and unwind sent
        """
        return [(m, p["length"], p["speed"]) for m, p in ace2_rfid_moves(unit)]

    # The pass places the tag to within one 0.05s poll at 25 mm/s (1.25mm).
    @pytest.mark.parametrize("tag_at,seen", [(30.0, 31), (210.0, 210), (470.0, 469)])
    def test_reads_applies_and_ends_exactly_where_it_started(self, monkeypatch, tag_at, seen):
        reader, unit, motion = ace2_rfid_rescan_rig()
        lane = unit.printer.afc.lanes["lane1"]
        tags = ace2_rfid_rescan_tag(monkeypatch, motion, tag_at)
        parked: List[float] = []
        tags.results = [lambda **kwargs: parked.append(motion.pos(0)) or ACE2_RFID_PLA]
        gcmd = make_gcmd(LANE="lane1")
        reader.rescan_lane("lane1", gcmd)
        back = 500.0 - tag_at
        assert parked == [pytest.approx(-back, abs=6.0)]  # read at rest on the tag
        assert motion.pos(0) == 0.0                      # back where it started
        moves = self._moves(unit)
        assert moves[:2] == [("unwind_filament", 500.0, 100.0),
                             ("feed_filament", 500.0, 25.0)]   # the pass runs to the end
        assert moves[2:] == [("unwind_filament", pytest.approx(back, abs=1.0), 100.0),
                             ("feed_filament", moves[2][1], 100.0)]
        assert "stop_feed_filament" not in [c[0] for c in unit._ace.commands]
        assert reader._slot_uid == {0: "aa"}
        assert lane.material == "PLA"
        assert unit._operation_active is False
        assert reader._assist_slots is None
        assert unit._assist_suppressed == set()
        assert gcmd.messages == [self.START]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs(
            f"tag went past ~{seen}mm into the 500mm feed; winding back {500 - seen}mm to it")
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane1", "aa")]

    def test_the_scan_feed_runs_at_the_staging_scan_speed(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig(values={"stage_scan_speed": 40.0})
        ace2_rfid_rescan_tag(monkeypatch, motion, None)
        gcmd = make_gcmd(LANE="lane1")
        reader.rescan_lane("lane1", gcmd)
        assert self._moves(unit) == [("unwind_filament", 500.0, 100.0),
                                     ("feed_filament", 500.0, 40.0)]
        assert gcmd.messages == [self.START, self.NO_TAG]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs("no tag went past in 500mm")
        assert unit.printer.gcode.messages == []

    def test_a_slow_starting_ace_does_not_shift_the_lane(self, monkeypatch):
        # The start-up delay once made each rescan leave the lane ~180 mm
        # short, so repeated scans walked it out of the slot.
        reader, unit, motion = ace2_rfid_rescan_rig()
        motion.start_delay = 2.0
        tags = ace2_rfid_rescan_tag(monkeypatch, motion, 260.0)
        for run in range(3):
            unit.printer.logger.clear()
            gcmd = make_gcmd(LANE="lane1")
            reader.rescan_lane("lane1", gcmd)
            assert motion.pos(0) == 0.0
            assert gcmd.messages == [self.START]
            assert len(tags.calls) == run + 1            # read at rest each time
            assert unit.printer.logger.messages == ace2_rfid_rescan_logs(
                "tag went past ~259mm into the 500mm feed; winding back 241mm to it")
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane1", "beef")] * 3

    def test_a_busy_blip_before_the_move_is_not_the_move(self, monkeypatch):
        # The ACE can read busy for a moment while it finishes the previous
        # command, then idle, before the move itself starts.
        reader, unit, motion = ace2_rfid_rescan_rig()
        motion.start_blips = [True, False, False]
        tags = ace2_rfid_rescan_tag(monkeypatch, motion, 210.0)
        gcmd = make_gcmd(LANE="lane1")
        reader.rescan_lane("lane1", gcmd)
        assert motion.pos(0) == 0.0
        assert len(tags.calls) == 1
        assert unit.printer.afc.lanes["lane1"].material == "PLA"
        assert gcmd.messages == [self.START]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs(
            "tag went past ~209mm into the 500mm feed; winding back 291mm to it")
        assert unit.printer.gcode.messages == [ace2_rfid_read_out("lane1", "beef")]

    def test_the_siblings_known_tag_is_passed_over(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        tags = ace2_rfid_rescan_tag(monkeypatch, motion, 210.0, uid="cafe")
        reader._slot_uid[1] = "cafe"                     # lane2's tag, still in slot 1
        gcmd = make_gcmd(LANE="lane1")
        reader.rescan_lane("lane1", gcmd)
        assert tags.calls == []
        assert unit.printer.afc.lanes["lane1"].material is None
        assert reader._slot_uid == {1: "cafe"}
        assert motion.pos(0) == 0.0
        assert gcmd.messages == [self.START, self.NO_TAG]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs("no tag went past in 500mm")

    def test_no_tag_is_reported_and_the_lane_put_back(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, None)
        gcmd = make_gcmd(LANE="lane1")
        reader.rescan_lane("lane1", gcmd)
        assert unit.printer.afc.lanes["lane1"].material is None
        assert motion.pos(0) == 0.0
        assert self._moves(unit) == [("unwind_filament", 500.0, 100.0),
                                     ("feed_filament", 500.0, 25.0)]
        assert gcmd.messages == [self.START, self.NO_TAG]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs("no tag went past in 500mm")
        assert unit.printer.gcode.messages == []

    def test_a_tag_seen_going_past_that_will_not_read_is_reported(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        tags = ace2_rfid_rescan_tag(monkeypatch, motion, 210.0)
        tags.results = [None]                            # it never decodes at rest
        gcmd = make_gcmd(LANE="lane1")
        reader.rescan_lane("lane1", gcmd)
        assert motion.pos(0) == 0.0
        assert gcmd.messages == [self.START, (
            "info", "ACE2 RFID: no tag read on lane1; the lane is back where it was")]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs(
            "tag went past ~210mm into the 500mm feed; winding back 290mm to it",
            "the tag was seen going past but did not read at rest")

    def test_the_wind_back_never_empties_the_slot(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig({"values": {"dist_hub": 100}})
        ace2_rfid_rescan_tag(monkeypatch, motion, None)
        gcmd = make_gcmd(LANE="lane1")
        reader.rescan_lane("lane1", gcmd)
        # 100mm staged less the 50mm margin
        assert self._moves(unit) == [("unwind_filament", 50.0, 100.0),
                                     ("feed_filament", 50.0, 25.0)]
        assert gcmd.messages == [
            ("info", "ACE2 RFID: winding lane1 back 50mm and feeding it past the reader..."),
            self.NO_TAG]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs("no tag went past in 50mm")
        assert unit.printer.gcode.messages == []

    def test_a_refused_move_stops_with_the_position_unknown(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0)
        unit._ace.set_reply("feed_filament", RuntimeError("busy"))
        gcmd = make_gcmd(LANE="lane1")
        with pytest.raises(CommandError) as err:
            reader.rescan_lane("lane1", gcmd)
        assert str(err.value) == "ACE2 RFID: the 500mm feed was refused: busy"
        # nothing more is moved once the position is not known
        assert self._moves(unit) == [("unwind_filament", 500.0, 100.0),
                                     ("feed_filament", 500.0, 25.0)]
        assert motion.pos(0) == -500.0
        assert unit._operation_active is False
        assert reader._assist_slots is None
        # the reader is still released and identify handed back
        assert ace2_rfid_sent(unit)[-3:] == [("mfrc522_reader_power", {"arg": 0}),
                                             ("set_rfid_enable", {"index": 0, "enable": True}),
                                             ("set_rfid_enable", {"index": 1, "enable": True})]
        assert gcmd.messages == [self.START]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs()
        assert unit.printer.gcode.messages == []

    def test_a_move_that_never_starts_stops_the_rescan(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0)
        motion.start_delay = 1000.0
        gcmd = make_gcmd(LANE="lane1")
        with pytest.raises(CommandError) as err:
            reader.rescan_lane("lane1", gcmd)
        assert str(err.value) == (
            "ACE2 RFID: the 500mm wind back on slot 0 was not seen to run to completion, so "
            "the lane's position is unknown; check it before loading")
        assert self._moves(unit) == [("unwind_filament", 500.0, 100.0)]
        assert unit._operation_active is False
        assert gcmd.messages == [self.START]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs()
        assert unit.printer.gcode.messages == []

    def test_a_unit_without_an_assist_hold_set_still_rescans(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, None)
        unit._assist_suppressed = None                   # nothing to hold assist off with
        gcmd = make_gcmd(LANE="lane1")
        reader.rescan_lane("lane1", gcmd)
        assert unit._assist_suppressed is None
        assert reader._assist_slots is None
        assert motion.pos(0) == 0.0
        assert gcmd.messages == [self.START, self.NO_TAG]
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs(
            "no tag went past in 500mm")
        assert unit.printer.gcode.messages == []

    @pytest.mark.parametrize("lane1,unit_options,msg", [
        ({"tool_loaded": True}, {},
         "lane1 is loaded to the toolhead; unload it before its spool is turned past the "
         "reader"),
        ({"load": False}, {},
         "lane1 is not staged, so there is no filament to turn the spool with"),
        ({}, {"printing": True},
         "turning the spool past the reader moves filament; not while printing"),
    ])
    def test_refusals_move_nothing(self, monkeypatch, lane1, unit_options, msg):
        reader, unit, motion = ace2_rfid_rescan_rig(lane1, **unit_options)
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0)
        gcmd = make_gcmd(LANE="lane1")
        with pytest.raises(CommandError) as err:
            reader.rescan_lane("lane1", gcmd)
        assert str(err.value) == msg
        assert unit._ace.commands == []
        assert gcmd.messages == []
        assert unit.printer.logger.messages == []

    def test_refused_while_the_ace_is_busy(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0)
        unit._operation_active = True
        gcmd = make_gcmd(LANE="lane1")
        with pytest.raises(CommandError) as err:
            reader.rescan_lane("lane1", gcmd)
        assert str(err.value) == "the ACE is loading or unloading; try again when it is done"
        assert unit._ace.commands == []
        assert unit._operation_active is True
        assert gcmd.messages == []
        assert unit.printer.logger.messages == []


class TestAFCACE2RFIDStageWriteAround:
    """A tag write with LANE= turns the spool the way ACE_RFID_RESCAN does and
    writes where the tag comes to rest. A blank tag answers but has nothing to
    decode, so at rest an answer is enough."""

    #: The log of a write whose tag went past 210mm into the feed.
    HELD = ace2_rfid_rescan_logs(
        "tag went past ~210mm into the 500mm feed; winding back 290mm to it",
        "holding tag beef at the reader for the write")

    def test_a_blank_tag_is_written_at_rest_and_the_lane_put_back(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0, filament=None)
        seen: List[Tuple[float, bool, Any]] = []

        def body(excluded: Any) -> None:
            seen.append((motion.pos(0), unit._operation_active, excluded))

        reader._stage_write_around("lane1", 0, body)
        assert len(seen) == 1
        pos, busy, excluded = seen[0]
        assert pos == pytest.approx(-290.0, abs=6.0)     # written with the tag at rest
        assert busy is True                              # heartbeat kept off meanwhile
        assert motion.pos(0) == 0.0                      # back where it started
        assert unit.printer.afc.lanes["lane1"].material is None   # the write applies it
        assert reader._slot_uid == {}
        assert unit._operation_active is False
        assert reader._assist_slots is None
        assert unit.printer.logger.messages == self.HELD
        assert unit.printer.gcode.messages == []

    def test_the_siblings_known_tag_is_excluded_from_the_write(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0, filament=None)
        reader._slot_uid[1] = "cafe"
        got: List[Any] = []
        reader._stage_write_around("lane1", 0, got.append)
        assert len(got) == 1
        assert got[0]("cafe") is True
        assert got[0]("beef") is False
        assert unit.printer.logger.messages == self.HELD

    def test_the_wrong_slots_reader_is_refused(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0, filament=None)
        body = Recorder()
        with pytest.raises(StageError) as err:
            reader._stage_write_around("lane1", 1, body)
        assert str(err.value) == "lane1 is on slot 0; write it with READER=ace2:slot0"
        assert body.calls == []
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    @pytest.mark.parametrize("lane1,unit_options,msg", [
        ({"tool_loaded": True}, {},
         "lane1 is loaded to the toolhead; unload it before its spool is turned past the "
         "reader"),
        ({"load": False}, {},
         "lane1 is not staged, so there is no filament to turn the spool with"),
        ({}, {"printing": True},
         "turning the spool past the reader moves filament; not while printing"),
    ])
    def test_refusals_move_nothing(self, monkeypatch, lane1, unit_options, msg):
        reader, unit, motion = ace2_rfid_rescan_rig(lane1, **unit_options)
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0, filament=None)
        with pytest.raises(StageError) as err:
            reader._stage_write_around("lane1", 0, Recorder())
        assert str(err.value) == msg
        assert unit._ace.commands == []
        assert unit.printer.logger.messages == []

    def test_no_tag_writes_nothing_and_puts_the_lane_back(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, None)
        body = Recorder()
        with pytest.raises(StageError) as err:
            reader._stage_write_around("lane1", 0, body)
        assert str(err.value) == (
            "no tag on lane1 came past the reader; the lane is back where it was")
        assert body.calls == []
        assert motion.pos(0) == 0.0
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs(
            "no tag went past in 500mm")

    def test_a_tag_that_never_comes_to_rest_writes_nothing(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        tags = ace2_rfid_rescan_tag(monkeypatch, motion, 210.0)
        tags.results = [None]                            # seen going past, silent at rest
        body = Recorder()
        with pytest.raises(StageError) as err:
            reader._stage_write_around("lane1", 0, body)
        assert str(err.value) == (
            "no tag on lane1 came to rest at the reader; the lane is back where it was")
        assert body.calls == []
        assert motion.pos(0) == 0.0
        assert unit.printer.logger.messages == ace2_rfid_rescan_logs(
            "tag went past ~210mm into the 500mm feed; winding back 290mm to it",
            "the tag was seen going past but did not read at rest")

    def test_a_failed_write_still_puts_the_lane_back(self, monkeypatch):
        reader, unit, motion = ace2_rfid_rescan_rig()
        ace2_rfid_rescan_tag(monkeypatch, motion, 210.0, filament=None)
        with pytest.raises(RuntimeError, match="^serial went away$"):
            reader._stage_write_around("lane1", 0,
                                       Recorder(raises=RuntimeError("serial went away")))
        assert motion.pos(0) == 0.0
        assert unit._operation_active is False
        assert reader._assist_slots is None
        assert unit.printer.logger.messages == self.HELD


class TestAFCACE2RFIDApplyWrittenTag:
    def test_the_written_tag_is_applied_and_noted_for_the_slot(self):
        reader, unit, motion = ace2_rfid_rescan_rig()
        reader.apply_written_tag("lane1", {"uid": "04ab", "tag_type": "MifareClassic1k",
                                           "filament": {"type": "PETG"}})
        assert reader._slot_uid == {0: "04ab"}
        assert unit.printer.afc.lanes["lane1"].material == "PETG"
        assert unit.printer.logger.messages == []
        assert unit.printer.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n  Name: PETG\n  Material: PETG\n"
                     "  Diameter: 1.75mm\n  Tag UID: 04ab")]

    def test_a_tag_without_uid_is_applied_but_not_noted(self):
        reader, unit, motion = ace2_rfid_rescan_rig()
        reader.apply_written_tag("lane1", {"tag_type": "MifareClassic1k",
                                           "filament": {"type": "PETG"}})
        assert reader._slot_uid == {}
        assert unit.printer.afc.lanes["lane1"].material == "PETG"
        assert unit.printer.logger.messages == []
        assert unit.printer.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n  Name: PETG\n  Material: PETG\n"
                     "  Diameter: 1.75mm")]

    def test_an_unknown_lane_raises(self):
        reader, unit, motion = ace2_rfid_rescan_rig()
        with pytest.raises(RuntimeError, match="^ghost is not an AFC lane$"):
            reader.apply_written_tag("ghost", {"uid": "04ab"})
        assert reader._slot_uid == {}
        assert unit.printer.logger.messages == []


class TestAFCACE2RFIDCmdACERFIDScan:
    SCAN = {"lane_slot_map": "scan_lane:0", "scanner_lanes": "scan_lane",
            "scan_interval": 0.05, "scanner_confirm_reads": 1}
    TAG = {"uid": "abcd", "tag_type": "MifareClassic1k", "filament": {"type": "PLA"}}
    #: The scan summary of TAG.
    LINES = ["Name: PLA", "Material: PLA", "Diameter: 1.75mm"]

    def _rig(self, values: Optional[Dict[str, Any]] = None
             ) -> Tuple[AFC_ACE2_RFID, afcACE2]:
        """
        :param values: options over SCAN
        :return tuple: reader and unit, with scan_lane on slot 0
        """
        return ace2_rfid_rig(dict(self.SCAN, **(values or {})),
                             lanes=[LaneSpec("scan_lane", prep=True)])

    @staticmethod
    def _started(seconds: int) -> List[Tuple[str, str]]:
        """
        :param seconds: the scan's length
        :return list: what the command itself answers
        """
        return [("info", f"ACE2 RFID scan: present the tag to scan_lane (scanning "
                         f"{seconds}s)...")]

    def test_cmd_scan_defaults_to_sole_scanner_lane(self, monkeypatch):
        reader, unit = self._rig()
        printer = unit.printer
        printer.afc.spool.next_spool_id = None
        tags = Ace2RfidTagScript(monkeypatch, self.TAG)
        gcmd = make_gcmd()                               # no LANE: the one scanner lane
        reader.cmd_ACE_RFID_SCAN(gcmd)
        assert gcmd.messages == self._started(30)        # scan_seconds default
        printer.reactor.run_callbacks()
        assert tags.slots == [0]
        assert reader._scan_running is False
        assert reader._slot_uid == {0: "abcd"}
        assert printer.logger.messages == []
        assert printer.gcode.messages == (
            ace2_rfid_hold_prompt("scan_lane")
            + ace2_rfid_scan_popup(ACE2_RFID_SCAN_TITLE, self.LINES)
            + [("info", "ACE2 RFID scan: staged next spool from scan_lane, uid=abcd "
                        "type=PLA")])

    def test_cmd_scan_returns_before_the_scan_runs(self, monkeypatch):
        reader, unit = self._rig()
        printer = unit.printer
        tags = Ace2RfidTagScript(monkeypatch, self.TAG)
        gcmd = make_gcmd(LANE="scan_lane")
        reader.cmd_ACE_RFID_SCAN(gcmd)
        assert tags.calls == []                          # the command did not scan
        assert reader._scan_running is True
        assert len(printer.reactor.pending) == 1         # handed to the reactor
        assert printer.reactor.now == 100.0
        assert gcmd.messages == self._started(30)
        assert printer.gcode.messages == []              # nothing shown before the scan
        printer.reactor.run_callbacks()
        assert tags.slots == [0]
        assert reader._scan_running is False
        assert gcmd.messages == self._started(30)        # nothing more on the command
        assert printer.logger.messages == []
        assert printer.gcode.messages == (
            ace2_rfid_hold_prompt("scan_lane")
            + ace2_rfid_scan_popup(ACE2_RFID_SCAN_TITLE, self.LINES)
            + [("info", "ACE2 RFID scan: staged next spool from scan_lane, uid=abcd "
                        "type=PLA")])

    def test_a_second_scan_is_refused_while_one_is_running(self, monkeypatch):
        reader, unit = self._rig()
        Ace2RfidTagScript(monkeypatch, self.TAG)
        first = make_gcmd()
        reader.cmd_ACE_RFID_SCAN(first)
        assert first.messages == self._started(30)
        second = make_gcmd()
        with pytest.raises(CommandError) as err:
            reader.cmd_ACE_RFID_SCAN(second)
        assert str(err.value) == "ACE2 RFID scan: a scan is already running"
        assert second.messages == []
        assert len(unit.printer.reactor.pending) == 1    # still the first scan only
        assert reader._scan_running is True
        assert unit.printer.logger.messages == []

    def test_the_running_flag_clears_even_when_the_scan_raises(self, monkeypatch):
        # Otherwise one failed scan locks the command out for good. Even a
        # command error is only reported: there is no command left to raise to.
        reader, unit = self._rig()
        printer = unit.printer
        first = make_gcmd(LANE="ghost")
        reader.cmd_ACE_RFID_SCAN(first)
        assert first.messages == [
            ("info", "ACE2 RFID scan: present the tag to ghost (scanning 30s)...")]
        printer.reactor.run_callbacks()                  # does not raise
        assert reader._scan_running is False
        assert printer.logger.messages == [("error", "ACE2 RFID scan failed")]
        assert printer.logger.calls[0][2]["traceback"].splitlines()[-1].endswith(
            "lane 'ghost' has no ACE2 reader slot (set lane_slot_map)")
        assert printer.gcode.messages == [
            ("info", "ACE2 RFID scan: error: lane 'ghost' has no ACE2 reader slot (set "
                     "lane_slot_map)")]
        gcmd = make_gcmd()
        reader.cmd_ACE_RFID_SCAN(gcmd)                   # and the command works again
        assert gcmd.messages == self._started(30)
        assert reader._scan_running is True

    def test_cmd_scan_requires_lane_when_multiple_scanners(self):
        reader, unit = self._rig({"scanner_lanes": "a, b"})
        gcmd = make_gcmd()
        with pytest.raises(CommandError) as err:
            reader.cmd_ACE_RFID_SCAN(gcmd)
        assert str(err.value) == "ACE_RFID_SCAN requires LANE= (no single scanner_lanes lane)"
        assert gcmd.messages == []
        assert reader._scan_running is False
        assert unit.printer.reactor.pending == []
        assert unit.printer.logger.messages == []

    def test_error_swallowed(self, monkeypatch):
        reader, unit = self._rig()
        printer = unit.printer
        unit._ace.set_reply("mfrc522_reader_power", RuntimeError("glitch"))
        gcmd = make_gcmd(LANE="scan_lane")
        reader.cmd_ACE_RFID_SCAN(gcmd)
        assert gcmd.messages == self._started(30)
        printer.reactor.run_callbacks()
        assert reader._scan_running is False
        assert printer.logger.messages == [("error", "ACE2 RFID scan failed")]
        assert printer.logger.calls[0][2]["traceback"].splitlines()[-1] == (
            "RuntimeError: glitch")
        assert printer.gcode.messages == [("info", "ACE2 RFID scan: error: glitch")]

    def test_no_tag(self, monkeypatch):
        reader, unit = self._rig()
        printer = unit.printer
        Ace2RfidTagScript(monkeypatch, None)
        gcmd = make_gcmd(LANE="scan_lane", SECONDS=1)
        reader.cmd_ACE_RFID_SCAN(gcmd)
        assert gcmd.messages == self._started(1)
        printer.reactor.run_callbacks()
        assert reader._scan_running is False
        assert printer.logger.messages == [
            ("info", "ACE2 RFID scan: no tag on lane scan_lane in 1s")]
        assert printer.gcode.messages == [
            ("info", "ACE2 RFID scan: no tag found on scan_lane")]

    def test_success_reports_staged_spool(self, monkeypatch):
        reader, unit = self._rig()
        printer = unit.printer
        printer.afc.spool.next_spool_id = 42
        Ace2RfidTagScript(monkeypatch, self.TAG)
        gcmd = make_gcmd(LANE="scan_lane")
        reader.cmd_ACE_RFID_SCAN(gcmd)
        assert gcmd.messages == self._started(30)
        printer.reactor.run_callbacks()
        assert printer.logger.messages == []
        assert printer.gcode.messages == (
            ace2_rfid_hold_prompt("scan_lane")
            + ace2_rfid_scan_popup(ACE2_RFID_SCAN_TITLE, self.LINES + ["Spoolman ID: 42"])
            + [("info", "ACE2 RFID scan: staged next spool from scan_lane, uid=abcd type=PLA "
                        "(spool #42)")])


class TestLoadConfig:
    def test_returns_instance(self):
        printer = AcePrinter()
        config = AceConfig("AFC_ACE2_rfid", printer, {"scan_seconds": 45})
        reader = load_config(config)
        assert type(reader) is AFC_ACE2_RFID
        assert reader.printer is printer
        assert reader.scan_seconds == 45.0               # built from this section


class TestAFCACE2RFIDUntaggedDefaults:
    def _rig(self):
        reader, unit = ace2_rfid_rig(lanes=["lane0"])
        reader.afc.default_material_type = "PLA"
        lane = reader.afc.lanes["lane0"]
        lane.material = ""
        return reader, unit, lane

    def test_defaults_fill_an_empty_lane(self):
        reader, _, lane = self._rig()
        reader._apply_untagged_defaults(lane)
        assert lane.material == "PLA"

    def test_defaults_never_overwrite(self):
        reader, _, lane = self._rig()
        lane.material = "PETG"
        reader._apply_untagged_defaults("lane0")
        assert lane.material == "PETG"

    def test_auto_read_with_no_tag_applies_defaults(self, monkeypatch):
        reader, _, lane = self._rig()
        monkeypatch.setattr(reader, "read_lane", lambda name: None)
        reader._auto_read("lane0")
        assert lane.material == "PLA"

    def test_auto_read_of_an_undecodable_tag_applies_defaults(self, monkeypatch):
        reader, _, lane = self._rig()
        monkeypatch.setattr(reader, "read_lane",
                            lambda name: {"uid": "04a1b2c3", "filament": None})
        reader._auto_read("lane0")
        assert lane.material == "PLA"

    def test_a_decoded_tag_gets_no_defaults(self, monkeypatch):
        reader, _, lane = self._rig()
        monkeypatch.setattr(reader, "read_lane",
                            lambda name: {"uid": "04a1b2c3", "filament": {"type": "ABS"}})
        reader._auto_read("lane0")
        assert lane.material == ""
