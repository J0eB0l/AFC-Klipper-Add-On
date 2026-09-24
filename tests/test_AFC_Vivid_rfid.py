"""Unit tests for extras/AFC_Vivid_rfid.py."""

from __future__ import annotations

import configparser
from dataclasses import dataclass
import sys
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import pytest

from extras import AFC_RFID as afc_rfid_mod, AFC_Vivid_rfid as vivid_mod
import extras
from extras.AFC_Vivid_rfid import (
    _VividSpiRegLink,
    AFC_Vivid_rfid,
    AFC_Vivid_rfid_reader,
    load_config,
    load_config_prefix,
)
from extras.AFC_lane import AssistActive, SpeedMode
from extras.AFC_rfid_keys import AFC_rfid_keys
from extras.AFC_rfid_readers import Mfrc522
from tests.bambu_helpers import BambuConfig, BambuLogger, BambuPrinter, FakeGcmd, LogLine, Recorder


#: The wall-clock time AFC_RFID stamps on every tag-read record here.
VIVID_WALL_TIME = 1700000000.0


#: config.error() raises this, as Klipper's ConfigWrapper.error does.
VIVID_CONFIG_ERROR = configparser.Error


#: One lane move: (distance, speed_mode, endstop, assist_active, use_homing).
VividMove = Tuple[float, Any, Optional[str], Any, bool]


#: The console read-out of vivid_btt_tag() on slot 3, built by hand.
VIVID_BTT_SLOT3_SUMMARY = ("ViViD RFID tag on slot 3:\n"
                           "  Name: BQ Tech PET (CEP)\n"
                           "  Brand: BQ Tech\n"
                           "  Material: PET\n"
                           "  Color: #c0ffee\n"
                           "  Diameter: 1.75mm\n"
                           "  Nozzle temp: 220\u00b0C (200\u2013240)\n"
                           "  Bed temp: 60\u00b0C\n"
                           "  Tag UID: d13fdb0e")


#: The console read-out apply_to_lane prints for vivid_elegoo_tag() on lane0.
VIVID_ELEGOO_READ_OUT: LogLine = ("respond_info",
                                  "ViViD RFID: read spool on lane0\n"
                                  "  Name: Elegoo PLA\n"
                                  "  Brand: Elegoo\n"
                                  "  Material: PLA\n"
                                  "  Diameter: 1.75mm\n"
                                  "  Tag UID: AABB")


def vivid_btt_tag() -> Dict[str, Any]:
    """:return dict: a decoded BTT "BQ Tech" tag, as read_tag returns it"""
    return {"uid": "d13fdb0e", "tag_type": "MifareClassic1k",
            "filament": {"manufacturer": "BQ Tech", "type": "PET",
                         "detailed": "PET (CEP)", "color_argb": 0xFFC0FFEE,
                         "diameter_mm": 1.75, "bed_temp_c": 60,
                         "hotend_min_c": 200, "hotend_max_c": 240}}


def vivid_elegoo_tag() -> Dict[str, Any]:
    """:return dict: a decoded Elegoo PLA tag, as read_tag returns it"""
    return {"uid": "AABB", "tag_type": "MifareClassic1k",
            "filament": {"type": "PLA", "manufacturer": "Elegoo"}}


def vivid_elegoo_record() -> Dict[str, Any]:
    """:return dict: the last-read record a decoded vivid_elegoo_tag() leaves"""
    return {"material": "PLA", "is_dual_color": False, "brand": "Elegoo",
            "diameter": 1.75, "uid": "AABB", "tag_type": "MifareClassic1k",
            "decoded": True, "scan_time": VIVID_WALL_TIME}


class VividSpi:
    """A reader's MCU_SPI: records every frame; a transfer answers ``response``."""

    def __init__(self, response: bytes = b"\x00\x00") -> None:
        """:param response: the bytes every spi_transfer clocks in"""
        self.response = response
        self.sent: List[List[int]] = []
        self.transfers: List[List[int]] = []

    def spi_send(self, data: List[int]) -> None:
        """:param data: the frame written"""
        self.sent.append(list(data))

    def spi_transfer(self, data: List[int]) -> Dict[str, bytes]:
        """
        :param data: the frame clocked out
        :return dict: Klipper's reply, the bytes clocked in under "response"
        """
        self.transfers.append(list(data))
        return {"response": self.response}


def install_vivid_bus(monkeypatch: pytest.MonkeyPatch, bus: "VividBus") -> None:
    """
    Install the fake as klippy's bus module under both names the reader tries.

    Another test file may already have left a stub ``extras.bus`` behind, and
    the reader's ``from . import bus`` finds that one first.

    :param monkeypatch: undoes the install after the test
    :param bus: the fake bus module
    """
    monkeypatch.setitem(sys.modules, "bus", bus)
    monkeypatch.setitem(sys.modules, "extras.bus", bus)
    monkeypatch.setattr(extras, "bus", bus, raising=False)


class VividBus:
    """klippy's ``bus`` module as the reader imports it: MCU_SPI_from_config
    builds a VividSpi and records how it was asked for."""

    def __init__(self) -> None:
        """Start with no SPI built."""
        self.requests: List[Tuple[str, int, Dict[str, Any]]] = []
        self.spis: List[VividSpi] = []

    def MCU_SPI_from_config(self, config: Any, mode: int,
                            pin_option: str = "cs_pin",
                            default_speed: int = 100000,
                            share_type: Any = None,
                            cs_active_high: bool = False) -> VividSpi:
        """
        :param config: the reader section's config
        :param mode: the SPI mode
        :param pin_option: the option naming the CS pin
        :param default_speed: the clock when the section sets none
        :param share_type: bus sharing, unused here
        :param cs_active_high: the CS polarity
        :return VividSpi: a new SPI
        """
        self.requests.append((config.get_name(), mode,
                              {"pin_option": pin_option,
                               "default_speed": default_speed,
                               "cs_active_high": cs_active_high}))
        spi = VividSpi()
        self.spis.append(spi)
        return spi


class VividTagField:
    """
    The shared ``read_tag`` stack, as the coil sees it.

    Each read answers the next scripted tag (the last one repeats), or raises
    ``raises``. Like read_tag, a tag the excluder halts is passed over.
    """

    def __init__(self, *tags: Optional[Dict[str, Any]]) -> None:
        """:param tags: what successive reads answer; None is an empty field"""
        self.tags: List[Optional[Dict[str, Any]]] = list(tags) or [None]
        self.raises: Optional[Exception] = None
        self.calls: List[Tuple[Any, Dict[str, Any]]] = []

    def __call__(self, link: Any, **kwargs: Any) -> Optional[Dict[str, Any]]:
        """
        :param link: the reader's register link
        :return Optional[dict]: the tag read, or None
        """
        self.calls.append((link, kwargs))
        if self.raises is not None:
            raise self.raises
        tag = self.tags.pop(0) if len(self.tags) > 1 else self.tags[0]
        excluded = kwargs.get("is_excluded")
        if tag and excluded is not None and excluded(str(tag.get("uid", "")).lower()):
            return None
        return tag


class VividAntenna:
    """
    ``MifareClassic`` as the stage poll uses it: built on the real Mfrc522,
    ``activate`` answers the UID of the tag on the coil, passing over one
    the excluder halts, or raises ``raises``.
    """

    def __init__(self, uid: Optional[bytes] = None) -> None:
        """:param uid: the UID of the tag on the coil; None for a clear coil"""
        self.uid = uid
        self.raises: Optional[Exception] = None
        self.activations: List[Tuple[Any, Optional[Callable[[str], bool]]]] = []
        self._mfrc: Optional[Mfrc522] = None

    def __call__(self, mfrc: Mfrc522) -> "VividAntenna":
        """
        :param mfrc: the MFRC522 driver the module built on a reader's link
        :return VividAntenna: this antenna, standing in for the new reader
        """
        self._mfrc = mfrc
        return self

    def activate(self, is_excluded: Optional[Callable[[str], bool]] = None
                 ) -> Tuple[Optional[bytes], Optional[int]]:
        """
        :param is_excluded: uid_hex -> True for a tag to halt
        :return tuple: (uid, sak), or (None, None) when no tag answers
        """
        link = self._mfrc.l if self._mfrc is not None else None
        self.activations.append((link, is_excluded))
        if self.raises is not None:
            raise self.raises
        if self.uid is None:
            return None, None
        if is_excluded is not None and is_excluded(self.uid.hex()):
            return None, None
        return self.uid, 0x08


class VividUnit:
    """A lane's unit: ``move_to_load`` as afcUnit's, a homing move onto the
    lane's load switch."""

    def __init__(self) -> None:
        """The unit leaves Spoolman auto-create to the RFID module."""
        self.auto_spoolman_create = False

    def move_to_load(self, lane: "VividLane", dist: float, dir: Any,
                     use_homing: bool = True,
                     speed_mode: Any = SpeedMode.LONG) -> Tuple[bool, float, None]:
        """
        :param lane: the lane to move
        :param dist: mm
        :param dir: the MoveDirection
        :param use_homing: home on the load switch
        :param speed_mode: AFC's speed mode
        :return tuple: the lane's move_to result
        """
        return lane.move_to(dist * dir, speed_mode, endstop=lane.load_es,
                            assist_active=AssistActive.DYNAMIC,
                            use_homing=use_homing)


class VividLane:
    """
    An AFC lane whose LOAD switch tracks the filament tip.

    ``pos`` is the tip; the switch reads filament above ``load_at``. ``slip``
    is the fraction of each feed the gears lose. A homing move stops just past
    the switch edge. Every move is recorded in ``moves``; with ``fault`` set
    every move raises it.
    """

    def __init__(self, name: str, *, pos: float = 500.0, load_at: float = 0.0,
                 slip: float = 0.0, prep_state: bool = True,
                 tool_loaded: bool = False,
                 load_endstop_name: Optional[str] = None,
                 unit: bool = True) -> None:
        """
        :param name: the lane's name
        :param pos: where the tip starts; 500mm is well past the switch
        :param load_at: where the load switch trips
        :param slip: fraction of each feed that is lost
        :param prep_state: the prep switch reads filament
        :param tool_loaded: loaded in the toolhead
        :param load_endstop_name: the load sensor's endstop name
        :param unit: give the lane a unit with move_to_load
        """
        self.name = name
        self.prep_state = prep_state
        self.tool_loaded = tool_loaded
        self.short_move_dis = 10.0
        self.load_es = "load"
        self.load_endstop_name = load_endstop_name
        self.unit_obj: Optional[VividUnit] = VividUnit() if unit else None
        self.pos = pos
        self.load_at = load_at
        self.slip = slip
        self.fault: Optional[Exception] = None
        self.moves: List[VividMove] = []
        # The filament fields apply_to_lane fills from a decoded tag.
        self.material: Optional[str] = None
        self.color: Optional[str] = None
        self.extruder_temp: Optional[float] = None
        self.bed_temp: Optional[float] = None
        self.weight = 0
        self.spool_vendor = ""
        self.multi_color: Optional[List[str]] = None
        self.spool_id: Optional[int] = None

    @property
    def raw_load_state(self) -> bool:
        """:return bool: whether the load switch reads filament"""
        return self.pos > self.load_at

    def move_to(self, distance: float, speed_mode: Any,
                endstop: Optional[str] = None, assist_active: Any = None,
                use_homing: bool = True) -> Tuple[bool, float, None]:
        """
        :param distance: signed mm
        :param speed_mode: AFC's speed mode
        :param endstop: the endstop to home to
        :param assist_active: the espooler mode
        :param use_homing: home on the endstop
        :return tuple: (homed, distance, None)
        """
        self.moves.append((distance, speed_mode, endstop, assist_active,
                           use_homing))
        if self.fault is not None:
            raise self.fault
        gained = distance * (1.0 - self.slip) if distance > 0 else distance
        if use_homing and endstop is not None:
            self.pos = (min(self.load_at + 0.5, self.pos + gained) if distance > 0
                        else max(self.load_at - 0.5, self.pos + gained))
        else:
            self.pos += gained
        return True, abs(distance), None


class VividBareAfc:
    """An AFC object without a ``lanes`` table."""


class VividWallClock:
    """``time`` as AFC_RFID reads it: a wall clock stopped at one instant."""

    def time(self) -> float:
        """:return float: VIVID_WALL_TIME"""
        return VIVID_WALL_TIME


class VividTriggerCmd:
    """A trsync's trigger command: records each send, or raises ``raises``."""

    def __init__(self, raises: Optional[Exception] = None) -> None:
        """:param raises: what every send raises"""
        self.raises = raises
        self.sent: List[List[int]] = []

    def send(self, data: List[int]) -> None:
        """:param data: the command's arguments"""
        if self.raises is not None:
            raise self.raises
        self.sent.append(list(data))


class VividTrsync:
    """An MCU trsync, with Klipper's host-request reason code."""

    REASON_HOST_REQUEST = 3

    def __init__(self, oid: int, cmd: VividTriggerCmd) -> None:
        """
        :param oid: the trsync's object id
        :param cmd: its trigger command
        """
        self._oid = oid
        self._trsync_trigger_cmd = cmd


class VividDispatch:
    """An endstop's trigger dispatch: the trsyncs it drives."""

    def __init__(self, trsyncs: Sequence[VividTrsync]) -> None:
        """:param trsyncs: the trsyncs, first one is the endstop's own"""
        self._trsyncs = list(trsyncs)


class VividMcuEndstop:
    """An MCU endstop with its dispatch."""

    def __init__(self, dispatch: Optional[VividDispatch]) -> None:
        """:param dispatch: the dispatch, or None before a homing move"""
        self._dispatch = dispatch


class VividQueryEndstops:
    """query_endstops: (mcu_endstop, name) for every registered endstop."""

    def __init__(self, endstops: Sequence[Tuple[VividMcuEndstop, str]]) -> None:
        """:param endstops: the registered endstops"""
        self.endstops = list(endstops)


def add_vivid_endstop(printer: BambuPrinter, name: str = "load_es0",
                      cmd: Optional[VividTriggerCmd] = None) -> VividTriggerCmd:
    """
    Register query_endstops holding one load endstop on a trsync with oid 7.

    :param printer: the printer
    :param name: the endstop's name
    :param cmd: its trsync's trigger command; a recording one when None
    :return VividTriggerCmd: the trigger command
    """
    cmd = cmd or VividTriggerCmd()
    endstop = VividMcuEndstop(VividDispatch([VividTrsync(7, cmd)]))
    printer.add_object("query_endstops", VividQueryEndstops([(endstop, name)]))
    return cmd


@dataclass
class VividRig:
    """A built coordinator and the fakes around it."""

    printer: BambuPrinter
    unit: AFC_Vivid_rfid
    readers: Dict[str, AFC_Vivid_rfid_reader]
    bus: VividBus
    field: VividTagField
    antenna: VividAntenna

    @property
    def logger(self) -> BambuLogger:
        """:return BambuLogger: AFC's logger, which the module logs to"""
        return self.printer.afc.logger

    @property
    def console(self) -> List[LogLine]:
        """:return List[LogLine]: the gcode console"""
        return self.printer.gcode.messages

    def lane(self, name: str) -> VividLane:
        """:return VividLane: the AFC lane registered under ``name``"""
        return self.printer.afc.lanes[name]


def build_vivid(monkeypatch: pytest.MonkeyPatch, *,
                options: Optional[Dict[str, Any]] = None,
                readers: Sequence[Tuple[str, str]] = (("reader0", "0, 1"),),
                lanes: Sequence[VividLane] = (),
                ready: bool = True) -> VividRig:
    """
    Build ``[AFC_Vivid_rfid]`` and its reader sections the way klippy does.

    :param monkeypatch: installs klippy's bus module, and patches read_tag and
        MifareClassic in the module and AFC_RFID's wall clock
    :param options: the coordinator's options
    :param readers: (name, slots) per reader section
    :param lanes: AFC lanes to register
    :param ready: send klippy:ready, then clear the logger and console
    :return VividRig: the coordinator and its fakes
    """
    bus = VividBus()
    install_vivid_bus(monkeypatch, bus)
    field = VividTagField()
    monkeypatch.setattr(vivid_mod, "read_tag", field)
    antenna = VividAntenna()
    monkeypatch.setattr(vivid_mod, "MifareClassic", antenna)
    monkeypatch.setattr(afc_rfid_mod, "time", VividWallClock())
    printer = BambuPrinter()
    unit = AFC_Vivid_rfid(BambuConfig("AFC_Vivid_rfid", printer, options))
    printer.add_object("AFC_Vivid_rfid", unit)
    built: Dict[str, AFC_Vivid_rfid_reader] = {}
    for index, (name, slots) in enumerate(readers):
        section = f"AFC_Vivid_rfid {name}"
        built[name] = AFC_Vivid_rfid_reader(BambuConfig(
            section, printer, {"cs_pin": f"PA{index}", "spi_bus": "spi1",
                               "slots": slots}))
        printer.add_object(section, built[name])
    for lane in lanes:
        printer.afc.lanes[lane.name] = lane
    if ready:
        printer.send_event("klippy:ready")
        printer.afc.logger.messages.clear()
        printer.gcode.messages.clear()
    return VividRig(printer, unit, built, bus, field, antenna)


def vivid_pair_rig(monkeypatch: pytest.MonkeyPatch, *,
                   options: Optional[Dict[str, Any]] = None,
                   sibling: Optional[VividLane] = None) -> VividRig:
    """
    lane0 on slot 0 and lane1 on slot 1, sharing reader0's antenna.

    :param monkeypatch: for build_vivid
    :param options: coordinator options over the lane_slot_map
    :param sibling: lane1; a seated, idle one when None
    :return VividRig: the rig
    """
    values: Dict[str, Any] = {"lane_slot_map": "lane0:0, lane1:1"}
    values.update(options or {})
    lanes = [VividLane("lane0"), sibling or VividLane("lane1")]
    return build_vivid(monkeypatch, options=values, lanes=lanes)


def vivid_probe(**changes: Any) -> Dict[str, Any]:
    """
    :param changes: fields that differ from a fresh probe
    :return dict: the stage-read probe for lane0 on slot 0
    """
    probe: Dict[str, Any] = {"slot": 0, "lane": "lane0", "uid": None, "count": 0,
                             "done": False, "sib_lane": None, "sib_dist": 0.0,
                             "baseline": None, "blocked_sib": None,
                             "read_ok": False}
    probe.update(changes)
    return probe


def vivid_retract_moves(*hops: float) -> List[VividMove]:
    """
    :param hops: the retract's steps, in mm
    :return list: the moves a sibling retract makes for them
    """
    return [(-hop, SpeedMode.SHORT, None, AssistActive.YES, False) for hop in hops]


#: The steps of a full 75mm sibling retract, 10mm at a time.
VIVID_FULL_RETRACT = (10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 5.0)


#: The log line of a sibling retract on lane1 before reading slot 0.
VIVID_RETRACTING: LogLine = (
    "info", "ViViD RFID: retracting sibling lane1 up to 75mm to clear the antenna "
            "before reading slot 0")


#: The log line of tag cafe found parked on the coil before reading slot 0.
VIVID_PARKED_CAFE: LogLine = (
    "info", "ViViD RFID: tag cafe parked on the shared reader for slot 0, "
            "clearing the sibling before the read")


#: The log line of a sibling retract refused because lane1 is loaded or printing.
VIVID_REFUSED: LogLine = (
    "info", "ViViD RFID: sibling lane1 is loaded/printing, not moving it; "
            "relying on the HALT dedup (nudge by hand if the read misses)")


class TestVividSpiRegLinkRegRead:
    def test_reg_read_address_byte_has_read_bit_and_returns_second_byte(self):
        spi = VividSpi(response=b"\x00\x42")
        link = _VividSpiRegLink(spi)
        # VersionReg 0x37: 0x80 | 0x6E
        assert link.reg_read(0x37) == 0x42
        assert spi.transfers == [[0xEE, 0x00]]
        assert spi.sent == []

    def test_reg_read_short_response_reads_zero(self):
        spi = VividSpi(response=b"\x42")
        link = _VividSpiRegLink(spi)
        assert link.reg_read(0x37) == 0
        assert spi.transfers == [[0xEE, 0x00]]


class TestVividSpiRegLinkRegWrite:
    def test_reg_write_address_byte_is_reg_shifted_left(self):
        spi = VividSpi()
        link = _VividSpiRegLink(spi)
        link.reg_write(0x11, 0x3D)                 # ModeReg
        assert spi.sent == [[0x22, 0x3D]]          # write bit clear
        assert spi.transfers == []

    def test_reg_write_value_is_one_byte(self):
        spi = VividSpi()
        _VividSpiRegLink(spi).reg_write(0x11, 0x13D)
        assert spi.sent == [[0x22, 0x3D]]


class TestVividSpiRegLinkReaderPower:
    def test_reader_power_is_noop(self):
        spi = VividSpi()
        link = _VividSpiRegLink(spi)
        assert link.reader_power(True) is None
        assert link.reader_power(False) is None
        assert spi.sent == [] and spi.transfers == []


class TestAFCVividrfidreaderInit:
    @staticmethod
    def _config(monkeypatch: pytest.MonkeyPatch,
                slots: Optional[str]) -> Tuple[BambuConfig, VividBus]:
        """
        :param monkeypatch: installs klippy's bus module
        :param slots: the section's slots option; None leaves it unset
        :return tuple: the reader section's config and the bus
        """
        bus = VividBus()
        install_vivid_bus(monkeypatch, bus)
        values: Dict[str, Any] = {"cs_pin": "PA4", "spi_bus": "spi1"}
        if slots is not None:
            values["slots"] = slots
        return BambuConfig("AFC_Vivid_rfid reader0", BambuPrinter(), values), bus

    def test_parses_slots_and_builds_link(self, monkeypatch):
        config, bus = self._config(monkeypatch, "0, 1")
        reader = AFC_Vivid_rfid_reader(config)
        printer = config.get_printer()
        assert reader.printer is printer
        assert reader.name == "reader0"
        assert reader.logger is printer.afc.logger
        assert reader.slots == [0, 1]
        assert bus.requests == [("AFC_Vivid_rfid reader0", 0,
                                 {"pin_option": "cs_pin", "default_speed": 5000000,
                                  "cs_active_high": False})]
        assert reader.spi is bus.spis[0]
        assert isinstance(reader.link, _VividSpiRegLink)
        assert reader.link.spi is reader.spi
        assert printer.afc.logger.messages == []

    def test_without_extras_bus_the_top_level_bus_is_used(self, monkeypatch):
        config, bus = self._config(monkeypatch, "0")
        monkeypatch.setitem(sys.modules, "extras.bus", None)
        monkeypatch.delattr(extras, "bus", raising=False)
        reader = AFC_Vivid_rfid_reader(config)
        assert reader.spi is bus.spis[0]
        assert config.get_printer().afc.logger.messages == []

    def test_blank_slot_entries_skipped(self, monkeypatch):
        config, _bus = self._config(monkeypatch, " , 2 , ")
        assert AFC_Vivid_rfid_reader(config).slots == [2]
        assert config.get_printer().afc.logger.messages == []

    def test_empty_slots_gives_empty_list(self, monkeypatch):
        config, _bus = self._config(monkeypatch, "")
        assert AFC_Vivid_rfid_reader(config).slots == []
        assert config.get_printer().afc.logger.messages == []

    def test_unset_slots_gives_empty_list(self, monkeypatch):
        config, _bus = self._config(monkeypatch, None)
        assert AFC_Vivid_rfid_reader(config).slots == []
        assert config.get_printer().afc.logger.messages == []

    def test_bad_slot_number_raises(self, monkeypatch):
        config, _bus = self._config(monkeypatch, "0, x")
        with pytest.raises(VIVID_CONFIG_ERROR) as exc:
            AFC_Vivid_rfid_reader(config)
        assert str(exc.value) == "AFC_Vivid_rfid reader0: bad slot number 'x' in 'slots'"
        assert config.get_printer().afc.logger.messages == []


class TestAFCVividrfidInit:
    def test_lane_slot_map_parsed(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False, options={
            "lane_slot_map": "lane0:0, lane1:1, , lane2:2, lane3 : 3,"})
        assert rig.unit._lane_slot == {"lane0": 0, "lane1": 1, "lane2": 2,
                                       "lane3": 3}
        assert rig.unit._get_slot("lane2") == 2
        assert rig.unit._get_slot("nope") is None
        assert rig.logger.messages == []

    def test_defaults(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False)
        unit = rig.unit
        assert unit.printer is rig.printer
        assert unit.reactor is rig.printer.reactor
        assert unit.gcode is rig.printer.gcode
        assert unit.logger is rig.printer.afc.logger
        assert unit.afc is None
        assert (unit.bambu_master_key, unit.creality_key,
                unit.creality_encryption_key) == (None, None, None)
        assert unit.auto_create is False
        assert unit.log_prefix == "ViViD RFID"
        assert unit.stage_read is True
        assert unit.stage_poll_interval == 0.15
        assert unit.stage_confirm_reads == 2
        assert unit.stage_max_aborts == 3
        assert unit.auto_tag_adjust is True
        assert unit.auto_tag_adjust_dist == 75.0
        assert unit._lane_slot == {}
        assert unit._slot_reader == {} and unit._slot_lane == {}
        assert unit._last == {} and unit._slot_uid == {}
        assert unit._probe is None and unit._poll_timer is None
        assert rig.logger.messages == []

    def test_options_and_brand_keys_parsed(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False, options={
            "bambu_master_key": " 00ff ", "creality_key": "0102",
            "creality_encryption_key": "a0b0", "auto_spoolman_create": True,
            "stage_read": False, "stage_poll_interval": 0.3,
            "stage_confirm_reads": 4, "stage_max_aborts": 5,
            "auto_tag_adjust": False, "auto_tag_adjust_dist": 40.0})
        unit = rig.unit
        assert unit.bambu_master_key == b"\x00\xff"
        assert unit.creality_key == b"\x01\x02"
        assert unit.creality_encryption_key == b"\xa0\xb0"
        assert unit.auto_create is True
        assert unit.stage_read is False
        assert unit.stage_poll_interval == 0.3
        assert unit.stage_confirm_reads == 4
        assert unit.stage_max_aborts == 5
        assert unit.auto_tag_adjust is False
        assert unit.auto_tag_adjust_dist == 40.0
        assert rig.logger.messages == []

    def test_bad_lane_slot_map_raises(self):
        printer = BambuPrinter()
        with pytest.raises(VIVID_CONFIG_ERROR) as exc:
            AFC_Vivid_rfid(BambuConfig("AFC_Vivid_rfid", printer,
                                       {"lane_slot_map": "lane0"}))
        assert str(exc.value) == ("AFC_Vivid_rfid: 'lane_slot_map' entries must be "
                                  "'lane:slot', got 'lane0'")
        assert printer.afc.logger.messages == []

    def test_bad_slot_number_in_lane_slot_map_raises(self):
        printer = BambuPrinter()
        with pytest.raises(VIVID_CONFIG_ERROR) as exc:
            AFC_Vivid_rfid(BambuConfig("AFC_Vivid_rfid", printer,
                                       {"lane_slot_map": "lane0:x"}))
        assert str(exc.value) == "AFC_Vivid_rfid: bad slot number in 'lane0:x'"
        assert printer.afc.logger.messages == []

    def test_read_command_registered(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False)
        assert rig.printer.gcode.ready_gcode_handlers == {
            "VIVID_RFID_READ": rig.unit.cmd_VIVID_RFID_READ}
        assert rig.logger.messages == []

    def test_stage_read_handlers_registered(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False)
        assert rig.printer._event_handlers == {
            "klippy:ready": [rig.unit._on_ready],
            "afc_vivid:stage_read_begin": [rig.unit._stage_read_begin],
            "afc_vivid:stage_read_end": [rig.unit._stage_read_end]}
        assert rig.logger.messages == []


class TestAFCVividrfidOnReady:
    MAPPED_ONE_READER: LogLine = (
        "info", "AFC_Vivid_rfid: 2 slot(s) mapped across 1 reader(s)")

    @staticmethod
    def _add_keys(rig: VividRig, values: Dict[str, Any]) -> None:
        """
        Register a real [AFC_rfid_keys] section.

        :param rig: the rig whose printer gets it
        :param values: its options
        """
        rig.printer.add_object("AFC_rfid_keys", AFC_rfid_keys(
            BambuConfig("AFC_rfid_keys", rig.printer, values)))

    def test_on_ready_indexes_slots_to_readers(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False,
                          options={"lane_slot_map": "lane0:0, lane3:3"},
                          readers=(("reader0", "0, 1"), ("reader1", "2, 3")))
        r0, r1 = rig.readers["reader0"], rig.readers["reader1"]
        rig.printer.send_event("klippy:ready")
        assert rig.unit.afc is rig.printer.afc
        assert rig.unit._slot_reader == {0: r0, 1: r0, 2: r1, 3: r1}
        assert rig.unit._slot_lane == {0: "lane0", 3: "lane3"}
        assert rig.unit.get_status()["slots"] == [0, 1, 2, 3]
        assert rig.logger.messages == [
            ("info", "AFC_Vivid_rfid: 4 slot(s) mapped across 2 reader(s)")]
        registry = rig.printer._afc_rfid_write_registry
        assert {name: t.label for name, t in registry.items()} == {
            "vivid:reader0": "ViViD reader0 (slots 0, 1)",
            "vivid:reader1": "ViViD reader1 (slots 2, 3)"}
        target = registry["vivid:reader0"]
        assert target.unit is rig.unit
        assert target.open_link() is r0.link
        assert registry["vivid:reader1"].open_link() is r1.link
        assert target.serves("lane0") is True
        assert target.serves("lane3") is False

    def test_no_reader_sections_warns_reads_disabled(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False, readers=())
        rig.printer.send_event("klippy:ready")
        assert rig.unit._slot_reader == {}
        assert rig.logger.messages == [
            ("warning", "AFC_Vivid_rfid: no [AFC_Vivid_rfid <name>] reader sections "
                        "found, RFID reads disabled")]

    def test_only_reader_sections_are_indexed(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False, readers=(("reader0", "0, 1"),))
        # A reader object outside a reader section, and a non-reader inside one.
        spare = AFC_Vivid_rfid_reader(BambuConfig(
            "AFC_Vivid_rfid spare", rig.printer, {"cs_pin": "PA9", "slots": "4"}))
        rig.printer.add_object("spare_reader", spare)
        rig.printer.add_object("AFC_Vivid_rfid notes", VividBareAfc())
        rig.printer.send_event("klippy:ready")
        r0 = rig.readers["reader0"]
        assert rig.unit._slot_reader == {0: r0, 1: r0}
        assert rig.logger.messages == [self.MAPPED_ONE_READER]

    def test_shared_keys_fallback_fills_unset_key(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False)
        self._add_keys(rig, {"bambu_master_key": "aabb"})
        rig.printer.send_event("klippy:ready")
        assert rig.unit.bambu_master_key == b"\xaa\xbb"
        assert rig.unit.creality_key is None
        assert rig.unit.creality_encryption_key is None
        assert rig.logger.messages == [self.MAPPED_ONE_READER]

    def test_shared_keys_do_not_override_own_key(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False,
                          options={"bambu_master_key": "0011"})
        self._add_keys(rig, {"bambu_master_key": "aa", "creality_key": "bb"})
        rig.printer.send_event("klippy:ready")
        assert rig.unit.bambu_master_key == b"\x00\x11"
        assert rig.unit.creality_key == b"\xbb"
        assert rig.logger.messages == [self.MAPPED_ONE_READER]

    def test_resolves_keys_when_helper_present(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False,
                          options={"bambu_master_key": "0011"})
        resolve = Recorder(result=(b"\x01", b"\x02", b"\x03"))
        monkeypatch.setattr(vivid_mod, "resolve_rfid_keys", resolve)
        rig.printer.send_event("klippy:ready")
        assert resolve.calls == [((rig.printer, b"\x00\x11", None, None), {})]
        assert rig.unit.bambu_master_key == b"\x01"
        assert rig.unit.creality_key == b"\x02"
        assert rig.unit.creality_encryption_key == b"\x03"
        assert rig.logger.messages == [self.MAPPED_ONE_READER]

    def test_skips_resolve_when_helper_absent(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False,
                          options={"bambu_master_key": "0011"})
        self._add_keys(rig, {"creality_key": "bb"})
        monkeypatch.setattr(vivid_mod, "resolve_rfid_keys", None)
        rig.printer.send_event("klippy:ready")
        assert rig.unit.bambu_master_key == b"\x00\x11"
        assert rig.unit.creality_key is None
        assert rig.logger.messages == [self.MAPPED_ONE_READER]

    def test_duplicate_slot_warns_and_keeps_first(self, monkeypatch):
        rig = build_vivid(monkeypatch, ready=False,
                          readers=(("reader0", "0, 1"), ("reader1", "0, 2")))
        r0, r1 = rig.readers["reader0"], rig.readers["reader1"]
        rig.printer.send_event("klippy:ready")
        assert rig.unit._slot_reader == {0: r0, 1: r0, 2: r1}
        assert rig.logger.messages == [
            ("warning", "AFC_Vivid_rfid: slot 0 served by more than one reader; "
                        "keeping reader0"),
            ("info", "AFC_Vivid_rfid: 3 slot(s) mapped across 2 reader(s)")]


class TestAFCVividrfidSiblingSlot:
    def test_sibling_slot_is_the_other_slot_on_the_reader(self, monkeypatch):
        rig = build_vivid(monkeypatch,
                          readers=(("reader0", "0, 1"), ("reader1", "2, 3")))
        assert [rig.unit._sibling_slot(s) for s in (0, 1, 2, 3)] == [1, 0, 3, 2]
        assert rig.logger.messages == []

    def test_returns_other_slot(self, monkeypatch):
        rig = build_vivid(monkeypatch, readers=(("reader0", "4, 6"),))
        assert rig.unit._sibling_slot(4) == 6
        assert rig.unit._sibling_slot(6) == 4
        assert rig.logger.messages == []

    def test_none_when_no_reader(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        assert rig.unit._sibling_slot(7) is None
        assert rig.logger.messages == []

    def test_none_when_multiple_others(self, monkeypatch):
        rig = build_vivid(monkeypatch, readers=(("reader0", "0, 1, 2"),))
        assert rig.unit._sibling_slot(0) is None
        assert rig.logger.messages == []

    def test_none_when_single_slot_reader(self, monkeypatch):
        rig = build_vivid(monkeypatch, readers=(("reader0", "5"),))
        assert rig.unit._sibling_slot(5) is None
        assert rig.logger.messages == []


class TestAFCVividrfidSiblingHasSpool:
    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch, prep_state: bool = False) -> VividRig:
        """
        :param monkeypatch: for build_vivid
        :param prep_state: lane1's prep switch
        :return VividRig: lane1 on slot 1, registered with AFC
        """
        return build_vivid(monkeypatch, options={"lane_slot_map": "lane1:1"},
                           lanes=[VividLane("lane1", prep_state=prep_state)])

    def test_true_when_sib_not_mapped(self, monkeypatch):
        rig = self._rig(monkeypatch)
        assert rig.unit._sibling_has_spool(9) is True
        assert rig.logger.messages == []

    def test_true_when_afc_none(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit.afc = None
        assert rig.unit._sibling_has_spool(1) is True
        assert rig.logger.messages == []

    def test_true_when_afc_has_no_lanes(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit.afc = VividBareAfc()
        assert rig.unit._sibling_has_spool(1) is True
        assert rig.logger.messages == []

    def test_true_when_lane_missing(self, monkeypatch):
        rig = self._rig(monkeypatch)
        del rig.printer.afc.lanes["lane1"]
        assert rig.unit._sibling_has_spool(1) is True
        assert rig.logger.messages == []

    def test_true_when_prep_state_true(self, monkeypatch):
        rig = self._rig(monkeypatch, prep_state=True)
        assert rig.unit._sibling_has_spool(1) is True
        assert rig.logger.messages == []

    def test_false_when_prep_state_false(self, monkeypatch):
        rig = self._rig(monkeypatch)
        assert rig.unit._sibling_has_spool(1) is False
        assert rig.logger.messages == []


class TestAFCVividrfidSiblingExcluder:
    def test_none_when_no_sibling(self, monkeypatch):
        rig = build_vivid(monkeypatch, readers=(("reader0", "5"),))
        rig.unit._slot_uid[5] = "AABB"
        assert rig.unit._sibling_excluder(5) is None
        assert rig.logger.messages == []

    def test_none_when_sibling_uid_unknown(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        assert rig.unit._sibling_excluder(0) is None
        assert rig.logger.messages == []

    def test_none_when_sibling_spool_removed(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch,
                             sibling=VividLane("lane1", prep_state=False))
        rig.unit._slot_uid[1] = "AABBCCDD"
        assert rig.unit._sibling_excluder(0) is None
        assert rig.logger.messages == []

    def test_predicate_halts_sibling_uid(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit._slot_uid[1] = "AABBCCDD"
        ex = rig.unit._sibling_excluder(0)
        assert ex is not None
        assert ex("aabbccdd") is True
        assert ex("AABBCCDD") is True
        assert ex("11223344") is False
        assert ex(None) is False
        assert rig.logger.messages == []


class TestAFCVividrfidExcluderWith:
    def test_returns_base_when_no_extra(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit._slot_uid[1] = "AABB"
        ex = rig.unit._excluder_with(0, None)
        assert ex is not None
        assert ex("aabb") is True
        assert ex("beef") is False
        assert rig.logger.messages == []

    def test_returns_none_base_when_no_extra_and_no_sibling(self, monkeypatch):
        rig = build_vivid(monkeypatch, readers=(("reader0", "5"),))
        assert rig.unit._excluder_with(5, None) is None
        assert rig.unit._excluder_with(5, "") is None
        assert rig.logger.messages == []

    def test_extra_only_when_base_none(self, monkeypatch):
        rig = build_vivid(monkeypatch, readers=(("reader0", "5"),))
        ex = rig.unit._excluder_with(5, "CAFE")
        assert ex is not None
        assert ex("cafe") is True
        assert ex("CAFE") is True
        assert ex("beef") is False
        assert ex(None) is False
        assert rig.logger.messages == []

    def test_extra_and_base_both_halt(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit._slot_uid[1] = "AABB"
        ex = rig.unit._excluder_with(0, "CAFE")
        assert ex("cafe") is True
        assert ex("aabb") is True
        assert ex("beef") is False
        assert rig.logger.messages == []


class TestAFCVividrfidParkedTag:
    def test_none_when_no_reader(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.antenna.uid = b"\xca\xfe"
        assert rig.unit._parked_tag(9) is None
        assert rig.antenna.activations == []
        assert rig.logger.messages == []

    def test_returns_hex_uid(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.antenna.uid = b"\xca\xfe"
        assert rig.unit._parked_tag(0) == "cafe"
        # A raw probe on reader0's link, with nothing excluded.
        assert rig.antenna.activations == [(rig.readers["reader0"].link, None)]
        assert rig.logger.messages == []

    def test_none_when_uid_missing(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        assert rig.unit._parked_tag(0) is None
        assert rig.antenna.activations == [(rig.readers["reader0"].link, None)]
        assert rig.logger.messages == []

    def test_none_on_activate_exception(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.antenna.uid = b"\xca\xfe"
        rig.antenna.raises = RuntimeError("boom")
        assert rig.unit._parked_tag(0) is None
        assert rig.logger.messages == []


class TestAFCVividrfidReadSlot:
    def test_read_slot_uses_the_slots_reader_link(self, monkeypatch):
        rig = build_vivid(monkeypatch,
                          readers=(("reader0", "0, 1"), ("reader1", "2, 3")))
        tag = {"uid": "aabb", "filament": {"type": "PLA", "manufacturer": "BQ Tech"}}
        rig.field.tags = [tag]
        assert rig.unit.read_slot(3) is tag
        assert rig.field.calls == [(rig.readers["reader1"].link, {
            "bambu_master_key": None, "creality_key": None,
            "creality_encryption_key": None, "is_excluded": None})]
        assert rig.unit._slot_uid == {3: "aabb"}
        assert rig.logger.messages == []

    def test_read_slot_none_when_no_reader(self, monkeypatch):
        rig = build_vivid(monkeypatch, readers=())
        rig.field.tags = [vivid_elegoo_tag()]
        assert rig.unit.read_slot(0) is None
        assert rig.field.calls == []
        assert rig.logger.messages == [
            ("warning", "AFC_Vivid_rfid: no reader configured for slot 0")]

    def test_read_slot_passes_configured_brand_keys(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={
            "bambu_master_key": "00112233445566778899aabbccddeeff",
            "creality_key": "0102", "creality_encryption_key": "0304"})
        assert rig.unit.read_slot(0) is None
        _link, kwargs = rig.field.calls[0]
        assert kwargs["bambu_master_key"] == bytes(
            [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
             0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF])
        assert kwargs["creality_key"] == b"\x01\x02"
        assert kwargs["creality_encryption_key"] == b"\x03\x04"
        # Nothing read, so nothing remembered.
        assert rig.unit._slot_uid == {}
        assert rig.logger.messages == []

    def test_read_slot_excludes_known_sibling_tag(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        # Slot 1 already read a spool; its UID must be halted when reading slot 0.
        rig.unit._slot_uid[1] = "AABBCCDD"
        rig.field.tags = [{"uid": "AABBCCDD", "filament": {"type": "PLA"}}]
        assert rig.unit.read_slot(0) is None
        ex = rig.field.calls[0][1]["is_excluded"]
        assert ex("aabbccdd") is True
        assert ex("11223344") is False
        assert rig.unit._slot_uid == {1: "AABBCCDD"}
        assert rig.logger.messages == []

    def test_read_slot_no_excluder_without_sibling_uid(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.unit.read_slot(0)                       # sibling slot 1 unknown
        assert rig.field.calls[0][1]["is_excluded"] is None
        assert rig.logger.messages == []

    def test_read_slot_remembers_uid_for_future_dedup(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.field.tags = [{"uid": "DEADBEEF", "filament": {"type": "PLA"}}]
        rig.unit.read_slot(0)
        assert rig.unit._slot_uid == {0: "DEADBEEF"}
        # Reading the sibling slot 1 now halts slot 0's remembered tag.
        ex = rig.unit._sibling_excluder(1)
        assert ex is not None and ex("deadbeef") is True
        assert rig.logger.messages == []


class TestAFCVividrfidMap:
    def test_map_single_color_btt(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        assert rig.unit._map(vivid_btt_tag()) == {
            "material": "PET", "color_hex": "c0ffee", "multi_color": ["c0ffee"],
            "is_dual_color": False, "sku": "", "brand": "BQ Tech",
            "sub_type": "PET (CEP)", "diameter": 1.75, "extruder_temp": 220,
            "bed_temp": 60, "mfg_date": "", "uid": "d13fdb0e", "weight_g": None,
            "extruder_temp_min": 200, "extruder_temp_max": 240,
            "tag_type": "MifareClassic1k"}
        assert rig.logger.messages == []

    def test_map_dual_color_bambu(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        si = rig.unit._map({"uid": "7bf0afff",
                            "filament": {"manufacturer": "Bambu", "type": "PLA",
                                         "color_argb": 0xFFE7C1D5,
                                         "colors_argb": [0xFFE7C1D5, 0xFF8EC9E9]}})
        assert si["color_hex"] == "e7c1d5"
        assert si["multi_color"] == ["e7c1d5", "8ec9e9"]
        assert si["is_dual_color"] is True
        assert si["brand"] == "Bambu" and si["material"] == "PLA"
        assert rig.logger.messages == []

    def test_map_empty_when_no_filament(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        assert rig.unit._map({"uid": "aa", "filament": None}) == {
            "material": "", "color_hex": "", "multi_color": [],
            "is_dual_color": False, "sku": "", "brand": "", "sub_type": "",
            "diameter": 1.75, "extruder_temp": None, "bed_temp": None,
            "mfg_date": "", "uid": "aa", "weight_g": None}
        assert rig.logger.messages == []


class TestAFCVividrfidReadLane:
    @staticmethod
    def _elegoo_info() -> Dict[str, Any]:
        """:return dict: the slot_info vivid_elegoo_tag() maps to, by hand"""
        return {"material": "PLA", "color_hex": "", "multi_color": [],
                "is_dual_color": False, "sku": "", "brand": "Elegoo",
                "sub_type": "", "diameter": 1.75, "extruder_temp": None,
                "bed_temp": None, "mfg_date": "", "uid": "AABB", "weight_g": None,
                "tag_type": "MifareClassic1k"}

    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch,
             lanes: Sequence[VividLane] = ()) -> VividRig:
        """
        :param monkeypatch: for build_vivid
        :param lanes: AFC lanes to register
        :return VividRig: lane0 mapped to slot 0 on reader0
        """
        return build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                           lanes=lanes)

    def test_read_lane_decoded_records_slot_info(self, monkeypatch):
        rig = self._rig(monkeypatch)
        tag = vivid_elegoo_tag()
        rig.field.tags = [tag]
        assert rig.unit.read_lane("lane0") == self._elegoo_info()
        assert rig.unit._last == {0: tag}
        assert rig.unit.get_status()["last_reads"] == {"lane0": vivid_elegoo_record()}
        assert rig.console == []
        assert rig.logger.messages == []

    def test_undecoded_tag_recorded_with_uid(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.field.tags = [{"uid": "AABB", "tag_type": "MifareClassic1k",
                           "filament": None}]
        assert rig.unit.read_lane("lane0") is None
        assert rig.unit._last == {}
        assert rig.unit.get_status()["last_reads"] == {"lane0": {
            "uid": "AABB", "tag_type": "MifareClassic1k", "decoded": False,
            "scan_time": VIVID_WALL_TIME}}
        assert rig.console == []
        assert rig.logger.messages == []

    def test_no_tag_at_all_records_nothing(self, monkeypatch):
        rig = self._rig(monkeypatch)
        assert rig.unit.read_lane("lane0") is None
        assert len(rig.field.calls) == 1
        assert rig.unit.get_status()["last_reads"] == {}
        assert rig.console == []
        assert rig.logger.messages == []

    def test_warns_when_lane_unmapped(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.field.tags = [vivid_elegoo_tag()]
        assert rig.unit.read_lane("ghost") is None
        assert rig.field.calls == []
        assert rig.unit._last == {}
        assert rig.unit.last_reads_status() == {}
        assert rig.console == []
        assert rig.logger.messages == [
            ("warning", "AFC_Vivid_rfid: lane 'ghost' has no slot (set lane_slot_map)")]

    def test_records_when_afc_none(self, monkeypatch):
        lane0 = VividLane("lane0")
        rig = self._rig(monkeypatch, lanes=[lane0])
        rig.unit.afc = None
        tag = vivid_elegoo_tag()
        rig.field.tags = [tag]
        assert rig.unit.read_lane("lane0") == self._elegoo_info()
        assert rig.unit._last == {0: tag}
        assert rig.unit.get_status()["last_reads"] == {"lane0": vivid_elegoo_record()}
        # Not applied: the registered lane is untouched and nothing is printed.
        assert lane0.material is None
        assert rig.console == []
        assert rig.logger.messages == []

    def test_records_when_afc_has_no_lanes(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit.afc = VividBareAfc()
        tag = vivid_elegoo_tag()
        rig.field.tags = [tag]
        assert rig.unit.read_lane("lane0") == self._elegoo_info()
        assert rig.unit._last == {0: tag}
        assert rig.unit.get_status()["last_reads"] == {"lane0": vivid_elegoo_record()}
        assert rig.console == []
        assert rig.logger.messages == []

    def test_records_when_lane_not_in_afc(self, monkeypatch):
        lane1 = VividLane("lane1")
        rig = self._rig(monkeypatch, lanes=[lane1])
        tag = vivid_elegoo_tag()
        rig.field.tags = [tag]
        assert rig.unit.read_lane("lane0") == self._elegoo_info()
        assert rig.unit._last == {0: tag}
        assert rig.unit.get_status()["last_reads"] == {"lane0": vivid_elegoo_record()}
        assert lane1.material is None
        assert rig.console == []
        assert rig.logger.messages == []

    def test_applies_when_lane_present(self, monkeypatch):
        lane0 = VividLane("lane0")
        rig = self._rig(monkeypatch, lanes=[lane0])
        tag = vivid_elegoo_tag()
        rig.field.tags = [tag]
        # Spy that still runs the real apply, to prove its result is returned.
        real = rig.unit.apply_to_lane
        returned: List[Dict[str, Any]] = []
        monkeypatch.setattr(rig.unit, "apply_to_lane",
                            lambda lane, t: returned.append(real(lane, t)) or returned[-1])
        result = rig.unit.read_lane("lane0")
        assert len(returned) == 1 and result is returned[0]
        assert result == self._elegoo_info()
        assert rig.unit._last == {0: tag}
        # apply_to_lane filled the lane and printed the read-out.
        assert (lane0.material, lane0.spool_vendor, lane0.bed_temp,
                lane0.weight) == ("PLA", "Elegoo", 60.0, 1000)
        assert rig.console == [VIVID_ELEGOO_READ_OUT]
        assert rig.unit.get_status()["last_reads"] == {"lane0": vivid_elegoo_record()}
        assert rig.logger.messages == []


class TestAFCVividrfidStageReadBegin:
    def test_stage_read_begin_starts_poll_for_mapped_lane(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                          lanes=[VividLane("lane0")])
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.unit._probe == vivid_probe()
        timer = rig.unit._poll_timer
        assert rig.printer.reactor.timers == [timer]
        assert timer.callback == rig.unit._stage_poll
        assert timer.waketime == pytest.approx(100.15)
        assert rig.antenna.activations == []
        assert rig.logger.messages == []

    def test_stage_read_begin_ignores_lane_without_reader(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                          readers=(), lanes=[VividLane("lane0")])
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.unit._probe is None and rig.unit._poll_timer is None
        assert rig.printer.reactor.timers == []
        assert rig.logger.messages == []

    def test_stage_read_begin_disabled_by_config(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0",
                                                "stage_read": False},
                          lanes=[VividLane("lane0")])
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.unit._probe is None and rig.unit._poll_timer is None
        assert rig.printer.reactor.timers == []
        assert rig.logger.messages == []

    def test_lane_without_a_name_is_ignored(self, monkeypatch):
        # ":0" maps the empty name to slot 0, so only the name guard stops it.
        rig = build_vivid(monkeypatch, options={"lane_slot_map": ":0"})
        assert rig.unit._lane_slot == {"": 0}
        rig.unit._stage_read_begin(VividLane(""))
        assert rig.unit._probe is None
        assert rig.printer.reactor.timers == []
        assert rig.logger.messages == []

    def test_stage_sibling_left_alone_when_antenna_clear(self, monkeypatch):
        # Sibling has a spool, but no tag is parked on the shared reader.
        rig = vivid_pair_rig(monkeypatch)
        rig.unit._slot_uid[0] = "CAFE"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.antenna.activations == [(rig.readers["reader0"].link, None)]
        assert rig.lane("lane1").moves == []
        assert rig.unit._probe == vivid_probe()
        assert rig.logger.messages == []

    def test_stage_own_parked_tag_does_not_move_the_sibling(self, monkeypatch):
        # The shared antenna works both ways: this slot's own stationary tag
        # can be the one on the coil, and must not move the sibling.
        rig = vivid_pair_rig(monkeypatch)
        rig.unit._slot_uid[0] = "CAFE"
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.lane("lane1").moves == []
        assert rig.unit._probe == vivid_probe()
        assert rig.logger.messages == [
            ("info", "ViViD RFID: tag cafe on the shared reader for slot 0 is this "
                     "slot's own, no collision, leaving the sibling alone")]

    def test_stage_an_unknown_parked_tag_is_still_cleared(self, monkeypatch):
        # No history to attribute it by, so it stays a collision.
        rig = vivid_pair_rig(monkeypatch)
        rig.antenna.uid = b"\xde\xad\xbe\xef"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        lane1 = rig.lane("lane1")
        assert lane1.moves == vivid_retract_moves(*VIVID_FULL_RETRACT)
        assert rig.unit._probe == vivid_probe(sib_lane=lane1, sib_dist=75.0)
        assert rig.logger.messages == [
            ("info", "ViViD RFID: tag deadbeef parked on the shared reader for slot 0, "
                     "clearing the sibling before the read"),
            VIVID_RETRACTING]

    def test_movable_sibling_no_baseline(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit._slot_uid.update({0: "BEEF", 1: "CAFE"})
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        lane1 = rig.lane("lane1")
        assert lane1.pos == 425.0
        assert rig.unit._probe == vivid_probe(sib_lane=lane1, sib_dist=75.0)
        assert rig.logger.messages == [VIVID_PARKED_CAFE, VIVID_RETRACTING]

    def test_stage_sibling_cleared_when_its_tag_parked(self, monkeypatch):
        # The sibling's own tag is parked and it cannot move: exclude it instead.
        rig = vivid_pair_rig(monkeypatch, options={"auto_tag_adjust": False})
        rig.unit._slot_uid.update({0: "BEEF", 1: "CAFE"})
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.lane("lane1").moves == []
        assert rig.unit._probe == vivid_probe(baseline="cafe", blocked_sib="lane1")
        assert rig.logger.messages == [VIVID_PARKED_CAFE]

    def test_parked_unmovable_sibling_sets_baseline_and_logs(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch,
                             sibling=VividLane("lane1", tool_loaded=True))
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.lane("lane1").moves == []
        assert rig.unit._probe == vivid_probe(baseline="cafe", blocked_sib="lane1")
        assert rig.logger.messages == [
            VIVID_PARKED_CAFE, VIVID_REFUSED]

    def test_no_sibling_lane_skips_precheck(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                          lanes=[VividLane("lane0"), VividLane("lane1")])
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.antenna.activations == []
        assert rig.unit._probe == vivid_probe()
        assert rig.printer.reactor.timers == [rig.unit._poll_timer]
        assert rig.logger.messages == []

    def test_single_slot_reader_skips_precheck(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:5, lane1:6"},
                          readers=(("reader0", "5"),),
                          lanes=[VividLane("lane0"), VividLane("lane1")])
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.antenna.activations == []
        assert rig.unit._probe == vivid_probe(slot=5)
        assert rig.logger.messages == []

    def test_sibling_unknown_without_afc_skips_precheck(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit.afc = None
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.antenna.activations == []
        assert rig.unit._probe == vivid_probe()
        assert rig.logger.messages == []

    def test_sibling_unknown_when_afc_has_no_lanes_skips_precheck(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit.afc = VividBareAfc()
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.antenna.activations == []
        assert rig.unit._probe == vivid_probe()
        assert rig.logger.messages == []

    def test_empty_sibling_skips_precheck(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch,
                             sibling=VividLane("lane1", prep_state=False))
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.antenna.activations == []
        assert rig.unit._probe == vivid_probe()
        assert rig.logger.messages == []

    def test_precheck_exception_warns(self, monkeypatch):
        sibling = VividLane("lane1")
        sibling.fault = RuntimeError("boom")
        rig = vivid_pair_rig(monkeypatch, sibling=sibling)
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert sibling.moves == vivid_retract_moves(10.0)
        assert rig.unit._probe == vivid_probe()
        assert rig.printer.reactor.timers == [rig.unit._poll_timer]
        assert rig.logger.messages == [
            VIVID_PARKED_CAFE, VIVID_RETRACTING,
            ("warning", "ViViD RFID: sibling pre-check failed: boom")]

    def test_timer_register_failure_clears_probe(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                          lanes=[VividLane("lane0")])
        monkeypatch.setattr(rig.printer.reactor, "register_timer",
                            Recorder(raises=RuntimeError("boom")))
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.unit._probe is None and rig.unit._poll_timer is None
        assert rig.logger.messages == [
            ("warning", "ViViD RFID: could not start stage poll: boom")]

    def test_timer_register_failure_restores_a_moved_sibling(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.antenna.uid = b"\xca\xfe"
        monkeypatch.setattr(rig.printer.reactor, "register_timer",
                            Recorder(raises=RuntimeError("boom")))
        rig.unit._stage_read_begin(rig.lane("lane0"))
        lane1 = rig.lane("lane1")
        assert lane1.moves == vivid_retract_moves(*VIVID_FULL_RETRACT) + [
            (75.0, SpeedMode.SHORT, None, AssistActive.NO, False)]
        assert lane1.pos == 500.0
        assert rig.unit._probe is None
        assert rig.logger.messages == [
            VIVID_PARKED_CAFE, VIVID_RETRACTING,
            ("warning", "ViViD RFID: could not start stage poll: boom")]


class TestAFCVividrfidStagePoll:
    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch,
             options: Optional[Dict[str, Any]] = None
             ) -> Tuple[VividRig, VividTriggerCmd]:
        """
        lane0 on slot 0, its load sensor on a trsync the poll can trigger,
        with a stage read begun.

        :param monkeypatch: for build_vivid
        :param options: coordinator options over a 0.2s poll interval
        :return tuple: the rig and the trsync's trigger command
        """
        values: Dict[str, Any] = {"lane_slot_map": "lane0:0",
                                  "stage_poll_interval": 0.2}
        values.update(options or {})
        rig = build_vivid(monkeypatch, options=values,
                          lanes=[VividLane("lane0", load_endstop_name="load_es0")])
        cmd = add_vivid_endstop(rig.printer)
        rig.unit._stage_read_begin(rig.lane("lane0"))
        return rig, cmd

    def test_stage_poll_detects_aborts_reads_and_applies(self, monkeypatch):
        rig, cmd = self._rig(monkeypatch)
        rig.antenna.uid = b"\xaa\xbb"
        tag = vivid_elegoo_tag()
        rig.field.tags = [tag]
        assert rig.unit._stage_poll(100.2) == rig.printer.reactor.NEVER
        assert cmd.sent == [[7, 3]]                    # the feed was stopped
        assert len(rig.field.calls) == 2               # read, then confirmed
        assert rig.unit._probe == vivid_probe(done=True, read_ok=True, aborts=1)
        assert rig.unit._last == {0: tag}
        assert rig.lane("lane0").material == "PLA"
        assert rig.console == [VIVID_ELEGOO_READ_OUT]
        assert rig.logger.messages == []

    def test_stage_poll_no_tag_in_range_keeps_polling(self, monkeypatch):
        rig, cmd = self._rig(monkeypatch)
        assert rig.unit._stage_poll(1.0) == pytest.approx(1.2)
        assert cmd.sent == []
        assert rig.field.calls == []
        assert rig.unit._probe == vivid_probe()
        assert rig.console == []
        assert rig.logger.messages == []

    def test_stage_poll_gives_up_after_max_aborts(self, monkeypatch):
        rig, cmd = self._rig(monkeypatch, options={"stage_max_aborts": 2})
        rig.antenna.uid = b"\xaa"
        rig.field.tags = [{"uid": "AA", "filament": None}]   # never decodes
        assert rig.unit._stage_poll(1.0) == pytest.approx(1.2)
        assert rig.unit._probe == vivid_probe(aborts=1)
        assert rig.logger.messages == []
        assert rig.unit._stage_poll(1.2) == rig.printer.reactor.NEVER
        assert rig.unit._probe == vivid_probe(aborts=2, done=True)
        assert cmd.sent == [[7, 3], [7, 3]]
        assert rig.console == []
        assert rig.logger.messages == [
            ("info", "ViViD RFID: gave up reading lane0 after 2 attempts")]

    def test_stage_poll_skips_parked_baseline_sibling(self, monkeypatch):
        # The unmovable parked sibling's UID never stops the feed.
        rig, cmd = self._rig(monkeypatch)
        rig.unit._probe["baseline"] = "cafe"
        rig.antenna.uid = b"\xca\xfe"
        assert rig.unit._stage_poll(1.0) == pytest.approx(1.2)
        assert cmd.sent == []
        assert rig.field.calls == []
        assert rig.unit._probe == vivid_probe(baseline="cafe")
        assert rig.console == []
        assert rig.logger.messages == []

    def test_returns_never_when_no_probe(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.antenna.uid = b"\xaa"
        assert rig.unit._stage_poll(1.0) == rig.printer.reactor.NEVER
        assert rig.antenna.activations == []
        assert rig.console == []
        assert rig.logger.messages == []

    def test_returns_never_when_probe_done(self, monkeypatch):
        rig, cmd = self._rig(monkeypatch)
        rig.unit._probe["done"] = True
        rig.antenna.uid = b"\xaa"
        assert rig.unit._stage_poll(1.0) == rig.printer.reactor.NEVER
        assert rig.antenna.activations == []
        assert cmd.sent == []
        assert rig.console == []
        assert rig.logger.messages == []

    def test_detect_exception_warns_and_keeps_polling(self, monkeypatch):
        rig, cmd = self._rig(monkeypatch)
        rig.antenna.raises = RuntimeError("boom")
        assert rig.unit._stage_poll(1.0) == pytest.approx(1.2)
        assert rig.unit._probe == vivid_probe()
        assert cmd.sent == []
        assert rig.console == []
        assert rig.logger.messages == [
            ("warning", "ViViD RFID: detect error on slot 0: boom")]


class TestAFCVividrfidDetectUid:
    def test_none_when_no_reader(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.antenna.uid = b"\xaa\xbb"
        assert rig.unit._detect_uid(5) is None
        assert rig.antenna.activations == []
        assert rig.logger.messages == []

    def test_returns_hex_when_tag_present(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.antenna.uid = b"\xaa\xbb"
        assert rig.unit._detect_uid(0) == "aabb"
        assert rig.antenna.activations == [(rig.readers["reader0"].link, None)]
        assert rig.logger.messages == []

    def test_none_when_no_uid(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        assert rig.unit._detect_uid(0) is None
        assert len(rig.antenna.activations) == 1
        assert rig.logger.messages == []

    def test_passes_sibling_excluder(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.unit._slot_uid[1] = "DEAD"
        rig.antenna.uid = b"\xaa"
        assert rig.unit._detect_uid(0) == "aa"
        ex = rig.antenna.activations[0][1]
        assert ex("dead") is True
        assert ex("aa") is False
        # The sibling's own tag on the coil is halted, so nothing is detected.
        rig.antenna.uid = b"\xde\xad"
        assert rig.unit._detect_uid(0) is None
        assert rig.logger.messages == []


class TestAFCVividrfidReadConfirmed:
    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch, confirm: int) -> VividRig:
        """
        :param monkeypatch: for build_vivid
        :param confirm: stage_confirm_reads
        :return VividRig: reader0 serving slots 0 and 1
        """
        return build_vivid(monkeypatch, options={"stage_confirm_reads": confirm})

    def test_read_slot_exception_warns_and_none(self, monkeypatch):
        rig = self._rig(monkeypatch, confirm=2)
        rig.field.raises = RuntimeError("boom")
        assert rig.unit._read_confirmed(0) is None
        assert len(rig.field.calls) == 1
        assert rig.logger.messages == [
            ("warning", "ViViD RFID: stage read error on slot 0: boom")]

    def test_none_when_no_tag(self, monkeypatch):
        rig = self._rig(monkeypatch, confirm=2)
        assert rig.unit._read_confirmed(0) is None
        assert len(rig.field.calls) == 1
        assert rig.logger.messages == []

    def test_none_when_no_filament(self, monkeypatch):
        rig = self._rig(monkeypatch, confirm=2)
        rig.field.tags = [{"uid": "A", "filament": None}]
        assert rig.unit._read_confirmed(0) is None
        assert len(rig.field.calls) == 1
        assert rig.logger.messages == []

    def test_none_when_uid_inconsistent(self, monkeypatch):
        rig = self._rig(monkeypatch, confirm=2)
        rig.field.tags = [{"uid": "AA", "filament": {"type": "PLA"}},
                          {"uid": "BB", "filament": {"type": "PLA"}}]
        assert rig.unit._read_confirmed(0) is None
        assert len(rig.field.calls) == 2
        assert rig.logger.messages == []

    def test_returns_tag_when_consistent(self, monkeypatch):
        rig = self._rig(monkeypatch, confirm=2)
        first = {"uid": "AA", "filament": {"type": "PLA"}}
        second = {"uid": "AA", "filament": {"type": "PLA"}}
        rig.field.tags = [first, second]
        assert rig.unit._read_confirmed(0) is second
        assert len(rig.field.calls) == 2
        assert rig.logger.messages == []

    def test_passes_baseline_as_extra_excluded(self, monkeypatch):
        rig = self._rig(monkeypatch, confirm=1)
        tag = {"uid": "AA", "filament": {"type": "PLA"}}
        rig.field.tags = [tag]
        assert rig.unit._read_confirmed(0, baseline="BASE") is tag
        ex = rig.field.calls[0][1]["is_excluded"]
        assert ex("base") is True
        assert ex("aa") is False
        assert rig.logger.messages == []


class TestAFCVividrfidAbortFeed:
    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch,
             endstop: Optional[str] = "load_es0") -> VividRig:
        """
        :param monkeypatch: for build_vivid
        :param endstop: lane0's load endstop name
        :return VividRig: lane0 on slot 0, registered with AFC
        """
        lane0 = VividLane("lane0", load_endstop_name=endstop)
        return build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                           lanes=[lane0])

    def test_noop_when_afc_none(self, monkeypatch):
        rig = self._rig(monkeypatch)
        cmd = add_vivid_endstop(rig.printer)
        rig.unit.afc = None
        rig.unit._abort_feed("lane0")
        assert cmd.sent == []
        assert rig.logger.messages == []

    def test_noop_when_afc_has_no_lanes(self, monkeypatch):
        rig = self._rig(monkeypatch)
        cmd = add_vivid_endstop(rig.printer)
        rig.unit.afc = VividBareAfc()
        rig.unit._abort_feed("lane0")
        assert cmd.sent == []
        assert rig.logger.messages == []

    def test_noop_when_lane_missing_endstop_name(self, monkeypatch):
        rig = self._rig(monkeypatch, endstop=None)
        cmd = add_vivid_endstop(rig.printer)
        rig.unit._abort_feed("lane0")
        assert cmd.sent == []
        assert rig.logger.messages == []

    def test_returns_when_query_endstops_absent(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit._abort_feed("lane0")
        assert rig.logger.messages == []

    def test_returns_when_endstop_not_matched(self, monkeypatch):
        rig = self._rig(monkeypatch)
        cmd = add_vivid_endstop(rig.printer, name="other")
        rig.unit._abort_feed("lane0")
        assert cmd.sent == []
        assert rig.logger.messages == []

    def test_returns_when_dispatch_none(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.printer.add_object("query_endstops", VividQueryEndstops(
            [(VividMcuEndstop(None), "load_es0")]))
        rig.unit._abort_feed("lane0")
        assert rig.logger.messages == []

    def test_returns_when_trsyncs_empty(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.printer.add_object("query_endstops", VividQueryEndstops(
            [(VividMcuEndstop(VividDispatch([])), "load_es0")]))
        rig.unit._abort_feed("lane0")
        assert rig.logger.messages == []

    def test_success_sends_trigger(self, monkeypatch):
        rig = self._rig(monkeypatch)
        cmd = add_vivid_endstop(rig.printer)
        rig.unit._abort_feed("lane0")
        assert cmd.sent == [[7, 3]]                    # oid, host request
        assert rig.logger.messages == []

    def test_send_exception_logs_debug(self, monkeypatch):
        rig = self._rig(monkeypatch)
        add_vivid_endstop(rig.printer, cmd=VividTriggerCmd(raises=RuntimeError("boom")))
        rig.unit._abort_feed("lane0")
        assert rig.logger.messages == [
            ("debug", "ViViD RFID: fake-trigger of load_es0 not available (boom), "
                      "feeding to the real sensor instead")]


class TestAFCVividrfidApplyStaged:
    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch, lane: str = "lane0") -> VividRig:
        """
        :param monkeypatch: for build_vivid
        :param lane: the AFC lane to register
        :return VividRig: lane0 mapped to slot 0
        """
        return build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                           lanes=[VividLane(lane)])

    def test_caches_without_apply_when_afc_none(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit.afc = None
        tag = vivid_elegoo_tag()
        rig.unit._apply_staged(vivid_probe(), tag)
        assert rig.unit._last == {0: tag}
        assert rig.lane("lane0").material is None
        assert rig.unit.last_reads_status() == {}
        assert rig.console == []
        assert rig.logger.messages == []

    def test_caches_without_apply_when_no_lanes_attr(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.unit.afc = VividBareAfc()
        tag = vivid_elegoo_tag()
        rig.unit._apply_staged(vivid_probe(), tag)
        assert rig.unit._last == {0: tag}
        assert rig.lane("lane0").material is None
        assert rig.unit.last_reads_status() == {}
        assert rig.console == []
        assert rig.logger.messages == []

    def test_skips_apply_when_lane_not_found(self, monkeypatch):
        rig = self._rig(monkeypatch, lane="lane1")
        tag = vivid_elegoo_tag()
        rig.unit._apply_staged(vivid_probe(), tag)
        assert rig.unit._last == {0: tag}
        assert rig.lane("lane1").material is None
        assert rig.unit.last_reads_status() == {}
        assert rig.console == []
        assert rig.logger.messages == []

    def test_applies_when_lane_present(self, monkeypatch):
        rig = self._rig(monkeypatch)
        tag = vivid_elegoo_tag()
        rig.unit._apply_staged(vivid_probe(), tag)
        assert rig.unit._last == {0: tag}
        assert rig.lane("lane0").material == "PLA"
        assert rig.unit.last_reads_status() == {"lane0": vivid_elegoo_record()}
        assert rig.console == [VIVID_ELEGOO_READ_OUT]
        assert rig.logger.messages == []

    def test_warns_when_apply_raises(self, monkeypatch):
        rig = self._rig(monkeypatch)
        tag = {"uid": "AA", "filament": "PLA"}         # a malformed decode
        rig.unit._apply_staged(vivid_probe(), tag)
        assert rig.unit._last == {0: tag}
        assert rig.lane("lane0").material is None
        assert rig.console == []
        assert rig.unit.last_reads_status() == {}
        assert rig.logger.messages == [
            ("warning", "ViViD RFID: applying lane0 read failed: "
                        "'str' object has no attribute 'get'")]


class TestAFCVividrfidRetractSibling:
    class _StallsOnHop(VividLane):
        """lane1, whose move_to raises on one hop only."""

        def __init__(self, stall_on: int) -> None:
            """
            :param stall_on: the 1-based move that raises
            """
            super().__init__("lane1")
            self._stall_on = stall_on

        def move_to(self, distance: float, speed_mode: Any,
                    endstop: Optional[str] = None, assist_active: Any = None,
                    use_homing: bool = True) -> Tuple[bool, float, None]:
            """
            :param distance: signed mm
            :param speed_mode: AFC's speed mode
            :param endstop: the endstop to home to
            :param assist_active: the espooler mode
            :param use_homing: home on the endstop
            :return tuple: (homed, distance, None)
            """
            stall = len(self.moves) + 1 == self._stall_on
            self.fault = RuntimeError("stall") if stall else None
            return super().move_to(distance, speed_mode, endstop, assist_active,
                                   use_homing)

    def test_a_retract_that_fails_midway_is_still_restored(self, monkeypatch):
        sibling = self._StallsOnHop(3)
        rig = vivid_pair_rig(monkeypatch, sibling=sibling)
        p: Dict[str, Any] = {"slot": 0}
        with pytest.raises(RuntimeError, match="stall"):
            rig.unit._retract_sibling(p)
        # Two hops landed before the third raised; the probe knows about them.
        assert sibling.moves == vivid_retract_moves(10.0, 10.0, 10.0)
        assert sibling.pos == 480.0
        assert p == {"slot": 0, "sib_lane": sibling, "sib_dist": 20.0}
        rig.unit._restore_sibling(p)
        assert sibling.moves[3:] == [
            (20.0, SpeedMode.SHORT, None, AssistActive.NO, False)]
        assert sibling.pos == 500.0
        assert rig.logger.messages == [VIVID_RETRACTING]

    def test_retracts_idle_sibling_and_records(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        lane1 = rig.lane("lane1")
        # Stepped, so the distance is a maximum the load switch can cut short.
        assert lane1.moves == vivid_retract_moves(*VIVID_FULL_RETRACT)
        assert lane1.pos == 425.0
        assert p == {"slot": 0, "sib_lane": lane1, "sib_dist": 75.0}
        assert rig.logger.messages == [VIVID_RETRACTING]

    def test_an_unseated_sibling_is_never_retracted(self, monkeypatch):
        # prep_state alone is a hand on the switch, not a seated tip.
        rig = vivid_pair_rig(monkeypatch, sibling=VividLane("lane1", pos=-5.0))
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 0}
        assert rig.logger.messages == [
            ("info", "ViViD RFID: sibling lane1 is not on its load switch, not moving "
                     "it, since there may be nothing to give back")]

    def test_the_retract_stops_at_the_load_switch(self, monkeypatch):
        # Only 25mm past the switch: the 75mm ask must not be a commitment.
        rig = vivid_pair_rig(monkeypatch, sibling=VividLane("lane1", pos=25.0))
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        lane1 = rig.lane("lane1")
        assert lane1.moves == vivid_retract_moves(10.0, 10.0, 10.0)
        assert lane1.pos == -5.0                    # one step past at most
        assert p == {"slot": 0, "sib_lane": lane1, "sib_dist": 30.0}
        assert rig.logger.messages == [
            VIVID_RETRACTING,
            ("info", "ViViD RFID: sibling lane1 stopped early at 30mm, its load "
                     "switch was about to release")]

    def test_noop_when_disabled(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch, options={"auto_tag_adjust": False})
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 0}
        assert rig.logger.messages == []

    def test_noop_when_movedirection_none(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        monkeypatch.setattr(vivid_mod, "MoveDirection", None)
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 0}
        assert rig.logger.messages == []

    def test_return_when_no_sibling(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:5, lane1:6"},
                          readers=(("reader0", "5"),),
                          lanes=[VividLane("lane0"), VividLane("lane1")])
        p: Dict[str, Any] = {"slot": 5}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 5}
        assert rig.logger.messages == []

    def test_return_when_afc_none(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit.afc = None
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 0}
        assert rig.logger.messages == []

    def test_return_when_afc_no_lanes(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit.afc = VividBareAfc()
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 0}
        assert rig.logger.messages == []

    def test_return_when_sibling_empty(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch,
                             sibling=VividLane("lane1", prep_state=False))
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 0}
        assert rig.logger.messages == []

    def test_skips_loaded_sibling_and_logs(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch,
                             sibling=VividLane("lane1", tool_loaded=True))
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 0}
        assert rig.logger.messages == [VIVID_REFUSED]

    def test_skips_printing_sibling(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.printer.set_print_state("printing")
        p: Dict[str, Any] = {"slot": 0}
        rig.unit._retract_sibling(p)
        assert rig.lane("lane1").moves == []
        assert p == {"slot": 0}
        assert rig.logger.messages == [VIVID_REFUSED]


class TestAFCVividrfidRestoreSibling:
    #: The unhomed give-back of a 30mm retract.
    GIVE_BACK_30: VividMove = (30.0, SpeedMode.SHORT, None, AssistActive.NO, False)
    #: One 10mm creep step towards the load switch.
    CREEP: VividMove = (10.0, SpeedMode.SHORT, None, AssistActive.NO, False)
    NOT_BACK: LogLine = (
        "error", "ViViD RFID: sibling lane1 is NOT back on its load switch after the "
                 "read; its filament may have come out of the lane. Re-seat it and "
                 "check the spool.")
    HOMED: LogLine = (
        "info", "ViViD RFID: homed sibling lane1 forward onto its load switch after "
                "the give-back landed short")

    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch, sibling: VividLane,
             homing: bool = True) -> VividRig:
        """
        :param monkeypatch: for build_vivid
        :param sibling: lane1, the sibling to put back
        :param homing: AFC's homing_enabled
        :return VividRig: the lane pair on reader0
        """
        rig = vivid_pair_rig(monkeypatch, sibling=sibling)
        rig.printer.afc.homing_enabled = homing
        return rig

    def test_the_restore_is_one_homing_move_onto_the_load_switch(self, monkeypatch):
        # Half of every feed slips, so the give-back lands 15mm short and AFC's
        # own homing move puts the tip back on the switch.
        sib = VividLane("lane1", pos=-20.0, slip=0.5)
        rig = self._rig(monkeypatch, sib)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 30.0}
        rig.unit._restore_sibling(p)
        assert sib.moves == [self.GIVE_BACK_30,
                             (50.0, SpeedMode.LONG, "load", AssistActive.DYNAMIC, True)]
        assert sib.pos == 0.5
        assert p == {"slot": 0, "sib_lane": None, "sib_dist": 0.0}
        assert rig.logger.messages == [self.HOMED]

    def test_without_homing_it_falls_back_to_the_stepped_creep(self, monkeypatch):
        sib = VividLane("lane1", pos=-20.0, slip=0.5)
        rig = self._rig(monkeypatch, sib, homing=False)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 30.0}
        rig.unit._restore_sibling(p)
        assert sib.moves == [self.GIVE_BACK_30, self.CREEP, self.CREEP]
        assert sib.pos == 5.0
        assert p == {"slot": 0, "sib_lane": None, "sib_dist": 0.0}
        assert rig.logger.messages == [
            ("info", "ViViD RFID: fed sibling lane1 a further 20mm to put it back on "
                     "its load switch")]

    def test_a_sibling_that_will_not_come_back_is_said_out_loud(self, monkeypatch):
        sib = VividLane("lane1", pos=-20.0, slip=1.0)   # feeds achieve nothing
        rig = self._rig(monkeypatch, sib)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 30.0}
        rig.unit._restore_sibling(p)
        assert sib.moves == [self.GIVE_BACK_30,
                             (50.0, SpeedMode.LONG, "load", AssistActive.DYNAMIC, True)]
        assert sib.pos == -20.0
        # The home is checked: no success line, a warning, then the error.
        assert rig.logger.messages == [
            ("warning", "ViViD RFID: homing sibling lane1 forward after the give-back "
                        "landed short did not reach its load switch"),
            self.NOT_BACK]

    def test_a_creep_that_never_lands_is_said_out_loud(self, monkeypatch):
        sib = VividLane("lane1", pos=-20.0, slip=1.0)
        rig = self._rig(monkeypatch, sib, homing=False)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 30.0}
        rig.unit._restore_sibling(p)
        # int(30 / 10) + 4 creep steps, then it gives up.
        assert sib.moves == [self.GIVE_BACK_30] + [self.CREEP] * 7
        assert rig.logger.messages == [
            ("info", "ViViD RFID: fed sibling lane1 a further 70mm to put it back on "
                     "its load switch"),
            self.NOT_BACK]

    def test_restoring_twice_only_moves_once(self, monkeypatch):
        sib = VividLane("lane1", pos=470.0)
        rig = self._rig(monkeypatch, sib)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 30.0}
        rig.unit._restore_sibling(p)
        rig.unit._restore_sibling(p)
        assert sib.moves == [self.GIVE_BACK_30]
        assert sib.pos == 500.0
        assert rig.logger.messages == []

    def test_noop_when_probe_none(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.unit._restore_sibling(None)
        rig.unit._restore_sibling({})
        assert rig.lane("lane1").moves == []
        assert rig.logger.messages == []

    def test_noop_when_no_sib_lane(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": None, "sib_dist": 75.0}
        rig.unit._restore_sibling(p)
        assert p == {"slot": 0, "sib_lane": None, "sib_dist": 75.0}
        assert rig.logger.messages == []

    def test_noop_when_dist_zero(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        sib = rig.lane("lane1")
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 0.0}
        rig.unit._restore_sibling(p)
        assert sib.moves == []
        assert p == {"slot": 0, "sib_lane": sib, "sib_dist": 0.0}
        assert rig.logger.messages == []

    def test_noop_when_movedirection_none(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        monkeypatch.setattr(vivid_mod, "MoveDirection", None)
        sib = rig.lane("lane1")
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 75.0}
        rig.unit._restore_sibling(p)
        assert sib.moves == []
        assert p == {"slot": 0, "sib_lane": sib, "sib_dist": 75.0}
        assert rig.logger.messages == []

    def test_restores_and_resets(self, monkeypatch):
        # A properly staged sibling never left its switch: the give-back stands alone.
        sib = VividLane("lane1", pos=425.0)
        rig = self._rig(monkeypatch, sib)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 75.0}
        rig.unit._restore_sibling(p)
        assert sib.moves == [(75.0, SpeedMode.SHORT, None, AssistActive.NO, False)]
        assert sib.pos == 500.0
        assert p == {"slot": 0, "sib_lane": None, "sib_dist": 0.0}
        assert rig.logger.messages == []

    def test_a_give_back_that_lands_short_creeps_onto_the_switch(self, monkeypatch):
        # Homing is on, but the unit has no move_to_load to home with.
        sib = VividLane("lane1", pos=-100.0, unit=False)
        rig = self._rig(monkeypatch, sib)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 75.0}
        rig.unit._restore_sibling(p)
        assert sib.moves == [(75.0, SpeedMode.SHORT, None, AssistActive.NO, False),
                             self.CREEP, self.CREEP, self.CREEP]
        assert sib.pos == 5.0
        assert rig.logger.messages == [
            ("info", "ViViD RFID: fed sibling lane1 a further 30mm to put it back on "
                     "its load switch")]

    def test_error_logged_and_state_reset(self, monkeypatch):
        sib = VividLane("lane1", pos=425.0)
        sib.fault = RuntimeError("boom")
        rig = self._rig(monkeypatch, sib)
        p: Dict[str, Any] = {"slot": 0, "sib_lane": sib, "sib_dist": 75.0}
        rig.unit._restore_sibling(p)
        assert p == {"slot": 0, "sib_lane": None, "sib_dist": 0.0}
        assert rig.logger.messages == [
            ("error", "ViViD RFID: FAILED to restore sibling by 75mm: boom")]


class TestAFCVividrfidAfcIsPrinting:
    def test_false_when_no_print_stats(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.printer.set_print_state("printing")
        del rig.printer._objects["print_stats"]
        assert rig.unit._afc_is_printing() is False
        assert rig.logger.messages == []

    def test_true_when_printing(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.printer.set_print_state("printing")
        assert rig.unit._afc_is_printing() is True
        assert rig.logger.messages == []

    def test_false_when_not_printing(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.printer.set_print_state("paused")
        assert rig.unit._afc_is_printing() is False
        assert rig.logger.messages == []

    def test_false_on_exception(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        rig.printer.set_print_state("printing")
        monkeypatch.setattr(rig.printer.print_stats, "get_status",
                            Recorder(raises=RuntimeError("boom")))
        assert rig.unit._afc_is_printing() is False
        assert rig.logger.messages == []


class TestAFCVividrfidStageReadEnd:
    NO_TAG: LogLine = ("info", "ViViD RFID: no tag decoded on lane0 during staging")

    @staticmethod
    def _blocked_rig(monkeypatch: pytest.MonkeyPatch) -> VividRig:
        """
        :param monkeypatch: for build_vivid
        :return VividRig: lane0's stage read begun, its tool-loaded sibling's
            tag parked on the coil
        """
        rig = vivid_pair_rig(monkeypatch,
                             sibling=VividLane("lane1", tool_loaded=True))
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        return rig

    def test_stage_read_end_tears_down_the_poll(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                          lanes=[VividLane("lane0")])
        rig.unit._stage_read_begin(rig.lane("lane0"))
        assert rig.printer.reactor.timers == [rig.unit._poll_timer]
        rig.unit._stage_read_end(rig.lane("lane0"))
        assert rig.unit._probe is None and rig.unit._poll_timer is None
        assert rig.printer.reactor.timers == []
        assert rig.console == []
        assert rig.logger.messages == [self.NO_TAG]

    def test_stage_read_end_without_a_probe_is_quiet(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                          lanes=[VividLane("lane0")])
        rig.unit._stage_read_end(rig.lane("lane0"))
        assert rig.unit._probe is None and rig.unit._poll_timer is None
        assert rig.console == []
        assert rig.logger.messages == []

    def test_stage_read_end_restores_a_moved_sibling(self, monkeypatch):
        rig = vivid_pair_rig(monkeypatch)
        rig.antenna.uid = b"\xca\xfe"
        rig.unit._stage_read_begin(rig.lane("lane0"))
        rig.unit._probe["read_ok"] = True
        rig.unit._stage_read_end(rig.lane("lane0"))
        lane1 = rig.lane("lane1")
        assert lane1.moves == vivid_retract_moves(*VIVID_FULL_RETRACT) + [
            (75.0, SpeedMode.SHORT, None, AssistActive.NO, False)]
        assert lane1.pos == 500.0
        assert rig.unit._probe is None
        assert rig.unit._poll_timer is None and rig.printer.reactor.timers == []
        assert rig.console == []
        assert rig.logger.messages == [VIVID_PARKED_CAFE, VIVID_RETRACTING]

    def test_stage_hint_when_unmovable_sister_blocks_and_no_read(self, monkeypatch):
        rig = self._blocked_rig(monkeypatch)
        assert rig.unit._probe["blocked_sib"] == "lane1"
        rig.unit._stage_read_end(rig.lane("lane0"))
        assert rig.console == [(
            "respond_info",
            "ViViD RFID: couldn't read lane0's RFID, lane lane1's spool is on the "
            "shared reader blocking it. Manually move lane lane1's spool and re-run "
            "VIVID_RFID_READ, or set lane0's spool id by hand.")]
        assert rig.logger.messages == [VIVID_PARKED_CAFE, VIVID_REFUSED, self.NO_TAG]
        assert rig.unit._probe is None

    def test_stage_no_hint_when_read_succeeded(self, monkeypatch):
        rig = self._blocked_rig(monkeypatch)
        rig.unit._probe["read_ok"] = True
        rig.unit._stage_read_end(rig.lane("lane0"))
        assert rig.console == []
        assert rig.logger.messages == [VIVID_PARKED_CAFE, VIVID_REFUSED]
        assert rig.unit._probe is None


class TestAFCVividrfidCancelPoll:
    def test_no_timer_just_clears_probe(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        unregister = Recorder()
        monkeypatch.setattr(rig.printer.reactor, "unregister_timer", unregister)
        rig.unit._probe = vivid_probe()
        rig.unit._cancel_poll()
        assert unregister.calls == []
        assert rig.unit._probe is None and rig.unit._poll_timer is None
        assert rig.logger.messages == []

    def test_unregister_exception_swallowed(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                          lanes=[VividLane("lane0")])
        rig.unit._stage_read_begin(rig.lane("lane0"))
        timer = rig.unit._poll_timer
        unregister = Recorder(raises=RuntimeError("boom"))
        monkeypatch.setattr(rig.printer.reactor, "unregister_timer", unregister)
        rig.unit._cancel_poll()
        assert unregister.calls == [((timer,), {})]
        assert rig.unit._poll_timer is None and rig.unit._probe is None
        assert rig.logger.messages == []


class TestAFCVividrfidCmdVividRFIDRead:
    #: gcmd.error() raises this.
    CMD_ERROR = BambuPrinter.command_error

    @staticmethod
    def _rig(monkeypatch: pytest.MonkeyPatch) -> VividRig:
        """
        :param monkeypatch: for build_vivid
        :return VividRig: lane0 on slot 0; reader1 serves slots 2 and 3
        """
        return build_vivid(monkeypatch, options={"lane_slot_map": "lane0:0"},
                           readers=(("reader0", "0, 1"), ("reader1", "2, 3")),
                           lanes=[VividLane("lane0")])

    def test_requires_lane_or_slot(self, monkeypatch):
        rig = self._rig(monkeypatch)
        with pytest.raises(self.CMD_ERROR) as exc:
            rig.printer.gcode.run("VIVID_RFID_READ")
        assert str(exc.value) == "VIVID_RFID_READ requires LANE= or SLOT="
        assert rig.field.calls == []
        assert rig.logger.messages == []

    def test_lane_success_no_message(self, monkeypatch):
        # read_lane's apply already printed the read-out.
        rig = self._rig(monkeypatch)
        rig.field.tags = [vivid_elegoo_tag()]
        gcmd = rig.printer.gcode.run("VIVID_RFID_READ", LANE="lane0")
        assert gcmd.messages == []
        assert rig.lane("lane0").material == "PLA"
        assert rig.console == [VIVID_ELEGOO_READ_OUT]
        assert rig.logger.messages == []

    def test_lane_no_decode_plain_hint(self, monkeypatch):
        rig = self._rig(monkeypatch)
        gcmd = rig.printer.gcode.run("VIVID_RFID_READ", LANE="lane0")
        assert gcmd.messages == [("respond_info", "ViViD RFID: no tag decoded on lane0")]
        assert rig.console == []
        assert rig.logger.messages == []

    def test_lane_no_decode_with_seen_uid_hint(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.field.tags = [{"uid": "AABB", "tag_type": "MifareClassic1k",
                           "filament": None}]
        gcmd = rig.printer.gcode.run("VIVID_RFID_READ", LANE="lane0")
        assert gcmd.messages == [(
            "respond_info", "ViViD RFID: no tag decoded on lane0 (saw tag UID AABB, "
                            "MifareClassic1k, no decoder/key matched)")]
        assert rig.console == []
        assert rig.logger.messages == []

    def test_slot_success_calls_respond_tag(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.field.tags = [vivid_btt_tag()]
        gcmd = rig.printer.gcode.run("VIVID_RFID_READ", SLOT=3)
        assert rig.field.calls[0][0] is rig.readers["reader1"].link
        assert gcmd.messages == [("respond_info", VIVID_BTT_SLOT3_SUMMARY)]
        # A slot read reports only: no lane is applied.
        assert rig.console == []
        assert rig.lane("lane0").material is None
        assert rig.logger.messages == []

    def test_slot_no_tag_plain(self, monkeypatch):
        rig = self._rig(monkeypatch)
        gcmd = rig.printer.gcode.run("VIVID_RFID_READ", SLOT=3)
        assert gcmd.messages == [("respond_info", "ViViD RFID: no tag decoded on slot 3")]
        assert rig.console == []
        assert rig.logger.messages == []

    def test_slot_tag_without_uid(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.field.tags = [{"uid": "", "tag_type": "MC1k", "filament": None}]
        gcmd = rig.printer.gcode.run("VIVID_RFID_READ", SLOT=3)
        assert gcmd.messages == [("respond_info", "ViViD RFID: no tag decoded on slot 3")]
        assert rig.console == []
        assert rig.logger.messages == []

    def test_slot_tag_uid_no_type_hint(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.field.tags = [{"uid": "AABB", "filament": None}]
        gcmd = rig.printer.gcode.run("VIVID_RFID_READ", SLOT=3)
        assert gcmd.messages == [(
            "respond_info", "ViViD RFID: no tag decoded on slot 3 (saw tag UID AABB, "
                            "no decoder/key matched)")]
        assert rig.console == []
        assert rig.logger.messages == []

    def test_slot_tag_uid_with_type_hint(self, monkeypatch):
        rig = self._rig(monkeypatch)
        rig.field.tags = [{"uid": "AABB", "tag_type": "MC1k", "filament": None}]
        gcmd = rig.printer.gcode.run("VIVID_RFID_READ", SLOT=3)
        assert gcmd.messages == [(
            "respond_info", "ViViD RFID: no tag decoded on slot 3 (saw tag UID AABB, "
                            "MC1k, no decoder/key matched)")]
        assert rig.console == []
        assert rig.logger.messages == []


class TestAFCVividrfidRespondTag:
    def test_echoes_formatted_summary(self, monkeypatch):
        rig = build_vivid(monkeypatch)
        gcmd = FakeGcmd()
        rig.unit._respond_tag(gcmd, rig.unit._map(vivid_btt_tag()), "slot 3")
        assert gcmd.messages == [("respond_info", VIVID_BTT_SLOT3_SUMMARY)]
        assert rig.console == []
        assert rig.logger.messages == []


class TestAFCVividrfidGetStatus:
    def test_empty_before_first_read(self, monkeypatch):
        rig = build_vivid(monkeypatch, options={"lane_slot_map": "lane1:1, lane0:0"})
        status = rig.unit.get_status()
        assert status == {"slots": [0, 1], "lane_slot_map": {"lane1": 1, "lane0": 0},
                          "last_reads": {}}
        assert status["lane_slot_map"] is not rig.unit._lane_slot
        assert rig.logger.messages == []


class TestLoadConfig:
    def test_load_config_builds_coordinator(self):
        printer = BambuPrinter()
        unit = load_config(BambuConfig("AFC_Vivid_rfid", printer,
                                       {"lane_slot_map": "lane0:0"}))
        assert isinstance(unit, AFC_Vivid_rfid)
        assert unit.printer is printer
        assert unit._lane_slot == {"lane0": 0}
        assert printer.afc.logger.messages == []


class TestLoadConfigPrefix:
    def test_load_config_prefix_builds_reader(self, monkeypatch):
        bus = VividBus()
        install_vivid_bus(monkeypatch, bus)
        printer = BambuPrinter()
        reader = load_config_prefix(BambuConfig(
            "AFC_Vivid_rfid reader0", printer, {"cs_pin": "PA4", "slots": "0, 1"}))
        assert isinstance(reader, AFC_Vivid_rfid_reader)
        assert reader.name == "reader0"
        assert reader.slots == [0, 1]
        assert reader.spi is bus.spis[0]
        assert printer.afc.logger.messages == []
