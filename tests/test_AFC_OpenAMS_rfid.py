"""
Unit tests for extras/AFC_OpenAMS_rfid.py.

Covers the SPI register link, the per-reader section, the coordinator methods
and the module entry points, complementing tests/test_AFC_OpenAMS_rfid_scan.py
(which drives the scan flow). SPI, the MFRC522 stack and read_tag are faked,
so no hardware is touched.
"""

from __future__ import annotations

import configparser
import importlib.util
import os
import sys
import types

import pytest

import extras
import extras.AFC_OpenAMS_rfid as oams_rfid
import extras.AFC_RFID as afc_rfid_mod
from extras.AFC_rfid_write import StageError
from tests.conftest import CommandError, MockConfig, MockGCodeCommand, MockPrinter

BAMBU_HEX = "00112233445566778899aabbccddeeff"
CREALITY_HEX = "0102030405060708"
CREALITY_ENC_HEX = "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf"
KEYS = {
    "bambu_master_key": BAMBU_HEX,
    "creality_key": CREALITY_HEX,
    "creality_encryption_key": CREALITY_ENC_HEX,
}
BAMBU_BYTES = bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                     0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF])
CREALITY_BYTES = bytes([1, 2, 3, 4, 5, 6, 7, 8])
CREALITY_ENC_BYTES = bytes(range(0xA0, 0xB0))

TOO_OLD_WARNING = ("AFC_OpenAMS_rfid: AFC_RFID.py is too old to resolve "
                   "[AFC_rfid_keys]. Update it or Bambu keys are ignored")
NO_BAMBU_WARNING = ("AFC_OpenAMS_rfid: no bambu_master_key resolved (section "
                    "option or [AFC_rfid_keys]). Bambu tags will not decode")
NO_READERS_WARNING = ("AFC_OpenAMS_rfid: no [AFC_OpenAMS_rfid <name>] reader "
                      "sections found. RFID reads will no-op")

# A decodable Bambu-style read and its hand-derived slot_info.
TAG_PLA = {
    "uid": "aabbccdd", "sak": 8, "tag_type": "MifareClassic1k",
    "filament": {"type": "PLA", "manufacturer": "Bambu", "color_argb": 0xFF112233},
}
SLOT_INFO_PLA = {
    "material": "PLA", "color_hex": "112233", "multi_color": ["112233"],
    "is_dual_color": False, "sku": "", "brand": "Bambu", "sub_type": "",
    "diameter": 1.75, "extruder_temp": None, "bed_temp": None, "mfg_date": "",
    "uid": "aabbccdd", "weight_g": None, "tag_type": "MifareClassic1k",
}
# A tag seen but not decoded (e.g. a missing key).
TAG_UNDECODED = {"uid": "aabb", "sak": 8, "tag_type": "MifareClassic1k",
                 "filament": None}
SCAN_TIME = 1000.0


# ── Helpers ───────────────────────────────────────────────────────────────────

class _FakeSpi:
    """MCU_SPI stand-in recording every transfer and send."""

    def __init__(self, response=(0x00, 0x00)):
        self.response = response
        self.transfers = []
        self.sends = []

    def spi_transfer(self, data):
        self.transfers.append(list(data))
        return {"response": self.response}

    def spi_send(self, data):
        self.sends.append(list(data))


def _fake_bus_module(name, built):
    """A bus module whose MCU_SPI_from_config records its call."""
    mod = types.ModuleType(name)

    def mcu_spi_from_config(config, mode, pin_option=None, default_speed=None,
                            cs_active_high=None):
        spi = _FakeSpi()
        built.append({"config": config, "mode": mode, "pin_option": pin_option,
                      "default_speed": default_speed,
                      "cs_active_high": cs_active_high, "spi": spi})
        return spi

    mod.MCU_SPI_from_config = mcu_spi_from_config
    return mod


def _install_bus(monkeypatch):
    """Serve a fake ``extras.bus`` to the reader's relative import."""
    built = []
    fake = _fake_bus_module("extras.bus", built)
    monkeypatch.setitem(sys.modules, "extras.bus", fake)
    monkeypatch.setattr(extras, "bus", fake, raising=False)
    return built


def _make_reader(monkeypatch, printer, name="RFID_A", values=None):
    _install_bus(monkeypatch)
    config = MockConfig(name=f"AFC_OpenAMS_rfid {name}", printer=printer,
                        values=values or {})
    return oams_rfid.AFC_OpenAMS_rfid_reader(config)


def _make_coord(printer=None, values=None):
    printer = printer or MockPrinter()
    config = MockConfig(name="AFC_OpenAMS_rfid", printer=printer,
                        values=values or {})
    return oams_rfid.AFC_OpenAMS_rfid(config), printer


def _connected(monkeypatch, readers=(("RFID_A", "0, 1"), ("RFID_B", "2, 3")),
               lane_map="lane1:0, lane2:2"):
    """A coordinator after klippy:connect, with real reader sections."""
    printer = MockPrinter()
    built = {}
    for name, slots in readers:
        rdr = _make_reader(monkeypatch, printer, name, {"slots": slots})
        printer._objects[f"AFC_OpenAMS_rfid {name}"] = rdr
        built[name] = rdr
    coord, _ = _make_coord(printer, {**KEYS, "lane_slot_map": lane_map})
    printer.send_event("klippy:connect")
    printer._afc.logger.messages.clear()
    return coord, printer, built


class _ReadTagRecorder:
    """read_tag stand-in returning a fixed result and recording calls."""

    def __init__(self, result=None):
        self.result = result
        self.calls = []

    def __call__(self, link, **kwargs):
        self.calls.append((link, kwargs))
        return self.result


def _install_activate(monkeypatch, behaviour):
    """Fake Mfrc522/MifareClassic; activate() runs ``behaviour``."""
    calls = []

    class FakeMfrc522:
        def __init__(self, link):
            self.link = link

    class FakeClassic:
        def __init__(self, mfrc):
            self.mfrc = mfrc

        def activate(self, is_excluded=None, seen=None):
            calls.append({"link": self.mfrc.link, "is_excluded": is_excluded,
                          "seen": seen})
            return behaviour(is_excluded, seen)

    monkeypatch.setattr(oams_rfid, "Mfrc522", FakeMfrc522)
    monkeypatch.setattr(oams_rfid, "MifareClassic", FakeClassic)
    return calls


def _freeze_tag_time(monkeypatch):
    """Pin the scan_time AFC_RFID stamps on last-read records."""
    monkeypatch.setattr(afc_rfid_mod, "time",
                        types.SimpleNamespace(time=lambda: SCAN_TIME))


class _FakeOams:
    def __init__(self, ready=True):
        self.ready = ready
        self.checked = []

    def is_bay_ready(self, slot):
        self.checked.append(slot)
        return self.ready


class _FakeScanUnit:
    """afcAMS stand-in with only what _stage_write_around touches."""

    def __init__(self, rfid_slot=0, excluder=None):
        self.rfid_slot = rfid_slot
        # Passed to hold() by the scan; None means no tag came to rest.
        self.excluder = excluder
        self._operation_active = False
        self._spool_map = {"lane1": 0}
        self.oams = _FakeOams()
        self._rfid_scanned = {"lane1"}
        self.blocked_reason = None
        self.slot_queries = []
        self.scans = []

    def _rfid_scan_slot(self, coord, lane):
        self.slot_queries.append((coord, lane))
        return self.rfid_slot

    def _rfid_scan_blocked_reason(self, lane):
        return self.blocked_reason

    def _do_rfid_scan(self, lane, hold=None):
        self.scans.append((lane, set(self._rfid_scanned)))
        if self.excluder is not None:
            hold(self.excluder)


def _boom(*args, **kwargs):
    raise RuntimeError("boom")


# ── Module-level optional helper imports ──────────────────────────────────────

class TestModuleOptionalHelperImports:
    def _load_without_helpers(self, monkeypatch):
        # An older AFC_RFID that lacks both optional helpers.
        stub = types.ModuleType("extras.AFC_RFID")
        stub.AFCUnitRFID = afc_rfid_mod.AFCUnitRFID
        stub.map_tag_to_slot_info = afc_rfid_mod.map_tag_to_slot_info
        monkeypatch.setitem(sys.modules, "extras.AFC_RFID", stub)
        spec = importlib.util.spec_from_file_location(
            "extras._oams_rfid_old_afc", os.path.abspath(oams_rfid.__file__))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    def test_helpers_bound_when_afc_rfid_has_them(self):
        assert oams_rfid.get_auto_spoolman_create is afc_rfid_mod.get_auto_spoolman_create
        assert oams_rfid.resolve_rfid_keys is afc_rfid_mod.resolve_rfid_keys

    def test_helpers_none_when_afc_rfid_is_too_old(self, monkeypatch):
        mod = self._load_without_helpers(monkeypatch)
        assert mod.get_auto_spoolman_create is None
        assert mod.resolve_rfid_keys is None


# ── _OamsSpiRegLink ───────────────────────────────────────────────────────────

class TestOamsSpiRegLinkInit:
    def test_keeps_the_spi(self):
        spi = _FakeSpi()
        link = oams_rfid._OamsSpiRegLink(spi)
        assert link.spi is spi
        assert spi.transfers == []
        assert spi.sends == []


class TestOamsSpiRegLinkRegRead:
    def test_returns_second_response_byte(self):
        spi = _FakeSpi(response=bytes([0xFF, 0x5A]))
        link = oams_rfid._OamsSpiRegLink(spi)
        assert link.reg_read(0x01) == 0x5A
        # Register 0x01 shifted left is 0x02, plus the 0x80 read bit.
        assert spi.transfers == [[0x82, 0x00]]

    def test_address_masks_register_to_six_bits(self):
        spi = _FakeSpi(response=[0x00, 0x11])
        link = oams_rfid._OamsSpiRegLink(spi)
        link.reg_read(0x3F)
        link.reg_read(0x40)
        # 0x3F -> 0x7E | 0x80; 0x40 overflows the mask, leaving only 0x80.
        assert spi.transfers == [[0xFE, 0x00], [0x80, 0x00]]

    def test_single_byte_response_reads_zero(self):
        spi = _FakeSpi(response=[0x5A])
        link = oams_rfid._OamsSpiRegLink(spi)
        assert link.reg_read(0x01) == 0

    def test_empty_response_reads_zero(self):
        spi = _FakeSpi(response=b"")
        link = oams_rfid._OamsSpiRegLink(spi)
        assert link.reg_read(0x37) == 0
        assert spi.transfers == [[0xEE, 0x00]]


class TestOamsSpiRegLinkRegWrite:
    def test_sends_shifted_address_and_value(self):
        spi = _FakeSpi()
        link = oams_rfid._OamsSpiRegLink(spi)
        assert link.reg_write(0x01, 0x3C) is None
        assert spi.sends == [[0x02, 0x3C]]
        assert spi.transfers == []

    def test_masks_register_and_value(self):
        spi = _FakeSpi()
        link = oams_rfid._OamsSpiRegLink(spi)
        link.reg_write(0x41, 0x1AB)
        # 0x41 << 1 is 0x82, masked to 0x02; 0x1AB keeps its low byte 0xAB.
        assert spi.sends == [[0x02, 0xAB]]


class TestOamsSpiRegLinkReaderPower:
    def test_power_on_and_off_are_no_ops(self):
        spi = _FakeSpi()
        link = oams_rfid._OamsSpiRegLink(spi)
        assert link.reader_power(True) is None
        assert link.reader_power(False) is None
        assert spi.transfers == []
        assert spi.sends == []
        assert link.spi is spi


# ── AFC_OpenAMS_rfid_reader ───────────────────────────────────────────────────

class _FakePin:
    def __init__(self):
        self.start_values = []

    def setup_start_value(self, start_value, shutdown_value):
        self.start_values.append((start_value, shutdown_value))


class _FakePins:
    def __init__(self):
        self.setups = []
        self.pin = _FakePin()

    def setup_pin(self, pin_type, pin_desc):
        self.setups.append((pin_type, pin_desc))
        return self.pin


class TestAFCOpenAMSRfidReaderInit:
    def _build(self, monkeypatch, values=None, name="AFC_OpenAMS_rfid RFID_A"):
        built = _install_bus(monkeypatch)
        printer = MockPrinter()
        config = MockConfig(name=name, printer=printer, values=values or {})
        return oams_rfid.AFC_OpenAMS_rfid_reader(config), printer, config, built

    def test_builds_spi_link_and_parses_slots(self, monkeypatch):
        reader, printer, config, built = self._build(
            monkeypatch, {"slots": "0, 1"})
        assert reader.printer is printer
        assert reader.name == "RFID_A"
        assert reader.logger is printer._afc.logger
        assert len(built) == 1
        call = built[0]
        assert call["config"] is config
        assert call["mode"] == 0
        assert call["pin_option"] == "cs_pin"
        assert call["default_speed"] == 5000000
        assert call["cs_active_high"] is False
        assert reader.spi is call["spi"]
        assert isinstance(reader.link, oams_rfid._OamsSpiRegLink)
        assert reader.link.spi is reader.spi
        assert reader._reset_pin is None
        assert reader.slots == [0, 1]
        assert printer._afc.logger.messages == []

    def test_falls_back_to_top_level_bus_import(self, monkeypatch):
        # extras.bus unavailable: the reader imports the top-level ``bus``.
        top_built = []
        monkeypatch.setitem(sys.modules, "extras.bus", None)
        monkeypatch.delattr(extras, "bus", raising=False)
        monkeypatch.setitem(sys.modules, "bus", _fake_bus_module("bus", top_built))
        config = MockConfig(name="AFC_OpenAMS_rfid RFID_B", printer=MockPrinter(),
                            values={"slots": "2"})
        reader = oams_rfid.AFC_OpenAMS_rfid_reader(config)
        assert len(top_built) == 1
        assert reader.spi is top_built[0]["spi"]
        assert reader.slots == [2]

    def test_transport_spelled_out_with_case_and_space_is_accepted(self, monkeypatch):
        reader, _, _, built = self._build(monkeypatch, {"transport": " SPI "})
        assert len(built) == 1
        assert reader.spi is built[0]["spi"]

    def test_unknown_transport_raises(self, monkeypatch):
        with pytest.raises(configparser.Error) as exc:
            self._build(monkeypatch, {"transport": " I2C "})
        assert str(exc.value) == "AFC_OpenAMS_rfid RFID_A: unknown transport 'i2c' (spi)"

    def test_unknown_transport_builds_no_spi(self, monkeypatch):
        built = _install_bus(monkeypatch)
        config = MockConfig(name="AFC_OpenAMS_rfid RFID_A", printer=MockPrinter(),
                            values={"transport": "uart"})
        with pytest.raises(configparser.Error):
            oams_rfid.AFC_OpenAMS_rfid_reader(config)
        assert built == []

    def test_reset_pin_is_set_up_high(self, monkeypatch):
        built = _install_bus(monkeypatch)
        printer = MockPrinter()
        pins = _FakePins()
        printer._objects["pins"] = pins
        config = MockConfig(name="AFC_OpenAMS_rfid RFID_A", printer=printer,
                            values={"reset_pin": "oams:PB5"})
        reader = oams_rfid.AFC_OpenAMS_rfid_reader(config)
        assert len(built) == 1
        assert pins.setups == [("digital_out", "oams:PB5")]
        assert reader._reset_pin is pins.pin
        assert pins.pin.start_values == [(1, 1)]

    def test_empty_reset_pin_is_ignored(self, monkeypatch):
        _install_bus(monkeypatch)
        printer = MockPrinter()
        pins = _FakePins()
        printer._objects["pins"] = pins
        config = MockConfig(name="AFC_OpenAMS_rfid RFID_A", printer=printer,
                            values={"reset_pin": ""})
        reader = oams_rfid.AFC_OpenAMS_rfid_reader(config)
        assert reader._reset_pin is None
        assert pins.setups == []

    def test_blank_slot_entries_are_skipped(self, monkeypatch):
        reader, _, _, _ = self._build(monkeypatch, {"slots": " , 2 ,, 3 , "})
        assert reader.slots == [2, 3]

    def test_unset_slots_gives_empty_list(self, monkeypatch):
        reader, _, _, _ = self._build(monkeypatch)
        assert reader.slots == []

    def test_none_slots_gives_empty_list(self, monkeypatch):
        reader, _, _, _ = self._build(monkeypatch, {"slots": None})
        assert reader.slots == []

    def test_bad_slot_number_raises(self, monkeypatch):
        with pytest.raises(configparser.Error) as exc:
            self._build(monkeypatch, {"slots": "0, x"})
        assert str(exc.value) == "AFC_OpenAMS_rfid RFID_A: bad slot number 'x' in 'slots'"


# ── AFC_OpenAMS_rfid.__init__ ─────────────────────────────────────────────────

class _SharedKeys:
    """[AFC_rfid_keys] stand-in."""

    def __init__(self):
        self.bambu_master_key = b"shared-bambu"
        self.creality_key = b"shared-creality"
        self.creality_encryption_key = b"shared-enc"


class TestAFCOpenAMSRfidInit:
    def test_attributes_and_registration(self):
        coord, printer = _make_coord(values=dict(KEYS))
        assert coord.printer is printer
        assert coord.reactor is printer._reactor
        assert coord.gcode is printer._gcode
        assert coord.logger is printer._afc.logger
        assert coord.afc is None
        assert coord.log_prefix == "OpenAMS RFID"
        assert coord.auto_create is False
        assert coord._lane_slot == {}
        assert coord._slot_reader == {}
        assert coord._last == {}
        assert coord._no_reader_warned == set()
        assert printer._gcode._commands == {"OAMS_RFID_READ": coord.cmd_OAMS_RFID_READ}
        assert printer._event_handlers == {"klippy:connect": [coord._on_connect]}
        assert printer._afc.logger.messages == []

    def test_auto_spoolman_create_enabled(self):
        coord, _ = _make_coord(values={**KEYS, "auto_spoolman_create": True})
        assert coord.auto_create is True

    def test_section_keys_are_parsed_from_hex(self):
        coord, printer = _make_coord(values=dict(KEYS))
        assert coord.bambu_master_key == BAMBU_BYTES
        assert coord.creality_key == CREALITY_BYTES
        assert coord.creality_encryption_key == CREALITY_ENC_BYTES
        assert printer._afc.logger.messages == []

    def test_unset_and_empty_keys_are_none(self):
        coord, printer = _make_coord(values={"creality_key": ""})
        assert coord.bambu_master_key is None
        assert coord.creality_key is None
        assert coord.creality_encryption_key is None
        assert printer._afc.logger.messages == [("warning", NO_BAMBU_WARNING)]

    def test_shared_keys_fill_unset_section_keys(self):
        printer = MockPrinter()
        printer._objects["AFC_rfid_keys"] = _SharedKeys()
        coord, _ = _make_coord(printer, {"creality_key": CREALITY_HEX})
        assert coord.bambu_master_key == b"shared-bambu"
        # The section's own key wins over the shared one.
        assert coord.creality_key == CREALITY_BYTES
        assert coord.creality_encryption_key == b"shared-enc"
        assert printer._afc.logger.messages == []

    def test_old_afc_rfid_warns_and_keeps_section_keys(self, monkeypatch):
        monkeypatch.setattr(oams_rfid, "resolve_rfid_keys", None)
        printer = MockPrinter()
        printer._objects["AFC_rfid_keys"] = _SharedKeys()
        coord, _ = _make_coord(printer, {"bambu_master_key": BAMBU_HEX})
        assert coord.bambu_master_key == BAMBU_BYTES
        # Without the resolver the shared keys are never consulted.
        assert coord.creality_key is None
        assert coord.creality_encryption_key is None
        assert printer._afc.logger.messages == [("warning", TOO_OLD_WARNING)]

    def test_old_afc_rfid_without_bambu_key_warns_only_too_old(self, monkeypatch):
        monkeypatch.setattr(oams_rfid, "resolve_rfid_keys", None)
        coord, printer = _make_coord()
        assert coord.bambu_master_key is None
        assert printer._afc.logger.messages == [("warning", TOO_OLD_WARNING)]

    def test_lane_slot_map_is_parsed(self):
        coord, _ = _make_coord(values={**KEYS, "lane_slot_map": "lane4:0, lane5:1"})
        assert coord._lane_slot == {"lane4": 0, "lane5": 1}

    def test_lane_slot_map_strips_and_skips_blank_entries(self):
        coord, _ = _make_coord(values={**KEYS, "lane_slot_map": " , lane4 : 2 ,, "})
        assert coord._lane_slot == {"lane4": 2}

    def test_none_lane_slot_map_gives_empty_map(self):
        coord, _ = _make_coord(values={**KEYS, "lane_slot_map": None})
        assert coord._lane_slot == {}

    @pytest.mark.parametrize("entry", ["lane4", "lane4:x", "lane4:0:1"])
    def test_bad_lane_slot_map_entry_raises(self, entry):
        with pytest.raises(configparser.Error) as exc:
            _make_coord(values={**KEYS, "lane_slot_map": f"lane1:0, {entry}"})
        expected = ("AFC_OpenAMS_rfid: 'lane_slot_map' entries must be "
                    f"'lane:slot'. Bad entry '{entry}'")
        assert str(exc.value) == expected


# ── AFC_OpenAMS_rfid._on_connect ──────────────────────────────────────────────

class TestAFCOpenAMSRfidOnConnect:
    def _record_register(self, monkeypatch):
        calls = []

        def fake_register(printer, name, label, unit, open_link,
                          stage_around=None, serves=None):
            calls.append({"printer": printer, "name": name, "label": label,
                          "unit": unit, "open_link": open_link,
                          "stage_around": stage_around, "serves": serves})

        monkeypatch.setattr(oams_rfid, "register_reader", fake_register)
        return calls

    def _setup(self, monkeypatch, readers, lane_map="lane1:0, lane2:2"):
        printer = MockPrinter()
        built = {}
        for key, name, slots in readers:
            rdr = _make_reader(monkeypatch, printer, name, {"slots": slots})
            printer._objects[key] = rdr
            built[name] = rdr
        coord, _ = _make_coord(printer, {**KEYS, "lane_slot_map": lane_map})
        return coord, printer, built

    def test_indexes_readers_by_slot_and_offers_each_once(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, printer, rdrs = self._setup(monkeypatch, [
            ("AFC_OpenAMS_rfid RFID_A", "RFID_A", "0, 1"),
            ("AFC_OpenAMS_rfid RFID_B", "RFID_B", "2, 3")])
        coord._on_connect()
        assert coord.afc is printer._afc
        assert coord._slot_reader == {0: rdrs["RFID_A"], 1: rdrs["RFID_A"],
                                      2: rdrs["RFID_B"], 3: rdrs["RFID_B"]}
        assert [c["name"] for c in calls] == ["oams:RFID_A", "oams:RFID_B"]
        assert [c["label"] for c in calls] == ["OpenAMS RFID_A (slots 0, 1)",
                                               "OpenAMS RFID_B (slots 2, 3)"]
        assert all(c["printer"] is printer for c in calls)
        assert all(c["unit"] is coord for c in calls)
        assert printer._afc.logger.messages == []

    def test_open_link_returns_each_readers_link(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, _, rdrs = self._setup(monkeypatch, [
            ("AFC_OpenAMS_rfid RFID_A", "RFID_A", "0"),
            ("AFC_OpenAMS_rfid RFID_B", "RFID_B", "2")])
        coord._on_connect()
        assert calls[0]["open_link"]() is rdrs["RFID_A"].link
        assert calls[1]["open_link"]() is rdrs["RFID_B"].link

    def test_serves_matches_the_lanes_mapped_slot(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, _, _ = self._setup(monkeypatch, [
            ("AFC_OpenAMS_rfid RFID_A", "RFID_A", "0, 1"),
            ("AFC_OpenAMS_rfid RFID_B", "RFID_B", "2, 3")])
        coord._on_connect()
        serves_a, serves_b = calls[0]["serves"], calls[1]["serves"]
        assert serves_a("lane1") is True
        assert serves_b("lane1") is False
        assert serves_a("lane2") is False
        assert serves_b("lane2") is True
        assert serves_a("unmapped") is False

    def test_stage_around_routes_to_stage_write_with_its_reader(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, printer, _ = self._setup(monkeypatch, [
            ("AFC_OpenAMS_rfid RFID_A", "RFID_A", "0"),
            ("AFC_OpenAMS_rfid RFID_B", "RFID_B", "2")])
        coord._on_connect()
        # The lane's tag passes slot 2, so only RFID_B's hook may proceed.
        unit = _FakeScanUnit(rfid_slot=2, excluder="ex")
        printer._afc.lanes["lane1"] = types.SimpleNamespace(name="lane1", unit_obj=unit)
        with pytest.raises(StageError) as exc:
            calls[0]["stage_around"]("lane1", lambda ex: None)
        assert str(exc.value) == "lane1 is not on RFID_A; write it with READER=oams:RFID_B"
        written = []
        calls[1]["stage_around"]("lane1", written.append)
        assert written == ["ex"]

    def test_duplicate_slot_warns_and_last_reader_wins(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, printer, rdrs = self._setup(monkeypatch, [
            ("AFC_OpenAMS_rfid RFID_A", "RFID_A", "0, 1"),
            ("AFC_OpenAMS_rfid RFID_B", "RFID_B", "1, 2")])
        coord._on_connect()
        assert coord._slot_reader == {0: rdrs["RFID_A"], 1: rdrs["RFID_B"],
                                      2: rdrs["RFID_B"]}
        assert printer._afc.logger.messages == [
            ("warning", "AFC_OpenAMS_rfid: slot 1 served by more than one reader "
                        "(RFID_A and RFID_B)")]
        assert [c["name"] for c in calls] == ["oams:RFID_A", "oams:RFID_B"]

    def test_object_under_reader_name_that_is_not_a_reader_is_ignored(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, printer, _ = self._setup(monkeypatch, [])
        printer._objects["AFC_OpenAMS_rfid RFID_X"] = types.SimpleNamespace(
            name="RFID_X", slots=[5])
        coord._on_connect()
        assert coord._slot_reader == {}
        assert calls == []
        assert printer._afc.logger.messages == [("warning", NO_READERS_WARNING)]

    def test_reader_under_another_name_is_ignored(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, printer, _ = self._setup(monkeypatch, [
            ("AFC_OpenAMS_rfid", "RFID_A", "0"),
            ("other_reader RFID_C", "RFID_C", "4")])
        coord._on_connect()
        assert coord._slot_reader == {}
        assert calls == []
        assert printer._afc.logger.messages == [("warning", NO_READERS_WARNING)]

    def test_reader_with_no_slots_is_not_offered(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, printer, _ = self._setup(monkeypatch, [
            ("AFC_OpenAMS_rfid RFID_A", "RFID_A", "")])
        coord._on_connect()
        assert coord._slot_reader == {}
        assert calls == []
        assert printer._afc.logger.messages == [("warning", NO_READERS_WARNING)]

    def test_no_readers_warns(self, monkeypatch):
        calls = self._record_register(monkeypatch)
        coord, printer, _ = self._setup(monkeypatch, [])
        coord._on_connect()
        assert coord.afc is printer._afc
        assert calls == []
        assert printer._afc.logger.messages == [("warning", NO_READERS_WARNING)]

    def test_registers_with_the_shared_writer_registry(self, monkeypatch):
        coord, printer, rdrs = _connected(monkeypatch)
        registry = printer._afc_rfid_write_registry
        assert list(registry) == ["oams:RFID_A", "oams:RFID_B"]
        target = registry["oams:RFID_A"]
        assert target.label == "OpenAMS RFID_A (slots 0, 1)"
        assert target.unit is coord
        assert target.open_link() is rdrs["RFID_A"].link


# ── AFC_OpenAMS_rfid._stage_write_around ──────────────────────────────────────

class TestAFCOpenAMSRfidStageWriteAround:
    def _setup(self, monkeypatch, unit=None, rfid_slot=0, excluder="ex"):
        coord, printer, rdrs = _connected(
            monkeypatch, readers=(("RFID_A", "0, 1"), ("RFID_B", "2")))
        unit = unit if unit is not None else _FakeScanUnit(rfid_slot, excluder)
        lane = types.SimpleNamespace(name="lane1", unit_obj=unit)
        printer._afc.lanes["lane1"] = lane
        return coord, rdrs, unit, lane

    def _refused(self, coord, rdr, lane_name="lane1"):
        body_calls = []
        with pytest.raises(StageError) as exc:
            coord._stage_write_around(lane_name, rdr, body_calls.append)
        assert body_calls == []
        return str(exc.value)

    def test_runs_the_write_inside_the_scan(self, monkeypatch):
        coord, rdrs, unit, lane = self._setup(monkeypatch, excluder="sisters")
        body_calls = []
        assert coord._stage_write_around("lane1", rdrs["RFID_A"],
                                         body_calls.append) is None
        assert body_calls == ["sisters"]
        assert unit.slot_queries == [(coord, lane)]
        # The scanned latch is cleared before the scan so it runs again.
        assert unit.scans == [(lane, set())]
        assert unit._rfid_scanned == set()
        assert unit.oams.checked == [0]

    def test_no_tag_at_rest_is_refused_after_the_scan(self, monkeypatch):
        coord, rdrs, unit, lane = self._setup(monkeypatch, excluder=None)
        msg = self._refused(coord, rdrs["RFID_A"])
        assert msg == ("no tag on lane1 came to rest at the reader; the lane "
                       "is back in its bay")
        assert unit.scans == [(lane, set())]
        assert unit._rfid_scanned == set()

    def test_unknown_lane_is_refused(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch)
        assert self._refused(coord, rdrs["RFID_A"], "lane9") == \
            "lane9 is not an OpenAMS lane"
        assert unit.slot_queries == []

    def test_before_connect_every_lane_is_refused(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch)
        coord.afc = None
        assert self._refused(coord, rdrs["RFID_A"]) == "lane1 is not an OpenAMS lane"
        assert unit.slot_queries == []

    def test_afc_without_lanes_refuses(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch)
        coord.afc = types.SimpleNamespace(lanes=None)
        assert self._refused(coord, rdrs["RFID_A"]) == "lane1 is not an OpenAMS lane"
        assert unit.slot_queries == []

    def test_lane_on_a_unit_without_a_scan_is_refused(self, monkeypatch):
        coord, rdrs, _, _ = self._setup(monkeypatch, unit=types.SimpleNamespace())
        assert self._refused(coord, rdrs["RFID_A"]) == "lane1 is not an OpenAMS lane"

    def test_lane_without_a_unit_is_refused(self, monkeypatch):
        coord, rdrs, _, _ = self._setup(monkeypatch)
        coord.afc.lanes["lane1"] = types.SimpleNamespace(name="lane1")
        assert self._refused(coord, rdrs["RFID_A"]) == "lane1 is not an OpenAMS lane"

    def test_other_readers_lane_names_its_owner(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch, rfid_slot=2)
        assert self._refused(coord, rdrs["RFID_A"]) == \
            "lane1 is not on RFID_A; write it with READER=oams:RFID_B"
        assert unit.scans == []
        assert unit._rfid_scanned == {"lane1"}

    def test_slot_without_a_reader_gives_no_hint(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch, rfid_slot=7)
        assert self._refused(coord, rdrs["RFID_A"]) == "lane1 is not on RFID_A"
        assert unit.scans == []

    def test_reader_without_slots_is_refused(self, monkeypatch):
        coord, _, unit, _ = self._setup(monkeypatch, rfid_slot=0)
        bare = types.SimpleNamespace(name="RFID_Z")
        assert self._refused(coord, bare) == \
            "lane1 is not on RFID_Z; write it with READER=oams:RFID_A"
        assert unit.scans == []

    def test_busy_unit_is_refused(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch)
        unit._operation_active = True
        assert self._refused(coord, rdrs["RFID_A"]) == (
            "the OpenAMS is busy; try again once the current operation completes")
        assert unit.scans == []
        assert unit._rfid_scanned == {"lane1"}

    def test_blocked_scan_feed_is_refused(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch)
        unit.blocked_reason = "lane1 is loaded to the toolhead"
        assert self._refused(coord, rdrs["RFID_A"]) == (
            "the scan feed is blocked: lane1 is loaded to the toolhead")
        assert unit.oams.checked == []
        assert unit.scans == []

    def test_unmapped_spool_is_refused(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch)
        unit._spool_map = {}
        assert self._refused(coord, rdrs["RFID_A"]) == (
            "no filament inserted in lane1 (the write feeds the lane to turn "
            "the spool)")
        # The bay check never ran: the missing slot short-circuits it.
        assert unit.oams.checked == []
        assert unit.scans == []

    def test_missing_oams_is_refused(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch)
        unit.oams = None
        assert self._refused(coord, rdrs["RFID_A"]) == (
            "no filament inserted in lane1 (the write feeds the lane to turn "
            "the spool)")
        assert unit.scans == []

    def test_empty_bay_is_refused(self, monkeypatch):
        coord, rdrs, unit, _ = self._setup(monkeypatch)
        unit._spool_map = {"lane1": 3}
        unit.oams.ready = False
        assert self._refused(coord, rdrs["RFID_A"]) == (
            "no filament inserted in lane1 (the write feeds the lane to turn "
            "the spool)")
        assert unit.oams.checked == [3]
        assert unit.scans == []
        assert unit._rfid_scanned == {"lane1"}


# ── AFC_OpenAMS_rfid._get_slot ────────────────────────────────────────────────

class TestAFCOpenAMSRfidGetSlot:
    def test_mapped_lane_returns_its_slot(self):
        coord, _ = _make_coord(values={**KEYS, "lane_slot_map": "lane4:2"})
        assert coord._get_slot("lane4") == 2

    def test_unmapped_lane_returns_none(self):
        coord, _ = _make_coord(values={**KEYS, "lane_slot_map": "lane4:2"})
        assert coord._get_slot("lane5") is None


# ── AFC_OpenAMS_rfid.read_slot ────────────────────────────────────────────────

class TestAFCOpenAMSRfidReadSlot:
    def test_reads_through_the_slots_reader_with_keys(self, monkeypatch):
        coord, printer, rdrs = _connected(monkeypatch)
        rec = _ReadTagRecorder(TAG_PLA)
        monkeypatch.setattr(oams_rfid, "read_tag", rec)
        assert coord.read_slot(3) is TAG_PLA
        assert rec.calls == [(rdrs["RFID_B"].link, {
            "bambu_master_key": BAMBU_BYTES, "creality_key": CREALITY_BYTES,
            "creality_encryption_key": CREALITY_ENC_BYTES})]
        assert coord._last == {3: TAG_PLA}
        assert printer._afc.logger.messages == []

    def test_no_tag_is_not_remembered(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)
        coord._last[0] = TAG_PLA
        rec = _ReadTagRecorder(None)
        monkeypatch.setattr(oams_rfid, "read_tag", rec)
        assert coord.read_slot(0) is None
        assert len(rec.calls) == 1
        assert coord._last == {0: TAG_PLA}
        assert printer._afc.logger.messages == []

    def test_slot_without_reader_warns_once(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)
        rec = _ReadTagRecorder(TAG_PLA)
        monkeypatch.setattr(oams_rfid, "read_tag", rec)
        assert coord.read_slot(7) is None
        assert coord._no_reader_warned == {7}
        assert printer._afc.logger.messages == [
            ("warning", "AFC_OpenAMS_rfid: no reader configured for slot 7")]
        assert coord.read_slot(7) is None
        assert printer._afc.logger.messages == [
            ("warning", "AFC_OpenAMS_rfid: no reader configured for slot 7")]
        assert rec.calls == []
        assert coord._last == {}


# ── AFC_OpenAMS_rfid.scan_slot_uids ───────────────────────────────────────────

class TestAFCOpenAMSRfidScanSlotUids:
    def test_lists_every_uid_in_the_field(self, monkeypatch):
        coord, printer, rdrs = _connected(monkeypatch)

        def field(is_excluded, seen):
            seen.extend([("aabb", 8, True), ("ccdd", 8, True)])
            return None, None

        calls = _install_activate(monkeypatch, field)
        assert coord.scan_slot_uids(1) == ["aabb", "ccdd"]
        assert len(calls) == 1
        assert calls[0]["link"] is rdrs["RFID_A"].link
        # Every tag is excluded so each one is HALTed and the next answers.
        assert calls[0]["is_excluded"]("aabb") is True
        assert calls[0]["is_excluded"]("anything") is True
        assert printer._afc.logger.messages == []

    def test_empty_field_gives_empty_list(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)
        calls = _install_activate(monkeypatch, lambda ex, seen: (None, None))
        assert coord.scan_slot_uids(0) == []
        assert len(calls) == 1
        assert printer._afc.logger.messages == []

    def test_slot_without_reader_gives_empty_list(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)
        calls = _install_activate(monkeypatch, _boom)
        assert coord.scan_slot_uids(9) == []
        assert calls == []
        assert printer._afc.logger.messages == []

    def test_reader_failure_logs_and_keeps_uids_seen_so_far(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)

        def flaky(is_excluded, seen):
            seen.append(("aabb", 8, True))
            raise RuntimeError("spi timeout")

        _install_activate(monkeypatch, flaky)
        assert coord.scan_slot_uids(2) == ["aabb"]
        assert printer._afc.logger.messages == [
            ("debug", "scan_slot_uids failed on slot 2: spi timeout")]


# ── AFC_OpenAMS_rfid.detect_slot_tag ──────────────────────────────────────────

class TestAFCOpenAMSRfidDetectSlotTag:
    def test_returns_new_tags_uid_hex(self, monkeypatch):
        coord, printer, rdrs = _connected(monkeypatch)
        calls = _install_activate(monkeypatch,
                                  lambda ex, seen: (bytes([0xAA, 0xBB, 0x0C]), 8))
        assert coord.detect_slot_tag(2, {"5157e12"}) == "aabb0c"
        assert calls[0]["link"] is rdrs["RFID_B"].link
        assert calls[0]["seen"] is None
        assert printer._afc.logger.messages == []

    def test_excluder_tests_membership_in_exclude(self, monkeypatch):
        coord, _, _ = _connected(monkeypatch)
        calls = _install_activate(monkeypatch, lambda ex, seen: (None, None))
        coord.detect_slot_tag(0, {"5157e12"})
        assert calls[0]["is_excluded"]("5157e12") is True
        assert calls[0]["is_excluded"]("aabb") is False

    def test_no_new_tag_returns_none(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)
        calls = _install_activate(monkeypatch, lambda ex, seen: (None, None))
        assert coord.detect_slot_tag(0, set()) is None
        assert len(calls) == 1
        assert printer._afc.logger.messages == []

    def test_slot_without_reader_returns_none(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)
        calls = _install_activate(monkeypatch, _boom)
        assert coord.detect_slot_tag(9, set()) is None
        assert calls == []
        assert printer._afc.logger.messages == []

    def test_reader_failure_logs_and_returns_none(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)
        _install_activate(monkeypatch, _boom)
        assert coord.detect_slot_tag(1, set()) is None
        assert printer._afc.logger.messages == [
            ("debug", "detect_slot_tag failed on slot 1: boom")]


# ── AFC_OpenAMS_rfid.read_slot_excluding ──────────────────────────────────────

class TestAFCOpenAMSRfidReadSlotExcluding:
    def test_reads_with_keys_and_excluder(self, monkeypatch):
        coord, printer, rdrs = _connected(monkeypatch)
        rec = _ReadTagRecorder(TAG_PLA)
        monkeypatch.setattr(oams_rfid, "read_tag", rec)
        assert coord.read_slot_excluding(1, {"5157e12"}) is TAG_PLA
        link, kwargs = rec.calls[0]
        assert link is rdrs["RFID_A"].link
        excluder = kwargs.pop("is_excluded")
        assert kwargs == {"bambu_master_key": BAMBU_BYTES,
                          "creality_key": CREALITY_BYTES,
                          "creality_encryption_key": CREALITY_ENC_BYTES}
        assert excluder("5157e12") is True
        assert excluder("aabbccdd") is False
        assert coord._last == {1: TAG_PLA}
        assert printer._afc.logger.messages == []

    def test_no_tag_is_not_remembered(self, monkeypatch):
        coord, _, _ = _connected(monkeypatch)
        rec = _ReadTagRecorder(None)
        monkeypatch.setattr(oams_rfid, "read_tag", rec)
        assert coord.read_slot_excluding(0, set()) is None
        assert len(rec.calls) == 1
        assert coord._last == {}

    def test_slot_without_reader_returns_none(self, monkeypatch):
        coord, printer, _ = _connected(monkeypatch)
        rec = _ReadTagRecorder(TAG_PLA)
        monkeypatch.setattr(oams_rfid, "read_tag", rec)
        assert coord.read_slot_excluding(9, set()) is None
        assert rec.calls == []
        assert coord._last == {}
        # Unlike read_slot, the excluding read stays silent.
        assert coord._no_reader_warned == set()
        assert printer._afc.logger.messages == []


# ── AFC_OpenAMS_rfid.read_lane ────────────────────────────────────────────────

class TestAFCOpenAMSRfidReadLane:
    def _setup(self, monkeypatch, tag):
        coord, printer, _ = _connected(monkeypatch)
        _freeze_tag_time(monkeypatch)
        rec = _ReadTagRecorder(tag)
        monkeypatch.setattr(oams_rfid, "read_tag", rec)
        applied = []

        def fake_apply(lane, raw):
            applied.append((lane, raw))
            return {"brand": "Applied"}

        monkeypatch.setattr(coord, "apply_to_lane", fake_apply)
        return coord, printer, rec, applied

    def test_unmapped_lane_warns(self, monkeypatch):
        coord, printer, rec, applied = self._setup(monkeypatch, TAG_PLA)
        assert coord.read_lane("lane9") is None
        assert printer._afc.logger.messages == [
            ("warning", "AFC_OpenAMS_rfid: lane 'lane9' has no slot (set lane_slot_map)")]
        assert rec.calls == []
        assert applied == []

    def test_no_tag_records_nothing(self, monkeypatch):
        coord, printer, rec, applied = self._setup(monkeypatch, None)
        assert coord.read_lane("lane1") is None
        assert len(rec.calls) == 1
        assert coord.last_reads_status() == {}
        assert applied == []
        assert printer._afc.logger.messages == []

    def test_undecoded_tag_records_uid_and_type(self, monkeypatch):
        coord, printer, _, applied = self._setup(monkeypatch, TAG_UNDECODED)
        assert coord.read_lane("lane1") is None
        assert coord.last_reads_status() == {"lane1": {
            "uid": "aabb", "tag_type": "MifareClassic1k", "decoded": False,
            "scan_time": SCAN_TIME}}
        assert applied == []
        assert printer._afc.logger.messages == []

    def test_undecoded_tag_with_empty_fields_records_blanks(self, monkeypatch):
        tag = {"uid": None, "tag_type": None, "filament": {}}
        coord, _, _, applied = self._setup(monkeypatch, tag)
        assert coord.read_lane("lane1") is None
        assert coord.last_reads_status() == {"lane1": {
            "decoded": False, "scan_time": SCAN_TIME}}
        assert applied == []

    def test_lane_present_applies_the_tag(self, monkeypatch):
        coord, printer, _, applied = self._setup(monkeypatch, TAG_PLA)
        lane = types.SimpleNamespace(name="lane1")
        printer._afc.lanes["lane1"] = lane
        assert coord.read_lane("lane1") == {"brand": "Applied"}
        assert applied == [(lane, TAG_PLA)]
        # apply_to_lane owns the record, so read_lane adds none itself.
        assert coord.last_reads_status() == {}
        assert printer._afc.logger.messages == []

    def test_lane_not_in_afc_maps_and_records(self, monkeypatch):
        coord, printer, _, applied = self._setup(monkeypatch, TAG_PLA)
        assert coord.read_lane("lane1") == SLOT_INFO_PLA
        assert applied == []
        # The record drops the empty and None fields of the slot_info.
        assert coord.last_reads_status() == {"lane1": {
            "material": "PLA", "color_hex": "112233", "multi_color": ["112233"],
            "is_dual_color": False, "brand": "Bambu", "diameter": 1.75,
            "uid": "aabbccdd", "tag_type": "MifareClassic1k", "decoded": True,
            "scan_time": SCAN_TIME}}
        assert printer._afc.logger.messages == []

    def test_before_connect_maps_without_applying(self, monkeypatch):
        coord, _, _, applied = self._setup(monkeypatch, TAG_PLA)
        coord.afc = None
        assert coord.read_lane("lane1") == SLOT_INFO_PLA
        assert applied == []
        assert coord.last_reads_status()["lane1"]["decoded"] is True

    def test_afc_without_lanes_maps_without_applying(self, monkeypatch):
        coord, _, _, applied = self._setup(monkeypatch, TAG_PLA)
        coord.afc = types.SimpleNamespace()
        assert coord.read_lane("lane1") == SLOT_INFO_PLA
        assert applied == []
        assert coord.last_reads_status()["lane1"]["decoded"] is True


# ── AFC_OpenAMS_rfid._map ─────────────────────────────────────────────────────

class TestAFCOpenAMSRfidMap:
    def test_maps_basic_tag(self):
        coord, _ = _make_coord(values=dict(KEYS))
        assert coord._map(TAG_PLA) == SLOT_INFO_PLA

    def test_maps_hotend_range_to_midpoint(self):
        coord, _ = _make_coord(values=dict(KEYS))
        tag = {"uid": "01020304", "tag_type": None,
               "filament": {"type": "PETG", "hotend_min_c": 190,
                            "hotend_max_c": 230, "bed_temp_c": 70}}
        info = coord._map(tag)
        assert info["material"] == "PETG"
        assert info["extruder_temp"] == 210
        assert info["extruder_temp_min"] == 190
        assert info["extruder_temp_max"] == 230
        assert info["bed_temp"] == 70
        assert info["uid"] == "01020304"
        assert "tag_type" not in info


# ── AFC_OpenAMS_rfid.get_status ───────────────────────────────────────────────

class TestAFCOpenAMSRfidGetStatus:
    def test_before_connect(self):
        coord, _ = _make_coord(values={**KEYS, "lane_slot_map": "lane4:1"})
        assert coord.get_status() == {"slots": [], "lane_slot_map": {"lane4": 1},
                                      "last_reads": {}}

    def test_reports_sorted_slots_map_copy_and_reads(self, monkeypatch):
        coord, _, _ = _connected(
            monkeypatch, readers=(("RFID_B", "3, 2"), ("RFID_A", "1")),
            lane_map="lane2:3, lane1:1")
        _freeze_tag_time(monkeypatch)
        coord.record_tag_read("lane1", None, decoded=False, uid="aabb")
        status = coord.get_status(eventtime=5.0)
        assert status == {
            "slots": [1, 2, 3],
            "lane_slot_map": {"lane2": 3, "lane1": 1},
            "last_reads": {"lane1": {"uid": "aabb", "decoded": False,
                                     "scan_time": SCAN_TIME}},
        }
        status["lane_slot_map"]["lane9"] = 0
        assert coord._lane_slot == {"lane2": 3, "lane1": 1}


# ── AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ ───────────────────────────────────────

class TestAFCOpenAMSRfidCmdOamsRfidRead:
    def _setup(self, monkeypatch, tag):
        coord, printer, _ = _connected(monkeypatch)
        _freeze_tag_time(monkeypatch)
        rec = _ReadTagRecorder(tag)
        monkeypatch.setattr(oams_rfid, "read_tag", rec)
        return coord, printer, rec

    def test_requires_lane_or_slot(self, monkeypatch):
        coord, _, rec = self._setup(monkeypatch, TAG_PLA)
        gcmd = MockGCodeCommand(params={})
        with pytest.raises(CommandError) as exc:
            coord.cmd_OAMS_RFID_READ(gcmd)
        assert str(exc.value) == "OAMS_RFID_READ requires LANE= or SLOT="
        gcmd.error.assert_called_once_with("OAMS_RFID_READ requires LANE= or SLOT=")
        gcmd.respond_info.assert_not_called()
        assert rec.calls == []

    def test_lane_decoded_reports_brand_and_material(self, monkeypatch):
        coord, _, rec = self._setup(monkeypatch, TAG_PLA)
        gcmd = MockGCodeCommand(params={"LANE": "lane1"})
        coord.cmd_OAMS_RFID_READ(gcmd)
        gcmd.respond_info.assert_called_once_with("OpenAMS RFID: lane1 -> Bambu PLA")
        assert len(rec.calls) == 1

    def test_lane_decoded_without_brand_or_material(self, monkeypatch):
        coord, _, _ = self._setup(monkeypatch, TAG_PLA)
        monkeypatch.setattr(coord, "read_lane", lambda name: {"uid": "aabbccdd"})
        gcmd = MockGCodeCommand(params={"LANE": "lane1"})
        coord.cmd_OAMS_RFID_READ(gcmd)
        gcmd.respond_info.assert_called_once_with("OpenAMS RFID: lane1 ->  ")

    def test_lane_undecoded_reports_what_was_seen(self, monkeypatch):
        coord, _, _ = self._setup(monkeypatch, TAG_UNDECODED)
        gcmd = MockGCodeCommand(params={"LANE": "lane1"})
        coord.cmd_OAMS_RFID_READ(gcmd)
        gcmd.respond_info.assert_called_once_with(
            "OpenAMS RFID: no tag decoded on lane1 (saw tag UID aabb, "
            "MifareClassic1k, no decoder/key matched)")

    def test_lane_without_tag_reports_none_decoded(self, monkeypatch):
        coord, _, _ = self._setup(monkeypatch, None)
        gcmd = MockGCodeCommand(params={"LANE": "lane1"})
        coord.cmd_OAMS_RFID_READ(gcmd)
        gcmd.respond_info.assert_called_once_with("OpenAMS RFID: no tag decoded on lane1")

    def test_lane_wins_over_slot(self, monkeypatch):
        coord, _, rec = self._setup(monkeypatch, TAG_PLA)
        gcmd = MockGCodeCommand(params={"LANE": "lane1", "SLOT": "3"})
        coord.cmd_OAMS_RFID_READ(gcmd)
        gcmd.respond_info.assert_called_once_with("OpenAMS RFID: lane1 -> Bambu PLA")
        assert len(rec.calls) == 1
        # lane1 maps to slot 0, so the slot-3 reader was never read.
        assert coord._last == {0: TAG_PLA}

    def test_slot_with_tag_reports_mapped_info(self, monkeypatch):
        coord, _, rec = self._setup(monkeypatch, TAG_PLA)
        gcmd = MockGCodeCommand(params={"SLOT": "3"})
        coord.cmd_OAMS_RFID_READ(gcmd)
        gcmd.respond_info.assert_called_once_with(f"OpenAMS RFID: slot 3 -> {SLOT_INFO_PLA}")
        assert coord._last == {3: TAG_PLA}
        # A raw slot read is reported only, never recorded against a lane.
        assert coord.last_reads_status() == {}

    def test_slot_without_tag_reports_none(self, monkeypatch):
        coord, _, _ = self._setup(monkeypatch, None)
        gcmd = MockGCodeCommand(params={"SLOT": "0"})
        coord.cmd_OAMS_RFID_READ(gcmd)
        gcmd.respond_info.assert_called_once_with("OpenAMS RFID: slot 0 -> None")


# ── load_config / load_config_prefix ──────────────────────────────────────────

class TestLoadConfig:
    def test_builds_the_coordinator(self):
        printer = MockPrinter()
        config = MockConfig(name="AFC_OpenAMS_rfid", printer=printer,
                            values={**KEYS, "lane_slot_map": "lane4:0"})
        coord = oams_rfid.load_config(config)
        assert isinstance(coord, oams_rfid.AFC_OpenAMS_rfid)
        assert coord._lane_slot == {"lane4": 0}
        assert printer._event_handlers == {"klippy:connect": [coord._on_connect]}


class TestLoadConfigPrefix:
    def test_builds_a_reader(self, monkeypatch):
        built = _install_bus(monkeypatch)
        config = MockConfig(name="AFC_OpenAMS_rfid RFID_B", printer=MockPrinter(),
                            values={"slots": "2, 3"})
        reader = oams_rfid.load_config_prefix(config)
        assert isinstance(reader, oams_rfid.AFC_OpenAMS_rfid_reader)
        assert reader.name == "RFID_B"
        assert reader.slots == [2, 3]
        assert reader.spi is built[0]["spi"]
