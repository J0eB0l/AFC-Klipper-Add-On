"""Unit tests for extras/afc_dryer.py."""

from __future__ import annotations

import hashlib
from http.server import BaseHTTPRequestHandler
import io
import json
import logging
import sys
import threading
from types import SimpleNamespace
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple
from unittest.mock import MagicMock

import pytest

from extras.afc_dryer import (
    _AceBackend,
    _Backend,
    _BACKENDS,
    _BambuBackend,
    _css_color,
    _env_reading,
    _GenericBackend,
    _lane_bays,
    _make_handler,
    _merge_bays,
    _PAGE_TEMPLATE,
    _reading,
    _secs,
    AFCDryer,
    load_config,
    PAGE_VERSION,
)
import extras.afc_dryer as dryer_mod
from tests.conftest import MockConfig, MockLogger, MockPrinter


DRYER_ACE_BACKEND = _BACKENDS[1]


DRYER_EMPTY_BAY = {"present": False, "color": "", "material": ""}


class DryerBambuUnit:
    """The attributes _BambuBackend reads from an AFC_BambuAMS unit."""

    PREFIX = "AFC_BambuAMS"

    def __init__(self, name: str = "AMS_2", status: Optional[Dict[str, Any]] = None,
                 model: Any = "ams2", heater: bool = True, max_temp: Any = 65,
                 slots: Any = 4) -> None:
        self.name = name
        self.ams_model = model
        self.has_heater = heater
        self.dry_max_temp = max_temp
        self.unit_slots = slots
        self.status = status if status is not None else {}

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        return self.status


class DryerAceUnit:
    """A V1 ACE: no ACE2 in the class name, so _is_v2 says False."""

    PREFIX = "AFC_ACE"
    SLOTS_PER_UNIT = 4

    def __init__(self, name: str = "Ace_1", status: Optional[Dict[str, Any]] = None,
                 max_temp: Any = 55.0, printer: Any = None) -> None:
        self.name = name
        self.max_dryer_temperature = max_temp
        self.status = status if status is not None else {}
        self.printer = printer

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        return self.status


class DryerAce2Unit(DryerAceUnit):
    """An ACE 2 Pro: ACE2 in the class name marks the second generation."""

    PREFIX = "AFC_ACE2"


class DryerOtherUnit:
    """A BoxTurtle-like unit: registered with AFC, no dryer, no vendor backend."""

    def __init__(self, name: str = "Turtle_1", type_: Optional[str] = "BoxTurtle",
                 lanes: Optional[Dict[str, Any]] = None, printer: Any = None,
                 oams_name: Optional[str] = None) -> None:
        self.name = name
        self.type = type_
        self.lanes = lanes if lanes is not None else {}
        self.printer = printer
        self.oams_name = oams_name


class DryerLane:
    """The lane attributes the panel reads for bay colour and the tooltip."""

    def __init__(self, index: Any = 1, color: Any = "", material: str = "",
                 load_state: bool = False, filament_name: str = "", sub_type: str = "",
                 spool_vendor: str = "", spool_id: Optional[int] = None,
                 weight: Any = 0.0, extruder_temp: Optional[int] = None) -> None:
        self.index = index
        self.color = color
        self.material = material
        self.load_state = load_state
        self.filament_name = filament_name
        self.sub_type = sub_type
        self.spool_vendor = spool_vendor
        self.spool_id = spool_id
        self.weight = weight
        self.extruder_temp = extruder_temp


class DryerSensor:
    """A Klipper sensor object: returns a copy of its status, or raises."""

    def __init__(self, status: Optional[Dict[str, Any]] = None,
                 error: Optional[Exception] = None) -> None:
        self.status = status
        self.error = error

    def get_status(self, eventtime: Optional[float] = None) -> Optional[Dict[str, Any]]:
        if self.error is not None:
            raise self.error
        return None if self.status is None else dict(self.status)


class DryerSock:
    """Socket stand-in for one HTTP request: reads fixed bytes, records sends."""

    def __init__(self, raw: bytes) -> None:
        self.raw = raw
        self.sent: List[bytes] = []

    def makefile(self, mode: str, bufsize: int = -1) -> io.BytesIO:
        return io.BytesIO(self.raw)

    def sendall(self, data: bytes) -> None:
        self.sent.append(bytes(data))


class DryerWriter:
    """wfile stand-in; the write numbered fail_on (1-based) raises."""

    def __init__(self, fail_on: Optional[int] = None) -> None:
        self.chunks: List[bytes] = []
        self.fail_on = fail_on

    def write(self, data: bytes) -> None:
        if self.fail_on == len(self.chunks) + 1:
            raise BrokenPipeError("client went away")
        self.chunks.append(bytes(data))

    def flush(self) -> None:
        pass


def dryer_sensor_printer(objects: Dict[str, Any]) -> MockPrinter:
    """
    A MockPrinter holding the given named sensor objects.

    :param objects: Klipper object name -> object
    :return MockPrinter: the printer
    """
    printer = MockPrinter()
    printer._objects.update(objects)
    return printer


def make_dryer_panel(*units: Any, values: Optional[Dict[str, Any]] = None,
                     afc_units: Optional[Dict[str, Any]] = None
                     ) -> Tuple[AFCDryer, MockPrinter]:
    """
    Build an AFCDryer through its real __init__ on a MockPrinter.

    Each unit is registered under "<PREFIX> <name>", as Klipper names it. The
    reactor's timer and async-callback hooks and gcode.run_script are mocks.

    :param units: unit fakes to register as printer objects
    :param values: [afc_dryer] config values
    :param afc_units: entries for AFC's own unit registry
    :return tuple: the panel and its printer
    """
    printer = MockPrinter()
    for unit in units:
        printer._objects[f"{unit.PREFIX} {unit.name}"] = unit
    printer._afc.units.update(afc_units or {})
    printer._reactor.register_timer = MagicMock(return_value="timer")
    printer._reactor.register_async_callback = MagicMock()
    printer._gcode.run_script = MagicMock()
    panel = AFCDryer(MockConfig(printer=printer, values=dict(values or {})))
    return panel, printer


def discovered_dryer_panel(*units: Any, values: Optional[Dict[str, Any]] = None,
                           afc_units: Optional[Dict[str, Any]] = None
                           ) -> Tuple[AFCDryer, MockPrinter]:
    """
    A panel that has run discovery, holding a fresh MockLogger afterwards.

    :param units: unit fakes to register as printer objects
    :param values: [afc_dryer] config values
    :param afc_units: entries for AFC's own unit registry
    :return tuple: the panel and its printer
    """
    panel, printer = make_dryer_panel(*units, values=values, afc_units=afc_units)
    panel.logger = MockLogger()
    panel._discover()
    panel.logger = MockLogger()
    return panel, printer


def dryer_bambu_row(name: str = "AMS_2", **over: Any) -> Dict[str, Any]:
    """
    The published row of an idle AMS 2 Pro, with the given fields replaced.

    :param name: unit name
    :param over: fields that differ from the idle defaults
    :return dict: the expected row
    """
    row = {"name": name, "kind": "bambu", "model": "ams2", "label": "AMS 2 Pro",
           "max_temp": 65, "slots": 4, "has_heater": True, "rotate": True,
           "online": False, "drying": False, "temperature": None, "humidity": None,
           "target": None, "remaining": None, "duration": None, "note": "",
           "error": ""}
    row.update(over)
    row.setdefault("bays", [dict(DRYER_EMPTY_BAY) for _ in range(row["slots"])])
    return row


def dryer_queued_callbacks(printer: MockPrinter) -> List[Callable[[float], None]]:
    """
    The callbacks the panel handed to the reactor, oldest first.

    :param printer: the panel's printer
    :return list: the queued callbacks
    """
    return [c.args[0] for c in printer._reactor.register_async_callback.call_args_list]


def dryer_page_version() -> str:
    """
    The page version recomputed here: the first 8 hex digits of the template's MD5.

    :return str: the expected version
    """
    return hashlib.md5(_PAGE_TEMPLATE.encode("utf-8")).hexdigest()[:8]


@pytest.fixture
def dryer_fake_net(monkeypatch: pytest.MonkeyPatch) -> SimpleNamespace:
    """
    Replace the HTTP server, threading and chelper inside afc_dryer.

    Nothing binds a port or starts a thread; the recorder exposes what was
    built and lets a test make the server, thread start or ffi fail.
    """
    rec = SimpleNamespace(servers=[], threads=[], names=[], server_error=None,
                          start_error=None, ffi_error=None)

    class Server:
        def __init__(self, address: Tuple[str, int], handler: type) -> None:
            if rec.server_error is not None:
                raise rec.server_error
            self.address = address
            self.handler = handler
            self.daemon_threads = False
            self.calls: List[str] = []
            self.shutdown_error: Optional[Exception] = None
            rec.servers.append(self)

        def serve_forever(self) -> None:
            self.calls.append("serve_forever")

        def shutdown(self) -> None:
            self.calls.append("shutdown")
            if self.shutdown_error is not None:
                raise self.shutdown_error

        def server_close(self) -> None:
            self.calls.append("server_close")

    class Thread:
        def __init__(self, target: Optional[Callable[[], None]] = None,
                     daemon: Optional[bool] = None, name: Optional[str] = None) -> None:
            self.target = target
            self.daemon = daemon
            self.name = name
            self.started = False
            rec.threads.append(self)

        def start(self) -> None:
            if rec.start_error is not None:
                raise rec.start_error
            self.started = True

    def get_ffi() -> Tuple[None, SimpleNamespace]:
        if rec.ffi_error is not None:
            raise rec.ffi_error
        return None, SimpleNamespace(set_thread_name=rec.names.append)

    monkeypatch.setattr(dryer_mod, "ThreadingHTTPServer", Server)
    monkeypatch.setattr(dryer_mod, "threading", SimpleNamespace(
        Thread=Thread, Lock=threading.Lock,
        current_thread=lambda: SimpleNamespace(name="worker-7")))
    monkeypatch.setattr(dryer_mod, "chelper", SimpleNamespace(get_ffi=get_ffi))
    return rec


def dryer_parse(raw: bytes) -> Tuple[str, Dict[str, str], bytes]:
    """
    Split a raw HTTP response into status line, headers and body.

    :param raw: the bytes written to the socket
    :return tuple: status line, header dict, body
    """
    head, _, body = raw.partition(b"\r\n\r\n")
    status, *lines = head.decode("latin-1").split("\r\n")
    headers = dict(line.split(": ", 1) for line in lines)
    return status, headers, body


def dryer_app_headers(headers: Dict[str, str]) -> Dict[str, str]:
    """
    Response headers minus the two the stdlib adds (Server, Date).

    :param headers: parsed response headers
    :return dict: the headers the panel itself sets
    """
    return {k: v for k, v in headers.items() if k not in ("Server", "Date")}


def dryer_json_headers(body: bytes) -> Dict[str, str]:
    """
    The headers the panel sends with a JSON body.

    :param body: the response body
    :return dict: expected headers
    """
    return {"Content-Type": "application/json", "Content-Length": str(len(body)),
            "Access-Control-Allow-Origin": "*", "Cache-Control": "no-store"}


def dryer_request(panel: AFCDryer, method: str, path: str, headers: Iterable[str] = (),
                  body: bytes = b"") -> Tuple[BaseHTTPRequestHandler,
                                              Tuple[str, Dict[str, str], bytes]]:
    """
    Run one request through the real handler class for this panel.

    :param panel: the panel to serve
    :param method: HTTP method
    :param path: request path
    :param headers: raw header lines
    :param body: request body
    :return tuple: the handler and the parsed response
    """
    lines = [f"{method} {path} HTTP/1.0", *headers, "", ""]
    sock = DryerSock("\r\n".join(lines).encode("latin-1") + body)
    handler = _make_handler(panel)(sock, ("127.0.0.1", 50000), SimpleNamespace())
    return handler, dryer_parse(b"".join(sock.sent))


class TestCssColor:
    def test_bambu_hex_string(self):
        assert _css_color("FF8800") == "#FF8800"
        # A leading # and padding are tolerated, and the case is kept.
        assert _css_color(" #ff8800 ") == "#ff8800"

    def test_bambu_hex_with_alpha_is_truncated(self):
        assert _css_color("FF8800FF") == "#FF8800"

    def test_ace_rgb_list(self):
        assert _css_color([200, 30, 30]) == "rgb(200,30,30)"
        # An RGBA list keeps only its first three channels.
        assert _css_color([10, 20, 30, 255]) == "rgb(10,20,30)"

    def test_black_is_empty_not_black(self):
        # Both vendors use all-zero for "no colour known".
        assert _css_color("000000") == ""
        assert _css_color([0, 0, 0]) == ""

    # "fff" and "#ABC" are valid hex, so only the length check rejects them.
    @pytest.mark.parametrize("bad", [None, "", "zz", "fff", "#ABC", "GGGGGG", [1, 2], 42,
                                     {}])
    def test_unusable_values_are_empty(self, bad):
        assert _css_color(bad) == ""

    def test_rgb_values_are_clamped(self):
        assert _css_color([999, -5, 30]) == "rgb(255,0,30)"

    def test_a_non_numeric_channel_is_unusable(self):
        # int("red") raises ValueError inside the generator.
        assert _css_color(["red", 0, 0]) == ""

    def test_a_none_channel_is_unusable(self):
        # int(None) raises TypeError, the other half of the except tuple.
        assert _css_color([None, 10, 10]) == ""

    def test_a_tuple_is_read_like_a_list(self):
        assert _css_color((1, 2, 3)) == "rgb(1,2,3)"

    # int(v, 16) accepts each of these, but none is a CSS colour.
    @pytest.mark.parametrize("bad", ["-12345", "+12345", "1_2345", "12345 6"])
    def test_only_six_hex_digits_are_a_colour(self, bad):
        assert _css_color(bad) == ""


class TestLaneBays:
    BLANK = {"color": "", "material": ""}

    @staticmethod
    def _one(lane: DryerLane) -> Dict[str, Any]:
        """
        The single bay a one-bay unit gets from this lane, mapped to bay 0.

        :param lane: the lane in bay 0
        :return dict: that bay
        """
        unit = SimpleNamespace(lanes={"lane1": lane}, _slot_map={"lane1": 0})
        bays = _lane_bays(unit, 1)
        assert len(bays) == 1
        return bays[0]

    @staticmethod
    def _bay(lane: str, **over: Any) -> Dict[str, Any]:
        """
        A lane bay with blank tooltip detail and the given fields replaced.

        :param lane: lane name
        :param over: fields that differ from the blanks
        :return dict: the expected bay
        """
        bay = {"color": "#ff0000", "material": "PLA", "lane": lane, "filament": "",
               "sub_type": "", "vendor": "", "spool_id": None, "weight": None,
               "temp": None}
        bay.update(over)
        return bay

    def test_the_lane_name_is_carried(self):
        lane = DryerLane(color="#ff0000", material="PLA")
        assert self._one(lane) == self._bay("lane1")

    def test_spoolman_identity_is_carried(self):
        lane = DryerLane(color="#ff0000", material="PLA", filament_name="Galaxy Black",
                         spool_vendor="Polymaker", sub_type="Matte", spool_id=42)
        assert self._one(lane) == self._bay(
            "lane1", filament="Galaxy Black", vendor="Polymaker", sub_type="Matte",
            spool_id=42)

    def test_a_tracked_weight_is_carried(self):
        lane = DryerLane(color="#ff0000", material="PLA", weight=812.34)
        assert self._one(lane) == self._bay("lane1", weight=812.3)

    def test_an_untracked_weight_is_omitted_not_zeroed(self):
        # 0 means "not tracked", not "empty spool".
        lane = DryerLane(color="#ff0000", material="PLA", weight=0.0)
        assert self._one(lane) == self._bay("lane1", weight=None)

    def test_a_junk_weight_does_not_raise(self):
        lane = DryerLane(color="#ff0000", material="PLA", weight="n/a")
        assert self._one(lane) == self._bay("lane1")

    def test_the_extruder_temp_is_carried(self):
        lane = DryerLane(color="#ff0000", material="PLA", extruder_temp=230)
        assert self._one(lane) == self._bay("lane1", temp=230)

    def test_each_lane_lands_in_its_own_bay(self):
        lanes = {"lane1": DryerLane(1, "#ff0000", "PLA", spool_id=1, weight=100.0),
                 "lane2": DryerLane(2, "#00ff00", "PETG", spool_id=2, weight=200.0),
                 "lane3": DryerLane(3, "#0000ff", "ABS", spool_id=3, weight=300.0),
                 "lane4": DryerLane(4, "#ffff00", "TPU", spool_id=4, weight=400.0)}
        unit = SimpleNamespace(
            lanes=lanes, _slot_map={"lane1": 0, "lane2": 1, "lane3": 2, "lane4": 3})
        assert _lane_bays(unit, 4) == [
            self._bay("lane1", spool_id=1, weight=100.0),
            self._bay("lane2", color="#00ff00", material="PETG", spool_id=2,
                      weight=200.0),
            self._bay("lane3", color="#0000ff", material="ABS", spool_id=3,
                      weight=300.0),
            self._bay("lane4", color="#ffff00", material="TPU", spool_id=4,
                      weight=400.0)]

    def test_the_index_fallback_also_separates_them(self):
        # No _slot_map: the lane's 1-based index decides, listed out of order.
        unit = SimpleNamespace(lanes={"b": DryerLane(2, "#00ff00", "PETG"),
                                      "a": DryerLane(1, "#ff0000", "PLA")})
        assert _lane_bays(unit, 2) == [
            self._bay("a"), self._bay("b", color="#00ff00", material="PETG")]

    def test_a_lane_with_no_index_is_skipped(self):
        unit = SimpleNamespace(lanes={
            "bad": SimpleNamespace(index=None, color="#FF0000"),
            "good": SimpleNamespace(index=1, color="#00FF00", material="PLA")})
        assert _lane_bays(unit, 2) == [
            self._bay("good", color="#00FF00"), self.BLANK]

    def test_a_lane_with_a_junk_index_is_skipped(self):
        unit = SimpleNamespace(lanes={
            "bad": SimpleNamespace(index="first", color="#FF0000"),
            "good": SimpleNamespace(index=1, color="#00FF00", material="PLA")})
        assert _lane_bays(unit, 2) == [
            self._bay("good", color="#00FF00"), self.BLANK]

    def test_index_zero_does_not_wrap_to_the_last_bay(self):
        # Index 0 gives slot -1, which would otherwise write bays[-1].
        unit = SimpleNamespace(lanes={
            "zero": SimpleNamespace(index=0, color="#FF0000"),
            "one": SimpleNamespace(index=1, color="#00FF00", material="PLA")})
        assert _lane_bays(unit, 2) == [
            self._bay("one", color="#00FF00"), self.BLANK]


class TestMergeBays:
    def test_lane_detail_survives_the_merge(self):
        # Bay 0 carries every tooltip field; bay 1's blank ones ("" or None)
        # are left out, not copied.
        merged = _merge_bays(
            [{"present": True}, {"present": True}],
            [{"color": "#fff", "material": "PLA", "lane": "lane1",
              "filament": "Galaxy Black", "sub_type": "Matte", "vendor": "Polymaker",
              "spool_id": 7, "weight": 500.0, "temp": 220},
             {"color": "#000", "material": "PETG", "lane": "lane2", "filament": "",
              "spool_id": 8, "weight": 250.0, "temp": None}])
        assert merged == [
            {"present": True, "color": "#fff", "material": "PLA", "lane": "lane1",
             "filament": "Galaxy Black", "sub_type": "Matte", "vendor": "Polymaker",
             "spool_id": 7, "weight": 500.0, "temp": 220},
            {"present": True, "color": "#000", "material": "PETG", "lane": "lane2",
             "spool_id": 8, "weight": 250.0}]

    def test_absent_detail_is_not_invented(self):
        merged = _merge_bays([{"present": False}], [{}])
        assert merged == [{"present": False, "color": "", "material": ""}]

    def test_a_vendor_bay_beyond_the_lane_list_keeps_vendor_data(self):
        vendor = [{"present": True, "color": "#111111"},
                  {"present": 1, "color": "#0000FF", "material": "PETG"}]
        lane = [{"color": "#FF0000", "material": "PLA", "lane": "lane1"}]
        assert _merge_bays(vendor, lane) == [
            {"present": True, "color": "#FF0000", "material": "PLA", "lane": "lane1"},
            {"present": True, "color": "#0000FF", "material": "PETG"}]


class TestSecs:
    def test_a_numeric_string_is_converted(self):
        assert _secs("12") == 12

    def test_junk_text_is_none(self):
        assert _secs("soon") is None

    def test_none_is_none(self):
        assert _secs(None) is None


class TestReading:
    def test_zero_is_absent(self):
        assert _reading(0.0) is None
        assert _reading(0) is None

    def test_a_real_reading_survives(self):
        assert _reading(29.79) == 29.79
        assert _reading(42.91) == 42.91

    def test_none_stays_none(self):
        assert _reading(None) is None

    def test_a_negative_reading_is_kept(self):
        # Absent is 0, not "falsy in general".
        assert _reading(-3.5) == -3.5

    def test_a_string_number_is_accepted(self):
        # Vendor status payloads are not always typed.
        assert _reading("29.8") == 29.8

    def test_junk_is_absent_not_a_crash(self):
        assert _reading("n/a") is None
        assert _reading(object()) is None

    def test_one_channel_absent_does_not_hide_the_other(self):
        # The HT case: temperature unread, humidity live.
        assert (_reading(0.0), _reading(41.0)) == (None, 41.0)

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf"), "nan"])
    def test_a_non_finite_reading_is_absent(self, bad):
        # json.dumps would write NaN/Infinity, which the page cannot parse.
        assert _reading(bad) is None


class TestBackendFind:
    def test_a_failing_prefix_does_not_hide_the_next_one(self, monkeypatch):
        ace1, ace2 = DryerAceUnit("Ace_1"), DryerAce2Unit("Ace2_1")
        printer = dryer_sensor_printer({"AFC_ACE Ace_1": ace1, "AFC_ACE2 Ace2_1": ace2})
        real = printer.lookup_objects

        def lookup(module: Optional[str] = None) -> List[Tuple[str, Any]]:
            if module == "AFC_ACE":
                raise RuntimeError("bus busy")
            return real(module)

        monkeypatch.setattr(printer, "lookup_objects", lookup)
        assert _AceBackend().find(printer) == [ace2]


class TestBackendHasHeater:
    def test_it_is_abstract(self):
        with pytest.raises(NotImplementedError):
            _Backend().has_heater(DryerBambuUnit())


class TestBackendDescribe:
    def test_it_is_abstract(self):
        with pytest.raises(NotImplementedError):
            _Backend().describe(DryerBambuUnit())


class TestBackendSnapshot:
    def test_it_is_abstract(self):
        with pytest.raises(NotImplementedError):
            _Backend().snapshot(DryerBambuUnit(), {})


class TestBackendSlots:
    def test_it_is_abstract(self):
        with pytest.raises(NotImplementedError):
            _Backend().slots(DryerBambuUnit(), {})


class TestBackendStartScript:
    def test_it_is_abstract(self):
        with pytest.raises(NotImplementedError):
            _Backend().start_script("AMS_2", 55, 60, 0)


class TestBackendStopScript:
    def test_it_is_abstract(self):
        with pytest.raises(NotImplementedError):
            _Backend().stop_script("AMS_2")


class TestBambuBackendDescribe:
    def test_a_blank_model_reads_as_an_ams_2_pro(self):
        unit = SimpleNamespace(ams_model=None)
        assert _BambuBackend().describe(unit) == {
            "model": "ams2", "label": "AMS 2 Pro", "max_temp": 65, "slots": 4}

    def test_the_model_is_lowercased_before_lookup(self):
        unit = SimpleNamespace(ams_model="HT", dry_max_temp="85", unit_slots="1")
        assert _BambuBackend().describe(unit) == {
            "model": "ht", "label": "AMS HT", "max_temp": 85, "slots": 1}

    def test_an_unknown_model_is_labelled_ams(self):
        unit = SimpleNamespace(ams_model="ams1", dry_max_temp=0, unit_slots=4)
        assert _BambuBackend().describe(unit) == {
            "model": "ams1", "label": "AMS", "max_temp": 0, "slots": 4}


class TestBambuBackendSnapshot:
    """An AMS that refuses to dry echoes the temp/time back first, so the start
    command is delivered and reports success. The refusal reason is the only
    thing that tells the card why the unit sits at "not drying"."""

    @staticmethod
    def _snap(**over: Any) -> Dict[str, Any]:
        """
        An idle, offline snapshot with the given fields replaced.

        :param over: fields that differ from the idle defaults
        :return dict: the expected snapshot
        """
        snap = {"online": False, "drying": False, "temperature": None,
                "humidity": None, "target": None, "remaining": None,
                "duration": None, "note": "", "error": ""}
        snap.update(over)
        return snap

    def test_the_reason_reaches_the_card(self):
        snap = _BambuBackend().snapshot(
            DryerBambuUnit(), {"bridge_online": True, "drying": False,
                               "dry_error": "filament hub load!"})
        assert snap == self._snap(online=True, error="filament hub load!")

    def test_no_refusal_leaves_it_empty(self):
        snap = _BambuBackend().snapshot(
            DryerBambuUnit(), {"bridge_online": True, "drying": False})
        assert snap == self._snap(online=True)

    def test_the_reason_is_its_OWN_field_not_the_note(self):
        # The note describes a running cycle; the error is the refusal. With
        # no note the reason must not leak into it.
        snap = _BambuBackend().snapshot(
            DryerBambuUnit(), {"bridge_online": True, "drying": True,
                               "dry_error": "filament hub load!"})
        assert snap == self._snap(online=True, drying=True, error="filament hub load!")
        snap = _BambuBackend().snapshot(
            DryerBambuUnit(), {"bridge_online": True, "drying": True,
                               "dry_note": "ams-ht shell open!",
                               "dry_error": "filament hub load!"})
        assert snap == self._snap(online=True, drying=True, note="ams-ht shell open!",
                                  error="filament hub load!")

    def test_the_reason_survives_the_drying_flag_being_set(self):
        # A refused start leaves drying True, the case the reason exists for.
        snap = _BambuBackend().snapshot(
            DryerBambuUnit(), {"bridge_online": True, "drying": True,
                               "dry_error": "filament hub load!"})
        assert snap == self._snap(online=True, drying=True, error="filament hub load!")

    def test_a_running_cycle_shows_no_stale_reason(self):
        # dry_minutes is minutes and dry_remaining seconds: 480 min is 28800 s.
        snap = _BambuBackend().snapshot(
            DryerBambuUnit(), {"bridge_online": True, "drying": True,
                               "dry_error": None, "temperature": 44.0,
                               "humidity": 18.0, "dry_target": 55,
                               "dry_remaining": 1800, "dry_minutes": 480})
        assert snap == self._snap(online=True, drying=True, temperature=44.0,
                                  humidity=18.0, target=55, remaining=1800,
                                  duration=28800)


class TestBambuBackendSlots:
    def test_a_missing_slot_record_reads_as_empty(self):
        st = {"slots": [None, {"present": 1, "color": [255, 0, 0], "material": None},
                        {"present": True, "color": "FF8800", "material": "PLA"}]}
        assert _BambuBackend().slots(DryerBambuUnit(), st) == [
            DRYER_EMPTY_BAY,
            {"present": True, "color": "rgb(255,0,0)", "material": ""},
            {"present": True, "color": "#FF8800", "material": "PLA"}]

    def test_no_slot_list_gives_no_bays(self):
        assert _BambuBackend().slots(DryerBambuUnit(), {"slots": None}) == []


class TestAceBackendCycleLength:
    def test_an_ace_2_zero_duration_is_none(self):
        assert _AceBackend()._cycle_length(DryerAce2Unit(), 0) is None

    def test_an_ace_2_missing_duration_is_none(self):
        assert _AceBackend()._cycle_length(DryerAce2Unit(), None) is None

    def test_an_ace_2_duration_is_already_seconds(self):
        assert _AceBackend()._cycle_length(DryerAce2Unit(), "7200") == 7200

    def test_a_v1_duration_is_minutes(self):
        # 8 minutes on a V1 is 480 seconds; an ACE 2 would have said 8.
        assert _AceBackend()._cycle_length(DryerAceUnit(), "8") == 480


class TestAceBackendSnapshot:
    """The V1 ACE has no humidity sensor. An owner who wires one beside it gets
    the reading on the card through _env_reading, for the missing channel only."""

    @staticmethod
    def _unit(objects: Dict[str, Any], name: str = "Ace_1") -> DryerAceUnit:
        """
        A V1 ACE whose printer holds the given sensor objects.

        :param objects: Klipper object name -> sensor
        :param name: unit name
        :return DryerAceUnit: the unit
        """
        return DryerAceUnit(name, printer=dryer_sensor_printer(objects))

    @staticmethod
    def _snap(**over: Any) -> Dict[str, Any]:
        """
        An idle, offline ACE snapshot with the given fields replaced.

        :param over: fields that differ from the idle defaults
        :return dict: the expected snapshot
        """
        snap = {"online": False, "drying": False, "temperature": None,
                "humidity": None, "target": None, "remaining": None,
                "duration": None, "note": ""}
        snap.update(over)
        return snap

    def test_a_v1_gets_its_humidity_from_the_sensor(self):
        unit = self._unit({"aht10 Ace_1": DryerSensor({"temperature": 27.0,
                                                       "humidity": 44.0})})
        snap = _AceBackend().snapshot(unit, {"ace_temp": 27, "ace_connected": True})
        assert snap == self._snap(online=True, temperature=27.0, humidity=44.0)

    def test_the_units_own_reading_still_wins(self):
        # An ACE 2 reports humidity itself; a nearby sensor must not override it.
        unit = self._unit({"aht10 Ace_1": DryerSensor({"temperature": 99.0,
                                                       "humidity": 11.0})})
        snap = _AceBackend().snapshot(unit, {"ace_temp": 27, "ace_humidity": 39})
        assert snap == self._snap(temperature=27.0, humidity=39.0)

    def test_only_the_missing_channel_is_filled(self):
        # Temperature comes from the unit, humidity from the sensor.
        unit = self._unit({"aht10 Ace_1": DryerSensor({"temperature": 99.0,
                                                       "humidity": 44.0})})
        snap = _AceBackend().snapshot(unit, {"ace_temp": 27})
        assert snap == self._snap(temperature=27.0, humidity=44.0)

    def test_no_sensor_is_still_a_dash_not_a_zero(self):
        snap = _AceBackend().snapshot(self._unit({}), {"ace_temp": 27})
        assert snap == self._snap(temperature=27.0)

    def test_the_dryer_state_is_untouched_by_any_of_this(self):
        unit = self._unit({"aht10 Ace_1": DryerSensor({"temperature": 27.0,
                                                       "humidity": 44.0})})
        snap = _AceBackend().snapshot(unit, {"ace_dryer": "drying",
                                             "ace_connected": True})
        assert snap == self._snap(online=True, drying=True, temperature=27.0,
                                  humidity=44.0, note="drying")

    def test_an_unlisted_driver_chip_is_still_a_sensor(self):
        # sensor_type aht3x: an object under the right name is the sensor.
        unit = self._unit({"aht3x Ace_1": DryerSensor({"temperature": 27.0,
                                                       "humidity": 44.0})})
        snap = _AceBackend().snapshot(unit, {})
        assert snap == self._snap(temperature=27.0, humidity=44.0)

    def test_a_sensor_named_something_else_is_found_when_declared(self):
        # [temperature_sensor ace_temp] beside Ace_1 is found only when declared.
        unit = self._unit({"aht2x ace_temp": DryerSensor({"temperature": 27.0,
                                                          "humidity": 44.0})})
        unit.environment_sensor = "ace_temp"
        snap = _AceBackend().snapshot(unit, {"ace_temp": 51})
        assert snap == self._snap(temperature=51.0, humidity=44.0)

    def test_an_undeclared_sensor_under_another_name_is_not_guessed(self):
        unit = self._unit({"aht2x ace_temp": DryerSensor({"temperature": 27.0,
                                                          "humidity": 44.0})})
        snap = _AceBackend().snapshot(unit, {"ace_temp": 51})
        assert snap == self._snap(temperature=51.0)

    def test_only_the_missing_temperature_comes_from_the_sensor(self):
        unit = self._unit({"aht10 Ace_1": DryerSensor({"temperature": 31.0,
                                                       "humidity": 99.0})})
        st = {"ace_humidity": 40, "ace_connected": True, "ace_dryer": "stop"}
        snap = _AceBackend().snapshot(unit, st)
        assert snap == self._snap(online=True, temperature=31.0, humidity=40.0)


class TestAceBackendSlots:
    def test_each_status_word_decides_presence(self):
        st = {"ace_slots": [
            None,
            {"status": "Ready", "color": [10, 20, 30], "material": "PLA"},
            {"status": "EMPTY", "color": [40, 50, 60], "material": "PETG"},
            {"status": None, "material": None}]}
        assert _AceBackend().slots(DryerAceUnit(), st) == [
            DRYER_EMPTY_BAY,
            {"present": True, "color": "rgb(10,20,30)", "material": "PLA"},
            {"present": False, "color": "rgb(40,50,60)", "material": "PETG"},
            DRYER_EMPTY_BAY]

    def test_no_slot_list_gives_no_bays(self):
        assert _AceBackend().slots(DryerAceUnit(), {}) == []


class TestEnvReading:
    EMPTY = {"temperature": None, "humidity": None}

    def test_a_printer_that_cannot_enumerate_gives_nothing(self, monkeypatch):
        printer = dryer_sensor_printer({"aht10 ams_1": DryerSensor({"humidity": 40.0})})

        def broken(module: Optional[str] = None) -> List[Tuple[str, Any]]:
            raise RuntimeError("not ready")

        monkeypatch.setattr(printer, "lookup_objects", broken)
        unit = SimpleNamespace(printer=printer, name="ams_1")
        assert _env_reading(unit) == self.EMPTY

    def test_an_object_name_without_a_type_prefix_is_ignored(self):
        # "ams_1" alone has no "<type> <name>" split, so it never matches.
        printer = dryer_sensor_printer({"ams_1": DryerSensor({"temperature": 30.0,
                                                              "humidity": 40.0})})
        unit = SimpleNamespace(printer=printer, name="ams_1")
        assert _env_reading(unit) == self.EMPTY

    def test_an_empty_status_is_skipped_for_the_next_object(self):
        printer = dryer_sensor_printer({
            "aht10 ams_1": DryerSensor(None),
            "temperature_sensor ams_1": DryerSensor({"temperature": 29.0})})
        unit = SimpleNamespace(printer=printer, name="ams_1")
        assert _env_reading(unit) == {"temperature": 29.0, "humidity": None}

    def test_the_first_temperature_only_match_is_kept(self):
        printer = dryer_sensor_printer({
            "temperature_sensor ams_1": DryerSensor({"temperature": 29.0}),
            "thermistor ams_1": DryerSensor({"temperature": 35.0})})
        unit = SimpleNamespace(printer=printer, name="ams_1")
        assert _env_reading(unit) == {"temperature": 29.0, "humidity": None}

    def test_an_unread_temperature_does_not_block_a_later_one(self):
        # An unread 0.0 must not lock in an empty fallback ahead of a real reading.
        printer = dryer_sensor_printer({
            "thermistor ams_1": DryerSensor({"temperature": 0.0}),
            "temperature_sensor ams_1": DryerSensor({"temperature": 29.0})})
        unit = SimpleNamespace(printer=printer, name="ams_1")
        assert _env_reading(unit) == {"temperature": 29.0, "humidity": None}


class TestGenericBackendHasHeater:
    def test_it_never_claims_a_heater(self):
        # Even a unit carrying a vendor's heater flag is read-only here.
        unit = DryerOtherUnit()
        unit.has_heater = True
        assert _GenericBackend().has_heater(unit) is False


class TestGenericBackendSnapshot:
    """An OpenAMS keeps temperature and humidity on a separate Klipper object
    named after its controller; only the driver object (aht10 oams1) carries
    humidity, the temperature_sensor wrapper has temperature alone."""

    @staticmethod
    def _snap(objects: Dict[str, Any], name: str = "ams_1",
              oams_name: Optional[str] = "oams1") -> Dict[str, Any]:
        """
        Snapshot a unit whose printer holds the given sensor objects.

        :param objects: Klipper object name -> sensor
        :param name: AFC unit name
        :param oams_name: the unit's controller name
        :return dict: the generic backend's snapshot
        """
        unit = DryerOtherUnit(name=name, printer=dryer_sensor_printer(objects),
                              oams_name=oams_name)
        return _GenericBackend().snapshot(unit, {})

    @staticmethod
    def _view(temperature: Optional[float], humidity: Optional[float],
              note: str) -> Dict[str, Any]:
        """
        A generic snapshot with these readings and note.

        :param temperature: expected temperature
        :param humidity: expected humidity
        :param note: expected note
        :return dict: the expected snapshot
        """
        return {"online": True, "drying": False, "temperature": temperature,
                "humidity": humidity, "target": None, "note": note}

    def test_the_snapshot_says_there_is_no_dryer(self):
        snap = _GenericBackend().snapshot(DryerOtherUnit(), {})
        assert snap == self._view(None, None, "no dryer")

    def test_the_driver_object_supplies_both(self):
        snap = self._snap({"aht10 oams1": DryerSensor({"temperature": 29.79,
                                                       "humidity": 42.91})})
        assert snap == self._view(29.79, 42.91, "monitor only")

    def test_it_is_found_by_the_controller_name_not_the_unit_name(self):
        # The AFC unit is ams_1; nothing is registered under that name.
        snap = self._snap({"aht10 oams1": DryerSensor({"temperature": 30.0,
                                                       "humidity": 40.0})})
        assert snap == self._view(30.0, 40.0, "monitor only")

    def test_a_sensor_named_after_the_unit_also_works(self):
        snap = self._snap({"aht10 ams_1": DryerSensor({"temperature": 28.0,
                                                       "humidity": 44.0})},
                          oams_name=None)
        assert snap == self._view(28.0, 44.0, "monitor only")

    def test_the_case_of_the_section_name_does_not_have_to_match(self):
        # `oams = OAMS1` beside [temperature_sensor oams1] on a real machine.
        snap = self._snap({"aht3x oams1": DryerSensor({"temperature": 27.4,
                                                       "humidity": 38.0})},
                          oams_name="OAMS1")
        assert snap == self._view(27.4, 38.0, "monitor only")

    def test_the_driver_wins_over_the_temperature_wrapper(self):
        # The wrapper has no humidity, so preferring it would drop the field.
        snap = self._snap({
            "temperature_sensor oams1": DryerSensor({"temperature": 29.0}),
            "aht10 oams1": DryerSensor({"temperature": 29.79, "humidity": 42.91})})
        assert snap == self._view(29.79, 42.91, "monitor only")

    def test_the_wrapper_alone_still_gives_temperature(self):
        snap = self._snap({"temperature_sensor oams1": DryerSensor({"temperature": 29.0})})
        assert snap == self._view(29.0, None, "monitor only")

    def test_a_sensor_that_has_not_read_yet_is_not_shown_as_zero_degrees(self):
        # A present-but-unread driver reports 0.0/0.0.
        snap = self._snap({"aht10 oams1": DryerSensor({"temperature": 0.0,
                                                       "humidity": 0.0})})
        assert snap == self._view(None, None, "no dryer")

    def test_humidity_alone_is_still_reported(self):
        snap = self._snap({"aht10 oams1": DryerSensor({"temperature": 0.0,
                                                       "humidity": 41.0})})
        assert snap == self._view(None, 41.0, "monitor only")

    def test_the_note_says_monitor_only_when_there_are_readings(self):
        snap = self._snap({"aht10 oams1": DryerSensor({"temperature": 29.79,
                                                       "humidity": 42.91})})
        assert snap == self._view(29.79, 42.91, "monitor only")

    def test_no_sensor_at_all_is_unchanged(self):
        assert self._snap({}) == self._view(None, None, "no dryer")

    def test_a_unit_with_no_printer_does_not_raise(self):
        snap = _GenericBackend().snapshot(DryerOtherUnit(printer=None), {})
        assert snap == self._view(None, None, "no dryer")

    def test_a_raising_sensor_is_survived(self):
        snap = self._snap({"aht10 oams1": DryerSensor(error=RuntimeError("i2c down"))})
        assert snap == self._view(None, None, "no dryer")

    def test_temperature_alone_counts_as_a_reading(self):
        snap = self._snap({"aht10 Turtle_1": DryerSensor({"temperature": 29.0})},
                          name="Turtle_1", oams_name=None)
        assert snap == self._view(29.0, None, "monitor only")

    def test_humidity_alone_counts_as_a_reading(self):
        snap = self._snap({"aht10 Turtle_1": DryerSensor({"temperature": 0.0,
                                                          "humidity": 41.0})},
                          name="Turtle_1", oams_name=None)
        assert snap == self._view(None, 41.0, "monitor only")


class TestGenericBackendSlots:
    def test_slots_report_lane_load_state(self):
        unit = DryerOtherUnit(lanes={
            "l1": DryerLane(color="#ff0000", material="PLA", load_state=True),
            "l2": DryerLane(color=None, load_state=False)})
        assert _GenericBackend().slots(unit, {}) == [
            {"present": True, "color": "#ff0000", "material": "PLA"},
            DRYER_EMPTY_BAY]


class TestGenericBackendStartScript:
    def test_starting_one_is_refused(self):
        # Unreachable through the panel, but an error beats a broken g-code line.
        with pytest.raises(ValueError) as excinfo:
            _GenericBackend().start_script("Turtle_1", 55, 60, 0)
        assert str(excinfo.value) == "Turtle_1 has no dryer"


class TestGenericBackendStopScript:
    def test_stopping_one_is_refused(self):
        with pytest.raises(ValueError) as excinfo:
            _GenericBackend().stop_script("Turtle_1")
        assert str(excinfo.value) == "Turtle_1 has no dryer"


class TestAFCDryerInit:
    def test_defaults_and_registrations(self, caplog):
        caplog.set_level(logging.DEBUG, logger="afc_dryer")
        printer = MockPrinter()
        panel = AFCDryer(MockConfig(printer=printer, values={}))
        assert (panel.port, panel.bind, panel.poll, panel.show_heaterless) == (
            8093, "0.0.0.0", 2.0, False)
        assert panel.printer is printer
        assert panel.reactor is printer._reactor
        assert panel.logger is logging.getLogger("afc_dryer")
        assert (panel._server, panel._thread, panel._timer) == (None, None, None)
        assert isinstance(panel._lock, type(threading.Lock()))
        assert panel._units == []
        assert panel._state == {"units": [], "ready": False}
        assert printer._gcode._commands == {"AFC_DRYER_STATUS": panel.cmd_STATUS}
        assert printer._event_handlers == {
            "klippy:ready": [panel._handle_ready],
            "klippy:disconnect": [panel._handle_disconnect]}
        assert [r for r in caplog.records if r.name == "afc_dryer"] == []

    def test_config_values_override_the_defaults(self, caplog):
        caplog.set_level(logging.DEBUG, logger="afc_dryer")
        panel, _ = make_dryer_panel(values={"port": 18093, "bind": "127.0.0.1",
                                            "poll": 0.5, "show_heaterless": True})
        assert (panel.port, panel.bind, panel.poll, panel.show_heaterless) == (
            18093, "127.0.0.1", 0.5, True)
        assert [r for r in caplog.records if r.name == "afc_dryer"] == []


class TestAFCDryerHandleReady:
    NO_UNITS_WARNING = (
        "afc_dryer: no units with a drying heater found; the panel will be empty "
        "(set show_heaterless: True to list every other AFC unit read-only)")
    WEBCAM_HINT = ("  Mainsail/Fluidd: add a webcam, service 'iframe', URL "
                   "http://<printer-host>:8093/")

    def test_it_adopts_the_afc_logger_and_starts_everything(self, dryer_fake_net):
        unit = DryerBambuUnit("AMS_2")
        panel, printer = make_dryer_panel(unit)
        panel._handle_ready()
        assert panel.logger is printer._afc.logger
        assert [u["name"] for u in panel._units] == ["AMS_2"]
        printer._reactor.register_timer.assert_called_once_with(
            panel._snapshot, printer._reactor.NOW)
        assert panel._timer == "timer"
        assert panel._server is dryer_fake_net.servers[0]
        assert panel._thread is dryer_fake_net.threads[0]
        assert dryer_fake_net.threads[0].started is True
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): AMS_2 (AMS 2 Pro)"),
            ("info", "afc_dryer: serving on http://0.0.0.0:8093/"),
            ("info", self.WEBCAM_HINT)]

    def test_an_empty_roster_warns(self, dryer_fake_net):
        panel, printer = make_dryer_panel()
        panel._handle_ready()
        assert panel.logger is printer._afc.logger
        assert panel._units == []
        assert panel._timer == "timer"
        assert panel._server is dryer_fake_net.servers[0]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 0 dryer(s): none"),
            ("warning", self.NO_UNITS_WARNING),
            ("info", "afc_dryer: serving on http://0.0.0.0:8093/"),
            ("info", self.WEBCAM_HINT)]

    def test_without_afc_the_stdlib_logger_is_kept(self, dryer_fake_net, caplog):
        panel, printer = make_dryer_panel()
        printer._afc = None
        caplog.set_level(logging.DEBUG, logger="afc_dryer")
        panel._handle_ready()
        assert panel.logger is logging.getLogger("afc_dryer")
        assert panel._units == []
        assert panel._timer == "timer"
        assert panel._server is dryer_fake_net.servers[0]
        records = [(r.levelname, r.getMessage()) for r in caplog.records
                   if r.name == "afc_dryer"]
        assert records == [
            ("INFO", "afc_dryer: 0 dryer(s): none"),
            ("WARNING", self.NO_UNITS_WARNING),
            ("INFO", "afc_dryer: serving on http://0.0.0.0:8093/"),
            ("INFO", self.WEBCAM_HINT)]


class TestAFCDryerHandleDisconnect:
    def test_a_running_server_is_shut_down_and_closed(self, dryer_fake_net):
        panel, _ = discovered_dryer_panel()
        panel._start_server()
        panel.logger = MockLogger()
        server = dryer_fake_net.servers[0]
        panel._handle_disconnect()
        assert server.calls == ["shutdown", "server_close"]
        assert panel._server is None
        assert panel.logger.messages == []

    def test_a_failing_shutdown_still_forgets_the_server(self, dryer_fake_net):
        panel, _ = discovered_dryer_panel()
        panel._start_server()
        panel.logger = MockLogger()
        server = dryer_fake_net.servers[0]
        server.shutdown_error = OSError("already closed")
        panel._handle_disconnect()
        assert server.calls == ["shutdown"]
        assert panel._server is None
        assert panel.logger.messages == []

    def test_no_server_is_a_no_op(self):
        panel, _ = discovered_dryer_panel()
        panel._handle_disconnect()
        assert panel._server is None
        assert panel.logger.messages == []


class TestAFCDryerDiscover:
    BAMBU_BACKEND = _BACKENDS[0]
    GENERIC_BACKEND = _BACKENDS[2]

    @classmethod
    def _bambu(cls, unit: DryerBambuUnit, **over: Any) -> Dict[str, Any]:
        """
        The descriptor of a heated AMS 2 Pro unit, with the given fields replaced.

        :param unit: the unit object
        :param over: fields that differ
        :return dict: the expected descriptor
        """
        desc = {"model": "ams2", "label": "AMS 2 Pro", "max_temp": 65, "slots": 4,
                "name": unit.name, "kind": "bambu", "has_heater": True,
                "rotate": True, "_obj": unit, "_backend": cls.BAMBU_BACKEND}
        desc.update(over)
        return desc

    @staticmethod
    def _ace(unit: DryerAceUnit, **over: Any) -> Dict[str, Any]:
        """
        The descriptor of a V1 ACE at the 55 C default, with fields replaced.

        :param unit: the unit object
        :param over: fields that differ
        :return dict: the expected descriptor
        """
        desc = {"model": "ace", "label": "ACE", "max_temp": 55, "slots": 4,
                "name": unit.name, "kind": "ace", "has_heater": True,
                "rotate": True, "_obj": unit, "_backend": DRYER_ACE_BACKEND}
        desc.update(over)
        return desc

    @classmethod
    def _generic(cls, unit: DryerOtherUnit, **over: Any) -> Dict[str, Any]:
        """
        The descriptor of a read-only BoxTurtle, with the given fields replaced.

        :param unit: the unit object
        :param over: fields that differ
        :return dict: the expected descriptor
        """
        desc = {"model": "generic", "label": "BoxTurtle", "max_temp": 0, "slots": 0,
                "name": unit.name, "kind": "generic", "has_heater": False,
                "rotate": False, "_obj": unit, "_backend": cls.GENERIC_BACKEND}
        desc.update(over)
        return desc

    @staticmethod
    def _run(*units: Any, values: Optional[Dict[str, Any]] = None,
             afc_units: Optional[Dict[str, Any]] = None) -> AFCDryer:
        """
        Build a panel and run _discover with a fresh MockLogger.

        :param units: unit fakes to register as printer objects
        :param values: [afc_dryer] config values
        :param afc_units: entries for AFC's own unit registry
        :return AFCDryer: the panel after discovery
        """
        panel, _ = make_dryer_panel(*units, values=values, afc_units=afc_units)
        panel.logger = MockLogger()
        panel._discover()
        return panel

    def test_finds_both_vendors(self):
        bambu, ace = DryerBambuUnit("AMS_2"), DryerAceUnit("Ace2_1")
        panel = self._run(bambu, ace)
        assert panel._units == [self._ace(ace), self._bambu(bambu)]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 2 dryer(s): Ace2_1 (ACE), AMS_2 (AMS 2 Pro)")]

    def test_heaterless_unit_is_hidden_by_default(self):
        ams1 = DryerBambuUnit("AMS_1", model="ams1", heater=False)
        ams2 = DryerBambuUnit("AMS_2")
        panel = self._run(ams1, ams2)
        assert panel._units == [self._bambu(ams2)]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): AMS_2 (AMS 2 Pro)")]

    def test_show_heaterless_lists_it_read_only(self):
        ams1 = DryerBambuUnit("AMS_1", model="ams1", heater=False)
        panel = self._run(ams1, values={"show_heaterless": True})
        assert panel._units == [
            self._bambu(ams1, model="ams1", label="AMS", has_heater=False)]
        assert panel.logger.messages == [("info", "afc_dryer: 1 dryer(s): AMS_1 (AMS)")]

    def test_model_drives_label_slots_and_ceiling(self):
        ht = DryerBambuUnit("HT", model="ht", max_temp=85, slots=1)
        panel = self._run(ht)
        assert panel._units == [
            self._bambu(ht, model="ht", label="AMS HT", max_temp=85, slots=1)]
        assert panel.logger.messages == [("info", "afc_dryer: 1 dryer(s): HT (AMS HT)")]

    def test_ace_descriptor_from_its_own_config(self):
        ace = DryerAceUnit("Ace2_1", max_temp=60.5)
        panel = self._run(ace)
        assert panel._units == [self._ace(ace, max_temp=60)]
        assert panel.logger.messages == [("info", "afc_dryer: 1 dryer(s): Ace2_1 (ACE)")]

    def test_bambu_and_ace_advertise_rotation(self):
        panel = self._run(DryerBambuUnit("AMS_2"), DryerAceUnit("Ace2_1"))
        assert {u["name"]: u["rotate"] for u in panel._units} == {
            "AMS_2": True, "Ace2_1": True}
        assert panel.logger.messages == [
            ("info", "afc_dryer: 2 dryer(s): Ace2_1 (ACE), AMS_2 (AMS 2 Pro)")]

    def test_ace_with_zero_ceiling_opts_out(self):
        panel = self._run(DryerAceUnit("Ace2_1", max_temp=0.0))
        assert panel._units == []
        assert panel.logger.messages == [("info", "afc_dryer: 0 dryer(s): none")]

    def test_a_broken_unit_does_not_sink_discovery(self):
        class Exploding(DryerBambuUnit):
            @property
            def ams_model(self) -> str:
                raise RuntimeError("boom")

            @ams_model.setter
            def ams_model(self, value: Any) -> None:
                pass

        good = DryerBambuUnit("good")
        panel = self._run(Exploding("bad"), good)
        assert panel._units == [self._bambu(good)]
        assert panel.logger.messages == [
            ("warning", "afc_dryer: skipping a bambu unit: boom"),
            ("info", "afc_dryer: 1 dryer(s): good (AMS 2 Pro)")]

    def test_a_heaterless_pool_slot_is_kept_for_a_future_claim(self):
        # A pool slot's heater is provisional, so it stays for a live claim.
        slot = DryerBambuUnit("AMS_1", model="ams1", heater=False)
        slot.pool = True
        panel = self._run(slot)
        assert panel._units == [
            self._bambu(slot, model="ams1", label="AMS", has_heater=False)]
        assert panel.logger.messages == [("info", "afc_dryer: 1 dryer(s): AMS_1 (AMS)")]

    def test_a_heaterless_NON_pool_unit_is_still_hidden(self):
        slot = DryerBambuUnit("AMS_1", model="ams1", heater=False)
        slot.pool = False
        panel = self._run(slot)
        assert panel._units == []
        assert panel.logger.messages == [("info", "afc_dryer: 0 dryer(s): none")]

    def test_ace2_prefix_is_discovered(self):
        ace2 = DryerAce2Unit("Ace2_1")
        panel = self._run(ace2)
        assert panel._units == [self._ace(ace2, label="ACE 2")]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Ace2_1 (ACE 2)")]

    def test_a_unit_matching_two_prefixes_is_listed_once(self):
        # The same object answers both ACE lookups.
        unit = DryerAce2Unit("Ace2_1")
        panel, printer = make_dryer_panel()
        printer._objects.update({"AFC_ACE Ace2_1": unit, "AFC_ACE2 Ace2_1": unit})
        panel.logger = MockLogger()
        panel._discover()
        assert panel._units == [self._ace(unit, label="ACE 2")]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Ace2_1 (ACE 2)")]

    def test_v1_says_ace(self):
        ace = DryerAceUnit("Ace_1")
        panel = self._run(ace)
        assert panel._units == [self._ace(ace)]
        assert panel.logger.messages == [("info", "afc_dryer: 1 dryer(s): Ace_1 (ACE)")]

    def test_v2_says_ace_2(self):
        # The ACE 2 class subclasses the V1 one, so isinstance would say "ACE".
        ace2 = DryerAce2Unit("Ace2_1")
        assert isinstance(ace2, DryerAceUnit)
        panel = self._run(ace2)
        assert panel._units == [self._ace(ace2, label="ACE 2")]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Ace2_1 (ACE 2)")]

    def test_a_boxturtle_is_listed_when_heaterless_is_on(self):
        unit = DryerOtherUnit()
        panel = self._run(values={"show_heaterless": True}, afc_units={"Turtle_1": unit})
        assert panel._units == [self._generic(unit)]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Turtle_1 (BoxTurtle)")]

    def test_it_is_hidden_by_default(self):
        panel = self._run(afc_units={"Turtle_1": DryerOtherUnit()})
        assert panel._units == []
        assert panel.logger.messages == [("info", "afc_dryer: 0 dryer(s): none")]

    def test_it_is_labelled_by_its_configured_type(self):
        unit = DryerOtherUnit(type_="HTLF")
        panel = self._run(values={"show_heaterless": True}, afc_units={"Turtle_1": unit})
        assert panel._units == [self._generic(unit, label="HTLF")]
        assert panel.logger.messages == [("info", "afc_dryer: 1 dryer(s): Turtle_1 (HTLF)")]

    def test_a_unit_with_no_type_falls_back_to_its_class_name(self):
        unit = DryerOtherUnit(type_=None)
        panel = self._run(values={"show_heaterless": True}, afc_units={"Turtle_1": unit})
        assert panel._units == [self._generic(unit, label="DryerOtherUnit")]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Turtle_1 (DryerOtherUnit)")]

    def test_an_underscored_type_reads_as_its_camelcase_prefix(self):
        # Box_Turtle reads as BoxTurtle, matching the AFC_BoxTurtle section.
        unit = DryerOtherUnit(type_="Box_Turtle")
        panel = self._run(values={"show_heaterless": True}, afc_units={"Turtle_1": unit})
        assert panel._units == [self._generic(unit, label="BoxTurtle")]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Turtle_1 (BoxTurtle)")]

    def test_slot_count_comes_from_the_lanes(self):
        unit = DryerOtherUnit(lanes={"l1": DryerLane(), "l2": DryerLane(),
                                     "l3": DryerLane()})
        panel = self._run(values={"show_heaterless": True}, afc_units={"Turtle_1": unit})
        assert panel._units == [self._generic(unit, slots=3)]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Turtle_1 (BoxTurtle)")]

    def test_no_afc_section_is_not_an_error(self):
        panel, printer = make_dryer_panel(values={"show_heaterless": True})
        printer._afc = None
        panel.logger = MockLogger()
        panel._discover()
        assert panel._units == []
        assert panel.logger.messages == [("info", "afc_dryer: 0 dryer(s): none")]

    def test_a_bambu_unit_keeps_its_own_backend(self):
        # afc.units holds the Bambu unit too; the seen set keeps its vendor view.
        bambu = DryerBambuUnit("BambuAMS_1")
        panel = self._run(bambu, values={"show_heaterless": True},
                          afc_units={bambu.name: bambu})
        assert panel._units == [self._bambu(bambu)]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): BambuAMS_1 (AMS 2 Pro)")]

    def test_the_unit_is_listed_once(self):
        bambu = DryerBambuUnit("BambuAMS_1")
        turtle = DryerOtherUnit()
        panel = self._run(bambu, values={"show_heaterless": True},
                          afc_units={bambu.name: bambu, "Turtle_1": turtle})
        assert panel._units == [self._bambu(bambu), self._generic(turtle)]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 2 dryer(s): BambuAMS_1 (AMS 2 Pro), "
                     "Turtle_1 (BoxTurtle)")]

    class AfcToolchanger:
        """A tool-changer registered in afc.units; its class name is what matches."""

        def __init__(self) -> None:
            self.name = "Tools"
            self.type = "Toolchanger"
            self.lanes: Dict[str, Any] = {}

    class _Sub(AfcToolchanger):
        """A fork subclassing it must be excluded too."""

    def test_it_is_not_listed(self):
        panel = self._run(values={"show_heaterless": True},
                          afc_units={"Tools": self.AfcToolchanger()})
        assert panel._units == []
        assert panel.logger.messages == [("info", "afc_dryer: 0 dryer(s): none")]

    def test_a_subclass_is_also_excluded(self):
        panel = self._run(values={"show_heaterless": True}, afc_units={"Tools": self._Sub()})
        assert panel._units == []
        assert panel.logger.messages == [("info", "afc_dryer: 0 dryer(s): none")]

    def test_real_units_alongside_it_are_still_listed(self):
        turtle = DryerOtherUnit()
        panel = self._run(values={"show_heaterless": True},
                          afc_units={"Tools": self.AfcToolchanger(), "Turtle_1": turtle})
        assert panel._units == [self._generic(turtle)]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Turtle_1 (BoxTurtle)")]

    def test_the_match_ignores_the_operator_settable_type_string(self):
        # `type` is config-settable, so renaming it must not bring one back.
        tc = self.AfcToolchanger()
        tc.type = "Box_Turtle"
        panel = self._run(values={"show_heaterless": True}, afc_units={"Tools": tc})
        assert panel._units == []
        assert panel.logger.messages == [("info", "afc_dryer: 0 dryer(s): none")]
        # Nor may a real unit typed "Toolchanger" be dropped for it.
        unit = DryerOtherUnit(type_="Toolchanger")
        panel = self._run(values={"show_heaterless": True}, afc_units={"Turtle_1": unit})
        assert panel._units == [self._generic(unit, label="Toolchanger")]
        assert panel.logger.messages == [
            ("info", "afc_dryer: 1 dryer(s): Turtle_1 (Toolchanger)")]

    def test_a_failing_backend_is_logged_and_the_rest_are_sorted(self):
        class BrokenRegistry:
            @property
            def units(self) -> Dict[str, Any]:
                raise RuntimeError("registry gone")

        bambu, ace = DryerBambuUnit("AMS_2"), DryerAceUnit("Ace_1")
        panel, printer = make_dryer_panel(bambu, ace)
        printer._afc = BrokenRegistry()
        panel.logger = MockLogger()
        panel._discover()
        assert panel._units == [self._ace(ace), self._bambu(bambu)]
        assert panel.logger.messages == [
            ("warning", "afc_dryer: could not enumerate generic units: registry gone"),
            ("info", "afc_dryer: 2 dryer(s): Ace_1 (ACE), AMS_2 (AMS 2 Pro)")]

    def test_a_unit_that_cannot_be_described_is_skipped_with_a_warning(self):
        good = DryerBambuUnit("AMS_2")
        panel = self._run(DryerBambuUnit("AMS_1", max_temp="hot"), good)
        assert panel._units == [self._bambu(good)]
        assert panel.logger.messages == [
            ("warning", "afc_dryer: skipping a bambu unit: invalid literal for "
                        "int() with base 10: 'hot'"),
            ("info", "afc_dryer: 1 dryer(s): AMS_2 (AMS 2 Pro)")]


class TestAFCDryerStartServer:
    def test_it_binds_and_starts_a_named_daemon_thread(self, dryer_fake_net):
        panel, _ = discovered_dryer_panel(values={"bind": "127.0.0.1", "port": 18093})
        panel._start_server()
        server = dryer_fake_net.servers[0]
        thread = dryer_fake_net.threads[0]
        assert server.address == ("127.0.0.1", 18093)
        assert issubclass(server.handler, BaseHTTPRequestHandler)
        assert server.handler.server_version == "AFCDryer/1.0"
        assert server.daemon_threads is True
        assert panel._server is server
        assert panel._thread is thread
        assert (thread.name, thread.daemon, thread.started) == (
            "afc_dryer_http", True, True)
        assert server.calls == []
        assert panel.logger.messages == [
            ("info", "afc_dryer: serving on http://127.0.0.1:18093/"),
            ("info", "  Mainsail/Fluidd: add a webcam, service 'iframe', URL "
                     "http://<printer-host>:18093/")]

    def test_a_bind_failure_is_logged(self, dryer_fake_net):
        dryer_fake_net.server_error = OSError(98, "Address already in use")
        panel, _ = discovered_dryer_panel(values={"bind": "127.0.0.1", "port": 18093})
        panel._start_server()
        assert panel._server is None
        assert panel._thread is None
        assert dryer_fake_net.threads == []
        assert panel.logger.messages == [
            ("error", "afc_dryer: failed to start HTTP server on 127.0.0.1:18093: "
                      "[Errno 98] Address already in use")]

    def test_a_thread_failure_closes_the_built_server(self, dryer_fake_net):
        dryer_fake_net.start_error = RuntimeError("can't start new thread")
        panel, _ = discovered_dryer_panel()
        panel._start_server()
        assert len(dryer_fake_net.servers) == 1
        assert panel._server is None
        assert panel._thread is dryer_fake_net.threads[0]
        assert dryer_fake_net.threads[0].started is False
        # The port was already bound, so it is released before the server is dropped.
        assert dryer_fake_net.servers[0].calls == ["server_close"]
        assert panel.logger.messages == [
            ("error", "afc_dryer: failed to start HTTP server on 0.0.0.0:8093: "
                      "can't start new thread")]

    def test_the_thread_names_itself_then_serves(self, dryer_fake_net):
        panel, _ = discovered_dryer_panel()
        panel._start_server()
        panel.logger = MockLogger()
        dryer_fake_net.threads[0].target()
        assert dryer_fake_net.names == [b"worker-7"]
        assert dryer_fake_net.servers[0].calls == ["serve_forever"]
        assert panel.logger.messages == []

    def test_a_naming_failure_still_serves(self, dryer_fake_net):
        panel, _ = discovered_dryer_panel()
        panel._start_server()
        panel.logger = MockLogger()
        dryer_fake_net.ffi_error = OSError("no libc")
        dryer_fake_net.threads[0].target()
        assert dryer_fake_net.names == []
        assert dryer_fake_net.servers[0].calls == ["serve_forever"]
        assert panel.logger.messages == []


class TestAFCDryerSnapshot:
    LANE_SLOTS = [{"present": True}, {"present": True}, {"present": False},
                  {"present": False}]

    @staticmethod
    def _ace_row(name: str = "Ace_1", **over: Any) -> Dict[str, Any]:
        """
        The published row of an idle V1 ACE, with the given fields replaced.

        :param name: unit name
        :param over: fields that differ from the idle defaults
        :return dict: the expected row
        """
        row = {"name": name, "kind": "ace", "model": "ace", "label": "ACE",
               "max_temp": 55, "slots": 4, "has_heater": True, "rotate": True,
               "online": False, "drying": False, "temperature": None, "humidity": None,
               "target": None, "remaining": None, "duration": None, "note": "",
               "error": ""}
        row.update(over)
        row.setdefault("bays", [dict(DRYER_EMPTY_BAY) for _ in range(row["slots"])])
        return row

    @staticmethod
    def _snap(panel: AFCDryer, eventtime: float = 0.0) -> List[Dict[str, Any]]:
        """
        Run one snapshot and return the published rows.

        Also checks the next firing time and that the state is marked ready.

        :param panel: a discovered panel
        :param eventtime: reactor time of the firing
        :return list: the rows in panel._state
        """
        assert panel._snapshot(eventtime) == eventtime + 2.0
        assert panel._state["ready"] is True
        assert sorted(panel._state) == ["ready", "units"]
        return panel._state["units"]

    @staticmethod
    def _lane_unit(lanes: Dict[str, DryerLane], slot_map: Optional[Dict[str, int]] = None,
                   slots: Optional[List[Dict[str, Any]]] = None) -> DryerBambuUnit:
        """
        An AMS 2 Pro carrying AFC lanes, optionally with a lane -> bay map.

        :param lanes: lane name -> lane
        :param slot_map: the unit's _slot_map, or None for none
        :param slots: vendor slot records
        :return DryerBambuUnit: the unit
        """
        unit = DryerBambuUnit("AMS_2", status={"slots": (
            slots if slots is not None else TestAFCDryerSnapshot.LANE_SLOTS)})
        unit.lanes = lanes
        if slot_map is not None:
            unit._slot_map = slot_map
        return unit

    @staticmethod
    def _lane_bay(lane: str, color: str, present: bool = True) -> Dict[str, Any]:
        """
        A published bay coloured by a lane that carries no tooltip detail.

        :param lane: lane name
        :param color: the CSS colour
        :param present: vendor presence
        :return dict: the expected bay
        """
        return {"present": present, "color": color, "material": "", "lane": lane}

    def test_an_unclaimed_pool_slot_is_hidden_from_the_page(self):
        slot = DryerBambuUnit("AMS_1", model="ams1", heater=False)
        slot.pool = True
        panel, _ = discovered_dryer_panel(slot)
        assert len(panel._units) == 1
        assert self._snap(panel) == []
        assert panel.logger.messages == []

    def test_a_live_claim_refreshes_model_label_heater_and_ceiling(self):
        # A fabricated ams1 (no heater, no ceiling) found as a pool slot.
        slot = DryerBambuUnit("AMS_1", model="ams1", heater=False, max_temp=0)
        slot.pool = True
        panel, _ = discovered_dryer_panel(slot)
        # It claims a physical AMS 2 Pro: pool clears, identity flips live.
        slot.pool = False
        slot.ams_model, slot.has_heater, slot.dry_max_temp = "ams2", True, 65
        assert self._snap(panel) == [dryer_bambu_row("AMS_1")]
        assert (panel._units[0]["model"], panel._units[0]["has_heater"]) == ("ams1", False)
        assert panel.logger.messages == []

    def test_bambu_fields_are_mapped(self):
        unit = DryerBambuUnit("AMS_2", status={
            "bridge_online": True, "drying": True, "temperature": 48.0,
            "humidity": 21.0, "dry_target": 55.0,
            "slots": [{"present": True, "color": "FF8800"}]})
        panel, _ = discovered_dryer_panel(unit)
        bays = [{"present": True, "color": "#FF8800", "material": ""},
                DRYER_EMPTY_BAY, DRYER_EMPTY_BAY, DRYER_EMPTY_BAY]
        assert self._snap(panel) == [dryer_bambu_row(
            online=True, drying=True, temperature=48.0, humidity=21.0, target=55.0,
            bays=bays)]
        assert panel.logger.messages == []

    def test_a_refusal_reason_reaches_the_published_row(self):
        panel, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2", status={
            "bridge_online": True, "drying": True, "dry_error": "filament hub load!"}))
        assert self._snap(panel) == [dryer_bambu_row(
            online=True, drying=True, error="filament hub load!")]
        assert panel.logger.messages == []

    def test_ace_dryer_string_decides_drying(self):
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace2_1", status={
            "ace_connected": True, "ace_dryer": "drying", "ace_temp": 51.2}))
        # The vendor's own wording is carried through as the note.
        assert self._snap(panel) == [self._ace_row(
            "Ace2_1", online=True, drying=True, temperature=51.2, note="drying")]
        assert panel.logger.messages == []

    @pytest.mark.parametrize("value", ["", "stop", "stopped", "idle", "off",
                                       "none", "OFF", " Idle "])
    def test_ace_idle_strings_are_not_drying(self, value):
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace2_1", status={
            "ace_connected": True, "ace_dryer": value, "ace_dryer_target": 55,
            "ace_dryer_remain": 60, "ace_dryer_duration": 8}))
        assert self._snap(panel) == [self._ace_row("Ace2_1", online=True)]
        assert panel.logger.messages == []

    def test_the_ace_set_point_reaches_the_card(self):
        # A V1's dryer_status carries the set-point beside the state word.
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace_1", status={
            "ace_connected": True, "ace_dryer": "drying",
            "ace_dryer_target": 55, "ace_temp": 51}))
        assert self._snap(panel) == [self._ace_row(
            online=True, drying=True, temperature=51.0, target=55, note="drying")]
        assert panel.logger.messages == []

    def test_a_v1_reports_its_cycle_length_in_minutes(self):
        # Measured: duration 480 beside 27679 s left, so 480 is minutes.
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace_1", status={
            "ace_connected": True, "ace_dryer": "drying",
            "ace_dryer_remain": 27679, "ace_dryer_duration": 480}))
        assert self._snap(panel) == [self._ace_row(
            online=True, drying=True, remaining=27679, duration=28800, note="drying")]
        assert panel.logger.messages == []

    def test_an_ace_2_reports_the_same_field_in_seconds(self):
        # Measured on an ACE 2 asked for 120 minutes: duration 7200.
        panel, _ = discovered_dryer_panel(DryerAce2Unit("Ace2_1", status={
            "ace_connected": True, "ace_dryer": "keeping",
            "ace_dryer_remain": 7200, "ace_dryer_duration": 7200}))
        assert self._snap(panel) == [self._ace_row(
            "Ace2_1", label="ACE 2", online=True, drying=True, remaining=7200,
            duration=7200, note="keeping")]
        assert panel.logger.messages == []

    def test_a_v1_near_the_end_is_not_mistaken_for_seconds(self):
        # 480 minutes against 100 s left would read as seconds to a size test.
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace_1", status={
            "ace_connected": True, "ace_dryer": "drying",
            "ace_dryer_remain": 100, "ace_dryer_duration": 480}))
        assert self._snap(panel) == [self._ace_row(
            online=True, drying=True, remaining=100, duration=28800, note="drying")]
        assert panel.logger.messages == []

    def test_a_bambu_clock_reaches_the_card_in_the_same_units(self):
        panel, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2", status={
            "bridge_online": True, "drying": True,
            "dry_remaining": 1800, "dry_minutes": 480}))
        assert self._snap(panel) == [dryer_bambu_row(
            online=True, drying=True, remaining=1800, duration=28800)]
        assert panel.logger.messages == []

    def test_a_unit_with_no_clock_reports_none_not_zero(self):
        # A zero length would draw a progress bar that is instantly full.
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace_1", status={
            "ace_connected": True, "ace_dryer": "drying",
            "ace_dryer_duration": 0}))
        assert self._snap(panel) == [self._ace_row(
            online=True, drying=True, note="drying")]
        assert panel.logger.messages == []

    def test_an_idle_ace_reports_no_clock_despite_its_zeros(self):
        # Measured: an idle ACE fills dryer_status with zeros.
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace_1", status={
            "ace_connected": True, "ace_dryer": "stop",
            "ace_dryer_target": 0, "ace_dryer_remain": 0,
            "ace_dryer_duration": 0}))
        assert self._snap(panel) == [self._ace_row(online=True)]
        assert panel.logger.messages == []

    def test_an_idle_ace_has_no_target(self):
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace_1", status={
            "ace_connected": True, "ace_dryer": "stop", "ace_dryer_target": 55}))
        assert self._snap(panel) == [self._ace_row(online=True)]
        assert panel.logger.messages == []

    def test_unknown_ace_state_reads_as_drying_and_says_so(self):
        # An unfamiliar state is shown verbatim rather than rounded to idle.
        panel, _ = discovered_dryer_panel(DryerAceUnit("Ace2_1", status={
            "ace_connected": True, "ace_dryer": "preheat"}))
        assert self._snap(panel) == [self._ace_row(
            "Ace2_1", online=True, drying=True, note="preheat")]
        assert panel.logger.messages == []

    def test_bays_are_padded_to_the_unit_slot_count(self):
        panel, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2", slots=4, status={
            "slots": [{"present": True, "color": "00FF00"}]}))
        bays = [{"present": True, "color": "#00FF00", "material": ""},
                DRYER_EMPTY_BAY, DRYER_EMPTY_BAY, DRYER_EMPTY_BAY]
        assert self._snap(panel) == [dryer_bambu_row(bays=bays)]
        assert panel.logger.messages == []

    def test_bays_are_truncated_to_the_unit_slot_count(self):
        panel, _ = discovered_dryer_panel(DryerBambuUnit("HT", model="ht", slots=1, status={
            "slots": [{"present": True}, {"present": True}]}))
        assert self._snap(panel) == [dryer_bambu_row(
            "HT", model="ht", label="AMS HT", slots=1,
            bays=[{"present": True, "color": "", "material": ""}])]
        assert panel.logger.messages == []

    def test_status_failure_degrades_to_offline(self):
        class Broken(DryerBambuUnit):
            def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
                raise RuntimeError("bus down")

        panel, _ = discovered_dryer_panel(Broken("AMS_2", status={"bridge_online": True}))
        assert self._snap(panel) == [dryer_bambu_row()]
        assert panel.logger.messages == [
            ("debug", "afc_dryer: status failed for AMS_2: bus down")]

    def test_snapshot_publishes_only_primitives(self):
        # The HTTP thread reads these rows; no printer object may leak in.
        panel, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2"), DryerAceUnit("Ace2_1"))
        rows = self._snap(panel)
        assert rows == [self._ace_row("Ace2_1"), dryer_bambu_row()]
        primitives = (str, int, float, bool, list, type(None))
        assert [k for row in rows for k, v in row.items()
                if not isinstance(v, primitives)] == []
        assert panel.logger.messages == []

    def test_lane_colour_reaches_the_bay(self):
        unit = self._lane_unit(
            {"lane1": DryerLane(1, "#FF0000"), "lane2": DryerLane(2, "00FF00")},
            slot_map={"lane1": 0, "lane2": 1})
        panel, _ = discovered_dryer_panel(unit)
        assert self._snap(panel) == [dryer_bambu_row(bays=[
            self._lane_bay("lane1", "#FF0000"), self._lane_bay("lane2", "#00FF00"),
            DRYER_EMPTY_BAY, DRYER_EMPTY_BAY])]
        assert panel.logger.messages == []

    def test_slot_map_decides_which_bay(self):
        # Lane order and index are not assumed; the map is authoritative.
        unit = self._lane_unit(
            {"lane1": DryerLane(1, "#FF0000"), "lane2": DryerLane(2, "#00FF00")},
            slot_map={"lane1": 1, "lane2": 0})
        panel, _ = discovered_dryer_panel(unit)
        assert self._snap(panel) == [dryer_bambu_row(bays=[
            self._lane_bay("lane2", "#00FF00"), self._lane_bay("lane1", "#FF0000"),
            DRYER_EMPTY_BAY, DRYER_EMPTY_BAY])]
        assert panel.logger.messages == []

    def test_falls_back_to_the_lane_index_without_a_map(self):
        unit = self._lane_unit(
            {"lane2": DryerLane(2, "#00FF00"), "lane1": DryerLane(1, "#FF0000")})
        panel, _ = discovered_dryer_panel(unit)
        assert self._snap(panel) == [dryer_bambu_row(bays=[
            self._lane_bay("lane1", "#FF0000"), self._lane_bay("lane2", "#00FF00"),
            DRYER_EMPTY_BAY, DRYER_EMPTY_BAY])]
        assert panel.logger.messages == []

    def test_vendor_colour_is_the_fallback_not_the_winner(self):
        unit = self._lane_unit({"lane1": DryerLane(1, "")}, slot_map={"lane1": 0},
                               slots=[{"present": True, "color": "0000FF"}])
        panel, _ = discovered_dryer_panel(unit)
        assert self._snap(panel) == [dryer_bambu_row(bays=[
            self._lane_bay("lane1", "#0000FF"),
            DRYER_EMPTY_BAY, DRYER_EMPTY_BAY, DRYER_EMPTY_BAY])]
        assert panel.logger.messages == []

    def test_lane_colour_beats_the_vendors(self):
        unit = self._lane_unit({"lane1": DryerLane(1, "#FF0000")}, slot_map={"lane1": 0},
                               slots=[{"present": True, "color": "0000FF"}])
        panel, _ = discovered_dryer_panel(unit)
        assert self._snap(panel) == [dryer_bambu_row(bays=[
            self._lane_bay("lane1", "#FF0000"),
            DRYER_EMPTY_BAY, DRYER_EMPTY_BAY, DRYER_EMPTY_BAY])]
        assert panel.logger.messages == []

    def test_presence_still_comes_from_the_vendor(self):
        # A lane can carry a colour for a spool that has been taken out.
        unit = self._lane_unit({"lane1": DryerLane(1, "#FF0000", load_state=True)},
                               slot_map={"lane1": 0}, slots=[{"present": False}])
        panel, _ = discovered_dryer_panel(unit)
        assert self._snap(panel) == [dryer_bambu_row(bays=[
            self._lane_bay("lane1", "#FF0000", present=False),
            DRYER_EMPTY_BAY, DRYER_EMPTY_BAY, DRYER_EMPTY_BAY])]
        assert panel.logger.messages == []

    def test_out_of_range_lane_index_is_ignored(self):
        unit = self._lane_unit({"lane9": DryerLane(99, "#FF0000")})
        panel, _ = discovered_dryer_panel(unit)
        assert self._snap(panel) == [dryer_bambu_row(bays=[
            {"present": True, "color": "", "material": ""},
            {"present": True, "color": "", "material": ""},
            DRYER_EMPTY_BAY, DRYER_EMPTY_BAY])]
        assert panel.logger.messages == []

    def test_unit_without_lanes_still_renders(self):
        panel, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2", status={"slots": []}))
        assert self._snap(panel) == [dryer_bambu_row()]
        assert panel.logger.messages == []

    def test_a_failing_live_describe_keeps_the_boot_identity(self):
        unit = DryerBambuUnit("AMS_2", status={"bridge_online": True})
        panel, _ = discovered_dryer_panel(unit, values={"poll": 0.5})
        unit.ams_model, unit.has_heater, unit.dry_max_temp = "ht", False, "hot"
        assert panel._snapshot(10.0) == 10.5
        assert panel._state == {"units": [dryer_bambu_row(online=True)], "ready": True}
        assert panel.logger.messages == []


class TestAFCDryerGetState:
    def test_state_reports_the_running_version(self):
        panel, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        assert panel.get_state() == {"units": [], "ready": False,
                                     "page_version": dryer_page_version()}
        panel._snapshot(0.0)
        state = panel.get_state()
        assert state == {"units": [dryer_bambu_row()], "ready": True,
                         "page_version": dryer_page_version()}
        # A copy: stamping and editing it leave the stored snapshot alone.
        state["ready"] = False
        assert panel._state == {"units": [dryer_bambu_row()], "ready": True}
        assert panel.logger.messages == []

    def test_version_survives_a_snapshot_replacing_the_state(self):
        # get_state stamps the version, so a swapped-in snapshot cannot lose it.
        panel, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        for when in (0.0, 2.0, 4.0):
            panel._snapshot(when)
            assert panel.get_state()["page_version"] == dryer_page_version()
        assert panel.logger.messages == []


class TestAFCDryerRequestDry:
    def _assert_queued(self, printer: MockPrinter, script: str) -> None:
        """
        One callback is queued, and running it runs exactly this script.

        :param printer: the panel's printer
        :param script: the expected g-code line
        """
        queued = dryer_queued_callbacks(printer)
        assert len(queued) == 1
        printer._gcode.run_script.assert_not_called()
        queued[0](0.0)
        printer._gcode.run_script.assert_called_once_with(script)

    @staticmethod
    def _spy_ace_start(monkeypatch: pytest.MonkeyPatch) -> List[Tuple[Any, ...]]:
        """
        Record every call to the shared ACE backend's start_script.

        :param monkeypatch: pytest's monkeypatch, which undoes the spy
        :return list: the argument tuples, filled in as calls arrive
        """
        calls: List[Tuple[Any, ...]] = []
        real = DRYER_ACE_BACKEND.start_script

        def spy(*args: Any) -> str:
            calls.append(args)
            return real(*args)

        monkeypatch.setattr(DRYER_ACE_BACKEND, "start_script", spy)
        return calls

    def test_bambu_start_builds_its_gcode(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        script = panel.request_dry("AMS_2", "start", 55, 480, 1)
        assert script == "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=55 TIME=480 ROTATE=1"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_ace_start_uses_duration_not_time(self):
        panel, printer = discovered_dryer_panel(DryerAceUnit("Ace2_1"))
        script = panel.request_dry("Ace2_1", "start", 50, 240, 0)
        assert script == "ACE_DRY UNIT=Ace2_1 TEMP=50 DURATION=240"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_stop_is_per_vendor(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"),
                                                DryerAceUnit("Ace2_1"))
        assert panel.request_dry("AMS_2", "stop", 0, 0, 0) == (
            "AFC_BAMBU_HEATER_STOP UNIT=AMS_2")
        assert panel.request_dry("Ace2_1", "stop", 0, 0, 0) == "ACE_DRY_STOP UNIT=Ace2_1"
        for callback in dryer_queued_callbacks(printer):
            callback(0.0)
        assert [c.args for c in printer._gcode.run_script.call_args_list] == [
            ("AFC_BAMBU_HEATER_STOP UNIT=AMS_2",), ("ACE_DRY_STOP UNIT=Ace2_1",)]
        assert panel.logger.messages == []

    def test_temp_is_clamped_to_the_units_ceiling(self):
        panel, printer = discovered_dryer_panel(DryerAceUnit("Ace2_1", max_temp=55.0))
        script = panel.request_dry("Ace2_1", "start", 85, 60, 0)
        assert script == "ACE_DRY UNIT=Ace2_1 TEMP=55 DURATION=60"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_ht_keeps_its_higher_ceiling(self):
        panel, printer = discovered_dryer_panel(
            DryerBambuUnit("HT", model="ht", max_temp=85, slots=1))
        script = panel.request_dry("HT", "start", 85, 60, 0)
        assert script == "AFC_BAMBU_HEATER_START UNIT=HT TEMP=85 TIME=60 ROTATE=0"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_ace_rotate_off_sends_no_rotate_field(self, monkeypatch):
        calls = self._spy_ace_start(monkeypatch)
        panel, printer = discovered_dryer_panel(DryerAceUnit("Ace2_1"))
        script = panel.request_dry("Ace2_1", "start", 50, 60, 0)
        assert script == "ACE_DRY UNIT=Ace2_1 TEMP=50 DURATION=60"
        assert calls == [("Ace2_1", 50, 60, 0)]
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_time_is_clamped_to_the_protocol_field(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        script = panel.request_dry("AMS_2", "start", 55, 999999, 0)
        assert script == "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=55 TIME=65535 ROTATE=0"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_negative_values_floor_at_zero(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        script = panel.request_dry("AMS_2", "start", -20, -5, 0)
        assert script == "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=0 TIME=0 ROTATE=0"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_unknown_unit_is_refused(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        with pytest.raises(ValueError) as excinfo:
            panel.request_dry("nope", "start", 55, 60, 0)
        assert str(excinfo.value) == "unknown unit 'nope'"
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_heaterless_unit_is_refused(self):
        panel, printer = discovered_dryer_panel(
            DryerBambuUnit("AMS_1", model="ams1", heater=False),
            values={"show_heaterless": True})
        with pytest.raises(ValueError) as excinfo:
            panel.request_dry("AMS_1", "start", 55, 60, 0)
        assert str(excinfo.value) == "AMS_1 has no drying heater"
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_unknown_action_is_refused(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        with pytest.raises(ValueError) as excinfo:
            panel.request_dry("AMS_2", "melt", 55, 60, 0)
        assert str(excinfo.value) == "unknown action 'melt'"
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    @pytest.mark.parametrize("hostile, message", [
        ("AMS_2\nM112", "unknown unit 'AMS_2\\nM112'"),       # second command
        ("AMS_2 TEMP=999", "unknown unit 'AMS_2 TEMP=999'"),  # argument injection
        ("AMS_2; RESTART", "unknown unit 'AMS_2; RESTART'"),
        ("", "unknown unit ''"),
    ])
    def test_a_name_is_never_interpolated_as_free_text(self, hostile, message):
        # The name must match a discovered unit, so it never reaches the G-code.
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        with pytest.raises(ValueError) as excinfo:
            panel.request_dry(hostile, "start", 55, 60, 0)
        assert str(excinfo.value) == message
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_commands_leave_the_http_thread(self):
        # Nothing may call into Klipper inline from a request.
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        script = panel.request_dry("AMS_2", "start", 55, 480, 0)
        printer._gcode.run_script.assert_not_called()
        self._assert_queued(printer, "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=55 "
                                     "TIME=480 ROTATE=0")
        assert script == "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=55 TIME=480 ROTATE=0"
        assert panel.logger.messages == []

    def test_a_unit_confirmed_ams2_live_can_start_drying(self):
        # AFC_BridgeBox confirms a boxed unit (no heater) as an ams2 live.
        unit = DryerBambuUnit("AMS_1", model="boxed", heater=False, max_temp=65)
        unit.pool = True
        panel, printer = discovered_dryer_panel(unit)
        unit.pool = False
        with pytest.raises(ValueError) as excinfo:
            panel.request_dry("AMS_1", "start", 55, 60, 0)
        assert str(excinfo.value) == "AMS_1 has no drying heater"
        assert dryer_queued_callbacks(printer) == []
        # The bus confirms an AMS 2 Pro; [AFC_BridgeBox ams2] sets 50.
        unit.ams_model, unit.has_heater, unit.dry_max_temp = "ams2", True, 50
        panel._snapshot(0.0)
        assert panel._state["units"][0]["has_heater"] is True
        script = panel.request_dry("AMS_1", "start", 65, 60, 0)
        assert script == "AFC_BAMBU_HEATER_START UNIT=AMS_1 TEMP=50 TIME=60 ROTATE=0"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_a_dryer_that_went_away_is_refused(self):
        unit = DryerBambuUnit("AMS_2")
        panel, printer = discovered_dryer_panel(unit)
        unit.has_heater = False
        with pytest.raises(ValueError) as excinfo:
            panel.request_dry("AMS_2", "start", 55, 60, 0)
        assert str(excinfo.value) == "AMS_2 has no drying heater"
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_other_vendors_keep_their_boot_descriptor(self):
        ace = DryerAceUnit("Ace2_1", max_temp=55.0)
        panel, printer = discovered_dryer_panel(ace)
        ace.max_dryer_temperature = 70.0
        script = panel.request_dry("Ace2_1", "start", 70, 60, 0)
        assert script == "ACE_DRY UNIT=Ace2_1 TEMP=55 DURATION=60"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_generation_does_not_change_the_commands(self):
        panel, printer = discovered_dryer_panel(DryerAce2Unit("Ace2_1"))
        script = panel.request_dry("Ace2_1", "start", 50, 240, 0)
        assert script == "ACE_DRY UNIT=Ace2_1 TEMP=50 DURATION=240"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_a_failing_live_describe_keeps_the_boot_ceiling(self):
        unit = DryerBambuUnit("AMS_2", max_temp=65)
        panel, printer = discovered_dryer_panel(unit)
        unit.dry_max_temp = "hot"
        script = panel.request_dry("AMS_2", "start", 80, 60, 0)
        assert script == "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=65 TIME=60 ROTATE=0"
        self._assert_queued(printer, script)
        assert panel.logger.messages == []

    def test_the_live_heater_read_lands_before_describe_fails(self):
        unit = DryerBambuUnit("AMS_2")
        panel, printer = discovered_dryer_panel(unit)
        unit.has_heater, unit.dry_max_temp = False, "hot"
        with pytest.raises(ValueError) as excinfo:
            panel.request_dry("AMS_2", "start", 55, 60, 0)
        assert str(excinfo.value) == "AMS_2 has no drying heater"
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_rotate_reaches_an_ace_unit(self, monkeypatch):
        calls = self._spy_ace_start(monkeypatch)
        panel, printer = discovered_dryer_panel(DryerAceUnit("Ace_1"))
        script = panel.request_dry("Ace_1", "start", 50, 60, 1)
        assert script == "ACE_DRY UNIT=Ace_1 TEMP=50 DURATION=60 ROTATE=1"
        assert calls == [("Ace_1", 50, 60, 1)]
        self._assert_queued(printer, script)
        assert panel.logger.messages == []


class TestAFCDryerRunScript:
    def test_a_failing_script_does_not_escape(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        printer._gcode.run_script.side_effect = RuntimeError("klipper is shut down")
        panel.request_dry("AMS_2", "stop", 0, 0, 0)
        queued = printer._reactor.register_async_callback.call_args.args[0]
        queued(0.0)
        printer._gcode.run_script.assert_called_once_with("AFC_BAMBU_HEATER_STOP UNIT=AMS_2")
        assert panel.logger.messages == [
            ("error", "afc_dryer: 'AFC_BAMBU_HEATER_STOP UNIT=AMS_2' failed: "
                      "klipper is shut down")]

    def test_the_script_runs_through_gcode(self):
        panel, printer = discovered_dryer_panel()
        panel._run_script("AFC_BAMBU_HEATER_STOP UNIT=AMS_2")
        printer._gcode.run_script.assert_called_once_with(
            "AFC_BAMBU_HEATER_STOP UNIT=AMS_2")
        assert panel.logger.messages == []

    def test_a_failure_is_logged(self):
        panel, printer = discovered_dryer_panel()
        printer._gcode.run_script.side_effect = RuntimeError("klipper is shut down")
        panel._run_script("AFC_BAMBU_HEATER_STOP UNIT=AMS_2")
        printer._gcode.run_script.assert_called_once_with(
            "AFC_BAMBU_HEATER_STOP UNIT=AMS_2")
        assert panel.logger.messages == [
            ("error", "afc_dryer: 'AFC_BAMBU_HEATER_STOP UNIT=AMS_2' failed: "
                      "klipper is shut down")]


class TestAFCDryerCmdStatus:
    class _Gcmd:
        """A gcode command that records what it is asked to print."""

        def __init__(self) -> None:
            self.responses: List[str] = []

        def respond_info(self, msg: str) -> None:
            self.responses.append(msg)

    def test_not_running_with_no_units(self):
        panel, _ = discovered_dryer_panel()
        gcmd = self._Gcmd()
        panel.cmd_STATUS(gcmd)
        assert gcmd.responses == [
            "AFC Unit Heaters: NOT running on http://0.0.0.0:8093/, 0 dryer(s): none"]
        assert panel.logger.messages == []

    def test_running_with_drying_idle_and_offline_units(self, dryer_fake_net):
        # Drying wins over both offline (AMS_1) and online (AMS_4).
        panel, _ = discovered_dryer_panel(
            DryerBambuUnit("AMS_1", status={"bridge_online": False, "drying": True}),
            DryerBambuUnit("AMS_2", status={"bridge_online": True}),
            DryerBambuUnit("AMS_3", status={}),
            DryerBambuUnit("AMS_4", status={"bridge_online": True, "drying": True}),
            values={"bind": "127.0.0.1", "port": 18093})
        panel._start_server()
        panel._snapshot(0.0)
        panel.logger = MockLogger()
        gcmd = self._Gcmd()
        panel.cmd_STATUS(gcmd)
        assert gcmd.responses == [
            "AFC Unit Heaters: running on http://127.0.0.1:18093/, 4 dryer(s): "
            "AMS_1 [AMS 2 Pro] drying, AMS_2 [AMS 2 Pro] idle, "
            "AMS_3 [AMS 2 Pro] offline, AMS_4 [AMS 2 Pro] drying"]
        assert panel.logger.messages == []


class TestAFCDryerGetStatus:
    def test_before_the_server_starts(self):
        panel, _ = discovered_dryer_panel()
        assert panel.get_status() == {"running": False, "port": 8093, "units": []}
        assert panel.logger.messages == []

    def test_with_the_server_running(self, dryer_fake_net):
        panel, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2"), values={"port": 18093})
        panel._start_server()
        panel._snapshot(0.0)
        panel.logger = MockLogger()
        assert panel.get_status(5.0) == {"running": True, "port": 18093,
                                         "units": [dryer_bambu_row()]}
        assert panel.logger.messages == []


class TestMakeHandler:
    def test_each_panel_gets_its_own_handler_class(self):
        busy, _ = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        busy._snapshot(0.0)
        idle, _ = discovered_dryer_panel()
        busy_cls = _make_handler(busy)
        idle_cls = _make_handler(idle)
        assert busy_cls is not idle_cls
        assert issubclass(busy_cls, BaseHTTPRequestHandler)
        assert busy_cls.server_version == "AFCDryer/1.0"
        _, (_, _, busy_body) = dryer_request(busy, "GET", "/api/state")
        _, (_, _, idle_body) = dryer_request(idle, "GET", "/api/state")
        assert json.loads(busy_body) == {"units": [dryer_bambu_row()], "ready": True,
                                         "page_version": dryer_page_version()}
        assert json.loads(idle_body) == {"units": [], "ready": False,
                                         "page_version": dryer_page_version()}
        assert busy.logger.messages == []
        assert idle.logger.messages == []

    def test_the_server_header_names_the_panel(self):
        panel, _ = discovered_dryer_panel()
        _, (_, headers, _) = dryer_request(panel, "GET", "/api/options")
        assert headers["Server"] == f"AFCDryer/1.0 Python/{sys.version.split()[0]}"
        assert panel.logger.messages == []


class TestLoadConfig:
    def test_it_builds_the_panel_from_the_section(self, caplog):
        caplog.set_level(logging.DEBUG, logger="afc_dryer")
        printer = MockPrinter()
        panel = load_config(MockConfig(printer=printer, values={"port": 9000}))
        assert isinstance(panel, AFCDryer)
        assert panel.printer is printer
        assert panel.port == 9000
        assert printer._gcode._commands == {"AFC_DRYER_STATUS": panel.cmd_STATUS}
        assert [r for r in caplog.records if r.name == "afc_dryer"] == []


class TestHandlerJson:
    def test_it_serialises_with_a_json_content_type(self):
        panel, _ = discovered_dryer_panel()
        handler, _ = dryer_request(panel, "GET", "/api/options")
        handler.wfile = DryerWriter()
        handler._json(202, {"a": [1, 2]})
        status, headers, body = dryer_parse(b"".join(handler.wfile.chunks))
        assert status == "HTTP/1.0 202 Accepted"
        assert body == b'{"a": [1, 2]}'
        assert dryer_app_headers(headers) == dryer_json_headers(body)
        assert panel.logger.messages == []


class TestHandlerSend:
    @staticmethod
    def _handler(fail_on: Optional[int] = None
                 ) -> Tuple[BaseHTTPRequestHandler, AFCDryer]:
        """
        A handler that has served one request, writing to a fresh DryerWriter.

        :param fail_on: the 1-based write that raises, or None
        :return tuple: the handler and the panel it serves
        """
        panel, _ = discovered_dryer_panel()
        handler, _ = dryer_request(panel, "GET", "/api/options")
        handler.wfile = DryerWriter(fail_on)
        return handler, panel

    def test_a_text_body_is_utf8_encoded(self):
        handler, panel = self._handler()
        handler._send(200, "héllo", "text/plain")
        status, headers, body = dryer_parse(b"".join(handler.wfile.chunks))
        assert status == "HTTP/1.0 200 OK"
        assert body == b"h\xc3\xa9llo"
        assert dryer_app_headers(headers) == {
            "Content-Type": "text/plain", "Content-Length": "6",
            "Access-Control-Allow-Origin": "*", "Cache-Control": "no-store"}
        assert panel.logger.messages == []

    def test_a_bytes_body_is_sent_as_is(self):
        handler, panel = self._handler()
        handler._send(201, b"\x00\x01", "application/octet-stream")
        status, headers, body = dryer_parse(b"".join(handler.wfile.chunks))
        assert status == "HTTP/1.0 201 Created"
        assert body == b"\x00\x01"
        assert dryer_app_headers(headers) == {
            "Content-Type": "application/octet-stream", "Content-Length": "2",
            "Access-Control-Allow-Origin": "*", "Cache-Control": "no-store"}
        assert panel.logger.messages == []

    def test_a_client_that_left_mid_body_is_ignored(self):
        handler, panel = self._handler(fail_on=2)
        handler._send(200, "page", "text/html")
        assert len(handler.wfile.chunks) == 1
        status, headers, body = dryer_parse(handler.wfile.chunks[0])
        assert status == "HTTP/1.0 200 OK"
        assert dryer_app_headers(headers) == {
            "Content-Type": "text/html", "Content-Length": "4",
            "Access-Control-Allow-Origin": "*", "Cache-Control": "no-store"}
        assert body == b""
        assert panel.logger.messages == []


class TestHandlerDoGet:
    @staticmethod
    def _page_check(path: str) -> None:
        """
        GET this path and check the full page comes back as HTML.

        :param path: request path
        """
        panel, _ = discovered_dryer_panel()
        _, (status, headers, body) = dryer_request(panel, "GET", path)
        page = _PAGE_TEMPLATE.replace("__PAGE_VERSION__", dryer_page_version())
        assert status == "HTTP/1.0 200 OK"
        assert body == page.encode("utf-8")
        assert dryer_app_headers(headers) == {
            "Content-Type": "text/html; charset=utf-8",
            "Content-Length": str(len(page.encode("utf-8"))),
            "Access-Control-Allow-Origin": "*", "Cache-Control": "no-store"}
        assert panel.logger.messages == []

    @staticmethod
    def _served_page(*units: Any) -> Tuple[str, AFCDryer]:
        """
        GET / through the real handler and return the page it served.

        The artwork and tooltips exist only in this page's script, so the
        served body is where they are checked.

        :param units: unit fakes the panel discovers
        :return tuple: the served page as text, and the panel that served it
        """
        panel, _ = discovered_dryer_panel(*units)
        _, (status, headers, body) = dryer_request(panel, "GET", "/")
        assert status == "HTTP/1.0 200 OK"
        assert headers["Content-Type"] == "text/html; charset=utf-8"
        return body.decode("utf-8"), panel

    @staticmethod
    def _js_function(page: str, opening: str) -> str:
        """
        The body of one top-level function in the served page's script.

        :param page: the served page
        :param opening: the function's opening, up to and including its "{"
        :return str: the text between that opening and the function's closing brace
        """
        assert page.count(opening) == 1
        return page.split(opening, 1)[1].split("\n}\n", 1)[0]

    def test_the_root_serves_the_page(self):
        self._page_check("/")

    def test_a_query_string_on_the_root_still_serves_the_page(self):
        self._page_check("/?v=abc")

    def test_state_with_a_trailing_slash(self):
        panel, _ = discovered_dryer_panel()
        _, (status, headers, body) = dryer_request(panel, "GET", "/api/state/")
        assert status == "HTTP/1.0 200 OK"
        assert json.loads(body) == {"units": [], "ready": False,
                                    "page_version": dryer_page_version()}
        assert dryer_app_headers(headers) == dryer_json_headers(body)
        assert panel.logger.messages == []

    def test_options_lists_temperatures_and_hours(self):
        panel, _ = discovered_dryer_panel()
        _, (status, headers, body) = dryer_request(panel, "GET", "/api/options")
        assert status == "HTTP/1.0 200 OK"
        assert json.loads(body) == {
            "temps": [40, 45, 50, 55, 60, 65, 70, 75, 80, 85],
            "times": [{"minutes": 60, "label": "1 h"}, {"minutes": 120, "label": "2 h"},
                      {"minutes": 180, "label": "3 h"}, {"minutes": 240, "label": "4 h"},
                      {"minutes": 300, "label": "5 h"}, {"minutes": 360, "label": "6 h"},
                      {"minutes": 420, "label": "7 h"}, {"minutes": 480, "label": "8 h"},
                      {"minutes": 540, "label": "9 h"}, {"minutes": 600, "label": "10 h"},
                      {"minutes": 660, "label": "11 h"},
                      {"minutes": 720, "label": "12 h"}]}
        assert dryer_app_headers(headers) == dryer_json_headers(body)
        assert panel.logger.messages == []

    def test_an_unknown_path_is_404(self):
        panel, _ = discovered_dryer_panel()
        _, (status, headers, body) = dryer_request(panel, "GET", "/api/nothing")
        assert status == "HTTP/1.0 404 Not Found"
        assert json.loads(body) == {"error": "not found"}
        assert dryer_app_headers(headers) == dryer_json_headers(body)
        assert panel.logger.messages == []

    def test_page_reloads_only_once_on_a_mismatch(self):
        # A tab left open across a deploy runs the old markup. It reloads when
        # /api/state reports another version, and the latch stops a reload loop.
        page, panel = self._served_page()
        _, (_, _, state) = dryer_request(panel, "GET", "/api/state")
        version = json.loads(state)["page_version"]
        tick = self._js_function(page, "function tick(){")
        guard = ("if (!reloading && state.page_version"
                 " && state.page_version !== MY_VERSION){")
        assert version == dryer_page_version()
        assert page.count(f'var MY_VERSION = "{version}";') == 1
        assert page.count("var reloading = false;") == 1
        assert page.count("if (!reloading && state.page_version") == 1
        assert page.count("reloading = true;") == 1
        assert page.count("location.reload();") == 1
        assert tick.count(guard) == 1
        assert (tick.index(guard) < tick.index("reloading = true;")
                < tick.index("location.reload();"))
        assert panel.logger.messages == []

    def test_the_ace_grid_is_gone(self):
        # An ACE stands its four spools side by side as an AMS does, so the
        # only unit with its own artwork is the single-bay HT.
        page, panel = self._served_page()
        art = self._js_function(page, "function art(u){")
        assert "u.model === 'ace'" not in page
        # the 2-column arithmetic that produced the stack
        assert "j % 2" not in page
        assert art.count("u.model === ") == 1
        assert panel.logger.messages == []

    def test_the_ht_keeps_its_single_bay_tower(self):
        # One spool, genuinely a different shape, not a row of one.
        page, panel = self._served_page()
        art = self._js_function(page, "function art(u){")
        tower = art.split("if (u.model === 'ht'){", 1)[1].split("\n  }\n", 1)[0]
        assert page.count("u.model === 'ht'") == 1
        assert tower.count("bay(") == 1
        assert tower.count("bays[0] || {}") == 1
        assert tower.count('aria-label="AMS HT"') == 1
        assert panel.logger.messages == []

    def test_the_row_renderer_spaces_bays_horizontally(self):
        # x advances per bay, y is fixed: that is what "side by side" means.
        page, panel = self._served_page()
        art = self._js_function(page, "function art(u){")
        assert page.count("bay(21 + i*24, 44,") == 1
        assert art.count("for (var i=0;i<bays.length;i++) b += bay(21 + i*24, 44,") == 1
        assert panel.logger.messages == []

    def test_the_label_is_taken_from_the_unit(self):
        # Shared renderer, so the aria-label can no longer be hardcoded "AMS";
        # it reads the label /api/state publishes for each unit.
        page, panel = self._served_page(DryerBambuUnit("AMS_2"))
        panel._snapshot(0.0)
        _, (_, _, state) = dryer_request(panel, "GET", "/api/state")
        art = self._js_function(page, "function art(u){")
        assert [u["label"] for u in json.loads(state)["units"]] == ["AMS 2 Pro"]
        assert page.count("u.label || u.model") == 1
        assert art.count("var label = u.label || u.model || 'unit';") == 1
        assert art.count("aria-label=\"' + label + '\">'") == 1
        assert 'aria-label="AMS">' not in page
        assert panel.logger.messages == []

    def test_it_uses_a_native_svg_title(self):
        # No JS positioning, and screen readers get it for free.
        page, panel = self._served_page()
        bay = self._js_function(page, "function bay(cx, cy, r, s, hub){")
        assert page.count("'<title>' + tip + '</title>'") == 1
        assert bay.count("var tip = spoolTip(s);") == 1
        assert bay.count("'<title>' + tip + '</title>'") == 1
        assert panel.logger.messages == []

    def test_the_text_is_escaped(self):
        # Spool names come from Spoolman and are user-controlled, so every
        # return from spoolTip goes through esc, and esc turns & first.
        page, panel = self._served_page()
        tip = self._js_function(page, "function spoolTip(s){")
        returns = [ln.strip() for ln in tip.splitlines() if ln.strip().startswith("return")]
        escs = [part.split("\n}\n", 1)[0] for part in page.split("function esc(")[1:]]
        assert "function esc(" in page
        assert page.count("return esc(out.join(") == 1
        assert returns == ["return out.length ? esc(out[0] + ', empty') : '';",
                           "return esc(out.join('\\n'));"]
        assert escs
        for esc in escs:
            marks = [esc.index(m) for m in ("'&amp;'", "'&lt;'", "'&gt;'", "'&quot;'")]
            assert marks == sorted(marks)
        assert panel.logger.messages == []

    def test_an_empty_bay_still_names_its_lane(self):
        # Hovering an empty bay says which lane it is rather than nothing.
        page, panel = self._served_page()
        tip = self._js_function(page, "function spoolTip(s){")
        empty = tip.split("if (!s.present){", 1)[1].split("\n  }\n", 1)[0]
        assert page.count("', empty'") == 1
        assert empty.count("return out.length ? esc(out[0] + ', empty') : '';") == 1
        assert tip.index("if (s.lane) out.push(s.lane);") < tip.index("if (!s.present){")
        assert panel.logger.messages == []

    def test_blank_fields_are_omitted_rather_than_shown_empty(self):
        page, panel = self._served_page()
        tip = self._js_function(page, "function spoolTip(s){")
        assert page.count(".filter(Boolean).join(' ')") == 1
        assert tip.count(".filter(Boolean).join(' ')") == 1
        assert tip.count("if (name) out.push(name);") == 1
        for field in ("weight", "temp", "spool_id"):
            assert tip.count(f"if (s.{field} != null) out.push(") == 1
        assert panel.logger.messages == []

    def test_each_bay_title_is_scoped_to_its_own_group(self):
        # The bug: without the <g>, the first bay's <title> became the whole
        # SVG's tooltip and every spool showed it.
        page, panel = self._served_page()
        bay = self._js_function(page, "function bay(cx, cy, r, s, hub){")
        drawn = bay[bay.index("return "):]
        assert page.count("'<g>' + (tip ? '<title>' + tip + '</title>' : '')") == 1
        assert page.count("</g>'") == 1
        assert drawn.startswith(
            "return '<g>' + (tip ? '<title>' + tip + '</title>' : '') +")
        assert drawn.rstrip().endswith("</g>';")
        assert (drawn.count("<g>"), drawn.count("</g>")) == (1, 1)
        assert panel.logger.messages == []


class TestHandlerDoOptions:
    def test_the_preflight_allows_the_iframe(self):
        panel, _ = discovered_dryer_panel()
        _, (status, headers, body) = dryer_request(panel, "OPTIONS", "/api/dry")
        assert status == "HTTP/1.0 204 No Content"
        assert dryer_app_headers(headers) == {
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Methods": "GET, POST",
            "Access-Control-Allow-Headers": "Content-Type"}
        assert body == b""
        assert panel.logger.messages == []


class TestHandlerDoPost:
    @staticmethod
    def _post(panel: AFCDryer, payload: Any, path: str = "/api/dry"
              ) -> Tuple[BaseHTTPRequestHandler, Tuple[str, Dict[str, str], bytes]]:
        """
        POST a JSON payload with a correct Content-Length.

        :param panel: the panel to serve
        :param payload: any json.dumps-able value
        :param path: request path
        :return tuple: the handler and the parsed response
        """
        body = json.dumps(payload).encode("utf-8")
        return dryer_request(panel, "POST", path, [f"Content-Length: {len(body)}"], body)

    @staticmethod
    def _answer(response: Tuple[str, Dict[str, str], bytes]) -> Tuple[str, Any]:
        """
        The status line and decoded JSON body, after checking the JSON headers.

        :param response: a parsed response
        :return tuple: status line and body object
        """
        status, headers, body = response
        assert dryer_app_headers(headers) == dryer_json_headers(body)
        return status, json.loads(body)

    def test_a_start_is_queued_and_echoed(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {
            "unit": "AMS_2", "action": "start", "temp": 50, "minutes": 120,
            "rotate": 1})
        script = "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=50 TIME=120 ROTATE=1"
        assert self._answer(response) == ("HTTP/1.0 200 OK", {"ok": True, "queued": script})
        queued = dryer_queued_callbacks(printer)
        assert len(queued) == 1
        printer._gcode.run_script.assert_not_called()
        queued[0](0.0)
        printer._gcode.run_script.assert_called_once_with(script)
        assert panel.logger.messages == []

    def test_omitted_fields_take_their_defaults(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {"unit": "AMS_2", "action": "start"})
        assert self._answer(response) == ("HTTP/1.0 200 OK", {
            "ok": True,
            "queued": "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=55 TIME=480 ROTATE=0"})
        assert len(dryer_queued_callbacks(printer)) == 1
        assert panel.logger.messages == []

    def test_a_query_string_and_trailing_slash_still_reach_dry(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {"unit": "AMS_2", "action": "stop"},
                                 path="/api/dry/?from=panel")
        assert self._answer(response) == ("HTTP/1.0 200 OK", {
            "ok": True, "queued": "AFC_BAMBU_HEATER_STOP UNIT=AMS_2"})
        assert len(dryer_queued_callbacks(printer)) == 1
        assert panel.logger.messages == []

    def test_another_path_is_404_and_queues_nothing(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {"unit": "AMS_2", "action": "stop"},
                                 path="/api/state")
        assert self._answer(response) == ("HTTP/1.0 404 Not Found",
                                          {"error": "not found"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_the_root_path_is_404(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {"unit": "AMS_2", "action": "stop"}, path="/")
        assert self._answer(response) == ("HTTP/1.0 404 Not Found",
                                          {"error": "not found"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_no_body_reads_as_an_empty_object(self):
        # No Content-Length: nothing is read and {} reaches request_dry.
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = dryer_request(panel, "POST", "/api/dry")
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request",
                                          {"error": "unknown unit ''"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_a_junk_content_length_is_a_bad_body(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = dryer_request(panel, "POST", "/api/dry", ["Content-Length: abc"])
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request", {
            "error": "bad request body: invalid literal for int() with base 10: 'abc'"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_malformed_json_is_a_bad_body(self):
        raw = b"{not json"
        try:
            json.loads(raw)
        except ValueError as err:
            detail = str(err)
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = dryer_request(panel, "POST", "/api/dry",
                                    [f"Content-Length: {len(raw)}"], raw)
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request",
                                          {"error": f"bad request body: {detail}"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_a_negative_content_length_is_a_bad_body(self):
        # A valid stop follows; reading to EOF would have queued it.
        raw = b'{"unit": "AMS_2", "action": "stop"}'
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = dryer_request(panel, "POST", "/api/dry", ["Content-Length: -1"], raw)
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request", {
            "error": "bad request body: negative Content-Length -1"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    @pytest.mark.parametrize("payload, kind", [
        ([{"unit": "AMS_2", "action": "stop"}], "list"),
        ("AMS_2", "str"),
        (5, "int"),
        (None, "NoneType"),
    ])
    def test_a_body_that_is_not_an_object_is_a_bad_body(self, payload, kind):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, payload)
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request", {
            "error": f"bad request body: expected a JSON object, got {kind}"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_a_null_number_is_refused(self):
        try:
            int(None)
        except TypeError as err:
            detail = str(err)
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {"unit": "AMS_2", "action": "start", "temp": None})
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request", {"error": detail})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_an_infinite_number_is_refused(self):
        try:
            int(float("inf"))
        except OverflowError as err:
            detail = str(err)
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {"unit": "AMS_2", "action": "start",
                                         "temp": float("inf")})
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request", {"error": detail})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_a_non_numeric_number_is_refused(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {"unit": "AMS_2", "action": "start",
                                         "minutes": "all night"})
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request", {
            "error": "invalid literal for int() with base 10: 'all night'"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []

    def test_a_refused_request_is_reported(self):
        panel, printer = discovered_dryer_panel(DryerBambuUnit("AMS_2"))
        _, response = self._post(panel, {"unit": "AMS_2", "action": "melt"})
        assert self._answer(response) == ("HTTP/1.0 400 Bad Request",
                                          {"error": "unknown action 'melt'"})
        assert dryer_queued_callbacks(printer) == []
        assert panel.logger.messages == []


class TestHandlerLogMessage:
    def test_it_writes_nothing(self, capsys):
        panel, _ = discovered_dryer_panel()
        handler, _ = dryer_request(panel, "GET", "/api/options")
        capsys.readouterr()
        assert handler.log_message("%s %s", "GET", "/") is None
        assert capsys.readouterr() == ("", "")
        assert panel.logger.messages == []

    def test_a_served_request_leaves_no_access_log(self, capsys):
        panel, _ = discovered_dryer_panel()
        capsys.readouterr()
        _, (status, _, _) = dryer_request(panel, "GET", "/missing")
        assert status == "HTTP/1.0 404 Not Found"
        assert capsys.readouterr() == ("", "")
        assert panel.logger.messages == []


class TestPage:
    def test_served_page_has_the_version_substituted(self):
        # The page compares its own version with the one /api/state reports.
        panel, _ = discovered_dryer_panel()
        _, (_, _, page) = dryer_request(panel, "GET", "/")
        _, (_, _, state) = dryer_request(panel, "GET", "/api/state")
        served = page.decode("utf-8")
        version = dryer_page_version()
        assert json.loads(state)["page_version"] == version
        assert served.count(f'var MY_VERSION = "{version}";') == 1
        assert served.count("__PAGE_VERSION__") == 0
        assert panel.logger.messages == []


class TestPageVersion:
    def test_version_is_derived_from_the_template(self):
        panel, _ = discovered_dryer_panel()
        _, (_, _, state) = dryer_request(panel, "GET", "/api/state")
        expected = hashlib.md5(_PAGE_TEMPLATE.encode("utf-8")).hexdigest()[:8]
        assert PAGE_VERSION == expected
        assert json.loads(state)["page_version"] == expected
        assert panel.logger.messages == []


class TestPageTemplate:
    def test_template_keeps_the_placeholder(self):
        # Hashing the template, not the served page, keeps the version
        # non-circular; the served page fills the one placeholder in. With no
        # placeholder, or two, the served page would not equal the splice.
        before, _, after = _PAGE_TEMPLATE.partition("__PAGE_VERSION__")
        panel, _ = discovered_dryer_panel()
        _, (_, _, page) = dryer_request(panel, "GET", "/")
        served = page.decode("utf-8")
        assert served == before + dryer_page_version() + after
        assert served != _PAGE_TEMPLATE
        assert panel.logger.messages == []


class TestAceBackendObjectPrefixes:
    def test_ace_backend_covers_both_unit_classes(self):
        # The ACE 2 Pro registers under its own prefix; missing it hid a live unit.
        ace1, ace2 = DryerAceUnit("Ace_1"), DryerAce2Unit("Ace2_1")
        printer = dryer_sensor_printer({"AFC_ACE Ace_1": ace1, "AFC_ACE2 Ace2_1": ace2,
                                        "AFC_BambuAMS AMS_2": DryerBambuUnit("AMS_2")})
        assert _AceBackend().object_prefixes == ("AFC_ACE", "AFC_ACE2")
        assert _AceBackend().find(printer) == [ace1, ace2]


class TestBackendInterface:
    @pytest.mark.parametrize("backend, unit, expected", [
        (_BambuBackend(), DryerBambuUnit("AMS_2"), {
            "kind": "bambu", "has_heater": True,
            "describe": {"model": "ams2", "label": "AMS 2 Pro", "max_temp": 65,
                         "slots": 4},
            "snapshot": {"online": False, "drying": False, "temperature": None,
                         "humidity": None, "target": None, "remaining": None,
                         "duration": None, "note": "", "error": ""},
            "start": "AFC_BAMBU_HEATER_START UNIT=AMS_2 TEMP=55 TIME=60 ROTATE=1",
            "stop": "AFC_BAMBU_HEATER_STOP UNIT=AMS_2"}),
        (_AceBackend(), DryerAceUnit("Ace_1"), {
            "kind": "ace", "has_heater": True,
            "describe": {"model": "ace", "label": "ACE", "max_temp": 55, "slots": 4},
            "snapshot": {"online": False, "drying": False, "temperature": None,
                         "humidity": None, "target": None, "remaining": None,
                         "duration": None, "note": ""},
            "start": "ACE_DRY UNIT=Ace_1 TEMP=55 DURATION=60 ROTATE=1",
            "stop": "ACE_DRY_STOP UNIT=Ace_1"}),
    ], ids=["bambu", "ace"])
    def test_every_backend_implements_the_hooks(self, backend, unit, expected):
        # Every hook answers for its own unit; none falls through to _Backend.
        printer = dryer_sensor_printer({f"{unit.PREFIX} {unit.name}": unit})
        assert backend.kind == expected["kind"]
        assert backend.find(printer) == [unit]
        assert backend.has_heater(unit) is expected["has_heater"]
        assert backend.describe(unit) == expected["describe"]
        assert backend.snapshot(unit, {}) == expected["snapshot"]
        assert backend.slots(unit, {}) == []
        assert backend.start_script(unit.name, 55, 60, 1) == expected["start"]
        assert backend.stop_script(unit.name) == expected["stop"]
