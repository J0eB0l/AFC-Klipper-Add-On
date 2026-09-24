"""Unit tests for extras/AFC_autocal.py."""

from __future__ import annotations

import configparser
import inspect
import sys
import threading
import types
from typing import Any, Callable, Dict, List, Optional, Tuple

import pytest

from extras.AFC import State
from extras.AFC_autocal import AFC_autocal, load_config
import extras.AFC_autocal as autocal_mod
from extras.AFC_lane import AFCLane
from extras.AFC_spool import AFCSpool
from tests.bambu_helpers import make_afc_lane, make_afc_spool, make_printer


_AUTOCAL_REQUIRED = object()


AUTOCAL_TOOL_PATCH_LOG = ("info", "AFC_autocal: set_tool_loaded now emits afc:tool_loaded on load")


AUTOCAL_SPOOL_PATCH_LOG = ("info", "AFC_autocal: set_spoolID now emits afc:spool_assigned")


# Taken at collection, before any test can have left a class patch installed.
AUTOCAL_GENUINE_SET_TOOL_LOADED = AFCLane.set_tool_loaded


AUTOCAL_GENUINE_SET_SPOOLID = AFCSpool.set_spoolID


class AutocalLogger:
    """AFC's logger: every call recorded as a (level, message) tuple."""

    def __init__(self) -> None:
        self.messages: List[Tuple[str, str]] = []

    def info(self, message: str, console_only: bool = False) -> None:
        self.messages.append(("info", message))

    def warning(self, message: str) -> None:
        self.messages.append(("warning", message))

    def debug(self, message: str, only_debug: bool = False,
              traceback: Optional[str] = None) -> None:
        self.messages.append(("debug", message))

    def error(self, message: str, traceback: Optional[str] = None,
              stack_name: str = "") -> None:
        self.messages.append(("error", message))


class AutocalReactor:
    """Klipper's reactor: a fixed clock; callbacks are recorded and run on demand."""

    NOW = 0.0

    def __init__(self, now: float = 1000.0) -> None:
        self.now = now
        self.callbacks: List[Tuple[Callable[[float], Any], float]] = []
        self.async_callbacks: List[Callable[[float], Any]] = []

    def monotonic(self) -> float:
        return self.now

    def register_callback(self, callback: Callable[[float], Any],
                          waketime: float = NOW) -> None:
        self.callbacks.append((callback, waketime))

    def register_async_callback(self, callback: Callable[[float], Any],
                                waketime: float = NOW) -> None:
        self.async_callbacks.append(callback)

    def waketimes(self) -> List[float]:
        """:return List[float]: the waketime of every queued callback"""
        return [waketime for _, waketime in self.callbacks]

    def run_pending(self) -> None:
        """Run every queued callback once: reactor callbacks, then async ones."""
        pending = [cb for cb, _ in self.callbacks] + self.async_callbacks
        self.callbacks = []
        self.async_callbacks = []
        for callback in pending:
            callback(self.now)


class AutocalGcode:
    """Klipper's gcode: registered commands, mux handlers and the scripts run."""

    def __init__(self) -> None:
        self.commands: Dict[str, Tuple[Callable[..., Any], Optional[str]]] = {}
        self.mux_commands: Dict[str, Tuple[str, Dict[str, Callable[..., Any]]]] = {}
        self.scripts: List[str] = []
        self.command_scripts: List[str] = []
        self.on_script: Optional[Callable[[str], None]] = None

    def register_command(self, cmd: str, func: Callable[..., Any],
                         when_not_ready: bool = False, desc: Optional[str] = None) -> None:
        self.commands[cmd] = (func, desc)

    def register_mux_command(self, cmd: str, key: str, value: str,
                             func: Callable[..., Any], desc: Optional[str] = None) -> None:
        self.mux_commands.setdefault(cmd, (key, {}))[1][value] = func

    def run_script(self, script: str) -> None:
        self.scripts.append(script)
        if self.on_script is not None:
            self.on_script(script)

    def run_script_from_command(self, script: str) -> None:
        self.command_scripts.append(script)
        if self.on_script is not None:
            self.on_script(script)


class AutocalGcmd:
    """A gcode command: what the handler responds."""

    def __init__(self) -> None:
        self.responses: List[str] = []

    def respond_info(self, msg: str, log: bool = True) -> None:
        self.responses.append(msg)


class AutocalPaHandler:
    """A per-extruder SET_PRESSURE_ADVANCE mux handler: records every gcmd."""

    def __init__(self) -> None:
        self.calls: List[AutocalGcmd] = []

    def __call__(self, gcmd: AutocalGcmd) -> None:
        self.calls.append(gcmd)


class AutocalExtruderObj:
    """An [AFC_extruder] section: its name, its Klipper extruder name, its load flag."""

    def __init__(self, name: str = "extruder", th_extruder_name: Optional[str] = None,
                 load_active: bool = False) -> None:
        self.name = name
        self.th_extruder_name: Optional[str] = th_extruder_name or name
        self.load_active = load_active


class AutocalLane:
    """An AFC lane: the attributes AFC_autocal reads."""

    def __init__(self, name: str = "lane1", spool_id: Any = 5, *,
                 tool_loaded: bool = False, load_state: bool = True,
                 extruder: Optional[str] = "extruder",
                 th_extruder_name: Optional[str] = None,
                 load_active: bool = False) -> None:
        self.name = name
        self.spool_id = spool_id
        self.tool_loaded = tool_loaded
        self.load_state = load_state
        self.extruder_obj: Optional[AutocalExtruderObj] = (
            AutocalExtruderObj(extruder, th_extruder_name, load_active)
            if extruder is not None else None)


class AutocalFunction:
    """afcFunction: the print state and the current lane AFC reports."""

    def __init__(self) -> None:
        self.printing = False
        self.current_lane: Optional[AutocalLane] = None
        self.error: Optional[Exception] = None

    def is_printing(self) -> bool:
        return self.printing

    def get_current_lane_obj(self) -> Optional[AutocalLane]:
        if self.error is not None:
            raise self.error
        return self.current_lane


class AutocalAFC:
    """AFC's core: logger, lanes, prep and state, function, moonraker and spoolman."""

    def __init__(self) -> None:
        self.logger = AutocalLogger()
        self.lanes: Dict[str, AutocalLane] = {}
        self.prep_done = True
        self.current_state: Any = State.IDLE
        self.function = AutocalFunction()
        self.moonraker: Optional[object] = None
        self.spoolman: Optional[object] = None


class AutocalKlipperExtruder:
    """A Klipper extruder as the toolhead reports it."""

    def __init__(self, name: str) -> None:
        self.name = name

    def get_name(self) -> str:
        return self.name


class AutocalToolhead:
    """Klipper's toolhead: its active extruder, None, or a lookup that raises."""

    def __init__(self, active: Optional[str] = "extruder",
                 error: Optional[Exception] = None) -> None:
        self.extruder = AutocalKlipperExtruder(active) if active is not None else None
        self.error = error

    def get_extruder(self) -> Optional[AutocalKlipperExtruder]:
        if self.error is not None:
            raise self.error
        return self.extruder


class AutocalFlowCalibrator:
    """The U1 flow_calibrator: K per Klipper extruder, and every K set."""

    def __init__(self) -> None:
        self._current_k: Dict[str, float] = {}
        self.applied: List[Tuple[Optional[AutocalKlipperExtruder], float]] = []
        self.error: Optional[Exception] = None

    def _set_pressure_advance(self, extruder: Optional[AutocalKlipperExtruder],
                              k: float) -> None:
        if self.error is not None:
            raise self.error
        self.applied.append((extruder, k))


class AutocalExtruderStepper:
    """A Klipper extruder_stepper: every (pressure advance, smooth time) set."""

    def __init__(self, smooth_time: float = 0.04,
                 config_smooth_time: Optional[float] = None) -> None:
        self.pressure_advance_smooth_time = smooth_time
        if config_smooth_time is not None:
            self.config_smooth_time = config_smooth_time
        self.applied: List[Tuple[float, float]] = []

    def _set_pressure_advance(self, pressure_advance: float, smooth_time: float) -> None:
        self.applied.append((pressure_advance, smooth_time))


class AutocalPrinterExtruder:
    """A Klipper extruder object as printer.lookup_object returns it."""

    def __init__(self, stepper: Optional[AutocalExtruderStepper]) -> None:
        self.extruder_stepper = stepper


class AutocalPrinter:
    """Klipper's printer: objects, event handlers and AFC built on first load."""

    def __init__(self) -> None:
        self.reactor = AutocalReactor()
        self.gcode = AutocalGcode()
        self.afc = AutocalAFC()
        self.objects: Dict[str, Any] = {"gcode": self.gcode}
        self.event_handlers: List[Tuple[str, Callable[..., Any]]] = []
        self.loaded: List[str] = []

    def get_reactor(self) -> AutocalReactor:
        return self.reactor

    def lookup_object(self, name: str, default: Any = _AUTOCAL_REQUIRED) -> Any:
        if name in self.objects:
            return self.objects[name]
        if default is _AUTOCAL_REQUIRED:
            error_str = f"Unknown config object '{name}'"
            raise configparser.Error(error_str)
        return default

    def load_object(self, config: AutocalConfig, section: str) -> Any:
        """
        klippy's load_object for the one section this module loads: AFC.

        :param config: the calling section's config
        :param section: the section to load
        :return Any: the AFC core, registered on first load
        """
        self.loaded.append(section)
        self.objects.setdefault(section, self.afc)
        return self.objects[section]

    def register_event_handler(self, event: str, callback: Callable[..., Any]) -> None:
        self.event_handlers.append((event, callback))


class AutocalConfig:
    """The [AFC_autocal] section."""

    def __init__(self, printer: AutocalPrinter,
                 values: Optional[Dict[str, Any]] = None) -> None:
        self.printer = printer
        self.values = dict(values or {})

    def get_printer(self) -> AutocalPrinter:
        return self.printer

    def get_name(self) -> str:
        return "AFC_autocal"

    def _value(self, option: str, default: Any) -> Any:
        if option in self.values:
            return self.values[option]
        if default is _AUTOCAL_REQUIRED:
            error_str = f"Option '{option}' in section 'AFC_autocal' must be specified"
            raise configparser.Error(error_str)
        return default

    def get(self, option: str, default: Any = _AUTOCAL_REQUIRED) -> Any:
        return self._value(option, default)

    def getboolean(self, option: str, default: Any = _AUTOCAL_REQUIRED) -> Any:
        return self._value(option, default)

    def getfloat(self, option: str, default: Any = _AUTOCAL_REQUIRED, **limits: Any) -> Any:
        return self._value(option, default)


class AutocalSpoolman:
    """SpoolmanClient stand-in: the one client every SpoolmanClient(moonraker) returns."""

    def __init__(self) -> None:
        self.flow_k: Dict[int, float] = {}
        self.read_errors: Dict[int, Exception] = {}
        self.built: List[object] = []
        self.reads: List[int] = []
        self.writes: List[Tuple[int, float]] = []

    def __call__(self, moonraker: object) -> AutocalSpoolman:
        self.built.append(moonraker)
        return self

    def read_flow_k(self, spool_id: int) -> Optional[float]:
        self.reads.append(spool_id)
        if spool_id in self.read_errors:
            raise self.read_errors[spool_id]
        return self.flow_k.get(spool_id)

    def write_flow_k(self, spool_id: int, k: float) -> None:
        self.writes.append((spool_id, k))
        self.flow_k[spool_id] = k


class AutocalThread:
    """A threading.Thread whose start() only records it; AutocalThreads runs it."""

    def __init__(self, threads: AutocalThreads, target: Callable[[], None],
                 name: str, daemon: bool) -> None:
        self.threads = threads
        self.target = target
        self.name = name
        self.daemon = daemon

    def start(self) -> None:
        if self.threads.error is not None:
            raise self.threads.error
        self.threads.started.append((self.name, self.daemon))
        self.threads.targets.append(self.target)


class AutocalThreads:
    """The threads AFC_autocal starts: (name, daemon) per start, targets run on demand."""

    def __init__(self) -> None:
        self.started: List[Tuple[str, bool]] = []
        self.targets: List[Callable[[], None]] = []
        self.error: Optional[Exception] = None

    def thread(self, target: Callable[[], None], name: str, daemon: bool) -> AutocalThread:
        return AutocalThread(self, target, name, daemon)

    def run_all(self) -> None:
        """Run every started thread's target, in start order."""
        targets, self.targets = self.targets, []
        for target in targets:
            target()


def make_autocal(values: Optional[Dict[str, Any]] = None, *, calibrator: bool = False,
                 ready: bool = False, spoolman: bool = False,
                 toolhead: Optional[str] = "extruder") -> AFC_autocal:
    """
    An AFC_autocal through its real __init__ on an AutocalPrinter.

    :param values: the [AFC_autocal] options
    :param calibrator: register a flow_calibrator (U1 mode); consumer mode without
    :param ready: take AFC's core and stamp the ready time past the startup grace,
      the state _handle_ready leaves, without running its class patches
    :param spoolman: give AFC a moonraker and spoolman, so _spoolman() builds a client
    :param toolhead: the toolhead's active extruder; no toolhead object when None
    :return AFC_autocal: the module; its printer is ``cal.printer``
    """
    printer = AutocalPrinter()
    if calibrator:
        printer.objects["flow_calibrator"] = AutocalFlowCalibrator()
    if toolhead is not None:
        printer.objects["toolhead"] = AutocalToolhead(toolhead)
    if spoolman:
        printer.afc.moonraker = object()
        printer.afc.spoolman = object()
    cal = AFC_autocal(AutocalConfig(printer, values))
    if ready:
        cal.afc = printer.afc
        cal._ready_time = printer.reactor.monotonic() - cal._startup_cal_grace - 1.0
    return cal


def add_autocal_lane(cal: AFC_autocal, name: str = "lane1", spool_id: Any = 5,
                     **lane_kw: Any) -> AutocalLane:
    """
    :param cal: the module; the lane goes in its printer's AFC lane table
    :param name: the lane name
    :param spool_id: the lane's spool id
    :param lane_kw: further AutocalLane arguments
    :return AutocalLane: the lane
    """
    lane = AutocalLane(name, spool_id, **lane_kw)
    cal.printer.afc.lanes[name] = lane
    return lane


def add_autocal_stepper(cal: AFC_autocal, name: str = "extruder", smooth_time: float = 0.04,
                        config_smooth_time: Optional[float] = None) -> AutocalExtruderStepper:
    """
    :param cal: the module; the extruder is registered on its printer
    :param name: the Klipper extruder name
    :param smooth_time: its pressure_advance_smooth_time
    :param config_smooth_time: its configured smooth time, when it has one
    :return AutocalExtruderStepper: the extruder's stepper
    """
    stepper = AutocalExtruderStepper(smooth_time, config_smooth_time)
    cal.printer.objects[name] = AutocalPrinterExtruder(stepper)
    return stepper


def add_autocal_pa_handler(cal: AFC_autocal, extruder: str) -> AutocalPaHandler:
    """
    :param cal: the module; the handler is registered on its printer's gcode
    :param extruder: the extruder the SET_PRESSURE_ADVANCE handler serves
    :return AutocalPaHandler: the handler
    """
    handler = AutocalPaHandler()
    cal.printer.gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", extruder,
                                           handler)
    return handler


@pytest.fixture
def autocal_spoolman(monkeypatch: pytest.MonkeyPatch) -> AutocalSpoolman:
    """SpoolmanClient replaced by one AutocalSpoolman for the test."""
    spoolman = AutocalSpoolman()
    monkeypatch.setattr(autocal_mod, "SpoolmanClient", spoolman)
    return spoolman


@pytest.fixture
def autocal_threads(monkeypatch: pytest.MonkeyPatch) -> AutocalThreads:
    """The module's threading replaced: threads are recorded and run by run_all()."""
    threads = AutocalThreads()
    monkeypatch.setattr(autocal_mod, "threading", types.SimpleNamespace(
        Thread=threads.thread, current_thread=threading.current_thread))
    return threads


@pytest.fixture
def autocal_class_patches(monkeypatch: pytest.MonkeyPatch) -> None:
    """AFCLane and AFCSpool start with upstream's methods and get them back after the test."""
    monkeypatch.setattr(AFCLane, "set_tool_loaded", AUTOCAL_GENUINE_SET_TOOL_LOADED)
    monkeypatch.setattr(AFCLane, "_afc_autocal_emit_patched", False, raising=False)
    monkeypatch.setattr(AFCSpool, "set_spoolID", AUTOCAL_GENUINE_SET_SPOOLID)
    monkeypatch.setattr(AFCSpool, "_afc_autocal_spoolid_patched", False, raising=False)


class TestAFCautocalInit:
    def test_toggles_default_off(self):
        cal = make_autocal()
        assert cal.apply_stored_k is False
        assert cal.auto_calibrate is False
        assert cal._startup_cal_grace == 30.0
        assert cal._ready_time is None
        assert cal._lane_flow_k == {}
        assert cal._k_fetch_inflight == set()
        assert cal._staged_handled == {}
        assert cal._staged_pending == set()
        assert cal._cal_pending == set()
        assert cal._managed_extruders == set()
        assert cal._wrapped_extruders == set()
        assert cal.logger.messages == []

    def test_enabled_master_turns_both_on(self):
        cal = make_autocal({"enabled": True})
        assert cal.apply_stored_k is True
        assert cal.auto_calibrate is True
        assert cal.logger.messages == []

    def test_explicit_toggles_override_master(self):
        no_cal = make_autocal({"enabled": True, "auto_calibrate": False})
        assert no_cal.apply_stored_k is True
        assert no_cal.auto_calibrate is False
        assert no_cal.logger.messages == []
        no_apply = make_autocal({"enabled": True, "apply_stored_k": False})
        assert no_apply.apply_stored_k is False
        assert no_apply.auto_calibrate is True
        assert no_apply.logger.messages == []

    def test_calibrate_gcode_default_and_override(self):
        default = make_autocal()
        assert default.calibrate_gcode == "FLOW_CALIBRATE"
        assert default.logger.messages == []
        cal = make_autocal({"calibrate_gcode": "MY_CAL", "startup_cal_grace": 5.0})
        assert cal.calibrate_gcode == "MY_CAL"
        assert cal._startup_cal_grace == 5.0
        assert cal.logger.messages == []

    def test_commands_and_events_registered(self):
        cal = make_autocal()
        assert cal.printer.gcode.commands == {
            "AFC_APPLY_LANE_FLOW_K": (cal.cmd_APPLY_LANE_FLOW_K,
                                      "Apply stored flow K for the current lane"),
            "AFC_CALIBRATE_LANE_FLOW_K": (
                cal.cmd_CALIBRATE_LANE_FLOW_K,
                "Run flow calibration on the current lane and store K"),
        }
        assert cal.printer.event_handlers == [
            ("klippy:ready", cal._handle_ready),
            ("afc:tool_loaded", cal._handle_tool_loaded),
            ("afc:spool_assigned", cal._handle_spool_assigned),
            ("homing:home_rails_end", cal._handle_home_rails_end),
            ("extruder:activate_extruder", cal._handle_activate_extruder),
        ]
        assert cal.logger.messages == []

    def test_the_logger_is_afcs_from_construction(self):
        # load_object builds AFC when this section loads first, so its logger is
        # taken here; self.afc stays None as the ready marker.
        cal = make_autocal()
        assert cal.printer.loaded == ["AFC"]
        assert cal.logger is cal.printer.afc.logger
        assert cal.afc is None
        assert cal.logger.messages == []


class TestAFCautocalHandleReady:
    def test_afc_missing_disables(self, autocal_class_patches):
        cal = make_autocal({"apply_stored_k": True})
        add_autocal_pa_handler(cal, "extruder")
        del cal.printer.objects["AFC"]
        cal._handle_ready()
        assert cal.afc is None
        assert cal._ready_time == 1000.0
        assert cal._wrapped_extruders == set()
        assert AFCLane._afc_autocal_emit_patched is False
        assert cal.logger.messages == [("warning", "AFC_autocal: AFC not loaded; disabled")]

    def test_ready_takes_afc_and_leaves_the_logger_alone(self):
        cal = make_autocal()
        before = cal.logger
        cal._handle_ready()
        assert cal.afc is cal.printer.afc
        assert cal.logger is before
        assert cal._ready_time == 1000.0
        assert cal.logger.messages == []

    @pytest.mark.parametrize("values, calibrator, spool_patched", [
        ({"apply_stored_k": True, "auto_calibrate": True}, True, True),
        ({"apply_stored_k": True}, True, False),
        ({"auto_calibrate": True}, True, True),
        ({"auto_calibrate": True}, False, False),
    ])
    def test_patches_gated_on_toggles(self, autocal_class_patches, values, calibrator,
                                      spool_patched):
        cal = make_autocal(values, calibrator=calibrator)
        cal._handle_ready()
        assert AFCLane._afc_autocal_emit_patched is True
        assert AFCSpool._afc_autocal_spoolid_patched is spool_patched
        expected = [AUTOCAL_TOOL_PATCH_LOG]
        if spool_patched:
            expected.append(AUTOCAL_SPOOL_PATCH_LOG)
        assert cal.logger.messages == expected

    def test_no_patches_when_both_toggles_off(self, autocal_class_patches):
        cal = make_autocal(calibrator=True)
        cal._handle_ready()
        assert AFCLane.set_tool_loaded is AUTOCAL_GENUINE_SET_TOOL_LOADED
        assert AFCSpool.set_spoolID is AUTOCAL_GENUINE_SET_SPOOLID
        assert AFCLane._afc_autocal_emit_patched is False
        assert AFCSpool._afc_autocal_spoolid_patched is False
        assert cal.logger.messages == []

    def test_wraps_and_preloads_in_consumer_mode(self, autocal_class_patches,
                                                 autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, spoolman=True)
        handler = add_autocal_pa_handler(cal, "extruder")
        add_autocal_lane(cal, "lane1", 5)
        autocal_spoolman.flow_k[5] = 0.04
        cal._handle_ready()
        handlers = cal.printer.gcode.mux_commands["SET_PRESSURE_ADVANCE"][1]
        assert handlers["extruder"] is not handler
        assert cal._wrapped_extruders == {"extruder"}
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert cal.logger.messages == [AUTOCAL_TOOL_PATCH_LOG]

    def test_u1_mode_skips_wrap_and_preload(self, autocal_class_patches, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, spoolman=True)
        handler = add_autocal_pa_handler(cal, "extruder")
        add_autocal_lane(cal, "lane1", 5)
        autocal_spoolman.flow_k[5] = 0.04
        cal._handle_ready()
        handlers = cal.printer.gcode.mux_commands["SET_PRESSURE_ADVANCE"][1]
        assert handlers["extruder"] is handler
        assert cal._wrapped_extruders == set()
        assert cal._lane_flow_k == {}
        assert autocal_spoolman.reads == []
        assert cal.logger.messages == [AUTOCAL_TOOL_PATCH_LOG]


class TestAFCautocalPatchSetToolLoadedEmit:
    @staticmethod
    def _install_original(monkeypatch: pytest.MonkeyPatch,
                          order: List[Tuple[Any, ...]]) -> None:
        """Stand a recording set_tool_loaded in for upstream's, before the patch wraps it."""
        def set_tool_loaded(lane: AFCLane, normal_toolchange: bool = False) -> None:
            order.append(("set_tool_loaded", lane, normal_toolchange))

        monkeypatch.setattr(AFCLane, "set_tool_loaded", set_tool_loaded)

    @staticmethod
    def _real_lane(monkeypatch: pytest.MonkeyPatch,
                   handler: Callable[[AFCLane], None]) -> AFCLane:
        """A real AFCLane whose printer runs ``handler`` on afc:tool_loaded."""
        printer = make_printer(monkeypatch=monkeypatch)
        lane = make_afc_lane("lane1", "Turtle_1", 0, printer=printer)
        printer.register_event_handler("afc:tool_loaded", handler)
        return lane

    def test_patches_and_emits(self, monkeypatch, autocal_class_patches):
        order: List[Tuple[Any, ...]] = []
        self._install_original(monkeypatch, order)
        cal = make_autocal()
        cal._patch_set_tool_loaded_emit()
        assert AFCLane._afc_autocal_emit_patched is True
        assert cal.logger.messages == [AUTOCAL_TOOL_PATCH_LOG]
        lane = self._real_lane(monkeypatch, lambda ln: order.append(("afc:tool_loaded", ln)))
        lane.set_tool_loaded(normal_toolchange=True)
        lane.set_tool_loaded()
        assert order == [("set_tool_loaded", lane, True), ("afc:tool_loaded", lane),
                         ("set_tool_loaded", lane, False), ("afc:tool_loaded", lane)]

    def test_idempotent(self, monkeypatch, autocal_class_patches):
        def sentinel(lane: AFCLane, normal_toolchange: bool = False) -> None:
            return None

        monkeypatch.setattr(AFCLane, "set_tool_loaded", sentinel)
        monkeypatch.setattr(AFCLane, "_afc_autocal_emit_patched", True)
        cal = make_autocal()
        cal._patch_set_tool_loaded_emit()
        assert AFCLane.set_tool_loaded is sentinel
        assert cal.logger.messages == []

    def test_tool_loaded_import_failure_warns(self, monkeypatch, autocal_class_patches):
        monkeypatch.setitem(sys.modules, "extras.AFC_lane", None)
        cal = make_autocal()
        cal._patch_set_tool_loaded_emit()
        assert AFCLane._afc_autocal_emit_patched is False
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: cannot patch set_tool_loaded: "
                        "import of extras.AFC_lane halted; None in sys.modules")]

    def test_tool_loaded_emit_swallows(self, monkeypatch, autocal_class_patches):
        order: List[Tuple[Any, ...]] = []
        self._install_original(monkeypatch, order)
        cal = make_autocal()
        cal._patch_set_tool_loaded_emit()

        def failing_handler(lane: AFCLane) -> None:
            order.append(("afc:tool_loaded", lane))
            raise RuntimeError("boom")

        lane = self._real_lane(monkeypatch, failing_handler)
        lane.set_tool_loaded()
        assert order == [("set_tool_loaded", lane, False), ("afc:tool_loaded", lane)]
        assert cal.logger.messages == [AUTOCAL_TOOL_PATCH_LOG]


class TestAFCautocalPatchSetSpoolidEmit:
    """
    The wrapper must survive upstream's signature, not restate it: a wrapper that
    restated an older one raised TypeError on ``on_done=``. Where the genuine
    upstream set_spoolID can run (no Spoolman, so it completes in the call) the
    tests use it; a stand-in upstream keeps the genuine parameters.
    """

    @staticmethod
    def _real_spool(monkeypatch: pytest.MonkeyPatch, events: List[Tuple[Any, ...]],
                    fail: bool = False) -> Tuple[AFCSpool, AFCLane]:
        """
        A real AFCSpool and AFCLane on a Bambu printer with no Spoolman.

        :param monkeypatch: isolates the Bambu printer's module state
        :param events: gets (event, lane, lane.spool_id) on each afc:spool_assigned
        :param fail: the event handler raises after recording
        :return Tuple[AFCSpool, AFCLane]: the spool object and the lane
        """
        printer = make_printer(monkeypatch=monkeypatch)
        lane = make_afc_lane("lane1", "Turtle_1", 0, printer=printer)
        spool = make_afc_spool(printer)

        def handler(cur_lane: AFCLane) -> None:
            events.append(("afc:spool_assigned", cur_lane, cur_lane.spool_id))
            if fail:
                raise RuntimeError("boom")

        printer.register_event_handler("afc:spool_assigned", handler)
        return spool, lane

    @staticmethod
    def _parameters(func: Callable[..., Any]) -> List[Tuple[str, Any]]:
        """:return List[Tuple[str, Any]]: each parameter's name and default"""
        return [(p.name, p.default) for p in inspect.signature(func).parameters.values()]

    def test_accepts_every_upstream_parameter(self, monkeypatch, autocal_class_patches):
        genuine = inspect.signature(AFCSpool.set_spoolID)
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        assert AFCSpool._afc_autocal_spoolid_patched is True
        assert cal.logger.messages == [AUTOCAL_SPOOL_PATCH_LOG]
        events: List[Tuple[Any, ...]] = []
        spool, lane = self._real_spool(monkeypatch, events)
        call = {"cur_lane": lane, "SpoolID": 7, "save_vars": False,
                "on_done": lambda: events.append(("on_done",))}
        # Fails when upstream grows a parameter this call does not pass.
        assert list(genuine.parameters) == ["self", *call]
        spool.set_spoolID(**call)
        assert events == [("on_done",), ("afc:spool_assigned", lane, None)]

    def test_emits_through_on_done_not_on_return(self, monkeypatch, autocal_class_patches):
        pending: List[Callable[[], None]] = []
        calls: List[Tuple[Any, ...]] = []

        def set_spoolID(spool: AFCSpool, cur_lane: AFCLane, SpoolID: Any,
                        save_vars: bool = True,
                        on_done: Optional[Callable[[], None]] = None) -> None:
            """Upstream's shape: the Spoolman fetch completes after the call returns."""
            calls.append((SpoolID, save_vars))

            def fetched() -> None:
                cur_lane.spool_id = SpoolID
                if on_done is not None:
                    on_done()

            pending.append(fetched)

        assert self._parameters(set_spoolID)[1:] == self._parameters(AFCSpool.set_spoolID)[1:]
        monkeypatch.setattr(AFCSpool, "set_spoolID", set_spoolID)
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        events: List[Tuple[Any, ...]] = []
        spool, lane = self._real_spool(monkeypatch, events)
        spool.set_spoolID(lane, 7, save_vars=False)
        assert calls == [(7, False)]
        assert events == []
        pending[0]()
        assert events == [("afc:spool_assigned", lane, 7)]
        assert cal.logger.messages == [AUTOCAL_SPOOL_PATCH_LOG]

    def test_callers_on_done_still_runs_and_runs_first(self, monkeypatch,
                                                         autocal_class_patches):
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        events: List[Tuple[Any, ...]] = []
        spool, lane = self._real_spool(monkeypatch, events)
        spool.set_spoolID(lane, 7, save_vars=False, on_done=lambda: events.append(("caller",)))
        assert events == [("caller",), ("afc:spool_assigned", lane, None)]
        assert cal.logger.messages == [AUTOCAL_SPOOL_PATCH_LOG]

    def test_event_survives_a_raising_caller_callback(self, monkeypatch,
                                                       autocal_class_patches):
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        events: List[Tuple[Any, ...]] = []
        spool, lane = self._real_spool(monkeypatch, events)

        def boom() -> None:
            raise RuntimeError("caller callback failed")

        # The caller's exception still propagates, after the event went out.
        with pytest.raises(RuntimeError, match="^caller callback failed$"):
            spool.set_spoolID(lane, 7, save_vars=False, on_done=boom)
        assert events == [("afc:spool_assigned", lane, None)]
        assert cal.logger.messages == [AUTOCAL_SPOOL_PATCH_LOG]

    def test_falls_back_to_emit_on_return_without_on_done(self, monkeypatch,
                                                          autocal_class_patches):
        calls: List[Tuple[Any, ...]] = []

        def set_spoolID(spool: AFCSpool, cur_lane: AFCLane, SpoolID: Any,
                        save_vars: bool = True) -> None:
            calls.append((cur_lane, SpoolID, save_vars))

        monkeypatch.setattr(AFCSpool, "set_spoolID", set_spoolID)
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        assert cal.logger.messages == [AUTOCAL_SPOOL_PATCH_LOG]
        events: List[Tuple[Any, ...]] = []
        spool, lane = self._real_spool(monkeypatch, events)
        spool.set_spoolID(lane, 7, save_vars=False)
        spool.set_spoolID(cur_lane=lane, SpoolID=8)
        assert calls == [(lane, 7, False), (lane, 8, True)]
        assert events == [("afc:spool_assigned", lane, None),
                          ("afc:spool_assigned", lane, None)]

    def test_unreadable_upstream_signature_emits_on_return(self, monkeypatch,
                                                            autocal_class_patches):
        calls: List[Tuple[Any, ...]] = []

        class OpaqueSetSpoolID:
            """An upstream whose signature inspect cannot read."""

            __signature__ = "unreadable"

            def __call__(self, spool: AFCSpool, cur_lane: AFCLane, SpoolID: Any) -> None:
                calls.append((cur_lane, SpoolID))

        monkeypatch.setattr(AFCSpool, "set_spoolID", OpaqueSetSpoolID())
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        events: List[Tuple[Any, ...]] = []
        spool, lane = self._real_spool(monkeypatch, events)
        spool.set_spoolID(lane, 7)
        assert calls == [(lane, 7)]
        assert events == [("afc:spool_assigned", lane, None)]
        assert cal.logger.messages == [AUTOCAL_SPOOL_PATCH_LOG]

    def test_bad_call_is_reported_by_upstream_not_the_wrapper(self, monkeypatch,
                                                              autocal_class_patches):
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        events: List[Tuple[Any, ...]] = []
        spool, lane = self._real_spool(monkeypatch, events)
        with pytest.raises(TypeError) as exc:
            spool.set_spoolID(lane, 7, nonsense=1)
        assert str(exc.value) == (
            "AFCSpool.set_spoolID() got an unexpected keyword argument 'nonsense'")
        assert events == []
        assert cal.logger.messages == [AUTOCAL_SPOOL_PATCH_LOG]

    def test_idempotent(self, monkeypatch, autocal_class_patches):
        def sentinel(spool: AFCSpool, cur_lane: AFCLane, SpoolID: Any,
                     save_vars: bool = True,
                     on_done: Optional[Callable[[], None]] = None) -> None:
            return None

        monkeypatch.setattr(AFCSpool, "set_spoolID", sentinel)
        monkeypatch.setattr(AFCSpool, "_afc_autocal_spoolid_patched", True)
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        assert AFCSpool.set_spoolID is sentinel
        assert cal.logger.messages == []

    def test_spoolid_import_failure_warns(self, monkeypatch, autocal_class_patches):
        monkeypatch.setitem(sys.modules, "extras.AFC_spool", None)
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        assert AFCSpool._afc_autocal_spoolid_patched is False
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: cannot patch set_spoolID: "
                        "import of extras.AFC_spool halted; None in sys.modules")]

    def test_spoolid_emit_swallows(self, monkeypatch, autocal_class_patches):
        cal = make_autocal()
        cal._patch_set_spoolid_emit()
        events: List[Tuple[Any, ...]] = []
        spool, lane = self._real_spool(monkeypatch, events, fail=True)
        spool.set_spoolID(lane, 5, save_vars=False)
        assert events == [("afc:spool_assigned", lane, None)]
        assert cal.logger.messages == [AUTOCAL_SPOOL_PATCH_LOG]


class TestAFCautocalSpoolman:
    def test_none_without_afc(self, autocal_spoolman):
        cal = make_autocal(spoolman=True)
        assert cal._spoolman() is None
        assert autocal_spoolman.built == []
        assert cal.logger.messages == []

    def test_none_without_moonraker(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        cal.afc.moonraker = None
        assert cal._spoolman() is None
        assert autocal_spoolman.built == []
        assert cal.logger.messages == []

    def test_none_without_spoolman(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        cal.afc.spoolman = None
        assert cal._spoolman() is None
        assert autocal_spoolman.built == []
        assert cal.logger.messages == []

    def test_builds_client(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        assert cal._spoolman() is autocal_spoolman
        assert autocal_spoolman.built == [cal.afc.moonraker]
        assert cal.logger.messages == []


class TestAFCautocalFlowCalibrator:
    def test_present_and_absent(self):
        cal = make_autocal(calibrator=True)
        assert cal._flow_calibrator() is cal.printer.objects["flow_calibrator"]
        assert cal.logger.messages == []
        bare = make_autocal()
        assert bare._flow_calibrator() is None
        assert bare.logger.messages == []


class TestAFCautocalCanCalibrate:
    def test_requires_toggle_and_calibrator(self):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True)
        assert cal._can_calibrate() is True
        assert cal.logger.messages == []

    def test_false_without_calibrator(self):
        cal = make_autocal({"auto_calibrate": True})
        assert cal._can_calibrate() is False
        assert cal.logger.messages == []

    def test_false_without_toggle(self):
        cal = make_autocal(calibrator=True)
        assert cal._can_calibrate() is False
        assert cal.logger.messages == []


class TestAFCautocalApplyGateOk:
    def test_consumer_mode_always_ok(self):
        # The lane's tool is not the active one, which U1 mode would refuse.
        cal = make_autocal(toolhead="extruder1")
        assert cal._apply_gate_ok(AutocalLane(extruder="extruder")) is True
        assert cal.logger.messages == []

    def test_u1_mode_defers_to_active_toolhead(self):
        cal = make_autocal(calibrator=True, toolhead="extruder1")
        assert cal._apply_gate_ok(AutocalLane(extruder="extruder")) is False
        assert cal._apply_gate_ok(AutocalLane(extruder="extruder1")) is True
        assert cal.logger.messages == []


class TestAFCautocalNormSpoolId:
    def test_empty_values_are_none(self):
        cal = make_autocal()
        assert [cal._norm_spool_id(sid) for sid in (None, "", 0, "0")] == [None] * 4
        assert cal.logger.messages == []

    def test_garbage_is_none(self):
        cal = make_autocal()
        assert cal._norm_spool_id("abc") is None
        assert cal._norm_spool_id(object()) is None
        assert cal.logger.messages == []

    def test_valid_ids_normalize(self):
        cal = make_autocal()
        assert cal._norm_spool_id(7) == 7
        assert cal._norm_spool_id("12") == 12
        assert cal.logger.messages == []


class TestAFCautocalSetLaneK:
    def test_caches_keyed_to_spool(self):
        cal = make_autocal()
        cal._set_lane_k(AutocalLane("lane1", "5"), 0.04)
        cal._set_lane_k(AutocalLane("lane2", None), 0.05)
        assert cal._lane_flow_k == {"lane1": (5, 0.04), "lane2": (None, 0.05)}
        assert cal.logger.messages == []


class TestAFCautocalGetLaneK:
    def test_missing_is_none(self):
        cal = make_autocal()
        assert cal._get_lane_k(AutocalLane()) is None
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == []

    def test_returns_cached_for_same_spool(self):
        cal = make_autocal()
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._get_lane_k(AutocalLane("lane1", "5")) == 0.04
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert cal.logger.messages == []

    def test_spool_change_drops_entry(self):
        cal = make_autocal()
        cal._lane_flow_k.update({"lane1": (5, 0.04), "lane2": (7, 0.05)})
        assert cal._get_lane_k(AutocalLane("lane1", 6)) is None
        assert cal._lane_flow_k == {"lane2": (7, 0.05)}
        assert cal.logger.messages == []


class TestAFCautocalReadKFromSpoolman:
    def test_none_without_spool(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        autocal_spoolman.flow_k[5] = 0.04
        assert cal._read_k_from_spoolman(AutocalLane(spool_id=None)) is None
        assert autocal_spoolman.built == []
        assert autocal_spoolman.reads == []
        assert cal.logger.messages == []

    def test_none_without_client(self, autocal_spoolman):
        cal = make_autocal(spoolman=True)
        autocal_spoolman.flow_k[5] = 0.04
        assert cal._read_k_from_spoolman(AutocalLane(spool_id=5)) is None
        assert autocal_spoolman.reads == []
        assert cal.logger.messages == []

    def test_reads_by_spool_id(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        autocal_spoolman.flow_k[5] = 0.04
        assert cal._read_k_from_spoolman(AutocalLane(spool_id="5")) == 0.04
        assert autocal_spoolman.reads == [5]
        assert cal.logger.messages == []


class TestAFCautocalWriteKToSpoolman:
    def test_noop_without_spool(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        cal._write_k_to_spoolman(AutocalLane(spool_id=None), 0.04)
        assert autocal_spoolman.built == []
        assert autocal_spoolman.writes == []
        assert cal.logger.messages == []

    def test_noop_without_client(self, autocal_spoolman):
        cal = make_autocal(spoolman=True)
        cal._write_k_to_spoolman(AutocalLane(spool_id=5), 0.04)
        assert autocal_spoolman.writes == []
        assert cal.logger.messages == []

    def test_writes_by_spool_id(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        cal._write_k_to_spoolman(AutocalLane(spool_id="5"), 0.04)
        assert autocal_spoolman.writes == [(5, 0.04)]
        assert cal.logger.messages == []


class TestAFCautocalApplyLaneK:
    def test_none_without_cache(self):
        cal = make_autocal(calibrator=True, ready=True)
        flow = cal.printer.objects["flow_calibrator"]
        assert cal._apply_lane_k("lane1") is None
        assert flow.applied == []
        assert flow._current_k == {}
        assert cal.logger.messages == []

    def test_consumer_mode_applies_on_stepper(self):
        # lane1 is not in AFC's lane table, so its extruder falls back to 'extruder'.
        cal = make_autocal(ready=True)
        stepper = add_autocal_stepper(cal, "extruder", smooth_time=0.02)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        msg = cal._apply_lane_k("lane1")
        assert msg == "AFC autocal: applied K=0.040000 for lane1 on extruder"
        assert stepper.applied == [(0.04, 0.02)]
        assert cal._managed_extruders == {"extruder"}
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_consumer_mode_falls_back_to_extruder_before_ready(self):
        cal = make_autocal()
        add_autocal_lane(cal, "lane1", 5, extruder="extruder1")
        stepper = add_autocal_stepper(cal, "extruder")
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._apply_lane_k("lane1") == (
            "AFC autocal: applied K=0.040000 for lane1 on extruder")
        assert stepper.applied == [(0.04, 0.04)]
        assert cal._managed_extruders == {"extruder"}
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_lane_without_extruder_uses_default(self):
        cal = make_autocal(ready=True)
        add_autocal_lane(cal, "lane1", 5, extruder=None)
        stepper = add_autocal_stepper(cal, "extruder")
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._apply_lane_k("lane1") == (
            "AFC autocal: applied K=0.040000 for lane1 on extruder")
        assert stepper.applied == [(0.04, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_applies_to_active_extruder(self):
        cal = make_autocal(calibrator=True, ready=True, toolhead="extruder1")
        stepper = add_autocal_stepper(cal, "extruder1")
        flow = cal.printer.objects["flow_calibrator"]
        toolhead = cal.printer.objects["toolhead"]
        cal._lane_flow_k["lane1"] = (5, 0.0425)
        msg = cal._apply_lane_k("lane1")
        assert msg == "AFC autocal: applied K=0.042500 for lane1 on extruder1"
        assert flow.applied == [(toolhead.extruder, 0.0425)]
        assert flow._current_k == {"extruder1": 0.0425}
        assert stepper.applied == []
        assert cal._managed_extruders == set()
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.042500 for lane1 on extruder1")]

    def test_applies_on_lane_own_extruder(self):
        cal = make_autocal(ready=True)
        default = add_autocal_stepper(cal, "extruder")
        own = add_autocal_stepper(cal, "extruder1", smooth_time=0.035)
        add_autocal_lane(cal, "lane1", 5, extruder="extruder1")
        cal._lane_flow_k["lane1"] = (5, 0.04)
        msg = cal._apply_lane_k("lane1")
        assert msg == "AFC autocal: applied K=0.040000 for lane1 on extruder1"
        assert own.applied == [(0.04, 0.035)]
        assert default.applied == []
        assert cal._managed_extruders == {"extruder1"}
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder1")]

    def test_prefers_config_smooth_time(self):
        cal = make_autocal(ready=True)
        stepper = add_autocal_stepper(cal, "extruder", smooth_time=0.035,
                                      config_smooth_time=0.01)
        add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.05)
        cal._apply_lane_k("lane1")
        assert stepper.applied == [(0.05, 0.01)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.050000 for lane1 on extruder")]

    def test_extruder_not_found_warns(self):
        cal = make_autocal(ready=True)
        add_autocal_lane(cal, "lane1", 5, extruder="gone")
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._apply_lane_k("lane1") is None
        assert cal._managed_extruders == set()
        assert cal.logger.messages == [("warning", "AFC autocal: extruder gone not found")]

    def test_no_stepper_warns(self):
        cal = make_autocal(ready=True)
        cal.printer.objects["extruder"] = AutocalPrinterExtruder(None)
        add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._apply_lane_k("lane1") is None
        assert cal._managed_extruders == set()
        assert cal.logger.messages == [
            ("warning", "AFC autocal: extruder extruder has no extruder_stepper")]


class TestAFCautocalWrapPaHandlers:
    def test_no_mux_is_noop(self):
        cal = make_autocal()
        cal._wrap_pa_handlers()
        assert cal._wrapped_extruders == set()
        assert cal.printer.gcode.mux_commands == {}
        assert cal.logger.messages == []

    def test_wraps_each_once(self):
        cal = make_autocal()
        first = add_autocal_pa_handler(cal, "extruder")
        second = add_autocal_pa_handler(cal, "extruder1")
        cal._wrapped_extruders.add("extruder1")
        cal._wrap_pa_handlers()
        handlers = cal.printer.gcode.mux_commands["SET_PRESSURE_ADVANCE"][1]
        wrapped = handlers["extruder"]
        assert wrapped is not first
        assert handlers["extruder1"] is second
        cal._wrap_pa_handlers()
        assert handlers["extruder"] is wrapped
        assert cal._wrapped_extruders == {"extruder", "extruder1"}
        gcmd = AutocalGcmd()
        wrapped(gcmd)
        assert first.calls == [gcmd]
        assert cal.logger.messages == []


class TestAFCautocalMakePaWrapper:
    @staticmethod
    def _call(cal: AFC_autocal, name: str = "extruder"
              ) -> Tuple[AutocalPaHandler, AutocalGcmd]:
        """Build the wrapper for ``name``, send it one gcmd; return the original and gcmd."""
        original = AutocalPaHandler()
        gcmd = AutocalGcmd()
        cal._make_pa_wrapper(original, name)(gcmd)
        return original, gcmd

    def test_blocks_slicer_pa_for_managed_extruder_while_printing(self):
        cal = make_autocal(ready=True)
        cal.afc.function.printing = True
        cal._managed_extruders.add("extruder")
        original, gcmd = self._call(cal)
        assert original.calls == []
        assert gcmd.responses == ["AFC flow K active, slicer pressure advance ignored"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: slicer PA change ignored for extruder (flow K managed)")]

    def test_passes_through_when_not_managed(self):
        cal = make_autocal(ready=True)
        cal.afc.function.printing = True
        cal._managed_extruders.add("extruder1")
        original, gcmd = self._call(cal)
        assert original.calls == [gcmd]
        assert gcmd.responses == []
        assert cal.logger.messages == []

    def test_passes_through_when_not_printing(self):
        cal = make_autocal(ready=True)
        cal._managed_extruders.add("extruder")
        original, gcmd = self._call(cal)
        assert original.calls == [gcmd]
        assert gcmd.responses == []
        assert cal.logger.messages == []


class TestAFCautocalLoadAllSpoolmanK:
    def test_apply_off_noop(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        add_autocal_lane(cal, "lane1", 5)
        autocal_spoolman.flow_k[5] = 0.04
        cal._load_all_spoolman_k()
        assert cal._lane_flow_k == {}
        assert autocal_spoolman.reads == []
        assert cal.logger.messages == []

    def test_not_ready_noop(self, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, spoolman=True)
        add_autocal_lane(cal, "lane1", 5)
        autocal_spoolman.flow_k[5] = 0.04
        cal._load_all_spoolman_k()
        assert cal._lane_flow_k == {}
        assert autocal_spoolman.reads == []
        assert cal.logger.messages == []

    def test_no_moonraker_noop(self):
        # A stale entry survives: walking the lanes would have dropped it.
        cal = make_autocal({"apply_stored_k": True}, ready=True)
        cal.afc.spoolman = object()
        add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (4, 0.09)
        cal._load_all_spoolman_k()
        assert cal._lane_flow_k == {"lane1": (4, 0.09)}
        assert cal.logger.messages == []

    def test_all_lanes_loaded(self, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, ready=True, spoolman=True)
        add_autocal_lane(cal, "a", 5)
        add_autocal_lane(cal, "b", 6)
        autocal_spoolman.flow_k.update({5: 0.04, 6: 0.05})
        cal._load_all_spoolman_k()
        assert autocal_spoolman.reads == [5, 6]
        assert cal._lane_flow_k == {"a": (5, 0.04), "b": (6, 0.05)}
        assert cal.logger.messages == []

    def test_already_cached_skipped(self, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, ready=True, spoolman=True)
        add_autocal_lane(cal, "s", 5)
        autocal_spoolman.flow_k[5] = 0.04
        cal._lane_flow_k["s"] = (5, 0.09)
        cal._load_all_spoolman_k()
        assert autocal_spoolman.reads == []
        assert cal._lane_flow_k == {"s": (5, 0.09)}
        assert cal.logger.messages == []

    def test_read_error_swallowed(self, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, ready=True, spoolman=True)
        add_autocal_lane(cal, "s", 5)
        add_autocal_lane(cal, "t", 6)
        autocal_spoolman.read_errors[5] = RuntimeError("x")
        autocal_spoolman.flow_k[6] = 0.05
        cal._load_all_spoolman_k()
        assert cal._lane_flow_k == {"t": (6, 0.05)}
        assert cal.logger.messages == [
            ("debug", "AFC autocal: startup K load failed for s: x")]

    def test_read_none_not_cached(self, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, ready=True, spoolman=True)
        add_autocal_lane(cal, "s", 5)
        cal._load_all_spoolman_k()
        assert autocal_spoolman.reads == [5]
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == []


class TestAFCautocalEnsureKLoaded:
    def test_cached_short_circuits(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        autocal_spoolman.flow_k[5] = 0.09
        cal._lane_flow_k["lane1"] = (5, 0.04)
        assert cal._ensure_k_loaded(AutocalLane("lane1", 5)) == 0.04
        assert autocal_spoolman.reads == []
        assert cal.logger.messages == []

    def test_reads_and_caches(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        autocal_spoolman.flow_k[5] = 0.05
        assert cal._ensure_k_loaded(AutocalLane("lane1", 5)) == 0.05
        assert autocal_spoolman.reads == [5]
        assert cal._lane_flow_k == {"lane1": (5, 0.05)}
        assert cal.logger.messages == []

    def test_none_when_neither(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        assert cal._ensure_k_loaded(AutocalLane("lane1", 5)) is None
        assert autocal_spoolman.reads == [5]
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == []


class TestAFCautocalCalibrate:
    def test_missing_flow_calibrator_gcmd(self):
        cal = make_autocal(ready=True)
        gcmd = AutocalGcmd()
        assert cal._calibrate(AutocalLane(), gcmd=gcmd) is None
        assert gcmd.responses == ["AFC_autocal: flow_calibrator not found"]
        assert cal.printer.gcode.command_scripts == []
        assert cal.logger.messages == []

    def test_missing_flow_calibrator_logs(self):
        cal = make_autocal(ready=True)
        assert cal._calibrate(AutocalLane()) is None
        assert cal.printer.gcode.command_scripts == []
        assert cal.logger.messages == [("warning", "AFC_autocal: flow_calibrator not found")]

    def test_no_new_k(self):
        cal = make_autocal(calibrator=True, ready=True)
        flow = cal.printer.objects["flow_calibrator"]
        flow._current_k["extruder"] = 0.03
        lane = add_autocal_lane(cal)
        assert cal._calibrate(lane) is None
        assert cal.printer.gcode.command_scripts == ["FLOW_CALIBRATE"]
        assert cal.printer.gcode.scripts == []
        assert cal._lane_flow_k == {}
        assert flow.applied == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_no_k_measured(self):
        cal = make_autocal(calibrator=True, ready=True)
        lane = add_autocal_lane(cal)
        assert cal._calibrate(lane) is None
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == [
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_k_cleared_by_calibration(self, autocal_spoolman):
        cal = make_autocal(calibrator=True, ready=True, spoolman=True)
        flow = cal.printer.objects["flow_calibrator"]
        flow._current_k["extruder"] = 0.03
        lane = add_autocal_lane(cal, "lane1", 5)

        def runner(script: str) -> None:
            flow._current_k.pop("extruder")

        assert cal._calibrate(lane, runner=runner) is None
        assert cal._lane_flow_k == {}
        assert flow.applied == []
        assert autocal_spoolman.writes == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_new_k_cached_applied_persisted(self, autocal_spoolman):
        cal = make_autocal(calibrator=True, ready=True, spoolman=True)
        flow = cal.printer.objects["flow_calibrator"]
        flow._current_k["extruder"] = 0.03
        lane = add_autocal_lane(cal, "lane1", 5)
        runs: List[str] = []

        def runner(script: str) -> None:
            runs.append(script)
            flow._current_k["extruder"] = 0.07

        assert cal._calibrate(lane, runner=runner) == 0.07
        assert runs == ["FLOW_CALIBRATE"]
        assert cal.printer.gcode.command_scripts == []
        assert cal._lane_flow_k == {"lane1": (5, 0.07)}
        assert flow.applied == [(cal.printer.objects["toolhead"].extruder, 0.07)]
        assert autocal_spoolman.writes == [(5, 0.07)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.070000 for lane1 on extruder"),
            ("info", "AFC autocal: calibrated and stored K=0.070000 for lane1")]

    def test_uses_th_extruder_name_for_k_key(self):
        cal = make_autocal(calibrator=True, ready=True, toolhead="extruder2")
        flow = cal.printer.objects["flow_calibrator"]
        flow._current_k.update({"extruder2": 0.03, "e2": 0.01})
        lane = add_autocal_lane(cal, "lane1", 5, extruder="e2", th_extruder_name="extruder2")
        cal.printer.gcode.on_script = lambda script: flow._current_k.update(extruder2=0.06)
        assert cal._calibrate(lane) == 0.06
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.060000 for lane1 on extruder2"),
            ("info", "AFC autocal: calibrated and stored K=0.060000 for lane1")]

    def test_falls_back_to_section_name_without_klipper_name(self):
        cal = make_autocal(calibrator=True, ready=True)
        flow = cal.printer.objects["flow_calibrator"]
        lane = add_autocal_lane(cal, "lane1", 5)
        lane.extruder_obj.th_extruder_name = None
        cal.printer.gcode.on_script = lambda script: flow._current_k.update(extruder=0.06)
        assert cal._calibrate(lane) == 0.06
        assert cal._lane_flow_k == {"lane1": (5, 0.06)}
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.060000 for lane1 on extruder"),
            ("info", "AFC autocal: calibrated and stored K=0.060000 for lane1")]


class TestAFCautocalCurrentLane:
    def test_none_without_afc(self):
        cal = make_autocal()
        cal.printer.afc.function.current_lane = AutocalLane()
        assert cal._current_lane() is None
        assert cal.logger.messages == []

    def test_none_on_exception(self):
        cal = make_autocal(ready=True)
        cal.afc.function.error = RuntimeError("no extruder")
        assert cal._current_lane() is None
        assert cal.logger.messages == []

    def test_returns_current(self):
        cal = make_autocal(ready=True)
        lane = AutocalLane()
        cal.afc.function.current_lane = lane
        assert cal._current_lane() is lane
        assert cal.logger.messages == []


class TestAFCautocalHandleToolLoaded:
    def test_gated_on_afc_lane_and_toggles(self):
        cal = make_autocal({"apply_stored_k": True})
        lane = add_autocal_lane(cal)
        cal._handle_tool_loaded(lane)
        assert cal.printer.reactor.callbacks == []
        cal.afc = cal.printer.afc
        cal._handle_tool_loaded(None)
        assert cal.printer.reactor.callbacks == []
        cal.apply_stored_k = False
        cal._handle_tool_loaded(lane)
        assert cal.printer.reactor.callbacks == []
        assert cal.logger.messages == []

    def test_defers_to_reactor(self):
        cal = make_autocal({"apply_stored_k": True}, ready=True)
        stepper = add_autocal_stepper(cal, "extruder")
        lane = add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        cal._handle_tool_loaded(lane)
        assert cal.printer.reactor.waketimes() == [0.0]
        assert stepper.applied == []
        cal.printer.reactor.run_pending()
        assert stepper.applied == [(0.04, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_auto_calibrate_alone_defers(self):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True)
        cal._handle_tool_loaded(add_autocal_lane(cal))
        assert cal.printer.reactor.waketimes() == [0.0]
        assert cal.logger.messages == []


class TestAFCautocalDoToolLoaded:
    def test_cached_k_applied_when_active(self, autocal_threads):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, ready=True)
        flow = cal.printer.objects["flow_calibrator"]
        lane = add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        cal._do_tool_loaded(lane)
        assert flow.applied == [(cal.printer.objects["toolhead"].extruder, 0.04)]
        assert autocal_threads.started == []
        assert cal._k_fetch_inflight == set()
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_cached_k_not_applied_when_inactive(self, autocal_threads):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, ready=True,
                           toolhead="extruder1")
        flow = cal.printer.objects["flow_calibrator"]
        lane = add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        cal._do_tool_loaded(lane)
        assert flow.applied == []
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_cached_k_not_applied_without_apply_toggle(self, autocal_threads):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True)
        flow = cal.printer.objects["flow_calibrator"]
        lane = add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        cal._do_tool_loaded(lane)
        assert flow.applied == []
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_uncached_fetches_async(self, autocal_threads):
        cal = make_autocal({"apply_stored_k": True}, ready=True)
        cal._do_tool_loaded(add_autocal_lane(cal, "lane1", 5))
        assert autocal_threads.started == [("afc_autocal_k", True)]
        assert cal._k_fetch_inflight == {"lane1"}
        assert cal.logger.messages == []

    def test_auto_calibrate_alone_passes_the_gate(self, autocal_threads):
        # Uncached with only auto_calibrate: no read, straight to an idle-gated calibration.
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True)
        cal._do_tool_loaded(add_autocal_lane(cal, "lane1", 5, tool_loaded=True))
        assert autocal_threads.started == []
        assert cal.printer.gcode.scripts == ["FLOW_CALIBRATE"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: running flow calibration for lane1"),
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_error_logged(self, autocal_threads):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, ready=True)
        cal.printer.objects["flow_calibrator"].error = RuntimeError("x")
        lane = add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        cal._do_tool_loaded(lane)
        assert cal.logger.messages == [("warning", "AFC_autocal: tool_loaded error: x")]

    def test_no_afc_returns(self, autocal_threads):
        cal = make_autocal({"apply_stored_k": True})
        cal._do_tool_loaded(add_autocal_lane(cal, "lane1", 5))
        assert autocal_threads.started == []
        assert cal._k_fetch_inflight == set()
        assert cal.logger.messages == []

    def test_no_lane_returns(self, autocal_threads):
        cal = make_autocal({"apply_stored_k": True}, ready=True)
        cal._do_tool_loaded(None)
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_disabled_returns(self, autocal_threads):
        cal = make_autocal(ready=True)
        cal._do_tool_loaded(add_autocal_lane(cal, "lane1", 5))
        assert autocal_threads.started == []
        assert cal._k_fetch_inflight == set()
        assert cal.logger.messages == []


class TestAFCautocalHandleSpoolAssigned:
    def test_gated(self):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True)
        lane = add_autocal_lane(cal)
        cal._handle_spool_assigned(lane)
        assert cal.printer.reactor.callbacks == []
        cal.afc = cal.printer.afc
        cal._handle_spool_assigned(None)
        assert cal.printer.reactor.callbacks == []
        cal.auto_calibrate = False
        cal._handle_spool_assigned(lane)
        assert cal.printer.reactor.callbacks == []
        assert cal.logger.messages == []

    def test_defers_to_reactor(self):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True)
        lane = add_autocal_lane(cal, "lane1", None)
        cal._staged_handled["lane1"] = 5
        cal._handle_spool_assigned(lane)
        assert cal.printer.reactor.waketimes() == [0.0]
        assert cal._staged_handled == {"lane1": 5}
        cal.printer.reactor.run_pending()
        assert cal._staged_handled == {}
        assert cal.logger.messages == []


class TestAFCautocalDoSpoolAssigned:
    @staticmethod
    def _staged(spool_id: Any = 5, **lane_kw: Any) -> Tuple[AFC_autocal, AutocalLane]:
        """A ready U1 autocal with auto_calibrate on and one lane, past the grace."""
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True)
        return cal, add_autocal_lane(cal, "lane1", spool_id, **lane_kw)

    def test_spool_cleared_resets_tracking(self, autocal_threads):
        cal, lane = self._staged(None)
        cal._staged_handled["lane1"] = 5
        cal._staged_pending.add("lane1")
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {}
        assert cal._staged_pending == set()
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_tool_loaded_skips(self, autocal_threads):
        cal, lane = self._staged(tool_loaded=True)
        cal._staged_pending.add("lane1")
        cal._do_spool_assigned(lane)
        assert cal._staged_pending == set()
        assert cal._staged_handled == {}
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_no_filament_skips(self, autocal_threads):
        cal, lane = self._staged(load_state=False)
        cal._staged_pending.add("lane1")
        cal._do_spool_assigned(lane)
        assert cal._staged_pending == set()
        assert cal._staged_handled == {}
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_already_handled_spool_skips(self, autocal_threads):
        cal, lane = self._staged()
        cal._staged_handled["lane1"] = 5
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {"lane1": 5}
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_pending_retry_dedupes_new_events(self, autocal_threads):
        cal, lane = self._staged()
        cal._staged_pending.add("lane1")
        cal._do_spool_assigned(lane, attempts=0)
        assert cal._staged_pending == {"lane1"}
        assert cal._staged_handled == {}
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_retry_attempt_proceeds_despite_pending(self, autocal_threads):
        cal, lane = self._staged()
        cal._staged_pending.add("lane1")
        cal._do_spool_assigned(lane, attempts=1)
        assert cal._staged_pending == set()
        assert cal._staged_handled == {"lane1": 5}
        assert autocal_threads.started == [("afc_autocal_stg", True)]
        assert cal.logger.messages == []

    def test_startup_grace_skips(self, autocal_threads):
        cal, lane = self._staged()
        cal._ready_time = 999.0
        cal._staged_pending.add("lane1")
        cal._do_spool_assigned(lane, attempts=1)
        assert cal._staged_pending == set()
        assert cal._staged_handled == {}
        assert cal.printer.reactor.callbacks == []
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_no_ready_stamp_skips_the_grace_check(self, autocal_threads):
        cal, lane = self._staged()
        cal._ready_time = None
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {"lane1": 5}
        assert autocal_threads.started == [("afc_autocal_stg", True)]
        assert cal.logger.messages == []

    def test_not_settled_retries_bounded(self, autocal_threads):
        cal, lane = self._staged()
        cal.afc.current_state = State.LOADING
        reactor = cal.printer.reactor
        cal._do_spool_assigned(lane)
        assert cal._staged_pending == {"lane1"}
        assert reactor.waketimes() == [1001.0]
        reactor.callbacks.clear()
        cal._do_spool_assigned(lane, attempts=29)
        assert reactor.waketimes() == [1001.0]
        assert cal._staged_pending == {"lane1"}
        reactor.callbacks.clear()
        cal._do_spool_assigned(lane, attempts=30)
        assert reactor.callbacks == []
        assert cal._staged_pending == set()
        assert cal._staged_handled == {}
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_retry_callback_resumes_once_idle(self, autocal_threads):
        # The retry carries attempts + 1, so the pending-lane dedupe lets it through.
        cal, lane = self._staged()
        cal.afc.current_state = State.LOADING
        cal._do_spool_assigned(lane)
        cal.afc.current_state = State.IDLE
        cal.printer.reactor.run_pending()
        assert cal._staged_pending == set()
        assert cal._staged_handled == {"lane1": 5}
        assert autocal_threads.started == [("afc_autocal_stg", True)]
        assert cal.logger.messages == []

    def test_load_in_flight_retries(self, autocal_threads):
        cal, lane = self._staged(load_active=True)
        cal._do_spool_assigned(lane)
        assert cal._staged_pending == {"lane1"}
        assert cal.printer.reactor.waketimes() == [1001.0]
        assert autocal_threads.started == []
        assert cal.logger.messages == []

    def test_happy_path_records_and_checks_k(self, autocal_threads):
        cal, lane = self._staged()
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {"lane1": 5}
        assert cal._staged_pending == set()
        assert autocal_threads.started == [("afc_autocal_stg", True)]
        assert cal.logger.messages == []

    def test_error_logged(self, autocal_threads):
        cal, lane = self._staged()
        autocal_threads.error = RuntimeError("y")
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {"lane1": 5}
        assert cal.logger.messages == [("warning", "AFC_autocal: spool_assigned error: y")]

    def test_cannot_calibrate_returns(self):
        cal = make_autocal({"auto_calibrate": True}, ready=True)
        lane = add_autocal_lane(cal, "lane1", None)
        cal._staged_handled["lane1"] = 5
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {"lane1": 5}
        assert cal.logger.messages == []

    def test_not_ready_returns(self):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True)
        lane = add_autocal_lane(cal, "lane1", None)
        cal._staged_handled["lane1"] = 5
        cal._do_spool_assigned(lane)
        assert cal._staged_handled == {"lane1": 5}
        assert cal.logger.messages == []

    def test_no_lane_returns(self):
        cal, _ = self._staged()
        cal._do_spool_assigned(None)
        assert cal.logger.messages == []


class TestAFCautocalCheckStagedKAsync:
    @staticmethod
    def _staged(spoolman: bool = True) -> AFC_autocal:
        """A ready U1 autocal with staged lane1 holding spool 5."""
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True,
                           spoolman=spoolman)
        add_autocal_lane(cal, "lane1", 5)
        return cal

    def test_reads_off_thread_then_hops_to_reactor(self, autocal_threads, autocal_spoolman):
        cal = self._staged()
        autocal_spoolman.flow_k[5] = 0.04
        cal._check_staged_k_async("lane1", 5)
        assert autocal_threads.started == [("afc_autocal_stg", True)]
        assert autocal_spoolman.reads == []
        autocal_threads.run_all()
        assert autocal_spoolman.reads == [5]
        assert cal._lane_flow_k == {}
        cal.printer.reactor.run_pending()
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 already has K=0.040000, not auto-loading")]

    def test_successful_read_without_k_reports_ok(self, autocal_threads,
                                                  autocal_spoolman):
        cal = self._staged()
        cal._check_staged_k_async("lane1", 5)
        autocal_threads.run_all()
        cal.printer.reactor.run_pending()
        assert autocal_spoolman.reads == [5]
        assert cal._lane_flow_k == {}
        assert cal.printer.gcode.scripts == ["CHANGE_TOOL LANE=lane1"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 has no stored K, loading to calibrate")]

    def test_read_failure_reports_not_ok(self, autocal_threads, autocal_spoolman):
        cal = self._staged()
        autocal_spoolman.read_errors[5] = RuntimeError("down")
        cal._check_staged_k_async("lane1", 5)
        autocal_threads.run_all()
        cal.printer.reactor.run_pending()
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("debug", "AFC_autocal: staged K read failed: down"),
            ("info", "AFC autocal: lane1 spool 5 K unknown (Spoolman read failed), "
                     "not auto-loading")]

    def test_no_client_reports_none(self, autocal_threads, autocal_spoolman):
        cal = self._staged(spoolman=False)
        cal._check_staged_k_async("lane1", 5)
        autocal_threads.run_all()
        cal.printer.reactor.run_pending()
        assert autocal_spoolman.built == []
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 K unknown (Spoolman read failed), "
                     "not auto-loading")]

    def test_thread_naming_failure_is_ignored(self, monkeypatch, autocal_threads,
                                              autocal_spoolman):
        def get_ffi() -> None:
            raise OSError("no chelper")

        monkeypatch.setattr(autocal_mod, "chelper", types.SimpleNamespace(get_ffi=get_ffi))
        cal = self._staged()
        autocal_spoolman.flow_k[5] = 0.04
        cal._check_staged_k_async("lane1", 5)
        autocal_threads.run_all()
        cal.printer.reactor.run_pending()
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 already has K=0.040000, not auto-loading")]


class TestAFCautocalStagedKReady:
    @staticmethod
    def _staged(**lane_kw: Any) -> AFC_autocal:
        """A ready U1 autocal with auto_calibrate on; lane1 only when lane_kw is given."""
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True)
        if lane_kw:
            add_autocal_lane(cal, "lane1", **lane_kw)
        return cal

    def test_lane_gone(self):
        cal = self._staged()
        cal._staged_k_ready("lane1", 5, 0.04, True)
        assert cal._lane_flow_k == {}
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == []

    def test_not_ready(self):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True)
        add_autocal_lane(cal, "lane1", 5)
        cal._staged_k_ready("lane1", 5, 0.04, True)
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == []

    def test_spool_changed(self):
        cal = self._staged(spool_id=6)
        cal._staged_k_ready("lane1", 5, 0.04, True)
        assert cal._lane_flow_k == {}
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == []

    def test_loaded_meanwhile(self):
        cal = self._staged(spool_id=5, tool_loaded=True)
        cal._staged_k_ready("lane1", 5, 0.04, True)
        assert cal._lane_flow_k == {}
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == []

    def test_existing_k_cached_no_load(self):
        cal = self._staged(spool_id=5)
        cal._staged_k_ready("lane1", 5, 0.0412, True)
        assert cal._lane_flow_k == {"lane1": (5, 0.0412)}
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 already has K=0.041200, not auto-loading")]

    def test_unreadable_k_no_load(self):
        cal = self._staged(spool_id=5)
        cal._staged_k_ready("lane1", 5, None, False)
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 K unknown (Spoolman read failed), "
                     "not auto-loading")]

    def test_unsafe_no_load(self):
        cal = self._staged(spool_id=5)
        cal.afc.current_state = State.LOADING
        cal._staged_k_ready("lane1", 5, None, True)
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == []

    def test_no_k_auto_loads(self):
        cal = self._staged(spool_id=5)
        cal._staged_k_ready("lane1", 5, None, True)
        assert cal.printer.gcode.scripts == ["CHANGE_TOOL LANE=lane1"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 has no stored K, loading to calibrate")]

    def test_error_logged(self):
        cal = self._staged(spool_id=5)

        def refuse(script: str) -> None:
            raise RuntimeError("boom")

        cal.printer.gcode.on_script = refuse
        cal._staged_k_ready("lane1", 5, None, True)
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 spool 5 has no stored K, loading to calibrate"),
            ("warning", "AFC_autocal: staged load error: boom")]


class TestAFCautocalFetchKAsync:
    def test_inflight_dedupes(self, autocal_threads):
        cal = make_autocal({"apply_stored_k": True}, ready=True)
        cal._k_fetch_inflight.add("lane1")
        cal._fetch_k_async(add_autocal_lane(cal, "lane1", 5))
        assert autocal_threads.started == []
        assert cal._k_fetch_inflight == {"lane1"}
        assert cal.logger.messages == []

    def test_no_spool_calibrates_when_enabled(self, autocal_threads):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True)
        cal._fetch_k_async(add_autocal_lane(cal, "lane1", None, tool_loaded=True))
        assert autocal_threads.started == []
        assert cal.printer.gcode.scripts == ["FLOW_CALIBRATE"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: running flow calibration for lane1"),
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_no_spool_no_calibrate_when_disabled(self, autocal_threads):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, ready=True)
        cal._fetch_k_async(add_autocal_lane(cal, "lane1", None, tool_loaded=True))
        assert autocal_threads.started == []
        assert cal._k_fetch_inflight == set()
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == []

    def test_apply_off_goes_straight_to_k_applied(self, autocal_threads, autocal_spoolman):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True,
                           spoolman=True)
        autocal_spoolman.flow_k[5] = 0.04
        cal._fetch_k_async(add_autocal_lane(cal, "lane1", 5, tool_loaded=True))
        assert autocal_threads.started == []
        assert autocal_spoolman.reads == []
        assert cal.printer.gcode.scripts == ["FLOW_CALIBRATE"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: running flow calibration for lane1"),
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_reads_then_applies_on_reactor(self, autocal_threads, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, ready=True, spoolman=True)
        stepper = add_autocal_stepper(cal, "extruder")
        autocal_spoolman.flow_k[5] = 0.04
        cal._fetch_k_async(add_autocal_lane(cal, "lane1", 5))
        assert cal._k_fetch_inflight == {"lane1"}
        assert autocal_threads.started == [("afc_autocal_k", True)]
        assert autocal_spoolman.reads == []
        autocal_threads.run_all()
        assert autocal_spoolman.reads == [5]
        assert stepper.applied == []
        cal.printer.reactor.run_pending()
        assert cal._k_fetch_inflight == set()
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert stepper.applied == [(0.04, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_worker_failure_still_hops_back(self, autocal_threads, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, ready=True, spoolman=True)
        autocal_spoolman.read_errors[5] = RuntimeError("down")
        cal._fetch_k_async(add_autocal_lane(cal, "lane1", 5))
        autocal_threads.run_all()
        assert cal._k_fetch_inflight == {"lane1"}
        cal.printer.reactor.run_pending()
        assert cal._k_fetch_inflight == set()
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == [("debug", "AFC_autocal: async K read failed: down")]

    def test_no_client_applies_none(self, autocal_threads, autocal_spoolman):
        cal = make_autocal({"apply_stored_k": True}, ready=True)
        cal._fetch_k_async(add_autocal_lane(cal, "lane1", 5))
        autocal_threads.run_all()
        cal.printer.reactor.run_pending()
        assert autocal_spoolman.built == []
        assert cal._k_fetch_inflight == set()
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == []

    def test_thread_naming_failure_is_ignored(self, monkeypatch, autocal_threads,
                                              autocal_spoolman):
        def get_ffi() -> None:
            raise OSError("no chelper")

        monkeypatch.setattr(autocal_mod, "chelper", types.SimpleNamespace(get_ffi=get_ffi))
        cal = make_autocal({"apply_stored_k": True}, ready=True, spoolman=True)
        stepper = add_autocal_stepper(cal, "extruder")
        autocal_spoolman.flow_k[5] = 0.04
        cal._fetch_k_async(add_autocal_lane(cal, "lane1", 5))
        autocal_threads.run_all()
        cal.printer.reactor.run_pending()
        assert autocal_spoolman.reads == [5]
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert stepper.applied == [(0.04, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]


class TestAFCautocalKApplied:
    def test_clears_inflight_even_when_lane_gone(self):
        cal = make_autocal({"apply_stored_k": True}, ready=True)
        cal._k_fetch_inflight.add("lane1")
        cal._k_applied("lane1", 5, 0.04)
        assert cal._k_fetch_inflight == set()
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == []

    def test_not_ready_returns(self):
        cal = make_autocal({"apply_stored_k": True})
        add_autocal_lane(cal, "lane1", 5)
        cal._k_fetch_inflight.add("lane1")
        cal._k_applied("lane1", 5, 0.04)
        assert cal._k_fetch_inflight == set()
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == []

    def test_stale_spool_ignored(self):
        cal = make_autocal({"apply_stored_k": True}, ready=True)
        add_autocal_lane(cal, "lane1", 6)
        cal._k_applied("lane1", 5, 0.04)
        assert cal._lane_flow_k == {}
        assert cal.logger.messages == []

    def test_k_cached_and_applied_when_active(self):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, ready=True)
        flow = cal.printer.objects["flow_calibrator"]
        add_autocal_lane(cal, "lane1", 5)
        cal._k_applied("lane1", 5, 0.04)
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert flow.applied == [(cal.printer.objects["toolhead"].extruder, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_k_cached_not_applied_when_inactive(self):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, ready=True,
                           toolhead="extruder1")
        flow = cal.printer.objects["flow_calibrator"]
        add_autocal_lane(cal, "lane1", 5)
        cal._k_applied("lane1", 5, 0.04)
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert flow.applied == []
        assert cal.logger.messages == []

    def test_k_cached_not_applied_without_apply_toggle(self):
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True)
        flow = cal.printer.objects["flow_calibrator"]
        add_autocal_lane(cal, "lane1", 5)
        cal._k_applied("lane1", 5, 0.04)
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert flow.applied == []
        assert cal.logger.messages == []

    def test_no_k_calibrates_when_enabled(self):
        cal = make_autocal({"apply_stored_k": True, "auto_calibrate": True},
                           calibrator=True, ready=True)
        add_autocal_lane(cal, "lane1", 5, tool_loaded=True)
        cal._k_applied("lane1", 5, None)
        assert cal.printer.gcode.scripts == ["FLOW_CALIBRATE"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: running flow calibration for lane1"),
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_no_k_no_calibrate_when_disabled(self):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, ready=True)
        add_autocal_lane(cal, "lane1", 5, tool_loaded=True)
        cal._k_applied("lane1", 5, None)
        assert cal.printer.gcode.scripts == []
        assert cal._cal_pending == set()
        assert cal.logger.messages == []

    def test_error_logged(self):
        cal = make_autocal({"apply_stored_k": True}, calibrator=True, ready=True)
        cal.printer.objects["flow_calibrator"].error = RuntimeError("z")
        add_autocal_lane(cal, "lane1", 5)
        cal._k_applied("lane1", 5, 0.04)
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert cal.logger.messages == [("warning", "AFC_autocal: K apply error: z")]


class TestAFCautocalExtruderLoadInFlight:
    def test_no_extruder_obj(self):
        cal = make_autocal()
        assert cal._extruder_load_in_flight(AutocalLane(extruder=None)) is False
        assert cal.logger.messages == []

    def test_load_active_states(self):
        cal = make_autocal()
        assert cal._extruder_load_in_flight(AutocalLane(load_active=True)) is True
        assert cal._extruder_load_in_flight(AutocalLane(load_active=False)) is False
        assert cal.logger.messages == []


class TestAFCautocalIsPrinting:
    def test_states(self):
        cal = make_autocal(ready=True)
        cal.afc.function.printing = True
        assert cal._is_printing() is True
        cal.afc.function.printing = False
        assert cal._is_printing() is False
        assert cal.logger.messages == []

    def test_exception_is_false(self):
        cal = make_autocal()
        cal.printer.afc.function.printing = True
        assert cal._is_printing() is False
        assert cal.logger.messages == []


class TestAFCautocalCalibrateWhenLoaded:
    @staticmethod
    def _loaded(toolhead: Optional[str] = "extruder", **lane_kw: Any
                ) -> Tuple[AFC_autocal, AutocalLane]:
        """A ready, idle U1 autocal with auto_calibrate on and lane1 in the toolhead."""
        cal = make_autocal({"auto_calibrate": True}, calibrator=True, ready=True,
                           toolhead=toolhead)
        lane_kw.setdefault("tool_loaded", True)
        return cal, add_autocal_lane(cal, "lane1", 5, **lane_kw)

    def test_dedupes_running_chain(self):
        cal, lane = self._loaded()
        cal._cal_pending.add("lane1")
        cal._calibrate_when_loaded(lane, attempts=0)
        assert cal._cal_pending == {"lane1"}
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == []

    def test_printing_skips(self):
        cal, lane = self._loaded()
        cal.afc.function.printing = True
        cal._calibrate_when_loaded(lane)
        assert cal._cal_pending == set()
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration skipped, printing")]

    def test_prep_not_done_hard_skips(self):
        cal, lane = self._loaded()
        cal.afc.prep_done = False
        cal._calibrate_when_loaded(lane)
        assert cal._cal_pending == set()
        assert cal.printer.reactor.callbacks == []
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration skipped, prep not done")]

    def test_startup_grace_hard_skips(self):
        cal, lane = self._loaded()
        cal._ready_time = 999.0
        cal._calibrate_when_loaded(lane)
        assert cal._cal_pending == set()
        assert cal.printer.reactor.callbacks == []
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration skipped, within startup grace")]

    def test_transient_state_retries_then_gives_up(self):
        cal, lane = self._loaded()
        cal.afc.current_state = State.LOADING
        reactor = cal.printer.reactor
        cal._calibrate_when_loaded(lane, attempts=0)
        assert cal._cal_pending == {"lane1"}
        assert reactor.waketimes() == [1001.0]
        reactor.callbacks.clear()
        cal._calibrate_when_loaded(lane, attempts=239)
        assert reactor.waketimes() == [1001.0]
        reactor.callbacks.clear()
        cal._calibrate_when_loaded(lane, attempts=240)
        assert reactor.callbacks == []
        assert cal._cal_pending == set()
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration gave up waiting to settle "
                     "(state=Loading (not idle))")]

    def test_retry_callback_calibrates_once_idle(self):
        cal, lane = self._loaded()
        cal.afc.current_state = State.LOADING
        cal._calibrate_when_loaded(lane)
        cal.afc.current_state = State.IDLE
        cal.printer.reactor.run_pending()
        assert cal._cal_pending == set()
        assert cal.printer.gcode.scripts == ["FLOW_CALIBRATE"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: running flow calibration for lane1"),
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_load_in_flight_waits(self):
        cal, lane = self._loaded(load_active=True)
        cal._calibrate_when_loaded(lane, attempts=0)
        assert cal.printer.reactor.waketimes() == [1001.0]
        assert cal._cal_pending == {"lane1"}
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == []

    def test_load_in_flight_gives_up(self):
        cal, lane = self._loaded(load_active=True)
        cal._calibrate_when_loaded(lane, attempts=240)
        assert cal.printer.reactor.callbacks == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration gave up waiting to settle "
                     "(load in flight)")]

    def test_unloaded_while_waiting_skips(self):
        cal, lane = self._loaded(tool_loaded=False)
        cal._calibrate_when_loaded(lane)
        assert cal._cal_pending == set()
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == []

    def test_off_toolhead_skips_with_message(self):
        cal, lane = self._loaded(toolhead="extruder2")
        cal._calibrate_when_loaded(lane)
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration skipped, its tool is not on the "
                     "toolhead (active extruder=extruder2); pick up/load this lane's "
                     "tool to calibrate it")]
        assert cal._cal_pending == set()

    def test_off_toolhead_active_unknown(self):
        cal, lane = self._loaded()
        cal.printer.objects["toolhead"] = AutocalToolhead(error=RuntimeError("no toolhead"))
        cal._calibrate_when_loaded(lane)
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("info", "AFC autocal: lane1 calibration skipped, its tool is not on the "
                     "toolhead (active extruder=?); pick up/load this lane's tool to "
                     "calibrate it")]
        assert cal._cal_pending == set()

    def test_happy_runs_with_run_script_runner(self):
        cal, lane = self._loaded()
        cal._calibrate_when_loaded(lane)
        assert cal.printer.gcode.scripts == ["FLOW_CALIBRATE"]
        assert cal.printer.gcode.command_scripts == []
        assert cal._cal_pending == set()
        assert cal.logger.messages == [
            ("info", "AFC autocal: running flow calibration for lane1"),
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_error_clears_pending_and_logs(self):
        cal, lane = self._loaded()

        def refuse(script: str) -> None:
            raise RuntimeError("q")

        cal.printer.gcode.on_script = refuse
        cal._calibrate_when_loaded(lane)
        assert cal._cal_pending == set()
        assert cal.logger.messages == [
            ("info", "AFC autocal: running flow calibration for lane1"),
            ("warning", "AFC_autocal: deferred calibrate error: q")]


    def test_error_while_pending_clears_pending(self):
        cal, lane = self._loaded()
        cal.afc.current_state = State.LOADING

        def refuse(callback: Callable[[float], Any], waketime: float = 0.0) -> None:
            raise RuntimeError("q")

        cal.printer.reactor.register_callback = refuse
        cal._calibrate_when_loaded(lane)
        assert cal._cal_pending == set()
        assert cal.printer.gcode.scripts == []
        assert cal.logger.messages == [
            ("warning", "AFC_autocal: deferred calibrate error: q")]


class TestAFCautocalLaneOnActiveToolhead:
    def test_no_extruder_obj(self):
        cal = make_autocal()
        assert cal._lane_on_active_toolhead(AutocalLane(extruder=None)) is False
        assert cal.logger.messages == []

    def test_matches_section_name(self):
        cal = make_autocal(toolhead="e1")
        lane = AutocalLane(extruder="e1", th_extruder_name="extruder1")
        assert cal._lane_on_active_toolhead(lane) is True
        assert cal.logger.messages == []

    def test_matches_klipper_name(self):
        cal = make_autocal(toolhead="extruder1")
        lane = AutocalLane(extruder="e1", th_extruder_name="extruder1")
        assert cal._lane_on_active_toolhead(lane) is True
        assert cal.logger.messages == []

    def test_mismatch(self):
        cal = make_autocal(toolhead="extruder2")
        lane = AutocalLane(extruder="e1", th_extruder_name="extruder1")
        assert cal._lane_on_active_toolhead(lane) is False
        assert cal.logger.messages == []

    def test_lookup_failure_is_false(self):
        cal = make_autocal(toolhead=None)
        assert cal._lane_on_active_toolhead(AutocalLane()) is False
        assert cal.logger.messages == []

    def test_active_extruder_none(self):
        cal = make_autocal()
        cal.printer.objects["toolhead"] = AutocalToolhead(None)
        assert cal._lane_on_active_toolhead(AutocalLane()) is False
        assert cal.logger.messages == []


class TestAFCautocalSafeToCalibrate:
    def test_mirrors_block_reason(self):
        cal = make_autocal(ready=True)
        assert cal._safe_to_calibrate() is True
        cal.afc.prep_done = False
        assert cal._safe_to_calibrate() is False
        assert cal.logger.messages == []


class TestAFCautocalCalBlockReason:
    def test_prep_not_done(self):
        cal = make_autocal(ready=True)
        cal.afc.prep_done = False
        assert cal._cal_block_reason() == "prep not done"
        assert cal.logger.messages == []

    def test_within_startup_grace(self):
        cal = make_autocal(ready=True)
        cal._ready_time = 999.0
        assert cal._cal_block_reason() == "within startup grace"
        assert cal.logger.messages == []

    def test_not_idle(self):
        cal = make_autocal(ready=True)
        cal.afc.current_state = State.LOADING
        assert cal._cal_block_reason() == "state=Loading (not idle)"
        cal.afc.current_state = "State.LOADING"
        assert cal._cal_block_reason() == "state=LOADING (not idle)"
        assert cal.logger.messages == []

    def test_idle_is_clear(self):
        cal = make_autocal(ready=True)
        assert cal._cal_block_reason() is None
        cal.afc.current_state = "State.IDLE"
        assert cal._cal_block_reason() is None
        assert cal.logger.messages == []

    def test_no_ready_stamp_or_state_is_clear(self):
        cal = make_autocal(ready=True)
        cal._ready_time = None
        cal.afc.current_state = None
        assert cal._cal_block_reason() is None
        assert cal.logger.messages == []


class TestAFCautocalReapplyCurrentK:
    @staticmethod
    def _cached_current(values: Optional[Dict[str, Any]] = None
                        ) -> Tuple[AFC_autocal, AutocalExtruderStepper]:
        """A ready consumer autocal whose current lane1 has a cached K of 0.04."""
        cal = make_autocal(values, ready=True)
        stepper = add_autocal_stepper(cal, "extruder")
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        return cal, stepper

    def test_gated_off_when_not_applying(self):
        cal, stepper = self._cached_current()
        cal._reapply_current_k()
        assert stepper.applied == []
        assert cal.logger.messages == []

    def test_gated_on_prep_and_idle(self):
        cal, stepper = self._cached_current({"apply_stored_k": True})
        cal.afc.prep_done = False
        cal._reapply_current_k()
        assert stepper.applied == []
        cal.afc.prep_done = True
        cal.afc.current_state = State.LOADING
        cal._reapply_current_k()
        assert stepper.applied == []
        assert cal.logger.messages == []

    def test_no_current_or_uncached_is_noop(self):
        cal, stepper = self._cached_current({"apply_stored_k": True})
        lane = cal.afc.function.current_lane
        cal.afc.function.current_lane = None
        cal._reapply_current_k()
        assert stepper.applied == []
        cal.afc.function.current_lane = lane
        cal._lane_flow_k.clear()
        cal._reapply_current_k()
        assert stepper.applied == []
        assert cal.logger.messages == []

    def test_reapplies_cached_k(self):
        cal, stepper = self._cached_current({"apply_stored_k": True})
        cal._reapply_current_k()
        assert stepper.applied == [(0.04, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_no_state_reapplies(self):
        cal, stepper = self._cached_current({"apply_stored_k": True})
        cal.afc.current_state = None
        cal._reapply_current_k()
        assert stepper.applied == [(0.04, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_no_afc_returns(self):
        cal, stepper = self._cached_current({"apply_stored_k": True})
        cal.afc = None
        cal._reapply_current_k()
        assert stepper.applied == []
        assert cal.logger.messages == []


class TestAFCautocalHandleHomeRailsEnd:
    @staticmethod
    def _cached_current(calibrator: bool = False) -> AFC_autocal:
        """A ready autocal applying stored K whose current lane1 has a cached K of 0.04."""
        cal = make_autocal({"apply_stored_k": True}, calibrator=calibrator, ready=True)
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        return cal

    def test_delegates(self):
        cal = self._cached_current()
        stepper = add_autocal_stepper(cal, "extruder")
        cal._handle_home_rails_end(object(), [])
        assert stepper.applied == [(0.04, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_error_logged(self):
        cal = self._cached_current(calibrator=True)
        cal.printer.objects["flow_calibrator"].error = RuntimeError("h")
        cal._handle_home_rails_end(object(), [])
        assert cal.logger.messages == [("warning", "AFC_autocal: home reapply error: h")]


class TestAFCautocalHandleActivateExtruder:
    @staticmethod
    def _cached_current(calibrator: bool = False) -> AFC_autocal:
        """A ready autocal applying stored K whose current lane1 has a cached K of 0.04."""
        cal = make_autocal({"apply_stored_k": True}, calibrator=calibrator, ready=True)
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        cal._lane_flow_k["lane1"] = (5, 0.04)
        return cal

    def test_delegates(self):
        cal = self._cached_current()
        stepper = add_autocal_stepper(cal, "extruder")
        cal._handle_activate_extruder()
        assert stepper.applied == [(0.04, 0.04)]
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_error_logged(self):
        cal = self._cached_current(calibrator=True)
        cal.printer.objects["flow_calibrator"].error = RuntimeError("a")
        cal._handle_activate_extruder()
        assert cal.logger.messages == [("warning", "AFC_autocal: activate reapply error: a")]


class TestAFCautocalCmdApplyLaneFlowK:
    def test_no_current_lane(self):
        cal = make_autocal(ready=True)
        gcmd = AutocalGcmd()
        cal.cmd_APPLY_LANE_FLOW_K(gcmd)
        assert gcmd.responses == ["AFC_autocal: no current lane"]
        assert cal.logger.messages == []

    def test_refuses_when_tool_not_on_toolhead(self, autocal_spoolman):
        cal = make_autocal(calibrator=True, ready=True, spoolman=True, toolhead="extruder1")
        flow = cal.printer.objects["flow_calibrator"]
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        autocal_spoolman.flow_k[5] = 0.04
        gcmd = AutocalGcmd()
        cal.cmd_APPLY_LANE_FLOW_K(gcmd)
        assert gcmd.responses == [
            "AFC_autocal: lane1's tool is not on the toolhead, pick up/load its tool first"]
        assert autocal_spoolman.reads == []
        assert cal._lane_flow_k == {}
        assert flow.applied == []
        assert cal.logger.messages == []

    def test_no_stored_k(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        gcmd = AutocalGcmd()
        cal.cmd_APPLY_LANE_FLOW_K(gcmd)
        assert gcmd.responses == ["AFC_autocal: no stored K for lane1"]
        assert autocal_spoolman.reads == [5]
        assert cal.logger.messages == []

    def test_applies(self, autocal_spoolman):
        cal = make_autocal(ready=True, spoolman=True)
        stepper = add_autocal_stepper(cal, "extruder")
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        autocal_spoolman.flow_k[5] = 0.04
        gcmd = AutocalGcmd()
        cal.cmd_APPLY_LANE_FLOW_K(gcmd)
        assert gcmd.responses == ["AFC autocal: applied K=0.040000 for lane1 on extruder"]
        assert stepper.applied == [(0.04, 0.04)]
        assert cal._lane_flow_k == {"lane1": (5, 0.04)}
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.040000 for lane1 on extruder")]

    def test_reports_not_applied_when_extruder_missing(self):
        cal = make_autocal(ready=True)
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5, extruder="gone")
        cal._lane_flow_k["lane1"] = (5, 0.04)
        gcmd = AutocalGcmd()
        cal.cmd_APPLY_LANE_FLOW_K(gcmd)
        assert gcmd.responses == ["AFC_autocal: K for lane1 was not applied, see the log"]
        assert cal._managed_extruders == set()
        assert cal.logger.messages == [("warning", "AFC autocal: extruder gone not found")]


class TestAFCautocalCmdCalibrateLaneFlowK:
    def test_no_current_lane(self):
        cal = make_autocal(calibrator=True, ready=True)
        gcmd = AutocalGcmd()
        cal.cmd_CALIBRATE_LANE_FLOW_K(gcmd)
        assert gcmd.responses == ["AFC_autocal: no current lane"]
        assert cal.printer.gcode.command_scripts == []
        assert cal.logger.messages == []

    def test_no_new_k(self):
        cal = make_autocal(calibrator=True, ready=True)
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        gcmd = AutocalGcmd()
        cal.cmd_CALIBRATE_LANE_FLOW_K(gcmd)
        assert gcmd.responses == ["AFC_autocal: calibration produced no new K"]
        assert cal.printer.gcode.command_scripts == ["FLOW_CALIBRATE"]
        assert cal.logger.messages == [
            ("info", "AFC autocal: calibration produced no new K for lane1")]

    def test_missing_calibrator_reports_both(self):
        cal = make_autocal(ready=True)
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        gcmd = AutocalGcmd()
        cal.cmd_CALIBRATE_LANE_FLOW_K(gcmd)
        assert gcmd.responses == ["AFC_autocal: flow_calibrator not found",
                                  "AFC_autocal: calibration produced no new K"]
        assert cal.logger.messages == []

    def test_reports_stored_k(self):
        cal = make_autocal(calibrator=True, ready=True)
        flow = cal.printer.objects["flow_calibrator"]
        cal.afc.function.current_lane = add_autocal_lane(cal, "lane1", 5)
        cal.printer.gcode.on_script = lambda script: flow._current_k.update(extruder=0.07)
        gcmd = AutocalGcmd()
        cal.cmd_CALIBRATE_LANE_FLOW_K(gcmd)
        assert gcmd.responses == ["AFC_autocal: stored K=0.070000 for lane1"]
        assert cal.printer.gcode.command_scripts == ["FLOW_CALIBRATE"]
        assert cal._lane_flow_k == {"lane1": (5, 0.07)}
        assert cal.logger.messages == [
            ("info", "AFC autocal: applied K=0.070000 for lane1 on extruder"),
            ("info", "AFC autocal: calibrated and stored K=0.070000 for lane1")]


class TestLoadConfig:
    def test_builds_instance(self):
        printer = AutocalPrinter()
        cal = load_config(AutocalConfig(printer, {"apply_stored_k": True}))
        assert isinstance(cal, AFC_autocal)
        assert cal.printer is printer
        assert cal.apply_stored_k is True
        assert sorted(printer.gcode.commands) == ["AFC_APPLY_LANE_FLOW_K",
                                                  "AFC_CALIBRATE_LANE_FLOW_K"]
        assert cal.logger.messages == []
