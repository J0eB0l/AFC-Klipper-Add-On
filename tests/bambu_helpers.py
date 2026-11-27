"""Typed builders for the Bambu AMS and AFC_BridgeBox tests."""

from __future__ import annotations

import configparser
import inspect
import json
import os
import pathlib
import queue
import socket
import tempfile
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union
from unittest.mock import patch

import pytest

from extras import AFC_BambuAMS as unit_mod
from extras import AFC_BambuAMS_bridge as bridge_mod
from extras import AFC_BambuAMS_rfid as rfid_mod
from extras import AFC_BoxTurtle_rfid as bt_rfid_mod
from extras.AFC import State
from extras.AFC_BambuAMS import afcBambuAMS, bridge_slot_to_info, build_slot_map
from extras.AFC_BambuAMS_bridge import BambuBridge, TcpPort
from extras.AFC_BambuAMS_rfid import AFC_BambuAMS_RFID, BambuSpoolman
from extras.AFC_BridgeBox import afcBridgeBox
from extras.AFC_functions import afcFunction
from extras.AFC_lane import AFCLane, AFCLaneState
from extras.AFC_spool import AFCSpool
from tests.conftest import (_MOCK_CONFIG_SENTINEL, CommandError, MockAFC,
                            MockConfig, MockGCodeCommand, MockPrinter)

#: A log line as every recorder here keeps it: (level, message).
LogLine = Tuple[str, str]

#: The print_stats states AFC treats as "not in a print".
IDLE_PRINT_STATES = ("standby", "error", "complete", "cancelled")

#: AFC's VarFile until a test gives it one.
NO_VAR_FILE = "/nonexistent-bambu-test/AFC.var"

#: The bridge table the bridge module was imported with.
_MODULE_BRIDGES = bridge_mod._BRIDGES


# ── recorders ────────────────────────────────────────────────────────────────

class Recorder:
    """A plain callable that records its calls; may return a value or raise."""

    def __init__(self, result: Any = None,
                 raises: Optional[BaseException] = None) -> None:
        """
        :param result: what every call returns
        :param raises: an exception every call raises instead
        """
        self.calls: List[Tuple[tuple, dict]] = []
        self.result = result
        self.raises = raises

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        """Record the call, then return ``result`` or raise ``raises``."""
        self.calls.append((args, kwargs))
        if self.raises is not None:
            raise self.raises
        return self.result

    @property
    def call_count(self) -> int:
        """:return int: how many calls were made"""
        return len(self.calls)


class GcmdRecorder(Recorder):
    """A Recorder standing in for a method whose gcmd is required."""

    def __call__(self, *args: Any, gcmd: Any, **kwargs: Any) -> Any:
        """Record the call, gcmd among its keywords."""
        return super().__call__(*args, gcmd=gcmd, **kwargs)


class BambuLogger:
    """AFC_logger with the real signatures, recording ``(level, message)``."""

    #: AFC's debug switch.
    print_debug_console = True

    def __init__(self) -> None:
        """Start with every record empty."""
        self.messages: List[LogLine] = []
        #: Lines logged with only_debug=True (AFC.log only, never console).
        self.file_only: List[str] = []
        #: Lines logged with console_only=True (console only, never AFC.log).
        self.console_only: List[str] = []
        #: (message, traceback) for every line logged with a traceback.
        self.tracebacks: List[Tuple[str, str]] = []
        #: (message, stack_name) for every error logged with a stack name.
        self.stack_names: List[Tuple[str, str]] = []

    def info(self, message: str, console_only: bool = False) -> None:
        """
        :param message: the line
        :param console_only: True keeps it out of AFC.log; listed in
          console_only
        """
        self.messages.append(("info", message))
        if console_only:
            self.console_only.append(message)

    def warning(self, message: str) -> None:
        """:param message: the line"""
        self.messages.append(("warning", message))

    def debug(self, message: str, only_debug: bool = False,
              traceback: Optional[str] = None) -> None:
        """
        :param message: the line
        :param only_debug: True keeps it off the console; listed in file_only
        :param traceback: listed in tracebacks with the line
        """
        self.messages.append(("debug", message))
        if only_debug:
            self.file_only.append(message)
        if traceback is not None:
            self.tracebacks.append((message, traceback))

    def error(self, message: str, traceback: Optional[str] = None,
              stack_name: str = "") -> None:
        """
        :param message: the line
        :param traceback: listed in tracebacks with the line
        :param stack_name: listed in stack_names with the line, when set
        """
        self.messages.append(("error", message))
        if traceback is not None:
            self.tracebacks.append((message, traceback))
        if stack_name:
            self.stack_names.append((message, stack_name))

    def raw(self, message: str) -> None:
        """:param message: the line"""
        self.messages.append(("raw", message))


class FakeMutex:
    """A reactor or gcode mutex: a context manager that is never contended."""

    def __init__(self) -> None:
        """Start unlocked."""
        self.locked = False

    def test(self) -> bool:
        """:return bool: whether the mutex is held"""
        return self.locked

    def __enter__(self) -> "FakeMutex":
        """:return FakeMutex: this mutex, now held"""
        self.locked = True
        return self

    def __exit__(self, *exc: Any) -> None:
        """Release the mutex."""
        self.locked = False


class RecordingGcode:
    """Klipper's gcode object as AFC uses it, recording every console line."""

    def __init__(self) -> None:
        """Start with no commands and no output."""
        self.messages: List[LogLine] = []
        self.ready_gcode_handlers: Dict[str, Callable[[Any], Any]] = {}
        self.mux_commands: Dict[str, Tuple[str, Dict[Any, Callable]]] = {}
        self.scripts: List[str] = []
        self.mutex = FakeMutex()

    def register_command(self, cmd: str, func: Optional[Callable],
                         when_not_ready: bool = False,
                         desc: Optional[str] = None) -> Optional[Callable]:
        """
        :param cmd: command name
        :param func: handler, or None to unregister
        :return Optional[Callable]: the handler removed, when unregistering
        """
        if func is None:
            return self.ready_gcode_handlers.pop(cmd, None)
        if cmd in self.ready_gcode_handlers:
            error_str = f"gcode command {cmd} already registered"
            raise configparser.Error(error_str)
        self.ready_gcode_handlers[cmd] = func
        return None

    def register_mux_command(self, cmd: str, key: str, value: Any,
                             func: Callable, desc: Optional[str] = None
                             ) -> None:
        """
        :param cmd: command name
        :param key: the parameter that selects the handler
        :param value: this handler's value of ``key``
        :param func: handler
        """
        prev = self.mux_commands.get(cmd)
        if prev is None:
            self.register_command(cmd, lambda gcmd: self._cmd_mux(cmd, gcmd),
                                  desc=desc)
            prev = self.mux_commands[cmd] = (key, {})
        prev_key, values = prev
        if prev_key != key:
            error_str = (f"mux command {cmd} {key} {value} may have only one "
                         f"key ({prev_key})")
            raise configparser.Error(error_str)
        if value in values:
            error_str = f"mux command {cmd} {key} {value} already registered"
            raise configparser.Error(error_str)
        values[value] = func

    def _cmd_mux(self, cmd: str, gcmd: Any) -> Any:
        """Route a mux command to the handler its key selects, as Klipper does."""
        key, values = self.mux_commands[cmd]
        if None in values:
            key_param = gcmd.get(key, None)
        else:
            key_param = gcmd.get(key)
        if key_param not in values:
            error_str = f"The value '{key_param}' is not valid for {key}"
            raise gcmd.error(error_str)
        return values[key_param](gcmd)

    def run(self, cmd: str, **params: Any) -> "FakeGcmd":
        """Run a registered command with ``params``."""
        line = " ".join([cmd] + [f"{k}={v}" for k, v in params.items()])
        gcmd = FakeGcmd(params, command=cmd, commandline=line)
        self.ready_gcode_handlers[cmd](gcmd)
        return gcmd

    def respond_info(self, msg: str, log: bool = True) -> None:
        """:param msg: the console line"""
        self.messages.append(("respond_info", msg))

    def respond_raw(self, msg: str) -> None:
        """:param msg: the console line"""
        self.messages.append(("respond_raw", msg))

    def run_script_from_command(self, script: str, gcmd: Any = None) -> None:
        """
        Newer Kalico's signature, so AFC dispatches on the command context.

        :param script: recorded in ``scripts``
        :param gcmd: the running command; unused
        """
        self.scripts.append(script)

    def run_script(self, script: str) -> None:
        """:param script: recorded in ``scripts``"""
        self.scripts.append(script)

    def get_mutex(self) -> FakeMutex:
        """:return FakeMutex: the gcode mutex"""
        return self.mutex


class FakeGcmd(MockGCodeCommand):
    """
    A GCodeCommand: conftest's get/get_int/get_float semantics, with ``respond_info`` /
    ``respond_raw`` recorded in ``.messages``.
    """

    def __init__(self, params: Optional[Dict[str, Any]] = None,
                 command: str = "", commandline: str = "") -> None:
        """
        :param params: parameter values, as given (ints may stay ints)
        :param command: command name
        :param commandline: the raw line
        """
        super().__init__(params=params, commandline=commandline,
                         command=command)
        self.messages: List[LogLine] = []
        self.respond_info = self._respond_info
        self.respond_raw = self._respond_raw

    def _respond_info(self, msg: str, log: bool = True) -> None:
        """:param msg: the response"""
        self.messages.append(("respond_info", msg))

    def _respond_raw(self, msg: str) -> None:
        """:param msg: the response"""
        self.messages.append(("respond_raw", msg))


# ── time ─────────────────────────────────────────────────────────────────────

class FakeTimer:
    """A registered reactor timer."""

    def __init__(self, callback: Callable[[float], float],
                 waketime: float) -> None:
        """
        :param callback: called with eventtime; returns the next waketime
        :param waketime: when it is due
        """
        self.callback = callback
        self.waketime = waketime


class FakeCompletion:
    """A reactor completion: ``wait`` returns what ``complete`` was given."""

    def __init__(self) -> None:
        """Start not completed."""
        self.result: Any = None
        self.done = False

    def complete(self, result: Any) -> None:
        """:param result: the value wait() returns"""
        self.result, self.done = result, True

    def test(self) -> bool:
        """:return bool: whether it completed"""
        return self.done

    def wait(self, waketime: float = 0.0, waketime_result: Any = None) -> Any:
        """:return Any: the result, or ``waketime_result`` if not completed"""
        return self.result if self.done else waketime_result


class FakeReactor:
    """A controllable reactor and monotonic clock."""

    NOW = 0.0
    NEVER = 9_999_999_999.0

    def __init__(self, now: float = 100.0, step: float = 0.0,
                 run_in_pause: bool = False) -> None:
        """
        :param now: the starting time
        :param step: seconds every monotonic() read advances the clock
        :param run_in_pause: pause() runs due callbacks and timers
        """
        self.now = float(now)
        self.step = float(step)
        self.run_in_pause = run_in_pause
        self.callbacks: List[Tuple[Callable[[float], Any], float]] = []
        self.async_callbacks: List[Callable[[float], Any]] = []
        self.timers: List[FakeTimer] = []
        self.pauses: List[float] = []

    def monotonic(self) -> float:
        """:return float: the current time (then ticks by ``step``)"""
        t = self.now
        self.now += self.step
        return t

    def advance(self, seconds: float) -> float:
        """
        :param seconds: how far to move the clock
        :return float: the new time
        """
        self.now += float(seconds)
        return self.now

    def pause(self, waketime: float) -> float:
        """
        :param waketime: when to wake; the clock jumps there if it is later
        :return float: the time after the pause
        """
        self.pauses.append(float(waketime))
        self.now = max(self.now, float(waketime))
        if self.run_in_pause:
            self.run_callbacks()
            self.run_timers()
        return self.now

    def register_callback(self, callback: Callable[[float], Any],
                          waketime: float = NOW) -> None:
        """:param callback: run by run_callbacks once ``waketime`` is due"""
        self.callbacks.append((callback, float(waketime)))

    def register_async_callback(self, callback: Callable[[float], Any],
                                waketime: float = NOW) -> None:
        """:param callback: run by the next run_callbacks"""
        self.async_callbacks.append(callback)

    def register_timer(self, callback: Callable[[float], float],
                       waketime: float = NEVER) -> FakeTimer:
        """
        :param callback: the timer function
        :param waketime: first wake
        :return FakeTimer: the handle
        """
        timer = FakeTimer(callback, float(waketime))
        self.timers.append(timer)
        return timer

    def update_timer(self, timer: FakeTimer, waketime: float) -> None:
        """:param waketime: the timer's next wake"""
        timer.waketime = float(waketime)

    def unregister_timer(self, timer: FakeTimer) -> None:
        """:param timer: the handle to drop"""
        if timer in self.timers:
            self.timers.remove(timer)

    def mutex(self, is_locked: bool = False) -> FakeMutex:
        """:return FakeMutex: an uncontended mutex"""
        return FakeMutex()

    def completion(self) -> FakeCompletion:
        """:return FakeCompletion: a new completion"""
        return FakeCompletion()

    def run_callbacks(self, until: Optional[float] = None) -> int:
        """Run every async callback and every callback due by now."""
        if until is not None:
            self.now = max(self.now, float(until))
        ran = 0
        for _ in range(1000):
            pending, self.async_callbacks = self.async_callbacks, []
            due = [c for c in self.callbacks if c[1] <= self.now]
            self.callbacks = [c for c in self.callbacks if c[1] > self.now]
            if not pending and not due:
                return ran
            for cb in pending:
                cb(self.now)
                ran += 1
            for cb, _when in due:
                cb(self.now)
                ran += 1
        return ran

    def run_timers(self, until: Optional[float] = None) -> int:
        """Fire every timer due by now once, rescheduling it to what it returns."""
        if until is not None:
            self.now = max(self.now, float(until))
        fired = 0
        for timer in [t for t in self.timers if t.waketime <= self.now]:
            timer.waketime = float(timer.callback(self.now))
            fired += 1
        return fired


class _ClockTime:
    """A ``time`` module whose clocks read a FakeReactor."""

    def __init__(self, reactor: FakeReactor) -> None:
        """:param reactor: the clock to read"""
        self._reactor = reactor

    @property
    def reactor(self) -> FakeReactor:
        """:return FakeReactor: the clock this module reads"""
        return self._reactor

    def monotonic(self) -> float:
        """:return float: the reactor's time"""
        return self._reactor.now

    def time(self) -> float:
        """:return float: the reactor's time"""
        return self._reactor.now

    def sleep(self, seconds: float) -> None:
        """:param seconds: advances the reactor's clock"""
        self._reactor.advance(seconds)


def patch_module_time(monkeypatch: pytest.MonkeyPatch,
                      reactor: FakeReactor) -> None:
    """Point ``time`` in AFC_BambuAMS and AFC_BambuAMS_bridge at the reactor."""
    clock = _ClockTime(reactor)
    monkeypatch.setattr(bridge_mod, "time", clock)
    monkeypatch.setattr(unit_mod, "time", clock)


def _module_clock() -> Optional[FakeReactor]:
    """:return Optional[FakeReactor]: the reactor the bridge module reads"""
    clock = bridge_mod.time
    return clock.reactor if isinstance(clock, _ClockTime) else None


# ── Klipper objects ──────────────────────────────────────────────────────────

class FakePrintStats:
    """Klipper's print_stats object."""

    def __init__(self, state: str = "standby", filename: str = "") -> None:
        """
        :param state: print_stats state
        :param filename: the file being printed
        """
        self.state = state
        self.filename = filename

    def get_status(self, eventtime: float = 0.0) -> Dict[str, Any]:
        """:return dict: state and filename"""
        return {"state": self.state, "filename": self.filename}


class FakePauseResume:
    """Klipper's pause_resume: its paused flag and the requests sent to it."""

    def __init__(self, is_paused: bool = False) -> None:
        """:param is_paused: whether the print is paused"""
        self.is_paused = is_paused
        #: "pause" / "resume", in the order they were requested.
        self.sent: List[str] = []

    def get_status(self, eventtime: float = 0.0) -> Dict[str, Any]:
        """:return dict: is_paused"""
        return {"is_paused": self.is_paused}

    def send_pause_command(self) -> None:
        """Record a pause request (Klipper's PAUSE, not this, sets is_paused)."""
        self.sent.append("pause")

    def send_resume_command(self) -> None:
        """Record a resume request."""
        self.sent.append("resume")


class FakeIdleTimeout:
    """Klipper's idle_timeout: ``state`` is "Idle", "Ready" or "Printing"."""

    def __init__(self, state: str = "Idle", idle_timeout: float = 600.0
                 ) -> None:
        """
        :param state: the idle state
        :param idle_timeout: the configured timeout, seconds
        """
        self.state = state
        self.idle_timeout = idle_timeout

    def get_status(self, eventtime: float = 0.0) -> Dict[str, Any]:
        """:return dict: state and printing_time"""
        return {"state": self.state, "printing_time": 0.0}


class FakeMcu:
    """Klipper's primary mcu: its print time is the reactor's time."""

    def estimated_print_time(self, eventtime: float) -> float:
        """:return float: ``eventtime``"""
        return eventtime


class FakeToolheadExtruder:
    """
    Klipper's PrinterExtruder (what ``AFC_extruder.toolhead_extruder`` holds), registered on the
    printer under its name.
    """

    def __init__(self, name: str = "extruder", position: float = 0.0) -> None:
        """
        :param name: the extruder's Klipper name
        :param position: what find_past_position returns
        """
        self.name = name
        self.position = position

    def get_name(self) -> str:
        """:return str: the extruder's Klipper name"""
        return self.name

    def find_past_position(self, print_time: float) -> float:
        """:return float: ``position``"""
        return self.position


class FakeKinematics:
    """The toolhead kinematics: just the homed axes."""

    def __init__(self, homed_axes: str = "xyz") -> None:
        """:param homed_axes: the homed axes, e.g. "xyz" or ""."""
        self.homed_axes = homed_axes

    def get_status(self, eventtime: float = 0.0) -> Dict[str, Any]:
        """:return dict: homed_axes"""
        return {"homed_axes": self.homed_axes}


class FakeToolhead:
    """
    Klipper's toolhead as AFC drives it: a settable E position, the active extruder, and recorded
    waits and extruder switches.
    """

    def __init__(self, extruder: FakeToolheadExtruder, e_pos: float = 0.0,
                 homed_axes: str = "xyz") -> None:
        """
        :param extruder: the active extruder
        :param e_pos: the commanded E position
        :param homed_axes: what the kinematics report homed
        """
        self.extruder = extruder
        self.e_pos = e_pos
        self.kinematics = FakeKinematics(homed_axes)
        #: Names of the extruders set_extruder activated, in order.
        self.activations: List[str] = []
        self.wait_moves_calls = 0
        self.flushes = 0
        self.dwells: List[float] = []
        self.last_move_time = 0.0

    def get_position(self) -> List[float]:
        """:return list: [x, y, z, e], with e = ``e_pos``"""
        return [0.0, 0.0, 0.0, self.e_pos]

    def get_extruder(self) -> FakeToolheadExtruder:
        """:return FakeToolheadExtruder: the active extruder"""
        return self.extruder

    def set_extruder(self, extruder: FakeToolheadExtruder,
                     extrude_pos: float) -> None:
        """
        :param extruder: the extruder to activate (recorded in activations)
        :param extrude_pos: the new E position, as Klipper sets it
        """
        self.extruder = extruder
        self.e_pos = float(extrude_pos)
        self.activations.append(extruder.name)

    def flush_step_generation(self) -> None:
        """Count a flush."""
        self.flushes += 1

    def wait_moves(self) -> None:
        """Count a wait."""
        self.wait_moves_calls += 1

    def get_kinematics(self) -> FakeKinematics:
        """:return FakeKinematics: the kinematics"""
        return self.kinematics

    def get_last_move_time(self) -> float:
        """:return float: ``last_move_time``"""
        return self.last_move_time

    def dwell(self, delay: float) -> None:
        """:param delay: recorded in dwells"""
        self.dwells.append(delay)


class FakeGcodeMove:
    """Klipper's gcode_move as AFC reads and moves it."""

    def __init__(self, toolhead: FakeToolhead) -> None:
        """:param toolhead: the toolhead its moves reach"""
        self.toolhead = toolhead
        self.last_position: List[float] = [0.0, 0.0, 0.0, toolhead.e_pos]
        self.base_position: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.absolute_coord = True
        self.absolute_extrude = True
        self.speed = 25.0
        self.speed_factor = 1.0 / 60.0
        self.extrude_factor = 1.0
        #: (position, speed) of every move_with_transform, in order.
        self.moves: List[Tuple[List[float], float]] = []

    def move_with_transform(self, newpos: List[float], speed: float) -> None:
        """
        :param newpos: the target [x, y, z, e]
        :param speed: mm/s
        """
        self.moves.append((list(newpos), speed))
        self.toolhead.e_pos = float(newpos[3])


class FakeLed:
    """An [AFC_led] section: records every ``led_change``."""

    def __init__(self, name: str) -> None:
        """:param name: the section's name (AFC_led <name>)"""
        self.name = name
        #: (indexes, colour) of every led_change, in order.
        self.changes: List[Tuple[List[int], Any]] = []

    def led_change(self, index: Sequence[int], status: Any) -> None:
        """
        :param index: the LED indexes
        :param status: the colour
        """
        self.changes.append((list(index), status))


# ── AFC core ─────────────────────────────────────────────────────────────────

class FakeError:
    """AFC's error object (afcError), recording what reaches it."""

    def __init__(self, afc: Optional["BambuAFC"] = None,
                 pause_resume: Optional[FakePauseResume] = None) -> None:
        """
        :param afc: the AFC core whose error state a pause sets
        :param pause_resume: the printer's pause_resume
        """
        self.afc = afc
        self.pause_resume = pause_resume or FakePauseResume()
        self.messages: List[LogLine] = []
        self.calls: List[Tuple[str, tuple, dict]] = []

    def _pause(self) -> None:
        """Enter AFC's error state, as pause_print does."""
        if self.afc is not None:
            self.afc.error_state = True
            self.afc.current_state = State.ERROR

    def AFC_error(self, msg: str, pause: bool = True,
                  stack_name: Optional[str] = None, *, gcmd: Any) -> None:
        """
        :param msg: the error
        :param pause: pause the print (enters the error state)
        :param stack_name: recorded in calls when given
        :param gcmd: command context the pause runs through
        """
        self.messages.append(("AFC_error", msg))
        kwargs = {} if stack_name is None else {"stack_name": stack_name}
        self.calls.append(("AFC_error", (msg, pause), kwargs))
        if pause:
            self._pause()

    def handle_lane_failure(self, lane: Any, message: str,
                            pause: bool = True, *, gcmd: Any) -> None:
        """
        :param lane: the failed lane: disabled, ERROR, its unit's lane_fault
        :param message: the failure text
        :param pause: pause the print (enters the error state)
        :param gcmd: command context the pause and LED macro run through
        """
        self.messages.append(("handle_lane_failure", message))
        self.calls.append(("handle_lane_failure", (lane, message),
                           {"pause": pause}))
        lane.do_enable(False)
        lane.status = AFCLaneState.ERROR
        if pause:
            self._pause()
        lane.unit_obj.lane_fault(lane, gcmd)


class FakeAfcSpool:
    """AFC core's spool object (AFC_spool) as lanes, units and masters call it."""

    def __init__(self) -> None:
        """Start with no calls."""
        self.calls: List[Tuple[str, tuple, dict]] = []
        # The gcmd each set_spoolID ran under, in call order.
        self.gcmds: List[Any] = []

    def _rec(self, name: str, args: tuple, kwargs: dict) -> None:
        """
        :param name: the method called
        :param args: its positional arguments
        :param kwargs: its keyword arguments
        """
        self.calls.append((name, args, kwargs))

    def set_spoolID(self, lane: Any, spool_id: Any, *args: Any, gcmd: Any,
                    **kwargs: Any) -> None:
        """Record a Spoolman binding, and its gcmd in gcmds."""
        self.gcmds.append(gcmd)
        self._rec("set_spoolID", (lane, spool_id) + args, kwargs)

    def _set_values(self, lane: Any) -> None:
        """Record a lane's values being filled."""
        self._rec("_set_values", (lane,), {})

    def clear_values(self, lane: Any) -> None:
        """Record a lane's values being cleared."""
        self._rec("clear_values", (lane,), {})

    def set_active_spool(self, spool_id: Any) -> None:
        """Record the active spool."""
        self._rec("set_active_spool", (spool_id,), {})

    def set_snapmaker_filament_params(self, lane: Any) -> None:
        """Record the Snapmaker filament hook."""
        self._rec("set_snapmaker_filament_params", (lane,), {})

    def _reset_mapping(self, runout_opt: str = "no") -> None:
        """Record a T# mapping reset (AFC_RESET_MAPPING), which a master wraps."""
        self._rec("_reset_mapping", (runout_opt,), {})


class FakeDeltaTime:
    """
    afcDeltaTime: stage markers in ``.messages`` as ``(level, message)``, the level the real one
    logs at ("debug" or "info") and the message without its wall-clock suffix.
    """

    def __init__(self) -> None:
        """Start with no markers."""
        self.start_time: Optional[float] = None
        self.messages: List[LogLine] = []

    def set_start_time(self) -> None:
        """Start the clock."""
        self.start_time = 0.0

    def log_with_time(self, msg: str, debug: bool = True) -> None:
        """:param msg: the stage marker"""
        self.messages.append(("debug" if debug else "info", msg))

    def log_major_delta(self, msg: str, debug: bool = True) -> None:
        """:param msg: the stage marker"""
        self.messages.append(("debug" if debug else "info", msg))


class FakeFunction:
    """
    The bare :class:`BambuPrinter`'s stand-in for afcFunction: print state and the current lane and
    extruder, read the way afcFunction reads them, and command registration. :func:`make_printer`
    replaces it with a real afcFunction; build lanes and units on that printer.
    """

    def __init__(self, afc: "BambuAFC", printer: "BambuPrinter") -> None:
        """
        :param afc: the AFC core this belongs to
        :param printer: the printer whose Klipper objects it reads
        """
        self.afc = afc
        self.printer = printer
        self.logger = afc.logger
        #: Macros afcFunction would show in the web UI (show_macros).
        self.macros: List[str] = []

    def _status(self, name: str) -> Dict[str, Any]:
        """
        :param name: a Klipper object with get_status
        :return dict: its status now
        """
        obj = self.printer.lookup_object(name)
        return obj.get_status(self.afc.reactor.monotonic())

    def in_print(self, return_file: bool = False) -> Any:
        """:return bool: the print is neither idle nor finished"""
        status = self._status("print_stats")
        busy = status["state"] not in IDLE_PRINT_STATES
        return (busy, status["filename"]) if return_file else busy

    def is_printing(self, check_movement: bool = False) -> bool:
        """:return bool: print_stats says printing (or idle_timeout moving)"""
        moving = False
        if check_movement:
            moving = self._status("idle_timeout")["state"] == "Printing"
        return self._status("print_stats")["state"] == "printing" or moving

    def is_paused(self) -> bool:
        """:return bool: pause_resume says paused"""
        return bool(self._status("pause_resume")["is_paused"])

    def get_current_extruder(self) -> Optional[str]:
        """:return Optional[str]: the toolhead's extruder, if AFC has it"""
        name = self.afc.toolhead.get_extruder().name
        tool = self.afc.tools.get(name, None)
        return name if tool is not None and tool.on_shuttle() else None

    def get_current_lane(self) -> Optional[str]:
        """:return Optional[str]: the current extruder's loaded lane"""
        if self.printer.state_message == "Printer is ready":
            current = self.get_current_extruder()
            if current is not None:
                return self.afc.tools[current].lane_loaded
        return None

    def get_current_lane_obj(self) -> Any:
        """:return Any: the lane get_current_lane names, or None"""
        return self.afc.lanes.get(self.get_current_lane())

    def register_mux_command(self, show_macros: bool, cmd: str, key: str,
                             value: str, func: Callable,
                             description: Optional[str] = None,
                             options: Any = None) -> None:
        """
        Register through the printer's gcode as afcFunction does: as ``_<cmd>`` when
        ``show_macros``, with the macro recorded in ``macros`` instead of written into the config.
        """
        name = f"_{cmd}" if show_macros else cmd
        self.afc.gcode.register_mux_command(name, key, value, func,
                                            desc=description)
        if show_macros:
            self.macros.append(cmd)

    def register_commands(self, show_macros: bool, cmd: str, func: Callable,
                          description: Optional[str] = None,
                          options: Any = None) -> None:
        """afcFunction.register_commands, as :meth:`register_mux_command`."""
        name = f"_{cmd}" if show_macros else cmd
        self.afc.gcode.register_command(name, func, desc=description)
        if show_macros:
            self.macros.append(cmd)


class SaveVars:
    """``afc.save_vars`` as a recorder."""

    def __init__(self, afc: "BambuAFC") -> None:
        """:param afc: the AFC core whose lanes are saved"""
        self.afc = afc
        self.calls: List[Tuple[tuple, dict]] = []

    def __call__(self) -> None:
        """Record the call and queue the snapshot."""
        self.calls.append(((), {}))
        afc = self.afc
        if not afc.prep_done:
            return
        snap: Dict[str, Any] = {}
        for unit in afc.units.values():
            snap[unit.name] = {name: afc.lanes[name].get_status(
                save_to_file=True) for name in unit.lanes}
        extruders: Dict[str, Dict[str, Any]] = {}
        for tool in afc.tools.values():
            entry: Dict[str, Any] = {"lane_loaded": tool.lane_loaded}
            if getattr(tool, "tool_start", None) == "virtual":
                entry["virtual_tool_start"] = bool(tool.tool_start_state)
            extruders[tool.name] = entry
        snap["system"] = {"current_load": afc.current,
                          "num_units": len(afc.units),
                          "num_lanes": len(afc.lanes),
                          "num_extruders": len(afc.tools),
                          "extruders": extruders,
                          "bypass": {"enabled": False}}
        afc.printer.send_event("afc:save_vars", snap)
        afc._var_write_queue.put_nowait(snap)

    @property
    def call_count(self) -> int:
        """:return int: how many calls were made"""
        return len(self.calls)


def _config_dir(var_file: str) -> str:
    """
    :param var_file: AFC's VarFile
    :return str: its directory with the trailing slash, as AFC's cfgloc
    """
    return var_file[:var_file.rfind("/") + 1] if "/" in var_file else var_file


class BambuAFC(MockAFC):
    """
    conftest's MockAFC with the parts the Bambu code reads made explicit: one logger, one reactor,
    the printer's gcode, toolhead and gcode_move, and recording fakes for function, error, spool,
    save_vars, the var-file write queue, the delta-time logger and AFC's own toolhead actions
    (``save_pos``, ``restore_pos``, ``move_z_pos``, ``CHANGE_TOOL``: :class:`Recorder`).
    """

    cmd_CHANGE_TOOL_help = "change tool"

    def run_script(self, script: str, gcmd: Any) -> None:
        """
        AFC's run_script on newer Kalico. A real command runs the script in its own context, which
        records it through run_script_from_command; "callback" and "sub_command" take AFC's own
        dispatch, so "callback" goes through gcode.run_script.
        """
        if (gcmd is None
            or isinstance(gcmd, str)):
            MockAFC.run_script(self, script, gcmd)
        else:
            self.gcode.run_script_from_command(script)

    def __init__(self, printer: "BambuPrinter") -> None:
        """:param printer: the printer this AFC core lives on"""
        super().__init__()
        self.printer = printer
        self.logger = BambuLogger()
        self.reactor = printer.reactor
        self.gcode = printer.gcode
        self._rsfc_accepts_gcmd = "gcmd" in inspect.signature(
            self.gcode.run_script_from_command).parameters
        self.toolhead = printer.toolhead
        self.gcode_move = printer.gcode_move
        self.function: Any = FakeFunction(self, printer)
        self.error = FakeError(self, printer.pause_resume)
        self.spool: Any = FakeAfcSpool()
        self._var_write_queue: Any = queue.Queue()
        self.save_vars = SaveVars(self)
        self.afcDeltaTime = FakeDeltaTime()
        self.common_density_values = ["PLA:1.24", "PETG:1.23", "ABS:1.04",
                                      "ASA:1.07"]
        self.force_assign_map = False
        self.prep_done = True
        self.current_loading: Optional[str] = None
        self.post_unload_macro: Optional[str] = None
        self.disable_homing_check = False
        self.auto_home = False
        self.auto_level_macro: Optional[str] = None
        self.tool_changes: List[Any] = []
        self.save_pos = Recorder()
        self.restore_pos = Recorder()
        self.move_z_pos = Recorder()
        self.CHANGE_TOOL = Recorder()
        #: AFC's bypass sensor: none configured, so get_bypass_state() reads ``bypass_state``.
        self.bypass: Any = None
        self.bypass_state = False
        self.e_moves: List[Tuple[float, float, str, bool]] = []
        self.cut_tip_forms: List[Tuple[Any, Any]] = []
        self.VarFile = NO_VAR_FILE
        self.cfgloc = _config_dir(NO_VAR_FILE)

    @property
    def current(self) -> Optional[str]:
        """:return Optional[str]: AFC.current, the function's current lane"""
        return self.function.get_current_lane()

    @current.setter
    def current(self, value: None) -> None:
        """Accept MockAFC's initial None only: AFC.current has no setter."""
        if value is not None:
            error_str = ("AFC.current is read from the toolhead's extruder: "
                         "set afc.tools[<extruder>].lane_loaded instead")
            raise AttributeError(error_str)

    def get_bypass_state(self) -> bool:
        """:return bool: ``bypass_state``"""
        return self.bypass_state

    def move_e_pos(self, e_amount: float, speed: float, log_string: str = "",
                   wait_tool: bool = False) -> None:
        """
        AFC.move_e_pos: move E by ``e_amount`` through gcode_move, then wait on the toolhead when
        asked.
        """
        self.e_moves.append((e_amount, speed, log_string, wait_tool))
        newpos = self.gcode_move.last_position
        newpos[3] += e_amount
        self.gcode_move.move_with_transform(newpos, speed)
        if wait_tool:
            self.toolhead.wait_moves()

    def do_tool_cut_tip_form(self, cur_lane: Any, cur_extruder: Any, gcmd: Any) -> None:
        """
        :param cur_lane: the lane being unloaded
        :param cur_extruder: its extruder; both recorded in cut_tip_forms
        :param gcmd: the unload's command, or "callback"
        """
        self.cut_tip_forms.append((cur_lane, cur_extruder))

    def cmd_CHANGE_TOOL(self, gcmd: Any) -> None:
        """The T# handler: records the command."""
        self.tool_changes.append(gcmd)


# ── config and printer ───────────────────────────────────────────────────────

class BambuConfig(MockConfig):
    """
    conftest's MockConfig plus the accessors the Bambu code needs: ``getchoice``, a ``getlist`` that
    splits strings, ``getsection`` over a fileconfig shared by every section on the printer, and the
    printer's ``access_tracking``.
    """

    def __init__(self, name: str, printer: Any,
                 values: Optional[Dict[str, Any]] = None,
                 fileconfig: Optional[configparser.RawConfigParser] = None
                 ) -> None:
        """
        :param name: the section name, e.g. "AFC_BambuAMS Bambu_AMS_1"
        :param printer: the printer
        :param values: option values; a missing option takes its default
        :param fileconfig: the merged config; the printer's when None
        """
        super().__init__(name=name, printer=printer, values=dict(values or {}))
        shared = (fileconfig if fileconfig is not None
                  else getattr(printer, "fileconfig", None))
        if shared is not None:
            self.fileconfig = shared
        tracking = getattr(printer, "access_tracking", None)
        self.access_tracking: Dict[Tuple[str, str], int] = (
            tracking if isinstance(tracking, dict) else {})

    def getchoice(self, option: str, choices: Any,
                  default: Any = _MOCK_CONFIG_SENTINEL) -> Any:
        """
        :param choices: a dict (value -> result) or a list of values
        :return Any: the chosen result
        """
        value = self._require(option, default)
        if value not in choices:
            error_str = f"Choice '{value}' for option '{option}' is not valid"
            raise configparser.Error(error_str)
        return choices[value] if isinstance(choices, dict) else value

    def getlist(self, option: str, default: Any = _MOCK_CONFIG_SENTINEL,
                sep: str = ",", count: Optional[int] = None,
                **kwargs: Any) -> Any:
        """:return list: the option split on ``sep``, stripped"""
        value = self._require(option, default)
        if value is None:
            return []
        if isinstance(value, str):
            return [p.strip() for p in value.split(sep) if p.strip()]
        return list(value)

    def getsection(self, section: str) -> "BambuConfig":
        """
        :param section: another section of the merged config
        :return BambuConfig: that section, its options read from the config
        """
        values: Dict[str, Any] = {}
        if self.fileconfig.has_section(section):
            values = dict(self.fileconfig.items(section))
        return BambuConfig(section, self.get_printer(), values,
                           fileconfig=self.fileconfig)

    def setdefault(self, option: str, value: Any) -> None:
        """:param value: used when ``option`` is not set"""
        self._values.setdefault(option, value)


class FabricatedSection:
    """
    A section loaded only as its name and keys: what a master fabricates on a printer without
    ``fabricate``, and klippy's ``gcode_macro`` wrappers.
    """

    def __init__(self, section: str, keys: Dict[str, str]) -> None:
        """
        :param section: the section name
        :param keys: its options, as written
        """
        self.section = section
        self.name = section.split()[-1]
        self.keys = keys
        self.lanes: Dict[str, Any] = {}


def _bambu_ready_owners(handlers: Sequence[Callable]) -> List[Any]:
    """
    :param handlers: klippy:ready handlers
    :return list: the units and masters among their owners
    """
    owners = [getattr(h, "__self__", None) for h in handlers]
    return [o for o in owners if isinstance(o, (afcBambuAMS, afcBridgeBox))]


def bridges_isolated() -> bool:
    """:return bool: a test-scoped bridge table is live (see use_bridges)"""
    return bridge_mod._BRIDGES is not _MODULE_BRIDGES


def live_bridges() -> Dict[str, Any]:
    """:return dict: the bridge table ready reads (serial port -> bridge)"""
    return bridge_mod._BRIDGES


def _require_isolated_bridges(what: str) -> None:
    """
    :param what: what is about to read the bridge table
    :raises RuntimeError: when it is the module-wide table
    """
    if bridges_isolated():
        return
    error_str = (f"{what} would store real bridges in AFC_BambuAMS_bridge's "
                 f"module-wide _BRIDGES, where they outlive the test: build "
                 f"the printer with make_printer(monkeypatch=monkeypatch) or "
                 f"call use_bridges(monkeypatch, {{...}}) first")
    raise RuntimeError(error_str)


class BambuPrinter(MockPrinter):
    """
    conftest's MockPrinter with one reactor, one recording gcode, a :class:`BambuAFC`, the Klipper
    objects AFC drives (toolhead, gcode_move, print_stats, pause_resume, mcu and the ``extruder``),
    a merged fileconfig and klippy's section loading.
    """

    config_error = configparser.Error
    command_error = CommandError

    def __init__(self, now: float = 100.0, step: float = 0.0,
                 fabricate: bool = False) -> None:
        """
        :param now: the reactor's starting time
        :param step: seconds every monotonic() read advances the clock
        :param fabricate: build fabricated sections for real
        """
        reactor = FakeReactor(now=now, step=step)
        gcode = RecordingGcode()
        self.reactor, self.gcode = reactor, gcode
        self.fileconfig = configparser.RawConfigParser()
        self.access_tracking: Dict[Tuple[str, str], int] = {}
        self.fabricate = fabricate
        self.loaded: List[Tuple[str, Any]] = []
        th_extruder = FakeToolheadExtruder("extruder")
        self.toolhead = FakeToolhead(th_extruder)
        self.gcode_move = FakeGcodeMove(self.toolhead)
        self.print_stats = FakePrintStats()
        self.pause_resume = FakePauseResume()
        self.mcu = FakeMcu()
        afc = BambuAFC(self)
        # MockPrinter.__init__ installs its own reactor and gcode.
        super().__init__(afc=afc)
        self.reactor = self._reactor = reactor
        self.gcode = self._gcode = gcode
        self.afc = afc
        self._connected: List[Callable[[], Any]] = []
        for name, obj in (("toolhead", self.toolhead),
                          ("gcode_move", self.gcode_move),
                          ("print_stats", self.print_stats),
                          ("pause_resume", self.pause_resume),
                          ("mcu", self.mcu), ("extruder", th_extruder),
                          ("AFC_functions", afc.function)):
            self._objects[name] = obj

    def is_shutdown(self) -> bool:
        """:return bool: never shut down"""
        return False

    def add_object(self, name: str, obj: Any) -> None:
        """
        :param name: the object's name
        :param obj: the object; a second add of one name raises, as klippy
        """
        if name in self._objects:
            error_str = f"Printer object '{name}' already created"
            raise configparser.Error(error_str)
        self._objects[name] = obj

    def lookup_object(self, name: str,
                      default: Any = MockPrinter._NO_DEFAULT) -> Any:
        """
        :param name: the object's name
        :param default: returned when it is missing; without one, a missing
          ``AFC_led`` raises as klippy does
        :return Any: the object
        """
        if name.startswith("AFC_led ") and name not in self._objects:
            if default is MockPrinter._NO_DEFAULT:
                error_str = f"Unknown config object '{name}'"
                raise configparser.Error(error_str)
            return default
        if default is MockPrinter._NO_DEFAULT:
            default = None
        return super().lookup_object(name, default)

    def run_connect_handler(self, handler: Callable[[], Any]) -> bool:
        """Run one klippy:connect handler, unless it already ran."""
        if any(done == handler for done in self._connected):
            return False
        self._connected.append(handler)
        handler()
        return True

    def connect(self) -> None:
        """
        Send klippy:connect, as klippy does once every section is loaded: each unit's
        ``handle_connect`` wires its lanes.
        """
        self.send_event("klippy:connect")

    def send_event(self, event: str, *args: Any) -> None:
        """
        :param event: the event; klippy:connect runs each handler once, and
          klippy:ready needs an isolated bridge table for a Bambu handler
        :param args: the event's arguments
        """
        handlers = list(self._event_handlers.get(event, []))
        if event == "klippy:connect":
            for handler in handlers:
                self.run_connect_handler(handler)
            return
        if event == "klippy:ready" and _bambu_ready_owners(handlers):
            _require_isolated_bridges("klippy:ready")
        super().send_event(event, *args)

    def set_print_state(self, state: str) -> None:
        """
        :param state: print_stats state ("printing", "paused", "standby"...);
          pause_resume reads paused exactly when it is "paused"
        """
        self.lookup_object("print_stats").state = state
        self.lookup_object("pause_resume").is_paused = state == "paused"

    def load_object(self, config: Any, name: str,
                    default: Any = MockPrinter._NO_DEFAULT) -> Any:
        """
        :param config: the loading section's config
        :param name: the object or section to load
        :return Any: the object
        """
        if name in self._objects:
            return self._objects[name]
        fc = getattr(config, "fileconfig", None)
        if (name.startswith("gcode_macro ") and fc is not None
                and fc.has_section(name)):
            obj = FabricatedSection(name, dict(fc.items(name)))
            self._objects[name] = obj
            return obj
        if (getattr(config, "get_name", lambda: None)() == name
                and fc is not None and fc.has_section(name)):
            self.loaded.append((name, config))
            obj = self._build_section(fc, name)
            self._objects[name] = obj
            return obj
        return super().load_object(config, name, default)

    def _build_section(self, fc: Any, section: str) -> Any:
        """
        :param fc: the fileconfig holding the section
        :param section: the section to build
        :return Any: the object klippy would register under that name
        """
        keys = dict(fc.items(section))
        if not self.fabricate:
            return FabricatedSection(section, keys)
        kind, _sep, name = section.partition(" ")
        if kind == "AFC_BambuAMS":
            return afcBambuAMS(BambuConfig(section, self, keys, fc))
        if kind == "AFC_lane":
            return AFCLane(BambuConfig(section, self, keys, fc))
        if kind == "AFC_hub":
            return FakeHub(name, virtual=keys.get("switch_pin") == "virtual",
                           td1_bowden_length=float(
                               keys.get("td1_bowden_length", 850.0)))
        if kind == "AFC_buffer":
            buffer = FakeBuffer(name, keys=keys)
            self.afc.buffers[name] = buffer
            return buffer
        return FabricatedSection(section, keys)

    def add_section(self, section: str, values: Dict[str, Any]) -> None:
        """Record a section in the merged config, as a config file declares it."""
        if not self.fileconfig.has_section(section):
            self.fileconfig.add_section(section)
        for k, v in values.items():
            self.fileconfig.set(section, k, str(v))


def set_var_file(printer: BambuPrinter, path: str) -> None:
    """
    Give AFC a VarFile, and with it its config directory (``cfgloc``, where ConfigRewrite edits) and
    the afcFunction's ``AFC_auto_vars.cfg`` beside it, as AFC derives both from the VarFile.
    """
    afc = printer.afc
    afc.VarFile = path
    afc.cfgloc = _config_dir(path)
    if isinstance(afc.function, afcFunction):
        afc.function.auto_var_file = pathlib.Path(path).parent.joinpath(
            "AFC_auto_vars.cfg")


def make_printer(now: float = 100.0, step: float = 0.0,
                 fabricate: bool = False, extruder: Optional[str] = "extruder",
                 print_state: Optional[str] = None,
                 var_file: Optional[str] = None,
                 monkeypatch: Optional[pytest.MonkeyPatch] = None
                 ) -> BambuPrinter:
    """
    A printer with its clock, gcode, AFC core, toolhead extruder, an idle ``idle_timeout`` and a
    real afcFunction.
    """
    printer = BambuPrinter(now=now, step=step, fabricate=fabricate)
    printer.add_object("idle_timeout", FakeIdleTimeout())
    function = afcFunction(BambuConfig("AFC_functions", printer))
    printer._objects["AFC_functions"] = function
    printer.afc.function = function
    printer.run_connect_handler(function.handle_connect)
    if var_file is not None:
        set_var_file(printer, var_file)
    if extruder:
        add_extruder(printer, extruder)
    if print_state is not None:
        printer.set_print_state(print_state)
    if monkeypatch is not None:
        use_bridges(monkeypatch, {})
        patch_module_time(monkeypatch, printer.reactor)
    return printer


# ── hub, extruder, buffer, LED, transports ───────────────────────────────────

class FakeRunoutHelper:
    """A filament switch's runout helper, scriptable by read count."""

    def __init__(self, present: bool = False,
                 after: Optional[int] = None) -> None:
        """
        :param present: the state every read returns
        :param after: instead, read False this many times, then True
        """
        self.present = present
        self.after = after
        self.reads = 0
        self.sensor_enabled = True

    @property
    def filament_present(self) -> bool:
        """:return bool: the switch state for this read"""
        self.reads += 1
        if self.after is not None:
            return self.reads > self.after
        return self.present


class FakeSwitch:
    """A filament switch sensor: just its runout helper."""

    def __init__(self, helper: FakeRunoutHelper) -> None:
        """:param helper: its runout helper"""
        self.runout_helper = helper


class FakeExtruder:
    """AFC_extruder as Bambu lanes and AFC's load and unload use it."""

    def __init__(self, name: str = "extruder", tool_start: str = "tool_pin",
                 present: bool = False, after: Optional[int] = None, *,
                 tool_stn: float = 72.0, tool_stn_unload: float = 100.0,
                 tool_load_speed: float = 25.0,
                 tool_unload_speed: float = 25.0,
                 toolhead_extruder: Optional[FakeToolheadExtruder] = None
                 ) -> None:
        """
        :param name: the extruder's name
        :param tool_start: its tool_start pin ("buffer" needs a buffer)
        :param present: the toolhead sensor's state
        :param after: the sensor reads False this many times, then True
        :param tool_stn: mm from the toolhead sensor to the nozzle
        :param tool_stn_unload: mm retracted to unload the toolhead
        :param tool_load_speed: mm/s of the tool_stn advance
        :param tool_unload_speed: mm/s of the unload retracts
        :param toolhead_extruder: Klipper's extruder object; a new
          :class:`FakeToolheadExtruder` named ``name`` when None
        """
        self.name = name
        self.th_extruder_name = name
        self.tool_start = tool_start
        self.tool_start_state = False
        self.tool_end_state = False
        self.buffer_name: Optional[str] = None
        self.lanes: Dict[str, Any] = {}
        self.lane_loaded: Optional[str] = None
        self.fila_tool_start = FakeSwitch(FakeRunoutHelper(present, after))
        self.fila_tool_end: Optional[FakeSwitch] = None
        self.tool_stn = tool_stn
        self.tool_stn_unload = tool_stn_unload
        self.tool_load_speed = tool_load_speed
        self.tool_unload_speed = tool_unload_speed
        self.toolhead_extruder = (toolhead_extruder
                                  or FakeToolheadExtruder(name))
        self.tc_unit_obj: Any = None
        self.tc_lane: Any = None
        self.no_lanes = False
        self.check_lanes_calls = 0
        #: Colours set_status_led was given, in order.
        self.status_leds: List[Any] = []
        #: (state, quiet) of every set_print_leds, in order.
        self.print_leds: List[Tuple[int, bool]] = []

    @property
    def sensor(self) -> FakeRunoutHelper:
        """:return FakeRunoutHelper: the toolhead sensor"""
        return self.fila_tool_start.runout_helper

    def check_lanes(self) -> None:
        """Count the lane check AFCLane runs on connect."""
        self.check_lanes_calls += 1

    def on_shuttle(self) -> bool:
        """:return bool: always on the shuttle (single toolhead)"""
        return True

    def is_standalone(self) -> bool:
        """:return bool: ``no_lanes``"""
        return self.no_lanes

    def set_status_led(self, color: Any) -> None:
        """:param color: recorded in status_leds"""
        self.status_leds.append(color)

    def set_print_leds(self, state: int = 1, quiet: bool = False) -> None:
        """
        :param state: 1 on, 0 off
        :param quiet: suppress the console line; both recorded in print_leds
        """
        self.print_leds.append((state, quiet))


def add_extruder(printer: BambuPrinter, name: str = "extruder",
                 **options: Any) -> FakeExtruder:
    """
    An [AFC_extruder <name>] on ``printer``: registered as ``AFC_extruder <name>`` and in
    ``afc.tools``, its Klipper extruder registered as ``<name>``.
    """
    th_extruder = printer._objects.get(name)
    if not isinstance(th_extruder, FakeToolheadExtruder):
        th_extruder = FakeToolheadExtruder(name)
        printer._objects[name] = th_extruder
    ext = FakeExtruder(name, toolhead_extruder=th_extruder, **options)
    printer.add_object(f"AFC_extruder {name}", ext)
    printer.afc.tools[name] = ext
    return ext


class FakeHub:
    """AFC_hub as a Bambu lane connects to it."""

    def __init__(self, name: str, virtual: bool = True,
                 td1_bowden_length: float = 850.0,
                 afc_bowden_length: float = 2100.0) -> None:
        """
        :param name: the hub's name
        :param virtual: True for the fabricated virtual hub (no switch)
        :param td1_bowden_length: inherited by lanes with none of their own
        :param afc_bowden_length: the hub's bowden
        """
        self.name = name
        self.lanes: Dict[str, Any] = {}
        self.switch_pin = "virtual" if virtual else "hub_pin"
        self._virtual = virtual
        self.state = False
        self.td1_bowden_length = td1_bowden_length
        self.afc_bowden_length = afc_bowden_length
        self.afc_unload_bowden_length = afc_bowden_length

    def is_virtual_pin(self) -> bool:
        """:return bool: whether the hub has no physical switch"""
        return self._virtual


class FakeBuffer:
    """AFC_buffer as lanes use it: settable states, recorded enable/disable."""

    def __init__(self, name: str, state: str = "Unknown",
                 keys: Optional[Dict[str, str]] = None) -> None:
        """
        :param name: the buffer's name (AFC_buffer <name>)
        :param state: what buffer_status returns
        :param keys: its config options, when fabricated
        """
        self.name = name
        self.section = f"AFC_buffer {name}"
        self.keys: Dict[str, str] = dict(keys or {})
        self.lanes: Dict[str, Any] = {}
        self.advance_pin: Optional[str] = None
        self.advance_state = False
        self.trailing_state = False
        self.state = state
        self.calls: List[Tuple[str, tuple]] = []

    def buffer_status(self) -> str:
        """:return str: ``state``"""
        return self.state

    def enable_buffer(self, lane: Any) -> None:
        """:param lane: the lane the buffer now serves"""
        self.calls.append(("enable_buffer", (lane,)))

    def disable_buffer(self) -> None:
        """Record the buffer being disabled."""
        self.calls.append(("disable_buffer", ()))

    def disable_fault_sensitivity(self) -> None:
        """Record fault detection being disabled."""
        self.calls.append(("disable_fault_sensitivity", ()))

    def restore_fault_sensitivity(self) -> None:
        """Record fault detection being restored."""
        self.calls.append(("restore_fault_sensitivity", ()))


def add_buffer(printer: BambuPrinter, name: str, **options: Any
               ) -> FakeBuffer:
    """
    An [AFC_buffer <name>] on ``printer``: registered as ``AFC_buffer <name>`` and in
    ``afc.buffers``.
    """
    buffer = FakeBuffer(name, **options)
    printer.add_object(buffer.section, buffer)
    printer.afc.buffers[name] = buffer
    return buffer


def add_led(printer: BambuPrinter, name: str) -> FakeLed:
    """
    An [AFC_led <name>] on ``printer``, which the afcFunction's afc_led writes to for a
    ``led_index`` such as ``<name>:1-4``.
    """
    led = FakeLed(name)
    printer.add_object(f"AFC_led {name}", led)
    return led


class FakeSerial:
    """A pyserial port: records writes, replays ``lines`` to reads."""

    def __init__(self, lines: Sequence[bytes] = (),
                 fail_read: Optional[BaseException] = None,
                 fail_write: Optional[BaseException] = None) -> None:
        """
        :param lines: what readline/read hand back, in order
        :param fail_read: raised by every read (a port that died)
        :param fail_write: raised by every write
        """
        self.lines: List[bytes] = list(lines)
        self.written: List[bytes] = []
        self.closed = False
        self.fail_read = fail_read
        self.fail_write = fail_write

    def write(self, data: bytes) -> int:
        """:return int: bytes written"""
        if self.fail_write is not None:
            raise self.fail_write
        self.written.append(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        """:return bytes: the next line, or b"" (a timeout)"""
        if self.fail_read is not None:
            raise self.fail_read
        return self.lines.pop(0) if self.lines else b""

    def readline(self) -> bytes:
        """:return bytes: the next line, or b"" (a timeout)"""
        return self.read()

    def reset_input_buffer(self) -> None:
        """Nothing buffered to drop."""

    def close(self) -> None:
        """Mark the port closed."""
        self.closed = True


class FakeSocket:
    """A TCP socket replaying ``script`` to recv and recording sends."""

    def __init__(self, script: Sequence[bytes] = (),
                 clock: Optional[FakeReactor] = None) -> None:
        """
        :param script: the chunks recv returns, in order
        :param clock: the clock an empty recv advances
        """
        self.inbox: List[bytes] = list(script)
        self.sent = b""
        self.timeout: Optional[float] = None
        self.options: List[tuple] = []
        self.closed = False
        self.clock = clock

    def settimeout(self, timeout: Optional[float]) -> None:
        """:param timeout: recorded"""
        self.timeout = timeout

    def setsockopt(self, *args: Any) -> None:
        """Record a socket option."""
        self.options.append(args)

    def recv(self, size: int) -> bytes:
        """:return bytes: the next chunk; raises socket.timeout when empty"""
        if not self.inbox:
            if self.clock is not None:
                self.clock.advance(self.timeout or 0.0)
            raise socket.timeout()
        return self.inbox.pop(0)

    def sendall(self, data: bytes) -> None:
        """:param data: appended to ``sent``"""
        self.sent += bytes(data)

    def send(self, data: bytes) -> int:
        """:return int: bytes sent (all of them)"""
        self.sent += bytes(data)
        return len(data)

    def close(self) -> None:
        """Mark the socket closed."""
        self.closed = True


# ── the bridge a unit talks to ───────────────────────────────────────────────

class FakeBridge:
    """The BambuBridge surface afcBambuAMS uses, with recorded sends and settable tables."""

    def __init__(self, *, status: Optional[dict] = None,
                 uids: Sequence[str] = (), connected: bool = True,
                 chmb: Optional[Dict[int, dict]] = None,
                 cap: Optional[Dict[int, dict]] = None,
                 cali: Optional[Dict[int, dict]] = None,
                 finish: Tuple[int, bool, str] = (0, False, ""),
                 switch: Tuple[int, str] = (0, ""),
                 assist: int = 0,
                 fault: Tuple[int, str, float] = (0, "", 0.0),
                 online: Optional[Sequence[bool]] = None,
                 htmask: int = 0,
                 info: Optional[dict] = None,
                 on_send: Optional[Callable[["FakeBridge", dict], None]] = None,
                 raising: Optional[Dict[str, BaseException]] = None) -> None:
        """
        :param status: what latest_status() returns
        :param uids: chain index -> unit UID (chain_uids)
        :param connected: is_connected(), and whether ``_serial`` is set
        :param chmb: chain index -> [AMS_CHMB] drying record
        :param cap: device address -> capacity measurement record
        :param cali: chain index -> HT calibration verdict
        :param finish: last_finish() (sequence, ok, text)
        :param switch: last_switch_finish() (sequence, text)
        :param assist: last_assist_done() sequence
        :param fault: last_fault() (sequence, text, motor current)
        :param online: chain index -> online flag; builds ``status`` as
          ``{"units": [{"n": i, "online": flag}, ...]}`` when status is None
        :param htmask: chain index bits that are AMS HTs (chain_diag()[0])
        :param info: the bridge's last info reply (``_info``: chip,
          firmware...), None until it answered one
        :param on_send: called as ``on_send(bridge, frame)`` after each send,
          to script the unit's answer (bump ``finish``, set a table...)
        :param raising: method name -> the exception it raises
        """
        self.sent: List[dict] = []
        if status is None and online is not None:
            status = {"units": [{"n": i, "online": bool(b)}
                                for i, b in enumerate(online)]}
        self.status = status
        self.on_send = on_send
        self.uids: List[str] = list(uids)
        self.connected = connected
        self._serial: Optional[Any] = object() if connected else None
        self._info: Optional[dict] = dict(info) if info is not None else None
        self._chmb_by_unit: Dict[int, dict] = dict(chmb or {})
        self.cap_by_addr: Dict[int, dict] = dict(cap or {})
        self.cali_by_unit: Dict[int, dict] = dict(cali or {})
        self.dry_err_by_unit: Dict[int, str] = {}
        self.finish = finish
        self.switch = switch
        self.assist = assist
        #: buff_refill_seq() and last_buff_refill().
        self.refill_seq = 0
        self.refill: Optional[tuple] = None
        self.fault = fault
        self.fault_by_unit: Dict[int, Tuple[int, str, float]] = {}
        self.tray_release: Dict[Optional[int], Tuple[int, Optional[int]]] = {}
        self.dw: Dict[Optional[int], tuple] = {}
        self.tube: Dict[Optional[int], float] = {}
        #: Stamps by device address (None = bridge-wide), as the real _rfid_*_by_addr tables.
        self.rfid_ok: Dict[Optional[int], float] = {}
        self.rfid_end: Dict[Optional[int], float] = {}
        self.rfid_foreign: Dict[Optional[int], float] = {}
        self.gave_up: Dict[Optional[int], float] = {}
        self.terminal: Dict[int, float] = {}
        self.calibrating: Dict[int, bool] = {}
        self.err_code: Tuple[Optional[int], float] = (None, 0.0)
        self.down_t: Optional[float] = None if connected else 0.0
        self.silent: Optional[float] = None
        self.mcaddr: Optional[List[int]] = None
        self.mcaddr_acks: Dict[int, int] = {}
        self.diag: tuple = (htmask, "", (-1, 0, 0))
        self.dialect: Tuple[int, List[int]] = (0, [])
        self.chain_seq = 0
        self.rebooted = False
        self.listeners: List[Callable[[dict], None]] = []
        self.reconnect_listeners: List[Callable[[], None]] = []
        self.active_unit: Optional[int] = None
        self.claimed_bus: Optional[str] = None
        self.released_bus: List[str] = []
        self.started = False
        self.stopped = False
        self.raw_writes: List[bytes] = []
        self.narration_log: Optional[Tuple[str, str]] = None
        self.raising: Dict[str, BaseException] = dict(raising or {})
        self._lock = threading.Lock()
        self._last_fw: Optional[tuple] = None
        self._fw_raw = False
        self._last_idsave: Optional[tuple] = None
        self._last_raw_reply = ""
        self._TRAY_NOW_RE = bridge_mod._TRAY_NOW_RE
        #: Each hold() asked for, and whether the link is let go now.
        self.holds: List[float] = []
        self.held_now = False

    def _check(self, name: str) -> None:
        """
        :param name: the method being called
        :raises BaseException: what ``raising`` names for it
        """
        exc = self.raising.get(name)
        if exc is not None:
            raise exc

    @staticmethod
    def _stamp(table: Dict[Optional[int], float],
               addr: Optional[int]) -> Optional[float]:
        """
        :param table: stamps by device address (None = bridge-wide)
        :param addr: the address asked about
        :return Optional[float]: its stamp, else the bridge-wide one
        """
        if addr is not None and addr in table:
            return table[addr]
        return table.get(None)

    # writes
    def send(self, obj: dict) -> None:
        """:param obj: recorded in ``sent``"""
        self._check("send")
        self.sent.append(obj)
        if self.on_send is not None:
            self.on_send(self, obj)

    def write_raw(self, data: bytes) -> bool:
        """:return bool: True; the bytes are kept in ``raw_writes``"""
        self._check("write_raw")
        self.raw_writes.append(data)
        return True

    def request_info(self) -> None:
        """Send the info request, as the real one does."""
        self.send({"cmd": "info"})

    def start(self, defer_open: bool = False) -> None:
        """Mark started; no threads."""
        self.started = True

    def stop(self) -> None:
        """Mark stopped."""
        self.stopped = True

    def set_narration_log(self, log_dir: str, tag: str = "",
                          max_bytes: int = 10 * 1024 * 1024) -> bool:
        """:return bool: True; the request is kept in ``narration_log``"""
        self.narration_log = (log_dir, tag)
        return True

    def hold(self, seconds: float) -> None:
        """:param seconds: kept in ``holds``; the link is let go while above zero"""
        self._check("hold")
        self.holds.append(seconds)
        self.held_now = seconds > 0

    def held(self) -> bool:
        """:return bool: ``held_now``"""
        self._check("held")
        return self.held_now

    # identity
    def chip(self) -> Optional[str]:
        """:return Optional[str]: ``_info``'s chip, None when it said none"""
        with self._lock:
            value = (self._info or {}).get("chip")
        return str(value) if value else None

    # listeners
    def add_listener(self, cb: Callable[[dict], None]) -> None:
        """:param cb: called by push() with each status frame"""
        if cb not in self.listeners:
            self.listeners.append(cb)

    def remove_listener(self, cb: Callable[[dict], None]) -> None:
        """:param cb: dropped if present"""
        if cb in self.listeners:
            self.listeners.remove(cb)

    def add_reconnect_listener(self, cb: Callable[[], None]) -> None:
        """:param cb: called by reconnect()"""
        if cb not in self.reconnect_listeners:
            self.reconnect_listeners.append(cb)

    def remove_reconnect_listener(self, cb: Callable[[], None]) -> None:
        """:param cb: dropped if present"""
        if cb in self.reconnect_listeners:
            self.reconnect_listeners.remove(cb)

    def replay_reconnect_listeners(self) -> None:
        """Call every reconnect listener now."""
        for cb in list(self.reconnect_listeners):
            cb()

    def push(self, status: dict) -> None:
        """
        Deliver a status frame: it becomes latest_status() and reaches every listener, as the
        reader's reactor hop does.
        """
        self.status = status
        for cb in list(self.listeners):
            cb(dict(status))

    def reconnect(self) -> None:
        """A reconnect: every reconnect listener runs."""
        self.replay_reconnect_listeners()

    def consume_reboot(self) -> bool:
        """:return bool: ``rebooted``, once"""
        was, self.rebooted = self.rebooted, False
        return was

    # link
    def latest_status(self) -> Optional[dict]:
        """:return Optional[dict]: a copy of ``status``"""
        self._check("latest_status")
        return dict(self.status) if self.status is not None else None

    def is_connected(self) -> bool:
        """:return bool: ``connected``"""
        return self.connected

    def link(self) -> Optional[str]:
        """:return Optional[str]: "usb" while connected (the fake port is serial), else None"""
        self._check("link")
        return "usb" if self.connected else None

    def down_since(self) -> Optional[float]:
        """:return Optional[float]: when the link went down, or None"""
        self._check("down_since")
        return self.down_t

    def silent_for(self) -> Optional[float]:
        """:return Optional[float]: seconds since the last frame, or None"""
        return self.silent

    def set_active_unit(self, unit: Optional[int]) -> None:
        """:param unit: kept in ``active_unit``"""
        self.active_unit = unit

    # chain
    def chain_uids(self) -> List[str]:
        """:return list: ``uids``"""
        self._check("chain_uids")
        return list(self.uids)

    def chain_snapshot(self) -> Dict[str, Any]:
        """:return dict: seq, uids, htmask, a2mask, a2asks, as the real one"""
        return {"seq": self.chain_seq, "uids": list(self.uids),
                "htmask": self.diag[0], "a2mask": self.dialect[0],
                "a2asks": list(self.dialect[1])}

    def chain_diag(self) -> tuple:
        """:return tuple: (htmask, text, capdiag)"""
        return self.diag

    def chain_dialect(self) -> Tuple[int, List[int]]:
        """:return tuple: (AMS 2 mask, units asked for their dialect)"""
        return self.dialect[0], list(self.dialect[1])

    def chain_mcaddr(self) -> Optional[List[int]]:
        """:return Optional[list]: ``mcaddr``"""
        return self.mcaddr

    def mcaddr_ack(self, unit: int) -> Optional[int]:
        """:return Optional[int]: the acked address for ``unit``"""
        return self.mcaddr_acks.get(unit)

    # bus
    def try_claim_bus(self, owner: str, now: float) -> bool:
        """:return bool: free or already ours"""
        if self.claimed_bus not in (None, owner):
            return False
        self.claimed_bus = owner
        return True

    def release_bus(self, owner: str) -> None:
        """:param owner: released if it holds the bus"""
        self.released_bus.append(owner)
        if self.claimed_bus == owner:
            self.claimed_bus = None

    def bus_owner(self) -> Optional[str]:
        """:return Optional[str]: who holds the bus"""
        return self.claimed_bus

    # motion
    def last_finish(self) -> Tuple[int, bool, str]:
        """:return tuple: (sequence, ok, text)"""
        self._check("last_finish")
        return self.finish

    def last_switch_finish(self) -> Tuple[int, str]:
        """:return tuple: (sequence, text)"""
        return self.switch

    def last_assist_done(self) -> int:
        """:return int: the assist-finish sequence"""
        return self.assist

    def buff_refill_seq(self) -> int:
        """:return int: the buffer refill count"""
        return self.refill_seq

    def last_buff_refill(self) -> Optional[tuple]:
        """:return Optional[tuple]: the last buffer refill"""
        return self.refill

    def last_tray_release(self, unit: Optional[int] = None
                          ) -> Tuple[int, Optional[int]]:
        """:return tuple: (sequence, released tray)"""
        return self.tray_release.get(unit, self.tray_release.get(None,
                                                                 (0, None)))

    def last_fault(self, unit: Optional[int] = None
                   ) -> Tuple[int, str, float]:
        """:return tuple: (sequence, text, motor current)"""
        self._check("last_fault")
        if unit is not None and unit in self.fault_by_unit:
            return self.fault_by_unit[unit]
        return self.fault

    def last_err_code(self, unit: Optional[int] = None
                      ) -> Tuple[Optional[int], float]:
        """:return tuple: (error level, when)"""
        return self.err_code

    def dw_len(self, unit: Optional[int] = None) -> tuple:
        """:return tuple: (dw_len mm, times seen, device address)"""
        return self.dw.get(unit, self.dw.get(None, (None, 0, None)))

    def tube_len(self, addr: Optional[int] = None,
                 unit: Optional[int] = None) -> Optional[float]:
        """:return Optional[float]: the calibrated PTFE path, mm"""
        if unit is not None and unit in self.tube:
            return self.tube[unit]
        return self.tube.get(None)

    # tags and measurement
    def rfid_read_succeeded_since(self, since: Optional[float],
                                  addr: Optional[int] = None) -> bool:
        """:return bool: a tag read at or after ``since``"""
        self._check("rfid_read_succeeded_since")
        t = self._stamp(self.rfid_ok, addr)
        return t is not None and since is not None and t >= since

    def rfid_cycle_ended_since(self, since: Optional[float],
                               addr: Optional[int] = None) -> bool:
        """:return bool: a scan cycle ended at or after ``since``"""
        self._check("rfid_cycle_ended_since")
        t = self._stamp(self.rfid_end, addr)
        return t is not None and since is not None and t >= since

    def rfid_foreign_tag_since(self, since: Optional[float],
                               addr: Optional[int] = None) -> bool:
        """:return bool: a foreign tag refused at or after ``since``"""
        t = self._stamp(self.rfid_foreign, addr)
        return t is not None and since is not None and t >= since

    def gave_up_since(self, since: Optional[float],
                      addr: Optional[int] = None) -> bool:
        """:return bool: the unit gave up at or after ``since``"""
        self._check("gave_up_since")
        if since is None:
            return False
        t = self._stamp(self.gave_up, addr)
        return t is not None and t >= since

    def last_scan_end(self) -> Optional[float]:
        """:return Optional[float]: the bridge-wide scan-cycle end stamp"""
        return self.rfid_end.get(None)

    def last_terminal(self, addr: Optional[int]) -> Optional[float]:
        """:return Optional[float]: when ``addr`` last said it finished"""
        return self.terminal.get(addr) if addr is not None else None

    def cap_calibrating(self, addr: Optional[int]) -> bool:
        """:return bool: a measurement is live on ``addr``"""
        return bool(self.calibrating.get(addr)) if addr is not None else False

    def last_cap_measure(self, addr: Optional[int]
                         ) -> Optional[Dict[str, Any]]:
        """:return Optional[dict]: a copy of the record for ``addr``"""
        self._check("last_cap_measure")
        rec = self.cap_by_addr.get(int(addr)) if addr is not None else None
        return dict(rec) if rec else None

    def last_ht_cali(self, unit: Optional[int]) -> Optional[Dict[str, Any]]:
        """:return Optional[dict]: a copy of the verdict for ``unit``"""
        rec = self.cali_by_unit.get(unit) if unit is not None else None
        return dict(rec) if rec else None

    # drying
    def clear_dry_error(self, unit: Optional[int]) -> None:
        """:param unit: its refusal is dropped"""
        self.dry_err_by_unit.pop(unit, None)  # type: ignore[arg-type]

    def last_dry_error(self, unit: Optional[int]) -> Optional[str]:
        """:return Optional[str]: the last refusal for ``unit``"""
        return self.dry_err_by_unit.get(unit)  # type: ignore[arg-type]


def use_bridges(monkeypatch: pytest.MonkeyPatch,
                bridges: Dict[str, Any]) -> Dict[str, Any]:
    """
    Make ``bridges`` (serial port -> bridge) the module's live bridge table for this test, which
    ready, claim and the BridgeBox master look bridges up in and store new ones into.
    """
    table = dict(bridges)
    monkeypatch.setattr(bridge_mod, "_BRIDGES", table, raising=False)
    return table


def make_bambu_bridge(monkeypatch: pytest.MonkeyPatch, *,
                      printer: Optional[BambuPrinter] = None,
                      reactor: Optional[FakeReactor] = None,
                      logger: Optional[BambuLogger] = None,
                      listener: Optional[Callable[[dict], None]] = None,
                      connected: bool = True,
                      name: str = "bridge",
                      narration: bool = False) -> BambuBridge:
    """A real BambuBridge on a fake serial port, without its threads."""
    if printer is not None:
        reactor = reactor or printer.reactor
        logger = logger or printer.afc.logger
    reactor = reactor or FakeReactor()
    patch_module_time(monkeypatch, reactor)
    bridge = BambuBridge(FakeSerial, reactor, logger or BambuLogger())
    bridge.name = name
    if narration:
        bridge._nar_lg = BambuLogger()
    if listener is not None:
        bridge.add_listener(listener)
    if connected:
        bridge._serial = bridge._serial_factory()
        bridge._down_t = None
        bridge._mark_connected()
    return bridge


def bridge_sent(bridge: BambuBridge) -> List[dict]:
    """Drain the commands a real bridge queued for its writer thread."""
    out: List[dict] = []
    while not bridge._wq.empty():
        out.append(json.loads(bridge._wq.get_nowait().decode()))
    return out


# ── slot records ─────────────────────────────────────────────────────────────

def wire_slot(index: int, *, present: bool = True,
              material: Optional[str] = None, color: Optional[str] = None,
              **fields: Any) -> Dict[str, Any]:
    """One slot of a bridge status frame, in wire keys."""
    slot: Dict[str, Any] = {"i": index, "present": present}
    if material is not None:
        slot["material"] = material
    if color is not None:
        slot["color"] = color
    slot.update(fields)
    return slot


def slot_info(index: int, *, present: bool = True,
              material: Optional[str] = None, color: Optional[str] = None,
              **fields: Any) -> Dict[str, Any]:
    """A unit's ``_slots`` entry: :func:`wire_slot` through the real ``bridge_slot_to_info``."""
    return bridge_slot_to_info(wire_slot(index, present=present,
                                         material=material, color=color,
                                         **fields))


# ── lanes ────────────────────────────────────────────────────────────────────

@dataclass
class LaneSpec:
    """A lane for :func:`make_bambu_unit`: its bay, its options and the state PREP leaves."""

    name: str
    slot: int
    prep: bool = False
    load: bool = False
    loaded_to_hub: bool = False
    tool_loaded: bool = False
    status: Optional[str] = None
    material: Optional[str] = None
    color: str = ""
    spool_id: Optional[int] = None
    weight: float = 0.0
    values: Dict[str, Any] = field(default_factory=dict)
    extras: Dict[str, Any] = field(default_factory=dict)


def _lane_values(unit_name: str, slot: int,
                 values: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """
    :param unit_name: the lane's unit
    :param slot: its bay, 0-based
    :param values: further options, which win
    :return dict: the [AFC_lane] options, ``unit`` as ``<unit>:<slot + 1>``
    """
    out: Dict[str, Any] = {"unit": f"{unit_name}:{slot + 1}"}
    out.update(values or {})
    return out


def make_afc_lane(name: str, unit_name: str, slot: int, *,
                  printer: Optional[BambuPrinter] = None,
                  values: Optional[Dict[str, Any]] = None) -> AFCLane:
    """
    A real AFCLane through ``AFCLane.__init__``, registered on the printer as ``AFC_lane <name>``
    (klippy's load_object does), not connected to a unit.
    """
    printer = printer or make_printer()
    section = f"AFC_lane {name}"
    opts = _lane_values(unit_name, slot, values)
    printer.add_section(section, opts)
    lane = AFCLane(BambuConfig(section, printer, opts))
    printer.add_object(section, lane)
    return lane


def add_bambu_lane(unit: afcBambuAMS, name: str, slot: int, *,
                   prep: bool = False, load: bool = False,
                   loaded_to_hub: bool = False, tool_loaded: bool = False,
                   status: Optional[str] = None,
                   material: Optional[str] = None, color: str = "",
                   spool_id: Optional[int] = None, weight: float = 0.0,
                   values: Optional[Dict[str, Any]] = None,
                   **extras: Any) -> AFCLane:
    """A real AFCLane on ``unit``'s bay ``slot``, connected and mapped."""
    lane = make_afc_lane(name, unit.name, slot, printer=unit.printer,
                         values=values)
    lane.handle_unit_connect(unit)
    unit._slot_map = build_slot_map(unit.lanes, unit.SLOTS_PER_UNIT)
    _apply_lane_spec(lane, LaneSpec(
        name, slot, prep=prep, load=load, loaded_to_hub=loaded_to_hub,
        tool_loaded=tool_loaded, status=status, material=material,
        color=color, spool_id=spool_id, weight=weight,
        values=dict(values or {}), extras=dict(extras)))
    return lane


# ── the unit ─────────────────────────────────────────────────────────────────

def unit_config(name: str = "Bambu_AMS_1", *,
                printer: Optional[BambuPrinter] = None,
                values: Optional[Dict[str, Any]] = None) -> BambuConfig:
    """The [AFC_BambuAMS <name>] section afcBambuAMS.__init__ reads."""
    printer = printer or make_printer()
    opts: Dict[str, Any] = {"serial_port": "/dev/serial/by-id/usb-bambu-if00",
                            "hub": f"{name}_hub", "extruder": "extruder"}
    opts.update(values or {})
    opts = {k: v for k, v in opts.items() if v is not None}
    section = f"AFC_BambuAMS {name}"
    printer.add_section(section, opts)
    return BambuConfig(section, printer, opts)


def make_bambu_unit(name: str = "Bambu_AMS_1", *,
                    printer: Optional[BambuPrinter] = None,
                    model: str = "ams2", ams_index: int = 0,
                    values: Optional[Dict[str, Any]] = None,
                    bridge: Optional[Any] = None, offline: bool = False,
                    lanes: Sequence[Union[str, LaneSpec]] = (),
                    slots: Optional[Sequence[Dict[str, Any]]] = None,
                    inserted: Sequence[int] = (),
                    hub: bool = True, virtual_hub: bool = True,
                    connect: bool = True, prep_seen: bool = False,
                    scan_primed: bool = False, drying: bool = False,
                    id_resolved: bool = True) -> afcBambuAMS:
    """An afcBambuAMS through its real ``__init__``, connected as klippy does."""
    printer = printer or make_printer()
    opts: Dict[str, Any] = {"ams_model": model, "ams_index": ams_index}
    if not hub:
        opts["hub"] = None
    opts.update(values or {})
    cfg = unit_config(name, printer=printer, values=opts)
    if hub and cfg.get("hub", None):
        hub_name = cfg.get("hub")
        printer._objects.setdefault(f"AFC_hub {hub_name}",
                                    FakeHub(hub_name, virtual=virtual_hub))
    unit = afcBambuAMS(cfg)
    printer.add_object(cfg.get_name(), unit)
    specs = [LaneSpec(s, i) if isinstance(s, str) else s
             for i, s in enumerate(lanes)]
    built = []
    for spec in specs:
        built.append((spec, make_afc_lane(spec.name, name, spec.slot,
                                          printer=printer,
                                          values=spec.values)))
    if connect:
        printer.run_connect_handler(unit.handle_connect)
    if unit.lanes:
        unit._slot_map = build_slot_map(unit.lanes, unit.SLOTS_PER_UNIT)
    for lane in unit.lanes.values():
        if afcBambuAMS._is_virtual_hub(lane):
            lane._load_state = False
    for spec, lane in built:
        _apply_lane_spec(lane, spec)
    if not offline:
        attached = attach_bridge(unit, bridge if bridge is not None
                                 else FakeBridge())
        if bridges_isolated() and unit.serial_port:
            live_bridges().setdefault(unit.serial_port, attached)
    for i, info in enumerate(slots or ()):
        unit._slots[i] = dict(info)
        if (info.get("present") and i not in inserted
                and i < len(unit._prev_present)):
            unit._prev_present[i] = True
            unit._present_seen.add(i)
    unit._prep_seen = prep_seen
    unit._scan_primed = scan_primed
    unit._drying = drying
    unit._id_resolved = id_resolved
    return unit


def _apply_lane_spec(lane: AFCLane, spec: LaneSpec) -> None:
    """
    :param lane: a connected lane
    :param spec: the state to leave on it
    """
    lane.prep_state = spec.prep
    lane._load_state = spec.load
    lane.loaded_to_hub = spec.loaded_to_hub
    if spec.status is not None:
        lane.status = spec.status
    elif spec.loaded_to_hub:
        lane.status = AFCLaneState.LOADED
    if spec.tool_loaded:
        lane.tool_loaded = True
        lane.status = spec.status or AFCLaneState.TOOLED
        if lane.extruder_obj is not None:
            lane.extruder_obj.lane_loaded = lane.name
    lane.material = spec.material
    lane.color = spec.color
    lane.spool_id = spec.spool_id
    lane.weight = spec.weight
    for key, value in spec.extras.items():
        setattr(lane, key, value)


def attach_bridge(unit: afcBambuAMS, bridge: Any) -> Any:
    """
    Put ``unit`` on ``bridge`` as ready and claim do: the bridge, its status listener and its
    reconnect listener.
    """
    unit._bridge = bridge
    bridge.add_listener(unit._on_status)
    bridge.add_reconnect_listener(unit._on_bridge_reconnect)
    return bridge


# ── Spoolman ─────────────────────────────────────────────────────────────────

def make_spoolman_section(printer: BambuPrinter,
                          enabled: bool = True) -> AFC_BambuAMS_RFID:
    """The [AFC_BambuAMS_rfid] object through its real ``__init__``, registered where units look it up."""
    cfg = BambuConfig("AFC_BambuAMS_rfid", printer, {"enabled": enabled})
    section = AFC_BambuAMS_RFID(cfg)
    printer.add_object("AFC_BambuAMS_rfid", section)
    return section


class FakeSpoolmanClient:
    """
    AFC's SpoolmanClient as the Bambu delegate calls it, over a table of spools; every write is
    recorded in ``.calls``.
    """

    def __init__(self, spools: Sequence[Dict[str, Any]] = (),
                 reachable: bool = True) -> None:
        """
        :param spools: Spoolman's spool records (``id``, ``extra``...)
        :param reachable: what reachable() answers
        """
        self.spools: List[Dict[str, Any]] = [dict(sp) for sp in spools]
        self.is_reachable = reachable
        self.calls: List[Tuple[str, tuple]] = []

    def search_spools(self) -> List[Dict[str, Any]]:
        """:return list: every spool"""
        return [dict(sp) for sp in self.spools]

    def get_spool(self, spool_id: int) -> Optional[Dict[str, Any]]:
        """:return Optional[dict]: the spool with that id"""
        return next((dict(sp) for sp in self.spools
                     if sp.get("id") == spool_id), None)

    def reachable(self) -> bool:
        """:return bool: whether Spoolman answers"""
        return self.is_reachable

    def set_remaining_weight(self, spool_id: int, grams: float) -> None:
        """Record a weight write."""
        self.calls.append(("set_remaining_weight", (spool_id, grams)))

    def write_tray_uid(self, spool_id: int, tray_uid: str) -> None:
        """Record a roll-identity write."""
        self.calls.append(("write_tray_uid", (spool_id, tray_uid)))

    def write_spool_metadata(self, spool_id: int, **fields: Any) -> None:
        """Record a metadata write."""
        self.calls.append(("write_spool_metadata", (spool_id, fields)))


def use_spoolman_client(monkeypatch: pytest.MonkeyPatch,
                        client: Optional[FakeSpoolmanClient]) -> None:
    """Make ``client`` the Spoolman client the Bambu delegate builds (None: Spoolman unreachable)."""
    monkeypatch.setattr(rfid_mod, "_bambu_spoolman_client",
                        lambda afc: client)


def make_bambu_spoolman(unit: Optional[afcBambuAMS] = None, *,
                        section: bool = True,
                        spoolman_url: Optional[str] = None,
                        inline: bool = True) -> BambuSpoolman:
    """A real BambuSpoolman, reached the way the unit reaches it."""
    unit = unit or make_bambu_unit()
    if spoolman_url is not None:
        unit.afc.spoolman = spoolman_url
    if section and unit.printer.lookup_object("AFC_BambuAMS_rfid",
                                              None) is None:
        make_spoolman_section(unit.printer)
    delegate = unit._measure
    if delegate is None:
        error_str = f"{unit.name}: no Spoolman delegate could be built"
        raise RuntimeError(error_str)
    if section and unit._spool is not delegate:
        error_str = (f"{unit.name}: the Spoolman delegate was looked up "
                     f"before the section existed")
        raise RuntimeError(error_str)
    if inline:
        delegate.SPOOLMAN_BG = False
    return delegate


# ── AFC_BridgeBox ────────────────────────────────────────────────────────────

def bridgebox_options(tmp_path: pathlib.Path, name: str = "chain1",
                      **options: Any) -> Dict[str, Any]:
    """A chain master's options with its files under ``tmp_path``."""
    opts: Dict[str, Any] = {
        "serial_port": f"/dev/serial/by-id/usb-{name}-if00",
        "extruder": "extruder", "lane_base": 24,
        "roster": "ht:0123456789ABCDEF00003331",
        "pool_ams": 0, "pool_ht": 0,
        "auto_vars_file": str(tmp_path / "AFC_auto_vars.cfg"),
        "state_file": str(tmp_path / "AFC_BridgeBox.cfg")}
    opts.update(options)
    return opts


def make_bridgebox(tmp_path: pathlib.Path, name: str = "chain1", *,
                   printer: Optional[BambuPrinter] = None,
                   fabricate: Optional[bool] = None,
                   ready: bool = False, register: bool = True,
                   pack: bool = False, **options: Any) -> afcBridgeBox:
    """
    A chain master through ``afcBridgeBox.__init__``.

    :param pack: False for a start whose T# pack already ran (see ``_pack_at_start``), so saved
        T#s with a free one below them stay as a test wrote them
    """
    printer = printer or make_printer()
    if fabricate is not None:
        printer.fabricate = fabricate
    if printer.afc.VarFile == NO_VAR_FILE:
        set_var_file(printer, str(tmp_path / "AFC.var"))
    section = f"AFC_BridgeBox {name}"
    opts = bridgebox_options(tmp_path, name, **options)
    printer.add_section(section, {k: v for k, v in opts.items()
                                  if v is not None})
    master = afcBridgeBox(BambuConfig(section, printer, opts))
    master._pack_checked = not pack
    if register:
        printer.add_object(section, master)
    if ready:
        _require_isolated_bridges(f"make_bridgebox({name!r}, ready=True)")
        live_bridges().setdefault(master.serial_port, FakeBridge())
        master._scout_ready()
    else:
        master.logger = printer.afc.logger
    return master


def record_chain_state(tmp_path: pathlib.Path, name: str = "chain1", *,
                       state_file: Optional[str] = None, **keys: str) -> None:
    """
    Leave the chain's state file as an earlier boot would (``roster``, ``name_map``, ``lane_map``,
    ``bay_owner``...), written by the real ``_state_set`` of a master that builds nothing, on a
    printer of its own.
    """
    target = state_file or str(tmp_path / "AFC_BridgeBox.cfg")
    with tempfile.TemporaryDirectory() as scratch:
        printer = make_printer(var_file=os.path.join(scratch, "AFC.var"))
        master = make_bridgebox(pathlib.Path(scratch), name, printer=printer,
                                register=False, roster="", pool_ams=0,
                                pool_ht=0, state_file=target)
        master._state_set({f"AFC_BridgeBox {name}": dict(keys)})


def write_unit_vars(printer: BambuPrinter, data: Union[str, dict]) -> str:
    """
    Write AFC's ``<VarFile>.unit`` (unit -> lane -> saved record), as AFC's save_vars leaves it for
    the master and the units to read.
    """
    path = f"{printer.afc.VarFile}.unit"
    with open(path, "w") as fh:
        fh.write(data if isinstance(data, str) else json.dumps(data))
    return path


def drain_var_writes(printer: BambuPrinter) -> List[Dict[str, Any]]:
    """
    Drain the snapshots ``afc.save_vars`` queued for AFC's var-file writer, after a chain master
    filled them in.
    """
    out: List[Dict[str, Any]] = []
    write_queue = printer.afc._var_write_queue
    while not write_queue.empty():
        out.append(write_queue.get_nowait())
    return out


# ── transports ───────────────────────────────────────────────────────────────

def make_tcp_port(script: Sequence[bytes] = (), *, key: Optional[str] = None,
                  host: str = "test", port: int = 8888,
                  timeout: float = 0.1,
                  monkeypatch: Optional[pytest.MonkeyPatch] = None,
                  reactor: Optional[FakeReactor] = None) -> TcpPort:
    """A TcpPort through its real ``__init__`` on a :class:`FakeSocket`."""
    if monkeypatch is not None:
        reactor = reactor or FakeReactor()
        patch_module_time(monkeypatch, reactor)
    clock = _module_clock()
    if reactor is not None and clock is not reactor:
        error_str = ("make_tcp_port: the bridge module does not read this "
                     "reactor's clock; pass monkeypatch")
        raise ValueError(error_str)
    sock = FakeSocket(script, clock=clock)

    def _connect(address: Tuple[str, int],
                 timeout: Optional[float] = None) -> FakeSocket:
        """:return FakeSocket: the scripted socket, whatever the address"""
        return sock

    with patch.object(bridge_mod.socket, "create_connection", _connect):
        return TcpPort(host, port, timeout=timeout, connect_timeout=0.5,
                       key=key)


def make_bt_bridge_serial(lines: Sequence[bytes] = (), *,
                          port: str = "/dev/test",
                          logger: Optional[BambuLogger] = None,
                          connect: bool = True
                          ) -> "bt_rfid_mod._BridgeSerial":
    """AFC_BoxTurtle_rfid's ``_BridgeSerial`` through its real ``__init__``."""
    bridge = bt_rfid_mod._BridgeSerial(port, logger or BambuLogger())
    if connect:
        fake = FakeSerial(lines)

        class _Serial:
            """pyserial, as the module imports it: Serial opens ``fake``."""

            @staticmethod
            def Serial(*args: Any, **kwargs: Any) -> FakeSerial:
                """:return FakeSerial: the scripted port"""
                return fake

        with patch.object(bt_rfid_mod, "serial", _Serial):
            bridge.connect()
    return bridge


# ── upstream AFC objects ─────────────────────────────────────────────────────

def make_afc_spool(printer: Optional[BambuPrinter] = None) -> AFCSpool:
    """AFC_spool through its real ``__init__``, connected as AFC connects it."""
    printer = printer or make_printer()
    spool = AFCSpool(BambuConfig("AFC_spool", printer))
    spool.register_commands(printer.afc)
    printer.run_connect_handler(spool.handle_connect)
    printer.afc.spool = spool
    return spool
