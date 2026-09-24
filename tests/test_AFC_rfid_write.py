"""Unit tests for extras/AFC_rfid_write.py."""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, List, Optional, Tuple
from unittest.mock import call

import pytest

from extras.AFC_rfid_readers import decode_afc_block, decode_anycubic
from extras.AFC_rfid_write import RfidWriteTarget, StageError
import extras.AFC_rfid_write as rfid_write_mod
from tests.conftest import CommandError, MockGcode, MockGCodeCommand, MockLogger, MockPrinter


#: resolve_reader's answer when no unit has registered a reader.
RFIDW_NO_READERS = ("no RFID readers are registered. AFC_RFID_WRITE needs a unit "
                    "with a host-side reader (BoxTurtle, ACE2, ViViD or OpenAMS).")


#: _read_blocking's answer when the field holds no tag.
RFIDW_NO_TAG = "no tag in the reader's field"


#: The first four bytes of an Anycubic tag image, written out by hand.
RFIDW_ANYCUBIC_MAGIC = b"\x7b\x00\x65\x00"


#: Every command _ensure_commands registers, in registration order.
RFIDW_COMMANDS = ["AFC_RFID_READERS", "AFC_RFID_WRITE", "AFC_RFID_READ",
                  "AFC_RFID_CLASSIC_WRITE", "AFC_RFID_ERASE", "AFC_RFID_ENROLL"]


class RfidwLink:
    """A reader's register link. reader_power calls are recorded when it has one."""

    def __init__(self, powerable: bool = False, fail_power: bool = False) -> None:
        """
        :param powerable: give the link a reader_power method
        :param fail_power: make reader_power raise after recording the call
        """
        self.events: List[Tuple[str, bool]] = []
        self.fail_power = fail_power
        if powerable:
            self.reader_power = self._power

    def _power(self, on: bool) -> None:
        """
        :param on: the power state asked for
        """
        self.events.append(("power", on))
        if self.fail_power:
            raise OSError("reader gone")

    def reg_read(self, reg: int) -> int:
        """
        :param reg: register number
        :return int: always zero
        """
        return 0

    def reg_write(self, reg: int, val: int) -> None:
        """
        :param reg: register number
        :param val: value written
        """


class RfidwSpoolman:
    """Spoolman client fake: one spool record to read, every create, bind attempt and bind."""

    def __init__(self, spool: Any = None, *, vendor: Any = None, filament: Any = None,
                 created: Any = None, read_error: Optional[Exception] = None,
                 bind_error: Optional[Exception] = None,
                 create_error: Optional[Exception] = None) -> None:
        """
        :param spool: what get_spool returns
        :param vendor: what get_or_create_vendor returns
        :param filament: what create_filament returns
        :param created: what create_spool returns
        :param read_error: raised by get_spool when given
        :param bind_error: raised by write_spool_metadata when given
        :param create_error: raised by get_or_create_vendor when given
        """
        self.spool = spool
        self.vendor = vendor
        self.filament = filament
        self.created = created
        self.read_error = read_error
        self.bind_error = bind_error
        self.create_error = create_error
        self.read: List[int] = []
        self.attempts: List[Tuple[int, Optional[str]]] = []
        self.bound: List[Tuple[int, Optional[str]]] = []
        self.calls: List[Tuple[Any, ...]] = []

    def get_spool(self, spool_id: int) -> Any:
        """
        :param spool_id: Spoolman spool id
        :return Any: the configured spool record
        """
        self.read.append(spool_id)
        if self.read_error is not None:
            raise self.read_error
        return self.spool

    def write_spool_metadata(self, spool_id: int, lot_nr: Optional[str] = None,
                             uid: Optional[str] = None) -> None:
        """
        :param spool_id: Spoolman spool id
        :param lot_nr: not used by the writer
        :param uid: the tag UID bound to the spool
        """
        self.attempts.append((spool_id, uid))
        if self.bind_error is not None:
            raise self.bind_error
        self.bound.append((spool_id, uid))

    def get_or_create_vendor(self, name: str) -> Any:
        """
        :param name: vendor name
        :return Any: the configured vendor
        """
        self.calls.append(("vendor", name))
        if self.create_error is not None:
            raise self.create_error
        return self.vendor

    def create_filament(self, **kwargs: Any) -> Any:
        """
        :param kwargs: the filament fields
        :return Any: the configured filament
        """
        self.calls.append(("filament", kwargs))
        return self.filament

    def create_spool(self, filament_id: int, initial_weight: Optional[float] = None) -> Any:
        """
        :param filament_id: Spoolman filament id
        :param initial_weight: the spool's starting weight
        :return Any: the configured spool
        """
        self.calls.append(("spool", filament_id, initial_weight))
        return self.created


class RfidwAFC:
    """The owning unit's AFC: its lanes, and the Spoolman client cached on it."""

    def __init__(self, client: Optional[RfidwSpoolman] = None, lanes: Any = None,
                 saved: Optional[List[str]] = None) -> None:
        """
        :param client: the client the real _cached_spoolman_client finds cached
        :param lanes: the AFC's lanes mapping
        :param saved: when given, save_vars appends "saved" to it
        """
        self._afc_spoolman_client_cache = client
        self.lanes = lanes
        if saved is not None:
            self.save_vars = lambda: saved.append("saved")


class RfidwUnit:
    """The unit that owns a reader. Optional hooks and keys are set only when given."""

    def __init__(self, afc: Optional[RfidwAFC] = None, reactor: Any = None,
                 **extra: Any) -> None:
        """
        :param afc: the unit's AFC
        :param reactor: the unit's reactor; None so an unthreaded path that
            touched it would fail
        :param extra: further unit attributes (apply hooks, brand keys)
        """
        self.afc = afc
        self.reactor = reactor
        self.logger = MockLogger()
        for name, value in extra.items():
            setattr(self, name, value)


class RfidwCompletion:
    """Reactor completion that records complete() values and wait() deadlines."""

    def __init__(self) -> None:
        self.completed: List[Any] = []
        self.waited: List[Optional[float]] = []

    def complete(self, value: Any) -> None:
        """
        :param value: the completion value
        """
        self.completed.append(value)

    def wait(self, waketime: Optional[float] = None, waketime_result: Any = None) -> Any:
        """
        :param waketime: the absolute deadline
        :param waketime_result: returned as on a timeout
        :return Any: waketime_result
        """
        self.waited.append(waketime)
        return waketime_result


class RfidwReactor:
    """Reactor whose async callbacks run at once, so an inline worker completes."""

    def __init__(self, now: float = 100.0) -> None:
        """
        :param now: the reactor clock
        """
        self.now = now
        self.completions: List[RfidwCompletion] = []

    def monotonic(self) -> float:
        """
        :return float: the reactor clock
        """
        return self.now

    def completion(self) -> RfidwCompletion:
        """
        :return RfidwCompletion: a new recorded completion
        """
        comp = RfidwCompletion()
        self.completions.append(comp)
        return comp

    def register_async_callback(self, callback: Callable[[float], Any],
                                waketime: Optional[float] = None) -> None:
        """
        :param callback: run at once with the reactor clock
        :param waketime: ignored
        """
        callback(self.now)


class RfidwThread:
    """A thread whose start() runs its target inline, or not at all."""

    def __init__(self, owner: RfidwThreading, target: Optional[Callable[[], None]],
                 daemon: bool, name: str) -> None:
        """
        :param owner: the threading stand-in that made it
        :param target: the worker function
        :param daemon: the daemon flag the code asked for
        :param name: the thread name the code asked for
        """
        self.owner = owner
        self.target = target
        self.daemon = daemon
        self.name = name

    def start(self) -> None:
        """Record the start, then run the worker inline when the owner allows it."""
        self.owner.started.append((self.name, self.daemon))
        if self.owner.run and self.target is not None:
            self.owner.current = self
            self.target()


class RfidwThreading:
    """Stands in for the threading module so worker paths run deterministically."""

    def __init__(self, run: bool = True) -> None:
        """
        :param run: whether a started thread runs its worker
        """
        self.run = run
        self.started: List[Tuple[str, bool]] = []
        self.current = RfidwThread(self, None, False, "MainThread")

    def Thread(self, target: Callable[[], None], daemon: bool = False,
               name: str = "") -> RfidwThread:
        """
        :param target: the worker function
        :param daemon: the daemon flag
        :param name: the thread name
        :return RfidwThread: an inline thread
        """
        return RfidwThread(self, target, daemon, name)

    def current_thread(self) -> RfidwThread:
        """
        :return RfidwThread: the thread running now
        """
        return self.current


class RfidwChelper:
    """Stands in for chelper: records the OS thread names set, or has no ffi."""

    def __init__(self, fail: bool = False) -> None:
        """
        :param fail: make get_ffi raise
        """
        self.fail = fail
        self.names: List[bytes] = []

    def get_ffi(self) -> Tuple[None, RfidwChelper]:
        """
        :return tuple: (ffi_main, ffi_lib), the lib being this recorder
        """
        if self.fail:
            raise OSError("no ffi")
        return None, self

    def set_thread_name(self, name: bytes) -> None:
        """
        :param name: the encoded thread name
        """
        self.names.append(name)


class RfidwWriteTag:
    """Stands in for the shared write_tag: records every call, returns a fixed result."""

    def __init__(self, result: Tuple[Optional[str], Optional[str]] = ("04ab", None),
                 raises: Optional[Exception] = None,
                 seen: Optional[List[Any]] = None) -> None:
        """
        :param result: the (uid, error) returned
        :param raises: raised instead when given
        :param seen: when given, "write" is appended to it on each call
        """
        self.result = result
        self.raises = raises
        self.seen = seen
        self.calls: List[Tuple[Any, bytes, Any]] = []

    def __call__(self, link: Any, payload: bytes,
                 is_excluded: Optional[Callable[[str], bool]] = None
                 ) -> Tuple[Optional[str], Optional[str]]:
        """
        :param link: the reader link
        :param payload: the bytes to write
        :param is_excluded: tags to pass over
        :return tuple: the configured (uid, error)
        """
        self.calls.append((link, payload, is_excluded))
        if self.seen is not None:
            self.seen.append("write")
        if self.raises is not None:
            raise self.raises
        return self.result


def rfidw_register(printer: MockPrinter, name: str = "bt:reader0",
                   label: str = "BoxTurtle reader0", unit: Optional[RfidwUnit] = None,
                   open_link: Optional[Callable[[], Any]] = None,
                   **hooks: Any) -> RfidWriteTarget:
    """
    Register a reader through the real register_reader and return its target.

    :param printer: the printer to register on
    :param name: the reader's registry key
    :param label: its AFC_RFID_READERS label
    :param unit: the owning unit, a fresh RfidwUnit when omitted
    :param open_link: the link factory, one returning a fixed RfidwLink when omitted
    :param hooks: further register_reader keyword arguments
    :return RfidWriteTarget: the registered target
    """
    if open_link is None:
        link = RfidwLink()

        def open_link() -> RfidwLink:
            """
            :return RfidwLink: the reader's one link
            """
            return link
    rfid_write_mod.register_reader(printer, name, label, unit or RfidwUnit(), open_link,
                                   **hooks)
    return rfid_write_mod._registry(printer)[name]


def rfidw_threaded(name: str = "bt:reader0", unit: Optional[RfidwUnit] = None,
                   **hooks: Any) -> Tuple[RfidWriteTarget, RfidwReactor]:
    """
    A threaded target whose unit has a recording reactor at clock 100.

    :param name: the reader's registry key
    :param unit: the owning unit; its reactor is replaced
    :param hooks: further register_reader keyword arguments
    :return tuple: (target, reactor)
    """
    reactor = RfidwReactor()
    owner = unit or RfidwUnit()
    owner.reactor = reactor
    target = rfidw_register(MockPrinter(), name=name, unit=owner, threaded=True, **hooks)
    return target, reactor


def rfidw_workers(monkeypatch: pytest.MonkeyPatch, run: bool = True,
                  ffi_fails: bool = False) -> Tuple[RfidwThreading, RfidwChelper]:
    """
    Swap the module's threading and chelper for the inline stand-ins.

    :param monkeypatch: pytest's monkeypatch
    :param run: whether a started worker runs
    :param ffi_fails: whether chelper.get_ffi raises
    :return tuple: (threading stand-in, chelper stand-in)
    """
    threads = RfidwThreading(run=run)
    ffi = RfidwChelper(fail=ffi_fails)
    monkeypatch.setattr(rfid_write_mod, "threading", threads)
    monkeypatch.setattr(rfid_write_mod, "chelper", ffi)
    return threads, ffi


def rfidw_worker_write(monkeypatch: pytest.MonkeyPatch, threads: RfidwThreading,
                       raises: Optional[Exception] = None
                       ) -> List[Tuple[str, Optional[Callable[[str], bool]]]]:
    """
    A write_tag that records the thread it ran on and the tags it was told to pass over.

    :param monkeypatch: pytest's monkeypatch
    :param threads: the threading stand-in
    :param raises: raised by the write instead when given
    :return list: (thread name, is_excluded), one per write
    """
    ran_on: List[Tuple[str, Optional[Callable[[str], bool]]]] = []

    def _write(link: Any, payload: bytes,
               is_excluded: Optional[Callable[[str], bool]] = None) -> Tuple[str, None]:
        """
        :param link: the reader link
        :param payload: the bytes to write
        :param is_excluded: tags to pass over
        :return tuple: the tag's uid and no error
        """
        ran_on.append((threads.current_thread().name, is_excluded))
        if raises is not None:
            raise raises
        return "04ab", None
    monkeypatch.setattr(rfid_write_mod, "write_tag", _write)
    return ran_on


def rfidw_filament_call(name: str, vendor_id: Optional[int] = None, material: str = "PLA",
                        **fields: Any) -> Tuple[str, Dict[str, Any]]:
    """
    The create_filament call _create_spool_blocking should make, every keyword spelled out.

    :param name: the filament name
    :param vendor_id: the vendor id
    :param material: the material
    :param fields: the other keywords that are set
    :return tuple: the recorded ("filament", kwargs) call
    """
    kwargs: Dict[str, Any] = {
        "name": name, "vendor_id": vendor_id, "material": material, "density": None,
        "diameter": None, "color_hex": None, "settings_extruder_temp": None,
        "settings_bed_temp": None, "weight": None, "article_number": None}
    kwargs.update(fields)
    return "filament", kwargs


def rfidw_gcmd(**params: Any) -> MockGCodeCommand:
    """
    :param params: the command's parameters
    :return MockGCodeCommand: a command carrying them
    """
    return MockGCodeCommand(params=params)


def rfidw_command_error(handler: Callable[[Any, Any], None], printer: MockPrinter,
                        gcmd: MockGCodeCommand) -> str:
    """
    Run a command handler that must refuse, and return the refusal.

    :param handler: the cmd_* function
    :param printer: the printer passed to it
    :param gcmd: the command passed to it
    :return str: the CommandError's message
    """
    with pytest.raises(CommandError) as exc:
        handler(printer, gcmd)
    return str(exc.value)


def rfidw_spool_record() -> Dict[str, Any]:
    """
    A full Spoolman spool record, fresh for each test.

    :return dict: spool 136, a Polymaker PLA with every tag field set
    """
    return {
        "id": 136,
        "initial_weight": 1000,
        "remaining_weight": 412.5,
        "filament": {
            "name": "PLA Basic Black", "material": "PLA", "color_hex": "1A2B3C",
            "density": 1.24, "diameter": 1.75, "weight": 823,
            "vendor": {"name": "Polymaker"},
            "settings_extruder_temp": 225, "settings_bed_temp": 60,
        },
    }


#: What spool_fields makes of rfidw_spool_record, worked out by hand.
RFIDW_SPOOL_FIELDS = {
    "spool_id": 136, "manufacturer": "Polymaker", "ftype": "PLA",
    "sku": "PLA Basic Black", "color_argb": 0xFF1A2B3C, "diameter_mm": 1.75,
    "density": 1.24, "weight_g": 823, "hotend_max_c": 225, "bed_temp_c": 60,
}


def rfidw_spoolman_unit(client: RfidwSpoolman, reactor: Any = None) -> RfidwUnit:
    """
    A unit whose AFC has this Spoolman client cached on it.

    :param client: the Spoolman client fake
    :param reactor: the unit's reactor
    :return RfidwUnit: the unit
    """
    return RfidwUnit(afc=RfidwAFC(client=client), reactor=reactor)


class RfidwReadTag:
    """Stands in for the shared read_tag: records each link and key set it is given."""

    def __init__(self, result: Any = None, raises: Optional[Exception] = None) -> None:
        """
        :param result: the read_tag dict returned
        :param raises: raised instead when given
        """
        self.result = result
        self.raises = raises
        self.calls: List[Tuple[Any, Dict[str, Any]]] = []

    def __call__(self, link: Any, **keys: Any) -> Any:
        """
        :param link: the reader link
        :param keys: the brand keys
        :return Any: the configured read result
        """
        self.calls.append((link, keys))
        if self.raises is not None:
            raise self.raises
        return self.result


def rfidw_tracked(seen: List[str]) -> Dict[str, Callable[[Any], None]]:
    """
    prepare/release hooks that record their order.

    :param seen: where "prepare" and "release" are appended
    :return dict: register_reader keyword arguments
    """
    return {"prepare": lambda link: seen.append("prepare"),
            "release": lambda link: seen.append("release")}


class RfidwCommandRuns:
    """Stands in for run_write/run_enroll and apply_written under a command test."""

    def __init__(self, result: Tuple[Any, ...] = ("04ab", None, {}, ""),
                 raises: Optional[Exception] = None) -> None:
        """
        :param result: the (uid, error, fields, note) the run returns
        :param raises: raised by the run instead when given
        """
        self.result = result
        self.raises = raises
        self.runs: List[Tuple[Any, int, Dict[str, Any], Optional[str]]] = []
        self.applies: List[Tuple[Any, str, Optional[str], Dict[str, Any]]] = []

    def run(self, target: RfidWriteTarget, spool_id: int, overrides: Dict[str, Any],
            lane: Optional[str] = None) -> Tuple[Any, ...]:
        """
        :param target: the resolved reader
        :param spool_id: the SPOOL= id
        :param overrides: the typed fields, defaults included
        :param lane: the LANE= value
        :return tuple: the configured result
        """
        self.runs.append((target, spool_id, dict(overrides), lane))
        if self.raises is not None:
            raise self.raises
        return self.result

    def apply(self, target: RfidWriteTarget, lane: str, uid: Optional[str],
              fields: Dict[str, Any]) -> str:
        """
        :param target: the resolved reader
        :param lane: the LANE= value
        :param uid: the tag written
        :param fields: the fields written
        :return str: the apply note
        """
        self.applies.append((target, lane, uid, fields))
        return f"applied to {lane}"


def rfidw_command_setup(monkeypatch: pytest.MonkeyPatch, run_name: str,
                        result: Tuple[Any, ...] = ("04ab", None, {}, ""),
                        raises: Optional[Exception] = None
                        ) -> Tuple[MockPrinter, RfidWriteTarget, RfidwCommandRuns]:
    """
    A printer with one reader, its run and apply swapped for recorders.

    :param monkeypatch: pytest's monkeypatch
    :param run_name: "run_write" or "run_enroll"
    :param result: what the run returns
    :param raises: what the run raises instead
    :return tuple: (printer, target, recorder)
    """
    printer = MockPrinter()
    target = rfidw_register(printer)
    runs = RfidwCommandRuns(result, raises)
    monkeypatch.setattr(rfid_write_mod, run_name, runs.run)
    monkeypatch.setattr(rfid_write_mod, "apply_written", runs.apply)
    return printer, target, runs


class TestDefaultRelease:
    def test_a_powerable_link_is_powered_off(self):
        link = RfidwLink(powerable=True)
        assert rfid_write_mod._default_release(link) is None
        assert link.events == [("power", False)]

    def test_a_failing_power_off_is_swallowed(self):
        link = RfidwLink(powerable=True, fail_power=True)
        assert rfid_write_mod._default_release(link) is None
        assert link.events == [("power", False)]

    def test_a_link_without_power_control_is_left_alone(self):
        link = RfidwLink(powerable=False)
        assert rfid_write_mod._default_release(link) is None
        # Nothing to power off: the evidence is that no AttributeError escaped.
        assert not hasattr(link, "reader_power")


class TestRegistry:
    def test_the_registry_is_per_printer_not_per_module(self):
        """Klipper builds a new Printer without re-importing extras, so module
        state would outlive the units in it and hand out dead links."""
        first, second = MockPrinter(), MockPrinter()
        target = rfidw_register(first, "bt:reader0")
        assert rfid_write_mod._registry(first) == {"bt:reader0": target}
        reg = rfid_write_mod._registry(second)
        assert reg == {}
        assert second._afc_rfid_write_registry is reg
        assert first._afc_rfid_write_registry == {"bt:reader0": target}


class TestRegisterReader:
    class _CountingGcode(MockGcode):
        """A gcode object that also records each command registration."""

        def __init__(self) -> None:
            super().__init__()
            self.registered: List[str] = []

        def register_command(self, name: str, func: Callable[[Any], None],
                             desc: Optional[str] = None) -> None:
            """
            :param name: the command name
            :param func: its handler
            :param desc: its help text
            """
            self.registered.append(name)
            super().register_command(name, func, desc)

    class _LatePrinter(MockPrinter):
        """A printer whose gcode object only appears once it is ready."""

        def __init__(self) -> None:
            super().__init__()
            self.ready = False

        def lookup_object(self, name: str, default: Any = None) -> Any:
            """
            :param name: the object name
            :param default: returned for a missing object
            :return Any: the object, or default for gcode before ready
            """
            if name == "gcode" and not self.ready:
                return default
            return super().lookup_object(name, default)

    def test_registering_creates_the_commands(self):
        printer = MockPrinter()
        unit = RfidwUnit()

        def open_link() -> RfidwLink:
            return RfidwLink()
        assert rfid_write_mod.register_reader(printer, "bt:reader0", "BoxTurtle reader0",
                                              unit, open_link) is None
        target = printer._afc_rfid_write_registry["bt:reader0"]
        assert list(printer._afc_rfid_write_registry) == ["bt:reader0"]
        assert (target.name, target.label, target.unit, target.open_link) == (
            "bt:reader0", "BoxTurtle reader0", unit, open_link)
        assert list(printer._gcode._commands) == RFIDW_COMMANDS
        assert printer._afc_rfid_write_cmds is True

    def test_a_printer_with_no_reader_gets_no_commands(self):
        with_reader, without = MockPrinter(), MockPrinter()
        rfidw_register(with_reader, "bt:reader0")
        assert list(with_reader._gcode._commands) == RFIDW_COMMANDS
        assert without._gcode._commands == {}
        assert not hasattr(without, "_afc_rfid_write_cmds")
        assert not hasattr(without, "_afc_rfid_write_registry")

    def test_the_commands_are_registered_once_for_many_readers(self):
        printer = MockPrinter()
        printer._gcode = self._CountingGcode()
        for name in ("bt:reader0", "bt:reader1", "oams:rfid_a"):
            rfidw_register(printer, name)
        assert printer._gcode.registered == RFIDW_COMMANDS
        assert list(printer._afc_rfid_write_registry) == [
            "bt:reader0", "bt:reader1", "oams:rfid_a"]

    def test_re_registering_a_reader_replaces_it(self):
        """A unit that re-probes on reconnect registers again; that must not
        pile up duplicates."""
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0", "first probe")
        again = rfidw_register(printer, "bt:reader0", "second probe")
        assert printer._afc_rfid_write_registry == {"bt:reader0": again}
        assert again.label == "second probe"

    def test_registration_retries_while_there_is_no_gcode_object(self):
        """Registering before the gcode object exists must not mark the
        commands done and leave the printer without them."""
        printer = self._LatePrinter()
        rfidw_register(printer, "bt:reader0")
        assert printer._gcode._commands == {}
        assert not hasattr(printer, "_afc_rfid_write_cmds")
        assert list(printer._afc_rfid_write_registry) == ["bt:reader0"]
        printer.ready = True
        rfidw_register(printer, "bt:reader1")
        assert list(printer._gcode._commands) == RFIDW_COMMANDS
        assert printer._afc_rfid_write_cmds is True

    def test_the_default_is_the_safe_one(self):
        target = rfidw_register(MockPrinter(), "vivid:reader0")
        assert target.threaded is False
        assert target.prepare is rfid_write_mod._default_prepare
        assert target.release is rfid_write_mod._default_release
        assert (target.write_payload, target.stage, target.unstage, target.exclude,
                target.stage_around, target.serves) == (None,) * 6


class TestEnsureCommands:
    def test_each_command_calls_its_handler_with_the_printer(self, monkeypatch):
        printer = MockPrinter()
        seen: List[Tuple[str, Any, Any]] = []
        handlers = ["cmd_AFC_RFID_READERS", "cmd_AFC_RFID_WRITE", "cmd_AFC_RFID_READ",
                    "cmd_AFC_RFID_CLASSIC_WRITE", "cmd_AFC_RFID_ERASE",
                    "cmd_AFC_RFID_ENROLL"]
        for handler in handlers:
            monkeypatch.setattr(rfid_write_mod, handler,
                                lambda p, g, h=handler: seen.append((h, p, g)))
        assert rfid_write_mod._ensure_commands(printer) is None
        gcmd = rfidw_gcmd()
        for command in printer._gcode._commands.values():
            command(gcmd)
        assert printer._afc_rfid_write_cmds is True
        assert list(printer._gcode._commands) == RFIDW_COMMANDS
        assert seen == [(handler, printer, gcmd) for handler in handlers]

    def test_a_printer_already_flagged_registers_nothing(self):
        printer = MockPrinter()
        printer._afc_rfid_write_cmds = True
        rfid_write_mod._ensure_commands(printer)
        assert printer._gcode._commands == {}
        assert printer._afc_rfid_write_cmds is True


class TestOnline:
    def test_a_link_that_opens_is_online(self):
        target = rfidw_register(MockPrinter(), open_link=RfidwLink)
        assert rfid_write_mod._online(target) is True

    def test_no_link_is_offline(self):
        target = rfidw_register(MockPrinter(), open_link=lambda: None)
        assert rfid_write_mod._online(target) is False

    def test_a_raising_open_is_offline(self):
        def _boom() -> RfidwLink:
            raise OSError("port vanished")
        target = rfidw_register(MockPrinter(), open_link=_boom)
        assert rfid_write_mod._online(target) is False

    def test_a_handoff_target_is_always_listed_online(self):
        opened: List[str] = []
        target = rfidw_register(
            MockPrinter(), "u1:scanner0", "U1 OpenRFID scanner",
            open_link=lambda: opened.append("open"),
            write_payload=lambda pay: ("04", None))
        assert rfid_write_mod._online(target) is True
        assert opened == []


class TestResolveReader:
    @staticmethod
    def _lane_printer() -> Tuple[MockPrinter, Dict[str, RfidWriteTarget]]:
        """
        Three readers that each serve their own lanes, and a U1 that says nothing.

        :return tuple: (printer, name -> target)
        """
        printer = MockPrinter()
        targets = {}
        for name, lanes in (("bt:reader0", {"lane8", "lane9"}),
                            ("bt:reader1", {"lane10", "lane11"}),
                            ("ace2:slot2", {"lane2"})):
            targets[name] = rfidw_register(printer, name, name,
                                           serves=lambda ln, ls=lanes: ln in ls)
        targets["u1:e1"] = rfidw_register(printer, "u1:e1", "U1")
        return printer, targets

    def test_two_readers_serving_the_lane_need_a_name(self):
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0", serves=lambda lane: True)
        rfidw_register(printer, "bt:reader1", serves=lambda lane: True)
        assert rfid_write_mod.resolve_reader(printer, None, "lane4") == (
            None, "READER= is required. Available: bt:reader0, bt:reader1")

    def test_a_qualified_name_is_found(self):
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0")
        rfid_a = rfidw_register(printer, "oams:rfid_a")
        assert rfid_write_mod.resolve_reader(printer, "oams:rfid_a") == (rfid_a, None)

    def test_a_bare_name_works_when_it_is_unambiguous(self):
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0")
        rfid_a = rfidw_register(printer, "oams:rfid_a")
        assert rfid_write_mod.resolve_reader(printer, "rfid_a") == (rfid_a, None)

    def test_a_bare_name_shared_by_two_units_is_refused(self):
        """bt:reader0 and vivid:reader0 both exist; picking one silently
        would write the tag at the wrong bench."""
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0")
        rfidw_register(printer, "vivid:reader0")
        assert rfid_write_mod.resolve_reader(printer, "reader0") == (
            None, "reader0 is ambiguous, name the unit too: bt:reader0, vivid:reader0")

    def test_matching_ignores_case(self):
        printer = MockPrinter()
        target = rfidw_register(printer, "bt:reader0")
        assert rfid_write_mod.resolve_reader(printer, " BT:Reader0 ") == (target, None)

    def test_a_lone_reader_needs_no_name(self):
        printer = MockPrinter()
        target = rfidw_register(printer, "bt:reader0")
        assert rfid_write_mod.resolve_reader(printer, None) == (target, None)

    def test_with_several_readers_a_name_is_required(self):
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0")
        rfidw_register(printer, "bt:reader1")
        assert rfid_write_mod.resolve_reader(printer, None) == (
            None, "READER= is required. Available: bt:reader0, bt:reader1")

    def test_an_unknown_name_lists_what_there_is(self):
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0")
        assert rfid_write_mod.resolve_reader(printer, "nope") == (
            None, "no reader called nope. Available: bt:reader0")
        assert rfid_write_mod.resolve_reader(printer, "Nope") == (
            None, "no reader called Nope. Available: bt:reader0")

    def test_no_readers_at_all_says_so(self):
        assert rfid_write_mod.resolve_reader(MockPrinter(), "anything") == (
            None, RFIDW_NO_READERS)

    def test_the_one_reader_serving_the_lane_is_used(self):
        printer, targets = self._lane_printer()
        assert rfid_write_mod.resolve_reader(printer, None, "lane10") == (
            targets["bt:reader1"], None)
        assert rfid_write_mod.resolve_reader(printer, None, "lane2") == (
            targets["ace2:slot2"], None)

    def test_a_named_reader_still_wins(self):
        printer, targets = self._lane_printer()
        assert rfid_write_mod.resolve_reader(printer, "bt:reader0", "lane10") == (
            targets["bt:reader0"], None)

    def test_a_lane_no_reader_serves_asks_for_one(self):
        printer, _targets = self._lane_printer()
        assert rfid_write_mod.resolve_reader(printer, None, "lane99") == (
            None, "no reader serves lane99; give READER=. Available: bt:reader0, "
                  "bt:reader1, ace2:slot2, u1:e1")

    def test_a_failing_check_is_a_no(self):
        printer, targets = self._lane_printer()

        def boom(lane: str) -> bool:
            raise KeyError(lane)
        rfidw_register(printer, "oams:RFID_A", "x", serves=boom)
        assert rfid_write_mod.resolve_reader(printer, None, "lane9") == (
            targets["bt:reader0"], None)


class TestSpoolFields:
    @staticmethod
    def _fields(spool: Any, spool_id: int = 3) -> Tuple[Dict[str, Any], str]:
        """
        :param spool: the record Spoolman returns
        :param spool_id: the spool asked for
        :return tuple: spool_fields' (fields, note)
        """
        unit = rfidw_spoolman_unit(RfidwSpoolman(spool=spool))
        return rfid_write_mod.spool_fields(unit, spool_id)

    def test_no_client_gives_no_fields(self):
        # The real _cached_spoolman_client finds no moonraker on the AFC.
        unit = RfidwUnit(afc=RfidwAFC())
        assert rfid_write_mod.spool_fields(unit, 3) == ({}, "no Spoolman client")

    def test_no_afc_gives_no_fields(self):
        unit = RfidwUnit(afc=None)
        assert rfid_write_mod.spool_fields(unit, 3) == ({}, "no Spoolman client")

    def test_a_filament_without_material_or_vendor_sets_neither(self):
        assert self._fields({"filament": {"name": "Mystery", "vendor": None},
                             "initial_weight": 500}) == (
            {"spool_id": 3, "sku": "Mystery", "weight_g": 500}, "spool 3")

    def test_no_weight_anywhere_leaves_the_weight_out(self):
        assert self._fields({"filament": {"material": "PLA"}}) == (
            {"spool_id": 3, "ftype": "PLA"}, "spool 3")

    def test_the_remaining_weight_is_the_last_fallback(self):
        assert self._fields({"filament": {"material": "PLA"},
                             "remaining_weight": 412.6}) == (
            {"spool_id": 3, "ftype": "PLA", "weight_g": 413}, "spool 3")

    def test_a_null_filament_is_treated_as_empty(self):
        assert self._fields({"filament": None, "initial_weight": 250}) == (
            {"spool_id": 3, "weight_g": 250}, "spool 3")

    def test_a_hash_and_alpha_on_the_colour_are_dropped(self):
        assert self._fields({"filament": {"color_hex": "#0A0B0CFF"}}) == (
            {"spool_id": 3, "color_argb": 0xFF0A0B0C}, "spool 3")

    def test_it_maps_the_record_onto_tag_fields(self):
        client = RfidwSpoolman(spool=rfidw_spool_record())
        assert rfid_write_mod.spool_fields(rfidw_spoolman_unit(client), 136) == (
            RFIDW_SPOOL_FIELDS, "spool 136")
        assert client.read == [136]

    def test_string_diameter_and_density_become_floats(self):
        fields, note = self._fields({"filament": {"diameter": "1.75", "density": "1.24"}})
        assert (fields, note) == (
            {"spool_id": 3, "diameter_mm": 1.75, "density": 1.24}, "spool 3")
        assert isinstance(fields["diameter_mm"], float)
        assert isinstance(fields["density"], float)

    def test_the_weight_is_the_spools_not_what_is_left_on_it(self):
        """The tag describes the filament. Remaining weight is a running
        figure Spoolman already tracks against the spool id we also write."""
        fields, _note = self._fields(rfidw_spool_record(), 136)
        assert fields["weight_g"] == 823

    def test_it_falls_back_to_the_initial_weight(self):
        assert self._fields({"filament": {"material": "PLA"}, "initial_weight": 750,
                             "remaining_weight": 300}, 136) == (
            {"spool_id": 136, "ftype": "PLA", "weight_g": 750}, "spool 136")

    def test_a_missing_spool_is_reported_and_nothing_is_written(self):
        assert self._fields(None, 999999) == ({}, "Spoolman has no spool 999999")

    def test_a_spoolman_error_is_reported_not_raised(self):
        client = RfidwSpoolman(read_error=RuntimeError("connection refused"))
        assert rfid_write_mod.spool_fields(rfidw_spoolman_unit(client), 136) == (
            {}, "Spoolman read failed: connection refused")
        assert client.read == [136]


class TestWriteBlocking:
    @staticmethod
    def _target(monkeypatch: pytest.MonkeyPatch,
                result: Tuple[Optional[str], Optional[str]] = ("04ab", None),
                client: Optional[RfidwSpoolman] = None, link: Optional[RfidwLink] = None,
                raises: Optional[Exception] = None, **hooks: Any
                ) -> Tuple[RfidWriteTarget, List[str], RfidwWriteTag, RfidwLink]:
        """
        A registered reader whose link opens, and a write_tag stand-in.

        :param monkeypatch: pytest's monkeypatch
        :param result: what write_tag returns
        :param client: the unit's Spoolman client
        :param link: the reader's link, a plain RfidwLink when omitted
        :param raises: what write_tag raises instead
        :param hooks: further register_reader keyword arguments
        :return tuple: (target, opens recorded, write_tag stand-in, link)
        """
        opened: List[str] = []
        the_link = link if link is not None else RfidwLink()

        def _open() -> RfidwLink:
            opened.append("open")
            return the_link
        writer = RfidwWriteTag(result, raises=raises)
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        unit = rfidw_spoolman_unit(client or RfidwSpoolman())
        target = rfidw_register(MockPrinter(), unit=unit, open_link=_open, **hooks)
        return target, opened, writer, the_link

    def test_an_unreadable_spool_returns_before_the_reader_opens(self, monkeypatch):
        client = RfidwSpoolman(spool=None)
        target, opened, writer, _link = self._target(monkeypatch, client=client)
        assert rfid_write_mod._write_blocking(target, 136, {"ftype": "PETG"}) == (
            None, "Spoolman has no spool 136", {}, "")
        assert opened == []
        assert writer.calls == []
        assert client.read == [136]

    def test_a_failed_uid_bind_is_noted_not_raised(self, monkeypatch):
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}},
                               bind_error=RuntimeError("Spoolman 500"))
        target, _opened, writer, _link = self._target(monkeypatch, client=client)
        assert rfid_write_mod._write_blocking(target, 136, {}) == (
            "04ab", None, {"spool_id": 136, "ftype": "PLA"},
            "spool 136, but binding the uid failed: Spoolman 500")
        assert client.attempts == [(136, "04ab")]
        assert client.bound == []
        assert len(writer.calls) == 1

    def test_a_write_error_alone_skips_the_bind(self, monkeypatch):
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target, _opened, _writer, _link = self._target(monkeypatch, ("04ab", "NAK"),
                                                       client=client)
        assert rfid_write_mod._write_blocking(target, 136, {}) == (
            "04ab", "NAK", {"spool_id": 136, "ftype": "PLA"}, "spool 136")
        assert client.bound == []

    def test_no_spool_alone_skips_the_bind(self, monkeypatch):
        client = RfidwSpoolman(spool={"filament": {"material": "PETG"}})
        target, _opened, _writer, _link = self._target(monkeypatch, client=client)
        assert rfid_write_mod._write_blocking(target, 0, {"ftype": "PLA"}) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert client.read == []
        assert client.bound == []

    def test_no_uid_alone_skips_the_bind(self, monkeypatch):
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target, _opened, _writer, _link = self._target(monkeypatch, (None, None),
                                                       client=client)
        assert rfid_write_mod._write_blocking(target, 136, {}) == (
            None, None, {"spool_id": 136, "ftype": "PLA"}, "spool 136")
        assert client.bound == []

    def test_a_clean_write_with_a_spool_binds_the_uid(self, monkeypatch):
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target, _opened, _writer, _link = self._target(monkeypatch, client=client)
        assert rfid_write_mod._write_blocking(target, 136, {}) == (
            "04ab", None, {"spool_id": 136, "ftype": "PLA"}, "spool 136, uid bound")
        assert client.bound == [(136, "04ab")]

    def test_a_powerable_link_is_powered_for_the_write(self, monkeypatch):
        link = RfidwLink(powerable=True)
        target, opened, writer, _link = self._target(monkeypatch, link=link)
        assert rfid_write_mod._write_blocking(target, 0, {"ftype": "PLA"}) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert link.events == [("power", True), ("power", False)]
        assert opened == ["open"]
        assert [c[0] for c in writer.calls] == [link]

    def test_a_link_with_no_power_control_is_left_alone(self, monkeypatch):
        link = RfidwLink(powerable=False)
        target, _opened, writer, _link = self._target(monkeypatch, link=link)

        def is_sister(uid: str) -> bool:
            return uid == "cafe"
        assert rfid_write_mod._write_blocking(target, 0, {"ftype": "PLA"}, is_sister) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert not hasattr(link, "reader_power")
        assert [(c[0], c[2]) for c in writer.calls] == [(link, is_sister)]

    def test_the_reader_is_given_back_even_when_the_write_fails(self, monkeypatch):
        """Leaving the ACE2's identify loop off, or the reader powered, would
        outlast the command and break the next insert."""
        link = RfidwLink(powerable=True)
        target, _opened, _writer, _link = self._target(
            monkeypatch, link=link, raises=RuntimeError("serial went away"))
        with pytest.raises(RuntimeError, match="^serial went away$"):
            rfid_write_mod._write_blocking(target, 0, {"ftype": "PLA"})
        assert link.events == [("power", True), ("power", False)]

    def test_a_units_own_prepare_and_release_are_used(self, monkeypatch):
        seen: List[Tuple[str, Any]] = []
        link = RfidwLink(powerable=True)
        target, _opened, _writer, _link = self._target(
            monkeypatch, link=link, prepare=lambda lnk: seen.append(("prepare", lnk)),
            release=lambda lnk: seen.append(("release", lnk)))
        assert rfid_write_mod._write_blocking(target, 0, {"ftype": "PLA"}) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert seen == [("prepare", link), ("release", link)]
        assert link.events == []

    def test_an_offline_reader_is_reported_not_written(self, monkeypatch):
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        target = rfidw_register(MockPrinter(), open_link=lambda: None)
        assert rfid_write_mod._write_blocking(target, 0, {"ftype": "PLA"}) == (
            None, "bt:reader0 is offline", {}, "")
        assert writer.calls == []

    def test_the_uid_is_bound_back_to_the_spool(self, monkeypatch):
        client = RfidwSpoolman(spool=rfidw_spool_record())
        target, _opened, _writer, _link = self._target(
            monkeypatch, ("04a1b2c3d4e5f6", None), client=client)
        assert rfid_write_mod._write_blocking(target, 136, {}) == (
            "04a1b2c3d4e5f6", None, RFIDW_SPOOL_FIELDS, "spool 136, uid bound")
        assert client.bound == [(136, "04a1b2c3d4e5f6")]

    def test_nothing_is_bound_when_the_write_failed(self, monkeypatch):
        client = RfidwSpoolman(spool=rfidw_spool_record())
        target, _opened, _writer, _link = self._target(
            monkeypatch, ("04ab", "tag did not ACK page 9"), client=client)
        assert rfid_write_mod._write_blocking(target, 136, {}) == (
            "04ab", "tag did not ACK page 9", RFIDW_SPOOL_FIELDS, "spool 136")
        assert client.bound == []

    def test_explicit_params_beat_the_spoolman_record(self, monkeypatch):
        client = RfidwSpoolman(spool=rfidw_spool_record())
        target, _opened, _writer, _link = self._target(monkeypatch, client=client)
        _uid, _err, fields, _note = rfid_write_mod._write_blocking(
            target, 136, {"manufacturer": "Overture", "weight_g": 500})
        assert fields == {
            "spool_id": 136, "manufacturer": "Overture", "ftype": "PLA",
            "sku": "PLA Basic Black", "color_argb": 0xFF1A2B3C, "diameter_mm": 1.75,
            "density": 1.24, "weight_g": 500, "hotend_max_c": 225, "bed_temp_c": 60}

    def test_the_handoff_replaces_the_link_path(self, monkeypatch):
        payloads: List[bytes] = []

        def handoff(payload: bytes) -> Tuple[Optional[str], Optional[str]]:
            payloads.append(payload)
            return "04deadbeef1122", None
        target, opened, writer, _link = self._target(monkeypatch, threaded=True,
                                                     write_payload=handoff)
        assert rfid_write_mod._write_blocking(
            target, 0, {"ftype": "PLA", "weight_g": 1000}) == (
            "04deadbeef1122", None, {"ftype": "PLA", "weight_g": 1000}, "")
        assert opened == []
        assert writer.calls == []
        # The fully encoded tag (Anycubic layout + AFC block), not the fields.
        assert len(payloads) == 1
        assert len(payloads[0]) == 144
        assert payloads[0][:4] == RFIDW_ANYCUBIC_MAGIC

    def test_a_handoff_error_is_passed_through(self, monkeypatch):
        target, _opened, writer, _link = self._target(
            monkeypatch, write_payload=lambda pay: (None, "no answer from OpenRFID"))
        assert rfid_write_mod._write_blocking(target, 0, {"ftype": "PLA"}) == (
            None, "no answer from OpenRFID", {"ftype": "PLA"}, "")
        assert writer.calls == []

    def test_the_handoff_still_binds_the_spool_uid(self, monkeypatch):
        client = RfidwSpoolman(spool=rfidw_spool_record())
        target, _opened, _writer, _link = self._target(
            monkeypatch, client=client,
            write_payload=lambda pay: ("04a1b2c3d4e5f6", None))
        assert rfid_write_mod._write_blocking(target, 136, {}) == (
            "04a1b2c3d4e5f6", None, RFIDW_SPOOL_FIELDS, "spool 136, uid bound")
        assert client.bound == [(136, "04a1b2c3d4e5f6")]


class TestWithStaging:
    @staticmethod
    def _body(seen: List[Any], result: Tuple[Any, ...] = ("04ab", None, {}, "")
              ) -> Callable[[Any], Tuple[Any, ...]]:
        """
        :param seen: where each body run is recorded
        :param result: what the body returns
        :return callable: the body
        """
        def body(excluded: Any) -> Tuple[Any, ...]:
            seen.append(("body", excluded))
            return result
        return body

    def test_a_body_called_twice_inside_the_scan_runs_once(self):
        seen: List[Any] = []
        scans: List[Any] = []

        def around(lane: str, run: Callable[..., Any]) -> None:
            scans.append((lane, run("first"), run("second")))
        target = rfidw_register(MockPrinter(), stage_around=around)
        result = rfid_write_mod._with_staging(target, "lane4", self._body(seen))
        assert result == ("04ab", None, {}, "")
        assert seen == [("body", "first")]
        assert scans == [("lane4", result, result)]

    def test_no_lane_skips_every_staging_hook(self):
        seen: List[Any] = []
        target = rfidw_register(
            MockPrinter(), stage=lambda lane: seen.append("stage") or "tok",
            unstage=lambda tok: seen.append("unstage"),
            stage_around=lambda lane, run: seen.append("around"))
        assert rfid_write_mod._with_staging(target, None, self._body(seen)) == (
            "04ab", None, {}, "")
        assert seen == [("body", None)]

    def test_a_lane_without_a_stage_runs_the_body_directly(self):
        seen: List[Any] = []
        target = rfidw_register(MockPrinter(),
                                unstage=lambda tok: seen.append(("unstage", tok)))
        assert rfid_write_mod._with_staging(target, "lane4", self._body(seen)) == (
            "04ab", None, {}, "")
        assert seen == [("body", None)]

    def test_a_none_token_skips_exclude_and_unstage(self):
        seen: List[Any] = []
        target = rfidw_register(
            MockPrinter(), stage=lambda lane: seen.append(("stage", lane)),
            unstage=lambda tok: seen.append(("unstage", tok)),
            exclude=lambda tok: seen.append(("exclude", tok)))
        assert rfid_write_mod._with_staging(target, "lane4", self._body(seen)) == (
            "04ab", None, {}, "")
        assert seen == [("stage", "lane4"), ("body", None)]

    def test_a_token_without_exclude_passes_none(self):
        seen: List[Any] = []
        target = rfidw_register(
            MockPrinter(), stage=lambda lane: seen.append(("stage", lane)) or "tok",
            unstage=lambda tok: seen.append(("unstage", tok)))
        assert rfid_write_mod._with_staging(target, "lane4", self._body(seen)) == (
            "04ab", None, {}, "")
        assert seen == [("stage", "lane4"), ("body", None), ("unstage", "tok")]

    def test_a_token_with_exclude_passes_its_test(self):
        seen: List[Any] = []

        def is_sister(uid: str) -> bool:
            return uid == "cafe"
        target = rfidw_register(
            MockPrinter(), stage=lambda lane: "tok",
            unstage=lambda tok: seen.append(("unstage", tok)),
            exclude=lambda tok: seen.append(("exclude", tok)) or is_sister)
        assert rfid_write_mod._with_staging(target, "lane4", self._body(seen)) == (
            "04ab", None, {}, "")
        assert seen == [("exclude", "tok"), ("body", is_sister), ("unstage", "tok")]

    def test_a_token_without_unstage_is_not_restored(self):
        seen: List[Any] = []
        target = rfidw_register(MockPrinter(),
                                stage=lambda lane: seen.append(("stage", lane)) or "tok")
        assert rfid_write_mod._with_staging(target, "lane4", self._body(seen)) == (
            "04ab", None, {}, "")
        assert seen == [("stage", "lane4"), ("body", None)]


class TestRunWrite:
    class _EventCompletion:
        """A completion whose wait() blocks until a real worker thread completes it."""

        def __init__(self, waited: List[Optional[float]]) -> None:
            """
            :param waited: where each wait deadline is recorded
            """
            self.waited = waited
            self.done = threading.Event()

        def complete(self, value: Any) -> None:
            """
            :param value: the completion value
            """
            self.done.set()

        def wait(self, waketime: Optional[float] = None, waketime_result: Any = None) -> Any:
            """
            :param waketime: the absolute deadline
            :param waketime_result: returned as on a timeout
            :return Any: waketime_result
            """
            self.waited.append(waketime)
            self.done.wait(5.0)
            return waketime_result

    class _ThreadReactor:
        """A reactor for a real worker thread: the clock is zero, callbacks run at once."""

        def __init__(self) -> None:
            self.waited: List[Optional[float]] = []

        def monotonic(self) -> float:
            """
            :return float: the reactor clock
            """
            return 0.0

        def completion(self) -> TestRunWrite._EventCompletion:
            """
            :return _EventCompletion: a completion the worker sets
            """
            return TestRunWrite._EventCompletion(self.waited)

        def register_async_callback(self, callback: Callable[[float], Any],
                                    waketime: Optional[float] = None) -> None:
            """
            :param callback: run at once
            :param waketime: ignored
            """
            callback(0.0)

    @staticmethod
    def _staged_target(monkeypatch: pytest.MonkeyPatch, seen: List[Any],
                       **hooks: Any) -> RfidWriteTarget:
        """
        A reader with stage/unstage hooks and a write_tag that records into seen.

        :param monkeypatch: pytest's monkeypatch
        :param seen: where stage, write and unstage are recorded
        :param hooks: register_reader hooks replacing the default stage/unstage
        :return RfidWriteTarget: the target
        """
        stage_hooks: Dict[str, Any] = {
            "stage": lambda lane: seen.append(("stage", lane)) or ("tok", lane),
            "unstage": lambda tok: seen.append(("unstage", tok))}
        stage_hooks.update(hooks)
        monkeypatch.setattr(rfid_write_mod, "write_tag", RfidwWriteTag(seen=seen))
        return rfidw_register(MockPrinter(), **stage_hooks)

    @staticmethod
    def _around_target(monkeypatch: pytest.MonkeyPatch, around: Callable[..., Any],
                       seen: List[Any]) -> RfidWriteTarget:
        """
        An OpenAMS-style reader whose scan runs the write itself.

        :param monkeypatch: pytest's monkeypatch
        :param around: the unit's stage_around
        :param seen: where each write records ("write", is_excluded)
        :return RfidWriteTarget: the target
        """
        def _write(link: Any, payload: bytes,
                   is_excluded: Optional[Callable[[str], bool]] = None
                   ) -> Tuple[str, None]:
            seen.append(("write", is_excluded))
            return "04ab", None
        monkeypatch.setattr(rfid_write_mod, "write_tag", _write)
        return rfidw_register(MockPrinter(), "oams:RFID_A", "OpenAMS RFID_A",
                              stage_around=around)

    def test_a_threaded_write_runs_on_a_named_worker(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert ran_on == [("afc_wr_reader0", None)]
        assert threads.started == [("afc_wr_reader0", True)]
        assert ffi.names == [b"afc_wr_reader0"]
        assert reactor.completions[0].completed == [None]
        assert reactor.completions[0].waited == [190.0]

    def test_a_long_reader_name_is_clipped_to_the_os_limit(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch)
        rfidw_worker_write(monkeypatch, threads)
        target, _reactor = rfidw_threaded(name="ace2:slot_number_twelve")
        rfid_write_mod.run_write(target, 0, {"ftype": "PLA"})
        assert threads.started == [("afc_wr_slot_num", True)]
        assert ffi.names == [b"afc_wr_slot_num"]

    def test_a_thread_name_failure_does_not_stop_the_write(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch, ffi_fails=True)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert ran_on == [("afc_wr_reader0", None)]
        assert ffi.names == []
        assert reactor.completions[0].completed == [None]

    def test_a_raising_write_becomes_the_error(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch)
        rfidw_worker_write(monkeypatch, threads, raises=RuntimeError("serial went away"))
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}) == (
            None, "serial went away", {}, "")
        assert reactor.completions[0].completed == [None]

    def test_a_worker_that_never_runs_says_so(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch, run=False)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}) == (
            None, "the write never ran", {}, "")
        assert ran_on == []
        assert threads.started == [("afc_wr_reader0", True)]
        assert reactor.completions[0].completed == []
        assert reactor.completions[0].waited == [190.0]

    def test_a_threaded_write_passes_the_staged_exclusions_to_the_worker(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        tokens: List[str] = []

        def is_sister(uid: str) -> bool:
            return uid == "cafe"
        target, _reactor = rfidw_threaded(
            stage=lambda lane: "tok", exclude=lambda tok: tokens.append(tok) or is_sister)
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane4") == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert tokens == ["tok"]
        assert ran_on == [("afc_wr_reader0", is_sister)]

    def test_a_named_lane_stages_around_the_write(self, monkeypatch):
        seen: List[Any] = []
        target = self._staged_target(monkeypatch, seen)
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane11") == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert seen == [("stage", "lane11"), "write", ("unstage", ("tok", "lane11"))]

    def test_no_lane_skips_staging(self, monkeypatch):
        seen: List[Any] = []
        target = self._staged_target(monkeypatch, seen)
        rfid_write_mod.run_write(target, 0, {"ftype": "PLA"})
        assert seen == ["write"]

    def test_unstage_runs_even_when_the_write_fails(self, monkeypatch):
        seen: List[Any] = []
        target = self._staged_target(monkeypatch, seen)
        monkeypatch.setattr(rfid_write_mod, "write_tag",
                            RfidwWriteTag(raises=RuntimeError("serial went away"),
                                          seen=seen))
        with pytest.raises(RuntimeError, match="^serial went away$"):
            rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane11")
        assert seen == [("stage", "lane11"), "write", ("unstage", ("tok", "lane11"))]

    def test_a_target_without_a_stage_ignores_the_lane(self, monkeypatch):
        seen: List[Any] = []
        target = self._staged_target(monkeypatch, seen, stage=None)
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane1") == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert seen == ["write"]

    def test_an_mcu_backed_reader_runs_on_the_reactor(self, monkeypatch):
        """Klipper shuts the printer down if an MCU_SPI transfer happens on a
        worker. Registering without threaded=True must keep the write inline;
        the unit here has no reactor at all, so touching one would blow up."""
        threads, _ffi = rfidw_workers(monkeypatch)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        target = rfidw_register(MockPrinter(), "oams:RFID_A", "OpenAMS RFID_A",
                                open_link=RfidwLink)
        assert target.threaded is False
        assert target.unit.reactor is None
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert ran_on == [("MainThread", None)]
        assert threads.started == []

    def test_a_host_owned_serial_port_may_be_threaded(self, monkeypatch):
        """The BoxTurtle bridge is a plain USB-CDC port this module opened, so
        its write belongs off the reactor: it is seconds of round trips."""
        ran_on: List[str] = []

        def _write(link: Any, payload: bytes,
                   is_excluded: Optional[Callable[[str], bool]] = None
                   ) -> Tuple[str, None]:
            ran_on.append(threading.current_thread().name)
            return "04ab", None
        monkeypatch.setattr(rfid_write_mod, "write_tag", _write)
        reactor = self._ThreadReactor()
        target = rfidw_register(MockPrinter(), unit=RfidwUnit(reactor=reactor),
                                threaded=True)
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert ran_on == ["afc_wr_reader0"]
        assert reactor.waited == [90.0]

    def test_the_write_runs_where_the_scan_holds_the_tag(self, monkeypatch):
        seen: List[Any] = []

        def excl(uid: str) -> bool:
            return uid == "cafe"

        def around(lane: str, body: Callable[..., Any]) -> None:
            seen.append(("scan", lane))
            body(excl)
            seen.append("unwind")
        target = self._around_target(monkeypatch, around, seen)
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane4") == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert seen == [("scan", "lane4"), ("write", excl), "unwind"]

    def test_a_scan_that_never_holds_the_tag_writes_nothing(self, monkeypatch):
        seen: List[Any] = []
        scans: List[str] = []
        target = self._around_target(monkeypatch, lambda lane, body: scans.append(lane),
                                     seen)
        with pytest.raises(StageError) as exc:
            rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane4")
        assert str(exc.value) == ("lane4's tag did not come to rest in the reader's "
                                  "field, so nothing was written")
        assert scans == ["lane4"]
        assert seen == []

    def test_without_a_lane_the_scan_is_not_run(self, monkeypatch):
        seen: List[Any] = []
        scans: List[str] = []
        target = self._around_target(monkeypatch, lambda lane, body: scans.append(lane),
                                     seen)
        assert rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}) == (
            "04ab", None, {"ftype": "PLA"}, "")
        assert scans == []
        assert seen == [("write", None)]

    def test_the_stage_token_decides_what_the_write_passes_over(self, monkeypatch):
        got: List[Any] = []
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        target = rfidw_register(
            MockPrinter(), stage=lambda lane: ("tok", lane), unstage=lambda tok: None,
            exclude=lambda tok: got.append(tok) or (
                lambda uid: uid == f"sister-of-{tok[1]}"))
        rfid_write_mod.run_write(target, 0, {"ftype": "PLA"}, lane="lane9")
        assert got == [("tok", "lane9")]
        passed_over = writer.calls[0][2]
        assert passed_over("sister-of-lane9") is True
        assert passed_over("04ab") is False


class TestReadBlocking:
    def test_a_tag_without_a_uid_counts_as_no_tag(self, monkeypatch):
        seen: List[str] = []
        monkeypatch.setattr(rfid_write_mod, "read_tag",
                            RfidwReadTag({"uid": None, "tag_type": None}))
        target = rfidw_register(MockPrinter(), **rfidw_tracked(seen))
        assert rfid_write_mod._read_blocking(target) == (None, RFIDW_NO_TAG)
        assert seen == ["prepare", "release"]

    def test_a_tag_with_a_uid_is_returned(self, monkeypatch):
        link = RfidwLink()
        tag = {"uid": "04ab", "tag_type": "MifareUltralight"}
        reader = RfidwReadTag(tag)
        monkeypatch.setattr(rfid_write_mod, "read_tag", reader)
        target = rfidw_register(MockPrinter(), open_link=lambda: link)
        assert rfid_write_mod._read_blocking(target) == (tag, None)
        assert reader.calls == [(link, {"bambu_master_key": None, "creality_key": None,
                                        "creality_encryption_key": None})]

    def test_release_runs_even_if_the_read_raises(self, monkeypatch):
        seen: List[str] = []
        monkeypatch.setattr(rfid_write_mod, "read_tag",
                            RfidwReadTag(raises=RuntimeError("reader wedged")))
        target = rfidw_register(MockPrinter(), "ace2:slot0", "ACE2 slot0",
                                **rfidw_tracked(seen))
        with pytest.raises(RuntimeError, match="^reader wedged$"):
            rfid_write_mod._read_blocking(target)
        assert seen == ["prepare", "release"]


class TestEraseBlocking:
    def test_the_reader_is_released_when_the_erase_raises(self, monkeypatch):
        seen: List[str] = []
        monkeypatch.setattr(rfid_write_mod, "write_tag",
                            RfidwWriteTag(raises=RuntimeError("serial went away")))
        target = rfidw_register(MockPrinter(), **rfidw_tracked(seen))
        with pytest.raises(RuntimeError, match="^serial went away$"):
            rfid_write_mod._erase_blocking(target)
        assert seen == ["prepare", "release"]

    def test_a_handoff_target_never_opens_a_link(self):
        opened: List[int] = []
        payloads: List[bytes] = []
        target = rfidw_register(
            MockPrinter(), "u1:scanner0", open_link=lambda: opened.append(1),
            write_payload=lambda pay: payloads.append(pay) or ("04bb", None))
        assert rfid_write_mod._erase_blocking(target) == ("04bb", None)
        assert opened == []
        assert payloads == [b"\x00" * 144]


class TestRunErase:
    @staticmethod
    def _worker_erase(monkeypatch: pytest.MonkeyPatch, threads: RfidwThreading,
                      raises: Optional[Exception] = None) -> List[Tuple[str, bytes]]:
        """
        A write_tag that records the thread it ran on and the bytes it wrote.

        :param monkeypatch: pytest's monkeypatch
        :param threads: the threading stand-in
        :param raises: raised by the write when given
        :return list: (thread name, payload) per write
        """
        writes: List[Tuple[str, bytes]] = []

        def _write(link: Any, payload: bytes) -> Tuple[str, None]:
            writes.append((threads.current_thread().name, payload))
            if raises is not None:
                raise raises
            return "04aa", None
        monkeypatch.setattr(rfid_write_mod, "write_tag", _write)
        return writes

    def test_a_threaded_erase_runs_on_a_named_worker(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch)
        writes = self._worker_erase(monkeypatch, threads)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_erase(target) == ("04aa", None)
        assert writes == [("afc_rfid_er", b"\x00" * 144)]
        assert threads.started == [("afc_rfid_er", True)]
        assert ffi.names == [b"afc_rfid_er"]
        assert reactor.completions[0].completed == [None]
        assert reactor.completions[0].waited == [190.0]

    def test_a_thread_name_failure_does_not_stop_the_erase(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch, ffi_fails=True)
        writes = self._worker_erase(monkeypatch, threads)
        target, _reactor = rfidw_threaded()
        assert rfid_write_mod.run_erase(target) == ("04aa", None)
        assert writes == [("afc_rfid_er", b"\x00" * 144)]
        assert ffi.names == []

    def test_a_raising_erase_becomes_the_error(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch)
        self._worker_erase(monkeypatch, threads, raises=RuntimeError("serial went away"))
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_erase(target) == (None, "serial went away")
        assert reactor.completions[0].completed == [None]

    def test_a_worker_that_never_runs_says_so(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch, run=False)
        writes = self._worker_erase(monkeypatch, threads)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_erase(target) == (None, "the erase never ran")
        assert writes == []
        assert reactor.completions[0].completed == []
        assert reactor.completions[0].waited == [190.0]

    def test_it_writes_zeros_over_the_record(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch)
        writes = self._worker_erase(monkeypatch, threads)
        link = RfidwLink(powerable=True)
        target = rfidw_register(MockPrinter(), open_link=lambda: link)
        assert rfid_write_mod.run_erase(target) == ("04aa", None)
        assert writes == [("MainThread", b"\x00" * 144)]
        assert link.events == [("power", True), ("power", False)]
        assert threads.started == []

    def test_it_does_not_touch_spoolman(self, monkeypatch):
        monkeypatch.setattr(rfid_write_mod, "write_tag", RfidwWriteTag(("04aa", None)))
        client = RfidwSpoolman(spool=rfidw_spool_record())
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_erase(target) == ("04aa", None)
        assert (client.read, client.bound, client.calls) == ([], [], [])

    def test_offline_is_reported(self):
        target = rfidw_register(MockPrinter(), open_link=lambda: None)
        assert rfid_write_mod.run_erase(target) == (None, "bt:reader0 is offline")

    def test_a_handoff_reader_erases_through_its_payload_path(self, monkeypatch):
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        payloads: List[bytes] = []
        target = rfidw_register(
            MockPrinter(), "u1:scanner0", "U1 scanner", open_link=object,
            write_payload=lambda pay: payloads.append(pay) or ("04bb", None))
        assert rfid_write_mod.run_erase(target) == ("04bb", None)
        assert payloads == [b"\x00" * 144]
        assert writer.calls == []


class TestClassicWriteRun:
    DATA = bytes.fromhex("00112233445566778899aabbccddeeff")
    KEY = b"\xa0\xa1\xa2\xa3\xa4\xa5"

    @staticmethod
    def _stub(monkeypatch: pytest.MonkeyPatch,
              result: Tuple[Optional[str], Optional[str]] = ("01020304", None),
              raises: Optional[Exception] = None,
              threads: Optional[RfidwThreading] = None) -> List[Tuple[Any, ...]]:
        """
        A classic_write_block that records each block write.

        :param monkeypatch: pytest's monkeypatch
        :param result: the (uid, error) returned
        :param raises: raised instead when given
        :param threads: when given, the thread name is recorded too
        :return list: (block, data, key[, thread name]) per write
        """
        calls: List[Tuple[Any, ...]] = []

        def _write(link: Any, block: int, data16: bytes,
                   key6: bytes) -> Tuple[Optional[str], Optional[str]]:
            where = (threads.current_thread().name,) if threads is not None else ()
            calls.append((block, data16, key6) + where)
            if raises is not None:
                raise raises
            return result
        monkeypatch.setattr(rfid_write_mod, "classic_write_block", _write)
        return calls

    def test_a_reactor_write_is_bracketed_by_prepare_and_release(self, monkeypatch):
        seen: List[str] = []
        calls = self._stub(monkeypatch)
        target = rfidw_register(MockPrinter(), **rfidw_tracked(seen))
        assert rfid_write_mod._classic_write_run(target, 4, self.DATA, self.KEY) == (
            "01020304", None)
        assert calls == [(4, self.DATA, self.KEY)]
        assert seen == ["prepare", "release"]

    def test_an_offline_reader_writes_nothing(self, monkeypatch):
        calls = self._stub(monkeypatch)
        target = rfidw_register(MockPrinter(), open_link=lambda: None)
        assert rfid_write_mod._classic_write_run(target, 4, self.DATA, self.KEY) == (
            None, "bt:reader0 is offline")
        assert calls == []

    def test_a_link_without_registers_writes_nothing(self, monkeypatch):
        seen: List[str] = []
        calls = self._stub(monkeypatch)
        target = rfidw_register(MockPrinter(), "u1:scanner0", open_link=object,
                                **rfidw_tracked(seen))
        assert rfid_write_mod._classic_write_run(target, 4, self.DATA, self.KEY) == (
            None, "u1:scanner0 cannot do a Classic write here; its reader is "
                  "driven elsewhere")
        assert calls == []
        assert seen == []

    def test_the_reader_is_released_when_the_write_raises(self, monkeypatch):
        seen: List[str] = []
        self._stub(monkeypatch, raises=RuntimeError("crypto1 lost"))
        target = rfidw_register(MockPrinter(), **rfidw_tracked(seen))
        with pytest.raises(RuntimeError, match="^crypto1 lost$"):
            rfid_write_mod._classic_write_run(target, 4, self.DATA, self.KEY)
        assert seen == ["prepare", "release"]

    def test_a_threaded_write_runs_on_a_named_worker(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch)
        calls = self._stub(monkeypatch, threads=threads)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod._classic_write_run(target, 5, self.DATA, self.KEY) == (
            "01020304", None)
        assert calls == [(5, self.DATA, self.KEY, "afc_cw")]
        assert threads.started == [("afc_cw", True)]
        assert ffi.names == [b"afc_cw"]
        assert reactor.completions[0].completed == [None]
        assert reactor.completions[0].waited == [190.0]

    def test_a_thread_name_failure_does_not_stop_the_write(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch, ffi_fails=True)
        calls = self._stub(monkeypatch, threads=threads)
        target, _reactor = rfidw_threaded()
        assert rfid_write_mod._classic_write_run(target, 5, self.DATA, self.KEY) == (
            "01020304", None)
        assert calls == [(5, self.DATA, self.KEY, "afc_cw")]
        assert ffi.names == []

    def test_a_raising_threaded_write_becomes_the_error(self, monkeypatch):
        rfidw_workers(monkeypatch)
        self._stub(monkeypatch, raises=RuntimeError("crypto1 lost"))
        target, reactor = rfidw_threaded()
        assert rfid_write_mod._classic_write_run(target, 5, self.DATA, self.KEY) == (
            None, "crypto1 lost")
        assert reactor.completions[0].completed == [None]

    def test_a_worker_that_never_runs_says_so(self, monkeypatch):
        rfidw_workers(monkeypatch, run=False)
        calls = self._stub(monkeypatch)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod._classic_write_run(target, 5, self.DATA, self.KEY) == (
            None, "the write never ran")
        assert calls == []
        assert reactor.completions[0].completed == []
        assert reactor.completions[0].waited == [190.0]


class TestCmdAFCRFIDClassicWrite:
    DATA_HEX = "00112233445566778899aabbccddeeff"
    DATA = (b"\x00\x11\x22\x33\x44\x55\x66\x77"
            b"\x88\x99\xaa\xbb\xcc\xdd\xee\xff")

    @staticmethod
    def _setup(monkeypatch: pytest.MonkeyPatch,
               result: Tuple[Optional[str], Optional[str]] = ("01020304", None)
               ) -> Tuple[MockPrinter, List[Tuple[int, bytes, bytes]]]:
        """
        A printer with one reader, and a classic_write_block that records.

        :param monkeypatch: pytest's monkeypatch
        :param result: the (uid, error) the block write returns
        :return tuple: (printer, block writes)
        """
        printer = MockPrinter()
        rfidw_register(printer)
        calls: List[Tuple[int, bytes, bytes]] = []

        def _write(link: Any, block: int, data16: bytes,
                   key6: bytes) -> Tuple[Optional[str], Optional[str]]:
            calls.append((block, data16, key6))
            return result
        monkeypatch.setattr(rfid_write_mod, "classic_write_block", _write)
        return printer, calls

    def _refused(self, monkeypatch: pytest.MonkeyPatch,
                 **params: Any) -> Tuple[str, MockGCodeCommand, List[Any]]:
        """
        Run the command where it must refuse before writing.

        :param monkeypatch: pytest's monkeypatch
        :param params: the command's parameters
        :return tuple: (error message, gcmd, block writes)
        """
        printer, calls = self._setup(monkeypatch)
        gcmd = rfidw_gcmd(**params)
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_CLASSIC_WRITE, printer, gcmd)
        assert gcmd.respond_info.call_args_list == []
        return msg, gcmd, calls

    def test_no_reader_is_a_command_error(self):
        gcmd = rfidw_gcmd(BLOCK="4", DATA=self.DATA_HEX)
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_CLASSIC_WRITE,
                                  MockPrinter(), gcmd)
        assert msg == f"AFC_RFID_CLASSIC_WRITE: {RFIDW_NO_READERS}"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.get_int.call_args_list == []
        assert gcmd.respond_info.call_args_list == []

    def test_data_that_is_not_hex_is_refused(self, monkeypatch):
        msg, gcmd, calls = self._refused(monkeypatch, READER="bt:reader0", BLOCK="4",
                                         DATA="zz" * 16)
        assert msg == "AFC_RFID_CLASSIC_WRITE: DATA is not hex"
        assert gcmd.error.call_args_list == [call(msg)]
        assert calls == []

    def test_data_of_the_wrong_length_is_refused(self, monkeypatch):
        msg, gcmd, calls = self._refused(monkeypatch, READER="bt:reader0", BLOCK="4",
                                         DATA="0011223344556677")
        assert msg == ("AFC_RFID_CLASSIC_WRITE: DATA is 8 bytes, need 16 "
                       "(32 hex chars)")
        assert gcmd.error.call_args_list == [call(msg)]
        assert calls == []

    def test_missing_data_is_zero_bytes(self, monkeypatch):
        msg, gcmd, calls = self._refused(monkeypatch, READER="bt:reader0", BLOCK="4")
        assert msg == ("AFC_RFID_CLASSIC_WRITE: DATA is 0 bytes, need 16 "
                       "(32 hex chars)")
        assert gcmd.error.call_args_list == [call(msg)]
        assert calls == []

    def test_a_key_that_is_not_hex_is_refused(self, monkeypatch):
        msg, gcmd, calls = self._refused(monkeypatch, READER="bt:reader0", BLOCK="4",
                                         DATA=self.DATA_HEX, KEY="nothexnothex")
        assert msg == "AFC_RFID_CLASSIC_WRITE: KEY is not hex"
        assert gcmd.error.call_args_list == [call(msg)]
        assert calls == []

    def test_a_key_of_the_wrong_length_is_refused(self, monkeypatch):
        msg, gcmd, calls = self._refused(monkeypatch, READER="bt:reader0", BLOCK="4",
                                         DATA=self.DATA_HEX, KEY="FFFFFFFF")
        assert msg == "AFC_RFID_CLASSIC_WRITE: KEY must be 12 hex chars"
        assert gcmd.error.call_args_list == [call(msg)]
        assert calls == []

    def test_a_given_key_is_used(self, monkeypatch):
        printer, calls = self._setup(monkeypatch)
        gcmd = rfidw_gcmd(READER="bt:reader0", BLOCK="9", DATA=f" {self.DATA_HEX} ",
                          KEY=" A0A1A2A3A4A5 ")
        rfid_write_mod.cmd_AFC_RFID_CLASSIC_WRITE(printer, gcmd)
        assert calls == [(9, self.DATA, b"\xa0\xa1\xa2\xa3\xa4\xa5")]
        assert gcmd.respond_info.call_args_list == [call(
            "AFC_RFID_CLASSIC_WRITE: wrote block 9 on tag 01020304 "
            "(bt:reader0), verified.")]
        assert gcmd.error.call_args_list == []

    def test_no_key_uses_the_blank_card_key(self, monkeypatch):
        printer, calls = self._setup(monkeypatch)
        gcmd = rfidw_gcmd(READER="bt:reader0", BLOCK="4", DATA=self.DATA_HEX)
        rfid_write_mod.cmd_AFC_RFID_CLASSIC_WRITE(printer, gcmd)
        assert calls == [(4, self.DATA, b"\xff\xff\xff\xff\xff\xff")]
        assert gcmd.get_int.call_args_list == [call("BLOCK", minval=1, maxval=62)]
        assert gcmd.respond_info.call_args_list == [call(
            "AFC_RFID_CLASSIC_WRITE: wrote block 4 on tag 01020304 "
            "(bt:reader0), verified.")]
        assert gcmd.error.call_args_list == []

    def test_a_write_error_names_the_tag(self, monkeypatch):
        printer, _calls = self._setup(monkeypatch, ("01020304", "key refused"))
        gcmd = rfidw_gcmd(READER="bt:reader0", BLOCK="4", DATA=self.DATA_HEX)
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_CLASSIC_WRITE, printer, gcmd)
        assert msg == "AFC_RFID_CLASSIC_WRITE: key refused (tag 01020304)"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []

    def test_a_write_error_without_a_tag(self, monkeypatch):
        printer, _calls = self._setup(monkeypatch, (None, RFIDW_NO_TAG))
        gcmd = rfidw_gcmd(READER="bt:reader0", BLOCK="4", DATA=self.DATA_HEX)
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_CLASSIC_WRITE, printer, gcmd)
        assert msg == f"AFC_RFID_CLASSIC_WRITE: {RFIDW_NO_TAG}"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []


class TestCmdAFCRFIDErase:
    def test_no_reader_is_a_command_error(self):
        gcmd = rfidw_gcmd()
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_ERASE, MockPrinter(), gcmd)
        assert msg == f"AFC_RFID_ERASE: {RFIDW_NO_READERS}"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []

    def test_an_unknown_reader_is_a_command_error(self, monkeypatch):
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        printer = MockPrinter()
        rfidw_register(printer)
        gcmd = rfidw_gcmd(READER="nope")
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_ERASE, printer, gcmd)
        assert msg == "AFC_RFID_ERASE: no reader called nope. Available: bt:reader0"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []
        assert writer.calls == []

    def test_an_erase_error_names_the_tag(self, monkeypatch):
        printer = MockPrinter()
        rfidw_register(printer)
        monkeypatch.setattr(rfid_write_mod, "write_tag",
                            RfidwWriteTag(("01020304", "not an NTAG")))
        gcmd = rfidw_gcmd(READER="bt:reader0")
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_ERASE, printer, gcmd)
        assert msg == "AFC_RFID_ERASE: not an NTAG (tag 01020304)"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []

    def test_an_erase_error_without_a_tag(self):
        printer = MockPrinter()
        rfidw_register(printer, open_link=lambda: None)
        gcmd = rfidw_gcmd(READER="bt:reader0")
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_ERASE, printer, gcmd)
        assert msg == "AFC_RFID_ERASE: bt:reader0 is offline"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []

    def test_a_clean_erase_is_reported(self, monkeypatch):
        printer = MockPrinter()
        rfidw_register(printer)
        writer = RfidwWriteTag(("04aa", None))
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        gcmd = rfidw_gcmd(READER="bt:reader0")
        rfid_write_mod.cmd_AFC_RFID_ERASE(printer, gcmd)
        assert gcmd.respond_info.call_args_list == [call(
            "AFC_RFID_ERASE: erased tag 04aa on bt:reader0. It now reads blank; "
            "write or enroll it again to re-use it.")]
        assert gcmd.error.call_args_list == []
        assert [c[1] for c in writer.calls] == [b"\x00" * 144]


class TestRunRead:
    TAG = {"uid": "04ab", "tag_type": "MifareUltralight"}

    def _worker_read(self, monkeypatch: pytest.MonkeyPatch, threads: RfidwThreading,
                     raises: Optional[Exception] = None) -> List[str]:
        """
        A read_tag that records the thread it ran on and returns TAG.

        :param monkeypatch: pytest's monkeypatch
        :param threads: the threading stand-in
        :param raises: raised by the read when given
        :return list: the thread names, one per read
        """
        ran_on: List[str] = []

        def _read(link: Any, **keys: Any) -> Dict[str, Any]:
            ran_on.append(threads.current_thread().name)
            if raises is not None:
                raise raises
            return self.TAG
        monkeypatch.setattr(rfid_write_mod, "read_tag", _read)
        return ran_on

    def test_a_threaded_read_runs_on_a_named_worker(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch)
        ran_on = self._worker_read(monkeypatch, threads)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_read(target) == (self.TAG, None)
        assert ran_on == ["afc_rfid_rd"]
        assert threads.started == [("afc_rfid_rd", True)]
        assert ffi.names == [b"afc_rfid_rd"]
        assert reactor.completions[0].completed == [None]
        assert reactor.completions[0].waited == [190.0]

    def test_a_thread_name_failure_does_not_stop_the_read(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch, ffi_fails=True)
        ran_on = self._worker_read(monkeypatch, threads)
        target, _reactor = rfidw_threaded()
        assert rfid_write_mod.run_read(target) == (self.TAG, None)
        assert ran_on == ["afc_rfid_rd"]
        assert ffi.names == []

    def test_a_raising_read_becomes_the_error(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch)
        self._worker_read(monkeypatch, threads, raises=RuntimeError("reader wedged"))
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_read(target) == (None, "reader wedged")
        assert reactor.completions[0].completed == [None]

    def test_a_worker_that_never_runs_says_so(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch, run=False)
        ran_on = self._worker_read(monkeypatch, threads)
        target, reactor = rfidw_threaded()
        assert rfid_write_mod.run_read(target) == (None, "the read never ran")
        assert ran_on == []
        assert reactor.completions[0].completed == []
        assert reactor.completions[0].waited == [190.0]

    def test_it_reads_through_the_link_and_reports(self, monkeypatch):
        link = RfidwLink()
        tag = {"uid": "04a1b2c3d4e5f6", "tag_type": "MifareUltralight",
               "filament": {"manufacturer": "Polymaker", "type": "PLA",
                            "weight_g": 1000, "color_argb": 0xFF1A2B3C}}
        reader = RfidwReadTag(tag)
        monkeypatch.setattr(rfid_write_mod, "read_tag", reader)
        target = rfidw_register(MockPrinter(), unit=RfidwUnit(bambu_master_key=b"k"),
                                open_link=lambda: link)
        assert rfid_write_mod.run_read(target) == (tag, None)
        assert reader.calls == [(link, {"bambu_master_key": b"k", "creality_key": None,
                                        "creality_encryption_key": None})]

    def test_the_unit_keys_reach_read_tag(self, monkeypatch):
        reader = RfidwReadTag({"uid": "04"})
        monkeypatch.setattr(rfid_write_mod, "read_tag", reader)
        unit = RfidwUnit(bambu_master_key=b"master", creality_key=b"ck",
                         creality_encryption_key=b"cek")
        target = rfidw_register(MockPrinter(), "ace2:slot0", "ACE2 slot0", unit=unit)
        assert rfid_write_mod.run_read(target) == ({"uid": "04"}, None)
        assert [keys for _link, keys in reader.calls] == [
            {"bambu_master_key": b"master", "creality_key": b"ck",
             "creality_encryption_key": b"cek"}]

    def test_no_tag_is_reported(self, monkeypatch):
        reader = RfidwReadTag(None)
        monkeypatch.setattr(rfid_write_mod, "read_tag", reader)
        target = rfidw_register(MockPrinter(), "bt:reader0", "r")
        assert rfid_write_mod.run_read(target) == (None, RFIDW_NO_TAG)
        assert len(reader.calls) == 1

    def test_an_offline_reader_is_reported(self, monkeypatch):
        reader = RfidwReadTag({"uid": "04"})
        monkeypatch.setattr(rfid_write_mod, "read_tag", reader)
        target = rfidw_register(MockPrinter(), "bt:reader0", "r", open_link=lambda: None)
        assert rfid_write_mod.run_read(target) == (None, "bt:reader0 is offline")
        assert reader.calls == []

    def test_a_handoff_reader_says_it_scans_itself(self, monkeypatch):
        """The U1: open_link returns a sentinel with no reg_read, so it cannot
        be read on demand through the link path."""
        seen: List[str] = []
        reader = RfidwReadTag({"uid": "04"})
        monkeypatch.setattr(rfid_write_mod, "read_tag", reader)
        target = rfidw_register(MockPrinter(), "u1:scanner0", "U1 scanner",
                                open_link=object, **rfidw_tracked(seen))
        assert rfid_write_mod.run_read(target) == (
            None, "u1:scanner0 cannot be read on demand here; its reader scans on its own")
        assert reader.calls == []
        assert seen == []

    def test_prepare_and_release_run_around_the_read(self, monkeypatch):
        """The ACE2's identify/power dance must bracket a read too."""
        seen: List[str] = []

        def _read(link: Any, **keys: Any) -> Dict[str, Any]:
            seen.append("read")
            return {"uid": "04"}
        monkeypatch.setattr(rfid_write_mod, "read_tag", _read)
        target = rfidw_register(MockPrinter(), "ace2:slot0", "ACE2 slot0",
                                **rfidw_tracked(seen))
        assert rfid_write_mod.run_read(target) == ({"uid": "04"}, None)
        assert seen == ["prepare", "read", "release"]


class TestFmtTag:
    def test_an_undecoded_tag_says_so(self):
        tag = {"uid": "04ab", "tag_type": None, "filament": None}
        assert rfid_write_mod._fmt_tag(tag) == (
            "uid: 04ab\ntype: None\nfilament: (undecoded: blank or unknown layout)")

    def test_a_chip_without_user_memory_lists_only_the_chip(self):
        tag = {"uid": "04ab", "tag_type": "MifareUltralight", "chip": "NTAG213",
               "user_bytes": 0, "filament": {}}
        assert rfid_write_mod._fmt_tag(tag) == (
            "uid: 04ab\ntype: MifareUltralight\nchip: NTAG213\n"
            "filament: (undecoded: blank or unknown layout)")

    def test_every_field_is_listed_in_a_fixed_order(self):
        fil = {"color_argb": 0xFF0A0B0C, "spool_id": 136, "bed_temp_c": 60,
               "hotend_max_c": 230, "hotend_min_c": 190, "density": 1.24,
               "diameter_mm": 1.75, "weight_g": 1000, "sku": "PM-1",
               "detailed": "Matte", "type": "PLA", "manufacturer": "Polymaker"}
        tag = {"uid": "04ab", "tag_type": "MifareUltralight", "chip": "NTAG215",
               "user_bytes": 504, "filament": fil}
        assert rfid_write_mod._fmt_tag(tag) == "\n".join([
            "uid: 04ab", "type: MifareUltralight", "chip: NTAG215",
            "user memory: 504 bytes", "brand: Polymaker", "material: PLA",
            "detail: Matte", "sku: PM-1", "weight g: 1000", "diameter: 1.75",
            "density: 1.24", "hotend min: 190", "hotend max: 230", "bed: 60",
            "spool id: 136", "colour: #0A0B0C"])

    def test_empty_values_and_a_zero_colour_are_skipped(self):
        fil = {"manufacturer": "", "type": "PETG", "sku": None, "weight_g": 0,
               "color_argb": 0}
        tag = {"uid": "04ab", "tag_type": "MifareUltralight", "filament": fil}
        assert rfid_write_mod._fmt_tag(tag) == "uid: 04ab\ntype: MifareUltralight\nmaterial: PETG"


class TestCmdAFCRFIDRead:
    def test_no_reader_is_a_command_error(self):
        gcmd = rfidw_gcmd()
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_READ, MockPrinter(), gcmd)
        assert msg == f"AFC_RFID_READ: {RFIDW_NO_READERS}"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []

    def test_a_read_error_is_a_command_error(self, monkeypatch):
        printer = MockPrinter()
        rfidw_register(printer)
        monkeypatch.setattr(rfid_write_mod, "read_tag", RfidwReadTag(None))
        gcmd = rfidw_gcmd(READER="bt:reader0")
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_READ, printer, gcmd)
        assert msg == f"AFC_RFID_READ: {RFIDW_NO_TAG}"
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []

    def test_a_tag_is_reported(self, monkeypatch):
        printer = MockPrinter()
        rfidw_register(printer)
        monkeypatch.setattr(rfid_write_mod, "read_tag", RfidwReadTag(
            {"uid": "04ab", "tag_type": "MifareUltralight", "filament": None}))
        gcmd = rfidw_gcmd(READER="bt:reader0")
        rfid_write_mod.cmd_AFC_RFID_READ(printer, gcmd)
        assert gcmd.respond_info.call_args_list == [call(
            "AFC_RFID_READ on bt:reader0\nuid: 04ab\ntype: MifareUltralight\n"
            "filament: (undecoded: blank or unknown layout)")]
        assert gcmd.error.call_args_list == []


class TestCmdAFCRFIDReaders:
    def test_none_registered_says_where_readers_come_from(self):
        gcmd = rfidw_gcmd()
        rfid_write_mod.cmd_AFC_RFID_READERS(MockPrinter(), gcmd)
        assert gcmd.respond_info.call_args_list == [call(
            "AFC_RFID_READERS: none registered. A host-side reader (BoxTurtle, "
            "ACE2, ViViD or OpenAMS) is what provides them; the Bambu AMS and the "
            "U1 read their own tags and cannot write.")]
        assert gcmd.error.call_args_list == []

    def test_each_reader_is_listed_with_its_state(self):
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0", "BoxTurtle reader0")
        rfidw_register(printer, "ace2:slot0", "ACE2 slot 0", open_link=lambda: None)
        gcmd = rfidw_gcmd()
        rfid_write_mod.cmd_AFC_RFID_READERS(printer, gcmd)
        assert gcmd.respond_info.call_args_list == [call(
            "AFC RFID readers (READER= takes any name below)\n"
            "  bt:reader0: BoxTurtle reader0 [online]\n"
            "  ace2:slot0: ACE2 slot 0 [OFFLINE]")]
        assert gcmd.error.call_args_list == []


class TestOverridesFrom:
    def test_no_parameters_give_no_fields(self):
        gcmd = rfidw_gcmd()
        assert rfid_write_mod._overrides_from(gcmd, "AFC_RFID_WRITE") == {}
        assert gcmd.get_int.call_args_list == []
        assert gcmd.get_float.call_args_list == []
        assert gcmd.error.call_args_list == []
        assert gcmd.respond_info.call_args_list == []

    def test_text_fields_are_taken_as_given(self):
        gcmd = rfidw_gcmd(BRAND="Inland", SKU="IN-1", TYPE="PETG")
        assert rfid_write_mod._overrides_from(gcmd, "AFC_RFID_WRITE") == {
            "manufacturer": "Inland", "sku": "IN-1", "ftype": "PETG"}
        assert gcmd.get_int.call_args_list == []
        assert gcmd.get_float.call_args_list == []
        assert gcmd.error.call_args_list == []
        assert gcmd.respond_info.call_args_list == []

    def test_a_colour_gets_an_opaque_alpha(self):
        gcmd = rfidw_gcmd(COLOR="#00ff80")
        assert rfid_write_mod._overrides_from(gcmd, "AFC_RFID_WRITE") == {
            "color_argb": 0xFF00FF80}
        assert gcmd.error.call_args_list == []
        assert gcmd.respond_info.call_args_list == []

    def test_a_bad_colour_is_a_command_error(self):
        gcmd = rfidw_gcmd(COLOR="#zz1122")
        with pytest.raises(CommandError) as exc:
            rfid_write_mod._overrides_from(gcmd, "AFC_RFID_WRITE")
        assert str(exc.value) == "AFC_RFID_WRITE: COLOR=zz1122 is not hex RRGGBB"
        assert gcmd.error.call_args_list == [
            call("AFC_RFID_WRITE: COLOR=zz1122 is not hex RRGGBB")]
        assert gcmd.respond_info.call_args_list == []

    def test_a_bad_colour_is_reported_under_the_calling_command(self):
        gcmd = rfidw_gcmd(COLOR="zz")
        with pytest.raises(CommandError) as exc:
            rfid_write_mod._overrides_from(gcmd, "AFC_RFID_ENROLL")
        assert str(exc.value) == "AFC_RFID_ENROLL: COLOR=zz is not hex RRGGBB"
        assert gcmd.error.call_args_list == [
            call("AFC_RFID_ENROLL: COLOR=zz is not hex RRGGBB")]
        assert gcmd.respond_info.call_args_list == []

    def test_whole_number_fields_are_ints(self):
        gcmd = rfidw_gcmd(WEIGHT="1000", HOTEND_MIN="190", HOTEND_MAX="230", BED="60",
                          DRY_TEMP="55", DRY_TIME="8")
        assert rfid_write_mod._overrides_from(gcmd, "AFC_RFID_WRITE") == {
            "weight_g": 1000, "hotend_min_c": 190, "hotend_max_c": 230,
            "bed_temp_c": 60, "drying_temp_c": 55, "drying_time_h": 8}
        assert gcmd.get_int.call_args_list == [
            call("WEIGHT", 0, minval=0), call("HOTEND_MIN", 0, minval=0),
            call("HOTEND_MAX", 0, minval=0), call("BED", 0, minval=0),
            call("DRY_TEMP", 0, minval=0), call("DRY_TIME", 0, minval=0)]
        assert gcmd.error.call_args_list == []
        assert gcmd.respond_info.call_args_list == []

    def test_decimal_fields_are_floats(self):
        gcmd = rfidw_gcmd(DIAMETER="2.85", DENSITY="1.24")
        assert rfid_write_mod._overrides_from(gcmd, "AFC_RFID_WRITE") == {
            "diameter_mm": 2.85, "density": 1.24}
        assert gcmd.get_float.call_args_list == [
            call("DIAMETER", 0.0, above=0.0), call("DENSITY", 0.0, above=0.0)]
        assert gcmd.error.call_args_list == []
        assert gcmd.respond_info.call_args_list == []


class TestTagFromFields:
    @staticmethod
    def _decoders(monkeypatch: pytest.MonkeyPatch, anycubic: Optional[Dict[str, Any]],
                  afc: Optional[Dict[str, Any]]) -> List[bytes]:
        """
        Swap both decoders for ones that record the image and return fixed fields.

        :param monkeypatch: pytest's monkeypatch
        :param anycubic: what decode_anycubic returns
        :param afc: what decode_afc_block returns
        :return list: the images decoded, in call order
        """
        images: List[bytes] = []
        monkeypatch.setattr(rfid_write_mod, "decode_anycubic",
                            lambda image: images.append(image) or anycubic)
        monkeypatch.setattr(rfid_write_mod, "decode_afc_block",
                            lambda image: images.append(image) or afc)
        return images

    def test_nothing_decoded_leaves_the_filament_empty(self, monkeypatch):
        images = self._decoders(monkeypatch, None, None)
        assert rfid_write_mod.tag_from_fields("04ab", {"ftype": "PLA"}) == {
            "uid": "04ab", "sak": 0, "tag_type": "MifareUltralight", "filament": None}
        # Both decoders see one image: 16 reserved bytes, then the 144-byte record.
        assert len(images) == 2
        assert images[0] == images[1]
        assert len(images[0]) == 160
        assert images[0][:16] == b"\x00" * 16
        assert images[0][16:20] == RFIDW_ANYCUBIC_MAGIC

    def test_only_the_afc_block_decodes(self, monkeypatch):
        self._decoders(monkeypatch, None, {"weight_g": 823})
        assert rfid_write_mod.tag_from_fields("04ab", {"weight_g": 823}) == {
            "uid": "04ab", "sak": 0, "tag_type": "MifareUltralight",
            "filament": {"weight_g": 823}}

    def test_only_the_anycubic_layout_decodes(self, monkeypatch):
        self._decoders(monkeypatch, {"type": "PLA", "weight_g": 1000}, None)
        assert rfid_write_mod.tag_from_fields("04ab", {"ftype": "PLA"})["filament"] == {
            "type": "PLA", "weight_g": 1000}

    def test_the_afc_block_overrides_the_anycubic_layout(self, monkeypatch):
        self._decoders(monkeypatch, {"type": "PLA", "weight_g": 1000},
                       {"weight_g": 823, "spool_id": 7})
        assert rfid_write_mod.tag_from_fields("04ab", {"ftype": "PLA"})["filament"] == {
            "type": "PLA", "weight_g": 823, "spool_id": 7}

    def test_the_tag_is_what_a_read_of_it_would_decode(self):
        fields = {"manufacturer": "Inland", "ftype": "PETG", "color_argb": 0xFFF20808,
                  "weight_g": 1000, "spool_id": 153, "diameter_mm": 1.75,
                  "density": 1.27, "hotend_max_c": 240, "bed_temp_c": 70}
        # length_m is the Anycubic layout's 330 m for a 1000 g spool; the
        # spool id and density come from the AFC block.
        assert rfid_write_mod.tag_from_fields("04ab", fields) == {
            "uid": "04ab", "sak": 0, "tag_type": "MifareUltralight",
            "filament": {"manufacturer": "Inland", "sku": "", "type": "PETG",
                         "color_argb": 0xFFF20808, "diameter_mm": 1.75,
                         "weight_g": 1000, "length_m": 330, "hotend_min_c": 0,
                         "hotend_max_c": 240, "bed_temp_c": 70, "spool_id": 153,
                         "density": 1.27}}


class TestApplyWritten:
    FIELDS = {"manufacturer": "Inland", "ftype": "PETG", "color_argb": 0xFFF20808,
              "weight_g": 1000, "spool_id": 153, "diameter_mm": 1.75, "density": 1.27,
              "hotend_max_c": 240, "bed_temp_c": 70}

    class _Lane:
        """An AFC lane; send_lane_data exists only when a record list is given."""

        def __init__(self, sent: Optional[List[str]] = None) -> None:
            """
            :param sent: when given, send_lane_data appends "sent" to it
            """
            if sent is not None:
                self.send_lane_data = lambda: sent.append("sent")

    def _apply(self, unit: RfidwUnit, lane_name: str = "lane4") -> str:
        """
        :param unit: the reader's owning unit
        :param lane_name: the lane to apply to
        :return str: apply_written's note
        """
        target = rfidw_register(MockPrinter(), unit=unit)
        return rfid_write_mod.apply_written(target, lane_name, "04ab", self.FIELDS)

    def test_an_unknown_lane_is_not_applied(self):
        applied: List[Any] = []
        saved: List[str] = []
        unit = RfidwUnit(afc=RfidwAFC(lanes={"lane1": self._Lane()}, saved=saved),
                         apply_to_lane=lambda lane, tag: applied.append(lane))
        assert self._apply(unit) == "not applied: lane4 is not a lane this unit can set"
        assert applied == []
        assert saved == []

    def test_a_unit_without_apply_is_not_applied(self):
        sent: List[str] = []
        saved: List[str] = []
        unit = RfidwUnit(afc=RfidwAFC(lanes={"lane4": self._Lane(sent)}, saved=saved))
        assert self._apply(unit) == "not applied: lane4 is not a lane this unit can set"
        assert sent == []
        assert saved == []

    def test_a_unit_without_afc_is_not_applied(self):
        applied: List[Any] = []
        unit = RfidwUnit(afc=None, apply_to_lane=lambda lane, tag: applied.append(lane))
        assert self._apply(unit) == "not applied: lane4 is not a lane this unit can set"
        assert applied == []

    def test_an_afc_with_no_lanes_is_not_applied(self):
        applied: List[Any] = []
        unit = RfidwUnit(afc=RfidwAFC(lanes=None),
                         apply_to_lane=lambda lane, tag: applied.append(lane))
        assert self._apply(unit) == "not applied: lane4 is not a lane this unit can set"
        assert applied == []

    def test_send_and_save_are_optional(self):
        applied: List[Any] = []
        lane = self._Lane()
        unit = RfidwUnit(afc=RfidwAFC(lanes={"lane4": lane}),
                         apply_to_lane=lambda ln, tag: applied.append((ln, tag["uid"])))
        assert self._apply(unit) == "applied to lane4"
        assert applied == [(lane, "04ab")]

    def test_a_raising_apply_is_reported(self):
        def _boom(lane: Any, tag: Dict[str, Any]) -> None:
            raise RuntimeError("Spoolman down")
        unit = RfidwUnit(afc=RfidwAFC(lanes={"lane4": self._Lane()}), apply_to_lane=_boom)
        assert self._apply(unit) == "but applying it to lane4 failed: Spoolman down"

    def test_a_units_own_hook_is_used(self):
        seen: List[Any] = []
        unit = RfidwUnit(
            afc=RfidwAFC(lanes={"lane9": self._Lane(seen)}, saved=seen),
            apply_written_tag=lambda ln, tag: seen.append(
                (ln, tag["uid"], tag["filament"]["spool_id"])),
            apply_to_lane=lambda ln, tag: seen.append("shared apply"))
        assert self._apply(unit, "lane9") == "applied to lane9"
        assert seen == [("lane9", "04ab", 153)]

    def test_otherwise_the_shared_apply_runs_and_the_lane_is_saved(self):
        seen: List[Any] = []
        lane = self._Lane(seen)
        unit = RfidwUnit(afc=RfidwAFC(lanes={"lane4": lane}, saved=seen),
                         apply_to_lane=lambda ln, tag: seen.append(
                             ("apply", ln is lane, tag["filament"]["spool_id"])))
        assert self._apply(unit) == "applied to lane4"
        assert seen == [("apply", True, 153), "sent", "saved"]

    def test_a_failed_apply_is_reported_not_raised(self):
        def boom(ln: str, tag: Dict[str, Any]) -> None:
            raise RuntimeError("Spoolman down")
        unit = RfidwUnit(apply_written_tag=boom)
        assert self._apply(unit, "lane9") == "but applying it to lane9 failed: Spoolman down"


class TestCmdAFCRFIDWrite:
    FULL = {"manufacturer": "Inland", "ftype": "PETG", "weight_g": 1000}
    BARE_REPLY = ("AFC_RFID_WRITE: wrote tag 04ab on bt:reader0: unbranded filament. "
                  "Stick it on the spool and scan to confirm.")

    @staticmethod
    def _setup(monkeypatch: pytest.MonkeyPatch,
               result: Tuple[Any, ...] = ("04ab", None, {}, ""),
               raises: Optional[Exception] = None
               ) -> Tuple[MockPrinter, RfidWriteTarget, RfidwCommandRuns]:
        """
        :param monkeypatch: pytest's monkeypatch
        :param result: what run_write returns
        :param raises: what run_write raises instead
        :return tuple: (printer, target, recorder)
        """
        return rfidw_command_setup(monkeypatch, "run_write", result, raises)

    @staticmethod
    def _refused(printer: MockPrinter, **params: Any) -> Tuple[str, MockGCodeCommand]:
        """
        Run AFC_RFID_WRITE where it must refuse, with nothing reported.

        :param printer: the printer
        :param params: the command's parameters
        :return tuple: (error message, gcmd)
        """
        gcmd = rfidw_gcmd(**params)
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_WRITE, printer, gcmd)
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []
        return msg, gcmd

    @staticmethod
    def _replied(printer: MockPrinter, **params: Any) -> List[Any]:
        """
        Run AFC_RFID_WRITE where it must succeed, and return its replies.

        :param printer: the printer
        :param params: the command's parameters
        :return list: the respond_info calls
        """
        gcmd = rfidw_gcmd(**params)
        rfid_write_mod.cmd_AFC_RFID_WRITE(printer, gcmd)
        assert gcmd.error.call_args_list == []
        return gcmd.respond_info.call_args_list

    def test_no_reader_is_a_command_error(self):
        msg, _gcmd = self._refused(MockPrinter(), TYPE="PLA")
        assert msg == f"AFC_RFID_WRITE: {RFIDW_NO_READERS}"

    def test_a_bad_colour_is_reported_under_write(self, monkeypatch):
        printer, _target, runs = self._setup(monkeypatch)
        msg, _gcmd = self._refused(printer, READER="bt:reader0", COLOR="zz")
        assert msg == "AFC_RFID_WRITE: COLOR=zz is not hex RRGGBB"
        assert runs.runs == []

    def test_nothing_to_write_is_refused(self, monkeypatch):
        printer, _target, runs = self._setup(monkeypatch)
        msg, _gcmd = self._refused(printer, READER="bt:reader0")
        assert msg == ("AFC_RFID_WRITE: nothing to write. Give a SPOOL= id to take "
                       "the record from Spoolman, or at least TYPE=.")
        assert runs.runs == []

    def test_a_spool_alone_is_enough_and_gets_no_defaults(self, monkeypatch):
        printer, target, runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0", SPOOL="7") == [
            call(self.BARE_REPLY)]
        assert runs.runs == [(target, 7, {}, None)]

    def test_a_field_alone_is_enough_and_gets_defaults(self, monkeypatch):
        printer, target, runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0", BRAND="Inland") == [
            call(self.BARE_REPLY)]
        assert runs.runs == [(target, 0, {"manufacturer": "Inland", "ftype": "PLA",
                                          "diameter_mm": 1.75}, None)]

    def test_given_fields_beat_the_defaults(self, monkeypatch):
        printer, target, runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0", TYPE="PETG",
                             DIAMETER="2.85") == [call(self.BARE_REPLY)]
        assert runs.runs == [(target, 0, {"ftype": "PETG", "diameter_mm": 2.85}, None)]

    def test_a_stage_refusal_is_a_command_error(self, monkeypatch):
        printer, _target, runs = self._setup(
            monkeypatch, raises=StageError("lane9 is loaded in the toolhead"))
        msg, _gcmd = self._refused(printer, READER="bt:reader0", LANE="lane9", TYPE="PLA")
        assert msg == "AFC_RFID_WRITE: lane9 is loaded in the toolhead"
        assert runs.applies == []

    def test_a_write_error_names_the_tag(self, monkeypatch):
        printer, _target, runs = self._setup(
            monkeypatch, ("04ab", "tag did not ACK page 9", {}, ""))
        msg, _gcmd = self._refused(printer, READER="bt:reader0", LANE="lane9", TYPE="PLA")
        assert msg == "AFC_RFID_WRITE: tag did not ACK page 9 (tag 04ab)"
        assert runs.applies == []

    def test_a_write_error_without_a_tag(self, monkeypatch):
        printer, _target, _runs = self._setup(monkeypatch, (None, RFIDW_NO_TAG, {}, ""))
        msg, _gcmd = self._refused(printer, READER="bt:reader0", TYPE="PLA")
        assert msg == f"AFC_RFID_WRITE: {RFIDW_NO_TAG}"

    def test_the_reply_lists_brand_type_weight_and_note(self, monkeypatch):
        printer, _target, runs = self._setup(
            monkeypatch, ("04ab", None, self.FULL, "spool 7, uid bound"))
        assert self._replied(printer, READER="bt:reader0", SPOOL="7") == [call(
            "AFC_RFID_WRITE: wrote tag 04ab on bt:reader0: Inland PETG, 1000g "
            "(spool 7, uid bound). Stick it on the spool and scan to confirm.")]
        assert runs.applies == []

    def test_the_reply_falls_back_when_fields_are_bare(self, monkeypatch):
        printer, _target, _runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0", TYPE="PLA") == [call(
            "AFC_RFID_WRITE: wrote tag 04ab on bt:reader0: unbranded filament. "
            "Stick it on the spool and scan to confirm.")]

    def test_a_lane_apply_joins_the_note(self, monkeypatch):
        printer, target, runs = self._setup(
            monkeypatch, ("04ab", None, self.FULL, "spool 7, uid bound"))
        assert self._replied(printer, READER="bt:reader0", LANE="lane9", SPOOL="7") == [
            call("AFC_RFID_WRITE: wrote tag 04ab on bt:reader0: Inland PETG, 1000g "
                 "(spool 7, uid bound, applied to lane9). Stick it on the spool and "
                 "scan to confirm.")]
        assert runs.runs == [(target, 7, {}, "lane9")]
        assert runs.applies == [(target, "lane9", "04ab", self.FULL)]

    def test_a_lane_apply_alone_is_the_note(self, monkeypatch):
        printer, _target, _runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0", LANE="lane9", TYPE="PLA") == [
            call("AFC_RFID_WRITE: wrote tag 04ab on bt:reader0: unbranded filament "
                 "(applied to lane9). Stick it on the spool and scan to confirm.")]

    @staticmethod
    def _refusing_stage_printer(monkeypatch: pytest.MonkeyPatch
                                ) -> Tuple[MockPrinter, List[str], RfidwWriteTag]:
        """
        A printer whose one reader's stage hook refuses lane9.

        :param monkeypatch: pytest's monkeypatch
        :return tuple: (printer, unstage calls, write_tag stand-in)
        """
        unstaged: List[str] = []

        def _refuse(lane: str) -> None:
            error_str = f"{lane} is loaded in the toolhead"
            raise StageError(error_str)
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        printer = MockPrinter()
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        rfidw_register(printer, unit=rfidw_spoolman_unit(client), stage=_refuse,
                       unstage=lambda tok: unstaged.append("unstage"))
        return printer, unstaged, writer

    # Flagged multi: the enroll command shares the stage refusal, so both run here.
    @pytest.mark.parametrize("command, refusal", [
        ("AFC_RFID_WRITE", "AFC_RFID_WRITE: lane9 is loaded in the toolhead"),
        ("AFC_RFID_ENROLL", "AFC_RFID_ENROLL: lane9 is loaded in the toolhead")])
    def test_a_refused_stage_is_a_command_error_and_writes_nothing(self, monkeypatch,
                                                                    command, refusal):
        printer, unstaged, writer = self._refusing_stage_printer(monkeypatch)
        gcmd = rfidw_gcmd(READER="bt:reader0", LANE="lane9", SPOOL=7)
        with pytest.raises(CommandError) as exc:
            printer._gcode._commands[command](gcmd)
        assert str(exc.value) == refusal
        assert gcmd.error.call_args_list == [call(refusal)]
        assert gcmd.respond_info.call_args_list == []
        assert writer.calls == []
        assert unstaged == []

    def test_the_write_command_applies_to_the_named_lane(self, monkeypatch):
        seen: List[str] = []
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0", "x",
                       unit=RfidwUnit(apply_written_tag=lambda ln, tag: seen.append(ln)))
        monkeypatch.setattr(rfid_write_mod, "write_tag", RfidwWriteTag())
        gcmd = rfidw_gcmd(READER="bt:reader0", LANE="lane9", TYPE="PLA")
        printer._gcode._commands["AFC_RFID_WRITE"](gcmd)
        assert seen == ["lane9"]
        assert gcmd.respond_info.call_args_list == [call(
            "AFC_RFID_WRITE: wrote tag 04ab on bt:reader0: unbranded PLA "
            "(applied to lane9). Stick it on the spool and scan to confirm.")]
        assert gcmd.error.call_args_list == []

    def test_no_lane_applies_nothing(self, monkeypatch):
        seen: List[str] = []
        printer = MockPrinter()
        rfidw_register(printer, "bt:reader0", "x",
                       unit=RfidwUnit(apply_written_tag=lambda ln, tag: seen.append(ln)))
        monkeypatch.setattr(rfid_write_mod, "write_tag", RfidwWriteTag())
        gcmd = rfidw_gcmd(READER="bt:reader0", TYPE="PLA")
        printer._gcode._commands["AFC_RFID_WRITE"](gcmd)
        assert seen == []
        assert gcmd.respond_info.call_args_list == [call(
            "AFC_RFID_WRITE: wrote tag 04ab on bt:reader0: unbranded PLA. "
            "Stick it on the spool and scan to confirm.")]
        assert gcmd.error.call_args_list == []


class TestCreateSpoolBlocking:
    def test_no_client_creates_nothing(self):
        # The real _cached_spoolman_client finds no moonraker on the AFC.
        unit = RfidwUnit(afc=RfidwAFC())
        assert rfid_write_mod._create_spool_blocking(unit, {"ftype": "PLA"}) == (
            0, "no Spoolman client")

    def test_no_afc_creates_nothing(self):
        unit = RfidwUnit(afc=None)
        assert rfid_write_mod._create_spool_blocking(unit, {"ftype": "PLA"}) == (
            0, "no Spoolman client")

    def test_bare_fields_fall_back_to_generic_pla(self):
        client = RfidwSpoolman(vendor="not a dict", filament={"id": 42},
                               created={"id": "12"})
        assert rfid_write_mod._create_spool_blocking(rfidw_spoolman_unit(client), {}) == (
            12, None)
        assert client.calls == [("vendor", "Generic"), rfidw_filament_call("Generic PLA"),
                                ("spool", 42, None)]

    def test_full_fields_reach_spoolman(self):
        client = RfidwSpoolman(vendor={"id": 7}, filament={"id": 42}, created={"id": 99})
        fields = {"manufacturer": "Inland", "ftype": "PETG", "color_argb": 0xFF0A0B0C,
                  "weight_g": 1000, "sku": "IN-PETG-RED", "density": 1.27,
                  "diameter_mm": 1.75, "hotend_max_c": 240, "bed_temp_c": 70}
        assert rfid_write_mod._create_spool_blocking(rfidw_spoolman_unit(client),
                                                     fields) == (99, None)
        assert client.calls == [
            ("vendor", "Inland"),
            rfidw_filament_call("IN-PETG-RED", 7, "PETG", density=1.27, diameter=1.75,
                                color_hex="0A0B0C", settings_extruder_temp=240,
                                settings_bed_temp=70, weight=1000,
                                article_number="IN-PETG-RED"),
            ("spool", 42, 1000)]

    def test_an_empty_sku_and_zero_temps_are_left_unset(self):
        client = RfidwSpoolman(vendor={"id": 7}, filament={"id": 42}, created={"id": 99})
        fields = {"manufacturer": "Inland", "ftype": "PETG", "sku": "",
                  "hotend_max_c": 0, "bed_temp_c": 0}
        assert rfid_write_mod._create_spool_blocking(rfidw_spoolman_unit(client),
                                                     fields) == (99, None)
        assert client.calls == [("vendor", "Inland"),
                                rfidw_filament_call("Inland PETG", 7, "PETG"),
                                ("spool", 42, None)]

    def test_a_filament_without_an_id_is_a_failure(self):
        client = RfidwSpoolman(vendor={"id": 7}, filament={"name": "x"},
                               created={"id": 99})
        assert rfid_write_mod._create_spool_blocking(rfidw_spoolman_unit(client), {}) == (
            0, "Spoolman filament create failed ({'name': 'x'})")
        assert client.calls == [("vendor", "Generic"), rfidw_filament_call("Generic PLA", 7)]

    def test_a_filament_that_is_not_a_dict_is_a_failure(self):
        client = RfidwSpoolman(vendor={"id": 7}, filament=None, created={"id": 99})
        assert rfid_write_mod._create_spool_blocking(rfidw_spoolman_unit(client), {}) == (
            0, "Spoolman filament create failed (None)")
        assert client.calls == [("vendor", "Generic"), rfidw_filament_call("Generic PLA", 7)]

    def test_a_spool_that_is_not_a_dict_is_a_failure(self):
        client = RfidwSpoolman(vendor={"id": 7}, filament={"id": 42}, created=None)
        assert rfid_write_mod._create_spool_blocking(rfidw_spoolman_unit(client), {}) == (
            0, "Spoolman spool create failed (None)")
        assert client.calls[-1] == ("spool", 42, None)

    def test_a_spool_without_an_id_is_a_failure(self):
        client = RfidwSpoolman(vendor={"id": 7}, filament={"id": 42},
                               created={"filament_id": 42})
        assert rfid_write_mod._create_spool_blocking(rfidw_spoolman_unit(client), {}) == (
            0, "Spoolman spool create failed ({'filament_id': 42})")
        assert client.calls[-1] == ("spool", 42, None)

    def test_a_spoolman_exception_is_reported(self):
        client = RfidwSpoolman(create_error=RuntimeError("connection refused"))
        assert rfid_write_mod._create_spool_blocking(rfidw_spoolman_unit(client), {}) == (
            0, "Spoolman create failed: connection refused")
        assert client.calls == [("vendor", "Generic")]


class TestRunEnroll:
    def test_an_offline_reader_creates_no_spool(self, monkeypatch):
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        client = RfidwSpoolman(vendor={"id": 7}, filament={"id": 42}, created={"id": 99})
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client),
                                open_link=lambda: None)
        assert rfid_write_mod.run_enroll(target, 0, {"ftype": "PLA"}) == (
            None, "bt:reader0 is offline", {}, "")
        assert client.calls == []
        assert writer.calls == []

    def test_a_reader_that_cannot_read_on_demand_still_enrolls(self):
        client = RfidwSpoolman(spool={"filament": {"material": "PETG"},
                                      "initial_weight": 1000},
                               vendor={"id": 7}, filament={"id": 42}, created={"id": 99})
        payloads: List[bytes] = []
        target = rfidw_register(
            MockPrinter(), "u1:scanner0", unit=rfidw_spoolman_unit(client),
            open_link=object,
            write_payload=lambda pay: payloads.append(pay) or ("04ffee", None))
        assert rfid_write_mod.run_enroll(target, 0, {"ftype": "PETG", "weight_g": 1000}) == (
            "04ffee", None, {"spool_id": 99, "ftype": "PETG", "weight_g": 1000},
            "created spool 99; spool 99, uid bound")
        assert client.calls == [("vendor", "Generic"),
                                rfidw_filament_call("Generic PETG", 7, "PETG", weight=1000),
                                ("spool", 42, 1000)]
        assert client.read == [99]
        assert client.bound == [(99, "04ffee")]
        assert len(payloads) == 1

    def test_a_created_spool_keeps_its_record_and_adds_tag_only_fields(self, monkeypatch):
        client = RfidwSpoolman(
            spool={"filament": {"material": "PLA", "name": "Generic PLA",
                                "vendor": {"name": "Inland"}}},
            vendor={"id": 7}, filament={"id": 42}, created={"id": 99})
        monkeypatch.setattr(rfid_write_mod, "read_tag", RfidwReadTag({"uid": "04ab"}))
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        typed = {"manufacturer": "inland", "sku": "", "ftype": "PLA",
                 "hotend_min_c": 190, "drying_temp_c": 55, "drying_time_h": 8}
        assert rfid_write_mod.run_enroll(target, 0, typed) == (
            "04ab", None,
            {"spool_id": 99, "manufacturer": "Inland", "ftype": "PLA",
             "sku": "Generic PLA", "hotend_min_c": 190, "drying_temp_c": 55,
             "drying_time_h": 8},
            "created spool 99; spool 99, uid bound")
        assert client.calls == [("vendor", "inland"),
                                rfidw_filament_call("inland PLA", 7),
                                ("spool", 42, None)]
        # The new record's brand and name win; the fields Spoolman cannot hold
        # come from the typed params.
        assert len(writer.calls) == 1
        image = b"\x00" * 16 + writer.calls[0][1]
        anycubic = decode_anycubic(image)
        assert (anycubic["manufacturer"], anycubic["hotend_min_c"]) == ("Inland", 190)
        assert decode_afc_block(image) == {
            "spool_id": 99, "drying_temp_c": 55, "drying_time_h": 8}
        assert client.bound == [(99, "04ab")]

    def test_a_failed_write_names_the_spool_it_created(self, monkeypatch):
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}},
                               vendor={"id": 7}, filament={"id": 42}, created={"id": 99})
        monkeypatch.setattr(rfid_write_mod, "read_tag", RfidwReadTag({"uid": "04ab"}))
        monkeypatch.setattr(rfid_write_mod, "write_tag",
                            RfidwWriteTag(("04ab", "not an NTAG")))
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 0, {"ftype": "PLA"}) == (
            "04ab",
            "not an NTAG. Spool 99 was created in Spoolman: give SPOOL=99 when you "
            "enroll a writable tag, or delete it",
            {"spool_id": 99, "ftype": "PLA"}, "created spool 99; spool 99")
        assert [c[0] for c in client.calls] == ["vendor", "filament", "spool"]
        assert client.bound == []

    def test_a_failed_write_to_a_linked_spool_has_no_retry_hint(self, monkeypatch):
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        reader = RfidwReadTag({"uid": "04ab"})
        monkeypatch.setattr(rfid_write_mod, "read_tag", reader)
        monkeypatch.setattr(rfid_write_mod, "write_tag",
                            RfidwWriteTag(("04ab", "not an NTAG")))
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 7, {}) == (
            "04ab", "not an NTAG", {"spool_id": 7, "ftype": "PLA"}, "spool 7")
        assert client.calls == []
        assert client.bound == []
        assert reader.calls == []

    def test_an_unreadable_linked_spool_is_reported(self, monkeypatch):
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        client = RfidwSpoolman(spool=None)
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 5, {"ftype": "PLA"}) == (
            None, "Spoolman has no spool 5", {}, "")
        assert client.read == [5]
        assert writer.calls == []

    def test_a_threaded_enroll_runs_on_a_named_worker(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target, reactor = rfidw_threaded(unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 7, {}) == (
            "04ab", None, {"spool_id": 7, "ftype": "PLA"}, "spool 7, uid bound")
        assert ran_on == [("afc_rfid_en", None)]
        assert threads.started == [("afc_rfid_en", True)]
        assert ffi.names == [b"afc_rfid_en"]
        assert reactor.completions[0].completed == [None]
        assert reactor.completions[0].waited == [220.0]

    def test_a_thread_name_failure_does_not_stop_the_enroll(self, monkeypatch):
        threads, ffi = rfidw_workers(monkeypatch, ffi_fails=True)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target, _reactor = rfidw_threaded(unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 7, {}) == (
            "04ab", None, {"spool_id": 7, "ftype": "PLA"}, "spool 7, uid bound")
        assert ran_on == [("afc_rfid_en", None)]
        assert ffi.names == []

    def test_a_raising_enroll_becomes_the_error(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch)
        rfidw_worker_write(monkeypatch, threads, raises=RuntimeError("serial went away"))
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target, reactor = rfidw_threaded(unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 7, {}) == (
            None, "serial went away", {}, "")
        assert reactor.completions[0].completed == [None]
        assert client.bound == []

    def test_a_worker_that_never_runs_says_so(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch, run=False)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target, reactor = rfidw_threaded(unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 7, {}) == (
            None, "the enroll never ran", {}, "")
        assert ran_on == []
        assert client.read == []
        assert reactor.completions[0].completed == []
        assert reactor.completions[0].waited == [220.0]

    def test_a_threaded_enroll_passes_the_staged_exclusions_to_the_worker(self, monkeypatch):
        threads, _ffi = rfidw_workers(monkeypatch)
        ran_on = rfidw_worker_write(monkeypatch, threads)
        tokens: List[str] = []

        def is_sister(uid: str) -> bool:
            return uid == "cafe"
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target, _reactor = rfidw_threaded(
            unit=rfidw_spoolman_unit(client), stage=lambda lane: "tok",
            exclude=lambda tok: tokens.append(tok) or is_sister)
        assert rfid_write_mod.run_enroll(target, 7, {}, lane="lane4") == (
            "04ab", None, {"spool_id": 7, "ftype": "PLA"}, "spool 7, uid bound")
        assert tokens == ["tok"]
        assert ran_on == [("afc_rfid_en", is_sister)]

    def test_enroll_stages_too(self, monkeypatch):
        seen: List[Any] = []
        monkeypatch.setattr(rfid_write_mod, "write_tag", RfidwWriteTag(seen=seen))
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target = rfidw_register(
            MockPrinter(), unit=rfidw_spoolman_unit(client),
            stage=lambda lane: seen.append(("stage", lane)) or ("tok", lane),
            unstage=lambda tok: seen.append(("unstage", tok)))
        assert rfid_write_mod.run_enroll(target, 7, {"ftype": "PLA"}, lane="lane11") == (
            "04ab", None, {"spool_id": 7, "ftype": "PLA"}, "spool 7, uid bound")
        assert seen == [("stage", "lane11"), "write", ("unstage", ("tok", "lane11"))]

    def test_linking_an_existing_spool_writes_and_binds(self, monkeypatch):
        monkeypatch.setattr(rfid_write_mod, "write_tag",
                            RfidwWriteTag(("04a1b2c3d4e5f6", None)))
        client = RfidwSpoolman(spool=rfidw_spool_record())
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 136, {}) == (
            "04a1b2c3d4e5f6", None, RFIDW_SPOOL_FIELDS, "spool 136, uid bound")
        assert client.bound == [(136, "04a1b2c3d4e5f6")]
        assert client.calls == []

    def test_a_new_spool_is_created_then_written(self, monkeypatch):
        client = RfidwSpoolman(
            spool={"id": 99, "filament": {"name": "Inland PETG", "material": "PETG",
                                          "weight": 1000}},
            vendor={"id": 7, "name": "Inland"}, filament={"id": 42}, created={"id": 99})
        monkeypatch.setattr(rfid_write_mod, "read_tag", RfidwReadTag({"uid": "04ffee"}))
        monkeypatch.setattr(rfid_write_mod, "write_tag", RfidwWriteTag(("04ffee", None)))
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(
            target, 0, {"manufacturer": "Inland", "ftype": "PETG",
                        "color_argb": 0xFF00FF00, "weight_g": 1000,
                        "diameter_mm": 1.75}) == (
            "04ffee", None,
            {"spool_id": 99, "ftype": "PETG", "sku": "Inland PETG", "weight_g": 1000},
            "created spool 99; spool 99, uid bound")
        assert client.calls == [
            ("vendor", "Inland"),
            rfidw_filament_call("Inland PETG", 7, "PETG", diameter=1.75,
                                color_hex="00FF00", weight=1000),
            ("spool", 42, 1000)]
        assert client.bound == [(99, "04ffee")]

    def test_a_spoolman_create_failure_is_reported(self, monkeypatch):
        client = RfidwSpoolman(vendor={"id": 7}, filament=None, created={"id": 99})
        monkeypatch.setattr(rfid_write_mod, "read_tag", RfidwReadTag({"uid": "04ffee"}))
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 0, {"ftype": "PLA"}) == (
            None, "Spoolman filament create failed (None)", {}, "")
        assert writer.calls == []
        assert client.read == []

    def test_no_tag_present_creates_no_orphan_spool(self, monkeypatch):
        """The orphan guard: a missing tag must not leave a Spoolman spool."""
        client = RfidwSpoolman(vendor={"id": 7}, filament={"id": 42}, created={"id": 99})
        monkeypatch.setattr(rfid_write_mod, "read_tag", RfidwReadTag(None))
        writer = RfidwWriteTag()
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        target = rfidw_register(MockPrinter(), unit=rfidw_spoolman_unit(client))
        assert rfid_write_mod.run_enroll(target, 0, {"ftype": "PLA"}) == (
            None, RFIDW_NO_TAG, {}, "")
        assert client.calls == []
        assert writer.calls == []

    def test_enroll_runs_inside_the_scan_too(self, monkeypatch):
        seen: List[Any] = []

        def excl(uid: str) -> bool:
            return uid == "cafe"

        def around(lane: str, body: Callable[..., Any]) -> None:
            seen.append(("scan", lane))
            body(excl)
            seen.append("unwind")
        writer = RfidwWriteTag(seen=seen)
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}})
        target = rfidw_register(MockPrinter(), "oams:RFID_A", "OpenAMS RFID_A",
                                unit=rfidw_spoolman_unit(client), stage_around=around)
        assert rfid_write_mod.run_enroll(target, 7, {}, lane="lane4") == (
            "04ab", None, {"spool_id": 7, "ftype": "PLA"}, "spool 7, uid bound")
        assert seen == [("scan", "lane4"), "write", "unwind"]
        assert len(writer.calls) == 1
        assert writer.calls[0][2] is excl

    def test_a_created_spool_is_written_inside_the_scan_too(self, monkeypatch):
        seen: List[Any] = []

        def excl(uid: str) -> bool:
            return uid == "cafe"

        def around(lane: str, body: Callable[..., Any]) -> None:
            seen.append(("scan", lane))
            body(excl)
            seen.append("unwind")
        monkeypatch.setattr(rfid_write_mod, "read_tag", RfidwReadTag({"uid": "04ab"}))
        writer = RfidwWriteTag(seen=seen)
        monkeypatch.setattr(rfid_write_mod, "write_tag", writer)
        client = RfidwSpoolman(spool={"filament": {"material": "PLA"}},
                               vendor={"id": 7}, filament={"id": 42}, created={"id": 99})
        target = rfidw_register(MockPrinter(), "oams:RFID_A", "OpenAMS RFID_A",
                                unit=rfidw_spoolman_unit(client), stage_around=around)
        assert rfid_write_mod.run_enroll(target, 0, {"ftype": "PLA"}, lane="lane4") == (
            "04ab", None, {"spool_id": 99, "ftype": "PLA"},
            "created spool 99; spool 99, uid bound")
        assert seen == [("scan", "lane4"), "write", "unwind"]
        assert len(writer.calls) == 1
        assert writer.calls[0][2] is excl
        assert client.bound == [(99, "04ab")]


class TestCmdAFCRFIDEnroll:
    FULL = {"manufacturer": "Inland", "ftype": "PETG", "weight_g": 1000}

    @staticmethod
    def _setup(monkeypatch: pytest.MonkeyPatch,
               result: Tuple[Any, ...] = ("04ab", None, {}, ""),
               raises: Optional[Exception] = None
               ) -> Tuple[MockPrinter, RfidWriteTarget, RfidwCommandRuns]:
        """
        :param monkeypatch: pytest's monkeypatch
        :param result: what run_enroll returns
        :param raises: what run_enroll raises instead
        :return tuple: (printer, target, recorder)
        """
        return rfidw_command_setup(monkeypatch, "run_enroll", result, raises)

    @staticmethod
    def _refused(printer: MockPrinter, **params: Any) -> str:
        """
        Run AFC_RFID_ENROLL where it must refuse, with nothing reported.

        :param printer: the printer
        :param params: the command's parameters
        :return str: the error message
        """
        gcmd = rfidw_gcmd(**params)
        msg = rfidw_command_error(rfid_write_mod.cmd_AFC_RFID_ENROLL, printer, gcmd)
        assert gcmd.error.call_args_list == [call(msg)]
        assert gcmd.respond_info.call_args_list == []
        return msg

    @staticmethod
    def _replied(printer: MockPrinter, **params: Any) -> List[Any]:
        """
        Run AFC_RFID_ENROLL where it must succeed, and return its replies.

        :param printer: the printer
        :param params: the command's parameters
        :return list: the respond_info calls
        """
        gcmd = rfidw_gcmd(**params)
        rfid_write_mod.cmd_AFC_RFID_ENROLL(printer, gcmd)
        assert gcmd.error.call_args_list == []
        return gcmd.respond_info.call_args_list

    def test_no_reader_is_a_command_error(self):
        assert self._refused(MockPrinter()) == f"AFC_RFID_ENROLL: {RFIDW_NO_READERS}"

    def test_a_bad_colour_is_reported_under_enroll(self, monkeypatch):
        printer, _target, runs = self._setup(monkeypatch)
        assert self._refused(printer, READER="bt:reader0", COLOR="zz") == (
            "AFC_RFID_ENROLL: COLOR=zz is not hex RRGGBB")
        assert runs.runs == []

    def test_no_spool_gets_the_pla_defaults(self, monkeypatch):
        printer, target, runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0") == [call(
            "AFC_RFID_ENROLL: tag 04ab on bt:reader0: unbranded filament.")]
        assert runs.runs == [(target, 0, {"ftype": "PLA", "diameter_mm": 1.75}, None)]

    def test_a_linked_spool_gets_no_defaults(self, monkeypatch):
        printer, target, runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0", SPOOL="7") == [call(
            "AFC_RFID_ENROLL: tag 04ab on bt:reader0: unbranded filament.")]
        assert runs.runs == [(target, 7, {}, None)]

    def test_a_stage_refusal_is_a_command_error(self, monkeypatch):
        printer, _target, runs = self._setup(
            monkeypatch, raises=StageError("lane9 is loaded in the toolhead"))
        assert self._refused(printer, READER="bt:reader0", LANE="lane9") == (
            "AFC_RFID_ENROLL: lane9 is loaded in the toolhead")
        assert runs.applies == []

    def test_an_enroll_error_names_the_tag(self, monkeypatch):
        printer, _target, runs = self._setup(
            monkeypatch, ("04ab", "tag did not ACK page 9", {}, ""))
        assert self._refused(printer, READER="bt:reader0", LANE="lane9") == (
            "AFC_RFID_ENROLL: tag did not ACK page 9 (tag 04ab)")
        assert runs.applies == []

    def test_an_enroll_error_without_a_tag(self, monkeypatch):
        printer, _target, _runs = self._setup(monkeypatch, (None, RFIDW_NO_TAG, {}, ""))
        assert self._refused(printer, READER="bt:reader0") == (
            f"AFC_RFID_ENROLL: {RFIDW_NO_TAG}")

    def test_the_reply_lists_brand_type_weight_and_note(self, monkeypatch):
        printer, _target, runs = self._setup(
            monkeypatch, ("04ab", None, self.FULL, "created spool 99; spool 99"))
        assert self._replied(printer, READER="bt:reader0") == [call(
            "AFC_RFID_ENROLL: tag 04ab on bt:reader0: Inland PETG, 1000g "
            "(created spool 99; spool 99).")]
        assert runs.applies == []

    def test_the_reply_falls_back_when_fields_are_bare(self, monkeypatch):
        printer, _target, _runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0") == [call(
            "AFC_RFID_ENROLL: tag 04ab on bt:reader0: unbranded filament.")]

    def test_a_lane_apply_joins_the_note(self, monkeypatch):
        printer, target, runs = self._setup(
            monkeypatch, ("04ab", None, self.FULL, "spool 7, uid bound"))
        assert self._replied(printer, READER="bt:reader0", LANE="lane9", SPOOL="7") == [
            call("AFC_RFID_ENROLL: tag 04ab on bt:reader0: Inland PETG, 1000g "
                 "(spool 7, uid bound, applied to lane9).")]
        assert runs.runs == [(target, 7, {}, "lane9")]
        assert runs.applies == [(target, "lane9", "04ab", self.FULL)]

    def test_a_lane_apply_alone_is_the_note(self, monkeypatch):
        printer, _target, _runs = self._setup(monkeypatch)
        assert self._replied(printer, READER="bt:reader0", LANE="lane9") == [call(
            "AFC_RFID_ENROLL: tag 04ab on bt:reader0: unbranded filament "
            "(applied to lane9).")]
