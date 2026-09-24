"""Unit tests for extras/AFC_U1_rfid.py."""

from __future__ import annotations

import configparser
import json
import os
from typing import Any, Callable, Dict, List, Optional, Tuple

import pytest

from extras.AFC_RFID import sync_rfid_to_spoolman
from extras.AFC_U1_rfid import AFC_U1_RFID, load_config
import extras.AFC_U1_rfid as u1_mod
from tests.bambu_helpers import (
    BambuConfig,
    BambuPrinter,
    FakeIdleTimeout,
    LogLine,
    make_afc_spool,
    Recorder,
)


#: The wall clock the reader's ``time`` module reads where a test pins it.
U1_WALL_TIME = 1700000000.0


#: A card UID as filament_detect reports it: a list of byte values.
U1_UID = [0x56, 0xA3, 0x6A, 0xEA]


#: A second spool's card UID.
U1_UID_OTHER = [0x26, 0xA3, 0x6A, 0xEA]


class U1Printer(BambuPrinter):
    """
    BambuPrinter whose ``lookup_object`` knows only registered objects.

    MockPrinter hands out a MagicMock for webhooks, idle_timeout and a few
    other names even when a default is given. klippy returns the default, and
    the reader branches on exactly that (no idle_timeout, no filament_detect).
    """

    def lookup_object(self, name: str,
                      default: Any = BambuPrinter._NO_DEFAULT) -> Any:
        """
        :param name: the object's name
        :param default: returned when it is not registered; without one a
          missing object raises, as klippy's lookup does
        :return Any: the object
        """
        if name in ("AFC", "gcode") or name in self._objects:
            return super().lookup_object(name, default)
        if default is BambuPrinter._NO_DEFAULT:
            error_str = f"Unknown config object '{name}'"
            raise configparser.Error(error_str)
        return default


class U1Config(BambuConfig):
    """BambuConfig whose ``error`` is the exception class, as ConfigWrapper's
    is: the reader builds the error and raises it itself."""

    error = configparser.Error


class U1Webhooks:
    """Klipper's webhooks object: records endpoint registrations, or raises."""

    def __init__(self, raises: Optional[BaseException] = None) -> None:
        """:param raises: raised by every registration instead"""
        self.raises = raises
        self.endpoints: List[Tuple[str, Callable[[Any], None]]] = []

    def register_endpoint(self, path: str,
                          callback: Callable[[Any], None]) -> None:
        """
        :param path: the endpoint path
        :param callback: its handler
        """
        if self.raises is not None:
            raise self.raises
        self.endpoints.append((path, callback))


class U1KlipperObject:
    """
    A U1 firmware object (filament_detect, print_task_config) carrying only
    the attributes a test gives it. The reader discovers the firmware API with
    hasattr(), so an attribute that is not given must not exist.
    """

    def __init__(self, **attrs: Any) -> None:
        """:param attrs: the object's attributes (methods as Recorders)"""
        for name, value in attrs.items():
            setattr(self, name, value)


class U1Extruder:
    """An AFC_extruder as the reader resolves names against it."""

    def __init__(self, name: Optional[str] = None,
                 th_extruder_name: Optional[str] = None,
                 lanes: Optional[Dict[str, Any]] = None,
                 auto_spoolman_create: bool = False) -> None:
        """
        :param name: the AFC_extruder section name
        :param th_extruder_name: the toolhead extruder it drives
        :param lanes: the lanes feeding it, by name
        :param auto_spoolman_create: the extruder's Spoolman auto-create
        """
        self.name = name
        self.th_extruder_name = th_extruder_name
        self.lanes: Dict[str, Any] = dict(lanes or {})
        self.auto_spoolman_create = auto_spoolman_create


class U1Lane:
    """
    An AFC lane with the attributes the reader, AFC_RFID's lane helpers and
    AFC_spool's ``clear_values`` read and write.
    """

    def __init__(self, name: str, *, extruder_obj: Any = None,
                 unit_obj: Any = None, spool_scanner: bool = False,
                 status: str = "", spool_id: Any = None,
                 tool_loaded: bool = False, material: str = "",
                 color: str = "",
                 send_raises: Optional[BaseException] = None,
                 clear_raises: Optional[BaseException] = None) -> None:
        """
        :param name: the lane name
        :param extruder_obj: its AFC_extruder
        :param unit_obj: its unit
        :param spool_scanner: the lane doubles as a spool scanner
        :param status: its AFCLaneState value
        :param spool_id: its Spoolman spool
        :param tool_loaded: loaded into the toolhead
        :param material: its material
        :param color: its colour
        :param send_raises: raised by send_lane_data
        :param clear_raises: raised by clear_lane_data
        """
        self.name = name
        self.extruder_obj = extruder_obj
        self.unit_obj = unit_obj
        self.spool_scanner = spool_scanner
        self.status = status
        self.spool_id = spool_id
        self.tool_loaded = tool_loaded
        self.material = material
        self.color = color
        self.multi_color: List[str] = []
        self.weight = 0
        self.extruder_temp: Optional[float] = None
        self.bed_temp: Optional[float] = None
        self.spool_vendor = ""
        self.filament_name = ""
        self.sub_type = ""
        self.filament_density: Optional[float] = None
        self.remember_spool = False
        self.auto_switch_triggered = False
        self.send_lane_data = Recorder(raises=send_raises)
        self.clear_lane_data = Recorder(raises=clear_raises)


class U1Clock:
    """
    A ``time`` module for AFC_U1_rfid: a wall clock that moves only when the
    reader sleeps, running ``on_sleep`` after every sleep.
    """

    def __init__(self, now: float = U1_WALL_TIME,
                 on_sleep: Optional[Callable[[], None]] = None) -> None:
        """
        :param now: the starting wall-clock time
        :param on_sleep: run after each sleep (another process's turn)
        """
        self.now = now
        self.on_sleep = on_sleep
        self.sleeps: List[float] = []

    def time(self) -> float:
        """:return float: the wall-clock time"""
        return self.now

    def time_ns(self) -> int:
        """:return int: the whole seconds of the wall clock, in nanoseconds"""
        return int(self.now) * 1_000_000_000

    def sleep(self, seconds: float) -> None:
        """:param seconds: how far the clock moves"""
        self.sleeps.append(seconds)
        self.now += seconds
        if self.on_sleep is not None:
            self.on_sleep()


def u1_pass_through_spy(owner: Any, name: str) -> List[Tuple[tuple, dict]]:
    """
    Record each call to ``owner.<name>``, then run the real method.

    :param owner: the object whose method is wrapped
    :param name: the method's name
    :return list: the calls, as (args, kwargs), filled as they happen
    """
    real = getattr(owner, name)
    calls: List[Tuple[tuple, dict]] = []

    def spy(*args: Any, **kwargs: Any) -> Any:
        """Record the call and run the real method."""
        calls.append((args, kwargs))
        return real(*args, **kwargs)

    setattr(owner, name, spy)
    return calls


def make_u1_reader(values: Optional[Dict[str, Any]] = None, *,
                   printer: Optional[U1Printer] = None,
                   webhooks: Optional[U1Webhooks] = None) -> AFC_U1_RFID:
    """
    An AFC_U1_RFID built through its real ``__init__``.

    :param values: the [AFC_U1_rfid] options
    :param printer: the printer; a new U1Printer when None
    :param webhooks: registered as ``webhooks`` when the printer has none
    :return AFC_U1_RFID: the reader; ``reader.printer.afc`` is AFC's core
    """
    printer = printer if printer is not None else U1Printer()
    if "webhooks" not in printer._objects:
        printer.add_object("webhooks",
                           webhooks if webhooks is not None else U1Webhooks())
    return AFC_U1_RFID(U1Config("AFC_U1_rfid", printer, values))


class TestAFCU1RFIDInit:
    def test_defaults_empty_config(self):
        printer = U1Printer()
        webhooks = U1Webhooks()
        reader = make_u1_reader(printer=printer, webhooks=webhooks)
        assert reader.printer is printer
        assert reader.reactor is printer.reactor
        assert reader.logger is printer.afc.logger
        assert reader.afc is None
        assert reader._cfg_channels == {}
        assert reader._cfg_scanner_channels == set()
        assert reader._cfg_scanners == set()
        assert reader._scanner_auto_create is False
        assert reader._scanner_confirm_reads == 1
        assert reader._lane_auto_create is False
        assert reader._webhook_grace == 0.0
        assert reader._cfg_write_dir is None
        assert reader._tag_reads == {}
        assert reader._filament_detect is None
        assert reader._lane_channel_map == {}
        assert reader._lane_objects == {}
        assert reader._last_uid == {}
        assert reader._poll_timer is None
        assert reader._scanner_channels == set()
        assert reader._channel_to_lane == {}
        assert reader._consecutive_failures == {}
        assert reader._backed_off is False
        assert reader._backoff_cycles == 0
        assert reader._fd_cb_registered is False
        assert reader._pending_confirm == {}
        assert reader._webhook_channels_seen == set()
        assert reader._pending_defer == {}
        assert webhooks.endpoints == [("afc/u1_rfid", reader._handle_webhook_scan)]
        assert printer._event_handlers == {"klippy:ready": [reader._handle_ready]}
        assert reader.logger.messages == []

    def test_lane_channels_parsed_with_blank_skipped(self):
        reader = make_u1_reader({"lane_channels": " lane4:1, lane5 : 2, "})
        assert reader._cfg_channels == {"lane4": 1, "lane5": 2}
        assert reader.logger.messages == []

    def test_channels_alias_used_when_lane_channels_absent(self):
        reader = make_u1_reader({"channels": "lane6:3"})
        assert reader._cfg_channels == {"lane6": 3}
        assert reader.logger.messages == []

    def test_lane_channels_win_over_the_channels_alias(self):
        reader = make_u1_reader({"lane_channels": "lane4:1", "channels": "lane6:3"})
        assert reader._cfg_channels == {"lane4": 1}
        assert reader.logger.messages == []

    def test_lane_channels_missing_separator_raises(self):
        printer = U1Printer()
        webhooks = U1Webhooks()
        with pytest.raises(configparser.Error) as err:
            make_u1_reader({"lane_channels": "lane4"}, printer=printer,
                           webhooks=webhooks)
        assert str(err.value) == ("AFC_U1_rfid: 'lane_channels' entries must be "
                                  "'lane:channel', got 'lane4'")
        assert webhooks.endpoints == []
        assert printer._event_handlers == {}
        assert printer.afc.logger.messages == []

    def test_lane_channels_bad_number_raises(self):
        printer = U1Printer()
        with pytest.raises(configparser.Error) as err:
            make_u1_reader({"lane_channels": "lane4:x"}, printer=printer)
        assert str(err.value) == "AFC_U1_rfid: bad channel number in 'lane4:x'"
        assert printer.afc.logger.messages == []

    def test_scanner_channels_parsed_with_blank_skipped(self):
        reader = make_u1_reader({"scanner_channels": "0, 2, "})
        assert reader._cfg_scanner_channels == {0, 2}
        assert reader.logger.messages == []

    def test_scanner_channels_bad_value_raises(self):
        printer = U1Printer()
        with pytest.raises(configparser.Error) as err:
            make_u1_reader({"scanner_channels": "0, x"}, printer=printer)
        assert str(err.value) == "AFC_U1_rfid: bad scanner channel 'x'"
        assert printer.afc.logger.messages == []

    def test_scanner_lanes_and_flags(self):
        reader = make_u1_reader({
            "scanner_lanes": "lane1, lane2, ",
            "scanner_auto_create": True,
            "auto_spoolman_create": True,
            "scanner_confirm_reads": 3,
            "webhook_grace": 1.5,
            "openrfid_write_dir": "/srv/u1-write",
        })
        assert reader._cfg_scanners == {"lane1", "lane2"}
        assert reader._scanner_auto_create is True
        assert reader._lane_auto_create is True
        assert reader._scanner_confirm_reads == 3
        assert reader._webhook_grace == 1.5
        assert reader._cfg_write_dir == "/srv/u1-write"
        assert reader.logger.messages == []

    def test_webhook_endpoint_registered(self):
        webhooks = U1Webhooks()
        reader = make_u1_reader(webhooks=webhooks)
        assert webhooks.endpoints == [("afc/u1_rfid", reader._handle_webhook_scan)]
        assert reader.logger.messages == []

    def test_webhook_registration_failure_logs_warning(self):
        printer = U1Printer()
        webhooks = U1Webhooks(raises=ValueError("boom"))
        reader = make_u1_reader(printer=printer, webhooks=webhooks)
        assert webhooks.endpoints == []
        assert printer._event_handlers == {"klippy:ready": [reader._handle_ready]}
        assert reader.logger.messages == [
            ("warning", "AFC_U1_rfid: failed to register webhook endpoint: boom")]


class TestAFCU1RFIDHandleReady:
    class _ReadOnlyScannerLane(U1Lane):
        """A lane whose spool_scanner flag cannot be set once it is built."""

        def __init__(self, name: str) -> None:
            """:param name: the lane name"""
            self._frozen = False
            super().__init__(name)
            self._frozen = True

        @property
        def spool_scanner(self) -> bool:
            """:return bool: the flag it was built with"""
            return self._scanner

        @spool_scanner.setter
        def spool_scanner(self, value: bool) -> None:
            """:param value: accepted only while the lane is being built"""
            if self._frozen:
                raise AttributeError("spool_scanner is read-only")
            self._scanner = value

    @staticmethod
    def _timers(reader: AFC_U1_RFID) -> List[Tuple[Any, float]]:
        """:return list: each registered timer as (callback, waketime)"""
        return [(t.callback, t.waketime) for t in reader.reactor.timers]

    def test_afc_not_loaded_disables_reader(self):
        reader = make_u1_reader({"lane_channels": "lane1:1"})
        logger = reader.logger
        reader.printer._afc = None
        reader._handle_ready()
        assert reader.afc is None
        assert reader._lane_channel_map == {}
        assert reader._poll_timer is None
        assert reader.reactor.timers == []
        assert logger.messages == [
            ("warning", "AFC_U1_rfid: AFC not loaded; reader disabled")]

    def test_single_lane_registers_and_starts(self):
        reader = make_u1_reader({"lane_channels": "lane1:1"})
        afc = reader.printer.afc
        lane = U1Lane("lane1")
        afc.lanes = {"lane1": lane}
        reader._handle_ready()
        assert reader.afc is afc
        assert reader._lane_channel_map == {"lane1": 1}
        assert reader._lane_objects == {"lane1": lane}
        assert reader._channel_to_lane == {1: "lane1"}
        assert reader._last_uid == {1: None}
        assert reader._consecutive_failures == {1: 0}
        assert reader._cfg_scanner_channels == set()
        assert lane.spool_scanner is False
        assert self._timers(reader) == [(reader._poll_cb, 102.0)]
        assert reader._poll_timer is reader.reactor.timers[0]
        assert list(reader.printer._afc_rfid_write_registry) == ["u1:lane1"]
        assert reader.logger.messages == [
            ("info", "U1 RFID: monitoring 1 lane channel(s): lane1=ch1")]

    def test_combined_extruder_becomes_scanner_channel(self):
        reader = make_u1_reader({"lane_channels": "e0:4"})
        ext = U1Extruder(name="e0")
        lane_a = U1Lane("lane_a", extruder_obj=ext)
        lane_b = U1Lane("lane_b", extruder_obj=ext)
        ext.lanes = {"lane_a": lane_a, "lane_b": lane_b}
        reader.printer.afc.lanes = {"lane_a": lane_a, "lane_b": lane_b}
        reader._handle_ready()
        assert reader._cfg_scanner_channels == {4}
        assert reader._lane_channel_map == {}
        assert reader._channel_to_lane == {4: None}
        assert reader._last_uid == {4: None}
        assert reader._consecutive_failures == {4: 0}
        assert reader._scanner_channels == {4}
        assert self._timers(reader) == [(reader._poll_cb, 102.0)]
        assert reader.logger.messages == [
            ("info", "U1 RFID: 'e0' is a combined extruder (2 lanes), ch4 acts "
                     "as a spool scanner (stages next spool)"),
            ("info", "U1 RFID: standalone spool scanner channel(s): ch4"),
        ]

    def test_unresolved_lane_warns(self):
        reader = make_u1_reader({"lane_channels": "ghost:5"})
        reader._handle_ready()
        assert reader._lane_channel_map == {}
        assert reader._channel_to_lane == {}
        assert reader._cfg_scanner_channels == set()
        assert reader.reactor.timers == []
        assert reader.logger.messages == [
            ("warning", "U1 RFID: 'ghost' resolved to no lanes. Available "
                        "lanes=[]; extruders=[]. If 'ghost' is a standalone "
                        "toolhead, ensure its [AFC_stepper] has 'standalone: "
                        "True' and the [AFC_extruder ghost] section exists."),
            ("warning", "U1 RFID: configured lane 'ghost' not found in AFC "
                        "(neither a lane name nor a single-lane extruder)"),
        ]

    def test_scanner_lane_flag_set_on_lane(self):
        reader = make_u1_reader({"lane_channels": "lane1:1",
                                 "scanner_lanes": "lane1"})
        lane = U1Lane("lane1")
        reader.printer.afc.lanes = {"lane1": lane}
        reader._handle_ready()
        assert lane.spool_scanner is True
        assert reader._lane_channel_map == {"lane1": 1}
        assert reader._scanner_channels == {1}
        assert reader.logger.messages == [
            ("info", "U1 RFID: monitoring 1 lane channel(s): lane1=ch1"),
            ("info", "U1 RFID: lane-attached scanner(s): lane1"),
        ]

    def test_scanner_flag_set_error_is_swallowed(self):
        reader = make_u1_reader({"lane_channels": "lane1:1",
                                 "scanner_lanes": "lane1"})
        lane = self._ReadOnlyScannerLane("lane1")
        reader.printer.afc.lanes = {"lane1": lane}
        reader._handle_ready()
        assert lane.spool_scanner is False
        assert reader._lane_objects == {"lane1": lane}
        assert reader._lane_channel_map == {"lane1": 1}
        assert self._timers(reader) == [(reader._poll_cb, 102.0)]
        assert reader.logger.messages == [
            ("info", "U1 RFID: monitoring 1 lane channel(s): lane1=ch1"),
            ("info", "U1 RFID: lane-attached scanner(s): lane1"),
        ]

    def test_standalone_scanner_channels_registered(self):
        reader = make_u1_reader({"scanner_channels": "7"})
        original = Recorder()
        fd = U1KlipperObject(_notify_data_update_cb=[original])
        reader.printer.add_object("filament_detect", fd)
        reader.printer.add_object(
            "print_task_config",
            U1KlipperObject(_rfid_filament_info_update_cb=original))
        reader._handle_ready()
        assert reader._channel_to_lane == {7: None}
        assert reader._last_uid == {7: None}
        assert reader._consecutive_failures == {7: 0}
        assert reader._lane_channel_map == {}
        assert self._timers(reader) == [(reader._poll_cb, 102.0)]
        # start() attached the push callback, then the scanner patch replaced
        # the U1's own display callback.
        assert fd._notify_data_update_cb[0] is not original
        assert fd._notify_data_update_cb[1:] == [reader._on_filament_info_update]
        assert reader.logger.messages == [
            ("info", "U1 RFID: standalone spool scanner channel(s): ch7"),
            ("info", "U1 RFID: filament_detect attached (api: "
                     "_notify_data_update_cb)"),
            ("info", "U1 RFID: push callback registered via "
                     "_notify_data_update_cb"),
            ("info", "U1 RFID: protecting scanner channels [7] from U1 "
                     "display overwrite"),
        ]


class TestAFCU1RFIDPatchScannerRfidUpdate:
    @staticmethod
    def _reader(scanner_channels: str = "0", **objects: Any) -> AFC_U1_RFID:
        """
        :param scanner_channels: the scanner_channels option
        :param objects: printer objects to register, by name
        :return AFC_U1_RFID: the reader
        """
        reader = make_u1_reader({"scanner_channels": scanner_channels})
        for name, obj in objects.items():
            reader.printer.add_object(name, obj)
        return reader

    def test_noop_when_ptc_missing(self):
        original = Recorder()
        fd = U1KlipperObject(_notify_data_update_cb=[original])
        reader = self._reader(filament_detect=fd)
        reader._patch_scanner_rfid_update()
        assert fd._notify_data_update_cb == [original]
        assert reader.logger.messages == []

    def test_noop_when_fd_missing(self):
        original = Recorder()
        ptc = U1KlipperObject(_rfid_filament_info_update_cb=original)
        reader = self._reader(print_task_config=ptc)
        reader._patch_scanner_rfid_update()
        assert ptc._rfid_filament_info_update_cb is original
        assert reader.logger.messages == []

    def test_noop_when_fd_lacks_notify_attr(self):
        original = Recorder()
        fd = U1KlipperObject()
        reader = self._reader(
            print_task_config=U1KlipperObject(_rfid_filament_info_update_cb=original),
            filament_detect=fd)
        reader._patch_scanner_rfid_update()
        assert vars(fd) == {}
        assert reader.logger.messages == []

    def test_noop_when_original_cb_missing(self):
        other = Recorder()
        fd = U1KlipperObject(_notify_data_update_cb=[other])
        reader = self._reader(print_task_config=U1KlipperObject(),
                              filament_detect=fd)
        reader._patch_scanner_rfid_update()
        assert fd._notify_data_update_cb == [other]
        assert reader.logger.messages == []

    def test_noop_when_no_scanner_channels(self):
        original = Recorder()
        fd = U1KlipperObject(_notify_data_update_cb=[original])
        reader = self._reader(
            "",
            print_task_config=U1KlipperObject(_rfid_filament_info_update_cb=original),
            filament_detect=fd)
        reader._patch_scanner_rfid_update()
        assert fd._notify_data_update_cb == [original]
        assert reader.logger.messages == []

    def test_patches_callback_and_suppresses_scanner_channel(self):
        original = Recorder(result="sentinel")
        other = Recorder()
        fd = U1KlipperObject(_notify_data_update_cb=[other, original])
        reader = self._reader(
            "2, 0",
            print_task_config=U1KlipperObject(_rfid_filament_info_update_cb=original),
            filament_detect=fd)
        reader._patch_scanner_rfid_update()
        assert fd._notify_data_update_cb[0] is other
        patched = fd._notify_data_update_cb[1]
        assert patched is not original
        assert reader.logger.messages == [
            ("info", "U1 RFID: protecting scanner channels [0, 2] from U1 "
                     "display overwrite")]
        # The firmware in the field calls (channel, info, is_clear),
        # positionally; is_clear has a default, so two arguments are legal too.
        patched(0, {"x": 1}, False)
        patched(3, {"y": 2}, True)
        patched(3, {"y": 2})
        patched(2, {"x": 1})
        # The wrapper is a notify hook (-> None): it does not hand back the
        # original's return value.
        assert patched(5, {"z": 3}) is None
        assert original.calls == [((3, {"y": 2}, True), {}),
                                  ((3, {"y": 2}), {}),
                                  ((5, {"z": 3}), {})]
        assert other.calls == []

    def test_survives_a_firmware_that_calls_it_differently(self):
        """A Snapmaker update that adds an argument, or passes one by keyword,
        must not raise from our wrapper: an exception on filament_detect's
        notify path shuts Klipper down."""
        original = Recorder()
        fd = U1KlipperObject(_notify_data_update_cb=[original])
        reader = self._reader(
            print_task_config=U1KlipperObject(_rfid_filament_info_update_cb=original),
            filament_detect=fd)
        reader._patch_scanner_rfid_update()
        patched = fd._notify_data_update_cb[0]
        patched(3, {"y": 2}, True, "source")
        patched(3, {"y": 2}, is_clear=True, official=False)
        # The channel by keyword only: still a scanner channel, suppressed.
        patched(channel=0, info={"x": 1})
        # Nothing recognisable as a channel: passed on rather than guessed.
        patched("not-a-channel", {"z": 3})
        patched(info={"z": 4})
        assert original.calls == [
            ((3, {"y": 2}, True, "source"), {}),
            ((3, {"y": 2}), {"is_clear": True, "official": False}),
            (("not-a-channel", {"z": 3}), {}),
            ((), {"info": {"z": 4}}),
        ]
        assert reader.logger.messages == [
            ("info", "U1 RFID: protecting scanner channels [0] from U1 "
                     "display overwrite")]

    def test_warns_when_callback_not_found(self):
        original = Recorder()
        other = Recorder()
        fd = U1KlipperObject(_notify_data_update_cb=[other])
        reader = self._reader(
            print_task_config=U1KlipperObject(_rfid_filament_info_update_cb=original),
            filament_detect=fd)
        reader._patch_scanner_rfid_update()
        assert fd._notify_data_update_cb == [other]
        assert reader.logger.messages == [
            ("warning", "U1 RFID: could not locate print_task_config RFID "
                        "callback to patch")]


class TestAFCU1RFIDResolveLane:
    @staticmethod
    def _reader(lanes: Optional[Dict[str, Any]] = None,
                tools: Optional[Dict[str, Any]] = None) -> AFC_U1_RFID:
        """
        :param lanes: AFC's lane registry
        :param tools: AFC's extruder registry
        :return AFC_U1_RFID: a reader past klippy:ready's AFC lookup
        """
        reader = make_u1_reader()
        reader.afc = reader.printer.afc
        reader.afc.lanes = dict(lanes or {})
        reader.afc.tools = dict(tools or {})
        return reader

    def test_direct_lane_name(self):
        lane = U1Lane("lane1", extruder_obj=U1Extruder(name="lane1"))
        other = U1Lane("lane2", extruder_obj=U1Extruder(name="lane1"))
        reader = self._reader(lanes={"lane1": lane, "lane2": other})
        assert reader._resolve_lane("lane1") == (lane, None)
        assert reader.logger.messages == []

    def test_single_lane_by_extruder_name(self):
        lane = U1Lane("lane1", extruder_obj=U1Extruder(name="e0"))
        other = U1Lane("lane2", extruder_obj=U1Extruder(name="e1"))
        reader = self._reader(lanes={"lane1": lane, "lane2": other})
        assert reader._resolve_lane("e0") == (lane, None)
        assert reader.logger.messages == []

    def test_single_lane_by_th_extruder_name(self):
        lane = U1Lane("lane1", extruder_obj=U1Extruder(
            name="e0", th_extruder_name="extruder1"))
        reader = self._reader(lanes={"lane1": lane})
        assert reader._resolve_lane("extruder1") == (lane, None)
        assert reader.logger.messages == []

    def test_multiple_lanes_share_extruder_returns_extruder(self):
        ext = U1Extruder(name="e0")
        lane1 = U1Lane("lane1", extruder_obj=ext)
        lane2 = U1Lane("lane2", extruder_obj=ext)
        reader = self._reader(lanes={"lane1": lane1, "lane2": lane2})
        assert reader._resolve_lane("e0") == (None, ext)
        assert reader.logger.messages == []

    def test_tools_registry_single_lane(self):
        lane = U1Lane("lane1")
        ext = U1Extruder(name="e0", lanes={"lane1": lane})
        reader = self._reader(tools={"e0": ext})
        assert reader._resolve_lane("e0") == (lane, None)
        assert reader.logger.messages == []

    def test_tools_registry_multiple_lanes_returns_extruder(self):
        ext = U1Extruder(name="e0", lanes={"a": U1Lane("a"), "b": U1Lane("b")})
        reader = self._reader(tools={"e0": ext})
        assert reader._resolve_lane("e0") == (None, ext)
        assert reader.logger.messages == []

    def test_tools_registry_matched_by_extruder_attr(self):
        lane = U1Lane("lane1")
        nonmatch = U1Extruder(name="zzz", lanes={"lane9": U1Lane("lane9")})
        ext = U1Extruder(name="e0", lanes={"lane1": lane})
        # Registered under other keys, the non-matching one first.
        reader = self._reader(tools={"aaa": nonmatch, "bbb": ext})
        assert reader._resolve_lane("e0") == (lane, None)
        assert reader.logger.messages == []

    def test_nothing_matched_warns_and_returns_none_none(self):
        reader = self._reader()
        assert reader._resolve_lane("ghost") == (None, None)
        assert reader.logger.messages == [
            ("warning", "U1 RFID: 'ghost' resolved to no lanes. Available "
                        "lanes=[]; extruders=[]. If 'ghost' is a standalone "
                        "toolhead, ensure its [AFC_stepper] has 'standalone: "
                        "True' and the [AFC_extruder ghost] section exists.")]

    def test_nothing_matched_lists_the_registered_names(self):
        lane2 = U1Lane("lane2", extruder_obj=U1Extruder(
            name="e0", th_extruder_name="extruder"))
        lane1 = U1Lane("lane1")
        reader = self._reader(lanes={"lane2": lane2, "lane1": lane1},
                              tools={"e9": U1Extruder(name="e9")})
        assert reader._resolve_lane("ghost") == (None, None)
        assert reader.logger.messages == [
            ("warning", "U1 RFID: 'ghost' resolved to no lanes. Available "
                        "lanes=['lane1', 'lane2']; extruders=['e0', "
                        "'extruder']. If 'ghost' is a standalone toolhead, "
                        "ensure its [AFC_stepper] has 'standalone: True' and "
                        "the [AFC_extruder ghost] section exists.")]


class TestAFCU1RFIDRegisterLane:
    def test_registers_lane_and_channel(self):
        reader = make_u1_reader()
        lane1 = U1Lane("lane1")
        lane2 = U1Lane("lane2")
        reader.register_lane(lane1, 3)
        reader.register_lane(lane2, 0)
        assert reader._lane_channel_map == {"lane1": 3, "lane2": 0}
        assert reader._lane_objects == {"lane1": lane1, "lane2": lane2}
        assert reader._last_uid == {3: None, 0: None}
        assert reader._channel_to_lane == {3: "lane1", 0: "lane2"}
        assert reader._consecutive_failures == {3: 0, 0: 0}
        assert reader.logger.messages == []


class TestAFCU1RFIDStart:
    @staticmethod
    def _reader(values: Optional[Dict[str, Any]] = None) -> AFC_U1_RFID:
        """
        :param values: the [AFC_U1_rfid] options
        :return AFC_U1_RFID: a reader past klippy:ready's AFC lookup
        """
        reader = make_u1_reader(values)
        reader.afc = reader.printer.afc
        return reader

    def test_early_return_when_nothing_configured(self):
        reader = self._reader()
        reader.printer.add_object("filament_detect", U1KlipperObject())
        reader.start()
        assert reader._poll_timer is None
        assert reader.reactor.timers == []
        assert reader._filament_detect is None
        assert reader._scanner_channels == set()
        assert not hasattr(reader, "_gcode")
        assert not hasattr(reader.printer, "_afc_rfid_write_registry")
        assert reader.logger.messages == []

    def test_lane_and_scanner_channels_logged(self):
        reader = self._reader({"scanner_channels": "3, 0"})
        fd = U1KlipperObject()
        reader.printer.add_object("filament_detect", fd)
        reader.register_lane(U1Lane("lane1"), 1)
        reader.register_lane(U1Lane("lane2"), 2)
        reader.start()
        assert reader._gcode is reader.afc.gcode
        assert reader._scanner_channels == {0, 3}
        assert reader._filament_detect is fd
        assert [(t.callback, t.waketime) for t in reader.reactor.timers] == [
            (reader._poll_cb, 102.0)]
        assert reader._poll_timer is reader.reactor.timers[0]
        assert list(reader.printer._afc_rfid_write_registry) == [
            "u1:scanner0", "u1:scanner3", "u1:lane1", "u1:lane2"]
        assert reader.logger.messages == [
            ("info", "U1 RFID: monitoring 2 lane channel(s): lane1=ch1, "
                     "lane2=ch2"),
            ("info", "U1 RFID: standalone spool scanner channel(s): ch0, ch3"),
            ("info", "U1 RFID: filament_detect attached (api: none "
                     "recognized)"),
            ("warning", "U1 RFID: no recognized filament_detect push-callback "
                        "API; scanner will rely on polling only"),
        ]

    def test_only_scanner_channels_no_lane_log(self):
        reader = self._reader({"scanner_channels": "0"})
        reader.start()
        assert reader._scanner_channels == {0}
        assert [(t.callback, t.waketime) for t in reader.reactor.timers] == [
            (reader._poll_cb, 102.0)]
        assert reader._poll_timer is reader.reactor.timers[0]
        assert reader.logger.messages == [
            ("info", "U1 RFID: standalone spool scanner channel(s): ch0")]

    def test_lane_attached_scanner_logged(self):
        reader = self._reader({"scanner_lanes": "lane1"})
        reader.register_lane(U1Lane("lane1"), 1)
        reader.register_lane(U1Lane("lane2"), 2)
        reader.start()
        assert reader._scanner_channels == {1}
        assert [(t.callback, t.waketime) for t in reader.reactor.timers] == [
            (reader._poll_cb, 102.0)]
        assert reader.logger.messages == [
            ("info", "U1 RFID: monitoring 2 lane channel(s): lane1=ch1, "
                     "lane2=ch2"),
            ("info", "U1 RFID: lane-attached scanner(s): lane1"),
        ]


class TestAFCU1RFIDRegisterWriters:
    def test_scanner_and_lane_channels_each_get_a_target(self):
        reader = make_u1_reader({"scanner_channels": "0"})
        reader.register_lane(U1Lane("lane5"), 2)
        reader.register_lane(U1Lane("lane4"), 1)
        reader._openrfid_write = Recorder(result=("04a1", None))
        reader._register_writers()
        registry = reader.printer._afc_rfid_write_registry
        assert {name: (t.label, t.unit, t.threaded)
                for name, t in registry.items()} == {
            "u1:scanner0": ("U1 OpenRFID spool scanner (channel 0)", reader, True),
            "u1:lane4": ("U1 OpenRFID lane lane4 (channel 1)", reader, True),
            "u1:lane5": ("U1 OpenRFID lane lane5 (channel 2)", reader, True),
        }
        assert list(registry) == ["u1:scanner0", "u1:lane4", "u1:lane5"]
        assert [t.open_link() for t in registry.values()] == [True, True, True]
        # Each hand-off writes on its own channel.
        assert registry["u1:lane5"].write_payload(b"\x01") == ("04a1", None)
        assert registry["u1:scanner0"].write_payload(b"\x02") == ("04a1", None)
        assert registry["u1:lane4"].write_payload(b"\x03") == ("04a1", None)
        assert reader._openrfid_write.calls == [((2, b"\x01"), {}),
                                                ((0, b"\x02"), {}),
                                                ((1, b"\x03"), {})]
        assert reader.logger.messages == []

    def test_a_channel_used_by_both_is_registered_once(self):
        reader = make_u1_reader({"scanner_channels": "1"})
        reader.register_lane(U1Lane("lane4"), 1)
        reader._register_writers()
        registry = reader.printer._afc_rfid_write_registry
        # Channel 1 is seen first as the scanner; the lane does not double it.
        assert list(registry) == ["u1:scanner1"]
        assert registry["u1:scanner1"].label == "U1 OpenRFID spool scanner (channel 1)"
        assert reader.logger.messages == []


class TestAFCU1RFIDOpenrfidWrite:
    class _Daemon:
        """
        The OpenRFID write-watch controller, polled each time the reader
        sleeps: it takes (removes) a request, then writes one queued result
        file per poll.
        """

        def __init__(self, directory: str, results: List[str]) -> None:
            """
            :param directory: the write-request directory it watches
            :param results: result-file contents, one written per poll
            """
            self.directory = directory
            self.results = list(results)
            self.taken: List[str] = []
            self.requests: List[Dict[str, Any]] = []
            self._token: Optional[str] = None

        def poll(self) -> None:
            """Take any request, then write the next result, if any."""
            for name in sorted(os.listdir(self.directory)):
                if name.startswith("req-") and name.endswith(".json"):
                    path = os.path.join(self.directory, name)
                    with open(path) as f:
                        self.requests.append(json.load(f))
                    os.remove(path)
                    self.taken.append(name)
                    self._token = name[len("req-"):-len(".json")]
            if self._token is not None and self.results:
                path = os.path.join(self.directory, f"res-{self._token}.json")
                with open(path, "w") as f:
                    f.write(self.results.pop(0))

    @staticmethod
    def _token() -> str:
        """:return str: the request token on the pinned clock"""
        return f"{os.getpid()}-1700000000000000000"

    @staticmethod
    def _reader(monkeypatch: pytest.MonkeyPatch, write_dir: Optional[str],
                on_sleep: Optional[Callable[[], None]] = None
                ) -> Tuple[AFC_U1_RFID, U1Clock]:
        """
        :param write_dir: the openrfid_write_dir option, None for the default
        :param on_sleep: run each time the reader sleeps
        :return tuple: the reader and its clock
        """
        values = {} if write_dir is None else {"openrfid_write_dir": write_dir}
        reader = make_u1_reader(values)
        clock = U1Clock(on_sleep=on_sleep)
        monkeypatch.setattr(u1_mod, "time", clock)
        return reader, clock

    def test_a_successful_write_returns_the_uid(self, tmp_path, monkeypatch):
        write_dir = str(tmp_path / ".afc_u1_write")
        daemon = self._Daemon(write_dir, ['{"ok": true, "uid": "04a1b2c3d4e5f6"}'])
        reader, clock = self._reader(monkeypatch, write_dir, daemon.poll)
        assert reader._openrfid_write(0, bytes(4)) == ("04a1b2c3d4e5f6", None)
        assert clock.sleeps == [0.15]
        assert daemon.requests == [{"slot": 0, "start_page": 4, "data": "00000000"}]
        assert os.listdir(write_dir) == [f"res-{self._token()}.json"]
        assert reader.logger.messages == []

    def test_the_request_carries_slot_page_and_data(self, tmp_path, monkeypatch):
        write_dir = str(tmp_path / ".afc_u1_write")
        daemon = self._Daemon(write_dir, ['{"ok": true, "uid": "04"}'])
        reader, _clock = self._reader(monkeypatch, write_dir, daemon.poll)
        reader._openrfid_write(2, b"\x04\xa1\xff\x00")
        assert daemon.taken == [f"req-{self._token()}.json"]
        assert daemon.requests == [{"slot": 2, "start_page": 4, "data": "04a1ff00"}]
        assert reader.logger.messages == []

    def test_a_daemon_error_is_surfaced(self, tmp_path, monkeypatch):
        write_dir = str(tmp_path / ".afc_u1_write")
        daemon = self._Daemon(
            write_dir, ['{"ok": false, "error": "no tag in the reader\'s field"}'])
        reader, _clock = self._reader(monkeypatch, write_dir, daemon.poll)
        assert reader._openrfid_write(0, bytes(4)) == (
            None, "no tag in the reader's field")
        assert reader.logger.messages == []

    def test_a_failure_without_a_reason_says_the_write_failed(self, tmp_path,
                                                              monkeypatch):
        write_dir = str(tmp_path / ".afc_u1_write")
        daemon = self._Daemon(write_dir, ['{"ok": false, "uid": "04"}'])
        reader, _clock = self._reader(monkeypatch, write_dir, daemon.poll)
        assert reader._openrfid_write(0, bytes(4)) == ("04", "the write failed")
        assert reader.logger.messages == []

    def test_a_half_written_result_is_not_read_early(self, tmp_path, monkeypatch):
        """A result caught mid-write does not parse: wait and read it again."""
        write_dir = str(tmp_path / ".afc_u1_write")
        daemon = self._Daemon(write_dir, ['{"ok": tr', '{"ok": true, "uid": "04"}'])
        reader, clock = self._reader(monkeypatch, write_dir, daemon.poll)
        assert reader._openrfid_write(0, bytes(4)) == ("04", None)
        assert clock.sleeps == [0.15, 0.15]
        assert reader.logger.messages == []

    def test_no_daemon_times_out_with_a_helpful_message(self, tmp_path,
                                                        monkeypatch):
        write_dir = str(tmp_path / ".afc_u1_write")
        reader, clock = self._reader(monkeypatch, write_dir)
        assert reader._openrfid_write(0, bytes(4)) == (
            None, "no answer from OpenRFID within 45s. Is the write-watch "
                  "controller installed and the daemon running? "
                  "(contrib/openrfid-ntag-write)")
        assert set(clock.sleeps) == {0.15}
        assert clock.now >= U1_WALL_TIME + 45.0
        assert clock.now - clock.sleeps[-1] < U1_WALL_TIME + 45.0
        assert reader.logger.messages == []

    def test_a_timed_out_request_is_cleaned_up(self, tmp_path, monkeypatch):
        write_dir = str(tmp_path / ".afc_u1_write")
        reader, _clock = self._reader(monkeypatch, write_dir)
        reader._openrfid_write(0, bytes(4))
        assert os.listdir(write_dir) == []
        assert reader.logger.messages == []

    def test_a_request_taken_but_never_answered_times_out(self, tmp_path,
                                                          monkeypatch):
        """The daemon removes a request before writing, so the timeout's own
        clean-up finds nothing left to remove."""
        write_dir = str(tmp_path / ".afc_u1_write")
        daemon = self._Daemon(write_dir, [])
        reader, _clock = self._reader(monkeypatch, write_dir, daemon.poll)
        assert reader._openrfid_write(1, b"\x01") == (
            None, "no answer from OpenRFID within 45s. Is the write-watch "
                  "controller installed and the daemon running? "
                  "(contrib/openrfid-ntag-write)")
        assert daemon.requests == [{"slot": 1, "start_page": 4, "data": "01"}]
        assert os.listdir(write_dir) == []
        assert reader.logger.messages == []

    def test_an_unreachable_write_directory_is_reported(self, tmp_path,
                                                        monkeypatch):
        (tmp_path / "printer.cfg").write_text("")
        write_dir = str(tmp_path / "printer.cfg" / "u1")
        reader, clock = self._reader(monkeypatch, write_dir)
        assert reader._openrfid_write(0, bytes(4)) == (
            None, f"cannot reach the write directory {write_dir}: [Errno 20] "
                  f"Not a directory: '{write_dir}'")
        assert clock.sleeps == []
        assert reader.logger.messages == []

    def test_a_request_that_cannot_be_queued_is_reported(self, tmp_path,
                                                         monkeypatch):
        write_dir = tmp_path / ".afc_u1_write"
        req = write_dir / f"req-{self._token()}.json"
        (req / "blocker").mkdir(parents=True)
        reader, clock = self._reader(monkeypatch, str(write_dir))
        assert reader._openrfid_write(0, bytes(4)) == (
            None, f"could not queue the write request: [Errno 21] Is a "
                  f"directory: '{req}.tmp' -> '{req}'")
        assert clock.sleeps == []
        assert reader.logger.messages == []

    def test_the_default_directory_sits_beside_the_config(self, tmp_path,
                                                          monkeypatch):
        write_dir = str(tmp_path / ".afc_u1_write")
        daemon = self._Daemon(write_dir, ['{"ok": true, "uid": "04"}'])
        reader, _clock = self._reader(monkeypatch, None, daemon.poll)
        reader.printer.start_args = {"config_file": str(tmp_path / "printer.cfg")}
        assert reader._openrfid_write(3, b"\xff") == ("04", None)
        assert daemon.requests == [{"slot": 3, "start_page": 4, "data": "ff"}]
        assert reader.logger.messages == []

    def test_without_a_config_path_the_directory_is_relative(self, tmp_path,
                                                            monkeypatch):
        monkeypatch.chdir(tmp_path)
        daemon = self._Daemon(".afc_u1_write", ['{"ok": true, "uid": "04"}'])
        reader, _clock = self._reader(monkeypatch, None, daemon.poll)
        # No start args at all: reading a config path from them would raise.
        reader.printer.start_args = None
        assert reader._openrfid_write(3, b"\xff") == ("04", None)
        assert daemon.requests == [{"slot": 3, "start_page": 4, "data": "ff"}]
        assert os.listdir(tmp_path / ".afc_u1_write") == [
            f"res-{self._token()}.json"]
        assert os.listdir(tmp_path) == [".afc_u1_write"]
        assert reader.logger.messages == []


class TestAFCU1RFIDTryAttachFilamentDetect:
    def test_already_attached_returns_true(self):
        reader = make_u1_reader()
        attached = U1KlipperObject(_notify_data_update_cb=[])
        reader._filament_detect = attached
        newer = U1KlipperObject(_notify_data_update_cb=[])
        reader.printer.add_object("filament_detect", newer)
        assert reader._try_attach_filament_detect() is True
        assert reader._filament_detect is attached
        assert reader._fd_cb_registered is False
        assert attached._notify_data_update_cb == []
        assert newer._notify_data_update_cb == []
        assert reader.logger.messages == []

    def test_fd_missing_returns_false(self):
        reader = make_u1_reader()
        assert reader._try_attach_filament_detect() is False
        assert reader._filament_detect is None
        assert reader.logger.messages == []

    def test_attaches_and_logs_recognized_api(self):
        reader = make_u1_reader()
        register = Recorder()
        fd = U1KlipperObject(get_status=Recorder(),
                             register_cb_2_update_filament_info=register,
                             get_a_filament_info=Recorder())
        reader.printer.add_object("filament_detect", fd)
        assert reader._try_attach_filament_detect() is True
        assert reader._filament_detect is fd
        assert register.calls == [((reader._on_filament_info_update,), {})]
        assert reader._fd_cb_registered is True
        assert reader.logger.messages == [
            ("info", "U1 RFID: filament_detect attached (api: "
                     "register_cb_2_update_filament_info, get_a_filament_info, "
                     "get_status)"),
            ("info", "U1 RFID: push callback registered via "
                     "register_cb_2_update_filament_info"),
        ]

    def test_attaches_and_logs_none_recognized(self):
        reader = make_u1_reader()
        fd = U1KlipperObject()
        reader.printer.add_object("filament_detect", fd)
        assert reader._try_attach_filament_detect() is True
        assert reader._filament_detect is fd
        assert reader._fd_cb_registered is False
        assert reader.logger.messages == [
            ("info", "U1 RFID: filament_detect attached (api: none recognized)"),
            ("warning", "U1 RFID: no recognized filament_detect push-callback "
                        "API; scanner will rely on polling only"),
        ]


class TestAFCU1RFIDRegisterFdCallback:
    def test_returns_early_when_already_registered(self):
        reader = make_u1_reader()
        reader._fd_cb_registered = True
        register = Recorder()
        cb_list: List[Any] = []
        reader._register_fd_callback(U1KlipperObject(
            register_cb_2_update_filament_info=register,
            _notify_data_update_cb=cb_list))
        assert register.calls == []
        assert cb_list == []
        assert reader.logger.messages == []

    def test_registers_via_proven_api(self):
        reader = make_u1_reader()
        register = Recorder()
        cb_list: List[Any] = []
        reader._register_fd_callback(U1KlipperObject(
            register_cb_2_update_filament_info=register,
            _notify_data_update_cb=cb_list))
        assert register.calls == [((reader._on_filament_info_update,), {})]
        assert cb_list == []
        assert reader._fd_cb_registered is True
        assert reader.logger.messages == [
            ("info", "U1 RFID: push callback registered via "
                     "register_cb_2_update_filament_info")]

    def test_falls_back_to_cb_list_on_register_error(self):
        reader = make_u1_reader()
        cb_list: List[Any] = []
        reader._register_fd_callback(U1KlipperObject(
            register_cb_2_update_filament_info=Recorder(
                raises=RuntimeError("nope")),
            _notify_data_update_cb=cb_list))
        assert cb_list == [reader._on_filament_info_update]
        assert reader._fd_cb_registered is True
        assert reader.logger.messages == [
            ("warning", "U1 RFID: failed to register info callback: nope"),
            ("info", "U1 RFID: push callback registered via "
                     "_notify_data_update_cb"),
        ]

    def test_appends_to_cb_list_when_no_register_api(self):
        reader = make_u1_reader()
        native = Recorder()
        fd = U1KlipperObject(_notify_data_update_cb=[native])
        reader._register_fd_callback(fd)
        assert fd._notify_data_update_cb == [native, reader._on_filament_info_update]
        assert reader._fd_cb_registered is True
        assert reader.logger.messages == [
            ("info", "U1 RFID: push callback registered via "
                     "_notify_data_update_cb")]

    def test_does_not_double_append_when_already_in_cb_list(self):
        reader = make_u1_reader()
        fd = U1KlipperObject(_notify_data_update_cb=[reader._on_filament_info_update])
        reader._register_fd_callback(fd)
        assert fd._notify_data_update_cb == [reader._on_filament_info_update]
        assert reader._fd_cb_registered is True
        assert reader.logger.messages == [
            ("info", "U1 RFID: push callback registered via "
                     "_notify_data_update_cb")]

    @pytest.mark.parametrize("attrs", [{}, {"_notify_data_update_cb": ()}],
                             ids=["no_cb_list", "cb_list_not_a_list"])
    def test_warns_when_no_push_api(self, attrs):
        reader = make_u1_reader()
        # No callback list, or one that is not a list to append to.
        reader._register_fd_callback(U1KlipperObject(**attrs))
        assert reader._fd_cb_registered is False
        assert reader.logger.messages == [
            ("warning", "U1 RFID: no recognized filament_detect push-callback "
                        "API; scanner will rely on polling only")]


class TestAFCU1RFIDOnFilamentInfoUpdate:
    @staticmethod
    def _reader(check: Optional[Recorder] = None) -> AFC_U1_RFID:
        """
        :param check: stands in for _check_channel, recording what it is given
        :return AFC_U1_RFID: the reader, lane1 on ch1 and a scanner on ch0
        """
        reader = make_u1_reader()
        reader.register_lane(U1Lane("lane1"), 1)
        reader._channel_to_lane[0] = None
        reader._check_channel = check if check is not None else Recorder()
        return reader

    def test_dispatches_registered_channel(self):
        reader = self._reader()
        reader._on_filament_info_update(1, {"CARD_UID": U1_UID}, False)
        reader._on_filament_info_update(0, {"CARD_UID": U1_UID_OTHER})
        assert reader._check_channel.calls == [
            (("lane1", 1), {"info": {"CARD_UID": U1_UID}}),
            ((None, 0), {"info": {"CARD_UID": U1_UID_OTHER}}),
        ]
        assert reader.logger.messages == []

    def test_unregistered_channel_is_ignored(self):
        reader = self._reader()
        reader._on_filament_info_update(5, {"CARD_UID": U1_UID})
        assert reader._check_channel.calls == []
        assert reader.logger.messages == []

    def test_dispatch_error_logs_warning(self):
        reader = self._reader(check=Recorder(raises=RuntimeError("bad")))
        reader._on_filament_info_update(1, {"CARD_UID": U1_UID})
        assert reader._check_channel.calls == [
            (("lane1", 1), {"info": {"CARD_UID": U1_UID}})]
        assert reader.logger.messages == [
            ("warning", "U1 RFID: _on_filament_info_update error ch1: bad")]

    def test_short_args_fall_back_to_lane_loop(self):
        reader = self._reader()
        reader.register_lane(U1Lane("lane2"), 2)
        reader._on_filament_info_update(1)
        assert reader._check_channel.calls == [(("lane1", 1), {}),
                                               (("lane2", 2), {})]
        assert reader.logger.messages == []

    def test_non_int_first_arg_falls_back_to_lane_loop(self):
        reader = self._reader()
        reader._on_filament_info_update("1", {"CARD_UID": U1_UID})
        assert reader._check_channel.calls == [(("lane1", 1), {})]
        assert reader.logger.messages == []

    def test_non_dict_second_arg_falls_back_to_lane_loop(self):
        reader = self._reader()
        reader._on_filament_info_update(1, "x")
        assert reader._check_channel.calls == [(("lane1", 1), {})]
        assert reader.logger.messages == []

    def test_lane_loop_error_logs_warning(self):
        reader = self._reader(check=Recorder(raises=RuntimeError("oops")))
        reader.register_lane(U1Lane("lane2"), 2)
        reader._on_filament_info_update()
        # One failing lane does not stop the next.
        assert reader._check_channel.calls == [(("lane1", 1), {}),
                                               (("lane2", 2), {})]
        assert reader.logger.messages == [
            ("warning", "U1 RFID: _on_filament_info_update error lane1: oops"),
            ("warning", "U1 RFID: _on_filament_info_update error lane2: oops"),
        ]


class TestAFCU1RFIDStop:
    def test_noop_when_no_timer(self):
        reader = make_u1_reader()
        reader.reactor.update_timer = Recorder()
        reader.stop()
        assert reader.reactor.update_timer.calls == []
        assert reader._poll_timer is None
        assert reader.logger.messages == []

    def test_updates_timer_to_never(self):
        reader = make_u1_reader({"scanner_channels": "0"})
        reader.afc = reader.printer.afc
        reader.start()
        timer = reader._poll_timer
        assert timer.waketime == 102.0
        reader.stop()
        assert reader._poll_timer is timer
        assert timer.waketime == 9_999_999_999.0
        assert reader.logger.messages == [
            ("info", "U1 RFID: standalone spool scanner channel(s): ch0")]


class TestAFCU1RFIDTriggerChannelUpdate:
    class _BusyGcode:
        """A gcode whose FILAMENT_DT_UPDATE fails: its M400 refuses mid-move."""

        def __init__(self) -> None:
            """Start with no scripts."""
            self.scripts: List[str] = []

        def run_script_from_command(self, script: str) -> None:
            """:param script: recorded, then refused"""
            self.scripts.append(script)
            raise RuntimeError("m400")

    @staticmethod
    def _reader(fd: Any, busy: bool = False) -> AFC_U1_RFID:
        """
        :param fd: the attached filament_detect, or None
        :param busy: FILAMENT_DT_UPDATE fails
        :return AFC_U1_RFID: the reader as start() leaves it
        """
        reader = make_u1_reader()
        reader._filament_detect = fd
        reader._gcode = (TestAFCU1RFIDTriggerChannelUpdate._BusyGcode() if busy
                         else reader.printer.gcode)
        return reader

    def test_returns_false_when_no_fd(self):
        reader = self._reader(None)
        assert reader._trigger_channel_update(0) is False
        assert reader._gcode.scripts == []
        assert reader.logger.messages == []

    def test_success_via_filament_dt_update(self):
        update = Recorder()
        reader = self._reader(U1KlipperObject(update_filament_info=update))
        assert reader._trigger_channel_update(2) is True
        assert reader._gcode.scripts == ["FILAMENT_DT_UPDATE CHANNEL=2"]
        assert update.calls == []
        assert reader.logger.messages == []

    def test_fallback_to_update_filament_info(self):
        update = Recorder()
        request = Recorder()
        reader = self._reader(U1KlipperObject(update_filament_info=update,
                                              request_update=request),
                              busy=True)
        assert reader._trigger_channel_update(3) is True
        assert reader._gcode.scripts == ["FILAMENT_DT_UPDATE CHANNEL=3"]
        assert update.calls == [((3,), {})]
        assert request.calls == []
        assert reader.logger.messages == [
            ("warning", "U1 RFID: FILAMENT_DT_UPDATE failed ch3: m400")]

    def test_fallback_to_request_update(self):
        request = Recorder()
        reader = self._reader(U1KlipperObject(request_update=request), busy=True)
        assert reader._trigger_channel_update(4) is True
        assert request.calls == [((4,), {})]
        assert reader.logger.messages == [
            ("warning", "U1 RFID: FILAMENT_DT_UPDATE failed ch4: m400")]

    def test_all_paths_fail_returns_false(self):
        update = Recorder(raises=RuntimeError("x"))
        request = Recorder(raises=RuntimeError("y"))
        reader = self._reader(U1KlipperObject(update_filament_info=update,
                                              request_update=request),
                              busy=True)
        assert reader._trigger_channel_update(5) is False
        assert update.calls == [((5,), {})]
        assert request.calls == [((5,), {})]
        assert reader.logger.messages == [
            ("warning", "U1 RFID: FILAMENT_DT_UPDATE failed ch5: m400")]

    def test_no_fallback_api_returns_false(self):
        reader = self._reader(U1KlipperObject(), busy=True)
        assert reader._trigger_channel_update(6) is False
        assert reader._gcode.scripts == ["FILAMENT_DT_UPDATE CHANNEL=6"]
        assert reader.logger.messages == [
            ("warning", "U1 RFID: FILAMENT_DT_UPDATE failed ch6: m400")]


class TestAFCU1RFIDPollCb:
    @staticmethod
    def _reader(idle_state: Optional[str] = None, attached: bool = True,
                trigger_ok: bool = True) -> AFC_U1_RFID:
        """
        :param idle_state: idle_timeout's state; None leaves it unregistered
        :param attached: filament_detect is attached
        :param trigger_ok: what _trigger_channel_update reports
        :return AFC_U1_RFID: the reader, _trigger_channel_update and
          _check_channel recording what they are given
        """
        reader = make_u1_reader()
        if idle_state is not None:
            reader.printer.add_object("idle_timeout",
                                      FakeIdleTimeout(state=idle_state))
        if attached:
            reader._filament_detect = U1KlipperObject()
        reader._trigger_channel_update = Recorder(result=trigger_ok)
        reader._check_channel = Recorder()
        return reader

    def test_backoff_when_fd_unavailable(self):
        reader = self._reader(attached=False)
        reader._scanner_channels = {0}
        reader._cfg_scanner_channels = {0}
        assert reader._poll_cb(100.0) == 110.0
        assert reader._trigger_channel_update.calls == []
        assert reader._check_channel.calls == []
        assert reader.logger.messages == []

    def test_deferred_while_printing(self):
        reader = self._reader(idle_state="Printing")
        reader._scanner_channels = {0}
        reader.register_lane(U1Lane("lane1"), 1)
        assert reader._poll_cb(50.0) == 52.0
        assert reader._trigger_channel_update.calls == []
        assert reader._check_channel.calls == []
        assert reader.logger.messages == []

    def test_idle_none_proceeds(self):
        reader = self._reader()
        reader.register_lane(U1Lane("lane1"), 1)
        assert reader._poll_cb(10.0) == 12.0
        assert reader._check_channel.calls == [(("lane1", 1), {})]
        assert reader.logger.messages == []

    def test_idle_not_printing_proceeds(self):
        reader = self._reader(idle_state="Idle")
        reader._cfg_scanner_channels = {0}
        reader._scanner_channels = {0}
        reader.register_lane(U1Lane("lane1"), 1)
        assert reader._poll_cb(0.0) == 2.0
        assert reader._trigger_channel_update.calls == [((0,), {})]
        assert reader._check_channel.calls == [((None, 0), {}),
                                               (("lane1", 1), {})]
        assert reader.logger.messages == []

    def test_trigger_failure_reaches_backoff_threshold(self):
        reader = self._reader(trigger_ok=False)
        reader._scanner_channels = {0}
        reader._consecutive_failures = {0: 4}
        assert reader._poll_cb(0.0) == 10.0
        assert reader._consecutive_failures == {0: 5}
        assert reader._backed_off is True
        assert reader._backoff_cycles == 1
        assert reader.logger.messages == [
            ("error", "U1 RFID: ch0 failed 5 times consecutively, backing off")]

    def test_trigger_failure_below_threshold_no_backoff(self):
        reader = self._reader(trigger_ok=False)
        reader._scanner_channels = {0}
        assert reader._poll_cb(0.0) == 2.0
        assert reader._consecutive_failures == {0: 1}
        assert reader._backed_off is False
        assert reader._backoff_cycles == 0
        assert reader.logger.messages == []

    def test_trigger_success_resets_failures(self):
        reader = self._reader()
        reader._scanner_channels = {0}
        reader._consecutive_failures = {0: 3}
        assert reader._poll_cb(0.0) == 2.0
        assert reader._trigger_channel_update.calls == [((0,), {})]
        assert reader._consecutive_failures == {0: 0}
        assert reader.logger.messages == []

    def test_scanner_poll_error_logs_warning(self):
        reader = self._reader()
        reader._cfg_scanner_channels = {0}
        reader.register_lane(U1Lane("lane1"), 1)
        reader._check_channel = Recorder(raises=RuntimeError("boom"))
        assert reader._poll_cb(0.0) == 2.0
        # The lane loop still runs after the scanner's failure.
        assert reader._check_channel.calls == [((None, 0), {}),
                                               (("lane1", 1), {})]
        assert reader.logger.messages == [
            ("warning", "U1 RFID: poll error on scanner ch0: boom"),
            ("warning", "U1 RFID: poll error on lane1 ch1: boom"),
        ]

    def test_lane_poll_error_logs_warning(self):
        reader = self._reader()
        reader.register_lane(U1Lane("lane1"), 2)
        reader._check_channel = Recorder(raises=RuntimeError("bad"))
        assert reader._poll_cb(0.0) == 2.0
        assert reader.logger.messages == [
            ("warning", "U1 RFID: poll error on lane1 ch2: bad")]

    def test_backoff_reset_after_reset_cycles(self):
        reader = self._reader(trigger_ok=False)
        reader._scanner_channels = {0}
        reader._consecutive_failures = {0: 5}
        reader._backed_off = True
        reader._backoff_cycles = 17
        assert reader._poll_cb(0.0) == 10.0
        assert reader._backed_off is False
        assert reader._backoff_cycles == 0
        assert reader._consecutive_failures == {0: 0}
        assert reader.logger.messages == [
            ("info", "U1 RFID: backoff reset, retrying normal polling")]

    def test_backoff_clears_when_all_recovered(self):
        reader = self._reader()
        reader._scanner_channels = {0, 1}
        reader._consecutive_failures = {0: 5, 1: 0}
        reader._backed_off = True
        reader._backoff_cycles = 2
        # This poll's successful reads reset both channels, so all recovered.
        assert reader._poll_cb(0.0) == 10.0
        assert reader._consecutive_failures == {0: 0, 1: 0}
        assert reader._backed_off is False
        assert reader._backoff_cycles == 0
        assert reader.logger.messages == []

    def test_backoff_persists_when_not_recovered(self):
        reader = self._reader(trigger_ok=False)
        reader._scanner_channels = {0}
        reader._consecutive_failures = {0: 5}
        reader._backed_off = True
        reader._backoff_cycles = 2
        # A sixth failure stays backed off without logging the threshold again.
        assert reader._poll_cb(0.0) == 10.0
        assert reader._consecutive_failures == {0: 6}
        assert reader._backed_off is True
        assert reader._backoff_cycles == 3
        assert reader.logger.messages == []


class TestAFCU1RFIDSendLaneData:
    @staticmethod
    def _reader() -> AFC_U1_RFID:
        """:return AFC_U1_RFID: a reader past klippy:ready's AFC lookup"""
        reader = make_u1_reader()
        reader.afc = reader.printer.afc
        return reader

    def test_skips_when_moonraker_down(self):
        reader = self._reader()
        reader.afc.moonraker = None
        lane = U1Lane("lane1")
        reader._send_lane_data(lane)
        assert lane.send_lane_data.calls == []
        assert reader.logger.messages == []

    def test_pushes_lane_data(self):
        reader = self._reader()
        lane = U1Lane("lane1")
        reader._send_lane_data(lane)
        assert lane.send_lane_data.calls == [((), {})]
        assert reader.logger.messages == []

    def test_push_failure_logs_debug(self):
        reader = self._reader()
        lane = U1Lane("lane9", send_raises=RuntimeError("nope"))
        reader._send_lane_data(lane)
        assert lane.send_lane_data.calls == [((), {})]
        assert reader.logger.messages == [
            ("debug", "U1 RFID: send_lane_data skipped for lane9: nope")]


class TestAFCU1RFIDHandleWebhookScan:
    class _WebRequest:
        """Klipper's WebRequest over a JSON body: a missing argument without a
        default, or an argument of the wrong type, raises."""

        _MISSING = object()

        def __init__(self, body: Dict[str, Any]) -> None:
            """:param body: the request's JSON body"""
            self.body = dict(body)

        def get(self, item: str, default: Any = _MISSING,
                types: Optional[Tuple[type, ...]] = None) -> Any:
            """
            :param item: the argument
            :param default: returned when it is absent
            :param types: the types it must have when present
            :return Any: the value
            """
            value = self.body.get(item, default)
            if value is self._MISSING:
                error_str = f"Missing Argument [{item}]"
                raise ValueError(error_str)
            if (types is not None
                and type(value) not in types
                and item in self.body):
                error_str = f"Invalid Argument Type [{item}]"
                raise ValueError(error_str)
            return value

        def get_int(self, item: str, default: Any = _MISSING) -> Any:
            """
            :param item: the argument
            :param default: returned when it is absent
            :return Any: the int value
            """
            return self.get(item, default, types=(int,))

    @staticmethod
    def _reader(check: Optional[Recorder] = None) -> AFC_U1_RFID:
        """
        :param check: stands in for _check_channel
        :return AFC_U1_RFID: the reader, lane1 on ch0
        """
        reader = make_u1_reader()
        reader.register_lane(U1Lane("lane1"), 0)
        reader._check_channel = check if check is not None else Recorder()
        return reader

    @staticmethod
    def _bare_info(**over: Any) -> Dict[str, Any]:
        """
        :param over: keys that differ from a body carrying only the channel
        :return dict: the info a body with only ``channel`` packs
        """
        info: Dict[str, Any] = {
            "VENDOR": "", "MAIN_TYPE": "", "SUB_TYPE": "",
            "HOTEND_MIN_TEMP": 0, "HOTEND_MAX_TEMP": 0, "BED_TEMP": 0,
            "WEIGHT": 0, "COLOR_NUMS": 0, "CARD_UID": None, "MF_DATE": "",
        }
        info.update(over)
        return info

    def test_missing_channel_returns(self):
        reader = self._reader()
        reader._handle_webhook_scan(self._WebRequest({"type": "PLA"}))
        reader._handle_webhook_scan(self._WebRequest({"channel": "0"}))
        assert reader._check_channel.calls == []
        assert reader._webhook_channels_seen == set()
        assert reader.logger.messages == []

    def test_unmonitored_channel_returns(self):
        reader = self._reader()
        reader._handle_webhook_scan(self._WebRequest({"channel": 3}))
        assert reader._check_channel.calls == []
        assert reader._webhook_channels_seen == set()
        assert reader.logger.messages == []

    def test_full_payload_builds_info(self):
        reader = self._reader()
        reader._handle_webhook_scan(self._WebRequest({
            "channel": 0, "manufacturer": "Bambu", "type": "PLA",
            "sub_type": "Basic", "hotend_min_temp": 200, "hotend_max_temp": 220,
            "bed_temp": 60, "weight_grams": 1000,
            "colors": [0xFF112233, 0xFF445566],
            "card_uid": [1, 2, 3], "manufacturing_date": "20240101",
            "diameter_mm": 1.75, "density": 1.24, "serial_number": "S1",
            "sku": "SKU9", "drying_temp_c": 55, "drying_time_hours": 8,
        }))
        assert reader._webhook_channels_seen == {0}
        assert reader._check_channel.calls == [(("lane1", 0), {
            "info": {
                "VENDOR": "Bambu", "MAIN_TYPE": "PLA", "SUB_TYPE": "Basic",
                "HOTEND_MIN_TEMP": 200, "HOTEND_MAX_TEMP": 220, "BED_TEMP": 60,
                "WEIGHT": 1000, "COLOR_NUMS": 2, "CARD_UID": [1, 2, 3],
                "MF_DATE": "20240101", "DIAMETER": 1.75, "DENSITY": 1.24,
                "SERIAL": "S1", "SKU": "SKU9", "DRYING_TEMP": 55,
                "DRYING_TIME": 8, "RGB_1": 4279312947, "RGB_2": 4282668390,
            },
            "source": "webhook"})]
        assert reader.logger.messages == []

    def test_non_list_colors_becomes_empty(self):
        reader = self._reader()
        reader._handle_webhook_scan(self._WebRequest(
            {"channel": 0, "colors": "notalist"}))
        assert reader._check_channel.calls == [
            (("lane1", 0), {"info": self._bare_info(), "source": "webhook"})]
        assert reader.logger.messages == []

    def test_non_int_color_skipped(self):
        reader = self._reader()
        reader._handle_webhook_scan(self._WebRequest(
            {"channel": 0, "colors": ["bad", None, "4282664294"]}))
        assert reader._check_channel.calls == [(("lane1", 0), {
            "info": self._bare_info(COLOR_NUMS=3, RGB_3=4282664294),
            "source": "webhook"})]
        assert reader.logger.messages == []

    def test_zero_valued_optional_field_omitted(self):
        reader = self._reader()
        reader._handle_webhook_scan(self._WebRequest({
            "channel": 0, "drying_temp_c": 0, "serial_number": "",
            "sku": None, "density": 1.24}))
        assert reader._check_channel.calls == [(("lane1", 0), {
            "info": self._bare_info(DENSITY=1.24), "source": "webhook"})]
        assert reader.logger.messages == []

    def test_already_seen_channel_not_readded(self):
        reader = self._reader()
        reader._webhook_channels_seen = {0}
        reader._handle_webhook_scan(self._WebRequest({"channel": 0}))
        assert reader._webhook_channels_seen == {0}
        assert reader._check_channel.calls == [
            (("lane1", 0), {"info": self._bare_info(), "source": "webhook"})]
        assert reader.logger.messages == []

    def test_check_channel_error_logs_warning(self):
        reader = self._reader(check=Recorder(raises=RuntimeError("x")))
        reader._handle_webhook_scan(self._WebRequest({"channel": 0}))
        assert reader._webhook_channels_seen == {0}
        assert reader.logger.messages == [
            ("warning", "U1 RFID: webhook scan error ch0: x")]


class TestAFCU1RFIDCheckChannel:
    #: A lane tag as filament_detect reports it: the primary colour only.
    LANE_TAG = {"CARD_UID": U1_UID, "VENDOR": "Snapmaker", "MAIN_TYPE": "PLA",
                "SUB_TYPE": "Matte", "RGB_1": 0xFFFF0000,
                "HOTEND_MIN_TEMP": 190, "HOTEND_MAX_TEMP": 230, "BED_TEMP": 60}
    LANE_TAG_RAW = ("debug", "U1 RFID: ch0 raw tag info: {'CARD_UID': [86, 163, "
                             "106, 234], 'VENDOR': 'Snapmaker', 'MAIN_TYPE': "
                             "'PLA', 'SUB_TYPE': 'Matte', 'RGB_1': 4294901760, "
                             "'HOTEND_MIN_TEMP': 190, 'HOTEND_MAX_TEMP': 230, "
                             "'BED_TEMP': 60}")
    LANE_TAG_PARSED = ("debug", "U1 RFID: parsed 1 colour(s) ['ff0000'] from RGB "
                                "slots ['ff0000'] (no tag count field; "
                                "white-sentinel heuristic)")
    #: The slot info LANE_TAG maps to.
    LANE_SLOT = {"material": "PLA", "color_hex": "ff0000",
                 "multi_color": ["ff0000"], "is_dual_color": False, "sku": "",
                 "brand": "Snapmaker", "sub_type": "Matte", "diameter": 1.75,
                 "extruder_temp": 210, "bed_temp": 60, "mfg_date": None,
                 "uid": "56A36AEA", "extruder_temp_min": 190,
                 "extruder_temp_max": 230}
    LANE_LOADED = ("respond_info", "Spool loaded on lane1:\n"
                                   "  Name: Snapmaker PLA Matte\n"
                                   "  Brand: Snapmaker\n"
                                   "  Material: PLA\n"
                                   "  Color: #ff0000\n"
                                   "  Nozzle temp: 210°C\n"
                                   "  Bed temp: 60°C")
    #: The last-read record LANE_TAG leaves: the slot info's set values.
    LANE_RECORD = {"material": "PLA", "color_hex": "ff0000",
                   "multi_color": ["ff0000"], "is_dual_color": False,
                   "brand": "Snapmaker", "sub_type": "Matte", "diameter": 1.75,
                   "extruder_temp": 210, "bed_temp": 60, "uid": "56A36AEA",
                   "extruder_temp_min": 190, "extruder_temp_max": 230,
                   "decoded": True, "scan_time": 1700000000.0}
    #: A scanner tag: material only.
    SCAN_TAG = {"CARD_UID": U1_UID, "MAIN_TYPE": "PLA"}
    SCAN_TAG_OTHER = {"CARD_UID": U1_UID_OTHER, "MAIN_TYPE": "PLA"}
    NO_COLOUR_PARSED = ("debug", "U1 RFID: parsed 0 colour(s) [] from RGB slots "
                                 "[] (no tag count field; white-sentinel "
                                 "heuristic)")
    #: The slot info SCAN_TAG maps to.
    SCAN_SLOT = {"material": "PLA", "color_hex": "", "multi_color": [],
                 "is_dual_color": False, "sku": "", "brand": "", "sub_type": "",
                 "diameter": 1.75, "extruder_temp": None, "bed_temp": None,
                 "mfg_date": None, "uid": "56A36AEA"}
    #: The console report of one SCAN_TAG read on the standalone scanner.
    SCANNED_CONSOLE = ("respond_info", "Spool scanned on scanner-ch0:\n"
                                       "  Name: PLA\n  Material: PLA")
    #: The last-read record SCAN_TAG leaves.
    SCAN_RECORD = {"material": "PLA", "is_dual_color": False, "diameter": 1.75,
                   "uid": "56A36AEA", "decoded": True,
                   "scan_time": 1700000000.0}

    class _SyncSpy:
        """sync_rfid_to_spoolman as the reader calls it: each call is
        recorded, then the real one runs (with no Spoolman it stages a scanned
        spool and calls on_done)."""

        def __init__(self) -> None:
            """Start with no calls."""
            self.calls: List[Tuple[tuple, dict]] = []

        def __call__(self, *args: Any, **kwargs: Any) -> None:
            """Record the call and run the real sync."""
            self.calls.append((args, kwargs))
            sync_rfid_to_spoolman(*args, **kwargs)

    @pytest.fixture
    def sync(self, monkeypatch: pytest.MonkeyPatch) -> Any:
        """:return _SyncSpy: the spy, installed; the wall clock is pinned"""
        spy = self._SyncSpy()
        monkeypatch.setattr(u1_mod, "sync_rfid_to_spoolman", spy)
        monkeypatch.setattr(u1_mod, "time", U1Clock())
        return spy

    @staticmethod
    def _reader(values: Optional[Dict[str, Any]] = None,
                lane: Optional[U1Lane] = None) -> AFC_U1_RFID:
        """
        :param values: the [AFC_U1_rfid] options
        :param lane: registered on channel 0
        :return AFC_U1_RFID: the reader as klippy:ready leaves it, filament_detect
          attached and AFC's real AFC_spool behind it
        """
        reader = make_u1_reader(values)
        reader.afc = reader.printer.afc
        make_afc_spool(reader.printer)
        reader._filament_detect = U1KlipperObject()
        if lane is not None:
            reader.register_lane(lane, 0)
        return reader

    @staticmethod
    def _scan(reader: AFC_U1_RFID, tag: Dict[str, Any],
              source: str = "poll") -> None:
        """Read ``tag`` on the standalone scanner channel 0."""
        reader._check_channel(None, 0, info=dict(tag), source=source)

    @staticmethod
    def _scanned(uid_list: str, where: str = "scanner-ch0") -> List[LogLine]:
        """
        :param uid_list: the card UID as the raw tag dump prints it
        :param where: what the scan is reported on
        :return list: the reader's log for one scanner read of a PLA tag
        """
        return [
            ("debug", f"U1 RFID: ch0 raw tag info: {{'CARD_UID': {uid_list}, "
                      f"'MAIN_TYPE': 'PLA'}}"),
            ("debug", "U1 RFID: parsed 0 colour(s) [] from RGB slots [] (no tag "
                      "count field; white-sentinel heuristic)"),
            ("info", "U1 RFID: spool scanned: PLA"),
            ("raw", "// action:prompt_begin Spool Scanned"),
            ("raw", "// action:prompt_text Name: PLA"),
            ("raw", "// action:prompt_text Material: PLA"),
            ("raw", "// action:prompt_footer_button OK|RESPOND TYPE=command "
                    "MSG=action:prompt_end|info"),
            ("raw", "// action:prompt_show"),
        ]

    # lane reads

    def test_full_lane_load(self, sync):
        lane = U1Lane("lane1", spool_id=5, tool_loaded=True, material="PETG",
                      color="#00ff00")
        # The stable-read gate is for scanners: a lane read acts at once.
        reader = self._reader({"auto_spoolman_create": True,
                               "scanner_confirm_reads": 3}, lane)
        events: List[Any] = []
        reader.printer.register_event_handler("afc:tool_loaded", events.append)
        set_ids = u1_pass_through_spy(reader.afc.spool, "set_spoolID")
        reader._check_channel("lane1", 0, info=dict(self.LANE_TAG), source="poll")
        assert reader._last_uid == {0: U1_UID}
        assert reader._tag_reads == {"lane1": self.LANE_RECORD}
        # The old spool is cleared, then the lane takes the tag's values.
        assert set_ids == [((lane, ""), {})]
        assert lane.clear_lane_data.calls == [((), {})]
        assert (lane.spool_id, lane.material, lane.color) == (None, "PLA", "#ff0000")
        assert (lane.extruder_temp, lane.bed_temp) == (210.0, 60.0)
        assert (lane.spool_vendor, lane.sub_type, lane.weight) == (
            "Snapmaker", "Matte", 1000)
        args, kwargs = sync.calls[0]
        assert len(sync.calls) == 1
        assert args == (reader.afc, lane, self.LANE_SLOT, reader.logger, "U1 RFID")
        assert sorted(kwargs) == ["allow_create", "on_done", "reactor"]
        assert kwargs["allow_create"] is True
        assert kwargs["reactor"] is reader.reactor
        assert lane.send_lane_data.calls == [((), {})]
        assert reader.afc.save_vars.call_count == 2
        assert events == [lane]
        assert reader.printer.gcode.messages == [self.LANE_LOADED]
        assert reader.logger.messages == [
            self.LANE_TAG_RAW, self.LANE_TAG_PARSED,
            ("info", "U1 RFID: tag detected on lane1: Snapmaker PLA (#ff0000)"),
        ]

    def test_colorless_tag_desc_has_no_color_label(self, sync):
        lane = U1Lane("lane1")
        reader = self._reader(lane=lane)
        tag = {"CARD_UID": U1_UID, "VENDOR": "Snapmaker", "MAIN_TYPE": "PLA"}
        reader._check_channel("lane1", 0, info=tag, source="poll")
        assert reader._last_uid == {0: U1_UID}
        assert (lane.material, lane.color) == ("PLA", "")
        assert sync.calls[0][1]["allow_create"] is False
        assert reader.printer.gcode.messages == [
            ("respond_info", "Spool loaded on lane1:\n  Name: Snapmaker PLA\n"
                             "  Brand: Snapmaker\n  Material: PLA")]
        assert reader.logger.messages == [
            ("debug", "U1 RFID: ch0 raw tag info: {'CARD_UID': [86, 163, 106, "
                      "234], 'VENDOR': 'Snapmaker', 'MAIN_TYPE': 'PLA'}"),
            self.NO_COLOUR_PARSED,
            ("info", "U1 RFID: tag detected on lane1: Snapmaker PLA"),
        ]

    def test_no_spoolid_skips_clear(self, sync):
        lane = U1Lane("lane1", material="PETG")
        reader = self._reader(lane=lane)
        events: List[Any] = []
        reader.printer.register_event_handler("afc:tool_loaded", events.append)
        reader._check_channel("lane1", 0, info=dict(self.LANE_TAG), source="poll")
        # No spool to clear, so the lane's own material stands.
        assert lane.clear_lane_data.calls == []
        assert (lane.material, lane.color) == ("PETG", "#ff0000")
        assert reader.afc.save_vars.call_count == 1
        assert lane.send_lane_data.calls == [((), {})]
        assert events == []
        assert len(sync.calls) == 1
        assert reader.printer.gcode.messages == [self.LANE_LOADED]
        assert reader.logger.messages == [
            self.LANE_TAG_RAW, self.LANE_TAG_PARSED,
            ("info", "U1 RFID: tag detected on lane1: Snapmaker PLA (#ff0000)"),
        ]

    def test_info_none_and_no_fd_returns(self, sync):
        reader = self._reader(lane=U1Lane("lane1"))
        reader._filament_detect = None
        reader._get_channel_info = Recorder(result=dict(self.LANE_TAG))
        reader._check_channel("lane1", 0)
        assert reader._get_channel_info.calls == []
        assert reader._last_uid == {0: None}
        assert sync.calls == []
        assert reader.logger.messages == []
        assert reader.printer.gcode.messages == []

    def test_info_none_reads_live_and_returns_on_none(self, sync):
        reader = self._reader(lane=U1Lane("lane1"))
        read = Recorder(result=None)
        reader._filament_detect = U1KlipperObject(get_a_filament_info=read)
        reader._check_channel("lane1", 0)
        assert read.calls == [((0,), {})]
        assert reader._last_uid == {0: None}
        assert sync.calls == []
        assert reader.logger.messages == []
        assert reader.printer.gcode.messages == []

    def test_removal_clears_lane(self, sync):
        lane = U1Lane("lane1", spool_id=7, material="PLA", color="#ff0000")
        reader = self._reader(lane=lane)
        reader._last_uid[0] = U1_UID
        reader._pending_confirm[0] = (U1_UID, 1)
        clears = u1_pass_through_spy(reader, "_clear_lane")
        reader._check_channel("lane1", 0, info={"CARD_UID": 0})
        assert clears == [((lane, "lane1"), {})]
        assert reader._last_uid == {0: 0}
        assert reader._pending_confirm == {}
        assert (lane.spool_id, lane.material, lane.color) == (None, "", "")
        assert lane.clear_lane_data.calls == [((), {})]
        assert lane.send_lane_data.calls == [((), {})]
        assert reader.afc.save_vars.call_count == 2
        assert sync.calls == []
        assert reader.logger.messages == []
        assert reader.printer.gcode.messages == []

    def test_removal_skips_clear_when_locked(self, sync):
        lane = U1Lane("lane1", status="Loaded", spool_id=7, material="PLA",
                      color="#ff0000")
        reader = self._reader(lane=lane)
        reader._last_uid[0] = U1_UID
        reader._check_channel("lane1", 0, info={"CARD_UID": 0})
        assert reader._last_uid == {0: 0}
        assert (lane.spool_id, lane.material, lane.color) == (7, "PLA", "#ff0000")
        assert lane.send_lane_data.calls == []
        assert reader.afc.save_vars.call_count == 0
        assert reader.logger.messages == []
        assert reader.printer.gcode.messages == []

    def test_removal_without_a_lane_only_resets_uid(self, sync):
        reader = self._reader()
        reader._last_uid[0] = U1_UID
        reader._check_channel("ghost", 0, info={"CARD_UID": []})
        assert reader._last_uid == {0: 0}
        assert reader.afc.save_vars.call_count == 0
        assert reader.logger.messages == []
        assert reader.printer.gcode.messages == []

    def test_webhook_seen_suppresses_poll_read(self, sync):
        reader = self._reader(lane=U1Lane("lane1"))
        reader._webhook_channels_seen = {0}
        reader._check_channel("lane1", 0, info=dict(self.LANE_TAG), source="poll")
        assert reader._last_uid == {0: None}
        assert sync.calls == []
        assert reader.printer.gcode.messages == []
        assert reader.logger.messages == []

    def test_webhook_read_on_a_webhook_channel_is_processed(self, sync):
        lane = U1Lane("lane1")
        reader = self._reader({"webhook_grace": 1.0}, lane)
        reader._webhook_channels_seen = {0}
        info = {"CARD_UID": "56a36aea", "VENDOR": "Bambu", "MAIN_TYPE": "PLA",
                "COLOR_NUMS": 2, "RGB_1": 0xFFFF0000, "RGB_2": 0xFF0000FF}
        reader._check_channel("lane1", 0, info=info, source="webhook")
        assert reader._last_uid == {0: "56a36aea"}
        assert reader._pending_defer == {}
        assert reader.reactor.callbacks == []
        assert lane.multi_color == ["ff0000", "0000ff"]
        assert reader.printer.gcode.messages == [
            ("respond_info", "Spool loaded on lane1:\n  Name: Bambu PLA\n"
                             "  Brand: Bambu\n  Material: PLA\n"
                             "  Color: #ff0000")]
        assert reader.logger.messages == [
            ("debug", "U1 RFID: ch0 raw tag info: {'CARD_UID': '56a36aea', "
                      "'VENDOR': 'Bambu', 'MAIN_TYPE': 'PLA', 'COLOR_NUMS': 2, "
                      "'RGB_1': 4294901760, 'RGB_2': 4278190335}"),
            ("debug", "U1 RFID: parsed 2 colour(s) ['ff0000', '0000ff'] from "
                      "RGB slots ['ff0000', '0000ff'] (tag count=2)"),
            ("info", "U1 RFID: tag detected on lane1: Bambu PLA (#ff0000 + "
                     "#0000ff)"),
        ]

    def test_dedup_same_uid_returns(self, sync):
        reader = self._reader(lane=U1Lane("lane1"))
        reader._last_uid[0] = list(U1_UID)
        reader._check_channel("lane1", 0, info=dict(self.LANE_TAG), source="poll")
        assert reader._tag_reads == {}
        assert sync.calls == []
        assert reader.printer.gcode.messages == []
        assert reader.logger.messages == []

    def test_lane_none_returns(self, sync):
        reader = self._reader()
        reader._check_channel("ghost", 0, info=dict(self.LANE_TAG), source="poll")
        assert reader._last_uid == {}
        assert reader._tag_reads == {}
        assert sync.calls == []
        assert reader.logger.messages == []
        assert reader.printer.gcode.messages == []

    def test_locked_status_returns(self, sync):
        lane = U1Lane("lane1", status="Loaded", material="PETG")
        reader = self._reader(lane=lane)
        reader._check_channel("lane1", 0, info=dict(self.LANE_TAG), source="poll")
        assert reader._last_uid == {0: None}
        assert lane.material == "PETG"
        assert sync.calls == []
        assert reader.logger.messages == []
        assert reader.printer.gcode.messages == []

    def test_main_type_none_records_uid_only(self, sync):
        reader = self._reader(lane=U1Lane("lane1"))
        tag = {"CARD_UID": U1_UID, "MAIN_TYPE": "NONE"}
        reader._check_channel("lane1", 0, info=tag, source="poll")
        assert reader._last_uid == {0: U1_UID}
        assert reader._tag_reads == {}
        assert sync.calls == []
        assert reader.logger.messages == [
            ("debug", "U1 RFID: ch0 raw tag info: {'CARD_UID': [86, 163, 106, "
                      "234], 'MAIN_TYPE': 'NONE'}")]
        assert reader.printer.gcode.messages == []

    def test_webhook_grace_defers_new_tag(self, sync):
        reader = self._reader({"webhook_grace": 1.0}, U1Lane("lane1"))
        reader._check_channel("lane1", 0, info=dict(self.LANE_TAG), source="poll")
        assert reader._pending_defer == {0: U1_UID}
        assert reader._last_uid == {0: None}
        assert [when for _cb, when in reader.reactor.callbacks] == [101.0]
        assert sync.calls == []
        assert reader.logger.messages == []
        # When the grace runs out, the deferred read is re-checked for this tag.
        reader._grace_expired = Recorder()
        reader.reactor.run_callbacks(101.0)
        assert reader._grace_expired.calls == [(("lane1", 0, U1_UID), {})]
        assert reader.printer.gcode.messages == []

    def test_webhook_grace_not_rearmed_for_same_uid(self, sync):
        reader = self._reader({"webhook_grace": 1.0}, U1Lane("lane1"))
        reader._pending_defer[0] = list(U1_UID)
        reader._check_channel("lane1", 0, info=dict(self.LANE_TAG), source="poll")
        assert reader.reactor.callbacks == []
        assert reader._pending_defer == {0: U1_UID}
        assert reader._last_uid == {0: None}
        assert sync.calls == []
        assert reader.logger.messages == []
        assert reader.printer.gcode.messages == []

    def test_poll_final_bypasses_grace(self, sync):
        lane = U1Lane("lane1")
        reader = self._reader({"webhook_grace": 1.0}, lane)
        reader._check_channel("lane1", 0, info=dict(self.LANE_TAG),
                              source="poll-final")
        assert reader._last_uid == {0: U1_UID}
        assert reader._pending_defer == {}
        assert reader.reactor.callbacks == []
        assert len(sync.calls) == 1
        assert reader.printer.gcode.messages == [self.LANE_LOADED]
        assert reader.logger.messages == [
            self.LANE_TAG_RAW, self.LANE_TAG_PARSED,
            ("info", "U1 RFID: tag detected on lane1: Snapmaker PLA (#ff0000)"),
        ]

    def test_scanner_lane_scans_when_loaded_with_the_lane_auto_create(self, sync):
        ext = U1Extruder(name="e0", auto_spoolman_create=True)
        lane = U1Lane("lane1", extruder_obj=ext, spool_scanner=True,
                      status="Tooled")
        reader = self._reader({"scanner_auto_create": False}, lane)
        reader._check_channel("lane1", 0, info=dict(self.SCAN_TAG), source="poll")
        assert reader._last_uid == {0: U1_UID}
        assert reader._tag_reads == {"lane1": self.SCAN_RECORD}
        args, kwargs = sync.calls[0]
        assert args[1] is lane
        assert (kwargs["allow_create"], kwargs["set_next"]) == (True, True)
        assert reader.afc.spool.next_spool_info == self.SCAN_SLOT
        assert reader.printer.gcode.messages == [
            ("respond_info", "Spool scanned on lane1:\n  Name: PLA\n"
                             "  Material: PLA")]
        assert reader.logger.messages == self._scanned("[86, 163, 106, 234]")

    # standalone scanner reads

    def test_scan_waits_for_n_consecutive_reads(self, sync):
        reader = self._reader({"scanner_channels": "0",
                               "scanner_confirm_reads": 3})
        self._scan(reader, self.SCAN_TAG)
        assert reader._pending_confirm == {0: (U1_UID, 1)}
        self._scan(reader, self.SCAN_TAG)
        assert reader._pending_confirm == {0: (U1_UID, 2)}
        assert reader._last_uid == {}
        assert sync.calls == []
        self._scan(reader, self.SCAN_TAG)
        assert len(sync.calls) == 1
        assert reader._pending_confirm == {}
        assert reader._last_uid == {0: U1_UID}
        assert reader.logger.messages == [
            ("debug", "U1 RFID: ch0 UID 56A36AEA seen 1/3, waiting for a "
                      "stable read"),
            ("debug", "U1 RFID: ch0 UID 56A36AEA seen 2/3, waiting for a "
                      "stable read"),
        ] + self._scanned("[86, 163, 106, 234]")
        assert reader.printer.gcode.messages == [self.SCANNED_CONSOLE]

    def test_different_uid_mid_confirmation_resets_count(self, sync):
        """A corrupt read mid-stream must not accumulate: only a stable read
        acts."""
        reader = self._reader({"scanner_channels": "0",
                               "scanner_confirm_reads": 3})
        self._scan(reader, self.SCAN_TAG)
        self._scan(reader, self.SCAN_TAG_OTHER)
        assert reader._pending_confirm == {0: (U1_UID_OTHER, 1)}
        self._scan(reader, self.SCAN_TAG)
        assert reader._pending_confirm == {0: (U1_UID, 1)}
        assert sync.calls == []
        assert reader.logger.messages == [
            ("debug", "U1 RFID: ch0 UID 56A36AEA seen 1/3, waiting for a "
                      "stable read"),
            ("debug", "U1 RFID: ch0 UID 26A36AEA seen 1/3, waiting for a "
                      "stable read"),
            ("debug", "U1 RFID: ch0 UID 56A36AEA seen 1/3, waiting for a "
                      "stable read"),
        ]
        assert reader.printer.gcode.messages == []

    def test_confirm_reads_of_one_acts_immediately(self, sync):
        reader = self._reader({"scanner_channels": "0"})
        self._scan(reader, self.SCAN_TAG)
        assert len(sync.calls) == 1
        assert reader._pending_confirm == {}
        assert reader._last_uid == {0: U1_UID}
        assert reader.logger.messages == self._scanned("[86, 163, 106, 234]")
        assert reader.printer.gcode.messages == [self.SCANNED_CONSOLE]

    def test_webhook_bypasses_gate(self, sync):
        """A webhook is a full, authoritative read: no confirmation needed."""
        reader = self._reader({"scanner_channels": "0",
                               "scanner_confirm_reads": 3})
        self._scan(reader, self.SCAN_TAG, source="webhook")
        assert len(sync.calls) == 1
        assert reader._pending_confirm == {}
        assert reader._last_uid == {0: U1_UID}
        assert reader.logger.messages == self._scanned("[86, 163, 106, 234]")
        assert reader.printer.gcode.messages == [self.SCANNED_CONSOLE]

    def test_same_uid_never_refires(self, sync):
        reader = self._reader({"scanner_channels": "0"})
        self._scan(reader, self.SCAN_TAG)
        self._scan(reader, self.SCAN_TAG)
        assert len(sync.calls) == 1
        assert reader.printer.gcode.messages == [self.SCANNED_CONSOLE]
        assert reader.logger.messages == self._scanned("[86, 163, 106, 234]")

    def test_new_spool_after_first_fires_again(self, sync):
        reader = self._reader({"scanner_channels": "0"})
        self._scan(reader, self.SCAN_TAG)
        self._scan(reader, self.SCAN_TAG_OTHER)
        assert len(sync.calls) == 2
        assert reader._last_uid == {0: U1_UID_OTHER}
        assert reader._tag_reads == {
            "scanner-ch0": dict(self.SCAN_RECORD, uid="26A36AEA")}
        assert reader.logger.messages == (self._scanned("[86, 163, 106, 234]")
                                          + self._scanned("[38, 163, 106, 234]"))
        assert reader.printer.gcode.messages == [self.SCANNED_CONSOLE] * 2

    def test_tag_removal_clears_pending_confirmation(self, sync):
        reader = self._reader({"scanner_channels": "0",
                               "scanner_confirm_reads": 3})
        self._scan(reader, self.SCAN_TAG)
        self._scan(reader, {"CARD_UID": 0})
        assert reader._pending_confirm == {}
        assert reader._last_uid == {}
        assert sync.calls == []
        assert reader.logger.messages == [
            ("debug", "U1 RFID: ch0 UID 56A36AEA seen 1/3, waiting for a "
                      "stable read")]
        assert reader.printer.gcode.messages == []

    def test_scanner_keeps_last_uid_after_removal(self, sync):
        """A staged spool must not re-fire when it is presented again."""
        reader = self._reader({"scanner_channels": "0"})
        self._scan(reader, self.SCAN_TAG)
        self._scan(reader, {"CARD_UID": 0})
        assert reader._last_uid == {0: U1_UID}
        self._scan(reader, self.SCAN_TAG)
        assert len(sync.calls) == 1
        assert reader.logger.messages == self._scanned("[86, 163, 106, 234]")
        assert reader.printer.gcode.messages == [self.SCANNED_CONSOLE]

    def test_main_type_none_records_uid_but_does_not_act(self, sync):
        reader = self._reader({"scanner_channels": "0"})
        self._scan(reader, {"CARD_UID": U1_UID, "MAIN_TYPE": "NONE"})
        self._scan(reader, {"CARD_UID": U1_UID_OTHER})
        assert reader._last_uid == {0: U1_UID_OTHER}
        assert reader._tag_reads == {}
        assert sync.calls == []
        assert reader.printer.gcode.messages == []
        assert reader.logger.messages == [
            ("debug", "U1 RFID: ch0 raw tag info: {'CARD_UID': [86, 163, 106, "
                      "234], 'MAIN_TYPE': 'NONE'}"),
            ("debug", "U1 RFID: ch0 raw tag info: {'CARD_UID': [38, 163, 106, "
                      "234]}"),
        ]

    def test_scanner_sets_next_spool_staging(self, sync):
        """A scanner read stages the spool as next_spool_id instead of
        assigning it to a lane."""
        reader = self._reader({"scanner_channels": "0",
                               "scanner_auto_create": True})
        self._scan(reader, self.SCAN_TAG)
        args, kwargs = sync.calls[0]
        assert args == (reader.afc, None, self.SCAN_SLOT, reader.logger, "U1 RFID")
        assert sorted(kwargs) == ["allow_create", "on_done", "reactor", "set_next"]
        assert (kwargs["allow_create"], kwargs["set_next"]) == (True, True)
        assert kwargs["reactor"] is reader.reactor
        assert reader.afc.spool.next_spool_info == self.SCAN_SLOT
        assert reader._tag_reads == {"scanner-ch0": self.SCAN_RECORD}
        assert reader.afc.save_vars.call_count == 0
        assert [when for _cb, when in reader.reactor.callbacks] == [110.0]
        assert reader.printer.gcode.messages == [self.SCANNED_CONSOLE]
        assert reader.logger.messages == self._scanned("[86, 163, 106, 234]")

    def test_a_reader_without_read_records_starts_them(self, sync):
        """State from before the read records existed (no _tag_reads at all)
        gets a fresh record table on the first read."""
        reader = self._reader({"scanner_channels": "0"})
        del reader._tag_reads
        self._scan(reader, self.SCAN_TAG)
        assert reader._tag_reads == {"scanner-ch0": self.SCAN_RECORD}
        assert reader.printer.gcode.messages == [self.SCANNED_CONSOLE]
        assert reader.logger.messages == self._scanned("[86, 163, 106, 234]")


class TestAFCU1RFIDGraceExpired:
    @staticmethod
    def _reader(check: Optional[Recorder] = None) -> AFC_U1_RFID:
        """
        :param check: stands in for _check_channel
        :return AFC_U1_RFID: the reader
        """
        reader = make_u1_reader()
        reader._check_channel = check if check is not None else Recorder()
        return reader

    def test_superseded_uid_skips(self):
        reader = self._reader()
        reader._pending_defer = {0: U1_UID_OTHER}
        reader._grace_expired("lane1", 0, U1_UID)
        assert reader._check_channel.calls == []
        assert reader._pending_defer == {0: U1_UID_OTHER}
        assert reader.logger.messages == []

    def test_webhook_landed_skips_read(self):
        reader = self._reader()
        reader._pending_defer = {0: U1_UID}
        reader._webhook_channels_seen = {0}
        reader._grace_expired("lane1", 0, U1_UID)
        assert reader._pending_defer == {}
        assert reader._check_channel.calls == []
        assert reader.logger.messages == []

    def test_processes_deferred_read(self):
        reader = self._reader()
        reader._pending_defer = {0: U1_UID, 1: U1_UID_OTHER}
        reader._grace_expired("lane1", 0, U1_UID)
        assert reader._pending_defer == {1: U1_UID_OTHER}
        assert reader._check_channel.calls == [
            (("lane1", 0), {"source": "poll-final"})]
        assert reader.logger.messages == []

    def test_deferred_read_error_logs_warning(self):
        reader = self._reader(check=Recorder(raises=RuntimeError("bad")))
        reader._pending_defer = {0: U1_UID}
        reader._grace_expired(None, 0, U1_UID)
        assert reader._pending_defer == {}
        assert reader._check_channel.calls == [
            ((None, 0), {"source": "poll-final"})]
        assert reader.logger.messages == [
            ("warning", "U1 RFID: deferred read error ch0: bad")]


class TestAFCU1RFIDClearLane:
    @staticmethod
    def _reader() -> AFC_U1_RFID:
        """:return AFC_U1_RFID: a reader past klippy:ready, AFC's real
        AFC_spool behind it"""
        reader = make_u1_reader()
        reader.afc = reader.printer.afc
        make_afc_spool(reader.printer)
        return reader

    def test_clears_material_color_and_spool(self):
        reader = self._reader()
        lane = U1Lane("lane1", spool_id=7, material="PLA", color="#ff0000")
        lane.extruder_temp = 210.0
        set_ids = u1_pass_through_spy(reader.afc.spool, "set_spoolID")
        reader._clear_lane(lane, "lane1")
        assert set_ids == [((lane, ""), {})]
        assert (lane.material, lane.color) == ("", "")
        # AFC_spool cleared the spool and what it brought.
        assert (lane.spool_id, lane.extruder_temp) == (None, None)
        assert lane.clear_lane_data.calls == [((), {})]
        assert lane.send_lane_data.calls == [((), {})]
        assert reader.afc.save_vars.call_count == 2
        assert reader.logger.messages == []

    def test_skips_spool_clear_when_unset(self):
        reader = self._reader()
        lane = U1Lane("lane1", material="PLA", color="#ff0000")
        lane.extruder_temp = 210.0
        reader._clear_lane(lane, "lane1")
        assert (lane.material, lane.color) == ("", "")
        assert (lane.spool_id, lane.extruder_temp) == (None, 210.0)
        assert lane.clear_lane_data.calls == []
        assert lane.send_lane_data.calls == [((), {})]
        assert reader.afc.save_vars.call_count == 1
        assert reader.logger.messages == []

    def test_spool_clear_error_logs_warning(self):
        reader = self._reader()
        lane = U1Lane("lane2", spool_id=7, material="PLA",
                      clear_raises=RuntimeError("nope"))
        reader._clear_lane(lane, "lane2")
        assert lane.material == ""
        assert lane.send_lane_data.calls == [((), {})]
        assert reader.afc.save_vars.call_count == 1
        assert reader.logger.messages == [
            ("warning", "U1 RFID: failed to clear spool_id on lane2: nope")]


class TestAFCU1RFIDGetChannelInfo:
    @staticmethod
    def _reader(**api: Any) -> AFC_U1_RFID:
        """
        :param api: filament_detect's methods, by name
        :return AFC_U1_RFID: the reader with that filament_detect attached
        """
        reader = make_u1_reader()
        reader._filament_detect = U1KlipperObject(**api)
        return reader

    def test_get_a_filament_info_dict(self):
        get_all = Recorder(result=[{"CARD_UID": [9]}])
        reader = self._reader(get_a_filament_info=Recorder(result={"CARD_UID": [1]}),
                              get_all_filament_info=get_all)
        assert reader._get_channel_info(0) == {"CARD_UID": [1]}
        assert reader._filament_detect.get_a_filament_info.calls == [((0,), {})]
        assert get_all.calls == []
        assert reader.logger.messages == []

    def test_get_a_filament_info_non_dict_falls_through(self):
        reader = self._reader(get_a_filament_info=Recorder(result=None),
                              get_all_filament_info=Recorder(
                                  result=[{"CARD_UID": [2]}]))
        assert reader._get_channel_info(0) == {"CARD_UID": [2]}
        assert reader.logger.messages == []

    def test_get_a_filament_info_error_falls_through(self):
        reader = self._reader(
            get_a_filament_info=Recorder(raises=RuntimeError("x")),
            get_all_filament_info=Recorder(result={"0": {"CARD_UID": [3]}}))
        assert reader._get_channel_info(0) == {"CARD_UID": [3]}
        assert reader.logger.messages == []

    def test_get_all_dict_by_int_key(self):
        reader = self._reader(get_all_filament_info=Recorder(
            result={0: {"CARD_UID": [4]}, "0": {"CARD_UID": [9]}}))
        assert reader._get_channel_info(0) == {"CARD_UID": [4]}
        assert reader.logger.messages == []

    def test_get_all_error_falls_through_to_status(self):
        reader = self._reader(
            get_all_filament_info=Recorder(raises=RuntimeError("x")),
            get_status=Recorder(result={"info": [{"CARD_UID": [5]}]}))
        assert reader._get_channel_info(0) == {"CARD_UID": [5]}
        assert reader.logger.messages == []

    def test_status_entry_without_uid_returns_none(self):
        reader = self._reader(get_status=Recorder(
            result={"info": [{"MAIN_TYPE": "PLA", "CARD_UID": []}]}))
        assert reader._get_channel_info(0) is None
        assert reader.logger.messages == []

    def test_get_all_list_out_of_range_falls_through(self):
        status = Recorder(result={"info": [{}, {"CARD_UID": [6]}]})
        reader = self._reader(get_all_filament_info=Recorder(
            result=({"CARD_UID": [1]},)), get_status=status)
        assert reader._get_channel_info(1) == {"CARD_UID": [6]}
        assert status.calls == [((), {})]
        assert reader.logger.messages == []

    def test_get_all_list_entry_not_dict_falls_through(self):
        status = Recorder(result={"info": [{"CARD_UID": [7]}]})
        reader = self._reader(get_all_filament_info=Recorder(result=["not-a-dict"]),
                              get_status=status)
        assert reader._get_channel_info(0) == {"CARD_UID": [7]}
        assert reader.logger.messages == []

    def test_get_all_dict_missing_key_falls_through(self):
        status = Recorder(result={"info": [{"CARD_UID": [8]}]})
        reader = self._reader(get_all_filament_info=Recorder(
            result={"9": {"CARD_UID": [1]}}), get_status=status)
        assert reader._get_channel_info(0) == {"CARD_UID": [8]}
        assert reader.logger.messages == []

    def test_status_raises_returns_none(self):
        reader = self._reader(get_status=Recorder(raises=RuntimeError("x")))
        assert reader._get_channel_info(0) is None
        assert reader.logger.messages == []

    def test_status_non_dict_returns_none(self):
        reader = self._reader(get_status=Recorder(result=[{"CARD_UID": [1]}]))
        assert reader._get_channel_info(0) is None
        assert reader.logger.messages == []

    def test_status_channel_out_of_range_returns_none(self):
        reader = self._reader(get_status=Recorder(
            result={"info": [{"CARD_UID": [1]}]}))
        assert reader._get_channel_info(9) is None
        reader = self._reader(get_status=Recorder(result={"info": None}))
        assert reader._get_channel_info(0) is None
        assert reader.logger.messages == []

    def test_status_entry_not_dict_returns_none(self):
        reader = self._reader(get_status=Recorder(result={"info": ["not-a-dict"]}))
        assert reader._get_channel_info(0) is None
        assert reader.logger.messages == []

    def test_no_api_returns_none(self):
        reader = self._reader()
        assert reader._get_channel_info(0) is None
        assert reader.logger.messages == []


class TestAFCU1RFIDTagColorCount:
    def test_reads_color_nums(self):
        reader = make_u1_reader()
        assert reader._tag_color_count({"MAIN_TYPE": "PLA", "COLOR_NUMS": 2}) == 2
        assert reader._tag_color_count({"color_nums": 4}) == 4
        assert reader.logger.messages == []

    def test_reads_colour_count_variant(self):
        reader = make_u1_reader()
        assert reader._tag_color_count({"COLOUR_COUNT": "3"}) == 3
        assert reader.logger.messages == []

    def test_non_numeric_skipped(self):
        reader = make_u1_reader()
        assert reader._tag_color_count({"COLOR_NUM": "abc"}) is None
        assert reader._tag_color_count({"COLOR_NUM": None,
                                        "COLOUR_NUMS": "2"}) == 2
        assert reader.logger.messages == []

    def test_zero_count_skipped(self):
        reader = make_u1_reader()
        assert reader._tag_color_count({"COLOR_NUM": 0}) is None
        assert reader._tag_color_count({"COLOR_NUM": 0, "COLOR_COUNT": 3}) == 3
        assert reader.logger.messages == []

    def test_no_matching_key_returns_none(self):
        reader = make_u1_reader()
        # A colour key without a count, and a count key without a colour.
        assert reader._tag_color_count({"MAIN_TYPE": "PLA", "COLOR_HEX": 7,
                                        "RGB_1": 5, "NUMBER": 2}) is None
        assert reader.logger.messages == []


class TestAFCU1RFIDMapToSlotInfo:
    #: The debug line for a tag with no colour slots and no colour count.
    NO_COLOUR = ("debug", "U1 RFID: parsed 0 colour(s) [] from RGB slots [] "
                          "(no tag count field; white-sentinel heuristic)")

    @staticmethod
    def _slot(**over: Any) -> Dict[str, Any]:
        """
        :param over: keys that differ from a bare PLA tag's slot info
        :return dict: the slot info
        """
        slot: Dict[str, Any] = {
            "material": "PLA", "color_hex": "", "multi_color": [],
            "is_dual_color": False, "sku": "", "brand": "", "sub_type": "",
            "diameter": 1.75, "extruder_temp": None, "bed_temp": None,
            "mfg_date": None, "uid": None,
        }
        slot.update(over)
        return slot

    def test_weight_consumed(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "WEIGHT": 600}) == (
            self._slot(weight_g=600))
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_weight_zero_skipped(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "WEIGHT": 0}) == (
            self._slot())
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_weight_invalid_skipped(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "WEIGHT": "junk"}) == (
            self._slot())
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_diameter_from_tag(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "DIAMETER": 2.85}) == (
            self._slot(diameter=2.85))
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_diameter_defaults_without_key(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA"}) == self._slot()
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_diameter_invalid_defaults(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "DIAMETER": "junk"}) == (
            self._slot())
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_temp_range_kept(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "HOTEND_MIN_TEMP": 200,
                                         "HOTEND_MAX_TEMP": 230}) == self._slot(
            extruder_temp=215, extruder_temp_min=200, extruder_temp_max=230)
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_temp_range_absent_when_unset(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "HOTEND_MIN_TEMP": 0,
                                         "HOTEND_MAX_TEMP": 0}) == self._slot()
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_optional_rich_keys_copied(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({
            "MAIN_TYPE": "PLA", "SERIAL": "S1", "DENSITY": 1.31,
            "DRYING_TEMP": 70, "DRYING_TIME": 8, "COLOR_NUMS": 2}) == self._slot(
            serial="S1", density=1.31, drying_temp=70, drying_time_h=8,
            color_count=2)
        assert reader.logger.messages == [
            ("debug", "U1 RFID: parsed 0 colour(s) [] from RGB slots [] "
                      "(tag count=2)")]

    def test_optional_rich_keys_skip_unset(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({
            "MAIN_TYPE": "PLA", "SERIAL": "", "DENSITY": 0,
            "DRYING_TEMP": None, "COLOR_NUMS": "0"}) == self._slot()
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_full_tag_with_color_count(self):
        reader = make_u1_reader()
        info = {
            "RGB_1": 0xFF112233, "RGB_2": 0xFF445566, "RGB_3": 0xFF778899,
            "COLOR_NUMS": 2, "MAIN_TYPE": "PLA", "HOTEND_MAX_TEMP": 220,
            "HOTEND_MIN_TEMP": 200, "BED_TEMP": 60, "VENDOR": "Bambu",
            "SUB_TYPE": "Basic", "CARD_UID": [1, 2, 3, 4], "MF_DATE": "20240101",
            "WEIGHT": 1000, "DIAMETER": 1.75, "SKU": 123, "SERIAL": "S1",
            "DENSITY": 1.24, "DRYING_TEMP": 55, "DRYING_TIME": 8,
        }
        assert reader._map_to_slot_info(info) == {
            "material": "PLA", "color_hex": "112233",
            "multi_color": ["112233", "445566"], "is_dual_color": True,
            "sku": "123", "brand": "Bambu", "sub_type": "Basic",
            "diameter": 1.75, "extruder_temp": 210, "bed_temp": 60,
            "mfg_date": "2024-01-01", "uid": "01020304",
            "extruder_temp_min": 200, "extruder_temp_max": 220,
            "weight_g": 1000, "serial": "S1", "density": 1.24,
            "drying_temp": 55, "drying_time_h": 8, "color_count": 2,
        }
        assert reader.logger.messages == [
            ("debug", "U1 RFID: parsed 2 colour(s) ['112233', '445566'] from RGB "
                      "slots ['112233', '445566', '778899'] (tag count=2)")]

    def test_white_sentinel_heuristic_without_count(self):
        reader = make_u1_reader()
        info = {"RGB_1": 0xAA112233, "RGB_2": 0xFFFFFFFF, "MAIN_TYPE": "PLA"}
        assert reader._map_to_slot_info(info) == self._slot(
            color_hex="112233", multi_color=["112233"])
        assert reader.logger.messages == [
            ("debug", "U1 RFID: parsed 1 colour(s) ['112233'] from RGB slots "
                      "['112233', 'ffffff'] (no tag count field; white-sentinel "
                      "heuristic)")]

    def test_white_first_colour_is_kept(self):
        reader = make_u1_reader()
        info = {"RGB_1": 0xFFFFFFFF, "RGB_2": 0xFFFEFEFE, "RGB_3": 0xFFFFFFFF,
                "MAIN_TYPE": "PLA"}
        assert reader._map_to_slot_info(info) == self._slot(
            color_hex="ffffff", multi_color=["ffffff", "fefefe"],
            is_dual_color=True)
        assert reader.logger.messages == [
            ("debug", "U1 RFID: parsed 2 colour(s) ['ffffff', 'fefefe'] from RGB "
                      "slots ['ffffff', 'fefefe', 'ffffff'] (no tag count field; "
                      "white-sentinel heuristic)")]

    def test_duplicate_colors_deduped_with_count(self):
        reader = make_u1_reader()
        info = {"RGB_1": 0xFF112233, "RGB_2": 0xFF112233, "COLOR_NUMS": 2,
                "MAIN_TYPE": "PLA"}
        assert reader._map_to_slot_info(info) == self._slot(
            color_hex="112233", multi_color=["112233"], color_count=2)
        assert reader.logger.messages == [
            ("debug", "U1 RFID: parsed 1 colour(s) ['112233'] from RGB slots "
                      "['112233', '112233'] (tag count=2)")]

    def test_duplicate_colors_deduped_in_heuristic(self):
        reader = make_u1_reader()
        info = {"RGB_1": 0xAA112233, "RGB_2": 0xBB112233, "MAIN_TYPE": "PLA"}
        assert reader._map_to_slot_info(info) == self._slot(
            color_hex="112233", multi_color=["112233"])
        assert reader.logger.messages == [
            ("debug", "U1 RFID: parsed 1 colour(s) ['112233'] from RGB slots "
                      "['112233', '112233'] (no tag count field; white-sentinel "
                      "heuristic)")]

    def test_skips_blank_and_bad_rgb_values(self):
        reader = make_u1_reader()
        info = {"RGB_10": 0xFF0A0A0A, "RGB_1": "", "RGB_2": "zz", "RGB_X": 5,
                "RGB_3": 0xFF010203, "RGB_4": None, "MAIN_TYPE": "PLA"}
        # Slots go in number order: RGB_3 before RGB_10.
        assert reader._map_to_slot_info(info) == self._slot(
            color_hex="010203", multi_color=["010203", "0a0a0a"],
            is_dual_color=True)
        assert reader.logger.messages == [
            ("debug", "U1 RFID: parsed 2 colour(s) ['010203', '0a0a0a'] from RGB "
                      "slots ['010203', '0a0a0a'] (no tag count field; "
                      "white-sentinel heuristic)")]

    def test_extruder_temp_max_only(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"HOTEND_MAX_TEMP": 240}) == self._slot(
            material="", extruder_temp=240, extruder_temp_max=240)
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_extruder_temp_none_with_only_a_minimum(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA",
                                         "HOTEND_MIN_TEMP": 190}) == self._slot(
            extruder_temp_min=190)
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_extruder_temp_none_when_no_temps(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "BED_TEMP": 0}) == (
            self._slot())
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_bad_diameter_defaults(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "DIAMETER": "bad"}) == (
            self._slot())
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_zero_diameter_defaults(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "DIAMETER": 0}) == (
            self._slot())
        assert reader.logger.messages == [self.NO_COLOUR]

    def test_vendor_none_blanked_and_bad_weight(self):
        reader = make_u1_reader()
        assert reader._map_to_slot_info({"MAIN_TYPE": "PLA", "VENDOR": "None",
                                         "WEIGHT": "bad", "SKU": 0}) == (
            self._slot())
        assert reader.logger.messages == [self.NO_COLOUR]


class TestAFCU1RFIDFmtMfgDate:
    def test_none_when_empty(self):
        assert AFC_U1_RFID._fmt_mfg_date("") is None
        assert AFC_U1_RFID._fmt_mfg_date(None) is None
        assert AFC_U1_RFID._fmt_mfg_date("   ") is None

    def test_none_for_epoch(self):
        assert AFC_U1_RFID._fmt_mfg_date("19700101") is None
        assert AFC_U1_RFID._fmt_mfg_date("1970-01-01") is None

    def test_none_for_zero_strings(self):
        assert AFC_U1_RFID._fmt_mfg_date("00000000") is None
        assert AFC_U1_RFID._fmt_mfg_date("0") is None

    def test_formats_yyyymmdd(self):
        assert AFC_U1_RFID._fmt_mfg_date("20240517") == "2024-05-17"
        assert AFC_U1_RFID._fmt_mfg_date(20240517) == "2024-05-17"

    def test_passthrough_iso(self):
        assert AFC_U1_RFID._fmt_mfg_date(" 2024-05-17 ") == "2024-05-17"
        assert AFC_U1_RFID._fmt_mfg_date("2024051") == "2024051"
        assert AFC_U1_RFID._fmt_mfg_date("2024-5-1") == "2024-5-1"


class TestAFCU1RFIDFmtUid:
    def test_none_when_empty(self):
        assert AFC_U1_RFID._fmt_uid(None) is None
        assert AFC_U1_RFID._fmt_uid([]) is None
        assert AFC_U1_RFID._fmt_uid("  ") is None

    def test_byte_list_to_hex(self):
        assert AFC_U1_RFID._fmt_uid([123, 240, 175, 255]) == "7BF0AFFF"
        assert AFC_U1_RFID._fmt_uid((1, 0x1AB)) == "01AB"

    def test_bad_byte_list_returns_none(self):
        assert AFC_U1_RFID._fmt_uid([1, "x"]) is None
        assert AFC_U1_RFID._fmt_uid([1, None]) is None

    def test_string_uppercased(self):
        assert AFC_U1_RFID._fmt_uid(" 7bf0afff ") == "7BF0AFFF"


class TestAFCU1RFIDNotifyScan:
    #: The prompt's OK button.
    FOOTER = ("raw", "// action:prompt_footer_button OK|RESPOND TYPE=command "
                     "MSG=action:prompt_end|info")

    class _ExceptionManager:
        """The U1 factory display's exception_manager: records each raise."""

        def __init__(self) -> None:
            """Start with nothing raised."""
            self.raised: List[Dict[str, Any]] = []

        def raise_exception_async(self, **kwargs: Any) -> None:
            """Record one display exception."""
            self.raised.append(kwargs)

    @staticmethod
    def _reader(display: Optional[Any] = None) -> AFC_U1_RFID:
        """
        :param display: registered as exception_manager
        :return AFC_U1_RFID: a reader past klippy:ready's AFC lookup
        """
        reader = make_u1_reader()
        reader.afc = reader.printer.afc
        if display is not None:
            reader.printer.add_object("exception_manager", display)
        return reader

    def test_lane_load_console_only(self):
        display = self._ExceptionManager()
        reader = self._reader(display)
        slot = {"sub_type": "Basic", "extruder_temp": 210, "bed_temp": 60}
        reader._notify_scan("Bambu", "PLA", "FF0000", slot, lane_name="lane1")
        assert reader.printer.gcode.messages == [
            ("respond_info", "Spool loaded on lane1:\n  Name: Bambu PLA Basic\n"
                             "  Brand: Bambu\n  Material: PLA\n  Color: #FF0000\n"
                             "  Nozzle temp: 210°C\n  Bed temp: 60°C")]
        # A lane load gets no popup and no display message.
        assert display.raised == []
        assert reader.reactor.callbacks == []
        assert reader.logger.messages == []

    def test_lane_load_no_name_header(self):
        reader = self._reader()
        reader._notify_scan("", "", "", {}, lane_name="")
        assert reader.printer.gcode.messages == [("respond_info", "Spool loaded:")]
        assert reader.logger.messages == []

    def test_scanner_emits_prompt_and_exception(self):
        display = self._ExceptionManager()
        reader = self._reader(display)
        reader.register_lane(U1Lane("lane1"), 2)
        reader._notify_scan("Bambu", "PLA", "FF0000", {"sub_type": "Basic"},
                            lane_name="lane1", is_scanner=True)
        assert reader.printer.gcode.messages == [
            ("respond_info", "Spool scanned on lane1:\n  Name: Bambu PLA Basic\n"
                             "  Brand: Bambu\n  Material: PLA\n  Color: #FF0000")]
        assert display.raised == [{"id": 529, "index": 2, "code": 99,
                                   "message": "Spool Scanned: Bambu PLA Basic",
                                   "oneshot": 1, "level": 1}]
        prompt = [
            ("raw", "// action:prompt_begin Spool Scanned"),
            ("raw", "// action:prompt_text Name: Bambu PLA Basic"),
            ("raw", "// action:prompt_text Brand: Bambu"),
            ("raw", "// action:prompt_text Material: PLA"),
            ("raw", "// action:prompt_text Color: #FF0000"),
            self.FOOTER,
            ("raw", "// action:prompt_show"),
        ]
        assert reader.logger.messages == prompt
        # Ten seconds on, the popup closes itself.
        assert [when for _cb, when in reader.reactor.callbacks] == [110.0]
        reader.reactor.run_callbacks(110.0)
        assert reader.logger.messages == prompt + [("raw", "// action:prompt_end")]

    def test_scanner_prompt_includes_temps_and_spool_id(self):
        reader = self._reader()
        lane = U1Lane("lane1", spool_id=42, material="PLA")
        # set_spoolID has filled the lane from Spoolman; it carries no name or
        # vendor, so those stay the tag's.
        lane.extruder_temp = 210
        lane.bed_temp = 60
        reader.register_lane(lane, 0)
        reader._notify_scan("Bambu", "PLA", "FF0000", {"sub_type": ""},
                            lane_name="lane1", is_scanner=True)
        assert reader.printer.gcode.messages == [
            ("respond_info", "Spool scanned on lane1:\n  Name: Bambu PLA\n"
                             "  Brand: Bambu\n  Material: PLA\n  Color: #FF0000\n"
                             "  Nozzle temp: 210°C\n  Bed temp: 60°C\n"
                             "  Spoolman ID: 42")]
        assert reader.logger.messages == [
            ("raw", "// action:prompt_begin Spool Scanned"),
            ("raw", "// action:prompt_text Name: Bambu PLA"),
            ("raw", "// action:prompt_text Brand: Bambu"),
            ("raw", "// action:prompt_text Material: PLA"),
            ("raw", "// action:prompt_text Color: #FF0000"),
            ("raw", "// action:prompt_text Nozzle: 210°C"),
            ("raw", "// action:prompt_text Bed: 60°C"),
            ("raw", "// action:prompt_text Spoolman ID: 42"),
            self.FOOTER,
            ("raw", "// action:prompt_show"),
        ]

    def test_scanner_no_exception_manager(self):
        reader = self._reader()
        reader._notify_scan("", "", "", {}, lane_name="", is_scanner=True)
        assert reader.printer.gcode.messages == [("respond_info", "Spool scanned:")]
        assert reader.logger.messages == [
            ("raw", "// action:prompt_begin Spool Scanned"),
            self.FOOTER,
            ("raw", "// action:prompt_show"),
        ]

    def test_scanner_with_nothing_known_shows_the_bare_title(self):
        display = self._ExceptionManager()
        reader = self._reader(display)
        reader._notify_scan("", "", "", {}, lane_name="scanner-ch3",
                            is_scanner=True)
        assert display.raised == [{"id": 529, "index": 0, "code": 99,
                                   "message": "Spool Scanned", "oneshot": 1,
                                   "level": 1}]
        assert reader.printer.gcode.messages == [
            ("respond_info", "Spool scanned on scanner-ch3:")]
        assert reader.logger.messages == [
            ("raw", "// action:prompt_begin Spool Scanned"), self.FOOTER,
            ("raw", "// action:prompt_show")]

    def test_notification_error_logs_warning(self):
        reader = self._reader()
        reader._notify_scan("Bambu", "PLA", "FF0000", None, lane_name="lane1")
        assert reader.printer.gcode.messages == []
        assert reader.logger.messages == [
            ("warning", "U1 RFID: notification error: 'NoneType' object has no "
                        "attribute 'get'")]

    def test_enrichment_overlays_spool_record(self):
        """The lane, which set_spoolID has just filled from Spoolman, wins over
        the tag, so the read-out matches the lane it describes."""
        reader = self._reader()
        lane = U1Lane("lane1", spool_id=42, material="PETG")
        lane.filament_name = "Stored Name"
        lane.spool_vendor = "StoredBrand"
        lane.extruder_temp = 230
        lane.bed_temp = 80
        reader.register_lane(lane, 0)
        reader._notify_scan("Bambu", "PLA", "FF0000",
                            {"sub_type": "", "extruder_temp": 210, "bed_temp": 60},
                            lane_name="lane1")
        assert reader.printer.gcode.messages == [
            ("respond_info", "Spool loaded on lane1:\n  Name: Stored Name\n"
                             "  Brand: StoredBrand\n  Material: PETG\n"
                             "  Color: #FF0000\n  Nozzle temp: 230°C\n"
                             "  Bed temp: 80°C\n  Spoolman ID: 42")]
        assert reader.logger.messages == []


class TestAFCU1RFIDForceRead:
    class _Reads:
        """filament_detect.get_a_filament_info answering from a script; the
        last answer repeats."""

        def __init__(self, *answers: Optional[Dict[str, Any]]) -> None:
            """:param answers: what each read returns, in order"""
            self.answers = list(answers)
            self.calls: List[int] = []

        def __call__(self, channel: int) -> Optional[Dict[str, Any]]:
            """
            :param channel: the channel read
            :return Optional[dict]: the next scripted answer
            """
            self.calls.append(channel)
            return self.answers[min(len(self.calls), len(self.answers)) - 1]

    @staticmethod
    def _reader(reads: Any) -> AFC_U1_RFID:
        """
        :param reads: filament_detect's get_a_filament_info, or None for no
          filament_detect
        :return AFC_U1_RFID: lane1 on ch0 with a spool seen, its clock moving
          0.3s per read and _check_channel recording what it is given
        """
        reader = make_u1_reader(printer=U1Printer(step=0.3))
        reader.register_lane(U1Lane("lane1"), 0)
        reader._last_uid[0] = U1_UID_OTHER
        reader._gcode = reader.printer.gcode
        if reads is not None:
            reader._filament_detect = U1KlipperObject(get_a_filament_info=reads)
        reader._check_channel = Recorder()
        return reader

    def test_unknown_lane_returns(self):
        reads = self._Reads({"CARD_UID": U1_UID})
        reader = self._reader(reads)
        reader.force_read("ghost")
        assert reader._last_uid == {0: U1_UID_OTHER}
        assert reader._gcode.scripts == []
        assert reads.calls == []
        assert reader._check_channel.calls == []
        assert reader.logger.messages == []

    def test_trigger_failure_warns(self):
        reader = self._reader(None)
        reader.force_read("lane1")
        assert reader._last_uid == {0: None}
        assert reader._check_channel.calls == []
        assert reader.logger.messages == [
            ("warning", "U1 RFID: force_read failed to trigger update for lane1")]

    def test_reads_within_deadline(self):
        tag = {"CARD_UID": U1_UID, "MAIN_TYPE": "PLA"}
        reads = self._Reads(None, tag)
        reader = self._reader(reads)
        reader.force_read("lane1")
        # The forced re-read forgets the last UID, so the same spool reports.
        assert reader._last_uid == {0: None}
        assert reader._gcode.scripts == ["FILAMENT_DT_UPDATE CHANNEL=0"]
        assert reads.calls == [0, 0]
        assert len(reader.reactor.pauses) == 1
        assert reader._check_channel.calls == [(("lane1", 0), {"info": tag})]
        assert reader.logger.messages == []

    def test_timeout_falls_back_to_final_check(self):
        reads = self._Reads(None, {"CARD_UID": 0})
        reader = self._reader(reads)
        reader.force_read("lane1")
        assert reads.calls == [0, 0]
        assert len(reader.reactor.pauses) == 2
        assert reader._check_channel.calls == [(("lane1", 0), {})]
        assert reader.logger.messages == []


class TestAFCU1RFIDGetStatus:
    def test_shape_when_empty(self):
        reader = make_u1_reader()
        assert reader.get_status() == {"lane_channel_map": {},
                                       "scanner_channels": [], "last_reads": {}}
        assert reader.logger.messages == []

    def test_last_reads_included_and_copied(self):
        reader = make_u1_reader()
        reader._tag_reads["lane1"] = {"material": "PLA"}
        status = reader.get_status(100.0)
        assert status["last_reads"] == {"lane1": {"material": "PLA"}}
        status["last_reads"].clear()
        assert reader._tag_reads == {"lane1": {"material": "PLA"}}
        assert reader.logger.messages == []

    def test_reports_wiring_and_reads(self):
        reader = make_u1_reader()
        reader.register_lane(U1Lane("lane1"), 1)
        reader._scanner_channels = {2, 0}
        reader._tag_reads = {"lane1": {"material": "PLA"}}
        status = reader.get_status()
        assert status == {
            "lane_channel_map": {"lane1": 1},
            "scanner_channels": [0, 2],
            "last_reads": {"lane1": {"material": "PLA"}},
        }
        status["lane_channel_map"]["lane2"] = 2
        assert reader._lane_channel_map == {"lane1": 1}
        assert reader.logger.messages == []

    def test_missing_tag_reads_defaults_empty(self):
        reader = make_u1_reader()
        del reader._tag_reads
        assert reader.get_status()["last_reads"] == {}
        reader._tag_reads = None
        assert reader.get_status()["last_reads"] == {}
        assert reader.logger.messages == []


class TestLoadConfig:
    def test_returns_reader_instance(self):
        printer = U1Printer()
        webhooks = U1Webhooks()
        printer.add_object("webhooks", webhooks)
        reader = load_config(U1Config("AFC_U1_rfid", printer,
                                      {"lane_channels": "lane4:1"}))
        assert isinstance(reader, AFC_U1_RFID)
        assert reader.printer is printer
        assert reader._cfg_channels == {"lane4": 1}
        assert webhooks.endpoints == [("afc/u1_rfid", reader._handle_webhook_scan)]
        assert reader.logger.messages == []
