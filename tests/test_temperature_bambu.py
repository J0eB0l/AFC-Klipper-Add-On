"""Unit tests for extras/temperature_bambu.py."""

from __future__ import annotations

import configparser
import logging
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import pytest

from extras.temperature_bambu import load_config, TemperatureBambu
from tests.bambu_helpers import BambuConfig, BambuPrinter, FakeMcu, FakeReactor, FakeTimer, Recorder


class BambuTempMcu(FakeMcu):
    """The primary mcu, its print time 250 s ahead of the reactor clock."""

    def __init__(self) -> None:
        """Start with no estimate asked for."""
        #: Every eventtime an estimate was asked for, in order.
        self.asked: List[float] = []

    def estimated_print_time(self, eventtime: float) -> float:
        """
        Record the estimate request and return a print time 250 s ahead.

        :param eventtime: reactor time; recorded in ``asked``
        :return float: ``eventtime`` plus 250 s
        """
        self.asked.append(eventtime)
        return eventtime + 250.0


class BambuTempReactor(FakeReactor):
    """FakeReactor recording every ``update_timer`` call in ``updates``."""

    def __init__(self, now: float) -> None:
        """
        Start the clock at ``now`` with no reschedule recorded.

        :param now: the starting time
        """
        super().__init__(now=now)
        #: Every (timer, waketime) passed to update_timer, in order.
        self.updates: List[Tuple[FakeTimer, float]] = []

    def update_timer(self, timer: FakeTimer, waketime: float) -> None:
        """
        Record the reschedule, then apply it.

        :param timer: the timer to reschedule
        :param waketime: the timer's next wake
        """
        self.updates.append((timer, waketime))
        super().update_timer(timer, waketime)


class BambuTempPrinter(BambuPrinter):
    """
    BambuPrinter plus what a Klipper temperature sensor uses beyond it:
    ``invoke_shutdown`` (recorded in ``shutdowns``), a record of every
    ``add_object`` attempt (``adds``, refused duplicates included) and of
    every ``lookup_object`` name (``lookups``), a :class:`BambuTempMcu` and
    a :class:`BambuTempReactor` on the same clock.
    ``lookup_object`` is klippy's: a registered object, else the default,
    else a config error.
    """

    def __init__(self) -> None:
        """Build the printer with every record empty."""
        self.adds: List[str] = []
        self.lookups: List[str] = []
        self.shutdowns: List[str] = []
        super().__init__()
        self.mcu = BambuTempMcu()
        self._objects["mcu"] = self.mcu
        # Swap in the recording reactor wherever BambuPrinter put its own.
        reactor = BambuTempReactor(now=self.reactor.now)
        self.reactor = self._reactor = self.afc.reactor = reactor

    def add_object(self, name: str, obj: Any) -> None:
        """
        Record the registration attempt, then register as klippy does.

        :param name: the object's name; recorded, then refused if taken
        :param obj: the object
        """
        self.adds.append(name)
        super().add_object(name, obj)

    def lookup_object(self, name: str,
                      default: Any = BambuPrinter._NO_DEFAULT) -> Any:
        """
        Record the lookup and resolve it as klippy does.

        :param name: the object's name; recorded in ``lookups``
        :param default: returned when it is missing; without one, a missing
            object raises as klippy's lookup does
        :return Any: the object
        """
        self.lookups.append(name)
        if name in self.objects:
            return self.objects[name]
        if default is BambuPrinter._NO_DEFAULT:
            error_str = f"Unknown config object '{name}'"
            raise configparser.Error(error_str)
        return default

    def invoke_shutdown(self, msg: str,
                        details: Optional[Dict[str, Any]] = None) -> None:
        """
        Record the shutdown instead of stopping the printer.

        :param msg: the shutdown reason; recorded in ``shutdowns``
        :param details: Klipper's extra detail; unused
        """
        self.shutdowns.append(msg)


class BambuTempConfig(BambuConfig):
    """BambuConfig whose getint refuses a value below ``minval``, as Klipper's."""

    def getint(self, option: str, *args: Any, minval: Optional[int] = None,
               **kwargs: Any) -> Any:
        """
        Read an int option, refusing one below ``minval``.

        :param option: the option
        :param minval: the smallest value allowed
        :return Any: the value, or the default when it is not set
        """
        value = super().getint(option, *args, **kwargs)
        if (minval is not None
            and value is not None
            and value < minval):
            error_str = (f"Option '{option}' in section '{self.get_name()}' "
                         f"must have minimum of {minval}")
            raise configparser.Error(error_str)
        return value


class BambuTempUnit:
    """
    The AFC_BambuAMS unit a sensor reads: ``get_status`` (its "humidity"
    and "temperature") and ``has_heater``, as afcBambuAMS has them.
    """

    def __init__(self, status: Optional[Dict[str, Any]] = None, *,
                 has_heater: Optional[Union[bool, int]] = None,
                 error: Optional[Exception] = None) -> None:
        """
        Hold the status to report and, optionally, has_heater.

        :param status: what get_status returns
        :param has_heater: the unit's has_heater; None leaves the attribute
            off, as on a unit object without one
        :param error: raised by get_status instead of returning ``status``
        """
        self.status = status
        self.error = error
        #: Every eventtime get_status was called with, in order.
        self.eventtimes: List[float] = []
        if has_heater is not None:
            self.has_heater = has_heater

    def get_status(self, eventtime: float) -> Optional[Dict[str, Any]]:
        """
        Record the call, then return ``status`` or raise ``error``.

        :param eventtime: the sample's eventtime; recorded in ``eventtimes``
        :return Optional[dict]: ``status``
        """
        self.eventtimes.append(eventtime)
        if self.error is not None:
            raise self.error
        return self.status


def build_temperature_bambu(options: Optional[Dict[str, Any]] = None, *,
                            printer: Optional[BambuTempPrinter] = None,
                            units: Optional[Dict[str, BambuTempUnit]] = None
                            ) -> TemperatureBambu:
    """
    Build ``[temperature_sensor Bambu_AMS_1]`` through the real __init__.

    :param options: the section's options
    :param printer: the printer to build on; a new BambuTempPrinter when None
    :param units: AFC_BambuAMS units registered first, keyed by unit name
    :return TemperatureBambu: the sensor; ``sensor.printer`` is the printer
    """
    if printer is None:
        printer = BambuTempPrinter()
    for name, unit in (units or {}).items():
        # klippy registers a unit under its section name.
        printer.objects[f"AFC_BambuAMS {name}"] = unit
    config = BambuTempConfig("temperature_sensor Bambu_AMS_1", printer, options)
    return TemperatureBambu(config)


@pytest.fixture
def bambu_temp_log(caplog: pytest.LogCaptureFixture) -> pytest.LogCaptureFixture:
    """
    caplog, recording the module's "temperature_bambu" logger from DEBUG up.

    :param caplog: pytest's caplog fixture
    :return pytest.LogCaptureFixture: read its ``record_tuples``
    """
    caplog.set_level(logging.DEBUG, logger="temperature_bambu")
    return caplog


class TestTemperatureBambuInit:
    def test_defaults(self, bambu_temp_log):
        printer = BambuTempPrinter()
        sensor = build_temperature_bambu(printer=printer)
        assert sensor.printer is printer
        assert sensor.reactor is printer.reactor
        assert sensor.name == "Bambu_AMS_1"
        assert sensor.unit_name == "Bambu_AMS_1"
        assert sensor.report_time == 5
        assert sensor.temp == 0.0
        assert sensor.min_temp == 0.0
        assert sensor.max_temp == 90.0
        assert sensor.humidity == 0.0
        assert sensor._has_temp is False
        assert sensor._temp_capable is None
        assert sensor._temp_pinned is False
        assert sensor._unit is None
        assert sensor._callback is None
        assert sensor._warned is False
        assert sensor.simulate_aht is True
        assert bambu_temp_log.record_tuples == []

    def test_bambu_unit_overrides_the_unit_name(self, bambu_temp_log):
        sensor = build_temperature_bambu({"bambu_unit": "BambuAMS_2"})
        assert sensor.unit_name == "BambuAMS_2"
        assert sensor.name == "Bambu_AMS_1"
        assert bambu_temp_log.record_tuples == []

    def test_report_time_is_read(self, bambu_temp_log):
        assert build_temperature_bambu({"report_time": 7}).report_time == 7
        assert bambu_temp_log.record_tuples == []

    def test_report_time_of_one_is_accepted(self, bambu_temp_log):
        assert build_temperature_bambu({"report_time": 1}).report_time == 1
        assert bambu_temp_log.record_tuples == []

    def test_report_time_below_one_is_rejected(self, bambu_temp_log):
        printer = BambuTempPrinter()
        with pytest.raises(configparser.Error) as raised:
            build_temperature_bambu({"report_time": 0}, printer=printer)
        assert str(raised.value) == (
            "Option 'report_time' in section 'temperature_sensor Bambu_AMS_1' "
            "must have minimum of 1")
        # Refused before anything was registered.
        assert printer.adds == []
        assert printer.reactor.timers == []
        assert bambu_temp_log.record_tuples == []

    @pytest.mark.parametrize("pinned", [True, False])
    def test_report_temperature_pins_capability(self, pinned, bambu_temp_log):
        sensor = build_temperature_bambu({"report_temperature": pinned})
        assert sensor._temp_capable is pinned
        assert sensor._temp_pinned is True
        assert bambu_temp_log.record_tuples == []

    def test_no_sensor_type_registers_only_aht10(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        assert sensor.printer.adds == ["aht10 Bambu_AMS_1"]
        assert sensor.printer.objects["aht10 Bambu_AMS_1"] is sensor
        assert bambu_temp_log.record_tuples == []

    @pytest.mark.parametrize("stype", ["", "   "])
    def test_blank_sensor_type_registers_only_aht10(self, stype, bambu_temp_log):
        sensor = build_temperature_bambu({"sensor_type": stype})
        # A blank type is not even tried (it would add " Bambu_AMS_1").
        assert sensor.printer.adds == ["aht10 Bambu_AMS_1"]
        assert bambu_temp_log.record_tuples == []

    @pytest.mark.parametrize("stype", ["aht10", "AHT10", " Aht10 "])
    def test_aht10_sensor_type_is_not_registered_twice(self, stype, bambu_temp_log):
        sensor = build_temperature_bambu({"sensor_type": stype})
        # Not tried a second time, not even as a refused duplicate.
        assert sensor.printer.adds == ["aht10 Bambu_AMS_1"]
        assert sensor.printer.objects["aht10 Bambu_AMS_1"] is sensor
        assert bambu_temp_log.record_tuples == []

    @pytest.mark.parametrize("stype, key", [
        ("aht4x", "aht4x Bambu_AMS_1"),
        (" sht3x ", "sht3x Bambu_AMS_1"),
        ("temperature_bambu", "temperature_bambu Bambu_AMS_1"),
    ])
    def test_other_sensor_type_is_also_registered(self, stype, key, bambu_temp_log):
        sensor = build_temperature_bambu({"sensor_type": stype})
        assert sensor.printer.adds == ["aht10 Bambu_AMS_1", key]
        assert sensor.printer.objects["aht10 Bambu_AMS_1"] is sensor
        assert sensor.printer.objects[key] is sensor
        assert bambu_temp_log.record_tuples == []

    def test_sensor_type_name_already_taken_is_skipped(self, bambu_temp_log):
        printer = BambuTempPrinter()
        real = object()
        printer.objects["sht3x Bambu_AMS_1"] = real
        sensor = build_temperature_bambu({"sensor_type": "sht3x"}, printer=printer)
        # Tried, refused as a duplicate, and the real sensor keeps the name.
        assert printer.adds == ["aht10 Bambu_AMS_1", "sht3x Bambu_AMS_1"]
        assert printer.objects["sht3x Bambu_AMS_1"] is real
        assert printer.objects["aht10 Bambu_AMS_1"] is sensor
        # Construction went on past the refusal.
        assert printer.reactor.timers == [sensor.sample_timer]
        assert printer._event_handlers == {"klippy:ready": [sensor._handle_ready]}
        assert bambu_temp_log.record_tuples == []

    def test_no_simulation_registers_under_temperature_bambu(self, bambu_temp_log):
        sensor = build_temperature_bambu({"simulate_supported_sensor_mainsail": False,
                                          "sensor_type": "aht4x"})
        assert sensor.simulate_aht is False
        assert sensor.printer.adds == ["temperature_bambu Bambu_AMS_1"]
        assert sensor.printer.objects["temperature_bambu Bambu_AMS_1"] is sensor
        assert bambu_temp_log.record_tuples == []

    def test_registers_sample_timer_and_ready_handler(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        reactor = sensor.printer.reactor
        assert reactor.timers == [sensor.sample_timer]
        assert sensor.sample_timer.callback == sensor._sample
        # Registered idle and never rescheduled: nothing samples before klippy:ready.
        assert sensor.sample_timer.waketime == reactor.NEVER
        assert reactor.updates == []
        assert sensor.printer._event_handlers == {"klippy:ready": [sensor._handle_ready]}
        assert bambu_temp_log.record_tuples == []


class TestTemperatureBambuHandleReady:
    def test_ready_starts_the_sample_timer_now(self, bambu_temp_log):
        unit = BambuTempUnit({"humidity": 30.0})
        sensor = build_temperature_bambu(units={"Bambu_AMS_1": unit})
        assert sensor.sample_timer.waketime == sensor.printer.reactor.NEVER
        sensor.printer.send_event("klippy:ready")
        # One reschedule, of the sample timer, to reactor.NOW: due at once.
        assert sensor.printer.reactor.updates == [(sensor.sample_timer, 0.0)]
        assert sensor.sample_timer.waketime == 0.0
        # Only scheduled: the first sample is the reactor's to run.
        assert unit.eventtimes == []
        assert sensor.printer.lookups == []
        assert bambu_temp_log.record_tuples == []


class TestTemperatureBambuSetupMinmax:
    def test_stores_the_range(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        sensor.setup_minmax(-10.0, 120.0)
        assert (sensor.min_temp, sensor.max_temp) == (-10.0, 120.0)
        assert bambu_temp_log.record_tuples == []


class TestTemperatureBambuSetupCallback:
    def test_stores_the_callback(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        callback = Recorder()
        sensor.setup_callback(callback)
        assert sensor._callback is callback
        assert callback.calls == []
        assert bambu_temp_log.record_tuples == []


class TestTemperatureBambuGetReportTimeDelta:
    def test_default(self, bambu_temp_log):
        assert build_temperature_bambu().get_report_time_delta() == 5
        assert bambu_temp_log.record_tuples == []

    def test_configured(self, bambu_temp_log):
        sensor = build_temperature_bambu({"report_time": 9})
        assert sensor.get_report_time_delta() == 9
        assert bambu_temp_log.record_tuples == []


class TestTemperatureBambuResolveUnit:
    def test_found_unit_is_returned_without_a_warning(self, bambu_temp_log):
        unit = BambuTempUnit()
        sensor = build_temperature_bambu(units={"Bambu_AMS_1": unit})
        assert sensor._resolve_unit() is unit
        assert sensor.printer.lookups == ["AFC_BambuAMS Bambu_AMS_1"]
        assert sensor._warned is False
        # Only looked up: holding it is _sample's job.
        assert sensor._unit is None
        assert bambu_temp_log.record_tuples == []

    def test_bambu_unit_names_the_lookup(self, bambu_temp_log):
        mine, decoy = BambuTempUnit(), BambuTempUnit()
        sensor = build_temperature_bambu({"bambu_unit": "BambuAMS_2"},
                                         units={"BambuAMS_2": mine, "Bambu_AMS_1": decoy})
        assert sensor._resolve_unit() is mine
        assert sensor.printer.lookups == ["AFC_BambuAMS BambuAMS_2"]
        assert sensor._warned is False
        assert bambu_temp_log.record_tuples == []

    def test_missing_unit_warns(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        assert sensor._resolve_unit() is None
        assert sensor._warned is True
        assert bambu_temp_log.record_tuples == [
            ("temperature_bambu", logging.WARNING,
             "temperature_bambu Bambu_AMS_1: no AFC_BambuAMS unit 'Bambu_AMS_1'"),
        ]

    def test_warning_names_the_configured_unit(self, bambu_temp_log):
        # A unit under the sensor's own name is not the one configured.
        sensor = build_temperature_bambu({"bambu_unit": "BambuAMS_9"},
                                         units={"Bambu_AMS_1": BambuTempUnit()})
        assert sensor._resolve_unit() is None
        assert sensor._warned is True
        assert bambu_temp_log.record_tuples == [
            ("temperature_bambu", logging.WARNING,
             "temperature_bambu Bambu_AMS_1: no AFC_BambuAMS unit 'BambuAMS_9'"),
        ]

    def test_missing_unit_warns_only_once(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        sensor._resolve_unit()
        assert sensor._resolve_unit() is None
        assert sensor._warned is True
        assert sensor.printer.lookups == ["AFC_BambuAMS Bambu_AMS_1",
                                          "AFC_BambuAMS Bambu_AMS_1"]
        assert bambu_temp_log.record_tuples == [
            ("temperature_bambu", logging.WARNING,
             "temperature_bambu Bambu_AMS_1: no AFC_BambuAMS unit 'Bambu_AMS_1'"),
        ]

    def test_unit_appearing_after_the_warning_is_found(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        sensor._resolve_unit()
        unit = BambuTempUnit()
        sensor.printer.objects["AFC_BambuAMS Bambu_AMS_1"] = unit
        assert sensor._resolve_unit() is unit
        assert sensor._warned is True
        assert bambu_temp_log.record_tuples == [
            ("temperature_bambu", logging.WARNING,
             "temperature_bambu Bambu_AMS_1: no AFC_BambuAMS unit 'Bambu_AMS_1'"),
        ]


class TestTemperatureBambuSample:
    @staticmethod
    def _state(sensor: TemperatureBambu) -> Dict[str, Any]:
        """
        Collect the attributes _sample sets, for one comparison.

        :param sensor: the sensor
        :return dict: every attribute _sample sets
        """
        return {"unit": sensor._unit, "capable": sensor._temp_capable,
                "humidity": sensor.humidity, "temp": sensor.temp,
                "has_temp": sensor._has_temp}

    @staticmethod
    def _with_unit(unit: BambuTempUnit,
                   options: Optional[Dict[str, Any]] = None) -> TemperatureBambu:
        """
        Build the sensor with ``unit`` registered under its name.

        :param unit: the AFC_BambuAMS unit registered under the sensor's name
        :param options: the sensor section's options
        :return TemperatureBambu: the sensor
        """
        return build_temperature_bambu(options, units={"Bambu_AMS_1": unit})

    def test_no_unit_and_no_callback(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        # The reactor's time (100) plus report_time (5), whatever the eventtime.
        assert sensor._sample(12.5) == 105.0
        assert self._state(sensor) == {"unit": None, "capable": None, "humidity": 0.0,
                                       "temp": 0.0, "has_temp": False}
        # The unit was looked up; with no callback the mcu was never asked.
        assert sensor.printer.lookups == ["AFC_BambuAMS Bambu_AMS_1"]
        assert sensor.printer.mcu.asked == []
        assert sensor.printer.shutdowns == []
        assert bambu_temp_log.record_tuples == [
            ("temperature_bambu", logging.WARNING,
             "temperature_bambu Bambu_AMS_1: no AFC_BambuAMS unit 'Bambu_AMS_1'"),
        ]

    def test_resolves_and_keeps_the_unit(self, bambu_temp_log):
        unit = BambuTempUnit({"humidity": 30.0}, has_heater=True)
        sensor = self._with_unit(unit)
        sensor._sample(1.0)
        sensor._sample(2.0)
        assert self._state(sensor) == {"unit": unit, "capable": True, "humidity": 30.0,
                                       "temp": 0.0, "has_temp": False}
        assert sensor.printer.lookups == ["AFC_BambuAMS Bambu_AMS_1"]
        assert unit.eventtimes == [1.0, 2.0]
        assert bambu_temp_log.record_tuples == []

    def test_held_unit_is_not_looked_up_again(self, bambu_temp_log):
        held = BambuTempUnit({"humidity": 20.0}, has_heater=False)
        registered = BambuTempUnit({"humidity": 70.0}, has_heater=True)
        sensor = self._with_unit(registered)
        sensor._unit = held
        sensor._sample(3.0)
        assert sensor.printer.lookups == []
        assert self._state(sensor) == {"unit": held, "capable": False, "humidity": 20.0,
                                       "temp": 0.0, "has_temp": False}
        assert held.eventtimes == [3.0]
        assert registered.eventtimes == []
        assert bambu_temp_log.record_tuples == []

    @pytest.mark.parametrize("has_heater, expected", [
        (False, False), (True, True), (0, False), (1, True),
    ])
    def test_capability_follows_has_heater(self, has_heater, expected, bambu_temp_log):
        unit = BambuTempUnit({}, has_heater=has_heater)
        sensor = self._with_unit(unit)
        sensor._sample(0.0)
        # "is": stored as a bool, whatever type the unit holds.
        assert sensor._temp_capable is expected
        assert sensor._unit is unit
        assert bambu_temp_log.record_tuples == []

    def test_unit_without_has_heater_counts_as_capable(self, bambu_temp_log):
        unit = BambuTempUnit({})
        assert not hasattr(unit, "has_heater")
        sensor = self._with_unit(unit)
        sensor._sample(0.0)
        assert sensor._temp_capable is True
        assert bambu_temp_log.record_tuples == []

    @pytest.mark.parametrize("pinned", [True, False])
    def test_pinned_capability_ignores_has_heater(self, pinned, bambu_temp_log):
        sensor = self._with_unit(BambuTempUnit({}, has_heater=not pinned),
                                 {"report_temperature": pinned})
        sensor._sample(0.0)
        assert sensor._temp_capable is pinned
        assert bambu_temp_log.record_tuples == []

    def test_none_status_changes_nothing(self, bambu_temp_log):
        unit = BambuTempUnit(None, has_heater=False)
        sensor = self._with_unit(unit)
        sensor.humidity, sensor.temp = 12.0, 33.0
        sensor._sample(4.0)
        assert self._state(sensor) == {"unit": unit, "capable": False, "humidity": 12.0,
                                       "temp": 33.0, "has_temp": False}
        assert unit.eventtimes == [4.0]
        assert bambu_temp_log.record_tuples == []

    def test_humidity_only(self, bambu_temp_log):
        unit = BambuTempUnit({"humidity": "45.5"})
        sensor = self._with_unit(unit)
        sensor._sample(0.0)
        assert self._state(sensor) == {"unit": unit, "capable": True, "humidity": 45.5,
                                       "temp": 0.0, "has_temp": False}
        assert isinstance(sensor.humidity, float)
        assert sensor.printer.shutdowns == []
        assert bambu_temp_log.record_tuples == []

    def test_temperature_without_humidity(self, bambu_temp_log):
        unit = BambuTempUnit({"humidity": None, "temperature": "41"})
        sensor = self._with_unit(unit)
        sensor.humidity = 12.0
        sensor._sample(0.0)
        assert self._state(sensor) == {"unit": unit, "capable": True, "humidity": 12.0,
                                       "temp": 41.0, "has_temp": True}
        assert isinstance(sensor.temp, float)
        assert sensor.printer.shutdowns == []
        assert bambu_temp_log.record_tuples == []

    def test_zero_readings_are_kept(self, bambu_temp_log):
        # Only a missing value is skipped: 0 is a reading like any other.
        unit = BambuTempUnit({"humidity": 0, "temperature": 0})
        sensor = self._with_unit(unit)
        sensor.humidity, sensor.temp = 12.0, 33.0
        sensor._sample(0.0)
        assert self._state(sensor) == {"unit": unit, "capable": True, "humidity": 0.0,
                                       "temp": 0.0, "has_temp": True}
        assert sensor.printer.shutdowns == []
        assert bambu_temp_log.record_tuples == []

    @pytest.mark.parametrize("temp", [10.0, 35.0, 60.0])
    def test_reading_inside_the_range_is_kept(self, temp, bambu_temp_log):
        sensor = self._with_unit(BambuTempUnit({"temperature": temp}))
        sensor.setup_minmax(10.0, 60.0)
        sensor._sample(0.0)
        assert (sensor.temp, sensor._has_temp) == (temp, True)
        assert sensor.printer.shutdowns == []
        assert bambu_temp_log.record_tuples == []

    def test_reading_below_min_shuts_down(self, bambu_temp_log):
        sensor = self._with_unit(BambuTempUnit({"temperature": 9.4}))
        sensor.setup_minmax(10.0, 60.0)
        sensor._sample(0.0)
        assert (sensor.temp, sensor._has_temp) == (9.4, True)
        assert sensor.printer.shutdowns == [
            "temperature_bambu Bambu_AMS_1: 9.4 outside range 10.0:60.0"]
        assert bambu_temp_log.record_tuples == []

    def test_reading_above_max_shuts_down(self, bambu_temp_log):
        sensor = self._with_unit(BambuTempUnit({"temperature": 61.37}))
        sensor.setup_minmax(10.0, 60.0)
        sensor._sample(0.0)
        assert (sensor.temp, sensor._has_temp) == (61.37, True)
        assert sensor.printer.shutdowns == [
            "temperature_bambu Bambu_AMS_1: 61.4 outside range 10.0:60.0"]
        assert bambu_temp_log.record_tuples == []

    def test_status_failure_is_logged_and_sampling_continues(self, bambu_temp_log):
        unit = BambuTempUnit(has_heater=False, error=RuntimeError("bridge gone"))
        sensor = self._with_unit(unit)
        callback = Recorder()
        sensor.setup_callback(callback)
        sensor.setup_minmax(5.0, 60.0)
        assert sensor._sample(7.0) == 105.0
        # has_heater was read before the status read failed.
        assert self._state(sensor) == {"unit": unit, "capable": False, "humidity": 0.0,
                                       "temp": 0.0, "has_temp": False}
        assert unit.eventtimes == [7.0]
        # The mcu's print time for reactor time 100, and min_temp for no reading.
        assert callback.calls == [((350.0, 5.0), {})]
        assert bambu_temp_log.record_tuples == [
            ("temperature_bambu", logging.DEBUG,
             "temperature_bambu Bambu_AMS_1: sample failed: bridge gone"),
        ]

    def test_bad_temperature_keeps_humidity_and_drops_the_reading(self, bambu_temp_log):
        unit = BambuTempUnit({"humidity": "45.5", "temperature": "hot"})
        sensor = self._with_unit(unit)
        sensor._sample(0.0)
        assert self._state(sensor) == {"unit": unit, "capable": True, "humidity": 45.5,
                                       "temp": 0.0, "has_temp": False}
        assert sensor.printer.shutdowns == []
        assert bambu_temp_log.record_tuples == [
            ("temperature_bambu", logging.DEBUG,
             "temperature_bambu Bambu_AMS_1: sample failed: "
             "could not convert string to float: 'hot'"),
        ]

    def test_callback_gets_min_temp_before_any_reading(self, bambu_temp_log):
        sensor = self._with_unit(BambuTempUnit({"humidity": 30.0}))
        callback = Recorder()
        sensor.setup_callback(callback)
        sensor.setup_minmax(5.0, 60.0)
        sensor._sample(0.0)
        # Print time is the reactor's 100 plus the mcu's 250 offset.
        assert callback.calls == [((350.0, 5.0), {})]
        assert sensor.printer.mcu.asked == [100.0]
        assert bambu_temp_log.record_tuples == []

    def test_callback_gets_the_reading(self, bambu_temp_log):
        sensor = self._with_unit(BambuTempUnit({"temperature": 41.0}))
        callback = Recorder()
        sensor.setup_callback(callback)
        sensor.setup_minmax(5.0, 60.0)
        sensor._sample(0.0)
        assert callback.calls == [((350.0, 41.0), {})]
        assert sensor.printer.mcu.asked == [100.0]
        assert bambu_temp_log.record_tuples == []

    def test_callback_keeps_the_last_reading_when_it_stops(self, bambu_temp_log):
        unit = BambuTempUnit({"temperature": 41.0})
        sensor = self._with_unit(unit)
        callback = Recorder()
        sensor.setup_callback(callback)
        sensor.setup_minmax(5.0, 60.0)
        sensor._sample(0.0)
        unit.status = {"humidity": 30.0}
        sensor.printer.reactor.advance(1.0)
        sensor._sample(1.0)
        assert callback.calls == [((350.0, 41.0), {}), ((351.0, 41.0), {})]
        assert sensor.printer.mcu.asked == [100.0, 101.0]
        assert self._state(sensor) == {"unit": unit, "capable": True, "humidity": 30.0,
                                       "temp": 41.0, "has_temp": True}
        assert bambu_temp_log.record_tuples == []

    def test_next_wake_uses_report_time(self, bambu_temp_log):
        sensor = self._with_unit(BambuTempUnit({}), {"report_time": 7})
        sensor.printer.reactor.now = 250.0
        assert sensor._sample(0.0) == 257.0
        assert bambu_temp_log.record_tuples == []


class TestTemperatureBambuGetStatus:
    def test_a_live_ams2_shows_its_chamber_temperature(self, bambu_temp_log):
        unit = BambuTempUnit({"humidity": 30.0, "temperature": 41.0}, has_heater=False)
        sensor = build_temperature_bambu(units={"Bambu_AMS_1": unit})
        sensor._sample(0.0)
        assert sensor.get_status(0.0) == {"humidity": 30.0}
        # AFC_BridgeBox confirms the unit an ams2 and gives it its dryer live.
        unit.has_heater = True
        sensor._sample(1.0)
        assert sensor.get_status(1.0) == {"humidity": 30.0, "temperature": 41.0}
        assert bambu_temp_log.record_tuples == []

    def test_report_temperature_still_pins_it(self, bambu_temp_log):
        unit = BambuTempUnit({"humidity": 30.0, "temperature": 41.0}, has_heater=True)
        sensor = build_temperature_bambu({"report_temperature": False},
                                         units={"Bambu_AMS_1": unit})
        sensor._sample(0.0)
        assert sensor.get_status(0.0) == {"humidity": 30.0}
        assert bambu_temp_log.record_tuples == []

    @pytest.mark.parametrize("capable", [None, True])
    def test_temperature_reported_unless_incapable(self, capable, bambu_temp_log):
        sensor = build_temperature_bambu()
        sensor._temp_capable = capable
        sensor.humidity, sensor.temp = 45.26, 41.06
        assert sensor.get_status(0.0) == {"humidity": 45.3, "temperature": 41.1}
        assert bambu_temp_log.record_tuples == []

    def test_incapable_unit_omits_temperature(self, bambu_temp_log):
        sensor = build_temperature_bambu()
        sensor._temp_capable = False
        sensor.humidity, sensor.temp = 45.26, 41.06
        assert sensor.get_status(0.0) == {"humidity": 45.3}
        assert bambu_temp_log.record_tuples == []


class TestLoadConfig:
    class _Heaters:
        """Klipper's heaters object, recording add_sensor_factory calls."""

        def __init__(self) -> None:
            """Start with no factory registered."""
            self.factories: List[Tuple[str, Callable[..., Any]]] = []

        def add_sensor_factory(self, sensor_type: str,
                               sensor_factory: Callable[..., Any]) -> None:
            """
            Record the factory registration.

            :param sensor_type: the sensor_type the factory builds
            :param sensor_factory: called with a sensor section's config
            """
            self.factories.append((sensor_type, sensor_factory))

    def test_registers_both_sensor_factories(self, bambu_temp_log):
        printer = BambuTempPrinter()
        heaters = self._Heaters()
        printer.objects["heaters"] = heaters
        config = BambuTempConfig("temperature_bambu", printer)
        assert load_config(config) is None
        assert heaters.factories == [
            ("temperature_bambu", TemperatureBambu),
            ("aht4x", TemperatureBambu),
        ]
        assert printer.lookups == ["heaters"]
        assert bambu_temp_log.record_tuples == []

    def test_missing_heaters_is_a_config_error(self, bambu_temp_log):
        printer = BambuTempPrinter()
        config = BambuTempConfig("temperature_bambu", printer)
        with pytest.raises(configparser.Error) as raised:
            load_config(config)
        # Looked up as required, so klippy names what is missing.
        assert str(raised.value) == "Unknown config object 'heaters'"
        assert bambu_temp_log.record_tuples == []
