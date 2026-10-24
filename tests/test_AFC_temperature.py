"""Tests for extras/AFC_temperature.py: a unit's temperature card, its humidity
object, the registration with the printer and heaters, and the HDC1080."""

from __future__ import annotations

import configparser
import types
from typing import Any, Dict, List, Optional, Tuple

import pytest

import extras.AFC_temperature as temperature_module
from extras.AFC_temperature import (
    CONF_REG, DVID_REG, HDC1080, HDC1080_MAX_ERRORS, HEATER_ENABLE_BIT, HUMI_REG,
    HUMI_RES, HumidityView, MFID_REG, REPORT_TIME, SENSOR_TYPE, TEMP_REG, TEMP_RES,
    UnitTemperatureCard, _register_sensor, _resolution, add_card)
from tests.ace_helpers import AceConfig, AcePrinter, Recorder, make_ace_printer


# ── fakes ─────────────────────────────────────────────────────────────────────

class FakeReader:
    """A unit's reader: answers from a list, one per call, or raises."""

    def __init__(self, *readings: Any) -> None:
        """
        :param readings: (temperature, humidity) tuples, or an exception to raise
        """
        self.readings = list(readings)
        self.calls: List[float] = []

    def __call__(self, eventtime: float) -> Tuple[Optional[float], Optional[float]]:
        """
        :param eventtime: the card's sample time
        :return tuple: the next reading
        """
        self.calls.append(eventtime)
        reading = self.readings.pop(0)
        if isinstance(reading, Exception):
            raise reading
        return reading


class FakeI2C:
    """bus.MCU_I2C stand-in: records writes, answers reads from a list."""

    def __init__(self, *responses: Any) -> None:
        """
        :param responses: 16-bit values to answer reads with, in order, or an
            exception to raise on that read
        """
        self.responses = list(responses)
        self.writes: List[List[int]] = []
        self.reads: List[Tuple[List[int], int]] = []

    def i2c_write(self, data: List[int]) -> None:
        """:param data: the bytes written"""
        self.writes.append(list(data))

    def i2c_read(self, data: List[int], count: int) -> Dict[str, bytes]:
        """
        :param data: bytes written first (none here)
        :param count: bytes wanted
        :return dict: Klipper's reply shape, the value big-endian
        """
        self.reads.append((list(data), count))
        value = self.responses.pop(0)
        if isinstance(value, Exception):
            raise value
        return {"response": bytes([value >> 8, value & 0xFF])}


class _TrackingWrapper:
    """configfile.ConfigWrapper as Klipper's records reads: every option read
    lands in the shared access_tracking dict, which configfile.settings is
    built from."""

    def __init__(self, printer: Any, fileconfig: configparser.RawConfigParser,
                 access_tracking: Dict[Tuple[str, str], Any], section: str) -> None:
        self.printer = printer
        self.fileconfig = fileconfig
        self.access_tracking = access_tracking
        self.section = section

    def get_name(self) -> str:
        """:return str: the section name"""
        return self.section

    def get(self, option: str, default: Any = None) -> Any:
        """:return Any: the option's value, recorded"""
        if self.fileconfig.has_option(self.section, option):
            value = self.fileconfig.get(self.section, option)
        else:
            value = default
        self.access_tracking[(self.section.lower(), option.lower())] = value
        return value


@pytest.fixture
def tracking_configfile(monkeypatch: pytest.MonkeyPatch) -> None:
    """Give the module a configfile whose wrapper records reads."""
    module = types.ModuleType("configfile")
    module.ConfigWrapper = _TrackingWrapper  # type: ignore[attr-defined]
    monkeypatch.setattr(temperature_module, "configfile", module)


def _config(printer: Optional[AcePrinter] = None, name: str = "AFC_ACE Ace_1",
            values: Optional[Dict[str, Any]] = None) -> AceConfig:
    """
    :param printer: the printer, a new one when None
    :param name: the unit section's name
    :param values: its options
    :return AceConfig: the unit's section wrapper
    """
    return AceConfig(name, printer or make_ace_printer(), values)


def _card(config: Optional[AceConfig] = None, reader: Any = None,
          name: str = "Ace_1", **kwargs: Any) -> UnitTemperatureCard:
    """
    :param config: the unit's section wrapper, a new one when None
    :param reader: the card's reader, an empty FakeReader when None
    :param name: the card's name
    :param kwargs: further UnitTemperatureCard arguments
    :return UnitTemperatureCard: the card
    """
    config = config or _config()
    return UnitTemperatureCard(config, name, reader or FakeReader(),
                               config.get_printer().logger, **kwargs)


def _hdc1080(monkeypatch: pytest.MonkeyPatch, i2c: Optional[FakeI2C] = None,
             values: Optional[Dict[str, Any]] = None,
             config: Optional[AceConfig] = None) -> HDC1080:
    """
    :param monkeypatch: pytest's monkeypatch fixture
    :param i2c: the bus the sensor reads, an empty FakeI2C when None
    :param values: the unit section's options
    :param config: the unit's section wrapper, a new one when None
    :return HDC1080: the sensor
    """
    config = config or _config(name="AFC_OpenAMS AMS_1", values=values)
    monkeypatch.setattr(temperature_module.bus, "MCU_I2C_from_config",
                        Recorder(result=i2c or FakeI2C()))
    return HDC1080(config, config.get_printer().logger)


# ── UnitTemperatureCard ───────────────────────────────────────────────────────

class TestUnitTemperatureCardInit:
    def test_starts_with_no_readings_and_the_timer_armed_at_ready(self):
        config = _config()
        printer = config.get_printer()
        reader = FakeReader()

        card = UnitTemperatureCard(config, "Ace_1", reader, printer.logger)

        assert (card.printer, card.reactor, card.name, card.reader, card.logger,
                card.report_time) == (printer, printer.reactor, "Ace_1", reader,
                                      printer.logger, REPORT_TIME)
        assert (card.temperature, card.humidity, card.measured_min,
                card.measured_max, card._read_failed) == (None, None, None, None, False)
        assert card.humidity_view.card is card
        assert printer.reactor.timers == [card.sample_timer]
        assert card.sample_timer.callback == card._sample
        assert card.sample_timer.waketime == printer.reactor.NEVER
        assert printer.event_handlers == {"klippy:ready": [card._handle_ready]}

    def test_report_time_is_taken(self):
        card = _card(report_time=2.5)
        assert card.report_time == 2.5


class TestUnitTemperatureCardHandleReady:
    def test_arms_the_sample_timer_now(self):
        card = _card()
        card._handle_ready()
        assert card.sample_timer.waketime == card.reactor.NOW


class TestUnitTemperatureCardSample:
    def test_first_reading_sets_the_values_and_the_range(self):
        reader = FakeReader((26, 41))
        card = _card(reader=reader)

        assert card._sample(100.0) == 100.0 + REPORT_TIME

        assert reader.calls == [100.0]
        assert (card.temperature, card.humidity) == (26.0, 41.0)
        assert (card.measured_min, card.measured_max) == (26.0, 26.0)
        assert card.printer.logger.messages == []

    def test_a_lower_reading_lowers_the_minimum_only(self):
        card = _card(reader=FakeReader((26.0, None), (24.5, None)))
        card._sample(100.0)
        card._sample(105.0)
        assert (card.temperature, card.measured_min, card.measured_max) == (
            24.5, 24.5, 26.0)

    def test_a_higher_reading_raises_the_maximum_only(self):
        card = _card(reader=FakeReader((26.0, None), (31.0, None)))
        card._sample(100.0)
        card._sample(105.0)
        assert (card.temperature, card.measured_min, card.measured_max) == (
            31.0, 26.0, 31.0)

    def test_no_temperature_keeps_the_last_one_and_its_range(self):
        card = _card(reader=FakeReader((26.0, 40.0), (None, 45.0)))
        card._sample(100.0)
        card._sample(105.0)
        assert (card.temperature, card.measured_min, card.measured_max,
                card.humidity) == (26.0, 26.0, 26.0, 45.0)

    def test_no_humidity_keeps_the_last_one(self):
        card = _card(reader=FakeReader((26.0, 40.0), (27.0, None)))
        card._sample(100.0)
        card._sample(105.0)
        assert (card.temperature, card.humidity) == (27.0, 40.0)

    def test_a_failing_reader_is_logged_once(self):
        card = _card(reader=FakeReader(RuntimeError("no unit"), RuntimeError("no unit")))

        assert card._sample(100.0) == 100.0 + REPORT_TIME
        assert card._sample(105.0) == 105.0 + REPORT_TIME

        assert card._read_failed is True
        assert (card.temperature, card.humidity) == (None, None)
        assert card.printer.logger.messages == [
            ("warning", "AFC temperature Ace_1: read failed (no unit)")]

    def test_a_reading_after_a_failure_lets_the_next_failure_log_again(self):
        card = _card(reader=FakeReader(RuntimeError("a"), (26.0, 40.0),
                                       RuntimeError("b")))
        card._sample(100.0)
        card._sample(105.0)
        assert card._read_failed is False
        card._sample(110.0)
        assert card.printer.logger.messages == [
            ("warning", "AFC temperature Ace_1: read failed (a)"),
            ("warning", "AFC temperature Ace_1: read failed (b)")]


class TestUnitTemperatureCardGetTemp:
    def test_no_reading_yet_is_zero_with_no_target(self):
        assert _card().get_temp(100.0) == (0.0, 0.0)

    def test_the_last_temperature_with_no_target(self):
        card = _card(reader=FakeReader((26.4, None)))
        card._sample(100.0)
        assert card.get_temp(105.0) == (26.4, 0.0)


class TestUnitTemperatureCardGetStatus:
    def test_zeros_before_the_first_reading(self):
        assert _card().get_status(100.0) == {
            "temperature": 0.0, "measured_min_temp": 0.0, "measured_max_temp": 0.0}

    def test_the_readings_rounded_to_two_places(self):
        card = _card(reader=FakeReader((26.456, 40.0), (24.123, None)))
        card._sample(100.0)
        card._sample(105.0)
        assert card.get_status(110.0) == {
            "temperature": 24.12, "measured_min_temp": 24.12,
            "measured_max_temp": 26.46}


# ── HumidityView ──────────────────────────────────────────────────────────────

class TestHumidityViewInit:
    def test_keeps_its_card(self):
        card = _card()
        assert HumidityView(card).card is card


class TestHumidityViewGetStatus:
    def test_temperature_alone_until_a_humidity_is_reported(self):
        card = _card(reader=FakeReader((26.456, None)))
        card._sample(100.0)
        assert card.humidity_view.get_status(105.0) == {"temperature": 26.46}

    def test_zero_temperature_before_any_reading(self):
        assert _card().humidity_view.get_status(100.0) == {"temperature": 0.0}

    def test_humidity_beside_the_temperature_rounded(self):
        card = _card(reader=FakeReader((26.0, 41.456)))
        card._sample(100.0)
        assert card.humidity_view.get_status(105.0) == {
            "temperature": 26.0, "humidity": 41.46}


# ── add_card ──────────────────────────────────────────────────────────────────

class TestAddCard:
    def test_registers_the_card_and_its_humidity_object_and_lists_it(
            self, tracking_configfile):
        config = _config()
        printer = config.get_printer()
        reader = FakeReader()

        card = add_card(config, "Ace_1", reader, printer.logger, report_time=3.0)

        assert isinstance(card, UnitTemperatureCard)
        assert (card.name, card.reader, card.report_time) == ("Ace_1", reader, 3.0)
        assert printer.objects["temperature_sensor Ace_1"] is card
        assert printer.objects["aht10 Ace_1"] is card.humidity_view
        assert printer.heaters.available_sensors == ["temperature_sensor Ace_1"]
        assert config.access_tracking == {
            ("temperature_sensor ace_1", "sensor_type"): SENSOR_TYPE}
        assert printer.logger.messages == []

    def test_a_config_still_carrying_the_sensor_section_is_refused(self):
        config = _config()
        printer = config.get_printer()
        printer.config_sections.append("temperature_sensor Ace_1")

        with pytest.raises(configparser.Error) as exc_info:
            add_card(config, "Ace_1", FakeReader(), printer.logger)

        assert str(exc_info.value) == (
            "[AFC_ACE Ace_1] builds its own temperature card; remove the "
            "[temperature_sensor Ace_1] section")
        assert "temperature_sensor Ace_1" not in printer.objects
        assert printer.logger.messages == []

    def test_a_card_object_already_there_leaves_the_unit_without_one(self):
        config = _config()
        printer = config.get_printer()
        printer.add_object("temperature_sensor Ace_1", object())

        assert add_card(config, "Ace_1", FakeReader(), printer.logger) is None

        assert "aht10 Ace_1" not in printer.objects
        assert printer.heaters.available_sensors == []
        assert printer.logger.messages == [
            ("warning", "AFC_ACE Ace_1: no temperature card (Printer object "
                        "'temperature_sensor Ace_1' already created)")]

    def test_a_heaters_refusal_leaves_the_unit_without_one(self):
        config = _config()
        printer = config.get_printer()
        printer.heaters.register_sensor = Recorder(raises=RuntimeError("heaters boom"))

        assert add_card(config, "Ace_1", FakeReader(), printer.logger) is None

        assert printer.logger.messages == [
            ("warning", "AFC_ACE Ace_1: no temperature card (heaters boom)")]


# ── _register_sensor ──────────────────────────────────────────────────────────

class TestRegisterSensor:
    def test_lists_the_card_with_heaters_under_a_wrapper_of_its_own(
            self, tracking_configfile):
        config = _config()
        printer = config.get_printer()
        card = _card(config)

        _register_sensor(config, "temperature_sensor Ace_1", card)

        (wrapper, sensor), = printer.heaters.registered
        assert sensor is card
        assert isinstance(wrapper, _TrackingWrapper)
        assert (wrapper.printer, wrapper.get_name()) == (printer, "temperature_sensor Ace_1")
        assert wrapper.fileconfig.items("temperature_sensor Ace_1") == [
            ("sensor_type", "aht2x")]
        assert wrapper.access_tracking is config.access_tracking
        assert printer.heaters.available_sensors == ["temperature_sensor Ace_1"]

    def test_the_type_is_read_so_it_lands_in_the_settings(self, tracking_configfile):
        config = _config()
        _register_sensor(config, "temperature_sensor Ace_1", _card(config))
        assert config.access_tracking == {
            ("temperature_sensor ace_1", "sensor_type"): "aht2x"}


# ── HDC1080 ───────────────────────────────────────────────────────────────────

class TestHDC1080Init:
    def test_defaults_and_the_bus_from_the_unit_section(self, monkeypatch):
        config = _config(name="AFC_OpenAMS AMS_1")
        printer = config.get_printer()
        i2c = FakeI2C()
        monkeypatch.setattr(temperature_module.bus, "MCU_I2C_from_config",
                            Recorder(result=i2c))

        sensor = HDC1080(config, printer.logger)

        assert temperature_module.bus.MCU_I2C_from_config.calls == [
            ((config,), {"default_addr": 0x40, "default_speed": 200000})]
        assert (sensor.printer, sensor.reactor, sensor.name, sensor.logger,
                sensor.i2c) == (printer, printer.reactor, "AFC_OpenAMS AMS_1",
                                printer.logger, i2c)
        assert (sensor.temp_resolution, sensor.humidity_resolution) == (
            TEMP_RES[14], HUMI_RES[14])
        assert (sensor.temp_offset, sensor.humidity_offset,
                sensor.heater_enabled) == (0.0, 0.0, False)
        assert (sensor.init_sent, sensor.errors) == (False, 0)
        assert printer.event_handlers == {"klippy:connect": [sensor.handle_connect]}

    def test_the_options_are_taken(self, monkeypatch):
        sensor = _hdc1080(monkeypatch, values={
            "temp_resolution": 11, "humidity_resolution": 8, "temp_offset": -1.5,
            "humidity_offset": 2.0, "heater_enabled": True})
        assert (sensor.temp_resolution, sensor.humidity_resolution) == (0x0400, 0x0200)
        assert (sensor.temp_offset, sensor.humidity_offset,
                sensor.heater_enabled) == (-1.5, 2.0, True)


class TestHDC1080HandleConnect:
    def test_a_configured_sensor_is_read_from_then_on(self, monkeypatch):
        sensor = _hdc1080(monkeypatch, FakeI2C(0x5449, 0x1050, 0x1000, 0x1000))

        sensor.handle_connect()

        assert sensor.init_sent is True
        assert sensor.printer.logger.messages == [
            ("debug", "AFC_OpenAMS AMS_1: HDC1080 manufacturer 0x5449 device 0x1050")]

    def test_a_sensor_that_does_not_answer_is_reported_and_left_unread(
            self, monkeypatch):
        sensor = _hdc1080(monkeypatch, FakeI2C(OSError("i2c timeout")))

        sensor.handle_connect()

        assert sensor.init_sent is False
        assert sensor.printer.logger.messages == [
            ("warning", "AFC_OpenAMS AMS_1: HDC1080 not configured (i2c timeout); "
                        "no temperature or humidity")]


class TestHDC1080InitDevice:
    def test_configures_the_sequence_the_resolutions_and_logs_the_ids(
            self, monkeypatch):
        i2c = FakeI2C(0x5449, 0x1050, 0x1000, 0x1400)
        sensor = _hdc1080(monkeypatch, i2c, values={"temp_resolution": 11,
                                                    "humidity_resolution": 8})

        sensor._init_device()

        assert i2c.writes == [
            [CONF_REG, 0x10, 0x00],                 # acquire both, temperature first
            [MFID_REG], [DVID_REG],
            [CONF_REG], [CONF_REG, 0x14, 0x00],     # 0x1000 with the 11-bit temp bit
            [CONF_REG], [CONF_REG, 0x16, 0x00]]     # 0x1400 with the 8-bit humidity field
        assert i2c.reads == [([], 2)] * 4
        assert sensor.printer.logger.messages == [
            ("debug", "AFC_OpenAMS AMS_1: HDC1080 manufacturer 0x5449 device 0x1050")]

    def test_the_heater_is_switched_on_when_asked(self, monkeypatch):
        i2c = FakeI2C(0x5449, 0x1050, 0x1000, 0x1000, 0x1000)
        sensor = _hdc1080(monkeypatch, i2c, values={"heater_enabled": True})

        sensor._init_device()

        assert i2c.writes[-1] == [CONF_REG, (0x1000 | HEATER_ENABLE_BIT) >> 8, 0x00]
        assert len(i2c.writes) == 9

    def test_the_heater_is_left_alone_by_default(self, monkeypatch):
        i2c = FakeI2C(0x5449, 0x1050, 0x1000, 0x1000)
        sensor = _hdc1080(monkeypatch, i2c)
        sensor._init_device()
        assert len(i2c.writes) == 7


class TestHDC1080ReadRegister16:
    def test_selects_the_register_waits_and_reads_big_endian(self, monkeypatch):
        i2c = FakeI2C(0x1234)
        sensor = _hdc1080(monkeypatch, i2c)
        reactor = sensor.reactor
        start = reactor.monotonic()

        assert sensor._read_register_16(MFID_REG) == 0x1234

        assert i2c.writes == [[MFID_REG]]
        assert i2c.reads == [([], 2)]
        assert reactor.pause.calls == [((start + 0.0635,), {})]


class TestHDC1080SetConfigBits:
    def test_rewrites_one_field_of_the_configuration_register(self, monkeypatch):
        i2c = FakeI2C(0x1700)
        sensor = _hdc1080(monkeypatch, i2c)
        reactor = sensor.reactor
        start = reactor.monotonic()

        sensor._set_config_bits(0x0300, 0x0100)

        # 0x1700 with the 0x0300 field cleared (0x1400) and set to 0x0100.
        assert i2c.writes == [[CONF_REG], [CONF_REG, 0x15, 0x00]]
        # The reactor's pause moves the clock, so the waits add up.
        assert [c[0][0] for c in reactor.pause.calls] == pytest.approx(
            [start + 0.0635, start + 0.0635 + 0.015])


class TestHDC1080Read:
    def test_scales_the_raw_value_and_adds_the_offset(self, monkeypatch):
        sensor = _hdc1080(monkeypatch, FakeI2C(0x8000))
        # Half scale of 165 C from -40 C, plus a 1.5 C offset.
        assert sensor._read(TEMP_REG, 165.0, -40.0 + 1.5) == 0.5 * 165.0 - 40.0 + 1.5
        assert sensor.printer.logger.messages == []

    def test_a_failed_read_is_none_and_logged(self, monkeypatch):
        sensor = _hdc1080(monkeypatch, FakeI2C(OSError("nack")))
        assert sensor._read(HUMI_REG, 100.0, 0.0) is None
        assert sensor.printer.logger.messages == [
            ("debug", "AFC_OpenAMS AMS_1: HDC1080 read 0x1 failed (nack)")]


class TestHDC1080ReadForCard:
    @staticmethod
    def _ready(monkeypatch: pytest.MonkeyPatch, *responses: Any,
               values: Optional[Dict[str, Any]] = None) -> Tuple[HDC1080, FakeI2C]:
        """
        :param monkeypatch: pytest's monkeypatch fixture
        :param responses: the bus's answers once configured
        :param values: the unit section's options
        :return tuple: a configured sensor and its bus
        """
        i2c = FakeI2C(*responses)
        sensor = _hdc1080(monkeypatch, i2c, values=values)
        sensor.init_sent = True
        return sensor, i2c

    def test_nothing_is_read_before_the_sensor_is_configured(self, monkeypatch):
        i2c = FakeI2C()
        sensor = _hdc1080(monkeypatch, i2c)
        assert sensor.read(100.0) == (None, None)
        assert (i2c.writes, i2c.reads) == ([], [])

    def test_temperature_then_humidity_with_the_offsets(self, monkeypatch):
        sensor, i2c = self._ready(monkeypatch, 0x4000, 0x8000,
                                  values={"temp_offset": 0.5, "humidity_offset": -2.0})
        start = sensor.reactor.monotonic()

        temperature, humidity = sensor.read(100.0)

        assert temperature == pytest.approx(0.25 * 165.0 - 40.0 + 0.5)
        assert humidity == pytest.approx(0.5 * 100.0 - 2.0)
        assert i2c.writes == [[TEMP_REG], [HUMI_REG]]
        # Register wait, the gap between the two reads, register wait again.
        assert [c[0][0] for c in sensor.reactor.pause.calls] == pytest.approx(
            [start + 0.0635, start + 0.0635 + 0.015, start + 0.0635 + 0.015 + 0.0635])
        assert sensor.errors == 0

    def test_a_failed_temperature_alone_is_no_error(self, monkeypatch):
        sensor, _i2c = self._ready(monkeypatch, OSError("nack"), 0x8000)
        sensor.errors = 2
        assert sensor.read(100.0) == (None, 50.0)
        assert sensor.errors == 0

    def test_a_failed_humidity_alone_is_no_error(self, monkeypatch):
        sensor, _i2c = self._ready(monkeypatch, 0x8000, OSError("nack"))
        sensor.errors = 2
        assert sensor.read(100.0) == (0.5 * 165.0 - 40.0, None)
        assert sensor.errors == 0

    def test_both_failing_counts_and_reports_once_at_the_limit(self, monkeypatch):
        failures = [OSError("nack")] * (2 * (HDC1080_MAX_ERRORS + 1))
        sensor, _i2c = self._ready(monkeypatch, *failures)

        for _ in range(HDC1080_MAX_ERRORS + 1):
            assert sensor.read(100.0) == (None, None)

        assert sensor.errors == HDC1080_MAX_ERRORS + 1
        warnings = [m for m in sensor.printer.logger.messages if m[0] == "warning"]
        assert warnings == [
            ("warning", f"AFC_OpenAMS AMS_1: HDC1080 not answering "
                        f"({HDC1080_MAX_ERRORS} reads failed)")]

    def test_a_good_read_resets_the_count(self, monkeypatch):
        sensor, _i2c = self._ready(monkeypatch, OSError("a"), OSError("b"), 0x8000, 0x8000)
        sensor.read(100.0)
        assert sensor.errors == 1
        sensor.read(105.0)
        assert sensor.errors == 0


# ── _resolution ───────────────────────────────────────────────────────────────

class TestResolution:
    def test_defaults_to_fourteen_bits(self):
        assert _resolution(_config(), "temp_resolution", TEMP_RES) == TEMP_RES[14]

    def test_a_listed_resolution_maps_to_its_register_bits(self):
        config = _config(values={"humidity_resolution": 8})
        assert _resolution(config, "humidity_resolution", HUMI_RES) == 0x0200

    def test_an_unlisted_resolution_in_range_is_refused(self):
        config = _config(values={"humidity_resolution": 12})
        with pytest.raises(configparser.Error) as exc_info:
            _resolution(config, "humidity_resolution", HUMI_RES)
        assert str(exc_info.value) == (
            "[AFC_ACE Ace_1] humidity_resolution: 12 is not one of 8, 11, 14")

    def test_a_resolution_out_of_range_is_refused_by_the_config(self):
        config = _config(values={"temp_resolution": 16})
        with pytest.raises(configparser.Error):
            _resolution(config, "temp_resolution", TEMP_RES)
