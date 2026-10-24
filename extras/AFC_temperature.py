# AFC temperature cards
#
# Puts an AFC unit's temperature and humidity on the Mainsail and Fluidd
# temperature card with no [temperature_sensor] section and no sensor type.
# A unit builds its card when it loads: a "temperature_sensor <unit>" object
# shaped like Klipper's own, listed with heaters, and an "aht10 <unit>" object
# carrying the humidity, which both UIs read by that name. The card's type is
# recorded in configfile.settings as aht2x, the type Fluidd checks before it
# looks for humidity.
#
# HDC1080 driver Copyright (C) 2024 JR Lomas (discord:knight_rad.iant)
# <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations

import configparser
from typing import Any, Callable, Dict, Optional, Tuple, TYPE_CHECKING

import configfile
from . import bus

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from klippy import Printer
    from reactor import SelectReactor as Reactor
    from extras.AFC_logger import AFC_logger

#: A reader's answer: (temperature C, humidity %RH), None for a value the
#: unit does not have.
Reading = Tuple[Optional[float], Optional[float]]
Reader = Callable[[float], Reading]

#: The object kind Mainsail and Fluidd list sensors by.
CARD_KIND = "temperature_sensor"
#: The object kind both UIs read humidity from (Fluidd for every aht type).
HUMIDITY_KIND = "aht10"
#: Recorded in configfile.settings so Fluidd looks for the humidity object.
SENSOR_TYPE = "aht2x"
#: Seconds between samples.
REPORT_TIME = 5.0

# HDC1080 registers, address and configuration bits.
TEMP_REG = 0x00
HUMI_REG = 0x01
CONF_REG = 0x02
MFID_REG = 0xFE
DVID_REG = 0xFF
HDC1080_I2C_ADDR = 0x40
HEATER_ENABLE_BIT = 0x2000
TEMP_RES_MASK = 0x0400
HUMI_RES_MASK = 0x0300
TEMP_RES = {14: 0x0000, 11: 0x0400}
HUMI_RES = {14: 0x0000, 11: 0x0100, 8: 0x0200}
#: Consecutive failed reads before the sensor is reported as not answering.
HDC1080_MAX_ERRORS = 5


class UnitTemperatureCard:
    """
    A unit's temperature card: the status Klipper's own temperature_sensor
    publishes (temperature, measured_min_temp, measured_max_temp), fed by the
    unit's reader on a timer instead of by a sensor type.
    """

    def __init__(self, config: ConfigWrapper, name: str, reader: Reader,
                 logger: AFC_logger, report_time: float = REPORT_TIME) -> None:
        """
        :param config: the unit's section wrapper
        :param name: the unit's name, the card's name on the UIs
        :param reader: eventtime -> (temperature, humidity), None for a value
            the unit does not have
        :param logger: the AFC logger
        :param report_time: seconds between samples
        """
        self.printer: Printer = config.get_printer()
        self.reactor: Reactor = self.printer.get_reactor()
        self.name = name
        self.reader = reader
        self.logger = logger
        self.report_time = report_time
        self.temperature: Optional[float] = None
        self.humidity: Optional[float] = None
        self.measured_min: Optional[float] = None
        self.measured_max: Optional[float] = None
        self._read_failed = False
        self.humidity_view = HumidityView(self)
        self.sample_timer = self.reactor.register_timer(self._sample)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self) -> None:
        """
        Start sampling once the printer is up and the unit can answer.
        """
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _sample(self, eventtime: float) -> float:
        """
        Timer callback: take the unit's reading and track the measured range.

        A reader that raises is logged once, until it answers again; a value
        it has not got (None) keeps the last one.

        :param eventtime: reactor time of this firing
        :return float: the next firing time
        """
        try:
            temperature, humidity = self.reader(eventtime)
        except Exception as e:
            if not self._read_failed:
                self._read_failed = True
                self.logger.warning(
                    f"AFC temperature {self.name}: read failed ({e})")
            return eventtime + self.report_time
        self._read_failed = False
        if temperature is not None:
            self.temperature = float(temperature)
            self.measured_min = (self.temperature if self.measured_min is None
                                 else min(self.measured_min, self.temperature))
            self.measured_max = (self.temperature if self.measured_max is None
                                 else max(self.measured_max, self.temperature))
        if humidity is not None:
            self.humidity = float(humidity)
        return eventtime + self.report_time

    def get_temp(self, eventtime: float) -> Tuple[float, float]:
        """
        Klipper's sensor interface, for TEMPERATURE_WAIT.

        :param eventtime: reactor time (unused)
        :return tuple: (temperature, target); a card has no target
        """
        return self.temperature or 0.0, 0.0

    def get_status(self, eventtime: float) -> Dict[str, float]:
        """
        :param eventtime: reactor time (unused)
        :return dict: temperature, measured_min_temp and measured_max_temp,
            0.0 before the first reading
        """
        return {"temperature": round(self.temperature or 0.0, 2),
                "measured_min_temp": round(self.measured_min or 0.0, 2),
                "measured_max_temp": round(self.measured_max or 0.0, 2)}


class HumidityView:
    """
    The card's ``aht10 <unit>`` object: the temperature with the humidity
    beside it, which is where Mainsail and Fluidd look for humidity.
    """

    def __init__(self, card: UnitTemperatureCard) -> None:
        """
        :param card: the card whose readings this shows
        """
        self.card = card

    def get_status(self, eventtime: float) -> Dict[str, float]:
        """
        :param eventtime: reactor time (unused)
        :return dict: temperature, plus humidity once the unit has reported one
        """
        status = {"temperature": round(self.card.temperature or 0.0, 2)}
        if self.card.humidity is not None:
            status["humidity"] = round(self.card.humidity, 2)
        return status


def add_card(config: ConfigWrapper, name: str, reader: Reader,
             logger: AFC_logger, report_time: float = REPORT_TIME
             ) -> Optional[UnitTemperatureCard]:
    """
    Build ``name``'s card and register it with the printer and heaters.

    A config that still carries a ``[temperature_sensor <name>]`` section is
    refused with the fix, since the unit builds that card itself now. Any
    other failure is logged and leaves the unit without a card.

    :param config: the unit's section wrapper
    :param name: the unit's name
    :param reader: eventtime -> (temperature, humidity), None for a value the
        unit does not have
    :param logger: the AFC logger
    :param report_time: seconds between samples
    :return Optional[UnitTemperatureCard]: the card, or None when it could not
        be built
    """
    section = f"{CARD_KIND} {name}"
    if config.fileconfig.has_section(section):
        error_str = (f"[{config.get_name()}] builds its own temperature card; "
                     f"remove the [{section}] section")
        raise config.error(error_str)
    printer = config.get_printer()
    try:
        card = UnitTemperatureCard(config, name, reader, logger, report_time)
        printer.add_object(section, card)
        printer.add_object(f"{HUMIDITY_KIND} {name}", card.humidity_view)
        _register_sensor(config, section, card)
    except Exception as e:
        logger.warning(f"{config.get_name()}: no temperature card ({e})")
        return None
    return card


def _register_sensor(config: ConfigWrapper, section: str,
                     card: UnitTemperatureCard) -> None:
    """
    List the card in heaters.available_sensors (Mainsail's sensor list) and
    record its type in configfile.settings (Fluidd's humidity check).

    heaters takes a section wrapper, so the card gets one over a section of
    its own that shares the printer's access tracking: what is read through
    it lands in configfile.settings like any real section's options.

    :param config: the unit's section wrapper
    :param section: the card's section name, ``temperature_sensor <name>``
    :param card: the card
    """
    printer = config.get_printer()
    fileconfig = configparser.RawConfigParser()
    fileconfig.add_section(section)
    fileconfig.set(section, "sensor_type", SENSOR_TYPE)
    wrapper = configfile.ConfigWrapper(printer, fileconfig,
                                       config.access_tracking, section)
    wrapper.get("sensor_type")
    printer.load_object(config, "heaters").register_sensor(wrapper, card)


class HDC1080:
    """
    The HDC1080 temperature and humidity sensor on an OpenAMS board, read for
    the unit's card over the unit section's I2C options (i2c_mcu, i2c_bus,
    i2c_address, i2c_speed) with optional offsets, resolutions and heater.
    """

    def __init__(self, config: ConfigWrapper, logger: AFC_logger) -> None:
        """
        :param config: the unit's section wrapper
        :param logger: the AFC logger
        """
        self.printer: Printer = config.get_printer()
        self.reactor: Reactor = self.printer.get_reactor()
        self.name = config.get_name()
        self.logger = logger
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=HDC1080_I2C_ADDR, default_speed=200000)
        self.temp_resolution = _resolution(config, "temp_resolution", TEMP_RES)
        self.humidity_resolution = _resolution(config, "humidity_resolution",
                                               HUMI_RES)
        self.temp_offset = config.getfloat("temp_offset", 0.0)
        self.humidity_offset = config.getfloat("humidity_offset", 0.0)
        self.heater_enabled = config.getboolean("heater_enabled", False)
        self.init_sent = False
        self.errors = 0
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self) -> None:
        """
        Configure the sensor once the MCU is up. One that does not answer is
        reported and left unread.
        """
        try:
            self._init_device()
        except Exception as e:
            self.logger.warning(f"{self.name}: HDC1080 not configured ({e}); "
                                "no temperature or humidity")
            return
        self.init_sent = True

    def _init_device(self) -> None:
        """
        Acquire temperature then humidity in one sequence, apply the
        resolutions and the heater, and log the chip's ids.
        """
        self.i2c.i2c_write([CONF_REG, 1 << 4, 0x00])
        manufacturer_id = self._read_register_16(MFID_REG)
        device_id = self._read_register_16(DVID_REG)
        self._set_config_bits(TEMP_RES_MASK, self.temp_resolution)
        self._set_config_bits(HUMI_RES_MASK, self.humidity_resolution)
        if self.heater_enabled:
            self._set_config_bits(HEATER_ENABLE_BIT, HEATER_ENABLE_BIT)
        self.logger.debug(f"{self.name}: HDC1080 manufacturer "
                          f"{manufacturer_id:#x} device {device_id:#x}")

    def _read_register_16(self, reg: int) -> int:
        """
        :param reg: register address
        :return int: its 16-bit big-endian value
        """
        self.i2c.i2c_write([reg])
        self.reactor.pause(self.reactor.monotonic() + 0.0635)
        data = bytearray(self.i2c.i2c_read([], 2)["response"])
        return (data[0] << 8) | data[1]

    def _set_config_bits(self, mask: int, value: int) -> None:
        """
        Rewrite one field of the configuration register.

        :param mask: the field's bits
        :param value: the field's new bits
        """
        conf = (self._read_register_16(CONF_REG) & ~mask) | value
        self.i2c.i2c_write([CONF_REG, conf >> 8, 0x00])
        self.reactor.pause(self.reactor.monotonic() + 0.015)

    def _read(self, reg: int, scale: float, offset: float) -> Optional[float]:
        """
        One measurement, raw / 65536 * scale + offset.

        :param reg: TEMP_REG or HUMI_REG
        :param scale: the register's full-scale span
        :param offset: added to the result
        :return Optional[float]: the value, None on an I2C failure
        """
        try:
            raw = self._read_register_16(reg)
        except Exception as e:
            self.logger.debug(f"{self.name}: HDC1080 read {reg:#x} failed ({e})")
            return None
        return raw / 65536.0 * scale + offset

    def read(self, eventtime: float) -> Reading:
        """
        The card's reader. Nothing is read before the sensor is configured;
        after HDC1080_MAX_ERRORS consecutive failed reads the sensor is
        reported once as not answering.

        :param eventtime: reactor time (unused)
        :return tuple: (temperature, humidity), None for a failed read
        """
        if not self.init_sent:
            return None, None
        temperature = self._read(TEMP_REG, 165.0, self.temp_offset - 40.0)
        self.reactor.pause(self.reactor.monotonic() + 0.015)
        humidity = self._read(HUMI_REG, 100.0, self.humidity_offset)
        if temperature is None and humidity is None:
            self.errors += 1
            if self.errors == HDC1080_MAX_ERRORS:
                self.logger.warning(f"{self.name}: HDC1080 not answering "
                                    f"({self.errors} reads failed)")
        else:
            self.errors = 0
        return temperature, humidity


def _resolution(config: ConfigWrapper, option: str, table: Dict[int, int]) -> int:
    """
    :param config: the unit's section wrapper
    :param option: temp_resolution or humidity_resolution
    :param table: bits -> register value
    :return int: the register value for the configured bits (default 14)
    """
    bits = config.getint(option, 14, minval=min(table), maxval=max(table))
    if bits not in table:
        valid = ", ".join(str(b) for b in sorted(table))
        error_str = f"[{config.get_name()}] {option}: {bits} is not one of {valid}"
        raise config.error(error_str)
    return table[bits]
