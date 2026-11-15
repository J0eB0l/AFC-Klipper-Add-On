# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Bambu AMS temperature and humidity sensor


from __future__ import annotations

import logging
from typing import Any, Callable, Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from klippy import Printer

_fallback_logger = logging.getLogger("temperature_bambu")


class TemperatureBambu:
    """
    Temperature/humidity sensor backed by an AFC_BambuAMS unit's status.
    """

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Initialize the sensor and bind it to an AFC_BambuAMS unit.

        :param config: Klipper config for this ``[temperature_sensor]`` section
        """
        self.printer: "Printer" = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.unit_name = config.get("bambu_unit", self.name)
        self.report_time = config.getint("report_time", 5, minval=1)

        self.temp = 0.0
        self.min_temp = 0.0
        self.max_temp = 90.0
        self.humidity = 0.0
        self._has_temp = False
        # Whether this unit reports a temperature at all (a regular AMS has no dryer).
        self._temp_capable: Optional[bool] = config.getboolean("report_temperature", None)
        self._temp_pinned = self._temp_capable is not None
        self._unit: Any = None
        self._callback: Optional[Callable[[float, float], None]] = None
        self._warned = False

        # Register as "aht10 <name>" so Mainsail and Fluidd show humidity, same as temperature_oams
        # and temperature_ace.
        self.simulate_aht = config.getboolean("simulate_supported_sensor_mainsail", True)
        if self.simulate_aht:
            self.printer.add_object("aht10 " + self.name, self)
            # Also register under the configured sensor_type, since Fluidd looks up e.g.
            stype = config.get("sensor_type", "").strip()
            if (stype
                and stype.lower() != "aht10"):
                try:
                    self.printer.add_object(stype + " " + self.name, self)
                except Exception:
                    pass      # already taken by a real sensor of that type
        else:
            self.printer.add_object("temperature_bambu " + self.name, self)

        self.sample_timer = self.reactor.register_timer(self._sample)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self) -> None:
        """
        Start sampling once the printer is up and units exist.
        """
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp: float, max_temp: float) -> None:
        """
        Store the configured allowed range.

        :param min_temp: Minimum allowed temperature in C
        :param max_temp: Maximum allowed temperature in C
        """
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb: Callable[[float, float], None]) -> None:
        """
        Register the heaters callback used to report temperatures.

        :param cb: Callable invoked as ``cb(print_time, temperature)``
        """
        self._callback = cb

    def get_report_time_delta(self) -> float:
        """
        Return the reporting interval.

        :return float: Interval in seconds
        """
        return self.report_time

    def _resolve_unit(self) -> Any:
        """
        Look up the AFC_BambuAMS unit backing this sensor.

        :return Any: The unit object, or None if it is not configured
        """
        unit = self.printer.lookup_object("AFC_BambuAMS " + self.unit_name, None)
        if (unit is None
            and not self._warned):
            self._warned = True
            _fallback_logger.warning("temperature_bambu %s: no AFC_BambuAMS unit '%s'", self.name,
                                     self.unit_name)
        return unit

    def _sample(self, eventtime: float) -> float:
        """
        Timer callback: read the unit's status and feed the heaters system.

        :param eventtime: Reactor event time of this firing
        :return float: Next reactor time to fire
        """
        try:
            if self._unit is None:
                self._unit = self._resolve_unit()
            if self._unit is not None:
                if not self._temp_pinned:
                    # Re-read each sample, since AFC_BridgeBox can add the dryer live.
                    self._temp_capable = bool(getattr(self._unit, "has_heater", True))
                st = self._unit.get_status(eventtime) or {}
                h = st.get("humidity")
                if h is not None:
                    self.humidity = float(h)
                t = st.get("temperature")
                if t is not None:
                    self.temp = float(t)
                    self._has_temp = True
                    if (self.temp < self.min_temp
                        or self.temp > self.max_temp):
                        self.printer.invoke_shutdown(
                            "temperature_bambu %s: %.1f outside range %.1f:%.1f"
                            % (self.name, self.temp, self.min_temp,
                               self.max_temp))
        except Exception as e:
            _fallback_logger.debug("temperature_bambu %s: sample failed: %s", self.name, e)

        measured_time = self.reactor.monotonic()
        if self._callback is not None:
            # Report the last real reading, or min_temp when there has been none, so a 0.0 below
            # min_temp does not shut the printer down.
            value = self.temp if self._has_temp else self.min_temp
            self._callback(
                self.printer.lookup_object("mcu").estimated_print_time(
                    measured_time),
                value)
        return measured_time + self.report_time

    def get_status(self, eventtime: float) -> Dict[str, float]:
        """
        Return the values the web UI renders.

        :param eventtime: Reactor event time (unused)
        :return dict: ``temperature`` and ``humidity``
        """
        out = {"humidity": round(self.humidity, 1)}
        # Omit temperature entirely on a unit that cannot produce one, rather than publishing a flat
        # 0.0 next to a live humidity.
        if self._temp_capable is not False:
            out["temperature"] = round(self.temp, 1)
        return out


def load_config(config: "ConfigWrapper") -> None:
    """
    Register ``temperature_bambu`` as a heaters sensor factory.

    :param config: ConfigWrapper used to look up the heaters object
    """
    pheaters = config.get_printer().lookup_object("heaters")
    pheaters.add_sensor_factory("temperature_bambu", TemperatureBambu)
    # Also the shared aht2x type, which sends a bambu_unit section here.
    from extras.temperature_afc import register_on_heaters
    register_on_heaters(pheaters)
