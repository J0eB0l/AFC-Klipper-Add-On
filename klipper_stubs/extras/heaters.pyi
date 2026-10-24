# Minimal stub for Klipper's klippy/extras/heaters.py -- only the members
# actually used by this project's extras/AFC_extruder.py,
# extras/AFC_form_tip.py and extras/temperature_ace.py.
from typing import Any, Callable

class Heater:
    target_temp: float

class PrinterHeaters:
    def set_temperature(self, heater: Heater, temp: float, wait: bool = False) -> None: ...
    def add_sensor_factory(self, sensor_type: str,
                           sensor_factory: Callable[..., Any]) -> None: ...
