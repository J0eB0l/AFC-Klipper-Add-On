# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The Bambu AMS buffer.


from __future__ import annotations

import traceback

from configparser import Error as error

from typing import TYPE_CHECKING, Optional, Union

if TYPE_CHECKING:
    from configfile import ConfigWrapper

try: from extras.AFC_buffer import AFCFPSBuffer
except:
    error_str = f"Error when trying to import AFC_buffer.AFCFPSBuffer\n{traceback.format_exc()}"
    raise error(error_str)
try: from extras.AFC_lane import AFCLaneState
except:
    error_str = f"Error when trying to import AFC_lane.AFCLaneState\n{traceback.format_exc()}"
    raise error(error_str)

# Load gate: buffer compressed AND the AMS odometer moved then stopped.
class AFCBambuBuffer(AFCFPSBuffer):
    """
    Bambu AMS buffer used as a toolhead load sensor.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Initialize the Bambu buffer from its config section.

        :param config: ConfigWrapper; everything AFCFPSBuffer takes, plus the odometer gate's own
        options
        """
        super().__init__(config)
        # A sample moving less than this is "still".
        self.odom_eps_mm: float = config.getfloat("odom_eps_mm", 3.0, minval=0.1)
        # 0.5 s is two 250 ms odometer frames.
        self.odom_still_seconds: float = config.getfloat("odom_still_seconds", 0.5, minval=0.0)
        # With no odometer on any bound unit.
        self.odom_required: bool = config.getboolean("odom_required", False)
        # Seconds at or below low_point before a latched load counts as run out.
        self.unload_confirm_seconds: float = config.getfloat("unload_confirm_seconds",
                                                             2.0, minval=0.0)
        self.odom_move_ttl_seconds: float = config.getfloat("odom_move_ttl_seconds",
                                                            5.0, minval=0.0)

        # The gate stays shut quietly while the pool binds a unit at boot.
        self.odom_bind_grace_seconds: float = config.getfloat("odom_bind_grace_seconds",
                                                              10.0, minval=0.0)
        # Covers a restart with filament loaded, which the arrival detector cannot see.
        self.resting_load_seconds: float = config.getfloat("resting_load_seconds", 3.0, minval=0.0)
        self.record_check_seconds: float = config.getfloat("record_check_seconds", 60.0, minval=0.0)

        self._odom_units: Optional[list] = None
        self._odom_prev: dict = {}
        self._first_sample_t: Optional[float] = None
        self._prep_seen_t: Optional[float] = None
        self._last_move_t: Optional[float] = None
        self._now_t: float = 0.0
        self._tension_since: Optional[float] = None
        self._compressed_since: Optional[float] = None
        self._record_checked: bool = False
        self._odom_moved: bool = False
        self._gate_latched: bool = False
        self._odom_warned: bool = False

    def _bound_units(self) -> list:
        """
        The AFC_BambuAMS units whose lanes name this buffer.

        :return list: the bound units, possibly empty
        """
        if self._odom_units:
            return self._odom_units
        found = []
        try:
            for name, obj in self.printer.lookup_objects():
                if not name.startswith("AFC_BambuAMS "):
                    continue
                if not callable(getattr(obj, "_odom_now_mm", None)):
                    continue
                lanes = getattr(obj, "lanes", None) or {}
                for lane in getattr(lanes, "values", lambda: [])():
                    if getattr(lane, "buffer_name", None) == self.name:
                        found.append(obj)
                        break
        except Exception:
            return []          # never let discovery break a live callback
        if found:
            # A unit turned up after a degrade warning.
            self._odom_warned = False
        self._odom_units = found
        return found

    def rebind(self) -> None:
        """
        Look again for the units whose lanes name this buffer, after a lane took or left it. The
        odometer history starts over with them, so a unit that left decides nothing here.
        """
        self._odom_units = None
        self._odom_prev = {}
        self._last_move_t = None
        self._odom_moved = False

    def _update_odom(self) -> None:
        """
        Fold this sample's odometer readings into the stillness run.
        """
        units = self._bound_units()
        if not units:
            return
        moved = False
        saw_any = False
        for unit in units:
            try:
                v = unit._odom_now_mm()
            except Exception:
                v = None
            if v is None:
                continue
            saw_any = True
            key = getattr(unit, "name", id(unit))
            prev = self._odom_prev.get(key)
            self._odom_prev[key] = v
            if (prev is not None
                and abs(v - prev) >= self.odom_eps_mm):
                moved = True
        if not saw_any:
            return
        if moved:
            # Moved first, then still.
            self._odom_moved = True
            self._last_move_t = self._now_t
        elif (self._last_move_t is not None
              and self._now_t - self._last_move_t > self.odom_move_ttl_seconds):
            self._odom_moved = False

    def _sample_time(self, read_time: Union[float, list], read_value: Optional[float]) -> float:
        """
        The reactor time a sample belongs to, however it was delivered.

        :param read_time: reactor time, a list of (time, value), or a value
        :param read_value: the reading when ``read_time`` really is a time
        :return float: the sample's time
        """
        if isinstance(read_time, list):
            if read_time:
                return float(read_time[-1][0])
            return float(self.reactor.monotonic())
        if read_value is None:
            return float(self.reactor.monotonic())
        return float(read_time)

    def _odom_still_for(self) -> float:
        """
        Seconds since any bound unit's odometer last moved.

        :return float: the gap, or 0.0 before any movement has been seen
        """
        if self._last_move_t is None:
            return 0.0
        return max(0.0, self._now_t - self._last_move_t)

    def _odom_confirms(self) -> bool:
        """
        Whether the odometer agrees the filament has stopped at the gears.

        :return bool: True when it has moved and then gone still, or when no odometer exists and
        ``odom_required`` allows it
        """
        if self._bound_units():
            if not self._odom_prev:
                # Bound, but no odometer reading yet.
                return False
            return (self._odom_moved
                    and self._odom_still_for() >= self.odom_still_seconds)
        # No unit bound yet may just mean the pool has not claimed one, so refuse quietly until the
        # grace is spent.
        start = self._bind_wait_start()
        if (start is None
            or self._now_t - start < self.odom_bind_grace_seconds):
            return False
        if self.odom_required:
            return False
        if not self._odom_warned:
            self._odom_warned = True
            self.logger.warning(f"{self.name}: no Bambu unit on this buffer reports an "
                                f"odometer, so a load is being judged on buffer pressure "
                                f"alone. Bowden friction can read as filament that way. "
                                f"Set odom_required: True to refuse instead.")
        return True

    def _bind_wait_start(self) -> Optional[float]:
        """
        When the wait for a unit to bind begins.

        :return float: that sample's time, or None while PREP has not run
        """
        if (self._first_sample_t is None
            or self._prep_seen_t is None):
            return None
        return max(self._first_sample_t, self._prep_seen_t)

    def _reset_gate(self) -> None:
        """
        Forget the odometer history so the next load starts clean.
        """
        self._odom_prev = {}
        self._last_move_t = None
        self._odom_moved = False
        self._gate_latched = False
        self._tension_since = None
        self._compressed_since = None

    def _adc_callback(self, read_time: Union[float, list],
                      read_value: Optional[float] = None) -> None:
        """
        Update pressure as an FPS buffer does, then apply the odometer gate.

        :param read_time: reactor time, or a list of (time, value) samples
        :param read_value: the 0..1 reading when read_time is a bare time
        """
        self._now_t = self._sample_time(read_time, read_value)
        if self._first_sample_t is None:
            self._first_sample_t = self._now_t
        if (self._prep_seen_t is None
            and getattr(self.afc, "prep_done", True)):
            self._prep_seen_t = self._now_t
        super()._adc_callback(read_time, read_value)
        self._update_odom()
        self._adopt_recorded_load()
        # Track compression from the pressure itself, not from advance_state, which the gate clears
        # further down.
        if self.smoothed_fps > self.set_point + self.deadband / 2.0:
            if self._compressed_since is None:
                self._compressed_since = self._now_t
        else:
            self._compressed_since = None
        if self._gate_latched:
            self._hold_latched()
            return
        if not self.advance_state:
            return
        if self._odom_confirms():
            # Latch: the odometer is frozen for the whole print, so re-testing it would drop the
            # sensor mid-print.
            self._gate_latched = True
            self._log_detection(read_time)
            return
        if self._resting_load():
            # Filament sitting at the gears that this gate never saw arrive.
            self._gate_latched = True
            self._log_detection(read_time, resting=True)
            return
        # Clear the FPS latch too, or the next tick re-latches off the pressure alone.
        self.advance_state = False
        self._advance_latched = False

    def _adopt_recorded_load(self) -> None:
        """
        Believe AFC's own record that a lane on this buffer is tool-loaded.
        """
        if (self._record_checked
            or self._gate_latched):
            return
        if self._first_sample_t is None:
            return
        lanes = getattr(self, "lanes", None) or {}
        loaded = [ln for ln in lanes.values()
                  if getattr(ln, "tool_loaded", False)]
        if not loaded:
            # The record is restored seconds after the lanes bind, so keep asking until
            # record_check_seconds has passed.
            if (self._now_t - self._first_sample_t
                    >= self.record_check_seconds):
                self._record_checked = True
            return
        self._record_checked = True
        names = ", ".join(getattr(ln, "name", "?") for ln in loaded)
        # The smoothed value may still be converging, so the raw reading counts too.
        reading = min(self.fps_value, self.smoothed_fps)
        if reading <= self.low_point:
            self.logger.warning(f"{self.name}: AFC records {names} loaded to the toolhead, "
                                f"but the buffer is at max tension ({reading:.2f}), which is "
                                f"what an empty path reads like. Not adopting it -- the next "
                                f"load will settle this.")
            return
        self._gate_latched = True
        self.logger.debug(f"{self.name}: adopting {names} as loaded at the toolhead from "
                          f"AFC's own record, buffer agreeing at {reading:.2f}")

    def _resting_load(self) -> bool:
        """
        Whether a still, steadily compressed buffer means filament is held.

        :return bool: True when the resting state should be adopted
        """
        if self._compressed_since is None:
            return False
        if self._unloading():
            return False
        if self._now_t - self._compressed_since < self.resting_load_seconds:
            return False
        if self._last_move_t is None:
            return True
        return (self._now_t - self._last_move_t
                >= self.resting_load_seconds)

    def _unloading(self) -> bool:
        """
        Whether a lane on this buffer is in the middle of a tool unload.

        :return bool: True while any bound lane reports Tool Unloading
        """
        lanes = getattr(self, "lanes", None) or {}
        return any(getattr(ln, "status", None) == AFCLaneState.TOOL_UNLOADING
                   for ln in lanes.values())

    def _log_detection(self, read_time: float, resting: bool = False) -> None:
        """
        Record the instant the gate called a load, with what decided it.

        :param read_time: reactor time of the deciding sample
        :param resting: True when a resting compressed buffer decided it, not the odometer
        """
        try:
            odoms = ", ".join(
                f"{getattr(u, 'name', '?')}="
                f"{self._odom_prev.get(getattr(u, 'name', id(u)), float('nan')):.1f}mm"
                for u in self._bound_units())
            how = ("filament already at the gears (resting, compressed "
                   f"for {self._now_t - self._compressed_since:.1f}s)"
                   if resting and self._compressed_since is not None
                   else f"odometer still for {self._odom_still_for():.2f}s")
            self.logger.debug(f"{self.name}: load detected at {self._now_t:.2f} -- "
                              f"fps {self.fps_value:.2f} (smoothed {self.smoothed_fps:.2f}), "
                              f"{how} [{odoms}]")
        except Exception:
            pass               # a log line must never break the gate

    def _hold_latched(self) -> None:
        """
        Keep a confirmed load reported while the buffer swings under a print.
        """
        if self.smoothed_fps <= self.low_point:
            if self._tension_since is None:
                self._tension_since = self._now_t
        else:
            self._tension_since = None
        if (self._tension_since is not None
            and self._now_t - self._tension_since >= self.unload_confirm_seconds):
            held = self._now_t - self._tension_since
            self.logger.debug(f"{self.name}: buffer has been at max tension for "
                              f"{held:.1f}s, releasing the load latch")
            self._reset_gate()
            return
        self.advance_state = True

    @property
    def buffer_triggered(self) -> bool:
        """
        True when the buffer is compressed AND the odometer confirms arrival.

        :return bool: whether filament is at the toolhead
        """
        if not AFCFPSBuffer.buffer_triggered.fget(self):
            return False
        return self._gate_latched or self._odom_confirms()

    # The gate resets on unload, since the inherited enable_buffer runs right after arrival.
    def disable_buffer(self) -> None:
        """
        Disable the buffer and reset the odometer gate.
        """
        self._reset_gate()
        super().disable_buffer()

    def get_status(self, eventtime: Optional[float] = None) -> dict:
        """
        Publish the gate alongside the usual FPS fields.

        :param eventtime: reactor time (unused)
        :return dict: status for Moonraker/Mainsail
        """
        response = super().get_status(eventtime)
        response["odom_still_s"] = round(self._odom_still_for(), 2)
        response["odom_moved"] = self._odom_moved
        response["odom_confirms"] = self._odom_confirms()
        response["load_latched"] = self._gate_latched
        response["tension_s"] = (
            0.0 if self._tension_since is None
            else round(self._now_t - self._tension_since, 2))
        # The gate's verdict.
        response["load_detected"] = self.buffer_triggered
        response["advance_state"] = self.advance_state
        return response
