# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The Bambu AMS buffer. It subclasses upstream's AFCFPSBuffer and lives here,
# not in AFC_buffer.py, so that file stays stock: AFC_BridgeBox constructs it
# directly for a chain whose `buffer_type` is `bambu`.

from __future__ import annotations

import traceback

from configparser import Error as error

from typing import TYPE_CHECKING, Optional, Union

if TYPE_CHECKING:
    from configfile import ConfigWrapper

try: from extras.AFC_buffer import AFCFPSBuffer
except:
    error_str = (f"Error when trying to import AFC_buffer.AFCFPSBuffer\n"
                 f"{traceback.format_exc()}")
    raise error(error_str)
try: from extras.AFC_lane import AFCLaneState
except:
    error_str = (f"Error when trying to import AFC_lane.AFCLaneState\n"
                 f"{traceback.format_exc()}")
    raise error(error_str)

# ── Bambu AMS buffer ───────────────────────────────────────────────────────────
# Load gate: buffer compressed AND the AMS odometer moved then stopped; either
# alone misreads friction or a mid-feed stop. Windows are in seconds.
class AFCBambuBuffer(AFCFPSBuffer):
    """
    Bambu AMS buffer used as a toolhead load sensor.

    Behaves as an ordinary FPS buffer in every other respect (gauge,
    QUERY_BUFFER, buffer ramming, the Mainsail panel), and only narrows
    when the buffer is allowed to report filament AT THE TOOLHEAD. That
    answer additionally requires the AMS's own odometer to have moved and
    then stopped, which is what distinguishes filament held up by the
    extruder gears from filament merely dragging in the bowden.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Initialize the Bambu buffer from its config section.

        :param config: ConfigWrapper; everything AFCFPSBuffer takes, plus the
                       odometer gate's own options.
        """
        super().__init__(config)
        # A sample moving less than this is "still": feeding steps 70-80 mm,
        # the braking tail flickers at 1-2 mm.
        self.odom_eps_mm: float = config.getfloat(
            "odom_eps_mm", 3.0, minval=0.1)
        # Seconds the odometer must hold still to count as stopped; 0.5 s is
        # two 250 ms frames.
        self.odom_still_seconds: float = config.getfloat(
            "odom_still_seconds", 0.5, minval=0.0)
        # With no odometer on any bound unit: True keeps the gate shut, False
        # falls back to buffer pressure alone.
        self.odom_required: bool = config.getboolean("odom_required", False)
        # Seconds at or below low_point before a latched load counts as run
        # out. The print sawtooth dips to ~0.13 and recovers within a second;
        # an empty buffer stays near 0.02.
        self.unload_confirm_seconds: float = config.getfloat(
            "unload_confirm_seconds", 2.0, minval=0.0)
        # Seconds a movement stays evidence, so an idle unit cannot satisfy the
        # gate on an old move.
        self.odom_move_ttl_seconds: float = config.getfloat(
            "odom_move_ttl_seconds", 5.0, minval=0.0)

        # Seconds to wait at boot for the pool to bind a unit before treating
        # the buffer as having none. The gate stays shut quietly meanwhile.
        self.odom_bind_grace_seconds: float = config.getfloat(
            "odom_bind_grace_seconds", 10.0, minval=0.0)
        # Seconds a still, compressed buffer must rest before it counts as
        # filament at the gears. Covers a restart with filament loaded, which
        # the arrival detector cannot see.
        self.resting_load_seconds: float = config.getfloat(
            "resting_load_seconds", 3.0, minval=0.0)
        # Seconds after the first sample that AFC's restored tool_loaded record
        # is consulted (restart recovery only).
        self.record_check_seconds: float = config.getfloat(
            "record_check_seconds", 60.0, minval=0.0)

        self._odom_units: Optional[list] = None   # resolved lazily, see below
        self._odom_prev: dict = {}
        # These are reactor times, not counters (see the module note above the
        # class).
        self._first_sample_t: Optional[float] = None
        # The first sample taken after AFC's PREP had run (see _bind_wait_start).
        self._prep_seen_t: Optional[float] = None
        self._last_move_t: Optional[float] = None
        self._now_t: float = 0.0
        self._tension_since: Optional[float] = None
        self._compressed_since: Optional[float] = None
        self._record_checked: bool = False
        self._odom_moved: bool = False
        self._gate_latched: bool = False
        self._odom_warned: bool = False

    # ── Which units' odometers belong to this buffer ───────────────────────
    def _bound_units(self) -> list:
        """
        The AFC_BambuAMS units whose lanes name this buffer.

        Resolved on first use rather than at config time: lanes bind their
        buffer_name during connect, so asking earlier finds nothing.

        Only a non-empty answer is cached: the first ADC callback can land
        while the pool is still claiming units, so a scan that found nothing
        means "not yet", not "never".

        Every unit on one bridge shares one buffer and one extruder, so the
        set is normally the units of a single Pico. They are all polled and
        the gate takes the worst case, which avoids having to know which
        unit is feeding: ``current_lane`` cannot answer that, because
        enable_buffer() does not run until after the load has finished.

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
            # A unit turned up after a degrade warning: let a genuine later
            # loss say so again rather than staying quiet on the first one.
            self._odom_warned = False
        self._odom_units = found
        return found

    def _update_odom(self) -> None:
        """
        Fold this sample's odometer readings into the stillness run.

        Per unit, because the units of one bridge each keep their own
        odometer and only the feeding one moves. Any unit moving resets the
        run; the run only advances when every unit that reports is still.
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
            if prev is not None and abs(v - prev) >= self.odom_eps_mm:
                moved = True
        if not saw_any:
            return
        if moved:
            # Moved first, then still: without the movement a gate that opens
            # before the unit starts would call an unstarted feed "arrived".
            self._odom_moved = True
            self._last_move_t = self._now_t
        elif (self._last_move_t is not None
              and self._now_t - self._last_move_t
              > self.odom_move_ttl_seconds):
            # Too old to be this feed's movement; let it lapse.
            self._odom_moved = False

    def _sample_time(self, read_time: Union[float, list],
                     read_value: Optional[float]) -> float:
        """
        The reactor time a sample belongs to, however it was delivered.

        The base class accepts three shapes, a (time, value) pair, a list
        of them, or a bare value with no time at all, and normalises them
        privately. The gate needs the time itself, so it reads it here
        before handing the arguments on unchanged.

        Where there is no time, ask the reactor, as the base class does. The
        virtual chip delivers bare values, and a fixed timestamp would stop
        the stillness clock from ever advancing, so the gate would never open.

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

        :return bool: True when it has moved and then gone still, or when no
                      odometer exists and ``odom_required`` allows it
        """
        if self._bound_units():
            if not self._odom_prev:
                # Bound, but no odometer reading yet: answer "not yet" rather
                # than fall back to pressure alone.
                return False
            return (self._odom_moved
                    and self._odom_still_for() >= self.odom_still_seconds)
        # No unit bound yet may just mean the pool has not claimed one, so
        # refuse quietly until the grace is spent.
        start = self._bind_wait_start()
        if start is None or self._now_t - start < self.odom_bind_grace_seconds:
            return False
        # Genuinely no unit on this buffer has an odometer.
        if self.odom_required:
            return False
        if not self._odom_warned:
            self._odom_warned = True
            self.logger.warning(
                f"{self.name}: no Bambu unit on this buffer reports an "
                f"odometer, so a load is being judged on buffer pressure "
                f"alone. Bowden friction can read as filament that way. "
                f"Set odom_required: True to refuse instead.")
        return True

    def _bind_wait_start(self) -> Optional[float]:
        """
        When the wait for a unit to bind begins.

        BridgeBox claims pool units only once AFC's PREP has run, which can
        be well after the first sample, so the wait runs from whichever came
        later: the first sample, or the first sample after PREP.

        :return float: that sample's time, or None while PREP has not run
        """
        if self._first_sample_t is None or self._prep_seen_t is None:
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

    # ── The gate itself ────────────────────────────────────────────────────
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
        # Track compression from the pressure itself, not from advance_state,
        # which the gate clears further down.
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
            # Latch: the odometer is frozen for the whole print, so re-testing
            # it would drop the sensor mid-print. _hold_latched handles the
            # release.
            self._gate_latched = True
            self._log_detection(read_time)
            return
        if self._resting_load():
            # Filament sitting at the gears that this gate never saw arrive:
            # after a restart, or after any reset while a lane stayed loaded.
            self._gate_latched = True
            self._log_detection(read_time, resting=True)
            return
        # Pressure says yes, the odometer does not. Clear the FPS latch too,
        # or the next tick re-latches off the pressure alone.
        self.advance_state = False
        self._advance_latched = False
        # _now_t, not read_time: that can be a bare value or an empty batch.
        self._update_virtual_sensors(self._now_t)

    def _adopt_recorded_load(self) -> None:
        """
        Believe AFC's own record that a lane on this buffer is tool-loaded.

        A lane keeps ``tool_loaded`` across a reboot in saved vars, and
        ``AFC_BambuAMS._startup_restore_loaded`` re-asserts the AMS follower
        off the same flag. The gate needs it because it detects an arrival
        event, and after a restart there is no arrival to witness.

        The record is the primary signal and the buffer is the check on it:
        a record claiming loaded while the buffer sits at max tension (what
        an empty path reads like) is refused with a warning. After adoption,
        sustained max tension releases the latch as usual, so a stale record
        does not survive long.

        Answered once, within `record_check_seconds` of the first sample.
        """
        if self._record_checked or self._gate_latched:
            return
        if self._first_sample_t is None:
            return                        # no clock yet
        lanes = getattr(self, "lanes", None) or {}
        loaded = [ln for ln in lanes.values()
                  if getattr(ln, "tool_loaded", False)]
        if not loaded:
            # The record is restored seconds after the lanes bind, so keep
            # asking until record_check_seconds has passed.
            if (self._now_t - self._first_sample_t
                    >= self.record_check_seconds):
                self._record_checked = True
            return
        self._record_checked = True
        names = ", ".join(getattr(ln, "name", "?") for ln in loaded)
        # Check the raw reading too: the smoothed value may still be
        # converging. Either one at max tension refuses.
        reading = min(self.fps_value, self.smoothed_fps)
        if reading <= self.low_point:
            self.logger.warning(
                f"{self.name}: AFC records {names} loaded to the toolhead, "
                f"but the buffer is at max tension ({reading:.2f}), which is "
                f"what an empty path reads like. Not adopting it -- the next "
                f"load will settle this.")
            return
        self._gate_latched = True
        self.logger.info(
            f"{self.name}: adopting {names} as loaded at the toolhead from "
            f"AFC's own record, buffer agreeing at {reading:.2f}")

    def _resting_load(self) -> bool:
        """
        Whether a still, steadily compressed buffer means filament is held.

        Both halves are required. An active feed refreshes the odometer every
        250 ms, so it can never accumulate the stillness window; this only
        fires on a unit doing nothing while its buffer stays compressed.

        An unload is excluded outright: its cut pushes the tip back into the
        hotend with the unit parked, which holds the buffer compressed and
        the odometer still for long enough to look like a resting load.

        :return bool: True when the resting state should be adopted
        """
        if self._compressed_since is None:
            return False
        if self._unloading():
            return False
        if self._now_t - self._compressed_since < self.resting_load_seconds:
            return False
        if self._last_move_t is None:
            return True                   # nothing has moved since boot
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

    def _log_detection(self, read_time: float,
                       resting: bool = False) -> None:
        """
        Record the instant the gate called a load, with what decided it.

        One line, on latch only, so a detection can be compared after the fact
        against the toolhead switch's own line in the same log.

        :param read_time: reactor time of the deciding sample
        :param resting: True when a resting compressed buffer decided it, not the
                        odometer
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
            self.logger.info(
                f"{self.name}: load detected at {self._now_t:.2f} -- "
                f"fps {self.fps_value:.2f} (smoothed {self.smoothed_fps:.2f}), "
                f"{how} [{odoms}]")
        except Exception:
            pass               # a log line must never break the gate

    def _hold_latched(self) -> None:
        """
        Keep a confirmed load reported while the buffer swings under a print.

        Once the filament is at the gears the odometer is frozen for the
        whole print (it only advances during a commanded feed), and the
        buffer swings roughly 0.14 to 0.96 on a ~4 s cycle as the AMS refills
        it. Letting either drive the sensor would report false runouts.

        Sustained max tension releases the latch, so a real runout is still
        seen: a printing sawtooth bottoms out around 0.12-0.14 and recovers
        within about a second, while an emptied buffer goes to ~0.02 and
        stays.
        """
        if self.smoothed_fps <= self.low_point:
            if self._tension_since is None:
                self._tension_since = self._now_t
        else:
            self._tension_since = None
        if (self._tension_since is not None
            and self._now_t - self._tension_since
            >= self.unload_confirm_seconds):
            held = self._now_t - self._tension_since
            self.logger.debug(
                f"{self.name}: buffer has been at max tension for "
                f"{held:.1f}s, releasing the load latch")
            self._reset_gate()
            return
        if not self.advance_state:
            self.advance_state = True
            self._update_virtual_sensors(self._now_t)

    @property
    def buffer_triggered(self) -> bool:
        """
        True when the buffer is compressed AND the odometer confirms arrival.

        The software endstop homes on this, so it carries the same gate as
        advance_state rather than pressure alone.

        :return bool: whether filament is at the toolhead
        """
        if not AFCFPSBuffer.buffer_triggered.fget(self):
            return False
        return self._gate_latched or self._odom_confirms()

    # ── Lifecycle ──────────────────────────────────────────────────────────
    # enable_buffer() is inherited: it runs right after arrival, so the gate is
    # reset on unload (disable_buffer) instead.
    def disable_buffer(self) -> None:
        """
        Disable the buffer and reset the odometer gate.

        The record check is not re-armed here. AFC calls this near the top of
        TOOL_UNLOAD and only clears `tool_loaded` much later, at the end, so
        for the whole of an unload the record still reads loaded, so re-arming
        would adopt the lane on its way out and latch the gate back on. The
        same reasoning keeps it out of _reset_gate(), which the runout
        release calls precisely because the buffer has proved the path empty.
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
        # The gate's verdict. An extruder with a real toolhead switch never
        # consults the gate, so this field shows what it would have said.
        response["load_detected"] = self.buffer_triggered
        response["advance_state"] = self.advance_state
        return response
