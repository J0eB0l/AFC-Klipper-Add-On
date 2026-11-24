# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The ACE 2 Pro's own buffer, used as a toolhead load sensor.

from __future__ import annotations

import traceback

from configparser import Error as error

from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from extras.AFC_lane import AFCLane

try: from extras.AFC_buffer import AFCBuffer
except:
    error_str = f"Error when trying to import AFC_buffer.AFCBuffer\n{traceback.format_exc()}"
    raise error(error_str)


def signed_decoder(value: Any) -> int:
    """
    The unit's encoder count as a signed number (it reports it as an
    unsigned 64-bit value, so a move backwards wraps).

    :param value: the raw decoder value from get_feed_info
    :return int: the signed count
    """
    value = int(value)
    return value - (1 << 64) if value >= (1 << 63) else value


class AFCACEBuffer(AFCBuffer):
    """
    The ACE 2 Pro's buffer as a toolhead load sensor (pin_tool_start: buffer).

    The unit has three switches per buffer: rest (buffer centred), back
    (buffer pulled, filament under tension) and a full switch at the pushed
    end, which the four buffers share. During a feed command the unit keeps
    pushing past full, but in feed assist it stops by itself once its buffer
    is full. So a load ends in feed assist: the filament has arrived when the
    full switch is on and the unit's encoder has stopped.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Initialize the ACE buffer from its config section.

        :param config: ConfigWrapper; everything AFCBuffer takes, plus the
            options of the arrival check below
        """
        self._arrived: bool = False
        super().__init__(config)
        # The last stretch of a load runs in feed assist; the feed before it
        # stops this far short of the extruder.
        self.approach_length: float = config.getfloat("approach_length", 100.0, minval=10.0)
        # Read the unit this often while a load waits for arrival.
        self.poll_interval: float = config.getfloat("poll_interval", 0.1,
                                                    minval=0.05, maxval=1.0)
        # Full for this long with the encoder still counts as arrived.
        self.still_seconds: float = config.getfloat("still_seconds", 0.3, minval=0.0)
        # An encoder reading moving less than this between reads is still.
        self.still_eps_mm: float = config.getfloat("still_eps_mm", 1.0, minval=0.0)
        # Not full and not moving for this long is a jam.
        self.stall_seconds: float = config.getfloat("stall_seconds", 3.0, minval=0.5)
        # Give up when the assist pushes this much more than approach_length.
        self.max_extra_length: float = config.getfloat("max_extra_length", 150.0, minval=0.0)
        self.arrival_timeout: float = config.getfloat("arrival_timeout", 30.0, minval=1.0)

        self.arrival_mm: Optional[float] = None
        self._watch_timer = None
        self._watch: Optional[Dict[str, Any]] = None

    # AFCBuffer's __init__ assigns these two, so they need setters.
    @property
    def advance_state(self) -> bool:
        """
        True while filament is at the extruder: from a confirmed arrival
        until the next load or unload starts, and for a lane AFC has
        recorded as loaded in its toolhead (which covers a restart, where
        the arrival itself was not seen).
        """
        if self._arrived:
            return True
        for lane in self.lanes.values():
            extruder = getattr(lane, "extruder_obj", None)
            if (getattr(lane, "tool_loaded", False)
                and getattr(extruder, "lane_loaded", None) == lane.name):
                return True
        return False

    @advance_state.setter
    def advance_state(self, value: bool) -> None:
        self._arrived = bool(value)

    @property
    def trailing_state(self) -> bool:
        """
        True while one of this buffer's lanes reads its back switch (buffer
        pulled, the filament under tension), from the unit's last sensor read.
        """
        for unit, slot in self._lane_slots():
            sensors = (getattr(unit, "_cached_sensor_state", None) or {}).get("slot_sensors")
            if (sensors
                and slot < len(sensors)
                and sensors[slot].get("buf_back")):
                return True
        return False

    @trailing_state.setter
    def trailing_state(self, value: bool) -> None:
        pass

    def _lane_slots(self) -> list:
        """
        The (unit, slot) of every lane on this buffer that sits on an ACE.

        :return list: (unit, slot) pairs
        """
        pairs = []
        for lane in self.lanes.values():
            unit = getattr(lane, "unit_obj", None)
            get_slot = getattr(unit, "_get_slot", None)
            if get_slot is None:
                continue
            try:
                pairs.append((unit, get_slot(lane.name)))
            except Exception:
                continue
        return pairs

    def enable_advance_latch(self) -> None:
        """
        A load is starting: forget the last arrival.
        """
        self._arrived = False
        self.arrival_mm = None

    def clear_advance_latch(self) -> None:
        """
        Forget the last arrival.
        """
        self._arrived = False

    def mark_arrival(self, lane: AFCLane, unit: Any, slot: int, pushed_mm: float) -> None:
        """
        Record that the filament reached the extruder, and start watching the
        buffer while the extruder takes it, for the log.

        :param lane: lane being loaded
        :param unit: its ACE unit
        :param slot: its slot on the unit
        :param pushed_mm: how far feed assist pushed before the buffer filled
        """
        self._arrived = True
        self.arrival_mm = pushed_mm
        self._start_watch(lane, unit, slot)

    def enable_buffer(self, lane: AFCLane) -> None:
        """
        The lane is loaded: end the engagement watch. The arrival stays
        latched while the lane is loaded.

        :param lane: the loaded lane
        """
        self._end_watch()
        super().enable_buffer(lane)

    def disable_buffer(self) -> None:
        """
        Disable the buffer and drop the latch; this runs as an unload starts.
        """
        self._end_watch()
        self._arrived = False
        super().disable_buffer()

    # The watch after arrival: logs how the buffer behaves while the
    # extruder takes the filament, so engagement can be judged from real
    # loads before anything acts on it.

    def _start_watch(self, lane: AFCLane, unit: Any, slot: int) -> None:
        """
        Start sampling the unit until the load finishes.

        :param lane: lane being loaded
        :param unit: its ACE unit
        :param slot: its slot on the unit
        """
        self._end_watch()
        self._watch = {"lane": lane.name, "unit": unit, "slot": slot, "full": None,
                       "drains": 0, "fills": 0, "fed_mm": 0.0, "last_dec": None,
                       "start": self.reactor.monotonic()}
        if self._watch_timer is None:
            self._watch_timer = self.reactor.register_timer(self._watch_tick)
        self.reactor.update_timer(self._watch_timer, self.reactor.NOW)

    def _watch_tick(self, eventtime: float) -> float:
        """
        Reactor timer: read the unit once and count buffer fills and drains.

        :param eventtime: reactor event time
        :return float: the next tick, or NEVER once the watch ended
        """
        watch = self._watch
        if watch is None:
            return self.reactor.NEVER
        if eventtime - watch["start"] > 120.0:
            self._end_watch()
            return self.reactor.NEVER
        sample = read_slot(watch["unit"], watch["slot"])
        if sample is not None:
            full, dec = sample["full"], sample["decoder"]
            if watch["full"] is not None and full != watch["full"]:
                watch["fills" if full else "drains"] += 1
            watch["full"] = full
            watch["fed_mm"] += decoder_step(watch["last_dec"], dec)
            watch["last_dec"] = dec
        return eventtime + self.poll_interval

    def _end_watch(self) -> None:
        """
        Stop the engagement watch and log what it saw.
        """
        watch = self._watch
        if watch is None:
            return
        self._watch = None
        if self._watch_timer is not None:
            self.reactor.update_timer(self._watch_timer, self.reactor.NEVER)
        self.logger.debug(f"{self.name}: {watch['lane']} after arrival, the buffer drained "
                          f"{watch['drains']} and refilled {watch['fills']} times while the "
                          f"assist fed {watch['fed_mm']:.0f}mm")

    def get_status(self, eventtime: Optional[float] = None) -> dict:
        """
        AFCBuffer's status plus the arrival state.

        :param eventtime: unused, Klipper passes it
        :return dict: the status
        """
        response = super().get_status(eventtime)
        response["arrived"] = self._arrived
        response["arrival_mm"] = self.arrival_mm
        return response


def decoder_step(last: Optional[int], current: int) -> float:
    """
    How far the encoder moved forward between two reads. Each feed or assist
    push starts the unit's count from zero, so a drop means a new push.

    :param last: the previous signed decoder count, or None
    :param current: this one
    :return float: mm moved forward, never negative
    """
    if last is None:
        return 0.0
    step = current - last if current >= last else current
    return float(max(step, 0))


def read_slot(unit: Any, slot: int) -> Optional[Dict[str, Any]]:
    """
    Read one slot's buffer switches and encoder from the unit.

    :param unit: the ACE unit
    :param slot: 0-based slot
    :return dict: {"full", "rest", "back", "decoder"}, or None if a read failed
    """
    ace = getattr(unit, "_ace", None)
    if (ace is None
        or not getattr(ace, "connected", False)):
        return None
    try:
        sensors = ace.get_sensor_state(timeout=0.5)
        feed = ace.send_command("get_feed_info", timeout=0.5)["feed_info"][slot]
        slot_sensors = sensors["slot_sensors"][slot]
        return {"full": bool(sensors["buf_feed"]),
                "rest": bool(slot_sensors["buf_rst"]),
                "back": bool(slot_sensors["buf_back"]),
                "decoder": signed_decoder(feed.get("decoder", 0))}
    except Exception:
        return None
