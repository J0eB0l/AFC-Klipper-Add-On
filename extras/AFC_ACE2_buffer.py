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

try: from extras.AFC_buffer import (AFCBuffer, ADVANCING_STATE_NAME, NEUTRAL_STATE_NAME,
                                   TRAILING_STATE_NAME)
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
    full switch is on and the unit's encoder has stopped. Another lane resting
    full in its own toolhead holds the shared switch on, so it only counts for
    a slot whose own buffer is pressed forward (see `slot_full`).
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
        # After arrival the buffer must drain within the tool_stn move plus this.
        self.engage_extra_seconds: float = config.getfloat("engage_extra_seconds", 2.0,
                                                           minval=0.0)
        # Still full after that: push this far again while the extruder turns, this many times.
        self.engage_push_length: float = config.getfloat("engage_push_length", 50.0, minval=1.0)
        self.engage_retries: int = config.getint("engage_retries", 3, minval=0)

        self.arrival_mm: Optional[float] = None

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
        True unless this buffer reads full (the filament pressed forward),
        from the unit's last sensor read. An unload backs off until then
        before it retracts. Filament that never reached the gears is not held,
        so the buffer can never come under tension, and an unread buffer must
        not block an unload either.
        """
        for unit, slot in self._lane_slots():
            sensors = getattr(unit, "_cached_sensor_state", None) or {}
            if slot_full(sensors, slot):
                return False
        return True

    @trailing_state.setter
    def trailing_state(self, value: bool) -> None:
        pass

    @property
    def last_state(self) -> str:
        """
        The buffer state the GUI shows, read from the unit's last sensor
        read: Trailing while full (pressed forward), Advancing while pulled
        under tension, Neutral otherwise.
        """
        read = False
        for unit, slot in self._lane_slots():
            sensors = getattr(unit, "_cached_sensor_state", None) or {}
            slot_sensors = sensors.get("slot_sensors") or []
            if slot >= len(slot_sensors):
                continue
            read = True
            if slot_sensors[slot].get("buf_back"):
                return ADVANCING_STATE_NAME
            if slot_full(sensors, slot):
                return TRAILING_STATE_NAME
        return NEUTRAL_STATE_NAME if read else "Unknown"

    @last_state.setter
    def last_state(self, value: str) -> None:
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
        Record that the filament reached the extruder.

        :param lane: lane being loaded
        :param unit: its ACE unit
        :param slot: its slot on the unit
        :param pushed_mm: how far feed assist pushed before the buffer filled
        """
        self._arrived = True
        self.arrival_mm = pushed_mm

    def disable_buffer(self) -> None:
        """
        Disable the buffer and drop the latch; this runs as an unload starts.
        """
        self._arrived = False
        super().disable_buffer()

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


def slot_full(sensors: Dict[str, Any], slot: int) -> bool:
    """
    Whether one slot's buffer is full. The full switch is shared by the
    unit's four buffers, so it counts for a slot only while that slot's own
    buffer is pressed forward: off its rest switch and not pulled back.

    :param sensors: a get_sensor_state result
    :param slot: 0-based slot
    :return bool: True when the full switch is on and this slot's buffer is pressed forward
    """
    slot_sensors = sensors.get("slot_sensors") or []
    if (not sensors.get("buf_feed")
        or slot >= len(slot_sensors)):
        return False
    return (not slot_sensors[slot].get("buf_rst")
            and not slot_sensors[slot].get("buf_back"))


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
        return {"full": slot_full(sensors, slot),
                "rest": bool(slot_sensors["buf_rst"]),
                "back": bool(slot_sensors["buf_back"]),
                "decoder": signed_decoder(feed.get("decoder", 0))}
    except Exception:
        return None
