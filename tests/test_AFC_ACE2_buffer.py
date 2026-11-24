"""Unit tests for extras/AFC_ACE2_buffer.py."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from extras.AFC_ACE2_buffer import (AFCACEBuffer, decoder_step, read_slot, signed_decoder,
                                    slot_full)
from extras.AFC_buffer import load_config_prefix
from tests.conftest import MockAFC, MockConfig, MockPrinter


def make_buffer(**values: Any) -> AFCACEBuffer:
    """
    :param values: [AFC_buffer] options beyond type: ace
    :return AFCACEBuffer: a buffer built through its real __init__
    """
    printer = MockPrinter(afc=MockAFC())
    config = MockConfig(name="AFC_buffer Ace_Buffer", printer=printer,
                        values={"type": "ace", **values})
    return AFCACEBuffer(config)


class SensorLink:
    """An ACE connection answering the two reads the buffer makes."""

    def __init__(self, sensors: Any, feed: Any, connected: bool = True) -> None:
        self.sensors = sensors
        self.feed = feed
        self.connected = connected
        self.calls: List[Tuple[str, Optional[float]]] = []

    def get_sensor_state(self, timeout: float = 3.0) -> Any:
        self.calls.append(("get_sensor_state", timeout))
        if isinstance(self.sensors, Exception):
            raise self.sensors
        return self.sensors

    def send_command(self, method: str, timeout: float = 3.0) -> Any:
        self.calls.append((method, timeout))
        return self.feed


class SlotUnit:
    """An ACE unit with a slot per lane name and a cached sensor read."""

    def __init__(self, slots: Dict[str, int], cached: Optional[Dict[str, Any]] = None,
                 ace: Any = None) -> None:
        self.slots = slots
        self._cached_sensor_state = cached or {}
        self._ace = ace

    def _get_slot(self, name: str) -> int:
        return self.slots[name]


class BufExtruder:
    """An extruder as the buffer reads it."""

    def __init__(self) -> None:
        self.lane_loaded: Optional[str] = None


class BufLane:
    """A lane as the buffer reads it."""

    def __init__(self, name: str, unit: Any) -> None:
        self.name = name
        self.unit_obj = unit
        self.extruder_stepper = None
        self.extruder_obj = BufExtruder()
        self.tool_loaded = False


# slot 0 at rest, slot 1 pulled back, slot 2 pressed forward onto the shared full switch
SENSORS = {"slot_sensors": [{"buf_rst": True, "buf_back": False},
                            {"buf_rst": False, "buf_back": True},
                            {"buf_rst": False, "buf_back": False},
                            {"buf_rst": True, "buf_back": False}],
           "buf_feed": True}


class TestHelpers:
    def test_signed_decoder_unwraps_a_backwards_count(self):
        assert signed_decoder(18446744073709549711) == -1905
        assert signed_decoder(1843) == 1843

    def test_decoder_step(self):
        assert decoder_step(None, 40) == 0.0
        assert decoder_step(20, 45) == 25.0
        # a new push counts from zero again
        assert decoder_step(25, 6) == 6.0

    def test_slot_full_needs_the_slots_own_buffer_pressed_forward(self):
        assert slot_full(SENSORS, 2) is True
        # at rest or pulled back: another slot holds the shared switch
        assert slot_full(SENSORS, 0) is False
        assert slot_full(SENSORS, 1) is False
        assert slot_full(dict(SENSORS, buf_feed=False), 2) is False
        assert slot_full(SENSORS, 4) is False
        assert slot_full({"buf_feed": True}, 0) is False

    def test_read_slot(self):
        link = SensorLink(SENSORS, {"feed_info": [{}, {"decoder": 18446744073709551613,
                                                       "length": 5}]})
        unit = SlotUnit({}, ace=link)

        assert read_slot(unit, 1) == {"full": False, "rest": False, "back": True,
                                      "decoder": -3, "length": 5.0}
        assert link.calls == [("get_sensor_state", 0.5), ("get_feed_info", 0.5)]

    def test_read_slot_reads_full_only_for_the_slot_pressed_forward(self):
        link = SensorLink(SENSORS, {"feed_info": [{"decoder": 4}, {}, {"decoder": 7}]})
        unit = SlotUnit({}, ace=link)

        # A slot the unit gives no length for reads 0.
        assert read_slot(unit, 2) == {"full": True, "rest": False, "back": False,
                                      "decoder": 7, "length": 0.0}
        assert read_slot(unit, 0) == {"full": False, "rest": True, "back": False,
                                      "decoder": 4, "length": 0.0}

    def test_read_slot_failures_read_none(self):
        assert read_slot(SlotUnit({}), 0) is None
        assert read_slot(SlotUnit({}, ace=SensorLink(SENSORS, {}, connected=False)), 0) is None
        assert read_slot(SlotUnit({}, ace=SensorLink(RuntimeError("x"), {})), 0) is None


class TestAFCACEBuffer:
    def test_load_config_prefix_builds_it(self):
        printer = MockPrinter(afc=MockAFC())
        config = MockConfig(name="AFC_buffer Ace_Buffer", printer=printer,
                            values={"type": "ace"})

        buf = load_config_prefix(config)

        assert isinstance(buf, AFCACEBuffer)
        assert buf.type == "ace"
        assert printer.lookup_object("AFC").buffers["Ace_Buffer"] is buf

    def test_defaults(self):
        buf = make_buffer()

        assert (buf.approach_length, buf.poll_interval, buf.still_seconds, buf.still_eps_mm,
                buf.stall_seconds, buf.max_extra_length, buf.arrival_timeout,
                buf.engage_extra_seconds, buf.engage_push_length, buf.engage_retries) == (
            100.0, 0.1, 0.3, 1.0, 3.0, 150.0, 30.0, 2.0, 50.0, 3)
        assert buf.advance_state is False

    def test_arrival_latches_until_the_next_load(self):
        buf = make_buffer()
        unit = SlotUnit({"lane0": 0})
        lane = BufLane("lane0", unit)

        buf.mark_arrival(lane, unit, 0, 45.0)
        assert buf.advance_state is True
        assert buf.arrival_mm == 45.0

        buf.enable_advance_latch()
        assert buf.advance_state is False
        assert buf.arrival_mm is None

    def test_enable_keeps_the_latch_and_disable_drops_it(self):
        buf = make_buffer()
        unit = SlotUnit({"lane0": 0})
        lane = BufLane("lane0", unit)

        buf.mark_arrival(lane, unit, 0, 45.0)
        buf.enable_buffer(lane)
        assert buf.advance_state is True

        buf.mark_arrival(lane, unit, 0, 45.0)
        buf.disable_buffer()
        assert buf.advance_state is False

        buf.mark_arrival(lane, unit, 0, 45.0)
        buf.clear_advance_latch()
        assert buf.advance_state is False

    def test_a_lane_recorded_in_its_toolhead_reads_arrived(self):
        buf = make_buffer()
        lane = BufLane("lane0", SlotUnit({"lane0": 0}))
        other = BufLane("lane1", SlotUnit({"lane1": 1}))
        buf.lanes = {"lane0": lane}
        other.tool_loaded = True
        other.extruder_obj.lane_loaded = "lane1"
        assert buf.advance_state is False

        lane.tool_loaded = True
        assert buf.advance_state is False

        lane.extruder_obj.lane_loaded = "lane0"
        assert buf.advance_state is True

    def test_trailing_until_the_buffer_reads_full(self):
        buf = make_buffer()
        unit = SlotUnit({"lane2": 2}, cached=SENSORS)
        buf.lanes = {"lane2": BufLane("lane2", unit)}
        assert buf.trailing_state is False

        unit._cached_sensor_state = dict(SENSORS, buf_feed=False)
        assert buf.trailing_state is True

    def test_trailing_while_another_slot_holds_the_full_switch(self):
        buf = make_buffer()
        unit = SlotUnit({"lane0": 0}, cached=SENSORS)
        buf.lanes = {"lane0": BufLane("lane0", unit)}

        assert buf.trailing_state is True

    def test_trailing_with_no_sensor_read(self):
        buf = make_buffer()
        buf.lanes = {"lane0": BufLane("lane0", SlotUnit({"lane0": 0}))}

        assert buf.trailing_state is True

    def test_state_name_for_the_gui(self):
        buf = make_buffer()
        unit = SlotUnit({"lane0": 0, "lane1": 1, "lane2": 2})
        buf.lanes = {"lane0": BufLane("lane0", unit)}
        assert buf.last_state == "Unknown"
        assert buf.get_status()["state"] == "Unknown"

        unit._cached_sensor_state = SENSORS
        # at rest: the shared full switch belongs to another slot
        assert buf.last_state == "Neutral"
        assert buf.get_status()["state"] == "Neutral"

        buf.lanes = {"lane2": BufLane("lane2", unit)}
        assert buf.last_state == "Trailing"

        unit._cached_sensor_state = dict(SENSORS, buf_feed=False)
        assert buf.last_state == "Neutral"

        buf.lanes = {"lane1": BufLane("lane1", unit)}
        assert buf.last_state == "Advancing"

    def test_status_carries_the_arrival(self):
        buf = make_buffer()
        unit = SlotUnit({"lane0": 0})

        buf.mark_arrival(BufLane("lane0", unit), unit, 0, 45.0)
        status = buf.get_status()

        assert (status["arrived"], status["arrival_mm"]) == (True, 45.0)
