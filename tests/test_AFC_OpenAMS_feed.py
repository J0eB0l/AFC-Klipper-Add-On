"""
Tests for the feed an OpenAMS unit runs for a caller that looks for a lane's filament at a
sensor of its own (afcAMS.feed_to, afcAMS.measure_to and their helpers): the lane is fed out of
its bay until the caller's check sees it, and is then unloaded back. A feed that is measured
gives the way the lane went by the encoder.
"""

import sys
import types
from typing import Any, List, Optional, Tuple
from unittest.mock import MagicMock

import pytest

# Match the defensive stubs the sibling OpenAMS test modules install.
_mcu_stub = types.ModuleType("mcu")
_mcu_stub.get_printer_mcu = MagicMock()
sys.modules.setdefault("mcu", _mcu_stub)
_bus_stub = types.ModuleType("extras.bus")
_bus_stub.MCU_I2C_from_config = MagicMock()
sys.modules.setdefault("extras.bus", _bus_stub)

from extras.AFC_OpenAMS import afcAMS, OAMSStatus  # noqa: E402
from tests.conftest import MockAFC, MockConfig, MockGCodeCommand, MockPrinter  # noqa: E402

# What the unit is sent for a feed that ran and was stopped while its load was still going.
FOLLOWER = [("follower", 0, 0), ("follower", 1, 1)]
STOPPED = [("follower", 0, 0), ("cancel",), ("unload",), ("clear",)]


class AdvancingReactor:
    """A reactor whose pause() moves its clock on, so a timed loop ends."""

    NEVER = 9_999_999_999.0
    NOW = 0.0

    def __init__(self) -> None:
        self.now = 100.0

    def monotonic(self) -> float:
        return self.now

    def pause(self, until: float) -> None:
        self.now = max(self.now, until)


class FakeOams:
    """
    An OAMS that keeps a load going until it is canceled and records what it is sent, in
    order.
    """

    def __init__(self) -> None:
        self.sent: List[Tuple[Any, ...]] = []
        self.oams_load_spool_cmd = self
        self.action_status: Optional[int] = None
        self.action_status_code: Optional[int] = None
        self.current_spool: Optional[int] = 2
        self.fps_value: Any = 0.0
        self.hub_hes_value = [0, 0, 0, 0]
        # Whether a bay holds filament, by bay.
        self.ready = [True, True, True, True]
        # What an unload answers, or the error it fails with.
        self.unloads: Any = (True, "ok")
        # The errors a load command and a clear fail with, None for none.
        self.send_error: Optional[Exception] = None
        self.clear_error: Optional[Exception] = None
        # Whether the unit ends a load by itself as soon as it is sent, and the code it
        # reports for it.
        self.ends = False
        self.ends_code: Optional[int] = None
        # The encoder count, and the clicks a load turns it by as soon as it is sent.
        self.encoder_clicks = 5000
        self.turns = 0

    def is_bay_ready(self, bay: int) -> bool:
        self.sent.append(("ready", bay))
        return self.ready[bay]

    def set_oams_follower(self, enable: int, direction: int) -> None:
        self.sent.append(("follower", enable, direction))

    def send(self, args: List[int]) -> None:
        self.sent.append(("load", args[0]))
        if self.send_error is not None:
            raise self.send_error
        self.encoder_clicks += self.turns
        if self.ends:
            self.action_status = None
            self.action_status_code = self.ends_code

    def load_spool_cancel(self) -> None:
        self.sent.append(("cancel",))
        self.action_status = None

    def unload_spool(self) -> Tuple[bool, str]:
        self.sent.append(("unload",))
        if isinstance(self.unloads, Exception):
            raise self.unloads
        return self.unloads

    def clear_errors(self) -> None:
        self.sent.append(("clear",))
        if self.clear_error is not None:
            raise self.clear_error


class FakeLane:
    """A lane with only what the unit reads before it feeds one."""

    def __init__(self, name: str = "lane1") -> None:
        self.name = name
        self.tool_loaded = False


def make_unit() -> Tuple[afcAMS, FakeOams, FakeLane]:
    """:return tuple: a unit whose lane1 is in bay 1, its OAMS, and that lane"""
    afc = MockAFC()
    afc.reactor = AdvancingReactor()
    afc.function.in_print = lambda: False
    printer = MockPrinter(afc=afc)
    printer.reactor = afc.reactor
    ams = afcAMS(MockConfig(name="AFC_OpenAMS ams1", printer=printer, values={}))
    ams.afc = afc
    ams.oams = FakeOams()
    lane = FakeLane()
    ams.lanes = {"lane1": lane}
    ams._spool_map = {"lane1": 1}
    # Without a follower controller the follower is driven with the raw command.
    ams._follower = None
    ams._prev_states_stale = False
    return ams, ams.oams, lane


def left_idle(ams: afcAMS) -> None:
    """Check a feed that ran left the unit free, with no spool loaded."""
    assert ams.oams.current_spool is None
    assert ams.oams.action_status is None
    assert ams._operation_active is False
    assert ams._prev_states_stale is True


class TestAfcAMSFeedToBlocked:
    def test_a_lane_with_filament_in_its_bay_may_be_fed(self):
        ams, oams, lane = make_unit()
        assert ams._feed_to_blocked(lane) is None
        assert oams.sent == [("ready", 1)]

    def test_a_unit_with_no_hardware(self):
        ams, _oams, lane = make_unit()
        ams.oams = None
        assert ams._feed_to_blocked(lane) == "its OAMS hardware is not available"

    def test_a_unit_that_runs_something_else(self):
        ams, oams, lane = make_unit()
        ams._operation_active = True
        assert ams._feed_to_blocked(lane) == "the unit is busy"
        assert oams.sent == []

    def test_a_lane_that_has_no_bay(self):
        ams, oams, _lane = make_unit()
        assert ams._feed_to_blocked(FakeLane("lane9")) == "it is not mapped to a bay"
        assert oams.sent == []

    def test_a_bay_that_holds_no_filament(self):
        ams, oams, lane = make_unit()
        oams.ready[1] = False
        assert ams._feed_to_blocked(lane) == "its bay holds no filament"

    def test_a_lane_that_a_scan_feed_would_not_move_either(self):
        ams, _oams, lane = make_unit()
        lane.tool_loaded = True
        assert ams._feed_to_blocked(lane) == "lane1 is loaded to the toolhead"


class TestAfcAMSFeedTo:
    GCMD = MockGCodeCommand()

    @pytest.mark.parametrize("found", [True, False])
    def test_it_says_whether_the_feed_was_seen_and_measures_nothing(self, found):
        ams, _oams, lane = make_unit()
        ams._feed_out = MagicMock(return_value=(found, None))
        seen = MagicMock()
        assert ams.feed_to(lane, seen, self.GCMD) is found
        ams._feed_out.assert_called_once_with(lane, seen, False)


class TestAfcAMSMeasureTo:
    GCMD = MockGCodeCommand()

    def test_the_clicks_the_feed_turned_are_given_in_mm(self):
        ams, _oams, lane = make_unit()
        ams._feed_out = MagicMock(return_value=(True, 1400))
        seen = MagicMock()
        # 1.75 clicks to the mm: 1400 clicks are 800 mm.
        assert ams.measure_to(lane, seen, self.GCMD) == 800.0
        ams._feed_out.assert_called_once_with(lane, seen, True)
        assert ams.logger.messages == [
            ("info", "ams1: lane1 went 1400 encoder clicks, 800 mm")]

    def test_a_lane_nothing_saw_that_got_to_an_extruder_has_a_length_too(self):
        ams, _oams, lane = make_unit()
        ams._feed_out = MagicMock(return_value=(False, 350))
        assert ams.measure_to(lane, MagicMock(), self.GCMD) == 200.0
        assert ams.logger.messages == [
            ("info", "ams1: lane1 went 350 encoder clicks, 200 mm")]

    def test_a_feed_that_gave_no_clicks_gives_no_length(self):
        ams, _oams, lane = make_unit()
        ams._feed_out = MagicMock(return_value=(True, None))
        assert ams.measure_to(lane, MagicMock(), self.GCMD) is None
        assert ams.logger.messages == []


class TestAfcAMSFeedOut:
    # What the unit is sent for a feed whose clicks were read before it was unloaded.
    READ = [("follower", 0, 0), ("cancel",), ("clicks",), ("unload",), ("clear",)]

    @staticmethod
    def reads(ams: afcAMS, oams: FakeOams, clicks: Optional[int] = 1200) -> MagicMock:
        """:return MagicMock: put in place of the read of the encoder, which it records"""
        ams._fed_clicks = MagicMock(
            side_effect=lambda lane, start: oams.sent.append(("clicks",)) or clicks)
        return ams._fed_clicks

    def test_the_lane_is_fed_until_it_is_seen_and_unloaded_back(self):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams)
        seen = MagicMock(side_effect=[False, False, True])
        assert ams._feed_out(lane, seen, False) == (True, None)
        assert seen.call_count == 3
        # The follower runs forward before the load, as for a load to the toolhead, and the
        # load is canceled where it stands.
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: its filament was seen")]
        # A feed that is not measured does not wait for the encoder.
        read.assert_not_called()
        left_idle(ams)

    def test_the_load_is_marked_as_going_before_it_is_sent(self):
        ams, oams, lane = make_unit()
        going: List[Tuple[Any, Any, bool]] = []
        send = oams.send

        def record(args: List[int]) -> None:
            going.append((oams.action_status, oams.action_status_code, ams._operation_active))
            send(args)

        oams.send = record
        ams._feed_out(lane, MagicMock(return_value=True), False)
        assert going == [(OAMSStatus.LOADING, None, True)]

    def test_a_lane_that_may_not_be_fed_is_left_alone(self):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams)
        oams.ready[1] = False
        seen = MagicMock(return_value=True)
        assert ams._feed_out(lane, seen, True) == (False, None)
        seen.assert_not_called()
        read.assert_not_called()
        assert oams.sent == [("ready", 1)]
        assert ams.logger.messages == [
            ("warning", "ams1: lane1 is not fed: its bay holds no filament")]
        # Nothing ran, so nothing was reset.
        assert (oams.current_spool, ams._prev_states_stale) == (2, False)

    def test_a_unit_that_is_busy_stays_busy(self):
        ams, _oams, lane = make_unit()
        ams._operation_active = True
        assert ams._feed_out(lane, MagicMock(return_value=True), False) == (False, None)
        assert ams.logger.messages == [("warning", "ams1: lane1 is not fed: the unit is busy")]
        assert ams._operation_active is True

    def test_filament_at_an_extruder_ends_the_feed_unseen(self):
        ams, oams, lane = make_unit()
        oams.fps_value = 0.45
        seen = MagicMock(return_value=False)
        assert ams._feed_out(lane, seen, False) == (False, None)
        assert seen.call_count == 1
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: the FPS reads 0.45, filament at an "
                      "extruder")]
        left_idle(ams)

    def test_a_check_that_sees_the_lane_wins_over_the_fps(self):
        ams, oams, lane = make_unit()
        oams.fps_value = 0.9
        assert ams._feed_out(lane, MagicMock(return_value=True), False) == (True, None)
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: its filament was seen")]

    @pytest.mark.parametrize("fps", [None, "n/a", 0.44])
    def test_no_fps_reading_and_a_low_one_do_not_end_the_feed(self, fps):
        ams, oams, lane = make_unit()
        oams.fps_value = fps
        seen = MagicMock(side_effect=[False, False, True])
        assert ams._feed_out(lane, seen, False) == (True, None)
        assert seen.call_count == 3

    @pytest.mark.parametrize("code, name", [(0, "success"), (4, "no spool in bay"),
                                            (None, "code None")])
    def test_a_load_the_unit_ended_itself_is_not_canceled_and_is_unloaded(self, code, name):
        ams, oams, lane = make_unit()
        # The load ran to its end, or was refused, before anything saw the lane.
        oams.ends, oams.ends_code = True, code
        seen = MagicMock(return_value=False)
        assert ams._feed_out(lane, seen, False) == (False, None)
        assert seen.call_count == 1
        assert oams.sent == [("ready", 1)] + FOLLOWER + [
            ("load", 1), ("follower", 0, 0), ("unload",), ("clear",)]
        assert ams.logger.messages == [
            ("debug", f"ams1: feed of lane1 stopped: the unit ended its load ({name})")]
        left_idle(ams)

    def test_a_lane_nothing_saw_is_fed_no_longer_than_the_timeout(self):
        ams, oams, lane = make_unit()
        ams.FEED_TO_TIMEOUT = 2.05
        seen = MagicMock(return_value=False)
        assert ams._feed_out(lane, seen, False) == (False, None)
        # It is looked for every tenth of a second, the last time 2.0 s into the load.
        assert seen.call_count == 21
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: nothing saw it within 2s")]
        left_idle(ams)

    def test_a_check_that_fails_still_has_the_lane_unloaded(self):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams)
        seen = MagicMock(side_effect=RuntimeError("the switch is gone"))
        with pytest.raises(RuntimeError) as err:
            ams._feed_out(lane, seen, True)
        assert str(err.value) == "the switch is gone"
        # The lane got nowhere that is known, so its clicks are not read.
        read.assert_not_called()
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED
        assert ams.logger.messages == []
        left_idle(ams)

    def test_a_load_that_could_not_be_sent_leaves_nothing_to_unload(self):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams)
        oams.send_error = RuntimeError("the unit is gone")
        seen = MagicMock(return_value=True)
        with pytest.raises(RuntimeError) as err:
            ams._feed_out(lane, seen, True)
        assert str(err.value) == "the unit is gone"
        seen.assert_not_called()
        read.assert_not_called()
        # The load it was marked for is canceled, and nothing was fed.
        assert oams.sent == [("ready", 1)] + FOLLOWER + [
            ("load", 1), ("follower", 0, 0), ("cancel",)]
        left_idle(ams)

    def test_a_measured_feed_that_was_seen_reads_its_clicks_before_it_is_unloaded(self):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams, 1200)
        # The encoder stood at 5000 before the load, which the load itself moves on.
        oams.turns = 900
        assert ams._feed_out(lane, MagicMock(return_value=True), True) == (True, 1200)
        read.assert_called_once_with(lane, 5000)
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + self.READ
        left_idle(ams)

    def test_a_measured_feed_that_got_to_an_extruder_unseen_reads_its_clicks(self):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams, 640)
        oams.fps_value = 0.45
        assert ams._feed_out(lane, MagicMock(return_value=False), True) == (False, 640)
        read.assert_called_once_with(lane, 5000)
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + self.READ

    def test_a_measured_feed_nothing_saw_in_time_reads_no_clicks(self):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams)
        ams.FEED_TO_TIMEOUT = 0.35
        assert ams._feed_out(lane, MagicMock(return_value=False), True) == (False, None)
        read.assert_not_called()
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED

    def test_a_measured_load_the_unit_ended_well_is_at_an_extruder(self):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams, 1810)
        oams.ends, oams.ends_code = True, 0
        assert ams._feed_out(lane, MagicMock(return_value=False), True) == (False, 1810)
        read.assert_called_once_with(lane, 5000)
        assert oams.sent == [("ready", 1)] + FOLLOWER + [
            ("load", 1), ("follower", 0, 0), ("clicks",), ("unload",), ("clear",)]

    @pytest.mark.parametrize("code", [4, None])
    def test_a_measured_load_the_unit_ended_any_other_way_reads_no_clicks(self, code):
        ams, oams, lane = make_unit()
        read = self.reads(ams, oams)
        oams.ends, oams.ends_code = True, code
        assert ams._feed_out(lane, MagicMock(return_value=False), True) == (False, None)
        read.assert_not_called()

    def test_a_feed_whose_encoder_did_not_come_to_rest_is_still_unloaded(self):
        ams, oams, lane = make_unit()
        self.reads(ams, oams, None)
        assert ams._feed_out(lane, MagicMock(return_value=True), True) == (True, None)
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + self.READ
        left_idle(ams)

    def test_the_clicks_are_the_ones_the_encoder_turned_since_before_the_load(self):
        ams, oams, lane = make_unit()
        oams.turns = 1470
        assert ams._feed_out(lane, MagicMock(return_value=True), True) == (True, 1470)
        # The encoder is asked whether it is at rest three times before it is read.
        probes = [("follower", 0, 0)] * 3
        assert oams.sent == [("ready", 1)] + FOLLOWER + [
            ("load", 1), ("follower", 0, 0), ("cancel",)] + probes + [("unload",), ("clear",)]
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: its filament was seen")]


class TestAfcAMSFedClicks:
    def test_the_clicks_are_read_once_the_encoder_is_at_rest(self):
        ams, oams, lane = make_unit()
        ams._rfid_wait_for_unit_ready = MagicMock(return_value=True)
        oams.encoder_clicks = 6200
        assert ams._fed_clicks(lane, 5000) == 1200
        # It is given 8 s to come to rest, for the second a scan feed is given to settle.
        ams._rfid_wait_for_unit_ready.assert_called_once_with(8.0, quiet_time=1.0)
        assert ams.logger.messages == []

    def test_an_encoder_that_counts_down_gives_the_same_clicks(self):
        ams, oams, lane = make_unit()
        ams._rfid_wait_for_unit_ready = MagicMock(return_value=True)
        oams.encoder_clicks = 3800
        assert ams._fed_clicks(lane, 5000) == 1200

    def test_an_encoder_that_does_not_come_to_rest_gives_no_clicks(self):
        ams, oams, lane = make_unit()
        ams._rfid_wait_for_unit_ready = MagicMock(return_value=False)
        oams.encoder_clicks = 6200
        assert ams._fed_clicks(lane, 5000) is None
        assert ams.logger.messages == [
            ("warning", "ams1: the encoder did not come to rest after the feed of lane1, so it "
                        "gives no length")]


class TestAfcAMSUnloadAfterFeed:
    def test_the_lane_is_unloaded_once_the_unit_is_idle_and_errors_are_cleared(self):
        ams, oams, lane = make_unit()
        ams._wait_for_idle = MagicMock(side_effect=lambda: oams.sent.append(("idle",)))
        ams._unload_after_feed(lane)
        assert oams.sent == [("idle",), ("unload",), ("clear",)]
        assert ams.logger.messages == []

    def test_an_unload_the_unit_refused_is_warned_of(self):
        ams, oams, lane = make_unit()
        oams.unloads = (False, "the spool is jammed")
        ams._unload_after_feed(lane)
        assert oams.sent == [("unload",), ("clear",)]
        assert ams.logger.messages == [
            ("warning", "ams1: lane1 did not unload after its feed: the spool is jammed")]

    def test_so_is_an_unload_that_fails(self):
        ams, oams, lane = make_unit()
        oams.unloads = RuntimeError("no answer")
        ams._unload_after_feed(lane)
        assert oams.sent == [("unload",), ("clear",)]
        assert ams.logger.messages == [
            ("warning", "ams1: lane1 did not unload after its feed: no answer")]

    def test_errors_that_cannot_be_cleared_are_only_noted(self):
        ams, oams, lane = make_unit()
        oams.clear_error = RuntimeError("no answer")
        ams._unload_after_feed(lane)
        assert ams.logger.messages == [
            ("debug", "ams1: errors not cleared after the feed of lane1: no answer")]
