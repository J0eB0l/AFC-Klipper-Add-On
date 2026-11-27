"""
Tests for the feed an OpenAMS unit runs for a caller that looks for a lane's filament at a
sensor of its own (afcAMS.feed_to and its helpers): the lane is fed out of its bay until the
caller's check sees it, and is then unloaded back.
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

    def is_bay_ready(self, bay: int) -> bool:
        self.sent.append(("ready", bay))
        return self.ready[bay]

    def set_oams_follower(self, enable: int, direction: int) -> None:
        self.sent.append(("follower", enable, direction))

    def send(self, args: List[int]) -> None:
        self.sent.append(("load", args[0]))
        if self.send_error is not None:
            raise self.send_error
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

    def test_the_lane_is_fed_until_it_is_seen_and_unloaded_back(self):
        ams, oams, lane = make_unit()
        seen = MagicMock(side_effect=[False, False, True])
        assert ams.feed_to(lane, seen, self.GCMD) is True
        assert seen.call_count == 3
        # The follower runs forward before the load, as for a load to the toolhead, and the
        # load is canceled where it stands.
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: its filament was seen")]
        left_idle(ams)

    def test_the_load_is_marked_as_going_before_it_is_sent(self):
        ams, oams, lane = make_unit()
        going: List[Tuple[Any, Any, bool]] = []
        send = oams.send

        def record(args: List[int]) -> None:
            going.append((oams.action_status, oams.action_status_code, ams._operation_active))
            send(args)

        oams.send = record
        ams.feed_to(lane, MagicMock(return_value=True), self.GCMD)
        assert going == [(OAMSStatus.LOADING, None, True)]

    def test_a_lane_that_may_not_be_fed_is_left_alone(self):
        ams, oams, lane = make_unit()
        oams.ready[1] = False
        seen = MagicMock(return_value=True)
        assert ams.feed_to(lane, seen, self.GCMD) is False
        seen.assert_not_called()
        assert oams.sent == [("ready", 1)]
        assert ams.logger.messages == [
            ("warning", "ams1: lane1 is not fed: its bay holds no filament")]
        # Nothing ran, so nothing was reset.
        assert (oams.current_spool, ams._prev_states_stale) == (2, False)

    def test_a_unit_that_is_busy_stays_busy(self):
        ams, _oams, lane = make_unit()
        ams._operation_active = True
        assert ams.feed_to(lane, MagicMock(return_value=True), self.GCMD) is False
        assert ams.logger.messages == [("warning", "ams1: lane1 is not fed: the unit is busy")]
        assert ams._operation_active is True

    def test_filament_at_an_extruder_ends_the_feed_unseen(self):
        ams, oams, lane = make_unit()
        oams.fps_value = 0.45
        seen = MagicMock(return_value=False)
        assert ams.feed_to(lane, seen, self.GCMD) is False
        assert seen.call_count == 1
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: the FPS reads 0.45, filament at an "
                      "extruder")]
        left_idle(ams)

    def test_a_check_that_sees_the_lane_wins_over_the_fps(self):
        ams, oams, lane = make_unit()
        oams.fps_value = 0.9
        assert ams.feed_to(lane, MagicMock(return_value=True), self.GCMD) is True
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: its filament was seen")]

    @pytest.mark.parametrize("fps", [None, "n/a", 0.44])
    def test_no_fps_reading_and_a_low_one_do_not_end_the_feed(self, fps):
        ams, oams, lane = make_unit()
        oams.fps_value = fps
        seen = MagicMock(side_effect=[False, False, True])
        assert ams.feed_to(lane, seen, self.GCMD) is True
        assert seen.call_count == 3

    @pytest.mark.parametrize("code, name", [(0, "success"), (4, "no spool in bay"),
                                            (None, "code None")])
    def test_a_load_the_unit_ended_itself_is_not_canceled_and_is_unloaded(self, code, name):
        ams, oams, lane = make_unit()
        # The load ran to its end, or was refused, before anything saw the lane.
        oams.ends, oams.ends_code = True, code
        seen = MagicMock(return_value=False)
        assert ams.feed_to(lane, seen, self.GCMD) is False
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
        assert ams.feed_to(lane, seen, self.GCMD) is False
        # It is looked for every tenth of a second, the last time 2.0 s into the load.
        assert seen.call_count == 21
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED
        assert ams.logger.messages == [
            ("debug", "ams1: feed of lane1 stopped: nothing saw it within 2s")]
        left_idle(ams)

    def test_a_check_that_fails_still_has_the_lane_unloaded(self):
        ams, oams, lane = make_unit()
        seen = MagicMock(side_effect=RuntimeError("the switch is gone"))
        with pytest.raises(RuntimeError) as err:
            ams.feed_to(lane, seen, self.GCMD)
        assert str(err.value) == "the switch is gone"
        assert oams.sent == [("ready", 1)] + FOLLOWER + [("load", 1)] + STOPPED
        assert ams.logger.messages == []
        left_idle(ams)

    def test_a_load_that_could_not_be_sent_leaves_nothing_to_unload(self):
        ams, oams, lane = make_unit()
        oams.send_error = RuntimeError("the unit is gone")
        seen = MagicMock(return_value=True)
        with pytest.raises(RuntimeError) as err:
            ams.feed_to(lane, seen, self.GCMD)
        assert str(err.value) == "the unit is gone"
        seen.assert_not_called()
        # The load it was marked for is canceled, and nothing was fed.
        assert oams.sent == [("ready", 1)] + FOLLOWER + [
            ("load", 1), ("follower", 0, 0), ("cancel",)]
        left_idle(ams)


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
