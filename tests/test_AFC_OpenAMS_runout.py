"""
Unit tests for afcAMS runout handling in extras/AFC_OpenAMS.py

Style: typed fakes (tests/openams_helpers.py) instead of MagicMock, full
state verification after every call, and branch-complete coverage:

  handle_runout       , all 4 branches (no runout lane / unresolvable target
                         / same-extruder seamless / cross-extruder defer)
  _is_same_extruder   , all miss branches + case/whitespace matching
  _resolve_lane_reference, empty / exact / case-insensitive / miss
  _should_block_sensor_for_runout, every combination of the active-runout
                         condition plus the exception path
  FollowerController  , constructor contract (the e1a2da0 wrong-args bug)
"""

from __future__ import annotations

from extras.AFC_OpenAMS import afcAMS, FollowerController, FollowerState
from extras.AFC_lane import AFCLaneState

from tests.openams_helpers import (
    FakeAFC,
    FakeExtruderObj,
    FakeLane,
    FakeLogger,
    FakeReactor,
    Recorder,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_unit(lanes=()):
    unit = afcAMS.__new__(afcAMS)
    unit.name = "AMS_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    for lane in lanes:
        unit.afc.lanes[lane.name] = lane
    unit.lane_not_ready = Recorder()
    # The runout paths moved onto lane_unloaded upstream: the spool is still
    # in the lane, so its illumination stays on. Stub it too, or the real one
    # runs against a FakeLane with no led attributes.
    unit.lane_unloaded = Recorder()
    unit.handle_same_fps_reload = Recorder()
    return unit


def _lane(name, extruder="extruder", runout_lane=None, **kw):
    return FakeLane(name, extruder_obj=FakeExtruderObj(extruder),
                    runout_lane=runout_lane, **kw)


# ── handle_runout: branch 1, no runout lane configured ───────────────────────

def test_runout_without_runout_lane_pauses_and_handles():
    lane = _lane("lane0", runout_lane=None)
    unit = _make_unit([lane])

    handled = unit.handle_runout(lane)

    # Return + every state mutation
    assert handled is True
    assert lane.status == AFCLaneState.NONE
    assert unit.lane_unloaded.calls == [((lane,), {})]
    assert unit.afc.error.AFC_error.call_count == 1
    msg = unit.afc.error.AFC_error.last_args[0]
    assert "Runout detected on OAMS lane0" in msg
    assert "No runout lane configured" in msg
    assert unit.afc.error.AFC_error.last_kwargs == {"pause": True}
    # Nothing else fired, no flags set
    assert not unit.handle_same_fps_reload.called
    assert lane._oams_runout_detected is False
    assert lane._oams_runout_empty is False


# ── handle_runout: branch 2, runout lane doesn't resolve ─────────────────────

def test_runout_with_unresolvable_target_pauses_and_handles():
    lane = _lane("lane0", runout_lane="lane_missing")
    unit = _make_unit([lane])

    handled = unit.handle_runout(lane)

    assert handled is True
    assert lane.status == AFCLaneState.NONE
    assert unit.lane_unloaded.calls == [((lane,), {})]
    assert unit.afc.error.AFC_error.call_count == 1
    assert "lane_missing" in unit.afc.error.AFC_error.last_args[0]
    assert "not found" in unit.afc.error.AFC_error.last_args[0]
    assert unit.afc.error.AFC_error.last_kwargs == {"pause": True}
    assert not unit.handle_same_fps_reload.called
    assert lane._oams_runout_empty is False


# ── handle_runout: branch 3, same extruder, seamless reload ──────────────────

def test_same_extruder_runout_does_seamless_reload():
    lane = _lane("lane0", extruder="extruder", runout_lane="lane1")
    target = _lane("lane1", extruder="extruder")
    unit = _make_unit([lane, target])

    handled = unit.handle_runout(lane)

    assert handled is True
    assert lane._oams_runout_detected is True   # blocks sensor noise
    assert lane._oams_runout_empty is False     # hardware unload NOT skipped
    assert unit.handle_same_fps_reload.calls == [((lane, target), {})]
    # No pause path artifacts
    assert not unit.afc.error.AFC_error.called
    assert not unit.lane_not_ready.called
    assert lane.status is None                  # untouched on this branch


# ── handle_runout: branch 4, cross extruder, defer to generic ────────────────

def test_cross_extruder_runout_defers_to_generic_infinite_spool():
    lane = _lane("lane0", extruder="extruder", runout_lane="lane4")
    target = _lane("lane4", extruder="extruder4")
    unit = _make_unit([lane, target])

    handled = unit.handle_runout(lane)

    assert handled is False                     # generic path takes over
    assert lane._oams_runout_empty is True      # hardware unload will be skipped
    assert lane._oams_runout_detected is False
    assert not unit.handle_same_fps_reload.called
    assert not unit.afc.error.AFC_error.called
    assert not unit.lane_not_ready.called


# ── _is_same_extruder: every branch ───────────────────────────────────────────

def test_is_same_extruder_matches_case_and_whitespace():
    unit = _make_unit()
    a = _lane("a", extruder=" Extruder ")
    b = _lane("b", extruder="extruder")
    assert unit._is_same_extruder(a, b) is True


def test_is_same_extruder_differs():
    unit = _make_unit()
    assert unit._is_same_extruder(_lane("a", extruder="extruder"),
                                  _lane("b", extruder="extruder4")) is False


def test_is_same_extruder_missing_extruder_obj():
    unit = _make_unit()
    a = _lane("a")
    a.extruder_obj = None
    assert unit._is_same_extruder(a, _lane("b")) is False
    b = _lane("b")
    b.extruder_obj = None
    assert unit._is_same_extruder(_lane("a"), b) is False


def test_is_same_extruder_missing_names():
    unit = _make_unit()
    a = _lane("a")
    a.extruder_obj = FakeExtruderObj(name=None)
    assert unit._is_same_extruder(a, _lane("b")) is False
    c = _lane("c")
    c.extruder_obj = FakeExtruderObj(name="")
    assert unit._is_same_extruder(_lane("a"), c) is False


# ── _resolve_lane_reference: every branch ─────────────────────────────────────

def test_resolve_lane_exact_match():
    lane = _lane("lane0")
    unit = _make_unit([lane])
    assert unit._resolve_lane_reference("lane0") is lane


def test_resolve_lane_case_insensitive_fallback():
    lane = _lane("Lane0")
    unit = _make_unit([lane])
    assert unit._resolve_lane_reference("lane0") is lane


def test_resolve_lane_missing_and_empty():
    unit = _make_unit()
    assert unit._resolve_lane_reference("nope") is None
    assert unit._resolve_lane_reference(None) is None
    assert unit._resolve_lane_reference("") is None


# ── _should_block_sensor_for_runout: full condition matrix ────────────────────

def _runout_lane(printing=True, tool_loaded=True,
                 status=AFCLaneState.INFINITE_RUNOUT, detected=True):
    lane = _lane("lane0", tool_loaded=tool_loaded, status=status)
    lane._oams_runout_detected = detected
    return lane, printing


def test_sensor_block_noop_without_flag():
    unit = _make_unit()
    lane, _ = _runout_lane(detected=False)
    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_active_runout_blocks_true_updates():
    unit = _make_unit()
    unit.afc.function.printing = True
    lane, _ = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, True) is True
    assert lane._oams_runout_detected is True  # flag survives while blocking


def test_sensor_block_tool_unloading_status_also_blocks():
    unit = _make_unit()
    unit.afc.function.printing = True
    lane, _ = _runout_lane(status=AFCLaneState.TOOL_UNLOADING)
    assert unit._should_block_sensor_for_runout(lane, True) is True


def test_sensor_block_false_update_clears_flag():
    unit = _make_unit()
    unit.afc.function.printing = True
    lane, _ = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, False) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_not_printing_clears_flag():
    unit = _make_unit()
    unit.afc.function.printing = False
    lane, _ = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_not_tool_loaded_clears_flag():
    unit = _make_unit()
    unit.afc.function.printing = True
    lane, _ = _runout_lane(tool_loaded=False)

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_wrong_status_clears_flag():
    unit = _make_unit()
    unit.afc.function.printing = True
    lane, _ = _runout_lane(status=AFCLaneState.NONE)

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_exception_treated_as_inactive():
    """If the printing check raises, active_runout falls to False and the
    flag clears rather than blocking forever."""
    unit = _make_unit()
    unit.afc.function.raise_on_is_printing = RuntimeError("boom")
    lane, _ = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


# ── FollowerController constructor contract ──────────────────────────────────
# The e1a2da0 bug was a call-site/constructor mismatch:
# FollowerController(oams_obj, printer, logger) vs (oams_dict, reactor, logger).

def test_follower_controller_constructed_like_call_site():
    oams = object()
    reactor = FakeReactor()
    logger = FakeLogger()

    follower = FollowerController({"oams1": oams}, reactor, logger)

    assert follower.oams == {"oams1": oams}
    assert follower.reactor is reactor
    assert follower.logger is logger
    assert follower.follower_state == {}
    assert follower._mcu_command_queue == {}
    assert follower._mcu_command_in_flight == {}
    assert follower._led_error_state == {}


def test_follower_state_created_on_demand():
    follower = FollowerController({"oams1": object()}, FakeReactor(), FakeLogger())
    state = follower.get_follower_state("oams1")
    assert isinstance(state, FollowerState)
    assert follower.get_follower_state("oams1") is state
    assert follower.follower_state == {"oams1": state}
