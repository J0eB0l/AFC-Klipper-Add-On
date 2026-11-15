"""
Unit tests for afcAMS sensor polling and load finishing in
extras/AFC_OpenAMS.py

Style: typed fakes (tests/openams_helpers.py) instead of MagicMock, full
state verification after every call, branch-complete coverage:

  _poll_oams_sensors, no-hardware stop, operation-active skip, resync
      consumption, lane-not-mapped skip, short sensor arrays, F1S present/
      lost/inserted transitions (prep_state, loaded_to_hub, runout events),
      suppressed-load marking, sensor-block integration, hub HES -> raw
      _load_state
  _advance_tool_stn_to_nozzle, no tool_stn / fully-credited / partial /
      full advance, speed derivation incl. the default
"""

from __future__ import annotations

from extras.AFC_OpenAMS import afcAMS
from extras.AFC_lane import AFCLaneState

from tests.openams_helpers import (
    FakeAFC,
    FakeExtruderObj,
    FakeLane,
    FakeLogger,
    FakeOams,
    Recorder,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_unit(lane=None, f1s=None, hub=None, last_f1s=None,
               stale=False, op_active=False, spool_map=None):
    unit = afcAMS.__new__(afcAMS)
    unit.name = "AMS_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.lanes = {}
    if lane is not None:
        unit.lanes[lane.name] = lane
        unit.afc.lanes[lane.name] = lane
    unit._spool_map = spool_map if spool_map is not None else (
        {lane.name: 0} if lane is not None else {})
    unit._current_action = ""
    unit._operation_active = op_active
    unit._prev_states_stale = stale
    unit._last_f1s = list(last_f1s) if last_f1s is not None else [None] * 4
    unit._last_hub = [None] * 4
    unit._hub_load_suppressed = set()
    unit.oams = FakeOams(f1s=f1s, hub=hub)
    return unit


def _lane(name="lane0", loaded_to_hub=True):
    lane = FakeLane(name, extruder_obj=FakeExtruderObj("extruder"))
    lane.loaded_to_hub = loaded_to_hub
    return lane


# ── Early-out branches ────────────────────────────────────────────────────────

def test_poll_stops_without_hardware():
    lane = _lane()
    unit = _make_unit(lane)
    unit.oams = None

    assert unit._poll_oams_sensors(100.0) == unit.afc.reactor.NEVER


def test_poll_skipped_during_operation():
    lane = _lane()
    unit = _make_unit(lane, f1s=[0, 0, 0, 0], last_f1s=[True, None, None, None],
                      op_active=True)

    result = unit._poll_oams_sensors(100.0)

    assert result == 100.25  # re-schedules without touching state
    assert not lane.handle_load_runout.called
    assert lane.loaded_to_hub is True
    assert lane.prep_state is False           # untouched
    assert unit._last_f1s[0] is True          # snapshot untouched


def test_poll_skips_unmapped_lane():
    lane = _lane()
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], spool_map={})  # not in map

    unit._poll_oams_sensors(100.0)

    assert lane.prep_state is False
    assert not lane.handle_load_runout.called


def test_poll_handles_short_sensor_arrays():
    """Slot index past the reported arrays: neither block runs."""
    lane = _lane()
    unit = _make_unit(lane, f1s=[], hub=[], spool_map={"lane0": 0})
    unit.oams.f1s_hes_value = []
    unit.oams.hub_hes_value = []

    unit._poll_oams_sensors(100.0)

    assert lane.prep_state is False
    assert lane._load_state is False
    assert not lane.handle_load_runout.called


# ── F1S transitions ───────────────────────────────────────────────────────────

def test_f1s_present_drives_prep_state_no_event_without_change():
    lane = _lane()
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane.prep_state is True
    assert lane.loaded_to_hub is True         # untouched while present
    assert not lane.handle_load_runout.called
    assert unit._last_f1s[0] is True


def test_f1s_first_reading_sets_snapshot_without_event():
    """old_f1s None (first poll ever): no insert/remove event fires."""
    lane = _lane()
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[None, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert not lane.handle_load_runout.called
    assert unit._last_f1s[0] is True


def test_f1s_loss_fires_runout_and_clears_staging():
    lane = _lane()
    unit = _make_unit(lane, f1s=[0, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)      # starts the debounce window
    assert not lane.handle_load_runout.called
    unit._poll_oams_sensors(101.0)      # held past f1s_debounce -> commits

    assert lane.prep_state is False
    assert lane.loaded_to_hub is False        # spool gone -> can't be staged
    assert lane.handle_load_runout.calls == [((101.0, False), {})]
    assert unit._last_f1s[0] is False


def test_f1s_insert_fires_load():
    lane = _lane(loaded_to_hub=False)
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[False, None, None, None])

    unit._poll_oams_sensors(100.0)      # starts the debounce window
    assert not lane.handle_load_runout.called
    assert lane.prep_state is False     # committed state until it holds
    unit._poll_oams_sensors(101.0)      # held past f1s_debounce -> commits

    assert lane.prep_state is True
    assert lane.handle_load_runout.calls == [((101.0, True), {})]
    assert lane._load_suppressed is False


def test_resync_after_operation_suppresses_events():
    """First poll after a load/unload: the previous snapshot is stale, just
    re-sync it without firing insert/remove."""
    lane = _lane()
    unit = _make_unit(lane, f1s=[0, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None], stale=True)

    unit._poll_oams_sensors(100.0)

    assert not lane.handle_load_runout.called
    assert unit._last_f1s[0] is False         # re-synced
    assert unit._prev_states_stale is False   # consumed
    assert lane.prep_state is False           # live state still applied
    assert lane.loaded_to_hub is False


def test_suppressed_load_marks_lane_suppressed():
    """A load event the unit itself caused (hub feed) is marked suppressed so
    handle_load_runout can tell it apart from a user insert."""
    lane = _lane(loaded_to_hub=False)
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[False, None, None, None])
    unit._hub_load_suppressed = {"lane0"}

    unit._poll_oams_sensors(100.0)      # starts the debounce window
    unit._poll_oams_sensors(101.0)      # commits

    assert lane._load_suppressed is True
    assert unit._hub_load_suppressed == set()  # consumed
    assert lane.handle_load_runout.calls == [((101.0, True), {})]


def test_sensor_block_swallows_true_update_during_runout():
    """During an active same-FPS runout reload, a True F1S flicker is
    blocked: no event, but the snapshot still records the new value."""
    lane = _lane()
    lane.tool_loaded = True
    lane.status = AFCLaneState.INFINITE_RUNOUT
    lane._oams_runout_detected = True
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[False, None, None, None])
    unit.afc.function.printing = True

    unit._poll_oams_sensors(100.0)      # starts the debounce window
    unit._poll_oams_sensors(101.0)      # commits (blocked path)

    assert not lane.handle_load_runout.called
    assert unit._last_f1s[0] is True          # snapshot advanced anyway
    assert lane._oams_runout_detected is True  # block window still active


# ── Hub HES -> raw load state ─────────────────────────────────────────────────

def test_hub_hes_drives_raw_load_state():
    lane = _lane()
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[1, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane._load_state == 1
    assert unit._last_hub[0] == 1


def test_hub_hes_clear_reads_zero():
    lane = _lane()
    lane._load_state = True
    unit = _make_unit(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane._load_state == 0


# ── _advance_tool_stn_to_nozzle ───────────────────────────────────────────────

class _Ext:
    """Extruder with explicit tool_stn/tool_load_speed (omit to test the
    getattr defaults)."""

    def __init__(self, tool_stn=None, tool_load_speed=None):
        if tool_stn is not None:
            self.tool_stn = tool_stn
        if tool_load_speed is not None:
            self.tool_load_speed = tool_load_speed


def _advance_unit():
    unit = afcAMS.__new__(afcAMS)
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.afc.afcDeltaTime = Recorder()
    unit.afc.afcDeltaTime.log_with_time = Recorder()
    unit._oams_extrude = Recorder()
    return unit


def _advance_lane(ext):
    lane = FakeLane("lane0", extruder_obj=ext)
    return lane


def test_tool_stn_advance_full_distance():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(100.0, 25.0)))
    assert unit._oams_extrude.calls == [
        ((100.0, 25.0 * 60.0, "tool_stn_to_nozzle"), {})]


def test_tool_stn_advance_credits_engagement():
    """Engagement verification already pushed filament past the sensor,
    only the remainder is extruded."""
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(100.0, 25.0)),
                                     already_advanced=30.0)
    assert unit._oams_extrude.last_args[0] == 70.0


def test_tool_stn_advance_noop_when_fully_covered():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(50.0, 25.0)),
                                     already_advanced=60.0)
    assert not unit._oams_extrude.called


def test_tool_stn_advance_noop_without_tool_stn():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(0, 25.0)))
    assert not unit._oams_extrude.called
    # Attribute missing entirely -> getattr default 0 -> no-op
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext()))
    assert not unit._oams_extrude.called


def test_tool_stn_advance_default_speed():
    """tool_load_speed missing -> 25mm/s default (x60 for mm/min)."""
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(tool_stn=40.0)))
    assert unit._oams_extrude.last_args == (40.0, 1500.0, "tool_stn_to_nozzle")
