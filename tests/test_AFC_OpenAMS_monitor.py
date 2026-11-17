"""
Unit tests for the OAMSMonitor clog and stuck-spool detection in
extras/AFC_OpenAMS.py

Style: typed fakes (no MagicMock), full state verification, branch-complete:

  _check_clog, post-load grace, missing extruder, genuine clog (dwell +
      extrusion window), below-window no-fire, already-active no re-fire,
      extruder swap restarts the window (the toolchange false-positive fix),
      detection still works post-swap, cumulative encoder re-baseline,
      pressure/encoder condition resets
  _check_stuck_spool, engagement grace, dwell fire, encoder-moving and
      pressure-ok no-fire, partial recovery keeps the latch, full recovery
      clears + notifies, no-callback safety
  FPSState.reset, clears the clog window incl. the extruder object
"""

from __future__ import annotations

from extras.AFC_OpenAMS import (
    OAMSMonitor,
    CLOG_PRESSURE_TARGET,
    CLOG_EXTRUSION_WINDOW,
    CLOG_DWELL,
    CLOG_ENCODER_SLACK,
    STUCK_PRESSURE_LOW,
    STUCK_PRESSURE_CLEAR,
    STUCK_DWELL,
    STUCK_MIN_ENCODER,
)

from tests.openams_helpers import FakeLogger, FakeReactor, Recorder


# ── Fakes ─────────────────────────────────────────────────────────────────────

class _FakeExtruder:
    """A Klipper extruder stand-in: identity + its own position counter."""

    def __init__(self, name, last_position=0.0):
        self.name = name
        self.last_position = last_position


class _FakeFps:
    """Minimal fps object: fps_value + an `extruder` attribute the test can
    swap to emulate a toolchange (the real property resolves the ACTIVE
    toolhead extruder). Pass extruder=None to omit the attribute entirely."""

    def __init__(self, extruder=None):
        if extruder is not None:
            self.extruder = extruder
        self.fps_value = CLOG_PRESSURE_TARGET


def _make_monitor(fps=None):
    on_clog = Recorder()
    on_stuck = Recorder()
    on_cleared = Recorder()
    monitor = OAMSMonitor(
        fps_name="FPS_test",
        fps_obj=fps if fps is not None else _FakeFps(_FakeExtruder("extruder")),
        reactor=FakeReactor(),
        logger=FakeLogger(),
        on_clog=on_clog,
        on_stuck_spool=on_stuck,
        on_stuck_cleared=on_cleared,
        clog_sensitivity="medium",
        is_printing_fn=lambda: True,
    )
    # _check_clog reads st.last_encoder (normally fed by the timer loop)
    monitor.state.last_encoder = 0
    return monitor, on_clog, on_stuck, on_cleared


# ── _check_clog: gates ────────────────────────────────────────────────────────

def test_clog_post_load_grace_suppresses():
    monitor, on_clog, _, _ = _make_monitor()
    monitor.state.last_lane_change_time = 100.0

    monitor._check_clog(100.0 + monitor.clog_post_load_grace - 1.0,
                        0, CLOG_PRESSURE_TARGET)

    assert monitor.state.clog_start_time is None
    assert not on_clog.called


def test_clog_requires_extruder_position():
    """fps without an extruder attribute: detection cannot run."""
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder=None))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)

    assert monitor.state.clog_start_time is None
    assert not on_clog.called


# ── _check_clog: genuine clog on a single extruder ────────────────────────────

def test_clog_fires_same_extruder():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)  # opens window
    assert monitor.state.clog_start_time == 100.0
    assert monitor.state.clog_start_extruder == 100.0
    assert monitor.state.clog_start_extruder_obj is extruder
    assert monitor.state.clog_start_encoder == 0

    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    assert on_clog.call_count == 1
    assert monitor.state.clog_active is True
    fps_name, msg = on_clog.last_args
    assert fps_name == "FPS_test"
    assert "Clog detected" in msg


def test_clog_does_not_fire_below_extrusion_window():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW / 2
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    assert not on_clog.called
    assert monitor.state.clog_active is False
    assert monitor.state.clog_start_time == 100.0  # window still open


def test_clog_active_does_not_refire():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)
    monitor._check_clog(100.0 + 2 * CLOG_DWELL, 0, CLOG_PRESSURE_TARGET)

    assert on_clog.call_count == 1  # one-shot while active


# ── _check_clog: extruder swap (the toolchange false positive) ────────────────

def test_extruder_swap_restarts_window():
    """A toolchange mid-window swaps fps.extruder to a different object whose
    position counter differs arbitrarily. The window must restart on the new
    extruder instead of firing off the cross-extruder position delta."""
    extruder_a = _FakeExtruder("extruder", last_position=1000.0)
    extruder_b = _FakeExtruder("extruder4", last_position=1060.9)
    fps = _FakeFps(extruder_a)
    monitor, on_clog, _, _ = _make_monitor(fps=fps)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    assert monitor.state.clog_start_extruder_obj is extruder_a

    fps.extruder = extruder_b  # toolchange (counters differ by 60.9mm)
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    assert not on_clog.called
    assert monitor.state.clog_active is False
    # Window restarted on B with B's own baseline
    assert monitor.state.clog_start_extruder_obj is extruder_b
    assert monitor.state.clog_start_extruder == extruder_b.last_position
    assert monitor.state.clog_start_time == 100.0 + CLOG_DWELL + 1.0


def test_clog_fires_on_new_extruder_after_swap():
    extruder_a = _FakeExtruder("extruder", last_position=1000.0)
    extruder_b = _FakeExtruder("extruder4", last_position=0.0)
    fps = _FakeFps(extruder_a)
    monitor, on_clog, _, _ = _make_monitor(fps=fps)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    fps.extruder = extruder_b
    monitor._check_clog(110.0, 0, CLOG_PRESSURE_TARGET)   # restart on B
    assert not on_clog.called

    extruder_b.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(110.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    assert on_clog.call_count == 1
    assert monitor.state.clog_active is True


def test_swap_back_and_forth_never_fires_without_advance():
    extruder_a = _FakeExtruder("extruder", last_position=500.0)
    extruder_b = _FakeExtruder("extruder4", last_position=1234.5)
    fps = _FakeFps(extruder_a)
    monitor, on_clog, _, _ = _make_monitor(fps=fps)

    eventtime = 100.0
    for _ in range(5):
        monitor._check_clog(eventtime, 0, CLOG_PRESSURE_TARGET)
        eventtime += CLOG_DWELL + 1.0
        fps.extruder = extruder_b if fps.extruder is extruder_a else extruder_a

    assert not on_clog.called
    assert monitor.state.clog_active is False


# ── _check_clog: re-baselining and condition resets ───────────────────────────

def test_cumulative_encoder_progress_restarts_window():
    """Cumulative encoder movement past the slack means filament IS flowing:
    the window re-baselines (incl. the extruder object) instead of firing."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    monitor.state.last_encoder = CLOG_ENCODER_SLACK * 3
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    fire_time = 100.0 + CLOG_DWELL + 1.0
    monitor._check_clog(fire_time, 0, CLOG_PRESSURE_TARGET)

    assert not on_clog.called
    assert monitor.state.clog_start_time == fire_time
    assert monitor.state.clog_start_extruder == extruder.last_position
    assert monitor.state.clog_start_extruder_obj is extruder
    assert monitor.state.clog_start_encoder == CLOG_ENCODER_SLACK * 3


def test_pressure_out_of_band_resets_tracking():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, 0.2)  # tension, not target

    assert not on_clog.called
    assert monitor.state.clog_start_time is None


def test_encoder_moving_resets_tracking():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0,
                        CLOG_ENCODER_SLACK + 1, CLOG_PRESSURE_TARGET)

    assert not on_clog.called
    assert monitor.state.clog_start_time is None


# ── _check_stuck_spool ────────────────────────────────────────────────────────

LOW = STUCK_PRESSURE_LOW - 0.01


def test_stuck_spool_engagement_grace_suppresses():
    monitor, _, on_stuck, _ = _make_monitor()
    monitor.state.engagement_checked_at = 100.0

    monitor._check_stuck_spool(101.0, 0, LOW)  # within 6s grace

    assert monitor.state.stuck_start_time is None
    assert not on_stuck.called


def test_stuck_spool_fires_after_dwell():
    monitor, _, on_stuck, _ = _make_monitor()

    monitor._check_stuck_spool(100.0, 0, LOW)
    assert monitor.state.stuck_start_time == 100.0
    assert monitor.state.stuck_active is False
    assert not on_stuck.called

    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, LOW)

    assert on_stuck.call_count == 1
    assert monitor.state.stuck_active is True
    fps_name, msg = on_stuck.last_args
    assert fps_name == "FPS_test"
    assert "Stuck spool" in msg


def test_stuck_spool_no_fire_when_encoder_moving():
    monitor, _, on_stuck, _ = _make_monitor()

    monitor._check_stuck_spool(100.0, STUCK_MIN_ENCODER, LOW)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, STUCK_MIN_ENCODER, LOW)

    assert not on_stuck.called
    assert monitor.state.stuck_start_time is None


def test_stuck_spool_no_fire_when_pressure_ok():
    monitor, _, on_stuck, _ = _make_monitor()
    ok = STUCK_PRESSURE_LOW + 0.05

    monitor._check_stuck_spool(100.0, 0, ok)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, ok)

    assert not on_stuck.called
    assert monitor.state.stuck_start_time is None


def test_stuck_spool_partial_recovery_keeps_latch():
    """Pressure between the low and clear thresholds with the encoder still:
    the stuck state persists (hysteresis), no premature clear."""
    monitor, _, on_stuck, on_cleared = _make_monitor()

    monitor._check_stuck_spool(100.0, 0, LOW)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, LOW)
    assert monitor.state.stuck_active is True

    between = (STUCK_PRESSURE_LOW + STUCK_PRESSURE_CLEAR) / 2
    monitor._check_stuck_spool(110.0, 0, between)

    assert monitor.state.stuck_active is True
    assert not on_cleared.called


def test_stuck_spool_clears_on_pressure_recovery():
    monitor, _, on_stuck, on_cleared = _make_monitor()

    monitor._check_stuck_spool(100.0, 0, LOW)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, LOW)
    assert monitor.state.stuck_active is True

    monitor._check_stuck_spool(110.0, 0, STUCK_PRESSURE_CLEAR + 0.01)

    assert on_cleared.calls == [(("FPS_test",), {})]
    assert monitor.state.stuck_active is False
    assert monitor.state.stuck_start_time is None


def test_stuck_spool_pending_timer_clears_before_firing():
    """Recovery during the dwell (before firing) resets the timer without
    notifying anyone."""
    monitor, _, on_stuck, on_cleared = _make_monitor()

    monitor._check_stuck_spool(100.0, 0, LOW)
    monitor._check_stuck_spool(100.5, STUCK_MIN_ENCODER, STUCK_PRESSURE_CLEAR + 0.01)

    assert monitor.state.stuck_start_time is None
    assert not on_stuck.called
    assert not on_cleared.called


# ── FPSState.reset ────────────────────────────────────────────────────────────

def test_state_reset_clears_clog_window():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, _, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    assert monitor.state.clog_start_extruder_obj is extruder

    monitor.state.reset()

    assert monitor.state.clog_start_time is None
    assert monitor.state.clog_start_extruder is None
    assert monitor.state.clog_start_extruder_obj is None
    assert monitor.state.clog_start_encoder is None
    assert monitor.state.clog_active is False
    assert monitor.state.stuck_active is False
    assert monitor.state.stuck_start_time is None
