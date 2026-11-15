"""
Unit tests for afcAMS.get_status (extras/AFC_OpenAMS.py), surfacing the live
OpenAMS state on the AFC unit (parallels afcACE.get_status).

Adds, on top of the base unit status: oams_connected, controller sensors
(current_spool, fps_value, f1s/hub HES), failure counts, a readable
oams_action (from OAMSStatus), and the monitor's tracked load state /
clog-stuck detection windows.

Style: typed fakes (tests/openams_helpers.py), full state verification.
"""

from __future__ import annotations

from extras.AFC_OpenAMS import afcAMS, OAMSStatus, FPSLoadState

from tests.openams_helpers import FakeOams


class _FakeState:
    def __init__(self, state=FPSLoadState.LOADED, current_lane=None,
                 current_spool_idx=None, clog_start_time=None,
                 stuck_start_time=None):
        self.state = state
        self.current_lane = current_lane
        self.current_spool_idx = current_spool_idx
        self.clog_start_time = clog_start_time
        self.stuck_start_time = stuck_start_time


class _FakeMonitor:
    def __init__(self, state):
        self.state = state


def _make_unit(oams=None, monitor=None, operation_active=False):
    unit = afcAMS.__new__(afcAMS)
    unit.lanes = {}          # empty -> base get_status returns empty aggregates
    unit.oams = oams
    unit._monitor = monitor
    unit._operation_active = operation_active
    return unit


def test_get_status_surfaces_controller_and_action():
    oams = FakeOams(current_spool=0, fps_value=0.4547, f1s=[1, 1, 1, 1],
                    hub=[1, 0, 0, 0], action_status=OAMSStatus.LOADING,
                    load_failures=2, unload_failures=1)
    mon = _FakeMonitor(_FakeState(state=FPSLoadState.LOADING,
                                  current_lane="lane4", current_spool_idx=0))
    unit = _make_unit(oams=oams, monitor=mon, operation_active=True)

    st = unit.get_status()

    # base structure preserved
    assert st["lanes"] == [] and "hubs" in st
    # controller live state
    assert st["oams_connected"] is True
    assert st["oams_current_spool"] == 0
    assert st["oams_fps_value"] == 0.4547
    assert st["oams_f1s_hes"] == [1, 1, 1, 1]
    assert st["oams_hub_hes"] == [1, 0, 0, 0]
    assert st["oams_load_failures"] == 2
    assert st["oams_unload_failures"] == 1
    # action + busy
    assert st["oams_action"] == "loading"
    assert st["oams_busy"] is True
    # monitor state
    assert st["oams_load_state"] == "loading"
    assert st["oams_current_lane"] == "lane4"
    assert st["oams_current_spool_idx"] == 0
    assert st["oams_clog_detecting"] is False
    assert st["oams_stuck_detecting"] is False


def test_get_status_action_idle_when_no_action():
    oams = FakeOams(action_status=None)
    unit = _make_unit(oams=oams, monitor=_FakeMonitor(_FakeState()))
    assert unit.get_status()["oams_action"] == "idle"


def test_get_status_action_following_and_unknown_code():
    for code, name in ((OAMSStatus.FORWARD_FOLLOWING, "forward_following"),
                       (OAMSStatus.UNLOADING, "unloading"),
                       (OAMSStatus.COASTING, "coasting")):
        oams = FakeOams(action_status=code)
        unit = _make_unit(oams=oams, monitor=_FakeMonitor(_FakeState()))
        assert unit.get_status()["oams_action"] == name
    # An out-of-range code falls back to a generic "busy" (not idle).
    unit = _make_unit(oams=FakeOams(action_status=99),
                      monitor=_FakeMonitor(_FakeState()))
    assert unit.get_status()["oams_action"] == "busy"


def test_get_status_reports_active_clog_stuck_windows():
    mon = _FakeMonitor(_FakeState(clog_start_time=123.0, stuck_start_time=45.0))
    unit = _make_unit(oams=FakeOams(), monitor=mon)
    st = unit.get_status()
    assert st["oams_clog_detecting"] is True
    assert st["oams_stuck_detecting"] is True


def test_get_status_no_controller_or_monitor():
    unit = _make_unit(oams=None, monitor=None)
    st = unit.get_status()
    assert st["oams_connected"] is False
    assert st["oams_action"] == "idle"
    assert st["oams_busy"] is False
    # controller/monitor-only keys are simply absent when not connected
    assert "oams_current_spool" not in st
    assert "oams_load_state" not in st


# ── _current_oams_action ──────────────────────────────────────────────────────

def test_current_oams_action_maps_names():
    assert _make_unit(oams=FakeOams(action_status=OAMSStatus.UNLOADING)) \
        ._current_oams_action() == "unloading"
    assert _make_unit(oams=FakeOams(action_status=OAMSStatus.FORWARD_FOLLOWING)) \
        ._current_oams_action() == "forward_following"


def test_current_oams_action_idle_and_unknown():
    assert _make_unit(oams=FakeOams(action_status=None))._current_oams_action() == ""
    assert _make_unit(oams=None)._current_oams_action() == ""
    assert _make_unit(oams=FakeOams(action_status=99))._current_oams_action() == "busy"


# ── poll action-transition logging (parallels afcACE) ─────────────────────────

from tests.openams_helpers import FakeLogger, FakeAFC  # noqa: E402


def _poll_unit(action_status):
    unit = afcAMS.__new__(afcAMS)
    unit.name = "AMS_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.oams = FakeOams(action_status=action_status)
    unit._operation_active = True    # returns right after action logging
    unit._current_action = ""
    unit.lanes = {}
    return unit


def test_poll_logs_action_transition():
    unit = _poll_unit(OAMSStatus.LOADING)
    unit._poll_oams_sensors(100.0)
    assert unit._current_action == "loading"
    assert any("AMS_1: idle -> loading" in m for m in unit.logger.lines["info"])


def test_poll_no_duplicate_log_when_unchanged():
    unit = _poll_unit(OAMSStatus.LOADING)
    unit._poll_oams_sensors(100.0)
    n = len(unit.logger.lines["info"])
    unit._poll_oams_sensors(102.0)   # same action
    assert len(unit.logger.lines["info"]) == n


def test_poll_logs_return_to_idle():
    unit = _poll_unit(OAMSStatus.LOADING)
    unit._poll_oams_sensors(100.0)
    unit.oams.action_status = None   # operation finished
    unit._poll_oams_sensors(102.0)
    assert unit._current_action == ""
    assert any("loading -> idle" in m for m in unit.logger.lines["info"])
