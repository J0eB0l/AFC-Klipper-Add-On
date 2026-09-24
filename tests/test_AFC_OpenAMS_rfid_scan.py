"""
Tests for the OpenAMS RFID scan-on-insert flow (afcAMS._do_rfid_scan and the
insert-edge scheduling/latch around it).

The scan drives real filament motion, so the whole point of these tests is to
pin the ORCHESTRATION (TD-1-capture style): wait for the unit's ready signal,
take the sister-tag baseline on the shared antenna, send a verified NORMAL
load, gate reader polling on encoder movement, light-probe for a NEW uid, stop
the feed at the detection position, full-read stationary (with an
unload/re-feed-to-position retry when unreadable), and always unwind back:
with the operation guard and follower state cleaned up on every path.
"""

import sys
import types
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
from tests.conftest import MockAFC, MockPrinter, MockConfig  # noqa: E402


class AdvancingReactor:
    """Reactor whose pause() advances monotonic time, so timed loops terminate."""

    NEVER = 9_999_999_999.0
    NOW = 0.0

    def __init__(self, monotonic_value=100.0):
        self._monotonic = monotonic_value
        self.registered = []

    def monotonic(self):
        return self._monotonic

    def pause(self, until):
        # Callers pass monotonic()+delay; advance to it so deadlines are reached.
        self._monotonic = max(self._monotonic, until)

    def register_timer(self, callback, waketime=None):
        handle = ("timer", len(self.registered))
        self.registered.append((handle, callback, waketime))
        return handle

    def unregister_timer(self, handle):
        self.registered = [r for r in self.registered if r[0] != handle]


class FakeCmd:
    def __init__(self):
        self.sent = []

    def send(self, args=None):
        self.sent.append(args)


class FakeController:
    """Minimal AFC_OAMS stand-in exposing exactly what the scan touches.

    The load command "moves filament": sending it bumps encoder_clicks past
    the polling gate and trips the hub HES, so engagement detection and the
    encoder gate both see motion.
    """

    def __init__(self, reactor=None):
        self.follower_calls = []
        self.oams_load_spool_cmd = FakeCmd()
        self.oams_load_spool_cmd.send = self._send_load
        self.action_status = None
        self.action_status_code = None
        # The firmware reports its motor state only in ANSWER to a command --
        # there is no periodic stream -- which is why the ready-wait probes.
        self.reactor = reactor
        self.motion_status = None
        self.motion_status_code = None
        self.motion_status_time = 0.0
        self.cancel_calls = 0
        self.unload_calls = 0
        self.clear_errors_calls = 0
        self.current_spool = 3
        self.encoder_clicks = 500          # running counter, never zero
        self.hub_hes_value = [0, 0, 0, 0]
        # Number of load attempts to reject ERROR_BUSY before accepting
        # (models the firmware's insert-staging window).
        self.busy_rejections = 0
        # Number of readiness PROBES answered ERROR_BUSY before the unit
        # reports STOPPED. Independent of busy_rejections: a load can be
        # refused for reasons that have nothing to do with the motor.
        self.staging_probes = 0

    def _send_load(self, args):
        # Model the firmware load: filament moves, hub trips, load completes.
        self.oams_load_spool_cmd.sent.append(args)
        if self.busy_rejections > 0:
            self.busy_rejections -= 1
            self.action_status = None
            self.action_status_code = 2    # OAMSOpCode.ERROR_BUSY
            return
        self.encoder_clicks += 120
        self.hub_hes_value[args[0]] = 1
        self.action_status = None          # ack: load done
        self.action_status_code = 0        # OAMSOpCode.SUCCESS

    # Motor primitives
    def set_oams_follower(self, enable, direction):
        self.follower_calls.append((enable, direction))
        # Answer like the firmware: a status comes back only when the command
        # is REFUSED or the motor state actually CHANGES. While a routine
        # (e.g. the insert auto-stage) owns the motor, every stop is refused
        # ERROR_BUSY. Once it is done, a stop sent to an already-stopped unit
        # changes nothing -- so the firmware answers with SILENCE, and silence
        # is what the ready-wait has to read as "ready".
        if self.staging_probes > 0:
            self.staging_probes -= 1
            self.motion_status = OAMSStatus.REVERSE_FOLLOWING
            self.motion_status_code = 2      # OAMSOpCode.ERROR_BUSY
            if self.reactor is not None:
                self.motion_status_time = self.reactor.monotonic()

    def load_spool_cancel(self):
        self.cancel_calls += 1
        self.action_status = None
        return "cancelled"

    def unload_spool(self):
        self.unload_calls += 1
        return True, "ok"

    def clear_errors(self):
        self.clear_errors_calls += 1

    def is_bay_ready(self, bay):
        return True


class FakeCoordinator:
    """AFC_OpenAMS_rfid stand-in with the field/read API.

    ``fields`` is a list of uid-lists, one per scan_slot_uids() call: the
    FIRST is the rest-time baseline, later entries are the per-poll field
    contents during the feed (the last entry repeats once exhausted).
    """

    def __init__(self, fields=None, full_reads=None):
        self._fields = [list(f) for f in (fields or [[]])]
        self._reads = list(full_reads or [])
        self.read_excludes = []
        self.applied = []
        self.slot_map = {"lane1": 0}

    def _get_slot(self, name):
        return self.slot_map.get(name)

    def scan_slot_uids(self, slot):
        if len(self._fields) > 1:
            return self._fields.pop(0)
        return list(self._fields[0])

    def read_slot_excluding(self, slot, exclude):
        self.read_excludes.append(set(exclude))
        if not self._reads:
            return None
        return self._reads.pop(0)

    def read_slot(self, slot):              # manual-path compat
        return self.read_slot_excluding(slot, set())

    def apply_to_lane(self, lane, tag):
        self.applied.append((lane, tag))
        return {"brand": "X", "material": "PLA"}

    def undecoded_hint(self, name):
        return ""


class FakeLane:
    def __init__(self, name="lane1"):
        self.name = name
        self.tool_loaded = False
        self.loaded_to_hub = False
        self.send_lane_data_calls = 0

    def send_lane_data(self):
        self.send_lane_data_calls += 1


def _build_unit(values=None, coord=None):
    afc = MockAFC()
    afc.reactor = AdvancingReactor()
    printer = MockPrinter(afc=afc)
    printer.reactor = afc.reactor
    cfg_values = {"rfid_scan_on_insert": True, "rfid_scan_timeout": 2.0,
                  "rfid_scan_poll": 0.2, "rfid_scan_read_retries": 2}
    cfg_values.update(values or {})
    config = MockConfig(name="AFC_OpenAMS ams1", printer=printer, values=cfg_values)
    ams = afcAMS(config)
    ams.afc = afc
    # in_print() must return a real bool (MagicMock's default is truthy).
    afc.function.in_print = lambda: False
    ams.oams = FakeController(reactor=afc.reactor)
    ams.lanes = {"lane1": FakeLane()}
    ams._spool_map = {"lane1": 0}
    coord = coord if coord is not None else FakeCoordinator()
    ams._rfid_coord = coord
    printer._objects["AFC_OpenAMS_rfid"] = coord
    return ams, coord


TAG = {"uid": "AABB", "filament": {"material": "PLA"}, "tag_type": "MifareClassic1k"}


class TestConfigDefaults:
    def test_defaults(self):
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        config = MockConfig(name="AFC_OpenAMS ams1", printer=printer, values={})
        ams = afcAMS(config)
        assert ams.rfid_scan_on_insert is False
        assert ams.rfid_scan_timeout == 15.0
        assert ams.rfid_scan_read_retries == 3
        # Tuned on hardware: sweep_back must cover the re-feed overshoot
        # (~150 clicks) before it buys any pre-roll before the detect point.
        assert ams.rfid_scan_sweep_back == 240
        assert ams.rfid_scan_sweep_step == 25
        assert ams.rfid_scan_sweep_past == 200

    def test_enabled(self):
        ams, _ = _build_unit()
        assert ams.rfid_scan_on_insert is True


class TestScanTagFound:
    def _run(self):
        # Empty field at rest; the moving tag arrives on the second poll.
        coord = FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_returns_true(self):
        _, _, result = self._run()
        assert result is True

    def test_applies_tag_to_lane(self):
        ams, coord, _ = self._run()
        assert len(coord.applied) == 1
        lane, tag = coord.applied[0]
        assert lane is ams.lanes["lane1"]
        assert tag is TAG

    def test_surfaces_to_mainsail(self):
        ams, _, _ = self._run()
        assert ams.lanes["lane1"].send_lane_data_calls == 1

    def test_persists_across_restart(self):
        # Without save_vars a FIRMWARE_RESTART wipes the applied data when
        # PREP rebuilds lane_data (the field-observed lane6 clear).
        ams, _, _ = self._run()
        assert ams.afc.save_vars.called

    def test_sends_the_load_once(self):
        ams, _, _ = self._run()
        assert ams.oams.oams_load_spool_cmd.sent == [[0]]

    def test_follower_stopped_at_end(self):
        ams, _, _ = self._run()
        assert (1, 1) in ams.oams.follower_calls      # pre-load forward
        assert ams.oams.follower_calls[-1] == (0, 0)  # stopped at the end

    def test_unwinds_after_engagement(self):
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 1

    def test_clears_operation_guard_and_latches(self):
        ams, _, _ = self._run()
        assert ams._operation_active is False
        assert ams._prev_states_stale is True
        assert "lane1" in ams._rfid_scanned


class TestSisterTags:
    def test_constant_sister_never_detected(self):
        # A seated neighbour's tag answers every poll: stationary = sister,
        # so a scan with ONLY it in field times out instead of detecting.
        coord = FakeCoordinator(fields=[["5157e12"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []

    def test_new_uid_beside_sister_detected_and_sister_excluded(self):
        # Sister present throughout; the moving tag arrives later: it is
        # detected and the sister is excluded from the full read.
        coord = FakeCoordinator(
            fields=[["5157e12"], ["5157e12"], ["5157e12", "aabb"]],
            full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert all("5157e12" in ex for ex in coord.read_excludes)

    def test_own_tag_at_rest_detected_by_motion(self):
        # The inserted spool's OWN tag rests on the antenna (in the baseline),
        # then blinks out for >= reappear_polls during the feed and returns:
        # motion brands it OURS, not a sister (the 01d0ec0f field case).
        coord = FakeCoordinator(
            fields=[["01d0ec0f"],                       # baseline (at rest)
                    ["01d0ec0f"],                        # still there
                    [], [], [], [],                      # gone 4 polls (moving)
                    ["01d0ec0f"]],                       # back in range
            full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        # It is the target, so it must NOT be excluded from the read.
        assert all("01d0ec0f" not in ex for ex in coord.read_excludes)


class TestUnreadableReposition:
    def _run(self):
        # Detect succeeds; the stationary reads at the stop position all fail
        # (2 retries), then the reposition read decodes.
        coord = FakeCoordinator(fields=[[], ["aabb"]],
                                full_reads=[None, None, TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_decodes_after_reposition(self):
        ams, coord, result = self._run()
        assert result is True
        assert len(coord.applied) == 1

    def test_reloads_the_lane(self):
        # One load for the scan feed + one for the reposition.
        ams, _, _ = self._run()
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2

    def test_unloads_twice(self):
        # Once before the reposition re-feed, once at the final unwind.
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 2

    def test_gives_up_cleanly_when_still_unreadable(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []
        assert ams._operation_active is False
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestSafetyGates:
    def test_lane_loaded_to_shared_toolhead_blocks_scan(self):
        # Some lane (any unit's) is loaded into the toolhead this unit's
        # lanes feed -> blocked.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        shared_ext = types.SimpleNamespace(name="extruder1",
                                           lane_loaded="lane9")
        ams.lanes["lane1"].extruder_obj = shared_ext
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []
        assert coord.applied == []

    def test_unrelated_toolhead_does_not_block(self):
        # The unit's shared toolhead is free; other toolheads on a multi-tool
        # machine are irrelevant to this unit's scan.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        free_ext = types.SimpleNamespace(name="extruder1", lane_loaded=None)
        ams.lanes["lane1"].extruder_obj = free_ext
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True

    def test_occupied_hub_blocks_scan(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.oams.hub_hes_value[2] = 1        # some other bay at the hub
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []

    def test_occupied_hub_does_not_block_scheduling(self):
        # Insert staging trips the hub HES briefly: scheduling must not be
        # blocked by it (the scan re-checks after the ready-wait).
        ams, _ = _build_unit()
        ams.oams.hub_hes_value[0] = 1
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert "lane1" in ams._rfid_scan_timers

    def test_shared_toolhead_loaded_blocks_scheduling(self):
        ams, _ = _build_unit()
        ams.lanes["lane1"].extruder_obj = types.SimpleNamespace(
            name="extruder1", lane_loaded="lane9")
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_in_print_blocks_scan(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.afc.function.in_print = lambda: True
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []


class TestHubEngageCancel:
    def test_load_cancelled_when_hub_engages(self):
        # A load still in flight when the hub HES trips must be cancelled
        # (TD-1 style) and replaced with slow follower creep.
        coord = FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)

        def slow_load(args):
            # Load stays in flight; hub trips immediately.
            ams.oams.oams_load_spool_cmd.sent.append(args)
            ams.oams.encoder_clicks += 30
            ams.oams.hub_hes_value[args[0]] = 1
            ams.oams.action_status = OAMSStatus.LOADING

        ams.oams.oams_load_spool_cmd.send = slow_load
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert ams.oams.cancel_calls >= 1          # load cancelled at hub
        assert (1, 1) in ams.oams.follower_calls   # creep enabled


class TestScanTimeout:
    def _run(self):
        # Reader never sees a new tag (e.g. a spool without one).
        ams, coord = _build_unit()
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_returns_false(self):
        _, _, result = self._run()
        assert result is False

    def test_does_not_apply(self):
        _, coord, _ = self._run()
        assert coord.applied == []

    def test_still_unwinds_and_cleans_up(self):
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 1
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestBusyHandling:
    def test_one_retry_after_busy_rejection(self):
        # Firmware refuses the first load ERROR_BUSY (staging raced the ready
        # wait); the scan waits for ready again and sends exactly ONE more.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.oams.busy_rejections = 1
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2
        assert len(coord.applied) == 1

    def test_aborts_after_second_busy_no_hammering(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(values={"rfid_scan_ready_timeout": 5.0},
                                 coord=coord)
        ams.oams.busy_rejections = 10_000
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        # Never more than two load attempts: we wait on ready, we don't spam.
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2
        assert coord.applied == []
        # Guard cleared even on the abort path; PTFE never touched.
        assert ams._operation_active is False

    def test_follower_left_stopped_after_busy_abort(self):
        # The pre-load dance enables the follower forward (mirroring
        # _oams_load), but an aborted scan must always leave it STOPPED.
        ams, _ = _build_unit(values={"rfid_scan_ready_timeout": 5.0})
        ams.oams.busy_rejections = 10_000
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestRefusedLoad:
    def test_instant_error_refusal_aborts_without_retry(self):
        # A load that instantly completes with a non-success, non-busy code
        # (e.g. "no spool in bay") is a REFUSAL: reported, not retried.
        ams, coord = _build_unit()

        def dead_send(args):
            ams.oams.oams_load_spool_cmd.sent.append(args)
            ams.oams.action_status = None
            ams.oams.action_status_code = 4    # NO_SPOOL_IN_BAY

        ams.oams.oams_load_spool_cmd.send = dead_send
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert len(ams.oams.oams_load_spool_cmd.sent) == 1
        # Nothing engaged, so no unwind noise.
        assert ams.oams.unload_calls == 0


class TestScheduling:
    def test_disabled_does_not_schedule(self):
        ams, _ = _build_unit(values={"rfid_scan_on_insert": False})
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_enabled_schedules_timer(self):
        ams, _ = _build_unit()
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert "lane1" in ams._rfid_scan_timers

    def test_already_scanned_not_rescheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scanned.add("lane1")
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_operation_active_blocks_scheduling(self):
        ams, _ = _build_unit()
        ams._operation_active = True
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_cancel_clears_latch_and_timer(self):
        ams, _ = _build_unit()
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        ams._rfid_scanned.add("lane1")
        ams._cancel_rfid_scan("lane1")
        assert ams._rfid_scan_timers == {}
        assert "lane1" not in ams._rfid_scanned


class TestOperationActiveGuard:
    def test_scan_bails_if_operation_active(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams._operation_active = True
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []


class TestUnitReadyWait:
    def test_waits_out_encoder_motion(self):
        ams, _ = _build_unit()
        ticks = {"n": 0}
        real_monotonic = ams.afc.reactor.monotonic
        # Simulate: the encoder advances during the first few poll pauses
        # (firmware auto-stage still feeding), then holds still.
        base = ams.oams.encoder_clicks

        def fake_pause(until):
            AdvancingReactor.pause(ams.afc.reactor, until)
            if ticks["n"] < 3:
                ams.oams.encoder_clicks = base + ticks["n"]
                ticks["n"] += 1

        ams.afc.reactor.pause = fake_pause
        start = real_monotonic()
        assert ams._rfid_wait_for_unit_ready(10.0, quiet_time=1.0) is True
        # It must have waited at least quiet_time past the last movement.
        assert real_monotonic() - start >= 1.0

    def test_probes_instead_of_waiting_for_an_unprompted_report(self):
        # The firmware answers with its motor state only when spoken to. An
        # idle unit that has said nothing since boot must still be declared
        # ready PROMPTLY -- waiting passively for a spontaneous STOPPED report
        # burned the whole ready timeout (30s of dead air per insert scan).
        ams, _ = _build_unit()
        reactor = ams.afc.reactor
        start = reactor.monotonic()
        assert ams.oams.motion_status is None
        assert ams._rfid_wait_for_unit_ready(30.0, quiet_time=1.0) is True
        assert reactor.monotonic() - start < 5.0
        # Readiness came from a probe, and the probe is a harmless stop.
        assert ams.oams.follower_calls
        assert set(ams.oams.follower_calls) == {(0, 0)}

    def test_stale_active_state_is_not_ready(self):
        # A unit mid-stage keeps answering "reverse following / busy". The old
        # code only counted a FRESH active report, so a stale one let the wait
        # return ready ~1s in and the very next command took an ERROR_BUSY.
        ams, _ = _build_unit()
        ams.oams.staging_probes = 10_000
        assert ams._rfid_wait_for_unit_ready(3.0, quiet_time=1.0) is False

    def test_ready_once_staging_finishes(self):
        ams, _ = _build_unit()
        ams.oams.staging_probes = 3
        assert ams._rfid_wait_for_unit_ready(30.0, quiet_time=1.0) is True

    def test_no_load_sent_while_the_unit_is_still_staging(self):
        # The scan must not even reach the load while the motor is owned by
        # the firmware's insert routine.
        ams, _ = _build_unit(values={"rfid_scan_ready_timeout": 3.0})
        ams.oams.staging_probes = 10_000
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False
        assert ams.oams.oams_load_spool_cmd.sent == []
        assert ams._operation_active is False

    def test_fresh_satisfied_by_new_stopped_report(self):
        ams, _ = _build_unit()
        reactor = ams.afc.reactor

        def report_stopped(until):
            AdvancingReactor.pause(reactor, until)
            # Firmware reports STOPPED shortly after the wait begins.
            ams.oams.motion_status = OAMSStatus.STOPPED
            ams.oams.motion_status_time = reactor.monotonic()

        # Only the first pause plants the report; later pauses advance time.
        calls = {"n": 0}

        def fake_pause(until):
            if calls["n"] == 0:
                report_stopped(until)
            else:
                AdvancingReactor.pause(reactor, until)
            calls["n"] += 1

        reactor.pause = fake_pause
        assert ams._rfid_wait_for_unit_ready(10.0, quiet_time=0.5) is True


BLANK = {"uid": "AABB", "filament": None, "tag_type": "MifareUltralight"}


class TestHeldForAWrite:
    """AFC_RFID_WRITE / ENROLL LANE= run this same scan with a hold: the write
    happens where the scan stops on the tag, and the unwind follows."""

    def _unit(self, fields=([], [], ["aabb"]), reads=(BLANK,)):
        coord = FakeCoordinator(fields=[list(f) for f in fields],
                                full_reads=list(reads))
        return _build_unit(coord=coord)

    def test_a_blank_tag_is_held_for_the_write_then_unwound(self):
        ams, coord = self._unit()
        seen = []

        def hold(excluded):
            seen.append((ams.oams.unload_calls, ams._operation_active))

        assert ams._do_rfid_scan(ams.lanes["lane1"], hold=hold) is True
        assert seen == [(0, True)]           # written before the unwind
        assert coord.applied == []           # the write applies, not the scan
        assert ams.oams.unload_calls == 1
        assert ams._operation_active is False

    def test_the_sister_tag_is_excluded_from_the_write(self):
        ams, coord = self._unit(fields=[["5157e12"], ["5157e12"],
                                        ["5157e12", "aabb"]])
        got = []
        ams._do_rfid_scan(ams.lanes["lane1"], hold=got.append)
        assert got[0]("5157E12") is True
        assert got[0]("aabb") is False

    def test_no_tag_holds_nothing(self):
        ams, coord = self._unit(fields=[[]], reads=())
        held = []
        assert ams._do_rfid_scan(ams.lanes["lane1"], hold=held.append) is False
        assert held == []

    def test_a_failed_write_still_unwinds(self):
        ams, coord = self._unit()

        def hold(excluded):
            raise RuntimeError("SPI write failed")

        with pytest.raises(RuntimeError, match="SPI write failed"):
            ams._do_rfid_scan(ams.lanes["lane1"], hold=hold)
        assert ams.oams.unload_calls == 1
        assert ams._operation_active is False
        assert ams.oams.follower_calls[-1] == (0, 0)

    def test_without_a_hold_a_blank_tag_is_still_not_applied(self):
        ams, coord = self._unit(reads=(BLANK, BLANK, BLANK))
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False
        assert coord.applied == []


class TestWriteStagedByTheScan:
    """AFC_OpenAMS_rfid._stage_write_around: the write command's hook into the
    scan above, with the same refusals as AFC_OAMS_RFID_SCAN."""

    def _setup(self, fields=([], [], ["aabb"]), reads=(BLANK,)):
        from extras.AFC_OpenAMS_rfid import AFC_OpenAMS_rfid
        ams, scan_coord = _build_unit(coord=FakeCoordinator(
            fields=[list(f) for f in fields], full_reads=list(reads)))
        lane = ams.lanes["lane1"]
        lane.unit_obj = ams
        rfid = AFC_OpenAMS_rfid.__new__(AFC_OpenAMS_rfid)
        rfid.afc = types.SimpleNamespace(lanes={"lane1": lane})
        rfid._lane_slot = {"lane1": 0}
        rdr_a = types.SimpleNamespace(name="RFID_A", slots=[0, 1])
        rdr_b = types.SimpleNamespace(name="RFID_B", slots=[2, 3])
        rfid._slot_reader = {0: rdr_a, 1: rdr_a, 2: rdr_b, 3: rdr_b}
        return rfid, ams, lane, rdr_a, rdr_b

    def test_the_write_runs_inside_the_scan(self):
        rfid, ams, lane, rdr_a, _ = self._setup()
        ams._rfid_scanned.add("lane1")       # a scan on insert already ran
        calls = []
        rfid._stage_write_around(
            "lane1", rdr_a,
            lambda ex: calls.append(ams.oams.unload_calls))
        assert calls == [0]
        assert ams.oams.unload_calls == 1

    def test_the_other_readers_lane_is_refused(self):
        from extras.AFC_rfid_write import StageError
        rfid, ams, lane, _, rdr_b = self._setup()
        with pytest.raises(StageError, match="READER=oams:RFID_A"):
            rfid._stage_write_around("lane1", rdr_b, lambda ex: None)
        assert ams.oams.oams_load_spool_cmd.sent == []

    def test_a_lane_in_the_toolhead_is_refused(self):
        from extras.AFC_rfid_write import StageError
        rfid, ams, lane, rdr_a, _ = self._setup()
        lane.tool_loaded = True
        with pytest.raises(StageError, match="blocked"):
            rfid._stage_write_around("lane1", rdr_a, lambda ex: None)
        assert ams.oams.oams_load_spool_cmd.sent == []

    def test_no_tag_is_reported(self):
        from extras.AFC_rfid_write import StageError
        rfid, ams, lane, rdr_a, _ = self._setup(fields=[[]], reads=())
        with pytest.raises(StageError, match="came to rest"):
            rfid._stage_write_around(
                "lane1", rdr_a, lambda ex: pytest.fail("must not write"))
        assert ams._operation_active is False
