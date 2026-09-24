"""Unit tests for extras/AFC_BambuAMS_buffer.py."""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import pytest

from extras.AFC_BambuAMS_buffer import AFCBambuBuffer
from extras.AFC_buffer import AFCFPSBuffer
from extras.AFC_lane import AFCLaneState
from tests.conftest import MockAFC, MockConfig, MockPrinter


BAMBU_GATE_BUFFER = "Bambu_AMS_Buffer"


# Load 3 (HT, 2026-09-22) as (fps, odometer in metres), one bridge frame each.
# The real switch fired between the last two samples.
BAMBU_GATE_LOAD3: List[Tuple[float, float]] = [
    (0.02, 0.002), (0.02, 0.044), (0.02, 0.111), (0.02, 0.151),
    (0.02, 0.208), (0.02, 0.283), (0.02, 0.357), (0.02, 0.432),
    (0.02, 0.509), (0.02, 0.578), (0.02, 0.658), (0.02, 0.719),
    (0.02, 0.811), (0.02, 0.883), (0.02, 0.961), (0.02, 1.037),
    (0.02, 1.112), (0.02, 1.186), (0.02, 1.249), (0.02, 1.329),
    (0.02, 1.411), (0.02, 1.487), (0.02, 1.561), (0.03, 1.641),
    # friction plateau: the tube is filling and the buffer is loading up
    (0.35, 1.717), (0.35, 1.787), (0.35, 1.861), (0.35, 1.936),
    (0.35, 2.011), (0.36, 2.089), (0.37, 2.161), (0.38, 2.237),
    (0.39, 2.309),
    # at the gears
    (0.96, 2.364), (0.96, 2.428), (0.96, 2.483), (0.96, 2.533),
    (0.95, 2.538), (0.94, 2.542), (0.94, 2.543), (0.94, 2.545),
    (0.95, 2.545),
]


# Index of the first 0.96 sample, where a pressure-only sensor fires.
BAMBU_GATE_LOAD3_FIRST_HIGH = 33


class BambuGateAdc:
    """A Klipper-shaped MCU ADC that records how the buffer set it up."""

    def __init__(self) -> None:
        self.sample_setup: Optional[Tuple[float, float, int]] = None
        self.report_time: Optional[float] = None
        self.callback: Optional[Callable[..., None]] = None

    def setup_adc_sample(self, report_time: float, sample_time: float,
                         sample_count: int) -> None:
        self.sample_setup = (report_time, sample_time, sample_count)

    def setup_adc_callback(self, report_time: float,
                           callback: Callable[..., None]) -> None:
        self.report_time = report_time
        self.callback = callback


class BambuGatePins:
    """Klipper's `pins` object, handing out one ADC."""

    def __init__(self) -> None:
        self.adc = BambuGateAdc()
        self.pins: List[Tuple[str, str]] = []

    def setup_pin(self, pin_type: str, pin: str) -> BambuGateAdc:
        self.pins.append((pin_type, pin))
        return self.adc


class BambuGateLane:
    """A lane as the buffer reads it: name, buffer, tool_loaded and status."""

    def __init__(self, name: str, buffer_name: str = BAMBU_GATE_BUFFER,
                 tool_loaded: bool = False,
                 status: AFCLaneState = AFCLaneState.LOADED) -> None:
        self.name = name
        self.buffer_name = buffer_name
        self.tool_loaded = tool_loaded
        self.status = status


class BambuGateUnit:
    """An AFC_BambuAMS unit as the buffer reads it: name, lanes, odometer."""

    def __init__(self, name: str, buffer_name: str = BAMBU_GATE_BUFFER,
                 lane_name: str = "lane12") -> None:
        self.name = name
        self.lanes: Dict[str, BambuGateLane] = {
            lane_name: BambuGateLane(lane_name, buffer_name)}
        # Metres, as the traces were captured; None until a frame lands.
        self.odom_m: Optional[float] = None

    def _odom_now_mm(self) -> Optional[float]:
        # Rounded to 0.1 mm, so the traces' +1/+2 mm steps are exact.
        if self.odom_m is None:
            return None
        return round(self.odom_m * 1000.0, 1)


def make_bambu_gate_buffer(options: Optional[Dict[str, Any]] = None,
                           units: Sequence[str] = ("Bambu_AMS_1",),
                           prep_done: bool = True
                           ) -> Tuple[AFCBambuBuffer, List[BambuGateUnit]]:
    """
    Build an AFCBambuBuffer through its real __init__, with units on the printer.

    :param options: config values over `type: bambu` and the ADC pin
    :param units: names of AFC_BambuAMS units whose lane names this buffer
    :param prep_done: whether AFC's PREP has run
    :return: the buffer and its units, in the order given
    """
    afc = MockAFC()
    afc.prep_done = prep_done
    printer = MockPrinter(afc=afc)
    printer._objects["pins"] = BambuGatePins()
    unit_objs = []
    for name in units:
        unit = BambuGateUnit(name)
        printer._objects[f"AFC_BambuAMS {name}"] = unit
        unit_objs.append(unit)
    values: Dict[str, Any] = {"type": "bambu", "adc_pin": "bambu_buffer:fps"}
    values.update(options or {})
    config = MockConfig(name=f"AFC_buffer {BAMBU_GATE_BUFFER}", printer=printer,
                        values=values)
    return AFCBambuBuffer(config), unit_objs


def add_bambu_gate_unit(buf: AFCBambuBuffer, name: str,
                        buffer_name: str = BAMBU_GATE_BUFFER,
                        lane_name: str = "lane12") -> BambuGateUnit:
    """
    Register a unit on the buffer's printer after construction, as a late claim.

    :param buf: the buffer whose printer gains the unit
    :param name: the unit's name
    :param buffer_name: the buffer its lane names
    :param lane_name: its lane's name
    :return: the new unit
    """
    unit = BambuGateUnit(name, buffer_name, lane_name)
    buf.printer._objects[f"AFC_BambuAMS {name}"] = unit
    return unit


def spend_bambu_bind_grace(buf: AFCBambuBuffer) -> None:
    """
    Put the buffer past its bind grace, so a missing unit means no unit.

    :param buf: the buffer to age
    """
    buf._first_sample_t = -(buf.odom_bind_grace_seconds + 1.0)
    buf._prep_seen_t = buf._first_sample_t


def feed_bambu_gate(buf: AFCBambuBuffer, unit: Optional[BambuGateUnit],
                    samples: Sequence[Tuple[float, Optional[float]]]) -> List[bool]:
    """
    Replay (fps, odometer metres) samples 0.25 s apart through the ADC callback.

    The clock continues from the buffer's own, so a second feed never runs time
    backwards.

    :param buf: the buffer under test
    :param unit: the unit the odometer readings belong to, or None
    :param samples: (fps, odom_m) pairs; an odom_m of None leaves it alone
    :return: advance_state after each sample
    """
    states = []
    t = buf._now_t
    for fps, odom_m in samples:
        if unit is not None and odom_m is not None:
            unit.odom_m = odom_m
        t += 0.25
        buf._adc_callback(t, fps)
        states.append(buf.advance_state)
    return states


# The line load 3 logs when the gate latches: the +2 mm step at 10.25 s ends
# 0.5 s of stillness, and the buffer has settled near 0.94.
BAMBU_GATE_LOAD3_DETECTED = (
    "info",
    "Bambu_AMS_Buffer: load detected at 10.25 -- fps 0.94 (smoothed 0.94), "
    "odometer still for 0.50s [Bambu_AMS_1=2545.0mm]")


BAMBU_GATE_NO_ODOMETER = (
    "warning",
    "Bambu_AMS_Buffer: no Bambu unit on this buffer reports an odometer, so a "
    "load is being judged on buffer pressure alone. Bowden friction can read "
    "as filament that way. Set odom_required: True to refuse instead.")


class TestAFCBambuBufferInit:
    """__init__: the gate's options and its starting state."""

    def test_the_gate_options_default_when_unset(self):
        buf, _units = make_bambu_gate_buffer()

        assert buf.odom_eps_mm == 3.0
        assert buf.odom_still_seconds == 0.5
        assert buf.odom_required is False
        assert buf.unload_confirm_seconds == 2.0
        assert buf.odom_move_ttl_seconds == 5.0
        assert buf.odom_bind_grace_seconds == 10.0
        assert buf.resting_load_seconds == 3.0
        assert buf.record_check_seconds == 60.0
        assert buf.logger.messages == []

    def test_every_gate_option_is_read_from_config(self):
        buf, _units = make_bambu_gate_buffer({
            "odom_eps_mm": 1.5, "odom_still_seconds": 0.75,
            "odom_required": True, "unload_confirm_seconds": 4.0,
            "odom_move_ttl_seconds": 8.0, "odom_bind_grace_seconds": 20.0,
            "resting_load_seconds": 6.0, "record_check_seconds": 30.0})

        assert buf.odom_eps_mm == 1.5
        assert buf.odom_still_seconds == 0.75
        assert buf.odom_required is True
        assert buf.unload_confirm_seconds == 4.0
        assert buf.odom_move_ttl_seconds == 8.0
        assert buf.odom_bind_grace_seconds == 20.0
        assert buf.resting_load_seconds == 6.0
        assert buf.record_check_seconds == 30.0
        assert buf.logger.messages == []

    def test_the_gate_starts_having_seen_nothing(self):
        buf, _units = make_bambu_gate_buffer()

        assert buf._odom_units is None
        assert buf._odom_prev == {}
        assert buf._first_sample_t is None
        assert buf._prep_seen_t is None
        assert buf._last_move_t is None
        assert buf._now_t == 0.0
        assert buf._tension_since is None
        assert buf._compressed_since is None
        assert buf._record_checked is False
        assert buf._odom_moved is False
        assert buf._gate_latched is False
        assert buf._odom_warned is False
        # The ADC delivers its samples to the gate, not to the plain FPS callback.
        adc = buf.printer.lookup_object("pins").adc
        assert adc.callback.__func__ is AFCBambuBuffer._adc_callback
        assert adc.callback.__self__ is buf
        assert buf.logger.messages == []

    def test_the_lapse_is_far_longer_than_a_real_braking_tail(self):
        # The real tail between the filament stopping and the switch closing is
        # one or two samples on all three logged loads; the lapse must never
        # reach into that.
        buf, (unit,) = make_bambu_gate_buffer()
        assert buf.odom_move_ttl_seconds == 5.0

        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)

        assert buf._gate_latched is True
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

        # A lapse as short as one frame drops the movement before the 0.5 s of
        # stillness is up, and the same trace is never detected.
        short, (short_unit,) = make_bambu_gate_buffer({"odom_move_ttl_seconds": 0.25})
        states = feed_bambu_gate(short, short_unit, BAMBU_GATE_LOAD3)
        assert states == [False] * len(BAMBU_GATE_LOAD3)
        assert short._gate_latched is False
        assert short._odom_moved is False
        assert short.logger.messages == []


class TestAFCBambuBufferBoundUnits:
    """_bound_units: which AFC_BambuAMS units' odometers belong to this buffer."""

    class NoOdometerUnit:
        """A unit-named section whose lane is on this buffer but has no odometer."""

        def __init__(self) -> None:
            self.name = "Bambu_AMS_2"
            self.lanes = {"lane20": BambuGateLane("lane20")}
            self._odom_now_mm: Optional[float] = 12.0

    def test_it_finds_its_units_by_their_lanes_buffer_name(self):
        buf, units = make_bambu_gate_buffer(units=("Bambu_AMS_1", "Bambu_AMS_HT_1"))

        assert buf._bound_units() == units
        assert buf._odom_units == units
        assert buf.logger.messages == []

    def test_a_found_set_is_remembered(self):
        buf, units = make_bambu_gate_buffer()
        first = buf._bound_units()
        del buf.printer._objects["AFC_BambuAMS Bambu_AMS_1"]

        assert buf._bound_units() is first
        assert first == units
        assert buf.logger.messages == []

    def test_a_scan_that_finds_nothing_is_not_remembered_as_never(self):
        # Live fault, printer 1, 2026-09-22: the pool claims its units a moment
        # after klippy:ready, and caching the first empty scan ran the whole
        # session on pressure alone.
        buf, _units = make_bambu_gate_buffer(units=())
        assert buf._bound_units() == []
        assert buf._odom_units == []

        late = add_bambu_gate_unit(buf, "Bambu_AMS_1")

        assert buf._bound_units() == [late]
        assert buf._odom_units == [late]
        assert buf.logger.messages == []

    def test_a_late_unit_clears_the_degrade_warning_latch(self):
        # So a genuine later loss of every odometer says so again.
        buf, _units = make_bambu_gate_buffer(units=())
        spend_bambu_bind_grace(buf)
        assert buf._odom_confirms() is True
        assert buf._odom_warned is True
        late = add_bambu_gate_unit(buf, "Bambu_AMS_1")

        assert buf._bound_units() == [late]

        assert buf._odom_warned is False
        assert buf.logger.messages == [BAMBU_GATE_NO_ODOMETER]

    def test_an_empty_scan_leaves_the_warning_latch_set(self):
        buf, _units = make_bambu_gate_buffer(units=())
        spend_bambu_bind_grace(buf)
        assert buf._odom_confirms() is True

        assert buf._bound_units() == []

        assert buf._odom_warned is True
        assert buf.logger.messages == [BAMBU_GATE_NO_ODOMETER]

    def test_a_unit_on_a_different_buffer_is_not_bound(self):
        buf, (unit,) = make_bambu_gate_buffer()
        add_bambu_gate_unit(buf, "Bambu_AMS_9", "Some_Other_Buffer", "lane90")

        assert buf._bound_units() == [unit]
        assert buf.logger.messages == []

    def test_only_bambu_units_with_an_odometer_and_lanes_are_bound(self):
        buf, (unit,) = make_bambu_gate_buffer()
        printer_objects = buf.printer._objects
        printer_objects["AFC_BambuAMS Bambu_AMS_2"] = self.NoOdometerUnit()
        laneless = add_bambu_gate_unit(buf, "Bambu_AMS_3")
        laneless.lanes = None
        # An odometer-bearing object on this buffer, under another section prefix.
        printer_objects["AFC_BoxTurtle Turtle_1"] = BambuGateUnit("Turtle_1")

        assert buf._bound_units() == [unit]
        assert buf.logger.messages == []

    def test_a_failing_scan_binds_nothing_and_is_retried(self):
        buf, units = make_bambu_gate_buffer()

        def lookup_objects(module: Optional[str] = None) -> list:
            raise RuntimeError("printer objects unavailable")

        buf.printer.lookup_objects = lookup_objects

        assert buf._bound_units() == []
        assert buf._odom_units is None

        del buf.printer.lookup_objects       # the printer's own scan again
        assert buf._bound_units() == units
        assert buf.logger.messages == []


class TestAFCBambuBufferUpdateOdom:
    """_update_odom: folding each sample's odometer readings into the stillness run."""

    class FailingUnit(BambuGateUnit):
        """A bound unit whose odometer read raises."""

        def _odom_now_mm(self) -> Optional[float]:
            raise RuntimeError("bridge frame unreadable")

    # The braking tail of load 3, odometer in metres: +5, +4, +1, +2 and 0 mm.
    BRAKE_TAIL = [2.533, 2.538, 2.542, 2.543, 2.545, 2.545]

    @classmethod
    def stills_through_the_tail(cls, buf: AFCBambuBuffer,
                                unit: BambuGateUnit) -> List[float]:
        """
        Step the braking tail through _update_odom alone, 0.25 s per frame.

        :param buf: the buffer under test
        :param unit: its unit
        :return: _odom_still_for() after each frame
        """
        stills = []
        t = buf._now_t
        for odom_m in cls.BRAKE_TAIL:
            unit.odom_m = odom_m
            t += 0.25
            buf._now_t = t
            buf._update_odom()
            stills.append(buf._odom_still_for())
        return stills

    @pytest.mark.parametrize("eps, stills", [
        (1.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.25]),
        (2.0, [0.0, 0.0, 0.0, 0.25, 0.0, 0.25]),
    ])
    def test_a_small_epsilon_flickers_on_the_braking_tail(self, eps, stills):
        """
        Why the default is 3 mm and not 1 or 2: at those the +1/+2 mm samples
        in the tail read as movement and reset the run.
        """
        buf, (unit,) = make_bambu_gate_buffer({"odom_eps_mm": eps,
                                               "odom_still_seconds": 0.5})
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3[:BAMBU_GATE_LOAD3_FIRST_HIGH])
        assert buf._now_t == 8.25
        assert buf._odom_moved is True

        assert self.stills_through_the_tail(buf, unit) == stills

        # The +2 mm step at 9.5 s was the last "movement".
        assert buf._last_move_t == 9.5
        assert buf._odom_prev == {"Bambu_AMS_1": 2545.0}
        assert buf.logger.messages == []

    def test_the_default_epsilon_resolves_the_braking_tail(self):
        buf, (unit,) = make_bambu_gate_buffer()
        assert buf.odom_eps_mm == 3.0
        assert buf.odom_still_seconds == 0.5
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3[:BAMBU_GATE_LOAD3_FIRST_HIGH])

        assert self.stills_through_the_tail(buf, unit) == [0.0, 0.0, 0.0, 0.25, 0.5, 0.75]

        # The +4 mm step at 9.0 s was the last movement; +1 and +2 are still.
        assert buf._last_move_t == 9.0
        assert buf._odom_moved is True
        assert buf.logger.messages == []

    def test_a_first_reading_is_not_movement(self):
        buf, (unit,) = make_bambu_gate_buffer()
        unit.odom_m = 2.0
        buf._now_t = 1.0

        buf._update_odom()

        assert buf._odom_prev == {"Bambu_AMS_1": 2000.0}
        assert buf._odom_moved is False
        assert buf._last_move_t is None

        # Below the epsilon is still; exactly the epsilon is movement.
        unit.odom_m = 2.002
        buf._now_t = 1.25
        buf._update_odom()
        assert buf._odom_moved is False
        unit.odom_m = 2.005
        buf._now_t = 1.5
        buf._update_odom()
        assert buf._odom_moved is True
        assert buf._last_move_t == 1.5
        assert buf._odom_prev == {"Bambu_AMS_1": 2005.0}
        assert buf.logger.messages == []

    def test_a_failing_odometer_is_skipped_and_another_unit_still_counts(self):
        buf, (unit,) = make_bambu_gate_buffer()
        broken = self.FailingUnit("Bambu_AMS_HT_1")
        buf.printer._objects["AFC_BambuAMS Bambu_AMS_HT_1"] = broken
        unit.odom_m = 1.0
        buf._now_t = 1.0
        buf._update_odom()
        unit.odom_m = 1.074
        buf._now_t = 1.25

        buf._update_odom()

        assert buf._bound_units() == [unit, broken]
        assert buf._odom_prev == {"Bambu_AMS_1": 1074.0}
        assert buf._odom_moved is True
        assert buf._last_move_t == 1.25
        assert buf.logger.messages == []

    def test_a_round_with_no_reading_leaves_the_movement_record_alone(self):
        buf, (unit,) = make_bambu_gate_buffer()
        buf._odom_moved = True
        buf._last_move_t = 1.0
        # Far past the TTL: a round that got as far as the lapse check drops it.
        buf._now_t = 10.0

        buf._update_odom()

        assert unit.odom_m is None
        assert buf._odom_prev == {}
        assert buf._odom_moved is True
        assert buf._last_move_t == 1.0
        assert buf.logger.messages == []

    def test_movement_lapses_only_once_it_is_older_than_the_ttl(self):
        buf, (unit,) = make_bambu_gate_buffer()
        unit.odom_m = 2.5
        buf._now_t = 1.0
        buf._update_odom()
        buf._odom_moved = True
        buf._last_move_t = 1.0

        buf._now_t = 6.0
        buf._update_odom()
        assert buf._odom_moved is True

        buf._now_t = 6.25
        buf._update_odom()
        assert buf._odom_moved is False
        assert buf._last_move_t == 1.0
        assert buf.logger.messages == []


class TestAFCBambuBufferOdomConfirms:
    """_odom_confirms: whether the odometer agrees the filament stopped at the gears."""

    @staticmethod
    def sample_until(buf: AFCBambuBuffer, t: float, until: float,
                     fps: float = 0.46) -> float:
        """
        Feed bare-time samples 0.25 s apart while the clock is below `until`.

        :param buf: the buffer under test
        :param t: the time of the last sample fed
        :param until: keep sampling while the clock is below this
        :param fps: the reading every sample carries
        :return: the time of the last sample fed
        """
        while t < until:
            t += 0.25
            buf._adc_callback(t, fps)
        return t

    def test_a_bound_unit_that_has_not_reported_yet_is_not_a_missing_odometer(self):
        # Live, printer 1, 2026-09-22, seconds after a restart: the units were
        # bound but no bridge frame had landed, and "nothing read back yet"
        # took the "no odometer" branch, handing the gate to pressure alone.
        buf, (unit,) = make_bambu_gate_buffer()
        # Past the bind wait, an unbound buffer would degrade here.
        spend_bambu_bind_grace(buf)
        unit.odom_m = None

        assert buf._bound_units() == [unit]
        assert buf._odom_confirms() is False
        assert buf._odom_prev == {}
        assert buf._odom_warned is False
        assert buf.logger.messages == []

    def test_it_needs_movement_and_then_stillness(self):
        buf, (unit,) = make_bambu_gate_buffer()
        unit.odom_m = 2.5
        buf._now_t = 2.0
        buf._update_odom()

        # Still long enough, but never moved.
        buf._last_move_t = 1.0
        assert buf._odom_confirms() is False
        # Moved, but only 0.25 s ago.
        buf._odom_moved = True
        buf._last_move_t = 1.75
        assert buf._odom_confirms() is False
        # Moved, then still for exactly odom_still_seconds.
        buf._last_move_t = 1.5
        assert buf._odom_confirms() is True
        assert buf.logger.messages == []

    def test_a_buffer_that_has_never_sampled_is_still_waiting(self):
        buf, _units = make_bambu_gate_buffer(units=())

        assert buf._odom_confirms() is False

        assert buf._first_sample_t is None
        assert buf._odom_warned is False
        assert buf.logger.messages == []

    def test_no_degrade_warning_while_the_pool_is_still_claiming_units(self):
        # The warning fired in the seconds between Klipper being ready and the
        # pool binding a unit, on every restart, and read as the gate being
        # broken.
        buf, _units = make_bambu_gate_buffer(units=())
        t = self.sample_until(buf, 0.0, buf.odom_bind_grace_seconds - 0.5)
        assert t == 9.5
        assert buf._first_sample_t == 0.25
        assert buf._prep_seen_t == 0.25

        # It refuses rather than degrading while it waits, so a load landing in
        # that window is not judged on pressure alone either.
        assert buf._odom_confirms() is False
        assert buf._odom_warned is False
        assert buf.logger.messages == []

    def test_a_buffer_that_really_has_no_unit_still_says_so(self):
        # The warning must not be lost, only delayed until it is true.
        buf, _units = make_bambu_gate_buffer(units=())
        t = self.sample_until(buf, 0.0, buf.odom_bind_grace_seconds + 0.5)
        assert t == 10.5

        assert buf._odom_confirms() is True

        assert buf._odom_warned is True
        assert buf.logger.messages == [BAMBU_GATE_NO_ODOMETER]

    def test_the_degrade_warning_is_given_once(self):
        buf, _units = make_bambu_gate_buffer(units=())
        spend_bambu_bind_grace(buf)
        assert buf._odom_confirms() is True

        assert buf._odom_confirms() is True

        assert buf.logger.messages == [BAMBU_GATE_NO_ODOMETER]

    def test_odom_required_refuses_quietly(self):
        buf, _units = make_bambu_gate_buffer({"odom_required": True}, units=())
        spend_bambu_bind_grace(buf)

        assert buf._odom_confirms() is False

        assert buf._odom_warned is False
        assert buf.logger.messages == []

    def test_the_bind_wait_starts_once_prep_has_run(self):
        # The pool claims its units only after AFC's PREP, which can come well
        # after the buffer's first sample. A wait counted from the first sample
        # ran out before the claims on a real boot, and warned every time.
        buf, _units = make_bambu_gate_buffer(units=(), prep_done=False)
        t = self.sample_until(buf, 0.0, buf.odom_bind_grace_seconds + 5.0)
        assert t == 15.0
        assert buf._prep_seen_t is None
        assert buf._odom_confirms() is False

        buf.afc.prep_done = True
        t = self.sample_until(buf, t, 18.25)
        assert buf._first_sample_t == 0.25
        assert buf._prep_seen_t == 15.25
        # 18 s after the first sample but 3 s after PREP: still waiting.
        assert buf._odom_confirms() is False

        unit = add_bambu_gate_unit(buf, "Bambu_AMS_1")
        self.sample_until(buf, t, 15.25 + buf.odom_bind_grace_seconds * 2)

        assert buf._bound_units() == [unit]
        assert buf._odom_confirms() is False
        assert buf._odom_warned is False
        assert buf.logger.messages == []

    def test_a_buffer_with_no_unit_says_so_once_the_wait_after_prep_is_spent(self):
        buf, _units = make_bambu_gate_buffer(units=(), prep_done=False)
        t = self.sample_until(buf, 0.0, 5.0)
        buf.afc.prep_done = True
        t = self.sample_until(buf, t, 5.25 + buf.odom_bind_grace_seconds - 0.5)
        assert buf._prep_seen_t == 5.25
        assert t == 14.75
        assert buf._odom_confirms() is False
        assert buf.logger.messages == []

        t = self.sample_until(buf, t, 5.25 + buf.odom_bind_grace_seconds + 0.5)

        assert t == 15.75
        assert buf._odom_confirms() is True
        assert buf.logger.messages == [BAMBU_GATE_NO_ODOMETER]


class TestAFCBambuBufferAdcCallback:
    """_adc_callback: FPS pressure, then the odometer gate on top of it."""

    # The lines _adopt_recorded_load writes for lane28.
    ADOPTED_AT_081 = (
        "info",
        "Bambu_AMS_Buffer: adopting lane28 as loaded at the toolhead from AFC's "
        "own record, buffer agreeing at 0.81")
    REFUSED_AT_002 = (
        "warning",
        "Bambu_AMS_Buffer: AFC records lane28 loaded to the toolhead, but the "
        "buffer is at max tension (0.02), which is what an empty path reads "
        "like. Not adopting it -- the next load will settle this.")
    # The latch releases 2.0 s after the smoothed reading first reaches 0.1.
    RELEASED = (
        "debug",
        "Bambu_AMS_Buffer: buffer has been at max tension for 2.0s, releasing "
        "the load latch")

    class IdleTimeout:
        """Klipper's idle_timeout, recording the eventtime each sensor change carries."""

        def __init__(self) -> None:
            self.eventtimes: List[Any] = []

        def get_status(self, eventtime: Any) -> Dict[str, str]:
            self.eventtimes.append(eventtime)
            return {"state": "Ready"}

    @staticmethod
    def feed_at_10hz(buf: AFCBambuBuffer, unit: BambuGateUnit, fps: float,
                     callbacks: int) -> Tuple[List[bool], float]:
        """
        Feed at the ADC's 10 Hz while the bridge moves the odometer 74 mm per 250 ms frame.

        :param buf: the buffer under test
        :param unit: its unit
        :param fps: the reading every callback carries
        :param callbacks: how many callbacks to run
        :return: buffer_triggered after each callback, and the longest stillness seen
        """
        t = 0.0
        odom_m = 0.10
        frame_due = 0.0
        fired = []
        worst = 0.0
        for _ in range(callbacks):
            t += 0.1
            if t >= frame_due:
                odom_m += 0.074
                frame_due += 0.25
            unit.odom_m = odom_m
            buf._adc_callback(t, fps)
            fired.append(buf.buffer_triggered)
            worst = max(worst, buf._odom_still_for())
        return fired, worst

    @staticmethod
    def with_record(buf: AFCBambuBuffer, loaded: bool) -> BambuGateLane:
        """
        Give the buffer lane28, with AFC's tool_loaded record as given.

        :param buf: the buffer under test
        :param loaded: the lane's tool_loaded
        :return: the lane
        """
        lane = BambuGateLane("lane28", tool_loaded=loaded)
        buf.lanes = {"lane28": lane}
        return lane

    def test_movement_stops_counting_once_the_unit_has_long_since_stopped(self):
        # Live, printer 1, 2026-09-22: idle at the gate, odom_moved had latched
        # true with the unit still for half a minute, so "moved, then stopped"
        # was really just "not moving".
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, [(0.46, 0.100), (0.46, 0.140), (0.46, 0.180)])
        assert buf._odom_moved is True
        assert buf._last_move_t == 0.75

        # At 5.75 s the last move is exactly the 5.0 s TTL old, and still counts.
        feed_bambu_gate(buf, unit, [(0.46, 0.180)] * 20)
        assert buf._now_t == 5.75
        assert buf._odom_moved is True

        feed_bambu_gate(buf, unit, [(0.46, 0.180)])

        assert buf._odom_moved is False
        assert buf._last_move_t == 0.75
        assert buf._odom_confirms() is False
        assert buf.logger.messages == []

    def test_friction_plateau_alone_never_reports_filament(self):
        """
        The 0.35-0.39 plateau is bowden drag with a metre of tube still to fill.
        A buffer-only sensor reads that as arrival; this one must not.
        """
        buf, (unit,) = make_bambu_gate_buffer()

        states = feed_bambu_gate(buf, unit,
                                 BAMBU_GATE_LOAD3[:BAMBU_GATE_LOAD3_FIRST_HIGH])

        assert states == [False] * BAMBU_GATE_LOAD3_FIRST_HIGH
        assert buf._odom_moved is True
        assert buf._compressed_since is None
        assert buf._gate_latched is False
        assert buf.logger.messages == []

    def test_the_load_is_detected_before_the_real_switch_fired(self):
        """
        The real switch fired between the last two samples of load 3. The
        gate fires on the sample before, after the buffer went high.
        """
        buf, (unit,) = make_bambu_gate_buffer()

        states = feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)

        assert states == [False] * 40 + [True] * 2
        assert buf._gate_latched is True
        assert buf.fila_adv.runout_helper.filament_present is True
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_pressure_alone_would_have_fired_two_seconds_early(self):
        """
        A plain FPS buffer on this trace calls it at the first 0.96, seven
        samples (1.75 s) before the gated one does.
        """
        buf, (unit,) = make_bambu_gate_buffer()
        # A buffer with no Bambu unit on it at all degrades to pressure alone.
        plain, _units = make_bambu_gate_buffer(units=())
        spend_bambu_bind_grace(plain)

        gated = feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        ungated = feed_bambu_gate(plain, None, BAMBU_GATE_LOAD3)

        assert ungated == [False] * 33 + [True] * 9
        assert gated == [False] * 40 + [True] * 2
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]
        assert plain.logger.messages == [
            BAMBU_GATE_NO_ODOMETER,
            ("info", "Bambu_AMS_Buffer: load detected at 8.50 -- fps 0.96 "
                     "(smoothed 0.79), odometer still for 0.00s []")]

    def test_a_bare_value_sample_still_carries_a_moving_clock(self):
        """
        Live, printer 1, 2026-09-22: the virtual chip delivers bare values with
        no time attached, so every sample was stamped 0.0, the stillness clock
        never advanced and the gate could not fire. The reactor is the fallback
        clock, as it is for the base class.
        """
        buf, (unit,) = make_bambu_gate_buffer()
        unit.odom_m = 0.10
        buf.reactor._monotonic = 100.0

        buf._adc_callback(0.46)

        assert buf._now_t == 100.0
        assert buf._first_sample_t == 100.0
        assert buf._prep_seen_t == 100.0
        assert buf.fps_value == 0.46
        assert buf._odom_prev == {"Bambu_AMS_1": 100.0}

        buf.reactor._monotonic = 100.6
        buf._adc_callback(0.46)

        assert buf._now_t == 100.6
        assert buf._first_sample_t == 100.0
        assert buf.logger.messages == []

    def test_a_bare_value_feed_can_still_detect_a_load(self):
        # On the bare-value path the gate must still see movement, then
        # stillness, then latch.
        buf, (unit,) = make_bambu_gate_buffer()
        t = 100.0
        for fps, odom_m in BAMBU_GATE_LOAD3:
            unit.odom_m = odom_m
            t += 0.25
            buf.reactor._monotonic = t
            buf._adc_callback(fps)

        assert buf._gate_latched is True
        assert buf.logger.messages == [
            ("info", "Bambu_AMS_Buffer: load detected at 110.25 -- fps 0.94 "
                     "(smoothed 0.94), odometer still for 0.50s "
                     "[Bambu_AMS_1=2545.0mm]")]

    def test_a_list_of_samples_is_timed_by_its_last_one(self):
        buf, (unit,) = make_bambu_gate_buffer()
        idle = self.IdleTimeout()
        buf.printer._objects["idle_timeout"] = idle
        unit.odom_m = 0.10

        buf._adc_callback([(3.0, 0.40), (3.5, 0.96)])

        assert buf._now_t == 3.5
        assert buf._first_sample_t == 3.5
        assert buf.fps_value == 0.96
        # Compressed but never moved: the gate clears the sensor the pressure
        # set, at the sample's own time rather than the list's.
        assert buf.advance_state is False
        assert buf.fila_adv.runout_helper.filament_present is False
        assert idle.eventtimes == [3.5, 3.5]
        assert buf.logger.messages == []

    def test_a_bare_value_refused_by_the_gate_takes_the_reactor_clock(self):
        buf, (unit,) = make_bambu_gate_buffer()
        idle = self.IdleTimeout()
        buf.printer._objects["idle_timeout"] = idle
        unit.odom_m = 0.10
        buf.reactor._monotonic = 9.0

        buf._adc_callback(0.96)

        # Compressed but never moved: the gate clears the sensor at the
        # reactor time, not at the reading.
        assert buf.advance_state is False
        assert buf.fila_adv.runout_helper.filament_present is False
        assert idle.eventtimes == [9.0, 9.0]
        assert buf.logger.messages == []

    def test_an_empty_sample_list_takes_the_reactor_clock(self):
        buf, _units = make_bambu_gate_buffer()
        buf.reactor._monotonic = 42.0

        buf._adc_callback([])

        assert buf._now_t == 42.0
        assert buf._first_sample_t == 42.0
        assert buf.fps_value == 0.0
        assert buf.logger.messages == []

    def test_an_empty_sample_list_refused_by_the_gate_takes_no_list_time(self):
        # The base callback returns early on [], so the pressure set before it
        # stands and the gate refuses it with an empty list still in hand.
        buf, _units = make_bambu_gate_buffer()
        idle = self.IdleTimeout()
        buf.printer._objects["idle_timeout"] = idle
        buf.reactor._monotonic = 7.0
        buf.advance_state = True
        buf.smoothed_fps = 0.9
        buf.fila_adv.runout_helper.filament_present = True

        buf._adc_callback([])

        assert buf._now_t == 7.0
        assert buf._compressed_since == 7.0
        assert buf._gate_latched is False
        assert buf.advance_state is False
        assert buf._advance_latched is False
        assert buf.fila_adv.runout_helper.filament_present is False
        # The sensors get the reactor time, not the empty list.
        assert idle.eventtimes == [7.0]
        assert buf.logger.messages == []

    def test_a_latched_load_holds_through_an_empty_sample_list(self):
        buf, _units = make_bambu_gate_buffer()
        idle = self.IdleTimeout()
        buf.printer._objects["idle_timeout"] = idle
        buf.reactor._monotonic = 7.0
        buf._gate_latched = True
        buf.smoothed_fps = 0.9
        assert buf.advance_state is False
        assert buf.fila_adv.runout_helper.filament_present is False

        buf._adc_callback([])

        assert buf._tension_since is None
        assert buf._gate_latched is True
        assert buf.advance_state is True
        assert buf.fila_adv.runout_helper.filament_present is True
        assert idle.eventtimes == [7.0]
        assert buf.logger.messages == []

    def test_a_latched_load_holds_through_a_list_sample(self):
        buf, _units = make_bambu_gate_buffer()
        idle = self.IdleTimeout()
        buf.printer._objects["idle_timeout"] = idle
        self.with_record(buf, True)
        buf._adc_callback(0.25, 0.94)
        assert buf._gate_latched is True

        # 0.3 sits inside the deadband, so pressure alone says not advancing.
        buf._adc_callback([(0.5, 0.30)])

        assert buf.advance_state is True
        assert buf.fila_adv.runout_helper.filament_present is True
        assert idle.eventtimes == [0.25, 0.5, 0.5]
        assert buf.logger.messages == [self.ADOPTED_AT_081]

    def test_oversampling_a_moving_feed_is_not_mistaken_for_stillness(self):
        """
        The ADC runs at about 10 Hz but the bridge refreshes the odometer every
        250 ms, so mid-feed two or three callbacks read the same value. A
        stillness test that counted callbacks scored that as stopped and, with
        the buffer compressed, called the load. Here the filament moves 74 mm
        per frame with the buffer compressed throughout, and nothing may fire.
        """
        buf, (unit,) = make_bambu_gate_buffer()

        fired, _worst = self.feed_at_10hz(buf, unit, 0.96, 120)

        assert fired == [False] * 120
        # Pressure asked every time; the gate refused every time.
        assert buf._compressed_since == 0.1
        assert buf.advance_state is False
        assert buf._gate_latched is False
        # It saw plenty of movement, it just never saw it stop.
        assert buf._odom_moved is True
        assert buf.logger.messages == []

    def test_a_feed_at_speed_never_looks_still_for_the_full_window(self):
        # While frames keep arriving with movement in them, the stillness
        # clock tops out at two callbacks between frames.
        buf, (unit,) = make_bambu_gate_buffer()

        _fired, worst = self.feed_at_10hz(buf, unit, 0.39, 200)

        assert worst == pytest.approx(0.2)
        assert worst < buf.odom_still_seconds
        assert buf.logger.messages == []

    def test_a_dead_stop_mid_feed_does_not_report_filament(self):
        """
        Load 2 (AMS 2, 2026-09-22): the odometer repeated 0.558 m for a whole
        sample with a metre still to run. The odometer alone calls that
        arrival; the buffer at 0.02 is what rejects it.
        """
        buf, (unit,) = make_bambu_gate_buffer()

        states = feed_bambu_gate(buf, unit, [
            (0.02, 0.412), (0.02, 0.484), (0.02, 0.558),
            (0.02, 0.558), (0.02, 0.558),
        ])
        assert states == [False] * 5
        assert buf._odom_confirms() is True

        states = feed_bambu_gate(buf, unit, [(0.02, 0.701), (0.02, 0.774)])

        assert states == [False] * 2
        assert buf._gate_latched is False
        assert buf.logger.messages == []

    def test_a_compressed_buffer_that_never_moved_does_not_report_filament(self):
        """
        Moved first, then still. Without that, a gate opening before the unit
        starts calls an unstarted feed "arrived".
        """
        buf, (unit,) = make_bambu_gate_buffer()

        states = feed_bambu_gate(buf, unit, [(0.96, 1.500)] * 6)

        assert states == [False] * 6
        assert buf._odom_prev == {"Bambu_AMS_1": 1500.0}
        assert buf._odom_moved is False
        assert buf._compressed_since == 0.25
        assert buf.fila_adv.runout_helper.filament_present is False
        assert buf.logger.messages == []

    def test_a_refused_sample_clears_the_fps_load_latch(self):
        # Or the next tick re-latches off the pressure alone.
        buf, (unit,) = make_bambu_gate_buffer()
        buf._latch_enabled = True

        feed_bambu_gate(buf, unit, [(0.96, 1.500)])

        assert buf._advance_latched is False
        assert buf.advance_state is False
        assert buf.logger.messages == []

    def test_it_stays_loaded_through_a_print_with_a_frozen_odometer(self):
        """
        Measured: the odometer does not advance while a print consumes
        filament, while the buffer sawtooths 0.13 to 0.95 as the AMS refills
        it. Run long enough that the movement evidence lapses partway through:
        the latch is what carries the sensor once the print starts.
        """
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        assert buf.advance_state is True
        cycle = [(0.95, 2.545), (0.59, 2.545), (0.40, 2.545), (0.13, 2.545),
                 (0.39, 2.545), (0.47, 2.545), (0.63, 2.545), (0.52, 2.545)]

        printing = feed_bambu_gate(buf, unit, cycle * 6)

        assert printing == [True] * 48
        assert buf._odom_moved is False
        assert buf._odom_confirms() is False
        assert buf._gate_latched is True
        assert buf._tension_since is None
        assert buf.fila_adv.runout_helper.filament_present is True
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_afcs_own_record_is_believed_at_startup(self):
        # A lane keeps tool_loaded across a reboot, and after a restart there
        # was no arrival for the gate to witness.
        buf, _units = make_bambu_gate_buffer()
        self.with_record(buf, True)

        # Buffer agrees: compressed. min(0.94, 0.3 * 0.5 + 0.7 * 0.94) = 0.808.
        buf._adc_callback(0.25, 0.94)

        assert buf._gate_latched is True
        assert buf._record_checked is True
        assert buf.advance_state is True
        assert buf.logger.messages == [self.ADOPTED_AT_081]

    def test_a_record_the_buffer_contradicts_is_refused_out_loud(self):
        # Max tension is what an empty path reads like; adopting a loaded
        # record against it quietly is the dangerous direction.
        buf, _units = make_bambu_gate_buffer()
        self.with_record(buf, True)

        buf._adc_callback(0.25, 0.02)

        assert buf._gate_latched is False
        assert buf._record_checked is True
        assert buf.logger.messages == [self.REFUSED_AT_002]

    def test_a_raw_reading_at_max_tension_refuses_the_record_on_its_own(self):
        buf, _units = make_bambu_gate_buffer()
        self.with_record(buf, True)
        buf.smoothed_fps = 0.9

        # Smoothed 0.3 * 0.9 + 0.7 * 0.05 = 0.305 is clear of low_point; raw is not.
        buf._adc_callback(0.25, 0.05)

        assert buf._gate_latched is False
        assert buf.logger.messages == [(
            "warning",
            "Bambu_AMS_Buffer: AFC records lane28 loaded to the toolhead, but "
            "the buffer is at max tension (0.05), which is what an empty path "
            "reads like. Not adopting it -- the next load will settle this.")]

    def test_a_smoothed_reading_at_max_tension_refuses_the_record_on_its_own(self):
        buf, _units = make_bambu_gate_buffer()
        self.with_record(buf, True)
        buf.smoothed_fps = 0.0

        # Raw 0.14 is clear of low_point; smoothed 0.7 * 0.14 = 0.098 is not.
        buf._adc_callback(0.25, 0.14)

        assert buf._gate_latched is False
        assert buf.logger.messages == [(
            "warning",
            "Bambu_AMS_Buffer: AFC records lane28 loaded to the toolhead, but "
            "the buffer is at max tension (0.10), which is what an empty path "
            "reads like. Not adopting it -- the next load will settle this.")]

    def test_an_empty_record_adopts_nothing(self):
        buf, _units = make_bambu_gate_buffer()
        self.with_record(buf, False)

        buf._adc_callback(0.25, 0.94)

        assert buf._gate_latched is False
        assert buf._record_checked is False
        assert buf.advance_state is False
        assert buf.logger.messages == []

    def test_the_runout_release_is_not_undone_by_a_stale_record(self):
        """
        The release fires because the buffer has proved the path empty, so a
        record that has not caught up yet must not latch it straight back on.
        Only an unload re-arms the record check.
        """
        buf, (unit,) = make_bambu_gate_buffer()
        self.with_record(buf, True)
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        assert buf._gate_latched is True

        # The path empties and stays empty, well past unload_confirm_seconds.
        released = feed_bambu_gate(buf, unit, [(0.02, 2.545)] * 12)
        assert released == [True] * 10 + [False] * 2
        assert buf._gate_latched is False

        after = feed_bambu_gate(buf, unit, [(0.02, 2.545)] * 4)

        assert after == [False] * 4
        assert buf._gate_latched is False
        assert buf._record_checked is True
        # The first LOAD3 sample at 0.02 refused the record before the load.
        assert buf.logger.messages == [
            self.REFUSED_AT_002, BAMBU_GATE_LOAD3_DETECTED, self.RELEASED]

    def test_the_record_is_consulted_again_until_afc_restores_it(self):
        """
        AFC restores tool_loaded from saved vars once the units have claimed
        their lanes, measured 8 s after the first sample on printer 1. Asking
        once and giving up asked before the answer existed.
        """
        buf, _units = make_bambu_gate_buffer()
        lane = self.with_record(buf, False)
        # 0.3 rests between low_point 0.1 and set_point 0.5: not compressed.
        t = 0.0
        for _ in range(12):
            t += 0.25
            buf._adc_callback(t, 0.3)
        assert buf._gate_latched is False
        assert buf._record_checked is False

        lane.tool_loaded = True
        buf._adc_callback(t + 0.25, 0.3)

        assert buf._gate_latched is True
        assert buf._record_checked is True
        assert buf.advance_state is True
        assert buf.logger.messages == [(
            "info",
            "Bambu_AMS_Buffer: adopting lane28 as loaded at the toolhead from "
            "AFC's own record, buffer agreeing at 0.30")]

    def test_a_latched_gate_does_not_consult_the_record(self):
        # AFC records the load the gate just watched; adopting it again would
        # log a second, false account of where the load came from.
        buf, (unit,) = make_bambu_gate_buffer()
        lane = self.with_record(buf, False)
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        assert buf._gate_latched is True
        assert buf._record_checked is False

        lane.tool_loaded = True
        feed_bambu_gate(buf, unit, [(0.95, 2.545)])

        assert buf._record_checked is False
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_the_record_window_shuts(self):
        # Adoption is a restart recovery only; past the window a loaded lane
        # got there through a load this gate watched.
        buf, _units = make_bambu_gate_buffer({"record_check_seconds": 1.0})
        lane = self.with_record(buf, False)
        t = 0.0
        checked = []
        for _ in range(8):
            t += 0.25
            buf._adc_callback(t, 0.3)
            checked.append(buf._record_checked)
        # The window closes at the sample 1.0 s after the first (0.25 s).
        assert checked == [False] * 4 + [True] * 4

        lane.tool_loaded = True
        buf._adc_callback(t + 0.25, 0.3)

        assert buf._gate_latched is False
        assert buf.logger.messages == []

    def test_filament_already_at_the_gears_is_adopted_after_a_restart(self):
        # The odometer route reports an arrival; after a restart nobody
        # witnessed it, so a loaded lane would come back reading empty.
        buf, (unit,) = make_bambu_gate_buffer()
        unit.odom_m = 2.624                  # parked where the load left it
        latched = []
        t = 0.0
        for _ in range(14):
            t += 0.25
            buf._adc_callback(t, 0.94)       # compressed, nothing moving
            latched.append(buf._gate_latched)

        # Compressed from 0.25 s, so resting from 3.25 s.
        assert latched == [False] * 12 + [True] * 2
        assert buf.advance_state is True
        assert buf.logger.messages == [(
            "info",
            "Bambu_AMS_Buffer: load detected at 3.25 -- fps 0.94 (smoothed "
            "0.94), filament already at the gears (resting, compressed for "
            "3.0s) [Bambu_AMS_1=2624.0mm]")]

    def test_an_unload_is_never_adopted_as_resting(self):
        # Printer 1, lane15, 2026-09-24: the cut pushes the tip back into the
        # hotend with the unit parked, so the buffer sat compressed and the
        # odometer still, and the resting path called a load mid-unload.
        buf, (unit,) = make_bambu_gate_buffer()
        buf.lanes = {"lane15": BambuGateLane("lane15", tool_loaded=True,
                                             status=AFCLaneState.TOOL_UNLOADING)}
        buf._record_checked = True           # the record route is not under test
        buf.disable_buffer()                 # as TOOL_UNLOAD does, near its top
        unit.odom_m = 2.623
        t = 0.0
        for _ in range(36):
            t += 0.25
            buf._adc_callback(t, 0.94)       # compressed, nothing moving

        assert buf._compressed_since == 0.25
        assert buf._gate_latched is False
        assert buf.advance_state is False
        assert buf.logger.messages == []

    def test_the_friction_plateau_never_reaches_the_resting_path_either(self):
        # Bowden friction peaked at 0.39, below the advance threshold, so the
        # compression clock never even starts.
        buf, _units = make_bambu_gate_buffer()
        t = 0.0
        for _ in range(36):
            t += 0.25
            buf._adc_callback(t, 0.39)

        assert buf._compressed_since is None
        assert buf._gate_latched is False
        assert buf.logger.messages == []

    def test_without_any_odometer_it_falls_back_to_pressure_and_warns(self):
        buf, _units = make_bambu_gate_buffer(units=())
        spend_bambu_bind_grace(buf)

        states = feed_bambu_gate(buf, None, BAMBU_GATE_LOAD3)

        assert states == [False] * 33 + [True] * 9
        assert buf._odom_warned is True
        assert buf._gate_latched is True
        assert buf.logger.messages == [
            BAMBU_GATE_NO_ODOMETER,
            ("info", "Bambu_AMS_Buffer: load detected at 8.50 -- fps 0.96 "
                     "(smoothed 0.79), odometer still for 0.00s []")]

    def test_a_unit_claimed_during_the_grace_is_never_warned_about(self):
        buf, _units = make_bambu_gate_buffer(units=())
        t = 0.0
        for _ in range(3):
            t += 0.25
            buf._adc_callback(t, 0.46)
        unit = add_bambu_gate_unit(buf, "Bambu_AMS_1")
        while t < buf.odom_bind_grace_seconds * 2:
            t += 0.25
            buf._adc_callback(t, 0.46)

        # Past the wait, an unbound buffer would degrade and warn here.
        assert buf._odom_confirms() is False
        assert buf._odom_units == [unit]
        assert buf._odom_warned is False
        assert buf.logger.messages == []

    def test_odom_required_refuses_rather_than_degrading(self):
        buf, _units = make_bambu_gate_buffer({"odom_required": True}, units=())
        spend_bambu_bind_grace(buf)

        states = feed_bambu_gate(buf, None, BAMBU_GATE_LOAD3)

        # The same trace latches at the first 0.96 without odom_required.
        assert states == [False] * len(BAMBU_GATE_LOAD3)
        assert buf._compressed_since == 8.5
        assert buf._gate_latched is False
        assert buf._odom_warned is False
        assert buf.logger.messages == []

    def test_the_detection_instant_is_logged_once(self):
        # So a load can be checked afterwards against the toolhead switch's own
        # line at the same eventtime.
        buf, (unit,) = make_bambu_gate_buffer()

        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3 + [(0.95, 2.545)] * 4)

        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_a_silent_unit_reads_nan_in_the_detection_line(self):
        buf, (unit, _silent) = make_bambu_gate_buffer(
            units=("Bambu_AMS_1", "Bambu_AMS_HT_1"))

        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)

        assert buf.logger.messages == [(
            "info",
            "Bambu_AMS_Buffer: load detected at 10.25 -- fps 0.94 (smoothed "
            "0.94), odometer still for 0.50s [Bambu_AMS_1=2545.0mm, "
            "Bambu_AMS_HT_1=nanmm]")]

    def test_a_load_that_is_never_detected_logs_nothing(self):
        buf, (unit,) = make_bambu_gate_buffer()
        # Compressed, but the filament never stopped.

        states = feed_bambu_gate(buf, unit, [(0.96, 0.10), (0.96, 0.20), (0.96, 0.30)])

        assert states == [False] * 3
        assert buf._compressed_since == 0.25
        assert buf.logger.messages == []

    def test_sustained_max_tension_releases_the_latch(self):
        """
        An emptied buffer sits at 0.02 and stays there, unlike the printing
        sawtooth, which bottoms around 0.13 and recovers within a second.
        """
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        assert buf._gate_latched is True

        states = feed_bambu_gate(buf, unit, [(0.02, 2.545)] * 12)

        # Smoothed reaches 0.1 on the third sample (11.25 s); 2.0 s later the
        # eleventh releases the latch.
        assert states == [True] * 10 + [False] * 2
        assert buf._gate_latched is False
        assert buf._tension_since is None
        assert buf._odom_moved is False
        assert buf.fila_adv.runout_helper.filament_present is False
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED, self.RELEASED]

    def test_the_printing_sawtooth_never_starts_the_runout_count(self):
        """
        The sawtooth's floor is above low_point once smoothed, so the release
        count never begins; the 2 s dwell is the second line of defence.
        """
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)

        states = feed_bambu_gate(buf, unit, [
            (0.13, 2.545), (0.39, 2.545), (0.63, 2.545),
            (0.14, 2.545), (0.47, 2.545), (0.95, 2.545)] * 3)

        assert states == [True] * 18
        assert buf._tension_since is None
        assert buf._gate_latched is True
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]


class TestAFCBambuBufferRestingLoad:
    """_resting_load: whether a still, steadily compressed buffer means filament is held."""

    @staticmethod
    def compressed_since(seconds: float, last_move_ago: Optional[float] = None,
                         status: AFCLaneState = AFCLaneState.LOADED) -> AFCBambuBuffer:
        """
        A buffer at 100 s, compressed for `seconds`, with lane15 in `status`.

        :param seconds: how long the buffer has been compressed
        :param last_move_ago: seconds since the odometer last moved, None for never
        :param status: lane15's status
        :return: the buffer
        """
        buf, _units = make_bambu_gate_buffer()
        buf.lanes = {"lane15": BambuGateLane("lane15", status=status)}
        buf._now_t = 100.0
        buf._compressed_since = 100.0 - seconds
        if last_move_ago is not None:
            buf._last_move_t = 100.0 - last_move_ago
        return buf

    def test_a_moving_feed_can_never_be_adopted_as_resting(self):
        # A feed refreshes the odometer every 250 ms, so the stillness window
        # cannot accumulate however long the buffer stays compressed.
        buf, (unit,) = make_bambu_gate_buffer()
        t = 0.0
        odom_m = 0.10
        for _ in range(48):
            t += 0.25
            odom_m += 0.074                  # still feeding hard
            unit.odom_m = odom_m
            buf._adc_callback(t, 0.94)       # and compressed the whole way

        assert buf._gate_latched is False
        # Compressed for 11.75 s, far past the window, but moved just now.
        assert buf._compressed_since == 0.25
        assert buf._last_move_t == 12.0
        assert buf._resting_load() is False
        assert buf.logger.messages == []

    def test_an_uncompressed_buffer_is_not_resting(self):
        # Never moved, so only the missing compression can refuse it.
        buf = self.compressed_since(10.0)
        buf._compressed_since = None

        assert buf._resting_load() is False
        assert buf.logger.messages == []

    def test_an_unload_is_never_resting(self):
        loaded = self.compressed_since(10.0)
        unloading = self.compressed_since(10.0, status=AFCLaneState.TOOL_UNLOADING)

        assert loaded._resting_load() is True
        assert unloading._resting_load() is False
        assert unloading.logger.messages == []

    def test_it_must_be_compressed_for_the_whole_window(self):
        short = self.compressed_since(2.75)
        full = self.compressed_since(3.0)

        assert short._resting_load() is False
        assert full._resting_load() is True
        assert short.logger.messages == []
        assert full.logger.messages == []

    def test_the_odometer_must_have_been_still_for_the_whole_window(self):
        recent = self.compressed_since(10.0, last_move_ago=2.75)
        settled = self.compressed_since(10.0, last_move_ago=3.0)

        assert recent._resting_load() is False
        assert settled._resting_load() is True
        assert recent.logger.messages == []
        assert settled.logger.messages == []


class TestAFCBambuBufferBufferTriggered:
    """buffer_triggered: the software endstop carries the same gate as advance_state."""

    def test_buffer_triggered_carries_the_odometer_gate(self):
        """
        The software endstop homes on buffer_triggered, so pressure alone must
        not satisfy it either.
        """
        buf, (unit,) = make_bambu_gate_buffer()

        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3[:BAMBU_GATE_LOAD3_FIRST_HIGH + 1])
        # Pressure alone: 0.3 * 0.386 + 0.7 * 0.96 = 0.788, past homing's 0.7,
        # so the FPS endstop would fire and only the gate holds it back.
        assert buf._homing_high_point == 0.7
        assert buf.smoothed_fps == pytest.approx(0.788, abs=1e-3)
        assert AFCFPSBuffer.buffer_triggered.fget(buf) is True
        assert buf.buffer_triggered is False

        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3[BAMBU_GATE_LOAD3_FIRST_HIGH + 1:])

        assert buf.buffer_triggered is True
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_a_latched_load_needs_the_pressure_too(self):
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        # 0.3 * 0.947 + 0.7 * 0.5 = 0.634, under homing's 0.7.
        feed_bambu_gate(buf, unit, [(0.5, 2.545)])
        assert buf._gate_latched is True
        assert buf._odom_confirms() is True

        assert buf.buffer_triggered is False
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_a_latched_load_outlives_the_odometer(self):
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        # Frozen for 4.5 s more: past the 5.0 s TTL since the last move at 9.75 s.
        feed_bambu_gate(buf, unit, [(0.95, 2.545)] * 18)
        assert buf._odom_moved is False
        assert buf._odom_confirms() is False

        assert buf.buffer_triggered is True
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_the_odometer_opens_it_before_anything_latched(self):
        buf, (unit,) = make_bambu_gate_buffer()
        unit.odom_m = 2.545
        buf._now_t = 2.0
        buf._update_odom()
        buf._odom_moved = True
        buf._last_move_t = 1.0
        buf.smoothed_fps = 0.9

        assert buf.buffer_triggered is True

        assert buf._gate_latched is False
        assert buf.logger.messages == []


class TestAFCBambuBufferDisableBuffer:
    """disable_buffer: the gate resets on unload, and the FPS buffer stops too."""

    def test_an_unload_does_not_re_open_the_record(self):
        """
        AFC calls disable_buffer() near the top of TOOL_UNLOAD and clears
        tool_loaded only at the end, so for the whole unload the record still
        reads loaded. Re-arming the check would adopt the lane on its way out.
        """
        buf, _units = make_bambu_gate_buffer()
        buf.lanes = {"lane28": BambuGateLane("lane28", tool_loaded=True)}
        buf._adc_callback(0.25, 0.94)
        assert buf._gate_latched is True

        buf.disable_buffer()

        assert buf._gate_latched is False
        assert buf._record_checked is True

        buf._adc_callback(0.5, 0.94)         # record has not caught up yet

        assert buf._gate_latched is False
        assert buf.advance_state is False
        assert buf.logger.messages == [(
            "info",
            "Bambu_AMS_Buffer: adopting lane28 as loaded at the toolhead from "
            "AFC's own record, buffer agreeing at 0.81")]

    def test_unloading_resets_the_gate(self):
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        assert buf._gate_latched is True
        assert buf._compressed_since == 8.5

        buf.disable_buffer()

        assert buf._gate_latched is False
        assert buf._odom_moved is False
        assert buf._odom_prev == {}
        assert buf._last_move_t is None
        assert buf._compressed_since is None
        # What the boot already established is kept.
        assert buf._first_sample_t == 0.25
        assert buf._odom_units == [unit]
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_the_fps_buffer_is_disabled_too(self):
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        # Mid-runout: the max-tension count has started.
        feed_bambu_gate(buf, unit, [(0.02, 2.545)] * 3)
        assert buf._tension_since == 11.25
        buf.enable = True
        buf.current_lane = BambuGateLane("lane28")

        buf.disable_buffer()

        assert buf._tension_since is None
        assert buf._gate_latched is False
        assert buf.enable is False
        assert buf.current_lane is None
        assert buf.logger.messages == [
            BAMBU_GATE_LOAD3_DETECTED,
            ("debug", "Bambu_AMS_Buffer bambu buffer disabled for lane28")]


class TestAFCBambuBufferGetStatus:
    """get_status: the gate published alongside the usual FPS fields."""

    @staticmethod
    def fps_fields(state: str, fps: float, smoothed: float) -> Dict[str, Any]:
        """
        The FPS buffer's own status fields for an idle buffer with no lane.

        :param state: last_state
        :param fps: the raw reading
        :param smoothed: the smoothed reading, to three places
        :return: the fields
        """
        return {
            "state": state, "lanes": [], "enabled": False,
            "rotation_distance": None, "active_lane": None,
            "multiplier_high": 1.15, "multiplier_low": 0.85, "multiplier": 1.0,
            "fault_detection_enabled": False, "error_sensitivity": 0.0,
            "fault_timer": None, "distance_to_fault": None,
            "fps_value": fps, "smoothed_fps": smoothed, "set_point": 0.5,
        }

    def test_status_publishes_the_gate(self):
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)

        status = buf.get_status(0.0)

        # Last moved at 9.75 s, now 10.5 s. load_detected is the verdict, so the
        # gate can be watched on a printer still homing on a real switch.
        assert status == {
            **self.fps_fields("Trailing", 0.95, 0.947),
            "odom_still_s": 0.75, "odom_moved": True, "odom_confirms": True,
            "load_latched": True, "tension_s": 0.0, "load_detected": True,
            "advance_state": True,
        }
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]

    def test_status_reports_no_load_before_one_happens(self):
        buf, (unit,) = make_bambu_gate_buffer()
        unit.odom_m = 0.101

        buf._adc_callback(0.25, 0.46)          # resting, centred buffer
        status = buf.get_status(0.0)

        assert status == {
            **self.fps_fields("Neutral", 0.46, 0.472),
            "odom_still_s": 0.0, "odom_moved": False, "odom_confirms": False,
            "load_latched": False, "tension_s": 0.0, "load_detected": False,
            "advance_state": False,
        }
        assert buf.logger.messages == []

    def test_status_counts_the_time_at_max_tension(self):
        buf, (unit,) = make_bambu_gate_buffer()
        feed_bambu_gate(buf, unit, BAMBU_GATE_LOAD3)
        # At max tension from 11.25 s; now 12.25 s.
        feed_bambu_gate(buf, unit, [(0.02, 2.545)] * 7)

        status = buf.get_status(0.0)

        # Still latched, but the endstop reads live pressure and says no.
        assert status == {
            **self.fps_fields("Advancing", 0.02, 0.02),
            "odom_still_s": 2.5, "odom_moved": True, "odom_confirms": True,
            "load_latched": True, "tension_s": 1.0, "load_detected": False,
            "advance_state": True,
        }
        assert buf.logger.messages == [BAMBU_GATE_LOAD3_DETECTED]
