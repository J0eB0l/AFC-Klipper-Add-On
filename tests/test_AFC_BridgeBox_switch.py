"""Unit tests for extras/AFC_BridgeBox_switch.py"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Optional, Tuple
from unittest.mock import MagicMock, call, patch

import pytest
from configfile import error

from extras import AFC_BridgeBox_switch as switch_mod
from extras.AFC_BridgeBox import afcBridgeBox
from extras.AFC_BridgeBox_switch import (
    LETTERS,
    SwitchError,
    SwitchResetError,
    afcBridgeBoxSwitch,
    in_order,
    load_config_prefix,
    other,
    outlet_at,
    route_with,
)
from extras.AFC_lane import AFCLane, AFCMoveWarning, AssistActive, SpeedMode
from extras.AFC_respond import AFCprompt
from extras.AFC_unit import afcUnit
from tests.conftest import CommandError, MockAFC, MockConfig, MockGCodeCommand, MockPrinter

SECTION = "AFC_BridgeBox_switch Switch_1"
VALUES = {"inlet_a": ["Bambu_AMS_1", "Turtle_1"], "inlet_b": ["Ace2_1"],
          "outlet_a": "extruder", "outlet_b": "extruder1"}
# The tube from each outlet to its toolhead in mm, as a calibrated section gives them.
LENGTHS = {"outlet_a_length": 850, "outlet_b_length": 1010}


def fs(route: int = 0, position: Optional[int] = None, a: bool = False, b: bool = False,
       on: bool = True) -> Dict[str, Any]:
    """:return dict: a chain's switch state, its sensors given by inlet letter"""
    return {"on": on, "route": route, "position": route if position is None else position,
            "inlets": [b, a], "values": []}


class FakeChain(afcBridgeBox):
    """A chain that reports a switch state and records the routes it is sent."""

    def __init__(self, name: str = "Chain_1", state: Optional[Dict[str, Any]] = None,
                 linked: bool = True, then: Optional[List[Any]] = None,
                 bays: Optional[List[str]] = None) -> None:
        self.name = name
        self.fs = state
        self.linked = linked
        # States reported one per poll once a route is sent. The last one repeats.
        self.then = list(then or [])
        # The units the chain makes for its bays.
        self.bays = list(bays or [])
        self.routes: List[int] = []
        self.polls = 0
        # Whether a unit that answers on the bus has no bay yet.
        self.arriving = False
        # The AMS each bay holds or is kept for, by unit name.
        self.uids: Dict[str, Optional[str]] = {}
        # The popups other modules queued behind the chain's own.
        self.offers: List[Tuple[Any, ...]] = []
        # The lanes of each unit that was put on its extruder, by name.
        self.joined: List[List[str]] = []
        # What a switch the chain made had it keep, in turn.
        self.saved: List[Dict[str, Any]] = []

    def unit_names(self) -> List[str]:
        return list(self.bays)

    def bay_uid(self, bay: str) -> Optional[str]:
        return self.uids.get(bay)

    def inlet_buffer(self, inlet: Optional[str]) -> Tuple[str, str]:
        # The chain's second buffer is behind inlet B.
        return ("Buffer_B", "chip_inlet_b") if inlet == "B" else ("Buffer", "chip")

    def offer(self, key: str, title: str, lines: List[str],
              buttons: List[Tuple[str, str, str]]) -> None:
        self.offers.append((key, title, lines, buttons))

    def _clear_standalone(self, lanes: List[Any]) -> None:
        self.joined.append([lane.name for lane in lanes])

    def save_switch(self, keys: Dict[str, Any]) -> None:
        self.saved.append(dict(keys))

    def units_arriving(self) -> bool:
        return self.arriving

    def switch_state(self) -> Optional[Dict[str, Any]]:
        if not self.routes:
            return self.fs
        self.polls += 1
        if len(self.then) > 1:
            return self.then.pop(0)
        return self.then[0] if self.then else self.fs

    def switch_set_route(self, route: int) -> bool:
        if not self.linked:
            return False
        self.routes.append(route)
        return True


class SimChain(FakeChain):
    """
    A switch that acts as a real one does: a path follows a route that changed for it, unless
    its inlet holds filament or it is stuck.
    """

    BITS = {"A": 2, "B": 1}
    SENSORS = {"A": 1, "B": 0}

    def __init__(self, route: int = 0, position: Optional[int] = None, a: bool = False,
                 b: bool = False, stuck: bool = False) -> None:
        super().__init__(state=fs(route, position, a, b))
        self.stuck = stuck

    def switch_state(self) -> Optional[Dict[str, Any]]:
        return self.fs

    def switch_set_route(self, route: int) -> bool:
        self.routes.append(route)
        for letter, bit in self.BITS.items():
            if ((route ^ self.fs["route"]) & bit
                    and not self.fs["inlets"][self.SENSORS[letter]]
                    and not self.stuck):
                self.fs["position"] = (self.fs["position"] & ~bit) | (route & bit)
        self.fs["route"] = route
        return True

    def fill(self, a: Optional[bool] = None, b: Optional[bool] = None) -> None:
        """Set what the inlet sensors read, None leaving one as it is."""
        for letter, value in (("A", a), ("B", b)):
            if value is not None:
                self.fs["inlets"][self.SENSORS[letter]] = value


class FakeUnit(afcUnit):
    """An AFC unit with only a name, and a tube length calibration a test scripts."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.calibrate_bowden = MagicMock()
        self.return_to_home = MagicMock()


class FakeAms(FakeUnit):
    """A unit a chain makes for a bay: it goes behind a buffer and feeds a lane to the switch."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.pool = False
        # The buffer and chip it was put behind, in turn.
        self.behind: List[Tuple[str, str]] = []
        # Whether the last of them was only assumed, None before the first.
        self.assumed: Optional[bool] = None
        self.feed_to = MagicMock(return_value=False)

    def use_buffer(self, buffer: str, chip: str, assumed: bool = False) -> None:
        self.behind.append((buffer, chip))
        self.assumed = assumed


class FakePathUnit(FakeUnit):
    """A unit that keeps its own tube length and measures its path whenever it loads."""

    def __init__(self, name: str, length: float = 2000.0, unload: float = 1990.0) -> None:
        super().__init__(name)
        self.afc_bowden_length = length
        self.afc_unload_bowden_length = unload
        # What its last load measured.
        self.path: Optional[float] = None
        # The lengths it was told to keep.
        self.kept: List[Tuple[float, float]] = []
        # How often it was told to forget its path.
        self.forgot = 0
        # The lengths it held as each of its loads set off.
        self.began: List[Tuple[float, float]] = []

    def load_path_mm(self) -> Optional[float]:
        return self.path

    def keep_path(self, load: float, unload: float) -> None:
        self.afc_bowden_length, self.afc_unload_bowden_length = load, unload
        self.kept.append((load, unload))

    def forget_path(self) -> None:
        # The length a unit holds for a path it never measured.
        self.afc_bowden_length = self.afc_unload_bowden_length = 3000.0
        self.forgot += 1


class FakeHub:
    """A hub with only the tube lengths the switch changes and whether it reads filament."""

    def __init__(self, name: str, length: float = 900.0, unload: float = 880.0) -> None:
        self.fullname = f"AFC_hub {name}"
        self.afc_bowden_length = length
        self.afc_unload_bowden_length = unload
        self.state = False


class FakeExtruder:
    """An AFC extruder with only what a lane and the switch read."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.th_extruder_name = f"th_{name}"
        self.lanes: Dict[str, Any] = {}
        self.check_lanes = MagicMock()
        # It has no toolhead sensor of its own until a test gives it one.
        self.tool_start: Optional[str] = None
        self.fila_tool_start: Optional[Any] = None
        self.tool_start_state = False


class FakePinSensor:
    """A toolhead sensor on a pin. AFC keeps what it reads on the extruder."""


class FakeMotionSensor:
    """A toolhead motion sensor, whose switch reads the filament that is there now."""

    def __init__(self, reads: bool = False) -> None:
        self.runout_buttun_state = reads


class FakeStepper:
    """A stepper AFC homes a lane with, which records the endstops it is given."""

    def __init__(self) -> None:
        self.add_polled_endstop = MagicMock()


class FakeLane(AFCLane):
    """A lane with only what the switch reads. Its set_extruder is the real one."""

    # In place of the real property, which reads the lane's sensors.
    load_state = False

    def __init__(self, name: str, unit: str, extruder: FakeExtruder,
                 loaded: bool = False) -> None:
        self.name = name
        self.unit = unit
        self.unit_obj: Optional[FakeUnit] = None
        self.hub_obj: Optional[Any] = None
        self.tool_loaded = loaded
        self.tool_swap = MagicMock()
        self.extruder_obj = extruder
        self.afc_extruder_name = extruder.name
        extruder.lanes[name] = self
        # No stepper moves it until a test gives it one.
        self.drive_stepper: Optional[Any] = None
        self.dist_hub = 60.0
        self.set_load_current = MagicMock()
        self.do_enable = MagicMock()
        self.move_advanced = MagicMock()
        self.move_to = MagicMock(return_value=(True, 0.0, AFCMoveWarning.NONE))


def make_printer(chains: Optional[List[FakeChain]] = None,
                 units: Optional[List[str]] = None,
                 extruders: Optional[List[str]] = None,
                 connected: bool = True) -> MockPrinter:
    """:return MockPrinter: a printer holding these chains, units and extruders"""
    printer = MockPrinter(afc=MockAFC())
    printer._gcode.register_mux_command = MagicMock()
    printer._reactor.pause = MagicMock()
    printer._reactor.register_timer = MagicMock()
    printer._afc.function.in_print.return_value = False
    printer._afc.VarFile = "/nonexistent/AFC.var"
    for chain in [FakeChain()] if chains is None else chains:
        printer._objects[f"AFC_BridgeBox {chain.name}"] = chain
    for unit in VALUES["inlet_a"] + VALUES["inlet_b"] if units is None else units:
        printer._objects[f"AFC_Unit {unit}"] = FakeUnit(unit)
        if connected:
            printer._afc.units[unit] = printer._objects[f"AFC_Unit {unit}"]
    for name in ["extruder", "extruder1"] if extruders is None else extruders:
        printer._objects[f"AFC_extruder {name}"] = FakeExtruder(name)
    return printer


def add_lane(printer: MockPrinter, name: str, unit: str, extruder: str = "extruder",
             loaded: bool = False) -> FakeLane:
    """:return FakeLane: a lane added to ``printer`` on one of its extruders"""
    lane = FakeLane(name, unit, printer._objects[f"AFC_extruder {extruder}"], loaded)
    lane.unit_obj = printer._objects.get(f"AFC_Unit {unit}")
    printer._objects[f"AFC_lane {name}"] = lane
    return lane


def add_hub(lane: FakeLane, length: float = 900.0, unload: float = 880.0) -> FakeHub:
    """:return FakeHub: a hub given to ``lane``, named after the lane's unit"""
    lane.hub_obj = FakeHub(lane.unit, length, unload)
    return lane.hub_obj


def add_path_unit(switch: afcBridgeBoxSwitch, lane: FakeLane, length: float = 2000.0,
                  unload: float = 1990.0) -> FakePathUnit:
    """:return FakePathUnit: put in place of the unit of ``lane``"""
    unit = FakePathUnit(lane.unit, length, unload)
    lane.unit_obj = unit
    switch.afc.units[lane.unit] = unit
    return unit


def tube_of(hub: Any) -> Tuple[float, float]:
    """:return tuple: the load and unload lengths a hub or a unit holds now"""
    return hub.afc_bowden_length, hub.afc_unload_bowden_length


def plumbed(chain: Optional[FakeChain] = None, **values: Any) -> afcBridgeBoxSwitch:
    """
    :return afcBridgeBoxSwitch: a connected switch with one lane per unit, each named after it:
    Turtle_1 and Bambu_AMS_1 on inlet A and `extruder`, Ace2_1 on inlet B and `extruder1`
    """
    printer = make_printer([chain or SimChain()])
    for unit in VALUES["inlet_a"] + VALUES["inlet_b"]:
        add_lane(printer, f"lane_{unit}", unit)
    return make_switch(printer, **values)


# The bays of a chain that is on both inlets, and the AMS each one holds.
BAYS = {"Bambu_AMS_1": "AAAA", "Bambu_AMS_2": "BBBB"}
BOTH = {"inlet_a": ["Chain_1"], "inlet_b": ["Chain_1"]}
# The inlets of a switch that has the unit Turtle_1, which is on no chain, on both.
SHARED = {"inlet_a": ["Bambu_AMS_1", "Turtle_1"], "inlet_b": ["Ace2_1", "Turtle_1"]}

# How a prompt with a cancel button ends, and how one with no button does.
CANCEL = [("raw", "// action:prompt_footer_button Cancel|RESPOND TYPE=command "
                  "MSG=action:prompt_end|warning"),
          ("raw", "// action:prompt_show")]
SHOW = ("raw", "// action:prompt_show")
END = ("raw", "// action:prompt_end")
# The footer of a prompt that goes back to AFC's calibration prompt, and of one that goes back
# to the prompt that offers what to calibrate on a switch.
TO_AFC = [("raw", "// action:prompt_footer_button Back to unit selection|AFC_CALIBRATION|"
                  "info")] + CANCEL
TO_SWITCH = [("raw", "// action:prompt_footer_button Back|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                     "SWITCH=Switch_1|info")] + CANCEL


# The section of the switch Chain_1 makes for itself, and what the chain gives it when the
# chain's own section sets nothing.
MADE = "AFC_BridgeBox_switch Chain_1"
OWN = {"chain": "Chain_1", "chain_made": True, "inlet_a": None, "inlet_b": None,
       "outlet_a": None, "outlet_b": None}


def give_sensor(switch: afcBridgeBoxSwitch, name: str, reads: bool = False) -> FakeExtruder:
    """:return FakeExtruder: one of the printer's extruders, given a toolhead sensor on a pin"""
    extruder = extruder_of(switch, name)
    extruder.tool_start = "PA1"
    extruder.fila_tool_start = FakePinSensor()
    extruder.tool_start_state = reads
    return extruder


def made(chain: Optional[FakeChain] = None, connect: bool = True,
         **values: Any) -> afcBridgeBoxSwitch:
    """
    :return afcBridgeBoxSwitch: the switch Chain_1 made for itself. Its bays Bambu_AMS_1 and
    Bambu_AMS_2 are found by loading and no outlet has its toolhead. Each bay has one lane named
    after it, on `extruder`, and both extruders have a toolhead sensor that reads empty
    """
    chain = chain or SimChain()
    chain.bays = list(BAYS)
    chain.uids = dict(BAYS)
    printer = make_printer([chain], units=[])
    for unit in BAYS:
        printer._objects[f"AFC_Unit {unit}"] = printer._afc.units[unit] = FakeAms(unit)
        add_lane(printer, f"lane_{unit}", unit)
    switch = make_switch(printer, section=MADE, connect=connect, **dict(OWN, **values))
    for name in ("extruder", "extruder1"):
        give_sensor(switch, name)
    return switch


# The footer of a prompt that goes back to the prompt that offers what to calibrate on the
# switch Chain_1 made, and the toolheads its outlets feed in the tests that find them.
TO_MADE = [("raw", "// action:prompt_footer_button Back|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                   "SWITCH=Chain_1|info")] + CANCEL
PLUMBING = {"A": ["extruder1"], "B": ["extruder"]}


def known(chain: Optional[FakeChain] = None, **values: Any) -> afcBridgeBoxSwitch:
    """
    :return afcBridgeBoxSwitch: the switch Chain_1 made, with Bambu_AMS_1 found on inlet A and
    Bambu_AMS_2 on inlet B
    """
    switch = made(chain, **values)
    found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
    return switch


def reaching(switch: afcBridgeBoxSwitch, unit: str, toolheads: Dict[str, List[str]],
             lingers: bool = False) -> MagicMock:
    """
    Script a unit's feed through the switch to a toolhead: the filament shows at the unit's
    inlet, the toolhead sensors ``toolheads`` names for the outlet that inlet is at read it, it
    is looked for once and reeled back, or stays at the inlet when it ``lingers``.

    :return MagicMock: the unit's feed_to
    """
    chain = chain_of(switch)

    def feed_to(lane: FakeLane, seen: Any, gcmd: Any) -> bool:
        inlet = switch.inlet_of(lane.unit)
        reached = toolheads.get(switch.state()["inlets"][inlet]["outlet"], [])
        chain.fill(**{inlet.lower(): True})
        for name in reached:
            extruder_of(switch, name).tool_start_state = True
        saw = seen()
        for name in reached:
            extruder_of(switch, name).tool_start_state = False
        if not lingers:
            chain.fill(a=False, b=False)
        return saw

    ams_of(switch, unit).feed_to = MagicMock(side_effect=feed_to)
    return ams_of(switch, unit).feed_to


def finding(chain: Optional[FakeChain] = None, connect: bool = True,
            **values: Any) -> afcBridgeBoxSwitch:
    """
    :return afcBridgeBoxSwitch: a switch with Chain_1 on both inlets, so its bays Bambu_AMS_1
    and Bambu_AMS_2 are found by loading. Each has one lane named after it, on `extruder`
    """
    chain = chain or SimChain()
    chain.bays = list(BAYS)
    chain.uids = dict(BAYS)
    printer = make_printer([chain], units=[])
    for unit in BAYS:
        printer._objects[f"AFC_Unit {unit}"] = printer._afc.units[unit] = FakeAms(unit)
        add_lane(printer, f"lane_{unit}", unit)
    return make_switch(printer, connect=connect, **dict(BOTH, **values))


def found(switch: afcBridgeBoxSwitch, **inlets: str) -> None:
    """Put units that are found by loading on the inlet found for the AMS in their bay."""
    for unit, inlet in inlets.items():
        switch._found[chain_of(switch).uids[unit]] = inlet
    switch._follow()
    switch.afc.save_vars.reset_mock()


def sharing(chain: Optional[FakeChain] = None, connect: bool = True,
            **values: Any) -> afcBridgeBoxSwitch:
    """
    :return afcBridgeBoxSwitch: a switch as ``plumbed`` makes it, but with Turtle_1 on both
    inlets, so that unit is found by loading. Both extruders have a toolhead sensor that reads
    empty
    """
    printer = make_printer([chain or SimChain(route=2)])
    for unit in VALUES["inlet_a"] + VALUES["inlet_b"]:
        add_lane(printer, f"lane_{unit}", unit)
    switch = make_switch(printer, connect=connect, **dict(SHARED, **values))
    for name in ("extruder", "extruder1"):
        give_sensor(switch, name)
    return switch


def stepped(lane: FakeLane) -> FakeLane:
    """:return FakeLane: the lane, of a unit with no stepper that AFC moves by distance"""
    lane.unit_obj.stepperless_drive = True
    lane.unit_obj.lane_move = MagicMock()
    return lane


def homed(lane: FakeLane) -> FakeStepper:
    """:return FakeStepper: the stepper given to a lane, which AFC homes it with"""
    lane.drive_stepper = FakeStepper()
    return lane.drive_stepper


def stepping(switch: afcBridgeBoxSwitch, lane: FakeLane, inlet: Optional[str] = None,
             toolhead: Optional[str] = None, after: int = 3) -> MagicMock:
    """
    Script the moves of a lane AFC moves by distance: once it was fed ``after`` steps its
    filament shows at ``inlet`` and at the toolhead sensor of ``toolhead``, None for neither,
    and it is gone from both when the lane is pulled back.

    :return MagicMock: the lane's move_advanced
    """
    chain = chain_of(switch)
    steps: List[float] = []

    def move_advanced(distance: float, speed_mode: Any, assist_active: Any, gcmd: Any) -> None:
        if distance > 0:
            steps.append(distance)
        shows = distance > 0 and len(steps) >= after
        if inlet is not None:
            chain.fill(**{inlet.lower(): shows})
        if toolhead is not None:
            extruder_of(switch, toolhead).tool_start_state = shows

    lane.move_advanced = MagicMock(side_effect=move_advanced)
    return lane.move_advanced


def ams_of(switch: afcBridgeBoxSwitch, unit: str) -> FakeAms:
    """:return FakeAms: one of the units ``finding`` made"""
    return switch.afc.units[unit]


def other_extruder(name: str) -> str:
    """:return str: the other one of the two extruders a switch feeds"""
    return "extruder1" if name == "extruder" else "extruder"


def lane_of(switch: afcBridgeBoxSwitch, unit: str) -> FakeLane:
    """:return FakeLane: the lane ``plumbed`` or ``finding`` made for a unit"""
    return switch.printer._objects[f"AFC_lane lane_{unit}"]


def extruder_of(switch: afcBridgeBoxSwitch, name: str) -> FakeExtruder:
    """:return FakeExtruder: one of the printer's extruders"""
    return switch.printer._objects[f"AFC_extruder {name}"]


def make_switch(printer: Optional[MockPrinter] = None, section: str = SECTION,
                connect: bool = True, **values: Any) -> afcBridgeBoxSwitch:
    """:return afcBridgeBoxSwitch: a switch built from VALUES with ``values`` on top"""
    printer = printer or make_printer()
    merged = {key: value for key, value in dict(VALUES, **values).items() if value is not None}
    switch = afcBridgeBoxSwitch(MockConfig(name=section, printer=printer, values=merged))
    printer._objects[section] = switch
    if connect:
        switch.handle_connect()
    return switch


def chain_of(switch: afcBridgeBoxSwitch) -> FakeChain:
    """:return FakeChain: the chain the switch found"""
    assert isinstance(switch.chain, FakeChain)
    return switch.chain


def two_switches() -> Tuple[afcBridgeBoxSwitch, afcBridgeBoxSwitch, FakeLane]:
    """
    :return tuple: Switch_1 as ``plumbed`` makes it, Switch_2 on Chain_2 with the unit Other
    on `extruder2`, and Other's lane20
    """
    chains = [SimChain(), SimChain()]
    chains[1].name = "Chain_2"
    printer = make_printer(chains, units=VALUES["inlet_a"] + VALUES["inlet_b"] + ["Other"],
                           extruders=["extruder", "extruder1", "extruder2"])
    for unit in VALUES["inlet_a"] + VALUES["inlet_b"]:
        add_lane(printer, f"lane_{unit}", unit)
    lane = add_lane(printer, "lane20", "Other", extruder="extruder2")
    first = make_switch(printer, chain="Chain_1")
    second = make_switch(printer, section="AFC_BridgeBox_switch Switch_2", chain="Chain_2",
                         inlet_a=["Other"], inlet_b=None, outlet_a="extruder2", outlet_b=None)
    return first, second, lane


class Run:
    """One run of a unit's tube length calibration: its answer and what the filament does."""

    def __init__(self, length: float, done: bool = True, why: str = "ok",
                 shows: Optional[str] = "A", watched: bool = True,
                 stays: bool = False) -> None:
        self.answer = (done, why, length)
        # The inlet the filament shows at on its way, None for neither.
        self.shows = shows
        # Whether the watch comes by while the filament is on its way.
        self.watched = watched
        # Whether the filament is still at the inlet when the run ends.
        self.stays = stays


def measuring(switch: afcBridgeBoxSwitch, lane: FakeLane, *runs: Run) -> List[Tuple[str, str]]:
    """
    Script the tube length calibration of a lane's unit, one ``Run`` per call. A run that
    measured saves its length as the hub's own, as a real unit does.

    :return list: filled per run with the extruder the lane was on and the outlet its inlet
    was at
    """
    script = list(runs)
    through: List[Tuple[str, str]] = []
    # As after PREP's section, so a watch that comes by has no unit to report.
    switch._reported = switch.live_units()

    def calibrate_bowden(cur_lane: FakeLane, dis: float, tol: float,
                         gcmd: Any) -> Tuple[bool, str, float]:
        run = script.pop(0)
        inlet = switch.inlet_of(cur_lane.unit)
        through.append((cur_lane.extruder_obj.name, switch.state()["inlets"][inlet]["outlet"]))
        if run.shows is not None:
            chain_of(switch).fill(**{run.shows.lower(): True})
        if run.watched:
            switch._watch(0.0)
        if (run.shows is not None
            and not run.stays):
            chain_of(switch).fill(**{run.shows.lower(): False})
        done, _why, length = run.answer
        if (done
            and length > 0):
            cur_lane.hub_obj.afc_bowden_length = length
            cur_lane.hub_obj.afc_unload_bowden_length = length
        return run.answer

    lane.unit_obj.calibrate_bowden = MagicMock(side_effect=calibrate_bowden)
    return through


class Load:
    """One load of a lane whose unit measures its own path, and the unload after it."""

    def __init__(self, path: Optional[float], loads: bool = True, unloads: bool = True,
                 shows: Optional[str] = "A", lingers: bool = False) -> None:
        # What the unit measured on the way in, None for nothing.
        self.path = path
        self.loads = loads
        self.unloads = unloads
        # The inlet the filament shows at while the lane is loaded, None for neither.
        self.shows = shows
        # Whether that inlet still reads filament once the lane is unloaded.
        self.lingers = lingers


def loading(switch: afcBridgeBoxSwitch, lane: FakeLane, *loads: Load) -> List[Tuple[str, str]]:
    """
    Script AFC's loads and unloads of a lane whose unit measures its own path, one ``Load``
    per load. A load asks the switch first and tells it after, as AFC does, and a path that
    was measured is left as the unit's own lengths, as a real unit does.

    :return list: filled per load with the extruder the lane was on and the outlet its inlet
    was at
    """
    script = list(loads)
    through: List[Tuple[str, str]] = []
    under_way: List[Load] = []
    # As after PREP's section, so a watch that comes by has no unit to report.
    switch._reported = switch.live_units()
    # A lane's toolhead is the active one once the lane is loaded.
    switch.afc.function.get_current_extruder.side_effect = (
        lambda: lane.extruder_obj.th_extruder_name)

    def tool_load(cur_lane: FakeLane, gcmd: Any) -> bool:
        load = script.pop(0)
        under_way[:] = [load]
        inlet = switch.inlet_of(cur_lane.unit)
        if not switch.handle_tool_load_prepare(cur_lane, gcmd):
            return False
        through.append((cur_lane.extruder_obj.name, switch.state()["inlets"][inlet]["outlet"]))
        cur_lane.unit_obj.began.append(tube_of(cur_lane.unit_obj))
        if not load.loads:
            return False
        if load.shows is not None:
            chain_of(switch).fill(**{load.shows.lower(): True})
        cur_lane.tool_loaded = True
        cur_lane.unit_obj.path = load.path
        if load.path is not None:
            cur_lane.unit_obj.afc_bowden_length = load.path
            cur_lane.unit_obj.afc_unload_bowden_length = load.path
        switch.handle_tool_loaded(cur_lane)
        return True

    def tool_unload(cur_lane: FakeLane, set_start_time: bool, force_unload: bool,
                    gcmd: Any) -> bool:
        load = under_way[0]
        if not load.unloads:
            return False
        cur_lane.tool_loaded = False
        if (load.shows is not None
            and not load.lingers):
            chain_of(switch).fill(**{load.shows.lower(): False})
        return True

    switch.afc.TOOL_LOAD = MagicMock(side_effect=tool_load)
    switch.afc.TOOL_UNLOAD = MagicMock(side_effect=tool_unload)
    return through


class TestOther:
    def test_each_letter_gives_the_other(self):
        assert (other("A"), other("B")) == ("B", "A")


class TestInOrder:
    def test_a_name_is_split_into_its_text_and_its_numbers(self):
        assert in_order("lane12") == ["lane", 12, ""]
        assert in_order("lane_Turtle_1b") == ["lane_Turtle_", 1, "b"]

    def test_a_name_without_a_number_is_its_text(self):
        assert in_order("bypass") == ["bypass"]

    def test_names_sort_by_number_and_not_by_digit(self):
        names = ["lane12", "lane8", "lane100", "lane9", "lane_b", "lane_a"]
        assert sorted(names, key=in_order) == ["lane8", "lane9", "lane12", "lane100", "lane_a",
                                               "lane_b"]


class TestOutletAt:
    # The table measured on a switch: released, an inlet exits outlet B.
    @pytest.mark.parametrize("route, a_exits, b_exits", [
        (0, "B", "B"), (1, "B", "A"), (2, "A", "B"), (3, "A", "A")])
    def test_every_route(self, route, a_exits, b_exits):
        assert (outlet_at(route, "A"), outlet_at(route, "B")) == (a_exits, b_exits)


class TestRouteWith:
    @pytest.mark.parametrize("route, inlet, outlet, expected", [
        (0, "A", "A", 2), (0, "B", "A", 1), (0, "A", "B", 0), (0, "B", "B", 0),
        (3, "A", "B", 1), (3, "B", "B", 2), (3, "A", "A", 3), (3, "B", "A", 3),
        (1, "A", "A", 3), (2, "B", "A", 3), (1, "B", "B", 0), (2, "A", "B", 0)])
    def test_one_inlet_moves_and_the_other_stays(self, route, inlet, outlet, expected):
        assert route_with(route, inlet, outlet) == expected


class TestAfcBridgeBoxSwitchInit:
    def test_it_reads_the_section(self):
        printer = make_printer()
        switch = make_switch(printer, connect=False)
        assert switch.printer is printer
        assert switch.reactor is printer.get_reactor()
        assert switch.afc is printer._afc
        assert switch.logger is printer._afc.logger
        assert (switch.fullname, switch.name) == (SECTION, "Switch_1")
        assert (switch.chain_name, switch.chain) == (None, None)
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Ace2_1"]}
        assert switch.outlets == {"A": "extruder", "B": "extruder1"}
        assert (switch._report_by, switch._reported) == (None, [])
        assert switch.finding == []
        assert (switch._check, switch._homing) == (None, [])

    def test_a_name_on_both_inlets_is_kept_apart_to_be_found_by_loading(self):
        switch = make_switch(connect=False, inlet_a=["Chain_1", "Turtle_1", "Chain_2"],
                             inlet_b=["Ace2_1", "Chain_2", "Chain_1"])
        assert switch.finding == ["Chain_1", "Chain_2"]
        assert switch.inlets == {"A": ["Turtle_1"], "B": ["Ace2_1"]}

    def test_a_section_with_only_a_chain_on_both_inlets_is_taken(self):
        switch = make_switch(connect=False, **BOTH)
        assert switch.finding == ["Chain_1"]
        assert switch.inlets == {"A": [], "B": []}

    def test_the_chain_is_kept_as_named(self):
        assert make_switch(connect=False, chain="Chain_2").chain_name == "Chain_2"

    def test_blank_unit_names_are_dropped(self):
        switch = make_switch(connect=False, inlet_a=[" Turtle_1 ", "", " "])
        assert switch.inlets["A"] == ["Turtle_1"]

    def test_an_inlet_and_an_outlet_may_be_left_out(self):
        switch = make_switch(connect=False, inlet_b=None, outlet_a=None)
        assert switch.inlets["B"] == []
        assert switch.outlets["A"] is None

    def test_each_inlet_starts_and_rests_on_its_own_outlet(self):
        switch = make_switch(connect=False)
        assert switch.start == {"A": "A", "B": "B"}
        assert switch.rest == {"A": "A", "B": "B"}
        assert switch.rest is not switch.start

    @pytest.mark.parametrize("values, only", [
        (dict(outlet_a=None), "B"), (dict(outlet_b=None), "A")])
    def test_with_one_outlet_both_inlets_start_on_it(self, values, only):
        switch = make_switch(connect=False, **values)
        assert switch.start == {"A": only, "B": only}
        assert switch.rest == {"A": only, "B": only}

    def test_the_tube_lengths_are_unset_until_the_section_gives_them(self):
        assert make_switch(connect=False).lengths == {"A": None, "B": None}

    def test_the_tube_lengths_are_read_as_mm_above_zero(self):
        values = dict(VALUES, outlet_a_length="850", outlet_b_length=1010.5)
        config = MockConfig(name=SECTION, printer=make_printer(), values=values)
        config.getfloat = MagicMock(side_effect=config.getfloat)
        assert afcBridgeBoxSwitch(config).lengths == {"A": 850.0, "B": 1010.5}
        assert config.getfloat.call_args_list == [call("outlet_a_length", None, above=0.0),
                                                  call("outlet_b_length", None, above=0.0)]

    def test_one_tube_length_may_be_given_alone(self):
        switch = make_switch(connect=False, outlet_b_length=1010)
        assert switch.lengths == {"A": None, "B": 1010.0}

    def test_it_starts_with_no_lanes_and_nothing_under_way(self):
        switch = make_switch(connect=False)
        assert switch.lanes == []
        assert switch._routed == {"A": None, "B": None}
        assert switch._saved_lanes == {}
        assert (switch._by_print, switch._due_back, switch._printing) == ([], [], False)
        assert switch.confirmed == []
        assert (switch._floating, switch._found, switch._held) == ({}, {}, {})
        assert switch._offered == []
        assert switch._offsets == {}
        assert switch._seen == {"A": False, "B": False}
        assert switch._lengthless == []
        assert switch._measuring is None
        assert (switch._moving, switch._told) == (False, "")

    def test_it_registers_its_handlers_and_commands(self):
        printer = make_printer()
        switch = make_switch(printer, connect=False)
        assert printer._event_handlers == {
            "klippy:connect": [switch.handle_connect],
            "klippy:ready": [switch.handle_ready],
            "afc:prep": [switch.handle_prep],
            "afc:tool_change_unload": [switch.handle_tool_change_unload],
            "afc:tool_load_prepare": [switch.handle_tool_load_prepare],
            "afc:tool_loaded": [switch.handle_tool_loaded],
            "afc:save_vars": [switch.handle_save_vars],
            "afc:calibration_buttons": [switch.handle_calibration_buttons]}
        route, assign = "AFC_BRIDGEBOX_SWITCH_ROUTE", "AFC_BRIDGEBOX_SWITCH_ASSIGN"
        calibration, identify = "AFC_BRIDGEBOX_SWITCH_CALIBRATION", "AFC_BRIDGEBOX_SWITCH_IDENTIFY"
        setup = "AFC_BRIDGEBOX_SWITCH_SETUP"
        assert printer._gcode.register_mux_command.call_args_list == [
            call(route, "SWITCH", "Switch_1", switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE_help),
            call(route, "SWITCH", None, switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE_help),
            call(assign, "SWITCH", "Switch_1", switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN_help),
            call(assign, "SWITCH", None, switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN_help),
            call(calibration, "SWITCH", "Switch_1", switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION_help),
            call(calibration, "SWITCH", None, switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION_help),
            call(identify, "SWITCH", "Switch_1", switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY_help),
            call(identify, "SWITCH", None, switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY_help),
            call(setup, "SWITCH", "Switch_1", switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP_help),
            call(setup, "SWITCH", None, switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP,
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP_help)]

    def test_a_bad_section_is_refused(self):
        with pytest.raises(Exception) as err:
            make_switch(connect=False, inlet_a=None, inlet_b=None)
        assert str(err.value) == f"[{SECTION}] names no unit. Add inlet_a or inlet_b."

    def test_a_switch_with_a_section_of_its_own_is_not_chain_made(self):
        switch = make_switch(connect=False)
        assert switch.chain_made is False
        assert switch.section == SECTION
        assert switch.by_hand == []
        assert (switch._watched, switch._read, switch._setup_offered) == ({}, [], False)

    def test_a_switch_its_chain_made_has_its_options_in_the_chains_section(self):
        switch = made(connect=False)
        assert switch.chain_made is True
        assert (switch.fullname, switch.name) == (MADE, "Chain_1")
        assert switch.section == "AFC_BridgeBox Chain_1"
        # Its chain is on both inlets, so the inlet of each bay is found by loading.
        assert switch.finding == ["Chain_1"]
        assert switch.inlets == {"A": [], "B": []}
        assert switch.outlets == {"A": None, "B": None}
        assert (switch.start, switch.rest) == ({"A": "A", "B": "B"}, {"A": "A", "B": "B"})

    def test_the_options_its_chains_section_sets_are_kept_by_name(self):
        switch = made(connect=False, by_hand=[" outlet_a ", "", "inlet_b"], outlet_a="extruder",
                      inlet_b=["Bambu_AMS_2"])
        assert switch.by_hand == ["outlet_a", "inlet_b"]

    @pytest.mark.parametrize("values, finding, inlets", [
        # With its chain on one inlet every bay is on that inlet, and none is found.
        (dict(inlet_a=["Chain_1"]), [], {"A": ["Chain_1"], "B": []}),
        (dict(inlet_b=["Chain_1"]), [], {"A": [], "B": ["Chain_1"]}),
        # A unit that is put on an inlet leaves the chain on both.
        (dict(inlet_b=["Bambu_AMS_2"]), ["Chain_1"], {"A": [], "B": ["Bambu_AMS_2"]}),
        # The chain written on both is on both once.
        (dict(inlet_a=["Chain_1"], inlet_b=["Chain_1"]), ["Chain_1"], {"A": [], "B": []}),
    ])
    def test_a_chain_made_switch_has_its_chain_on_both_inlets_unless_it_is_put_on_one(
            self, values, finding, inlets):
        switch = made(connect=False, **values)
        assert switch.finding == finding
        assert switch.inlets == inlets

    def test_a_section_that_names_its_chain_on_no_inlet_finds_no_unit_by_loading(self):
        assert make_switch(connect=False, chain="Chain_1").finding == []


class TestAfcBridgeBoxSwitchRegister:
    def test_a_second_switch_does_not_take_the_default(self):
        # Klipper refuses a mux value that is taken, and the first switch has None.
        printer = make_printer()
        switch = make_switch(printer, connect=False)
        handler = MagicMock()
        printer._gcode.register_mux_command = MagicMock(side_effect=[None, error("taken")])
        switch._register("CMD", handler, "what it does")
        assert printer._gcode.register_mux_command.call_args_list == [
            call("CMD", "SWITCH", "Switch_1", handler, desc="what it does"),
            call("CMD", "SWITCH", None, handler, desc="what it does")]

    def test_its_own_name_being_taken_is_an_error(self):
        printer = make_printer()
        switch = make_switch(printer, connect=False)
        printer._gcode.register_mux_command = MagicMock(side_effect=error("taken"))
        with pytest.raises(error) as err:
            switch._register("CMD", MagicMock(), "what it does")
        assert str(err.value) == "taken"

    def test_another_failure_of_the_default_is_not_hidden(self):
        printer = make_printer()
        switch = make_switch(printer, connect=False)
        printer._gcode.register_mux_command = MagicMock(side_effect=[None, ValueError("x")])
        with pytest.raises(ValueError):
            switch._register("CMD", MagicMock(), "what it does")


class TestAfcBridgeBoxSwitchCheckSection:
    @pytest.mark.parametrize("values, said", [
        (dict(inlet_a=None, inlet_b=None), "names no unit. Add inlet_a or inlet_b."),
        (dict(outlet_a=None, outlet_b=None), "names no extruder. Add outlet_a or outlet_b."),
        (dict(outlet_b="extruder"),
         "has extruder on both outlets. An outlet feeds one extruder."),
    ])
    def test_what_it_refuses(self, values, said):
        with pytest.raises(Exception) as err:
            make_switch(connect=False, **values)
        assert str(err.value) == f"[{SECTION}] {said}"

    @pytest.mark.parametrize("values", [
        dict(inlet_a=None), dict(inlet_b=None), dict(outlet_a=None), dict(outlet_b=None), {},
        # A chain on both inlets, whose units are found by loading, is a unit to name.
        BOTH])
    def test_what_it_takes(self, values):
        make_switch(connect=False, **values)._check_section()

    def test_a_switch_its_chain_made_may_name_no_unit_and_no_extruder(self):
        made(connect=False)._check_section()

    def test_but_not_one_extruder_on_both_outlets(self):
        with pytest.raises(Exception) as err:
            made(connect=False, outlet_a="extruder", outlet_b="extruder")
        assert str(err.value) == ("[AFC_BridgeBox Chain_1] has extruder on both outlets. An "
                                  "outlet feeds one extruder.")


class TestAfcBridgeBoxSwitchStarts:
    @pytest.mark.parametrize("values, starts", [
        ({}, {"A": "A", "B": "B"}),
        (dict(outlet_a=None), {"A": "B", "B": "B"}),
        (dict(outlet_b=None), {"A": "A", "B": "A"}),
    ])
    def test_an_inlet_starts_on_its_own_outlet_or_on_the_only_one_with_an_extruder(self, values,
                                                                                    starts):
        assert make_switch(connect=False, **values)._starts() == starts

    def test_with_no_extruder_on_either_outlet_each_inlet_keeps_its_own(self):
        assert made(connect=False)._starts() == {"A": "A", "B": "B"}


class TestAfcBridgeBoxSwitchKey:
    def test_a_switch_with_a_section_writes_an_option_as_it_is(self):
        assert make_switch(connect=False)._key("outlet_a") == "outlet_a"

    def test_a_switch_its_chain_made_writes_it_with_switch_before_it(self):
        assert made(connect=False)._key("outlet_a") == "switch_outlet_a"


class TestAfcBridgeBoxSwitchLengthKeys:
    def test_both_tube_length_options_as_the_switch_writes_them(self):
        assert make_switch(connect=False)._length_keys() == (
            "outlet_a_length and outlet_b_length")
        assert made(connect=False)._length_keys() == (
            "switch_outlet_a_length and switch_outlet_b_length")


class TestAfcBridgeBoxSwitchSave:
    def test_a_switch_with_a_section_rewrites_its_section(self):
        switch = make_switch()
        switch._save({"outlet_a_length": 850.0, "outlet_b_length": 1010.0})
        assert switch.afc.function.ConfigRewrite.call_args_list == [
            call(SECTION, "outlet_a_length", 850.0), call(SECTION, "outlet_b_length", 1010.0)]
        assert chain_of(switch).saved == []

    def test_a_switch_its_chain_made_has_the_chain_keep_them(self):
        switch = made()
        switch._save({"outlet_a": "extruder", "outlet_a_length": ""})
        assert chain_of(switch).saved == [{"outlet_a": "extruder", "outlet_a_length": ""}]
        switch.afc.function.ConfigRewrite.assert_not_called()


class TestAfcBridgeBoxSwitchHandleConnect:
    def test_it_finds_its_chain(self):
        chain = FakeChain()
        switch = make_switch(make_printer([chain]))
        assert switch.chain is chain
        assert switch.chain_name == "Chain_1"

    def test_the_first_problem_is_raised(self):
        printer = make_printer(units=["Ace2_1"], extruders=["extruder"])
        with pytest.raises(Exception) as err:
            make_switch(printer)
        assert str(err.value) == (
            f"[{SECTION}] names unit Bambu_AMS_1, and there is no AFC unit of that name.")

    def test_a_clash_is_raised_when_nothing_is_missing(self):
        printer = make_printer()
        make_switch(printer, section="AFC_BridgeBox_switch Switch_2", connect=False,
                    inlet_a=["Other"], inlet_b=None, outlet_a="extruder2", outlet_b=None)
        with pytest.raises(Exception) as err:
            make_switch(printer)
        assert str(err.value) == (
            f"[{SECTION}] and [AFC_BridgeBox_switch Switch_2] are both on chain Chain_1. "
            "A chain's bus carries one filament switch.")

    def test_it_keeps_the_lanes_of_its_units(self):
        printer = make_printer()
        mine = [add_lane(printer, "lane1", "Turtle_1"), add_lane(printer, "lane9", "Ace2_1")]
        add_lane(printer, "lane0", "AMS_9")
        # Only a lane counts, whatever unit another object names.
        printer._objects["AFC_hub Turtle_1"] = MagicMock(unit="Turtle_1")
        assert make_switch(printer).lanes == mine

    def test_the_lanes_of_connected_units_go_to_their_starting_extruder(self):
        switch = plumbed()
        assert lane_of(switch, "Turtle_1").extruder_obj is extruder_of(switch, "extruder")
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj is extruder_of(switch, "extruder")
        assert lane_of(switch, "Ace2_1").extruder_obj is extruder_of(switch, "extruder1")
        assert list(switch.printer._event_handlers) == [
            "klippy:connect", "klippy:ready", "afc:prep", "afc:tool_change_unload",
            "afc:tool_load_prepare", "afc:tool_loaded", "afc:save_vars",
            "afc:calibration_buttons"]

    def test_a_unit_that_connects_later_is_started_then(self):
        printer = make_printer(connected=False)
        lane = add_lane(printer, "lane9", "Ace2_1")
        switch = make_switch(printer)
        assert lane.extruder_obj.name == "extruder"
        for unit in ("Bambu_AMS_1", "Turtle_1", "Ace2_1"):
            assert printer._event_handlers[f"AFC_unit_{unit}:connect"] == [switch._start_unit]
        printer.send_event("AFC_unit_Ace2_1:connect", printer._objects["AFC_Unit Ace2_1"])
        assert lane.extruder_obj.name == "extruder1"

    def test_saved_lanes_are_read_before_the_units_start(self, tmp_path):
        printer = make_printer()
        printer._afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {"switches": {
            "Switch_1": {"lanes": {"lane1": "extruder1"}, "confirmed": []}}}}))
        lane = add_lane(printer, "lane1", "Turtle_1")
        switch = make_switch(printer)
        assert switch._saved_lanes == {"lane1": "extruder1"}
        assert lane.extruder_obj.name == "extruder1"

    def test_a_lane_saved_as_loaded_starts_on_that_toolhead(self, tmp_path):
        printer = make_printer()
        printer._afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {
            "extruders": {"extruder": {"lane_loaded": "lane9"},
                          "extruder1": {"lane_loaded": "lane1"}},
            "switches": {"Switch_1": {"lanes": {"lane9": "extruder1"}, "confirmed": [],
                                      "by_print": ["lane1"], "due_back": ["lane9"]}}}}))
        lanes = [add_lane(printer, "lane1", "Turtle_1"), add_lane(printer, "lane9", "Ace2_1")]
        switch = make_switch(printer)
        # What AFC saved as loaded wins over what was saved for the switch.
        assert switch._saved_lanes == {"lane9": "extruder", "lane1": "extruder1"}
        assert [lane.extruder_obj.name for lane in lanes] == ["extruder1", "extruder"]
        assert (switch._by_print, switch._due_back) == (["lane1"], ["lane9"])

    def test_the_saved_units_of_this_switch_are_confirmed_again(self, tmp_path):
        printer = make_printer()
        printer._afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {"switches": {
            "Switch_1": {"lanes": {}, "confirmed": ["Turtle_1", "Gone_1", "Ace2_1"]}}}}))
        # A unit that was taken out of the section since is dropped.
        assert make_switch(printer).confirmed == ["Turtle_1", "Ace2_1"]

    def test_the_saved_lanes_a_print_moved_are_read_with_the_lanes(self, tmp_path):
        printer = make_printer()
        printer._afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {"switches": {"Switch_1": {
            "lanes": {"lane1": "extruder1", "lane2": "extruder1", "lane9": "extruder"},
            "confirmed": [], "by_print": ["lane1", "lane3"],
            "due_back": ["lane4", "lane9", "lane2"]}}}}))
        switch = make_switch(printer)
        # A lane that is not saved as off its starting extruder has nowhere to go back from.
        assert switch._by_print == ["lane1"]
        assert switch._due_back == ["lane9", "lane2"]
        assert switch._printing is False

    def test_with_nothing_saved_no_lane_is_away_and_no_unit_confirmed(self):
        switch = plumbed()
        assert switch._saved_lanes == {}
        assert (switch._by_print, switch._due_back) == ([], [])
        assert switch.confirmed == []

    def test_a_chain_named_on_an_inlet_stands_for_its_units(self):
        printer = make_printer([FakeChain(bays=["Bambu_AMS_1", "Turtle_1"])])
        lane = add_lane(printer, "lane1", "Turtle_1", extruder="extruder1")
        switch = make_switch(printer, inlet_a=["Chain_1"])
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Ace2_1"]}
        # The chain's units are known by the time the lanes are kept and started.
        assert switch.lanes == [lane]
        assert lane.extruder_obj.name == "extruder"

    def test_the_bays_of_a_chain_follow_the_unit_of_it_an_inlet_names(self):
        printer = make_printer([FakeChain(bays=["Bambu_AMS_1", "Bambu_AMS_2"])],
                               units=VALUES["inlet_a"] + VALUES["inlet_b"] + ["Bambu_AMS_2"])
        lane = add_lane(printer, "lane4", "Bambu_AMS_2", extruder="extruder1")
        switch = make_switch(printer)
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1", "Bambu_AMS_2"],
                                 "B": ["Ace2_1"]}
        # They are on the inlet by the time the lanes are kept and started.
        assert switch.lanes == [lane]
        assert lane.extruder_obj.name == "extruder"

    def test_a_chain_with_every_bay_named_may_be_on_both_inlets(self):
        printer = make_printer([FakeChain(bays=["Bambu_AMS_1", "Ace2_1"])])
        assert make_switch(printer).inlets == {"A": ["Bambu_AMS_1", "Turtle_1"],
                                               "B": ["Ace2_1"]}

    def test_a_chain_with_units_on_both_inlets_and_a_bay_on_neither_is_refused(self):
        printer = make_printer([FakeChain(bays=["Bambu_AMS_1", "Ace2_1", "Bambu_AMS_2"])])
        with pytest.raises(error) as err:
            make_switch(printer)
        assert str(err.value) == (f"[{SECTION}] has units of chain Chain_1 on both inlets. Put "
                                  "Chain_1 itself on the inlet its other units enter.")

    def test_a_chain_with_a_unit_on_another_switch_is_a_clash(self):
        chains = [FakeChain("Chain_1", bays=["Bambu_AMS_1", "Other"]), FakeChain("Chain_2")]
        printer = make_printer(chains, units=VALUES["inlet_a"] + VALUES["inlet_b"] + ["Other"],
                               extruders=["extruder", "extruder1", "extruder2"])
        make_switch(printer, section="AFC_BridgeBox_switch Switch_2", connect=False,
                    chain="Chain_2", inlet_a=["Other"], inlet_b=None, outlet_a="extruder2",
                    outlet_b=None)
        with pytest.raises(error) as err:
            make_switch(printer, chain="Chain_1")
        assert str(err.value) == (f"[{SECTION}] and [AFC_BridgeBox_switch Switch_2] both name "
                                  "Other.")

    def test_a_chain_on_one_inlet_keeps_off_the_unit_of_it_on_the_other(self):
        printer = make_printer([FakeChain(bays=["Bambu_AMS_1", "Ace2_1"])])
        lane = add_lane(printer, "lane9", "Ace2_1", extruder="extruder")
        switch = make_switch(printer, inlet_a=["Chain_1"])
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": ["Ace2_1"]}
        assert switch.lanes == [lane]
        assert lane.extruder_obj.name == "extruder1"

    def test_a_section_whose_chains_have_no_bay_names_no_unit(self):
        with pytest.raises(error) as err:
            make_switch(inlet_a=["Chain_1"], inlet_b=None)
        assert str(err.value) == f"[{SECTION}] names no unit. Add inlet_a or inlet_b."

    def test_the_units_of_a_chain_on_both_inlets_wait_to_be_found(self):
        switch = finding()
        chain = chain_of(switch)
        assert switch._floating == {"Bambu_AMS_1": chain, "Bambu_AMS_2": chain}
        assert switch.inlets == {"A": [], "B": []}
        assert (switch._found, switch._held) == ({}, BAYS)
        # Their lanes are the switch's all the same, and stay on the extruder they are on.
        assert switch.lanes == [lane_of(switch, unit) for unit in BAYS]
        assert [lane.extruder_obj.name for lane in switch.lanes] == ["extruder", "extruder"]
        assert [ams_of(switch, unit).behind for unit in BAYS] == [[("Buffer", "chip")]] * 2
        assert [ams_of(switch, unit).assumed for unit in BAYS] == [True, True]

    def test_a_unit_enters_the_inlet_saved_for_the_ams_in_its_bay(self, tmp_path):
        switch = finding(connect=False)
        switch.afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {"switches": {"Switch_1": {
            "lanes": {}, "confirmed": ["Bambu_AMS_1", "Bambu_AMS_2"],
            "found": {"BBBB": "B", "CCCC": "A"}}}}}))
        switch.handle_connect()
        # What was found for an AMS that holds no bay now is kept until the watch drops it.
        assert switch._found == {"BBBB": "B", "CCCC": "A"}
        assert switch._held == BAYS
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_2"]}
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder1"
        assert ams_of(switch, "Bambu_AMS_2").behind == [("Buffer_B", "chip_inlet_b")]
        assert ams_of(switch, "Bambu_AMS_2").assumed is False
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder"
        assert ams_of(switch, "Bambu_AMS_1").behind == [("Buffer", "chip")]
        assert ams_of(switch, "Bambu_AMS_1").assumed is True
        # A unit whose inlet is not known was not seen at one.
        assert switch.confirmed == ["Bambu_AMS_2"]

    def test_a_bay_that_holds_no_ams_enters_no_inlet(self, tmp_path):
        chain = SimChain()
        switch = finding(chain, connect=False)
        chain.uids = {"Bambu_AMS_1": "AAAA"}
        switch.afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {"switches": {"Switch_1": {
            "lanes": {}, "confirmed": [], "found": {"AAAA": "A"}}}}}))
        switch.handle_connect()
        assert switch._held == {"Bambu_AMS_1": "AAAA", "Bambu_AMS_2": None}
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": []}

    def test_a_unit_on_both_inlets_waits_to_be_found(self):
        switch = sharing()
        assert switch._floating == {"Turtle_1": None}
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": ["Ace2_1"]}
        # It is on no chain, so what is found for it is kept for its own name.
        assert (switch._found, switch._held) == ({}, {"Turtle_1": "Turtle_1"})
        # Its lane is the switch's all the same, and stays on the extruder it is on.
        assert lane_of(switch, "Turtle_1") in switch.lanes
        assert lane_of(switch, "Turtle_1").extruder_obj.name == "extruder"

    def test_such_a_unit_enters_the_inlet_saved_for_its_name(self, tmp_path):
        switch = sharing(connect=False)
        switch.afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {"switches": {"Switch_1": {
            "lanes": {}, "confirmed": ["Turtle_1"], "found": {"Turtle_1": "B"}}}}}))
        switch.handle_connect()
        assert switch._found == {"Turtle_1": "B"}
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": ["Ace2_1", "Turtle_1"]}
        assert lane_of(switch, "Turtle_1").extruder_obj.name == "extruder1"
        assert switch.confirmed == ["Turtle_1"]

    def test_with_no_toolhead_on_an_outlet_the_saved_lanes_are_dropped(self, tmp_path):
        switch = made(connect=False)
        switch.afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {"switches": {"Chain_1": {
            "lanes": {"lane_Bambu_AMS_1": "extruder1"}, "confirmed": [],
            "by_print": ["lane_Bambu_AMS_1"], "due_back": ["lane_Bambu_AMS_1"],
            "found": {"AAAA": "A"}}}}}))
        switch.handle_connect()
        # They were saved for outlets the switch does not have now.
        assert switch._saved_lanes == {}
        assert (switch._by_print, switch._due_back) == ([], [])
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": []}
        # The lane stays on the extruder its unit names.
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder"


class TestAfcBridgeBoxSwitchHandleReady:
    def test_it_starts_the_watch_one_period_from_now(self):
        switch = make_switch()
        switch.handle_ready()
        switch.reactor.register_timer.assert_called_once_with(switch._watch, 101.0)
        assert switch_mod._WATCH_PERIOD == 1.0


class TestAfcBridgeBoxSwitchSaved:
    @staticmethod
    def _switch(tmp_path, text: Optional[str]) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch whose AFC saved ``text``, None for no file"""
        switch = make_switch(connect=False)
        switch.afc.VarFile = str(tmp_path / "AFC.var")
        if text is not None:
            (tmp_path / "AFC.var.unit").write_text(text)
        return switch

    def test_the_lanes_and_units_saved_for_this_switch(self, tmp_path):
        saved = {"system": {"switches": {
            "Switch_1": {"lanes": {"lane1": "extruder1", "lane2": "extruder1"},
                         "confirmed": ["Turtle_1"], "by_print": ["lane1"],
                         "due_back": ["lane2"]},
            "Switch_2": {"lanes": {"lane20": "extruder3"}, "confirmed": ["Other"],
                         "by_print": ["lane20"], "due_back": []}}}}
        assert self._switch(tmp_path, json.dumps(saved))._saved() == {
            "lanes": {"lane1": "extruder1", "lane2": "extruder1"}, "confirmed": ["Turtle_1"],
            "by_print": ["lane1"], "due_back": ["lane2"], "found": {}}

    def test_the_inlet_found_for_each_ams(self, tmp_path):
        saved = {"system": {"switches": {"Switch_1": {
            "lanes": {}, "confirmed": [],
            # Only an inlet's letter is one that was found.
            "found": {"AAAA": "A", "BBBB": "B", "CCCC": "C", "DDDD": None, "EEEE": 1}}}}}
        assert self._switch(tmp_path, json.dumps(saved))._saved() == {
            "lanes": {}, "confirmed": [], "by_print": [], "due_back": [],
            "found": {"AAAA": "A", "BBBB": "B"}}

    @pytest.mark.parametrize("extra, by_print, due_back", [
        ({}, [], []), ({"by_print": ["lane1"]}, ["lane1"], []),
        ({"due_back": ["lane1"]}, [], ["lane1"])])
    def test_what_was_saved_without_a_prints_lanes_reads_with_none(self, tmp_path, extra,
                                                                   by_print, due_back):
        saved = {"system": {"switches": {"Switch_1": dict(
            {"lanes": {"lane1": "extruder1"}, "confirmed": ["Turtle_1"]}, **extra)}}}
        assert self._switch(tmp_path, json.dumps(saved))._saved() == {
            "lanes": {"lane1": "extruder1"}, "confirmed": ["Turtle_1"],
            "by_print": by_print, "due_back": due_back, "found": {}}

    @pytest.mark.parametrize("text", [
        None, "", "{not json", "{}", '{"system": {}}', '{"system": null}',
        '{"system": {"switches": {"Switch_2": {"lanes": {"lane1": "extruder1"}, '
        '"confirmed": ["Turtle_1"]}}}}',
        '{"system": {"switches": {"Switch_1": 5}}}',
        # What was saved before units were confirmed: lane name -> extruder name.
        '{"system": {"switches": {"Switch_1": {"lane1": "extruder1"}}}}',
        '{"system": {"switches": {"Switch_1": {"lanes": {"lane1": "extruder1"}}}}}',
        '{"system": {"switches": {"Switch_1": {"confirmed": ["Turtle_1"]}}}}',
        '{"system": {"switches": {"Switch_1": {"lanes": 5, "confirmed": ["Turtle_1"]}}}}',
        '{"system": {"switches": {"Switch_1": {"lanes": "ab", "confirmed": ["Turtle_1"]}}}}',
        '{"system": {"switches": {"Switch_1": {"lanes": {"lane1": "extruder1"}, '
        '"confirmed": 5}}}}',
        '{"system": {"switches": {"Switch_1": {"lanes": {"lane1": "extruder1"}, '
        '"confirmed": [], "by_print": 5}}}}',
        '{"system": {"switches": {"Switch_1": {"lanes": {"lane1": "extruder1"}, '
        '"confirmed": [], "due_back": 5}}}}',
        '{"system": {"switches": {"Switch_1": {"lanes": {"lane1": "extruder1"}, '
        '"confirmed": [], "found": 5}}}}',
        '{"system": {"switches": {"Switch_1": {"lanes": {"lane1": "extruder1"}, '
        '"confirmed": [], "found": "ab"}}}}'])
    def test_nothing_usable_saved_reads_as_nothing(self, tmp_path, text):
        assert self._switch(tmp_path, text)._saved() == {
            "lanes": {}, "confirmed": [], "by_print": [], "due_back": [], "found": {}}


class TestAfcBridgeBoxSwitchSavedLoaded:
    @staticmethod
    def _switch(tmp_path, text: Optional[str]) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch whose AFC saved ``text``, None for no file"""
        switch = make_switch(connect=False)
        switch.afc.VarFile = str(tmp_path / "AFC.var")
        if text is not None:
            (tmp_path / "AFC.var.unit").write_text(text)
        return switch

    def test_the_lanes_loaded_in_the_switchs_toolheads(self, tmp_path):
        saved = {"system": {"extruders": {
            "extruder": {"lane_loaded": "lane9"}, "extruder1": {"lane_loaded": "lane1"},
            # Not one of the switch's.
            "extruder2": {"lane_loaded": "lane20"}}}}
        assert self._switch(tmp_path, json.dumps(saved))._saved_loaded() == {
            "lane9": "extruder", "lane1": "extruder1"}

    def test_an_empty_toolhead_and_one_that_was_not_saved_name_no_lane(self, tmp_path):
        saved = {"system": {"extruders": {"extruder": {"lane_loaded": None}}}}
        assert self._switch(tmp_path, json.dumps(saved))._saved_loaded() == {}

    @pytest.mark.parametrize("text", [
        None, "", "{not json", "{}", '{"system": {}}', '{"system": null}',
        '{"system": {"extruders": null}}', '{"system": {"extruders": 5}}',
        '{"system": {"extruders": {"extruder": 5}}}',
        '{"system": {"extruders": {"extruder": {}}}}',
        '{"system": {"extruders": {"extruder": {"lane_loaded": ["lane1"]}}}}'])
    def test_nothing_usable_saved_reads_as_no_lane(self, tmp_path, text):
        assert self._switch(tmp_path, text)._saved_loaded() == {}


class TestAfcBridgeBoxSwitchHandleSaveVars:
    def test_lanes_on_their_starting_extruder_are_not_listed(self):
        switch = plumbed()
        data: Dict[str, Any] = {"system": {}}
        switch.handle_save_vars(data)
        assert data == {"system": {"switches": {"Switch_1": {
            "lanes": {}, "confirmed": [], "by_print": [], "due_back": [], "found": {}}}}}

    def test_lanes_off_their_starting_extruder_are_listed(self):
        switch = plumbed()
        lane_of(switch, "Turtle_1").set_extruder(extruder_of(switch, "extruder1"))
        lane_of(switch, "Ace2_1").set_extruder(extruder_of(switch, "extruder"))
        data: Dict[str, Any] = {"system": {"switches": {"Switch_2": {"lane20": "extruder3"}}}}
        switch.handle_save_vars(data)
        assert data == {"system": {"switches": {
            "Switch_2": {"lane20": "extruder3"},
            "Switch_1": {"lanes": {"lane_Turtle_1": "extruder1", "lane_Ace2_1": "extruder"},
                         "confirmed": [], "by_print": [], "due_back": [], "found": {}}}}}

    def test_the_lanes_a_print_moved_are_listed_in_lists_of_their_own(self):
        switch = plumbed()
        lane_of(switch, "Turtle_1").set_extruder(extruder_of(switch, "extruder1"))
        lane_of(switch, "Ace2_1").set_extruder(extruder_of(switch, "extruder"))
        switch._by_print = ["lane_Turtle_1"]
        switch._due_back = ["lane_Ace2_1"]
        data: Dict[str, Any] = {"system": {}}
        switch.handle_save_vars(data)
        saved = data["system"]["switches"]["Switch_1"]
        assert saved == {"lanes": {"lane_Turtle_1": "extruder1", "lane_Ace2_1": "extruder"},
                         "confirmed": [], "by_print": ["lane_Turtle_1"],
                         "due_back": ["lane_Ace2_1"], "found": {}}
        # What is saved does not change with the switch's own lists.
        assert saved["by_print"] is not switch._by_print
        assert saved["due_back"] is not switch._due_back

    def test_the_units_seen_at_their_inlet_are_listed_in_a_list_of_their_own(self):
        switch = plumbed()
        switch.confirmed = ["Turtle_1", "Ace2_1"]
        data: Dict[str, Any] = {"system": {}}
        switch.handle_save_vars(data)
        saved = data["system"]["switches"]["Switch_1"]
        assert saved == {"lanes": {}, "confirmed": ["Turtle_1", "Ace2_1"], "by_print": [],
                         "due_back": [], "found": {}}
        # What is saved does not grow with the switch's own list.
        assert saved["confirmed"] is not switch.confirmed

    def test_the_inlet_found_for_each_ams_is_listed(self):
        switch = finding()
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        for unit in BAYS:
            lane_of(switch, unit).set_extruder(extruder_of(switch, "extruder1"))
        data: Dict[str, Any] = {"system": {}}
        switch.handle_save_vars(data)
        saved = data["system"]["switches"]["Switch_1"]
        # Only the lane of the unit on inlet A is off the extruder its unit starts on.
        assert saved == {"lanes": {"lane_Bambu_AMS_1": "extruder1"}, "confirmed": [],
                         "by_print": [], "due_back": [], "found": {"AAAA": "A", "BBBB": "B"}}
        assert saved["found"] is not switch._found

    @pytest.mark.parametrize("extruder", ["extruder", "extruder1"])
    def test_a_lane_of_a_unit_whose_inlet_is_not_known_is_off_no_starting_extruder(self,
                                                                                    extruder):
        switch = finding()
        lane_of(switch, "Bambu_AMS_1").set_extruder(extruder_of(switch, extruder))
        data: Dict[str, Any] = {"system": {}}
        switch.handle_save_vars(data)
        assert data["system"]["switches"]["Switch_1"]["lanes"] == {}

    def test_with_no_toolhead_on_an_outlet_no_lane_is_off_its_starting_extruder(self):
        switch = made()
        found(switch, Bambu_AMS_1="A")
        lane_of(switch, "Bambu_AMS_1").set_extruder(extruder_of(switch, "extruder1"))
        data: Dict[str, Any] = {"system": {}}
        switch.handle_save_vars(data)
        assert data["system"]["switches"]["Chain_1"] == {
            "lanes": {}, "confirmed": [], "by_print": [], "due_back": [],
            "found": {"AAAA": "A"}}


class TestAfcBridgeBoxSwitchStartExtruder:
    def test_the_extruder_of_the_inlets_own_outlet(self):
        switch = make_switch()
        assert [switch.start_extruder(unit) for unit in ("Turtle_1", "Ace2_1")] == [
            "extruder", "extruder1"]

    @pytest.mark.parametrize("values, only", [
        (dict(outlet_a=None), "extruder1"), (dict(outlet_b=None), "extruder")])
    def test_the_only_extruder_when_the_own_outlet_feeds_none(self, values, only):
        switch = make_switch(connect=False, **values)
        assert [switch.start_extruder(unit) for unit in ("Turtle_1", "Ace2_1")] == [only, only]

    def test_none_while_no_outlet_has_its_toolhead(self):
        switch = made()
        found(switch, Bambu_AMS_1="A")
        assert switch.start_extruder("Bambu_AMS_1") is None


class TestAfcBridgeBoxSwitchExtruder:
    def test_it_looks_the_extruder_up(self):
        switch = make_switch()
        assert switch._extruder("extruder1") is extruder_of(switch, "extruder1")


class TestAfcBridgeBoxSwitchStartUnit:
    def test_only_that_units_lanes_move(self):
        printer = make_printer()
        mine = add_lane(printer, "lane9", "Ace2_1")
        another = add_lane(printer, "lane1", "Turtle_1", extruder="extruder1")
        switch = make_switch(printer, connect=False)
        switch.chain = FakeChain()
        switch.lanes = [mine, another]
        switch._start_unit(printer._objects["AFC_Unit Ace2_1"])
        assert mine.extruder_obj.name == "extruder1"
        assert another.extruder_obj.name == "extruder1"
        # The chain is told of the lanes that were put on an extruder, and of no other.
        assert switch.chain.joined == [["lane9"]]

    def test_a_saved_extruder_wins_over_the_start(self):
        printer = make_printer()
        lane = add_lane(printer, "lane9", "Ace2_1", extruder="extruder1")
        switch = make_switch(printer, connect=False)
        switch.chain = FakeChain()
        switch.lanes = [lane]
        switch._saved_lanes = {"lane9": "extruder"}
        switch._start_unit(printer._objects["AFC_Unit Ace2_1"])
        assert lane.extruder_obj.name == "extruder"
        assert switch.chain.joined == [["lane9"]]

    def test_a_saved_extruder_the_switch_does_not_feed_is_ignored(self):
        printer = make_printer(extruders=["extruder", "extruder1", "extruder2"])
        lane = add_lane(printer, "lane9", "Ace2_1")
        switch = make_switch(printer, connect=False)
        switch.chain = FakeChain()
        switch.lanes = [lane]
        switch._saved_lanes = {"lane9": "extruder2"}
        switch._start_unit(printer._objects["AFC_Unit Ace2_1"])
        assert lane.extruder_obj.name == "extruder1"
        assert switch.chain.joined == [["lane9"]]

    def test_a_unit_that_is_named_on_an_inlet_keeps_its_buffer(self):
        switch = finding(inlet_b=["Chain_1", "Bambu_AMS_2"])
        unit = ams_of(switch, "Bambu_AMS_2")
        assert "Bambu_AMS_2" not in switch._floating
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder1"
        assert unit.behind == []

    @pytest.mark.parametrize("inlet, behind, start", [
        ("A", ("Buffer", "chip"), "extruder"), ("B", ("Buffer_B", "chip_inlet_b"), "extruder1")])
    def test_a_unit_that_was_found_goes_behind_its_inlets_buffer(self, inlet, behind, start):
        switch = finding()
        unit = ams_of(switch, "Bambu_AMS_1")
        switch.inlets[inlet].append("Bambu_AMS_1")
        unit.behind.clear()
        lane_of(switch, "Bambu_AMS_1").set_extruder(extruder_of(switch, other_extruder(start)))
        switch._start_unit(unit)
        assert unit.behind == [behind]
        # Its buffer is known now.
        assert unit.assumed is False
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == start
        # The other unit's lane is left alone.
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder"

    @pytest.mark.parametrize("on", ["extruder", "extruder1"])
    def test_while_its_inlet_is_not_known_its_lanes_stay_where_they_are(self, on):
        switch = finding()
        unit = ams_of(switch, "Bambu_AMS_1")
        unit.behind.clear()
        lane = lane_of(switch, "Bambu_AMS_1")
        lane.set_extruder(extruder_of(switch, on))
        switch.chain.joined.clear()
        switch._start_unit(unit)
        # It keeps the chain's own buffer until then, which is only assumed to be its own.
        assert unit.behind == [("Buffer", "chip")]
        assert unit.assumed is True
        assert lane.extruder_obj.name == on
        # No lane was put on an extruder.
        assert switch.chain.joined == [[]]

    def test_but_a_lane_saved_on_one_of_the_switchs_extruders_is_put_there(self):
        switch = finding()
        switch._saved_lanes = {"lane_Bambu_AMS_1": "extruder1"}
        switch.chain.joined.clear()
        switch._start_unit(ams_of(switch, "Bambu_AMS_1"))
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder1"
        assert switch.chain.joined == [["lane_Bambu_AMS_1"]]

    def test_and_one_saved_on_an_extruder_the_switch_does_not_feed_stays(self):
        switch = finding()
        switch._saved_lanes = {"lane_Bambu_AMS_1": "extruder2"}
        switch._start_unit(ams_of(switch, "Bambu_AMS_1"))
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder"

    def test_a_unit_on_no_chain_that_was_found_goes_behind_no_buffer(self):
        switch = sharing()
        unit = switch.afc.units["Turtle_1"]
        # Only a chain's unit has a buffer to go behind.
        assert not hasattr(unit, "use_buffer")
        switch.inlets["B"].append("Turtle_1")
        switch._start_unit(unit)
        assert lane_of(switch, "Turtle_1").extruder_obj.name == "extruder1"


class TestAfcBridgeBoxSwitchChains:
    def test_every_chain_by_its_name(self):
        chains = [FakeChain("Chain_1"), FakeChain("Chain_2")]
        switch = make_switch(make_printer(chains), connect=False)
        assert switch._chains() == {"Chain_1": chains[0], "Chain_2": chains[1]}

    def test_no_chain_is_configured(self):
        assert make_switch(make_printer([]), connect=False)._chains() == {}

    def test_a_section_that_is_not_a_chain_does_not_count(self):
        # An [AFC_BridgeBox <unit>] section without a serial_port only holds overrides.
        chain = FakeChain("Chain_1")
        printer = make_printer([chain])
        printer._objects["AFC_BridgeBox Bambu_AMS_1"] = MagicMock()
        printer._objects["AFC_BridgeBox Bambu_AMS_1"].name = "Bambu_AMS_1"
        assert make_switch(printer, connect=False)._chains() == {"Chain_1": chain}

    def test_a_chain_kept_under_another_section_does_not_count(self):
        printer = make_printer([FakeChain("Chain_1")])
        printer._objects["AFC_hub Chain_2"] = FakeChain("Chain_2")
        assert list(make_switch(printer, connect=False)._chains()) == ["Chain_1"]


class TestAfcBridgeBoxSwitchPlaceChains:
    AS_NAMED = {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Ace2_1"]}

    @staticmethod
    def _switch(chains: List[FakeChain], **values: Any) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch that has not connected, on Chain_1"""
        return make_switch(make_printer(chains), connect=False, chain="Chain_1", **values)

    def test_units_are_left_as_they_are(self):
        switch = self._switch([FakeChain(bays=["Bay_1", "Bay_2"])])
        switch._place_chains()
        assert switch.inlets == self.AS_NAMED

    def test_a_chain_is_replaced_by_its_units_where_it_stood(self):
        switch = self._switch([FakeChain(bays=["Bay_1", "Bay_2"])],
                              inlet_a=["Turtle_1", "Chain_1", "Bambu_AMS_1"])
        switch._place_chains()
        assert switch.inlets == {"A": ["Turtle_1", "Bay_1", "Bay_2", "Bambu_AMS_1"],
                                 "B": ["Ace2_1"]}

    def test_a_chain_on_each_inlet(self):
        chains = [FakeChain("Chain_1", bays=["Bay_1"]), FakeChain("Chain_2", bays=["Bay_2"])]
        switch = self._switch(chains, inlet_a=["Chain_2"], inlet_b=["Chain_1"])
        switch._place_chains()
        assert switch.inlets == {"A": ["Bay_2"], "B": ["Bay_1"]}

    def test_a_chain_keeps_off_the_units_of_it_the_other_inlet_names(self):
        # Those units are behind a second buffer, which feeds the other inlet.
        switch = self._switch([FakeChain(bays=["Bay_1", "Ace2_1", "Bay_3"])],
                              inlet_a=["Chain_1"])
        switch._place_chains()
        assert switch.inlets == {"A": ["Bay_1", "Bay_3"], "B": ["Ace2_1"]}

    def test_a_chain_name_that_is_also_a_unit_name_is_left_as_the_unit(self):
        switch = self._switch([FakeChain(), FakeChain("Turtle_1", bays=["Bay_1", "Bay_2"])])
        switch._place_chains()
        assert switch.inlets == self.AS_NAMED

    def test_only_an_afc_unit_of_that_name_keeps_a_chain_from_being_replaced(self):
        printer = make_printer([FakeChain(bays=["Bay_1"])])
        printer._objects["AFC_hub Chain_1"] = MagicMock()
        printer._objects["AFC_hub Chain_1"].name = "Chain_1"
        switch = make_switch(printer, connect=False, inlet_b=["Chain_1"])
        switch._place_chains()
        assert switch.inlets["B"] == ["Bay_1"]

    def test_a_name_that_is_neither_is_left_for_the_check_of_what_is_missing(self):
        switch = self._switch([FakeChain(bays=["Bay_1"])], inlet_b=["Nothing_1"])
        switch._place_chains()
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Nothing_1"]}

    def test_a_chain_without_bays_leaves_nothing_on_its_inlet(self):
        switch = self._switch([FakeChain()], inlet_b=["Chain_1"])
        switch._place_chains()
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": []}

    def test_the_bays_no_inlet_names_join_the_inlet_of_the_named_one(self):
        switch = self._switch([FakeChain(bays=["Bay_0", "Bambu_AMS_1", "Bay_2"])])
        switch._place_chains()
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1", "Bay_0", "Bay_2"],
                                 "B": ["Ace2_1"]}

    def test_each_chain_follows_its_own_named_unit(self):
        chains = [FakeChain("Chain_1", bays=["Bambu_AMS_1", "Bay_1"]),
                  FakeChain("Chain_2", bays=["Bay_4", "Ace2_1"]),
                  FakeChain("Chain_3", bays=["Bay_8"])]
        switch = self._switch(chains)
        switch._place_chains()
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1", "Bay_1"],
                                 "B": ["Ace2_1", "Bay_4"]}

    def test_a_chain_with_units_on_both_inlets_and_bays_on_neither_is_refused(self):
        chains = [FakeChain("Chain_1", bays=["Bay_0"]),
                  FakeChain("Chain_2", bays=["Bambu_AMS_1", "Bay_5", "Ace2_1"])]
        switch = self._switch(chains)
        with pytest.raises(error) as err:
            switch._place_chains()
        assert str(err.value) == (f"[{SECTION}] has units of chain Chain_2 on both inlets. Put "
                                  "Chain_2 itself on the inlet its other units enter.")
        # Nothing was added for it.
        assert switch.inlets == self.AS_NAMED

    def test_the_bays_of_a_chain_on_both_inlets_are_found_by_loading(self):
        chains = [FakeChain("Chain_1", bays=["Bay_0", "Bay_1"]),
                  FakeChain("Chain_2", bays=["Bay_4"])]
        switch = self._switch(chains, inlet_a=["Chain_1", "Chain_2"], inlet_b=["Chain_1"])
        switch._place_chains()
        assert switch._floating == {"Bay_0": chains[0], "Bay_1": chains[0]}
        # A chain on one inlet has its bays on that inlet as ever.
        assert switch.inlets == {"A": ["Bay_4"], "B": []}

    def test_a_bay_of_such_a_chain_that_an_inlet_names_stays_on_that_inlet(self):
        chain = FakeChain(bays=["Bay_0", "Ace2_1", "Bay_2"])
        switch = self._switch([chain], inlet_a=["Chain_1"], inlet_b=["Chain_1", "Ace2_1"])
        switch._place_chains()
        assert switch._floating == {"Bay_0": chain, "Bay_2": chain}
        assert switch.inlets == {"A": [], "B": ["Ace2_1"]}

    def test_units_on_both_inlets_are_found_by_loading(self):
        switch = self._switch([FakeChain(bays=["Bay_0"])],
                              inlet_b=["Turtle_1", "Ace2_1", "Bambu_AMS_1"])
        switch._place_chains()
        # They are on no chain.
        assert switch._floating == {"Bambu_AMS_1": None, "Turtle_1": None}
        assert switch.inlets == {"A": [], "B": ["Ace2_1"]}

    def test_a_unit_and_a_chain_can_both_be_on_both_inlets(self):
        chain = FakeChain(bays=["Bay_0", "Bay_1"])
        switch = self._switch([chain], inlet_a=["Chain_1", "Turtle_1"],
                              inlet_b=["Chain_1", "Turtle_1"])
        switch._place_chains()
        assert switch._floating == {"Turtle_1": None, "Bay_0": chain, "Bay_1": chain}
        assert switch.inlets == {"A": [], "B": []}

    def test_a_chains_unit_on_both_inlets_is_refused(self):
        switch = self._switch([FakeChain(bays=["Bay_0", "Bambu_AMS_1"])],
                              inlet_b=["Ace2_1", "Bambu_AMS_1"])
        with pytest.raises(error) as err:
            switch._place_chains()
        assert str(err.value) == (f"[{SECTION}] has Bambu_AMS_1 on both inlets. A chain's unit "
                                  "enters one inlet. Put its chain on both inlets to have the "
                                  "chain's units found by loading.")
        assert switch._floating == {}

    def test_so_is_a_name_that_is_neither_a_chain_nor_a_unit(self):
        switch = self._switch([FakeChain(bays=["Bay_0"])], inlet_a=["Nothing_1"],
                              inlet_b=["Nothing_1"])
        with pytest.raises(error) as err:
            switch._place_chains()
        assert str(err.value) == (f"[{SECTION}] names unit Nothing_1, and there is no AFC unit "
                                  "of that name.")
        assert switch._floating == {}

    def test_a_chain_whose_name_is_also_a_units_is_left_as_the_unit(self):
        chains = [FakeChain(), FakeChain("Turtle_1", bays=["Bay_1", "Bay_2"])]
        switch = self._switch(chains, inlet_a=["Turtle_1"], inlet_b=["Turtle_1"])
        switch._place_chains()
        # The unit is found by loading, and the chain's bays are not.
        assert switch._floating == {"Turtle_1": None}
        assert switch.inlets == {"A": [], "B": []}


class TestAfcBridgeBoxSwitchId:
    def test_a_chains_bay_is_kept_for_the_ams_it_holds(self):
        assert finding()._id("Bambu_AMS_2") == "BBBB"

    def test_a_bay_that_holds_no_ams_is_kept_for_none(self):
        switch = finding()
        chain_of(switch).uids["Bambu_AMS_2"] = None
        assert switch._id("Bambu_AMS_2") is None

    def test_a_unit_on_no_chain_is_kept_for_its_name(self):
        assert sharing()._id("Turtle_1") == "Turtle_1"


class TestAfcBridgeBoxSwitchFindChain:
    def test_the_named_chain(self):
        chains = [FakeChain("Chain_1"), FakeChain("Chain_2")]
        switch = make_switch(make_printer(chains), connect=False, chain="Chain_2")
        assert switch._find_chain() is chains[1]

    def test_a_named_chain_that_does_not_exist(self):
        switch = make_switch(connect=False, chain="Chain_9")
        with pytest.raises(Exception) as err:
            switch._find_chain()
        assert str(err.value) == (f"[{SECTION}] names chain Chain_9, and there is no "
                                  "[AFC_BridgeBox Chain_9] chain.")

    def test_the_only_chain_when_none_is_named(self):
        chain = FakeChain("Chain_7")
        switch = make_switch(make_printer([chain]), connect=False)
        assert switch._find_chain() is chain
        assert switch.chain_name is None

    def test_several_chains_need_a_name(self):
        printer = make_printer([FakeChain("Chain_1"), FakeChain("Chain_2")])
        switch = make_switch(printer, connect=False)
        with pytest.raises(Exception) as err:
            switch._find_chain()
        assert str(err.value) == (f"[{SECTION}] needs chain: there are 2 BridgeBox chains "
                                  "(Chain_1, Chain_2).")

    def test_no_chain_at_all(self):
        switch = make_switch(make_printer([]), connect=False)
        with pytest.raises(Exception) as err:
            switch._find_chain()
        assert str(err.value) == (f"[{SECTION}] needs a BridgeBox chain, and none is "
                                  "configured.")

    def test_a_section_that_is_not_a_chain_does_not_count(self):
        # An [AFC_BridgeBox <unit>] section without a serial_port only holds overrides.
        chain = FakeChain("Chain_1")
        printer = make_printer([chain])
        printer._objects["AFC_BridgeBox Bambu_AMS_1"] = MagicMock()
        printer._objects["AFC_BridgeBox Bambu_AMS_1"].name = "Bambu_AMS_1"
        assert make_switch(printer, connect=False)._find_chain() is chain


class TestAfcBridgeBoxSwitchMissing:
    def test_nothing_is_missing(self):
        assert make_switch()._missing() == []

    def test_a_missing_unit_and_a_missing_extruder(self):
        printer = make_printer(units=["Bambu_AMS_1", "Ace2_1"], extruders=["extruder1"])
        switch = make_switch(printer, connect=False)
        assert switch._missing() == [
            f"[{SECTION}] names unit Turtle_1, and there is no AFC unit of that name.",
            f"[{SECTION}] names extruder extruder, and there is no [AFC_extruder extruder]."]

    def test_only_an_afc_unit_counts_as_a_unit(self):
        printer = make_printer(units=["Bambu_AMS_1", "Ace2_1"])
        printer._objects["AFC_hub Turtle_1"] = MagicMock()
        printer._objects["AFC_hub Turtle_1"].name = "Turtle_1"
        assert make_switch(printer, connect=False)._missing() == [
            f"[{SECTION}] names unit Turtle_1, and there is no AFC unit of that name."]


class TestAfcBridgeBoxSwitchClashes:
    OTHER = "AFC_BridgeBox_switch Switch_2"

    def _pair(self, chains: List[FakeChain], **values: Any) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: Switch_1, with a Switch_2 made from ``values``"""
        printer = make_printer(chains)
        base = dict(inlet_a=["Other"], inlet_b=None, outlet_a="extruder2", outlet_b=None,
                    chain="Chain_2")
        make_switch(printer, section=self.OTHER, connect=False, **dict(base, **values))
        switch = make_switch(printer, connect=False, chain="Chain_1")
        switch.chain = switch._find_chain()
        switch.chain_name = switch.chain.name
        return switch

    def test_a_switch_alone_clashes_with_nothing(self):
        assert make_switch()._clashes() == []

    def test_another_switch_that_shares_nothing(self):
        switch = self._pair([FakeChain("Chain_1"), FakeChain("Chain_2")])
        assert switch._clashes() == []

    def test_the_same_chain(self):
        switch = self._pair([FakeChain("Chain_1"), FakeChain("Chain_2")], chain="Chain_1")
        assert switch._clashes() == [
            f"[{SECTION}] and [{self.OTHER}] are both on chain Chain_1. A chain's bus "
            "carries one filament switch."]

    def test_a_shared_unit(self):
        switch = self._pair([FakeChain("Chain_1"), FakeChain("Chain_2")],
                            inlet_b=["Ace2_1"])
        assert switch._clashes() == [f"[{SECTION}] and [{self.OTHER}] both name Ace2_1."]

    def test_a_shared_extruder(self):
        switch = self._pair([FakeChain("Chain_1"), FakeChain("Chain_2")],
                            outlet_b="extruder1")
        assert switch._clashes() == [f"[{SECTION}] and [{self.OTHER}] both name extruder1."]

    def test_a_shared_chain_unit_and_extruder(self):
        switch = self._pair([FakeChain("Chain_1"), FakeChain("Chain_2")], chain="Chain_1",
                            inlet_a=["Turtle_1"], outlet_a="extruder")
        assert switch._clashes() == [
            f"[{SECTION}] and [{self.OTHER}] are both on chain Chain_1. A chain's bus "
            "carries one filament switch.",
            f"[{SECTION}] and [{self.OTHER}] both name Turtle_1, extruder."]


class TestAfcBridgeBoxSwitchUnits:
    def test_inlet_a_first(self):
        assert make_switch().units() == ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]

    def test_the_units_whose_inlet_is_not_known_come_last(self):
        switch = finding()
        assert switch.units() == ["Bambu_AMS_1", "Bambu_AMS_2"]
        found(switch, Bambu_AMS_2="A")
        assert switch.units() == ["Bambu_AMS_2", "Bambu_AMS_1"]

    def test_a_unit_that_was_found_is_listed_once_at_its_inlet(self):
        switch = finding()
        found(switch, Bambu_AMS_1="B", Bambu_AMS_2="A")
        assert switch.units() == ["Bambu_AMS_2", "Bambu_AMS_1"]


class TestAfcBridgeBoxSwitchLiveUnits:
    ALL = ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]

    def test_units_that_are_no_bays_are_in_use(self):
        switch = make_switch()
        assert not hasattr(switch.afc.units["Turtle_1"], "pool")
        assert switch.live_units() == self.ALL

    def test_a_bay_that_holds_no_unit_now_is_left_out(self):
        switch = make_switch()
        switch.afc.units["Turtle_1"].pool = True
        assert switch.live_units() == ["Bambu_AMS_1", "Ace2_1"]

    def test_a_bay_that_holds_a_unit_is_in_use(self):
        switch = make_switch()
        switch.afc.units["Turtle_1"].pool = False
        assert switch.live_units() == self.ALL

    def test_a_unit_afc_does_not_have_yet_counts_as_in_use(self):
        switch = make_switch(make_printer(connected=False))
        assert switch.afc.units == {}
        assert switch.live_units() == self.ALL


class TestAfcBridgeBoxSwitchUnfound:
    def test_a_switch_whose_units_are_all_named_has_none(self):
        assert make_switch().unfound() == []

    def test_the_units_in_use_whose_inlet_is_still_to_be_found(self):
        switch = finding()
        assert switch.unfound() == ["Bambu_AMS_1", "Bambu_AMS_2"]
        found(switch, Bambu_AMS_1="B")
        assert switch.unfound() == ["Bambu_AMS_2"]

    def test_a_bay_that_holds_no_unit_now_has_no_inlet_to_find(self):
        switch = finding()
        ams_of(switch, "Bambu_AMS_1").pool = True
        assert switch.unfound() == ["Bambu_AMS_2"]


class TestAfcBridgeBoxSwitchNeed:
    def test_it_names_the_unit_and_the_command_for_it(self):
        assert make_switch()._need("Bambu_AMS_2") == (
            "the inlet of Bambu_AMS_2 is not known. Run AFC_BRIDGEBOX_SWITCH_IDENTIFY "
            "UNIT=Bambu_AMS_2 first.")


class TestAfcBridgeBoxSwitchUnset:
    def test_it_names_the_setup_and_the_options_as_the_switch_writes_them(self):
        assert made(connect=False)._unset() == (
            "the toolhead each outlet feeds is not known. Run AFC_BRIDGEBOX_SWITCH_SETUP "
            "SWITCH=Chain_1 first, or set switch_outlet_a and switch_outlet_b.")


class TestAfcBridgeBoxSwitchExtruders:
    def test_outlet_a_first(self):
        assert make_switch().extruders() == ["extruder", "extruder1"]

    @pytest.mark.parametrize("values, expected", [
        (dict(outlet_a=None), ["extruder1"]), (dict(outlet_b=None), ["extruder"])])
    def test_an_outlet_left_out_is_skipped(self, values, expected):
        assert make_switch(connect=False, **values).extruders() == expected


class TestAfcBridgeBoxSwitchInletOf:
    def test_each_unit_and_a_stranger(self):
        switch = make_switch()
        assert [switch.inlet_of(unit) for unit in ("Turtle_1", "Ace2_1", "AMS_1")] == [
            "A", "B", None]


class TestAfcBridgeBoxSwitchOutletOf:
    def test_each_extruder_and_a_stranger(self):
        switch = make_switch()
        assert [switch.outlet_of(name) for name in ("extruder", "extruder1", "extruder2")] == [
            "A", "B", None]


class TestAfcBridgeBoxSwitchState:
    SAID = "Switch_1: the filament switch is not answering"

    def test_before_it_has_a_chain(self):
        with pytest.raises(SwitchError) as err:
            make_switch(connect=False).state()
        assert str(err.value) == self.SAID

    @pytest.mark.parametrize("state", [None, fs(on=False)])
    def test_a_switch_that_is_not_answering(self, state):
        switch = make_switch(make_printer([FakeChain(state=state)]))
        with pytest.raises(SwitchError) as err:
            switch.state()
        assert str(err.value) == self.SAID

    def test_outlets_come_from_the_position_and_targets_from_the_route(self):
        switch = make_switch(make_printer([FakeChain(state=fs(route=1, position=2))]))
        assert switch.state() == {
            "route": 1,
            "inlets": {"A": {"outlet": "A", "target": "B", "filament": False},
                       "B": {"outlet": "B", "target": "A", "filament": False}}}

    @pytest.mark.parametrize("a, b", [(True, False), (False, True)])
    def test_each_inlet_reads_its_own_sensor(self, a, b):
        switch = make_switch(make_printer([FakeChain(state=fs(a=a, b=b))]))
        inlets = switch.state()["inlets"]
        assert (inlets["A"]["filament"], inlets["B"]["filament"]) == (a, b)


class TestAfcBridgeBoxSwitchSetInlet:
    @staticmethod
    def _switch(state: Dict[str, Any], linked: bool = True,
                then: Optional[List[Any]] = None) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch whose chain reports ``state``"""
        return make_switch(make_printer([FakeChain(state=state, linked=linked, then=then)]))

    def test_an_inlet_already_there_is_left_alone(self):
        # Filament in it does not matter when nothing has to move.
        switch = self._switch(fs(route=2, a=True, b=True))
        assert switch.set_inlet("A", "A") is False
        assert chain_of(switch).routes == []
        switch.reactor.pause.assert_not_called()

    def test_it_moves_and_waits_for_the_switch_to_report_it(self):
        switch = self._switch(fs(route=1), then=[None, fs(route=3, on=False), fs(route=1),
                                                 fs(route=3, position=1), fs(route=3)])
        assert switch.set_inlet("A", "A") is True
        assert chain_of(switch).routes == [3]
        assert chain_of(switch).polls == 5
        assert switch.reactor.pause.call_args_list == [((100.1,),)] * 5

    def test_a_move_the_switch_gave_up_is_asked_for_afresh(self):
        # The route says outlet A and the path is not there. The switch only acts on a route
        # that changes, so the other route goes out first and is held for a moment.
        chain = SimChain(route=2, position=0)
        switch = make_switch(make_printer([chain]))
        assert switch.set_inlet("A", "A") is True
        assert chain.routes == [0, 2]
        assert chain.fs["position"] == 2
        assert switch.reactor.pause.call_args_list == [call(100.1), call(100.5), call(100.1)]
        assert switch_mod._ROUTE_HOLD == 0.5

    def test_a_given_up_release_is_asked_for_afresh_too(self):
        chain = SimChain(route=1, position=3)
        switch = make_switch(make_printer([chain]))
        assert switch.set_inlet("A", "B") is True
        assert chain.routes == [3, 1]
        assert chain.fs["position"] == 1

    @pytest.mark.parametrize("filament", [False, True])
    def test_a_path_already_there_only_has_its_route_sent(self, filament):
        # As after a BridgeBox restart under a loaded path: the filament does not matter.
        chain = SimChain(route=0, position=2, a=filament)
        switch = make_switch(make_printer([chain]))
        assert switch.set_inlet("A", "A") is False
        assert chain.routes == [2]
        assert chain.fs["position"] == 2

    def test_a_route_that_does_not_read_back_for_a_path_already_there_is_no_error(self):
        switch = self._switch(fs(route=0, position=2), then=[fs(route=0, position=2)])
        assert switch.set_inlet("A", "A") is False
        assert chain_of(switch).routes == [2]

    def test_an_inlet_that_holds_filament_is_not_moved(self):
        switch = self._switch(fs(route=0, a=True))
        with pytest.raises(SwitchError) as err:
            switch.set_inlet("A", "A")
        assert str(err.value) == ("Switch_1: inlet A holds filament, so it cannot be moved to "
                                  "outlet A")
        assert chain_of(switch).routes == []

    @pytest.mark.parametrize("state", [fs(route=1, b=True), fs(route=0, position=1, b=True)])
    def test_an_outlet_that_holds_the_other_inlets_filament_is_not_entered(self, state):
        switch = self._switch(state)
        with pytest.raises(SwitchError) as err:
            switch.set_inlet("A", "A")
        assert str(err.value) == ("Switch_1: outlet A holds the filament of inlet B, so inlet "
                                  "A cannot be moved to it")
        assert chain_of(switch).routes == []

    def test_the_other_inlets_filament_in_the_other_outlet_is_no_obstacle(self):
        switch = self._switch(fs(route=0, b=True), then=[fs(route=2, b=True)])
        assert switch.set_inlet("A", "A") is True
        assert chain_of(switch).routes == [2]

    def test_the_other_inlets_filament_only_routed_to_the_outlet_is_no_obstacle(self):
        # Inlet B is asked to outlet A and its path is still at outlet B, where its filament is.
        switch = self._switch(fs(route=1, position=0, b=True),
                              then=[fs(route=3, position=2, b=True)])
        assert switch.set_inlet("A", "A") is True
        assert chain_of(switch).routes == [3]

    def test_the_other_inlet_empty_at_the_outlet_is_no_obstacle(self):
        switch = self._switch(fs(route=1), then=[fs(route=3)])
        assert switch.set_inlet("A", "A") is True
        assert chain_of(switch).routes == [3]

    def test_inlet_b_moves_by_its_own_bit(self):
        switch = self._switch(fs(route=3), then=[fs(route=2)])
        assert switch.set_inlet("B", "B") is True
        assert chain_of(switch).routes == [2]

    def test_no_link_to_the_bridge(self):
        switch = self._switch(fs(route=0), linked=False)
        with pytest.raises(SwitchError) as err:
            switch.set_inlet("A", "A")
        assert str(err.value) == "Switch_1: no link to the BridgeBox of Chain_1"
        switch.reactor.pause.assert_not_called()

    def test_a_switch_that_never_gets_there(self):
        switch = self._switch(fs(route=0), then=[fs(route=2, position=0)])
        with pytest.raises(SwitchError) as err:
            switch.set_inlet("A", "A")
        assert str(err.value) == ("Switch_1: inlet A did not reach outlet A. Check that "
                                  "nothing is left in the switch.")
        assert chain_of(switch).polls == switch_mod._ROUTE_POLLS == 50

    def test_a_switch_that_is_not_answering(self):
        switch = self._switch(fs(on=False))
        with pytest.raises(SwitchError) as err:
            switch.set_inlet("A", "A")
        assert str(err.value) == "Switch_1: the filament switch is not answering"


class TestAfcBridgeBoxSwitchSend:
    @staticmethod
    def _switch(then: List[Any], linked: bool = True) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch whose chain reports ``then`` once a route is sent"""
        return make_switch(make_printer([FakeChain(state=fs(), linked=linked, then=then)]))

    def test_no_link_to_the_bridge(self):
        switch = self._switch([], linked=False)
        with pytest.raises(SwitchError) as err:
            switch._send(2, "A")
        assert str(err.value) == "Switch_1: no link to the BridgeBox of Chain_1"
        switch.reactor.pause.assert_not_called()

    def test_it_waits_until_the_inlet_is_on_the_route(self):
        switch = self._switch([fs(route=0), fs(route=2)])
        assert switch._send(2, "A") is True
        assert chain_of(switch).routes == [2]
        assert switch.reactor.pause.call_args_list == [call(100.1)] * 2

    def test_only_the_inlets_own_path_is_waited_for(self):
        # Inlet B is routed to outlet A and not there, which is not inlet A's business.
        switch = self._switch([fs(route=3, position=2)])
        assert switch._send(3, "A") is True
        assert chain_of(switch).polls == 1

    def test_a_release_is_waited_for_the_same_way(self):
        switch = self._switch([fs(route=1, position=3), fs(route=1)])
        assert switch._send(1, "A") is True
        assert chain_of(switch).polls == 2

    @pytest.mark.parametrize("stays", [
        None, fs(route=2, on=False), fs(route=0, position=2), fs(route=2, position=0)])
    def test_what_does_not_count_as_there(self, stays):
        switch = self._switch([stays])
        assert switch._send(2, "A") is False
        assert chain_of(switch).polls == 50


class TestAfcBridgeBoxSwitchOwn:
    def test_the_switch_is_held_for_the_job_and_let_go_after(self):
        switch = make_switch()
        with switch._own():
            assert switch._moving is True
        assert switch._moving is False
        switch.reactor.pause.assert_not_called()

    def test_it_is_let_go_when_the_job_fails(self):
        switch = make_switch()
        with pytest.raises(SwitchError):
            with switch._own():
                raise SwitchError("x")
        assert switch._moving is False

    def test_it_waits_out_a_job_already_moving_the_switch(self):
        switch = make_switch()
        switch._moving = True
        waits = []

        def other_job_ends(until: float) -> None:
            waits.append(until)
            if len(waits) == 3:
                switch._moving = False

        switch.reactor.pause.side_effect = other_job_ends
        with switch._own():
            assert switch._moving is True
        assert waits == [100.1] * 3
        assert switch._moving is False


class TestAfcBridgeBoxSwitchOutletFor:
    def test_the_outlet_of_the_lanes_extruder(self):
        switch = plumbed()
        assert switch.outlet_for(lane_of(switch, "Turtle_1")) == "A"
        assert switch.outlet_for(lane_of(switch, "Ace2_1")) == "B"

    def test_an_extruder_the_switch_does_not_feed(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(FakeExtruder("extruder2"))
        with pytest.raises(SwitchError) as err:
            switch.outlet_for(lane)
        assert str(err.value) == ("Switch_1: lane_Turtle_1 is on extruder2, which this switch "
                                  "does not feed")


class TestAfcBridgeBoxSwitchLoaded:
    def test_no_lane_is_loaded(self):
        switch = plumbed()
        assert (switch._loaded("A"), switch._loaded("B")) == (None, None)

    def test_the_loaded_lane_of_each_inlet(self):
        switch = plumbed()
        lane_of(switch, "Bambu_AMS_1").tool_loaded = True
        assert switch._loaded("A") is lane_of(switch, "Bambu_AMS_1")
        # Loaded through inlet A, so it is not inlet B's.
        assert switch._loaded("B") is None

    def test_a_lane_of_the_inlet_that_is_not_loaded_does_not_count(self):
        switch = plumbed()
        lane_of(switch, "Ace2_1").tool_loaded = True
        assert switch._loaded("A") is None
        assert switch._loaded("B") is lane_of(switch, "Ace2_1")


class TestAfcBridgeBoxSwitchWatch:
    @staticmethod
    def _switch() -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: an idle switch after PREP, what the watch calls mocked"""
        switch = make_switch()
        switch.afc.prep_done = True
        switch._look = MagicMock()
        switch._release = MagicMock()
        switch._follow = MagicMock()
        switch._note_arrivals = MagicMock()
        switch._send_back = MagicMock()
        switch._offer = MagicMock()
        switch._settle = MagicMock(return_value=["a problem"])
        switch._say = MagicMock()
        return switch

    @staticmethod
    def _only_looked(switch: afcBridgeBoxSwitch) -> None:
        """Assert the watch looked at the inlets and for a print, and did nothing else."""
        switch._look.assert_called_once_with()
        switch._release.assert_called_once_with(False)
        switch._follow.assert_not_called()
        switch._note_arrivals.assert_not_called()
        switch._send_back.assert_not_called()
        switch._offer.assert_not_called()
        switch._settle.assert_not_called()
        switch._say.assert_not_called()

    def test_it_looks_notes_who_came_in_then_settles_and_says_while_holding_the_switch(self):
        switch = self._switch()
        held = []

        def settle() -> List[str]:
            held.append(("settle", switch._moving, switch._look.call_count))
            return ["a problem"]

        switch._release.side_effect = lambda printing: held.append(("release", printing))
        switch._follow.side_effect = lambda: held.append(("follow", switch._moving))
        switch._note_arrivals.side_effect = lambda: held.append(("arrivals", switch._moving))
        switch._send_back.side_effect = lambda: held.append(("back", switch._moving))
        switch._offer.side_effect = lambda: held.append(("offer", switch._moving))
        switch._settle.side_effect = settle
        switch._say.side_effect = lambda problems: held.append(("say", switch._moving))
        assert switch._watch(55.0) == 101.0
        switch._look.assert_called_once_with()
        switch._say.assert_called_once_with(["a problem"])
        assert held == [("release", False), ("follow", False), ("arrivals", False),
                        ("back", False), ("offer", False), ("settle", True, 1), ("say", True)]
        assert switch._moving is False

    def test_while_a_print_runs_no_lane_is_sent_back(self):
        switch = self._switch()
        switch.afc.function.in_print.return_value = True
        assert switch._watch(55.0) == 101.0
        switch._release.assert_called_once_with(True)
        switch._send_back.assert_not_called()
        # Nor is a prompt opened over the print.
        switch._offer.assert_not_called()
        # The switch is still brought in line with the lanes and the units in the bays.
        switch._follow.assert_called_once_with()
        switch._note_arrivals.assert_called_once_with()
        switch._say.assert_called_once_with(["a problem"])

    def test_a_print_is_looked_for_while_afc_is_busy(self):
        switch = self._switch()
        switch.afc.function.in_print.return_value = True
        switch.afc.current_state = "Loading"
        assert switch._watch(55.0) == 101.0
        switch._release.assert_called_once_with(True)
        switch._send_back.assert_not_called()
        switch._settle.assert_not_called()

    def test_before_prep_is_done_it_only_looks(self):
        switch = self._switch()
        switch.afc.prep_done = False
        assert switch._watch(55.0) == 101.0
        self._only_looked(switch)

    @pytest.mark.parametrize("state", ["Loading", "Unloading", "Error"])
    def test_while_afc_is_busy_it_only_looks(self, state):
        switch = self._switch()
        switch.afc.current_state = state
        assert switch._watch(55.0) == 101.0
        self._only_looked(switch)

    def test_while_another_job_moves_the_switch_it_only_looks(self):
        switch = self._switch()
        switch._moving = True
        assert switch._watch(55.0) == 101.0
        self._only_looked(switch)
        assert switch._moving is True

    def test_what_it_sees_and_what_is_wrong_are_noted(self):
        chain = SimChain(route=0, b=True)
        switch = plumbed(chain)
        switch.afc.prep_done = True
        # As after PREP's section.
        switch._reported = switch.live_units()
        assert switch._watch(55.0) == 101.0
        assert switch._seen == {"A": False, "B": True}
        assert chain.routes == [2]
        held = TestAfcBridgeBoxSwitchSettle.HELD_B
        assert switch.logger.messages == [("warning", held)]
        assert switch._told == held

    def test_the_lane_of_a_print_that_ended_goes_back_once_it_is_unloaded(self):
        switch = plumbed(SimChain(route=2))
        switch.afc.prep_done = True
        switch._reported = switch.live_units()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        lane.tool_loaded = True
        switch._by_print = ["lane_Turtle_1"]
        switch.afc.function.in_print.return_value = True
        switch._watch(55.0)
        assert (switch._by_print, switch._due_back) == (["lane_Turtle_1"], [])
        switch.afc.function.in_print.return_value = False
        switch._watch(56.0)
        assert (switch._by_print, switch._due_back) == ([], ["lane_Turtle_1"])
        assert lane.extruder_obj.name == "extruder1"
        lane.tool_loaded = False
        switch._watch(57.0)
        assert switch._due_back == []
        assert lane.extruder_obj is extruder_of(switch, "extruder")
        assert switch.logger.messages == [
            ("info", "Switch_1: lane_Turtle_1 is back on extruder, the print that put it on "
                     "extruder1 is over")]

    def test_a_unit_that_came_in_since_the_section_has_it_printed_again(self):
        switch = plumbed(SimChain(route=2))
        switch.afc.prep_done = True
        switch._reported = ["Turtle_1", "Ace2_1"]
        assert switch._watch(55.0) == 101.0
        assert switch.logger.messages == [
            TestAfcBridgeBoxSwitchHandlePrep.PREPPING,
            ("raw", "Inlet A: Bambu_AMS_1, Turtle_1, at outlet A, empty\n"
                    "Inlet B: Ace2_1, at outlet B, empty\n"
                    + TestAfcBridgeBoxSwitchHandlePrep.OUTLETS)]
        assert switch._reported == ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]
        assert switch._report_by is None

    def test_an_ams_that_took_a_bay_loses_the_bays_inlet_and_is_offered_once(self):
        switch = finding(SimChain(route=2))
        chain = chain_of(switch)
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch._offered = ["AAAA", "BBBB"]
        switch.afc.prep_done = True
        switch._reported = switch.live_units()
        chain.uids["Bambu_AMS_2"] = "CCCC"
        switch._watch(55.0)
        switch._watch(56.0)
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": []}
        assert switch._found == {"AAAA": "A"}
        assert [offer[:2] for offer in chain.offers] == [("Switch_1", "Switch_1 new unit")]
        assert switch._offered == ["AAAA", "BBBB", "CCCC"]


class TestAfcBridgeBoxSwitchFollow:
    def test_a_switch_with_no_unit_to_find_has_nothing_to_follow(self):
        switch = plumbed()
        switch._follow()
        assert switch._held == {}
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Ace2_1"]}
        switch.afc.save_vars.assert_not_called()

    def test_a_unit_enters_the_inlet_found_for_the_ams_in_its_bay(self):
        switch = finding()
        switch._found = {"BBBB": "B"}
        switch._follow()
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_2"]}
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder1"
        assert ams_of(switch, "Bambu_AMS_2").behind == [("Buffer", "chip"),
                                                        ("Buffer_B", "chip_inlet_b")]
        # The unit whose AMS has no inlet yet is left as it is.
        assert ams_of(switch, "Bambu_AMS_1").behind == [("Buffer", "chip")]
        assert (switch._found, switch._held) == ({"BBBB": "B"}, BAYS)
        switch.afc.save_vars.assert_not_called()

    def test_a_unit_that_is_on_its_inlet_is_left_there(self):
        switch = finding()
        found(switch, Bambu_AMS_2="B")
        unit = ams_of(switch, "Bambu_AMS_2")
        unit.behind.clear()
        switch._follow()
        assert unit.behind == []
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_2"]}
        # An AMS that holds its bay keeps what was found for it.
        assert switch._found == {"BBBB": "B"}
        switch.afc.save_vars.assert_not_called()

    def test_a_bay_another_ams_took_has_no_inlet_and_the_ams_that_left_is_forgotten(self):
        switch = finding()
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch.confirmed = ["Bambu_AMS_1", "Bambu_AMS_2"]
        chain_of(switch).uids["Bambu_AMS_2"] = "CCCC"
        switch._follow()
        assert switch._found == {"AAAA": "A"}
        assert switch._held == {"Bambu_AMS_1": "AAAA", "Bambu_AMS_2": "CCCC"}
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": []}
        assert switch.confirmed == ["Bambu_AMS_1"]
        # It is back behind the chain's own buffer, and its lane stays where it was.
        assert ams_of(switch, "Bambu_AMS_2").behind[-1] == ("Buffer", "chip")
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder1"
        switch.afc.save_vars.assert_called_once_with()

    def test_an_ams_that_moved_to_another_bay_takes_its_inlet_along(self):
        switch = finding()
        found(switch, Bambu_AMS_2="B")
        chain_of(switch).uids = {"Bambu_AMS_1": "BBBB", "Bambu_AMS_2": None}
        switch._follow()
        assert switch._found == {"BBBB": "B"}
        assert switch._held == {"Bambu_AMS_1": "BBBB", "Bambu_AMS_2": None}
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_1"]}
        switch.afc.save_vars.assert_not_called()

    def test_an_ams_that_left_before_its_inlet_was_found_leaves_nothing_to_forget(self):
        switch = finding()
        found(switch, Bambu_AMS_2="B")
        chain_of(switch).uids["Bambu_AMS_1"] = None
        switch._follow()
        assert switch._found == {"BBBB": "B"}
        assert switch._held == {"Bambu_AMS_1": None, "Bambu_AMS_2": "BBBB"}
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_2"]}
        switch.afc.save_vars.assert_not_called()

    def test_a_unit_on_no_chain_enters_the_inlet_found_for_its_name(self):
        switch = sharing()
        switch._found = {"Turtle_1": "B"}
        switch._follow()
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": ["Ace2_1", "Turtle_1"]}
        assert lane_of(switch, "Turtle_1").extruder_obj.name == "extruder1"
        # A unit holds its own name for good, so what was found for it is never forgotten.
        assert (switch._found, switch._held) == ({"Turtle_1": "B"}, {"Turtle_1": "Turtle_1"})
        switch.afc.save_vars.assert_not_called()


class TestAfcBridgeBoxSwitchPlace:
    def test_a_unit_is_put_on_an_inlet_and_started_there(self):
        switch = finding()
        switch.confirmed = ["Bambu_AMS_1"]
        switch._saved_lanes = {"lane_Bambu_AMS_1": "extruder", "lane_Bambu_AMS_2": "extruder1"}
        switch._by_print = ["lane_Bambu_AMS_1", "lane_Bambu_AMS_2"]
        switch._due_back = ["lane_Bambu_AMS_2", "lane_Bambu_AMS_1"]
        switch._place("Bambu_AMS_1", "B")
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_1"]}
        # What was kept for its lane was for the inlet it had. The other unit's is left.
        assert switch._saved_lanes == {"lane_Bambu_AMS_2": "extruder1"}
        assert (switch._by_print, switch._due_back) == (["lane_Bambu_AMS_2"],
                                                        ["lane_Bambu_AMS_2"])
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder1"
        assert ams_of(switch, "Bambu_AMS_1").behind[-1] == ("Buffer_B", "chip_inlet_b")
        assert switch.confirmed == ["Bambu_AMS_1"]

    def test_a_unit_moves_from_one_inlet_to_the_other(self):
        switch = finding()
        found(switch, Bambu_AMS_1="B", Bambu_AMS_2="B")
        switch._place("Bambu_AMS_1", "A")
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": ["Bambu_AMS_2"]}
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder"
        assert ams_of(switch, "Bambu_AMS_1").behind[-1] == ("Buffer", "chip")

    def test_a_unit_whose_inlet_is_no_longer_known_is_taken_off_and_was_not_seen_at_one(self):
        switch = finding()
        found(switch, Bambu_AMS_1="B", Bambu_AMS_2="B")
        switch.confirmed = ["Bambu_AMS_1", "Bambu_AMS_2"]
        switch._place("Bambu_AMS_1", None)
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_2"]}
        assert switch.confirmed == ["Bambu_AMS_2"]
        assert ams_of(switch, "Bambu_AMS_1").behind[-1] == ("Buffer", "chip")
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder1"

    def test_a_unit_that_was_not_seen_at_its_inlet_is_only_taken_off(self):
        switch = finding()
        found(switch, Bambu_AMS_1="B")
        switch.confirmed = ["Bambu_AMS_2"]
        switch._place("Bambu_AMS_1", None)
        assert switch.inlets == {"A": [], "B": []}
        assert switch.confirmed == ["Bambu_AMS_2"]

    def test_a_unit_afc_does_not_have_yet_is_started_once_it_connects(self):
        switch = finding()
        unit = switch.afc.units.pop("Bambu_AMS_1")
        unit.behind.clear()
        switch._place("Bambu_AMS_1", "B")
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_1"]}
        assert unit.behind == []
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder"


class TestAfcBridgeBoxSwitchOffer:
    HOW = ("Put a spool in the unit and pick it. One lane is fed to the switch and reeled back, "
           "and a lane that is loaded through the switch is unloaded first.")

    @staticmethod
    def _button(unit: str) -> Tuple[str, str, str]:
        """:return tuple: the button that finds a unit's inlet"""
        return (unit, f"AFC_BRIDGEBOX_SWITCH_IDENTIFY SWITCH=Switch_1 UNIT={unit}", "primary")

    # Offered with more than one unit, to find them one after the other.
    ALL = ("All new units", "AFC_BRIDGEBOX_SWITCH_IDENTIFY SWITCH=Switch_1 UNIT=all", "default")

    def test_every_unit_whose_inlet_is_not_known_is_offered_and_all_of_them_at_once(self):
        switch = finding()
        switch._offer()
        assert chain_of(switch).offers == [
            ("Switch_1", "Switch_1 new unit",
             ["Switch_1: the inlet of Bambu_AMS_1, Bambu_AMS_2 is not known yet. The lanes of a "
              "unit do not load until its inlet is found.", self.HOW],
             [self._button("Bambu_AMS_1"), self._button("Bambu_AMS_2"), self.ALL])]
        assert switch._offered == ["AAAA", "BBBB"]

    def test_an_ams_is_offered_once(self):
        switch = finding()
        switch._offer()
        switch._offer()
        assert len(chain_of(switch).offers) == 1
        assert switch._offered == ["AAAA", "BBBB"]

    def test_an_ams_that_is_new_is_offered_with_the_ones_still_to_find(self):
        switch = finding()
        found(switch, Bambu_AMS_2="A")
        switch._offer()
        chain = chain_of(switch)
        # One unit is offered alone.
        assert chain.offers[0][3] == [self._button("Bambu_AMS_1")]
        chain.uids["Bambu_AMS_2"] = "CCCC"
        switch._follow()
        switch._offer()
        assert chain.offers[1] == (
            "Switch_1", "Switch_1 new unit",
            ["Switch_1: the inlet of Bambu_AMS_1, Bambu_AMS_2 is not known yet. The lanes of a "
             "unit do not load until its inlet is found.", self.HOW],
            [self._button("Bambu_AMS_1"), self._button("Bambu_AMS_2"), self.ALL])
        assert switch._offered == ["AAAA", "CCCC"]

    def test_with_every_inlet_known_nothing_is_offered(self):
        switch = finding()
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch._offer()
        assert chain_of(switch).offers == []
        assert switch._offered == []

    def test_a_bay_that_holds_no_unit_now_is_not_offered(self):
        switch = finding()
        ams_of(switch, "Bambu_AMS_1").pool = True
        switch._offer()
        assert chain_of(switch).offers[0][3] == [self._button("Bambu_AMS_2")]
        assert switch._offered == ["BBBB"]

    def test_a_switch_with_no_toolhead_on_an_outlet_offers_to_be_set_up_instead(self):
        switch = made()
        switch._offer()
        assert [offer[1] for offer in chain_of(switch).offers] == ["Chain_1 filament switch"]
        # No unit is offered by itself until the outlets have their toolheads.
        assert switch._offered == []
        assert switch._setup_offered is True

    def test_a_unit_on_no_chain_is_offered_once_by_its_name(self):
        switch = sharing()
        switch._offer()
        switch._offer()
        assert chain_of(switch).offers == [
            ("Switch_1", "Switch_1 new unit",
             ["Switch_1: the inlet of Turtle_1 is not known yet. The lanes of a unit do not "
              "load until its inlet is found.", self.HOW],
             [self._button("Turtle_1")])]
        assert switch._offered == ["Turtle_1"]


class TestAfcBridgeBoxSwitchOfferSetup:
    NOT_SET = ("Chain_1: the filament switch is not set up yet, and its lanes do not load until "
               "it is.")
    COMMAND = "AFC_BRIDGEBOX_SWITCH_SETUP SWITCH=Chain_1"

    def test_it_offers_to_find_the_inlets_and_the_toolheads(self):
        switch = made()
        switch._offer_setup()
        assert chain_of(switch).offers == [
            ("Chain_1", "Chain_1 filament switch",
             [self.NOT_SET,
              "Put a spool in each unit and unload the toolheads the switch feeds, then pick "
              "Set up. One lane of each unit is fed to the switch to find its inlet, then a "
              "lane is fed through each outlet until a toolhead sensor reads it. Each lane is "
              "reeled back."],
             [("Set up", self.COMMAND, "primary")])]
        assert switch._setup_offered is True

    def test_with_buffer_ramming_it_says_to_set_the_outlets_and_offers_the_inlets(self):
        switch = made()
        extruder_of(switch, "extruder").tool_start = "buffer"
        switch._offer_setup()
        assert chain_of(switch).offers == [
            ("Chain_1", "Chain_1 filament switch",
             [self.NOT_SET,
              "Chain_1: the toolhead each outlet feeds cannot be found by feeding a lane. With "
              "buffer ramming on extruder there is no toolhead sensor to read. Set "
              "switch_outlet_a and switch_outlet_b in [AFC_BridgeBox Chain_1] by hand, then "
              "RESTART.",
              "The inlets can be found now. Put a spool in each unit, then pick Find the "
              "inlets: one lane of each unit is fed to the switch and reeled back."],
             [("Find the inlets", self.COMMAND, "primary")])]
        assert switch._setup_offered is True

    def test_it_is_offered_once(self):
        switch = made()
        switch._offer_setup()
        switch._offer_setup()
        assert len(chain_of(switch).offers) == 1


class TestAfcBridgeBoxSwitchRelease:
    @staticmethod
    def _switch(printing: bool) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch with a lane a print moved and one due back"""
        switch = plumbed()
        switch._by_print = ["lane_Turtle_1"]
        switch._due_back = ["lane_Ace2_1"]
        switch._printing = printing
        return switch

    def test_the_lanes_of_a_print_that_ended_are_due_back_and_that_is_saved(self):
        switch = self._switch(True)
        switch._release(False)
        assert switch._by_print == []
        assert switch._due_back == ["lane_Ace2_1", "lane_Turtle_1"]
        assert switch._printing is False
        switch.afc.save_vars.assert_called_once_with()

    def test_a_print_that_still_runs_keeps_its_lanes(self):
        switch = self._switch(True)
        switch._release(True)
        assert switch._by_print == ["lane_Turtle_1"]
        assert switch._due_back == ["lane_Ace2_1"]
        assert switch._printing is True
        switch.afc.save_vars.assert_not_called()

    def test_with_no_print_seen_running_the_lanes_of_a_print_stay(self):
        # As after a restart during a print, which may be resumed.
        switch = self._switch(False)
        switch._release(False)
        assert switch._by_print == ["lane_Turtle_1"]
        assert switch._due_back == ["lane_Ace2_1"]
        assert switch._printing is False
        switch.afc.save_vars.assert_not_called()

    def test_a_print_that_starts_is_noted(self):
        switch = self._switch(False)
        switch._release(True)
        assert switch._by_print == ["lane_Turtle_1"]
        assert switch._due_back == ["lane_Ace2_1"]
        assert switch._printing is True
        switch.afc.save_vars.assert_not_called()

    def test_a_print_that_moved_no_lane_saves_nothing_when_it_ends(self):
        switch = self._switch(True)
        switch._by_print = []
        switch._release(False)
        assert switch._due_back == ["lane_Ace2_1"]
        assert switch._printing is False
        switch.afc.save_vars.assert_not_called()


class TestAfcBridgeBoxSwitchSendBack:
    @staticmethod
    def _switch() -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch whose three lanes are on their other extruder"""
        switch = plumbed()
        for unit, extruder in (("Turtle_1", "extruder1"), ("Bambu_AMS_1", "extruder1"),
                               ("Ace2_1", "extruder")):
            lane_of(switch, unit).set_extruder(extruder_of(switch, extruder))
        return switch

    @staticmethod
    def _on(switch: afcBridgeBoxSwitch) -> Dict[str, str]:
        """:return dict: unit -> the extruder its lane is on"""
        return {unit: lane_of(switch, unit).extruder_obj.name
                for unit in ("Turtle_1", "Bambu_AMS_1", "Ace2_1")}

    def test_a_lane_that_is_due_back_goes_to_its_units_starting_extruder(self):
        switch = self._switch()
        switch._due_back = ["lane_Turtle_1"]
        switch._send_back()
        assert lane_of(switch, "Turtle_1").extruder_obj is extruder_of(switch, "extruder")
        assert "lane_Turtle_1" in extruder_of(switch, "extruder").lanes
        assert switch._due_back == []
        assert switch.logger.messages == [
            ("info", "Switch_1: lane_Turtle_1 is back on extruder, the print that put it on "
                     "extruder1 is over")]
        switch.afc.save_vars.assert_called_once_with()

    def test_a_lane_of_a_bay_that_holds_no_unit_waits_for_its_unit(self):
        switch = self._switch()
        switch._due_back = ["lane_Bambu_AMS_1"]
        # Its saved record, loaded in extruder1 or not, is not on the lane until it is claimed.
        switch.afc.units["Bambu_AMS_1"].pool = True
        switch._send_back()
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj is extruder_of(switch, "extruder1")
        assert switch._due_back == ["lane_Bambu_AMS_1"]
        assert switch.logger.messages == []
        switch.afc.save_vars.assert_not_called()
        switch.afc.units["Bambu_AMS_1"].pool = False
        switch._send_back()
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj is extruder_of(switch, "extruder")
        assert switch._due_back == []

    def test_a_lane_that_is_not_due_back_stays(self):
        switch = self._switch()
        switch._due_back = ["lane_Turtle_1"]
        switch._send_back()
        assert self._on(switch) == {"Turtle_1": "extruder", "Bambu_AMS_1": "extruder1",
                                    "Ace2_1": "extruder"}

    def test_each_inlets_lane_goes_back_to_its_own_start_and_all_are_saved_once(self):
        switch = self._switch()
        switch._due_back = ["lane_Ace2_1", "lane_Turtle_1"]
        switch._send_back()
        assert self._on(switch) == {"Turtle_1": "extruder", "Bambu_AMS_1": "extruder1",
                                    "Ace2_1": "extruder1"}
        assert switch._due_back == []
        # In the order of the switch's lanes.
        assert switch.logger.messages == [
            ("info", "Switch_1: lane_Turtle_1 is back on extruder, the print that put it on "
                     "extruder1 is over"),
            ("info", "Switch_1: lane_Ace2_1 is back on extruder1, the print that put it on "
                     "extruder is over")]
        switch.afc.save_vars.assert_called_once_with()

    def test_a_lane_that_is_still_loaded_waits(self):
        switch = self._switch()
        switch._due_back = ["lane_Turtle_1", "lane_Ace2_1"]
        lane_of(switch, "Turtle_1").tool_loaded = True
        switch._send_back()
        assert self._on(switch) == {"Turtle_1": "extruder1", "Bambu_AMS_1": "extruder1",
                                    "Ace2_1": "extruder1"}
        assert switch._due_back == ["lane_Turtle_1"]
        assert switch.logger.messages == [
            ("info", "Switch_1: lane_Ace2_1 is back on extruder1, the print that put it on "
                     "extruder is over")]

    def test_a_lane_on_its_way_into_a_toolhead_waits(self):
        switch = self._switch()
        switch._due_back = ["lane_Turtle_1"]
        switch._routed["A"] = lane_of(switch, "Turtle_1")
        switch._send_back()
        assert lane_of(switch, "Turtle_1").extruder_obj.name == "extruder1"
        assert switch._due_back == ["lane_Turtle_1"]
        assert switch.logger.messages == []
        switch.afc.save_vars.assert_not_called()

    def test_a_lane_of_a_unit_whose_inlet_is_not_known_waits_for_the_inlet(self):
        switch = finding()
        lane = lane_of(switch, "Bambu_AMS_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._due_back = ["lane_Bambu_AMS_1"]
        switch._send_back()
        assert lane.extruder_obj.name == "extruder1"
        assert switch._due_back == ["lane_Bambu_AMS_1"]
        assert switch.logger.messages == []
        switch.afc.save_vars.assert_not_called()
        switch.inlets["A"].append("Bambu_AMS_1")
        switch._send_back()
        assert lane.extruder_obj.name == "extruder"
        assert switch._due_back == []

    def test_with_no_lane_due_back_nothing_is_saved(self):
        switch = self._switch()
        switch._send_back()
        assert self._on(switch) == {"Turtle_1": "extruder1", "Bambu_AMS_1": "extruder1",
                                    "Ace2_1": "extruder"}
        assert switch.logger.messages == []
        switch.afc.save_vars.assert_not_called()


class TestAfcBridgeBoxSwitchLook:
    def test_an_inlet_that_reads_filament_is_noted(self):
        switch = plumbed(SimChain(route=2, a=True))
        switch._look()
        assert switch._seen == {"A": True, "B": False}

    def test_each_inlet_is_noted_by_its_own_sensor(self):
        switch = plumbed(SimChain(route=2, b=True))
        switch._look()
        assert switch._seen == {"A": False, "B": True}

    def test_empty_inlets_are_not_noted(self):
        switch = plumbed(SimChain(route=2))
        switch._look()
        assert switch._seen == {"A": False, "B": False}

    def test_an_inlet_noted_before_stays_noted_once_it_reads_empty(self):
        chain = SimChain(route=2, a=True)
        switch = plumbed(chain)
        switch._look()
        chain.fill(a=False, b=True)
        switch._look()
        assert switch._seen == {"A": True, "B": True}

    def test_a_switch_that_is_not_answering_changes_nothing(self):
        switch = plumbed(FakeChain(state=None))
        switch._seen = {"A": True, "B": False}
        switch._look()
        assert switch._seen == {"A": True, "B": False}
        assert switch.logger.messages == []


class TestAfcBridgeBoxSwitchTell:
    def test_a_problem_is_warned_about_and_remembered(self):
        switch = make_switch()
        switch._tell(["inlet A is stuck"])
        assert switch.logger.messages == [("warning", "inlet A is stuck")]
        assert switch._told == "inlet A is stuck"

    def test_several_problems_are_said_together_one_per_line(self):
        switch = make_switch()
        switch._tell(["inlet A is stuck", "inlet B is stuck"])
        assert switch.logger.messages == [("warning", "inlet A is stuck\ninlet B is stuck")]
        assert switch._told == "inlet A is stuck\ninlet B is stuck"

    def test_a_problem_that_stays_is_said_once(self):
        switch = make_switch()
        switch._tell(["inlet A is stuck"])
        switch._tell(["inlet A is stuck"])
        assert switch.logger.messages == [("warning", "inlet A is stuck")]
        assert switch._told == "inlet A is stuck"

    def test_no_problem_says_nothing_and_forgets_what_was_said(self):
        switch = make_switch()
        switch._told = "inlet A is stuck"
        switch._tell([])
        assert switch.logger.messages == []
        assert switch._told == ""

    def test_a_problem_that_went_and_came_back_is_said_again(self):
        switch = make_switch()
        switch._tell(["inlet A is stuck"])
        switch._tell([])
        switch._tell(["inlet A is stuck"])
        assert switch.logger.messages == [("warning", "inlet A is stuck")] * 2

    def test_a_changed_problem_is_said(self):
        switch = make_switch()
        switch._tell(["inlet A is stuck"])
        switch._tell(["inlet A is stuck", "inlet B is stuck"])
        assert switch.logger.messages == [("warning", "inlet A is stuck"),
                                          ("warning", "inlet A is stuck\ninlet B is stuck")]
        assert switch._told == "inlet A is stuck\ninlet B is stuck"


class TestAfcBridgeBoxSwitchSettle:
    HELD_A = ("Switch_1: inlet A holds filament and none of its lanes is loaded, so it is left "
              "where it is. Run LANE_UNLOAD on the lane that is in it.")
    HELD_B = ("Switch_1: inlet B holds filament and none of its lanes is loaded, so it is left "
              "where it is. Run LANE_UNLOAD on the lane that is in it.")

    def test_a_switch_that_is_not_answering_is_the_one_problem(self):
        switch = plumbed(FakeChain(state=None), **LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        lane.tool_loaded = True
        assert switch._settle() == ["Switch_1: the filament switch is not answering"]
        # No tube is given a length either.
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {}
        assert switch.logger.messages == []

    def test_empty_inlets_go_to_rest(self):
        chain = SimChain(route=0)
        switch = plumbed(chain)
        assert switch._settle() == []
        assert chain.routes == [2]
        assert (chain.fs["route"], chain.fs["position"]) == (2, 2)
        assert switch.logger.messages == []

    def test_a_problem_is_returned_and_the_other_inlet_still_settles(self):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        switch.rest["B"] = "A"
        assert switch._settle() == [self.HELD_A]
        assert chain.routes == [1]
        assert chain.fs["position"] == 1
        # Saying it is left to the caller.
        assert switch.logger.messages == []
        assert switch._told == ""

    def test_a_refused_move_is_a_problem_too(self):
        # Inlet A's lane is loaded in outlet B, and its path sits at outlet A with filament.
        chain = SimChain(route=2, a=True)
        switch = plumbed(chain)
        lane_of(switch, "Turtle_1").set_extruder(extruder_of(switch, "extruder1"))
        lane_of(switch, "Turtle_1").tool_loaded = True
        assert switch._settle() == [
            "Switch_1: inlet A holds filament, so it cannot be moved to outlet B"]
        assert chain.routes == []

    def test_the_problems_of_both_inlets_come_back_inlet_a_first(self):
        switch = plumbed(SimChain(route=0, a=True, b=True))
        assert switch._settle() == [self.HELD_A, self.HELD_B]

    def test_the_tube_of_a_loaded_lane_gets_its_length(self):
        # Inlet A's lane is loaded in outlet B, where its path is.
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain, **LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        lane.tool_loaded = True
        assert switch._settle() == []
        assert chain.routes == []
        assert tube_of(hub) == (1060.0, 1040.0)

    def test_the_tubes_get_their_lengths_where_an_inlet_has_a_problem_too(self):
        # Inlet B's lane is loaded in outlet A, and inlet A holds filament of no lane.
        switch = plumbed(SimChain(route=1, a=True, b=True), **LENGTHS)
        lane = lane_of(switch, "Ace2_1")
        hub = add_hub(lane, 700.0, 690.0)
        lane.set_extruder(extruder_of(switch, "extruder"))
        lane.tool_loaded = True
        assert switch._settle() == [self.HELD_A]
        assert tube_of(hub) == (540.0, 530.0)

    def test_with_no_toolhead_on_an_outlet_no_inlet_is_pointed(self):
        # Inlet A is away from the outlet it rests on.
        chain = SimChain(route=0)
        switch = made(chain)
        assert switch._settle() == []
        assert chain.routes == []
        assert switch.logger.messages == []


class TestAfcBridgeBoxSwitchSettleLengths:
    @staticmethod
    def _switch() -> Tuple[afcBridgeBoxSwitch, FakeLane, FakeHub]:
        """:return tuple: a calibrated switch, Turtle_1's lane on the other extruder, its hub"""
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        return switch, lane, add_hub(lane)

    def test_the_tube_of_a_loaded_lane_gets_the_length_for_its_outlet(self):
        switch, lane, hub = self._switch()
        lane.tool_loaded = True
        switch._settle_lengths()
        assert tube_of(hub) == (1060.0, 1040.0)
        assert switch._offsets == {hub: 160.0}

    def test_so_does_the_tube_of_a_lane_that_is_on_its_way(self):
        switch, lane, hub = self._switch()
        switch._routed["A"] = lane
        switch._settle_lengths()
        assert tube_of(hub) == (1060.0, 1040.0)
        assert switch._offsets == {hub: 160.0}

    def test_a_lane_that_is_neither_leaves_its_tube_alone(self):
        switch, _lane, hub = self._switch()
        switch._settle_lengths()
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {}

    def test_a_tube_keeps_its_length_while_its_lane_stays(self):
        switch, lane, hub = self._switch()
        lane.tool_loaded = True
        switch._settle_lengths()
        switch._settle_lengths()
        assert tube_of(hub) == (1060.0, 1040.0)
        assert switch._offsets == {hub: 160.0}

    def test_a_tube_gets_its_own_length_back_once_its_lane_is_gone(self):
        switch, lane, hub = self._switch()
        lane.tool_loaded = True
        switch._settle_lengths()
        lane.tool_loaded = False
        switch._settle_lengths()
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {hub: 0.0}

    def test_only_the_tube_whose_lane_is_gone_goes_back(self):
        switch, lane, hub = self._switch()
        lane.tool_loaded = True
        crossing = lane_of(switch, "Ace2_1")
        crossing.set_extruder(extruder_of(switch, "extruder"))
        crossing.tool_loaded = True
        other_hub = add_hub(crossing, 700.0, 690.0)
        switch._settle_lengths()
        assert (tube_of(hub), tube_of(other_hub)) == ((1060.0, 1040.0), (540.0, 530.0))
        crossing.tool_loaded = False
        switch._settle_lengths()
        assert (tube_of(hub), tube_of(other_hub)) == ((1060.0, 1040.0), (700.0, 690.0))
        assert switch._offsets == {hub: 160.0, other_hub: 0.0}

    def test_a_loaded_lane_without_a_hub_changes_nothing(self):
        switch, lane, _hub = self._switch()
        lane.hub_obj = None
        lane.tool_loaded = True
        switch._settle_lengths()
        assert switch._offsets == {}

    @pytest.mark.parametrize("loaded", [True, False])
    def test_a_unit_that_is_still_coming_in_is_left_out(self, loaded):
        # Loaded or on its way: neither counts before the unit has its own lengths.
        switch, lane, hub = self._switch()
        lane.tool_loaded = loaded
        switch._routed["A"] = None if loaded else lane
        lane.unit_obj.arriving = True
        switch._settle_lengths()
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {}

    def test_it_gets_its_length_once_it_is_in(self):
        switch, lane, hub = self._switch()
        lane.tool_loaded = True
        lane.unit_obj.arriving = True
        switch._settle_lengths()
        lane.unit_obj.arriving = False
        switch._settle_lengths()
        assert tube_of(hub) == (1060.0, 1040.0)
        assert switch._offsets == {hub: 160.0}

    def test_a_unit_that_keeps_its_own_length_gets_it_there(self):
        switch, lane, hub = self._switch()
        unit = add_path_unit(switch, lane)
        lane.tool_loaded = True
        switch._settle_lengths()
        assert tube_of(unit) == (2160.0, 2150.0)
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {unit: 160.0}


class TestAfcBridgeBoxSwitchCalibrated:
    def test_with_a_length_for_each_outlet(self):
        assert make_switch(connect=False, **LENGTHS).calibrated() is True

    @pytest.mark.parametrize("values", [
        {}, dict(outlet_a_length=850), dict(outlet_b_length=1010)])
    def test_not_while_one_of_two_outlets_has_no_length(self, values):
        assert make_switch(connect=False, **values).calibrated() is False

    @pytest.mark.parametrize("values", [dict(outlet_a=None), dict(outlet_b=None)])
    def test_with_one_outlet_in_use_no_length_is_needed(self, values):
        switch = make_switch(connect=False, **values)
        assert switch.lengths == {"A": None, "B": None}
        assert switch.calibrated() is True


class TestAfcBridgeBoxSwitchTube:
    def test_the_hub_of_the_lane(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        assert switch._tube(lane) is hub

    def test_a_lane_without_a_hub_has_none(self):
        switch = plumbed()
        assert lane_of(switch, "Turtle_1").hub_obj is None
        assert switch._tube(lane_of(switch, "Turtle_1")) is None

    def test_a_hub_that_holds_no_tube_length_does_not_count(self):
        class BareHub:
            fullname = "AFC_hub Turtle_1"

        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.hub_obj = BareHub()
        assert switch._tube(lane) is None

    def test_a_unit_that_keeps_the_length_itself_is_the_tube(self):
        switch = plumbed()
        lane = lane_of(switch, "Bambu_AMS_1")
        unit = add_path_unit(switch, lane)
        assert switch._tube(lane) is unit

    def test_such_a_unit_comes_before_the_hub_of_the_lane(self):
        switch = plumbed()
        lane = lane_of(switch, "Bambu_AMS_1")
        add_hub(lane)
        unit = add_path_unit(switch, lane)
        assert switch._tube(lane) is unit


class TestAfcBridgeBoxSwitchByLoad:
    def test_a_unit_that_reports_the_path_of_its_last_load(self):
        switch = plumbed()
        lane = lane_of(switch, "Bambu_AMS_1")
        add_path_unit(switch, lane)
        assert switch._by_load(lane) is True

    def test_any_other_unit_measures_with_its_calibration(self):
        switch = plumbed()
        assert switch._by_load(lane_of(switch, "Turtle_1")) is False


class TestAfcBridgeBoxSwitchSetLength:
    def test_through_the_other_outlet_the_difference_of_the_tubes_is_added(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._set_length(lane)
        assert tube_of(hub) == (1060.0, 1040.0)
        assert switch._offsets == {hub: 160.0}

    def test_a_shorter_tube_through_the_other_outlet_takes_length_off(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Ace2_1")
        hub = add_hub(lane, 700.0, 690.0)
        lane.set_extruder(extruder_of(switch, "extruder"))
        switch._set_length(lane)
        assert tube_of(hub) == (540.0, 530.0)
        assert switch._offsets == {hub: -160.0}

    def test_through_its_own_outlet_a_tube_keeps_its_own_length(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        switch._set_length(lane)
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {hub: 0.0}

    def test_back_on_its_own_outlet_the_difference_comes_off_again(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._set_length(lane)
        lane.set_extruder(extruder_of(switch, "extruder"))
        switch._set_length(lane)
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {hub: 0.0}

    @pytest.mark.parametrize("values", [
        {}, dict(outlet_a_length=850), dict(outlet_b_length=1010)])
    def test_without_both_lengths_nothing_is_added_through_the_other_outlet(self, values):
        switch = plumbed(**values)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._set_length(lane)
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {hub: 0.0}

    def test_with_one_outlet_in_use_nothing_is_added_and_no_length_is_read(self):
        # The switch counts as calibrated, and its only outlet is every lane's own.
        switch = plumbed(outlet_b=None)
        lane = lane_of(switch, "Ace2_1")
        hub = add_hub(lane)
        assert lane.extruder_obj.name == "extruder"
        switch._set_length(lane)
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {hub: 0.0}

    def test_a_lane_that_is_being_measured_by_loading_it_gets_no_difference(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Bambu_AMS_1")
        unit = add_path_unit(switch, lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._measuring = lane
        switch._set_length(lane)
        assert tube_of(unit) == (2000.0, 1990.0)
        assert switch._offsets == {unit: 0.0}

    def test_another_lane_gets_the_difference_while_that_one_is_measured(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._measuring = lane_of(switch, "Bambu_AMS_1")
        switch._set_length(lane)
        assert tube_of(hub) == (1060.0, 1040.0)
        assert switch._offsets == {hub: 160.0}

    def test_a_lane_without_a_hub_is_left_alone(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._set_length(lane)
        assert switch._offsets == {}

    def test_a_lane_on_an_extruder_the_switch_does_not_feed_is_left_alone(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        lane.set_extruder(FakeExtruder("extruder2"))
        switch._set_length(lane)
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {}

    def test_a_lane_of_a_unit_whose_inlet_is_not_known_is_left_alone(self):
        switch = finding(**LENGTHS)
        lane = lane_of(switch, "Bambu_AMS_1")
        hub = add_hub(lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._set_length(lane)
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {}


class TestAfcBridgeBoxSwitchOffset:
    def test_both_lengths_of_the_tube_grow_by_the_offset(self):
        switch = make_switch()
        hub = FakeHub("Turtle_1")
        switch._offset(hub, 160.0)
        assert tube_of(hub) == (1060.0, 1040.0)
        assert switch._offsets == {hub: 160.0}

    def test_a_new_offset_takes_the_place_of_the_one_before(self):
        switch = make_switch()
        hub = FakeHub("Turtle_1")
        switch._offset(hub, 160.0)
        switch._offset(hub, -40.0)
        assert tube_of(hub) == (860.0, 840.0)
        assert switch._offsets == {hub: -40.0}

    def test_no_offset_gives_the_tube_its_own_lengths_back(self):
        switch = make_switch()
        hub = FakeHub("Turtle_1")
        switch._offset(hub, 160.0)
        switch._offset(hub, 0.0)
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {hub: 0.0}

    def test_each_tube_has_its_own_offset(self):
        switch = make_switch()
        hubs = [FakeHub("Turtle_1"), FakeHub("Ace2_1", 700.0, 690.0)]
        switch._offset(hubs[0], 160.0)
        switch._offset(hubs[1], -160.0)
        assert [tube_of(hub) for hub in hubs] == [(1060.0, 1040.0), (540.0, 530.0)]
        assert switch._offsets == {hubs[0]: 160.0, hubs[1]: -160.0}

    def test_a_length_that_was_changed_under_an_offset_keeps_the_change(self):
        switch = make_switch()
        hub = FakeHub("Turtle_1")
        switch._offset(hub, 160.0)
        hub.afc_bowden_length = 1100.0
        switch._offset(hub, 0.0)
        assert tube_of(hub) == (940.0, 880.0)


class TestAfcBridgeBoxSwitchHandlePrep:
    PREPPING = ("info", "Filament switch Switch_1 Prepping inlets")
    OUTLETS = ("Outlet A: extruder (lane_Bambu_AMS_1, lane_Turtle_1)\n"
               "Outlet B: extruder1 (lane_Ace2_1)\n"
               "Tube lengths not set: run AFC_CALIBRATION, or set outlet_a_length and "
               "outlet_b_length\n"
               "Not seen at their inlet yet: Bambu_AMS_1, Turtle_1, Ace2_1")

    def test_it_settles_the_switch_and_then_reports_it(self):
        chain = SimChain(route=0)
        switch = plumbed(chain)
        switch._told = "said before"
        switch.handle_prep(MockGCodeCommand())
        assert chain.routes == [2]
        assert switch.logger.messages == [
            self.PREPPING,
            ("raw", "Inlet A: Bambu_AMS_1, Turtle_1, at outlet A, empty\n"
                    "Inlet B: Ace2_1, at outlet B, empty\n" + self.OUTLETS)]
        assert switch._told == ""
        assert switch._moving is False

    def test_the_switch_is_held_while_it_settles(self):
        switch = plumbed()
        held = []

        def settle() -> List[str]:
            held.append(switch._moving)
            return []

        switch._settle = MagicMock(side_effect=settle)
        switch.handle_prep(MockGCodeCommand())
        assert held == [True]
        assert switch._moving is False

    def test_a_problem_is_listed_under_the_report_and_counts_as_said(self):
        switch = plumbed(SimChain(route=2, b=True))
        switch.handle_prep(MockGCodeCommand())
        held = TestAfcBridgeBoxSwitchSettle.HELD_B
        said = [
            self.PREPPING,
            ("raw", "Inlet A: Bambu_AMS_1, Turtle_1, at outlet A, empty\n"
                    "Inlet B: Ace2_1, at outlet B, holds filament\n" + self.OUTLETS),
            ("raw", "<span class=error--text>Switch_1: inlet B holds filament and none of "
                    "its lanes is loaded, so it is left where it is. Run LANE_UNLOAD on "
                    "the lane that is in it.</span>")]
        assert switch.logger.messages == said
        assert switch._told == held
        # So the watch does not say it again.
        switch._tell([held])
        assert switch.logger.messages == said

    def test_each_problem_gets_its_own_line(self):
        switch = plumbed(SimChain(route=2, a=True, b=True))
        switch.handle_prep(MockGCodeCommand())
        assert switch.logger.messages == [
            self.PREPPING,
            ("raw", "Inlet A: Bambu_AMS_1, Turtle_1, at outlet A, holds filament\n"
                    "Inlet B: Ace2_1, at outlet B, holds filament\n" + self.OUTLETS),
            ("raw", "<span class=error--text>Switch_1: inlet A holds filament and none of "
                    "its lanes is loaded, so it is left where it is. Run LANE_UNLOAD on "
                    "the lane that is in it.</span>"),
            ("raw", "<span class=error--text>Switch_1: inlet B holds filament and none of "
                    "its lanes is loaded, so it is left where it is. Run LANE_UNLOAD on "
                    "the lane that is in it.</span>")]
        assert switch._told == (f"{TestAfcBridgeBoxSwitchSettle.HELD_A}\n"
                                f"{TestAfcBridgeBoxSwitchSettle.HELD_B}")

    def test_a_switch_that_is_not_answering_has_no_report(self):
        switch = plumbed(FakeChain(state=None))
        switch.handle_prep(MockGCodeCommand())
        assert switch.logger.messages == [
            self.PREPPING,
            ("raw", "<span class=error--text>Switch_1: the filament switch is not "
                    "answering</span>")]
        assert switch._told == "Switch_1: the filament switch is not answering"
        assert switch._moving is False

    def test_the_section_is_printed_at_once_when_every_unit_is_in(self):
        switch = plumbed(SimChain(route=2))
        switch.handle_prep(MockGCodeCommand())
        assert switch._report_by is None
        assert switch._reported == ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]

    def test_the_section_waits_for_a_unit_that_comes_in_after_prep(self):
        chain = SimChain(route=0)
        chain.arriving = True
        switch = plumbed(chain)
        switch._told = "said before"
        switch.handle_prep(MockGCodeCommand())
        # The switch is settled all the same.
        assert chain.routes == [2]
        assert switch.logger.messages == []
        # 30 s from the reactor's 100.0.
        assert switch._report_by == 130.0
        assert switch._reported == []
        assert switch._told == "said before"
        assert switch._moving is False
        assert switch_mod._REPORT_WAIT == 30.0

    def test_a_section_that_waited_is_printed_by_the_watch_once_the_unit_is_in(self):
        chain = SimChain(route=2)
        chain.arriving = True
        switch = plumbed(chain)
        switch.afc.prep_done = True
        switch.handle_prep(MockGCodeCommand())
        switch._watch(0.0)
        assert switch.logger.messages == []
        chain.arriving = False
        switch._watch(0.0)
        assert switch.logger.messages == [
            self.PREPPING,
            ("raw", "Inlet A: Bambu_AMS_1, Turtle_1, at outlet A, empty\n"
                    "Inlet B: Ace2_1, at outlet B, empty\n" + self.OUTLETS)]
        assert switch._report_by is None
        # Printed once.
        switch._watch(0.0)
        assert len(switch.logger.messages) == 2


class TestAfcBridgeBoxSwitchArriving:
    def test_no_unit_on_its_way_is_none_arriving(self):
        switch = plumbed()
        assert not hasattr(switch.afc.units["Turtle_1"], "arriving")
        assert switch._arriving() is False

    def test_a_unit_the_chain_has_no_bay_for_yet_is_arriving(self):
        chain = SimChain()
        chain.arriving = True
        assert plumbed(chain)._arriving() is True

    @pytest.mark.parametrize("unit", ["Bambu_AMS_1", "Turtle_1", "Ace2_1"])
    def test_a_unit_of_the_switch_that_is_being_brought_in_is_arriving(self, unit):
        switch = plumbed()
        switch.afc.units[unit].arriving = True
        assert switch._arriving() is True
        switch.afc.units[unit].arriving = False
        assert switch._arriving() is False

    def test_a_unit_of_another_switch_is_not_waited_for(self):
        first, second, _lane = two_switches()
        first.afc.units["Other"].arriving = True
        assert first._arriving() is False
        assert second._arriving() is True

    def test_a_unit_that_afc_does_not_hold_is_not_arriving(self):
        printer = make_printer(connected=False)
        assert printer._afc.units == {}
        assert make_switch(printer)._arriving() is False


class TestAfcBridgeBoxSwitchNoteArrivals:
    ALL = ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]

    def test_no_new_unit_leaves_no_section_due(self):
        switch = plumbed()
        switch._reported = list(self.ALL)
        switch._note_arrivals()
        assert switch._report_by is None
        assert switch._reported == self.ALL

    def test_a_unit_that_came_in_makes_a_section_due(self):
        switch = plumbed()
        switch._reported = ["Turtle_1", "Ace2_1"]
        switch._note_arrivals()
        # 30 s from the reactor's 100.0.
        assert switch._report_by == 130.0
        # Until the section is printed.
        assert switch._reported == ["Turtle_1", "Ace2_1"]

    def test_a_section_that_is_due_already_keeps_its_time(self):
        switch = plumbed()
        switch._reported = ["Turtle_1", "Ace2_1"]
        switch._report_by = 110.0
        switch._note_arrivals()
        assert switch._report_by == 110.0

    def test_a_bay_that_holds_no_unit_is_not_new(self):
        switch = plumbed()
        switch.afc.units["Bambu_AMS_1"].pool = True
        switch._reported = ["Turtle_1", "Ace2_1"]
        switch._note_arrivals()
        assert switch._report_by is None

    def test_a_unit_that_left_is_new_when_it_is_back(self):
        switch = plumbed()
        switch._reported = list(self.ALL)
        switch.afc.units["Turtle_1"].pool = True
        switch._note_arrivals()
        assert switch._report_by is None
        assert switch._reported == ["Bambu_AMS_1", "Ace2_1"]
        switch.afc.units["Turtle_1"].pool = False
        switch._note_arrivals()
        assert switch._report_by == 130.0


class TestAfcBridgeBoxSwitchSay:
    @staticmethod
    def _switch(due: Optional[float], arriving: bool = False) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch whose section is due by a time, or not due"""
        chain = SimChain(route=2)
        chain.arriving = arriving
        switch = plumbed(chain)
        switch._report_by = due
        switch._tell = MagicMock()
        switch._section = MagicMock()
        return switch

    def test_with_no_section_due_a_problem_is_a_warning(self):
        switch = self._switch(None)
        switch._say(["a problem"])
        switch._tell.assert_called_once_with(["a problem"])
        switch._section.assert_not_called()

    def test_with_no_section_due_a_unit_on_its_way_changes_nothing(self):
        switch = self._switch(None, arriving=True)
        switch._say([])
        switch._tell.assert_called_once_with([])
        switch._section.assert_not_called()

    def test_a_section_that_is_due_is_printed_when_no_unit_is_on_its_way(self):
        switch = self._switch(130.0)
        switch._say(["a problem"])
        switch._section.assert_called_once_with(["a problem"])
        switch._tell.assert_not_called()

    def test_a_section_that_is_due_waits_for_a_unit_on_its_way(self):
        switch = self._switch(130.0, arriving=True)
        switch._say(["a problem"])
        switch._section.assert_not_called()
        switch._tell.assert_not_called()
        assert switch._report_by == 130.0

    @pytest.mark.parametrize("now", [130.0, 131.5])
    def test_the_wait_ends_at_the_time_the_section_is_due_by(self, now):
        switch = self._switch(130.0, arriving=True)
        switch.reactor._monotonic = now
        switch._say(["a problem"])
        switch._section.assert_called_once_with(["a problem"])
        switch._tell.assert_not_called()

    def test_just_before_that_time_it_still_waits(self):
        switch = self._switch(130.0, arriving=True)
        switch.reactor._monotonic = 129.9
        switch._say([])
        switch._section.assert_not_called()


class TestAfcBridgeBoxSwitchSection:
    PREPPING = ("info", "Filament switch Switch_1 Prepping inlets")

    def test_it_lists_the_switch_and_no_section_is_due_any_more(self):
        switch = plumbed(SimChain(route=2))
        switch.afc.units["Bambu_AMS_1"].pool = True
        switch._report_by = 130.0
        switch._told = "said before"
        switch._section([])
        assert switch.logger.messages == [
            self.PREPPING,
            ("raw", "Inlet A: Turtle_1, at outlet A, empty\n"
                    "Inlet B: Ace2_1, at outlet B, empty\n"
                    "Outlet A: extruder (lane_Turtle_1)\n"
                    "Outlet B: extruder1 (lane_Ace2_1)\n"
                    "Tube lengths not set: run AFC_CALIBRATION, or set outlet_a_length and "
                    "outlet_b_length\n"
                    "Not seen at their inlet yet: Turtle_1, Ace2_1")]
        assert switch._report_by is None
        assert switch._reported == ["Turtle_1", "Ace2_1"]
        assert switch._told == ""

    def test_each_problem_is_listed_under_it_and_counts_as_said(self):
        switch = plumbed(SimChain(route=2))
        switch._report = MagicMock(return_value="the report")
        switch._section(["inlet A is stuck", "inlet B is stuck"])
        assert switch.logger.messages == [
            self.PREPPING,
            ("raw", "the report"),
            ("raw", "<span class=error--text>inlet A is stuck</span>"),
            ("raw", "<span class=error--text>inlet B is stuck</span>")]
        assert switch._told == "inlet A is stuck\ninlet B is stuck"

    def test_a_switch_that_is_not_answering_has_only_its_problems(self):
        switch = plumbed(FakeChain(state=None))
        switch._report_by = 130.0
        switch._section(["Switch_1: the filament switch is not answering"])
        assert switch.logger.messages == [
            self.PREPPING,
            ("raw", "<span class=error--text>Switch_1: the filament switch is not "
                    "answering</span>")]
        assert switch._report_by is None
        assert switch._reported == ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]


class TestAfcBridgeBoxSwitchSettleInlet:
    def test_the_route_is_sent_again_under_a_loaded_lane(self):
        # A BridgeBox restart: the route fell to 0 and the loaded path stayed at outlet A.
        chain = SimChain(route=0, position=2, a=True)
        switch = plumbed(chain)
        lane_of(switch, "Turtle_1").tool_loaded = True
        switch.afc.function.in_print.return_value = True
        switch._settle_inlet("A", switch.state()["inlets"])
        assert chain.routes == [2]

    def test_a_loaded_lane_on_the_other_extruder_keeps_its_outlet(self):
        chain = SimChain(route=2, position=0, a=True)
        switch = plumbed(chain)
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        lane.tool_loaded = True
        switch._settle_inlet("A", switch.state()["inlets"])
        assert chain.routes == [0]

    def test_an_inlet_pointed_for_a_load_stays_there_until_the_lane_arrives(self):
        chain = SimChain(route=3)
        switch = plumbed(chain)
        switch._routed["B"] = lane_of(switch, "Ace2_1")
        lane_of(switch, "Ace2_1").set_extruder(extruder_of(switch, "extruder"))
        switch.afc.function.in_print.return_value = True
        switch._settle_inlet("B", switch.state()["inlets"])
        assert chain.routes == []
        switch._routed["B"] = None
        switch.afc.function.in_print.return_value = False
        switch._settle_inlet("B", switch.state()["inlets"])
        assert chain.routes == [2]

    def test_the_loaded_lane_decides_over_the_lane_last_pointed_for(self):
        chain = SimChain(route=0, position=2, a=True)
        switch = plumbed(chain)
        lane_of(switch, "Turtle_1").tool_loaded = True
        away = lane_of(switch, "Bambu_AMS_1")
        away.set_extruder(extruder_of(switch, "extruder1"))
        switch._routed["A"] = away
        switch._settle_inlet("A", switch.state()["inlets"])
        assert chain.routes == [2]

    def test_filament_no_lane_accounts_for_is_a_problem(self):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        with pytest.raises(SwitchError) as err:
            switch._settle_inlet("A", switch.state()["inlets"])
        assert str(err.value) == TestAfcBridgeBoxSwitchSettle.HELD_A
        assert chain.routes == []

    def test_an_empty_inlet_is_left_alone_during_a_print(self):
        chain = SimChain(route=0)
        switch = plumbed(chain)
        switch.afc.function.in_print.return_value = True
        switch._settle_inlet("A", switch.state()["inlets"])
        assert chain.routes == []

    def test_an_empty_inlet_waits_while_the_other_inlets_filament_is_in_its_outlet(self):
        # Inlet B's lane is loaded in outlet A, where inlet A rests.
        chain = SimChain(route=1, b=True)
        switch = plumbed(chain)
        switch._settle_inlet("A", switch.state()["inlets"])
        assert chain.routes == []

    @pytest.mark.parametrize("state, sent", [(dict(route=0, b=True), 2), (dict(route=1), 3)])
    def test_neither_filament_elsewhere_nor_an_empty_path_there_keeps_it_waiting(self, state,
                                                                                 sent):
        chain = SimChain(**state)
        switch = plumbed(chain)
        switch._settle_inlet("A", switch.state()["inlets"])
        assert chain.routes == [sent]
        assert chain.fs["position"] == sent

    def test_an_empty_inlet_goes_to_the_outlet_it_rests_on(self):
        chain = SimChain(route=2)
        switch = plumbed(chain)
        switch.rest["A"] = "B"
        switch._settle_inlet("A", switch.state()["inlets"])
        assert chain.routes == [0]
        assert chain.fs["position"] == 0


class TestAfcBridgeBoxSwitchHandleToolChangeUnload:
    @staticmethod
    def _switch() -> Tuple[afcBridgeBoxSwitch, FakeLane, FakeLane]:
        """
        :return tuple: a switch, the Bambu_AMS_1 lane loaded in `extruder1` and the Turtle_1
        lane of the same inlet, which is to load into `extruder`
        """
        switch = plumbed(SimChain(route=0, a=True))
        held = lane_of(switch, "Bambu_AMS_1")
        held.set_extruder(extruder_of(switch, "extruder1"))
        held.tool_loaded = True
        return switch, held, lane_of(switch, "Turtle_1")

    def test_a_loaded_lane_in_the_way_of_the_next_one_is_unloaded_first(self):
        switch, held, lane = self._switch()
        assert switch.handle_tool_change_unload(held, lane) is True
        # AFC unloads it, the switch is only asked.
        assert chain_of(switch).routes == []
        switch.afc.TOOL_UNLOAD.assert_not_called()

    def test_a_loaded_lane_that_is_not_in_the_way_stays_in_its_toolhead(self):
        switch = plumbed(SimChain(route=2, a=True))
        held = lane_of(switch, "Turtle_1")
        held.tool_loaded = True
        assert switch.handle_tool_change_unload(held, lane_of(switch, "Ace2_1")) is False

    def test_a_loaded_lane_of_no_switch_is_left_to_afc(self):
        switch, _, lane = self._switch()
        stranger = add_lane(switch.printer, "lane0", "AMS_9", loaded=True)
        assert switch.handle_tool_change_unload(stranger, lane) is False

    def test_a_next_lane_that_is_not_on_the_switch(self):
        switch, held, _ = self._switch()
        stranger = add_lane(switch.printer, "lane0", "AMS_9")
        assert switch.handle_tool_change_unload(held, stranger) is False

    def test_a_next_lane_on_an_extruder_the_switch_does_not_feed(self):
        switch, held, lane = self._switch()
        lane.set_extruder(FakeExtruder("extruder2"))
        assert switch.handle_tool_change_unload(held, lane) is False

    def test_a_next_lane_already_in_its_toolhead_needs_nothing_unloaded(self):
        switch, held, lane = self._switch()
        lane.tool_loaded = True
        assert switch.handle_tool_change_unload(held, lane) is False


class TestAfcBridgeBoxSwitchHandleToolLoadPrepare:
    def test_a_lane_that_is_not_on_the_switch_loads_as_ever(self):
        chain = SimChain(route=0)
        switch = plumbed(chain)
        stranger = add_lane(switch.printer, "lane0", "AMS_9")
        assert switch.handle_tool_load_prepare(stranger, MagicMock()) is True
        assert chain.routes == []
        assert switch._routed == {"A": None, "B": None}

    def test_a_lane_already_in_its_toolhead_needs_nothing(self):
        chain = SimChain(route=0, position=2, a=True)
        switch = plumbed(chain)
        lane = lane_of(switch, "Turtle_1")
        add_hub(lane)
        lane.tool_loaded = True
        assert switch.handle_tool_load_prepare(lane, MagicMock()) is True
        assert chain.routes == []
        assert switch._routed == {"A": None, "B": None}
        assert switch._offsets == {}

    @pytest.mark.parametrize("printing", [True, False])
    def test_a_lane_of_a_unit_whose_inlet_is_not_known_is_not_loaded(self, printing):
        chain = SimChain(route=0)
        switch = finding(chain)
        switch.afc.function.in_print.return_value = printing
        gcmd = MagicMock()
        assert switch.handle_tool_load_prepare(lane_of(switch, "Bambu_AMS_1"), gcmd) is False
        switch.afc.error.AFC_error.assert_called_once_with(
            "Switch_1: lane_Bambu_AMS_1 is not loaded, the inlet of Bambu_AMS_1 is not known. "
            "Run AFC_BRIDGEBOX_SWITCH_IDENTIFY UNIT=Bambu_AMS_1 first.",
            pause=printing, gcmd=gcmd)
        assert chain.routes == []
        assert switch._routed == {"A": None, "B": None}

    def test_but_one_that_is_in_its_toolhead_already_needs_nothing(self):
        chain = SimChain(route=0, position=2, a=True)
        switch = finding(chain)
        lane_of(switch, "Bambu_AMS_1").tool_loaded = True
        assert switch.handle_tool_load_prepare(lane_of(switch, "Bambu_AMS_1"), MagicMock()) is True
        switch.afc.error.AFC_error.assert_not_called()
        assert chain.routes == []
        assert switch._routed == {"A": None, "B": None}

    def test_it_points_the_lanes_inlet_and_remembers_the_lane(self):
        chain = SimChain(route=0)
        switch = plumbed(chain)
        lane = lane_of(switch, "Turtle_1")
        assert switch.handle_tool_load_prepare(lane, MagicMock()) is True
        assert chain.routes == [2]
        assert switch._routed == {"A": lane, "B": None}
        assert switch._moving is False
        switch.afc.error.AFC_error.assert_not_called()

    def test_the_lanes_tube_gets_the_length_for_its_outlet(self):
        chain = SimChain(route=2)
        switch = plumbed(chain, **LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        assert switch.handle_tool_load_prepare(lane, MagicMock()) is True
        assert chain.routes == [0]
        assert tube_of(hub) == (1060.0, 1040.0)
        assert switch._offsets == {hub: 160.0}

    def test_the_lane_is_remembered_before_it_waits_for_the_switch(self):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        switch._moving = True
        remembered = []

        def watch_ends(until: float) -> None:
            remembered.append(switch._routed["B"])
            switch._moving = False

        switch.reactor.pause.side_effect = watch_ends
        assert switch.handle_tool_load_prepare(lane, MagicMock()) is True
        assert remembered[0] is lane

    @pytest.mark.parametrize("printing", [True, False])
    def test_a_path_that_cannot_be_set_stops_the_load_with_an_afc_error(self, printing):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        switch.afc.function.in_print.return_value = printing
        add_hub(lane_of(switch, "Turtle_1"))
        gcmd = MagicMock()
        assert switch.handle_tool_load_prepare(lane_of(switch, "Turtle_1"), gcmd) is False
        switch.afc.error.AFC_error.assert_called_once_with(
            "Switch_1: inlet A holds filament, so it cannot be moved to outlet A",
            pause=printing, gcmd=gcmd)
        assert switch._routed == {"A": None, "B": None}
        assert switch._moving is False
        # No tube is given a length for a load that does not happen.
        assert switch._offsets == {}

    def test_a_lane_in_the_way_that_does_not_unload_stops_the_load(self):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        held = lane_of(switch, "Bambu_AMS_1")
        held.set_extruder(extruder_of(switch, "extruder1"))
        held.tool_loaded = True
        add_hub(lane_of(switch, "Turtle_1"))
        switch.afc.TOOL_UNLOAD = MagicMock(return_value=False)
        assert switch.handle_tool_load_prepare(lane_of(switch, "Turtle_1"), MagicMock()) is False
        # TOOL_UNLOAD said why itself.
        switch.afc.error.AFC_error.assert_not_called()
        assert switch._routed == {"A": None, "B": None}
        assert chain.routes == []
        assert switch._offsets == {}

    @pytest.mark.parametrize("printing", [True, False])
    def test_no_lane_is_loaded_while_no_outlet_has_its_toolhead(self, printing):
        chain = SimChain(route=0)
        switch = made(chain)
        # Also a lane of a unit whose inlet is known.
        found(switch, Bambu_AMS_1="A")
        switch.afc.function.in_print.return_value = printing
        gcmd = MagicMock()
        assert switch.handle_tool_load_prepare(lane_of(switch, "Bambu_AMS_1"), gcmd) is False
        switch.afc.error.AFC_error.assert_called_once_with(
            "Chain_1: lane_Bambu_AMS_1 is not loaded, the toolhead each outlet feeds is not "
            "known. Run AFC_BRIDGEBOX_SWITCH_SETUP SWITCH=Chain_1 first, or set switch_outlet_a "
            "and switch_outlet_b.", pause=printing, gcmd=gcmd)
        assert chain.routes == []
        assert switch._routed == {"A": None, "B": None}


class TestAfcBridgeBoxSwitchRoute:
    @staticmethod
    def _unloads(switch: afcBridgeBoxSwitch, works: bool = True) -> None:
        """Make AFC's TOOL_UNLOAD take a lane out of its toolhead and its inlet, or fail."""
        def tool_unload(lane: FakeLane, **_kwargs: Any) -> bool:
            if works:
                lane.tool_loaded = False
                inlet = switch.inlet_of(lane.unit)
                chain_of(switch).fill(a=False if inlet == "A" else None,
                                      b=False if inlet == "B" else None)
            return works
        switch.afc.TOOL_UNLOAD = MagicMock(side_effect=tool_unload)

    def test_with_nothing_in_the_way_the_inlet_is_pointed(self):
        chain = SimChain(route=2)
        switch = plumbed(chain)
        lane = lane_of(switch, "Ace2_1")
        lane.set_extruder(extruder_of(switch, "extruder"))
        assert switch._route(lane, "B", MagicMock()) is True
        assert chain.routes == [3]
        assert switch.logger.messages == []

    def test_a_lane_of_the_same_inlet_in_the_other_toolhead_is_unloaded_first(self):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        self._unloads(switch)
        held = lane_of(switch, "Bambu_AMS_1")
        held.set_extruder(extruder_of(switch, "extruder1"))
        held.tool_loaded = True
        gcmd = MagicMock()
        assert switch._route(lane_of(switch, "Turtle_1"), "A", gcmd) is True
        switch.afc.TOOL_UNLOAD.assert_called_once_with(held, set_start_time=False,
                                                       force_unload=True, gcmd=gcmd)
        assert chain.routes == [2]
        assert switch.logger.messages == [
            ("info", "Switch_1: unloading lane_Bambu_AMS_1 from extruder1, it is in the way of "
                     "lane_Turtle_1 to extruder")]

    def test_a_lane_of_the_other_inlet_in_the_same_toolhead_is_unloaded_first(self):
        chain = SimChain(route=2, a=True)
        switch = plumbed(chain)
        self._unloads(switch)
        held = lane_of(switch, "Turtle_1")
        held.tool_loaded = True
        lane = lane_of(switch, "Ace2_1")
        lane.set_extruder(extruder_of(switch, "extruder"))
        assert switch._route(lane, "B", MagicMock()) is True
        assert held.tool_loaded is False
        assert chain.routes == [3]
        assert switch.logger.messages == [
            ("info", "Switch_1: unloading lane_Turtle_1 from extruder, it is in the way of "
                     "lane_Ace2_1 to extruder")]

    def test_an_unload_that_fails_leaves_the_switch_alone(self):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        self._unloads(switch, works=False)
        held = lane_of(switch, "Bambu_AMS_1")
        held.set_extruder(extruder_of(switch, "extruder1"))
        held.tool_loaded = True
        assert switch._route(lane_of(switch, "Turtle_1"), "A", MagicMock()) is False
        assert chain.routes == []
        switch.reactor.pause.assert_not_called()

    @staticmethod
    def _clears_on_the_third_poll(switch: afcBridgeBoxSwitch) -> List[float]:
        """:return list: the reactor pauses, inlet A reading empty from the third one on"""
        polls: List[float] = []

        def pause(until: float) -> None:
            polls.append(until)
            if len(polls) == 3:
                chain_of(switch).fill(a=False)

        switch.reactor.pause.side_effect = pause
        return polls

    def test_the_sensor_is_given_time_to_clear_after_the_unload(self):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        held = lane_of(switch, "Bambu_AMS_1")
        held.set_extruder(extruder_of(switch, "extruder1"))
        held.tool_loaded = True

        def tool_unload(lane: FakeLane, **_kwargs: Any) -> bool:
            lane.tool_loaded = False
            return True

        switch.afc.TOOL_UNLOAD = MagicMock(side_effect=tool_unload)
        polls = self._clears_on_the_third_poll(switch)
        assert switch._route(lane_of(switch, "Turtle_1"), "A", MagicMock()) is True
        # Three waits for the sensor, then one for the route to read back.
        assert polls == [100.1] * 4
        assert chain.routes == [2]

    def test_the_sensor_is_given_time_to_clear_after_a_tool_change_unloaded_the_lane(self):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        polls = self._clears_on_the_third_poll(switch)
        assert switch._route(lane_of(switch, "Turtle_1"), "A", MagicMock()) is True
        switch.afc.TOOL_UNLOAD.assert_not_called()
        assert polls == [100.1] * 4
        assert chain.routes == [2]
        assert switch.logger.messages == []

    def test_an_inlet_with_a_lane_loaded_is_not_waited_for(self):
        chain = SimChain(route=2, a=True)
        switch = plumbed(chain)
        lane_of(switch, "Turtle_1").tool_loaded = True
        lane = lane_of(switch, "Ace2_1")
        assert switch._route(lane, "B", MagicMock()) is True
        switch.reactor.pause.assert_not_called()
        assert chain.routes == []

    def test_a_lane_on_an_extruder_the_switch_does_not_feed(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(FakeExtruder("extruder2"))
        with pytest.raises(SwitchError) as err:
            switch._route(lane, "A", MagicMock())
        assert str(err.value) == ("Switch_1: lane_Turtle_1 is on extruder2, which this switch "
                                  "does not feed")


class TestAfcBridgeBoxSwitchInTheWay:
    @staticmethod
    def _switch() -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch with its three lanes loaded nowhere yet"""
        return plumbed()

    def test_lanes_that_are_not_loaded_are_never_in_the_way(self):
        switch = self._switch()
        lane_of(switch, "Bambu_AMS_1").set_extruder(extruder_of(switch, "extruder1"))
        assert switch._in_the_way(lane_of(switch, "Turtle_1"), "A", "A") == []

    def test_same_inlet_other_outlet_is_in_the_way(self):
        switch = self._switch()
        held = lane_of(switch, "Bambu_AMS_1")
        held.set_extruder(extruder_of(switch, "extruder1"))
        held.tool_loaded = True
        assert switch._in_the_way(lane_of(switch, "Turtle_1"), "A", "A") == [held]

    def test_same_inlet_same_outlet_is_left_to_the_load(self):
        switch = self._switch()
        lane_of(switch, "Bambu_AMS_1").tool_loaded = True
        assert switch._in_the_way(lane_of(switch, "Turtle_1"), "A", "A") == []

    def test_other_inlet_same_outlet_is_in_the_way(self):
        switch = self._switch()
        held = lane_of(switch, "Ace2_1")
        held.set_extruder(extruder_of(switch, "extruder"))
        held.tool_loaded = True
        assert switch._in_the_way(lane_of(switch, "Turtle_1"), "A", "A") == [held]

    def test_other_inlet_other_outlet_is_not_in_the_way(self):
        switch = self._switch()
        lane_of(switch, "Ace2_1").tool_loaded = True
        assert switch._in_the_way(lane_of(switch, "Turtle_1"), "A", "A") == []

    def test_the_lane_is_never_in_its_own_way(self):
        switch = self._switch()
        lane = lane_of(switch, "Turtle_1")
        lane.tool_loaded = True
        assert switch._in_the_way(lane, "A", "B") == []


class TestAfcBridgeBoxSwitchUnload:
    @pytest.mark.parametrize("unloaded", [True, False])
    def test_it_swaps_to_the_lanes_toolhead_first(self, unloaded):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        switch.afc.function.get_current_extruder.return_value = "th_extruder"
        switch.afc.TOOL_UNLOAD = MagicMock(return_value=unloaded)
        gcmd = MagicMock()
        assert switch._unload(lane, gcmd) is unloaded
        lane.tool_swap.assert_called_once_with(gcmd=gcmd)
        switch.afc.TOOL_UNLOAD.assert_called_once_with(lane, set_start_time=False,
                                                       force_unload=True, gcmd=gcmd)

    def test_no_swap_when_its_toolhead_is_the_active_one(self):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        switch.afc.function.get_current_extruder.return_value = "th_extruder1"
        switch.afc.TOOL_UNLOAD = MagicMock(return_value=True)
        assert switch._unload(lane, MagicMock()) is True
        lane.tool_swap.assert_not_called()


class TestAfcBridgeBoxSwitchWaitClear:
    def test_an_empty_inlet_is_not_waited_for(self):
        switch = plumbed(SimChain(route=0, b=True))
        switch._wait_clear("A")
        switch.reactor.pause.assert_not_called()

    def test_it_waits_until_the_sensor_reads_empty(self):
        chain = SimChain(route=0, b=True)
        switch = plumbed(chain)
        polls = []

        def clears_on_the_second_poll(until: float) -> None:
            polls.append(until)
            if len(polls) == 2:
                chain.fill(b=False)

        switch.reactor.pause.side_effect = clears_on_the_second_poll
        switch._wait_clear("B")
        assert polls == [100.1] * 2

    def test_a_sensor_that_stays_on_is_given_up_on_quietly(self):
        switch = plumbed(SimChain(route=0, a=True))
        switch._wait_clear("A")
        assert switch.reactor.pause.call_count == switch_mod._CLEAR_POLLS == 30

    def test_a_switch_that_stops_answering(self):
        switch = plumbed(FakeChain(state=None))
        with pytest.raises(SwitchError) as err:
            switch._wait_clear("A")
        assert str(err.value) == "Switch_1: the filament switch is not answering"


class TestAfcBridgeBoxSwitchHandleToolLoaded:
    def test_a_lane_that_is_not_on_the_switch(self):
        switch = plumbed(FakeChain(state=fs(route=0, a=True)))
        switch._routed["A"] = lane_of(switch, "Turtle_1")
        switch.handle_tool_loaded(add_lane(switch.printer, "lane0", "AMS_9"))
        assert switch._routed["A"] is lane_of(switch, "Turtle_1")
        assert switch.confirmed == []
        assert switch.logger.messages == []

    def test_the_lane_that_was_pointed_for_has_arrived(self):
        switch = plumbed(SimChain(route=2, a=True))
        switch._routed = {"A": lane_of(switch, "Turtle_1"), "B": lane_of(switch, "Ace2_1")}
        switch.handle_tool_loaded(lane_of(switch, "Turtle_1"))
        assert switch._routed == {"A": None, "B": lane_of(switch, "Ace2_1")}
        assert switch.logger.messages == []

    def test_another_lane_of_the_inlet_arriving_leaves_the_remembered_one(self):
        switch = plumbed(SimChain(route=2, a=True))
        switch._routed["A"] = lane_of(switch, "Bambu_AMS_1")
        switch.handle_tool_loaded(lane_of(switch, "Turtle_1"))
        assert switch._routed["A"] is lane_of(switch, "Bambu_AMS_1")

    def test_a_lane_that_shows_at_its_inlet_confirms_its_unit(self):
        switch = plumbed(SimChain(route=2, a=True))
        switch.handle_tool_loaded(lane_of(switch, "Turtle_1"))
        assert switch.confirmed == ["Turtle_1"]
        switch.afc.save_vars.assert_called_once_with()
        assert switch.logger.messages == []

    def test_a_loaded_lane_whose_inlet_reads_empty_is_warned_about(self):
        switch = plumbed(SimChain(route=2))
        switch.handle_tool_loaded(lane_of(switch, "Turtle_1"))
        assert switch.logger.messages == [
            ("warning", "Switch_1: lane_Turtle_1 is loaded and inlet A reads empty. Check that "
                        "Turtle_1 is plumbed into inlet A.")]
        assert switch.confirmed == []
        switch.afc.save_vars.assert_not_called()

    def test_filament_no_lane_accounts_for_at_the_other_inlet_is_pointed_at(self):
        switch = plumbed(SimChain(route=2, b=True))
        switch.handle_tool_loaded(lane_of(switch, "Turtle_1"))
        assert switch.logger.messages == [
            ("warning", "Switch_1: lane_Turtle_1 is loaded and inlet A reads empty. Inlet B "
                        "reads filament, so Turtle_1 looks plumbed into it.")]
        assert switch.confirmed == []
        switch.afc.save_vars.assert_not_called()

    def test_filament_of_a_lane_loaded_through_the_other_inlet_is_not_pointed_at(self):
        switch = plumbed(SimChain(route=2, b=True))
        lane_of(switch, "Ace2_1").tool_loaded = True
        switch.handle_tool_loaded(lane_of(switch, "Turtle_1"))
        assert switch.logger.messages == [
            ("warning", "Switch_1: lane_Turtle_1 is loaded and inlet A reads empty. Check that "
                        "Turtle_1 is plumbed into inlet A.")]

    def test_a_lane_of_inlet_b_is_checked_at_inlet_b(self):
        switch = plumbed(SimChain(route=2, a=True))
        switch.handle_tool_loaded(lane_of(switch, "Ace2_1"))
        assert switch.logger.messages == [
            ("warning", "Switch_1: lane_Ace2_1 is loaded and inlet B reads empty. Inlet A "
                        "reads filament, so Ace2_1 looks plumbed into it.")]
        assert switch.confirmed == []

    def test_nothing_is_said_while_the_switch_is_not_answering(self):
        switch = plumbed(FakeChain(state=None))
        switch._routed["A"] = lane_of(switch, "Turtle_1")
        switch.handle_tool_loaded(lane_of(switch, "Turtle_1"))
        assert switch._routed["A"] is None
        assert switch.confirmed == []
        assert switch.logger.messages == []


class TestAfcBridgeBoxSwitchConfirm:
    def test_a_unit_is_noted_and_saved_the_first_time(self):
        switch = plumbed()
        switch._confirm("Turtle_1")
        assert switch.confirmed == ["Turtle_1"]
        switch.afc.save_vars.assert_called_once_with()

    def test_a_unit_that_is_noted_already_is_not_saved_again(self):
        switch = plumbed()
        switch.confirmed = ["Turtle_1"]
        switch._confirm("Turtle_1")
        assert switch.confirmed == ["Turtle_1"]
        switch.afc.save_vars.assert_not_called()

    def test_units_are_noted_in_the_order_they_showed(self):
        switch = plumbed()
        switch._confirm("Ace2_1")
        switch._confirm("Turtle_1")
        assert switch.confirmed == ["Ace2_1", "Turtle_1"]
        assert switch.afc.save_vars.call_args_list == [call(), call()]


class TestAfcBridgeBoxSwitchReport:
    NOT_SET = ("Tube lengths not set: run AFC_CALIBRATION, or set outlet_a_length and "
               "outlet_b_length")

    def test_the_inlets_then_the_outlets_then_what_is_still_to_do(self):
        switch = plumbed(SimChain(route=2, a=True))
        assert switch._report() == (
            "Inlet A: Bambu_AMS_1, Turtle_1, at outlet A, holds filament\n"
            "Inlet B: Ace2_1, at outlet B, empty\n"
            "Outlet A: extruder (lane_Bambu_AMS_1, lane_Turtle_1)\n"
            "Outlet B: extruder1 (lane_Ace2_1)\n"
            f"{self.NOT_SET}\n"
            "Not seen at their inlet yet: Bambu_AMS_1, Turtle_1, Ace2_1")

    def test_a_loaded_lane_is_named_and_nothing_is_left_to_do(self):
        switch = plumbed(SimChain(route=0, a=True), **LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        lane.tool_loaded = True
        switch.confirmed = ["Ace2_1", "Bambu_AMS_1", "Turtle_1"]
        assert switch._report() == (
            "Inlet A: Bambu_AMS_1, Turtle_1, at outlet B, lane_Turtle_1 loaded\n"
            "Inlet B: Ace2_1, at outlet B, empty\n"
            "Outlet A: extruder (lane_Bambu_AMS_1)\n"
            "Outlet B: extruder1 (lane_Ace2_1, lane_Turtle_1)")

    def test_an_outlets_lanes_are_listed_in_number_order(self):
        printer = make_printer([SimChain(route=2)])
        for name, unit in (("lane12", "Bambu_AMS_1"), ("lane13", "Bambu_AMS_1"),
                           ("lane8", "Turtle_1"), ("lane9", "Turtle_1")):
            add_lane(printer, name, unit)
        switch = make_switch(printer, **LENGTHS)
        switch.confirmed = ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]
        assert [lane.name for lane in switch.lanes] == ["lane12", "lane13", "lane8", "lane9"]
        assert switch._report() == (
            "Inlet A: Bambu_AMS_1, Turtle_1, at outlet A, empty\n"
            "Inlet B: Ace2_1, at outlet B, empty\n"
            "Outlet A: extruder (lane8, lane9, lane12, lane13)\n"
            "Outlet B: extruder1 (no lane)")

    def test_only_the_units_that_are_still_to_be_seen_are_listed(self):
        switch = plumbed(SimChain(route=2), **LENGTHS)
        switch.confirmed = ["Turtle_1"]
        assert switch._report() == (
            "Inlet A: Bambu_AMS_1, Turtle_1, at outlet A, empty\n"
            "Inlet B: Ace2_1, at outlet B, empty\n"
            "Outlet A: extruder (lane_Bambu_AMS_1, lane_Turtle_1)\n"
            "Outlet B: extruder1 (lane_Ace2_1)\n"
            "Not seen at their inlet yet: Bambu_AMS_1, Ace2_1")

    def test_an_inlet_without_units_and_an_outlet_without_an_extruder_or_a_lane(self):
        printer = make_printer([FakeChain(state=fs(route=0))])
        switch = make_switch(printer, inlet_b=None, outlet_b=None)
        # With one outlet in use there is no tube length to set.
        assert switch._report() == (
            "Inlet A: Bambu_AMS_1, Turtle_1, at outlet B, empty\n"
            "Inlet B: no unit, at outlet B, empty\n"
            "Outlet A: extruder (no lane)\n"
            "Outlet B: no extruder (no lane)\n"
            "Not seen at their inlet yet: Bambu_AMS_1, Turtle_1")

    def test_bays_that_hold_no_unit_are_left_out_with_their_lanes(self):
        switch = plumbed(SimChain(route=2))
        switch.afc.units["Bambu_AMS_1"].pool = True
        switch.afc.units["Ace2_1"].pool = True
        assert switch._report() == (
            "Inlet A: Turtle_1, at outlet A, empty\n"
            "Inlet B: no unit, at outlet B, empty\n"
            "Outlet A: extruder (lane_Turtle_1)\n"
            "Outlet B: extruder1 (no lane)\n"
            f"{self.NOT_SET}\n"
            "Not seen at their inlet yet: Turtle_1")

    def test_the_units_whose_inlet_is_not_known_are_listed_last_and_nowhere_else(self):
        switch = finding(SimChain(route=2), **LENGTHS)
        found(switch, Bambu_AMS_2="B")
        assert switch._report() == (
            "Inlet A: no unit, at outlet A, empty\n"
            "Inlet B: Bambu_AMS_2, at outlet B, empty\n"
            "Outlet A: extruder (no lane)\n"
            "Outlet B: extruder1 (lane_Bambu_AMS_2)\n"
            "Not seen at their inlet yet: Bambu_AMS_2\n"
            "Inlet not known: Bambu_AMS_1. Run AFC_BRIDGEBOX_SWITCH_IDENTIFY, or pick the "
            "switch and its inlets in AFC_CALIBRATION")

    def test_a_switch_that_is_not_answering(self):
        switch = plumbed(FakeChain(state=None))
        with pytest.raises(SwitchError) as err:
            switch._report()
        assert str(err.value) == "Switch_1: the filament switch is not answering"

    def test_a_switch_with_no_toolhead_on_an_outlet_says_how_its_outlets_are_set(self):
        switch = made(SimChain(route=2))
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch.confirmed = list(BAYS)
        assert switch._report() == (
            "Inlet A: Bambu_AMS_1, at outlet A, empty\n"
            "Inlet B: Bambu_AMS_2, at outlet B, empty\n"
            "Outlet A: no extruder (no lane)\n"
            "Outlet B: no extruder (no lane)\n"
            "Outlets not set up: run AFC_BRIDGEBOX_SWITCH_SETUP, or set switch_outlet_a and "
            "switch_outlet_b")

    def test_a_switch_its_chain_made_names_the_tube_lengths_as_its_chain_writes_them(self):
        switch = made(SimChain(route=2), outlet_a="extruder", outlet_b="extruder1")
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch.confirmed = list(BAYS)
        assert switch._report() == (
            "Inlet A: Bambu_AMS_1, at outlet A, empty\n"
            "Inlet B: Bambu_AMS_2, at outlet B, empty\n"
            "Outlet A: extruder (lane_Bambu_AMS_1)\n"
            "Outlet B: extruder1 (lane_Bambu_AMS_2)\n"
            "Tube lengths not set: run AFC_CALIBRATION, or set switch_outlet_a_length and "
            "switch_outlet_b_length")


class TestAfcBridgeBoxSwitchCmdAFCBridgeboxSwitchRoute:
    @staticmethod
    def _switch(state: Dict[str, Any], then: Optional[List[Any]] = None,
                **values: Any) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch whose chain reports ``state``"""
        return make_switch(make_printer([FakeChain(state=state, then=then)]), **values)

    def test_without_letters_it_reports(self):
        switch = self._switch(fs(route=0))
        gcmd = MockGCodeCommand()
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        gcmd.respond_info.assert_called_once_with(
            "Switch_1 on Chain_1\n"
            "Inlet A: Bambu_AMS_1, Turtle_1, at outlet B, empty\n"
            "Inlet B: Ace2_1, at outlet B, empty\n"
            "Outlet A: extruder (no lane)\n"
            "Outlet B: extruder1 (no lane)\n"
            "Tube lengths not set: run AFC_CALIBRATION, or set outlet_a_length and "
            "outlet_b_length\n"
            "Not seen at their inlet yet: Bambu_AMS_1, Turtle_1, Ace2_1")
        assert chain_of(switch).routes == []

    def test_it_moves_an_inlet(self):
        switch = self._switch(fs(route=0), then=[fs(route=2)])
        gcmd = MockGCodeCommand({"INLET": "a", "OUTLET": "a"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: inlet A now exits outlet A (extruder)")
        assert chain_of(switch).routes == [2]
        assert switch._moving is False

    def test_the_inlet_then_rests_on_that_outlet(self):
        switch = self._switch(fs(route=2), then=[fs(route=0)])
        switch._routed = {"A": MagicMock(), "B": None}
        was = switch._routed["A"]
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(MockGCodeCommand({"INLET": "A", "OUTLET": "B"}))
        assert switch.rest == {"A": "B", "B": "B"}
        assert switch._routed == {"A": None, "B": None}
        assert was is not None

    def test_an_inlet_already_there(self):
        switch = self._switch(fs(route=0), outlet_b=None)
        gcmd = MockGCodeCommand({"INLET": "B", "OUTLET": "B"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: inlet B already exits outlet B (no extruder)")
        assert chain_of(switch).routes == []
        assert switch.rest["B"] == "B"

    @pytest.mark.parametrize("params", [
        {"INLET": "A"}, {"OUTLET": "A"}, {"INLET": "C", "OUTLET": "A"},
        {"INLET": "A", "OUTLET": "1"}])
    def test_letters_other_than_a_and_b_are_refused(self, params):
        switch = self._switch(fs(route=0))
        gcmd = MockGCodeCommand(params)
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        assert str(err.value) == "Switch_1: INLET and OUTLET each take A or B"
        gcmd.respond_info.assert_not_called()
        assert chain_of(switch).routes == []

    def test_a_refused_move_is_the_commands_error(self):
        switch = self._switch(fs(route=0, a=True))
        gcmd = MockGCodeCommand({"INLET": "A", "OUTLET": "A"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        assert str(err.value) == ("Switch_1: inlet A holds filament, so it cannot be moved to "
                                  "outlet A")
        gcmd.respond_info.assert_not_called()
        assert switch.rest == {"A": "A", "B": "B"}
        assert switch._moving is False

    def test_a_refused_move_keeps_the_resting_outlet(self):
        switch = self._switch(fs(route=2, a=True))
        was = MagicMock()
        switch._routed["A"] = was
        with pytest.raises(CommandError):
            switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(MockGCodeCommand({"INLET": "A", "OUTLET": "B"}))
        assert switch.rest["A"] == "A"
        assert switch._routed["A"] is was

    def test_a_report_from_a_switch_that_is_not_answering(self):
        switch = self._switch(fs(on=False))
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        assert str(err.value) == "Switch_1: the filament switch is not answering"


class TestAfcBridgeBoxSwitchCmdAFCBridgeboxSwitchAssign:
    def test_it_needs_a_lane(self):
        switch = plumbed()
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(MockGCodeCommand())
        assert str(err.value) == "Error on '': missing LANE"

    def test_a_t_number_is_assigned_by_the_lane_mapped_to_it(self):
        switch = plumbed()
        switch._assign_tool = MagicMock()
        switch.assign = MagicMock()
        gcmd = MockGCodeCommand({"TOOL": "T3", "EXTRUDER": "extruder1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(gcmd)
        switch._assign_tool.assert_called_once_with("T3", "extruder1", False, gcmd)
        switch.assign.assert_not_called()

    def test_a_t_number_passes_the_unload_on_and_goes_before_a_lane(self):
        switch = plumbed()
        switch._assign_tool = MagicMock()
        switch.assign = MagicMock()
        gcmd = MockGCodeCommand({"TOOL": "3", "LANE": "lane_Ace2_1", "EXTRUDER": "extruder",
                                 "UNLOAD": "1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(gcmd)
        switch._assign_tool.assert_called_once_with("3", "extruder", True, gcmd)
        switch.assign.assert_not_called()

    def test_a_t_number_needs_an_extruder(self):
        switch = plumbed()
        switch._assign_tool = MagicMock()
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(MockGCodeCommand({"TOOL": "T3"}))
        assert str(err.value) == "Error on '': missing EXTRUDER"
        switch._assign_tool.assert_not_called()

    def test_a_lane_that_is_on_no_switch(self):
        switch = plumbed()
        add_lane(switch.printer, "lane0", "AMS_9")
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(MockGCodeCommand({"LANE": "lane0"}))
        assert str(err.value) == ("AFC_BRIDGEBOX_SWITCH_ASSIGN: lane0 is not a lane of a "
                                  "filament switch")

    def test_the_lane_goes_to_its_starting_extruder_and_is_not_unloaded_unless_asked(self):
        switch = plumbed()
        switch.assign = MagicMock()
        gcmd = MockGCodeCommand({"LANE": "lane_Ace2_1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(gcmd)
        switch.assign.assert_called_once_with(lane_of(switch, "Ace2_1"), None, False, gcmd)

    def test_the_extruder_and_the_unload_are_passed_on(self):
        switch = plumbed()
        switch.assign = MagicMock()
        gcmd = MockGCodeCommand({"LANE": "lane_Ace2_1", "EXTRUDER": "extruder", "UNLOAD": "1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(gcmd)
        switch.assign.assert_called_once_with(lane_of(switch, "Ace2_1"), "extruder", True, gcmd)

    def test_a_lane_of_another_switch_is_assigned_by_that_switch(self):
        first, second, lane = two_switches()
        first.assign = MagicMock()
        second.assign = MagicMock()
        gcmd = MockGCodeCommand({"LANE": "lane20"})
        first.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(gcmd)
        first.assign.assert_not_called()
        second.assign.assert_called_once_with(lane, None, False, gcmd)


class TestAfcBridgeBoxSwitchLane:
    def test_a_lane_of_the_switch_by_its_name(self):
        switch = plumbed()
        assert switch.lane("lane_Ace2_1") is lane_of(switch, "Ace2_1")

    def test_a_name_no_lane_has(self):
        assert plumbed().lane("lane99") is None

    def test_a_lane_of_a_unit_that_is_not_on_the_switch(self):
        printer = make_printer()
        add_lane(printer, "lane0", "AMS_9")
        mine = add_lane(printer, "lane1", "Turtle_1")
        switch = make_switch(printer)
        assert switch.lane("lane0") is None
        assert switch.lane("lane1") is mine


class TestAfcBridgeBoxSwitchSwitchOf:
    def test_a_lane_of_this_switch(self):
        first, _second, _lane = two_switches()
        assert first._switch_of("lane_Turtle_1") is first

    def test_a_lane_of_another_switch(self):
        first, second, _lane = two_switches()
        assert first._switch_of("lane20") is second
        assert second._switch_of("lane_Ace2_1") is first

    def test_a_lane_that_goes_through_no_switch(self):
        first, _second, _lane = two_switches()
        add_lane(first.printer, "lane0", "AMS_9")
        assert first._switch_of("lane0") is None


class TestAfcBridgeBoxSwitchAssignTool:
    @staticmethod
    def _mapped(switch: afcBridgeBoxSwitch, lane: Optional[FakeLane]) -> None:
        """Make AFC map every T number to ``lane``, None for to no lane."""
        switch.afc.function.get_lane_by_map = MagicMock(return_value=lane)

    @pytest.mark.parametrize("tool", ["T3", "t3", "3"])
    def test_a_t_number_no_lane_is_mapped_to_is_warned_about(self, tool):
        switch = plumbed()
        self._mapped(switch, None)
        gcmd = MockGCodeCommand()
        switch._assign_tool(tool, "extruder1", False, gcmd)
        switch.afc.function.get_lane_by_map.assert_called_once_with("T3")
        assert switch.logger.messages == [
            ("warning", "AFC_BRIDGEBOX_SWITCH_ASSIGN: no lane is mapped to T3")]
        gcmd.respond_info.assert_not_called()

    def test_the_lane_goes_to_the_extruder_its_switch_feeds(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        self._mapped(switch, lane)
        gcmd = MockGCodeCommand()
        switch._assign_tool("T0", "extruder1", False, gcmd)
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        switch.afc.save_vars.assert_called_once_with()
        gcmd.respond_info.assert_called_once_with("Switch_1: lane_Turtle_1 now feeds extruder1")
        assert switch.logger.messages == []

    @pytest.mark.parametrize("unload", [True, False])
    def test_the_unload_and_the_command_are_passed_on(self, unload):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        self._mapped(switch, lane)
        switch.assign = MagicMock()
        gcmd = MockGCodeCommand()
        switch._assign_tool("T2", "extruder", unload, gcmd)
        switch.assign.assert_called_once_with(lane, "extruder", unload, gcmd)
        assert switch.logger.messages == []

    def test_a_lane_of_another_switch_is_assigned_by_that_switch(self):
        first, second, lane = two_switches()
        first.assign = MagicMock()
        second.assign = MagicMock()
        self._mapped(first, lane)
        gcmd = MockGCodeCommand()
        first._assign_tool("T4", "extruder2", False, gcmd)
        first.assign.assert_not_called()
        second.assign.assert_called_once_with(lane, "extruder2", False, gcmd)
        assert first.logger.messages == []

    def test_an_extruder_the_lanes_own_switch_does_not_feed_is_warned_about(self):
        # Switch_1 feeds extruder1, and lane20 goes through Switch_2.
        first, second, lane = two_switches()
        first.assign = MagicMock()
        second.assign = MagicMock()
        self._mapped(first, lane)
        first._assign_tool("T4", "extruder1", False, MockGCodeCommand())
        first.assign.assert_not_called()
        second.assign.assert_not_called()
        assert lane.extruder_obj.name == "extruder2"
        assert first.logger.messages == [
            ("warning", "AFC_BRIDGEBOX_SWITCH_ASSIGN: T4 is lane20, which cannot reach "
                        "extruder1. It prints from extruder2.")]

    def test_a_lane_on_no_switch_cannot_reach_another_extruder(self):
        switch = plumbed()
        stranger = add_lane(switch.printer, "lane0", "AMS_9")
        switch.assign = MagicMock()
        self._mapped(switch, stranger)
        switch._assign_tool("7", "extruder1", False, MockGCodeCommand())
        switch.assign.assert_not_called()
        assert stranger.extruder_obj.name == "extruder"
        assert switch.logger.messages == [
            ("warning", "AFC_BRIDGEBOX_SWITCH_ASSIGN: T7 is lane0, which cannot reach "
                        "extruder1. It prints from extruder.")]

    def test_a_lane_on_no_switch_that_is_on_the_extruder_already_needs_nothing(self):
        switch = plumbed()
        stranger = add_lane(switch.printer, "lane0", "AMS_9")
        switch.assign = MagicMock()
        self._mapped(switch, stranger)
        gcmd = MockGCodeCommand()
        switch._assign_tool("T7", "extruder", False, gcmd)
        switch.assign.assert_not_called()
        assert switch.logger.messages == []
        gcmd.respond_info.assert_not_called()


class TestAfcBridgeBoxSwitchAssign:
    def test_a_lane_of_a_unit_whose_inlet_is_not_known_stays_where_it_is(self):
        switch = finding()
        lane = lane_of(switch, "Bambu_AMS_1")
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch.assign(lane, "extruder1", False, gcmd)
        assert str(err.value) == (
            "Switch_1: lane_Bambu_AMS_1 stays where it is, the inlet of Bambu_AMS_1 is not "
            "known. Run AFC_BRIDGEBOX_SWITCH_IDENTIFY UNIT=Bambu_AMS_1 first.")
        assert lane.extruder_obj.name == "extruder"
        switch.afc.save_vars.assert_not_called()
        gcmd.respond_info.assert_not_called()

    def test_an_extruder_the_switch_does_not_feed(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        with pytest.raises(CommandError) as err:
            switch.assign(lane, "extruder2", False, MockGCodeCommand())
        assert str(err.value) == "Switch_1: EXTRUDER takes extruder or extruder1"
        assert lane.extruder_obj.name == "extruder"
        switch.afc.save_vars.assert_not_called()

    def test_a_lane_that_is_there_already(self):
        switch = plumbed()
        gcmd = MockGCodeCommand()
        switch.assign(lane_of(switch, "Turtle_1"), "extruder", False, gcmd)
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: lane_Turtle_1 already feeds extruder")

    def test_a_lane_a_print_put_there_is_kept_there_by_naming_it_again(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._due_back = ["lane_Turtle_1", "lane_Ace2_1"]
        gcmd = MockGCodeCommand()
        switch.assign(lane, "extruder1", False, gcmd)
        assert switch._due_back == ["lane_Ace2_1"]
        switch.afc.save_vars.assert_called_once_with()
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: lane_Turtle_1 already feeds extruder1")

    def test_a_print_that_names_a_lane_where_it_is_leaves_it_due_back(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._due_back = ["lane_Turtle_1"]
        switch.afc.function.in_print.return_value = True
        gcmd = MockGCodeCommand()
        switch.assign(lane, "extruder1", False, gcmd)
        assert (switch._by_print, switch._due_back) == ([], ["lane_Turtle_1"])
        switch.afc.save_vars.assert_not_called()
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: lane_Turtle_1 already feeds extruder1")

    def test_a_lane_moves_to_the_other_extruder_and_the_choice_is_saved(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        gcmd = MockGCodeCommand()
        switch.assign(lane, "extruder1", False, gcmd)
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        assert "lane_Turtle_1" in extruder_of(switch, "extruder1").lanes
        assert "lane_Turtle_1" not in extruder_of(switch, "extruder").lanes
        # With no print running the lane stays there.
        assert (switch._by_print, switch._due_back) == ([], [])
        switch.afc.save_vars.assert_called_once_with()
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: lane_Turtle_1 now feeds extruder1")

    def test_a_lane_a_print_moves_is_there_for_that_print(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        switch.afc.function.in_print.return_value = True
        gcmd = MockGCodeCommand()
        switch.assign(lane, "extruder1", False, gcmd)
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        assert (switch._by_print, switch._due_back) == (["lane_Turtle_1"], [])
        switch.afc.save_vars.assert_called_once_with()
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: lane_Turtle_1 now feeds extruder1 for this print")

    def test_a_lane_a_print_sends_to_its_starting_extruder_stays_there(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._due_back = ["lane_Turtle_1"]
        switch.afc.function.in_print.return_value = True
        gcmd = MockGCodeCommand()
        switch.assign(lane, "extruder", False, gcmd)
        assert lane.extruder_obj is extruder_of(switch, "extruder")
        assert (switch._by_print, switch._due_back) == ([], [])
        switch.afc.save_vars.assert_called_once_with()
        gcmd.respond_info.assert_called_once_with("Switch_1: lane_Turtle_1 now feeds extruder")

    def test_without_an_extruder_the_lane_goes_back_to_its_start(self):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        lane.set_extruder(extruder_of(switch, "extruder"))
        gcmd = MockGCodeCommand()
        switch.assign(lane, None, False, gcmd)
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        gcmd.respond_info.assert_called_once_with("Switch_1: lane_Ace2_1 now feeds extruder1")

    def test_a_loaded_lane_is_not_moved_unless_it_may_be_unloaded(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.tool_loaded = True
        switch.afc.TOOL_UNLOAD = MagicMock()
        with pytest.raises(CommandError) as err:
            switch.assign(lane, "extruder1", False, MockGCodeCommand())
        assert str(err.value) == ("Switch_1: lane_Turtle_1 is loaded in extruder. Unload it "
                                  "first, or add UNLOAD=1.")
        assert lane.extruder_obj.name == "extruder"
        switch.afc.TOOL_UNLOAD.assert_not_called()

    def test_a_loaded_lane_is_unloaded_first_when_asked(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.tool_loaded = True
        switch.afc.function.get_current_extruder.return_value = "th_extruder"

        def tool_unload(held: FakeLane, **_kwargs: Any) -> bool:
            held.tool_loaded = False
            return True

        switch.afc.TOOL_UNLOAD = MagicMock(side_effect=tool_unload)
        gcmd = MockGCodeCommand()
        switch.assign(lane, "extruder1", True, gcmd)
        switch.afc.TOOL_UNLOAD.assert_called_once_with(lane, set_start_time=False,
                                                       force_unload=True, gcmd=gcmd)
        assert lane.extruder_obj.name == "extruder1"
        switch.afc.save_vars.assert_called_once_with()

    def test_a_lane_that_does_not_unload_stays(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.tool_loaded = True
        switch.afc.function.get_current_extruder.return_value = "th_extruder"
        switch.afc.TOOL_UNLOAD = MagicMock(return_value=False)
        with pytest.raises(CommandError) as err:
            switch.assign(lane, "extruder1", True, MockGCodeCommand())
        assert str(err.value) == "Switch_1: lane_Turtle_1 could not be unloaded from extruder"
        assert lane.extruder_obj.name == "extruder"
        switch.afc.save_vars.assert_not_called()

    def test_an_unloaded_lane_is_not_sent_through_an_unload(self):
        switch = plumbed()
        switch.afc.TOOL_UNLOAD = MagicMock()
        switch.assign(lane_of(switch, "Turtle_1"), "extruder1", True, MockGCodeCommand())
        switch.afc.TOOL_UNLOAD.assert_not_called()

    def test_no_lane_is_moved_while_no_outlet_has_its_toolhead(self):
        switch = known()
        lane = lane_of(switch, "Bambu_AMS_1")
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch.assign(lane, "extruder1", False, gcmd)
        assert str(err.value) == (
            "Chain_1: lane_Bambu_AMS_1 stays where it is, the toolhead each outlet feeds is "
            "not known. Run AFC_BRIDGEBOX_SWITCH_SETUP SWITCH=Chain_1 first, or set "
            "switch_outlet_a and switch_outlet_b.")
        assert lane.extruder_obj.name == "extruder"
        switch.afc.save_vars.assert_not_called()
        gcmd.respond_info.assert_not_called()


class TestAfcBridgeBoxSwitchNoteAssign:
    @staticmethod
    def _switch() -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch with other lanes a print moved and due back"""
        switch = plumbed()
        switch._by_print = ["lane_Ace2_1"]
        switch._due_back = ["lane_Bambu_AMS_1"]
        return switch

    def test_a_lane_a_running_print_put_off_its_starting_extruder_is_the_prints(self):
        switch = self._switch()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._note_assign(lane, True)
        assert switch._by_print == ["lane_Ace2_1", "lane_Turtle_1"]
        assert switch._due_back == ["lane_Bambu_AMS_1"]
        switch.afc.save_vars.assert_called_once_with()

    def test_a_lane_that_was_due_back_is_the_new_prints_and_listed_once(self):
        switch = self._switch()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._by_print = ["lane_Turtle_1", "lane_Ace2_1"]
        switch._due_back = ["lane_Turtle_1", "lane_Bambu_AMS_1"]
        switch._note_assign(lane, True)
        assert switch._by_print == ["lane_Ace2_1", "lane_Turtle_1"]
        assert switch._due_back == ["lane_Bambu_AMS_1"]

    def test_with_no_print_running_the_lane_is_no_prints(self):
        switch = self._switch()
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._by_print.append("lane_Turtle_1")
        switch._due_back.append("lane_Turtle_1")
        switch._note_assign(lane, False)
        assert switch._by_print == ["lane_Ace2_1"]
        assert switch._due_back == ["lane_Bambu_AMS_1"]
        switch.afc.save_vars.assert_called_once_with()

    def test_a_lane_a_print_put_on_its_starting_extruder_is_no_prints(self):
        switch = self._switch()
        lane = lane_of(switch, "Turtle_1")
        switch._by_print.append("lane_Turtle_1")
        switch._due_back.append("lane_Turtle_1")
        switch._note_assign(lane, True)
        assert switch._by_print == ["lane_Ace2_1"]
        assert switch._due_back == ["lane_Bambu_AMS_1"]
        switch.afc.save_vars.assert_called_once_with()


class TestAfcBridgeBoxSwitchHandleCalibrationButtons:
    def test_its_button_is_added_after_the_ones_that_are_there(self):
        switch = make_switch()
        buttons = [("Turtle_1", "UNIT_CALIBRATION UNIT=Turtle_1", "primary")]
        switch.handle_calibration_buttons(buttons)
        assert buttons == [
            ("Turtle_1", "UNIT_CALIBRATION UNIT=Turtle_1", "primary"),
            ("Switch_1 filament switch", "AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH=Switch_1",
             "primary")]

    def test_a_switch_with_units_that_are_found_by_loading_adds_one_button_too(self):
        switch = finding()
        buttons: List[Tuple[str, str, str]] = []
        switch.handle_calibration_buttons(buttons)
        assert buttons == [
            ("Switch_1 filament switch", "AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH=Switch_1",
             "primary")]

    def test_afc_asks_for_it_when_it_builds_its_calibration_menu(self):
        printer = make_printer()
        make_switch(printer)
        buttons: List[Tuple[str, str, str]] = []
        printer.send_event("afc:calibration_buttons", buttons)
        assert buttons == [
            ("Switch_1 filament switch", "AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH=Switch_1",
             "primary")]


class TestAfcBridgeBoxSwitchCmdAFCBridgeboxSwitchCalibration:
    @staticmethod
    def _measuring(lane: str) -> List[Tuple[str, str]]:
        """:return list: the lines of the prompt that says a lane is measuring"""
        return [("raw", "// action:prompt_begin Switch_1 Outlets"),
                ("raw", "// action:prompt_text Measuring the tube from each outlet to its "
                        f"toolhead with {lane}. The lane goes to each toolhead in turn and "
                        "back."),
                SHOW]

    @staticmethod
    def _again(outcome: str) -> List[Tuple[str, str]]:
        """:return list: the lines of the units' prompt with an outcome, no unit to offer"""
        menu = TestAfcBridgeBoxSwitchCalibrationMenu
        return [menu.BEGIN, ("raw", f"// action:prompt_text {outcome}"), menu.NO_LANE] + TO_AFC

    @pytest.mark.parametrize("params", [{}, {"OUTLETS": 1}, {"OUTLETS": 0}])
    def test_without_a_lane_it_opens_the_menu(self, params):
        switch = plumbed(**LENGTHS)
        switch._measure = MagicMock()
        gcmd = MockGCodeCommand(params)
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        menu = TestAfcBridgeBoxSwitchCalibrationMenu
        assert switch.logger.messages == [menu.BEGIN, menu.NO_LANE] + menu.FOOT
        switch._measure.assert_not_called()
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        switch.afc.function.ConfigRewrite.assert_not_called()
        gcmd.respond_info.assert_not_called()

    @pytest.mark.parametrize("params", [{}, {"OUTLETS": 0}])
    def test_with_units_that_are_found_by_loading_it_first_offers_what_to_calibrate(self,
                                                                                    params):
        switch = finding(**LENGTHS)
        switch._measure = MagicMock()
        gcmd = MockGCodeCommand(params)
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        menu = TestAfcBridgeBoxSwitchMenu
        assert switch.logger.messages == [
            menu.BEGIN, ("raw", menu.PICK[1] + " Inlet not known yet: Bambu_AMS_1, "
                                               "Bambu_AMS_2."),
            menu.INLETS, menu.OUTLETS] + TO_AFC
        switch._measure.assert_not_called()
        gcmd.respond_info.assert_not_called()

    def test_with_outlets_it_goes_on_to_the_units_that_can_measure(self):
        switch = finding()
        switch._measure = MagicMock()
        gcmd = MockGCodeCommand({"OUTLETS": 1})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        menu = TestAfcBridgeBoxSwitchCalibrationMenu
        assert switch.logger.messages == [menu.BEGIN, menu.NO_LANE] + TO_SWITCH
        switch._measure.assert_not_called()
        gcmd.respond_info.assert_not_called()

    def test_with_a_unit_it_opens_that_units_lanes(self):
        switch = plumbed(**LENGTHS)
        switch._measure = MagicMock()
        lane = lane_of(switch, "Ace2_1")
        lane.load_state = True
        add_hub(lane)
        gcmd = MockGCodeCommand({"UNIT": "Ace2_1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        menu = TestAfcBridgeBoxSwitchLaneMenu
        assert switch.logger.messages == [
            ("raw", "// action:prompt_begin Switch_1 Outlets Ace2_1"),
            ("raw", "// action:prompt_text Select a loaded lane from Ace2_1 to measure the tube "
                    "from each outlet with."),
            menu.ROW, menu.button("lane_Ace2_1", "primary"), menu.ROW_END] + menu.FOOT
        switch._measure.assert_not_called()
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        switch.afc.function.ConfigRewrite.assert_not_called()
        gcmd.respond_info.assert_not_called()

    def test_a_unit_that_cannot_be_offered_is_the_commands_error(self):
        switch = plumbed(**LENGTHS)
        switch._measure = MagicMock()
        gcmd = MockGCodeCommand({"UNIT": "Turtle_9"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        assert str(err.value) == "Switch_1: Turtle_9 is not a unit of this switch"
        switch._measure.assert_not_called()
        assert switch.logger.messages == []

    def test_a_lane_is_measured_whatever_unit_is_named_with_it(self):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        switch._measure = MagicMock(return_value={"A": 850.0, "B": 1010.0})
        gcmd = MockGCodeCommand({"LANE": "lane_Ace2_1", "UNIT": "Turtle_1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        switch._measure.assert_called_once_with(lane, gcmd)
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        assert switch.logger.messages == self._measuring("lane_Ace2_1") + self._again(
            "Switch_1: with lane_Ace2_1 the tube through outlet A is 850.0 mm and through "
            "outlet B 1010.0 mm")

    def test_a_lane_that_is_not_on_this_switch(self):
        switch = plumbed(**LENGTHS)
        switch._measure = MagicMock()
        gcmd = MockGCodeCommand({"LANE": "lane0"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        assert str(err.value) == "Switch_1: lane0 is not a lane of this switch"
        switch._measure.assert_not_called()
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        switch.afc.function.ConfigRewrite.assert_not_called()
        gcmd.respond_info.assert_not_called()
        # Nothing was measuring, so the units' prompt opens with the reason alone.
        assert switch.logger.messages == self._again(
            "Switch_1: lane0 is not a lane of this switch")

    def test_a_measurement_that_fails_is_the_commands_error_and_keeps_the_lengths(self):
        switch = plumbed(**LENGTHS)
        switch._measure = MagicMock(side_effect=SwitchError("Switch_1: it did not work"))
        gcmd = MockGCodeCommand({"LANE": "lane_Turtle_1"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        assert str(err.value) == "Switch_1: it did not work"
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        switch.afc.function.ConfigRewrite.assert_not_called()
        gcmd.respond_info.assert_not_called()
        assert switch.logger.messages == self._measuring("lane_Turtle_1") + self._again(
            "Switch_1: it did not work")

    def test_a_measurement_that_left_the_lane_reset_prompt_open_does_not_replace_it(self):
        switch = plumbed(**LENGTHS)
        switch._measure = MagicMock(side_effect=SwitchResetError("Switch_1: reset it"))
        gcmd = MockGCodeCommand({"LANE": "lane_Turtle_1"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        assert str(err.value) == "Switch_1: reset it"
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        switch.afc.function.ConfigRewrite.assert_not_called()
        gcmd.respond_info.assert_not_called()
        assert switch.logger.messages == self._measuring("lane_Turtle_1")

    def test_any_other_failure_closes_the_prompt_and_is_passed_on(self):
        switch = plumbed(**LENGTHS)
        switch._measure = MagicMock(side_effect=RuntimeError("the link dropped"))
        gcmd = MockGCodeCommand({"LANE": "lane_Turtle_1"})
        with pytest.raises(RuntimeError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        assert str(err.value) == "the link dropped"
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        switch.afc.function.ConfigRewrite.assert_not_called()
        gcmd.respond_info.assert_not_called()
        gcmd.error.assert_not_called()
        assert switch.logger.messages == self._measuring("lane_Turtle_1") + [END]

    def test_both_lengths_are_rounded_kept_and_saved_to_the_section(self):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        switch._measure = MagicMock(return_value={"B": 1012.12, "A": 851.87})
        gcmd = MockGCodeCommand({"LANE": "lane_Ace2_1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        switch._measure.assert_called_once_with(lane, gcmd)
        assert switch.lengths == {"A": 851.9, "B": 1012.1}
        assert switch.afc.function.ConfigRewrite.call_args_list == [
            call(SECTION, "outlet_a_length", 851.9), call(SECTION, "outlet_b_length", 1012.1)]
        said = ("Switch_1: with lane_Ace2_1 the tube through outlet A is 851.9 mm and through "
                "outlet B 1012.1 mm")
        gcmd.respond_info.assert_called_once_with(said)
        assert switch.logger.messages == self._measuring("lane_Ace2_1") + self._again(said)

    def test_a_lane_measures_both_tubes_from_start_to_end(self):
        switch = plumbed(SimChain(route=2))
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane)
        measuring(switch, lane, Run(852.0), Run(1013.0))
        gcmd = MockGCodeCommand({"LANE": "lane_Turtle_1"})
        assert switch.calibrated() is False
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        assert switch.lengths == {"A": 852.0, "B": 1013.0}
        assert switch.calibrated() is True
        assert tube_of(hub) == (852.0, 852.0)
        assert switch.afc.function.ConfigRewrite.call_args_list == [
            call("AFC_hub Turtle_1", "afc_bowden_length", 852.0),
            call("AFC_hub Turtle_1", "afc_unload_bowden_length", 852.0),
            call(SECTION, "outlet_a_length", 852.0), call(SECTION, "outlet_b_length", 1013.0)]
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: with lane_Turtle_1 the tube through outlet A is 852.0 mm and through "
            "outlet B 1013.0 mm")

    def test_a_switch_its_chain_made_has_the_chain_keep_both_lengths(self):
        switch = known(outlet_a="extruder1", outlet_b="extruder")
        lane = lane_of(switch, "Bambu_AMS_1")
        switch._measure = MagicMock(return_value={"A": 851.87, "B": 1012.12})
        gcmd = MockGCodeCommand({"LANE": "lane_Bambu_AMS_1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        switch._measure.assert_called_once_with(lane, gcmd)
        assert switch.lengths == {"A": 851.9, "B": 1012.1}
        assert chain_of(switch).saved == [{"outlet_a_length": 851.9, "outlet_b_length": 1012.1}]
        switch.afc.function.ConfigRewrite.assert_not_called()
        said = ("Chain_1: with lane_Bambu_AMS_1 the tube through outlet A is 851.9 mm and "
                "through outlet B 1012.1 mm")
        gcmd.respond_info.assert_called_once_with(said)
        menu = TestAfcBridgeBoxSwitchCalibrationMenu
        assert switch.logger.messages == [
            menu.MADE_BEGIN,
            ("raw", "// action:prompt_text Measuring the tube from each outlet to its toolhead "
                    "with lane_Bambu_AMS_1. The lane goes to each toolhead in turn and back."),
            SHOW, menu.MADE_BEGIN, ("raw", f"// action:prompt_text {said}"), menu.MADE_NO_LANE,
            menu.FIND] + TO_MADE


class TestAfcBridgeBoxSwitchOpenMenu:
    @staticmethod
    def _stubbed(switch: afcBridgeBoxSwitch) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: ``switch`` with both of its prompts stubbed"""
        switch._menu = MagicMock()
        switch._calibration_menu = MagicMock()
        return switch

    def test_a_switch_with_units_that_are_found_by_loading_offers_what_to_calibrate(self):
        switch = self._stubbed(finding())
        gcmd = MockGCodeCommand()
        switch._open_menu(gcmd, False, "Switch_1: it did not work")
        switch._menu.assert_called_once_with(gcmd, "Switch_1: it did not work")
        switch._calibration_menu.assert_not_called()

    def test_for_the_outlets_it_opens_their_prompt_at_once(self):
        switch = self._stubbed(finding())
        gcmd = MockGCodeCommand()
        switch._open_menu(gcmd, True, "Switch_1: it did not work")
        switch._calibration_menu.assert_called_once_with(gcmd, "Switch_1: it did not work")
        switch._menu.assert_not_called()

    def test_any_other_switch_has_only_the_prompt_for_the_outlets(self):
        switch = self._stubbed(plumbed())
        gcmd = MockGCodeCommand()
        switch._open_menu(gcmd, False)
        switch._calibration_menu.assert_called_once_with(gcmd, "")
        switch._menu.assert_not_called()

    def test_with_no_outcome_none_is_shown(self):
        switch = self._stubbed(finding())
        gcmd = MockGCodeCommand()
        switch._open_menu(gcmd, False)
        switch._menu.assert_called_once_with(gcmd, "")


class TestAfcBridgeBoxSwitchCalibrationMenu:
    BEGIN = ("raw", "// action:prompt_begin Switch_1 Outlets")
    PICK = ("raw", "// action:prompt_text Measures the tube from each outlet to its toolhead "
                   "with one lane, so a lane loads the right distance through either outlet. "
                   "Unload the toolheads, then pick the unit to measure with.")
    NO_LANE = ("raw", "// action:prompt_text No lane that can measure is loaded in its unit. "
                      "Load one, or set outlet_a_length and outlet_b_length in "
                      "[AFC_BridgeBox_switch Switch_1] by hand.")
    FOOT = TO_AFC
    # The same on the switch Chain_1 made, what it says while no outlet has its toolhead, and
    # the button that finds the toolheads.
    MADE_BEGIN = ("raw", "// action:prompt_begin Chain_1 Outlets")
    MADE_NO_LANE = ("raw", "// action:prompt_text No lane that can measure is loaded in its "
                           "unit. Load one, or set switch_outlet_a_length and "
                           "switch_outlet_b_length in [AFC_BridgeBox Chain_1] by hand.")
    NO_TOOLHEAD = ("raw", "// action:prompt_text The toolhead each outlet feeds is not known "
                          "yet. Find the toolheads feeds a lane through each outlet until a "
                          "toolhead sensor reads it, and reels it back. Unload the toolheads "
                          "first.")
    FIND = ("raw", "// action:prompt_button Find the toolheads|AFC_BRIDGEBOX_SWITCH_SETUP "
                   "SWITCH=Chain_1 OUTLETS=1|default")

    @staticmethod
    def _button(unit: str, style: str) -> Tuple[str, str]:
        """:return tuple: the line of the button that opens a unit's lanes"""
        return ("raw", f"// action:prompt_button {unit}|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                       f"SWITCH=Switch_1 UNIT={unit}|{style}")

    def test_one_button_for_each_unit_that_can_measure_primary_and_secondary_in_turn(self):
        switch = plumbed()
        for unit in VALUES["inlet_a"] + VALUES["inlet_b"]:
            lane_of(switch, unit).load_state = True
            add_hub(lane_of(switch, unit))
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK, self._button("Bambu_AMS_1", "primary"),
            self._button("Turtle_1", "secondary"), self._button("Ace2_1", "primary")] + self.FOOT

    def test_a_unit_is_offered_once_however_many_of_its_lanes_can_measure(self):
        printer = make_printer([SimChain()])
        for name in ("lane1", "lane2", "lane3"):
            lane = add_lane(printer, name, "Turtle_1")
            lane.load_state = True
            add_hub(lane)
        switch = make_switch(printer)
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK, self._button("Turtle_1", "primary")] + self.FOOT

    def test_with_no_lane_that_can_measure_it_says_what_to_do(self):
        switch = plumbed()
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [self.BEGIN, self.NO_LANE] + self.FOOT

    def test_the_outcome_of_the_last_run_is_shown_above_the_offer(self):
        switch = plumbed()
        lane_of(switch, "Turtle_1").load_state = True
        add_hub(lane_of(switch, "Turtle_1"))
        switch._calibration_menu(MockGCodeCommand(), "Switch_1: it did not work")
        assert switch.logger.messages == [
            self.BEGIN, ("raw", "// action:prompt_text Switch_1: it did not work"), self.PICK,
            self._button("Turtle_1", "primary")] + self.FOOT

    def test_and_above_what_to_do_when_no_lane_can_measure(self):
        switch = plumbed()
        switch._calibration_menu(MockGCodeCommand(), "Switch_1: it did not work")
        assert switch.logger.messages == [
            self.BEGIN, ("raw", "// action:prompt_text Switch_1: it did not work"),
            self.NO_LANE] + self.FOOT

    def test_with_units_that_are_found_by_loading_it_goes_back_to_what_to_calibrate(self):
        switch = finding()
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [self.BEGIN, self.NO_LANE] + TO_SWITCH

    def test_a_unit_that_measures_by_loading_is_said_to(self):
        switch = plumbed()
        for unit in ("Bambu_AMS_1", "Turtle_1"):
            lane_of(switch, unit).load_state = True
        add_path_unit(switch, lane_of(switch, "Bambu_AMS_1"))
        add_hub(lane_of(switch, "Turtle_1"))
        switch._calibration_menu(MockGCodeCommand())
        said = ("raw", self.PICK[1] + " A lane of Bambu_AMS_1 is loaded into each toolhead and "
                                      "unloaded again to measure.")
        assert switch.logger.messages == [
            self.BEGIN, said, self._button("Bambu_AMS_1", "primary"),
            self._button("Turtle_1", "secondary")] + self.FOOT

    def test_so_is_each_of_several(self):
        switch = plumbed()
        for unit in ("Bambu_AMS_1", "Ace2_1"):
            lane_of(switch, unit).load_state = True
            add_path_unit(switch, lane_of(switch, unit))
        switch._calibration_menu(MockGCodeCommand())
        said = ("raw", self.PICK[1] + " A lane of Bambu_AMS_1 or Ace2_1 is loaded into each "
                                      "toolhead and unloaded again to measure.")
        assert switch.logger.messages == [
            self.BEGIN, said, self._button("Bambu_AMS_1", "primary"),
            self._button("Ace2_1", "secondary")] + self.FOOT

    def test_with_no_toolhead_on_an_outlet_it_only_offers_to_find_them(self):
        switch = known()
        # A lane that could measure is not offered: there is no toolhead to measure to.
        lane_of(switch, "Bambu_AMS_1").load_state = True
        add_hub(lane_of(switch, "Bambu_AMS_1"))
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.MADE_BEGIN, self.NO_TOOLHEAD, self.FIND] + TO_MADE

    def test_the_outcome_is_shown_above_that_too(self):
        switch = known()
        switch._calibration_menu(MockGCodeCommand(), "Chain_1: it did not work")
        assert switch.logger.messages == [
            self.MADE_BEGIN, ("raw", "// action:prompt_text Chain_1: it did not work"),
            self.NO_TOOLHEAD, self.FIND] + TO_MADE

    def test_a_switch_its_chain_made_names_the_lengths_as_its_chain_writes_them(self):
        switch = known(outlet_a="extruder1", outlet_b="extruder")
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.MADE_BEGIN, self.MADE_NO_LANE, self.FIND] + TO_MADE

    def test_the_toolheads_can_be_found_again_once_they_are_known(self):
        switch = known(outlet_a="extruder1", outlet_b="extruder")
        lane_of(switch, "Bambu_AMS_1").load_state = True
        add_hub(lane_of(switch, "Bambu_AMS_1"))
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.MADE_BEGIN, self.PICK,
            ("raw", "// action:prompt_button Bambu_AMS_1|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                    "SWITCH=Chain_1 UNIT=Bambu_AMS_1|primary"),
            self.FIND] + TO_MADE

    def test_outlets_the_chains_section_sets_are_not_offered_to_be_found(self):
        switch = known(outlet_a="extruder1", outlet_b="extruder",
                       by_hand=["outlet_a", "outlet_b"])
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [self.MADE_BEGIN, self.MADE_NO_LANE] + TO_MADE


class TestAfcBridgeBoxSwitchMenu:
    BEGIN = ("raw", "// action:prompt_begin Switch_1 Calibration")
    PICK = ("raw", "// action:prompt_text Inlets finds the inlet each unit is plumbed into, by "
                   "feeding one of its lanes to the switch. Outlets measures the tube from each "
                   "outlet to its toolhead, with a lane of a unit whose inlet is known.")
    INLETS = ("raw", "// action:prompt_button Inlets|AFC_BRIDGEBOX_SWITCH_IDENTIFY "
                     "SWITCH=Switch_1|primary")
    OUTLETS = ("raw", "// action:prompt_button Outlets|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                      "SWITCH=Switch_1 OUTLETS=1|secondary")

    def test_it_offers_the_inlets_and_the_outlets(self):
        switch = finding(**LENGTHS)
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch._menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK, self.INLETS, self.OUTLETS] + TO_AFC

    def test_the_units_whose_inlet_is_not_known_are_named(self):
        switch = finding(**LENGTHS)
        found(switch, Bambu_AMS_1="A")
        switch._menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, ("raw", self.PICK[1] + " Inlet not known yet: Bambu_AMS_2."),
            self.INLETS, self.OUTLETS] + TO_AFC

    def test_tube_lengths_that_are_not_measured_are_said_to_be(self):
        switch = finding()
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch._menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, ("raw", self.PICK[1] + " The tube lengths are not measured yet."),
            self.INLETS, self.OUTLETS] + TO_AFC

    def test_with_no_toolhead_on_an_outlet_it_says_the_outlets_find_them(self):
        switch = made()
        switch._menu(MockGCodeCommand())
        assert switch.logger.messages == [
            ("raw", "// action:prompt_begin Chain_1 Calibration"),
            ("raw", self.PICK[1] + " Inlet not known yet: Bambu_AMS_1, Bambu_AMS_2. The "
                                   "toolhead each outlet feeds is not known yet: Outlets finds "
                                   "them."),
            ("raw", "// action:prompt_button Inlets|AFC_BRIDGEBOX_SWITCH_IDENTIFY "
                    "SWITCH=Chain_1|primary"),
            ("raw", "// action:prompt_button Outlets|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                    "SWITCH=Chain_1 OUTLETS=1|secondary")] + TO_AFC

    def test_the_outcome_of_the_last_run_is_shown_above_the_offer_line_by_line(self):
        switch = finding(**LENGTHS)
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch._menu(MockGCodeCommand(), "Switch_1: one thing\nSwitch_1: another")
        assert switch.logger.messages == [
            self.BEGIN, ("raw", "// action:prompt_text Switch_1: one thing"),
            ("raw", "// action:prompt_text Switch_1: another"), self.PICK, self.INLETS,
            self.OUTLETS] + TO_AFC


class TestAfcBridgeBoxSwitchBack:
    def test_a_switch_with_units_that_are_found_by_loading_goes_back_to_its_own_prompt(self):
        assert finding()._back() == [
            ("Back", "AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH=Switch_1", "info")]

    def test_any_other_switch_goes_back_to_afcs_calibration_prompt(self):
        assert plumbed()._back() == [("Back to unit selection", "AFC_CALIBRATION", "info")]


class TestAfcBridgeBoxSwitchLaneMenu:
    BEGIN = ("raw", "// action:prompt_begin Switch_1 Outlets Turtle_1")
    PICK = ("raw", "// action:prompt_text Select a loaded lane from Turtle_1 to measure the "
                   "tube from each outlet with.")
    ROW = ("raw", "// action:prompt_button_group_start")
    ROW_END = ("raw", "// action:prompt_button_group_end")
    # Back to the units that can measure, past the prompt that offers what to calibrate.
    FOOT = [("raw", "// action:prompt_footer_button Back|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                    "SWITCH=Switch_1 OUTLETS=1|info")] + CANCEL

    @staticmethod
    def button(lane: str, style: str) -> Tuple[str, str]:
        """:return tuple: the line of the button that measures with a lane"""
        return ("raw", f"// action:prompt_button {lane}|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                       f"SWITCH=Switch_1 LANE={lane}|{style}")

    @staticmethod
    def _turtle(loaded: Tuple[str, ...]) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch whose Turtle_1 has lanes 1 to 4, these loaded"""
        printer = make_printer([SimChain()])
        for name in ("lane1", "lane2", "lane3", "lane4"):
            lane = add_lane(printer, name, "Turtle_1")
            lane.load_state = name in loaded
            add_hub(lane)
        return make_switch(printer)

    def test_every_lane_that_can_measure_two_to_a_row_primary_then_secondary(self):
        switch = self._turtle(("lane1", "lane2", "lane4"))
        switch._lane_menu("Turtle_1", MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK,
            self.ROW, self.button("lane1", "primary"), self.button("lane2", "secondary"),
            self.ROW_END,
            self.ROW, self.button("lane4", "primary"), self.ROW_END] + self.FOOT

    def test_a_unit_that_measures_by_loading_is_said_to(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        lane.load_state = True
        add_path_unit(switch, lane)
        switch._lane_menu("Turtle_1", MockGCodeCommand())
        said = ("raw", self.PICK[1] + " It is loaded into each toolhead and unloaded again to "
                                      "measure.")
        assert switch.logger.messages == [
            self.BEGIN, said, self.ROW, self.button("lane_Turtle_1", "primary"),
            self.ROW_END] + self.FOOT

    def test_a_unit_that_is_not_on_this_switch(self):
        switch = self._turtle(("lane1",))
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch._lane_menu("Turtle_9", gcmd)
        assert str(err.value) == "Switch_1: Turtle_9 is not a unit of this switch"
        gcmd.error.assert_called_once_with("Switch_1: Turtle_9 is not a unit of this switch")
        assert switch.logger.messages == []

    def test_a_unit_with_no_lane_that_can_measure(self):
        switch = self._turtle(())
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch._lane_menu("Turtle_1", gcmd)
        said = ("Switch_1: no lane that can measure is loaded in Turtle_1. Load one, pick "
                "another unit, or set outlet_a_length and outlet_b_length by hand.")
        assert str(err.value) == said
        gcmd.error.assert_called_once_with(said)
        assert switch.logger.messages == []

    def test_a_switch_its_chain_made_names_the_lengths_as_its_chain_writes_them(self):
        switch = known(outlet_a="extruder1", outlet_b="extruder")
        with pytest.raises(CommandError) as err:
            switch._lane_menu("Bambu_AMS_1", MockGCodeCommand())
        assert str(err.value) == (
            "Chain_1: no lane that can measure is loaded in Bambu_AMS_1. Load one, pick another "
            "unit, or set switch_outlet_a_length and switch_outlet_b_length by hand.")
        assert switch.logger.messages == []


class TestAfcBridgeBoxSwitchMeasuringLanes:
    def test_no_lane_is_loaded_in_its_unit(self):
        switch = plumbed()
        for lane in switch.lanes:
            add_hub(lane)
        assert switch._measuring_lanes() == {}

    def test_a_lane_that_is_loaded_in_its_unit_and_has_a_hub(self):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        lane.load_state = True
        add_hub(lane)
        assert switch._measuring_lanes() == {"Ace2_1": [lane]}

    def test_a_loaded_lane_without_a_hub_cannot_measure(self):
        switch = plumbed()
        lane_of(switch, "Ace2_1").load_state = True
        assert switch._measuring_lanes() == {}

    def test_a_lane_of_a_unit_that_keeps_its_own_length_needs_no_hub(self):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        lane.load_state = True
        add_path_unit(switch, lane)
        assert switch._measuring_lanes() == {"Ace2_1": [lane]}

    def test_a_unit_whose_calibration_gave_no_length_is_not_offered_again(self):
        switch = plumbed()
        for unit in ("Turtle_1", "Ace2_1"):
            lane_of(switch, unit).load_state = True
            add_hub(lane_of(switch, unit))
        switch._lengthless = ["Ace2_1"]
        assert switch._measuring_lanes() == {"Turtle_1": [lane_of(switch, "Turtle_1")]}

    def test_a_unit_whose_inlet_is_not_known_is_not_offered(self):
        switch = finding()
        for unit in BAYS:
            lane_of(switch, unit).load_state = True
            add_hub(lane_of(switch, unit))
        found(switch, Bambu_AMS_2="A")
        assert switch._measuring_lanes() == {"Bambu_AMS_2": [lane_of(switch, "Bambu_AMS_2")]}

    def test_every_lane_of_each_unit_that_can_in_the_order_of_the_lanes(self):
        printer = make_printer([SimChain()])
        lanes = [add_lane(printer, name, unit) for name, unit in (
            ("lane1", "Turtle_1"), ("lane2", "Turtle_1"), ("lane3", "Turtle_1"),
            ("lane9", "Ace2_1"))]
        switch = make_switch(printer)
        for lane in lanes:
            add_hub(lane)
        for lane in lanes[1:]:
            lane.load_state = True
        found = switch._measuring_lanes()
        assert found == {"Turtle_1": [lanes[1], lanes[2]], "Ace2_1": [lanes[3]]}
        assert list(found) == ["Turtle_1", "Ace2_1"]


class TestAfcBridgeBoxSwitchMeasure:
    @staticmethod
    def _switch(unit: str = "Turtle_1",
                **values: Any) -> Tuple[afcBridgeBoxSwitch, FakeLane, FakeHub]:
        """:return tuple: a switch at rest after PREP, a unit's lane and the lane's hub"""
        switch = plumbed(SimChain(route=2), **values)
        switch.afc.prep_done = True
        lane = lane_of(switch, unit)
        return switch, lane, add_hub(lane)

    @staticmethod
    def _refused(switch: afcBridgeBoxSwitch, lane: FakeLane, said: str) -> None:
        """Check a measurement is refused with ``said`` before anything is touched."""
        with pytest.raises(SwitchError) as err:
            switch._measure(lane, MockGCodeCommand())
        assert str(err.value) == said
        lane.unit_obj.calibrate_bowden.assert_not_called()
        assert chain_of(switch).routes == []
        assert switch._routed == {"A": None, "B": None}
        assert switch._offsets == {}
        assert switch.confirmed == []

    def test_a_lane_of_a_unit_whose_inlet_is_not_known_cannot_measure(self):
        switch = finding(SimChain(route=2))
        lane = lane_of(switch, "Bambu_AMS_1")
        add_hub(lane)
        self._refused(switch, lane,
                      "Switch_1: lane_Bambu_AMS_1 cannot measure, the inlet of Bambu_AMS_1 is "
                      "not known. Run AFC_BRIDGEBOX_SWITCH_IDENTIFY UNIT=Bambu_AMS_1 first.")

    def test_with_one_outlet_in_use_there_is_nothing_to_measure(self):
        switch, lane, _hub = self._switch(outlet_b=None)
        self._refused(switch, lane,
                      "Switch_1: one outlet is in use, so there is nothing to measure")

    def test_a_lane_without_a_hub_cannot_measure(self):
        switch, lane, _hub = self._switch()
        lane.hub_obj = None
        self._refused(switch, lane,
                      "Switch_1: lane_Turtle_1 has no hub that holds a tube length. Use a lane "
                      "of another unit, or set outlet_a_length and outlet_b_length by hand.")

    def test_a_lane_of_the_switch_that_is_in_a_toolhead_has_to_come_out_first(self):
        switch, lane, _hub = self._switch()
        lane_of(switch, "Ace2_1").tool_loaded = True
        self._refused(switch, lane,
                      "Switch_1: unload lane_Ace2_1 first. The toolheads have to be empty to "
                      "measure.")

    def test_so_does_the_lane_afc_has_loaded_whatever_it_goes_through(self):
        switch, lane, _hub = self._switch()
        switch.afc.current = "lane0"
        self._refused(switch, lane,
                      "Switch_1: unload lane0 first. The toolheads have to be empty to measure.")

    def test_every_lane_to_unload_is_named_once(self):
        switch, lane, _hub = self._switch()
        lane_of(switch, "Bambu_AMS_1").tool_loaded = True
        lane_of(switch, "Ace2_1").tool_loaded = True
        switch.afc.current = "lane_Bambu_AMS_1"
        self._refused(switch, lane,
                      "Switch_1: unload lane_Bambu_AMS_1, lane_Ace2_1 first. The toolheads "
                      "have to be empty to measure.")

    def test_one_outlet_in_use_is_said_before_a_missing_hub(self):
        switch, lane, _hub = self._switch(outlet_b=None)
        lane.hub_obj = None
        switch.afc.current = "lane0"
        self._refused(switch, lane,
                      "Switch_1: one outlet is in use, so there is nothing to measure")

    def test_a_missing_hub_is_said_before_the_lanes_to_unload(self):
        switch, lane, _hub = self._switch()
        lane.hub_obj = None
        lane_of(switch, "Ace2_1").tool_loaded = True
        self._refused(switch, lane,
                      "Switch_1: lane_Turtle_1 has no hub that holds a tube length. Use a lane "
                      "of another unit, or set outlet_a_length and outlet_b_length by hand.")

    def test_the_units_own_outlet_is_measured_first_and_then_the_other(self):
        switch, lane, _hub = self._switch()
        through = measuring(switch, lane, Run(852.0), Run(1013.0))
        gcmd = MockGCodeCommand()
        assert switch._measure(lane, gcmd) == {"A": 852.0, "B": 1013.0}
        assert through == [("extruder", "A"), ("extruder1", "B")]
        assert lane.unit_obj.calibrate_bowden.call_args_list == [
            call(lane, 25.0, 5.0, gcmd=gcmd), call(lane, 25.0, 5.0, gcmd=gcmd)]
        assert switch.logger.messages == []

    def test_a_lane_of_inlet_b_goes_through_outlet_b_first(self):
        switch, lane, hub = self._switch("Ace2_1")
        through = measuring(switch, lane, Run(1013.0, shows="B"), Run(852.0, shows="B"))
        assert switch._measure(lane, MockGCodeCommand()) == {"A": 852.0, "B": 1013.0}
        assert through == [("extruder1", "B"), ("extruder", "A")]
        assert tube_of(hub) == (1013.0, 1013.0)
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        assert switch.confirmed == ["Ace2_1"]

    def test_the_lane_and_its_tube_are_left_as_they_were_and_the_unit_is_confirmed(self):
        switch, lane, hub = self._switch()
        measuring(switch, lane, Run(852.0), Run(1013.0))
        switch._measure(lane, MockGCodeCommand())
        assert lane.extruder_obj is extruder_of(switch, "extruder")
        assert switch._routed == {"A": None, "B": None}
        # The other outlet's run saved its length as the hub's own, so the own one is put
        # back: the one the first run measured.
        assert tube_of(hub) == (852.0, 852.0)
        assert switch.afc.function.ConfigRewrite.call_args_list == [
            call("AFC_hub Turtle_1", "afc_bowden_length", 852.0),
            call("AFC_hub Turtle_1", "afc_unload_bowden_length", 852.0)]
        assert switch.confirmed == ["Turtle_1"]
        switch.afc.save_vars.assert_called_once_with()
        assert switch._moving is False
        # The watch then sends the inlet back to the outlet it rests on.
        assert chain_of(switch).routes == [0]
        switch._watch(0.0)
        assert chain_of(switch).routes == [0, 2]

    def test_a_watch_that_came_by_during_the_runs_leaves_no_difference_on_the_tube(self):
        # With both lengths set the watch adds their difference to the tube of a lane that
        # is on its way through the other outlet.
        switch, lane, hub = self._switch(**LENGTHS)
        measuring(switch, lane, Run(852.0), Run(1013.0))
        switch._measure(lane, MockGCodeCommand())
        assert tube_of(hub) == (852.0, 852.0)
        assert switch._offsets == {hub: 0.0}
        switch._watch(0.0)
        assert tube_of(hub) == (852.0, 852.0)

    def test_a_lane_its_unit_calibrates_is_not_one_that_is_measured_by_loading(self):
        # So the watch adds the difference during its run through the other outlet, as for
        # any lane on its way there.
        switch, lane, hub = self._switch(**LENGTHS)
        measuring(switch, lane, Run(852.0), Run(1013.0))
        scripted = lane.unit_obj.calibrate_bowden.side_effect
        during: List[Tuple[Optional[FakeLane], float]] = []

        def noted(cur_lane: FakeLane, dis: float, tol: float,
                  gcmd: Any) -> Tuple[bool, str, float]:
            answer = scripted(cur_lane, dis, tol, gcmd)
            during.append((switch._measuring, switch._offsets[hub]))
            return answer

        lane.unit_obj.calibrate_bowden = MagicMock(side_effect=noted)
        switch._measure(lane, MockGCodeCommand())
        assert during == [(None, 0.0), (None, 160.0)]

    def test_what_was_added_to_the_tube_is_taken_off_before_it_is_measured(self):
        switch, lane, hub = self._switch(**LENGTHS)
        switch._offset(hub, 160.0)
        measuring(switch, lane, Run(0.0, done=False, why="Failed"))
        with pytest.raises(SwitchError):
            switch._measure(lane, MockGCodeCommand())
        assert tube_of(hub) == (900.0, 880.0)
        assert switch._offsets == {hub: 0.0}
        switch.afc.function.ConfigRewrite.assert_not_called()

    def test_a_first_run_that_fails_stops_the_measurement_there(self):
        switch, lane, hub = self._switch()
        measuring(switch, lane, Run(120.0, done=False, why="Failed Moving to hub after 120mm"))
        with pytest.raises(SwitchError) as err:
            switch._measure(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 did not measure the tube to extruder through outlet A. "
            "Failed Moving to hub after 120mm. Reset lane_Turtle_1 before the next try.")
        assert lane.unit_obj.calibrate_bowden.call_count == 1
        assert chain_of(switch).routes == []
        assert tube_of(hub) == (900.0, 880.0)
        switch.afc.function.ConfigRewrite.assert_not_called()
        assert switch._routed == {"A": None, "B": None}
        assert switch.confirmed == []
        switch.afc.save_vars.assert_not_called()

    def test_a_unit_that_measures_no_length_stops_at_its_own_outlet(self):
        switch, lane, hub = self._switch()
        measuring(switch, lane, Run(0))
        with pytest.raises(SwitchError) as err:
            switch._measure(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: the unit of lane_Turtle_1 measures no tube length. Use a lane of "
            "another unit, or set outlet_a_length and outlet_b_length by hand.")
        assert lane.unit_obj.calibrate_bowden.call_count == 1
        assert chain_of(switch).routes == []
        assert tube_of(hub) == (900.0, 880.0)
        assert switch.confirmed == []

    def test_a_second_run_that_fails_leaves_what_the_first_run_measured(self):
        switch, lane, hub = self._switch()
        measuring(switch, lane, Run(852.0), Run(300.0, done=False, why="Failed"))
        with pytest.raises(SwitchError) as err:
            switch._measure(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 did not measure the tube to extruder1 through outlet B. "
            "Failed. Reset lane_Turtle_1 before the next try.")
        assert tube_of(hub) == (852.0, 852.0)
        switch.afc.function.ConfigRewrite.assert_not_called()
        assert lane.extruder_obj is extruder_of(switch, "extruder")
        assert switch._routed == {"A": None, "B": None}
        assert switch.confirmed == []

    def test_a_second_run_that_measured_and_is_refused_has_its_length_taken_back(self):
        # The run saved the other outlet's length as the hub's own before the inlet's check.
        switch, lane, hub = self._switch()
        measuring(switch, lane, Run(852.0), Run(1013.0, shows=None))
        with pytest.raises(SwitchError) as err:
            switch._measure(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 reached extruder1 and inlet A never read filament. "
            "Neither inlet did. Check where Turtle_1 is plumbed.")
        assert tube_of(hub) == (852.0, 852.0)
        assert switch.afc.function.ConfigRewrite.call_args_list == [
            call("AFC_hub Turtle_1", "afc_bowden_length", 852.0),
            call("AFC_hub Turtle_1", "afc_unload_bowden_length", 852.0)]
        assert lane.extruder_obj is extruder_of(switch, "extruder")
        assert switch._routed == {"A": None, "B": None}
        assert switch.confirmed == []

    def test_a_run_that_breaks_still_leaves_the_lane_and_its_tube_as_they_were(self):
        switch, lane, hub = self._switch()

        def breaks_through_the_other_outlet(cur_lane: FakeLane, dis: float, tol: float,
                                            gcmd: Any) -> Tuple[bool, str, float]:
            if cur_lane.extruder_obj.name == "extruder1":
                hub.afc_bowden_length = hub.afc_unload_bowden_length = 1013.0
                raise RuntimeError("the unit broke")
            chain_of(switch).fill(a=True)
            switch._look()
            chain_of(switch).fill(a=False)
            return True, "ok", 900.0

        lane.unit_obj.calibrate_bowden = MagicMock(side_effect=breaks_through_the_other_outlet)
        with pytest.raises(RuntimeError):
            switch._measure(lane, MockGCodeCommand())
        assert tube_of(hub) == (900.0, 880.0)
        assert switch.afc.function.ConfigRewrite.call_args_list == [
            call("AFC_hub Turtle_1", "afc_bowden_length", 900.0),
            call("AFC_hub Turtle_1", "afc_unload_bowden_length", 880.0)]
        assert lane.extruder_obj is extruder_of(switch, "extruder")
        assert switch._routed == {"A": None, "B": None}
        assert switch.confirmed == []


class TestAfcBridgeBoxSwitchMeasureByLoading:
    """`_measure` with a lane whose unit keeps its own length and measures its path at a load."""

    @staticmethod
    def _switch(**values: Any) -> Tuple[afcBridgeBoxSwitch, FakeLane, FakePathUnit]:
        """:return tuple: a switch at rest after PREP, Bambu_AMS_1's lane and its unit"""
        switch = plumbed(SimChain(route=2), **values)
        switch.afc.prep_done = True
        lane = lane_of(switch, "Bambu_AMS_1")
        return switch, lane, add_path_unit(switch, lane)

    def test_the_lane_is_loaded_through_its_own_outlet_and_then_the_other(self):
        switch, lane, unit = self._switch()
        through = loading(switch, lane, Load(2104.0), Load(2251.0))
        gcmd = MockGCodeCommand()
        assert switch._measure(lane, gcmd) == {"A": 2104.0, "B": 2251.0}
        assert through == [("extruder", "A"), ("extruder1", "B")]
        assert switch.afc.TOOL_LOAD.call_args_list == [call(lane, gcmd=gcmd)] * 2
        unit.calibrate_bowden.assert_not_called()
        assert switch.logger.messages == []

    def test_the_lane_and_its_units_length_are_left_as_they_were(self):
        switch, lane, unit = self._switch()
        loading(switch, lane, Load(2104.0), Load(2251.0))
        switch._measure(lane, MockGCodeCommand())
        # The second load left the other outlet's path as the unit's own, so the one the
        # first load measured is put back where the unit saves it.
        assert tube_of(unit) == (2104.0, 2104.0)
        assert unit.kept == [(2104.0, 2104.0)]
        switch.afc.function.ConfigRewrite.assert_not_called()
        assert lane.tool_loaded is False
        assert lane.extruder_obj is extruder_of(switch, "extruder")
        assert switch._routed == {"A": None, "B": None}
        assert switch._offsets == {unit: 0.0}
        assert switch.confirmed == ["Bambu_AMS_1"]

    def test_each_load_sets_off_from_a_path_the_unit_does_not_know(self):
        # With both lengths set, a load through the other outlet has their difference added
        # to the unit's length first. A load that measures has not.
        switch, lane, unit = self._switch(**LENGTHS)
        loading(switch, lane, Load(2104.0), Load(2251.0))
        switch._measure(lane, MockGCodeCommand())
        assert unit.forgot == 2
        assert unit.began == [(3000.0, 3000.0), (3000.0, 3000.0)]
        assert switch._offsets == {unit: 0.0}
        assert switch._measuring is None

    def test_a_watch_that_comes_by_while_the_lane_is_in_a_toolhead_adds_nothing(self):
        switch, lane, unit = self._switch(**LENGTHS)
        loading(switch, lane, Load(2104.0), Load(2251.0))
        scripted = switch.afc.TOOL_UNLOAD.side_effect
        held: List[Tuple[Optional[FakeLane], Tuple[float, float]]] = []

        def watched(cur_lane: FakeLane, set_start_time: bool, force_unload: bool,
                    gcmd: Any) -> bool:
            switch._watch(0.0)
            held.append((switch._measuring, tube_of(unit)))
            return scripted(cur_lane, set_start_time, force_unload, gcmd)

        switch.afc.TOOL_UNLOAD = MagicMock(side_effect=watched)
        switch._measure(lane, MockGCodeCommand())
        assert held == [(lane, (2104.0, 2104.0)), (lane, (2251.0, 2251.0))]
        assert switch._offsets == {unit: 0.0}

    def test_a_second_load_that_measured_nothing_leaves_what_the_first_load_measured(self):
        # The unit forgot its path for that load, so the one the first load measured is put
        # back where the unit saves it.
        switch, lane, unit = self._switch(**LENGTHS)
        loading(switch, lane, Load(2104.0), Load(None))
        with pytest.raises(SwitchError) as err:
            switch._measure(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Bambu_AMS_1 did not measure the tube to extruder1 through outlet "
            "B. Bambu_AMS_1 measured no path at that load. Reset lane_Bambu_AMS_1 before the "
            "next try.")
        assert tube_of(unit) == (2104.0, 2104.0)
        assert unit.kept == [(2104.0, 2104.0)]
        assert switch._offsets == {unit: 0.0}
        switch._watch(0.0)
        assert tube_of(unit) == (2104.0, 2104.0)

    def test_a_lane_that_stays_in_the_other_toolhead_stays_on_its_extruder(self):
        switch, lane, unit = self._switch()
        loading(switch, lane, Load(2104.0), Load(2251.0, unloads=False))
        with pytest.raises(SwitchError) as err:
            switch._measure(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Bambu_AMS_1 did not measure the tube to extruder1 through outlet "
            "B. It could not be unloaded again. Reset lane_Bambu_AMS_1 before the next try.")
        assert lane.tool_loaded is True
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        assert tube_of(unit) == (2104.0, 2104.0)
        assert switch._routed == {"A": None, "B": None}
        # Still at outlet B, where its filament is.
        switch._watch(0.0)
        assert switch.state()["inlets"]["A"]["outlet"] == "B"

    def test_a_first_load_that_fails_stops_the_measurement_there(self):
        switch, lane, unit = self._switch()
        loading(switch, lane, Load(2104.0, loads=False))
        with pytest.raises(SwitchError) as err:
            switch._measure(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Bambu_AMS_1 did not measure the tube to extruder through outlet "
            "A. It could not be loaded. Reset lane_Bambu_AMS_1 before the next try.")
        assert switch.afc.TOOL_LOAD.call_count == 1
        switch.afc.TOOL_UNLOAD.assert_not_called()
        # The unit forgot its path for the load, so its own lengths are put back where it
        # saves them.
        assert tube_of(unit) == (2000.0, 1990.0)
        assert unit.kept == [(2000.0, 1990.0)]
        assert switch._measuring is None
        assert switch.confirmed == []


class TestAfcBridgeBoxSwitchMeasureOutlet:
    @staticmethod
    def _switch(chain: Optional[SimChain] = None) -> Tuple[afcBridgeBoxSwitch, FakeLane]:
        """:return tuple: a switch at rest, and Turtle_1's lane with a hub"""
        switch = plumbed(chain or SimChain(route=2))
        add_hub(lane_of(switch, "Turtle_1"))
        return switch, lane_of(switch, "Turtle_1")

    def test_the_lane_is_put_on_the_outlets_extruder_and_measured_through_it(self):
        switch, lane = self._switch()
        through = measuring(switch, lane, Run(1013))
        gcmd = MockGCodeCommand()
        length = switch._measure_outlet(lane, "A", "B", gcmd)
        assert (length, type(length)) == (1013.0, float)
        lane.unit_obj.calibrate_bowden.assert_called_once_with(lane, 25.0, 5.0, gcmd=gcmd)
        assert through == [("extruder1", "B")]
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        assert chain_of(switch).routes == [0]
        assert switch._routed == {"A": lane, "B": None}
        assert switch._seen == {"A": True, "B": False}
        assert switch._moving is False
        assert (switch_mod._MEASURE_STEP, switch_mod._MEASURE_TOLERANCE) == (25.0, 5.0)

    def test_the_outlet_the_lane_is_on_already_is_measured_where_it_is(self):
        switch, lane = self._switch()
        through = measuring(switch, lane, Run(852.5))
        assert switch._measure_outlet(lane, "A", "A", MockGCodeCommand()) == 852.5
        assert through == [("extruder", "A")]
        assert chain_of(switch).routes == []

    def test_the_inlet_is_kept_for_the_lane_and_the_switch_let_go_during_the_run(self):
        switch, lane = self._switch()
        during = []

        def calibrate_bowden(cur_lane: FakeLane, dis: float, tol: float,
                             gcmd: Any) -> Tuple[bool, str, float]:
            during.append((switch._routed["A"], switch._moving, dict(switch._seen)))
            return True, "ok", 1013.0

        lane.unit_obj.calibrate_bowden = MagicMock(side_effect=calibrate_bowden)
        # What an inlet read before the run does not count for it.
        switch._seen = {"A": True, "B": True}
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert during == [(lane, False, {"A": False, "B": False})]
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 reached extruder1 and inlet A never read filament. "
            "Neither inlet did. Check where Turtle_1 is plumbed.")
        assert switch._seen == {"A": False, "B": False}

    def test_filament_that_is_still_at_the_inlet_when_the_run_ends_counts(self):
        switch, lane = self._switch()
        measuring(switch, lane, Run(1013.0, watched=False, stays=True))
        assert switch._measure_outlet(lane, "A", "B", MockGCodeCommand()) == 1013.0
        assert switch._seen == {"A": True, "B": False}

    def test_filament_only_the_other_inlet_read_says_where_the_unit_looks_plumbed(self):
        switch, lane = self._switch()
        measuring(switch, lane, Run(1013.0, shows="B"))
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 reached extruder1 and inlet A never read filament. Inlet "
            "B did. Check where Turtle_1 is plumbed.")
        assert switch._seen == {"A": False, "B": True}

    @pytest.mark.parametrize("shows", ["A", None])
    def test_a_run_that_fails_says_why_whatever_its_length_and_the_inlet_say(self, shows):
        switch, lane = self._switch()
        measuring(switch, lane, Run(300.0, done=False, why="Failed Moving to hub after 300mm",
                                    shows=shows))
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 did not measure the tube to extruder1 through outlet B. "
            "Failed Moving to hub after 300mm. Reset lane_Turtle_1 before the next try.")

    def test_a_reason_that_ends_in_a_period_keeps_one(self):
        switch, lane = self._switch()
        measuring(switch, lane, Run(300.0, done=False, why="The buffer is already extended."))
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 did not measure the tube to extruder1 through outlet B. "
            "The buffer is already extended. Reset lane_Turtle_1 before the next try.")

    def test_a_run_that_fails_without_a_reason_leaves_none_out(self):
        switch, lane = self._switch()
        measuring(switch, lane, Run(0.0, done=False, why=""))
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 did not measure the tube to extruder1 through outlet B. "
            "Reset lane_Turtle_1 before the next try.")

    @pytest.mark.parametrize("moved", [300.0, -300.0])
    def test_a_run_that_fails_past_the_hub_offers_the_lane_reset(self, moved):
        switch, lane = self._switch()
        lane.hub_obj.state = True
        measuring(switch, lane, Run(moved, done=False, why="Error occurred"))
        gcmd = MockGCodeCommand()
        with patch.object(switch.afc, "run_script") as run_script, \
                pytest.raises(SwitchResetError) as err:
            switch._measure_outlet(lane, "A", "B", gcmd)
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 did not measure the tube to extruder1 through outlet B. "
            "Error occurred. Reset lane_Turtle_1 before the next try.")
        run_script.assert_called_once_with(
            "AFC_CALI_FAIL TITLE='Switch_1 Calibration Failed' FAIL=lane_Turtle_1 "
            "DISTANCE=300.0 RESET=1", gcmd=gcmd)

    @pytest.mark.parametrize("moved,at_hub", [(300.0, False), (0.0, True)])
    def test_a_run_that_fails_with_the_hub_clear_or_before_it_fed_offers_none(self, moved,
                                                                             at_hub):
        switch, lane = self._switch()
        lane.hub_obj.state = at_hub
        measuring(switch, lane, Run(moved, done=False, why="Error occurred"))
        with patch.object(switch.afc, "run_script") as run_script, \
                pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        # No prompt is left open, so it is not the error that keeps one.
        assert type(err.value) is SwitchError
        run_script.assert_not_called()

    def test_a_run_that_measures_offers_no_lane_reset(self):
        switch, lane = self._switch()
        lane.hub_obj.state = True
        measuring(switch, lane, Run(1013.0))
        with patch.object(switch.afc, "run_script") as run_script:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        run_script.assert_not_called()

    @pytest.mark.parametrize("length", [0, 0.0, -5.0])
    @pytest.mark.parametrize("shows", ["A", None])
    def test_a_run_that_is_done_without_a_length_measures_none(self, length, shows):
        switch, lane = self._switch()
        measuring(switch, lane, Run(length, why="calibration_lane", shows=shows))
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: the unit of lane_Turtle_1 measures no tube length. Use a lane of "
            "another unit, or set outlet_a_length and outlet_b_length by hand.")
        assert switch._lengthless == ["Turtle_1"]

    def test_a_unit_that_measures_no_length_is_noted_once(self):
        switch, lane = self._switch()
        measuring(switch, lane, Run(0), Run(0))
        for _try in range(2):
            with pytest.raises(SwitchError):
                switch._measure_outlet(lane, "A", "A", MockGCodeCommand())
        assert switch._lengthless == ["Turtle_1"]

    @pytest.mark.parametrize("run", [Run(1013.0), Run(0.0, done=False, why="Failed"),
                                     Run(1013.0, shows=None)])
    def test_a_unit_that_measured_failed_or_never_showed_is_not_noted(self, run):
        switch, lane = self._switch()
        measuring(switch, lane, run)
        try:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        except SwitchError:
            pass
        assert switch._lengthless == []

    def test_a_unit_that_measures_its_own_path_has_the_lane_loaded_instead(self):
        switch = plumbed(SimChain(route=2))
        lane = lane_of(switch, "Bambu_AMS_1")
        unit = add_path_unit(switch, lane)
        through = loading(switch, lane, Load(2251.0))
        gcmd = MockGCodeCommand()
        assert switch._measure_outlet(lane, "A", "B", gcmd) == 2251.0
        unit.calibrate_bowden.assert_not_called()
        switch.afc.TOOL_LOAD.assert_called_once_with(lane, gcmd=gcmd)
        assert through == [("extruder1", "B")]
        assert switch._seen == {"A": True, "B": False}
        assert switch._lengthless == []

    def test_a_load_that_did_not_measure_says_why(self):
        switch = plumbed(SimChain(route=2))
        lane = lane_of(switch, "Bambu_AMS_1")
        add_path_unit(switch, lane)
        loading(switch, lane, Load(2251.0, loads=False))
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Bambu_AMS_1 did not measure the tube to extruder1 through outlet "
            "B. It could not be loaded. Reset lane_Bambu_AMS_1 before the next try.")
        assert switch._lengthless == []

    def test_an_inlet_that_cannot_be_pointed_stops_before_the_unit_feeds(self):
        switch, lane = self._switch(SimChain(route=2, a=True))
        measuring(switch, lane, Run(1013.0))
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert str(err.value) == ("Switch_1: inlet A holds filament, so it cannot be moved to "
                                  "outlet B")
        lane.unit_obj.calibrate_bowden.assert_not_called()
        assert switch._moving is False


class TestAfcBridgeBoxSwitchOfferReset:
    @staticmethod
    def _switch(at_hub: bool) -> Tuple[afcBridgeBoxSwitch, FakeLane]:
        """:return tuple: a switch, and Turtle_1's lane with a hub that reads ``at_hub``"""
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        add_hub(lane).state = at_hub
        return switch, lane

    @pytest.mark.parametrize("moved", [300.0, -300.0, 12.34])
    def test_a_lane_that_fed_and_is_at_its_hub_gets_afcs_prompt(self, moved):
        switch, lane = self._switch(True)
        gcmd = MockGCodeCommand()
        with patch.object(switch.afc, "run_script") as run_script:
            assert switch._offer_reset(lane, moved, gcmd) is True
        distance = {300.0: "300.0", -300.0: "300.0", 12.34: "12.3"}[moved]
        run_script.assert_called_once_with(
            "AFC_CALI_FAIL TITLE='Switch_1 Calibration Failed' FAIL=lane_Turtle_1 "
            f"DISTANCE={distance} RESET=1", gcmd=gcmd)

    @pytest.mark.parametrize("moved", [0.0, 0])
    def test_a_lane_that_did_not_feed_gets_none(self, moved):
        switch, lane = self._switch(True)
        with patch.object(switch.afc, "run_script") as run_script:
            assert switch._offer_reset(lane, moved, MockGCodeCommand()) is False
        run_script.assert_not_called()

    def test_a_lane_whose_hub_is_clear_gets_none(self):
        switch, lane = self._switch(False)
        with patch.object(switch.afc, "run_script") as run_script:
            assert switch._offer_reset(lane, 300.0, MockGCodeCommand()) is False
        run_script.assert_not_called()

    def test_a_lane_with_no_hub_gets_none(self):
        switch, lane = self._switch(True)
        lane.hub_obj = None
        with patch.object(switch.afc, "run_script") as run_script:
            assert switch._offer_reset(lane, 300.0, MockGCodeCommand()) is False
        run_script.assert_not_called()


class TestAfcBridgeBoxSwitchLoadToMeasure:
    @staticmethod
    def _switch() -> Tuple[afcBridgeBoxSwitch, FakeLane, FakePathUnit]:
        """:return tuple: a switch at rest, Bambu_AMS_1's lane and its unit"""
        switch = plumbed(SimChain(route=2))
        lane = lane_of(switch, "Bambu_AMS_1")
        return switch, lane, add_path_unit(switch, lane)

    def test_the_lane_is_loaded_measured_and_unloaded_again(self):
        switch, lane, _unit = self._switch()
        through = loading(switch, lane, Load(2104))
        gcmd = MockGCodeCommand()
        answer = switch._load_to_measure(lane, "A", gcmd)
        assert answer == (True, "", 2104.0)
        assert type(answer[2]) is float
        switch.afc.TOOL_LOAD.assert_called_once_with(lane, gcmd=gcmd)
        switch.afc.TOOL_UNLOAD.assert_called_once_with(lane, set_start_time=False,
                                                       force_unload=True, gcmd=gcmd)
        assert through == [("extruder", "A")]
        assert lane.tool_loaded is False
        # Read while the lane was loaded: the inlet is empty again by now.
        assert switch._seen == {"A": True, "B": False}
        switch.reactor.pause.assert_not_called()
        assert switch.logger.messages == []

    def test_the_unit_forgets_the_path_it_knew_before_the_lane_is_loaded(self):
        switch, lane, unit = self._switch()
        loading(switch, lane, Load(2104.0))
        switch._load_to_measure(lane, "A", MockGCodeCommand())
        assert unit.forgot == 1
        assert unit.began == [(3000.0, 3000.0)]
        assert unit.kept == []

    def test_a_lane_that_does_not_load_is_not_unloaded(self):
        switch, lane, _unit = self._switch()
        loading(switch, lane, Load(2104.0, loads=False))
        assert switch._load_to_measure(lane, "A", MockGCodeCommand()) == (
            False, "It could not be loaded", 0.0)
        switch.afc.TOOL_UNLOAD.assert_not_called()
        assert switch._seen == {"A": False, "B": False}

    @pytest.mark.parametrize("path", [2104.0, None])
    def test_a_lane_that_does_not_unload_again_is_said_first(self, path):
        switch, lane, _unit = self._switch()
        loading(switch, lane, Load(path, unloads=False))
        assert switch._load_to_measure(lane, "A", MockGCodeCommand()) == (
            False, "It could not be unloaded again", 0.0)
        assert lane.tool_loaded is True
        # Its inlet still holds filament and is not waited for.
        switch.reactor.pause.assert_not_called()

    def test_a_load_that_measured_no_path_is_unloaded_and_says_so(self):
        switch, lane, _unit = self._switch()
        loading(switch, lane, Load(None))
        assert switch._load_to_measure(lane, "A", MockGCodeCommand()) == (
            False, "Bambu_AMS_1 measured no path at that load", 0.0)
        assert lane.tool_loaded is False
        switch.afc.TOOL_UNLOAD.assert_called_once()

    def test_the_inlet_is_given_time_to_read_empty_after_the_unload(self):
        switch, lane, _unit = self._switch()
        loading(switch, lane, Load(2104.0, lingers=True))
        assert switch._load_to_measure(lane, "A", MockGCodeCommand()) == (True, "", 2104.0)
        assert switch.reactor.pause.call_count == switch_mod._CLEAR_POLLS


class TestAfcBridgeBoxSwitchRestoreTube:
    def test_a_unit_that_keeps_its_own_length_saves_what_is_put_back(self):
        switch = make_switch()
        unit = FakePathUnit("Bambu_AMS_1", 2251.0, 2251.0)
        switch._restore_tube(unit, (2104.0, 2090.0))
        assert tube_of(unit) == (2104.0, 2090.0)
        assert unit.kept == [(2104.0, 2090.0)]
        switch.afc.function.ConfigRewrite.assert_not_called()

    def test_such_a_unit_that_holds_its_own_lengths_is_left_alone(self):
        switch = make_switch()
        unit = FakePathUnit("Bambu_AMS_1", 2104.0, 2090.0)
        switch._restore_tube(unit, (2104.0, 2090.0))
        assert unit.kept == []

    def test_a_tube_that_holds_its_own_lengths_is_left_alone(self):
        switch = make_switch()
        hub = FakeHub("Turtle_1")
        switch._restore_tube(hub, (900.0, 880.0))
        assert tube_of(hub) == (900.0, 880.0)
        switch.afc.function.ConfigRewrite.assert_not_called()

    @pytest.mark.parametrize("left", [(1013.0, 880.0), (900.0, 1013.0), (1013.0, 1013.0)])
    def test_lengths_another_outlet_left_are_put_back_and_saved(self, left):
        switch = make_switch()
        hub = FakeHub("Turtle_1", *left)
        switch._restore_tube(hub, (900.0, 880.0))
        assert tube_of(hub) == (900.0, 880.0)
        assert switch.afc.function.ConfigRewrite.call_args_list == [
            call("AFC_hub Turtle_1", "afc_bowden_length", 900.0),
            call("AFC_hub Turtle_1", "afc_unload_bowden_length", 880.0)]


def feeding(switch: afcBridgeBoxSwitch, unit: str, *shows: str,
            lingers: bool = False) -> MagicMock:
    """
    Script a unit's feed to the switch: the filament shows at the inlets ``shows`` names, is
    looked for once and is reeled back, or stays at the inlets when it ``lingers``.

    :return MagicMock: the unit's feed_to
    """
    chain = chain_of(switch)

    def feed_to(lane: FakeLane, seen: Any, gcmd: Any) -> bool:
        chain.fill(**{letter.lower(): True for letter in shows})
        saw = seen()
        if not lingers:
            chain.fill(a=False, b=False)
        return saw

    ams_of(switch, unit).feed_to = MagicMock(side_effect=feed_to)
    return ams_of(switch, unit).feed_to


class TestAfcBridgeBoxSwitchCmdAFCBridgeboxSwitchIdentify:
    @staticmethod
    def _finding(unit: str) -> List[Tuple[str, str]]:
        """:return list: the lines of the prompt that says a unit's own lane is fed"""
        return [("raw", "// action:prompt_begin Switch_1 Inlets"),
                ("raw", f"// action:prompt_text Finding the inlet of {unit}: lane_{unit} is fed "
                        "to the switch and reeled back. A lane that is loaded through the "
                        "switch is unloaded first."),
                SHOW]

    @staticmethod
    def _again(outcome: str, first: str, second: str) -> List[Tuple[str, str]]:
        """
        :return list: the lines of the units' prompt with an outcome, each unit `new` or the
        inlet found for it
        """
        menu = TestAfcBridgeBoxSwitchIdentifyMenu
        states = {"Bambu_AMS_1": first, "Bambu_AMS_2": second}
        buttons = [menu.button(unit, state, "primary" if state == "new" else "secondary")
                   for unit, state in states.items()]
        if (first, second) == ("new", "new"):
            buttons.append(menu.ALL_NEW)
        elif "new" not in (first, second):
            buttons.append(menu.ALL)
        lines = [("raw", f"// action:prompt_text {line}") for line in outcome.split("\n")]
        return [menu.BEGIN] + lines + [menu.PICK] + buttons + menu.FOOT

    def test_without_a_unit_or_a_lane_it_opens_the_menu(self):
        switch = finding()
        switch._identify = MagicMock()
        gcmd = MockGCodeCommand()
        switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        menu = TestAfcBridgeBoxSwitchIdentifyMenu
        assert switch.logger.messages == [
            menu.BEGIN, menu.PICK, menu.button("Bambu_AMS_1", "new", "primary"),
            menu.button("Bambu_AMS_2", "new", "primary"), menu.ALL_NEW] + menu.FOOT
        switch._identify.assert_not_called()
        gcmd.respond_info.assert_not_called()

    def test_with_a_unit_its_first_lane_that_holds_filament_is_fed(self):
        switch = finding()
        lane = lane_of(switch, "Bambu_AMS_2")
        lane.load_state = True

        def identify(fed: FakeLane, gcmd: Any) -> str:
            switch.inlets["B"].append("Bambu_AMS_2")
            return "B"

        switch._identify = MagicMock(side_effect=identify)
        gcmd = MockGCodeCommand({"UNIT": "Bambu_AMS_2"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        switch._identify.assert_called_once_with(lane, gcmd)
        said = "Switch_1: Bambu_AMS_2 enters inlet B. Its lanes start on extruder1."
        # What is being done takes the place of the offer, then the offer opens with the outcome.
        assert switch.logger.messages == self._finding("Bambu_AMS_2") + self._again(
            said, "new", "inlet B")
        gcmd.respond_info.assert_called_once_with(said)

    def test_with_a_lane_that_lane_is_fed_and_its_unit_is_the_one_found(self):
        switch = finding()
        lane = lane_of(switch, "Bambu_AMS_1")
        lane.load_state = True

        def identify(fed: FakeLane, gcmd: Any) -> str:
            switch.inlets["A"].append("Bambu_AMS_1")
            return "A"

        switch._identify = MagicMock(side_effect=identify)
        gcmd = MockGCodeCommand({"LANE": "lane_Bambu_AMS_1"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        switch._identify.assert_called_once_with(lane, gcmd)
        said = "Switch_1: Bambu_AMS_1 enters inlet A. Its lanes start on extruder."
        assert switch.logger.messages == self._finding("Bambu_AMS_1") + self._again(
            said, "inlet A", "new")
        gcmd.respond_info.assert_called_once_with(said)

    def test_a_lane_that_cannot_be_fed_is_the_commands_error(self):
        switch = finding()
        switch._identify = MagicMock()
        gcmd = MockGCodeCommand({"UNIT": "Bambu_AMS_2"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        said = "Switch_1: no lane of Bambu_AMS_2 holds filament. Put a spool in it first."
        assert str(err.value) == said
        switch._identify.assert_not_called()
        # No lane was fed, so the offer opens with the reason alone.
        assert switch.logger.messages == self._again(said, "new", "new")
        gcmd.respond_info.assert_not_called()

    def test_an_inlet_that_was_not_found_is_the_commands_error(self):
        switch = finding()
        lane_of(switch, "Bambu_AMS_2").load_state = True
        switch._identify = MagicMock(side_effect=SwitchError("Switch_1: it did not work"))
        gcmd = MockGCodeCommand({"UNIT": "Bambu_AMS_2"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        assert str(err.value) == "Switch_1: it did not work"
        assert switch.logger.messages == self._finding("Bambu_AMS_2") + self._again(
            "Switch_1: it did not work", "new", "new")
        gcmd.respond_info.assert_not_called()

    def test_any_other_failure_closes_the_prompt_and_is_passed_on(self):
        switch = finding()
        lane_of(switch, "Bambu_AMS_2").load_state = True
        switch._identify = MagicMock(side_effect=RuntimeError("the link dropped"))
        gcmd = MockGCodeCommand({"UNIT": "Bambu_AMS_2"})
        with pytest.raises(RuntimeError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        assert str(err.value) == "the link dropped"
        assert switch.logger.messages == self._finding("Bambu_AMS_2") + [END]
        gcmd.respond_info.assert_not_called()
        gcmd.error.assert_not_called()

    @staticmethod
    def _all(switch: afcBridgeBoxSwitch, fails: Optional[str] = None) -> MagicMock:
        """
        Script the run that finds an inlet: Bambu_AMS_1 enters A and Bambu_AMS_2 enters B, but
        for the unit that ``fails``. Both units hold filament.

        :return MagicMock: the switch's _identify
        """
        for unit in BAYS:
            lane_of(switch, unit).load_state = True

        def identify(fed: FakeLane, gcmd: Any) -> str:
            if fed.unit == fails:
                raise SwitchError(f"Switch_1: {fed.name} did not work")
            inlet = "A" if fed.unit == "Bambu_AMS_1" else "B"
            switch.inlets[inlet].append(fed.unit)
            return inlet

        switch._identify = MagicMock(side_effect=identify)
        return switch._identify

    FIRST = "Switch_1: Bambu_AMS_1 enters inlet A. Its lanes start on extruder."
    SECOND = "Switch_1: Bambu_AMS_2 enters inlet B. Its lanes start on extruder1."

    @pytest.mark.parametrize("unit", ["all", "ALL", "All"])
    def test_all_finds_every_unit_whose_inlet_is_not_known_one_after_the_other(self, unit):
        switch = finding()
        identify = self._all(switch)
        gcmd = MockGCodeCommand({"UNIT": unit})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        assert identify.call_args_list == [call(lane_of(switch, "Bambu_AMS_1"), gcmd),
                                           call(lane_of(switch, "Bambu_AMS_2"), gcmd)]
        second = self._finding("Bambu_AMS_2")
        # What the first unit gave stays above what is being done for the second.
        second.insert(1, ("raw", f"// action:prompt_text {self.FIRST}"))
        assert switch.logger.messages == self._finding("Bambu_AMS_1") + second + self._again(
            f"{self.FIRST}\n{self.SECOND}", "inlet A", "inlet B")
        assert gcmd.respond_info.call_args_list == [call(self.FIRST), call(self.SECOND)]

    def test_all_leaves_out_a_unit_whose_inlet_is_known(self):
        switch = finding()
        found(switch, Bambu_AMS_1="A")
        identify = self._all(switch)
        gcmd = MockGCodeCommand({"UNIT": "all"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        identify.assert_called_once_with(lane_of(switch, "Bambu_AMS_2"), gcmd)
        assert switch.logger.messages == self._finding("Bambu_AMS_2") + self._again(
            self.SECOND, "inlet A", "inlet B")
        gcmd.respond_info.assert_called_once_with(self.SECOND)

    def test_all_stops_at_the_first_unit_that_fails(self):
        switch = finding()
        identify = self._all(switch, fails="Bambu_AMS_1")
        gcmd = MockGCodeCommand({"UNIT": "all"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        said = "Switch_1: lane_Bambu_AMS_1 did not work"
        assert str(err.value) == said
        identify.assert_called_once_with(lane_of(switch, "Bambu_AMS_1"), gcmd)
        assert switch.logger.messages == self._finding("Bambu_AMS_1") + self._again(
            said, "new", "new")
        gcmd.respond_info.assert_not_called()

    def test_a_unit_that_fails_after_another_keeps_what_the_other_gave(self):
        switch = finding()
        identify = self._all(switch, fails="Bambu_AMS_2")
        gcmd = MockGCodeCommand({"UNIT": "all"})
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        said = "Switch_1: lane_Bambu_AMS_2 did not work"
        assert str(err.value) == said
        assert identify.call_count == 2
        second = self._finding("Bambu_AMS_2")
        second.insert(1, ("raw", f"// action:prompt_text {self.FIRST}"))
        assert switch.logger.messages == self._finding("Bambu_AMS_1") + second + self._again(
            f"{self.FIRST}\n{said}", "inlet A", "new")
        gcmd.respond_info.assert_called_once_with(self.FIRST)

    def test_all_with_a_lane_feeds_that_lane_alone(self):
        switch = finding()
        identify = self._all(switch)
        gcmd = MockGCodeCommand({"UNIT": "all", "LANE": "lane_Bambu_AMS_2"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        identify.assert_called_once_with(lane_of(switch, "Bambu_AMS_2"), gcmd)
        assert switch.logger.messages == self._finding("Bambu_AMS_2") + self._again(
            self.SECOND, "new", "inlet B")
        gcmd.respond_info.assert_called_once_with(self.SECOND)

    def test_all_on_a_switch_with_no_unit_to_find_opens_the_menu(self):
        switch = plumbed()
        switch._identify = MagicMock()
        gcmd = MockGCodeCommand({"UNIT": "all"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        menu = TestAfcBridgeBoxSwitchIdentifyMenu
        assert switch.logger.messages == [menu.BEGIN, menu.NO_UNIT] + TO_AFC
        switch._identify.assert_not_called()
        gcmd.respond_info.assert_not_called()

    def test_a_unit_is_found_from_start_to_end(self):
        chain = SimChain(route=2, a=True)
        switch = finding(chain)
        found(switch, Bambu_AMS_1="A")
        held = lane_of(switch, "Bambu_AMS_1")
        held.tool_loaded = True
        lane = lane_of(switch, "Bambu_AMS_2")
        lane.load_state = True

        def tool_unload(cur_lane: FakeLane, set_start_time: bool, force_unload: bool,
                        gcmd: Any) -> bool:
            cur_lane.tool_loaded = False
            chain.fill(a=False)
            return True

        switch.afc.TOOL_UNLOAD = MagicMock(side_effect=tool_unload)
        feed = feeding(switch, "Bambu_AMS_2", "B")
        gcmd = MockGCodeCommand({"UNIT": "Bambu_AMS_2"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(gcmd)
        switch.afc.TOOL_UNLOAD.assert_called_once_with(held, set_start_time=False,
                                                       force_unload=True, gcmd=gcmd)
        feed.assert_called_once_with(lane, switch._saw, gcmd)
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": ["Bambu_AMS_2"]}
        assert switch._found == {"AAAA": "A", "BBBB": "B"}
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        assert ams_of(switch, "Bambu_AMS_2").behind[-1] == ("Buffer_B", "chip_inlet_b")
        said = "Switch_1: Bambu_AMS_2 enters inlet B. Its lanes start on extruder1."
        assert switch.logger.messages == self._finding("Bambu_AMS_2") + [
            ("info", "Switch_1: unloading lane_Bambu_AMS_1 from extruder, both inlets have to "
                     "be empty to find one")] + self._again(said, "inlet A", "inlet B")
        gcmd.respond_info.assert_called_once_with(said)


class TestAfcBridgeBoxSwitchIdentifyUnits:
    FIRST = "Switch_1: Bambu_AMS_1 enters inlet A. Its lanes start on extruder."
    SECOND = "Switch_1: Bambu_AMS_2 enters inlet B. Its lanes start on extruder1."

    @staticmethod
    def _switch(fails: Optional[str] = None) -> afcBridgeBoxSwitch:
        """
        :return afcBridgeBoxSwitch: a switch whose units both hold filament, with the run that
        finds an inlet scripted: Bambu_AMS_1 enters A and Bambu_AMS_2 enters B, but for the
        unit that ``fails``
        """
        switch = finding()
        for unit in BAYS:
            lane_of(switch, unit).load_state = True

        def identify(fed: FakeLane, gcmd: Any) -> str:
            if fed.unit == fails:
                raise SwitchError(f"Switch_1: {fed.name} did not work")
            inlet = "A" if fed.unit == "Bambu_AMS_1" else "B"
            switch.inlets[inlet].append(fed.unit)
            return inlet

        switch._identify = MagicMock(side_effect=identify)
        return switch

    @staticmethod
    def _finding(unit: str, *done: str) -> List[Tuple[str, str]]:
        """
        :return list: the lines of the prompt that says a unit's own lane is fed, under what
        was ``done`` before
        """
        return ([("raw", "// action:prompt_begin Switch_1 Inlets")]
                + [("raw", f"// action:prompt_text {line}") for line in done]
                + [("raw", f"// action:prompt_text Finding the inlet of {unit}: lane_{unit} is "
                           "fed to the switch and reeled back. A lane that is loaded through "
                           "the switch is unloaded first."),
                   SHOW])

    def test_each_unit_is_found_in_turn_under_what_the_ones_before_it_gave(self):
        switch = self._switch()
        gcmd = MockGCodeCommand()
        done: List[str] = []
        switch._identify_units(list(BAYS), None, done, AFCprompt(gcmd, switch.logger), gcmd)
        assert switch._identify.call_args_list == [call(lane_of(switch, "Bambu_AMS_1"), gcmd),
                                                   call(lane_of(switch, "Bambu_AMS_2"), gcmd)]
        assert done == [self.FIRST, self.SECOND]
        assert gcmd.respond_info.call_args_list == [call(self.FIRST), call(self.SECOND)]
        assert switch.logger.messages == (self._finding("Bambu_AMS_1")
                                          + self._finding("Bambu_AMS_2", self.FIRST))

    def test_what_was_done_before_stays_above_what_is_being_done(self):
        switch = self._switch()
        gcmd = MockGCodeCommand()
        done = ["Switch_1: something else"]
        switch._identify_units(["Bambu_AMS_2"], None, done, AFCprompt(gcmd, switch.logger), gcmd)
        assert done == ["Switch_1: something else", self.SECOND]
        assert switch.logger.messages == self._finding("Bambu_AMS_2", "Switch_1: something else")

    def test_a_unit_that_fails_keeps_the_ones_after_it_from_being_found(self):
        switch = self._switch(fails="Bambu_AMS_1")
        gcmd = MockGCodeCommand()
        done: List[str] = []
        with pytest.raises(SwitchError) as err:
            switch._identify_units(list(BAYS), None, done, AFCprompt(gcmd, switch.logger), gcmd)
        assert str(err.value) == "Switch_1: lane_Bambu_AMS_1 did not work"
        switch._identify.assert_called_once_with(lane_of(switch, "Bambu_AMS_1"), gcmd)
        assert done == []
        gcmd.respond_info.assert_not_called()
        assert switch.logger.messages == self._finding("Bambu_AMS_1")

    def test_a_lane_that_is_named_is_fed_and_its_unit_is_the_one_found(self):
        switch = self._switch()
        gcmd = MockGCodeCommand()
        done: List[str] = []
        switch._identify_units([None], "lane_Bambu_AMS_2", done,
                               AFCprompt(gcmd, switch.logger), gcmd)
        switch._identify.assert_called_once_with(lane_of(switch, "Bambu_AMS_2"), gcmd)
        assert done == [self.SECOND]
        gcmd.respond_info.assert_called_once_with(self.SECOND)
        assert switch.logger.messages == self._finding("Bambu_AMS_2")

    def test_a_unit_with_no_lane_to_feed_opens_no_prompt(self):
        switch = self._switch()
        lane_of(switch, "Bambu_AMS_1").load_state = False
        gcmd = MockGCodeCommand()
        done: List[str] = []
        with pytest.raises(SwitchError) as err:
            switch._identify_units(["Bambu_AMS_1"], None, done, AFCprompt(gcmd, switch.logger),
                                   gcmd)
        assert str(err.value) == ("Switch_1: no lane of Bambu_AMS_1 holds filament. Put a spool "
                                  "in it first.")
        switch._identify.assert_not_called()
        assert done == []
        assert switch.logger.messages == []

    def test_with_no_unit_there_is_nothing_to_do(self):
        switch = self._switch()
        gcmd = MockGCodeCommand()
        done: List[str] = []
        switch._identify_units([], None, done, AFCprompt(gcmd, switch.logger), gcmd)
        switch._identify.assert_not_called()
        assert done == []
        gcmd.respond_info.assert_not_called()
        assert switch.logger.messages == []


class TestAfcBridgeBoxSwitchEntered:
    def test_it_names_the_extruder_the_units_lanes_start_on(self):
        switch = finding()
        found(switch, Bambu_AMS_2="B")
        assert switch._entered("Bambu_AMS_2", "B") == (
            "Switch_1: Bambu_AMS_2 enters inlet B. Its lanes start on extruder1.")

    def test_while_no_outlet_has_its_toolhead_it_names_the_inlet_alone(self):
        assert known()._entered("Bambu_AMS_1", "A") == "Chain_1: Bambu_AMS_1 enters inlet A."


class TestAfcBridgeBoxSwitchIdentifyMenu:
    BEGIN = ("raw", "// action:prompt_begin Switch_1 Inlets")
    PICK = ("raw", "// action:prompt_text Feeds one lane of a unit to the switch and reels it "
                   "back, to find the inlet the unit is plumbed into. A lane that is loaded "
                   "through the switch is unloaded first. Put a spool in the unit, then pick it.")
    NO_UNIT = ("raw", "// action:prompt_text No unit's inlet is found by loading. That is for "
                      "a unit or a chain that is on both inlet_a and inlet_b in "
                      f"[{SECTION}].")
    FOOT = TO_SWITCH
    ALL_NEW = ("raw", "// action:prompt_button All new units|AFC_BRIDGEBOX_SWITCH_IDENTIFY "
                      "SWITCH=Switch_1 UNIT=all|default")
    ALL = ("raw", "// action:prompt_button All units|AFC_BRIDGEBOX_SWITCH_IDENTIFY "
                  "SWITCH=Switch_1 UNIT=all|default")

    @staticmethod
    def button(unit: str, state: str, style: str) -> Tuple[str, str]:
        """:return tuple: the line of the button that finds a unit's inlet"""
        return ("raw", f"// action:prompt_button {unit}: {state}|AFC_BRIDGEBOX_SWITCH_IDENTIFY "
                       f"SWITCH=Switch_1 UNIT={unit}|{style}")

    def test_a_unit_is_new_until_its_inlet_is_found_and_names_it_after(self):
        switch = finding()
        found(switch, Bambu_AMS_2="B")
        switch._identify_menu(MockGCodeCommand())
        # One unit is new, so there is nothing to run all at once.
        assert switch.logger.messages == [
            self.BEGIN, self.PICK, self.button("Bambu_AMS_1", "new", "primary"),
            self.button("Bambu_AMS_2", "inlet B", "secondary")] + self.FOOT

    def test_several_new_units_are_also_offered_all_at_once(self):
        switch = finding()
        switch._identify_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK, self.button("Bambu_AMS_1", "new", "primary"),
            self.button("Bambu_AMS_2", "new", "primary"), self.ALL_NEW] + self.FOOT

    def test_with_every_inlet_known_all_units_are_offered_at_once(self):
        switch = finding()
        found(switch, Bambu_AMS_1="A", Bambu_AMS_2="B")
        switch._identify_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK, self.button("Bambu_AMS_1", "inlet A", "secondary"),
            self.button("Bambu_AMS_2", "inlet B", "secondary"), self.ALL] + self.FOOT

    def test_a_bay_that_holds_no_unit_now_is_not_offered(self):
        switch = finding()
        found(switch, Bambu_AMS_2="A")
        ams_of(switch, "Bambu_AMS_1").pool = True
        switch._identify_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK, self.button("Bambu_AMS_2", "inlet A", "secondary")] + self.FOOT

    def test_a_unit_that_is_named_on_an_inlet_is_not_offered(self):
        switch = finding(inlet_b=["Chain_1", "Bambu_AMS_2"])
        switch._identify_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK, self.button("Bambu_AMS_1", "new", "primary")] + self.FOOT

    def test_a_switch_with_no_unit_to_find_says_what_it_is_for(self):
        switch = plumbed()
        switch._identify_menu(MockGCodeCommand())
        # It has no prompt that offers what to calibrate, so it goes back to AFC's.
        assert switch.logger.messages == [self.BEGIN, self.NO_UNIT] + TO_AFC

    def test_the_outcome_of_the_last_run_is_shown_above_the_offer(self):
        switch = finding()
        found(switch, Bambu_AMS_2="B")
        switch._identify_menu(MockGCodeCommand(), "Switch_1: it did not work")
        assert switch.logger.messages == [
            self.BEGIN, ("raw", "// action:prompt_text Switch_1: it did not work"), self.PICK,
            self.button("Bambu_AMS_1", "new", "primary"),
            self.button("Bambu_AMS_2", "inlet B", "secondary")] + self.FOOT

    def test_and_above_what_it_is_for_on_a_switch_with_no_unit_to_find(self):
        switch = plumbed()
        switch._identify_menu(MockGCodeCommand(), "Switch_1: it did not work")
        assert switch.logger.messages == [
            self.BEGIN, ("raw", "// action:prompt_text Switch_1: it did not work"),
            self.NO_UNIT] + TO_AFC


class TestAfcBridgeBoxSwitchAll:
    def test_the_units_whose_inlet_is_not_known(self):
        assert finding()._all() == ["Bambu_AMS_1", "Bambu_AMS_2"]

    def test_a_unit_whose_inlet_is_known_is_left_out_while_another_is_not(self):
        switch = finding()
        found(switch, Bambu_AMS_1="B")
        assert switch._all() == ["Bambu_AMS_2"]

    def test_with_every_inlet_known_it_is_every_unit_that_is_found_by_loading(self):
        switch = finding()
        found(switch, Bambu_AMS_1="B", Bambu_AMS_2="A")
        assert switch._all() == ["Bambu_AMS_1", "Bambu_AMS_2"]

    @pytest.mark.parametrize("inlets", [{}, {"Bambu_AMS_2": "A"}])
    def test_a_bay_that_holds_no_unit_now_is_not_one(self, inlets):
        switch = finding()
        found(switch, **inlets)
        ams_of(switch, "Bambu_AMS_1").pool = True
        assert switch._all() == ["Bambu_AMS_2"]

    def test_a_unit_that_is_named_on_an_inlet_is_not_one(self):
        switch = finding(inlet_b=["Chain_1", "Bambu_AMS_2"])
        found(switch, Bambu_AMS_1="A")
        assert switch.live_units() == ["Bambu_AMS_1", "Bambu_AMS_2"]
        assert switch._all() == ["Bambu_AMS_1"]

    def test_a_switch_with_no_unit_that_is_found_by_loading_has_none(self):
        assert plumbed()._all() == []


class TestAfcBridgeBoxSwitchIdentifyLane:
    NOT_FOUND_BY_LOADING = ("is not found by loading. That is for a unit or a chain that is "
                            f"on both inlet_a and inlet_b in [{SECTION}].")

    @staticmethod
    def _switch(**values: Any) -> Tuple[afcBridgeBoxSwitch, List[FakeLane]]:
        """
        :return tuple: a switch whose Bambu_AMS_1 also has lane1 to lane3, and those lanes.
        Only lane2 and lane3 hold filament
        """
        switch = finding(connect=False, **values)
        lanes = [add_lane(switch.printer, name, "Bambu_AMS_1")
                 for name in ("lane1", "lane2", "lane3")]
        for lane in lanes[1:]:
            lane.load_state = True
        switch.handle_connect()
        return switch, lanes

    @staticmethod
    def _refused(switch: afcBridgeBoxSwitch, unit: Optional[str], name: Optional[str],
                 said: str) -> None:
        """Check no lane is picked, with ``said``."""
        with pytest.raises(SwitchError) as err:
            switch._identify_lane(unit, name)
        assert str(err.value) == f"Switch_1: {said}"

    def test_a_units_first_lane_that_holds_filament(self):
        switch, lanes = self._switch()
        # A lane of another unit that holds filament is not this unit's.
        lane_of(switch, "Bambu_AMS_2").load_state = True
        assert [lane.name for lane in switch.lanes][:3] == [
            "lane_Bambu_AMS_1", "lane_Bambu_AMS_2", "lane1"]
        assert switch._identify_lane("Bambu_AMS_1", None) is lanes[1]

    @pytest.mark.parametrize("unit", [None, "Bambu_AMS_1", "Bambu_AMS_2"])
    def test_a_lane_that_is_named_whatever_unit_is_named_with_it(self, unit):
        switch, lanes = self._switch()
        assert switch._identify_lane(unit, "lane3") is lanes[2]

    def test_a_lane_that_is_not_on_this_switch(self):
        switch, _lanes = self._switch()
        self._refused(switch, "Bambu_AMS_1", "lane9", "lane9 is not a lane of this switch")

    def test_a_unit_that_is_named_on_an_inlet(self):
        switch, _lanes = self._switch(inlet_b=["Chain_1", "Bambu_AMS_2"])
        lane_of(switch, "Bambu_AMS_2").load_state = True
        said = f"the inlet of Bambu_AMS_2 {self.NOT_FOUND_BY_LOADING}"
        self._refused(switch, "Bambu_AMS_2", None, said)
        self._refused(switch, None, "lane_Bambu_AMS_2", said)

    def test_a_unit_that_is_not_on_this_switch(self):
        switch, _lanes = self._switch()
        self._refused(switch, "Turtle_9", None,
                      f"the inlet of Turtle_9 {self.NOT_FOUND_BY_LOADING}")

    def test_a_bay_that_holds_no_unit_now(self):
        switch, _lanes = self._switch()
        ams_of(switch, "Bambu_AMS_1").pool = True
        self._refused(switch, "Bambu_AMS_1", None, "Bambu_AMS_1 holds no AMS now")

    def test_a_unit_whose_bay_names_no_ams(self):
        switch, _lanes = self._switch()
        chain_of(switch).uids["Bambu_AMS_1"] = None
        assert "Bambu_AMS_1" in switch.live_units()
        self._refused(switch, "Bambu_AMS_1", None, "Bambu_AMS_1 holds no AMS now")

    def test_a_unit_with_no_lane_that_holds_filament(self):
        switch, _lanes = self._switch()
        self._refused(switch, "Bambu_AMS_2", None,
                      "no lane of Bambu_AMS_2 holds filament. Put a spool in it first.")

    def test_a_named_lane_that_holds_no_filament(self):
        switch, _lanes = self._switch()
        self._refused(switch, None, "lane1", "lane1 holds no filament. Put a spool in it first.")

    def test_a_unit_on_no_chain_is_in_use_as_it_is(self):
        switch = sharing()
        lane = stepped(lane_of(switch, "Turtle_1"))
        lane.load_state = True
        assert switch._identify_lane("Turtle_1", None) is lane

    def test_a_lane_that_only_whole_loads_move(self):
        switch = sharing()
        lane_of(switch, "Turtle_1").load_state = True
        said = ("Turtle_1 only runs whole loads, so lane_Turtle_1 cannot be fed to a sensor and "
                f"reeled back. Name Turtle_1 on one inlet in [{SECTION}]: inlet_a or inlet_b.")
        self._refused(switch, "Turtle_1", None, said)
        self._refused(switch, None, "lane_Turtle_1", said)


class TestAfcBridgeBoxSwitchByLoading:
    def test_a_switch_with_a_section_of_its_own(self):
        assert plumbed()._by_loading() == (
            f"That is for a unit or a chain that is on both inlet_a and inlet_b in [{SECTION}].")

    def test_a_switch_its_chain_made(self):
        assert made()._by_loading() == (
            "That is for a unit or a chain that is on both switch_inlet_a and switch_inlet_b in "
            "[AFC_BridgeBox Chain_1].")


class TestAfcBridgeBoxSwitchFeeds:
    @staticmethod
    def _lane() -> Tuple[afcBridgeBoxSwitch, FakeLane]:
        """:return tuple: a switch and the lane of Turtle_1, which only whole loads move"""
        switch = plumbed()
        return switch, lane_of(switch, "Turtle_1")

    def test_a_lane_that_only_whole_loads_move_cannot_be_fed(self):
        switch, lane = self._lane()
        assert switch._feeds(lane) is False
        # Nor can a lane that has the place for a stepper and none in it.
        lane.extruder_stepper = None
        assert switch._feeds(lane) is False

    def test_a_unit_with_a_feed_of_its_own(self):
        switch, lane = self._lane()
        lane.unit_obj.feed_to = MagicMock()
        assert switch._feeds(lane) is True

    def test_a_lane_with_a_drive_stepper(self):
        switch, lane = self._lane()
        homed(lane)
        assert switch._feeds(lane) is True

    def test_a_lane_that_is_a_stepper_itself(self):
        switch, lane = self._lane()
        lane.extruder_stepper = MagicMock()
        assert switch._feeds(lane) is True

    def test_a_unit_with_no_stepper_that_moves_its_lanes_by_distance(self):
        switch, lane = self._lane()
        assert switch._feeds(stepped(lane)) is True

    def test_a_unit_with_no_stepper_and_no_move_of_its_own_cannot(self):
        switch, lane = self._lane()
        lane.unit_obj.stepperless_drive = True
        assert switch._feeds(lane) is False

    def test_a_move_of_its_own_does_not_count_on_a_unit_that_is_not_without_steppers(self):
        switch, lane = self._lane()
        lane.unit_obj.lane_move = MagicMock()
        assert switch._feeds(lane) is False


class TestAfcBridgeBoxSwitchWholeLoads:
    def test_it_names_the_unit_and_the_lane(self):
        switch = plumbed()
        assert switch._whole_loads(lane_of(switch, "Turtle_1")) == (
            "Switch_1: Turtle_1 only runs whole loads, so lane_Turtle_1 cannot be fed to a "
            "sensor and reeled back.")


class TestAfcBridgeBoxSwitchFeed:
    @staticmethod
    def _switch() -> Tuple[afcBridgeBoxSwitch, FakeLane, List[Any]]:
        """
        :return tuple: a switch whose two feeds with AFC's moves are stand-ins, the lane of
        Turtle_1, and a list that is filled with the check each stand-in ran under
        """
        switch = plumbed()
        under: List[Any] = []

        def home_to(lane: FakeLane, stepper: Any, gcmd: Any) -> float:
            under.append(switch._check)
            return 310.0

        def step_to(lane: FakeLane, gcmd: Any) -> float:
            under.append(switch._check)
            return 75.0

        switch._home_to = MagicMock(side_effect=home_to)
        switch._step_to = MagicMock(side_effect=step_to)
        return switch, lane_of(switch, "Turtle_1"), under

    def test_a_unit_with_a_feed_of_its_own_feeds_the_lane_itself(self):
        switch, lane, under = self._switch()
        # Its own feed comes before a stepper AFC could home the lane with.
        homed(lane)
        lane.unit_obj.feed_to = MagicMock(return_value=True)
        seen, gcmd = MagicMock(), MockGCodeCommand()
        assert switch._feed(lane, seen, gcmd) is None
        lane.unit_obj.feed_to.assert_called_once_with(lane, seen, gcmd)
        switch._home_to.assert_not_called()
        switch._step_to.assert_not_called()
        assert (under, switch._check) == ([], None)

    def test_a_lane_whose_stepper_can_home_to_the_check_is_fed_in_one_move(self):
        switch, lane, under = self._switch()
        stepper = homed(lane)
        seen, gcmd = MagicMock(), MockGCodeCommand()
        assert switch._feed(lane, seen, gcmd) == 310.0
        switch._home_to.assert_called_once_with(lane, stepper, gcmd)
        switch._step_to.assert_not_called()
        # The check is the feed's own while it runs, and none is left behind.
        assert (under, switch._check) == ([seen], None)

    def test_any_other_lane_is_fed_a_step_at_a_time(self):
        switch, lane, under = self._switch()
        seen, gcmd = MagicMock(), MockGCodeCommand()
        assert switch._feed(lane, seen, gcmd) == 75.0
        switch._step_to.assert_called_once_with(lane, gcmd)
        switch._home_to.assert_not_called()
        assert (under, switch._check) == ([seen], None)

    @pytest.mark.parametrize("with_stepper", [True, False])
    def test_a_feed_that_fails_leaves_no_check_behind(self, with_stepper):
        switch, lane, _under = self._switch()
        if with_stepper:
            homed(lane)
        switch._home_to.side_effect = SwitchError("Switch_1: it did not home")
        switch._step_to.side_effect = SwitchError("Switch_1: it did not step")
        with pytest.raises(SwitchError) as err:
            switch._feed(lane, MagicMock(), MockGCodeCommand())
        assert str(err.value) == ("Switch_1: it did not home" if with_stepper
                                  else "Switch_1: it did not step")
        assert switch._check is None


class TestAfcBridgeBoxSwitchStepper:
    @staticmethod
    def _lane() -> Tuple[afcBridgeBoxSwitch, FakeLane]:
        """:return tuple: a switch and the lane of Turtle_1, which has no stepper"""
        switch = plumbed()
        return switch, lane_of(switch, "Turtle_1")

    def test_the_drive_stepper_of_the_lane(self):
        switch, lane = self._lane()
        stepper = homed(lane)
        assert switch._stepper(lane) is stepper

    def test_a_lane_that_is_a_stepper_itself(self):
        switch, lane = self._lane()
        lane.add_polled_endstop = MagicMock()
        assert switch._stepper(lane) is lane

    def test_a_lane_with_no_stepper_has_none(self):
        switch, lane = self._lane()
        assert switch._stepper(lane) is None

    def test_a_stepper_that_takes_no_polled_endstop_does_not_count(self):
        switch, lane = self._lane()
        lane.drive_stepper = object()
        assert switch._stepper(lane) is None

    def test_while_afc_does_not_home_there_is_none(self):
        switch, lane = self._lane()
        homed(lane)
        switch.afc.homing_enabled = False
        assert switch._stepper(lane) is None


class TestAfcBridgeBoxSwitchChecked:
    @staticmethod
    def _watching(reads: bool) -> afcBridgeBoxSwitch:
        """
        :return afcBridgeBoxSwitch: a switch that watches the toolhead sensor of `extruder1`,
        which ``reads`` filament or does not
        """
        switch = sharing()
        switch._watched = {"extruder1": extruder_of(switch, "extruder1")}
        extruder_of(switch, "extruder1").tool_start_state = reads
        return switch

    def test_while_no_feed_runs_nothing_ends_one_whatever_a_sensor_reads(self):
        switch = self._watching(reads=True)
        assert switch._checked() is False
        assert switch._read == []

    def test_the_check_of_the_feed_saw_the_lane(self):
        switch = self._watching(reads=False)
        switch._check = MagicMock(return_value=True)
        assert switch._checked() is True
        assert switch._read == []

    def test_a_watched_toolhead_sensor_reads_the_lane(self):
        switch = self._watching(reads=True)
        switch._check = MagicMock(return_value=False)
        assert switch._checked() is True
        assert switch._read == ["extruder1"]

    def test_neither_saw_the_lane(self):
        switch = self._watching(reads=False)
        switch._check = MagicMock(return_value=False)
        assert switch._checked() is False
        switch._check.assert_called_once_with()
        assert switch._read == []


class TestAfcBridgeBoxSwitchLimit:
    def test_a_lane_with_no_tube_is_fed_past_its_way_to_the_hub(self):
        switch = plumbed()
        assert switch._limit(lane_of(switch, "Turtle_1")) == 60.0 + 500.0

    def test_the_tube_of_the_lane_is_added(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        add_hub(lane, length=900.0)
        assert switch._limit(lane) == 60.0 + 900.0 + 500.0

    def test_but_not_what_the_outlet_the_lane_goes_through_added_to_the_tube(self):
        switch = plumbed(**LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        hub = add_hub(lane, length=900.0)
        lane.set_extruder(extruder_of(switch, "extruder1"))
        switch._set_length(lane)
        # The other outlet's tube is 160 mm longer.
        assert tube_of(hub)[0] == 1060.0
        assert switch._limit(lane) == 60.0 + 900.0 + 500.0


class TestAfcBridgeBoxSwitchHomeTo:
    # The mm the lane is fed at the most: to its hub, through its tube and 500 mm past it.
    LIMIT = 60.0 + 900.0 + 500.0

    @staticmethod
    def _switch(moved: float = 310.0, warn: AFCMoveWarning = AFCMoveWarning.NONE
                ) -> Tuple[afcBridgeBoxSwitch, FakeLane, FakeStepper]:
        """
        :return tuple: a switch, the lane of Turtle_1, whose homing move ends after ``moved``
        mm with ``warn``, and the stepper the lane homes with
        """
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        add_hub(lane, length=900.0)
        lane.move_to.return_value = (moved > 0, moved, warn)
        return switch, lane, homed(lane)

    @staticmethod
    def _left_off(lane: FakeLane, gcmd: Any) -> None:
        """Check the lane's stepper was turned off and its unit sent home."""
        lane.do_enable.assert_called_once_with(False)
        lane.unit_obj.return_to_home.assert_called_once_with(gcmd=gcmd)

    def test_the_lane_is_fed_in_one_homing_move_and_pulled_back_as_far(self):
        switch, lane, stepper = self._switch()
        gcmd = MockGCodeCommand()
        assert switch._home_to(lane, stepper, gcmd) == 310.0
        lane.set_load_current.assert_called_once_with(gcmd=gcmd)
        lane.move_to.assert_called_once_with(self.LIMIT, SpeedMode.CALIBRATION,
                                             endstop="filament_switch", use_homing=True,
                                             gcmd=gcmd)
        lane.move_advanced.assert_called_once_with(-310.0, SpeedMode.LONG, AssistActive.YES,
                                                   gcmd=gcmd)
        self._left_off(lane, gcmd)

    def test_the_stepper_has_its_endstop_and_current_before_the_lane_moves(self):
        switch, lane, stepper = self._switch()
        before: List[Tuple[int, int, int]] = []

        def move_to(*args: Any, **kwargs: Any) -> Tuple[bool, float, AFCMoveWarning]:
            before.append((stepper.add_polled_endstop.call_count,
                           lane.set_load_current.call_count, lane.do_enable.call_count))
            return True, 310.0, AFCMoveWarning.NONE

        lane.move_to = MagicMock(side_effect=move_to)
        switch._home_to(lane, stepper, MockGCodeCommand())
        assert before == [(1, 1, 0)]

    def test_a_stepper_is_given_the_endstop_that_reads_the_check_once(self):
        switch, lane, stepper = self._switch()
        switch._home_to(lane, stepper, MockGCodeCommand())
        switch._home_to(lane, stepper, MockGCodeCommand())
        stepper.add_polled_endstop.assert_called_once_with("filament_switch", switch._checked)
        assert switch._homing == [stepper]
        assert lane.move_to.call_count == 2

    def test_each_stepper_is_given_its_own(self):
        switch, lane, stepper = self._switch()
        other_lane = lane_of(switch, "Ace2_1")
        other_stepper = homed(other_lane)
        other_lane.move_to.return_value = (True, 280.0, AFCMoveWarning.NONE)
        switch._home_to(lane, stepper, MockGCodeCommand())
        switch._home_to(other_lane, other_stepper, MockGCodeCommand())
        stepper.add_polled_endstop.assert_called_once_with("filament_switch", switch._checked)
        other_stepper.add_polled_endstop.assert_called_once_with("filament_switch",
                                                                 switch._checked)
        assert switch._homing == [stepper, other_stepper]

    def test_a_move_that_the_check_ended_short_of_its_limit_is_no_error(self):
        # AFC warns of a homing move that ends well before its distance, which is how one that
        # found its sensor ends here.
        switch, lane, stepper = self._switch(warn=AFCMoveWarning.WARN)
        assert switch._home_to(lane, stepper, MockGCodeCommand()) == 310.0

    def test_a_lane_that_did_not_move_is_not_pulled_back(self):
        switch, lane, stepper = self._switch(moved=0.0)
        gcmd = MockGCodeCommand()
        assert switch._home_to(lane, stepper, gcmd) == 0.0
        lane.move_advanced.assert_not_called()
        self._left_off(lane, gcmd)

    def test_a_move_that_failed_before_it_is_known_to_have_gone_is_an_error(self):
        switch, lane, stepper = self._switch(moved=0.0, warn=AFCMoveWarning.ERROR)
        gcmd = MockGCodeCommand()
        with pytest.raises(SwitchError) as err:
            switch._home_to(lane, stepper, gcmd)
        assert str(err.value) == (
            "Switch_1: the move that feeds lane_Turtle_1 to a sensor failed, and the lane was "
            "not pulled back. Run LANE_UNLOAD LANE=lane_Turtle_1 if its filament is out of its "
            "unit, then try again.")
        lane.move_advanced.assert_not_called()
        self._left_off(lane, gcmd)

    def test_a_move_that_failed_part_way_is_pulled_back_and_is_an_error(self):
        switch, lane, stepper = self._switch(moved=212.4, warn=AFCMoveWarning.ERROR)
        gcmd = MockGCodeCommand()
        with pytest.raises(SwitchError) as err:
            switch._home_to(lane, stepper, gcmd)
        assert str(err.value) == (
            "Switch_1: the move that feeds lane_Turtle_1 to a sensor failed, and the lane was "
            "pulled back the 212 mm it went. Run LANE_UNLOAD LANE=lane_Turtle_1 if its filament "
            "is out of its unit, then try again.")
        lane.move_advanced.assert_called_once_with(-212.4, SpeedMode.LONG, AssistActive.YES,
                                                   gcmd=gcmd)
        self._left_off(lane, gcmd)

    def test_a_move_that_raises_leaves_the_stepper_off_all_the_same(self):
        switch, lane, stepper = self._switch()
        lane.move_to.side_effect = CommandError("the stepper is lost")
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch._home_to(lane, stepper, gcmd)
        assert str(err.value) == "the stepper is lost"
        # Nothing is known to be fed, so nothing is pulled back.
        lane.move_advanced.assert_not_called()
        self._left_off(lane, gcmd)


class TestAfcBridgeBoxSwitchStepTo:
    @staticmethod
    def _switch(*checks: bool) -> Tuple[afcBridgeBoxSwitch, FakeLane]:
        """
        :return tuple: a switch whose running feed is checked with ``checks`` in turn and
        sees nothing after them, and the lane of Turtle_1
        """
        switch = plumbed()
        script = list(checks)
        switch._check = MagicMock(side_effect=lambda: script.pop(0) if script else False)
        return switch, lane_of(switch, "Turtle_1")

    @staticmethod
    def _step(mm: float, gcmd: Any) -> Any:
        """:return call: one step of a feed"""
        return call(mm, SpeedMode.CALIBRATION, AssistActive.NO, gcmd=gcmd)

    @staticmethod
    def _back(mm: float, gcmd: Any) -> Any:
        """:return call: the move that pulls a lane back"""
        return call(-mm, SpeedMode.LONG, AssistActive.YES, gcmd=gcmd)

    @staticmethod
    def _left_off(lane: FakeLane, gcmd: Any) -> None:
        """Check the lane's drive was turned off and its unit sent home."""
        lane.do_enable.assert_called_once_with(False)
        lane.unit_obj.return_to_home.assert_called_once_with(gcmd=gcmd)

    def test_the_lane_is_fed_a_step_at_a_time_until_the_check_saw_it_and_pulled_back(self):
        switch, lane = self._switch(False, False, True)
        gcmd = MockGCodeCommand()
        assert switch._step_to(lane, gcmd) == 75.0
        lane.set_load_current.assert_called_once_with(gcmd=gcmd)
        assert lane.move_advanced.call_args_list == [self._step(25.0, gcmd)] * 3 + [
            self._back(75.0, gcmd)]
        # A sensor is given a moment after each step.
        assert switch.reactor.pause.call_args_list == [call(100.0 + 0.2)] * 3
        self._left_off(lane, gcmd)

    def test_the_step_is_the_one_the_units_own_calibration_feeds_by(self):
        switch, lane = self._switch(False, True)
        lane.unit_obj.calibration_step = 40.0
        gcmd = MockGCodeCommand()
        assert switch._step_to(lane, gcmd) == 80.0
        assert lane.move_advanced.call_args_list == [self._step(40.0, gcmd)] * 2 + [
            self._back(80.0, gcmd)]

    def test_a_lane_no_check_saw_is_fed_to_its_limit_and_pulled_back(self):
        switch, lane = self._switch()
        # It is fed 560 mm at the most, which is three steps of 200 mm.
        lane.unit_obj.calibration_step = 200.0
        gcmd = MockGCodeCommand()
        assert switch._step_to(lane, gcmd) == 600.0
        assert lane.move_advanced.call_args_list == [self._step(200.0, gcmd)] * 3 + [
            self._back(600.0, gcmd)]
        assert switch._check.call_count == 3

    def test_a_first_step_that_fails_leaves_nothing_to_pull_back(self):
        switch, lane = self._switch()
        lane.move_advanced.side_effect = CommandError("the unit is gone")
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch._step_to(lane, gcmd)
        assert str(err.value) == "the unit is gone"
        assert lane.move_advanced.call_args_list == [self._step(25.0, gcmd)]
        self._left_off(lane, gcmd)

    def test_a_later_step_that_fails_has_what_was_fed_pulled_back(self):
        switch, lane = self._switch()
        lane.move_advanced.side_effect = [None, CommandError("the unit is gone"), None]
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError):
            switch._step_to(lane, gcmd)
        assert lane.move_advanced.call_args_list == [self._step(25.0, gcmd)] * 2 + [
            self._back(25.0, gcmd)]
        self._left_off(lane, gcmd)


class TestAfcBridgeBoxSwitchUnseen:
    @staticmethod
    def _unseen(fed: Optional[float], read: Optional[List[str]] = None) -> str:
        """:return str: why the lane of Turtle_1 was not read, after a feed of ``fed`` mm"""
        switch = plumbed()
        switch._read = list(read or [])
        return switch._unseen(lane_of(switch, "Turtle_1"), fed)

    # What is said of a lane its unit fed itself.
    BY_UNIT = ("Switch_1: lane_Turtle_1 was fed and neither inlet read filament. Put the spool "
               "of Turtle_1 in again so its feeder takes the filament, and check that the unit "
               "is plumbed into the switch.")

    def test_a_lane_its_unit_fed_itself(self):
        assert self._unseen(None) == self.BY_UNIT

    def test_what_a_toolhead_read_is_not_said_of_such_a_lane(self):
        assert self._unseen(None, ["extruder1"]) == self.BY_UNIT

    @pytest.mark.parametrize("read, names", [
        (["extruder1"], "extruder1"), (["extruder", "extruder1"], "extruder and extruder1")])
    def test_a_lane_afc_fed_as_far_as_a_toolhead(self, read, names):
        assert self._unseen(430.0, read) == (
            f"Switch_1: lane_Turtle_1 reached the toolhead sensor of {names} and neither inlet "
            "read filament. Check that Turtle_1 is plumbed into the switch.")

    @pytest.mark.parametrize("fed, said", [(1460.0, "1460"), (0.0, "0")])
    def test_a_lane_afc_fed_that_nothing_read(self, fed, said):
        assert self._unseen(fed) == (
            f"Switch_1: lane_Turtle_1 was fed {said} mm and neither inlet read filament. Check "
            "that Turtle_1 is plumbed into the switch, and the tube length of Turtle_1: a lane "
            "is fed 500 mm past it at the most.")


class TestAfcBridgeBoxSwitchIdentify:
    @staticmethod
    def _switch(*shows: str) -> Tuple[afcBridgeBoxSwitch, FakeLane, MagicMock]:
        """
        :return tuple: a switch at rest, the lane of Bambu_AMS_1, which holds filament, and
        the unit's feed, at which the filament shows at the inlets ``shows`` names
        """
        switch = finding(SimChain(route=2))
        lane = lane_of(switch, "Bambu_AMS_1")
        lane.load_state = True
        return switch, lane, feeding(switch, "Bambu_AMS_1", *shows)

    @pytest.mark.parametrize("inlet, start, behind", [
        ("A", "extruder", ("Buffer", "chip")), ("B", "extruder1", ("Buffer_B", "chip_inlet_b"))])
    def test_the_inlet_that_read_the_lane_becomes_its_units(self, inlet, start, behind):
        switch, lane, feed = self._switch(inlet)
        lane.set_extruder(extruder_of(switch, other_extruder(start)))
        gcmd = MockGCodeCommand()
        assert switch._identify(lane, gcmd) == inlet
        feed.assert_called_once_with(lane, switch._saw, gcmd)
        assert switch._found == {"AAAA": inlet}
        assert switch.inlets == {"A": [], "B": [], inlet: ["Bambu_AMS_1"]}
        assert lane.extruder_obj.name == start
        assert ams_of(switch, "Bambu_AMS_1").behind[-1] == behind
        assert switch.confirmed == ["Bambu_AMS_1"]
        switch.afc.save_vars.assert_called_once_with()
        assert switch._moving is False

    def test_the_inlets_are_emptied_and_forgotten_before_the_lane_is_fed(self):
        switch, lane, feed = self._switch("B")
        switch._seen = {"A": True, "B": True}
        order = []
        switch._clear = MagicMock(side_effect=lambda gcmd: order.append(
            ("clear", switch._moving, feed.call_count)))
        gcmd = MockGCodeCommand()
        # Inlet A read filament before the feed, and only what the feed shows counts.
        assert switch._identify(lane, gcmd) == "B"
        switch._clear.assert_called_once_with(gcmd)
        assert order == [("clear", True, 0)]
        assert switch._seen == {"A": False, "B": True}

    def test_the_inlet_that_read_it_is_given_time_to_read_empty_again(self):
        switch, lane, _feed = self._switch("B")
        switch._clear = MagicMock()
        switch._wait_clear = MagicMock()
        switch._identify(lane, MockGCodeCommand())
        assert switch._wait_clear.call_args_list == [call("B")]

    def test_the_ams_in_the_bay_when_the_lane_was_fed_is_the_one_found(self):
        switch, lane, _feed = self._switch("B")
        chain_of(switch).uids["Bambu_AMS_1"] = "CCCC"
        switch._identify(lane, MockGCodeCommand())
        assert switch._found == {"CCCC": "B"}
        assert switch._held == {"Bambu_AMS_1": "CCCC", "Bambu_AMS_2": "BBBB"}

    def test_a_unit_that_is_found_again_at_its_inlet_is_left_as_it_is(self):
        switch, lane, _feed = self._switch("B")
        found(switch, Bambu_AMS_1="B")
        switch.confirmed = ["Bambu_AMS_1"]
        unit = ams_of(switch, "Bambu_AMS_1")
        unit.behind.clear()
        assert switch._identify(lane, MockGCodeCommand()) == "B"
        assert unit.behind == []
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_1"]}
        assert switch.confirmed == ["Bambu_AMS_1"]
        switch.afc.save_vars.assert_called_once_with()

    def test_a_unit_that_is_found_at_the_other_inlet_moves_there(self):
        switch, lane, _feed = self._switch("B")
        found(switch, Bambu_AMS_1="A")
        assert switch._identify(lane, MockGCodeCommand()) == "B"
        assert switch._found == {"AAAA": "B"}
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_1"]}
        assert lane.extruder_obj.name == "extruder1"

    def test_a_lane_that_is_still_in_the_inlet_it_found_is_an_error(self):
        switch, lane, _feed = self._switch()
        feed = feeding(switch, "Bambu_AMS_1", "B", lingers=True)
        with pytest.raises(SwitchError) as err:
            switch._identify(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: Bambu_AMS_1 enters inlet B, but lane_Bambu_AMS_1 was not reeled back "
            "and is still in it. Run LANE_UNLOAD LANE=lane_Bambu_AMS_1, then go on.")
        assert feed.call_count == 1
        # The inlet was given its time to read empty.
        assert switch.reactor.pause.call_count == switch_mod._CLEAR_POLLS
        # What was found is kept.
        assert switch._found == {"AAAA": "B"}
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_1"]}
        assert lane.extruder_obj.name == "extruder1"
        assert switch.confirmed == ["Bambu_AMS_1"]
        switch.afc.save_vars.assert_called_once_with()
        assert switch._moving is False

    def test_filament_in_the_inlet_that_did_not_read_the_lane_is_not_its_error(self):
        switch, lane, _feed = self._switch("B")
        chain = chain_of(switch)
        fed = ams_of(switch, "Bambu_AMS_1").feed_to.side_effect

        def feed_to(lane: FakeLane, seen: Any, gcmd: Any) -> bool:
            saw = fed(lane, seen, gcmd)
            # Something enters inlet A once the lane is back.
            chain.fill(a=True)
            return saw
        ams_of(switch, "Bambu_AMS_1").feed_to = MagicMock(side_effect=feed_to)
        assert switch._identify(lane, MockGCodeCommand()) == "B"
        assert switch._found == {"AAAA": "B"}

    def _not_found(self, switch: afcBridgeBoxSwitch, lane: FakeLane, said: str) -> None:
        """Check no inlet is found, with ``said``, and the switch is left as it was."""
        with pytest.raises(SwitchError) as err:
            switch._identify(lane, MockGCodeCommand())
        assert str(err.value) == f"Switch_1: {said}"
        assert switch._found == {}
        assert switch.inlets == {"A": [], "B": []}
        assert switch.confirmed == []
        assert lane.extruder_obj.name == "extruder"
        switch.afc.save_vars.assert_not_called()
        assert switch._moving is False

    def test_a_lane_that_neither_inlet_read(self):
        switch, lane, feed = self._switch()
        self._not_found(switch, lane,
                        "lane_Bambu_AMS_1 was fed and neither inlet read filament. Put the "
                        "spool of Bambu_AMS_1 in again so its feeder takes the filament, and "
                        "check that the unit is plumbed into the switch.")
        assert feed.call_count == 1

    def test_a_lane_that_both_inlets_read(self):
        switch, lane, feed = self._switch("A", "B")
        self._not_found(switch, lane,
                        "both inlets read filament while lane_Bambu_AMS_1 was fed, so the inlet "
                        "of Bambu_AMS_1 is not known. Check that nothing else is in the switch.")
        assert feed.call_count == 1

    def test_while_a_print_runs_nothing_is_fed(self):
        switch, lane, feed = self._switch("B")
        switch._clear = MagicMock()
        switch.afc.function.in_print.return_value = True
        self._not_found(switch, lane, "the inlet of Bambu_AMS_1 is not found while a print runs")
        switch._clear.assert_not_called()
        feed.assert_not_called()

    def test_inlets_that_cannot_be_emptied_keep_the_lane_from_being_fed(self):
        switch, lane, feed = self._switch("B")
        switch._clear = MagicMock(side_effect=SwitchError("Switch_1: it is full"))
        self._not_found(switch, lane, "it is full")
        feed.assert_not_called()

    @staticmethod
    def _shared() -> Tuple[afcBridgeBoxSwitch, FakeLane]:
        """
        :return tuple: a switch at rest with Turtle_1 on both inlets, and that unit's lane,
        which holds filament and which AFC moves by distance
        """
        switch = sharing()
        lane = stepped(lane_of(switch, "Turtle_1"))
        lane.load_state = True
        return switch, lane

    def test_a_unit_on_no_chain_is_fed_with_afcs_moves_and_kept_for_its_name(self):
        switch, lane = self._shared()
        moves = stepping(switch, lane, inlet="B")
        gcmd = MockGCodeCommand()
        assert switch._identify(lane, gcmd) == "B"
        step = call(25.0, SpeedMode.CALIBRATION, AssistActive.NO, gcmd=gcmd)
        assert moves.call_args_list == [step, step, step,
                                        call(-75.0, SpeedMode.LONG, AssistActive.YES, gcmd=gcmd)]
        assert switch._found == {"Turtle_1": "B"}
        assert switch._held == {"Turtle_1": "Turtle_1"}
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": ["Ace2_1", "Turtle_1"]}
        assert lane.extruder_obj.name == "extruder1"
        assert switch.confirmed == ["Turtle_1"]
        switch.afc.save_vars.assert_called_once_with()
        assert (switch._moving, switch._check) == (False, None)

    def test_a_lane_a_stepper_homes_is_fed_in_one_move_that_its_inlet_ends(self):
        switch, lane = self._shared()
        stepper = homed(lane)
        chain = chain_of(switch)
        triggered: List[bool] = []

        def move_to(*args: Any, **kwargs: Any) -> Tuple[bool, float, AFCMoveWarning]:
            # The endstop the stepper homes to, read before and once the filament is there.
            trigger = stepper.add_polled_endstop.call_args.args[1]
            triggered.append(trigger())
            chain.fill(b=True)
            triggered.append(trigger())
            return True, 640.0, AFCMoveWarning.WARN

        lane.move_to = MagicMock(side_effect=move_to)
        lane.move_advanced = MagicMock(side_effect=lambda *args, **kwargs: chain.fill(b=False))
        gcmd = MockGCodeCommand()
        assert switch._identify(lane, gcmd) == "B"
        assert triggered == [False, True]
        lane.move_advanced.assert_called_once_with(-640.0, SpeedMode.LONG, AssistActive.YES,
                                                   gcmd=gcmd)
        assert switch._found == {"Turtle_1": "B"}
        # Once the feed is over the endstop reads nothing, whatever is in an inlet.
        chain.fill(b=True)
        assert stepper.add_polled_endstop.call_args.args[1]() is False

    def test_a_lane_afcs_moves_fed_as_far_as_a_toolhead_is_said_to_have_reached_it(self):
        switch, lane = self._shared()
        moves = stepping(switch, lane, toolhead="extruder", after=2)
        with pytest.raises(SwitchError) as err:
            switch._identify(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 reached the toolhead sensor of extruder and neither inlet "
            "read filament. Check that Turtle_1 is plumbed into the switch.")
        # It was fed no further than the toolhead, and pulled back.
        assert [move.args[0] for move in moves.call_args_list] == [25.0, 25.0, -50.0]
        assert switch._found == {}

    def test_the_toolhead_sensors_are_watched_afresh_while_the_lane_is_fed(self):
        switch, lane = self._shared()
        switch._read = ["extruder1"]
        during: List[Tuple[List[str], List[str]]] = []

        def feed(lane: FakeLane, seen: Any, gcmd: Any) -> float:
            during.append((list(switch._watched), list(switch._read)))
            return 560.0

        switch._feed = MagicMock(side_effect=feed)
        gcmd = MockGCodeCommand()
        with pytest.raises(SwitchError) as err:
            switch._identify(lane, gcmd)
        switch._feed.assert_called_once_with(lane, switch._saw, gcmd)
        assert during == [(["extruder", "extruder1"], [])]
        assert str(err.value) == (
            "Switch_1: lane_Turtle_1 was fed 560 mm and neither inlet read filament. Check "
            "that Turtle_1 is plumbed into the switch, and the tube length of Turtle_1: a lane "
            "is fed 500 mm past it at the most.")


class TestAfcBridgeBoxSwitchClear:
    @staticmethod
    def _loaded(chain: SimChain) -> Tuple[afcBridgeBoxSwitch, List[FakeLane]]:
        """
        :return tuple: a switch with both its lanes in a toolhead, lane_Bambu_AMS_1 of a unit
        whose inlet is not known and lane_Bambu_AMS_2 through inlet B, and the lanes
        """
        switch = finding(chain)
        found(switch, Bambu_AMS_2="B")
        lanes = [lane_of(switch, unit) for unit in BAYS]
        for lane in lanes:
            lane.tool_loaded = True
        return switch, lanes

    def test_with_no_lane_loaded_and_both_inlets_empty_there_is_nothing_to_do(self):
        switch = finding(SimChain(route=2))
        switch.afc.TOOL_UNLOAD = MagicMock()
        switch._clear(MockGCodeCommand())
        switch.afc.TOOL_UNLOAD.assert_not_called()
        switch.reactor.pause.assert_not_called()
        assert switch.logger.messages == []

    def test_every_lane_that_is_loaded_through_the_switch_is_unloaded(self):
        chain = SimChain(route=0, a=True, b=True)
        switch, lanes = self._loaded(chain)

        def tool_unload(cur_lane: FakeLane, set_start_time: bool, force_unload: bool,
                        gcmd: Any) -> bool:
            cur_lane.tool_loaded = False
            chain.fill(**{"a" if cur_lane is lanes[0] else "b": False})
            return True

        switch.afc.TOOL_UNLOAD = MagicMock(side_effect=tool_unload)
        gcmd = MockGCodeCommand()
        switch._clear(gcmd)
        assert switch.afc.TOOL_UNLOAD.call_args_list == [
            call(lanes[0], set_start_time=False, force_unload=True, gcmd=gcmd),
            call(lanes[1], set_start_time=False, force_unload=True, gcmd=gcmd)]
        assert switch.logger.messages == [
            ("info", "Switch_1: unloading lane_Bambu_AMS_1 from extruder, both inlets have to "
                     "be empty to find one"),
            ("info", "Switch_1: unloading lane_Bambu_AMS_2 from extruder1, both inlets have to "
                     "be empty to find one")]

    def test_a_lane_that_does_not_unload_stops_it(self):
        switch, lanes = self._loaded(SimChain(route=0, a=True, b=True))
        switch.afc.TOOL_UNLOAD = MagicMock(return_value=False)
        with pytest.raises(SwitchError) as err:
            switch._clear(MockGCodeCommand())
        assert str(err.value) == ("Switch_1: lane_Bambu_AMS_1 could not be unloaded from "
                                  "extruder. Unload it, then try again.")
        # The other lane is left in its toolhead.
        assert switch.afc.TOOL_UNLOAD.call_count == 1
        assert switch.logger.messages == [
            ("info", "Switch_1: unloading lane_Bambu_AMS_1 from extruder, both inlets have to "
                     "be empty to find one")]

    @pytest.mark.parametrize("a, b, full", [
        (True, False, "A"), (False, True, "B"), (True, True, "A and B")])
    def test_filament_that_stays_in_an_inlet_stops_it(self, a, b, full):
        switch = finding(SimChain(route=2, a=a, b=b))
        with pytest.raises(SwitchError) as err:
            switch._clear(MockGCodeCommand())
        assert str(err.value) == (f"Switch_1: filament is still in inlet {full}. Run "
                                  "LANE_UNLOAD on the lane that is in it, then try again.")
        # Each inlet that reads filament was given its time to read empty.
        assert switch.reactor.pause.call_count == switch_mod._CLEAR_POLLS * (a + b)

    def test_an_inlet_is_given_time_to_read_empty(self):
        chain = SimChain(route=2, b=True)
        switch = finding(chain)
        switch.reactor.pause.side_effect = lambda until: chain.fill(b=False)
        switch._clear(MockGCodeCommand())
        assert switch.reactor.pause.call_count == 1


class TestAfcBridgeBoxSwitchSaw:
    def test_no_inlet_read_filament(self):
        switch = plumbed(SimChain(route=2))
        assert switch._saw() is False
        assert switch._seen == {"A": False, "B": False}

    def test_an_inlet_that_reads_filament_now(self):
        switch = plumbed(SimChain(route=2, b=True))
        assert switch._saw() is True
        assert switch._seen == {"A": False, "B": True}

    def test_an_inlet_that_read_filament_since_it_was_last_cleared(self):
        switch = plumbed(SimChain(route=2))
        switch._seen["A"] = True
        assert switch._saw() is True


def setting_up(switch: afcBridgeBoxSwitch, inlets: Dict[str, Optional[str]],
               toolheads: Dict[str, List[str]]) -> MagicMock:
    """
    Script every unit's feed for a setup. A lane of a unit whose inlet is not known shows at
    the inlet ``inlets`` names for the unit, at neither for None. A lane of any other unit goes
    on through its inlet to the toolheads ``toolheads`` names for the outlet that inlet is at.
    Each lane is looked for once and reeled back.

    :return MagicMock: the feed_to of every unit
    """
    chain = chain_of(switch)

    def feed_to(lane: FakeLane, seen: Any, gcmd: Any) -> bool:
        shows = switch.inlet_of(lane.unit)
        reached: List[str] = []
        if shows is None:
            shows = inlets[lane.unit]
        else:
            reached = toolheads.get(switch.state()["inlets"][shows]["outlet"], [])
        if shows is not None:
            chain.fill(**{shows.lower(): True})
        for name in reached:
            extruder_of(switch, name).tool_start_state = True
        saw = seen()
        for name in reached:
            extruder_of(switch, name).tool_start_state = False
        chain.fill(a=False, b=False)
        return saw

    feed = MagicMock(side_effect=feed_to)
    for unit in BAYS:
        ams_of(switch, unit).feed_to = feed
    return feed


class TestAfcBridgeBoxSwitchCmdAFCBridgeboxSwitchSetup:
    NOTHING = ("Chain_1: every inlet and the toolhead of an outlet are known, so there is "
               "nothing to set up.")
    NOT_READ = ("Chain_1: lane_Bambu_AMS_1 was fed and neither inlet read filament. Put the "
                "spool of Bambu_AMS_1 in again so its feeder takes the filament, and check that "
                "the unit is plumbed into the switch.")
    OUTLET_A = "Chain_1: outlet A feeds extruder1."
    OUTLET_B = "Chain_1: outlet B feeds extruder."
    WHAT = ("Inlets finds the inlet each unit is plumbed into, by feeding one of its lanes to "
            "the switch. Outlets measures the tube from each outlet to its toolhead, with a "
            "lane of a unit whose inlet is known.")
    BUTTONS = [("raw", "// action:prompt_button Inlets|AFC_BRIDGEBOX_SWITCH_IDENTIFY "
                       "SWITCH=Chain_1|primary"),
               ("raw", "// action:prompt_button Outlets|AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                       "SWITCH=Chain_1 OUTLETS=1|secondary")]

    @staticmethod
    def _stubbed(switch: afcBridgeBoxSwitch, fails: Tuple[str, ...] = (),
                 error: Any = SwitchError) -> afcBridgeBoxSwitch:
        """
        Put a script in place of the steps of a setup: finding a unit's inlet gives
        `<unit> found` and finding the toolheads gives `toolheads found`, but for the steps
        ``fails`` names, which raise ``error`` with `<step> failed`.

        :return afcBridgeBoxSwitch: ``switch``, its prompt stubbed too
        """
        def identify_units(units: List[str], name: Optional[str], done: List[str],
                           prompt: AFCprompt, gcmd: Any) -> None:
            if units[0] in fails:
                raise error(f"{units[0]} failed")
            done.append(f"{units[0]} found")

        def find_toolheads(name: Optional[str], done: List[str], prompt: AFCprompt,
                           gcmd: Any) -> None:
            if "toolheads" in fails:
                raise error("toolheads failed")
            done.append("toolheads found")

        switch._identify_units = MagicMock(side_effect=identify_units)
        switch._find_toolheads = MagicMock(side_effect=find_toolheads)
        switch._open_menu = MagicMock()
        return switch

    @staticmethod
    def _ran(switch: afcBridgeBoxSwitch) -> Tuple[List[Any], List[Any]]:
        """
        :return tuple: the units and the lane each run that finds an inlet was given, and the
        lane each run that finds the toolheads was given
        """
        return ([step.args[:2] for step in switch._identify_units.call_args_list],
                [step.args[0] for step in switch._find_toolheads.call_args_list])

    BOTH_UNITS = [(["Bambu_AMS_1"], None), (["Bambu_AMS_2"], None)]

    @pytest.mark.parametrize("params", [{}, {"OUTLETS": 0}])
    def test_each_unit_whose_inlet_is_not_known_then_the_toolheads(self, params):
        switch = self._stubbed(made())
        gcmd = MockGCodeCommand(params)
        switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert self._ran(switch) == (self.BOTH_UNITS, [None])
        switch._open_menu.assert_called_once_with(
            gcmd, False, "Bambu_AMS_1 found\nBambu_AMS_2 found\ntoolheads found")
        gcmd.error.assert_not_called()
        assert switch.logger.messages == []

    def test_every_step_is_given_one_list_one_prompt_and_the_command(self):
        switch = self._stubbed(made())
        gcmd = MockGCodeCommand()
        switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        first, second = [step.args for step in switch._identify_units.call_args_list]
        last = switch._find_toolheads.call_args.args
        assert first[2] is second[2]
        assert first[2] is last[1]
        assert isinstance(first[3], AFCprompt)
        assert second[3] is first[3]
        assert last[2] is first[3]
        assert (first[4], second[4], last[3]) == (gcmd, gcmd, gcmd)

    def test_a_unit_whose_inlet_is_known_is_left_out(self):
        switch = made()
        found(switch, Bambu_AMS_1="A")
        self._stubbed(switch)
        gcmd = MockGCodeCommand()
        switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert self._ran(switch) == ([(["Bambu_AMS_2"], None)], [None])
        switch._open_menu.assert_called_once_with(gcmd, False,
                                                  "Bambu_AMS_2 found\ntoolheads found")

    def test_a_unit_that_is_not_found_does_not_keep_the_rest_from_being_set_up(self):
        switch = self._stubbed(made(), fails=("Bambu_AMS_1",))
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert str(err.value) == "Bambu_AMS_1 failed"
        gcmd.error.assert_called_once_with("Bambu_AMS_1 failed")
        assert self._ran(switch) == (self.BOTH_UNITS, [None])
        # The prompt opens with every step's outcome before the error is raised.
        switch._open_menu.assert_called_once_with(
            gcmd, False, "Bambu_AMS_1 failed\nBambu_AMS_2 found\ntoolheads found")

    def test_toolheads_that_are_not_found_are_the_commands_error(self):
        switch = self._stubbed(made(), fails=("toolheads",))
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert str(err.value) == "toolheads failed"
        assert self._ran(switch) == (self.BOTH_UNITS, [None])
        switch._open_menu.assert_called_once_with(
            gcmd, False, "Bambu_AMS_1 found\nBambu_AMS_2 found\ntoolheads failed")

    def test_every_step_that_failed_is_in_the_commands_error(self):
        switch = self._stubbed(made(), fails=("Bambu_AMS_1", "Bambu_AMS_2", "toolheads"))
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        said = "Bambu_AMS_1 failed\nBambu_AMS_2 failed\ntoolheads failed"
        assert str(err.value) == said
        gcmd.error.assert_called_once_with(said)
        assert self._ran(switch) == (self.BOTH_UNITS, [None])
        switch._open_menu.assert_called_once_with(gcmd, False, said)

    def test_toolheads_that_are_known_are_not_found_again(self):
        switch = self._stubbed(made(outlet_a="extruder1", outlet_b="extruder"))
        gcmd = MockGCodeCommand()
        switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert self._ran(switch) == (self.BOTH_UNITS, [])
        switch._open_menu.assert_called_once_with(gcmd, False,
                                                  "Bambu_AMS_1 found\nBambu_AMS_2 found")

    @pytest.mark.parametrize("values", [{}, {"outlet_a": "extruder1", "outlet_b": "extruder"}])
    def test_with_outlets_the_inlets_are_left_out_and_the_toolheads_are_found(self, values):
        switch = self._stubbed(made(**values))
        gcmd = MockGCodeCommand({"OUTLETS": 1})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert self._ran(switch) == ([], [None])
        switch._open_menu.assert_called_once_with(gcmd, True, "toolheads found")

    def test_a_lane_that_is_named_is_the_one_fed_through_the_outlets(self):
        switch = self._stubbed(made())
        gcmd = MockGCodeCommand({"LANE": "lane_Bambu_AMS_2"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        # A unit's inlet is found with the unit's own lane.
        assert self._ran(switch) == (self.BOTH_UNITS, ["lane_Bambu_AMS_2"])

    def test_with_nothing_to_set_up_it_says_so(self):
        switch = self._stubbed(known(outlet_a="extruder1", outlet_b="extruder"))
        gcmd = MockGCodeCommand()
        switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert self._ran(switch) == ([], [])
        switch._open_menu.assert_called_once_with(gcmd, False, self.NOTHING)
        gcmd.error.assert_not_called()

    def test_any_other_failure_of_a_unit_closes_the_prompt_and_is_passed_on(self):
        switch = self._stubbed(made(), fails=("Bambu_AMS_1",), error=RuntimeError)
        gcmd = MockGCodeCommand()
        with pytest.raises(RuntimeError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert str(err.value) == "Bambu_AMS_1 failed"
        # Nothing is run after it.
        assert self._ran(switch) == ([(["Bambu_AMS_1"], None)], [])
        switch._open_menu.assert_not_called()
        gcmd.error.assert_not_called()
        assert switch.logger.messages == [END]

    def test_any_other_failure_of_the_toolheads_closes_the_prompt_and_is_passed_on(self):
        switch = self._stubbed(made(), fails=("toolheads",), error=RuntimeError)
        gcmd = MockGCodeCommand()
        with pytest.raises(RuntimeError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert str(err.value) == "toolheads failed"
        assert self._ran(switch) == (self.BOTH_UNITS, [None])
        switch._open_menu.assert_not_called()
        gcmd.error.assert_not_called()
        assert switch.logger.messages == [END]

    @staticmethod
    def _text(*lines: str) -> List[Tuple[str, str]]:
        """:return list: the lines of a prompt's text"""
        return [("raw", f"// action:prompt_text {line}") for line in lines]

    @staticmethod
    def _inlet(unit: str) -> str:
        """:return str: what the prompt says while a unit's inlet is found"""
        return (f"Finding the inlet of {unit}: lane_{unit} is fed to the switch and reeled "
                "back. A lane that is loaded through the switch is unloaded first.")

    @staticmethod
    def _outlets(lane: str) -> str:
        """:return str: what the prompt says while the toolheads are found with a lane"""
        return (f"Finding the toolhead each outlet feeds: {lane} is fed through each outlet "
                "until a toolhead sensor reads it, and reeled back.")

    def test_a_switch_is_set_up_from_start_to_end(self):
        switch = made()
        for unit in BAYS:
            lane_of(switch, unit).load_state = True
        feed = setting_up(switch, {"Bambu_AMS_1": "A", "Bambu_AMS_2": "B"}, PLUMBING)
        gcmd = MockGCodeCommand()
        switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        first, second = lane_of(switch, "Bambu_AMS_1"), lane_of(switch, "Bambu_AMS_2")
        # Each unit's lane to the switch, then the first lane through each outlet.
        assert feed.call_args_list == [
            call(first, switch._saw, gcmd), call(second, switch._saw, gcmd),
            call(first, switch._saw_toolhead, gcmd), call(first, switch._saw_toolhead, gcmd)]
        assert switch.inlets == {"A": ["Bambu_AMS_1"], "B": ["Bambu_AMS_2"]}
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}
        assert (first.extruder_obj.name, second.extruder_obj.name) == ("extruder1", "extruder")
        chain = chain_of(switch)
        assert chain.routes == [2, 0]
        assert chain.saved == [{"outlet_a": "extruder1", "outlet_a_length": "",
                                "outlet_b": "extruder", "outlet_b_length": ""}]
        entered = ["Chain_1: Bambu_AMS_1 enters inlet A.", "Chain_1: Bambu_AMS_2 enters inlet B."]
        assert gcmd.respond_info.call_args_list == [
            call(entered[0]), call(entered[1]), call(f"{self.OUTLET_A}\n{self.OUTLET_B}")]
        inlets = ("raw", "// action:prompt_begin Chain_1 Inlets")
        assert switch.logger.messages == (
            [inlets] + self._text(self._inlet("Bambu_AMS_1")) + [SHOW]
            + [inlets] + self._text(entered[0], self._inlet("Bambu_AMS_2")) + [SHOW]
            + [TestAfcBridgeBoxSwitchCalibrationMenu.MADE_BEGIN]
            + self._text(*entered, self._outlets("lane_Bambu_AMS_1")) + [SHOW]
            + [("raw", "// action:prompt_begin Chain_1 Calibration")]
            + self._text(*entered, self.OUTLET_A, self.OUTLET_B,
                         f"{self.WHAT} The tube lengths are not measured yet.")
            + self.BUTTONS + TO_AFC)

    def test_a_unit_that_is_not_found_leaves_the_rest_set_up_from_start_to_end(self):
        switch = made()
        for unit in BAYS:
            lane_of(switch, unit).load_state = True
        feed = setting_up(switch, {"Bambu_AMS_1": None, "Bambu_AMS_2": "B"}, PLUMBING)
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_SETUP(gcmd)
        assert str(err.value) == self.NOT_READ
        first, second = lane_of(switch, "Bambu_AMS_1"), lane_of(switch, "Bambu_AMS_2")
        # The toolheads are found with the lane of the unit whose inlet is known.
        assert feed.call_args_list == [
            call(first, switch._saw, gcmd), call(second, switch._saw, gcmd),
            call(second, switch._saw_toolhead, gcmd), call(second, switch._saw_toolhead, gcmd)]
        assert switch.inlets == {"A": [], "B": ["Bambu_AMS_2"]}
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}
        assert second.extruder_obj.name == "extruder"
        assert chain_of(switch).routes == [1, 0]
        entered = "Chain_1: Bambu_AMS_2 enters inlet B."
        assert gcmd.respond_info.call_args_list == [
            call(entered), call(f"{self.OUTLET_A}\n{self.OUTLET_B}")]
        inlets = ("raw", "// action:prompt_begin Chain_1 Inlets")
        assert switch.logger.messages == (
            [inlets] + self._text(self._inlet("Bambu_AMS_1")) + [SHOW]
            + [inlets] + self._text(self.NOT_READ, self._inlet("Bambu_AMS_2")) + [SHOW]
            + [TestAfcBridgeBoxSwitchCalibrationMenu.MADE_BEGIN]
            + self._text(self.NOT_READ, entered, self._outlets("lane_Bambu_AMS_2")) + [SHOW]
            + [("raw", "// action:prompt_begin Chain_1 Calibration")]
            + self._text(self.NOT_READ, entered, self.OUTLET_A, self.OUTLET_B,
                         f"{self.WHAT} Inlet not known yet: Bambu_AMS_1. The tube lengths are "
                         "not measured yet.")
            + self.BUTTONS + TO_AFC)


class TestAfcBridgeBoxSwitchFindToolheads:
    SAYS = ("raw", "// action:prompt_text Finding the toolhead each outlet feeds: "
                   "lane_Bambu_AMS_1 is fed through each outlet until a toolhead sensor reads "
                   "it, and reeled back.")
    FOUND = ["Chain_1: outlet A feeds extruder1.", "Chain_1: outlet B feeds extruder."]

    @staticmethod
    def _switch() -> Tuple[afcBridgeBoxSwitch, FakeLane]:
        """
        :return tuple: a switch whose lanes both hold filament, with the run that finds the
        toolheads stubbed, and the first of its lanes
        """
        switch = known()
        for unit in BAYS:
            lane_of(switch, unit).load_state = True
        switch._find_outlets = MagicMock(return_value=list(
            TestAfcBridgeBoxSwitchFindToolheads.FOUND))
        return switch, lane_of(switch, "Bambu_AMS_1")

    def test_the_first_lane_that_can_be_is_fed_under_what_was_done_before(self):
        switch, lane = self._switch()
        gcmd = MockGCodeCommand()
        done = ["Chain_1: Bambu_AMS_1 enters inlet A."]
        switch._find_toolheads(None, done, AFCprompt(gcmd, switch.logger), gcmd)
        switch._find_outlets.assert_called_once_with(lane, gcmd)
        assert done == ["Chain_1: Bambu_AMS_1 enters inlet A."] + self.FOUND
        gcmd.respond_info.assert_called_once_with(
            "Chain_1: outlet A feeds extruder1.\nChain_1: outlet B feeds extruder.")
        assert switch.logger.messages == [
            TestAfcBridgeBoxSwitchCalibrationMenu.MADE_BEGIN,
            ("raw", "// action:prompt_text Chain_1: Bambu_AMS_1 enters inlet A."), self.SAYS,
            SHOW]

    def test_a_lane_that_is_named_is_the_one_fed(self):
        switch, _lane = self._switch()
        gcmd = MockGCodeCommand()
        done: List[str] = []
        switch._find_toolheads("lane_Bambu_AMS_2", done, AFCprompt(gcmd, switch.logger), gcmd)
        switch._find_outlets.assert_called_once_with(lane_of(switch, "Bambu_AMS_2"), gcmd)
        assert done == self.FOUND
        assert switch.logger.messages == [
            TestAfcBridgeBoxSwitchCalibrationMenu.MADE_BEGIN,
            ("raw", self.SAYS[1].replace("lane_Bambu_AMS_1", "lane_Bambu_AMS_2")), SHOW]

    def test_with_no_lane_to_feed_no_prompt_opens(self):
        switch, _lane = self._switch()
        gcmd = MockGCodeCommand()
        done: List[str] = []
        with pytest.raises(SwitchError) as err:
            switch._find_toolheads("lane9", done, AFCprompt(gcmd, switch.logger), gcmd)
        assert str(err.value) == "Chain_1: lane9 is not a lane of this switch"
        switch._find_outlets.assert_not_called()
        assert done == []
        gcmd.respond_info.assert_not_called()
        assert switch.logger.messages == []

    def test_toolheads_that_are_not_found_add_nothing(self):
        switch, lane = self._switch()
        switch._find_outlets = MagicMock(side_effect=SwitchError("Chain_1: it did not work"))
        gcmd = MockGCodeCommand()
        done: List[str] = []
        with pytest.raises(SwitchError) as err:
            switch._find_toolheads(None, done, AFCprompt(gcmd, switch.logger), gcmd)
        assert str(err.value) == "Chain_1: it did not work"
        switch._find_outlets.assert_called_once_with(lane, gcmd)
        assert done == []
        gcmd.respond_info.assert_not_called()
        assert switch.logger.messages == [
            TestAfcBridgeBoxSwitchCalibrationMenu.MADE_BEGIN, self.SAYS, SHOW]


class TestAfcBridgeBoxSwitchToFind:
    def test_a_switch_with_a_section_of_its_own_has_no_outlet_to_find(self):
        assert finding()._to_find() == []

    def test_a_switch_its_chain_made_has_both(self):
        assert made()._to_find() == ["A", "B"]

    @pytest.mark.parametrize("key, letters", [("outlet_a", ["B"]), ("outlet_b", ["A"])])
    def test_an_outlet_the_chains_section_sets_is_not_one(self, key, letters):
        switch = made(**{key: "extruder", "by_hand": [key]})
        assert switch._to_find() == letters

    def test_with_both_set_there_is_none(self):
        switch = made(outlet_a="extruder1", outlet_b="extruder",
                      by_hand=["outlet_a", "outlet_b"])
        assert switch._to_find() == []

    def test_a_tube_length_the_chains_section_sets_does_not_set_its_outlet(self):
        switch = made(outlet_a_length=850, by_hand=["outlet_a_length"])
        assert switch._to_find() == ["A", "B"]


class TestAfcBridgeBoxSwitchRamming:
    @staticmethod
    def _said(names: str) -> str:
        """:return str: what is said about the extruders that load by buffer ramming"""
        return ("Chain_1: the toolhead each outlet feeds cannot be found by feeding a lane. "
                f"With buffer ramming on {names} there is no toolhead sensor to read. Set "
                "switch_outlet_a and switch_outlet_b in [AFC_BridgeBox Chain_1] by hand, then "
                "RESTART.")

    def test_with_a_toolhead_sensor_on_every_extruder_there_is_nothing_to_say(self):
        assert made()._ramming() == ""

    def test_an_extruder_of_its_lanes_that_loads_by_buffer_ramming_is_named_once(self):
        switch = made()
        # Both of the switch's lanes are on it.
        extruder_of(switch, "extruder").tool_start = "buffer"
        assert switch._ramming() == self._said("extruder")

    def test_each_of_them_is_named_in_order(self):
        switch = made()
        lane_of(switch, "Bambu_AMS_1").set_extruder(extruder_of(switch, "extruder1"))
        for name in ("extruder", "extruder1"):
            extruder_of(switch, name).tool_start = "buffer"
        assert switch._ramming() == self._said("extruder and extruder1")

    def test_an_extruder_none_of_its_lanes_is_on_does_not_count(self):
        switch = made()
        extruder_of(switch, "extruder1").tool_start = "buffer"
        assert switch._ramming() == ""

    def test_a_switch_with_a_section_of_its_own_names_its_own_options(self):
        switch = finding()
        extruder_of(switch, "extruder").tool_start = "buffer"
        assert switch._ramming() == (
            "Switch_1: the toolhead each outlet feeds cannot be found by feeding a lane. With "
            "buffer ramming on extruder there is no toolhead sensor to read. Set outlet_a and "
            "outlet_b in [AFC_BridgeBox_switch Switch_1] by hand, then RESTART.")


class TestAfcBridgeBoxSwitchSensor:
    def test_an_extruder_with_no_toolhead_sensor_has_none_to_read(self):
        switch = plumbed()
        extruder = extruder_of(switch, "extruder")
        extruder.tool_start = "PA1"
        extruder.tool_start_state = True
        assert switch._sensor(extruder) is None

    def test_a_sensor_that_stands_in_for_one_is_not_the_extruders_own(self):
        switch = plumbed()
        extruder = give_sensor(switch, "extruder", reads=True)
        extruder.tool_start = "virtual"
        assert switch._sensor(extruder) is None

    @pytest.mark.parametrize("reads", [True, False])
    def test_a_sensor_on_a_pin_reads_what_afc_keeps_for_it(self, reads):
        switch = plumbed()
        assert switch._sensor(give_sensor(switch, "extruder", reads=reads)) is reads

    @pytest.mark.parametrize("reads", [True, False])
    def test_a_motion_sensor_reads_what_its_switch_reads_now(self, reads):
        switch = plumbed()
        # What AFC keeps for it is what it read before the filament last moved.
        extruder = give_sensor(switch, "extruder", reads=not reads)
        extruder.fila_tool_start = FakeMotionSensor(reads)
        assert switch._sensor(extruder) is reads


class TestAfcBridgeBoxSwitchToolheads:
    def test_every_extruder_whose_own_toolhead_sensor_reads_empty(self):
        switch = made()
        assert switch._toolheads() == {"extruder": extruder_of(switch, "extruder"),
                                       "extruder1": extruder_of(switch, "extruder1")}

    def test_one_whose_sensor_reads_filament_is_left_out(self):
        switch = made()
        extruder_of(switch, "extruder").tool_start_state = True
        assert switch._toolheads() == {"extruder1": extruder_of(switch, "extruder1")}

    def test_one_with_no_toolhead_sensor_of_its_own_is_left_out(self):
        switch = made()
        extruder_of(switch, "extruder1").fila_tool_start = None
        assert switch._toolheads() == {"extruder": extruder_of(switch, "extruder")}

    def test_the_ones_another_switch_feeds_are_left_out(self):
        first, second, _lane = two_switches()
        for name in ("extruder", "extruder1", "extruder2"):
            give_sensor(first, name)
        # Its own extruders are among them: an outlet may be found again.
        assert list(first._toolheads()) == ["extruder", "extruder1"]
        assert list(second._toolheads()) == ["extruder2"]


class TestAfcBridgeBoxSwitchSawToolhead:
    @staticmethod
    def _watching(*names: str) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch that watches these extruders' sensors"""
        switch = made()
        switch._watched = {name: extruder_of(switch, name) for name in names}
        return switch

    def test_no_watched_sensor_read_filament(self):
        switch = self._watching("extruder", "extruder1")
        assert switch._saw_toolhead() is False
        assert switch._read == []

    def test_a_watched_sensor_that_reads_filament_now(self):
        switch = self._watching("extruder", "extruder1")
        extruder_of(switch, "extruder1").tool_start_state = True
        assert switch._saw_toolhead() is True
        assert switch._read == ["extruder1"]

    def test_each_one_that_reads_filament_is_noted(self):
        switch = self._watching("extruder", "extruder1")
        for name in ("extruder", "extruder1"):
            extruder_of(switch, name).tool_start_state = True
        assert switch._saw_toolhead() is True
        assert switch._read == ["extruder", "extruder1"]

    def test_one_that_read_filament_since_it_was_last_cleared_is_noted_once(self):
        switch = self._watching("extruder", "extruder1")
        extruder = extruder_of(switch, "extruder1")
        extruder.tool_start_state = True
        switch._saw_toolhead()
        assert switch._saw_toolhead() is True
        assert switch._read == ["extruder1"]
        extruder.tool_start_state = False
        assert switch._saw_toolhead() is True
        assert switch._read == ["extruder1"]

    def test_a_sensor_that_is_not_watched_does_not_count(self):
        switch = self._watching("extruder")
        extruder_of(switch, "extruder1").tool_start_state = True
        assert switch._saw_toolhead() is False
        assert switch._read == []


class TestAfcBridgeBoxSwitchOutletLane:
    @staticmethod
    def _refused(switch: afcBridgeBoxSwitch, name: Optional[str], said: str) -> None:
        """Check no lane is picked, with ``said``."""
        with pytest.raises(SwitchError) as err:
            switch._outlet_lane(name)
        assert str(err.value) == f"Chain_1: {said}"

    def test_the_first_lane_that_holds_filament(self):
        switch = known()
        lane_of(switch, "Bambu_AMS_2").load_state = True
        assert switch._outlet_lane(None) is lane_of(switch, "Bambu_AMS_2")

    def test_a_lane_of_a_unit_whose_inlet_is_not_known_is_passed_over(self):
        switch = made()
        found(switch, Bambu_AMS_2="B")
        for unit in BAYS:
            lane_of(switch, unit).load_state = True
        assert switch._outlet_lane(None) is lane_of(switch, "Bambu_AMS_2")

    def test_a_lane_that_is_named(self):
        switch = known()
        for unit in BAYS:
            lane_of(switch, unit).load_state = True
        assert switch._outlet_lane("lane_Bambu_AMS_2") is lane_of(switch, "Bambu_AMS_2")

    def test_a_lane_that_is_not_on_this_switch(self):
        self._refused(known(), "lane9", "lane9 is not a lane of this switch")

    def test_no_lane_holds_filament(self):
        self._refused(known(), None, "no lane of a unit whose inlet is known holds filament. Put "
                                     "a spool in one first.")

    def test_only_a_lane_of_a_unit_whose_inlet_is_not_known_holds_filament(self):
        switch = made()
        found(switch, Bambu_AMS_2="B")
        lane_of(switch, "Bambu_AMS_1").load_state = True
        self._refused(switch, None, "no lane of a unit whose inlet is known holds filament. Put "
                                    "a spool in one first.")

    def test_a_named_lane_of_a_unit_whose_inlet_is_not_known(self):
        switch = made()
        lane_of(switch, "Bambu_AMS_1").load_state = True
        self._refused(switch, "lane_Bambu_AMS_1",
                      "lane_Bambu_AMS_1 cannot find the toolheads, the inlet of Bambu_AMS_1 is "
                      "not known. Run AFC_BRIDGEBOX_SWITCH_IDENTIFY UNIT=Bambu_AMS_1 first.")

    def test_a_named_lane_that_holds_no_filament(self):
        self._refused(known(), "lane_Bambu_AMS_2",
                      "lane_Bambu_AMS_2 holds no filament. Put a spool in it first.")

    BY_HAND = "set switch_outlet_a and switch_outlet_b by hand."

    @staticmethod
    def _whole_loads(switch: afcBridgeBoxSwitch, unit: str) -> FakeLane:
        """:return FakeLane: a unit's lane, which holds filament and only whole loads move"""
        lane = lane_of(switch, unit)
        lane.load_state = True
        lane.unit_obj = FakeUnit(unit)
        return lane

    def test_a_lane_that_cannot_be_fed_is_passed_over(self):
        switch = known()
        self._whole_loads(switch, "Bambu_AMS_1")
        lane_of(switch, "Bambu_AMS_2").load_state = True
        assert switch._outlet_lane(None) is lane_of(switch, "Bambu_AMS_2")

    def test_only_lanes_that_cannot_be_fed_hold_filament(self):
        switch = known()
        self._whole_loads(switch, "Bambu_AMS_1")
        self._refused(switch, None, "no lane that holds filament can be fed to a toolhead "
                                    "sensor and reeled back. Their units only run whole loads, "
                                    f"so {self.BY_HAND}")

    def test_a_named_lane_that_cannot_be_fed(self):
        switch = known()
        self._whole_loads(switch, "Bambu_AMS_1")
        self._refused(switch, "lane_Bambu_AMS_1",
                      "Bambu_AMS_1 only runs whole loads, so lane_Bambu_AMS_1 cannot be fed to "
                      f"a sensor and reeled back. Use a lane of another unit, or {self.BY_HAND}")


class TestAfcBridgeBoxSwitchFindOutlets:
    SAVED = {"outlet_a": "extruder1", "outlet_a_length": "", "outlet_b": "extruder",
             "outlet_b_length": ""}

    @staticmethod
    def _switch(toolheads: Optional[Dict[str, List[str]]] = None, unit: str = "Bambu_AMS_1",
                lingers: bool = False,
                **values: Any) -> Tuple[afcBridgeBoxSwitch, FakeLane, MagicMock]:
        """
        :return tuple: a switch with every inlet known, the lane of ``unit``, which holds
        filament, and the unit's feed, which reaches the ``toolheads`` each outlet feeds
        """
        switch = known(**values)
        lane = lane_of(switch, unit)
        lane.load_state = True
        feed = reaching(switch, unit, PLUMBING if toolheads is None else toolheads, lingers)
        return switch, lane, feed

    def _refused(self, switch: afcBridgeBoxSwitch, lane: FakeLane, said: str) -> None:
        """Check no toolhead is found, with ``said``, and nothing is saved."""
        was = dict(switch.outlets)
        with pytest.raises(SwitchError) as err:
            switch._find_outlets(lane, MockGCodeCommand())
        assert str(err.value) == said
        assert switch.outlets == was
        assert chain_of(switch).saved == []
        switch.afc.save_vars.assert_not_called()
        assert switch._moving is False

    def test_the_lane_is_fed_through_each_outlet_and_the_toolhead_that_read_it_is_kept(self):
        switch, lane, feed = self._switch()
        gcmd = MockGCodeCommand()
        assert switch._find_outlets(lane, gcmd) == ["Chain_1: outlet A feeds extruder1.",
                                                    "Chain_1: outlet B feeds extruder."]
        assert feed.call_args_list == [call(lane, switch._saw_toolhead, gcmd),
                                       call(lane, switch._saw_toolhead, gcmd)]
        chain = chain_of(switch)
        # Inlet A is at outlet B at first.
        assert chain.routes == [2, 0]
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}
        assert chain.saved == [self.SAVED]
        assert lane.extruder_obj.name == "extruder1"
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder"
        switch.afc.save_vars.assert_called_once_with()
        assert switch._moving is False
        assert switch.logger.messages == []

    def test_a_lane_of_inlet_b_has_that_inlet_pointed_at_each_outlet(self):
        switch, lane, feed = self._switch(unit="Bambu_AMS_2")
        assert switch._find_outlets(lane, MockGCodeCommand()) == [
            "Chain_1: outlet A feeds extruder1.", "Chain_1: outlet B feeds extruder."]
        assert feed.call_count == 2
        assert chain_of(switch).routes == [1, 0]
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}

    def test_both_inlets_are_emptied_before_the_lane_is_fed_with_the_switch_held(self):
        switch, lane, feed = self._switch()
        order = []
        switch._clear = MagicMock(side_effect=lambda gcmd: order.append(
            ("clear", switch._moving, feed.call_count)))
        gcmd = MockGCodeCommand()
        switch._find_outlets(lane, gcmd)
        switch._clear.assert_called_once_with(gcmd)
        assert order == [("clear", True, 0)]

    def test_inlets_that_cannot_be_emptied_keep_the_lane_from_being_fed(self):
        switch, lane, feed = self._switch()
        switch._clear = MagicMock(side_effect=SwitchError("Chain_1: it is full"))
        self._refused(switch, lane, "Chain_1: it is full")
        feed.assert_not_called()

    def test_while_a_print_runs_nothing_is_fed(self):
        switch, lane, feed = self._switch()
        switch._clear = MagicMock()
        switch.afc.function.in_print.return_value = True
        self._refused(switch, lane, "Chain_1: the toolheads are not found while a print runs")
        switch._clear.assert_not_called()
        feed.assert_not_called()

    def test_outlets_the_chains_section_sets_are_not_found(self):
        switch, lane, feed = self._switch(outlet_a="extruder1", outlet_b="extruder",
                                          by_hand=["outlet_a", "outlet_b"])
        self._refused(switch, lane, "Chain_1: its outlets are set in [AFC_BridgeBox Chain_1], "
                                    "so there is no toolhead to find")
        feed.assert_not_called()

    def test_with_buffer_ramming_it_says_to_set_the_outlets_by_hand(self):
        switch, lane, feed = self._switch()
        extruder_of(switch, "extruder").tool_start = "buffer"
        self._refused(switch, lane, TestAfcBridgeBoxSwitchRamming._said("extruder"))
        feed.assert_not_called()

    def test_a_toolhead_that_reads_filament_keeps_the_lane_from_being_fed(self):
        switch, lane, feed = self._switch()
        extruder_of(switch, "extruder").tool_start_state = True
        self._refused(switch, lane, "Chain_1: the toolhead sensor of extruder reads filament. "
                                    "Unload it first, a lane cannot be seen arriving there.")
        feed.assert_not_called()

    def test_an_outlet_through_which_no_sensor_read_the_lane_stays_as_it_was(self):
        switch, lane, feed = self._switch({"A": ["extruder1"]})
        assert switch._find_outlets(lane, MockGCodeCommand()) == [
            "Chain_1: outlet A feeds extruder1.",
            "Chain_1: no toolhead sensor read lane_Bambu_AMS_1 through outlet B, which stays as "
            "it was. An outlet that feeds a toolhead with no sensor is set with "
            "switch_outlet_b."]
        assert feed.call_count == 2
        assert switch.outlets == {"A": "extruder1", "B": None}
        assert chain_of(switch).saved == [dict(self.SAVED, outlet_b="")]

    def test_a_lane_no_sensor_read_through_either_outlet_finds_nothing(self):
        switch, lane, feed = self._switch({})
        self._refused(switch, lane,
                      "Chain_1: no toolhead sensor read lane_Bambu_AMS_1 through an outlet. "
                      "Check the tubes from the switch to the toolheads, or set switch_outlet_a "
                      "and switch_outlet_b by hand.")
        # It was fed through both.
        assert feed.call_count == 2
        assert chain_of(switch).routes == [2, 0]

    def test_a_lane_that_is_not_reeled_back_ends_it_and_what_was_found_is_kept(self):
        switch, lane, feed = self._switch(lingers=True)
        with pytest.raises(SwitchError) as err:
            switch._find_outlets(lane, MockGCodeCommand())
        assert str(err.value) == (
            "Chain_1: lane_Bambu_AMS_1 was not reeled back from outlet A and is still in inlet "
            "A. Run LANE_UNLOAD LANE=lane_Bambu_AMS_1, then find the toolheads again.")
        assert feed.call_count == 1
        assert switch.outlets == {"A": "extruder1", "B": None}
        assert chain_of(switch).saved == [dict(self.SAVED, outlet_b="")]
        assert lane.extruder_obj.name == "extruder1"
        switch.afc.save_vars.assert_called_once_with()
        assert switch._moving is False

    def test_the_extruder_of_an_outlet_the_chains_section_sets_is_not_watched(self):
        switch, lane, feed = self._switch(outlet_b="extruder", by_hand=["outlet_b"])
        assert switch._find_outlets(lane, MockGCodeCommand()) == [
            "Chain_1: outlet A feeds extruder1."]
        # Only outlet A was looked for.
        assert feed.call_count == 1
        assert chain_of(switch).routes == [2]
        assert list(switch._watched) == ["extruder1"]
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}
        assert chain_of(switch).saved == [{"outlet_a": "extruder1", "outlet_a_length": "",
                                           "outlet_b_length": ""}]

    def test_an_outlet_that_is_not_found_beside_one_that_is_set_is_no_error(self):
        switch, lane, _feed = self._switch({}, outlet_b="extruder", by_hand=["outlet_b"])
        assert switch._find_outlets(lane, MockGCodeCommand()) == [
            "Chain_1: no toolhead sensor read lane_Bambu_AMS_1 through outlet A, which stays as "
            "it was. An outlet that feeds a toolhead with no sensor is set with "
            "switch_outlet_a."]
        assert switch.outlets == {"A": None, "B": "extruder"}
        assert chain_of(switch).saved == []


class TestAfcBridgeBoxSwitchCheckToolheads:
    def test_with_every_toolhead_empty_the_lane_may_be_fed(self):
        switch = made()
        switch._check_toolheads()
        assert switch.logger.messages == []

    def test_a_toolhead_of_its_lanes_that_reads_filament_is_named_once(self):
        switch = made()
        # Both of the switch's lanes are on it.
        extruder_of(switch, "extruder").tool_start_state = True
        with pytest.raises(SwitchError) as err:
            switch._check_toolheads()
        assert str(err.value) == ("Chain_1: the toolhead sensor of extruder reads filament. "
                                  "Unload it first, a lane cannot be seen arriving there.")

    def test_each_of_them_is_named_in_order(self):
        switch = made()
        lane_of(switch, "Bambu_AMS_1").set_extruder(extruder_of(switch, "extruder1"))
        for name in ("extruder", "extruder1"):
            extruder_of(switch, name).tool_start_state = True
        with pytest.raises(SwitchError) as err:
            switch._check_toolheads()
        assert str(err.value) == ("Chain_1: the toolhead sensor of extruder and extruder1 reads "
                                  "filament. Unload it first, a lane cannot be seen arriving "
                                  "there.")

    def test_a_toolhead_none_of_its_lanes_is_on_does_not_count(self):
        switch = made()
        extruder_of(switch, "extruder1").tool_start_state = True
        switch._check_toolheads()
        assert switch.logger.messages == []


class TestAfcBridgeBoxSwitchFindOutlet:
    @staticmethod
    def _switch(toolheads: Optional[Dict[str, List[str]]] = None
                ) -> Tuple[afcBridgeBoxSwitch, FakeLane, MagicMock]:
        """
        :return tuple: a switch with every inlet known, the lane of Bambu_AMS_1 and the unit's
        feed, which reaches the ``toolheads`` each outlet feeds
        """
        switch = known()
        lane = lane_of(switch, "Bambu_AMS_1")
        return switch, lane, reaching(switch, "Bambu_AMS_1",
                                      PLUMBING if toolheads is None else toolheads)

    def test_the_inlet_is_pointed_at_the_outlet_and_the_sensor_that_read_the_lane_is_it(self):
        switch, lane, feed = self._switch()
        gcmd = MockGCodeCommand()
        assert switch._find_outlet(lane, "A", "A", [], gcmd) == "extruder1"
        assert chain_of(switch).routes == [2]
        feed.assert_called_once_with(lane, switch._saw_toolhead, gcmd)
        assert list(switch._watched) == ["extruder", "extruder1"]
        assert switch._read == ["extruder1"]

    def test_an_inlet_that_is_at_the_outlet_is_fed_as_it_is(self):
        switch, lane, feed = self._switch()
        assert switch._find_outlet(lane, "A", "B", [], MockGCodeCommand()) == "extruder"
        assert chain_of(switch).routes == []
        assert feed.call_count == 1

    def test_no_sensor_read_the_lane(self):
        switch, lane, feed = self._switch({})
        # What was read through the outlet before does not count.
        switch._read = ["extruder"]
        assert switch._find_outlet(lane, "A", "A", [], MockGCodeCommand()) is None
        assert feed.call_count == 1
        assert switch._read == []

    @pytest.mark.parametrize("taken", [["extruder1"], [None, "extruder1"]])
    def test_the_extruders_the_other_outlet_feeds_are_not_watched(self, taken):
        switch, lane, feed = self._switch()
        assert switch._find_outlet(lane, "A", "A", taken, MockGCodeCommand()) is None
        assert feed.call_count == 1
        assert list(switch._watched) == ["extruder"]

    def test_with_no_sensor_left_to_watch_the_lane_is_not_fed(self):
        switch, lane, feed = self._switch()
        with pytest.raises(SwitchError) as err:
            switch._find_outlet(lane, "A", "A", ["extruder", "extruder1"], MockGCodeCommand())
        assert str(err.value) == ("Chain_1: no toolhead sensor that reads empty is left to find "
                                  "outlet A with")
        assert chain_of(switch).routes == []
        feed.assert_not_called()

    def test_two_sensors_that_read_the_lane_leave_the_outlet_unknown(self):
        switch, lane, _feed = self._switch({"A": ["extruder", "extruder1"]})
        with pytest.raises(SwitchError) as err:
            switch._find_outlet(lane, "A", "A", [], MockGCodeCommand())
        assert str(err.value) == (
            "Chain_1: the toolhead sensors of extruder and extruder1 read filament while "
            "lane_Bambu_AMS_1 was fed through outlet A, so its toolhead is not known.")

    def test_the_inlet_is_given_time_to_read_empty_once_the_lane_was_fed(self):
        switch, lane, feed = self._switch()
        fed = []
        switch._wait_clear = MagicMock(side_effect=lambda inlet: fed.append(feed.call_count))
        switch._find_outlet(lane, "A", "A", [], MockGCodeCommand())
        switch._wait_clear.assert_called_once_with("A")
        assert fed == [1]

    def test_a_lane_afc_moves_is_fed_until_a_toolhead_sensor_reads_it(self):
        switch = known()
        lane = lane_of(switch, "Bambu_AMS_1")
        lane.unit_obj = FakeUnit("Bambu_AMS_1")
        moves = stepping(switch, stepped(lane), toolhead="extruder1", after=2)
        gcmd = MockGCodeCommand()
        assert switch._find_outlet(lane, "A", "A", [], gcmd) == "extruder1"
        step = call(25.0, SpeedMode.CALIBRATION, AssistActive.NO, gcmd=gcmd)
        assert moves.call_args_list == [step, step,
                                        call(-50.0, SpeedMode.LONG, AssistActive.YES, gcmd=gcmd)]
        assert switch._read == ["extruder1"]
        assert switch._check is None


class TestAfcBridgeBoxSwitchSetOutlets:
    @staticmethod
    def _measured(**values: Any) -> afcBridgeBoxSwitch:
        """
        :return afcBridgeBoxSwitch: a switch with every inlet known, both tube lengths measured
        and something kept for each of its lanes
        """
        switch = known(outlet_a_length=850, outlet_b_length=1010, **values)
        switch._saved_lanes = {"lane_Bambu_AMS_1": "extruder", "lane_Bambu_AMS_2": "extruder1"}
        switch._by_print = ["lane_Bambu_AMS_1"]
        switch._due_back = ["lane_Bambu_AMS_2"]
        return switch

    def test_each_extruder_is_put_on_its_outlet_and_saved(self):
        switch = self._measured()
        switch._set_outlets({"A": "extruder1", "B": "extruder"})
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}
        # The tubes were measured to other toolheads.
        assert switch.lengths == {"A": None, "B": None}
        assert chain_of(switch).saved == [{"outlet_a": "extruder1", "outlet_a_length": "",
                                           "outlet_b": "extruder", "outlet_b_length": ""}]
        assert (switch.start, switch.rest) == ({"A": "A", "B": "B"}, {"A": "A", "B": "B"})
        assert (switch._saved_lanes, switch._by_print, switch._due_back) == ({}, [], [])
        # Every lane goes to the extruder its unit starts on now, whatever was kept for it.
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder1"
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder"
        assert chain_of(switch).joined[-2:] == [["lane_Bambu_AMS_1"], ["lane_Bambu_AMS_2"]]
        switch.afc.save_vars.assert_called_once_with()

    def test_with_one_outlet_found_every_unit_starts_on_it(self):
        switch = self._measured()
        lane_of(switch, "Bambu_AMS_1").set_extruder(extruder_of(switch, "extruder1"))
        switch._set_outlets({"B": "extruder"})
        assert switch.outlets == {"A": None, "B": "extruder"}
        assert chain_of(switch).saved == [{"outlet_a": "", "outlet_a_length": "",
                                           "outlet_b": "extruder", "outlet_b_length": ""}]
        assert (switch.start, switch.rest) == ({"A": "B", "B": "B"}, {"A": "B", "B": "B"})
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder"
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder"

    def test_an_extruder_that_is_found_on_the_other_outlet_leaves_the_one_it_was_on(self):
        switch = self._measured(outlet_a="extruder", outlet_b="extruder1")
        switch._set_outlets({"B": "extruder"})
        assert switch.outlets == {"A": None, "B": "extruder"}
        assert switch.lengths == {"A": None, "B": None}
        assert chain_of(switch).saved == [{"outlet_a": "", "outlet_a_length": "",
                                           "outlet_b": "extruder", "outlet_b_length": ""}]

    def test_outlets_that_are_found_the_other_way_round_change_places(self):
        switch = self._measured(outlet_a="extruder", outlet_b="extruder1")
        switch._set_outlets({"A": "extruder1", "B": "extruder"})
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}
        assert lane_of(switch, "Bambu_AMS_1").extruder_obj.name == "extruder1"
        assert lane_of(switch, "Bambu_AMS_2").extruder_obj.name == "extruder"

    @pytest.mark.parametrize("found_now", [{"A": "extruder1", "B": "extruder"},
                                           {"A": "extruder1"}, {}])
    def test_outlets_that_are_found_as_they_were_change_nothing(self, found_now):
        switch = self._measured(outlet_a="extruder1", outlet_b="extruder")
        kept = dict(switch._saved_lanes)
        switch._start_unit = MagicMock()
        switch._set_outlets(found_now)
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        assert chain_of(switch).saved == []
        assert (switch._saved_lanes, switch._by_print, switch._due_back) == (
            kept, ["lane_Bambu_AMS_1"], ["lane_Bambu_AMS_2"])
        switch._start_unit.assert_not_called()
        switch.afc.save_vars.assert_not_called()

    def test_an_outlet_the_chains_section_sets_is_not_saved(self):
        switch = self._measured(outlet_b="extruder", by_hand=["outlet_b"])
        switch._set_outlets({"A": "extruder1"})
        assert switch.outlets == {"A": "extruder1", "B": "extruder"}
        # Its tube was measured with the other outlet's, so that length goes too.
        assert switch.lengths == {"A": None, "B": None}
        assert chain_of(switch).saved == [{"outlet_a": "extruder1", "outlet_a_length": "",
                                           "outlet_b_length": ""}]

    def test_a_tube_length_the_chains_section_sets_is_kept_and_not_saved(self):
        switch = self._measured(by_hand=["outlet_b_length"])
        switch._set_outlets({"A": "extruder1", "B": "extruder"})
        assert switch.lengths == {"A": None, "B": 1010.0}
        assert chain_of(switch).saved == [{"outlet_a": "extruder1", "outlet_a_length": "",
                                           "outlet_b": "extruder"}]

    def test_a_unit_that_has_not_connected_is_not_started(self):
        switch = self._measured()
        first = ams_of(switch, "Bambu_AMS_1")
        del switch.afc.units["Bambu_AMS_2"]
        switch._start_unit = MagicMock()
        switch._set_outlets({"A": "extruder1", "B": "extruder"})
        assert switch._start_unit.call_args_list == [call(first)]


class TestAfcBridgeBoxSwitchForget:
    def test_what_was_found_by_loading_is_forgotten_and_saved_so(self):
        switch = known()
        switch.confirmed = ["Bambu_AMS_1"]
        assert switch._found == {"AAAA": "A", "BBBB": "B"}
        switch.forget()
        assert switch._found == {}
        assert switch.confirmed == []
        switch.afc.save_vars.assert_called_once_with()


class TestAfcBridgeBoxSwitchGetStatus:
    def test_an_answering_switch(self):
        switch = plumbed(FakeChain(state=fs(route=2, b=True)), outlet_a_length=850)
        lane_of(switch, "Ace2_1").tool_loaded = True
        switch.confirmed = ["Ace2_1"]
        status = switch.get_status(0.)
        assert status == {
            "chain": "Chain_1", "answering": True, "calibrated": False,
            "inlets": {"A": {"outlet": "A", "target": "A", "filament": False,
                             "units": ["Bambu_AMS_1", "Turtle_1"], "lane": None},
                       "B": {"outlet": "B", "target": "B", "filament": True,
                             "units": ["Ace2_1"], "lane": "lane_Ace2_1"}},
            "outlets": {"A": "extruder", "B": "extruder1"},
            "lengths": {"A": 850.0, "B": None},
            "lanes": {"lane_Bambu_AMS_1": "extruder", "lane_Turtle_1": "extruder",
                      "lane_Ace2_1": "extruder1"},
            "unconfirmed": ["Bambu_AMS_1", "Turtle_1"], "unfound": []}
        # Fresh copies, so nothing a reader changes reaches the switch.
        status["inlets"]["A"]["units"].clear()
        status["outlets"].clear()
        status["lengths"].clear()
        assert switch.inlets["A"] == ["Bambu_AMS_1", "Turtle_1"]
        assert switch.outlets == {"A": "extruder", "B": "extruder1"}
        assert switch.lengths == {"A": 850.0, "B": None}

    def test_a_calibrated_switch_with_a_lane_on_the_other_extruder(self):
        switch = plumbed(FakeChain(state=fs(route=0, a=True)), **LENGTHS)
        lane = lane_of(switch, "Turtle_1")
        lane.set_extruder(extruder_of(switch, "extruder1"))
        lane.tool_loaded = True
        switch.confirmed = ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]
        assert switch.get_status() == {
            "chain": "Chain_1", "answering": True, "calibrated": True,
            "inlets": {"A": {"outlet": "B", "target": "B", "filament": True,
                             "units": ["Bambu_AMS_1", "Turtle_1"], "lane": "lane_Turtle_1"},
                       "B": {"outlet": "B", "target": "B", "filament": False,
                             "units": ["Ace2_1"], "lane": None}},
            "outlets": {"A": "extruder", "B": "extruder1"},
            "lengths": {"A": 850.0, "B": 1010.0},
            "lanes": {"lane_Bambu_AMS_1": "extruder", "lane_Turtle_1": "extruder1",
                      "lane_Ace2_1": "extruder1"},
            "unconfirmed": [], "unfound": []}

    def test_bays_that_hold_no_unit_are_left_out_with_their_lanes(self):
        switch = plumbed(FakeChain(state=fs(route=2)))
        switch.afc.units["Bambu_AMS_1"].pool = True
        switch.afc.units["Ace2_1"].pool = True
        assert switch.get_status() == {
            "chain": "Chain_1", "answering": True, "calibrated": False,
            "inlets": {"A": {"outlet": "A", "target": "A", "filament": False,
                             "units": ["Turtle_1"], "lane": None},
                       "B": {"outlet": "B", "target": "B", "filament": False,
                             "units": [], "lane": None}},
            "outlets": {"A": "extruder", "B": "extruder1"},
            "lengths": {"A": None, "B": None},
            "lanes": {"lane_Turtle_1": "extruder"},
            "unconfirmed": ["Turtle_1"], "unfound": []}

    def test_a_switch_that_is_not_answering(self):
        switch = make_switch(make_printer([FakeChain(state=None)]))
        assert switch.get_status() == {
            "chain": "Chain_1", "answering": False, "calibrated": False,
            "inlets": {"A": {"outlet": None, "filament": None,
                             "units": ["Bambu_AMS_1", "Turtle_1"], "lane": None},
                       "B": {"outlet": None, "filament": None, "units": ["Ace2_1"],
                             "lane": None}},
            "outlets": {"A": "extruder", "B": "extruder1"},
            "lengths": {"A": None, "B": None},
            "lanes": {},
            "unconfirmed": ["Bambu_AMS_1", "Turtle_1", "Ace2_1"], "unfound": []}

    def test_the_units_whose_inlet_is_not_known_are_listed_apart_without_their_lanes(self):
        switch = finding(SimChain(route=2))
        found(switch, Bambu_AMS_2="B")
        assert switch.get_status() == {
            "chain": "Chain_1", "answering": True, "calibrated": False,
            "inlets": {"A": {"outlet": "A", "target": "A", "filament": False, "units": [],
                             "lane": None},
                       "B": {"outlet": "B", "target": "B", "filament": False,
                             "units": ["Bambu_AMS_2"], "lane": None}},
            "outlets": {"A": "extruder", "B": "extruder1"},
            "lengths": {"A": None, "B": None},
            "lanes": {"lane_Bambu_AMS_2": "extruder1"},
            "unconfirmed": ["Bambu_AMS_2"], "unfound": ["Bambu_AMS_1"]}


class TestLoadConfigPrefix:
    def test_it_builds_a_switch(self):
        printer = make_printer()
        switch = load_config_prefix(MockConfig(name=SECTION, printer=printer, values=VALUES))
        assert isinstance(switch, afcBridgeBoxSwitch)
        assert switch.name == "Switch_1"
        assert LETTERS == ("A", "B")
