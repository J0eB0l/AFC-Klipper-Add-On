"""Unit tests for extras/AFC_BridgeBox_switch.py"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Optional, Tuple
from unittest.mock import MagicMock, call

import pytest
from configfile import error

from extras import AFC_BridgeBox_switch as switch_mod
from extras.AFC_BridgeBox import afcBridgeBox
from extras.AFC_BridgeBox_switch import (
    LETTERS,
    SwitchError,
    afcBridgeBoxSwitch,
    load_config_prefix,
    other,
    outlet_at,
    route_with,
)
from extras.AFC_lane import AFCLane
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

    def unit_names(self) -> List[str]:
        return list(self.bays)

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


class FakeHub:
    """A hub with only the tube lengths the switch changes."""

    def __init__(self, name: str, length: float = 900.0, unload: float = 880.0) -> None:
        self.fullname = f"AFC_hub {name}"
        self.afc_bowden_length = length
        self.afc_unload_bowden_length = unload


class FakeExtruder:
    """An AFC extruder with only what a lane and the switch read."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.th_extruder_name = f"th_{name}"
        self.lanes: Dict[str, Any] = {}
        self.check_lanes = MagicMock()


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


def tube_of(hub: FakeHub) -> Tuple[float, float]:
    """:return tuple: the load and unload lengths a hub holds now"""
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


def lane_of(switch: afcBridgeBoxSwitch, unit: str) -> FakeLane:
    """:return FakeLane: the lane ``plumbed`` made for a unit"""
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


class TestOther:
    def test_each_letter_gives_the_other(self):
        assert (other("A"), other("B")) == ("B", "A")


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
        assert switch.confirmed == []
        assert switch._offsets == {}
        assert switch._seen == {"A": False, "B": False}
        assert (switch._moving, switch._told) == (False, "")

    def test_it_registers_its_handlers_and_commands(self):
        printer = make_printer()
        switch = make_switch(printer, connect=False)
        assert printer._event_handlers == {
            "klippy:connect": [switch.handle_connect],
            "klippy:ready": [switch.handle_ready],
            "afc:prep": [switch.handle_prep],
            "afc:tool_load_prepare": [switch.handle_tool_load_prepare],
            "afc:tool_loaded": [switch.handle_tool_loaded],
            "afc:save_vars": [switch.handle_save_vars],
            "afc:calibration_buttons": [switch.handle_calibration_buttons]}
        route, assign = "AFC_BRIDGEBOX_SWITCH_ROUTE", "AFC_BRIDGEBOX_SWITCH_ASSIGN"
        calibration = "AFC_BRIDGEBOX_SWITCH_CALIBRATION"
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
                 desc=switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION_help)]

    def test_a_bad_section_is_refused(self):
        with pytest.raises(Exception) as err:
            make_switch(connect=False, inlet_a=None, inlet_b=None)
        assert str(err.value) == f"[{SECTION}] names no unit. Add inlet_a or inlet_b."


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
        (dict(inlet_b=["Turtle_1", "Bambu_AMS_1"]),
         "has Bambu_AMS_1, Turtle_1 on both inlets. A unit enters one inlet."),
        (dict(outlet_a=None, outlet_b=None), "names no extruder. Add outlet_a or outlet_b."),
        (dict(outlet_b="extruder"),
         "has extruder on both outlets. An outlet feeds one extruder."),
    ])
    def test_what_it_refuses(self, values, said):
        with pytest.raises(Exception) as err:
            make_switch(connect=False, **values)
        assert str(err.value) == f"[{SECTION}] {said}"

    @pytest.mark.parametrize("values", [
        dict(inlet_a=None), dict(inlet_b=None), dict(outlet_a=None), dict(outlet_b=None), {}])
    def test_what_it_takes(self, values):
        make_switch(connect=False, **values)._check_section()


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
            "klippy:connect", "klippy:ready", "afc:prep", "afc:tool_load_prepare",
            "afc:tool_loaded", "afc:save_vars", "afc:calibration_buttons"]

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

    def test_the_saved_units_of_this_switch_are_confirmed_again(self, tmp_path):
        printer = make_printer()
        printer._afc.VarFile = str(tmp_path / "AFC.var")
        (tmp_path / "AFC.var.unit").write_text(json.dumps({"system": {"switches": {
            "Switch_1": {"lanes": {}, "confirmed": ["Turtle_1", "Gone_1", "Ace2_1"]}}}}))
        # A unit that was taken out of the section since is dropped.
        assert make_switch(printer).confirmed == ["Turtle_1", "Ace2_1"]

    def test_with_nothing_saved_no_lane_is_away_and_no_unit_confirmed(self):
        switch = plumbed()
        assert switch._saved_lanes == {}
        assert switch.confirmed == []

    def test_a_chain_named_on_an_inlet_stands_for_its_units(self):
        printer = make_printer([FakeChain(bays=["Bambu_AMS_1", "Turtle_1"])])
        lane = add_lane(printer, "lane1", "Turtle_1", extruder="extruder1")
        switch = make_switch(printer, inlet_a=["Chain_1"])
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Ace2_1"]}
        # The chain's units are known by the time the lanes are kept and started.
        assert switch.lanes == [lane]
        assert lane.extruder_obj.name == "extruder"

    def test_a_chain_on_one_inlet_and_a_unit_of_it_on_the_other_is_refused(self):
        printer = make_printer([FakeChain(bays=["Bambu_AMS_1", "Ace2_1"])])
        with pytest.raises(error) as err:
            make_switch(printer, inlet_a=["Chain_1"])
        assert str(err.value) == (f"[{SECTION}] has Ace2_1 on both inlets. A unit enters one "
                                  "inlet.")

    def test_a_section_whose_chains_have_no_bay_names_no_unit(self):
        with pytest.raises(error) as err:
            make_switch(inlet_a=["Chain_1"], inlet_b=None)
        assert str(err.value) == f"[{SECTION}] names no unit. Add inlet_a or inlet_b."


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
            "Switch_1": {"lanes": {"lane1": "extruder1"}, "confirmed": ["Turtle_1"]},
            "Switch_2": {"lanes": {"lane20": "extruder3"}, "confirmed": ["Other"]}}}}
        assert self._switch(tmp_path, json.dumps(saved))._saved() == {
            "lanes": {"lane1": "extruder1"}, "confirmed": ["Turtle_1"]}

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
        '"confirmed": 5}}}}'])
    def test_nothing_usable_saved_reads_as_nothing(self, tmp_path, text):
        assert self._switch(tmp_path, text)._saved() == {"lanes": {}, "confirmed": []}


class TestAfcBridgeBoxSwitchHandleSaveVars:
    def test_lanes_on_their_starting_extruder_are_not_listed(self):
        switch = plumbed()
        data: Dict[str, Any] = {"system": {}}
        switch.handle_save_vars(data)
        assert data == {"system": {"switches": {"Switch_1": {"lanes": {}, "confirmed": []}}}}

    def test_lanes_off_their_starting_extruder_are_listed(self):
        switch = plumbed()
        lane_of(switch, "Turtle_1").set_extruder(extruder_of(switch, "extruder1"))
        lane_of(switch, "Ace2_1").set_extruder(extruder_of(switch, "extruder"))
        data: Dict[str, Any] = {"system": {"switches": {"Switch_2": {"lane20": "extruder3"}}}}
        switch.handle_save_vars(data)
        assert data == {"system": {"switches": {
            "Switch_2": {"lane20": "extruder3"},
            "Switch_1": {"lanes": {"lane_Turtle_1": "extruder1", "lane_Ace2_1": "extruder"},
                         "confirmed": []}}}}

    def test_the_units_seen_at_their_inlet_are_listed_in_a_list_of_their_own(self):
        switch = plumbed()
        switch.confirmed = ["Turtle_1", "Ace2_1"]
        data: Dict[str, Any] = {"system": {}}
        switch.handle_save_vars(data)
        saved = data["system"]["switches"]["Switch_1"]
        assert saved == {"lanes": {}, "confirmed": ["Turtle_1", "Ace2_1"]}
        # What is saved does not grow with the switch's own list.
        assert saved["confirmed"] is not switch.confirmed


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
        switch.lanes = [mine, another]
        switch._start_unit(printer._objects["AFC_Unit Ace2_1"])
        assert mine.extruder_obj.name == "extruder1"
        assert another.extruder_obj.name == "extruder1"

    def test_a_saved_extruder_wins_over_the_start(self):
        printer = make_printer()
        lane = add_lane(printer, "lane9", "Ace2_1", extruder="extruder1")
        switch = make_switch(printer, connect=False)
        switch.lanes = [lane]
        switch._saved_lanes = {"lane9": "extruder"}
        switch._start_unit(printer._objects["AFC_Unit Ace2_1"])
        assert lane.extruder_obj.name == "extruder"

    def test_a_saved_extruder_the_switch_does_not_feed_is_ignored(self):
        printer = make_printer(extruders=["extruder", "extruder1", "extruder2"])
        lane = add_lane(printer, "lane9", "Ace2_1")
        switch = make_switch(printer, connect=False)
        switch.lanes = [lane]
        switch._saved_lanes = {"lane9": "extruder2"}
        switch._start_unit(printer._objects["AFC_Unit Ace2_1"])
        assert lane.extruder_obj.name == "extruder1"


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


class TestAfcBridgeBoxSwitchExpandChains:
    @staticmethod
    def _switch(chains: List[FakeChain], **values: Any) -> afcBridgeBoxSwitch:
        """:return afcBridgeBoxSwitch: a switch that has not connected, on Chain_1"""
        return make_switch(make_printer(chains), connect=False, chain="Chain_1", **values)

    def test_units_are_left_as_they_are(self):
        switch = self._switch([FakeChain(bays=["Bay_1", "Bay_2"])])
        switch._expand_chains()
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Ace2_1"]}

    def test_a_chain_is_replaced_by_its_units_where_it_stood(self):
        switch = self._switch([FakeChain(bays=["Bay_1", "Bay_2"])],
                              inlet_a=["Turtle_1", "Chain_1", "Bambu_AMS_1"])
        switch._expand_chains()
        assert switch.inlets == {"A": ["Turtle_1", "Bay_1", "Bay_2", "Bambu_AMS_1"],
                                 "B": ["Ace2_1"]}

    def test_a_chain_on_each_inlet(self):
        chains = [FakeChain("Chain_1", bays=["Bay_1"]), FakeChain("Chain_2", bays=["Bay_2"])]
        switch = self._switch(chains, inlet_a=["Chain_2"], inlet_b=["Chain_1"])
        switch._expand_chains()
        assert switch.inlets == {"A": ["Bay_2"], "B": ["Bay_1"]}

    def test_a_chain_name_that_is_also_a_unit_name_is_left_as_the_unit(self):
        switch = self._switch([FakeChain(), FakeChain("Turtle_1", bays=["Bay_1", "Bay_2"])])
        switch._expand_chains()
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Ace2_1"]}

    def test_only_an_afc_unit_of_that_name_keeps_a_chain_from_being_replaced(self):
        printer = make_printer([FakeChain(bays=["Bay_1"])])
        printer._objects["AFC_hub Chain_1"] = MagicMock()
        printer._objects["AFC_hub Chain_1"].name = "Chain_1"
        switch = make_switch(printer, connect=False, inlet_b=["Chain_1"])
        switch._expand_chains()
        assert switch.inlets["B"] == ["Bay_1"]

    def test_a_name_that_is_neither_is_left_for_the_check_of_what_is_missing(self):
        switch = self._switch([FakeChain(bays=["Bay_1"])], inlet_b=["Nothing_1"])
        switch._expand_chains()
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": ["Nothing_1"]}

    def test_a_chain_without_bays_leaves_nothing_on_its_inlet(self):
        switch = self._switch([FakeChain()], inlet_b=["Chain_1"])
        switch._expand_chains()
        assert switch.inlets == {"A": ["Bambu_AMS_1", "Turtle_1"], "B": []}


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
        switch._note_arrivals = MagicMock()
        switch._settle = MagicMock(return_value=["a problem"])
        switch._say = MagicMock()
        return switch

    @staticmethod
    def _only_looked(switch: afcBridgeBoxSwitch) -> None:
        """Assert the watch looked at the inlets and did nothing else."""
        switch._look.assert_called_once_with()
        switch._note_arrivals.assert_not_called()
        switch._settle.assert_not_called()
        switch._say.assert_not_called()

    def test_it_looks_notes_who_came_in_then_settles_and_says_while_holding_the_switch(self):
        switch = self._switch()
        held = []

        def settle() -> List[str]:
            held.append(("settle", switch._moving, switch._look.call_count))
            return ["a problem"]

        switch._note_arrivals.side_effect = lambda: held.append(("arrivals", switch._moving))
        switch._settle.side_effect = settle
        switch._say.side_effect = lambda problems: held.append(("say", switch._moving))
        assert switch._watch(55.0) == 101.0
        switch._look.assert_called_once_with()
        switch._say.assert_called_once_with(["a problem"])
        assert held == [("arrivals", False), ("settle", True, 1), ("say", True)]
        assert switch._moving is False

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
              "where it is. Unload it or pull it out.")
    HELD_B = ("Switch_1: inlet B holds filament and none of its lanes is loaded, so it is left "
              "where it is. Unload it or pull it out.")

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
                    "its lanes is loaded, so it is left where it is. Unload it or pull it "
                    "out.</span>")]
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
                    "its lanes is loaded, so it is left where it is. Unload it or pull it "
                    "out.</span>"),
            ("raw", "<span class=error--text>Switch_1: inlet B holds filament and none of "
                    "its lanes is loaded, so it is left where it is. Unload it or pull it "
                    "out.</span>")]
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

    def test_the_sensor_is_given_time_to_clear_after_the_unload(self):
        chain = SimChain(route=0, a=True)
        switch = plumbed(chain)
        held = lane_of(switch, "Bambu_AMS_1")
        held.set_extruder(extruder_of(switch, "extruder1"))
        held.tool_loaded = True
        switch.afc.TOOL_UNLOAD = MagicMock(return_value=True)
        polls = []

        def clears_on_the_third_poll(until: float) -> None:
            polls.append(until)
            if len(polls) == 3:
                chain.fill(a=False)

        switch.reactor.pause.side_effect = clears_on_the_third_poll
        assert switch._route(lane_of(switch, "Turtle_1"), "A", MagicMock()) is True
        # Three waits for the sensor, then one for the route to read back.
        assert polls == [100.1] * 4
        assert chain.routes == [2]

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
            "Outlet B: extruder1 (lane_Turtle_1, lane_Ace2_1)")

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

    def test_a_switch_that_is_not_answering(self):
        switch = plumbed(FakeChain(state=None))
        with pytest.raises(SwitchError) as err:
            switch._report()
        assert str(err.value) == "Switch_1: the filament switch is not answering"


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
        switch.afc.save_vars.assert_not_called()

    def test_a_lane_moves_to_the_other_extruder_and_the_choice_is_saved(self):
        switch = plumbed()
        lane = lane_of(switch, "Turtle_1")
        gcmd = MockGCodeCommand()
        switch.assign(lane, "extruder1", False, gcmd)
        assert lane.extruder_obj is extruder_of(switch, "extruder1")
        assert "lane_Turtle_1" in extruder_of(switch, "extruder1").lanes
        assert "lane_Turtle_1" not in extruder_of(switch, "extruder").lanes
        switch.afc.save_vars.assert_called_once_with()
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: lane_Turtle_1 now feeds extruder1")

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


class TestAfcBridgeBoxSwitchHandleCalibrationButtons:
    def test_its_button_is_added_after_the_ones_that_are_there(self):
        switch = make_switch()
        buttons = [("Turtle_1", "UNIT_CALIBRATION UNIT=Turtle_1", "primary")]
        switch.handle_calibration_buttons(buttons)
        assert buttons == [
            ("Turtle_1", "UNIT_CALIBRATION UNIT=Turtle_1", "primary"),
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
    def test_without_a_lane_it_opens_the_menu(self):
        switch = plumbed(**LENGTHS)
        switch._measure = MagicMock()
        gcmd = MockGCodeCommand()
        switch.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(gcmd)
        menu = TestAfcBridgeBoxSwitchCalibrationMenu
        assert switch.logger.messages == [menu.BEGIN, menu.NO_LANE] + menu.FOOT
        switch._measure.assert_not_called()
        assert switch.lengths == {"A": 850.0, "B": 1010.0}
        switch.afc.function.ConfigRewrite.assert_not_called()
        gcmd.respond_info.assert_not_called()

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
        assert switch.logger.messages == []

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
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: with lane_Ace2_1 the tube through outlet A is 851.9 mm and through "
            "outlet B 1012.1 mm")
        assert switch.logger.messages == []

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


class TestAfcBridgeBoxSwitchCalibrationMenu:
    BEGIN = ("raw", "// action:prompt_begin Switch_1 Calibration")
    PICK = ("raw", "// action:prompt_text Measures the tube from each outlet to its toolhead "
                   "with one lane, so a lane loads the right distance through either outlet. "
                   "Unload the toolheads, then pick the lane to measure with.")
    NO_LANE = ("raw", "// action:prompt_text No lane that can measure is loaded in its unit. "
                      "Load one, or set outlet_a_length and outlet_b_length in the switch's "
                      "section by hand.")
    FOOT = [("raw", "// action:prompt_footer_button Back to unit selection|AFC_CALIBRATION|info"),
            ("raw", "// action:prompt_footer_button Cancel|RESPOND TYPE=command "
                    "MSG=action:prompt_end|warning"),
            ("raw", "// action:prompt_show")]

    def test_one_button_for_each_lane_that_can_measure(self):
        switch = plumbed()
        for unit in ("Turtle_1", "Ace2_1"):
            lane_of(switch, unit).load_state = True
            add_hub(lane_of(switch, unit))
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [
            self.BEGIN, self.PICK,
            ("raw", "// action:prompt_button Turtle_1 lane_Turtle_1|"
                    "AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH=Switch_1 LANE=lane_Turtle_1|"
                    "primary"),
            ("raw", "// action:prompt_button Ace2_1 lane_Ace2_1|"
                    "AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH=Switch_1 LANE=lane_Ace2_1|"
                    "primary")] + self.FOOT

    def test_with_no_lane_that_can_measure_it_says_what_to_do(self):
        switch = plumbed()
        switch._calibration_menu(MockGCodeCommand())
        assert switch.logger.messages == [self.BEGIN, self.NO_LANE] + self.FOOT


class TestAfcBridgeBoxSwitchMeasuringLanes:
    def test_no_lane_is_loaded_in_its_unit(self):
        switch = plumbed()
        for lane in switch.lanes:
            add_hub(lane)
        assert switch._measuring_lanes() == []

    def test_a_lane_that_is_loaded_in_its_unit_and_has_a_hub(self):
        switch = plumbed()
        lane = lane_of(switch, "Ace2_1")
        lane.load_state = True
        add_hub(lane)
        assert switch._measuring_lanes() == [lane]

    def test_a_loaded_lane_without_a_hub_cannot_measure(self):
        switch = plumbed()
        lane_of(switch, "Ace2_1").load_state = True
        assert switch._measuring_lanes() == []

    def test_the_first_lane_of_each_unit_that_can(self):
        printer = make_printer([SimChain()])
        lanes = [add_lane(printer, name, unit) for name, unit in (
            ("lane1", "Turtle_1"), ("lane2", "Turtle_1"), ("lane3", "Turtle_1"),
            ("lane9", "Ace2_1"))]
        switch = make_switch(printer)
        for lane in lanes:
            add_hub(lane)
        for lane in lanes[1:]:
            lane.load_state = True
        assert switch._measuring_lanes() == [lanes[1], lanes[3]]


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

    def test_an_inlet_that_cannot_be_pointed_stops_before_the_unit_feeds(self):
        switch, lane = self._switch(SimChain(route=2, a=True))
        measuring(switch, lane, Run(1013.0))
        with pytest.raises(SwitchError) as err:
            switch._measure_outlet(lane, "A", "B", MockGCodeCommand())
        assert str(err.value) == ("Switch_1: inlet A holds filament, so it cannot be moved to "
                                  "outlet B")
        lane.unit_obj.calibrate_bowden.assert_not_called()
        assert switch._moving is False


class TestAfcBridgeBoxSwitchRestoreTube:
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
            "unconfirmed": ["Bambu_AMS_1", "Turtle_1"]}
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
            "unconfirmed": []}

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
            "unconfirmed": ["Turtle_1"]}

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
            "unconfirmed": ["Bambu_AMS_1", "Turtle_1", "Ace2_1"]}


class TestLoadConfigPrefix:
    def test_it_builds_a_switch(self):
        printer = make_printer()
        switch = load_config_prefix(MockConfig(name=SECTION, printer=printer, values=VALUES))
        assert isinstance(switch, afcBridgeBoxSwitch)
        assert switch.name == "Switch_1"
        assert LETTERS == ("A", "B")
