"""Unit tests for extras/AFC_BridgeBox_switch.py"""

from __future__ import annotations

from typing import Any, Dict, List, Optional
from unittest.mock import MagicMock

import pytest

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
from extras.AFC_unit import afcUnit
from tests.conftest import CommandError, MockAFC, MockConfig, MockGCodeCommand, MockPrinter

SECTION = "AFC_BridgeBox_switch Switch_1"
VALUES = {"inlet_a": ["Bambu_AMS_1", "Turtle_1"], "inlet_b": ["Ace2_1"],
          "outlet_a": "extruder", "outlet_b": "extruder1"}


def fs(route: int = 0, position: Optional[int] = None, a: bool = False, b: bool = False,
       on: bool = True) -> Dict[str, Any]:
    """:return dict: a chain's switch state, its sensors given by inlet letter"""
    return {"on": on, "route": route, "position": route if position is None else position,
            "inlets": [b, a], "values": []}


class FakeChain(afcBridgeBox):
    """A chain that reports a switch state and records the routes it is sent."""

    def __init__(self, name: str = "Chain_1", state: Optional[Dict[str, Any]] = None,
                 linked: bool = True, then: Optional[List[Any]] = None) -> None:
        self.name = name
        self.fs = state
        self.linked = linked
        # States reported one per poll once a route is sent. The last one repeats.
        self.then = list(then or [])
        self.routes: List[int] = []
        self.polls = 0

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


class FakeUnit(afcUnit):
    """An AFC unit with only a name."""

    def __init__(self, name: str) -> None:
        self.name = name


def make_printer(chains: Optional[List[FakeChain]] = None,
                 units: Optional[List[str]] = None,
                 extruders: Optional[List[str]] = None) -> MockPrinter:
    """:return MockPrinter: a printer holding these chains, units and extruders"""
    printer = MockPrinter(afc=MockAFC())
    printer._gcode.register_mux_command = MagicMock()
    printer._reactor.pause = MagicMock()
    for chain in [FakeChain()] if chains is None else chains:
        printer._objects[f"AFC_BridgeBox {chain.name}"] = chain
    for unit in VALUES["inlet_a"] + VALUES["inlet_b"] if units is None else units:
        printer._objects[f"AFC_Unit {unit}"] = FakeUnit(unit)
    for name in ["extruder", "extruder1"] if extruders is None else extruders:
        printer._objects[f"AFC_extruder {name}"] = MagicMock()
    return printer


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

    def test_the_chain_is_kept_as_named(self):
        assert make_switch(connect=False, chain="Chain_2").chain_name == "Chain_2"

    def test_blank_unit_names_are_dropped(self):
        switch = make_switch(connect=False, inlet_a=[" Turtle_1 ", "", " "])
        assert switch.inlets["A"] == ["Turtle_1"]

    def test_an_inlet_and_an_outlet_may_be_left_out(self):
        switch = make_switch(connect=False, inlet_b=None, outlet_a=None)
        assert switch.inlets["B"] == []
        assert switch.outlets["A"] is None

    def test_it_registers_its_connect_handler_and_command(self):
        printer = make_printer()
        switch = make_switch(printer, connect=False)
        assert printer._event_handlers["klippy:connect"] == [switch.handle_connect]
        calls = printer._gcode.register_mux_command.call_args_list
        assert [call.args[:3] for call in calls] == [
            ("AFC_BRIDGEBOX_SWITCH_ROUTE", "SWITCH", "Switch_1"),
            ("AFC_BRIDGEBOX_SWITCH_ROUTE", "SWITCH", None)]
        for call in calls:
            assert call.args[3] == switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE
            assert call.kwargs == {"desc": switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE_help}

    def test_a_bad_section_is_refused(self):
        with pytest.raises(Exception) as err:
            make_switch(connect=False, inlet_a=None, inlet_b=None)
        assert str(err.value) == f"[{SECTION}] names no unit. Add inlet_a or inlet_b."


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

    def test_the_route_is_sent_again_when_the_position_is_not_there_yet(self):
        switch = self._switch(fs(route=2, position=0), then=[fs(route=2)])
        assert switch.set_inlet("A", "A") is True
        assert chain_of(switch).routes == [2]

    def test_the_route_is_sent_when_only_the_position_is_there(self):
        switch = self._switch(fs(route=0, position=2), then=[fs(route=2)])
        assert switch.set_inlet("A", "A") is True
        assert chain_of(switch).routes == [2]

    def test_an_inlet_that_holds_filament_is_not_moved(self):
        switch = self._switch(fs(route=0, a=True))
        with pytest.raises(SwitchError) as err:
            switch.set_inlet("A", "A")
        assert str(err.value) == ("Switch_1: inlet A holds filament, so it cannot be moved to "
                                  "outlet A")
        assert chain_of(switch).routes == []

    @pytest.mark.parametrize("state", [fs(route=1, b=True), fs(route=1, position=0, b=True),
                                       fs(route=0, position=1, b=True)])
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
        assert str(err.value) == "Switch_1: inlet A did not reach outlet A"
        assert chain_of(switch).polls == switch_mod._ROUTE_POLLS == 50

    def test_a_switch_that_is_not_answering(self):
        switch = self._switch(fs(on=False))
        with pytest.raises(SwitchError) as err:
            switch.set_inlet("A", "A")
        assert str(err.value) == "Switch_1: the filament switch is not answering"


class TestAfcBridgeBoxSwitchDescribe:
    def test_one_line_for_each_inlet(self):
        switch = make_switch(make_printer([FakeChain(state=fs(route=2, a=True))]))
        assert switch._describe() == (
            "Switch_1: inlet A (Bambu_AMS_1, Turtle_1) exits outlet A (extruder), holds "
            "filament\n"
            "Switch_1: inlet B (Ace2_1) exits outlet B (extruder1), empty")

    def test_an_inlet_without_units_and_an_outlet_without_an_extruder(self):
        printer = make_printer([FakeChain(state=fs(route=0))])
        switch = make_switch(printer, inlet_b=None, outlet_b=None)
        assert switch._describe() == (
            "Switch_1: inlet A (Bambu_AMS_1, Turtle_1) exits outlet B (no extruder), empty\n"
            "Switch_1: inlet B (no unit) exits outlet B (no extruder), empty")


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
            "Switch_1: inlet A (Bambu_AMS_1, Turtle_1) exits outlet B (extruder1), empty\n"
            "Switch_1: inlet B (Ace2_1) exits outlet B (extruder1), empty")
        assert chain_of(switch).routes == []

    def test_it_moves_an_inlet(self):
        switch = self._switch(fs(route=0), then=[fs(route=2)])
        gcmd = MockGCodeCommand({"INLET": "a", "OUTLET": "a"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: inlet A now exits outlet A (extruder)")
        assert chain_of(switch).routes == [2]

    def test_an_inlet_already_there(self):
        switch = self._switch(fs(route=0), outlet_b=None)
        gcmd = MockGCodeCommand({"INLET": "B", "OUTLET": "B"})
        switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        gcmd.respond_info.assert_called_once_with(
            "Switch_1: inlet B already exits outlet B (no extruder)")
        assert chain_of(switch).routes == []

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

    def test_a_report_from_a_switch_that_is_not_answering(self):
        switch = self._switch(fs(on=False))
        gcmd = MockGCodeCommand()
        with pytest.raises(CommandError) as err:
            switch.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(gcmd)
        assert str(err.value) == "Switch_1: the filament switch is not answering"


class TestAfcBridgeBoxSwitchGetStatus:
    def test_an_answering_switch(self):
        switch = make_switch(make_printer([FakeChain(state=fs(route=2, b=True))]))
        status = switch.get_status(0.)
        assert status == {
            "chain": "Chain_1", "answering": True,
            "inlets": {"A": {"outlet": "A", "target": "A", "filament": False,
                             "units": ["Bambu_AMS_1", "Turtle_1"]},
                       "B": {"outlet": "B", "target": "B", "filament": True,
                             "units": ["Ace2_1"]}},
            "outlets": {"A": "extruder", "B": "extruder1"}}
        # Fresh copies, so nothing a reader changes reaches the switch.
        status["inlets"]["A"]["units"].clear()
        status["outlets"].clear()
        assert switch.inlets["A"] == ["Bambu_AMS_1", "Turtle_1"]
        assert switch.outlets == {"A": "extruder", "B": "extruder1"}

    def test_a_switch_that_is_not_answering(self):
        switch = make_switch(make_printer([FakeChain(state=None)]))
        assert switch.get_status() == {
            "chain": "Chain_1", "answering": False,
            "inlets": {"A": {"outlet": None, "filament": None,
                             "units": ["Bambu_AMS_1", "Turtle_1"]},
                       "B": {"outlet": None, "filament": None, "units": ["Ace2_1"]}},
            "outlets": {"A": "extruder", "B": "extruder1"}}


class TestLoadConfigPrefix:
    def test_it_builds_a_switch(self):
        printer = make_printer()
        switch = load_config_prefix(MockConfig(name=SECTION, printer=printer, values=VALUES))
        assert isinstance(switch, afcBridgeBoxSwitch)
        assert switch.name == "Switch_1"
        assert LETTERS == ("A", "B")
