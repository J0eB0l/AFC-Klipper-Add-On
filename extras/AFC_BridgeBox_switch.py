# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""
A filament switch on a BridgeBox chain's bus. It has two inlets and two outlets. Each inlet takes
the units plumbed into it and each outlet feeds one extruder, so a lane of those units can be
loaded into either extruder.

    [AFC_BridgeBox_switch Switch_1]
    chain: Chain_1
    inlet_a: Bambu_AMS_1, Turtle_1
    inlet_b: Ace2_1
    outlet_a: extruder
    outlet_b: extruder1
    outlet_a_length: 850
    outlet_b_length: 1010

The letters are the ones printed on the switch. `chain` may be left out when there is one chain.
An inlet may name a BridgeBox chain in place of a unit, for every unit that chain makes for its
bays.

An inlet's units start on the extruder of the outlet with the same letter, and an empty inlet rests
on that outlet. A lane goes to the other extruder with `AFC_BRIDGEBOX_SWITCH_ASSIGN`, and every
load points the lane's inlet at its extruder's outlet before filament moves.

`outlet_a_length` and `outlet_b_length` are the tube from each outlet to its toolhead in mm. Only
their difference is used: it is added to a lane's tube length while the lane goes through the
outlet its unit does not start on. `AFC_BRIDGEBOX_SWITCH_CALIBRATION` measures them, and they may
be set by hand.
"""

from __future__ import annotations

import json
import re
from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, Dict, Iterator, List, Optional, Tuple

from configfile import error

from extras.AFC import State
from extras.AFC_BridgeBox import afcBridgeBox
from extras.AFC_lane import AFCLane
from extras.AFC_respond import AFCprompt
from extras.AFC_unit import afcUnit

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand

    from extras.AFC_extruder import AFCExtruder
    from extras.AFC_utils import AFCGcmd

#: The letters printed on the switch, for its inlets and for its outlets.
LETTERS = ("A", "B")

#: Route bit that sends each inlet to outlet A. With its bit clear an inlet exits outlet B.
_TO_OUTLET_A = {"A": 2, "B": 1}

#: Where each inlet's filament sensor sits in the chain's `inlets` list.
_SENSOR = {"A": 1, "B": 0}

#: Status polls, 0.1 s apart, a route change may take before the switch reports it.
_ROUTE_POLLS = 50

#: Status polls, 0.1 s apart, an inlet's sensor is given to read empty after an unload.
_CLEAR_POLLS = 30

#: Seconds a route is held so the switch sees it before the next one is sent.
_ROUTE_HOLD = 0.5

#: Seconds between looks at the switch while nothing is being loaded.
_WATCH_PERIOD = 1.0
# The most seconds the switch's section waits for a unit that is on its way into AFC.
_REPORT_WAIT = 30.0

#: Step and tolerance, in mm, a unit's own tube length calibration is run with.
_MEASURE_STEP = 25.0
_MEASURE_TOLERANCE = 5.0


class SwitchError(Exception):
    """A route the switch cannot take. The message says why."""


def other(letter: str) -> str:
    """
    The other inlet, or the other outlet.

    :param letter: `A` or `B`
    :return str: `B` for `A`, `A` for `B`
    """
    return "B" if letter == "A" else "A"


def in_order(name: str) -> List[Any]:
    """
    A sort key that puts names in number order, so `lane8` comes before `lane12`.

    :param name: a lane name
    :return list: the name's text and numbers in turn
    """
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", name)]


def outlet_at(route: int, inlet: str) -> str:
    """
    The outlet an inlet exits on a route.

    :param route: route or actuator position, 0 to 3
    :param inlet: `A` or `B`
    :return str: `A` or `B`
    """
    return "A" if route & _TO_OUTLET_A[inlet] else "B"


def route_with(route: int, inlet: str, outlet: str) -> int:
    """
    A route changed so one inlet exits an outlet, the other inlet left as it is.

    :param route: route to start from, 0 to 3
    :param inlet: inlet to move, `A` or `B`
    :param outlet: outlet it should exit, `A` or `B`
    :return int: the new route, 0 to 3
    """
    bit = _TO_OUTLET_A[inlet]
    return route | bit if outlet == "A" else route & ~bit


class afcBridgeBoxSwitch:
    """One filament switch: which units enter each inlet and which extruder each outlet feeds."""

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Read the section and check it against itself.

        :param config: the `[AFC_BridgeBox_switch <name>]` section
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.afc = self.printer.load_object(config, "AFC")
        self.logger = self.afc.logger
        self.fullname: str = config.get_name()
        self.name: str = self.fullname.split()[-1]
        self.chain_name: Optional[str] = config.get("chain", None)
        self.chain: Optional[afcBridgeBox] = None
        self.inlets: Dict[str, List[str]] = {}
        self.outlets: Dict[str, Optional[str]] = {}
        # The tube from each outlet to its toolhead in mm. Only their difference is used.
        self.lengths: Dict[str, Optional[float]] = {}
        for letter in LETTERS:
            units = config.getlist(f"inlet_{letter.lower()}", [])
            self.inlets[letter] = [unit.strip() for unit in units if unit.strip()]
            self.outlets[letter] = config.get(f"outlet_{letter.lower()}", None)
            self.lengths[letter] = config.getfloat(f"outlet_{letter.lower()}_length", None,
                                                   above=0.)
        self._check_section()
        # The outlet each inlet's units start on: its own letter, else the only outlet.
        self.start: Dict[str, str] = {letter: letter if self.outlets[letter] is not None
                                      else other(letter) for letter in LETTERS}
        # The outlet each inlet rests on while it is empty.
        self.rest: Dict[str, str] = dict(self.start)
        self.lanes: List[AFCLane] = []
        # The lane each inlet was last pointed for, until it reaches its toolhead.
        self._routed: Dict[str, Optional[AFCLane]] = {letter: None for letter in LETTERS}
        self._saved_lanes: Dict[str, str] = {}
        # The units a lane of which has shown at its inlet.
        self.confirmed: List[str] = []
        # The mm added to each tube for the outlet its lane goes through.
        self._offsets: Dict[Any, float] = {}
        # The inlets that read filament since this was last cleared.
        self._seen: Dict[str, bool] = {letter: False for letter in LETTERS}
        self._moving = False
        self._told = ""
        # Reactor time the switch's section is printed by at the latest, None when none is due.
        self._report_by: Optional[float] = None
        # The units that were in AFC when the section was last printed.
        self._reported: List[str] = []

        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("afc:prep", self.handle_prep)
        self.printer.register_event_handler("afc:tool_load_prepare",
                                            self.handle_tool_load_prepare)
        self.printer.register_event_handler("afc:tool_loaded", self.handle_tool_loaded)
        self.printer.register_event_handler("afc:save_vars", self.handle_save_vars)
        self.printer.register_event_handler("afc:calibration_buttons",
                                            self.handle_calibration_buttons)
        self._register("AFC_BRIDGEBOX_SWITCH_ROUTE", self.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE,
                       self.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE_help)
        self._register("AFC_BRIDGEBOX_SWITCH_ASSIGN", self.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN,
                       self.cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN_help)
        self._register("AFC_BRIDGEBOX_SWITCH_CALIBRATION",
                       self.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION,
                       self.cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION_help)

    def _register(self, command: str, handler: Any, desc: str) -> None:
        """
        Register a command under this switch's name. The first switch also answers when
        SWITCH is left out.

        :param command: the command name
        :param handler: its handler
        :param desc: its help text
        """
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(command, "SWITCH", self.name, handler, desc=desc)
        try:
            gcode.register_mux_command(command, "SWITCH", None, handler, desc=desc)
        except error:
            pass

    def _check_section(self) -> None:
        """
        Refuse a section that names no unit, no extruder, a unit on both inlets or one extruder
        on both outlets.
        """
        error_str = ""
        both = [unit for unit in self.inlets["A"] if unit in self.inlets["B"]]
        if not (self.inlets["A"]
                or self.inlets["B"]):
            error_str = f"[{self.fullname}] names no unit. Add inlet_a or inlet_b."
        elif both:
            error_str = (f"[{self.fullname}] has {', '.join(both)} on both inlets. A unit "
                         "enters one inlet.")
        elif (self.outlets["A"] is None
              and self.outlets["B"] is None):
            error_str = f"[{self.fullname}] names no extruder. Add outlet_a or outlet_b."
        elif self.outlets["A"] == self.outlets["B"]:
            error_str = (f"[{self.fullname}] has {self.outlets['A']} on both outlets. An "
                         "outlet feeds one extruder.")
        if error_str:
            raise error(error_str)

    def handle_connect(self) -> None:
        """
        Find the chain, check the units and extruders exist and no other switch uses them, then
        put each unit's lanes on their extruder.
        """
        self.chain = self._find_chain()
        self.chain_name = self.chain.name
        self._expand_chains()
        self._check_section()
        problems = self._missing() + self._clashes()
        if problems:
            raise error(problems[0])
        self.lanes = [obj for _name, obj in self.printer.lookup_objects()
                      if (isinstance(obj, AFCLane)
                          and self.inlet_of(obj.unit) is not None)]
        saved = self._saved()
        self._saved_lanes = saved["lanes"]
        self.confirmed = [unit for unit in saved["confirmed"] if unit in self.units()]
        for unit in self.units():
            if unit in self.afc.units:
                self._start_unit(self.afc.units[unit])
            else:
                # Registered now, so it runs after the unit's lanes took the unit's extruder.
                self.printer.register_event_handler(f"AFC_unit_{unit}:connect",
                                                    self._start_unit)

    def handle_ready(self) -> None:
        """
        Start watching the switch.
        """
        self.reactor.register_timer(self._watch, self.reactor.monotonic() + _WATCH_PERIOD)

    def _saved(self) -> Dict[str, Any]:
        """
        What AFC saved for this switch.

        :return dict: `lanes`, lane name -> extruder name for the lanes off the extruder their
        inlet starts on, and `confirmed`, the units seen at their inlet. Both empty when
        nothing usable is saved
        """
        try:
            with open(f"{self.afc.VarFile}.unit") as file:
                saved = json.load(file)["system"]["switches"][self.name]
            return {"lanes": dict(saved["lanes"]), "confirmed": list(saved["confirmed"])}
        except (OSError, ValueError, KeyError, TypeError):
            return {"lanes": {}, "confirmed": []}

    def handle_save_vars(self, data: Dict[str, Any]) -> None:
        """
        Add the lanes that are off their starting extruder, and the units seen at their inlet,
        to what AFC saves.

        :param data: what AFC is about to save
        """
        away = {lane.name: lane.extruder_obj.name for lane in self.lanes
                if lane.extruder_obj.name != self.start_extruder(lane.unit)}
        data["system"].setdefault("switches", {})[self.name] = {
            "lanes": away, "confirmed": list(self.confirmed)}

    def start_extruder(self, unit: str) -> str:
        """
        The extruder a unit's lanes start on: the one its inlet's own outlet feeds.

        :param unit: a unit of this switch
        :return str: AFC extruder name
        """
        return self.outlets[self.start[self.inlet_of(unit)]]

    def _extruder(self, name: str) -> AFCExtruder:
        """
        One of the switch's extruders.

        :param name: AFC extruder name
        :return AFCExtruder: the extruder
        """
        return self.printer.lookup_object(f"AFC_extruder {name}")

    def _start_unit(self, unit_obj: afcUnit) -> None:
        """
        Put a unit's lanes on the extruder they were saved on, else on the unit's starting
        extruder, whatever extruder the unit itself names.

        :param unit_obj: a unit of this switch that has connected
        """
        start = self.start_extruder(unit_obj.name)
        for lane in self.lanes:
            if lane.unit != unit_obj.name:
                continue
            saved = self._saved_lanes.get(lane.name, start)
            lane.set_extruder(self._extruder(saved if saved in self.extruders() else start))

    def _chains(self) -> Dict[str, afcBridgeBox]:
        """
        The BridgeBox chains that are configured.

        :return dict: chain name -> chain
        """
        return {obj.name: obj for _name, obj in self.printer.lookup_objects("AFC_BridgeBox")
                if isinstance(obj, afcBridgeBox)}

    def _expand_chains(self) -> None:
        """
        Replace a chain named on an inlet with the units that chain makes for its bays. A unit
        of that name is left as the unit.
        """
        chains = self._chains()
        known = [obj.name for _name, obj in self.printer.lookup_objects()
                 if isinstance(obj, afcUnit)]
        for letter in LETTERS:
            units: List[str] = []
            for name in self.inlets[letter]:
                if (name in chains
                    and name not in known):
                    units += chains[name].unit_names()
                else:
                    units.append(name)
            self.inlets[letter] = units

    def _find_chain(self) -> afcBridgeBox:
        """
        The chain whose bus the switch is on: the one `chain` names, else the only one.

        :return afcBridgeBox: the chain
        """
        chains = self._chains()
        name = self.chain_name
        error_str = ""
        if (name is None
            and len(chains) == 1):
            name = next(iter(chains))
        elif (name is None
              and chains):
            error_str = (f"[{self.fullname}] needs chain: there are {len(chains)} BridgeBox "
                         f"chains ({', '.join(chains)}).")
        elif name is None:
            error_str = f"[{self.fullname}] needs a BridgeBox chain, and none is configured."
        elif name not in chains:
            error_str = (f"[{self.fullname}] names chain {name}, and there is no "
                         f"[AFC_BridgeBox {name}] chain.")
        if error_str:
            raise error(error_str)
        return chains[name]

    def _missing(self) -> List[str]:
        """
        The units and extruders the section names that are not configured.

        :return list: one message per missing unit or extruder
        """
        units = [obj.name for _name, obj in self.printer.lookup_objects()
                 if isinstance(obj, afcUnit)]
        problems = [f"[{self.fullname}] names unit {name}, and there is no AFC unit of that name."
                    for name in self.units() if name not in units]
        problems += [f"[{self.fullname}] names extruder {name}, and there is no "
                     f"[AFC_extruder {name}]."
                     for name in self.extruders()
                     if self.printer.lookup_object(f"AFC_extruder {name}", None) is None]
        return problems

    def _clashes(self) -> List[str]:
        """
        What this switch shares with another one: its chain, a unit or an extruder.

        :return list: one message per clash
        """
        problems = []
        for _name, switch in self.printer.lookup_objects("AFC_BridgeBox_switch"):
            if switch is self:
                continue
            shared = ([unit for unit in self.units() if unit in switch.units()]
                      + [name for name in self.extruders() if name in switch.extruders()])
            if switch._find_chain() is self.chain:
                problems.append(f"[{self.fullname}] and [{switch.fullname}] are both on chain "
                                f"{self.chain_name}. A chain's bus carries one filament switch.")
            if shared:
                problems.append(f"[{self.fullname}] and [{switch.fullname}] both name "
                                f"{', '.join(shared)}.")
        return problems

    def units(self) -> List[str]:
        """
        Every unit plumbed into the switch.

        :return list: unit names, inlet A's first
        """
        return self.inlets["A"] + self.inlets["B"]

    def live_units(self) -> List[str]:
        """
        The units plumbed into the switch that are in use: all but a chain's bays that hold no
        unit now.

        :return list: unit names, inlet A's first
        """
        return [unit for unit in self.units()
                if not getattr(self.afc.units.get(unit), "pool", False)]

    def extruders(self) -> List[str]:
        """
        Every extruder the switch feeds.

        :return list: extruder names, outlet A's first
        """
        return [name for name in (self.outlets["A"], self.outlets["B"]) if name is not None]

    def inlet_of(self, unit: str) -> Optional[str]:
        """
        The inlet a unit enters.

        :param unit: unit name
        :return str: `A` or `B`, None for a unit not on this switch
        """
        return next((letter for letter in LETTERS if unit in self.inlets[letter]), None)

    def outlet_of(self, extruder: str) -> Optional[str]:
        """
        The outlet that feeds an extruder.

        :param extruder: AFC extruder name
        :return str: `A` or `B`, None for an extruder not on this switch
        """
        return next((letter for letter in LETTERS if self.outlets[letter] == extruder), None)

    def state(self) -> Dict[str, Any]:
        """
        The switch as it last reported, by inlet letter.

        :return dict: `route`, and per inlet the `outlet` it exits now, the `target` outlet of
        the route and whether it holds `filament`
        """
        st = self.chain.switch_state() if self.chain is not None else None
        if (st is None
            or not st["on"]):
            error_str = f"{self.name}: the filament switch is not answering"
            raise SwitchError(error_str)
        return {"route": st["route"],
                "inlets": {letter: {"outlet": outlet_at(st["position"], letter),
                                    "target": outlet_at(st["route"], letter),
                                    "filament": bool(st["inlets"][_SENSOR[letter]])}
                           for letter in LETTERS}}

    def set_inlet(self, inlet: str, outlet: str) -> bool:
        """
        Point an inlet at an outlet and wait until the switch reports it there.

        The switch cannot move a path that holds filament, so that is refused, and so is an
        outlet that holds the other inlet's filament. A path that is at the outlet already only
        has its route sent again.

        :param inlet: inlet to move, `A` or `B`
        :param outlet: outlet it should exit, `A` or `B`
        :return bool: True when the path moved, False when it was at the outlet already
        """
        st = self.state()
        here = st["inlets"][inlet]
        there = st["inlets"][other(inlet)]
        route = st["route"]
        if here["outlet"] == outlet:
            # Only the route is off, as after a BridgeBox restart under a loaded path.
            if here["target"] != outlet:
                self._send(route_with(route, inlet, outlet), inlet)
            return False
        error_str = ""
        if here["filament"]:
            error_str = (f"{self.name}: inlet {inlet} holds filament, so it cannot be moved "
                         f"to outlet {outlet}")
        elif (there["filament"]
              and there["outlet"] == outlet):
            error_str = (f"{self.name}: outlet {outlet} holds the filament of inlet "
                         f"{other(inlet)}, so inlet {inlet} cannot be moved to it")
        if error_str:
            raise SwitchError(error_str)
        if here["target"] == outlet:
            # The switch gave this move up earlier and only acts on a route that changes.
            route = route_with(route, inlet, other(outlet))
            self._send(route, inlet)
            self.reactor.pause(self.reactor.monotonic() + _ROUTE_HOLD)
        if not self._send(route_with(route, inlet, outlet), inlet):
            error_str = (f"{self.name}: inlet {inlet} did not reach outlet {outlet}. Check "
                         "that nothing is left in the switch.")
            raise SwitchError(error_str)
        return True

    def _send(self, route: int, inlet: str) -> bool:
        """
        Send a route and wait until the switch reports an inlet on it.

        :param route: the route, 0 to 3
        :param inlet: the inlet the route is sent for, `A` or `B`
        :return bool: False when the switch did not report the inlet there in time
        """
        if not self.chain.switch_set_route(route):
            error_str = f"{self.name}: no link to the BridgeBox of {self.chain_name}"
            raise SwitchError(error_str)
        outlet = outlet_at(route, inlet)
        for _poll in range(_ROUTE_POLLS):
            self.reactor.pause(self.reactor.monotonic() + 0.1)
            now = self.chain.switch_state()
            if (now is not None
                and now["on"]
                and outlet_at(now["route"], inlet) == outlet
                and outlet_at(now["position"], inlet) == outlet):
                return True
        return False

    @contextmanager
    def _own(self) -> Iterator[None]:
        """
        Hold the switch for one job, waiting out a job that is already moving it.
        """
        while self._moving:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        self._moving = True
        try:
            yield
        finally:
            self._moving = False

    def outlet_for(self, lane: AFCLane) -> str:
        """
        The outlet that feeds a lane's extruder.

        :param lane: a lane of this switch
        :return str: `A` or `B`
        """
        outlet = self.outlet_of(lane.extruder_obj.name)
        if outlet is None:
            error_str = (f"{self.name}: {lane.name} is on {lane.extruder_obj.name}, which this "
                         "switch does not feed")
            raise SwitchError(error_str)
        return outlet

    def _loaded(self, inlet: str) -> Optional[AFCLane]:
        """
        The lane of an inlet that is loaded in a toolhead.

        :param inlet: `A` or `B`
        :return AFCLane: the lane, None when the inlet has none loaded
        """
        return next((lane for lane in self.lanes
                     if (lane.tool_loaded
                         and self.inlet_of(lane.unit) == inlet)), None)

    def _watch(self, eventtime: float) -> float:
        """
        Reactor timer: bring the switch in line with the lanes while AFC is idle.

        :param eventtime: reactor time of this call
        :return float: reactor time of the next call
        """
        self._look()
        if (self.afc.prep_done
            and self.afc.current_state == State.IDLE
            and not self._moving):
            self._note_arrivals()
            with self._own():
                self._say(self._settle())
        return self.reactor.monotonic() + _WATCH_PERIOD

    def _look(self) -> None:
        """
        Note the inlets that read filament now, for a check that asks later whether one did.
        """
        try:
            inlets = self.state()["inlets"]
        except SwitchError:
            return
        for letter in LETTERS:
            if inlets[letter]["filament"]:
                self._seen[letter] = True

    def _tell(self, problems: List[str]) -> None:
        """
        Warn about what keeps the switch from settling, once for as long as it stays the same.

        :param problems: what `_settle` returned
        """
        told = "\n".join(problems)
        if (told
            and told != self._told):
            self.logger.warning(told)
        self._told = told

    def _settle(self) -> List[str]:
        """
        Point each inlet where it belongs and give each tube its length.

        :return list: what keeps an inlet from getting there, one message each
        """
        try:
            inlets = self.state()["inlets"]
        except SwitchError as ex:
            return [str(ex)]
        problems = []
        for inlet in LETTERS:
            try:
                self._settle_inlet(inlet, inlets)
            except SwitchError as ex:
                problems.append(str(ex))
        self._settle_lengths()
        return problems

    def _settle_lengths(self) -> None:
        """
        Give each tube the length for the lane that is loaded or on its way through it, and its
        own length back once there is none.
        """
        active = [lane for lane in self.lanes
                  if (lane.tool_loaded
                      or lane in self._routed.values())]
        for lane in active:
            self._set_length(lane)
        tubes = [self._tube(lane) for lane in active]
        for tube in [tube for tube in self._offsets if tube not in tubes]:
            self._offset(tube, 0.0)

    def calibrated(self) -> bool:
        """
        Whether a lane can be given the right tube length through either outlet.

        :return bool: True when both outlets have a length, or only one outlet is in use
        """
        return (len(self.extruders()) < 2
                or None not in self.lengths.values())

    def _tube(self, lane: AFCLane) -> Optional[Any]:
        """
        The hub that holds the length of a lane's tube to the toolhead.

        :param lane: a lane of this switch
        :return AFCHub: the hub, None for a lane that has none
        """
        hub = lane.hub_obj
        return hub if hasattr(hub, "afc_bowden_length") else None

    def _set_length(self, lane: AFCLane) -> None:
        """
        Give a lane's tube the length for the outlet the lane goes through: its own, plus the
        difference between the two outlets' tubes when that is not the outlet its unit starts
        on.

        :param lane: a lane that is loaded or about to be
        """
        tube = self._tube(lane)
        outlet = self.outlet_of(lane.extruder_obj.name)
        if (tube is None
            or outlet is None):
            return
        start = self.start[self.inlet_of(lane.unit)]
        offset = 0.0
        if (outlet != start
            and self.calibrated()):
            offset = self.lengths[outlet] - self.lengths[start]
        self._offset(tube, offset)

    def _offset(self, tube: Any, offset: float) -> None:
        """
        Change what is added to a tube's own length, for loading and for unloading.

        :param tube: a hub `_tube` returned
        :param offset: mm to have added from now on
        """
        change = offset - self._offsets.get(tube, 0.0)
        tube.afc_bowden_length += change
        tube.afc_unload_bowden_length += change
        self._offsets[tube] = offset

    def handle_prep(self, gcmd: AFCGcmd) -> None:
        """
        PREP's section for the switch: bring it in line with the lanes that are loaded, then
        list it. A unit that only comes into AFC once PREP has run is waited for first.

        :param gcmd: the PREP command, unused
        """
        self._report_by = self.reactor.monotonic() + _REPORT_WAIT
        with self._own():
            self._say(self._settle())

    def _arriving(self) -> bool:
        """
        Whether a unit is on its way into AFC: one that answers on the chain and has no bay
        yet, or a unit of this switch that is being brought in.

        :return bool: True while the switch's section should wait
        """
        units = [self.afc.units.get(unit) for unit in self.units()]
        return (self.chain.units_arriving()
                or any(getattr(unit, "arriving", False) for unit in units))

    def _note_arrivals(self) -> None:
        """
        Have the switch's section printed again when a unit came into AFC since it last was.
        """
        live = self.live_units()
        new = [unit for unit in live if unit not in self._reported]
        if (new
            and self._report_by is None):
            self._report_by = self.reactor.monotonic() + _REPORT_WAIT
        # A unit that left counts as new when it is back.
        self._reported = [unit for unit in self._reported if unit in live]

    def _say(self, problems: List[str]) -> None:
        """
        Say what keeps the switch from settling: in its section when one is due, else as a
        warning. A section that is due waits for the units on their way into AFC.

        :param problems: what `_settle` returned
        """
        if self._report_by is None:
            self._tell(problems)
        elif (not self._arriving()
              or self.reactor.monotonic() >= self._report_by):
            self._section(problems)

    def _section(self, problems: List[str]) -> None:
        """
        Print the switch's section: its inlets and outlets, then what is wrong with them.

        :param problems: what `_settle` returned
        """
        self._report_by = None
        self._reported = self.live_units()
        self.logger.info(f"Filament switch {self.name} Prepping inlets")
        try:
            self.logger.raw(self._report())
        except SwitchError:
            pass
        for problem in problems:
            self.logger.raw(f"<span class=error--text>{problem}</span>")
        # Said here, so the watch does not say it again.
        self._told = "\n".join(problems)

    def _settle_inlet(self, inlet: str, inlets: Dict[str, Dict[str, Any]]) -> None:
        """
        Point one inlet at the outlet of its loaded lane, else of the lane it was last pointed
        for, else at the outlet it rests on. An empty inlet stays where it is during a print
        and while the other inlet's filament is in that outlet.

        :param inlet: `A` or `B`
        :param inlets: both inlets as `state` reports them
        """
        there = inlets[other(inlet)]
        lane = self._loaded(inlet) or self._routed[inlet]
        if lane is not None:
            outlet = self.outlet_for(lane)
        elif inlets[inlet]["filament"]:
            error_str = (f"{self.name}: inlet {inlet} holds filament and none of its lanes is "
                         "loaded, so it is left where it is. Unload it or pull it out.")
            raise SwitchError(error_str)
        elif (self.afc.function.in_print()
              or (there["filament"]
                  and there["outlet"] == self.rest[inlet])):
            return
        else:
            outlet = self.rest[inlet]
        self.set_inlet(inlet, outlet)

    def handle_tool_load_prepare(self, lane: AFCLane, gcmd: AFCGcmd) -> bool:
        """
        Point a lane's inlet at its extruder's outlet before AFC loads the lane.

        :param lane: the lane about to be loaded
        :param gcmd: command context the load runs through
        :return bool: False stops the load
        """
        inlet = self.inlet_of(lane.unit)
        if (inlet is None
            or lane.tool_loaded):
            return True
        # Set before anything pauses, so the watch does not send this inlet back to rest.
        self._routed[inlet] = lane
        try:
            with self._own():
                if self._route(lane, inlet, gcmd):
                    self._set_length(lane)
                    return True
        except SwitchError as ex:
            self.afc.error.AFC_error(str(ex), pause=self.afc.function.in_print(), gcmd=gcmd)
        self._routed[inlet] = None
        return False

    def _route(self, lane: AFCLane, inlet: str, gcmd: AFCGcmd) -> bool:
        """
        Unload what is in the way of a lane, then point the lane's inlet at its outlet.

        :param lane: the lane about to be loaded
        :param inlet: the lane's inlet
        :param gcmd: command context the unloads run through
        :return bool: False when a lane in the way could not be unloaded
        """
        outlet = self.outlet_for(lane)
        for held in self._in_the_way(lane, inlet, outlet):
            self.logger.info(f"{self.name}: unloading {held.name} from "
                             f"{held.extruder_obj.name}, it is in the way of {lane.name} to "
                             f"{lane.extruder_obj.name}")
            if not self._unload(held, gcmd):
                return False
            self._wait_clear(self.inlet_of(held.unit))
        self.set_inlet(inlet, outlet)
        return True

    def _in_the_way(self, lane: AFCLane, inlet: str, outlet: str) -> List[AFCLane]:
        """
        The loaded lanes whose filament keeps a lane's path from being set: one of the same inlet
        in the other outlet, and one of the other inlet in the same outlet.

        :param lane: the lane about to be loaded
        :param inlet: the lane's inlet
        :param outlet: the outlet the lane needs
        :return list: lanes to unload first
        """
        return [held for held in self.lanes
                if (held.tool_loaded
                    and held is not lane
                    and ((self.inlet_of(held.unit) == inlet)
                         != (self.outlet_of(held.extruder_obj.name) == outlet)))]

    def _unload(self, lane: AFCLane, gcmd: AFCGcmd) -> bool:
        """
        Unload a lane from its toolhead, whichever toolhead is active.

        :param lane: the lane to unload
        :param gcmd: command context the unload runs through
        :return bool: True when the lane was unloaded
        """
        if self.afc.function.get_current_extruder() != lane.extruder_obj.th_extruder_name:
            lane.tool_swap(gcmd=gcmd)
        return self.afc.TOOL_UNLOAD(lane, set_start_time=False, force_unload=True, gcmd=gcmd)

    def _wait_clear(self, inlet: str) -> None:
        """
        Give an inlet's sensor time to read empty after its lane was unloaded.

        :param inlet: `A` or `B`
        """
        for _poll in range(_CLEAR_POLLS):
            if not self.state()["inlets"][inlet]["filament"]:
                return
            self.reactor.pause(self.reactor.monotonic() + 0.1)

    def handle_tool_loaded(self, lane: AFCLane) -> None:
        """
        Check a lane that reached its toolhead also shows at its inlet.

        :param lane: the lane AFC just marked as loaded
        """
        inlet = self.inlet_of(lane.unit)
        if inlet is None:
            return
        if self._routed[inlet] is lane:
            self._routed[inlet] = None
        try:
            inlets = self.state()["inlets"]
        except SwitchError:
            return
        if inlets[inlet]["filament"]:
            self._confirm(lane.unit)
            return
        there = other(inlet)
        hint = f"Check that {lane.unit} is plumbed into inlet {inlet}."
        if (inlets[there]["filament"]
            and self._loaded(there) is None):
            hint = f"Inlet {there} reads filament, so {lane.unit} looks plumbed into it."
        self.logger.warning(f"{self.name}: {lane.name} is loaded and inlet {inlet} reads "
                            f"empty. {hint}")

    def _confirm(self, unit: str) -> None:
        """
        Note that a unit's filament showed at its inlet, and save that the first time.

        :param unit: a unit of this switch
        """
        if unit not in self.confirmed:
            self.confirmed.append(unit)
            self.afc.save_vars()

    def _report(self) -> str:
        """
        One line per inlet: its units, the outlet it is at and what it holds. One per outlet:
        its extruder and the lanes that feed it. Then what is still to be set up.

        :return str: the report
        """
        st = self.state()
        live = self.live_units()
        lines = []
        for letter in LETTERS:
            inlet = st["inlets"][letter]
            loaded = self._loaded(letter)
            units = ", ".join(unit for unit in self.inlets[letter] if unit in live) or "no unit"
            holds = "holds filament" if inlet["filament"] else "empty"
            if loaded is not None:
                holds = f"{loaded.name} loaded"
            lines.append(f"Inlet {letter}: {units}, at outlet {inlet['outlet']}, {holds}")
        for letter in LETTERS:
            name = self.outlets[letter]
            lanes = ", ".join(sorted((lane.name for lane in self.lanes
                                      if (lane.unit in live
                                          and lane.extruder_obj.name == name)), key=in_order))
            lines.append(f"Outlet {letter}: {name or 'no extruder'} ({lanes or 'no lane'})")
        if not self.calibrated():
            lines.append("Tube lengths not set: run AFC_CALIBRATION, or set outlet_a_length and "
                         "outlet_b_length")
        waiting = [unit for unit in live if unit not in self.confirmed]
        if waiting:
            lines.append(f"Not seen at their inlet yet: {', '.join(waiting)}")
        return "\n".join(lines)

    cmd_AFC_BRIDGEBOX_SWITCH_ROUTE_help = ("AFC_BRIDGEBOX_SWITCH_ROUTE [SWITCH=<switch>] "
                                           "[INLET=<A|B> OUTLET=<A|B>]. Points an inlet of a "
                                           "filament switch at an outlet, or shows where each "
                                           "inlet exits")

    def cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(self, gcmd: GCodeCommand) -> None:
        """
        Point an inlet of the filament switch at an outlet, by the letters printed on it.

        An inlet that holds filament is not moved, and neither is one whose new outlet holds the
        other inlet's filament. The inlet then rests on that outlet while it is empty, until
        Klipper restarts. Without INLET and OUTLET it reports where each inlet exits and whether
        it holds filament.

        Usage
        -------
        `AFC_BRIDGEBOX_SWITCH_ROUTE [SWITCH=<switch>] [INLET=<A|B> OUTLET=<A|B>]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_SWITCH_ROUTE SWITCH=Switch_1 INLET=A OUTLET=B
        ```
        """
        inlet = gcmd.get("INLET", "").upper()
        outlet = gcmd.get("OUTLET", "").upper()
        try:
            if (not inlet
                and not outlet):
                gcmd.respond_info(f"{self.name} on {self.chain_name}\n{self._report()}")
                return
            if (inlet not in LETTERS
                or outlet not in LETTERS):
                error_str = f"{self.name}: INLET and OUTLET each take A or B"
                raise SwitchError(error_str)
            with self._own():
                moved = self.set_inlet(inlet, outlet)
        except SwitchError as ex:
            raise gcmd.error(str(ex))
        self.rest[inlet] = outlet
        self._routed[inlet] = None
        feeds = self.outlets[outlet] or "no extruder"
        gcmd.respond_info(f"{self.name}: inlet {inlet} {'now exits' if moved else 'already exits'} "
                          f"outlet {outlet} ({feeds})")

    cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN_help = ("AFC_BRIDGEBOX_SWITCH_ASSIGN LANE=<lane> | "
                                            "TOOL=<T number> [EXTRUDER=<extruder>] "
                                            "[UNLOAD=<0|1>]. Puts a lane on one of the "
                                            "extruders its filament switch feeds")

    def cmd_AFC_BRIDGEBOX_SWITCH_ASSIGN(self, gcmd: GCodeCommand) -> None:
        """
        Put a lane on one of the extruders its filament switch feeds. The lane is loaded into
        that extruder from then on, and the choice is kept over restarts.

        Without EXTRUDER the lane goes back to the extruder its unit starts on. A lane that is
        loaded in a toolhead is only moved with UNLOAD=1, which unloads it first.

        TOOL names the lane by the T number mapped to it now, for a print file that says which
        extruder each of its T numbers was sliced for. A T number whose lane cannot reach that
        extruder is warned about and the print goes on from the lane's own extruder.

        Usage
        -------
        `AFC_BRIDGEBOX_SWITCH_ASSIGN LANE=<lane> [EXTRUDER=<extruder>] [UNLOAD=<0|1>]`
        `AFC_BRIDGEBOX_SWITCH_ASSIGN TOOL=<T number> EXTRUDER=<extruder> [UNLOAD=<0|1>]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_SWITCH_ASSIGN LANE=lane12 EXTRUDER=extruder4
        ```
        """
        tool = gcmd.get("TOOL", None)
        unload = bool(gcmd.get_int("UNLOAD", 0))
        if tool is not None:
            self._assign_tool(tool, gcmd.get("EXTRUDER"), unload, gcmd)
            return
        name = gcmd.get("LANE")
        switch = self._switch_of(name)
        if switch is None:
            error_str = f"AFC_BRIDGEBOX_SWITCH_ASSIGN: {name} is not a lane of a filament switch"
            raise gcmd.error(error_str)
        switch.assign(switch.lane(name), gcmd.get("EXTRUDER", None), unload, gcmd)

    def lane(self, name: str) -> Optional[AFCLane]:
        """
        One of this switch's lanes.

        :param name: lane name
        :return AFCLane: the lane, None when the switch has no lane of that name
        """
        return next((lane for lane in self.lanes if lane.name == name), None)

    def _switch_of(self, name: str) -> Optional[afcBridgeBoxSwitch]:
        """
        The filament switch a lane goes through.

        :param name: lane name
        :return afcBridgeBoxSwitch: the switch, None for a lane that is on none
        """
        return next((switch
                     for _name, switch in self.printer.lookup_objects("AFC_BridgeBox_switch")
                     if switch.lane(name) is not None), None)

    def _assign_tool(self, tool: str, extruder: str, unload: bool, gcmd: GCodeCommand) -> None:
        """
        Put the lane a T number is mapped to on the extruder a print file names for it. What
        cannot be done is warned about, so the print goes on from the lane's own extruder.

        :param tool: the T number, as `T3` or `3`
        :param extruder: AFC extruder name the T number was sliced for
        :param unload: True to unload the lane first when it is loaded in a toolhead
        :param gcmd: the command, for its answer and the unload
        """
        tool = tool.upper() if tool.upper().startswith("T") else f"T{tool}"
        lane = self.afc.function.get_lane_by_map(tool)
        if lane is None:
            self.logger.warning(f"AFC_BRIDGEBOX_SWITCH_ASSIGN: no lane is mapped to {tool}")
            return
        switch = self._switch_of(lane.name)
        if (switch is not None
            and extruder in switch.extruders()):
            switch.assign(lane, extruder, unload, gcmd)
        elif lane.extruder_obj.name != extruder:
            self.logger.warning(f"AFC_BRIDGEBOX_SWITCH_ASSIGN: {tool} is {lane.name}, which "
                                f"cannot reach {extruder}. It prints from "
                                f"{lane.extruder_obj.name}.")

    def assign(self, lane: AFCLane, extruder: Optional[str], unload: bool,
               gcmd: GCodeCommand) -> None:
        """
        Put one of this switch's lanes on one of its extruders and save the choice.

        :param lane: the lane
        :param extruder: AFC extruder name, None for the one the lane's unit starts on
        :param unload: True to unload the lane first when it is loaded in a toolhead
        :param gcmd: the command, for its answer and the unload
        """
        extruder = extruder or self.start_extruder(lane.unit)
        was = lane.extruder_obj.name
        if extruder not in self.extruders():
            error_str = f"{self.name}: EXTRUDER takes {' or '.join(self.extruders())}"
            raise gcmd.error(error_str)
        if extruder == was:
            gcmd.respond_info(f"{self.name}: {lane.name} already feeds {extruder}")
            return
        if (lane.tool_loaded
            and not unload):
            error_str = (f"{self.name}: {lane.name} is loaded in {was}. Unload it first, or "
                         "add UNLOAD=1.")
            raise gcmd.error(error_str)
        if lane.tool_loaded:
            self._unload(lane, gcmd)
        if not lane.set_extruder(self._extruder(extruder)):
            error_str = f"{self.name}: {lane.name} could not be unloaded from {was}"
            raise gcmd.error(error_str)
        self.afc.save_vars()
        gcmd.respond_info(f"{self.name}: {lane.name} now feeds {extruder}")

    def handle_calibration_buttons(self, buttons: List[Tuple[str, str, str]]) -> None:
        """
        Add this switch to AFC's calibration menu.

        :param buttons: the menu's buttons so far, each a label, a command and a style
        """
        buttons.append((f"{self.name} filament switch",
                        f"AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH={self.name}", "primary"))

    cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION_help = ("AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                                                 "[SWITCH=<switch>] [LANE=<lane>]. Measures "
                                                 "the tube from each outlet of a filament "
                                                 "switch to its toolhead")

    def cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(self, gcmd: GCodeCommand) -> None:
        """
        Measure the tube from each outlet of the filament switch to its toolhead with one lane,
        and save both lengths.

        The lane's unit runs its own tube length calibration through each outlet, so the lane
        has to be loaded in its unit and the toolheads empty. Each run also checks the lane
        shows at the inlet its unit is configured on. Without LANE a prompt offers the lanes
        that can measure.

        Usage
        -------
        `AFC_BRIDGEBOX_SWITCH_CALIBRATION [SWITCH=<switch>] [LANE=<lane>]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH=Switch_1 LANE=lane8
        ```
        """
        name = gcmd.get("LANE", None)
        if name is None:
            self._calibration_menu(gcmd)
            return
        lane = self.lane(name)
        try:
            if lane is None:
                error_str = f"{self.name}: {name} is not a lane of this switch"
                raise SwitchError(error_str)
            lengths = self._measure(lane, gcmd)
        except SwitchError as ex:
            raise gcmd.error(str(ex))
        for letter in LETTERS:
            self.lengths[letter] = round(lengths[letter], 1)
            self.afc.function.ConfigRewrite(self.fullname, f"outlet_{letter.lower()}_length",
                                            self.lengths[letter])
        gcmd.respond_info(f"{self.name}: with {lane.name} the tube through outlet A is "
                          f"{self.lengths['A']} mm and through outlet B {self.lengths['B']} mm")

    def _calibration_menu(self, gcmd: GCodeCommand) -> None:
        """
        Open the prompt that offers one lane per unit to measure the outlets with.

        :param gcmd: the command the prompt answers
        """
        buttons = [(f"{lane.unit} {lane.name}",
                    f"AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH={self.name} LANE={lane.name}",
                    "primary") for lane in self._measuring_lanes()]
        text = ("Measures the tube from each outlet to its toolhead with one lane, so a lane "
                "loads the right distance through either outlet. Unload the toolheads, then "
                "pick the lane to measure with.")
        if not buttons:
            text = ("No lane that can measure is loaded in its unit. Load one, or set "
                    "outlet_a_length and outlet_b_length in the switch's section by hand.")
        back = [("Back to unit selection", "AFC_CALIBRATION", "info")]
        AFCprompt(gcmd, self.logger).create_custom_p(f"{self.name} Calibration", text, buttons,
                                                     True, None, back)

    def _measuring_lanes(self) -> List[AFCLane]:
        """
        The lanes the outlets can be measured with: the first lane of each unit that is loaded
        in the unit and has a hub.

        :return list: one lane per unit
        """
        lanes: Dict[str, AFCLane] = {}
        for lane in self.lanes:
            if (lane.load_state
                and self._tube(lane) is not None):
                lanes.setdefault(lane.unit, lane)
        return list(lanes.values())

    def _measure(self, lane: AFCLane, gcmd: GCodeCommand) -> Dict[str, float]:
        """
        Measure a lane's tube through each outlet with its unit's own calibration, leaving the
        lane, its unit's length and the switch as they were.

        :param lane: the lane to measure with
        :param gcmd: the command the calibrations run through
        :return dict: outlet letter -> mm
        """
        inlet = self.inlet_of(lane.unit)
        tube = self._tube(lane)
        loaded = [held.name for held in self.lanes if held.tool_loaded]
        if self.afc.current is not None:
            loaded.append(self.afc.current)
        error_str = ""
        if len(self.extruders()) < 2:
            error_str = f"{self.name}: one outlet is in use, so there is nothing to measure"
        elif tube is None:
            error_str = (f"{self.name}: {lane.name} has no hub that holds a tube length. Use a "
                         "lane of another unit, or set outlet_a_length and outlet_b_length by "
                         "hand.")
        elif loaded:
            error_str = (f"{self.name}: unload {', '.join(dict.fromkeys(loaded))} first. The "
                         "toolheads have to be empty to measure.")
        if error_str:
            raise SwitchError(error_str)
        home = lane.extruder_obj
        start = self.start[inlet]
        self._offset(tube, 0.0)
        own = (tube.afc_bowden_length, tube.afc_unload_bowden_length)
        lengths: Dict[str, float] = {}
        try:
            # The unit's own outlet goes first: a unit that measures no length stops there,
            # before anything of its own was calibrated through the other outlet.
            lengths[start] = self._measure_outlet(lane, inlet, start, gcmd)
            own = (tube.afc_bowden_length, tube.afc_unload_bowden_length)
            lengths[other(start)] = self._measure_outlet(lane, inlet, other(start), gcmd)
        finally:
            # The other outlet's run saved its length as the unit's own.
            self._restore_tube(tube, own)
            # The watch may have added the outlets' difference during that run.
            self._offsets[tube] = 0.0
            self._routed[inlet] = None
            lane.set_extruder(home)
        self._confirm(lane.unit)
        return lengths

    def _measure_outlet(self, lane: AFCLane, inlet: str, outlet: str,
                        gcmd: GCodeCommand) -> float:
        """
        Run the unit's own tube length calibration with a lane through one outlet, and check
        the lane showed at its inlet on the way.

        :param lane: the lane to measure with
        :param inlet: the lane's inlet
        :param outlet: the outlet to measure through
        :param gcmd: the command the calibration runs through
        :return float: mm from the lane's hub to the outlet's toolhead
        """
        extruder = self.outlets[outlet]
        lane.set_extruder(self._extruder(extruder))
        # Keeps the watch from sending the inlet back to rest.
        self._routed[inlet] = lane
        with self._own():
            self.set_inlet(inlet, outlet)
        self._seen = {letter: False for letter in LETTERS}
        done, why, length = lane.unit_obj.calibrate_bowden(lane, _MEASURE_STEP,
                                                           _MEASURE_TOLERANCE, gcmd=gcmd)
        self._look()
        error_str = ""
        if not done:
            reason = f" {why.rstrip('.')}." if why else ""
            error_str = (f"{self.name}: {lane.name} did not measure the tube to {extruder} "
                         f"through outlet {outlet}.{reason} Reset {lane.name} before the next "
                         "try.")
        elif length <= 0:
            error_str = (f"{self.name}: the unit of {lane.name} measures no tube length. Use a "
                         "lane of another unit, or set outlet_a_length and outlet_b_length by "
                         "hand.")
        elif not self._seen[inlet]:
            saw = (f"Inlet {other(inlet)} did" if self._seen[other(inlet)]
                   else "Neither inlet did")
            error_str = (f"{self.name}: {lane.name} reached {extruder} and inlet {inlet} never "
                         f"read filament. {saw}. Check where {lane.unit} is plumbed.")
        if error_str:
            raise SwitchError(error_str)
        return float(length)

    def _restore_tube(self, tube: Any, lengths: Tuple[float, float]) -> None:
        """
        Put a tube's own lengths back, in memory and in its config, when a measurement through
        the other outlet left that outlet's length in them.

        :param tube: the hub
        :param lengths: its own load and unload lengths
        """
        if (tube.afc_bowden_length, tube.afc_unload_bowden_length) == lengths:
            return
        tube.afc_bowden_length, tube.afc_unload_bowden_length = lengths
        self.afc.function.ConfigRewrite(tube.fullname, "afc_bowden_length", lengths[0])
        self.afc.function.ConfigRewrite(tube.fullname, "afc_unload_bowden_length", lengths[1])

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        The switch for the status API.

        :param eventtime: reactor time of the query, unused
        :return dict: chain, whether the switch answers and is calibrated, each inlet's units
        in use, state and loaded lane, each outlet's extruder and tube length, each lane's
        extruder and the units not seen at their inlet yet
        """
        try:
            inlets = self.state()["inlets"]
        except SwitchError:
            inlets = {letter: {"outlet": None, "filament": None} for letter in LETTERS}
        loaded = {letter: self._loaded(letter) for letter in LETTERS}
        live = self.live_units()
        return {"chain": self.chain_name,
                "answering": inlets["A"]["outlet"] is not None,
                "calibrated": self.calibrated(),
                "inlets": {letter: dict(inlets[letter],
                                        units=[unit for unit in self.inlets[letter]
                                               if unit in live],
                                        lane=loaded[letter].name if loaded[letter] else None)
                           for letter in LETTERS},
                "outlets": dict(self.outlets),
                "lengths": dict(self.lengths),
                "lanes": {lane.name: lane.extruder_obj.name for lane in self.lanes
                          if lane.unit in live},
                "unconfirmed": [unit for unit in live if unit not in self.confirmed]}


def load_config_prefix(config: ConfigWrapper) -> afcBridgeBoxSwitch:
    """
    Klipper config entry point for `[AFC_BridgeBox_switch <name>]` sections.

    :param config: the section
    :return afcBridgeBoxSwitch: the switch
    """
    return afcBridgeBoxSwitch(config)
