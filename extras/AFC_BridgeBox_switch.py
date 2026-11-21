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

The letters are the ones printed on the switch. `chain` may be left out when there is one chain.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List, Optional

from configfile import error

from extras.AFC_BridgeBox import afcBridgeBox
from extras.AFC_unit import afcUnit

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand

#: The letters printed on the switch, for its inlets and for its outlets.
LETTERS = ("A", "B")

#: Route bit that sends each inlet to outlet A. With its bit clear an inlet exits outlet B.
_TO_OUTLET_A = {"A": 2, "B": 1}

#: Where each inlet's filament sensor sits in the chain's `inlets` list.
_SENSOR = {"A": 1, "B": 0}

#: Status polls, 0.1 s apart, a route change may take before the switch reports it.
_ROUTE_POLLS = 50


class SwitchError(Exception):
    """A route the switch cannot take. The message says why."""


def other(letter: str) -> str:
    """
    The other inlet, or the other outlet.

    :param letter: `A` or `B`
    :return str: `B` for `A`, `A` for `B`
    """
    return "B" if letter == "A" else "A"


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
        for letter in LETTERS:
            units = config.getlist(f"inlet_{letter.lower()}", [])
            self.inlets[letter] = [unit.strip() for unit in units if unit.strip()]
            self.outlets[letter] = config.get(f"outlet_{letter.lower()}", None)
        self._check_section()

        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        gcode = self.printer.lookup_object("gcode")
        for value in (self.name, None):
            gcode.register_mux_command("AFC_BRIDGEBOX_SWITCH_ROUTE", "SWITCH", value,
                                       self.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE,
                                       desc=self.cmd_AFC_BRIDGEBOX_SWITCH_ROUTE_help)

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
        Find the chain, and check the units and extruders exist and no other switch uses them.
        """
        self.chain = self._find_chain()
        self.chain_name = self.chain.name
        problems = self._missing() + self._clashes()
        if problems:
            raise error(problems[0])

    def _find_chain(self) -> afcBridgeBox:
        """
        The chain whose bus the switch is on: the one `chain` names, else the only one.

        :return afcBridgeBox: the chain
        """
        chains = {obj.name: obj for _name, obj in self.printer.lookup_objects("AFC_BridgeBox")
                  if isinstance(obj, afcBridgeBox)}
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

        Refused while the inlet holds filament, since its path cannot move then, and while the
        outlet holds the other inlet's filament.

        :param inlet: inlet to move, `A` or `B`
        :param outlet: outlet it should exit, `A` or `B`
        :return bool: True when the switch moved, False when it was there already
        """
        st = self.state()
        here = st["inlets"][inlet]
        there = st["inlets"][other(inlet)]
        if (here["outlet"] == outlet
            and here["target"] == outlet):
            return False
        error_str = ""
        if here["filament"]:
            error_str = (f"{self.name}: inlet {inlet} holds filament, so it cannot be moved "
                         f"to outlet {outlet}")
        elif (there["filament"]
              and outlet in (there["outlet"], there["target"])):
            error_str = (f"{self.name}: outlet {outlet} holds the filament of inlet "
                         f"{other(inlet)}, so inlet {inlet} cannot be moved to it")
        if error_str:
            raise SwitchError(error_str)
        route = route_with(st["route"], inlet, outlet)
        if not self.chain.switch_set_route(route):
            error_str = f"{self.name}: no link to the BridgeBox of {self.chain_name}"
            raise SwitchError(error_str)
        for _poll in range(_ROUTE_POLLS):
            self.reactor.pause(self.reactor.monotonic() + 0.1)
            now = self.chain.switch_state()
            if (now is not None
                and now["on"]
                and now["route"] == route
                and now["position"] == route):
                return True
        error_str = f"{self.name}: inlet {inlet} did not reach outlet {outlet}"
        raise SwitchError(error_str)

    def _describe(self) -> str:
        """
        One line per inlet: its units, where it exits and whether it holds filament.

        :return str: the report
        """
        st = self.state()
        lines = []
        for letter in LETTERS:
            inlet = st["inlets"][letter]
            units = ", ".join(self.inlets[letter]) or "no unit"
            feeds = self.outlets[inlet["outlet"]] or "no extruder"
            holds = "holds filament" if inlet["filament"] else "empty"
            lines.append(f"{self.name}: inlet {letter} ({units}) exits outlet "
                         f"{inlet['outlet']} ({feeds}), {holds}")
        return "\n".join(lines)

    cmd_AFC_BRIDGEBOX_SWITCH_ROUTE_help = ("AFC_BRIDGEBOX_SWITCH_ROUTE [SWITCH=<switch>] "
                                           "[INLET=<A|B> OUTLET=<A|B>]. Points an inlet of a "
                                           "filament switch at an outlet, or shows where each "
                                           "inlet exits")

    def cmd_AFC_BRIDGEBOX_SWITCH_ROUTE(self, gcmd: GCodeCommand) -> None:
        """
        Point an inlet of the filament switch at an outlet, by the letters printed on it.

        An inlet that holds filament is not moved, and neither is one whose new outlet holds the
        other inlet's filament. Without INLET and OUTLET it reports where each inlet exits and
        whether it holds filament.

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
                gcmd.respond_info(self._describe())
                return
            if (inlet not in LETTERS
                or outlet not in LETTERS):
                error_str = f"{self.name}: INLET and OUTLET each take A or B"
                raise SwitchError(error_str)
            moved = self.set_inlet(inlet, outlet)
        except SwitchError as ex:
            raise gcmd.error(str(ex))
        feeds = self.outlets[outlet] or "no extruder"
        gcmd.respond_info(f"{self.name}: inlet {inlet} {'now exits' if moved else 'already exits'} "
                          f"outlet {outlet} ({feeds})")

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        The switch for the status API.

        :param eventtime: reactor time of the query, unused
        :return dict: chain, the units and state of each inlet, and each outlet's extruder
        """
        try:
            inlets = self.state()["inlets"]
        except SwitchError:
            inlets = {letter: {"outlet": None, "filament": None} for letter in LETTERS}
        return {"chain": self.chain_name,
                "answering": inlets["A"]["outlet"] is not None,
                "inlets": {letter: dict(inlets[letter], units=list(self.inlets[letter]))
                           for letter in LETTERS},
                "outlets": dict(self.outlets)}


def load_config_prefix(config: ConfigWrapper) -> afcBridgeBoxSwitch:
    """
    Klipper config entry point for `[AFC_BridgeBox_switch <name>]` sections.

    :param config: the section
    :return afcBridgeBoxSwitch: the switch
    """
    return afcBridgeBoxSwitch(config)
