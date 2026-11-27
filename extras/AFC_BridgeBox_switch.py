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
    inlet_a: Chain_1, Turtle_1
    inlet_b: Ace2_1
    outlet_a: extruder
    outlet_b: extruder1
    outlet_a_length: 850
    outlet_b_length: 1010

The letters are the ones printed on the switch. `chain` may be left out when there is one chain.
A buffer feeds one inlet, and a BridgeBox chain's units share its buffer. The chain's name on an
inlet, or the name of any one of its units, puts every unit that chain makes for its bays on that
inlet, so a unit that is plugged into the chain later enters it too.

A chain can feed both inlets with a buffer on each. The units behind the second buffer are named
on the other inlet, and the chain makes them a buffer of their own, named for that inlet:

    inlet_a: Chain_1
    inlet_b: Bambu_AMS_HT_1

A chain that is on both inlets has each of its units found by loading, so no unit is named. A
unit of another kind that is on both inlets is found by loading too:

    inlet_a: Chain_1, Turtle_1
    inlet_b: Chain_1, Turtle_1

`AFC_BRIDGEBOX_SWITCH_IDENTIFY` feeds one lane of a unit to the switch and reels it back. The
inlet whose sensor read it is the unit's inlet from then on, and a chain's unit goes behind that
inlet's buffer. An AMS that is new on such a chain does not load until its inlet is found, and
what was found is kept for as long as the AMS keeps a bay of the chain.

A unit that has a feed of its own for this is fed with it. A lane AFC homes with a stepper is fed
in one move that ends when the sensor reads it, and any other lane AFC moves by distance is fed a
step at a time. Either is pulled back as far as it was fed. A unit that only runs whole loads
cannot be found by loading: it is named on one inlet.

An inlet's units start on the extruder of the outlet with the same letter, and an empty inlet rests
on that outlet. A lane goes to the other extruder with `AFC_BRIDGEBOX_SWITCH_ASSIGN`, and every
load points the lane's inlet at its extruder's outlet before filament moves.

A lane that is assigned while a print runs, as a print file does for the toolhead each of its
filaments was sliced for, is on that extruder for the print: once the print is over and the lane
is out of its toolhead it goes back to the extruder its unit starts on. A lane that is assigned
at any other time stays where it is put.

`outlet_a_length` and `outlet_b_length` are the tube from each outlet to its toolhead in mm. Only
their difference is used: it is added to a lane's tube length while the lane goes through the
outlet its unit does not start on. `AFC_BRIDGEBOX_SWITCH_CALIBRATION` measures them, and they may
be set by hand.

A chain can make its switch itself, and then the switch has no section: `AFC_BRIDGEBOX_SWITCH_ADD`
gives a chain the switch that answers on its bus. That switch has the chain's name and each of
its chain's units is found by loading. `AFC_BRIDGEBOX_SWITCH_SETUP` finds those inlets, and the
toolhead each outlet feeds: a lane is fed through the outlet until a toolhead sensor reads it. Its
lanes do not load until an outlet has its toolhead. What is found is kept with the chain, and the
options above set one by hand in the chain's section, each with `switch_` before its name.

A toolhead with no sensor of its own, as one that loads by buffer ramming has none, cannot be
found. `AFC_BRIDGEBOX_SWITCH_OUTLET` sets the toolhead of an outlet by hand from a prompt, and
that is kept with the chain as one that was found is.
"""

from __future__ import annotations

import json
import re
from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, Callable, Dict, Iterator, List, Optional, Tuple

from configfile import error

from extras.AFC import State
from extras.AFC_BridgeBox import afcBridgeBox, place_chain
from extras.AFC_lane import AFCLane, AFCMoveWarning, AssistActive, SpeedMode
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

#: The mm a lane AFC moves by distance is fed past its own way to a toolhead before a feed that
#: looks for it at a sensor is given up, and the seconds a sensor is given after each step.
_FEED_PAST = 500.0
_FEED_SETTLE = 0.2

#: The endstop key a stepper homes to while its lane is fed until a check sees its filament.
_ENDSTOP = "filament_switch"


class SwitchError(Exception):
    """A route the switch cannot take. The message says why."""


class SwitchResetError(SwitchError):
    """A measurement that stopped and left AFC's prompt to reset its lane open."""


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
        # A switch its chain made has no section: its options are in the chain's.
        self.chain_made: bool = config.getboolean("chain_made", False)
        self.section: str = self.fullname
        if self.chain_made:
            self.section = f"AFC_BridgeBox {self.chain_name}"
        # The options of a switch its chain made that the chain's section sets by hand.
        self.by_hand: List[str] = [key.strip() for key in config.getlist("by_hand", [])
                                   if key.strip()]
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
        # The chains and units on both inlets, which are found by loading (see _place_chains).
        self.finding: List[str] = [name for name in self.inlets["A"] if name in self.inlets["B"]]
        # The chain of a switch it made is on both, unless its section puts it on one.
        if (self.chain_made
            and self.chain_name not in self.inlets["A"]
            and self.chain_name not in self.inlets["B"]):
            self.finding.append(self.chain_name)
        for letter in LETTERS:
            self.inlets[letter] = [name for name in self.inlets[letter]
                                   if name not in self.finding]
        self._check_section()
        # The outlet each inlet's units start on: its own letter, else the only outlet.
        self.start: Dict[str, str] = self._starts()
        # The outlet each inlet rests on while it is empty.
        self.rest: Dict[str, str] = dict(self.start)
        self.lanes: List[AFCLane] = []
        # The lane each inlet was last pointed for, until it reaches its toolhead.
        self._routed: Dict[str, Optional[AFCLane]] = {letter: None for letter in LETTERS}
        self._saved_lanes: Dict[str, str] = {}
        # The lanes a running print put on an extruder their unit does not start on, and those
        # of a print that is over, which go back once they are out of their toolhead.
        self._by_print: List[str] = []
        self._due_back: List[str] = []
        # Whether a print ran at the last look.
        self._printing = False
        # The units a lane of which has shown at its inlet.
        self.confirmed: List[str] = []
        # The units found by loading, each with its chain, None for a unit that is on none.
        self._floating: Dict[str, Optional[afcBridgeBox]] = {}
        # The inlet found for each AMS, by its uid, and for a unit on no chain, by its name.
        self._found: Dict[str, str] = {}
        # What each of those units' inlet was kept for at the last look, by unit (see _id).
        self._held: Dict[str, Optional[str]] = {}
        # The AMS and units a prompt to find their inlet was opened for.
        self._offered: List[str] = []
        # The mm added to each tube for the outlet its lane goes through.
        self._offsets: Dict[Any, float] = {}
        # The inlets that read filament since this was last cleared.
        self._seen: Dict[str, bool] = {letter: False for letter in LETTERS}
        # The extruders whose toolhead sensor is watched while a lane is fed through an outlet,
        # and the ones that read filament since.
        self._watched: Dict[str, AFCExtruder] = {}
        self._read: List[str] = []
        # The check of the feed that runs now with AFC's own moves, and the steppers that were
        # given the endstop that reads it.
        self._check: Optional[Callable[[], bool]] = None
        self._homing: List[Any] = []
        # Whether the prompt that offers to set the switch up was opened.
        self._setup_offered = False
        # The units whose own calibration gave no tube length.
        self._lengthless: List[str] = []
        self._moving = False
        self._told = ""
        # Reactor time the switch's section is printed by at the latest, None when none is due.
        self._report_by: Optional[float] = None
        # The units that were in AFC when the section was last printed.
        self._reported: List[str] = []

        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("afc:prep", self.handle_prep)
        self.printer.register_event_handler("afc:tool_change_unload",
                                            self.handle_tool_change_unload)
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
        self._register("AFC_BRIDGEBOX_SWITCH_IDENTIFY", self.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY,
                       self.cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY_help)
        self._register("AFC_BRIDGEBOX_SWITCH_SETUP", self.cmd_AFC_BRIDGEBOX_SWITCH_SETUP,
                       self.cmd_AFC_BRIDGEBOX_SWITCH_SETUP_help)
        self._register("AFC_BRIDGEBOX_SWITCH_OUTLET", self.cmd_AFC_BRIDGEBOX_SWITCH_OUTLET,
                       self.cmd_AFC_BRIDGEBOX_SWITCH_OUTLET_help)

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
        Refuse a section that names no unit, no extruder or one extruder on both outlets. A
        switch its chain made may name neither: its units and its extruders are found.
        """
        error_str = ""
        if (not self.chain_made
            and not (self.inlets["A"]
                     or self.inlets["B"]
                     or self.finding)):
            error_str = f"[{self.section}] names no unit. Add inlet_a or inlet_b."
        elif (not self.chain_made
              and not self.extruders()):
            error_str = f"[{self.section}] names no extruder. Add outlet_a or outlet_b."
        elif (self.extruders()
              and self.outlets["A"] == self.outlets["B"]):
            error_str = (f"[{self.section}] has {self.outlets['A']} on both outlets. An "
                         "outlet feeds one extruder.")
        if error_str:
            raise error(error_str)

    def _starts(self) -> Dict[str, str]:
        """
        The outlet each inlet's units start on: the one with its own letter, or the other one
        when only that outlet has an extruder.

        :return dict: inlet letter -> outlet letter
        """
        starts: Dict[str, str] = {}
        for letter in LETTERS:
            starts[letter] = letter
            if (self.outlets[letter] is None
                and self.outlets[other(letter)] is not None):
                starts[letter] = other(letter)
        return starts

    def _key(self, key: str) -> str:
        """
        One of the switch's options as it is written: in its own section, or with `switch_`
        before it in the section of the chain that made the switch.

        :param key: the option's name in a switch section
        :return str: the option's name where this switch's options are written
        """
        return f"switch_{key}" if self.chain_made else key

    def _length_keys(self) -> str:
        """
        The two tube length options as they are written, for a message that says to set them.

        :return str: both names
        """
        return f"{self._key('outlet_a_length')} and {self._key('outlet_b_length')}"

    def _save(self, keys: Dict[str, Any]) -> None:
        """
        Save some of the switch's options: in its own section, or with its chain for a switch
        the chain made.

        :param keys: the option's name in a switch section -> its value, "" for one that is not
        known any more
        """
        if self.chain_made:
            self.chain.save_switch(keys)
            return
        for key, value in keys.items():
            self.afc.function.ConfigRewrite(self.fullname, key, value)

    def handle_connect(self) -> None:
        """
        Find the chain, check the units and extruders exist and no other switch uses them, then
        put each unit's lanes on their extruder. A unit that is found by loading enters the
        inlet found for the AMS its bay is kept for, or for the unit itself when it is on no
        chain.
        """
        self.chain = self._find_chain()
        self.chain_name = self.chain.name
        self._place_chains()
        self._check_section()
        saved = self._saved()
        self._found = saved["found"]
        for unit in self._floating:
            self._held[unit] = self._id(unit)
            if self._held[unit] in self._found:
                self.inlets[self._found[self._held[unit]]].append(unit)
        problems = self._missing() + self._clashes()
        if problems:
            raise error(problems[0])
        self.lanes = [obj for _name, obj in self.printer.lookup_objects()
                      if (isinstance(obj, AFCLane)
                          and obj.unit in self.units())]
        # A lane AFC saved as loaded is in that toolhead, whatever was saved for the switch.
        self._saved_lanes = {**saved["lanes"], **self._saved_loaded()}
        if not self.extruders():
            # What was saved was for outlets the switch does not have now.
            self._saved_lanes = {}
        self._by_print = [name for name in saved["by_print"] if name in self._saved_lanes]
        self._due_back = [name for name in saved["due_back"] if name in self._saved_lanes]
        self.confirmed = [unit for unit in saved["confirmed"]
                          if self.inlet_of(unit) is not None]
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
        inlet starts on, `confirmed`, the units seen at their inlet, `by_print`, the lanes a
        print that was running put there, `due_back`, those of a print that is over, and
        `found`, AMS uid or unit name -> the inlet found for it by loading. All empty when
        nothing usable is saved
        """
        try:
            with open(f"{self.afc.VarFile}.unit") as file:
                saved = json.load(file)["system"]["switches"][self.name]
            return {"lanes": dict(saved["lanes"]), "confirmed": list(saved["confirmed"]),
                    "by_print": list(saved.get("by_print", [])),
                    "due_back": list(saved.get("due_back", [])),
                    "found": {uid: inlet for uid, inlet in dict(saved.get("found", {})).items()
                              if inlet in LETTERS}}
        except (OSError, ValueError, KeyError, TypeError):
            return {"lanes": {}, "confirmed": [], "by_print": [], "due_back": [], "found": {}}

    def _saved_loaded(self) -> Dict[str, str]:
        """
        The lanes AFC saved as loaded in one of the switch's toolheads.

        :return dict: lane name -> extruder name, empty when nothing usable is saved
        """
        try:
            with open(f"{self.afc.VarFile}.unit") as file:
                extruders = json.load(file)["system"]["extruders"]
            return {extruders[name]["lane_loaded"]: name for name in self.extruders()
                    if (name in extruders
                        and extruders[name]["lane_loaded"])}
        except (OSError, ValueError, KeyError, TypeError):
            return {}

    def handle_save_vars(self, data: Dict[str, Any]) -> None:
        """
        Add the lanes that are off their starting extruder, which of them a print put there,
        the units seen at their inlet and the inlet found for each AMS and unit, to what AFC
        saves.

        :param data: what AFC is about to save
        """
        away = {lane.name: lane.extruder_obj.name for lane in self.lanes
                if (self.extruders()
                    and self.inlet_of(lane.unit) is not None
                    and lane.extruder_obj.name != self.start_extruder(lane.unit))}
        data["system"].setdefault("switches", {})[self.name] = {
            "lanes": away, "confirmed": list(self.confirmed),
            "by_print": list(self._by_print), "due_back": list(self._due_back),
            "found": dict(self._found)}

    def start_extruder(self, unit: str) -> Optional[str]:
        """
        The extruder a unit's lanes start on: the one its inlet's own outlet feeds.

        :param unit: a unit of this switch
        :return str: AFC extruder name, None while no outlet has its toolhead
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
        extruder, whatever extruder the unit itself names. A chain's unit that is found by
        loading goes behind its inlet's buffer, and while a unit's inlet is not known its lanes
        stay where they are, but for one saved on an extruder of the switch.

        :param unit_obj: a unit of this switch that has connected
        """
        inlet = self.inlet_of(unit_obj.name)
        chain = self._floating.get(unit_obj.name)
        if chain is not None:
            # Until its inlet is found, the buffer it reports may be the other one.
            unit_obj.use_buffer(*chain.inlet_buffer(inlet), assumed=inlet is None)
        start = self.start_extruder(unit_obj.name) if inlet is not None else None
        moved: List[AFCLane] = []
        for lane in self.lanes:
            if lane.unit != unit_obj.name:
                continue
            saved = self._saved_lanes.get(lane.name, start)
            if saved not in self.extruders():
                saved = start
            if saved is not None:
                lane.set_extruder(self._extruder(saved))
                moved.append(lane)
        # An extruder that had no lane when Klipper was ready has one now, or AFC goes on
        # refusing to eject its lanes.
        self.chain._clear_standalone(moved)

    def _chains(self) -> Dict[str, afcBridgeBox]:
        """
        The BridgeBox chains that are configured.

        :return dict: chain name -> chain
        """
        return {obj.name: obj for _name, obj in self.printer.lookup_objects("AFC_BridgeBox")
                if isinstance(obj, afcBridgeBox)}

    def _place_chains(self) -> None:
        """
        Put each chain's bays on the inlets: a chain's name stands for the bays the other inlet
        does not name, and the bays no inlet names follow the ones that are named, so a unit that
        takes a bay later enters an inlet too. A unit that has a chain's name is left as the unit.
        A chain that is on both inlets has the bays no inlet names found by loading, and so is
        a unit that is on both inlets and is no chain's bay.
        """
        known = [obj.name for _name, obj in self.printer.lookup_objects()
                 if isinstance(obj, afcUnit)]
        chains = self._chains()
        bays = [bay for chain in chains.values() for bay in chain.unit_names()]
        units = [name for name in self.finding
                 if (name not in chains
                     or name in known)]
        for name in units:
            error_str = ""
            if name in bays:
                error_str = (f"[{self.section}] has {name} on both inlets. A chain's unit enters "
                             "one inlet. Put its chain on both inlets to have the chain's units "
                             "found by loading.")
            elif name not in known:
                error_str = (f"[{self.section}] names unit {name}, and there is no AFC unit of "
                             "that name.")
            if error_str:
                raise error(error_str)
            self._floating[name] = None
        for chain in chains.values():
            if (chain.name in self.finding
                and chain.name not in units):
                self._floating.update({bay: chain for bay in chain.unit_names()
                                       if self.inlet_of(bay) is None})
                continue
            try:
                self.inlets = place_chain(self.inlets, chain.name, chain.unit_names(),
                                          chain.name in known)
            except ValueError as e:
                error_str = f"[{self.section}] {e}"
                raise error(error_str)

    def _id(self, unit: str) -> Optional[str]:
        """
        What a found inlet is kept for: the AMS a chain's bay holds, by its uid, and a unit that
        is on no chain itself, by its name.

        :param unit: a unit that is found by loading
        :return str: the uid or the name, None for a bay that holds no AMS
        """
        chain = self._floating[unit]
        return chain.bay_uid(unit) if chain is not None else unit

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
            error_str = (f"[{self.section}] needs chain: there are {len(chains)} BridgeBox "
                         f"chains ({', '.join(chains)}).")
        elif name is None:
            error_str = f"[{self.section}] needs a BridgeBox chain, and none is configured."
        elif name not in chains:
            error_str = (f"[{self.section}] names chain {name}, and there is no "
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
        problems = [f"[{self.section}] names unit {name}, and there is no AFC unit of that name."
                    for name in self.units() if name not in units]
        problems += [f"[{self.section}] names extruder {name}, and there is no "
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
                problems.append(f"[{self.section}] and [{switch.section}] are both on chain "
                                f"{self.chain_name}. A chain's bus carries one filament switch.")
            if shared:
                problems.append(f"[{self.section}] and [{switch.section}] both name "
                                f"{', '.join(shared)}.")
        return problems

    def units(self) -> List[str]:
        """
        Every unit plumbed into the switch.

        :return list: unit names, inlet A's first and those whose inlet is not known last
        """
        return (self.inlets["A"] + self.inlets["B"]
                + [unit for unit in self._floating if self.inlet_of(unit) is None])

    def live_units(self) -> List[str]:
        """
        The units plumbed into the switch that are in use: all but a chain's bays that hold no
        unit now.

        :return list: unit names, inlet A's first
        """
        return [unit for unit in self.units()
                if not getattr(self.afc.units.get(unit), "pool", False)]

    def unfound(self) -> List[str]:
        """
        The units in use whose inlet is still to be found by loading.

        :return list: unit names
        """
        return [unit for unit in self.live_units() if self.inlet_of(unit) is None]

    def _need(self, unit: str) -> str:
        """
        What to do about a unit whose inlet is not known.

        :param unit: unit name
        :return str: the sentence an error about that unit ends with
        """
        return (f"the inlet of {unit} is not known. Run AFC_BRIDGEBOX_SWITCH_IDENTIFY "
                f"UNIT={unit} first.")

    def _unset(self) -> str:
        """
        What to do about a switch none of whose outlets has its toolhead.

        :return str: the sentence an error about that ends with
        """
        return ("the toolhead each outlet feeds is not known. Run AFC_BRIDGEBOX_SWITCH_SETUP "
                f"SWITCH={self.name} first, or set {self._key('outlet_a')} and "
                f"{self._key('outlet_b')}.")

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
        Reactor timer: bring the switch in line with the lanes while AFC is idle, and put the
        lanes of a print that is over back on their starting extruder.

        :param eventtime: reactor time of this call
        :return float: reactor time of the next call
        """
        self._look()
        printing = self.afc.function.in_print()
        self._release(printing)
        if (self.afc.prep_done
            and self.afc.current_state == State.IDLE
            and not self._moving):
            self._follow()
            self._note_arrivals()
            if not printing:
                self._send_back()
                self._offer()
            with self._own():
                self._say(self._settle())
        return self.reactor.monotonic() + _WATCH_PERIOD

    def _release(self, printing: bool) -> None:
        """
        Note a print that ended since the last look: the lanes it put on an extruder are due
        back on the one their unit starts on. A print Klipper restarted under keeps its lanes
        where they are, for when it is resumed, until a print ends.

        :param printing: whether a print runs now
        """
        if (self._printing
            and not printing
            and self._by_print):
            self._due_back += self._by_print
            self._by_print = []
            self.afc.save_vars()
        self._printing = printing

    def _follow(self) -> None:
        """
        Keep each unit that is found by loading on the inlet found for the AMS in its bay, or
        for the unit itself when it is on no chain. A bay another AMS took has no inlet until
        that AMS is fed to the switch, and what was found for an AMS that left its bay and holds
        none is forgotten.
        """
        held = {unit: self._id(unit) for unit in self._floating}
        gone = [uid for uid in self._held.values()
                if (uid in self._found
                    and uid not in held.values())]
        for uid in gone:
            del self._found[uid]
        self._held = held
        for unit, uid in held.items():
            inlet = self._found.get(uid)
            if inlet != self.inlet_of(unit):
                self._place(unit, inlet)
        if gone:
            self.afc.save_vars()

    def _place(self, unit: str, inlet: Optional[str]) -> None:
        """
        Put a unit that is found by loading on an inlet, or on none, and its lanes on that
        inlet's starting extruder and behind its buffer.

        :param unit: a unit that is found by loading
        :param inlet: `A` or `B`, None for a unit whose inlet is not known
        """
        for letter in LETTERS:
            self.inlets[letter] = [name for name in self.inlets[letter] if name != unit]
        if inlet is not None:
            self.inlets[inlet].append(unit)
        elif unit in self.confirmed:
            self.confirmed.remove(unit)
        # What was kept for its lanes was for the inlet the unit had.
        names = [lane.name for lane in self.lanes if lane.unit == unit]
        self._saved_lanes = {name: extruder for name, extruder in self._saved_lanes.items()
                             if name not in names}
        self._by_print = [name for name in self._by_print if name not in names]
        self._due_back = [name for name in self._due_back if name not in names]
        if unit in self.afc.units:
            self._start_unit(self.afc.units[unit])

    def _offer(self) -> None:
        """
        Open a prompt, once for each AMS and unit, that offers to find the inlet of the units
        whose inlet is not known. A switch that has no toolhead on an outlet yet offers to be
        set up first, once.
        """
        if not self.extruders():
            self._offer_setup()
            return
        unfound = self.unfound()
        uids = [self._held.get(unit) for unit in unfound]
        new = [uid for uid in uids if uid not in self._offered]
        if not new:
            return
        self._offered += new
        command = f"AFC_BRIDGEBOX_SWITCH_IDENTIFY SWITCH={self.name}"
        buttons = [(unit, f"{command} UNIT={unit}", "primary") for unit in unfound]
        if len(unfound) > 1:
            buttons.append(("All new units", f"{command} UNIT=all", "default"))
        lines = [f"{self.name}: the inlet of {', '.join(unfound)} is not known yet. The lanes "
                 "of a unit do not load until its inlet is found.",
                 "Put a spool in the unit and pick it. One lane is fed to the switch and reeled "
                 "back, and a lane that is loaded through the switch is unloaded first."]
        self.chain.offer(self.name, f"{self.name} new unit", lines, buttons)

    def _offer_setup(self) -> None:
        """
        Open the prompt, once, that offers to set up a switch none of whose outlets has its
        toolhead. When no toolhead sensor can find the outlets it offers the inlets alone, and
        to set the outlets by hand.
        """
        if self._setup_offered:
            return
        self._setup_offered = True
        lines = [f"{self.name}: the filament switch is not set up yet, and its lanes do not "
                 "load until it is."]
        label = "Set up"
        buttons: List[Tuple[str, str, str]] = []
        no_sensor = self._no_sensor()
        if no_sensor:
            label = "Find the inlets"
            lines += [no_sensor,
                      "The inlets can be found now. Put a spool in each unit, then pick "
                      f"{label}: one lane of each unit is fed to the switch and reeled back."]
            buttons.append(("Set by hand", f"AFC_BRIDGEBOX_SWITCH_OUTLET SWITCH={self.name}",
                            "default"))
        else:
            lines.append("Put a spool in each unit and unload the toolheads the switch feeds, "
                         f"then pick {label}. One lane of each unit is fed to the switch to "
                         "find its inlet, then a lane is fed through each outlet until a "
                         "toolhead sensor reads it. Each lane is reeled back.")
            note = self._sensorless_note()
            if note:
                lines.append(note)
        buttons.insert(0, (label, f"AFC_BRIDGEBOX_SWITCH_SETUP SWITCH={self.name}", "primary"))
        self.chain.offer(self.name, f"{self.name} filament switch", lines, buttons)

    def _send_back(self) -> None:
        """
        Put the lanes that are due back on their unit's starting extruder, each once it is out
        of its toolhead and not on its way into one. A lane of a bay that holds no unit now
        waits for its unit: what AFC saved for it is not on the lane until then. A lane of a
        unit whose inlet is not known waits for the inlet.
        """
        live = self.live_units()
        back = [lane for lane in self.lanes
                if (lane.name in self._due_back
                    and lane.unit in live
                    and self.inlet_of(lane.unit) is not None
                    and not lane.tool_loaded
                    and lane not in self._routed.values())]
        for lane in back:
            was = lane.extruder_obj.name
            start = self.start_extruder(lane.unit)
            lane.set_extruder(self._extruder(start))
            self._due_back.remove(lane.name)
            self.logger.info(f"{self.name}: {lane.name} is back on {start}, the print that put "
                             f"it on {was} is over")
        if back:
            self.afc.save_vars()

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
        if not self.extruders():
            # There is nowhere to point an inlet until an outlet has its toolhead.
            return []
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
        own length back once there is none. A unit that is still coming into AFC is left out:
        it gets its own lengths on the way in.
        """
        active = [lane for lane in self.lanes
                  if ((lane.tool_loaded
                       or lane in self._routed.values())
                      and not getattr(lane.unit_obj, "arriving", False))]
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
        What holds the length of a lane's tube to the toolhead: the lane's unit when it keeps
        that length itself, else the lane's hub.

        :param lane: a lane of this switch
        :return Any: the unit or the hub, None for a lane that has neither
        """
        for holder in (lane.unit_obj, lane.hub_obj):
            if hasattr(holder, "afc_bowden_length"):
                return holder
        return None

    def _by_feed(self, lane: AFCLane) -> bool:
        """
        Whether a lane's unit counts the way a lane goes when it feeds it to a sensor, so the
        lane is measured by feeding it to the toolhead and back.

        :param lane: a lane of this switch
        :return bool: True for a unit that has a feed of its own that measures
        """
        return hasattr(lane.unit_obj, "measure_to")

    def _set_length(self, lane: AFCLane) -> None:
        """
        Give a lane's tube the length for the outlet the lane goes through: its own, plus the
        difference between the two outlets' tubes when that is not the outlet its unit starts
        on.

        :param lane: a lane that is loaded or about to be
        """
        tube = self._tube(lane)
        outlet = self.outlet_of(lane.extruder_obj.name)
        inlet = self.inlet_of(lane.unit)
        if (tube is None
            or outlet is None
            or inlet is None):
            return
        start = self.start[inlet]
        offset = 0.0
        if (outlet != start
            and self.calibrated()):
            offset = self.lengths[outlet] - self.lengths[start]
        self._offset(tube, offset)

    def _offset(self, tube: Any, offset: float) -> None:
        """
        Change what is added to a tube's own length, for loading and for unloading.

        :param tube: what `_tube` returned
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
                         "loaded, so it is left where it is. Run LANE_UNLOAD on the lane "
                         "that is in it.")
            raise SwitchError(error_str)
        elif (self.afc.function.in_print()
              or (there["filament"]
                  and there["outlet"] == self.rest[inlet])):
            return
        else:
            outlet = self.rest[inlet]
        self.set_inlet(inlet, outlet)

    def handle_tool_change_unload(self, held: AFCLane, lane: AFCLane) -> bool:
        """
        Have a tool change unload the loaded lane while its toolhead is still the active one
        when it is in the way of the next lane, so that toolhead is not picked up again for it.

        :param held: the lane loaded in the active toolhead
        :param lane: the lane the tool change loads next
        :return bool: True when the loaded lane has to be unloaded first
        """
        inlet = self.inlet_of(lane.unit)
        outlet = self.outlet_of(lane.extruder_obj.name)
        if (inlet is None
            or outlet is None
            or lane.tool_loaded):
            return False
        return held in self._in_the_way(lane, inlet, outlet)

    def handle_tool_load_prepare(self, lane: AFCLane, gcmd: AFCGcmd) -> bool:
        """
        Point a lane's inlet at its extruder's outlet before AFC loads the lane. A lane of a
        unit whose inlet is not known is not loaded, and neither is any lane of a switch none
        of whose outlets has its toolhead.

        :param lane: the lane about to be loaded
        :param gcmd: command context the load runs through
        :return bool: False stops the load
        """
        inlet = self.inlet_of(lane.unit)
        if (lane.tool_loaded
            or lane.unit not in self.units()):
            return True
        need = ""
        if not self.extruders():
            need = self._unset()
        elif inlet is None:
            need = self._need(lane.unit)
        if need:
            self.afc.error.AFC_error(f"{self.name}: {lane.name} is not loaded, {need}",
                                     pause=self.afc.function.in_print(), gcmd=gcmd)
            return False
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
        Unload what is in the way of a lane, then point the lane's inlet at its outlet. An inlet
        with no lane loaded is given time to read empty first, as its lane may have just left
        it, unloaded here or by the tool change this load belongs to.

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
        for letter in LETTERS:
            if self._loaded(letter) is None:
                self._wait_clear(letter)
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
        its extruder and the lanes that feed it. Then what is still to be set up, the units whose
        inlet is not known last.

        :return str: the report
        """
        st = self.state()
        unfound = self.unfound()
        live = [unit for unit in self.live_units() if unit not in unfound]
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
        if not self.extruders():
            lines.append("Outlets not set up: run AFC_BRIDGEBOX_SWITCH_SETUP, or set "
                         f"{self._key('outlet_a')} and {self._key('outlet_b')}")
        if not self.calibrated():
            lines.append(f"Tube lengths not set: run AFC_CALIBRATION, or set {self._length_keys()}")
        waiting = [unit for unit in live if unit not in self.confirmed]
        if waiting:
            lines.append(f"Not seen at their inlet yet: {', '.join(waiting)}")
        if unfound:
            lines.append(f"Inlet not known: {', '.join(unfound)}. Run "
                         "AFC_BRIDGEBOX_SWITCH_IDENTIFY, or pick the switch and its inlets in "
                         "AFC_CALIBRATION")
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

        While a print runs the choice is for that print: once the print is over and the lane is
        out of its toolhead, the lane goes back to the extruder its unit starts on.

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
        Put one of this switch's lanes on one of its extruders and save the choice. While a
        print runs the choice is for that print, at any other time it is kept. A lane of a unit
        whose inlet is not known is refused, and so is any lane of a switch none of whose
        outlets has its toolhead.

        :param lane: the lane
        :param extruder: AFC extruder name, None for the one the lane's unit starts on
        :param unload: True to unload the lane first when it is loaded in a toolhead
        :param gcmd: the command, for its answer and the unload
        """
        need = ""
        if not self.extruders():
            need = self._unset()
        elif self.inlet_of(lane.unit) is None:
            need = self._need(lane.unit)
        if need:
            error_str = f"{self.name}: {lane.name} stays where it is, {need}"
            raise gcmd.error(error_str)
        extruder = extruder or self.start_extruder(lane.unit)
        was = lane.extruder_obj.name
        if extruder not in self.extruders():
            error_str = f"{self.name}: EXTRUDER takes {' or '.join(self.extruders())}"
            raise gcmd.error(error_str)
        printing = self.afc.function.in_print()
        if extruder == was:
            # Outside a print this keeps a lane where a print put it.
            if not printing:
                self._note_assign(lane, False)
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
        self._note_assign(lane, printing)
        until = " for this print" if lane.name in self._by_print else ""
        gcmd.respond_info(f"{self.name}: {lane.name} now feeds {extruder}{until}")

    def _note_assign(self, lane: AFCLane, printing: bool) -> None:
        """
        Note whether a lane is on its extruder for a running print only, and save that.

        :param lane: the lane that was assigned
        :param printing: whether a print runs now
        """
        self._by_print = [name for name in self._by_print if name != lane.name]
        self._due_back = [name for name in self._due_back if name != lane.name]
        if (printing
            and lane.extruder_obj.name != self.start_extruder(lane.unit)):
            self._by_print.append(lane.name)
        self.afc.save_vars()

    def handle_calibration_buttons(self, buttons: List[Tuple[str, str, str]]) -> None:
        """
        Add this switch to AFC's calibration menu.

        :param buttons: the menu's buttons so far, each a label, a command and a style
        """
        buttons.append((f"{self.name} filament switch",
                        f"AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH={self.name}", "primary"))

    cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION_help = ("AFC_BRIDGEBOX_SWITCH_CALIBRATION "
                                                 "[SWITCH=<switch>] [OUTLETS=1] [UNIT=<unit>] "
                                                 "[LANE=<lane>]. Measures the tube from each "
                                                 "outlet of a filament switch to its toolhead")

    def cmd_AFC_BRIDGEBOX_SWITCH_CALIBRATION(self, gcmd: GCodeCommand) -> None:
        """
        Measure the tube from each outlet of the filament switch to its toolhead with one lane,
        and save both lengths.

        The lane's unit runs its own tube length calibration through each outlet, so the lane
        has to be loaded in its unit and the toolheads empty. A unit that counts the way of
        its own feed has the lane fed to each toolhead and pulled back instead, with nothing
        loaded.
        Each run also checks the lane shows at the inlet its unit is configured on. Without
        LANE a prompt offers the units that can measure, and with UNIT the lanes of that unit.
        A switch with units whose inlet is found by loading first offers what to calibrate, the
        inlets or the outlets, and OUTLETS=1 goes on to the units that can measure. A prompt
        says what is being done, and the prompt that offers the units opens again with the
        outcome. On a switch its chain made, the prompt for the outlets also offers to find the
        toolhead each outlet feeds.

        Usage
        -------
        `AFC_BRIDGEBOX_SWITCH_CALIBRATION [SWITCH=<switch>] [OUTLETS=1] [UNIT=<unit>]
        [LANE=<lane>]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH=Switch_1 LANE=lane8
        ```
        """
        name = gcmd.get("LANE", None)
        unit = gcmd.get("UNIT", None)
        if name is None:
            if unit is not None:
                self._lane_menu(unit, gcmd)
            else:
                self._open_menu(gcmd, bool(gcmd.get_int("OUTLETS", 0)))
            return
        lane = self.lane(name)
        prompt = AFCprompt(gcmd, self.logger)
        try:
            if lane is None:
                error_str = f"{self.name}: {name} is not a lane of this switch"
                raise SwitchError(error_str)
            text = (f"Measuring the tube from each outlet to its toolhead with {lane.name}. The "
                    "lane goes to each toolhead in turn and back.")
            prompt.create_custom_p(f"{self.name} Outlets", text, None, False, None, None)
            lengths = self._measure(lane, gcmd)
        except SwitchResetError as ex:
            # AFC's prompt to reset the lane is open, and stays.
            raise gcmd.error(str(ex))
        except SwitchError as ex:
            self._calibration_menu(gcmd, str(ex))
            raise gcmd.error(str(ex))
        except Exception:
            # The prompt has no button, so it is not left open.
            prompt.p_end()
            raise
        for letter in LETTERS:
            self.lengths[letter] = round(lengths[letter], 1)
        self._save({f"outlet_{letter.lower()}_length": self.lengths[letter]
                    for letter in LETTERS})
        found = (f"{self.name}: with {lane.name} the tube through outlet A is "
                 f"{self.lengths['A']} mm and through outlet B {self.lengths['B']} mm")
        gcmd.respond_info(found)
        self._calibration_menu(gcmd, found)

    def _open_menu(self, gcmd: GCodeCommand, outlets: bool, outcome: str = "") -> None:
        """
        Open the prompt a calibration starts from: the one that offers what to calibrate on a
        switch with units whose inlet is found by loading, else the one for the outlets.

        :param gcmd: the command the prompt answers
        :param outlets: True for the outlets' prompt on any switch
        :param outcome: what the last run gave or why it failed, shown above the offer
        """
        if (self._floating
            and not outlets):
            self._menu(gcmd, outcome)
        else:
            self._calibration_menu(gcmd, outcome)

    def _menu(self, gcmd: GCodeCommand, outcome: str = "") -> None:
        """
        Open the prompt that offers what to calibrate on a switch with units whose inlet is
        found by loading: the inlets or the outlets. While an outlet that can be set by hand
        has no toolhead, it offers that too.

        :param gcmd: the command the prompt answers
        :param outcome: what the last run gave or why it failed, shown above the offer
        """
        buttons = [("Inlets", f"AFC_BRIDGEBOX_SWITCH_IDENTIFY SWITCH={self.name}", "primary"),
                   ("Outlets", f"AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH={self.name} OUTLETS=1",
                    "secondary")]
        without = [letter for letter in self._to_find() if self.outlets[letter] is None]
        text = ("Inlets finds the inlet each unit is plumbed into, by feeding one of its lanes "
                "to the switch. Outlets measures the tube from each outlet to its toolhead, "
                "with a lane of a unit whose inlet is known.")
        unfound = self.unfound()
        if unfound:
            text += f" Inlet not known yet: {', '.join(unfound)}."
        if not self.extruders():
            text += " The toolhead each outlet feeds is not known yet: Outlets finds them."
        elif not self.calibrated():
            text += " The tube lengths are not measured yet."
        if without:
            buttons.append(("Set outlets by hand",
                            f"AFC_BRIDGEBOX_SWITCH_OUTLET SWITCH={self.name}", "default"))
            text += (f" Set outlets by hand picks the toolhead of outlet {' and '.join(without)}"
                     ", for a toolhead that has no sensor to find it with.")
        if outcome:
            text = f"{outcome}\n{text}"
        back = [("Back to unit selection", "AFC_CALIBRATION", "info")]
        AFCprompt(gcmd, self.logger).create_custom_p(f"{self.name} Calibration", text, buttons,
                                                     True, None, back)

    def _back(self) -> List[Tuple[str, str, str]]:
        """
        The footer button of a prompt that offers units: back to the prompt that offers what to
        calibrate, or to AFC's calibration prompt when this switch has none.

        :return list: one button, a label, a command and a style
        """
        if self._floating:
            return [("Back", f"AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH={self.name}", "info")]
        return [("Back to unit selection", "AFC_CALIBRATION", "info")]

    def _calibration_menu(self, gcmd: GCodeCommand, outcome: str = "") -> None:
        """
        Open the prompt that offers the units whose lanes can measure the outlets, and on a
        switch its chain made to find the toolhead each outlet feeds or to set it by hand.

        :param gcmd: the command the prompt answers
        :param outcome: what the last run measured or why it failed, shown above the offer
        """
        lanes: Dict[str, List[AFCLane]] = {}
        if self.extruders():
            lanes = self._measuring_lanes()
        command = f"AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH={self.name}"
        buttons = [(unit, f"{command} UNIT={unit}", "primary" if index % 2 == 0 else "secondary")
                   for index, unit in enumerate(lanes)]
        text = ("Measures the tube from each outlet to its toolhead with one lane, so a lane "
                "loads the right distance through either outlet. Unload the toolheads, then "
                "pick the unit to measure with.")
        if not buttons:
            text = ("No lane that can measure is loaded in its unit. Load one, or set "
                    f"{self._length_keys()} in [{self.section}] by hand.")
        no_sensor = self._no_sensor()
        if not self.extruders():
            text = ("The toolhead each outlet feeds is not known yet. Find the toolheads feeds "
                    "a lane through each outlet until a toolhead sensor reads it, and reels it "
                    "back. Unload the toolheads first. Set by hand picks the toolhead of an "
                    "outlet that feeds one with no sensor.")
            if no_sensor:
                text = no_sensor
        if (self._to_find()
            and not no_sensor):
            buttons.append(("Find the toolheads",
                            f"AFC_BRIDGEBOX_SWITCH_SETUP SWITCH={self.name} OUTLETS=1",
                            "default"))
        if self._to_find():
            buttons.append(("Set by hand", f"AFC_BRIDGEBOX_SWITCH_OUTLET SWITCH={self.name}",
                            "default"))
        if outcome:
            text = f"{outcome}\n{text}"
        AFCprompt(gcmd, self.logger).create_custom_p(f"{self.name} Outlets", text, buttons,
                                                     True, None, self._back())

    def _lane_menu(self, unit: str, gcmd: GCodeCommand) -> None:
        """
        Open the prompt that offers a unit's lanes to measure the outlets with, two to a row.

        :param unit: the unit picked in the switch's calibration prompt
        :param gcmd: the command the prompt answers
        """
        lanes = self._measuring_lanes().get(unit, [])
        error_str = ""
        if unit not in self.units():
            error_str = f"{self.name}: {unit} is not a unit of this switch"
        elif not lanes:
            error_str = (f"{self.name}: no lane that can measure is loaded in {unit}. Load one, "
                         f"pick another unit, or set {self._length_keys()} by hand.")
        if error_str:
            raise gcmd.error(error_str)
        command = f"AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH={self.name}"
        buttons = [(lane.name, f"{command} LANE={lane.name}",
                    "primary" if index % 2 == 0 else "secondary")
                   for index, lane in enumerate(lanes)]
        rows = [buttons[first:first + 2] for first in range(0, len(buttons), 2)]
        text = f"Select a loaded lane from {unit} to measure the tube from each outlet with."
        back = [("Back", f"{command} OUTLETS=1", "info")]
        AFCprompt(gcmd, self.logger).create_custom_p(f"{self.name} Outlets {unit}", text,
                                                     None, True, rows, back)

    def _measuring_lanes(self) -> Dict[str, List[AFCLane]]:
        """
        The lanes the outlets can be measured with, by unit: the lanes that are loaded in their
        unit and have a tube length to measure. A unit whose calibration gave no length is not
        offered again, and neither is one whose inlet is not known.

        :return dict: unit -> its lanes that can measure, in the order of the switch's lanes
        """
        lanes: Dict[str, List[AFCLane]] = {}
        for lane in self.lanes:
            if (lane.load_state
                and self._tube(lane) is not None
                and lane.unit not in self._lengthless
                and self.inlet_of(lane.unit) is not None):
                lanes.setdefault(lane.unit, []).append(lane)
        return lanes

    def _measure(self, lane: AFCLane, gcmd: GCodeCommand) -> Dict[str, float]:
        """
        Measure a lane's tube through each outlet with its unit's own calibration, or for a
        unit that counts the way of its own feed by feeding the lane to the toolhead, leaving
        the lane, its unit's length and the switch as they were.

        :param lane: the lane to measure with
        :param gcmd: the command the calibrations or feeds run through
        :return dict: outlet letter -> mm
        """
        inlet = self.inlet_of(lane.unit)
        tube = self._tube(lane)
        loaded = [held.name for held in self.lanes if held.tool_loaded]
        if self.afc.current is not None:
            loaded.append(self.afc.current)
        error_str = ""
        if inlet is None:
            error_str = f"{self.name}: {lane.name} cannot measure, {self._need(lane.unit)}"
        elif len(self.extruders()) < 2:
            error_str = f"{self.name}: one outlet is in use, so there is nothing to measure"
        elif tube is None:
            error_str = (f"{self.name}: {lane.name} has no hub that holds a tube length. Use a "
                         f"lane of another unit, or set {self._length_keys()} by hand.")
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
        Measure a lane's tube through one outlet, with the unit's own tube length calibration
        or by feeding it to the toolhead, and check the lane showed at its inlet on the way.

        :param lane: the lane to measure with
        :param inlet: the lane's inlet
        :param outlet: the outlet to measure through
        :param gcmd: the command the calibration or the feed runs through
        :return float: mm from the lane's hub or unit to the outlet's toolhead
        """
        extruder = self.outlets[outlet]
        lane.set_extruder(self._extruder(extruder))
        # Keeps the watch from sending the inlet back to rest.
        self._routed[inlet] = lane
        with self._own():
            self.set_inlet(inlet, outlet)
        self._seen = {letter: False for letter in LETTERS}
        if self._by_feed(lane):
            done, why, length = self._feed_to_measure(lane, inlet, gcmd)
        else:
            done, why, length = lane.unit_obj.calibrate_bowden(lane, _MEASURE_STEP,
                                                               _MEASURE_TOLERANCE, gcmd=gcmd)
        self._look()
        error_str = ""
        if not done:
            reason = f" {why.rstrip('.')}." if why else ""
            # A unit that fed the lane itself has pulled it back.
            reset = "" if self._by_feed(lane) else f" Reset {lane.name} before the next try."
            error_str = (f"{self.name}: {lane.name} did not measure the tube to {extruder} "
                         f"through outlet {outlet}.{reason}{reset}")
            if self._offer_reset(lane, length, gcmd):
                raise SwitchResetError(error_str)
        elif length <= 0:
            if lane.unit not in self._lengthless:
                self._lengthless.append(lane.unit)
            error_str = (f"{self.name}: the unit of {lane.name} measures no tube length. Use a "
                         f"lane of another unit, or set {self._length_keys()} by hand.")
        elif not self._seen[inlet]:
            saw = (f"Inlet {other(inlet)} did" if self._seen[other(inlet)]
                   else "Neither inlet did")
            error_str = (f"{self.name}: {lane.name} reached {extruder} and inlet {inlet} never "
                         f"read filament. {saw}. Check where {lane.unit} is plumbed.")
        if error_str:
            raise SwitchError(error_str)
        return float(length)

    def _offer_reset(self, lane: AFCLane, moved: float, gcmd: GCodeCommand) -> bool:
        """
        Open AFC's failed calibration prompt for a lane that stopped on its way to the toolhead
        and left filament past its hub. The prompt's button pulls the lane back to the hub,
        which frees the inlet and lets the next try start from there.

        :param lane: the lane that did not measure
        :param moved: mm the lane's calibration says it fed before it stopped
        :param gcmd: the command the prompt answers
        :return bool: True when the prompt was opened
        """
        if not moved:
            return False
        if not getattr(lane.hub_obj, "state", False):
            return False
        self.afc.run_script(f"AFC_CALI_FAIL TITLE='{self.name} Calibration Failed' "
                            f"FAIL={lane.name} DISTANCE={abs(moved):.1f} RESET=1", gcmd=gcmd)
        return True

    def _feed_to_measure(self, lane: AFCLane, inlet: str,
                         gcmd: GCodeCommand) -> Tuple[bool, str, float]:
        """
        Feed a lane to the toolhead it is on and back with its unit's own feed, and take the
        way the unit counted it going. The feed ends when the toolhead's sensor reads the
        lane, or where the unit itself finds it at an extruder, as one that loads by buffer
        ramming has no sensor to read. A toolhead whose sensor reads filament already is not
        fed.

        :param lane: the lane to measure with, on the extruder to measure to
        :param inlet: the lane's inlet
        :param gcmd: the command the feed runs through
        :return tuple: whether it measured, why not, and the mm from the unit to the toolhead
        """
        extruder = lane.extruder_obj
        reads = self._sensor(extruder)
        if reads:
            return False, (f"The toolhead sensor of {extruder.name} reads filament, so nothing "
                           "was fed. Unload that toolhead first"), 0.0
        self._watched = {}
        if reads is False:
            self._watched = {extruder.name: extruder}
        self._read = []
        length = lane.unit_obj.measure_to(lane, self._saw_toolhead, gcmd)
        self._wait_clear(inlet)
        if length is None:
            return False, f"{lane.unit} counted no length to the toolhead, and pulled it back", 0.0
        return True, "", float(length)

    def _restore_tube(self, tube: Any, lengths: Tuple[float, float]) -> None:
        """
        Put a tube's own lengths back, in memory and where it saves them, when a measurement
        through the other outlet left that outlet's length in them.

        :param tube: what `_tube` returned
        :param lengths: its own load and unload lengths
        """
        if (tube.afc_bowden_length, tube.afc_unload_bowden_length) == lengths:
            return
        keep = getattr(tube, "keep_path", None)
        if keep is not None:
            keep(*lengths)
            return
        tube.afc_bowden_length, tube.afc_unload_bowden_length = lengths
        self.afc.function.ConfigRewrite(tube.fullname, "afc_bowden_length", lengths[0])
        self.afc.function.ConfigRewrite(tube.fullname, "afc_unload_bowden_length", lengths[1])

    cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY_help = ("AFC_BRIDGEBOX_SWITCH_IDENTIFY [SWITCH=<switch>] "
                                              "[UNIT=<unit>] [LANE=<lane>]. Feeds a lane to a "
                                              "filament switch to find the inlet its unit is "
                                              "plumbed into")

    def cmd_AFC_BRIDGEBOX_SWITCH_IDENTIFY(self, gcmd: GCodeCommand) -> None:
        """
        Find the inlet a unit is plumbed into by feeding one of its lanes to the filament
        switch.

        The lane is fed until an inlet reads its filament and is then reeled back into its
        unit. That inlet is the unit's from then on: its lanes start on the extruder of the
        outlet with the same letter, and a chain's unit goes through that inlet's buffer. A
        lane that is loaded through the switch is unloaded first. It is for a unit, and the
        units of a chain, that are on both inlets, and an AMS that is new on such a chain does
        not load until it was run. A lane AFC moves itself is pulled back as far as it was
        fed, and a unit that only runs whole loads is refused. Without UNIT and LANE a prompt
        offers those units, and with UNIT alone the unit's first lane that holds filament is
        fed. UNIT=all runs every unit whose inlet is not known, one after the other until one
        fails, and every unit once all are known. A prompt says what is being done, and the
        prompt that offers the units opens again with the outcome, for the next unit.

        Usage
        -------
        `AFC_BRIDGEBOX_SWITCH_IDENTIFY [SWITCH=<switch>] [UNIT=<unit>] [LANE=<lane>]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_SWITCH_IDENTIFY SWITCH=Switch_1 UNIT=Bambu_AMS_2
        ```
        """
        name = gcmd.get("LANE", None)
        unit = gcmd.get("UNIT", None)
        if (name is None
            and unit is None):
            self._identify_menu(gcmd)
            return
        units = [unit]
        if (name is None
            and unit.lower() == "all"):
            units = self._all()
        prompt = AFCprompt(gcmd, self.logger)
        # What each unit before the one that is being found gave.
        done: List[str] = []
        try:
            self._identify_units(units, name, done, prompt, gcmd)
        except SwitchError as ex:
            self._identify_menu(gcmd, "\n".join(done + [str(ex)]))
            raise gcmd.error(str(ex))
        except Exception:
            # The prompt has no button, so it is not left open.
            prompt.p_end()
            raise
        self._identify_menu(gcmd, "\n".join(done))

    def _identify_units(self, units: List[Optional[str]], name: Optional[str], done: List[str],
                        prompt: AFCprompt, gcmd: GCodeCommand) -> None:
        """
        Find the inlet of each of some units, one after the other until one fails, with a
        prompt that says what is being done.

        :param units: the units, or None for the unit of the lane that is named
        :param name: the lane to feed, None for each unit's first that holds filament
        :param done: what each unit gave, added to as each is found
        :param prompt: the prompt that says what is being done
        :param gcmd: the command the unloads and the feeds run through
        """
        for unit in units:
            lane = self._identify_lane(unit, name)
            text = (f"Finding the inlet of {lane.unit}: {lane.name} is fed to the switch and "
                    "reeled back. A lane that is loaded through the switch is unloaded first.")
            prompt.create_custom_p(f"{self.name} Inlets", "\n".join(done + [text]), None,
                                   False, None, None)
            found = self._entered(lane.unit, self._identify(lane, gcmd))
            gcmd.respond_info(found)
            done.append(found)

    def _entered(self, unit: str, inlet: str) -> str:
        """
        What finding a unit's inlet gave.

        :param unit: the unit
        :param inlet: the inlet found for it, `A` or `B`
        :return str: the sentence that says so, with the extruder its lanes start on when the
        outlets have their toolheads
        """
        found = f"{self.name}: {unit} enters inlet {inlet}."
        if self.extruders():
            found += f" Its lanes start on {self.start_extruder(unit)}."
        return found

    def _all(self) -> List[str]:
        """
        The units `UNIT=all` finds the inlet of: the ones whose inlet is not known, and once
        every inlet is known every unit that is found by loading.

        :return list: unit names
        """
        live = self.live_units()
        return self.unfound() or [unit for unit in self._floating if unit in live]

    def _identify_menu(self, gcmd: GCodeCommand, outcome: str = "") -> None:
        """
        Open the prompt that offers the units whose inlet is found by loading, and all of
        them at once when that is more than one.

        :param gcmd: the command the prompt answers
        :param outcome: what the last run found or why it failed, shown above the offer
        """
        command = f"AFC_BRIDGEBOX_SWITCH_IDENTIFY SWITCH={self.name}"
        live = self.live_units()
        buttons = []
        for unit in self._floating:
            if unit not in live:
                continue
            inlet = self.inlet_of(unit)
            if inlet is None:
                buttons.append((f"{unit}: new", f"{command} UNIT={unit}", "primary"))
            else:
                buttons.append((f"{unit}: inlet {inlet}", f"{command} UNIT={unit}", "secondary"))
        if len(self._all()) > 1:
            buttons.append(("All new units" if self.unfound() else "All units",
                            f"{command} UNIT=all", "default"))
        text = ("Feeds one lane of a unit to the switch and reels it back, to find the inlet "
                "the unit is plumbed into. A lane that is loaded through the switch is unloaded "
                "first. Put a spool in the unit, then pick it.")
        if not buttons:
            text = f"No unit's inlet is found by loading. {self._by_loading()}"
        if outcome:
            text = f"{outcome}\n{text}"
        AFCprompt(gcmd, self.logger).create_custom_p(f"{self.name} Inlets", text, buttons, True,
                                                     None, self._back())

    def _identify_lane(self, unit: Optional[str], name: Optional[str]) -> AFCLane:
        """
        The lane to feed to the switch: the one named, else the unit's first that holds
        filament.

        :param unit: the unit whose inlet to find, None when the lane names it
        :param name: the lane to feed, None for the unit's first that holds filament
        :return AFCLane: the lane
        """
        lane = self.lane(name) if name is not None else None
        if lane is not None:
            unit = lane.unit
        elif name is None:
            lane = next((held for held in self.lanes
                         if (held.unit == unit
                             and held.load_state)), None)
        error_str = ""
        if (name is not None
            and lane is None):
            error_str = f"{self.name}: {name} is not a lane of this switch"
        elif unit not in self._floating:
            error_str = (f"{self.name}: the inlet of {unit} is not found by loading. "
                         f"{self._by_loading()}")
        elif (unit not in self.live_units()
              or self._id(unit) is None):
            error_str = f"{self.name}: {unit} holds no AMS now"
        elif lane is None:
            error_str = f"{self.name}: no lane of {unit} holds filament. Put a spool in it first."
        elif not lane.load_state:
            error_str = f"{self.name}: {lane.name} holds no filament. Put a spool in it first."
        elif not self._feeds(lane):
            error_str = (f"{self._whole_loads(lane)} Name {unit} on one inlet in "
                         f"[{self.section}]: {self._key('inlet_a')} or {self._key('inlet_b')}.")
        if error_str:
            raise SwitchError(error_str)
        return lane

    def _by_loading(self) -> str:
        """
        Which units have their inlet found by loading, for a message about one that has not.

        :return str: the sentence that says so
        """
        return (f"That is for a unit or a chain that is on both {self._key('inlet_a')} and "
                f"{self._key('inlet_b')} in [{self.section}].")

    def _feeds(self, lane: AFCLane) -> bool:
        """
        Whether a lane can be fed until a sensor reads it and then reeled back: its unit has a
        feed of its own for that, or AFC moves the lane by distance.

        :param lane: a lane of this switch
        :return bool: False for a lane that only its unit's own whole loads move
        """
        unit = lane.unit_obj
        return (hasattr(unit, "feed_to")
                or lane.drive_stepper is not None
                or getattr(lane, "extruder_stepper", None) is not None
                or (getattr(unit, "stepperless_drive", False)
                    and hasattr(unit, "lane_move")))

    def _whole_loads(self, lane: AFCLane) -> str:
        """
        Why a lane cannot be fed to a sensor and reeled back.

        :param lane: a lane `_feeds` refuses
        :return str: the sentence an error about that lane starts with
        """
        return (f"{self.name}: {lane.unit} only runs whole loads, so {lane.name} cannot be fed "
                "to a sensor and reeled back.")

    def _feed(self, lane: AFCLane, seen: Callable[[], bool],
              gcmd: GCodeCommand) -> Optional[float]:
        """
        Feed a lane until a check sees its filament, then reel it back: with its unit's own
        feed when the unit has one, else with AFC's lane moves, in one homing move when the
        lane's stepper can end it at the check and a step at a time when it cannot.

        :param lane: a lane `_feeds` accepts, with filament in its unit
        :param seen: called while the lane feeds, True once its filament was seen
        :param gcmd: the command the feed runs through
        :return float: the mm AFC's moves fed the lane, None for a unit that fed it itself
        """
        feed_to = getattr(lane.unit_obj, "feed_to", None)
        if feed_to is not None:
            feed_to(lane, seen, gcmd)
            return None
        stepper = self._stepper(lane)
        self._check = seen
        try:
            if stepper is not None:
                return self._home_to(lane, stepper, gcmd)
            return self._step_to(lane, gcmd)
        finally:
            self._check = None

    def _stepper(self, lane: AFCLane) -> Optional[Any]:
        """
        The stepper a lane can be fed with in one homing move that a check ends.

        :param lane: a lane of this switch
        :return: the stepper, None while AFC does not home or for a lane that has no stepper
        that takes a polled endstop
        """
        stepper = lane.drive_stepper or lane
        if (not self.afc.homing_enabled
            or not hasattr(stepper, "add_polled_endstop")):
            return None
        return stepper

    def _checked(self) -> bool:
        """
        What ends a feed that runs with AFC's own moves: its check saw the lane's filament, or
        a watched toolhead sensor reads it. A lane that is at a toolhead has passed the switch,
        whatever the check looks for.

        :return bool: True once the feed that runs now can end, False while none runs
        """
        return (self._check is not None
                and (self._check()
                     or self._saw_toolhead()))

    def _limit(self, lane: AFCLane) -> float:
        """
        The mm a lane is fed at the most while a check looks for its filament: `_FEED_PAST`
        past the lane's own way to its toolhead.

        :param lane: the lane that is fed
        :return float: the mm
        """
        tube = self._tube(lane)
        limit = lane.dist_hub + _FEED_PAST
        if tube is not None:
            limit += tube.afc_bowden_length - self._offsets.get(tube, 0.0)
        return limit

    def _home_to(self, lane: AFCLane, stepper: Any, gcmd: GCodeCommand) -> float:
        """
        Feed a lane in one homing move that ends once `_checked` says so, then pull it back as
        far as it went. The lane's stepper is given the endstop that reads `_checked` the first
        time it is fed this way.

        :param lane: a lane with filament in its unit
        :param stepper: the stepper the lane homes with (see `_stepper`)
        :param gcmd: the command the moves run through
        :return float: the mm the lane was fed
        """
        if stepper not in self._homing:
            stepper.add_polled_endstop(_ENDSTOP, self._checked)
            self._homing.append(stepper)
        moved = 0.0
        warn = AFCMoveWarning.NONE
        lane.set_load_current(gcmd=gcmd)
        try:
            _homed, moved, warn = lane.move_to(self._limit(lane), SpeedMode.CALIBRATION,
                                               endstop=_ENDSTOP, use_homing=True, gcmd=gcmd)
        finally:
            # Also when the move failed: what is known to be fed comes back.
            if moved:
                lane.move_advanced(-moved, SpeedMode.LONG, AssistActive.YES, gcmd=gcmd)
            lane.do_enable(False)
            lane.unit_obj.return_to_home(gcmd=gcmd)
        if warn == AFCMoveWarning.ERROR:
            back = f"was pulled back the {moved:.0f} mm it went" if moved else "was not pulled back"
            error_str = (f"{self.name}: the move that feeds {lane.name} to a sensor failed, and "
                         f"the lane {back}. Run LANE_UNLOAD LANE={lane.name} if its filament is "
                         "out of its unit, then try again.")
            raise SwitchError(error_str)
        return moved

    def _step_to(self, lane: AFCLane, gcmd: GCodeCommand) -> float:
        """
        Feed a lane AFC moves by distance a step at a time until `_checked` says so, then pull
        it back as far as it was fed. The step is the one the unit's own calibration feeds by.

        :param lane: a lane with filament in its unit
        :param gcmd: the command the moves run through
        :return float: the mm the lane was fed
        """
        limit = self._limit(lane)
        step = getattr(lane.unit_obj, "calibration_step", _MEASURE_STEP)
        fed = 0.0
        lane.set_load_current(gcmd=gcmd)
        try:
            while fed < limit:
                lane.move_advanced(step, SpeedMode.CALIBRATION, AssistActive.NO, gcmd=gcmd)
                fed += step
                self.reactor.pause(self.reactor.monotonic() + _FEED_SETTLE)
                if self._checked():
                    break
        finally:
            # Also when a step failed: what was fed comes back.
            if fed:
                lane.move_advanced(-fed, SpeedMode.LONG, AssistActive.YES, gcmd=gcmd)
            lane.do_enable(False)
            lane.unit_obj.return_to_home(gcmd=gcmd)
        return fed

    def _unseen(self, lane: AFCLane, fed: Optional[float]) -> str:
        """
        Why neither inlet read a lane that was fed to the switch, as far as that can be told.

        :param lane: the lane that was fed
        :param fed: the mm AFC's moves fed it, None for a unit that fed it itself
        :return str: the error
        """
        unit = lane.unit
        if fed is None:
            return (f"{self.name}: {lane.name} was fed and neither inlet read filament. "
                    f"Put the spool of {unit} in again so its feeder takes the filament, "
                    "and check that the unit is plumbed into the switch.")
        if self._read:
            return (f"{self.name}: {lane.name} reached the toolhead sensor of "
                    f"{' and '.join(self._read)} and neither inlet read filament. Check that "
                    f"{unit} is plumbed into the switch.")
        return (f"{self.name}: {lane.name} was fed {fed:.0f} mm and neither inlet read "
                f"filament. Check that {unit} is plumbed into the switch, and the tube length "
                f"of {unit}: a lane is fed {_FEED_PAST:.0f} mm past it at the most.")

    def _identify(self, lane: AFCLane, gcmd: GCodeCommand) -> str:
        """
        Find the inlet a lane's unit is plumbed into: empty both inlets, feed the lane until an
        inlet reads its filament, reel it back, and put the unit on that inlet. An inlet that
        was found and still holds the lane afterwards is kept, and is an error.

        :param lane: a lane that holds filament and can be fed, of a unit that is found by
        loading
        :param gcmd: the command the unloads and the feed run through
        :return str: the inlet, `A` or `B`
        """
        unit = lane.unit
        if self.afc.function.in_print():
            error_str = f"{self.name}: the inlet of {unit} is not found while a print runs"
            raise SwitchError(error_str)
        with self._own():
            self._clear(gcmd)
            self._seen = {letter: False for letter in LETTERS}
            # A lane that AFC's own moves feed also stops at a toolhead (see _checked).
            self._watched = self._toolheads()
            self._read = []
            fed = self._feed(lane, self._saw, gcmd)
            saw = [letter for letter in LETTERS if self._seen[letter]]
            for letter in saw:
                self._wait_clear(letter)
            inlets = self.state()["inlets"]
            left = [letter for letter in saw if inlets[letter]["filament"]]
        error_str = ""
        if not saw:
            error_str = self._unseen(lane, fed)
        elif len(saw) > 1:
            error_str = (f"{self.name}: both inlets read filament while {lane.name} was fed, "
                         f"so the inlet of {unit} is not known. Check that nothing else is in "
                         "the switch.")
        if error_str:
            raise SwitchError(error_str)
        self._held[unit] = self._id(unit)
        self._found[self._held[unit]] = saw[0]
        if self.inlet_of(unit) != saw[0]:
            self._place(unit, saw[0])
        if unit not in self.confirmed:
            self.confirmed.append(unit)
        self.afc.save_vars()
        if left:
            error_str = (f"{self.name}: {unit} enters inlet {saw[0]}, but {lane.name} was not "
                         f"reeled back and is still in it. Run LANE_UNLOAD LANE={lane.name}, "
                         "then go on.")
            raise SwitchError(error_str)
        return saw[0]

    def _clear(self, gcmd: GCodeCommand) -> None:
        """
        Empty both inlets: unload every lane that is loaded through the switch, then check that
        neither inlet reads filament.

        :param gcmd: the command the unloads run through
        """
        for held in [held for held in self.lanes if held.tool_loaded]:
            self.logger.info(f"{self.name}: unloading {held.name} from "
                             f"{held.extruder_obj.name}, both inlets have to be empty to find "
                             "one")
            if not self._unload(held, gcmd):
                error_str = (f"{self.name}: {held.name} could not be unloaded from "
                             f"{held.extruder_obj.name}. Unload it, then try again.")
                raise SwitchError(error_str)
        for letter in LETTERS:
            self._wait_clear(letter)
        inlets = self.state()["inlets"]
        full = [letter for letter in LETTERS if inlets[letter]["filament"]]
        if full:
            error_str = (f"{self.name}: filament is still in inlet {' and '.join(full)}. Run "
                         "LANE_UNLOAD on the lane that is in it, then try again.")
            raise SwitchError(error_str)

    def _saw(self) -> bool:
        """
        Look at the switch, and say whether an inlet read filament since `_seen` was cleared.

        :return bool: True once an inlet read filament
        """
        self._look()
        return any(self._seen.values())

    cmd_AFC_BRIDGEBOX_SWITCH_SETUP_help = ("AFC_BRIDGEBOX_SWITCH_SETUP [SWITCH=<switch>] "
                                           "[OUTLETS=1] [LANE=<lane>]. Finds the inlet of each "
                                           "unit of a filament switch and the toolhead each of "
                                           "its outlets feeds")

    def cmd_AFC_BRIDGEBOX_SWITCH_SETUP(self, gcmd: GCodeCommand) -> None:
        """
        Set up a filament switch by feeding lanes through it.

        One lane of each unit whose inlet is not known is fed to the switch and reeled back, as
        AFC_BRIDGEBOX_SWITCH_IDENTIFY does. Then, on a switch its chain made that has no
        toolhead on an outlet yet, a lane is fed through each outlet until a toolhead sensor
        reads it, and reeled back: that toolhead is the one the outlet feeds. The toolheads
        have to be empty. An outlet the chain's section sets stays as it is set, and one
        through which no sensor read the lane stays as it was. A toolhead with no sensor of its
        own, as one that loads by buffer ramming has none, is not found: its outlet is set by
        hand with AFC_BRIDGEBOX_SWITCH_OUTLET. A unit whose inlet is not found does not keep
        the rest from being set up. OUTLETS=1 leaves the inlets out and finds the toolheads,
        also when they are known, and LANE names the lane that is fed through the outlets. A
        prompt says what is being done, and the calibration prompt opens again with the
        outcome.

        Usage
        -------
        `AFC_BRIDGEBOX_SWITCH_SETUP [SWITCH=<switch>] [OUTLETS=1] [LANE=<lane>]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_SWITCH_SETUP SWITCH=Chain_1
        ```
        """
        outlets = bool(gcmd.get_int("OUTLETS", 0))
        name = gcmd.get("LANE", None)
        prompt = AFCprompt(gcmd, self.logger)
        # What each step before the one that is being run gave, and why the ones that failed
        # did. A step that fails does not keep the ones after it from being run.
        done: List[str] = []
        failed: List[str] = []
        try:
            for unit in ([] if outlets else self.unfound()):
                try:
                    self._identify_units([unit], None, done, prompt, gcmd)
                except SwitchError as ex:
                    failed.append(str(ex))
                    done.append(str(ex))
            if (outlets
                or not self.extruders()):
                try:
                    self._find_toolheads(name, done, prompt, gcmd)
                except SwitchError as ex:
                    failed.append(str(ex))
                    done.append(str(ex))
        except Exception:
            # The prompt has no button, so it is not left open.
            prompt.p_end()
            raise
        if not done:
            done.append(f"{self.name}: every inlet and the toolhead of an outlet are known, so "
                        "there is nothing to set up.")
        self._open_menu(gcmd, outlets, "\n".join(done))
        if failed:
            error_str = "\n".join(failed)
            raise gcmd.error(error_str)

    def _find_toolheads(self, name: Optional[str], done: List[str], prompt: AFCprompt,
                        gcmd: GCodeCommand) -> None:
        """
        Find the toolhead each outlet feeds with one lane, with a prompt that says what is being
        done.

        :param name: the lane to feed, None for the first that can be
        :param done: what each step gave, added to with what each outlet gave
        :param prompt: the prompt that says what is being done
        :param gcmd: the command the unloads and the feeds run through
        """
        lane = self._outlet_lane(name)
        text = (f"Finding the toolhead each outlet feeds: {lane.name} is fed through each "
                "outlet until a toolhead sensor reads it, and reeled back.")
        prompt.create_custom_p(f"{self.name} Outlets", "\n".join(done + [text]), None, False,
                               None, None)
        found = self._find_outlets(lane, gcmd)
        gcmd.respond_info("\n".join(found))
        done.extend(found)

    def _to_find(self) -> List[str]:
        """
        The outlets whose toolhead can be found by feeding a lane: those of a switch its chain
        made that the chain's section does not set.

        :return list: outlet letters
        """
        if not self.chain_made:
            return []
        return [letter for letter in LETTERS if f"outlet_{letter.lower()}" not in self.by_hand]

    def _ramming(self) -> str:
        """
        Why the toolheads cannot be found by feeding a lane, when an extruder the switch's lanes
        are on loads by buffer ramming: it has no toolhead sensor to read.

        :return str: the sentence that says so and how an outlet is set by hand, "" when none
        does
        """
        names = sorted({lane.extruder_obj.name for lane in self.lanes
                        if lane.extruder_obj.tool_start == "buffer"})
        if not names:
            return ""
        return (f"{self.name}: the toolhead each outlet feeds cannot be found by feeding a "
                f"lane. With buffer ramming on {' and '.join(names)} there is no toolhead "
                f"sensor to read. {self._by_hand()}")

    def _no_sensor(self) -> str:
        """
        Why no toolhead can be found by feeding a lane: an extruder the switch's lanes are on
        loads by buffer ramming, or no extruder the switch can feed has a toolhead sensor of
        its own.

        :return str: the sentence that says so and how an outlet is set by hand, "" when a
        toolhead can be found
        """
        ramming = self._ramming()
        if ramming:
            return ramming
        if len(self._sensorless()) < len(self._reachable()):
            return ""
        return (f"{self.name}: the toolhead each outlet feeds cannot be found by feeding a "
                "lane. No toolhead the switch can feed has a sensor of its own. "
                f"{self._by_hand()}")

    def _by_hand(self) -> str:
        """
        How the toolhead of an outlet is set by hand, for a message that says to: from a prompt
        on a switch whose outlets are found, else in the section.

        :return str: the sentence
        """
        if self._to_find():
            return (f"Set by hand, or AFC_BRIDGEBOX_SWITCH_OUTLET SWITCH={self.name}, picks the "
                    "toolhead of an outlet.")
        return (f"{self._key('outlet_a')} and {self._key('outlet_b')} in [{self.section}] set "
                "the toolhead of each outlet, with a RESTART.")

    def _sensorless(self) -> List[str]:
        """
        The extruders the switch can feed that have no toolhead sensor of their own, so a lane
        is not seen arriving there.

        :return list: extruder names, in number order
        """
        return sorted((name for name, extruder in self._reachable().items()
                       if self._sensor(extruder) is None), key=in_order)

    def _sensorless_note(self) -> str:
        """
        The toolheads an outlet cannot be found at, for a message about finding the outlets.

        :return str: the sentence that names them, "" when every toolhead has a sensor
        """
        names = self._sensorless()
        if not names:
            return ""
        return (f"Toolheads with no sensor of their own: {', '.join(names)}. An outlet that "
                "feeds one of them is not found, and is set by hand.")

    def _missed(self) -> str:
        """
        What to do about an outlet whose toolhead was not found, for the end of what a find
        gave.

        :return str: the toolheads with no sensor when there are any, else what to check, and
        how an outlet is set by hand
        """
        note = self._sensorless_note()
        if not note:
            note = ("Check the tube from the outlet to its toolhead, and that the toolhead is "
                    "unloaded.")
        return f"{note} {self._by_hand()}"

    def _sensor(self, extruder: AFCExtruder) -> Optional[bool]:
        """
        What an extruder's toolhead sensor reads.

        :param extruder: an AFC extruder
        :return bool: True with filament at the sensor, None for an extruder that has no
        toolhead sensor of its own
        """
        sensor = extruder.fila_tool_start
        if (sensor is None
            or extruder.tool_start == "virtual"):
            return None
        # A motion sensor says filament is there until it has moved, its switch says it now.
        if hasattr(sensor, "runout_buttun_state"):
            return bool(sensor.runout_buttun_state)
        return bool(extruder.tool_start_state)

    def _toolheads(self) -> Dict[str, AFCExtruder]:
        """
        The extruders an outlet's toolhead can be found among: every AFC extruder with a
        toolhead sensor of its own that reads empty, but for the ones another filament switch
        feeds.

        :return dict: extruder name -> extruder
        """
        return {name: extruder for name, extruder in self._reachable().items()
                if self._sensor(extruder) is False}

    def _reachable(self) -> Dict[str, AFCExtruder]:
        """
        The extruders an outlet of this switch can feed: every AFC extruder but the ones
        another filament switch feeds.

        :return dict: extruder name -> extruder
        """
        fed = [name for _name, switch in self.printer.lookup_objects("AFC_BridgeBox_switch")
               if switch is not self
               for name in switch.extruders()]
        return {extruder.name: extruder
                for _name, extruder in self.printer.lookup_objects("AFC_extruder")
                if extruder.name not in fed}

    def _saw_toolhead(self) -> bool:
        """
        Note the watched extruders whose toolhead sensor reads filament now, and say whether
        one did since `_read` was cleared.

        :return bool: True once a watched toolhead sensor read filament
        """
        self._read += [name for name, extruder in self._watched.items()
                       if (name not in self._read
                           and self._sensor(extruder))]
        return bool(self._read)

    def _outlet_lane(self, name: Optional[str]) -> AFCLane:
        """
        The lane to feed through the outlets: the one named, else the first that holds filament
        and can be fed, of a unit whose inlet is known.

        :param name: the lane to feed, None for the first that can be
        :return AFCLane: the lane
        """
        lane = self.lane(name) if name is not None else None
        held = [held for held in self.lanes
                if (held.load_state
                    and self.inlet_of(held.unit) is not None)]
        if name is None:
            lane = next((one for one in held if self._feeds(one)), None)
        error_str = ""
        if (name is not None
            and lane is None):
            error_str = f"{self.name}: {name} is not a lane of this switch"
        elif (lane is None
              and held):
            error_str = (f"{self.name}: no lane that holds filament can be fed to a toolhead "
                         "sensor and reeled back. Their units only run whole loads. "
                         f"{self._by_hand()}")
        elif lane is None:
            error_str = (f"{self.name}: no lane of a unit whose inlet is known holds filament. "
                         "Put a spool in one first.")
        elif self.inlet_of(lane.unit) is None:
            error_str = (f"{self.name}: {lane.name} cannot find the toolheads, "
                         f"{self._need(lane.unit)}")
        elif not lane.load_state:
            error_str = f"{self.name}: {lane.name} holds no filament. Put a spool in it first."
        elif not self._feeds(lane):
            error_str = (f"{self._whole_loads(lane)} Use a lane of another unit. "
                         f"{self._by_hand()}")
        if error_str:
            raise SwitchError(error_str)
        return lane

    def _find_outlets(self, lane: AFCLane, gcmd: GCodeCommand) -> List[str]:
        """
        Find the toolhead each outlet feeds that the chain's section does not set, one outlet
        after the other with one lane. What is found is put on the outlets and saved, also when
        a later outlet fails. An outlet no toolhead sensor is left to find with is not fed.

        :param lane: a lane that holds filament, of a unit whose inlet is known
        :param gcmd: the command the unloads and the feeds run through
        :return list: what each outlet gave, one line each
        """
        inlet = self.inlet_of(lane.unit)
        letters = self._to_find()
        error_str = ""
        if self.afc.function.in_print():
            error_str = f"{self.name}: the toolheads are not found while a print runs"
        elif not letters:
            error_str = (f"{self.name}: its outlets are set in [{self.section}], so there is no "
                         "toolhead to find")
        elif self._no_sensor():
            error_str = self._no_sensor()
        if error_str:
            raise SwitchError(error_str)
        found: Dict[str, str] = {}
        lines: List[str] = []
        try:
            with self._own():
                self._clear(gcmd)
                self._check_toolheads()
                for letter in letters:
                    taken = [self.outlets[fixed] for fixed in LETTERS if fixed not in letters]
                    taken += list(found.values())
                    watch = {name: extruder for name, extruder in self._toolheads().items()
                             if name not in taken}
                    if not watch:
                        lines.append(f"{self.name}: {lane.name} was not fed through outlet "
                                     f"{letter}, which stays as it was. No toolhead sensor "
                                     "that reads empty is left to find its toolhead with.")
                        continue
                    name = self._find_outlet(lane, inlet, letter, watch, gcmd)
                    if name is None:
                        lines.append(f"{self.name}: no toolhead sensor read {lane.name} through "
                                     f"outlet {letter}, which stays as it was.")
                    else:
                        found[letter] = name
                        lines.append(f"{self.name}: outlet {letter} feeds {name}.")
                    if self.state()["inlets"][inlet]["filament"]:
                        error_str = (f"{self.name}: {lane.name} was not reeled back from outlet "
                                     f"{letter} and is still in inlet {inlet}. Run LANE_UNLOAD "
                                     f"LANE={lane.name}, then find the toolheads again.")
                        raise SwitchError(error_str)
        finally:
            self._set_outlets(found)
        if len(found) < len(letters):
            lines.append(self._missed())
        if not self.extruders():
            error_str = "\n".join(lines)
            raise SwitchError(error_str)
        return lines

    def _check_toolheads(self) -> None:
        """
        Refuse to feed a lane through the outlets while the toolhead sensor of an extruder the
        switch's lanes are on reads filament: a lane cannot be seen arriving there.
        """
        full = sorted({held.extruder_obj.name for held in self.lanes
                       if self._sensor(held.extruder_obj)})
        if full:
            error_str = (f"{self.name}: the toolhead sensor of {' and '.join(full)} reads "
                         "filament. Unload it first, a lane cannot be seen arriving there.")
            raise SwitchError(error_str)

    def _find_outlet(self, lane: AFCLane, inlet: str, outlet: str,
                     watch: Dict[str, AFCExtruder], gcmd: GCodeCommand) -> Optional[str]:
        """
        Find the toolhead one outlet feeds: point the lane's inlet at the outlet, feed the lane
        until a toolhead sensor reads it, and reel it back.

        :param lane: the lane to feed
        :param inlet: the lane's inlet
        :param outlet: the outlet to feed it through
        :param watch: the extruders whose toolhead sensor is watched, by name
        :param gcmd: the command the feed runs through
        :return str: the extruder whose toolhead sensor read the lane, None when none did
        """
        self._watched = dict(watch)
        self._read = []
        self.set_inlet(inlet, outlet)
        self._feed(lane, self._saw_toolhead, gcmd)
        self._wait_clear(inlet)
        if len(self._read) > 1:
            error_str = (f"{self.name}: the toolhead sensors of {' and '.join(self._read)} read "
                         f"filament while {lane.name} was fed through outlet {outlet}, so its "
                         "toolhead is not known.")
            raise SwitchError(error_str)
        return self._read[0] if self._read else None

    def _set_outlets(self, found: Dict[str, str]) -> None:
        """
        Put the extruders that were found on their outlets and save them. When an outlet's
        extruder changed, the tube lengths that were measured are dropped, and every unit's
        lanes go to the extruder they start on now.

        :param found: outlet letter -> the extruder found for it
        """
        was = dict(self.outlets)
        for letter, name in found.items():
            # An extruder is on one outlet.
            if self.outlets[other(letter)] == name:
                self.outlets[other(letter)] = None
            self.outlets[letter] = name
        if self.outlets == was:
            return
        keys: Dict[str, Any] = {}
        for letter in LETTERS:
            key = f"outlet_{letter.lower()}"
            if key not in self.by_hand:
                keys[key] = self.outlets[letter] or ""
            if f"{key}_length" not in self.by_hand:
                self.lengths[letter] = None
                keys[f"{key}_length"] = ""
        self._save(keys)
        self.start = self._starts()
        self.rest = dict(self.start)
        # What was kept for the lanes was for the outlets as they were.
        self._saved_lanes = {}
        self._by_print = []
        self._due_back = []
        for unit in self.units():
            if unit in self.afc.units:
                self._start_unit(self.afc.units[unit])
        self.afc.save_vars()

    cmd_AFC_BRIDGEBOX_SWITCH_OUTLET_help = ("AFC_BRIDGEBOX_SWITCH_OUTLET [SWITCH=<switch>] "
                                            "[OUTLET=<A|B>] [EXTRUDER=<extruder>]. Sets the "
                                            "toolhead an outlet of a filament switch feeds by "
                                            "hand")

    def cmd_AFC_BRIDGEBOX_SWITCH_OUTLET(self, gcmd: GCodeCommand) -> None:
        """
        Set the toolhead an outlet of the filament switch feeds by hand, for a toolhead that
        has no sensor to find it with.

        Without OUTLET a prompt offers the outlets, and with OUTLET alone a prompt offers the
        extruders the outlet can feed. With EXTRUDER that extruder is put on the outlet and
        kept with the chain, as one that was found is, and the prompt for the outlets opens
        again with the outcome. The lanes of the switch have to be out of their toolheads. The
        outlets of a switch its chain made are set this way, but for one the chain's section
        sets.

        Usage
        -------
        `AFC_BRIDGEBOX_SWITCH_OUTLET [SWITCH=<switch>] [OUTLET=<A|B>] [EXTRUDER=<extruder>]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_SWITCH_OUTLET SWITCH=Chain_1 OUTLET=B EXTRUDER=extruder1
        ```
        """
        letter = gcmd.get("OUTLET", "").upper()
        name = gcmd.get("EXTRUDER", None)
        if not self._to_find():
            error_str = (f"{self.name}: its outlets are set in [{self.section}], and are "
                         "changed there")
            raise gcmd.error(error_str)
        if not letter:
            self._outlet_menu(gcmd)
            return
        try:
            names = self._pickable(letter)
            if name is None:
                self._extruder_menu(letter, names, gcmd)
                return
            outcome = self._pick(letter, name, names)
        except SwitchError as ex:
            self._outlet_menu(gcmd, str(ex))
            raise gcmd.error(str(ex))
        gcmd.respond_info(outcome)
        self._outlet_menu(gcmd, outcome)

    def _outlet_menu(self, gcmd: GCodeCommand, outcome: str = "") -> None:
        """
        Open the prompt that offers the outlets whose toolhead can be set by hand.

        :param gcmd: the command the prompt answers
        :param outcome: what the last pick gave or why it failed, shown above the offer
        """
        command = f"AFC_BRIDGEBOX_SWITCH_OUTLET SWITCH={self.name}"
        buttons = [(f"Outlet {letter}", f"{command} OUTLET={letter}",
                    "primary" if index % 2 == 0 else "secondary")
                   for index, letter in enumerate(self._to_find())]
        feeds = [f"outlet {letter} feeds {self.outlets[letter] or 'no toolhead yet'}"
                 for letter in LETTERS]
        text = ("Sets the toolhead an outlet feeds by hand, for a toolhead that has no sensor "
                f"to find it with. Now {' and '.join(feeds)}. Pick the outlet to set.")
        if outcome:
            text = f"{outcome}\n{text}"
        back = [("Back", f"AFC_BRIDGEBOX_SWITCH_CALIBRATION SWITCH={self.name} OUTLETS=1",
                 "info")]
        AFCprompt(gcmd, self.logger).create_custom_p(f"{self.name} Outlets by hand", text,
                                                     buttons, True, None, back)

    def _extruder_menu(self, outlet: str, names: List[str], gcmd: GCodeCommand) -> None:
        """
        Open the prompt that offers the extruders an outlet can be given by hand, two to a row.

        :param outlet: the outlet picked in the prompt for the outlets
        :param names: the extruders the outlet can be given
        :param gcmd: the command the prompt answers
        """
        command = f"AFC_BRIDGEBOX_SWITCH_OUTLET SWITCH={self.name} OUTLET={outlet}"
        buttons = [(name, f"{command} EXTRUDER={name}",
                    "primary" if index % 2 == 0 else "secondary")
                   for index, name in enumerate(names)]
        rows = [buttons[first:first + 2] for first in range(0, len(buttons), 2)]
        text = f"Pick the toolhead outlet {outlet} feeds."
        if self.outlets[outlet] is not None:
            text += f" It feeds {self.outlets[outlet]} now."
        back = [("Back", f"AFC_BRIDGEBOX_SWITCH_OUTLET SWITCH={self.name}", "info")]
        AFCprompt(gcmd, self.logger).create_custom_p(f"{self.name} Outlet {outlet}", text,
                                                     None, True, rows, back)

    def _pickable(self, outlet: str) -> List[str]:
        """
        The extruders an outlet can be given by hand: every extruder the switch can feed, but
        for the one on an outlet the chain's section sets.

        :param outlet: the outlet to set, `A` or `B`
        :return list: extruder names, in number order
        """
        letters = self._to_find()
        taken = [self.outlets[fixed] for fixed in LETTERS if fixed not in letters]
        names = sorted((name for name in self._reachable() if name not in taken), key=in_order)
        error_str = ""
        if outlet not in LETTERS:
            error_str = f"{self.name}: OUTLET takes A or B"
        elif outlet not in letters:
            error_str = (f"{self.name}: outlet {outlet} is set in [{self.section}], and is "
                         "changed there")
        elif not names:
            error_str = (f"{self.name}: no extruder is left for outlet {outlet}. Another "
                         "outlet or filament switch feeds each one.")
        if error_str:
            raise SwitchError(error_str)
        return names

    def _pick(self, outlet: str, name: str, names: List[str]) -> str:
        """
        Put an extruder on an outlet by hand, and save it as one that was found is.

        :param outlet: the outlet to set
        :param name: AFC extruder name
        :param names: the extruders the outlet can be given
        :return str: what the outlet feeds now
        """
        loaded = [lane for lane in self.lanes if lane.tool_loaded]
        error_str = ""
        if self.afc.function.in_print():
            error_str = f"{self.name}: an outlet is not set while a print runs"
        elif name not in names:
            error_str = f"{self.name}: EXTRUDER takes {' or '.join(names)}"
        elif loaded:
            error_str = (f"{self.name}: {loaded[0].name} is loaded in "
                         f"{loaded[0].extruder_obj.name}. Unload it first, then set the outlet.")
        if error_str:
            raise SwitchError(error_str)
        if self.outlets[outlet] == name:
            return f"{self.name}: outlet {outlet} already feeds {name}."
        moved = self.outlets[other(outlet)] == name
        self._set_outlets({outlet: name})
        outcome = f"{self.name}: outlet {outlet} feeds {name}, set by hand."
        if moved:
            outcome += f" Outlet {other(outlet)} has no toolhead now."
        return outcome

    def forget(self) -> None:
        """
        Forget what was found for the switch by loading and saved with AFC: the inlet of each
        AMS and unit, and the units seen at their inlet.
        """
        self._found = {}
        self.confirmed = []
        self.afc.save_vars()

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        The switch for the status API.

        :param eventtime: reactor time of the query, unused
        :return dict: chain, whether the switch answers and is calibrated, each inlet's units
        in use, state and loaded lane, each outlet's extruder and tube length, each lane's
        extruder, the units not seen at their inlet yet and those whose inlet is not known
        """
        try:
            inlets = self.state()["inlets"]
        except SwitchError:
            inlets = {letter: {"outlet": None, "filament": None} for letter in LETTERS}
        loaded = {letter: self._loaded(letter) for letter in LETTERS}
        unfound = self.unfound()
        live = [unit for unit in self.live_units() if unit not in unfound]
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
                "unconfirmed": [unit for unit in live if unit not in self.confirmed],
                "unfound": unfound}


def load_config_prefix(config: ConfigWrapper) -> afcBridgeBoxSwitch:
    """
    Klipper config entry point for `[AFC_BridgeBox_switch <name>]` sections.

    :param config: the section
    :return afcBridgeBoxSwitch: the switch
    """
    return afcBridgeBoxSwitch(config)
