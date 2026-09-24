"""Unit tests for extras/AFC_BridgeBox.py."""

from __future__ import annotations

import builtins
import configparser
import os
import pathlib
import sys
import tempfile
import types
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import pytest

from extras import AFC_BambuAMS, AFC_BambuAMS_bridge as bridge_mod, AFC_BridgeBox as bb_module
from extras.AFC_BambuAMS import _BambuBufferChip, afcBambuAMS, DEFAULT_BOWDEN_MM
from extras.AFC_BambuAMS_bridge import BambuBridge, TcpPort
from extras.AFC_BridgeBox import (
    _ChildCommand,
    _hook_reset_mapping,
    _norm_model,
    _parse_map,
    _plan_lane_map,
    _SLOTS_BY_MODEL,
    _SweptSection,
    activate_from_pool,
    afcBridgeBox,
    assign_pool_tcmd,
    BridgeBoxOverrideHolder,
    deactivate_to_pool,
    lane_in_toolhead,
    load_config_prefix,
    unset_tool_loaded,
)
from extras.AFC_lane import AFCLane, AFCLaneState
from extras.AFC_spool import AFCSpool
from tests.bambu_helpers import (
    add_buffer,
    add_extruder,
    attach_bridge,
    BambuAFC,
    BambuConfig,
    BambuLogger,
    BambuPrinter,
    bridgebox_options,
    drain_var_writes,
    FakeBridge,
    FakeGcmd,
    LaneSpec,
    live_bridges,
    LogLine,
    make_afc_lane,
    make_afc_spool,
    make_bambu_unit,
    make_bridgebox,
    make_printer,
    NO_VAR_FILE,
    record_chain_state,
    RecordingGcode,
    set_var_file,
    slot_info,
    write_unit_vars,
)


class _P1OtherLane:
    """A lane of a non-Bambu unit holding T#s, as PREP leaves it."""

    def __init__(self, name: str, maps: Sequence[str],
                 config_map: Sequence[str] = ()) -> None:
        """
        :param name: the lane
        :param maps: the T#s it holds
        :param config_map: the map: its config section sets (AFC's _map)
        """
        self.name = name
        self.fullname = f"AFC_stepper {name}"
        self.map = list(maps)
        self._map = list(config_map)
        self.current_map = self.map[0] if self.map else ""
        self.runout_lane: Optional[str] = None
        #: The map at each send_lane_data, in order.
        self.sent: List[List[str]] = []

    def send_lane_data(self) -> None:
        """Record the map the panel is sent."""
        self.sent.append(list(self.map))

    def get_status(self, eventtime: Optional[float] = None,
                   save_to_file: bool = False) -> Dict[str, Any]:
        """:return dict: what AFC saves for the lane"""
        return {"name": self.name, "map": ", ".join(self.map) or "NONE",
                "current_map": self.current_map}


def _p1_other(afc: BambuAFC, name: str, maps: Sequence[str],
              config_map: Sequence[str] = ()) -> _P1OtherLane:
    """
    Register a non-Bambu lane in AFC, each of its T#s in AFC's tool table
    and registered to AFC's CHANGE_TOOL, as PREP leaves them.

    :param afc: the AFC core
    :param name: the lane
    :param maps: the T#s it holds
    :param config_map: the map: its config section sets
    :return _P1OtherLane: the lane
    """
    lane = _P1OtherLane(name, maps, config_map)
    afc.lanes[name] = lane
    for cmd in maps:
        afc.tool_cmds[cmd] = name
        afc.gcode.register_command(cmd, afc.cmd_CHANGE_TOOL)
    return lane


def _p1_user_macro(gcmd: Any = None) -> None:
    """A user's own [gcode_macro], not AFC's CHANGE_TOOL."""


SEC = "AFC_BridgeBox chain1"


A, B, C, D, E, G, H = ("AAAA", "BBBB", "CCCC", "DDDD", "EEEE", "GGGG",
                       "HHHH")


HT_UID = "0123456789ABCDEF00003331"


FOUR = f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{D}, ht:{H}"


#: The pool most bay tests start: four AMS bays and two HT bays.
POOL = {"pool_ams": 4, "pool_ht": 2}


def _bbx_boot(tmp_path: pathlib.Path, printer: Optional[BambuPrinter] = None,
              name: str = "chain1", **opts: Any) -> afcBridgeBox:
    """
    One start of a chain, on a printer of its own unless one is given.

    The chain names an external buffer (Bamb_1) unless ``buffer`` is given,
    so only the units and their lanes, hubs and sensors are fabricated.

    :param tmp_path: where the state and auto_vars files live
    :param printer: the printer; a new one when None
    :param name: the chain's name
    :return afcBridgeBox: the master
    """
    opts.setdefault("buffer", "Bamb_1")
    return make_bridgebox(tmp_path, name, printer=printer or make_printer(),
                          **opts)


def _bbx_restart(tmp_path: pathlib.Path, **opts: Any) -> afcBridgeBox:
    """
    A start from the recorded roster (no roster: option) with the
    four-AMS, two-HT pool, as a RESTART after earlier starts.

    :param tmp_path: where the state file lives
    :return afcBridgeBox: the master
    """
    merged: Dict[str, Any] = dict(POOL, roster="")
    merged.update(opts)
    return _bbx_boot(tmp_path, **merged)


def _bbx_record(tmp_path: pathlib.Path, roster: str,
                name_map: Optional[str] = None,
                lane_map: Optional[str] = None) -> None:
    """
    Leave the state file as an earlier start would: the recorded roster,
    and the maps when given.

    :param tmp_path: where the state file lives
    :param roster: the recorded roster
    :param name_map: the recorded name map
    :param lane_map: the recorded lane map
    """
    keys = {"roster": roster}
    if name_map is not None:
        keys["name_map"] = name_map
    if lane_map is not None:
        keys["lane_map"] = lane_map
    record_chain_state(tmp_path, **keys)


def _bbx_loaded(master: afcBridgeBox) -> List[str]:
    """
    :param master: a built master
    :return list: the sections its printer loaded, in load order
    """
    return [section for section, _wrapper in master.printer.loaded]


def _bbx_keys(master: afcBridgeBox, section: str) -> Dict[str, str]:
    """
    :param master: a built master
    :param section: a section it fabricated
    :return dict: the keys that section was loaded with, as written
    """
    wrapper = dict(master.printer.loaded)[section]
    return dict(wrapper.fileconfig.items(section))


def _bbx_uids(master: afcBridgeBox) -> Dict[str, str]:
    """
    :param master: a built master
    :return dict: fabricated unit name -> its unit_uid, "" for a spare
    """
    return {s.split(" ", 1)[1]: w.fileconfig.get(s, "unit_uid", fallback="")
            for s, w in master.printer.loaded
            if s.startswith("AFC_BambuAMS ")}


def _bbx_ready(master: afcBridgeBox) -> List[LogLine]:
    """
    Run the master's klippy:ready handler, asserting it wrote nothing to the
    printer's gcode.

    :param master: a built master; a scouting one needs a bridge registered
        on its port or a refusing serial module
    :return list: the log lines the handler wrote
    """
    logger = master.printer.afc.logger
    raw = list(master.printer.gcode.messages)
    before = len(logger.messages)
    master._scout_ready()
    assert master.printer.gcode.messages == raw
    return logger.messages[before:]


def _bbx_quiet(printer: BambuPrinter) -> None:
    """
    Assert nothing reached AFC's logger or the printer's gcode, as when a
    master is only built: __init__ has no logger yet.

    :param printer: the printer the masters were built on
    """
    assert printer.afc.logger.messages == []
    assert printer.gcode.messages == []


def _bbx_lanes_span(first: int, last: int) -> List[str]:
    """
    :param first: first lane number
    :param last: last lane number, inclusive
    :return list: the AFC_lane section names of that range
    """
    return [f"AFC_lane lane{n}" for n in range(first, last + 1)]


def _bbx_chain(tmp_path: pathlib.Path, printer: BambuPrinter, name: str,
               register: bool = True, **over: Any) -> afcBridgeBox:
    """
    One of two chains on a printer, sharing tmp_path's state and auto_vars
    files: chain1 enrolls ht:AAAA and any other chain ht:BBBB, each with one
    AMS bay, two HT bays and an automatic lane base.

    :param tmp_path: where the shared state and auto_vars files live
    :param printer: the printer
    :param name: the chain's name
    :param register: register it on the printer, as klippy does
    :param over: options over those defaults
    :return afcBridgeBox: the master
    """
    opts: Dict[str, Any] = dict(lane_base=0, pool_ams=1, pool_ht=2,
                                roster=f"ht:{A}" if name == "chain1"
                                else f"ht:{B}")
    opts.update(over)
    return make_bridgebox(tmp_path, name, printer=printer, register=register,
                          **opts)


def _bbx_two_chains(tmp_path: pathlib.Path, printer: BambuPrinter,
                    **chain2: Any) -> Tuple[afcBridgeBox, afcBridgeBox]:
    """
    chain1 and, below it, chain2 (unit prefix Bambu_AMS_B), past the
    declared lane1 and lane4 (see _bbx_steppers).

    :param tmp_path: where the shared state and auto_vars files live
    :param printer: the printer
    :param chain2: chain2's options over the defaults
    :return tuple: (chain1, chain2)
    """
    _bbx_steppers(printer)
    first = _bbx_chain(tmp_path, printer, "chain1")
    opts: Dict[str, Any] = dict(unit_prefix="Bambu_AMS_B")
    opts.update(chain2)
    return first, _bbx_chain(tmp_path, printer, "chain2", **opts)


def _bbx_steppers(printer: BambuPrinter) -> BambuPrinter:
    """
    Declare [AFC_stepper lane1] and [AFC_stepper lane4] in the printer's
    merged config, so an automatic lane base starts at 5.

    :param printer: the printer
    :return BambuPrinter: the printer
    """
    for section in ("AFC_stepper lane1", "AFC_stepper lane4"):
        printer.add_section(section, {})
    return printer


def _bbx_unlistable(printer: BambuPrinter) -> BambuPrinter:
    """
    Make the printer's object registry refuse to be listed.

    :param printer: the printer
    :return BambuPrinter: the printer
    """
    def _refuse(module: Optional[str] = None) -> Any:
        raise RuntimeError("no registry")

    printer.lookup_objects = _refuse
    return printer


class _BbxUnlistedConfig(BambuConfig):
    """BambuConfig that cannot list a section's options."""

    def get_prefix_options(self, prefix: str) -> List[str]:
        """
        :param prefix: the option prefix
        :raises configparser.Error: always
        """
        error_str = f"cannot list the options of [{self.get_name()}]"
        raise configparser.Error(error_str)


#: Four AMS and an HT recorded, as most bay scenarios start.
FOUR_p3 = f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{D}, ht:{H}"


#: The chain in index order: the HT answers at index 6.
CHAIN = (A, B, C, D, E, G, H)


END = ("respond_raw", "// action:prompt_end")


NAMES = {"ams_names": "Alpha, Bravo, Charlie", "ht_names": "Hot"}


#: Alpha's lane records as AFC.var.unit holds them after an upgrade.
UPGRADE_VAR = {"Alpha": {
    "lane24": {"map": "T24", "current_map": "T24", "spool_id": 159,
               "material": "PLA", "color": "#0086D6", "weight": 412.0},
    "lane25": {"map": "T25", "current_map": "T25", "material": "PETG"}}}


def _p3_chain(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
              recorded: Optional[str] = None,
              state: Optional[Dict[str, str]] = None,
              var: Optional[Any] = None,
              online: Optional[Iterable[str]] = None,
              uids: Sequence[str] = CHAIN, htmask: int = 0,
              print_state: Optional[str] = None,
              owners: Optional[str] = None,
              sections: Optional[Dict[str, Dict[str, str]]] = None,
              ready: bool = False, **options: Any) -> afcBridgeBox:
    """
    One start of chain1 on a printer of its own whose fabricated units and
    lanes are real.

    :param tmp_path: where the state, auto_vars and var files live
    :param monkeypatch: isolates the bridge table and the module clocks
    :param recorded: the roster an earlier start recorded; the roster:
        option is then unset unless ``options`` gives one
    :param state: further state keys an earlier start left
    :param var: what AFC.var.unit holds (raw text for an unreadable file)
    :param online: the uids online on the chain; no bridge when None
    :param uids: the chain, in index order
    :param htmask: the chain indexes that are HTs
    :param print_state: print_stats state
    :param owners: the bay_owner key, set once the master is built
    :param sections: further config sections (overrides, other masters)
    :param ready: run klippy:ready's handler
    :param options: the master's options
    :return afcBridgeBox: the master; its logger is AFC's
    """
    printer = make_printer(monkeypatch=monkeypatch, fabricate=True,
                           print_state=print_state)
    for section, values in (sections or {}).items():
        printer.add_section(section, values)
    keys = dict(state or {})
    if recorded is not None:
        keys["roster"] = recorded
        options.setdefault("roster", "")
    if keys:
        _p3_seed(tmp_path, {SEC: keys})
    options.setdefault("pool_ams", 0)
    options.setdefault("pool_ht", 0)
    master = make_bridgebox(tmp_path, printer=printer, **options)
    printer.connect()
    if owners is not None:
        master._state_set({SEC: {"bay_owner": owners}})
    if var is not None:
        write_unit_vars(printer, var)
    if online is not None:
        _p3_wire(master, online, uids, htmask)
    if ready:
        master._scout_ready()
    return master


def _p3_named(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
              recorded: Optional[str] = f"boxed:{A}", ready: bool = True,
              owners: Optional[str] = None, **options: Any) -> afcBridgeBox:
    """
    AAAA recorded, three AMS bays named Alpha, Bravo and Charlie and an HT
    bay named Hot, started through klippy:ready, on a chain where no unit
    answers unless ``options`` puts some ``online``.

    :param recorded: the recorded roster; None leaves the state as it is
    :param ready: run klippy:ready's handler
    :param owners: the bay_owner key an earlier start left
    :return afcBridgeBox: the master
    """
    for key, value in dict(pool_ams=3, pool_ht=1, online=(),
                           **NAMES).items():
        options.setdefault(key, value)
    return _p3_chain(tmp_path, monkeypatch, recorded=recorded,
                     owners=owners, ready=ready, **options)


def _p3_wire(master: afcBridgeBox, online: Iterable[str],
             uids: Sequence[str] = CHAIN, htmask: int = 0) -> FakeBridge:
    """
    Register the chain's bridge.

    :param master: the chain master
    :param online: the uids online
    :param uids: the chain, in index order
    :param htmask: the chain indexes that are HTs
    :return FakeBridge: the bridge
    """
    on = set(online)
    bridge = FakeBridge(uids=list(uids), online=[u in on for u in uids],
                        htmask=htmask)
    live_bridges()[master.serial_port] = bridge
    return bridge


def _p3_rec(maps: str = "T24", **fields: Any) -> Dict[str, Any]:
    """:return dict: a saved lane record with this map"""
    return dict({"map": maps, "current_map": maps.split(",")[0].strip()},
                **fields)


def _p3_lane(master: afcBridgeBox, name: str) -> Any:
    """:return AFCLane: the fabricated lane of that name"""
    return master.printer.lookup_object(f"AFC_lane {name}")


def _p3_stuck(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
              online: Iterable[str] = (A, B, C, E, H), recorded: str = FOUR_p3,
              **options: Any) -> afcBridgeBox:
    """
    Four AMS and an HT recorded with four AMS bays and two HT bays; every
    unit is plugged in but D, and E is new.

    :param online: the uids online
    :param recorded: the recorded roster
    :return afcBridgeBox: the master, watching the chain
    """
    options.setdefault("pool_ams", 4)
    options.setdefault("pool_ht", 2)
    return _p3_chain(tmp_path, monkeypatch, recorded=recorded, online=online,
                     htmask=1 << 6, **options)


def _p3_claimed(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
                online: bool = False, **options: Any) -> afcBridgeBox:
    """
    AAAA, the roster: option's one unit, claimed onto the one AMS bay, with
    auto-drop on (10s grace, 5s settle).

    :param online: whether AAAA answers on the chain
    :return afcBridgeBox: the master
    """
    for key, value in dict(auto_drop=True, release_grace=10.0,
                           release_settle=5.0, pool_ams=1,
                           pool_ht=0).items():
        options.setdefault(key, value)
    master = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}",
                       online=(A,) if online else (), uids=(A,), **options)
    master._claim_pool_unit(A, "boxed")
    return master


def _p3_load(master: afcBridgeBox, name: str) -> Any:
    """
    Record a lane as loaded to the toolhead, both halves as set_tool_loaded
    leaves them.

    :param master: the chain master
    :param name: the lane
    :return AFCLane: the lane
    """
    lane = _p3_lane(master, name)
    lane.tool_loaded = True
    lane.status = AFCLaneState.TOOLED
    master.printer.afc.tools["extruder"].lane_loaded = name
    return lane


def _p3_refused(master: afcBridgeBox, tmp_path: pathlib.Path, command: str,
                cmd: FakeGcmd, log: Sequence[LogLine] = ()) -> str:
    """
    Run a bay command that must be refused, and check it changed nothing:
    not the state file, not a bay, and it said nothing but the refusal.

    :param master: the chain master
    :param tmp_path: where the state file lives
    :param command: FORGET, ASSIGN, UNASSIGN or REPLACE
    :param cmd: the command
    :param log: what the refusal logs
    :return str: the refusal
    """
    before = (_p3_state(tmp_path), [dict(pu) for pu in master._pool_units])
    _p3_quiet(master)
    with pytest.raises(Exception) as err:
        getattr(master, f"cmd_AFC_BRIDGEBOX_{command}")(cmd)
    assert (_p3_state(tmp_path),
            [dict(pu) for pu in master._pool_units]) == before
    assert _p3_log(master) == list(log)
    assert _p3_console(master) == []
    assert cmd.messages == []
    return str(err.value)


def _p3_upgrade(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                **options: Any) -> afcBridgeBox:
    """
    The first start after an upgrade: an earlier start pinned AAAA to
    Alpha, the var file holds Alpha's records (:data:`UPGRADE_VAR`), and
    the state has no bay_owner key.

    :return afcBridgeBox: the master, through klippy:ready
    """
    _p3_named(tmp_path, monkeypatch, ready=False)
    return _p3_named(tmp_path, monkeypatch, var=UPGRADE_VAR, **options)


def _p3_bound(tmp_path: pathlib.Path,
              monkeypatch: pytest.MonkeyPatch) -> afcBridgeBox:
    """
    :return afcBridgeBox: the stuck chain with DDDD plugged in and claimed
        onto Bambu_AMS_4, its lane37 recorded as loaded to the toolhead
    """
    master = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, D, H))
    _p3_tick(master, 100, 101)
    _p3_lane(master, "lane37").tool_loaded = True
    return master


def _p3_loaded(tmp_path: pathlib.Path,
               monkeypatch: pytest.MonkeyPatch) -> afcBridgeBox:
    """
    :return afcBridgeBox: the stuck chain watched past release_grace, with
        AFC recording lane36 of DDDD's unclaimed bay in the toolhead
    """
    master = _p3_stuck(tmp_path, monkeypatch)
    _p3_tick(master, 100, 111)
    master.printer.afc.tools["extruder"].lane_loaded = "lane36"
    return master


def _p3_bridge(master: afcBridgeBox) -> FakeBridge:
    """:return FakeBridge: the bridge registered for the master's port"""
    return live_bridges()[master.serial_port]


def _p3_online(master: afcBridgeBox, online: Iterable[str]) -> None:
    """
    Set who answers on the chain from the next status on.

    :param master: the chain master
    :param online: the uids online
    """
    bridge = _p3_bridge(master)
    on = set(online)
    bridge.status = {"units": [{"n": i, "online": u in on}
                               for i, u in enumerate(bridge.uids)]}


def _p3_tick(master: afcBridgeBox, first: int, last: int) -> None:
    """
    Run the chain watch once a second, with the printer's clock there too.

    :param master: the chain master
    :param first: the first tick's time
    :param last: the last tick's time, inclusive
    """
    for t in range(first, last + 1):
        master.printer.reactor.now = float(t)
        master._scout_tick(float(t))


def _p3_bay(master: afcBridgeBox, name: str) -> Dict[str, Any]:
    """:return dict: the pool bay of that name"""
    return next(pu for pu in master._pool_units if pu["name"] == name)


def _p3_cmd(**params: Any) -> FakeGcmd:
    """:return FakeGcmd: a command with these parameters"""
    return FakeGcmd(params)


def _p3_quiet(master: afcBridgeBox) -> None:
    """
    Forget what the setup logged, so a test sees what its call logs.

    :param master: the chain master
    """
    master.logger.messages.clear()
    master.printer.gcode.messages.clear()


def _p3_log(master: afcBridgeBox) -> List[LogLine]:
    """:return list: what the master and its units logged"""
    return master.logger.messages


def _p3_console(master: afcBridgeBox) -> List[LogLine]:
    """:return list: what reached the printer's console (the popups)"""
    return master.printer.gcode.messages


def _p3_state(tmp_path: pathlib.Path) -> bytes:
    """:return bytes: the state file as it stands"""
    return (tmp_path / "AFC_BridgeBox.cfg").read_bytes()


def _p3_seed(tmp_path: pathlib.Path,
             updates: Dict[str, Dict[str, str]]) -> None:
    """
    Leave sections in the state file as an earlier start would, written by
    the real ``_state_set`` of a master that builds nothing.

    That master starts on a scratch file and is pointed at the state file
    only once built, so its own start rewrites nothing an earlier start
    recorded there.

    :param tmp_path: where the state file lives
    :param updates: section -> keys
    """
    with tempfile.TemporaryDirectory() as scratch:
        master = make_bridgebox(
            pathlib.Path(scratch), printer=make_printer(), register=False,
            roster="", pool_ams=0, pool_ht=0)
        master.state_file = str(tmp_path / "AFC_BridgeBox.cfg")
        master._state_set(updates)


def _p3_popup(title: str, lines: Sequence[str],
              buttons: Sequence[str] = ()) -> List[LogLine]:
    """
    The console lines of one action:prompt dialog.

    :param title: its title
    :param lines: its text lines
    :param buttons: its buttons, each as ``label|command|style``
    :return list: begin, the lines, the buttons, the Dismiss footer, show
    """
    raw = ([f"// action:prompt_begin {title}"]
           + [f"// action:prompt_text {line}" for line in lines]
           + [f"// action:prompt_button {button}" for button in buttons]
           + ["// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
              "MSG=action:prompt_end|info", "// action:prompt_show"])
    return [("respond_raw", line) for line in raw]


def _p3_unit_uids(master: afcBridgeBox) -> Dict[str, str]:
    """:return dict: fabricated unit name -> its unit_uid, "" for a spare"""
    return {s.split(" ", 1)[1]: w.fileconfig.get(s, "unit_uid", fallback="")
            for s, w in master.printer.loaded
            if s.startswith("AFC_BambuAMS ")}


class _P3BareConfig(BambuConfig):
    """
    A section wrapper that shows no merged config and cannot list its
    options, which the master guards against.
    """

    def __init__(self, name: str, printer: Any,
                 values: Optional[Dict[str, Any]] = None) -> None:
        """
        :param name: the section name
        :param printer: the printer
        :param values: its options
        """
        super().__init__(name, printer, values)
        self.fileconfig = None

    def get_prefix_options(self, prefix: str) -> List[str]:
        """:raises AttributeError: it cannot list its options"""
        raise AttributeError("get_prefix_options")


class _P3OtherLane:
    """A non-Bambu lane holding T#s, as PREP leaves it."""

    def __init__(self, name: str, maps: List[str]) -> None:
        """
        :param name: the lane
        :param maps: the T#s it holds
        """
        self.name = name
        self.fullname = f"AFC_stepper {name}"
        self.map = list(maps)
        self._map: List[str] = []
        self.current_map = maps[0]
        self.runout_lane = None
        self.sent: List[List[str]] = []

    def send_lane_data(self) -> None:
        """Record the map sent."""
        self.sent.append(list(self.map))

    def get_status(self, eventtime: Optional[float] = None,
                   save_to_file: bool = False) -> Dict[str, Any]:
        """:return dict: what AFC saves for the lane"""
        return {"name": self.name, "map": ", ".join(self.map),
                "current_map": self.current_map}


class _P3BareUnit:
    """
    A pool unit object with only what a claim needs: no set_master,
    hold_lanes, apply_learned or release.
    """

    pool = True
    type = "AFC_BambuAMS"

    def __init__(self, name: str) -> None:
        """:param name: its bay"""
        self.name = name
        self.lanes: Dict[str, Any] = {}
        self.claims: List[tuple] = []

    def claim(self, uid: str, model: str) -> bool:
        """
        :return bool: True; the claim is recorded in ``claims``
        """
        self.claims.append((uid, model))
        return True


class _P3BareLane:
    """
    A pool lane with no hub, extruder or buffer wired, no config kept, no
    Moonraker push and no way back from a toolhead, whose status cannot be
    read.
    """

    def __init__(self, name: str, unit: _P3BareUnit, afc: Any) -> None:
        """
        :param name: the lane
        :param unit: its unit
        :param afc: the AFC core
        """
        self.name = name
        self.fullname = f"AFC_lane {name}"
        self.unit_obj, self.afc = unit, afc
        self.hub_obj = self.extruder_obj = self.buffer_obj = None
        self.buffer_name = None
        self.unassigned = True
        self.tool_loaded = False
        self.loaded_to_hub = False
        self.map: List[str] = []
        self._map: List[str] = []
        self.current_map = ""

    def get_status(self, eventtime: Optional[float] = None,
                   save_to_file: bool = False) -> Dict[str, Any]:
        """:raises RuntimeError: its status cannot be read"""
        raise RuntimeError("no status")


def _p3_bare_bay(master: afcBridgeBox, monkeypatch: pytest.MonkeyPatch,
                 bay: str) -> List[_P3BareLane]:
    """
    Put a :class:`_P3BareUnit` on a bay, its first lane never built and the
    others :class:`_P3BareLane`.

    :param master: the chain master
    :param bay: the bay
    :return list: the bare lanes
    """
    objects = master.printer._objects
    unit = _P3BareUnit(bay)
    monkeypatch.setitem(objects, f"AFC_BambuAMS {bay}", unit)
    first, *rest = _p3_bay(master, bay)["lanes"]
    monkeypatch.delitem(objects, f"AFC_lane {first}")
    lanes = [_P3BareLane(name, unit, master.printer.afc) for name in rest]
    for lane in lanes:
        monkeypatch.setitem(objects, f"AFC_lane {lane.name}", lane)
    return lanes


def _p4_quiet(master: afcBridgeBox) -> None:
    """
    Forget what the setup logged, so a test sees what its calls log.

    :param master: the chain master
    """
    master.logger.messages.clear()
    master.printer.gcode.messages.clear()


HT_ROSTER = f"ht:{HT_UID}"


A_p5, B_p5, C_p5, D_p5, E_p5, G_p5, H_p5 = ("AAAA", "BBBB", "CCCC", "DDDD", "EEEE", "GGGG", "HHHH")


Z = "ZZZZ"


A24, B24, C24 = "A" * 24, "B" * 24, "C" * 24


#: Four AMS and an HT recorded, as the stuck-chain scenarios start.
FOUR_p6 = f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{D}, ht:{H}"


def _p6_chain(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
              recorded: Optional[str] = None,
              state: Optional[Dict[str, str]] = None,
              var: Optional[Any] = None,
              owners: Optional[str] = None,
              online: Optional[Iterable[str]] = (),
              uids: Sequence[str] = CHAIN, htmask: int = 0,
              print_state: Optional[str] = None,
              sections: Optional[Dict[str, Dict[str, str]]] = None,
              ready: bool = True, **options: Any) -> afcBridgeBox:
    """
    One start of chain1 on a printer of its own whose fabricated units and
    lanes are real, with the chain's bridge registered.

    :param tmp_path: where the state, auto_vars and var files live
    :param monkeypatch: isolates the bridge table and the module clocks
    :param recorded: the roster an earlier start recorded; the roster:
        option is then unset unless ``options`` gives one
    :param state: further state keys an earlier start left
    :param var: what AFC.var.unit holds
    :param owners: the bay_owner key, set once the master is built
    :param online: the uids online on the chain; no bridge when None
    :param uids: the chain, in index order
    :param htmask: the chain indexes that are HTs
    :param print_state: print_stats state
    :param sections: further config sections (operator overrides)
    :param ready: run klippy:ready's handler
    :param options: the master's options
    :return afcBridgeBox: the master; its logger is AFC's
    """
    printer = make_printer(monkeypatch=monkeypatch, fabricate=True,
                           print_state=print_state)
    for section, values in (sections or {}).items():
        printer.add_section(section, values)
    keys = dict(state or {})
    if recorded is not None:
        keys["roster"] = recorded
        options.setdefault("roster", "")
    if keys:
        record_chain_state(tmp_path, **keys)
    options.setdefault("pool_ams", 0)
    options.setdefault("pool_ht", 0)
    master = make_bridgebox(tmp_path, printer=printer, **options)
    printer.connect()
    if owners is not None:
        master._state_set({SEC: {"bay_owner": owners}})
    if var is not None:
        write_unit_vars(printer, var)
    if online is not None:
        _p6_wire(master, online, uids, htmask)
    if ready:
        master._scout_ready()
    return master


def _p6_named(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
              recorded: Optional[str] = f"boxed:{A}",
              **options: Any) -> afcBridgeBox:
    """
    AAAA recorded, three AMS bays named Alpha, Bravo and Charlie and an HT
    bay named Hot, started through klippy:ready on a chain where no unit
    answers.

    :param recorded: the recorded roster
    :param options: further :func:`_p6_chain` arguments
    :return afcBridgeBox: the master
    """
    for key, value in dict(pool_ams=3, pool_ht=1, **NAMES).items():
        options.setdefault(key, value)
    return _p6_chain(tmp_path, monkeypatch, recorded=recorded, **options)


def _p6_stuck(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
              online: Iterable[str] = (A, B, C, E, H), recorded: str = FOUR_p6,
              **options: Any) -> afcBridgeBox:
    """
    Four AMS and an HT recorded with four AMS bays and two HT bays; every
    unit is plugged in but D, and E is new.

    :param online: the uids online
    :param recorded: the recorded roster
    :return afcBridgeBox: the master, watching the chain
    """
    options.setdefault("pool_ams", 4)
    options.setdefault("pool_ht", 2)
    return _p6_chain(tmp_path, monkeypatch, recorded=recorded, online=online,
                     htmask=1 << 6, ready=False, **options)


def _p6_wire(master: afcBridgeBox, online: Iterable[str],
             uids: Sequence[str] = CHAIN, htmask: int = 0) -> FakeBridge:
    """
    Register the chain's bridge.

    :param master: the chain master
    :param online: the uids online
    :param uids: the chain, in index order
    :param htmask: the chain indexes that are HTs
    :return FakeBridge: the bridge
    """
    on = set(online)
    bridge = FakeBridge(uids=list(uids), online=[u in on for u in uids],
                        htmask=htmask)
    live_bridges()[master.serial_port] = bridge
    return bridge


def _p6_tick(master: afcBridgeBox, first: int, last: int) -> None:
    """
    Run the chain watch once a second, with the printer's clock there too.

    :param master: the chain master
    :param first: the first tick's time
    :param last: the last tick's time, inclusive
    """
    for t in range(first, last + 1):
        master.printer.reactor.now = float(t)
        master._scout_tick(float(t))


def _p6_rec(maps: str = "T24", current: Optional[str] = None,
            **fields: Any) -> Dict[str, Any]:
    """
    :param maps: the saved map
    :param current: the saved current_map; the map's first T# when None
    :return dict: a saved lane record
    """
    cur = maps.split(",")[0].strip() if current is None else current
    return dict({"map": maps, "current_map": cur}, **fields)


def _p6_lane(master: afcBridgeBox, name: str) -> Any:
    """:return AFCLane: the fabricated lane of that name"""
    return master.printer.lookup_object(f"AFC_lane {name}")


def _p6_unit(master: afcBridgeBox, name: str) -> Any:
    """:return afcBambuAMS: the fabricated unit of that name"""
    return master.printer.lookup_object(f"AFC_BambuAMS {name}")


def _p6_bay(master: afcBridgeBox, name: str) -> Dict[str, Any]:
    """:return dict: the pool bay of that name"""
    return next(pu for pu in master._pool_units if pu["name"] == name)


def _p6_quiet(master: afcBridgeBox) -> None:
    """
    Forget what the setup logged, so a test sees what its call logs.

    :param master: the chain master
    """
    master.logger.messages.clear()
    master.printer.gcode.messages.clear()


def _p6_log(master: afcBridgeBox) -> List[LogLine]:
    """:return list: what the master and its units logged"""
    return master.logger.messages


def _p6_console(master: afcBridgeBox) -> List[LogLine]:
    """:return list: what reached the printer's console (the popups)"""
    return master.printer.gcode.messages


def _p6_popup(title: str, lines: Sequence[str],
              buttons: Sequence[str] = ()) -> List[LogLine]:
    """
    The console lines of one action:prompt dialog.

    :param title: its title
    :param lines: its text lines
    :param buttons: its buttons, each as ``label|command|style``
    :return list: begin, the lines, the buttons, the Dismiss footer, show
    """
    raw = ([f"// action:prompt_begin {title}"]
           + [f"// action:prompt_text {line}" for line in lines]
           + [f"// action:prompt_button {button}" for button in buttons]
           + ["// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
              "MSG=action:prompt_end|info", "// action:prompt_show"])
    return [("respond_raw", line) for line in raw]


def _p6_claim_log(uid: str, bay: str, model: str = "boxed",
                  lanes: int = 4, tail: str = "") -> List[LogLine]:
    """
    :param uid: the claimed uid
    :param bay: the bay it claims
    :param model: the model it claims as
    :param lanes: the lanes the bay has
    :param tail: what the master's line ends with past "no restart."
    :return list: what a claim that maps every lane quietly logs: the
        unit's two lines, then the master's
    """
    return [
        ("debug",
         f"AFC bambu {bay}: chain index not resolved yet (UID {uid}); "
         "holding this unit's registrations until the chain map arrives"),
        ("info",
         f"AFC bambu {bay}: claimed UID {uid} as {model} and brought online "
         "live (ams_index=0)."),
        ("info",
         f"AFC_BridgeBox chain1: CLAIMED {uid} as {model} onto {bay} "
         f"({lanes} lanes) -- live, no restart.{tail}"),
    ]


class _P6OtherLane:
    """A non-Bambu lane holding T#s, registered the way PREP leaves it."""

    def __init__(self, name: str, maps: Sequence[str],
                 config_map: Sequence[str] = ()) -> None:
        """
        :param name: the lane
        :param maps: the T#s it holds
        :param config_map: the map: its config section sets
        """
        self.name = name
        self.fullname = f"AFC_stepper {name}"
        self.map = list(maps)
        self._map = list(config_map)
        self.current_map = maps[0] if maps else ""
        self.sent: List[List[str]] = []

    def send_lane_data(self) -> None:
        """Record the map the panel is sent."""
        self.sent.append(list(self.map))


def _p6_other(master: afcBridgeBox, name: str, maps: Sequence[str],
              config_map: Sequence[str] = ()) -> _P6OtherLane:
    """
    Register a non-Bambu lane holding ``maps`` in AFC, each T# registered
    to AFC's CHANGE_TOOL.

    :param master: the chain master
    :param name: the lane
    :param maps: the T#s it holds
    :param config_map: the map: its config section sets
    :return _P6OtherLane: the lane
    """
    afc = master.printer.afc
    lane = _P6OtherLane(name, maps, config_map)
    afc.lanes[name] = lane
    for cmd in maps:
        afc.tool_cmds[cmd] = name
        afc.gcode.register_command(cmd, afc.cmd_CHANGE_TOOL)
    return lane


def _p6_consistent(master: afcBridgeBox) -> None:
    """
    Each T# is on at most one live lane's map, AFC's tool table names that
    lane, and the command is registered; the table names nothing else.

    :param master: the chain master
    """
    afc = master.printer.afc
    owner: Dict[str, str] = {}
    for lane in afc.lanes.values():
        for cmd in lane.map or []:
            if cmd == "NONE":
                continue
            assert cmd not in owner, f"{cmd} on {owner[cmd]} and {lane.name}"
            owner[cmd] = lane.name
            assert afc.tool_cmds.get(cmd) == lane.name, cmd
            assert afc.gcode.ready_gcode_handlers.get(cmd) is not None, cmd
    assert afc.tool_cmds == owner


class TestNormModel:
    """A model tag as written: trimmed and lower-cased, never coerced."""

    def test_model_names_are_case_and_space_insensitive(self):
        assert _norm_model("  HT ") == "ht"
        assert _norm_model("AMS2") == "ams2"
        assert _norm_model(None) == ""

    def test_unknown_names_are_not_coerced_to_a_model(self):
        # A typo must stay unknown, so it becomes an error, not a wrong unit.
        names = [_norm_model(n) for n in (" AMSHT", "ams2Pro ", "Ams")]
        assert names == ["amsht", "ams2pro", "ams"]
        assert [n for n in names if n in _SLOTS_BY_MODEL] == []


class TestActivateFromPool:
    """
    A claimed pool lane joins the registries handle_connect kept it out of,
    publishes its buffer's name and takes PREP's mark.
    """

    @staticmethod
    def _pool_lane(*, buffer: bool = True,
                   lane_values: Optional[Dict[str, Any]] = None
                   ) -> Tuple[BambuPrinter, Any, Any]:
        """
        lane28, a pool lane (``unassigned: True``) on Bambu_AMS_1, connected
        as klippy connects it.

        :param buffer: the unit names Bambu_AMS_Buffer as its buffer
        :param lane_values: further lane options
        :return tuple: the printer, the unit and the lane
        """
        printer = make_printer()
        values: Dict[str, Any] = {}
        if buffer:
            add_buffer(printer, "Bambu_AMS_Buffer")
            values["buffer"] = "Bambu_AMS_Buffer"
        unit = make_bambu_unit(
            "Bambu_AMS_1", printer=printer, values=values,
            lanes=[LaneSpec("lane28", 0,
                            values=dict({"unassigned": True},
                                        **(lane_values or {})))])
        return printer, unit, printer.lookup_object("AFC_lane lane28")

    @staticmethod
    def _quiet(printer: BambuPrinter) -> None:
        """Nothing reaches AFC's log or the console."""
        assert printer.afc.logger.messages == []
        assert printer.gcode.messages == []

    def test_a_claimed_lane_publishes_its_units_buffer_name(self):
        printer, unit, lane = self._pool_lane()
        buf, hub, ext = lane.buffer_obj, unit.hub_obj, lane.extruder_obj
        assert lane.buffer_name is None
        assert (unit.lanes, printer.afc.lanes, hub.lanes, ext.lanes,
                buf.lanes) == ({}, {}, {}, {}, {})
        activate_from_pool(lane)
        assert lane.buffer_name == "Bambu_AMS_Buffer"
        for registry in (unit.lanes, printer.afc.lanes, hub.lanes, ext.lanes,
                         buf.lanes):
            assert registry == {"lane28": lane}
        assert ext.check_lanes_calls == 1
        assert lane.unassigned is False
        self._quiet(printer)

    def test_a_lane_with_its_own_buffer_name_keeps_it(self):
        # An explicit per-lane buffer: overrides the unit's, and the
        # back-fill only fills a gap. The name differs from the object's, so
        # a back-fill would show.
        printer, _unit, lane = self._pool_lane()
        lane.buffer_name = "MyOwnBuffer"
        activate_from_pool(lane)
        assert lane.buffer_name == "MyOwnBuffer"
        assert lane.buffer_obj.lanes == {"lane28": lane}
        self._quiet(printer)

    def test_a_lane_with_no_buffer_at_all_is_left_alone(self):
        # Valid configuration, as AFC_lane says.
        printer, unit, lane = self._pool_lane(buffer=False)
        activate_from_pool(lane)
        assert (lane.buffer_obj, lane.buffer_name) == (None, None)
        assert lane.unassigned is False
        assert unit.lanes == {"lane28": lane}
        self._quiet(printer)

    def test_the_mirror_is_complete(self):
        # A claimed pool lane ends registered where handle_connect registers
        # a lane that was never pooled: lane1, on the same unit, hub,
        # extruder and buffer.
        printer = make_printer()
        add_buffer(printer, "Bambu_AMS_Buffer")
        unit = make_bambu_unit(
            "Bambu_AMS_1", printer=printer,
            values={"buffer": "Bambu_AMS_Buffer"},
            lanes=["lane1", LaneSpec("lane28", 1,
                                     values={"unassigned": True})])
        lane1 = printer.lookup_object("AFC_lane lane1")
        lane28 = printer.lookup_object("AFC_lane lane28")
        activate_from_pool(lane28)

        def where(lane: Any) -> Tuple[bool, ...]:
            """:return tuple: in which registry the lane is, and as itself"""
            return tuple(reg.get(lane.name) is lane for reg in (
                unit.lanes, printer.afc.lanes, unit.hub_obj.lanes,
                lane.extruder_obj.lanes, lane.buffer_obj.lanes))

        assert where(lane1) == where(lane28) == (True,) * 5
        assert lane28.buffer_name == lane1.buffer_name == "Bambu_AMS_Buffer"
        assert lane28.unassigned is lane1.unassigned is False
        # handle_connect checked the extruder's lanes once, the claim again.
        assert lane1.extruder_obj.check_lanes_calls == 2
        self._quiet(printer)

    def test_a_claim_after_prep_marks_the_lane_prep_done(self):
        printer, _unit, lane = self._pool_lane()
        assert (printer.afc.prep_done, lane._afc_prep_done) == (True, False)
        activate_from_pool(lane)
        assert lane.unassigned is False
        assert lane._afc_prep_done is True
        self._quiet(printer)

    def test_a_claim_before_prep_leaves_the_mark_to_prep(self):
        printer, _unit, lane = self._pool_lane()
        printer.afc.prep_done = False
        activate_from_pool(lane)
        assert lane.unassigned is False
        assert lane._afc_prep_done is False
        self._quiet(printer)

    def test_activation_brings_no_spool_back(self):
        printer = make_printer()
        make_bambu_unit("Bambu_AMS_1", printer=printer, lanes=[
            LaneSpec("lane28", 0, load=True, material="PLA",
                     color="#0086D6", spool_id=136, weight=750.0)])
        lane = printer.lookup_object("AFC_lane lane28")
        deactivate_to_pool(lane)
        activate_from_pool(lane)
        assert (lane.spool_id, lane.color, lane.weight, lane.material) == (
            None, "", 0., None)
        assert printer.afc.lanes == {"lane28": lane}
        self._quiet(printer)

    def test_a_direct_hub_takes_no_lane(self):
        # The lane keeps its unit's hub object, so only the direct-hub check
        # keeps it out of that hub's lanes.
        printer, unit, lane = self._pool_lane()
        lane.hub = "direct"
        assert lane.is_direct_hub() is True
        assert lane.hub_obj is unit.hub_obj
        activate_from_pool(lane)
        assert unit.hub_obj.lanes == {}
        assert (unit.lanes, lane.extruder_obj.lanes) == ({"lane28": lane},
                                                         {"lane28": lane})
        self._quiet(printer)

    def test_a_lane_without_an_extruder_is_registered_everywhere_else(self):
        # handle_connect refuses a lane with no extruder, so this guard only
        # meets a lane whose extruder was dropped after connect.
        printer, unit, lane = self._pool_lane()
        ext = lane.extruder_obj
        lane.extruder_obj = None
        activate_from_pool(lane)
        assert (ext.lanes, ext.check_lanes_calls) == ({}, 0)
        assert (unit.lanes, unit.hub_obj.lanes, lane.buffer_obj.lanes) == (
            {"lane28": lane}, {"lane28": lane}, {"lane28": lane})
        assert lane.buffer_name == "Bambu_AMS_Buffer"
        self._quiet(printer)

    @pytest.mark.parametrize("unit_type, fullname, registered", [
        ("HTLF", "AFC_lane lane28", True),
        ("HTLF", "AFC_stepper lane28", False),
        ("BambuAMS", "AFC_stepper lane28", True)],
        ids=["an-afc-lane", "a-stepper", "a-stepper-of-a-normal-unit"])
    def test_a_load_switch_only_unit_registers_only_its_afc_lanes(
            self, unit_type, fullname, registered):
        # As handle_connect: on an HTLF-type unit only an [AFC_lane] is a
        # lane; its steppers are not. Any other unit takes either.
        printer, unit, lane = self._pool_lane()
        assert unit.type == "BambuAMS"
        unit.type, lane.fullname = unit_type, fullname
        activate_from_pool(lane)
        assert lane.unassigned is False
        expect = {"lane28": lane} if registered else {}
        assert (unit.lanes, printer.afc.lanes, lane.buffer_obj.lanes) == (
            expect, expect, expect)
        assert lane.buffer_name == ("Bambu_AMS_Buffer" if registered
                                    else None)
        assert lane._afc_prep_done is registered
        self._quiet(printer)


class TestParseMap:
    """A saved lane map as the T# commands PREP reads from it."""

    def test_a_saved_map_reads_as_prep_reads_it(self):
        assert _parse_map(" T3 ,T3, none,T40") == ["T3", "T40"]
        assert _parse_map("T3,,T40") == ["T3", "T40"]
        assert _parse_map(["T3", "NONE"]) == ["T3"]
        assert _parse_map(("T7", " T8 ")) == ["T7", "T8"]
        assert _parse_map(None) == []


class TestPlanLaneMap:
    """The T# map lane12 (home T12) comes back with from its saved record."""

    @staticmethod
    def _rec(maps: str, current: Optional[str] = None) -> Dict[str, Any]:
        """
        :param maps: the saved map
        :param current: the saved current_map; the map's first T# when None
        :return dict: a saved lane record
        """
        cur = maps.split(",")[0].strip() if current is None else current
        return {"map": maps, "current_map": cur}

    @staticmethod
    def _plan(afc: BambuAFC, rec: Dict[str, Any], *, notes: bool = False,
              bambu: Sequence[str] = ()) -> Tuple[tuple, List[LogLine]]:
        """
        Plan lane12's map, its warnings as ("warning", line) and, with
        ``notes``, the lines that need no action as ("debug", line).

        :param afc: the AFC core
        :param rec: lane12's saved record
        :param notes: pass a note channel (None otherwise, as a caller with
            none does)
        :param bambu: the Bambu lanes whose home tool is theirs
        :return tuple: the plan, then the lines in the order they came
        """
        log = BambuLogger()
        plan = _plan_lane_map(afc, "lane12", "T12", rec, log.warning,
                              log.debug if notes else None, set(bambu))
        return plan, log.messages

    def test_no_record_or_no_map_takes_the_home_tool(self):
        afc = make_printer().afc
        assert self._plan(afc, {}) == ((["T12"], "T12", True), [])
        assert self._plan(afc, {"spool_id": 5}) == ((["T12"], "T12", True),
                                                    [])

    def test_a_free_saved_tool_comes_back_and_home_is_not_taken(self):
        afc = make_printer().afc
        assert self._plan(afc, self._rec("T3")) == ((["T3"], "T3", False), [])

    def test_a_tool_another_lane_holds_is_dropped(self):
        afc = make_printer().afc
        _p1_other(afc, "lane5", ["T3"])
        assert self._plan(afc, self._rec("T3")) == (
            (["T12"], "T12", False),
            [("warning", "lane12: saved T3 is held by lane5 -- not restored; "
                         "lane12 is back on T12")])

    def test_the_home_tool_of_the_bambu_lane_holding_it_is_a_note(self):
        # The holder's home wins and the lane is back on its own: nothing
        # for the user to do, so AFC.log only.
        afc = make_printer().afc
        _p1_other(afc, "lane28", ["T28"])
        assert self._plan(afc, self._rec("T28"), notes=True,
                          bambu=["lane28"]) == (
            (["T12"], "T12", False),
            [("debug", "lane12: saved T28 is held by lane28 -- not restored; "
                       "lane12 is back on T12")])

    @pytest.mark.parametrize("maps, bambu, notes, home_held, expected", [
        # Not the holder's home tool.
        (["T3"], ["lane28"], True, False,
         ((["T12"], "T12", False),
          [("warning", "lane12: saved T3 is held by lane28 -- not restored; "
                       "lane12 is back on T12")])),
        # Not a Bambu lane.
        (["T28"], [], True, False,
         ((["T12"], "T12", False),
          [("warning", "lane12: saved T28 is held by lane28 -- not restored; "
                       "lane12 is back on T12")])),
        # The lane is left with no T#.
        (["T28"], ["lane28"], True, True,
         ((["NONE"], "", False),
          [("warning", "lane12: saved T28 is held by lane28 -- not restored"),
           ("warning", "lane12 has no T# -- its home T12 is held by lane6; "
                       "use SET_MAP")])),
        # A caller with no note channel.
        (["T28"], ["lane28"], False, False,
         ((["T12"], "T12", False),
          [("warning", "lane12: saved T28 is held by lane28 -- not restored; "
                       "lane12 is back on T12")])),
    ], ids=["not-its-home-tool", "not-a-bambu-lane", "left-with-no-tool",
            "no-note-channel"])
    def test_anything_else_still_warns(self, maps, bambu, notes, home_held,
                                       expected):
        afc = make_printer().afc
        _p1_other(afc, "lane28", maps)
        if home_held:
            _p1_other(afc, "lane6", ["T12"])
        assert self._plan(afc, self._rec(maps[0]), notes=notes,
                          bambu=bambu) == expected

    def test_a_multi_map_keeping_its_home_tool_is_a_note(self):
        afc = make_printer().afc
        _p1_other(afc, "lane28", ["T28"])
        assert self._plan(afc, self._rec("T12, T28"), notes=True,
                          bambu=["lane28"]) == (
            (["T12"], "T12", True),
            [("debug", "lane12: saved T28 is held by lane28 -- not restored")])
        # A non-Bambu holder still warns.
        assert self._plan(afc, self._rec("T12, T28"), notes=True) == (
            (["T12"], "T12", True),
            [("warning",
              "lane12: saved T28 is held by lane28 -- not restored")])

    def test_with_the_home_tool_held_too_the_lane_gets_none(self):
        afc = make_printer().afc
        _p1_other(afc, "lane5", ["T3"])
        _p1_other(afc, "lane6", ["T12"])
        assert self._plan(afc, self._rec("T3")) == (
            (["NONE"], "", False),
            [("warning", "lane12: saved T3 is held by lane5 -- not restored"),
             ("warning", "lane12 has no T# -- its home T12 is held by lane6; "
                         "use SET_MAP")])

    def test_a_left_over_table_entry_holds_nothing(self):
        afc = make_printer().afc
        afc.tool_cmds["T3"] = "lane5"                  # lane5 is not live
        assert self._plan(afc, self._rec("T3")) == ((["T3"], "T3", False), [])
        _p1_other(afc, "lane5", ["T7"])
        afc.tool_cmds["T3"] = "lane5"                  # its map lacks T3
        assert self._plan(afc, self._rec("T3")) == ((["T3"], "T3", False), [])

    def test_a_macro_keeps_its_tool_unless_force_assign_map(self):
        afc = make_printer().afc
        afc.gcode.register_command("T3", _p1_user_macro)
        assert self._plan(afc, self._rec("T3")) == (
            (["T12"], "T12", False),
            [("warning", "lane12: saved T3 is held by a macro -- not "
                         "restored; lane12 is back on T12")])
        afc.force_assign_map = True
        assert self._plan(afc, self._rec("T3")) == ((["T3"], "T3", False), [])

    def test_a_saved_none_stays_none_whether_home_is_free_or_held(self):
        # As PREP restores an AFC lane's NONE: the owner took its last T#
        # away, and only SET_MAP gives it one.
        afc = make_printer().afc
        assert self._plan(afc, self._rec("NONE", "")) == (
            (["NONE"], "", False), [])
        _p1_other(afc, "lane6", ["T12"])
        assert self._plan(afc, self._rec("NONE", "")) == (
            (["NONE"], "", False), [])

    def test_a_blank_saved_map_is_home_while_free_and_none_while_held(self):
        afc = make_printer().afc
        assert self._plan(afc, self._rec("", "")) == (
            (["T12"], "T12", False), [])
        _p1_other(afc, "lane6", ["T12"])
        assert self._plan(afc, self._rec("", "")) == (
            (["NONE"], "", False), [])

    def test_a_multi_map_keeps_its_current_tool(self):
        afc = make_printer().afc
        assert self._plan(afc, self._rec("T12, T40", "T40")) == (
            (["T12", "T40"], "T40", True), [])

    def test_a_current_tool_that_was_dropped_falls_to_the_first_kept(self):
        afc = make_printer().afc
        _p1_other(afc, "lane5", ["T40"])
        assert self._plan(afc, self._rec("T3, T40", "T40")) == (
            (["T3"], "T3", False),
            [("warning",
              "lane12: saved T40 is held by lane5 -- not restored")])


class TestAssignPoolTcmd:
    """A claimed pool lane's T#s, registered once."""

    @staticmethod
    def _lanes() -> Tuple[BambuAFC, Any, Any]:
        """
        :return tuple: the AFC core (a real afcFunction behind it), then
            lane7 and lane8 of a claimed unit, mapped to nothing yet
        """
        printer = make_printer()
        unit = make_bambu_unit("Bambu_AMS_1", printer=printer,
                               lanes=("lane7", "lane8"))
        return printer.afc, unit.lanes["lane7"], unit.lanes["lane8"]

    @staticmethod
    def _change_tool(afc: BambuAFC, cmd: str) -> bool:
        """:return bool: ``cmd`` is registered to AFC's CHANGE_TOOL"""
        handler = afc.gcode.ready_gcode_handlers.get(cmd)
        return getattr(handler, "__func__", None) is BambuAFC.cmd_CHANGE_TOOL

    def test_a_reclaimed_lane_whose_tcmd_is_ours_is_not_registered_again(
            self):
        afc, lane7, _lane8 = self._lanes()
        # A fresh bound method, as Klipper stores it.
        afc.gcode.register_command("T7", afc.cmd_CHANGE_TOOL)
        lane7.map = ["T7"]
        assign_pool_tcmd(lane7, afc)
        assert afc.tool_cmds == {"T7": "lane7"}
        assert (lane7.map, lane7.current_map) == (["T7"], "T7")
        assert self._change_tool(afc, "T7")
        # TcmdAssign would register T7 again, which Klipper refuses (an
        # error line), and save.
        assert afc.logger.messages == []
        assert afc.save_vars.call_count == 0

    def test_a_reclaimed_lane_keeps_its_current_tool(self):
        afc, lane7, _lane8 = self._lanes()
        for cmd in ("T7", "T40"):
            afc.gcode.register_command(cmd, afc.cmd_CHANGE_TOOL)
        lane7.map, lane7.current_map = ["T7", "T40"], "T40"
        assign_pool_tcmd(lane7, afc)
        assert afc.tool_cmds == {"T7": "lane7", "T40": "lane7"}
        assert lane7.current_map == "T40"
        assert afc.logger.messages == []
        assert afc.save_vars.call_count == 0

    def test_a_new_lane_goes_through_tcmdassign(self):
        afc, lane7, lane8 = self._lanes()
        assign_pool_tcmd(lane7)                       # the lane's own AFC
        lane8.map = ["T8"]
        assign_pool_tcmd(lane8, afc)
        # TcmdAssign gives a lane with no T# the lowest free one.
        assert (lane7.map, lane7.current_map) == (["T0"], "T0")
        assert (lane8.map, lane8.current_map) == (["T8"], "T8")
        assert afc.tool_cmds == {"T0": "lane7", "T8": "lane8"}
        assert self._change_tool(afc, "T0") and self._change_tool(afc, "T8")
        assert afc.logger.messages == []
        assert afc.save_vars.call_count == 2

    def test_a_tcmd_held_by_another_macro_still_goes_through_tcmdassign(
            self):
        afc, lane7, _lane8 = self._lanes()
        afc.gcode.register_command("T7", _p1_user_macro)
        lane7.map = ["T7"]
        assign_pool_tcmd(lane7, afc)
        # Its conflict is reported, and the macro keeps T7.
        assert afc.logger.messages == [
            ("error", "Error trying to map lane lane7 to T7, please make sure "
                      "there are no macros already setup for T7")]
        assert afc.gcode.ready_gcode_handlers["T7"] is _p1_user_macro
        assert afc.tool_cmds == {"T7": "lane7"}
        assert lane7.current_map == "T7"
        assert afc.save_vars.call_count == 1


class TestDeactivateToPool:
    """
    A released lane goes back to the inert pool state: out of every
    registry, no map, and nothing of its spool left on it.
    """

    @staticmethod
    def _live_lane(lane_values: Optional[Dict[str, Any]] = None,
                   **extras: Any) -> Tuple[BambuPrinter, Any, Any]:
        """
        lane28 of Bambu_AMS_1, claimed and carrying Spoolman spool 136, on
        T28, behind Bambu_AMS_Buffer.

        :param lane_values: further lane options (its config)
        :param extras: lane attributes set once it is built
        :return tuple: the printer, the unit and the lane
        """
        printer = make_printer()
        add_buffer(printer, "Bambu_AMS_Buffer")
        values = dict({"map": "T28"}, **(lane_values or {}))
        extras = dict({"current_map": "T28"}, **extras)
        unit = make_bambu_unit(
            "Bambu_AMS_1", printer=printer,
            values={"buffer": "Bambu_AMS_Buffer"},
            lanes=[LaneSpec("lane28", 0, load=True, loaded_to_hub=True,
                            material="PLA", color="#0086D6", spool_id=136,
                            weight=750.0, values=values, extras=extras)])
        return printer, unit, printer.lookup_object("AFC_lane lane28")

    @staticmethod
    def _quiet(printer: BambuPrinter) -> None:
        """Nothing reaches AFC's log or the console."""
        assert printer.afc.logger.messages == []
        assert printer.gcode.messages == []

    def test_release_still_clears_the_lane(self):
        # A lane in the pool carries no spool, and keeps no copy of one.
        printer, unit, lane = self._live_lane()
        regs = (unit.lanes, printer.afc.lanes, unit.hub_obj.lanes,
                lane.extruder_obj.lanes, lane.buffer_obj.lanes)
        assert regs == ({"lane28": lane},) * 5
        assert (lane.map, lane.current_map, lane.raw_load_state) == (
            ["T28"], "T28", True)
        deactivate_to_pool(lane)
        assert regs == ({},) * 5
        assert (lane.spool_id, lane.color, lane.weight) == (None, "", 0.)
        assert (lane.map, lane._map, lane.current_map) == ([], [], "")
        assert "_pool_spool" not in vars(lane)
        # _load_state, not the read-only load_state property: an
        # AttributeError there would be swallowed by the caller and leave
        # the lane assigned.
        assert lane.raw_load_state is False
        assert lane.unassigned is True
        self._quiet(printer)

    def test_a_lane_with_no_buffer_or_hub_object_is_still_released(self):
        printer = make_printer()
        unit = make_bambu_unit("Bambu_AMS_1", printer=printer, lanes=[
            LaneSpec("lane28", 0, load=True, spool_id=136,
                     values={"hub": "direct"})])
        lane = printer.lookup_object("AFC_lane lane28")
        assert (lane.buffer_obj, getattr(lane.hub_obj, "lanes", None)) == (
            None, None)
        deactivate_to_pool(lane)
        assert (unit.lanes, printer.afc.lanes, lane.extruder_obj.lanes) == (
            {}, {}, {})
        assert (lane.spool_id, lane.unassigned) == (None, True)
        self._quiet(printer)

    def test_release_leaves_nothing_of_the_spool_for_the_next_claimant(self):
        # A different unit claiming the bay next must not start from this
        # spool's variant, temperatures, runout lane or TD-1 data. Its own
        # unit gets them back from the record held for it.
        printer, _unit, lane = self._live_lane(
            sub_type="Matte", extruder_temp=225.0, bed_temp=60.0,
            multi_color=["FF0000", "00FF00"], spool_vendor="Bambu Lab",
            filament_name="Bambu PLA Matte", runout_lane="lane29",
            td1_data={"td": 1.2}, need_purge=True)
        deactivate_to_pool(lane)
        assert (lane.sub_type, lane.extruder_temp, lane.bed_temp) == (
            "", None, None)
        assert (lane.multi_color, lane.spool_vendor, lane.filament_name) == (
            [], "", "")
        assert (lane.runout_lane, lane.td1_data, lane.need_purge) == (
            None, {}, False)
        assert (lane.color, lane.material) == ("", None)
        self._quiet(printer)

    def test_release_gives_the_lane_back_its_configured_tare_and_profile(
            self):
        # A Spoolman link or a record set the tare, density and diameter,
        # and the tag the SKU: the next claimant starts from the lane's
        # config, as AFC_lane reads it at startup.
        printer, _unit, lane = self._live_lane(
            {"empty_spool_weight": 210.0}, empty_spool_weight=250.0,
            filament_density=1.27, filament_diameter=2.85, bambu_sku="GFA00")
        deactivate_to_pool(lane)
        assert (lane.empty_spool_weight, lane.filament_density,
                lane.filament_diameter, lane.bambu_sku) == (
            210.0, 1.24, 1.75, "")
        self._quiet(printer)

    def test_release_leaves_none_of_the_spools_details(self):
        # A PC spool on the released unit must not set the load temperature,
        # the variant or the weight maths for whichever unit claims the bay
        # next.
        printer, _unit, lane = self._live_lane(
            {"filament_density": 1.20}, extruder_temp=280, bed_temp=110,
            sub_type="CF", filament_name="PC-CF", spool_vendor="Bambu",
            bambu_sku="GFC01", filament_density=1.3, filament_diameter=2.85,
            empty_spool_weight=250.0)
        deactivate_to_pool(lane)
        assert (lane.extruder_temp, lane.bed_temp) == (None, None)
        assert (lane.sub_type, lane.filament_name, lane.spool_vendor,
                lane.bambu_sku) == ("", "", "", "")
        assert (lane.filament_density, lane.filament_diameter,
                lane.empty_spool_weight) == (1.20, 1.75, 190.0)
        self._quiet(printer)

    def test_a_lane_with_no_config_keeps_its_profile(self):
        # Without a config to read them from, the spool's tare, density and
        # diameter stay as they are.
        printer, _unit, lane = self._live_lane(
            empty_spool_weight=250.0, filament_density=1.3,
            filament_diameter=2.85)
        lane._config = None
        deactivate_to_pool(lane)
        assert (lane.empty_spool_weight, lane.filament_density,
                lane.filament_diameter) == (250.0, 1.3, 2.85)
        assert (lane.spool_id, lane.unassigned) == (None, True)
        self._quiet(printer)


class TestUnsetToolLoaded:
    """AFC's record of a lane in a toolhead, cleared before it is pooled."""

    @staticmethod
    def _setup() -> Tuple[BambuPrinter, Any, Any, Any, Any]:
        """
        lane24 on extruder1 and lane25 on extruder (the active toolhead), on
        Bambu_AMS_1, both staged in their bays.

        :return tuple: the printer, lane24, lane25, extruder, extruder1
        """
        printer = make_printer()
        e1 = add_extruder(printer, "extruder1")
        make_bambu_unit("Bambu_AMS_1", printer=printer, lanes=[
            LaneSpec("lane24", 0, load=True, loaded_to_hub=True,
                     values={"extruder": "extruder1"}),
            LaneSpec("lane25", 1, load=True, loaded_to_hub=True)])
        e0 = printer.afc.tools["extruder"]
        lane24 = printer.lookup_object("AFC_lane lane24")
        lane25 = printer.lookup_object("AFC_lane lane25")
        assert (lane24.extruder_obj, lane25.extruder_obj) == (e1, e0)
        return printer, lane24, lane25, e0, e1

    @staticmethod
    def _load(lane: Any) -> None:
        """What set_tool_loaded leaves: both halves of the record."""
        lane.tool_loaded = True
        lane.status = AFCLaneState.TOOLED
        lane.extruder_obj.lane_loaded = lane.name

    def test_a_lane_in_another_toolhead_skips_the_toolchange_bookkeeping(
            self):
        printer, mine, active, e0, e1 = self._setup()
        self._load(mine)
        self._load(active)
        afc = printer.afc
        afc.current_loading = "loading"
        assert unset_tool_loaded(mine, afc) is True
        assert (mine.tool_loaded, mine.loaded_to_hub) == (False, False)
        # set_tool_unloaded ran on the lane's own extruder.
        assert (e1.lane_loaded, mine.status) == (None, AFCLaneState.NONE)
        # The active tool is untouched, and so is the toolchange state.
        assert (e0.lane_loaded, active.tool_loaded) == ("lane25", True)
        assert afc.spool.calls == []
        assert afc.current_loading == "loading"
        assert afc.save_vars.call_count == 0
        assert afc.logger.messages == []

    def test_the_active_toolheads_lane_goes_through_unset_lane_loaded(self):
        printer, _mine, active, e0, _e1 = self._setup()
        self._load(active)
        afc = printer.afc
        afc.current_loading = "loading"
        assert unset_tool_loaded(active) is True          # the lane's AFC
        assert (active.tool_loaded, active.loaded_to_hub) == (False, False)
        assert (e0.lane_loaded, active.status) == (None, AFCLaneState.NONE)
        # The UNSET_LANE_LOADED path: the active spool cleared, then the
        # extruder re-activated with no lane, and the toolchange state.
        assert afc.spool.calls == [("set_active_spool", (None,), {}),
                                   ("set_active_spool", ("",), {})]
        assert afc.current_loading is None
        # One save per lane the re-activation disables, then its own.
        assert afc.save_vars.call_count == 3
        assert afc.logger.messages == [
            ("debug", "Activating extruder lane: None"),
            ("info", "Manually removing lane25 loaded from toolhead")]

    def test_an_extruder_naming_another_lane_is_left_alone(self):
        printer, mine, other, e0, _e1 = self._setup()
        mine.extruder_obj = e0
        self._load(other)
        mine.tool_loaded = True                   # a stale flag on its own
        mine.status = AFCLaneState.TOOLED
        assert unset_tool_loaded(mine, printer.afc) is True
        assert mine.tool_loaded is False
        # set_tool_unloaded did not run: it would have emptied extruder.
        assert mine.status == AFCLaneState.TOOLED
        assert (e0.lane_loaded, other.tool_loaded) == ("lane25", True)
        assert printer.afc.logger.messages == []

    def test_a_lane_with_no_extruder_only_drops_its_own_flags(self):
        printer, mine, _active, _e0, e1 = self._setup()
        self._load(mine)
        mine.extruder_obj = None
        assert unset_tool_loaded(mine, printer.afc) is True
        assert (mine.tool_loaded, mine.loaded_to_hub) == (False, False)
        # No set_tool_unloaded; the extruder naming it is cleared at the end.
        assert mine.status == AFCLaneState.TOOLED
        assert e1.lane_loaded is None
        assert printer.afc.logger.messages == []

    def test_a_lane_not_in_a_toolhead_is_untouched(self):
        printer, lane, _active, _e0, _e1 = self._setup()
        assert lane_in_toolhead(lane, printer.afc) is False
        assert unset_tool_loaded(lane, printer.afc) is False
        assert lane.loaded_to_hub is True       # staged spool left as it is
        assert lane.status == AFCLaneState.LOADED
        assert printer.afc.logger.messages == []


class TestChildCommandGetattr:
    """A child command reads its own parameters and answers as its parent."""

    def test_the_child_command_answers_through_its_parent(self):
        parent = FakeGcmd({"UID": "EEEE", "FORCE": 1})
        child = _ChildCommand(parent, UID="DDDD")
        assert child.get("UID") == "DDDD"
        assert child.get("NAME", "") == ""
        assert child.get_int("FORCE", 0) == 0
        assert child.get_command_parameters() == {"UID": "DDDD"}
        child.respond_info("hello")
        assert parent.messages == [("respond_info", "hello")]
        assert child.error is parent.error
        with pytest.raises(AttributeError):
            child.no_such_attribute


class TestResetHome:
    """
    AFC_RESET_MAPPING, through the chain master's wrap of AFC's real
    AFC_spool, puts every claimed Bambu lane on its home T#.

    The printer has twelve lanes on a non-Bambu unit (lane0-lane11 on
    T0-T11) ahead of the pool: AMS bays lane12-15, lane16-19, lane20-23,
    lane24-27 and an HT bay on lane28.
    """

    A, B, H = "AAAA", "BBBB", "HHHH"
    #: The AMS on lane12-15 and its home T#s.
    HOMES = {f"lane{n}": f"T{n}" for n in (12, 13, 14, 15)}

    class _Box:
        """A non-Bambu unit."""

        def __init__(self, lanes: Sequence[Any]) -> None:
            """:param lanes: its lanes"""
            self.name = "Box"
            self.lanes = {lane.name: lane for lane in lanes}

    @classmethod
    def _printer(cls, tmp_path: pathlib.Path,
                 monkeypatch: pytest.MonkeyPatch, *,
                 var: Optional[Dict[str, Any]] = None,
                 owners: Optional[str] = None,
                 box: Optional[Dict[int, Tuple[List[str], List[str]]]] = None,
                 print_state: Optional[str] = None,
                 ready: bool = True) -> afcBridgeBox:
        """
        chain1 with AAAA, BBBB and HHHH recorded and online, AFC's real
        AFC_spool, and the Box unit ahead of it.

        :param var: what AFC.var.unit holds
        :param owners: the bay_owner key an earlier start left
        :param box: Box lane number -> (its T#s, its config map:); the
            others are on T<number> with no config map:
        :param print_state: print_stats state
        :param ready: run klippy:ready's handler, which wraps the reset
        :return afcBridgeBox: the master; its logger is AFC's
        """
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True,
                               print_state=print_state)
        # The roster an earlier start recorded, as a scout leaves it.
        record_chain_state(
            tmp_path, roster=f"boxed:{cls.A}, boxed:{cls.B}, ht:{cls.H}")
        m = make_bridgebox(
            tmp_path, printer=printer, roster="", pool_ams=4, pool_ht=1,
            ams_names="Alpha, Bravo, Charlie, Delta", ht_names="Hot",
            lane_base=12)
        printer.connect()
        if owners is not None:
            m._state_set({"AFC_BridgeBox chain1": {"bay_owner": owners}})
        if var is not None:
            write_unit_vars(printer, var)
        uids = [cls.A, cls.B, "", "", cls.H]
        live_bridges()[m.serial_port] = FakeBridge(
            uids=uids, online=[bool(u) for u in uids], htmask=1 << 4)
        make_afc_spool(printer)
        afc = printer.afc
        afc.moonraker = None
        lanes = []
        for n in range(12):
            maps, config = (box or {}).get(n, ([f"T{n}"], []))
            lanes.append(_p1_other(afc, f"lane{n}", maps, config))
        afc.units = dict(Box=cls._Box(lanes), **afc.units)
        if ready:
            m._scout_ready()
        return m

    @staticmethod
    def _lane(m: afcBridgeBox, name: str) -> Any:
        """:return Any: the lane AFC has under that name, else the bay's"""
        afc = m.printer.afc
        return (afc.lanes.get(name)
                or m.printer.lookup_object(f"AFC_lane {name}"))

    @classmethod
    def _maps(cls, m: afcBridgeBox, *names: str) -> Dict[str, List[str]]:
        """:return dict: lane name -> its map"""
        return {n: cls._lane(m, n).map for n in names}

    @staticmethod
    def _reset(m: afcBridgeBox) -> FakeGcmd:
        """:return FakeGcmd: AFC_RESET_MAPPING RUNOUT=no, run"""
        cmd = FakeGcmd({"RUNOUT": "no"})
        m.printer.afc.spool.cmd_AFC_RESET_MAPPING(cmd)
        return cmd

    @staticmethod
    def _quiet(m: afcBridgeBox) -> None:
        """Forget what the setup logged and saved."""
        m.logger.messages.clear()
        m.printer.gcode.messages.clear()
        drain_var_writes(m.printer)

    @staticmethod
    def _saved(m: afcBridgeBox) -> Dict[str, Any]:
        """:return dict: the one snapshot AFC's writer was handed"""
        (snap,) = drain_var_writes(m.printer)
        return snap

    @staticmethod
    def _table(**extra: str) -> Dict[str, str]:
        """
        :param extra: T# -> lane, besides the Box's T0-T11
        :return dict: AFC's tool table as expected
        """
        table = {f"T{n}": f"lane{n}" for n in range(12)}
        table.update(extra)
        return table

    class _Moonraker:
        """Moonraker's lane_data store, as send_lane_data writes it."""

        def __init__(self) -> None:
            """Start with nothing sent."""
            #: (T#, lane) of every record sent, in order.
            self.sent: List[Tuple[str, str]] = []
            self.removed: List[Tuple[str, str]] = []

        def send_lane_data(self, data: Dict[str, Any]) -> None:
            """:param data: the lane_data record"""
            self.sent.append((data["key"], data["value"]["lane"]))

        def remove_database_entry(self, namespace: str, key: str) -> None:
            """
            :param namespace: the database namespace
            :param key: the record removed
            """
            self.removed.append((namespace, key))

    @staticmethod
    def _change_tool(m: afcBridgeBox, cmd: str) -> bool:
        """:return bool: ``cmd`` is registered to AFC's CHANGE_TOOL"""
        handler = m.printer.gcode.ready_gcode_handlers.get(cmd)
        return getattr(handler, "__func__", None) is BambuAFC.cmd_CHANGE_TOOL

    @classmethod
    def _consistent(cls, m: afcBridgeBox, macros: Sequence[str] = ()) -> None:
        """
        Each T# is on one live lane's map only, AFC's tool table is exactly
        those owners, and each is AFC's CHANGE_TOOL unless it is a macro.

        :param m: the chain master
        :param macros: the T#s held by :func:`_p1_user_macro`
        """
        afc = m.printer.afc
        owner: Dict[str, str] = {}
        for lane in afc.lanes.values():
            for cmd in lane.map or []:
                if cmd == "NONE":
                    continue
                assert cmd not in owner, f"{cmd} on {owner[cmd]} and {lane.name}"
                owner[cmd] = lane.name
        assert afc.tool_cmds == owner
        handlers = m.printer.gcode.ready_gcode_handlers
        for cmd in owner:
            if cmd in macros:
                assert handlers[cmd] is _p1_user_macro, cmd
            else:
                assert cls._change_tool(m, cmd), cmd

    @classmethod
    def _claimed(cls, m: afcBridgeBox, *uids: str) -> None:
        """
        Claim each uid onto its bay: HHHH as an HT, the others as AMS.

        :param m: the chain master
        :param uids: the uids, in claim order
        """
        for uid in uids:
            model = "ht" if uid == cls.H else "boxed"
            assert m._claim_pool_unit(uid, model) is not None

    @classmethod
    def _box_is_numbered_1_to_1(cls, m: afcBridgeBox) -> None:
        """The Box's lanes are on T0-T11, in lane order."""
        names = [f"lane{n}" for n in range(12)]
        assert cls._maps(m, *names) == {
            f"lane{n}": [f"T{n}"] for n in range(12)}

    def _homes(self) -> Dict[str, List[str]]:
        """:return dict: the AMS on lane12-15, each on its home T#"""
        return {ln: [t] for ln, t in self.HOMES.items()}

    def test_the_ht_behind_sixteen_lanes_is_back_on_t28(
            self, tmp_path, monkeypatch):
        # The unit on lane16-19 is offline, and an earlier reset left the HT
        # on T16, which its saved map brought back.
        A, B, H = self.A, self.B, self.H
        m = self._printer(tmp_path, monkeypatch,
                          var={"Hot": {"lane28": {"map": "T16",
                                                  "current_map": "T16"}}},
                          owners=f"{A}:Alpha, {B}:Bravo, {H}:Hot")
        self._claimed(m, A, H)
        afc = m.printer.afc
        ht = self._lane(m, "lane28")
        assert ht.map == ["T16"]
        moonraker = afc.moonraker = self._Moonraker()
        self._quiet(m)
        cmd = self._reset(m)
        assert (ht.map, ht.current_map, ht._map) == (["T28"], "T28", [])
        assert self._maps(m, *self.HOMES) == self._homes()
        self._box_is_numbered_1_to_1(m)
        assert afc.tool_cmds == self._table(T12="lane12", T13="lane13",
                                            T14="lane14", T15="lane15",
                                            T28="lane28")
        # T16 is no lane's: gone from Klipper's commands too. T28 is
        # registered to CHANGE_TOOL, under its own name.
        handlers = m.printer.gcode.ready_gcode_handlers
        assert ("T16" in handlers, "_T28" in handlers) == (False, False)
        assert self._change_tool(m, "T28")
        self._consistent(m)
        # The panel gets each Bambu lane on its home T#.
        assert moonraker.sent == [("T12", "12"), ("T13", "13"),
                                  ("T14", "14"), ("T15", "15"),
                                  ("T28", "28")]
        assert moonraker.removed == []
        # The bay held for the offline unit is left alone.
        assert "lane16" not in afc.lanes
        bay = m.printer.lookup_object("AFC_lane lane16")
        assert (bay.map, bay._map) == ([], [])
        # Saved as any reset: the HT and the AMS lanes on their home T#s.
        snap = self._saved(m)
        assert (snap["Hot"]["lane28"]["map"],
                snap["Hot"]["lane28"]["current_map"]) == ("T28", "T28")
        assert {ln: r["map"] for ln, r in snap["Alpha"].items()} == (
            self.HOMES)
        assert snap["Box"]["lane0"]["map"] == "T0"
        assert m.logger.messages == [
            ("info", "T16 remain, removing these mappings"),
            ("info", "Tool mappings reset"),
            ("debug", "AFC_BridgeBox chain1: the mapping reset put the Bambu "
                      "lanes on their home T#s: lane28 T16->T28.")]
        assert (cmd.messages, m.printer.gcode.messages) == ([], [])

    def test_every_bay_claimed(self, tmp_path, monkeypatch):
        m = self._printer(tmp_path, monkeypatch)
        self._claimed(m, self.A, self.B, self.H)
        self._quiet(m)
        self._reset(m)
        bravo = {f"lane{n}": [f"T{n}"] for n in (16, 17, 18, 19)}
        assert self._maps(m, "lane28", *self.HOMES, *bravo) == dict(
            self._homes(), lane28=["T28"], **bravo)
        self._box_is_numbered_1_to_1(m)
        assert m.printer.afc.tool_cmds == self._table(
            **{f"T{n}": f"lane{n}" for n in (12, 13, 14, 15, 16, 17, 18, 19,
                                              28)})
        self._consistent(m)
        snap = self._saved(m)
        assert (snap["Hot"]["lane28"]["map"],
                snap["Bravo"]["lane16"]["map"]) == ("T28", "T16")
        # Every lane was on its home T# already: nothing moved to report.
        assert m.logger.messages == [("info", "Tool mappings reset")]

    def test_a_released_bay_is_left_out_and_comes_back_home(
            self, tmp_path, monkeypatch):
        m = self._printer(tmp_path, monkeypatch)
        self._claimed(m, self.A, self.B, self.H)
        m._release_pool_unit(self.B)
        afc = m.printer.afc
        assert "lane16" not in afc.lanes
        self._quiet(m)
        self._reset(m)
        assert self._maps(m, "lane28", *self.HOMES) == dict(
            self._homes(), lane28=["T28"])
        assert m.printer.lookup_object("AFC_lane lane16").map == []
        assert "lane16" not in afc.lanes
        assert afc.tool_cmds == self._table(T12="lane12", T13="lane13",
                                            T14="lane14", T15="lane15",
                                            T28="lane28")
        self._consistent(m)
        # Its records are held for its return, on its home T#s.
        held = m._held["Bravo"]["lanes"]
        assert {ln: r["map"] for ln, r in held.items()} == {
            "lane16": "T16", "lane17": "T17", "lane18": "T18",
            "lane19": "T19"}
        assert self._saved(m)["Bravo"]["lane16"]["map"] == "T16"
        assert m.logger.messages == [("info", "Tool mappings reset")]
        self._claimed(m, self.B)
        assert self._maps(m, "lane16", "lane28") == {"lane16": ["T16"],
                                                     "lane28": ["T28"]}
        assert afc.tool_cmds == self._table(
            **{f"T{n}": f"lane{n}" for n in (12, 13, 14, 15, 16, 17, 18, 19,
                                              28)})
        self._consistent(m)

    def test_a_remap_on_a_bambu_lane_is_undone(self, tmp_path, monkeypatch):
        m = self._printer(tmp_path, monkeypatch)
        self._claimed(m, self.A, self.H)
        m.printer.afc.spool.cmd_SET_MAP(FakeGcmd({"LANE": "lane28",
                                                   "MAP": "T3"}))
        assert self._maps(m, "lane28", "lane3") == {"lane28": ["T3"],
                                                    "lane3": ["T28"]}
        self._quiet(m)
        self._reset(m)
        assert self._maps(m, "lane28", "lane3") == {"lane28": ["T28"],
                                                    "lane3": ["T3"]}
        assert m.printer.afc.tool_cmds == self._table(
            T12="lane12", T13="lane13", T14="lane14", T15="lane15",
            T28="lane28")
        self._consistent(m)
        assert m.logger.messages == [
            ("info", "Tool mappings reset"),
            ("debug", "AFC_BridgeBox chain1: the mapping reset put the Bambu "
                      "lanes on their home T#s: lane28 T3->T28.")]

    @pytest.mark.parametrize("how", ["AFC_RESET_MAPPING",
                                     "AFC_ENABLE_MULTIPLE_MAPPING ENABLE=0"])
    def test_a_multi_mapped_bambu_lane_is_back_to_one_tool(
            self, tmp_path, monkeypatch, how):
        m = self._printer(tmp_path, monkeypatch)
        self._claimed(m, self.A, self.H)
        spool = m.printer.afc.spool
        spool.enable_multiple_mapping = True
        spool.cmd_SET_MAP(FakeGcmd({"LANE": "lane28", "MAP": "T2"}))
        assert self._maps(m, "lane28", "lane2") == {"lane28": ["T28", "T2"],
                                                    "lane2": []}
        self._quiet(m)
        moved = ("debug", "AFC_BridgeBox chain1: the mapping reset put the "
                          "Bambu lanes on their home T#s: lane28 T28+T2->T28.")
        if how == "AFC_RESET_MAPPING":
            self._reset(m)
            expected = [("info", "Tool mappings reset"), moved]
        else:
            spool.cmd_AFC_ENABLE_MULTIPLE_MAPPING(FakeGcmd({"ENABLE": 0}))
            expected = [
                ("info", "Multiple T(n) mapping per lane and virtual tools "
                         "has been disabled"),
                ("info", "Tool mappings reset"), moved,
                ("info", "\n<span class=info--text>Key enable_multiple_mapping "
                         "not found in section AFC added to AFC_auto_vars.cfg "
                         "file</span>")]
        lane28 = self._lane(m, "lane28")
        assert (lane28.map, lane28.current_map) == (["T28"], "T28")
        assert self._lane(m, "lane2").map == ["T2"]
        self._box_is_numbered_1_to_1(m)
        assert m.printer.afc.tool_cmds == self._table(
            T12="lane12", T13="lane13", T14="lane14", T15="lane15",
            T28="lane28")
        self._consistent(m)
        assert self._saved(m)["Hot"]["lane28"]["map"] == "T28"
        assert m.logger.messages == expected

    def test_the_reset_leaves_the_tool_with_the_config_lane(
            self, tmp_path, monkeypatch):
        # lane5's config gives it T28, the HT's home T#.
        m = self._printer(tmp_path, monkeypatch, box={5: (["T28"], ["T28"])})
        self._claimed(m, self.A, self.H)
        self._quiet(m)
        self._reset(m)
        afc = m.printer.afc
        ht = self._lane(m, "lane28")
        # AFC numbers the HT as any lane with no map:, around lane5's T28
        # and the AMS's T12-T15: lane0-4 on T0-T4, lane6-11 on T5-T10, the
        # HT on T11.
        assert (self._lane(m, "lane5").map, ht.map, ht._map) == (
            ["T28"], ["T11"], [])
        assert self._maps(m, *self.HOMES) == self._homes()
        assert afc.tool_cmds == dict(
            {f"T{n}": f"lane{n}" for n in range(5)},
            **{f"T{n - 1}": f"lane{n}" for n in range(6, 12)},
            T11="lane28", T12="lane12", T13="lane13", T14="lane14",
            T15="lane15", T28="lane5")
        self._consistent(m)
        assert self._saved(m)["Hot"]["lane28"]["map"] == "T11"
        left = ("AFC_BridgeBox chain1: the mapping reset left lane28 on T11, "
                "not its home T28: map: T28 in [AFC_stepper lane5] keeps T28 "
                "for lane5. Set that map: outside T12-T28 to have lane28 on "
                "T28.")
        # The claim had moved lane5 to T29; the reset drops it.
        assert m.logger.messages == [
            ("info", "T29 remain, removing these mappings"),
            ("info", "Tool mappings reset"),
            ("warning", left)]
        # Said once a session; AFC.log after that.
        self._quiet(m)
        self._reset(m)
        assert m.logger.messages == [("info", "Tool mappings reset"),
                                     ("debug", left)]

    @classmethod
    def _waiting(cls, m: afcBridgeBox) -> None:
        """
        lane11 is on T28 (PREP restored it there before the HT was
        plugged), and the HT is claimed during the print, so its take of
        T28 waits for the print to end.

        :param m: the chain master, lane11 on T28
        """
        m.printer.set_print_state("printing")
        cls._claimed(m, cls.A, cls.H)
        assert m.printer.lookup_object("AFC_lane lane28").map == ["NONE"]
        assert m._deferred_takes == {"lane28": {
            "home": "T28", "maps": ["T28"], "current": "T28",
            "quiet": False}}

    def test_a_reset_before_the_print_ends_takes_its_place(
            self, tmp_path, monkeypatch):
        m = self._printer(tmp_path, monkeypatch, box={11: (["T28"], [])})
        self._waiting(m)
        self._quiet(m)
        self._reset(m)                                 # PRINT_END
        assert self._maps(m, "lane28", "lane11") == {"lane28": ["T28"],
                                                     "lane11": ["T11"]}
        assert m._deferred_takes == {}
        # The reset's save has the map the reset gave, not the planned one.
        assert self._saved(m)["Hot"]["lane28"]["map"] == "T28"
        assert m.printer.afc.tool_cmds == self._table(
            T12="lane12", T13="lane13", T14="lane14", T15="lane15",
            T28="lane28")
        self._consistent(m)
        assert m.logger.messages == [
            ("info", "Tool mappings reset"),
            ("debug", "AFC_BridgeBox chain1: the mapping reset put the Bambu "
                      "lanes on their home T#s: lane28 NONE->T28."),
            ("debug", "AFC_BridgeBox chain1: the mapping reset mapped "
                      "lane28, so the T#s it waited for are not taken.")]
        # The print ends: nothing is left to take.
        m.printer.set_print_state("complete")
        self._quiet(m)
        m._take_deferred_tools()
        assert self._maps(m, "lane28", "lane11") == {"lane28": ["T28"],
                                                     "lane11": ["T11"]}
        assert m.logger.messages == []
        assert drain_var_writes(m.printer) == []

    def test_a_failed_reset_puts_the_wait_and_the_maps_back(
            self, tmp_path, monkeypatch):
        m = self._printer(tmp_path, monkeypatch, box={11: (["T28"], [])})
        self._waiting(m)
        afc = m.printer.afc
        entry = dict(m._deferred_takes["lane28"])
        save_vars = afc.save_vars

        def full_disk() -> None:
            """AFC's save, failing."""
            raise RuntimeError("disk full")

        afc.save_vars = full_disk
        self._quiet(m)
        with pytest.raises(RuntimeError, match="disk full"):
            self._reset(m)
        assert m._deferred_takes == {"lane28": entry}
        assert {n: self._lane(m, f"lane{n}")._map
                for n in (12, 13, 14, 15, 28)} == dict.fromkeys(
            (12, 13, 14, 15, 28), [])
        # AFC's reset mapped the lanes before its save failed.
        assert self._maps(m, "lane28", "lane11") == {"lane28": ["T28"],
                                                     "lane11": ["T11"]}
        assert afc.tool_cmds == self._table(T12="lane12", T13="lane13",
                                            T14="lane14", T15="lane15",
                                            T28="lane28")
        # A failed reset says nothing of the lanes.
        assert (m.logger.messages, m.printer.gcode.messages) == ([], [])
        # The next reset that goes through takes the wait's place. The
        # failed one left lane28 on T28 already, so nothing moved to report.
        afc.save_vars = save_vars
        self._quiet(m)
        self._reset(m)
        assert self._lane(m, "lane28").map == ["T28"]
        assert m._deferred_takes == {}
        assert self._saved(m)["Hot"]["lane28"]["map"] == "T28"
        assert m.logger.messages == [
            ("info", "Tool mappings reset"),
            ("debug", "AFC_BridgeBox chain1: the mapping reset mapped "
                      "lane28, so the T#s it waited for are not taken.")]

    def test_the_maps_come_back_when_the_reset_raises(
            self, tmp_path, monkeypatch):
        m = self._printer(tmp_path, monkeypatch, ready=False)
        spool = m.printer.afc.spool
        seen: Dict[str, List[str]] = {}

        def boom(*args: Any, **kwargs: Any) -> None:
            """AFC's reset, recording each lane's map: and failing."""
            seen.update({n: list(self._lane(m, n)._map)
                         for n in ("lane12", "lane28", "lane0")})
            raise RuntimeError("reset failed")

        spool._reset_mapping = boom
        m._scout_ready()
        assert spool._reset_mapping.__wrapped__ is boom
        self._claimed(m, self.A, self.H)
        self._quiet(m)
        with pytest.raises(RuntimeError, match="reset failed"):
            self._reset(m)
        # During the call each Bambu lane had its home T# as its map:...
        assert seen == {"lane12": ["T12"], "lane28": ["T28"], "lane0": []}
        # ...and after it, the map: it had before.
        assert {n: self._lane(m, f"lane{n}")._map
                for n in (0, 12, 13, 14, 15, 28)} == dict.fromkeys(
            (0, 12, 13, 14, 15, 28), [])
        assert self._lane(m, "lane28").map == ["T28"]
        assert m.logger.messages == []

    @classmethod
    def _two_tools(cls, tmp_path: pathlib.Path,
                   monkeypatch: pytest.MonkeyPatch,
                   config: List[str]) -> afcBridgeBox:
        """
        :param config: lane5's config map:, which it is on too
        :return afcBridgeBox: AAAA and HHHH claimed
        """
        m = cls._printer(tmp_path, monkeypatch, box={5: (config, config)})
        cls._quiet(m)
        cls._claimed(m, cls.A, cls.H)
        return m

    @staticmethod
    def _claim_lines(uid: str, unit: str, model: str,
                     lanes: int, tail: str = "") -> List[Tuple[str, str]]:
        """
        :param uid: the uid claimed
        :param unit: the unit it is claimed onto
        :param model: the model it is claimed as
        :param lanes: the unit's lane count
        :param tail: what the CLAIMED line ends with after "no restart."
        :return list: the claim's log, without any warning between
        """
        return [
            ("debug", f"AFC bambu {unit}: chain index not resolved yet (UID "
                      f"{uid}); holding this unit's registrations until the "
                      "chain map arrives"),
            ("info", f"AFC bambu {unit}: claimed UID {uid} as {model} and "
                     "brought online live (ams_index=0)."),
            ("info", f"AFC_BridgeBox chain1: CLAIMED {uid} as {model} onto "
                     f"{unit} ({lanes} lanes) -- live, no restart.{tail}"),
        ]

    def test_a_home_tool_after_the_first_goes_home(self, tmp_path,
                                                   monkeypatch):
        # AFC's reset gives a lane with a config map: only its first T#, so
        # only that one keeps a Bambu lane off its home T#.
        m = self._two_tools(tmp_path, monkeypatch, ["T5", "T28"])
        # The claim takes T28 and does not say a reset gives it back.
        hot = self._claim_lines("HHHH", "Hot", "ht", 1)
        assert m.logger.messages == [
            *self._claim_lines("AAAA", "Alpha", "boxed", 4), *hot[:2],
            ("warning", "AFC_BridgeBox chain1: T28 is the tool of Bambu lane "
                        "lane28 (Bambu lanes lane12-lane28 are T12-T28). "
                        "lane5 was mapped to it and is now T5."),
            hot[2]]
        assert m.printer.gcode.messages == []
        self._quiet(m)
        self._reset(m)
        assert self._maps(m, "lane5", "lane28") == {"lane5": ["T5"],
                                                    "lane28": ["T28"]}
        assert m.printer.afc.tool_cmds == self._table(
            T12="lane12", T13="lane13", T14="lane14", T15="lane15",
            T28="lane28")
        assert self._change_tool(m, "T28")
        self._box_is_numbered_1_to_1(m)
        self._consistent(m)
        assert self._saved(m)["Hot"]["lane28"]["map"] == "T28"
        assert m.logger.messages == [("info", "Tool mappings reset")]

    def test_a_home_tool_first_in_the_map_stays_with_it(self, tmp_path,
                                                        monkeypatch):
        m = self._two_tools(tmp_path, monkeypatch, ["T28", "T5"])
        hot = self._claim_lines("HHHH", "Hot", "ht", 1)
        assert m.logger.messages == [
            *self._claim_lines("AAAA", "Alpha", "boxed", 4), *hot[:2],
            ("warning", "AFC_BridgeBox chain1: T28 is the tool of Bambu lane "
                        "lane28 (Bambu lanes lane12-lane28 are T12-T28). "
                        "lane5 was mapped to it and is now T5. Also set map: "
                        "in [AFC_stepper lane5] outside T12-T28, or "
                        "AFC_RESET_MAPPING puts lane5 back on T28 and lane28 "
                        "on another T#."),
            hot[2]]
        assert m.printer.gcode.messages == []
        self._quiet(m)
        self._reset(m)
        # lane5's map: holds T5 too, so AFC numbers past it: the HT, the
        # seventeenth lane, is T16.
        assert self._maps(m, "lane5", "lane28") == {"lane5": ["T28"],
                                                    "lane28": ["T16"]}
        assert m.printer.afc.tool_cmds == dict(
            {f"T{n}": f"lane{n}" for n in (0, 1, 2, 3, 4, 6, 7, 8, 9, 10,
                                           11, 12, 13, 14, 15)},
            T16="lane28", T28="lane5")
        self._consistent(m)
        assert m.logger.messages == [
            ("info", "T5 remain, removing these mappings"),
            ("info", "Tool mappings reset"),
            ("warning", "AFC_BridgeBox chain1: the mapping reset left lane28 "
                        "on T16, not its home T28: map: T28 in [AFC_stepper "
                        "lane5] keeps T28 for lane5. Set that map: outside "
                        "T12-T28 to have lane28 on T28.")]

    @pytest.mark.parametrize("force", [False, True])
    def test_the_reset_leaves_the_lane_to_afc(self, tmp_path, monkeypatch,
                                              force):
        # A macro other than AFC's CHANGE_TOOL on the HT's home T#. A claim
        # with a saved record leaves the T# to it, and AFC's reset registers
        # a T# without renaming what holds it, so the reset leaves it to the
        # macro too, whatever force_assign_map says.
        A, B, H = self.A, self.B, self.H
        m = self._printer(tmp_path, monkeypatch,
                          var={"Hot": {"lane28": {"map": "T16",
                                                  "current_map": "T16"}}},
                          owners=f"{A}:Alpha, {B}:Bravo, {H}:Hot")
        afc = m.printer.afc
        afc.force_assign_map = force
        handlers = m.printer.gcode.ready_gcode_handlers
        m.printer.gcode.register_command("T28", _p1_user_macro)
        self._quiet(m)
        self._claimed(m, A, H)
        ht = self._lane(m, "lane28")
        assert ht.map == ["T16"]
        assert handlers["T28"] is _p1_user_macro
        # No warning: the saved record keeps the HT off T28. With force,
        # AFC renames whatever holds each T# the claim registers first.
        alpha = self._claim_lines("AAAA", "Alpha", "boxed", 4)
        hot = self._claim_lines("HHHH", "Hot", "ht", 1,
                                " Saved maps: lane28->T16.")
        renames = {
            cmd: [("debug", "<span class=warning--text>Existing command "
                            f"{cmd} not found in gcode_macros</span>"),
                  ("debug", f"PREP-renaming macro {cmd}")] if force else []
            for cmd in ("T12", "T13", "T14", "T15", "T16")}
        assert m.logger.messages == [
            *alpha[:2], *renames["T12"], *renames["T13"], *renames["T14"],
            *renames["T15"], alpha[2], *hot[:2], *renames["T16"], hot[2]]
        assert m.printer.gcode.messages == []
        self._quiet(m)
        self._reset(m)
        # AFC numbers it: the seventeenth lane, past the AMS's T12-T15.
        assert (ht.map, ht.current_map, ht._map) == (["T16"], "T16", [])
        assert handlers["T28"] is _p1_user_macro
        assert afc.tool_cmds == self._table(T12="lane12", T13="lane13",
                                            T14="lane14", T15="lane15",
                                            T16="lane28")
        assert self._maps(m, *self.HOMES) == self._homes()
        self._box_is_numbered_1_to_1(m)
        self._consistent(m)
        assert self._saved(m)["Hot"]["lane28"]["map"] == "T16"
        left = ("AFC_BridgeBox chain1: the mapping reset left lane28 on T16, "
                "not its home T28: T28 is a macro other than AFC's "
                "CHANGE_TOOL, which AFC does not replace. Remove or rename "
                "that macro to have lane28 on T28.")
        assert m.logger.messages == [("info", "Tool mappings reset"),
                                     ("warning", left)]
        # Said once a session; AFC.log after that.
        self._quiet(m)
        self._reset(m)
        assert m.logger.messages == [("info", "Tool mappings reset"),
                                     ("debug", left)]

    def test_a_lane_already_on_it_stays_and_the_macro_too(
            self, tmp_path, monkeypatch):
        # A claim with no record takes its home T# whatever holds it.
        m = self._printer(tmp_path, monkeypatch)
        handlers = m.printer.gcode.ready_gcode_handlers
        m.printer.gcode.register_command("T28", _p1_user_macro)
        self._claimed(m, self.A, self.H)
        ht = self._lane(m, "lane28")
        assert ht.map == ["T28"]
        self._quiet(m)
        self._reset(m)
        # Left where the claim put it: numbering it elsewhere would have
        # AFC's reset unregister T28, the user's macro, as a T# no lane has.
        assert ht.map == ["T28"]
        assert handlers["T28"] is _p1_user_macro
        assert m.printer.afc.tool_cmds == self._table(
            T12="lane12", T13="lane13", T14="lane14", T15="lane15",
            T28="lane28")
        self._consistent(m, macros=("T28",))
        assert m.logger.messages == [("info", "Tool mappings reset")]

    def test_the_warning_says_when_the_tool_it_got_is_a_macro(
            self, tmp_path, monkeypatch):
        # lane5's config holds T28, and T11, where AFC's reset numbers the
        # HT, is a macro (lane11 is on T40).
        m = self._printer(tmp_path, monkeypatch,
                          box={5: (["T28"], ["T28"]), 11: (["T40"], [])})
        m.printer.gcode.register_command("T11", _p1_user_macro)
        self._claimed(m, self.A, self.H)
        self._quiet(m)
        self._reset(m)
        assert self._lane(m, "lane28").map == ["T11"]
        assert m.printer.gcode.ready_gcode_handlers["T11"] is _p1_user_macro
        self._consistent(m, macros=("T11",))
        assert m.logger.messages == [
            ("error", "Error trying to map lane lane28 to T11, please make "
                      "sure there are no macros already setup for T11"),
            ("info", "T29, T40 remain, removing these mappings"),
            ("info", "Tool mappings reset"),
            ("warning", "AFC_BridgeBox chain1: the mapping reset left lane28 "
                        "on T11, not its home T28: map: T28 in [AFC_stepper "
                        "lane5] keeps T28 for lane5. Set that map: outside "
                        "T12-T28 to have lane28 on T28. T11 is a macro other "
                        "than AFC's CHANGE_TOOL, so it does not select "
                        "lane28: SET_MAP LANE=lane28 MAP=<T#> gives it a T# "
                        "that does.")]


class TestAfcBridgeBoxInit:
    """What a start builds from its options and its recorded state."""

    AUTOV = ("[AFC_BambuAMS Bambu_AMS_HT_1]\n"
             "afc_bowden_length : 3632.0\n"
             "afc_unload_bowden_length : 3632.0\n")

    class _MinvalConfig(BambuConfig):
        """BambuConfig whose getint enforces minval, as klippy's does."""

        def getint(self, option: str, *args: Any,
                   minval: Optional[int] = None, **kwargs: Any) -> Any:
            """
            :param option: the option
            :param minval: the lowest value accepted
            :return Any: the value
            """
            value = super().getint(option, *args, **kwargs)
            if minval is not None and value is not None and value < minval:
                error_str = (f"Option '{option}' in section "
                             f"'{self.get_name()}' must have minimum of "
                             f"{minval}")
                raise configparser.Error(error_str)
            return value

    @staticmethod
    def _direct(tmp_path: pathlib.Path, config_cls: Any,
                printer: Optional[BambuPrinter] = None,
                name: str = "chain1", **options: Any) -> afcBridgeBox:
        """
        A chain's master built as make_bridgebox builds it, through a config
        class of the test's own and loaded as klippy loads a section, by
        load_config_prefix.

        :param tmp_path: where its files live
        :param config_cls: the config class
        :param printer: the printer; a new one when None
        :param name: the chain's name
        :return afcBridgeBox: the master, registered on the printer
        """
        printer = printer or make_printer()
        if printer.afc.VarFile == NO_VAR_FILE:
            set_var_file(printer, str(tmp_path / "AFC.var"))
        section = f"AFC_BridgeBox {name}"
        opts = bridgebox_options(tmp_path, name, **options)
        printer.add_section(section, {k: v for k, v in opts.items()
                                      if v is not None})
        master = load_config_prefix(config_cls(section, printer, opts))
        assert isinstance(master, afcBridgeBox)
        printer.add_object(section, master)
        return master

    # layouts and notes the tests compare against

    @staticmethod
    def _layout(master: afcBridgeBox) -> Dict[int, str]:
        """
        :param master: a built master
        :return dict: lane number -> the unit it was fabricated for
        """
        return {int(s.rsplit("lane", 1)[1]):
                w.fileconfig.get(s, "unit").split(":")[0]
                for s, w in master.printer.loaded if s.startswith("AFC_lane ")}

    @staticmethod
    def _expect(*bays: Tuple[int, int, str]) -> Dict[int, str]:
        """
        :param bays: (first lane, lane count, unit name) per bay
        :return dict: lane number -> unit name, the layout those bays make
        """
        return {first + i: name for first, count, name in bays
                for i in range(count)}

    #: The layout FOUR builds with the four-AMS, two-HT pool.
    FOUR_LAYOUT = _expect(
        (24, 4, "Bambu_AMS_1"), (28, 4, "Bambu_AMS_2"),
        (32, 4, "Bambu_AMS_3"), (36, 4, "Bambu_AMS_4"),
        (40, 1, "Bambu_AMS_HT_1"), (41, 1, "Bambu_AMS_HT_2"))

    @staticmethod
    def _lane_numbers(master: afcBridgeBox, since: int = 0) -> List[int]:
        """
        :param master: a built master
        :param since: the first load call to read
        :return list: the lane numbers its printer loaded from that call on
        """
        return [int(s.split("lane")[-1])
                for s, _w in master.printer.loaded[since:]
                if s.startswith("AFC_lane ")]

    @staticmethod
    def _held_note(need: int) -> str:
        """
        :param need: the AMS bays pool_ams and the recorded AMS need
        :return str: the ready line for HHHH holding the AMS band at four bays
        """
        return (f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) on lane40 (T40) "
                f"keeps its lanes, so the AMS band stays 4 bays although pool_ams "
                f"and the recorded AMS need only {need}. Set pool_ams: 4 to "
                f"silence this. AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={H} lets "
                f"the band shrink to {need} at the next RESTART, and erases what "
                f"it learned.")

    class _DescGcode(RecordingGcode):
        """RecordingGcode that also keeps each mux registration's help text."""

        def __init__(self) -> None:
            """Start with no registrations."""
            super().__init__()
            self.descs: Dict[Tuple[str, Any], Optional[str]] = {}

        def register_mux_command(self, cmd: str, key: str, value: Any,
                                 func: Any, desc: Optional[str] = None) -> None:
            """
            Record the help text, then register as Klipper does.

            :param cmd: command name
            :param key: the selecting parameter
            :param value: this handler's value of ``key``
            :param func: handler
            :param desc: help text
            """
            super().register_mux_command(cmd, key, value, func, desc=desc)
            self.descs[(cmd, value)] = desc

    class _FakePins:
        """Klipper's pins object as the buffer chip registration uses it."""

        def __init__(self) -> None:
            """Start with no chips."""
            self.chips: Dict[str, Any] = {}

        def register_chip(self, name: str, chip: Any) -> None:
            """
            :param name: the chip name
            :param chip: the chip
            """
            self.chips[name] = chip

    # fabricated shape

    def test_an_ht_is_unit_lane_hub_sensor_in_that_order(self, tmp_path):
        m = _bbx_boot(tmp_path)
        assert _bbx_loaded(m) == [
            "AFC_BambuAMS Bambu_AMS_HT_1",
            "AFC_lane lane24",
            "AFC_hub Bambu_AMS_HT_1",
            "temperature_sensor Bambu_AMS_HT_1",
        ]
        assert m.units == [{"model": "ht", "uid": HT_UID}]
        assert m._lane_map == {HT_UID: (24, 1)}
        assert m._name_map == {HT_UID: "Bambu_AMS_HT_1"}
        _bbx_quiet(m.printer)

    def test_a_boxed_unit_takes_four_lanes(self, tmp_path):
        m = _bbx_boot(tmp_path, roster="boxed:AAAABBBBCCCCDDDD")
        assert _bbx_loaded(m) == [
            "AFC_BambuAMS Bambu_AMS_1",
            "AFC_lane lane24", "AFC_lane lane25",
            "AFC_lane lane26", "AFC_lane lane27",
            "AFC_hub Bambu_AMS_1",
            "temperature_sensor Bambu_AMS_1",
        ]
        assert _bbx_keys(m, "AFC_lane lane27") == {"unit": "Bambu_AMS_1:4",
                                                   "unassigned": "True"}
        assert m._lane_map == {"AAAABBBBCCCCDDDD": (24, 4)}
        _bbx_quiet(m.printer)

    class _BareAfc:
        """AFC with nothing but its logger: no var-file queue, no spool."""

        def __init__(self, logger: Any) -> None:
            """
            :param logger: the logger AFC hands its units
            """
            self.logger = logger

    @staticmethod
    def _refuse_serial(monkeypatch: pytest.MonkeyPatch) -> None:
        """
        Make pyserial refuse every port, as when no bridge is plugged in.

        :param monkeypatch: undoes the stand-in after the test
        """
        fake = types.ModuleType("serial")

        def _refuse(port: str, *args: Any, **kwargs: Any) -> Any:
            raise OSError(f"could not open port {port}")

        fake.Serial = _refuse
        monkeypatch.setitem(sys.modules, "serial", fake)

    @pytest.mark.parametrize("pool", [{"pool_ams": 2}, {"roster": ""}],
                             ids=["pool", "bridge-only-scout"])
    def test_a_negative_dry_max_temp_is_unset_and_never_stops_a_start(
            self, tmp_path, monkeypatch, pool):
        # The start runs on as little as AFC can offer at ready: a bare AFC
        # and no bridge on the port.
        printer = make_printer(monkeypatch=monkeypatch)
        printer._afc = self._BareAfc(printer.afc.logger)
        self._refuse_serial(monkeypatch)
        opts: Dict[str, Any] = dict(roster="", buffer="Bamb_1",
                                    dry_max_temp=-1)
        opts.update(pool)
        m = self._direct(tmp_path, self._MinvalConfig, printer, **opts)
        note = ("AFC_BridgeBox chain1: dry_max_temp is -1, below 0, so it is "
                "ignored and each heated unit dries up to its model's own "
                "ceiling. Remove it or set a positive value to silence this.")
        assert m.dry_max_temp == 0
        assert m._dry_note == note
        assert _bbx_ready(m) == [
            ("warning", note),
            ("debug", "AFC_BridgeBox chain1: AFC has no var-file write "
                      "queue; the records of a bay no unit is claimed onto "
                      "are held for this session only"),
            ("debug", "AFC_BridgeBox chain1: AFC has no mapping reset to "
                      "wrap; AFC_RESET_MAPPING numbers the Bambu lanes as "
                      "any other lane"),
            ("warning", "AFC_BridgeBox chain1: could not open the bridge for "
                        "scouting (could not open port "
                        "/dev/serial/by-id/usb-chain1-if00); will keep "
                        "retrying."),
            ("info", "AFC_BridgeBox chain1: no roster configured and no "
                     "recorded roster -- scouting the chain. Detected units "
                     "are recorded in AFC_BridgeBox.cfg; RESTART then "
                     "enrolls them.")]
        assert m._dry_note == ""
        # The bridge stays registered for the watch to retry; its port never
        # opened, so no reader thread runs.
        bridge = live_bridges()[m.serial_port]
        assert (type(bridge), bridge._serial, bridge._thread) == (
            BambuBridge, None, None)

    def test_each_family_numbers_independently(self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"ht:{A}, ht:{B}, boxed:{C}")
        units = [s for s in _bbx_loaded(m) if s.startswith("AFC_BambuAMS")]
        assert units == ["AFC_BambuAMS Bambu_AMS_1",
                         "AFC_BambuAMS Bambu_AMS_HT_1",
                         "AFC_BambuAMS Bambu_AMS_HT_2"]
        assert m._name_map == {A: "Bambu_AMS_HT_1", B: "Bambu_AMS_HT_2",
                               C: "Bambu_AMS_1"}
        _bbx_quiet(m.printer)

    def test_a_second_chain_can_set_its_own_prefix(self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"ht:{A}", unit_prefix="Bambu_AMS_B")
        units = [s for s in _bbx_loaded(m) if s.startswith("AFC_BambuAMS")]
        assert units == ["AFC_BambuAMS Bambu_AMS_B_HT_1"]
        assert m.unit_prefix == "Bambu_AMS_B"
        _bbx_quiet(m.printer)

    # refusals

    def test_a_name_collision_refuses_before_loading_anything(self, tmp_path):
        printer = make_printer()
        printer.add_object("AFC_lane lane24", object())
        with pytest.raises(configparser.Error) as err:
            _bbx_boot(tmp_path, printer)
        assert str(err.value) == (
            "[AFC_BridgeBox chain1] would fabricate [AFC_lane lane24] but it "
            "already exists in the config -- remove one")
        assert printer.loaded == []
        _bbx_quiet(printer)

    def test_an_empty_roster_scouts_rather_than_halting(self, tmp_path):
        m = _bbx_boot(tmp_path, roster="")
        assert m.printer.loaded == []
        assert m.units == []
        assert m._fabricated_names == set()
        assert m._roster_source == "scout"
        assert m.get_status()["roster_source"] == "scout"
        _bbx_quiet(m.printer)

    def test_a_malformed_entry_names_itself(self, tmp_path):
        printer = make_printer()
        with pytest.raises(configparser.Error) as err:
            _bbx_boot(tmp_path, printer, roster=HT_UID)
        assert str(err.value) == (
            f"[AFC_BridgeBox chain1] roster:: roster entry '{HT_UID}' is not "
            f"<model>:<unit_uid>")
        assert printer.loaded == []
        _bbx_quiet(printer)

    def test_an_unknown_model_lists_the_known_ones(self, tmp_path):
        printer = make_printer()
        with pytest.raises(configparser.Error) as err:
            _bbx_boot(tmp_path, printer, roster="ams9:1111222233334444")
        assert str(err.value) == (
            "[AFC_BridgeBox chain1] roster:: roster entry "
            "'ams9:1111222233334444': unknown model 'ams9' (one of ['ams1', "
            "'ams2', 'boxed', 'ht', 'lite'])")
        _bbx_quiet(printer)

    def test_a_duplicated_uid_is_refused(self, tmp_path):
        printer = make_printer()
        with pytest.raises(configparser.Error) as err:
            _bbx_boot(tmp_path, printer, roster=f"ht:{A}, boxed:aaaa")
        assert str(err.value) == (
            f"[AFC_BridgeBox chain1] roster:: roster lists uid {A} twice")
        _bbx_quiet(printer)

    def test_a_malformed_recorded_roster_names_the_state_file(self, tmp_path):
        record_chain_state(tmp_path, roster="ams9:1111")
        printer = make_printer()
        with pytest.raises(configparser.Error) as err:
            _bbx_boot(tmp_path, printer, roster="")
        state = tmp_path / "AFC_BridgeBox.cfg"
        assert str(err.value) == (
            f"[AFC_BridgeBox chain1] state file {state}: roster entry "
            f"'ams9:1111': unknown model 'ams9' (one of ['ams1', 'ams2', "
            f"'boxed', 'ht', 'lite'])")
        _bbx_quiet(printer)

    # fold and sweep of AFC_auto_vars.cfg

    AUTOV_HEADER = ("# This file is autogenerated and updated when variables "
                    "are not in your normal AFC config files\n\n")

    class _ReadTrackingConfig(BambuConfig):
        """BambuConfig whose getsection sections record what is read."""

        def __init__(self, *args: Any, reads: Optional[set] = None,
                     track: bool = False, **kwargs: Any) -> None:
            """
            :param reads: where (section, option) reads are recorded
            :param track: record this section's reads
            """
            super().__init__(*args, **kwargs)
            self.reads = reads if reads is not None else set()
            self.track = track

        def getsection(self, section: str) -> BambuConfig:
            """
            :param section: another section of the merged config
            :return BambuConfig: that section, recording its reads
            """
            values = dict(self.fileconfig.items(section))
            return type(self)(section, self.get_printer(), values,
                              fileconfig=self.fileconfig, reads=self.reads,
                              track=True)

        def get(self, option: str, *args: Any) -> Any:
            """
            :param option: the option read
            :return Any: its value
            """
            if self.track:
                self.reads.add((self.get_name().lower(), option.lower()))
            return super().get(option, *args)

    def test_the_folded_section_is_swept_out_of_auto_vars(self, tmp_path):
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text(self.AUTOV)
        m = _bbx_boot(tmp_path)
        assert autov.read_text() == self.AUTOV_HEADER
        unit = _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_HT_1")
        assert unit["afc_bowden_length"] == "3632.0"
        assert unit["afc_unload_bowden_length"] == "3632.0"
        assert m._learned_for(HT_UID) == {
            "afc_bowden_length": "3632.0",
            "afc_unload_bowden_length": "3632.0"}
        assert m._learned_notes == []
        _bbx_quiet(m.printer)

    def test_an_orphan_for_a_renamed_unit_is_deleted(self, tmp_path):
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text("[AFC_BambuAMS chain1_ht0_3331]\n"
                         "afc_bowden_length : 3632.0\n")
        printer = make_printer()
        printer.add_section("AFC_BambuAMS chain1_ht0_3331",
                            {"afc_bowden_length": "3632.0"})
        m = _bbx_boot(tmp_path, printer)
        assert autov.read_text() == self.AUTOV_HEADER
        assert m._learned_notes == []
        assert printer.lookup_object("AFC_BambuAMS chain1_ht0_3331") is None
        _bbx_quiet(printer)

    def test_a_handwritten_units_entry_is_left_alone(self, tmp_path):
        entry = ("[AFC_BambuAMS BambuAMS_HT]\n"
                 "afc_bowden_length : 2100.0\n")
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text(entry)
        printer = make_printer()
        printer.add_section("AFC_BambuAMS BambuAMS_HT",
                            {"afc_bowden_length": "2100.0",
                             "serial_port": "/dev/real"})
        m = _bbx_boot(tmp_path, printer)
        assert autov.read_text() == entry
        assert m._learned_notes == []
        _bbx_quiet(printer)

    def test_no_files_at_all_is_a_clean_noop(self, tmp_path):
        m = _bbx_boot(tmp_path)
        assert not (tmp_path / "AFC_auto_vars.cfg").exists()
        assert not (tmp_path / "AFC_BridgeBox_chain1.vars").exists()
        assert m._learned_notes == []
        assert m._learned_for(HT_UID) == {}
        _bbx_quiet(m.printer)

    def test_a_lane_and_hub_no_longer_built_are_moved_and_stood_in_for(
            self, tmp_path):
        record_chain_state(tmp_path, roster="boxed:A1, ht:H1",
                           lane_base="24", lane_map="A1:24:4, H1:56:1",
                           name_map="A1:Bambu_AMS_1, H1:Bambu_AMS_HT_1")
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text("[AFC_lane lane56]\nremember_spool : False\n\n"
                         "[AFC_hub Bambu_AMS_5]\nafc_bowden_length : 1900\n\n"
                         "[AFC_lane lane99]\nremember_spool : False\n")
        printer = make_printer()
        printer.add_section("AFC_lane lane56", {"remember_spool": "False"})
        printer.add_section("AFC_hub Bambu_AMS_5",
                            {"afc_bowden_length": "1900"})
        printer.add_section("AFC_lane lane99", {"remember_spool": "False"})
        reads: set = set()

        def config(*args: Any) -> BambuConfig:
            return self._ReadTrackingConfig(*args, reads=reads)

        m = self._direct(tmp_path, config, printer, roster="",
                         buffer="Bamb_1", pool_ams=8, pool_ht=2)
        assert m._lane_map["H1"] == (40, 1)
        assert autov.read_text() == (self.AUTOV_HEADER
                                     + "[AFC_lane lane99]\n"
                                       "remember_spool : False\n\n")
        assert m._state_get("AFC_lane lane56", "remember_spool") == "False"
        assert m._state_get("AFC_hub Bambu_AMS_5",
                            "afc_bowden_length") == "1900"
        for name in ("AFC_lane lane56", "AFC_hub Bambu_AMS_5"):
            assert isinstance(printer.lookup_object(name), _SweptSection)
        assert printer.lookup_object("AFC_lane lane99") is None
        assert reads == {("afc_lane lane56", "remember_spool"),
                         ("afc_hub bambu_ams_5", "afc_bowden_length")}
        assert m._learned_notes == [
            (False, "auto_vars [AFC_lane lane56] moved to "
                    "AFC_BridgeBox.cfg: no chain builds that section now"),
            (False, "auto_vars [AFC_hub Bambu_AMS_5] moved to "
                    "AFC_BridgeBox.cfg: no chain builds that section now")]
        assert 56 not in m._loaded_lane_numbers()
        _bbx_quiet(printer)

    # scouting

    def test_scout_mode_still_provides_the_buffer_pin_chip(self, tmp_path):
        printer = make_printer()
        pins = self._FakePins()
        printer.add_object("pins", pins)
        _bbx_boot(tmp_path, printer, roster="")
        assert list(pins.chips) == ["bambu_buffer"]
        chips = printer._bambu_buffer_chips
        assert list(chips) == ["bambu_buffer"]
        assert chips["bambu_buffer"]._unit.fps_buffer_value() is None
        assert chips["bambu_buffer"]._unit.scout_stub is True
        _bbx_quiet(printer)

    class _RefusingPins(_FakePins):
        """Klipper's pins object refusing every chip, as for a taken name."""

        def register_chip(self, name: str, chip: Any) -> None:
            """
            :param name: the chip name
            :param chip: the chip
            :raises configparser.Error: always
            """
            error_str = f"Duplicate chip name '{name}'"
            raise configparser.Error(error_str)

    def test_a_scout_starts_when_its_pin_chip_is_refused(self, tmp_path):
        printer = make_printer()
        printer.add_object("pins", self._RefusingPins())
        m = _bbx_boot(tmp_path, printer, roster="")
        assert m._roster_source == "scout"
        assert m.units == []
        assert printer.loaded == []
        assert printer._bambu_buffer_chips == {}
        assert printer._event_handlers["klippy:ready"] == [m._scout_ready]
        _bbx_quiet(printer)

    def test_enrolled_mode_does_not_register_the_stub(self, tmp_path):
        printer = make_printer()
        pins = self._FakePins()
        printer.add_object("pins", pins)
        _bbx_boot(tmp_path, printer)
        assert pins.chips == {}
        assert not hasattr(printer, "_bambu_buffer_chips")
        _bbx_quiet(printer)

    def test_no_roster_anywhere_scouts_instead_of_halting(
            self, tmp_path, monkeypatch):
        printer = make_printer(monkeypatch=monkeypatch)
        pins = self._FakePins()
        printer.add_object("pins", pins)
        m = _bbx_boot(tmp_path, printer, roster="")
        assert printer.loaded == []
        assert m.get_status()["roster_source"] == "scout"
        chip = printer._bambu_buffer_chips["bambu_buffer"]
        assert printer._event_handlers["klippy:ready"] == [
            chip._start, m._scout_ready]
        # At ready it watches the bridge already on its port.
        bridge = FakeBridge()
        live_bridges()[m.serial_port] = bridge
        assert _bbx_ready(m) == [
            ("info", "AFC_BridgeBox chain1: no roster configured and no "
                     "recorded roster -- scouting the chain. Detected units "
                     "are recorded in AFC_BridgeBox.cfg; RESTART then "
                     "enrolls them.")]
        assert m._bridge is bridge
        assert live_bridges() == {m.serial_port: bridge}

    def test_a_bay_name_colliding_with_a_foreign_unit_is_refused(
            self, tmp_path):
        printer = make_printer()
        printer.add_section("AFC_lane lane4", {"unit": "AMS_1:1"})
        with pytest.raises(configparser.Error) as err:
            _bbx_boot(tmp_path, printer, roster="", pool_ams=1, pool_ht=0,
                      ams_names="AMS_1")
        assert str(err.value) == (
            "[AFC_BridgeBox chain1] pool bay name 'AMS_1' collides with an "
            "existing AFC unit of the same name -- rename that ams_names / "
            "ht_names entry to something unique. AFC identifies units by "
            "name, so a duplicate double-registers that unit's lanes and "
            "stops Klipper from starting.")
        assert printer.loaded == []
        _bbx_quiet(printer)

    def test_a_unique_bay_name_beside_a_foreign_unit_is_fine(self, tmp_path):
        printer = make_printer()
        printer.add_section("AFC_lane lane4", {"unit": "AMS_1:1"})
        m = _bbx_boot(tmp_path, printer, roster="", pool_ams=1, pool_ht=0,
                      ams_names="AMS1_1")
        assert _bbx_loaded(m) == ["AFC_BambuAMS AMS1_1",
                                  *_bbx_lanes_span(24, 27), "AFC_hub AMS1_1"]
        _bbx_quiet(printer)

    def test_a_pool_scout_fabricates_the_empty_named_pool(self, tmp_path):
        m = _bbx_boot(tmp_path, roster="", pool_ams=2, pool_ht=1,
                      ams_names="One, Two", ht_names="Hot")
        assert m.get_status()["roster_source"] == "scout"
        units = [s for s in _bbx_loaded(m) if s.startswith("AFC_BambuAMS")]
        assert units == ["AFC_BambuAMS One", "AFC_BambuAMS Two",
                         "AFC_BambuAMS Hot"]
        assert m._pool_units == [
            {"name": "One", "lanes": ["lane24", "lane25", "lane26",
                                      "lane27"],
             "family": "ams", "uid": None, "bound": None, "spare": True},
            {"name": "Two", "lanes": ["lane28", "lane29", "lane30",
                                      "lane31"],
             "family": "ams", "uid": None, "bound": None, "spare": True},
            {"name": "Hot", "lanes": ["lane32"], "family": "ht",
             "uid": None, "bound": None, "spare": True}]
        assert m.printer._event_handlers["klippy:ready"] == [m._scout_ready]
        _bbx_quiet(m.printer)

    def test_a_pool_scout_of_ht_bays_alone_fabricates_them(self, tmp_path):
        # pool_ht alone makes a pool scout, as pool_ams alone does: the spare
        # is fabricated, where a bridge-only scout fabricates nothing.
        m = _bbx_boot(tmp_path, roster="", pool_ams=0, pool_ht=1)
        assert m._roster_source == "scout"
        assert m.units == []
        assert _bbx_loaded(m) == ["AFC_BambuAMS Bambu_AMS_HT_1",
                                  "AFC_lane lane24", "AFC_hub Bambu_AMS_HT_1"]
        assert m._pool_units == [
            {"name": "Bambu_AMS_HT_1", "lanes": ["lane24"], "family": "ht",
             "uid": None, "bound": None, "spare": True}]
        _bbx_quiet(m.printer)

    def test_a_roster_file_from_a_previous_scout_enrolls(self, tmp_path):
        legacy = tmp_path / "AFC_BridgeBox_chain1.roster"
        legacy.write_text("# chain detected by [AFC_BridgeBox chain1]\n"
                          f"ht:{HT_UID}\n")
        m = _bbx_boot(tmp_path, roster="")
        assert _bbx_loaded(m)[0] == "AFC_BambuAMS Bambu_AMS_HT_1"
        assert m.get_status()["roster_source"] == "file"
        assert m._state_get(SEC, "roster") == f"ht:{HT_UID}"
        assert not legacy.exists()
        _bbx_quiet(m.printer)

    def test_the_option_overrides_the_file(self, tmp_path):
        (tmp_path / "AFC_BridgeBox_chain1.roster").write_text(
            "ht:AAAABBBBCCCCDDDD\n")
        m = _bbx_boot(tmp_path, roster="boxed:1111222233334444")
        units = [s for s in _bbx_loaded(m) if s.startswith("AFC_BambuAMS")]
        assert units == ["AFC_BambuAMS Bambu_AMS_1"]
        assert m.get_status()["roster_source"] == "option"
        assert m.units == [{"model": "boxed", "uid": "1111222233334444"}]
        _bbx_quiet(m.printer)

    def test_enrolled_units_still_watch_for_newcomers(self, tmp_path):
        m = _bbx_boot(tmp_path)
        assert m.printer._event_handlers["klippy:ready"] == [m._scout_ready]
        _bbx_quiet(m.printer)

    def test_the_watch_registers_after_the_fabricated_units(self, tmp_path):
        m = _bbx_boot(tmp_path, make_printer(fabricate=True))
        unit = m.printer.lookup_object("AFC_BambuAMS Bambu_AMS_HT_1")
        assert isinstance(unit, afcBambuAMS)
        handlers = m.printer._event_handlers["klippy:ready"]
        assert handlers[-1] == m._scout_ready
        assert handlers.index(unit._handle_ready) < len(handlers) - 1
        _bbx_quiet(m.printer)

    # the automatic lane base

    class _BlindConfig(BambuConfig):
        """BambuConfig with no merged fileconfig to look at."""

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            """Build it, then drop the fileconfig."""
            super().__init__(*args, **kwargs)
            self.fileconfig = None

    @staticmethod
    def _declare(printer: BambuPrinter, *sections: str) -> None:
        """
        :param printer: the printer
        :param sections: sections the merged config declares, empty
        """
        for section in sections:
            printer.add_section(section, {})

    def _declared_lanes_printer(self) -> BambuPrinter:
        """:return BambuPrinter: one declaring lanes 4, 7, 8 and 11"""
        printer = make_printer()
        self._declare(printer, "AFC_stepper lane8", "afc_stepper lane11",
                      "AFC_lane lane4", "AFC_lane lane7", "AFC_hub Turtle_1")
        return printer

    def test_default_is_one_past_the_highest_declared_lane(self, tmp_path):
        m = _bbx_boot(tmp_path, self._declared_lanes_printer(), lane_base=0)
        assert m.lane_base == 12
        assert m._resolved_base == 12
        assert _bbx_loaded(m)[1] == "AFC_lane lane12"
        assert m._state_get(SEC, "lane_base") == "12"
        _bbx_quiet(m.printer)

    def test_the_computed_base_is_remembered(self, tmp_path):
        _bbx_boot(tmp_path, self._declared_lanes_printer(), lane_base=0)
        grown = make_printer()
        self._declare(grown, "AFC_stepper lane30")
        m2 = _bbx_boot(tmp_path, grown, lane_base=0)
        assert m2.lane_base == 12
        assert not hasattr(m2, "_resolved_base")
        _bbx_quiet(grown)

    def test_an_explicit_option_still_wins(self, tmp_path):
        m = _bbx_boot(tmp_path, self._declared_lanes_printer(), lane_base=24)
        assert m.lane_base == 24
        assert m._lane_base_set is True
        assert not hasattr(m, "_resolved_base")
        assert m._state_get(SEC, "lane_base") is None
        _bbx_quiet(m.printer)

    def test_toolchanger_extruders_set_the_base(self, tmp_path):
        printer = make_printer()
        self._declare(printer, "AFC_extruder e0", "AFC_extruder e1",
                      "AFC_extruder e2", "AFC_extruder e3")
        m = _bbx_boot(tmp_path, printer, lane_base=0)
        assert m.lane_base == 4
        _bbx_quiet(printer)

    def test_a_mapped_tool_sets_the_base(self, tmp_path):
        printer = make_printer()
        printer.add_section("AFC_lane spool_a", {"map": "T6"})
        printer.add_section("AFC_extruder extruder", {})
        m = _bbx_boot(tmp_path, printer, lane_base=0)
        assert m.lane_base == 7
        _bbx_quiet(printer)

    def test_a_single_unnumbered_extruder_keeps_the_fallback(self, tmp_path):
        printer = make_printer()
        self._declare(printer, "AFC_extruder extruder")
        m = _bbx_boot(tmp_path, printer, lane_base=0)
        assert m.lane_base == 24
        assert m._resolved_base == 24
        _bbx_quiet(printer)

    def test_no_visible_config_falls_back_to_24(self, tmp_path):
        m = self._direct(tmp_path, self._BlindConfig, lane_base=0,
                         buffer="Bamb_1")
        assert m.lane_base == 24
        assert m._resolved_base == 24
        assert m._declared_lanes == set()
        assert m._later_lanes == {}
        assert _bbx_loaded(m)[1] == "AFC_lane lane24"
        _bbx_quiet(m.printer)

    # the chain's buffer

    def test_the_buffer_loads_after_the_units(self, tmp_path):
        m = _bbx_boot(tmp_path, buffer=None)
        assert _bbx_loaded(m) == [
            "AFC_BambuAMS Bambu_AMS_HT_1", "AFC_lane lane24",
            "AFC_hub Bambu_AMS_HT_1", "temperature_sensor Bambu_AMS_HT_1",
            "AFC_buffer Bambu_AMS_Buffer"]
        assert m.buffer == "Bambu_AMS_Buffer"
        assert m._fabricate_buffer is True
        _bbx_quiet(m.printer)

    def test_a_named_external_buffer_suppresses_fabrication(self, tmp_path):
        m = _bbx_boot(tmp_path, buffer="Bamb_1")
        assert [s for s in _bbx_loaded(m) if s.startswith("AFC_buffer")] == []
        assert _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_HT_1")["buffer"] == \
            "Bamb_1"
        assert m._fabricate_buffer is False
        _bbx_quiet(m.printer)

    def test_an_adopted_buffer_keeps_its_own_type(self, tmp_path):
        printer = make_printer()
        printer.add_section("AFC_buffer Bamb_1",
                            {"adc_pin": "bambu_buffer:fps", "type": "FPS_PSF"})
        m = _bbx_boot(tmp_path, printer, buffer=None, buffer_type="bambu")
        assert [s for s in _bbx_loaded(m) if s.startswith("AFC_buffer")] == []
        assert m.buffer == "Bamb_1"
        assert m._fabricate_buffer is False
        _bbx_quiet(printer)

    def test_an_existing_chip_buffer_is_adopted_not_rivalled(self, tmp_path):
        printer = make_printer()
        printer.add_section("AFC_buffer Bamb_1",
                            {"adc_pin": "bambu_buffer:fps", "type": "FPS_PSF"})
        m = _bbx_boot(tmp_path, printer, buffer=None)
        assert [s for s in _bbx_loaded(m) if s.startswith("AFC_buffer")] == []
        assert _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_HT_1")["buffer"] == \
            "Bamb_1"
        assert m.buffer == "Bamb_1"
        _bbx_quiet(printer)

    def test_a_buffer_on_another_chip_is_not_adopted(self, tmp_path):
        printer = make_printer()
        printer.add_section("AFC_buffer fps_buffer1", {"adc_pin": "fps:PA2"})
        m = _bbx_boot(tmp_path, printer, buffer=None)
        assert _bbx_loaded(m)[-1] == "AFC_buffer Bambu_AMS_Buffer"
        assert m.buffer == "Bambu_AMS_Buffer"
        _bbx_quiet(printer)

    # where the state lives

    def test_state_lands_in_the_file_declaring_the_section(self, tmp_path):
        cfgdir = tmp_path / "config"
        (cfgdir / "AFC").mkdir(parents=True)
        (cfgdir / "printer.cfg").write_text("[include AFC/*.cfg]\n")
        mine = cfgdir / "AFC" / "whatever_i_called_it.cfg"
        mine.write_text("[AFC_BridgeBox chain1]\n"
                        "serial_port: /dev/x\nextruder: extruder\n")
        printer = make_printer()
        printer.start_args = {"config_file": str(cfgdir / "printer.cfg")}
        m = _bbx_boot(tmp_path, printer, roster=f"ht:{A}", state_file=None)
        assert m.state_file == str(mine)
        m._state_set({SEC: {"roster": f"ht:{A}"}})
        assert mine.read_text() == (
            "[AFC_BridgeBox chain1]\n"
            "serial_port: /dev/x\nextruder: extruder\n\n"
            "#~# --- AFC_BridgeBox managed state -- everything below is "
            "auto-written ---\n"
            "#~# [AFC_BridgeBox chain1]\n"
            f"#~# lane_map : {A}:24:1\n"
            f"#~# name_map : {A}:Bambu_AMS_HT_1\n"
            f"#~# roster : ht:{A}\n"
            "#~# \n")
        assert m._state_get(SEC, "roster") == f"ht:{A}"
        _bbx_quiet(printer)

    def test_unfindable_section_falls_back_to_the_default_path(self, tmp_path):
        m = _bbx_boot(tmp_path, roster="", state_file=None)
        assert m.state_file == str(
            pathlib.Path.home() / "printer_data/config/AFC/AFC_BridgeBox.cfg")
        assert m.printer.loaded == []
        _bbx_quiet(m.printer)

    def test_an_explicit_state_file_still_wins(self, tmp_path):
        printer = make_printer()
        printer.start_args = {"config_file": str(tmp_path / "printer.cfg")}
        m = _bbx_boot(tmp_path, printer, roster=f"ht:{A}",
                      state_file=str(tmp_path / "pinned.cfg"))
        assert m.state_file == str(tmp_path / "pinned.cfg")
        assert m._state_get(SEC, "name_map") == f"{A}:Bambu_AMS_HT_1"
        _bbx_quiet(printer)

    # operator overrides

    def test_an_orphan_override_is_ignored_not_fatal(self, tmp_path):
        printer = make_printer()
        printer.add_section("AFC_BridgeBox Bambu_AMS_9",
                            {"measure_on_insert": "False"})
        m = _bbx_boot(tmp_path, printer)
        assert _bbx_loaded(m)[0] == "AFC_BambuAMS Bambu_AMS_HT_1"
        assert m._unmatched_overrides == ["AFC_BridgeBox Bambu_AMS_9"]
        assert _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_HT_1")[
            "measure_on_insert"] == "True"
        _bbx_quiet(printer)

    # lane and name tombstones

    def test_families_number_independently_through_the_map(self, tmp_path):
        _bbx_boot(tmp_path, roster=f"ht:{A}, boxed:{B}", pool_ams=2,
                  pool_ht=1)
        m2 = _bbx_boot(tmp_path, roster=f"ht:{A}, boxed:{B}, boxed:{C}",
                       pool_ams=2, pool_ht=1)
        names = [s for s in _bbx_loaded(m2) if s.startswith("AFC_BambuAMS")]
        assert names == ["AFC_BambuAMS Bambu_AMS_1", "AFC_BambuAMS Bambu_AMS_2",
                         "AFC_BambuAMS Bambu_AMS_HT_1"]
        assert m2._name_map == {A: "Bambu_AMS_HT_1", B: "Bambu_AMS_1",
                                C: "Bambu_AMS_2"}
        _bbx_quiet(m2.printer)

    @pytest.mark.parametrize("bad", ["AAAA:24:400", "AAAA:24:2",
                                     "AAAA:-4:4"])
    def test_a_lane_map_entry_no_unit_could_have_is_dropped(self, tmp_path,
                                                            bad):
        record_chain_state(tmp_path, lane_map=f"{bad}, HHHH:40:1",
                           name_map="AAAA:Bambu_AMS_1, HHHH:Bambu_AMS_HT_1")
        m = _bbx_boot(tmp_path, roster="boxed:AAAA, ht:HHHH", **POOL)
        assert m._lanes_at_boot == {H: (40, 1)}
        assert m._lane_map == {A: (24, 4), H: (40, 1)}
        assert m._layout_notes == []
        _bbx_quiet(m.printer)

    def test_the_maps_live_in_the_managed_comment_block(self, tmp_path):
        _bbx_boot(tmp_path, roster=f"ht:{A}")
        assert (tmp_path / "AFC_BridgeBox.cfg").read_text() == (
            "# AFC_BridgeBox: put your [AFC_BridgeBox <name>] section here "
            "(serial_port + extruder is enough).\n"
            "# The block below is maintained by the module.\n\n"
            "#~# --- AFC_BridgeBox managed state -- everything below is "
            "auto-written ---\n"
            "#~# [AFC_BridgeBox chain1]\n"
            f"#~# lane_map : {A}:24:1\n"
            f"#~# name_map : {A}:Bambu_AMS_HT_1\n"
            "#~# \n")
        m2 = _bbx_boot(tmp_path, roster=f"ht:{A}")
        assert m2._lanes_at_boot == {A: (24, 1)}
        assert m2._pins_at_boot == {A: "Bambu_AMS_HT_1"}
        assert m2._lane_map == {A: (24, 1)}
        assert m2._name_map == {A: "Bambu_AMS_HT_1"}
        _bbx_quiet(m2.printer)

    def test_the_default_is_about_a_second(self, tmp_path):
        m = _bbx_boot(tmp_path)
        assert m.hotplug_poll == 1.0
        _bbx_quiet(m.printer)

    def test_all_ams_share_one_family_ht_separate(self, tmp_path):
        m = _bbx_boot(tmp_path,
                      roster=f"ams2:{A}, ams1:{B}, boxed:{C}, ht:{D}")
        names = [s for s in _bbx_loaded(m) if s.startswith("AFC_BambuAMS")]
        assert names == ["AFC_BambuAMS Bambu_AMS_1", "AFC_BambuAMS Bambu_AMS_2",
                         "AFC_BambuAMS Bambu_AMS_3",
                         "AFC_BambuAMS Bambu_AMS_HT_1"]
        _bbx_quiet(m.printer)

    # the operator commands

    HELP = {
        "AFC_BRIDGEBOX_FORGET": (
            "Release a departed unit's lane numbers and unit name for reuse "
            "and erase its learned values and saved lane records: "
            "AFC_BRIDGEBOX_FORGET [CHAIN=<chain>] UID=<uid> (or NAME=<unit "
            "name>) [FORCE=1]. With no argument, pops a picker of every "
            "recorded unit with a Forget button each."),
        "AFC_BRIDGEBOX_ASSIGN": (
            "Pin a detected unit's UID onto a named pool bay, live: "
            "AFC_BRIDGEBOX_ASSIGN [CHAIN=<chain>] UID=<uid> NAME=<bay name> "
            "[FORCE=1] (or UID=<uid> alone to pop the bay picker). Refuses an "
            "occupied bay or one saved for another unit -- UNASSIGN it first "
            "-- with roster: set, a uid it does not list, and moving a unit "
            "off a bay with a lane AFC records as loaded to the toolhead "
            "(FORCE=1 clears that record)."),
        "AFC_BRIDGEBOX_UNASSIGN": (
            "Unlink a UID from its pool bay (keeps learned values), live: "
            "AFC_BRIDGEBOX_UNASSIGN [CHAIN=<chain>] UID=<uid> (or NAME=<bay "
            "name>) [FORCE=1]"),
        "AFC_BRIDGEBOX_BAYS": (
            "Pop the bay manager: every pool bay, its occupant, an Unassign "
            "button for each occupied bay, and a Replace button for a unit "
            "waiting for a bay -- AFC_BRIDGEBOX_BAYS [CHAIN=<chain>]"),
        "AFC_BRIDGEBOX_REPLACE": (
            "Forget an offline unit and put a new unit on its bay, live: "
            "AFC_BRIDGEBOX_REPLACE [CHAIN=<chain>] UID=<new uid> OLD=<old uid "
            "or bay name> [FORCE=1]. Without OLD, pops a picker of the bays "
            "held for offline units."),
    }

    @staticmethod
    def _bridgebox_mux(printer: BambuPrinter) -> Dict[str, Any]:
        """
        :param printer: the printer
        :return dict: the AFC_BRIDGEBOX_* mux registrations on its gcode
        """
        return {cmd: reg for cmd, reg in printer.gcode.mux_commands.items()
                if cmd.startswith("AFC_BRIDGEBOX_")}

    def test_the_command_registers_muxed_with_a_default(self, tmp_path):
        m = _bbx_boot(tmp_path)
        handlers = {"AFC_BRIDGEBOX_FORGET": m.cmd_AFC_BRIDGEBOX_FORGET,
                    "AFC_BRIDGEBOX_ASSIGN": m.cmd_AFC_BRIDGEBOX_ASSIGN,
                    "AFC_BRIDGEBOX_UNASSIGN": m.cmd_AFC_BRIDGEBOX_UNASSIGN,
                    "AFC_BRIDGEBOX_BAYS": m.cmd_AFC_BRIDGEBOX_BAYS,
                    "AFC_BRIDGEBOX_REPLACE": m.cmd_AFC_BRIDGEBOX_REPLACE}
        assert self._bridgebox_mux(m.printer) == {
            cmd: ("CHAIN", {"chain1": fn, None: fn})
            for cmd, fn in handlers.items()}
        _bbx_quiet(m.printer)

    def test_the_replace_command_registers_muxed_with_a_default(
            self, tmp_path):
        printer = make_printer()
        m1, m2 = _bbx_two_chains(tmp_path, printer)
        assert self._bridgebox_mux(printer)["AFC_BRIDGEBOX_REPLACE"] == (
            "CHAIN", {"chain1": m1.cmd_AFC_BRIDGEBOX_REPLACE,
                      None: m1.cmd_AFC_BRIDGEBOX_REPLACE,
                      "chain2": m2.cmd_AFC_BRIDGEBOX_REPLACE})
        _bbx_quiet(printer)

    def test_every_command_help_shows_chain(self, tmp_path):
        printer = make_printer()
        printer.gcode = printer._gcode = self._DescGcode()
        _bbx_boot(tmp_path, printer)
        assert printer.gcode.descs == {
            (cmd, value): text for cmd, text in self.HELP.items()
            for value in ("chain1", None)}
        _bbx_quiet(printer)

    # pool bays and names

    def test_a_spare_bay_wears_its_configured_name(self, tmp_path):
        (tmp_path / "AFC_BridgeBox_chain1.roster").write_text(f"boxed:{A}\n")
        m = _bbx_boot(tmp_path, roster="", pool_ams=2, pool_ht=1,
                      ams_names="Alpha, Bravo", ht_names="Hot")
        assert m._pool_units == [
            {"name": "Alpha", "lanes": ["lane24", "lane25", "lane26",
                                        "lane27"],
             "family": "ams", "uid": A, "bound": None, "spare": False},
            {"name": "Bravo", "lanes": ["lane28", "lane29", "lane30",
                                        "lane31"],
             "family": "ams", "uid": None, "bound": None, "spare": True},
            {"name": "Hot", "lanes": ["lane32"], "family": "ht", "uid": None,
             "bound": None, "spare": True}]
        _bbx_quiet(m.printer)

    POOL2 = {"pool_ams": 2, "pool_ht": 1}

    @staticmethod
    def _lanes(master: afcBridgeBox) -> List[str]:
        """
        :param master: a built master
        :return list: the AFC_lane sections it loaded
        """
        return [s for s in _bbx_loaded(master) if s.startswith("AFC_lane")]

    @staticmethod
    def _state_names(master: afcBridgeBox) -> Dict[str, str]:
        """
        :param master: a built master
        :return dict: uid -> name, as the state file's name_map records
        """
        raw = master._state_get(SEC, "name_map") or ""
        return dict(e.strip().split(":", 1) for e in raw.split(",")
                    if e.strip())

    def test_a_uid_dropped_from_the_roster_option_frees_its_bay(
            self, tmp_path):
        _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{C}", **self.POOL2)
        m = _bbx_boot(tmp_path, roster=f"boxed:{A}", **self.POOL2)
        assert self._lanes(m) == _bbx_lanes_span(24, 32)
        assert _bbx_keys(m, "AFC_lane lane28")["unit"] == "Bambu_AMS_2:1"
        assert _bbx_keys(m, "AFC_lane lane32")["unit"] == "Bambu_AMS_HT_1:1"
        assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_2": "",
                                "Bambu_AMS_HT_1": ""}
        _bbx_quiet(m.printer)

    def test_a_pin_recorded_for_an_unlisted_uid_no_longer_overlaps(
            self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"boxed:{A}", **self.POOL2)
        m._persist_pin(C, 28, 4, "Bambu_AMS_2", "boxed")
        m2 = _bbx_boot(tmp_path, roster=f"boxed:{A}", **self.POOL2)
        assert self._lanes(m2) == _bbx_lanes_span(24, 32)
        assert _bbx_uids(m2) == {"Bambu_AMS_1": A, "Bambu_AMS_2": "",
                                 "Bambu_AMS_HT_1": ""}
        _bbx_quiet(m.printer)
        _bbx_quiet(m2.printer)

    def test_a_new_rostered_uid_takes_the_name_over(self, tmp_path):
        _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{C}", **self.POOL2)
        _bbx_boot(tmp_path, roster=f"boxed:{A}", **self.POOL2)
        m = _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{D}", **self.POOL2)
        assert self._lanes(m) == _bbx_lanes_span(24, 32)
        assert self._state_names(m) == {A: "Bambu_AMS_1", D: "Bambu_AMS_2"}
        assert m._state_get(SEC, "lane_map") == f"{A}:24:4, {D}:28:4"
        _bbx_quiet(m.printer)

    def test_lowering_pool_ams_below_a_recorded_ams_keeps_its_bay(
            self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"boxed:{A}", pool_ams=4, pool_ht=8)
        gcmd = FakeGcmd({"UID": A, "NAME": "Bambu_AMS_4"})
        m.cmd_AFC_BRIDGEBOX_ASSIGN(gcmd)
        m2 = _bbx_boot(tmp_path, roster=f"boxed:{A}", pool_ams=2, pool_ht=8)
        assert self._lanes(m2) == (_bbx_lanes_span(24, 27)
                                   + _bbx_lanes_span(36, 47))
        assert _bbx_keys(m2, "AFC_BambuAMS Bambu_AMS_4")["unit_uid"] == A
        assert _bbx_keys(m2, "AFC_lane lane24")["unit"] == "Bambu_AMS_1:1"
        assert _bbx_keys(m2, "AFC_lane lane40")["unit"] == "Bambu_AMS_HT_1:1"
        assert m2._ams_band == 4
        _bbx_quiet(m2.printer)

    def test_a_renamed_ams_names_entry_redraws_the_unit(self, tmp_path):
        _bbx_boot(tmp_path, roster=f"boxed:{A}", pool_ams=2, pool_ht=0,
                  ams_names="Alpha, Bravo")
        m = _bbx_boot(tmp_path, roster=f"boxed:{A}", pool_ams=2, pool_ht=0,
                      ams_names="Red, Blue")
        assert self._lanes(m) == _bbx_lanes_span(24, 31)
        assert self._state_names(m) == {A: "Red"}
        assert m._layout_notes == [
            f"AFC_BridgeBox chain1: AMS {A} was recorded as Alpha, which no "
            f"ams_names entry or default name gives an AMS. It is now Red, "
            f"and its lanes stay lane24-lane27 (T24-T27). Lane records saved "
            f"under Alpha (spool, material, colour, T# map) do not carry over "
            f"to Red; a tagged spool is read again from its tag. Its learned "
            f"bowden lengths stay with the unit."]
        _bbx_quiet(m.printer)

    # what __init__ must not do

    def test_the_fabrication_shares_it_rather_than_a_throwaway(
            self, tmp_path, monkeypatch):
        configfile = sys.modules["configfile"]
        seen: List[Tuple[str, Any]] = []

        class _Recording(configfile.ConfigWrapper):
            def __init__(self, printer: Any, fileconfig: Any,
                         access_tracking: Any, section: str) -> None:
                super().__init__(printer, fileconfig, access_tracking,
                                 section)
                seen.append((section, access_tracking))

        monkeypatch.setattr(configfile, "ConfigWrapper", _Recording)
        printer = make_printer()
        live: Dict[Tuple[str, str], int] = {}

        class _Validate:
            access_tracking = live

        class _ConfigFile:
            validate = _Validate()

        printer.add_object("configfile", _ConfigFile())
        _bbx_boot(tmp_path, printer)
        assert [section for section, _t in seen] == [
            "AFC_BambuAMS Bambu_AMS_HT_1", "AFC_lane lane24",
            "AFC_hub Bambu_AMS_HT_1", "temperature_sensor Bambu_AMS_HT_1"]
        assert [tracking is live for _s, tracking in seen] == [True] * 4
        _bbx_quiet(printer)

    @pytest.mark.parametrize("opts", [
        {"roster": f"boxed:{A}, ht:{H}", "pool_ams": 2, "pool_ht": 1,
         "ams_names": "X, X"},
        {"roster": "", "pool_ams": 1, "pool_ht": 0},
        {"roster": ""},
    ], ids=["roster", "pool-scout", "bridge-only-scout"])
    def test_init_never_touches_self_logger(self, tmp_path, opts):
        (tmp_path / "AFC_auto_vars.cfg").write_text(
            "[AFC_BambuAMS Bambu_AMS_2]\nafc_bowden_length : 1.0\n")
        m = self._direct(tmp_path, BambuConfig, buffer="Bamb_1",
                         dry_max_temp=-1, **opts)
        assert hasattr(m, "logger") is False
        assert m._dry_note == (
            "AFC_BridgeBox chain1: dry_max_temp is -1, below 0, so it is "
            "ignored and each heated unit dries up to its model's own "
            "ceiling. Remove it or set a positive value to silence this.")
        _bbx_quiet(m.printer)

    class _OptionReadingConfig(BambuConfig):
        """BambuConfig recording each option read and the value it gave."""

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            """Start with nothing read."""
            super().__init__(*args, **kwargs)
            self.read: Dict[str, Any] = {}

        def _note(self, option: str, value: Any) -> Any:
            """
            :param option: the option read
            :param value: the value it gave
            :return Any: the value
            """
            self.read.setdefault(option, value)
            return value

        def get(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            return self._note(option, super().get(option, *args, **kwargs))

        def getint(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            return self._note(option,
                              super().getint(option, *args, **kwargs))

        def getfloat(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            return self._note(option,
                              super().getfloat(option, *args, **kwargs))

        def getboolean(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            return self._note(option,
                              super().getboolean(option, *args, **kwargs))

    def _reads_of_every_start(self, tmp_path: pathlib.Path
                              ) -> Dict[str, Any]:
        """
        Build masters through a recording config in each kind of start
        __init__ branches on: enrolled with a fabricated buffer, a
        bridge-only scout, a pool scout, a named buffer, an adopted buffer
        and a second chain.

        :param tmp_path: where each start's files live
        :return dict: every option any of them read -> the first value read
        """
        configs: List[Any] = []

        def recording(*args: Any) -> BambuConfig:
            configs.append(self._OptionReadingConfig(*args))
            return configs[-1]

        adopting = make_printer()
        adopting.add_section("AFC_buffer Bamb_1", {
            "adc_pin": "bambu_buffer:fps", "type": "FPS_PSF"})
        shared = _bbx_steppers(make_printer())
        starts: List[Tuple[str, Optional[BambuPrinter], Dict[str, Any]]] = [
            ("enrolled", None, {"buffer": None}),
            ("bridge-only", None, {"roster": "", "buffer": None}),
            ("pool-scout", None, {"roster": "", "pool_ams": 1}),
            ("named", None, {"buffer": "Bamb_1"}),
            ("adopted", adopting, {"buffer": None}),
            ("chain1", shared, {"buffer": None, "lane_base": 0}),
            ("chain2", shared, {"buffer": None, "lane_base": 0,
                                "roster": f"ht:{B}",
                                "unit_prefix": "Bambu_AMS_B"})]
        for sub, printer, opts in starts:
            name = "chain2" if sub == "chain2" else "chain1"
            self._direct(tmp_path / ("shared" if printer is shared else sub),
                         recording, printer, name=name, **opts)
        assert len(configs) == len(starts)
        read: Dict[str, Any] = {}
        for config in configs:
            for option, value in config.read.items():
                read.setdefault(option, value)
        return read

    def test_the_master_option_list_matches_what_init_actually_reads(
            self, tmp_path):
        read = self._reads_of_every_start(tmp_path / "starts")
        read.pop("roster")
        m = self._direct(tmp_path / "every", BambuConfig, **read)
        assert m._chain_defaults == {}
        m2 = self._direct(tmp_path / "extra", BambuConfig, foo_bar="1",
                          **read)
        assert m2._chain_defaults == {"foo_bar": "1"}
        assert _bbx_keys(m2, "AFC_BambuAMS Bambu_AMS_HT_1")["foo_bar"] == "1"
        _bbx_quiet(m.printer)
        _bbx_quiet(m2.printer)

    def test_options_that_cannot_be_listed_give_no_chain_defaults(
            self, tmp_path):
        m = self._direct(tmp_path, _BbxUnlistedConfig, foo_bar="1",
                         buffer="Bamb_1")
        assert m._chain_defaults == {}
        # A master with no tcp_key gives its units none, not the text "None".
        assert _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_HT_1") == {
            "serial_port": "/dev/serial/by-id/usb-chain1-if00",
            "ams_model": "ht", "extruder": "extruder",
            "hub": "Bambu_AMS_HT_1", "auto_error_recovery": "True",
            "measure_on_insert": "True", "buffer": "Bamb_1",
            "buffer_chip_name": "bambu_buffer", "pool": "True",
            "unit_uid": HT_UID, "heater": "True", "dry_max_temp": "85"}
        _bbx_quiet(m.printer)

    # at most four AMS bays

    NO_BAY_E = (
        f"AFC_BridgeBox chain1: AMS {E} is recorded but has no bay: all 4 "
        f"AMS bays belong to other units (Bambu_AMS_1 ({A}), Bambu_AMS_2 "
        f"({B}), Bambu_AMS_3 ({C}), Bambu_AMS_4 ({D})), and a Bambu bus "
        f"addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
        f"one. If this AMS replaces one of them, AFC_BRIDGEBOX_FORGET "
        f"CHAIN=chain1 UID=<that unit's uid> ")

    @staticmethod
    def _forget(master: afcBridgeBox, uid: str) -> List[LogLine]:
        """
        Run AFC_BRIDGEBOX_FORGET for ``uid`` on ``master``.

        :param master: the master
        :param uid: the unit to forget
        :return list: the command's responses
        """
        gcmd = FakeGcmd({"UID": uid})
        master.cmd_AFC_BRIDGEBOX_FORGET(gcmd)
        return gcmd.messages

    @pytest.mark.parametrize("first", [False, True],
                             ids=["listed-last", "listed-first"])
    def test_it_waits_without_a_bay_and_takes_the_forgotten_ones(
            self, tmp_path, first):
        _bbx_record(tmp_path, FOUR)
        assert self._layout(_bbx_restart(tmp_path)) == self.FOUR_LAYOUT
        _bbx_record(tmp_path, (f"boxed:{E}, " + FOUR) if first
                    else (FOUR + f", boxed:{E}"))
        m2 = _bbx_restart(tmp_path)
        assert self._layout(m2) == self.FOUR_LAYOUT
        assert _bbx_uids(m2) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                                 "Bambu_AMS_3": C, "Bambu_AMS_4": D,
                                 "Bambu_AMS_HT_1": H, "Bambu_AMS_HT_2": ""}
        assert E in {u["uid"] for u in m2.units}
        assert E not in m2._name_map and E not in m2._lane_map
        assert m2._unbayed == {E: None}
        assert _bbx_ready(m2) == [
            ("warning", self.NO_BAY_E + "frees that bay and this AMS claims "
                                        "it live.")]
        self._forget(m2, D)
        m3 = _bbx_restart(tmp_path)
        assert self._layout(m3) == self.FOUR_LAYOUT
        assert _bbx_uids(m3)["Bambu_AMS_4"] == E
        assert m3._layout_notes == []

    def test_without_a_pool_it_takes_the_bay_at_the_next_restart(
            self, tmp_path):
        nopool = {"pool_ams": 0, "pool_ht": 0}
        _bbx_record(tmp_path, FOUR)
        before = self._layout(_bbx_restart(tmp_path, **nopool))
        assert before == self._expect(
            (24, 4, "Bambu_AMS_1"), (28, 4, "Bambu_AMS_2"),
            (32, 4, "Bambu_AMS_3"), (36, 4, "Bambu_AMS_4"),
            (40, 1, "Bambu_AMS_HT_1"))
        _bbx_record(tmp_path, FOUR + f", boxed:{E}")
        m2 = _bbx_restart(tmp_path, **nopool)
        assert self._layout(m2) == before
        assert E not in _bbx_uids(m2).values()
        assert _bbx_ready(m2) == [
            ("warning", self.NO_BAY_E + "frees that bay for it at the next "
                                        "RESTART. An AMS offline on a live "
                                        "chain for 120s is removed from the "
                                        "recorded roster, which frees its "
                                        "bay at the next RESTART too.")]
        self._forget(m2, D)
        m3 = _bbx_restart(tmp_path, **nopool)
        assert self._layout(m3) == before
        assert _bbx_uids(m3)["Bambu_AMS_4"] == E

    def test_a_state_recording_five_ams_bays_boots(self, tmp_path):
        _bbx_record(tmp_path, FOUR + f", boxed:{E}",
                    name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                              f"{C}:Bambu_AMS_3, {D}:Bambu_AMS_4, "
                              f"{E}:Bambu_AMS_5, {H}:Bambu_AMS_HT_1"),
                    lane_map=(f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, "
                              f"{E}:40:4, {H}:44:1"))
        m = _bbx_restart(tmp_path)
        assert self._layout(m) == self.FOUR_LAYOUT
        assert E not in _bbx_uids(m).values()
        assert m._state_get(SEC, "name_map") == (
            f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, {C}:Bambu_AMS_3, "
            f"{D}:Bambu_AMS_4, {H}:Bambu_AMS_HT_1")
        assert m._state_get(SEC, "lane_map") == (
            f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, {H}:40:1")
        assert _bbx_ready(m) == [
            ("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) "
                        f"keeps its name, and its lanes and T# changed: "
                        f"lane44 (T44) -> lane40 (T40)."),
            ("warning", self.NO_BAY_E + "frees that bay and this AMS claims "
                                        "it live. Its record as Bambu_AMS_5 "
                                        "is dropped.")]

    def test_an_ams_saved_past_the_fourth_bay_redraws_inside_the_four(
            self, tmp_path):
        _bbx_record(tmp_path,
                    f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{E}, ht:{H}",
                    name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                              f"{C}:Bambu_AMS_3, {E}:Bambu_AMS_5, "
                              f"{H}:Bambu_AMS_HT_1"),
                    lane_map=(f"{A}:24:4, {B}:28:4, {C}:32:4, {E}:40:4, "
                              f"{H}:44:1"))
        m = _bbx_restart(tmp_path)
        assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                                "Bambu_AMS_3": C, "Bambu_AMS_4": E,
                                "Bambu_AMS_HT_1": H, "Bambu_AMS_HT_2": ""}
        assert self._layout(m) == self.FOUR_LAYOUT
        assert m._lane_map[E] == (36, 4)
        assert m._state_get(SEC, "name_map") == (
            f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, {C}:Bambu_AMS_3, "
            f"{E}:Bambu_AMS_4, {H}:Bambu_AMS_HT_1")
        assert _bbx_ready(m) == [
            ("warning", f"AFC_BridgeBox chain1: AMS {E} was recorded as "
                        f"Bambu_AMS_5, past the 4 AMS bays a Bambu bus "
                        f"addresses. It is now Bambu_AMS_4, and its lanes and "
                        f"T# changed: lane40-lane43 (T40-T43) -> "
                        f"lane36-lane39 (T36-T39). Lane records saved under "
                        f"Bambu_AMS_5 (spool, material, colour, T# map) do "
                        f"not carry over to Bambu_AMS_4; a tagged spool is "
                        f"read again from its tag. Its learned bowden "
                        f"lengths stay with the unit."),
            ("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) "
                        f"keeps its name, and its lanes and T# changed: "
                        f"lane44 (T44) -> lane40 (T40).")]
        m2 = _bbx_restart(tmp_path)
        assert self._layout(m2) == self.FOUR_LAYOUT
        assert m2._layout_notes == []
        _bbx_quiet(m2.printer)

    def test_an_ams_saved_as_the_fifth_ams_names_entry_redraws(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}",
                    name_map=f"{A}:N5, {H}:Bambu_AMS_HT_1",
                    lane_map=f"{A}:40:4, {H}:44:1")
        m = _bbx_restart(tmp_path, pool_ams=1, pool_ht=1,
                         ams_names="N1, N2, N3, N4, N5")
        assert _bbx_uids(m) == {"N1": A, "Bambu_AMS_HT_1": H}
        assert self._layout(m) == self._expect((24, 4, "N1"),
                                             (40, 1, "Bambu_AMS_HT_1"))
        # _ht_band caps the recorded five-bay band at four, so the HT moves
        # from lane44 to lane40, and the band note says so.
        assert _bbx_ready(m) == [
            ("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) on "
                        f"lane44 (T44) holds the AMS band at 4 bays although "
                        f"pool_ams and the recorded AMS need only 1. "
                        f"Bambu_AMS_HT_1 moves to lane40 (T40), past those 4 "
                        f"bays. Set pool_ams: 4 to silence this. "
                        f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={H} lets the "
                        f"band shrink to 1 at the next RESTART, and erases "
                        f"what it learned."),
            ("warning", f"AFC_BridgeBox chain1: AMS {A} was recorded as N5, "
                        f"past the 4 AMS bays a Bambu bus addresses. It is "
                        f"now N1, and its lanes and T# changed: lane40-lane43 "
                        f"(T40-T43) -> lane24-lane27 (T24-T27). Lane records "
                        f"saved under N5 (spool, material, colour, T# map) do "
                        f"not carry over to N1; a tagged spool is read again "
                        f"from its tag. Its learned bowden lengths stay with "
                        f"the unit."),
            ("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) "
                        f"keeps its name, and its lanes and T# changed: "
                        f"lane44 (T44) -> lane40 (T40).")]

    def test_a_held_ht_recorded_short_of_the_band_is_named_as_moving(
            self, tmp_path):
        # HHHH's lane40 holds the band at four bays; GGGG's lane37 sits past
        # three, so it moves to the band's edge and only HHHH keeps its lane.
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}, ht:{G}",
                    name_map=(f"{A}:Bambu_AMS_1, {H}:Bambu_AMS_HT_1, "
                              f"{G}:Bambu_AMS_HT_2"),
                    lane_map=f"{A}:24:4, {H}:40:1, {G}:37:1")
        m = _bbx_restart(tmp_path, pool_ams=1, pool_ht=2)
        assert self._layout(m) == self._expect((24, 4, "Bambu_AMS_1"),
                                             (40, 1, "Bambu_AMS_HT_1"),
                                             (41, 1, "Bambu_AMS_HT_2"))
        assert _bbx_ready(m) == [
            ("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) on "
                        f"lane40 (T40), HT {G} (Bambu_AMS_HT_2) on lane37 "
                        f"(T37) hold the AMS band at 4 bays although pool_ams "
                        f"and the recorded AMS need only 1. Bambu_AMS_HT_2 "
                        f"moves to lane41 (T41), past those 4 bays. Set "
                        f"pool_ams: 4 to silence this. AFC_BRIDGEBOX_FORGET "
                        f"CHAIN=chain1 UID=<uid> for each lets the band "
                        f"shrink to 1 at the next RESTART, and erases what "
                        f"they learned."),
            ("warning", f"AFC_BridgeBox chain1: HT {G} (Bambu_AMS_HT_2) "
                        f"keeps its name, and its lanes and T# changed: "
                        f"lane37 (T37) -> lane41 (T41).")]

    def test_an_ams_that_redraws_keeps_its_lanes_when_their_rank_is_free(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}",
                    name_map=f"{A}:Alpha, {H}:Bambu_AMS_HT_1",
                    lane_map=f"{A}:32:4, {H}:40:1")
        m = _bbx_restart(tmp_path)
        assert _bbx_uids(m)["Bambu_AMS_3"] == A
        assert m._lane_map[A] == (32, 4)
        assert self._layout(m)[40] == "Bambu_AMS_HT_1"
        assert _bbx_ready(m) == [
            ("warning", f"AFC_BridgeBox chain1: AMS {A} was recorded as "
                        f"Alpha, which no ams_names entry or default name "
                        f"gives an AMS. It is now Bambu_AMS_3, and its lanes "
                        f"stay lane32-lane35 (T32-T35). Lane records saved "
                        f"under Alpha (spool, material, colour, T# map) do "
                        f"not carry over to Bambu_AMS_3; a tagged spool is "
                        f"read again from its tag. Its learned bowden "
                        f"lengths stay with the unit.")]

    def test_a_recorded_ams_draws_before_a_new_uid_listed_ahead_of_it(
            self, tmp_path):
        _bbx_record(tmp_path, FOUR, name_map=(
            f"{A}:Alpha, {B}:Bambu_AMS_2, {C}:Bambu_AMS_3, "
            f"{D}:Bambu_AMS_4, {H}:Bambu_AMS_HT_1"),
            lane_map=f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, {H}:40:1")
        m = _bbx_boot(tmp_path, roster=f"boxed:{E}, " + FOUR, **POOL)
        assert _bbx_uids(m)["Bambu_AMS_1"] == A
        assert E not in _bbx_uids(m).values()
        assert m._unbayed == {E: None}
        assert _bbx_ready(m) == [
            ("warning", f"AFC_BridgeBox chain1: AMS {A} was recorded as "
                        f"Alpha, which no ams_names entry or default name "
                        f"gives an AMS. It is now Bambu_AMS_1, and its lanes "
                        f"stay lane24-lane27 (T24-T27). Lane records saved "
                        f"under Alpha (spool, material, colour, T# map) do "
                        f"not carry over to Bambu_AMS_1; a tagged spool is "
                        f"read again from its tag. Its learned bowden "
                        f"lengths stay with the unit."),
            ("warning", self.NO_BAY_E + "frees that bay and this AMS claims "
                                        "it live; remove that unit from "
                                        "roster: too.")]

    def test_lowering_pool_ams_keeps_every_recorded_bay_and_the_ht_lanes(
            self, tmp_path):
        _bbx_record(tmp_path, FOUR)
        assert self._layout(_bbx_restart(tmp_path, pool_ams=4)) == self.FOUR_LAYOUT
        m2 = _bbx_restart(tmp_path, pool_ams=2)
        assert self._layout(m2) == self.FOUR_LAYOUT
        assert _bbx_uids(m2)["Bambu_AMS_4"] == D
        assert m2._layout_notes == []
        _bbx_quiet(m2.printer)
        # Three AMS left, the highest on the fourth bay: the band still
        # covers it, and pool_ams builds no spare past the three recorded.
        assert self._forget(m2, B) == [
            ("respond_info", f"AFC_BridgeBox chain1: forgot {B} -- lanes "
                             f"28-31 and the name Bambu_AMS_2 freed for "
                             f"reuse -- slot freed to the pool LIVE; the "
                             f"next same-family unit claims it with no "
                             f"reboot.")]
        assert m2.printer.afc.logger.messages == []
        assert m2.printer.gcode.messages == [
            ("respond_raw", "// action:prompt_end")]
        m3 = _bbx_restart(tmp_path, pool_ams=2)
        assert self._layout(m3) == self._expect(
            (24, 4, "Bambu_AMS_1"), (32, 4, "Bambu_AMS_3"),
            (36, 4, "Bambu_AMS_4"), (40, 1, "Bambu_AMS_HT_1"),
            (41, 1, "Bambu_AMS_HT_2"))
        assert _bbx_uids(m3) == {"Bambu_AMS_1": A, "Bambu_AMS_3": C,
                                 "Bambu_AMS_4": D, "Bambu_AMS_HT_1": H,
                                 "Bambu_AMS_HT_2": ""}
        _bbx_quiet(m3.printer)

    def test_lowering_pool_ams_keeps_a_recorded_ht_past_the_spares(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        m1 = _bbx_restart(tmp_path, pool_ams=4)
        assert self._layout(m1)[40] == "Bambu_AMS_HT_1"
        assert _bbx_ready(m1) == []
        for _again in range(2):
            m = _bbx_restart(tmp_path, pool_ams=2)
            assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                                    "Bambu_AMS_HT_1": H, "Bambu_AMS_HT_2": ""}
            assert self._layout(m) == self._expect(
                (24, 4, "Bambu_AMS_1"), (28, 4, "Bambu_AMS_2"),
                (40, 1, "Bambu_AMS_HT_1"), (41, 1, "Bambu_AMS_HT_2"))
            assert m._lane_map[H] == (40, 1)
            assert m._lane_moves == {}
            assert m._ams_band == 4
            assert _bbx_ready(m) == [("warning", self._held_note(2))]

    def test_forgetting_the_top_ams_keeps_the_ht_lanes(self, tmp_path):
        _bbx_record(tmp_path, FOUR)
        assert self._layout(_bbx_restart(tmp_path, pool_ams=2)) == self.FOUR_LAYOUT
        m = _bbx_restart(tmp_path, pool_ams=2)
        assert _bbx_ready(m) == []
        self._forget(m, D)
        for _again in range(2):
            m = _bbx_restart(tmp_path, pool_ams=2)
            assert self._layout(m)[40] == "Bambu_AMS_HT_1"
            assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                                    "Bambu_AMS_3": C, "Bambu_AMS_HT_1": H,
                                    "Bambu_AMS_HT_2": ""}
            assert _bbx_ready(m) == [("warning", self._held_note(3))]
        self._forget(m, C)
        for _again in range(2):
            m = _bbx_restart(tmp_path, pool_ams=2)
            assert self._layout(m)[40] == "Bambu_AMS_HT_1"
            assert m._lane_map[H] == (40, 1)
            assert _bbx_ready(m) == [("warning", self._held_note(2))]
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}, boxed:{E}")
        m = _bbx_restart(tmp_path, pool_ams=2)
        assert _bbx_uids(m)["Bambu_AMS_3"] == E
        assert self._layout(m)[40] == "Bambu_AMS_HT_1"

    def test_lanes_saved_by_a_start_that_never_got_ready_hold_no_band(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        m1 = _bbx_restart(tmp_path, pool_ams=2)
        assert self._layout(m1)[32] == "Bambu_AMS_HT_1"
        assert _bbx_ready(m1) == []
        assert m1._state_get(SEC, "ams_band") == "2"
        m2 = _bbx_restart(tmp_path, pool_ams=4)
        assert self._layout(m2)[40] == "Bambu_AMS_HT_1"
        assert m2._lane_map[H] == (40, 1)
        assert m2._state_get(SEC, "ams_band") == "2"
        moved = [("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) "
                             f"keeps its name, and its lanes and T# changed: "
                             f"lane40 (T40) -> lane32 (T32).")]
        for notes in (moved, []):
            m = _bbx_restart(tmp_path, pool_ams=2)
            assert self._layout(m)[32] == "Bambu_AMS_HT_1"
            assert m._lane_map[H] == (32, 1)
            assert _bbx_ready(m) == notes
            assert m._state_get(SEC, "lane_map") == (
                f"{A}:24:4, {B}:28:4, {H}:32:1")

    @pytest.mark.parametrize("where", ["printer", "config"])
    def test_an_ht_lane_that_exists_elsewhere_ends_the_hold(self, tmp_path,
                                                            where):
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        assert _bbx_ready(_bbx_restart(tmp_path, pool_ams=4)) == []
        lanes = [f"AFC_lane lane{n}" for n in range(40, 44)]

        def _around() -> BambuPrinter:
            printer = make_printer()
            for i, section in enumerate(lanes):
                if where == "printer":
                    printer.add_object(section, object())
                else:
                    printer.add_section(section, {"unit": f"Box_1:{i + 1}"})
            return printer

        m = _bbx_restart(tmp_path, pool_ams=2, printer=_around())
        assert self._layout(m) == self._expect(
            (24, 4, "Bambu_AMS_1"), (28, 4, "Bambu_AMS_2"),
            (32, 1, "Bambu_AMS_HT_1"), (33, 1, "Bambu_AMS_HT_2"))
        assert m._lane_map[H] == (32, 1)
        assert _bbx_ready(m) == [
            ("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) on "
                        f"lane40 (T40) cannot keep its lanes: [AFC_lane "
                        f"lane40] already exists outside this chain. The AMS "
                        f"band is 2 bays, what pool_ams and the recorded AMS "
                        f"need, and the HT lanes follow it."),
            ("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) "
                        f"keeps its name, and its lanes and T# changed: "
                        f"lane40 (T40) -> lane32 (T32).")]
        m = _bbx_restart(tmp_path, pool_ams=2, printer=_around())
        assert self._layout(m)[32] == "Bambu_AMS_HT_1"
        assert _bbx_ready(m) == []

    def test_a_learned_value_filed_under_a_held_lane_is_no_clash(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        _bbx_ready(_bbx_restart(tmp_path, pool_ams=4))
        (tmp_path / "AFC_auto_vars.cfg").write_text(
            "[AFC_lane lane40]\ndist_hub : 61.0\n")
        printer = make_printer()
        printer.add_section("AFC_lane lane40", {"dist_hub": "61.0"})
        m = _bbx_restart(tmp_path, pool_ams=2, printer=printer)
        assert self._layout(m)[40] == "Bambu_AMS_HT_1"
        assert m._declared_lanes == set()
        assert _bbx_ready(m) == [("warning", self._held_note(2))]

    # an ams_names / ht_names entry renamed

    @staticmethod
    def _renamed(uid: str, saved: str, now: str, why: str,
                 lanes: str, fam: str = "AMS") -> str:
        """
        :param uid: the unit
        :param saved: the name its state records
        :param now: the name it wears now
        :param why: why it could not keep the saved name
        :param lanes: what happened to its lanes
        :param fam: AMS or HT
        :return str: the ready line for a unit that drew a new name
        """
        return (f"AFC_BridgeBox chain1: {fam} {uid} was recorded as {saved}, "
                f"{why}. It is now {now}, and {lanes}. Lane records saved "
                f"under {saved} (spool, material, colour, T# map) do not "
                f"carry over to {now}; a tagged spool is read again from its "
                f"tag. Its learned bowden lengths stay with the unit.")

    def test_the_unit_redraws_and_no_bay_doubles_up(self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}")
        m1 = _bbx_restart(tmp_path, pool_ams=2, pool_ht=1,
                          ams_names="Alpha, Bravo")
        m1._state_set({f"{SEC} learned {A}": {"afc_bowden_length": "1234.0"}})
        m = _bbx_restart(tmp_path, pool_ams=2, pool_ht=1,
                         ams_names="Red, Blue")
        assert _bbx_uids(m) == {"Red": A, "Blue": "", "Bambu_AMS_HT_1": H}
        assert self._layout(m) == self._expect(
            (24, 4, "Red"), (28, 4, "Blue"), (32, 1, "Bambu_AMS_HT_1"))
        assert _bbx_keys(m, "AFC_BambuAMS Red")["afc_bowden_length"] == \
            "1234.0"
        assert _bbx_ready(m) == [("warning", self._renamed(
            A, "Alpha", "Red", "which no ams_names entry or default name "
                               "gives an AMS",
            "its lanes stay lane24-lane27 (T24-T27)"))]

    def test_a_renamed_second_entry_leaves_the_first_unit_alone(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        _bbx_restart(tmp_path, pool_ams=2, pool_ht=1,
                     ams_names="Alpha, Bravo")
        m = _bbx_restart(tmp_path, pool_ams=2, pool_ht=1,
                         ams_names="Alpha, Charlie")
        assert _bbx_uids(m) == {"Alpha": A, "Charlie": B,
                                "Bambu_AMS_HT_1": H}
        assert m._lane_map[A] == (24, 4) and m._lane_map[B] == (28, 4)
        assert _bbx_ready(m) == [("warning", self._renamed(
            B, "Bravo", "Charlie", "which no ams_names entry or default name "
                                   "gives an AMS",
            "its lanes stay lane28-lane31 (T28-T31)"))]

    def test_adding_ams_names_renames_the_units_wearing_defaults(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        _bbx_restart(tmp_path, pool_ams=2, pool_ht=1)
        m = _bbx_restart(tmp_path, pool_ams=2, pool_ht=1,
                         ams_names="Red, Blue")
        assert _bbx_uids(m) == {"Red": A, "Blue": B, "Bambu_AMS_HT_1": H}
        why = ("which no ams_names entry or default name gives an AMS "
               "(ams_names replaces the first 2 default names)")
        assert _bbx_ready(m) == [
            ("warning", self._renamed(A, "Bambu_AMS_1", "Red", why,
                                      "its lanes stay lane24-lane27 "
                                      "(T24-T27)")),
            ("warning", self._renamed(B, "Bambu_AMS_2", "Blue", why,
                                      "its lanes stay lane28-lane31 "
                                      "(T28-T31)"))]

    def test_a_renamed_ht_redraws_too(self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}")
        _bbx_restart(tmp_path, pool_ams=1, pool_ht=2, ht_names="Hot, Warm")
        m = _bbx_restart(tmp_path, pool_ams=1, pool_ht=2,
                         ht_names="Red, Warm")
        assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Red": H, "Warm": ""}
        assert m._name_map[H] == "Red"
        assert _bbx_ready(m) == [("warning", self._renamed(
            H, "Hot", "Red", "which no ht_names entry or default name gives "
                             "an HT",
            "its lanes stay lane28 (T28)", fam="HT"))]

    # pool_ams above the four AMS a Bambu bus addresses

    POOL6_NOTE = ("AFC_BridgeBox chain1: pool_ams is 6, but a Bambu bus "
                  "addresses at most 4 AMS, so 4 AMS bays are built and the "
                  "HT lanes start at lane40. Set pool_ams: 4 to silence this.")

    def test_pool_ams_above_four_builds_four_and_says_so(self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"ht:{H}", pool_ams=6, pool_ht=1)
        assert m.pool_ams == 4
        assert _bbx_uids(m) == {"Bambu_AMS_1": "", "Bambu_AMS_2": "",
                                "Bambu_AMS_3": "", "Bambu_AMS_4": "",
                                "Bambu_AMS_HT_1": H}
        assert self._layout(m) == self._expect(
            (24, 4, "Bambu_AMS_1"), (28, 4, "Bambu_AMS_2"),
            (32, 4, "Bambu_AMS_3"), (36, 4, "Bambu_AMS_4"),
            (40, 1, "Bambu_AMS_HT_1"))
        assert _bbx_ready(m) == [("warning", self.POOL6_NOTE)]
        (tmp_path / "four").mkdir()
        m4 = _bbx_boot(tmp_path / "four", roster=f"ht:{H}", pool_ams=4,
                       pool_ht=1)
        assert m4.pool_ams == 4
        assert self._layout(m4) == self._layout(m)
        assert _bbx_ready(m4) == []

    def test_pool_ams_above_four_moves_an_ht_laid_out_past_four_ams_bays(
            self, tmp_path):
        # The state as a six-bay AMS band left it: the HT on lane48.
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}",
                    name_map=f"{A}:Bambu_AMS_1, {H}:Bambu_AMS_HT_1",
                    lane_map=f"{A}:24:4, {H}:48:1")
        m = _bbx_restart(tmp_path, pool_ams=6, pool_ht=2)
        assert self._layout(m) == self.FOUR_LAYOUT
        assert m._lane_map == {A: (24, 4), H: (40, 1)}
        assert m._lane_moves == {"lane48": {
            "uid": H, "family": "ht", "name": "Bambu_AMS_HT_1", "now": None}}
        assert _bbx_ready(m) == [
            ("warning", self.POOL6_NOTE),
            ("warning", f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) "
                        f"keeps its name, and its lanes and T# changed: "
                        f"lane48 (T48) -> lane40 (T40).")]

    # no pool, a lower AMS pruned from the roster

    def test_without_a_pool_the_ht_stays_clear_of_a_surviving_higher_ams(
            self, tmp_path):
        nopool = {"pool_ams": 0, "pool_ht": 0}
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        m1 = _bbx_restart(tmp_path, **nopool)
        assert self._layout(m1)[32] == "Bambu_AMS_HT_1"
        # Auto-removal drops A from the recorded roster; its tombstone stays.
        _bbx_record(tmp_path, f"boxed:{B}, ht:{H}")
        m = _bbx_restart(tmp_path, **nopool)
        assert self._layout(m) == self._expect((28, 4, "Bambu_AMS_2"),
                                             (32, 1, "Bambu_AMS_HT_1"))
        assert m._name_map[A] == "Bambu_AMS_1"
        assert m._lane_map[A] == (24, 4)
        assert m._layout_notes == []
        _bbx_quiet(m.printer)

    def test_without_a_pool_a_new_ams_takes_a_free_bay_inside_the_band(
            self, tmp_path):
        # A and B departed (tombstones); D kept the fourth bay, so the band
        # spans four and the third bay moves no HT lane: E takes it rather
        # than a departed unit's name and learned values.
        _bbx_record(tmp_path, f"boxed:{D}, ht:{H}, boxed:{E}",
                    name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                              f"{D}:Bambu_AMS_4, {H}:Bambu_AMS_HT_1"),
                    lane_map=f"{A}:24:4, {B}:28:4, {D}:36:4, {H}:40:1")
        m = _bbx_restart(tmp_path, pool_ams=0, pool_ht=0)
        assert _bbx_uids(m) == {"Bambu_AMS_3": E, "Bambu_AMS_4": D,
                                "Bambu_AMS_HT_1": H}
        assert m._name_map == {A: "Bambu_AMS_1", B: "Bambu_AMS_2",
                               D: "Bambu_AMS_4", H: "Bambu_AMS_HT_1",
                               E: "Bambu_AMS_3"}
        assert self._layout(m) == self._expect(
            (32, 4, "Bambu_AMS_3"), (36, 4, "Bambu_AMS_4"),
            (40, 1, "Bambu_AMS_HT_1"))
        assert _bbx_ready(m) == []

    # names saved past a family's defaults, and two uids on one name

    def test_an_ht_saved_past_sixteen_keeps_its_bay(self, tmp_path):
        _bbx_record(tmp_path, f"ht:{H}", name_map=f"{H}:Bambu_AMS_HT_18")
        m = _bbx_restart(tmp_path, pool_ams=1, pool_ht=1)
        assert _bbx_uids(m) == {"Bambu_AMS_1": "", "Bambu_AMS_HT_18": H}
        assert self._layout(m) == self._expect((24, 4, "Bambu_AMS_1"),
                                             (28 + 17, 1, "Bambu_AMS_HT_18"))
        assert m._lane_map == {H: (45, 1)}
        assert m._layout_notes == []
        _bbx_quiet(m.printer)

    @pytest.mark.parametrize("roster, name, second, fam, lanes", [
        (f"boxed:{A}, boxed:{B}", "Bambu_AMS_1", "Bambu_AMS_2", "AMS",
         "lane28-lane31 (T28-T31)"),
        (f"ht:{G}, ht:{H}", "Bambu_AMS_HT_1", "Bambu_AMS_HT_2", "HT",
         "lane41 (T41)"),
    ])
    def test_of_two_uids_on_one_name_the_first_keeps_it(
            self, tmp_path, roster, name, second, fam, lanes):
        # A hand-edited state file: both uids recorded with one name.
        first, other = [e.split(":")[1] for e in roster.split(", ")]
        _bbx_record(tmp_path, roster,
                    name_map=f"{first}:{name}, {other}:{name}")
        m = _bbx_restart(tmp_path)
        uids = _bbx_uids(m)
        assert (uids[name], uids[second]) == (first, other)
        assert _bbx_ready(m) == [("warning", self._renamed(
            other, name, second, f"which {first} also holds",
            f"its lanes and T# are now {lanes}", fam=fam))]

    # name lists that give one name to two bays

    @staticmethod
    def _clash(what: str, bay: str, named: str) -> str:
        """
        :param what: the entry and what else it is
        :param bay: the bay it would name
        :param named: the name the bay wears instead
        :return str: the ready line for a name list entry that gave way
        """
        return (f"AFC_BridgeBox chain1: {what}, so {bay} is named {named} -- "
                f"give each bay a name of its own.")

    def test_an_ams_entry_equal_to_an_ht_name_costs_no_bay(self, tmp_path):
        _bbx_record(tmp_path, FOUR)
        m1 = _bbx_restart(tmp_path)
        for _again in range(2):
            m = _bbx_restart(tmp_path, ams_names="Bambu_AMS_HT_1")
            assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                                    "Bambu_AMS_3": C, "Bambu_AMS_4": D,
                                    "Bambu_AMS_HT_1": H, "Bambu_AMS_HT_2": ""}
            assert self._layout(m) == self._layout(m1) == self.FOUR_LAYOUT
            assert _bbx_ready(m) == [("warning", self._clash(
                "ams_names entry 1 (Bambu_AMS_HT_1) is also the default name "
                "of HT bay 1", "AMS bay 1", "Bambu_AMS_1"))]

    @pytest.mark.parametrize("pool, spare", [
        (POOL, {"Bambu_AMS_HT_2": ""}), ({"pool_ams": 0, "pool_ht": 0}, {})],
        ids=["pool", "no-pool"])
    def test_a_repeated_ams_entry_still_builds_four_ams_bays(
            self, tmp_path, pool, spare):
        _bbx_record(tmp_path, FOUR)
        for _again in range(2):
            m = _bbx_restart(tmp_path, ams_names="X, X, Y, Z", **pool)
            assert _bbx_uids(m) == dict({"X": A, "Bambu_AMS_2": B, "Y": C,
                                         "Z": D, "Bambu_AMS_HT_1": H},
                                        **spare)
            assert self._layout(m)[40] == "Bambu_AMS_HT_1"
            assert _bbx_ready(m) == [("warning", self._clash(
                "ams_names entry 2 (X) is also ams_names entry 1",
                "AMS bay 2", "Bambu_AMS_2"))]

    def test_a_repeated_ht_entry_names_its_bay_by_default(self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{G}, ht:{H}")
        for _again in range(2):
            m = _bbx_restart(tmp_path, pool_ams=1, ht_names="Hot, Hot")
            assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Hot": G,
                                    "Bambu_AMS_HT_2": H}
            assert self._layout(m) == self._expect(
                (24, 4, "Bambu_AMS_1"), (28, 1, "Hot"),
                (29, 1, "Bambu_AMS_HT_2"))
            assert _bbx_ready(m) == [("warning", self._clash(
                "ht_names entry 2 (Hot) is also ht_names entry 1",
                "HT bay 2", "Bambu_AMS_HT_2"))]

    def test_a_fallback_name_an_entry_holds_takes_a_suffix(self, tmp_path):
        # Entry 2 repeats entry 1, and its default is entry 3.
        m = _bbx_boot(tmp_path, roster="", pool_ams=3, pool_ht=0,
                      ams_names="X, X, Bambu_AMS_2")
        assert self._layout(m) == self._expect(
            (24, 4, "X"), (28, 4, "Bambu_AMS_2_2"), (32, 4, "Bambu_AMS_2"))
        assert m._rank_of("ams", "Bambu_AMS_2_2") == 1
        assert m._rank_of("ams", "Bambu_AMS_2") == 2
        assert m._name_notes == [self._clash(
            "ams_names entry 2 (X) is also ams_names entry 1", "AMS bay 2",
            "Bambu_AMS_2_2")]
        _bbx_quiet(m.printer)

    # Each test below seeds the maps of a unit wearing an entry that is
    # also the default of an index past its list, and checks that two boots
    # leave its name and lanes as recorded.

    def test_an_entry_named_like_a_later_default_keeps_its_lanes(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}",
                    name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                              f"{C}:Bambu_AMS_4, {H}:Bambu_AMS_HT_1"),
                    lane_map=f"{A}:24:4, {B}:28:4, {C}:32:4, {H}:40:1")
        for _again in range(2):
            m = _bbx_restart(
                tmp_path, ams_names="Bambu_AMS_1, Bambu_AMS_2, Bambu_AMS_4")
            assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                                    "Bambu_AMS_4": C, "Bambu_AMS_4_2": "",
                                    "Bambu_AMS_HT_1": H, "Bambu_AMS_HT_2": ""}
            assert self._layout(m) == self._expect(
                (24, 4, "Bambu_AMS_1"), (28, 4, "Bambu_AMS_2"),
                (32, 4, "Bambu_AMS_4"), (36, 4, "Bambu_AMS_4_2"),
                (40, 1, "Bambu_AMS_HT_1"), (41, 1, "Bambu_AMS_HT_2"))
            assert m._lane_map[C] == (32, 4)
            assert m._lane_moves == {}
            assert _bbx_ready(m) == [
                ("warning", "AFC_BridgeBox chain1: the default name of AMS "
                            "bay 4 (Bambu_AMS_4) is ams_names entry 3, so AMS "
                            "bay 4 is named Bambu_AMS_4_2.")]

    def test_without_a_pool_the_entry_and_the_ht_stay_put(self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}",
                    name_map=f"{A}:Bambu_AMS_3, {H}:Bambu_AMS_HT_1",
                    lane_map=f"{A}:24:4, {H}:28:1")
        for _again in range(2):
            m = _bbx_restart(tmp_path, ams_names="Bambu_AMS_3", pool_ams=0,
                             pool_ht=0)
            assert self._layout(m) == self._expect(
                (24, 4, "Bambu_AMS_3"), (28, 1, "Bambu_AMS_HT_1"))
            assert _bbx_ready(m) == []

    def test_an_ht_entry_named_like_an_unbuilt_default_says_nothing(
            self, tmp_path):
        # No HT bay 2 is built, so its default needs no other name.
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}",
                    name_map=f"{A}:Bambu_AMS_1, {H}:Bambu_AMS_HT_2",
                    lane_map=f"{A}:24:4, {H}:28:1")
        opts = {"ht_names": "Bambu_AMS_HT_2", "pool_ams": 1, "pool_ht": 0}
        for _again in range(2):
            m = _bbx_restart(tmp_path, **opts)
            assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_HT_2": H}
            assert self._layout(m) == self._expect(
                (24, 4, "Bambu_AMS_1"), (28, 1, "Bambu_AMS_HT_2"))
            assert _bbx_ready(m) == []
        # A new HT takes the entry too.
        _bbx_record(tmp_path, f"boxed:{A}, ht:{G}", name_map="", lane_map="")
        m = _bbx_restart(tmp_path, **opts)
        assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_HT_2": G}

    def test_a_built_ht_default_an_entry_holds_takes_a_suffix(
            self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}")
        for _again in range(2):
            m = _bbx_restart(tmp_path, ht_names="Bambu_AMS_HT_2", pool_ams=1)
            assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_HT_2": H,
                                    "Bambu_AMS_HT_2_2": ""}
            assert self._layout(m) == self._expect(
                (24, 4, "Bambu_AMS_1"), (28, 1, "Bambu_AMS_HT_2"),
                (29, 1, "Bambu_AMS_HT_2_2"))
            assert m._rank_of("ht", "Bambu_AMS_HT_2_2") == 1
            assert _bbx_ready(m) == [
                ("warning", "AFC_BridgeBox chain1: the default name of HT "
                            "bay 2 (Bambu_AMS_HT_2) is ht_names entry 1, so "
                            "HT bay 2 is named Bambu_AMS_HT_2_2.")]

    @pytest.mark.parametrize("family, names, uids", [
        ("ams", {"ams_names": "Bambu_AMS_2"},
         {"Bambu_AMS_2": A, "Bambu_AMS_HT_1": H}),
        ("ht", {"ht_names": "Bambu_AMS_HT_2"},
         {"Bambu_AMS_1": A, "Bambu_AMS_HT_2": H}),
    ])
    def test_an_entry_chosen_around_a_hand_written_unit_boots(
            self, tmp_path, family, names, uids):
        # The hand-written unit wears the default name; the entry names the
        # chain's bay after the next one.
        taken = "Bambu_AMS_1" if family == "ams" else "Bambu_AMS_HT_1"
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}")
        for _again in range(2):
            printer = make_printer()
            printer.add_section("AFC_stepper lane1", {"unit": f"{taken}:1"})
            printer.add_section("AFC_stepper lane2", {"unit": f"{taken}:2"})
            m = _bbx_restart(tmp_path, pool_ams=1, pool_ht=1,
                             printer=printer, **names)
            ams, ht = list(uids)
            assert _bbx_uids(m) == uids
            assert self._layout(m) == self._expect((24, 4, ams), (28, 1, ht))
            assert _bbx_ready(m) == []

    def test_an_ams_entry_an_ht_entry_takes_says_so(self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}")
        assert _bbx_uids(_bbx_restart(tmp_path, ams_names="Hot"))["Hot"] == A
        m = _bbx_restart(tmp_path, ams_names="Hot", ht_names="Hot")
        assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_2": "",
                                "Bambu_AMS_3": "", "Bambu_AMS_4": "",
                                "Hot": H, "Bambu_AMS_HT_2": ""}
        assert self._layout(m)[24] == "Bambu_AMS_1"
        assert _bbx_ready(m) == [
            ("warning", self._clash(
                "ams_names entry 1 (Hot) is also ht_names entry 1",
                "AMS bay 1", "Bambu_AMS_1")),
            ("warning", self._renamed(
                A, "Hot", "Bambu_AMS_1",
                "whose ams_names entry 1 gives way to ht_names entry 1",
                "its lanes stay lane24-lane27 (T24-T27)")),
            ("warning", self._renamed(
                H, "Bambu_AMS_HT_1", "Hot",
                "which no ht_names entry or default name gives an HT "
                "(ht_names replaces the first 1 default names)",
                "its lanes stay lane40 (T40)", fam="HT"))]

    # the console line when an AMS finds no free bay

    def test_with_roster_set_a_listed_unit_is_told_to_drop_the_other(
            self, tmp_path):
        _bbx_record(tmp_path, FOUR)
        m = _bbx_boot(tmp_path, roster=FOUR + f", boxed:{E}", **POOL)
        assert m._unbayed == {E: None}
        assert _bbx_ready(m) == [
            ("warning", self.NO_BAY_E + "frees that bay and this AMS claims "
                                        "it live; remove that unit from "
                                        "roster: too.")]

    # a loaded lane that changes unit

    def test_the_layout_records_whose_lanes_change_hands(self, tmp_path):
        # E recorded on Bambu_AMS_5 (lane40-lane43), the HT on lane44: this
        # boot gives lane40 to the HT and leaves E waiting.
        _bbx_record(tmp_path, FOUR + f", boxed:{E}",
                    name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                              f"{C}:Bambu_AMS_3, {D}:Bambu_AMS_4, "
                              f"{E}:Bambu_AMS_5, {H}:Bambu_AMS_HT_1"),
                    lane_map=(f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, "
                              f"{E}:40:4, {H}:44:1"))
        m = _bbx_restart(tmp_path)

        def _was(uid: str, fam: str, name: str,
                 now: Optional[str]) -> Dict[str, Optional[str]]:
            return {"uid": uid, "family": fam, "name": name, "now": now}

        assert m._lane_moves == {
            "lane40": _was(E, "ams", "Bambu_AMS_5", "Bambu_AMS_HT_1"),
            "lane41": _was(E, "ams", "Bambu_AMS_5", "Bambu_AMS_HT_2"),
            "lane42": _was(E, "ams", "Bambu_AMS_5", None),
            "lane43": _was(E, "ams", "Bambu_AMS_5", None),
            "lane44": _was(H, "ht", "Bambu_AMS_HT_1", None)}
        assert [m.loaded_lane_moved(f"lane{n}") for n in (39, 40, 44)] == [
            False, True, True]
        _bbx_quiet(m.printer)

    def test_an_unmoved_layout_tracks_nothing(self, tmp_path):
        _bbx_record(tmp_path, FOUR)
        _bbx_restart(tmp_path)
        m = _bbx_restart(tmp_path)
        assert m._lane_moves == {}
        assert m.loaded_lane_moved("lane40") is False
        _bbx_quiet(m.printer)

    # a bay saved when its unit is recorded; learned values follow the uid

    #: The pool the saved-bay tests start: three AMS bays and one HT bay.
    POOL3 = {"pool_ams": 3, "pool_ht": 1}
    #: The state section holding what a uid learned on chain1.
    LEARNED = f"{SEC} learned "

    @staticmethod
    def _seed(tmp_path: pathlib.Path,
              updates: Dict[str, Dict[str, str]]) -> None:
        """
        Leave the state file as an earlier boot would, with sections
        besides the chain's own: the real ``_state_set`` of a master that
        builds nothing, as record_chain_state writes the chain's section.

        :param tmp_path: where the state file lives
        :param updates: state section -> its keys
        """
        with tempfile.TemporaryDirectory() as scratch:
            printer = make_printer(var_file=os.path.join(scratch, "AFC.var"))
            seeder = make_bridgebox(
                pathlib.Path(scratch), printer=printer, register=False,
                roster="", pool_ams=0, pool_ht=0,
                state_file=str(tmp_path / "AFC_BridgeBox.cfg"))
            seeder._state_set(updates)

    def _live(self, tmp_path: pathlib.Path,
              monkeypatch: pytest.MonkeyPatch, bridge: FakeBridge,
              **opts: Any) -> afcBridgeBox:
        """
        The three-AMS, one-HT pool chain from the recorded roster, with
        real fabricated units connected as klippy connects them, watching
        ``bridge``.

        :param tmp_path: where the state and auto_vars files live
        :param monkeypatch: isolates the bridge table
        :param bridge: the chain's bridge
        :return afcBridgeBox: the master
        """
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True)
        merged: Dict[str, Any] = dict(self.POOL3, roster="")
        merged.update(opts)
        m = _bbx_boot(tmp_path, printer=printer, **merged)
        printer.connect()
        live_bridges()[m.serial_port] = bridge
        return m

    @staticmethod
    def _tick(master: afcBridgeBox, *times: float) -> None:
        """
        Run the chain watch at each time, with the printer's clock there.

        :param master: the master
        :param times: reactor times, in order
        """
        for t in times:
            master.printer.reactor.now = t
            master._scout_tick(t)

    #: What AFC's logger holds once a tick claims CCCC onto Bambu_AMS_2.
    CCCC_CLAIMED = [
        ("debug", f"AFC bambu Bambu_AMS_2: chain index not resolved yet "
                  f"(UID {C}); holding this unit's registrations until the "
                  f"chain map arrives"),
        ("info", f"AFC bambu Bambu_AMS_2: claimed UID {C} as boxed and "
                 f"brought online live (ams_index=0)."),
        ("info", f"AFC_BridgeBox chain1: CLAIMED {C} as boxed onto "
                 f"Bambu_AMS_2 (4 lanes) -- live, no restart.")]
    #: The popup that claim raises on the printer's gcode.
    CCCC_PROMPT = [
        ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_2"),
        ("respond_raw", f"// action:prompt_text UID {C} is on 'Bambu_AMS_2' "
                        f"(its T# and lanes are live)."),
        ("respond_raw", "// action:prompt_text It is saved on this bay once "
                        "it has been online 15s."),
        ("respond_raw", "// action:prompt_text Keep it here, or move it to "
                        "another named bay:"),
        ("respond_raw", f"// action:prompt_button Bambu_AMS_3|"
                        f"AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID={C} "
                        f"NAME=Bambu_AMS_3|primary"),
        ("respond_raw", "// action:prompt_footer_button Dismiss|RESPOND "
                        "TYPE=command MSG=action:prompt_end|info"),
        ("respond_raw", "// action:prompt_show")]

    def test_a_restart_keeps_it_after_a_lower_bay_is_freed(
            self, tmp_path, monkeypatch):
        # FORGET frees Bambu_AMS_1. A unit with no saved name would draw it
        # at the restart and come up on lane24/T24 instead of lane28/T28.
        _bbx_record(tmp_path, f"boxed:{A}")
        m = self._live(tmp_path, monkeypatch,
                       FakeBridge(uids=[A, C], online=[False, True]))
        self._tick(m, 100.0, 116.0)
        assert m._name_map[C] == "Bambu_AMS_2"
        assert m._lane_map[C] == (28, 4)
        assert self._forget(m, A) == [
            ("respond_info", f"AFC_BridgeBox chain1: forgot {A} -- lanes "
                             f"24-27 and the name Bambu_AMS_1 freed for "
                             f"reuse -- slot freed to the pool LIVE; the "
                             f"next same-family unit claims it with no "
                             f"reboot.")]
        # The ticks claimed, recorded and saved CCCC and raised its popup;
        # FORGET logged nothing and closed the popup.
        assert m.printer.afc.logger.messages == self.CCCC_CLAIMED + [
            ("info", f"AFC_BridgeBox chain1: NEW unit(s) on the chain: "
                     f"boxed:{C} -- recorded. A restart adds its temperature "
                     f"card."),
            ("info", f"AFC_BridgeBox chain1: saved {C} on Bambu_AMS_2 "
                     f"(lane28-lane31, T28-T31); it comes back there after a "
                     f"restart.")]
        assert m.printer.gcode.messages == self.CCCC_PROMPT + [
            ("respond_raw", "// action:prompt_end")]
        m2 = _bbx_restart(tmp_path, **self.POOL3)
        assert _bbx_uids(m2) == {"Bambu_AMS_1": "", "Bambu_AMS_2": C,
                                 "Bambu_AMS_3": "", "Bambu_AMS_HT_1": ""}
        assert "unit_uid" not in _bbx_keys(m2, "AFC_BambuAMS Bambu_AMS_1")
        assert _bbx_keys(m2, "AFC_lane lane28")["unit"] == "Bambu_AMS_2:1"
        _bbx_quiet(m2.printer)

    def test_migration_moves_a_name_value_to_the_pinned_uid(self, tmp_path):
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}", "name_map": f"{A}:Bambu_AMS_1",
                  "lane_map": f"{A}:24:4"},
            "AFC_BambuAMS Bambu_AMS_1": {"afc_bowden_length": "3632.0"}})
        m = _bbx_restart(tmp_path, **self.POOL3)
        assert m._learned_for(A) == {"afc_bowden_length": "3632.0"}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_1",
                            "afc_bowden_length") is None
        assert _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_1")[
            "afc_bowden_length"] == "3632.0"
        assert _bbx_ready(m) == [
            ("info", f"AFC_BridgeBox chain1: learned values stored under "
                     f"Bambu_AMS_1 now belong to its unit {A} "
                     f"(afc_bowden_length)")]

    def test_an_unowned_name_value_is_dropped_not_inherited(self, tmp_path):
        # Bambu_AMS_2 is a spare: whoever measured 3632 there, the next unit
        # to claim it did not.
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}"},
            "AFC_BambuAMS Bambu_AMS_2": {"afc_bowden_length": "3632.0"}})
        m = _bbx_restart(tmp_path, **self.POOL3)
        assert m._state_get("AFC_BambuAMS Bambu_AMS_2",
                            "afc_bowden_length") is None
        assert "afc_bowden_length" not in _bbx_keys(
            m, "AFC_BambuAMS Bambu_AMS_2")
        assert _bbx_ready(m) == [
            ("info", "AFC_BridgeBox chain1: learned values stored under "
                     "Bambu_AMS_2 dropped -- no unit is recorded with that "
                     "name")]

    def test_a_spare_value_goes_to_the_one_unit_roster_leaves_out(
            self, tmp_path):
        # roster: lists AAAA only. A build before bay_owner ran the recorded
        # CCCC on the Bambu_AMS_2 spare every session, and it measured 1800
        # there.
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}, boxed:{C}",
                  "name_map": f"{A}:Bambu_AMS_1", "lane_map": f"{A}:24:4"},
            "AFC_BambuAMS Bambu_AMS_2": {"afc_bowden_length": "1800.0"}})
        m = _bbx_boot(tmp_path, roster=f"boxed:{A}", **self.POOL3)
        assert m._roster_source == "option"
        assert m._learned_for(C) == {"afc_bowden_length": "1800.0"}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_2",
                            "afc_bowden_length") is None
        assert m._learned_notes == [
            (False, f"learned values stored under Bambu_AMS_2 now belong to "
                    f"its unit {C} (afc_bowden_length)")]
        _bbx_quiet(m.printer)

    @pytest.mark.parametrize("recorded, spares", [
        (f"boxed:{A}, boxed:{C}, boxed:{D}", ["Bambu_AMS_2"]),
        (f"boxed:{A}, boxed:{C}", ["Bambu_AMS_2", "Bambu_AMS_3"]),
    ], ids=["two-units", "two-spares"])
    def test_a_spare_value_is_not_guessed_among_several(self, tmp_path,
                                                         recorded, spares):
        updates = {SEC: {"roster": recorded, "name_map": f"{A}:Bambu_AMS_1",
                         "lane_map": f"{A}:24:4"}}
        for name in spares:
            updates[f"AFC_BambuAMS {name}"] = {"afc_bowden_length": "1800.0"}
        self._seed(tmp_path, updates)
        m = _bbx_boot(tmp_path, roster=f"boxed:{A}", **self.POOL3)
        assert m._learned_for(C) == {}
        assert m._learned_for(D) == {}
        assert m._learned_notes == [
            (False, f"learned values stored under {n} dropped -- no unit is "
                    f"recorded with that name") for n in spares]
        assert m._read_state().sections() == [SEC]
        _bbx_quiet(m.printer)

    def test_the_uid_record_wins_a_migration_conflict(self, tmp_path):
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}", "name_map": f"{A}:Bambu_AMS_1",
                  "lane_map": f"{A}:24:4"},
            self.LEARNED + A: {"afc_bowden_length": "3000.0"},
            "AFC_BambuAMS Bambu_AMS_1": {"afc_bowden_length": "3632.0",
                                         "afc_unload_bowden_length": "3500"}})
        m = _bbx_restart(tmp_path, **self.POOL3)
        assert m._learned_for(A) == {"afc_bowden_length": "3000.0",
                                     "afc_unload_bowden_length": "3500"}
        keys = _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_1")
        assert (keys["afc_bowden_length"],
                keys["afc_unload_bowden_length"]) == ("3000.0", "3500")
        assert m._learned_notes == [
            (False, f"learned values stored under Bambu_AMS_1 now belong to "
                    f"its unit {A} (afc_unload_bowden_length)")]
        _bbx_quiet(m.printer)

    def test_a_non_learned_key_is_not_migrated_or_folded(self, tmp_path):
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}", "name_map": f"{A}:Bambu_AMS_1",
                  "lane_map": f"{A}:24:4"},
            "AFC_BambuAMS Bambu_AMS_1": {"pool": "False", "heater": "True",
                                         "afc_bowden_length": "3632.0"}})
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text(
            "[AFC_BambuAMS Bambu_AMS_1]\nmeasure_on_insert : True\n")
        m = _bbx_restart(tmp_path, **self.POOL3)
        keys = _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_1")
        assert (keys["pool"], keys["measure_on_insert"],
                keys["afc_bowden_length"]) == ("True", "False", "3632.0")
        assert "heater" not in keys
        assert m._learned_for(A) == {"afc_bowden_length": "3632.0"}
        assert {s: dict(m._read_state().items(s))
                for s in m._read_state().sections()} == {
            SEC: {"roster": f"boxed:{A}", "name_map": f"{A}:Bambu_AMS_1",
                  "lane_map": f"{A}:24:4"},
            self.LEARNED + A: {"afc_bowden_length": "3632.0"}}
        assert autov.read_text() == self.AUTOV_HEADER
        assert m._learned_notes == [
            (False, f"learned values stored under Bambu_AMS_1 now belong to "
                    f"its unit {A} (afc_bowden_length); heater, pool "
                    f"dropped -- only a bowden length that is a positive "
                    f"number is kept"),
            (False, "auto_vars [AFC_BambuAMS Bambu_AMS_1]: measure_on_insert "
                    "not folded -- a unit takes only its bowden lengths from "
                    "there, each a positive number")]
        _bbx_quiet(m.printer)

    def test_migration_leaves_another_chains_sections_alone(self, tmp_path):
        # Bambu_AMS_B_1 is no name of this chain: it belongs to a chain
        # with its own prefix, which migrates it itself.
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}", "name_map": f"{A}:Bambu_AMS_1",
                  "lane_map": f"{A}:24:4"},
            "AFC_BambuAMS Bambu_AMS_1": {"afc_bowden_length": "3632.0"},
            "AFC_BambuAMS Bambu_AMS_B_1": {"afc_bowden_length": "1234"}})
        m = _bbx_restart(tmp_path, **self.POOL3)
        assert m._learned_for(A) == {"afc_bowden_length": "3632.0"}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_B_1",
                            "afc_bowden_length") == "1234"
        assert m._state_get("AFC_BambuAMS Bambu_AMS_1",
                            "afc_bowden_length") is None
        _bbx_quiet(m.printer)

    class _PoolUnit:
        """
        A pool bay's unit as the master drives it: claimed onto a uid,
        released, and handed what that uid learned.
        """

        def __init__(self, name: str) -> None:
            """
            :param name: the bay's unit name
            """
            self.name = name
            self.pool = True
            self.unit_uid: Optional[str] = None
            self.ams_model = "boxed"
            self.has_heater = False
            self.dry_max_temp = 65
            self.measure_on_insert = False
            self.master: Optional[afcBridgeBox] = None
            self.learned: List[Dict[str, Any]] = []

        def set_master(self, master: afcBridgeBox) -> None:
            """
            :param master: the chain master that owns the bay
            """
            self.master = master

        def claim(self, uid: str, model: str) -> bool:
            """
            :param uid: the unit claimed onto the bay
            :param model: its model
            :return bool: True, the claim took
            """
            self.unit_uid, self.ams_model, self.pool = uid, model, False
            return True

        def release(self) -> None:
            """Give the bay back to the pool."""
            self.unit_uid, self.pool = None, True

        def apply_learned(self, values: Dict[str, Any]) -> None:
            """
            :param values: what the claimed uid learned
            """
            self.learned.append(dict(values))

    def _pool_fakes(self, tmp_path: pathlib.Path,
                    monkeypatch: pytest.MonkeyPatch, bridge: FakeBridge,
                    **opts: Any) -> Tuple[afcBridgeBox, Dict[str, Any]]:
        """
        The three-AMS, one-HT pool chain from the recorded roster, its pool
        bays driven through _PoolUnit stand-ins in place of their units and
        lanes, watching ``bridge``.

        :param tmp_path: where the state and auto_vars files live
        :param monkeypatch: isolates the bridge table
        :param bridge: the chain's bridge
        :return tuple: (the master, pool unit name -> its stand-in)
        """
        printer = make_printer(monkeypatch=monkeypatch)
        merged: Dict[str, Any] = dict(self.POOL3, roster="")
        merged.update(opts)
        m = _bbx_boot(tmp_path, printer=printer, **merged)
        units: Dict[str, Any] = {}
        for bay in m._pool_units:
            for lane in bay["lanes"]:
                del printer._objects[f"AFC_lane {lane}"]
            units[bay["name"]] = self._PoolUnit(bay["name"])
            printer._objects[f"AFC_BambuAMS {bay['name']}"] = units[bay["name"]]
        live_bridges()[m.serial_port] = bridge
        return m, units

    def test_a_moved_unit_takes_its_values_to_the_new_bay_at_restart(
            self, tmp_path, monkeypatch):
        names = "Alpha, Bravo, Charlie"
        _bbx_record(tmp_path, f"boxed:{A}")
        m, units = self._pool_fakes(
            tmp_path, monkeypatch,
            FakeBridge(uids=[A, C], online=[False, True]), ams_names=names)
        assign = FakeGcmd({"UID": C, "NAME": "Bravo"})
        m.cmd_AFC_BRIDGEBOX_ASSIGN(assign)
        assert assign.messages == [
            ("respond_info", f"AFC_BridgeBox chain1: assigned {C} to bay "
                             f"'Bravo' (lane28-lane31, T28-T31) -- claimed "
                             f"LIVE, no restart.")]
        assert m.persist_learned("Bravo", "afc_bowden_length", 3632.0) is True
        force = FakeGcmd({"UID": C, "NAME": "Charlie", "FORCE": 1})
        m.cmd_AFC_BRIDGEBOX_ASSIGN(force)
        assert force.messages == [
            ("respond_info", f"AFC_BridgeBox chain1: assigned {C} to bay "
                             f"'Charlie' (lane32-lane35, T32-T35) -- claimed "
                             f"LIVE, no restart.")]
        # Alpha waits for the offline A; Bravo was handed nothing (C had
        # learned nothing yet); Charlie gets the length C learned on Bravo.
        assert {name: (u.unit_uid, u.pool, u.learned)
                for name, u in units.items()} == {
            "Alpha": (None, True, []),
            "Bravo": (None, True, [{}]),
            "Charlie": (C, False, [{"afc_bowden_length": 3632.0}]),
            "Bambu_AMS_HT_1": (None, True, [])}
        assert m.printer.afc.logger.messages == [
            ("info", f"AFC_BridgeBox chain1: CLAIMED {C} as boxed onto Bravo "
                     f"(0 lanes) -- live, no restart."),
            ("info", f"AFC_BridgeBox chain1: released Bravo (UID {C}, "
                     f"AFC_BRIDGEBOX_ASSIGN); lanes dropped live"),
            ("info", f"AFC_BridgeBox chain1: Charlie takes the "
                     f"afc_bowden_length 3632mm that UID {C} learned."),
            ("info", f"AFC_BridgeBox chain1: CLAIMED {C} as boxed onto "
                     f"Charlie (0 lanes) -- live, no restart.")]
        # Each ASSIGN closes the popup that could have run it, and each
        # claim saves AFC's var file.
        assert m.printer.gcode.messages == [
            ("respond_raw", "// action:prompt_end"),
            ("respond_raw", "// action:prompt_end")]
        assert m.printer.afc.save_vars.calls == [((), {}), ((), {})]
        m2 = _bbx_restart(tmp_path, ams_names=names, **self.POOL3)
        assert _bbx_uids(m2) == {"Alpha": A, "Bravo": "", "Charlie": C,
                                 "Bambu_AMS_HT_1": ""}
        assert _bbx_keys(m2, "AFC_BambuAMS Charlie")[
            "afc_bowden_length"] == "3632.0"
        assert "afc_bowden_length" not in _bbx_keys(m2, "AFC_BambuAMS Bravo")
        _bbx_quiet(m2.printer)

    def _assign_bravo(self, tmp_path: pathlib.Path,
                      monkeypatch: pytest.MonkeyPatch,
                      printer_change: Callable[[BambuPrinter], None]
                      ) -> Tuple[afcBridgeBox, Dict[str, Any]]:
        """
        The moved-unit chain, its printer changed by ``printer_change``,
        with CCCC assigned to Bravo; asserts the claim went through as in
        test_a_moved_unit_takes_its_values_to_the_new_bay_at_restart.

        :param tmp_path: where the state and auto_vars files live
        :param monkeypatch: isolates the bridge table
        :param printer_change: changes the chain's printer before the ASSIGN
        :return tuple: (the master, pool unit name -> its stand-in)
        """
        _bbx_record(tmp_path, f"boxed:{A}")
        m, units = self._pool_fakes(
            tmp_path, monkeypatch,
            FakeBridge(uids=[A, C], online=[False, True]),
            ams_names="Alpha, Bravo, Charlie")
        printer_change(m.printer)
        assign = FakeGcmd({"UID": C, "NAME": "Bravo"})
        m.cmd_AFC_BRIDGEBOX_ASSIGN(assign)
        assert assign.messages == [
            ("respond_info", f"AFC_BridgeBox chain1: assigned {C} to bay "
                             f"'Bravo' (lane28-lane31, T28-T31) -- claimed "
                             f"LIVE, no restart.")]
        assert m.printer.afc.logger.messages == [
            ("info", f"AFC_BridgeBox chain1: CLAIMED {C} as boxed onto Bravo "
                     f"(0 lanes) -- live, no restart.")]
        assert m.printer.gcode.messages == [
            ("respond_raw", "// action:prompt_end")]
        assert (units["Bravo"].unit_uid, units["Bravo"].pool) == (C, False)
        return m, units

    # Misplaced here: this tests _claim_pool_unit (via ASSIGN) and belongs in
    # TestAfcBridgeBoxClaimPoolUnit; the map flags it for the move.

    def test_a_claim_whose_var_file_save_fails_still_takes_the_bay(
            self, tmp_path, monkeypatch):
        tried: List[str] = []

        def _refuse() -> None:
            tried.append("save_vars")
            raise OSError("disk full")

        def _break_save(printer: BambuPrinter) -> None:
            printer.afc.save_vars = _refuse

        m, _units = self._assign_bravo(tmp_path, monkeypatch, _break_save)
        assert tried == ["save_vars"]
        assert m._owners() == {"Bravo": C}

    def test_a_name_taken_over_at_boot_keeps_its_values_with_its_holder(
            self, tmp_path):
        # ZZZZ is outside the roster and CCCC, new in it, draws ZZZZ's name
        # at this boot. What is stored under the name is ZZZZ's: CCCC
        # measures its own.
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}, boxed:{C}",
                  "name_map": f"{A}:Bambu_AMS_1, ZZZZ:Bambu_AMS_2",
                  "lane_map": f"{A}:24:4, ZZZZ:28:4"},
            "AFC_BambuAMS Bambu_AMS_2": {"afc_bowden_length": "3632.0"}})
        m = _bbx_restart(tmp_path, **self.POOL3)
        assert m._name_map == {A: "Bambu_AMS_1", C: "Bambu_AMS_2"}
        assert m._learned_for(C) == {}
        assert m._learned_for("ZZZZ") == {"afc_bowden_length": "3632.0"}
        assert "afc_bowden_length" not in _bbx_keys(
            m, "AFC_BambuAMS Bambu_AMS_2")
        assert m._learned_notes == [
            (False, f"learned values stored under Bambu_AMS_2 now belong to "
                    f"its unit ZZZZ (afc_bowden_length), which wore that name "
                    f"before {C} took it; {C} measures its own")]
        _bbx_quiet(m.printer)

    def test_a_record_that_is_not_a_length_is_never_folded(self, tmp_path):
        # The fold writes a record into the unit section, and the unit reads
        # it with getfloat(above=0): anything else would stop Klipper.
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}", "name_map": f"{A}:Bambu_AMS_1",
                  "lane_map": f"{A}:24:4"},
            self.LEARNED + A: {"afc_bowden_length": "abc",
                               "afc_unload_bowden_length": "-5",
                               "pool": "False"}})
        m = _bbx_restart(tmp_path, **self.POOL3)
        assert m._learned_for(A) == {}
        keys = _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_1")
        assert "afc_bowden_length" not in keys
        assert "afc_unload_bowden_length" not in keys
        assert keys["pool"] == "True"
        assert m._learned_notes == []
        _bbx_quiet(m.printer)

    class _PlainBridge(FakeBridge):
        """
        FakeBridge without chain_snapshot, as an older bridge: the watch
        reads the uids, flags and dialect through their separate getters.
        """

        chain_snapshot = None

    def test_a_length_that_is_not_finite_is_never_folded_or_applied(
            self, tmp_path, monkeypatch):
        self._seed(tmp_path, {
            SEC: {"roster": f"boxed:{A}", "name_map": f"{A}:Bambu_AMS_1",
                  "lane_map": f"{A}:24:4"},
            self.LEARNED + A: {"afc_bowden_length": "inf",
                               "afc_unload_bowden_length": "nan"},
            self.LEARNED + C: {"afc_bowden_length": "1e400"}})
        m = self._live(tmp_path, monkeypatch,
                       self._PlainBridge(uids=[A, C], online=[False, True]))
        keys = _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_1")
        assert "afc_bowden_length" not in keys
        assert "afc_unload_bowden_length" not in keys
        assert m._learned_for(C) == {}
        unit = m.printer.lookup_object("AFC_BambuAMS Bambu_AMS_2")
        unit.afc_bowden_length = unit.afc_unload_bowden_length = 1.0
        self._tick(m, 100.0)
        # The claim gave the unit no learned length: it took its default.
        assert unit.unit_uid == C
        assert (unit.afc_bowden_length, unit.afc_unload_bowden_length) == (
            DEFAULT_BOWDEN_MM, DEFAULT_BOWDEN_MM)
        assert m.printer.afc.logger.messages == self.CCCC_CLAIMED
        assert m.printer.gcode.messages == self.CCCC_PROMPT

    def test_an_auto_vars_value_that_is_not_a_length_is_not_folded(
            self, tmp_path):
        # A leftover auto_vars section for a rostered bay folds only a
        # usable length: getfloat(above=0) would stop this very boot.
        _bbx_record(tmp_path, f"boxed:{A}", name_map=f"{A}:Bambu_AMS_1",
                    lane_map=f"{A}:24:4")
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text("[AFC_BambuAMS Bambu_AMS_1]\nafc_bowden_length : 0\n"
                         "afc_unload_bowden_length : inf\n")
        m = _bbx_restart(tmp_path, **self.POOL3)
        keys = _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_1")
        assert "afc_bowden_length" not in keys
        assert "afc_unload_bowden_length" not in keys
        assert m._learned_for(A) == {}
        assert m._read_state().sections() == [SEC]
        assert autov.read_text() == self.AUTOV_HEADER
        assert m._learned_notes == [
            (False, "auto_vars [AFC_BambuAMS Bambu_AMS_1]: "
                    "afc_bowden_length, afc_unload_bowden_length not folded "
                    "-- a unit takes only its bowden lengths from there, each "
                    "a positive number")]
        _bbx_quiet(m.printer)

    def test_an_auto_vars_leftover_for_a_spare_is_swept_not_folded(
            self, tmp_path):
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text(
            "[AFC_BambuAMS Bambu_AMS_2]\nafc_bowden_length : 3632.0\n")
        _bbx_record(tmp_path, f"boxed:{A}")
        m = _bbx_restart(tmp_path, **self.POOL3)
        assert "afc_bowden_length" not in _bbx_keys(
            m, "AFC_BambuAMS Bambu_AMS_2")
        assert autov.read_text() == self.AUTOV_HEADER
        assert m._read_state().sections() == [SEC]
        assert m._learned_notes == [
            (False, "auto_vars [AFC_BambuAMS Bambu_AMS_2] swept, not folded "
                    "-- the bay is a spare, and learned values belong to the "
                    "unit that learned them")]
        _bbx_quiet(m.printer)

    def test_an_old_state_file_boots_cleanly(self, tmp_path):
        _bbx_record(tmp_path, f"boxed:{A}, ht:{H}",
                    name_map=f"{A}:Bambu_AMS_1, {H}:Bambu_AMS_HT_1",
                    lane_map=f"{A}:24:4, {H}:36:1")
        m = _bbx_restart(tmp_path, **self.POOL3)
        assert _bbx_uids(m) == {"Bambu_AMS_1": A, "Bambu_AMS_2": "",
                                "Bambu_AMS_3": "", "Bambu_AMS_HT_1": H}
        assert m._learned_notes == []
        assert m._read_state().sections() == [SEC]
        assert _bbx_ready(m) == []

    # two chains: the automatic lane base past what earlier chains built

    def test_a_second_chain_starts_past_every_lane_the_first_fabricated(
            self, tmp_path):
        printer = _bbx_steppers(make_printer())
        m1 = _bbx_chain(tmp_path, printer, "chain1")
        first = self._lane_numbers(m1)
        n = len(printer.loaded)
        m2 = _bbx_chain(tmp_path, printer, "chain2",
                        unit_prefix="Bambu_AMS_B")
        # lane4 is the highest declared lane. Chain 1: a one-AMS band at
        # 5-8, then an HT band of two at 9-10. Chain 2 repeats the shape
        # from 11: AMS 11-14, HTs 15-16.
        assert (m1.lane_base, m2.lane_base) == (5, 11)
        assert first == [5, 6, 7, 8, 9, 10]
        assert self._lane_numbers(m2, n) == [11, 12, 13, 14, 15, 16]
        _bbx_quiet(printer)

    def test_each_chain_keeps_its_own_locked_base(self, tmp_path):
        _bbx_two_chains(tmp_path, make_printer())
        # Next boot: the config has grown past both chains. Neither base
        # moves, and each is read from the chain's own state section.
        grown = make_printer()
        grown.add_section("AFC_stepper lane1", {})
        grown.add_section("AFC_lane lane40", {})
        a = _bbx_chain(tmp_path, grown, "chain1")
        b = _bbx_chain(tmp_path, grown, "chain2", unit_prefix="Bambu_AMS_B")
        assert (a.lane_base, b.lane_base) == (5, 11)
        assert a._state_get("AFC_BridgeBox chain1", "lane_base") == "5"
        assert b._state_get("AFC_BridgeBox chain2", "lane_base") == "11"
        _bbx_quiet(grown)

    def test_named_lane_machine_second_chain_continues_the_sequence(
            self, tmp_path):
        # A toolchanger with tools e0..e3 and no laneN anywhere: chain 1
        # continues at T4 (lanes 4-9), chain 2 past it at 10.
        printer = make_printer()
        for i in range(4):
            printer.add_section(f"AFC_extruder e{i}", {})
        m1 = _bbx_chain(tmp_path, printer, "chain1")
        m2 = _bbx_chain(tmp_path, printer, "chain2",
                        unit_prefix="Bambu_AMS_B")
        assert (m1.lane_base, m2.lane_base) == (4, 10)
        _bbx_quiet(printer)

    def test_a_first_chain_with_an_explicit_base_is_stepped_over(
            self, tmp_path):
        # Chain 1 is pinned at 30 (AMS 30-33, HTs 34-35), past every
        # declared lane; the automatic chain 2 starts after it.
        printer = _bbx_steppers(make_printer())
        _bbx_chain(tmp_path, printer, "chain1", lane_base=30)
        n = len(printer.loaded)
        m2 = _bbx_chain(tmp_path, printer, "chain2",
                        unit_prefix="Bambu_AMS_B")
        assert m2.lane_base == 36
        assert self._lane_numbers(m2, n) == [36, 37, 38, 39, 40, 41]
        _bbx_quiet(printer)

    def test_a_single_chain_base_is_unchanged(self, tmp_path):
        m = _bbx_chain(tmp_path, _bbx_steppers(make_printer()), "chain1")
        assert m.lane_base == 5
        assert self._lane_numbers(m) == [5, 6, 7, 8, 9, 10]
        _bbx_quiet(m.printer)

    def test_a_printer_that_cannot_list_objects_keeps_the_config_base(
            self, tmp_path):
        printer = _bbx_unlistable(_bbx_steppers(make_printer()))
        m = _bbx_chain(tmp_path, printer, "chain1")
        assert m.lane_base == 5
        assert m.buffer_chip_name == "bambu_buffer"
        _bbx_quiet(printer)

    # two chains: each chain's buffer chip

    @staticmethod
    def _holder(printer: BambuPrinter) -> BridgeBoxOverrideHolder:
        """
        Register the [AFC_BridgeBox ht] override holder, as klippy loads it
        above the chains.

        :param printer: the printer
        :return BridgeBoxOverrideHolder: the holder
        """
        holder = BridgeBoxOverrideHolder(BambuConfig(
            "AFC_BridgeBox ht", printer, {"measure_on_insert": "False"}))
        printer.add_object("AFC_BridgeBox ht", holder)
        return holder

    #: The buffer a chain fabricates, but for its chip.
    FPS = {"type": "FPS_PSF", "deadband": "0.48",
           "filament_error_sensitivity": "0"}

    def test_the_first_chain_keeps_the_plain_default(self, tmp_path):
        printer = _bbx_steppers(make_printer())
        self._holder(printer)
        m = _bbx_chain(tmp_path, printer, "chain1")
        assert m.buffer_chip_name == "bambu_buffer"
        assert _bbx_keys(m, "AFC_buffer Bambu_AMS_Buffer") == dict(
            self.FPS, adc_pin="bambu_buffer:fps")
        _bbx_quiet(printer)

    def test_a_second_chain_defaults_to_a_chip_of_its_own(self, tmp_path):
        m1, m2 = _bbx_two_chains(tmp_path, make_printer())
        assert m1.buffer_chip_name == "bambu_buffer"
        assert m2.buffer_chip_name == "bambu_buffer_chain2"
        assert _bbx_keys(m2, "AFC_buffer Bambu_AMS_B_Buffer") == dict(
            self.FPS, adc_pin="bambu_buffer_chain2:fps")
        _bbx_quiet(m2.printer)

    def test_every_unit_is_told_its_chains_chip(self, tmp_path):
        printer = make_printer()
        _bbx_two_chains(tmp_path, printer)
        chips = {s.split()[1]: w.fileconfig.get(s, "buffer_chip_name")
                 for s, w in printer.loaded if s.startswith("AFC_BambuAMS ")}
        assert chips == {
            "Bambu_AMS_1": "bambu_buffer",
            "Bambu_AMS_HT_1": "bambu_buffer",
            "Bambu_AMS_HT_2": "bambu_buffer",
            "Bambu_AMS_B_1": "bambu_buffer_chain2",
            "Bambu_AMS_B_HT_1": "bambu_buffer_chain2",
            "Bambu_AMS_B_HT_2": "bambu_buffer_chain2",
        }
        _bbx_quiet(printer)

    def test_an_explicit_chip_name_still_wins(self, tmp_path):
        _m1, m2 = _bbx_two_chains(tmp_path, make_printer(),
                                  buffer_chip_name="pico_b")
        assert m2.buffer_chip_name == "pico_b"
        assert _bbx_keys(m2, "AFC_buffer Bambu_AMS_B_Buffer") == dict(
            self.FPS, adc_pin="pico_b:fps")
        _bbx_quiet(m2.printer)

    def test_two_scouting_chains_register_two_chips(self, tmp_path):
        # With no roster and no pool a master registers its chip itself,
        # under the same per-chain name its units would use.
        pins = self._FakePins()
        printer = _bbx_steppers(make_printer())
        printer.add_object("pins", pins)
        for name in ("chain1", "chain2"):
            m = _bbx_chain(tmp_path, printer, name, roster="", pool_ams=0,
                           pool_ht=0)
            assert m._roster_source == "scout"
        assert printer.loaded == []
        assert sorted(pins.chips) == ["bambu_buffer", "bambu_buffer_chain2"]
        assert {type(c) for c in pins.chips.values()} == {_BambuBufferChip}
        _bbx_quiet(printer)

    def test_a_buffer_on_the_plain_chip_below_a_scout_is_adopted(
            self, tmp_path):
        # chain1 only watches its bridge, so bambu_buffer is its stub chip:
        # the hand-written buffer on it belongs to chain2, as it did when
        # every chain defaulted to bambu_buffer, and chain2 fabricates none.
        printer = make_printer()
        printer.add_section("AFC_stepper lane4", {})
        printer.add_section("AFC_buffer Bambu_AMS_Buffer",
                            {"adc_pin": "bambu_buffer:fps"})
        _bbx_chain(tmp_path, printer, "chain1", roster="", pool_ams=0,
                   pool_ht=0)
        m2 = _bbx_chain(tmp_path, printer, "chain2", roster="", pool_ams=2,
                        pool_ht=1)
        assert m2.buffer_chip_name == "bambu_buffer"
        assert m2.buffer == "Bambu_AMS_Buffer"
        assert [s for s in _bbx_loaded(m2)
                if not s.startswith("AFC_lane ")] == [
            "AFC_BambuAMS Bambu_AMS_1", "AFC_hub Bambu_AMS_1",
            "AFC_BambuAMS Bambu_AMS_2", "AFC_hub Bambu_AMS_2",
            "AFC_BambuAMS Bambu_AMS_HT_1", "AFC_hub Bambu_AMS_HT_1"]
        assert {_bbx_keys(m2, s)["buffer_chip_name"]
                for s in _bbx_loaded(m2)
                if s.startswith("AFC_BambuAMS ")} == {"bambu_buffer"}
        _bbx_quiet(printer)

    def test_a_chip_name_below_a_scout_blocks_the_adoption(self, tmp_path):
        # The same scout above, but chain2 names its own chip: the buffer on
        # bambu_buffer is not chain2's, so it fabricates one on pico_b.
        printer = make_printer()
        printer.add_section("AFC_stepper lane4", {})
        printer.add_section("AFC_buffer Hand", {"adc_pin": "bambu_buffer:fps"})
        _bbx_chain(tmp_path, printer, "chain1", roster="", pool_ams=0,
                   pool_ht=0)
        m2 = _bbx_chain(tmp_path, printer, "chain2", roster="", pool_ams=2,
                        pool_ht=1, buffer_chip_name="pico_b")
        assert m2.buffer_chip_name == "pico_b"
        assert m2.buffer == "Bambu_AMS_Buffer"
        assert m2._fabricate_buffer is True
        assert _bbx_loaded(m2)[-1] == "AFC_buffer Bambu_AMS_Buffer"
        assert _bbx_keys(m2, "AFC_buffer Bambu_AMS_Buffer") == dict(
            self.FPS, adc_pin="pico_b:fps")
        assert "AFC_buffer Hand" not in _bbx_loaded(m2)
        _bbx_quiet(printer)

    def test_a_chain_below_a_real_chain_keeps_its_own_chip(self, tmp_path):
        printer = make_printer()
        printer.add_section("AFC_stepper lane4", {})
        printer.add_section("AFC_buffer Hand", {"adc_pin": "bambu_buffer:fps"})
        m1 = _bbx_chain(tmp_path, printer, "chain1")
        m2 = _bbx_chain(tmp_path, printer, "chain2",
                        unit_prefix="Bambu_AMS_B")
        assert m1.buffer == "Hand"
        assert m2.buffer_chip_name == "bambu_buffer_chain2"
        assert m2.buffer == "Bambu_AMS_B_Buffer"
        assert _bbx_loaded(m2)[-1] == "AFC_buffer Bambu_AMS_B_Buffer"
        _bbx_quiet(printer)

    def test_an_override_section_cannot_rewire_one_units_chip(self, tmp_path):
        # The chip is chain wiring, like buffer: a per-unit override leaves
        # the unit on the chip its chain's buffer reads.
        printer = make_printer()
        printer.add_section("AFC_stepper lane4", {})
        printer.add_section("AFC_BridgeBox Bambu_AMS_HT_1",
                            {"buffer_chip_name": "elsewhere"})
        m = _bbx_chain(tmp_path, printer, "chain1")
        assert _bbx_keys(m, "AFC_BambuAMS Bambu_AMS_HT_1")[
            "buffer_chip_name"] == "bambu_buffer"
        _bbx_quiet(printer)

    # two chains: one AFC_auto_vars.cfg

    SHARED_AUTOV = ("[AFC_BambuAMS Bambu_AMS_B_HT_1]\n"
                    "afc_bowden_length : 3632.0\n\n"
                    "[AFC_BambuAMS Gone]\n"
                    "afc_bowden_length : 1500.0\n")

    def _shared(self, tmp_path: pathlib.Path) -> BambuPrinter:
        """
        Both chains fold from one AFC_auto_vars.cfg, and the second has not
        fabricated its units while the first loads.

        :param tmp_path: where the shared files live
        :return BambuPrinter: a printer whose merged config holds both
            masters, an override holder and the auto_vars sections klippy
            parsed
        """
        (tmp_path / "AFC_auto_vars.cfg").write_text(self.SHARED_AUTOV)
        printer = _bbx_steppers(make_printer())
        for name in ("chain1", "chain2"):
            printer.add_section(
                f"AFC_BridgeBox {name}",
                {"serial_port": f"/dev/serial/by-id/usb-{name}-if00"})
            if name == "chain1":
                printer.add_section("AFC_BridgeBox ht",
                                    {"measure_on_insert": "False"})
        printer.add_section("AFC_BambuAMS Bambu_AMS_B_HT_1",
                            {"afc_bowden_length": "3632.0"})
        printer.add_section("AFC_BambuAMS Gone",
                            {"afc_bowden_length": "1500.0"})
        return printer

    def test_the_second_chain_folds_its_own_sections(self, tmp_path):
        autov = tmp_path / "AFC_auto_vars.cfg"
        # What chain2 learned for BBBB is kept under chain2's name.
        self._seed(tmp_path, {"AFC_BridgeBox chain2 learned BBBB":
                              {"afc_unload_bowden_length": "999.0"}})
        printer = self._shared(tmp_path)
        m1 = _bbx_chain(tmp_path, printer, "chain1")
        assert autov.read_text() == self.SHARED_AUTOV
        assert m1._learned_for(B) == {}
        m2 = _bbx_chain(tmp_path, printer, "chain2",
                        unit_prefix="Bambu_AMS_B")
        keys = _bbx_keys(m2, "AFC_BambuAMS Bambu_AMS_B_HT_1")
        assert (keys["afc_bowden_length"],
                keys["afc_unload_bowden_length"]) == ("3632.0", "999.0")
        assert m2._learned_for(B) == {"afc_bowden_length": "3632.0",
                                      "afc_unload_bowden_length": "999.0"}
        # The last chain sweeps: its own section folded, the orphan gone.
        assert autov.read_text() == self.AUTOV_HEADER
        _bbx_quiet(printer)

    def test_a_last_chain_that_fabricates_nothing_sweeps(self, tmp_path):
        # chain2 only watches its bridge (no roster, no pool), and it is the
        # last master: it sweeps, so the orphan does not come back at every
        # boot as an offline unit.
        autov = tmp_path / "AFC_auto_vars.cfg"
        printer = self._shared(tmp_path)
        _bbx_chain(tmp_path, printer, "chain1")
        assert autov.read_text() == self.SHARED_AUTOV
        m2 = _bbx_chain(tmp_path, printer, "chain2",
                        unit_prefix="Bambu_AMS_B", roster="", pool_ams=0,
                        pool_ht=0)
        assert m2._roster_source == "scout"
        assert m2.units == []
        assert autov.read_text() == self.AUTOV_HEADER
        _bbx_quiet(printer)

    def test_a_first_chain_that_fabricates_nothing_leaves_it_to_the_last(
            self, tmp_path):
        autov = tmp_path / "AFC_auto_vars.cfg"
        printer = self._shared(tmp_path)
        _bbx_chain(tmp_path, printer, "chain1", roster="", pool_ams=0,
                   pool_ht=0)
        assert autov.read_text() == self.SHARED_AUTOV
        m2 = _bbx_chain(tmp_path, printer, "chain2",
                        unit_prefix="Bambu_AMS_B")
        assert _bbx_keys(m2, "AFC_BambuAMS Bambu_AMS_B_HT_1")[
            "afc_bowden_length"] == "3632.0"
        assert autov.read_text() == self.AUTOV_HEADER
        _bbx_quiet(printer)

    # two chains: a start another chain stopped

    @staticmethod
    def _pair(tmp_path: pathlib.Path, **over: Any
              ) -> Tuple[afcBridgeBox, Any, List[LogLine]]:
        """
        Load chain1 (``over`` its options) and chain2 below it and, when
        both load, run klippy:ready for each. The loads log nothing and
        nothing reaches the printer's gcode (asserted).

        :param tmp_path: where the shared files live
        :return tuple: (chain1, chain2 or the error that stopped it, what
            klippy:ready logged)
        """
        printer = _bbx_steppers(make_printer())
        m1 = _bbx_chain(tmp_path, printer, "chain1", **over)
        try:
            m2 = _bbx_chain(tmp_path, printer, "chain2",
                            unit_prefix="Bambu_AMS_B")
        except configparser.Error as err:
            _bbx_quiet(printer)
            return m1, err, []
        _bbx_quiet(printer)
        m1._scout_ready()
        m2._scout_ready()
        assert printer.gcode.messages == []
        return m1, m2, list(printer.afc.logger.messages)

    def test_reverting_the_change_that_stopped_it_starts_again(
            self, tmp_path):
        m1, m2, logs = self._pair(tmp_path)
        assert m1._lane_map[A] == (9, 1)
        assert m2.lane_base == 11
        assert logs == []
        # pool_ams 2 moves chain1's HT onto lane13, inside chain2's lanes.
        m1, err, logs = self._pair(tmp_path, pool_ams=2)
        assert m1._lane_map[A] == (13, 1)
        assert str(err) == (
            f"[AFC_BridgeBox chain2] would fabricate [AFC_lane lane11], but "
            f"[AFC_BridgeBox chain1] above it in the config already builds "
            f"it: this chain's lane_base 11 (saved in the #~# block of "
            f"{tmp_path / 'AFC_BridgeBox.cfg'}, used while lane_base: is "
            f"unset or 0) falls inside that chain's lanes. Set lane_base: in "
            f"one chain's section past the other chain's last lane.")
        moved = [("warning", f"AFC_BridgeBox chain1: HT {A} (Bambu_AMS_HT_1) "
                             f"keeps its name, and its lanes and T# changed: "
                             f"lane13 (T13) -> lane9 (T9).")]
        for notes in (moved, []):
            m1, m2, logs = self._pair(tmp_path)
            assert isinstance(m2, afcBridgeBox)
            assert m1._lane_map[A] == (9, 1)
            assert logs == notes

    def test_a_lane_base_given_in_the_section_is_named(self, tmp_path):
        printer = _bbx_steppers(make_printer())
        _bbx_chain(tmp_path, printer, "chain1", lane_base=24)
        with pytest.raises(configparser.Error) as err:
            _bbx_chain(tmp_path, printer, "chain2", unit_prefix="Bambu_AMS_B",
                       lane_base=26)
        assert str(err.value) == (
            "[AFC_BridgeBox chain2] would fabricate [AFC_lane lane26], but "
            "[AFC_BridgeBox chain1] above it in the config already builds "
            "it: this chain's lane_base 26 (set by lane_base: in this "
            "section) falls inside that chain's lanes. Set lane_base: in one "
            "chain's section past the other chain's last lane.")
        _bbx_quiet(printer)

    def test_a_unit_name_both_chains_build_names_unit_prefix(self, tmp_path):
        printer = _bbx_steppers(make_printer())
        _bbx_chain(tmp_path, printer, "chain1")
        with pytest.raises(configparser.Error) as err:
            _bbx_chain(tmp_path, printer, "chain2", roster=f"ht:{A}")
        assert str(err.value) == (
            "[AFC_BridgeBox chain2] would fabricate [AFC_BambuAMS "
            "Bambu_AMS_1], but [AFC_BridgeBox chain1] above it in the config "
            "already builds it. Give one chain its own unit_prefix, or its "
            "own ams_names / ht_names.")
        _bbx_quiet(printer)

    def test_a_section_of_the_config_is_still_named_as_one(self, tmp_path):
        printer = _bbx_steppers(make_printer())
        printer.add_object("AFC_hub Bambu_AMS_1", object())
        with pytest.raises(configparser.Error) as err:
            _bbx_chain(tmp_path, printer, "chain1")
        assert str(err.value) == (
            "[AFC_BridgeBox chain1] would fabricate [AFC_hub Bambu_AMS_1] but "
            "it already exists in the config -- remove one")
        _bbx_quiet(printer)

    # two chains: a held HT band makes way for a later chain

    def _held_pair(self, tmp_path: pathlib.Path,
                   stored: Optional[str] = None, **chain2: Any
                   ) -> Tuple[afcBridgeBox, afcBridgeBox, List[str]]:
        """
        chain1 recorded HT HHHH on lane40 behind four AMS bays and now
        builds one; chain2 below it builds two AMS bays.

        :param tmp_path: where the shared files live
        :param stored: the lane base chain2's state records
        :param chain2: chain2's options in the merged config
        :return tuple: (chain1, chain2, chain1's layout notes)
        """
        self._seed(tmp_path, {SEC: {
            "roster": f"ht:{H}", "lane_base": "24", "ams_band": "4",
            "lane_map": f"{H}:40:1", "name_map": f"{H}:Bambu_AMS_HT_1"}})
        if stored:
            self._seed(tmp_path,
                       {"AFC_BridgeBox chain2": {"lane_base": stored}})
        printer = make_printer()
        printer.add_section(SEC, {
            "serial_port": "/dev/serial/by-id/usb-chain1-if00"})
        printer.add_section("AFC_BridgeBox chain2", dict(
            {"serial_port": "/dev/serial/by-id/usb-chain2-if00",
             "state_file": str(tmp_path / "AFC_BridgeBox.cfg"),
             "pool_ams": "2", "pool_ht": "0"},
            **{k: str(v) for k, v in chain2.items()}))
        m1 = _bbx_chain(tmp_path, printer, "chain1", roster="", lane_base=24,
                        pool_ams=1, pool_ht=2)
        notes = list(m1._layout_notes)
        m2 = _bbx_chain(tmp_path, printer, "chain2", roster="",
                        unit_prefix="Bambu_AMS_B", pool_ams=2, pool_ht=0,
                        lane_base=int(chain2.get("lane_base", 0)))
        return m1, m2, notes

    HT_MOVED = [
        f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) on lane40 (T40) "
        f"cannot keep its lanes: [AFC_BridgeBox chain2] further down the "
        f"config builds lane40. The AMS band is 1 bays, what pool_ams and "
        f"the recorded AMS need, and the HT lanes follow it.",
        f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) keeps its name, and "
        f"its lanes and T# changed: lane40 (T40) -> lane28 (T28)."]

    def test_an_explicit_lane_base_over_the_ht_lane_moves_it(self, tmp_path):
        m1, m2, notes = self._held_pair(tmp_path, lane_base=36)
        assert m1._lane_map[H] == (28, 1)
        assert m2.lane_base == 36
        assert notes == self.HT_MOVED
        _bbx_quiet(m1.printer)

    def test_a_saved_lane_base_over_the_ht_lane_moves_it(self, tmp_path):
        m1, m2, notes = self._held_pair(tmp_path, stored="36")
        assert m1._lane_map[H] == (28, 1)
        assert m2.lane_base == 36
        assert notes == self.HT_MOVED
        _bbx_quiet(m1.printer)

    def test_a_chain_clear_of_the_ht_lane_leaves_it(self, tmp_path):
        m1, m2, notes = self._held_pair(tmp_path, lane_base=28)
        assert m1._lane_map[H] == (40, 1)
        assert m2.lane_base == 28
        assert notes == [self._held_note(1)]
        _bbx_quiet(m1.printer)

    def test_a_chain_with_its_base_still_to_compute_leaves_it(self,
                                                             tmp_path):
        m1, m2, notes = self._held_pair(tmp_path)
        assert m1._lane_map[H] == (40, 1)
        assert m2.lane_base == 42
        assert notes == [self._held_note(1)]
        _bbx_quiet(m1.printer)


class TestAfcBridgeBoxRosterSections:
    """The sections, and their keys, a master's roster and pool imply."""

    @staticmethod
    def _sections(master: afcBridgeBox) -> Dict[str, Dict[str, Any]]:
        """
        :param master: a built master
        :return dict: section -> keys, as _roster_sections gives them for
            the master's units, which it does without logging
        """
        sections = dict(master._roster_sections(master.units))
        _bbx_quiet(master.printer)
        return sections

    @staticmethod
    def _pooled_ams(name: str, first: int, uid: str) -> Dict[str, Any]:
        """
        :param name: the bay's name
        :param first: its first lane number
        :param uid: the unit that owns it
        :return dict: the _pool_units entry of that owned four-lane AMS bay
        """
        return {"name": name,
                "lanes": [f"lane{n}" for n in range(first, first + 4)],
                "family": "ams", "uid": uid, "bound": None, "spare": False}

    @staticmethod
    def _unit(name: str, uid: Optional[str], model: str,
              measure: bool = False, buffer: str = "Bamb_1",
              **extra: Any) -> Dict[str, Any]:
        """
        :param name: the unit's name
        :param uid: its unit_uid; None for a spare, which has no such key
        :param model: its ams_model
        :param measure: its measure_on_insert
        :param buffer: the buffer it names
        :param extra: the heater keys, for a unit that dries
        :return dict: the keys of its fabricated [AFC_BambuAMS] section
        """
        keys: Dict[str, Any] = {
            "serial_port": "/dev/serial/by-id/usb-chain1-if00",
            "ams_model": model, "extruder": "extruder",
            "hub": name, "auto_error_recovery": True,
            "measure_on_insert": measure, "buffer": buffer,
            "buffer_chip_name": "bambu_buffer", "pool": True}
        if uid is not None:
            keys["unit_uid"] = uid
        keys.update(extra)
        return keys

    @staticmethod
    def _lane(unit: str, slot: int) -> Dict[str, Any]:
        """:return dict: the keys of the lane on ``unit``'s ``slot``"""
        return {"unit": f"{unit}:{slot}", "unassigned": True}

    #: A fabricated hub's keys with the master's default bowden lengths.
    HUB = {"switch_pin": "virtual", "afc_bowden_length": 2100.0,
           "afc_unload_bowden_length": 2100.0, "td1_bowden_length": 850.0}

    @staticmethod
    def _sensor(unit: str) -> Dict[str, Any]:
        """:return dict: the keys of ``unit``'s temperature card sensor"""
        return {"sensor_type": "aht2x", "bambu_unit": unit, "min_temp": 0,
                "max_temp": 90}

    #: The buffer a chain fabricates, but for its type and chip.
    FPS = {"deadband": 0.48, "filament_error_sensitivity": 0}

    @staticmethod
    def _slots(*bays: Tuple[int, int, str]) -> Dict[str, str]:
        """
        :param bays: (first lane, lane count, unit name) per bay
        :return dict: lane name -> "unit:slot", as the lanes those bays
            make name them
        """
        return {f"lane{first + i}": f"{name}:{i + 1}"
                for first, count, name in bays for i in range(count)}

    def _lane_units(self, master: afcBridgeBox) -> Dict[str, str]:
        """
        :param master: a built master
        :return dict: lane name -> the "unit:slot" its section names
        """
        return {s.split()[1]: keys["unit"]
                for s, keys in self._sections(master).items()
                if s.startswith("AFC_lane ")}

    def _uids(self, master: afcBridgeBox) -> Dict[str, Optional[str]]:
        """
        :param master: a built master
        :return dict: unit section -> its unit_uid, None for a spare
        """
        return {s: keys.get("unit_uid")
                for s, keys in self._sections(master).items()
                if s.startswith("AFC_BambuAMS ")}

    # the fabricated shape, held exactly

    def test_the_unit_keys_mirror_the_handwritten_block(self, tmp_path):
        m = _bbx_boot(tmp_path)
        u = self._sections(m)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u == self._unit("Bambu_AMS_HT_1", HT_UID, "ht", measure=True,
                               heater=True, dry_max_temp=85)
        # getint on the consuming side: "85.0" halts the printer at config
        # parse, so the value must be an int, stringified with no point.
        assert type(u["dry_max_temp"]) is int

    def test_a_master_tcp_key_is_passed_to_its_units(self, tmp_path):
        m = _bbx_boot(tmp_path, tcp_key="k")
        u = self._sections(m)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u == dict(self._unit("Bambu_AMS_HT_1", HT_UID, "ht",
                                    measure=True, heater=True,
                                    dry_max_temp=85), tcp_key="k")

    def test_the_lane_points_at_its_unit_and_slot(self, tmp_path):
        m = _bbx_boot(tmp_path)
        assert self._sections(m)["AFC_lane lane24"] == self._lane(
            "Bambu_AMS_HT_1", 1)

    def test_the_hub_is_virtual_with_the_masters_bowden(self, tmp_path):
        m = _bbx_boot(tmp_path, afc_bowden_length=1900.0,
                      td1_bowden_length=700.0)
        assert self._sections(m)["AFC_hub Bambu_AMS_HT_1"] == {
            "switch_pin": "virtual", "afc_bowden_length": 1900.0,
            "afc_unload_bowden_length": 1900.0, "td1_bowden_length": 700.0}

    def test_every_unit_gets_a_temperature_card_sensor(self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"ht:{A}, boxed:{B}")
        sections = self._sections(m)
        assert {s: keys for s, keys in sections.items()
                if s.startswith("temperature_sensor ")} == {
            "temperature_sensor Bambu_AMS_1": self._sensor("Bambu_AMS_1"),
            "temperature_sensor Bambu_AMS_HT_1": self._sensor(
                "Bambu_AMS_HT_1")}

    def test_an_ams2_pin_gets_its_own_ceiling_not_the_hts(self, tmp_path):
        # The generation steers heater and ceiling but not the name.
        m = _bbx_boot(tmp_path, roster=f"ht:{A}, ams2:{B}")
        sections = self._sections(m)
        assert sections["AFC_BambuAMS Bambu_AMS_HT_1"]["dry_max_temp"] == 85
        assert sections["AFC_BambuAMS Bambu_AMS_1"] == self._unit(
            "Bambu_AMS_1", B, "ams2", heater=True, dry_max_temp=65)

    def test_an_explicit_dry_max_temp_still_caps_every_heater(self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"ht:{A}, ams2:{B}", dry_max_temp=60)
        sections = self._sections(m)
        assert (sections["AFC_BambuAMS Bambu_AMS_HT_1"]["dry_max_temp"],
                sections["AFC_BambuAMS Bambu_AMS_1"]["dry_max_temp"]) == (
            60, 60)

    def test_a_chain_dry_max_temp_caps_but_never_raises_a_ceiling(
            self, tmp_path):
        # 80 holds the HT (85) down to 80 but leaves the AMS 2 Pro at its
        # own 65: the chain value is a cap, not a replacement.
        m = _bbx_boot(tmp_path, roster=f"ht:{A}, ams2:{B}", dry_max_temp=80)
        sections = self._sections(m)
        assert (sections["AFC_BambuAMS Bambu_AMS_HT_1"]["dry_max_temp"],
                sections["AFC_BambuAMS Bambu_AMS_1"]["dry_max_temp"]) == (
            80, 65)

    def test_an_unconfirmed_boxed_unit_gets_no_heater_key(self, tmp_path):
        m = _bbx_boot(tmp_path, roster="boxed:AAAABBBBCCCCDDDD")
        assert self._sections(m)["AFC_BambuAMS Bambu_AMS_1"] == self._unit(
            "Bambu_AMS_1", "AAAABBBBCCCCDDDD", "boxed")

    def test_ams_takes_the_low_band_ht_the_high_band_regardless_of_order(
            self, tmp_path):
        # The HT is listed first, yet the boxed AMS takes the low block and
        # the HT its own band above it.
        m = _bbx_boot(tmp_path,
                      roster="ht:1111222233334444, boxed:5555666677778888")
        assert [s for s in _bbx_loaded(m) if s.startswith("AFC_lane ")] == \
            _bbx_lanes_span(24, 28)
        assert self._lane_units(m) == self._slots(
            (24, 4, "Bambu_AMS_1"), (28, 1, "Bambu_AMS_HT_1"))

    def test_the_same_roster_always_fabricates_the_same_names(self, tmp_path):
        # Spoolman bindings and T# macros hang off these names; a roster
        # that renamed anything between restarts would orphan them all.
        roster = "ht:1111222233334444, boxed:5555666677778888"
        built = []
        for sub in ("a", "b"):
            (tmp_path / sub).mkdir()
            built.append(_bbx_boot(tmp_path / sub, roster=roster))
        a, b = built
        assert a._roster_sections(a.units) == b._roster_sections(b.units)
        _bbx_quiet(b.printer)
        assert list(self._sections(a)) == [
            "AFC_BambuAMS Bambu_AMS_1", *_bbx_lanes_span(24, 27),
            "AFC_hub Bambu_AMS_1", "temperature_sensor Bambu_AMS_1",
            "AFC_BambuAMS Bambu_AMS_HT_1", "AFC_lane lane28",
            "AFC_hub Bambu_AMS_HT_1", "temperature_sensor Bambu_AMS_HT_1"]

    # fixed bands: AMS low, HT high, no collapse

    def test_two_ams_then_ht_pack_12_16_20(self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{B}, ht:{C}",
                      lane_base=12)
        assert self._lane_units(m) == self._slots(
            (12, 4, "Bambu_AMS_1"), (16, 4, "Bambu_AMS_2"),
            (20, 1, "Bambu_AMS_HT_1"))

    def test_every_ams_sits_below_the_ht_band(self, tmp_path):
        m = _bbx_boot(tmp_path,
                      roster=f"boxed:{A}, boxed:{B}, ht:{C}, boxed:{D}",
                      lane_base=12)
        assert self._lane_units(m) == self._slots(
            (12, 4, "Bambu_AMS_1"), (16, 4, "Bambu_AMS_2"),
            (20, 4, "Bambu_AMS_3"), (24, 1, "Bambu_AMS_HT_1"))

    def test_ht_never_takes_an_ams_band_lane_even_first_in_roster(
            self, tmp_path):
        m = _bbx_boot(tmp_path, roster=f"ht:{C}, boxed:{A}", lane_base=12)
        assert self._lane_units(m) == self._slots(
            (12, 4, "Bambu_AMS_1"), (16, 1, "Bambu_AMS_HT_1"))

    def test_declared_pool_ams_pins_the_ht_band_so_it_cannot_collapse(
            self, tmp_path):
        # A declared pool reserves the AMS band at its full width, so the HT
        # band starts at lane_base+16 however many AMS are listed; without
        # one it follows the listed AMS.
        lanes = {}
        for sub, roster, pool in (
                ("one", f"boxed:{A}, ht:{C}", 4),
                ("four", f"boxed:{A}, boxed:{B}, boxed:{E}, boxed:GGGG, "
                         f"ht:{C}", 4),
                ("none", f"boxed:{A}, ht:{C}", 0)):
            (tmp_path / sub).mkdir()
            lanes[sub] = self._lane_units(_bbx_boot(
                tmp_path / sub, roster=roster, lane_base=12, pool_ams=pool,
                pool_ht=8 if pool else 0))
        pooled = self._slots(
            (12, 4, "Bambu_AMS_1"), (16, 4, "Bambu_AMS_2"),
            (20, 4, "Bambu_AMS_3"), (24, 4, "Bambu_AMS_4"),
            *[(28 + i, 1, f"Bambu_AMS_HT_{i + 1}") for i in range(8)])
        assert lanes["one"] == lanes["four"] == pooled
        assert lanes["none"] == self._slots((12, 4, "Bambu_AMS_1"),
                                            (16, 1, "Bambu_AMS_HT_1"))

    def test_a_removed_tombstone_never_renumbers_a_survivor(self, tmp_path):
        # Removed (not forgotten), AAAA keeps its lanes and name: BBBB is
        # never renumbered, and DDDD takes the next free block and name.
        _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{B}", lane_base=12)
        m2 = _bbx_boot(tmp_path, roster=f"boxed:{B}, boxed:{D}",
                       lane_base=12)
        assert self._uids(m2) == {"AFC_BambuAMS Bambu_AMS_2": B,
                                  "AFC_BambuAMS Bambu_AMS_3": D}
        assert self._lane_units(m2) == self._slots(
            (16, 4, "Bambu_AMS_2"), (20, 4, "Bambu_AMS_3"))
        self._sections(m2)
        assert m2._lane_map == {A: (12, 4), B: (16, 4), D: (20, 4)}
        assert m2._name_map == {A: "Bambu_AMS_1", B: "Bambu_AMS_2",
                                D: "Bambu_AMS_3"}
        assert m2._pool_units == [self._pooled_ams("Bambu_AMS_2", 16, B),
                                  self._pooled_ams("Bambu_AMS_3", 20, D)]

    # the chain's fabricated buffer

    def test_no_buffer_option_fabricates_one_for_the_chain(self, tmp_path):
        m = _bbx_boot(tmp_path, buffer=None)
        assert _bbx_loaded(m)[-1] == "AFC_buffer Bambu_AMS_Buffer"
        # The two values tuned on hardware: the AMS buffer's swing, and jam
        # detection off because the AMS meters its own moves.
        assert self._sections(m)["AFC_buffer Bambu_AMS_Buffer"] == dict(
            self.FPS, type="FPS_PSF", adc_pin="bambu_buffer:fps")

    def test_every_unit_on_the_chain_references_it(self, tmp_path):
        m = _bbx_boot(tmp_path, buffer=None, roster=f"ht:{A}, boxed:{B}")
        assert {s: keys["buffer"] for s, keys in self._sections(m).items()
                if s.startswith("AFC_BambuAMS ")} == {
            "AFC_BambuAMS Bambu_AMS_1": "Bambu_AMS_Buffer",
            "AFC_BambuAMS Bambu_AMS_HT_1": "Bambu_AMS_Buffer"}

    def test_buffer_type_defaults_to_the_plain_tension_follower(
            self, tmp_path):
        m = _bbx_boot(tmp_path, buffer=None)
        assert self._sections(m)["AFC_buffer Bambu_AMS_Buffer"][
            "type"] == "FPS_PSF"

    def test_buffer_type_bambu_fabricates_the_gated_buffer(self, tmp_path):
        # Only the type changes: same pin, same deadband, jam detection off.
        m = _bbx_boot(tmp_path, buffer=None, buffer_type="bambu")
        assert self._sections(m)["AFC_buffer Bambu_AMS_Buffer"] == dict(
            self.FPS, type="bambu", adc_pin="bambu_buffer:fps")

    def test_buffer_type_never_reaches_a_unit_section(self, tmp_path):
        m = _bbx_boot(tmp_path, buffer=None, buffer_type="bambu")
        assert self._sections(m)["AFC_BambuAMS Bambu_AMS_HT_1"] == \
            self._unit("Bambu_AMS_HT_1", HT_UID, "ht", measure=True,
                       buffer="Bambu_AMS_Buffer", heater=True,
                       dry_max_temp=85)

    def test_a_second_chain_gets_its_own_buffer_name(self, tmp_path):
        m = _bbx_boot(tmp_path, buffer=None, unit_prefix="Bambu_AMS_B",
                      buffer_chip_name="bambu_buffer_b")
        assert self._sections(m)["AFC_buffer Bambu_AMS_B_Buffer"] == dict(
            self.FPS, type="FPS_PSF", adc_pin="bambu_buffer_b:fps")

    # lane and name tombstones

    def test_removing_a_unit_never_renumbers_or_renames_survivors(
            self, tmp_path):
        # Roster-order allocation alone would hand BBBB lane24 and the name
        # Bambu_AMS_1 the moment AAAA left, with AAAA's Spoolman bindings.
        _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{B}")
        m2 = _bbx_boot(tmp_path, roster=f"boxed:{B}")
        assert list(self._sections(m2)) == [
            "AFC_BambuAMS Bambu_AMS_2", *_bbx_lanes_span(28, 31),
            "AFC_hub Bambu_AMS_2", "temperature_sensor Bambu_AMS_2"]
        assert self._lane_units(m2) == self._slots((28, 4, "Bambu_AMS_2"))
        assert m2._lane_map == {A: (24, 4), B: (28, 4)}
        assert m2._name_map == {A: "Bambu_AMS_1", B: "Bambu_AMS_2"}
        assert m2._pool_units == [self._pooled_ams("Bambu_AMS_2", 28, B)]

    def test_a_returning_unit_gets_its_lanes_and_name_back(self, tmp_path):
        _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{B}")
        _bbx_boot(tmp_path, roster=f"boxed:{B}")
        # ...and it returns at the END of the roster, order be damned.
        m3 = _bbx_boot(tmp_path, roster=f"boxed:{B}, boxed:{A}")
        assert self._uids(m3) == {"AFC_BambuAMS Bambu_AMS_1": A,
                                  "AFC_BambuAMS Bambu_AMS_2": B}
        assert self._lane_units(m3) == self._slots(
            (24, 4, "Bambu_AMS_1"), (28, 4, "Bambu_AMS_2"))
        self._sections(m3)
        assert m3._lane_map == {A: (24, 4), B: (28, 4)}
        assert m3._name_map == {A: "Bambu_AMS_1", B: "Bambu_AMS_2"}
        assert m3._pool_units == [self._pooled_ams("Bambu_AMS_1", 24, A),
                                  self._pooled_ams("Bambu_AMS_2", 28, B)]

    def test_a_new_unit_allocates_past_every_tombstone(self, tmp_path):
        # A new unit wearing a removed unit's name would take over its
        # lanes, and with them its Spoolman bindings and T#.
        _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{B}")
        m2 = _bbx_boot(tmp_path, roster=f"boxed:{B}, boxed:{C}")
        assert self._uids(m2) == {"AFC_BambuAMS Bambu_AMS_2": B,
                                  "AFC_BambuAMS Bambu_AMS_3": C}
        assert self._lane_units(m2) == self._slots(
            (28, 4, "Bambu_AMS_2"), (32, 4, "Bambu_AMS_3"))
        assert m2._name_map[A] == "Bambu_AMS_1"

    def test_a_refined_unit_keeps_its_name_no_rename(self, tmp_path):
        # The name carries no generation, so a boxed unit confirming as
        # ams2 stays Bambu_AMS_1 and its learned values stay put.
        _bbx_boot(tmp_path, roster=f"boxed:{A}")
        m1 = _bbx_boot(tmp_path, roster=f"boxed:{A}")
        m1._state_set({"AFC_hub Bambu_AMS_1":
                       {"afc_bowden_length": "3632.0"}})
        m2 = _bbx_boot(tmp_path, roster=f"ams2:{A}")
        sections = self._sections(m2)
        assert list(sections) == [
            "AFC_BambuAMS Bambu_AMS_1", *_bbx_lanes_span(24, 27),
            "AFC_hub Bambu_AMS_1", "temperature_sensor Bambu_AMS_1"]
        assert sections["AFC_BambuAMS Bambu_AMS_1"] == self._unit(
            "Bambu_AMS_1", A, "ams2", heater=True, dry_max_temp=65)
        assert sections["AFC_lane lane24"] == self._lane("Bambu_AMS_1", 1)
        assert m2._state_get("AFC_hub Bambu_AMS_1",
                             "afc_bowden_length") == "3632.0"
        _bbx_quiet(m1.printer)

    def test_without_forget_the_tombstone_still_holds(self, tmp_path):
        # Mere absence, however long, never frees anything: FORGET is the
        # reclaim path.
        _bbx_boot(tmp_path, roster=f"boxed:{A}, boxed:{B}")
        _bbx_boot(tmp_path, roster=f"boxed:{B}")
        m2 = _bbx_boot(tmp_path, roster=f"boxed:{B}, boxed:{C}")
        assert self._lane_units(m2) == self._slots(
            (28, 4, "Bambu_AMS_2"), (32, 4, "Bambu_AMS_3"))
        assert self._uids(m2)["AFC_BambuAMS Bambu_AMS_3"] == C
        self._sections(m2)
        assert m2._lane_map == {A: (24, 4), B: (28, 4), C: (32, 4)}
        assert m2._name_map == {A: "Bambu_AMS_1", B: "Bambu_AMS_2",
                                C: "Bambu_AMS_3"}
        assert m2._pool_units == [self._pooled_ams("Bambu_AMS_2", 28, B),
                                  self._pooled_ams("Bambu_AMS_3", 32, C)]

    def test_without_a_pool_a_tombstone_never_puts_an_ams_on_the_ht(
            self, tmp_path):
        # No pool: a new AMS passes over a tombstone unless that lands it on
        # the HT band (AMS count 1 -> the HT sits at lane28).
        _bbx_boot(tmp_path, roster=f"boxed:{A}, ht:{H}")
        m = _bbx_boot(tmp_path, roster=f"ht:{H}, boxed:{B}")
        assert self._uids(m) == {"AFC_BambuAMS Bambu_AMS_1": B,
                                 "AFC_BambuAMS Bambu_AMS_HT_1": H}
        assert self._lane_units(m) == self._slots(
            (24, 4, "Bambu_AMS_1"), (28, 1, "Bambu_AMS_HT_1"))
        assert m._state_get(SEC, "name_map") == (
            f"{B}:Bambu_AMS_1, {H}:Bambu_AMS_HT_1")

    def test_an_owner_entry_changes_no_fabricated_section(self, tmp_path):
        opts = dict(roster="", pool_ams=3, pool_ht=1, ht_names="Hot",
                    ams_names="Alpha, Bravo, Charlie")
        for sub in ("a", "b"):
            (tmp_path / sub).mkdir()
            _bbx_record(tmp_path / sub, f"boxed:{A}")
        before = _bbx_boot(tmp_path / "a", **opts)
        after = _bbx_boot(tmp_path / "b", **opts)
        after._state_set({SEC: {"bay_owner": f"{D}:Charlie, {A}:Alpha"}})
        rebuilt = _bbx_boot(tmp_path / "b", **opts)
        assert rebuilt._owners() == {"Charlie": D, "Alpha": A}
        assert before._owners() == {}
        assert rebuilt._roster_sections(rebuilt.units) == \
            before._roster_sections(before.units)
        assert self._uids(rebuilt) == {
            "AFC_BambuAMS Alpha": A, "AFC_BambuAMS Bravo": None,
            "AFC_BambuAMS Charlie": None, "AFC_BambuAMS Hot": None}
        _bbx_quiet(before.printer)
        _bbx_quiet(after.printer)

    # the keys a fabricated unit section carries are options the unit reads

    class _NameReadingConfig(BambuConfig):
        """BambuConfig recording the name of every option read."""

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            """Start with nothing read."""
            super().__init__(*args, **kwargs)
            self.names: set = set()

        def get(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            self.names.add(option)
            return super().get(option, *args, **kwargs)

        def getint(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            self.names.add(option)
            return super().getint(option, *args, **kwargs)

        def getfloat(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            self.names.add(option)
            return super().getfloat(option, *args, **kwargs)

        def getboolean(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            self.names.add(option)
            return super().getboolean(option, *args, **kwargs)

        def getlist(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            self.names.add(option)
            return super().getlist(option, *args, **kwargs)

        def getchoice(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option's value"""
            self.names.add(option)
            return super().getchoice(option, *args, **kwargs)

    def _emitted(self, tmp_path: pathlib.Path
                 ) -> Dict[str, Tuple[Dict[str, str], set]]:
        """
        Fabricate one unit of each kind (ht and ams2 dry, boxed does not)
        on a chain with a tcp_key, and build each through
        afcBambuAMS.__init__ from the keys it was loaded with, recording
        the options it reads.

        :param tmp_path: where each master's files live
        :return dict: model -> (the fabricated keys, the options read)
        """
        out: Dict[str, Tuple[Dict[str, str], set]] = {}
        for model, uid in (("ht", A), ("ams2", B), ("boxed", C)):
            (tmp_path / model).mkdir()
            m = _bbx_boot(tmp_path / model, roster=f"{model}:{uid}",
                          tcp_key="k")
            (section,) = [s for s in _bbx_loaded(m)
                          if s.startswith("AFC_BambuAMS ")]
            keys = _bbx_keys(m, section)
            _bbx_quiet(m.printer)
            printer = make_printer()
            config = self._NameReadingConfig(section, printer, keys)
            afcBambuAMS(config)
            out[model] = (keys, config.names)
        return out

    def test_every_fabricated_key_is_an_option_the_unit_reads(
            self, tmp_path):
        # A key the unit never reads is dead config, or an option renamed on
        # the reading side that every fabricated unit silently loses.
        for model, (keys, read) in self._emitted(tmp_path).items():
            assert set(keys) - read == set(), model

    def test_the_emitted_set_is_not_accidentally_empty(self, tmp_path):
        # Guards the guard: the test above passes vacuously on no keys.
        common = {"serial_port", "tcp_key", "ams_model", "extruder", "hub",
                  "auto_error_recovery", "measure_on_insert", "buffer",
                  "buffer_chip_name", "pool", "unit_uid"}
        assert {model: set(keys) for model, (keys, _read)
                in self._emitted(tmp_path).items()} == {
            "ht": common | {"heater", "dry_max_temp"},
            "ams2": common | {"heater", "dry_max_temp"},
            "boxed": common}


class TestAfcBridgeBoxLoadedLaneNumbers:
    """The laneN numbers of the lane objects the printer already holds."""

    def test_numbered_lanes_and_steppers_only(self, tmp_path):
        printer = make_printer()
        m = _bbx_chain(tmp_path, printer, "chain1", register=False,
                       pool_ams=0, pool_ht=0, lane_base=24)
        assert m._loaded_lane_numbers() == [24]
        for name in ("AFC_lane lane3", "AFC_lane e0", "AFC_stepper lane7",
                     "AFC_stepper Lane12", "AFC_hub lane9"):
            printer.add_object(name, object())
        # A leftover section swept at this start is no lane.
        printer.add_object("AFC_lane lane30", _SweptSection())
        assert sorted(m._loaded_lane_numbers()) == [3, 7, 12, 24]
        _bbx_quiet(printer)

    def test_an_unlistable_registry_yields_nothing(self, tmp_path):
        printer = make_printer()
        m = _bbx_chain(tmp_path, printer, "chain1", register=False,
                       pool_ams=0, pool_ht=0, lane_base=24)
        _bbx_unlistable(printer)
        assert m._loaded_lane_numbers() == []
        _bbx_quiet(printer)


class TestAfcBridgeBoxEarlierChains:
    """The chain masters klippy loaded before this one."""

    def test_only_masters_count_not_override_holders(self, tmp_path):
        printer = _bbx_steppers(make_printer())
        # A section without serial_port loads as a holder, even from a
        # config that cannot list its options.
        holder = load_config_prefix(_BbxUnlistedConfig(
            "AFC_BridgeBox ht", printer, {"measure_on_insert": "False"}))
        assert (type(holder), holder.name) == (BridgeBoxOverrideHolder, "ht")
        printer.add_object("AFC_BridgeBox ht", holder)
        m1 = _bbx_chain(tmp_path, printer, "chain1")
        m2 = _bbx_chain(tmp_path, printer, "chain2", register=False,
                        unit_prefix="Bambu_AMS_B")
        assert m1._earlier_chains() == [m1]
        assert m2._earlier_chains() == [m1]
        _bbx_quiet(printer)

    def test_an_unlistable_registry_yields_nothing(self, tmp_path):
        printer = make_printer()
        m = _bbx_chain(tmp_path, printer, "chain1", register=False,
                       pool_ams=0, pool_ht=0, lane_base=24)
        _bbx_unlistable(printer)
        assert m._earlier_chains() == []
        _bbx_quiet(printer)


class TestAfcBridgeBoxLaterChains:
    """The chain masters the config declares below this one."""

    def test_later_chains_are_the_masters_not_loaded_yet(self, tmp_path):
        printer = _bbx_steppers(make_printer())
        for name in ("chain1", "chain2"):
            printer.add_section(
                f"AFC_BridgeBox {name}",
                {"serial_port": f"/dev/serial/by-id/usb-{name}-if00"})
            if name == "chain1":
                # An override holder carries no serial_port.
                printer.add_section("AFC_BridgeBox ht",
                                    {"measure_on_insert": "False"})
        config = BambuConfig(SEC, printer)
        m1 = _bbx_chain(tmp_path, printer, "chain1")
        assert m1._later_chains(config) == ["chain2"]
        m2 = _bbx_chain(tmp_path, printer, "chain2",
                        unit_prefix="Bambu_AMS_B")
        assert m2._later_chains(config) == []
        assert m1._later_chains(config) == []
        _bbx_quiet(printer)

    def test_a_config_without_its_merged_file_names_none(self, tmp_path):
        printer = _bbx_steppers(make_printer())
        printer.add_section(
            "AFC_BridgeBox chain2",
            {"serial_port": "/dev/serial/by-id/usb-chain2-if00"})
        config = BambuConfig(SEC, printer)
        m1 = _bbx_chain(tmp_path, printer, "chain1")
        assert m1._later_chains(config) == ["chain2"]
        config.fileconfig = None
        assert m1._later_chains(config) == []
        _bbx_quiet(printer)


class TestAfcBridgeBoxSetBayOwner:
    """Which unit a pool bay was last claimed onto, recorded per chain."""

    def test_two_chains_on_one_state_file_keep_their_own_owners(
            self, tmp_path):
        printer = make_printer()
        m1, m2 = _bbx_two_chains(tmp_path, printer)
        b1, b2 = m1._pool_units[0]["name"], m2._pool_units[0]["name"]
        assert (b1, b2) == ("Bambu_AMS_1", "Bambu_AMS_B_1")
        m1._set_bay_owner(b1, A)
        m2._set_bay_owner(b2, B)
        assert m1._owners() == {b1: A}
        assert m2._owners() == {b2: B}
        assert m1._state_get("AFC_BridgeBox chain1", "bay_owner") == \
            f"{A}:{b1}"
        assert m2._state_get("AFC_BridgeBox chain2", "bay_owner") == \
            f"{B}:{b2}"
        _bbx_quiet(printer)
        again = _bbx_chain(tmp_path, _bbx_steppers(make_printer()), "chain2",
                           unit_prefix="Bambu_AMS_B")
        assert again._load_bay_owner() == ({b2: B}, True)

    def test_a_unit_leaves_its_other_bay_and_the_records_held_there(
            self, tmp_path):
        # One bay per unit: claiming AAAA onto Bambu_AMS_1 drops its owner
        # entry on Bambu_AMS_HT_1 and the records held for it there, and
        # leaves BBBB's held bay alone.
        m = _bbx_chain(tmp_path, _bbx_steppers(make_printer()), "chain1")
        m._set_bay_owner("Bambu_AMS_HT_1", A)
        assert m._owners() == {"Bambu_AMS_HT_1": A}
        m._held = {"Bambu_AMS_1": {"uid": A, "lanes": {}},
                   "Bambu_AMS_HT_1": {"uid": A, "lanes": {}},
                   "Bambu_AMS_HT_2": {"uid": B, "lanes": {}}}
        m._set_bay_owner("Bambu_AMS_1", A)
        assert m._owners() == {"Bambu_AMS_1": A}
        assert m._held == {"Bambu_AMS_1": {"uid": A, "lanes": {}},
                           "Bambu_AMS_HT_2": {"uid": B, "lanes": {}}}
        assert m._state_get(SEC, "bay_owner") == f"{A}:Bambu_AMS_1"
        _bbx_quiet(m.printer)


class TestAfcBridgeBoxPersistBayOwner:
    """bay_owner reaches the state block only once PREP has run."""

    @staticmethod
    def _rec(tool: str, **fields: Any) -> Dict[str, Any]:
        """
        :param tool: the lane's T#
        :param fields: further saved fields
        :return dict: a lane record as AFC's save_vars writes one
        """
        return dict({"map": tool, "current_map": tool}, **fields)

    def test_the_guess_is_the_owner_and_is_written_once_prep_has_run(
            self, tmp_path):
        # The first start after an upgrade: AAAA is pinned to Alpha, the
        # var file holds Alpha's records, and the state has no bay_owner.
        opts = dict(roster="", pool_ams=3, pool_ht=1, ht_names="Hot",
                    ams_names="Alpha, Bravo, Charlie")
        _bbx_record(tmp_path, f"boxed:{A}")
        _bbx_boot(tmp_path, **opts)
        records = {"lane24": self._rec("T24", spool_id=159, material="PLA",
                                       color="#0086D6", weight=412.0),
                   "lane25": self._rec("T25", material="PETG")}
        printer = make_printer(var_file=str(tmp_path / "AFC.var"))
        write_unit_vars(printer, {"Alpha": records})
        m = _bbx_boot(tmp_path, printer=printer, **opts)
        held = [("debug", "AFC_BridgeBox chain1: holding the saved lane "
                          "records of Alpha for their units")]
        assert _bbx_ready(m) == held
        assert m._held == {"Alpha": {"uid": A, "lanes": records}}
        assert m._owners() == {"Alpha": A}
        assert m._state_get(SEC, "bay_owner") is None
        assert m._bay_owner_pending is True
        printer.afc.prep_done = False
        m._persist_bay_owner()
        assert m._state_get(SEC, "bay_owner") is None
        assert m._bay_owner_pending is True
        printer.afc.prep_done = True
        m._persist_bay_owner()                   # the tick after PREP
        assert m._state_get(SEC, "bay_owner") == f"{A}:Alpha"
        assert m._bay_owner_pending is False
        # Unchanged owners return before any write: a rewrite of the state
        # file would drop this trailing line.
        state = tmp_path / "AFC_BridgeBox.cfg"
        with open(state, "a", encoding="utf-8") as fh:
            fh.write("# operator note\n")
        m._persist_bay_owner()
        assert m._state_get(SEC, "bay_owner") == f"{A}:Alpha"
        assert state.read_text(encoding="utf-8").endswith("# operator note\n")
        # The persist calls added nothing to the ready log.
        assert printer.afc.logger.messages == held
        assert printer.gcode.messages == []

    def test_with_no_afc_the_owner_is_written_at_once(self, tmp_path):
        printer = make_printer()
        printer._afc = None
        m = _bbx_boot(tmp_path, printer=printer, **POOL)
        m._bay_owner = {"Bambu_AMS_1": A}
        m._bay_owner_pending = True
        m._persist_bay_owner()
        assert m._state_get(SEC, "bay_owner") == f"{A}:Bambu_AMS_1"
        assert m._bay_owner_pending is False
        _bbx_quiet(printer)


class TestAfcBridgeBoxGivenNames:
    """The name each AMS rank and ht_names index is given."""

    def test_lists_without_a_clash_say_nothing(self, tmp_path):
        _bbx_record(tmp_path, FOUR)
        m = _bbx_restart(tmp_path, ams_names="W, X, Y, Z", ht_names="Hot")
        assert _bbx_ready(m) == []
        assert m._given_names() == (["W", "X", "Y", "Z"], ["Hot"], {}, {})
        assert _bbx_uids(m) == {"W": A, "X": B, "Y": C, "Z": D, "Hot": H,
                                "Bambu_AMS_HT_2": ""}

    def test_a_repeated_entry_names_its_bay_by_default(self, tmp_path):
        m = _bbx_restart(tmp_path, ams_names="X, X", ht_names="Hot")
        first = m._given_names()
        assert first == (
            ["X", "Bambu_AMS_2", "Bambu_AMS_3", "Bambu_AMS_4"], ["Hot"],
            {("ams", 1): "ams_names entry 2 (X) is also ams_names entry 1, "
                         "so AMS bay 2 is named Bambu_AMS_2 -- give each bay "
                         "a name of its own."},
            {("ams", "X"): (1, "ams_names entry 1")})
        # Asked again for the same lists, the answer is the same.
        assert m._given_names() == first
        _bbx_quiet(m.printer)


class TestAfcBridgeBoxRankOf:
    """The rank a bay name gives a unit of a family."""

    def test_rank_of_reads_ht_names_past_sixteen(self, tmp_path):
        m = _bbx_restart(tmp_path)
        assert [m._rank_of("ht", n) for n in (
            "Bambu_AMS_HT_1", "Bambu_AMS_HT_17", "Bambu_AMS_HT_40",
            "Bambu_AMS_HT_01", "Bambu_AMS_2")] == [0, 16, 39, None, None]
        _bbx_quiet(m.printer)

    def test_rank_of_gives_an_ams_the_four_bay_names_only(self, tmp_path):
        m = _bbx_restart(tmp_path, ams_names="Alpha")
        # Index 0 is Alpha, so Bambu_AMS_1 names no AMS rank.
        assert [m._rank_of("ams", n) for n in (
            "Alpha", "Bambu_AMS_1", "Bambu_AMS_4", "Bambu_AMS_5",
            "Bambu_AMS_HT_1")] == [0, None, 3, None, None]
        _bbx_quiet(m.printer)


class TestAfcBridgeBoxOverlapError:
    """The backstop refusal for two bays on one lane."""

    @staticmethod
    def _bay(name: str, uid: Optional[str], lane: int, slots: int,
             spare: bool = False) -> Dict[str, Any]:
        """:return dict: a bay as the layout pass holds it"""
        return {"name": name, "uid": uid,
                "family": "ams" if slots == 4 else "ht", "rank": 0,
                "lane": lane, "slots": slots, "spare": spare}

    def test_the_overlap_backstop_names_both_bays_and_the_record(
            self, tmp_path):
        m = _bbx_restart(tmp_path)
        state = tmp_path / "AFC_BridgeBox.cfg"
        assert m._overlap_error(self._bay("X", A, 24, 4),
                                self._bay("Y", B, 24, 4)) == (
            f"bays X (lane24-lane27) and Y (lane24-lane27) overlap: their "
            f"lanes follow from the unit names the #~# block of {state} "
            f"records for AAAA and BBBB. AFC_BRIDGEBOX_FORGET and "
            f"AFC_BRIDGEBOX_UNASSIGN need Klipper running, so remove the "
            f"name_map and lane_map entries of AAAA or BBBB there by hand "
            f"and RESTART; that unit then draws a free bay.")
        _bbx_quiet(m.printer)

    def test_a_spare_bay_names_only_the_recorded_unit(self, tmp_path):
        m = _bbx_restart(tmp_path)
        state = tmp_path / "AFC_BridgeBox.cfg"
        assert m._overlap_error(self._bay("X", A, 24, 4),
                                self._bay("Z", None, 26, 1, spare=True)) == (
            f"bays X (lane24-lane27) and Z (lane26) overlap: their lanes "
            f"follow from the unit names the #~# block of {state} records "
            f"for AAAA. AFC_BRIDGEBOX_FORGET and AFC_BRIDGEBOX_UNASSIGN need "
            f"Klipper running, so remove the name_map and lane_map entries "
            f"of AAAA there by hand and RESTART; that unit then draws a free "
            f"bay.")
        _bbx_quiet(m.printer)

    def test_two_spare_bays_name_no_record(self, tmp_path):
        m = _bbx_restart(tmp_path)
        assert m._overlap_error(
            self._bay("X", None, 24, 4, spare=True),
            self._bay("Z", None, 26, 1, spare=True)) == (
            "bays X (lane24-lane27) and Z (lane26) overlap.")
        _bbx_quiet(m.printer)


class TestAfcBridgeBoxCmdAFCBridgeboxForget:
    """FORGET erases a unit's tombstones, records and bay, live."""

    @staticmethod
    def _seeded(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                **options: Any) -> afcBridgeBox:
        """
        AAAA and BBBB known, then a start with only BBBB in roster: AAAA is
        a tombstone holding lanes 24-27 and the name Bambu_AMS_1.

        :return afcBridgeBox: the second start's master
        """
        _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}, boxed:{B}")
        return _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{B}",
                         **options)

    @staticmethod
    def _lane_units(master: afcBridgeBox) -> Dict[str, str]:
        """:return dict: fabricated lane -> the unit:slot it is on"""
        return {s.split(" ", 1)[1]: w.fileconfig.get(s, "unit")
                for s, w in master.printer.loaded
                if s.startswith("AFC_lane ")}

    @staticmethod
    def _klippy(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                **options: Any) -> afcBridgeBox:
        """
        chain1 as klippy's load_config_prefix builds it from a
        :class:`_P3BareConfig`, its buffer given, on a printer with no gcode
        object that records the sections the master fabricates.

        :param options: the master's options
        :return afcBridgeBox: the master; its logger is AFC's
        """
        printer = make_printer(monkeypatch=monkeypatch)
        set_var_file(printer, str(tmp_path / "AFC.var"))
        monkeypatch.setattr(printer, "_gcode", None)
        opts = bridgebox_options(tmp_path, buffer="Bamb_1", **options)
        master = load_config_prefix(_P3BareConfig(SEC, printer, opts))
        master.logger = printer.afc.logger
        return master

    def test_forget_frees_the_lane_and_name_for_the_next_unit(
            self, tmp_path, monkeypatch):
        m1 = _p3_chain(tmp_path, monkeypatch,
                       roster=f"boxed:{A}, boxed:{B}, ht:{C}", lane_base=12)
        _p3_quiet(m1)
        cmd = _p3_cmd(UID=A)
        m1.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 12-15 and the name "
            "Bambu_AMS_1 freed for reuse -- its bay is free now, but with no "
            "pool nothing claims it: AFC_BRIDGEBOX_ASSIGN another unit onto "
            "it while that unit is online, or RESTART. NOTE: your roster: "
            "option still lists this uid -- remove it there too, the option "
            "overrides the recorded roster.")]
        assert _p3_log(m1) == []
        assert _p3_console(m1) == [END]
        m2 = _p3_chain(tmp_path, monkeypatch,
                       roster=f"boxed:{B}, ht:{C}, boxed:{D}", lane_base=12)
        assert self._lane_units(m2)["lane12"] == "Bambu_AMS_1:1"
        assert _p3_unit_uids(m2) == {"Bambu_AMS_1": D, "Bambu_AMS_2": B,
                                     "Bambu_AMS_HT_1": C}

    def test_forget_frees_the_lanes_and_name_for_the_next_unit(
            self, tmp_path, monkeypatch):
        m = self._seeded(tmp_path, monkeypatch)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse. Applies at the next RESTART.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        m2 = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{B}, boxed:{C}")
        loaded = dict(m2.printer.loaded)
        assert dict(loaded["AFC_lane lane24"].fileconfig.items(
            "AFC_lane lane24")) == {"unit": "Bambu_AMS_1:1",
                                    "unassigned": "True"}
        assert _p3_unit_uids(m2)["Bambu_AMS_1"] == C

    def test_forget_frees_the_held_slot_live(self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}, boxed:{B}")
        bay = _p3_bay(m, "Bambu_AMS_1")
        assert (bay["uid"], bay["bound"]) == (A, None)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse -- its bay is free now, but with no "
            "pool nothing claims it: AFC_BRIDGEBOX_ASSIGN another unit onto "
            "it while that unit is online, or RESTART. NOTE: your roster: "
            "option still lists this uid -- remove it there too, the option "
            "overrides the recorded roster.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_erases_the_learned_values(self, tmp_path, monkeypatch):
        m = self._seeded(tmp_path, monkeypatch)
        m._state_set({SEC: {"roster": f"boxed:{A}, boxed:{B}"},
                      m._learned_section(A): {"afc_bowden_length": "3632.0"},
                      "AFC_BambuAMS Bambu_AMS_1":
                      {"afc_bowden_length": "3632.0"},
                      "AFC_hub Bambu_AMS_1": {"afc_bowden_length": "3632.0"}})
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._learned_for(A) == {}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_1",
                            "afc_bowden_length") is None
        assert m._state_get("AFC_hub Bambu_AMS_1",
                            "afc_bowden_length") is None
        assert m._state_get(SEC, "roster") == "boxed:BBBB"
        assert m._state_get(SEC, "lane_map") == "BBBB:28:4"
        assert m._state_get(SEC, "name_map") == "BBBB:Bambu_AMS_2"
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse, learned values erased. Applies at "
            "the next RESTART."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_by_name(self, tmp_path, monkeypatch):
        m = self._seeded(tmp_path, monkeypatch)
        _p3_quiet(m)
        cmd = _p3_cmd(NAME="Bambu_AMS_1")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse. Applies at the next RESTART."))]
        assert m._name_map == {B: "Bambu_AMS_2"}
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_by_an_unknown_name_lists_the_known_ones(
            self, tmp_path, monkeypatch):
        m = self._seeded(tmp_path, monkeypatch)
        refusal = _p3_refused(m, tmp_path, "FORGET", _p3_cmd(NAME="Nope"))
        assert refusal == (
            "AFC_BRIDGEBOX_FORGET: no recorded unit named 'Nope' (known: "
            "Bambu_AMS_1, Bambu_AMS_2)")

    def test_forget_refuses_an_unknown_uid(self, tmp_path, monkeypatch):
        m = self._seeded(tmp_path, monkeypatch)
        refusal = _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID="FEED"))
        assert refusal == "AFC_BRIDGEBOX_FORGET: nothing recorded for uid FEED"

    def test_forget_drops_a_live_unit_and_suppresses_reenroll(
            self, tmp_path, monkeypatch):
        m = self._seeded(tmp_path, monkeypatch, online=(A, B))
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._state_get(SEC, "lane_map") == "BBBB:28:4"
        assert m._forget_suppressed == {A}
        assert _p3_bridge(m).sent == [{"cmd": "forget", "uid": "aaaa"}]
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse. Applies at the next RESTART. It is "
            "still on the wire, so re-enroll is suppressed until you "
            "physically pull it -- a re-plug then enrolls it fresh."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_drops_a_bound_live_unit(self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}, boxed:{B}",
                      online=(A, B))
        bay = _p3_bay(m, "Bambu_AMS_1")
        assert m._claim_pool_unit(A, "boxed") is not None
        assert bay["bound"] == A
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert m._forget_suppressed == {A}
        assert m.printer.lookup_object("AFC_lane lane24").unassigned is True
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse, saved lane records erased -- its "
            "bay is free now, but with no pool nothing claims it: "
            "AFC_BRIDGEBOX_ASSIGN another unit onto it while that unit is "
            "online, or RESTART. It is still on the wire, so re-enroll is "
            "suppressed until you physically pull it -- a re-plug then "
            "enrolls it fresh. NOTE: your roster: option still lists this "
            "uid -- remove it there too, the option overrides the recorded "
            "roster."))]
        assert _p3_log(m) == [(
            "info",
            "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA, "
            "AFC_BRIDGEBOX_FORGET); lanes dropped live")]
        assert _p3_console(m) == [END]

    def test_forget_refuses_a_unit_on_a_bay_mid_print(self, tmp_path,
                                                      monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}, boxed:{B}",
                      online=(A, B))
        bay = _p3_bay(m, "Bambu_AMS_1")
        assert m._claim_pool_unit(A, "boxed") is not None
        m.printer.set_print_state("printing")
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=A)) == (
            "AFC_BRIDGEBOX_FORGET: AAAA is on Bambu_AMS_1 and a print is "
            "active -- dropping its lanes now would disrupt the print. "
            "Finish the print, or FORCE=1 to forget it anyway")
        assert bay["bound"] == A
        cmd = _p3_cmd(UID=A, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._state_get(SEC, "lane_map") == "BBBB:28:4"
        assert bay["bound"] is None
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse, saved lane records erased -- its "
            "bay is free now, but with no pool nothing claims it: "
            "AFC_BRIDGEBOX_ASSIGN another unit onto it while that unit is "
            "online, or RESTART. It is still on the wire, so re-enroll is "
            "suppressed until you physically pull it -- a re-plug then "
            "enrolls it fresh. NOTE: your roster: option still lists this "
            "uid -- remove it there too, the option overrides the recorded "
            "roster."))]
        assert _p3_log(m) == [(
            "info",
            "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA, "
            "AFC_BRIDGEBOX_FORGET); lanes dropped live")]
        assert _p3_console(m) == [END]

    def test_forget_of_an_online_unit_on_no_bay_mid_print_goes_ahead(
            self, tmp_path, monkeypatch):
        m = self._seeded(tmp_path, monkeypatch, online=(A, B),
                         print_state="printing")
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._state_get(SEC, "lane_map") == "BBBB:28:4"
        assert m._forget_suppressed == {A}
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse. Applies at the next RESTART. It is "
            "still on the wire, so re-enroll is suppressed until you "
            "physically pull it -- a re-plug then enrolls it fresh."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_with_no_args_pops_a_picker_of_known_units(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, **NAMES)
        before = _p3_state(tmp_path)
        _p3_quiet(m)
        cmd = _p3_cmd()
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert _p3_console(m) == _p3_popup(
            "Forget a Bambu AMS unit", ["Alpha  [AAAA\u2026]"],
            ["Forget Alpha|AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=AAAA|error"])
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: forget picker -- Alpha")]
        assert _p3_state(tmp_path) == before
        assert _p3_log(m) == []

    @pytest.mark.parametrize("live", [False, True])
    def test_forget_drops_the_owner_and_the_held_records(
            self, tmp_path, monkeypatch, live):
        m = _p3_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert m._claim_pool_unit(C, "boxed") is not None
        _p3_lane(m, "lane24").spool_id = 159
        if live:
            _p3_wire(m, (A, C))
        else:
            m._release_pool_unit(A)
            assert m._held["Alpha"]["uid"] == A
        drain_var_writes(m.printer)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._state_get(SEC, "bay_owner") == "CCCC:Bravo"
        assert m._owners() == {"Bravo": C}
        assert m._held == {}
        assert drain_var_writes(m.printer)[-1]["Alpha"] == {}
        wire = (" It is still on the wire, so re-enroll is suppressed until "
                "you physically pull it -- a re-plug then enrolls it fresh."
                if live else "")
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Alpha freed for reuse, saved lane records erased -- slot freed "
            "to the pool LIVE; the next same-family unit claims it with no "
            "reboot." + wire)]
        released = [(
            "info",
            "AFC_BridgeBox chain1: released Alpha (UID AAAA, "
            "AFC_BRIDGEBOX_FORGET); lanes dropped live")]
        assert _p3_log(m) == (released if live else [])
        assert _p3_console(m) == [END]

    def test_forget_with_an_owner_entry_and_nothing_held_erases_no_records(
            self, tmp_path, monkeypatch):
        _p3_named(tmp_path, monkeypatch, ready=False)
        m = _p3_named(tmp_path, monkeypatch, var={"Alpha": {}},
                      owners="AAAA:Alpha")
        assert (m._held, m._owners()) == ({}, {"Alpha": A})
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Alpha freed for reuse -- slot freed to the pool LIVE; the next "
            "same-family unit claims it with no reboot."))]
        assert m._owners() == {}
        assert m._state_get(SEC, "bay_owner") == ""
        assert drain_var_writes(m.printer) == []
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forgetting_a_floating_unit_says_its_records_went(
            self, tmp_path, monkeypatch):
        m = _p3_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        _p3_lane(m, "lane24").spool_id = 159
        m._release_pool_unit(A)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_p3_cmd(UID=A))
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert (m._held, m._owners()) == ({}, {})
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- saved lane records erased. "
            "Applies at the next RESTART."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    @pytest.mark.parametrize("ticked", [True, False])
    def test_forget_erases_the_records_it_says_it_erased(
            self, tmp_path, monkeypatch, ticked):
        var = {"Alpha": {"lane24": _p3_rec("T24", spool_id=159,
                                           material="PLA", color="#0086D6",
                                           weight=412.0),
                         "lane25": _p3_rec("T25", material="PETG")}}
        _p3_named(tmp_path, monkeypatch, ready=False, roster=f"boxed:{A}")
        m = _p3_named(tmp_path, monkeypatch, var=var, roster=f"boxed:{A}")
        m.printer.afc.save_vars()
        assert drain_var_writes(m.printer)[-1]["Alpha"] == var["Alpha"]
        if ticked:
            m._persist_bay_owner()
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Alpha freed for reuse, saved lane records erased -- slot freed "
            "to the pool LIVE; the next same-family unit claims it with no "
            "reboot. NOTE: your roster: option still lists this uid -- "
            "remove it there too, the option overrides the recorded roster."))]
        assert (m._held, m._owners()) == ({}, {})
        assert drain_var_writes(m.printer)[-1]["Alpha"] == {}
        assert m._state_get(SEC, "bay_owner") == ""
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        again = _p3_named(tmp_path, monkeypatch, var=var,
                          recorded=f"boxed:{C}", roster=f"boxed:{C}")
        assert _p3_bay(again, "Alpha")["uid"] == C
        assert again._held == {}
        assert again._claim_pool_unit(C, "boxed") is not None
        assert _p3_lane(again, "lane24").spool_id is None

    def test_it_accepts_an_unrecorded_unit_that_holds_a_bay(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                      recorded=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _p3_tick(m, 100, 101)
        bay = _p3_bay(m, "Bambu_AMS_4")
        assert bay["bound"] == G
        _p3_online(m, (A, B, C, H))
        _p3_tick(m, 102, 103)
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID="FFFF")) == (
            "AFC_BRIDGEBOX_FORGET: nothing recorded for uid FFFF")
        cmd = _p3_cmd(UID=G)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot GGGG -- saved lane records erased "
            "-- slot freed to the pool LIVE; the next same-family unit claims "
            "it with no reboot."))]
        assert _p3_log(m) == [(
            "info",
            "AFC_BridgeBox chain1: released Bambu_AMS_4 (UID GGGG, "
            "AFC_BRIDGEBOX_FORGET); lanes dropped live")]
        assert _p3_console(m) == [END]

    def test_forget_accepts_a_uid_only_its_bay_holds(self, tmp_path,
                                                     monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                      recorded=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _p3_tick(m, 100, 101)
        bay = _p3_bay(m, "Bambu_AMS_4")
        _p3_online(m, (A, B, C, H))
        _p3_tick(m, 102, 103)
        # The bay still holds G, but its owner entry no longer names it.
        owners = {b: u for b, u in m._owners().items() if u != G}
        m._bay_owner = owners
        m._state_set({SEC: {"bay_owner": m._ser_bay_owner(owners)}})
        assert m._bay_of_uid(G) is bay and G not in m._owners().values()
        assert (G in m._lane_map, G in m._name_map, m._held,
                m._read_state().has_section(m._learned_section(G)),
                m._state_get(SEC, "roster")) == (
            False, False, {}, False,
            f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _p3_quiet(m)
        cmd = _p3_cmd(UID=G)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot GGGG -- saved lane records erased "
            "-- slot freed to the pool LIVE; the next same-family unit claims "
            "it with no reboot."))]
        assert _p3_log(m) == [(
            "info",
            "AFC_BridgeBox chain1: released Bambu_AMS_4 (UID GGGG, "
            "AFC_BRIDGEBOX_FORGET); lanes dropped live")]
        assert _p3_console(m) == [END]

    def test_it_drops_what_the_watch_tracks_for_the_uid(self, tmp_path,
                                                        monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch)
        _p3_tick(m, 100, 111)
        assert m._missing_since == {D: 100.0}
        m._last_online[D], m._online_run[D] = 1.0, 1.0
        m._no_bay[D] = "boxed"
        m._no_bay_told.add(D)
        m._replace_offered.add(D)
        m._popup_queue = [("removed", D, "Bambu_AMS_4"), ("replace", D),
                          ("new", A)]
        _p3_quiet(m)
        cmd = _p3_cmd(UID=D)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._missing_since == {}
        assert D not in m._last_online and D not in m._online_run
        assert m._no_bay == {E: "boxed"}
        assert D not in m._no_bay_told and D not in m._replace_offered
        assert m._popup_queue == [("new", A)]
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot DDDD -- lanes 36-39 and the name "
            "Bambu_AMS_4 freed for reuse -- slot freed to the pool LIVE; the "
            "next same-family unit claims it with no reboot."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_refuses_and_force_clears_the_record(self, tmp_path,
                                                        monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch)
        _p3_tick(m, 100, 111)
        ext = m.printer.afc.tools["extruder"]
        ext.lane_loaded = "lane36"
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=D)) == (
            "AFC_BRIDGEBOX_FORGET: AFC records lane36 on Bambu_AMS_4 as "
            "loaded to the toolhead, from DDDD. While Bambu_AMS_4 is "
            "unclaimed its lanes are not registered, so no unload or "
            "UNSET_LANE_LOADED reaches them -- plug DDDD back in and unload "
            "it, or take the filament out by hand and FORCE=1 clears the "
            "record and forgets DDDD")
        saves = m.printer.afc.save_vars.call_count
        drain_var_writes(m.printer)
        cmd = _p3_cmd(UID=D, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert ext.lane_loaded is None
        assert m.printer.afc.save_vars.call_count == saves + 1
        (snap,) = drain_var_writes(m.printer)
        assert snap["system"]["extruders"] == {
            "extruder": {"lane_loaded": None}}
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot DDDD -- lanes 36-39 and the name "
            "Bambu_AMS_4 freed for reuse -- cleared lane36 from the toolhead "
            "-- slot freed to the pool LIVE; the next same-family unit claims "
            "it with no reboot."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        _p3_tick(m, 112, 113)
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E

    def test_before_prep_forget_of_an_unclaimed_bay_waits(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch)
        _p3_tick(m, 100, 111)
        m.printer.afc.prep_done = False
        m._ready_at = m.printer.reactor.now
        wait = [("debug", "AFC_BridgeBox chain1: unit claims wait for PREP")]
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=D), wait) == (
            "AFC_BRIDGEBOX_FORGET: PREP has not run yet, so which lane AFC "
            "records as loaded to the toolhead is not known -- run this "
            "again once it has")
        _p3_quiet(m)
        cmd = _p3_cmd(UID=D, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert _p3_bay(m, "Bambu_AMS_4")["uid"] is None
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot DDDD -- lanes 36-39 and the name "
            "Bambu_AMS_4 freed for reuse -- slot freed to the pool LIVE; the "
            "next same-family unit claims it with no reboot."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_mid_print_names_the_lane_and_the_print(
            self, tmp_path, monkeypatch):
        m = _p3_bound(tmp_path, monkeypatch)
        m.printer.set_print_state("printing")
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=D)) == (
            "AFC_BRIDGEBOX_FORGET: AFC records lane37 on DDDD as loaded to "
            "the toolhead -- unload it first (UNSET_LANE_LOADED if the "
            "filament is already out), or FORCE=1 to clear it from the "
            "toolhead and release anyway (a print is active, and FORCE=1 "
            "also pulls the lane out from under it)")

    def test_the_unset_hint_names_a_toolhead_that_is_not_active(
            self, tmp_path, monkeypatch):
        m = _p3_bound(tmp_path, monkeypatch)
        printer = m.printer
        e1 = add_extruder(printer, "extruder1")
        printer.afc.tools["extruder"].lane_loaded = "lane1"
        e1.lane_loaded = "lane37"
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=D)) == (
            "AFC_BRIDGEBOX_FORGET: AFC records lane37 on DDDD as loaded to "
            "the toolhead -- unload it first (UNSET_LANE_LOADED with "
            "extruder1 as the active tool if the filament is already out), "
            "or FORCE=1 to clear it from the toolhead and release anyway")
        printer.toolhead.extruder = printer.lookup_object("extruder1")
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=D)) == (
            "AFC_BRIDGEBOX_FORGET: AFC records lane37 on DDDD as loaded to "
            "the toolhead -- unload it first (UNSET_LANE_LOADED if the "
            "filament is already out), or FORCE=1 to clear it from the "
            "toolhead and release anyway")

        # AFC cannot say which tool is active: the hint names lane37's.
        def unknown() -> str:
            """AFC's active extruder, unknown."""
            raise RuntimeError("no active extruder")

        monkeypatch.setattr(printer.afc.function, "get_current_extruder",
                            unknown)
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=D)) == (
            "AFC_BRIDGEBOX_FORGET: AFC records lane37 on DDDD as loaded to "
            "the toolhead -- unload it first (UNSET_LANE_LOADED with "
            "extruder1 as the active tool if the filament is already out), "
            "or FORCE=1 to clear it from the toolhead and release anyway")

    def test_forget_says_nothing_claims_the_freed_bay(self, tmp_path,
                                                      monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, pool_ams=0, pool_ht=0)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=D)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot DDDD -- lanes 36-39 and the name "
            "Bambu_AMS_4 freed for reuse -- its bay is free now, but with no "
            "pool nothing claims it: AFC_BRIDGEBOX_ASSIGN another unit onto "
            "it while that unit is online, or RESTART."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        assert _p3_bay(m, "Bambu_AMS_4")["uid"] is None
        assert m._name_map == {A: "Bambu_AMS_1", B: "Bambu_AMS_2",
                              C: "Bambu_AMS_3", H: "Bambu_AMS_HT_1"}

    def test_with_a_pool_forget_names_no_restart_regularising(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=D)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot DDDD -- lanes 36-39 and the name "
            "Bambu_AMS_4 freed for reuse -- slot freed to the pool LIVE; the "
            "next same-family unit claims it with no reboot."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        assert _p3_bay(m, "Bambu_AMS_4")["uid"] is None
        assert m._name_map == {A: "Bambu_AMS_1", B: "Bambu_AMS_2",
                              C: "Bambu_AMS_3", H: "Bambu_AMS_HT_1"}

    def test_forget_accepts_a_released_unit_only_its_bay_names(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                      recorded=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _p3_tick(m, 100, 101)
        bay = _p3_bay(m, "Bambu_AMS_4")
        assert bay["bound"] == G
        _p3_online(m, (A, B, C, H))
        m._release_pool_unit(G)
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert m._owners()["Bambu_AMS_4"] == G
        _p3_quiet(m)
        m._prompt_removed_unit(G, "Bambu_AMS_4")
        # Only the bay's owner entry names G once its held records go.
        m._held = {}
        assert _p3_console(m) == _p3_popup(
            "AMS removed: Bambu_AMS_4", [
            "Bambu_AMS_4 (UID GGGG) was unplugged; its bay went back to the "
            "pool.",
            "A re-plug takes it back while it is free, else the lowest free "
            "bay of its family. Or forget it if it is not coming back:"], [
            "Forget Bambu_AMS_4|AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=GGGG|"
            "error"])
        _p3_quiet(m)
        cmd = _p3_cmd(UID=G)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: forgot GGGG.")]
        assert "Bambu_AMS_4" not in m._owners()
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_accepts_a_uid_only_held_records_name(self, tmp_path,
                                                         monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                      recorded=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _p3_tick(m, 100, 101)
        _p3_online(m, (A, B, C, H))
        m._release_pool_unit(G)
        # Only the records held for the bay name G once its owner entry goes.
        owners = {b: u for b, u in m._owners().items() if u != G}
        m._bay_owner = owners
        m._state_set({SEC: {"bay_owner": m._ser_bay_owner(owners)}})
        assert {bay: e["uid"] for bay, e in m._held.items()} == {
            "Bambu_AMS_4": G}
        _p3_quiet(m)
        cmd = _p3_cmd(UID=G)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: forgot GGGG -- saved lane records erased.")]
        assert m._held == {}
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_accepts_a_uid_only_the_recorded_roster_names(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=())
        m._state_set({SEC: {"roster": f"boxed:{A}, boxed:QQQQ"}})
        assert ("QQQQ" in m._lane_map, "QQQQ" in m._name_map) == (False, False)
        _p3_quiet(m)
        cmd = _p3_cmd(UID="QQQQ")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._state_get(SEC, "roster") == "boxed:AAAA"
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: forgot QQQQ. Applies at the next RESTART.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_erases_the_uid_record_and_unassign_keeps_it(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=())
        for uid in (A, C):
            m._state_set({m._learned_section(uid):
                          {"afc_bowden_length": "3632.0"}})
        _p3_quiet(m)
        assign = _p3_cmd(UID=C, NAME="Bambu_AMS_2")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(assign)
        unassign = _p3_cmd(UID=C)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(unassign)
        assert m._learned_for(C) == {"afc_bowden_length": "3632.0"}
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._learned_for(A) == {}
        assert m._learned_for(C) == {"afc_bowden_length": "3632.0"}
        assert assign.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Bambu_AMS_2' "
            "(lane28-lane31, T28-T31) -- pinned; it claims this bay when next "
            "online."))]
        assert unassign.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bambu_AMS_2' "
            "(learned values stay with the unit). It takes a free bay of its "
            "family, its last one first, and is saved there once it has been "
            "online 15s."))]
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse, learned values erased -- slot freed "
            "to the pool LIVE; the next same-family unit claims it with no "
            "reboot."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END, END, END]

    def test_forget_says_learned_values_erased_only_when_there_were_some(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=())
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse -- slot freed to the pool LIVE; the "
            "next same-family unit claims it with no reboot."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_forget_keeps_a_name_another_uid_still_holds(
            self, tmp_path, monkeypatch):
        # XXXX and YYYY are recorded with one name, so what is stored under
        # it is left unread; once XXXX is forgotten it is YYYY's.
        _p3_seed(tmp_path, {
            "AFC_BambuAMS Bambu_AMS_3": {"afc_bowden_length": "3632.0"},
            "AFC_hub Bambu_AMS_3": {"afc_bowden_length": "1800"}})
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, state={
                          "name_map": "AAAA:Bambu_AMS_1, XXXX:Bambu_AMS_3, "
                                      "YYYY:Bambu_AMS_3",
                          "lane_map": "AAAA:24:4, XXXX:32:4, YYYY:32:4"})
        assert m._learned_notes == [(
            True,
            "learned values stored under Bambu_AMS_3 left unread -- XXXX, "
            "YYYY are all recorded with that name. Run AFC_BRIDGEBOX_FORGET "
            "CHAIN=chain1 UID=<uid> for each one that is gone, and the one "
            "left takes them at the next restart.")]
        _p3_quiet(m)
        cmd = _p3_cmd(UID="XXXX")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: forgot XXXX.")]
        assert m._state_get("AFC_BambuAMS Bambu_AMS_3",
                            "afc_bowden_length") == "3632.0"
        assert m._state_get("AFC_hub Bambu_AMS_3",
                            "afc_bowden_length") == "1800"
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        m2 = _p3_chain(tmp_path, monkeypatch, roster="", pool_ams=3,
                       pool_ht=1)
        assert m2._learned_for("YYYY") == {"afc_bowden_length": "3632.0"}
        assert m2._learned_for("XXXX") == {}
        assert m2._learned_notes == [(
            False,
            "learned values stored under Bambu_AMS_3 now belong to its unit "
            "YYYY (afc_bowden_length)")]

    def test_forget_accepts_a_learned_only_uid(self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=())
        m._state_set({m._learned_section("QQQQ"):
                      {"afc_bowden_length": "3632.0"}})
        _p3_quiet(m)
        cmd = _p3_cmd(UID="QQQQ")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._learned_for("QQQQ") == {}
        assert m._read_state().has_section(
            m._learned_section("QQQQ")) is False
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: forgot QQQQ -- learned values erased.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_refuses_before_erasing_anything(self, tmp_path, monkeypatch):
        m = _p3_claimed(tmp_path, monkeypatch)
        _p3_load(m, "lane24")
        assert _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=A)) == (
            "AFC_BRIDGEBOX_FORGET: AFC records lane24 on AAAA as loaded to "
            "the toolhead -- unload it first (UNSET_LANE_LOADED if the "
            "filament is already out), or FORCE=1 to clear it from the "
            "toolhead and release anyway")
        assert _p3_bay(m, "Bambu_AMS_1")["bound"] == A
        assert m.printer.afc.tools["extruder"].lane_loaded == "lane24"

    def test_force_clears_and_forgets(self, tmp_path, monkeypatch):
        m = _p3_claimed(tmp_path, monkeypatch)
        lane = _p3_load(m, "lane24")
        drain_var_writes(m.printer)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        bay = _p3_bay(m, "Bambu_AMS_1")
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert (lane.unassigned, lane.tool_loaded, lane.loaded_to_hub) == (
            True, False, False)
        assert "lane24" not in m.printer.afc.lanes
        assert m.printer.afc.tools["extruder"].lane_loaded is None
        assert drain_var_writes(m.printer)[-1]["system"]["extruders"] == {
            "extruder": {"lane_loaded": None}}
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse, saved lane records erased -- slot "
            "freed to the pool LIVE; the next same-family unit claims it with "
            "no reboot. NOTE: your roster: option still lists this uid -- "
            "remove it there too, the option overrides the recorded roster."))]
        assert _p3_log(m) == [("debug", "Activating extruder lane: None"),
            ("info", "Manually removing lane24 loaded from toolhead"),
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA, "
                     "AFC_BRIDGEBOX_FORGET); lanes dropped live; cleared "
                     "lane24 from the toolhead")]
        assert _p3_console(m) == [END]

    class _Idle:
        """idle_timeout, recording the times it is asked about."""

        def __init__(self, state: str) -> None:
            """:param state: its state"""
            self.state = state
            self.asked: List[float] = []

        def get_status(self, eventtime: float) -> Dict[str, Any]:
            """:return dict: its state"""
            self.asked.append(eventtime)
            return {"state": self.state}

    def test_without_print_stats_idle_timeout_says_a_print_is_active(
            self, tmp_path, monkeypatch):
        # No [virtual_sdcard], so no print_stats; and a clock that cannot be
        # read, so idle_timeout is asked about time 0.
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}, boxed:{B}",
                      online=(A, B))
        assert m._claim_pool_unit(A, "boxed") is not None
        printer = m.printer
        idle = self._Idle("Printing")
        monkeypatch.delitem(printer._objects, "print_stats")
        monkeypatch.setitem(printer._objects, "idle_timeout", idle)

        def stopped() -> float:
            """The reactor's clock, unreadable."""
            raise RuntimeError("no clock")

        monkeypatch.setattr(printer.reactor, "monotonic", stopped)
        refusal = _p3_refused(m, tmp_path, "FORGET", _p3_cmd(UID=A))
        assert refusal == (
            "AFC_BRIDGEBOX_FORGET: AAAA is on Bambu_AMS_1 and a print is "
            "active -- dropping its lanes now would disrupt the print. "
            "Finish the print, or FORCE=1 to forget it anyway")
        assert idle.asked == [0.0]

    def test_forget_needs_no_gcode_object(self, tmp_path, monkeypatch):
        # Built by klippy's loader from a bare section wrapper, on a printer
        # with no gcode object: FORGET still answers, and closes no dialog.
        m = self._klippy(tmp_path, monkeypatch)
        assert (type(m), m.buffer, m._fabricate_buffer) == (
            afcBridgeBox, "Bamb_1", False)
        assert [name for name, _config in m.printer.loaded] == [
            "AFC_BambuAMS Bambu_AMS_HT_1",
            "AFC_lane lane24",
            "AFC_hub Bambu_AMS_HT_1",
            "temperature_sensor Bambu_AMS_HT_1"]
        _p3_quiet(m)
        cmd = _p3_cmd(UID="0123456789ABCDEF00003331")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._name_map == {}
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: forgot "
                             "0123456789ABCDEF00003331 -- lane24 and the "
                             "name Bambu_AMS_HT_1 freed for reuse -- its "
                             "bay is free now, but with no pool nothing "
                             "claims it: AFC_BRIDGEBOX_ASSIGN another unit "
                             "onto it while that unit is online, or "
                             "RESTART. NOTE: your roster: option still "
                             "lists this uid -- remove it there too, the "
                             "option overrides the recorded roster.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == []


class TestAfcBridgeBoxDropPin:
    """_drop_pin unpins a uid, first writing the owners a restart guesses."""

    def test_a_pin_changed_before_prep_writes_what_the_file_holds(
            self, tmp_path, monkeypatch):
        # Before PREP, AFC.var.unit holds what this start held for each bay;
        # an owner a claim records before PREP waits for the tick after.
        m = _p3_upgrade(tmp_path, monkeypatch)
        assert m._state_get(SEC, "bay_owner") is None
        m.printer.afc.prep_done = False
        m._set_bay_owner("Bravo", D)
        _p3_quiet(m)
        m._drop_pin(A)
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert m._bay_owner_pending is True
        assert (m._state_get(SEC, "name_map"),
                m._state_get(SEC, "lane_map")) == ("", "")
        assert (m._name_map, m._lane_map) == ({}, {})
        m.printer.afc.prep_done = True
        m._persist_bay_owner()
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha, DDDD:Bravo"
        assert m._bay_owner_pending is False
        assert _p3_log(m) == []


class TestAfcBridgeBoxCmdAFCBridgeboxAssign:
    """ASSIGN pins a uid onto a named pool bay, claiming it live if it can."""

    @staticmethod
    def _pool(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
              **options: Any) -> afcBridgeBox:
        """
        AAAA recorded on Alpha, one free AMS bay Bravo and one free HT bay
        Hot, with no bridge, so ASSIGN only pins.

        :return afcBridgeBox: the master
        """
        return _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                         pool_ams=2, pool_ht=1, ams_names="Alpha, Bravo",
                         ht_names="Hot", **options)

    def test_assign_pins_a_uid_to_a_named_bay(self, tmp_path, monkeypatch):
        m = self._pool(tmp_path, monkeypatch)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, NAME="Bravo")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert _p3_bay(m, "Bravo")["uid"] == C
        assert m._state_get(SEC, "lane_map") == "AAAA:24:4, CCCC:28:4"
        assert m._state_get(SEC, "name_map") == "AAAA:Alpha, CCCC:Bravo"
        assert m._state_get(SEC, "roster") == "boxed:AAAA, boxed:CCCC"
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Bravo' "
            "(lane28-lane31, T28-T31) -- pinned; it claims this bay when next "
            "online."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_a_pinned_bay_survives_a_restart(self, tmp_path, monkeypatch):
        m = self._pool(tmp_path, monkeypatch)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, NAME="Bravo")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Bravo' "
            "(lane28-lane31, T28-T31) -- pinned; it claims this bay when next "
            "online."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        m2 = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}, boxed:{C}",
                       pool_ams=2, pool_ht=1, ams_names="Alpha, Bravo",
                       ht_names="Hot")
        assert _p3_unit_uids(m2)["Bravo"] == C
        assert m2._lane_map[C] == (28, 4)

    def test_an_option_boxed_unit_the_bus_confirmed_ams2_is_claimed_ams2(
            self, tmp_path, monkeypatch):
        # A roster: option boxed is unconfirmed: the ams2 the recorded
        # roster holds for the uid stands, so a moved AMS 2 Pro keeps its
        # heater.
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}", pool_ams=2,
                      ams_names="Alpha, Bravo", online=(A,))
        m._state_set({SEC: {"roster": f"ams2:{A}"}})
        assert m._model_for_uid(A) == "ams2"
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A, NAME="Bravo")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert _p3_bay(m, "Bravo")["bound"] == A
        unit = m.printer.lookup_object("AFC_BambuAMS Bravo")
        assert unit.ams_model == "ams2"
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned AAAA to bay 'Bravo' "
            "(lane28-lane31, T28-T31) -- claimed LIVE, no restart."))]
        assert _p3_log(m) == [
            ("debug", "AFC bambu Bravo: chain index not resolved yet (UID "
                      "AAAA); holding this unit's registrations until the "
                      "chain map arrives"),
            ("info", "AFC bambu Bravo: claimed UID AAAA as ams2 and brought "
                     "online live (ams_index=0)."),
            ("info", "AFC_BridgeBox chain1: CLAIMED AAAA as ams2 onto Bravo "
                     "(4 lanes) -- live, no restart.")]
        assert _p3_console(m) == [END]
        m._state_set({SEC: {"roster": f"boxed:{A}"}})
        assert m._model_for_uid(A) == "boxed"

    def test_assign_refuses_an_occupied_bay(self, tmp_path, monkeypatch):
        m = self._pool(tmp_path, monkeypatch)
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=D, NAME="Alpha"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: bay 'Alpha' is already assigned to AAAA -- "
            "AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=AAAA first")

    def test_assign_refuses_a_family_mismatch(self, tmp_path, monkeypatch):
        # HHHH is a known HT; an HT (one lane) cannot take a four-lane bay.
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}, ht:{H}",
                      pool_ams=2, pool_ht=1, ams_names="Alpha, Bravo",
                      ht_names="Hot")
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=H, NAME="Bravo"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: HHHH is an HT unit but bay 'Bravo' is an "
            "AMS bay (their lane counts differ)")

    def test_assign_refuses_an_unknown_bay(self, tmp_path, monkeypatch):
        m = self._pool(tmp_path, monkeypatch)
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=C, NAME="Nope"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: no pool bay named 'Nope' (bays: Alpha, "
            "Bravo, Hot)")

    def test_assign_needs_both_uid_and_name(self, tmp_path, monkeypatch):
        m = self._pool(tmp_path, monkeypatch)
        assert _p3_refused(m, tmp_path, "ASSIGN", _p3_cmd(NAME="Bravo")) == (
            "AFC_BRIDGEBOX_ASSIGN: give UID=<unit_uid> and NAME=<bay name>")
        # UID alone opens the picker, which needs the unit on a bay.
        assert _p3_refused(m, tmp_path, "ASSIGN", _p3_cmd(UID=C)) == (
            "AFC_BRIDGEBOX_ASSIGN: CCCC is not on any bay -- plug it in "
            "first, or give NAME=<bay name> to pin it")

    def test_assign_moves_a_unit_between_bays(self, tmp_path, monkeypatch):
        m = _p3_named(tmp_path, monkeypatch, ready=False, online=None)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_p3_cmd(UID=C, NAME="Bravo"))
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, NAME="Charlie")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert _p3_bay(m, "Bravo")["uid"] is None
        assert _p3_bay(m, "Charlie")["uid"] == C
        assert m._state_get(SEC, "name_map") == "AAAA:Alpha, CCCC:Charlie"
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Charlie' "
            "(lane32-lane35, T32-T35) -- pinned; it claims this bay when next "
            "online."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_assign_with_no_name_pops_the_picker(self, tmp_path, monkeypatch):
        m = _p3_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(C, "boxed") is not None
        assert _p3_bay(m, "Bravo")["bound"] == C
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert _p3_console(m) == _p3_popup(
            "New AMS on Bravo",
            ["UID CCCC is on 'Bravo' (its T# and lanes are live).",
             "It is saved on this bay once it has been online 15s.",
             "Keep it here, or move it to another named bay:"],
            ["Charlie|AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=CCCC "
             "NAME=Charlie|primary"])
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: opened the bay picker for CCCC.")]
        assert _p3_log(m) == []

    def test_assign_closes_the_dialog_on_success(self, tmp_path, monkeypatch):
        # A picker button runs ASSIGN, and Mainsail does not close the dialog
        # itself.
        m = _p3_named(tmp_path, monkeypatch)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, NAME="Bravo")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert _p3_console(m) == [END]
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Bravo' "
            "(lane28-lane31, T28-T31) -- pinned; it claims this bay when next "
            "online."))]
        assert _p3_log(m) == []
        bay = _p3_bay(m, "Bravo")
        assert (bay["uid"], bay["bound"]) == (C, None)

    def test_assign_with_no_name_needs_a_placed_unit(self, tmp_path,
                                                     monkeypatch):
        m = _p3_named(tmp_path, monkeypatch)
        assert _p3_refused(m, tmp_path, "ASSIGN", _p3_cmd(UID="ZZZZ")) == (
            "AFC_BRIDGEBOX_ASSIGN: ZZZZ is not on any bay -- plug it in "
            "first, or give NAME=<bay name> to pin it")

    def test_assign_refuses_a_uid_the_roster_option_does_not_list(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}", pool_ams=2,
                      pool_ht=1)
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=C, NAME="Bambu_AMS_2"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: CCCC is not in your roster: option. While "
            "roster: is set it is the whole roster, so a unit it does not "
            "list gets no bay of its own at restart and cannot be pinned -- "
            "add boxed:CCCC to roster: first, RESTART, then assign it.")
        assert m._state_get(SEC, "name_map") == "AAAA:Bambu_AMS_1"
        assert _p3_bay(m, "Bambu_AMS_2")["uid"] is None

    def test_assign_still_moves_a_listed_uid(self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}", pool_ams=2,
                      pool_ht=1)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A, NAME="Bambu_AMS_2")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert m._state_get(SEC, "name_map") == "AAAA:Bambu_AMS_2"
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned AAAA to bay 'Bambu_AMS_2' "
            "(lane28-lane31, T28-T31) -- pinned; it claims this bay when next "
            "online."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_assign_onto_a_spare_wearing_a_recorded_name_takes_it_over(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}, boxed:{C}",
                      pool_ams=2, pool_ht=1)
        m._state_set({SEC: {"roster": f"boxed:{A}"}})         # C dropped
        m2 = _p3_chain(tmp_path, monkeypatch, roster="", pool_ams=2,
                       pool_ht=1)
        _p3_quiet(m2)
        cmd = _p3_cmd(UID=D, NAME="Bambu_AMS_2")
        m2.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert m2._state_get(SEC, "name_map") == (
            "AAAA:Bambu_AMS_1, DDDD:Bambu_AMS_2")
        assert m2._name_map == {A: "Bambu_AMS_1", D: "Bambu_AMS_2"}
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned DDDD to bay 'Bambu_AMS_2' "
            "(lane28-lane31, T28-T31) -- pinned; it claims this bay when next "
            "online."))]
        assert _p3_log(m2) == []
        assert _p3_console(m2) == [END]

    @staticmethod
    def _waiting(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                 **options: Any) -> afcBridgeBox:
        """
        :return afcBridgeBox: the named chain started through klippy:ready,
            with CCCC online and PREP not run yet
        """
        m = _p3_named(tmp_path, monkeypatch, online=(C,), **options)
        m.printer.afc.prep_done = False
        _p3_quiet(m)
        return m

    def test_assign_before_prep_pins_and_says_it_waits(self, tmp_path,
                                                       monkeypatch):
        m = self._waiting(tmp_path, monkeypatch)
        cmd = _p3_cmd(UID=C, NAME="Charlie")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert (_p3_bay(m, "Charlie")["uid"],
                _p3_bay(m, "Charlie")["bound"]) == (C, None)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Charlie' "
            "(lane32-lane35, T32-T35) -- pinned; it claims this bay once PREP "
            "finishes."))]
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: unit claims wait for PREP")]
        assert _p3_console(m) == [END]

    def test_assign_before_prep_without_a_pool_says_to_run_it_again(
            self, tmp_path, monkeypatch):
        m = self._waiting(tmp_path, monkeypatch)
        m.pool_ams = m.pool_ht = 0                    # nothing claims later
        cmd = _p3_cmd(UID=C, NAME="Charlie")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert _p3_bay(m, "Charlie")["bound"] is None
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Charlie' "
            "(lane32-lane35, T32-T35) -- pinned; PREP has not finished, run "
            "this again once it has."))]
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: unit claims wait for PREP")]
        assert _p3_console(m) == [END]

    def test_assign_says_the_unit_is_claimed(self, tmp_path, monkeypatch):
        m = _p3_named(tmp_path, monkeypatch, online=(C,),
                      print_state="printing")
        afc = m.printer.afc
        lane5 = _P3OtherLane("lane5", ["T28"])
        afc.lanes["lane5"] = lane5
        afc.tool_cmds["T28"] = "lane5"
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, NAME="Bravo")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert _p3_bay(m, "Bravo")["bound"] == C
        assert (afc.tool_cmds["T28"], lane5.map) == ("lane5", ["T28"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Bravo' "
            "(lane28-lane31, T28-T31) -- claimed LIVE, no restart."))]
        assert _p3_log(m) == [
            ("debug", "AFC bambu Bravo: chain index not resolved yet (UID "
                      "CCCC); holding this unit's registrations until the "
                      "chain map arrives"),
            ("info", "AFC bambu Bravo: claimed UID CCCC as boxed and brought "
                     "online live (ams_index=0)."),
            ("info", "AFC_BridgeBox chain1: lane28 takes T28 from lane5 once "
                     "the print ends, as the print may be using it; until "
                     "then lane28 has no T#."),
            ("info", "AFC_BridgeBox chain1: CLAIMED CCCC as boxed onto Bravo "
                     "(4 lanes) -- live, no restart.")]
        assert _p3_console(m) == [END]

    def test_assign_without_a_name_points_a_waiting_unit_at_replace(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch)
        _p3_tick(m, 100, 101)
        assert _p3_refused(m, tmp_path, "ASSIGN", _p3_cmd(UID=E)) == (
            "AFC_BRIDGEBOX_ASSIGN: EEEE is on the chain, but every AMS bay is "
            "taken -- AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE gives it "
            "the bay of a unit that is offline")

    def test_assign_names_no_replace_while_every_holder_is_online(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, D, E, H))
        _p3_tick(m, 100, 101)
        assert _p3_refused(m, tmp_path, "ASSIGN", _p3_cmd(UID=E)) == (
            "AFC_BRIDGEBOX_ASSIGN: EEEE is on the chain, but every AMS bay "
            "is taken")

    def test_assign_says_when_the_chain_has_no_bay_of_the_family(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      online=(A, G), htmask=1 << 5, pool_ams=1)
        _p3_tick(m, 100, 101)
        assert _p3_refused(m, tmp_path, "ASSIGN", _p3_cmd(UID=G)) == (
            "AFC_BRIDGEBOX_ASSIGN: GGGG is on the chain, but chain chain1 "
            "has no HT bay")

    def test_assign_off_the_bay_refuses_and_force_clears_the_record(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, H),
                      recorded=f"boxed:{A}, boxed:{D}, ht:{H}")
        _p3_tick(m, 100, 111)
        assert _p3_bay(m, "Bambu_AMS_2")["uid"] == D
        ext = m.printer.afc.tools["extruder"]
        ext.lane_loaded = "lane29"
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=D, NAME="Bambu_AMS_3"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: AFC records lane29 on Bambu_AMS_2 as "
            "loaded to the toolhead, from DDDD. While Bambu_AMS_2 is "
            "unclaimed its lanes are not registered, so no unload or "
            "UNSET_LANE_LOADED reaches them -- plug DDDD back in and unload "
            "it, or take the filament out by hand and FORCE=1 clears the "
            "record and moves DDDD")
        cmd = _p3_cmd(UID=D, NAME="Bambu_AMS_3", FORCE=1)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert ext.lane_loaded is None
        assert _p3_bay(m, "Bambu_AMS_2")["uid"] is None
        assert _p3_bay(m, "Bambu_AMS_3")["uid"] == D
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned DDDD to bay 'Bambu_AMS_3' "
            "(lane32-lane35, T32-T35) -- cleared lane29 from the toolhead -- "
            "pinned; it claims this bay when next online."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_assign_of_an_offline_unit_says_to_run_it_again(self, tmp_path,
                                                            monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, pool_ams=0, pool_ht=0)
        m.cmd_AFC_BRIDGEBOX_FORGET(_p3_cmd(UID=D))
        _p3_quiet(m)
        cmd = _p3_cmd(UID=G, NAME="Bambu_AMS_4")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: assigned GGGG to bay 'Bambu_AMS_4' "
            "(lane36-lane39, T36-T39) -- pinned; with no pool nothing claims "
            "it, so run this again once GGGG is online and PREP has "
            "finished, and after every restart.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_the_picker_of_a_reserved_offline_unit_says_so(self, tmp_path,
                                                           monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch)
        _p3_tick(m, 100, 111)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=D)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: opened the bay picker for DDDD.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == _p3_popup(
            "New AMS on Bambu_AMS_4",
            ["UID DDDD is on 'Bambu_AMS_4' (reserved for it; its T# and lanes "
             "are not live yet).",
             "It is saved on this bay.",
             "No other free bay of this type to move it to."])
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: opened the bay picker for AAAA.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == _p3_popup(
            "New AMS on Bambu_AMS_1",
            ["UID AAAA is on 'Bambu_AMS_1' (its T# and lanes are live).",
             "It is saved on this bay.",
             "No other free bay of this type to move it to."])

    def test_assign_onto_a_spare_is_held_across_an_auto_drop(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, auto_drop=True, online=(C,),
                      uids=(A, C))
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, NAME="Bambu_AMS_3")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        bay = _p3_bay(m, "Bambu_AMS_3")
        assert bay["bound"] == C
        assert _p3_console(m) == [END]
        _p3_online(m, ())
        _p3_tick(m, 100, 111)
        assert (bay["bound"], bay["uid"]) == (None, C)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Bambu_AMS_3' "
            "(lane32-lane35, T32-T35) -- claimed LIVE, no restart."))]
        assert _p3_log(m) == [
            ("debug", "AFC bambu Bambu_AMS_3: chain index not resolved yet "
                      "(UID CCCC); holding this unit's registrations until "
                      "the chain map arrives"),
            ("info", "AFC bambu Bambu_AMS_3: claimed UID CCCC as boxed and "
                     "brought online live (ams_index=0)."),
            ("info", "AFC_BridgeBox chain1: CLAIMED CCCC as boxed onto "
                     "Bambu_AMS_3 (4 lanes) -- live, no restart."),
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_3 (UID CCCC "
                     "offline >10s); lanes dropped live, slot kept for "
                     "re-plug")]

    def test_assign_refuses_a_name_saved_for_another_uid(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=(), uids=(A,))
        m._state_set({SEC: {"roster": f"boxed:{A}, boxed:ZZZZ"}})
        m._name_map["ZZZZ"] = "Bambu_AMS_3"
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=C, NAME="Bambu_AMS_3"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: bay 'Bambu_AMS_3' is saved for ZZZZ -- "
            "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=ZZZZ or "
            "AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=ZZZZ first")
        assert _p3_bay(m, "Bambu_AMS_3")["uid"] is None
        assert m._name_map["ZZZZ"] == "Bambu_AMS_3"

    def test_a_new_ams_on_a_spare_is_refused_an_ht_bay(self, tmp_path,
                                                       monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=(C,), uids=(A, C))
        _p3_tick(m, 100, 100)
        assert _p3_bay(m, "Bambu_AMS_2")["bound"] == C
        assert m._model_for_uid(C) is None
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=C, NAME="Bambu_AMS_HT_1"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: CCCC is an AMS unit but bay "
            "'Bambu_AMS_HT_1' is an HT bay (their lane counts differ)")
        assert _p3_bay(m, "Bambu_AMS_2")["bound"] == C
        assert _p3_bay(m, "Bambu_AMS_HT_1")["uid"] is None
        assert m._state_get(SEC, "roster") == "boxed:AAAA"
        assert C not in m._name_map

    def test_a_waiting_ams_is_refused_an_ht_bay(self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=1, pool_ht=1, online=(C,), uids=(A, C))
        _p3_tick(m, 100, 100)
        assert m._bay_of_uid(C) is None
        assert m._no_bay[C] == "boxed"
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=C, NAME="Bambu_AMS_HT_1"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: CCCC is an AMS unit but bay "
            "'Bambu_AMS_HT_1' is an HT bay (their lane counts differ)")
        assert _p3_bay(m, "Bambu_AMS_HT_1")["uid"] is None
        assert m._state_get(SEC, "roster") == "boxed:AAAA"

    def test_assign_refusals_name_the_command_to_run(self, tmp_path,
                                                     monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=(), uids=(A,))
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=C, NAME="Bambu_AMS_1"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: bay 'Bambu_AMS_1' is already assigned to "
            "AAAA -- AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=AAAA first")
        m._state_set({SEC: {"roster": f"boxed:{A}, boxed:ZZZZ"}})
        m._name_map["ZZZZ"] = "Bambu_AMS_3"
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=C, NAME="Bambu_AMS_3"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: bay 'Bambu_AMS_3' is saved for ZZZZ -- "
            "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=ZZZZ or "
            "AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=ZZZZ first")

    @staticmethod
    def _two_bays(tmp_path: pathlib.Path,
                  monkeypatch: pytest.MonkeyPatch) -> afcBridgeBox:
        """:return afcBridgeBox: AAAA claimed onto Alpha, Bravo free"""
        return _p3_claimed(tmp_path, monkeypatch, pool_ams=2,
                           ams_names="Alpha, Bravo")

    def test_moving_a_loaded_unit_refuses_without_force(
            self, tmp_path, monkeypatch):
        m = self._two_bays(tmp_path, monkeypatch)
        _p3_load(m, "lane27")
        refusal = _p3_refused(m, tmp_path, "ASSIGN",
                              _p3_cmd(UID=A, NAME="Bravo"))
        assert refusal == (
            "AFC_BRIDGEBOX_ASSIGN: AFC records lane27 on AAAA as loaded to "
            "the toolhead -- unload it first (UNSET_LANE_LOADED if the "
            "filament is already out), or FORCE=1 to clear it from the "
            "toolhead and release anyway")
        alpha = _p3_bay(m, "Alpha")
        assert (alpha["uid"], alpha["bound"]) == (A, A)
        assert _p3_bay(m, "Bravo")["uid"] is None

    def test_force_clears_it_and_moves(self, tmp_path, monkeypatch):
        m = self._two_bays(tmp_path, monkeypatch)
        lane = _p3_load(m, "lane27")
        drain_var_writes(m.printer)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A, NAME="Bravo", FORCE=1)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        alpha = _p3_bay(m, "Alpha")
        assert (alpha["uid"], alpha["bound"]) == (None, None)
        assert (lane.unassigned, lane.tool_loaded, lane.loaded_to_hub) == (
            True, False, False)
        assert "lane27" not in m.printer.afc.lanes
        assert m.printer.afc.tools["extruder"].lane_loaded is None
        assert drain_var_writes(m.printer)[-1]["system"]["extruders"] == {
            "extruder": {"lane_loaded": None}}
        assert _p3_bay(m, "Bravo")["uid"] == A
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned AAAA to bay 'Bravo' "
            "(lane28-lane31, T28-T31) -- pinned; it claims this bay when next "
            "online."))]
        assert _p3_log(m) == [
            ("debug", "Activating extruder lane: None"),
            ("info", "Manually removing lane27 loaded from toolhead"),
            ("info", "AFC_BridgeBox chain1: released Alpha (UID AAAA, "
                     "AFC_BRIDGEBOX_ASSIGN); lanes dropped live; cleared "
                     "lane27 from the toolhead")]
        assert _p3_console(m) == [END]

    def test_assign_claims_a_bay_whose_objects_have_only_the_basics(
            self, tmp_path, monkeypatch):
        # Bravo's unit object has none of the optional hooks, lane28 was
        # never built, the others have nothing wired, and AFC cannot save.
        m = _p3_named(tmp_path, monkeypatch, online=(C,))
        afc = m.printer.afc
        lanes = _p3_bare_bay(m, monkeypatch, "Bravo")
        # lane29 still names T5, kept for a lane that is gone.
        lanes[0].map = ["T5"]
        afc.tool_cmds["T5"] = "lane5"

        def full() -> None:
            """AFC's save, failing."""
            raise RuntimeError("disk full")

        monkeypatch.setattr(afc, "save_vars", full)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, NAME="Bravo")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        unit = m.printer.lookup_object("AFC_BambuAMS Bravo")
        assert (_p3_bay(m, "Bravo")["bound"], unit.claims) == (
            C, [(C, "boxed")])
        assert [(lane.unassigned, lane.map) for lane in lanes] == [
            (False, ["T29"]), (False, ["T30"]), (False, ["T31"])]
        assert afc.tool_cmds == {"T29": "lane29", "T30": "lane30",
                                 "T31": "lane31"}
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: assigned CCCC to bay "
                             "'Bravo' (lane28-lane31, T28-T31) -- claimed "
                             "LIVE, no restart.")]
        assert _p3_log(m) == [
            ("warning", "AFC_BridgeBox chain1: TcmdAssign lane29 failed: "
                        "disk full"),
            ("warning", "AFC_BridgeBox chain1: TcmdAssign lane30 failed: "
                        "disk full"),
            ("warning", "AFC_BridgeBox chain1: TcmdAssign lane31 failed: "
                        "disk full"),
            ("info", "AFC_BridgeBox chain1: CLAIMED CCCC as boxed onto "
                     "Bravo (3 lanes) -- live, no restart.")]
        assert _p3_console(m) == [("respond_raw", "// action:prompt_end")]


class TestAfcBridgeBoxCmdAFCBridgeboxUnassign:
    """UNASSIGN unpins a uid from its bay, keeping what it learned."""

    @staticmethod
    def _pinned(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                **options: Any) -> afcBridgeBox:
        """
        :return afcBridgeBox: AAAA recorded on Alpha and CCCC assigned to
            Bravo, with Hot free; the console and log start empty
        """
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=2, pool_ht=1, ams_names="Alpha, Bravo",
                      ht_names="Hot", **options)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_p3_cmd(UID=C, NAME="Bravo"))
        _p3_quiet(m)
        return m

    def test_unassign_frees_the_bay(self, tmp_path, monkeypatch):
        m = self._pinned(tmp_path, monkeypatch)
        cmd = _p3_cmd(UID=C)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert _p3_bay(m, "Bravo")["uid"] is None
        assert m._state_get(SEC, "name_map") == "AAAA:Alpha"
        assert m._state_get(SEC, "lane_map") == "AAAA:24:4"
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bravo' "
            "(learned values stay with the unit). It takes a free bay of "
            "its family, its last one first, and is saved there once it "
            "has been online 15s."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_unassign_keeps_the_learned_values(self, tmp_path, monkeypatch):
        # Unlike FORGET, it leaves calibration with the unit.
        m = self._pinned(tmp_path, monkeypatch)
        m._state_set({m._learned_section(C): {"afc_bowden_length": "3632.0"}})
        cmd = _p3_cmd(UID=C)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert m._learned_for(C) == {"afc_bowden_length": "3632.0"}
        bay = _p3_bay(m, "Bravo")
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bravo' "
            "(learned values stay with the unit). It takes a free bay of "
            "its family, its last one first, and is saved there once it "
            "has been online 15s."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_unassign_by_bay_name(self, tmp_path, monkeypatch):
        m = self._pinned(tmp_path, monkeypatch)
        cmd = _p3_cmd(NAME="Bravo")
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        bay = _p3_bay(m, "Bravo")
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bravo' "
            "(learned values stay with the unit). It takes a free bay of "
            "its family, its last one first, and is saved there once it "
            "has been online 15s."))]

    def test_unassign_refuses_an_online_unit(self, tmp_path, monkeypatch):
        m = self._pinned(tmp_path, monkeypatch)
        _p3_wire(m, (A, C), uids=(A, C))
        assert _p3_refused(m, tmp_path, "UNASSIGN", _p3_cmd(UID=C)) == (
            "AFC_BRIDGEBOX_UNASSIGN: CCCC is ONLINE on the chain right now "
            "-- unhook it first, or FORCE=1 to unassign it live (drops its "
            "lanes)")
        cmd = _p3_cmd(UID=C, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert _p3_bay(m, "Bravo")["uid"] is None
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bravo' "
            "(learned values stay with the unit). It takes a free bay of "
            "its family, its last one first, and is saved there once it "
            "has been online 15s."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_unassign_closes_the_dialog(self, tmp_path, monkeypatch):
        m = self._pinned(tmp_path, monkeypatch)
        cmd = _p3_cmd(UID=C)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert _p3_console(m) == [END]
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bravo' "
            "(learned values stay with the unit). It takes a free bay of "
            "its family, its last one first, and is saved there once it "
            "has been online 15s."))]
        assert _p3_log(m) == []
        bay = _p3_bay(m, "Bravo")
        assert (bay["uid"], bay["bound"]) == (None, None)

    def test_unassign_keeps_the_owner_and_the_held_records(self, tmp_path,
                                                          monkeypatch):
        m = _p3_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        _p3_lane(m, "lane24").spool_id = 159
        m._release_pool_unit(A)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert m._held["Alpha"]["uid"] == A
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned AAAA from bay 'Alpha' "
            "(learned values stay with the unit). It takes a free bay of "
            "its family, its last one first, and is saved there once it "
            "has been online 15s."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    @pytest.mark.parametrize("ticked", [True, False])
    def test_unassign_and_assign_give_the_new_unit_nothing(
            self, tmp_path, monkeypatch, ticked):
        # AAAA (offline) is swapped for CCCC before CCCC is plugged in, then
        # the printer restarts: a guess from CCCC's pin on Alpha would hand
        # it AAAA's spool.
        m = _p3_upgrade(tmp_path, monkeypatch)
        m.printer.afc.save_vars()                     # PREP's save
        drain_var_writes(m.printer)
        if ticked:
            m._persist_bay_owner()                    # the tick after PREP
        _p3_quiet(m)
        unassign = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(unassign)
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assign = _p3_cmd(UID=C, NAME="Alpha")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(assign)
        m.printer.afc.save_vars()
        assert drain_var_writes(m.printer)[-1]["Alpha"] == UPGRADE_VAR["Alpha"]
        assert unassign.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned AAAA from bay 'Alpha' "
            "(learned values stay with the unit). It takes a free bay of "
            "its family, its last one first, and is saved there once it "
            "has been online 15s."))]
        assert assign.messages == [("respond_info", (
            "AFC_BridgeBox chain1: assigned CCCC to bay 'Alpha' "
            "(lane24-lane27, T24-T27) -- pinned; it claims this bay when "
            "next online."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END, END]
        again = _p3_named(tmp_path, monkeypatch, var=UPGRADE_VAR,
                          roster=f"boxed:{A}, boxed:{C}")
        assert again._pins_at_boot.get(C) == "Alpha"
        assert {b: e["uid"] for b, e in again._held.items()} == {"Alpha": A}
        assert again._claim_pool_unit(C, "boxed") is not None
        lane = _p3_lane(again, "lane24")
        assert (lane.spool_id, lane.material, lane.weight) == (None, None, 0.0)
        assert lane.map == ["T24"]

    def test_unassign_refuses_and_force_clears_the_record(self, tmp_path,
                                                          monkeypatch):
        m = _p3_loaded(tmp_path, monkeypatch)
        assert _p3_refused(m, tmp_path, "UNASSIGN", _p3_cmd(UID=D)) == (
            "AFC_BRIDGEBOX_UNASSIGN: AFC records lane36 on Bambu_AMS_4 as "
            "loaded to the toolhead, from DDDD. While Bambu_AMS_4 is "
            "unclaimed its lanes are not registered, so no unload or "
            "UNSET_LANE_LOADED reaches them -- plug DDDD back in and "
            "unload it, or take the filament out by hand and FORCE=1 "
            "clears the record and unassigns DDDD")
        cmd = _p3_cmd(UID=D, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert m.printer.afc.tools["extruder"].lane_loaded is None
        assert _p3_bay(m, "Bambu_AMS_4")["uid"] is None
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned DDDD from bay 'Bambu_AMS_4' "
            "-- cleared lane36 from the toolhead (learned values stay with "
            "the unit). It takes a free bay of its family, its last one "
            "first, and is saved there once it has been online 15s."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]

    def test_unassign_of_an_online_unit_names_the_lane(self, tmp_path,
                                                       monkeypatch):
        m = _p3_bound(tmp_path, monkeypatch)
        assert _p3_refused(m, tmp_path, "UNASSIGN", _p3_cmd(UID=D)) == (
            "AFC_BRIDGEBOX_UNASSIGN: AFC records lane37 on DDDD as loaded "
            "to the toolhead -- unload it first (UNSET_LANE_LOADED if the "
            "filament is already out), or FORCE=1 to clear it from the "
            "toolhead and release anyway")

    def test_a_live_unassign_rehomes_and_saves_on_the_next_tick(
            self, tmp_path, monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=(C,), uids=(A, C))
        _p3_tick(m, 100, 100)
        _p3_tick(m, 116, 116)
        assert m._name_map[C] == "Bambu_AMS_2"
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert _p3_bay(m, "Bambu_AMS_2")["uid"] is None
        assert C not in m._name_map
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bambu_AMS_2' "
            "-- lanes dropped live (learned values stay with the unit). It "
            "takes a free bay of its family, its last one first, and is "
            "saved there once it has been online 15s."))]
        assert _p3_log(m) == [
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID "
                     "CCCC, AFC_BRIDGEBOX_UNASSIGN); lanes dropped live")]
        assert _p3_console(m) == [END]
        _p3_quiet(m)
        _p3_tick(m, 120, 120)
        assert _p3_bay(m, "Bambu_AMS_2")["bound"] == C
        assert m._name_map[C] == "Bambu_AMS_2"
        assert _p3_log(m) == [
            ("debug", "AFC bambu Bambu_AMS_2: chain index not resolved yet "
                      "(UID CCCC); holding this unit's registrations until "
                      "the chain map arrives"),
            ("info", "AFC bambu Bambu_AMS_2: claimed UID CCCC as boxed and "
                     "brought online live (ams_index=0)."),
            ("info", "AFC_BridgeBox chain1: CLAIMED CCCC as boxed onto "
                     "Bambu_AMS_2 (4 lanes) -- live, no restart."),
            ("info", "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 "
                     "(lane28-lane31, T28-T31); it comes back there after "
                     "a restart.")]

    def test_with_a_pool_it_floats_for_the_session(self, tmp_path,
                                                   monkeypatch):
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}", pool_ams=3,
                      pool_ht=1, online=(C,), uids=(A, C))
        _p3_tick(m, 100, 100)
        assert _p3_bay(m, "Bambu_AMS_2")["bound"] == C
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bambu_AMS_2' -- "
            "lanes dropped live (learned values stay with the unit). roster: "
            "is set and does not list it, so it takes a free bay of its "
            "family for this session only and is not saved there; add "
            "boxed:CCCC to roster: and RESTART to give it a bay of its own.")]
        assert _p3_console(m) == [END]
        _p3_tick(m, 101, 139)
        assert _p3_bay(m, "Bambu_AMS_2")["bound"] == C
        assert C not in m._name_map
        assert _p3_log(m) == [
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID "
                     "CCCC, AFC_BRIDGEBOX_UNASSIGN); lanes dropped live"),
            ("debug", "AFC bambu Bambu_AMS_2: chain index not resolved yet "
                      "(UID CCCC); holding this unit's registrations until "
                      "the chain map arrives"),
            ("info", "AFC bambu Bambu_AMS_2: claimed UID CCCC as boxed and "
                     "brought online live (ams_index=0)."),
            ("info", "AFC_BridgeBox chain1: CLAIMED CCCC as boxed onto "
                     "Bambu_AMS_2 (4 lanes) -- live, no restart."),
            ("info", "AFC_BridgeBox chain1: NEW unit(s) on the chain: "
                     "boxed:CCCC -- recorded. Add boxed:CCCC to roster: to "
                     "enroll it (the option is set and overrides the "
                     "file); the next restart gives it the lowest free bay "
                     "of its family, which need not be the one it is on "
                     "now.")]

    def test_an_unlisted_ht_is_named_as_an_ht(self, tmp_path, monkeypatch):
        # Not recorded yet: the bay it held says what it is.
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}", pool_ams=3,
                      pool_ht=1, online=(H,), uids=(A, "", "", "", H))
        _p3_tick(m, 100, 100)
        assert _p3_bay(m, "Bambu_AMS_HT_1")["bound"] == H
        _p3_quiet(m)
        cmd = _p3_cmd(UID=H, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned HHHH from bay "
            "'Bambu_AMS_HT_1' -- lanes dropped live (learned values stay "
            "with the unit). roster: is set and does not list it, so it "
            "takes a free bay of its family for this session only and is "
            "not saved there; add ht:HHHH to roster: and RESTART to give "
            "it a bay of its own."))]
        assert _p3_log(m) == [
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_HT_1 (UID "
                     "HHHH, AFC_BRIDGEBOX_UNASSIGN); lanes dropped live")]
        assert _p3_console(m) == [END]
        bay = _p3_bay(m, "Bambu_AMS_HT_1")
        assert (bay["uid"], bay["bound"]) == (None, None)

    def test_without_a_pool_it_gets_no_bay_at_restart(self, tmp_path,
                                                      monkeypatch):
        _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{A}, boxed:{B}")
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{B}")
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned AAAA from bay 'Bambu_AMS_1' "
            "(learned values stay with the unit). roster: is set and does "
            "not list it, so it gets no bay at the next restart; add "
            "boxed:AAAA to roster: to give it one."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        assert [(pu["name"], pu["uid"]) for pu in m._pool_units] == [
            ("Bambu_AMS_2", B)]
        assert (m._state_get(SEC, "name_map"), m._name_map) == (
            "BBBB:Bambu_AMS_2", {B: "Bambu_AMS_2"})
        m2 = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{B}")
        assert _p3_unit_uids(m2) == {"Bambu_AMS_2": "BBBB"}

    def test_without_a_pool_an_unlisted_ht_is_named_as_an_ht(
            self, tmp_path, monkeypatch):
        # The option is not written to the file: the name kept for it says
        # what it is.
        _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{B}, ht:{H}")
        m = _p3_chain(tmp_path, monkeypatch, roster=f"boxed:{B}")
        _p3_quiet(m)
        cmd = _p3_cmd(UID=H)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned HHHH from bay "
            "'Bambu_AMS_HT_1' (learned values stay with the unit). roster: "
            "is set and does not list it, so it gets no bay at the next "
            "restart; add ht:HHHH to roster: to give it one."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        assert [(pu["name"], pu["uid"]) for pu in m._pool_units] == [
            ("Bambu_AMS_1", B)]
        assert (m._state_get(SEC, "name_map"), m._name_map) == (
            "BBBB:Bambu_AMS_1", {B: "Bambu_AMS_1"})

    def test_refuses_without_force_and_changes_nothing(
            self, tmp_path, monkeypatch):
        m = _p3_claimed(tmp_path, monkeypatch)
        lane = _p3_load(m, "lane25")
        assert _p3_refused(m, tmp_path, "UNASSIGN", _p3_cmd(UID=A)) == (
            "AFC_BRIDGEBOX_UNASSIGN: AFC records lane25 on AAAA as loaded "
            "to the toolhead -- unload it first (UNSET_LANE_LOADED if the "
            "filament is already out), or FORCE=1 to clear it from the "
            "toolhead and release anyway")
        bay = _p3_bay(m, "Bambu_AMS_1")
        assert (bay["uid"], bay["bound"]) == (A, A)
        assert lane.tool_loaded is True
        assert m.printer.afc.tools["extruder"].lane_loaded == "lane25"
        assert "lane25" in m.printer.afc.lanes

    def test_force_clears_the_toolhead_record_before_pooling(
            self, tmp_path, monkeypatch):
        m = _p3_claimed(tmp_path, monkeypatch)
        afc = m.printer.afc
        lane = _p3_load(m, "lane25")
        afc.current_loading = "lane25"
        drain_var_writes(m.printer)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert _p3_bay(m, "Bambu_AMS_1")["bound"] is None
        assert (lane.unassigned, lane.tool_loaded, lane.loaded_to_hub) == (
            True, False, False)
        assert lane.status == AFCLaneState.NONE
        assert "lane25" not in afc.lanes
        assert afc.tools["extruder"].lane_loaded is None
        assert drain_var_writes(m.printer)[-1]["system"]["extruders"] == {
            "extruder": {"lane_loaded": None}}
        # AFC's UNSET_LANE_LOADED path drops the toolchange bookkeeping.
        assert afc.current_loading is None
        assert afc.spool.calls == [
            ("set_active_spool", (None,), {}),
            ("set_active_spool", ("",), {})]
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned AAAA from bay 'Bambu_AMS_1' "
            "-- lanes dropped live (learned values stay with the unit). It "
            "takes a free bay of its family, its last one first, and is "
            "saved there once it has been online 15s."))]
        assert _p3_log(m) == [
            ("debug", "Activating extruder lane: None"),
            ("info", "Manually removing lane25 loaded from toolhead"),
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID "
                     "AAAA, AFC_BRIDGEBOX_UNASSIGN); lanes dropped live; "
                     "cleared lane25 from the toolhead")]
        assert _p3_console(m) == [END]

    def test_force_on_a_live_unit_clears_it_too(self, tmp_path, monkeypatch):
        m = _p3_claimed(tmp_path, monkeypatch, online=True)
        lane = _p3_load(m, "lane24")
        drain_var_writes(m.printer)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert _p3_bay(m, "Bambu_AMS_1")["bound"] is None
        assert (lane.unassigned, lane.tool_loaded, lane.loaded_to_hub) == (
            True, False, False)
        assert "lane24" not in m.printer.afc.lanes
        assert m.printer.afc.tools["extruder"].lane_loaded is None
        assert drain_var_writes(m.printer)[-1]["system"]["extruders"] == {
            "extruder": {"lane_loaded": None}}
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned AAAA from bay 'Bambu_AMS_1' "
            "-- lanes dropped live (learned values stay with the unit). It "
            "takes a free bay of its family, its last one first, and is "
            "saved there once it has been online 15s."))]
        assert _p3_log(m) == [
            ("debug", "Activating extruder lane: None"),
            ("info", "Manually removing lane24 loaded from toolhead"),
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID "
                     "AAAA, AFC_BRIDGEBOX_UNASSIGN); lanes dropped live; "
                     "cleared lane24 from the toolhead")]
        assert _p3_console(m) == [END]

    def test_force_on_a_lane_in_another_toolhead_saves_it_cleared(
            self, tmp_path, monkeypatch):
        # Not the active tool, so AFC's own unset_lane_loaded (which saves)
        # is not the path: only the release's save writes extruder1 cleared.
        m = _p3_claimed(tmp_path, monkeypatch)
        afc = m.printer.afc
        afc.current_loading = "lane1"
        e1 = add_extruder(m.printer, "extruder1")
        lane = _p3_lane(m, "lane26")
        lane.extruder_obj = e1
        lane.tool_loaded = True
        e1.lane_loaded = "lane26"
        drain_var_writes(m.printer)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert _p3_bay(m, "Bambu_AMS_1")["bound"] is None
        assert (lane.unassigned, lane.tool_loaded, lane.loaded_to_hub) == (
            True, False, False)
        assert e1.lane_loaded is None
        # One save, the release's, with extruder1 cleared.
        assert [w["system"]["extruders"]
                for w in drain_var_writes(m.printer)] == [{
                    "extruder": {"lane_loaded": None},
                    "extruder1": {"lane_loaded": None}}]
        assert afc.current_loading == "lane1"     # the active tool untouched
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned AAAA from bay 'Bambu_AMS_1' "
            "-- lanes dropped live (learned values stay with the unit). It "
            "takes a free bay of its family, its last one first, and is "
            "saved there once it has been online 15s."))]
        assert _p3_log(m) == [
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID "
                     "AAAA, AFC_BRIDGEBOX_UNASSIGN); lanes dropped live; "
                     "cleared lane26 from the toolhead")]
        assert _p3_console(m) == [END]

    def test_an_offline_unit_with_nothing_loaded_needs_no_force(
            self, tmp_path, monkeypatch):
        m = _p3_claimed(tmp_path, monkeypatch)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=A)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert _p3_bay(m, "Bambu_AMS_1")["bound"] is None
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: unassigned AAAA from bay 'Bambu_AMS_1' "
            "-- lanes dropped live (learned values stay with the unit). It "
            "takes a free bay of its family, its last one first, and is "
            "saved there once it has been online 15s."))]
        assert _p3_log(m) == [
            ("info", "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID "
                     "AAAA, AFC_BRIDGEBOX_UNASSIGN); lanes dropped live")]
        assert _p3_console(m) == [END]

    def test_force_frees_a_bay_whose_objects_have_only_the_basics(
            self, tmp_path, monkeypatch):
        # Bravo's unit object has none of the optional hooks, lane28 was
        # never built and the others have nothing wired. lane29 is recorded
        # in the toolhead, AFC cannot say which lane is current, nor save.
        m = _p3_named(tmp_path, monkeypatch, online=(C,))
        afc = m.printer.afc
        lanes = _p3_bare_bay(m, monkeypatch, "Bravo")
        assert m._claim_pool_unit(C, "boxed") is not None
        lanes[0].tool_loaded = True

        def unknown() -> str:
            """AFC's current lane, unknown."""
            raise RuntimeError("no current lane")

        def full() -> None:
            """AFC's save, failing."""
            raise RuntimeError("disk full")

        monkeypatch.setattr(afc.function, "get_current_lane", unknown)
        monkeypatch.setattr(afc, "save_vars", full)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=C, FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        bay = _p3_bay(m, "Bravo")
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert [(lane.tool_loaded, lane.unassigned, lane.map)
                for lane in lanes] == [(False, True, [])] * 3
        # No status to keep, so nothing is held for CCCC.
        assert m._held == {}
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: unassigned CCCC from "
                             "bay 'Bravo' -- lanes dropped live (learned "
                             "values stay with the unit). It takes a free "
                             "bay of its family, its last one first, and "
                             "is saved there once it has been online 15s.")]
        assert _p3_log(m) == [
            ("info", "AFC_BridgeBox chain1: released Bravo (UID CCCC, "
                     "AFC_BRIDGEBOX_UNASSIGN); lanes dropped live; cleared "
                     "lane29 from the toolhead")]
        assert _p3_console(m) == [("respond_raw", "// action:prompt_end")]


class TestAfcBridgeBoxCmdAFCBridgeboxBays:
    """BAYS pops the bay manager: each bay, its unit, and what it offers."""

    @staticmethod
    def _bays(master: afcBridgeBox) -> FakeGcmd:
        """
        :param master: the chain master
        :return FakeGcmd: a BAYS run on a quiet console and log
        """
        _p3_quiet(master)
        cmd = _p3_cmd()
        master.cmd_AFC_BRIDGEBOX_BAYS(cmd)
        return cmd

    def test_bays_manager_lists_and_offers_unassign(self, tmp_path,
                                                    monkeypatch):
        m = _p3_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(C, "boxed") is not None
        cmd = self._bays(m)
        # Occupied bays get an Unassign button; free ones do not.
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Alpha [AMS]: AAAA",
             "Bravo [AMS]: CCCC (live)",
             "Charlie [AMS]: free",
             "Hot [HT]: free"],
            ["Unassign Alpha|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=AAAA "
             "FORCE=1|warning",
             "Unassign Bravo|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=CCCC "
             "FORCE=1|warning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Alpha [AMS]: AAAA\n  "
            "Bravo [AMS]: CCCC (live)\n  Charlie [AMS]: free\n  Hot [HT]: "
            "free"))]
        assert _p3_log(m) == []

    def test_a_live_unit_gets_no_unassign_button(self, tmp_path,
                                                 monkeypatch):
        m = _p3_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_p3_cmd(UID=C, NAME="Bravo"))
        _p3_online(m, (A,))
        m.printer.set_print_state("printing")
        cmd = self._bays(m)
        # An unclaimed bay's pin drops no lane or T#, so Bravo keeps it.
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Alpha [AMS]: AAAA (live) -- a print is active, unassign it once "
             "it ends",
             "Bravo [AMS]: CCCC",
             "Charlie [AMS]: free",
             "Hot [HT]: free"],
            ["Unassign Bravo|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=CCCC "
             "FORCE=1|warning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Alpha [AMS]: AAAA "
            "(live) -- a print is active, unassign it once it ends\n  "
            "Bravo [AMS]: CCCC\n  Charlie [AMS]: free\n  Hot [HT]: free"))]
        assert _p3_log(m) == []
        m.printer.set_print_state("complete")
        cmd = self._bays(m)
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Alpha [AMS]: AAAA (live)",
             "Bravo [AMS]: CCCC",
             "Charlie [AMS]: free",
             "Hot [HT]: free"],
            ["Unassign Alpha|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=AAAA "
             "FORCE=1|warning",
             "Unassign Bravo|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=CCCC "
             "FORCE=1|warning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Alpha [AMS]: AAAA "
            "(live)\n  Bravo [AMS]: CCCC\n  Charlie [AMS]: free\n  Hot [HT]: "
            "free"))]
        assert _p3_log(m) == []

    def test_the_bay_manager_lists_waiting_units_with_replace_first(
            self, tmp_path, monkeypatch):
        new = "0123456789ABCDEF01234567"             # a full 24-hex uid
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, new, H),
                      uids=(A, B, C, D, new, G, H))
        _p3_tick(m, 100, 111)
        cmd = self._bays(m)
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Bambu_AMS_1 [AMS]: AAAA (live)",
             "Bambu_AMS_2 [AMS]: BBBB (live)",
             "Bambu_AMS_3 [AMS]: CCCC (live)",
             "Bambu_AMS_4 [AMS]: DDDD",
             "Bambu_AMS_HT_1 [HT]: HHHH (live)",
             "Bambu_AMS_HT_2 [HT]: free",
             "Waiting: 0123456789ABCDEF01234567 [AMS] -- no free bay"],
            ["Replace for 01234567|AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=0123456789ABCDEF01234567|primary",
             "Unassign Bambu_AMS_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=AAAA FORCE=1|warning",
             "Unassign Bambu_AMS_2|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=BBBB FORCE=1|warning",
             "Unassign Bambu_AMS_3|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=CCCC FORCE=1|warning",
             "Unassign Bambu_AMS_4|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=DDDD FORCE=1|warning",
             "Unassign Bambu_AMS_HT_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=HHHH FORCE=1|warning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Bambu_AMS_1 [AMS]: "
            "AAAA (live)\n  Bambu_AMS_2 [AMS]: BBBB (live)\n  Bambu_AMS_3 "
            "[AMS]: CCCC (live)\n  Bambu_AMS_4 [AMS]: DDDD\n  "
            "Bambu_AMS_HT_1 [HT]: HHHH (live)\n  Bambu_AMS_HT_2 [HT]: "
            "free\n  Waiting: 0123456789ABCDEF01234567 [AMS] -- no free bay"))]
        assert _p3_log(m) == []

    def test_the_bay_manager_gives_a_roster_option_no_button(self, tmp_path,
                                                             monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, roster=FOUR_p3)
        _p3_tick(m, 100, 111)
        cmd = self._bays(m)
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Bambu_AMS_1 [AMS]: AAAA (live)",
             "Bambu_AMS_2 [AMS]: BBBB (live)",
             "Bambu_AMS_3 [AMS]: CCCC (live)",
             "Bambu_AMS_4 [AMS]: DDDD",
             "Bambu_AMS_HT_1 [HT]: HHHH (live)",
             "Bambu_AMS_HT_2 [HT]: free",
             "Waiting: EEEE [AMS] -- no free bay"],
            ["Unassign Bambu_AMS_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=AAAA FORCE=1|warning",
             "Unassign Bambu_AMS_2|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=BBBB FORCE=1|warning",
             "Unassign Bambu_AMS_3|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=CCCC FORCE=1|warning",
             "Unassign Bambu_AMS_4|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=DDDD FORCE=1|warning",
             "Unassign Bambu_AMS_HT_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=HHHH FORCE=1|warning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Bambu_AMS_1 [AMS]: "
            "AAAA (live)\n  Bambu_AMS_2 [AMS]: BBBB (live)\n  Bambu_AMS_3 "
            "[AMS]: CCCC (live)\n  Bambu_AMS_4 [AMS]: DDDD\n  "
            "Bambu_AMS_HT_1 [HT]: HHHH (live)\n  Bambu_AMS_HT_2 [HT]: "
            "free\n  Waiting: EEEE [AMS] -- no free bay"))]
        assert _p3_log(m) == []

    def test_the_bay_manager_offers_no_unassign_for_it(self, tmp_path,
                                                       monkeypatch):
        m = _p3_loaded(tmp_path, monkeypatch)
        cmd = self._bays(m)
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Bambu_AMS_1 [AMS]: AAAA (live)",
             "Bambu_AMS_2 [AMS]: BBBB (live)",
             "Bambu_AMS_3 [AMS]: CCCC (live)",
             "Bambu_AMS_4 [AMS]: DDDD -- lane36 in the toolhead from DDDD, "
             "which is not claimed: plug it back in and unload it before "
             "unassigning",
             "Bambu_AMS_HT_1 [HT]: HHHH (live)",
             "Bambu_AMS_HT_2 [HT]: free",
             "Waiting: EEEE [AMS] -- no free bay"],
            ["Replace for EEEE|AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE|primary",
             "Unassign Bambu_AMS_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=AAAA FORCE=1|warning",
             "Unassign Bambu_AMS_2|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=BBBB FORCE=1|warning",
             "Unassign Bambu_AMS_3|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=CCCC FORCE=1|warning",
             "Unassign Bambu_AMS_HT_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=HHHH FORCE=1|warning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Bambu_AMS_1 [AMS]: "
            "AAAA (live)\n  Bambu_AMS_2 [AMS]: BBBB (live)\n  Bambu_AMS_3 "
            "[AMS]: CCCC (live)\n  Bambu_AMS_4 [AMS]: DDDD -- lane36 in "
            "the toolhead from DDDD, which is not claimed: plug it back in "
            "and unload it before unassigning\n  Bambu_AMS_HT_1 [HT]: HHHH "
            "(live)\n  Bambu_AMS_HT_2 [HT]: free\n  Waiting: EEEE [AMS] -- "
            "no free bay"))]
        assert _p3_log(m) == []

    def test_the_bay_manager_offers_no_unassign_for_it_before_prep(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch)
        _p3_tick(m, 100, 111)
        m.printer.afc.prep_done = False
        m._ready_at = m.printer.reactor.now
        cmd = self._bays(m)
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Bambu_AMS_1 [AMS]: AAAA (live)",
             "Bambu_AMS_2 [AMS]: BBBB (live)",
             "Bambu_AMS_3 [AMS]: CCCC (live)",
             "Bambu_AMS_4 [AMS]: DDDD -- PREP has not run yet",
             "Bambu_AMS_HT_1 [HT]: HHHH (live)",
             "Bambu_AMS_HT_2 [HT]: free",
             "Waiting: EEEE [AMS] -- no free bay"],
            ["Replace for EEEE|AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE|primary",
             "Unassign Bambu_AMS_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=AAAA FORCE=1|warning",
             "Unassign Bambu_AMS_2|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=BBBB FORCE=1|warning",
             "Unassign Bambu_AMS_3|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=CCCC FORCE=1|warning",
             "Unassign Bambu_AMS_HT_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=HHHH FORCE=1|warning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Bambu_AMS_1 [AMS]: "
            "AAAA (live)\n  Bambu_AMS_2 [AMS]: BBBB (live)\n  Bambu_AMS_3 "
            "[AMS]: CCCC (live)\n  Bambu_AMS_4 [AMS]: DDDD -- PREP has not "
            "run yet\n  Bambu_AMS_HT_1 [HT]: HHHH (live)\n  Bambu_AMS_HT_2 "
            "[HT]: free\n  Waiting: EEEE [AMS] -- no free bay"))]
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: unit claims wait for PREP")]

    def test_a_bay_with_a_lane_in_the_toolhead_has_no_unassign_button(
            self, tmp_path, monkeypatch):
        m = _p3_claimed(tmp_path, monkeypatch, online=True)
        _p3_load(m, "lane25")
        cmd = self._bays(m)
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Bambu_AMS_1 [AMS]: AAAA (live) -- lane25 in the toolhead, "
             "unload it before unassigning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Bambu_AMS_1 [AMS]: "
            "AAAA (live) -- lane25 in the toolhead, unload it before "
            "unassigning"))]
        assert _p3_bay(m, "Bambu_AMS_1")["bound"] == A
        assert _p3_log(m) == []

    def test_the_button_is_back_once_it_is_unloaded(self, tmp_path,
                                                    monkeypatch):
        m = _p3_claimed(tmp_path, monkeypatch, online=True)
        _p3_load(m, "lane25")
        m.printer.afc.function.unset_lane_loaded()        # UNSET_LANE_LOADED
        cmd = self._bays(m)
        assert _p3_console(m) == _p3_popup(
            "Bambu AMS Units",
            ["Bambu_AMS_1 [AMS]: AAAA (live)"],
            ["Unassign Bambu_AMS_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=AAAA FORCE=1|warning"])
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: bay manager --\n  Bambu_AMS_1 [AMS]: "
            "AAAA (live)"))]
        assert _p3_log(m) == []


class TestAfcBridgeBoxCmdAFCBridgeboxReplace:
    """REPLACE forgets an offline unit and assigns its bay to a waiting one."""

    REPLACING = ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                                 "Bambu_AMS_4 with EEEE.")
    FORGOT = ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                              "36-39 and the name Bambu_AMS_4 freed for "
                              "reuse -- slot freed to the pool LIVE; the "
                              "next same-family unit claims it with no "
                              "reboot.")
    ASSIGNED = ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                                "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                                "claimed LIVE, no restart.")
    PICKER = ("respond_info",
              "AFC_BridgeBox chain1: opened the replace picker for EEEE.")
    #: What EEEE's live claim onto Bambu_AMS_4 logs.
    CLAIM_E = [
        ("debug", "AFC bambu Bambu_AMS_4: chain index not resolved yet "
                  "(UID EEEE); holding this unit's registrations until "
                  "the chain map arrives"),
        ("info", "AFC bambu Bambu_AMS_4: claimed UID EEEE as boxed and "
                 "brought online live (ams_index=0)."),
        ("info", "AFC_BridgeBox chain1: CLAIMED EEEE as boxed onto "
                 "Bambu_AMS_4 (4 lanes) -- live, no restart.")]

    @staticmethod
    def _ready(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
               until: int = 111, **options: Any) -> afcBridgeBox:
        """
        :param until: the last watch tick, from 100 on
        :return afcBridgeBox: the stuck chain, D offline since 100 and E
            waiting for a bay; the console and log start empty
        """
        m = _p3_stuck(tmp_path, monkeypatch, **options)
        _p3_tick(m, 100, until)
        _p3_quiet(m)
        return m

    @staticmethod
    def _replaced(master: afcBridgeBox, **params: Any) -> FakeGcmd:
        """
        :return FakeGcmd: a REPLACE run on a quiet console and log
        """
        _p3_quiet(master)
        cmd = _p3_cmd(**params)
        master.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        return cmd

    def test_it_forgets_the_old_unit_and_claims_the_new_one_live(
            self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        cmd = self._replaced(m, UID=E, OLD=D)
        bay = _p3_bay(m, "Bambu_AMS_4")
        assert (bay["uid"], bay["bound"]) == (E, E)
        assert m._state_get(SEC, "roster") == (
            "boxed:AAAA, boxed:BBBB, boxed:CCCC, ht:HHHH, boxed:EEEE")
        assert m._state_get(SEC, "name_map") == (
            "AAAA:Bambu_AMS_1, BBBB:Bambu_AMS_2, CCCC:Bambu_AMS_3, "
            "EEEE:Bambu_AMS_4, HHHH:Bambu_AMS_HT_1")
        assert m._state_get(SEC, "lane_map") == (
            "AAAA:24:4, BBBB:28:4, CCCC:32:4, EEEE:36:4, HHHH:40:1")
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                             "Bambu_AMS_4 with EEEE."),
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse -- slot freed to the pool LIVE; the "
                             "next same-family unit claims it with no "
                             "reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "claimed LIVE, no restart.")]
        assert _p3_log(m) == [
            ("debug", "AFC bambu Bambu_AMS_4: chain index not resolved yet "
                      "(UID EEEE); holding this unit's registrations until "
                      "the chain map arrives"),
            ("info", "AFC bambu Bambu_AMS_4: claimed UID EEEE as boxed and "
                     "brought online live (ams_index=0)."),
            ("info", "AFC_BridgeBox chain1: CLAIMED EEEE as boxed onto "
                     "Bambu_AMS_4 (4 lanes) -- live, no restart.")]
        assert _p3_console(m) == [END, END]
        assert m.get_status()["waiting_for_bay"] == []
        # The restart keeps E there, and nothing moves.
        m2 = _p3_chain(tmp_path, monkeypatch, roster="", pool_ams=4,
                       pool_ht=2)
        assert _p3_unit_uids(m2)["Bambu_AMS_4"] == E
        assert m2._layout_notes == []

    def test_old_may_name_the_bay(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        cmd = self._replaced(m, UID=E, OLD="Bambu_AMS_4")
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                             "Bambu_AMS_4 with EEEE."),
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse -- slot freed to the pool LIVE; the "
                             "next same-family unit claims it with no "
                             "reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "claimed LIVE, no restart.")]
        assert _p3_log(m) == self.CLAIM_E
        assert _p3_console(m) == [END, END]

    def test_without_uid_it_takes_the_one_waiting_unit(self, tmp_path,
                                                       monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        cmd = self._replaced(m, OLD=D)
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                             "Bambu_AMS_4 with EEEE."),
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse -- slot freed to the pool LIVE; the "
                             "next same-family unit claims it with no "
                             "reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "claimed LIVE, no restart.")]
        assert _p3_log(m) == self.CLAIM_E
        assert _p3_console(m) == [END, END]

    def test_without_uid_and_nothing_waiting_it_refuses(self, tmp_path,
                                                        monkeypatch):
        m = self._ready(tmp_path, monkeypatch, online=(A, B, C, H))
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: give UID=<new uid> -- no unit is "
            "waiting for a bay")

    def test_an_offline_new_unit_is_pinned_for_its_return(self, tmp_path,
                                                          monkeypatch):
        m = self._ready(tmp_path, monkeypatch, online=(A, B, C, H),
                        recorded=FOUR_p3 + f", boxed:{E}")
        cmd = self._replaced(m, UID=E, OLD=D)
        bay = _p3_bay(m, "Bambu_AMS_4")
        assert (bay["uid"], bay["bound"]) == (E, None)
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                             "Bambu_AMS_4 with EEEE."),
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse -- slot freed to the pool LIVE; the "
                             "next same-family unit claims it with no "
                             "reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "pinned; it claims this bay when next online.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END, END]
        _p3_online(m, (A, B, C, E, H))
        _p3_tick(m, 112, 112)
        assert bay["bound"] == E

    def test_an_unrecorded_spare_occupant_is_replaced(self, tmp_path,
                                                      monkeypatch):
        # G claimed the spare live and was pulled before it was recorded:
        # nothing is saved for it, but it holds the bay.
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                      recorded=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _p3_tick(m, 100, 104)
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == G
        _p3_online(m, (A, B, C, E, H))            # G pulled, E plugged in
        _p3_tick(m, 105, 116)
        assert m._state_get(SEC, "roster") == (
            "boxed:AAAA, boxed:BBBB, boxed:CCCC, ht:HHHH")
        cmd = self._replaced(m, UID=E, OLD=G)
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing GGGG on "
                             "Bambu_AMS_4 with EEEE."),
            ("respond_info", "AFC_BridgeBox chain1: forgot GGGG -- saved "
                             "lane records erased -- slot freed to the "
                             "pool LIVE; the next same-family unit claims "
                             "it with no reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "claimed LIVE, no restart.")]
        assert _p3_log(m) == [(
            "info", "AFC_BridgeBox chain1: released Bambu_AMS_4 (UID GGGG, "
                    "AFC_BRIDGEBOX_FORGET); lanes dropped live")] + self.CLAIM_E
        assert _p3_console(m) == [END, END]

    def test_two_waiting_units_and_one_bay(self, tmp_path, monkeypatch):
        # E and G both wait; E's offer shows first and E takes D's bay. G's
        # offer, next in the queue, has no bay left: it is dropped unshown,
        # and a button still naming D is refused.
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, E, G, H))
        _p3_tick(m, 100, 115)
        assert _p3_console(m) == _p3_popup(
            "No free bay for new AMS",
            ["UID EEEE has no free bay: every AMS bay is taken.",
             "Replace a unit that is offline: the new one takes its bay, "
             "lanes and T# now, and the old one is forgotten (its learned "
             "values and saved lane records, spools included, are erased).",
             "Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 15s",
             "Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE opens this again."],
            ["Replace Bambu_AMS_4|AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE "
             "OLD=DDDD|error"])
        assert m._popup_queue == [("replace", G)]
        cmd = self._replaced(m, UID=E, OLD=D)
        assert cmd.messages == [self.REPLACING, self.FORGOT, self.ASSIGNED]
        assert _p3_log(m) == self.CLAIM_E
        assert _p3_console(m) == [END, END]
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E
        _p3_quiet(m)
        _p3_tick(m, 116, 140)
        assert _p3_console(m) == []
        assert m._popup_queue == []
        assert G not in m._replace_offered and G in m._no_bay
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=G, OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: no pool bay is named or held for DDDD "
            "(bays: Bambu_AMS_1, Bambu_AMS_2, Bambu_AMS_3, Bambu_AMS_4, "
            "Bambu_AMS_HT_1, Bambu_AMS_HT_2)")

    def test_the_handlers_get_their_own_parameters(self, tmp_path,
                                                   monkeypatch):
        # FORGET is handed the bay's uid, not OLD; ASSIGN the bay's name.
        m = self._ready(tmp_path, monkeypatch)
        seen: List[tuple] = []
        forget, assign = m.cmd_AFC_BRIDGEBOX_FORGET, m.cmd_AFC_BRIDGEBOX_ASSIGN
        monkeypatch.setattr(m, "cmd_AFC_BRIDGEBOX_FORGET", lambda g: (
            seen.append(("forget", g.get("UID", None), g.get("OLD", None),
                         g.get_int("FORCE", 0))), forget(g)))
        monkeypatch.setattr(m, "cmd_AFC_BRIDGEBOX_ASSIGN", lambda g: (
            seen.append(("assign", g.get("UID", None), g.get("NAME", None),
                         g.get_int("FORCE", 0))), assign(g)))
        cmd = self._replaced(m, UID=E, OLD="Bambu_AMS_4")
        assert seen == [("forget", D, None, 0),
                        ("assign", E, "Bambu_AMS_4", 0)]
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                             "Bambu_AMS_4 with EEEE."),
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse -- slot freed to the pool LIVE; the "
                             "next same-family unit claims it with no "
                             "reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "claimed LIVE, no restart.")]
        assert _p3_log(m) == self.CLAIM_E
        assert _p3_console(m) == [END, END]
        bay = _p3_bay(m, "Bambu_AMS_4")
        assert (bay["uid"], bay["bound"]) == (E, E)

    def test_a_failed_assign_says_what_is_left(self, tmp_path, monkeypatch):
        # Every refusal ASSIGN could make is checked before FORGET runs, so
        # only a stand-in handler fails here.
        m = self._ready(tmp_path, monkeypatch)

        def _refuse(gcmd: Any) -> None:
            """ASSIGN refusing."""
            raise gcmd.error("AFC_BRIDGEBOX_ASSIGN: no")

        monkeypatch.setattr(m, "cmd_AFC_BRIDGEBOX_ASSIGN", _refuse)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=E, OLD=D)
        with pytest.raises(Exception) as err:
            m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        assert cmd.messages == [self.REPLACING, self.FORGOT]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        assert str(err.value) == (
            "AFC_BRIDGEBOX_REPLACE: DDDD is forgotten and bay "
            "'Bambu_AMS_4' is free, but EEEE is not on it "
            "(AFC_BRIDGEBOX_ASSIGN: no). While the bay is free, EEEE "
            "claims it once it is on the chain.")
        assert _p3_bay(m, "Bambu_AMS_4")["uid"] is None
        _p3_tick(m, 112, 112)                      # E claims the free bay
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E

    def test_a_failed_claim_says_the_bay_is_pinned(self, tmp_path,
                                                   monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        unit = m.printer.lookup_object("AFC_BambuAMS Bambu_AMS_4")

        def _fail(uid: str, model: str) -> bool:
            """The unit's claim failing."""
            raise RuntimeError("bus busy")

        monkeypatch.setattr(unit, "claim", _fail)
        _p3_quiet(m)
        cmd = _p3_cmd(UID=E, OLD=D)
        with pytest.raises(Exception) as err:
            m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        assert cmd.messages == [self.REPLACING, self.FORGOT]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END]
        assert str(err.value) == (
            "AFC_BRIDGEBOX_REPLACE: DDDD is forgotten and bay "
            "'Bambu_AMS_4' is pinned to EEEE, but EEEE is not claimed onto "
            "it (bus busy). The chain watch claims it there while EEEE is "
            "on the chain.")
        assert _p3_bay(m, "Bambu_AMS_4")["uid"] == E
        monkeypatch.delattr(unit, "claim")
        _p3_quiet(m)
        _p3_tick(m, 112, 112)                      # the watch claims it
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E
        assert _p3_log(m) == [
            ("debug", "AFC bambu Bambu_AMS_4: chain index not resolved yet "
                      "(UID EEEE); holding this unit's registrations until "
                      "the chain map arrives"),
            ("info", "AFC bambu Bambu_AMS_4: claimed UID EEEE as boxed and "
                     "brought online live (ams_index=0)."),
            ("info", "AFC_BridgeBox chain1: CLAIMED EEEE as boxed onto "
                     "Bambu_AMS_4 (4 lanes) -- live, no restart.")]

    def test_an_online_old_unit_even_with_force(self, tmp_path,
                                                monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        for force in (0, 1):
            refusal = _p3_refused(m, tmp_path, "REPLACE",
                                  _p3_cmd(UID=E, OLD=A, FORCE=force))
            assert refusal == (
                "AFC_BRIDGEBOX_REPLACE: AAAA on bay 'Bambu_AMS_1' is "
                "online, and only a unit that is offline is replaced -- "
                "unplug it first, or AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
                "UID=AAAA forgets it and frees the bay now")

    def test_an_old_unit_whose_online_flag_cannot_be_read_even_with_force(
            self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        bridge = _p3_bridge(m)
        bridge._serial = None                       # the link is down
        for old in (D, A):                          # A is on the chain
            refusal = _p3_refused(m, tmp_path, "REPLACE",
                                  _p3_cmd(UID=E, OLD=old, FORCE=1))
            assert refusal == (
                f"AFC_BRIDGEBOX_REPLACE: cannot tell whether {old} is "
                f"offline: the bridge link is down -- run this again once it "
                f"reconnects")
        bridge._serial = object()
        status = bridge.status
        bridge.status = None                        # no status read yet
        refusal = _p3_refused(m, tmp_path, "REPLACE",
                              _p3_cmd(UID=E, OLD=D, FORCE=1))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: cannot tell whether DDDD is offline: "
            "the bridge has not reported the chain yet -- run this again "
            "once it has")
        bridge.status = status
        cmd = self._replaced(m, UID=E, OLD=D, FORCE=1)
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                             "Bambu_AMS_4 with EEEE."),
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse -- slot freed to the pool LIVE; the "
                             "next same-family unit claims it with no "
                             "reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "claimed LIVE, no restart.")]
        assert _p3_log(m) == self.CLAIM_E
        assert _p3_console(m) == [END, END]

    def test_a_new_unit_on_a_bay_even_with_force(self, tmp_path,
                                                 monkeypatch):
        # A mistyped uid would move a unit that has a bay onto the offline
        # unit's, and forget the offline unit for good.
        m = self._ready(tmp_path, monkeypatch)
        for force in (0, 1):
            refusal = _p3_refused(m, tmp_path, "REPLACE",
                                  _p3_cmd(UID=A, OLD=D, FORCE=force))
            assert refusal == (
                "AFC_BRIDGEBOX_REPLACE: AAAA is on bay 'Bambu_AMS_1', and "
                "only a unit with no bay takes over another's -- "
                "AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=AAAA NAME=<bay "
                "name> moves a unit between bays")
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=A))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: AAAA is on bay 'Bambu_AMS_1', and only "
            "a unit with no bay takes over another's -- "
            "AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=AAAA NAME=<bay name> "
            "moves a unit between bays")

    def test_an_offline_unit_named_for_its_own_bay(self, tmp_path,
                                                   monkeypatch):
        # D is offline and pinned there: it is not waiting for a bay, and a
        # FORGET would erase its learned values.
        m = self._ready(tmp_path, monkeypatch)
        refusal = _p3_refused(m, tmp_path, "REPLACE",
                              _p3_cmd(UID=D, OLD="Bambu_AMS_4", FORCE=1))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: DDDD is on bay 'Bambu_AMS_4', and only "
            "a unit with no bay takes over another's -- "
            "AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=DDDD NAME=<bay name> "
            "moves a unit between bays")

    def test_a_bay_saved_for_another_recorded_unit_even_with_force(
            self, tmp_path, monkeypatch):
        # ASSIGN would refuse the bay once FORGET had run, so it is refused
        # before.
        m = self._ready(tmp_path, monkeypatch)
        m._state_set({SEC: {"roster": FOUR_p3 + f", boxed:{G}"}})
        m._name_map[G] = "Bambu_AMS_4"
        refusal = _p3_refused(m, tmp_path, "REPLACE",
                              _p3_cmd(UID=E, OLD=D, FORCE=1))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: bay 'Bambu_AMS_4' is saved for GGGG "
            "too -- AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=GGGG or "
            "AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=GGGG first")

    def test_before_prep_without_force(self, tmp_path, monkeypatch):
        # PREP restores AFC's loaded-lane record; before it, a lane the
        # record names is not known.
        m = self._ready(tmp_path, monkeypatch)
        m.printer.afc.prep_done = False
        m._ready_at = m.printer.reactor.now
        wait = [("debug", "AFC_BridgeBox chain1: unit claims wait for PREP")]
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=D),
                              wait)
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: PREP has not run yet, so which lane "
            "AFC records as loaded to the toolhead is not known -- run "
            "this again once it has")
        cmd = self._replaced(m, UID=E, OLD=D, FORCE=1)
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                             "Bambu_AMS_4 with EEEE."),
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse -- slot freed to the pool LIVE; the "
                             "next same-family unit claims it with no "
                             "reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "pinned; it claims this bay once PREP "
                             "finishes.")]
        assert _p3_log(m) == []
        assert _p3_console(m) == [END, END]
        m.printer.afc.prep_done = True
        _p3_tick(m, 112, 112)
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E

    def test_a_bay_of_the_other_family(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch, online=(A, B, C, E))
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=H))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: EEEE is an AMS unit but bay "
            "'Bambu_AMS_HT_1' is an HT bay (their lane counts differ)")

    def test_a_free_bay_points_at_assign(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_p3_cmd(UID=D))   # frees Bambu_AMS_4
        refusal = _p3_refused(m, tmp_path, "REPLACE",
                              _p3_cmd(UID=E, OLD="Bambu_AMS_4"))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: bay 'Bambu_AMS_4' is free -- "
            "AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=EEEE NAME=Bambu_AMS_4 "
            "puts EEEE on it")

    def test_a_free_bay_of_the_other_family_is_refused_for_its_family(
            self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        assert _p3_bay(m, "Bambu_AMS_HT_2")["uid"] is None
        refusal = _p3_refused(m, tmp_path, "REPLACE",
                              _p3_cmd(UID=E, OLD="Bambu_AMS_HT_2"))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: EEEE is an AMS unit but bay "
            "'Bambu_AMS_HT_2' is an HT bay (their lane counts differ)")

    def test_an_unknown_bay_or_unit(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        refusal = _p3_refused(m, tmp_path, "REPLACE",
                              _p3_cmd(UID=E, OLD="Nope"))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: no pool bay is named or held for Nope "
            "(bays: Bambu_AMS_1, Bambu_AMS_2, Bambu_AMS_3, Bambu_AMS_4, "
            "Bambu_AMS_HT_1, Bambu_AMS_HT_2)")
        refusal = _p3_refused(m, tmp_path, "REPLACE",
                              _p3_cmd(UID="FFFF", OLD=D))
        assert refusal == "AFC_BRIDGEBOX_REPLACE: FFFF is not on chain chain1"

    def test_a_roster_option(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch, roster=FOUR_p3)
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: roster: is set, so it decides which "
            "units get a bay. To swap an offline unit for EEEE, change its "
            "entry to boxed:EEEE in roster:, run AFC_BRIDGEBOX_FORGET "
            "CHAIN=chain1 UID=<old uid> (its bay frees now and the new "
            "unit claims it live), then RESTART.")

    def test_a_chain_without_a_pool(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch, pool_ams=0, pool_ht=0)
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: chain chain1 has no pool bays, so no "
            "unit is put on a bay live -- AFC_BRIDGEBOX_FORGET "
            "CHAIN=chain1 UID=<old uid> and RESTART; the new unit, once "
            "recorded, takes the bay that frees")

    def test_a_print_without_force(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        m.printer.set_print_state("printing")
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: a print is active -- finish it, or "
            "FORCE=1 to replace DDDD anyway")
        cmd = self._replaced(m, UID=E, OLD=D, FORCE=1)
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E
        assert cmd.messages == [self.REPLACING, self.FORGOT, self.ASSIGNED]
        assert _p3_log(m) == self.CLAIM_E
        assert _p3_console(m) == [END, END]

    def test_inside_release_grace_without_force(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch, until=105)
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: DDDD has been offline 5s, under "
            "release_grace (10s) -- wait, or FORCE=1 to replace it now")
        cmd = self._replaced(m, UID=E, OLD=D, FORCE=1)
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E
        assert cmd.messages == [self.REPLACING, self.FORGOT, self.ASSIGNED]
        assert _p3_log(m) == self.CLAIM_E
        assert _p3_console(m) == [END, END]

    def test_no_counted_absence_without_force(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        bridge = _p3_bridge(m)
        bridge._serial = None
        _p3_tick(m, 112, 112)
        bridge._serial = object()
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: no absence is counted for DDDD -- it "
            "counts only while the bridge link is up and a unit on the "
            "chain is online. FORCE=1 replaces it anyway")

    def test_a_boot_restored_loaded_lane_and_force_clears_it(
            self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        ext = m.printer.afc.tools["extruder"]
        ext.lane_loaded = "lane36"
        # The lane is not registered while the bay is unclaimed, so no
        # unload or UNSET_LANE_LOADED reaches it.
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: AFC records lane36 on Bambu_AMS_4 as "
            "loaded to the toolhead, from DDDD. While Bambu_AMS_4 is "
            "unclaimed its lanes are not registered, so no unload or "
            "UNSET_LANE_LOADED reaches them -- plug DDDD back in and "
            "unload it, or take the filament out by hand and FORCE=1 "
            "clears the record and replaces DDDD")
        drain_var_writes(m.printer)
        cmd = self._replaced(m, UID=E, OLD=D, FORCE=1)
        assert ext.lane_loaded is None
        assert drain_var_writes(m.printer)[0]["system"]["extruders"] == {
            "extruder": {"lane_loaded": None}}
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing DDDD on "
                             "Bambu_AMS_4 with EEEE; cleared lane36 from "
                             "the toolhead."),
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse -- slot freed to the pool LIVE; the "
                             "next same-family unit claims it with no "
                             "reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned EEEE to bay "
                             "'Bambu_AMS_4' (lane36-lane39, T36-T39) -- "
                             "claimed LIVE, no restart.")]
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == E
        assert _p3_log(m) == self.CLAIM_E
        assert _p3_console(m) == [END, END]

    def test_a_loaded_lane_of_a_bound_unit_and_force_clears_it(
            self, tmp_path, monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, D, H))
        _p3_tick(m, 100, 100)
        _p3_online(m, (A, B, C, E, H))             # D for E
        _p3_tick(m, 101, 112)
        bay = _p3_bay(m, "Bambu_AMS_4")
        assert bay["bound"] == D                    # auto_drop off
        lane = _p3_lane(m, "lane37")
        lane.tool_loaded = True
        refusal = _p3_refused(m, tmp_path, "REPLACE", _p3_cmd(UID=E, OLD=D))
        assert refusal == (
            "AFC_BRIDGEBOX_REPLACE: AFC records lane37 on Bambu_AMS_4 as "
            "loaded to the toolhead -- unload it first (UNSET_LANE_LOADED "
            "if the filament is already out), or FORCE=1 to clear it from "
            "the toolhead and replace anyway")
        cmd = self._replaced(m, UID=E, OLD=D, FORCE=1)
        assert lane.tool_loaded is False
        assert bay["bound"] == E
        assert cmd.messages == [
            self.REPLACING,
            ("respond_info", "AFC_BridgeBox chain1: forgot DDDD -- lanes "
                             "36-39 and the name Bambu_AMS_4 freed for "
                             "reuse, saved lane records erased -- slot freed "
                             "to the pool LIVE; the next same-family unit "
                             "claims it with no reboot."),
            self.ASSIGNED]
        assert _p3_log(m) == [(
            "info", "AFC_BridgeBox chain1: released Bambu_AMS_4 (UID DDDD, "
                    "AFC_BRIDGEBOX_FORGET); lanes dropped live; cleared "
                    "lane37 from the toolhead")] + self.CLAIM_E
        assert _p3_console(m) == [END, END]

    def test_replace_without_old_opens_the_picker(self, tmp_path,
                                                  monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        cmd = self._replaced(m, UID=E)
        assert _p3_console(m) == _p3_popup(
            "No free bay for new AMS",
            ["UID EEEE has no free bay: every AMS bay is taken.",
             "Replace a unit that is offline: the new one takes its bay, "
             "lanes and T# now, and the old one is forgotten (its learned "
             "values and saved lane records, spools included, are erased).",
             "Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 11s",
             "Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE opens this again."],
            ["Replace Bambu_AMS_4|AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE "
             "OLD=DDDD|error"])
        assert cmd.messages == [(
            "respond_info",
            "AFC_BridgeBox chain1: opened the replace picker for EEEE.")]
        assert _p3_log(m) == []

    def test_a_loaded_bay_is_listed_without_a_button(self, tmp_path,
                                                     monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        m.printer.afc.tools["extruder"].lane_loaded = "lane36"
        cmd = self._replaced(m, UID=E)
        assert _p3_console(m) == _p3_popup(
            "No free bay for new AMS",
            ["UID EEEE has no free bay: every AMS bay is taken.",
             "Replace a unit that is offline: the new one takes its bay, "
             "lanes and T# now, and the old one is forgotten (its learned "
             "values and saved lane records, spools included, are erased).",
             "Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 11s -- AFC "
             "records lane36 as loaded to the toolhead: plug DDDD back in and "
             "unload it, or take the filament out by hand and "
             "AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE OLD=DDDD FORCE=1 "
             "clears the record",
             "Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE opens this again."])
        assert cmd.messages == [self.PICKER]
        assert _p3_log(m) == []

    def test_a_claimed_loaded_bay_is_listed_with_its_unload(self, tmp_path,
                                                            monkeypatch):
        m = _p3_stuck(tmp_path, monkeypatch, online=(A, B, C, D, H))
        _p3_tick(m, 100, 100)
        _p3_online(m, (A, B, C, E, H))             # D for E
        _p3_tick(m, 101, 112)
        assert _p3_bay(m, "Bambu_AMS_4")["bound"] == D   # auto_drop off
        _p3_lane(m, "lane37").tool_loaded = True
        cmd = self._replaced(m, UID=E)
        assert _p3_console(m) == _p3_popup(
            "No free bay for new AMS",
            ["UID EEEE has no free bay: every AMS bay is taken.",
             "Replace a unit that is offline: the new one takes its bay, "
             "lanes and T# now, and the old one is forgotten (its learned "
             "values and saved lane records, spools included, are erased).",
             "Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 11s -- AFC "
             "records lane37 as loaded to the toolhead: unload it "
             "(UNSET_LANE_LOADED if the filament is already out), or "
             "AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE OLD=DDDD FORCE=1 "
             "clears it",
             "Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE opens this again."])
        assert cmd.messages == [self.PICKER]
        assert _p3_log(m) == []

    def test_before_release_grace_it_says_why(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch, until=103)
        cmd = self._replaced(m, UID=E)
        assert _p3_console(m) == []
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: no AMS bay is held for a unit offline "
            "10s or longer: Bambu_AMS_4 (DDDD, offline 3s) -- "
            "AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE OLD=Bambu_AMS_4 "
            "FORCE=1 skips the wait."))]
        assert _p3_log(m) == []

    def test_a_unit_on_another_chain_is_not_on_this_one(self, tmp_path,
                                                        monkeypatch):
        # An earlier start of chain1 computed its lane base and recorded it.
        first = make_bridgebox(tmp_path, printer=make_printer(),
                               roster=f"ht:{A}", lane_base=0, pool_ams=1,
                               pool_ht=2)
        assert first._state_get(SEC, "lane_base") == "24"
        first._state_set({SEC: {"roster": f"ht:{A}"},
                          "AFC_BridgeBox chain2": {"roster": f"ht:{B}"}})
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True)
        chain = dict(roster="", lane_base=0, pool_ams=1, pool_ht=2)
        m1 = make_bridgebox(tmp_path, printer=printer, **chain)
        m2 = make_bridgebox(tmp_path, "chain2", printer=printer,
                            unit_prefix="Bambu_AMS_B", **chain)
        printer.connect()
        assert (m1.lane_base, m2._family_of_uid(B)) == (24, "ht")
        refusal = _p3_refused(m1, tmp_path, "REPLACE",
                              _p3_cmd(UID=B, OLD=A))
        assert refusal == "AFC_BRIDGEBOX_REPLACE: BBBB is not on chain chain1"

    @pytest.mark.parametrize("saved", ["released-this-session",
                                       "at-last-boot"])
    def test_the_new_unit_gets_none_of_the_old_units_data(
            self, tmp_path, monkeypatch, saved):
        # H's bay Hot (lane32) held its spool 136 on T45, and H had learned
        # its bowden length. J, an HT with no bay, replaces H: it gets the
        # home T32, AFC's defaults for its untagged spool and no learned
        # value.
        J = "JJJJ"
        rec = _p3_rec("T45", spool_id=136, material="PLA", color="#0086D6",
                      weight=750.0)
        boot = saved == "at-last-boot"
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}, ht:{H}",
                      pool_ams=2, pool_ht=1, ams_names="Alpha, Bravo",
                      ht_names="Hot", online=(A, H), uids=(A, H, J),
                      htmask=0b110, var={"Hot": {"lane32": rec}} if boot
                      else None, owners=f"{H}:Hot" if boot else None,
                      ready=True)
        afc = m.printer.afc
        gcode = m.printer.gcode
        lane = _p3_lane(m, "lane32")
        if not boot:
            assert m._claim_pool_unit(H, "ht") is not None
            afc.tool_cmds.pop("T32")
            gcode.register_command("T32", None)
            lane.map, lane.current_map = ["T45"], "T45"
            afc.tool_cmds["T45"] = "lane32"
            gcode.register_command("T45", afc.CHANGE_TOOL)
            lane.spool_id, lane.material = 136, "PLA"
            lane.color, lane.weight = "#0086D6", 750.0
            m._release_pool_unit(H)
        assert m._held["Hot"]["uid"] == H
        assert m._bay_of_uid(H)["name"] == "Hot"
        m._state_set({m._learned_section(H): {"afc_bowden_length": "3632.0"}})
        # H's measured path, as the unit holds it.
        unit = m.printer.lookup_object("AFC_BambuAMS Hot")
        unit.afc_bowden_length = unit.afc_unload_bowden_length = 3632.0
        afc.spoolman = "http://spoolman:7912"
        _p3_online(m, (A, J))
        assert m._claim_pool_unit(J, "ht") is None     # every HT bay taken
        m._missing_since[H] = 0.0
        drain_var_writes(m.printer)
        cmd = self._replaced(m, UID=J, OLD="Hot")
        writes = drain_var_writes(m.printer)
        assert writes[0]["Hot"] == {}
        assert (unit.afc_bowden_length, unit.afc_unload_bowden_length) == (
            3000.0, 3000.0)
        assert _p3_console(m) == [END, END]
        assert m._bay_of_uid(J)["bound"] == J
        assert (lane.map, lane.current_map) == (["T32"], "T32")
        assert afc.tool_cmds == {"T32": "lane32"}
        assert (lane.spool_id, lane.material, lane.color, lane.weight) == (
            None, None, "", 0.0)
        assert m._learned_for(H) == {} and m._learned_for(J) == {}
        assert m._owners()["Hot"] == J and "Hot" not in m._held
        assert cmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: replacing HHHH on Hot "
                             "with JJJJ."),
            ("respond_info", "AFC_BridgeBox chain1: forgot HHHH -- lane32 "
                             "and the name Hot freed for reuse, learned "
                             "values and saved lane records erased -- slot "
                             "freed to the pool LIVE; the next same-family "
                             "unit claims it with no reboot."),
            ("respond_info", "AFC_BridgeBox chain1: assigned JJJJ to bay "
                             "'Hot' (lane32, T32) -- claimed LIVE, no "
                             "restart.")]
        assert _p3_log(m) == [
            ("debug", "AFC bambu Hot: chain index not resolved yet (UID "
                      "JJJJ); holding this unit's registrations until the "
                      "chain map arrives"),
            ("info", "AFC bambu Hot: claimed UID JJJJ as ht and brought "
                     "online live (ams_index=0)."),
            ("info", "AFC_BridgeBox chain1: CLAIMED JJJJ as ht onto Hot (1 "
                     "lanes) -- live, no restart.")]
        # J's bay reports an untagged spool and scan priming runs.
        _p3_quiet(m)
        afc.spool.calls.clear()
        unit._slots[0] = slot_info(0, present=True)
        unit._prev_present[0] = True
        unit._presence_ok = True
        unit._prime_scan_baseline()
        # No spool is bound from H's record: J's lane gets AFC's defaults.
        assert afc.spool.calls == []
        assert (lane.spool_id, lane.material, lane.color, lane.weight) == (
            None, "PLA", "", 1000)
        assert unit._held_lanes == {}
        assert drain_var_writes(m.printer) == []
        assert _p3_console(m) == []
        box = ("<span class=success--text>R  +--------+\nE  |BambuAMS|\n"
               "A  +--------+\nD  |   O    |\nY  +--------+</span>\n"
               "   Hot\n")
        assert _p3_log(m) == [
            ("info", "BambuAMS Hot Prepping lanes"),
            ("info", "lane32 tool cmd: T32 <span class=success--text>LOCKED "
                     "AND LOADED</span>"),
            ("raw", box),
            ("info", "AFC bambu Hot: no tag on record for slot 0; applied "
                     "lane defaults to lane32 -- nothing has read this bay "
                     "yet. No scan was attempted, so this says nothing about "
                     "the reader or the spool; reseat it, or run "
                     "AFC_BAMBU_SCAN LANE=lane32"),
            # Claimed live: the bay goes to the insert path, once.
            ("info", "AFC bambu Hot: slot 0 came up with a spool and no record "
                     "of it -- scanning it as a fresh insert")]

    def test_during_a_print_it_offers_no_force(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch, until=103)
        m.printer.set_print_state("printing")
        cmd = self._replaced(m, UID=E)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: no AMS bay is held for a unit offline "
            "10s or longer: Bambu_AMS_4 (DDDD, offline 3s) -- a print is "
            "active: run this again once it ends."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == []

    def test_with_a_loaded_lane_it_says_force_clears_it(self, tmp_path,
                                                        monkeypatch):
        m = self._ready(tmp_path, monkeypatch, until=103)
        m.printer.afc.tools["extruder"].lane_loaded = "lane36"
        cmd = self._replaced(m, UID=E)
        assert cmd.messages == [("respond_info", (
            "AFC_BridgeBox chain1: no AMS bay is held for a unit offline "
            "10s or longer: Bambu_AMS_4 (DDDD, offline 3s) -- AFC records "
            "lane36 as loaded to the toolhead: plug DDDD back in and "
            "unload it, or take the filament out by hand and "
            "AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE OLD=Bambu_AMS_4 "
            "FORCE=1 clears the record and skips the wait."))]
        assert _p3_log(m) == []
        assert _p3_console(m) == []

    def test_the_picker_offers_no_force_during_a_print(self, tmp_path,
                                                       monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        m.printer.afc.tools["extruder"].lane_loaded = "lane36"
        m.printer.set_print_state("printing")
        cmd = self._replaced(m, UID=E)
        assert _p3_console(m) == _p3_popup(
            "No free bay for new AMS",
            ["UID EEEE has no free bay: every AMS bay is taken.",
             "Replace a unit that is offline: the new one takes its bay, "
             "lanes and T# now, and the old one is forgotten (its learned "
             "values and saved lane records, spools included, are erased).",
             "Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 11s -- AFC "
             "records lane36 as loaded to the toolhead, and a print is "
             "active: replace it once the print ends",
             "Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE opens this again."])
        assert cmd.messages == [self.PICKER]
        assert _p3_log(m) == []


class TestAfcBridgeBoxFoldAndSweep:
    """The fold: learned values, chain defaults and operator overrides."""

    #: What ConfigRewrite filed for the HT unit in auto_vars.
    AUTOV = ("[AFC_BambuAMS Bambu_AMS_HT_1]\n"
             "afc_bowden_length : 3632.0\n"
             "afc_unload_bowden_length : 3632.0\n")
    #: The default roster's HT unit.
    UID = "0123456789ABCDEF00003331"

    @staticmethod
    def _fold(master: afcBridgeBox) -> Dict[str, Dict[str, Any]]:
        """
        :param master: the chain master
        :return dict: section -> keys, as the fold builds them now
        """
        config = BambuConfig(SEC, master.printer)
        return dict(master._fold_and_sweep(
            config, master._roster_sections(master.units)))

    @staticmethod
    def _boot(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
              autov: Optional[str] = None,
              sections: Optional[Dict[str, Dict[str, str]]] = None,
              **options: Any) -> afcBridgeBox:
        """
        :param autov: what AFC_auto_vars.cfg holds before the start
        :param sections: override sections in the printer's config
        :return afcBridgeBox: a start of chain1 on the default HT roster
        """
        if autov is not None:
            (tmp_path / "AFC_auto_vars.cfg").write_text(autov)
        return _p3_chain(tmp_path, monkeypatch, sections=sections, **options)

    def test_a_learned_value_reaches_the_fabricated_section(self, tmp_path,
                                                            monkeypatch):
        # ConfigRewrite filed the unit's measured path in auto_vars, which
        # the fabricated unit's own parser never reads.
        m = self._boot(tmp_path, monkeypatch, autov=self.AUTOV)
        unit = m.printer.lookup_object("AFC_BambuAMS Bambu_AMS_HT_1")
        assert unit.afc_bowden_length == 3632.0
        folded = self._fold(m)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert folded["afc_bowden_length"] == "3632.0"
        assert folded["afc_unload_bowden_length"] == "3632.0"
        # A wrapper that shows no merged config folds it the same.
        bare = dict(m._fold_and_sweep(_P3BareConfig(SEC, m.printer),
                                      m._roster_sections(m.units)))
        assert bare["AFC_BambuAMS Bambu_AMS_HT_1"]["afc_bowden_length"] == (
            "3632.0")
        assert m._learned_notes == []
        assert _p3_log(m) == []

    def test_and_survives_in_the_state_file_for_the_next_boot(
            self, tmp_path, monkeypatch):
        self._boot(tmp_path, monkeypatch, autov=self.AUTOV)
        # Filed under the unit's uid, not its bay name, and only in the
        # managed block klippy never parses.
        assert (tmp_path / "AFC_BridgeBox.cfg").read_text().split("\n") == [
            "# AFC_BridgeBox: put your [AFC_BridgeBox <name>] section here "
            "(serial_port + extruder is enough).",
            "# The block below is maintained by the module.",
            "",
            "#~# --- AFC_BridgeBox managed state -- everything below is "
            "auto-written ---",
            "#~# [AFC_BridgeBox chain1]",
            f"#~# lane_map : {self.UID}:24:1",
            f"#~# name_map : {self.UID}:Bambu_AMS_HT_1",
            "#~# ",
            f"#~# [AFC_BridgeBox chain1 learned {self.UID}]",
            "#~# afc_bowden_length : 3632.0",
            "#~# afc_unload_bowden_length : 3632.0",
            "#~# ",
            ""]
        m2 = self._boot(tmp_path, monkeypatch)
        folded = self._fold(m2)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert folded["afc_bowden_length"] == "3632.0"

    def test_a_leftover_cannot_rewrite_identity(self, tmp_path, monkeypatch):
        # Folding is for calibrations, never wiring.
        evil = ("[AFC_BambuAMS Bambu_AMS_HT_1]\n"
                "serial_port : /dev/evil\n"
                "unit_uid : FFFFFFFFFFFFFFFFFFFFFFFF\n"
                "afc_bowden_length : 3632.0\n")
        m = self._boot(tmp_path, monkeypatch, autov=evil)
        folded = self._fold(m)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert (folded["serial_port"], folded["unit_uid"],
                folded["afc_bowden_length"]) == (
            "/dev/serial/by-id/usb-chain1-if00", self.UID, "3632.0")
        assert m._learned_notes == [
            (False, "auto_vars [AFC_BambuAMS Bambu_AMS_HT_1]: serial_port, "
                    "unit_uid not folded -- a unit takes only its bowden "
                    "lengths from there, each a positive number")]

    def test_a_second_master_is_never_read_as_an_override(self, tmp_path,
                                                          monkeypatch):
        # Another chain's master carries serial_port: its options must not
        # overlay this chain's units.
        m = self._boot(tmp_path, monkeypatch, sections={
            "AFC_BridgeBox chain2": {"serial_port": "/dev/other",
                                     "unit_prefix": "Bambu_AMS_B"}})
        folded = self._fold(m)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert "unit_prefix" not in folded
        assert folded["serial_port"] == "/dev/serial/by-id/usb-chain1-if00"
        assert m._unmatched_overrides == []

    def test_a_bare_name_overrides_that_units_section(self, tmp_path,
                                                      monkeypatch):
        m = self._boot(tmp_path, monkeypatch, sections={
            "AFC_BridgeBox Bambu_AMS_HT_1": {"measure_on_insert": "False"}})
        folded = self._fold(m)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert folded["measure_on_insert"] == "False"
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_a_qualified_name_reaches_hubs_and_lanes(self, tmp_path,
                                                     monkeypatch):
        m = self._boot(tmp_path, monkeypatch, sections={
            "AFC_BridgeBox AFC_hub Bambu_AMS_HT_1":
                {"afc_bowden_length": "1800"},
            "AFC_BridgeBox AFC_lane lane24": {"dist_hub": "120"}})
        folded = self._fold(m)
        assert folded["AFC_hub Bambu_AMS_HT_1"]["afc_bowden_length"] == "1800"
        assert folded["AFC_lane lane24"]["dist_hub"] == "120"
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_a_model_tag_covers_every_unit_of_that_model(self, tmp_path,
                                                         monkeypatch):
        # One block for every HT, one for every AMS 2 Pro.
        m = self._boot(tmp_path, monkeypatch,
                       roster=f"ht:{A}, ht:{B}, ams2:{C}", sections={
                           "AFC_BridgeBox ht": {"measure_on_insert": "False"},
                           "AFC_BridgeBox ams2": {"dry_max_temp": "60"}})
        folded = self._fold(m)
        for name in ("Bambu_AMS_HT_1", "Bambu_AMS_HT_2"):
            assert folded[f"AFC_BambuAMS {name}"]["measure_on_insert"] == (
                "False")
            assert folded[f"AFC_BambuAMS {name}"]["dry_max_temp"] == 85
        # The ams2 unit, named with no generation, matches by model; the
        # ht block does not reach it, so it keeps its own default.
        ams2 = folded["AFC_BambuAMS Bambu_AMS_1"]
        assert (ams2["dry_max_temp"], ams2["measure_on_insert"]) == (
            "60", False)
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_the_unit_exception_beats_the_model_policy(self, tmp_path,
                                                       monkeypatch):
        m = self._boot(tmp_path, monkeypatch, roster=f"ht:{A}, ht:{B}",
                       sections={
                           "AFC_BridgeBox ht": {"measure_on_insert": "False"},
                           "AFC_BridgeBox Bambu_AMS_HT_2":
                               {"measure_on_insert": "True"}})
        folded = self._fold(m)
        assert folded["AFC_BambuAMS Bambu_AMS_HT_1"][
            "measure_on_insert"] == "False"
        assert folded["AFC_BambuAMS Bambu_AMS_HT_2"][
            "measure_on_insert"] == "True"
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_identity_keys_are_protected(self, tmp_path, monkeypatch):
        m = self._boot(tmp_path, monkeypatch, sections={
            "AFC_BridgeBox Bambu_AMS_HT_1": {"unit_uid": "FFFF",
                                             "measure_on_insert": "False"}})
        folded = self._fold(m)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert (folded["unit_uid"], folded["measure_on_insert"]) == (
            self.UID, "False")
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_the_operator_outranks_learned_values(self, tmp_path,
                                                  monkeypatch):
        m = self._boot(tmp_path, monkeypatch,
                       autov=("[AFC_hub Bambu_AMS_HT_1]\n"
                              "afc_bowden_length : 3632.0\n"),
                       sections={"AFC_BridgeBox AFC_hub Bambu_AMS_HT_1":
                                 {"afc_bowden_length": "2000"}})
        folded = self._fold(m)
        assert folded["AFC_hub Bambu_AMS_HT_1"]["afc_bowden_length"] == "2000"
        # The learned value is kept, under the hub's own name.
        assert m._state_get("AFC_hub Bambu_AMS_HT_1",
                            "afc_bowden_length") == "3632.0"

    def test_overrides_are_never_laundered_into_learned_state(
            self, tmp_path, monkeypatch):
        # With the override section deleted, the override is gone.
        self._boot(tmp_path, monkeypatch, sections={
            "AFC_BridgeBox Bambu_AMS_HT_1": {"measure_on_insert": "False"}})
        m2 = self._boot(tmp_path, monkeypatch)
        folded = self._fold(m2)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert folded["measure_on_insert"] is True
        assert m2._learned_for(self.UID) == {}

    def test_a_chain_default_reaches_a_unit_with_no_other_override(
            self, tmp_path, monkeypatch):
        m = self._boot(tmp_path, monkeypatch, auto_spoolman_create="True")
        folded = self._fold(m)
        assert folded["AFC_BambuAMS Bambu_AMS_HT_1"][
            "auto_spoolman_create"] == "True"
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_a_model_section_overrides_the_chain_default(self, tmp_path,
                                                         monkeypatch):
        m = self._boot(tmp_path, monkeypatch, auto_spoolman_create="True",
                       sections={"AFC_BridgeBox ht":
                                 {"auto_spoolman_create": "False"}})
        folded = self._fold(m)
        assert folded["AFC_BambuAMS Bambu_AMS_HT_1"][
            "auto_spoolman_create"] == "False"
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_a_unit_section_overrides_both(self, tmp_path, monkeypatch):
        m = self._boot(tmp_path, monkeypatch, auto_spoolman_create="True",
                       sections={"AFC_BridgeBox ht":
                                 {"auto_spoolman_create": "False"},
                                 "AFC_BridgeBox Bambu_AMS_HT_1":
                                 {"auto_spoolman_create": "True"}})
        folded = self._fold(m)
        assert folded["AFC_BambuAMS Bambu_AMS_HT_1"][
            "auto_spoolman_create"] == "True"
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_the_fold_really_applies_the_chain_layer_first(self, tmp_path,
                                                           monkeypatch):
        # Each unit takes the most specific layer that sets a key: an ams2
        # with no section of its own gets the chain default, an HT the ht
        # section, and the HT with a section of its own that.
        m = self._boot(tmp_path, monkeypatch, roster=f"ht:{A}, ht:{B}, "
                       f"ams2:{C}", auto_spoolman_create="chain",
                       sections={"AFC_BridgeBox ht":
                                 {"auto_spoolman_create": "model"},
                                 "AFC_BridgeBox Bambu_AMS_HT_2":
                                 {"auto_spoolman_create": "unit"}})
        folded = self._fold(m)
        assert {name: folded[f"AFC_BambuAMS {name}"]["auto_spoolman_create"]
                for name in ("Bambu_AMS_1", "Bambu_AMS_HT_1",
                             "Bambu_AMS_HT_2")} == {
            "Bambu_AMS_1": "chain", "Bambu_AMS_HT_1": "model",
            "Bambu_AMS_HT_2": "unit"}
        assert m._unmatched_overrides == []
        assert m._learned_notes == []

    def test_a_model_section_with_no_unit_plugged_in_is_not_reported_as_unknown(
            self, tmp_path, monkeypatch):
        # A model default for hardware not plugged in starts working once
        # such a unit is claimed; a name that is no unit and no model is the
        # mistake reported.
        m = self._boot(tmp_path, monkeypatch, roster=f"ams2:{A}, ht:{B}",
                       sections={"AFC_BridgeBox ams1":
                                 {"measure_on_insert": "True"},
                                 "AFC_BridgeBox Nope":
                                 {"measure_on_insert": "True"}}, ready=True)
        assert m._unmatched_overrides == ["AFC_BridgeBox Nope"]
        # It names the section as written, not the unit section it maps to.
        assert _p3_log(m) == [
            ("warning", "AFC_BridgeBox chain1: [AFC_BridgeBox Nope] "
                        "overrides nothing -- no unit and no model "
                        "by that name, so its keys are ignored. Models: "
                        "ams1, ams2, boxed, ht, lite.")]


class TestAfcBridgeBoxLearnedSection:
    """A learned record is scoped by chain as well as by uid."""

    @staticmethod
    def _start(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
               name: str) -> afcBridgeBox:
        """
        :param name: chain1 (lanes from 24) or chain2 (lanes from 40, units
            named Bambu_AMS_B_<n>)
        :return afcBridgeBox: a start of that chain on a printer of its own,
            AAAA its roster's one unit on its one AMS bay
        """
        options: Dict[str, Any] = (
            {"lane_base": 24} if name == "chain1" else
            {"lane_base": 40, "unit_prefix": "Bambu_AMS_B",
             "buffer_chip_name": "bambu_buffer_chain2"})
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True)
        master = make_bridgebox(tmp_path, name, printer=printer,
                                roster=f"boxed:{A}", pool_ams=1, pool_ht=0,
                                **options)
        printer.connect()
        return master

    def test_two_chains_keep_separate_records(self, tmp_path, monkeypatch):
        m1 = self._start(tmp_path, monkeypatch, "chain1")
        m2 = self._start(tmp_path, monkeypatch, "chain2")
        assert (m1._learned_section(A), m2._learned_section(A)) == (
            "AFC_BridgeBox chain1 learned AAAA",
            "AFC_BridgeBox chain2 learned AAAA")
        m1._state_set({"AFC_BridgeBox chain1 learned AAAA":
                       {"afc_bowden_length": "3632.0"}})
        assert (m1._learned_for(A), m2._learned_for(A)) == (
            {"afc_bowden_length": "3632.0"}, {})
        # A restart of each: only chain1's unit takes the length.
        again1 = self._start(tmp_path, monkeypatch, "chain1")
        again2 = self._start(tmp_path, monkeypatch, "chain2")
        assert again2.name == "chain2"
        unit1 = again1.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        unit2 = again2.printer.lookup_object("AFC_BambuAMS Bambu_AMS_B_1")
        assert (unit1.afc_bowden_length, unit2.afc_bowden_length) == (
            3632.0, 3000.0)
        assert [dict(again1.printer.loaded)[
                    "AFC_BambuAMS Bambu_AMS_1"].fileconfig.get(
                        "AFC_BambuAMS Bambu_AMS_1", "afc_bowden_length",
                        fallback=None),
                dict(again2.printer.loaded)[
                    "AFC_BambuAMS Bambu_AMS_B_1"].fileconfig.get(
                        "AFC_BambuAMS Bambu_AMS_B_1", "afc_bowden_length",
                        fallback=None)] == ["3632.0", None]


class TestAfcBridgeBoxPersistLearned:
    """What a unit learned is saved under the uid on its bay, or not at all."""

    #: The section the HT's learned record is filed under.
    HT_REC = f"AFC_BridgeBox chain1 learned {H}"

    @staticmethod
    def _ht(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
            claim: bool = True) -> afcBridgeBox:
        """
        :param claim: claim HHHH onto the HT bay
        :return afcBridgeBox: HHHH, the roster: option's one unit, plugged
            in on a chain with one HT bay, Bambu_AMS_HT_1; logs cleared
        """
        master = _p3_chain(tmp_path, monkeypatch, roster=f"ht:{H}",
                           pool_ht=1, online=(H,), uids=(H,), htmask=1)
        if claim:
            assert master._claim_pool_unit(H, "ht") is not None
        _p3_quiet(master)
        return master

    @staticmethod
    def _pool(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
              uids: Sequence[str], online: Iterable[str],
              **options: Any) -> afcBridgeBox:
        """
        :param uids: the chain, in index order
        :param online: the uids online
        :return afcBridgeBox: AAAA recorded, three AMS bays and an HT bay,
            the chain watched once at 100s; logs cleared
        """
        master = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                           pool_ams=3, pool_ht=1, online=online, uids=uids,
                           **options)
        _p3_tick(master, 100, 100)
        _p3_quiet(master)
        return master

    @staticmethod
    def _refused(master: afcBridgeBox, tmp_path: pathlib.Path, unit: str,
                 key: str, value: Any) -> List[LogLine]:
        """
        Persist a value that must not be saved, and check nothing was.

        :param unit: the bay's unit name
        :param key: the option
        :param value: its value
        :return list: what the refusal logged
        """
        before = _p3_state(tmp_path)
        assert master.persist_learned(unit, key, value) is False
        assert _p3_state(tmp_path) == before
        log = list(_p3_log(master))
        _p3_quiet(master)
        return log

    def test_structural_keys_are_refused(self, tmp_path, monkeypatch):
        # A learned value must never be able to redefine what a unit is.
        m = self._ht(tmp_path, monkeypatch)
        log = self._refused(m, tmp_path, "Bambu_AMS_HT_1", "serial_port",
                             "/dev/evil")
        assert log == [
            ("warning", "AFC_BridgeBox chain1: refusing to persist "
                        "'serial_port' for Bambu_AMS_HT_1 -- only "
                        "afc_bowden_length, afc_unload_bowden_length are "
                        "saved")]
        assert _p3_console(m) == []

    def test_the_master_persists_where_klippy_cannot_parse_it(
            self, tmp_path, monkeypatch):
        # The state file, never the klippy-parsed auto_vars, where a section
        # for a fabricated unit is an orphan that halts the next start.
        m = self._ht(tmp_path, monkeypatch)
        autov = tmp_path / "AFC_auto_vars.cfg"
        before = autov.read_bytes() if autov.exists() else None
        assert m.persist_learned("Bambu_AMS_HT_1", "afc_bowden_length",
                                 3627.0) is True
        assert m._read_state().items(self.HT_REC) == [
            ("afc_bowden_length", "3627.0")]
        assert (autov.read_bytes() if autov.exists() else None) == before
        assert _p3_log(m) == []
        # The same value again rewrites nothing.
        assert self._refused(m, tmp_path, "Bambu_AMS_HT_1",
                             "afc_bowden_length", 3627.0) == []
        # Nor is a structural key written, whatever a unit asks for.
        keys = ("unit_uid", "extruder", "ams_model")
        assert [self._refused(m, tmp_path, "Bambu_AMS_HT_1", key, "x")
                for key in keys] == [
            [("warning", f"AFC_BridgeBox chain1: refusing to persist "
                         f"'{key}' for Bambu_AMS_HT_1 -- only "
                         f"afc_bowden_length, afc_unload_bowden_length are "
                         f"saved")]
            for key in keys]

    def test_a_key_outside_the_learned_set_is_refused(self, tmp_path,
                                                      monkeypatch):
        # A leftover key saved as learned would ride along with the unit to
        # every bay it claims.
        m = self._ht(tmp_path, monkeypatch)
        log = self._refused(m, tmp_path, "Bambu_AMS_HT_1",
                             "measure_on_insert", "False")
        assert log == [
            ("warning", "AFC_BridgeBox chain1: refusing to persist "
                        "'measure_on_insert' for Bambu_AMS_HT_1 -- only "
                        "afc_bowden_length, afc_unload_bowden_length are "
                        "saved")]
        assert m._learned_for(H) == {}

    def test_an_unbound_bay_saves_nothing(self, tmp_path, monkeypatch):
        # No unit is on the bay, so there is no uid to file the value under.
        m = self._ht(tmp_path, monkeypatch, claim=False)
        assert _p3_bay(m, "Bambu_AMS_HT_1").get("bound") is None
        log = self._refused(m, tmp_path, "Bambu_AMS_HT_1",
                             "afc_bowden_length", 3627.0)
        assert log == [
            ("info", "AFC_BridgeBox chain1: afc_bowden_length for "
                     "Bambu_AMS_HT_1 kept for this session only -- no unit "
                     "is bound to the bay")]

    def test_a_unit_carrying_another_uid_saves_nothing(self, tmp_path,
                                                       monkeypatch):
        # The bay and its unit object disagree about who is on it: filing
        # the value under either could hand one unit's path to the other.
        m = self._ht(tmp_path, monkeypatch)
        unit = m.printer.lookup_object("AFC_BambuAMS Bambu_AMS_HT_1")
        unit.unit_uid = "FFFFFFFFFFFFFFFFFFFFFFFF"
        log = self._refused(m, tmp_path, "Bambu_AMS_HT_1",
                             "afc_bowden_length", 3627.0)
        assert log == [
            ("info", "AFC_BridgeBox chain1: afc_bowden_length for "
                     "Bambu_AMS_HT_1 kept for this session only -- the bay "
                     "is bound to HHHH but its unit carries "
                     "FFFFFFFFFFFFFFFFFFFFFFFF")]
        # The same uid in lower case is the same unit.
        unit.unit_uid = H.lower()
        assert m.persist_learned("Bambu_AMS_HT_1", "afc_bowden_length",
                                 3627.0) is True
        assert m._learned_for(H) == {"afc_bowden_length": "3627.0"}
        assert _p3_log(m) == []

    def test_persist_then_restart_round_trip(self, tmp_path, monkeypatch):
        m = self._pool(tmp_path, monkeypatch, uids=(A,), online=(A,))
        assert m.persist_learned("Bambu_AMS_1", "afc_bowden_length",
                                 3627.0) is True
        assert m._learned_for(A) == {"afc_bowden_length": "3627.0"}
        assert _p3_log(m) == []
        again = _p3_chain(tmp_path, monkeypatch, roster="", pool_ams=3,
                          pool_ht=1)
        folded = dict(again._fold_and_sweep(
            BambuConfig(SEC, again.printer),
            again._roster_sections(again.units)))
        assert folded["AFC_BambuAMS Bambu_AMS_1"]["afc_bowden_length"] == (
            "3627.0")
        unit = again.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        assert unit.afc_bowden_length == 3627.0

    def test_persist_learned_files_under_the_bound_uid(self, tmp_path,
                                                       monkeypatch):
        m = self._pool(tmp_path, monkeypatch, uids=(A, C), online=(C,))
        assert _p3_bay(m, "Bambu_AMS_2")["bound"] == C
        assert m.persist_learned("Bambu_AMS_2", "afc_bowden_length",
                                 3627.0) is True
        assert m._learned_for(C) == {"afc_bowden_length": "3627.0"}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_2",
                            "afc_bowden_length") is None
        assert _p3_log(m) == []
        # Nothing is on Bambu_AMS_3, so nothing is saved for it.
        log = self._refused(m, tmp_path, "Bambu_AMS_3", "afc_bowden_length",
                             1.0)
        assert log == [
            ("info", "AFC_BridgeBox chain1: afc_bowden_length for "
                     "Bambu_AMS_3 kept for this session only -- no unit is "
                     "bound to the bay")]

    def test_the_new_uid_files_its_own_measurement(self, tmp_path,
                                                   monkeypatch):
        m = self._pool(tmp_path, monkeypatch, uids=(A, C, E), online=(C,),
                       auto_drop=True)
        unit = m.printer.lookup_object("AFC_BambuAMS Bambu_AMS_2")
        _p3_quiet(m)
        assert m.persist_learned(unit.name, "afc_bowden_length",
                                 3600.0) is True
        assert _p3_log(m) == []
        # CCCC's measurement as the unit holds it once measured.
        unit.afc_bowden_length = unit.afc_unload_bowden_length = 3600.0
        # Pulled: the bay goes back to the pool, then EEEE takes it.
        _p3_online(m, ())
        _p3_tick(m, 101, 112)
        _p3_online(m, (E,))
        _p3_tick(m, 113, 113)
        assert _p3_bay(m, "Bambu_AMS_2")["bound"] == E
        # Not CCCC's 3600: EEEE has no record, so the unit's default.
        assert (unit.afc_bowden_length, unit.afc_unload_bowden_length) == (
            3000.0, 3000.0)
        _p3_quiet(m)
        assert m.persist_learned(unit.name, "afc_bowden_length",
                                 3100.0) is True
        assert _p3_log(m) == []
        assert (m._learned_for(C), m._learned_for(E)) == (
            {"afc_bowden_length": "3600.0"}, {"afc_bowden_length": "3100.0"})
        assert m._read_state().has_section("AFC_BambuAMS Bambu_AMS_2") is (
            False)


class TestAfcBridgeBoxMigrateNameLearned:
    """Values stored under a bay name go to the uid that wore it."""

    @staticmethod
    def _bare(tmp_path: pathlib.Path,
              monkeypatch: pytest.MonkeyPatch) -> afcBridgeBox:
        """
        :return afcBridgeBox: a chain master with no roster and no bays,
            its learned notes cleared
        """
        master = _p3_chain(tmp_path, monkeypatch, roster="")
        master._learned_notes = []
        _p3_quiet(master)
        return master

    def test_two_owners_resolve_to_the_rostered_bay(self, tmp_path,
                                                     monkeypatch):
        # A legacy state recording two uids with one name. The rostered one
        # takes the name at boot, so the values are its own.
        _p3_seed(tmp_path, {"AFC_BambuAMS Bambu_AMS_1":
                            {"afc_bowden_length": "3632.0"}})
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      state={"name_map": "AAAA:Bambu_AMS_1, "
                                         "XXXX:Bambu_AMS_1",
                             "lane_map": "AAAA:24:4, XXXX:24:4"},
                      pool_ams=3, pool_ht=1)
        assert (m._learned_for(A), m._learned_for("XXXX")) == (
            {"afc_bowden_length": "3632.0"}, {})
        assert m._learned_notes == [
            (False, "learned values stored under Bambu_AMS_1 now belong to "
                    "its unit AAAA (afc_bowden_length)")]
        # The same narrowing inside the migration itself, and a name left
        # to two uids neither on its bay stays unread.
        m._name_map.update({"XXXX": "Bambu_AMS_1", "YYYY": "Bambu_AMS_9",
                            "WWWW": "Bambu_AMS_9"})
        m._state_set({"AFC_BambuAMS Bambu_AMS_1":
                      {"afc_unload_bowden_length": "3500"},
                      "AFC_BambuAMS Bambu_AMS_9":
                      {"afc_bowden_length": "1111"}})
        m._learned_notes = []
        _p3_quiet(m)
        m._migrate_name_learned()
        assert m._learned_for(A) == {"afc_bowden_length": "3632.0",
                                     "afc_unload_bowden_length": "3500"}
        assert m._learned_for("XXXX") == {}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_1",
                            "afc_unload_bowden_length") is None
        assert m._state_get("AFC_BambuAMS Bambu_AMS_9",
                            "afc_bowden_length") == "1111"
        assert m._learned_notes == [
            (False, "learned values stored under Bambu_AMS_1 now belong to "
                    "its unit AAAA (afc_unload_bowden_length), which wore "
                    "that name before XXXX took it; XXXX measures its own"),
            (True, "learned values stored under Bambu_AMS_9 left unread -- "
                   "WWWW, YYYY are all recorded with that name. Run "
                   "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=<uid> for each "
                   "one that is gone, and the one left takes them at the "
                   "next restart.")]
        assert _p3_log(m) == []

    def test_a_uid_moved_off_its_name_keeps_what_it_learned_there(
            self, tmp_path, monkeypatch):
        # What was stored under a uid's old name is its own; what is stored
        # under the name it moved to belongs to the uid that wore it before.
        m = self._bare(tmp_path, monkeypatch)
        m._state_set({
            "AFC_BambuAMS Bambu_AMS_5": {"afc_bowden_length": "3632.0"},
            "AFC_BambuAMS Bambu_AMS_3": {"afc_bowden_length": "1111.0"},
            "AFC_BambuAMS Bambu_AMS_6": {"afc_bowden_length": "2222.0"}})
        m._name_map = {"XXXX": "Bambu_AMS_3"}
        m._migrate_name_learned({"XXXX": "Bambu_AMS_5",
                                 "TTTT": "Bambu_AMS_3",
                                 "YYYY": "Bambu_AMS_6"})
        assert [m._learned_for(u) for u in ("XXXX", "TTTT", "YYYY")] == [
            {"afc_bowden_length": "3632.0"}, {"afc_bowden_length": "1111.0"},
            {"afc_bowden_length": "2222.0"}]
        assert [s for s in m._read_state().sections()
                if s.startswith("AFC_BambuAMS")] == []
        assert m._learned_notes == [
            (False, "learned values stored under Bambu_AMS_5 now belong to "
                    "its unit XXXX (afc_bowden_length)"),
            (False, "learned values stored under Bambu_AMS_3 now belong to "
                    "its unit TTTT (afc_bowden_length), which wore that "
                    "name before XXXX took it; XXXX measures its own"),
            (False, "learned values stored under Bambu_AMS_6 now belong to "
                    "its unit YYYY (afc_bowden_length)")]
        assert _p3_log(m) == []

    def test_a_name_several_gone_uids_shared_is_dropped_once_taken_over(
            self, tmp_path, monkeypatch):
        # Nobody left can claim it: the uids that shared the name lost it
        # to RRRR, which must not get their values.
        m = self._bare(tmp_path, monkeypatch)
        m._state_set({"AFC_BambuAMS Bambu_AMS_3":
                      {"afc_bowden_length": "3632.0"}})
        m._name_map = {"RRRR": "Bambu_AMS_3"}
        m._migrate_name_learned({"T1T1": "Bambu_AMS_3",
                                 "T2T2": "Bambu_AMS_3"})
        assert m._learned_for("RRRR") == {}
        assert m._read_state().has_section("AFC_BambuAMS Bambu_AMS_3") is (
            False)
        assert m._learned_notes == [(
            False, "learned values stored under Bambu_AMS_3 dropped -- "
            "T1T1, T2T2 were all recorded with that name and another unit "
            "took it over")]
        assert _p3_log(m) == []


class TestAfcBridgeBoxChainToRoster:
    """The chain reply, rendered in roster syntax."""

    #: A boxed unit at index 0 and an HT at index 4.
    UIDS = ["1111222233334444", "", "", "", "0123456789ABCDEF00003331"]

    def test_htmask_names_the_models(self):
        # Firmware's own flag: a bit per chain index.
        assert afcBridgeBox._chain_to_roster(self.UIDS, htmask=0b10000) == (
            "boxed:1111222233334444, ht:0123456789ABCDEF00003331")
        # The flag alone decides: index 0 flagged is an HT.
        assert afcBridgeBox._chain_to_roster(self.UIDS, htmask=0b00001) == (
            "ht:1111222233334444, boxed:0123456789ABCDEF00003331")

    def test_no_htmask_falls_back_to_enrollment_convention(self):
        # Boxed at 0..3, HTs at 4.., the addressing ranges enrollment uses.
        assert afcBridgeBox._chain_to_roster(self.UIDS, htmask=0) == (
            "boxed:1111222233334444, ht:0123456789ABCDEF00003331")
        assert afcBridgeBox._chain_to_roster(
            ["", "", "", "AAAA", "BBBB"], htmask=0) == (
            "boxed:AAAA, ht:BBBB")

    def test_empty_and_placeholder_positions_are_skipped(self):
        assert afcBridgeBox._chain_to_roster(
            ["", "FFFFFFFFFFFFFFFFFFFFFFFF", ""], htmask=0) == ""
        # A lower-case uid is kept, upper-cased.
        assert afcBridgeBox._chain_to_roster(
            ["", "ffffffffffffffffffffffff", " aaaa "], htmask=0) == (
            "boxed:AAAA")

    def test_an_empty_chain_is_an_empty_roster(self):
        assert afcBridgeBox._chain_to_roster([], htmask=0) == ""

    def test_the_output_is_valid_roster_syntax(self):
        # What the scout writes, _parse_roster must read back: the file is
        # the same grammar as the option, so it can be copied in.
        got = afcBridgeBox._chain_to_roster(["AAAA", "", "", "", "BBBB"],
                                            htmask=0b10000)
        assert got == "boxed:AAAA, ht:BBBB"
        assert afcBridgeBox._parse_roster(got) == [
            {"model": "boxed", "uid": "AAAA"},
            {"model": "ht", "uid": "BBBB"}]


class TestAfcBridgeBoxChainSnapshot:
    """One chain reply, read once."""

    class _GetterBridge(FakeBridge):
        """A bridge from before chain_snapshot: only the separate getters."""

        chain_snapshot = None

    class _OldBridge:
        """A bridge from before the dialect counters, too."""

        def chain_uids(self) -> List[str]:
            """:return list: the chain's uids"""
            return [A]

        def chain_diag(self) -> tuple:
            """:return tuple: htmask, text, counters"""
            return (0, "", (-1, 0, 0))

    def test_a_bridge_without_a_snapshot_is_read_through_its_getters(self):
        bridge = self._GetterBridge(uids=[A, ""], online=[True],
                                    htmask=0b10)
        bridge.dialect = (0b1, [4, 0])
        assert afcBridgeBox._chain_snapshot(bridge) == {
            "seq": None, "uids": [A, ""], "htmask": 0b10, "a2mask": 0b1,
            "a2asks": [4, 0]}
        # A bridge with one reads it, sequence number and all.
        full = FakeBridge(uids=[A, ""], online=[True], htmask=0b10)
        full.dialect, full.chain_seq = (0b1, [4, 0]), 7
        assert afcBridgeBox._chain_snapshot(full) == {
            "seq": 7, "uids": [A, ""], "htmask": 0b10, "a2mask": 0b1,
            "a2asks": [4, 0]}
        # One without dialect counters reports none.
        assert afcBridgeBox._chain_snapshot(self._OldBridge()) == {
            "seq": None, "uids": [A], "htmask": 0, "a2mask": 0,
            "a2asks": []}


class TestAfcBridgeBoxScoutReady:
    """klippy:ready holds each pool bay's saved records for its unit."""

    #: Alpha's records as the last session saved them.
    VAR = {"Alpha": {"lane24": _p3_rec("T24", spool_id=159, material="PLA"),
                     "lane25": {}}}
    #: Bravo's one record besides Alpha's.
    BRAVO = {"lane28": _p3_rec("T28", spool_id=7)}

    @staticmethod
    def _legacy(tmp_path: pathlib.Path, recorded: str) -> None:
        """
        Leave a recorded roster in the .roster file a build before the
        state file kept, which the next start takes in.

        :param recorded: the roster
        """
        (tmp_path / "AFC_BridgeBox_chain1.roster").write_text(
            "".join(f"{entry.strip()}\n" for entry in recorded.split(",")))

    def test_a_bays_records_are_held_for_its_owner_and_nothing_is_written(
            self, tmp_path, monkeypatch):
        m = _p3_named(tmp_path, monkeypatch, var=self.VAR,
                      owners="AAAA:Alpha", ready=False)
        # The band this start built is recorded once, when it changes.
        m._state_set({SEC: {"ams_band": m._ams_band}})
        state = _p3_state(tmp_path)
        _p3_quiet(m)
        m._scout_ready()
        assert m._held == {"Alpha": {"uid": A, "lanes": {
            "lane24": {"map": "T24", "current_map": "T24", "spool_id": 159,
                       "material": "PLA"}}}}
        assert _p3_state(tmp_path) == state
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha for their units")]
        assert _p3_console(m) == []

    @pytest.mark.parametrize("owners", ["CCCC:Charlie", ""],
                             ids=["other-bay", "emptied"])
    def test_a_bay_the_owner_key_does_not_name_holds_nothing(
            self, tmp_path, monkeypatch, owners):
        # AAAA was pinned to Alpha before this boot, but a key that is there
        # at all (emptied by FORGET, too) says who owns what.
        _p3_named(tmp_path, monkeypatch, ready=False)
        m = _p3_named(tmp_path, monkeypatch, var=self.VAR, owners=owners,
                      ready=False)
        _p3_quiet(m)
        m._scout_ready()
        assert m._pins_at_boot == {A: "Alpha"}
        assert m._held == {}
        assert _p3_log(m) == []
        assert _p3_console(m) == []

    def test_records_are_taken_before_the_chain_watch_starts(
            self, tmp_path, monkeypatch):
        m = _p3_named(tmp_path, monkeypatch, var=self.VAR,
                      owners="AAAA:Alpha", ready=False)
        reactor = m.printer.reactor
        seen: List[Any] = []
        start = reactor.register_timer

        def watch(callback: Any, waketime: float = 0.0) -> Any:
            """Note what is held when a timer starts."""
            seen.append((callback, dict(getattr(m, "_held", {}) or {})))
            return start(callback, waketime)

        monkeypatch.setattr(reactor, "register_timer", watch)
        _p3_quiet(m)
        m._scout_ready()
        assert seen == [(m._scout_tick, {"Alpha": {"uid": A, "lanes": {
            "lane24": self.VAR["Alpha"]["lane24"]}}})]
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha for their units")]
        assert _p3_console(m) == []

    def test_state_from_before_the_owner_key_goes_to_the_unit_on_the_bay(
            self, tmp_path, monkeypatch):
        # BBBB was claimed onto Bravo live in the last session, which saved
        # no name for it: it draws Bravo again now, as that build did.
        _p3_named(tmp_path, monkeypatch, ready=False)
        m = _p3_named(tmp_path, monkeypatch,
                      recorded=f"boxed:{A}, boxed:{B}",
                      var=dict(self.VAR, Bravo=self.BRAVO), ready=False)
        _p3_quiet(m)
        m._scout_ready()
        assert _p3_bay(m, "Bravo")["uid"] == B
        assert {bay: e["uid"] for bay, e in m._held.items()} == {
            "Alpha": A, "Bravo": B}
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha, Bravo for their units")]
        assert _p3_console(m) == []
        _p3_quiet(m)
        assert m._claim_pool_unit(B, "boxed") is (
            m.printer.lookup_object("AFC_BambuAMS Bravo"))
        lane = _p3_lane(m, "lane28")
        assert (lane.map, lane.current_map, lane.spool_id) == (
            ["T28"], "T28", 7)
        assert m.printer.afc.spool.calls == []

    def test_a_bay_another_unit_was_named_for_is_not_guessed(
            self, tmp_path, monkeypatch):
        first = _p3_named(tmp_path, monkeypatch, ready=False)
        first._state_set({SEC: {"name_map": "AAAA:Alpha, ZZZZ:Bravo",
                                "lane_map": "AAAA:24:4, ZZZZ:28:4"}})
        m = _p3_named(tmp_path, monkeypatch,
                      recorded=f"boxed:{A}, boxed:{B}",
                      var=dict(self.VAR, Bravo=self.BRAVO), ready=False)
        _p3_quiet(m)
        m._scout_ready()
        assert _p3_bay(m, "Bravo")["uid"] == B
        assert sorted(m._held) == ["Alpha"]
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha for their units")]
        assert _p3_console(m) == []

    def test_a_unit_that_wore_another_name_is_not_guessed(self, tmp_path,
                                                           monkeypatch):
        # BBBB was saved as Charlie, which the config no longer builds.
        first = _p3_named(tmp_path, monkeypatch,
                          recorded=f"boxed:{A}, boxed:{B}", ready=False)
        assert first._name_map == {A: "Alpha", B: "Bravo"}
        first._state_set({SEC: {"name_map": "AAAA:Alpha, BBBB:Charlie"}})
        m = _p3_named(tmp_path, monkeypatch,
                      recorded=f"boxed:{A}, boxed:{B}", pool_ams=2,
                      ams_names="Alpha, Bravo",
                      var=dict(self.VAR, Bravo=self.BRAVO), ready=False)
        _p3_quiet(m)
        m._scout_ready()
        assert m._pins_at_boot == {A: "Alpha", B: "Charlie"}
        assert _p3_bay(m, "Bravo")["uid"] == B
        assert sorted(m._held) == ["Alpha"]
        assert _p3_log(m) == [
            ("warning", "AFC_BridgeBox chain1: AMS BBBB was recorded as "
                        "Charlie, which no ams_names entry or default name "
                        "gives an AMS. It is now Bravo, and its lanes stay "
                        "lane28-lane31 (T28-T31). Lane records saved under "
                        "Charlie (spool, material, colour, T# map) do not "
                        "carry over to Bravo; a tagged spool is read again "
                        "from its tag. Its learned bowden lengths stay with "
                        "the unit."),
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha for their units")]
        assert _p3_console(m) == []

    def test_an_unlisted_unit_gets_the_one_spare_it_ran_on(self, tmp_path,
                                                           monkeypatch):
        # roster: lists AAAA only; the last session ran CCCC on the Bravo
        # spare and recorded it, with no name.
        _p3_named(tmp_path, monkeypatch, ready=False)
        self._legacy(tmp_path, f"boxed:{A}, boxed:{C}")
        m = _p3_named(tmp_path, monkeypatch, recorded=None,
                      roster=f"boxed:{A}",
                      var=dict(self.VAR, Bravo=self.BRAVO), ready=False)
        _p3_quiet(m)
        m._scout_ready()
        assert m._state_get(SEC, "roster") == "boxed:AAAA, boxed:CCCC"
        assert (tmp_path / "AFC_BridgeBox_chain1.roster").exists() is False
        assert _p3_bay(m, "Bravo")["uid"] is None
        assert {bay: e["uid"] for bay, e in m._held.items()} == {
            "Alpha": A, "Bravo": C}
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha, Bravo for their units")]
        assert _p3_console(m) == []
        _p3_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is (
            m.printer.lookup_object("AFC_BambuAMS Bravo"))
        lane = _p3_lane(m, "lane28")
        assert (lane.map, lane.current_map, lane.spool_id) == (
            ["T28"], "T28", 7)

    @pytest.mark.parametrize("recorded, var", [
        (f"boxed:{A}, boxed:{C}, boxed:{D}",
         {"Bravo": {"lane28": _p3_rec("T28", spool_id=7)}}),
        (f"boxed:{A}, boxed:{C}",
         {"Bravo": {"lane28": _p3_rec("T28", spool_id=7)},
          "Charlie": {"lane32": _p3_rec("T32", spool_id=8)}}),
    ], ids=["two-units", "two-spares"])
    def test_an_unlisted_unit_is_not_guessed_among_several(
            self, tmp_path, monkeypatch, recorded, var):
        _p3_named(tmp_path, monkeypatch, ready=False)
        self._legacy(tmp_path, recorded)
        m = _p3_named(tmp_path, monkeypatch, recorded=None, var=var,
                      roster=f"boxed:{A}", ready=False)
        _p3_quiet(m)
        m._scout_ready()
        assert m._held == {}
        assert _p3_log(m) == []
        assert _p3_console(m) == []

    def test_with_the_owner_key_an_unlisted_unit_gets_nothing(
            self, tmp_path, monkeypatch):
        _p3_named(tmp_path, monkeypatch, ready=False)
        self._legacy(tmp_path, f"boxed:{A}, boxed:{C}")
        m = _p3_named(tmp_path, monkeypatch, recorded=None,
                      roster=f"boxed:{A}",
                      var=dict(self.VAR, Bravo=self.BRAVO),
                      owners="AAAA:Alpha", ready=False)
        _p3_quiet(m)
        m._scout_ready()
        assert sorted(m._held) == ["Alpha"]
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha for their units")]
        assert _p3_console(m) == []

    @pytest.mark.parametrize("var", [None, "{not json", "[1, 2]",
                                     '{"Alpha": "x"}'])
    def test_an_unreadable_var_file_holds_nothing(self, tmp_path,
                                                  monkeypatch, var):
        m = _p3_named(tmp_path, monkeypatch, var=var, owners="AAAA:Alpha",
                      ready=False)
        _p3_quiet(m)
        m._scout_ready()
        assert m._held == {}
        assert _p3_log(m) == []
        assert _p3_console(m) == []

    def test_the_writers_stop_passes_and_the_hook_is_set_once(
            self, tmp_path, monkeypatch):
        m = _p3_named(tmp_path, monkeypatch, var=self.VAR,
                      owners="AAAA:Alpha", ready=False)
        afc = m.printer.afc
        inner = afc._var_write_queue
        _p3_quiet(m)
        m._scout_ready()
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha for their units")]
        assert _p3_console(m) == []
        hook = afc._var_write_queue
        assert hook is not inner
        _p3_quiet(m)
        m._scout_ready()
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha for their units")]
        assert _p3_console(m) == []
        assert afc._var_write_queue is hook
        # The writer's stop sentinel reaches AFC's queue as it is.
        stop = object()
        hook.put_nowait(stop)
        assert inner.get_nowait() is stop
        assert inner.empty()

    def test_an_afc_without_the_queue_is_left_alone(self, tmp_path,
                                                    monkeypatch):
        m = _p3_named(tmp_path, monkeypatch, var=self.VAR,
                      owners="AAAA:Alpha", ready=False)
        afc = m.printer.afc
        del afc._var_write_queue
        _p3_quiet(m)
        m._scout_ready()
        assert hasattr(afc, "_var_write_queue") is False
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                      "records of Alpha for their units"),
            ("debug", "AFC_BridgeBox chain1: AFC has no var-file write "
                      "queue; the records of a bay no unit is claimed onto "
                      "are held for this session only")]


class TestAfcBridgeBoxCaptureBootRecords:
    """What klippy:ready holds, and when it holds nothing."""

    VAR = {"Alpha": {"lane24": _p3_rec("T24", spool_id=159, material="PLA"),
                     "lane25": {}}}
    #: What klippy:ready logs for the HT that :meth:`_moved` moves.
    MOVED_LOG = [
        ("warning", "AFC_BridgeBox chain1: HT HHHH (Bambu_AMS_HT_1) on "
                    "lane40 (T40) cannot keep its lanes: "
                    "[AFC_BridgeBox chain2] further down the config "
                    "builds lane40. The AMS band is 1 bays, what "
                    "pool_ams and the recorded AMS need, and the HT "
                    "lanes follow it."),
        ("warning", "AFC_BridgeBox chain1: HT HHHH (Bambu_AMS_HT_1) "
                    "keeps its name, and its lanes and T# changed: "
                    "lane40 (T40) -> lane28 (T28)."),
        ("debug", "AFC_BridgeBox chain1: holding the saved lane "
                  "records of Bambu_AMS_HT_1 for their units")]

    class _TakenPins:
        """Klippy's pin registry, with every chip name taken."""

        def __init__(self) -> None:
            """Start with no registrations asked for."""
            self.asked: List[str] = []

        def register_chip(self, name: str, chip: Any) -> None:
            """:raises configparser.Error: the name is taken"""
            self.asked.append(name)
            raise configparser.Error(f"Duplicate chip name '{name}'")

    @staticmethod
    def _moved(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
               rec: Dict[str, Any]) -> afcBridgeBox:
        """
        chain1 recorded HT HHHH on lane40 behind four AMS bays and now
        builds one; chain2, further down the config, builds lane36-43, so
        the HT moves to lane28.

        :param rec: lane40's record, saved under the HT's bay
        :return afcBridgeBox: chain1, through klippy:ready
        """
        chain2 = {"serial_port": "/dev/serial/by-id/usb-chain2-if00",
                  "state_file": str(tmp_path / "AFC_BridgeBox.cfg"),
                  "pool_ams": "2", "pool_ht": "0", "lane_base": "36"}
        m = _p3_chain(tmp_path, monkeypatch, recorded=f"ht:{H}",
                      state={"lane_base": "24", "ams_band": "4",
                             "lane_map": f"{H}:40:1",
                             "name_map": f"{H}:Bambu_AMS_HT_1",
                             "bay_owner": f"{H}:Bambu_AMS_HT_1"},
                      sections={"AFC_BridgeBox chain2": chain2},
                      var={"Bambu_AMS_HT_1": {"lane40": rec}},
                      pool_ams=1, pool_ht=2)
        # chain2 has no roster and nothing recorded: it scouts, its pool
        # on lane36-43.
        m2 = make_bridgebox(tmp_path, "chain2", printer=m.printer, roster="",
                            pool_ams=2, pool_ht=0, lane_base=36,
                            unit_prefix="Bambu_AMS_B",
                            buffer_chip_name="bambu_buffer_chain2")
        m.printer.connect()
        assert (m2._roster_source, m2.units) == ("scout", [])
        assert [pu["lanes"] for pu in m2._pool_units] == [
            ["lane36", "lane37", "lane38", "lane39"],
            ["lane40", "lane41", "lane42", "lane43"]]
        assert _p3_bay(m, "Bambu_AMS_HT_1")["lanes"] == ["lane28"]
        assert _p3_bay(m, "Bambu_AMS_HT_1")["uid"] == H
        m._scout_ready()
        return m

    def test_without_afc_nothing_is_held(self, tmp_path, monkeypatch):
        m = _p3_named(tmp_path, monkeypatch, var=self.VAR,
                      owners="AAAA:Alpha", ready=False)
        _p3_quiet(m)
        m._capture_boot_records()
        assert sorted(m._held) == ["Alpha"]
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: holding the saved lane records "
                      "of Alpha for their units")]
        monkeypatch.setattr(m.printer, "_afc", None)
        _p3_quiet(m)
        m._capture_boot_records()
        assert m._held == {}
        assert _p3_log(m) == []

    def test_a_scout_only_chain_holds_nothing(self, tmp_path, monkeypatch):
        # Its chip name is taken (by an [mcu bambu_buffer], say): the scout
        # starts all the same.
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True)
        pins = self._TakenPins()
        monkeypatch.setitem(printer._objects, "pins", pins)
        m = make_bridgebox(tmp_path, printer=printer, roster="", pool_ams=0,
                           pool_ht=0)
        assert (m._roster_source, m.units, pins.asked) == (
            "scout", [], ["bambu_buffer"])
        write_unit_vars(printer, self.VAR)
        m._state_set({SEC: {"bay_owner": "AAAA:Alpha"}})
        _p3_quiet(m)
        m._capture_boot_records()
        assert m._held == {}
        assert _p3_log(m) == []

    def test_an_ht_moved_for_a_later_chain_keeps_its_record(
            self, tmp_path, monkeypatch):
        rec = dict(_p3_rec("T40", spool_id=55, material="PLA",
                           weight=300.0), name="lane40", tool_loaded=True)
        m = self._moved(tmp_path, monkeypatch, rec)
        # Its old home T# is not its T# now; the rest of the record is.
        assert m._held["Bambu_AMS_HT_1"] == {"uid": H, "lanes": {"lane28": {
            "name": "lane28", "spool_id": 55, "material": "PLA",
            "weight": 300.0, "tool_loaded": False}}}
        assert _p3_log(m) == self.MOVED_LOG

    def test_a_map_set_by_hand_moves_with_it(self, tmp_path, monkeypatch):
        m = self._moved(tmp_path, monkeypatch, _p3_rec("T3", spool_id=55))
        assert m._held["Bambu_AMS_HT_1"] == {"uid": H, "lanes": {"lane28": {
            "map": "T3", "current_map": "T3", "spool_id": 55,
            "tool_loaded": False}}}
        assert _p3_log(m) == self.MOVED_LOG


class TestAfcBridgeBoxMovedBayRecords:
    """A bay's records put on the lanes it has now."""

    #: An AMS bay on lane24-27.
    BAY = {"lanes": ["lane24", "lane25", "lane26", "lane27"]}

    def test_records_that_do_not_fit_the_bay_are_not_moved(self):
        # Fewer records than lanes.
        saved = {"lane28": _p3_rec("T28"), "lane29": _p3_rec("T29")}
        assert afcBridgeBox._moved_bay_records(self.BAY, saved) == {}
        # One of them a lane the bay has now.
        saved = {"lane24": _p3_rec("T24"), "lane29": _p3_rec("T29"),
                 "lane30": {}, "lane31": {}}
        assert afcBridgeBox._moved_bay_records(self.BAY, saved) == {}
        # Four records on four other lanes fit, in lane order.
        saved = {"lane31": _p3_rec("T31"), "lane28": _p3_rec("T5"),
                 "lane29": {}, "lane30": {}}
        assert afcBridgeBox._moved_bay_records(self.BAY, saved) == {
            "lane24": {"map": "T5", "current_map": "T5",
                       "tool_loaded": False},
            "lane27": {"tool_loaded": False}}


class TestAfcBridgeBoxHookReset:
    """AFC_RESET_MAPPING puts this chain's claimed lanes on their home T#."""

    class _Box:
        """A non-Bambu unit."""

        def __init__(self, lanes: Iterable[Any]) -> None:
            """:param lanes: its lanes"""
            self.name = "Box"
            self.lanes = {lane.name: lane for lane in lanes}

    class _BareSpool:
        """An AFC_spool with no mapping reset."""

        def __init__(self, afc: Any) -> None:
            """:param afc: the AFC core"""
            self.afc = afc

    @classmethod
    def _printer(cls, tmp_path: pathlib.Path,
                 monkeypatch: pytest.MonkeyPatch, *, ready: bool = True,
                 online: bool = True, box: bool = False) -> afcBridgeBox:
        """
        :param ready: run klippy:ready's handler
        :param online: wire the chain's bridge, AAAA, BBBB and HHHH online
        :param box: put a non-Bambu unit first, its lane0 on T0
        :return afcBridgeBox: AAAA, BBBB and HHHH rostered on four AMS bays
            from lane12 and an HT bay on lane28, with AFC's real AFC_spool
            and no Moonraker
        """
        m = _p3_chain(tmp_path, monkeypatch,
                      roster=f"boxed:{A}, boxed:{B}, ht:{H}", pool_ams=4,
                      pool_ht=1, ams_names="Alpha, Bravo, Charlie, Delta",
                      ht_names="Hot", lane_base=12,
                      online=(A, B, H) if online else None,
                      uids=(A, B, "", "", H), htmask=1 << 4)
        make_afc_spool(m.printer)
        afc = m.printer.afc
        afc.moonraker = None
        if box:
            lane0 = _P3OtherLane("lane0", ["T0"])
            afc.units = dict(Box=cls._Box([lane0]), **afc.units)
            afc.lanes["lane0"] = lane0
            afc.tool_cmds["T0"] = "lane0"
        if ready:
            m._scout_ready()
        return m

    @staticmethod
    def _reset(m: afcBridgeBox) -> FakeGcmd:
        """:return FakeGcmd: AFC_RESET_MAPPING RUNOUT=no, run"""
        cmd = _p3_cmd(RUNOUT="no")
        m.printer.afc.spool.cmd_AFC_RESET_MAPPING(cmd)
        return cmd

    def test_it_is_set_once_per_spool_object(self, tmp_path, monkeypatch):
        # The next start's master of the same chain, on files of its own;
        # built first, as each start has a bridge table of its own.
        (tmp_path / "again").mkdir()
        again = self._printer(tmp_path / "again", monkeypatch, ready=False,
                              online=False)
        m = self._printer(tmp_path, monkeypatch, ready=False, box=True)
        afc = m.printer.afc
        spool = afc.spool
        calls: List[tuple] = []
        real = spool._reset_mapping

        def counted(*args: Any, **kwargs: Any) -> Any:
            """AFC's reset, counted."""
            calls.append(args)
            return real(*args, **kwargs)

        spool._reset_mapping = counted
        m._scout_ready()
        wrapped = spool._reset_mapping
        assert wrapped.__wrapped__ is counted
        m._hook_reset(afc)
        # The next start's master takes the old one's place on the same
        # wrap.
        again._hook_reset(afc)
        assert spool._reset_mapping is wrapped
        assert spool._bridgebox_reset_masters == {"chain1": again}
        assert m._claim_pool_unit(H, "ht") is not None
        lane = _p3_lane(m, "lane28")
        _p3_quiet(m)
        cmd = self._reset(m)
        assert len(calls) == 1
        assert (lane.map, lane.current_map) == (["T28"], "T28")
        # AFC numbers the Box's lane around it.
        assert afc.lanes["lane0"].map == ["T0"]
        assert afc.tool_cmds == {"T0": "lane0", "T28": "lane28"}
        assert cmd.messages == []
        assert _p3_log(m) == [("info", "Tool mappings reset")]

    def test_a_restart_wraps_the_new_spool_object_not_the_class(
            self, tmp_path, monkeypatch):
        m = self._printer(tmp_path, monkeypatch)
        own = AFCSpool.__dict__["_reset_mapping"]
        assert m.printer.afc.spool._reset_mapping.__wrapped__.__func__ is own
        # RESTART: a new spool object; this module and its class stay.
        afc = m.printer.afc
        fresh = make_afc_spool()
        assert "_reset_mapping" not in vars(fresh)
        fresh.afc, fresh.function = afc, afc.function
        fresh.gcode, fresh.logger = afc.gcode, afc.logger
        afc.spool = fresh
        _p3_quiet(m)
        m._hook_reset(afc)
        assert _p3_log(m) == []
        assert fresh._reset_mapping.__wrapped__.__func__ is own
        assert hasattr(own, "__wrapped__") is False
        assert m._claim_pool_unit(H, "ht") is not None
        lane = _p3_lane(m, "lane28")
        _p3_quiet(m)
        cmd = self._reset(m)
        assert (lane.map, lane.current_map) == (["T28"], "T28")
        assert cmd.messages == []
        assert _p3_log(m) == [("info", "Tool mappings reset")]

    def test_a_spool_object_without_a_reset_is_left_alone(self, tmp_path,
                                                          monkeypatch):
        m = self._printer(tmp_path, monkeypatch, ready=False)
        afc = m.printer.afc
        bare = self._BareSpool(afc)
        afc.spool = bare
        _p3_quiet(m)
        m._scout_ready()
        assert vars(bare) == {"afc": afc}
        assert _p3_log(m) == [
            ("debug", "AFC_BridgeBox chain1: AFC has no mapping reset to "
                      "wrap; AFC_RESET_MAPPING numbers the Bambu lanes as "
                      "any other lane")]
        assert _hook_reset_mapping(None, m) is False
        assert m._claim_pool_unit(H, "ht") is not None
        lane = _p3_lane(m, "lane28")
        assert (lane.map, lane.current_map) == (["T28"], "T28")


class TestAfcBridgeBoxFillHeldBays:
    """
    AFC.save_vars saves each unit's registered lanes, so a pool bay no unit
    is claimed onto is saved empty. Every save passes through the chain
    master, which writes the records it holds for the bay's owner there: a
    restart before that unit is claimed again (offline all boot, released,
    or not claimed yet after PREP's first save) holds them again.
    """

    #: The UID Alpha is reserved for.
    A = "AAAA"
    #: Alpha's lane records as the last session saved them, under AAAA.
    VAR = {"Alpha": {
        "lane24": {"map": "T24", "current_map": "T24", "spool_id": 159,
                   "material": "PLA", "color": "#0086D6", "weight": 412.0},
        "lane25": {"map": "T25", "current_map": "T25", "material": "PETG"}}}
    #: Alpha's lanes, in slot order.
    ALPHA = ("lane24", "lane25", "lane26", "lane27")
    #: What klippy:ready logs when it holds Alpha's records.
    HOLDING: LogLine = (
        "debug", "AFC_BridgeBox chain1: holding the saved lane records of "
                 "Alpha for their units")
    #: What a claim of AAAA onto Alpha logs, with no T# to say anything of.
    CLAIMED: List[LogLine] = [
        ("debug", "AFC bambu Alpha: chain index not resolved yet (UID AAAA); "
                  "holding this unit's registrations until the chain map "
                  "arrives"),
        ("info", "AFC bambu Alpha: claimed UID AAAA as boxed and brought "
                 "online live (ams_index=0)."),
        ("info", "AFC_BridgeBox chain1: CLAIMED AAAA as boxed onto Alpha "
                 "(4 lanes) -- live, no restart.")]

    @classmethod
    def _chain(cls, tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
               *, var: Optional[Dict[str, Any]] = None,
               owners: Optional[str] = "AAAA:Alpha",
               print_state: Optional[str] = None) -> afcBridgeBox:
        """
        One start of chain1: three AMS bays named Alpha, Bravo and Charlie and
        an HT bay named Hot, AAAA recorded (so Alpha is reserved for it) and
        offline.

        The recorded roster is left as the first builds left it, a split
        AFC_BridgeBox_chain1.roster file the master absorbs at startup.

        :param tmp_path: where the state, roster and var files live
        :param monkeypatch: isolates the bridge table and the module clocks
        :param var: what AFC.var.unit holds; the file is left as it is when None
        :param owners: the bay_owner key an earlier start left; None for state
            from before the key
        :param print_state: print_stats state
        :return afcBridgeBox: the master; its logger is AFC's
        """
        (tmp_path / "AFC_BridgeBox_chain1.roster").write_text("boxed:AAAA\n")
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True,
                               print_state=print_state)
        options = dict(roster="", pool_ams=3, pool_ht=1, buffer="Bamb_1",
                       ams_names="Alpha, Bravo, Charlie", ht_names="Hot")
        master = make_bridgebox(tmp_path, printer=printer, **options)
        printer.connect()
        if owners is not None:
            master._state_set({SEC: {"bay_owner": owners}})
        if var is not None:
            write_unit_vars(printer, var)
        live_bridges()[master.serial_port] = FakeBridge(uids=[cls.A],
                                                        online=[False])
        master._scout_ready()
        return master

    @staticmethod
    def _lane(master: afcBridgeBox, name: str) -> AFCLane:
        """:return AFCLane: the fabricated lane of that name"""
        return master.printer.lookup_object(f"AFC_lane {name}")

    @staticmethod
    def _bays(snap: Dict[str, Any]) -> Dict[str, Any]:
        """:return dict: a snapshot's pool bays, without AFC's system block"""
        return {name: recs for name, recs in snap.items() if name != "system"}

    @staticmethod
    def _maps(recs: Dict[str, Dict[str, Any]]) -> Dict[str, Tuple[str, str]]:
        """:return dict: lane -> (map, current_map) as a snapshot saves them"""
        return {ln: (rec["map"], rec["current_map"]) for ln, rec in recs.items()}

    @staticmethod
    def _other(master: afcBridgeBox, name: str, cmd: str) -> AFCLane:
        """
        A lane of another unit on ``cmd``, as PREP leaves it: in AFC's lanes
        and tool table, the T# registered to AFC's CHANGE_TOOL.

        :param master: the chain master
        :param name: the lane
        :param cmd: the T# it holds
        :return AFCLane: the lane
        """
        afc = master.printer.afc
        lane = make_afc_lane(name, "Box_1", 0, printer=master.printer)
        lane.map, lane.current_map = [cmd], cmd
        afc.lanes[name] = lane
        afc.tool_cmds[cmd] = name
        afc.gcode.register_command(cmd, afc.cmd_CHANGE_TOOL)
        return lane

    def test_preps_save_before_any_claim_keeps_them(self, tmp_path,
                                                    monkeypatch):
        m = self._chain(tmp_path, monkeypatch, var=self.VAR)
        assert m.logger.messages == [self.HOLDING]
        _p4_quiet(m)
        m.printer.afc.save_vars()                     # PREP's first save
        (snap,) = drain_var_writes(m.printer)
        assert self._bays(snap) == {"Alpha": self.VAR["Alpha"], "Bravo": {},
                                    "Charlie": {}, "Hot": {}}
        assert m.logger.messages == []
        # A restart on what AFC's writer wrote holds them again, for AAAA.
        write_unit_vars(m.printer, snap)
        again = self._chain(tmp_path, monkeypatch)
        assert again._held == {"Alpha": {"uid": self.A,
                                         "lanes": self.VAR["Alpha"]}}
        assert again._claim_pool_unit(self.A, "boxed") is not None
        assert again.logger.messages == [self.HOLDING] + self.CLAIMED
        assert again.printer.gcode.messages == []
        lane24 = self._lane(again, "lane24")
        assert (lane24.spool_id, lane24.material, lane24.color,
                lane24.weight) == (159, "PLA", "#0086D6", 412.0)

    def test_the_file_keeps_its_own_copy(self, tmp_path, monkeypatch):
        m = self._chain(tmp_path, monkeypatch, var=self.VAR)
        _p4_quiet(m)
        m.printer.afc.save_vars()
        (snap,) = drain_var_writes(m.printer)
        snap["Alpha"]["lane24"]["spool_id"] = 7
        assert m._held == {"Alpha": {"uid": self.A,
                                     "lanes": self.VAR["Alpha"]}}
        assert m.logger.messages == []

    def test_a_claimed_bay_is_saved_from_its_lanes(self, tmp_path,
                                                   monkeypatch):
        m = self._chain(tmp_path, monkeypatch, var=self.VAR)
        assert m._claim_pool_unit(self.A, "boxed") is not None
        drain_var_writes(m.printer)
        self._lane(m, "lane24").material = "PETG"      # SET_MATERIAL
        _p4_quiet(m)
        m.printer.afc.save_vars()
        (snap,) = drain_var_writes(m.printer)
        fields = ("map", "current_map", "spool_id", "material", "color",
                  "weight")
        assert {ln: {k: rec[k] for k in fields}
                for ln, rec in snap["Alpha"].items()} == {
            "lane24": {"map": "T24", "current_map": "T24", "spool_id": 159,
                       "material": "PETG", "color": "#0086D6",
                       "weight": 412.0},
            "lane25": {"map": "T25", "current_map": "T25", "spool_id": None,
                       "material": "PETG", "color": "", "weight": 0.0},
            "lane26": {"map": "T26", "current_map": "T26", "spool_id": None,
                       "material": None, "color": "", "weight": 0.0},
            "lane27": {"map": "T27", "current_map": "T27", "spool_id": None,
                       "material": None, "color": "", "weight": 0.0}}
        assert (snap["Bravo"], snap["Charlie"], snap["Hot"]) == ({}, {}, {})
        assert m.logger.messages == []

    def test_a_released_bay_is_saved_with_its_records(self, tmp_path,
                                                      monkeypatch):
        m = self._chain(tmp_path, monkeypatch, var=self.VAR)
        assert m._claim_pool_unit(self.A, "boxed") is not None
        self._lane(m, "lane24").weight = 300.0
        drain_var_writes(m.printer)
        m._release_pool_unit(self.A)
        assert drain_var_writes(m.printer) == []
        _p4_quiet(m)
        m.printer.afc.save_vars()
        (snap,) = drain_var_writes(m.printer)
        held = m._held["Alpha"]
        assert held["uid"] == self.A
        assert snap["Alpha"] == held["lanes"]
        assert sorted(snap["Alpha"]) == list(self.ALPHA)
        rec = snap["Alpha"]["lane24"]
        assert {k: rec[k] for k in ("map", "current_map", "spool_id",
                                    "material", "color", "weight")} == {
            "map": "T24", "current_map": "T24", "spool_id": 159,
            "material": "PLA", "color": "#0086D6", "weight": 300.0}
        assert m.logger.messages == []
        write_unit_vars(m.printer, snap)
        again = self._chain(tmp_path, monkeypatch)
        assert again._held == {"Alpha": {"uid": self.A,
                                         "lanes": snap["Alpha"]}}
        assert again.logger.messages == [self.HOLDING]

    def test_a_lane_waiting_for_its_tool_is_saved_with_its_plan(
            self, tmp_path, monkeypatch):
        # Claimed during a print, lane24 waits for T24, which lane5 holds. A
        # restart before the print ends brings it back to that plan, not to
        # a map without T24.
        m = self._chain(tmp_path, monkeypatch, var=self.VAR,
                      print_state="printing")
        self._other(m, "lane5", "T24")
        _p4_quiet(m)
        assert m._claim_pool_unit(self.A, "boxed") is not None
        lane24 = self._lane(m, "lane24")
        assert (lane24.map, lane24.current_map) == (["NONE"], "")
        # One save per lane's TcmdAssign, then the claim's own.
        snaps = drain_var_writes(m.printer)
        m.printer.afc.save_vars()
        snaps += drain_var_writes(m.printer)
        planned = {"lane24": ("T24", "T24"), "lane25": ("T25", "T25"),
                   "lane26": ("T26", "T26"), "lane27": ("T27", "T27")}
        assert [self._maps(s["Alpha"]) for s in snaps] == [planned] * 6
        assert m.logger.messages == self.CLAIMED[:2] + [
            ("info", "AFC_BridgeBox chain1: lane24 takes T24 from lane5 once "
                     "the print ends, as the print may be using it; until "
                     "then lane24 has no T#.")] + self.CLAIMED[2:]
        write_unit_vars(m.printer, snaps[-1])
        again = self._chain(tmp_path, monkeypatch)
        self._other(again, "lane5", "T24")
        _p4_quiet(again)
        assert again._claim_pool_unit(self.A, "boxed") is not None
        # The Bambu lanes are lane24-lane36: lane5 gets the first T# past.
        afc = again.printer.afc
        assert {n: (lane.map, lane.current_map)
                for n, lane in afc.lanes.items()} == {
            "lane5": (["T37"], "T37"), "lane24": (["T24"], "T24"),
            "lane25": (["T25"], "T25"), "lane26": (["T26"], "T26"),
            "lane27": (["T27"], "T27")}
        assert afc.tool_cmds == {"T24": "lane24", "T25": "lane25",
                                 "T26": "lane26", "T27": "lane27",
                                 "T37": "lane5"}
        assert {cmd for cmd in afc.gcode.ready_gcode_handlers
                if cmd[1:].isdigit()} == {"T24", "T25", "T26", "T27", "T37"}
        assert again.logger.messages == self.CLAIMED[:2] + [
            ("warning", "AFC_BridgeBox chain1: T24 is the tool of Bambu lane "
                        "lane24 (Bambu lanes lane24-lane36 are T24-T36). "
                        "lane5 was mapped to it and is now T37. To give lane5 "
                        "another tool outside T24-T36, use SET_MAP LANE=lane5 "
                        "MAP=<T#>.")] + self.CLAIMED[2:]

    def test_state_from_before_the_owner_key_is_kept_by_preps_save(
            self, tmp_path, monkeypatch):
        # No bay_owner key (the first start after upgrading): AAAA, the unit
        # Alpha is reserved for, is guessed its owner and keeps the records
        # through PREP's save as well.
        m = self._chain(tmp_path, monkeypatch, var=self.VAR, owners=None)
        assert m._held["Alpha"]["uid"] == self.A
        _p4_quiet(m)
        m.printer.afc.save_vars()
        (snap,) = drain_var_writes(m.printer)
        assert snap["Alpha"] == self.VAR["Alpha"]
        assert m.logger.messages == []

    def test_a_bay_held_for_nobody_is_saved_empty(self, tmp_path,
                                                  monkeypatch):
        # bay_owner is recorded and names another bay: no unit owns Alpha.
        m = self._chain(tmp_path, monkeypatch, var=self.VAR,
                      owners="DDDD:Charlie")
        assert m._held == {}
        _p4_quiet(m)
        m.printer.afc.save_vars()
        (snap,) = drain_var_writes(m.printer)
        assert self._bays(snap) == {"Alpha": {}, "Bravo": {}, "Charlie": {},
                                    "Hot": {}}
        assert m.logger.messages == []

    def test_a_snapshot_without_the_bay_gets_nothing_added(self, tmp_path,
                                                           monkeypatch):
        m = self._chain(tmp_path, monkeypatch, var=self.VAR)
        _p4_quiet(m)
        data: Dict[str, Any] = {"Bravo": {}}
        m._fill_held_bays(data)
        assert data == {"Bravo": {}}
        assert m.logger.messages == []

    def test_a_bay_claimed_but_saved_empty_stays_empty(self, tmp_path,
                                                       monkeypatch):
        m = self._chain(tmp_path, monkeypatch, var=self.VAR)
        assert m._claim_pool_unit(self.A, "boxed") is not None
        assert m._held["Alpha"]["uid"] == self.A
        _p4_quiet(m)
        data: Dict[str, Any] = {"Alpha": {}}
        m._fill_held_bays(data)
        assert data == {"Alpha": {}}
        assert m.logger.messages == []

    def test_a_bay_entry_that_is_not_a_lane_table_is_left_as_it_is(
            self, tmp_path, monkeypatch):
        m = self._chain(tmp_path, monkeypatch, var=self.VAR)
        _p4_quiet(m)
        data: Dict[str, Any] = {"Alpha": ["lane24"]}
        m._fill_held_bays(data)
        assert data == {"Alpha": ["lane24"]}
        assert m.logger.messages == []


class TestAfcBridgeBoxPrepSettled:
    """
    Claims wait for AFC's PREP, which restores every lane from AFC.var.unit,
    and wait at most _prep_wait seconds after klippy:ready.
    """

    #: What _prep_settled logs the first time it holds a claim back.
    WAITING: LogLine = ("debug",
                        "AFC_BridgeBox chain1: unit claims wait for PREP")

    @staticmethod
    def _ready(tmp_path: pathlib.Path,
               monkeypatch: pytest.MonkeyPatch) -> afcBridgeBox:
        """
        :return afcBridgeBox: a master through klippy:ready at reactor time
            100, PREP not run yet, nothing logged since
        """
        printer = make_printer(monkeypatch=monkeypatch)
        printer.afc.prep_done = False
        master = make_bridgebox(tmp_path, printer=printer, ready=True)
        _p4_quiet(master)
        return master

    def test_an_afc_without_a_prep_flag_or_a_chain_not_ready_is_settled(
            self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        afc = m.printer.afc
        # PREP's wait has not begun to run out, so only the missing flag or
        # the missing ready time settles it.
        assert (m._ready_at, m.printer.reactor.now) == (100.0, 100.0)
        del afc.prep_done
        assert m._prep_settled() is True
        afc.prep_done = False
        m._ready_at = None
        assert m._prep_settled() is True
        assert m.logger.messages == []
        assert hasattr(m, "_prep_wait_said") is False

    def test_with_no_afc_claims_are_settled(self, tmp_path, monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        monkeypatch.setattr(m.printer, "_afc", None)
        assert m._prep_settled() is True
        assert m.logger.messages == []

    def test_once_prep_has_run_claims_are_settled(self, tmp_path,
                                                  monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        m.printer.afc.prep_done = True
        assert m._prep_settled() is True
        assert m.logger.messages == []
        assert hasattr(m, "_prep_wait_said") is False

    def test_a_reactor_that_cannot_tell_the_time_settles_it(self, tmp_path,
                                                            monkeypatch):
        m = self._ready(tmp_path, monkeypatch)

        def _no_clock() -> float:
            """:raises RuntimeError: the reactor cannot tell the time"""
            raise RuntimeError("no clock")
        monkeypatch.setattr(m.printer.reactor, "monotonic", _no_clock)
        assert m._prep_settled() is True
        assert m.logger.messages == []
        assert hasattr(m, "_prep_wait_said") is False

    def test_inside_the_wait_claims_hold_and_say_so_once(self, tmp_path,
                                                         monkeypatch):
        m = self._ready(tmp_path, monkeypatch)
        assert (m._ready_at, m._prep_wait) == (100.0, 90.0)
        m.printer.reactor.now = 189.5
        assert m._prep_settled() is False
        assert m._prep_wait_said is True
        assert m.logger.messages == [self.WAITING]
        m.printer.reactor.now = 189.9
        assert m._prep_settled() is False
        assert m.logger.messages == [self.WAITING]
        assert hasattr(m, "_prep_wait_warned") is False

    @pytest.mark.parametrize("wait, said", [(90.0, "90"), (135.0, "135")])
    def test_at_the_end_of_the_wait_claims_go_ahead_with_one_warning(
            self, tmp_path, monkeypatch, wait, said):
        m = self._ready(tmp_path, monkeypatch)
        m._prep_wait = wait
        m.printer.reactor.now = 100.0 + wait
        warned = [("warning", f"AFC_BridgeBox chain1: PREP has not finished "
                              f"{said}s after startup; claiming units "
                              f"anyway.")]
        assert m._prep_settled() is True
        assert m._prep_wait_warned is True
        assert m.logger.messages == warned
        m.printer.reactor.now += 30.0
        assert m._prep_settled() is True
        assert m.logger.messages == warned
        assert hasattr(m, "_prep_wait_said") is False


class TestAfcBridgeBoxEnsureBridge:
    """
    A scouting master opens the chain's bridge when no unit has, the same
    way the units open theirs, and shares one a unit already opened.
    """

    class _Bridge:
        """BambuBridge as _ensure_bridge builds and starts it."""

        def __init__(self, opener: Any, reactor: Any, logger: Any) -> None:
            """
            :param opener: opens the bridge's port
            :param reactor: the reactor it runs on
            :param logger: where it logs
            """
            self.opener, self.reactor, self.logger = opener, reactor, logger
            self.starts: List[bool] = []

        def start(self, defer_open: bool = False) -> None:
            """:param defer_open: keep retrying a port that is not up yet"""
            self.starts.append(defer_open)

    class _Port:
        """A port class recording how each port was opened."""

        opened: List[Tuple[tuple, Dict[str, Any]]] = []
        parse = staticmethod(TcpPort.parse)

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            """Record the arguments the port was opened with."""
            type(self).opened.append((args, kwargs))

    @classmethod
    def _scout(cls, tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
               port: str) -> afcBridgeBox:
        """
        A master with no roster and no pool, which fabricates nothing and
        scouts the chain on ``port``, built as klippy's load_config_prefix
        builds it on a printer with no gcode object and no pin registry:
        the bridge needs only the reactor. BambuBridge records what it is
        built and started with.

        :param port: the master's serial_port
        :return afcBridgeBox: the master; its logger is AFC's
        """
        printer = make_printer(monkeypatch=monkeypatch)
        monkeypatch.setattr(printer, "_gcode", None)
        monkeypatch.setitem(printer._objects, "pins", None)
        monkeypatch.setattr(bridge_mod, "BambuBridge", cls._Bridge)
        opts = bridgebox_options(tmp_path, roster="", serial_port=port,
                                 tcp_key="k", buffer="Bamb_1")
        master = load_config_prefix(BambuConfig(SEC, printer, opts))
        master.logger = printer.afc.logger
        return master

    def test_scouting_opens_a_tcp_bridge_like_the_units(self, tmp_path,
                                                        monkeypatch):
        # A WiFi bridge on a from-scratch chain: the scout opens a socket,
        # with the chain's tcp_key, and a bridge not up yet keeps retrying.
        port = "tcp://bridge.local:3333"
        monkeypatch.setattr(self._Port, "opened", [])
        monkeypatch.setattr(bridge_mod, "TcpPort", self._Port)
        m = self._scout(tmp_path, monkeypatch, port)
        bridge = m._ensure_bridge()
        assert isinstance(bridge, self._Bridge)
        assert m._bridge is bridge
        assert live_bridges() == {port: bridge}
        assert (bridge.reactor, bridge.logger) == (m.printer.reactor,
                                                   m.printer.afc.logger)
        assert bridge.starts == [True]
        assert self._Port.opened == []
        assert isinstance(bridge.opener(), self._Port)
        assert self._Port.opened == [
            (("bridge.local", 3333),
             {"timeout": 0.1, "write_timeout": 0.5, "key": "k"})]
        assert m.logger.messages == []

    def test_scouting_a_usb_bridge_still_fails_loud(self, tmp_path,
                                                    monkeypatch):
        # A USB bridge is opened at once, so a port that is not there fails
        # the scout instead of retrying.
        port = "/dev/serial/by-id/usb-Pico-if00"
        monkeypatch.setattr(self._Port, "opened", [])
        serial = types.ModuleType("serial")
        serial.Serial = self._Port                        # type: ignore
        monkeypatch.setitem(sys.modules, "serial", serial)
        m = self._scout(tmp_path, monkeypatch, port)
        bridge = m._ensure_bridge()
        assert m._bridge is bridge
        assert live_bridges() == {port: bridge}
        assert bridge.starts == [False]
        assert isinstance(bridge.opener(), self._Port)
        assert self._Port.opened == [
            ((port, 115200), {"timeout": 0.1, "write_timeout": 0.5})]
        assert m.logger.messages == []

    def test_a_bridge_a_unit_already_opened_is_shared(self, tmp_path,
                                                      monkeypatch):
        port = "tcp://bridge.local:3333"
        m = self._scout(tmp_path, monkeypatch, port)
        opened = FakeBridge()
        live_bridges()[port] = opened
        assert m._ensure_bridge() is opened
        assert m._bridge is opened
        assert live_bridges() == {port: opened}
        assert m.logger.messages == []


class TestAfcBridgeBoxScoutTick:
    """The chain watch, one scenario per test."""

    # ── builders ─────────────────────────────────────────────────────────

    @staticmethod
    def _chain(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch, *,
               name: str = "chain1",
               bridge: Optional[FakeBridge] = None,
               recorded: Optional[str] = None,
               state: Optional[Dict[str, str]] = None,
               sections: Optional[Dict[str, Dict[str, str]]] = None,
               print_state: Optional[str] = None,
               **options: Any) -> afcBridgeBox:
        """
        A chain master on a printer whose fabricated units and lanes are
        real, connected as klippy connects them, watching ``bridge``.

        :param tmp_path: where the state and auto_vars files live
        :param monkeypatch: isolates the bridge table and the module clocks
        :param name: the chain's name
        :param bridge: the chain's bridge, registered under the master's port
        :param recorded: the roster an earlier boot recorded; the roster:
            option is unset unless ``options`` gives one
        :param state: further state keys an earlier boot left
        :param sections: operator sections in the merged config
        :param print_state: print_stats state
        :param options: the master's options
        :return afcBridgeBox: the master; its logger is AFC's
        """
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True,
                               print_state=print_state)
        for section, keys in (sections or {}).items():
            printer.add_section(section, keys)
        keys = dict(state or {})
        if recorded is not None:
            keys["roster"] = recorded
            options.setdefault("roster", "")
        if keys:
            record_chain_state(tmp_path, name, **keys)
        master = make_bridgebox(tmp_path, name, printer=printer, **options)
        printer.connect()
        if bridge is not None:
            live_bridges()[master.serial_port] = bridge
        return master

    @staticmethod
    def _online(bridge: FakeBridge, flags: Sequence[bool]) -> None:
        """
        :param bridge: the chain's bridge
        :param flags: chain index -> online, as the next status reads it
        """
        bridge.status = {"units": [{"n": i, "online": bool(on)}
                                   for i, on in enumerate(flags)]}

    @staticmethod
    def _tick(master: afcBridgeBox, *times: float) -> float:
        """
        Run the watch at each time, with the printer's clock there too.

        :param master: the chain master
        :param times: reactor times, in order
        :return float: the last tick's next wake
        """
        wake = 0.0
        for t in times:
            master.printer.reactor.now = float(t)
            wake = master._scout_tick(float(t))
        return wake

    @staticmethod
    def _roster(master: afcBridgeBox) -> Optional[str]:
        """:return Optional[str]: the recorded roster"""
        return master._state_get(SEC, "roster")

    @staticmethod
    def _log(master: afcBridgeBox) -> List[LogLine]:
        """:return list: what the master and its units logged"""
        return master.logger.messages

    @staticmethod
    def _console(master: afcBridgeBox) -> List[LogLine]:
        """:return list: what reached the printer's gcode console"""
        return master.printer.gcode.messages

    @staticmethod
    def _bay(master: afcBridgeBox, name: str) -> Dict[str, Any]:
        """:return dict: the pool bay of that name"""
        return next(pu for pu in master._pool_units if pu["name"] == name)

    @staticmethod
    def _sent(bridge: FakeBridge, skip: Tuple[str, ...] = ("chain",)
              ) -> List[dict]:
        """:return list: what was sent on the bridge, the chain asks left out"""
        return [c for c in bridge.sent if c.get("cmd") not in skip]

    @staticmethod
    def _claim_log(uid: str, bay: str, model: str = "boxed",
                   lanes: int = 4, takes: Optional[str] = None
                   ) -> List[LogLine]:
        """
        :param takes: the learned lengths the claim hands over, as its line
            names them
        :return list: what a live claim of ``uid`` onto ``bay`` logs: the
            unit's two lines, the learned line when ``takes``, then the
            master's
        """
        learned: List[LogLine] = [] if takes is None else [
            ("info",
             f"AFC_BridgeBox chain1: {bay} takes the {takes} that UID {uid} "
             "learned.")]
        return [
            ("debug",
             f"AFC bambu {bay}: chain index not resolved yet (UID {uid}); "
             "holding this unit's registrations until the chain map arrives"),
            ("info",
             f"AFC bambu {bay}: claimed UID {uid} as {model} and brought "
             "online live (ams_index=0)."),
            *learned,
            ("info",
             f"AFC_BridgeBox chain1: CLAIMED {uid} as {model} onto {bay} "
             f"({lanes} lanes) -- live, no restart."),
        ]

    # ── a watch with units fabricated waits for their bridge ─────────────

    def test_the_enrolled_watch_never_creates_a_bridge(self, tmp_path,
                                                       monkeypatch):
        master = self._chain(tmp_path, monkeypatch)
        assert master._roster_source == "option"
        assert self._tick(master, 100.0) == 103.0
        assert master._watch_state == "no-bridge"
        assert live_bridges() == {}
        assert getattr(master, "_bridge", None) is None
        assert self._log(master) == []
        assert self._console(master) == []

    # ── the hot-plug poll interval ────────────────────────────────────────

    def test_a_roster_only_chain_does_not_poll_at_thirty_seconds(
            self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[HT_UID], online=[True])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge)
        assert (master.pool_ams, master.pool_ht) == (0, 0)
        assert self._tick(master, 100.0) == 101.0
        assert master._watch_next == 1.0
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_settled_pooled_chain_does_not_fall_back_to_a_heartbeat(
            self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[HT_UID], online=[True], htmask=0b1)
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             pool_ams=1, pool_ht=1)
        assert self._tick(master, 100.0) == 101.0
        assert master._watch_next == 1.0
        assert self._log(master) == [
            *self._claim_log(HT_UID, "Bambu_AMS_HT_1", "ht", 1),
        ]
        assert self._console(master) == []

    def test_an_operator_can_slow_it_down(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[], online=[])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             hotplug_poll=4.0)
        assert self._tick(master, 100.0) == 104.0
        assert self._log(master) == []
        assert self._console(master) == []

    def test_the_chain_query_never_reaches_the_bus(self, tmp_path,
                                                   monkeypatch):
        bridge = FakeBridge(uids=[HT_UID], online=[True])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge)
        self._tick(master, 100.0)
        assert bridge.sent == [{"cmd": "chain"}]
        assert self._log(master) == []
        assert self._console(master) == []

    # ── removal from the recorded roster is debounced ─────────────────────

    def _recorded_watch(self, tmp_path: pathlib.Path,
                        monkeypatch: pytest.MonkeyPatch,
                        bridge: FakeBridge,
                        roster: str = f"boxed:{A}, boxed:{B}",
                        **options: Any) -> afcBridgeBox:
        """
        :return afcBridgeBox: a master booted from ``roster`` as recorded,
            no pool, watching ``bridge``
        """
        return self._chain(tmp_path, monkeypatch, bridge=bridge,
                           recorded=roster, **options)

    def test_a_unit_absent_past_the_grace_is_recorded_removed(
            self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[B_p5], online=[True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge)
        self._tick(master, 100.0)
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB"
        assert master._missing_since == {A_p5: 100.0}
        self._tick(master, 180.0)
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB"
        assert master._missing_since == {A_p5: 100.0}
        self._tick(master, 300.0)
        assert self._roster(master) == "boxed:BBBB"
        assert master._missing_since == {}
        assert master._recorded_raw == "boxed:BBBB"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
            ("info",
             "AFC_BridgeBox chain1: unit(s) gone from the chain for over 120s: boxed:AAAA -- "
             "removed from the recorded roster; applies at the next RESTART. Their learned "
             "values are kept. Plugged back in before then, one keeps its lanes and name; after "
             "the restart they stay reserved for its return unless a new AMS would otherwise "
             "have no bay or push the HT lanes up. If one is never coming back, "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=<uid> frees its lanes and name for reuse."),
        ]
        assert self._console(master) == []

    def test_the_removal_line_names_the_chain_in_its_forget_hint(
            self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[B_p5], online=[True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge,
                                      name="chain2")
        assert master._roster_source == "file"
        self._tick(master, 100.0, 300.0)
        assert master._state_get("AFC_BridgeBox chain2",
                                 "roster") == "boxed:BBBB"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain2: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
            ("info",
             "AFC_BridgeBox chain2: unit(s) gone from the chain for over 120s: boxed:AAAA -- "
             "removed from the recorded roster; applies at the next RESTART. Their learned "
             "values are kept. Plugged back in before then, one keeps its lanes and name; after "
             "the restart they stay reserved for its return unless a new AMS would otherwise "
             "have no bay or push the HT lanes up. If one is never coming back, "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain2 UID=<uid> frees its lanes and name for reuse."),
        ]
        assert self._console(master) == []

    def test_the_removal_line_promises_no_bay_a_waiting_ams_takes(
            self, tmp_path, monkeypatch):
        four = f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{D}"
        bridge = FakeBridge(uids=[A_p5, B_p5, C_p5, D_p5, E_p5],
                            online=[True, True, True, False, True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge,
                                      roster=four + f", boxed:{E}")
        assert master._unbayed == {E_p5: None}
        self._tick(master, 100.0, 300.0)
        assert self._roster(master) == (
            "boxed:AAAA, boxed:BBBB, boxed:CCCC, boxed:EEEE")
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_4 (DDDD) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
            ("info",
             "AFC_BridgeBox chain1: unit(s) gone from the chain for over 120s: boxed:DDDD -- "
             "removed from the recorded roster; applies at the next RESTART. Their learned "
             "values are kept. Plugged back in before then, one keeps its lanes and name; after "
             "the restart they stay reserved for its return unless a new AMS would otherwise "
             "have no bay or push the HT lanes up (EEEE is waiting for a bay and takes one). If "
             "one is never coming back, AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=<uid> frees its "
             "lanes and name for reuse."),
        ]
        reboot = make_bridgebox(tmp_path, printer=make_printer(), roster="")
        assert reboot._name_map.get(E_p5) == "Bambu_AMS_4"
        assert self._console(master) == []

    def test_a_roster_option_is_not_pruned_or_reported_pending(
            self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[B_p5, "X9X9"], online=[True, True])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=f"boxed:{A}, boxed:{B}",
                             roster=f"boxed:{A}, boxed:{B}")
        self._tick(master, 100.0, 116.0, 300.0)
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB, boxed:X9X9"
        assert master._missing_since == {}
        assert master._watch_state == "watching"
        assert master.get_status()["pending_restart"] == []
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:X9X9 -- recorded. Add "
             "boxed:X9X9 to roster: to enroll it (the option is set and overrides the file)."),
        ]
        assert self._console(master) == []

    def test_a_sticky_offline_unit_is_not_enrolled(self, tmp_path,
                                                   monkeypatch):
        bridge = FakeBridge(uids=[A_p5], online=[False])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge,
                                      roster="")
        assert master._roster_source == "scout"
        self._tick(master, 100.0)
        assert (self._roster(master) or "") == ""
        assert master._enroll_since == {}
        self._online(bridge, [True])
        self._tick(master, 105.0)
        assert (self._roster(master) or "") == ""
        assert master._enroll_since == {A_p5: 105.0}
        self._tick(master, 121.0)
        assert self._roster(master) == "boxed:AAAA"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: chain reports [boxed:AAAA] -- written to AFC_BridgeBox.cfg. "
             "RESTART to enroll, or copy it into roster: to pin it."),
        ]
        assert self._console(master) == []

    def test_a_blip_online_does_not_enroll_a_forgotten_ghost(
            self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[A_p5], online=[False])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge,
                                      roster="")
        for i, on in enumerate([True, False, True, False, True, False,
                                True]):
            self._online(bridge, [on])
            self._tick(master, 100.0 + i * 5.0)
        assert (self._roster(master) or "") == ""
        assert master._enroll_since == {A_p5: 130.0}
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_live_forget_is_not_rescouted(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[A_p5, B_p5], online=[True, True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge,
                                      roster=f"boxed:{B}")
        master._forget_suppressed = {A_p5}
        self._tick(master, 100.0, 116.0)
        assert self._roster(master) == "boxed:BBBB"
        assert master._forget_suppressed == {A_p5}
        assert master._enroll_since == {}
        assert self._log(master) == []
        assert self._console(master) == []

    def test_an_outage_does_not_lift_the_hold(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[A_p5, B_p5], online=[True, True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge,
                                      roster=f"boxed:{B}")
        master._forget_suppressed = {A_p5}
        bridge._serial = None
        self._tick(master, 100.0, 105.0)
        assert master._forget_suppressed == {A_p5}
        bridge._serial = object()
        self._tick(master, 110.0, 126.0)
        assert self._roster(master) == "boxed:BBBB"
        assert master._forget_suppressed == {A_p5}
        assert master._enroll_since == {}
        assert self._log(master) == []
        assert self._console(master) == []

    def test_pulling_a_suppressed_unit_lifts_the_hold(self, tmp_path,
                                                      monkeypatch):
        bridge = FakeBridge(uids=[A_p5, B_p5], online=[False, True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge,
                                      roster=f"boxed:{B}")
        master._forget_suppressed = {A_p5}
        self._tick(master, 100.0)
        assert master._forget_suppressed == set()
        self._online(bridge, [True, True])
        self._tick(master, 105.0)
        assert self._roster(master) == "boxed:BBBB"
        self._tick(master, 121.0)
        assert self._roster(master) == "boxed:BBBB, boxed:AAAA"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:AAAA -- recorded. RESTART "
             "to enroll."),
        ]
        assert self._console(master) == []

    def test_the_countdown_announces_itself_when_it_starts(
            self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[B_p5], online=[True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge)
        self._tick(master, 100.0)
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
        ]
        self._tick(master, 110.0)
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
        ]
        assert self._console(master) == []

    def test_reappearing_resets_the_clock(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[B_p5], online=[True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge)
        self._tick(master, 100.0)
        bridge.uids = [B_p5, A_p5]
        self._online(bridge, [True, True])
        self._tick(master, 400.0)
        assert master._missing_since == {}
        bridge.uids = [B_p5]
        self._online(bridge, [True])
        self._tick(master, 800.0)
        assert master._missing_since == {A_p5: 800.0}
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
        ]
        assert self._console(master) == []

    def test_a_dead_link_proves_nothing(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[B_p5], online=[True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge)
        self._tick(master, 100.0)
        bridge._serial = None
        self._tick(master, 200.0)
        assert master._watch_state == "link-down"
        assert master._missing_since == {}
        bridge._serial = object()
        self._tick(master, 900.0)
        assert master._missing_since == {A_p5: 900.0}
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
        ]
        assert self._console(master) == []

    def test_a_dark_chain_removes_nothing(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[B_p5], online=[False])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge)
        self._tick(master, 100.0, 800.0, 1500.0)
        assert master._watch_state == "chain-dark"
        assert master._missing_since == {}
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB"
        assert self._log(master) == []
        assert self._console(master) == []

    def test_grace_zero_disables_auto_removal(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[B_p5], online=[True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge,
                                      removal_grace=0)
        self._tick(master, 100.0, 100000.0)
        assert master._watch_state == "removal-disabled"
        assert master._missing_since == {}
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB"
        assert self._log(master) == []
        assert self._console(master) == []

    def test_additions_merge_instead_of_replacing(self, tmp_path,
                                                  monkeypatch):
        bridge = FakeBridge(uids=[B_p5, C_p5], online=[True, True])
        master = self._recorded_watch(tmp_path, monkeypatch, bridge)
        self._tick(master, 100.0)
        assert master._enroll_since == {C_p5: 100.0}
        self._tick(master, 116.0)
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB, boxed:CCCC"
        assert master._enroll_since == {C_p5: 100.0}
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. RESTART "
             "to enroll."),
        ]
        assert self._console(master) == []

    # ── a boxed entry's generation is confirmed by the bus dialect ────────

    class _NoDialectBridge(FakeBridge):
        """A bridge from before the dialect counters: no snapshot, no
        chain_dialect, read through chain_uids and chain_diag."""

        chain_snapshot = None
        chain_dialect = None

    class _TornBridge(FakeBridge):
        """
        A bridge whose reader swaps the chain cache right after the first
        read: whatever is read after it sees the next reply.
        """

        def __init__(self, first: Tuple[List[str], int, List[int]],
                     then: Tuple[List[str], int, List[int]],
                     online: List[bool]) -> None:
            """
            :param first: (uids, a2mask, a2asks) of the reply read first
            :param then: the reply that lands right after
            :param online: chain index -> online
            """
            super().__init__(uids=first[0], online=online)
            self.dialect = (first[1], list(first[2]))
            self.then: Optional[Tuple[List[str], int, List[int]]] = then
            self.snapshots = 0

        def chain_snapshot(self) -> Dict[str, Any]:
            """:return dict: this reply; the next one lands right after"""
            self.snapshots += 1
            snap = super().chain_snapshot()
            snap["seq"] = self.snapshots
            if self.then is not None:
                self.uids = list(self.then[0])
                self.dialect = (self.then[1], list(self.then[2]))
                self.then = None
            return snap

    def _dialect_watch(self, tmp_path: pathlib.Path,
                       monkeypatch: pytest.MonkeyPatch, bridge: FakeBridge,
                       roster: str) -> afcBridgeBox:
        """
        :return afcBridgeBox: a master enrolled from the HT roster: option,
            with ``roster`` recorded, watching ``bridge``
        """
        return self._chain(tmp_path, monkeypatch, bridge=bridge,
                           recorded=roster, roster=HT_ROSTER)

    @staticmethod
    def _dialect(bridge: FakeBridge, a2mask: int, a2asks: List[int]
                 ) -> FakeBridge:
        """:return FakeBridge: the bridge, its dialect counters set"""
        bridge.dialect = (a2mask, list(a2asks))
        return bridge

    def test_an_answered_3702_confirms_ams2(self, tmp_path, monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A_p5], online=[True]), 0b1, [5])
        master = self._dialect_watch(tmp_path, monkeypatch, bridge,
                                     f"boxed:{A}")
        self._tick(master, 100.0)
        assert self._roster(master) == "ams2:AAAA"
        assert master._recorded_raw == "ams2:AAAA"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): AAAA -> ams2 "
             "-- applied to the running unit and recorded. Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_hundreds_of_silent_asks_confirm_ams1(self, tmp_path,
                                                  monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A_p5], online=[True]), 0, [301])
        master = self._dialect_watch(tmp_path, monkeypatch, bridge,
                                     f"boxed:{A}")
        self._tick(master, 100.0)
        assert self._roster(master) == "ams1:AAAA"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): AAAA -> ams1 "
             "-- applied to the running unit and recorded. Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_silence_from_an_offline_unit_proves_nothing(self, tmp_path,
                                                         monkeypatch):
        bridge = self._dialect(
            FakeBridge(uids=[A_p5, B_p5], online=[False, True]), 0, [999, 10])
        master = self._dialect_watch(tmp_path, monkeypatch, bridge,
                                     f"boxed:{A}, boxed:{B}")
        self._tick(master, 100.0)
        assert self._roster(master) == "boxed:AAAA, boxed:BBBB"
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_refinement_applies_to_the_running_unit_live(self, tmp_path,
                                                           monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A_p5], online=[True]), 0b1, [5])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=f"boxed:{A}", roster=f"boxed:{A}")
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        assert (unit.unit_uid, unit.ams_model, unit.has_heater) == (
            A_p5, "boxed", False)
        self._tick(master, 100.0)
        assert (unit.ams_model, unit.has_heater) == ("ams2", True)
        assert master.units[0]["model"] == "ams2"
        assert master.get_status()["pending_restart"] == []
        assert self._sent(bridge) == []
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): AAAA -> ams2 "
             "-- applied to the running unit and recorded. Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_status_names_the_gap_a_restart_will_close(self, tmp_path,
                                                        monkeypatch):
        bridge = FakeBridge(uids=[HT_UID, B_p5], online=[True, True],
                            htmask=0b1)
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=HT_ROSTER)
        self._tick(master, 100.0)
        assert master.get_status()["pending_restart"] == []
        self._tick(master, 116.0)
        assert master.get_status()["pending_restart"] == ["add boxed:BBBB"]
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:BBBB -- recorded. RESTART "
             "to enroll."),
        ]
        assert self._console(master) == []

    def test_a_pinned_entry_is_never_second_guessed(self, tmp_path,
                                                    monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A_p5], online=[True]), 0b1, [50])
        master = self._dialect_watch(tmp_path, monkeypatch, bridge,
                                     f"ams1:{A}")
        self._tick(master, 100.0)
        assert self._roster(master) == "ams1:AAAA"
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_bridge_without_the_counters_is_a_noop(self, tmp_path,
                                                     monkeypatch):
        bridge = self._NoDialectBridge(uids=[A_p5], online=[True])
        master = self._dialect_watch(tmp_path, monkeypatch, bridge,
                                     f"boxed:{A}")
        self._tick(master, 100.0)
        assert self._roster(master) == "boxed:AAAA"
        assert bridge.sent == [{"cmd": "chain"}]
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_counter_reset_never_unrefines_and_boxed_needs_fresh_evidence(
            self, tmp_path, monkeypatch):
        bridge = self._dialect(
            FakeBridge(uids=[A24, B24, C24], online=[True, True, True]),
            0b001, [3, 300, 11])
        master = self._dialect_watch(tmp_path, monkeypatch, bridge,
                                     f"boxed:{A24}, boxed:{B24}, "
                                     f"boxed:{C24}")
        self._tick(master, 100.0)
        assert self._roster(master) == (
            f"ams2:{A24}, ams1:{B24}, boxed:{C24}")
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2, {B24} -> ams1 -- applied to "
             "the running unit and recorded. Lanes and names do not move."),
        ]
        self._dialect(bridge, 0, [0, 0, 5])
        self._tick(master, 101.0)
        assert self._roster(master) == (
            f"ams2:{A24}, ams1:{B24}, boxed:{C24}")
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2, {B24} -> ams1 -- applied to "
             "the running unit and recorded. Lanes and names do not move."),
        ]
        self._dialect(bridge, 0, [0, 0, 12])
        self._tick(master, 102.0)
        assert self._roster(master) == (
            f"ams2:{A24}, ams1:{B24}, ams1:{C24}")
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2, {B24} -> ams1 -- applied to "
             "the running unit and recorded. Lanes and names do not move."),
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{C24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    # ── one read of the chain reply per tick ──────────────────────────────

    def test_a_torn_read_cannot_produce_a_verdict(self, tmp_path,
                                                  monkeypatch):
        bridge = self._TornBridge(([A24], 0, [0]), ([B24], 0b1, [3]),
                                  online=[True])
        master = self._dialect_watch(tmp_path, monkeypatch, bridge,
                                     f"boxed:{A24}")
        self._tick(master, 100.0)
        assert bridge.snapshots == 1
        self._tick(master, 101.0)
        assert bridge.snapshots == 2
        assert self._roster(master) == f"boxed:{A24}"
        assert self._log(master) == []
        assert self._console(master) == []

    # ── a live refinement reaches the unit and the firmware ───────────────

    def _live_refine(self, tmp_path: pathlib.Path,
                     monkeypatch: pytest.MonkeyPatch, bridge: FakeBridge, *,
                     resolved: bool = True, index: int = 0,
                     **options: Any) -> Tuple[afcBridgeBox, Any]:
        """
        A master enrolled with boxed:A24 whose unit is on ``bridge`` with
        its index pinned (``resolved``) at ``index``, after one tick.

        :return tuple: (master, unit)
        """
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=f"boxed:{A24}", roster=f"boxed:{A24}",
                             **options)
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        attach_bridge(unit, bridge)
        unit.ams_index = index
        unit._id_resolved = resolved
        self._tick(master, 100.0)
        return master, unit

    def test_ams2_gets_its_heater_and_the_firmware_is_told(self, tmp_path,
                                                           monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0b1,
                               [5])
        master, unit = self._live_refine(tmp_path, monkeypatch, bridge)
        assert (unit.ams_model, unit.has_heater) == ("ams2", True)
        bind = {"cmd": "bind", "uid": A24, "idx": 0, "m": 1}
        assert self._sent(bridge) == [{"cmd": "model", "unit": 0, "m": 1},
                                      bind, bind, {"cmd": "idsave"}]
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_ams1_moves_the_firmware_off_the_ams2_vocabulary(self, tmp_path,
                                                             monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0,
                               [12])
        master, unit = self._live_refine(tmp_path, monkeypatch, bridge)
        assert (unit.ams_model, unit.has_heater) == ("ams1", False)
        bind = {"cmd": "bind", "uid": A24, "idx": 0, "m": 0}
        assert self._sent(bridge) == [{"cmd": "model", "unit": 0, "m": 0},
                                      bind, bind, {"cmd": "idsave"}]
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_an_unresolved_index_is_not_told(self, tmp_path, monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0,
                               [12])
        master, unit = self._live_refine(tmp_path, monkeypatch, bridge,
                                         resolved=False)
        assert unit.ams_model == "ams1"
        assert self._sent(bridge) == []
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    @pytest.mark.parametrize("held, told", [(0, False), (1, True)])
    def test_only_the_index_the_verdict_was_read_at_is_told(
            self, tmp_path, monkeypatch, held, told):
        bridge = self._dialect(
            FakeBridge(uids=[C24, A24], online=[True, True]), 0, [0, 12])
        master, unit = self._live_refine(tmp_path, monkeypatch, bridge,
                                         index=held)
        assert unit.ams_model == "ams1"
        assert self._roster(master) == f"ams1:{A24}"
        bind = {"cmd": "bind", "uid": A24, "idx": 0, "m": 0}
        if told:
            assert self._sent(bridge) == [
                {"cmd": "model", "unit": 1, "m": 0}, bind, bind,
                {"cmd": "idsave"}]
        else:
            assert self._sent(bridge) == []
        assert self._log(master) == [
            ("info",
             f"AFC_BridgeBox chain1: generation confirmed by bus dialect "
             f"(0x3702): {A24} -> ams1 -- applied to the running unit and "
             f"recorded. Lanes and names do not move.")]
        assert self._console(master) == []

    def test_an_unresolved_unit_with_a_model_override_is_not_told(
            self, tmp_path, monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0,
                               [12])
        master, unit = self._live_refine(
            tmp_path, monkeypatch, bridge, resolved=False,
            sections={"AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        assert (unit.ams_model, unit.measure_on_insert) == ("ams1", True)
        assert self._sent(bridge) == []
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 is a ams1; applied measure_on_insert=True from "
             "[AFC_BridgeBox ams1] (the bay was fabricated as a spare, before its model was "
             "known)"),
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_a_model_override_rides_the_one_save(self, tmp_path,
                                                 monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0,
                               [12])
        master, unit = self._live_refine(
            tmp_path, monkeypatch, bridge,
            sections={"AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        assert unit.measure_on_insert is True
        assert self._sent(bridge) == [
            {"cmd": "model", "unit": 0, "m": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 0},
            {"cmd": "htunit", "unit": 0, "on": 0},
            {"cmd": "capen", "unit": 0, "on": 1},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 0},
            {"cmd": "idsave"},
        ]
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 is a ams1; applied measure_on_insert=True from "
             "[AFC_BridgeBox ams1] (the bay was fabricated as a spare, before its model was "
             "known)"),
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_nothing_is_saved_to_flash_mid_print(self, tmp_path,
                                                 monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0,
                               [12])
        master, unit = self._live_refine(tmp_path, monkeypatch, bridge,
                                         print_state="printing")
        assert unit.ams_model == "ams1"
        assert self._sent(bridge) == [
            {"cmd": "model", "unit": 0, "m": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 0}]
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move. The bridge has it in RAM and saves it at the next "
             "connect (no flash write mid-print)."),
        ]
        assert self._console(master) == []

    def test_two_verdicts_in_one_tick_save_once(self, tmp_path, monkeypatch):
        bridge = self._dialect(
            FakeBridge(uids=[A24, B24], online=[True, True]), 0, [12, 12])
        roster = f"boxed:{A24}, boxed:{B24}"
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=roster, roster=roster)
        units = [master.printer.lookup_object(f"AFC_BambuAMS {name}")
                 for name in ("Bambu_AMS_1", "Bambu_AMS_2")]
        for index, unit in enumerate(units):
            attach_bridge(unit, bridge)
            unit.ams_index = index
            unit._id_resolved = True
        self._tick(master, 100.0)
        assert [u.ams_model for u in units] == ["ams1", "ams1"]
        assert self._sent(bridge) == [
            {"cmd": "model", "unit": 0, "m": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 0},
            {"cmd": "model", "unit": 1, "m": 0},
            {"cmd": "bind", "uid": B24, "idx": 1, "m": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 0},
            {"cmd": "bind", "uid": B24, "idx": 1, "m": 0},
            {"cmd": "idsave"},
        ]
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1, {B24} -> ams1 -- applied to "
             "the running unit and recorded. Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_the_chains_dry_ceiling_reaches_a_live_ams2(self, tmp_path,
                                                        monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0b1,
                               [5])
        master, unit = self._live_refine(tmp_path, monkeypatch, bridge,
                                         dry_max_temp=55)
        assert (unit.ams_model, unit.dry_max_temp) == ("ams2", 55)
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_a_chain_ceiling_above_the_ams2s_leaves_it_at_65(self, tmp_path,
                                                            monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0b1,
                               [5])
        master, unit = self._live_refine(tmp_path, monkeypatch, bridge,
                                         dry_max_temp=80)
        assert (unit.ams_model, unit.dry_max_temp) == ("ams2", 65)
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    # ── a live ams2 gets the dryer a restart would give it ───────────────

    def _live_ams2(self, tmp_path: pathlib.Path,
                   monkeypatch: pytest.MonkeyPatch,
                   sections: Dict[str, Dict[str, str]], a2mask: int = 0b1,
                   a2asks: Tuple[int, ...] = (5,), **options: Any
                   ) -> Tuple[afcBridgeBox, Any]:
        """
        A boxed:A24 unit (65 degrees, no heater) refined by one tick.

        :return tuple: (master, unit)
        """
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]),
                               a2mask, list(a2asks))
        (tmp_path / "live").mkdir()
        return self._live_refine(tmp_path / "live", monkeypatch, bridge,
                                 sections=sections, **options)

    @staticmethod
    def _restart_unit(tmp_path: pathlib.Path,
                      sections: Dict[str, Dict[str, str]],
                      **options: Any) -> Any:
        """
        :return Any: the Bambu_AMS_1 unit the next boot builds once the
            roster records ams2:A24, with the same sections and options
        """
        (tmp_path / "restart").mkdir()
        printer = make_printer(fabricate=True)
        for section, keys in sections.items():
            printer.add_section(section, keys)
        make_bridgebox(tmp_path / "restart", printer=printer,
                       roster=f"ams2:{A24}", **options)
        return printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")

    @pytest.mark.parametrize("sections, options, expected", [
        ({}, {}, (True, 65)),
        ({}, {"dry_max_temp": 55}, (True, 55)),
        ({}, {"dry_max_temp": 75}, (True, 65)),
        ({"AFC_BridgeBox ams2": {"dry_max_temp": "50"}}, {}, (True, 50)),
        ({"AFC_BridgeBox ams2": {"dry_max_temp": "50"}},
         {"dry_max_temp": 60}, (True, 50)),
        ({"AFC_BridgeBox ams2": {"dry_max_temp": "50"},
          "AFC_BridgeBox Bambu_AMS_1": {"dry_max_temp": "45"}}, {},
         (True, 45)),
    ])
    def test_the_live_ceiling_is_the_restarts(self, tmp_path, monkeypatch,
                                              sections, options, expected):
        master, unit = self._live_ams2(tmp_path, monkeypatch, sections,
                                       **options)
        assert unit.ams_model == "ams2"
        assert (unit.has_heater, unit.dry_max_temp) == expected
        again = self._restart_unit(tmp_path, sections, **options)
        assert (again.has_heater, again.dry_max_temp) == expected
        assert self._log(master) == [
            ("info",
             f"AFC_BridgeBox chain1: generation confirmed by bus dialect "
             f"(0x3702): {A24} -> ams2 -- applied to the running unit and "
             f"recorded. Lanes and names do not move.")]
        assert self._console(master) == []

    def test_a_heater_override_is_kept(self, tmp_path, monkeypatch):
        sections = {"AFC_BridgeBox Bambu_AMS_1": {"heater": "False"}}
        master, unit = self._live_ams2(tmp_path, monkeypatch, sections)
        assert (unit.ams_model, unit.has_heater) == ("ams2", False)
        assert self._restart_unit(tmp_path, sections).has_heater is False
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_the_ceiling_is_clamped_to_the_hard_max(self, tmp_path,
                                                    monkeypatch):
        sections = {"AFC_BridgeBox ams2": {"dry_max_temp": "99"}}
        master, unit = self._live_ams2(tmp_path, monkeypatch, sections)
        assert (unit.has_heater, unit.dry_max_temp) == (True, 85)
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_an_ams1_keeps_its_ceiling_and_gets_no_heater(self, tmp_path,
                                                          monkeypatch):
        master, unit = self._live_ams2(tmp_path, monkeypatch, {}, a2mask=0,
                                       a2asks=(12,), dry_max_temp=55)
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == (
            "ams1", False, 65)
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    # ── the chain poll waits out a silent bridge ─────────────────────────

    class _NoSilenceBridge(FakeBridge):
        """A bridge from before silent_for: it cannot say how long it has
        been quiet."""

        silent_for = None

    @staticmethod
    def _pool_at_ask(master: afcBridgeBox, bridge: FakeBridge,
                     bay: str) -> List[bool]:
        """
        :return list: filled with whether ``bay``'s unit was still in the
            pool at each chain ask: the watch's own ask comes before any
            claim, a claim's index request after it
        """
        unit = master.printer.lookup_object(f"AFC_BambuAMS {bay}")
        seen: List[bool] = []

        def _record(_bridge: FakeBridge, frame: dict) -> None:
            if frame.get("cmd") == "chain":
                seen.append(unit.pool)
        bridge.on_send = _record
        return seen

    @staticmethod
    def _asks(bridge: FakeBridge) -> List[dict]:
        """:return list: the chain asks sent on the bridge"""
        return [c for c in bridge.sent if c.get("cmd") == "chain"]

    def test_not_asked_mid_stall(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[A_p5], online=[True])
        bridge.silent = 5.0
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             roster=f"boxed:{A}")
        assert self._tick(master, 100.0) == 101.0
        assert self._asks(bridge) == []
        assert master._enroll_since == {A_p5: 100.0}
        assert self._log(master) == []
        assert self._console(master) == []

    @pytest.mark.parametrize("quiet", [0.3, 2.0, None])
    def test_asked_when_fresh_or_when_it_cannot_say(self, tmp_path,
                                                    monkeypatch, quiet):
        bridge = FakeBridge(uids=[A_p5], online=[True])
        bridge.silent = quiet
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             roster=f"boxed:{A}")
        self._tick(master, 100.0)
        assert self._asks(bridge) == [{"cmd": "chain"}]
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_bridge_without_silent_for_is_asked_as_before(
            self, tmp_path, monkeypatch):
        bridge = self._NoSilenceBridge(uids=[A_p5], online=[True])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             roster=f"boxed:{A}")
        self._tick(master, 100.0)
        assert self._asks(bridge) == [{"cmd": "chain"}]
        assert self._log(master) == []
        assert self._console(master) == []

    def test_the_threshold_follows_a_slow_poll(self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[A_p5], online=[True])
        bridge.silent = 5.0
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             roster=f"boxed:{A}", hotplug_poll=4.0)
        self._tick(master, 100.0)
        assert self._asks(bridge) == [{"cmd": "chain"}]
        bridge.silent = 6.5
        self._tick(master, 104.0)
        assert self._asks(bridge) == [{"cmd": "chain"}]
        assert self._log(master) == []
        assert self._console(master) == []

    def test_prune_still_runs_on_the_cached_chain(self, tmp_path,
                                                  monkeypatch):
        bridge = FakeBridge(uids=[B_p5], online=[True])
        bridge.silent = 12.0
        master = self._recorded_watch(tmp_path, monkeypatch, bridge)
        self._tick(master, 100.0, 180.0, 300.0)
        assert self._asks(bridge) == []
        assert self._roster(master) == "boxed:BBBB"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
            ("info",
             "AFC_BridgeBox chain1: unit(s) gone from the chain for over 120s: boxed:AAAA -- "
             "removed from the recorded roster; applies at the next RESTART. Their learned "
             "values are kept. Plugged back in before then, one keeps its lanes and name; after "
             "the restart they stay reserved for its return unless a new AMS would otherwise "
             "have no bay or push the HT lanes up. If one is never coming back, "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=<uid> frees its lanes and name for reuse."),
        ]
        assert self._console(master) == []

    def test_claim_still_runs_on_the_cached_chain(self, tmp_path,
                                                  monkeypatch):
        bridge = FakeBridge(uids=[A_p5], online=[True])
        bridge.silent = 12.0
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             roster=f"boxed:{A}", pool_ams=1, pool_ht=0)
        asks = self._pool_at_ask(master, bridge, "Bambu_AMS_1")
        self._tick(master, 100.0)
        # The one ask is the claim's own index request, made off the pool.
        assert asks == [False]
        assert self._bay(master, "Bambu_AMS_1")["bound"] == A_p5
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
        ]
        assert self._console(master) == []

    def test_a_frozen_online_flag_does_not_release_a_unit(self, tmp_path,
                                                          monkeypatch):
        bridge = FakeBridge(uids=[A_p5], online=[True])
        bridge.silent = 0.2
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             roster=f"boxed:{A}", pool_ams=1, pool_ht=0,
                             auto_drop=True, release_grace=10.0,
                             release_settle=5.0)
        asks = self._pool_at_ask(master, bridge, "Bambu_AMS_1")
        self._tick(master, 0.0)
        bay = self._bay(master, "Bambu_AMS_1")
        assert bay["bound"] == A_p5
        # The watch's ask, then the claim's own.
        assert asks == [True, False]
        bridge.silent = 15.0
        self._tick(master, 3.0, 6.0, 9.0, 12.0, 15.0)
        assert bay["bound"] == A_p5
        assert master._last_online == {A_p5: 15.0}
        assert asks == [True, False]
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            ("info", "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:AAAA -- recorded."),
        ]
        assert self._console(master) == []

    # ── a live refine leaves what a restart would ─────────────────────────

    def test_an_override_is_not_pushed_to_an_index_the_unit_left(
            self, tmp_path, monkeypatch):
        bridge = self._dialect(
            FakeBridge(uids=[C24, A24], online=[True, True]), 0, [0, 12])
        master, unit = self._live_refine(
            tmp_path, monkeypatch, bridge, index=0,
            sections={"AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        assert (unit.ams_model, unit.measure_on_insert) == ("ams1", True)
        assert self._sent(bridge) == []
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 is a ams1; applied measure_on_insert=True from "
             "[AFC_BridgeBox ams1] (the bay was fabricated as a spare, before its model was "
             "known)"),
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
        ]
        assert self._console(master) == []

    def test_while_printing_only_the_refined_uid_is_bound_and_nothing_saved(
            self, tmp_path, monkeypatch):
        bridge = self._dialect(
            FakeBridge(uids=[A24, B24], online=[True, True]), 0, [12, 0])
        roster = f"boxed:{A24}, ams2:{B24}"
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=roster, roster=roster,
                             print_state="printing")
        units = [master.printer.lookup_object(f"AFC_BambuAMS {name}")
                 for name in ("Bambu_AMS_1", "Bambu_AMS_2")]
        for index, unit in enumerate(units):
            attach_bridge(unit, bridge)
            unit.ams_index = index
            unit._id_resolved = True
        assert units[1].ams_model == "ams2"
        self._tick(master, 100.0)
        assert [u.ams_model for u in units] == ["ams1", "ams2"]
        assert self._sent(bridge) == [
            {"cmd": "model", "unit": 0, "m": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 0}]
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams1 -- applied to the running unit and recorded. "
             "Lanes and names do not move. The bridge has it in RAM and saves it at the next "
             "connect (no flash write mid-print)."),
        ]
        assert self._console(master) == []

    # ── a phantom online flag cannot cancel a release ────────────────────

    def _claimed(self, tmp_path: pathlib.Path,
                 monkeypatch: pytest.MonkeyPatch, **options: Any
                 ) -> Tuple[afcBridgeBox, FakeBridge, Dict[str, Any]]:
        """
        boxed:AAAA claimed live onto its bay by a tick at 0, the claim's
        lines cleared.

        :param options: the master's options over pool_ams=1, pool_ht=0,
            release_grace=10, release_settle=5
        :return tuple: (master, bridge, the bay)
        """
        bridge = FakeBridge(uids=[A_p5], online=[True])
        opts: Dict[str, Any] = dict(release_grace=10.0, release_settle=5.0,
                                    pool_ams=1, pool_ht=0)
        opts.update(options)
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=f"boxed:{A}", roster=f"boxed:{A}",
                             **opts)
        self._tick(master, 0.0)
        bay = self._bay(master, "Bambu_AMS_1")
        assert bay["bound"] == A_p5
        self._log(master).clear()
        return master, bridge, bay

    #: The removed popup boxed:AAAA leaves when its bay is released.
    REMOVED_AAAA = [
        ("respond_raw", "// action:prompt_begin AMS removed: Bambu_AMS_1"),
        ("respond_raw",
         "// action:prompt_text Bambu_AMS_1 (UID AAAA) was unplugged; its bay is held for a "
         "re-plug."),
        ("respond_raw",
         "// action:prompt_text Re-plug it and it reclaims the same lanes/T#. Or forget it "
         "to free the bay to the pool:"),
        ("respond_raw",
         "// action:prompt_button Forget Bambu_AMS_1|AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
         "UID=AAAA|error"),
        ("respond_raw",
         "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
         "MSG=action:prompt_end|info"),
        ("respond_raw", "// action:prompt_show"),
    ]

    def _reads(self, master: afcBridgeBox, bridge: FakeBridge,
               reads: Sequence[Tuple[float, bool]]) -> None:
        """
        :param reads: (time, online) of each tick in turn
        """
        for t, on in reads:
            self._online(bridge, [on])
            self._tick(master, t)

    def test_a_phantom_flap_still_drops_after_the_grace(
            self, tmp_path, monkeypatch):
        master, bridge, bay = self._claimed(tmp_path, monkeypatch,
                                            auto_drop=True)
        flaps = [False, True, False, True, False, True, False]
        self._reads(master, bridge,
                    [(1.0 + 3 * i, on) for i, on in enumerate(flaps)])
        assert bay["bound"] is None
        assert master._released_at == {"AAAA": 13.0}
        assert (master._last_online, master._online_run) == ({}, {})
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
        ]
        assert self._console(master) == self.REMOVED_AAAA

    def test_a_solid_replug_is_not_dropped(self, tmp_path, monkeypatch):
        master, bridge, bay = self._claimed(tmp_path, monkeypatch,
                                            auto_drop=True)
        self._reads(master, bridge,
                    [(t, True) for t in (3.0, 6.0, 9.0, 12.0, 15.0, 18.0)])
        assert bay["bound"] == A_p5
        assert master._last_online == {A_p5: 18.0}
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_late_solid_replug_cancels_the_pending_drop(
            self, tmp_path, monkeypatch):
        master, bridge, bay = self._claimed(tmp_path, monkeypatch,
                                            auto_drop=True)
        self._reads(master, bridge,
                    [(1.0, False), (4.0, False), (7.0, False), (10.0, True),
                     (13.0, True), (16.0, True), (19.0, True), (22.0, True)])
        assert bay["bound"] == A_p5
        assert master._last_online == {A_p5: 22.0}
        assert self._log(master) == [
            ("debug",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (UID AAAA) is back after 6s offline and stayed "
             "bound; its loaded-lane follower restore engaged nothing."),
        ]
        assert self._console(master) == []

    def test_a_clean_sustained_offline_drops(self, tmp_path, monkeypatch):
        master, bridge, bay = self._claimed(tmp_path, monkeypatch,
                                            auto_drop=True)
        self._reads(master, bridge,
                    [(t, False) for t in (1.0, 4.0, 7.0, 10.0)])
        assert bay["bound"] == A_p5
        self._reads(master, bridge, [(13.0, False)])
        assert bay["bound"] is None
        assert master._released_at == {A_p5: 13.0}
        assert (master._last_online, master._online_run) == ({}, {})
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
        ]
        assert self._console(master) == self.REMOVED_AAAA
        # Re-plugged, it reclaims after flap_claim_grace; its first offline
        # read starts a fresh clock rather than one left from before.
        self._log(master).clear()
        self._reads(master, bridge, [(float(t), True) for t in range(14, 30)])
        assert bay["bound"] == A_p5
        self._reads(master, bridge, [(30.0, False)])
        assert bay["bound"] == A_p5
        assert master._last_online == {A_p5: 30.0}
        assert self._log(master) == self._claim_log(A_p5, "Bambu_AMS_1")
        assert self._console(master) == self.REMOVED_AAAA

    def test_an_outage_pauses_the_release_clock(self, tmp_path, monkeypatch):
        master, bridge, bay = self._claimed(tmp_path, monkeypatch,
                                            auto_drop=True)
        self._reads(master, bridge, [(1.0, False), (4.0, False)])
        assert master._last_online == {A_p5: 1.0}
        # The link drops; the last status still reads it offline.
        bridge._serial = None
        self._reads(master, bridge,
                    [(t, False) for t in (7.0, 10.0, 13.0, 20.0)])
        assert bay["bound"] == A_p5
        assert master._last_online == {A_p5: 20.0}
        # Back up: the whole release_grace counts from the last outage tick.
        bridge._serial = object()
        self._reads(master, bridge, [(21.0, False), (29.0, False)])
        assert bay["bound"] == A_p5
        self._reads(master, bridge, [(30.0, False)])
        assert bay["bound"] is None
        assert master._released_at == {A_p5: 30.0}
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
        ]
        assert self._console(master) == self.REMOVED_AAAA

    def test_an_outage_leaves_a_unit_online_before_it_bound(
            self, tmp_path, monkeypatch):
        master, bridge, bay = self._claimed(tmp_path, monkeypatch,
                                            auto_drop=True)
        self._reads(master, bridge, [(1.0, True), (6.0, True)])
        bridge._serial = None
        self._reads(master, bridge, [(t, True) for t in (9.0, 15.0, 25.0)])
        assert bay["bound"] == A_p5
        assert master._last_online == {A_p5: 25.0}
        assert master._online_since == {}
        bridge._serial = object()
        self._reads(master, bridge, [(26.0, True), (31.0, True)])
        assert bay["bound"] == A_p5
        assert master._online_since == {A_p5: 26.0}
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_reclaim_onto_the_slot_it_kept_offers_no_bay(
            self, tmp_path, monkeypatch):
        # Its saved name is dropped, so only the slot it still owns marks it
        # as coming back rather than new.
        master, bridge, bay = self._claimed(tmp_path, monkeypatch,
                                            auto_drop=True)
        self._reads(master, bridge, [(t, False) for t in (1.0, 6.0, 11.0)])
        assert (bay["bound"], bay["uid"]) == (None, A_p5)
        del master._name_map[A_p5]
        self._reads(master, bridge, [(float(t), True) for t in range(12, 27)])
        assert bay["bound"] is None
        self._reads(master, bridge, [(27.0, True)])
        assert bay["bound"] == A_p5
        self._reads(master, bridge, [(float(t), True) for t in range(28, 60)])
        assert master._popup_queue == []
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            ("info",
             "AFC_BridgeBox chain1: saved AAAA on Bambu_AMS_1 (lane24-lane27, T24-T27); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self.REMOVED_AAAA

    # ── a settled return restores the follower ────────────────────────────

    def _returning(self, tmp_path: pathlib.Path,
                   monkeypatch: pytest.MonkeyPatch, loaded: bool = True,
                   **options: Any) -> Tuple[afcBridgeBox, FakeBridge, Any]:
        """
        A unit claimed at 0 whose claim-time follower restore ran at 9,
        with AFC recording its lane24 loaded to the toolhead when
        ``loaded``; the setup's lines and sends cleared.

        :return tuple: (master, bridge, unit)
        """
        master, bridge, _bay = self._claimed(tmp_path, monkeypatch,
                                             **options)
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        master.printer.reactor.run_callbacks(until=9.0)
        assert (unit._id_resolved, unit._loaded_restore_at) == (True, 9.0)
        if loaded:
            master.printer.afc.tools["extruder"].lane_loaded = "lane24"
            master.printer.lookup_object("AFC_lane lane24").tool_loaded = True
        self._log(master).clear()
        bridge.sent.clear()
        return master, bridge, unit

    @staticmethod
    def _spy_restore(monkeypatch: pytest.MonkeyPatch, unit: Any) -> List[float]:
        """
        Record the ``since`` each return hands the unit's follower restore,
        passing every call on to the real method.

        :return list: each call's ``since``, in order
        """
        calls: List[float] = []
        real = unit.restore_follower_on_return

        def _restore(since: float) -> bool:
            calls.append(since)
            return real(since)
        monkeypatch.setattr(unit, "restore_follower_on_return", _restore)
        return calls

    #: What the unit sends when it engages the follower for lane24.
    ENGAGE = [{"cmd": "raw", "hex": "3DC50CC803000900A502800C"},
              {"cmd": "raw", "hex": "3DC50CC8030007000002514C"},
              {"cmd": "raw", "hex": "3DC50CC8030007007F023654"},
              {"cmd": "select", "unit": 0, "slot": 0},
              {"cmd": "assist", "unit": 0, "slot": 0, "on": True}]

    def test_a_settled_return_after_an_absence_restores_once(
            self, tmp_path, monkeypatch):
        master, bridge, unit = self._returning(tmp_path, monkeypatch)
        calls = self._spy_restore(monkeypatch, unit)
        self._reads(master, bridge, [(20.0, True)]
                    + [(t, False) for t in (21.0, 22.0, 23.0, 24.0)]
                    + [(t, True) for t in (25.0, 27.0, 29.0)])
        assert unit._loaded_restore_at == 9.0
        assert self._sent(bridge) == []
        assert calls == []
        self._reads(master, bridge, [(30.0, True)])
        assert unit._loaded_restore_at == 30.0
        assert self._sent(bridge) == self.ENGAGE
        # The settled run began at 25, not at the tick that saw it settle.
        assert calls == [25.0]
        self._reads(master, bridge, [(t, True) for t in (31.0, 40.0, 80.0)])
        assert unit._loaded_restore_at == 30.0
        assert self._sent(bridge) == self.ENGAGE
        assert calls == [25.0]
        assert self._bay(master, "Bambu_AMS_1")["bound"] == A_p5
        assert self._log(master) == [
            ("debug",
             "AFC bambu Bambu_AMS_1: lane24 loaded at startup, re-asserting AMS loaded state + "
             "follower"),
            ("info",
             "AFC bambu Bambu_AMS_1: restored the follower for lane24 (AFC records it loaded to "
             "the toolhead) -- mode:4, one-shot."),
            ("debug",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (UID AAAA) is back after 3s offline and stayed "
             "bound; its loaded-lane follower restore engaged."),
        ]
        assert self._console(master) == []

    def test_a_return_with_nothing_to_engage_stays_off_the_console(
            self, tmp_path, monkeypatch):
        master, bridge, unit = self._returning(tmp_path, monkeypatch,
                                               loaded=False)
        calls = self._spy_restore(monkeypatch, unit)
        self._reads(master, bridge, [(20.0, True)]
                    + [(t, False) for t in (21.0, 22.0, 23.0, 24.0)]
                    + [(t, True) for t in (25.0, 30.0)])
        assert unit._loaded_restore_at == 30.0
        assert calls == [25.0]
        assert self._sent(bridge) == []
        assert self._log(master) == [
            ("debug",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (UID AAAA) is back after 3s offline and stayed "
             "bound; its loaded-lane follower restore engaged nothing."),
        ]
        assert self._console(master) == []

    def test_a_lone_blip_is_not_a_return(self, tmp_path, monkeypatch):
        master, bridge, unit = self._returning(tmp_path, monkeypatch)
        calls = self._spy_restore(monkeypatch, unit)
        self._reads(master, bridge, [(20.0, True), (21.0, False)]
                    + [(float(t), True) for t in range(22, 40)])
        assert unit._loaded_restore_at == 9.0
        assert calls == []
        assert self._sent(bridge) == []
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_short_dropout_is_not_a_return(self, tmp_path, monkeypatch):
        master, bridge, unit = self._returning(tmp_path, monkeypatch)
        calls = self._spy_restore(monkeypatch, unit)
        self._reads(master, bridge,
                    [(20.0, True), (21.0, False), (22.0, False)]
                    + [(float(t), True) for t in range(23, 40)])
        assert unit._loaded_restore_at == 9.0
        assert calls == []
        assert self._sent(bridge) == []
        assert self._log(master) == []
        assert self._console(master) == []

    def test_a_flapping_absence_restores_only_on_the_settled_return(
            self, tmp_path, monkeypatch):
        master, bridge, unit = self._returning(tmp_path, monkeypatch)
        calls = self._spy_restore(monkeypatch, unit)
        flap = [(float(t), t % 2 == 0) for t in range(21, 50)]
        self._reads(master, bridge, [(20.0, True)] + flap)
        assert unit._loaded_restore_at == 9.0
        assert self._sent(bridge) == []
        assert calls == []
        self._reads(master, bridge, [(float(t), True) for t in range(50, 60)])
        assert unit._loaded_restore_at == 55.0
        assert calls == [50.0]
        assert self._sent(bridge) == self.ENGAGE
        assert self._log(master) == [
            ("debug",
             "AFC bambu Bambu_AMS_1: lane24 loaded at startup, re-asserting AMS loaded state + "
             "follower"),
            ("info",
             "AFC bambu Bambu_AMS_1: restored the follower for lane24 (AFC records it loaded to "
             "the toolhead) -- mode:4, one-shot."),
            ("debug",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (UID AAAA) is back after 28s offline and stayed "
             "bound; its loaded-lane follower restore engaged."),
        ]
        assert self._console(master) == []

    def test_a_return_mid_print_still_restores(self, tmp_path, monkeypatch):
        master, bridge, unit = self._returning(tmp_path, monkeypatch)
        master.printer.set_print_state("printing")
        calls = self._spy_restore(monkeypatch, unit)
        self._reads(master, bridge, [(20.0, True)]
                    + [(t, False) for t in (21.0, 22.0, 23.0, 24.0)]
                    + [(t, True) for t in (25.0, 30.0)])
        assert unit._loaded_restore_at == 30.0
        assert calls == [25.0]
        assert self._sent(bridge) == self.ENGAGE
        assert self._log(master) == [
            ("debug",
             "AFC bambu Bambu_AMS_1: lane24 loaded at startup, re-asserting AMS loaded state + "
             "follower"),
            ("info",
             "AFC bambu Bambu_AMS_1: restored the follower for lane24 (AFC records it loaded to "
             "the toolhead) -- mode:4, one-shot."),
            ("debug",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (UID AAAA) is back after 3s offline and stayed "
             "bound; its loaded-lane follower restore engaged."),
        ]
        assert self._console(master) == []

    def test_a_released_unit_leaves_the_restore_to_its_reclaim(
            self, tmp_path, monkeypatch):
        master, bridge, unit = self._returning(tmp_path, monkeypatch,
                                               auto_drop=True,
                                               loaded=False)
        bay = self._bay(master, "Bambu_AMS_1")
        calls = self._spy_restore(monkeypatch, unit)
        self._reads(master, bridge,
                    [(20.0, True)] + [(float(t), False) for t in range(21, 33)])
        assert bay["bound"] is None
        assert unit.pool is True
        self._reads(master, bridge, [(float(t), True) for t in range(33, 80)])
        assert bay["bound"] == A_p5
        assert unit.pool is False
        assert calls == []
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
            *self._claim_log(A_p5, "Bambu_AMS_1"),
        ]
        assert self._console(master) == self.REMOVED_AAAA

    # ── every claim resolves the claimed model ────────────────────────────

    @pytest.mark.parametrize("recorded", [None, "boxed"])
    def test_the_roster_option_names_the_claimed_model(
            self, tmp_path, monkeypatch, recorded):
        bridge = FakeBridge(uids=[A24], online=[True])
        state = {"roster": f"{recorded}:{A24}"} if recorded else None
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             state=state, roster=f"ams2:{A24}", pool_ams=1)
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        self._tick(master, 100.0)
        assert (unit.unit_uid, unit.pool) == (A24, False)
        assert (unit.ams_model, unit.has_heater) == ("ams2", True)
        assert self._log(master) == [
            *self._claim_log(A24, "Bambu_AMS_1", "ams2"),
        ]
        assert self._console(master) == []

    def test_a_boxed_option_entry_takes_the_recorded_generation(
            self, tmp_path, monkeypatch):
        bridge = FakeBridge(uids=[A24], online=[True])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=f"ams2:{A24}", roster=f"boxed:{A24}",
                             pool_ams=1)
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        assert unit.ams_model == "boxed"
        self._tick(master, 100.0)
        assert (unit.unit_uid, unit.pool) == (A24, False)
        assert (unit.ams_model, unit.has_heater) == ("ams2", True)
        assert self._log(master) == [
            *self._claim_log(A24, "Bambu_AMS_1", "ams2"),
        ]
        assert self._console(master) == []

    def test_a_refinement_after_the_claim_lands_and_the_reclaim_keeps_it(
            self, tmp_path, monkeypatch):
        bridge = self._dialect(FakeBridge(uids=[A24], online=[True]), 0b1,
                               [5])
        sections = {"AFC_BridgeBox ams2": {"heater": "False",
                                           "dry_max_temp": "50"}}
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=f"boxed:{A24}", roster=f"boxed:{A24}",
                             sections=sections, pool_ams=1)
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        self._tick(master, 100.0)
        assert self._roster(master) == f"ams2:{A24}"
        assert master.units[0]["model"] == "ams2"
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == (
            "ams2", False, 50)
        master._release_pool_unit(A24)
        assert unit.pool is True
        self._tick(master, 101.0)
        assert unit.pool is False
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == (
            "ams2", False, 50)
        assert self._log(master) == [
            *self._claim_log(A24, "Bambu_AMS_1"),
            ("info",
             "AFC_BridgeBox chain1: generation confirmed by bus dialect (0x3702): "
             f"{A24} -> ams2 -- applied to the running unit and recorded. "
             "Lanes and names do not move."),
            ("info",
             f"AFC_BridgeBox chain1: released Bambu_AMS_1 (UID {A24} offline "
             ">10s); lanes dropped live, slot kept for re-plug"),
            *self._claim_log(A24, "Bambu_AMS_1", "ams2"),
        ]
        assert self._console(master) == []

    # ── a claim during a print holds the binding table's save ─────────────

    def _spare_claim(self, tmp_path: pathlib.Path,
                     monkeypatch: pytest.MonkeyPatch, state: str
                     ) -> Tuple[afcBridgeBox, FakeBridge]:
        """
        A new AMS claimed onto the spare by a tick at 100 with print_stats
        at ``state``, its chain index resolved after.

        :return tuple: (master, bridge)
        """
        bridge = FakeBridge(uids=[A24], online=[True])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             roster=HT_ROSTER, pool_ams=1, print_state=state)
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        self._tick(master, 100.0)
        assert unit.pool is False
        master.printer.reactor.run_callbacks(until=100.6)
        assert unit._id_resolved is True
        return master, bridge

    def test_a_claim_mid_print_saves_nothing_until_the_print_ends(
            self, tmp_path, monkeypatch):
        master, bridge = self._spare_claim(tmp_path, monkeypatch, "printing")
        # The claim binds in RAM and asks for no save.
        held = [
            {"cmd": "status"},
            {"cmd": "htunit", "unit": 0, "on": 0},
            {"cmd": "capen", "unit": 0, "on": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 1},
            {"cmd": "bind", "uid": HT_UID, "idx": 1, "m": 2},
            {"cmd": "model", "unit": 0, "m": 1},
            {"cmd": "mcaddr", "unit": 0, "addr": 1792, "pay": 0},
            {"cmd": "units", "n": 1},
            {"cmd": "htunit", "unit": 0, "on": 0},
            {"cmd": "capen", "unit": 0, "on": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 1},
            {"cmd": "bind", "uid": HT_UID, "idx": 1, "m": 2},
            {"cmd": "model", "unit": 0, "m": 1},
            {"cmd": "mcaddr", "unit": 0, "addr": 1792, "pay": 0},
            {"cmd": "armms", "ms": 600000},
        ]
        assert self._sent(bridge) == held
        self._tick(master, 101.0)
        assert self._sent(bridge) == held
        master.printer.set_print_state("complete")
        bridge.sent.clear()
        self._tick(master, 102.0)
        assert self._sent(bridge) == [
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 1},
            {"cmd": "bind", "uid": HT_UID, "idx": 1, "m": 2},
            {"cmd": "idsave"},
        ]
        bridge.sent.clear()
        self._tick(master, 103.0)
        assert self._sent(bridge) == []
        assert self._log(master) == [
            *self._claim_log(A24, "Bambu_AMS_1"),
            ("debug", f"AFC bambu Bambu_AMS_1: UID {A24} confirmed at ams_index 0"),
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 bound in the bridge's RAM; the table is saved "
             "when the print ends (no flash write mid-print)."),
        ]
        assert self._console(master) == []

    def test_a_claim_when_idle_saves_straight_away(self, tmp_path,
                                                   monkeypatch):
        master, bridge = self._spare_claim(tmp_path, monkeypatch, "standby")
        assert self._sent(bridge) == [
            {"cmd": "status"},
            {"cmd": "htunit", "unit": 0, "on": 0},
            {"cmd": "capen", "unit": 0, "on": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 1},
            {"cmd": "bind", "uid": HT_UID, "idx": 1, "m": 2},
            {"cmd": "idsave"},
            {"cmd": "model", "unit": 0, "m": 1},
            {"cmd": "mcaddr", "unit": 0, "addr": 1792, "pay": 0},
            {"cmd": "units", "n": 1},
            {"cmd": "htunit", "unit": 0, "on": 0},
            {"cmd": "capen", "unit": 0, "on": 0},
            {"cmd": "bind", "uid": A24, "idx": 0, "m": 1},
            {"cmd": "bind", "uid": HT_UID, "idx": 1, "m": 2},
            {"cmd": "idsave"},
            {"cmd": "model", "unit": 0, "m": 1},
            {"cmd": "mcaddr", "unit": 0, "addr": 1792, "pay": 0},
            {"cmd": "armms", "ms": 600000},
        ]
        bridge.sent.clear()
        self._tick(master, 101.0)
        assert self._sent(bridge) == []
        assert self._log(master) == [
            *self._claim_log(A24, "Bambu_AMS_1"),
            ("debug", f"AFC bambu Bambu_AMS_1: UID {A24} confirmed at ams_index 0"),
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_1"),
            ("respond_raw",
             f"// action:prompt_text UID {A24} is on 'Bambu_AMS_1' (its T# "
             "and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text roster: is set and does not list it. Add "
             f"boxed:{A24} to roster: to pin it to a named bay."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]

    # ── an AMS with no free bay ───────────────────────────────────────────

    #: Four AMS and an HT, as a full chain records them.
    FOUR = f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{D}, ht:{H}"

    def _pooled(self, tmp_path: pathlib.Path,
                monkeypatch: pytest.MonkeyPatch, recorded: Optional[str],
                uids: Sequence[str], online: Sequence[str], htmask: int = 0,
                **options: Any) -> Tuple[afcBridgeBox, FakeBridge]:
        """
        A chain with four AMS bays and two HT bays booted from
        ``recorded``, watching a bridge that reads ``uids`` with those in
        ``online`` online.

        :param options: the master's options over pool_ams=4, pool_ht=2
        :return tuple: (master, bridge)
        """
        bridge = FakeBridge(uids=list(uids),
                            online=[u in online for u in uids],
                            htmask=htmask)
        opts: Dict[str, Any] = dict(pool_ams=4, pool_ht=2)
        opts.update(options)
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=recorded, **opts)
        return master, bridge

    @staticmethod
    def _restart(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                 **options: Any) -> afcBridgeBox:
        """
        The next boot from what the chain recorded, through klippy:ready.

        :param options: the master's options over roster="", pool_ams=4,
            pool_ht=2
        :return afcBridgeBox: the restarted master; its ready notes are in
            its logger
        """
        opts: Dict[str, Any] = dict(roster="", pool_ams=4, pool_ht=2)
        opts.update(options)
        return make_bridgebox(tmp_path,
                              printer=make_printer(monkeypatch=monkeypatch),
                              ready=True, **opts)

    @staticmethod
    def _uids(master: afcBridgeBox) -> Dict[str, str]:
        """:return dict: fabricated unit name -> its unit_uid, "" if spare"""
        printer = master.printer
        return {section.split(" ", 1)[1]:
                printer.lookup_object(section).keys.get("unit_uid", "")
                for section, _config in printer.loaded
                if section.startswith("AFC_BambuAMS ")}

    @staticmethod
    def _bound(master: afcBridgeBox, family: str = "ams"
               ) -> Dict[str, Optional[str]]:
        """:return dict: bay name -> the uid bound there, for one family"""
        return {pu["name"]: pu.get("bound") for pu in master._pool_units
                if pu["family"] == family}

    def test_two_new_ams_and_one_spare_the_second_is_told_to_forget(
            self, tmp_path, monkeypatch):
        master, bridge = self._pooled(
            tmp_path, monkeypatch,
            f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}",
            uids=(A_p5, B_p5, C_p5, D_p5, E_p5, H_p5), online=(A_p5, B_p5, D_p5, H_p5),
            htmask=1 << 5)
        self._tick(master, 100.0)
        assert self._bound(master) == {"Bambu_AMS_1": A_p5, "Bambu_AMS_2": B_p5,
                                       "Bambu_AMS_3": None, "Bambu_AMS_4": D_p5}
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            *self._claim_log(B_p5, "Bambu_AMS_2"),
            *self._claim_log(H_p5, "Bambu_AMS_HT_1", "ht", 1),
            *self._claim_log(D_p5, "Bambu_AMS_4"),
        ]
        self._log(master).clear()
        self._online(bridge, [True, True, False, True, True, True])
        self._tick(master, 105.0)
        assert master._bay_of_uid(E_p5) is None
        assert self._log(master) == [
            ("warning",
             "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays belong to other units "
             "(Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), Bambu_AMS_4 (DDDD)), "
             "and a Bambu bus addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
             "one. Bambu_AMS_3 (CCCC) is offline: if this AMS replaces it, AFC_BRIDGEBOX_FORGET "
             "CHAIN=chain1 UID=CCCC frees that bay and this AMS claims it live. "
             "AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE OLD=Bambu_AMS_3 does both in one step."),
        ]
        self._log(master).clear()
        self._tick(master, *range(106, 122))
        assert master._name_map[D_p5] == "Bambu_AMS_4"
        assert self._roster(master) == ("boxed:AAAA, boxed:BBBB, boxed:CCCC, "
                                        "ht:HHHH, boxed:DDDD, boxed:EEEE")
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:DDDD -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved DDDD on Bambu_AMS_4 (lane36-lane39, T36-T39); it "
             "comes back there after a restart."),
            self.NEW_E,
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_4"),
            ("respond_raw",
             "// action:prompt_text UID DDDD is on 'Bambu_AMS_4' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text It is saved on this bay once it has been online 15s."),
            ("respond_raw", "// action:prompt_text No other free bay of this type to move it to."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
            *self._offer_popup(20, bay="Bambu_AMS_3", old=C_p5, lanes="lane32-lane35 (T32-T35)"),
        ]
        again = self._restart(tmp_path, monkeypatch)
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "BBBB",
            "Bambu_AMS_3": "CCCC",
            "Bambu_AMS_4": "DDDD",
            "Bambu_AMS_HT_1": "HHHH",
            "Bambu_AMS_HT_2": "",
        }
        assert self._log(again) == [
            ("warning",
             "AFC_BridgeBox chain1: AMS EEEE is recorded but has no bay: all 4 AMS bays belong "
             "to other units (Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), "
             "Bambu_AMS_4 (DDDD)), and a Bambu bus addresses at most 4 AMS, so neither pool_ams "
             "nor RESTART adds one. If this AMS replaces one of them, AFC_BRIDGEBOX_FORGET "
             "CHAIN=chain1 UID=<that unit's uid> frees that bay and this AMS claims it live."),
        ]

    def _waiting(self, tmp_path: pathlib.Path,
                 monkeypatch: pytest.MonkeyPatch,
                 online: Sequence[str] = (A_p5, B_p5, C_p5, E_p5)
                 ) -> Tuple[afcBridgeBox, FakeBridge]:
        """
        EEEE recorded beside four AMS and booted, so it waits with no bay,
        on a chain of A to E.

        :return tuple: (master, bridge)
        """
        master, bridge = self._pooled(tmp_path, monkeypatch,
                                      self.FOUR + f", boxed:{E}",
                                      uids=(A_p5, B_p5, C_p5, D_p5, E_p5), online=online)
        assert master._no_bay_told == {E_p5}
        return master, bridge

    @pytest.mark.parametrize("d_bound, erased, released", [
        (False, "", []),
        (True, ", saved lane records erased",
         [("info",
           "AFC_BridgeBox chain1: released Bambu_AMS_4 (UID DDDD, "
           "AFC_BRIDGEBOX_FORGET); lanes dropped live")]),
    ], ids=["forgotten-offline", "forgotten-still-bound"])
    def test_a_waiting_ams_claims_the_bay_forget_frees(
            self, tmp_path, monkeypatch, d_bound, erased, released):
        master, bridge = self._waiting(
            tmp_path, monkeypatch,
            online=(A_p5, B_p5, C_p5, D_p5, E_p5) if d_bound else (A_p5, B_p5, C_p5, E_p5))
        self._tick(master, 99.0)
        self._online(bridge, [True, True, True, False, True])
        self._tick(master, 100.0, 101.0)
        # auto_drop is off: a claimed D stays bound once unplugged.
        assert self._bound(master)["Bambu_AMS_4"] == (D_p5 if d_bound else None)
        # The boot told E it has no bay; the watch does not say it again.
        assert self._log(master) == (
            self._claim_log(A_p5, "Bambu_AMS_1") + self._claim_log(B_p5, "Bambu_AMS_2")
            + self._claim_log(C_p5, "Bambu_AMS_3")
            + (self._claim_log(D_p5, "Bambu_AMS_4") if d_bound else []))
        assert self._console(master) == []
        self._log(master).clear()
        gcmd = master.printer.gcode.run("AFC_BRIDGEBOX_FORGET",
                                        CHAIN="chain1", UID=D_p5)
        assert gcmd.messages == [
            ("respond_info",
             "AFC_BridgeBox chain1: forgot DDDD -- lanes 36-39 and the name "
             f"Bambu_AMS_4 freed for reuse{erased} -- slot freed to the pool "
             "LIVE; the next same-family unit claims it with no reboot."),
        ]
        self._tick(master, 102.0)
        assert self._bound(master) == {"Bambu_AMS_1": A_p5, "Bambu_AMS_2": B_p5,
                                       "Bambu_AMS_3": C_p5, "Bambu_AMS_4": E_p5}
        assert self._log(master) == released + self._claim_log(
            E_p5, "Bambu_AMS_4")
        assert self._console(master) == [
            *self.PROMPT_END, *self._new_popup(E_p5, "Bambu_AMS_4")]
        again = self._restart(tmp_path, monkeypatch)
        assert self._uids(again)["Bambu_AMS_4"] == E_p5
        assert again._layout_notes == []
        assert self._log(again) == []

    def test_a_bay_a_waiting_ams_claims_after_unassign_survives_a_restart(
            self, tmp_path, monkeypatch):
        master, _bridge = self._waiting(tmp_path, monkeypatch)
        gcmd = master.printer.gcode.run("AFC_BRIDGEBOX_UNASSIGN",
                                        CHAIN="chain1", UID=D_p5)
        assert gcmd.messages == [
            ("respond_info",
             "AFC_BridgeBox chain1: unassigned DDDD from bay 'Bambu_AMS_4' (learned values stay "
             "with the unit). It takes a free bay of its family, its last one first, and is "
             "saved there once it has been online 15s."),
        ]
        assert self._console(master) == self.PROMPT_END
        self._tick(master, 100.0)
        assert master._bay_of_uid(E_p5)["name"] == "Bambu_AMS_4"
        name_map = master._state_get(SEC, "name_map")
        assert name_map == ("AAAA:Bambu_AMS_1, BBBB:Bambu_AMS_2, "
                            "CCCC:Bambu_AMS_3, HHHH:Bambu_AMS_HT_1")
        self._tick(master, 115.0)
        name_map = master._state_get(SEC, "name_map")
        assert name_map == ("AAAA:Bambu_AMS_1, BBBB:Bambu_AMS_2, "
                            "CCCC:Bambu_AMS_3, EEEE:Bambu_AMS_4, "
                            "HHHH:Bambu_AMS_HT_1")
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            *self._claim_log(B_p5, "Bambu_AMS_2"),
            *self._claim_log(C_p5, "Bambu_AMS_3"),
            *self._claim_log(E_p5, "Bambu_AMS_4"),
            ("info",
             "AFC_BridgeBox chain1: saved EEEE on Bambu_AMS_4 (lane36-lane39, T36-T39); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == [
            *self.PROMPT_END, *self._new_popup(E_p5, "Bambu_AMS_4")]
        again = self._restart(tmp_path, monkeypatch)
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "BBBB",
            "Bambu_AMS_3": "CCCC",
            "Bambu_AMS_4": "EEEE",
            "Bambu_AMS_HT_1": "HHHH",
            "Bambu_AMS_HT_2": "",
        }
        assert self._log(again) == [
            ("warning",
             "AFC_BridgeBox chain1: AMS DDDD is recorded but has no bay: all 4 AMS bays belong "
             "to other units (Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), "
             "Bambu_AMS_4 (EEEE)), and a Bambu bus addresses at most 4 AMS, so neither pool_ams "
             "nor RESTART adds one. If this AMS replaces one of them, AFC_BRIDGEBOX_FORGET "
             "CHAIN=chain1 UID=<that unit's uid> frees that bay and this AMS claims it live."),
        ]

    def test_a_waiting_ams_that_returns_is_told_once(self, tmp_path,
                                                     monkeypatch):
        master, bridge = self._waiting(tmp_path, monkeypatch)
        self._tick(master, 100.0)
        self._online(bridge, [True, True, True, False, False])
        self._tick(master, 101.0)
        assert master._no_bay_told == set()
        self._online(bridge, [True, True, True, False, True])
        self._tick(master, 102.0, 103.0)
        assert master._no_bay_told == {E_p5}
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            *self._claim_log(B_p5, "Bambu_AMS_2"),
            *self._claim_log(C_p5, "Bambu_AMS_3"),
            self.TOLD_E,
        ]
        assert self._console(master) == []

    @pytest.mark.parametrize("dark", [[True], [False, True]],
                             ids=["before-the-first-status", "bridge-dropout"])
    def test_a_tick_with_nothing_online_does_not_retell_a_waiting_ams(
            self, tmp_path, monkeypatch, dark):
        master, bridge = self._waiting(tmp_path, monkeypatch)
        live = [True, True, True, False, True]
        t = 100.0
        for all_off in dark:
            self._online(bridge, [False] * 5 if all_off else live)
            self._tick(master, t)
            t += 1.0
        self._online(bridge, live)
        self._tick(master, t, t + 1.0)
        assert master._bay_of_uid(E_p5) is None
        assert master._no_bay_told == {E_p5}
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            *self._claim_log(B_p5, "Bambu_AMS_2"),
            *self._claim_log(C_p5, "Bambu_AMS_3"),
        ]
        assert self._console(master) == []

    @pytest.mark.parametrize("pooled", [True, False], ids=["pool", "no-pool"])
    def test_a_new_ams_with_every_bay_held_is_not_told_to_restart(
            self, tmp_path, monkeypatch, pooled):
        pool = {} if pooled else {"pool_ams": 0, "pool_ht": 0}
        master, _bridge = self._pooled(
            tmp_path, monkeypatch, self.FOUR, uids=(A_p5, B_p5, C_p5, E_p5, H_p5, G_p5),
            online=(A_p5, B_p5, C_p5, E_p5, H_p5, G_p5), htmask=0b110000, **pool)
        self._tick(master, *range(100, 116))
        assert self._roster(master) == self.FOUR + ", boxed:EEEE, ht:GGGG"
        told = (
            "all 4 AMS bays belong to other units (Bambu_AMS_1 (AAAA), "
            "Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), Bambu_AMS_4 (DDDD)), and a "
            "Bambu bus addresses at most 4 AMS, so neither pool_ams nor RESTART "
            "adds one. Bambu_AMS_4 (DDDD) is offline: if this AMS replaces it, "
            "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=DDDD frees that bay")
        if not pooled:
            assert self._log(master) == [
                ("info",
                 "AFC_BridgeBox chain1: Bambu_AMS_4 (DDDD) is offline on a live "
                 "chain -- if it stays gone 120s it will be recorded as removed "
                 "(takes effect at the next RESTART)."),
                ("info",
                 "AFC_BridgeBox chain1: NEW unit(s) on the chain: ht:GGGG -- "
                 "recorded. RESTART to enroll."),
                ("info",
                 "AFC_BridgeBox chain1: NEW AMS on the chain: boxed:EEEE -- "
                 f"recorded, but it has no bay: {told} for it at the next "
                 "RESTART. An AMS offline on a live chain for 120s is removed "
                 "from the recorded roster, which frees its bay at the next "
                 "RESTART too."),
            ]
            assert self._console(master) == []
            return
        assert self._console(master) == self._new_popup(G_p5, "Bambu_AMS_HT_2")
        assert self._log(master) == (
            self._claim_log(A_p5, "Bambu_AMS_1") + self._claim_log(B_p5, "Bambu_AMS_2")
            + self._claim_log(C_p5, "Bambu_AMS_3")
            + self._claim_log(H_p5, "Bambu_AMS_HT_1", "ht", 1)
            + [("warning",
                f"AFC_BridgeBox chain1: AMS EEEE has no bay: {told} and this AMS "
                "claims it live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE "
                "OLD=Bambu_AMS_4 does both in one step.")]
            + self._claim_log(G_p5, "Bambu_AMS_HT_2", "ht", 1)
            + [("info",
                "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:EEEE, "
                "ht:GGGG -- recorded. A restart adds its temperature card."),
               ("info",
                "AFC_BridgeBox chain1: saved GGGG on Bambu_AMS_HT_2 (lane41, "
                "T41); it comes back there after a restart.")])

    @pytest.mark.parametrize("pooled", [True, False], ids=["pool", "no-pool"])
    def test_with_roster_set_a_new_ams_with_every_bay_held_is_not_told_to_add_it(
            self, tmp_path, monkeypatch, pooled):
        pool = {} if pooled else {"pool_ams": 0, "pool_ht": 0}
        master, _bridge = self._pooled(
            tmp_path, monkeypatch, self.FOUR, uids=(A_p5, B_p5, C_p5, E_p5, H_p5),
            online=(A_p5, B_p5, C_p5, E_p5, H_p5), htmask=0b10000, roster=self.FOUR,
            **pool)
        self._tick(master, *range(100, 116))
        assert self._roster(master) == self.FOUR + ", boxed:EEEE"
        told = (
            "has no bay: all 4 AMS bays belong to other units (Bambu_AMS_1 "
            "(AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), Bambu_AMS_4 "
            "(DDDD)), and a Bambu bus addresses at most 4 AMS, so neither "
            "pool_ams nor RESTART adds one. Bambu_AMS_4 (DDDD) is offline: if "
            "this AMS replaces it, replace its entry in roster: with "
            "boxed:EEEE, run AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=DDDD, and "
            "RESTART.")
        if not pooled:
            assert self._log(master) == [
                ("info",
                 "AFC_BridgeBox chain1: NEW AMS on the chain: boxed:EEEE -- "
                 f"recorded, but it {told}"),
            ]
            assert self._console(master) == []
            return
        # The claim said it; the enrollment line only names it recorded.
        assert self._log(master) == (
            self._claim_log(A_p5, "Bambu_AMS_1") + self._claim_log(B_p5, "Bambu_AMS_2")
            + self._claim_log(C_p5, "Bambu_AMS_3")
            + self._claim_log(H_p5, "Bambu_AMS_HT_1", "ht", 1)
            + [("warning", f"AFC_BridgeBox chain1: AMS EEEE {told}"),
               self.NEW_E])
        assert self._console(master) == []

    def test_without_a_pool_a_new_ams_in_an_auto_removed_units_place_enrolls(
            self, tmp_path, monkeypatch):
        master, _bridge = self._pooled(
            tmp_path, monkeypatch, self.FOUR, uids=(A_p5, B_p5, C_p5, E_p5, H_p5),
            online=(A_p5, B_p5, C_p5, E_p5, H_p5), htmask=0b10000, pool_ams=0, pool_ht=0)
        record_chain_state(tmp_path,
                           roster=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        self._tick(master, 100.0, 115.0)
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:EEEE -- recorded. RESTART "
             "to enroll."),
        ]
        again = self._restart(tmp_path, monkeypatch, pool_ams=0, pool_ht=0)
        assert self._uids(again)["Bambu_AMS_4"] == E_p5
        assert self._log(again) == []
        assert self._console(master) == []

    # ── a loaded lane that changes unit at boot ──────────────────────────

    def test_a_record_on_a_lane_no_bay_holds_is_told_too(self, tmp_path,
                                                         monkeypatch):
        # EEEE was saved on Bambu_AMS_5 (lane40-lane43) and the HT on
        # lane44: this boot gives lane40 to the HT and lane44 to no bay.
        master = self._chain(
            tmp_path, monkeypatch, recorded=self.FOUR + f", boxed:{E}",
            state={"name_map": (f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                                f"{C}:Bambu_AMS_3, {D}:Bambu_AMS_4, "
                                f"{E}:Bambu_AMS_5, {H}:Bambu_AMS_HT_1"),
                   "lane_map": (f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, "
                                f"{E}:40:4, {H}:44:1")},
            pool_ams=4, pool_ht=2)
        master.printer.afc.tools["extruder"].lane_loaded = "lane44"
        # Nothing is said until PREP has restored the record.
        master.printer.afc.prep_done = False
        assert self._tick(master, 97.0) == 100.0
        assert self._log(master) == []
        assert master._lane_moves["lane44"]["uid"] == H_p5
        master.printer.afc.prep_done = True
        # No bridge yet: the check runs ahead of the chain read.
        assert self._tick(master, 100.0) == 103.0
        assert self._log(master) == [
            ("warning",
             "AFC_BridgeBox chain1: extruder records lane44 as loaded, but lane44 belonged to "
             "HT HHHH (Bambu_AMS_HT_1), and this boot's layout gives it to no bay. The filament "
             "in extruder is from HHHH. Unload that filament by hand and run UNSET_LANE_LOADED "
             "to clear the record."),
        ]
        self._tick(master, 103.0)
        assert self._log(master) == [
            ("warning",
             "AFC_BridgeBox chain1: extruder records lane44 as loaded, but lane44 belonged to "
             "HT HHHH (Bambu_AMS_HT_1), and this boot's layout gives it to no bay. The filament "
             "in extruder is from HHHH. Unload that filament by hand and run UNSET_LANE_LOADED "
             "to clear the record."),
        ]
        assert self._console(master) == []

    # ── claims wait for AFC's PREP ────────────────────────────────────────

    def _prep_chain(self, tmp_path: pathlib.Path,
                    monkeypatch: pytest.MonkeyPatch,
                    online: Sequence[str] = (A_p5, C_p5),
                    owners: Optional[str] = None,
                    moonraker: Optional[str] = None,
                    var: Optional[Dict[str, Any]] = None,
                    ready_log: Sequence[LogLine] = ()) -> afcBridgeBox:
        """
        boxed:AAAA recorded, three named AMS bays and one HT bay, readied at
        0 with AFC's PREP not run yet, watching ``online`` on the chain.

        :param owners: the bay_owner an earlier boot recorded
        :param moonraker: AFC's moonraker_connect_to
        :param var: AFC.var.unit as the last session saved it
        :param ready_log: what klippy:ready logs
        :return afcBridgeBox: the master, its ready lines cleared
        """
        bridge = FakeBridge(uids=list(online), online=[True] * len(online))
        state = {"bay_owner": owners} if owners else None
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=f"boxed:{A}", state=state, pool_ams=3,
                             pool_ht=1, ams_names="Alpha, Bravo, Charlie",
                             ht_names="Hot")
        afc = master.printer.afc
        if moonraker is not None:
            afc.moonraker_connect_to = moonraker
        if var is not None:
            write_unit_vars(master.printer, var)
        master.printer.reactor.now = 0.0
        master._scout_ready()
        assert self._log(master) == list(ready_log)
        afc.prep_done = False
        self._log(master).clear()
        return master

    def test_nothing_is_claimed_or_adopted_before_prep(self, tmp_path,
                                                       monkeypatch):
        master = self._prep_chain(tmp_path, monkeypatch)
        self._tick(master, 10.0, 89.0)
        assert [pu["bound"] for pu in master._pool_units] == [None] * 4
        assert self._bay(master, "Bravo")["uid"] is None
        assert self._log(master) == [
            ("debug", "AFC_BridgeBox chain1: unit claims wait for PREP"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. CCCC has "
             "no bay yet and takes the next free one of its family."),
        ]
        assert self._console(master) == []

    def test_units_claim_once_prep_has_run(self, tmp_path, monkeypatch):
        master = self._prep_chain(tmp_path, monkeypatch)
        self._tick(master, 10.0)
        master.printer.afc.prep_done = True
        self._tick(master, 13.0)
        assert self._bay(master, "Alpha")["bound"] == A_p5
        assert self._bay(master, "Bravo")["bound"] == C_p5
        assert self._log(master) == [
            ("debug", "AFC_BridgeBox chain1: unit claims wait for PREP"),
            *self._claim_log(A_p5, "Alpha"),
            *self._claim_log(C_p5, "Bravo"),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bravo", "Charlie")

    def test_a_prep_that_never_finishes_holds_claims_for_90s_only(
            self, tmp_path, monkeypatch):
        master = self._prep_chain(tmp_path, monkeypatch)
        self._tick(master, 89.0)
        assert self._bay(master, "Alpha")["bound"] is None
        self._tick(master, 90.0, 93.0)
        assert self._bay(master, "Alpha")["bound"] == A_p5
        assert self._log(master) == [
            ("debug", "AFC_BridgeBox chain1: unit claims wait for PREP"),
            ("warning",
             "AFC_BridgeBox chain1: PREP has not finished 90s after startup; claiming units "
             "anyway."),
            *self._claim_log(A_p5, "Alpha"),
            *self._claim_log(C_p5, "Bravo"),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bravo", "Charlie")

    def test_a_long_moonraker_timeout_holds_claims_longer(self, tmp_path,
                                                           monkeypatch):
        master = self._prep_chain(tmp_path, monkeypatch, moonraker="75")
        self._tick(master, 100.0, 134.0)
        assert self._bay(master, "Alpha")["bound"] is None
        self._tick(master, 135.0, 138.0)
        assert self._bay(master, "Alpha")["bound"] == A_p5
        assert self._log(master) == [
            ("debug", "AFC_BridgeBox chain1: unit claims wait for PREP"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. CCCC has "
             "no bay yet and takes the next free one of its family."),
            ("warning",
             "AFC_BridgeBox chain1: PREP has not finished 135s after startup; claiming units "
             "anyway."),
            *self._claim_log(A_p5, "Alpha"),
            *self._claim_log(C_p5, "Bravo"),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bravo (lane28-lane31, T28-T31); it comes back "
             "there after a restart."),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bravo", "Charlie",
                                                        saved=True)

    def test_the_owner_is_written_once_prep_has_run(self, tmp_path,
                                                    monkeypatch):
        bravo = {"Bravo": {"lane28": {"map": "T28", "current_map": "T28",
                                      "spool_id": 159, "material": "PLA"}}}
        master = self._prep_chain(
            tmp_path, monkeypatch, online=(D_p5,), owners="EEEE:Bravo", var=bravo,
            ready_log=[("debug",
                        "AFC_BridgeBox chain1: holding the saved lane records "
                        "of Bravo for their units")])
        self._tick(master, 91.0)
        assert self._bay(master, "Bravo")["bound"] == D_p5
        assert master._owners() == {"Bravo": D_p5}
        assert master._bay_owner_pending is True
        self._tick(master, 92.0)
        assert master._state_get(SEC, "bay_owner") == "EEEE:Bravo"
        master.printer.afc.prep_done = True
        self._tick(master, 95.0)
        assert master._state_get(SEC, "bay_owner") == "DDDD:Bravo"
        assert master._bay_owner_pending is False
        assert self._log(master) == [
            ("warning",
             "AFC_BridgeBox chain1: PREP has not finished 90s after startup; claiming units "
             "anyway."),
            *self._claim_log(D_p5, "Bravo"),
        ]
        assert self._console(master) == self._new_popup(D_p5, "Bravo", "Charlie")

    def test_it_claims_before_a_new_unit_seen_on_the_same_tick(
            self, tmp_path, monkeypatch):
        # ZZZZ sorts after CCCC by uid, so only its last bay puts it first.
        master = self._prep_chain(tmp_path, monkeypatch, online=(C_p5, Z),
                                  owners="ZZZZ:Bravo")
        master.printer.afc.prep_done = True
        self._tick(master, 0.0)
        assert (self._bay(master, "Bravo")["bound"],
                self._bay(master, "Charlie")["bound"]) == (Z, C_p5)
        assert self._log(master) == [
            *self._claim_log(Z, "Bravo"),
            *self._claim_log(C_p5, "Charlie"),
        ]
        # One popup shows per hold; CCCC's waits its turn.
        assert self._console(master) == self._new_popup(Z, "Bravo")
        assert master._popup_queue == [("new", C_p5)]

    # ── a new unit takes over the bay of an offline one ──────────────────

    #: The chain _stuck reads, HHHH at index 6.
    CHAIN = (A_p5, B_p5, C_p5, D_p5, E_p5, G_p5, H_p5)

    def _stuck(self, tmp_path: pathlib.Path,
               monkeypatch: pytest.MonkeyPatch,
               online: Sequence[str] = (A_p5, B_p5, C_p5, E_p5, H_p5),
               recorded: Optional[str] = None,
               state: Optional[Dict[str, str]] = None, htmask: int = 1 << 6,
               **options: Any) -> Tuple[afcBridgeBox, FakeBridge]:
        """
        Four AMS and an HT recorded (``recorded``, FOUR by default) and on
        their bays but DDDD, which is unplugged; EEEE is plugged in and
        finds no free AMS bay.

        :param online: the uids of CHAIN online
        :param state: further state keys an earlier boot left
        :param htmask: the chain indices that answer as an HT
        :param options: the master's options over pool_ams=4, pool_ht=2
        :return tuple: (master, bridge)
        """
        return self._pooled(tmp_path, monkeypatch,
                            self.FOUR if recorded is None else recorded,
                            uids=self.CHAIN, online=online, htmask=htmask,
                            state=state, **options)

    def _run(self, master: afcBridgeBox, first: int, last: int) -> None:
        """Run the watch once a second from ``first`` to ``last``."""
        self._tick(master, *range(first, last + 1))

    @staticmethod
    def _set_online(bridge: FakeBridge, uid: str, on: bool) -> None:
        """Flip one unit of CHAIN's online flag."""
        index = TestAfcBridgeBoxScoutTick.CHAIN.index(uid)
        bridge.status["units"][index]["online"] = on

    def _first_claims(self) -> List[LogLine]:
        """:return list: _stuck's first tick: A to C and the HT claimed"""
        return (self._claim_log(A_p5, "Bambu_AMS_1")
                + self._claim_log(B_p5, "Bambu_AMS_2")
                + self._claim_log(C_p5, "Bambu_AMS_3")
                + self._claim_log(H_p5, "Bambu_AMS_HT_1", "ht", 1))

    #: The no-bay line EEEE gets while DDDD's bay is held for it offline.
    TOLD_E = (
        "warning",
        "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays belong to "
        "other units (Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 "
        "(CCCC), Bambu_AMS_4 (DDDD)), and a Bambu bus addresses at most 4 AMS, "
        "so neither pool_ams nor RESTART adds one. Bambu_AMS_4 (DDDD) is "
        "offline: if this AMS replaces it, AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
        "UID=DDDD frees that bay and this AMS claims it live. "
        "AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE OLD=Bambu_AMS_4 does "
        "both in one step.")

    #: EEEE's enrollment, after enroll_grace.
    NEW_E = ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:EEEE -- "
             "recorded.")

    @staticmethod
    def _offer_popup(offline: int, waiting: str = E_p5,
                     bay: str = "Bambu_AMS_4", old: str = D_p5,
                     lanes: str = "lane36-lane39 (T36-T39)",
                     kind: str = "AMS") -> List[LogLine]:
        """
        :param offline: the seconds ``old`` has been offline
        :return list: the replace offer a new unit waiting for a bay gets
        """
        return [
            ("respond_raw", f"// action:prompt_begin No free bay for new {kind}"),
            ("respond_raw",
             f"// action:prompt_text UID {waiting} has no free bay: every "
             f"{kind} bay is taken."),
            ("respond_raw",
             "// action:prompt_text Replace a unit that is offline: the new "
             "one takes its bay, lanes and T# now, and the old one is "
             "forgotten (its learned values and saved lane records, spools "
             "included, are erased)."),
            ("respond_raw",
             f"// action:prompt_text {bay}: {old}, {lanes}, offline {offline}s"),
            ("respond_raw",
             "// action:prompt_text Dismiss leaves it waiting; "
             f"AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID={waiting} opens this "
             "again."),
            ("respond_raw",
             f"// action:prompt_button Replace {bay}|AFC_BRIDGEBOX_REPLACE "
             f"CHAIN=chain1 UID={waiting} OLD={old}|error"),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]

    def test_it_names_replace_beside_forget_once(self, tmp_path,
                                                 monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch)
        self._run(master, 100, 130)
        assert master.get_status()["waiting_for_bay"] == [E_p5]
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            self.NEW_E,
        ]
        assert self._console(master) == self._offer_popup(15)

    def test_with_every_holder_online_it_names_no_replace(self, tmp_path,
                                                          monkeypatch):
        master, bridge = self._stuck(tmp_path, monkeypatch,
                                     online=(A_p5, B_p5, C_p5, D_p5, E_p5, H_p5))
        self._run(master, 100, 130)
        assert master.get_status()["waiting_for_bay"] == [E_p5]
        assert self._console(master) == []
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            *self._claim_log(B_p5, "Bambu_AMS_2"),
            *self._claim_log(C_p5, "Bambu_AMS_3"),
            *self._claim_log(D_p5, "Bambu_AMS_4"),
            *self._claim_log(H_p5, "Bambu_AMS_HT_1", "ht", 1),
            ("warning",
             "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays belong to other units "
             "(Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), Bambu_AMS_4 (DDDD)), "
             "and a Bambu bus addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
             "one. If this AMS replaces one of them, AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
             "UID=<that unit's uid> frees that bay and this AMS claims it live."),
            self.NEW_E,
        ]
        self._log(master).clear()
        self._set_online(bridge, D_p5, False)
        self._run(master, 131, 140)
        assert self._console(master) == []
        self._run(master, 141, 141)
        assert self._console(master) == self._offer_popup(10)
        assert self._log(master) == []

    def test_with_roster_set_it_names_no_replace(self, tmp_path,
                                                 monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch, roster=self.FOUR)
        self._run(master, 100, 130)
        assert master._replace_offered == set()
        assert self._log(master) == [
            *self._first_claims(),
            ("warning",
             "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays belong to other units "
             "(Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), Bambu_AMS_4 (DDDD)), "
             "and a Bambu bus addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
             "one. Bambu_AMS_4 (DDDD) is offline: if this AMS replaces it, replace its entry in "
             "roster: with boxed:EEEE, run AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=DDDD, and "
             "RESTART."),
            self.NEW_E,
        ]
        assert self._console(master) == []

    def test_with_fewer_ams_bays_it_names_the_offline_one_first(
            self, tmp_path, monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch,
                                      online=(A_p5, E_p5, H_p5),
                                      recorded=f"boxed:{A}, boxed:{B}, ht:{H}",
                                      pool_ams=2)
        self._run(master, 100, 101)
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            *self._claim_log(H_p5, "Bambu_AMS_HT_1", "ht", 1),
            ("warning",
             "AFC_BridgeBox chain1: new AMS EEEE has no free bay: every AMS bay built belongs "
             "to a known unit. Bambu_AMS_2 (BBBB) is offline: if this AMS replaces it, "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=BBBB frees that bay and this AMS claims it "
             "live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE OLD=Bambu_AMS_2 does both in "
             "one step. Once it is recorded, RESTART builds it one, past the AMS band, which "
             "moves the HT lanes up 4; raise pool_ams to keep spare AMS bays for units plugged "
             "in live (at most 4 AMS)."),
        ]
        assert self._console(master) == []

    def test_an_ht_is_told_too(self, tmp_path, monkeypatch):
        # GGGG answers as an HT too.
        master, _bridge = self._stuck(tmp_path, monkeypatch, online=(A_p5, G_p5),
                                      recorded=f"boxed:{A}, ht:{H}",
                                      htmask=(1 << 6) | (1 << 5),
                                      pool_ams=1, pool_ht=1)
        self._run(master, 100, 101)
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            ("warning",
             "AFC_BridgeBox chain1: new HT GGGG has no free bay: every HT bay built belongs to "
             "a known unit. Bambu_AMS_HT_1 (HHHH) is offline: if this HT replaces it, "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=HHHH frees that bay and this HT claims it "
             "live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=GGGG OLD=Bambu_AMS_HT_1 does both in "
             "one step. Once it is recorded, RESTART builds it one; raise pool_ht to keep spare "
             "HT bays for units plugged in live."),
        ]
        assert self._console(master) == []

    def test_two_ams_plugged_in_for_one_spare_are_not_told_to_restart(
            self, tmp_path, monkeypatch):
        master, _bridge = self._stuck(
            tmp_path, monkeypatch, online=(A_p5, B_p5, C_p5, E_p5, G_p5, H_p5),
            recorded=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        self._run(master, 100, 120)
        assert self._bay(master, "Bambu_AMS_4")["bound"] == E_p5
        assert self._roster(master) == (
            "boxed:AAAA, boxed:BBBB, boxed:CCCC, ht:HHHH, boxed:EEEE, boxed:GGGG"
        )
        assert self._log(master) == [
            *self._first_claims(),
            *self._claim_log(E_p5, "Bambu_AMS_4"),
            ("warning",
             "AFC_BridgeBox chain1: AMS GGGG has no bay: all 4 AMS bays belong to other units "
             "(Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), Bambu_AMS_4 (EEEE)), "
             "and a Bambu bus addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
             "one. If this AMS replaces one of them, AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
             "UID=<that unit's uid> frees that bay and this AMS claims it live."),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:EEEE, boxed:GGGG -- "
             "recorded. A restart adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved EEEE on Bambu_AMS_4 (lane36-lane39, T36-T39); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_4"),
            ("respond_raw",
             "// action:prompt_text UID EEEE is on 'Bambu_AMS_4' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text It is saved on this bay once it has been online 15s."),
            ("respond_raw", "// action:prompt_text No other free bay of this type to move it to."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]
        again = self._restart(tmp_path, monkeypatch)
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "BBBB",
            "Bambu_AMS_3": "CCCC",
            "Bambu_AMS_4": "EEEE",
            "Bambu_AMS_HT_1": "HHHH",
            "Bambu_AMS_HT_2": "",
        }

    def test_a_roster_option_with_room_says_to_list_it(self, tmp_path,
                                                       monkeypatch):
        three = f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}"
        master, _bridge = self._stuck(tmp_path, monkeypatch,
                                      online=(A_p5, B_p5, C_p5, E_p5, G_p5, H_p5),
                                      recorded=three, roster=three)
        self._run(master, 100, 101)
        assert self._bay(master, "Bambu_AMS_4")["bound"] == E_p5
        assert self._log(master) == [
            *self._first_claims(),
            *self._claim_log(E_p5, "Bambu_AMS_4"),
            ("warning",
             "AFC_BridgeBox chain1: new AMS GGGG has no free bay: every AMS bay built belongs "
             "to a known unit. roster: is set and does not list it: add boxed:GGGG to roster: "
             "and RESTART to give it a bay."),
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_4"),
            ("respond_raw",
             "// action:prompt_text UID EEEE is on 'Bambu_AMS_4' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text roster: is set and does not list it. Add boxed:EEEE to "
             "roster: to pin it to a named bay."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]
        again = self._restart(tmp_path, monkeypatch,
                              roster=three + f", boxed:{G}")
        assert self._bay(again, "Bambu_AMS_4")["uid"] == G_p5
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "BBBB",
            "Bambu_AMS_3": "CCCC",
            "Bambu_AMS_4": "GGGG",
            "Bambu_AMS_HT_1": "HHHH",
            "Bambu_AMS_HT_2": "",
        }

    # ── the replace offer ─────────────────────────────────────────────────

    def test_it_is_offered_once_after_both_graces(self, tmp_path,
                                                  monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch)
        self._run(master, 100, 114)
        assert self._console(master) == []
        self._run(master, 115, 200)
        assert self._console(master) == self._offer_popup(15)
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            self.NEW_E,
        ]

    def test_a_unit_waiting_since_boot_is_offered_too(self, tmp_path,
                                                      monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch,
                                      recorded=self.FOUR + f", boxed:{E}")
        self._run(master, 100, 114)
        assert self._console(master) == []
        self._run(master, 115, 115)
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin No bay for AMS EEEE"),
            ("respond_raw",
             "// action:prompt_text UID EEEE is recorded but has no bay: every AMS bay is taken."),
            ("respond_raw",
             "// action:prompt_text Replace a unit that is offline: the new one takes its bay, "
             "lanes and T# now, and the old one is forgotten (its learned values and saved lane "
             "records, spools included, are erased)."),
            ("respond_raw",
             "// action:prompt_text Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 15s"),
            ("respond_raw",
             "// action:prompt_text Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE "
             "CHAIN=chain1 UID=EEEE opens this again."),
            ("respond_raw",
             "// action:prompt_button Replace Bambu_AMS_4|AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE OLD=DDDD|error"),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]
        assert self._log(master) == [
            *self._first_claims(),
        ]

    def test_an_offer_queued_before_a_print_waits_for_it(self, tmp_path,
                                                         monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch)
        master._popup_active_until = 1e9
        self._run(master, 100, 115)
        assert master._popup_queue == [("replace", E_p5)]
        master.printer.set_print_state("printing")
        master._popup_active_until = 0.0
        self._run(master, 116, 130)
        assert self._console(master) == []
        assert E_p5 not in master._replace_offered
        master.printer.set_print_state("standby")
        self._run(master, 131, 131)
        assert self._console(master) == self._offer_popup(31)
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            self.NEW_E,
        ]

    def test_it_waits_for_release_grace(self, tmp_path, monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch,
                                      release_grace=40.0)
        self._run(master, 100, 139)
        assert self._console(master) == []
        self._run(master, 140, 140)
        assert self._console(master) == self._offer_popup(40)
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            self.NEW_E,
        ]

    def test_a_flapping_new_unit_is_never_offered(self, tmp_path,
                                                  monkeypatch):
        master, bridge = self._stuck(tmp_path, monkeypatch)
        for t in range(100, 160):
            self._set_online(bridge, E_p5, t % 2 == 0)
            self._run(master, t, t)
        assert master._replace_offered == set()
        assert self._console(master) == []
        # By design a waiting unit that drops off is told again on return
        # (every offline read resets the tell), so a flap repeats it.
        assert self._log(master) == self._first_claims() + [self.TOLD_E] * 30

    def test_a_phantom_blip_does_not_restart_the_absence_clock(
            self, tmp_path, monkeypatch):
        master, bridge = self._stuck(tmp_path, monkeypatch)
        self._run(master, 100, 104)
        self._set_online(bridge, D_p5, True)
        self._run(master, 105, 105)
        # Claimed on the blip.
        assert self._bay(master, "Bambu_AMS_4")["bound"] == D_p5
        self._set_online(bridge, D_p5, False)
        self._run(master, 106, 115)
        assert self._bay(master, "Bambu_AMS_4")["bound"] == D_p5
        assert self._console(master) == self._offer_popup(15)
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            *self._claim_log(D_p5, "Bambu_AMS_4"),
            self.NEW_E,
        ]

    @pytest.mark.parametrize("outage", ["link-down", "chain-dark"])
    def test_an_outage_is_not_absence(self, tmp_path, monkeypatch, outage):
        master, bridge = self._stuck(tmp_path, monkeypatch)
        live = bridge.status
        if outage == "link-down":
            bridge._serial, bridge.connected = None, False
        else:
            self._online(bridge, [False] * len(self.CHAIN))
        self._run(master, 100, 130)
        # Nothing claimed or recorded: no unit is known online through either
        # outage, though the dead link's last status still reads them online.
        assert {pu["bound"] for pu in master._pool_units} == {None}
        assert master._missing_since == {}
        assert self._console(master) == []
        assert self._log(master) == []
        bridge._serial, bridge.connected, bridge.status = object(), True, live
        self._run(master, 131, 140)
        # DDDD's clock began at 131.
        assert self._console(master) == []
        self._run(master, 141, 150)
        # The outage restarted EEEE's online run too.
        assert self._console(master) == self._offer_popup(15)
        assert self._log(master) == (self._first_claims()
                                     + [self.TOLD_E, self.NEW_E])

    def test_during_a_print_it_waits_for_the_print_to_end(self, tmp_path,
                                                          monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch)
        master.printer.set_print_state("printing")
        self._run(master, 100, 150)
        assert master._replace_offered == set()
        assert self._console(master) == []
        master.printer.set_print_state("complete")
        self._run(master, 151, 151)
        assert self._console(master) == self._offer_popup(51)
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            self.NEW_E,
        ]

    def test_a_bay_with_a_loaded_lane_is_not_offered(self, tmp_path,
                                                     monkeypatch):
        # PREP restored the extruder's record of DDDD's lane36, though its
        # bay was never claimed this session.
        master, _bridge = self._stuck(tmp_path, monkeypatch)
        extruder = master.printer.afc.tools["extruder"]
        extruder.lane_loaded = "lane36"
        self._run(master, 100, 150)
        assert self._console(master) == []
        extruder.lane_loaded = None
        self._run(master, 151, 151)
        assert self._console(master) == self._offer_popup(51)
        assert self._log(master) == [
            *self._first_claims(),
            ("warning",
             "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays belong to other units "
             "(Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), Bambu_AMS_4 (DDDD)), "
             "and a Bambu bus addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
             "one. Bambu_AMS_4 (DDDD) is offline: if this AMS replaces it, AFC_BRIDGEBOX_FORGET "
             "CHAIN=chain1 UID=DDDD (AFC records lane36 as loaded to the toolhead: plug it back "
             "in and unload it, or take the filament out by hand and add FORCE=1, which clears "
             "the record) frees that bay and this AMS claims it live. AFC_BRIDGEBOX_REPLACE "
             "CHAIN=chain1 UID=EEEE OLD=Bambu_AMS_4 does both in one step."),
            self.NEW_E,
        ]

    def test_an_offer_for_a_unit_no_longer_waiting_is_dropped_and_made_again(
            self, tmp_path, monkeypatch):
        master, bridge = self._stuck(tmp_path, monkeypatch)
        master._popup_active_until = 1e9
        self._run(master, 100, 115)
        assert master._popup_queue == [("replace", E_p5)]
        self._set_online(bridge, E_p5, False)
        self._run(master, 116, 116)
        assert E_p5 not in master._no_bay
        master._popup_active_until = 0.0
        self._run(master, 117, 117)
        assert self._console(master) == []
        assert E_p5 not in master._replace_offered
        self._set_online(bridge, E_p5, True)
        self._run(master, 118, 133)
        assert self._console(master) == self._offer_popup(33)
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            self.NEW_E,
            self.TOLD_E,
        ]

    def test_getting_a_bay_ends_the_wait(self, tmp_path, monkeypatch):
        master, _bridge = self._stuck(tmp_path, monkeypatch)
        self._run(master, 100, 120)
        assert self._console(master) == self._offer_popup(15)
        gcmd = master.printer.gcode.run("AFC_BRIDGEBOX_FORGET",
                                        CHAIN="chain1", UID=D_p5)
        assert gcmd.messages == [
            ("respond_info",
             "AFC_BridgeBox chain1: forgot DDDD -- lanes 36-39 and the name Bambu_AMS_4 freed "
             "for reuse -- slot freed to the pool LIVE; the next same-family unit claims it "
             "with no reboot."),
        ]
        self._log(master).clear()
        self._run(master, 121, 121)
        assert self._bay(master, "Bambu_AMS_4")["bound"] == E_p5
        assert E_p5 not in master._no_bay
        assert E_p5 not in master._no_bay_told
        assert E_p5 not in master._replace_offered
        assert master.get_status()["waiting_for_bay"] == []
        assert self._log(master) == [
            *self._claim_log(E_p5, "Bambu_AMS_4"),
            ("info",
             "AFC_BridgeBox chain1: saved EEEE on Bambu_AMS_4 (lane36-lane39, T36-T39); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == [*self._offer_popup(15),
                                         *self.PROMPT_END]

    def test_a_unit_pulled_and_plugged_back_is_offered_again(self, tmp_path,
                                                             monkeypatch):
        master, bridge = self._stuck(tmp_path, monkeypatch)
        self._run(master, 100, 115)
        assert self._console(master) == self._offer_popup(15)
        self._set_online(bridge, E_p5, False)
        self._run(master, 116, 125)
        assert E_p5 not in master._replace_offered
        self._set_online(bridge, E_p5, True)
        self._run(master, 126, 140)
        assert self._console(master) == self._offer_popup(15)
        self._run(master, 141, 141)
        assert self._console(master) == [
            *self._offer_popup(15),
            *self._offer_popup(41),
        ]
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            self.NEW_E,
            self.TOLD_E,
        ]

    def test_a_unit_that_leaves_while_the_chain_is_dark_waits_no_longer(
            self, tmp_path, monkeypatch):
        master, bridge = self._stuck(tmp_path, monkeypatch)
        self._run(master, 100, 111)
        assert master.get_status()["waiting_for_bay"] == [E_p5]
        live = bridge.status
        self._online(bridge, [False] * len(self.CHAIN))
        self._run(master, 112, 115)
        bridge.status = live
        self._set_online(bridge, E_p5, False)
        self._run(master, 116, 117)
        assert master.get_status()["waiting_for_bay"] == []
        bays = master.printer.gcode.run("AFC_BRIDGEBOX_BAYS", CHAIN="chain1")
        assert bays.messages == [
            ("respond_info",
             "AFC_BridgeBox chain1: bay manager --\n  Bambu_AMS_1 [AMS]: AAAA (live)\n  "
             "Bambu_AMS_2 [AMS]: BBBB (live)\n  Bambu_AMS_3 [AMS]: CCCC (live)\n  Bambu_AMS_4 "
             "[AMS]: DDDD\n  Bambu_AMS_HT_1 [HT]: HHHH (live)\n  Bambu_AMS_HT_2 [HT]: free"),
        ]
        with pytest.raises(Exception) as err:
            master.printer.gcode.run("AFC_BRIDGEBOX_REPLACE", CHAIN="chain1",
                                     OLD=D_p5)
        assert str(err.value) == (
            "AFC_BRIDGEBOX_REPLACE: give UID=<new uid> -- no unit is waiting for a bay"
        )
        with pytest.raises(Exception) as err:
            master.printer.gcode.run("AFC_BRIDGEBOX_REPLACE", CHAIN="chain1",
                                     UID=E_p5, OLD=D_p5)
        assert str(err.value) == "AFC_BRIDGEBOX_REPLACE: EEEE is not on chain chain1"
        assert self._bay(master, "Bambu_AMS_4")["uid"] == D_p5
        bays_popup = [
            ("respond_raw", "// action:prompt_begin Bambu AMS Units"),
            ("respond_raw", "// action:prompt_text Bambu_AMS_1 [AMS]: AAAA (live)"),
            ("respond_raw", "// action:prompt_text Bambu_AMS_2 [AMS]: BBBB (live)"),
            ("respond_raw", "// action:prompt_text Bambu_AMS_3 [AMS]: CCCC (live)"),
            ("respond_raw", "// action:prompt_text Bambu_AMS_4 [AMS]: DDDD"),
            ("respond_raw", "// action:prompt_text Bambu_AMS_HT_1 [HT]: HHHH (live)"),
            ("respond_raw", "// action:prompt_text Bambu_AMS_HT_2 [HT]: free"),
            ("respond_raw",
             "// action:prompt_button Unassign Bambu_AMS_1|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=AAAA FORCE=1|warning"),
            ("respond_raw",
             "// action:prompt_button Unassign Bambu_AMS_2|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=BBBB FORCE=1|warning"),
            ("respond_raw",
             "// action:prompt_button Unassign Bambu_AMS_3|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=CCCC FORCE=1|warning"),
            ("respond_raw",
             "// action:prompt_button Unassign Bambu_AMS_4|AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=DDDD FORCE=1|warning"),
            ("respond_raw",
             "// action:prompt_button Unassign Bambu_AMS_HT_1|AFC_BRIDGEBOX_UNASSIGN "
             "CHAIN=chain1 UID=HHHH FORCE=1|warning"),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]
        assert self._console(master) == bays_popup
        self._set_online(bridge, E_p5, True)
        self._run(master, 118, 118)
        assert self._log(master) == [
            *self._first_claims(),
            self.TOLD_E,
            self.TOLD_E,
        ]
        # Told again in the log only; the console still holds just BAYS.
        assert self._console(master) == bays_popup

    def test_an_ht_offer_says_ams_ht(self, tmp_path, monkeypatch):
        # GGGG answers as an HT too.
        master, _bridge = self._stuck(tmp_path, monkeypatch, online=(A_p5, G_p5),
                                      recorded=f"boxed:{A}, ht:{H}",
                                      htmask=(1 << 6) | (1 << 5),
                                      pool_ams=1, pool_ht=1)
        self._run(master, 100, 115)
        assert self._console(master) == self._offer_popup(
            15, waiting=G_p5, bay="Bambu_AMS_HT_1", old=H_p5, lanes="lane28 (T28)",
            kind="AMS HT")
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            ("warning",
             "AFC_BridgeBox chain1: new HT GGGG has no free bay: every HT bay built belongs to "
             "a known unit. Bambu_AMS_HT_1 (HHHH) is offline: if this HT replaces it, "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=HHHH frees that bay and this HT claims it "
             "live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=GGGG OLD=Bambu_AMS_HT_1 does both in "
             "one step. Once it is recorded, RESTART builds it one; raise pool_ht to keep spare "
             "HT bays for units plugged in live."),
            ("info", "AFC_BridgeBox chain1: NEW unit(s) on the chain: ht:GGGG -- recorded."),
        ]

    def test_a_bay_no_longer_held_stops_its_absence_clock(self, tmp_path,
                                                          monkeypatch):
        # GGGG claimed the spare and was pulled before it was recorded;
        # with auto_drop the release frees the bay, and its clock goes too.
        master, bridge = self._stuck(
            tmp_path, monkeypatch, online=(A_p5, B_p5, C_p5, G_p5, H_p5),
            recorded=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}",
            auto_drop=True)
        self._run(master, 100, 104)
        self._set_online(bridge, G_p5, False)
        self._run(master, 105, 114)
        assert master._missing_since == {"GGGG": 105.0}
        self._run(master, 115, 115)
        assert self._bay(master, "Bambu_AMS_4")["uid"] is None
        assert master._missing_since == {}
        assert self._log(master) == [
            *self._first_claims(),
            *self._claim_log(G_p5, "Bambu_AMS_4"),
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_4 (UID GGGG offline >10s); lanes dropped "
             "live"),
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_4"),
            ("respond_raw",
             "// action:prompt_text UID GGGG is on 'Bambu_AMS_4' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text It is saved on this bay once it has been online 15s."),
            ("respond_raw", "// action:prompt_text No other free bay of this type to move it to."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]

    def test_the_no_bay_line_says_what_the_forget_needs_first(
            self, tmp_path, monkeypatch):
        master, bridge = self._stuck(tmp_path, monkeypatch,
                                     online=(A_p5, B_p5, C_p5, H_p5))
        self._run(master, 100, 111)
        master.printer.afc.tools["extruder"].lane_loaded = "lane36"
        self._set_online(bridge, E_p5, True)
        self._run(master, 112, 113)
        assert self._log(master) == [
            *self._first_claims(),
            ("warning",
             "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays belong to other units "
             "(Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 (CCCC), Bambu_AMS_4 (DDDD)), "
             "and a Bambu bus addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
             "one. Bambu_AMS_4 (DDDD) is offline: if this AMS replaces it, AFC_BRIDGEBOX_FORGET "
             "CHAIN=chain1 UID=DDDD (AFC records lane36 as loaded to the toolhead: plug it back "
             "in and unload it, or take the filament out by hand and add FORCE=1, which clears "
             "the record) frees that bay and this AMS claims it live. AFC_BRIDGEBOX_REPLACE "
             "CHAIN=chain1 UID=EEEE OLD=Bambu_AMS_4 does both in one step."),
        ]
        assert self._console(master) == []

    def test_a_recorded_ams_the_cap_left_waiting_is_told_why(
            self, tmp_path, monkeypatch):
        # EEEE ran on a fifth bay under an earlier start; a bus addresses
        # four.
        master, _bridge = self._stuck(
            tmp_path, monkeypatch, recorded=self.FOUR + f", boxed:{E}",
            state={"name_map": (f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                                f"{C}:Bambu_AMS_3, {D}:Bambu_AMS_4, "
                                f"{E}:Bambu_AMS_5, {H}:Bambu_AMS_HT_1"),
                   "lane_map": (f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, "
                                f"{E}:40:4, {H}:44:1")})
        assert master._unbayed == {E_p5: "Bambu_AMS_5"}
        self._run(master, 100, 115)
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin No bay for AMS EEEE"),
            ("respond_raw",
             "// action:prompt_text UID EEEE is recorded but has no bay: every AMS bay is "
             "taken. A Bambu bus addresses at most 4 AMS, and its saved bay Bambu_AMS_5 is not "
             "one of the 4 AMS bays."),
            ("respond_raw",
             "// action:prompt_text Replace a unit that is offline: the new one takes its bay, "
             "lanes and T# now, and the old one is forgotten (its learned values and saved lane "
             "records, spools included, are erased)."),
            ("respond_raw",
             "// action:prompt_text Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 15s"),
            ("respond_raw",
             "// action:prompt_text Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE "
             "CHAIN=chain1 UID=EEEE opens this again."),
            ("respond_raw",
             "// action:prompt_button Replace Bambu_AMS_4|AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE OLD=DDDD|error"),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]
        assert self._log(master) == [
            *self._first_claims(),
        ]

    # ── a recorded unit's bay is saved, so a restart never moves it ──────

    def _saved_pool(self, tmp_path: pathlib.Path,
                    monkeypatch: pytest.MonkeyPatch, uids: Sequence[str],
                    online: Sequence[str],
                    recorded: Optional[str] = f"boxed:{A}",
                    **options: Any) -> Tuple[afcBridgeBox, FakeBridge]:
        """
        Three AMS bays and one HT bay (lane24 to lane36), ``recorded`` as
        the recorded roster, watching ``uids`` with ``online`` online.

        :param options: the master's options over pool_ams=3, pool_ht=1
        :return tuple: (master, bridge)
        """
        opts: Dict[str, Any] = dict(pool_ams=3, pool_ht=1)
        opts.update(options)
        return self._pooled(tmp_path, monkeypatch, recorded, uids=uids,
                            online=online, **opts)

    def _at(self, master: afcBridgeBox, bridge: FakeBridge, *times: float,
            online: Optional[Sequence[bool]] = None) -> None:
        """Tick at each time, with the bridge reading ``online`` first."""
        if online is not None:
            self._online(bridge, online)
        self._tick(master, *times)

    @staticmethod
    def _restart_section(again: afcBridgeBox, section: str) -> Dict[str, str]:
        """:return dict: the keys the restart fabricated ``section`` with"""
        return again.printer.lookup_object(section).keys

    @staticmethod
    def _seed_section(tmp_path: pathlib.Path, section: str,
                      keys: Dict[str, str]) -> None:
        """
        Leave a section of the state file as an earlier boot would, written
        by the real _state_set of a master that builds nothing.

        record_chain_state writes the chain's own section only; a learned
        record lives in a section of its own.
        """
        scratch = tmp_path / "seed"
        scratch.mkdir()
        printer = make_printer(var_file=str(scratch / "AFC.var"))
        seed = make_bridgebox(scratch, printer=printer, register=False,
                              roster="", pool_ams=0, pool_ht=0,
                              state_file=str(tmp_path / "AFC_BridgeBox.cfg"))
        seed._state_set({section: dict(keys)})

    def test_a_new_unit_is_saved_on_the_bay_it_claimed(self, tmp_path,
                                                       monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,))
        self._at(master, bridge, 100.0)
        assert self._bay(master, "Bambu_AMS_2")["bound"] == C_p5
        assert C_p5 not in master._name_map
        self._at(master, bridge, 116.0)
        assert master._name_map[C_p5] == "Bambu_AMS_2"
        assert master._lane_map[C_p5] == (28, 4)
        assert master._state_get(SEC, "name_map") == (
            "AAAA:Bambu_AMS_1, CCCC:Bambu_AMS_2")
        assert master._state_get(SEC, "lane_map") == "AAAA:24:4, CCCC:28:4"
        assert master._bay_held(self._bay(master, "Bambu_AMS_2")) is True
        self._at(master, bridge, 130.0)
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_2"),
            ("respond_raw",
             "// action:prompt_text UID CCCC is on 'Bambu_AMS_2' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text It is saved on this bay once it has been online 15s."),
            ("respond_raw", "// action:prompt_text Keep it here, or move it to another named bay:"),
            ("respond_raw",
             "// action:prompt_button Bambu_AMS_3|AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=CCCC "
             "NAME=Bambu_AMS_3|primary"),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]

    def test_two_units_claimed_on_one_tick_keep_their_bays(self, tmp_path,
                                                           monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, E_p5),
                                          (C_p5, E_p5))
        self._at(master, bridge, 100.0, 116.0)
        assert self._bound(master) == {
            "Bambu_AMS_1": None,
            "Bambu_AMS_2": "CCCC",
            "Bambu_AMS_3": "EEEE",
        }
        assert master._name_map == {
            "AAAA": "Bambu_AMS_1",
            "CCCC": "Bambu_AMS_2",
            "EEEE": "Bambu_AMS_3",
        }
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            *self._claim_log(E_p5, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC, boxed:EEEE -- "
             "recorded. A restart adds their temperature cards."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
            ("info",
             "AFC_BridgeBox chain1: saved EEEE on Bambu_AMS_3 (lane32-lane35, T32-T35); it "
             "comes back there after a restart."),
        ]
        # Both claimed before the popup showed, so CCCC is offered no bay.
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_2")
        assert master._popup_queue == [("new", E_p5)]
        again = self._restart(tmp_path, monkeypatch, pool_ams=3, pool_ht=1)
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "CCCC",
            "Bambu_AMS_3": "EEEE",
            "Bambu_AMS_HT_1": "",
        }

    def test_units_keep_their_bays_whatever_order_they_are_recorded_in(
            self, tmp_path, monkeypatch):
        # CCCC claims first but blips offline, so EEEE is recorded first.
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, E_p5),
                                          (C_p5,))
        self._at(master, bridge, 100.0)
        self._at(master, bridge, 101.0, online=[False, False, True])
        self._at(master, bridge, *range(102, 120),
                 online=[False, True, True])
        assert self._roster(master) == "boxed:AAAA, boxed:EEEE, boxed:CCCC"
        assert (self._bay(master, "Bambu_AMS_2")["bound"],
                self._bay(master, "Bambu_AMS_3")["bound"]) == (C_p5, E_p5)
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            *self._claim_log(E_p5, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:EEEE -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved EEEE on Bambu_AMS_3 (lane32-lane35, T32-T35); it "
             "comes back there after a restart."),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_2",
                                                        "Bambu_AMS_3")
        assert master._popup_queue == [("new", E_p5)]
        again = self._restart(tmp_path, monkeypatch, pool_ams=3, pool_ht=1)
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "CCCC",
            "Bambu_AMS_3": "EEEE",
            "Bambu_AMS_HT_1": "",
        }

    def test_a_unit_pulled_before_the_grace_is_saved_only_once_it_holds(
            self, tmp_path, monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
                                          auto_drop=True, release_grace=2.0)
        self._at(master, bridge, 100.0)
        self._at(master, bridge, 101.0, 102.0, 103.0, 104.0,
                 online=[False, False])
        bay = self._bay(master, "Bambu_AMS_2")
        assert (bay["bound"], bay["uid"]) == (None, None)
        assert C_p5 not in master._name_map
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID CCCC offline >2s); lanes dropped "
             "live"),
        ]
        new_c = self._new_popup(C_p5, "Bambu_AMS_2", "Bambu_AMS_3")
        assert self._console(master) == new_c
        assert master._popup_queue == [("removed", C_p5, "Bambu_AMS_2")]
        self._log(master).clear()
        self._at(master, bridge, *range(105, 121), online=[False, True])
        assert bay["bound"] == C_p5
        assert master._name_map[C_p5] == "Bambu_AMS_2"
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        # The removal shows once the first popup's hold is up, by when
        # CCCC is back and saved there.
        assert self._console(master) == [
            *new_c, *self._removed_popup(C_p5, "Bambu_AMS_2")]
        assert master._popup_queue == [("new", C_p5)]

    def test_a_saved_bay_is_held_across_an_auto_drop(self, tmp_path,
                                                     monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, E_p5),
                                          (C_p5,), auto_drop=True)
        self._at(master, bridge, 100.0, 116.0)
        self._at(master, bridge, *range(117, 130),
                 online=[False, False, False])
        bay = self._bay(master, "Bambu_AMS_2")
        assert (bay["bound"], bay["uid"]) == (None, C_p5)
        self._at(master, bridge, 131.0, online=[False, False, True])
        assert self._bay(master, "Bambu_AMS_3")["bound"] == E_p5
        self._at(master, bridge, *range(132, 150),
                 online=[False, True, True])
        assert bay["bound"] == C_p5
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID CCCC offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
            *self._claim_log(E_p5, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:EEEE -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved EEEE on Bambu_AMS_3 (lane32-lane35, T32-T35); it "
             "comes back there after a restart."),
            *self._claim_log(C_p5, "Bambu_AMS_2"),
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_2"),
            ("respond_raw",
             "// action:prompt_text UID CCCC is on 'Bambu_AMS_2' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text It is saved on this bay once it has been online 15s."),
            ("respond_raw", "// action:prompt_text Keep it here, or move it to another named bay:"),
            ("respond_raw",
             "// action:prompt_button Bambu_AMS_3|AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=CCCC "
             "NAME=Bambu_AMS_3|primary"),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
            ("respond_raw", "// action:prompt_begin AMS removed: Bambu_AMS_2"),
            ("respond_raw",
             "// action:prompt_text Bambu_AMS_2 (UID CCCC) was unplugged; its bay is held for a "
             "re-plug."),
            ("respond_raw",
             "// action:prompt_text Re-plug it and it reclaims the same lanes/T#. Or forget it "
             "to free the bay to the pool:"),
            ("respond_raw",
             "// action:prompt_button Forget Bambu_AMS_2|AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
             "UID=CCCC|error"),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_3"),
            ("respond_raw",
             "// action:prompt_text UID EEEE is on 'Bambu_AMS_3' (its T# and lanes are live)."),
            ("respond_raw", "// action:prompt_text It is saved on this bay."),
            ("respond_raw", "// action:prompt_text No other free bay of this type to move it to."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]

    def test_enroll_before_claim_pins_on_the_claim_tick(self, tmp_path,
                                                        monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
                                          claim_grace=30.0)
        self._at(master, bridge, 100.0, 116.0)
        assert self._roster(master) == "boxed:AAAA, boxed:CCCC"
        assert C_p5 not in master._name_map
        self._at(master, bridge, 130.0)
        assert self._bay(master, "Bambu_AMS_2")["bound"] == C_p5
        assert master._name_map[C_p5] == "Bambu_AMS_2"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. CCCC has "
             "no bay yet and takes the next free one of its family."),
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(
            C_p5, "Bambu_AMS_2", "Bambu_AMS_3", saved=True)

    def test_a_unit_on_a_wrong_family_bay_is_not_saved(self, tmp_path,
                                                       monkeypatch):
        # With no htmask, chain index 4 reads as an HT: CCCC claims the HT
        # bay. By enrollment the htmask says boxed.
        master, bridge = self._saved_pool(tmp_path, monkeypatch,
                                          (A_p5, "", "", "", C_p5), (C_p5,))
        self._at(master, bridge, 100.0)
        assert self._bay(master, "Bambu_AMS_HT_1")["bound"] == C_p5
        bridge.diag = (1 << 5, "", (-1, 0, 0))
        self._at(master, bridge, 116.0, 117.0, 130.0)
        assert self._roster(master) == "boxed:AAAA, boxed:CCCC"
        assert C_p5 not in master._name_map
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_HT_1", "ht", 1),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("warning",
             "AFC_BridgeBox chain1: CCCC is recorded as boxed but is on HT bay Bambu_AMS_HT_1; "
             "not saving it. AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=CCCC FORCE=1 re-homes it."),
        ]
        # The popup showed on the claim tick, before the htmask said boxed.
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_HT_1")

    def test_option_mode_does_not_save_an_unlisted_unit(self, tmp_path,
                                                        monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
                                          recorded=None, roster=f"boxed:{A}")
        self._at(master, bridge, 100.0, 116.0, 130.0)
        assert self._roster(master) == "boxed:CCCC"
        assert C_p5 not in master._name_map
        assert C_p5 not in master._lane_map
        assert master._bay_held(self._bay(master, "Bambu_AMS_2")) is False
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. Add "
             "boxed:CCCC to roster: to enroll it (the option is set and overrides the file); "
             "the next restart gives it the lowest free bay of its family, which need not be "
             "the one it is on now."),
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_2"),
            ("respond_raw",
             "// action:prompt_text UID CCCC is on 'Bambu_AMS_2' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text roster: is set and does not list it. Add boxed:CCCC to "
             "roster: to pin it to a named bay."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]

    def test_option_mode_names_two_unlisted_units_together(self, tmp_path,
                                                           monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, E_p5),
                                          (C_p5, E_p5), recorded=None,
                                          roster=f"boxed:{A}")
        self._at(master, bridge, 100.0, 116.0)
        assert self._roster(master) == "boxed:CCCC, boxed:EEEE"
        assert (C_p5 in master._name_map, E_p5 in master._name_map) == (False, False)
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            *self._claim_log(E_p5, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC, boxed:EEEE -- "
             "recorded. Add boxed:CCCC, boxed:EEEE to roster: to enroll them (the option is "
             "set and overrides the file); the next restart gives them the lowest free bays "
             "of their family, which need not be the ones they are on now."),
        ]
        assert self._console(master) == [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_2"),
            ("respond_raw",
             "// action:prompt_text UID CCCC is on 'Bambu_AMS_2' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text roster: is set and does not list it. Add boxed:CCCC to "
             "roster: to pin it to a named bay."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]
        assert master._popup_queue == [("new", E_p5)]

    def test_a_name_saved_for_another_recorded_uid_is_not_double_pinned(
            self, tmp_path, monkeypatch):
        # ZZZZ is recorded and saved as Bambu_AMS_2, which a spare wears
        # this session; it is the only free AMS bay, so CCCC takes it.
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
                                          pool_ams=2)
        master._state_set({SEC: {"roster": f"boxed:{A}, boxed:{Z}"}})
        master._name_map[Z] = "Bambu_AMS_2"
        self._at(master, bridge, 100.0, 116.0, 117.0, 130.0)
        assert self._bay(master, "Bambu_AMS_2")["bound"] == C_p5
        assert C_p5 not in master._name_map
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("warning",
             "AFC_BridgeBox chain1: Bambu_AMS_2 is saved for ZZZZ, so CCCC is not saved on it. "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=ZZZZ or AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 "
             "UID=ZZZZ, or AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=CCCC NAME=<other bay> moves "
             "CCCC to another bay."),
        ]
        # Its popup promises no save that cannot happen.
        popup = [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_2"),
            ("respond_raw",
             "// action:prompt_text UID CCCC is on 'Bambu_AMS_2' (its T# and lanes are live)."),
            ("respond_raw", "// action:prompt_text No other free bay of this type to move it to."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]
        assert self._console(master) == popup
        self._log(master).clear()
        gcmd = master.printer.gcode.run("AFC_BRIDGEBOX_FORGET",
                                        CHAIN="chain1", UID=Z)
        assert gcmd.messages == [
            ("respond_info", "AFC_BridgeBox chain1: forgot ZZZZ. Applies at the next RESTART."),
        ]
        self._at(master, bridge, 131.0)
        assert master._name_map[C_p5] == "Bambu_AMS_2"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        # FORGET closed the popup.
        assert self._console(master) == [*popup, *self.PROMPT_END]

    def test_a_new_unit_leaves_a_bay_saved_for_another_unit_free(
            self, tmp_path, monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,))
        master._state_set({SEC: {"roster": f"boxed:{A}, boxed:{Z}"}})
        master._name_map[Z] = "Bambu_AMS_2"
        self._at(master, bridge, 100.0, 116.0)
        assert self._bay(master, "Bambu_AMS_2")["bound"] is None
        assert self._bay(master, "Bambu_AMS_3")["bound"] == C_p5
        assert master._name_map[C_p5] == "Bambu_AMS_3"
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_3 (lane32-lane35, T32-T35); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_3")

    def test_a_name_saved_outside_the_roster_is_taken_over(self, tmp_path,
                                                           monkeypatch):
        master, bridge = self._saved_pool(
            tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
            state={"name_map": f"{A}:Bambu_AMS_1, {Z}:Bambu_AMS_2",
                   "lane_map": f"{A}:24:4, {Z}:28:4"})
        assert master._name_map.get(Z) == "Bambu_AMS_2"
        self._at(master, bridge, 100.0, 116.0)
        assert master._state_get(SEC, "name_map") == (
            "AAAA:Bambu_AMS_1, CCCC:Bambu_AMS_2")
        assert Z not in master._lane_map
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_2",
                                                        "Bambu_AMS_3")

    def _cccc_saved(self, tmp_path: pathlib.Path,
                    monkeypatch: pytest.MonkeyPatch, uids: Sequence[str] = (A_p5, C_p5)
                    ) -> Tuple[afcBridgeBox, FakeBridge]:
        """
        CCCC plugged into a chain that records boxed:AAAA (offline): it
        claims the Bambu_AMS_2 spare at once and is saved there 16s later.

        :return tuple: (master, bridge), the setup's lines cleared
        """
        master, bridge = self._saved_pool(tmp_path, monkeypatch, uids, (C_p5,))
        self._at(master, bridge, 100.0, 116.0)
        assert master._name_map[C_p5] == "Bambu_AMS_2"
        self._log(master).clear()
        self._console(master).clear()
        return master, bridge

    def test_unassign_then_replug_saves_the_new_bay(self, tmp_path,
                                                    monkeypatch):
        master, bridge = self._cccc_saved(tmp_path, monkeypatch,
                                          uids=(A_p5, C_p5, E_p5))
        self._at(master, bridge, 117.0, online=[False, False, False])
        gcmd = master.printer.gcode.run("AFC_BRIDGEBOX_UNASSIGN",
                                        CHAIN="chain1", UID=C_p5)
        assert gcmd.messages == [
            ("respond_info",
             "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bambu_AMS_2' -- lanes dropped "
             "live (learned values stay with the unit). It takes a free bay of its family, its "
             "last one first, and is saved there once it has been online 15s."),
        ]
        assert self._console(master) == self.PROMPT_END
        assert C_p5 not in master._name_map
        self._at(master, bridge, 118.0, online=[False, False, True])
        self._at(master, bridge, *range(119, 136), online=[False, True, True])
        assert self._bay(master, "Bambu_AMS_3")["bound"] == C_p5
        assert (master._name_map[C_p5], master._name_map[E_p5]) == (
            "Bambu_AMS_3", "Bambu_AMS_2")
        assert master._state_get(SEC, "name_map") == (
            "AAAA:Bambu_AMS_1, EEEE:Bambu_AMS_2, CCCC:Bambu_AMS_3")
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID CCCC, AFC_BRIDGEBOX_UNASSIGN); "
             "lanes dropped live"),
            *self._claim_log(E_p5, "Bambu_AMS_2"),
            *self._claim_log(C_p5, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:EEEE -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved EEEE on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_3 (lane32-lane35, T32-T35); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == [
            *self.PROMPT_END, *self._new_popup(E_p5, "Bambu_AMS_2")]

    def test_a_replugged_recorded_unit_is_saved_only_after_enroll_grace(
            self, tmp_path, monkeypatch):
        master, bridge = self._cccc_saved(tmp_path, monkeypatch)
        self._at(master, bridge, 117.0, online=[False, False])
        master.printer.gcode.run("AFC_BRIDGEBOX_UNASSIGN", CHAIN="chain1",
                                 UID=C_p5)
        assert self._console(master) == self.PROMPT_END
        self._at(master, bridge, 118.0, online=[False, True])
        assert self._bay(master, "Bambu_AMS_2")["bound"] == C_p5
        assert C_p5 not in master._name_map
        self._at(master, bridge, 132.0)
        assert C_p5 not in master._name_map
        self._at(master, bridge, 133.0)
        assert master._name_map[C_p5] == "Bambu_AMS_2"
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID CCCC, AFC_BRIDGEBOX_UNASSIGN); "
             "lanes dropped live"),
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == [
            *self.PROMPT_END,
            *self._new_popup(C_p5, "Bambu_AMS_2", "Bambu_AMS_3")]

    def test_a_returning_uid_takes_the_spare_wearing_its_saved_name(
            self, tmp_path, monkeypatch):
        master, bridge = self._saved_pool(
            tmp_path, monkeypatch, (A_p5, Z), (Z,),
            state={"name_map": f"{A}:Bambu_AMS_1, {Z}:Bambu_AMS_3",
                   "lane_map": f"{A}:24:4, {Z}:32:4"})
        self._at(master, bridge, 100.0)
        assert self._bay(master, "Bambu_AMS_3")["bound"] == Z
        assert self._bay(master, "Bambu_AMS_2")["bound"] is None
        assert self._console(master) == []
        # Not recorded yet, so it has no bay at restart and holds none.
        assert master._bay_held(self._bay(master, "Bambu_AMS_3")) is False
        self._at(master, bridge, 116.0)
        assert master._bay_held(self._bay(master, "Bambu_AMS_3")) is True
        assert self._console(master) == []
        assert self._log(master) == [
            *self._claim_log(Z, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:ZZZZ -- recorded. A restart "
             "adds its temperature card."),
        ]
        again = self._restart(tmp_path, monkeypatch, pool_ams=3, pool_ht=1)
        assert self._uids(again)["Bambu_AMS_3"] == Z

    def test_an_unlisted_uid_on_the_spare_wearing_its_name_is_not_held(
            self, tmp_path, monkeypatch):
        master, bridge = self._saved_pool(
            tmp_path, monkeypatch, (A_p5, Z), (Z,), recorded=None,
            state={"name_map": f"{A}:Bambu_AMS_1, {Z}:Bambu_AMS_3",
                   "lane_map": f"{A}:24:4, {Z}:32:4"},
            roster=f"boxed:{A}", auto_drop=True)
        self._at(master, bridge, 100.0)
        bay = self._bay(master, "Bambu_AMS_3")
        assert bay["bound"] == Z
        assert master._bay_held(bay) is False
        popup = [
            ("respond_raw", "// action:prompt_begin New AMS on Bambu_AMS_3"),
            ("respond_raw",
             "// action:prompt_text UID ZZZZ is on 'Bambu_AMS_3' (its T# and lanes are live)."),
            ("respond_raw",
             "// action:prompt_text roster: is set and does not list it. Add boxed:ZZZZ to "
             "roster: to pin it to a named bay."),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]
        assert self._console(master) == popup
        self._at(master, bridge, *range(101, 115), online=[False, False])
        assert (bay["bound"], bay["uid"]) == (None, None)
        assert self._log(master) == [
            *self._claim_log(Z, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_3 (UID ZZZZ offline >10s); lanes dropped "
             "live"),
        ]
        # The removal popup waits behind the New AMS one still showing.
        assert self._console(master) == popup
        assert master._popup_queue == [("removed", Z, "Bambu_AMS_3")]

    def test_pools_off_never_auto_pins(self, tmp_path, monkeypatch):
        # Nothing is claimed live with no pool, so nothing is pinned live;
        # the boot still gives a rostered bay its uid's record.
        self._seed_section(tmp_path, "AFC_BridgeBox chain1 learned AAAA",
                           {"afc_bowden_length": "3632.0"})
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
                                          pool_ams=0, pool_ht=0)
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        assert unit.afc_bowden_length == 3632.0
        self._at(master, bridge, 100.0, 116.0, 130.0)
        assert self._roster(master) == "boxed:AAAA, boxed:CCCC"
        assert C_p5 not in master._name_map
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (AAAA) is offline on a live chain -- if it "
             "stays gone 120s it will be recorded as removed (takes effect at the next RESTART)."),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. RESTART "
             "to enroll."),
        ]
        assert self._console(master) == []
        gcmd = master.printer.gcode.run("AFC_BRIDGEBOX_UNASSIGN",
                                        CHAIN="chain1", UID=A_p5)
        assert gcmd.messages == [
            ("respond_info",
             "AFC_BridgeBox chain1: unassigned AAAA from bay 'Bambu_AMS_1' (learned values stay "
             "with the unit). It takes the lowest free bay of its family at the next restart."),
        ]
        assert self._console(master) == self.PROMPT_END

    def test_a_tombstone_seen_for_a_moment_does_not_hold_the_spare(
            self, tmp_path, monkeypatch):
        # ZZZZ is outside the roster, so its name holds no bay. It blips
        # online, claims the spare wearing its name, and drops.
        master, bridge = self._saved_pool(
            tmp_path, monkeypatch, (A_p5, Z, "NEWW"), (Z,), pool_ams=2,
            auto_drop=True,
            state={"name_map": f"{A}:Bambu_AMS_1, {Z}:Bambu_AMS_2",
                   "lane_map": f"{A}:24:4, {Z}:28:4"})
        self._at(master, bridge, 100.0)
        bay = self._bay(master, "Bambu_AMS_2")
        assert bay["bound"] == Z
        self._at(master, bridge, *range(101, 115),
                 online=[False, False, False])
        assert (bay["bound"], bay["uid"]) == (None, None)
        self._at(master, bridge, *range(115, 131),
                 online=[False, False, True])
        assert bay["bound"] == "NEWW"
        assert master._state_get(SEC, "name_map") == (
            "AAAA:Bambu_AMS_1, NEWW:Bambu_AMS_2")
        assert self._log(master) == [
            *self._claim_log(Z, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID ZZZZ offline >10s); lanes dropped "
             "live"),
            *self._claim_log("NEWW", "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:NEWW -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved NEWW on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        # ZZZZ held no bay, so its removal says the bay went back to the
        # pool; NEWW's popup waits out that one's hold.
        assert self._console(master) == self._removed_popup(
            Z, "Bambu_AMS_2", held=False)
        assert master._popup_queue == [("new", "NEWW")]

    #: ZZZZ saved on Bambu_AMS_2 by an earlier start, outside the roster.
    Z_ON_2 = {"name_map": f"{A}:Bambu_AMS_1, {Z}:Bambu_AMS_2",
              "lane_map": f"{A}:24:4, {Z}:28:4"}

    def test_a_returning_and_a_new_unit_on_one_tick_keep_their_bays(
            self, tmp_path, monkeypatch):
        # CCCC sorts before ZZZZ by uid; only the bay order lets ZZZZ
        # claim first.
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, Z),
                                          (C_p5, Z), state=self.Z_ON_2)
        self._at(master, bridge, *range(100, 118))
        assert self._bound(master) == {"Bambu_AMS_1": None,
                                       "Bambu_AMS_2": Z, "Bambu_AMS_3": C_p5}
        assert self._log(master) == [
            *self._claim_log(Z, "Bambu_AMS_2"),
            *self._claim_log(C_p5, "Bambu_AMS_3"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC, boxed:ZZZZ -- "
             "recorded. A restart adds their temperature cards."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_3 (lane32-lane35, T32-T35); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_3")
        again = self._restart(tmp_path, monkeypatch, pool_ams=3, pool_ht=1)
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "ZZZZ",
            "Bambu_AMS_3": "CCCC",
            "Bambu_AMS_HT_1": "",
        }

    def test_a_last_bay_another_unit_comes_back_to_is_left_to_it(
            self, tmp_path, monkeypatch):
        state = dict(self.Z_ON_2,
                     bay_owner=f"{A}:Bambu_AMS_1, {C}:Bambu_AMS_2")
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, Z),
                                          (C_p5, Z), state=state)
        self._at(master, bridge, 100.0, 116.0, 130.0)
        assert self._bound(master) == {"Bambu_AMS_1": None,
                                       "Bambu_AMS_2": Z, "Bambu_AMS_3": C_p5}
        assert (master._name_map[Z], master._name_map[C_p5]) == (
            "Bambu_AMS_2", "Bambu_AMS_3")
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_3"),
            *self._claim_log(Z, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC, boxed:ZZZZ -- "
             "recorded. A restart adds their temperature cards."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_3 (lane32-lane35, T32-T35); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_3")
        again = self._restart(tmp_path, monkeypatch, pool_ams=3, pool_ht=1)
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "ZZZZ",
            "Bambu_AMS_3": "CCCC",
            "Bambu_AMS_HT_1": "",
        }

    def test_a_returning_unit_claims_its_bay_before_a_new_one(
            self, tmp_path, monkeypatch):
        # CCCC sorts before ZZZZ by uid; only the bay order lets ZZZZ
        # claim first.
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, Z),
                                          (C_p5, Z), state=self.Z_ON_2,
                                          pool_ams=2)
        self._at(master, bridge, 100.0)
        assert self._bound(master) == {"Bambu_AMS_1": None, "Bambu_AMS_2": Z}
        assert self._log(master) == [
            *self._claim_log(Z, "Bambu_AMS_2"),
            ("warning",
             "AFC_BridgeBox chain1: new AMS CCCC has no free bay: every AMS bay built belongs "
             "to a known unit. Bambu_AMS_1 (AAAA) is offline: if this AMS replaces it, "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=AAAA frees that bay and this AMS claims it "
             "live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=CCCC OLD=Bambu_AMS_1 does both in "
             "one step. Once it is recorded, RESTART builds it one, past the AMS band, which "
             "moves the HT lanes up 4; raise pool_ams to keep spare AMS bays for units plugged "
             "in live (at most 4 AMS)."),
        ]
        assert self._console(master) == []

    def test_a_new_unit_leaves_the_bay_a_waiting_unit_comes_back_to(
            self, tmp_path, monkeypatch):
        # ZZZZ was released moments ago, so it must hold online for
        # flap_claim_grace before it claims again.
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, Z),
                                          (C_p5, Z), state=self.Z_ON_2)
        master._released_at = {Z: 95.0}
        self._at(master, bridge, 100.0)
        assert self._bay(master, "Bambu_AMS_3")["bound"] == C_p5
        assert self._bay(master, "Bambu_AMS_2")["bound"] is None
        self._at(master, bridge, *range(101, 118))
        assert self._bay(master, "Bambu_AMS_2")["bound"] == Z
        assert (master._name_map[C_p5], master._name_map[Z]) == (
            "Bambu_AMS_3", "Bambu_AMS_2")
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_3"),
            *self._claim_log(Z, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC, boxed:ZZZZ -- "
             "recorded. A restart adds their temperature cards."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_3 (lane32-lane35, T32-T35); it "
             "comes back there after a restart."),
        ]
        # Shown on the first tick, while Bambu_AMS_2 was still free.
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_3",
                                                        "Bambu_AMS_2")
        again = self._restart(tmp_path, monkeypatch, pool_ams=3, pool_ht=1)
        assert self._uids(again) == {
            "Bambu_AMS_1": "AAAA",
            "Bambu_AMS_2": "ZZZZ",
            "Bambu_AMS_3": "CCCC",
            "Bambu_AMS_HT_1": "",
        }

    def test_a_saved_name_that_is_no_bay_now_gives_way_to_the_live_bay(
            self, tmp_path, monkeypatch):
        # ZZZZ is saved as Bambu_AMS_5, which three AMS bays do not have.
        master, bridge = self._saved_pool(
            tmp_path, monkeypatch, (A_p5, Z), (Z,),
            state={"name_map": f"{A}:Bambu_AMS_1, {Z}:Bambu_AMS_5",
                   "lane_map": f"{A}:24:4, {Z}:40:4"})
        self._at(master, bridge, 100.0, 116.0)
        assert self._bay(master, "Bambu_AMS_2")["bound"] == Z
        assert self._log(master) == [
            *self._claim_log(Z, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:ZZZZ -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved ZZZZ on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(Z, "Bambu_AMS_2",
                                                        "Bambu_AMS_3")
        again = self._restart(tmp_path, monkeypatch, pool_ams=3, pool_ht=1)
        assert self._uids(again)["Bambu_AMS_2"] == Z
        assert self._restart_section(again, "AFC_lane lane28")["unit"] == (
            "Bambu_AMS_2:1")

    def test_an_ht_is_saved_on_its_one_lane(self, tmp_path, monkeypatch):
        # Chain index 4 with no htmask reads as an HT.
        master, bridge = self._saved_pool(tmp_path, monkeypatch,
                                          (A_p5, "", "", "", H_p5), (H_p5,))
        self._at(master, bridge, 100.0, 116.0)
        assert self._log(master) == [
            *self._claim_log(H_p5, "Bambu_AMS_HT_1", "ht", 1),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: ht:HHHH -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved HHHH on Bambu_AMS_HT_1 (lane36, T36); it comes back "
             "there after a restart."),
        ]
        assert self._console(master) == self._new_popup(H_p5, "Bambu_AMS_HT_1")
        again = self._restart(tmp_path, monkeypatch, pool_ams=3, pool_ht=1)
        assert self._uids(again)["Bambu_AMS_HT_1"] == H_p5
        assert self._restart_section(again, "AFC_lane lane36")["unit"] == (
            "Bambu_AMS_HT_1:1")

    def test_a_unit_recorded_with_no_bay_is_saved_when_one_frees(
            self, tmp_path, monkeypatch):
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
                                          pool_ams=1)
        self._at(master, bridge, 100.0, 116.0)
        assert self._roster(master) == "boxed:AAAA, boxed:CCCC"
        # The claim said CCCC has no free bay; the enrollment line does not
        # say it again.
        assert self._log(master) == [
            ("warning",
             "AFC_BridgeBox chain1: new AMS CCCC has no free bay: every AMS bay built belongs "
             "to a known unit. Bambu_AMS_1 (AAAA) is offline: if this AMS replaces it, "
             "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=AAAA frees that bay and this AMS claims it "
             "live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=CCCC OLD=Bambu_AMS_1 does both in "
             "one step. Once it is recorded, RESTART builds it one, past the AMS band, which "
             "moves the HT lanes up 4; raise pool_ams to keep spare AMS bays for units plugged "
             "in live (at most 4 AMS)."),
            ("info", "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded."),
        ]
        offer = self._offer_popup(16, waiting=C_p5, bay="Bambu_AMS_1", old=A_p5,
                                  lanes="lane24-lane27 (T24-T27)")
        assert self._console(master) == offer
        self._log(master).clear()
        master.printer.gcode.run("AFC_BRIDGEBOX_FORGET", CHAIN="chain1",
                                 UID=A_p5)
        self._at(master, bridge, 117.0)
        assert self._bay(master, "Bambu_AMS_1")["bound"] == C_p5
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_1"),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_1 (lane24-lane27, T24-T27); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == [*offer, *self.PROMPT_END]

    # ── learned values follow the uid, not the bay ───────────────────────

    #: The state section holding what CCCC learned on chain1.
    LEARNED_C = "AFC_BridgeBox chain1 learned CCCC"

    @staticmethod
    def _new_popup(uid: str, bay: str, offer: Optional[str] = None, *,
                   saved: bool = False) -> List[LogLine]:
        """
        :param offer: the named bay the popup offers to move it to; None
            when no other free bay of its family is left
        :param saved: the bay is already saved for it, so the popup says
            so instead of naming the enroll grace
        :return list: the popup a new unit claimed onto spare ``bay`` gets
        """
        if saved:
            when = "// action:prompt_text It is saved on this bay."
        else:
            when = ("// action:prompt_text It is saved on this bay once it "
                    "has been online 15s.")
        if offer is None:
            choice = [("respond_raw",
                       "// action:prompt_text No other free bay of this type "
                       "to move it to.")]
        else:
            choice = [
                ("respond_raw",
                 "// action:prompt_text Keep it here, or move it to another "
                 "named bay:"),
                ("respond_raw",
                 f"// action:prompt_button {offer}|AFC_BRIDGEBOX_ASSIGN "
                 f"CHAIN=chain1 UID={uid} NAME={offer}|primary"),
            ]
        return [
            ("respond_raw", f"// action:prompt_begin New AMS on {bay}"),
            ("respond_raw",
             f"// action:prompt_text UID {uid} is on '{bay}' (its T# and "
             "lanes are live)."),
            ("respond_raw", when),
            *choice,
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]

    @staticmethod
    def _removed_popup(uid: str, bay: str, held: bool = True
                       ) -> List[LogLine]:
        """
        :param held: the bay is saved for ``uid`` and kept for its re-plug;
            False when it went back to the pool
        :return list: the popup an auto-drop of ``uid`` from ``bay`` shows
        """
        if held:
            body = [
                ("respond_raw",
                 f"// action:prompt_text {bay} (UID {uid}) was unplugged; its "
                 "bay is held for a re-plug."),
                ("respond_raw",
                 "// action:prompt_text Re-plug it and it reclaims the same "
                 "lanes/T#. Or forget it to free the bay to the pool:"),
            ]
        else:
            body = [
                ("respond_raw",
                 f"// action:prompt_text {bay} (UID {uid}) was unplugged; its "
                 "bay went back to the pool."),
                ("respond_raw",
                 "// action:prompt_text A re-plug takes it back while it is "
                 "free, else the lowest free bay of its family. Or forget it "
                 "if it is not coming back:"),
            ]
        return [
            ("respond_raw", f"// action:prompt_begin AMS removed: {bay}"),
            *body,
            ("respond_raw",
             f"// action:prompt_button Forget {bay}|AFC_BRIDGEBOX_FORGET "
             f"CHAIN=chain1 UID={uid}|error"),
            ("respond_raw",
             "// action:prompt_footer_button Dismiss|RESPOND TYPE=command "
             "MSG=action:prompt_end|info"),
            ("respond_raw", "// action:prompt_show"),
        ]

    #: What a gcode command that closes the open popup puts on the console.
    PROMPT_END = [("respond_raw", "// action:prompt_end")]

    def _bowden(self, master: afcBridgeBox, bay: str) -> Tuple[float, float]:
        """:return tuple: the bay's unit's load and unload bowden lengths"""
        unit = master.printer.lookup_object(f"AFC_BambuAMS {bay}")
        return unit.afc_bowden_length, unit.afc_unload_bowden_length

    def test_a_live_claim_applies_the_uids_record(self, tmp_path, monkeypatch):
        self._seed_section(tmp_path, self.LEARNED_C,
                           {"afc_bowden_length": "3632.0"})
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,))
        assert self._bowden(master, "Bambu_AMS_2") == (3000.0, 3000.0)
        self._at(master, bridge, 100.0)
        assert self._bowden(master, "Bambu_AMS_2") == (3632.0, 3632.0)
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2",
                             takes="afc_bowden_length 3632mm"),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_2",
                                                        "Bambu_AMS_3")

    def test_the_claim_line_shows_a_learned_fraction(self, tmp_path,
                                                     monkeypatch):
        self._seed_section(tmp_path, self.LEARNED_C,
                           {"afc_bowden_length": "1234.5",
                            "afc_unload_bowden_length": "1234.44"})
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,))
        self._at(master, bridge, 100.0)
        assert self._bowden(master, "Bambu_AMS_2") == (1234.5, 1234.44)
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2",
                             takes="afc_bowden_length 1234.5mm, "
                                   "afc_unload_bowden_length 1234.4mm"),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_2",
                                                        "Bambu_AMS_3")

    def test_a_different_uid_on_the_same_bay_gets_defaults(
            self, tmp_path, monkeypatch):
        self._seed_section(tmp_path, self.LEARNED_C,
                           {"afc_bowden_length": "3632.0"})
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5, E_p5),
                                          (C_p5,), auto_drop=True)
        self._at(master, bridge, 100.0)
        assert self._bowden(master, "Bambu_AMS_2") == (3632.0, 3632.0)
        self._at(master, bridge, *range(101, 113),
                 online=[False, False, False])
        assert self._bay(master, "Bambu_AMS_2")["bound"] is None
        self._at(master, bridge, 113.0, online=[False, False, True])
        assert self._bay(master, "Bambu_AMS_2")["bound"] == E_p5
        assert self._bowden(master, "Bambu_AMS_2") == (3000.0, 3000.0)
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2",
                             takes="afc_bowden_length 3632mm"),
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID CCCC offline >10s); lanes dropped "
             "live"),
            *self._claim_log(E_p5, "Bambu_AMS_2"),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_2",
                                                        "Bambu_AMS_3")

    def test_an_override_beats_the_record_at_claim(
            self, tmp_path, monkeypatch):
        self._seed_section(tmp_path, self.LEARNED_C,
                           {"afc_bowden_length": "3632.0",
                            "afc_unload_bowden_length": "3632.0"})
        master, bridge = self._saved_pool(
            tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
            sections={"AFC_BridgeBox Bambu_AMS_2": {"afc_bowden_length": "1800"}})
        self._at(master, bridge, 100.0)
        assert self._bowden(master, "Bambu_AMS_2") == (1800.0, 3632.0)
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2",
                             takes="afc_unload_bowden_length 3632mm"),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_2",
                                                        "Bambu_AMS_3")

    # ── the chain watch around a claim ───────────────────────────────────

    class _UnreadableExtruder:
        """A toolhead whose loaded-lane record cannot be read."""

        name = "extruder1"

        @property
        def lane_loaded(self) -> str:
            """:raises RuntimeError: always"""
            raise RuntimeError("no tools")

    def test_a_failing_loaded_lane_check_stops_no_claim(
            self, tmp_path, monkeypatch):
        # A toolhead whose record cannot be read fails the check on every
        # tick; the claim's own reads of it warn and go on.
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,))
        master._lane_moves = {"lane24": {"family": "ams", "uid": D_p5,
                                         "name": "", "now": "Bambu_AMS_1"}}
        master.printer.afc.tools["extruder1"] = self._UnreadableExtruder()
        self._at(master, bridge, 100.0, 101.0, 116.0)
        assert self._bay(master, "Bambu_AMS_2")["bound"] == C_p5
        assert master._name_map[C_p5] == "Bambu_AMS_2"
        assert master._moved_err_logged == "RuntimeError: no tools"
        assert self._log(master) == [
            ("warning",
             "AFC_BridgeBox chain1: loaded-lane check failed (RuntimeError: no tools); the "
             "chain watch carries on."),
            ("warning",
             "AFC bambu Bambu_AMS_2: claim could not restore the saved lane records: no tools"),
            ("debug",
             "AFC bambu Bambu_AMS_2: chain index not resolved yet (UID CCCC); holding this "
             "unit's registrations until the chain map arrives"),
            ("info",
             "AFC bambu Bambu_AMS_2: claimed UID CCCC as boxed and brought online live "
             "(ams_index=0)."),
            ("warning", "AFC_BridgeBox chain1: TcmdAssign lane28 failed: no tools"),
            ("warning", "AFC_BridgeBox chain1: TcmdAssign lane29 failed: no tools"),
            ("warning", "AFC_BridgeBox chain1: TcmdAssign lane30 failed: no tools"),
            ("warning", "AFC_BridgeBox chain1: TcmdAssign lane31 failed: no tools"),
            ("info",
             "AFC_BridgeBox chain1: CLAIMED CCCC as boxed onto Bambu_AMS_2 (4 lanes) -- live, "
             "no restart."),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_2",
                                                        "Bambu_AMS_3")

    def test_the_enrollment_line_survives_a_tick_that_fails(
            self, tmp_path, monkeypatch):
        # CCCC waits for PREP, is recorded on a tick whose claim raises, and
        # is named on the next good tick, which claims it, and only once. A
        # second tick failing the same way is not logged again.
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,))
        afc = master.printer.afc
        afc.prep_done = False
        master.printer.reactor.now = 100.0
        master._scout_ready()
        assert self._log(master) == []
        self._at(master, bridge, 100.0)
        assert self._bay(master, "Bambu_AMS_2")["bound"] is None
        afc.prep_done = True
        cmds, afc.tool_cmds = afc.tool_cmds, None
        self._at(master, bridge, 116.0, 116.5)
        assert master._watch_state == (
            "error: AttributeError: 'NoneType' object has no attribute 'items'")
        assert master._tick_err_logged == (
            "AttributeError: 'NoneType' object has no attribute 'items'")
        assert self._roster(master) == "boxed:AAAA, boxed:CCCC"
        assert self._bay(master, "Bambu_AMS_2")["bound"] is None
        assert self._log(master) == [
            ("debug", "AFC_BridgeBox chain1: unit claims wait for PREP"),
            ("warning",
             "AFC_BridgeBox chain1: chain watch tick failed (AttributeError: 'NoneType' object "
             "has no attribute 'items'); will keep retrying."),
        ]
        self._log(master).clear()
        afc.tool_cmds = cmds
        self._at(master, bridge, 117.0, 118.0)
        assert self._bay(master, "Bambu_AMS_2")["bound"] == C_p5
        assert self._log(master) == [
            *self._claim_log(C_p5, "Bambu_AMS_2"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, T28-T31); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == []

    def test_the_new_unit_popup_offers_only_bays_assign_takes(
            self, tmp_path, monkeypatch):
        # Bambu_AMS_2 is free but saved for the recorded ZZZZ, which ASSIGN
        # refuses; the popup offers Bambu_AMS_4 alone.
        master, bridge = self._saved_pool(tmp_path, monkeypatch, (A_p5, C_p5), (C_p5,),
                                          pool_ams=4)
        master._state_set({SEC: {"roster": f"boxed:{A}, boxed:{Z}"}})
        master._name_map[Z] = "Bambu_AMS_2"
        self._at(master, bridge, 100.0)
        assert self._bay(master, "Bambu_AMS_3")["bound"] == C_p5
        assert self._log(master) == self._claim_log(C_p5, "Bambu_AMS_3")
        assert self._console(master) == self._new_popup(C_p5, "Bambu_AMS_3",
                                                        "Bambu_AMS_4")
        with pytest.raises(Exception) as err:
            master.printer.gcode.run("AFC_BRIDGEBOX_ASSIGN", CHAIN="chain1",
                                     UID=C_p5, NAME="Bambu_AMS_2")
        assert str(err.value) == (
            "AFC_BRIDGEBOX_ASSIGN: bay 'Bambu_AMS_2' is saved for ZZZZ -- AFC_BRIDGEBOX_FORGET "
            "CHAIN=chain1 UID=ZZZZ or AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=ZZZZ first"
        )
        assert self._bay(master, "Bambu_AMS_2")["uid"] is None
        assert self._bay(master, "Bambu_AMS_3")["bound"] == C_p5

    # ── auto_drop keeps a unit with a lane in a toolhead claimed ─────────

    def _dropping(self, tmp_path: pathlib.Path,
                  monkeypatch: pytest.MonkeyPatch
                  ) -> Tuple[afcBridgeBox, FakeBridge, Dict[str, Any]]:
        """
        boxed:AAAA claimed at 0 with auto_drop, then read offline.

        :return tuple: (master, bridge, its bay)
        """
        master, bridge, bay = self._claimed(tmp_path, monkeypatch,
                                            auto_drop=True)
        self._online(bridge, [False])
        return master, bridge, bay

    @staticmethod
    def _load(master: afcBridgeBox, name: str) -> Any:
        """
        Record lane ``name`` loaded to the extruder, as set_tool_loaded
        leaves both halves of the record.

        :return AFCLane: the lane
        """
        lane = master.printer.lookup_object(f"AFC_lane {name}")
        lane.tool_loaded = True
        lane.status = AFCLaneState.TOOLED
        master.printer.afc.tools["extruder"].lane_loaded = name
        return lane

    @staticmethod
    def _held(lane: str, unset: str = "UNSET_LANE_LOADED") -> LogLine:
        """
        :param unset: the unload step the line names
        :return tuple: the line saying boxed:AAAA stays claimed for ``lane``
        """
        return ("info",
                "AFC_BridgeBox chain1: Bambu_AMS_1 (UID AAAA) is offline but "
                f"AFC records {lane} as loaded to the toolhead -- keeping it "
                f"claimed. Unload it ({unset} if the filament is already out) "
                "and it is released on the next check.")

    def test_a_unit_with_a_lane_in_the_toolhead_stays_claimed(
            self, tmp_path, monkeypatch):
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        lane = self._load(master, "lane25")
        self._tick(master, 1.0, 6.0, 11.0, 16.0, 21.0, 26.0)
        assert bay["bound"] == A_p5
        assert lane.unassigned is False
        assert master.printer.afc.lanes["lane25"] is lane
        assert master.printer.afc.tools["extruder"].lane_loaded == "lane25"
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        assert (unit.pool, unit.unit_uid) == (False, A_p5)
        assert master._drop_held == {A_p5}
        assert self._log(master) == [
            self._held("lane25"),
        ]
        assert self._console(master) == []

    def test_the_hold_is_logged_once_not_every_tick(
            self, tmp_path, monkeypatch):
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        self._load(master, "lane24")
        self._tick(master, 1.0, 6.0, 11.0, 16.0, 21.0, 26.0, 31.0)
        assert bay["bound"] == A_p5
        assert self._log(master) == [
            self._held("lane24"),
        ]
        assert self._console(master) == []

    def test_it_releases_on_the_next_tick_after_the_unload(
            self, tmp_path, monkeypatch):
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        self._load(master, "lane26")
        self._tick(master, 1.0, 6.0, 11.0, 16.0)
        assert bay["bound"] == A_p5
        master.printer.afc.function.unset_lane_loaded()
        self._tick(master, 21.0)
        assert bay["bound"] is None
        lanes = [master.printer.lookup_object(f"AFC_lane lane{n}")
                 for n in range(24, 28)]
        assert [ln.unassigned for ln in lanes] == [True] * 4
        unit = master.printer.lookup_object("AFC_BambuAMS Bambu_AMS_1")
        assert (unit.pool, unit.unit_uid) == (True, None)
        assert self._log(master) == [
            self._held("lane26"),
            ("debug", "Activating extruder lane: None"),
            ("info", "Manually removing lane26 loaded from toolhead"),
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
        ]
        assert self._console(master) == self.REMOVED_AAAA

    def test_the_hold_names_a_toolhead_that_is_not_active(
            self, tmp_path, monkeypatch):
        # UNSET_LANE_LOADED clears the active tool's lane only.
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        self._load(master, "lane24")
        other = add_extruder(master.printer, "extruder0")
        other.lane_loaded = "lane1"
        master.printer.toolhead.set_extruder(
            master.printer.lookup_object("extruder0"), 0.0)
        assert master.printer.afc.function.get_current_extruder() == (
            "extruder0")
        self._tick(master, 1.0, 6.0, 11.0, 16.0)
        assert bay["bound"] == A_p5
        assert self._log(master) == [
            self._held("lane24",
                       "UNSET_LANE_LOADED with extruder as the active tool"),
        ]
        assert self._console(master) == []

    def test_an_extruder_record_alone_holds_it(self, tmp_path, monkeypatch):
        # PREP restores lane_loaded from the var file; a pool lane's own
        # tool_loaded is not, so the extruder's half is all there may be.
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        master.printer.afc.tools["extruder"].lane_loaded = "lane27"
        self._tick(master, 1.0, 6.0, 11.0, 16.0)
        assert bay["bound"] == A_p5
        assert master.printer.lookup_object("AFC_lane lane27").tool_loaded is False
        assert self._log(master) == [
            self._held("lane27"),
        ]
        assert self._console(master) == []

    def test_a_replug_between_absences_logs_the_next_hold_again(
            self, tmp_path, monkeypatch):
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        self._load(master, "lane24")
        self._tick(master, 1.0, 6.0, 11.0, 16.0)
        self._at(master, bridge, 21.0, 26.0, 31.0, online=[True])
        assert master._drop_held == set()
        self._at(master, bridge, 36.0, 41.0, 46.0, 51.0, online=[False])
        assert bay["bound"] == A_p5
        assert self._log(master) == [
            self._held("lane24"),
            ("debug",
             "AFC_BridgeBox chain1: Bambu_AMS_1 (UID AAAA) is back after 15s offline and stayed "
             "bound; its loaded-lane follower restore engaged nothing."),
            self._held("lane24"),
        ]
        assert self._console(master) == []

    def test_nothing_in_the_toolhead_releases_as_before(
            self, tmp_path, monkeypatch):
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        saves = master.printer.afc.save_vars.call_count
        self._tick(master, 1.0, 6.0, 11.0, 16.0)
        assert bay["bound"] is None
        assert master._drop_held == set()
        assert master.printer.afc.tools["extruder"].lane_loaded is None
        assert master.printer.afc.save_vars.call_count - saves == 0
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
        ]
        assert self._console(master) == self.REMOVED_AAAA

    def test_a_print_holds_an_auto_drop_until_it_ends(self, tmp_path,
                                                      monkeypatch):
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        master.printer.set_print_state("printing")
        self._tick(master, 1.0, 6.0, 11.0, 16.0)
        assert bay["bound"] == A_p5
        assert master._released_at == {}
        assert self._log(master) == []
        assert self._console(master) == []
        master.printer.set_print_state("complete")
        self._tick(master, 21.0)
        assert bay["bound"] is None
        assert master._released_at == {A_p5: 21.0}
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
        ]
        assert self._console(master) == self.REMOVED_AAAA

    def test_a_print_read_without_a_clock_still_holds_an_auto_drop(
            self, tmp_path, monkeypatch):
        # print_stats is asked at time 0.0 when the clock cannot be read.
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        master.printer.set_print_state("printing")

        def _no_clock() -> float:
            raise RuntimeError("no clock")
        monkeypatch.setattr(master.printer.reactor, "monotonic", _no_clock)
        self._tick(master, 1.0, 6.0, 11.0, 16.0)
        assert bay["bound"] == A_p5
        assert master._watch_state == "watching"
        assert self._log(master) == []
        assert self._console(master) == []

    def test_the_hold_names_the_tool_when_the_active_one_cannot_be_read(
            self, tmp_path, monkeypatch):
        # With no toolhead to ask, the extruder holding the lane is not
        # known to be the active one, so the step names it.
        master, bridge, bay = self._dropping(tmp_path, monkeypatch)
        self._load(master, "lane24")
        add_extruder(master.printer, "extruder0")
        master.printer.afc.toolhead = None
        self._tick(master, 1.0, 6.0, 11.0, 16.0)
        assert bay["bound"] == A_p5
        assert self._log(master) == [
            self._held("lane24",
                       "UNSET_LANE_LOADED with extruder as the active tool"),
        ]
        assert self._console(master) == []

    # ── a watch that finds parts of the printer missing ──────────────────

    @staticmethod
    def _klippy_chain(tmp_path: pathlib.Path, printer: Any,
                      bridge: Optional[FakeBridge] = None,
                      **options: Any) -> afcBridgeBox:
        """
        chain1's master as klippy loads a section, by load_config_prefix;
        make_bridgebox builds it directly.

        :param printer: the printer
        :param bridge: the chain's bridge, registered under its port
        :return afcBridgeBox: the master, registered, logging to AFC's logger
        """
        if printer.afc.VarFile == NO_VAR_FILE:
            set_var_file(printer, str(tmp_path / "AFC.var"))
        opts = bridgebox_options(tmp_path, "chain1", **options)
        printer.add_section(SEC, {k: v for k, v in opts.items()
                                  if v is not None})
        master = load_config_prefix(BambuConfig(SEC, printer, opts))
        printer.add_object(SEC, master)
        master.logger = printer.afc.logger
        if bridge is not None:
            live_bridges()[master.serial_port] = bridge
        return master

    @pytest.mark.parametrize("legacy", ["entry", "comments only"])
    def test_a_roster_file_an_older_version_left_is_watched(
            self, tmp_path, monkeypatch, legacy):
        # The chain's buffer is a hand-written [AFC_buffer Bamb_1], so it
        # fabricates none.
        old = tmp_path / "AFC_BridgeBox_chain1.roster"
        old.write_text("# chain detected by [AFC_BridgeBox chain1]\n"
                       + (f"boxed:{A}\n" if legacy == "entry" else ""))
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True)
        master = self._klippy_chain(
            tmp_path, printer, bridge=FakeBridge(uids=[A_p5], online=[True]),
            roster="", pool_ams=1, pool_ht=0, buffer="Bamb_1")
        printer.connect()
        self._tick(master, 100.0, 116.0)
        assert self._bay(master, "Bambu_AMS_1")["bound"] == A_p5
        lane = printer.lookup_object("AFC_lane lane24")
        assert (lane.unassigned, lane.buffer_obj) == (False, None)
        if legacy == "entry":
            # Migrated, so AAAA is already recorded and back on its bay.
            assert self._log(master) == self._claim_log(A_p5, "Bambu_AMS_1")
            assert self._console(master) == []
            return
        # Nothing to migrate: the watch records AAAA and saves its bay.
        assert self._roster(master) == f"boxed:{A}"
        assert self._log(master) == [
            *self._claim_log(A_p5, "Bambu_AMS_1"),
            ("info",
             "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:AAAA -- recorded. A restart "
             "adds its temperature card."),
            ("info",
             "AFC_BridgeBox chain1: saved AAAA on Bambu_AMS_1 (lane24-lane27, T24-T27); it "
             "comes back there after a restart."),
        ]
        assert self._console(master) == self._new_popup(A_p5, "Bambu_AMS_1")

    def test_a_printer_without_afc_claims_nothing(
            self, tmp_path, monkeypatch):
        # AAAA is recorded on Bambu_AMS_1 and online, but AFC is gone.
        master = self._chain(tmp_path, monkeypatch,
                             bridge=FakeBridge(uids=[A_p5], online=[True]),
                             recorded=f"boxed:{A}", roster=f"boxed:{A}",
                             pool_ams=1, pool_ht=0, auto_drop=True)
        assert self._log(master) == []
        master.printer._afc = None
        self._tick(master, 100.0, 101.0)
        bay = self._bay(master, "Bambu_AMS_1")
        # The bay is AAAA's from the roster, yet AAAA is never claimed.
        assert (bay["bound"], bay["uid"]) == (None, A_p5)
        assert master._online_since == {A_p5: 100.0}
        assert (master._tick_count, master._watch_state) == (2, "watching")
        assert self._log(master) == []
        assert self._console(master) == []

    class _BareUnit:
        """A pool unit with only claim and release; its release fails."""

        type = "AFC_BambuAMS"

        def __init__(self, name: str) -> None:
            self.name = name
            self.pool = True
            self.unit_uid: Optional[str] = None
            self.lanes: Dict[str, Any] = {}
            self.claims: List[Tuple[str, str]] = []

        def claim(self, uid: str, model: str) -> bool:
            """:return bool: always claimed"""
            self.claims.append((uid, model))
            self.pool, self.unit_uid = False, uid
            return True

        def release(self) -> None:
            """:raises RuntimeError: always"""
            raise RuntimeError("bus gone")

    class _BareLane:
        """A lane with no hub, extruder, buffer or config, whose record
        cannot be read."""

        def __init__(self, name: str, unit: Any, afc: Any) -> None:
            self.name = name
            self.fullname = f"AFC_lane {name}"
            self.unit_obj, self.afc = unit, afc
            self.hub_obj = self.extruder_obj = self.buffer_obj = None
            self.unassigned = True
            self.tool_loaded = False
            self.map: List[str] = []
            self._map: List[str] = []
            self.current_map = ""

        def get_status(self, save_to_file: bool = False) -> Dict[str, Any]:
            """:raises RuntimeError: always"""
            raise RuntimeError("no record")

    def test_a_bay_with_bare_parts_claims_returns_and_releases(
            self, tmp_path, monkeypatch):
        # Bambu_AMS_1's unit and two of its lanes are bare; the other two
        # lanes are missing. The console is gone too.
        bridge = FakeBridge(uids=[A_p5], online=[True])
        master = self._chain(tmp_path, monkeypatch, bridge=bridge,
                             recorded=f"boxed:{A}", roster=f"boxed:{A}",
                             pool_ams=1, pool_ht=0, auto_drop=True,
                             release_grace=10.0, release_settle=5.0)
        printer = master.printer
        unit = self._BareUnit("Bambu_AMS_1")
        printer._objects["AFC_BambuAMS Bambu_AMS_1"] = unit
        lanes = [self._BareLane(n, unit, printer.afc)
                 for n in ("lane24", "lane25")]
        for lane in lanes:
            printer._objects[lane.fullname] = lane
        del printer._objects["AFC_lane lane26"]
        del printer._objects["AFC_lane lane27"]
        printer._gcode = None
        self._tick(master, 0.0)
        bay = self._bay(master, "Bambu_AMS_1")
        assert bay["bound"] == A_p5
        assert unit.claims == [(A_p5, "boxed")]
        assert [ln.unassigned for ln in lanes] == [False, False]
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: CLAIMED AAAA as boxed onto Bambu_AMS_1 (2 lanes) -- live, "
             "no restart."),
        ]
        self._log(master).clear()
        self._at(master, bridge, 1.0, 4.0, online=[False])
        self._at(master, bridge, 5.0, 10.0, online=[True])
        assert master._last_online == {A_p5: 10.0}
        assert self._log(master) == []
        self._at(master, bridge, *range(11, 22), online=[False])
        assert bay["bound"] is None
        assert [ln.unassigned for ln in lanes] == [True, True]
        assert (unit.pool, unit.unit_uid) == (False, A_p5)
        assert self._log(master) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_1 (UID AAAA offline >10s); lanes dropped "
             "live, slot kept for re-plug"),
        ]
        assert self._console(master) == []

    def test_a_scout_whose_chip_name_is_taken_still_watches(
            self, tmp_path, monkeypatch):
        printer = make_printer(monkeypatch=monkeypatch, fabricate=True)
        printer.add_object("pins", self._Pins(["bambu_buffer"]))
        master = make_bridgebox(tmp_path, printer=printer, roster="")
        printer.connect()
        live_bridges()[master.serial_port] = FakeBridge(
            uids=[HT_UID], online=[True], htmask=0b1)
        self._tick(master, 100.0, 115.0)
        assert self._roster(master) == HT_ROSTER
        assert self._log(master) == [
            ("info",
             f"AFC_BridgeBox chain1: chain reports [ht:{HT_UID}] -- written to "
             "AFC_BridgeBox.cfg. RESTART to enroll, or copy it into roster: to pin it."),
        ]
        assert self._console(master) == []

    class _Pins:
        """Klipper's pin registry, refusing a chip name already taken."""

        def __init__(self, taken: List[str]) -> None:
            self.chips = {name: object() for name in taken}

        def register_chip(self, name: str, chip: Any) -> None:
            """:raises configparser.Error: for a name already registered"""
            if name in self.chips:
                raise configparser.Error(f"Duplicate chip name '{name}'")
            self.chips[name] = chip


class TestAfcBridgeBoxPrompt:
    """Each popup's auto-dismiss closes that popup only."""

    def test_a_stale_dismiss_timer_does_not_close_a_later_popup(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, ready=False)
        assert m._claim_pool_unit(C, "boxed") is not None
        reactor = m.printer.reactor
        m._prompt_new_unit(C)
        close1, when1 = reactor.callbacks[-1]
        assert (m._prompt_gen, when1) == (1, reactor.now + 45.0)
        m.cmd_AFC_BRIDGEBOX_BAYS(FakeGcmd({}))
        close2, _when2 = reactor.callbacks[-1]
        assert m._prompt_gen == 2
        _p6_quiet(m)
        close1(0.0)
        assert _p6_console(m) == []
        close2(0.0)
        assert _p6_console(m) == [END]
        assert _p6_log(m) == []


class TestAfcBridgeBoxPromptNewUnit:
    """The bay picker of a unit that already sits on a bay."""

    def test_new_unit_popup_offers_only_free_same_family_bays(
            self, tmp_path, monkeypatch):
        # CCCC on the Bravo spare: Alpha is AAAA's, Hot is an HT bay.
        m = _p6_named(tmp_path, monkeypatch, ready=False)
        assert m._claim_pool_unit(C, "boxed") is not None
        _p6_quiet(m)
        m._prompt_new_unit(C)
        assert _p6_console(m) == _p6_popup(
            "New AMS on Bravo",
            ["UID CCCC is on 'Bravo' (its T# and lanes are live).",
             "It is saved on this bay once it has been online 15s.",
             "Keep it here, or move it to another named bay:"],
            ["Charlie|AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=CCCC "
             "NAME=Charlie|primary"])
        assert _p6_log(m) == []

    def test_the_new_unit_popup_offers_no_bay_to_an_unlisted_uid(
            self, tmp_path, monkeypatch):
        m = _p6_chain(tmp_path, monkeypatch, roster=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, ready=False)
        assert m._claim_pool_unit(C, "boxed") is not None
        assert _p6_bay(m, "Bambu_AMS_2")["bound"] == C
        _p6_quiet(m)
        m._prompt_new_unit(C)
        assert _p6_console(m) == _p6_popup(
            "New AMS on Bambu_AMS_2",
            ["UID CCCC is on 'Bambu_AMS_2' (its T# and lanes are live).",
             "roster: is set and does not list it. Add boxed:CCCC to "
             "roster: to pin it to a named bay."])
        assert _p6_log(m) == []


class TestAfcBridgeBoxPromptRemovedUnit:
    """The popup for a unit dropped on unplug."""

    def test_removed_unit_popup_offers_forget(self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, ready=False)
        _p6_quiet(m)
        m._prompt_removed_unit(A, "Alpha")
        assert _p6_console(m) == _p6_popup(
            "AMS removed: Alpha",
            ["Alpha (UID AAAA) was unplugged; its bay is held for a "
             "re-plug.",
             "Re-plug it and it reclaims the same lanes/T#. Or forget it to "
             "free the bay to the pool:"],
            ["Forget Alpha|AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=AAAA|error"])
        assert _p6_log(m) == []

    def test_removed_popup_says_held_only_for_a_reserved_bay(
            self, tmp_path, monkeypatch):
        # CCCC sat on the Bravo spare, which its release hands back to the
        # pool; AAAA is recorded, so its bay keeps its uid.
        m = _p6_named(tmp_path, monkeypatch, ready=False)
        assert m._claim_pool_unit(C, "boxed") is not None
        m._release_pool_unit(C)
        assert _p6_bay(m, "Bravo")["uid"] is None
        _p6_quiet(m)
        m._prompt_removed_unit(C, "Bravo")
        assert _p6_console(m) == _p6_popup(
            "AMS removed: Bravo",
            ["Bravo (UID CCCC) was unplugged; its bay went back to the "
             "pool.",
             "A re-plug takes it back while it is free, else the lowest free "
             "bay of its family. Or forget it if it is not coming back:"],
            ["Forget Bravo|AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=CCCC|error"])
        assert _p6_log(m) == []

    def test_a_unit_without_a_name_is_shown_by_its_uid(self, tmp_path,
                                                       monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, ready=False)
        _p6_quiet(m)
        m._prompt_removed_unit(C, None)
        assert _p6_console(m) == _p6_popup(
            "AMS removed: CCCC",
            ["CCCC (UID CCCC) was unplugged; its bay went back to the pool.",
             "A re-plug takes it back while it is free, else the lowest free "
             "bay of its family. Or forget it if it is not coming back:"],
            ["Forget CCCC|AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=CCCC|error"])
        assert _p6_log(m) == []


class TestAfcBridgeBoxOfferReplace:
    """The replace picker queued for a unit waiting for a bay."""

    def test_the_watch_reads_no_state_file_to_make_an_offer(
            self, tmp_path, monkeypatch):
        m = _p6_stuck(tmp_path, monkeypatch)
        _p6_tick(m, 100, 114)
        assert (getattr(m, "_popup_queue", None), m._no_bay) == (
            None, {E: "boxed"})
        _p6_quiet(m)

        def _read(*args: Any) -> None:
            """:raises AssertionError: the state file is read"""
            raise AssertionError("state file read")
        monkeypatch.setattr(m, "_read_state", _read)
        m._offer_replace(115.0, {A, B, C, E, H}, m._online_since)
        assert m._popup_queue == [("replace", E)]
        assert m._replace_offered == {E}
        assert _p6_log(m) == []
        assert _p6_console(m) == []


class TestAfcBridgeBoxQueuePopup:
    """Popups queue in order, one per (kind, uid)."""

    def test_the_queue_dedupes_a_flapping_uid(self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, ready=False)
        _p6_quiet(m)
        m._queue_popup(("new", C))
        m._queue_popup(("new", D))
        m._queue_popup(("new", C))
        assert m._popup_queue == [("new", D), ("new", C)]
        assert _p6_console(m) == []
        assert _p6_log(m) == []


class TestAfcBridgeBoxPumpPopups:
    """One queued popup shows per hold."""

    def test_the_queue_shows_one_popup_per_hold(self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, ready=False)
        assert m._claim_pool_unit(C, "boxed") is not None
        assert m._claim_pool_unit(D, "boxed") is not None
        m._queue_popup(("new", C))
        m._queue_popup(("new", D))
        _p6_quiet(m)

        def _shown(uid: str, bay: str) -> List[LogLine]:
            """:return list: the bay picker of ``uid`` on ``bay``"""
            return _p6_popup(
                f"New AMS on {bay}",
                [f"UID {uid} is on '{bay}' (its T# and lanes are live).",
                 "It is saved on this bay once it has been online 15s.",
                 "No other free bay of this type to move it to."])
        m._pump_popups(0.0)
        assert _p6_console(m) == _shown(C, "Bravo")
        assert (m._popup_queue, m._popup_active_until) == ([("new", D)],
                                                           20.0)
        m._pump_popups(1.0)
        assert _p6_console(m) == _shown(C, "Bravo")
        assert m._popup_queue == [("new", D)]
        m._pump_popups(20.0)
        assert _p6_console(m) == _shown(C, "Bravo") + _shown(D, "Charlie")
        assert (m._popup_queue, m._popup_active_until) == ([], 40.0)
        assert _p6_log(m) == []

    def test_a_pump_of_a_replace_event_shows_the_picker(self, tmp_path,
                                                        monkeypatch):
        m = _p6_stuck(tmp_path, monkeypatch)
        _p6_tick(m, 100, 111)
        _p6_quiet(m)
        m._queue_popup(("replace", E))
        m._pump_popups(111.0)
        assert _p6_console(m) == _p6_popup(
            "No free bay for new AMS",
            ["UID EEEE has no free bay: every AMS bay is taken.",
             "Replace a unit that is offline: the new one takes its bay, "
             "lanes and T# now, and the old one is forgotten (its learned "
             "values and saved lane records, spools included, are erased).",
             "Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 11s",
             "Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE opens this again."],
            ["Replace Bambu_AMS_4|AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
             "UID=EEEE OLD=DDDD|error"])
        assert (m._popup_queue, m._popup_active_until) == ([], 131.0)
        assert _p6_log(m) == []


class TestAfcBridgeBoxRestoreReturned:
    """A unit back while still bound runs its own follower restore."""

    def test_the_return_restore_reaches_the_bound_unit(self, tmp_path,
                                                       monkeypatch):
        # The claim's own restore is still due, so the unit engages nothing
        # and sends nothing.
        m = _p6_named(tmp_path, monkeypatch)
        unit = m._claim_pool_unit(A, "boxed")
        assert unit._loaded_restore_done is False
        bridge = live_bridges()[m.serial_port]
        bridge.sent.clear()
        _p6_quiet(m)
        m._restore_returned(A, 100.0, 5.0)
        assert _p6_log(m) == [(
            "debug",
            "AFC_BridgeBox chain1: Alpha (UID AAAA) is back after 5s offline "
            "and stayed bound; its loaded-lane follower restore engaged "
            "nothing.")]
        assert (unit._loaded_restore_done, bridge.sent) == (False, [])

    def test_a_unit_that_owns_the_loaded_lane_engages_its_follower(
            self, tmp_path, monkeypatch):
        # The claim's own restore has run and AFC records lane24 loaded to
        # the toolhead: the unit engages and says so, and so does the master.
        m = _p6_named(tmp_path, monkeypatch)
        unit = m._claim_pool_unit(A, "boxed")
        unit._id_resolved = True
        unit._loaded_restore_done = True
        m.printer.afc.tools["extruder"].lane_loaded = "lane24"
        _p6_lane(m, "lane24").tool_loaded = True
        assert getattr(unit, "_loaded_restore_at", None) is None
        bridge = live_bridges()[m.serial_port]
        bridge.sent.clear()
        m.printer.reactor.now = 120.0
        _p6_quiet(m)
        m._restore_returned(A, 100.0, 5.0)
        assert _p6_log(m) == [
            ("debug",
             "AFC bambu Alpha: lane24 loaded at startup, re-asserting AMS "
             "loaded state + follower"),
            ("info",
             "AFC bambu Alpha: restored the follower for lane24 (AFC records "
             "it loaded to the toolhead) -- mode:4, one-shot."),
            ("debug",
             "AFC_BridgeBox chain1: Alpha (UID AAAA) is back after 5s offline "
             "and stayed bound; its loaded-lane follower restore engaged.")]
        # finish (three mode-07 frames), then select, then assist.
        assert bridge.sent == [
            {"cmd": "raw", "hex": "3DC50CC803000900A502800C"},
            {"cmd": "raw", "hex": "3DC50CC8030007000002514C"},
            {"cmd": "raw", "hex": "3DC50CC8030007007F023654"},
            {"cmd": "select", "unit": 0, "slot": 0},
            {"cmd": "assist", "unit": 0, "slot": 0, "on": True}]
        assert (unit._loaded_restore_done, unit._loaded_restore_at) == (
            True, 120.0)

    def test_a_uid_on_no_bay_restores_nothing(self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        _p6_quiet(m)
        m._restore_returned(C, 100.0, 5.0)
        assert _p6_log(m) == []

    def test_a_restore_that_fails_is_a_warning(self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        unit = m._claim_pool_unit(A, "boxed")

        def _fail(since: float) -> bool:
            """:raises RuntimeError: the restore fails"""
            raise RuntimeError("bus busy")
        monkeypatch.setattr(unit, "restore_follower_on_return", _fail)
        _p6_quiet(m)
        m._restore_returned(A, 100.0, 5.0)
        assert _p6_log(m) == [(
            "warning",
            "AFC_BridgeBox chain1: follower restore for Alpha (UID AAAA) on "
            "its return failed: bus busy")]


class TestAfcBridgeBoxAnnounceEnrolled:
    """The line for units just recorded on a pooled chain."""

    def test_a_unit_on_a_built_bay_is_promised_no_temperature_card(
            self, tmp_path, monkeypatch):
        m = _p6_stuck(tmp_path, monkeypatch)
        _p6_tick(m, 100, 111)
        m.cmd_AFC_BRIDGEBOX_REPLACE(FakeGcmd({"UID": E, "OLD": D}))
        assert _p6_bay(m, "Bambu_AMS_4")["bound"] == E
        _p6_quiet(m)
        m._announce_enrolled([f"boxed:{E}"])
        assert _p6_log(m) == [(
            "info",
            "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:EEEE -- "
            "recorded.")]

    def test_a_unit_on_a_spare_bay_is_promised_its_temperature_card(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(C, "boxed") is not None
        assert _p6_bay(m, "Bravo")["spare"] is True
        _p6_quiet(m)
        m._announce_enrolled([f"boxed:{C}"])
        assert _p6_log(m) == [(
            "info",
            "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- "
            "recorded. A restart adds its temperature card.")]


class TestAfcBridgeBoxPinRecordedUnits:
    """Saving the bay a recorded unit holds."""

    def test_a_unit_saved_on_another_bay_of_this_session_keeps_that_pin(
            self, tmp_path, monkeypatch):
        # ZZZZ is saved on Bambu_AMS_2 but live on Bambu_AMS_3, because
        # XXXX sat on its bay when it came back: it goes back to its bay at
        # restart, and XXXX, on a bay saved for ZZZZ, is not saved there.
        m = _p6_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1)
        m._name_map["ZZZZ"] = "Bambu_AMS_2"
        for name, uid in (("Bambu_AMS_2", "XXXX"), ("Bambu_AMS_3", "ZZZZ")):
            _p6_bay(m, name)["uid"] = _p6_bay(m, name)["bound"] = uid
        before = (tmp_path / "AFC_BridgeBox.cfg").read_bytes()
        _p6_quiet(m)
        recorded = {A, "XXXX", "ZZZZ"}
        for t in (100.0, 101.0):
            m._pin_recorded_units(recorded, t, {"XXXX": 0.0, "ZZZZ": 0.0})
        assert _p6_log(m) == [
            ("warning",
             "AFC_BridgeBox chain1: Bambu_AMS_2 is saved for ZZZZ, so XXXX is "
             "not saved on it. AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=ZZZZ or "
             "AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=ZZZZ, or "
             "AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=XXXX NAME=<other bay> "
             "moves XXXX to another bay."),
            ("info",
             "AFC_BridgeBox chain1: ZZZZ is on Bambu_AMS_3 this session, not "
             "on Bambu_AMS_2, the bay it is saved on; it comes back to "
             "Bambu_AMS_2 after a restart.")]
        assert m._pin_warned == {("XXXX", "Bambu_AMS_2", "holder"),
                                 ("ZZZZ", "Bambu_AMS_3", "elsewhere")}
        assert m._name_map == {A: "Bambu_AMS_1", "ZZZZ": "Bambu_AMS_2"}
        assert (tmp_path / "AFC_BridgeBox.cfg").read_bytes() == before


class TestAfcBridgeBoxReleasePoolUnit:
    """The hot-unplug half: a bay's lanes dropped live, their records held."""

    #: What a release of AAAA from its saved bay Alpha logs.
    RELEASED = ("info",
                "AFC_BridgeBox chain1: released Alpha (UID AAAA offline "
                ">10s); lanes dropped live, slot kept for re-plug")

    @staticmethod
    def _loaded(tmp_path: pathlib.Path,
                monkeypatch: pytest.MonkeyPatch) -> afcBridgeBox:
        """
        AAAA claimed onto Alpha, lane24 then given spool 159, PLA, blue.

        :return afcBridgeBox: the master
        """
        m = _p6_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        lane = _p6_lane(m, "lane24")
        lane.spool_id, lane.material, lane.color = 159, "PLA", "#0086D6"
        return m

    @staticmethod
    def _tools(master: afcBridgeBox) -> List[str]:
        """:return list: the T# commands registered on the gcode, sorted"""
        return sorted(cmd for cmd in master.printer.gcode.ready_gcode_handlers
                      if cmd[:1] == "T" and cmd[1:].isdigit())

    def test_a_release_holds_the_lanes_records_with_their_maps(
            self, tmp_path, monkeypatch):
        m = self._loaded(tmp_path, monkeypatch)
        unit = _p6_unit(m, "Alpha")
        _p6_quiet(m)
        m._release_pool_unit(A)
        held = m._held["Alpha"]
        assert held["uid"] == A
        assert sorted(held["lanes"]) == ["lane24", "lane25", "lane26",
                                         "lane27"]
        rec = held["lanes"]["lane24"]
        assert (rec["spool_id"], rec["material"], rec["color"]) == (
            159, "PLA", "#0086D6")
        assert (rec["map"], rec["current_map"]) == ("T24", "T24")
        assert held["lanes"]["lane25"]["map"] == "T25"
        assert m.get_status()["held_bays"] == {"Alpha": A}
        # The lanes are dropped live and the unit is idle again; AAAA is
        # recorded on Alpha, so the bay keeps its uid for a re-plug.
        assert (m.printer.afc.lanes, m.printer.afc.tool_cmds) == ({}, {})
        assert self._tools(m) == []
        assert (unit.pool, unit.unit_uid) == (True, None)
        assert (_p6_bay(m, "Alpha")["uid"], _p6_bay(m, "Alpha")["bound"]) == (
            A, None)
        assert _p6_log(m) == [self.RELEASED]

    def test_records_the_unit_had_not_settled_are_held_again(
            self, tmp_path, monkeypatch):
        # The unit never primes, so what the claim put on lane24 is still
        # what was saved; the user then moves it to T7 (SET_MAP) and sets its
        # colour and weight, and all of it is held.
        var = {"Alpha": {"lane24": _p6_rec("T24", spool_id=159,
                                           material="PETG")}}
        m = _p6_named(tmp_path, monkeypatch, var=var, owners=f"{A}:Alpha")
        assert m._claim_pool_unit(A, "boxed") is not None
        lane = _p6_lane(m, "lane24")
        assert (lane.spool_id, lane.material) == (159, "PETG")
        afc = m.printer.afc
        afc.tool_cmds.pop("T24")
        afc.gcode.register_command("T24", None)
        lane.map, lane.current_map = ["T7"], "T7"
        afc.tool_cmds["T7"] = "lane24"
        afc.gcode.register_command("T7", afc.cmd_CHANGE_TOOL)
        lane.color, lane.weight = "#123456", 640.0
        _p6_quiet(m)
        m._release_pool_unit(A)
        rec = m._held["Alpha"]["lanes"]["lane24"]
        assert (rec["spool_id"], rec["material"]) == (159, "PETG")
        assert (rec["map"], rec["current_map"]) == ("T7", "T7")
        assert (rec["color"], rec["weight"]) == ("#123456", 640.0)
        assert (lane.spool_id, lane.color, lane.map) == (None, "", [])
        assert _p6_log(m) == [self.RELEASED]
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert (lane.spool_id, lane.color, lane.weight, lane.map) == (
            159, "#123456", 640.0, ["T7"])
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane24->T7.")

    def test_the_release_save_of_a_toolhead_lane_keeps_them(
            self, tmp_path, monkeypatch):
        # lane24 is in the extruder: the release clears that record and
        # saves once the bay is unbound, and that save writes the records
        # held for AAAA, taken before the record was cleared.
        var = {"Alpha": {"lane24": _p6_rec("T24", spool_id=159,
                                           material="PLA", color="#0086D6",
                                           weight=412.0),
                         "lane25": _p6_rec("T25", material="PETG")}}
        m = _p6_named(tmp_path, monkeypatch, var=var, owners=f"{A}:Alpha")
        assert m._claim_pool_unit(A, "boxed") is not None
        drain_var_writes(m.printer)
        afc = m.printer.afc
        afc.tools["extruder"].lane_loaded = "lane24"
        lane = _p6_lane(m, "lane24")
        lane.tool_loaded = True
        _p6_quiet(m)
        m._release_pool_unit(A)
        held = m._held["Alpha"]["lanes"]
        assert (held["lane24"]["spool_id"], held["lane24"]["tool_loaded"]) == (
            159, True)
        snaps = drain_var_writes(m.printer)
        # AFC's unload saves while the lanes are live; the release's own
        # save is the last, with the bay pooled.
        assert [s["system"]["num_lanes"] for s in snaps] == [4, 4, 4, 4, 4, 0]
        last = snaps[-1]
        assert last["Alpha"] == held
        assert last["system"]["extruders"] == {
            "extruder": {"lane_loaded": None}}
        assert (afc.tools["extruder"].lane_loaded, lane.tool_loaded) == (
            None, False)
        assert _p6_log(m) == [
            ("debug", "Activating extruder lane: None"),
            ("info", "Manually removing lane24 loaded from toolhead"),
            ("info",
             "AFC_BridgeBox chain1: released Alpha (UID AAAA offline >10s); "
             "lanes dropped live, slot kept for re-plug; cleared lane24 from "
             "the toolhead")]

    def test_a_lane_another_extruder_holds_is_cleared_there(
            self, tmp_path, monkeypatch):
        # lane24 is loaded in extruder1; the active extruder holds nothing.
        m = self._loaded(tmp_path, monkeypatch)
        afc = m.printer.afc
        ext1 = add_extruder(m.printer, "extruder1")
        ext1.lane_loaded = "lane24"
        lane = _p6_lane(m, "lane24")
        lane.tool_loaded = True
        _p6_quiet(m)
        m._release_pool_unit(A)
        assert (ext1.lane_loaded, afc.tools["extruder"].lane_loaded) == (
            None, None)
        assert lane.tool_loaded is False
        assert m._held["Alpha"]["lanes"]["lane24"]["tool_loaded"] is True
        assert _p6_log(m) == [
            ("info",
             "AFC_BridgeBox chain1: released Alpha (UID AAAA offline >10s); "
             "lanes dropped live, slot kept for re-plug; cleared lane24 from "
             "the toolhead")]

    def test_a_release_drops_the_restored_tool_and_not_the_home_one(
            self, tmp_path, monkeypatch):
        # SET_MAP left lane24 on T3 and gave its T24 to lane5.
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T3")}},
                      owners=f"{A}:Alpha")
        lane5 = _p6_other(m, "lane5", ["T24"])
        assert m._claim_pool_unit(A, "boxed") is not None
        afc = m.printer.afc
        assert (afc.tool_cmds["T3"], afc.tool_cmds["T24"]) == (
            "lane24", "lane5")
        _p6_quiet(m)
        m._release_pool_unit(A)
        assert afc.tool_cmds == {"T24": "lane5"}
        assert self._tools(m) == ["T24"]
        assert afc.gcode.ready_gcode_handlers["T24"] == afc.cmd_CHANGE_TOOL
        assert (lane5.map, lane5.sent) == (["T24"], [])
        _p6_consistent(m)
        assert _p6_log(m) == [self.RELEASED]

    def test_a_release_before_the_print_ends_drops_the_wait(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, print_state="printing")
        lane5 = _p6_other(m, "lane5", ["T24"])
        assert m._claim_pool_unit(A, "boxed") is not None
        assert list(m._deferred_takes) == ["lane24"]
        m.printer.set_print_state("complete")
        _p6_quiet(m)
        m._release_pool_unit(A)
        assert m._deferred_takes == {}
        m._take_deferred_tools()
        afc = m.printer.afc
        assert (lane5.map, afc.tool_cmds) == (["T24"], {"T24": "lane5"})
        # The next claim is handed the map this claim planned.
        rec = m._held["Alpha"]["lanes"]["lane24"]
        assert (rec["map"], rec["current_map"]) == ("T24", "T24")
        assert _p6_log(m) == [self.RELEASED]

    def test_release_keeps_a_saved_bay_and_frees_an_unsaved_one(
            self, tmp_path, monkeypatch):
        # CCCC is pinned to Bambu_AMS_2; EEEE sits on Bambu_AMS_3 unsaved.
        m = _p6_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1)
        m._persist_pin(C, 28, 4, "Bambu_AMS_2", "boxed")
        for name, uid in (("Bambu_AMS_2", C), ("Bambu_AMS_3", E)):
            _p6_bay(m, name)["uid"] = _p6_bay(m, name)["bound"] = uid
        _p6_quiet(m)
        m._release_pool_unit(C)
        m._release_pool_unit(E)
        assert (_p6_bay(m, "Bambu_AMS_2")["uid"],
                _p6_bay(m, "Bambu_AMS_2")["bound"]) == (C, None)
        assert (_p6_bay(m, "Bambu_AMS_3")["uid"],
                _p6_bay(m, "Bambu_AMS_3")["bound"]) == (None, None)
        # Neither bay had live lanes, so nothing is held.
        assert m._held == {}
        assert _p6_log(m) == [
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID CCCC offline "
             ">10s); lanes dropped live, slot kept for re-plug"),
            ("info",
             "AFC_BridgeBox chain1: released Bambu_AMS_3 (UID EEEE offline "
             ">10s); lanes dropped live")]

    def test_a_command_release_names_it_and_promises_no_replug(
            self, tmp_path, monkeypatch):
        m = self._loaded(tmp_path, monkeypatch)
        _p6_quiet(m)
        m._release_pool_unit(A, why="UNASSIGN")
        assert _p6_bay(m, "Alpha")["bound"] is None
        assert m._held["Alpha"]["uid"] == A
        assert _p6_log(m) == [(
            "info",
            "AFC_BridgeBox chain1: released Alpha (UID AAAA, UNASSIGN); lanes "
            "dropped live")]

    def test_a_uid_on_no_bay_releases_nothing(self, tmp_path, monkeypatch):
        m = self._loaded(tmp_path, monkeypatch)
        m._drop_held = {A, C}
        _p6_quiet(m)
        m._release_pool_unit(C)
        # Only the auto-drop hold of the uid itself goes.
        assert m._drop_held == {A}
        assert _p6_bay(m, "Alpha")["bound"] == A
        assert sorted(m.printer.afc.lanes) == ["lane24", "lane25", "lane26",
                                               "lane27"]
        assert m._held == {}
        assert _p6_log(m) == []

    def test_a_lane_whose_record_cannot_be_read_is_not_held(
            self, tmp_path, monkeypatch):
        m = self._loaded(tmp_path, monkeypatch)
        lane25 = _p6_lane(m, "lane25")
        real = lane25.get_status
        asked: List[bool] = []

        def _status(eventtime: Optional[float] = None,
                    save_to_file: bool = False) -> Dict[str, Any]:
            """:raises RuntimeError: the release's read of the record"""
            if save_to_file and not asked:
                asked.append(True)
                raise RuntimeError("status unreadable")
            return real(eventtime, save_to_file=save_to_file)
        monkeypatch.setattr(lane25, "get_status", _status)
        _p6_quiet(m)
        m._release_pool_unit(A)
        assert asked == [True]
        assert sorted(m._held["Alpha"]["lanes"]) == ["lane24", "lane26",
                                                     "lane27"]
        assert m._held["Alpha"]["uid"] == A
        assert _p6_bay(m, "Alpha")["bound"] is None
        assert _p6_log(m) == [self.RELEASED]


class TestAfcBridgeBoxLiveAccessTracking:
    """The config access-tracking dict fabricated sections share."""

    @staticmethod
    def _master(tmp_path: pathlib.Path, configfile: Any) -> afcBridgeBox:
        """
        :param configfile: the printer's configfile object; none when None
        :return afcBridgeBox: a master that builds nothing
        """
        printer = make_printer()
        m = make_bridgebox(tmp_path, printer=printer, roster="")
        if configfile is not None:
            printer.add_object("configfile", configfile)
        return m

    def test_it_finds_tracking_on_the_validate_object(self, tmp_path):
        # Where current Klipper keeps it.
        live: Dict[Any, Any] = {}
        cf = types.SimpleNamespace(
            validate=types.SimpleNamespace(access_tracking=live),
            access_tracking={})
        m = self._master(tmp_path, cf)
        assert m._live_access_tracking() is live
        assert _p6_log(m) == []

    def test_it_finds_tracking_on_configfile_itself(self, tmp_path):
        # Where older Klipper keeps it.
        live: Dict[Any, Any] = {}
        m = self._master(tmp_path,
                         types.SimpleNamespace(access_tracking=live))
        assert m._live_access_tracking() is live
        assert _p6_log(m) == []

    @pytest.mark.parametrize("cf", [
        None, types.SimpleNamespace(),
        types.SimpleNamespace(validate=None, access_tracking=None)])
    def test_a_shape_it_does_not_recognise_is_not_fatal(self, tmp_path, cf):
        m = self._master(tmp_path, cf)
        assert m._live_access_tracking() == {}
        assert _p6_log(m) == []

    def test_a_lookup_that_raises_is_logged_and_not_fatal(
            self, tmp_path, monkeypatch):
        m = self._master(tmp_path, None)

        def _lookup(name: str, default: Any = None) -> Any:
            """:raises RuntimeError: the printer cannot be asked"""
            raise RuntimeError("shutdown")
        monkeypatch.setattr(m.printer, "lookup_object", _lookup)
        assert m._live_access_tracking() == {}
        assert _p6_log(m) == [(
            "debug",
            "AFC_BridgeBox chain1: no access tracking to share (shutdown); "
            "fabricated sections stay out of configfile.settings")]


class TestAfcBridgeBoxClaimPoolUnit:
    """A live claim of a pool bay."""

    A24, B24 = "A" * 24, "B" * 24

    # ── the dryer and measure settings the claimed model gets ────────────

    def test_a_boxed_spare_claimed_as_ams2_gets_the_same(self, tmp_path,
                                                         monkeypatch):
        m = _p6_chain(tmp_path, monkeypatch, pool_ams=1, sections={
            "AFC_BridgeBox ams2": {"dry_max_temp": "50"}})
        unit = _p6_unit(m, "Bambu_AMS_1")
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == (
            "boxed", False, 65)
        _p6_quiet(m)
        assert m._claim_pool_unit(self.A24, "ams2") is unit
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == (
            "ams2", True, 50)
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1", "ams2")

    def test_a_claim_as_the_built_model_leaves_the_ceiling(self, tmp_path,
                                                           monkeypatch):
        m = _p6_chain(tmp_path, monkeypatch, pool_ams=1, sections={
            "AFC_BridgeBox ams2": {"dry_max_temp": "50"}})
        unit = _p6_unit(m, "Bambu_AMS_1")
        _p6_quiet(m)
        assert m._claim_pool_unit(self.A24, "boxed") is unit
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == (
            "boxed", False, 65)
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1")

    def test_replug_after_a_live_refine_keeps_the_ceiling(self, tmp_path,
                                                          monkeypatch):
        # heater: False and a lower ceiling for ams2; the unit is claimed as
        # boxed, the bus confirms ams2, then it is pulled and re-plugged.
        # claim() turns the heater back on from the model table; the re-plug
        # still ends with the heater off and the 50 a restart folds in.
        m = _p6_chain(tmp_path, monkeypatch, pool_ams=1, sections={
            "AFC_BridgeBox ams2": {"heater": "False", "dry_max_temp": "50"}})
        unit = _p6_unit(m, "Bambu_AMS_1")
        assert m._claim_pool_unit(self.A24, "boxed") is unit
        assert (unit.has_heater, unit.dry_max_temp) == (False, 65)
        m._apply_model_live(self.A24, "ams2", 0)
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == (
            "ams2", False, 50)
        m._release_pool_unit(self.A24)
        assert unit.pool is True
        _p6_quiet(m)
        assert m._claim_pool_unit(self.A24, "ams2") is unit
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == (
            "ams2", False, 50)
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1", "ams2")

    @pytest.mark.parametrize("model, section, name, pool", [
        ("ams2", "AFC_BridgeBox ams2", "Bambu_AMS_1", {"pool_ams": 1}),
        ("ams2", "AFC_BridgeBox Bambu_AMS_1", "Bambu_AMS_1", {"pool_ams": 1}),
        ("ht", "AFC_BridgeBox ht", "Bambu_AMS_HT_1", {"pool_ht": 1}),
    ])
    def test_a_same_model_claim_keeps_a_heater_override(
            self, tmp_path, monkeypatch, model, section, name, pool):
        # Every bay is claimed live after ready, a rostered one too, and
        # claim() takes the heater from the model table.
        m = _p6_chain(tmp_path, monkeypatch, roster=f"{model}:{self.A24}",
                      sections={section: {"heater": "False"}}, **pool)
        unit = _p6_unit(m, name)
        built = (unit.ams_model, unit.has_heater, unit.dry_max_temp)
        assert built[:2] == (model, False)
        _p6_quiet(m)
        assert m._claim_pool_unit(self.A24, model) is unit
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == built
        assert _p6_log(m) == _p6_claim_log(
            self.A24, name, model, 1 if model == "ht" else 4)

    def test_a_claim_does_not_keep_another_models_measure_setting(
            self, tmp_path, monkeypatch):
        # An AMS 1 claims the spare and takes its model section's
        # measure_on_insert; an AMS 2 claiming the bay next has no section
        # and gets the spare's default, not the AMS 1's setting.
        m = _p6_chain(tmp_path, monkeypatch, pool_ams=1, sections={
            "AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        assert _p6_bay(m, "Bambu_AMS_1")["spare"] is True
        unit = _p6_unit(m, "Bambu_AMS_1")
        at_claim: List[bool] = []
        real = unit.claim

        def _claim(uid: str, model: str) -> bool:
            """Record measure_on_insert as claim() finds it."""
            at_claim.append(unit.measure_on_insert)
            return real(uid, model)
        monkeypatch.setattr(unit, "claim", _claim)
        assert m._claim_pool_unit(self.A24, "ams1") is unit
        assert unit.measure_on_insert is True
        m._release_pool_unit(self.A24)
        _p6_quiet(m)
        assert m._claim_pool_unit(self.B24, "ams2") is unit
        assert unit.measure_on_insert is False
        assert _p6_log(m) == _p6_claim_log(self.B24, "Bambu_AMS_1", "ams2")
        # In place when each claim() ran: the index adoption the claim
        # starts sends it to the firmware from the attribute.
        assert at_claim == [True, False]

    def test_the_fabricated_measure_default_still_holds(self, tmp_path,
                                                        monkeypatch):
        # No section sets it: a known HT measures on insert, a spare stays
        # off.
        m = _p6_chain(tmp_path, monkeypatch, roster=f"ht:{self.A24}",
                      pool_ht=2)
        known, spare = (_p6_unit(m, "Bambu_AMS_HT_1"),
                        _p6_unit(m, "Bambu_AMS_HT_2"))
        assert (_p6_bay(m, "Bambu_AMS_HT_1")["spare"],
                _p6_bay(m, "Bambu_AMS_HT_2")["spare"]) == (False, True)
        known.measure_on_insert, spare.measure_on_insert = False, True
        _p6_quiet(m)
        assert m._claim_pool_unit(self.A24, "ht") is known
        assert m._claim_pool_unit(self.B24, "ht") is spare
        assert (known.measure_on_insert, spare.measure_on_insert) == (
            True, False)
        assert _p6_log(m) == (
            _p6_claim_log(self.A24, "Bambu_AMS_HT_1", "ht", 1)
            + _p6_claim_log(self.B24, "Bambu_AMS_HT_2", "ht", 1))

    # ── the home tool a Bambu lane takes ─────────────────────────────────

    @staticmethod
    def _home_pool(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                   other_map: Optional[Sequence[str]] = None
                   ) -> afcBridgeBox:
        """
        One AMS spare on lane24-lane27 (the roster HT sits on lane28), and
        lane5, a non-Bambu lane PREP numbered, on ``other_map``.

        :param other_map: lane5's T#s; no lane5 when None
        :return afcBridgeBox: the master
        """
        m = _p6_chain(tmp_path, monkeypatch, pool_ams=1)
        if other_map is not None:
            other = _p6_other(m, "lane5", other_map)
            other.current_map = other_map[-1]
        _p6_quiet(m)
        return m

    @staticmethod
    def _moved(tmp: str, now: str, tail: str = "") -> LogLine:
        """
        :param tmp: the T# lane24's claim took from lane5
        :param now: what lane5 is now
        :param tail: what the warning ends with
        :return tuple: the warning a claim taking lane24's home tool logs
        """
        return ("warning",
                f"AFC_BridgeBox chain1: {tmp} is the tool of Bambu lane "
                f"lane24 (Bambu lanes lane24-lane28 are T24-T28). lane5 was "
                f"mapped to it and is now {now}.{tail}")

    def test_it_takes_the_tool_and_moves_the_other_lane_past_the_bambu_lanes(
            self, tmp_path, monkeypatch):
        m = self._home_pool(tmp_path, monkeypatch, other_map=["T24"])
        afc = m.printer.afc
        other = afc.lanes["lane5"]
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert afc.tool_cmds == {"T24": "lane24", "T25": "lane25",
                                 "T26": "lane26", "T27": "lane27",
                                 "T29": "lane5"}
        assert _p6_lane(m, "lane24").map == ["T24"]
        # lane24-lane28 are the Bambu lanes: lane5 takes the first T# after.
        assert (other.map, other.current_map) == (["T29"], "T29")
        assert afc.gcode.ready_gcode_handlers["T29"] == afc.cmd_CHANGE_TOOL
        assert other.sent == [["T29"]]
        assert afc.save_vars.call_count == 5
        claim = _p6_claim_log(self.A24, "Bambu_AMS_1")
        assert _p6_log(m) == claim[:2] + [self._moved(
            "T24", "T29", " To give lane5 another tool outside T24-T28, use "
            "SET_MAP LANE=lane5 MAP=<T#>.")] + claim[2:]
        _p6_consistent(m)

    def test_the_saved_map_brings_the_new_tool_back_after_a_restart(
            self, tmp_path, monkeypatch):
        # A real lane, so the var file gets what AFCLane writes for it.
        def _boot(where: pathlib.Path, maps: List[str]
                  ) -> Tuple[afcBridgeBox, Any]:
            """
            :param maps: lane5's T#s as PREP restored them
            :return tuple: the master and lane5
            """
            where.mkdir()
            m = _p6_chain(where, monkeypatch, pool_ams=1)
            lane5 = make_afc_lane("lane5", "Box_1", 0, printer=m.printer)
            lane5.map, lane5.current_map = list(maps), maps[0]
            afc = m.printer.afc
            afc.lanes["lane5"] = lane5
            for cmd in maps:
                afc.tool_cmds[cmd] = "lane5"
                afc.gcode.register_command(cmd, afc.cmd_CHANGE_TOOL)
            _p6_quiet(m)
            return m, lane5
        m, lane5 = _boot(tmp_path / "boot1", ["T24"])
        saved: List[str] = []
        real = m.printer.afc.save_vars

        def _save() -> None:
            """Record lane5's map as the var file writes it."""
            saved.append(lane5.map_to_string())
            real()
        monkeypatch.setattr(m.printer.afc, "save_vars", _save)
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert saved[-1] == "T29"
        # Next boot: PREP puts the saved map back ahead of the config one,
        # and the claim finds T24 free.
        m2, lane5 = _boot(tmp_path / "boot2", ["T29"])
        assert m2._claim_pool_unit(self.A24, "boxed") is not None
        afc2 = m2.printer.afc
        assert (afc2.tool_cmds["T24"], afc2.tool_cmds["T29"]) == (
            "lane24", "lane5")
        assert lane5.map == ["T29"]
        assert _p6_log(m2) == _p6_claim_log(self.A24, "Bambu_AMS_1")

    def test_a_config_map_in_the_bambu_range_is_named(self, tmp_path,
                                                      monkeypatch):
        m = self._home_pool(tmp_path, monkeypatch, other_map=["T24"])
        other = m.printer.afc.lanes["lane5"]
        other._map = ["T24"]
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert other.map == ["T29"]
        claim = _p6_claim_log(self.A24, "Bambu_AMS_1")
        assert _p6_log(m) == claim[:2] + [self._moved(
            "T24", "T29", " To give lane5 another tool outside T24-T28, use "
            "SET_MAP LANE=lane5 MAP=<T#>. Also set map: in "
            "[AFC_stepper lane5] outside T24-T28, or AFC_RESET_MAPPING puts "
            "lane5 back on T24 and lane24 on another T#.")] + claim[2:]

    def test_the_new_tool_skips_taken_ones(self, tmp_path, monkeypatch):
        m = self._home_pool(tmp_path, monkeypatch, other_map=["T24"])
        afc = m.printer.afc
        other = afc.lanes["lane5"]

        def _macro(gcmd: Any) -> None:
            """A user macro on T29."""
        afc.gcode.register_command("T29", _macro)
        _p6_other(m, "lane6", ["T30"], config_map=["T31"])
        # A CHANGE_TOOL no lane uses is taken as it is, not registered again.
        unused = afc.cmd_CHANGE_TOOL
        afc.gcode.register_command("T32", unused)
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert other.map == ["T32"]
        assert afc.tool_cmds["T32"] == "lane5"
        assert afc.gcode.ready_gcode_handlers["T32"] is unused
        assert afc.gcode.ready_gcode_handlers["T29"] is _macro
        claim = _p6_claim_log(self.A24, "Bambu_AMS_1")
        assert _p6_log(m) == claim[:2] + [self._moved(
            "T24", "T32", " To give lane5 another tool outside T24-T28, use "
            "SET_MAP LANE=lane5 MAP=<T#>.")] + claim[2:]

    def test_the_other_lane_keeps_its_other_tools(self, tmp_path,
                                                  monkeypatch):
        m = self._home_pool(tmp_path, monkeypatch, other_map=["T3", "T24"])
        afc = m.printer.afc
        other = afc.lanes["lane5"]
        assert other.current_map == "T24"
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert (other.map, other.current_map) == (["T3"], "T3")
        assert (afc.tool_cmds["T3"], afc.tool_cmds["T24"]) == (
            "lane5", "lane24")
        assert "T29" not in afc.gcode.ready_gcode_handlers
        claim = _p6_claim_log(self.A24, "Bambu_AMS_1")
        assert _p6_log(m) == claim[:2] + [self._moved("T24", "T3")] + claim[
            2:]

    def test_a_claim_over_free_tools_is_quiet(self, tmp_path, monkeypatch):
        m = self._home_pool(tmp_path, monkeypatch)
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert m.printer.afc.tool_cmds["T24"] == "lane24"
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1")

    def test_a_replug_does_not_warn_again(self, tmp_path, monkeypatch):
        m = self._home_pool(tmp_path, monkeypatch, other_map=["T24"])
        afc = m.printer.afc
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        m._release_pool_unit(self.A24)
        _p6_quiet(m)
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert afc.tool_cmds["T24"] == "lane24"
        assert afc.lanes["lane5"].map == ["T29"]
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1")

    def test_a_lane_already_on_its_home_tool_is_left_alone(
            self, tmp_path, monkeypatch):
        # Claimed, then claimed again with no release in between: lane24 is
        # still live and still owns T24.
        m = self._home_pool(tmp_path, monkeypatch)
        unit = m._claim_pool_unit(self.A24, "boxed")
        _p6_bay(m, "Bambu_AMS_1")["bound"] = None
        unit.pool = True
        _p6_quiet(m)
        assert m._claim_pool_unit(self.A24, "boxed") is unit
        assert m.printer.afc.tool_cmds["T24"] == "lane24"
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1")

    def test_a_failed_claim_leaves_the_other_lane_its_tool(
            self, tmp_path, monkeypatch):
        # No bridge on the port: the unit's claim() fails.
        m = self._home_pool(tmp_path, monkeypatch, other_map=["T24"])
        del live_bridges()[m.serial_port]
        assert m._claim_pool_unit(self.A24, "boxed") is None
        afc = m.printer.afc
        assert afc.tool_cmds == {"T24": "lane5"}
        assert afc.lanes["lane5"].map == ["T24"]
        assert _p6_bay(m, "Bambu_AMS_1")["uid"] is None
        assert _p6_log(m) == [(
            "warning",
            "AFC bambu Bambu_AMS_1: claim found no live bridge on "
            "/dev/serial/by-id/usb-chain1-if00; unit stays offline until "
            "restart.")]

    def test_a_stale_map_does_not_pop_another_lanes_tool(
            self, tmp_path, monkeypatch):
        # lane24 comes back carrying T3 in its map; T3 is lane5's.
        m = self._home_pool(tmp_path, monkeypatch, other_map=["T3"])
        _p6_lane(m, "lane24").map = ["T3"]
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        afc = m.printer.afc
        assert afc.tool_cmds["T3"] == "lane5"
        assert afc.lanes["lane5"].map == ["T3"]
        assert afc.lanes["lane24"].map == ["T24"]
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1")

    def test_stale_table_entries_of_the_pooled_lanes_are_cleared_first(
            self, tmp_path, monkeypatch):
        # lane24 comes back carrying T9, and AFC's tool table still names
        # lane24 for T9 and lane25 for T8; neither lane is live.
        m = self._home_pool(tmp_path, monkeypatch)
        afc = m.printer.afc
        _p6_lane(m, "lane24").map = ["T9"]
        afc.tool_cmds.update({"T9": "lane24", "T8": "lane25"})
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert afc.tool_cmds == {"T24": "lane24", "T25": "lane25",
                                 "T26": "lane26", "T27": "lane27"}
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1")

    def test_a_save_that_fails_does_not_stop_the_claim(self, tmp_path,
                                                       monkeypatch):
        # T24-T27 are left registered to CHANGE_TOOL, so the claim's own
        # save is the only one, and it fails.
        m = self._home_pool(tmp_path, monkeypatch)
        afc = m.printer.afc
        for n in (24, 25, 26, 27):
            afc.gcode.register_command(f"T{n}", afc.cmd_CHANGE_TOOL)
        tried: List[str] = []

        def _save() -> None:
            """:raises OSError: the var file cannot be written"""
            tried.append("save")
            raise OSError("disk full")
        monkeypatch.setattr(afc, "save_vars", _save)
        assert m._claim_pool_unit(self.A24, "boxed") is not None
        assert tried == ["save"]
        assert _p6_bay(m, "Bambu_AMS_1")["bound"] == self.A24
        assert _p6_log(m) == _p6_claim_log(self.A24, "Bambu_AMS_1")

    # ── a unit with no free bay of its family ────────────────────────────

    #: The opening of the line for an AMS that finds four AMS bays held.
    ALL_FOUR = (
        "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays belong to "
        "other units (Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), Bambu_AMS_3 "
        "(CCCC), Bambu_AMS_4 (DDDD)), and a Bambu bus addresses at most 4 "
        "AMS, so neither pool_ams nor RESTART adds one.")

    def test_with_four_ams_bays_held_it_says_forget_not_restart(
            self, tmp_path, monkeypatch):
        # A-C on the wire; D unplugged and still claimed (auto_drop off).
        m = _p6_stuck(tmp_path, monkeypatch, online=(A, B, C, E))
        for pu in m._pool_units:
            if pu["uid"] in (A, B, C, D):
                pu["bound"] = pu["uid"]
        _p6_quiet(m)
        assert m._claim_pool_unit(E, "boxed") is None
        said = [(
            "warning",
            self.ALL_FOUR + " Bambu_AMS_4 (DDDD) is offline: if this AMS "
            "replaces it, AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=DDDD frees "
            "that bay and this AMS claims it live. AFC_BRIDGEBOX_REPLACE "
            "CHAIN=chain1 UID=EEEE OLD=Bambu_AMS_4 does both in one step.")]
        assert _p6_log(m) == said
        assert (m._no_bay, m._no_bay_told) == ({E: "boxed"}, {E})
        # The watch retries each tick; the line is said once per wait.
        assert m._claim_pool_unit(E, "boxed") is None
        assert _p6_log(m) == said
        assert _p6_bay(m, "Bambu_AMS_HT_2")["uid"] is None

    def test_with_several_offline_it_lists_them(self, tmp_path,
                                                monkeypatch):
        m = _p6_stuck(tmp_path, monkeypatch, online=(A, B, E))
        _p6_quiet(m)
        assert m._claim_pool_unit(E, "boxed") is None
        assert _p6_log(m) == [(
            "warning",
            self.ALL_FOUR + " Offline: Bambu_AMS_3 (CCCC), Bambu_AMS_4 "
            "(DDDD). If this AMS replaces one of them, AFC_BRIDGEBOX_FORGET "
            "CHAIN=chain1 UID=<that unit's uid> frees that bay and this AMS "
            "claims it live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE "
            "OLD=<that unit's bay> does both in one step.")]
        assert (m._no_bay, m._no_bay_told) == ({E: "boxed"}, {E})

    def test_with_roster_set_it_says_to_swap_the_entry_there(
            self, tmp_path, monkeypatch):
        # roster: is the whole roster: FORGET alone leaves D's bay built for
        # D at the next restart, and E unlisted.
        m = _p6_stuck(tmp_path, monkeypatch, roster=FOUR_p6)
        _p6_quiet(m)
        assert m._claim_pool_unit(E, "boxed") is None
        assert _p6_log(m) == [(
            "warning",
            self.ALL_FOUR + " Bambu_AMS_4 (DDDD) is offline: if this AMS "
            "replaces it, replace its entry in roster: with boxed:EEEE, run "
            "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=DDDD, and RESTART.")]
        assert (m._no_bay, m._no_bay_told) == ({E: "boxed"}, {E})
        # What the line says works: FORGET D, then the swapped roster: gives
        # E that bay.
        m.cmd_AFC_BRIDGEBOX_FORGET(FakeGcmd({"UID": D}))
        swapped = f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{E}, ht:{H}"
        m2 = _p6_chain(tmp_path, monkeypatch, roster=swapped, pool_ams=4,
                       pool_ht=2, ready=False)
        assert _p6_bay(m2, "Bambu_AMS_4")["uid"] == E
        assert _p6_unit(m2, "Bambu_AMS_4").unit_uid == E

    def test_with_fewer_ams_bays_a_restart_builds_one(self, tmp_path,
                                                      monkeypatch):
        m = _p6_chain(tmp_path, monkeypatch,
                      recorded=f"boxed:{A}, boxed:{B}, ht:{H}", pool_ams=2,
                      pool_ht=2, ready=False)
        assert _p6_bay(m, "Bambu_AMS_HT_1")["lanes"] == ["lane32"]
        _p6_quiet(m)
        assert m._claim_pool_unit(E, "boxed") is None
        assert _p6_log(m) == [(
            "warning",
            "AFC_BridgeBox chain1: new AMS EEEE has no free bay: every AMS "
            "bay built belongs to a known unit. Offline: Bambu_AMS_1 (AAAA), "
            "Bambu_AMS_2 (BBBB). If this AMS replaces one of them, "
            "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=<that unit's uid> frees "
            "that bay and this AMS claims it live. AFC_BRIDGEBOX_REPLACE "
            "CHAIN=chain1 UID=EEEE OLD=<that unit's bay> does both in one "
            "step. Once it is recorded, RESTART builds it one, past the AMS "
            "band, which moves the HT lanes up 4; raise pool_ams to keep "
            "spare AMS bays for units plugged in live (at most 4 AMS).")]
        assert (m._no_bay, m._no_bay_told) == ({E: "boxed"}, {E})
        # Once recorded, the restart builds E a bay and moves the HT up.
        record_chain_state(tmp_path,
                           roster=f"boxed:{A}, boxed:{B}, ht:{H}, boxed:{E}")
        m2 = _p6_chain(tmp_path, monkeypatch, roster="", pool_ams=2,
                       pool_ht=2, ready=False)
        assert _p6_bay(m2, "Bambu_AMS_3")["uid"] == E
        assert _p6_bay(m2, "Bambu_AMS_HT_1")["lanes"] == ["lane36"]
        _p6_quiet(m2)
        m2._scout_ready()
        assert _p6_log(m2) == [(
            "warning",
            "AFC_BridgeBox chain1: HT HHHH (Bambu_AMS_HT_1) keeps its name, "
            "and its lanes and T# changed: lane32 (T32) -> lane36 (T36).")]

    def test_a_bay_inside_the_ams_band_moves_nothing(self, tmp_path,
                                                     monkeypatch):
        # B holds the fourth bay, so the band spans four and E's bay at
        # restart is the second, inside it.
        m = _p6_chain(tmp_path, monkeypatch,
                      recorded=f"boxed:{A}, boxed:{B}, ht:{H}",
                      state={"name_map": f"{A}:Bambu_AMS_1, "
                                         f"{B}:Bambu_AMS_4"},
                      pool_ams=1, pool_ht=2, ready=False)
        assert _p6_bay(m, "Bambu_AMS_4")["lanes"] == ["lane36", "lane37",
                                                      "lane38", "lane39"]
        _p6_quiet(m)
        assert m._claim_pool_unit(E, "boxed") is None
        assert _p6_log(m) == [(
            "warning",
            "AFC_BridgeBox chain1: new AMS EEEE has no free bay: every AMS "
            "bay built belongs to a known unit. Offline: Bambu_AMS_1 (AAAA), "
            "Bambu_AMS_4 (BBBB). If this AMS replaces one of them, "
            "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=<that unit's uid> frees "
            "that bay and this AMS claims it live. AFC_BRIDGEBOX_REPLACE "
            "CHAIN=chain1 UID=EEEE OLD=<that unit's bay> does both in one "
            "step. Once it is recorded, RESTART builds it one; raise pool_ams "
            "to keep spare AMS bays for units plugged in live (at most 4 "
            "AMS).")]
        assert (m._no_bay, m._no_bay_told) == ({E: "boxed"}, {E})

    def test_an_ht_is_told_a_restart_builds_its_bay(self, tmp_path,
                                                     monkeypatch):
        m = _p6_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}, ht:{H}",
                      pool_ams=1, pool_ht=0, ready=False)
        _p6_quiet(m)
        assert m._claim_pool_unit(G, "ht") is None
        assert _p6_log(m) == [(
            "warning",
            "AFC_BridgeBox chain1: new HT GGGG has no free bay: every HT bay "
            "built belongs to a known unit. Bambu_AMS_HT_1 (HHHH) is "
            "offline: if this HT replaces it, AFC_BRIDGEBOX_FORGET "
            "CHAIN=chain1 UID=HHHH frees that bay and this HT claims it live. "
            "AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=GGGG OLD=Bambu_AMS_HT_1 "
            "does both in one step. Once it is recorded, RESTART builds it "
            "one; raise pool_ht to keep spare HT bays for units plugged in "
            "live.")]
        assert (m._no_bay, m._no_bay_told) == ({G: "ht"}, {G})
        record_chain_state(tmp_path, roster=f"boxed:{A}, ht:{H}, ht:{G}")
        m2 = _p6_chain(tmp_path, monkeypatch, roster="", pool_ams=1,
                       pool_ht=0, ready=False)
        assert _p6_bay(m2, "Bambu_AMS_HT_2")["uid"] == G

    # ── the records held for the bay's owner ─────────────────────────────

    #: What AFC.var.unit saved for Alpha when AAAA was last claimed onto it.
    BOOT_VAR = {"Alpha": {"lane24": _p6_rec("T24", spool_id=159,
                                            material="PLA"),
                          "lane25": {}}}

    @staticmethod
    def _handed(monkeypatch: pytest.MonkeyPatch,
                unit: Any) -> List[Optional[Dict[str, Any]]]:
        """
        Record the records the unit holds each time claim() is called.

        :param unit: the pool unit
        :return list: its held records as each claim() found them
        """
        seen: List[Optional[Dict[str, Any]]] = []
        real = unit.claim

        def _claim(uid: str, model: str) -> bool:
            """Record what the master handed over, then claim."""
            held = unit._held_lanes
            seen.append(None if held is None else dict(held))
            return real(uid, model)
        monkeypatch.setattr(unit, "claim", _claim)
        return seen

    @staticmethod
    def _restart(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                 **options: Any) -> afcBridgeBox:
        """
        The next start of the named chain on the state an earlier start
        left, through klippy:ready.

        :param options: further :func:`_p6_chain` arguments
        :return afcBridgeBox: the master
        """
        return _p6_named(tmp_path, monkeypatch, recorded=None, roster="",
                         **options)

    def test_the_owner_claiming_after_prep_gets_them_before_it_claims(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, var=self.BOOT_VAR,
                      owners=f"{A}:Alpha")
        write_unit_vars(m.printer, {"Alpha": {}})       # PREP's first save
        unit = _p6_unit(m, "Alpha")
        handed = self._handed(monkeypatch, unit)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is unit
        assert handed == [{"lane24": self.BOOT_VAR["Alpha"]["lane24"]}]
        lane = _p6_lane(m, "lane24")
        assert (lane.spool_id, lane.material, lane.map) == (159, "PLA",
                                                           ["T24"])
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")

    def test_another_unit_on_that_bay_gets_nothing(self, tmp_path,
                                                   monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, var=self.BOOT_VAR,
                      owners=f"{A}:Alpha")
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(FakeGcmd({"UID": A}))
        m.cmd_AFC_BRIDGEBOX_ASSIGN(FakeGcmd({"UID": C, "NAME": "Alpha"}))
        unit = _p6_unit(m, "Alpha")
        handed = self._handed(monkeypatch, unit)
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is unit
        assert handed == [{}]
        lane = _p6_lane(m, "lane24")
        assert (lane.spool_id, lane.material, lane.map) == (None, None,
                                                           ["T24"])
        # AAAA's records stay held for it.
        assert m._held["Alpha"]["uid"] == A
        assert _p6_log(m) == _p6_claim_log(C, "Alpha")

    def test_the_ht_keeps_its_lane_record_and_map(self, tmp_path,
                                                  monkeypatch):
        # pool_ams lowered below the AMS band a recorded HT sits past: the
        # HT keeps its lanes, so what its lane saved, spool and T# map, comes
        # back with it at each boot.
        names = {"ams_names": "Alpha, Bravo, Charlie, Delta",
                 "ht_names": "Hot"}
        first = _p6_chain(tmp_path, monkeypatch,
                          recorded=f"boxed:{A}, boxed:{B}, ht:{H}",
                          pool_ams=4, pool_ht=1, ready=False, **names)
        assert _p6_bay(first, "Hot")["lanes"] == ["lane40"]
        rec = _p6_rec("T7", spool_id=5, material="PETG")
        for _again in range(2):
            m = _p6_chain(tmp_path, monkeypatch, roster="", pool_ams=2,
                          pool_ht=1, var={"Hot": {"lane40": rec}},
                          owners=f"{H}:Hot", **names)
            assert _p6_bay(m, "Hot")["lanes"] == ["lane40"]
            assert m._held["Hot"] == {"uid": H, "lanes": {"lane40": rec}}
            unit = _p6_unit(m, "Hot")
            handed = self._handed(monkeypatch, unit)
            _p6_quiet(m)
            assert m._claim_pool_unit(H, "ht") is unit
            lane = _p6_lane(m, "lane40")
            assert (lane.map, lane.current_map) == (["T7"], "T7")
            assert (lane.spool_id, lane.material) == (5, "PETG")
            assert handed == [{"lane40": rec}]
            assert _p6_log(m) == _p6_claim_log(
                H, "Hot", "ht", 1, " Saved maps: lane40->T7.")

    # ── bay_owner: the unit each bay was last claimed onto ───────────────

    @staticmethod
    def _owner_at_saves(monkeypatch: pytest.MonkeyPatch,
                        master: afcBridgeBox) -> List[Optional[str]]:
        """
        Record the bay_owner the state file holds at each AFC save.

        :param master: the chain master
        :return list: bay_owner as each save found it
        """
        seen: List[Optional[str]] = []
        afc = master.printer.afc
        real = afc.save_vars

        def _save() -> None:
            """Record bay_owner, then save."""
            seen.append(master._state_get(SEC, "bay_owner"))
            real()
        monkeypatch.setattr(afc, "save_vars", _save)
        return seen

    @staticmethod
    def _state_writes(monkeypatch: pytest.MonkeyPatch, master: afcBridgeBox,
                      fail: bool = False) -> List[str]:
        """
        Watch the writes of the chain's state file, as the disk sees them.

        :param master: the chain master
        :param fail: refuse each write, as a full or read-only disk does
        :return list: the state file's path once per write tried
        """
        tried: List[str] = []

        def _open(path: Any, mode: str = "r", *args: Any,
                  **kwargs: Any) -> Any:
            """Open as the builtin does; a state file write is counted."""
            if str(path) == master.state_file and "w" in mode:
                tried.append(str(path))
                if fail:
                    error_str = f"read-only file system: {path}"
                    raise OSError(error_str)
            return builtins.open(path, mode, *args, **kwargs)
        monkeypatch.setattr(bb_module, "open", _open, raising=False)
        return tried

    def test_the_claim_records_the_owner_before_any_tool_is_assigned(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        assert m._state_get(SEC, "bay_owner") is None
        seen = self._owner_at_saves(monkeypatch, m)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        # Four TcmdAssign saves and the claim's own: each one after it.
        assert seen == ["AAAA:Alpha"] * 5
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")

    def test_an_unchanged_owner_is_not_written_again(self, tmp_path,
                                                     monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        m._release_pool_unit(A)
        writes = self._state_writes(monkeypatch, m)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert writes == []
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")

    def test_a_failed_claim_records_nothing_and_takes_its_records_back(
            self, tmp_path, monkeypatch):
        # No bridge on the port: the unit's claim() fails.
        m = _p6_named(tmp_path, monkeypatch)
        m._held = {"Alpha": {"uid": A, "lanes": {"lane24": _p6_rec()}}}
        unit = _p6_unit(m, "Alpha")
        handed = self._handed(monkeypatch, unit)
        del live_bridges()[m.serial_port]
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is None
        assert handed == [{"lane24": _p6_rec()}]
        assert unit._held_lanes == {}
        assert m._state_get(SEC, "bay_owner") is None
        assert (m._owners(), m._claim_plans) == ({}, {})
        # The lanes go back to the pool, and the bay keeps AAAA's uid.
        assert (m.printer.afc.lanes, m.printer.afc.tool_cmds) == ({}, {})
        assert _p6_lane(m, "lane24").unassigned is True
        assert (_p6_bay(m, "Alpha")["uid"], _p6_bay(m, "Alpha")["bound"]) == (
            A, None)
        assert _p6_log(m) == [(
            "warning",
            "AFC bambu Alpha: claim found no live bridge on "
            "/dev/serial/by-id/usb-chain1-if00; unit stays offline until "
            "restart.")]

    def test_a_unit_is_the_owner_of_one_bay(self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(C, "boxed") is _p6_unit(m, "Bravo")
        assert m._state_get(SEC, "bay_owner") == "CCCC:Bravo"
        m._release_pool_unit(C)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(FakeGcmd({"UID": C, "NAME": "Charlie"}))
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is _p6_unit(m, "Charlie")
        assert m._state_get(SEC, "bay_owner") == "CCCC:Charlie"
        assert m._owners() == {"Charlie": C}
        assert _p6_log(m) == _p6_claim_log(C, "Charlie")

    def test_a_unit_moved_back_gets_nothing_from_before_the_move(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        _p6_lane(m, "lane24").spool_id = 159
        m.cmd_AFC_BRIDGEBOX_ASSIGN(FakeGcmd({"UID": A, "NAME": "Charlie"}))
        assert m._held["Alpha"]["lanes"]["lane24"]["spool_id"] == 159
        assert m._claim_pool_unit(A, "boxed") is _p6_unit(m, "Charlie")
        # Claimed onto Charlie: what was held for it on Alpha goes.
        assert m._held == {}
        _p6_lane(m, "lane32").spool_id = 777
        m.cmd_AFC_BRIDGEBOX_ASSIGN(FakeGcmd({"UID": A, "NAME": "Alpha"}))
        assert list(m._held) == ["Charlie"]
        alpha = _p6_unit(m, "Alpha")
        handed = self._handed(monkeypatch, alpha)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is alpha
        assert handed == [{}]
        assert _p6_lane(m, "lane24").spool_id is None
        assert m._held == {}
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")

    def test_the_claim_saves_the_lanes_when_no_tool_needed_assigning(
            self, tmp_path, monkeypatch):
        # T24-T27 are left registered to CHANGE_TOOL, so no TcmdAssign runs
        # and saves: the claim's own save writes the lanes.
        m = _p6_named(tmp_path, monkeypatch)
        afc = m.printer.afc
        for n in (24, 25, 26, 27):
            afc.gcode.register_command(f"T{n}", afc.cmd_CHANGE_TOOL)
        seen = self._owner_at_saves(monkeypatch, m)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert afc.tool_cmds == {"T24": "lane24", "T25": "lane25",
                                 "T26": "lane26", "T27": "lane27"}
        assert seen == ["AAAA:Alpha"]
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")

    def test_an_owner_the_state_file_missed_is_written_at_the_next_claim(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        writes = self._state_writes(monkeypatch, m, fail=True)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert writes == [m.state_file]
        assert m._state_get(SEC, "bay_owner") is None
        m._release_pool_unit(A)
        writes = self._state_writes(monkeypatch, m)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert writes == [m.state_file]
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")

    def test_a_claim_before_prep_leaves_the_saved_owner_alone(
            self, tmp_path, monkeypatch):
        # Past the wait for PREP, AFC.var.unit still holds what was saved
        # while EEEE was claimed onto Bravo: save_vars writes nothing until
        # PREP has run, so the state keeps naming EEEE.
        bravo = {"Bravo": {"lane28": _p6_rec("T28", spool_id=159,
                                             material="PLA")}}
        m = _p6_named(tmp_path, monkeypatch, var=bravo, owners=f"{E}:Bravo")
        m.printer.afc.prep_done = False
        m.printer.reactor.now = m._ready_at + 91.0
        unit = _p6_unit(m, "Bravo")
        handed = self._handed(monkeypatch, unit)
        _p6_quiet(m)
        assert m._claim_pool_unit(D, "boxed") is unit
        assert handed == [{}]
        assert m._owners() == {"Bravo": D}
        assert m._bay_owner_pending is True
        assert m._state_get(SEC, "bay_owner") == "EEEE:Bravo"
        assert _p6_log(m) == [
            ("warning",
             "AFC_BridgeBox chain1: PREP has not finished 90s after startup; "
             "claiming units anyway.")] + _p6_claim_log(D, "Bravo")
        # A restart before PREP runs holds the records for EEEE again.
        again = self._restart(tmp_path, monkeypatch)
        assert again._held == {"Bravo": {"uid": E, "lanes": bravo["Bravo"]}}
        unit = _p6_unit(again, "Bravo")
        handed = self._handed(monkeypatch, unit)
        assert again._claim_pool_unit(D, "boxed") is unit
        assert handed == [{}]
        assert _p6_lane(again, "lane28").map == ["T28"]

    # ── records held at a release, and the bay a spare goes back to ──────

    def test_the_same_unit_gets_them_back_and_another_does_not(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        alpha = _p6_unit(m, "Alpha")
        assert m._claim_pool_unit(A, "boxed") is alpha
        lane = _p6_lane(m, "lane24")
        lane.spool_id, lane.material, lane.color = 159, "PLA", "#0086D6"
        m._release_pool_unit(A)
        handed = self._handed(monkeypatch, alpha)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is alpha
        assert handed[-1]["lane24"]["spool_id"] == 159
        assert (lane.spool_id, lane.material, lane.color) == (159, "PLA",
                                                             "#0086D6")
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")
        m._release_pool_unit(A)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(FakeGcmd({"UID": A}))
        m.cmd_AFC_BRIDGEBOX_ASSIGN(FakeGcmd({"UID": C, "NAME": "Alpha"}))
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is alpha
        assert handed[-1] == {}
        assert (lane.spool_id, lane.material, lane.color) == (None, None, "")
        assert _p6_log(m) == _p6_claim_log(C, "Alpha")

    def test_spares_come_back_to_their_own_bays_with_their_records(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        bravo, charlie = _p6_unit(m, "Bravo"), _p6_unit(m, "Charlie")
        assert m._claim_pool_unit(C, "boxed") is bravo
        assert m._claim_pool_unit(D, "boxed") is charlie
        _p6_lane(m, "lane28").spool_id = 11
        _p6_lane(m, "lane32").spool_id = 22
        m._release_pool_unit(C)
        m._release_pool_unit(D)
        assert (_p6_bay(m, "Bravo")["uid"], _p6_bay(m, "Charlie")["uid"]) == (
            None, None)
        _p6_quiet(m)
        # Bravo is lower and free, but DDDD was last on Charlie.
        assert m._claim_pool_unit(D, "boxed") is charlie
        assert m._claim_pool_unit(C, "boxed") is bravo
        assert (_p6_bay(m, "Bravo")["bound"],
                _p6_bay(m, "Charlie")["bound"]) == (C, D)
        assert (_p6_lane(m, "lane28").spool_id,
                _p6_lane(m, "lane32").spool_id) == (11, 22)
        assert _p6_log(m) == (_p6_claim_log(D, "Charlie")
                              + _p6_claim_log(C, "Bravo"))

    def test_a_floating_unit_goes_back_to_its_last_bay(self, tmp_path,
                                                       monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, owners=f"{D}:Charlie")
        _p6_quiet(m)
        assert m._claim_pool_unit(D, "boxed") is _p6_unit(m, "Charlie")
        assert (_p6_bay(m, "Bravo")["uid"], _p6_bay(m, "Charlie")["uid"]) == (
            None, D)
        assert _p6_log(m) == _p6_claim_log(D, "Charlie")

    def test_a_reserved_or_other_family_bay_is_not_taken_back(
            self, tmp_path, monkeypatch):
        # Alpha is AAAA's; Hot is an HT bay.
        m = _p6_named(tmp_path, monkeypatch, owners=f"{D}:Alpha, {E}:Hot")
        _p6_quiet(m)
        assert m._claim_pool_unit(D, "boxed") is _p6_unit(m, "Bravo")
        assert m._claim_pool_unit(E, "boxed") is _p6_unit(m, "Charlie")
        assert (_p6_bay(m, "Alpha")["uid"], _p6_bay(m, "Hot")["uid"]) == (
            A, None)
        assert _p6_log(m) == (_p6_claim_log(D, "Bravo")
                              + _p6_claim_log(E, "Charlie"))

    # ── the T# maps the claimed lanes come back with ─────────────────────

    #: Two AMS bays, Alpha (lane24-lane27) and Bravo (lane28-lane31), then
    #: the HT bay Hot on lane32.
    TWO = {"recorded": f"boxed:{A}, boxed:{B}", "pool_ams": 2,
           "ams_names": "Alpha, Bravo"}

    @staticmethod
    def _swapped(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch
                 ) -> Tuple[afcBridgeBox, _P6OtherLane]:
        """
        SET_MAP left lane24 on T3 and gave its T24 to lane5.

        :return tuple: the master and lane5
        """
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T3")}},
                      owners=f"{A}:Alpha")
        lane5 = _p6_other(m, "lane5", ["T24"])
        _p6_quiet(m)
        return m, lane5

    @staticmethod
    def _maps(master: afcBridgeBox, *names: str) -> Dict[str, List[str]]:
        """:return dict: lane -> its map, for the lanes named"""
        return {name: list(_p6_lane(master, name).map) for name in names}

    def test_a_saved_map_comes_back_and_the_home_tool_stays_put(
            self, tmp_path, monkeypatch):
        m, lane5 = self._swapped(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        lane24 = _p6_lane(m, "lane24")
        afc = m.printer.afc
        assert (lane24.map, lane24.current_map) == (["T3"], "T3")
        assert afc.tool_cmds == {"T3": "lane24", "T24": "lane5",
                                 "T25": "lane25", "T26": "lane26",
                                 "T27": "lane27"}
        assert (lane5.map, lane5.sent) == (["T24"], [])
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane24->T3.")
        _p6_consistent(m)

    def test_only_a_home_tool_is_taken_and_only_after_the_claim(
            self, tmp_path, monkeypatch):
        m, lane5 = self._swapped(tmp_path, monkeypatch)
        unit = _p6_unit(m, "Alpha")
        seen: List[Tuple[str, List[str], bool]] = []
        real = m._take_home_tool

        def _spy(afc: Any, lane: Any) -> None:
            """Record the lane, its map and the unit's pool flag."""
            seen.append((lane.name, list(lane.map), unit.pool))
            real(afc, lane)
        monkeypatch.setattr(m, "_take_home_tool", _spy)
        bridge = live_bridges().pop(m.serial_port)
        assert m._claim_pool_unit(A, "boxed") is None
        assert seen == []
        assert (lane5.map, m.printer.afc.tool_cmds) == (["T24"],
                                                        {"T24": "lane5"})
        assert _p6_log(m) == [(
            "warning",
            "AFC bambu Alpha: claim found no live bridge on "
            "/dev/serial/by-id/usb-chain1-if00; unit stays offline until "
            "restart.")]
        live_bridges()[m.serial_port] = bridge
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is unit
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane24->T3.")
        # lane24 keeps its saved T3, so it takes no home tool.
        assert seen == [("lane25", ["T25"], False), ("lane26", ["T26"], False),
                        ("lane27", ["T27"], False)]
        _p6_consistent(m)

    def test_a_swap_inside_a_unit_comes_back(self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T25"),
                                     "lane25": _p6_rec("T24")}},
                      owners=f"{A}:Alpha")
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert self._maps(m, "lane24", "lane25") == {"lane24": ["T25"],
                                                     "lane25": ["T24"]}
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane24->T25, lane25->T24.")
        _p6_consistent(m)

    @pytest.mark.parametrize("order", [(A, B), (B, A)])
    def test_a_swap_across_bays_comes_back_in_either_order(
            self, tmp_path, monkeypatch, order):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T28")},
                           "Bravo": {"lane28": _p6_rec("T24")}},
                      owners=f"{A}:Alpha, {B}:Bravo", **self.TWO)
        _p6_quiet(m)
        for uid in order:
            assert m._claim_pool_unit(uid, "boxed") is not None
        assert self._maps(m, "lane24", "lane28") == {"lane24": ["T28"],
                                                     "lane28": ["T24"]}
        bays = {A: ("Alpha", "lane24->T28"), B: ("Bravo", "lane28->T24")}
        said: List[LogLine] = []
        for uid in order:
            bay, saved = bays[uid]
            said += _p6_claim_log(uid, bay, tail=f" Saved maps: {saved}.")
        assert _p6_log(m) == said
        _p6_consistent(m)

    def test_a_saved_tool_a_claimed_lane_took_stays_with_it(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T28")}},
                      owners=f"{A}:Alpha", **self.TWO)
        assert m._claim_pool_unit(B, "boxed") is not None   # lane28 takes T28
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert _p6_lane(m, "lane24").map == ["T24"]
        # Both lanes are on their own home tools: AFC.log only.
        claim = _p6_claim_log(A, "Alpha")
        assert _p6_log(m) == claim[:2] + [(
            "debug",
            "AFC_BridgeBox chain1: lane24: saved T28 is held by lane28 -- "
            "not restored; lane24 is back on T24")] + claim[2:]
        _p6_consistent(m)

    def test_a_saved_tool_a_bambu_lane_holds_off_its_home_still_warns(
            self, tmp_path, monkeypatch):
        # lane28 holds T3 (not its home) by SET_MAP: the user has something
        # to look at.
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T3")},
                           "Bravo": {"lane28": _p6_rec("T3")}},
                      owners=f"{A}:Alpha, {B}:Bravo", **self.TWO)
        assert m._claim_pool_unit(B, "boxed") is not None
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert _p6_lane(m, "lane24").map == ["T24"]
        claim = _p6_claim_log(A, "Alpha")
        assert _p6_log(m) == claim[:2] + [(
            "warning",
            "AFC_BridgeBox chain1: lane24: saved T3 is held by lane28 -- not "
            "restored; lane24 is back on T24")] + claim[2:]
        _p6_consistent(m)

    @staticmethod
    def _pushes(monkeypatch: pytest.MonkeyPatch,
                lane: Any) -> List[List[str]]:
        """
        Record the map each send_lane_data of a real lane pushes, then run it.

        :param lane: the lane
        :return list: the maps pushed, filled as they are sent
        """
        sent: List[List[str]] = []
        real = lane.send_lane_data

        def _push() -> None:
            """Record the map, then send it."""
            sent.append(list(lane.map))
            real()
        monkeypatch.setattr(lane, "send_lane_data", _push)
        return sent

    @pytest.mark.parametrize("order", [(A, C), (C, A)])
    def test_a_lane_on_a_sibling_home_tool_goes_back_to_its_own(
            self, tmp_path, monkeypatch, order):
        # lane24 was saved on T28 while Bravo was unclaimed; a unit with no
        # record for Bravo claims it and takes its home T28.
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T28")}},
                      owners=f"{A}:Alpha")
        lane24 = _p6_lane(m, "lane24")
        sent = self._pushes(monkeypatch, lane24)
        _p6_quiet(m)
        for uid in order:
            assert m._claim_pool_unit(uid, "boxed") is not None
        assert _p6_bay(m, "Bravo")["bound"] == C
        assert (lane24.map, lane24.current_map) == (["T24"], "T24")
        assert _p6_lane(m, "lane28").map == ["T28"]
        _p6_consistent(m)
        # The home wins and both lanes end on their own home tools, so the
        # move is in AFC.log only.
        if order[0] == A:
            bravo = _p6_claim_log(C, "Bravo")
            assert _p6_log(m) == (
                _p6_claim_log(A, "Alpha", tail=" Saved maps: lane24->T28.")
                + bravo[:2]
                + [("debug",
                    "AFC_BridgeBox chain1: T28 is the tool of Bambu lane "
                    "lane28. lane24 was mapped to it and is back on T24.")]
                + bravo[2:])
            assert sent == [["T28"], ["T24"]]
        else:
            alpha = _p6_claim_log(A, "Alpha")
            assert _p6_log(m) == (
                _p6_claim_log(C, "Bravo") + alpha[:2]
                + [("debug",
                    "AFC_BridgeBox chain1: lane24: saved T28 is held by "
                    "lane28 -- not restored; lane24 is back on T24")]
                + alpha[2:])
            assert sent == [["T24"]]
        m._release_pool_unit(A)
        assert m._held["Alpha"]["lanes"]["lane24"]["map"] == "T24"

    def test_a_bay_mate_on_a_lanes_home_tool_goes_back_to_its_own(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T25"),
                                     "lane25": {"spool_id": 5}}},
                      owners=f"{A}:Alpha")
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert self._maps(m, "lane24", "lane25") == {"lane24": ["T24"],
                                                     "lane25": ["T25"]}
        claim = _p6_claim_log(A, "Alpha")
        assert _p6_log(m) == claim[:2] + [(
            "debug",
            "AFC_BridgeBox chain1: T25 is the tool of Bambu lane lane25. "
            "lane24 was mapped to it and is back on T24.")] + claim[2:]
        _p6_consistent(m)

    def test_a_lane_whose_own_home_tool_is_held_is_still_moved_off(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T28")}},
                      owners=f"{A}:Alpha")
        lane5 = _p6_other(m, "lane5", ["T24"])
        assert m._claim_pool_unit(A, "boxed") is not None
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is not None
        assert (_p6_lane(m, "lane24").map, lane5.map) == (["T37"], ["T24"])
        assert _p6_lane(m, "lane28").map == ["T28"]
        _p6_consistent(m)
        # Left on a spare, not its home tool: the user has something to do.
        bravo = _p6_claim_log(C, "Bravo")
        assert _p6_log(m) == bravo[:2] + [(
            "warning",
            "AFC_BridgeBox chain1: T28 is the tool of Bambu lane lane28 "
            "(Bambu lanes lane24-lane36 are T24-T36). lane24 was mapped to it "
            "and is now T37. To give lane24 another tool outside T24-T36, use "
            "SET_MAP LANE=lane24 MAP=<T#>.")] + bravo[2:]

    @pytest.mark.parametrize("order", [(A, C), (C, A)])
    def test_a_multi_map_on_a_sibling_home_tool_keeps_its_own_quietly(
            self, tmp_path, monkeypatch, order):
        # SET_MAP gave lane24 T28 as well while Bravo was unclaimed. The
        # home wins, and lane24 keeps its own T24: AFC.log only.
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T24, T28")}},
                      owners=f"{A}:Alpha")
        _p6_quiet(m)
        for uid in order:
            assert m._claim_pool_unit(uid, "boxed") is not None
        assert self._maps(m, "lane24", "lane28") == {"lane24": ["T24"],
                                                     "lane28": ["T28"]}
        if order[0] == A:
            bravo = _p6_claim_log(C, "Bravo")
            assert _p6_log(m) == (
                _p6_claim_log(A, "Alpha", tail=" Saved maps: lane24->T24+T28.")
                + bravo[:2]
                + [("debug",
                    "AFC_BridgeBox chain1: T28 is the tool of Bambu lane "
                    "lane28 (Bambu lanes lane24-lane36 are T24-T36). lane24 "
                    "was mapped to it and is now T24.")]
                + bravo[2:])
        else:
            alpha = _p6_claim_log(A, "Alpha")
            assert _p6_log(m) == (
                _p6_claim_log(C, "Bravo") + alpha[:2]
                + [("debug",
                    "AFC_BridgeBox chain1: lane24: saved T28 is held by "
                    "lane28 -- not restored")]
                + alpha[2:])
        _p6_consistent(m)

    def test_a_multi_map_on_a_config_mapped_tool_still_warns(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T24, T28")}},
                      owners=f"{A}:Alpha")
        assert m._claim_pool_unit(A, "boxed") is not None
        _p6_lane(m, "lane24")._map = ["T28"]          # map: in its config
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is not None
        assert _p6_lane(m, "lane24").map == ["T24"]
        bravo = _p6_claim_log(C, "Bravo")
        assert _p6_log(m) == bravo[:2] + [(
            "warning",
            "AFC_BridgeBox chain1: T28 is the tool of Bambu lane lane28 "
            "(Bambu lanes lane24-lane36 are T24-T36). lane24 was mapped to it "
            "and is now T24. Also set map: in [AFC_lane lane24] outside "
            "T24-T36, or AFC_RESET_MAPPING puts lane24 back on T28 and lane28 "
            "on another T#.")] + bravo[2:]

    def test_the_claim_line_lists_saved_maps_one_lane_each(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T3, T40"),
                                     "lane25": _p6_rec("T5")}},
                      owners=f"{A}:Alpha")
        _p6_other(m, "lane5", ["T5"])
        _p6_other(m, "lane6", ["T25"])
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert self._maps(m, "lane24", "lane25") == {"lane24": ["T3", "T40"],
                                                     "lane25": ["NONE"]}
        claim = _p6_claim_log(A, "Alpha",
                              tail=" Saved maps: lane24->T3+T40.")
        assert _p6_log(m) == claim[:2] + [
            ("warning",
             "AFC_BridgeBox chain1: lane25: saved T5 is held by lane5 -- not "
             "restored"),
            ("warning",
             "AFC_BridgeBox chain1: lane25 has no T# -- its home T25 is held "
             "by lane6; use SET_MAP")] + claim[2:]

    def test_a_map_set_while_claimed_survives_a_replug(self, tmp_path,
                                                       monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        afc = m.printer.afc
        lane = _p6_lane(m, "lane25")
        afc.tool_cmds.pop("T25")
        afc.gcode.register_command("T25", None)
        lane.map, lane.current_map = ["T7"], "T7"
        afc.tool_cmds["T7"] = "lane25"
        afc.gcode.register_command("T7", afc.cmd_CHANGE_TOOL)
        m._release_pool_unit(A)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert (lane.map, lane.current_map) == (["T7"], "T7")
        assert "T25" not in afc.tool_cmds
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane25->T7.")
        _p6_consistent(m)

    def test_a_removed_last_tool_stays_removed_across_a_replug(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        assert m._claim_pool_unit(A, "boxed") is not None
        afc = m.printer.afc
        lane = _p6_lane(m, "lane25")
        # AFC_REMOVE_MAPPING MAPPING=T25, as AFC_spool does it.
        afc.tool_cmds.pop("T25")
        lane.map = [cmd for cmd in lane.map if cmd != "T25"]
        lane.current_map = ""
        afc.gcode.register_command("T25", None)
        m._release_pool_unit(A)
        assert m._held["Alpha"]["lanes"]["lane25"]["map"] == "NONE"
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert (lane.map, lane.current_map) == (["NONE"], "")
        assert "T25" not in afc.tool_cmds
        assert "T25" not in afc.gcode.ready_gcode_handlers
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane25->NONE.")
        _p6_consistent(m)

    def test_a_removed_last_tool_stays_removed_across_a_restart(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane25": _p6_rec("NONE", "")}},
                      owners=f"{A}:Alpha")
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert _p6_lane(m, "lane25").map == ["NONE"]
        assert "T25" not in m.printer.afc.gcode.ready_gcode_handlers
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane25->NONE.")
        _p6_consistent(m)

    def test_a_tool_moved_to_a_bay_mate_stays_there(self, tmp_path,
                                                    monkeypatch):
        # Multiple mapping: SET_MAP LANE=lane27 MAP=T25 left lane25 on NONE.
        m = _p6_named(tmp_path, monkeypatch, var={"Alpha": {
            "lane25": _p6_rec("NONE", ""),
            "lane27": _p6_rec("T25, T27", "T27")}}, owners=f"{A}:Alpha")
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert self._maps(m, "lane25", "lane27") == {
            "lane25": ["NONE"], "lane27": ["T25", "T27"]}
        assert m.printer.afc.tool_cmds["T25"] == "lane27"
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane25->NONE, lane27->T25+T27.")
        _p6_consistent(m)

    @pytest.mark.parametrize("order", [(A, B), (B, A)])
    def test_a_tool_moved_to_another_bay_stays_there_in_either_order(
            self, tmp_path, monkeypatch, order):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane25": _p6_rec("NONE", "")},
                           "Bravo": {"lane28": _p6_rec("T25, T28", "T28")}},
                      owners=f"{A}:Alpha, {B}:Bravo", **self.TWO)
        _p6_quiet(m)
        for uid in order:
            assert m._claim_pool_unit(uid, "boxed") is not None
        assert self._maps(m, "lane25", "lane28") == {
            "lane25": ["NONE"], "lane28": ["T25", "T28"]}
        bays = {A: ("Alpha", "lane25->NONE"), B: ("Bravo", "lane28->T25+T28")}
        said: List[LogLine] = []
        for uid in order:
            bay, saved = bays[uid]
            said += _p6_claim_log(uid, bay, tail=f" Saved maps: {saved}.")
        assert _p6_log(m) == said
        _p6_consistent(m)

    def test_a_claimed_lane_is_marked_prep_done(self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        lane = _p6_lane(m, "lane24")
        assert lane._afc_prep_done is False
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert lane._afc_prep_done is True
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")

    # ── a claim during a print ───────────────────────────────────────────

    #: lane24 saved in the extruder, with its spool details.
    LOADED = {"Alpha": {"lane24": _p6_rec("T24", material="PLA",
                                          color="#FF0000", weight=412.0,
                                          tool_loaded=True)}}

    def test_the_claim_leaves_a_live_tool_until_the_print_ends(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, var=self.LOADED,
                      owners=f"{A}:Alpha", print_state="printing")
        afc = m.printer.afc
        afc.tools["extruder"].lane_loaded = "lane24"
        lane5 = _p6_other(m, "lane5", ["T24"])
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is _p6_unit(m, "Alpha")
        assert _p6_bay(m, "Alpha")["bound"] == A
        lane24 = _p6_lane(m, "lane24")
        # The records are on the lanes and saved, the toolhead lane is
        # restored, and the live T# stays with lane5.
        assert (lane24.material, lane24.color, lane24.weight) == (
            "PLA", "#FF0000", 412.0)
        assert lane24.tool_loaded is True
        assert afc.save_vars.call_count == 5
        assert m._owners() == {"Alpha": A}
        assert (lane24.map, lane5.map) == (["NONE"], ["T24"])
        assert afc.tool_cmds == {"T24": "lane5", "T25": "lane25",
                                 "T26": "lane26", "T27": "lane27"}
        claim = _p6_claim_log(A, "Alpha")
        assert _p6_log(m) == claim[:2] + [(
            "info",
            "AFC_BridgeBox chain1: lane24 takes T24 from lane5 once the print "
            "ends, as the print may be using it; until then lane24 has no "
            "T#.")] + claim[2:]
        assert m._deferred_takes == {"lane24": {
            "home": "T24", "maps": ["T24"], "current": "T24",
            "quiet": False}}
        _p6_consistent(m)
        _p6_quiet(m)
        m._take_deferred_tools()                   # still printing
        assert (lane24.map, lane5.map) == (["NONE"], ["T24"])
        assert _p6_log(m) == []
        m.printer.set_print_state("complete")
        saves = afc.save_vars.call_count
        m._take_deferred_tools()
        assert (lane24.map, lane24.current_map) == (["T24"], "T24")
        assert (lane5.map, lane5.current_map) == (["T37"], "T37")
        assert _p6_log(m) == [
            ("warning",
             "AFC_BridgeBox chain1: T24 is the tool of Bambu lane lane24 "
             "(Bambu lanes lane24-lane36 are T24-T36). lane5 was mapped to it "
             "and is now T37. To give lane5 another tool outside T24-T36, use "
             "SET_MAP LANE=lane5 MAP=<T#>."),
            ("info",
             "AFC_BridgeBox chain1: the printer is idle, so the T#s the claim "
             "left in use are taken: lane24 is T24.")]
        # Two saves: lane24's T24 assignment, then the take's own.
        assert afc.save_vars.call_count == saves + 2
        assert m._deferred_takes == {}
        _p6_consistent(m)
        _p6_quiet(m)
        m._take_deferred_tools()                   # once
        assert _p6_log(m) == []

    def test_a_spare_bay_it_adopts_is_claimed_too(self, tmp_path,
                                                  monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, print_state="printing")
        _p6_other(m, "lane5", ["T28"])
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is _p6_unit(m, "Bravo")
        assert _p6_bay(m, "Bravo")["bound"] == C
        assert _p6_lane(m, "lane28").map == ["NONE"]
        assert list(m._deferred_takes) == ["lane28"]
        claim = _p6_claim_log(C, "Bravo")
        assert _p6_log(m) == claim[:2] + [(
            "info",
            "AFC_BridgeBox chain1: lane28 takes T28 from lane5 once the print "
            "ends, as the print may be using it; until then lane28 has no "
            "T#.")] + claim[2:]

    def test_a_macro_is_not_renamed_during_the_print(self, tmp_path,
                                                     monkeypatch):
        # force_assign_map lets TcmdAssign rename a macro out of the way.
        m = _p6_named(tmp_path, monkeypatch, print_state="printing")
        afc = m.printer.afc
        afc.force_assign_map = True
        handlers = afc.gcode.ready_gcode_handlers

        def _macro(gcmd: Any) -> None:
            """A user macro on T24."""
        afc.gcode.register_command("T24", _macro)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert handlers["T24"] is _macro
        assert "_T24" not in handlers
        assert _p6_lane(m, "lane24").map == ["NONE"]
        # TcmdAssign renames out of the way the T#s no macro holds.
        renamed = [line for n in (25, 26, 27) for line in (
            ("debug", f"<span class=warning--text>Existing command T{n} not "
                      "found in gcode_macros</span>"),
            ("debug", f"PREP-renaming macro T{n}"))]
        claim = _p6_claim_log(A, "Alpha")
        assert _p6_log(m) == claim[:2] + renamed + [(
            "info",
            "AFC_BridgeBox chain1: lane24 takes T24 from a macro once the "
            "print ends, as the print may be using it; until then lane24 has "
            "no T#.")] + claim[2:]
        m.printer.set_print_state("standby")
        _p6_quiet(m)
        m._take_deferred_tools()
        assert handlers["_T24"] is _macro
        assert handlers["T24"] == afc.cmd_CHANGE_TOOL
        assert _p6_lane(m, "lane24").map == ["T24"]
        assert afc.tool_cmds["T24"] == "lane24"
        assert _p6_log(m) == [
            ("debug", "PREP-renaming macro T24"),
            ("info",
             "AFC_BridgeBox chain1: the printer is idle, so the T#s the claim "
             "left in use are taken: lane24 is T24.")]

    def test_a_running_command_on_an_idle_printer_says_so(self, tmp_path,
                                                          monkeypatch):
        # No [virtual_sdcard], so no print_stats: only idle_timeout says a
        # command is running.
        m = _p6_named(tmp_path, monkeypatch)
        del m.printer._objects["print_stats"]
        _p6_other(m, "lane5", ["T24"])
        idle = m.printer.lookup_object("idle_timeout")
        idle.state = "Printing"
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        claim = _p6_claim_log(A, "Alpha")
        assert _p6_log(m) == claim[:2] + [(
            "info",
            "AFC_BridgeBox chain1: lane24 takes T24 from lane5 once the "
            "printer is idle, as a running command may be using it; until "
            "then lane24 has no T#.")] + claim[2:]
        _p6_quiet(m)
        m._take_deferred_tools()
        assert _p6_lane(m, "lane24").map == ["NONE"]
        assert _p6_log(m) == []
        idle.state = "Idle"
        m._take_deferred_tools()
        assert _p6_lane(m, "lane24").map == ["T24"]
        assert _p6_log(m) == [
            ("warning",
             "AFC_BridgeBox chain1: T24 is the tool of Bambu lane lane24 "
             "(Bambu lanes lane24-lane36 are T24-T36). lane5 was mapped to it "
             "and is now T37. To give lane5 another tool outside T24-T36, use "
             "SET_MAP LANE=lane5 MAP=<T#>."),
            ("info",
             "AFC_BridgeBox chain1: the printer is idle, so the T#s the claim "
             "left in use are taken: lane24 is T24.")]

    def test_a_claim_that_takes_no_live_tool_waits_for_nothing(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, print_state="printing")
        _p6_other(m, "lane5", ["T5"])
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is _p6_unit(m, "Alpha")
        assert _p6_lane(m, "lane24").map == ["T24"]
        assert m._deferred_takes == {}
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")

    def test_a_saved_map_that_leaves_the_home_tool_alone_goes_ahead(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, print_state="printing",
                      var={"Alpha": {"lane24": _p6_rec("T3")}},
                      owners=f"{A}:Alpha")
        _p6_other(m, "lane5", ["T24"])
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is _p6_unit(m, "Alpha")
        assert m.printer.afc.tool_cmds["T24"] == "lane5"
        assert _p6_lane(m, "lane24").map == ["T3"]
        assert m._deferred_takes == {}
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane24->T3.")

    # ── what a restart before the unit's scan priming boots on ───────────

    #: What the claimed lanes carry, as their records saved it.
    SPOOLS = {"lane24": {"spool_id": 159, "material": "PLA",
                         "color": "#0086D6", "weight": 412.0,
                         "extruder_temp": 225.0, "sub_type": "Matte",
                         "runout_lane": "lane25"},
              "lane25": {"material": "PETG", "color": "#FF0000",
                         "weight": 800.0, "extruder_temp": 245.0},
              "lane26": {"spool_id": 12, "material": "ABS",
                         "color": "#000000", "weight": 950.0}}

    def _spools_on(self, master: afcBridgeBox) -> None:
        """Assert every lane carries what SPOOLS says."""
        for name, rec in self.SPOOLS.items():
            lane = _p6_lane(master, name)
            assert {key: getattr(lane, key) for key in rec} == rec, name

    def test_the_records_survive_two_restarts_in_the_window(
            self, tmp_path, monkeypatch):
        var = {"Alpha": {name: _p6_rec(f"T{name[4:]}", **rec)
                         for name, rec in self.SPOOLS.items()}}
        var["Alpha"]["lane24"]["tool_loaded"] = True
        for boot in range(3):
            m = _p6_named(tmp_path, monkeypatch, owners=f"{A}:Alpha",
                          var=var if boot == 0 else None)
            if boot:
                assert set(m._held["Alpha"]["lanes"]) == {
                    "lane24", "lane25", "lane26", "lane27"}
            m.printer.afc.tools["extruder"].lane_loaded = "lane24"
            _p6_quiet(m)
            assert m._claim_pool_unit(A, "boxed") is not None
            assert _p6_log(m) == _p6_claim_log(A, "Alpha")
            self._spools_on(m)
            assert _p6_lane(m, "lane24").tool_loaded is True
            # AFC's writer puts the claim's last save in the file the next
            # boot reads.
            saved = drain_var_writes(m.printer)[-1]
            write_unit_vars(m.printer, saved)
            for name, rec in self.SPOOLS.items():
                assert {key: saved["Alpha"][name][key] for key in rec} == rec
            assert saved["Alpha"]["lane24"]["tool_loaded"] is True

    # ── a state from before bay_owner: the guessed owner ─────────────────

    #: What AFC.var.unit holds for Alpha at the first start after upgrade.
    GUESS_VAR = {"Alpha": {"lane24": _p6_rec("T24", spool_id=159,
                                             material="PLA", color="#0086D6",
                                             weight=412.0),
                           "lane25": _p6_rec("T25", material="PETG")}}

    def test_another_claim_writes_the_guess_with_it(self, tmp_path,
                                                    monkeypatch):
        # AAAA stays offline all session while CCCC claims a spare; the
        # restart holds AAAA's records again, and AAAA gets them.
        _p6_named(tmp_path, monkeypatch, ready=False)      # pins AAAA:Alpha
        m = self._restart(tmp_path, monkeypatch, var=self.GUESS_VAR)
        assert m._state_get(SEC, "bay_owner") is None
        assert m._owners() == {"Alpha": A}
        afc = m.printer.afc
        afc.save_vars()                            # PREP's first save
        assert drain_var_writes(m.printer)[-1]["Alpha"] == (
            self.GUESS_VAR["Alpha"])
        assert m._claim_pool_unit(C, "boxed") is _p6_unit(m, "Bravo")
        assert m._state_get(SEC, "bay_owner") == "AAAA:Alpha, CCCC:Bravo"
        drain_var_writes(m.printer)
        afc.save_vars()
        last = drain_var_writes(m.printer)[-1]
        assert last["Alpha"] == self.GUESS_VAR["Alpha"]
        write_unit_vars(m.printer, last)
        again = self._restart(tmp_path, monkeypatch)
        assert again._held["Alpha"] == {"uid": A,
                                        "lanes": self.GUESS_VAR["Alpha"]}
        _p6_quiet(again)
        assert again._claim_pool_unit(A, "boxed") is not None
        assert _p6_lane(again, "lane24").spool_id == 159
        assert _p6_log(again) == _p6_claim_log(A, "Alpha")

    def test_the_floater_takes_the_spare_its_records_are_on(
            self, tmp_path, monkeypatch):
        # roster: leaves CCCC out; the recorded roster lists it, and the
        # records on the second spare are guessed for it.
        _p6_named(tmp_path, monkeypatch, ready=False)      # pins AAAA:Alpha
        rec = _p6_rec("T32", spool_id=7, material="PETG")
        m = _p6_named(tmp_path, monkeypatch, recorded=f"boxed:{A}, boxed:{C}",
                      roster=f"boxed:{A}", var={"Charlie": {"lane32": rec}})
        assert m._held == {"Charlie": {"uid": C, "lanes": {"lane32": rec}}}
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is _p6_unit(m, "Charlie")
        lane = _p6_lane(m, "lane32")
        assert (lane.map, lane.spool_id, lane.material) == (["T32"], 7,
                                                            "PETG")
        assert m._held == {"Charlie": {"uid": C, "lanes": {"lane32": rec}}}
        assert _p6_log(m) == _p6_claim_log(C, "Charlie")

    # ── a lane another extruder records as loaded ────────────────────────

    @staticmethod
    def _two_tools(master: afcBridgeBox) -> None:
        """
        extruder (the active tool) holds lane5, a lane of another unit;
        extruder1 records lane28 as loaded.
        """
        _p6_other(master, "lane5", ["T5"])
        master.printer.afc.tools["extruder"].lane_loaded = "lane5"
        ext1 = add_extruder(master.printer, "extruder1")
        ext1.lane_loaded = "lane28"

    def test_no_record_attributed_says_it_cannot_tell(self, tmp_path,
                                                      monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        self._two_tools(m)
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is _p6_unit(m, "Bravo")
        assert _p6_lane(m, "lane28").tool_loaded is False
        claim = _p6_claim_log(C, "Bravo")
        assert _p6_log(m) == [(
            "warning",
            "AFC bambu Bravo: extruder1 records lane28 as loaded, but no "
            "saved record of lane28 is attributed to this unit (CCCC), so "
            "AFC cannot tell which unit the filament in extruder1 is from, "
            "and lane28 is left unloaded. Unload that filament by hand and "
            "run UNSET_LANE_LOADED with extruder1 as the active tool to "
            "clear the record.")] + claim

    def test_another_units_records_say_it_is_not_this_units(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Bravo": {"lane28": _p6_rec(
                          "T28", material="PLA", tool_loaded=True)}},
                      owners=f"{D}:Bravo")
        self._two_tools(m)
        assert m._held["Bravo"]["uid"] == D
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is _p6_unit(m, "Bravo")
        assert _p6_lane(m, "lane28").tool_loaded is False
        claim = _p6_claim_log(C, "Bravo")
        assert _p6_log(m) == [(
            "warning",
            "AFC bambu Bravo: extruder1 records lane28 as loaded, but lane28 "
            "was not saved loaded under this unit (CCCC), so the filament "
            "in extruder1 is not from this unit and lane28 is left "
            "unloaded. Unload that filament by hand and run "
            "UNSET_LANE_LOADED with extruder1 as the active tool to clear "
            "the record.")] + claim

    # ── the claim's own saves ────────────────────────────────────────────

    def test_no_save_has_a_lane_waiting_for_its_map_on_none(
            self, tmp_path, monkeypatch):
        # A restart between two of the claim's saves restores what the last
        # one written says.
        var = {"Alpha": {"lane24": _p6_rec("T3"), "lane25": _p6_rec("T25"),
                         "lane26": _p6_rec("NONE", current="")}}
        m = _p6_named(tmp_path, monkeypatch, var=var, owners=f"{A}:Alpha")
        drain_var_writes(m.printer)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane24->T3, lane26->NONE.")
        snaps = [s["Alpha"] for s in drain_var_writes(m.printer)]
        assert [{name: rec["map"] for name, rec in snap.items()}
                for snap in snaps] == [{
                    "lane24": "T3", "lane25": "T25", "lane26": "NONE",
                    "lane27": "T27"}] * 5
        assert m._claim_plans == {}

    def test_the_claim_says_what_a_reset_does(self, tmp_path, monkeypatch):
        # Twelve lanes of another unit on T0-T11 ahead of the pool (AMS bays
        # lane12-lane27, the HT on lane28); lane5's config gives it T28, the
        # HT's home T#.
        m = _p6_chain(tmp_path, monkeypatch,
                      recorded=f"boxed:{A}, boxed:{B}, ht:{H}", pool_ams=4,
                      pool_ht=1, lane_base=12,
                      ams_names="Alpha, Bravo, Charlie, Delta",
                      ht_names="Hot")
        for n in range(12):
            maps = ["T28"] if n == 5 else [f"T{n}"]
            _p6_other(m, f"lane{n}", maps, config_map=maps if n == 5 else ())
        assert m._claim_pool_unit(A, "boxed") is not None
        _p6_quiet(m)
        assert m._claim_pool_unit(H, "ht") is _p6_unit(m, "Hot")
        assert _p6_lane(m, "lane28").map == ["T28"]
        assert m.printer.afc.lanes["lane5"].map == ["T29"]
        claim = _p6_claim_log(H, "Hot", "ht", 1)
        assert _p6_log(m) == claim[:2] + [(
            "warning",
            "AFC_BridgeBox chain1: T28 is the tool of Bambu lane lane28 "
            "(Bambu lanes lane12-lane28 are T12-T28). lane5 was mapped to it "
            "and is now T29. To give lane5 another tool outside T12-T28, use "
            "SET_MAP LANE=lane5 MAP=<T#>. Also set map: in "
            "[AFC_stepper lane5] outside T12-T28, or AFC_RESET_MAPPING puts "
            "lane5 back on T28 and lane28 on another T#.")] + claim[2:]
        _p6_consistent(m)

    # ── a bay the claim adopts and the claim does not go through ─────────

    def test_a_failed_claim_hands_an_adopted_bay_back_but_keeps_a_held_one(
            self, tmp_path, monkeypatch):
        # No bridge on the port, so each claim() fails; FORGET and ASSIGN
        # find CCCC offline.
        m = _p6_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1)
        del live_bridges()[m.serial_port]

        def _failed(bay: str) -> None:
            """CCCC's claim fails on ``bay``; it logs the unit's line."""
            _p6_quiet(m)
            assert m._claim_pool_unit(C, "boxed") is None
            assert _p6_log(m) == [(
                "warning",
                f"AFC bambu {bay}: claim found no live bridge on "
                "/dev/serial/by-id/usb-chain1-if00; unit stays offline "
                "until restart.")]
        # A spare adopted for the claim goes back to the pool.
        _failed("Bambu_AMS_2")
        assert _p6_bay(m, "Bambu_AMS_2")["uid"] is None
        # So does a recorded bay FORGET freed.
        m.cmd_AFC_BRIDGEBOX_FORGET(FakeGcmd({"UID": A}))
        assert _p6_bay(m, "Bambu_AMS_1")["uid"] is None
        _failed("Bambu_AMS_1")
        assert _p6_bay(m, "Bambu_AMS_1")["uid"] is None
        # A bay the uid is saved on keeps it.
        m.cmd_AFC_BRIDGEBOX_ASSIGN(FakeGcmd({"UID": C,
                                             "NAME": "Bambu_AMS_3"}))
        assert _p6_bay(m, "Bambu_AMS_3")["uid"] == C
        _failed("Bambu_AMS_3")
        assert _p6_bay(m, "Bambu_AMS_3")["uid"] == C

    @pytest.mark.parametrize("gone", ["missing", "not-a-pool-unit"])
    def test_a_bay_with_no_pool_unit_behind_it_is_handed_back(
            self, tmp_path, monkeypatch, gone):
        m = _p6_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, online=())
        if gone == "missing":
            del m.printer._objects["AFC_BambuAMS Bambu_AMS_2"]
        else:
            _p6_unit(m, "Bambu_AMS_2").pool = False
        _p6_quiet(m)
        assert m._claim_pool_unit(C, "boxed") is None
        assert _p6_bay(m, "Bambu_AMS_2")["uid"] is None
        assert m._bay_of_uid(C) is None
        assert m.printer.afc.lanes == {}
        assert _p6_log(m) == []

    def test_the_claim_methods_bridgebox_calls_are_on_the_unit(
            self, tmp_path, monkeypatch):
        # The master reaches the unit's set_master, hold_lanes, claim and
        # release by name and skips a unit without them: each one runs on
        # the real unit.
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Alpha": {"lane24": _p6_rec("T24", spool_id=159)}},
                      owners=f"{A}:Alpha")
        unit = _p6_unit(m, "Alpha")
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is unit
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")
        assert unit._master is m
        assert (unit.pool, unit.unit_uid) == (False, A)
        assert _p6_lane(m, "lane24").spool_id == 159
        m._release_pool_unit(A)
        assert (unit.pool, unit.unit_uid) == (True, None)

    # ── a unit or lane missing what the claim reaches for ────────────────

    #: What the master holds for AAAA on Alpha: lane24 saved on T7.
    HELD = {"Alpha": {"uid": A, "lanes": {"lane24": _p6_rec(
        "T7", spool_id=159, material="PLA")}}}

    def test_a_unit_without_hold_lanes_is_handed_nothing(
            self, tmp_path, monkeypatch):
        # No setter: the records stay with the master and the map still
        # comes from them; the unit reads AFC.var.unit instead.
        m = _p6_named(tmp_path, monkeypatch, owners=f"{A}:Alpha")
        unit = _p6_unit(m, "Alpha")
        unit.hold_lanes = None
        m._held = {k: dict(v) for k, v in self.HELD.items()}
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is unit
        assert unit._held_lanes is None
        lane = _p6_lane(m, "lane24")
        assert (lane.spool_id, lane.material) == (None, None)
        assert (lane.map, lane.current_map) == (["T7"], "T7")
        assert (_p6_bay(m, "Alpha")["uid"], _p6_bay(m, "Alpha")["bound"]) == (
            A, A)
        assert _p6_log(m) == _p6_claim_log(
            A, "Alpha", tail=" Saved maps: lane24->T7.")

    def test_a_failed_claim_of_a_unit_without_hold_lanes_takes_nothing_back(
            self, tmp_path, monkeypatch):
        # No bridge on the port: claim() fails, and a unit with the setter
        # would be handed {} again; this one was never handed anything.
        m = _p6_named(tmp_path, monkeypatch, owners=f"{A}:Alpha")
        unit = _p6_unit(m, "Alpha")
        unit.hold_lanes = None
        m._held = {k: dict(v) for k, v in self.HELD.items()}
        del live_bridges()[m.serial_port]
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is None
        assert unit._held_lanes is None
        afc = m.printer.afc
        assert (afc.lanes, afc.tool_cmds) == ({}, {})
        assert (_p6_bay(m, "Alpha")["uid"], _p6_bay(m, "Alpha")["bound"]) == (
            A, None)
        assert _p6_log(m) == [(
            "warning",
            "AFC bambu Alpha: claim found no live bridge on "
            "/dev/serial/by-id/usb-chain1-if00; unit stays offline until "
            "restart.")]

    def test_a_failed_claim_hands_the_unit_an_empty_hold(
            self, tmp_path, monkeypatch):
        # The same failure on a unit with the setter: what it was handed
        # before claim() is taken back.
        m = _p6_named(tmp_path, monkeypatch, owners=f"{A}:Alpha")
        unit = _p6_unit(m, "Alpha")
        m._held = {k: dict(v) for k, v in self.HELD.items()}
        del live_bridges()[m.serial_port]
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is None
        assert unit._held_lanes == {}
        assert _p6_log(m) == [(
            "warning",
            "AFC bambu Alpha: claim found no live bridge on "
            "/dev/serial/by-id/usb-chain1-if00; unit stays offline until "
            "restart.")]

    def test_a_bay_lane_klippy_did_not_register_is_skipped(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        del m.printer._objects["AFC_lane lane25"]
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is _p6_unit(m, "Alpha")
        afc = m.printer.afc
        assert sorted(afc.lanes) == ["lane24", "lane26", "lane27"]
        assert afc.tool_cmds == {"T24": "lane24", "T26": "lane26",
                                 "T27": "lane27"}
        assert _p6_log(m) == _p6_claim_log(A, "Alpha", lanes=3)

    def test_a_lane_with_no_send_lane_data_is_still_mapped(
            self, tmp_path, monkeypatch):
        m = _p6_named(tmp_path, monkeypatch)
        lane24 = _p6_lane(m, "lane24")
        lane24.send_lane_data = None
        lane25 = _p6_lane(m, "lane25")
        sent = self._pushes(monkeypatch, lane25)
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is not None
        assert (lane24.map, lane24.current_map) == (["T24"], "T24")
        assert m.printer.afc.tool_cmds["T24"] == "lane24"
        # The lanes that have it are still pushed.
        assert sent == [["T25"]]
        assert _p6_log(m) == _p6_claim_log(A, "Alpha")


class TestAfcBridgeBoxTakeDeferredTools:
    """The T#s a claim during a print left in use, taken once it ends."""

    #: What _take_home_tool says when lane24 takes T24 from lane5.
    MOVED = ("warning",
             "AFC_BridgeBox chain1: T24 is the tool of Bambu lane lane24 "
             "(Bambu lanes lane24-lane36 are T24-T36). lane5 was mapped to it "
             "and is now T37. To give lane5 another tool outside T24-T36, use "
             "SET_MAP LANE=lane5 MAP=<T#>.")

    @staticmethod
    def _idle(done: str) -> str:
        """
        :param done: what each lane is now
        :return str: the line the take says
        """
        return (f"AFC_BridgeBox chain1: the printer is idle, so the T#s the "
                f"claim left in use are taken: {done}.")

    @staticmethod
    def _bravo_on_t24(tmp_path: pathlib.Path,
                      monkeypatch: pytest.MonkeyPatch,
                      maps: str) -> afcBridgeBox:
        """
        CCCC claimed onto Bravo, whose lane28 saved ``maps`` (holding T24,
        lane24's home), then a print starts and AAAA claims Alpha.

        :param maps: lane28's saved map
        :return afcBridgeBox: the master, its logs cleared after the claims
        """
        m = _p6_named(tmp_path, monkeypatch,
                      var={"Bravo": {"lane28": _p6_rec(maps)}},
                      owners=f"{C}:Bravo")
        assert m._claim_pool_unit(C, "boxed") is _p6_unit(m, "Bravo")
        assert _p6_lane(m, "lane28").map == [cmd.strip()
                                             for cmd in maps.split(",")]
        m.printer.set_print_state("printing")
        _p6_quiet(m)
        assert m._claim_pool_unit(A, "boxed") is _p6_unit(m, "Alpha")
        assert _p6_lane(m, "lane24").map == ["NONE"]
        return m

    def test_a_tool_the_lane_is_given_meanwhile_stays(self, tmp_path,
                                                      monkeypatch):
        m = _p6_named(tmp_path, monkeypatch, print_state="printing")
        lane5 = _p6_other(m, "lane5", ["T24"])
        assert m._claim_pool_unit(A, "boxed") is not None
        afc = m.printer.afc
        lane24 = _p6_lane(m, "lane24")
        lane24.map, lane24.current_map = ["T40"], "T40"   # SET_MAP
        afc.tool_cmds["T40"] = "lane24"
        afc.gcode.register_command("T40", afc.cmd_CHANGE_TOOL)
        m.printer.set_print_state("complete")
        _p6_quiet(m)
        m._take_deferred_tools()
        assert (lane24.map, lane24.current_map) == (["T24", "T40"], "T24")
        assert lane5.map == ["T37"]
        assert m._deferred_takes == {}
        assert _p6_log(m) == [self.MOVED,
                              ("info", self._idle("lane24 is T24, T40"))]
        _p6_consistent(m)

    def test_the_wait_and_the_take_go_to_afc_log_only(self, tmp_path,
                                                      monkeypatch):
        m = self._bravo_on_t24(tmp_path, monkeypatch, "T24")
        lane24, lane28 = _p6_lane(m, "lane24"), _p6_lane(m, "lane28")
        assert lane28.map == ["T24"]
        claim = _p6_claim_log(A, "Alpha")
        assert _p6_log(m) == claim[:2] + [(
            "debug",
            "AFC_BridgeBox chain1: lane24 takes T24 from lane28 once the print "
            "ends, as the print may be using it; until then lane24 has no "
            "T#.")] + claim[2:]
        assert sorted(m._deferred_takes) == ["lane24"]
        m.printer.set_print_state("complete")
        _p6_quiet(m)
        m._take_deferred_tools()
        assert (lane24.map, lane28.map) == (["T24"], ["T28"])
        assert m._deferred_takes == {}
        assert _p6_log(m) == [
            ("debug",
             "AFC_BridgeBox chain1: T24 is the tool of Bambu lane lane24. "
             "lane28 was mapped to it and is back on T28."),
            ("debug", self._idle("lane24 is T24"))]
        _p6_consistent(m)

    @pytest.mark.parametrize("maps,after,quiet", [
        ("T24, T28", ["T28"], True),
        ("T24, T28, T40", ["T28", "T40"], True),
        ("T24, T40", ["T40"], False)])
    def test_a_holder_mapped_to_more_t_numbers(self, tmp_path, monkeypatch,
                                               maps, after, quiet):
        # A holder that keeps its own home T# ends where a claim with no
        # print running leaves it; one that keeps only another T# does not.
        m = self._bravo_on_t24(tmp_path, monkeypatch, maps)
        level = "debug" if quiet else "info"
        claim = _p6_claim_log(A, "Alpha")
        assert _p6_log(m) == claim[:2] + [(
            level,
            "AFC_BridgeBox chain1: lane24 takes T24 from lane28 once the print "
            "ends, as the print may be using it; until then lane24 has no "
            "T#.")] + claim[2:]
        assert sorted(m._deferred_takes) == ["lane24"]
        m.printer.set_print_state("complete")
        _p6_quiet(m)
        m._take_deferred_tools()
        assert (_p6_lane(m, "lane24").map, _p6_lane(m, "lane28").map) == (
            ["T24"], after)
        assert m._deferred_takes == {}
        now = ", ".join(after)
        assert _p6_log(m) == [
            ("debug" if quiet else "warning",
             f"AFC_BridgeBox chain1: T24 is the tool of Bambu lane lane24 "
             f"(Bambu lanes lane24-lane36 are T24-T36). lane28 was mapped to "
             f"it and is now {now}."),
            (level, self._idle("lane24 is T24"))]
        _p6_consistent(m)


class TestAfcBridgeBoxNoBayMessage:
    """The console line for a unit that finds no free bay of its family."""

    def test_all_ams_ranks_held_by_a_bay_with_no_uid_does_not_fail(
            self, tmp_path, monkeypatch):
        # A bay bound live to a unit the restart roster does not hold yet:
        # four AMS bays are built and each is held, so the line names the
        # four holders, and neither pool_ams nor a restart is offered.
        m = _p6_chain(tmp_path, monkeypatch, recorded=FOUR_p6, pool_ams=4,
                      pool_ht=2, ready=False)
        bay4 = _p6_bay(m, "Bambu_AMS_4")
        bay4["uid"], bay4["bound"] = None, G
        m._state_set({SEC: {"roster": f"boxed:{A}, boxed:{B}, boxed:{C}, "
                                      f"ht:{H}"}})
        _p6_quiet(m)
        assert m._no_bay_message(E, "ams") == (
            "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays "
            "belong to other units (Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), "
            "Bambu_AMS_3 (CCCC), Bambu_AMS_4 (GGGG)), and a Bambu bus "
            "addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
            "one. Offline: Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), "
            "Bambu_AMS_3 (CCCC), Bambu_AMS_4 (GGGG). If this AMS replaces "
            "one of them, AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=<that unit's "
            "uid> frees that bay and this AMS claims it live. "
            "AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID=EEEE OLD=<that unit's "
            "bay> does both in one step.")
        assert _p6_log(m) == []

    def test_a_roster_option_listing_it_fifth_names_no_replace(
            self, tmp_path, monkeypatch):
        m = _p6_stuck(tmp_path, monkeypatch, roster=FOUR_p6 + f", boxed:{E}")
        _p6_tick(m, 100, 130)
        _p6_quiet(m)
        assert m._no_bay_message(E, "ams") == (
            "AFC_BridgeBox chain1: AMS EEEE has no bay: all 4 AMS bays "
            "belong to other units (Bambu_AMS_1 (AAAA), Bambu_AMS_2 (BBBB), "
            "Bambu_AMS_3 (CCCC), Bambu_AMS_4 (DDDD)), and a Bambu bus "
            "addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
            "one. Bambu_AMS_4 (DDDD) is offline: if this AMS replaces it, "
            "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=DDDD frees that bay and "
            "this AMS claims it live; remove it from roster: too.")
        assert _p6_log(m) == []
        assert _p6_console(m) == []
        assert m._replace_offered == set()


class TestAfcBridgeBoxCheckMovedLoaded:
    """A loaded lane this boot gave to another unit."""

    @staticmethod
    def _five_bays(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                   loaded: Optional[str]) -> afcBridgeBox:
        """
        EEEE recorded on Bambu_AMS_5 (lane40-lane43) and the HT on lane44:
        this boot gives lane40 to the HT and leaves EEEE waiting.

        :param loaded: the lane the extruder records as loaded
        :return afcBridgeBox: the master, the HT online
        """
        m = _p6_chain(
            tmp_path, monkeypatch, recorded=FOUR_p6 + f", boxed:{E}",
            state={"name_map": (f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                                f"{C}:Bambu_AMS_3, {D}:Bambu_AMS_4, "
                                f"{E}:Bambu_AMS_5, {H}:Bambu_AMS_HT_1"),
                   "lane_map": (f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, "
                                f"{E}:40:4, {H}:44:1")},
            pool_ams=4, pool_ht=2, online=(H,), htmask=1 << 6, ready=False)
        m.printer.afc.tools["extruder"].lane_loaded = loaded
        return m

    def test_the_new_owner_leaves_the_follower_off_and_the_user_is_told(
            self, tmp_path, monkeypatch):
        m = self._five_bays(tmp_path, monkeypatch, "lane40")
        unit = m._claim_pool_unit(H, "ht")
        assert unit is _p6_unit(m, "Bambu_AMS_HT_1")
        lane40 = _p6_lane(m, "lane40")
        bridge = live_bridges()[m.serial_port]
        # The unit asks the master and leaves the record alone.
        unit._id_resolved, unit._loaded_restore_done = True, False
        bridge.sent.clear()
        _p6_quiet(m)
        assert unit._restore_loaded_follower() is False
        assert (lane40.tool_loaded, bridge.sent) == (False, [])
        assert _p6_log(m) == []
        m._check_moved_loaded()
        m._check_moved_loaded()
        assert _p6_log(m) == [(
            "warning",
            "AFC_BridgeBox chain1: extruder records lane40 as loaded, but "
            "lane40 belonged to AMS EEEE (Bambu_AMS_5), and this boot's "
            "layout gives it to Bambu_AMS_HT_1. The filament in extruder is "
            "from EEEE, so Bambu_AMS_HT_1 leaves its follower off on lane40. "
            "Unload that filament by hand and run UNSET_LANE_LOADED to clear "
            "the record.")]
        # Lanes no record names stop being tracked; the loaded one stays.
        assert set(m._lane_moves) == {"lane40"}
        assert m._moved_told == {"lane40"}
        # Once the record is cleared, a lane loaded from the new owner is
        # the new owner's.
        m.printer.afc.tools["extruder"].lane_loaded = None
        m._check_moved_loaded()
        assert (m._lane_moves, m._moved_told) == ({}, set())
        assert m.loaded_lane_moved("lane40") is False
        m.printer.afc.tools["extruder"].lane_loaded = "lane40"
        lane40.tool_loaded = True
        unit._loaded_restore_done = False
        _p6_quiet(m)
        assert unit._restore_loaded_follower() is True
        assert bridge.sent[-2:] == [
            {"cmd": "select", "slot": 0, "unit": 0},
            {"cmd": "assist", "on": True, "slot": 0, "unit": 0}]
        assert _p6_log(m) == [
            ("debug",
             "AFC bambu Bambu_AMS_HT_1: lane40 loaded at startup, "
             "re-asserting AMS loaded state + follower"),
            ("info",
             "AFC bambu Bambu_AMS_HT_1: restored the follower for lane40 "
             "(AFC records it loaded to the toolhead) -- mode:4, one-shot.")]

    def test_nothing_is_decided_before_prep_restores_the_records(
            self, tmp_path, monkeypatch):
        m = self._five_bays(tmp_path, monkeypatch, "lane40")
        m.printer.afc.prep_done = False
        _p6_quiet(m)
        m._check_moved_loaded()
        assert _p6_log(m) == []
        assert getattr(m, "_moved_told", None) is None
        assert m.loaded_lane_moved("lane40") is True
        assert m.loaded_lane_moved("lane41") is True


class TestAfcBridgeBoxReapplyModelOverride:
    """A model section applied to a bay once its model is known."""

    def test_a_claim_time_override_waits_for_the_index(self, tmp_path,
                                                       monkeypatch):
        # With no index of the caller's own, the push still waits for the
        # unit's pinned one: claim() leaves it unresolved until its chain
        # request is answered.
        a24 = "A" * 24
        m = _p6_chain(tmp_path, monkeypatch, roster=f"boxed:{a24}",
                      uids=(a24,), online=(a24,),
                      sections={"AFC_BridgeBox ams1": {
                          "measure_on_insert": "True"}})
        unit = m._claim_pool_unit(a24, "boxed")
        assert (unit._id_resolved, unit.measure_on_insert) == (False, False)
        bridge = live_bridges()[m.serial_port]
        bridge.sent.clear()
        _p6_quiet(m)
        applied = [(
            "info",
            "AFC_BridgeBox chain1: Bambu_AMS_1 is a ams1; applied "
            "measure_on_insert=True from [AFC_BridgeBox ams1] (the bay was "
            "fabricated as a spare, before its model was known)")]
        m._reapply_model_override(unit, "ams1")
        assert unit.measure_on_insert is True
        assert bridge.sent == []
        assert _p6_log(m) == applied
        unit._id_resolved, unit.measure_on_insert = True, False
        _p6_quiet(m)
        m._reapply_model_override(unit, "ams1")
        assert unit.measure_on_insert is True
        assert bridge.sent[:2] == [{"cmd": "htunit", "unit": 0, "on": 0},
                                   {"cmd": "capen", "unit": 0, "on": 1}]
        assert bridge.sent[-1] == {"cmd": "idsave"}
        assert _p6_log(m) == applied


class TestAfcBridgeBoxGetStatus:
    """The chain as the status API reports it."""

    class _TornBridge(FakeBridge):
        """
        A bridge whose reader replaces the chain cache right after the
        first read: whatever is read after it sees the next reply.
        """

        def __init__(self, first: tuple, then: tuple,
                     online: Sequence[bool]) -> None:
            """
            :param first: (uids, a2mask, a2asks) of the first reply
            :param then: the same of the reply that lands after one read
            :param online: chain index -> online flag
            """
            super().__init__(uids=first[0], online=online)
            self.dialect = (first[1], list(first[2]))
            self._then: Optional[tuple] = then
            self.snapshots = 0

        def _landed(self, value: Any) -> Any:
            """
            :param value: what the read returned
            :return Any: ``value``; the next reply has landed since
            """
            if self._then is not None:
                self.uids = list(self._then[0])
                self.dialect = (self._then[1], list(self._then[2]))
                self._then = None
            return value

        def chain_snapshot(self) -> Dict[str, Any]:
            """:return dict: one reply's chain, counted in ``snapshots``"""
            self.snapshots += 1
            return self._landed(super().chain_snapshot())

        def chain_uids(self) -> List[str]:
            """:return list: the chain's uids"""
            return self._landed(super().chain_uids())

        def chain_dialect(self) -> Any:
            """:return tuple: the AMS 2 mask and ask counts"""
            return self._landed(super().chain_dialect())

    def test_status_reports_the_roster_as_fabricated(self, tmp_path):
        m = make_bridgebox(tmp_path, printer=make_printer(),
                           roster="ht:1111222233334444")
        _p6_quiet(m)
        assert m.get_status() == {
            "units": [{"model": "ht", "uid": "1111222233334444"}],
            "lane_base": 24, "dialect": {}, "roster_source": "option",
            "tombstones": [], "missing": {}, "waiting_for_bay": [],
            "pending_restart": [], "watch_ticks": 0, "held_bays": {},
            "watch_state": "not-started"}
        assert _p6_log(m) == []

    def test_status_rows_come_from_one_snapshot(self, tmp_path, monkeypatch):
        a24, b24 = "A" * 24, "B" * 24
        m = _p6_chain(tmp_path, monkeypatch, online=None, ready=False,
                      recorded=f"boxed:{a24}")
        bridge = self._TornBridge(([a24], 0, [2]), ([b24], 0b1, [7]),
                                  online=[True])
        live_bridges()[m.serial_port] = bridge
        _p6_quiet(m)
        assert m.get_status()["dialect"] == {
            "ams1_ask_floor": 12,
            "chain": [{"i": 0, "uid": a24, "online": True, "ht": False,
                       "answered_3702": False, "asks_3702": 2}]}
        assert bridge.snapshots == 1
        assert _p6_log(m) == []

    def test_status_reports_pool_absences_until_release_grace(
            self, tmp_path, monkeypatch):
        m = _p6_stuck(tmp_path, monkeypatch)
        _p6_tick(m, 100, 105)
        _p6_quiet(m)
        status = m.get_status()
        assert (status["missing"], status["waiting_for_bay"]) == (
            {D: 5}, [E])
        assert _p6_log(m) == []
        _p6_tick(m, 106, 110)
        assert m.get_status()["missing"] == {}
        assert m._missing_since == {D: 100.0}

    def test_status_without_a_reactor_reports_no_ages(
            self, tmp_path, monkeypatch):
        # With no reactor there is no clock: D's age is unknown, not
        # counted from 0.0.
        m = _p6_stuck(tmp_path, monkeypatch)
        _p6_tick(m, 100, 105)

        def _no_reactor() -> Any:
            """:raises RuntimeError: the reactor cannot be had"""
            raise RuntimeError("no reactor")
        monkeypatch.setattr(m.printer, "get_reactor", _no_reactor)
        _p6_quiet(m)
        status = m.get_status()
        assert (status["missing"], status["waiting_for_bay"]) == (
            {D: None}, [E])
        assert _p6_log(m) == []

    def test_status_does_not_list_a_unit_saved_this_session_as_a_tombstone(
            self, tmp_path, monkeypatch):
        m = _p6_chain(tmp_path, monkeypatch, recorded=f"boxed:{A}",
                      pool_ams=3, pool_ht=1, uids=(A, C), online=(C,))
        m.cmd_AFC_BRIDGEBOX_ASSIGN(FakeGcmd({"UID": C,
                                             "NAME": "Bambu_AMS_3"}))
        assert m._lane_map == {A: (24, 4), C: (32, 4)}
        _p6_quiet(m)
        assert m.get_status()["tombstones"] == []
        assert _p6_log(m) == []
        live_bridges()[m.serial_port].status = {"units": [
            {"n": 0, "online": False}, {"n": 1, "online": False}]}
        _p6_tick(m, 100, 100)
        m._release_pool_unit(C)
        assert _p6_bay(m, "Bambu_AMS_3")["bound"] is None
        _p6_quiet(m)
        assert m.get_status()["tombstones"] == []
        assert _p6_log(m) == []


class TestLoadConfigPrefix:
    """serial_port tells a chain master from an override section."""

    def test_a_section_without_serial_port_is_an_override_not_a_master(
            self):
        printer = make_printer()
        held = load_config_prefix(BambuConfig(
            "AFC_BridgeBox Bambu_AMS_1", printer,
            {"measure_on_insert": "True"}))
        assert isinstance(held, BridgeBoxOverrideHolder)
        assert held.name == "Bambu_AMS_1"
        assert printer.loaded == []
        assert (printer.afc.logger.messages, printer.gcode.messages) == (
            [], [])

    def test_a_section_with_serial_port_is_a_master(self, tmp_path):
        printer = make_printer()
        printer.add_section(SEC, {})
        master = load_config_prefix(BambuConfig(SEC, printer, dict(
            serial_port="/dev/serial/by-id/usb-chain1-if00",
            extruder="extruder", roster="", pool_ams=0, pool_ht=0,
            auto_vars_file=str(tmp_path / "AFC_auto_vars.cfg"),
            state_file=str(tmp_path / "AFC_BridgeBox.cfg"))))
        assert isinstance(master, afcBridgeBox)
        assert master.name == "chain1"
        assert (printer.afc.logger.messages, printer.gcode.messages) == (
            [], [])


class TestMasterOptions:
    """The master's own options never reach a unit as chain defaults."""

    def test_the_fence_covers_the_masters_identity_options(self, tmp_path):
        printer = make_printer()
        m = make_bridgebox(tmp_path, printer=printer, ams_names="Alpha",
                           ht_names="Hot", pool_ams=1, pool_ht=1,
                           lane_base=24, buffer="Bamb_1",
                           measure_on_insert="False")
        # Only the one option the master does not consume is a default.
        assert m._chain_defaults == {"measure_on_insert": "False"}
        # The named buffer is the chain's, so none is fabricated for it.
        assert [s for s, _w in printer.loaded
                if s.startswith("AFC_buffer ")] == []
        sections = {s: dict(w.fileconfig.items(s))
                    for s, w in printer.loaded
                    if s.startswith("AFC_BambuAMS ")}
        assert sorted(sections) == ["AFC_BambuAMS Alpha", "AFC_BambuAMS Hot"]
        for keys in sections.values():
            assert (keys["measure_on_insert"], keys["buffer"]) == ("False",
                                                                   "Bamb_1")
            for opt in ("pool_ams", "pool_ht", "ams_names", "ht_names",
                        "roster", "state_file", "lane_base"):
                assert opt not in keys, opt
        assert (printer.afc.logger.messages, printer.gcode.messages) == (
            [], [])


class TestSlotsByModel:
    """The lanes, heater and ceiling each roster model is built with."""

    #: The lanes, heater and dry_max_temp each model's fabricated unit gets.
    BUILT = {"ams1": (4, None, None), "ams2": (4, "True", "65"),
             "boxed": (4, None, None), "lite": (4, None, None),
             "ht": (1, "True", "85")}

    def test_bridgebox_tables_follow_the_unit_model_table(self, tmp_path):
        # One unit of each model the unit module knows, so a model added
        # there and not built here fails.
        models = sorted(AFC_BambuAMS._AMS_MODELS)
        printer = make_printer()
        make_bridgebox(tmp_path, printer=printer, roster=", ".join(
            f"{model}:{n:04X}" for n, model in enumerate(models, 1)))
        units = {s.split(" ", 1)[1]: dict(w.fileconfig.items(s))
                 for s, w in printer.loaded
                 if s.startswith("AFC_BambuAMS ")}
        lanes = {name: [s for s, w in printer.loaded
                        if s.startswith("AFC_lane ")
                        and w.fileconfig.get(s, "unit").startswith(
                            f"{name}:")]
                 for name in units}
        built = {keys["ams_model"]: (len(lanes[name]), keys.get("heater"),
                                     keys.get("dry_max_temp"))
                 for name, keys in units.items()}
        assert set(built) == set(AFC_BambuAMS._AMS_MODELS)
        assert built == self.BUILT
        assert (printer.afc.logger.messages, printer.gcode.messages) == (
            [], [])
