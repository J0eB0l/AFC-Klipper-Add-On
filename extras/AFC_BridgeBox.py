# AFC BambuAMS chain master: one section that fabricates the rest.
#
# Copyright (C) 2026 J0eB0l
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# A hand-written BambuAMS setup is three boilerplate sections per unit
# ([AFC_BambuAMS <name>], an [AFC_lane] per slot, an [AFC_hub]); this module
# fabricates them from the roster and the chain's extruder/buffer, the way
# AFC_utils.add_filament_switch does, so other modules cannot tell them from
# printer.cfg text.
#
#   [AFC_BridgeBox chain1]
#   serial_port: /dev/serial/by-id/usb-Raspberry_Pi_Pico_XXXX-if00
#   extruder: extruder
#   buffer: Bamb_1
#   lane_base: 24
#   roster: ht:0123456789ABCDEF00003331
#
# The roster comes from config plus a persisted roster file, since sections
# must exist before the port is usable; pooled spares (pool_ams / pool_ht)
# cover units that appear live. An entry is `<model>:<unit_uid>`; a boxed unit
# of unknown generation enrolls as `boxed` and is refined later without its
# lanes moving.
#
# Names are pinned per uid and a unit's lanes follow its name's rank, so
# removing a unit never renumbers the survivors (lane names carry Spoolman
# bindings and T# macros) and a returning unit gets its lanes back. Learned
# values (bowden lengths) are saved per uid, not per bay.
from __future__ import annotations

import configparser
import copy
import json
import math
import os
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from extras.AFC_BambuAMS import (DRY_TEMP_HARD_MAX, _AMS_MODELS,
                                 _BUFFER_CHIP_NAME, _HT_MODELS)

#: Every option the chain master consumes. Anything else in the master's
#: section is a chain-wide unit default (see _chain_defaults); learned-value
#: folding must never rewrite these. A test checks __init__'s config.get* calls
#: against this set.
_MASTER_OPTIONS = frozenset({
    "serial_port", "tcp_key", "extruder", "unit_prefix", "removal_grace",
    "buffer_chip_name", "buffer", "buffer_type", "lane_base", "pool_ams",
    "pool_ht",
    "ams_names", "ht_names", "auto_drop", "release_grace", "release_settle",
    "enroll_grace", "claim_grace", "flap_claim_grace", "flap_window",
    "afc_bowden_length", "td1_bowden_length", "dry_max_temp", "auto_vars_file",
    "state_file", "roster", "hotplug_poll",
})

_STRUCTURAL_KEYS = frozenset(
    {"serial_port", "tcp_key", "unit_uid", "unit", "hub", "extruder", "buffer",
     "buffer_chip_name", "ams_model", "switch_pin", "sensor_type",
     "bambu_unit"})

#: What a unit learns about itself (its measured bowden lengths, see
#: persist_learned), saved per unit UID so they follow the physical unit.
_LEARNED_KEYS = ("afc_bowden_length", "afc_unload_bowden_length")

# Model facts come from AFC_BambuAMS's _AMS_MODELS: roster tags are exactly the
# ams_model values a unit accepts. `boxed` is a 0x0700 unit of unconfirmed
# generation; `lite` is reserved for the untested AMS Lite.

#: Lane count per roster model tag: an HT has one bay, every boxed unit four.
_SLOTS_BY_MODEL = {m: (1 if m in _HT_MODELS else 4) for m in _AMS_MODELS}

#: AMS bays a chain builds at most: a Bambu bus addresses at most 4 boxed
#: units. HT units are addressed separately (MAX_AMS_HT).
_MAX_AMS_BAYS = 4


def _norm_model(name: Optional[str]) -> str:
    """
    A model tag as written, trimmed and lower-cased.

    :param name: a model tag, possibly None
    :return str: the normalised tag (unknown names are left as they are)
    """
    return (name or "").strip().lower()


def _norm_uid(uid: Optional[str]) -> str:
    """
    A unit uid trimmed and upper-cased.

    :param uid: a unit uid, possibly None
    :return str: the normalised uid, "" for None
    """
    return (uid or "").strip().upper()


def _pool_laneno(pu: Dict[str, Any]) -> int:
    """
    Sort key: the first lane number of a pool unit.

    :param pu: pool unit record
    :return int: lane number, large when unnumbered
    """
    ds = "".join(c for c in (pu.get("lanes") or ["0"])[0] if c.isdigit())
    return int(ds) if ds else 1 << 30


def _learned_length(value: Any) -> Optional[float]:
    """
    A stored or configured bowden length, when it can be one.

    A unit reads its bowden lengths with getfloat(above=0), so a value laid
    into its section that is not a positive, finite number would stop
    Klipper starting, and one handed over at a claim would size every feed
    and retract.

    :param value: the value as written
    :return float: the length in mm, or None when it is not a positive, finite
        number
    """
    try:
        mm = float(str(value).strip())
    except ValueError:
        return None
    return mm if math.isfinite(mm) and mm > 0 else None

#: Models whose unit section carries heater: True (the HT and the AMS 2 Pro).
#: `boxed` gets no heater until its generation is confirmed: a heater key on
#: an AMS 1 invites commands the unit ignores.
_HEATED_MODELS = {m for m, spec in _AMS_MODELS.items() if spec[0]}

#: Drying ceiling per heated model, so the panel never commands a temperature
#: the hardware does not honour.
_DRY_CEILING = {m: _AMS_MODELS[m][3] for m in _HEATED_MODELS}


def _dry_ceiling(chain_max: int, model: str) -> int:
    """
    The drying ceiling a heated model gets from its chain.

    A chain's dry_max_temp lowers every heated unit's ceiling but never
    raises one past its model's own, so dry_max_temp: 80 leaves an AMS 2
    Pro at 65 and holds an HT to 80.

    :param chain_max: the chain's dry_max_temp, 0 when unset
    :param model: a heated model
    :return int: the ceiling in degrees C
    """
    own = _DRY_CEILING[model]
    return min(chain_max, own) if chain_max else own

#: Models whose fabricated section arms measure_on_insert. See the emission
#: site for why this is an allow-list rather than every model.
_HT_MODELS_BB = set(_HT_MODELS)


# ── pooled lanes ────────────────────────────────────────────────────────────
# Pool lanes are [AFC_lane]s with `unassigned: True`, kept out of every
# registry at connect; these move them in and out as units claim and release.

def activate_from_pool(lane: Any) -> None:
    """
    Register a pooled lane into the registries AFC_lane.handle_connect skipped.

    Mirrors the add_to_other_obj block in handle_connect. Always re-asserts
    the writes (they are idempotent) rather than skipping on the flag, so a
    re-claimed unit always gets its lanes back.

    :param lane: the pool lane being claimed
    """
    from extras.AFC_lane import EXCLUDE_TYPES
    lane.unassigned = False
    if not (lane.unit_obj.type not in EXCLUDE_TYPES
            or (lane.unit_obj.type in EXCLUDE_TYPES
                and "AFC_lane" in lane.fullname)):
        return
    lane.unit_obj.lanes[lane.name] = lane
    lane.afc.lanes[lane.name] = lane
    if lane.hub_obj is not None and not lane.is_direct_hub():
        try: lane.hub_obj.lanes[lane.name] = lane
        except Exception: pass
    if lane.extruder_obj is not None:
        lane.extruder_obj.lanes[lane.name] = lane
        try: lane.extruder_obj.check_lanes()
        except Exception: pass
    if lane.buffer_obj is not None:
        try: lane.buffer_obj.lanes[lane.name] = lane
        except Exception: pass
        # get_status publishes the name; without it the panel shows "-:-".
        if lane.buffer_name is None:
            lane.buffer_name = lane.buffer_obj.name
    # CHANGE_TOOL unloads first only for a PREP-marked target, and PREP skipped
    # this lane while it was pooled, so mark it here once PREP has run.
    if getattr(getattr(lane, "afc", None), "prep_done", False):
        try: lane.set_afc_prep_done()
        except Exception: pass


def _tcmd_handlers(afc: Any) -> Dict[str, Any]:
    """
    The g-code command handlers Klipper has registered.

    :param afc: the AFC object
    :return dict: command name -> handler
    """
    return getattr(getattr(afc, "gcode", None), "ready_gcode_handlers",
                   None) or {}


def _tcmd_is_ours(afc: Any, handler: Any) -> bool:
    """
    Whether a registered g-code handler is AFC's CHANGE_TOOL.

    :param afc: the AFC object
    :param handler: a registered handler, or None
    :return bool: True when ``handler`` is AFC's CHANGE_TOOL
    """
    change_tool = getattr(afc, "cmd_CHANGE_TOOL", None)
    # Compare the underlying function: cmd_CHANGE_TOOL is a bound method, and
    # each attribute access builds a new one, so `is` on it never matches.
    ours = getattr(change_tool, "__func__", change_tool)
    return (ours is not None
            and handler is not None
            and getattr(handler, "__func__", handler) is ours)


def _tcmd_holder(afc: Any, cmd: str) -> Optional[str]:
    """
    The lane that holds a T#: the one AFC's tool table names, while that
    lane is in afc.lanes and its map still lists the T#.

    An entry naming a lane that is gone from afc.lanes, or whose map no
    longer lists the T#, is left over and holds nothing.

    :param afc: the AFC object
    :param cmd: the T# command
    :return str: the holding lane's name, or None
    """
    owner = (getattr(afc, "tool_cmds", None) or {}).get(cmd)
    lane = (getattr(afc, "lanes", None) or {}).get(owner) if owner else None
    if lane is None or cmd not in (getattr(lane, "map", None) or []):
        return None
    return owner


def _tcmd_free(afc: Any, lname: str, cmd: str) -> bool:
    """
    Whether lane ``lname`` may register ``cmd`` without taking it from
    anything.

    Another lane that holds it (see _tcmd_holder) keeps it. A macro other
    than CHANGE_TOOL keeps it too, unless force_assign_map lets TcmdAssign
    rename that macro out of the way, as PREP's TcmdAssign does.

    :param afc: the AFC object
    :param lname: the lane that wants the T#
    :param cmd: the T# command
    :return bool: True when the T# is free for this lane
    """
    holder = _tcmd_holder(afc, cmd)
    if holder is not None and holder != lname:
        return False
    handler = _tcmd_handlers(afc).get(cmd)
    if handler is None or _tcmd_is_ours(afc, handler):
        return True
    return bool(getattr(afc, "force_assign_map", False))


def _parse_map(value: Any) -> List[str]:
    """
    A saved lane map as a list of T# commands.

    Read the way PREP reads the var file's map: a comma-separated string
    ("T3, T40", the form save_vars writes) or a list. NONE, the placeholder
    save_vars writes for an empty map, and repeats are dropped.

    :param value: the saved map
    :return list: the T# commands, in saved order
    """
    if isinstance(value, str):
        items: List[Any] = value.split(",")
    elif isinstance(value, (list, tuple)):
        items = list(value)
    else:
        items = []
    out: List[str] = []
    for item in items:
        cmd = str(item).strip()
        if cmd and cmd.upper() != "NONE" and cmd not in out:
            out.append(cmd)
    return out


def _home_tool(lname: Optional[str]) -> Optional[str]:
    """
    The home T# of a lane, from its lane number.

    :param lname: a lane name
    :return str: T<lane number>, or None for a lane with no number
    """
    ds = "".join(c for c in (lname or "") if c.isdigit())
    return "T" + ds if ds else None


def _plan_lane_map(afc: Any, lname: str, home: str, rec: Dict[str, Any],
                   warn: Callable[[str], None],
                   note: Optional[Callable[[str], None]] = None,
                   bambu: Any = ()) -> Tuple[List[str], str, bool]:
    """
    The T# map a claimed Bambu lane comes back with.

    A lane with no saved record, or one with no map in it, gets its home
    T<lane number>, taken from whichever lane holds it (see
    _take_home_tool). A saved map is the lane's own, set by SET_MAP and the
    other mapping commands, and comes back as any AFC lane's map does across
    a restart. Its home T# is taken back only when the map lists it, and any
    other T# only while no other lane, and no other macro, holds it: the
    record can be older than the live state, so a T# another lane or macro
    holds at the claim stays where it is and is dropped from the map with a
    warning. A map saved as NONE (its last T# removed, or moved to another
    lane with multiple mapping) comes back as NONE, as PREP restores an AFC
    lane's NONE, and SET_MAP gives it a T#. A map left with nothing because
    every saved T# is held, or saved blank, falls back to the home T# while
    that is free, and to NONE when it is not.

    A saved T# that is the home tool of the Bambu lane holding it, dropped
    while the lane keeps or falls back to its own home tool, is the home
    winning as it does at every claim, and needs nothing from the user: that
    line goes to ``note`` (AFC.log) instead of ``warn``.

    :param afc: the AFC object
    :param lname: the lane's name
    :param home: its home T#, T<lane number>
    :param rec: its saved record ({} when there is none)
    :param warn: called with each warning line
    :param note: called with each line that needs no action; None warns
    :param bambu: the names of the Bambu lanes whose home tool is theirs
    :return tuple: (map, current_map, take_home); take_home is True when
        the home T# is to be taken from a lane that holds it
    """
    if not rec or "map" not in rec:
        return [home], home, True

    def _who(cmd: str) -> str:
        """
        Name what holds a T# that is not free.

        :param cmd: a T# that is not free
        :return str: the lane holding it, or "a macro"
        """
        return _tcmd_holder(afc, cmd) or "a macro"

    home_free = _tcmd_free(afc, lname, home)
    saved = _parse_map(rec.get("map"))
    if not saved and rec.get("map"):
        return ["NONE"], "", False
    if not saved:
        return ([home], home, False) if home_free else (["NONE"], "", False)
    kept: List[str] = []
    dropped: List[str] = []
    for cmd in saved:
        if cmd == home or _tcmd_free(afc, lname, cmd):
            kept.append(cmd)
        else:
            dropped.append(cmd)
    back = f"; {lname} is back on {home}" if not kept and home_free else ""
    for cmd in dropped:
        line = (f"{lname}: saved {cmd} is held by {_who(cmd)} -- not "
                f"restored{back}")
        holder = _tcmd_holder(afc, cmd)
        if ((back or home in kept)
            and note is not None
            and holder in bambu
            and _home_tool(holder) == cmd):
            note(line)
        else:
            warn(line)
    if not kept:
        if home_free:
            return [home], home, False
        warn(f"{lname} has no T# -- its home {home} is held by {_who(home)}; "
             f"use SET_MAP")
        return ["NONE"], "", False
    current = rec.get("current_map")
    return kept, (current if current in kept else kept[0]), home in kept


def assign_pool_tcmd(lane: Any, afc: Any = None) -> None:
    """
    Give a claimed pool lane its T# command without registering it twice.

    A lane claimed before PREP, or back on a release -> re-claim, already has
    its T# registered to AFC's CHANGE_TOOL. TcmdAssign would register it again,
    Klipper refuses ("already setup"), and register_tool_macro logs that as a
    mapping conflict. When every T# on the lane is already ours, only the
    lookup table TcmdAssign would have filled is updated; anything else goes
    through TcmdAssign as normal.

    :param lane: the claimed pool lane
    :param afc: the AFC object, or None to use the lane's own
    """
    afc = afc if afc is not None else lane.afc
    handlers = _tcmd_handlers(afc)
    maps = [m for m in (getattr(lane, "map", None) or []) if m != "NONE"]
    if maps and all(_tcmd_is_ours(afc, handlers.get(m)) for m in maps):
        for m in maps:
            afc.tool_cmds[m] = lane.name
        if not lane.current_map:
            lane.current_map = maps[0]
        return
    afc.function.TcmdAssign(lane)


def deactivate_to_pool(lane: Any) -> None:
    """
    Reverse activate_from_pool: return the lane to the inert pool state.

    Drops the lane from every registry and clears the map and spool, with none
    of the spool's details left: no temperatures, variant, name, vendor, SKU,
    runout lane or TD-1 data, and the density, diameter and empty-spool weight
    the lane is configured with. What the lane held is kept for its unit's next
    claim of the bay (see afcBridgeBox._bay_records), so nothing on the lane
    passes to the next unit that claims it. The mirror of release on the
    AFC_BridgeBox side.

    :param lane: the pool lane being released
    """
    for reg in (getattr(lane.unit_obj, "lanes", None),
                lane.afc.lanes,
                getattr(lane.hub_obj, "lanes", None),
                getattr(lane.extruder_obj, "lanes", None),
                getattr(lane.buffer_obj, "lanes", None)):
        try:
            if isinstance(reg, dict):
                reg.pop(lane.name, None)
        except Exception:
            pass
    lane.map = []
    lane._map = []
    lane.current_map = ""
    lane.spool_id = None
    lane._material = None
    lane.color = ""
    lane.weight = 0.
    for attr, blank in (("multi_color", []), ("sub_type", ""),
                        ("spool_vendor", ""), ("filament_name", ""),
                        ("bambu_sku", ""),
                        ("extruder_temp", None), ("bed_temp", None),
                        ("runout_lane", None), ("td1_data", {}),
                        ("need_purge", False)):
        try:
            setattr(lane, attr, blank)
        except Exception:
            pass
    # The tare, density and diameter are the spool's too (a Spoolman link or
    # a record sets them): back to what the lane's config gives it, as
    # AFC_lane reads them at startup.
    cfg = getattr(lane, "_config", None)
    for key, dflt in (("empty_spool_weight", 190.), ("filament_density", 1.24),
                      ("filament_diameter", 1.75)):
        try:
            if cfg is not None:
                setattr(lane, key, cfg.getfloat(key, dflt))
        except Exception:
            pass
    # _load_state, not load_state: the public name is a read-only property,
    # and an AttributeError here would be swallowed by the caller and skip
    # the `unassigned = True` below.
    lane._load_state = False
    lane.unassigned = True


def lane_in_toolhead(lane: Any, afc: Any = None) -> bool:
    """
    Whether AFC records ``lane`` as loaded to a toolhead.

    AFC keeps that record in two places, the lane's tool_loaded and the
    extruder's lane_loaded, and they can disagree: PREP restores lane_loaded
    from the var file, while a pool lane's tool_loaded is only repaired from
    it once its unit is claimed. Either one counts.

    :param lane: the lane to check
    :param afc: the AFC object; defaults to the lane's
    :return bool: True if the lane or any extruder says it is in a toolhead
    """
    if getattr(lane, "tool_loaded", False):
        return True
    afc = afc if afc is not None else getattr(lane, "afc", None)
    tools = getattr(afc, "tools", None) or {}
    return any(getattr(e, "lane_loaded", None) == lane.name
               for e in tools.values())


def unset_tool_loaded(lane: Any, afc: Any = None) -> bool:
    """
    Clear AFC's record of ``lane`` as loaded to a toolhead, ahead of pooling
    it.

    A pooled lane is gone from afc.lanes, so a record left behind names a
    lane AFC can no longer unload or select, and save_vars writes it out as
    the extruder's lane_loaded for the next boot. The active toolhead's lane
    goes through unset_lane_loaded, the UNSET_LANE_LOADED path, which also
    drops the toolchange bookkeeping and re-activates the extruder; a lane in
    any other toolhead gets the lane-level half of it (unsync_to_extruder,
    set_tool_unloaded). loaded_to_hub drops with tool_loaded, as in AFC's
    lane reset. The fields are cleared again at the end, so a failing LED or
    Spoolman call cannot leave the record half-cleared.

    The caller saves the vars once the lane is pooled.

    :param lane: the lane about to be pooled
    :param afc: the AFC object; defaults to the lane's
    :return bool: True if there was a record to clear
    """
    afc = afc if afc is not None else getattr(lane, "afc", None)
    if not lane_in_toolhead(lane, afc):
        return False
    try:
        current = afc.function.get_current_lane() == lane.name
    except Exception:
        current = False
    try:
        if current:
            afc.function.unset_lane_loaded()
        else:
            lane.unsync_to_extruder()
            ext = lane.extruder_obj
            # set_tool_unloaded() empties the lane's own extruder outright;
            # leave one that names a different lane alone.
            if (ext is not None
                and getattr(ext, "lane_loaded", None) in (None, lane.name)):
                lane.set_tool_unloaded()
    except Exception:
        pass
    lane.tool_loaded = False
    lane.loaded_to_hub = False
    for ext in (getattr(afc, "tools", None) or {}).values():
        if getattr(ext, "lane_loaded", None) == lane.name:
            ext.lane_loaded = None
    return True


class _ChildCommand:
    """
    A running command's gcmd with parameters of its own, for handing to
    another command's handler.

    The parameter getters read only the parameters given here; everything
    else (error, respond_info, respond_raw) is the parent's, so the other
    handler's refusals and console lines reach the operator unchanged.
    """

    def __init__(self, parent: Any, **params: Any) -> None:
        """
        Wrap the running command with the parameters another handler should see.

        :param parent: the running command's gcmd
        :param params: the parameters the other handler reads
        """
        self._parent = parent
        self._params = {k: str(v) for k, v in params.items()}

    def get(self, name: str, default: Any = None, parser: Any = str,
            **_kw: Any) -> Any:
        """
        Read a parameter, as gcmd.get does.

        :param name: parameter name
        :param default: returned when the parameter is not given
        :param parser: converts the given value
        :param _kw: gcmd.get's range options, ignored
        :return object: the parsed value, or ``default``
        """
        value = self._params.get(name)
        return default if value is None else parser(value)

    def get_int(self, name: str, default: Any = None, **kw: Any) -> Any:
        """
        Read a parameter as an int, as gcmd.get_int does.

        :param name: parameter name
        :param default: returned when the parameter is not given
        :param kw: gcmd.get_int's range options, passed on and ignored
        :return object: the value as an int, or ``default``
        """
        return self.get(name, default, parser=int, **kw)

    def get_float(self, name: str, default: Any = None, **kw: Any) -> Any:
        """
        Read a parameter as a float, as gcmd.get_float does.

        :param name: parameter name
        :param default: returned when the parameter is not given
        :param kw: gcmd.get_float's range options, passed on and ignored
        :return object: the value as a float, or ``default``
        """
        return self.get(name, default, parser=float, **kw)

    def get_command_parameters(self) -> Dict[str, str]:
        """
        The parameters this command was given.

        :return dict: the parameters given here
        """
        return dict(self._params)

    def __getattr__(self, attr: str) -> Any:
        """
        Fall back to the running command for anything else.

        :param attr: any other gcmd attribute
        :return object: the parent's attribute
        """
        return getattr(self._parent, attr)


class _HeldBaysQueue:
    """
    AFC's var-file write queue, as a chain master sees it.

    AFC.save_vars writes each unit's registered lanes and hands the snapshot
    to this queue for its background writer, so a pool bay no unit is
    claimed onto is saved empty. Each snapshot passes through the master
    first, which writes the lane records it holds for such a bay, and the
    planned T# map of a lane whose take waits for the print (see
    afcBridgeBox._fill_held_bays); everything else reaches AFC's queue as
    it was.
    """

    def __init__(self, inner: Any, fill: Callable[[Dict[str, Any]], None]
                 ) -> None:
        """
        Wrap AFC's writer queue so each snapshot is filled in before it is queued.

        :param inner: the queue AFC's writer reads
        :param fill: called with each snapshot before it is queued
        """
        self._inner = inner
        self._fill = fill

    def _pass(self, item: Any) -> None:
        """
        Fill in a snapshot before it reaches the queue.

        :param item: a snapshot, or the writer's stop sentinel
        """
        if isinstance(item, dict):
            try:
                self._fill(item)
            except Exception:
                pass

    def put_nowait(self, item: Any) -> Any:
        """
        Fill in and queue an item without blocking.

        :param item: what AFC queues for its writer
        :return object: what AFC's queue returns
        """
        self._pass(item)
        return self._inner.put_nowait(item)

    def put(self, item: Any, *args: Any, **kwargs: Any) -> Any:
        """
        Fill in and queue an item.

        :param item: what AFC queues for its writer
        :param args: queue.put's positional options
        :param kwargs: queue.put's keyword options
        :return object: what AFC's queue returns
        """
        self._pass(item)
        return self._inner.put(item, *args, **kwargs)

    def __getattr__(self, attr: str) -> Any:
        """
        Fall back to AFC's queue for anything else.

        :param attr: any other queue attribute (the writer's get)
        :return object: AFC's queue's attribute
        """
        inner = self.__dict__.get("_inner")
        if inner is None:
            raise AttributeError(attr)
        return getattr(inner, attr)


#: The attribute on AFC's spool object naming the chain masters its wrapped
#: _reset_mapping serves (see _hook_reset_mapping).
_RESET_MASTERS = "_bridgebox_reset_masters"


def _hook_reset_mapping(spool: Any, master: Any) -> bool:
    """
    Have AFC's mapping reset put every claimed Bambu lane on its home T#.

    AFC_spool._reset_mapping, which AFC_RESET_MAPPING and
    AFC_ENABLE_MULTIPLE_MAPPING ENABLE=0 run, gives each lane with a config
    map: that T# and numbers every other lane from T0 up in unit order. A
    pool lane has no map: (a claim gives it its home T#, see
    afcBridgeBox._map_claimed_lanes), so the reset would number it like any
    other lane: an HT on lane28 behind sixteen lanes comes out T16, and its
    saved map keeps T16 across restarts. The wrapped reset gives each
    claimed Bambu lane its home T# as its map: for the call only (see
    afcBridgeBox._reset_home_plan), so AFC numbers every other lane around
    it, unregisters what no lane has any more, and saves, as it does around
    a config map:.

    The wrap is set on the spool object, not on its class: a Klipper RESTART
    builds a new spool object and keeps this module, so a wrapped class would
    be wrapped again at every restart. One wrap serves every chain master on
    the printer: a second install (another chain, or the same chain's ready
    again) adds its master, and a master of the same name replaces the one
    before it.

    :param spool: AFC's AFC_spool object
    :param master: the chain master whose lanes the reset puts home
    :return bool: True when the reset is wrapped, False when there is no
        _reset_mapping to wrap
    """
    orig = getattr(spool, "_reset_mapping", None)
    if not callable(orig):
        return False
    masters = getattr(spool, _RESET_MASTERS, None)
    if not isinstance(masters, dict):
        masters = {}

        def _reset_mapping(*args: Any, **kwargs: Any) -> Any:
            """
            AFC's _reset_mapping, with each claimed Bambu lane on its home T#.

            :param args: the reset's arguments
            :param kwargs: the reset's keyword arguments
            :return object: what AFC's reset returns
            """
            return _reset_home(spool, orig, masters, args, kwargs)

        _reset_mapping.__wrapped__ = orig  # type: ignore[attr-defined]
        spool._reset_mapping = _reset_mapping
        setattr(spool, _RESET_MASTERS, masters)
    masters[getattr(master, "name", "")] = master
    return True


def _reset_home(spool: Any, orig: Callable[..., Any],
                masters: Dict[str, Any], args: Tuple[Any, ...],
                kwargs: Dict[str, Any]) -> Any:
    """
    Run AFC's _reset_mapping with each claimed Bambu lane's home T# as its
    map: (see _hook_reset_mapping).

    Every lane's map: is back to what it was when the call returns or
    raises. A master whose plan fails leaves its lanes to AFC's numbering.

    :param spool: AFC's AFC_spool object
    :param orig: its own _reset_mapping
    :param masters: chain master name -> master
    :param args: the reset's arguments
    :param kwargs: the reset's keyword arguments
    :return object: what AFC's reset returns
    """
    afc = getattr(spool, "afc", None)
    plans: List[Tuple[Any, Dict[str, Any]]] = []
    for master in list(masters.values()):
        try:
            plans.append((master, master._reset_home_plan(afc)))
        except Exception:
            pass
    given: List[Tuple[Any, Any]] = []
    ok = False
    try:
        for _master, plan in plans:
            for lane, home in plan["home"]:
                given.append((lane, lane._map))
                lane._map = [home]
        result = orig(*args, **kwargs)
        ok = True
        return result
    finally:
        for lane, was in reversed(given):
            lane._map = was
        for master, plan in plans:
            try:
                master._reset_home_done(plan, ok)
            except Exception:
                pass


class _SweptSection:
    """
    Stands in for a leftover auto_vars section klippy parsed at this start
    (see _sweep_orphans). Registered under the section's name, it keeps
    klippy from building an object from a section no chain fabricates now.
    """


class afcBridgeBox:
    """
    The chain master: parses the roster and fabricates unit sections.
    """

    def __init__(self, config: Any) -> None:
        """
        Set up the chain master: transport, roster, pool units and commands.

        :param config: the [AFC_BridgeBox <name>] section
        """
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.serial_port = config.get("serial_port")
        # Passed to fabricated units alongside serial_port: the key
        # belongs to the link, so every unit on one bridge shares it.
        self.tcp_key = config.get("tcp_key", None)
        self.extruder = config.get("extruder")
        # The chain's buffer: name an existing [AFC_buffer], or leave unset and
        # the master fabricates an FPS_PSF on the bridge's virtual ADC chip.
        self.unit_prefix = config.get("unit_prefix", "Bambu_AMS")
        # Seconds a rostered unit must be continuously absent from a live chain
        # before its removal is recorded (applied at restart). 0 disables
        # auto-removal.
        self.removal_grace = config.getfloat("removal_grace", 120.0,
                                             minval=0.0)
        # uid -> when its absence began: a rostered unit's, for removal; with
        # a pool, which never auto-removes, a unit a bay is held for (see
        # _track_absence).
        self._missing_since: Dict[str, float] = {}
        # UIDs forgotten while still online; the scout skips them until they
        # are physically pulled, so a live FORGET is not undone by
        # re-enrollment.
        self._forget_suppressed: set = set()
        # UIDs already told they have no bay (see _no_bay_message), so the line
        # is said once per wait.
        self._no_bay_told: Set[str] = set()
        # Model tag of each on-wire uid that found no free bay, and the waiting
        # uids already offered an offline unit's bay (see _offer_replace).
        self._no_bay: Dict[str, str] = {}
        self._replace_offered: Set[str] = set()
        # Lane records kept for the unit last claimed onto each pool bay,
        # {bay: {"uid", "lanes": {lane: record}}}, and which unit that is,
        # {bay: uid} (persisted as bay_owner). See _capture_boot_records.
        self._held: Dict[str, Dict[str, Any]] = {}
        self._bay_owner: Optional[Dict[str, str]] = None
        # A bay_owner a claim recorded before PREP had run, not yet written
        # (see _persist_bay_owner).
        self._bay_owner_pending = False
        # uid -> unit name as persisted before this boot named anything.
        self._pins_at_boot: Dict[str, str] = {}
        # Reactor time of klippy:ready, from which claims wait for PREP, and
        # how long they wait at most (see _scout_ready).
        self._ready_at: Optional[float] = None
        self._prep_wait = self._PREP_WAIT
        # The pin chip the chain's buffer reads. Chip names are printer-wide,
        # so only the first chain defaults to bambu_buffer; later ones append
        # their name.
        chip_set = config.get("buffer_chip_name", None)
        earlier = self._earlier_chains()
        self.buffer_chip_name = chip_set or (
            f"{_BUFFER_CHIP_NAME}_{self.name}" if earlier
            else _BUFFER_CHIP_NAME)
        # The fabricated buffer's type: "FPS_PSF" is the plain
        # tension-follower, "bambu" adds the odometer gate so the buffer can
        # stand in for a toolhead sensor (see AFCBambuBuffer). Ignored when an
        # [AFC_buffer] is adopted.
        self.buffer_type = config.get("buffer_type", "FPS_PSF")
        self.buffer = config.get("buffer", None)
        self._fabricate_buffer = self.buffer is None
        if self._fabricate_buffer:
            # Adopt a hand-written [AFC_buffer] already on this chain's chip: a
            # second section on the same pin halts klippy.
            adopted = self._find_chip_buffer(config)
            # When no chain above builds anything, bambu_buffer is only a scout
            # stub chip, so a buffer on it belongs to this chain.
            if (not adopted
                and not chip_set
                and earlier
                and not any(getattr(m, "_fabricated_names", None)
                            for m in earlier)):
                adopted = self._find_chip_buffer(config, _BUFFER_CHIP_NAME)
                if adopted:
                    self.buffer_chip_name = _BUFFER_CHIP_NAME
            if adopted:
                self.buffer = adopted
                self._fabricate_buffer = False
            else:
                self.buffer = f"{self.unit_prefix}_Buffer"
        # 0 = the next lane number after every declared lane, computed once and
        # remembered so it never shifts when the config grows. Precedence:
        # option > stored base > computed.
        self.lane_base = config.getint("lane_base", 0)
        self._lane_base_set = bool(self.lane_base)
        # Hot-enroll pool: spare units fabricated at boot (Klipper's object
        # graph is frozen after parse) that a UID on the chain claims live. At
        # most 4 AMS bays are built; _scout_ready reports a larger pool_ams.
        self._pool_ams_asked = config.getint("pool_ams", 4, minval=0)
        self.pool_ams = min(self._pool_ams_asked, _MAX_AMS_BAYS)
        self.pool_ht = config.getint("pool_ht", 8, minval=0)
        # Optional unit names by family and enrollment order (default Bambu_AMS_# /
        # Bambu_AMS_HT_#). List position sets lanes and T#; learned values follow the
        # uid. See _given_names.
        self.ams_names = [n.strip() for n in
                          (config.get("ams_names", "") or "").split(",")
                          if n.strip()]
        self.ht_names = [n.strip() for n in
                         (config.get("ht_names", "") or "").split(",")
                         if n.strip()]
        # Auto-drop: release a claimed unit whose online flag stays false for
        # release_grace. False keeps the lanes until restart, FORGET or
        # reassignment.
        self.auto_drop = config.getboolean("auto_drop", False)
        # Seconds a claimed unit's online flag must stay false before its lanes
        # drop. A brief online read does not reset the clock; see
        # release_settle.
        self.release_grace = config.getfloat("release_grace", 10.0, minval=2.0)
        # Seconds a bound unit must stay online before that cancels a pending
        # release, so a flapping phantom flag cannot. Keep it well under
        # release_grace.
        self.release_settle = config.getfloat("release_settle", 5.0, minval=0.0)
        # Seconds a new uid must stay online before it is recorded in the
        # roster, so a phantom flag cannot re-record a just-forgotten unit.
        self.enroll_grace = config.getfloat("enroll_grace", 15.0, minval=0.0)
        # Claim hysteresis: a unit normally claims as soon as it is online, but
        # within flap_window of a release it must stay online flap_claim_grace
        # first, so a flapping link cannot keep re-claiming.
        self.claim_grace = config.getfloat("claim_grace", 0.0, minval=0.0)
        self.flap_claim_grace = config.getfloat("flap_claim_grace", 15.0,
                                                minval=0.0)
        self.flap_window = config.getfloat("flap_window", 120.0, minval=0.0)
        # Chain watch poll interval, most of the hot-plug delay. Cheap (see
        # _scout_tick).
        self.hotplug_poll = config.getfloat("hotplug_poll", 1.0, minval=0.5)
        self.afc_bowden_length = config.getfloat("afc_bowden_length", 2100.0)
        self.td1_bowden_length = config.getfloat("td1_bowden_length", 850.0)
        # An int, emitted as one: AFC_BambuAMS reads it with getint. 0 uses
        # each heated model's own ceiling (_DRY_CEILING); otherwise the lower
        # of the two. A negative value is read as 0 and reported at ready.
        raw_dry = config.getint("dry_max_temp", 0)
        self.dry_max_temp = max(0, raw_dry)
        self._dry_note = (
            f"AFC_BridgeBox {self.name}: dry_max_temp is {raw_dry}, below 0, "
            f"so it is ignored and each heated unit dries up to its model's "
            f"own ceiling. Remove it or set a positive value to silence this."
            if raw_dry < 0 else "")
        # Prefix for fabricated unit names; set it for a second chain, or the
        # names collide. auto_vars_file is where AFC persists learned values
        # (see _fold_and_sweep).
        self.auto_vars_file = os.path.expanduser(config.get(
            "auto_vars_file",
            "~/printer_data/config/AFC/AFC_auto_vars.cfg"))
        # Everything this module persists lives in a #~# managed block in the
        # file that holds the master's section, found by grepping the config
        # root. state_file overrides it.
        sf = config.get("state_file", None)
        self.state_file = (os.path.expanduser(sf) if sf
                           else (self._locate_own_file()
                                 or os.path.expanduser(self._DEFAULT_STATE)))
        # The roster option wins, then the scouted roster file. Neither means
        # scout mode: fabricate nothing, ask the chain, write the file and say
        # "RESTART to enroll".
        self._register_commands()
        self._migrate_legacy_state()
        # Chain-wide unit defaults: any master option not consumed, at the
        # lowest precedence (master < model < unit, see _fold). Consumed so
        # Klipper accepts the section.
        self._chain_defaults: Dict[str, str] = {}
        try:
            for opt in config.get_prefix_options(""):
                if opt in _MASTER_OPTIONS or opt in _STRUCTURAL_KEYS:
                    continue
                self._chain_defaults[opt] = config.get(opt)
        except Exception:
            pass
        # Not logged here: self.logger is only assigned at connect, so it does
        # not exist yet in __init__. The fold reports what it applied.
        roster_raw = (config.get("roster", None) or "").strip()
        self._roster_source = "option"
        if not roster_raw:
            roster_raw = self._state_get(self._BASE_SECTION + " " + self.name,
                                         "roster") or ""
            self._roster_source = "file"
        if not roster_raw:
            self._roster_source = "scout"
            if not (self.pool_ams or self.pool_ht):
                # Bridge-only scout: no roster and no pool, so only watch the
                # chain.
                self.units: List[Dict[str, Any]] = []
                # An [AFC_buffer] on bambu_buffer:fps needs the pin chip a unit
                # registers, so register it here on a shim that reads "no
                # data", like an absent Pico.
                try:
                    self._register_scout_chip(config)
                except Exception:
                    pass
                # It fabricates nothing, but when it is the last chain in the
                # config it is the one that sweeps auto_vars (see
                # _sweep_orphans).
                self._fabricated_names: Set[str] = set()
                try:
                    autov = self._read_ini(self.auto_vars_file)
                    if (autov is not None
                        and self._sweep_orphans(config, autov)):
                        self._write_auto_vars(autov)
                except Exception:
                    pass
                self.printer.register_event_handler(
                    "klippy:ready", self._scout_ready)
                return
            # Pool scout: no roster, so fabricate the empty pool and let
            # plugged units claim named bays live. Detected uids are still
            # recorded for the next restart.

        if not self.lane_base:
            self.lane_base = self._resolve_lane_base(config)
        # uid -> (first lane, span) and uid -> name, kept after removal so survivors
        # never renumber and a returning unit gets its lanes back. Entries for
        # unrostered uids are tombstones (see _roster_sections).
        self._lane_map: Dict[str, Tuple[int, int]] = self._load_lane_map()
        self._name_map: Dict[str, str] = self._load_name_map()
        self._pins_at_boot = dict(self._name_map)
        self._lanes_at_boot = dict(self._lane_map)
        self._maps_dirty = False

        if self._roster_source == "scout":
            roster: List[Dict[str, Any]] = []      # pool scout: no known units
        else:
            try:
                roster = self._parse_roster(roster_raw)
            except ValueError as e:
                where = (f"state file {self.state_file}"
                         if self._roster_source == "file" else "roster:")
                error_str = f"[AFC_BridgeBox {self.name}] {where}: {e}"
                raise config.error(error_str)

        self.units: List[Dict[str, Any]] = roster
        # What the learned-value migration and fold have to report, logged
        # by _scout_ready once a logger exists: (is a warning, text).
        self._learned_notes: List[Tuple[bool, str]] = []
        names_before = dict(self._name_map)
        # Lanes a recorded HT may not keep (see _band_note).
        self._declared_lanes = self._declared_lane_numbers(config)
        self._later_lanes = self._later_chain_lanes(config)
        try:
            sections = self._roster_sections(roster)
        except ValueError as e:
            error_str = f"[AFC_BridgeBox {self.name}] {e}"
            raise config.error(error_str)
        self._flush_maps()           # persist any lanes/names just assigned
        self._migrate_name_learned(names_before)
        sections = self._fold_and_sweep(config, sections)

        # Collisions are refused, not resolved. A bay name clashing with
        # another AFC unit is refused before it causes a duplicate-lane crash.
        foreign = self._foreign_unit_names(config)
        for section, _keys in sections:
            if (section.startswith("AFC_BambuAMS ")
                and section.split(" ", 1)[1] in foreign):
                nm = section.split(" ", 1)[1]
                error_str = (
                    f"[AFC_BridgeBox {self.name}] pool bay name {nm!r} collides "
                    f"with an existing AFC unit of the same name -- rename that "
                    f"ams_names / ht_names entry to something unique. AFC "
                    f"identifies units by name, so a duplicate double-registers "
                    f"that unit's lanes and stops Klipper from starting.")
                raise config.error(error_str)
            existing = self.printer.lookup_object(section, None)
            if existing is not None:
                error_str = self._collision_error(section)
                raise config.error(error_str)

        fileconfig = configparser.RawConfigParser()
        for section, keys in sections:
            fileconfig.add_section(section)
            for k, v in keys.items():
                fileconfig.set(section, k, str(v))

        import configfile
        # Pass the live access-tracking dict, not {}: configfile.settings is
        # built from it, and Mainsail reads sensor_type there to show humidity.
        tracking = self._live_access_tracking()
        for section, _keys in sections:
            wrapper = configfile.ConfigWrapper(
                self.printer, fileconfig, tracking, section)
            self.printer.load_object(wrapper, section)
        # Registered after the fabrication loop so the units' ready handlers
        # run first and the first unit owns the bridge; a scout-created bridge
        # would leave every unit on the ownerless "sharing" path.
        self.printer.register_event_handler("klippy:ready", self._scout_ready)

    # ── pure parts, split out so tests can exercise them directly ───────────

    @staticmethod
    def _parse_roster(raw: str) -> List[Dict[str, Any]]:
        """
        `model:uid, model:uid, ...` -> [{"model", "uid"}], validated.

        :param raw: the roster option's text
        :return list: one dict per unit, in roster order
        """
        units: List[Dict[str, Any]] = []
        seen: set = set()
        for entry in [e.strip() for e in raw.split(",") if e.strip()]:
            model, sep, uid = entry.partition(":")
            model = model.strip().lower()
            uid = _norm_uid(uid)
            if not sep or not uid:
                error_str = f"roster entry {entry!r} is not <model>:<unit_uid>"
                raise ValueError(error_str)
            if model not in _SLOTS_BY_MODEL:
                error_str = (
                    f"roster entry {entry!r}: unknown model {model!r} "
                    f"(one of {sorted(_SLOTS_BY_MODEL)})")
                raise ValueError(error_str)
            if uid in seen:
                error_str = f"roster lists uid {uid} twice"
                raise ValueError(error_str)
            seen.add(uid)
            units.append({"model": model, "uid": uid})
        if not units:
            error_str = "roster is empty -- nothing to fabricate"
            raise ValueError(error_str)
        return units

    def _roster_sections(
            self, roster: List[Dict[str, Any]]
    ) -> List[Tuple[str, Dict[str, Any]]]:
        """
        Every section the roster implies, in load order.

        Unit first, then its lanes, then its hub, the same order a
        hand-written config uses. Cross-references (unit->hub, lane->unit)
        are names resolved at klippy:connect, so a unit may name a hub whose
        section loads after it.

        :param roster: parsed roster entries
        :return list: (section name, {key: value}) pairs
        """
        sections: List[Tuple[str, Dict[str, Any]]] = []
        # Every fabricated unit, rostered or spare, is a pool slot claimed
        # live.
        self._pool_units: List[Dict[str, Any]] = []
        # Names are numbered per family and pinned per uid by the name map;
        # only a new uid draws one. Fixed-band layout: AMS bays take four lanes
        # each from lane_base and the HT band follows, so lanes never depend on
        # who is online. Pass 1 lists the bays, 1b names them, 1c sets lanes.
        bays: List[Dict[str, Any]] = []
        for u in roster:
            slots = _SLOTS_BY_MODEL[u["model"]]
            fam = "ht" if u["model"] in _HT_MODELS_BB else "ams"
            bays.append({"family": fam, "slots": slots,
                         "uid": u["uid"], "model": u["model"], "spare": False})
        have_ams = sum(1 for b in bays if b["family"] == "ams")
        have_ht = sum(1 for b in bays if b["family"] == "ht")
        # Console notes on rostered units this layout renames, moves or leaves
        # without a bay, logged by _scout_ready once a logger exists.
        self._layout_notes: List[str] = []
        # Pass 1b: name each bay, fixing its rank. Rostered uids keep valid saved names
        # in roster order, the rest draw a free name, spares last. An AMS finding all
        # four names held gets no bay this boot. A tombstone name reserves nothing with
        # a pool; without one it is skipped unless that strands an AMS or moves HT lanes.
        rostered = {b["uid"] for b in bays}
        tombs = {nm for u, nm in self._name_map.items() if u not in rostered}
        pooled = bool(self.pool_ams or self.pool_ht)
        used: Dict[str, Optional[str]] = {}     # name -> uid wearing it
        for b in bays:
            nm = self._name_map.get(b["uid"])
            rank = self._rank_of(b["family"], nm) if nm else None
            if nm and rank is not None and nm not in used:
                b["name"], b["rank"] = nm, rank
                used[nm] = b["uid"]

        def _draw_name(family: str, skip: Set[str]) -> Optional[Tuple[str, int]]:
            """
            The first name in a family neither held nor skipped.

            :param family: unit family (HT or AMS)
            :param skip: further names to pass over
            :return tuple: (name, rank), None when every AMS rank is taken
            """
            i = 0
            while family == "ht" or i < _MAX_AMS_BAYS:
                nm = self._name_for_index(family, i)
                if nm not in used and nm not in skip:
                    return nm, i
                i += 1
            return None

        def _lanes_rank(b: Dict[str, Any], skip: Set[str]
                        ) -> Optional[Tuple[str, int]]:
            """
            The AMS name whose rank the unit's recorded lanes sit at, when
            nothing holds it.

            :param b: a rostered bay still to be named
            :param skip: names to pass over
            :return tuple: (name, rank); None for an HT, for a unit with no
                recorded lanes or lanes off the four AMS bays, and when the
                name is held or skipped
            """
            had = self._lane_map.get(b["uid"])
            if b["family"] != "ams" or not had:
                return None
            r, off = divmod(had[0] - self.lane_base, 4)
            if off or not 0 <= r < _MAX_AMS_BAYS:
                return None
            nm = self._name_for_index("ams", r)
            return None if nm in used or nm in skip else (nm, r)

        # The band the last start to reach ready built (see _scout_ready). With
        # none recorded, the recorded lanes stand.
        try:
            ready_band = int(self._state_get(
                self._BASE_SECTION + " " + self.name, "ams_band")
                             or "")
        except ValueError:
            ready_band = _MAX_AMS_BAYS

        def _ht_band(b: Dict[str, Any]) -> int:
            """
            The AMS bays a rostered HT's recorded lanes sit past: an HT of
            rank r sits on lane_base + band * 4 + r.

            :param b: a rostered HT bay, named
            :return int: that band, at most four and at most the band the
                last start built; 0 for an HT with no recorded lanes, one
                that drew a new name, or lanes that do not sit past whole
                AMS bays
            """
            had = self._lane_map.get(b["uid"])
            if not had or "was" in b:
                return 0
            n = int(had[0]) - int(self.lane_base) - int(b["rank"])
            if n < 0 or n % 4:
                return 0
            return min(n // 4, _MAX_AMS_BAYS, ready_band)

        # The AMS bays the roster fills, its named AMS already reach, or a
        # named HT's recorded lanes sit past: a new AMS drawn inside them
        # moves no HT lane.
        reach = max([min(have_ams, _MAX_AMS_BAYS)]
                    + [b["rank"] + 1 for b in bays
                       if b["family"] == "ams" and "name" in b]
                    + [_ht_band(b) for b in bays
                       if b["family"] == "ht" and "name" in b])
        # A unit with recorded lanes draws before a uid first seen, so a
        # long-standing unit keeps a bay ahead of a new one; roster order
        # decides the rest.
        for b in sorted((x for x in bays if "name" not in x),
                        key=lambda x: x["uid"] not in self._lane_map):
            skip = set() if pooled else tombs
            got = _lanes_rank(b, skip)
            if got is None:
                got = _draw_name(b["family"], skip)
                if (not pooled
                    and b["family"] == "ams"
                    and (got is None or (have_ht and got[1] >= reach))):
                    got = _draw_name(b["family"], set())
            saved = self._name_map.get(b["uid"])
            if saved:
                b["was"] = (saved, used.get(saved))
            b["name"], b["rank"] = got if got else (None, None)
            if got:
                used[got[0]] = b["uid"]
                self._name_map[b["uid"]] = got[0]
                self._maps_dirty = True
        # An AMS with no bay has no name or lanes until it gets one. Kept
        # with the name it had, for the texts that name it (see
        # _prompt_replace_unit).
        waiting = [b for b in bays if b["name"] is None]
        bays = [b for b in bays if b["name"] is not None]
        self._unbayed = {b["uid"]: (b["was"][0] if b.get("was") else None)
                         for b in waiting}
        for b in waiting:
            had_name = self._name_map.pop(b["uid"], None) is not None
            b["had"] = self._lane_map.pop(b["uid"], None)
            if had_name or b["had"]:
                self._maps_dirty = True
        for b in bays:
            if self._drop_name_holders(self._name_map, self._lane_map,
                                       b["name"], rostered):
                self._maps_dirty = True
        # Spares fill each family up to its pool size.
        built_ams = sum(1 for b in bays if b["family"] == "ams")
        spare_specs = ([("ams", 4)] * max(0, self.pool_ams - built_ams)
                       + [("ht", 1)] * max(0, self.pool_ht - have_ht))
        for fam, slots in spare_specs:
            got = _draw_name(fam, set())
            if got is None:
                continue
            bays.append({"family": fam, "slots": slots, "uid": None,
                         "model": "ht" if fam == "ht" else "boxed",
                         "spare": True, "name": got[0], "rank": got[1]})
            used[got[0]] = None
        # Pass 1c: AMS rank r -> lane_base + r*4, HT rank r -> ht_base + r. The AMS
        # band covers every built AMS bay and recorded HT lanes, so lowering pool_ams
        # does not move a recorded HT (see _band_note).
        need = max([self.pool_ams]
                   + [b["rank"] + 1 for b in bays if b["family"] == "ams"])
        held = [b for b in bays if b["family"] == "ht"
                                   and not b["spare"]
                                   and _ht_band(b) > need]
        band = max([need] + [_ht_band(b) for b in held])
        if held:
            taken = getattr(self, "_declared_lanes", None) or set()
            later = {n: c for n, c in
                     (getattr(self, "_later_lanes", None) or {}).items()
                     if n not in taken}
            clash = sorted(n for n in (self.lane_base + band * 4 + b["rank"]
                                       for b in bays if b["family"] == "ht")
                           if n in taken or n in later)
            self._layout_notes.append(
                self._band_note(held, band, need, clash, later))
            if clash:
                band = need
        self._ams_band = band
        ht_base = self.lane_base + band * 4
        for b in bays:
            if b["family"] == "ht":
                b["lane"] = ht_base + b["rank"]
            else:
                b["lane"] = self.lane_base + b["rank"] * 4
            if not b["spare"]:
                b["had"] = self._lane_map.get(b["uid"])
                self._lane_map[b["uid"]] = (b["lane"], b["slots"])
                self._maps_dirty = True
        self._lane_moves = self._moved_lanes(bays, waiting)
        for b in bays:
            note = self._layout_note(b)
            if note:
                self._layout_notes.append(note)
        # A bay built under neither its entry nor its default says why.
        self._name_notes = [
            f"AFC_BridgeBox {self.name}: {note}"
            for note in (self._name_note(b["family"], b["rank"])
                         for b in sorted(bays, key=lambda x: x["lane"]))
            if note]
        # The ready note tells a waiting AMS once; the live watch stays quiet
        # about it until it goes offline and returns.
        holders = [(x["name"], x["uid"])
                   for x in sorted(bays, key=lambda x: x["lane"])
                   if x["family"] == "ams" and not x["spare"]]
        for b in waiting:
            self._layout_notes.append(
                f"AFC_BridgeBox {self.name}: AMS {b['uid']} is recorded but "
                f"has no bay: "
                + self._all_ams_bays_held(b["uid"], holders, live=False)
                + (f" Its record as {b['was'][0]} is dropped."
                   if b.get("was") else ""))
            self._no_bay_told.add(b["uid"])
        # Pass 1d: no lane may belong to two bays; refused here with both
        # named.
        by_lane = sorted(bays, key=lambda x: x["lane"])
        for i, a in enumerate(by_lane):
            for b in by_lane[i + 1:]:
                if b["lane"] >= a["lane"] + a["slots"]:
                    break
                error_str = self._overlap_error(a, b)
                raise ValueError(error_str)
        # Pass 2: fabricate the sections in ascending lane order.
        for b in sorted(bays, key=lambda x: x["lane"]):
            name, lane_no, slots = b["name"], b["lane"], b["slots"]
            unit_keys: Dict[str, Any] = {
                "serial_port": self.serial_port,
                "ams_model": b["model"],
                "extruder": self.extruder,
                "hub": name,
                "auto_error_recovery": True,
                # A known HT measures on insert; a boxed unit does not (it can
                # wedge an AMS 2 Pro mid-auth). Spares wait for a claim.
                # Overridable per model.
                "measure_on_insert": (not b["spare"]
                                      and b["model"] in _HT_MODELS_BB),
                "buffer": self.buffer,
                # The unit registers the chain's buffer pin chip under this
                # name, which the fabricated buffer's adc_pin reads.
                "buffer_chip_name": self.buffer_chip_name,
                # Fabricated inert, known and spare alike, and claimed live by
                # uid.
                "pool": True,
            }
            if not b["spare"]:
                unit_keys["unit_uid"] = b["uid"]
            # Written only when set: the section takes str() of each value, and
            # a "None" key reads as a configured one.
            if self.tcp_key:
                unit_keys["tcp_key"] = self.tcp_key
            if b["model"] in _HEATED_MODELS:
                unit_keys["heater"] = True
                unit_keys["dry_max_temp"] = _dry_ceiling(self.dry_max_temp,
                                                         b["model"])
            sections.append((f"{self._UNIT_SECTION} {name}", unit_keys))
            for slot in range(slots):
                sections.append((f"AFC_lane lane{lane_no + slot}",
                                 {"unit": f"{name}:{slot + 1}",
                                  "unassigned": True}))
            sections.append((f"AFC_hub {name}", {
                "switch_pin": "virtual",
                "afc_bowden_length": self.afc_bowden_length,
                "afc_unload_bowden_length": self.afc_bowden_length,
                "td1_bowden_length": self.td1_bowden_length,
            }))
            # Known units get a Mainsail/Fluidd temperature card (humidity comes
            # off every AMS's motion-long reply). A spare gets none, so it shows
            # nothing until claimed.
            if not b["spare"]:
                sections.append((f"temperature_sensor {name}", {
                    "sensor_type": "aht2x",
                    "bambu_unit": name,
                    "min_temp": 0,
                    "max_temp": 90,
                }))
            self._pool_units.append({
                "name": name,
                "lanes": [f"lane{lane_no + slot}" for slot in range(slots)],
                "family": b["family"],
                "uid": b["uid"],
                "bound": None,
                "spare": b["spare"],
            })
        if self._fabricate_buffer:
            # Last on purpose: the buffer's adc_pin needs a unit's chip. Error
            # sensitivity 0 since the AMS meters its own moves.
            sections.append((f"AFC_buffer {self.buffer}", {
                "type": self.buffer_type,
                "adc_pin": f"{self.buffer_chip_name}:fps",
                "deadband": 0.48,
                "filament_error_sensitivity": 0,
            }))
        return sections

    _BASE_SECTION = "AFC_BridgeBox"
    #: Prefix of the sections this module fabricates for its units. Distinct
    #: from _BASE_SECTION, which names this module's own section and, in the
    #: state block, each unit's learned record (see _learned_section).
    _UNIT_SECTION = "AFC_BambuAMS"

    def _find_chip_buffer(self, config: Any, chip: Optional[str] = None
                          ) -> Optional[str]:
        """
        The name of an existing [AFC_buffer] on this chain's chip, if any.

        :param config: this section's wrapper (for the merged fileconfig)
        :param chip: another chip name to look for instead
        :return str: the buffer's name as written, or None
        """
        try:
            import re as _re
            fc = getattr(config, "fileconfig", None)
            for sec in (fc.sections() if fc else []):
                m = _re.match(r"(?i)afc_buffer\s+(\S+)$", sec)
                if not m:
                    continue
                pin = dict(fc.items(sec)).get("adc_pin", "")
                if pin.strip().lower().startswith(
                        (chip or self.buffer_chip_name).lower() + ":"):
                    return m.group(1)
        except Exception:
            pass
        return None

    def _override_sections(self, config: Any) -> Dict[str, Dict[str, str]]:
        """
        Operator overrides from serial_port-less [AFC_BridgeBox ...] sections.

        A section with serial_port is a chain master; one without is an
        override carrier. Three target grammars, most-specific wins:
        - a model tag ("ht", "ams1", "ams2", "boxed"; "lite" is reserved for
          the untested AMS Lite) applies to the unit section of every unit of
          that model on the chain;
        - a bare unit name ("Bambu_AMS_1") targets that unit's
          [AFC_BambuAMS] section;
        - a qualified name ("AFC_hub Bambu_AMS_1", "AFC_lane lane13")
          targets any fabricated section exactly.
        Targets nothing fabricates are ignored, so a removed unit's leftover
        override does not halt boot. The sections each target came from are
        kept in _override_written, so the warning for one that matches
        nothing names the section as the operator wrote it.

        :param config: this section's wrapper (for the merged fileconfig)
        :return dict: fabricated section name (or "model:<tag>") -> {k: v}
        """
        out: Dict[str, Dict[str, str]] = {}
        written: Dict[str, List[str]] = {}
        self._override_written = written
        try:
            import re as _re
            fc = getattr(config, "fileconfig", None)
            for sec in (fc.sections() if fc else []):
                m = _re.match(r"(?i)afc_bridgebox\s+(.+)$", sec)
                if not m:
                    continue
                kv = dict(fc.items(sec))
                if kv.get("serial_port"):
                    continue                  # a chain master, not an override
                target = m.group(1).strip()
                canon = _norm_model(target)
                if canon in _SLOTS_BY_MODEL:
                    target = "model:" + canon
                elif " " not in target:
                    target = f"AFC_BambuAMS {target}"
                out.setdefault(target, {}).update(kv)
                written.setdefault(target, []).append(sec)
        except Exception:
            pass
        return out

    def _resolve_lane_base(self, config: Any) -> int:
        """
        The auto lane base: stored if ever resolved, else computed.

        Computed = one past the highest laneN the parsed config declares
        under [AFC_lane] or [AFC_stepper], case-insensitively (the two
        section families lanes live in), or that a chain master above this
        one in the config has already fabricated (see _loaded_lane_numbers).

        When no numbered laneN sections exist (a machine whose toolhead lanes
        are named, e.g. [AFC_extruder e1] map: T1, not laneN), fall in behind
        the highest tool number (map: T<n>, or the number an [AFC_extruder eN]
        section carries) already assigned to any AFC lane / stepper /
        extruder instead of jumping to 24, so the fabricated pool
        continues the tool sequence with lane# == T# (T1/T2/T3 in use -> base 4
        -> lane4/T4). The 24 is only a last resort, when the config cannot be
        seen at all (no fileconfig) or nothing at all is numbered or mapped.

        :param config: this section's wrapper
        :return int: the base to allocate fabricated lanes from
        """
        stored = self._state_get(self._BASE_SECTION + " " + self.name,
                                 "lane_base")
        if stored:
            try:
                return int(stored)
            except Exception:
                pass
        base = 24
        try:
            import re as _re
            fc = getattr(config, "fileconfig", None)
            secs = list(fc.sections()) if fc else []
            # One past the highest laneN, declared here or fabricated by an
            # earlier chain.
            nums = [int(m.group(1)) for sec in secs
                    for m in [_re.match(r"(?i)afc_(?:lane|stepper)\s+lane(\d+)$",
                                        sec)] if m]
            nums += self._loaded_lane_numbers()
            if nums:
                base = max(nums) + 1
            else:
                # No numbered lanes: start after the highest T# in use so lane#
                # == T#.
                tnums = []
                for sec in secs:
                    if not _re.match(r"(?i)afc_(?:lane|stepper|extruder)\s+", sec):
                        continue
                    try:
                        mv = fc.get(sec, "map")
                    except Exception:
                        mv = ""
                    tnums += [int(t) for t in _re.findall(r"[Tt](\d+)", mv or "")]
                    # A toolchanger's tools are numbered by their extruder
                    # sections ([AFC_extruder e0] .. e3 are T0..T3) even when
                    # no map: is written, since the T# is assigned at runtime.
                    me = _re.match(r"(?i)afc_extruder\s+\D*(\d+)$", sec)
                    if me:
                        tnums.append(int(me.group(1)))
                if tnums:
                    base = max(tnums) + 1
        except Exception:
            pass
        self._resolved_base = base            # _fold_and_sweep persists it
        return base

    def _loaded_lane_numbers(self) -> List[int]:
        """
        The laneN numbers of every lane object the printer already holds.

        A chain master's lanes come from its private parser, so no fileconfig
        lists them, but load_object registers each one as a printer object
        before that master returns. A master later in the config finds them
        here, the same registry its collision check reads.

        :return list: lane numbers, empty when the printer cannot be asked
        """
        import re as _re
        nums: List[int] = []
        try:
            for family in ("AFC_lane", "AFC_stepper"):
                for name, obj in self.printer.lookup_objects(family):
                    if isinstance(obj, _SweptSection):
                        continue
                    m = _re.match(r"(?i)\S+\s+lane(\d+)$", name)
                    if m:
                        nums.append(int(m.group(1)))
        except Exception:
            pass
        return nums

    def _declared_lane_numbers(self, config: Any) -> Set[int]:
        """
        The laneN numbers that exist outside this chain: the [AFC_lane] /
        [AFC_stepper] sections of the config, case-insensitively, and the
        lanes the printer already holds (see _loaded_lane_numbers).

        A section auto_vars alone carries is left out: it is a learned value
        AFC filed under a fabricated lane's name (see _fold_and_sweep), not a
        lane of its own.

        :param config: this section's wrapper (for the merged fileconfig)
        :return set: lane numbers
        """
        import re as _re
        nums = set(self._loaded_lane_numbers())
        fc = getattr(config, "fileconfig", None)
        if fc is None:
            return nums
        autov = self._read_ini(self.auto_vars_file)
        try:
            for sec in fc.sections():
                m = _re.match(r"(?i)afc_(?:lane|stepper)\s+lane(\d+)$", sec)
                if not m:
                    continue
                if (autov is not None
                    and autov.has_section(sec)
                    and not set(dict(fc.items(sec)))
                    - set(dict(autov.items(sec)))):
                    continue
                nums.add(int(m.group(1)))
        except Exception:
            pass
        return nums

    def _earlier_chains(self) -> List["afcBridgeBox"]:
        """
        The chain masters loaded before this one.

        klippy loads sections in config order and registers each object when
        its load function returns, so the masters it already holds are the
        chains above this one. Override holders share the prefix and are
        skipped.

        :return list: earlier masters, empty when the printer cannot be asked
        """
        try:
            return [obj for _name, obj
                    in self.printer.lookup_objects(self._BASE_SECTION)
                    if isinstance(obj, afcBridgeBox)]
        except Exception:
            return []

    def _later_chains(self, config: Any) -> List[str]:
        """
        The chain masters the config declares below this one.

        A master's section carries serial_port; an override holder's does
        not. The masters loaded so far are this one and the earlier chains
        (see _earlier_chains), so any other is still to load.

        :param config: this section's wrapper (its fileconfig sees the merged
            user config)
        :return list: their chain names, empty when the config cannot say
        """
        fc = getattr(config, "fileconfig", None)
        if fc is None:
            return []
        loaded = {self.name} | {m.name for m in self._earlier_chains()}
        prefix = self._BASE_SECTION + " "
        try:
            return [sec.split()[-1] for sec in fc.sections()
                    if sec.startswith(prefix)
                       and sec.split()[-1] not in loaded
                       and dict(fc.items(sec)).get("serial_port")]
        except Exception:
            return []

    def _collision_error(self, section: str) -> str:
        """
        The refusal for a fabricated section whose name the printer already
        holds.

        When a chain above this one built it, the text names that chain and
        what makes the two meet: this chain's lane_base, and where it comes
        from, for a lane; buffer: for the buffer; the unit names otherwise.
        Any other holder is a section of the operator's config.

        :param section: the fabricated section name
        :return str: the error text
        """
        head = f"[AFC_BridgeBox {self.name}] would fabricate [{section}]"
        other = next((m.name for m in self._earlier_chains()
                      if section in (getattr(m, "_fabricated_names", None)
                                     or ())), None)
        if other is None:
            return head + " but it already exists in the config -- remove one"
        head += (f", but [AFC_BridgeBox {other}] above it in the config "
                 f"already builds it")
        kind = section.split(" ", 1)[0]
        if kind == "AFC_lane":
            if getattr(self, "_lane_base_set", False):
                where = "set by lane_base: in this section"
            elif hasattr(self, "_resolved_base"):
                where = "computed past the lanes loaded before this chain"
            else:
                where = (f"saved in the #~# block of {self.state_file}, "
                         f"used while lane_base: is unset or 0")
            return (head + f": this chain's lane_base {self.lane_base} "
                    f"({where}) falls inside that chain's lanes. Set "
                    f"lane_base: in one chain's section past the other "
                    f"chain's last lane.")
        if kind == "AFC_buffer":
            return head + ". Give one chain its own buffer: name."
        return (head + ". Give one chain its own unit_prefix, or its own "
                "ams_names / ht_names.")

    def _later_chain_lanes(self, config: Any) -> Dict[int, str]:
        """
        The lanes the chains below this one in the config build, as far as
        their sections and saved state say before they load.

        A later chain whose lane_base is set, or saved in its state, builds
        its AMS band there (its pool_ams, rostered AMS or last band, at most
        four bays) with its HT lanes above, and every unit its lane_map
        records. One whose base is still to be computed starts past every
        lane loaded before it, so it is left out.

        :param config: this section's wrapper
        :return dict: lane number -> the name of the later chain building it
        """
        fc = getattr(config, "fileconfig", None)
        out: Dict[int, str] = {}
        if fc is None:
            return out
        for name in self._later_chains(config):
            sec = self._BASE_SECTION + " " + name
            try:
                opts = dict(fc.items(sec))
                path = opts.get("state_file")
                path = (os.path.expanduser(path) if path
                        else self._locate_own_file(name)
                        or os.path.expanduser(self._DEFAULT_STATE))
                cp = self._read_state(path)
                saved = dict(cp.items(sec)) if cp.has_section(sec) else {}
                base = (int(opts.get("lane_base") or 0)
                        or int(saved.get("lane_base") or 0))
                if base <= 0:
                    continue
                tags = [e.split(":")[0].strip().lower() for e in
                        (opts.get("roster")
                         or saved.get("roster")
                         or "").split(",") if ":" in e]
                n_ht = sum(1 for t in tags if t in _HT_MODELS)
                band = min(max(int(opts.get("pool_ams", 4)),
                               len(tags) - n_ht,
                               int(saved.get("ams_band") or 0)),
                           _MAX_AMS_BAYS)
                lanes = set(range(base, base + band * 4
                                  + max(int(opts.get("pool_ht", 8)), n_ht)))
            except Exception:
                continue
            for entry in (saved.get("lane_map") or "").split(","):
                try:
                    _uid, start, count = entry.split(":")
                    first, span = int(start), int(count)
                except Exception:
                    continue
                if span in _SLOTS_BY_MODEL.values() and first >= 0:
                    lanes |= set(range(first, first + span))
            for n in lanes:
                out.setdefault(n, name)
        return out

    def _foreign_unit_names(self, config: Any) -> set:
        """
        Unit names that existing AFC_lane / AFC_stepper sections in the config
        already belong to (their `unit:` field, before the ':slot').

        A fabricated pool bay named the same as one of these collides: AFC keys
        its `AFC_unit_<name>:connect` event by the bare unit name, so a
        duplicate name makes that unit's lanes register their mux commands
        twice and Klipper fails to boot (`... LANE laneN already registered`).
        BridgeBox's own fabricated lanes live in a private fileconfig and are not
        in this one, so only foreign units appear here.

        :param config: this section's wrapper (its fileconfig sees the merged
            user config)
        :return set: foreign unit names
        """
        import re as _re
        fc = getattr(config, "fileconfig", None)
        names: set = set()
        if fc is None:
            return names
        for sec in fc.sections():
            if not _re.match(r"(?i)afc_(?:lane|stepper)\s+\S+$", sec):
                continue
            try:
                unit = dict(fc.items(sec)).get("unit", "") or ""
            except Exception:
                unit = ""
            unit = unit.split(":")[0].strip()
            if unit:
                names.add(unit)
        return names

    def _load_lane_map(self) -> Dict[str, Tuple[int, int]]:
        """
        The persisted uid -> (first lane, span) map, tombstones included.

        Serialized in the state block as `lane_map: UID:start:span, ...`.
        An entry outlives its unit's removal from the roster as a tombstone,
        which is what makes removal safe (see _prune_missing) and return
        cheap. Entries leave only by AFC_BRIDGEBOX_FORGET, by UNASSIGN, when
        a rostered unit takes over the tombstone's name (see _roster_sections
        and _persist_pin), which drops its name entry too, or when a rostered
        AMS finds all four AMS bays held at boot and so has no bay. An entry
        whose span is not a unit's lane count (1 or 4) or whose first lane
        is negative is dropped like any other bad entry.

        :return dict: uid -> (first lane number, lane count)
        """
        raw = self._state_get(self._BASE_SECTION + " " + self.name,
                              "lane_map") or ""
        spans = set(_SLOTS_BY_MODEL.values())
        out: Dict[str, Tuple[int, int]] = {}
        for entry in [e.strip() for e in raw.split(",") if e.strip()]:
            try:
                uid, start, span = entry.split(":")
                first, count = int(start), int(span)
            except Exception:
                continue                      # one bad entry loses one entry
            if count not in spans or first < 0:
                continue
            out[_norm_uid(uid)] = (first, count)
        return out

    def _load_name_map(self) -> Dict[str, str]:
        """
        The persisted uid -> unit name map: the lane map's twin, entries
        kept and dropped alongside it, serialized as `name_map: UID:Name,
        ...`.

        :return dict: uid -> fabricated unit name
        """
        raw = self._state_get(self._BASE_SECTION + " " + self.name,
                              "name_map") or ""
        out: Dict[str, str] = {}
        for entry in [e.strip() for e in raw.split(",") if e.strip()]:
            uid, sep, name = entry.partition(":")
            if sep and uid.strip() and name.strip():
                out[_norm_uid(uid)] = name.strip()
        return out

    def _load_bay_owner(self) -> Tuple[Dict[str, str], bool]:
        """
        The persisted bay -> uid map of which unit each pool bay was last
        claimed onto, serialized as `bay_owner: UID:Bay, ...`.

        A bay's lane records in AFC.var.unit are written only while a unit is
        claimed onto it, so this names whose records they are. It is not a
        pin: naming and lane assignment never read it.

        :return tuple: (bay -> uid, whether the key exists at all)
        """
        raw = self._state_get(self._BASE_SECTION + " " + self.name,
                              "bay_owner")
        out: Dict[str, str] = {}
        for entry in [e.strip() for e in (raw or "").split(",") if e.strip()]:
            uid, sep, bay = entry.partition(":")
            if sep and uid.strip() and bay.strip():
                out[bay.strip()] = _norm_uid(uid)
        return out, raw is not None

    def _owners(self) -> Dict[str, str]:
        """
        The bay owner map, read from the state the first time it is asked for.

        :return dict: bay -> uid of the unit last claimed onto it
        """
        owners = getattr(self, "_bay_owner", None)
        if owners is None:
            owners = self._bay_owner = self._load_bay_owner()[0]
        return owners

    def _ser_bay_owner(self, owners: Dict[str, str]) -> str:
        """
        Serialize the bay owner map for the state block.

        :param owners: bay -> uid
        :return str: the state-block value, entries in lane order
        """
        first = {pu["name"]: _pool_laneno(pu)
                 for pu in getattr(self, "_pool_units", None) or []}
        return ", ".join(f"{owners[bay]}:{bay}" for bay in sorted(
            owners, key=lambda b: (first.get(b, 1 << 30), b)))

    def _set_bay_owner(self, bay: str, uid: str) -> None:
        """
        Record that ``uid`` is claimed onto ``bay``, and persist it (see
        _persist_bay_owner).

        A unit sits on one bay at a time, so its entries for other bays go, and
        so do the lane records held for it there: the lanes it left there drop
        out of AFC.var.unit at the next save (see _fill_held_bays), and the
        spools its bays hold from here are recorded on this bay's lanes.

        :param bay: the pool bay's name
        :param uid: the claimed unit's uid
        """
        new = {b: u for b, u in self._owners().items() if u != uid}
        new[bay] = uid
        self._bay_owner = new
        self._held = {b: e for b, e in (getattr(self, "_held", None)
                                        or {}).items()
                      if b == bay or _norm_uid(e.get("uid")) != uid}
        self._persist_bay_owner()

    def _persist_bay_owner(self) -> None:
        """
        Write bay_owner to the state block when it differs from what is there.

        Only once AFC writes its var file: AFC's save_vars writes nothing
        until PREP has run, so until then AFC.var.unit keeps the lane records
        of the units the state names, and a restart holds them for those
        units. An owner a claim records before then is left pending, and the
        chain watch writes it once PREP has run (see _scout_tick).
        """
        afc = self.printer.lookup_object("AFC", None)
        if afc is not None and not getattr(afc, "prep_done", True):
            self._bay_owner_pending = True
            return
        self._bay_owner_pending = False
        owners = self._owners()
        if self._load_bay_owner()[0] == owners:
            return
        self._state_set({self._BASE_SECTION + " " + self.name:
                         {"bay_owner": self._ser_bay_owner(owners)}})

    def _keep_guessed_owners(self) -> None:
        """
        Write bay_owner before a pin changes or a unit is forgotten, while
        the state block has no bay_owner key yet.

        Without the key, the next start guesses each bay's owner again from
        the pins it finds (see _capture_boot_records), so a unit pinned
        where another was, or drawing the bay of a forgotten one, would be
        handed that unit's records. Until PREP has run, AFC.var.unit holds
        what this start held for each bay, so those owners are written; once
        it has, every owner recorded (see _file_owners). Best-effort: what
        it comes before goes ahead whatever happens here.
        """
        try:
            if self._load_bay_owner()[1]:
                return
            self._state_set({self._BASE_SECTION + " " + self.name:
                             {"bay_owner": self._ser_bay_owner(
                                 self._file_owners())}})
        except Exception:
            pass

    def _file_owners(self) -> Dict[str, str]:
        """
        The owner of each bay whose lane records AFC.var.unit holds.

        Until PREP has run, these are the units this start held each bay's records
        for (see _capture_boot_records). Once it has, every owner recorded, as every
        save since wrote each bay from its claimed unit or from what is held for its
        owner.

        :return dict: bay -> uid
        """
        afc = self.printer.lookup_object("AFC", None)
        if afc is None or getattr(afc, "prep_done", True):
            return dict(self._owners())
        return {bay: _norm_uid(entry.get("uid")) for bay, entry in
                (getattr(self, "_held", None) or {}).items()
                if entry.get("uid")}

    def _given_names(self) -> Tuple[List[str], List[str],
                                    Dict[Tuple[str, int], str],
                                    Dict[Tuple[str, str], Tuple[int, str]]]:
        """
        The names of the four AMS ranks and of the ht_names indices, each
        given to one bay only.

        A bay's name is its ams_names / ht_names entry, else its
        Bambu_AMS_# / Bambu_AMS_HT_# default. One name on two bays would be
        one section fabricated twice, and would leave an AMS rank with no
        name of its own. A name's index is its rank, which sets its lanes
        and T#, so within a family an entry keeps its name: a default of an
        index past the list that an entry holds takes the first of
        <default>_2, <default>_3, ... no bay has (for an HT, see
        _name_for_index). An entry gives way when an earlier entry of its
        family holds it, and when it is a name of the other family: an HT
        entry gives way to an AMS default, an AMS entry to an HT entry or
        default. Its bay then takes its own default, suffixed the same way
        when a bay has that.

        :return tuple: (the names of AMS ranks 0-3, the names of the
            ht_names indices, (family, index) -> a console line for each of
            those bays named neither by its entry nor by its default,
            (family, entry) -> (the entry's index, what it gave way to) for
            each entry that gave way)
        """
        key = (self.unit_prefix, tuple(self.ams_names), tuple(self.ht_names))
        cached = getattr(self, "_given_cache", None)
        if cached is not None and cached[0] == key:
            return (list(cached[1]), list(cached[2]), dict(cached[3]),
                    dict(cached[4]))
        n_ams = min(len(self.ams_names), _MAX_AMS_BAYS)
        n_ht = len(self.ht_names)
        ams = [f"{self.unit_prefix}_{i + 1}" for i in range(_MAX_AMS_BAYS)]
        ht = [f"{self.unit_prefix}_HT_{j + 1}" for j in range(n_ht)]
        stem = f"{self.unit_prefix}_HT_"
        ams_past = {ams[i]: f"the default name of AMS bay {i + 1}"
                    for i in range(n_ams, _MAX_AMS_BAYS)}

        def _ht_past(name: str) -> Optional[str]:
            """
            Whether a name is the default of an HT index past ht_names.

            :param name: a name
            :return str: what gives it when it is such a default, None otherwise
            """
            num = name[len(stem):] if name.startswith(stem) else ""
            if num.isdecimal() and num == str(int(num)) and int(num) > n_ht:
                return f"the default name of HT bay {num}"
            return None

        def _free(name: str) -> str:
            """
            A default name no bay has yet.

            :param name: a default name
            :return str: it, else the first of name_2, name_3, ... no bay has
            """
            got, k = name, 2
            while got in given or got in ams_past or _ht_past(got):
                got, k = f"{name}_{k}", k + 1
            return got

        # name -> what gives it: the entries first, so no default takes one.
        given: Dict[str, str] = {}
        gave: Dict[Tuple[str, str], Tuple[int, str]] = {}
        waiting = []
        for fam, key_name, entries, out in (
                ("ht", "ht_names", self.ht_names, ht),
                ("ams", "ams_names", self.ams_names[:n_ams], ams)):
            for i, entry in enumerate(entries):
                giver = (given.get(entry)
                         or (ams_past.get(entry) if fam == "ht"
                             else _ht_past(entry)))
                if giver is None:
                    out[i] = entry
                    given[entry] = f"{key_name} entry {i + 1}"
                else:
                    gave.setdefault((fam, entry), (i, giver))
                    waiting.append((fam, key_name, out, i, entry, giver))
        notes: Dict[Tuple[str, int], str] = {}
        for i in range(n_ams, _MAX_AMS_BAYS):
            default = ams[i]
            what = ams_past.pop(default)
            ams[i] = _free(default)
            if ams[i] != default:
                notes[("ams", i)] = (
                    f"the default name of AMS bay {i + 1} ({default}) is "
                    f"{given[default]}, so AMS bay {i + 1} is named "
                    f"{ams[i]}.")
            given[ams[i]] = what
        for fam, key_name, out, i, entry, giver in waiting:
            label = "HT" if fam == "ht" else "AMS"
            out[i] = _free(f"{stem}{i + 1}" if fam == "ht"
                           else f"{self.unit_prefix}_{i + 1}")
            given[out[i]] = f"the name of {label} bay {i + 1}"
            notes[(fam, i)] = (
                f"{key_name} entry {i + 1} ({entry}) is also {giver}, so "
                f"{label} bay {i + 1} is named {out[i]} -- give each bay a "
                f"name of its own.")
        self._given_cache = (key, list(ams), list(ht), dict(notes),
                             dict(gave))
        return ams, ht, notes, gave

    def _name_for_index(self, family: str, i: int) -> str:
        """
        The name of the i-th bay of a family (0-based): the operator's
        ams_names / ht_names entry if one is set for that index, else the
        Bambu_AMS_# / Bambu_AMS_HT_# default, and never a name another bay has
        (see _given_names): an HT default an ht_names entry holds takes the
        first of <default>_2, <default>_3, ... no bay has.

        Names are assigned by rank within a family (lowest lane = index 0), so
        position defines the name. An AMS index past the four AMS bays gives
        its entry or default as written: no bay is built for it.

        :param family: "ams" or "ht"
        :param i: the bay's 0-based rank within its family
        :return str: the bay name
        """
        ams, ht, _notes, _gave = self._given_names()
        if family == "ht":
            if i < len(ht):
                return ht[i]
            default = f"{self.unit_prefix}_HT_{i + 1}"
            name, k = default, 2
            while name in ht or name in ams:
                name, k = f"{default}_{k}", k + 1
            return name
        if i < len(ams):
            return ams[i]
        return (self.ams_names[i] if i < len(self.ams_names)
                else f"{self.unit_prefix}_{i + 1}")

    def _name_note(self, family: str, i: int) -> Optional[str]:
        """
        The console line for a bay named neither by its own entry nor its default.

        See _given_names.

        :param family: "ams" or "ht"
        :param i: a built bay's 0-based rank within its family
        :return str: the line, or None for any other bay
        """
        ams, ht, notes, _gave = self._given_names()
        if (family, i) in notes or family != "ht" or i < len(ht):
            return notes.get((family, i))
        default = f"{self.unit_prefix}_HT_{i + 1}"
        name = self._name_for_index("ht", i)
        if name == default:
            return None
        holder = (f"ht_names entry {ht.index(default) + 1}" if default in ht
                  else "the name of an AMS bay")
        return (f"the default name of HT bay {i + 1} ({default}) is {holder}, "
                f"so HT bay {i + 1} is named {name}.")

    @staticmethod
    def _drop_name_holders(name_map: Dict[str, str],
                           lane_map: Dict[str, Tuple[int, int]],
                           name: str, keep: set) -> List[str]:
        """
        Drop every uid outside ``keep`` recorded with ``name`` from both maps.

        A name is one bay, so it belongs to one uid; the lane entry goes too,
        since those lanes are the bay's and now belong to the new holder.

        :param name_map: uid -> unit name, edited in place
        :param lane_map: uid -> (first lane, span), edited in place
        :param name: the unit name being taken
        :param keep: uids that keep their entries
        :return list: the uids dropped
        """
        gone = [u for u, n in name_map.items() if n == name and u not in keep]
        for u in gone:
            name_map.pop(u, None)
            lane_map.pop(u, None)
        return gone

    def _name_index(self, family: str, name: str) -> Optional[int]:
        """
        The first index _name_for_index gives ``name`` at: the bay the list
        entries and defaults give it to (see _given_names), then an AMS
        entry past the fourth, else the number in its Bambu_AMS_# /
        Bambu_AMS_HT_# default (suffixed or not), which only names the
        indices past the list.

        :param family: "ams" or "ht"
        :param name: a unit name
        :return int: the 0-based index, None when no index gives that name
        """
        ams, ht, _notes, _gave = self._given_names()
        given = ht if family == "ht" else ams
        if name in given:
            return given.index(name)
        names = self.ht_names if family == "ht" else self.ams_names
        for i in range(len(given), len(names)):
            if names[i] == name:
                return i
        stem = (f"{self.unit_prefix}_HT_" if family == "ht"
                else f"{self.unit_prefix}_")
        num = name[len(stem):].split("_")[0] if name.startswith(stem) else ""
        if (num.isdecimal()
            and int(num) > len(names)
            and self._name_for_index(family, int(num) - 1) == name):
            return int(num) - 1
        return None

    def _rank_of(self, family: str, name: str) -> Optional[int]:
        """
        The rank a saved unit name stands for.

        An HT may hold the name of any rank; an AMS only those of ranks 0-3,
        the four AMS bays a Bambu bus can address (see _MAX_AMS_BAYS).

        :param family: "ams" or "ht"
        :param name: a saved unit name
        :return int: the rank, None when the family is never given that name
        """
        i = self._name_index(family, name)
        if i is None or (family == "ams" and i >= _MAX_AMS_BAYS):
            return None
        return i

    @staticmethod
    def _lanes_text(start: int, span: int) -> str:
        """
        Describe a lane range with its home tools.

        :param start: first lane number
        :param span: lane count
        :return str: "laneN-laneM (TN-TM)", or "laneN (TN)" for one lane
        """
        hi = start + span - 1
        if span == 1:
            return f"lane{start} (T{start})"
        return f"lane{start}-lane{hi} (T{start}-T{hi})"

    def _layout_note(self, b: Dict[str, Any]) -> Optional[str]:
        """
        The console note for a rostered unit whose name or lanes differ from
        what the state file records for it.

        :param b: its built bay, carrying "had": the recorded (first lane,
            span) or None, and, when it drew a new name, "was": (the recorded
            name, the uid that holds that name or None)
        :return str: the note naming the uid, both names and both lane
            spans; None when neither changed
        """
        fam = "HT" if b["family"] == "ht" else "AMS"
        had: Optional[Tuple[int, int]] = b.get("had")
        now = self._lanes_text(b["lane"], b["slots"])
        moved = had is not None and tuple(had) != (b["lane"], b["slots"])
        if "was" not in b:
            if had is None or not moved:
                return None
            return (f"AFC_BridgeBox {self.name}: {fam} {b['uid']} "
                    f"({b['name']}) keeps its name, and its lanes and T# "
                    f"changed: {self._lanes_text(*had)} -> {now}.")
        saved, holder = b["was"]
        key = "ht_names" if b["family"] == "ht" else "ams_names"
        gave = self._given_names()[3].get((b["family"], saved))
        if holder:
            why = f"which {holder} also holds"
        elif (b["family"] == "ams"
              and self._name_index("ams", saved) is not None):
            why = f"past the {_MAX_AMS_BAYS} AMS bays a Bambu bus addresses"
        elif gave:
            why = f"whose {key} entry {gave[0] + 1} gives way to {gave[1]}"
        else:
            why = f"which no {key} entry or default name gives an {fam}"
            names = self.ht_names if b["family"] == "ht" else self.ams_names
            stem = (f"{self.unit_prefix}_HT_" if b["family"] == "ht"
                    else f"{self.unit_prefix}_")
            num = saved[len(stem):] if saved.startswith(stem) else ""
            if num.isdecimal() and 1 <= int(num) <= len(names):
                why += (f" ({key} replaces the first {len(names)} default "
                        f"names)")
        if had is not None and moved:
            lanes = (f"its lanes and T# changed: "
                     f"{self._lanes_text(*had)} -> {now}")
        elif had is not None:
            lanes = f"its lanes stay {now}"
        else:
            lanes = f"its lanes and T# are now {now}"
        return (f"AFC_BridgeBox {self.name}: {fam} {b['uid']} was recorded "
                f"as {saved}, {why}. It is now {b['name']}, and {lanes}. "
                f"Lane records saved under {saved} (spool, material, colour, "
                f"T# map) do not carry over to {b['name']}; a tagged spool is "
                f"read again from its tag. Its learned bowden lengths stay "
                f"with the unit.")

    def _band_note(self, held: List[Dict[str, Any]], band: int, need: int,
                   clash: List[int],
                   later: Optional[Dict[int, str]] = None) -> str:
        """
        The console note for recorded HTs whose lanes sit past the AMS bays
        pool_ams and the recorded AMS need.

        Such an HT keeps its lanes, T# and lane records, so the AMS band
        stays as wide as they sit past, unless one of the HT lanes there
        already exists outside this chain, or a chain further down the
        config builds it, either of which would stop Klipper from starting.
        Then the band is what pool_ams and the recorded AMS need, and the HT
        lanes follow it. An HT whose recorded lanes sit past more bays than
        the band (one capped at four or at the band the last start built,
        or a wider band another held HT sets) moves to the band's edge, and
        the note says where instead of saying it keeps its lanes.

        :param held: those HT bays, named and ranked; the lane map still
            holds their recorded lanes
        :param band: the band their lanes sit past
        :param need: the band pool_ams and the recorded AMS need
        :param clash: HT lane numbers at ``band`` that exist outside this
            chain or that a later chain builds, empty when none does
        :param later: lane number -> the later chain building it, for the
            lanes no section outside this chain declares
        :return str: the note
        """
        one = len(held) == 1
        hts = ", ".join(f"HT {b['uid']} ({b['name']}) on "
                        f"{self._lanes_text(*self._lane_map[b['uid']])}"
                        for b in held)
        its = "its" if one else "their"
        if clash:
            by = (later or {}).get(clash[0])
            why = (f"[AFC_BridgeBox {by}] further down the config builds "
                   f"lane{clash[0]}" if by else
                   f"[AFC_lane lane{clash[0]}] already exists outside this "
                   f"chain")
            return (f"AFC_BridgeBox {self.name}: {hts} cannot keep {its} "
                    f"lanes: {why}. The AMS band is {need} bays, what "
                    f"pool_ams and the recorded AMS need, and the HT lanes "
                    f"follow it.")
        # HT rank r sits on lane_base + band * 4 + r; one recorded elsewhere
        # moves there.
        moves = [(b, self.lane_base + band * 4 + b["rank"]) for b in held]
        moves = [(b, at) for b, at in moves
                 if self._lane_map[b["uid"]][0] != at]
        if moves:
            text = (f"AFC_BridgeBox {self.name}: {hts} hold{'s' if one else ''} "
                    f"the AMS band at {band} bays although pool_ams and the "
                    f"recorded AMS need only {need}. "
                    + ", ".join(
                        f"{b['name']} moves to "
                        f"{self._lanes_text(at, self._lane_map[b['uid']][1])}"
                        for b, at in moves)
                    + f", past those {band} bays.")
        else:
            text = (f"AFC_BridgeBox {self.name}: {hts} keep{'s' if one else ''} "
                    f"{its} lanes, so the AMS band stays {band} bays although "
                    f"pool_ams and the recorded AMS need only {need}.")
        if self.pool_ams or self.pool_ht:
            text += f" Set pool_ams: {band} to silence this."
        uid = f"UID={held[0]['uid']}" if one else "UID=<uid> for each"
        return (text + f" AFC_BRIDGEBOX_FORGET CHAIN={self.name} {uid} lets "
                f"the band shrink to {need} at the next RESTART, and erases "
                f"what {'it' if one else 'they'} learned.")

    def _moved_lanes(self, bays: List[Dict[str, Any]],
                     waiting: List[Dict[str, Any]]
                     ) -> Dict[str, Dict[str, Any]]:
        """
        The lanes a rostered unit's state records that another bay, or none,
        holds in this layout.

        AFC saves a toolhead's loaded lane by lane name alone, so such a
        record names the old unit's filament on the new holder's lane (see
        loaded_lane_moved).

        :param bays: every bay built, lanes set, rostered ones carrying
            "had": their recorded (first lane, span) or None
        :param waiting: rostered AMS left without a bay, carrying "had"
        :return dict: lane name -> {"uid", "name", "family" of the unit the
            state records on it; "now": the name of the bay holding it, or
            None}
        """
        owner: Dict[int, Dict[str, Any]] = {}
        for b in bays:
            for n in range(b["lane"], b["lane"] + b["slots"]):
                owner.setdefault(n, b)
        moves: Dict[str, Dict[str, Any]] = {}
        for b in [x for x in bays if not x["spare"]] + waiting:
            had = b.get("had")
            if not had:
                continue
            for n in range(had[0], had[0] + had[1]):
                nb = owner.get(n)
                if nb is b:
                    continue
                moves.setdefault(f"lane{n}", {
                    "uid": b["uid"], "family": b["family"],
                    "name": b["was"][0] if "was" in b else b.get("name"),
                    "now": nb["name"] if nb else None})
        return moves

    def _all_ams_bays_held(self, uid: str, holders: List[Tuple[str, str]],
                           live: bool) -> str:
        """
        What an AMS that finds all four AMS bays held can do about it.

        A Bambu bus addresses at most four AMS, so neither pool_ams nor a
        restart adds a bay: only a unit this AMS replaces gives one up.
        FORGET of that unit frees its bay, which the waiting AMS claims live
        on a chain with a pool and takes at the next restart without one,
        as it does when auto-removal drops an absent AMS from the recorded
        roster. A FORGET named for a bay AFC records a lane of as loaded to
        a toolhead says what clears that record first (see _loaded_clause).
        With a pool and no roster: option, AFC_BRIDGEBOX_REPLACE does both
        in one step for a bay held for an offline unit (see _replace_text).
        With roster: set, the option names the units a restart builds bays
        for, so the swap is made there too.

        :param uid: the AMS with no bay
        :param holders: (name, uid) of the units holding the AMS bays; a
            unit not yet named has name ""
        :param live: whether the chain is running, so the holders that are
            offline (the ones this AMS can be replacing) are named
        :return str: the sentences that follow "has no bay: "
        """
        def _unit(name: str, u: str) -> str:
            """
            Label a unit by name and uid.

            :param name: a unit name, "" when it has none
            :param u: its uid
            :return str: "name (uid)", or the uid alone
            """
            return f"{name} ({u})" if name else u

        text = (f"all {_MAX_AMS_BAYS} AMS bays belong to other units "
                f"({', '.join(_unit(*h) for h in holders)}), and a Bambu bus "
                f"addresses at most {_MAX_AMS_BAYS} AMS, so neither pool_ams "
                f"nor RESTART adds one.")
        offline = ([h for h in holders if not self._uid_online_now(h[1])]
                   if live else [])
        if len(offline) == 1:
            one = "it"
            forget = (f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} "
                      f"UID={offline[0][1]}"
                      + self._loaded_clause(self._bay_of_uid(offline[0][1])))
            text += (f" {_unit(*offline[0])} is offline: if this AMS "
                     f"replaces it, ")
        else:
            one = "that unit"
            forget = (f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} "
                      f"UID=<that unit's uid>")
            if offline:
                text += (f" Offline: "
                         f"{', '.join(_unit(*h) for h in offline)}.")
            text += " If this AMS replaces one of them, "
        pooled = bool(self.pool_ams or self.pool_ht)
        if self._unlisted(uid):
            its = "its" if one == "it" else "that unit's"
            return (text + f"replace {its} entry in roster: with "
                    f"{self._roster_entry(uid)}, run {forget}, and RESTART.")
        text += (f"{forget} frees that bay "
                 + ("and this AMS claims it live" if pooled
                    else "for it at the next RESTART"))
        if self._roster_source == "option":
            text += f"; remove {one} from roster: too"
        text += "."
        text += self._replace_text(uid, [
            pu for pu in (self._bay_of_uid(h[1]) for h in offline)
            if pu is not None and pu.get("family") == "ams"])
        if (not pooled
            and self.removal_grace
            and self._roster_source != "option"):
            text += (f" An AMS offline on a live chain for "
                     f"{self.removal_grace:.0f}s is removed from the recorded "
                     f"roster, which frees its bay at the next RESTART too.")
        return text

    def _overlap_error(self, a: Dict[str, Any], b: Dict[str, Any]) -> str:
        """
        The refusal for two bays whose lanes overlap, naming both.

        Pass 1b gives every bay its own name and every name its own rank,
        and the HT band starts past every AMS bay, so this is the backstop
        for two bays on one lane. A recorded unit's lanes follow from the
        name its state records. FORGET and UNASSIGN are gcode commands, which
        need Klipper running, so the recovery the message gives is removing
        that record by hand; the unit then draws a free bay.

        :param a: one bay (name, uid, family, rank, lane, slots, spare)
        :param b: the bay whose lanes start inside a's
        :return str: the error text
        """
        def _span(x: Dict[str, Any]) -> str:
            """
            Describe a bay's lanes.

            :param x: a bay
            :return str: "laneN" or "laneN-laneM"
            """
            lo, hi = x["lane"], x["lane"] + x["slots"] - 1
            return f"lane{lo}" if lo == hi else f"lane{lo}-lane{hi}"
        head = (f"bays {a['name']} ({_span(a)}) and {b['name']} ({_span(b)}) "
                f"overlap")
        uids = [x["uid"] for x in (a, b) if not x["spare"]]
        if not uids:
            return f"{head}."
        return (f"{head}: their lanes follow from the unit names the #~# "
                f"block of {self.state_file} records for "
                f"{' and '.join(uids)}. AFC_BRIDGEBOX_FORGET and "
                f"AFC_BRIDGEBOX_UNASSIGN need Klipper running, so remove the "
                f"name_map and lane_map entries of {' or '.join(uids)} there "
                f"by hand and RESTART; that unit then draws a free bay.")

    @staticmethod
    def _ser_maps(lane_map: Dict[str, Tuple[int, int]],
                  name_map: Dict[str, str]) -> Dict[str, str]:
        """
        Serialize the lane and name maps for the state block.

        :param lane_map: uid -> (first lane, span)
        :param name_map: uid -> unit name
        :return dict: both maps as their state-block key/value strings
        """
        lanes = ", ".join(
            f"{uid}:{start}:{span}"
            for uid, (start, span) in sorted(lane_map.items(),
                                             key=lambda kv: kv[1][0]))
        names = ", ".join(
            f"{uid}:{name_map[uid]}"
            for uid in sorted(name_map,
                              key=lambda u: lane_map.get(u, (1 << 30,))))
        return {"lane_map": lanes, "name_map": names}

    def _flush_maps(self) -> None:
        """
        Persist the lane and name maps when fabrication just grew them.
        """
        if not getattr(self, "_maps_dirty", False):
            return
        self._state_set({self._BASE_SECTION + " " + self.name:
                         self._ser_maps(self._lane_map, self._name_map)})
        self._maps_dirty = False

    def _register_mux(self, name: str, handler: Any, desc: str) -> None:
        """
        Register one CHAIN-muxed command.

        The first master to load also claims the no-CHAIN default so a single-chain
        operator never has to type CHAIN=; a second chain is reached via
        CHAIN=<its name>.

        :param name: the command name
        :param handler: its handler
        :param desc: its help text
        """
        gcode = self.printer.lookup_object("gcode", None)
        if gcode is None:
            return
        gcode.register_mux_command(name, "CHAIN", self.name, handler, desc=desc)
        try:
            gcode.register_mux_command(name, "CHAIN", None, handler, desc=desc)
        except Exception:
            pass                  # a second chain: it is reached via CHAIN=

    def _register_commands(self) -> None:
        """
        Register the master's operator commands (CHAIN-muxed).
        """
        self._register_mux(
            "AFC_BRIDGEBOX_FORGET", self.cmd_AFC_BRIDGEBOX_FORGET,
            "Release a departed unit's lane numbers and unit name for reuse "
            "and erase its learned values and saved lane records: "
            "AFC_BRIDGEBOX_FORGET [CHAIN=<chain>] "
            "UID=<uid> (or NAME=<unit name>) [FORCE=1]. With no argument, "
            "pops a picker of every recorded unit with a Forget button each.")
        self._register_mux(
            "AFC_BRIDGEBOX_ASSIGN", self.cmd_AFC_BRIDGEBOX_ASSIGN,
            "Pin a detected unit's UID onto a named pool bay, live: "
            "AFC_BRIDGEBOX_ASSIGN [CHAIN=<chain>] UID=<uid> NAME=<bay name> "
            "[FORCE=1] (or UID=<uid> alone to pop the bay picker). Refuses an "
            "occupied bay or one saved for another unit -- UNASSIGN it first "
            "-- with roster: set, a uid it does not list, and moving a unit "
            "off a bay with a lane AFC records as loaded to the toolhead "
            "(FORCE=1 clears that record).")
        self._register_mux(
            "AFC_BRIDGEBOX_UNASSIGN", self.cmd_AFC_BRIDGEBOX_UNASSIGN,
            "Unlink a UID from its pool bay (keeps learned values), live: "
            "AFC_BRIDGEBOX_UNASSIGN [CHAIN=<chain>] UID=<uid> "
            "(or NAME=<bay name>) [FORCE=1]")
        self._register_mux(
            "AFC_BRIDGEBOX_BAYS", self.cmd_AFC_BRIDGEBOX_BAYS,
            "Pop the bay manager: every pool bay, its occupant, an Unassign "
            "button for each occupied bay, and a Replace button for a unit "
            "waiting for a bay -- AFC_BRIDGEBOX_BAYS [CHAIN=<chain>]")
        self._register_mux(
            "AFC_BRIDGEBOX_REPLACE", self.cmd_AFC_BRIDGEBOX_REPLACE,
            "Forget an offline unit and put a new unit on its bay, live: "
            "AFC_BRIDGEBOX_REPLACE [CHAIN=<chain>] UID=<new uid> "
            "OLD=<old uid or bay name> [FORCE=1]. Without OLD, pops a picker "
            "of the bays held for offline units.")

    def cmd_AFC_BRIDGEBOX_FORGET(self, gcmd: Any) -> None:
        """
        The deliberate half of removal: erase a unit's tombstones.

        Unplugging erases nothing: with a pool the unit stays in the roster and its
        bay waits for it, and without one auto-removal (see _prune_missing) only
        edits the roster, leaving the unit's lane numbers, unit name, and learned
        values recorded for its return. For a unit that is not coming back, this
        command releases them: the uid leaves the recorded roster and the lane and
        name maps, its learned record (see _learned_section) is erased, and so is
        every section stored under its name and lanes (from the state block and
        auto_vars), so the freed name and lanes are safe for the next new unit to
        reuse. A uid with only a learned record left can be forgotten too, and so
        can one that holds a pool bay, or is a bay's last owner, without being
        recorded (claimed onto a spare and pulled within enroll_grace). Its
        bay_owner entry and the lane records kept for it (see _held_for) go too,
        and AFC.var.unit is saved without them, so a re-plug starts from clean
        lanes, and so does what the chain watch tracks for it this session (see
        _drop_watch_state).

        Works on a live unit too: it drops the unit's lanes/T# immediately and
        parks the uid on a suppress list (see _forget_suppressed) so the scout does
        not re-enroll the hardware still on the wire; the hold clears when the unit
        is physically pulled, so a genuine re-plug enrolls it fresh. Forgetting a
        unit claimed onto a bay mid-print is refused, since it would pull a live
        lane out from under the job. So is forgetting a unit while AFC records a
        lane of its bay as loaded to a toolhead, claimed or not, and forgetting a
        unit whose bay is unclaimed before PREP has restored that record. FORCE=1
        overrides the mid-print, PREP and loaded-lane refusals, clearing the
        loaded-lane record first. The loaded-lane refusal comes before the print
        one, so it names the lane FORCE=1 clears. With neither UID nor NAME, pops a
        picker listing every recorded unit, each with its own Forget button.

        Usage
        -------
        `AFC_BRIDGEBOX_FORGET [CHAIN=<chain>] UID=<unit uid>|NAME=<unit name> [FORCE=1]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_FORGET UID=0123456789ABCDEF01234567
        ```
        """
        uid = _norm_uid(gcmd.get("UID", ""))
        name = (gcmd.get("NAME", "") or "").strip()
        sec = self._BASE_SECTION + " " + self.name
        lane_map = self._load_lane_map()
        name_map = self._load_name_map()
        if not uid and not name:
            # No target: pop a picker of every recorded unit, flagging live
            # ones.
            roster_raw = self._state_get(sec, "roster") or ""
            order = [_norm_uid(e.partition(":")[2])
                     for e in roster_raw.split(",")
                     if e.partition(":")[2].strip()]
            known, seen = [], set()
            for u in order + list(name_map) + list(lane_map):
                u = _norm_uid(u)
                if u and u not in seen:
                    seen.add(u)
                    known.append(u)
            lines, buttons = [], []
            for u in known:
                nm = name_map.get(u) or "(unnamed)"
                live = "  -- LIVE (drops now)" if self._uid_online_now(u) \
                    else ""
                lines.append(f"{nm}  [{u[:8]}…]{live}")
                if len(buttons) < self._MAX_BAY_BUTTONS:
                    buttons.append((
                        f"Forget {nm}",
                        f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} UID={u}",
                        "error"))
            if not lines:
                lines = ["No recorded units to forget."]
            self._prompt("Forget a Bambu AMS unit", lines, buttons,
                         timeout=180.0)
            gcmd.respond_info(
                f"AFC_BridgeBox {self.name}: forget picker -- "
                + (", ".join(name_map.get(u) or u[:8] for u in known)
                   or "nothing recorded"))
            return
        if not uid and name:
            hits = [u for u, n in name_map.items() if n == name]
            if not hits:
                error_str = (
                    f"AFC_BRIDGEBOX_FORGET: no recorded unit named {name!r} "
                    f"(known: {', '.join(sorted(name_map.values())) or 'none'})")
                raise gcmd.error(error_str)
            uid = hits[0]
        if not uid:
            error_str = "AFC_BRIDGEBOX_FORGET: give UID=<unit_uid> or NAME=<unit name>"
            raise gcmd.error(error_str)
        cp = self._read_state()
        roster_raw = (dict(cp.items(sec)).get("roster") or ""
                      if cp.has_section(sec) else "")
        entries = [e.strip() for e in roster_raw.split(",") if e.strip()]
        in_roster = any(_norm_uid(e.partition(":")[2]) == uid
                        for e in entries)
        record = self._learned_section(uid)
        # A unit claimed onto a spare and pulled before it was recorded has
        # nothing saved, but still holds that bay, or once released is still
        # named as the bay's last owner, with lane records kept for it.
        if (uid not in lane_map
            and uid not in name_map
            and not in_roster
            and not cp.has_section(record)
            and self._bay_of_uid(uid) is None
            and uid not in self._owners().values()
            and not any(_norm_uid(e.get("uid")) == uid for e in
                        (getattr(self, "_held", None) or {}).values())):
            error_str = f"AFC_BRIDGEBOX_FORGET: nothing recorded for uid {uid}"
            raise gcmd.error(error_str)
        # Refuse here, before anything is erased, if a lane of the unit's bay
        # is still in a toolhead, claimed or not.
        pu = next((p for p in getattr(self, "_pool_units", [])
                   if _norm_uid(p.get("uid")) == uid), None)
        unclaimed: List[str] = []
        if pu is not None and pu.get("bound"):
            self._refuse_toolhead_release(gcmd, "AFC_BRIDGEBOX_FORGET",
                                          pu["bound"])
        elif pu is not None:
            unclaimed = self._refuse_unclaimed_release(
                gcmd, "AFC_BRIDGEBOX_FORGET", pu, f"forgets {uid}")
        # A live FORGET drops the unit's lanes/T# now and holds the uid on the
        # suppress list (below). Mid-print it refuses without FORCE for a
        # unit claimed onto a bay, matching the auto-drop print gate.
        online_now = self._uid_online_now(uid)
        if (pu is not None
            and pu.get("bound")
            and self._is_printing()
            and not gcmd.get_int("FORCE", 0)):
            error_str = (
                f"AFC_BRIDGEBOX_FORGET: {uid} is on {pu['name']} and a print "
                f"is active -- dropping its lanes now would disrupt the "
                f"print. Finish the print, or FORCE=1 to forget it anyway")
            raise gcmd.error(error_str)
        cleared = self._clear_bay_toolhead(pu) if unclaimed else []
        # Tell the bridge to forget it too, or a gone unit reads its own
        # re-assert back and flaps online.
        self._bridge_forget(uid)
        unit_name = name_map.pop(uid, None)
        start_span = lane_map.pop(uid, None)
        if not cp.has_section(sec):
            cp.add_section(sec)
        if roster_raw:                # never invent an empty roster key
            cp.set(sec, "roster", ", ".join(
                e for e in entries
                if _norm_uid(e.partition(":")[2]) != uid))
        for k, v in self._ser_maps(lane_map, name_map).items():
            cp.set(sec, k, v)
        # With no bay_owner key yet, the next start would guess the owners
        # again, and could give this unit's bay and records to another (see
        # _keep_guessed_owners).
        owners, present = self._load_bay_owner()
        if not present:
            owners = self._file_owners()
        if uid in owners.values() or not present:
            cp.set(sec, "bay_owner", self._ser_bay_owner(
                {b: u for b, u in owners.items() if u != uid}))
        # Erase the uid's learned record and what is stored under its name and
        # lanes, unless another uid is recorded with them.
        if unit_name in name_map.values():
            unit_name = None
        if start_span:
            others = {n for s0, k in lane_map.values()
                      for n in range(s0, s0 + k)}
            lanes = [n for n in range(start_span[0], sum(start_span))
                     if n not in others]
        else:
            lanes = []
        doomed = [record]
        if unit_name:
            doomed += [f"{self._UNIT_SECTION} {unit_name}",
                       f"AFC_hub {unit_name}",
                       f"temperature_sensor {unit_name}"]
        doomed += [f"AFC_lane lane{n}" for n in lanes]
        by_name = f"{self._UNIT_SECTION} {unit_name}" if unit_name else None
        erased = (cp.has_section(record)
                  or bool(by_name and cp.has_section(by_name)))
        for section in doomed:
            if cp.has_section(section):
                cp.remove_section(section)
        self._write_state(cp)
        autov = self._read_ini(self.auto_vars_file)
        if autov is not None and by_name:
            erased = erased or autov.has_section(by_name)
        if autov is not None and any(autov.has_section(s) for s in doomed):
            for section in doomed:
                if autov.has_section(section):
                    autov.remove_section(section)
            try:
                with open(self.auto_vars_file, "w") as fp:
                    fp.write("# This file is autogenerated and updated when "
                             "variables are not in your normal AFC config "
                             "files\n\n")
                    autov.write(fp)
            except Exception:
                pass
        # This session keeps running on what was fabricated at boot; the
        # in-memory maps sync so a unit appearing later this session cannot
        # collide with the freed range.
        if hasattr(self, "_lane_map"):
            self._lane_map.pop(uid, None)
            self._name_map.pop(uid, None)
        self._drop_watch_state(uid)
        # Free the unit's slot now. With a pool the next unit of the family
        # claims it live; without one the bay goes to a unit at restart.
        freed_live = False
        if pu is not None:
            if pu.get("bound"):                         # drop its lanes/T# live
                self._release_pool_unit(pu["bound"], "AFC_BRIDGEBOX_FORGET")
            pu["uid"] = None                            # -> free pool slot
            freed_live = bool(self.pool_ams or self.pool_ht)
        # Its saved lane records go too, so a re-plug starts clean.
        held = getattr(self, "_held", None) or {}
        had_records = any(_norm_uid(e.get("uid")) == uid
                          for e in held.values())
        self._bay_owner = {b: u for b, u in self._owners().items()
                           if u != uid}
        self._held = {b: e for b, e in held.items()
                      if _norm_uid(e.get("uid")) != uid}
        # Saved now, so AFC.var.unit does not keep them until some later save
        # (see _fill_held_bays). Before PREP nothing is written, and the next
        # start holds them for no unit: bay_owner no longer names this one.
        if had_records:
            afc = self.printer.lookup_object("AFC", None)
            try:
                if afc is not None:
                    afc.save_vars()
            except Exception:
                pass
        # Still on the wire: suppress re-enroll until it is physically pulled.
        if online_now:
            self._forget_suppressed.add(uid)
        gone = " and ".join(what for what, on in (
            ("learned values", erased), ("saved lane records", had_records))
            if on)
        gone = f"{gone} erased" if gone else ""
        freed = []
        if lanes:
            lo, hi = lanes[0], lanes[-1]
            freed.append(f"lane{lo}" if lo == hi else f"lanes {lo}-{hi}")
        if unit_name:
            freed.append(f"the name {unit_name}")
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: forgot {uid}"
            + (f" -- {' and '.join(freed)} freed for reuse" if freed else "")
            + ((", " if freed else " -- ") + gone if gone else "")
            + (f" -- cleared {', '.join(cleared)} from the toolhead"
               if cleared else "")
            + (" -- slot freed to the pool LIVE; the next same-family unit "
               "claims it with no reboot." if freed_live
               else " -- its bay is free now, but with no pool nothing claims "
                    "it: AFC_BRIDGEBOX_ASSIGN another unit onto it while that "
                    "unit is online, or RESTART." if pu is not None
               else ". Applies at the next RESTART." if freed or in_roster
               else ".")
            + (" It is still on the wire, so re-enroll is suppressed until you "
               "physically pull it -- a re-plug then enrolls it fresh."
               if online_now else "")
            + (" NOTE: your roster: option still lists this uid -- remove "
               "it there too, the option overrides the recorded roster."
               if self._roster_source == "option"
               and uid in {x["uid"] for x in self.units} else ""))
        self._close_prompt()          # a removal-popup button may have run this

    def _drop_watch_state(self, uid: str) -> None:
        """
        Drop what the chain watch tracks for a forgotten uid this session:
        its absence clock, the release-side online clocks (a stale one would
        release a re-plugged unit on its first offline read), its waiting
        state, any popup queued for it, and its enrollment line if a failed
        tick left it unsaid.

        :param uid: the forgotten uid
        """
        self._missing_since.pop(uid, None)
        for attr in ("_last_online", "_online_run"):
            clock = getattr(self, attr, None)
            if clock is not None:
                clock.pop(uid, None)
        self._no_bay.pop(uid, None)
        self._no_bay_told.discard(uid)
        (getattr(self, "_unbayed", None) or {}).pop(uid, None)
        self._replace_offered.discard(uid)
        q = getattr(self, "_popup_queue", None)
        if q:
            q[:] = [e for e in q if e[1] != uid]
        pending = getattr(self, "_announce_pending", None)
        if pending:
            pending[:] = [e for e in pending
                          if _norm_uid(e.partition(":")[2]) != uid]

    def _model_for_uid(self, uid: str) -> Optional[str]:
        """
        The roster model tag recorded for a uid.

        Prefers the explicit roster: option, then the recorded roster file/ledger,
        except that an option `boxed` is only unconfirmed, so an ams1 or ams2 the
        recorded roster holds for that uid stands, as for a claim by the chain
        watch.

        :param uid: 24-hex unit uid
        :return str: ht/ams1/ams2/boxed, or None if it is not enrolled yet
        """
        uid = _norm_uid(uid)
        sec = self._BASE_SECTION + " " + self.name
        recorded = None
        for e in (self._state_get(sec, "roster") or "").split(","):
            m, _sep, u = e.partition(":")
            if _norm_uid(u) == uid:
                recorded = m.strip().lower()
                break
        for u in self.units:
            if _norm_uid(u.get("uid")) == uid:
                if (_norm_model(u["model"]) == "boxed"
                    and recorded in ("ams1", "ams2")):
                    return recorded
                return u["model"]
        return recorded

    def _unlisted(self, uid: str) -> bool:
        """
        Whether a set roster: option leaves this uid out.

        The option is then the whole roster, so the uid gets no bay of its own at
        restart and cannot be pinned to one.

        :param uid: a unit uid
        :return bool: True when a roster: option is set and does not list the uid
        """
        return (self._roster_source == "option"
                and _norm_uid(uid) not in {_norm_uid(u.get("uid"))
                                           for u in self.units})

    def _recorded_uids(self) -> set:
        """
        The uids that get a bay of their own at restart.

        :return set: the roster: option's uids when it is set, else the recorded
                     roster's
        """
        if getattr(self, "_roster_source", None) == "option":
            return {_norm_uid(u.get("uid")) for u in self.units}
        raw = self._state_get(
            f"{self._BASE_SECTION} {getattr(self, 'name', '')}",
            "roster") or ""
        return {_norm_uid(e.partition(":")[2]) for e in raw.split(",")
                if e.partition(":")[2].strip()}

    def _bay_held(self, pu: Dict[str, Any]) -> bool:
        """
        Whether a bay is saved for the uid on it.

        It is when the name map records the bay's name for that uid and the uid is
        recorded (see _recorded_uids), so it gets that bay at restart and the bay
        waits for that unit through an unplug instead of going back to the pool. A
        name kept for a uid outside the roster is a tombstone and holds nothing.

        :param pu: pool unit record
        :return bool: True when the bay's uid is saved on it
        """
        uid = _norm_uid(pu.get("uid"))
        return (bool(uid)
                and getattr(self, "_name_map", {}).get(uid) == pu.get("name")
                and uid in self._recorded_uids())

    def _name_holder(self, name: str, uid: str,
                     recorded: Optional[set] = None) -> Optional[str]:
        """
        The recorded uid, other than ``uid``, the name map saves ``name`` for.

        A name saved for a uid outside the roster is a tombstone that holds
        no bay, and _persist_pin takes it over; a recorded uid gets that
        name's bay at restart, so no other uid may be saved on it.

        :param name: a bay name
        :param uid: the uid asking for it
        :param recorded: the recorded uids; read when not given
        :return str: the holder's uid, or None
        """
        recorded = self._recorded_uids() if recorded is None else recorded
        uid = _norm_uid(uid)
        return next((u for u, n in getattr(self, "_name_map", {}).items()
                     if n == name and u != uid and u in recorded), None)

    def _roster_entry(self, uid: str) -> str:
        """
        The roster: entry for a uid, `<model>:<uid>`.

        The model is the recorded one, else the family of the bay it sits on, else
        of the name the name map keeps for it (ht, or boxed).

        :param uid: a unit uid
        :return str: its roster: entry
        """
        uid = _norm_uid(uid)
        model = self._model_for_uid(uid)
        if not model:
            bay = self._bay_of_uid(uid)
            name = getattr(self, "_name_map", {}).get(uid)
            ht = (bay.get("family") == "ht" if bay is not None
                  else bool(name) and self._rank_of("ht", name) is not None)
            model = "ht" if ht else "boxed"
        return f"{model}:{uid}"

    def _persist_pin(self, uid: str, first_lane: int, span: int,
                     name: str, model: str) -> None:
        """
        Persist a uid -> bay pin so it survives a restart: write the lane and
        name maps, and enroll the uid in the recorded roster if it is not there
        yet.

        In-memory maps are synced too so this session stays consistent, and the
        bay is held for the uid from here on (see _bay_held). Learned values
        are untouched: they follow the uid.

        :param uid: the unit's uid
        :param first_lane: the bay's first lane number
        :param span: 1 (HT) or 4 (AMS)
        :param name: the bay name the uid now owns
        :param model: the uid's roster model tag, for a fresh roster entry
        """
        self._keep_guessed_owners()
        sec = self._BASE_SECTION + " " + self.name
        lane_map = self._load_lane_map()
        name_map = self._load_name_map()
        # A spare can wear a name still recorded for a uid outside the roster
        # (see _roster_sections); pinning takes the name over from it.
        self._drop_name_holders(name_map, lane_map, name, {uid})
        lane_map[uid] = (first_lane, span)
        name_map[uid] = name
        cp = self._read_state()
        if not cp.has_section(sec):
            cp.add_section(sec)
        for k, v in self._ser_maps(lane_map, name_map).items():
            cp.set(sec, k, v)
        entries = [e.strip() for e in (self._state_get(sec, "roster") or ""
                                       ).split(",") if e.strip()]
        if not any(_norm_uid(e.partition(":")[2]) == uid for e in entries):
            entries.append(f"{model}:{uid}")
            cp.set(sec, "roster", ", ".join(entries))
        self._write_state(cp)
        self._drop_name_holders(self._name_map, self._lane_map, name, {uid})
        self._lane_map[uid] = (first_lane, span)
        self._name_map[uid] = name
        # A deliberate re-add (ASSIGN) lifts any live-forget suppression: the
        # operator is putting this uid back on purpose, so the scout should stop
        # treating it as forgotten and let the release/claim logic run normally.
        if hasattr(self, "_forget_suppressed"):
            self._forget_suppressed.discard(uid)

    def _drop_pin(self, uid: str) -> None:
        """
        Remove a uid's lane/name pin from the persisted maps (its roster
        enrolment and learned values are kept), and sync the in-memory maps.

        The unit reverts to a floating spare: it re-claims onto a free bay of
        its family: live, the one it was last claimed onto while that is free
        (see _claim_pool_unit), else the lowest; at the next restart, the
        lowest. Live, it is saved on the bay it holds once online for
        enroll_grace (see _pin_recorded_units); a restart saves the bay it
        draws. Its learned values follow it to whichever bay it claims. A uid a
        set roster: option does not list is never saved, and gets no bay at
        restart.

        :param uid: the unit's uid
        """
        self._keep_guessed_owners()
        sec = self._BASE_SECTION + " " + self.name
        lane_map = self._load_lane_map()
        name_map = self._load_name_map()
        lane_map.pop(uid, None)
        name_map.pop(uid, None)
        cp = self._read_state()
        if cp.has_section(sec):
            for k, v in self._ser_maps(lane_map, name_map).items():
                cp.set(sec, k, v)
            self._write_state(cp)
        self._lane_map.pop(uid, None)
        self._name_map.pop(uid, None)

    def cmd_AFC_BRIDGEBOX_ASSIGN(self, gcmd: Any) -> None:
        """
        Pin a detected unit's UID onto a chosen named pool bay, live.

        The bays are named by lane position (ams_names / ht_names, or the
        Bambu_AMS_# / Bambu_AMS_HT_# defaults). Auto-claim lands a fresh unit on the
        lowest free bay of its family; this command instead binds a specific uid to
        a specific named bay so it always comes up there. If the unit is online now
        it moves onto that bay immediately (lanes + T# re-registered, no restart);
        the pin is persisted so it holds across reboots. Same-family only: an HT
        bay is one lane, an AMS bay four. Moving a unit off a bay with a lane AFC
        records as loaded to a toolhead, claimed or not, or off an unclaimed bay
        before PREP has restored that record, is refused unless FORCE=1, which
        clears that record first. Without a pool nothing claims a bay, so an
        offline unit's lanes come up only when this runs again while it is online.
        With a roster: option set, only a uid it lists can be pinned: the option is
        the whole roster, so an unlisted uid has no bay at restart. A bay whose
        name is saved for another recorded uid is refused too: that uid gets the
        bay at restart. The uid's learned values go with it (see _apply_learned).
        UID alone pops the bay picker for a unit on a bay. A unit that found no
        free bay of its family is told so instead, and pointed at
        AFC_BRIDGEBOX_REPLACE while a bay of its family is held for a unit that is
        offline.

        Usage
        -------
        `AFC_BRIDGEBOX_ASSIGN [CHAIN=<chain>] UID=<unit uid> [NAME=<bay name>] [FORCE=1]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_ASSIGN UID=0123456789ABCDEF01234567 NAME=Bambu_AMS_2
        ```
        """
        uid = _norm_uid(gcmd.get("UID", ""))
        name = (gcmd.get("NAME", "") or "").strip()
        if uid and not name:
            # No target named: pop the bay-picker for this uid (the same dialog
            # a fresh plug-in raises) so the operator can click a destination.
            if self._bay_of_uid(uid) is None and uid in self._no_bay:
                family = "ht" if self._no_bay[uid] in _HT_MODELS_BB else "ams"
                fam = family.upper()
                built = any(pu.get("family") == family
                            for pu in getattr(self, "_pool_units", []))
                error_str = (
                    f"AFC_BRIDGEBOX_ASSIGN: {uid} is on the chain, but "
                    + (f"every {fam} bay is taken" if built
                       else f"chain {self.name} has no {fam} bay")
                    + (f" -- AFC_BRIDGEBOX_REPLACE CHAIN={self.name} "
                       f"UID={uid} gives it the bay of a unit that is offline"
                       if self._roster_source != "option"
                       and self._held_offline(family) else ""))
                raise gcmd.error(error_str)
            if self._bay_of_uid(uid) is None:
                error_str = (
                    f"AFC_BRIDGEBOX_ASSIGN: {uid} is not on any bay -- plug it "
                    f"in first, or give NAME=<bay name> to pin it")
                raise gcmd.error(error_str)
            self._prompt_new_unit(uid, timeout=180.0)   # manual: give time
            gcmd.respond_info(
                f"AFC_BridgeBox {self.name}: opened the bay picker for {uid}.")
            return
        if not uid or not name:
            error_str = "AFC_BRIDGEBOX_ASSIGN: give UID=<unit_uid> and NAME=<bay name>"
            raise gcmd.error(error_str)
        if self._unlisted(uid):
            error_str = (
                f"AFC_BRIDGEBOX_ASSIGN: {uid} is not in your roster: option. "
                f"While roster: is set it is the whole roster, so a unit it "
                f"does not list gets no bay of its own at restart and cannot be "
                f"pinned -- add {self._roster_entry(uid)} to roster: first, "
                f"RESTART, then assign it.")
            raise gcmd.error(error_str)
        pools = getattr(self, "_pool_units", [])
        target = next((p for p in pools if p.get("name") == name), None)
        if target is None:
            bays = ", ".join(sorted(p["name"] for p in pools)) or "none"
            error_str = (
                f"AFC_BRIDGEBOX_ASSIGN: no pool bay named {name!r} "
                f"(bays: {bays})")
            raise gcmd.error(error_str)
        # A uid not recorded yet goes by the model it waits with, else the
        # family of the bay it holds.
        model = self._model_for_uid(uid)
        fam = self._family_of_uid(uid)
        if fam is not None and fam != target["family"]:
            error_str = (
                f"AFC_BRIDGEBOX_ASSIGN: {uid} is an {fam.upper()} unit but bay "
                f"{name!r} is an {target['family'].upper()} bay (their lane "
                f"counts differ)")
            raise gcmd.error(error_str)
        owner = _norm_uid(target.get("uid"))
        if owner and owner != uid:
            error_str = (
                f"AFC_BRIDGEBOX_ASSIGN: bay {name!r} is already assigned to "
                f"{owner} -- AFC_BRIDGEBOX_UNASSIGN CHAIN={self.name} "
                f"UID={owner} first")
            raise gcmd.error(error_str)
        holder = self._name_holder(name, uid)
        if holder:
            error_str = (
                f"AFC_BRIDGEBOX_ASSIGN: bay {name!r} is saved for {holder} "
                f"-- AFC_BRIDGEBOX_FORGET CHAIN={self.name} UID={holder} or "
                f"AFC_BRIDGEBOX_UNASSIGN CHAIN={self.name} UID={holder} "
                f"first")
            raise gcmd.error(error_str)
        # Vacate the uid's current bay if it is sitting on a different one.
        cur = next((p for p in pools
                    if _norm_uid(p.get("uid")) == uid
                       and p is not target), None)
        unclaimed: List[str] = []
        if cur is not None and cur.get("bound"):
            self._refuse_toolhead_release(gcmd, "AFC_BRIDGEBOX_ASSIGN", uid)
        elif cur is not None:
            unclaimed = self._refuse_unclaimed_release(
                gcmd, "AFC_BRIDGEBOX_ASSIGN", cur, f"moves {uid}")
        cleared = self._clear_bay_toolhead(cur) if unclaimed else []
        if cur is not None:
            if cur.get("bound"):
                self._release_pool_unit(uid, "AFC_BRIDGEBOX_ASSIGN")
            cur["uid"] = None
        target["uid"] = uid
        first_lane = int("".join(c for c in target["lanes"][0] if c.isdigit()))
        span = len(target["lanes"])
        eff_model = model or ("ht" if target["family"] == "ht" else "boxed")
        self._persist_pin(uid, first_lane, span, name, eff_model)
        claimed = False
        online = not target.get("bound") and self._uid_online_now(uid)
        if online:
            if self._claim_pool_unit(uid, eff_model) is not None:
                claimed = True
        # Online and unclaimed only because PREP has not run (see
        # _prep_settled). The chain watch claims it once PREP has, but only
        # with a pool configured: it does no claiming without one.
        waits = online and not claimed and not self._prep_settled()
        pooled = bool(self.pool_ams or self.pool_ht)
        tools = (f"T{first_lane}" if span == 1
                 else f"T{first_lane}-T{first_lane + span - 1}")
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: assigned {uid} to bay {name!r} "
            f"(lane{first_lane}"
            + (f"-lane{first_lane + span - 1}" if span > 1 else "")
            + f", {tools})"
            + (f" -- cleared {', '.join(cleared)} from the toolhead"
               if cleared else "")
            + (" -- claimed LIVE, no restart." if claimed
               else " -- already claimed there." if target.get("bound") == uid
               else " -- pinned; it claims this bay once PREP finishes."
               if waits and pooled
               else " -- pinned; PREP has not finished, run this again once "
                    "it has." if waits
               else " -- pinned; it claims this bay when next online."
               if pooled
               else f" -- pinned; with no pool nothing claims it, so run this "
                    f"again once {uid} is online and PREP has finished, and "
                    f"after every restart."))
        self._close_prompt()          # a picker button ran this; close it

    def cmd_AFC_BRIDGEBOX_UNASSIGN(self, gcmd: Any) -> None:
        """
        Unlink a UID from its pool bay: the reverse of ASSIGN.

        Drops the name/lane pin so the unit reverts to a floating spare (it
        re-claims onto a free bay of its family, the one it was last claimed onto
        first, and is saved there once online for enroll_grace; see _drop_pin).
        Learned values, saved lane records and roster enrolment are kept, since this
        is not FORGET: learned values apply on whatever bay it claims next, and the
        lane records if that is its last bay. A unit unassigned live with FORCE=1
        has usually been online that long already, so it is re-homed and saved on
        the next watch tick; ASSIGN is the way to choose its bay. A uid a set
        roster: option does not list is never saved (see _pin_recorded_units): it
        floats for the session, and the reply says to add it to roster:. Refuses a
        unit that is online now unless FORCE=1 (which drops its lanes live first),
        and likewise a unit whose bay has a lane AFC records as loaded to a
        toolhead, claimed or not, or whose bay is unclaimed before PREP has
        restored that record (FORCE=1 clears the record first). The loaded-lane
        refusal comes first, so it names the lane.

        Usage
        -------
        `AFC_BRIDGEBOX_UNASSIGN [CHAIN=<chain>] UID=<unit uid>|NAME=<bay name> [FORCE=1]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_UNASSIGN NAME=Bambu_AMS_2 FORCE=1
        ```
        """
        uid = _norm_uid(gcmd.get("UID", ""))
        name = (gcmd.get("NAME", "") or "").strip()
        pools = getattr(self, "_pool_units", [])
        if not uid and name:
            pu = next((p for p in pools if p.get("name") == name), None)
            if pu is None:
                bays = ", ".join(sorted(p["name"] for p in pools)) or "none"
                error_str = (
                    f"AFC_BRIDGEBOX_UNASSIGN: no pool bay named {name!r} "
                    f"(bays: {bays})")
                raise gcmd.error(error_str)
            uid = _norm_uid(pu.get("uid"))
            if not uid:
                error_str = f"AFC_BRIDGEBOX_UNASSIGN: bay {name!r} has no unit assigned"
                raise gcmd.error(error_str)
        if not uid:
            error_str = "AFC_BRIDGEBOX_UNASSIGN: give UID=<unit_uid> or NAME=<bay name>"
            raise gcmd.error(error_str)
        pu = next((p for p in pools
                   if _norm_uid(p.get("uid")) == uid), None)
        if pu is None and uid not in self._lane_map and uid not in self._name_map:
            error_str = f"AFC_BRIDGEBOX_UNASSIGN: nothing assigned for uid {uid}"
            raise gcmd.error(error_str)
        # Ahead of the online refusal, so the refusal names the lane FORCE=1
        # clears.
        unclaimed: List[str] = []
        if pu is not None and pu.get("bound"):
            self._refuse_toolhead_release(gcmd, "AFC_BRIDGEBOX_UNASSIGN", uid)
        elif pu is not None:
            unclaimed = self._refuse_unclaimed_release(
                gcmd, "AFC_BRIDGEBOX_UNASSIGN", pu, f"unassigns {uid}")
        if not gcmd.get_int("FORCE", 0) and self._uid_online_now(uid):
            error_str = (
                f"AFC_BRIDGEBOX_UNASSIGN: {uid} is ONLINE on the chain right "
                f"now -- unhook it first, or FORCE=1 to unassign it live "
                f"(drops its lanes)")
            raise gcmd.error(error_str)
        cleared = self._clear_bay_toolhead(pu) if unclaimed else []
        was = pu.get("name") if pu is not None else self._name_map.get(uid)
        # Read while the uid still holds its bay, whose family names it.
        entry = self._roster_entry(uid) if self._unlisted(uid) else None
        dropped = False
        if pu is not None:
            if pu.get("bound"):
                self._release_pool_unit(uid, "AFC_BRIDGEBOX_UNASSIGN")
                dropped = True
            pu["uid"] = None
        self._drop_pin(uid)
        self._missing_since.pop(uid, None)
        pooled = bool(self.pool_ams or self.pool_ht)
        if entry and pooled:
            tail = (f"roster: is set and does not list it, so it takes a free "
                    f"bay of its family for this session only and is not "
                    f"saved there; add {entry} to roster: and RESTART to give "
                    f"it a bay of its own.")
        elif entry:
            tail = (f"roster: is set and does not list it, so it gets no bay "
                    f"at the next restart; add {entry} to roster: to give it "
                    f"one.")
        elif pooled:
            tail = (f"It takes a free bay of its family, its last one first, "
                    f"and is saved there once it has been online "
                    f"{self.enroll_grace:.0f}s.")
        else:
            tail = "It takes the lowest free bay of its family at the next restart."
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: unassigned {uid}"
            + (f" from bay {was!r}" if was else "")
            + (" -- lanes dropped live" if dropped else "")
            + (f" -- cleared {', '.join(cleared)} from the toolhead"
               if cleared else "")
            + " (learned values stay with the unit). " + tail)
        self._close_prompt()          # a manager button ran this; close it

    def cmd_AFC_BRIDGEBOX_BAYS(self, gcmd: Any) -> None:
        """
        Pop the bay manager, listing every pool bay with its occupant.

        Each occupied bay gets an Unassign button (FORCE=1, live), up to
        _MAX_BAY_BUTTONS. This is the manual entry point to the same unassign the
        plug/unplug popups lead to, callable any time.

        The Unassign button sends FORCE=1 so a live unit can be unlinked, and
        FORCE=1 also clears a lane AFC records as loaded to a toolhead, claimed or
        not, and skips the PREP wait for an unclaimed bay. So a bay with such a
        lane is listed with it and gets no button, and neither does an unclaimed
        bay before PREP has restored that record, nor, during a print, a bay a unit
        is claimed onto: unassigning it drops its lanes and T#s, which the print may
        be using. The typed command with FORCE=1 still releases each.

        A unit on the chain that found no free bay of its family is listed after
        the bays. While a bay of its family is held for a unit offline
        release_grace or longer (see _replace_candidates), it gets a Replace
        button, ahead of the Unassign buttons so the button cap never hides it,
        which opens the replace picker (see _prompt_replace_unit). A set roster:
        option decides which units get a bay, so it gets none then.

        Usage
        -------
        `AFC_BRIDGEBOX_BAYS [CHAIN=<chain>]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_BAYS
        ```
        """
        pools = sorted(getattr(self, "_pool_units", []), key=_pool_laneno)
        lines, buttons = [], []
        now = self._reactor_now()
        waiting = []
        for uid, model in sorted(self._no_bay.items()):
            fam = "ht" if model in _HT_MODELS_BB else "ams"
            waiting.append(f"Waiting: {uid} [{fam.upper()}] -- no free bay")
            if (self._roster_source != "option"
                and self._replace_candidates(fam, now)
                and len(buttons) < self._MAX_BAY_BUTTONS):
                buttons.append((
                    f"Replace for {uid[:8]}",
                    f"AFC_BRIDGEBOX_REPLACE CHAIN={self.name} UID={uid}",
                    "primary"))
        settled = self._prep_settled()
        printing = self._is_printing()
        for pu in pools:
            uid = _norm_uid(pu.get("uid"))
            live = " (live)" if pu.get("bound") else ""
            fam = "HT" if pu.get("family") == "ht" else "AMS"
            held = self._bay_loaded(pu) if uid else []
            note = ""
            if held and pu.get("bound"):
                note = (f" -- {', '.join(held)} in the toolhead, unload it "
                        f"before unassigning")
            elif held:
                note = (f" -- {', '.join(held)} in the toolhead from {uid}, "
                        f"which is not claimed: plug it back in and unload "
                        f"it before unassigning")
            elif pu.get("bound") and printing:
                note = " -- a print is active, unassign it once it ends"
            elif uid and not pu.get("bound") and not settled:
                note = " -- PREP has not run yet"
            lines.append(f"{pu['name']} [{fam}]: "
                         + (f"{uid}{live}" if uid else "free") + note)
            if uid and not note and len(buttons) < self._MAX_BAY_BUTTONS:
                buttons.append((
                    f"Unassign {pu['name']}",
                    f"AFC_BRIDGEBOX_UNASSIGN CHAIN={self.name} UID={uid} FORCE=1",
                    "warning"))
        if not lines:
            lines = ["No pool units configured (set pool_ams / pool_ht)."]
        lines += waiting
        # A manually-opened manager stays up longer than an event popup.
        self._prompt("Bambu AMS Units", lines, buttons, timeout=180.0)
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: bay manager --\n  "
            + "\n  ".join(lines))

    def cmd_AFC_BRIDGEBOX_REPLACE(self, gcmd: Any) -> None:
        """
        Put a unit that found no free bay onto the bay of a unit that is offline.

        Runs live: AFC_BRIDGEBOX_FORGET of the offline unit, then
        AFC_BRIDGEBOX_ASSIGN of the new one to the bay that frees. Each runs as its
        own handler (see _ChildCommand), so their refusals and console lines apply
        unchanged. The new unit takes the bay's name, lanes and T#, pinned so a
        restart keeps it there. FORGET erases what was saved for the old unit, so
        the new one gets none of its lane records, spools, T# maps or learned
        values.

        Every refusal of its own is made before anything changes, and FORCE=1
        overrides none of these: a chain with no pool (nothing is claimed live
        there), a set roster: option (it decides which units get a bay), a new
        unit on a bay already (ASSIGN moves a unit between bays), a bay of the
        other family, a free or unknown bay, a bay another recorded unit is saved
        on, and an old unit that is online or whose online flag cannot be read
        (the bridge link is down, or has not reported the chain yet). Without
        FORCE=1 it also refuses during a print, before PREP has run (AFC's
        loaded-lane record is not restored yet, see _prep_settled), a bay with a
        lane AFC records as loaded to a toolhead, and a bay whose unit has not been
        offline release_grace on a live chain (see _track_absence). FORCE=1 clears
        that loaded-lane record.

        UID defaults to the one unit waiting for a bay. OLD takes the old unit's
        uid or its bay name; without OLD, pops the replace picker (see
        _prompt_replace_unit).

        Usage
        -------
        `AFC_BRIDGEBOX_REPLACE [CHAIN=<chain>] [UID=<new uid>] [OLD=<old uid or bay>] [FORCE=1]`

        Example
        -------
        ```
        AFC_BRIDGEBOX_REPLACE OLD=Bambu_AMS_2
        ```
        """
        cmd = "AFC_BRIDGEBOX_REPLACE"
        new = _norm_uid(gcmd.get("UID", ""))
        old_arg = (gcmd.get("OLD", "") or "").strip()
        force = gcmd.get_int("FORCE", 0)
        pooled = bool(self.pool_ams or self.pool_ht)
        pools = getattr(self, "_pool_units", None) or []
        if self._roster_source == "option":
            error_str = (
                f"{cmd}: roster: is set, so it decides which units get a bay. "
                f"To swap an offline unit for {new or 'the new one'}, change "
                f"its entry to "
                f"{self._roster_entry(new) if new else '<model>:<new uid>'} "
                f"in roster:, run AFC_BRIDGEBOX_FORGET CHAIN={self.name} "
                f"UID=<old uid>"
                + (" (its bay frees now and the new unit claims it live)"
                   if pooled else "")
                + ", then RESTART.")
            raise gcmd.error(error_str)
        if not pooled or not pools:
            error_str = (
                f"{cmd}: chain {self.name} has no pool bays, so no unit is put "
                f"on a bay live -- AFC_BRIDGEBOX_FORGET CHAIN={self.name} "
                f"UID=<old uid> and RESTART; the new unit, once recorded, "
                f"takes the bay that frees")
            raise gcmd.error(error_str)
        if not new:
            waiting = sorted(self._no_bay)
            if len(waiting) != 1:
                error_str = (
                    f"{cmd}: give UID=<new uid> -- "
                    + (f"waiting for a bay: {', '.join(waiting)}" if waiting
                       else "no unit is waiting for a bay"))
                raise gcmd.error(error_str)
            new = waiting[0]
        fam = self._family_of_uid(new)
        if fam is None:
            error_str = f"{cmd}: {new} is not on chain {self.name}"
            raise gcmd.error(error_str)
        # A unit on a bay would be moved off it, and the old unit forgotten
        # for good, by one mistyped uid.
        cur = self._bay_of_uid(new)
        if cur is not None:
            error_str = (
                f"{cmd}: {new} is on bay {cur['name']!r}, and only a unit "
                f"with no bay takes over another's -- AFC_BRIDGEBOX_ASSIGN "
                f"CHAIN={self.name} UID={new} NAME=<bay name> moves a unit "
                f"between bays")
            raise gcmd.error(error_str)
        now = self._reactor_now()
        if not old_arg:
            if self._prompt_replace_unit(new, now, timeout=180.0):
                gcmd.respond_info(f"AFC_BridgeBox {self.name}: opened the "
                                  f"replace picker for {new}.")
            else:
                gcmd.respond_info(f"AFC_BridgeBox {self.name}: "
                                  + self._no_candidate_text(new, fam, now))
            return
        target = (next((pu for pu in pools if pu.get("name") == old_arg), None)
                  or self._bay_of_uid(old_arg))
        if target is None:
            bays = ", ".join(pu["name"] for pu in sorted(pools,
                                                         key=_pool_laneno))
            error_str = (f"{cmd}: no pool bay is named or held for "
                         f"{old_arg} (bays: {bays})")
            raise gcmd.error(error_str)
        bay = target["name"]
        if target.get("family") != fam:
            error_str = (
                f"{cmd}: {new} is an {fam.upper()} unit but bay {bay!r} is an "
                f"{target['family'].upper()} bay (their lane counts differ)")
            raise gcmd.error(error_str)
        old = _norm_uid(target.get("uid") or target.get("bound"))
        if not old:
            error_str = (
                f"{cmd}: bay {bay!r} is free -- AFC_BRIDGEBOX_ASSIGN "
                f"CHAIN={self.name} UID={new} NAME={bay} puts {new} on it")
            raise gcmd.error(error_str)
        # ASSIGN refuses a bay name saved for another recorded unit; checked
        # here, before FORGET has run.
        holder = self._name_holder(bay, new, self._recorded_uids() - {old})
        if holder:
            error_str = (
                f"{cmd}: bay {bay!r} is saved for {holder} too -- "
                f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} UID={holder} or "
                f"AFC_BRIDGEBOX_UNASSIGN CHAIN={self.name} UID={holder} "
                f"first")
            raise gcmd.error(error_str)
        # With the link down, or before the bridge's first status, every
        # unit reads offline.
        bridge = self._chain_bridge() or getattr(self, "_bridge", None)
        link_up = getattr(bridge, "_serial", None) is not None
        try:
            status = bridge.latest_status() if link_up else None
        except Exception:
            status = None
        if status is None:
            error_str = (
                f"{cmd}: cannot tell whether {old} is offline: "
                + ("the bridge has not reported the chain yet -- run this "
                   "again once it has" if link_up
                   else "the bridge link is down -- run this again once it "
                        "reconnects"))
            raise gcmd.error(error_str)
        if self._uid_online_now(old):
            error_str = (
                f"{cmd}: {old} on bay {bay!r} is online, and only a unit that "
                f"is offline is replaced -- unplug it first, or "
                f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} UID={old} forgets it "
                f"and frees the bay now")
            raise gcmd.error(error_str)
        loaded = self._bay_loaded(target)
        if not force:
            if self._is_printing():
                error_str = (f"{cmd}: a print is active -- finish it, or "
                             f"FORCE=1 to replace {old} anyway")
                raise gcmd.error(error_str)
            if not self._prep_settled():
                error_str = (
                    f"{cmd}: PREP has not run yet, so which lane AFC records "
                    f"as loaded to the toolhead is not known -- run this "
                    f"again once it has")
                raise gcmd.error(error_str)
            if loaded and target.get("bound"):
                error_str = (
                    f"{cmd}: AFC records {', '.join(loaded)} on {bay} as "
                    f"loaded to the toolhead -- unload it first "
                    f"({self._unset_hint(loaded)} if the filament is already "
                    f"out), or FORCE=1 to clear it from the toolhead and "
                    f"replace anyway")
                raise gcmd.error(error_str)
            if loaded:
                error_str = self._unclaimed_loaded_text(
               cmd, bay, old, loaded, f"replaces {old}")
                raise gcmd.error(error_str)
            since = self._missing_since.get(old)
            if since is None:
                error_str = (
                    f"{cmd}: no absence is counted for {old} -- it counts "
                    f"only while the bridge link is up and a unit on the "
                    f"chain is online. FORCE=1 replaces it anyway")
                raise gcmd.error(error_str)
            if now - since < self.release_grace:
                error_str = (
                    f"{cmd}: {old} has been offline {now - since:.0f}s, under "
                    f"release_grace ({self.release_grace:.0f}s) -- wait, or "
                    f"FORCE=1 to replace it now")
                raise gcmd.error(error_str)
        cleared = (self._clear_bay_toolhead(target)
                   if loaded and not target.get("bound") else [])
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: replacing {old} on {bay} with {new}"
            + (f"; cleared {', '.join(cleared)} from the toolhead"
               if cleared else "") + ".")
        self.cmd_AFC_BRIDGEBOX_FORGET(_ChildCommand(gcmd, UID=old,
                                                    FORCE=force))
        self._no_bay.pop(new, None)
        self._no_bay_told.discard(new)
        self._replace_offered.discard(new)
        q = getattr(self, "_popup_queue", None)
        if q:
            q[:] = [e for e in q if (e[0], e[1]) != ("replace", new)]
        try:
            self.cmd_AFC_BRIDGEBOX_ASSIGN(_ChildCommand(
                gcmd, UID=new, NAME=bay, FORCE=force))
        except Exception as e:
            # ASSIGN pins the bay before it claims, so a failed claim leaves
            # the pin.
            if _norm_uid(target.get("uid")) == new:
                error_str = (
                    f"{cmd}: {old} is forgotten and bay {bay!r} is pinned to "
                    f"{new}, but {new} is not claimed onto it ({e}). The "
                    f"chain watch claims it there while {new} is on the "
                    f"chain.")
                raise gcmd.error(error_str)
            error_str = (
                f"{cmd}: {old} is forgotten and bay {bay!r} is free, but {new} "
                f"is not on it ({e}). While the bay is free, {new} claims it "
                f"once it is on the chain.")
            raise gcmd.error(error_str)

    def _chain_bridge(self) -> Any:
        """
        The bridge registered for our serial port, if any.

        :return Any: the BambuBridge, or None when none is registered
        """
        from extras import AFC_BambuAMS_bridge as _bridge_mod
        return _bridge_mod._BRIDGES.get(self.serial_port)

    def _uid_online_now(self, uid: str) -> bool:
        """
        Whether a unit is answering on the chain right now.

        :param uid: 24-hex unit uid
        :return bool: True when that unit is online
        """
        try:
            bridge = (self._chain_bridge()
                      or getattr(self, "_bridge", None))
            if bridge is None or getattr(bridge, "_serial", None) is None:
                return False
            latest = bridge.latest_status() or {}
            online_idx = {u.get("n") for u in (latest.get("units") or [])
                          if u.get("online")}
            return any(i in online_idx
                       and _norm_uid(u) == uid
                       for i, u in enumerate(bridge.chain_uids()))
        except Exception:
            return False

    def _bridge_forget(self, uid: str) -> None:
        """
        Tell the bridge firmware to forget a uid: stop re-asserting it on the bus
        so a gone-but-still-enrolled unit stops reading its own re-assert back and
        flapping online.

        Best-effort; the bridge re-learns the unit on a real re-announce, and
        firmware without the command ignores it.

        :param uid: 24-hex unit uid
        """
        try:
            bridge = (self._chain_bridge()
                      or getattr(self, "_bridge", None))
            if bridge is not None:
                bridge.send({"cmd": "forget",
                             "uid": (uid or "").strip().lower()})
        except Exception:
            pass

    @staticmethod
    def _read_ini(path: str) -> Optional[configparser.RawConfigParser]:
        """
        Read an INI file.

        :param path: INI file to read
        :return RawConfigParser: parser, or None when absent/unreadable
        """
        if not os.path.exists(path):
            return None
        cp = configparser.RawConfigParser(delimiters=(":", "="))
        try:
            with open(path) as fp:
                cp.read_file(fp)
        except Exception:
            return None
        return cp

    def _fold_and_sweep(
            self, config: Any,
            sections: List[Tuple[str, Dict[str, Any]]]
    ) -> List[Tuple[str, Dict[str, Any]]]:
        """
        Close the loop between fabricated sections and AFC's learned values.

        ConfigRewrite files a learned key whose section is in no .cfg file
        into AFC_auto_vars.cfg under the section's name, and that file is
        klippy-parsed config. For a fabricated unit such a value would be
        invisible (its sections come from a private parser) and, once the
        fabricated name changed, klippy would instantiate a unit from the
        leftover at boot. This handles both:

        FOLD: a unit section takes the learned values of the uid it is
        built for: its record (see _learned_section), then the
        _LEARNED_KEYS of an auto_vars section under its name, newest last.
        Those go back into the uid's record. A spare is built for no uid, so
        it takes none, and any other key an auto_vars unit section carries is
        not folded; both are noted (see _learned_notes). Hub, lane and
        temperature sections take the store and auto_vars sections under
        their own names (structural keys excepted), so a value learned for a
        bay's lanes survives a restart there.

        SWEEP: folded auto_vars sections move into this module's own store
        file, which klippy never parses, and are deleted from auto_vars. An
        orphan, an AFC_BambuAMS section in auto_vars whose name nothing
        fabricates and which exists in no other config file (its key set in
        the merged fileconfig equals its key set in auto_vars alone), is
        deleted too: it is a leftover for a renamed unit, whose learned
        values travel under its uid. A lane, hub or temperature section this
        chain may have built at an earlier start (see _former_names) and no
        chain builds now is a leftover as well: it moves into the store
        under its own name, where the fold finds it if that name is built
        again. With several chains, only the last one in the config sweeps
        orphans and leftovers, past the names the others fabricate (see
        _sweep_orphans).

        File edits take effect next boot (this boot already parsed
        auto_vars). For the rest of this boot the fail-soft unit makes an
        orphan harmless, and a stand-in object keeps klippy from building a
        leftover (see _sweep_orphans).

        :param config: this section's wrapper (for the merged fileconfig)
        :param sections: fabricated (name, keys) pairs
        :return list: the same pairs with learned values folded in
        """
        fabricated = {name for name, _keys in sections}
        store = self._read_state()
        autov = self._read_ini(self.auto_vars_file)
        overrides = self._override_sections(config)
        # Kept so a model override can be applied once a claim reveals the
        # model (see _apply_model_measure and _reapply_model_override).
        self._overrides = overrides

        folded: List[Tuple[str, Dict[str, Any]]] = []
        updates: Dict[str, Dict[str, Any]] = {}
        for name, keys in sections:
            merged = dict(keys)
            target = name                     # where the learned diff is kept
            if name.startswith(self._UNIT_SECTION + " "):
                # A bay is worn by whichever unit claims it; what a unit
                # learned belongs to its uid (see persist_learned).
                uid = _norm_uid(keys.get("unit_uid"))
                record = self._learned_for(uid, store) if uid else {}
                merged.update(record)
                if autov is not None and autov.has_section(name):
                    got = dict(autov.items(name))
                    kept = ({k: v for k, v in got.items()
                             if k in _LEARNED_KEYS
                                and _learned_length(v) is not None}
                            if uid else {})
                    merged.update(kept)       # auto_vars newest, wins last
                    left = sorted(set(got) - set(kept))
                    if left and uid:
                        self._note_learned(
                            f"auto_vars [{name}]: {', '.join(left)} not "
                            f"folded -- a unit takes only its bowden lengths "
                            f"from there, each a positive number")
                    elif left:
                        self._note_learned(
                            f"auto_vars [{name}] swept, not folded -- the "
                            f"bay is a spare, and learned values belong to "
                            f"the unit that learned them")
                target = self._learned_section(uid) if uid else None
                learned = {k: merged[k] for k in _LEARNED_KEYS
                           if k in merged and keys.get(k) != merged[k]}
                if learned == record:
                    target = None             # the record already holds it
            else:
                for src in (store, autov):    # auto_vars newest, wins last
                    if src is not None and src.has_section(name):
                        for k, v in src.items(name):
                            if k not in _STRUCTURAL_KEYS:
                                merged[k] = v
                # Learned = whatever differs from the fabricated defaults, so
                # the state keeps it after the auto_vars copy is swept.
                learned = {k: v for k, v in merged.items()
                           if keys.get(k) != v}
            # Computed before operator overrides, which must never be persisted
            # as learned. Overrides overlay last (model-wide, then per-unit),
            # identity keys still protected (see _override_sections).
            ov: Dict[str, Any] = {}
            # Chain-wide first, so model and unit sections both override it.
            ov.update(getattr(self, "_chain_defaults", None) or {})
            if keys.get("ams_model"):
                mkey = "model:" + _norm_model(str(keys["ams_model"]))
                ov.update(overrides.get(mkey) or {})
            ov.update(overrides.get(name) or {})
            for k, v in ov.items():
                if k not in _STRUCTURAL_KEYS:
                    merged[k] = v
            folded.append((name, merged))
            if learned and target:
                updates[target] = learned

        # Remember override targets that name nothing known (likely typos) for
        # _scout_ready to report, as the sections the operator wrote. A model
        # section with no such unit plugged in is valid.
        known = {"model:" + m for m in _SLOTS_BY_MODEL}
        known |= {name for name, _keys in sections}
        self._unmatched_overrides = sorted(
            sec for target in set(overrides) - known
            for sec in self._override_written[target])

        self._fabricated_names = fabricated
        self._leftover_names = self._former_names(store) - fabricated
        changed = False
        if autov is not None:
            for name in list(autov.sections()):
                if name in fabricated:
                    autov.remove_section(name)
                    changed = True
        changed = self._sweep_orphans(config, autov) or changed
        # A freshly computed auto base is remembered alongside the learned
        # values, so it survives config growth.
        if getattr(self, "_resolved_base", None):
            updates.setdefault(self._BASE_SECTION + " " + self.name, {})[
                "lane_base"] = self._resolved_base
        if changed and autov is not None:
            self._write_auto_vars(autov)
        if updates:
            self._state_set(updates)
        return folded

    def _former_names(self, store: configparser.RawConfigParser) -> Set[str]:
        """
        The lane, hub and temperature sections this chain may have built at
        an earlier start.

        Those are its lanes from lane_base through as many AMS bays as
        pool_ams asks (uncapped, as an earlier start built them) or the
        roster lists, and as many HT lanes as pool_ht asks or the roster
        lists; every span the lane map held at this start; the hub and
        temperature sections of the names its bays wore or take by rank; and
        the sections of those kinds its store keeps.

        :param store: this chain's state, as read
        :return set: section names
        """
        units = getattr(self, "units", None) or []
        n_ht = sum(1 for u in units if u.get("model") in _HT_MODELS)
        ams = max(getattr(self, "_pool_ams_asked", 0), len(units) - n_ht)
        hts = max(self.pool_ht, n_ht)
        nums = set(range(self.lane_base, self.lane_base + ams * 4 + hts))
        for first, span in (getattr(self, "_lanes_at_boot", None)
                            or {}).values():
            nums |= set(range(first, first + span))
        bays = set((getattr(self, "_pins_at_boot", None) or {}).values())
        bays |= {self._name_for_index("ams", r) for r in range(ams)}
        bays |= {self._name_for_index("ht", r) for r in range(hts)}
        kinds = ("AFC_hub", "temperature_sensor")
        names = {f"AFC_lane lane{n}" for n in nums}
        names |= {f"{kind} {bay}" for bay in bays for kind in kinds}
        names |= {sec for sec in store.sections()
                  if sec.split(" ", 1)[0] in ("AFC_lane",) + kinds}
        return names

    def _sweep_orphans(self, config: Any,
                       autov: Optional[configparser.RawConfigParser]) -> bool:
        """
        Delete the orphans and leftovers from auto_vars (see
        _fold_and_sweep).

        Chains on one printer share auto_vars, and one further down the
        config has not fabricated its units yet, so its unit sections look
        like orphans here. Only the last chain master sweeps, whatever it
        fabricates, and it keeps the names every chain fabricated.

        A leftover moves into the store of the chain that may have built it.
        klippy parsed it at this start and builds every section it parsed
        that no object holds yet, and an [AFC_lane] with no unit would stop
        Klipper from starting, so a stand-in object takes the leftover's
        name and its options are read, which keeps klippy from reporting
        them unused.

        :param config: this section's wrapper (for the merged fileconfig)
        :param autov: auto_vars as read, edited in place; None when absent
        :return bool: whether a section was deleted
        """
        if autov is None or self._later_chains(config):
            return False
        chains = self._earlier_chains() + [self]
        kept: Set[str] = set()
        for m in chains:
            kept |= getattr(m, "_fabricated_names", None) or set()
        fc = getattr(config, "fileconfig", None)
        changed = False
        for name in list(autov.sections()):
            if name in kept:
                continue
            owner = next((m for m in chains if name in
                          (getattr(m, "_leftover_names", None) or ())), None)
            if (owner is None
                and not name.startswith(self._UNIT_SECTION + " ")):
                continue
            keys = dict(autov.items(name))
            try:
                in_real_cfg = bool(
                    fc is not None
                    and fc.has_section(name)
                    and set(dict(fc.items(name))) - set(keys))
            except Exception:
                in_real_cfg = True            # cannot prove orphan: keep it
            if in_real_cfg:
                continue
            autov.remove_section(name)
            changed = True
            if owner is None:
                continue
            owner._state_set({name: keys})
            self._stand_in(config, name, keys)
            owner._note_learned(
                f"auto_vars [{name}] moved to "
                f"{os.path.basename(owner.state_file)}: no chain builds that "
                f"section now")
        return changed

    def _stand_in(self, config: Any, name: str, keys: Dict[str, str]
                  ) -> None:
        """
        Hold a swept leftover's name for the rest of this start (see
        _sweep_orphans).

        :param config: this section's wrapper
        :param name: the section name
        :param keys: its options, read so klippy finds them used
        """
        try:
            if self.printer.lookup_object(name, None) is not None:
                return
            self.printer.add_object(name, _SweptSection())
        except Exception:
            return
        try:
            section = config.getsection(name)
            for opt in keys:
                section.get(opt, None)
        except Exception:
            pass

    def _write_auto_vars(self, autov: configparser.RawConfigParser) -> None:
        """
        Write auto_vars back after this module deleted sections from it.

        :param autov: auto_vars as edited
        """
        try:
            with open(self.auto_vars_file, "w") as fp:
                fp.write("# This file is autogenerated and updated when "
                         "variables are not in your normal AFC config "
                         "files\n\n")
                autov.write(fp)
        except Exception:
            pass                              # persistence is best-effort

    def _register_scout_chip(self, config: Any) -> None:
        """
        Register the bambu_buffer pin chip with no unit behind it.

        Same registry and dedupe the units use, so an enrolled boot's real
        chip and a scout boot's stub can never coexist or double-register.
        The stub is marked, so a unit a later chain fabricates on the same
        chip takes it over and the buffer reads that unit's bus.

        :param config: this section's wrapper (unused beyond symmetry)
        """
        import types as _types
        from extras.AFC_BambuAMS import _register_bambu_buffer_chip
        shim = _types.SimpleNamespace(
            printer=self.printer,
            buffer_chip_name=self.buffer_chip_name,
            fps_buffer_value=lambda: None,
            scout_stub=True)
        _register_bambu_buffer_chip(shim)

    # ── scouting: the chain names its own roster ─────────────────────────────

    def _locate_own_file(self, name: Optional[str] = None
                         ) -> Optional[str]:
        """
        The .cfg file declaring this [AFC_BridgeBox <name>] section.

        Searched under the main config file's directory, because that is
        where include chains live. First match wins; None when the section
        cannot be found (odd layouts), and the caller falls back to a
        default path rather than guessing.

        :param name: another chain's name, to find its file instead
        :return str: absolute path, or None
        """
        try:
            import glob as _glob
            import re as _re
            cfg = (getattr(self.printer, "start_args", {}) or {}).get(
                "config_file")
            if not cfg:
                return None
            root = os.path.dirname(os.path.abspath(cfg))
            pat = _re.compile(
                r"^[ \t]*\[[ \t]*AFC_BridgeBox[ \t]+"
                + _re.escape(name or self.name) + r"[ \t]*\]",
                _re.I | _re.M)
            for path in sorted(_glob.glob(os.path.join(root, "**", "*.cfg"),
                                          recursive=True)):
                try:
                    with open(path) as fp:
                        if pat.search(fp.read()):
                            return path
                except Exception:
                    continue
        except Exception:
            pass
        return None

    _DEFAULT_STATE = "~/printer_data/config/AFC/AFC_BridgeBox.cfg"
    _STATE_MARK = ("#~# --- AFC_BridgeBox managed state -- "
                   "everything below is auto-written ---")
    _STATE_PREFIX = "#~# "

    def _read_state(self, path: Optional[str] = None
                    ) -> configparser.RawConfigParser:
        """
        The managed block of the state file, decommented and parsed.

        :param path: another chain's state file, to read it instead
        :return RawConfigParser: parser over the state (empty when absent)
        """
        cp = configparser.RawConfigParser(delimiters=(":", "="))
        try:
            with open(path or self.state_file) as fp:
                text = fp.read()
        except Exception:
            return cp
        if self._STATE_MARK not in text:
            return cp
        block = text.split(self._STATE_MARK, 1)[1]
        lines = [ln[len(self._STATE_PREFIX):] if
                 ln.startswith(self._STATE_PREFIX) else ln.lstrip("#~ ")
                 for ln in block.splitlines() if ln.startswith("#~#")]
        try:
            cp.read_string("\n".join(lines))
        except Exception:
            return configparser.RawConfigParser(delimiters=(":", "="))
        return cp

    def _write_state(self, cp: configparser.RawConfigParser) -> None:
        """
        Rewrite the managed block, preserving everything the operator wrote
        above the marker, their [AFC_BridgeBox] section included.

        :param cp: the state to serialize
        """
        try:
            head = ""
            try:
                with open(self.state_file) as fp:
                    head = fp.read().split(self._STATE_MARK, 1)[0]
            except Exception:
                pass
            if not head.strip():
                head = ("# AFC_BridgeBox: put your [AFC_BridgeBox <name>] "
                        "section here (serial_port + extruder is enough).\n"
                        "# The block below is maintained by the module.\n\n")
            import io
            buf = io.StringIO()
            cp.write(buf)
            body = "".join(self._STATE_PREFIX + ln + "\n"
                           for ln in buf.getvalue().splitlines())
            with open(self.state_file, "w") as fp:
                fp.write(head.rstrip("\n") + "\n\n"
                         + self._STATE_MARK + "\n" + body)
        except Exception:
            pass                              # persistence is best-effort

    def _state_get(self, section: str, key: str) -> Optional[str]:
        """
        Read one value from the state file.

        :param section: state section name
        :param key: key within it
        :return str: the value, or None
        """
        cp = self._read_state()
        if cp.has_section(section):
            return dict(cp.items(section)).get(key)
        return None

    def _state_set(self, updates: Dict[str, Dict[str, Any]]) -> None:
        """
        Merge updates into the state block and write it back.

        :param updates: {section: {key: value}}
        """
        cp = self._read_state()
        for section, keys in updates.items():
            if not cp.has_section(section):
                cp.add_section(section)
            for k, v in keys.items():
                cp.set(section, k, str(v))
        self._write_state(cp)

    def _learned_section(self, uid: str) -> str:
        """
        The state section holding what a unit learned on this chain.

        Keyed by uid, not by bay: a bay is worn by one unit after another, and a
        bowden length is the path of the unit that measured it. Scoped by chain,
        since the path runs to the chain's own extruder.

        :param uid: a unit uid
        :return str: the section name
        """
        return f"{self._BASE_SECTION} {self.name} learned {_norm_uid(uid)}"

    def _learned_for(self, uid: str,
                     cp: Optional[configparser.RawConfigParser] = None
                     ) -> Dict[str, str]:
        """
        A uid's learned record.

        Only _LEARNED_KEYS holding a length are returned (see _learned_length): the
        fold writes them into a unit section.

        :param uid: a unit uid
        :param cp: the parsed state; read when not given
        :return dict: the learned record, {} when it has none
        """
        if not _norm_uid(uid):
            return {}
        cp = self._read_state() if cp is None else cp
        sec = self._learned_section(uid)
        if not cp.has_section(sec):
            return {}
        return {k: v for k, v in cp.items(sec)
                if k in _LEARNED_KEYS and _learned_length(v) is not None}

    def _note_learned(self, text: str, warn: bool = False) -> None:
        """
        Keep a line about learned values for _scout_ready to log, since
        there is no logger while the sections are built.

        :param text: the line, without the chain prefix
        :param warn: True to log it as a warning
        """
        notes = getattr(self, "_learned_notes", None)
        if notes is None:
            notes = self._learned_notes = []
        notes.append((warn, text))

    def persist_learned(self, unit_name: str, key: str, value: Any) -> bool:
        """
        Record one value a unit learned about itself, so it survives a restart.

        ConfigRewrite would file the key under [AFC_BambuAMS <name>] in
        AFC_auto_vars.cfg, which klippy parses; a pool-fabricated unit has no
        such section in any .cfg, so at next boot check_unused_options would
        reject the orphan and halt. This writes to the state file instead,
        which klippy never parses.

        The value is filed under the uid bound to the bay named
        ``unit_name`` (see _learned_section), not under the bay, so it goes
        wherever that unit goes: _fold_and_sweep lays it over the unit's
        section at boot and _apply_learned gives it to the unit at every
        claim. A bay with no unit bound, or whose unit object carries another
        uid, saves nothing; the unit keeps the value for this session.

        Only _LEARNED_KEYS are saved. Anything else is refused, structural
        keys included: those define what a unit is and are owned by the
        fabricator.

        :param unit_name: the fabricated unit's name, e.g. the value of the
          unit's own ``name``
        :param key: option to persist, one of _LEARNED_KEYS
        :param value: value to persist (stringified)
        :return bool: True if it was written
        """
        if not unit_name or not key:
            return False
        if key not in _LEARNED_KEYS:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: refusing to persist '{key}' for "
                f"{unit_name} -- only {', '.join(_LEARNED_KEYS)} are saved")
            return False
        pu = next((p for p in getattr(self, "_pool_units", [])
                   if p.get("name") == unit_name), None)
        uid = _norm_uid(pu.get("bound")) if pu is not None else ""
        try:
            unit = self.printer.lookup_object(
                f"{self._UNIT_SECTION} {unit_name}", None)
        except Exception:
            unit = None
        claimed = _norm_uid(getattr(unit, "unit_uid", None))
        if not uid or (claimed and claimed != uid):
            self.logger.info(
                f"AFC_BridgeBox {self.name}: {key} for {unit_name} kept for "
                f"this session only -- "
                + (f"the bay is bound to {uid} but its unit carries {claimed}"
                   if uid else "no unit is bound to the bay"))
            return False
        section = self._learned_section(uid)
        try:
            cur = self._read_state()
            if cur.has_section(section) and cur.has_option(section, key):
                if str(cur.get(section, key)) == str(value):
                    return False          # unchanged; no rewrite
            self._state_set({section: {key: value}})
            return True
        except Exception as ex:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: could not persist {key} for "
                f"{unit_name}: {ex}")
            return False

    def _unlisted_floaters(self, before: Dict[str, str]
                           ) -> Dict[str, List[str]]:
        """
        The units roster: leaves out that ran on its spare bays: each uid the
        recorded roster lists that roster: does not, with no name recorded
        before this boot or drawn at it.

        A build before bay_owner claimed such a unit onto a spare of its family
        every session.

        :param before: the name map as loaded, before this boot drew names
        :return dict: family ("ams" / "ht") -> uids, in recorded order; {}
            unless the roster is the roster: option
        """
        if getattr(self, "_roster_source", None) != "option":
            return {}
        raw = self._state_get(self._BASE_SECTION + " " + self.name,
                              "roster") or ""
        listed = {_norm_uid(u.get("uid"))
                  for u in getattr(self, "units", None) or []}
        named = set(before) | set(getattr(self, "_name_map", None) or {})
        out: Dict[str, List[str]] = {}
        for entry in raw.split(","):
            model, _sep, uid = entry.strip().partition(":")
            model, uid = model.strip().lower(), _norm_uid(uid)
            if (not uid
                or model not in _SLOTS_BY_MODEL
                or uid in listed
                or uid in named):
                continue
            fam = "ht" if model in _HT_MODELS_BB else "ams"
            if uid not in out.setdefault(fam, []):
                out[fam].append(uid)
        return out

    def _migrate_name_learned(
            self, before: Optional[Dict[str, str]] = None) -> None:
        """
        Move learned values stored under a bay name into the uid records.

        A state section [AFC_BambuAMS <name>] holds what the unit wearing
        that name learned. For a name this chain uses (a bay it fabricates,
        or a name its map records), the section goes to the uid that wore
        the name: the _LEARNED_KEYS its record lacks are copied (a length
        the record holds wins), anything that is not a length is dropped,
        and the section is removed.

        The wearer is the uid the name map records for the name before this
        boot drew names, so a uid that takes the name over at this boot
        never gets what an earlier holder learned. Only when no uid was
        recorded with the name is it the uid that draws it now: a unit the
        last session recorded but never saved. With roster: set, the values
        under a spare's name that no unit draws go to the one unit of its
        family that the recorded roster lists and roster: does not (see
        _unlisted_floaters), when that is the only such unit and this the
        only such spare with values: a build before bay_owner ran it on that
        spare. Both are guesses
        (the unit may have learned on another bay), and a wrong length
        corrects itself on the unit's next load, which re-adopts a
        measurement PATH_ADOPT_TOLERANCE_MM or more away.

        With several uids recorded for the name, the one holding the
        rostered bay of that name is the owner. If that does not settle it,
        the section is left in place and never read while one of them still
        holds the name, and removed once another unit has taken the name
        over. Any other name recorded for no uid is a spare's leftover, and
        its section is removed: folding it would hand those values to the
        next unit on the bay. Names this chain does not use are another chain's
        and are left alone. Writes once, only when something changed.

        :param before: the name map as loaded, before this boot drew names;
            the current map when not given
        """
        cp = self._read_state()
        pools = getattr(self, "_pool_units", [])
        before = self._name_map if before is None else before
        ours = ({pu["name"] for pu in pools} | set(self._name_map.values())
                | set(before.values()))
        prefix = self._UNIT_SECTION + " "
        floaters = self._unlisted_floaters(before)
        spares = {pu["name"]: pu["family"] for pu in pools
                  if pu.get("spare")
                     and not pu.get("uid")
                     and pu["name"] not in before.values()
                     and pu["name"] not in self._name_map.values()}
        loose: Dict[str, int] = {}
        for sec in cp.sections():
            n = sec[len(prefix):] if sec.startswith(prefix) else None
            if n in spares:
                loose[spares[n]] = loose.get(spares[n], 0) + 1
        changed = False
        for sec in cp.sections():
            n = sec[len(prefix):] if sec.startswith(prefix) else None
            if n not in ours:
                continue
            owners = [u for u, v in before.items() if v == n]
            drew = [u for u, v in self._name_map.items() if v == n]
            if not owners:
                owners = [u for u in drew if before.get(u) is None]
            fam = spares.get(n)
            if (not owners
                and fam
                and loose.get(fam) == 1
                and len(floaters.get(fam, [])) == 1):
                owners = list(floaters[fam])
            if len(owners) > 1:
                held = [u for u in owners if any(
                    pu["name"] == n and _norm_uid(pu.get("uid")) == u
                    for pu in pools)]
                owners = held if len(held) == 1 else owners
            if (len(owners) > 1
                and any(self._name_map.get(u) == n for u in owners)):
                self._note_learned(
                    f"learned values stored under {n} left unread -- "
                    f"{', '.join(sorted(owners))} are all recorded with that "
                    f"name. Run AFC_BRIDGEBOX_FORGET CHAIN={self.name} "
                    f"UID=<uid> for each one that is gone, and the one left "
                    f"takes them at the next restart.",
                    warn=True)
                continue
            keys = dict(cp.items(sec))
            cp.remove_section(sec)
            changed = True
            if len(owners) > 1:
                self._note_learned(
                    f"learned values stored under {n} dropped -- "
                    f"{', '.join(sorted(owners))} were all recorded with that "
                    f"name and another unit took it over")
                continue
            if not owners:
                self._note_learned(
                    f"learned values stored under {n} dropped -- no unit is "
                    f"recorded with that name")
                continue
            owner = owners[0]
            record = self._learned_section(owner)
            if not cp.has_section(record):
                cp.add_section(record)
            found = [k for k in _LEARNED_KEYS if k in keys
                                                 and _learned_length(keys[k]) is not None]
            moved = [k for k in found if not (
                cp.has_option(record, k)
                and _learned_length(cp.get(record, k)) is not None)]
            for k in moved:
                cp.set(record, k, keys[k])
            if not cp.items(record):
                cp.remove_section(record)
            other = sorted(set(keys) - set(found))
            dropped = (f"{', '.join(other)} dropped -- only a bowden length "
                       f"that is a positive number is kept" if other else "")
            if not found:
                if dropped:
                    self._note_learned(f"values stored under {n}: {dropped}")
                continue
            wearer = next((u for u in drew if u != owner), None)
            self._note_learned(
                f"learned values stored under {n} now belong to its unit "
                f"{owner}"
                + (f" ({', '.join(moved)})" if moved
                   else " (its own record holds them)")
                + (f", which wore that name before {wearer} took it; "
                   f"{wearer} measures its own" if wearer else "")
                + (f"; {dropped}" if dropped else ""))
        if changed:
            self._write_state(cp)

    def _migrate_legacy_state(self) -> None:
        """
        One-time absorb of the split .roster/.vars files this replaced.
        """
        base = os.path.dirname(self.auto_vars_file)
        legacy_roster = os.path.join(base, f"AFC_BridgeBox_{self.name}.roster")
        legacy_vars = os.path.join(base, f"AFC_BridgeBox_{self.name}.vars")
        updates: Dict[str, Dict[str, Any]] = {}
        try:
            with open(legacy_roster) as fp:
                roster = ", ".join(
                    ln.strip() for ln in fp
                    if ln.strip() and not ln.lstrip().startswith("#"))
            if roster:
                updates.setdefault(
                    self._BASE_SECTION + " " + self.name, {})["roster"] = roster
        except Exception:
            pass
        old = self._read_ini(legacy_vars)
        if old is not None:
            for sec in old.sections():
                tgt = (self._BASE_SECTION + " " + self.name
                       if sec == self._BASE_SECTION else sec)
                updates.setdefault(tgt, {}).update(dict(old.items(sec)))
        if updates:
            self._state_set(updates)
            for f in (legacy_roster, legacy_vars):
                try:
                    os.remove(f)
                except Exception:
                    pass

    @staticmethod
    def _chain_to_roster(uids: List[str], htmask: int) -> str:
        """
        The chain reply, rendered in roster syntax.

        Model comes from the firmware's htmask where it reports one (bit
        per chain index) and falls back to the enrollment convention (boxed
        units at 0..3, HTs at 4..) when htmask is 0. Empty positions are
        unenrolled slots and are skipped, but indices are preserved, because
        position is the polling address.

        :param uids: chain_uids(): index -> 24-hex UID, empties kept
        :param htmask: chain_diag()[0]: per-index HT flag bits, 0 if unknown
        :return str: "ht:UID, boxed:UID, ..." in chain order, or ""
        """
        entries = []
        for i, uid in enumerate(uids):
            uid = _norm_uid(uid)
            if not uid or set(uid) == {"F"}:
                continue
            is_ht = bool(htmask >> i & 1) if htmask else i >= 4
            entries.append(f"{'ht' if is_ht else 'boxed'}:{uid}")
        return ", ".join(entries)

    @staticmethod
    def _chain_snapshot(bridge: Any) -> Dict[str, Any]:
        """
        One chain reply, read once: the enrollment map and the counters that
        are judged against it must come from the same reply.

        The bridge's reader thread replaces the whole cache when a reply
        lands, so separate getter calls in one tick can pair uids from one
        reply with a2mask/a2asks from the next: a unit that just took an
        index would be judged by (or credited with) the other's counters.
        BambuBridge.chain_snapshot reads them under one lock. A bridge
        without it (older code, test stand-ins) is read through the separate
        getters as before, and one without dialect counters reports none.

        :param bridge: the chain's bridge
        :return dict: seq (None when the bridge cannot say), uids, htmask,
            a2mask, a2asks
        """
        snap = getattr(bridge, "chain_snapshot", None)
        if callable(snap):
            s = snap()
            return {"seq": s.get("seq"),
                    "uids": list(s.get("uids") or []),
                    "htmask": int(s.get("htmask") or 0),
                    "a2mask": int(s.get("a2mask") or 0),
                    "a2asks": list(s.get("a2asks") or [])}
        uids = bridge.chain_uids()
        htmask = bridge.chain_diag()[0]
        getter = getattr(bridge, "chain_dialect", None)
        a2mask, a2asks = getter() if callable(getter) else (0, [])
        return {"seq": None, "uids": list(uids), "htmask": int(htmask or 0),
                "a2mask": int(a2mask or 0), "a2asks": list(a2asks or [])}

    def _scout_ready(self, eventtime: Optional[float] = None) -> None:
        """
        klippy:ready handler: start the chain watch.

        Takes AFC's logger, makes sure a bridge exists on our port, and starts the
        slow chain watch.

        :param eventtime: reactor time (unused)
        """
        afc = self.printer.lookup_object("AFC", None)
        if afc is not None:
            self.logger = afc.logger
        # Report overrides that matched nothing (see where
        # _unmatched_overrides is built) now that a logger exists.
        for miss in getattr(self, "_unmatched_overrides", None) or []:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: [{miss}] overrides "
                f"nothing -- no unit and no model by that name, so its keys are "
                f"ignored. Models: {', '.join(sorted(_SLOTS_BY_MODEL))}.")
        # From config time, once: a negative dry_max_temp, the pool_ams
        # clamp, the bays named neither by their entry nor by their default
        # (see _given_names) and the layout notes.
        if getattr(self, "_dry_note", ""):
            self.logger.warning(self._dry_note)
            self._dry_note = ""
        if getattr(self, "_pool_ams_asked", 0) > _MAX_AMS_BAYS:
            ht = [pu for pu in getattr(self, "_pool_units", [])
                  if pu.get("family") == "ht"]
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: pool_ams is "
                f"{self._pool_ams_asked}, but a Bambu bus addresses at most "
                f"{_MAX_AMS_BAYS} AMS, so {_MAX_AMS_BAYS} AMS bays are built"
                + (f" and the HT lanes start at "
                   f"lane{self.lane_base + self._ams_band * 4}" if ht else "")
                + f". Set pool_ams: {_MAX_AMS_BAYS} to silence this.")
        for note in ((getattr(self, "_name_notes", None) or [])
                     + (getattr(self, "_layout_notes", None) or [])):
            self.logger.warning(note)
        self._name_notes, self._layout_notes = [], []
        # The band this start built, which a recorded HT holds no wider
        # than (see _roster_sections): klippy:ready means no config error
        # stopped it.
        band = getattr(self, "_ams_band", None)
        if band is not None:
            sec = self._BASE_SECTION + " " + self.name
            if self._state_get(sec, "ams_band") != str(band):
                self._state_set({sec: {"ams_band": band}})
        # Likewise what the fold and _migrate_name_learned did with learned
        # values stored under a bay name.
        for warn, note in getattr(self, "_learned_notes", None) or []:
            (self.logger.warning if warn else self.logger.info)(
                f"AFC_BridgeBox {self.name}: {note}")
        self._learned_notes = []
        # Before PREP's first save, which AFC would write with every unclaimed
        # bay empty; the hook has each save write the records held for them.
        try:
            self._capture_boot_records()
        except Exception as e:
            try:
                self.logger.debug(
                    f"AFC_BridgeBox {self.name}: no saved lane records held "
                    f"({e})")
            except Exception:
                pass
        try:
            self._hook_var_writes(afc)
        except Exception as e:
            try:
                self.logger.debug(
                    f"AFC_BridgeBox {self.name}: var-file hook not set ({e})")
            except Exception:
                pass
        try:
            self._hook_reset(afc)
        except Exception as e:
            try:
                self.logger.debug(
                    f"AFC_BridgeBox {self.name}: mapping reset hook not set "
                    f"({e})")
            except Exception:
                pass
        try:
            self._ready_at = self.printer.get_reactor().monotonic()
        except Exception:
            self._ready_at = None
        # PREP waits up to moonraker_timeout for Moonraker, then up to 30 s
        # for Spoolman's remote method, before it restores a lane.
        try:
            self._prep_wait = max(self._PREP_WAIT,
                                  float(afc.moonraker_connect_to) + 60.0)
        except Exception:
            self._prep_wait = self._PREP_WAIT
        # Only pure scout mode may create a bridge; otherwise the first unit
        # owns it.
        if self._roster_source == "scout":
            try:
                self._ensure_bridge()
            except Exception as e:
                self.logger.warning(
                    f"AFC_BridgeBox {self.name}: could not open the bridge "
                    f"for scouting ({e}); will keep retrying.")
        reactor = self.printer.get_reactor()
        self._scout_timer = reactor.register_timer(
            self._scout_tick, reactor.monotonic() + 2.0)
        if self._roster_source == "scout":
            self.logger.info(
                f"AFC_BridgeBox {self.name}: no roster configured and no "
                f"recorded roster -- scouting the chain. Detected units are "
                f"recorded in {os.path.basename(self.state_file)}; RESTART "
                f"then enrolls them.")

    def _capture_boot_records(self) -> None:
        """
        Hold the lane records the last session saved for each pool bay.

        AFC.var.unit still has them at klippy:ready: AFC's save_vars writes
        nothing until PREP has run. A bay's records were written while a unit
        was claimed onto it, so they are held for the unit bay_owner names
        for that bay, and handed to it alone when it claims the bay (see
        _held_for). They are held for the session, so a unit that comes
        online late still gets them, and every save writes them back to the
        bay while no unit is claimed onto it (see _fill_held_bays), so a
        restart before that unit comes back holds them again.

        A state block with no bay_owner key at all has recorded no owner:
        it is from a build before bay_owner, which gave a bay's records to
        whichever unit claimed it. Then a bay reserved for a unit at this
        boot is that unit's, as only that unit can claim a reserved bay,
        when the unit's name was pinned to that bay before this boot, or
        when the unit held no name and no unit held the bay's (the same
        guess _migrate_name_learned makes). With roster: set, the records of
        an unreserved spare whose name no unit held go to the one unit of
        its family that the recorded roster lists and roster: does not (see
        _unlisted_floaters), when that is the only such unit and this the
        only such spare with records. Any other bay's records are left
        unattributed. Each guess is recorded as its bay's owner, so the unit
        takes that bay back (see _last_bay), and is written once PREP has
        run (see _persist_bay_owner), or before a pin changes (see
        _keep_guessed_owners): later starts read it, and do not guess again
        from pins that can change by then.

        The records of a bay whose lanes this start moved, saved under the
        lanes it had, are held on the lanes it has now (see
        _moved_bay_records).
        """
        owners, present = self._load_bay_owner()
        self._bay_owner = owners
        self._bay_owner_pending = False
        self._held = {}
        afc = self.printer.lookup_object("AFC", None)
        pools = getattr(self, "_pool_units", None) or []
        if afc is None or not pools:
            return
        try:
            with open(f"{afc.VarFile}.unit") as fh:
                units = json.load(fh)
        except Exception:
            units = {}                  # no file, bad JSON: nothing to hold
        if not isinstance(units, dict):
            units = {}
        pins = getattr(self, "_pins_at_boot", None) or {}
        loose: Dict[str, List[Tuple[str, Dict[str, Any]]]] = {}
        for pu in pools:
            saved = units.get(pu["name"])
            if not isinstance(saved, dict):
                continue
            recs = {ln: dict(saved[ln]) for ln in pu["lanes"]
                    if isinstance(saved.get(ln), dict) and saved[ln]}
            if not recs:
                recs = self._moved_bay_records(pu, saved)
            if not recs:
                continue
            owner = owners.get(pu["name"])
            if owner is None and not present:
                drew = _norm_uid(pu.get("uid"))
                taken = pu["name"] in pins.values()
                if (drew
                    and (pins.get(drew) == pu["name"]
                         or (pins.get(drew) is None and not taken))):
                    owner = drew
                elif not drew and pu.get("spare") and not taken:
                    loose.setdefault(pu.get("family", ""), []).append(
                        (pu["name"], recs))
            if owner:
                self._held[pu["name"]] = {"uid": owner, "lanes": recs}
        floaters = self._unlisted_floaters(pins) if loose else {}
        for fam, bays in loose.items():
            if len(bays) == 1 and len(floaters.get(fam, [])) == 1:
                self._held[bays[0][0]] = {"uid": floaters[fam][0],
                                          "lanes": bays[0][1]}
        if not present and self._held:
            self._bay_owner = {bay: _norm_uid(entry["uid"])
                               for bay, entry in self._held.items()}
            self._bay_owner_pending = True
        if self._held:
            self.logger.debug(
                f"AFC_BridgeBox {self.name}: holding the saved lane records "
                f"of {', '.join(sorted(self._held))} for their units")

    @staticmethod
    def _moved_bay_records(pu: Dict[str, Any], saved: Dict[str, Any]
                           ) -> Dict[str, Dict[str, Any]]:
        """
        A bay's saved lane records, put on the lanes the bay has at this
        start when it had others when they were saved.

        A recorded HT moves to the lanes the AMS band leaves it when a lane
        it had is one a chain further down the config builds, or exists
        outside this chain (see _band_note). Its records are saved under the
        bay's name and keyed by the lanes it had. When none of them is a
        lane the bay has now, and there are as many as the bay has lanes,
        they go onto its lanes in lane-number order. A map that is only the
        old lane's home T# is dropped, as the lane's home T# is its new
        lane number's; any other saved map stays. A lane saved loaded to a
        toolhead comes back unloaded: AFC's loaded-lane record names the old
        lane, which the claim does not restore (see loaded_lane_moved).

        :param pu: the pool bay
        :param saved: what AFC.var.unit holds under the bay's name
        :return dict: current lane name -> record; {} when they do not fit
        """
        lanes = list(pu.get("lanes") or [])
        old = [ln for ln, rec in saved.items() if isinstance(rec, dict)]
        if (not lanes
            or len(old) != len(lanes)
            or any(ln in lanes for ln in old)
            or not all(_home_tool(ln) for ln in old + lanes)):
            return {}

        def num(lname: str) -> int:
            """
            The lane number of a lane.

            :param lname: a lane name
            :return int: its lane number
            """
            return int((_home_tool(lname) or "T0")[1:])

        out: Dict[str, Dict[str, Any]] = {}
        for was, now in zip(sorted(old, key=num), sorted(lanes, key=num)):
            rec = dict(saved[was])
            if not rec:
                continue
            if _parse_map(rec.get("map")) == [_home_tool(was)]:
                rec.pop("map", None)
                rec.pop("current_map", None)
            if "name" in rec:
                rec["name"] = now
            rec["tool_loaded"] = False
            out[now] = rec
        return out

    def _hook_var_writes(self, afc: Any) -> None:
        """
        Pass AFC's var-file saves through _fill_held_bays.

        AFC.save_vars hands each snapshot to its background writer through
        _var_write_queue, which the hook wraps (_HeldBaysQueue), once per
        chain. An AFC with no such queue is left alone: its saves write a
        bay no unit is claimed onto empty, and the bay's records are held for
        this session only.

        :param afc: the AFC object
        """
        if afc is None or getattr(self, "_var_hook", None) is not None:
            return
        queue = getattr(afc, "_var_write_queue", None)
        if queue is None or not callable(getattr(queue, "put_nowait", None)):
            self.logger.debug(
                f"AFC_BridgeBox {self.name}: AFC has no var-file write queue; "
                f"the records of a bay no unit is claimed onto are held for "
                f"this session only")
            return
        hook = _HeldBaysQueue(queue, self._fill_held_bays)
        afc._var_write_queue = hook
        self._var_hook = hook

    def _hook_reset(self, afc: Any) -> None:
        """
        Have AFC_RESET_MAPPING put this chain's claimed lanes on their home T#s
        (see _hook_reset_mapping).

        An AFC whose spool object has no _reset_mapping is left alone: its
        reset numbers the Bambu lanes as any other lane.

        :param afc: the AFC object
        """
        spool = getattr(afc, "spool", None) if afc is not None else None
        if spool is None:
            spool = self.printer.lookup_object("AFC_spool", None)
        if not _hook_reset_mapping(spool, self):
            self.logger.debug(
                f"AFC_BridgeBox {self.name}: AFC has no mapping reset to "
                f"wrap; AFC_RESET_MAPPING numbers the Bambu lanes as any "
                f"other lane")

    def _bambu_span(self) -> Tuple[int, int]:
        """
        The range of the Bambu lanes' home T#s.

        :return tuple: the first and last lane number of this chain's pool bays
        """
        nums = [int(d) for pu in getattr(self, "_pool_units", None) or []
                for d in ("".join(c for c in ln if c.isdigit())
                          for ln in pu.get("lanes") or []) if d]
        lo = getattr(self, "lane_base", 0) or min(nums, default=0)
        return lo, max(nums, default=lo)

    def _reset_home_plan(self, afc: Any) -> Dict[str, Any]:
        """
        What a mapping reset (see _hook_reset_mapping) does with this chain's
        claimed lanes.

        Each claimed Bambu lane takes its home T#, as AFC gives a lane its
        config map:, so the reset leaves it where every claim puts it, and a
        T# it had besides (SET_MAP, a virtual tool) goes as any lane's does.
        A home T# that is the first T# of a lane's config map:, the one AFC's
        reset gives that lane, stays that lane's, as at any reset, and the
        Bambu lane is numbered by AFC like any lane with no map: (said once,
        see _reset_home_done). So is a Bambu lane whose home T# is a macro
        other than AFC's CHANGE_TOOL: AFC's reset registers a T# without
        renaming what holds it, so the lane would be on a T# that does not
        select it, as a claim with a saved record leaves it (see
        _plan_lane_map). A lane already on that T#, where a claim with no
        record puts it, stays: numbered elsewhere, the reset would
        unregister the T#, and the macro with it, as one no lane has. A lane
        with no lane number, or with a map: of its own, is left to AFC too.

        A T# take waiting for the print (see _map_claimed_lanes) is dropped:
        the reset gives the lane its T#, and the take would put back the
        map the reset replaced. Its save would too (see _planned_maps), so it
        is dropped before AFC saves the reset, and put back when the reset
        fails.

        A bay no unit is claimed onto has no lanes in AFC's units, which the
        reset numbers, and is left as it is.

        :param afc: the AFC object
        :return dict: "afc": the AFC object; "home": [(lane, its home T#)]
            for the lanes that take it; "left": [(lane, its home T#, the
            lane whose map: names it, or None for a macro)] for the lanes
            left to AFC; "maps": lane name -> its map before the reset, for
            each of this chain's claimed lanes; "waits": lane name -> the
            dropped take
        """
        plan: Dict[str, Any] = {"afc": afc, "home": [], "left": [],
                                "maps": {}, "waits": {}}
        if afc is None:
            return plan
        bambu = {ln for pu in getattr(self, "_pool_units", None) or []
                 for ln in pu.get("lanes") or []}
        lanes = [(lname, lane)
                 for unit in list((getattr(afc, "units", None)
                                   or {}).values())
                 for lname, lane in list((getattr(unit, "lanes", None)
                                          or {}).items())]
        # AFC's reset gives a lane with a config map: its first T# only.
        config: Dict[str, str] = {}
        for lname, lane in lanes:
            for cmd in (getattr(lane, "_map", None) or [])[:1]:
                config.setdefault(cmd, lname)
        handlers = _tcmd_handlers(afc)
        pending = getattr(self, "_deferred_takes", None)
        for lname, lane in lanes:
            if lname not in bambu:
                continue
            plan["maps"][lname] = list(getattr(lane, "map", None) or [])
            if isinstance(pending, dict) and lname in pending:
                plan["waits"][lname] = pending.pop(lname)
            home = _home_tool(lname)
            if not home or getattr(lane, "_map", None):
                continue
            holder = config.get(home)
            handler = handlers.get(home)
            if holder is not None and holder != lname:
                plan["left"].append((lane, home, holder))
            elif (handler is not None
                  and not _tcmd_is_ours(afc, handler)
                  and home not in plan["maps"][lname]):
                plan["left"].append((lane, home, None))
            else:
                plan["home"].append((lane, home))
        return plan

    def _reset_home_done(self, plan: Dict[str, Any], ok: bool) -> None:
        """
        Say what a mapping reset did with this chain's lanes (see
        _reset_home_plan), or, when it failed, put back the T# takes it
        dropped.

        A lane whose home T# a config map: or a macro kept from it is said
        on the console once per session, as a claim says it (see
        _take_home_tool), and to AFC.log at every later reset; so is the T#
        AFC numbered it onto when that is a macro, which AFC's reset does not
        replace and which so does not select the lane. A lane the reset
        moved to its home T#, and a take it dropped, go to AFC.log.

        :param plan: what _reset_home_plan returned
        :param ok: whether AFC's reset returned
        """
        if not ok:
            pending = getattr(self, "_deferred_takes", None)
            if pending is None:
                pending = self._deferred_takes = {}
            for lname, entry in plan["waits"].items():
                pending.setdefault(lname, entry)
            return
        told: Set[Tuple[str, str, str]] = getattr(self, "_reset_told",
                                                  None) or set()
        self._reset_told = told
        lo, hi = self._bambu_span()
        afc = plan["afc"]
        handlers = _tcmd_handlers(afc)
        for lane, home, holder in plan["left"]:
            cmds = [m for m in getattr(lane, "map", None) or []
                    if m != "NONE"]
            msg = (f"AFC_BridgeBox {self.name}: the mapping reset left "
                   f"{lane.name} on {', '.join(cmds) or 'without a T#'}, not "
                   f"its home {home}: ")
            if holder is None:
                msg += (f"{home} is a macro other than AFC's CHANGE_TOOL, "
                        f"which AFC does not replace. Remove or rename that "
                        f"macro to have {lane.name} on {home}.")
            else:
                other = (getattr(afc, "lanes", None) or {}).get(holder)
                section = (getattr(other, "fullname", None)
                           or f"AFC_lane {holder}")
                msg += (f"map: {home} in [{section}] keeps {home} for "
                        f"{holder}. Set that map: outside T{lo}-T{hi} to have "
                        f"{lane.name} on {home}.")
            for cmd in cmds:
                handler = handlers.get(cmd)
                if handler is not None and not _tcmd_is_ours(afc, handler):
                    msg += (f" {cmd} is a macro other than AFC's CHANGE_TOOL, "
                            f"so it does not select {lane.name}: SET_MAP "
                            f"LANE={lane.name} MAP=<T#> gives it a T# that "
                            f"does.")
            key = (lane.name, home, holder or "")
            (self.logger.debug if key in told else self.logger.warning)(msg)
            told.add(key)
        moved = [f"{lane.name} {'+'.join(plan['maps'][lane.name]) or 'NONE'}"
                 f"->{home}" for lane, home in plan["home"]
                 if plan["maps"].get(lane.name) != [home]]
        if moved:
            self.logger.debug(
                f"AFC_BridgeBox {self.name}: the mapping reset put the Bambu "
                f"lanes on their home T#s: {', '.join(moved)}.")
        if plan["waits"]:
            self.logger.debug(
                f"AFC_BridgeBox {self.name}: the mapping reset mapped "
                f"{' and '.join(plan['waits'])}, so the T#s "
                f"{'it' if len(plan['waits']) == 1 else 'they'} waited for "
                f"are not taken.")

    def _fill_held_bays(self, data: Dict[str, Any]) -> None:
        """
        Write the lane records held for each pool bay no unit is claimed
        onto into a snapshot AFC.save_vars built.

        AFC saves a unit's registered lanes, and a pooled bay has none, so it
        is saved empty: a restart would find no record for the unit that
        owns the bay (offline for the whole boot, released, or not claimed
        yet after PREP's first save). The records held for the bay's owner
        (see _held_for) go there instead, as its claim would save them. A bay
        a unit is claimed onto is saved from its live lanes.

        A lane whose T# take waits for the print is saved with the map its
        claim planned (see _planned_maps), from the claim's own saves on, and
        a lane the claim has not mapped yet with the map its record gives it
        (see _unmapped_maps).

        :param data: the snapshot, unit name -> lane name -> record
        """
        held = getattr(self, "_held", None) or {}
        for pu in getattr(self, "_pool_units", None) or []:
            name = pu.get("name")
            if name not in data:
                continue
            if data[name]:
                if isinstance(data[name], dict):
                    self._unmapped_maps(data[name])
                    self._planned_maps(data[name])
                continue
            if pu.get("bound"):
                continue
            lanes = (held.get(name) or {}).get("lanes")
            if lanes:
                data[name] = copy.deepcopy(lanes)

    def _unmapped_maps(self, recs: Dict[str, Any]) -> None:
        """
        Give the record of each lane a claim has registered and not mapped
        yet the map it comes back with, in place of the NONE it has until
        its turn.

        A claim registers its lanes, then maps them one at a time (see
        _map_claimed_lanes), and each map's TcmdAssign saves: every save
        before a lane's turn has it on no T#. A restart before the last of
        them is written would restore that NONE as the lane's saved map.
        The map saved instead is the one its held record carries, else its
        home T#, which is what a restart plans from (see _plan_lane_map).

        :param recs: lane name -> record as save_vars writes it, changed in
            place
        """
        plans = getattr(self, "_claim_plans", None) or {}
        for lname, rec in recs.items():
            plan = plans.get(lname)
            if plan is None or not isinstance(rec, dict):
                continue
            if _parse_map(rec.get("map")):
                continue
            rec["map"], rec["current_map"] = plan

    def _planned_maps(self, recs: Dict[str, Any]) -> None:
        """
        Give the record of each lane whose T# take waits for the print (see
        _map_claimed_lanes) the map its claim planned, with any T# the lane
        was given since, in place of the map it has until then: a restart or
        a release before the print ends brings the lane back to that plan,
        not to a map without the T#s it waits for.

        :param recs: lane name -> record as save_vars writes it, changed in
            place
        """
        pending = getattr(self, "_deferred_takes", None) or {}
        for lname, rec in recs.items():
            entry = pending.get(lname)
            if entry is None or not isinstance(rec, dict):
                continue
            maps = [m for m in entry["maps"] if m != "NONE"]
            maps += [m for m in _parse_map(rec.get("map")) if m not in maps]
            maps.sort(key=lambda m: (len(m), m))
            rec["map"] = ", ".join(maps) or "NONE"
            current = rec.get("current_map")
            rec["current_map"] = (entry["current"] if entry["current"] in maps
                                  else current if current in maps
                                  else maps[0] if maps else "")

    def _held_for(self, uid: str, bay: str) -> Dict[str, Dict[str, Any]]:
        """
        The lane records a unit gets when it claims a bay.

        Only the unit they were saved under gets them, and only on that bay:
        a record describes the spools in that unit's bays. The entry stays
        held: a release replaces it, and FORGET, or a claim of that unit
        onto another bay (see _set_bay_owner), drops it.

        :param uid: the claiming unit's uid
        :param bay: the pool bay's name
        :return dict: lane name -> record, copies; {} for any other unit
        """
        entry = (getattr(self, "_held", None) or {}).get(bay) or {}
        if not uid or _norm_uid(entry.get("uid")) != _norm_uid(uid):
            return {}
        return {ln: copy.deepcopy(rec)
                for ln, rec in (entry.get("lanes") or {}).items()}

    #: Seconds after klippy:ready that claims wait for PREP at most, raised
    #: past a long moonraker_timeout (see _scout_ready).
    _PREP_WAIT = 90.0

    def _prep_settled(self) -> bool:
        """
        Whether a unit may be claimed yet: AFC's PREP has run.

        PREP restores every lane in afc.lanes from AFC.var.unit with no check
        of which unit a pool bay's record was saved under, so a lane claimed
        before it runs gets whatever the file holds for the bay. A lane
        claimed after gets only its own unit's records (see _held_for). The
        wait is bounded: a PREP that never finishes (an unreadable var file)
        must not keep every unit offline.

        :return bool: True when claims may go ahead
        """
        afc = self.printer.lookup_object("AFC", None)
        if afc is None or getattr(afc, "prep_done", True):
            return True
        since = getattr(self, "_ready_at", None)
        if since is None:
            return True
        try:
            now = self.printer.get_reactor().monotonic()
        except Exception:
            return True
        wait = getattr(self, "_prep_wait", self._PREP_WAIT)
        if now - since >= wait:
            if not getattr(self, "_prep_wait_warned", False):
                self._prep_wait_warned = True
                self.logger.warning(
                    f"AFC_BridgeBox {self.name}: PREP has not finished "
                    f"{wait:.0f}s after startup; claiming units anyway.")
            return True
        if not getattr(self, "_prep_wait_said", False):
            self._prep_wait_said = True
            self.logger.debug(
                f"AFC_BridgeBox {self.name}: unit claims wait for PREP")
        return False

    def _ensure_bridge(self) -> Any:
        """
        The bridge for our serial port, shared with any fabricated units
        via the same registry they use, created if scouting found none.

        :return object: the bridge
        """
        bridge = self._chain_bridge()
        if bridge is None:
            from extras import AFC_BambuAMS_bridge as _bridge_mod
            from extras.AFC_BambuAMS_bridge import BambuBridge, TcpPort
            is_tcp = str(self.serial_port or "").strip().lower() \
                .startswith("tcp://")

            def _open() -> Any:
                """
                Open the master's port, the same way the units do: a socket
                for a ``tcp://host:port`` bridge, else the USB-CDC serial port.

                :return Any: an open port, pyserial-shaped either way
                """
                if is_tcp:
                    host, port = TcpPort.parse(self.serial_port)
                    return TcpPort(host, port, timeout=0.1, write_timeout=0.5,
                                   key=self.tcp_key)
                import serial
                return serial.Serial(self.serial_port, 115200,
                                     timeout=0.1, write_timeout=0.5)

            bridge = BambuBridge(_open, self.printer.get_reactor(),
                                 self.logger)
            _bridge_mod._BRIDGES[self.serial_port] = bridge
            # A network bridge that is not up yet keeps retrying rather than
            # failing the scout, as it does for the units.
            bridge.start(defer_open=is_tcp)
        self._bridge = bridge
        return bridge

    def _scout_tick(self, eventtime: float) -> float:
        """
        The chain watch: ask, read, and record what changed.

        In scout mode this is how the first roster comes to exist; with
        units already fabricated it is how a unit plugged in later gets
        noticed. Either way the answer lands in the recorded roster and one
        console line says what to do. The record is only consulted when
        the roster: option is absent, so an explicit option stays
        authoritative. With a pool configured, units are also claimed onto
        and released from their bays live, a recorded unit's bay is saved
        once it has been online for enroll_grace (see _pin_recorded_units),
        and a unit that finds every bay of its family taken is offered the
        bay of one that is offline (see _offer_replace).

        A new uid is appended to the recorded roster once it has stayed
        online for enroll_grace. Absence alone proves little (a Pico reboot,
        a unit power-cycling mid-dry, the whole chain briefly deaf), so a
        rostered unit is only dropped after removal_grace of continuous
        absence from an otherwise-live chain (see _prune_missing). Roster
        edits take effect at the next restart.

        :param eventtime: reactor time
        :return float: next wake
        """
        self._tick_count = getattr(self, "_tick_count", 0) + 1
        # A bay owner a claim recorded before PREP had run is written once
        # it has (see _persist_bay_owner).
        if getattr(self, "_bay_owner_pending", False):
            try:
                self._persist_bay_owner()
            except Exception:
                pass
        # Its own guard: a fault in this warning must not stop the claims
        # and releases below. Each distinct failure is logged once.
        try:
            self._check_moved_loaded()
        except Exception as ex:
            msg = f"{type(ex).__name__}: {ex}"
            if msg != getattr(self, "_moved_err_logged", None):
                self._moved_err_logged = msg
                try:
                    self.logger.warning(
                        f"AFC_BridgeBox {self.name}: loaded-lane check failed "
                        f"({msg}); the chain watch carries on.")
                except Exception:
                    pass
        try:
            if self._roster_source == "scout":
                bridge = getattr(self, "_bridge", None) or self._ensure_bridge()
            else:
                bridge = self._chain_bridge()
                if bridge is None:
                    self._watch_state = "no-bridge"
                    return eventtime + 3.0      # pool-owner bridge still coming up
            # Skip the chain poll while the bridge is silent (an AMS 1 capscan
            # blocks it for 10-15 s); the rest of the tick reads cached state.
            # A bridge that cannot say is asked.
            _sf = getattr(bridge, "silent_for", None)
            try:
                _quiet = _sf() if callable(_sf) else None
            except Exception:
                _quiet = None
            if (not isinstance(_quiet, (int, float))
                or _quiet <= max(2.0, 1.5 * self.hotplug_poll)):
                bridge.send({"cmd": "chain"})
            # One read of the chain reply for the whole tick; see
            # _chain_snapshot for why uids and counters are never re-read.
            snap = self._chain_snapshot(bridge)
            uids = snap["uids"]
            seen = self._chain_to_roster(uids, snap["htmask"])
            sec = self._BASE_SECTION + " " + self.name
            entries = [e.strip()
                       for e in (self._state_get(sec, "roster") or "").split(",")
                       if e.strip()]
            known_uids = {_norm_uid(e.partition(":")[2])
                          for e in entries}
            # Enroll and claim need the unit online, not just in the sticky
            # chain_uids, which keeps pulled units until the Pico reboots.
            latest = bridge.latest_status() or {}
            online_idx = {u.get("n") for u in (latest.get("units") or [])
                          if u.get("online")}
            online_uids = {_norm_uid(uids[i])
                           for i in online_idx
                           if i < len(uids) and (uids[i] or "").strip()}
            # Treat a suppressed (forgotten but online) uid as absent; release
            # the hold once it is physically pulled.
            suppressed = getattr(self, "_forget_suppressed", None)
            if suppressed:
                for u in [x for x in suppressed if x not in online_uids]:
                    suppressed.discard(u)          # gone; let a re-plug enroll
                online_uids = online_uids - suppressed
            # With the link down the bridge still hands back its last status,
            # which says nothing about who is there now. Read it as no unit
            # known online, so nothing is enrolled, claimed, released or
            # given a model verdict on it, and every online run restarts at
            # the reconnect. After the suppression check, so a forgotten unit
            # is not let back in by an outage.
            link_up = getattr(bridge, "_serial", None) is not None
            if not link_up:
                online_idx, online_uids = set(), set()
            # Enroll only a unit that holds online continuously for enroll_grace
            # (see its option in __init__); a ghost that drops out between
            # blips resets and never enrolls.
            esince = getattr(self, "_enroll_since", None)
            if esince is None:
                esince = self._enroll_since = {}
            cand: Dict[str, str] = {}
            for e in seen.split(", "):
                u = _norm_uid(e.partition(":")[2])
                if e and u and u not in known_uids and u in online_uids:
                    cand[u] = e
            for u in list(esince):
                if u not in cand:
                    del esince[u]                  # dropped offline -> reset hold
            for u in cand:
                esince.setdefault(u, eventtime)
            new = [cand[u] for u in cand
                   if eventtime - esince.get(u, eventtime) >= self.enroll_grace]
            if new:
                entries = entries + new
                self._state_set({sec: {"roster": ", ".join(entries)}})
                pooled = bool(self.pool_ams or self.pool_ht)
                if self._roster_source == "scout" and not pooled:
                    self.logger.info(
                        f"AFC_BridgeBox {self.name}: chain reports "
                        f"[{', '.join(entries)}] -- written to "
                        f"{os.path.basename(self.state_file)}. RESTART to "
                        f"enroll, or copy it into roster: to pin it.")
                elif self._roster_source == "option":
                    no_bay = self._new_ams_with_no_bay(new)
                    # An AMS the claim already told it has no bay is not
                    # told again, but is still named as recorded; one not
                    # told yet is named by _tell_no_bay instead.
                    untold = [e for e in no_bay if _norm_uid(
                        e.partition(":")[2]) not in self._no_bay_told]
                    listed = [e for e in new if e not in untold]
                    unlisted = [e for e in listed if e not in no_bay
                                                     and self._unlisted(e.partition(":")[2])]
                    text = (f"AFC_BridgeBox {self.name}: NEW unit(s) on "
                            f"the chain: {', '.join(listed)} -- recorded.")
                    if unlisted:
                        one = len(unlisted) == 1
                        text += (f" Add {', '.join(unlisted)} to roster: to "
                                 f"enroll {'it' if one else 'them'} (the "
                                 f"option is set and overrides the file)")
                        # Pass 1b names a newly listed unit in roster:
                        # order, not by the bay it holds live.
                        if any(self._bay_of_uid(e.partition(":")[2])
                               for e in unlisted):
                            text += ("; the next restart gives it the lowest "
                                     "free bay of its family, which need not "
                                     "be the one it is on now" if one else
                                     "; the next restart gives them the "
                                     "lowest free bays of their family, which "
                                     "need not be the ones they are on now")
                        text += "."
                    if listed:
                        self.logger.info(text)
                    self._tell_no_bay(no_bay)
                elif pooled:
                    # Said after this tick's claims, and kept until said.
                    self._announce_pending = (
                        list(getattr(self, "_announce_pending", None) or [])
                        + new)
                else:
                    no_bay = self._new_ams_with_no_bay(new)
                    enroll = [e for e in new if e not in no_bay]
                    if enroll:
                        self.logger.info(
                            f"AFC_BridgeBox {self.name}: NEW unit(s) on the "
                            f"chain: {', '.join(enroll)} -- recorded. RESTART "
                            f"to enroll.")
                    self._tell_no_bay(no_bay)
            # Live claim: every present unbound unit gets its slot now (known
            # units their own, new ones a spare). The roster decides a known
            # unit's model; htmask classifies a new one.
            if self.pool_ams or self.pool_ht:
                model_by_uid: Dict[str, str] = {}
                for e in seen.split(", "):
                    m, _sep, u = e.partition(":")
                    if u.strip():
                        model_by_uid[_norm_uid(u)] = m.strip().lower()
                for e in entries:
                    m, _sep, u = e.partition(":")
                    if u.strip():
                        model_by_uid[_norm_uid(u)] = m.strip().lower()
                # The option's entries plus this session's refinements. An
                # option `boxed` yields to a generation the recorded roster
                # holds.
                if self._roster_source == "option":
                    for entry in self.units:
                        m = _norm_model(entry.get("model"))
                        if not (m == "boxed"
                                and model_by_uid.get(entry["uid"])
                   in ("ams1", "ams2")):
                            model_by_uid[entry["uid"]] = m
                bound = {pu["bound"] for pu in getattr(self, "_pool_units", [])
                         if pu.get("bound")}

                def _slot_laneno(u: str) -> Tuple[int, str]:
                    """
                    Sort key: the first lane number of the slot a uid owns,
                    else of the free slot wearing the name it is saved under
                    (see _claim_pool_unit), then the uid.

                    So a unit coming back claims its slot before a new unit
                    seen on the same tick can take it, and the order never
                    depends on set iteration.

                    A floating uid sorts by the free bay it was last claimed
                    onto, so it claims that bay back before a brand-new uid
                    can take it (see _claim_pool_unit).

                    :param u: unit uid
                    :return tuple: (lane number, 9999 for a uid with no slot
                        yet; uid)
                    """
                    saved = self._name_map.get(u)
                    mine = (next((pu for pu in self._pool_units
                                  if pu.get("uid") == u), None)
                            or next((pu for pu in self._pool_units
                                     if saved
                                        and pu.get("name") == saved
                                        and pu.get("uid") is None
                                        and pu.get("bound") is None), None))
                    if mine is not None and mine.get("lanes"):
                        ds = "".join(c for c in mine["lanes"][0]
                                     if c.isdigit())
                        return (int(ds) if ds else 9999, u)
                    last = self._last_bay(u)
                    if last is not None and last.get("lanes"):
                        return (_pool_laneno(last), u)
                    return (9999, u)

                # Presence is the per-unit online flag, for claim and release
                # alike. Track each unit's continuous online time; one offline
                # sample resets it.
                online_since = getattr(self, "_online_since", None)
                if online_since is None:
                    online_since = self._online_since = {}
                # A waiting unit that drops off is told and offered a bay again
                # on return. A tick with nobody online says nothing about who
                # left.
                if online_idx:
                    left = ({u for u in online_since if u not in online_uids}
                            | (set(self._no_bay) - online_uids))
                    self._no_bay_told -= left
                    self._replace_offered -= left
                    for u in left:
                        self._no_bay.pop(u, None)
                for u in online_uids:
                    online_since.setdefault(u, eventtime)
                for u in list(online_since):
                    if u not in online_uids:
                        del online_since[u]
                rel_at = getattr(self, "_released_at", None)
                if rel_at is None:
                    rel_at = self._released_at = {}
                # Claim once online long enough (claim_grace, or
                # flap_claim_grace right after a release), lowest lane first so
                # T# numbers line up with lanes.
                claimable = online_uids - bound
                for u in sorted(claimable, key=_slot_laneno):
                    req = (self.flap_claim_grace
                           if eventtime - rel_at.get(u, -1e9) < self.flap_window
                           else self.claim_grace)
                    if eventtime - online_since.get(u, eventtime) >= req:
                        # A unit that already owns a slot is just coming back
                        # (no popup); one that owns none takes a spare. Checked
                        # before the claim adopts it.
                        owned_before = any(
                            _norm_uid(pu.get("uid")) == u
                            for pu in self._pool_units)
                        if self._claim_pool_unit(
                                u, model_by_uid.get(u, "boxed")) is not None \
                                and not owned_before \
                                and not self._back_on_saved_bay(u) \
                                and not self._is_printing():
                            # It already has a home (the spare it just claimed);
                            # the popup only offers to move it to a named bay.
                            # Queued so a burst of adds serialize, none lost.
                            self._queue_popup(("new", u))
                # Release a unit offline for release_grace; a re-plug inside
                # the window resets its clock.
                online_seen = getattr(self, "_last_online", None)
                if online_seen is None:
                    online_seen = self._last_online = {}
                # A unit's continuous-online run start (absent = not online).
                # Only a run held for release_settle clears the release clock, so
                # a phantom online flag that merely blips can never cancel a drop.
                run = getattr(self, "_online_run", None)
                if run is None:
                    run = self._online_run = {}
                # A bound unit's absence, first to last offline tick; a settled
                # return spanning _REPLUG_ABSENCE restores its follower.
                away = getattr(self, "_away", None)
                if away is None:
                    away = self._away = {}
                returned = []
                bound_now = {pu["bound"] for pu in self._pool_units
                             if pu.get("bound")}
                # Units past the grace but kept claimed because a lane is still
                # in a toolhead. Tracked so the hold is logged once per absence.
                drop_held = getattr(self, "_drop_held", None)
                if drop_held is None:
                    drop_held = self._drop_held = set()
                for b in bound_now:
                    if not link_up:
                        # Paused, not counted: the release clock restarts at
                        # the reconnect, so a unit slow to answer after a
                        # bridge reboot still gets its whole release_grace.
                        online_seen[b] = eventtime
                        continue
                    if b in online_uids:
                        run.setdefault(b, eventtime)           # run continues
                        if eventtime - run[b] >= self.release_settle:
                            online_seen[b] = eventtime          # solidly back
                            first, last = away.pop(b, (0.0, 0.0))
                            if last - first >= self._REPLUG_ABSENCE:
                                returned.append((b, run[b], last - first))
                            drop_held.discard(b)
                    else:
                        run.pop(b, None)                       # a gap breaks it
                        online_seen.setdefault(b, eventtime)   # start the clock
                        away.setdefault(b, [eventtime, eventtime])
                        away[b][1] = eventtime                 # still away
                for b, since, gone_s in returned:
                    self._restore_returned(b, since, gone_s)
                announce = getattr(self, "_announce_pending", None)
                if announce:
                    self._announce_pending = []
                    self._announce_enrolled(announce)
                # Save the bay of every recorded unit that has held one for
                # enroll_grace, so a restart brings it back there.
                if self._roster_source == "option":
                    recorded = {_norm_uid(u["uid"]) for u in self.units}
                else:
                    recorded = {_norm_uid(e.partition(":")[2])
                                for e in entries}
                self._pin_recorded_units(recorded, eventtime, online_since)
                if self.auto_drop and not self._is_printing():
                    for b in list(bound_now):
                        # Drop only on a tick the unit is actually offline.
                        if (b not in online_uids
                            and eventtime - online_seen.get(b, eventtime)
                            >= self.release_grace):
                            # The bay's name, captured before release clears the
                            # binding, for the removal popup.
                            gone = next(
                                (pu.get("name") for pu in self._pool_units
                                 if pu.get("bound") == b), None)
                            # Stay claimed while AFC records a lane in a
                            # toolhead; drop once cleared.
                            loaded = self._toolhead_lanes(b)
                            if loaded:
                                if b not in drop_held:
                                    drop_held.add(b)
                                    self.logger.info(
                                        f"AFC_BridgeBox {self.name}: "
                                        f"{gone or b} (UID {b}) is offline but "
                                        f"AFC records "
                                        f"{', '.join(ln.name for ln in loaded)}"
                                        f" as loaded to the toolhead -- "
                                        f"keeping it claimed. Unload it ("
                                        + self._unset_hint(
                                            [ln.name for ln in loaded])
                                        + " if the filament is already out) "
                                          "and it is released on the next "
                                          "check.")
                                continue
                            self._release_pool_unit(b)
                            online_seen.pop(b, None)
                            run.pop(b, None)
                            rel_at[b] = eventtime   # anti-flap: bar rises before reclaim
                            self._queue_popup(("removed", b, gone))
                # After the claims and releases, so both see the bays as this
                # tick leaves them.
                self._track_absence(eventtime, bridge, online_uids,
                                    online_idx, online_since)
                self._offer_replace(eventtime, online_uids, online_since)
                # Walk the popup queue: one dialog per _POPUP_HOLD, so a burst
                # of plug/unplugs is shown in turn rather than clobbered.
                self._pump_popups(eventtime)
            # One fast fixed interval: the "chain" command only reads cached
            # state and never touches the bus. Graces use eventtime, so the
            # interval does not change them.
            self._watch_next = self.hotplug_poll
            entries = self._refine_models(snap, online_idx, entries, sec)
            self._save_held_ids(bridge)
            try:
                self._take_deferred_tools()
            except Exception as ex:
                self.logger.debug(
                    f"AFC_BridgeBox {self.name}: deferred T# take failed: "
                    f"{ex}")
            self._prune_missing(bridge, uids, entries, eventtime, sec,
                                online_idx)
            # Re-read what the record now says (refine and prune both may
            # have rewritten it) so get_status can show the recorded chain
            # beside the running one (the difference applies at restart).
            self._recorded_raw = self._state_get(sec, "roster") or ""
        except Exception as e:
            # Best-effort, but not silent: log each distinct failure once.
            msg = f"{type(e).__name__}: {e}"
            self._watch_state = f"error: {msg}"
            if msg != getattr(self, "_tick_err_logged", None):
                self._tick_err_logged = msg
                try:
                    self.logger.warning(
                        f"AFC_BridgeBox {self.name}: chain watch tick failed "
                        f"({msg}); will keep retrying.")
                except Exception:
                    pass
        return eventtime + getattr(self, "_watch_next", self.hotplug_poll)

    def _is_printing(self) -> bool:
        """
        Whether a print is active or paused.

        Auto-drop must never fire then: dropping a lane mid-print would disrupt a
        follower/feed. A pull during a print is left alone; the unit stays claimed
        (lanes intact) until the print ends, then the drop runs.

        :return bool: True while a print is active or paused
        """
        try:
            now = self.printer.get_reactor().monotonic()
        except Exception:
            now = 0.0
        for name, states in (("print_stats", ("printing", "paused")),
                             ("idle_timeout", ("Printing",))):
            try:
                obj = self.printer.lookup_object(name, None)
                if obj is not None:
                    st = obj.get_status(now).get("state")
                    if st in states:
                        return True
            except Exception:
                pass
        return False

    def _prompt(self, title: str, lines: List[str],
                buttons: List[Tuple[str, str, str]],
                timeout: float = 45.0) -> None:
        """
        Raise a Mainsail/Fluidd interactive dialog (the action:prompt protocol
        Klipper speaks over the g-code channel, no panel change needed).

        Each button runs a g-code command when clicked; a Dismiss footer closes
        it with no action, and it auto-closes after `timeout` so an unattended
        popup never lingers. Best-effort: a UI notification must never fault the
        watch tick.

        :param title: dialog title
        :param lines: body text lines
        :param buttons: (label, gcode command, style) each; style is one of
            primary/secondary/info/warning/error
        :param timeout: seconds before the dialog self-dismisses
        """
        gcode = self.printer.lookup_object("gcode", None)
        if gcode is None:
            return
        try:
            # Each popup gets a generation number, and its auto-dismiss timer
            # only fires prompt_end if it is still the current popup, so an
            # earlier popup's timer cannot close a later dialog early.
            gen = getattr(self, "_prompt_gen", 0) + 1
            self._prompt_gen = gen
            respond = gcode.respond_raw
            respond(f"// action:prompt_begin {title}")
            for ln in lines:
                respond(f"// action:prompt_text {ln}")
            for label, cmd, style in buttons:
                respond(f"// action:prompt_button {label}|{cmd}|{style}")
            respond("// action:prompt_footer_button "
                    "Dismiss|RESPOND TYPE=command MSG=action:prompt_end|info")
            respond("// action:prompt_show")
            if timeout:
                reactor = self.printer.get_reactor()

                def _close(e: float, _gen: int = gen) -> None:
                    """
                    Close the prompt if it is still the one this timer armed.

                    :param e: reactor event time
                    :param _gen: prompt generation this callback belongs to
                    """
                    if getattr(self, "_prompt_gen", 0) == _gen:
                        gcode.respond_raw("// action:prompt_end")
                reactor.register_callback(
                    _close, reactor.monotonic() + timeout)
        except Exception as ex:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: prompt error: {ex}")

    def _close_prompt(self) -> None:
        """
        Close whatever action:prompt dialog is open.

        Called at the end of an action a popup button triggered
        (assign/unassign/forget), because Mainsail runs a prompt_button's
        command but does not close the dialog on its own. Bumps the generation
        so no pending auto-dismiss fires against a later popup. Harmless when
        no dialog is open (called direct).
        """
        gcode = self.printer.lookup_object("gcode", None)
        if gcode is None:
            return
        self._prompt_gen = getattr(self, "_prompt_gen", 0) + 1
        try:
            gcode.respond_raw("// action:prompt_end")
        except Exception:
            pass

    #: Most "move to a named bay" buttons a popup offers, nearest first.
    _MAX_BAY_BUTTONS = 8

    def _bay_of_uid(self, uid: str) -> Optional[Dict[str, Any]]:
        """
        The pool bay a uid is bound to, or (failing that) reserves.

        :param uid: a unit uid
        :return dict: the pool unit record, or None
        """
        uid = _norm_uid(uid)
        pools = getattr(self, "_pool_units", [])
        return (next((pu for pu in pools
                      if _norm_uid(pu.get("bound")) == uid), None)
                or next((pu for pu in pools
                         if _norm_uid(pu.get("uid")) == uid), None))

    def _back_on_saved_bay(self, uid: str) -> bool:
        """
        Whether a unit is back on the bay saved for it, so there is no bay to offer.

        True when the bay bound to ``uid`` wears the name the name map keeps for it:
        a unit coming back, not a new one. A tombstone counts, since it is held
        there from the tick it is recorded (see _bay_held). A uid a set roster:
        option does not list does not: its popup names the roster: entry to add.

        :param uid: a unit uid
        :return bool: True when the unit is on its saved bay
        """
        uid = _norm_uid(uid)
        pu = next((p for p in getattr(self, "_pool_units", [])
                   if _norm_uid(p.get("bound")) == uid), None)
        return (pu is not None
                and _norm_uid(pu.get("uid")) == uid
                and getattr(self, "_name_map", {}).get(uid) == pu.get("name")
                and not self._unlisted(uid))

    def _prompt_new_unit(self, uid: str, timeout: float = 45.0) -> None:
        """
        Bay-picker popup for a unit that already has a home.

        The title shows the bay. The buttons only offer to move it to one of the
        operator's other free bays of the same family via AFC_BRIDGEBOX_ASSIGN.
        Dismiss keeps it where it is: there is no stuck state, because it already
        sits on a real named bay. Fired automatically when a brand-new unit is
        claimed, and on demand by AFC_BRIDGEBOX_ASSIGN UID=<uid> with no NAME. A uid
        a set roster: option does not list gets no buttons, only the roster: entry
        to add. The body says the bay is saved for the unit when it is, or will be
        once the unit has been online for enroll_grace (see _pin_recorded_units),
        and says nothing of saving otherwise.

        :param uid: the unit's uid
        :param timeout: seconds before the popup closes itself, 0 to keep it open
        """
        uid = _norm_uid(uid)
        here = self._bay_of_uid(uid)
        if here is None:
            return
        family = here.get("family")
        pools = getattr(self, "_pool_units", [])

        head = (f"UID {uid} is on {here['name']!r} "
                + ("(its T# and lanes are live)." if here.get("bound")
                   else "(reserved for it; its T# and lanes are not live "
                        "yet)."))
        if self._unlisted(uid):
            # ASSIGN refuses a uid a set roster: does not list, so no bay
            # button is offered.
            self._prompt(f"New AMS on {here['name']}",
                         [head, f"roster: is set and does not list it. Add "
                          f"{self._roster_entry(uid)} to roster: to pin it "
                          f"to a named bay."], [], timeout=timeout)
            return
        # Only bays ASSIGN takes: one saved for another recorded uid is that
        # uid's at restart, so ASSIGN refuses it.
        recorded = self._recorded_uids()
        free = sorted((pu for pu in pools
                       if pu is not here
                          and pu.get("family") == family
                          and not pu.get("bound")
                          and not pu.get("uid")
                          and not self._name_holder(pu["name"], uid, recorded)),
                      key=_pool_laneno)
        buttons = [(pu["name"],
                    f"AFC_BRIDGEBOX_ASSIGN CHAIN={self.name} UID={uid} "
                    f"NAME={pu['name']}", "primary")
                   for pu in free[:self._MAX_BAY_BUTTONS]]
        # Whether this bay is saved for it: said only where it is true or
        # will be (see _pin_recorded_units); the console says why not.
        if self._bay_held(here):
            saved = ["It is saved on this bay."]
        elif self._save_refusal(here, uid) is None:
            saved = [f"It is saved on this bay once it has been online "
                     f"{self.enroll_grace:.0f}s."]
        else:
            saved = []
        lines = ([head] + saved
                 + (["Keep it here, or move it to another named bay:"]
                    if buttons
                    else ["No other free bay of this type to move it to."]))
        self._prompt(f"New AMS on {here['name']}", lines, buttons,
                     timeout=timeout)

    def _prompt_removed_unit(self, uid: str, name: Optional[str],
                             timeout: float = 45.0) -> None:
        """
        Popup when a unit is auto-dropped on unplug.

        A bay the unit is saved on (see _bay_held) stays reserved for its re-plug,
        so there is nothing to do but Dismiss. Any other bay went back to the pool
        on release, so a re-plug takes it back while it is free (see _last_bay),
        else the lowest free bay of its family. The one button offers to forget the
        unit, for when it is not coming back. Never shown during a print (release
        is gated then anyway).

        :param uid: the unit's uid
        :param name: the unit's name, or None to show the uid
        :param timeout: seconds before the popup closes itself, 0 to keep it open
        """
        label = name or uid
        if self._bay_of_uid(uid) is not None:
            lines = [f"{label} (UID {uid}) was unplugged; its bay is held for "
                     "a re-plug.",
                     "Re-plug it and it reclaims the same lanes/T#. Or forget "
                     "it to free the bay to the pool:"]
        else:
            lines = [f"{label} (UID {uid}) was unplugged; its bay went back "
                     "to the pool.",
                     "A re-plug takes it back while it is free, else the "
                     "lowest free bay of its family. Or forget it if it is "
                     "not coming back:"]
        self._prompt(
            f"AMS removed: {label}",
            lines,
            [(f"Forget {label}",
              f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} UID={uid}", "error")],
            timeout=timeout)

    def _reactor_now(self) -> float:
        """
        The current reactor time.

        :return float: reactor time, 0.0 when there is no reactor to ask
        """
        try:
            return self.printer.get_reactor().monotonic()
        except Exception:
            return 0.0

    def _family_of_uid(self, uid: str) -> Optional[str]:
        """
        The bay family a unit belongs to.

        Taken from its recorded model (the one ASSIGN goes by), else the model it was
        seen as when it found no bay, else the family of the bay it holds.

        :param uid: a unit uid
        :return str: "ams" or "ht", or None when this chain knows nothing of it
        """
        uid = _norm_uid(uid)
        model = self._model_for_uid(uid) or self._no_bay.get(uid)
        if model:
            return "ht" if model in _HT_MODELS_BB else "ams"
        bay = self._bay_of_uid(uid)
        return bay.get("family") if bay is not None else None

    def _held_offline(self, family: str,
                      online: Optional[Set[str]] = None
                      ) -> List[Dict[str, Any]]:
        """
        The pool bays of a family held for a unit that is offline: the bay's
        uid is set, bound to it (auto_drop off, or not yet past the grace)
        or reserved for it (see _bay_held).

        :param family: "ams" or "ht"
        :param online: the uids online now; read from the bridge when not
            given
        :return list: those bays, lowest lane first
        """
        out = []
        for pu in sorted(getattr(self, "_pool_units", None) or [],
                         key=_pool_laneno):
            uid = _norm_uid(pu.get("uid"))
            if not uid or pu.get("family") != family:
                continue
            up = (uid in online if online is not None
                  else self._uid_online_now(uid))
            if not up:
                out.append(pu)
        return out

    def _replace_candidates(self, family: str, now: float,
                            online: Optional[Set[str]] = None
                            ) -> List[Dict[str, Any]]:
        """
        The bays of a family a unit waiting for a bay may take over: held for a
        unit that has been offline release_grace or longer by the absence clock
        (see _track_absence).

        A bay whose absence is not counted is not one.

        :param family: "ams" or "ht"
        :param now: reactor time
        :param online: the uids online now; read from the bridge when not
            given
        :return list: those bays, lowest lane first
        """
        return [pu for pu in self._held_offline(family, online)
                if now - self._missing_since.get(_norm_uid(pu["uid"]),
                                                 math.inf)
                >= self.release_grace]

    def _bay_loaded(self, pu: Dict[str, Any]) -> List[str]:
        """
        The lanes of a pool bay AFC records as loaded to a toolhead.

        A bound bay's are its unit's (see _toolhead_lanes). An unbound bay's
        lanes are pooled, and PREP restores each extruder's lane_loaded from
        the var file whether or not the lane it names is claimed, so an
        extruder naming one of them counts.

        :param pu: pool unit record
        :return list: lane names
        """
        if pu.get("bound"):
            return [ln.name for ln in self._toolhead_lanes(pu["bound"])]
        afc = self.printer.lookup_object("AFC", None)
        named = {getattr(e, "lane_loaded", None)
                 for e in (getattr(afc, "tools", None) or {}).values()}
        return [n for n in pu["lanes"]
                if n in named
                   or getattr(self.printer.lookup_object(
         f"AFC_lane {n}", None), "tool_loaded", False)]

    def _clear_bay_toolhead(self, pu: Dict[str, Any]) -> List[str]:
        """
        Clear AFC's record of an unbound bay's lanes as loaded to a toolhead.

        The lanes are pooled, out of afc.lanes, where no unload or
        UNSET_LANE_LOADED reaches them, so only the record is cleared: each
        extruder's lane_loaded naming one of them, and the lane's own
        tool_loaded. The vars are saved, or PREP restores the record at the
        next boot. A bound bay's record is cleared by its release (see
        _release_pool_unit).

        :param pu: an unbound pool bay
        :return list: the lanes whose record was cleared
        """
        afc = self.printer.lookup_object("AFC", None)
        cleared = []
        for n in pu["lanes"]:
            lane = self.printer.lookup_object(f"AFC_lane {n}", None)
            hit = bool(getattr(lane, "tool_loaded", False))
            if hit:
                lane.tool_loaded = False
                lane.loaded_to_hub = False
            for ext in (getattr(afc, "tools", None) or {}).values():
                if getattr(ext, "lane_loaded", None) == n:
                    ext.lane_loaded = None
                    hit = True
            if hit:
                cleared.append(n)
        if cleared and afc is not None:
            try: afc.save_vars()
            except Exception: pass
        return cleared

    def _track_absence(self, now: float, bridge: Any, online: Set[str],
                       online_idx: set, online_since: Dict[str, float]
                       ) -> None:
        """
        Time how long each unit a pool bay is held for has been offline, in
        _missing_since, which a pooled chain uses for nothing else (it never
        auto-removes, see _prune_missing).

        Absence counts only while it can be told from an outage, as for
        auto-removal: the bridge link is up and some unit is online.
        Otherwise every clock clears, so a bridge reconnect or a dark chain
        never makes a known unit look gone. A unit back online keeps its
        clock until it has held online release_settle, so a phantom online
        flag that blips does not restart it. Bound and reserved bays count
        alike.

        :param now: reactor time
        :param bridge: the chain's bridge
        :param online: the uids online this tick
        :param online_idx: the chain indices online this tick
        :param online_since: uid -> start of its continuous online run
        """
        if getattr(bridge, "_serial", None) is None or not online_idx:
            self._missing_since.clear()
            return
        held = {_norm_uid(pu["uid"]) for pu in self._pool_units
                if pu.get("uid")}
        for u in [x for x in self._missing_since if x not in held]:
            del self._missing_since[u]
        for u in held:
            if u not in online:
                self._missing_since.setdefault(u, now)
            elif now - online_since.get(u, now) >= self.release_settle:
                self._missing_since.pop(u, None)

    def _offer_replace(self, now: float, online: Set[str],
                       online_since: Dict[str, float]) -> None:
        """
        Queue the replace picker (see _prompt_replace_unit) for each unit
        that found no free bay, once it has been online enroll_grace (the
        run enrollment asks for, so a phantom online flag is never offered
        a bay), and a bay of its family is held for a unit offline
        release_grace or longer with no lane AFC records in a toolhead.

        Offered once per wait, as the no-bay console line is said: again
        only after the unit drops off the chain and comes back, and an offer
        the queue drops before showing it is made again. Nothing is offered
        during a print: the offer waits for the print to end. A set roster:
        option decides which units get a bay, so it offers nothing. The
        family is the one the unit's claim went by (see _claim_pool_unit).

        :param now: reactor time
        :param online: the uids online this tick
        :param online_since: uid -> start of its continuous online run
        """
        if self._roster_source == "option":
            return
        due = [uid for uid in sorted(self._no_bay)
               if uid not in self._replace_offered
                  and uid in online
                  and now - online_since.get(uid, now) >= self.enroll_grace]
        if not due or self._is_printing():
            return
        for uid in due:
            fam = "ht" if self._no_bay[uid] in _HT_MODELS_BB else "ams"
            if all(self._bay_loaded(pu) for pu in
                   self._replace_candidates(fam, now, online)):
                continue
            self._replace_offered.add(uid)
            self._queue_popup(("replace", uid))

    def _prompt_replace_unit(self, uid: str, now: float,
                             timeout: float = 45.0, auto: bool = False
                             ) -> bool:
        """
        The replace picker: every bay of the unit's family held for a unit
        offline release_grace or longer (see _replace_candidates), each with a
        Replace button that runs AFC_BRIDGEBOX_REPLACE with the old unit's uid,
        so a button left over after the bay changed hands is refused.

        A bay with a lane AFC records in a toolhead is listed with it and gets
        no button, with what clears the record: an unload of a claimed bay's
        lane, the old unit plugged back in for an unclaimed bay's, or the typed
        command with FORCE=1 for either, which is not offered during a print,
        since FORCE=1 also overrides the print gate.

        :param uid: the unit waiting for a bay
        :param now: reactor time
        :param timeout: seconds before the dialog closes itself
        :param auto: raised by the chain watch, which shows it only with a
            button to press
        :return bool: whether the dialog was raised
        """
        fam = self._family_of_uid(uid)
        held = self._replace_candidates(fam, now) if fam else []
        rows = [(pu, _norm_uid(pu["uid"]), self._bay_loaded(pu))
                for pu in held]
        buttons = [(f"Replace {pu['name']}",
                    f"AFC_BRIDGEBOX_REPLACE CHAIN={self.name} UID={uid} "
                    f"OLD={old}", "error")
                   for pu, old, loaded in rows
                   if not loaded][:self._MAX_BAY_BUTTONS]
        if not rows or (auto and not buttons):
            return False
        what = "AMS HT" if fam == "ht" else "AMS"
        printing = self._is_printing()
        # A recorded AMS this start gave no bay is not new: it had one, or
        # was recorded while every bay was held.
        unbayed = getattr(self, "_unbayed", None) or {}
        if uid in unbayed:
            title = f"No bay for {what} {uid}"
            head = (f"UID {uid} is recorded but has no bay: every {what} bay "
                    f"is taken."
                    + (f" A Bambu bus addresses at most {_MAX_AMS_BAYS} AMS, "
                       f"and its saved bay {unbayed[uid]} is not one of the "
                       f"{_MAX_AMS_BAYS} AMS bays." if unbayed[uid] else ""))
        else:
            title = f"No free bay for new {what}"
            head = f"UID {uid} has no free bay: every {what} bay is taken."
        lines = [head,
                 "Replace a unit that is offline: the new one takes its bay, "
                 "lanes and T# now, and the old one is forgotten (its learned "
                 "values and saved lane records, spools included, are "
                 "erased)."]
        for pu, old, loaded in rows:
            gone = now - self._missing_since.get(old, now)
            ago = f"{gone / 60:.0f} min" if gone >= 120 else f"{gone:.0f}s"
            line = (f"{pu['name']}: {old}, "
                    f"{self._lanes_text(_pool_laneno(pu), len(pu['lanes']))}, "
                    f"offline {ago}")
            force = (f"AFC_BRIDGEBOX_REPLACE CHAIN={self.name} UID={uid} "
                     f"OLD={old} FORCE=1")
            if loaded and printing:
                line += (f" -- AFC records {', '.join(loaded)} as loaded to "
                         f"the toolhead, and a print is active: replace it "
                         f"once the print ends")
            elif loaded and pu.get("bound"):
                line += (f" -- AFC records {', '.join(loaded)} as loaded to "
                         f"the toolhead: unload it "
                         f"({self._unset_hint(loaded)} if the filament is "
                         f"already out), or {force} clears it")
            elif loaded:
                # An unclaimed bay's lanes are not registered, so no unload
                # or UNSET_LANE_LOADED reaches them (see _clear_bay_toolhead).
                line += (f" -- AFC records {', '.join(loaded)} as loaded to "
                         f"the toolhead: plug {old} back in and unload it, or "
                         f"take the filament out by hand and {force} clears "
                         f"the record")
            lines.append(line)
        lines.append(f"Dismiss leaves it waiting; AFC_BRIDGEBOX_REPLACE "
                     f"CHAIN={self.name} UID={uid} opens this again.")
        self._prompt(title, lines, buttons, timeout=timeout)
        return True

    def _no_candidate_text(self, uid: str, family: str, now: float) -> str:
        """
        Why the replace picker has no bay to show, for the typed command.

        :param uid: the unit waiting for a bay
        :param family: its family
        :param now: reactor time
        :return str: the bays held for offline units that are not past
            release_grace yet, or that every bay holds a unit that is online
        """
        what = "HT" if family == "ht" else "AMS"
        held = self._held_offline(family)
        if not held:
            return (f"no {what} bay is held for a unit that is offline, so "
                    f"{uid} has none to take over -- unplug the unit it "
                    f"replaces, or AFC_BRIDGEBOX_FORGET CHAIN={self.name} "
                    f"UID=<its uid> frees that unit's bay now.")
        # FORCE=1 overrides the print gate, the PREP wait and the loaded-lane
        # refusal as well as the wait, so it is offered as skipping only the
        # wait when none of those applies.
        printing = self._is_printing()
        settled = self._prep_settled()
        rows = []
        for pu in held:
            old = _norm_uid(pu["uid"])
            since = self._missing_since.get(old)
            loaded = self._bay_loaded(pu)
            force = (f"AFC_BRIDGEBOX_REPLACE CHAIN={self.name} UID={uid} "
                     f"OLD={pu['name']} FORCE=1")
            if printing:
                tail = "a print is active: run this again once it ends"
            elif not settled:
                tail = "PREP has not run yet: run this again once it has"
            elif loaded and pu.get("bound"):
                tail = (f"AFC records {', '.join(loaded)} as loaded to the "
                        f"toolhead: unload it ({self._unset_hint(loaded)} if "
                        f"the filament is already out), or {force} clears it "
                        f"and skips the wait")
            elif loaded:
                tail = (f"AFC records {', '.join(loaded)} as loaded to the "
                        f"toolhead: plug {old} back in and unload it, or take "
                        f"the filament out by hand and {force} clears the "
                        f"record and skips the wait")
            else:
                tail = f"{force} skips the wait"
            rows.append(f"{pu['name']} ({old}, "
                        + ("no absence counted" if since is None
                           else f"offline {now - since:.0f}s")
                        + f") -- {tail}")
        return (f"no {what} bay is held for a unit offline "
                f"{self.release_grace:.0f}s or longer: "
                + "; ".join(rows) + ".")

    def _replace_text(self, uid: str, bays: List[Dict[str, Any]]) -> str:
        """
        The sentence naming AFC_BRIDGEBOX_REPLACE for a unit waiting for a bay.

        REPLACE forgets the old unit and puts this one on its bay in one step.

        :param uid: a unit waiting for a bay
        :param bays: the bays held for the offline units it may replace
        :return str: the sentence; "" with no bay, no pool, or roster: set
        """
        if (not bays
            or self._roster_source == "option"
            or not (self.pool_ams or self.pool_ht)):
            return ""
        old = bays[0]["name"] if len(bays) == 1 else "<that unit's bay>"
        return (f" AFC_BRIDGEBOX_REPLACE CHAIN={self.name} UID={uid} "
                f"OLD={old} does both in one step.")

    #: Seconds a queued popup holds the screen before the next replaces it.
    _POPUP_HOLD = 20.0

    def _queue_popup(self, ev: Tuple[Any, ...]) -> None:
        """
        Enqueue a popup event so simultaneous plug/unplugs serialize instead of
        overwriting each other (Mainsail shows only the newest prompt).

        Deduped by (kind, uid): a unit that flaps does not stack duplicate
        popups.

        :param ev: ("new", uid), ("removed", uid, name) or ("replace", uid)
        """
        q = getattr(self, "_popup_queue", None)
        if q is None:
            q = self._popup_queue = []
        key = (ev[0], ev[1])
        q[:] = [e for e in q if (e[0], e[1]) != key]
        q.append(ev)

    def _pump_popups(self, eventtime: float) -> None:
        """
        Show the next queued popup if the last one has had its turn.

        Called every watch tick; one popup shows per _POPUP_HOLD so a burst of adds
        is walked through, none lost. A "new" event whose unit is no longer present
        is dropped silently. A "replace" event is dropped, and may be offered again
        (see _offer_replace), when its unit is no longer waiting for a bay, a print
        is active, or no bay is left to offer.

        :param eventtime: reactor time of the watch tick
        """
        q = getattr(self, "_popup_queue", None)
        if not q or eventtime < getattr(self, "_popup_active_until", 0.0):
            return
        while q:
            ev = q.pop(0)
            if ev[0] == "new":
                if self._bay_of_uid(ev[1]) is None:
                    continue                  # gone before its turn; skip
                self._prompt_new_unit(ev[1], timeout=self._POPUP_HOLD)
            elif ev[0] == "replace":
                if (ev[1] not in self._no_bay
                    or self._is_printing()
                    or not self._prompt_replace_unit(
                        ev[1], eventtime, timeout=self._POPUP_HOLD,
                        auto=True)):
                    self._replace_offered.discard(ev[1])
                    continue
            else:
                self._prompt_removed_unit(ev[1], ev[2], timeout=self._POPUP_HOLD)
            self._popup_active_until = eventtime + self._POPUP_HOLD
            return

    #: Seconds a bound unit's offline reads must span for its return to count
    #: as a re-plug; a lone blip spans nothing.
    _REPLUG_ABSENCE = 2.0

    def _restore_returned(self, uid: str, since: float, gone_s: float) -> None:
        """
        Have a unit that came back while still bound restore its follower.

        A unit that stays bound through an absence is never claimed again, so
        nothing else re-engages the follower an AMS drops when it loses power.
        The unit decides whether it owns AFC's loaded lane and whether the
        follower already has an owner (see restore_follower_on_return), and
        prints its own line when it engages. The return itself is logged at
        debug, so a unit with nothing to engage puts nothing on the console.

        :param uid: the returning unit's uid
        :param since: reactor time its settled online run began
        :param gone_s: how long it read offline, first read to last
        """
        name = next((pu.get("name") for pu in self._pool_units
                     if pu.get("bound") == uid), None)
        unit = (self.printer.lookup_object(f"AFC_BambuAMS {name}", None)
                if name else None)
        restore = getattr(unit, "restore_follower_on_return", None)
        if restore is None:
            return
        try:
            engaged = bool(restore(since))
        except Exception as e:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: follower restore for {name} "
                f"(UID {uid}) on its return failed: {e}")
            return
        self.logger.debug(
            f"AFC_BridgeBox {self.name}: {name} (UID {uid}) is back after "
            f"{gone_s:.0f}s offline and stayed bound; its loaded-lane "
            f"follower restore {'engaged' if engaged else 'engaged nothing'}.")

    def _toolhead_lanes(self, uid: str) -> List[Any]:
        """
        The lanes of the unit bound to ``uid`` that AFC records as loaded to a
        toolhead (see lane_in_toolhead).

        Only assigned lanes count, as those are the ones a release pools.

        :param uid: the bound unit's uid
        :return list: those lanes, empty when there are none or nothing is
            bound to ``uid``
        """
        uid = _norm_uid(uid)
        afc = self.printer.lookup_object("AFC", None)
        for pu in getattr(self, "_pool_units", []):
            if pu.get("bound") != uid:
                continue
            lanes = (self.printer.lookup_object(f"AFC_lane {n}", None)
                     for n in pu["lanes"])
            return [ln for ln in lanes
                    if ln is not None
                       and not getattr(ln, "unassigned", True)
                       and lane_in_toolhead(ln, afc)]
        return []

    def _refuse_toolhead_release(self, gcmd: Any, cmd: str, uid: str) -> None:
        """
        Refuse a command that releases the unit bound to ``uid`` while AFC
        records one of its lanes as loaded to a toolhead, unless FORCE=1.

        With FORCE=1 the release clears that record (see _release_pool_unit).

        :param gcmd: the command being run
        :param cmd: its name, for the error
        :param uid: the bound unit's uid
        """
        held = self._toolhead_lanes(uid)
        if held and not gcmd.get_int("FORCE", 0):
            names = [ln.name for ln in held]
            error_str = (
                f"{cmd}: AFC records {', '.join(names)} on {uid} as loaded "
                f"to the toolhead -- unload it first "
                f"({self._unset_hint(names)} if the filament is already "
                f"out), or FORCE=1 to clear it from the toolhead and release "
                f"anyway"
                + (" (a print is active, and FORCE=1 also pulls the lane "
                   "out from under it)" if self._is_printing() else ""))
            raise gcmd.error(error_str)

    def _unset_hint(self, lanes: List[str]) -> str:
        """
        The UNSET_LANE_LOADED step for a loaded lane whose filament is
        already out.

        UNSET_LANE_LOADED clears the active tool's lane only, so for a lane
        another toolhead records the step names that toolhead.

        :param lanes: the loaded lanes' names
        :return str: "UNSET_LANE_LOADED", or "UNSET_LANE_LOADED with <tool>
            as the active tool"
        """
        afc = self.printer.lookup_object("AFC", None)
        tools = getattr(afc, "tools", None) or {}
        exts = [n for n, e in tools.items()
                if getattr(e, "lane_loaded", None) in set(lanes)]
        try:
            current = afc.function.get_current_extruder()
        except Exception:
            current = None
        other = [n for n in exts if n != current]
        if not other or (current is None and len(tools) <= 1):
            return "UNSET_LANE_LOADED"
        return (f"UNSET_LANE_LOADED with {' and '.join(other)} as the active "
                f"tool")

    def _unclaimed_loaded_text(self, cmd: str, bay: str, old: str,
                               loaded: List[str], act: str) -> str:
        """
        The refusal for freeing an unclaimed bay AFC records a lane of as
        loaded to a toolhead.

        :param cmd: the command's name
        :param bay: the bay's name
        :param old: the uid it is held for
        :param loaded: those lanes
        :param act: what FORCE=1 then does, e.g. "replaces AAAA"
        :return str: the error text
        """
        return (f"{cmd}: AFC records {', '.join(loaded)} on {bay} as loaded "
                f"to the toolhead, from {old}. While {bay} is unclaimed its "
                f"lanes are not registered, so no unload or UNSET_LANE_LOADED "
                f"reaches them -- plug {old} back in and unload it, or take "
                f"the filament out by hand and FORCE=1 clears the record and "
                f"{act}")

    def _refuse_unclaimed_release(self, gcmd: Any, cmd: str,
                                  pu: Dict[str, Any], act: str) -> List[str]:
        """
        Refuse a command that frees an unclaimed bay held for a unit while AFC
        records one of its lanes as loaded to a toolhead, or before PREP has
        restored that record (see _prep_settled), unless FORCE=1.

        The unit that claims the bay next would take the record as its own
        lane's and start its follower on it (see
        AFC_BambuAMS._restore_loaded_follower).

        :param gcmd: the command being run
        :param cmd: its name, for the error
        :param pu: the unbound pool bay the command frees
        :param act: what FORCE=1 then does, for the error
        :return list: the loaded lanes, which the caller clears with
            _clear_bay_toolhead once nothing else refuses
        """
        loaded = self._bay_loaded(pu)
        if gcmd.get_int("FORCE", 0):
            return loaded
        if not self._prep_settled():
            error_str = (
                f"{cmd}: PREP has not run yet, so which lane AFC records as "
                f"loaded to the toolhead is not known -- run this again once "
                f"it has")
            raise gcmd.error(error_str)
        if loaded:
            error_str = self._unclaimed_loaded_text(
           cmd, pu["name"], _norm_uid(pu.get("uid")), loaded, act)
            raise gcmd.error(error_str)
        return []

    def _loaded_clause(self, pu: Optional[Dict[str, Any]]) -> str:
        """
        What a FORGET suggested for a bay's offline unit needs first when
        AFC records a lane of that bay as loaded to a toolhead.

        :param pu: the bay, or None
        :return str: the clause, "" when no lane of it is recorded
        """
        loaded = self._bay_loaded(pu) if pu is not None else []
        if not loaded:
            return ""
        if pu.get("bound"):
            return (f" (AFC records {', '.join(loaded)} as loaded to the "
                    f"toolhead: unload it first, "
                    f"{self._unset_hint(loaded)} if the filament is already "
                    f"out)")
        return (f" (AFC records {', '.join(loaded)} as loaded to the "
                f"toolhead: plug it back in and unload it, or take the "
                f"filament out by hand and add FORCE=1, which clears the "
                f"record)")

    def _new_ams_with_no_bay(self, entries: List[str]
                             ) -> Dict[str, List[Tuple[str, str]]]:
        """
        The AMS just recorded that no restart gives a bay: each is off every
        bay and the roster a restart builds from puts four other AMS ahead of
        it (see _ams_ahead).

        They are not told to RESTART, or to add themselves to roster:, but what
        frees a bay (see _tell_no_bay).

        :param entries: the roster entries just recorded, `<model>:<uid>`
        :return dict: entry -> (name, uid) of the AMS ahead of it
        """
        out: Dict[str, List[Tuple[str, str]]] = {}
        for e in entries:
            u = _norm_uid(e.partition(":")[2])
            if e.startswith("ht:") or self._bay_of_uid(u):
                continue
            ahead = self._ams_ahead(u)
            if len(ahead) >= _MAX_AMS_BAYS:
                out[e] = ahead
        return out

    def _tell_no_bay(self, no_bay: Dict[str, List[Tuple[str, str]]]) -> None:
        """
        Say once, for each AMS _new_ams_with_no_bay found, that it has no bay
        and what frees one.

        An AMS the claim or the ready note already told is not told again.

        :param no_bay: entry -> (name, uid) of the AMS ahead of it
        """
        for e, ahead in no_bay.items():
            u = _norm_uid(e.partition(":")[2])
            if u in self._no_bay_told:
                continue
            self._no_bay_told.add(u)
            self.logger.info(
                f"AFC_BridgeBox {self.name}: NEW AMS on the chain: {e} -- "
                f"recorded, but it has no bay: "
                + self._all_ams_bays_held(u, ahead, live=True))

    def _announce_enrolled(self, entries: List[str]) -> None:
        """
        Log the units just recorded on a pooled chain.

        Each unit on a bay is saved there by the _pin_recorded_units call
        that follows, which logs the save or why not, so this line says only
        which units have no bay yet (none free of their family, or not
        claimed yet) and what a restart still adds: the temperature card of
        a unit on a spare bay, which is built without one. An AMS the claim
        already told it has no bay (see _no_bay_message) is not told again.

        :param entries: the roster entries just recorded, `<model>:<uid>`
        """
        on_bay = {_norm_uid(pu.get("bound")) for pu in self._pool_units}
        on_spare = {_norm_uid(pu.get("bound")) for pu in self._pool_units
                    if pu.get("spare")}
        uids = [_norm_uid(e.partition(":")[2]) for e in entries]
        waiting = [u for u in uids
                   if u not in on_bay and u not in self._no_bay_told]
        placed = [u for u in uids if u in on_spare]
        text = (f"AFC_BridgeBox {self.name}: NEW unit(s) on the chain: "
                f"{', '.join(entries)} -- recorded.")
        if len(waiting) == 1:
            text += (f" {waiting[0]} has no bay yet and takes the next free "
                     f"one of its family.")
        elif waiting:
            text += (f" {', '.join(waiting)} have no bay yet and take the "
                     f"next free ones of their family.")
        if placed and waiting:
            text += (f" A restart adds the temperature card"
                     f"{'s' if len(placed) > 1 else ''} of "
                     f"{', '.join(placed)}.")
        elif placed:
            text += (" A restart adds its temperature card."
                     if len(placed) == 1
                     else " A restart adds their temperature cards.")
        self.logger.info(text)

    def _save_refusal(self, pu: Dict[str, Any], uid: str,
                      recorded: Optional[set] = None
                      ) -> Optional[Tuple[str, str, bool]]:
        """
        Why ``uid`` cannot be saved on bay ``pu`` (see _pin_recorded_units).

        A uid saved on another bay of this session goes back to that bay at
        restart. A bay is never saved where that would stop the next boot:
        for a uid recorded as the other family (an HT bay has one lane, an
        AMS bay four), or under a name saved for another recorded uid, which
        gets that bay at restart. A name kept for a uid outside the roster
        holds no bay, and saving takes it over (see _persist_pin).

        :param pu: pool unit record
        :param uid: the uid on it
        :param recorded: the recorded uids; read when not given
        :return tuple: (reason, text, True for a warning), or None when it can be
            saved there
        """
        name = pu["name"]
        saved = getattr(self, "_name_map", {}).get(uid)
        if (saved not in (None, name)
            and any(p.get("name") == saved for p in self._pool_units)):
            return ("elsewhere",
                    f"{uid} is on {name} this session, not on {saved}, the "
                    f"bay it is saved on; it comes back to {saved} after a "
                    f"restart.", False)
        model = self._model_for_uid(uid)
        fam = (("ht" if model in _HT_MODELS_BB else "ams") if model
               else pu["family"])
        if fam != pu["family"]:
            return ("family",
                    f"{uid} is recorded as {model} but is on "
                    f"{pu['family'].upper()} bay {name}; not saving it. "
                    f"AFC_BRIDGEBOX_UNASSIGN CHAIN={self.name} UID={uid} "
                    f"FORCE=1 re-homes it.",
                    True)
        holder = self._name_holder(name, uid, recorded)
        if holder:
            return ("holder",
                    f"{name} is saved for {holder}, so {uid} is not saved on "
                    f"it. AFC_BRIDGEBOX_FORGET CHAIN={self.name} "
                    f"UID={holder} or AFC_BRIDGEBOX_UNASSIGN "
                    f"CHAIN={self.name} UID={holder}, or AFC_BRIDGEBOX_ASSIGN "
                    f"CHAIN={self.name} UID={uid} NAME=<other bay> moves "
                    f"{uid} to another bay.",
                    True)
        return None

    def _pin_recorded_units(self, recorded: set, eventtime: float,
                            online_since: Dict[str, float]) -> None:
        """
        Save the bay of each recorded unit that holds one, so a restart
        brings it back on the same bay, lanes and T#.

        A unit claimed live onto a spare wears that bay only for the session:
        at restart the recorded roster gives it a bay by its saved name, and
        with none it would draw the lowest free one. So once a bound uid is
        recorded (listed in roster: when that is set) and has been online
        for enroll_grace (the continuous run enrollment asks for, so a
        phantom online flag never saves a bay), its bay is pinned (see
        _persist_pin) and held from then on (see _bay_held). A uid already
        saved on the bay it holds is left alone, and so is one saved on
        another bay of this session: it goes back there at restart. A saved
        name that is no bay this session is replaced by the bay it holds.

        Where _save_refusal refuses the bay, that is logged once and checked
        again every tick, so the save lands on the tick after the operator
        clears it.

        :param recorded: the uids that get a bay of their own at restart
        :param eventtime: reactor time
        :param online_since: uid -> start of its continuous online run
        """
        warned = getattr(self, "_pin_warned", None)
        if warned is None:
            warned = self._pin_warned = set()
        for pu in sorted(getattr(self, "_pool_units", []), key=_pool_laneno):
            uid = _norm_uid(pu.get("bound"))
            if (not uid
                or uid not in recorded
                or uid not in online_since
                or eventtime - online_since[uid] < self.enroll_grace):
                continue
            name = pu["name"]
            if self._name_map.get(uid) == name:
                continue                      # saved on it already
            refusal = self._save_refusal(pu, uid, recorded)
            if refusal is not None:
                reason, text, warn = refusal
                if (uid, name, reason) not in warned:
                    warned.add((uid, name, reason))
                    (self.logger.warning if warn else self.logger.info)(
                        f"AFC_BridgeBox {self.name}: {text}")
                continue
            model = self._model_for_uid(uid)
            first = int("".join(c for c in pu["lanes"][0] if c.isdigit()))
            span = len(pu["lanes"])
            self._persist_pin(uid, first, span, name, model or (
                "ht" if pu["family"] == "ht" else "boxed"))
            pu["uid"] = uid
            last = first + span - 1
            where = (f"lane{first}, T{first}" if span == 1
                     else f"lane{first}-lane{last}, T{first}-T{last}")
            self.logger.info(
                f"AFC_BridgeBox {self.name}: saved {uid} on {name} ({where}); "
                f"it comes back there after a restart.")

    def _release_pool_unit(self, uid: str, why: Optional[str] = None) -> None:
        """
        Release the pool unit bound to ``uid``: the hot-unplug half.

        Drops its lanes' T# (by reverse lookup on tool_cmds, since lane.map
        surfaces as None on the AFC_stepper view) and returns each lane + the
        unit object to idle, live, no restart. A bay whose uid is saved on it
        (see _bay_held) keeps that uid so a re-plug re-claims the same
        lanes/name/T#; any other goes back to the generic pool (uid cleared). No
        relink, no survivor is touched. The lanes' records are held for this
        unit's next claim of the bay (see _bay_records).

        A lane AFC records in a toolhead has that record cleared before it is
        pooled (see unset_tool_loaded) and the vars saved. Callers decide
        whether that may happen: auto-drop keeps such a unit claimed, and the
        commands refuse it without FORCE=1 (see _refuse_toolhead_release).

        :param uid: the UID of the unit to release
        :param why: the command releasing it; None for an unplug. A command
            clears or moves the bay's uid itself right after, so its log line
            names the command and promises no re-plug.
        """
        uid = _norm_uid(uid)
        afc = self.printer.lookup_object("AFC", None)
        held = getattr(self, "_drop_held", None)
        if held is not None:
            held.discard(uid)
        for pu in getattr(self, "_pool_units", []):
            if pu.get("bound") != uid:
                continue
            gcode = self.printer.lookup_object("gcode", None)
            # Taken while the lanes are still registered and mapped, and
            # before a toolhead record is cleared from them.
            recs = self._bay_records(pu)
            # A T# a claim during a print left in use is not taken for a lane
            # that is gone.
            for lname in pu["lanes"]:
                (getattr(self, "_deferred_takes", None) or {}).pop(lname, None)
            cleared = []
            for lname in pu["lanes"]:
                lane = self.printer.lookup_object(f"AFC_lane {lname}", None)
                if lane is None or getattr(lane, "unassigned", True):
                    continue
                # Drop this lane's T#s from tool_cmds and unregister their
                # macros, or TcmdAssign pushes the re-claim onto the next free
                # set.
                cmds = set(getattr(lane, "map", None) or [])
                if afc is not None:
                    for m, owner in list(afc.tool_cmds.items()):
                        if owner == lane.name:
                            cmds.add(m)
                            afc.tool_cmds.pop(m, None)
                for cmd in cmds:
                    if cmd and cmd != "NONE" and gcode is not None:
                        try: gcode.register_command(cmd, None)
                        except Exception: pass
                try:
                    if unset_tool_loaded(lane, afc):
                        cleared.append(lane.name)
                except Exception: pass
                try: deactivate_to_pool(lane)
                except Exception: pass
            unit = self.printer.lookup_object(
                f"AFC_BambuAMS {pu['name']}", None)
            if unit is not None:
                try: unit.release()
                except Exception: pass
            pu["bound"] = None
            # The absence ends with the binding: a reclaim schedules its own
            # follower restore.
            getattr(self, "_away", {}).pop(uid, None)
            if recs:
                self._held = getattr(self, "_held", None) or {}
                self._held[pu["name"]] = {"uid": uid, "lanes": recs}
            # Save lane_loaded cleared, or the next boot restores the pooled
            # lane as loaded.
            if cleared and afc is not None:
                try: afc.save_vars()
                except Exception: pass
            held_bay = self._bay_held(pu)
            if not held_bay:
                pu["uid"] = None                        # unsaved -> generic pool
            self.logger.info(
                f"AFC_BridgeBox {self.name}: released {pu['name']} (UID {uid}"
                + (f", {why}" if why
                   else f" offline >{self.release_grace:.0f}s")
                + "); lanes dropped live"
                + (", slot kept for re-plug" if held_bay and not why else "")
                + (f"; cleared {', '.join(cleared)} from the toolhead"
                   if cleared else ""))
            return

    def _bay_records(self, pu: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        """
        The lane records a release holds for its unit's next claim of the bay.

        Each claimed lane's record is the one save_vars writes for it, so a
        held record and one captured from AFC.var.unit read alike. The claim
        put the records it was handed on the lanes (see
        AFC_BambuAMS._apply_held_lanes), so a lane's own record is its
        newest, even before scan priming, and carries what was set on it
        since. A lane whose T# take waits for the print is held with the map
        its claim planned (see _planned_maps).

        :param pu: the pool bay being released
        :return dict: lane name -> record
        """
        recs: Dict[str, Dict[str, Any]] = {}
        for lname in pu["lanes"]:
            lane = self.printer.lookup_object(f"AFC_lane {lname}", None)
            if lane is None or getattr(lane, "unassigned", True):
                continue
            try:
                rec = copy.deepcopy(lane.get_status(save_to_file=True))
            except Exception:
                continue
            if isinstance(rec, dict) and rec:
                recs[lname] = rec
        self._planned_maps(recs)
        return recs

    def _last_bay(self, uid: str,
                  family: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        The free pool bay ``uid`` was last claimed onto, if any (see
        _set_bay_owner).

        :param uid: a unit uid
        :param family: "ams" or "ht" to require that family, None for either
        :return dict: the bay, when it is unbound and reserved for no unit
        """
        uid = _norm_uid(uid)
        owners = self._owners()
        return next((pu for pu in getattr(self, "_pool_units", None) or []
                     if pu.get("bound") is None
                        and pu.get("uid") is None
                        and (family is None or pu.get("family") == family)
                        and owners.get(pu.get("name")) == uid), None)

    def _live_access_tracking(self) -> dict:
        """
        The printer's own config access-tracking dict, or a throwaway.

        Klipper has moved this between objects (it lives on configfile.validate
        in current versions and on configfile itself in older ones), so it is
        looked up by shape rather than by path. A miss only leaves fabricated
        sections absent from configfile.settings and never stops a chain
        coming up.

        :return dict: the live tracking dict, or a fresh one if not found
        """
        try:
            cf = self.printer.lookup_object("configfile", None)
            for owner in (getattr(cf, "validate", None), cf):
                at = getattr(owner, "access_tracking", None)
                if isinstance(at, dict):
                    return at
        except Exception as e:
            self.logger.debug(
                f"AFC_BridgeBox {self.name}: no access tracking to share "
                f"({e}); fabricated sections stay out of configfile.settings")
        return {}

    def _claim_pool_unit(self, uid: str, model: str) -> Any:
        """
        Bind a free pool unit to ``uid`` and bring it fully online, live.

        Activates the pool unit's lanes (registers them everywhere), mints
        their T# commands, and calls the unit's claim() to join the bus, then
        gives the unit the uid's learned values (see _apply_learned). The
        binding itself is saved by _pin_recorded_units once the uid is
        recorded and has been online for enroll_grace: a restart brings every
        pool unit back up idle, and the roster gives the uid its own named
        slot, which it re-claims when seen online. Which unit each bay was
        last claimed onto is saved at claim (bay_owner, see _set_bay_owner).

        The bay is the one whose uid is this uid, else a free bay of its
        family: the one wearing the name the uid is saved under (a unit whose
        saved name a spare wears this session comes back to it), else the one
        it was last claimed onto (see _last_bay: its lane records are held for
        it there and nowhere else) unless another unit is coming back to it,
        else the lowest one no other unit is coming back to, else the lowest.
        Another unit is coming back to a bay wearing its saved name when it
        is recorded (it gets that bay at restart, so this uid could not be
        saved there) or online and not yet claimed. A free bay adopted here
        is handed back if the claim does not go through.

        Nothing is claimed before PREP has run (see _prep_settled). The unit
        is handed the lane records held for it on this bay (see _held_for),
        which its lanes take at the claim, before the save below, and each
        lane's T# map comes from its record (see _map_claimed_lanes). During
        a print the claim takes no T# another live lane or a macro holds: the
        lane waits for it until the print ends (see _take_deferred_tools).

        :param uid: the new physical AMS's UID
        :param model: its generation tag (ht/ams1/ams2/boxed)
        :return object: the claimed unit object, or None if none was free/possible
        """
        uid = _norm_uid(uid)
        if not uid:
            return None
        afc = self.printer.lookup_object("AFC", None)
        if afc is None:
            return None
        if not self._prep_settled():
            return None
        pools = getattr(self, "_pool_units", [])
        if any(pu.get("bound") == uid for pu in pools):
            return None                       # already live on a pool unit
        family = "ht" if model == "ht" else "ams"
        # Prefer the slot this uid owns (a known unit's named slot, so it comes
        # back on the same lanes/name/T#); else the next free spare of the
        # right family, which this uid then adopts.
        chosen = next((pu for pu in pools
                       if pu.get("bound") is None and pu.get("uid") == uid),
                      None)
        adopted = False
        if chosen is None:
            free = [pu for pu in pools
                    if pu.get("bound") is None
                       and pu.get("uid") is None
                       and pu.get("family") == family]
            names = getattr(self, "_name_map", {})
            chosen = next((pu for pu in free
                           if pu.get("name") == names.get(uid)), None)
            if chosen is None and free:
                recorded = self._recorded_uids()
                waiting = (set(getattr(self, "_online_since", None) or ())
                           - {_norm_uid(pu.get("bound")) for pu in pools})
                taken = {n for u, n in names.items()
                         if u != uid and (u in recorded or u in waiting)}
                last = self._last_bay(uid, family)
                if last is not None and last.get("name") not in taken:
                    chosen = last
                else:
                    chosen = next((pu for pu in free
                                   if pu.get("name") not in taken), free[0])
            if chosen is not None:
                chosen["uid"] = uid           # the spare now belongs to this uid
                adopted = True
        if chosen is None:
            # Said once per wait: the watch retries every tick, and the unit
            # claims the first bay of its family that frees.
            self._no_bay[uid] = model
            if uid not in self._no_bay_told:
                self._no_bay_told.add(uid)
                self.logger.warning(self._no_bay_message(uid, family))
            return None
        unit = self.printer.lookup_object(
            f"AFC_BambuAMS {chosen['name']}", None)
        if unit is None or not getattr(unit, "pool", False):
            if adopted:
                chosen["uid"] = None          # hand the bay back
            return None
        held = self._held_for(uid, chosen["name"])
        # Clear stale T#/map entries for these lanes before assigning, except
        # entries another live lane owns. Lanes get their T# only after the
        # claim succeeds (see _map_claimed_lanes).
        live = getattr(afc, "lanes", None) or {}
        for lname in chosen["lanes"]:
            lane = self.printer.lookup_object(f"AFC_lane {lname}", None)
            if lane is None:
                continue
            for m in list(getattr(lane, "map", None) or []):
                if afc.tool_cmds.get(m) not in live:
                    afc.tool_cmds.pop(m, None)
            for m, owner in list(afc.tool_cmds.items()):
                if owner == lname:
                    afc.tool_cmds.pop(m, None)
            ds = "".join(c for c in lname if c.isdigit())
            if ds and afc.tool_cmds.get("T" + ds) not in live:
                afc.tool_cmds.pop("T" + ds, None)
            lane.map = []
            lane._map = []
            lane.current_map = ""
        activated = []
        for lname in chosen["lanes"]:
            lane = self.printer.lookup_object(f"AFC_lane {lname}", None)
            if lane is None:
                continue
            try:
                activate_from_pool(lane)
                activated.append(lane)
            except Exception as ex:
                self.logger.warning(
                    f"AFC_BridgeBox {self.name}: pool lane {lname} "
                    f"activate failed: {ex}")
        # The unit has no other way back to us: it is fabricated, so it never
        # parsed a config section naming this master. Set before claim() so
        # anything the claim itself learns can already be persisted.
        try: unit.set_master(self)
        except Exception: pass
        # Hand the held records over before claim() so they are on the lanes
        # before any save; a unit without the setter reads AFC.var.unit
        # instead.
        hold = getattr(unit, "hold_lanes", None)
        if callable(hold):
            try: hold(held)
            except Exception: pass
        # measure_on_insert is resolved before claim(), which leaves it alone:
        # the index adoption the claim starts sends it to the firmware.
        self._apply_model_measure(unit, model, bool(chosen.get("spare")))
        # From the lanes' registration on, every save writes each lane not
        # mapped yet with the map it comes back with (see _unmapped_maps).
        self._claim_plans = self._claim_map_plans(activated, held)
        if not unit.claim(uid, model):
            self._claim_plans = {}
            for lane in activated:
                try: deactivate_to_pool(lane)
                except Exception: pass
            if callable(hold):
                try: hold({})
                except Exception: pass
            if adopted:
                chosen["uid"] = None          # hand the bay back on failure
            return None
        # Before any T# work: TcmdAssign saves the vars, and AFC.var.unit
        # must not hold this unit's lanes under a bay recorded as another's.
        self._set_bay_owner(chosen["name"], uid)
        try:
            remapped = self._map_claimed_lanes(
                afc, activated, held, hold_tools=self._is_printing())
        finally:
            self._claim_plans = {}
        # Each lane's data goes to Moonraker under the T# the map gave it, as
        # PREP's lane test sends it for every lane it walks.
        for lane in activated:
            push = getattr(lane, "send_lane_data", None)
            if callable(push):
                try: push()
                except Exception: pass
        # AFC.var.unit carries this unit's lanes, their records and their maps
        # from here, whether or not a TcmdAssign (which saves) ran for them.
        try: afc.save_vars()
        except Exception: pass
        self._clear_standalone(activated)
        chosen["bound"] = uid
        self._no_bay_told.discard(uid)
        self._no_bay.pop(uid, None)
        self._replace_offered.discard(uid)
        # Give the unit the dryer a restart would give this model.
        self._apply_model_dryer(unit, model)
        self._apply_learned(unit, uid, model)
        self.logger.info(
            f"AFC_BridgeBox {self.name}: CLAIMED {uid} as {model} onto "
            f"{chosen['name']} ({len(activated)} lanes) -- live, no restart."
            + (f" Saved maps: {', '.join(remapped)}." if remapped else ""))
        return unit

    @staticmethod
    def _claim_map_plans(lanes: List[Any], held: Dict[str, Dict[str, Any]]
                         ) -> Dict[str, Tuple[str, str]]:
        """
        The map each lane a claim registers comes back with at a restart
        before the claim maps it (see _unmapped_maps).

        :param lanes: the lanes the claim activated
        :param held: lane name -> record handed to this claim
        :return dict: lane name -> (map, current_map) as save_vars writes
            them: the record's saved map, else the lane's home T#; none for
            a lane whose record saved NONE, which is what it comes back with
        """
        plans: Dict[str, Tuple[str, str]] = {}
        for lane in lanes:
            rec = held.get(lane.name) or {}
            saved = _parse_map(rec.get("map"))
            home = _home_tool(lane.name)
            if saved:
                cur = rec.get("current_map")
                plans[lane.name] = (", ".join(saved),
                                    cur if cur in saved else saved[0])
            elif home and not rec.get("map"):
                plans[lane.name] = (home, home)
        return plans

    @staticmethod
    def _tool_in_use(afc: Any, lname: str, cmd: str) -> Optional[str]:
        """
        What a claimed lane would take ``cmd`` from: another live lane that
        holds it (see _tcmd_holder), or a macro other than AFC's CHANGE_TOOL,
        which TcmdAssign renames out of the way when force_assign_map or a
        config map lets it.

        :param afc: the AFC object
        :param lname: the lane that wants the T#
        :param cmd: the T# command
        :return str: the holding lane's name, "a macro", or None when nothing
            holds it
        """
        holder = _tcmd_holder(afc, cmd)
        if holder is not None and holder != lname:
            return holder
        handler = _tcmd_handlers(afc).get(cmd)
        if handler is not None and not _tcmd_is_ours(afc, handler):
            return "a macro"
        return None

    def _print_wait(self) -> str:
        """
        What a T# a claim leaves in use waits for, as the console says it.

        :return str: "the print ends" while print_stats reports a print, else
                     "the printer is idle"
        """
        try:
            obj = self.printer.lookup_object("print_stats", None)
            now = self.printer.get_reactor().monotonic()
            if (obj is not None
                and obj.get_status(now).get("state") in ("printing", "paused")):
                return "the print ends"
        except Exception:
            pass
        return "the printer is idle"

    def _take_deferred_tools(self) -> None:
        """
        Give each lane a claim during a print left off a T# in use the map
        its claim planned (see _map_claimed_lanes), once no print is running.

        The home tool is taken as at any claim (_rehome_holder,
        _take_home_tool); a saved T# another lane took meanwhile is left
        with it, and a T# the lane was given meanwhile stays. A lane released
        since is skipped, and a lane a mapping reset has numbered since (a
        PRINT_END's AFC_RESET_MAPPING, see _reset_home_plan) has no take
        left. Runs from the chain watch; a no-op while printing.
        The line saying so goes to AFC.log only when every wait was a Bambu
        lane's home T# held by another Bambu lane, which ends on its own
        (see _say_deferred_tools).
        """
        pending = getattr(self, "_deferred_takes", None)
        if not pending or self._is_printing():
            return
        afc = self.printer.lookup_object("AFC", None)
        if afc is None:
            return
        done: List[str] = []
        quiet = True
        for lname, entry in list(pending.items()):
            del pending[lname]
            lane = (getattr(afc, "lanes", None) or {}).get(lname)
            if lane is None:
                continue
            home = entry["home"]
            holder = _tcmd_holder(afc, home)
            quiet = (quiet
                     and bool(entry.get("quiet"))
                     and (holder in (None, lname)
                          or self._bambu_home_move(afc, lname, home, holder)))
            now = [m for m in (getattr(lane, "map", None) or [])
                   if m != "NONE"]
            target = [m for m in entry["maps"]
                      if m == home or m in now or _tcmd_free(afc, lname, m)]
            target += [m for m in now if m not in target]
            if not target:
                continue
            current = next((c for c in (entry["current"],
                                        getattr(lane, "current_map", ""))
                            if c in target), target[0])
            if home in target:
                lane.map, lane.current_map = [home], home
                self._rehome_holder(afc, lname, home)
                self._take_home_tool(afc, lane)
            lane.map, lane.current_map = target, current
            try:
                assign_pool_tcmd(lane, afc)
            except Exception as ex:
                self.logger.warning(
                    f"AFC_BridgeBox {self.name}: TcmdAssign {lname} failed: "
                    f"{ex}")
            push = getattr(lane, "send_lane_data", None)
            if callable(push):
                try: push()
                except Exception: pass
            done.append(f"{lname} is {', '.join(target)}")
        if not done:
            return
        try: afc.save_vars()
        except Exception: pass
        (self.logger.debug if quiet else self.logger.info)(
            f"AFC_BridgeBox {self.name}: the printer is idle, so the T#s the "
            f"claim left in use are taken: {'; '.join(done)}.")

    def _restart_roster(self) -> List[Tuple[str, str]]:
        """
        The roster a restart builds from, in order.

        :return list: (model, uid) pairs from the roster: option when set, else the
                      recorded roster
        """
        if self._roster_source == "option":
            return [(u["model"], u["uid"]) for u in self.units]
        raw = self._state_get(self._BASE_SECTION + " " + self.name,
                              "roster") or ""
        out = []
        for e in raw.split(","):
            m, _sep, u = e.partition(":")
            if u.strip():
                out.append((m.strip().lower(), _norm_uid(u)))
        return out

    def _ams_ahead(self, uid: str) -> List[Tuple[str, str]]:
        """
        The AMS a restart gives the AMS bays to before ``uid``, in the order
        pass 1b of _roster_sections names them: saved names first, then
        recorded lanes, then roster order, where a uid the roster does not
        list comes last.

        :param uid: an AMS uid
        :return list: (name, uid) of at most _MAX_AMS_BAYS units, name ""
            for one not yet named; that many means a restart builds ``uid``
            no AMS bay
        """
        uid = _norm_uid(uid)
        roster = self._restart_roster()
        order = [u for _m, u in roster]
        pos = order.index(uid) if uid in order else len(order)
        ranked = []
        for i, (model, u) in enumerate(roster):
            if u == uid or model in _HT_MODELS_BB:
                continue
            tier = (0 if u in self._name_map
                    else 1 if u in self._lane_map else 2)
            if tier < 2 or i < pos:
                ranked.append((tier, i, u))
        return [(self._name_map.get(u, ""), u)
                for _t, _i, u in sorted(ranked)[:_MAX_AMS_BAYS]]

    def _no_bay_message(self, uid: str, family: str) -> str:
        """
        The console line for a unit that finds no free bay of its family.

        When the roster a restart builds from gives the four AMS bays to
        other units, or four AMS bays are built and the claim found each of
        them held, _all_ams_bays_held says what can be done: a Bambu bus
        addresses no fifth AMS, so neither a restart nor pool_ams gives this
        one a bay. A unit a set roster: option does not list is told to add
        it there, when the option leaves it room. Otherwise, or for an HT, a
        restart builds the recorded unit a bay of its own; an AMS bay past
        the AMS band moves the HT band up. A bay of its family held for a
        unit that is offline is named first, with the FORGET that frees it
        (and what clears a loaded-lane record on it first, see
        _loaded_clause) and, without a roster: option, the
        AFC_BRIDGEBOX_REPLACE that puts this unit on it (see _replace_text).

        :param uid: the waiting unit's uid
        :param family: "ams" or "ht"
        :return str: the message
        """
        if family == "ams":
            ahead = self._ams_ahead(uid)
            if len(ahead) >= _MAX_AMS_BAYS:
                return (f"AFC_BridgeBox {self.name}: AMS {uid} has no bay: "
                        + self._all_ams_bays_held(uid, ahead, live=True))
        pools = sorted(getattr(self, "_pool_units", []), key=_pool_laneno)
        ams = [pu for pu in pools if pu.get("family") == "ams"]
        fam, pool = (("HT", "pool_ht") if family == "ht"
                     else ("AMS", "pool_ams"))
        head = (f"AFC_BridgeBox {self.name}: new {fam} {uid} has no free "
                f"bay: every {fam} bay built belongs to a known unit.")
        # An AMS reaches this with fewer than four AMS in roster: (see
        # _ams_ahead), so once listed a restart builds it a bay.
        if self._unlisted(uid):
            return (f"{head} roster: is set and does not list it: add "
                    f"{self._roster_entry(uid)} to roster: and RESTART to "
                    f"give it a bay.")
        if family == "ams" and len(ams) >= _MAX_AMS_BAYS:
            # Each AMS bay holds a claimed unit or one saved for it, so a
            # restart gives this AMS none either.
            return (f"AFC_BridgeBox {self.name}: AMS {uid} has no bay: "
                    + self._all_ams_bays_held(uid, [
                        (pu["name"],
                         _norm_uid(pu.get("uid") or pu.get("bound")))
                        for pu in ams], live=True))
        offline = (self._held_offline(family)
                   if (self.pool_ams or self.pool_ht)
                   and self._roster_source != "option" else [])
        units = [(pu["name"], _norm_uid(pu["uid"])) for pu in offline]
        forget = f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} UID="
        if len(units) == 1:
            head += (f" {units[0][0]} ({units[0][1]}) is offline: if this "
                     f"{fam} replaces it, {forget}{units[0][1]}"
                     f"{self._loaded_clause(offline[0])} frees that bay and "
                     f"this {fam} claims it live.")
        elif units:
            head += (f" Offline: {', '.join(f'{n} ({u})' for n, u in units)}."
                     f" If this {fam} replaces one of them, {forget}<that "
                     f"unit's uid> frees that bay and this {fam} claims it "
                     f"live.")
        head += self._replace_text(uid, offline)
        moves = ""
        if family == "ams" and any(pu.get("family") == "ht" for pu in pools):
            held = {(_pool_laneno(pu) - self.lane_base) // 4 for pu in ams}
            rank = min((r for r in range(_MAX_AMS_BAYS) if r not in held),
                       default=None)
            grow = (0 if rank is None
                    else rank + 1 - getattr(self, "_ams_band", rank + 1))
            if grow > 0:
                moves = (f", past the AMS band, which moves the HT lanes up "
                         f"{grow * 4}")
        return (f"{head} Once it is recorded, RESTART builds it one{moves}; "
                f"raise {pool} to keep spare {fam} bays for units plugged in "
                f"live"
                + (f" (at most {_MAX_AMS_BAYS} AMS)." if family == "ams"
                   else "."))

    def loaded_lane_moved(self, lane_name: str) -> bool:
        """
        Whether AFC's loaded-lane record for ``lane_name`` belongs to another
        unit than the one holding the lane in this layout.

        AFC saves an extruder's loaded lane by lane name alone. When this
        boot gives a lane to a different unit (a saved name redrawn, pool_ams
        clamped, an AMS left without a bay), that record names the old
        unit's filament. A claimed unit asks this before it repairs a lane's
        tool_loaded from the record and engages its follower (see
        AFC_BambuAMS._restore_loaded_follower). The answer stays True until
        no extruder records the lane (see _check_moved_loaded).

        :param lane_name: an AFC lane name
        :return bool: True while the record is the old unit's
        """
        return lane_name in (getattr(self, "_lane_moves", None) or {})

    def bay_records_uid(self, bay: str) -> Optional[str]:
        """
        Which unit a pool bay's saved lane records are attributed to.

        A claimed unit asks this about a lane AFC records in a toolhead that
        its own records do not show loaded (see
        AFC_BambuAMS._restore_toolhead_lane): records attributed to another
        unit make the filament that unit's, and none attributed leaves it
        unknown.

        :param bay: the pool bay's name
        :return str: the uid the records are held for, else the one bay_owner
            names for the bay; None when neither names one
        """
        entry = (getattr(self, "_held", None) or {}).get(bay) or {}
        return (_norm_uid(entry.get("uid"))
                or self._owners().get(bay)
                or None)

    def _check_moved_loaded(self) -> None:
        """
        Once PREP has restored AFC's loaded-lane records, warn once per lane
        about an extruder that records a lane this boot gave to another unit,
        and stop tracking every moved lane no extruder records.

        :return None: runs every watch tick; a no-op with no moved lanes
        """
        moves = getattr(self, "_lane_moves", None)
        if not moves:
            return
        afc = self.printer.lookup_object("AFC", None)
        if afc is None or not getattr(afc, "prep_done", False):
            return                  # PREP has not restored the records yet
        loaded: Dict[str, List[str]] = {}
        for ext_name, ext in (getattr(afc, "tools", None) or {}).items():
            ln = getattr(ext, "lane_loaded", None)
            if isinstance(ln, str) and ln in moves:
                loaded.setdefault(ln, []).append(ext_name)
        told: Set[str] = getattr(self, "_moved_told", None) or set()
        self._moved_told = told
        for ln in list(moves):
            if ln not in loaded:
                del moves[ln]
                told.discard(ln)
                continue
            if ln in told:
                continue
            told.add(ln)
            mv = moves[ln]
            fam = "HT" if mv["family"] == "ht" else "AMS"
            was = f"{fam} {mv['uid']}" + (f" ({mv['name']})" if mv["name"]
                                          else "")
            exts = " and ".join(loaded[ln])
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: {exts} records {ln} as loaded, "
                f"but {ln} belonged to {was}, and this boot's layout gives it "
                f"to {mv['now'] or 'no bay'}. The filament in {exts} is from "
                f"{mv['uid']}"
                + (f", so {mv['now']} leaves its follower off on {ln}"
                   if mv["now"] else "")
                + ". Unload that filament by hand and run "
                + self._unset_hint([ln]) + " to clear the record.")

    def _map_claimed_lanes(self, afc: Any, lanes: List[Any],
                           held: Dict[str, Dict[str, Any]],
                           hold_tools: bool = False) -> List[str]:
        """
        Give each lane a claim activated its T# map, and register it.

        A numbered lane gets the map its held record saved (see
        _plan_lane_map), else its home tool T<lane number> (lane12 -> T12),
        so T# equals lane# whatever order units claim in. A home tool the
        plan takes is moved off a lane that holds it: a claimed Bambu lane
        goes back to its own home tool when it can (see _rehome_holder), and
        any other lane is moved as _take_home_tool says. A lane with no
        number is left with an empty map, which TcmdAssign fills with the
        lowest free T#. Lanes are mapped one at a time, so a lane sees the
        T# its bay-mates already took.

        With ``hold_tools`` (a claim during a print) a planned T# another
        live lane or a macro holds (see _tool_in_use) is not taken: the print
        may be using it. The lane keeps the rest of its plan, or no T#, until
        the print ends, when _take_deferred_tools gives it the whole plan;
        one console line says which T#s wait.

        :param afc: the AFC object
        :param lanes: the lanes the claim activated, in slot order
        :param held: lane name -> record handed to this claim
        :param hold_tools: take no T# that is in use
        :return list: "laneN->T#" (a lane's T#s joined by +) for each lane
            restored onto a saved map other than its home tool
        """
        def _warn(msg: str) -> None:
            """
            Log a warning about one lane's map.

            :param msg: the warning
            """
            self.logger.warning(f"AFC_BridgeBox {self.name}: {msg}")

        def _note(msg: str) -> None:
            """
            Log a line about one lane's map that needs no action.

            :param msg: the line
            """
            self.logger.debug(f"AFC_BridgeBox {self.name}: {msg}")

        bambu = {ln for pu in getattr(self, "_pool_units", None) or []
                 for ln in pu.get("lanes") or []}
        restored: List[Tuple[Any, List[str]]] = []
        waits: List[Tuple[str, List[Tuple[str, str]], List[str]]] = []
        for lane in lanes:
            ds = "".join(c for c in lane.name if c.isdigit())
            if ds:
                home = "T" + ds
                rec = held.get(lane.name) or {}
                maps, current, take_home = _plan_lane_map(
                    afc, lane.name, home, rec, _warn, _note, bambu)
                busy = []
                for m in (maps if hold_tools else []):
                    who = (None if m == "NONE"
                           else self._tool_in_use(afc, lane.name, m))
                    if who is not None:
                        busy.append((m, who))
                deferred = getattr(self, "_deferred_takes", None)
                if deferred is None:
                    deferred = self._deferred_takes = {}
                deferred.pop(lane.name, None)
                if busy:
                    deferred[lane.name] = {
                        "home": home, "maps": list(maps), "current": current,
                        "quiet": all(self._bambu_home_move(afc, lane.name,
                                                           m, who)
                                     for m, who in busy)}
                    taken = {m for m, _who in busy}
                    maps = [m for m in maps if m not in taken] or ["NONE"]
                    current = (current if current in maps
                               else next((m for m in maps if m != "NONE"), ""))
                    take_home = take_home and home in maps
                    waits.append((lane.name, busy, maps))
                # The takeover moves every T# on the lane's map, and only
                # the home tool is ever taken.
                lane.map, lane._map, lane.current_map = [home], [], home
                if take_home:
                    self._rehome_holder(afc, lane.name, home)
                    self._take_home_tool(afc, lane)
                lane.map, lane.current_map = maps, current
                # NONE in place of a saved T# has had its warning.
                if (not busy
                    and maps != [home]
                    and not (maps == ["NONE"]
                             and _parse_map(rec.get("map")))):
                    restored.append((lane, maps))
            try:
                assign_pool_tcmd(lane, afc)
            except Exception as ex:
                self.logger.warning(
                    f"AFC_BridgeBox {self.name}: TcmdAssign {lane.name} "
                    f"failed: {ex}")
            (getattr(self, "_claim_plans", None) or {}).pop(lane.name, None)
        if waits:
            self._say_deferred_tools(waits)
        # A bay-mate mapped later can move a lane off what it restored; that
        # move has its own warning.
        return [f"{lane.name}->{'+'.join(maps)}" for lane, maps in restored
                if list(lane.map) == maps]

    def _say_deferred_tools(
            self, waits: List[Tuple[str, List[Tuple[str, str]], List[str]]]
    ) -> None:
        """
        Say which T#s a claim during a print leaves where they are, and what
        each lane has until then (see _map_claimed_lanes).

        The home T# of a Bambu lane taken from another Bambu lane that ends
        on its own is where every claim puts both lanes, so that wait goes
        to AFC.log only, as the same move does at a claim with no print
        running (see _bambu_home_move); a line with any other wait is said
        on the console.

        :param waits: (lane, [(T#, what holds it)], its map until then)
        """
        count = sum(len(busy) for _ln, busy, _maps in waits)
        takes = " and ".join(
            f"{ln} takes " + " and ".join(f"{m} from {who}" for m, who in busy)
            for ln, busy, _maps in waits)
        bare = [ln for ln, _busy, maps in waits if maps == ["NONE"]]
        until = []
        if bare:
            until.append(f"{' and '.join(bare)} "
                         f"{'has' if len(bare) == 1 else 'have'} no T#")
        until += [f"{ln} is {', '.join(maps)}" for ln, _busy, maps in waits
                  if maps != ["NONE"]]
        when = self._print_wait()
        why = ("the print may be using" if when == "the print ends"
               else "a running command may be using")
        pending = getattr(self, "_deferred_takes", None) or {}
        quiet = all((pending.get(ln) or {}).get("quiet")
                    for ln, _busy, _maps in waits)
        (self.logger.debug if quiet else self.logger.info)(
            f"AFC_BridgeBox {self.name}: {takes} once {when}, as {why} "
            f"{'it' if count == 1 else 'them'}; until then "
            f"{', and '.join(until)}.")

    def _bambu_home_move(self, afc: Any, lname: str, cmd: str,
                         who: Optional[str]) -> bool:
        """
        Whether taking ``cmd`` from ``who`` is a Bambu lane's home T# going
        back to it from another Bambu lane that ends on its own, as a claim
        with no print running leaves both: ``who`` keeps its own home T#
        besides ``cmd`` (_take_home_tool), or has no other T# and its home
        T# is free (_rehome_holder).

        :param afc: the AFC object
        :param lname: the Bambu lane taking ``cmd``
        :param cmd: the T# it takes
        :param who: what holds it, a lane's name or "a macro"
        :return bool: True when both lanes end on their own home T#s
        """
        if not who or cmd != _home_tool(lname):
            return False
        if not any(who in (pu.get("lanes") or [])
                   for pu in getattr(self, "_pool_units", None) or []):
            return False
        other = (getattr(afc, "lanes", None) or {}).get(who)
        own = _home_tool(who)
        if other is None or not own:
            return False
        rest = [m for m in (getattr(other, "map", None) or [])
                if m not in (cmd, "NONE")]
        if rest:
            return own in rest
        return _tcmd_free(afc, who, own)

    def _rehome_holder(self, afc: Any, lname: str, cmd: str) -> None:
        """
        Put a claimed Bambu lane that holds ``cmd``, another Bambu lane's
        home tool, back on its own home tool before that lane takes ``cmd``.

        Such a lane was mapped to ``cmd`` (by its saved map, or SET_MAP)
        while the lane ``cmd`` belongs to was unclaimed. Only a lane with no
        other T# and a free home tool (see _tcmd_free) is moved: one with
        other T#s keeps them (see _take_home_tool), and one whose home tool
        is held is left to _take_home_tool's spare T#. Both lanes end on
        their own home tools, so the move is logged to AFC.log only.

        :param afc: the AFC object
        :param lname: the Bambu lane taking ``cmd``, its home tool
        :param cmd: that home tool
        """
        holder = _tcmd_holder(afc, cmd)
        if holder is None or holder == lname:
            return
        # A pool bay's lanes are in afc.lanes only while it is claimed.
        if not any(holder in (pu.get("lanes") or [])
                   for pu in getattr(self, "_pool_units", None) or []):
            return
        other = afc.lanes[holder]
        ds = "".join(c for c in holder if c.isdigit())
        if (not ds
            or [m for m in (getattr(other, "map", None) or [])
                if m not in (cmd, "NONE")]):
            return
        own = "T" + ds
        if not _tcmd_free(afc, holder, own):
            return
        afc.tool_cmds.pop(cmd, None)
        other.map, other.current_map = [own], own
        try:
            assign_pool_tcmd(other, afc)
        except Exception as ex:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: TcmdAssign {holder} failed: {ex}")
        push = getattr(other, "send_lane_data", None)
        if callable(push):
            try: push()
            except Exception: pass
        self.logger.debug(
            f"AFC_BridgeBox {self.name}: {cmd} is the tool of Bambu lane "
            f"{lname}. {holder} was mapped to it and is back on {own}.")

    def _take_home_tool(self, afc: Any, lane: Any) -> None:
        """
        Move a claimed lane's home tool off another lane that holds it.

        A Bambu lane whose claim takes its home T<lane number> (no saved
        map, or a saved map that lists it; see _plan_lane_map) takes it
        whatever order units claim in. PREP numbers lanes before any claim
        registers, so it can hand a lane added elsewhere a T# inside the
        Bambu range (or config maps one there). The claim still takes that
        T# and drops it from the other lane's map, so AFC's lanes and tool
        table agree. A lane left with no T# gets the lowest free one past
        the Bambu range: the var file saves an empty map as NONE and PREP
        restores that ahead of the config `map:`, so an emptied lane would
        stay without a tool across restarts. The var file keeps the new T#,
        so the next boot has no conflict. The move is a console warning,
        except for a Bambu lane left on its own home tool, which is where
        its claim puts it: that goes to AFC.log.

        :param afc: the AFC object
        :param lane: a Bambu lane this claim activated, already mapped home
        """
        live = getattr(afc, "lanes", None) or {}
        for home in [m for m in (getattr(lane, "map", None) or [])
                     if m != "NONE"]:
            owner = afc.tool_cmds.get(home)
            other = live.get(owner) if owner != lane.name else None
            if other is None:
                continue
            lo, hi = self._bambu_span()
            afc.tool_cmds.pop(home, None)
            other.map = [m for m in (getattr(other, "map", None) or [])
                         if m != home]
            spare = None
            if not [m for m in other.map if m != "NONE"]:
                spare = self._spare_tool(afc, hi)
                if spare is not None:
                    handlers = getattr(getattr(afc, "gcode", None),
                                       "ready_gcode_handlers", None) or {}
                    if handlers.get(spare) is None:
                        afc.function.register_tool_macro(other.name, spare)
                    afc.tool_cmds[spare] = other.name
                    other.map = [spare]
            if getattr(other, "current_map", "") == home or spare:
                other.current_map = next(
                    (m for m in other.map if m != "NONE"), "")
            for push in (getattr(other, "send_lane_data", None),
                         getattr(afc, "save_vars", None)):
                if callable(push):
                    try: push()
                    except Exception: pass
            span = f"T{lo}-T{hi}"
            now = ", ".join(m for m in other.map if m != "NONE")
            msg = (f"AFC_BridgeBox {self.name}: {home} is the tool of Bambu "
                   f"lane {lane.name} (Bambu lanes lane{lo}-lane{hi} are "
                   f"{span}). {owner} was mapped to it and is now "
                   f"{now or 'without a T#'}.")
            if spare:
                msg += (f" To give {owner} another tool outside {span}, use "
                        f"SET_MAP LANE={owner} MAP=<T#>.")
            # AFC's reset gives a lane only the first T# of its map:.
            in_config = (getattr(other, "_map", None) or [])[:1] == [home]
            if in_config:
                section = (getattr(other, "fullname", None)
                           or f"AFC_lane {owner}")
                msg += (f" Also set map: in [{section}] outside {span}, or "
                        f"AFC_RESET_MAPPING puts {owner} back on {home} and "
                        f"{lane.name} on another T#.")
            # A Bambu lane that keeps its own home tool ends where every
            # claim puts it, as this lane does: nothing for the user to do.
            bambu = any(owner in (pu.get("lanes") or [])
                        for pu in getattr(self, "_pool_units", None) or [])
            quiet = (bambu
                     and not spare
                     and not in_config
                     and _home_tool(owner) in other.map)
            (self.logger.debug if quiet else self.logger.warning)(msg)

    @staticmethod
    def _spare_tool(afc: Any, above: int) -> Optional[str]:
        """
        The lowest T# past ``above`` that no lane uses and no macro holds.

        Skips what TcmdAssign skips: tools in AFC's lookup table, tools any
        lane maps or its config maps, and names a macro other than AFC's
        CHANGE_TOOL already registered. A CHANGE_TOOL left on a T# no lane
        uses is free to take.

        :param afc: the AFC object
        :param above: the last Bambu lane number
        :return str: the T# name, or None when none is free
        """
        handlers = getattr(getattr(afc, "gcode", None),
                           "ready_gcode_handlers", None) or {}
        change_tool = getattr(afc, "cmd_CHANGE_TOOL", None)
        ours = getattr(change_tool, "__func__", change_tool)
        taken = set(afc.tool_cmds)
        for ln in (getattr(afc, "lanes", None) or {}).values():
            taken.update(getattr(ln, "map", None) or [])
            taken.update(getattr(ln, "_map", None) or [])
        for n in range(above + 1, above + 100):
            cmd = f"T{n}"
            held = handlers.get(cmd)
            if (cmd in taken
                or (held is not None
                    and getattr(held, "__func__", held) is not ours)):
                continue
            return cmd
        return None

    def _clear_standalone(self, lanes: list) -> None:
        """
        Take the extruder out of standalone mode once a live claim gives it lanes.

        An extruder with no lanes at ready registers itself as its own lane and
        sets no_lanes (AFC_extruder.handle_ready). Pool spares are unassigned at
        ready, so on a scout-only setup the extruder comes up standalone. A later
        claim runs check_lanes() through activate_from_pool, which pops that
        self-lane, but no_lanes stays set. is_standalone() then keeps answering
        True and the toolhead sensor callback fires the standalone auto-load
        (load_unload_sequence) the moment a claimed lane reaches the sensor. That
        loader moves the extruder stepper onto its private trapq while the unit's
        own tool_stn advance is still driving it through the toolhead queue: two
        step sources on one stepper, which shuts Klipper down with
        "stepcompress ... Invalid sequence".

        Once the self-lane is gone the extruder has real lanes, so mirror what
        ready would have concluded had they been there: standalone off.

        :param lanes: the lanes the claim just activated
        """
        seen = set()
        for lane in lanes:
            ext = getattr(lane, "extruder_obj", None)
            if ext is None or id(ext) in seen:
                continue
            seen.add(id(ext))
            if not getattr(ext, "no_lanes", False):
                continue
            tc = getattr(ext, "tc_lane", None)
            if tc is not None and tc.name in getattr(ext, "lanes", {}):
                continue                      # self-lane still registered
            ext.no_lanes = False
            self.logger.info(
                f"AFC_BridgeBox {self.name}: {ext.name} left standalone mode "
                f"(claimed lanes attached)")

    #: Unanswered 0x3702 queries before an online unit is called ams1: an AMS 2
    #: answers within ~1.5 s, an AMS 1 never does. ~6 s at the 500 ms cadence;
    #: must not be zero.
    _AMS1_ASK_FLOOR = 12

    def _refine_models(self, snap: Dict[str, Any], online_idx: set,
                       entries: List[str], sec: str) -> List[str]:
        """
        Confirm `boxed` roster entries' generation from the bus dialect.

        `boxed` means "not confirmed yet": the 0x3702 version query separates
        the generations on the wire (an AMS 2 answers it, an AMS 1 never
        does), and the firmware counts asks and answers per unit. One
        answered ask rewrites the entry ams2; _AMS1_ASK_FLOOR unanswered asks
        from an online unit rewrite it ams1. The verdict is recorded and
        applied to the running unit (see _apply_model_live), and moves
        nothing: lane count is identical across the boxed family, so the lane
        map and unit name hold.

        Every index-keyed input (uids, htmask, a2mask, a2asks) comes from
        the one snapshot the tick took, so a verdict is always about the UID
        that held that index in the reply the counters came from.

        :param snap: the tick's _chain_snapshot
        :param online_idx: chain indices the tick's status reads online
        :param entries: the recorded roster, parsed to entry strings
        :param sec: the state section holding the roster
        :return list: the entries, refined where the bus was decisive
        """
        a2mask, a2asks = snap["a2mask"], snap["a2asks"]
        htmask = snap["htmask"]
        idx_by_uid = {_norm_uid(u): i
                      for i, u in enumerate(snap["uids"]) if (u or "").strip()}
        refined: List[str] = []
        changed = []
        told: List[Any] = []
        for e in entries:
            model, _sep, uid = e.partition(":")
            uid = _norm_uid(uid)
            i = idx_by_uid.get(uid)
            if (model.strip().lower() != "boxed"
                or i is None
                or htmask >> i & 1):
                # Only boxed units on the chain get a dialect verdict, never an
                # HT.
                refined.append(e)
                continue
            if a2mask >> i & 1:
                new = "ams2"
            elif (i in online_idx
                  and i < len(a2asks)
                  and a2asks[i] >= self._AMS1_ASK_FLOOR):
                new = "ams1"
            else:
                refined.append(e)
                continue
            refined.append(f"{new}:{uid}")
            changed.append(f"{uid} -> {new}")
            # Refinements apply live (existing objects change): ams2 gains the
            # heater, ams1 moves the firmware to the AMS 1 vocabulary (see
            # _apply_model_live).
            told += self._apply_model_live(uid, new, i)
        # Told in RAM only; the table is saved once per tick, never mid-print.
        saved = False
        if told and not self._is_printing():
            try:
                told[-1]._send_bindings(told[-1]._bridge)   # binds + idsave
                saved = True
            except Exception:
                pass
        if changed:
            self._state_set({sec: {"roster": ", ".join(refined)}})
            self.logger.info(
                f"AFC_BridgeBox {self.name}: generation confirmed by bus "
                f"dialect (0x3702): {', '.join(changed)} -- applied to the "
                f"running unit and recorded. Lanes and names do not move."
                + (" The bridge has it in RAM and saves it at the next "
                   "connect (no flash write mid-print)."
                   if told and not saved else ""))
        return refined

    def hold_id_save(self, unit: Any) -> bool:
        """
        Whether a claimed unit must leave the binding table in RAM for now.

        A claim sends the whole table and asks the bridge to save it. The
        binds apply from RAM at once; the save only makes them survive a
        bridge power-up. The bridge compares before writing, but a UID it has
        not stored before turns that save into a flash erase that stalls the
        bus for tens of milliseconds, and the host cannot see which UIDs are
        stored. So while a print is active the save is held, and the watch
        tick makes it once the print is over (see _save_held_ids). A restart
        before then loses nothing: the next connect saves the table.

        :param unit: the AFC_BambuAMS unit about to send its bindings
        :return bool: True when the save is held and the caller must not save
        """
        if not self._is_printing():
            return False
        first = getattr(self, "_ids_held", None) is None
        self._ids_held = unit
        if first:
            try:
                self.logger.info(
                    f"AFC_BridgeBox {self.name}: {getattr(unit, 'name', '?')} "
                    f"bound in the bridge's RAM; the table is saved when the "
                    f"print ends (no flash write mid-print).")
            except Exception:
                pass
        return True

    def _save_held_ids(self, bridge: Any) -> None:
        """
        Make the binding-table save held back by hold_id_save.

        Runs every watch tick and does nothing until a save is held and the
        print is over, by the same test that held it. The table is re-sent
        ahead of the save, so a bridge that rebooted in between stores it
        too.

        :param bridge: the chain's bridge
        """
        unit = getattr(self, "_ids_held", None)
        if unit is None or self._is_printing():
            return
        self._ids_held = None
        try:
            unit._send_bindings(bridge)       # binds + idsave
        except Exception:
            pass

    #: Options a model override may re-apply to a live unit: pure policy pushed
    #: to the firmware. Wiring keys cannot be re-read after fabrication.
    _LATE_MODEL_KEYS = frozenset({"measure_on_insert"})

    def _reapply_model_override(self, obj: Any, model: str,
                                index: Optional[int] = None,
                                persist: bool = True) -> None:
        """
        Re-apply a ``[AFC_BridgeBox <model>]`` override once the model is known.

        A model override cannot reach a spare at config load: a pool bay is
        fabricated before anything is plugged into it, so its ams_model is the
        placeholder ``boxed`` and _fold_and_sweep looks up ``model:boxed``.
        Without this, e.g. ``[AFC_BridgeBox ams1] measure_on_insert: True``
        would match nothing and the bay would keep the spare default (False,
        see the emission site).

        The attribute is always set; the firmware is told only when
        _may_tell allows it. Otherwise _adopt_index sends it, from the
        updated attribute, once the index is pinned.

        :param obj: the live AFC_BambuAMS unit object
        :param model: the now-known canonical model tag
        :param index: the chain index the caller's evidence was read at, if
            any (see _may_tell)
        :param persist: passed to _send_ht_flag; False when the caller saves
            the binding table itself
        """
        ov = (getattr(self, "_overrides", None) or {}).get(
            "model:" + _norm_model(model))
        if not ov:
            return
        # Precedence as in _fold: chain defaults < model < unit.
        name = f"{self._UNIT_SECTION} {getattr(obj, 'name', '')}"
        per_unit = (getattr(self, "_overrides", None) or {}).get(name) or {}
        for key in self._LATE_MODEL_KEYS:
            if key in per_unit or key not in ov:
                continue
            val = str(ov[key]).strip().lower() in ("1", "true", "yes", "on")
            if bool(getattr(obj, key, False)) == val:
                continue
            setattr(obj, key, val)
            self.logger.info(
                f"AFC_BridgeBox {self.name}: {getattr(obj, 'name', '?')} is a "
                f"{model}; applied {key}={val} from [AFC_BridgeBox {model}] "
                f"(the bay was fabricated as a spare, before its model was "
                f"known)")
            # Push it to the firmware too (_send_ht_flag emits capen), but only
            # once the index is this unit's (see _may_tell).
            if not self._may_tell(obj, index):
                continue
            try:
                obj._send_ht_flag(obj._bridge, persist=persist)
            except Exception:
                pass

    @staticmethod
    def _may_tell(obj: Any, index: Optional[int] = None) -> bool:
        """
        Whether index-keyed firmware commands for a unit may be sent now.

        Until _adopt_index pins it, a unit's ams_index is a default that can
        be another unit's. A pinned one can also be stale: a relink
        re-resolves without clearing _id_resolved, and a mid-session move is
        re-pinned only by the 30 s UID watch. So a unit is told only when
        its index is pinned and, when the evidence came from a chain index,
        still that index. Nothing withheld is lost: _adopt_index sends the
        HT flag, capen, bindings and model from the updated object whenever
        it pins an index.

        :param obj: the live AFC_BambuAMS unit object
        :param index: the chain index the evidence was read at; None when
            the caller has no index of its own
        :return bool: True when the unit has a bridge, a pinned index, and
            (given index) holds that index
        """
        return (getattr(obj, "_bridge", None) is not None
                and bool(getattr(obj, "_id_resolved", False))
                and (index is None
                     or getattr(obj, "ams_index", None) == index))

    def _unit_override(self, obj: Any, model: str, key: str) -> Optional[str]:
        """
        The operator's value for one unit-section option, if any sets it.

        The same sources _fold_and_sweep lays over a fabricated unit, most
        specific first: the unit's own [AFC_BridgeBox <name>] section, then
        [AFC_BridgeBox <model>], then a chain-wide default on the master.

        :param obj: the live AFC_BambuAMS unit object
        :param model: the model the unit is being built as
        :param key: the unit-section option
        :return str: the configured value as written, or None
        """
        ov = getattr(self, "_overrides", None) or {}
        for src in (ov.get(f"{self._UNIT_SECTION} {getattr(obj, 'name', '')}"),
                    ov.get("model:" + _norm_model(model)),
                    getattr(self, "_chain_defaults", None)):
            if src and key in src:
                return src[key]
        return None

    def _apply_model_dryer(self, obj: Any, model: str) -> None:
        """
        Give a running unit the heater flag and drying ceiling a restart would.

        A restart builds the unit's section for `model` and folds the
        operator's sections over it: heater and dry_max_temp come from the
        model (a heated model's ceiling is capped by this chain's
        dry_max_temp when set),
        and a model or per-unit section overrides either. The unit reads the
        ceiling with getint's 1..DRY_TEMP_HARD_MAX bounds, so it is clamped
        to them here. Resolved the same way, a live model change leaves the
        unit with what the next restart gives it, e.g. 50 for
        ``[AFC_BridgeBox ams2] dry_max_temp: 50`` on a unit that ran as
        `boxed` (65) until the bus confirmed it. Learned values folded from
        the store are not consulted: nothing learns these two keys
        (persist_learned records only bowden lengths).

        :param obj: the live AFC_BambuAMS unit object
        :param model: the unit's model, now known
        """
        spec = _AMS_MODELS.get(model) or _AMS_MODELS["ams1"]
        heater = spec[0]
        raw = self._unit_override(obj, model, "heater")
        if raw is not None:
            heater = str(raw).strip().lower() in ("1", "true", "yes", "on")
        obj.has_heater = heater
        # The ceiling is resolved even with the heater off, as the fold does.
        ceiling = (_dry_ceiling(getattr(self, "dry_max_temp", 0), model)
                   if model in _HEATED_MODELS else spec[3])
        raw = self._unit_override(obj, model, "dry_max_temp")
        if raw is not None:
            try:
                ceiling = int(str(raw).strip())
            except ValueError:
                pass                          # a restart would refuse it
        obj.dry_max_temp = max(1, min(int(ceiling), DRY_TEMP_HARD_MAX))

    def _apply_learned(self, obj: Any, uid: str, model: str) -> None:
        """
        Give a claimed unit the learned values of the uid that claimed it.

        A bay is worn by one unit after another, so the bowden lengths it
        holds are the previous occupant's, or its fabricated default. Each
        of _LEARNED_KEYS resolves the way a restart builds it for this uid:
        the operator's value when a unit, model or chain section sets one
        (see _unit_override), else the uid's record (see _learned_section),
        else none, which the unit takes as its default. The unit also waits
        for its own next completed load before adopting a measured path, so
        a figure left from the previous occupant is never saved as this
        uid's (see afcBambuAMS.apply_learned).

        :param obj: the live AFC_BambuAMS unit object
        :param uid: the uid it was claimed for
        :param model: the model it was claimed as
        """
        apply = getattr(obj, "apply_learned", None)
        if apply is None:
            return
        record = self._learned_for(uid)
        values: Dict[str, float] = {}
        used = []
        for key in _LEARNED_KEYS:
            for raw, mine in ((self._unit_override(obj, model, key), False),
                              (record.get(key), True)):
                val = _learned_length(raw)
                if val is not None:
                    values[key] = val
                    if mine:
                        mm = f"{val:.1f}".rstrip("0").rstrip(".")
                        used.append(f"{key} {mm}mm")
                    break
        try:
            apply(values)
        except Exception as e:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: learned values for "
                f"{getattr(obj, 'name', '?')} (UID {uid}) not applied: {e}")
            return
        if used:
            self.logger.info(
                f"AFC_BridgeBox {self.name}: {getattr(obj, 'name', '?')} "
                f"takes the {', '.join(used)} that UID {uid} learned.")

    def _apply_model_measure(self, obj: Any, model: str, spare: bool) -> None:
        """
        Resolve measure_on_insert for the model a unit is being claimed as.

        The bay still holds the value resolved for the model it was built as
        or last claimed as, which may have come from that model's
        ``[AFC_BridgeBox <model>]`` section. This resolves it the way
        _fold_and_sweep does for the bay's section: the unit, model or chain
        section that sets it, else the fabricated default (on for a known
        HT, off for a boxed unit and for every spare; see the emission site).
        Nothing learns this key, so the store is not consulted.

        :param obj: the live AFC_BambuAMS unit object
        :param model: the model the unit is being claimed as
        :param spare: whether the bay was fabricated as a spare
        """
        raw = self._unit_override(obj, model, "measure_on_insert")
        if raw is not None:
            obj.measure_on_insert = (
                str(raw).strip().lower() in ("1", "true", "yes", "on"))
        else:
            obj.measure_on_insert = (not spare
                                     and _norm_model(model) in _HT_MODELS_BB)

    def _apply_model_live(self, uid: str, model: str,
                          index: int) -> List[Any]:
        """
        Flip a running unit's model in place after a dialect verdict.

        Sets ams_model, and the heater flag and drying ceiling a restart
        would give the unit (see _apply_model_dryer), so an ams2 gets the
        dryer `boxed` withheld. Then tells the firmware with the helpers
        index adoption uses (`model` for the unit's index and its UID's
        `bind`), because `boxed` runs on the AMS 2 profile, and an ams1 left
        untold would be judged by AMS 2's vocabulary until a restart. Only a
        unit that still holds the index the verdict was read at is told
        (see _may_tell); any other is sent nothing, and its next adoption
        sends the new model. Nothing is saved to flash here: the caller
        does that once.

        :param uid: the unit's 24-hex uid
        :param model: the confirmed model tag (ams1/ams2)
        :param index: the chain index the verdict was read at
        :return list: the units whose firmware was told
        """
        told: List[Any] = []
        try:
            for u in self.units:
                if u.get("uid") == uid:
                    u["model"] = model
            for _name, obj in self.printer.lookup_objects("AFC_BambuAMS"):
                if str(getattr(obj, "unit_uid", "")).upper() == uid:
                    obj.ams_model = model
                    self._apply_model_dryer(obj, model)
                    if self._may_tell(obj, index):
                        obj._send_unit_model(obj._bridge)
                        obj._send_bindings(obj._bridge, only_uid=uid,
                                           persist=False)
                        told.append(obj)
                    # The model is only now known, so this is the first moment
                    # a model-keyed override can be applied to this bay.
                    self._reapply_model_override(obj, model, index=index,
                                                 persist=False)
        except Exception:
            pass                              # the record still applies later
        return told

    def _prune_missing(self, bridge: Any, uids: List[str],
                       entries: List[str], eventtime: float,
                       sec: str, online_idx: Optional[set] = None) -> None:
        """
        Drop rostered units that have been provably gone for removal_grace.

        Presence, not enrollment, is the evidence: the firmware's chain map
        keeps a unit enrolled after it is unplugged, so membership in the
        chain reply cannot prove absence. What can is the per-unit online
        flag in the status stream: enrolled at index i AND units[i].online.
        Absence only counts while it is distinguishable from an outage:
        the serial link must be up and at least one unit must be online,
        otherwise "everything is missing" reads as "the chain is off", every
        clock resets, and nothing is removed. (Corollary: the sole unit of a
        single-unit chain is never auto-removed: its absence and a chain
        power-off look identical. Edit the recorded roster by hand for that.)
        Nothing is removed with a pool, or with a roster: option, which is
        the roster a restart builds from.

        The removal itself only rewrites the record. This session's
        fabricated sections, lanes, and bridge polling stand until the next
        restart, the same apply-at-restart contract as additions. The
        unit's learned values are kept, and so are its lane_map and name
        entries, so plugging it back in before the next restart keeps its
        lanes, its Spoolman bindings, and its calibration. At the restart
        its name and lanes stay reserved for its return, unless a recorded
        AMS would otherwise have no bay or push the HT lanes up: then that
        AMS takes them (see _roster_sections).

        :param bridge: the chain's bridge
        :param uids: chain_uids(): index -> UID, empties kept
        :param entries: the recorded roster, parsed to entry strings
        :param eventtime: reactor time
        :param sec: the state section holding the roster
        :param online_idx: the tick's online chain indices; read from the
            bridge when not given
        """
        # Never prune the roster with a pool (a re-plug keeps its named slot;
        # FORGET releases it) or with a roster: option (a restart builds from
        # that).
        if self.pool_ams or self.pool_ht or self._roster_source == "option":
            self._watch_state = "watching"
            return
        if not self.removal_grace:
            self._watch_state = "removal-disabled"
            return
        if getattr(bridge, "_serial", None) is None:
            self._watch_state = "link-down"
            self._missing_since.clear()
            return
        if online_idx is None:
            latest = bridge.latest_status() or {}
            online_idx = {u.get("n") for u in (latest.get("units") or [])
                          if u.get("online")}
        if not online_idx:
            self._watch_state = "chain-dark"
            self._missing_since.clear()
            return
        self._watch_state = "watching"

        present = {_norm_uid(u)
                   for i, u in enumerate(uids) if i in online_idx}
        survivors: List[str] = []
        dropped: List[str] = []
        for e in entries:
            uid = _norm_uid(e.partition(":")[2])
            if uid in present:
                self._missing_since.pop(uid, None)
                survivors.append(e)
                continue
            if uid not in self._missing_since:
                # Announce that the countdown started, so a running countdown
                # is distinguishable from a stalled watch.
                self._missing_since[uid] = eventtime
                name = getattr(self, "_name_map", {}).get(uid, uid)
                self.logger.info(
                    f"AFC_BridgeBox {self.name}: {name} ({uid}) is offline "
                    f"on a live chain -- if it stays gone "
                    f"{self.removal_grace:.0f}s it will be recorded as "
                    f"removed (takes effect at the next RESTART).")
            since = self._missing_since[uid]
            if eventtime - since < self.removal_grace:
                survivors.append(e)
                continue
            self._missing_since.pop(uid, None)
            dropped.append(e)
        if dropped:
            self._state_set({sec: {"roster": ", ".join(survivors)}})
            gone = {_norm_uid(e.partition(":")[2]) for e in dropped}
            waiting = sorted(
                u for u in (set(self._no_bay) | set(self._no_bay_told)
                            | set(getattr(self, "_unbayed", None) or ()))
                if u not in gone)
            self.logger.info(
                f"AFC_BridgeBox {self.name}: unit(s) gone from the chain for "
                f"over {self.removal_grace:.0f}s: {', '.join(dropped)} -- "
                f"removed from the recorded roster; applies at the next "
                f"RESTART. Their learned values are kept. Plugged back in "
                f"before then, one keeps its lanes and name; after the "
                f"restart they stay reserved for its return unless a new AMS "
                f"would otherwise have no bay or push the HT lanes up"
                + (f" ({', '.join(waiting)} is waiting for a bay and takes "
                   f"one)" if len(waiting) == 1
                   else f" ({', '.join(waiting)} are waiting for bays and "
                        f"take them)" if waiting else "")
                + f". If one is never coming back, AFC_BRIDGEBOX_FORGET "
                  f"CHAIN={self.name} UID=<uid> frees its lanes and name for "
                  f"reuse.")

    def _pending_restart(self) -> List[str]:
        """
        What a restart would change in the enrollment.

        Empty when a restart would change nothing, as with a roster: option, which a
        restart builds from. Reads the tick's cached copy of the record, so a status
        poll costs no file I/O.

        :return list: human-readable diffs between the recorded roster and the
                      running enrollment
        """
        raw = getattr(self, "_recorded_raw", None)
        if raw is None or self._roster_source == "option":
            return []
        try:
            recorded = self._parse_roster(raw) if raw.strip() else []
        except Exception:
            return []
        rec = {u["uid"]: u["model"] for u in recorded}
        run = {u["uid"]: u["model"] for u in self.units}
        out: List[str] = []
        for uid, model in rec.items():
            if uid not in run:
                out.append(f"add {model}:{uid}")
            elif run[uid] != model:
                out.append(f"{run[uid]} -> {model}: {uid}")
        for uid, model in run.items():
            if uid not in rec:
                out.append(f"remove {model}:{uid}")
        return sorted(out)

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        Report the chain for the status API.

        :param eventtime: reactor time (unused)
        :return dict: the roster as fabricated
        """
        enrolled = {u["uid"] for u in self.units}
        # Without a clock the ages are unknown; a stand-in time would make
        # them up (0.0 gave negative ages).
        now: Optional[float]
        try:
            now = self.printer.get_reactor().monotonic()
        except Exception:
            now = None
        # Chain dialect, per chain index: is the unit enumerated
        # and online, is it HT-flagged or has it answered 0x3702 (ams2), and how
        # many 0x3702 asks has it logged toward the ams1 floor.
        dialect: Dict[str, Any] = {}
        try:
            bridge = (self._chain_bridge()
                      or getattr(self, "_bridge", None))
            if bridge is not None and getattr(bridge, "_serial", None) is not None:
                # One snapshot, so each row's counters are the ones its
                # uid had (see _chain_snapshot).
                snap = self._chain_snapshot(bridge)
                a2mask, a2asks = snap["a2mask"], snap["a2asks"]
                htmask = snap["htmask"]
                latest = bridge.latest_status() or {}
                online = {u.get("n") for u in (latest.get("units") or [])
                          if u.get("online")}
                uids = snap["uids"]
                dialect = {
                    "ams1_ask_floor": self._AMS1_ASK_FLOOR,
                    "chain": [
                        {"i": i, "uid": (u or "").upper(),
                         "online": i in online,
                         "ht": bool(htmask >> i & 1),
                         "answered_3702": bool(a2mask >> i & 1),
                         "asks_3702": a2asks[i] if i < len(a2asks) else 0}
                        for i, u in enumerate(uids) if (u or "").strip()],
                }
        except Exception as e:
            dialect = {"error": str(e)}
        # Fresh dict copies: Moonraker diffs against its cached result, and
        # shared dicts would hide _apply_model_live's in-place changes.
        return {"units": [dict(u) for u in self.units],
                "lane_base": self.lane_base,
                "dialect": dialect,
                "roster_source": self._roster_source,
                # Departed units still recorded with lanes/names, the ones
                # FORGET is for.
                "tombstones": sorted(
                    set(getattr(self, "_lane_map", {})) - enrolled
                    - {pu["bound"] for pu in getattr(self, "_pool_units", [])
                       if pu.get("bound")}
                    - ({_norm_uid(e.partition(":")[2]) for e in
                        (getattr(self, "_recorded_raw", None) or "").split(",")
                        if e.partition(":")[2].strip()}
                       if self._roster_source != "option" else set())),
                # Running countdowns in seconds: removal debounce without a
                # pool, or the hold on an offline unit's bay with one (see
                # _track_absence). Dropped past their end so the status stays
                # still. Without a clock each age is None and every entry is
                # kept, since which holds have ended cannot be told.
                "missing": {uid: None if now is None else int(now - t)
                            for uid, t in self._missing_since.items()
                            if now is None
                               or not (self.pool_ams or self.pool_ht)
                               or now - t < self.release_grace},
                # Units on the chain that found no free bay of their family.
                "waiting_for_bay": sorted(self._no_bay),
                # The recorded chain vs the running one: roster additions
                # and removals land in the record and apply at the next
                # restart, so this lists what the next restart will change.
                "pending_restart": self._pending_restart(),
                "watch_ticks": getattr(self, "_tick_count", 0),
                # Bays whose saved lane records are held, and for which unit.
                "held_bays": {bay: e.get("uid") for bay, e in
                              (getattr(self, "_held", None) or {}).items()},
                # Which branch the last tick took.
                "watch_state": getattr(self, "_watch_state", "not-started")}


class BridgeBoxOverrideHolder:
    """
    A [AFC_BridgeBox <name>] section with no serial_port is an override
    carrier, not a second chain master:

        [AFC_BridgeBox ht]              # every HT on the chain
        measure_on_insert: False

        [AFC_BridgeBox ams2]            # every AMS 2 Pro
        dry_max_temp: 60

        [AFC_BridgeBox Bambu_AMS_1]     # this one unit
        measure_on_insert: True

        [AFC_BridgeBox AFC_hub Bambu_AMS_1]   # any fabricated section,
        afc_bowden_length: 1800              # named exactly

    The keys overlay the fabricated values at config time (identity keys
    protected), win over learned values, and never persist into state;
    deleting the section removes the override. This class only makes the
    section legal to klippy and consumes its options; the master reads the
    values from the merged fileconfig it already holds.
    """

    def __init__(self, config: Any) -> None:
        """
        Hold an override section, reading every option so Klipper accepts it.

        :param config: the override section
        """
        self.name = config.get_name().split(None, 1)[-1]
        try:
            for opt in config.get_prefix_options(""):
                config.get(opt)
        except Exception:
            pass


def load_config_prefix(config: Any) -> Any:
    # serial_port is the discriminator: every chain master requires one,
    # and no override should ever carry one (it is an identity key the
    # fold refuses anyway).
    """
    Build a chain master, or an override holder when the section has no serial_port.

    :param config: the [AFC_BridgeBox <name>] section
    :return Any: afcBridgeBox or BridgeBoxOverrideHolder
    """
    if config.get("serial_port", None) is None:
        return BridgeBoxOverrideHolder(config)
    return afcBridgeBox(config)
