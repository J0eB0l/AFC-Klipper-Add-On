# AFC Unit Heaters, one web panel for every filament dryer on the printer
#
# Bambu AMS 2 Pro / AMS HT and Anycubic ACE units all have drying heaters, and
# all of them are otherwise driven by typed G-code with different names, units
# and argument spellings (AFC_BAMBU_HEATER_START ... TIME=minutes vs
# ACE_DRY ... DURATION=minutes). This serves a single control panel for all of
# them, discovered from the printer config at startup: whatever dryers exist in
# AFC.cfg appear here, drawn to match their hardware.
#
# Configuration:
#
#   [afc_dryer]
#   port: 8093              # HTTP port to serve on
#   bind: 0.0.0.0           # 127.0.0.1 to restrict to the host
#   poll: 2.0               # seconds between status snapshots
#   show_heaterless: False  # also list every other AFC unit (BoxTurtle,
#                           # HTLF, OpenAMS and so on) read-only, so the
#                           # panel can show the whole machine at once
#
# Then add it to Mainsail/Fluidd as a webcam with service type "iframe" and
# URL http://<printer-host>:8093/, an iframe camera is the only way to get an
# interactive panel into either UI, neither having a plugin API. It is also
# just a web page, so that URL works on its own in any browser.
#
# Adding a vendor
# ---------------
# Subclass _Backend, fill in the six hooks, and add it to _BACKENDS. Nothing
# else in the module knows what an AMS or an ACE is; the page renders from
# the descriptor each backend returns.
#
# Threading
# ---------
# The HTTP server runs on its own thread and Klipper's reactor is emphatically
# not thread-safe, so this module never touches printer objects from a request:
#
#   * status is snapshotted by a reactor timer into a plain dict under a lock,
#     and requests only ever read that dict;
#   * commands are handed back to the reactor with register_async_callback and
#     run there, never inline in the handler.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations

import hashlib
import json
import logging
import math
import string
import threading
import chelper
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from typing import Any, Dict, List, Optional


# Offered in the UI. The per-unit ceiling disables the rest, so an ACE (55 C by
# default) and an AMS HT (85 C) share one list without either being wrong.
TEMP_CHOICES = (40, 45, 50, 55, 60, 65, 70, 75, 80, 85)
# Minutes, 1..12 h in whole hours. Both vendors' commands take minutes; the
# labels are hours because that is how dry cycles are quoted.
TIME_CHOICES = tuple((h * 60, "%d h" % h) for h in range(1, 13))


def _css_color(value: Any) -> str:
    """
    Normalise a vendor's spool colour into something CSS can use.

    Bambu reports "RRGGBB", ACE reports [r, g, b]. Returning "" for anything
    unusable keeps "no colour known" distinguishable all the way to the page,
    which decides what to draw: an empty bay stays empty, and a loaded spool
    with no colour draws black, the same fallback AFC itself uses.

    All-zero is treated as unknown rather than as black for the same reason:
    it is what both vendors send when they have no colour at all, and it
    reaches the page as "" so the loaded/empty distinction still decides.

    :param value: vendor colour field.
    :return str: a CSS colour, or "" when there isn't a usable one.
    """
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        try:
            r, g, b = (max(0, min(255, int(c))) for c in value[:3])
        except (TypeError, ValueError):
            return ""
        return "" if (r, g, b) == (0, 0, 0) else "rgb(%d,%d,%d)" % (r, g, b)
    if isinstance(value, str) and value:
        v = value.strip().lstrip("#")[:6]
        # Check each digit: int(v, 16) would also accept a sign, underscores
        # or trailing spaces, none of which is valid CSS.
        if len(v) == 6 and all(c in string.hexdigits for c in v):
            return "" if v.upper() == "000000" else "#" + v
    return ""


def _lane_bays(unit: Any, nslots: int) -> List[Dict[str, str]]:
    """
    Read spool colour and material per bay from the unit's AFC lanes.

    This is the colour the rest of AFC already agrees on (set from an RFID
    read, from Spoolman via Moonraker, or from the lane's own config), so it
    matches what Mainsail shows for the same spool. The vendor's own slot data
    is a poorer source: an ACE reports [0,0,0] for a spool it has no colour
    for, which would paint every bay the same fallback shade.

    Both vendors map lane -> bay through a ``_slot_map`` built from the lane's
    1-based config index, so one helper covers them; a unit without one falls
    back to that index directly.

    :param unit: the AFC unit object.
    :param nslots: how many bays the unit exposes.
    :return list: per-bay {"color", "material"}, blank where unknown.
    """
    bays: List[Dict[str, str]] = [{"color": "", "material": ""}
                                  for _ in range(nslots)]
    # Everything below colour/material is for the hover tooltip, the same data
    # Mainsail shows for the spool, so no Spoolman client is needed here.
    lanes = getattr(unit, "lanes", None) or {}
    slot_map = getattr(unit, "_slot_map", None) or {}
    for name, lane in lanes.items():
        slot = slot_map.get(name)
        if slot is None:
            try:
                slot = int(getattr(lane, "index", 0)) - 1
            except (TypeError, ValueError):
                continue
        if not 0 <= slot < nslots:
            continue
        weight = getattr(lane, "weight", 0) or 0
        try:
            weight = float(weight)
        except (TypeError, ValueError):
            weight = 0.0
        bays[slot] = {
            "color": _css_color(getattr(lane, "color", "") or ""),
            "material": str(getattr(lane, "material", "") or ""),
            "lane": str(name),
            # filament_name is Spoolman's own name for the spool when one is
            # linked; sub_type is the tag/Spoolman variant ("Matte").
            "filament": str(getattr(lane, "filament_name", "") or ""),
            "sub_type": str(getattr(lane, "sub_type", "") or ""),
            "vendor": str(getattr(lane, "spool_vendor", "") or ""),
            "spool_id": getattr(lane, "spool_id", None),
            # Grams remaining. 0 means "not tracked" rather than "empty", so
            # the tooltip omits it instead of claiming an empty spool.
            "weight": round(weight, 1) if weight > 0 else None,
            "temp": getattr(lane, "extruder_temp", None),
        }
    return bays


def _merge_bays(vendor: List[Dict[str, Any]],
                lane: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """
    Overlay AFC lane colour/material onto the vendor's presence data.

    Presence stays with the vendor, which can see whether a spool is
    physically in the bay, while colour and material prefer the
    lane, falling back to the vendor's own value when AFC has none.

    :param vendor: per-bay dicts from the backend (authoritative for presence).
    :param lane: per-bay dicts from _lane_bays.
    :return list: merged per-bay dicts.
    """
    out = []
    for i, v in enumerate(vendor):
        ln = lane[i] if i < len(lane) else {}
        merged = {
            "present": bool(v.get("present")),
            "color": ln.get("color") or v.get("color") or "",
            "material": ln.get("material") or v.get("material") or "",
        }
        # Tooltip detail comes from the lane only; a bay with no lane mapped
        # carries none and the page shows no tooltip.
        for k in ("lane", "filament", "sub_type", "vendor", "spool_id",
                  "weight", "temp"):
            if ln.get(k) not in (None, ""):
                merged[k] = ln[k]
        out.append(merged)
    return out


def _secs(value: Any) -> Optional[int]:
    """
    A whole number of seconds, or None for anything that is not one.

    :param value: a duration in seconds, or None
    :return Optional[int]: the duration, or None
    """
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _minutes_to_s(value: Any) -> Optional[int]:
    """
    A cycle length in minutes as seconds, or None when there is none.

    The vendors disagree about units: a Bambu unit reports the length it was
    asked for in minutes and the time left in seconds; an ACE reports minutes
    and seconds too. Seconds is what the page does arithmetic in, so the
    conversion happens here rather than in each backend.

    Zero is None, not a zero-length cycle: both vendors report 0 for "no cycle
    on record", and a 0 would draw a progress bar that is instantly full.

    :param value: a duration in minutes, or None
    :return Optional[int]: the same duration in seconds, or None
    """
    s = _secs(value)
    return s * 60 if s else None


def _reading(value: Any) -> Optional[float]:
    """
    A sensor value, or None when there is nothing to show.

    Every one of these sensors reports 0.0 for a channel it has not read yet:
    a Bambu HT's aht10 sits at temperature 0.0 while its humidity reads 41.0,
    and an unread OpenAMS driver reports 0.0 for both. The page renders a
    dash for None, so passing 0 through would put a false "0 °C" on a card.
    A genuine 0 °C or 0 % from an ambient sensor would mean something is badly
    wrong anyway. Applied here, where every backend's view converges.

    NaN and infinity are dropped too: json.dumps writes them as bare NaN and
    Infinity, which the page's JSON parser rejects, losing the whole state.

    :param value: a raw sensor reading, or None
    :return Optional[float]: the reading, or None if absent/unread/non-finite
    """
    try:
        reading = float(value)
    except (TypeError, ValueError):
        return None
    return reading if reading and math.isfinite(reading) else None


class _Backend:
    """One vendor's dryers. Subclasses map a unit object onto the panel's view."""

    #: Klipper object prefixes to enumerate, e.g. ("AFC_BambuAMS",). A vendor
    #: can ship more than one unit class (ACE and ACE 2 Pro).
    object_prefixes = ()
    #: Short tag used by the page to pick artwork.
    kind = ""

    def find(self, printer: Any) -> List[Any]:
        """
        Every unit this backend claims.

        Defaults to enumerating ``object_prefixes`` through Klipper. Override
        when a backend's units are not identifiable by object name; the
        generic backend sources them from AFC's own registry instead.

        :param printer: The Klipper printer object
        :return list: unit objects, possibly with duplicates
        """
        found: List[Any] = []
        for prefix in self.object_prefixes:
            try:
                found.extend(u for _, u in printer.lookup_objects(prefix))
            except Exception:
                continue
        return found

    def has_heater(self, unit: Any) -> bool:
        """
        Whether this unit can dry at all.

        :param unit: an AFC unit object claimed by this backend
        :return bool: True when the unit has a drying heater
        """
        raise NotImplementedError

    def describe(self, unit: Any) -> Dict[str, Any]:
        """
        Static descriptor: model, max_temp, slots. Read once at startup.

        :param unit: an AFC unit object claimed by this backend
        :return Dict[str, Any]: model/label/max_temp/slots for the page
        """
        raise NotImplementedError

    def snapshot(self, unit: Any, st: Dict[str, Any]) -> Dict[str, Any]:
        """
        Live view: online, drying, readings, target, remaining, note.

        :param unit: an AFC unit object claimed by this backend
        :param st: that unit's get_status() dict
        :return Dict[str, Any]: the card's live fields
        """
        raise NotImplementedError

    def slots(self, unit: Any, st: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Per-bay spool presence and colour, for the artwork.

        :param unit: an AFC unit object claimed by this backend
        :param st: that unit's get_status() dict
        :return List[Dict[str, Any]]: per-bay present/color/material
        """
        raise NotImplementedError

    def start_script(self, name: str, temp: int, minutes: int,
                     rotate: int) -> str:
        """
        G-code that starts a cycle on this vendor.

        :param name: unit name as configured
        :param temp: target temperature in C
        :param minutes: cycle length in minutes
        :param rotate: 1 to spin spools while drying, where supported
        :return str: a complete g-code command line
        """
        raise NotImplementedError

    def stop_script(self, name: str) -> str:
        """
        G-code that stops a cycle on this vendor.

        :param name: unit name as configured
        :return str: a complete g-code command line
        """
        raise NotImplementedError

    #: Whether the vendor supports spinning spools while drying.
    supports_rotate = False


class _BambuBackend(_Backend):
    """Bambu AMS 2 Pro (65 C, 4 bays) and AMS HT (85 C, 1 bay)."""

    object_prefixes = ("AFC_BambuAMS",)
    kind = "bambu"
    supports_rotate = True

    def has_heater(self, unit: Any) -> bool:
        """
        Whether this unit can dry at all.

        :param unit: an AFC_BambuAMS unit object
        :return bool: True when the unit reports a drying heater
        """
        return bool(getattr(unit, "has_heater", False))

    def describe(self, unit: Any) -> Dict[str, Any]:
        """
        Static descriptor for the card: model, label, ceiling, bay count.

        :param unit: an AFC_BambuAMS unit object
        :return Dict[str, Any]: model/label/max_temp/slots for the page
        """
        model = str(getattr(unit, "ams_model", "ams2") or "ams2").lower()
        return {
            "model": model,
            "label": {"ht": "AMS HT", "ams2": "AMS 2 Pro"}.get(model, "AMS"),
            "max_temp": int(getattr(unit, "dry_max_temp", 65)),
            "slots": int(getattr(unit, "unit_slots", 4)),
        }

    def snapshot(self, unit: Any, st: Dict[str, Any]) -> Dict[str, Any]:
        """
        Live view of the dryer from the unit's own get_status fields.

        :param unit: an AFC_BambuAMS unit object
        :param st: that unit's get_status() dict
        :return Dict[str, Any]: online/drying/readings/target/remaining/note
        """
        return {
            "online": bool(st.get("bridge_online")),
            "drying": bool(st.get("drying")),
            "temperature": st.get("temperature"),
            "humidity": st.get("humidity"),
            "target": st.get("dry_target"),
            "remaining": st.get("dry_remaining"),
            "duration": _minutes_to_s(st.get("dry_minutes")),
            # A condition reported about a running cycle, e.g. the HT's
            # "ams-ht shell open!". Rendered alongside "Drying".
            "note": str(st.get("dry_note") or ""),
            # Kept separate from note: this is the unit refusing to run a cycle.
            "error": str(st.get("dry_error") or ""),
        }

    def slots(self, unit: Any, st: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Per-bay presence and colour from the unit's slot records.

        :param unit: an AFC_BambuAMS unit object
        :param st: that unit's get_status() dict
        :return List[Dict[str, Any]]: per-bay present/color/material
        """
        out = []
        for s in (st.get("slots") or []):
            s = s or {}
            out.append({"present": bool(s.get("present")),
                        "color": _css_color(s.get("color")),
                        "material": s.get("material") or ""})
        return out

    def start_script(self, name: str, temp: int, minutes: int,
                     rotate: int) -> str:
        """
        G-code that starts a Bambu dry cycle.

        :param name: unit name as configured
        :param temp: target temperature in C
        :param minutes: cycle length in minutes
        :param rotate: 1 to spin spools while drying, 0 not to
        :return str: the AFC_BAMBU_HEATER_START command line
        """
        return (f"AFC_BAMBU_HEATER_START UNIT={name} TEMP={temp} "
                f"TIME={minutes} ROTATE={rotate}")

    def stop_script(self, name: str) -> str:
        """
        G-code that stops a Bambu dry cycle.

        :param name: unit name as configured
        :return str: the AFC_BAMBU_HEATER_STOP command line
        """
        return f"AFC_BAMBU_HEATER_STOP UNIT={name}"


class _AceBackend(_Backend):
    """Anycubic ACE / ACE 2 Pro. Commands take minutes, as on the Bambu side;
    what comes back does not agree between the two generations (see
    _cycle_length)."""

    object_prefixes = ("AFC_ACE", "AFC_ACE2")
    kind = "ace"
    # Rotation is AFC's own (ACE_DRY ROTATE=1), not the firmware's: it turns
    # the spools of unthreaded lanes, gated as on a Bambu unit.
    supports_rotate = True

    #: ace_dryer values that mean "not running". Anything else is treated as a
    #: live cycle and shown verbatim.
    _IDLE = ("", "stop", "stopped", "idle", "off", "none")

    def has_heater(self, unit: Any) -> bool:
        """
        Whether this ACE dries; max_dryer_temperature: 0 opts a unit out.

        :param unit: an AFC ACE unit object
        :return bool: True when the ceiling is above zero
        """
        return float(getattr(unit, "max_dryer_temperature", 55.0)) > 0

    @staticmethod
    def _is_v2(unit: Any) -> bool:
        """
        Whether this is an ACE 2 Pro rather than a V1 ACE.

        afcACE2 subclasses afcACE, so an isinstance check would call every
        Pro 2 an ACE. Walk the MRO names instead: the Pro 2 is a different
        machine to its owner and reports differently besides.

        :param unit: an AFC ACE unit object
        :return bool: True for an ACE 2 Pro
        """
        return any("ACE2" in c.__name__.upper() for c in type(unit).__mro__)

    def _cycle_length(self, unit: Any, raw: Any) -> Optional[int]:
        """
        dryer_status' `duration` as seconds.

        The two generations disagree, measured on both:

          V1     duration 480    remain_time 27679   -> minutes
          ACE 2  duration 7200   remain_time 7200    -> seconds

        The V1 echoes back the minutes it was asked for; the Pro 2 converts.
        Decided by generation rather than by comparing against remain_time,
        since a V1 near the end of a cycle would fool any magnitude test.

        :param unit: the unit, to tell the generations apart
        :param raw: dryer_status' duration field
        :return Optional[int]: seconds, or None when no cycle is on record
        """
        return (_secs(raw) or None) if self._is_v2(unit) else _minutes_to_s(raw)

    def describe(self, unit: Any) -> Dict[str, Any]:
        """
        Static descriptor for the card: generation label, ceiling, bays.

        :param unit: an AFC ACE unit object
        :return Dict[str, Any]: model/label/max_temp/slots for the page
        """
        return {
            "model": "ace",
            "label": "ACE 2" if self._is_v2(unit) else "ACE",
            "max_temp": int(float(getattr(unit, "max_dryer_temperature", 55.0))),
            "slots": int(getattr(unit, "SLOTS_PER_UNIT", 4)),
        }

    def snapshot(self, unit: Any, st: Dict[str, Any]) -> Dict[str, Any]:
        """
        Live view of the dryer from the ACE's get_status fields.

        :param unit: an AFC ACE unit object
        :param st: that unit's get_status() dict
        :return Dict[str, Any]: online/drying/readings/target/remaining/note
        """
        dryer = str(st.get("ace_dryer") or "").strip()
        running = dryer.lower() not in self._IDLE
        temp = _reading(st.get("ace_temp"))
        hum = _reading(st.get("ace_humidity"))
        # The V1 ACE has no humidity sensor, so fall back to an external sensor
        # like the generic backend does, only for the channel that is missing.
        if temp is None or hum is None:
            env = _env_reading(unit)
            if temp is None:
                temp = env["temperature"]
            if hum is None:
                hum = env["humidity"]
        return {
            "online": bool(st.get("ace_connected")),
            "drying": running,
            "temperature": temp,
            "humidity": hum,
            # Set-point and clock from dryer_status, only while a cycle runs: an
            # idle ACE reports zeros, which mean "no cycle". remain_time is
            # seconds on both generations; duration is not (see _cycle_length).
            "target": (st.get("ace_dryer_target") or None) if running else None,
            "remaining": _secs(st.get("ace_dryer_remain")) if running else None,
            "duration": (self._cycle_length(unit, st.get("ace_dryer_duration"))
                         if running else None),
            "note": dryer if running else "",
        }

    def slots(self, unit: Any, st: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Per-bay presence and colour from the ACE's slot records.

        :param unit: an AFC ACE unit object
        :param st: that unit's get_status() dict
        :return List[Dict[str, Any]]: per-bay present/color/material
        """
        out = []
        for s in (st.get("ace_slots") or []):
            s = s or {}
            status = str(s.get("status") or "").lower()
            out.append({
                "present": status not in ("", "empty"),
                "color": _css_color(s.get("color")),
                "material": s.get("material") or "",
            })
        return out

    def start_script(self, name: str, temp: int, minutes: int,
                     rotate: int) -> str:
        """
        G-code that starts an ACE dry cycle.

        :param name: unit name as configured
        :param temp: target temperature in C
        :param minutes: cycle length in minutes
        :param rotate: 1 to turn the free spools while drying, 0 not to
        :return str: the ACE_DRY command line
        """
        script = f"ACE_DRY UNIT={name} TEMP={temp} DURATION={minutes}"
        return script + " ROTATE=1" if rotate else script

    def stop_script(self, name: str) -> str:
        """
        G-code that stops an ACE dry cycle.

        :param name: unit name as configured
        :return str: the ACE_DRY_STOP command line
        """
        return f"ACE_DRY_STOP UNIT={name}"


def _env_reading(unit: Any) -> Dict[str, Optional[float]]:
    """
    Temperature and humidity for a unit that keeps them on a separate sensor.

    An OpenAMS's HDC1080/AHT is an ordinary Klipper sensor section, and the
    humidity only reaches the printer objects through the driver's own object
    (`aht10 oams1` carries both, `temperature_sensor oams1` temperature only).

    Candidates are tried from most to least specific: the name the unit
    declares in `environment_sensor`, then its controller, then its own name.
    Objects are matched by name (case-insensitive), not sensor type, and a
    humidity-carrying match wins over a temperature-only one.

    :param unit: an AFC unit object
    :return Dict[str, Optional[float]]: {"temperature": .., "humidity": ..}
    """
    empty = {"temperature": None, "humidity": None}
    printer = getattr(unit, "printer", None)
    if printer is None:
        return empty
    names = [n for n in (getattr(unit, "environment_sensor", None),
                         getattr(unit, "oams_name", None),
                         getattr(unit, "name", None)) if n]
    try:
        objects = list(printer.lookup_objects())
    except Exception:
        return empty
    for want in names:
        fallback = None
        for oname, obj in objects:
            part = str(oname).split(" ", 1)
            if len(part) != 2 or part[1].lower() != str(want).lower():
                continue
            try:
                st = obj.get_status(None)
            except Exception:
                continue
            if not st:
                continue
            # A driver that is present but has not read yet reports 0.0 for
            # both; treat that as "nothing to show" rather than a real 0C.
            temp = _reading(st.get("temperature"))
            hum = _reading(st.get("humidity"))
            if hum is not None:
                return {"temperature": temp, "humidity": hum}
            if temp is not None and fallback is None:
                fallback = {"temperature": temp, "humidity": None}
        if fallback is not None:
            return fallback
    return empty


class _GenericBackend(_Backend):
    """Any other AFC unit: BoxTurtle, HTLF, NightOwl, QuattroBox, OpenAMS,
    Vivid and anything added later.

    None of them dry, so they only appear when show_heaterless is on, and
    they are read-only.

    Sourced from AFC's own registry (afc.units) rather than object prefixes,
    so no list of class names needs to keep in step with new hardware.

    Listed last in _BACKENDS: the vendor backends claim their own units first
    and _discover's `seen` set keeps this one from re-adding them, so a Bambu
    unit is still described as a Bambu unit.
    """

    kind = "generic"
    supports_rotate = False

    #: Registered in afc.units but not a filament source (a toolchanger holds
    #: tools, not spools). Matched on the class MRO, not the settable `type`.
    _NOT_FILAMENT_UNITS = ("TOOLCHANGER",)

    def find(self, printer: Any) -> List[Any]:
        """
        Every AFC unit, from AFC's own registry rather than object names.

        :param printer: The Klipper printer object
        :return List[Any]: unit objects, tool-changers excluded
        """
        afc = printer.lookup_object("AFC", None)
        units = getattr(afc, "units", None) if afc is not None else None
        if not units:
            return []
        return [u for u in units.values() if not self._is_excluded(u)]

    def _is_excluded(self, unit: Any) -> bool:
        """
        Whether this object holds tools rather than filament.

        :param unit: an AFC unit object
        :return bool: True when the unit's class marks it a tool-changer
        """
        names = [c.__name__.upper() for c in type(unit).__mro__]
        return any(any(bad in n for n in names)
                   for bad in self._NOT_FILAMENT_UNITS)

    def has_heater(self, unit: Any) -> bool:
        """
        Never a dryer: this backend lists read-only units.

        :param unit: an AFC unit object
        :return bool: always False
        """
        return False

    def describe(self, unit: Any) -> Dict[str, Any]:
        """
        Static descriptor for a read-only card.

        :param unit: an AFC unit object
        :return Dict[str, Any]: model/label/max_temp/slots for the page
        """
        # `type` is the unit's config-declared model, falling back to the class
        # name. Strip underscores as the rest of AFC does (Box_Turtle -> BoxTurtle).
        declared = str(getattr(unit, "type", "") or "")
        label = declared.replace("_", "") if declared else type(unit).__name__
        return {
            "model": "generic",
            "label": label,
            "max_temp": 0,
            "slots": len(getattr(unit, "lanes", {}) or {}),
        }

    def snapshot(self, unit: Any, st: Dict[str, Any]) -> Dict[str, Any]:
        """
        Live view: environment readings only, no dryer state to report.

        :param unit: an AFC unit object
        :param st: that unit's get_status() dict (unused here)
        :return Dict[str, Any]: online/readings plus the monitor-only note
        """
        env = _env_reading(unit)
        has_env = env["temperature"] is not None or env["humidity"] is not None
        return {
            "online": True,
            "drying": False,
            "temperature": env["temperature"],
            "humidity": env["humidity"],
            "target": None,
            # "no dryer" would read as "nothing to report" next to a live reading.
            "note": "monitor only" if has_env else "no dryer",
        }

    def slots(self, unit: Any, st: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Per-bay presence and colour from the unit's own lanes.

        :param unit: an AFC unit object
        :param st: that unit's get_status() dict (unused here)
        :return List[Dict[str, Any]]: per-bay present/color/material
        """
        out = []
        for lane in (getattr(unit, "lanes", {}) or {}).values():
            out.append({
                "present": bool(getattr(lane, "load_state", False)),
                "color": _css_color(getattr(lane, "color", None)),
                "material": getattr(lane, "material", "") or "",
            })
        return out

    def start_script(self, name: str, temp: int, minutes: int,
                     rotate: int) -> str:
        """
        Unreachable through the panel: has_heater() is False, so no start
        control renders. Raises rather than returning a broken g-code line.

        :param name: unit name as configured
        :param temp: unused
        :param minutes: unused
        :param rotate: unused
        """
        error_str = f"{name} has no dryer"
        raise ValueError(error_str)

    def stop_script(self, name: str) -> str:
        """
        Unreachable, like start_script above, and errors the same way.

        :param name: unit name as configured
        """
        error_str = f"{name} has no dryer"
        raise ValueError(error_str)


_BACKENDS = (_BambuBackend(), _AceBackend(), _GenericBackend())


class AFCDryer:
    """Discovers every dryer on the printer and serves the control panel."""

    def __init__(self, config: Any) -> None:
        """
        Read config and register lifecycle hooks. Unit discovery waits for
        klippy:ready, since the AFC units do not exist yet while this section is
        being parsed.

        :param config: Klipper config for the ``[afc_dryer]`` section.
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.logger = logging.getLogger("afc_dryer")
        self.port = config.getint("port", 8093, minval=1, maxval=65535)
        self.bind = config.get("bind", "0.0.0.0")
        self.poll = config.getfloat("poll", 2.0, minval=0.5)
        # A dryer-less unit has nothing to control, but listing it keeps the
        # panel a picture of the whole machine rather than a partial one.
        self.show_heaterless = config.getboolean("show_heaterless", False)

        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._timer = None
        self._lock = threading.Lock()
        self._units: List[Dict[str, Any]] = []
        self._state: Dict[str, Any] = {"units": [], "ready": False}

        self.printer.lookup_object("gcode").register_command(
            "AFC_DRYER_STATUS", self.cmd_STATUS,
            desc="Show the AFC Unit Heaters panel status")
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)

    # ---- lifecycle ----

    def _handle_ready(self) -> None:
        """
        Discover units, start the snapshot timer and the HTTP server.
        """
        # Switch to AFC's logger (AFC.log and console); the stdlib logger from
        # __init__ covers the window before AFC exists.
        afc_log = getattr(self.printer.lookup_object("AFC", None),
                          "logger", None)
        if afc_log is not None:
            self.logger = afc_log
        self._discover()
        if not self._units:
            self.logger.warning(
                "afc_dryer: no units with a drying heater found; the panel "
                "will be empty (set show_heaterless: True to list every other "
                "AFC unit read-only)")
        self._timer = self.reactor.register_timer(
            self._snapshot, self.reactor.NOW)
        self._start_server()

    def _handle_disconnect(self) -> None:
        """
        Shut the server down so a Klipper restart can rebind the port.
        """
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except Exception:
                pass
            self._server = None

    def _discover(self) -> None:
        """
        Resolve every unit's static descriptor once, from config.
        """
        units: List[Dict[str, Any]] = []
        seen = set()
        for backend in _BACKENDS:
            try:
                found = backend.find(self.printer)
            except Exception as e:
                self.logger.warning(
                    f"afc_dryer: could not enumerate {backend.kind} "
                    f"units: {e}")
                continue
            for unit in found:
                # A subclass matches its parent's prefix too, so the same
                # object can come back from more than one lookup.
                if id(unit) in seen:
                    continue
                seen.add(id(unit))
                try:
                    heated = backend.has_heater(unit)
                    # A pool unit's heater is provisional until an AMS 2 claims
                    # it, so keep pool units regardless; _snapshot hides the
                    # unclaimed ones and refreshes has_heater/model/label live.
                    pooled = bool(getattr(unit, "pool", False))
                    if not heated and not pooled and not self.show_heaterless:
                        continue
                    desc = backend.describe(unit)
                except Exception as e:
                    self.logger.warning(
                        f"afc_dryer: skipping a {backend.kind} unit: {e}")
                    continue
                desc.update({
                    "name": getattr(unit, "name", "?"),
                    "kind": backend.kind,
                    "has_heater": heated,
                    "rotate": backend.supports_rotate,
                    "_obj": unit,
                    "_backend": backend,
                })
                units.append(desc)
        units.sort(key=lambda u: (u["kind"], u["name"]))
        self._units = units
        named = ", ".join(f"{u['name']} ({u['label']})"
                          for u in units) or "none"
        self.logger.info(f"afc_dryer: {len(units)} dryer(s): {named}")

    def _start_server(self) -> None:
        """
        Bind and start the HTTP server on its own daemon thread.
        """
        try:
            self._server = ThreadingHTTPServer(
                (self.bind, self.port), _make_handler(self))
            self._server.daemon_threads = True
            def _serve() -> None:
                """
                Name the thread, then run the HTTP server; serve_forever is a bound method.
                """
                # serve_forever is a stdlib method, so name the thread in this wrapper.
                try:
                    thread_name = threading.current_thread().name
                    chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
                except Exception:
                    pass
                self._server.serve_forever()

            self._thread = threading.Thread(
                target=_serve, daemon=True, name="afc_dryer_http")
            self._thread.start()
            self.logger.info(
                f"afc_dryer: serving on http://{self.bind}:{self.port}/")
            self.logger.info(
                f"  Mainsail/Fluidd: add a webcam, service 'iframe', URL "
                f"http://<printer-host>:{self.port}/")
        except Exception as e:
            self.logger.error(
                f"afc_dryer: failed to start HTTP server on "
                f"{self.bind}:{self.port}: {e}")
            # A thread failure comes after the bind, so release the port now:
            # with _server cleared, _handle_disconnect can no longer close it.
            if self._server is not None:
                self._server.server_close()
            self._server = None

    # ---- status snapshot (reactor thread) ----

    def _snapshot(self, eventtime: float) -> float:
        """
        Reactor timer: copy each unit's status into a plain dict the HTTP
        thread can read safely. Only primitives are published; no printer
        object escapes onto the other thread.

        :param eventtime: reactor time of this firing.
        :return float: next firing time.
        """
        units = []
        for desc in self._units:
            unit, backend = desc["_obj"], desc["_backend"]
            # A Bambu pool unit is inert until it claims a physical AMS (claim
            # clears .pool), so unclaimed or pulled units are not shown.
            if getattr(unit, "pool", False):
                continue
            try:
                st = unit.get_status(eventtime) or {}
                view = backend.snapshot(unit, st)
                # Vendor data decides which bays are occupied; the AFC lanes
                # supply the colour, matching the rest of the UI.
                slots = _merge_bays(backend.slots(unit, st),
                                    _lane_bays(unit, desc["slots"]))
            except Exception as e:
                self.logger.debug(
                    f"afc_dryer: status failed for {desc['name']}: {e}")
                view, slots = {"online": False, "drying": False}, []
            slots = slots[:desc["slots"]]
            # Pad so the artwork always draws the unit's full complement of
            # bays, empty ones included.
            while len(slots) < desc["slots"]:
                slots.append({"present": False, "color": "", "material": ""})
            row = {k: desc[k] for k in
                   ("name", "kind", "model", "label", "max_temp", "slots",
                    "has_heater", "rotate")}
            # Refresh the identity fields live: a Bambu pool slot changes model,
            # ceiling and heater when an AMS claims it. Both calls are attribute reads.
            try:
                fresh = backend.describe(unit)
                row["model"] = fresh.get("model", row["model"])
                row["label"] = fresh.get("label", row["label"])
                row["max_temp"] = fresh.get("max_temp", row["max_temp"])
                row["has_heater"] = bool(backend.has_heater(unit))
            except Exception:
                pass
            row.update({
                "online": bool(view.get("online")),
                "drying": bool(view.get("drying")),
                "temperature": _reading(view.get("temperature")),
                "humidity": _reading(view.get("humidity")),
                "target": view.get("target"),
                # Seconds left and the requested cycle length; None on a unit
                # that reports no clock.
                "remaining": _secs(view.get("remaining")),
                "duration": _secs(view.get("duration")),
                "note": view.get("note") or "",
                "error": view.get("error") or "",
                "bays": slots,
            })
            units.append(row)
        with self._lock:
            self._state = {"units": units, "ready": True}
        return eventtime + self.poll

    def get_state(self) -> Dict[str, Any]:
        """
        Thread-safe read of the latest snapshot, stamped with the page
        version so a stale browser can notice it is out of date.

        :return Dict[str, Any]: copy of the snapshot plus ``page_version``
        """
        with self._lock:
            state = dict(self._state)
        state["page_version"] = PAGE_VERSION
        return state

    # ---- command dispatch (HTTP thread -> reactor) ----

    def request_dry(self, name: str, action: str, temp: int, minutes: int,
                    rotate: int) -> str:
        """
        Validate a panel request and queue the matching G-code on the reactor.

        Every field is validated against the discovered units rather than
        trusted: the unit name is matched to a known unit and the rest are
        coerced to ints, so nothing from an HTTP body is ever interpolated
        into a G-code string as free text (the panel binds 0.0.0.0 by default).

        :param name: unit name from the request.
        :param action: "start" or "stop".
        :param temp: requested temperature in C.
        :param minutes: requested run time in minutes.
        :param rotate: 1 to spin the spools while drying, where supported.
        :return str: the G-code queued.
        :raises ValueError: if the request does not name a real dryer.
        """
        desc = next((u for u in self._units if u["name"] == name), None)
        if desc is None:
            error_str = f"unknown unit {name!r}"
            raise ValueError(error_str)
        backend = desc["_backend"]
        has_heater, max_temp = desc["has_heater"], desc["max_temp"]
        if desc["kind"] == "bambu":
            # A Bambu unit's heater and ceiling can change after _discover, so
            # check the same live attribute reads _snapshot shows.
            try:
                has_heater = bool(backend.has_heater(desc["_obj"]))
                max_temp = int(backend.describe(desc["_obj"])["max_temp"])
            except Exception:
                pass
        if not has_heater:
            error_str = f"{name} has no drying heater"
            raise ValueError(error_str)
        if action == "stop":
            script = backend.stop_script(desc["name"])
        elif action == "start":
            # Clamp here as well as in the unit: the panel should not be able
            # to ask for something the unit will only refuse.
            temp = max(0, min(int(temp), max_temp))
            minutes = max(0, min(int(minutes), 65535))
            rot = 1 if (int(rotate) and desc["rotate"]) else 0
            script = backend.start_script(desc["name"], temp, minutes, rot)
        else:
            error_str = f"unknown action {action!r}"
            raise ValueError(error_str)
        self.reactor.register_async_callback(
            lambda e, s=script: self._run_script(s))
        return script

    def _run_script(self, script: str) -> None:
        """
        Run a queued G-code on the reactor thread.

        :param script: the complete g-code command line
        """
        try:
            self.printer.lookup_object("gcode").run_script(script)
        except Exception as e:
            self.logger.error(f"afc_dryer: {script!r} failed: {e}")

    # ---- gcode / status ----

    def cmd_STATUS(self, gcmd: Any) -> None:
        """
        Report where the panel is served and what it found.

        Usage
        -------
        `AFC_DRYER_STATUS`

        Example
        -------
        ```
        AFC_DRYER_STATUS
        ```
        """
        units = self.get_state().get("units", [])
        running = "running" if self._server else "NOT running"
        states = []
        for u in units:
            st = "drying" if u["drying"] else ("idle" if u["online"] else "offline")
            states.append(f'{u["name"]} [{u["label"]}] {st}')
        gcmd.respond_info(
            f"AFC Unit Heaters: {running} on http://{self.bind}:{self.port}/, "
            f"{len(units)} dryer(s): {', '.join(states) or 'none'}")

    def get_status(self, eventtime: Any = None) -> Dict[str, Any]:
        """
        Expose the panel's own state to the printer status API.

        :param eventtime: reactor event time of the query (unused)
        :return Dict[str, Any]: running flag, port, and the unit list
        """
        return {
            "running": self._server is not None,
            "port": self.port,
            "units": self.get_state().get("units", []),
        }


def _make_handler(panel: AFCDryer) -> type:
    """
    Build the request handler bound to a panel instance.

    :param panel: the AFCDryer to serve.
    :return type: a BaseHTTPRequestHandler subclass.
    """

    class Handler(BaseHTTPRequestHandler):
        server_version = "AFCDryer/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:
            """
            Route request logging to the panel's logger at debug.

            The default logs every request to stderr, which lands in
            klippy.log; at 2s polling that is a lot of noise.

            :param fmt: printf-style format from BaseHTTPRequestHandler
            :param args: its arguments
            """
            # Silence the access log; dry start/stop and discovery are logged separately.
            return

        def _send(self, code: int, body: Any, ctype: str) -> None:
            """
            Write one complete response with the panel's standard headers.

            :param code: HTTP status code
            :param body: bytes or str payload
            :param ctype: Content-Type header value
            """
            if isinstance(body, str):
                body = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            # The page is embedded via an iframe on another origin, so it needs CORS.
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except Exception:
                pass          # client navigated away mid-response

        def _json(self, code: int, obj: Any) -> None:
            """
            Serialise and send a JSON response.

            :param code: HTTP status code
            :param obj: any json.dumps-able value
            """
            self._send(code, json.dumps(obj), "application/json")

        def do_OPTIONS(self) -> None:
            """
            Answer the CORS preflight the iframe's fetches trigger.
            """
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self) -> None:
            """
            Serve the page, the state snapshot, or the option lists.
            """
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            if path == "/":
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(200, panel.get_state())
            elif path == "/api/options":
                self._json(200, {
                    "temps": list(TEMP_CHOICES),
                    "times": [{"minutes": m, "label": lbl}
                              for m, lbl in TIME_CHOICES]})
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self) -> None:
            """
            Accept a dry start/stop and hand it to the reactor.
            """
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            if path != "/api/dry":
                self._json(404, {"error": "not found"})
                return
            try:
                n = int(self.headers.get("Content-Length") or 0)
                # read(-1) would block until the client closes the socket.
                if n < 0:
                    error_str = f"negative Content-Length {n}"
                    raise ValueError(error_str)
                body = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(body, dict):
                    error_str = f"expected a JSON object, got {type(body).__name__}"
                    raise ValueError(error_str)
            except Exception as e:
                self._json(400, {"error": f"bad request body: {e}"})
                return
            try:
                script = panel.request_dry(
                    name=str(body.get("unit", "")),
                    action=str(body.get("action", "")),
                    temp=int(body.get("temp", 55)),
                    minutes=int(body.get("minutes", 480)),
                    rotate=int(body.get("rotate", 0)))
            except (ValueError, TypeError, OverflowError) as e:
                self._json(400, {"error": str(e)})
                return
            self._json(200, {"ok": True, "queued": script})

    return Handler


# One self-contained document: no CDN, no build step, no assets to install.
# The unit artwork is inline SVG for the same reason.
_PAGE_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AFC Unit Heaters</title>
<style>
  :root{
    --bg:#f5f6f8; --card:#fff; --fg:#1b1d21; --dim:#6b7280; --line:#dfe3e8;
    --accent:#2f8f4e; --hot:#e2673a; --off:#9aa0a6; --shadow:rgba(0,0,0,.08);
  }
  @media (prefers-color-scheme: dark){
    :root{
      --bg:#1b1d21; --card:#25282e; --fg:#e8eaed; --dim:#9aa0a6; --line:#343941;
      --accent:#4caf6d; --hot:#ff8a5c; --off:#5f6570; --shadow:rgba(0,0,0,.35);
    }
  }
  *{box-sizing:border-box}
  body{margin:0;padding:14px;background:var(--bg);color:var(--fg);
    font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
  h1{font-size:13px;font-weight:600;margin:0 0 12px;color:var(--dim);
    letter-spacing:.06em;text-transform:uppercase}
  .grid{display:grid;gap:14px;grid-template-columns:repeat(auto-fill,minmax(330px,1fr))}
  .card{background:var(--card);border:1px solid var(--line);border-radius:10px;
    padding:14px;box-shadow:0 1px 3px var(--shadow)}
  .head{display:flex;align-items:center;gap:9px;margin-bottom:10px}
  .name{font-weight:600;font-size:15px}
  .model{font-size:10px;color:var(--dim);text-transform:uppercase;
    letter-spacing:.05em;border:1px solid var(--line);border-radius:4px;padding:1px 6px}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--off);
    margin-left:auto;flex:none}
  .dot.on{background:var(--accent)}
  .dot.hot{background:var(--hot);animation:pulse 1.6s ease-in-out infinite}
  @keyframes pulse{50%{opacity:.3}}
  .body{display:flex;gap:13px;align-items:flex-start}
  .art{flex:none}
  .readouts{flex:1;min-width:0;display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .ro{background:var(--bg);border-radius:7px;padding:7px 9px;min-width:0}
  .ro .k{font-size:10px;color:var(--dim);text-transform:uppercase;letter-spacing:.04em}
  .ro .v{font-size:17px;font-weight:600;white-space:nowrap;overflow:hidden;
    text-overflow:ellipsis}
  .state{grid-column:1/-1;font-size:13px;font-weight:500}
  .state.hot{color:var(--hot)} .state.on{color:var(--accent)}
  .state.off{color:var(--dim)}
  /* Hidden until a cycle reports a clock, so a unit that reports none looks
     the way it always did rather than carrying an empty track. */
  .bar{grid-column:1/-1;height:5px;border-radius:3px;background:var(--bg);
    overflow:hidden;display:none}
  .bar .fill{height:100%;width:0;background:var(--hot);transition:width .4s}
  .ctl{margin-top:12px;padding-top:12px;border-top:1px solid var(--line);
    display:flex;flex-wrap:wrap;gap:8px;align-items:flex-end}
  label{font-size:10px;color:var(--dim);text-transform:uppercase;
    letter-spacing:.04em;display:block;margin-bottom:3px}
  select{background:var(--bg);color:var(--fg);border:1px solid var(--line);
    border-radius:6px;padding:6px 8px;font:inherit;font-size:13px}
  select:disabled{opacity:.5}
  .sw{display:flex;align-items:center;gap:6px;font-size:13px;cursor:pointer;
    user-select:none;padding-bottom:6px;text-transform:none;letter-spacing:0;
    color:var(--fg);margin:0}
  .sw input{width:16px;height:16px;accent-color:var(--accent);cursor:pointer}
  button{border:0;border-radius:6px;padding:8px 15px;font:inherit;
    font-weight:600;font-size:13px;cursor:pointer;color:#fff;
    background:var(--accent)}
  .ctl button{margin-left:auto}
  button.stop{background:var(--hot)}
  button:disabled{opacity:.4;cursor:not-allowed}
  .msg{margin-top:8px;font-size:12px;color:var(--dim);min-height:1.2em;
    word-break:break-word}
  .msg.err{color:var(--hot)}
  .empty{color:var(--dim);padding:24px;text-align:center;background:var(--card);
    border:1px dashed var(--line);border-radius:10px}
  .spool{transition:fill .4s}
</style>
</head>
<body>
<h1>AFC Unit Heaters</h1>
<div id="grid" class="grid"></div>
<script>
"use strict";
var OPTS = {temps:[], times:[]};

// A loaded spool with no known colour draws black, AFC's own fallback. The bay
// ring is drawn in --line so a black spool still reads on a dark theme.
function bay(cx, cy, r, s, hub){
  var fill = s.present ? (s.color || '#000000') : 'transparent';
  // Native SVG tooltip. The <g> scopes the <title> to this bay; as a direct
  // child of <svg> it would title the whole image.
  var tip = spoolTip(s);
  return '<g>' + (tip ? '<title>' + tip + '</title>' : '') +
         '<circle cx="' + cx + '" cy="' + cy + '" r="' + r +
           '" fill="none" stroke="var(--line)" stroke-width="1.4"/>' +
         '<circle class="spool" cx="' + cx + '" cy="' + cy +
           '" r="' + (r-1.5) + '" fill="' + fill +
           '" opacity="' + (s.present ? '1' : '0') + '"/>' +
         '<circle cx="' + cx + '" cy="' + cy + '" r="' + hub +
           '" fill="var(--card)" stroke="var(--line)"/></g>';
}

// Spool tooltip, built from the AFC lane (the same identity Mainsail shows).
// Unknown lines are omitted rather than shown blank.
function esc(v){
  return String(v).replace(/&/g,'&amp;').replace(/</g,'&lt;')
                  .replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}
function spoolTip(s){
  var out = [];
  if (s.lane) out.push(s.lane);
  if (!s.present){
    // An empty bay still names its lane, so hovering tells you which bay it
    // is rather than nothing at all.
    return out.length ? esc(out[0] + ', empty') : '';
  }
  var name = [s.vendor, s.filament || s.material, s.sub_type]
               .filter(Boolean).join(' ');
  if (name) out.push(name);
  else if (s.material) out.push(s.material);
  if (s.weight != null) out.push(s.weight + ' g remaining');
  if (s.temp != null) out.push(s.temp + ' °C');
  if (s.spool_id != null) out.push('Spoolman #' + s.spool_id);
  return esc(out.join('\n'));
}

// Drawn rather than photographed: the artwork themes with the page, scales to
// any card width, and costs nothing to ship next to the module.
function art(u){
  var hot = u.drying;
  var ring = function(w,h){
    return hot ? '<rect x="6" y="6" width="' + w + '" height="' + h +
                 '" rx="7" fill="none" stroke="var(--hot)" stroke-width="2"/>'
               : ''; };
  var bays = u.bays || [];
  if (u.model === 'ht'){                       // tall single-spool tower
    return '<svg width="86" height="112" viewBox="0 0 86 112" role="img" aria-label="AMS HT">' +
      '<rect x="6" y="6" width="74" height="100" rx="7"' +
        ' fill="var(--bg)" stroke="var(--line)"/>' + ring(74,100) +
      bay(43, 52, 27, bays[0] || {}, 9) +
      '<rect x="20" y="88" width="46" height="8" rx="4" fill="var(--line)" opacity=".6"/></svg>';
  }
  // One wide row of bays for everything except the HT, which also handles any
  // bay count (e.g. a 5-lane BoxTurtle listed read-only).
  var b = '';
  for (var i=0;i<bays.length;i++) b += bay(21 + i*24, 44, 10.5, bays[i]||{}, 3.4);
  var w = 12 + bays.length*24;
  var label = u.label || u.model || 'unit';
  return '<svg width="' + (w+12) + '" height="86" viewBox="0 0 ' + (w+12) +
      ' 86" role="img" aria-label="' + label + '">' +
    '<rect x="6" y="6" width="' + w + '" height="74" rx="7"' +
      ' fill="var(--bg)" stroke="var(--line)"/>' + ring(w,74) + b +
    '<rect x="16" y="66" width="' + (w-20) + '" height="6" rx="3"' +
      ' fill="var(--line)" opacity=".6"/></svg>';
}

// What the heater is doing, stated only as far as the data supports. Missing
// readings during a cycle mean "starting", not "off".
function phase(u){
  if (!u.online) return {cls:'off', text:'Offline'};
  if (!u.has_heater) return {cls:'off', text:'No dryer on this unit'};
  // A refusal outranks everything below it, including our own drying flag.
  if (u.error) return {cls:'off', text:'Refused, ' + u.error};
  if (!u.drying) return {cls:'off', text:'Idle'};
  if (u.note) return {cls:'hot', text:'Drying, ' + u.note};
  if (u.target == null)
    return {cls:'hot', text:(u.temperature == null)
      ? 'Starting, waiting for the unit to report' : 'Drying' + left(u)};
  if (u.temperature == null) return {cls:'hot', text:'Heating to ' + u.target + '°C' + left(u)};
  return (u.temperature < u.target - 2)
    ? {cls:'hot', text:'Heating to ' + u.target + '°C' + left(u)}
    : {cls:'on',  text:'Holding at ' + u.target + '°C' + left(u)};
}

// Time left, when the unit keeps a clock. Absent is not zero, so say nothing.
function left(u){
  if (u.remaining == null || u.remaining < 0) return '';
  var m = Math.ceil(u.remaining / 60);
  return ', ' + (m >= 60 ? (Math.floor(m/60) + 'h ' + pad2(m%60) + 'm')
                          : (m + 'm')) + ' left';
}
function pad2(n){ return (n < 10 ? '0' : '') + n; }

function num(v, suffix){ return (v == null) ? '-' : (Math.round(v*10)/10) + suffix; }
function idOf(name){ return 'u' + name.replace(/[^A-Za-z0-9]/g,'_'); }
function esc(s){
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;')
                  .replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}
// Display only: the real underscore name still goes out in data-unit, which is
// what the server matches a request against.
function pretty(name){ return String(name).replace(/_/g, ' '); }

// Built once per unit. update() patches polled values in place so an open
// <select> is not torn out of the DOM.
function card(u){
  var id = idOf(u.name);
  var temps = OPTS.temps.map(function(t){
    return '<option value="' + t + '"' + (t > u.max_temp ? ' disabled' : '') +
           (t === 55 ? ' selected' : '') + '>' + t + '°C</option>'; }).join('');
  var times = OPTS.times.map(function(t){
    return '<option value="' + t.minutes + '"' + (t.minutes === 480 ? ' selected' : '') +
           '>' + t.label + '</option>'; }).join('');
  var rot = u.rotate
    ? '<label class="sw"><input type="checkbox" id="' + id + '-r"> Rotate</label>'
    : '';
  var ctl = u.has_heater ? (
    '<div class="ctl">' +
      '<div><label>Temp</label><select id="' + id + '-t">' + temps + '</select></div>' +
      '<div><label>Time</label><select id="' + id + '-m">' + times + '</select></div>' + rot +
      '<button id="' + id + '-btn" data-unit="' + esc(u.name) +
        '" data-id="' + id + '" data-act="start">Start</button>' +
    '</div><div class="msg" id="' + id + '-msg"></div>') : '';
  return '<div class="card" id="' + id + '">' +
    '<div class="head"><span class="name">' + esc(pretty(u.name)) + '</span>' +
      '<span class="model">' + esc(u.label) + '</span>' +
      '<span class="dot" id="' + id + '-dot"></span></div>' +
    '<div class="body"><div class="art" id="' + id + '-art"></div><div class="readouts">' +
      '<div class="ro"><div class="k">Chamber</div>' +
        '<div class="v" id="' + id + '-temp">-</div></div>' +
      '<div class="ro"><div class="k">Humidity</div>' +
        '<div class="v" id="' + id + '-hum">-</div></div>' +
      '<div class="ro state" id="' + id + '-state"></div>' +
      '<div class="bar" id="' + id + '-bar"><div class="fill" id="' + id + '-fill"></div></div>' +
    '</div></div>' + ctl + '</div>';
}

function setText(el, text){ if (el && el.textContent !== text) el.textContent = text; }
function setCls(el, cls){ if (el && el.className !== cls) el.className = cls; }
function setDis(el, off){ if (el && el.disabled !== off) el.disabled = off; }

// Patch a card in place. Nothing here replaces a control, so an open dropdown
// stays open and a half-made selection survives the poll.
function update(u){
  var id = idOf(u.name), p = phase(u), busy = u.drying;
  setCls(document.getElementById(id+'-dot'),
         'dot ' + (busy ? 'hot' : (u.online ? 'on' : '')));
  // The artwork is a string; only touch the DOM when it actually differs, or
  // the spool-colour transition restarts on every poll.
  var a = document.getElementById(id+'-art'), svg = art(u);
  if (a && a.dataset.svg !== svg){ a.dataset.svg = svg; a.innerHTML = svg; }
  setText(document.getElementById(id+'-temp'), num(u.temperature,'°C'));
  setText(document.getElementById(id+'-hum'), num(u.humidity,'%'));
  var st = document.getElementById(id+'-state');
  setText(st, p.text); setCls(st, 'ro state ' + p.cls);
  // The track needs both numbers: without the length there is no denominator,
  // and a bar drawn from remaining alone would be a fraction of nothing.
  var bar = document.getElementById(id+'-bar');
  if (bar){
    var run = busy && u.remaining != null && u.remaining >= 0 && u.duration > 0;
    bar.style.display = run ? 'block' : 'none';
    if (run){
      var pct = 100 - Math.round(u.remaining * 100 / u.duration);
      document.getElementById(id+'-fill').style.width =
        Math.max(0, Math.min(100, pct)) + '%';
    }
  }
  // Settings are meaningless mid-cycle, but Stop must stay reachable whenever
  // the unit is talking to us at all.
  setDis(document.getElementById(id+'-t'), busy);
  setDis(document.getElementById(id+'-m'), busy);
  setDis(document.getElementById(id+'-r'), busy);
  // One button that becomes Stop mid-cycle. Patched, not rebuilt, so the
  // dropdowns beside it survive the poll.
  var b = document.getElementById(id+'-btn');
  if (b){
    setText(b, busy ? 'Stop' : 'Start');
    setCls(b, busy ? 'stop' : '');
    b.dataset.act = busy ? 'stop' : 'start';
    setDis(b, !u.online);
  }
}

var built = "";        // which units the grid was built for

function render(state){
  var g = document.getElementById('grid');
  if (!state.units || !state.units.length){
    if (built !== "__none__"){
      built = "__none__";
      g.innerHTML = '<div class="empty">No filament dryers found.<br>' +
        'Expected a <code>[AFC_BambuAMS]</code> unit with <code>ams_model: ams2</code> or ' +
        '<code>ht</code>, or an <code>[AFC_ACE]</code> / <code>[AFC_ACE2]</code> unit.</div>';
    }
    return;
  }
  // Rebuild only when the set of units changes (a unit added, removed or
  // renamed). Steady state never touches the grid's structure.
  var key = state.units.map(function(u){ return u.name + ':' + u.max_temp; }).join('|');
  if (key !== built){
    built = key;
    g.innerHTML = state.units.map(card).join('');
  }
  state.units.forEach(update);
}

document.addEventListener('click', function(ev){
  var b = ev.target.closest ? ev.target.closest('button[data-unit]') : null;
  if (!b) return;
  var id = b.dataset.id, act = b.dataset.act;
  var msg = document.getElementById(id + '-msg');
  var payload = {unit:b.dataset.unit, action:act};
  if (act === 'start'){
    payload.temp = parseInt(document.getElementById(id+'-t').value, 10);
    payload.minutes = parseInt(document.getElementById(id+'-m').value, 10);
    var r = document.getElementById(id+'-r');
    payload.rotate = (r && r.checked) ? 1 : 0;
  }
  b.disabled = true;
  msg.className = 'msg';
  msg.textContent = (act === 'start') ? 'Starting…' : 'Stopping…';
  fetch('api/dry', {method:'POST', headers:{'Content-Type':'application/json'},
                    body:JSON.stringify(payload)})
    .then(function(r){ return r.json().then(function(j){ return {ok:r.ok, j:j}; }); })
    .then(function(res){
      // Only errors are shown; on success the state line already says enough.
      if (!res.ok){ msg.className = 'msg err'; msg.textContent = res.j.error || 'failed'; }
      else { msg.textContent = ''; }
    })
    .catch(function(e){ msg.className = 'msg err'; msg.textContent = String(e); })
    .then(function(){ b.disabled = false; tick(); });
});

// Stamped into the page when it is served; the server reports the version it
// is currently serving. A mismatch means this tab is running markup from
// before a deploy, so reload once.
var MY_VERSION = "__PAGE_VERSION__";
var reloading = false;

function tick(){
  fetch('api/state').then(function(r){ return r.json(); }).then(function(state){
    if (!reloading && state.page_version && state.page_version !== MY_VERSION){
      reloading = true;
      location.reload();
      return;
    }
    render(state);
  }).catch(function(){ /* Klipper restarting; the next poll picks it up */ });
}
fetch('api/options').then(function(r){ return r.json(); })
  .then(function(o){ OPTS = o; })
  .catch(function(){})
  .then(function(){ tick(); setInterval(tick, 2000); });
</script>
</body>
</html>
"""

# The page carries a hash of itself and reloads when the server reports a
# different one, so an open browser picks up a deploy.
PAGE_VERSION = hashlib.md5(_PAGE_TEMPLATE.encode("utf-8")).hexdigest()[:8]
PAGE = _PAGE_TEMPLATE.replace("__PAGE_VERSION__", PAGE_VERSION)



def load_config(config: Any) -> AFCDryer:
    """
    Register the ``[afc_dryer]`` object.

    :param config: ConfigWrapper for the section.
    :return AFCDryer: the panel instance.
    """
    return AFCDryer(config)
