# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Bambu AMS <-> Spoolman.

from __future__ import annotations

import chelper
import traceback
from configparser import Error as error
from typing import Any, Callable, Dict, Optional, Set, TYPE_CHECKING

if TYPE_CHECKING:
    from configfile import ConfigWrapper

# That module imports this one back lazily, so there is no import cycle.
from extras.AFC_BambuAMS import BAMBU_BRAND, _split_bambu_material

try: from extras.AFC_utils import ERROR_STR
except:
    error_str = f"Error when trying to import AFC_utils.ERROR_STR\n{traceback.format_exc()}"
    raise error(error_str)

try: from extras.AFC_RFID import (sync_rfid_to_spoolman,
                                  get_auto_spoolman_create,
                                  match_spool_for_tag, merge_duplicate_spool,
                                  density_for_material,
                                  _spool_uids, _norm_uid, _norm_tray_uid,
                                  _spool_tray_uid, _set_spoolid_takes_on_done,
                                  _cached_spoolman_client, report_tag_scan)
except:
    error_str = ERROR_STR.format(import_lib="AFC_RFID", trace=traceback.format_exc())
    raise error(error_str)


def _bambu_spoolman_client(afc: Any) -> Optional[Any]:
    """
    Build the SpoolmanClient the way every AFC reader does.

    :param afc: the AFC printer object
    :return Optional[SpoolmanClient]: the client, or None if Spoolman or moonraker is unavailable
    """
    if (afc is None
        or getattr(afc, "spoolman", None) is None
        or getattr(afc, "moonraker", None) is None):
        return None
    try:
        return _cached_spoolman_client(afc)
    except Exception:
        return None


# Fields only a decoded Bambu profile fills.
_PROFILE_FIELDS = ("material", "sku", "tray_uid", "temp_min", "temp_max")


def _profile_landed(info: Any) -> bool:
    """
    Whether the tag's Bambu profile decoded, whatever this line holds.

    :param info: the bay's slot record
    :return bool: True when any profile field carries a value
    """
    try:
        for key in _PROFILE_FIELDS:
            val = (info or {}).get(key)
            if val not in (None, "", 0):
                return True
    except Exception:
        pass
    return False


class _QuietInfo:
    """
    A logger passthrough with ``info()`` demoted to ``debug()``.
    """

    def __init__(self, inner: Any) -> None:
        """
        Wrap a logger.

        :param inner: the logger to pass through
        """
        self._inner = inner

    def __getattr__(self, name: str) -> Any:
        """
        Pass anything but info() straight through.

        :param name: the logger attribute
        :return object: the inner logger's attribute
        """
        return getattr(self._inner, name)

    def info(self, *a: Any, **k: Any) -> None:
        """
        Log at debug instead of info.

        :param a: the log call's positional arguments
        :param k: the log call's keyword arguments
        """
        # AFC's debug() has no console_only.
        k.pop("console_only", None)
        try:
            self._inner.debug(*a, **k)
        except Exception:
            pass


class BambuSpoolman:
    """
    The Spoolman half of one Bambu AMS unit.

    :param unit: the afcBambuAMS this instance serves
    """

    #: One Spoolman worker thread for every unit, class-level so a second unit reuses it.
    _spool_q: Any = None
    _spool_t: Any = None
    #: False on a measurement-only delegate: nothing reaches Spoolman.
    spoolman_on: bool = True

    def __init__(self, unit: Any, spoolman: bool = True) -> None:
        """
        Per-unit Spoolman state and helpers for one AFC_BambuAMS unit.

        :param unit: the AFC_BambuAMS unit object
        :param spoolman: False for a measurement-only delegate, which keeps the measurement state
        and never talks to Spoolman
        """
        self._u = unit
        self.spoolman_on = bool(spoolman)
        #: Slots whose binding has already been re-checked against the tag.
        self._binding_check: Dict[int, str] = {}
        #: slot -> the tag UID its Spoolman binding came from.
        self._bound_uid: Dict[int, str] = {}
        #: slot -> the AMS's own measured remaining percentage (see _adopt_measured_remain).
        self._measured_remain: Dict[int, Any] = {}
        #: slot -> (pct, nominal, spool_id, sent).
        self._bind_owed: Dict[int, Any] = {}
        #: slot -> (pct, nominal, grams).
        self._convert_owed: Dict[int, Any] = {}
        self._spoolman_inflight: Set[str] = set()
        #: Bays whose Spoolman bind has not answered yet.
        self._bind_pending: Set[int] = set()
        #: UIDs Spoolman has already said it does not know.
        self._spoolman_no_match: Set[str] = set()
        #: Operator summaries held until the bay's record can answer them.
        self._pending_summary: Dict[int, Any] = {}

    def _forget_spoolman_miss(self, slot: int) -> None:
        """
        Drop the "Spoolman has no spool for this UID" memo for a slot.

        :param slot: 0-based AMS slot index
        """
        try:
            miss = getattr(self, "_spoolman_no_match", None)
            if not miss:
                return
            # The whole set, not this slot's UID.
            miss.clear()
        except Exception:
            pass
        # The binding-contradiction memo is kept.

    def _spoolman_slot_info(self, info: dict) -> dict:
        """
        Translate a bridge slot dict into the shape sync_rfid_to_spoolman and find_spool_by_uid
        expect (the same dict every other AFC reader builds).

        :param info: normalized bridge slot info
        :return dict: Spoolman-shaped slot_info
        """
        material, sub_type = _split_bambu_material(info.get("material") or "")
        # A black tag's record has no "color" (bridge_color_to_rgb), only "color_black".
        color = info.get("color") or ("000000" if info.get("color_black")
                                      else None)
        color_hex = ((color if color.startswith("#") else "#" + color)
                     if color else None)
        try:
            w = int(info.get("weight")) if info.get("weight") else 1000
        except (TypeError, ValueError):
            w = 1000
        si = {
            "uid": info.get("rfid_uid") or "",
            "brand": BAMBU_BRAND,
            "material": material or "",
            "sub_type": sub_type or "",
            "color_hex": (color_hex or "").lstrip("#") or None,
            "diameter": 1.75,
            "extruder_temp": info.get("temp_min"),
            "extruder_temp_min": info.get("temp_min"),
            "extruder_temp_max": info.get("temp_max"),
            "weight_g": w,
        }
        if info.get("tray_uid"):
            si["tray_uid"] = info["tray_uid"]
        return si

    def _binding_contradicted(self, spool_id: Any, uid: str) -> bool:
        """
        Does Spoolman say the bound spool is NOT the one whose tag is in the bay?

        :param spool_id: the Spoolman id the lane is currently bound to
        :param uid: the tag UID actually present in the bay
        :return bool: True only if Spoolman positively contradicts the binding
        """
        if (not spool_id
            or not uid):
            return False
        memo = getattr(self, "_binding_check", None)
        if memo is None:
            memo = self._binding_check = {}
        key = (str(spool_id), str(uid))
        if key in memo:
            return memo[key]
        verdict = False
        try:
            client = _bambu_spoolman_client(getattr(self._u, "afc", None))
            spool = client.get_spool(int(spool_id)) if client else None
            if spool:
                known = _spool_uids(spool)
                # No UIDs recorded => nothing to contradict.
                if known:
                    verdict = _norm_uid(uid) not in known
        except Exception:
            # Never unbind or memoize on a lookup failure.
            return False
        memo[key] = verdict
        return verdict

    def _remember_bound_uid(self, slot: Optional[int], uid: str) -> None:
        """
        Record that this slot's Spoolman binding was made from tag ``uid``.

        :param slot: 0-based AMS slot index, or None when unknown
        :param uid: the tag UID the binding was made from
        """
        if (slot is None
            or not uid):
            return
        try:
            if getattr(self, "_bound_uid", None) is None:
                self._bound_uid = {}
            self._bound_uid[slot] = uid
        except Exception:
            pass

    #: Run Spoolman HTTP on a worker thread so blocking calls do not starve clock sync.
    SPOOLMAN_BG = True

    def _spoolman_bg(self, job: Callable[[], None]) -> None:
        """
        Run ``job`` on the shared Spoolman worker thread.

        :param job: zero-arg callable holding the HTTP work
        """
        if not getattr(self, "SPOOLMAN_BG", False):
            job()
            return
        import queue as _queue
        import threading
        q = getattr(BambuSpoolman, "_spool_q", None)
        if q is None:
            q = BambuSpoolman._spool_q = _queue.Queue()

            def _drain() -> None:
                """
                Consume queued Spoolman jobs forever; one bad job is eaten.
                """
                try:
                    thread_name = threading.current_thread().name
                    chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
                except Exception:
                    pass
                while True:
                    fn = q.get()
                    try:
                        fn()
                    except Exception:
                        pass

            t = threading.Thread(target=_drain, daemon=True, name="afc_bambu_spool")
            BambuSpoolman._spool_t = t
            t.start()
        q.put(job)

    def _bind_by_uid_bg(self, lane: Any, slot: Optional[int], uid: str, note: str,
                        tray_uid: str = "", restored: bool = False) -> None:
        """
        Match this tag against Spoolman off-reactor; bind the lane on-reactor.

        :param lane: the AFC lane to bind
        :param slot: 0-based AMS slot index, or None when unknown
        :param uid: the tag UID to look up
        :param note: where the read came from, for the log line
        :param tray_uid: the tag's 16-byte roll identity, when it carries one
        :param restored: the unit's lookup for a lane AFC restored with no spool
        (afcBambuAMS._lookup_unbound), not a read of the bay.
        """
        if not uid:
            return
        tray_uid = _norm_tray_uid(tray_uid)
        inflight = getattr(self, "_spoolman_inflight", None)
        if inflight is None:
            inflight = self._spoolman_inflight = set()
        if uid in inflight:
            return
        inflight.add(uid)
        pend_b = getattr(self, "_bind_pending", None)
        if pend_b is None:
            pend_b = self._bind_pending = set()
        if slot is not None:
            pend_b.add(slot)
        BambuSpoolman._mark_bind_sent(self, slot)
        afc = getattr(self._u, "afc", None)

        def job() -> None:
            """
            Worker-thread half: the Spoolman lookups, no Klipper state.
            """
            spool = by_tray = None
            # An empty answer is not a miss while Spoolman or Moonraker is starting.
            down = False
            try:
                client = _bambu_spoolman_client(afc)
                if client is None:
                    down = restored
                if client is not None:
                    spool, matched_by_tray, dupe_id = match_spool_for_tag(client, uid, tray_uid)
                    by_tray = spool if matched_by_tray else None
                    if (restored
                        and spool is None):
                        reach = getattr(client, "reachable", None)
                        down = callable(reach) and not reach()
                    if (dupe_id is not None
                        and spool is not None):
                        # Spoolman holds this reel twice: fold the other record in.
                        merged = merge_duplicate_spool(client, spool, dupe_id, tray_uid,
                                                       self._u.logger,
                                                       f"{self._u.name}")
                        if merged is not None:
                            spool = merged
                            if by_tray is not None:
                                by_tray = merged
                if (spool is not None
                    and client is not None):
                    # Union the chip UID into card_uids, or stamp the tray UID if none is saved.
                    try:
                        if (by_tray is None
                            and tray_uid):
                            have = _spool_tray_uid(spool)
                            if not have:
                                client.write_tray_uid(spool.get("id"), tray_uid)
                            elif have != tray_uid:
                                self._u.logger.debug(f"{self._u.name}: spool "
                                                     f"{spool.get('id')} carries tray UID "
                                                     f"{have} but this tag says {tray_uid} -- "
                                                     f"left alone; one of them is on the wrong "
                                                     f"record")
                    except Exception:
                        pass
                    # Warm AFC's spool cache so set_spoolID below need not block on get_spool.
                    try:
                        mr = getattr(afc, "moonraker", None)
                        if (mr is not None
                            and hasattr(mr, "get_spool")):
                            mr.get_spool(spool.get("id"), lambda _sp: None)
                    except Exception:
                        pass
                    # Off-reactor, so the next read matches directly.
                    try:
                        client.write_spool_metadata(spool.get("id"), uid=uid)
                    except Exception:
                        pass
                if not restored:
                    # A read of the bay, matched or not: tell Spoolman's paired browsers.
                    report_tag_scan(client, getattr(lane, "name", ""), uid, "bambu")
            except Exception:
                spool = None
                down = restored

            def apply(eventtime: Optional[float] = None) -> None:
                """
                Reactor half: bind the lane and drop the in-flight mark.

                :param eventtime: reactor time of this firing (unused)
                """
                landing = False
                try:
                    if not down:
                        (getattr(self._u, "_lookup_retry", None)
                         or {}).pop(slot, None)
                    if down:
                        BambuSpoolman._lookup_unanswered(self, lane, slot, uid)
                    elif (restored
                          and spool is not None
                          and not BambuSpoolman._spool_weighed(spool, afc)):
                        BambuSpoolman._lookup_refuse(self, lane, slot, uid, spool.get("id"))
                    elif (spool is not None
                          and afc is not None
                          and getattr(afc, "spool", None) is not None
                          and getattr(lane, "spool_id", None) in (None, "", 0)
                          and not BambuSpoolman._tag_left(self, slot, uid)):
                        # AFC's set_spoolID binds when its fetch answers (on_done).
                        if _set_spoolid_takes_on_done(afc):
                            afc.spool.set_spoolID(
                                lane, spool.get("id"),
                                on_done=lambda: BambuSpoolman._bind_landed(
                                    self, lane, slot, uid))
                            landing = True
                        else:
                            afc.spool.set_spoolID(lane, spool.get("id"))
                            self._remember_bound_uid(slot, uid)
                        self._u.logger.debug(
                            f"{self._u.name}: matched {lane.name} to "
                            f"Spoolman spool {spool.get('id')} by "
                            + ("tray UID " + tray_uid
                               if by_tray is not None else "UID " + str(uid))
                            + str(note))
                    elif spool is None:
                        miss = getattr(self, "_spoolman_no_match", None)
                        if miss is None:
                            miss = self._spoolman_no_match = set()
                        miss.add(uid)
                        # No spool, so no bind: nothing is owed a measurement a later hand-bind
                        # should not inherit.
                        (getattr(self, "_bind_owed", None) or {}).pop(slot, None)
                except Exception:
                    pass
                finally:
                    # Drop the in-flight mark only after the bind, then drain the summary, unless
                    # AFC is still fetching the spool (_bind_landed drops it then).
                    inflight.discard(uid)
                    try:
                        if not landing:
                            (getattr(self, "_bind_pending", None)
                             or set()).discard(slot)
                        if slot is not None:
                            BambuSpoolman._drain_spool_summary(self, slot)
                    except Exception:
                        pass

            try:
                self._u.afc.reactor.register_async_callback(apply)
            except Exception:
                apply()

        BambuSpoolman._spoolman_bg(self, job)

    @staticmethod
    def _spool_weighed(spool: dict, afc: Any) -> bool:
        """
        Whether AFC will keep a lane linked to this spool.

        :param spool: the matched Spoolman spool
        :param afc: the AFC printer object
        :return bool: True when a bind will not clear the lane
        """
        check_off = getattr(getattr(afc, "spool", None), "disable_weight_check",
                            getattr(afc, "disable_weight_check", False))
        if check_off is True:
            return True
        try:
            return float(spool.get("remaining_weight")) > 0
        except (TypeError, ValueError):
            return False

    def _lookup_unanswered(self, lane: Any, slot: Optional[int], uid: str) -> None:
        """
        A restored lane's lookup that Spoolman did not answer: ask again later.

        :param lane: the lane the lookup was for
        :param slot: 0-based AMS slot index
        :param uid: the tag UID that was looked up
        """
        if (slot is None
            or not BambuSpoolman._bay_carries(self, slot, uid)):
            return
        u = self._u
        latch = getattr(u, "_spoolman_latched", None)
        if latch is not None:
            latch.discard(slot)
        retry = getattr(u, "_lookup_retry", None)
        if retry is None:
            retry = u._lookup_retry = {}
        wait = getattr(u, "LOOKUP_RETRY_S", 30.0)
        try:
            now = u.afc.reactor.monotonic()
        except Exception:
            now = 0.0
        retry[slot] = now + wait
        u.logger.debug(f"{u.name}: Spoolman did not answer the lookup of "
                       f"{str(uid).upper()} for {getattr(lane, 'name', lane)}; asking "
                       f"again in {wait:.0f} s")

    def _lookup_refuse(self, lane: Any, slot: Optional[int], uid: str, sid: Any) -> None:
        """
        A restored lane's match that AFC would clear the lane for: not bound.

        :param lane: the lane the lookup was for
        :param slot: 0-based AMS slot index
        :param uid: the tag UID that was looked up
        :param sid: the Spoolman spool id it matched
        """
        if not BambuSpoolman._bay_carries(self, slot, uid):
            return
        (getattr(self, "_bind_owed", None) or {}).pop(slot, None)
        u = self._u
        refused = getattr(u, "_lookup_refused", None)
        if refused is None:
            refused = u._lookup_refused = {}
        if slot is not None:
            refused[slot] = (str(uid), sid)
        name = getattr(lane, "name", lane)
        u.logger.info(f"{u.name}: {name} -- tag {str(uid).upper()} matches "
                      f"Spoolman spool {sid}, which has no remaining weight on record, "
                      f"so the lane was not linked to it (AFC clears a lane linked to a "
                      f"spool without one). Correct spool {sid}'s weight in Spoolman, "
                      f"then run AFC_BAMBU_SCAN LANE={name} to link it")

    def _bay_carries(self, slot: Optional[int], uid: str) -> bool:
        """
        Whether the bay still holds the tag a lookup was sent about.

        :param slot: bay index
        :param uid: the tag UID the lookup was for
        :return bool: True when the bay's record is present with that UID (or there is no record to
        ask, on a stand-in)
        """
        try:
            slots = getattr(self._u, "_slots", None)
            if (not isinstance(slots, list)
                or not 0 <= slot < len(slots)):
                return True
            rec = slots[slot] or {}
            return (bool(rec.get("present"))
                    and str(rec.get("rfid_uid") or "").lower()
                    == str(uid or "").lower())
        except Exception:
            return True

    def _tag_left(self, slot: Optional[int], uid: str) -> bool:
        """
        Whether the tag a bind was made from has left the bay.

        :param slot: bay index
        :param uid: the tag UID the bind was made from
        :return bool: True when the bay is empty, holds another tag, or was seen emptied and has not
        been read since
        """
        try:
            slots = getattr(self._u, "_slots", None)
            if (slot is None
                or not isinstance(slots, list)
                or not 0 <= slot < len(slots)):
                return False
            rec = slots[slot] or {}
            if not rec:
                return False
            if not rec.get("present"):
                return True
            ruid = str(rec.get("rfid_uid") or "").lower()
            if ruid:
                return ruid != str(uid or "").lower()
            return slot in (getattr(self._u, "_removed_bays", None) or ())
        except Exception:
            return False

    def _bind_landed(self, lane: Any, slot: Optional[int], uid: str, miss: bool = False) -> None:
        """
        A bind this module sent has run its course: settle it, then say the summary that was waiting
        on it.

        :param lane: the lane the bind was for
        :param slot: 0-based AMS slot index, or None when unknown
        :param uid: the tag UID the bind was made from
        :param miss: record an unbound answer in _spoolman_no_match
        """
        try:
            (getattr(self, "_bind_pending", None) or set()).discard(slot)
            bound = getattr(lane, "spool_id", None) not in (None, "", 0)
            if BambuSpoolman._tag_left(self, slot, uid):
                if bound:
                    u = self._u
                    u._clear_lane_filament(lane)
                    u._unbind_spool(lane, f"its Spoolman link for tag {str(uid).upper()} "
                                    f"landed after that spool left the bay")
                    # A reel nothing has read goes back on lane defaults.
                    rec = u._slots[slot] or {}
                    dflt = getattr(u, "_defaults_until_read", None)
                    if (rec.get("present")
                        and callable(dflt)
                        and slot not in (getattr(u, "_defaults_due", None) or {})):
                        dflt(slot, rec, say=False)
                    u._save_lane_vars()
            elif bound:
                self._remember_bound_uid(slot, uid)
            else:
                if miss:
                    no_match = getattr(self, "_spoolman_no_match", None)
                    if no_match is None:
                        no_match = self._spoolman_no_match = set()
                    no_match.add(uid)
                (getattr(self, "_bind_owed", None) or {}).pop(slot, None)
        except Exception:
            pass
        try:
            if slot is not None:
                BambuSpoolman._drain_spool_summary(self, slot)
        except Exception:
            pass

    def _surface_released(self, slot: Optional[int]) -> bool:
        """
        Whether the unit's surface path will dispatch this bay's bind.

        :param slot: bay index
        :return bool: True when the surface path is not held for this bay
        """
        return any(slot in (getattr(self._u, name, None) or ())
                   for name in ("_scanned_bays", "_cleared_bays"))

    def _sync_owed(self, slot: Optional[int], info: dict) -> bool:
        """
        Whether this bay's Spoolman bind is still to be dispatched.

        :param slot: bay index the measurement belongs to
        :param info: that bay's record from the unit
        :return bool: True while the bind is still to be dispatched
        """
        try:
            if not getattr(self, "spoolman_on", True):
                return False
            if (slot is None
                or not info.get("rfid_uid")):
                return False
            afc = getattr(self._u, "afc", None)
            if (afc is None
                or getattr(afc, "spoolman", None) is None):
                return False
            # getattr throughout: duck-typed stand-ins carry only what their case needs.
            latch = getattr(self._u, "_spoolman_latched", None)
            if latch is None:
                return False
            if BambuSpoolman._surface_released(self, slot):
                return slot not in latch
            coming = getattr(self._u, "_lookup_coming", None)
            return bool(coming(slot, info)) if callable(coming) else False
        except Exception:
            return False

    def _bind_coming(self, slot: Optional[int], info: dict) -> bool:
        """
        Whether a Spoolman bind may still attach a spool to this bay's lane.

        :param slot: bay index the measurement belongs to
        :param info: that bay's record from the unit
        :return bool: True while a bind for this bay is out or may still be sent
        """
        try:
            if (not getattr(self, "spoolman_on", True)
                or slot is None):
                return False
            afc = getattr(self._u, "afc", None)
            if (afc is None
                or getattr(afc, "spoolman", None) is None):
                return False
            if slot in (getattr(self, "_bind_pending", None) or ()):
                return True
            latch = getattr(self._u, "_spoolman_latched", None)
            if latch is None:
                return False
            if (BambuSpoolman._surface_released(self, slot)
                and slot not in latch):
                if info.get("rfid_uid"):
                    return True
                t0 = getattr(self._u, "_scan_t0", None) or []
                if (0 <= slot < len(t0)
                    and t0[slot] is not None):
                    return True
            coming = getattr(self._u, "_lookup_coming", None)
            return bool(coming(slot, info)) if callable(coming) else False
        except Exception:
            return False

    def _mark_bind_sent(self, slot: Optional[int]) -> None:
        """
        This module has just dispatched a Spoolman bind for this bay.

        :param slot: the bay the bind is for, or None when unknown
        """
        owed = getattr(self, "_bind_owed", None)
        held = owed.get(slot) if owed and slot is not None else None
        if (held is not None
            and not held[3]):
            owed[slot] = (held[0], held[1], held[2], True)

    def _spoolman_sync(self, lane: Any, info: dict, restored: bool = False) -> None:
        """
        Bind this bay in Spoolman, then release the summary that was waiting.

        :param lane: the AFC lane
        :param info: normalized bridge slot info (must carry rfid_uid)
        :param restored: the unit's lookup for a lane AFC restored with no spool (see
        _spoolman_sync_inner)
        """
        slot = info.get("index")
        try:
            if restored:
                BambuSpoolman._spoolman_sync_inner(self, lane, info, restored=True)
            else:
                BambuSpoolman._spoolman_sync_inner(self, lane, info)
        finally:
            try:
                owed = getattr(self, "_bind_owed", None)
                held = owed.get(slot) if owed and slot is not None else None
                if (held is not None
                    and not held[3]):
                    owed.pop(slot, None)
            except Exception:
                pass
            try:
                if (slot is not None
                    and slot not in (getattr(self, "_bind_pending", None) or ())):
                    BambuSpoolman._drain_spool_summary(self, slot)
            except Exception:
                pass

    def _spoolman_sync_inner(self, lane: Any, info: dict, restored: bool = False) -> None:
        """
        Bind this lane's spool to Spoolman by tag UID, creating it if allowed.

        :param lane: the AFC lane
        :param info: normalized bridge slot info (must carry rfid_uid)
        :param restored: the unit's lookup for a restored lane with no spool
        (afcBambuAMS._lookup_unbound), not a read of the bay
        """
        uid = info.get("rfid_uid")
        if (not uid
            or lane is None):
            return
        # An empty bay has no tag to bind.
        if not info.get("present"):
            return
        slot = info.get("index")
        bound = getattr(lane, "spool_id", None)
        if bound not in (None, "", 0):
            prev = (getattr(self, "_bound_uid", None) or {}).get(slot)
            if prev == uid:
                return
            if prev is None:
                # The memo does not survive a restart but the binding does, so ask Spoolman whether
                # the bound spool's UIDs exclude this tag.
                checker = getattr(self, "_binding_contradicted", None)
                if (checker is None
                    or not checker(bound, uid)):
                    return
            # A different tag is in the bay, so the binding names the previous spool.
            if prev is None:
                why = (f"tag {str(uid).upper()} is in this bay now, and "
                       f"Spoolman does not list it on that spool")
            else:
                why = f"tag {str(uid).upper()} is in this bay now, not {str(prev).upper()}"
            self._u._unbind_spool(lane, why)
        afc = getattr(self._u, "afc", None)
        if (afc is None
            or getattr(afc, "spoolman", None) is None):
            return
        si = self._spoolman_slot_info(info)
        have_profile = bool(si.get("material"))
        # A UID Spoolman does not know is remembered, not retried.
        if uid:
            miss = getattr(self, "_spoolman_no_match", None)
            if miss is None:
                miss = self._spoolman_no_match = set()
            if uid in miss:
                return
        try:
            allow = get_auto_spoolman_create(lane, getattr(self._u, "auto_spoolman_create", False))
            if (have_profile
                and allow
                and not restored):
                # With the reactor, the create's HTTP runs off it.
                pend_b = getattr(self, "_bind_pending", None)
                if pend_b is None:
                    pend_b = self._bind_pending = set()
                if slot is not None:
                    pend_b.add(slot)
                BambuSpoolman._mark_bind_sent(self, slot)
                sync_rfid_to_spoolman(
                    afc, lane, si, _QuietInfo(self._u.logger), "Bambu RFID",
                    allow_create=True, reactor=getattr(afc, "reactor", None),
                    on_done=lambda _s=slot: BambuSpoolman._bind_landed(
                        self, lane, _s, uid, miss=True))
            else:
                # Match-only and off-reactor.
                BambuSpoolman._bind_by_uid_bg(self, lane, slot, uid,
                                              "" if have_profile else " (no tag profile decoded)",
                                              tray_uid=info.get("tray_uid") or "",
                                              restored=restored)
        except Exception:
            try:
                self._u.logger.debug("BridgeBox: spoolman sync failed",
                                     traceback=traceback.format_exc())
            except Exception:
                self._u.logger.debug("BridgeBox: spoolman sync failed")

    def _apply_remain_weight(self, lane: Any, info: dict) -> None:
        """
        Finish what a measurement still owes this bay, once.

        :param lane: the AFC lane
        :param info: normalized bridge slot info (index)
        """
        idx = info.get("index")
        owed = getattr(self, "_bind_owed", None) or {}
        conv = getattr(self, "_convert_owed", None) or {}
        if (lane is None
            or (idx not in owed and idx not in conv)):
            return
        # Extrusion owns the weight from here, and a spool that has been fed no longer holds what
        # was measured.
        if getattr(lane, "tool_loaded", False):
            owed.pop(idx, None)
            conv.pop(idx, None)
            return
        if idx in owed:
            BambuSpoolman._settle_bind_owed(self, lane, idx, owed)
        if idx in conv:
            BambuSpoolman._settle_convert_owed(self, lane, idx, conv)

    def _settle_bind_owed(self, lane: Any, idx: int, owed: dict) -> None:
        """
        Hand a measurement to the spool this bay's bind attached after it.

        :param lane: the AFC lane
        :param idx: 0-based AMS slot index
        :param owed: the ``_bind_owed`` dict holding this bay's claim
        """
        pct, nominal, prior, sent = owed[idx]
        sid = getattr(lane, "spool_id", None)
        if sid in (None, "", 0):
            afc = getattr(self._u, "afc", None)
            if (not getattr(self, "spoolman_on", True)
                or afc is None
                or getattr(afc, "spoolman", None) is None):
                owed.pop(idx, None)
            return
        if sid == prior:
            # The spool the adoption wrote.
            if (sent
                and idx not in (getattr(self, "_bind_pending", None) or ())):
                owed.pop(idx, None)
            return
        owed.pop(idx, None)
        if not sent:
            return
        grams = BambuSpoolman._grams_now(self, idx, lane, pct, nominal)
        # The lane takes it either way.
        lane.weight = grams
        self._u.logger.debug(f"{self._u.name}: {getattr(lane, 'name', '?')} bound "
                             f"to spool {sid} after its measurement; handing the measured "
                             f"{grams} g to it")
        self._push_measured_to_spoolman(lane, grams, "physical AMS measurement", pct=pct,
                                        nominal=nominal)
        # Grams made without a density are still owed their material, against the figure just
        # written.
        conv = getattr(self, "_convert_owed", None)
        if (conv is not None
            and idx in conv):
            if BambuSpoolman._density_known(self, idx, lane):
                conv.pop(idx, None)
            else:
                conv[idx] = (pct, nominal, grams)
        _save = getattr(self._u, "_save_lane_vars", None)
        if callable(_save):
            _save()

    def _settle_convert_owed(self, lane: Any, idx: int, conv: dict) -> None:
        """
        Weigh a measurement at its density once the bay's material is known.

        :param lane: the AFC lane
        :param idx: 0-based AMS slot index
        :param conv: the ``_convert_owed`` dict holding this bay's claim
        """
        pct, nominal, written = conv[idx]
        try:
            unchanged = int(getattr(lane, "weight", 0) or 0) == int(written)
        except (TypeError, ValueError):
            unchanged = False
        if not unchanged:
            conv.pop(idx, None)
            return
        if not BambuSpoolman._density_known(self, idx, lane):
            return
        conv.pop(idx, None)
        grams = BambuSpoolman._grams_now(self, idx, lane, pct, nominal)
        if grams == int(written):
            return
        lane.weight = grams
        self._u.logger.debug(f"{self._u.name}: {getattr(lane, 'name', '?')} -- "
                             f"{pct}% weighed at its material now it is known: {grams} g, not "
                             f"the {written} g written before the tag landed")
        self._push_measured_to_spoolman(lane, grams, "physical AMS measurement", pct=pct,
                                        nominal=nominal)
        # A summary still held for the record quotes the grams it was queued with.
        pend = getattr(self, "_pending_summary", None) or {}
        held = pend.get(idx)
        if held:
            pend[idx] = (held[0], grams) + tuple(held[2:])
        _save = getattr(self._u, "_save_lane_vars", None)
        if callable(_save):
            _save()

    def _grams_now(self, slot: int, lane: Any, pct: int, nominal: int) -> int:
        """
        A measured percent as grams, through the capacity model as it stands.

        :param slot: 0-based AMS slot index
        :param lane: the AFC lane
        :param pct: the percent, already floored
        :param nominal: the tag's declared full weight in grams
        :return int: grams, at least 1
        """
        _gf = getattr(self, "_grams_for", None)
        return (_gf(slot, lane, pct, nominal) if _gf is not None
                else max(1, (int(nominal) * min(pct, 100)) // 100))

    def _density_known(self, slot: int, lane: Any) -> bool:
        """
        Whether anything names a density for this bay's filament yet.

        :param slot: 0-based AMS slot index
        :param lane: the AFC lane
        :return bool: False when unknown, or on an object without the model
        """
        try:
            _dn = getattr(self, "_density_of", None)
            return bool(_dn(slot, lane)) if _dn is not None else False
        except Exception:
            return False

    def _push_measured_to_spoolman(self, lane: Any, grams: int, source: str = "", pct: int = 0,
                                   nominal: int = 0) -> None:
        """
        Write a remaining-weight figure back to the lane's Spoolman spool.

        :param lane: the AFC lane
        :param grams: remaining net weight, grams
        :param source: where the figure came from, for the log
        :param pct: the measured percent, for the log when grams were capped
        :param nominal: the spool's full weight in grams, for the same log
        """
        # The one HTTP path in the whole measurement closure, so the one place a measurement-only
        # delegate has to stop.
        if not getattr(self, "spoolman_on", True):
            return
        if not getattr(self._u, "sync_measured_to_spoolman", True):
            return
        if lane is None:
            return
        sid = getattr(lane, "spool_id", None)
        if sid in (None, "", 0):
            return
        afc = getattr(self._u, "afc", None)
        client = _bambu_spoolman_client(afc)
        setter = getattr(client, "set_remaining_weight", None)
        if (client is None
            or not callable(setter)):
            return
        # Off the reactor: set_remaining_weight is two blocking HTTP calls.
        src = source or "physical AMS measurement"

        def _job(sid: Any = sid, grams: float = float(grams), source: str = src,
                 pct: int = int(pct or 0), nominal: int = int(nominal or 0)) -> None:
            """
            Worker-thread half: the one Spoolman weight write.

            :param sid: the Spoolman spool id
            :param grams: measured grams remaining
            :param source: where the measurement came from, for the log
            :param pct: the measured percent, logged when over 100
            :param nominal: the spool's full weight in grams
            """
            try:
                setter(sid, grams)
                # When the grams were capped at nominal, print the measured percent too.
                _cap = (f" -- measured {pct}%, capped to the spool's "
                        f"{nominal} g nominal"
                        if pct > 100 and nominal else "")
                self._u.logger.debug(f"{self._u.name}: wrote {grams} g remaining to "
                                     f"Spoolman spool {sid} ({source}){_cap}")
            except Exception as e:
                self._u.logger.debug(f"{self._u.name}: Spoolman weight write failed: {e}")
        BambuSpoolman._spoolman_bg(self, _job)
        return

    def _queue_spool_summary(self, slot: int, pct: int, grams: int, nominal: int) -> None:
        """
        Hold the operator's summary until the bay's record can answer it.

        :param slot: bay index the measurement belongs to
        :param pct: measured remaining percent, RAW (may exceed 100)
        :param grams: remaining weight in grams, already capped
        :param nominal: the spool's full weight in grams
        """
        pend = getattr(self, "_pending_summary", None)
        if pend is None:
            pend = self._pending_summary = {}
        try:
            deadline = self._u.afc.reactor.monotonic() + self._u.SCAN_FALLBACK_CAP
        except Exception:
            deadline = None
        # A miss judges one lookup, not the UID forever.
        try:
            uid0 = ""
            for sl in (self._u._slots or []):
                if sl.get("index") == slot:
                    uid0 = str(sl.get("rfid_uid") or "").lower()
                    break
            miss0 = getattr(self, "_spoolman_no_match", None)
            latched0 = slot in (getattr(self._u, "_spoolman_latched", None)
                                or ())
            if (uid0
                and miss0
                and not latched0):
                for u0 in {u for u in miss0 if str(u).lower() == uid0}:
                    miss0.discard(u0)
        except Exception:
            pass
        pend[slot] = (pct, grams, nominal, deadline)
        self._drain_spool_summary(slot)

    def _drain_spool_summary(self, slot: int) -> None:
        """
        Say a held summary once the bay's record can answer it, or give up.

        :param slot: bay index the measurement belongs to
        """
        pend = getattr(self, "_pending_summary", None) or {}
        held = pend.get(slot)
        if not held:
            return
        pct, grams, nominal, deadline = held
        info = {}
        for sl in (self._u._slots or []):
            if sl.get("index") == slot:
                info = sl or {}
                break
        # Wait for the record or the backstop.
        ready = bool(info.get("material") or info.get("rfid_uid"))
        # On an HT the UID arrives well before the profile, so a bare UID settles nothing while
        # _scan_verdict says "waiting".
        if (ready
            and not info.get("material")):
            try:
                if self._u._scan_verdict(slot) == "waiting":
                    ready = False
            except Exception:
                pass
        # A Spoolman lookup in flight is unsettled too.
        if ready:
            try:
                uid = str(info.get("rfid_uid") or "").lower()
                inflight = getattr(self, "_spoolman_inflight", None) or set()
                if (uid
                    and uid in {str(u).lower() for u in inflight}):
                    ready = False
            except Exception:
                pass
        # Wait for the bind's answer, not a clock.
        if (ready
            and slot in (getattr(self, "_bind_pending", None) or ())):
            ready = False
        # And a bind not yet dispatched (see _sync_owed).
        _owed = getattr(self, "_sync_owed", None)
        if (ready
            and _owed is not None
            and _owed(slot, info)):
            ready = False
        settled = ready
        if not ready:
            try:
                ready = (deadline is None
                         or self._u.afc.reactor.monotonic() >= deadline)
            except Exception:
                ready = True
        if not ready:
            return
        # Finish the density conversion first (_settle_convert_owed) so the line quotes the lane's
        # final grams.
        lane = self._u._lane_for_slot(slot)
        conv = getattr(self, "_convert_owed", None) or {}
        if (slot in conv
            and lane is not None
            and not getattr(lane, "tool_loaded", False)):
            try:
                BambuSpoolman._settle_convert_owed(self, lane, slot, conv)
                grams = (pend.get(slot) or held)[1]
            except Exception:
                pass
        pend.pop(slot, None)
        self._say_spool_summary(slot, lane, pct, grams, nominal, settled=settled)

    def _lookup_open(self, slot: Optional[int], info: dict) -> bool:
        """
        Whether this bay's Spoolman lookup is out, owed, to be asked again, or sent with a
        measurement held for it and not bound yet: anything the summary may not answer for yet.

        :param slot: bay index
        :param info: that bay's record from the unit
        :return bool: True when an answer is still to come
        """
        try:
            uid = str(info.get("rfid_uid") or "").lower()
            if (uid
                and uid in {str(x).lower() for x in
                            (getattr(self, "_spoolman_inflight", None) or ())}):
                return True
            if slot in (getattr(self, "_bind_pending", None) or ()):
                return True
            owed = getattr(self, "_sync_owed", None)
            if (callable(owed)
                and owed(slot, info)):
                return True
            # A retry counts only while the unit will still send it.
            if slot in (getattr(self._u, "_lookup_retry", None) or {}):
                coming = getattr(self._u, "_lookup_coming", None)
                if (not callable(coming)
                    or coming(slot, info)):
                    return True
            # A bind this module sent that has not bound the lane yet.
            held = (getattr(self, "_bind_owed", None) or {}).get(slot)
            return bool(held and held[3])
        except Exception:
            return False

    def _lookup_refused_for(self, slot: Optional[int], uid: Any) -> Any:
        """
        The spool a restored lane's lookup matched and would not bind for want of a remaining
        weight, if that is this tag's answer.

        :param slot: bay index
        :param uid: the bay's tag UID
        :return Any: the Spoolman spool id, or None
        """
        try:
            ref = (getattr(self._u, "_lookup_refused", None) or {}).get(slot)
            if (ref
                and str(ref[0]).lower() == str(uid or "").lower()):
                return ref[1]
        except Exception:
            pass
        return None

    def _say_spool_summary(self, slot: int, lane: Any, pct: int, grams: int, nominal: int,
                           settled: bool = True) -> None:
        """
        One plain-English line for the operator when a spool is measured.

        :param slot: bay index the measurement belongs to
        :param lane: the AFC lane, or None
        :param pct: measured remaining percent, RAW (may exceed 100)
        :param grams: remaining weight in grams, already capped
        :param nominal: the spool's full weight in grams
        :param settled: False when the backstop fired before the tag's profile arrived, so a missing
        profile is not called a decode failure
        """
        info = {}
        for sl in (self._u._slots or []):
            if sl.get("index") == slot:
                info = sl or {}
                break
        where = getattr(lane, "name", None) or f"bay {slot}"
        # What the tag said, if anything.
        material = info.get("material")
        colour = info.get("color")
        uid = info.get("rfid_uid")
        if material:
            what = f"{material}" + (f" ({colour})" if colour else "")
            # "tag read" is a claim about this scan.
            _res = info.get("scan_res")
            if (_res is None
                or _res == 1):
                read = f"tag read: {what}"
            else:
                read = f"tag on file: {what}"
            if uid:
                read += f" [tag {str(uid).upper()}]"
        elif (uid
              and not settled):
            # The backstop fired before the profile arrived.
            read = (f"tag {str(uid).upper()} -- the profile had not arrived "
                    f"yet when this was reported, so the material is not in "
                    f"this line. It lands on the lane by itself a moment "
                    f"later; nothing needs doing")
        elif (uid
              and not _profile_landed(info)):
            read = (f"tag {str(uid).upper()} read but its profile could not be "
                    f"decoded (not a Bambu tag?)")
            # The advice only holds with the Spoolman module on.
            if getattr(self, "spoolman_on", True):
                read += " -- bind that UID to a spool in Spoolman and it will match from now on"
        elif uid:
            # The profile decoded.
            _sku = info.get("sku")
            read = (f"tag {str(uid).upper()} read"
                    + (f": {_sku}" if _sku else ""))
        else:
            read = "no tag on this spool"
        # A reading held down by _remain_floor_pct says so, or the figure looks stuck beside a
        # higher narrated percent.
        read_pct = None
        try:
            read_pct = (getattr(self, "_summary_read", None) or {}).get(slot)
        except Exception:
            read_pct = None
        held = (f" (the AMS read {read_pct}% this time against {pct}% before "
                f"it; filament does not grow, so the lower reading stands)"
                if read_pct is not None and int(read_pct) > int(pct) else "")
        if pct > 100:
            # Over 100% is a spool larger than the reference full radius (the percent is a volume
            # ratio), not extra filament.
            why = (held
                   or (f" (the AMS read {pct}%, meaning it measures a "
                       f"little larger than a reference full spool)"))
            amount = f"full -- roughly {grams} g of a {nominal} g spool{why}"
        else:
            amount = (f"about {pct}% left -- roughly {grams} g of a {nominal} g spool" + held)
        sid = getattr(lane, "spool_id", None)
        if not getattr(self, "spoolman_on", True):
            # No claim about Spoolman: nothing here talked to it.
            went = ("kept on the lane -- the Spoolman module "
                    "([AFC_BambuAMS_rfid]) is off"
                    + (f", so Spoolman spool {sid} was not updated"
                       if sid not in (None, "", 0) else ""))
        elif not getattr(self._u, "sync_measured_to_spoolman", True):
            went = "Spoolman sync is off, so this is kept on the lane only"
        elif sid in (None, "", 0):
            # The "not linked" prefix is a claim, so only branches that established it use it.
            unlinked = "not linked to a Spoolman spool, so this is kept on the "
            if not uid:
                went = unlinked + "lane only"
            elif BambuSpoolman._lookup_open(self, slot, info):
                # Asked, or about to be, and not answered yet.
                held = slot in (getattr(self, "_bind_owed", None) or {})
                went = (f"a Spoolman lookup for {str(uid).upper()} is in "
                        f"progress; the measurement "
                        + ("goes to the spool it finds" if held
                           else "is on the lane for now"))
            elif not material:
                went = unlinked + (f"lane only -- bind {str(uid).upper()} to a "
                                   f"spool in Spoolman to track this reel")
            elif str(uid).lower() in {
                    str(m).lower() for m in
                    (getattr(self, "_spoolman_no_match", None) or ())}:
                # Asked and answered: _spoolman_no_match is written only when a lookup came back
                # empty.
                went = unlinked + f"lane only -- Spoolman has no spool carrying {str(uid).upper()}"
                if not getattr(self._u, "auto_spoolman_create", False):
                    went += (f" and auto-create is off for this unit (add "
                             f"'auto_spoolman_create: True' to "
                             f"[AFC_BambuAMS {self._u.name}], or bind that UID "
                             f"to an existing spool)")
            elif BambuSpoolman._lookup_refused_for(self, slot, uid) is not None:
                # Asked, matched, and not bound.
                rsid = BambuSpoolman._lookup_refused_for(self, slot, uid)
                went = unlinked + (f"lane only -- {str(uid).upper()} matches Spoolman spool "
                                   f"{rsid}, which has no remaining weight on record; "
                                   f"correct spool {rsid}'s weight in Spoolman, then run "
                                   f"AFC_BAMBU_SCAN LANE={where} to link it")
            elif slot in (getattr(self._u, "_spoolman_latched", None) or ()):
                # The lookup went out on this connection and nothing it found is on the lane now
                # (the link was cleared since, say).
                went = (f"{str(uid).upper()} was looked up in Spoolman on this "
                        f"connection and no spool is linked to the lane from "
                        f"it, so the measurement stays on the lane only -- run "
                        f"AFC_BAMBU_SCAN LANE={where} to look it up again")
            else:
                went = (f"no Spoolman lookup has been made for "
                        f"{str(uid).upper()}, so the measurement stays on the "
                        f"lane only -- run AFC_BAMBU_SCAN LANE={where} to link it")
        else:
            went = f"updated Spoolman spool {sid}"
        line = f"{self._u.name} {where}: {read}. Measured {amount}; {went}."
        if getattr(self._u.logger, "print_debug_console", False):
            self._u.logger.info(line)
            return
        self._u.logger.debug(line)
        left = (f"full ({grams} g)" if pct > 100
                else f"about {pct}% left ({grams} g)")
        self._u.logger.info(f"{self._u.name} {where}: {what if material else 'no tag'}, {left}")

    #: Volume of filament at the unit's own 100% reference, in cm3.
    CAP_REF_VOLUME_CM3 = 663.0

    #: The AMS's reference geometry.
    CAP_HUB_MM = 47.5
    CAP_FULL_MM = 82.6

    @staticmethod
    def _material_key(lane: Any, info: dict) -> str:
        """
        The most SPECIFIC name for what is on the reel, for a density lookup.

        :param lane: the AFC lane (may be None)
        :param info: the bay's normalized slot info
        :return str: e.g.
        """
        full = (info or {}).get("material") or ""
        if len(full.split()) > 1:
            return full
        base = (getattr(lane, "material", None) or full or "")
        sub = getattr(lane, "sub_type", "") or ""
        # "PLA Basic" and "PLA" look up the same.
        return (f"{base} {sub}".strip()
                if (sub
                    and sub.lower() not in base.lower())
                else base)

    def _density_of(self, slot: Optional[int], lane: Any) -> Optional[float]:
        """
        The density the capacity model weighs this bay's filament at.

        :param slot: 0-based AMS slot index
        :param lane: the AFC lane (may be None)
        :return Optional[float]: g/cm3, or None when unknown
        """
        info = {}
        for sl in (self._u._slots or []):
            if sl.get("index") == slot:
                info = sl
                break
        material = self._material_key(lane, info)
        density = getattr(lane, "density", None)
        if (not density
            and material):
            density = density_for_material(material)
        return density or None

    def _grams_for(self, slot: int, lane: Any, pct: int, nominal: int) -> int:
        """
        Turn a measurement into grams: MASS, not the percent read as a mass.

        :param slot: 0-based AMS slot index
        :param lane: the AFC lane (its Spoolman density wins over the table)
        :param pct: the RAW measured percent, uncapped
        :param nominal: the tag's declared full weight in grams
        :return int: grams to write, at least 1
        """
        tag_linear = max(1, (int(nominal) * min(pct, 100)) // 100)
        try:
            density = self._density_of(slot, lane)
            if not density:
                return tag_linear
            # Prefer the radius.
            pct_eff = float(pct)
            rec = None
            try:
                rec = self._u._bridge.last_cap_measure(
                    getattr(self._u, "dry_dev_addr", 0)) if self._u._bridge \
                    else None
            except Exception:
                rec = None
            if (rec
                and rec.get("pct_raw") == pct
                and rec.get("save_radius_m")):
                r_mm = float(rec["save_radius_m"]) * 1000.0
                hub2 = self.CAP_HUB_MM ** 2
                span = self.CAP_FULL_MM ** 2 - hub2
                if (r_mm > self.CAP_HUB_MM
                    and span > 0):
                    pct_eff = 100.0 * (r_mm ** 2 - hub2) / span
            grams = int(round(density * self.CAP_REF_VOLUME_CM3
                              * pct_eff / 100.0))
            return max(1, min(grams, int(nominal)))
        except Exception:
            # Inventory must never depend on this arithmetic surviving.
            return tag_linear

    def _pct_for(self, slot: Optional[int], lane: Any, grams: Any, nominal: Any) -> Optional[float]:
        """
        The inverse of _grams_for: the percent a lane's grams stand for.

        :param slot: 0-based AMS slot index
        :param lane: the AFC lane (its density wins over the table)
        :param grams: the lane's remaining grams
        :param nominal: the tag's declared full weight in grams
        :return Optional[float]: the percent, unrounded and uncapped, or None when there are no
        grams or no nominal to read it against
        """
        try:
            g = float(grams or 0)
            n = float(nominal or 0)
        except (TypeError, ValueError):
            return None
        if (g <= 0
            or n <= 0):
            return None
        try:
            density = self._density_of(slot, lane)
        except Exception:
            density = None
        if not density:
            return g * 100.0 / n
        return g * 100.0 / (float(density) * self.CAP_REF_VOLUME_CM3)

    def _log_capacity_sample(self, slot: int, lane: Any, pct: int, grams: int, nominal: int,
                             source: str = "") -> None:
        """
        Record one capacity measurement, raw, for fitting grams from radius.

        :param slot: 0-based AMS slot index the measurement belongs to
        :param lane: the AFC lane, for density/spool identity (may be None)
        :param pct: the RAW measured percent, uncapped
        :param grams: the grams actually written, i.e. after the 100% cap
        :param nominal: the tag's declared full weight in grams
        :param source: what produced the measurement, as _adopt_measured_remain names it; a live
        capscan and the firmware's own stamp of a cycle the narration missed are different rows, and
        a fit should be able to tell them apart
        """
        try:
            info = {}
            for sl in (self._u._slots or []):
                if sl.get("index") == slot:
                    info = sl
                    break
            material = self._material_key(lane, info)
            # The lane's density is Spoolman's, which is the filament's own figure.
            density = getattr(lane, "density", None)
            if not density:
                density = density_for_material(material)
            # Use a radius source only when its own percent agrees with this
            # measurement, and name the source in the row:
            #   narration: the live line, the only one carrying C: as well
            #   slotrec: the firmware's stamp (AFC-2.72), which survives a
            #   restart
            radius_m = circ_m = save_r = None
            radius_src = "none"
            try:
                rec = self._u._bridge.last_cap_measure(
                    getattr(self._u, "dry_dev_addr", 0)) if self._u._bridge \
                    else None
                if rec:
                    if rec.get("pct_raw") == pct:
                        radius_m = rec.get("radius_m")
                        circ_m = rec.get("circumference_m")
                        # The "odom save" line's radius, unrounded.
                        save_r = rec.get("save_radius_m")
                        radius_src = "narration"
                    else:
                        radius_src = f"stale(rec={rec.get('pct_raw')})"
                if radius_m is None:
                    # Same identity test, against the slot record's own pair.
                    r_mm = info.get("meas_radius_mm")
                    if (r_mm
                        and info.get("meas_pct") == pct):
                        radius_m = r_mm / 1000.0
                        radius_src = "slotrec"
            except Exception:
                pass
            modelled = None
            if (density
                and pct):
                modelled = density * self.CAP_REF_VOLUME_CM3 * (pct / 100.0)
            self._u.logger.debug(f"capsample unit={self._u.name} slot={slot} "
                                 f"lane={getattr(lane, 'name', '?')} "
                                 f"spool={getattr(lane, 'spool_id', None)} "
                                 f"material={material!r} density={density} "
                                 f"pct_raw={pct} radius_m={radius_m} circ_m={circ_m} "
                                 f"save_r_m={save_r} radius_src={radius_src} source={source!r} "
                                 f"nominal={nominal} grams_written={grams} "
                                 f"grams_modelled={f'{modelled:.0f}' if modelled else None}")
        except Exception:
            # Instrumentation must never be able to break a measurement.
            pass

    def _remain_floor_pct(self, uid: str, pct: int) -> int:
        """
        The lowest remain% this reel has measured, which is the honest one.

        :param uid: the tag's chip UID, the reel's identity
        :param pct: the percent just measured, RAW
        :return int: the percent to report and store this reading as
        """
        try:
            if not uid:
                return pct
            floor = getattr(self, "_remain_floor", None) or {}
            prev = floor.get(str(uid).lower())
            return pct if prev is None else min(int(pct), int(prev))
        except Exception:
            return pct

    def _record_remain_floor(self, uid: str, pct: int) -> None:
        """
        Remember this reel's lowest measurement.

        :param uid: the tag's chip UID
        :param pct: the percent being adopted, already floored
        """
        try:
            if not uid:
                return
            floor = getattr(self, "_remain_floor", None)
            if floor is None:
                floor = self._remain_floor = {}
            key = str(uid).lower()
            prev = floor.get(key)
            if (prev is None
                or int(pct) < int(prev)):
                floor[key] = int(pct)
        except Exception:
            pass

    def _adopt_measured_remain(self, slot: int, pct: int, source: str = "capscan",
                               seq: Optional[int] = None) -> bool:
        """
        Record a physical spool measurement: slot remain%, lane grams, Spoolman.

        :param slot: slot index the measurement belongs to
        :param pct: remaining percent, RAW (may legitimately exceed 100)
        :param source: what produced it, for the log line
        :param seq: the firmware's meas_seq for this measurement, when the caller has one
        :return bool: True if the measurement was accepted
        """
        # Sanity range.
        if not (0 < pct <= 200):
            return False
        # Persist module-side.
        if not hasattr(self, "_measured_remain"):
            self._measured_remain = {}
        # Dedupe on the measurement's identity (meas_seq), not its value.
        if not hasattr(self, "_meas_seq_seen"):
            self._meas_seq_seen = {}
        if seq is not None:
            if self._meas_seq_seen.get(slot) == seq:
                return True
            self._meas_seq_seen[slot] = seq
            # Same number under a new sequence is the other path catching up, not a new measurement.
            if self._measured_remain.get(slot) == pct:
                return True
        elif self._measured_remain.get(slot) is not None:
            # Unsequenced, and this slot already has a measurement.
            return True
        elif self._measured_remain.get(slot) == pct:
            return True
        self._measured_remain[slot] = pct
        nominal = 1000
        uid = ""
        for sl in (self._u._slots or []):
            if sl.get("index") == slot:
                nominal = sl.get("weight") or 1000
                uid = str(sl.get("rfid_uid") or "")
                break
        # _measured_remain keeps the raw reading.
        pct_read = pct
        _floor = getattr(self, "_remain_floor_pct", None)
        if _floor is not None:
            pct = _floor(uid, pct)
        _record = getattr(self, "_record_remain_floor", None)
        if _record is not None:
            _record(uid, pct)
        if pct != pct_read:
            self._u.logger.debug(f"{self._u.name}: slot {slot} measured {pct_read}%, "
                                 f"held at the {pct}% this reel measured before -- filament "
                                 f"does not grow, and the odometer is worth about +/-3%")
        lane = self._u._lane_for_slot(slot)
        # Grams are capped at the tag's nominal weight.
        try:
            (getattr(self._u, "_fresh_insert", None) or {}).pop(slot, None)
        except Exception:
            pass
        # Duck-typed stand-ins reach this function without the rest of the class.
        grams = BambuSpoolman._grams_now(self, slot, lane, pct, nominal)
        _sample = getattr(self, "_log_capacity_sample", None)
        if _sample is not None:
            _sample(slot, lane, pct, grams, nominal, source)
        if lane is not None:
            lane.weight = grams
        # The raw reading, for the operator line.
        try:
            seen = getattr(self, "_summary_read", None)
            if seen is None:
                seen = self._summary_read = {}
            seen[slot] = pct_read
        except Exception:
            pass
        self._queue_spool_summary(slot, pct, grams, nominal)
        # Correct a bound Spoolman spool's remaining_weight to what the AMS measured, gated by
        # sync_measured_to_spoolman.
        self._push_measured_to_spoolman(lane, grams, pct=pct, nominal=nominal)
        # Owed to a bind that has not landed.
        owed = getattr(self, "_bind_owed", None)
        if owed is not None:
            owed.pop(slot, None)
            try:
                info = next((sl for sl in (self._u._slots or [])
                             if sl.get("index") == slot), {})
                if (lane is not None
                    and BambuSpoolman._bind_coming(self, slot, info)):
                    sid = getattr(lane, "spool_id", None)
                    owed[slot] = (pct, nominal, None if sid in (None, "", 0) else sid,
                                  slot in (getattr(self, "_bind_pending", None) or ()))
            except Exception:
                pass
        # Owed its material.
        conv = getattr(self, "_convert_owed", None)
        if conv is not None:
            conv.pop(slot, None)
            if (lane is not None
                and not BambuSpoolman._density_known(self, slot, lane)):
                conv[slot] = (pct, nominal, grams)
        # Persist it: an unbound lane has no Spoolman re-hydration, so without this the measurement
        # would not survive a restart.
        self._u._save_lane_vars()
        return True


class AFC_BambuAMS_RFID:
    """
    The [AFC_BambuAMS_rfid] Klipper object.

    :param config: the Klipper config wrapper for the section
    """

    #: Class defaults so an object that skipped __init__ still works.
    enabled: bool = True
    _units: set = frozenset()

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Read the section; per-unit helpers attach lazily.

        :param config: the [AFC_BambuAMS_rfid] section
        """
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Read at least one option, or Klipper rejects a mixed-case section that reads nothing as
        # "not a valid config section".
        self.enabled = config.getboolean("enabled", True)
        #: Names of the units that have asked for a delegate, so the status shows what is actually
        #: wired rather than just "configured".
        self._units: set = set()
    def get_status(self, eventtime: Optional[float] = None) -> dict:
        """
        Status for the API, and the only way to SEE this object.

        :param eventtime: reactor time (unused)
        :return dict: enabled flag and the units currently delegating to us
        """
        return {"enabled": bool(self.enabled), "units": sorted(self._units)}

    def for_unit(self, unit: Any) -> Optional[BambuSpoolman]:
        """
        Build the Spoolman delegate for one AMS unit.

        :param unit: the afcBambuAMS asking for its delegate
        :return Optional[BambuSpoolman]: the delegate the unit's shims call through, or None when
        the section is present but disabled
        """
        if not self.enabled:
            return None
        try:
            self._units.add(str(getattr(unit, "name", unit)))
        except Exception:
            pass
        return BambuSpoolman(unit)


def measurement_only(unit: Any) -> BambuSpoolman:
    """
    The delegate for a unit that no Spoolman section serves.

    :param unit: the afcBambuAMS it serves
    :return BambuSpoolman: the measurement-only delegate
    """
    return BambuSpoolman(unit, spoolman=False)


def load_config(config: "ConfigWrapper") -> AFC_BambuAMS_RFID:
    """
    Klipper entry point for [AFC_BambuAMS_rfid].

    :param config: the Klipper config wrapper
    :return AFC_BambuAMS_RFID: the configured object
    """
    return AFC_BambuAMS_RFID(config)
