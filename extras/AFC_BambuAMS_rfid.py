# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Bambu AMS <-> Spoolman integration: lookup/bind/sync, UID binding memos and
# the measured remaining weight. The AMS reads its own tags, so AFC_BambuAMS
# keeps scanning and tag application.
#
# Enabled by an [AFC_BambuAMS_rfid] section; without one the unit's Spoolman
# shims no-op. A unit with no Spoolman delegate still gets a measurement-only
# BambuSpoolman, so a capscan lands without Spoolman.
from __future__ import annotations

import chelper
import traceback
from typing import Any, Callable, Dict, Optional, Set, TYPE_CHECKING

if TYPE_CHECKING:
    from configfile import ConfigWrapper

# Imported from the unit module to keep one definition; that module imports
# this one back lazily, so there is no import cycle.
from extras.AFC_BambuAMS import BAMBU_BRAND, _split_bambu_material

#: Why AFC_RFID could not be imported, or None. The measurement-only delegate
#: works without it; for_unit() refuses a Spoolman delegate while this is set.
_AFC_RFID_ERR: Optional[str] = None
try:
    from extras.AFC_RFID import (sync_rfid_to_spoolman,
                                 get_auto_spoolman_create,
                                 find_spool_by_uid, match_spool_for_tag,
                                 SpoolmanClient, density_for_material,
                                 _spool_uids, _norm_uid, _norm_tray_uid,
                                 _spool_tray_uid, _set_spoolid_takes_on_done)
except Exception as _e:                     # AFC_RFID not deployed
    _AFC_RFID_ERR = f"{type(_e).__name__}: {_e}"
    # Same fallback shape as the unit module's own AFC_RFID import.
    sync_rfid_to_spoolman = None                 # type: ignore[assignment]
    get_auto_spoolman_create = None              # type: ignore[assignment]
    find_spool_by_uid = None                     # type: ignore[assignment]
    match_spool_for_tag = None                   # type: ignore[assignment]
    SpoolmanClient = None                        # type: ignore[misc,assignment]
    density_for_material = None                  # type: ignore[assignment]
    _spool_uids = None                           # type: ignore[assignment]
    _norm_uid = None                             # type: ignore[assignment]
    _norm_tray_uid = None                        # type: ignore[assignment]
    _spool_tray_uid = None                       # type: ignore[assignment]
    _set_spoolid_takes_on_done = None            # type: ignore[assignment]


def _bambu_spoolman_client(afc: Any) -> Optional[Any]:
    """
    Build the SpoolmanClient the way every AFC reader does.

    afc.spoolman is only a configured flag/URL, NOT the client, so calling
    client methods on it silently does nothing.

    :param afc: the AFC printer object
    :return Optional[SpoolmanClient]: the client, or None if Spoolman or
        moonraker is unavailable
    """
    if (SpoolmanClient is None
        or afc is None
        or getattr(afc, "spoolman", None) is None
        or getattr(afc, "moonraker", None) is None):
        return None
    try:
        # The shared per-afc cache (AFC_RFID._cached_spoolman_client), so the
        # client's ensure-fields memos survive between calls here too.
        try:
            from extras.AFC_RFID import _cached_spoolman_client
            return _cached_spoolman_client(afc)
        except Exception:
            return SpoolmanClient(afc.moonraker)
    except Exception:
        return None


# Fields only a decoded Bambu profile fills. Any present means the profile
# decoded; `material` alone is racy while the poll catches up.
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

    The shared AFC_RFID binder logs its assignment at INFO, which other
    readers rely on. On a Bambu scan the unit's own summary already says it,
    and the binder's line quotes Spoolman's previous weight, so it is demoted
    here rather than in the shared module.
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
        try:
            self._inner.debug(*a, **k)
        except Exception:
            pass


class BambuSpoolman:
    """
    The Spoolman half of one Bambu AMS unit.

    One instance per unit, created by AFC_BambuAMS_RFID.for_unit() (or by
    measurement_only()). Holds the Spoolman and measurement state, and
    reaches back through ``self._u`` for the unit's own slots, lanes and
    logger.

    :param unit: the afcBambuAMS this instance serves
    """

    #: ONE Spoolman worker thread for every unit, not one per unit: the jobs
    #: are HTTP calls that must not run on the reactor, and they serialize
    #: fine. Class-level so a second unit reuses the first one's thread.
    _spool_q: Any = None
    _spool_t: Any = None
    #: False on a measurement-only delegate: nothing reaches Spoolman. Class
    #: default True, so an object built without __init__ is a Spoolman
    #: delegate.
    spoolman_on: bool = True

    def __init__(self, unit: Any, spoolman: bool = True) -> None:
        """
        Per-unit Spoolman state and helpers for one AFC_BambuAMS unit.

        :param unit: the AFC_BambuAMS unit object
        :param spoolman: False for a measurement-only delegate, which keeps
          the measurement state and never talks to Spoolman
        """
        self._u = unit
        self.spoolman_on = bool(spoolman)
        #: Slots whose binding has already been re-checked against the tag.
        self._binding_check: Dict[int, str] = {}
        #: slot -> the tag UID its Spoolman binding came from. A slot with no
        #: entry was bound some other way (manual, restored) and is left alone.
        self._bound_uid: Dict[int, str] = {}
        #: slot -> the AMS's own measured remaining percentage. The
        #: measurement's identity (see _adopt_measured_remain), not a figure
        #: anything re-applies.
        self._measured_remain: Dict[int, Any] = {}
        #: slot -> (pct, nominal, spool_id, sent): a measurement owed to the
        #: spool this bay's pending bind will attach (see
        #: _apply_remain_weight).
        self._bind_owed: Dict[int, Any] = {}
        #: slot -> (pct, nominal, grams): a measurement written as tag-linear
        #: grams because nothing named the bay's material yet, to be turned
        #: into grams through the density once something does.
        self._convert_owed: Dict[int, Any] = {}
        #: UIDs with a Spoolman lookup in flight, so two scans of the same
        #: bay do not both hit the server.
        self._spoolman_inflight: Set[str] = set()
        #: Bays whose Spoolman binding has been asked for and has
        #: not answered yet. Cleared by the completion callback,
        #: never by a clock.
        self._bind_pending: Set[int] = set()
        #: UIDs Spoolman has already said it does not know.
        self._spoolman_no_match: Set[str] = set()
        #: Operator summaries held until the bay's record can answer them.
        self._pending_summary: Dict[int, Any] = {}

    def _forget_spoolman_miss(self, slot: int) -> None:
        """
        Drop the "Spoolman has no spool for this UID" memo for a slot.

        Called when a spool leaves the bay: the next one deserves its own
        lookup, and the same spool re-inserted after being added to Spoolman
        must be able to bind.

        :param slot: 0-based AMS slot index
        """
        try:
            miss = getattr(self, "_spoolman_no_match", None)
            if not miss:
                return
            # The whole set, not this slot's UID: the slot's UID may already be
            # gone (unit power-cycled). Costs at most one re-lookup per
            # unmatched bay.
            miss.clear()
        except Exception:
            pass
        # The binding-contradiction memo is kept: scan motion can flap
        # presence, and each edge would re-probe Spoolman. It resets on
        # disconnect.

    def _spoolman_slot_info(self, info: dict) -> dict:
        """
        Translate a bridge slot dict into the shape sync_rfid_to_spoolman and
        find_spool_by_uid expect (the same dict every other AFC reader builds).

        The match key is "uid" (the 4-byte Mifare chip UID), so a spool
        registered on ANY reader on this printer (OpenAMS, ACE2, U1) matches
        here, and vice versa. tray_uid rides along for richness.

        :param info: normalized bridge slot info
        :return dict: Spoolman-shaped slot_info
        """
        material, sub_type = _split_bambu_material(info.get("material") or "")
        color = info.get("color")
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

        The restart-proof half of the stale-binding check: `_bound_uid` is our
        own memory and starts empty every boot, but Spoolman's record of which
        UIDs a spool carries outlives any restart.

        Deliberately asymmetric: unbinding a hand-bound lane loses the
        operator's work, while a stale binding only shows the wrong spool
        until the next insert. So this returns True only on positive proof: the spool
        carries UIDs and this bay's tag is not among them. No record, no UIDs
        on it, no Spoolman, or a failed lookup all return False and the binding
        stands.

        Memoized on (spool_id, uid): the caller runs on every status pass,
        and this lookup is a blocking HTTP call on the reactor, which at 1 Hz
        starves MCU clock sync. Self-invalidating: a different spool or tag is
        a different key.

        :param spool_id: the Spoolman id the lane is currently bound to
        :param uid: the tag UID actually present in the bay
        :return bool: True only if Spoolman positively contradicts the binding
        """
        if not spool_id or not uid or _spool_uids is None or _norm_uid is None:
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
                # No UIDs recorded => nothing to contradict. That is the
                # hand-assigned spool, and it keeps its lane.
                if known:
                    verdict = _norm_uid(uid) not in known
        except Exception:
            # Never unbind or memoize on a lookup failure: a transient outage
            # would mask a stale binding for the session.
            return False
        memo[key] = verdict
        return verdict

    def _remember_bound_uid(self, slot: Optional[int], uid: str) -> None:
        """
        Record that this slot's Spoolman binding was made from tag ``uid``.

        Best-effort: a diagnostic bookkeeping entry must never be able to break
        a bind that already succeeded.

        :param slot: 0-based AMS slot index, or None when unknown
        :param uid: the tag UID the binding was made from
        """
        if slot is None or not uid:
            return
        try:
            if getattr(self, "_bound_uid", None) is None:
                self._bound_uid = {}
            self._bound_uid[slot] = uid
        except Exception:
            pass

    #: Run Spoolman HTTP on a worker thread so blocking calls do not starve
    #: clock sync; only the lane mutation returns via register_async_callback.
    SPOOLMAN_BG = True

    def _spoolman_bg(self, job: Callable[[], None]) -> None:
        """
        Run ``job`` on the shared Spoolman worker thread.

        Inline when SPOOLMAN_BG is absent or false.

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

            t = threading.Thread(target=_drain, daemon=True,
                                 name="afc_bambu_spool")
            BambuSpoolman._spool_t = t
            t.start()
        q.put(job)

    def _bind_by_uid_bg(self, lane: Any, slot: Optional[int], uid: str,
                        note: str, tray_uid: str = "",
                        restored: bool = False) -> None:
        """
        Match this tag against Spoolman off-reactor; bind the lane on-reactor.

        The worker does the HTTP (the lookups are full-table scans, and the UID
        stamp another two calls); the reactor callback does the ONE thing that
        touches Klipper state (set_spoolID) plus the memos. An in-flight set
        stops the status pass from enqueuing the same UID again while its
        lookup is still running.

        The roll is looked up before the tag. A Bambu spool has two tags, one
        per flange, with different chip UIDs and the same tray UID; keyed on
        the chip alone, one reel ends up as two Spoolman records. So the tray
        UID is tried first and the chip UID is the fallback, and whichever
        answers teaches the record the other: a tray-UID match appends this
        chip UID to card_uids, a chip-UID match stamps the tray UID on. The
        reel then matches whichever way round it goes in.

        Bambu only: other readers key off the chip UID alone.

        :param lane: the AFC lane to bind
        :param slot: 0-based AMS slot index, or None when unknown
        :param uid: the tag UID to look up
        :param note: where the read came from, for the log line
        :param tray_uid: the tag's 16-byte roll identity, when it carries one
        :param restored: the unit's lookup for a lane AFC restored with no
          spool (afcBambuAMS._lookup_unbound), not a read of the bay. Two
          answers then differ: a spool with no remaining weight on record is
          not bound (see _spool_weighed), and a lookup Spoolman did not answer
          is no miss; the bay is asked again after LOOKUP_RETRY_S.
        """
        if not uid or find_spool_by_uid is None:
            return
        # Normalized once so the lookup, conflict check and log line agree.
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
            # An empty answer is not a miss while Spoolman or Moonraker is
            # starting; for a restored lane, check whether the server answers
            # at all.
            down = False
            try:
                client = _bambu_spoolman_client(afc)
                if client is None:
                    down = restored
                if client is not None:
                    # The shared matcher (also used by _spoolman_resolve), so
                    # every reader resolves a tag to a spool the same way.
                    spool, matched_by_tray, dupe_id = match_spool_for_tag(
                        client, uid, tray_uid)
                    by_tray = spool if matched_by_tray else None
                    if restored and spool is None:
                        reach = getattr(client, "reachable", None)
                        down = callable(reach) and not reach()
                    if dupe_id is not None and spool is not None:
                        # Said while the reel is in the bay: Spoolman holds it
                        # twice, and its usage splits between the records.
                        self._u.logger.info(
                            f"AFC bambu {self._u.name}: Spoolman spool "
                            f"{dupe_id} is the SAME PHYSICAL REEL as "
                            f"{spool.get('id')} -- this spool has a tag on "
                            f"each side and was recorded twice. Merge them "
                            f"(keep one, add the other's card_uids and their "
                            f"two used weights) or its count stays split.")
                if spool is not None and client is not None:
                    # Teach the record the other half of its identity: union
                    # the chip UID into card_uids, or stamp the tray UID only
                    # when the record has none (a different one is logged, not
                    # overwritten).
                    try:
                        if by_tray is None and tray_uid:
                            have = _spool_tray_uid(spool)
                            if not have:
                                client.write_tray_uid(spool.get("id"),
                                                      tray_uid)
                            elif have != tray_uid:
                                self._u.logger.debug(
                                    f"AFC bambu {self._u.name}: spool "
                                    f"{spool.get('id')} carries tray UID "
                                    f"{have} but this tag says {tray_uid} -- "
                                    f"left alone; one of them is on the wrong "
                                    f"record")
                    except Exception:
                        pass
                    # Warm AFC's spool cache here so set_spoolID below need not
                    # block on its own get_spool. Best-effort.
                    try:
                        mr = getattr(afc, "moonraker", None)
                        if mr is not None and hasattr(mr, "get_spool"):
                            # get_spool is fire-and-forget; warming the cache
                            # is the point.
                            mr.get_spool(spool.get("id"), lambda _sp: None)
                    except Exception:
                        pass
                    # Union this chip UID into the spool's card_uids while
                    # off-reactor, so the next read matches directly.
                    try:
                        client.write_spool_metadata(spool.get("id"), uid=uid)
                    except Exception:
                        pass
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
                        # Answered: no retry of it is pending any more.
                        (getattr(self._u, "_lookup_retry", None)
                         or {}).pop(slot, None)
                    if down:
                        BambuSpoolman._lookup_unanswered(self, lane, slot, uid)
                    elif (restored
                          and spool is not None
                          and not BambuSpoolman._spool_weighed(spool, afc)):
                        BambuSpoolman._lookup_refuse(self, lane, slot, uid,
                                                     spool.get("id"))
                    elif (spool is not None
                          and afc is not None
                          and getattr(afc, "spool", None) is not None
                          and getattr(lane, "spool_id", None)
                          in (None, "", 0)
                          and not BambuSpoolman._tag_left(self, slot, uid)):
                        # AFC's set_spoolID binds when its fetch answers
                        # (on_done). Until then the bind stays pending
                        # (_bind_landed), holding the measurement and summary.
                        if (_set_spoolid_takes_on_done is not None
                            and _set_spoolid_takes_on_done(afc)):
                            afc.spool.set_spoolID(
                                lane, spool.get("id"),
                                on_done=lambda: BambuSpoolman._bind_landed(
                                    self, lane, slot, uid))
                            landing = True
                        else:
                            afc.spool.set_spoolID(lane, spool.get("id"))
                            self._remember_bound_uid(slot, uid)
                        # DEBUG: the summary this callback goes on to speak
                        # already ends with "updated Spoolman spool N".
                        self._u.logger.debug(
                            f"AFC bambu {self._u.name}: matched {lane.name} to "
                            f"Spoolman spool {spool.get('id')} by "
                            + ("tray UID " + tray_uid
                               if by_tray is not None else "UID " + str(uid))
                            + str(note))
                    elif spool is None:
                        miss = getattr(self, "_spoolman_no_match", None)
                        if miss is None:
                            miss = self._spoolman_no_match = set()
                        miss.add(uid)
                        # No spool, so no bind: nothing is owed a measurement
                        # a later hand-bind should not inherit.
                        (getattr(self, "_bind_owed", None) or {}).pop(
                            slot, None)
                except Exception:
                    pass
                finally:
                    # Drop the in-flight mark only after the bind, then drain
                    # the summary, unless AFC is still fetching the spool
                    # (_bind_landed drops it then).
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

        AFC's set_spoolID loads the spool's remaining_weight onto the lane and,
        unless disable_weight_check is set, CLEARS the lane when that is
        missing or not above zero (AFC_spool._apply_spool_data): material,
        colour and weight, all of it. Spoolman leaves the key out for a spool
        it cannot weigh.

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

    def _lookup_unanswered(self, lane: Any, slot: Optional[int],
                           uid: str) -> None:
        """
        A restored lane's lookup that Spoolman did not answer: ask again later.

        Not recorded as a miss: nothing said the spool is unknown. The bay's
        one-shot is re-armed and _lookup_unbound asks again once
        LOOKUP_RETRY_S has passed, until Spoolman answers, the lane is bound,
        the bay empties or the connection changes. A measurement already owed
        to this bind stays owed to the one that follows.

        :param lane: the lane the lookup was for
        :param slot: 0-based AMS slot index
        :param uid: the tag UID that was looked up
        """
        if slot is None or not BambuSpoolman._bay_carries(self, slot, uid):
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
        u.logger.debug(
            f"AFC bambu {u.name}: Spoolman did not answer the lookup of "
            f"{str(uid).upper()} for {getattr(lane, 'name', lane)}; asking "
            f"again in {wait:.0f} s")

    def _lookup_refuse(self, lane: Any, slot: Optional[int], uid: str,
                       sid: Any) -> None:
        """
        A restored lane's match that AFC would clear the lane for: not bound.

        The lane stays as AFC restored it, nothing is owed a measurement, and
        the operator is told once which spool to correct. The bay stays
        latched, so it is not asked again until something reads it
        (AFC_BAMBU_SCAN) or the spool comes out.

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
        u.logger.info(
            f"AFC bambu {u.name}: {name} -- tag {str(uid).upper()} matches "
            f"Spoolman spool {sid}, which has no remaining weight on record, "
            f"so the lane was not linked to it (AFC clears a lane linked to a "
            f"spool without one). Correct spool {sid}'s weight in Spoolman, "
            f"then run AFC_BAMBU_SCAN LANE={name} to link it")

    def _bay_carries(self, slot: Optional[int], uid: str) -> bool:
        """
        Whether the bay still holds the tag a lookup was sent about.

        The answer lands on the reactor well after the question: with
        Spoolman not answering, up to twenty seconds later (a search and a
        reachability probe, ten each), and the bay may have emptied or
        taken another reel since. "Ask again later" or "not bound for want of
        a weight" is then about a spool that has left: acted on, it would
        re-arm a retry for an empty bay, or drop the latch and the owed
        measurement of the reel that went in after it.

        :param slot: bay index
        :param uid: the tag UID the lookup was for
        :return bool: True when the bay's record is present with that UID
          (or there is no record to ask, on a stand-in)
        """
        try:
            slots = getattr(self._u, "_slots", None)
            if not isinstance(slots, list) or not 0 <= slot < len(slots):
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

        Stricter than ``not _bay_carries``: an occupied bay whose record
        names no tag has not shown that its reel left, unless this
        connection saw the bay emptied since. A bridge that has just
        restarted publishes every occupied bay that way, and an AMS 1 does
        not hand its tag back unasked, so a bind landing then is about the
        reel still in the bay; after a restart the host only asks the bridge
        whether the bay is occupied.

        :param slot: bay index
        :param uid: the tag UID the bind was made from
        :return bool: True when the bay is empty, holds another tag, or was
          seen emptied and has not been read since
        """
        try:
            slots = getattr(self._u, "_slots", None)
            if (slot is None
                or not isinstance(slots, list)
                or not 0 <= slot < len(slots)):
                return False
            rec = slots[slot] or {}
            if not rec:
                return False                 # nothing known about the bay
            if not rec.get("present"):
                return True
            ruid = str(rec.get("rfid_uid") or "").lower()
            if ruid:
                return ruid != str(uid or "").lower()
            return slot in (getattr(self._u, "_removed_bays", None) or ())
        except Exception:
            return False

    def _bind_landed(self, lane: Any, slot: Optional[int], uid: str,
                     miss: bool = False) -> None:
        """
        A bind this module sent has run its course: settle it, then say the
        summary that was waiting on it.

        Called once AFC has put the spool on the lane or given up (its
        set_spoolID fetches the spool from Spoolman first and fires on_done
        when that answers), so the lane itself says what the answer was:

        * the bay no longer holds the tag: the reel left while Spoolman
          answered, the removal edge cleared and unbound the lane, and the
          answer bound it again. A link to a spool that is not in the bay is
          stale by definition, so the lane is cleared and unbound as that edge
          left it, and put back on lane defaults when a reel nothing has
          read went in meanwhile (afcBambuAMS._defaults_until_read);
        * the lane is bound: the tag it was bound by is remembered;
        * the lane is not bound: no spool took the bind, so a measurement
          held for it is released (a spool linked later by hand is the
          operator's choice, see _settle_bind_owed). ``miss`` also records
          the UID as unknown to Spoolman, for the create path, whose helper
          does not say why nothing was bound.

        Never raises: it runs in a reactor callback.

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
                    u._unbind_spool(
                        lane, f"its Spoolman link for tag {str(uid).upper()} "
                              f"landed after that spool left the bay")
                    # A reel nothing has read goes back on lane defaults; one
                    # still settling (_defaults_due) gets them, and the line,
                    # once it has.
                    rec = u._slots[slot] or {}
                    dflt = getattr(u, "_defaults_until_read", None)
                    if (rec.get("present")
                        and callable(dflt)
                        and slot not in (getattr(u, "_defaults_due", None)
                                         or {})):
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

        It does for a bay scanned on this connection, and for one whose lane
        a removal edge cleared; the boot hold does not apply to it
        (afcBambuAMS._boot_hold). Anything else is held until a scan.

        :param slot: bay index
        :return bool: True when the surface path is not held for this bay
        """
        return any(slot in (getattr(self._u, name, None) or ())
                   for name in ("_scanned_bays", "_cleared_bays"))

    def _sync_owed(self, slot: Optional[int], info: dict) -> bool:
        """
        Whether this bay's Spoolman bind is still to be dispatched.

        ``_bind_pending`` covers the window between a bind being asked for and
        its answer, not the window before the ask. On a Bambu unit the
        measurement reaches ``_queue_spool_summary`` from narration, while the
        bind is dispatched later in the same scan from ``_surface_slot_info``,
        so without this the summary would report an unlinked lane about to be
        bound.

        ``_spoolman_latched`` is the unit's own record of that dispatch: added
        immediately before the ``_spoolman_sync`` call and discarded on the
        removal edge, so "carries a UID and is not latched" means a bind is
        still coming. A bay the unit has not scanned on this connection is NOT
        owed one (``_surface_slot_info`` returns before the dispatch under
        ``_boot_hold``), so holding there would buy a 45 s silence for a bind
        that was never going to happen, unless the unit will look its tag up
        anyway, for a restored lane with no spool (its ``_lookup_coming``). A
        bay whose lane a removal edge cleared is not held (see
        _surface_released), so it counts as a scanned one does.

        :param slot: bay index the measurement belongs to
        :param info: that bay's record from the unit
        :return bool: True while the bind is still to be dispatched
        """
        try:
            # No Spoolman here, so no bind will ever be dispatched for it.
            if not getattr(self, "spoolman_on", True):
                return False
            if slot is None or not info.get("rfid_uid"):
                return False
            afc = getattr(self._u, "afc", None)
            if afc is None or getattr(afc, "spoolman", None) is None:
                return False
            # getattr throughout: duck-typed stand-ins carry only what their
            # case needs, and one without the unit's memos returns False.
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

        Asked by _adopt_measured_remain, to decide whether a measurement is
        owed to a bind (``_bind_owed``). Wider than _sync_owed, which answers
        for a record that already carries its UID: an insert is often
        measured while its record is still blank, with the tag and the bind
        landing seconds later. Requiring the UID would owe that bind nothing,
        and it would load Spoolman's stored weight over the measured figure.

        So a bay scanned on this connection (or whose lane a removal edge
        cleared, see _surface_released) and not latched yet counts while its
        UID is on the record OR its scan is still open, and a bind in flight
        always counts, as does the lookup the unit makes for a restored lane
        with no spool (its ``_lookup_coming``).

        :param slot: bay index the measurement belongs to
        :param info: that bay's record from the unit
        :return bool: True while a bind for this bay is out or may still be
          sent
        """
        try:
            if not getattr(self, "spoolman_on", True) or slot is None:
                return False
            afc = getattr(self._u, "afc", None)
            if afc is None or getattr(afc, "spoolman", None) is None:
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
                if 0 <= slot < len(t0) and t0[slot] is not None:
                    return True
            coming = getattr(self._u, "_lookup_coming", None)
            return bool(coming(slot, info)) if callable(coming) else False
        except Exception:
            return False

    def _mark_bind_sent(self, slot: Optional[int]) -> None:
        """
        This module has just dispatched a Spoolman bind for this bay.

        A measurement owed to the bay's bind is from here owed to whatever
        spool THIS bind attaches. Until then the claim is only held: a spool
        bound some other way (by hand, say) is the operator's choice, and
        is not handed a measurement it was never waiting for.

        :param slot: the bay the bind is for, or None when unknown
        """
        owed = getattr(self, "_bind_owed", None)
        held = owed.get(slot) if owed and slot is not None else None
        if held is not None and not held[3]:
            owed[slot] = (held[0], held[1], held[2], True)

    def _spoolman_sync(self, lane: Any, info: dict,
                       restored: bool = False) -> None:
        """
        Bind this bay in Spoolman, then release the summary that was waiting.

        Every exit from the bind is an answer. The bind's own callbacks drain
        the held summary when a lookup runs, but most passes through
        ``_spoolman_sync_inner`` never start one: already bound by this tag, no
        Spoolman, a UID in the miss memo, an empty bay. Draining here keeps
        the ``_sync_owed`` hold from waiting out its backstop on those.

        The same exits answer a measurement waiting on this bay's bind: a
        pass that sent nothing means no bind of ours is coming for this scan,
        so a claim still unsent (``_bind_owed``) is dropped. One already sent
        is left to that bind's own answer.

        :param lane: the AFC lane
        :param info: normalized bridge slot info (must carry rfid_uid)
        :param restored: the unit's lookup for a lane AFC restored with no
          spool (see _spoolman_sync_inner)
        """
        slot = info.get("index")
        try:
            if restored:
                BambuSpoolman._spoolman_sync_inner(self, lane, info,
                                                   restored=True)
            else:
                BambuSpoolman._spoolman_sync_inner(self, lane, info)
        finally:
            try:
                owed = getattr(self, "_bind_owed", None)
                held = owed.get(slot) if owed and slot is not None else None
                if held is not None and not held[3]:
                    owed.pop(slot, None)
            except Exception:
                pass
            try:
                if (slot is not None
                    and slot not in (getattr(self, "_bind_pending", None)
                                     or ())):
                    BambuSpoolman._drain_spool_summary(self, slot)
            except Exception:
                pass

    def _spoolman_sync_inner(self, lane: Any, info: dict,
                             restored: bool = False) -> None:
        """
        Bind this lane's spool to Spoolman by tag UID, creating it if allowed.

        Two paths, and the UID is the key to both:
          - FULL decode (material known): sync_rfid_to_spoolman binds an
            existing spool by UID or, with auto-create on, makes a new
            filament+spool from the tag's own values.
          - UID-ONLY (a good UID but no usable profile: a foreign tag the AMS
            surfaced a chip UID for but could not decode): match by UID ALONE
            and bind if Spoolman already knows it. Never create from nothing.

        Silent no-op without Spoolman configured or without a UID. A lane that
        already carries a spool_id is left alone: a manual/prior binding wins.

        A lookup for a lane AFC restored (``restored``) is always MATCH-ONLY,
        whatever auto_spoolman_create says: for a UID Spoolman does not know,
        the create path makes a spool at the tag's nominal weight, which
        set_spoolID then loads over the restored grams. Nothing is created
        from a lane nothing has read.

        :param lane: the AFC lane
        :param info: normalized bridge slot info (must carry rfid_uid)
        :param restored: the unit's lookup for a restored lane with no spool
          (afcBambuAMS._lookup_unbound), not a read of the bay
        """
        uid = info.get("rfid_uid")
        if not uid or lane is None:
            return
        # An empty bay has no tag to bind: the unit keeps a bay's UID in its
        # record after the spool leaves, and that leftover UID would re-bind the
        # lane the removal just unbound.
        if not info.get("present"):
            return
        slot = info.get("index")
        bound = getattr(lane, "spool_id", None)
        if bound not in (None, "", 0):
            prev = (getattr(self, "_bound_uid", None) or {}).get(slot)
            if prev == uid:
                return           # already bound BY THIS TAG: nothing to do
            if prev is None:
                # The memo does not survive a restart but the binding does, so
                # ask Spoolman whether the bound spool's UIDs exclude this tag.
                # A spool with no recorded UID is left alone.
                checker = getattr(self, "_binding_contradicted", None)
                if checker is None or not checker(bound, uid):
                    return
            # A different tag is in the bay, so the binding names the previous
            # spool. Unbind through the unit, which owns the lane. After a
            # restart there is no remembered tag to name, so the reason says
            # what proved it instead: Spoolman's UIDs for the spool.
            if prev is None:
                why = (f"tag {str(uid).upper()} is in this bay now, and "
                       f"Spoolman does not list it on that spool")
            else:
                why = (f"tag {str(uid).upper()} is in this bay now, not "
                       f"{str(prev).upper()}")
            self._u._unbind_spool(lane, why)
        afc = getattr(self._u, "afc", None)
        if afc is None or getattr(afc, "spoolman", None) is None:
            return
        si = self._spoolman_slot_info(info)
        have_profile = bool(si.get("material"))
        # A UID Spoolman does not know is remembered, not retried: this runs
        # every status pass, and re-querying at 1 Hz starves MCU clock sync.
        # Keyed by UID and cleared on removal, so re-inserting re-checks.
        if uid:
            miss = getattr(self, "_spoolman_no_match", None)
            if miss is None:
                miss = self._spoolman_no_match = set()
            if uid in miss:
                return
        try:
            allow = False
            if get_auto_spoolman_create is not None:
                allow = get_auto_spoolman_create(
                    lane, getattr(self._u, "auto_spoolman_create", False))
            if (have_profile
                and allow
                and not restored
                and sync_rfid_to_spoolman is not None):
                # Create needs the full shared-helper flow. Passing the reactor
                # sends its HTTP off the reactor; the answer is read in
                # _bind_landed.
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
                # Match-only and off-reactor: find the spool by UID on the
                # worker, then set_spoolID from the reactor callback. The
                # in-flight set stops duplicate lookups.
                BambuSpoolman._bind_by_uid_bg(
                    self, lane, slot, uid,
                    "" if have_profile else " (no tag profile decoded)",
                    tray_uid=info.get("tray_uid") or "", restored=restored)
        except Exception:
            try:
                self._u.logger.debug("AFC bambu: spoolman sync failed",
                                  traceback=traceback.format_exc())
            except Exception:
                self._u.logger.debug("AFC bambu: spoolman sync failed")

    def _apply_remain_weight(self, lane: Any, info: dict) -> None:
        """
        Finish what a measurement still owes this bay, once.

        A measurement is applied once, when it is taken:
        _adopt_measured_remain writes the lane and a bound spool then.
        Re-applying it later would overwrite AFC's own consumption count and
        restored weights with a stale reading.

        Two things can be left over from that one application, and each is
        finished here exactly once:

        * The bind that had not landed (``_bind_owed``). A fresh insert is
          measured before its Spoolman lookup answers, and the bind then
          hydrates the lane with whatever Spoolman had stored. The spool it
          attaches is owed the measurement. See _settle_bind_owed.
        * The material that had not been named (``_convert_owed``). A percent
          landing while the bay's record is still blank has no density to be
          weighed at, so it is written as tag-linear grams. See
          _settle_convert_owed.

        Anything that means the follow-up is no longer the same spool's (the
        spool feeding the toolhead, the bay emptied or rescanned, a release)
        drops it; nothing here ever writes the measurement a second time.

        :param lane: the AFC lane
        :param info: normalized bridge slot info (index)
        """
        idx = info.get("index")
        owed = getattr(self, "_bind_owed", None) or {}
        conv = getattr(self, "_convert_owed", None) or {}
        if lane is None or (idx not in owed and idx not in conv):
            return
        # Extrusion owns the weight from here, and a spool that has been fed
        # no longer holds what was measured.
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

        The adoption left (pct, nominal, spool_id, sent) in ``_bind_owed``:
        the spool the lane was bound to then, and whether this module has
        dispatched the bind yet (_mark_bind_sent). The claim is spent by the
        first frame that finds the lane bound to a DIFFERENT spool through
        that bind: a fresh insert whose tag matched after the measurement,
        or a stale restored binding rebound to the spool actually in the bay.
        Dropped without a write when:

        * the lane was bound while nothing of ours had been sent: a spool
          assigned by hand is the operator's choice, not the bind's;
        * our bind attached the spool the lane already had, which the
          adoption wrote;
        * nothing is left that could deliver a bind (Spoolman off).

        The grams are made now, through the same _grams_for the adoption
        used: the bind is dispatched from the surfaced record, so the
        material is known by the time it lands even when it was not at the
        measurement.

        :param lane: the AFC lane
        :param idx: 0-based AMS slot index
        :param owed: the ``_bind_owed`` dict holding this bay's claim
        """
        pct, nominal, prior, sent = owed[idx]
        sid = getattr(lane, "spool_id", None)
        if sid in (None, "", 0):
            # Still coming, unless nothing is left that could deliver it.
            afc = getattr(self._u, "afc", None)
            if (not getattr(self, "spoolman_on", True)
                or afc is None
                or getattr(afc, "spoolman", None) is None):
                owed.pop(idx, None)
            return
        if sid == prior:
            # The spool the adoption wrote. Once our bind has answered with
            # it, nothing is owed; until then the lane is only still on it.
            if (sent
                and idx not in (getattr(self, "_bind_pending", None) or ())):
                owed.pop(idx, None)
            return
        owed.pop(idx, None)
        if not sent:
            return
        grams = BambuSpoolman._grams_now(self, idx, lane, pct, nominal)
        # The lane takes it either way: the bind just replaced the measurement
        # with Spoolman's stored figure. Whether Spoolman does is the push's
        # own call (sync_measured_to_spoolman).
        lane.weight = grams
        self._u.logger.debug(
            f"AFC bambu {self._u.name}: {getattr(lane, 'name', '?')} bound "
            f"to spool {sid} after its measurement; handing the measured "
            f"{grams} g to it")
        self._push_measured_to_spoolman(lane, grams, "physical AMS measurement",
                                        pct=pct, nominal=nominal)
        # Grams made without a density are still owed their material, against
        # the figure just written.
        conv = getattr(self, "_convert_owed", None)
        if conv is not None and idx in conv:
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

        The percent is a volume, and a blank record has no density. A removal
        blanks the lane and the tag often lands after the percent, so
        _grams_for falls back to tag-linear grams; read back later through the
        real density, remain_pct would then disagree with the measurement.

        So the conversion is finished on the first frame that names the
        material: the lane, the bound spool, the saved vars and a summary
        still waiting to be said all get the weighed figure. Only while the
        lane still holds exactly the grams written; anything else on it
        (extrusion, a bind, a hand) is newer than the measurement and is
        left alone, and the claim with it.

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
            return                      # still nothing names it
        conv.pop(idx, None)
        grams = BambuSpoolman._grams_now(self, idx, lane, pct, nominal)
        if grams == int(written):
            return
        lane.weight = grams
        self._u.logger.debug(
            f"AFC bambu {self._u.name}: {getattr(lane, 'name', '?')} -- "
            f"{pct}% weighed at its material now it is known: {grams} g, not "
            f"the {written} g written before the tag landed")
        self._push_measured_to_spoolman(lane, grams, "physical AMS measurement",
                                        pct=pct, nominal=nominal)
        # A summary still held for the record quotes the grams it was queued
        # with; it has to say the ones the lane now has.
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

        _grams_for, or the tag-linear figure it falls back to on an object
        that carries only part of the class (the duck-typed stand-ins).

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

    def _push_measured_to_spoolman(self, lane: Any, grams: int,
                                   source: str = "", pct: int = 0,
                                   nominal: int = 0) -> None:
        """
        Write a remaining-weight figure back to the lane's Spoolman spool.

        Two things produce that figure and they are not the same claim:

          measurement  the AMS physically pulled the spool and derived a radius
                       (P:NN% -> grams). A real reading, and the reason this
                       write exists.
          tag record   the percentage written on the tag in some previous life.
                       Better than nothing, but nothing was measured now.

        ``source`` records which, so the log cannot announce a tag record as a
        physical measurement: the machine asserting work it did not do.

        Applies to any lane bound to a Spoolman spool, tagged or not, which is
        also what stops a no-tag spool from looking full when it is not.

        Gated by sync_measured_to_spoolman (default on). No-op without Spoolman
        or a bound spool.

        :param lane: the AFC lane
        :param grams: remaining net weight, grams
        :param source: where the figure came from, for the log
        :param pct: the measured percent, for the log when grams were capped
        :param nominal: the spool's full weight in grams, for the same log
        """
        # The one HTTP path in the whole measurement closure, so the one place
        # a measurement-only delegate has to stop.
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
        if client is None or not callable(setter):
            return
        # Off the reactor: set_remaining_weight is two blocking HTTP calls.
        src = source or "physical AMS measurement"

        def _job(sid: Any = sid, grams: float = float(grams),
                 source: str = src, pct: int = int(pct or 0),
                 nominal: int = int(nominal or 0)) -> None:
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
                # When the grams were capped at nominal, print the measured
                # percent too: every reading over 100% writes the same figure,
                # which otherwise looks like lane defaults being written.
                _cap = (f" -- measured {pct}%, capped to the spool's "
                        f"{nominal} g nominal"
                        if pct > 100 and nominal else "")
                self._u.logger.info(
                    f"AFC bambu {self._u.name}: wrote {grams} g remaining to "
                    f"Spoolman spool {sid} ({source}){_cap}")
            except Exception as e:
                self._u.logger.debug(
                    f"AFC bambu {self._u.name}: Spoolman weight write failed: {e}")
        BambuSpoolman._spoolman_bg(self, _job)
        return

    def _queue_spool_summary(self, slot: int, pct: int, grams: int,
                             nominal: int) -> None:
        """
        Hold the operator's summary until the bay's record can answer it.

        The measurement finishes before the record catches up. The bridge
        firmware does not read 0x0211 during the capacity window (reading it
        mid-scan aborts the feed), so it clears ``info_valid`` at the window
        close and the round-robin fill collects the tag afterwards, while the
        measurement arrives from narration first. Said immediately, the
        summary would read the blank record and report "no tag".

        So: if the record already answers, or the UNIT says no tag read during
        this scan, say it now. Otherwise wait for the re-read; _sync_lanes
        drains this as soon as the record lands.

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
        # A miss judges one lookup, not the UID forever. A fresh measurement
        # voids it while the bay's lookup is still to go out; a latched bay
        # keeps it.
        try:
            uid0 = ""
            for sl in (self._u._slots or []):
                if sl.get("index") == slot:
                    uid0 = str(sl.get("rfid_uid") or "").lower()
                    break
            miss0 = getattr(self, "_spoolman_no_match", None)
            latched0 = slot in (getattr(self._u, "_spoolman_latched", None)
                                or ())
            if uid0 and miss0 and not latched0:
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
        # Wait for the record or the backstop; this only covers the poll
        # catching up, since an untagged spool is never measured.
        ready = bool(info.get("material") or info.get("rfid_uid"))
        # A UID is not an answer while the scan is still running: on an HT the
        # UID arrives well before the profile. While _scan_verdict says
        # "waiting", a bare UID settles nothing.
        if ready and not info.get("material"):
            try:
                if self._u._scan_verdict(slot) == "waiting":
                    ready = False
            except Exception:
                pass
        # A Spoolman lookup in flight (_spoolman_inflight, from the off-reactor
        # _bind_by_uid_bg) is likewise unsettled; the backstop still bounds the
        # hold.
        if ready:
            try:
                uid = str(info.get("rfid_uid") or "").lower()
                inflight = getattr(self, "_spoolman_inflight", None) or set()
                if uid and uid in {str(u).lower() for u in inflight}:
                    ready = False
            except Exception:
                pass
        # Wait for the bind's answer, not a clock. _bind_pending also holds the
        # summary for the shared AFC_RFID binder, which _spoolman_inflight
        # misses.
        if ready and slot in (getattr(self, "_bind_pending", None) or ()):
            ready = False
        # And a bind not yet dispatched (see _sync_owed): the gap between the
        # measurement landing and the bind going out. getattr because
        # duck-typed stand-ins may lack the helper.
        _owed = getattr(self, "_sync_owed", None)
        if ready and _owed is not None and _owed(slot, info):
            ready = False
        # Whether the record answered or the backstop expired; the summary
        # words the two differently (see `settled`).
        settled = ready
        if not ready:
            try:
                ready = (deadline is None
                         or self._u.afc.reactor.monotonic() >= deadline)
            except Exception:
                ready = True
        if not ready:
            return
        # Finish the density conversion first (_settle_convert_owed) so the
        # line quotes the lane's final grams.
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
        self._say_spool_summary(slot, lane, pct, grams, nominal,
                                settled=settled)

    def _lookup_open(self, slot: Optional[int], info: dict) -> bool:
        """
        Whether this bay's Spoolman lookup is out, owed, to be asked again, or
        sent with a measurement held for it and not bound yet: anything the
        summary may not answer for yet.

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
            if callable(owed) and owed(slot, info):
                return True
            # A retry counts only while the unit will still send it: a late
            # "no answer" can leave one behind for a bay the surface path has
            # since asked about and latched, and nothing will ask again.
            if slot in (getattr(self._u, "_lookup_retry", None) or {}):
                coming = getattr(self._u, "_lookup_coming", None)
                if not callable(coming) or coming(slot, info):
                    return True
            # A bind this module sent that has not bound the lane yet: a miss
            # or a refusal drops the claim, and the bind landing settles it.
            held = (getattr(self, "_bind_owed", None) or {}).get(slot)
            return bool(held and held[3])
        except Exception:
            return False

    def _lookup_refused_for(self, slot: Optional[int], uid: Any) -> Any:
        """
        The spool a restored lane's lookup matched and would not bind for
        want of a remaining weight, if that is this tag's answer.

        :param slot: bay index
        :param uid: the bay's tag UID
        :return Any: the Spoolman spool id, or None
        """
        try:
            ref = (getattr(self._u, "_lookup_refused", None) or {}).get(slot)
            if ref and str(ref[0]).lower() == str(uid or "").lower():
                return ref[1]
        except Exception:
            pass
        return None

    def _say_spool_summary(self, slot: int, lane: Any, pct: int,
                           grams: int, nominal: int,
                           settled: bool = True) -> None:
        """
        One plain-English line for the operator when a spool is measured.

        Says what was read, how much is left, and where it went.

        :param slot: bay index the measurement belongs to
        :param lane: the AFC lane, or None
        :param pct: measured remaining percent, RAW (may exceed 100)
        :param grams: remaining weight in grams, already capped
        :param nominal: the spool's full weight in grams
        :param settled: False when the backstop fired before the tag's profile
                        arrived, so a missing profile is not called a decode failure
        """
        info = {}
        for sl in (self._u._slots or []):
            if sl.get("index") == slot:
                info = sl or {}
                break
        where = getattr(lane, "name", None) or f"bay {slot}"
        # What the tag said, if anything. A no-tag spool is ordinary, so it is
        # stated, not warned about.
        material = info.get("material")
        colour = info.get("color")
        uid = info.get("rfid_uid")
        if material:
            what = f"{material}" + (f" ({colour})" if colour else "")
            # "tag read" is a claim about this scan; the record may hold a tag
            # from an earlier scan that this one merely kept. scan_res 1 is a
            # read, 2/3 mean no new read; absent means "tag read".
            _res = info.get("scan_res")
            if _res is None or _res == 1:
                read = f"tag read: {what}"
            else:
                read = f"tag on file: {what}"
            if uid:
                read += f" [tag {str(uid).upper()}]"
        elif uid and not settled:
            # The backstop fired before the profile arrived: that says nothing
            # about the tag, so it is not reported as a decode failure.
            read = (f"tag {str(uid).upper()} -- the profile had not arrived "
                    f"yet when this was reported, so the material is not in "
                    f"this line. It lands on the lane by itself a moment "
                    f"later; nothing needs doing")
        elif uid and not _profile_landed(info):
            # A tag without a decodable profile still has a UID the operator
            # can bind in Spoolman.
            read = (f"tag {str(uid).upper()} read but its profile could not be "
                    f"decoded (not a Bambu tag?)")
            # The advice only holds with the Spoolman module on: with it off
            # nothing matches a UID, and the same line goes on to say so.
            if getattr(self, "spoolman_on", True):
                read += (" -- bind that UID to a spool in Spoolman and it will "
                         "match from now on")
        elif uid:
            # The profile decoded; the material string is just not in this
            # snapshot yet. Say what is known (the sku) and nothing more.
            _sku = info.get("sku")
            read = (f"tag {str(uid).upper()} read"
                    + (f": {_sku}" if _sku else ""))
        else:
            read = "no tag on this spool"
        # A reading held down by _remain_floor_pct says so, or the figure
        # looks stuck beside a higher narrated percent.
        read_pct = None
        try:
            read_pct = (getattr(self, "_summary_read", None) or {}).get(slot)
        except Exception:
            read_pct = None
        held = (f" (the AMS read {read_pct}% this time against {pct}% before "
                f"it; filament does not grow, so the lower reading stands)"
                if read_pct is not None and int(read_pct) > int(pct) else "")
        if pct > 100:
            # Over 100% is a spool larger than the reference full radius (the
            # percent is a volume ratio), not extra filament. Say so in words.
            why = (held
                   or (f" (the AMS read {pct}%, meaning it measures a "
                       f"little larger than a reference full spool)"))
            amount = f"full -- roughly {grams} g of a {nominal} g spool{why}"
        else:
            amount = (f"about {pct}% left -- roughly {grams} g of a "
                      f"{nominal} g spool" + held)
        # Where the number went, including why Spoolman was not updated.
        sid = getattr(lane, "spool_id", None)
        if not getattr(self, "spoolman_on", True):
            # No claim about Spoolman: nothing here talked to it. A bound
            # lane's spool is named, since it now disagrees with the lane and
            # AFC re-reads the spool at the next restart.
            went = ("kept on the lane -- the Spoolman module "
                    "([AFC_BambuAMS_rfid]) is off"
                    + (f", so Spoolman spool {sid} was not updated"
                       if sid not in (None, "", 0) else ""))
        elif not getattr(self._u, "sync_measured_to_spoolman", True):
            went = "Spoolman sync is off, so this is kept on the lane only"
        elif sid in (None, "", 0):
            # Say why and the fix. The "not linked" prefix is a claim, so only
            # branches that established it use it.
            unlinked = "not linked to a Spoolman spool, so this is kept on the "
            if not uid:
                went = unlinked + "lane only"
            elif BambuSpoolman._lookup_open(self, slot, info):
                # Asked, or about to be, and not answered yet. Where the
                # measurement goes depends on whether it is held for that bind.
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
                # Asked and answered: _spoolman_no_match is written only when
                # a lookup came back empty.
                went = unlinked + (f"lane only -- Spoolman has no spool "
                                   f"carrying {str(uid).upper()}")
                if not getattr(self._u, "auto_spoolman_create", False):
                    went += (f" and auto-create is off for this unit (add "
                             f"'auto_spoolman_create: True' to "
                             f"[AFC_BambuAMS {self._u.name}], or bind that UID "
                             f"to an existing spool)")
            elif BambuSpoolman._lookup_refused_for(self, slot, uid) is not None:
                # Asked, matched, and not bound: the spool has no remaining
                # weight on record, and AFC clears a lane linked to it.
                rsid = BambuSpoolman._lookup_refused_for(self, slot, uid)
                went = unlinked + (
                    f"lane only -- {str(uid).upper()} matches Spoolman spool "
                    f"{rsid}, which has no remaining weight on record; "
                    f"correct spool {rsid}'s weight in Spoolman, then run "
                    f"AFC_BAMBU_SCAN LANE={where} to link it")
            elif slot in (getattr(self._u, "_spoolman_latched", None) or ()):
                # The lookup went out on this connection and nothing it found
                # is on the lane now (the link was cleared since, say).
                went = (f"{str(uid).upper()} was looked up in Spoolman on this "
                        f"connection and no spool is linked to the lane from "
                        f"it, so the measurement stays on the lane only -- run "
                        f"AFC_BAMBU_SCAN LANE={where} to look it up again")
            else:
                # Not asked: no lookup is pending, so claim nothing and say how
                # to ask.
                went = (f"no Spoolman lookup has been made for "
                        f"{str(uid).upper()}, so the measurement stays on the "
                        f"lane only -- run AFC_BAMBU_SCAN LANE={where} to "
                        f"link it")
        else:
            went = f"updated Spoolman spool {sid}"
        self._u.logger.info(
            f"{self._u.name} {where}: {read}. Measured {amount}; {went}.")

    #: Volume of filament at the unit's own 100% reference, in cm3. Derived
    #: from reels of known weight, not measured directly; _grams_for uses it
    #: to turn a percent into grams.
    CAP_REF_VOLUME_CM3 = 663.0

    #: The AMS's reference geometry: percent is cross-sectional area between a
    #: 47.5 mm hub and an 82.6 mm full radius. Re-deriving it from the measured
    #: radius matches the unit to within half a point.
    CAP_HUB_MM = 47.5
    CAP_FULL_MM = 82.6

    @staticmethod
    def _material_key(lane: Any, info: dict) -> str:
        """
        The most SPECIFIC name for what is on the reel, for a density lookup.

        A lane splits the tag's material in two (`material` holds "PLA" and
        `sub_type` holds "Matte"), and the variant is exactly the part that
        changes the density: a matte is PLA plus a filler heavier than the
        polymer, so `lane.material` alone would look up plain PLA.

        The bridge's own string is already the full "PLA Matte", so it is
        preferred; the lane's two halves are rejoined when it has none.

        :param lane: the AFC lane (may be None)
        :param info: the bay's normalized slot info
        :return str: e.g. "PLA Matte", or "" when nothing is known
        """
        full = (info or {}).get("material") or ""
        if len(full.split()) > 1:
            return full
        base = (getattr(lane, "material", None) or full or "")
        sub = getattr(lane, "sub_type", "") or ""
        # "PLA Basic" and "PLA" look up the same; the join only matters where
        # the variant has its own entry.
        return (f"{base} {sub}".strip()
                if (sub
                    and sub.lower() not in base.lower())
                else base)

    def _density_of(self, slot: Optional[int], lane: Any) -> Optional[float]:
        """
        The density the capacity model weighs this bay's filament at.

        The lane's own figure wins; otherwise the material table, looked up by
        the most specific name the bay carries (see _material_key). None when
        neither knows, which every caller reads as "do not guess".

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
        if not density and density_for_material is not None and material:
            density = density_for_material(material)
        return density or None

    def _grams_for(self, slot: int, lane: Any, pct: int, nominal: int) -> int:
        """
        Turn a measurement into grams: MASS, not the percent read as a mass.

        The AMS percent is a volume ratio (cross-sectional area against a
        fixed reference geometry) and says nothing about mass. Read as a
        mass fraction (nominal * pct/100) it over-reports for any filament
        denser than the reference. So:

            grams = density * CAP_REF_VOLUME_CM3 * pct/100

        using the unrounded radius from the last capacity measurement when it
        belongs to this percent.

        The tag's declared weight is still the ceiling. A reading proud of the
        reference geometry means a full reel sitting slightly large, not more
        filament than the spool was sold with.

        Falls back to tag-linear grams (nominal * pct/100) when no density is
        known, rather than guessing one.

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
            # Prefer the radius: the integer percent quantises the answer to
            # about 5 g, while the "odom save" line gives the radius to six
            # decimals. Same geometry, at the unit's own precision.
            pct_eff = float(pct)
            rec = None
            try:
                rec = self._u._bridge.last_cap_measure(
                    getattr(self._u, "dry_dev_addr", 0)) if self._u._bridge \
                    else None
            except Exception:
                rec = None
            if rec and rec.get("pct_raw") == pct and rec.get("save_radius_m"):
                r_mm = float(rec["save_radius_m"]) * 1000.0
                hub2 = self.CAP_HUB_MM ** 2
                span = self.CAP_FULL_MM ** 2 - hub2
                if r_mm > self.CAP_HUB_MM and span > 0:
                    pct_eff = 100.0 * (r_mm ** 2 - hub2) / span
            grams = int(round(density * self.CAP_REF_VOLUME_CM3
                              * pct_eff / 100.0))
            return max(1, min(grams, int(nominal)))
        except Exception:
            # Inventory must never depend on this arithmetic surviving.
            return tag_linear

    def _pct_for(self, slot: Optional[int], lane: Any, grams: Any,
                 nominal: Any) -> Optional[float]:
        """
        The inverse of _grams_for: the percent a lane's grams stand for.

        What a bay reports as its remaining percent is the LANE's figure, the
        one AFC restored or extrusion has been counting down, read back through
        the same model that turned a measurement into grams, so a fresh
        measurement reads its own percent back, and a spool printed from reads
        lower, without anything being re-measured or re-applied.

            pct = grams / (density * CAP_REF_VOLUME_CM3) * 100

        Unknown density falls back to grams against the tag's nominal, the
        inverse of the tag-linear figure _grams_for falls back to.

        :param slot: 0-based AMS slot index
        :param lane: the AFC lane (its density wins over the table)
        :param grams: the lane's remaining grams
        :param nominal: the tag's declared full weight in grams
        :return Optional[float]: the percent, unrounded and uncapped, or None
          when there are no grams or no nominal to read it against
        """
        try:
            g = float(grams or 0)
            n = float(nominal or 0)
        except (TypeError, ValueError):
            return None
        if g <= 0 or n <= 0:
            return None
        try:
            density = self._density_of(slot, lane)
        except Exception:
            density = None
        if not density:
            return g * 100.0 / n
        return g * 100.0 / (float(density) * self.CAP_REF_VOLUME_CM3)

    def _log_capacity_sample(self, slot: int, lane: Any, pct: int,
                             grams: int, nominal: int,
                             source: str = "") -> None:
        """
        Record one capacity measurement, raw, for fitting grams from radius.

        Logs one debug "capsample" row per measurement: the raw percent, the
        radius that belongs to it, the density, the grams written and the
        grams the model (density * CAP_REF_VOLUME_CM3 * pct/100) gives, so
        the capacity model can be checked against what each measurement said.
        Changes nothing.

        Mine it with:
            grep 'capsample' printer_data/logs/AFC.log

        :param slot: 0-based AMS slot index the measurement belongs to
        :param lane: the AFC lane, for density/spool identity (may be None)
        :param pct: the RAW measured percent, uncapped
        :param grams: the grams actually written, i.e. after the 100% cap
        :param nominal: the tag's declared full weight in grams
        :param source: what produced the measurement, as _adopt_measured_remain
          names it; a live capscan and the firmware's own stamp of a cycle
          the narration missed are different rows, and a fit should be able
          to tell them apart
        """
        try:
            info = {}
            for sl in (self._u._slots or []):
                if sl.get("index") == slot:
                    info = sl
                    break
            material = self._material_key(lane, info)
            # The lane's density is Spoolman's, which is the filament's own
            # figure; the table is the fallback for an unbound lane.
            density = getattr(lane, "density", None)
            if not density and density_for_material is not None:
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
                        # The "odom save" line's radius, unrounded: at
                        # R~80 mm one millimetre is about 2% of the reel, more
                        # than the unit's run-to-run spread.
                        save_r = rec.get("save_radius_m")
                        radius_src = "narration"
                    else:
                        radius_src = f"stale(rec={rec.get('pct_raw')})"
                if radius_m is None:
                    # Same identity test, against the slot record's own pair.
                    # A boxed unit can advance meas_seq while leaving meas_pct
                    # stale, so matching the percent keeps a stale radius out.
                    r_mm = info.get("meas_radius_mm")
                    if r_mm and info.get("meas_pct") == pct:
                        radius_m = r_mm / 1000.0
                        radius_src = "slotrec"
            except Exception:
                pass
            modelled = None
            if density and pct:
                modelled = density * self.CAP_REF_VOLUME_CM3 * (pct / 100.0)
            self._u.logger.debug(
                "capsample "
                f"unit={self._u.name} slot={slot} "
                f"lane={getattr(lane, 'name', '?')} "
                f"spool={getattr(lane, 'spool_id', None)} "
                f"material={material!r} density={density} "
                f"pct_raw={pct} radius_m={radius_m} circ_m={circ_m} "
                f"save_r_m={save_r} "
                f"radius_src={radius_src} source={source!r} "
                f"nominal={nominal} grams_written={grams} "
                f"grams_modelled={f'{modelled:.0f}' if modelled else None}")
        except Exception:
            # Instrumentation must never be able to break a measurement.
            pass

    def _remain_floor_pct(self, uid: str, pct: int) -> int:
        """
        The lowest remain% this reel has measured, which is the honest one.

        The AMS derives its percent from a circumference measured over two
        spool revolutions, and repeat readings of an untouched reel spread by
        about +/-3%. Left alone, that noise moves the inventory figure up and
        down at every reseat.

        So use the one thing known for certain about a reel: what is on it
        never increases while it stays the same reel. A reading above this
        reel's previous one is noise, because the alternative is filament
        appearing; a reading below it is either consumption or noise, and
        taking it is the conservative direction. The stored figure therefore
        only ever comes down, and it settles near the bottom of the noise
        band rather than wandering across it, understating by about a
        percent, which is the side to be wrong on when the question is
        whether a print will finish.

        Keyed by CHIP UID, not by bay: the tag is the reel. A Bambu reel
        carries a tag on each flange with different chip UIDs, so a flipped
        reel measures afresh rather than inheriting the other face's floor,
        and a different spool in the same bay is a different key. The memo
        lives for the life of the process; a Klipper restart is the way
        back to a clean reading if one is ever needed.

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
        Remember this reel's lowest measurement. See :meth:`_remain_floor_pct`.

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
            if prev is None or int(pct) < int(prev):
                floor[key] = int(pct)
        except Exception:
            pass

    def _adopt_measured_remain(self, slot: int, pct: int,
                               source: str = "capscan",
                               seq: Optional[int] = None) -> bool:
        """
        Record a physical spool measurement: slot remain%, lane grams, Spoolman.

        The single place a measured percent becomes state, so the capacity scan
        and a load-time reading cannot drift apart in how they round, cap or
        report it.

        :param slot: slot index the measurement belongs to
        :param pct: remaining percent, RAW (may legitimately exceed 100)
        :param source: what produced it, for the log line
        :param seq: the firmware's meas_seq for this measurement, when the
          caller has one. It is the measurement's IDENTITY, and it is what
          makes "is this new?" answerable at all.
        :return bool: True if the measurement was accepted
        """
        # Sanity range. Grams are capped at nominal regardless; this ceiling only
        # decides whether an over-reference reading is kept (a full ABS reel can
        # read over 150%).
        if not (0 < pct <= 200):
            return False
        # Persist module-side: self._u._slots is REPLACED by every bridge status
        # frame, so an in-place edit survives one pass at most. The dict is the
        # measurement's identity; what it applies, it applies here, once.
        if not hasattr(self, "_measured_remain"):
            self._measured_remain = {}
        # Dedupe on the measurement's identity (meas_seq), not its value. A
        # caller without a sequence (the narration scrape) may set the first
        # reading for a slot but never overwrite one.
        if not hasattr(self, "_meas_seq_seen"):
            self._meas_seq_seen = {}
        if seq is not None:
            if self._meas_seq_seen.get(slot) == seq:
                return True                      # same cycle, already adopted
            self._meas_seq_seen[slot] = seq
            # Same number under a new sequence is the other path catching up,
            # not a new measurement.
            if self._measured_remain.get(slot) == pct:
                return True
        elif self._measured_remain.get(slot) is not None:
            # Unsequenced, and this slot already has a measurement. Refuse
            # rather than overwrite: there is no way to tell a fresh scrape
            # from a stale one, and guessing wrong rewrites inventory.
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
        # _measured_remain keeps the raw reading; what is reported and stored
        # is the reel's floor (see _remain_floor_pct).
        pct_read = pct
        _floor = getattr(self, "_remain_floor_pct", None)
        if _floor is not None:
            pct = _floor(uid, pct)
        _record = getattr(self, "_record_remain_floor", None)
        if _record is not None:
            _record(uid, pct)
        if pct != pct_read:
            self._u.logger.debug(
                f"AFC bambu {self._u.name}: slot {slot} measured {pct_read}%, "
                f"held at the {pct}% this reel measured before -- filament "
                f"does not grow, and the odometer is worth about +/-3%")
        lane = self._u._lane_for_slot(slot)
        # Grams are capped at the tag's nominal weight: over 100% is a full
        # spool proud of the reference geometry, not extra filament.
        try:
            (getattr(self._u, "_fresh_insert", None) or {}).pop(slot, None)
        except Exception:
            pass
        # Through _grams_now, whose tag-linear fallback is there for the same
        # reason _log_capacity_sample below is called by getattr: duck-typed
        # stand-ins reach this function without the rest of the class.
        grams = BambuSpoolman._grams_now(self, slot, lane, pct, nominal)
        _sample = getattr(self, "_log_capacity_sample", None)
        if _sample is not None:
            _sample(slot, lane, pct, grams, nominal, source)
        if lane is not None:
            lane.weight = grams
        # The raw reading, for the operator line: a figure that does not move
        # when the unit plainly read something different needs to say why.
        try:
            seen = getattr(self, "_summary_read", None)
            if seen is None:
                seen = self._summary_read = {}
            seen[slot] = pct_read
        except Exception:
            pass
        self._queue_spool_summary(slot, pct, grams, nominal)
        # Push the physical measurement to a bound Spoolman spool, correcting
        # its remaining_weight to what the AMS measured. Covers a manually
        # bound no-tag spool too. Gated by sync_measured_to_spoolman.
        self._push_measured_to_spoolman(lane, grams, pct=pct, nominal=nominal)
        # Owed to a bind that has not landed: the spool it attaches gets this
        # figure once (see _settle_bind_owed). A bay with no bind coming is
        # owed nothing.
        owed = getattr(self, "_bind_owed", None)
        if owed is not None:
            owed.pop(slot, None)
            try:
                info = next((sl for sl in (self._u._slots or [])
                             if sl.get("index") == slot), {})
                if (lane is not None
                    and BambuSpoolman._bind_coming(self, slot, info)):
                    sid = getattr(lane, "spool_id", None)
                    owed[slot] = (
                        pct, nominal, None if sid in (None, "", 0) else sid,
                        slot in (getattr(self, "_bind_pending", None) or ()))
            except Exception:
                pass
        # Owed its material. Grams made with no density to weigh them at are
        # the tag-linear figure; the conversion is finished once something
        # names the material (see _settle_convert_owed).
        conv = getattr(self, "_convert_owed", None)
        if conv is not None:
            conv.pop(slot, None)
            if (lane is not None
                and not BambuSpoolman._density_known(self, slot, lane)):
                conv[slot] = (pct, nominal, grams)
        # Persist it: an unbound lane has no Spoolman re-hydration, so without
        # this the measurement would not survive a restart.
        self._u._save_lane_vars()
        return True


class AFC_BambuAMS_RFID:
    """
    The [AFC_BambuAMS_rfid] Klipper object.

    Adding the section is what turns Spoolman on. The per-unit knobs
    (auto_spoolman_create, the remaining-weight write-back) stay on each
    [AFC_BambuAMS ...] section where they already live, so enabling this
    changes no existing config.

    Options
    -------
    enabled: True
        Set False to keep the section in place with Spoolman switched off:
        the units then behave exactly as they do with no section at all.

    :param config: the Klipper config wrapper for the section
    """

    #: Class defaults so an object that skipped __init__ still works.
    enabled: bool = True
    _units: set = frozenset()
    #: Whether the missing-AFC_RFID warning has been given (once per section,
    #: not once per unit asking).
    _warned_no_afc_rfid: bool = False

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Read the section; per-unit helpers attach lazily.

        :param config: the [AFC_BambuAMS_rfid] section
        """
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Read at least one option, or Klipper rejects a mixed-case section
        # that reads nothing as "not a valid config section".
        self.enabled = config.getboolean("enabled", True)
        #: Names of the units that have asked for a delegate, so the status
        #: shows what is actually wired rather than just "configured".
        self._units: set = set()
    def get_status(self, eventtime: Optional[float] = None) -> dict:
        """
        Status for the API, and the only way to SEE this object.

        Klipper's objects/list webhook filters to objects that implement
        get_status, so without this the loaded section would not appear in
        `/printer/objects/list`.

        :param eventtime: reactor time (unused)
        :return dict: enabled flag and the units currently delegating to us
        """
        return {"enabled": bool(self.enabled),
                "units": sorted(self._units)}

    def for_unit(self, unit: Any) -> Optional[BambuSpoolman]:
        """
        Build the Spoolman delegate for one AMS unit.

        :param unit: the afcBambuAMS asking for its delegate
        :return Optional[BambuSpoolman]: the delegate the unit's shims call
            through, or None when the section is present but disabled, or
            when AFC_RFID could not be imported
        """
        if not self.enabled:
            return None
        if _AFC_RFID_ERR is not None:
            # No AFC_RFID means no Spoolman delegate; say why once. The unit
            # still measures.
            if not self._warned_no_afc_rfid:
                self._warned_no_afc_rfid = True
                try:
                    unit.logger.warning(
                        f"AFC_BambuAMS_rfid: Spoolman is off for every AMS "
                        f"unit -- AFC_RFID could not be loaded "
                        f"({_AFC_RFID_ERR}). Measurements are still kept on "
                        f"the lanes.")
                except Exception:
                    pass
            return None
        try:
            self._units.add(str(getattr(unit, "name", unit)))
        except Exception:
            pass
        return BambuSpoolman(unit)


def measurement_only(unit: Any) -> BambuSpoolman:
    """
    The delegate for a unit that no Spoolman section serves.

    A BambuSpoolman built with Spoolman off: it keeps the measurement state
    and applies a measured percent to the slot, the lane's grams and the saved
    vars, and every path to Spoolman returns before it starts. Built by the
    unit itself (afcBambuAMS._measure), not by the section object, so it is
    never listed among the units Spoolman serves.

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
