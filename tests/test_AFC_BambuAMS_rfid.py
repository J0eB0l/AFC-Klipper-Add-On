"""Unit tests for extras/AFC_BambuAMS_rfid.py."""

from __future__ import annotations

import contextlib
import importlib
import sys
import threading
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

import pytest

from extras import AFC_BambuAMS_rfid as rfid_mod
import extras
from extras.AFC_BambuAMS import afcBambuAMS
from extras.AFC_BambuAMS_rfid import _QuietInfo, AFC_BambuAMS_RFID, BambuSpoolman
from tests.bambu_helpers import (
    BambuConfig,
    BambuLogger,
    FakeAfcSpool,
    FakeBridge,
    FakeSpoolmanClient,
    LaneSpec,
    make_bambu_spoolman,
    make_bambu_unit,
    make_printer,
    make_spoolman_section,
    slot_info,
    use_spoolman_client,
)


#: AFC core's Spoolman setting on a unit whose lanes a bind can reach.
BRFID_SPOOLMAN_URL = "http://spoolman:7912"


#: A Bambu reel's roll identity (tray UID), the same on both of its tags.
BRFID_TRAY = "cf34cf1d212f46b5bc8561e05eb644c8"


#: The summary held for lane12's ABS reel (138% of a 1 kg spool, 955 g),
#: said once its bay's one Spoolman lookup has gone out and bound nothing.
BRFID_ABS_HELD_LINE = (
    "Bambu_AMS_1 lane12: tag read: ABS [tag C32A080A]. Measured full -- "
    "roughly 955 g of a 1000 g spool (the AMS read 138%, meaning it "
    "measures a little larger than a reference full spool); C32A080A was "
    "looked up in Spoolman on this connection and no spool is linked to "
    "the lane from it, so the measurement stays on the lane only -- run "
    "AFC_BAMBU_SCAN LANE=lane12 to look it up again.")


def brfid_unit(monkeypatch: pytest.MonkeyPatch, name: str = "Bambu_AMS_1", *,
               lanes: Sequence[Any] = (),
               slots: Sequence[Dict[str, Any]] = (),
               section: Optional[bool] = True,
               client: Optional[FakeSpoolmanClient] = None,
               spoolman: Optional[str] = BRFID_SPOOLMAN_URL,
               inline: bool = True, **kwargs: Any) -> afcBambuAMS:
    """
    A real unit on its own printer, with its real delegate built.

    The printer's clock starts at 100.0. ``unit._measure`` is the delegate:
    the Spoolman one with an enabled section, else the measurement-only one.

    :param monkeypatch: pytest's monkeypatch fixture
    :param name: the unit's name
    :param lanes: lane names or LaneSpecs, as make_bambu_unit takes them
    :param slots: the unit's ``_slots`` records, by bay
    :param section: True registers an enabled [AFC_BambuAMS_rfid], False a
      disabled one, None none at all
    :param client: the Spoolman client the delegate reaches; an empty one
      when None
    :param spoolman: AFC core's Spoolman setting (None: not configured)
    :param inline: run the delegate's Spoolman jobs on the calling thread
    :param kwargs: further make_bambu_unit arguments
    :return afcBambuAMS: the unit, its logger empty
    """
    printer = make_printer(monkeypatch=monkeypatch)
    use_spoolman_client(monkeypatch, client if client is not None
                        else FakeSpoolmanClient())
    unit = make_bambu_unit(name, printer=printer, lanes=lanes, slots=slots,
                           **kwargs)
    if section is False:
        make_spoolman_section(printer, enabled=False)
    make_bambu_spoolman(unit, section=bool(section), spoolman_url=spoolman,
                        inline=inline)
    unit.afc.spoolman = spoolman
    unit.logger.messages.clear()
    return unit


def brfid_spool(spool_id: int, *, uids: Sequence[str] = (),
                tray_uid: Optional[str] = None,
                **fields: Any) -> Dict[str, Any]:
    """
    A Spoolman spool record as SpoolmanClient returns it.

    :param spool_id: the spool's id
    :param uids: chip UIDs in its ``card_uids`` extra field
    :param tray_uid: the roll identity in its ``tray_uid`` extra field
    :param fields: further top-level fields (remaining_weight...)
    :return dict: the record, extra fields JSON-encoded as Spoolman keeps them
    """
    extra: Dict[str, str] = {}
    if uids:
        extra["card_uids"] = '"' + ",".join(uids) + '"'
    if tray_uid is not None:
        extra["tray_uid"] = f'"{tray_uid}"'
    spool: Dict[str, Any] = {"id": spool_id, "extra": extra}
    spool.update(fields)
    return spool


class TestQuietInfoGetattr:
    """Everything but info() passes straight through to the wrapped logger."""

    def test_errors_are_never_swallowed(self):
        # Quietening a duplicate must not quieten a failure.
        inner = BambuLogger()
        quiet = _QuietInfo(inner)
        quiet.error("Spoolman apply failed for lane12")
        quiet.debug("spool #124 fetched")
        assert inner.messages == [("error", "Spoolman apply failed for lane12"),
                                  ("debug", "spool #124 fetched")]


class TestQuietInfoInfo:
    """The shared binder's INFO lines land at DEBUG."""

    class _NoDebugLogger:
        """A logger whose debug() fails."""

        def __init__(self) -> None:
            """Start with nothing tried."""
            self.tried: List[str] = []

        def debug(self, message: str) -> None:
            """:param message: recorded, then refused"""
            self.tried.append(message)
            raise RuntimeError("logger closed")

    def test_info_lands_in_debug(self):
        inner = BambuLogger()
        _QuietInfo(inner).info("spool #124 assigned to lane12")
        assert inner.messages == [("debug", "spool #124 assigned to lane12")]

    def test_a_failing_logger_never_breaks_the_bind(self):
        inner = self._NoDebugLogger()
        _QuietInfo(inner).info("spool #124 assigned to lane12")
        assert inner.tried == ["spool #124 assigned to lane12"]


class TestBambuSpoolmanClient:
    """The delegate builds its client from AFC's shared per-afc cache."""

    class _Afc:
        """The two AFC attributes the builder reads, and the cache slot."""

        def __init__(self, spoolman: Any, moonraker: Any) -> None:
            """
            :param spoolman: AFC's configured Spoolman URL, or None
            :param moonraker: AFC's Moonraker client, or None
            """
            self.spoolman = spoolman
            self.moonraker = moonraker

    @pytest.mark.parametrize("spoolman,moonraker", [
        (None, object()), (BRFID_SPOOLMAN_URL, None)],
        ids=["no-spoolman", "no-moonraker"])
    def test_nothing_to_reach_spoolman_through_is_no_client(self, spoolman,
                                                           moonraker):
        afc = self._Afc(spoolman, moonraker)
        assert rfid_mod._bambu_spoolman_client(afc) is None
        assert rfid_mod._bambu_spoolman_client(None) is None
        assert not hasattr(afc, "_afc_spoolman_client_cache")

    class _Moonraker:
        """Moonraker as far as SpoolmanClient reads it."""
        host = "http://localhost:7125"
        logger = BambuLogger()

    def test_every_call_shares_the_cached_client(self):
        moonraker = self._Moonraker()
        afc = self._Afc(BRFID_SPOOLMAN_URL, moonraker)
        client = rfid_mod._bambu_spoolman_client(afc)
        assert isinstance(client, rfid_mod.SpoolmanClient)
        assert client.host == "http://localhost:7125"
        assert afc._afc_spoolman_client_cache is client
        assert rfid_mod._bambu_spoolman_client(afc) is client

    def test_a_client_that_cannot_be_built_is_no_client(self):
        # A Moonraker object with no host: both the cache and a fresh build fail.
        afc = self._Afc(BRFID_SPOOLMAN_URL, object())
        assert rfid_mod._bambu_spoolman_client(afc) is None
        assert not hasattr(afc, "_afc_spoolman_client_cache")


class TestBambuSpoolmanInit:
    """A new delegate starts with every memo empty, bound to its unit."""

    def test_the_delegate_starts_with_empty_memos(self, monkeypatch):
        unit = brfid_unit(monkeypatch)
        sp = unit._spool
        assert isinstance(sp, BambuSpoolman)
        assert sp._u is unit
        assert sp.spoolman_on is True
        assert sp._spoolman_no_match == set()
        assert sp._spoolman_inflight == set()
        assert sp._bind_pending == set()
        assert sp._bound_uid == {} and sp._binding_check == {}
        assert sp._bind_owed == {} and sp._measured_remain == {}
        assert sp._convert_owed == {} and sp._pending_summary == {}
        assert unit.logger.messages == []

    def test_a_measurement_only_delegate_has_spoolman_off(self, monkeypatch):
        unit = brfid_unit(monkeypatch, section=None)
        sp = unit._measure
        assert unit._spool is None
        assert sp._u is unit
        assert sp.spoolman_on is False
        assert sp._measured_remain == {} and sp._pending_summary == {}
        assert unit.logger.messages == []


class TestBambuSpoolmanForgetSpoolmanMiss:
    """A spool leaving a bay forgets every "Spoolman has no spool" memo."""

    def test_removal_forgets_the_miss(self, monkeypatch):
        sp = brfid_unit(monkeypatch)._measure
        sp._spoolman_no_match.add("ECB61CD0")
        sp._forget_spoolman_miss(0)
        assert sp._spoolman_no_match == set()
        assert sp._u.logger.messages == []

    def test_forgetting_clears_the_whole_set(self, monkeypatch):
        # The whole set, not this slot's UID: a power-cycled unit blanks the
        # record, leaving a memoized UID nobody can name any more.
        sp = brfid_unit(monkeypatch)._measure
        sp._spoolman_no_match.update({"ECB61CD0", "13F56D32"})
        sp._forget_spoolman_miss(1)
        assert sp._spoolman_no_match == set()
        assert sp._u.logger.messages == []

    def test_forget_survives_a_missing_memo(self, monkeypatch):
        # Nothing memoized (a fresh delegate, or one whose memo the connection
        # already reset): there is nothing to forget, and nothing fails.
        sp = brfid_unit(monkeypatch)._measure
        memo = sp._spoolman_no_match
        sp._forget_spoolman_miss(0)
        assert sp._spoolman_no_match is memo and memo == set()
        assert sp._u.logger.messages == []

    def test_forget_survives_a_deleted_memo(self, monkeypatch):
        # The memo is read with getattr: a delegate without it forgets
        # nothing, and does not grow one back.
        sp = brfid_unit(monkeypatch)._measure
        del sp._spoolman_no_match
        sp._forget_spoolman_miss(0)
        assert not hasattr(sp, "_spoolman_no_match")
        assert sp._u.logger.messages == []

    def test_forget_spares_the_binding_check(self, monkeypatch):
        # The binding-check memo resets only with the connection: clearing it
        # per edge re-probed Spoolman for every bound lane on the next pass.
        sp = brfid_unit(monkeypatch)._measure
        sp._spoolman_no_match = {"AABBCCDD"}
        sp._binding_check = {("87", "AABBCCDD"): False}
        sp._forget_spoolman_miss(0)
        assert sp._spoolman_no_match == set()
        assert sp._binding_check == {("87", "AABBCCDD"): False}
        assert sp._u.logger.messages == []


class TestBambuSpoolmanBindingContradicted:
    """Only a positive Spoolman answer unbinds, and only answers are memoized."""

    class _FlakyClient(FakeSpoolmanClient):
        """A Spoolman client whose get_spool fails while ``down``."""

        def __init__(self, spools: Sequence[Dict[str, Any]] = ()) -> None:
            """:param spools: Spoolman's spool records"""
            super().__init__(spools)
            self.down = False
            self.asked: List[int] = []

        def get_spool(self, spool_id: int) -> Optional[Dict[str, Any]]:
            """
            :param spool_id: the spool asked for
            :return Optional[dict]: that spool, unless Spoolman is down
            """
            self.asked.append(spool_id)
            if self.down:
                raise RuntimeError("down")
            return super().get_spool(spool_id)

    def test_a_binding_lookup_failure_is_not_memoized(self, monkeypatch):
        client = self._FlakyClient([brfid_spool(87, uids=["11223344"])])
        client.down = True
        sp = brfid_unit(monkeypatch, client=client)._measure
        assert sp._binding_contradicted(87, "AABBCCDD") is False
        assert sp._binding_check == {}
        # Spoolman comes back with a real contradiction: it lands and sticks.
        client.down = False
        assert sp._binding_contradicted(87, "AABBCCDD") is True
        assert sp._binding_check == {("87", "AABBCCDD"): True}
        assert client.asked == [87, 87]
        assert sp._u.logger.messages == []

    def test_a_memoized_verdict_is_not_asked_again(self, monkeypatch):
        client = self._FlakyClient([brfid_spool(87, uids=["11223344"])])
        sp = brfid_unit(monkeypatch, client=client)._measure
        sp._binding_check = {("87", "AABBCCDD"): False}
        assert sp._binding_contradicted(87, "AABBCCDD") is False
        assert client.asked == []
        assert sp._binding_check == {("87", "AABBCCDD"): False}
        assert sp._u.logger.messages == []

    def test_a_spool_with_no_recorded_uid_is_no_contradiction(self, monkeypatch):
        # The hand-assigned spool keeps its lane; the answer is still an
        # answer, so it is memoized.
        client = self._FlakyClient([brfid_spool(42)])
        sp = brfid_unit(monkeypatch, client=client)._measure
        assert sp._binding_contradicted(42, "ECB61CD0") is False
        assert sp._binding_check == {("42", "ECB61CD0"): False}
        assert client.asked == [42]
        assert sp._u.logger.messages == []

    def test_the_tag_on_the_spool_is_no_contradiction(self, monkeypatch):
        client = self._FlakyClient([brfid_spool(87, uids=["ECB61CD0"])])
        sp = brfid_unit(monkeypatch, client=client)._measure
        assert sp._binding_contradicted(87, "ec:b6:1c:d0") is False
        assert sp._binding_check == {("87", "ec:b6:1c:d0"): False}
        assert client.asked == [87]
        assert sp._u.logger.messages == []

    def test_a_lost_memo_is_rebuilt(self, monkeypatch):
        client = self._FlakyClient([brfid_spool(87, uids=["11223344"])])
        sp = brfid_unit(monkeypatch, client=client)._measure
        sp._binding_check = None
        assert sp._binding_contradicted(87, "AABBCCDD") is True
        assert sp._binding_check == {("87", "AABBCCDD"): True}
        assert client.asked == [87]
        assert sp._u.logger.messages == []

    def test_no_spool_or_no_tag_asks_nothing(self, monkeypatch):
        client = self._FlakyClient([brfid_spool(87, uids=["11223344"])])
        sp = brfid_unit(monkeypatch, client=client)._measure
        assert sp._binding_contradicted(None, "AABBCCDD") is False
        assert sp._binding_contradicted(87, "") is False
        assert client.asked == []
        assert sp._binding_check == {}
        assert sp._u.logger.messages == []

    @pytest.mark.parametrize("helper", ["_spool_uids", "_norm_uid"])
    def test_without_the_afc_rfid_helpers_it_asks_nothing(self, helper,
                                                          monkeypatch):
        # Spool 87 carries another tag, so Spoolman would contradict the
        # binding: only the missing helper keeps it from being asked.
        client = self._FlakyClient([brfid_spool(87, uids=["11223344"])])
        sp = brfid_unit(monkeypatch, client=client)._measure
        monkeypatch.setattr(rfid_mod, helper, None)
        assert sp._binding_contradicted(87, "AABBCCDD") is False
        assert client.asked == []
        assert sp._binding_check == {}
        assert sp._u.logger.messages == []


class TestBambuSpoolmanSpoolmanBg:
    """Spoolman HTTP runs on the shared worker thread, or inline when off."""

    @staticmethod
    def _fresh_worker(monkeypatch: pytest.MonkeyPatch) -> None:
        """
        Start this test with no worker, so the one it starts is its own.

        :param monkeypatch: pytest's monkeypatch fixture
        """
        monkeypatch.setattr(BambuSpoolman, "_spool_q", None)
        monkeypatch.setattr(BambuSpoolman, "_spool_t", None)

    def test_spoolman_jobs_run_off_the_reactor_for_a_real_delegate(
            self, monkeypatch):
        self._fresh_worker(monkeypatch)
        sp = brfid_unit(monkeypatch, inline=False)._measure
        ran = threading.Event()
        seen: List[str] = []

        def job() -> None:
            """Record the thread the job runs on."""
            seen.append(threading.current_thread().name)
            ran.set()

        sp._spoolman_bg(job)
        assert ran.wait(5.0)
        assert seen == ["afc_bambu_spool"]
        worker = BambuSpoolman._spool_t
        assert worker.name == "afc_bambu_spool" and worker.daemon is True
        # A second job goes to the same queue and the same thread.
        queue_before = BambuSpoolman._spool_q
        ran.clear()
        sp._spoolman_bg(job)
        assert ran.wait(5.0)
        assert seen == ["afc_bambu_spool", "afc_bambu_spool"]
        assert BambuSpoolman._spool_q is queue_before
        assert BambuSpoolman._spool_t is worker
        assert sp._u.logger.messages == []

    def test_a_stand_in_without_the_flag_runs_the_job_inline(self, monkeypatch):
        # SPOOLMAN_BG off on the delegate: the job runs before the call
        # returns, on the calling thread, and no worker is started.
        self._fresh_worker(monkeypatch)
        sp = brfid_unit(monkeypatch, inline=True)._measure
        seen: List[str] = []
        sp._spoolman_bg(lambda: seen.append(threading.current_thread().name))
        assert seen == [threading.current_thread().name]
        assert BambuSpoolman._spool_q is None
        assert BambuSpoolman._spool_t is None
        assert sp._u.logger.messages == []


class TestBambuSpoolmanBindByUidBg:
    """The roll is looked up before the tag, and the answer binds on-reactor."""

    class _LandingSpool(FakeAfcSpool):
        """
        AFC_spool's set_spoolID as AFC has it: the spool is fetched first,
        and on_done fires once the fetch has put it on the lane.
        """

        def __init__(self) -> None:
            """Start with nothing fetching."""
            super().__init__()
            self.fetching: List[Tuple[Any, Any, Any]] = []

        def set_spoolID(self, lane: Any, spool_id: Any,  # type: ignore[override]
                        save_vars: bool = True, on_done: Any = None) -> None:
            """Record the bind; it lands with land()."""
            super().set_spoolID(lane, spool_id, on_done=on_done)
            self.fetching.append((lane, spool_id, on_done))

        def land(self) -> None:
            """Every fetch answers: the lane takes its spool, then on_done."""
            fetching, self.fetching = self.fetching, []
            for lane, spool_id, on_done in fetching:
                lane.spool_id = spool_id
                if on_done is not None:
                    on_done()

    class _NoMetadataClient(FakeSpoolmanClient):
        """Spoolman refusing the card_uids write."""

        def __init__(self, spools: Sequence[Dict[str, Any]] = ()) -> None:
            """:param spools: Spoolman's spool records"""
            super().__init__(spools)
            self.refused: List[int] = []

        def write_spool_metadata(self, spool_id: int, **fields: Any) -> None:
            """:raises RuntimeError: always, after noting the spool"""
            self.refused.append(spool_id)
            raise RuntimeError("HTTP 500")

    @staticmethod
    def _bind_unit(monkeypatch: pytest.MonkeyPatch, client: FakeSpoolmanClient,
                   uid: str, tray_uid: str = BRFID_TRAY) -> afcBambuAMS:
        """
        A unit whose bay 2 holds the tagged reel, mapped to lane14.

        :param monkeypatch: pytest's monkeypatch fixture
        :param client: Spoolman
        :param uid: the chip UID in the bay's record
        :param tray_uid: the roll identity in the bay's record
        :return afcBambuAMS: the unit
        """
        record = slot_info(2, material="PLA Basic", uid=uid,
                           tray_uid=tray_uid or None, weight=1000)
        return brfid_unit(monkeypatch, client=client,
                          lanes=[LaneSpec("lane14", 2)],
                          slots=[{}, {}, record])

    def test_the_roll_is_looked_up_before_the_tag(self, monkeypatch):
        # The other face of a known reel: the chip UID is a stranger, the tray
        # UID is not. The record is taught this face, so the next flip matches.
        client = FakeSpoolmanClient([brfid_spool(132, uids=["D34E4E39"],
                                                 tray_uid=BRFID_TRAY)])
        unit = self._bind_unit(monkeypatch, client, "13f56d32")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "13f56d32", "", tray_uid=BRFID_TRAY)
        assert sp._spoolman_inflight == {"13f56d32"}
        assert sp._bind_pending == {2}
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 132), {})]
        assert client.calls == [("write_spool_metadata",
                                 (132, {"uid": "13f56d32"}))]
        assert sp._bound_uid == {2: "13f56d32"}
        assert sp._spoolman_inflight == set() and sp._bind_pending == set()
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "132 by tray UID cf34cf1d212f46b5bc8561e05eb644c8")]

    def test_lost_inflight_and_pending_sets_are_rebuilt(self, monkeypatch):
        client = FakeSpoolmanClient([brfid_spool(132, uids=["D34E4E39"],
                                                 tray_uid=BRFID_TRAY)])
        unit = self._bind_unit(monkeypatch, client, "13f56d32")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._spoolman_inflight = None
        sp._bind_pending = None
        sp._bind_by_uid_bg(lane, 2, "13f56d32", "", tray_uid=BRFID_TRAY)
        assert sp._spoolman_inflight == {"13f56d32"}
        assert sp._bind_pending == {2}
        assert unit.logger.messages == []
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 132), {})]
        assert sp._spoolman_inflight == set() and sp._bind_pending == set()
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "132 by tray UID cf34cf1d212f46b5bc8561e05eb644c8")]

    def test_a_chip_match_stamps_the_roll_identity_on(self, monkeypatch):
        # The face already known, on a record with no tray UID yet: stamp it,
        # or the other face is still a stranger next time.
        client = FakeSpoolmanClient([brfid_spool(132, uids=["D34E4E39"])])
        unit = self._bind_unit(monkeypatch, client, "d34e4e39")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "d34e4e39", "", tray_uid=BRFID_TRAY)
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 132), {})]
        assert client.calls == [
            ("write_tray_uid", (132, BRFID_TRAY)),
            ("write_spool_metadata", (132, {"uid": "d34e4e39"}))]
        assert sp._bound_uid == {2: "d34e4e39"}
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "132 by UID d34e4e39")]

    def test_a_refused_roll_identity_stamp_still_binds(self, monkeypatch):
        class _NoTrayClient(FakeSpoolmanClient):
            """Spoolman refusing the tray UID write."""

            def write_tray_uid(self, spool_id: int, tray_uid: str) -> None:
                """:raises RuntimeError: always, after noting the write"""
                super().write_tray_uid(spool_id, tray_uid)
                raise RuntimeError("HTTP 500")

        client = _NoTrayClient([brfid_spool(132, uids=["D34E4E39"])])
        unit = self._bind_unit(monkeypatch, client, "d34e4e39")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "d34e4e39", "", tray_uid=BRFID_TRAY)
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 132), {})]
        assert client.calls == [
            ("write_tray_uid", (132, BRFID_TRAY)),
            ("write_spool_metadata", (132, {"uid": "d34e4e39"}))]
        assert sp._bound_uid == {2: "d34e4e39"}
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "132 by UID d34e4e39")]

    def test_a_restored_lookup_with_no_client_is_asked_again(self, monkeypatch):
        # Spoolman or Moonraker not up yet: for a restored lane that is no
        # miss, and the bay is asked again later.
        client = FakeSpoolmanClient([brfid_spool(132, uids=["D34E4E39"])])
        unit = self._bind_unit(monkeypatch, client, "d34e4e39")
        use_spoolman_client(monkeypatch, None)
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "d34e4e39", "", tray_uid=BRFID_TRAY,
                           restored=True)
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [] and client.calls == []
        assert sp._spoolman_no_match == set()
        assert unit._lookup_retry == {2: unit.afc.reactor.monotonic() + 30.0}
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: Spoolman did not answer the lookup of "
                      "D34E4E39 for lane14; asking again in 30 s")]

    def test_a_conflicting_roll_identity_is_left_alone(self, monkeypatch):
        # The record names a different roll: overwriting would bury it.
        client = FakeSpoolmanClient([brfid_spool(132, uids=["D34E4E39"],
                                                 tray_uid="4e3177c3")])
        unit = self._bind_unit(monkeypatch, client, "d34e4e39")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "d34e4e39", "", tray_uid=BRFID_TRAY)
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 132), {})]
        assert client.calls == [("write_spool_metadata",
                                 (132, {"uid": "d34e4e39"}))]
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: spool 132 carries tray UID "
                      "4e3177c3 but this tag says "
                      "cf34cf1d212f46b5bc8561e05eb644c8 -- left alone; one of "
                      "them is on the wrong record"),
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "132 by UID d34e4e39")]

    def test_a_tagless_brand_still_binds_by_chip_uid(self, monkeypatch):
        # Only Bambu writes a tray UID; no tray UID must cost nothing.
        client = FakeSpoolmanClient([brfid_spool(77, uids=["AABBCCDD"])])
        unit = self._bind_unit(monkeypatch, client, "aabbccdd", tray_uid="")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "aabbccdd", " (no tag profile decoded)",
                           tray_uid="")
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 77), {})]
        assert client.calls == [("write_spool_metadata",
                                 (77, {"uid": "aabbccdd"}))]
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "77 by UID aabbccdd (no tag profile decoded)")]

    def test_a_lookup_answered_after_its_spool_left_binds_nothing(
            self, monkeypatch):
        # The answer reaches the reactor after the reel was pulled: the bay no
        # longer holds the tag, so the answer binds nothing.
        client = FakeSpoolmanClient([brfid_spool(77, uids=["AABBCCDD"])])
        unit = self._bind_unit(monkeypatch, client, "aabbccdd", tray_uid="")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "aabbccdd", "", tray_uid="")
        unit._slots[2] = slot_info(2, present=False)
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == []
        assert lane.spool_id is None
        assert sp._bound_uid == {} and sp._spoolman_no_match == set()
        assert sp._bind_pending == set() and sp._spoolman_inflight == set()
        assert unit.logger.messages == []

    def test_a_lookup_already_in_flight_is_not_sent_again(self, monkeypatch):
        client = FakeSpoolmanClient([brfid_spool(77, uids=["AABBCCDD"])])
        unit = self._bind_unit(monkeypatch, client, "aabbccdd", tray_uid="")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._spoolman_inflight = {"aabbccdd"}
        sp._bind_by_uid_bg(lane, 2, "aabbccdd", "", tray_uid="")
        assert unit.afc.reactor.run_callbacks() == 0
        assert client.calls == [] and sp._bind_pending == set()
        assert sp._spoolman_inflight == {"aabbccdd"}
        assert unit.logger.messages == []

    def test_a_bind_afc_still_fetches_stays_pending_until_it_lands(
            self, monkeypatch):
        # AFC binds once its own fetch answers: until then the bind is
        # pending, so the summary and a held measurement keep waiting.
        client = FakeSpoolmanClient([brfid_spool(77, uids=["AABBCCDD"])])
        unit = self._bind_unit(monkeypatch, client, "aabbccdd", tray_uid="")
        unit.afc.spool = self._LandingSpool()
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "aabbccdd", "", tray_uid="")
        unit.afc.reactor.run_callbacks()
        assert sp._bind_pending == {2} and sp._bound_uid == {}
        assert sp._spoolman_inflight == set()
        unit.afc.spool.land()
        assert lane.spool_id == 77
        assert sp._bind_pending == set()
        assert sp._bound_uid == {2: "aabbccdd"}
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "77 by UID aabbccdd")]

    def test_a_failed_card_uids_write_still_binds(self, monkeypatch):
        # Teaching the record its chip UID is best-effort: the bind stands.
        client = self._NoMetadataClient([brfid_spool(77, uids=["AABBCCDD"])])
        unit = self._bind_unit(monkeypatch, client, "aabbccdd", tray_uid="")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "aabbccdd", "", tray_uid="")
        unit.afc.reactor.run_callbacks()
        assert client.refused == [77]
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 77), {})]
        assert sp._bound_uid == {2: "aabbccdd"}
        assert sp._spoolman_no_match == set()
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "77 by UID aabbccdd")]

    def test_a_client_that_cannot_be_built_is_a_miss(self, monkeypatch):
        # A read of the bay, not a restored lane: no answer is a miss.
        client = FakeSpoolmanClient([brfid_spool(77, uids=["AABBCCDD"])])
        unit = self._bind_unit(monkeypatch, client, "aabbccdd", tray_uid="")

        def _no_client(afc: Any) -> Any:
            """:raises RuntimeError: always"""
            raise RuntimeError("moonraker has no spoolman section")

        monkeypatch.setattr(rfid_mod, "_bambu_spoolman_client", _no_client)
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_owed = {2: (74, 1000, None, False)}
        sp._bind_by_uid_bg(lane, 2, "aabbccdd", "", tray_uid="")
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [] and client.calls == []
        assert sp._spoolman_no_match == {"aabbccdd"}
        assert sp._bind_owed == {}
        assert sp._bind_pending == set() and sp._spoolman_inflight == set()
        assert unit.logger.messages == []

    def test_a_reactor_that_refuses_the_callback_binds_inline(
            self, monkeypatch):
        client = FakeSpoolmanClient([brfid_spool(77, uids=["AABBCCDD"])])
        unit = self._bind_unit(monkeypatch, client, "aabbccdd", tray_uid="")

        def _refuse(callback: Any, waketime: float = 0.0) -> None:
            """:raises RuntimeError: always"""
            raise RuntimeError("reactor shutting down")

        monkeypatch.setattr(unit.afc.reactor, "register_async_callback",
                            _refuse)
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, 2, "aabbccdd", "", tray_uid="")
        # Bound before the call returned, with nothing left queued.
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 77), {})]
        assert sp._bound_uid == {2: "aabbccdd"}
        assert sp._bind_pending == set() and sp._spoolman_inflight == set()
        assert unit.afc.reactor.run_callbacks() == 0
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "77 by UID aabbccdd")]

    def test_a_bind_for_an_unknown_bay_still_binds(self, monkeypatch):
        # No bay index: nothing is held pending for it and no bay can be
        # remembered as bound, but the lane is still bound.
        client = FakeSpoolmanClient([brfid_spool(77, uids=["AABBCCDD"])])
        unit = self._bind_unit(monkeypatch, client, "aabbccdd", tray_uid="")
        sp, lane = unit._measure, unit.lanes["lane14"]
        sp._bind_by_uid_bg(lane, None, "aabbccdd", "", tray_uid="")
        assert sp._bind_pending == set()
        assert sp._spoolman_inflight == {"aabbccdd"}
        unit.afc.reactor.run_callbacks()
        assert unit.afc.spool.calls == [("set_spoolID", (lane, 77), {})]
        assert sp._bound_uid == {} and sp._spoolman_inflight == set()
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: matched lane14 to Spoolman spool "
                      "77 by UID aabbccdd")]


class TestBambuSpoolmanBindLanded:
    """A bind that has run its course: kept, undone, or released."""

    #: The reel lane15's link came from: spool 136.
    UID = "4b8e44f6"

    def _landed_unit(self, monkeypatch: pytest.MonkeyPatch,
                     record: Dict[str, Any]) -> afcBambuAMS:
        """
        A unit whose lane15, on bay 3, AFC has just bound to spool 136, with
        its bind still pending.

        :param monkeypatch: pytest's monkeypatch fixture
        :param record: bay 3's record as the bind lands
        :return afcBambuAMS: the unit
        """
        unit = brfid_unit(
            monkeypatch,
            lanes=[LaneSpec("lane15", 3, spool_id=136, material="PLA",
                            color="#0086D6", weight=670.8,
                            extras={"sub_type": "Basic",
                                    "filament_name": "Bambu PLA Basic",
                                    "spool_vendor": "Bambu"})],
            slots=[{}, {}, {}, record])
        unit._measure._bind_pending = {3}
        return unit

    @staticmethod
    def _profile(lane: Any) -> Dict[str, Any]:
        """
        :param lane: the lane
        :return dict: the filament fields a link or a clear writes
        """
        return {k: getattr(lane, k) for k in (
            "spool_id", "material", "color", "sub_type", "filament_name",
            "spool_vendor", "weight")}

    @pytest.mark.parametrize("miss", [True, False], ids=["create", "match"])
    @pytest.mark.parametrize("lands", ["bay-empty", "after-defaults"])
    def test_a_link_that_lands_after_its_spool_left_is_undone(
            self, lands, miss, monkeypatch):
        # The reel was pulled while Spoolman answered. The removal edge has
        # been and gone, so nothing else would unbind the lane, and a reel put
        # in after it would be charged to spool 136.
        if lands == "bay-empty":
            unit = self._landed_unit(monkeypatch, slot_info(3, present=False))
            expected = {"spool_id": "", "material": "", "color": "",
                        "sub_type": "", "filament_name": "",
                        "spool_vendor": "", "weight": 0}
        else:
            # A reel nothing has read went in: lane defaults again.
            unit = self._landed_unit(monkeypatch, slot_info(3))
            unit._removed_bays = {3}
            expected = {"spool_id": "", "material": "PLA", "color": "",
                        "sub_type": "", "filament_name": "",
                        "spool_vendor": "", "weight": 1000}
        sp, lane = unit._measure, unit.lanes["lane15"]
        sp._bind_landed(lane, 3, self.UID, miss=miss)
        assert self._profile(lane) == expected
        assert lane.bambu_sku == "" and lane.empty_spool_weight == 190.0
        assert sp._bind_pending == set()
        assert sp._bound_uid == {} and sp._spoolman_no_match == set()
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: unbinding lane15 from spool 136 "
                      "-- its Spoolman link for tag 4B8E44F6 landed after "
                      "that spool left the bay")]

    def test_a_reel_still_settling_is_left_to_its_own_defaults(
            self, monkeypatch):
        # A bay the settle timer still owes defaults gets them, and the line,
        # from that timer: the link is undone and nothing more.
        unit = self._landed_unit(monkeypatch, slot_info(3))
        unit._removed_bays = {3}
        unit._defaults_due = {3: 130.0}
        sp, lane = unit._measure, unit.lanes["lane15"]
        sp._bind_landed(lane, 3, self.UID)
        # Cleared, not put on the PLA / 1000 g defaults the timer will give.
        assert self._profile(lane) == {
            "spool_id": "", "material": "", "color": "", "sub_type": "",
            "filament_name": "", "spool_vendor": "", "weight": 0}
        assert unit._defaults_due == {3: 130.0}
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: unbinding lane15 from spool 136 "
                      "-- its Spoolman link for tag 4B8E44F6 landed after "
                      "that spool left the bay")]

    @pytest.mark.parametrize("lands", ["nothing-known", "record-blank"])
    def test_a_link_in_flight_across_a_bridge_reboot_is_kept(
            self, lands, monkeypatch):
        # A bay nothing is known about, or one occupied with a blank record
        # (a polled bay after a reboot), has not shown its reel left. The
        # real frames are driven in ..._stays_kept below.
        record = {} if lands == "nothing-known" else slot_info(3)
        unit = self._landed_unit(monkeypatch, record)
        sp, lane = unit._measure, unit.lanes["lane15"]
        sp._bind_landed(lane, 3, self.UID)
        assert self._profile(lane) == {
            "spool_id": 136, "material": "PLA", "color": "#0086D6",
            "sub_type": "Basic", "filament_name": "Bambu PLA Basic",
            "spool_vendor": "Bambu", "weight": 670.8}
        assert sp._bound_uid == {3: self.UID}
        assert sp._bind_pending == set()
        assert unit.afc.save_vars.call_count == 0
        assert unit.logger.messages == []

    # ── the same landings, reached through the unit's own status frames ──
    #
    # Printer 1's lane15 mid-print, bound to spool 136, on bay 4 of an AMS 1:
    # a flap relinks it and the reel is pulled while Spoolman answers.

    #: lane15's chip and reel on printer 1.
    FLAP_UID = "95f2c30c"
    FLAP_TRAY = "4e3177c31fea42d08bc240c818a37f80"

    class _FetchingSpool(FakeAfcSpool):
        """
        AFC core's spool object as AFC_spool has it: set_spoolID fetches the
        spool, puts Spoolman's record on the lane when the fetch lands, and
        only then fires on_done.
        """

        def __init__(self) -> None:
            """Start with nothing fetching."""
            super().__init__()
            self.fetching: List[Tuple[Any, Any, Any]] = []

        def set_spoolID(self, lane: Any, spool_id: Any, save_vars: bool = True,
                        on_done: Any = None) -> None:
            """Record the binding and start its fetch."""
            super().set_spoolID(lane, spool_id, save_vars=save_vars,
                                on_done=on_done)
            self.fetching.append((lane, spool_id, on_done))

        def land(self) -> None:
            """The fetches answer with spool 136's record."""
            fetching, self.fetching = self.fetching, []
            for lane, spool_id, on_done in fetching:
                lane.spool_id, lane.weight = spool_id, 670.8
                lane.material, lane.color = "PLA", "#0086D6"
                if on_done is not None:
                    on_done()

    @staticmethod
    def _bay(slot: int, present: bool, tagged: bool = True,
             **fields: Any) -> Dict[str, Any]:
        """
        :param slot: the bay, 0-based
        :param present: a spool is in it
        :param tagged: its record carries lane15's tag (PLA Basic #0086D6)
        :param fields: raw keys to override (mpct...)
        :return dict: one raw slot entry as the bridge firmware sends it
        """
        entry: Dict[str, Any] = {
            "unit": 0, "i": slot, "present": 1 if present else 0,
            "state": "loaded" if present else "empty", "rrq": 0, "sseq": 0,
            "sres": 0, "mseq": 0, "mpct": 0}
        if tagged:
            entry.update({
                "material": "PLA Basic", "sku": "GFA00", "color": "0086D6FF",
                "tmin": 190, "tmax": 230, "weight": 1000,
                "uid": TestBambuSpoolmanBindLanded.FLAP_UID,
                "tray_uid": TestBambuSpoolmanBindLanded.FLAP_TRAY,
                "remain": 0})
        entry.update(fields)
        return entry

    def _frame(self, present: bool, tagged: bool = True,
               polled: Optional[bool] = None,
               **fields: Any) -> Dict[str, Any]:
        """
        :param present: lane15's bay (bay 4) is occupied
        :param tagged: lane15's bay carries its tag
        :param polled: None: a frame with no units list; False: a rebooted
          bridge's frame from before its first presence reply; True: one
          after it
        :param fields: raw keys for lane15's bay
        :return dict: a raw status frame, bay 1 empty
        """
        if polled is not None:
            fields = dict({"mpct": -1}, **fields)
        frame: Dict[str, Any] = {"slots": [
            self._bay(0, False, tagged=False),
            self._bay(3, present, tagged, **fields)]}
        if polled is not None:
            frame["units"] = [{"n": 0, "online": polled,
                               "preslen": 60 if polled else 0,
                               "presbyte": 8 if polled else 0}]
        return frame

    @staticmethod
    def _feed(unit: afcBambuAMS, frame: Dict[str, Any], dt: float = 0.24,
              times: int = 1) -> None:
        """
        Status frames ``dt`` seconds apart, each followed by the reactor's
        due callbacks.

        :param unit: the unit
        :param frame: the raw frame
        :param dt: seconds since the last frame
        :param times: how many
        """
        for _ in range(times):
            unit.reactor.now += dt
            unit._on_status(frame)
            unit.reactor.run_callbacks()

    def _flap_unit(self, monkeypatch: pytest.MonkeyPatch) -> afcBambuAMS:
        """
        lane15 mid-print, bound to spool 136, its bay occupied, PREP run and
        primed, the bay claimed by AFC; Spoolman knows spool 136 by lane15's
        chip and reel. Setup lines and sends are cleared.

        :param monkeypatch: pytest's monkeypatch fixture
        :return afcBambuAMS: "Bambu_AMS_1"
        """
        printer = make_printer(now=126000.0, monkeypatch=monkeypatch,
                               print_state="printing")
        unit = make_bambu_unit(printer=printer, model="ams1", lanes=[LaneSpec(
            "lane15", 3, prep=True, loaded_to_hub=True, material="PLA",
            color="#0086D6", spool_id=136, weight=670,
            extras={"sub_type": "Basic", "spool_vendor": "Bambu",
                    "filament_name": "Bambu PLA Basic",
                    "extruder_temp": 190.0, "bed_temp": 55.0,
                    "bambu_sku": "GFA00"})])
        use_spoolman_client(monkeypatch, FakeSpoolmanClient([
            {"id": 136, "remaining_weight": 670.8,
             "extra": {"card_uids": '"4B8E44F6,95F2C30C"',
                       "tray_uid": f'"{self.FLAP_TRAY}"'}}]))
        make_bambu_spoolman(unit, spoolman_url=BRFID_SPOOLMAN_URL)
        unit.afc.spool = self._FetchingSpool()
        session = self._frame(True)
        self._feed(unit, session)                      # the baseline frame
        unit._prep_seen = True
        unit._afc_owned.add(3)                         # PREP: the lane has data
        unit._scan_primed = True
        self._feed(unit, session, dt=1.0, times=3)
        unit.logger.messages.clear()
        unit._bridge.sent.clear()
        return unit

    #: Prefix of the flap unit's lines.
    FLAP = "AFC bambu Bambu_AMS_1"

    def _flap_lines(self, matched: bool) -> List[Tuple[str, str]]:
        """
        :param matched: the relink went through the match-only lookup
        :return list: what the flap, its relink and the real removal log
        """
        removed = ("info", f"{self.FLAP}: spool REMOVED from slot 3 (AMS bay 4)")
        lines = [
            removed,
            ("debug", f"{self.FLAP}: unbinding lane15 from spool 136 -- the "
                      f"bay is empty"),
            ("info", f"{self.FLAP}: spool INSERTED in slot 3 (AMS bay 4)"),
            ("info", f"{self.FLAP}: applied tag to lane15: Bambu PLA Basic "
                     f"#0086D6"),
            ("debug", f"{self.FLAP}: lane15 tag -- PLA Basic GFA00 | color "
                      f"#0086D6 | uid 95F2C30C | remaining no measurement yet "
                      f"| nominal 1000 g | nozzle 190-230C | bed 55C "
                      f"(material)")]
        if matched:
            lines.append(("debug", f"{self.FLAP}: matched lane15 to Spoolman "
                                   f"spool 136 by tray UID {self.FLAP_TRAY}"))
        return lines

    def _unread_lines(self) -> List[Tuple[str, str]]:
        """:return list: an unread reel going in mid-print, said once"""
        return [
            ("info", f"{self.FLAP}: spool INSERTED in slot 3 (AMS bay 4)"),
            ("info", f"{self.FLAP}: spool INSERTED in slot 3 during a print, "
                     f"and nothing has read it -- lane15 is on lane defaults "
                     f"and linked to no spool until it is read. The unit may "
                     f"read it on its own; to read it yourself, run "
                     f"AFC_BAMBU_SCAN LANE=lane15 once the print is done")]

    @pytest.mark.parametrize("path", ["create", "match"])
    @pytest.mark.parametrize("lands", ["bay-empty", "after-defaults"])
    def test_a_link_that_lands_after_its_spool_left_stays_undone(
            self, lands, path, monkeypatch):
        # A mid-print flap relinks lane15; the reel is then really pulled
        # while Spoolman answers. The link is undone when it lands, and the
        # unread reel that follows stays on defaults, linked to nothing.
        unit = self._flap_unit(monkeypatch)
        lane, sp = unit.lanes["lane15"], unit._measure
        answers: List[Any] = []
        if path == "create":
            unit.auto_spoolman_create = True

            def _bind(afc: Any, ln: Any, si: Any, logger: Any, prefix: str,
                      allow_create: bool = False, reactor: Any = None,
                      on_done: Any = None) -> None:
                def _answer() -> None:
                    ln.spool_id, ln.material, ln.color = 136, "PLA", "#0086D6"
                    on_done()
                answers.append(_answer)
            monkeypatch.setattr(rfid_mod, "sync_rfid_to_spoolman", _bind)
        else:
            answers.append(unit.afc.spool.land)
        self._feed(unit, self._frame(False))
        self._feed(unit, self._frame(True))
        assert lane.spool_id == "" and sp._bind_pending == {3}
        gone = self._frame(False, tagged=False)
        unread = self._frame(True, tagged=False)
        self._feed(unit, gone)
        expected = self._flap_lines(matched=path == "match")
        expected.append(
            ("info", f"{self.FLAP}: spool REMOVED from slot 3 (AMS bay 4)"))
        if lands == "after-defaults":
            self._feed(unit, unread, dt=1.0, times=4)
            expected += self._unread_lines()
            # The link lands on a lane already back on defaults.
            assert self._profile(lane) == {
                "spool_id": "", "material": "PLA", "color": "",
                "sub_type": "", "filament_name": "", "spool_vendor": "",
                "weight": 1000}
        answers.pop()()
        expected.append(
            ("debug", f"{self.FLAP}: unbinding lane15 from spool 136 -- its "
                      f"Spoolman link for tag 95F2C30C landed after that "
                      f"spool left the bay"))
        assert lane.spool_id == ""
        assert sp._bound_uid == {} and sp._bind_pending == set()
        assert sp._spoolman_no_match == set()
        if lands == "bay-empty":
            assert lane.material == ""
            self._feed(unit, gone, dt=1.0, times=2)
            self._feed(unit, unread, dt=1.0, times=8)
            expected += self._unread_lines()
        else:
            self._feed(unit, unread, dt=1.0, times=8)
        assert self._profile(lane) == {
            "spool_id": "", "material": "PLA", "color": "", "sub_type": "",
            "filament_name": "", "spool_vendor": "", "weight": 1000}
        assert unit.logger.messages == expected
        assert unit._bridge.sent == []
        assert [c[:2] for c in unit.afc.spool.calls] == (
            [("set_spoolID", (lane, 136))] if path == "match" else [])

    @pytest.mark.parametrize("lands", ["while-booting", "after-first-poll"])
    def test_a_link_in_flight_across_a_bridge_reboot_stays_kept(
            self, lands, monkeypatch):
        # The relink after a flap is still being fetched when the bridge
        # reboots. Booting frames keep the bay's old record; once polled the
        # bay is occupied with a blank one. The reel never left either way.
        unit = self._flap_unit(monkeypatch)
        lane, sp = unit.lanes["lane15"], unit._measure
        self._feed(unit, self._frame(False))
        self._feed(unit, self._frame(True))
        assert len(unit.afc.spool.fetching) == 1
        unit._on_bridge_reconnect()
        self._feed(unit, self._frame(False, tagged=False, polled=False),
                   times=7)
        assert unit._slots[3]["rfid_uid"] == self.FLAP_UID
        polled = self._frame(True, tagged=False, polled=True)
        if lands == "after-first-poll":
            self._feed(unit, polled, dt=1.0, times=3)
            assert unit._slots[3]["present"] is True
            assert unit._slots[3]["rfid_uid"] is None
        unit.afc.spool.land()
        self._feed(unit, polled, dt=1.0, times=5)
        assert self._profile(lane) == {
            "spool_id": 136, "material": "PLA", "color": "#0086D6",
            "sub_type": "Basic", "filament_name": "Bambu PLA Basic",
            "spool_vendor": "Bambu", "weight": 670.8}
        assert sp._bound_uid == {3: self.FLAP_UID}
        assert sp._bind_pending == set()
        assert unit.logger.messages == self._flap_lines(matched=True) + [
            ("debug", f"{self.FLAP}: the bridge has not polled this unit's "
                      f"bays yet; keeping the lanes as they are")]
        # The reconnect's info requests, announce and status request; no
        # scan and no capscan.
        assert [f["cmd"] for f in unit._bridge.sent] == [
            "info", "info", "units", "htunit", "capen", "model", "mcaddr",
            "armms", "status"]

    @pytest.mark.parametrize("miss", [True, False])
    def test_an_answer_that_bound_nothing_releases_its_claim(
            self, miss, monkeypatch):
        unit = self._landed_unit(monkeypatch, slot_info(3, uid=self.UID))
        sp, lane = unit._measure, unit.lanes["lane15"]
        lane.spool_id = None
        sp._bind_owed = {3: (74, 1000, None, True), 1: (60, 1000, None, True)}
        sp._bind_landed(lane, 3, self.UID, miss=miss)
        assert sp._bind_owed == {1: (60, 1000, None, True)}
        assert sp._spoolman_no_match == ({self.UID} if miss else set())
        assert sp._bound_uid == {} and sp._bind_pending == set()
        assert unit.logger.messages == []


class TestBambuSpoolmanSpoolmanSync:
    """The bay's bind follows its tag, and every exit answers the summary."""

    class _CountingClient(FakeSpoolmanClient):
        """Spoolman, counting the full-table lookups it is asked for."""

        def __init__(self, spools: Sequence[Dict[str, Any]] = ()) -> None:
            """:param spools: Spoolman's spool records"""
            super().__init__(spools)
            self.searches = 0

        def search_spools(self) -> List[Dict[str, Any]]:
            """:return list: every spool, counting the call"""
            self.searches += 1
            return super().search_spools()

    class _VanishingRecord(dict):
        """A record that stops answering part-way through the pass."""

        def get(self, key: Any, default: Any = None) -> Any:
            """
            :param key: the field asked for
            :param default: what a missing field reads as
            :return Any: the field, unless it is ``present``
            """
            if key == "present":
                raise RuntimeError("the record went away mid-pass")
            return dict.get(self, key, default)

    @staticmethod
    def _sync_unit(monkeypatch: pytest.MonkeyPatch, record: Dict[str, Any], *,
                   lane: str = "lane1", spool_id: Optional[int] = None,
                   client: Optional[FakeSpoolmanClient] = None,
                   spoolman: Optional[str] = BRFID_SPOOLMAN_URL
                   ) -> afcBambuAMS:
        """
        A unit whose bay 0 carries ``record``, mapped to ``lane``.

        :param monkeypatch: pytest's monkeypatch fixture
        :param record: bay 0's record
        :param lane: bay 0's lane
        :param spool_id: the lane's Spoolman binding
        :param client: Spoolman
        :param spoolman: AFC core's Spoolman setting
        :return afcBambuAMS: the unit
        """
        return brfid_unit(monkeypatch, client=client, spoolman=spoolman,
                          lanes=[LaneSpec(lane, 0, spool_id=spool_id)],
                          slots=[record])

    def _sync(self, unit: afcBambuAMS, record: Dict[str, Any],
              lane: str = "lane1", restored: bool = False) -> None:
        """
        One pass of the bind for bay 0, then whatever it queued on the
        reactor.

        :param unit: the unit
        :param record: the record the pass is given
        :param lane: the lane name
        :param restored: the restored-lane lookup
        """
        unit._measure._spoolman_sync(unit.lanes[lane], record,
                                     restored=restored)
        unit.afc.reactor.run_callbacks()

    def test_a_uid_only_miss_is_remembered(self, monkeypatch):
        # A UID Spoolman does not know is an answer, not a retry: this runs
        # on every status pass and re-querying starves MCU clock sync.
        record = slot_info(0, uid="deadbeef")
        unit = self._sync_unit(monkeypatch, record)
        self._sync(unit, record)
        sp = unit._measure
        assert sp._spoolman_no_match == {"deadbeef"}
        assert sp._bind_pending == set() and sp._spoolman_inflight == set()
        assert unit.afc.spool.calls == []
        assert unit.logger.messages == []

    def test_the_memo_short_circuits_the_next_pass(self, monkeypatch):
        client = self._CountingClient()
        record = slot_info(0, uid="deadbeef")
        unit = self._sync_unit(monkeypatch, record, client=client)
        for _ in range(3):
            self._sync(unit, record)
        assert client.searches == 1
        assert unit._measure._spoolman_no_match == {"deadbeef"}
        assert unit.logger.messages == []

    def test_an_empty_bay_never_binds(self, monkeypatch):
        # The leftover UID must not re-bind the lane the instant it unbinds.
        client = self._CountingClient([brfid_spool(87, uids=["0A1882AC"])])
        record = slot_info(0, present=False, uid="0a1882ac")
        unit = self._sync_unit(monkeypatch, record, lane="lane23",
                               client=client)
        self._sync(unit, record, "lane23")
        assert unit.lanes["lane23"].spool_id is None
        assert client.searches == 0 and unit.afc.spool.calls == []
        assert unit.logger.messages == []

    def test_the_same_tag_on_a_bound_lane_is_a_no_op(self, monkeypatch):
        # Spoolman says spool 87 carries another tag, so only the memo keeps
        # the pass from asking it and unbinding.
        client = self._CountingClient([brfid_spool(87, uids=["11223344"])])
        record = slot_info(0, uid="0a1882ac")
        unit = self._sync_unit(monkeypatch, record, lane="lane23",
                               spool_id=87, client=client)
        unit._measure._bound_uid = {0: "0a1882ac"}
        self._sync(unit, record, "lane23")
        assert unit.lanes["lane23"].spool_id == 87
        assert unit._measure._binding_check == {}
        assert client.searches == 0
        assert unit.logger.messages == []

    def test_a_different_tag_releases_the_old_binding(self, monkeypatch):
        # Spoolman is not configured, so the pass ends right after the
        # unbind: the stale link does not survive it.
        record = slot_info(0, uid="01d0ec0f")
        unit = self._sync_unit(monkeypatch, record, lane="lane23",
                               spool_id=87, spoolman=None)
        unit._measure._bound_uid = {0: "0a1882ac"}
        self._sync(unit, record, "lane23")
        assert unit.lanes["lane23"].spool_id == ""
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: unbinding lane23 from spool 87 "
                      "-- tag 01D0EC0F is in this bay now, not 0A1882AC")]

    def test_a_binding_we_did_not_make_is_left_alone(self, monkeypatch):
        # No recorded UID and nothing in Spoolman to say otherwise: a manual
        # or restored assignment, not ours to revoke.
        client = self._CountingClient()
        record = slot_info(0, uid="01d0ec0f")
        unit = self._sync_unit(monkeypatch, record, lane="lane23",
                               spool_id=42, client=client)
        self._sync(unit, record, "lane23")
        assert unit.lanes["lane23"].spool_id == 42
        assert unit._measure._binding_check == {("42", "01d0ec0f"): False}
        assert client.searches == 0
        assert unit.logger.messages == []

    def test_a_restart_does_not_preserve_a_stale_binding(self, monkeypatch):
        # No memo after a restart, but Spoolman says spool 87 carries the
        # Glow's tag, not the one in this bay.
        client = FakeSpoolmanClient([brfid_spool(87, uids=["0A1882AC"])])
        record = slot_info(0, uid="ecb61cd0")
        unit = self._sync_unit(monkeypatch, record, lane="lane23",
                               spool_id=87, client=client, spoolman=None)
        self._sync(unit, record, "lane23")
        assert unit.lanes["lane23"].spool_id == ""
        # No remembered tag to name, so the reason cites Spoolman instead.
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: unbinding lane23 from spool 87 "
                      "-- tag ECB61CD0 is in this bay now, and Spoolman does "
                      "not list it on that spool")]

    def test_a_spool_with_no_recorded_uid_still_survives_a_restart(
            self, monkeypatch):
        client = FakeSpoolmanClient([brfid_spool(42)])
        record = slot_info(0, uid="ecb61cd0")
        unit = self._sync_unit(monkeypatch, record, lane="lane23",
                               spool_id=42, client=client)
        self._sync(unit, record, "lane23")
        assert unit.lanes["lane23"].spool_id == 42
        assert unit._measure._binding_check == {("42", "ecb61cd0"): False}
        assert unit.logger.messages == []

    def test_a_miss_is_remembered(self, monkeypatch):
        # The full-profile branch: a decoded tag, auto-create off.
        record = slot_info(0, material="PLA Basic", uid="ecb61cd0",
                           weight=1000)
        unit = self._sync_unit(monkeypatch, record)
        self._sync(unit, record)
        assert unit._measure._spoolman_no_match == {"ecb61cd0"}
        assert unit.afc.spool.calls == []
        assert unit.logger.messages == []

    def test_a_different_uid_is_not_suppressed(self, monkeypatch):
        # Keyed by UID, so another spool still gets its own lookup.
        client = self._CountingClient()
        record = slot_info(0, material="PLA Basic", uid="13f56d32",
                           weight=1000)
        unit = self._sync_unit(monkeypatch, record, client=client)
        unit._measure._spoolman_no_match = {"ecb61cd0"}
        self._sync(unit, record)
        assert client.searches == 1
        assert unit._measure._spoolman_no_match == {"ecb61cd0", "13f56d32"}
        assert unit.logger.messages == []

    def test_a_full_tag_with_auto_create_goes_to_the_shared_binder(
            self, monkeypatch):
        # The create path hands the whole bind to AFC_RFID's binder, off the
        # reactor, and settles it in _bind_landed when it answers.
        calls: List[Tuple[tuple, dict]] = []
        monkeypatch.setattr(rfid_mod, "sync_rfid_to_spoolman",
                            lambda *a, **k: calls.append((a, k)))
        record = slot_info(0, material="PLA Basic", color="0086D6FF",
                           uid="ecb61cd0", weight=1000, tmin=190, tmax=230,
                           tray_uid=BRFID_TRAY)
        unit = self._sync_unit(monkeypatch, record)
        unit.auto_spoolman_create = True
        sp, lane = unit._measure, unit.lanes["lane1"]
        sp._bind_owed = {0: (74, 1000, None, False)}
        self._sync(unit, record)
        assert len(calls) == 1
        args, kwargs = calls[0]
        assert args[:2] == (unit.afc, lane)
        assert args[2] == {
            "uid": "ecb61cd0", "brand": "Bambu", "material": "PLA",
            "sub_type": "Basic", "color_hex": "0086D6", "diameter": 1.75,
            "extruder_temp": 190, "extruder_temp_min": 190,
            "extruder_temp_max": 230, "weight_g": 1000,
            "tray_uid": "cf34cf1d212f46b5bc8561e05eb644c8"}
        assert isinstance(args[3], _QuietInfo) and args[4] == "Bambu RFID"
        assert set(kwargs) == {"allow_create", "reactor", "on_done"}
        assert kwargs["allow_create"] is True
        assert kwargs["reactor"] is unit.afc.reactor
        # The claim is now owed to this bind, which is out.
        assert sp._bind_owed == {0: (74, 1000, None, True)}
        assert sp._bind_pending == {0}
        kwargs["on_done"]()
        assert sp._bind_pending == set()
        assert sp._spoolman_no_match == {"ecb61cd0"}
        assert sp._bind_owed == {}
        assert unit.logger.messages == []

    def test_a_restored_lane_is_only_ever_matched(self, monkeypatch):
        # A lookup for a lane AFC restored never creates, whatever
        # auto_spoolman_create says.
        calls: List[Tuple[tuple, dict]] = []
        monkeypatch.setattr(rfid_mod, "sync_rfid_to_spoolman",
                            lambda *a, **k: calls.append((a, k)))
        client = self._CountingClient()
        record = slot_info(0, material="PLA Basic", uid="ecb61cd0",
                           weight=1000)
        unit = self._sync_unit(monkeypatch, record, client=client)
        unit.auto_spoolman_create = True
        self._sync(unit, record, restored=True)
        assert calls == []
        assert client.searches == 1
        assert unit._measure._spoolman_no_match == {"ecb61cd0"}
        assert unit.logger.messages == []

    def test_a_uid_only_tag_is_only_matched_even_with_auto_create(
            self, monkeypatch):
        # No decoded profile: nothing to create a spool from, whatever
        # auto_spoolman_create says.
        calls: List[Tuple[tuple, dict]] = []
        monkeypatch.setattr(rfid_mod, "sync_rfid_to_spoolman",
                            lambda *a, **k: calls.append((a, k)))
        client = self._CountingClient()
        record = slot_info(0, uid="deadbeef")
        unit = self._sync_unit(monkeypatch, record, client=client)
        unit.auto_spoolman_create = True
        self._sync(unit, record)
        assert calls == []
        assert client.searches == 1
        assert unit._measure._spoolman_no_match == {"deadbeef"}
        assert unit.logger.messages == []

    def test_without_the_shared_binder_a_full_tag_is_only_matched(
            self, monkeypatch):
        monkeypatch.setattr(rfid_mod, "sync_rfid_to_spoolman", None)
        client = self._CountingClient()
        record = slot_info(0, material="PLA Basic", uid="ecb61cd0",
                           weight=1000)
        unit = self._sync_unit(monkeypatch, record, client=client)
        unit.auto_spoolman_create = True
        self._sync(unit, record)
        assert client.searches == 1
        assert unit._measure._spoolman_no_match == {"ecb61cd0"}
        assert unit._measure._bind_pending == set()
        assert unit.logger.messages == []

    @pytest.mark.parametrize("sent", [False, True])
    def test_a_pass_that_sends_nothing_drops_an_unsent_claim(
            self, sent, monkeypatch):
        # Nothing of ours is coming for this scan, so a claim not yet sent
        # is dropped; one already sent is left to its own bind's answer.
        record = slot_info(0, uid="c32a080a")
        unit = self._sync_unit(monkeypatch, record, spoolman=None)
        unit._measure._bind_owed = {0: (74, 1000, None, sent)}
        self._sync(unit, record)
        assert unit._measure._bind_owed == (
            {0: (74, 1000, None, True)} if sent else {})
        assert unit.logger.messages == []

    # A bay mid-scan with its summary held: the record is complete, the
    # latch already set (the unit sets it just before the call), and the
    # pass through the bind is all that is left.

    def _held(self, monkeypatch: pytest.MonkeyPatch,
              outstanding: Sequence[int] = ()) -> afcBambuAMS:
        """
        The held-summary unit; Spoolman is not configured, so the bind
        returns early.

        :param monkeypatch: pytest's monkeypatch fixture
        :param outstanding: bays whose bind is still out
        :return afcBambuAMS: the unit
        """
        record = slot_info(0, material="ABS", uid="c32a080a")
        unit = self._sync_unit(monkeypatch, record, lane="lane12",
                               spoolman=None)
        unit._spoolman_latched = {0}
        unit._scanned_bays = {0}
        sp = unit._measure
        sp._pending_summary = {0: (138, 955, 1000, 200.0)}
        sp._bind_pending = set(outstanding)
        return unit

    def test_a_pass_that_starts_no_lookup_releases_it(self, monkeypatch):
        unit = self._held(monkeypatch)
        self._sync(unit, dict(unit._slots[0]), "lane12")
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [("info", BRFID_ABS_HELD_LINE)]

    def test_it_leaves_an_outstanding_lookup_to_its_own_callback(
            self, monkeypatch):
        # Draining here would speak while the lookup it waits on still runs.
        unit = self._held(monkeypatch, outstanding=(0,))
        self._sync(unit, dict(unit._slots[0]), "lane12")
        assert unit._measure._pending_summary == {0: (138, 955, 1000, 200.0)}
        assert unit.logger.messages == []

    def test_a_raise_inside_the_bind_still_releases_it(self, monkeypatch):
        unit = self._held(monkeypatch)
        with pytest.raises(RuntimeError):
            unit._measure._spoolman_sync(
                unit.lanes["lane12"],
                self._VanishingRecord(index=0, rfid_uid="c32a080a"))
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [("info", BRFID_ABS_HELD_LINE)]


class TestBambuSpoolmanApplyRemainWeight:
    """A measurement is written once; only what it still owes is finished."""

    @staticmethod
    def _owing_unit(monkeypatch: pytest.MonkeyPatch, client: FakeSpoolmanClient,
                    lane: str, slot: int, *, spool_id: Optional[int] = None,
                    weight: float = 0.0, material: Optional[str] = None,
                    remain: Optional[int] = None,
                    extras: Optional[Dict[str, Any]] = None) -> afcBambuAMS:
        """
        A Spoolman-on unit whose bay ``slot`` holds an untagged-profile 1 kg
        reel, mapped to ``lane``.

        :param monkeypatch: pytest's monkeypatch fixture
        :param client: Spoolman
        :param lane: the lane's name
        :param slot: its bay
        :param spool_id: the lane's Spoolman binding
        :param weight: the lane's grams
        :param material: the bay record's material (None: no density known)
        :param remain: the percent written on the tag
        :param extras: further lane attributes (density...)
        :return afcBambuAMS: the unit
        """
        slots: List[Dict[str, Any]] = [{} for _ in range(slot)]
        slots.append(slot_info(slot, material=material, weight=1000,
                               remain=remain))
        return brfid_unit(
            monkeypatch, client=client, slots=slots,
            lanes=[LaneSpec(lane, slot, spool_id=spool_id, weight=weight,
                            extras=dict(extras or {}))])

    def test_apply_remain_weight_never_rewrites_a_held_measurement(
            self, monkeypatch):
        # A measurement merely held is owed nothing: without Spoolman there is
        # no bind, and a write would undo AFC's own consumption count.
        unit = brfid_unit(
            monkeypatch, section=None,
            lanes=[LaneSpec("lane8", 1, weight=999.0, material="PLA",
                            extras={"sub_type": "Basic"})],
            slots=[slot_info(0, present=False),
                   slot_info(1, material="PLA Basic", uid="d13fdb0e",
                             weight=1000, remain=80)])
        sp, lane = unit._measure, unit.lanes["lane8"]
        assert sp._adopt_measured_remain(1, 63, "capscan", seq=1) is True
        lane.weight = 450                    # consumed since
        unit.logger.messages.clear()
        saves = unit.afc.save_vars.call_count
        sp._apply_remain_weight(lane, {"index": 1, "weight": 1000})
        assert lane.weight == 450
        assert sp._bind_owed == {} and sp._convert_owed == {}
        assert unit.afc.save_vars.call_count == saves
        assert unit.logger.messages == []

    def test_a_bind_that_lands_later_gets_the_same_weighed_figure(
            self, monkeypatch):
        # The lane bound after its measurement is handed it through the same
        # _grams_for: 1.04 g/cm3 x 663 cm3 x 138% = 951.5, so 952 g, not
        # tag-linear and not Spoolman's stored 998.
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane12", 0,
                                material="ABS", extras={"density": 1.04})
        sp, lane = unit._measure, unit.lanes["lane12"]
        sp._bind_owed = {0: (138, 1000, None, True)}   # the bind was sent
        lane.spool_id, lane.weight = 163, 998
        sp._apply_remain_weight(lane, {"index": 0, "weight": 1000})
        assert lane.weight == 952
        assert sp._bind_owed == {}
        assert client.calls == [("set_remaining_weight", (163, 952.0))]
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: lane12 bound to spool 163 after "
                      "its measurement; handing the measured 952 g to it"),
            ("info", "AFC bambu Bambu_AMS_1: wrote 952.0 g remaining to "
                     "Spoolman spool 163 (physical AMS measurement) -- "
                     "measured 138%, capped to the spool's 1000 g nominal")]
        # Handed over once, not per frame.
        unit.logger.messages.clear()
        sp._apply_remain_weight(lane, {"index": 0, "weight": 1000})
        assert client.calls == [("set_remaining_weight", (163, 952.0))]
        assert lane.weight == 952
        assert sp._bind_owed == {}
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == []

    def test_a_real_measurement_says_so(self, monkeypatch):
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane23", 0,
                                spool_id=41, remain=100)
        sp, lane = unit._measure, unit.lanes["lane23"]
        sp._measured_remain = {0: 64}
        sp._bind_owed = {0: (64, 1000, None, True)}
        sp._apply_remain_weight(lane, {"index": 0, "remain_pct": 100,
                                       "weight": 1000})
        assert lane.weight == 640
        assert client.calls == [("set_remaining_weight", (41, 640.0))]
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: lane23 bound to spool 41 after "
                      "its measurement; handing the measured 640 g to it"),
            ("info", "AFC bambu Bambu_AMS_1: wrote 640.0 g remaining to "
                     "Spoolman spool 41 (physical AMS measurement)")]

    def test_a_tag_record_is_never_written_at_all(self, monkeypatch):
        # The tag's stored remain does not track printing: on a reel holding
        # 230 g it still reads 80%.
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane23", 0,
                                spool_id=41, remain=100)
        sp, lane = unit._measure, unit.lanes["lane23"]
        sp._apply_remain_weight(lane, {"index": 0, "remain_pct": 100,
                                       "weight": 1000})
        assert lane.weight == 0.0
        assert client.calls == [] and unit.logger.messages == []

    def test_a_held_measurement_alone_is_not_written_again(self, monkeypatch):
        # Holding the measurement is its identity, not a claim on the spool.
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane23", 0,
                                spool_id=41, remain=80)
        sp, lane = unit._measure, unit.lanes["lane23"]
        sp._measured_remain = {0: 64}
        sp._apply_remain_weight(lane, {"index": 0, "remain_pct": 80,
                                       "weight": 1000})
        assert lane.weight == 0.0
        assert client.calls == [] and unit.logger.messages == []

    def test_a_measurement_is_written_and_named(self, monkeypatch):
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane23", 0,
                                spool_id=41, remain=80)
        sp, lane = unit._measure, unit.lanes["lane23"]
        sp._measured_remain = {0: 23}
        sp._bind_owed = {0: (23, 1000, None, True)}
        sp._apply_remain_weight(lane, {"index": 0, "remain_pct": 80,
                                       "weight": 1000})
        assert lane.weight == 230
        assert client.calls == [("set_remaining_weight", (41, 230.0))]
        assert unit.logger.messages == [
            ("debug", "AFC bambu Bambu_AMS_1: lane23 bound to spool 41 after "
                      "its measurement; handing the measured 230 g to it"),
            ("info", "AFC bambu Bambu_AMS_1: wrote 230.0 g remaining to "
                     "Spoolman spool 41 (physical AMS measurement)")]

    def test_a_fresh_measurement_wins_over_the_tag_record(self, monkeypatch):
        # The tag says 70%; the measurement says 69%, and no later frame puts
        # the tag's 700 g back. No density is known: 69% of 1000 g is 690 g.
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane15", 2,
                                spool_id=87, remain=70)
        sp, lane = unit._measure, unit.lanes["lane15"]
        assert sp._adopt_measured_remain(2, 69, "capscan", seq=1) is True
        assert lane.weight == 690
        assert client.calls == [("set_remaining_weight", (87, 690.0))]
        unit.logger.messages.clear()
        sp._apply_remain_weight(lane, {"index": 2, "remain_pct": 70,
                                       "weight": 1000})
        assert lane.weight == 690
        assert client.calls == [("set_remaining_weight", (87, 690.0))]
        # The grams still wait for a material to be weighed at.
        assert sp._convert_owed == {2: (69, 1000, 690)}
        assert unit.logger.messages == []

    def test_it_is_the_right_slots_measurement(self, monkeypatch):
        # Keyed by slot: bay 3's claim does not describe bay 2's spool.
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane15", 2,
                                spool_id=87, weight=1000.0, remain=70)
        sp, lane = unit._measure, unit.lanes["lane15"]
        sp._bind_owed = {3: (69, 1000, None, True)}
        sp._apply_remain_weight(lane, {"index": 2, "remain_pct": 70,
                                       "weight": 1000})
        assert lane.weight == 1000.0
        assert sp._bind_owed == {3: (69, 1000, None, True)}
        assert client.calls == [] and unit.logger.messages == []

    def test_no_measurement_means_no_weight_is_written(self, monkeypatch):
        # The tag's stored remain is not a weight source: without a
        # measurement the lane keeps what it had.
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane15", 2,
                                spool_id=87, weight=1000.0, remain=70)
        sp, lane = unit._measure, unit.lanes["lane15"]
        sp._apply_remain_weight(lane, {"index": 2, "remain_pct": 70,
                                       "weight": 1000})
        assert lane.weight == 1000.0
        assert client.calls == [] and unit.logger.messages == []

    def test_a_loaded_lane_is_left_alone(self, monkeypatch):
        # Extrusion owns the weight, and a fed spool no longer holds what was
        # measured: whatever it was owed is dropped, not kept for later.
        client = FakeSpoolmanClient()
        unit = self._owing_unit(monkeypatch, client, "lane15", 2,
                                spool_id=87, remain=119)
        sp, lane = unit._measure, unit.lanes["lane15"]
        lane.tool_loaded = True
        sp._bind_owed = {2: (119, 1000, None, True)}
        sp._convert_owed = {2: (119, 1000, 1000)}
        sp._apply_remain_weight(lane, {"index": 2, "remain_pct": 119,
                                       "weight": 1000})
        assert lane.weight == 0.0
        assert sp._bind_owed == {} and sp._convert_owed == {}
        assert client.calls == [] and unit.logger.messages == []


class TestBambuSpoolmanQueueSpoolSummary:
    """The summary waits for the record it describes, then speaks."""

    @staticmethod
    def _queue_unit(monkeypatch: pytest.MonkeyPatch,
                    material: Optional[str] = None,
                    uid: Optional[str] = None, **fields: Any) -> afcBambuAMS:
        """
        Unit BambuAMS_1, its bay 0 mapped to lane16, Spoolman not configured.
        Its clock reads 100.0, so a summary queued now is held until 145.0.

        :param monkeypatch: pytest's monkeypatch fixture
        :param material: the record's material
        :param uid: the record's chip UID
        :param fields: further wire fields for the record (sseq, sres...)
        :return afcBambuAMS: the unit
        """
        record = slot_info(0, material=material, color="2D2B28FF", uid=uid,
                           weight=1000, **fields)
        return brfid_unit(monkeypatch, "BambuAMS_1", spoolman=None,
                          lanes=[LaneSpec("lane16", 0)], slots=[record])

    #: The line for a third-party tag on lane16, 25% of a 1 kg spool.
    FOREIGN_LINE = (
        "BambuAMS_1 lane16: tag 84EA7601 read but its profile could not be "
        "decoded (not a Bambu tag?) -- bind that UID to a spool in Spoolman "
        "and it will match from now on. Measured about 25% left -- roughly "
        "250 g of a 1000 g spool; not linked to a Spoolman spool, so this is "
        "kept on the lane only -- bind 84EA7601 to a spool in Spoolman to "
        "track this reel.")

    def test_a_blank_record_holds_the_line(self, monkeypatch):
        unit = self._queue_unit(monkeypatch)
        unit._measure._queue_spool_summary(0, 25, 250, 1000)
        assert unit._measure._pending_summary == {0: (25, 250, 1000, 145.0)}
        assert unit.logger.messages == []

    def test_a_lost_pending_memo_is_rebuilt(self, monkeypatch):
        unit = self._queue_unit(monkeypatch)
        unit._measure._pending_summary = None
        unit._measure._queue_spool_summary(0, 25, 250, 1000)
        assert unit._measure._pending_summary == {0: (25, 250, 1000, 145.0)}
        assert unit.logger.messages == []

    def test_a_record_that_already_answers_is_not_held(self, monkeypatch):
        unit = self._queue_unit(monkeypatch, material="Bambu PLA Sparkle")
        unit._measure._queue_spool_summary(0, 25, 250, 1000)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [
            ("info", "BambuAMS_1 lane16: tag read: Bambu PLA Sparkle "
                     "(2D2B28). Measured about 25% left -- roughly 250 g of a "
                     "1000 g spool; not linked to a Spoolman spool, so this "
                     "is kept on the lane only.")]

    def test_a_uid_alone_answers_it(self, monkeypatch):
        # A third-party tag has a UID even when its profile will not decode.
        unit = self._queue_unit(monkeypatch, uid="84ea7601")
        unit._measure._queue_spool_summary(0, 25, 250, 1000)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [("info", self.FOREIGN_LINE)]

    def test_a_uid_alone_is_not_an_answer_while_the_scan_is_still_running(
            self, monkeypatch):
        # On an HT the UID comes off the anticollision well before the
        # profile: a scan still open says nothing about the tag yet.
        unit = self._queue_unit(monkeypatch, uid="0a1882ac")
        unit._scan_t0[0] = 100.0
        unit._measure._queue_spool_summary(0, 74, 740, 1000)
        assert unit._measure._pending_summary == {0: (74, 740, 1000, 145.0)}
        assert unit.logger.messages == []

    def test_a_uid_only_bay_whose_scan_ENDED_still_answers_at_once(
            self, monkeypatch):
        # A confirmed no-tag verdict means the profile is never coming.
        unit = self._queue_unit(monkeypatch, uid="84ea7601", sseq=1, sres=3)
        unit._scan_t0[0] = 100.0
        unit._cycle_end_seen = {0: True}
        unit._measure._queue_spool_summary(0, 25, 250, 1000)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [("info", self.FOREIGN_LINE)]

    def test_a_verdict_that_raises_keeps_the_old_behaviour(self, monkeypatch):
        # A unit that cannot answer for its scan does not hold the line.
        unit = self._queue_unit(monkeypatch, uid="84ea7601")

        def _no_verdict(slot: Optional[int]) -> str:
            """:raises RuntimeError: always"""
            raise RuntimeError("no verdict")

        unit._scan_verdict = _no_verdict
        unit._measure._queue_spool_summary(0, 25, 250, 1000)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [("info", self.FOREIGN_LINE)]

    @pytest.mark.parametrize("latched", [False, True])
    def test_a_fresh_measurement_voids_a_miss_still_to_be_asked_again(
            self, latched, monkeypatch):
        # A miss judges one lookup, not the UID for ever; once the bay's
        # lookup has gone out on this connection the miss stands.
        unit = self._queue_unit(monkeypatch, uid="84ea7601")
        unit._scan_t0[0] = 100.0                 # held, so nothing is said
        if latched:
            unit._spoolman_latched = {0}
        sp = unit._measure
        sp._spoolman_no_match = {"84EA7601", "13f56d32"}
        sp._queue_spool_summary(0, 25, 250, 1000)
        assert sp._spoolman_no_match == (
            {"84EA7601", "13f56d32"} if latched else {"13f56d32"})
        assert unit.logger.messages == []


class TestBambuSpoolmanDrainSpoolSummary:
    """A held summary is said once the bay can answer it, or the backstop."""

    @staticmethod
    def _sparkle_unit(monkeypatch: pytest.MonkeyPatch,
                      uid: Optional[str] = None) -> afcBambuAMS:
        """
        Unit BambuAMS_1, bay 0 mapped to lane16 with a blank record (a
        sparkle reel being re-read), Spoolman not configured.

        :param monkeypatch: pytest's monkeypatch fixture
        :param uid: the chip UID already on the record
        :return afcBambuAMS: the unit
        """
        record = slot_info(0, color="2D2B28FF", uid=uid, weight=1000)
        return brfid_unit(monkeypatch, "BambuAMS_1", spoolman=None,
                          lanes=[LaneSpec("lane16", 0)], slots=[record])

    def test_it_speaks_once_the_record_lands(self, monkeypatch):
        unit = self._sparkle_unit(monkeypatch)
        sp = unit._measure
        sp._queue_spool_summary(0, 25, 250, 1000)
        assert sp._pending_summary == {0: (25, 250, 1000, 145.0)}
        unit._slots[0]["material"] = "Bambu PLA Sparkle"      # the re-read
        sp._drain_spool_summary(0)
        assert sp._pending_summary == {}
        assert unit.logger.messages == [
            ("info", "BambuAMS_1 lane16: tag read: Bambu PLA Sparkle "
                     "(2D2B28). Measured about 25% left -- roughly 250 g of a "
                     "1000 g spool; not linked to a Spoolman spool, so this "
                     "is kept on the lane only.")]

    def test_and_it_speaks_when_the_scan_resolves(self, monkeypatch):
        unit = self._sparkle_unit(monkeypatch, uid="0a1882ac")
        unit._scan_t0[0] = 100.0
        sp = unit._measure
        sp._queue_spool_summary(0, 74, 740, 1000)
        assert sp._pending_summary == {0: (74, 740, 1000, 145.0)}
        unit._slots[0]["material"] = "Bambu PLA Glow"          # the read
        unit._bridge.rfid_ok[unit.dry_dev_addr] = 101.0
        sp._drain_spool_summary(0)
        assert sp._pending_summary == {}
        assert unit.logger.messages == [
            ("info", "BambuAMS_1 lane16: tag read: Bambu PLA Glow (2D2B28) "
                     "[tag 0A1882AC]. Measured about 74% left -- roughly 740 "
                     "g of a 1000 g spool; no Spoolman lookup has been made "
                     "for 0A1882AC, so the measurement stays on the lane "
                     "only -- run AFC_BAMBU_SCAN LANE=lane16 to link it.")]

    def test_the_backstop_stops_it_waiting_forever(self, monkeypatch):
        unit = self._sparkle_unit(monkeypatch)
        sp = unit._measure
        sp._queue_spool_summary(0, 25, 250, 1000)
        sp._drain_spool_summary(0)
        assert unit.logger.messages == []
        unit.afc.reactor.advance(46.0)
        sp._drain_spool_summary(0)
        assert sp._pending_summary == {}
        assert unit.logger.messages == [
            ("info", "BambuAMS_1 lane16: no tag on this spool. Measured "
                     "about 25% left -- roughly 250 g of a 1000 g spool; not "
                     "linked to a Spoolman spool, so this is kept on the "
                     "lane only.")]

    # The record is complete while the Spoolman answer the sentence depends
    # on is still a round-trip away.

    #: lane19's line once nothing about its lookup is outstanding.
    MATTE_UNASKED = (
        "BambuAMS_2 lane19: tag read: PLA Matte [tag ECB61CD0]. Measured "
        "about 91% left -- roughly 910 g of a 1000 g spool; no Spoolman "
        "lookup has been made for ECB61CD0, so the measurement stays on the "
        "lane only -- run AFC_BAMBU_SCAN LANE=lane19 to link it.")
    #: lane19's line said over a lookup still out.
    MATTE_IN_PROGRESS = (
        "BambuAMS_2 lane19: tag read: PLA Matte [tag ECB61CD0]. Measured "
        "about 91% left -- roughly 910 g of a 1000 g spool; a Spoolman lookup "
        "for ECB61CD0 is in progress; the measurement is on the lane for "
        "now.")

    @staticmethod
    def _matte_unit(monkeypatch: pytest.MonkeyPatch,
                    inflight: Sequence[str] = (), deadline: float = 200.0,
                    uid: Optional[str] = "ecb61cd0") -> afcBambuAMS:
        """
        Unit BambuAMS_2, bay 0 mapped to lane19 with a PLA Matte reel whose
        summary is held; Spoolman not configured. The clock reads 100.0.

        :param monkeypatch: pytest's monkeypatch fixture
        :param inflight: UIDs whose Spoolman lookup is running
        :param deadline: the held summary's backstop
        :param uid: the record's chip UID
        :return afcBambuAMS: the unit
        """
        record = slot_info(0, material="PLA Matte", uid=uid, weight=1000)
        unit = brfid_unit(monkeypatch, "BambuAMS_2", spoolman=None,
                          lanes=[LaneSpec("lane19", 0)], slots=[record])
        sp = unit._measure
        sp._pending_summary = {0: (91, 910, 1000, deadline)}
        sp._spoolman_inflight = set(inflight)
        return unit

    def test_it_holds_while_the_lookup_is_running(self, monkeypatch):
        unit = self._matte_unit(monkeypatch, inflight=["ecb61cd0"])
        unit._measure._drain_spool_summary(0)
        assert unit._measure._pending_summary == {0: (91, 910, 1000, 200.0)}
        assert unit.logger.messages == []

    def test_it_holds_until_the_binding_answers(self, monkeypatch):
        # The binding's answer, not its in-flight mark: both binders run a
        # completion callback on every path, and that clears _bind_pending.
        unit = self._matte_unit(monkeypatch)
        sp = unit._measure
        sp._bind_pending.add(0)
        sp._drain_spool_summary(0)
        assert sp._pending_summary == {0: (91, 910, 1000, 200.0)}
        assert unit.logger.messages == []
        sp._bind_pending.discard(0)
        sp._drain_spool_summary(0)
        assert sp._pending_summary == {}
        assert unit.logger.messages == [("info", self.MATTE_UNASKED)]

    def test_an_answer_of_no_match_settles_it_too(self, monkeypatch):
        unit = self._matte_unit(monkeypatch)
        sp = unit._measure
        sp._bind_pending.add(0)
        sp._drain_spool_summary(0)
        assert unit.logger.messages == []
        sp._bind_pending.discard(0)             # answered: nothing found
        sp._spoolman_no_match.add("ecb61cd0")
        sp._drain_spool_summary(0)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane19: tag read: PLA Matte [tag ECB61CD0]. "
                     "Measured about 91% left -- roughly 910 g of a 1000 g "
                     "spool; not linked to a Spoolman spool, so this is kept "
                     "on the lane only -- Spoolman has no spool carrying "
                     "ECB61CD0 and auto-create is off for this unit (add "
                     "'auto_spoolman_create: True' to [AFC_BambuAMS "
                     "BambuAMS_2], or bind that UID to an existing spool).")]

    def test_the_deadline_still_bounds_the_wait(self, monkeypatch):
        # A binder that never answers costs a late summary, not a lost one.
        unit = self._matte_unit(monkeypatch, deadline=50.0)
        sp = unit._measure
        sp._bind_pending.add(0)
        sp._drain_spool_summary(0)
        assert sp._pending_summary == {}
        assert unit.logger.messages == [("info", self.MATTE_IN_PROGRESS)]

    def test_case_does_not_defeat_the_check(self, monkeypatch):
        # The record is lower case; the in-flight set may be upper.
        unit = self._matte_unit(monkeypatch, inflight=["ECB61CD0"])
        unit._measure._drain_spool_summary(0)
        assert unit._measure._pending_summary == {0: (91, 910, 1000, 200.0)}
        assert unit.logger.messages == []

    def test_an_unrelated_lookup_does_not_hold_this_bay(self, monkeypatch):
        unit = self._matte_unit(monkeypatch, inflight=["0a1882ac"])
        unit.lanes["lane19"].spool_id = 124
        unit._measure._drain_spool_summary(0)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane19: tag read: PLA Matte [tag ECB61CD0]. "
                     "Measured about 91% left -- roughly 910 g of a 1000 g "
                     "spool; updated Spoolman spool 124.")]

    def test_the_backstop_still_wins(self, monkeypatch):
        unit = self._matte_unit(monkeypatch, inflight=["ecb61cd0"],
                                deadline=50.0)
        unit._measure._drain_spool_summary(0)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [("info", self.MATTE_IN_PROGRESS)]

    def test_no_uid_on_the_record_is_not_held(self, monkeypatch):
        unit = self._matte_unit(monkeypatch, inflight=["ecb61cd0"], uid=None)
        unit._measure._drain_spool_summary(0)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane19: tag read: PLA Matte. Measured about "
                     "91% left -- roughly 910 g of a 1000 g spool; not linked "
                     "to a Spoolman spool, so this is kept on the lane "
                     "only.")]

    # The gap before the bind is dispatched: the measurement reaches the
    # summary from narration, the bind goes out later in the same scan.

    @staticmethod
    def _abs_unit(monkeypatch: pytest.MonkeyPatch, *,
                  latched: Sequence[int] = (), scanned: Sequence[int] = (0,),
                  uid: Optional[str] = "c32a080a",
                  spoolman: Optional[str] = BRFID_SPOOLMAN_URL,
                  deadline: float = 200.0) -> afcBambuAMS:
        """
        Unit Bambu_AMS_1, bay 0 mapped to lane12 with an ABS reel whose
        summary (138%, 955 g) is held. The clock reads 100.0.

        :param monkeypatch: pytest's monkeypatch fixture
        :param latched: bays whose lookup has gone out on this connection
        :param scanned: bays scanned on this connection
        :param uid: the record's chip UID
        :param spoolman: AFC core's Spoolman setting
        :param deadline: the held summary's backstop
        :return afcBambuAMS: the unit
        """
        record = slot_info(0, material="ABS", uid=uid, weight=1000)
        unit = brfid_unit(monkeypatch, spoolman=spoolman,
                          lanes=[LaneSpec("lane12", 0)], slots=[record])
        unit._spoolman_latched = set(latched)
        unit._scanned_bays = set(scanned)
        unit._measure._pending_summary = {0: (138, 955, 1000, deadline)}
        return unit

    #: lane12's line before its bay's lookup has been asked for.
    ABS_UNASKED = (
        "Bambu_AMS_1 lane12: tag read: ABS [tag C32A080A]. Measured full -- "
        "roughly 955 g of a 1000 g spool (the AMS read 138%, meaning it "
        "measures a little larger than a reference full spool); no Spoolman "
        "lookup has been made for C32A080A, so the measurement stays on the "
        "lane only -- run AFC_BAMBU_SCAN LANE=lane12 to link it.")

    def test_it_holds_until_the_bind_is_dispatched(self, monkeypatch):
        unit = self._abs_unit(monkeypatch)
        sp = unit._measure
        sp._drain_spool_summary(0)
        assert sp._pending_summary == {0: (138, 955, 1000, 200.0)}
        assert unit.logger.messages == []
        unit._spoolman_latched.add(0)          # _surface_slot_info sent it
        sp._drain_spool_summary(0)
        assert sp._pending_summary == {}
        assert unit.logger.messages == [("info", BRFID_ABS_HELD_LINE)]

    def test_a_bay_with_no_uid_is_not_waiting_on_a_bind(self, monkeypatch):
        unit = self._abs_unit(monkeypatch, uid=None)
        unit._measure._drain_spool_summary(0)
        assert unit.logger.messages == [
            ("info", "Bambu_AMS_1 lane12: tag read: ABS. Measured full -- "
                     "roughly 955 g of a 1000 g spool (the AMS read 138%, "
                     "meaning it measures a little larger than a reference "
                     "full spool); not linked to a Spoolman spool, so this "
                     "is kept on the lane only.")]

    def test_no_spoolman_means_no_bind_to_wait_for(self, monkeypatch):
        unit = self._abs_unit(monkeypatch, spoolman=None)
        unit._measure._drain_spool_summary(0)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [("info", self.ABS_UNASKED)]

    def test_an_unscanned_bay_is_not_held(self, monkeypatch):
        # The boot hold returns before the dispatch, so no bind is coming.
        unit = self._abs_unit(monkeypatch, scanned=())
        unit._measure._drain_spool_summary(0)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [("info", self.ABS_UNASKED)]

    def test_the_backstop_still_bounds_this_wait_too(self, monkeypatch):
        unit = self._abs_unit(monkeypatch, deadline=50.0)
        unit._measure._drain_spool_summary(0)
        assert unit._measure._pending_summary == {}
        assert unit.logger.messages == [
            ("info", "Bambu_AMS_1 lane12: tag read: ABS [tag C32A080A]. "
                     "Measured full -- roughly 955 g of a 1000 g spool (the "
                     "AMS read 138%, meaning it measures a little larger "
                     "than a reference full spool); a Spoolman lookup for "
                     "C32A080A is in progress; the measurement is on the "
                     "lane for now.")]


class TestBambuSpoolmanSaySpoolSummary:
    """One plain line: what was read, how much is left, and where it went."""

    @staticmethod
    def _say_unit(monkeypatch: pytest.MonkeyPatch, name: str, lane: str,
                  slot: int, record: Dict[str, Any], *,
                  spool_id: Optional[int] = None,
                  section: Optional[bool] = True,
                  spoolman: Optional[str] = None,
                  **kwargs: Any) -> afcBambuAMS:
        """
        A unit whose bay ``slot`` carries ``record``, mapped to ``lane``.

        :param monkeypatch: pytest's monkeypatch fixture
        :param name: the unit's name
        :param lane: the lane's name
        :param slot: its bay
        :param record: the bay's record
        :param spool_id: the lane's Spoolman binding
        :param section: the Spoolman section, as brfid_unit takes it
        :param spoolman: AFC core's Spoolman setting
        :param kwargs: further make_bambu_unit arguments
        :return afcBambuAMS: the unit
        """
        slots: List[Dict[str, Any]] = [{} for _ in range(slot)]
        slots.append(record)
        return brfid_unit(monkeypatch, name, section=section,
                          spoolman=spoolman, slots=slots,
                          lanes=[LaneSpec(lane, slot, spool_id=spool_id)],
                          **kwargs)

    # ── BambuAMS_2's bay 2: a Bambu PLA Basic reel on lane20 ──

    def _basic(self, monkeypatch: pytest.MonkeyPatch, *,
               material: Optional[str] = "Bambu PLA Basic",
               color: Optional[str] = "0080FFFF",
               uid: Optional[str] = None,
               spool_id: Optional[int] = 87) -> afcBambuAMS:
        """
        :param monkeypatch: pytest's monkeypatch fixture
        :param material: the tag's material
        :param color: the tag's colour, wire form
        :param uid: the tag's chip UID
        :param spool_id: lane20's Spoolman binding
        :return afcBambuAMS: the unit
        """
        record = slot_info(2, material=material, color=color, uid=uid,
                           weight=1000)
        return self._say_unit(monkeypatch, "BambuAMS_2", "lane20", 2, record,
                              spool_id=spool_id)

    def _said(self, unit: afcBambuAMS, lane: Optional[str], slot: int,
              pct: int, grams: int, settled: bool = True) -> None:
        """
        Say the summary for ``slot`` of a 1 kg spool.

        :param unit: the unit
        :param lane: the lane's name, or None for no lane
        :param slot: the bay
        :param pct: the measured percent
        :param grams: the grams
        :param settled: whether the record answered before the backstop
        """
        unit._measure._say_spool_summary(
            slot, unit.lanes[lane] if lane else None, pct, grams, 1000,
            settled=settled)

    def test_it_says_the_tag_the_amount_and_where_it_went(self, monkeypatch):
        unit = self._basic(monkeypatch)
        self._said(unit, "lane20", 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF). "
                     "Measured about 73% left -- roughly 730 g of a 1000 g "
                     "spool; updated Spoolman spool 87.")]

    def test_a_spool_with_no_tag_says_so_plainly(self, monkeypatch):
        # Third-party reels have no tag: ordinary, so stated, not warned.
        unit = self._basic(monkeypatch, material=None, color=None)
        self._said(unit, "lane20", 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: no tag on this spool. Measured about "
                     "73% left -- roughly 730 g of a 1000 g spool; updated "
                     "Spoolman spool 87.")]

    def test_an_unbound_spool_explains_where_the_number_went_instead(
            self, monkeypatch):
        unit = self._basic(monkeypatch, spool_id=None)
        self._said(unit, "lane20", 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF). "
                     "Measured about 73% left -- roughly 730 g of a 1000 g "
                     "spool; not linked to a Spoolman spool, so this is kept "
                     "on the lane only.")]

    def test_a_read_tag_that_did_not_link_names_the_setting_that_would(
            self, monkeypatch):
        # Say the cause and the fix, on THIS unit's section.
        unit = self._basic(monkeypatch, uid="04c07001", spool_id=None)
        unit._measure._spoolman_no_match = {"04c07001"}
        self._said(unit, "lane20", 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF) "
                     "[tag 04C07001]. Measured about 73% left -- roughly 730 "
                     "g of a 1000 g spool; not linked to a Spoolman spool, so "
                     "this is kept on the lane only -- Spoolman has no spool "
                     "carrying 04C07001 and auto-create is off for this unit "
                     "(add 'auto_spoolman_create: True' to [AFC_BambuAMS "
                     "BambuAMS_2], or bind that UID to an existing spool).")]

    def test_with_auto_create_on_it_does_not_blame_the_setting(
            self, monkeypatch):
        unit = self._basic(monkeypatch, uid="04c07001", spool_id=None)
        unit.auto_spoolman_create = True
        unit._measure._spoolman_no_match = {"04c07001"}
        self._said(unit, "lane20", 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF) "
                     "[tag 04C07001]. Measured about 73% left -- roughly 730 "
                     "g of a 1000 g spool; not linked to a Spoolman spool, so "
                     "this is kept on the lane only -- Spoolman has no spool "
                     "carrying 04C07001.")]

    def test_it_does_not_speak_for_spoolman_before_asking_it(self, monkeypatch):
        # An unbound lane is not evidence about Spoolman's contents: only the
        # miss memo, which a real lookup writes, licenses that sentence.
        unit = self._basic(monkeypatch, uid="4b8e44f6", spool_id=None)
        self._said(unit, "lane20", 2, 75, 750)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF) "
                     "[tag 4B8E44F6]. Measured about 75% left -- roughly 750 "
                     "g of a 1000 g spool; no Spoolman lookup has been made "
                     "for 4B8E44F6, so the measurement stays on the lane only "
                     "-- run AFC_BAMBU_SCAN LANE=lane20 to link it.")]

    # Where an unbound lane's one lookup stands: out or still owed, answered
    # with a miss, answered with a spool AFC would clear the lane for, sent
    # earlier, or never sent.

    #: The start of lane20's line for the 95F2C30C reel, 74% (611 g).
    UNBOUND = ("BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF) [tag "
               "95F2C30C]. Measured about 74% left -- roughly 611 g of a 1000 "
               "g spool; ")
    #: Where it went while the lookup is out and the measurement held for it.
    GOES_TO_SPOOL = ("a Spoolman lookup for 95F2C30C is in progress; the "
                     "measurement goes to the spool it finds.")

    def _unbound(self, monkeypatch: pytest.MonkeyPatch) -> afcBambuAMS:
        """
        :param monkeypatch: pytest's monkeypatch fixture
        :return afcBambuAMS: BambuAMS_2, lane20 unbound, tag 95f2c30c read
        """
        return self._basic(monkeypatch, uid="95f2c30c", spool_id=None)

    @pytest.mark.parametrize("state", ["in-flight", "pending", "retrying"])
    def test_a_lookup_not_answered_yet_is_said_to_be_in_progress(
            self, state, monkeypatch):
        # The measurement is held for the bind (a claim not yet sent), so
        # only the lookup's own state says it is in progress.
        unit = self._unbound(monkeypatch)
        sp = unit._measure
        if state == "in-flight":
            sp._spoolman_inflight = {"95f2c30c"}
        elif state == "pending":
            sp._bind_pending = {2}
        else:
            # Spoolman did not answer, and the unit will ask again.
            unit._lookup_retry = {2: 130.0}
            unit._lookup_coming = lambda slot, info: slot == 2
        sp._bind_owed = {2: (74, 1000, None, False)}
        self._said(unit, "lane20", 2, 74, 611)
        assert unit.logger.messages == [
            ("info", self.UNBOUND + self.GOES_TO_SPOOL)]

    def test_a_retry_the_unit_will_not_send_is_not_in_progress(
            self, monkeypatch):
        # A late "no answer" left a retry for a bay the surface path has
        # since asked about and latched: nothing will ask again.
        unit = self._unbound(monkeypatch)
        unit._lookup_retry = {2: 130.0}
        unit._spoolman_latched = {2}
        self._said(unit, "lane20", 2, 74, 611)
        assert unit.logger.messages == [
            ("info", self.UNBOUND + "95F2C30C was looked up in Spoolman on "
                     "this connection and no spool is linked to the lane "
                     "from it, so the measurement stays on the lane only -- "
                     "run AFC_BAMBU_SCAN LANE=lane20 to look it up again.")]

    def test_a_lookup_with_nothing_owed_says_the_figure_is_on_the_lane(
            self, monkeypatch):
        unit = self._unbound(monkeypatch)
        unit._measure._spoolman_inflight = {"95f2c30c"}
        self._said(unit, "lane20", 2, 74, 611)
        assert unit.logger.messages == [
            ("info", self.UNBOUND + "a Spoolman lookup for 95F2C30C is in "
                     "progress; the measurement is on the lane for now.")]

    def test_a_sent_bind_that_has_not_landed_is_in_progress(self, monkeypatch):
        # Matched and handed to set_spoolID, whose fetch binds the lane a
        # moment later: the measurement held for it goes with it.
        unit = self._unbound(monkeypatch)
        unit._measure._bind_owed = {2: (74, 1000, None, True)}
        self._said(unit, "lane20", 2, 74, 611)
        assert unit.logger.messages == [
            ("info", self.UNBOUND + self.GOES_TO_SPOOL)]

    def test_a_refused_match_names_the_spool_and_the_fix(self, monkeypatch):
        unit = self._unbound(monkeypatch)
        unit._lookup_refused = {2: ("95f2c30c", 136)}
        self._said(unit, "lane20", 2, 74, 611)
        assert unit.logger.messages == [
            ("info", self.UNBOUND + "not linked to a Spoolman spool, so this "
                     "is kept on the lane only -- 95F2C30C matches Spoolman "
                     "spool 136, which has no remaining weight on record; "
                     "correct spool 136's weight in Spoolman, then run "
                     "AFC_BAMBU_SCAN LANE=lane20 to link it.")]

    def test_a_refusal_for_another_tag_is_not_this_ones(self, monkeypatch):
        unit = self._unbound(monkeypatch)
        unit._lookup_refused = {2: ("4b8e44f6", 136)}
        self._said(unit, "lane20", 2, 74, 611)
        assert unit.logger.messages == [
            ("info", self.UNBOUND + "no Spoolman lookup has been made for "
                     "95F2C30C, so the measurement stays on the lane only -- "
                     "run AFC_BAMBU_SCAN LANE=lane20 to link it.")]

    def test_a_lookup_sent_earlier_is_not_said_to_be_unasked(
            self, monkeypatch):
        # The bay's one lookup went out on this connection, and nothing it
        # found is on the lane now (the link cleared by hand, say).
        unit = self._unbound(monkeypatch)
        unit._spoolman_latched = {2}
        self._said(unit, "lane20", 2, 74, 611)
        assert unit.logger.messages == [
            ("info", self.UNBOUND + "95F2C30C was looked up in Spoolman on "
                     "this connection and no spool is linked to the lane "
                     "from it, so the measurement stays on the lane only -- "
                     "run AFC_BAMBU_SCAN LANE=lane20 to look it up again.")]

    def test_a_miss_outranks_the_latch(self, monkeypatch):
        unit = self._unbound(monkeypatch)
        unit._spoolman_latched = {2}
        unit._measure._spoolman_no_match = {"95F2C30C"}
        self._said(unit, "lane20", 2, 74, 611)
        assert unit.logger.messages == [
            ("info", self.UNBOUND + "not linked to a Spoolman spool, so this "
                     "is kept on the lane only -- Spoolman has no spool "
                     "carrying 95F2C30C and auto-create is off for this unit "
                     "(add 'auto_spoolman_create: True' to [AFC_BambuAMS "
                     "BambuAMS_2], or bind that UID to an existing spool).")]

    def test_an_undecodable_tag_is_told_to_bind_by_hand(self, monkeypatch):
        # No profile to create a spool from, so the setting is not named.
        unit = self._basic(monkeypatch, material=None, color=None,
                           uid="84ea7601", spool_id=None)
        self._said(unit, "lane20", 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag 84EA7601 read but its profile "
                     "could not be decoded (not a Bambu tag?) -- bind that "
                     "UID to a spool in Spoolman and it will match from now "
                     "on. Measured about 73% left -- roughly 730 g of a 1000 "
                     "g spool; not linked to a Spoolman spool, so this is "
                     "kept on the lane only -- bind 84EA7601 to a spool in "
                     "Spoolman to track this reel.")]

    def test_a_decoded_profile_not_yet_in_the_record_says_what_is_known(
            self, monkeypatch):
        # The profile decoded (its sku is in), the material string is not in
        # this snapshot yet: say the sku and nothing more.
        unit = self._basic(monkeypatch, material=None, color=None,
                           uid="84ea7601")
        unit._slots[2]["sku"] = "GFA00"
        self._said(unit, "lane20", 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag 84EA7601 read: GFA00. Measured "
                     "about 73% left -- roughly 730 g of a 1000 g spool; "
                     "updated Spoolman spool 87.")]

    def test_sync_turned_off_is_distinguished_from_unbound(self, monkeypatch):
        unit = self._basic(monkeypatch)
        unit.sync_measured_to_spoolman = False
        self._said(unit, "lane20", 2, 54, 540)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF). "
                     "Measured about 54% left -- roughly 540 g of a 1000 g "
                     "spool; Spoolman sync is off, so this is kept on the "
                     "lane only.")]

    def test_over_100_percent_reads_as_full_not_as_a_number_to_discount(
            self, monkeypatch):
        # A spool proud of the reference radius, not extra filament.
        unit = self._basic(monkeypatch)
        self._said(unit, "lane20", 2, 113, 1000)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF). "
                     "Measured full -- roughly 1000 g of a 1000 g spool (the "
                     "AMS read 113%, meaning it measures a little larger than "
                     "a reference full spool); updated Spoolman spool 87.")]

    def test_it_names_the_lane_the_operator_knows(self, monkeypatch):
        unit = self._basic(monkeypatch)
        self._said(unit, "lane20", 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 lane20: tag read: Bambu PLA Basic (0080FF). "
                     "Measured about 73% left -- roughly 730 g of a 1000 g "
                     "spool; updated Spoolman spool 87.")]

    def test_no_lane_falls_back_to_the_bay(self, monkeypatch):
        unit = self._basic(monkeypatch)
        self._said(unit, None, 2, 73, 730)
        assert unit.logger.messages == [
            ("info", "BambuAMS_2 bay 2: tag read: Bambu PLA Basic (0080FF). "
                     "Measured about 73% left -- roughly 730 g of a 1000 g "
                     "spool; not linked to a Spoolman spool, so this is kept "
                     "on the lane only.")]

    @pytest.mark.parametrize("section, advised", [
        (None, False), (False, False), (True, True),
    ], ids=["no-section", "enabled-false", "section"])
    def test_the_bind_advice_needs_the_module(self, section, advised,
                                               monkeypatch):
        # With the module off nothing matches a UID, and the same line goes
        # on to say the module is off.
        record = slot_info(1, uid="aabbccdd", weight=1000)
        unit = self._say_unit(monkeypatch, "Bambu_AMS_1", "lane8", 1, record,
                              section=section)
        self._said(unit, "lane8", 1, 63, 518)
        read = ("Bambu_AMS_1 lane8: tag AABBCCDD read but its profile could "
                "not be decoded (not a Bambu tag?)")
        if advised:
            line = (read + " -- bind that UID to a spool in Spoolman and it "
                    "will match from now on. Measured about 63% left -- "
                    "roughly 518 g of a 1000 g spool; not linked to a "
                    "Spoolman spool, so this is kept on the lane only -- bind "
                    "AABBCCDD to a spool in Spoolman to track this reel.")
        else:
            line = (read + ". Measured about 63% left -- roughly 518 g of a "
                    "1000 g spool; kept on the lane -- the Spoolman module "
                    "([AFC_BambuAMS_rfid]) is off.")
        assert unit.logger.messages == [("info", line)]

    def test_a_module_that_is_off_names_the_spool_it_did_not_update(
            self, monkeypatch):
        record = slot_info(1, material="PLA Basic", weight=1000)
        unit = self._say_unit(monkeypatch, "Bambu_AMS_1", "lane8", 1, record,
                              section=None, spool_id=164)
        self._said(unit, "lane8", 1, 63, 518)
        assert unit.logger.messages == [
            ("info", "Bambu_AMS_1 lane8: tag read: PLA Basic. Measured about "
                     "63% left -- roughly 518 g of a 1000 g spool; kept on "
                     "the lane -- the Spoolman module ([AFC_BambuAMS_rfid]) is "
                     "off, so Spoolman spool 164 was not updated.")]

    # ── one reel's figure only ever comes down ──

    def _floored(self, monkeypatch: pytest.MonkeyPatch,
                 readings: Sequence[int]) -> afcBambuAMS:
        """
        Bambu_AMS_1's bay 0, an ABS reel (tag c32a080a) on lane12 bound to
        spool 163, measured ``readings`` in turn (one cycle each).

        :param monkeypatch: pytest's monkeypatch fixture
        :param readings: the percents the unit measured
        :return afcBambuAMS: the unit, its logger cleared after them
        """
        record = slot_info(0, uid="c32a080a", weight=1000)
        unit = self._say_unit(monkeypatch, "Bambu_AMS_1", "lane12", 0, record,
                              spool_id=163, spoolman=BRFID_SPOOLMAN_URL)
        for seq, pct in enumerate(readings, start=1):
            unit._measure._adopt_measured_remain(
                0, pct, "physical AMS measurement", seq=seq)
        unit.logger.messages.clear()
        return unit

    #: What lane12's reel says of itself: a tag whose profile did not decode.
    FLOORED_READ = (
        "Bambu_AMS_1 lane12: tag C32A080A read but its profile could not be "
        "decoded (not a Bambu tag?) -- bind that UID to a spool in Spoolman "
        "and it will match from now on. ")

    def test_it_says_why_the_figure_did_not_move(self, monkeypatch):
        # A weight that will not budge while the unit narrates a different
        # percent has to say why.
        unit = self._floored(monkeypatch, [138, 145])
        self._said(unit, "lane12", 0, 138, 952)
        assert unit.logger.messages == [
            ("info", self.FLOORED_READ + "Measured full -- roughly 952 g of a "
                     "1000 g spool (the AMS read 145% this time against 138% "
                     "before it; filament does not grow, so the lower reading "
                     "stands); updated Spoolman spool 163.")]

    def test_an_unheld_reading_keeps_the_plain_wording(self, monkeypatch):
        unit = self._floored(monkeypatch, [138])
        self._said(unit, "lane12", 0, 138, 952)
        assert unit.logger.messages == [
            ("info", self.FLOORED_READ + "Measured full -- roughly 952 g of a "
                     "1000 g spool (the AMS read 138%, meaning it measures a "
                     "little larger than a reference full spool); updated "
                     "Spoolman spool 163.")]

    def test_a_held_reading_under_100_says_why_too(self, monkeypatch):
        unit = self._floored(monkeypatch, [60, 64])
        self._said(unit, "lane12", 0, 60, 600)
        assert unit.logger.messages == [
            ("info", self.FLOORED_READ + "Measured about 60% left -- roughly "
                     "600 g of a 1000 g spool (the AMS read 64% this time "
                     "against 60% before it; filament does not grow, so the "
                     "lower reading stands); updated Spoolman spool 163.")]

    # ── "tag read" is a claim about this scan, not about the record ──

    def _matte_said(self, monkeypatch: pytest.MonkeyPatch,
                    **wire: Any) -> List[Tuple[str, str]]:
        """
        Say lane14's summary for its PLA Matte reel (tag 13f56d32), 86%.

        :param monkeypatch: pytest's monkeypatch fixture
        :param wire: further wire fields for the record (sres...)
        :return list: what was logged
        """
        record = slot_info(0, material="PLA Matte", color="757575FF",
                           uid="13f56d32", weight=1000, **wire)
        unit = self._say_unit(monkeypatch, "Bambu_AMS_1", "lane14", 0, record)
        self._said(unit, "lane14", 0, 86, 860)
        return unit.logger.messages

    @staticmethod
    def _matte_line(read: str) -> Tuple[str, str]:
        """
        :param read: how the tag is introduced ("tag read", "tag on file")
        :return tuple: lane14's summary line
        """
        return ("info", f"Bambu_AMS_1 lane14: {read}: PLA Matte (757575) [tag "
                        f"13F56D32]. Measured about 86% left -- roughly 860 g "
                        f"of a 1000 g spool; no Spoolman lookup has been made "
                        f"for 13F56D32, so the measurement stays on the lane "
                        f"only -- run AFC_BAMBU_SCAN LANE=lane14 to link it.")

    def test_a_fresh_read_says_tag_read(self, monkeypatch):
        assert self._matte_said(monkeypatch, sres=1) == [
            self._matte_line("tag read")]

    def test_a_kept_tag_says_on_file_not_read(self, monkeypatch):
        # scan_res 3: this scan read no new tag and kept the one on the bay.
        assert self._matte_said(monkeypatch, sres=3) == [
            self._matte_line("tag on file")]

    def test_a_foreign_result_is_also_on_file(self, monkeypatch):
        assert self._matte_said(monkeypatch, sres=2) == [
            self._matte_line("tag on file")]

    def test_older_firmware_keeps_the_old_wording(self, monkeypatch):
        # No scan_res published: state what is there rather than guess.
        assert self._matte_said(monkeypatch) == [self._matte_line("tag read")]

    @pytest.mark.parametrize("res, read", [(1, "tag read"),
                                           (3, "tag on file")])
    def test_the_uid_is_still_carried_either_way(self, res, read, monkeypatch):
        assert self._matte_said(monkeypatch, sres=res) == [
            self._matte_line(read)]

    # ── a backstop firing is not a diagnosis of the tag ──

    def _ht_said(self, monkeypatch: pytest.MonkeyPatch, settled: bool,
                 **wire: Any) -> List[Tuple[str, str]]:
        """
        Say lane28's summary for the HT's one bay (tag 4b8e44f6), 81%.

        :param monkeypatch: pytest's monkeypatch fixture
        :param settled: whether the record answered before the backstop
        :param wire: further wire fields for the record (material, color)
        :return list: what was logged
        """
        record = slot_info(0, uid="4b8e44f6", weight=1000, **wire)
        unit = self._say_unit(monkeypatch, "Bambu_AMS_HT_1", "lane28", 0,
                              record, model="ht")
        self._said(unit, "lane28", 0, 81, 810, settled=settled)
        return unit.logger.messages

    #: Where lane28's measurement went: unbound, with no profile to match.
    HT_UNDECODED_WENT = (
        "Measured about 81% left -- roughly 810 g of a 1000 g spool; not "
        "linked to a Spoolman spool, so this is kept on the lane only -- bind "
        "4B8E44F6 to a spool in Spoolman to track this reel.")
    #: lane28's line when the backstop fired before the profile arrived.
    HT_NOT_ARRIVED = (
        "info", "Bambu_AMS_HT_1 lane28: tag 4B8E44F6 -- the profile had not "
                "arrived yet when this was reported, so the material is not "
                "in this line. It lands on the lane by itself a moment later; "
                "nothing needs doing. " + HT_UNDECODED_WENT)
    #: lane28's line for a settled record whose profile did not decode.
    HT_UNDECODABLE = (
        "info", "Bambu_AMS_HT_1 lane28: tag 4B8E44F6 read but its profile "
                "could not be decoded (not a Bambu tag?) -- bind that UID to "
                "a spool in Spoolman and it will match from now on. "
                + HT_UNDECODED_WENT)

    def test_a_timed_out_summary_does_not_call_the_tag_undecodable(
            self, monkeypatch):
        assert self._ht_said(monkeypatch, settled=False) == [
            self.HT_NOT_ARRIVED]

    def test_it_says_the_profile_had_not_arrived(self, monkeypatch):
        messages = self._ht_said(monkeypatch, settled=False)
        assert messages == [self.HT_NOT_ARRIVED]

    def test_a_genuinely_undecodable_tag_is_still_called_out(self, monkeypatch):
        # A settled record with a UID and no profile is a tag nothing could
        # open, and the operator needs its UID to bind it by hand.
        assert self._ht_said(monkeypatch, settled=True) == [
            self.HT_UNDECODABLE]

    @pytest.mark.parametrize("settled", [True, False])
    def test_the_uid_is_reported_either_way(self, settled, monkeypatch):
        expected = self.HT_UNDECODABLE if settled else self.HT_NOT_ARRIVED
        assert self._ht_said(monkeypatch, settled=settled) == [expected]

    @pytest.mark.parametrize("settled", [True, False])
    def test_a_decoded_tag_is_unaffected_by_the_backstop(self, settled,
                                                         monkeypatch):
        messages = self._ht_said(monkeypatch, settled=settled,
                                 material="PLA Basic", color="0086D6FF")
        assert messages == [
            ("info", "Bambu_AMS_HT_1 lane28: tag read: PLA Basic (0086D6) "
                     "[tag 4B8E44F6]. Measured about 81% left -- roughly 810 "
                     "g of a 1000 g spool; no Spoolman lookup has been made "
                     "for 4B8E44F6, so the measurement stays on the lane only "
                     "-- run AFC_BAMBU_SCAN LANE=lane28 to link it.")]


class TestBambuSpoolmanMaterialKey:
    """The most specific name on the reel is what the density is looked up by."""

    def test_the_variant_is_part_of_the_material_for_density(self, monkeypatch):
        # A lane splits the tag: material "PLA", sub_type "Matte". The variant
        # is exactly the part that changes the density.
        unit = brfid_unit(monkeypatch, lanes=["lane1"])
        sp, lane = unit._measure, unit.lanes["lane1"]
        lane.material, lane.sub_type = "PLA", "Matte"
        assert sp._material_key(lane, {}) == "PLA Matte"
        # The bridge's own string is already complete and is preferred.
        lane.sub_type = "Basic"
        assert sp._material_key(lane, {"material": "PLA Matte"}) == "PLA Matte"
        # No variant, or one already spelled out, must not double up.
        lane.sub_type = ""
        assert sp._material_key(lane, {}) == "PLA"
        lane.material, lane.sub_type = "PLA Matte", "Matte"
        assert sp._material_key(lane, {}) == "PLA Matte"
        # A lane with nothing on it yet falls back to the bay's record.
        lane.material, lane.sub_type = None, ""
        assert sp._material_key(lane, {"material": "ABS"}) == "ABS"
        assert sp._material_key(None, {"material": "ABS"}) == "ABS"
        assert sp._material_key(None, None) == ""
        assert unit.logger.messages == []


class TestBambuSpoolmanGramsFor:
    """Grams are a mass: density x the unit's reference volume x percent."""

    @staticmethod
    def _grams_unit(monkeypatch: pytest.MonkeyPatch, material: Optional[str],
                    *, density: Optional[float] = None,
                    rec: Optional[Dict[str, Any]] = None,
                    record_material: Optional[str] = None,
                    sub_type: str = "") -> Tuple[BambuSpoolman, Any]:
        """
        A unit whose bay 0 is mapped to lane1; its bridge holds ``rec`` as
        the last capacity measurement of the unit's device.

        :param monkeypatch: pytest's monkeypatch fixture
        :param material: lane1's material
        :param density: lane1's own (Spoolman) density
        :param rec: the bridge's last capacity measurement
        :param record_material: the bay record's material; the lane's when
          None
        :param sub_type: lane1's variant
        :return tuple: (the delegate, lane1)
        """
        bridge = FakeBridge(cap={0x0700: rec} if rec else None)
        extras_: Dict[str, Any] = {"sub_type": sub_type}
        if density is not None:
            extras_["density"] = density
        record = slot_info(0, material=record_material or material,
                           weight=1000)
        unit = brfid_unit(monkeypatch, bridge=bridge, slots=[record],
                          lanes=[LaneSpec("lane1", 0, material=material,
                                          extras=extras_)])
        return unit._measure, unit.lanes["lane1"]

    def test_grams_follow_the_density_not_the_tag(self, monkeypatch):
        # The blue reel: 1.24 x 663 x 82% = 674.1, not the 820 g tag-linear.
        sp, lane = self._grams_unit(monkeypatch, "PLA")
        assert sp._grams_for(0, lane, 82, 1000) == 674
        assert sp._u.logger.messages == []

    def test_a_denser_material_reads_heavier_at_the_same_percent(
            self, monkeypatch):
        # ABS: 1.04 x 663 x 141% = 972.2. PLA at the same reading would be
        # 1159.2, which the 1 kg nominal caps.
        sp, lane = self._grams_unit(monkeypatch, "ABS")
        assert sp._grams_for(0, lane, 141, 1000) == 972
        sp2, lane2 = self._grams_unit(monkeypatch, "PLA")
        assert sp2._grams_for(0, lane2, 141, 1000) == 1000
        assert sp._u.logger.messages == [] and sp2._u.logger.messages == []

    def test_arithmetic_that_fails_falls_back_to_tag_linear(self, monkeypatch):
        # Inventory must not depend on the mass arithmetic surviving.
        sp, lane = self._grams_unit(monkeypatch, "PLA")
        monkeypatch.setattr(sp, "CAP_REF_VOLUME_CM3", None)
        assert sp._grams_for(0, lane, 82, 1000) == 820
        assert sp._grams_for(0, lane, 141, 1000) == 1000
        assert sp._u.logger.messages == []

    def test_the_tag_weight_is_still_the_ceiling(self, monkeypatch):
        # 1.04 x 663 x 160% = 1103.2: a full reel sitting large.
        sp, lane = self._grams_unit(monkeypatch, "ABS")
        assert sp._grams_for(0, lane, 160, 1000) == 1000
        assert sp._grams_for(0, lane, 160, 750) == 750
        assert sp._u.logger.messages == []

    def test_the_unrounded_radius_beats_the_integer_percent(self, monkeypatch):
        # R 77.357 mm between the 47.5 mm hub and the 82.6 mm full radius is
        # (77.357^2 - 47.5^2) / (82.6^2 - 47.5^2) = 81.63%, and
        # 1.24 x 663 x 81.63% = 671.1; the integer 82% would say 674.
        rec = {"pct_raw": 82, "radius_m": 0.077, "save_radius_m": 0.077357}
        sp, lane = self._grams_unit(monkeypatch, "PLA", rec=rec)
        assert sp._grams_for(0, lane, 82, 1000) == 671
        assert sp._u.logger.messages == []

    def test_a_stale_record_is_not_used_for_the_radius(self, monkeypatch):
        rec = {"pct_raw": 60, "radius_m": 0.071, "save_radius_m": 0.071200}
        sp, lane = self._grams_unit(monkeypatch, "PLA", rec=rec)
        assert sp._grams_for(0, lane, 82, 1000) == 674
        assert sp._u.logger.messages == []

    def test_an_unknown_material_keeps_the_old_answer(self, monkeypatch):
        # No density to weigh at: 82% of 1000 g, tag-linear.
        sp, lane = self._grams_unit(monkeypatch, None)
        assert sp._grams_for(0, lane, 82, 1000) == 820
        assert sp._u.logger.messages == []

    def test_the_lane_density_wins_over_the_table(self, monkeypatch):
        # Spoolman's 1.32: 1.32 x 663 x 87% = 761.4 (the table's 1.24 would
        # give 715).
        sp, lane = self._grams_unit(monkeypatch, "PLA", density=1.32)
        assert sp._grams_for(0, lane, 87, 1000) == 761
        assert sp._u.logger.messages == []

    @pytest.mark.parametrize("record_material, sub_type, grams", [
        ("PLA Matte", "", 761),
        ("PLA", "Matte", 761),
        ("PLA", "", 715),
    ], ids=["record-names-the-variant", "lane-joins-the-variant",
            "no-variant"])
    def test_the_variant_reaches_the_density_lookup_end_to_end(
            self, record_material, sub_type, grams, monkeypatch):
        # The table's matte entry is made heavier than plain PLA, so only a
        # lookup that saw "PLA Matte" gives 1.32 x 663 x 87% = 761.4; plain
        # PLA's 1.24 gives 715.2.
        monkeypatch.setitem(extras.AFC_RFID.MATERIAL_DENSITY, "plamatte",
                            1.32)
        sp, lane = self._grams_unit(monkeypatch, "PLA", sub_type=sub_type,
                                    record_material=record_material)
        assert sp._grams_for(0, lane, 87, 1000) == grams
        assert sp._u.logger.messages == []


class TestBambuSpoolmanPctFor:
    """The inverse of _grams_for: the percent a lane's grams stand for."""

    def test_the_inverse_undoes_grams_for(self, monkeypatch):
        unit = brfid_unit(
            monkeypatch, section=None,
            lanes=[LaneSpec("lane8", 1, material="PLA",
                            extras={"sub_type": "Basic"})],
            slots=[slot_info(0, present=False),
                   slot_info(1, material="PLA Basic", uid="d13fdb0e",
                             weight=1000)])
        sp, lane = unit._measure, unit.lanes["lane8"]
        for pct in (23, 56, 63, 89):
            grams = sp._grams_for(1, lane, pct, 1000)
            assert round(sp._pct_for(1, lane, grams, 1000)) == pct
        # PLA's full reference volume is 1.24 x 663 = 822.12 g.
        assert sp._pct_for(1, lane, 822.12, 1000) == pytest.approx(100.0)
        assert sp._pct_for(1, lane, 411.06, 1000) == pytest.approx(50.0)
        assert sp._pct_for(1, lane, 0, 1000) is None
        assert sp._pct_for(1, lane, 500, 0) is None
        assert sp._pct_for(1, lane, "lots", 1000) is None
        # With nothing naming the material there is no density: grams read
        # against the tag's nominal.
        lane.material, lane.sub_type = None, ""
        unit._slots[1]["material"] = None
        assert sp._pct_for(1, lane, 500, 1000) == pytest.approx(50.0)
        assert unit.logger.messages == []


class TestBambuSpoolmanLogCapacitySample:
    """One capsample row per measurement, its radius only when it belongs."""

    @staticmethod
    def _sample(monkeypatch: pytest.MonkeyPatch, record: Dict[str, Any],
                rec: Optional[Dict[str, Any]], source: str = "capscan"
                ) -> List[Tuple[str, str]]:
        """
        Log the row for a 144% reading of bay 0 on unit AMS, no lane.

        :param monkeypatch: pytest's monkeypatch fixture
        :param record: bay 0's record
        :param rec: the bridge's narration cache for the unit's device
        :param source: what produced the measurement
        :return list: what was logged
        """
        bridge = FakeBridge(cap={0x0700: rec} if rec else None)
        unit = brfid_unit(monkeypatch, "AMS", bridge=bridge, slots=[record])
        unit._measure._log_capacity_sample(0, None, 144, 1000, 1000, source)
        return unit.logger.messages

    @staticmethod
    def _row(radius: str, circ: str, save_r: str, src: str,
             source: str = "capscan") -> Tuple[str, str]:
        """
        The row for that reading. No lane and no material, so the table's
        PLA fallback models it: 1.24 x 663 x 144% = 1183.9 g.

        :param radius: the radius_m field
        :param circ: the circ_m field
        :param save_r: the save_r_m field
        :param src: the radius_src field
        :param source: the source field
        :return tuple: the debug line
        """
        return ("debug", f"capsample unit=AMS slot=0 lane=? spool=None "
                         f"material='' density=1.24 pct_raw=144 "
                         f"radius_m={radius} circ_m={circ} save_r_m={save_r} "
                         f"radius_src={src} source='{source}' nominal=1000 "
                         f"grams_written=1000 grams_modelled=1184")

    def test_the_narrated_radius_is_used_when_its_percent_agrees(
            self, monkeypatch):
        # The unrounded radius and the circumference ride along: only the
        # live narration states either.
        rec = {"pct_raw": 144, "radius_m": 0.094, "circumference_m": 0.592,
               "save_radius_m": 0.093643}
        logged = self._sample(monkeypatch, slot_info(0, mpct=144, mrad=94),
                              rec)
        assert logged == [self._row("0.094", "0.592", "0.093643",
                                    "narration")]

    def test_a_stale_narration_is_refused_and_the_row_says_so(
            self, monkeypatch):
        # The cache holds the previous spool's measurement.
        rec = {"pct_raw": 60, "radius_m": 0.071, "circumference_m": 0.446}
        logged = self._sample(monkeypatch, slot_info(0), rec)
        assert logged == [self._row("None", "None", "None", "stale(rec=60)")]

    def test_the_slot_record_supplies_the_radius_after_a_restart(
            self, monkeypatch):
        # A fresh bridge has no narration, but the firmware stamped the
        # radius on the bay beside the percent; the row names its source.
        logged = self._sample(monkeypatch, slot_info(0, mpct=144, mrad=94),
                              None, source="physical AMS measurement")
        assert logged == [self._row("0.094", "None", "None", "slotrec",
                                    source="physical AMS measurement")]

    def test_a_stale_slot_record_is_refused_too(self, monkeypatch):
        # A boxed unit can advance meas_seq and leave meas_pct behind.
        logged = self._sample(monkeypatch, slot_info(0, mpct=127, mrad=88),
                              None)
        assert logged == [self._row("None", "None", "None", "none")]

    def test_older_firmware_leaves_an_honest_gap(self, monkeypatch):
        logged = self._sample(monkeypatch, slot_info(0, mpct=144), None)
        assert logged == [self._row("None", "None", "None", "none")]


class TestBambuSpoolmanAdoptMeasuredRemain:
    """A measured percent becomes slot, lane, Spoolman and saved state once."""

    @staticmethod
    def _capsample(unit: str, slot: int, lane: str, spool: str,
                   material: str, density: str, pct: int, source: str,
                   nominal: int, grams: int, modelled: str) -> Tuple[str, str]:
        """
        The capsample row for a reading with no radius known.

        :param unit: the unit's name
        :param slot: the bay
        :param lane: the lane field ("?" for none)
        :param spool: the spool field
        :param material: the material field, quoted as the row quotes it
        :param density: the density field
        :param pct: the percent the grams were made from
        :param source: what produced the reading
        :param nominal: the tag's nominal
        :param grams: the grams written
        :param modelled: the grams_modelled field
        :return tuple: the debug line
        """
        return ("debug", f"capsample unit={unit} slot={slot} lane={lane} "
                         f"spool={spool} material={material} "
                         f"density={density} pct_raw={pct} radius_m=None "
                         f"circ_m=None save_r_m=None radius_src=none "
                         f"source='{source}' nominal={nominal} "
                         f"grams_written={grams} grams_modelled={modelled}")

    @staticmethod
    def _wrote(unit: str, grams: str, spool: int,
               tail: str = "") -> Tuple[str, str]:
        """
        :param unit: the unit's name
        :param grams: the grams as written
        :param spool: the Spoolman spool
        :param tail: what follows the source, for a capped reading
        :return tuple: the line the Spoolman weight write logs
        """
        return ("info", f"AFC bambu {unit}: wrote {grams} g remaining to "
                        f"Spoolman spool {spool} (physical AMS "
                        f"measurement){tail}")

    # ── the weighed figure: grams through the density, capped at nominal ──

    @staticmethod
    def _weighed(monkeypatch: pytest.MonkeyPatch, client: FakeSpoolmanClient,
                 material: Optional[str],
                 density: Optional[float]) -> afcBambuAMS:
        """
        Bambu_AMS_1's bay 0 on lane12, bound to spool 163.

        :param monkeypatch: pytest's monkeypatch fixture
        :param client: Spoolman
        :param material: the bay record's material
        :param density: lane12's own density, or None for none
        :return afcBambuAMS: the unit
        """
        extras_ = {"density": density} if density is not None else {}
        return brfid_unit(
            monkeypatch, client=client,
            lanes=[LaneSpec("lane12", 0, spool_id=163, extras=extras_)],
            slots=[slot_info(0, material=material, weight=1000)])

    def test_a_known_density_gives_the_weighed_figure_not_the_linear_one(
            self, monkeypatch):
        # 1.04 g/cm3 x 663 cm3 x 138% = 951.5, so 952 g. Tag-linear would say
        # the 1000 g nominal, which is no measurement at all.
        client = FakeSpoolmanClient()
        unit = self._weighed(monkeypatch, client, "ABS", 1.04)
        sp, lane = unit._measure, unit.lanes["lane12"]
        assert sp._adopt_measured_remain(0, 138, seq=1) is True
        assert lane.weight == 952
        assert sp._measured_remain == {0: 138} and sp._meas_seq_seen == {0: 1}
        assert sp._convert_owed == {} and sp._pending_summary == {}
        # The same number reaches Spoolman, not a second opinion.
        assert client.calls == [("set_remaining_weight", (163, 952.0))]
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == [
            self._capsample("Bambu_AMS_1", 0, "lane12", "163", "'ABS'", "1.04",
                            138, "capscan", 1000, 952, "952"),
            ("info", "Bambu_AMS_1 lane12: tag read: ABS. Measured full -- "
                     "roughly 952 g of a 1000 g spool (the AMS read 138%, "
                     "meaning it measures a little larger than a reference "
                     "full spool); updated Spoolman spool 163."),
            self._wrote("Bambu_AMS_1", "952.0", 163,
                        " -- measured 138%, capped to the spool's 1000 g "
                        "nominal")]

    def test_an_unknown_density_still_falls_back_to_tag_linear(
            self, monkeypatch):
        # No material and no lane density: 82% of 1000 g is 820 g, and the
        # grams stay owed their material.
        client = FakeSpoolmanClient()
        unit = self._weighed(monkeypatch, client, None, None)
        sp, lane = unit._measure, unit.lanes["lane12"]
        assert sp._adopt_measured_remain(0, 82, seq=1) is True
        assert lane.weight == 820
        assert sp._convert_owed == {0: (82, 1000, 820)}
        # A blank record holds the summary until the 145.0 backstop.
        assert sp._pending_summary == {0: (82, 820, 1000, 145.0)}
        assert client.calls == [("set_remaining_weight", (163, 820.0))]
        assert unit.logger.messages == [
            self._capsample("Bambu_AMS_1", 0, "lane12", "163", "''", "1.24",
                            82, "capscan", 1000, 820, "674"),
            self._wrote("Bambu_AMS_1", "820.0", 163)]

    def test_a_reading_over_100_percent_never_exceeds_the_tag_nominal(
            self, monkeypatch):
        # 1.24 x 663 x 141% = 1159.2 g, capped to the 1000 g nominal.
        client = FakeSpoolmanClient()
        unit = self._weighed(monkeypatch, client, "ABS", 1.24)
        sp, lane = unit._measure, unit.lanes["lane12"]
        assert sp._adopt_measured_remain(0, 141, seq=1) is True
        assert lane.weight == 1000
        assert client.calls == [("set_remaining_weight", (163, 1000.0))]
        assert unit.logger.messages == [
            self._capsample("Bambu_AMS_1", 0, "lane12", "163", "'ABS'", "1.24",
                            141, "capscan", 1000, 1000, "1159"),
            ("info", "Bambu_AMS_1 lane12: tag read: ABS. Measured full -- "
                     "roughly 1000 g of a 1000 g spool (the AMS read 141%, "
                     "meaning it measures a little larger than a reference "
                     "full spool); updated Spoolman spool 163."),
            self._wrote("Bambu_AMS_1", "1000.0", 163,
                        " -- measured 141%, capped to the spool's 1000 g "
                        "nominal")]

    # ── a spool cannot hold more than its own nominal ──
    # No material on the record, so these grams are tag-linear: the cap is
    # pinned where the grams are made.

    def _capped(self, monkeypatch: pytest.MonkeyPatch, pct: int, *,
                bay: int = 2, nominal: int = 1000,
                tag_pct: Optional[int] = None, name: str = "AMS",
                **kwargs: Any) -> Tuple[afcBambuAMS, FakeSpoolmanClient]:
        """
        Adopt one reading for ``bay`` of a unit whose lane15 there is bound
        to spool 87.

        :param monkeypatch: pytest's monkeypatch fixture
        :param pct: the reading
        :param bay: the bay
        :param nominal: the tag's nominal weight
        :param tag_pct: the percent stored on the tag
        :param name: the unit's name
        :param kwargs: further make_bambu_unit arguments
        :return tuple: (the unit, Spoolman)
        """
        client = FakeSpoolmanClient()
        slots: List[Dict[str, Any]] = [{} for _ in range(bay)]
        slots.append(slot_info(bay, weight=nominal, remain=tag_pct))
        unit = brfid_unit(monkeypatch, name, client=client, slots=slots,
                          lanes=[LaneSpec("lane15", bay, spool_id=87)],
                          **kwargs)
        assert unit._measure._adopt_measured_remain(
            bay, pct, "capscan", seq=1) is True
        return unit, client

    def test_the_measured_119_percent_becomes_1000g_not_1190g(
            self, monkeypatch):
        unit, client = self._capped(monkeypatch, 119)
        sp = unit._measure
        assert unit.lanes["lane15"].weight == 1000
        assert sp._measured_remain == {2: 119}
        assert sp._convert_owed == {2: (119, 1000, 1000)}
        assert sp._pending_summary == {2: (119, 1000, 1000, 145.0)}
        assert client.calls == [("set_remaining_weight", (87, 1000.0))]
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == [
            self._capsample("AMS", 2, "lane15", "87", "''", "1.24", 119,
                            "capscan", 1000, 1000, "978"),
            self._wrote("AMS", "1000.0", 87,
                        " -- measured 119%, capped to the spool's 1000 g "
                        "nominal")]

    def test_the_measured_107_percent_is_capped_too(self, monkeypatch):
        unit, client = self._capped(monkeypatch, 107)
        assert unit.lanes["lane15"].weight == 1000
        assert client.calls == [("set_remaining_weight", (87, 1000.0))]
        assert unit.logger.messages == [
            self._capsample("AMS", 2, "lane15", "87", "''", "1.24", 107,
                            "capscan", 1000, 1000, "880"),
            self._wrote("AMS", "1000.0", 87,
                        " -- measured 107%, capped to the spool's 1000 g "
                        "nominal")]

    def test_an_ordinary_reading_is_untouched(self, monkeypatch):
        # Uncapped and unrounded, and the write names no cap.
        unit, client = self._capped(monkeypatch, 80)
        assert unit.lanes["lane15"].weight == 800
        assert client.calls == [("set_remaining_weight", (87, 800.0))]
        assert unit.logger.messages == [
            self._capsample("AMS", 2, "lane15", "87", "''", "1.24", 80,
                            "capscan", 1000, 800, "658"),
            self._wrote("AMS", "800.0", 87)]

    def test_a_measurement_over_100_is_still_capped(self, monkeypatch):
        # The tag's 70% plays no part, and 1190 g does not get past the cap.
        unit, client = self._capped(monkeypatch, 119, tag_pct=70)
        assert unit.lanes["lane15"].weight == 1000
        assert client.calls == [("set_remaining_weight", (87, 1000.0))]
        assert unit.logger.messages == [
            self._capsample("AMS", 2, "lane15", "87", "''", "1.24", 119,
                            "capscan", 1000, 1000, "978"),
            self._wrote("AMS", "1000.0", 87,
                        " -- measured 119%, capped to the spool's 1000 g "
                        "nominal")]

    @pytest.mark.parametrize("bay", [0, 1, 2, 3])
    def test_the_measurement_wins_on_every_bay_of_a_boxed_unit(
            self, bay, monkeypatch):
        unit, client = self._capped(monkeypatch, 69, bay=bay, tag_pct=70)
        assert unit.lanes["lane15"].weight == 690
        assert unit._measure._measured_remain == {bay: 69}
        assert client.calls == [("set_remaining_weight", (87, 690.0))]
        assert unit.logger.messages == [
            self._capsample("AMS", bay, "lane15", "87", "''", "1.24", 69,
                            "capscan", 1000, 690, "567"),
            self._wrote("AMS", "690.0", 87)]

    def test_the_ht_single_bay_behaves_the_same(self, monkeypatch):
        unit, client = self._capped(monkeypatch, 69, bay=0, tag_pct=70,
                                    name="Bambu_AMS_HT_1", model="ht")
        assert unit.lanes["lane15"].weight == 690
        assert client.calls == [("set_remaining_weight", (87, 690.0))]
        assert unit.logger.messages == [
            self._capsample("Bambu_AMS_HT_1", 0, "lane15", "87", "''", "1.24",
                            69, "capscan", 1000, 690, "567"),
            self._wrote("Bambu_AMS_HT_1", "690.0", 87)]

    @pytest.mark.parametrize("nominal, grams", [(1000, 690), (250, 172),
                                                (750, 517)])
    def test_the_percent_is_applied_against_each_units_own_nominal(
            self, nominal, grams, monkeypatch):
        # 69% of 250 g is 172.5 and of 750 g is 517.5, both rounded down.
        unit, client = self._capped(monkeypatch, 69, bay=0, nominal=nominal,
                                    tag_pct=70)
        assert unit.lanes["lane15"].weight == grams
        assert unit._measure._convert_owed == {0: (69, nominal, grams)}
        assert client.calls == [("set_remaining_weight", (87, float(grams)))]
        assert unit.logger.messages == [
            self._capsample("AMS", 0, "lane15", "87", "''", "1.24", 69,
                            "capscan", nominal, grams, "567"),
            self._wrote("AMS", f"{grams}.0", 87)]

    def test_the_cap_follows_the_tags_nominal_weight(self, monkeypatch):
        # A 250 g sample spool caps at 250 g, not 1000 g.
        unit, client = self._capped(monkeypatch, 119, nominal=250)
        assert unit.lanes["lane15"].weight == 250
        assert client.calls == [("set_remaining_weight", (87, 250.0))]
        assert unit.logger.messages == [
            self._capsample("AMS", 2, "lane15", "87", "''", "1.24", 119,
                            "capscan", 250, 250, "978"),
            self._wrote("AMS", "250.0", 87,
                        " -- measured 119%, capped to the spool's 250 g "
                        "nominal")]

    def test_a_measured_weight_is_persisted(self, monkeypatch):
        # An unbound lane has no Spoolman re-hydration: without the save its
        # measured 250 g reverts to the stored 220 g at restart. The bay has
        # no record yet, so the nominal is the 1000 g default.
        unit = brfid_unit(monkeypatch, "BambuAMS_1", section=None,
                          lanes=[LaneSpec("lane15", 0, weight=220.0)],
                          slots=[])
        sp, lane = unit._measure, unit.lanes["lane15"]
        assert sp._adopt_measured_remain(0, 25, "test") is True
        assert lane.weight == 250
        assert unit.afc.save_vars.call_count == 1
        assert sp._measured_remain == {0: 25} and sp._meas_seq_seen == {}
        assert unit.logger.messages == [
            self._capsample("BambuAMS_1", 0, "lane15", "None", "''", "1.24",
                            25, "test", 1000, 250, "206")]

    def test_a_lost_measurement_memo_is_rebuilt(self, monkeypatch):
        # 63% of the 1000 g nominal is 630 g (no material: tag-linear); the
        # 1.24 model would say 1.24 x 663 x 63% = 517.9.
        client = FakeSpoolmanClient()
        unit = brfid_unit(monkeypatch, "AMS", client=client,
                          slots=[{}, {}, slot_info(2, weight=1000)],
                          lanes=[LaneSpec("lane15", 2, spool_id=87)])
        sp = unit._measure
        sp._measured_remain = {5: 40}
        del sp._measured_remain
        assert sp._adopt_measured_remain(2, 63, "capscan", seq=1) is True
        assert sp._measured_remain == {2: 63}
        assert unit.lanes["lane15"].weight == 630
        assert client.calls == [("set_remaining_weight", (87, 630.0))]
        assert unit.logger.messages == [
            self._capsample("AMS", 2, "lane15", "87", "''", "1.24", 63,
                            "capscan", 1000, 630, "518"),
            self._wrote("AMS", "630.0", 87)]

    def test_no_owed_memos_owe_nothing(self, monkeypatch):
        # With the owed memos gone the reading still lands and is saved.
        client = FakeSpoolmanClient()
        unit = brfid_unit(monkeypatch, "AMS", client=client,
                          slots=[{}, {}, slot_info(2, weight=1000)],
                          lanes=[LaneSpec("lane15", 2, spool_id=87)])
        sp = unit._measure
        sp._bind_owed = None
        sp._convert_owed = None
        assert sp._adopt_measured_remain(2, 63, "capscan", seq=1) is True
        assert sp._bind_owed is None and sp._convert_owed is None
        assert unit.lanes["lane15"].weight == 630
        assert unit.afc.save_vars.call_count == 1
        assert client.calls == [("set_remaining_weight", (87, 630.0))]
        assert unit.logger.messages == [
            self._capsample("AMS", 2, "lane15", "87", "''", "1.24", 63,
                            "capscan", 1000, 630, "518"),
            self._wrote("AMS", "630.0", 87)]

    # ── one reel's figure only ever comes down ──
    # The bay's scan is still open, so each summary is held, carrying the
    # percent it will report.

    @staticmethod
    def _reel(monkeypatch: pytest.MonkeyPatch,
              uid: Optional[str] = "c32a080a") -> afcBambuAMS:
        """
        Bambu_AMS_1's bay 0, an untyped 1 kg reel with no lane, measured with
        Spoolman off.

        :param monkeypatch: pytest's monkeypatch fixture
        :param uid: the reel's chip UID
        :return afcBambuAMS: the unit
        """
        unit = brfid_unit(monkeypatch, section=None,
                          slots=[slot_info(0, weight=1000, uid=uid)])
        unit._scan_t0[0] = 100.0
        return unit

    def _measured(self, pct: int) -> Tuple[str, str]:
        """
        :param pct: the percent reported
        :return tuple: bay 0's capsample row for it
        """
        modelled = {120: "987", 138: "1135", 145: "1192"}[pct]
        return self._capsample("Bambu_AMS_1", 0, "?", "None", "''", "1.24",
                               pct, "physical AMS measurement", 1000, 1000,
                               modelled)

    @staticmethod
    def _held(read: int, floor: int) -> Tuple[str, str]:
        """
        :param read: the raw reading
        :param floor: the reel's floor it is held at
        :return tuple: the line saying so
        """
        return ("debug", f"AFC bambu Bambu_AMS_1: slot 0 measured {read}%, "
                         f"held at the {floor}% this reel measured before -- "
                         f"filament does not grow, and the odometer is worth "
                         f"about +/-3%")

    def test_a_higher_reading_does_not_raise_the_figure(self, monkeypatch):
        unit = self._reel(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 138, "physical AMS measurement", seq=1)
        assert sp._pending_summary == {0: (138, 1000, 1000, 145.0)}
        assert unit.logger.messages == [self._measured(138)]
        unit.logger.messages.clear()
        sp._adopt_measured_remain(0, 145, "physical AMS measurement", seq=2)
        assert sp._pending_summary == {0: (138, 1000, 1000, 145.0)}
        assert sp._remain_floor == {"c32a080a": 138}
        assert unit.logger.messages == [self._held(145, 138),
                                        self._measured(138)]

    def test_a_lower_reading_is_taken_and_becomes_the_new_floor(
            self, monkeypatch):
        unit = self._reel(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 138, "physical AMS measurement", seq=1)
        unit.logger.messages.clear()
        sp._adopt_measured_remain(0, 120, "physical AMS measurement", seq=2)
        assert sp._pending_summary == {0: (120, 1000, 1000, 145.0)}
        assert sp._remain_floor == {"c32a080a": 120}
        assert unit.logger.messages == [self._measured(120)]
        unit.logger.messages.clear()
        sp._adopt_measured_remain(0, 138, "physical AMS measurement", seq=3)
        assert sp._pending_summary == {0: (120, 1000, 1000, 145.0)}
        assert sp._remain_floor == {"c32a080a": 120}
        assert unit.logger.messages == [self._held(138, 120),
                                        self._measured(120)]

    def test_the_raw_reading_is_still_the_measurements_identity(
            self, monkeypatch):
        # Storing the floored figure here would make every held reading look
        # new; the operator line reads the raw one too.
        unit = self._reel(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 138, "physical AMS measurement", seq=1)
        sp._adopt_measured_remain(0, 145, "physical AMS measurement", seq=2)
        assert sp._measured_remain == {0: 145}
        assert sp._summary_read == {0: 145}
        assert unit.logger.messages == [self._measured(138), self._held(145, 138),
                                        self._measured(138)]

    def test_the_floor_belongs_to_the_reel_not_the_bay(self, monkeypatch):
        # A flipped reel, or the next spool in the bay, has another chip UID
        # and does not inherit this one's floor.
        unit = self._reel(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 138, "physical AMS measurement", seq=1)
        unit._slots[0] = slot_info(0, weight=1000, uid="95f2c30c")
        unit.logger.messages.clear()
        sp._adopt_measured_remain(0, 145, "physical AMS measurement", seq=2)
        assert sp._pending_summary == {0: (145, 1000, 1000, 145.0)}
        assert sp._remain_floor == {"c32a080a": 138, "95f2c30c": 145}
        assert unit.logger.messages == [self._measured(145)]

    def test_a_bay_with_no_uid_is_not_floored(self, monkeypatch):
        # Nothing identifies the reel, so nothing is known of its past.
        unit = self._reel(monkeypatch, uid=None)
        sp = unit._measure
        sp._adopt_measured_remain(0, 138, "physical AMS measurement", seq=1)
        unit.logger.messages.clear()
        sp._adopt_measured_remain(0, 145, "physical AMS measurement", seq=2)
        assert sp._pending_summary == {0: (145, 1000, 1000, 145.0)}
        assert "_remain_floor" not in vars(sp)
        assert unit.logger.messages == [self._measured(145)]

    # ── one measurement, one adoption, decided by meas_seq ──

    @staticmethod
    def _blank(monkeypatch: pytest.MonkeyPatch, bays: int = 1) -> afcBambuAMS:
        """
        Bambu_AMS_1 with ``bays`` blank 1 kg records, no lanes, Spoolman off.
        A blank record holds each summary, so only the capsample row is said.

        :param monkeypatch: pytest's monkeypatch fixture
        :param bays: how many bays carry a record
        :return afcBambuAMS: the unit
        """
        return brfid_unit(monkeypatch, section=None,
                          slots=[slot_info(i, weight=1000)
                                 for i in range(bays)])

    def _row(self, pct: int, source: str = "physical AMS measurement",
             slot: int = 0) -> Tuple[str, str]:
        """
        :param pct: the reading adopted
        :param source: what produced it
        :param slot: the bay
        :return tuple: its capsample row, grams tag-linear
        """
        modelled = {60: "493", 77: "633", 92: "756", 148: "1217",
                    200: "1644"}[pct]
        return self._capsample("Bambu_AMS_1", slot, "?", "None", "''", "1.24",
                               pct, source, 1000, min(pct, 100) * 10,
                               modelled)

    @pytest.mark.parametrize("pct", [0, 201])
    def test_a_reading_outside_the_sane_range_is_refused(self, pct,
                                                         monkeypatch):
        unit = self._blank(monkeypatch)
        sp = unit._measure
        assert sp._adopt_measured_remain(0, pct, "capscan", seq=1) is False
        assert sp._measured_remain == {}
        assert "_meas_seq_seen" not in vars(sp)
        assert unit.afc.save_vars.call_count == 0
        assert unit.logger.messages == []

    def test_the_top_of_the_sane_range_is_kept(self, monkeypatch):
        # A full ABS reel can read over 150%; the grams are capped anyway.
        unit = self._blank(monkeypatch)
        sp = unit._measure
        assert sp._adopt_measured_remain(0, 200, seq=1) is True
        assert sp._measured_remain == {0: 200}
        assert unit.logger.messages == [self._row(200, "capscan")]

    def test_the_same_cycle_is_adopted_once(self, monkeypatch):
        unit = self._blank(monkeypatch)
        sp = unit._measure
        assert sp._adopt_measured_remain(
            0, 148, "physical AMS measurement", seq=7) is True
        assert unit.logger.messages == [self._row(148)]
        unit.logger.messages.clear()
        # The same seq again: a re-delivered frame, not a new measurement.
        assert sp._adopt_measured_remain(
            0, 148, "physical AMS measurement", seq=7) is True
        assert sp._measured_remain == {0: 148} and sp._meas_seq_seen == {0: 7}
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == []

    def test_a_stale_reading_cannot_overwrite_a_sequenced_one(
            self, monkeypatch):
        # 148% is this insert; 127% is this morning's, arriving with no seq
        # of its own from the narration scrape.
        unit = self._blank(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 148, "physical AMS measurement", seq=7)
        unit.logger.messages.clear()
        assert sp._adopt_measured_remain(0, 127, "narration") is True
        assert sp._measured_remain == {0: 148}
        assert sp._pending_summary == {0: (148, 1000, 1000, 145.0)}
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == []

    def test_a_new_cycle_is_adopted(self, monkeypatch):
        unit = self._blank(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 148, "physical AMS measurement", seq=7)
        unit.logger.messages.clear()
        assert sp._adopt_measured_remain(
            0, 92, "physical AMS measurement", seq=8) is True
        assert sp._measured_remain == {0: 92} and sp._meas_seq_seen == {0: 8}
        assert sp._pending_summary == {0: (92, 920, 1000, 145.0)}
        assert unit.afc.save_vars.call_count == 2
        assert unit.logger.messages == [self._row(92)]

    def test_the_unsequenced_path_still_establishes_a_first_reading(
            self, monkeypatch):
        # Older firmware publishes no meas_seq; the narration scrape may
        # still record the first value.
        unit = self._blank(monkeypatch)
        sp = unit._measure
        assert sp._adopt_measured_remain(0, 77, "narration") is True
        assert sp._measured_remain == {0: 77} and sp._meas_seq_seen == {}
        assert sp._pending_summary == {0: (77, 770, 1000, 145.0)}
        assert unit.logger.messages == [self._row(77, "narration")]

    def test_a_different_number_is_not_proof_of_a_new_measurement(
            self, monkeypatch):
        unit = self._blank(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 148, "physical AMS measurement", seq=7)
        unit.logger.messages.clear()
        # In range and unsequenced: refused as no new measurement.
        assert sp._adopt_measured_remain(0, 150, "narration") is True
        # Out of range: refused before identity is even asked.
        assert sp._adopt_measured_remain(0, 999, "narration") is False
        assert sp._measured_remain == {0: 148}
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == []

    def test_slots_do_not_share_a_sequence(self, monkeypatch):
        unit = self._blank(monkeypatch, bays=2)
        sp = unit._measure
        sp._adopt_measured_remain(0, 148, "physical AMS measurement", seq=7)
        sp._adopt_measured_remain(1, 60, "physical AMS measurement", seq=7)
        assert sp._measured_remain == {0: 148, 1: 60}
        assert sp._meas_seq_seen == {0: 7, 1: 7}
        assert unit.logger.messages == [self._row(148),
                                        self._row(60, slot=1)]

    # ── the capscan scrape and the meas_seq record adopt the same reading ──

    @staticmethod
    def _basic(monkeypatch: pytest.MonkeyPatch) -> afcBambuAMS:
        """
        Bambu_AMS_1's bay 0 holding a PLA Basic reel, no lane, Spoolman off:
        each adoption says its summary at once.

        :param monkeypatch: pytest's monkeypatch fixture
        :return afcBambuAMS: the unit
        """
        return brfid_unit(monkeypatch, section=None,
                          slots=[slot_info(0, material="PLA Basic",
                                           weight=1000)])

    def _announced(self, pct: int, source: str) -> List[Tuple[str, str]]:
        """
        :param pct: the reading
        :param source: what produced it
        :return list: the capsample row and the summary for it
        """
        # PLA Basic at 1.24 g/cm3: 89% is 731.7 g and 74% is 608.4 g.
        grams = {89: 732, 74: 608}[pct]
        return [
            self._capsample("Bambu_AMS_1", 0, "?", "None", "'PLA Basic'",
                            "1.24", pct, source, 1000, grams, f"{grams}"),
            ("info", f"Bambu_AMS_1 bay 0: tag read: PLA Basic. Measured about "
                     f"{pct}% left -- roughly {grams} g of a 1000 g spool; "
                     f"kept on the lane -- the Spoolman module "
                     f"([AFC_BambuAMS_rfid]) is off.")]

    def test_capscan_then_meas_seq_announces_once(self, monkeypatch):
        # The order that produced the duplicate on lane14.
        unit = self._basic(monkeypatch)
        sp = unit._measure
        assert sp._adopt_measured_remain(0, 89, "capscan") is True
        assert unit.logger.messages == self._announced(89, "capscan")
        unit.logger.messages.clear()
        assert sp._adopt_measured_remain(
            0, 89, "physical AMS measurement", seq=2) is True
        assert sp._meas_seq_seen == {0: 2}
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == []

    def test_meas_seq_then_capscan_announces_once(self, monkeypatch):
        unit = self._basic(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 89, "physical AMS measurement", seq=2)
        assert unit.logger.messages == self._announced(
            89, "physical AMS measurement")
        unit.logger.messages.clear()
        assert sp._adopt_measured_remain(0, 89, "capscan") is True
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == []

    def test_a_genuinely_new_measurement_still_announces(self, monkeypatch):
        unit = self._basic(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 89, "physical AMS measurement", seq=2)
        unit.logger.messages.clear()
        sp._adopt_measured_remain(0, 74, "physical AMS measurement", seq=3)
        assert sp._measured_remain == {0: 74}
        assert unit.afc.save_vars.call_count == 2
        assert unit.logger.messages == self._announced(
            74, "physical AMS measurement")

    def test_the_same_value_under_a_new_sequence_is_not_new_news(
            self, monkeypatch):
        # A re-measure landing on the same percent is the other path
        # catching up, not a second reading to announce.
        unit = self._basic(monkeypatch)
        sp = unit._measure
        sp._adopt_measured_remain(0, 89, "physical AMS measurement", seq=2)
        unit.logger.messages.clear()
        assert sp._adopt_measured_remain(
            0, 89, "physical AMS measurement", seq=3) is True
        assert sp._meas_seq_seen == {0: 3}
        assert unit.afc.save_vars.call_count == 1
        assert unit.logger.messages == []


class TestAFCBambuAMSRFIDInit:
    """The section reads its one option, so Klipper accepts it."""

    class _TrackingConfig(BambuConfig):
        """The section's config, recording which options were read."""

        def __init__(self, printer: Any, values: Dict[str, Any]) -> None:
            """
            :param printer: the printer
            :param values: the section's options
            """
            super().__init__("AFC_BambuAMS_rfid", printer, values)
            self.reads: List[str] = []

        def getboolean(self, option: str, *args: Any, **kwargs: Any) -> Any:
            """:return Any: the option, recording the read"""
            self.reads.append(option)
            return super().getboolean(option, *args, **kwargs)

    def test_the_section_reads_at_least_one_option(self, monkeypatch):
        # Klipper's check_unused only finds a mixed-case section through
        # access_tracking, which get_printer()/get_name() do not feed; with no
        # option read, [AFC_BambuAMS_rfid] is "not a valid config section".
        printer = make_printer(monkeypatch=monkeypatch)
        cfg = self._TrackingConfig(printer, {})
        section = AFC_BambuAMS_RFID(cfg)
        assert cfg.reads == ["enabled"]
        assert section.printer is printer
        assert section.name == "AFC_BambuAMS_rfid"
        assert section.enabled is True
        assert section._units == set()

    def test_enabled_is_read_from_the_section(self, monkeypatch):
        printer = make_printer(monkeypatch=monkeypatch)
        cfg = self._TrackingConfig(printer, {"enabled": False})
        section = AFC_BambuAMS_RFID(cfg)
        assert cfg.reads == ["enabled"]
        assert section.enabled is False


class TestAFCBambuAMSRFIDGetStatus:
    """The section is visible to the API, with the units it serves."""

    def test_get_status_makes_the_object_visible(self, monkeypatch):
        # objects/list only lists objects with get_status: without it the
        # section loads and is still invisible to the API.
        section = make_spoolman_section(make_printer(monkeypatch=monkeypatch))
        assert section.get_status(0.0) == {"enabled": True, "units": []}

    def test_get_status_names_the_units_that_wired_up(self, monkeypatch):
        printer = make_printer(monkeypatch=monkeypatch)
        section = make_spoolman_section(printer)
        ht = make_bambu_unit("Bambu_AMS_HT_1", printer=printer, model="ht")
        ams = make_bambu_unit("Bambu_AMS_1", printer=printer)
        section.for_unit(ht)
        section.for_unit(ams)
        assert section.get_status() == {
            "enabled": True, "units": ["Bambu_AMS_1", "Bambu_AMS_HT_1"]}

    def test_get_status_reports_a_disabled_section(self, monkeypatch):
        # A disabled section claims no unit it never served.
        printer = make_printer(monkeypatch=monkeypatch)
        section = make_spoolman_section(printer, enabled=False)
        section.for_unit(make_bambu_unit("Bambu_AMS_1", printer=printer))
        assert section.get_status() == {"enabled": False, "units": []}


class TestAFCBambuAMSRFIDForUnit:
    """Each unit gets its own Spoolman delegate, or None and a reason."""

    class _Nameless:
        """A unit nothing can name yet."""

        @property
        def name(self) -> str:
            """:raises RuntimeError: always"""
            raise RuntimeError("not configured yet")

    class _NoAfcRfid:
        """An import hook that makes extras.AFC_RFID unimportable."""

        def find_spec(self, fullname: str, path: Any = None,
                      target: Any = None) -> None:
            """
            :param fullname: the module being imported
            :param path: the package path (unused)
            :param target: the module being reloaded (unused)
            :raises ImportError: for extras.AFC_RFID
            """
            if fullname == "extras.AFC_RFID":
                raise ImportError("AFC_RFID is not deployed")

    @staticmethod
    @contextlib.contextmanager
    def _without_afc_rfid() -> Iterator[Any]:
        """
        Re-import this module with extras.AFC_RFID absent, as on a printer
        that never deployed it, and put everything back on exit.

        The unit imports the module lazily (``_measure``), so the fresh one
        is what it builds from while the ``with`` lasts.

        :yields module: the fresh extras.AFC_BambuAMS_rfid
        """
        names = ("extras.AFC_RFID", "extras.AFC_BambuAMS_rfid")
        saved = {n: sys.modules[n] for n in names if n in sys.modules}
        # The import rebinds the package attributes too.
        saved_attrs = {n: getattr(extras, n.rsplit(".", 1)[1])
                       for n in names
                       if hasattr(extras, n.rsplit(".", 1)[1])}
        for n in names:
            sys.modules.pop(n, None)
        blocker = TestAFCBambuAMSRFIDForUnit._NoAfcRfid()
        sys.meta_path.insert(0, blocker)
        try:
            yield importlib.import_module("extras.AFC_BambuAMS_rfid")
        finally:
            sys.meta_path.remove(blocker)
            for n in names:
                sys.modules.pop(n, None)
            sys.modules.update(saved)
            for n, mod in saved_attrs.items():
                setattr(extras, n.rsplit(".", 1)[1], mod)

    def test_for_unit_hands_back_a_delegate_for_that_unit(self, monkeypatch):
        printer = make_printer(monkeypatch=monkeypatch)
        section = make_spoolman_section(printer)
        unit = make_bambu_unit("Bambu_AMS_1", printer=printer)
        unit.logger.messages.clear()
        sp = section.for_unit(unit)
        assert isinstance(sp, BambuSpoolman)
        assert sp._u is unit and sp.spoolman_on is True
        assert section._units == {"Bambu_AMS_1"}
        # Every call builds a delegate; the unit keeps the first it gets.
        again = section.for_unit(unit)
        assert again is not sp and again._u is unit
        assert section._units == {"Bambu_AMS_1"}
        assert unit.logger.messages == []

    def test_enabled_false_keeps_the_section_but_switches_spoolman_off(
            self, monkeypatch):
        printer = make_printer(monkeypatch=monkeypatch)
        section = make_spoolman_section(printer, enabled=False)
        unit = make_bambu_unit("Bambu_AMS_1", printer=printer)
        unit.logger.messages.clear()
        assert section.enabled is False
        assert section.for_unit(unit) is None
        assert section._units == set()
        assert unit.logger.messages == []

    def test_a_unit_that_cannot_be_named_is_still_served(self, monkeypatch):
        # Listing the unit is bookkeeping: it never costs the unit Spoolman.
        section = make_spoolman_section(make_printer(monkeypatch=monkeypatch))
        unit = self._Nameless()
        sp = section.for_unit(unit)
        assert isinstance(sp, BambuSpoolman) and sp._u is unit
        assert section._units == set()
        assert section._warned_no_afc_rfid is False

    def test_with_afc_rfid_absent_the_section_says_why_once(self, monkeypatch):
        # The section asks for Spoolman and cannot have it: say so once for
        # the printer, not once per AMS, and measure anyway.
        printer = make_printer(monkeypatch=monkeypatch)
        unit1 = make_bambu_unit(
            "Bambu_AMS_1", printer=printer, lanes=[LaneSpec("lane8", 1)],
            slots=[slot_info(0, present=False),
                   slot_info(1, material="ABS", weight=1000)])
        unit2 = make_bambu_unit("Bambu_AMS_2", printer=printer)
        unit1.logger.messages.clear()
        with self._without_afc_rfid() as fresh:
            cfg = BambuConfig("AFC_BambuAMS_rfid", printer, {})
            section = fresh.AFC_BambuAMS_RFID(cfg)
            printer.add_object("AFC_BambuAMS_rfid", section)
            assert unit1._spool is None and unit2._spool is None
            sp = unit1._measure
            assert isinstance(sp, fresh.BambuSpoolman)
            assert sp.spoolman_on is False
            # No density table without AFC_RFID: 63% of 1000 g, tag-linear.
            assert sp._adopt_measured_remain(1, 63, "capscan", seq=1) is True
        assert fresh._AFC_RFID_ERR == "ImportError: AFC_RFID is not deployed"
        assert section._warned_no_afc_rfid is True
        assert section.get_status() == {"enabled": True, "units": []}
        assert unit1.lanes["lane8"].weight == 630
        assert unit1.afc.save_vars.call_count == 1
        # One logger for the printer, so both units' asks land here.
        assert unit2.logger is unit1.logger
        assert unit1.logger.messages == [
            ("warning", "AFC_BambuAMS_rfid: Spoolman is off for every AMS "
                        "unit -- AFC_RFID could not be loaded (ImportError: "
                        "AFC_RFID is not deployed). Measurements are still "
                        "kept on the lanes."),
            ("debug", "capsample unit=Bambu_AMS_1 slot=1 lane=lane8 "
                      "spool=None material='ABS' density=None pct_raw=63 "
                      "radius_m=None circ_m=None save_r_m=None "
                      "radius_src=none source='capscan' nominal=1000 "
                      "grams_written=630 grams_modelled=None"),
            ("info", "Bambu_AMS_1 lane8: tag read: ABS. Measured about 63% "
                     "left -- roughly 630 g of a 1000 g spool; kept on the "
                     "lane -- the Spoolman module ([AFC_BambuAMS_rfid]) is "
                     "off.")]


class TestLoadConfig:
    """Klipper's entry point builds the section object."""

    def test_load_config_builds_the_object(self, monkeypatch):
        printer = make_printer(monkeypatch=monkeypatch)
        section = rfid_mod.load_config(
            BambuConfig("AFC_BambuAMS_rfid", printer, {}))
        assert isinstance(section, AFC_BambuAMS_RFID)
        assert section.printer is printer
        assert section.enabled is True
        assert section.get_status() == {"enabled": True, "units": []}


class TestBambuSpoolmanClass:
    """One Spoolman worker serves every unit on the printer."""

    def test_the_worker_queue_is_shared_by_every_unit(self, monkeypatch):
        # The jobs are HTTP calls kept off the reactor; they serialize fine,
        # so a second AMS reuses the first one's queue and thread.
        monkeypatch.setattr(BambuSpoolman, "_spool_q", None)
        monkeypatch.setattr(BambuSpoolman, "_spool_t", None)
        printer = make_printer(monkeypatch=monkeypatch)
        make_spoolman_section(printer)
        delegates = [
            make_bambu_spoolman(make_bambu_unit(name, printer=printer),
                                inline=False)
            for name in ("Bambu_AMS_1", "Bambu_AMS_2")]
        assert delegates[0] is not delegates[1]
        seen: List[str] = []
        done = threading.Event()

        def job() -> None:
            """Record the thread the job runs on."""
            seen.append(threading.current_thread().name)
            done.set()

        delegates[0]._spoolman_bg(job)
        assert done.wait(5.0)
        first = (BambuSpoolman._spool_q, BambuSpoolman._spool_t)
        done.clear()
        delegates[1]._spoolman_bg(job)
        assert done.wait(5.0)
        assert seen == ["afc_bambu_spool", "afc_bambu_spool"]
        assert (BambuSpoolman._spool_q, BambuSpoolman._spool_t) == first
        # Class state, not per-delegate state.
        assert "_spool_q" not in vars(delegates[0])
        assert "_spool_q" not in vars(delegates[1])
