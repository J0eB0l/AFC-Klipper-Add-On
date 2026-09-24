"""Unit tests for extras/AFC_RFID.py."""

from __future__ import annotations

import copy
import json
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple
from urllib.request import Request

import pytest

from extras.AFC_ACE2_rfid import AFC_ACE2_RFID
from extras.AFC_RFID import (
    _cached_spoolman_client,
    _decode_extra,
    _missing_filament_fields,
    _norm_tray_uid,
    _norm_uid,
    _spool_uids,
    apply_filament_defaults,
    bed_temp_for_material,
    build_filament_name,
    default_bed_temp_for_material,
    density_for_material,
    dismiss_prompt,
    enrich_from_spool,
    find_spool_by_tray_uid,
    find_spool_by_uid,
    format_tag_summary,
    get_auto_spoolman_create,
    log_new_filament,
    log_new_spool,
    make_tag_record,
    map_tag_to_slot_info,
    match_spool_for_tag,
    prompt_hold_spool,
    resolve_rfid_keys,
    rgb_array_to_hex,
    SpoolmanClient,
    sync_rfid_to_spoolman,
)
from extras.AFC_lane import AFCLane
from extras.AFC_rfid_keys import AFC_rfid_keys
from tests.ace_helpers import (
    AceConfig,
    AceLogger,
    AcePrinter,
    Hook,
    make_ace2_rfid,
    make_ace2_unit,
    Recorder,
)


ProxyCall = Tuple[str, str, Any, bool]


class RfidSpoolmanMoonraker:
    """
    AFC_moonraker stand-in for SpoolmanClient. _get_results is the HTTP edge.

    A moonraker Spoolman proxy request is decoded and answered from
    routes[(method, path)]: an exception instance is raised, a callable is
    called with the request body, anything else is returned as a deep copy.
    An unknown route answers None, as a failed request does. Each proxy
    request lands in .calls as (method, path, body, print_error) and in
    .requests as (url, content type, decoded payload); any other fetch lands
    in .fetches as (url, print_error) and answers fetch_result.
    """

    def __init__(self, routes: Optional[Dict[Tuple[str, str], Any]] = None, *,
                 logger: Optional[AceLogger] = None, fetch_result: Any = None) -> None:
        """
        :param routes: answers keyed by (method, path)
        :param logger: AFC logger, a new AceLogger when None
        :param fetch_result: answer to a fetch that is not a proxy request
        """
        self.host = "http://moonraker:7125/"
        self.logger = logger if logger is not None else AceLogger()
        self.routes: Dict[Tuple[str, str], Any] = dict(routes or {})
        self.fetch_result = fetch_result
        self.calls: List[ProxyCall] = []
        self.requests: List[Tuple[str, Optional[str], Dict[str, Any]]] = []
        self.fetches: List[Tuple[Any, bool]] = []
        self._write_queue: Any = None

    def _get_results(self, url_string: Any, print_error: bool = True) -> Any:
        """
        :param url_string: URL string or urllib Request
        :param print_error: moonraker's flag, recorded
        :return Any: the routed answer
        """
        if not isinstance(url_string, Request):
            self.fetches.append((url_string, print_error))
            return self.fetch_result
        payload = json.loads(url_string.data.decode("utf-8"))
        self.requests.append((url_string.full_url, url_string.get_header("Content-type"),
                              payload))
        method, path = payload["request_method"], payload["path"]
        body = payload.get("body")
        self.calls.append((method, path, body, print_error))
        answer = self.routes.get((method, path))
        if isinstance(answer, BaseException):
            raise answer
        if callable(answer):
            return answer(body)
        return copy.deepcopy(answer)

    def sent(self, method: str, path: str) -> List[Any]:
        """
        :param method: HTTP method
        :param path: Spoolman path
        :return list: the body of every request made to that route, in order
        """
        return [body for m, p, body, _ in self.calls if (m, p) == (method, path)]


def rfid_spoolman_client(routes: Optional[Dict[Tuple[str, str], Any]] = None
                         ) -> Tuple[SpoolmanClient, RfidSpoolmanMoonraker]:
    """
    Build a real SpoolmanClient around an RfidSpoolmanMoonraker.

    :param routes: the moonraker's Spoolman answers
    :return tuple: (client, moonraker)
    """
    moonraker = RfidSpoolmanMoonraker(routes)
    return SpoolmanClient(moonraker), moonraker


def rfid_lane(name: str = "lane1") -> AFCLane:
    """
    Build a real AFCLane on an ACE 2 unit, through the ACE builders.

    :param name: lane name
    :return AFCLane: the connected lane, fresh from its __init__
    """
    unit = make_ace2_unit(lanes=[name])
    return unit.printer.afc.lanes[name]


def rfid_reader(lane_name: str = "lane1") -> Tuple[AFC_ACE2_RFID, AFCLane]:
    """
    Build a real AFC_ACE2_RFID, the AFCUnitRFID production subclass, bound to
    an ACE 2 unit with one lane. Its afc, logger and gcode are the printer's.

    :param lane_name: the unit's lane
    :return tuple: (reader, lane)
    """
    unit = make_ace2_unit(lanes=[lane_name])
    reader = make_ace2_rfid(ace2=unit)
    return reader, unit.printer.afc.lanes[lane_name]


@pytest.fixture
def rfid_wall_clock(monkeypatch: pytest.MonkeyPatch) -> List[float]:
    """
    Hold time.time at a settable wall-clock value.

    :param monkeypatch: pytest monkeypatch
    :return list: one-element list holding the current time, 1000.0 to start
    """
    now = [1000.0]
    monkeypatch.setattr(time, "time", lambda: now[0])
    return now


class TestSpoolmanClientGetResults:
    def test_delegates_to_moonraker(self):
        client, moonraker = rfid_spoolman_client()
        moonraker.fetch_result = "R"
        assert client._get_results("http://x", print_error=False) == "R"
        assert moonraker.fetches == [("http://x", False)]
        assert moonraker.calls == []
        assert moonraker.logger.messages == []


class TestSpoolmanClientSpoolmanProxy:
    _URL = "http://moonraker:7125/server/spoolman/proxy"

    def test_get_none_result_does_not_log(self):
        client, moonraker = rfid_spoolman_client()
        assert client._spoolman_proxy("GET", "/v1/info") is None
        assert moonraker.requests == [
            (self._URL, "application/json",
             {"request_method": "GET", "path": "/v1/info"})]
        assert moonraker.calls == [("GET", "/v1/info", None, True)]
        assert moonraker.logger.messages == []

    def test_non_get_none_result_logs_decoded_body(self):
        client, moonraker = rfid_spoolman_client()
        assert client._spoolman_proxy("POST", "/v1/x", body='{"a": 1}') is None
        # The JSON string goes upstream as an object, so moonraker sets the type.
        assert moonraker.requests == [
            (self._URL, "application/json",
             {"request_method": "POST", "path": "/v1/x", "body": {"a": 1}})]
        assert moonraker.logger.messages == [
            ("error", 'Spoolman POST /v1/x failed; request body: {"a": 1}')]

    def test_invalid_json_string_body_kept_as_string(self):
        client, moonraker = rfid_spoolman_client()
        assert client._spoolman_proxy("POST", "/v1/x", body="nothex") is None
        assert moonraker.calls == [("POST", "/v1/x", "nothex", True)]
        assert moonraker.logger.messages == [
            ("error", 'Spoolman POST /v1/x failed; request body: "nothex"')]

    def test_none_body_reports_none(self):
        client, moonraker = rfid_spoolman_client()
        assert client._spoolman_proxy("PATCH", "/v1/x") is None
        assert moonraker.requests == [
            (self._URL, "application/json",
             {"request_method": "PATCH", "path": "/v1/x"})]
        assert moonraker.logger.messages == [
            ("error", "Spoolman PATCH /v1/x failed; request body: (none)")]

    def test_success_dict_body_returns_result_no_log(self):
        client, moonraker = rfid_spoolman_client({("POST", "/v1/x"): {"ok": 1}})
        assert client._spoolman_proxy("POST", "/v1/x", body={"a": 1},
                                      print_error=False) == {"ok": 1}
        assert moonraker.calls == [("POST", "/v1/x", {"a": 1}, False)]
        assert moonraker.logger.messages == []


class TestSpoolmanClientLoggingTo:
    def test_this_thread_logs_to_the_sink_for_the_block_only(self):
        client, moonraker = rfid_spoolman_client()
        sink = AceLogger()
        with client.logging_to(sink):
            assert client._spoolman_proxy("PATCH", "/v1/a") is None
        assert client._spoolman_proxy("PATCH", "/v1/b") is None
        assert sink.messages == [
            ("error", "Spoolman PATCH /v1/a failed; request body: (none)")]
        assert moonraker.logger.messages == [
            ("error", "Spoolman PATCH /v1/b failed; request body: (none)")]

    def test_another_thread_keeps_the_real_logger(self):
        client, moonraker = rfid_spoolman_client()
        sink = AceLogger()
        with client.logging_to(sink):
            other = threading.Thread(
                target=lambda: client._spoolman_proxy("POST", "/v1/c"))
            other.start()
            other.join()
        assert sink.messages == []
        assert moonraker.logger.messages == [
            ("error", "Spoolman POST /v1/c failed; request body: (none)")]

    def test_nested_blocks_restore_the_outer_sink(self):
        client, moonraker = rfid_spoolman_client()
        outer, inner = AceLogger(), AceLogger()
        with client.logging_to(outer):
            with client.logging_to(inner):
                client._spoolman_proxy("PATCH", "/v1/in")
            client._spoolman_proxy("PATCH", "/v1/out")
        assert inner.messages == [
            ("error", "Spoolman PATCH /v1/in failed; request body: (none)")]
        assert outer.messages == [
            ("error", "Spoolman PATCH /v1/out failed; request body: (none)")]
        assert moonraker.logger.messages == []


class TestSpoolmanClientReachable:
    def test_true_when_info_returns(self):
        client, moonraker = rfid_spoolman_client({("GET", "/v1/info"): {"version": "1"}})
        assert client.reachable() is True
        assert moonraker.calls == [("GET", "/v1/info", None, False)]
        assert moonraker.logger.messages == []

    def test_false_when_info_none(self):
        client, moonraker = rfid_spoolman_client()
        assert client.reachable() is False
        assert moonraker.calls == [("GET", "/v1/info", None, False)]
        assert moonraker.logger.messages == []

    def test_false_on_exception(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/info"): RuntimeError("down")})
        assert client.reachable() is False
        assert moonraker.calls == [("GET", "/v1/info", None, False)]
        assert moonraker.logger.messages == []


class TestSpoolmanClientSearchSpools:
    def test_no_filter_returns_list(self):
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool"): [{"id": 1}]})
        assert client.search_spools() == [{"id": 1}]
        assert moonraker.calls == [("GET", "/v1/spool", None, False)]
        assert moonraker.logger.messages == []

    def test_filter_by_filament_id(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/spool?filament.id=5"): [{"id": 2}]})
        assert client.search_spools(filament_id=5) == [{"id": 2}]
        assert moonraker.calls == [("GET", "/v1/spool?filament.id=5", None, False)]
        assert moonraker.logger.messages == []

    def test_non_list_returns_empty(self):
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool"): {"not": "list"}})
        assert client.search_spools() == []
        assert moonraker.logger.messages == []


class TestSpoolmanClientGetOrCreateVendor:
    def test_exact_case_insensitive_match(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/vendor?name=Bambu%20Lab"): [
                {"id": 1, "name": "Other"}, {"id": 2, "name": " bambu lab "}]})
        assert client.get_or_create_vendor("Bambu Lab") == {"id": 2, "name": " bambu lab "}
        assert moonraker.calls == [("GET", "/v1/vendor?name=Bambu%20Lab", None, False)]
        assert moonraker.logger.messages == []

    def test_no_exact_match_returns_first(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/vendor?name=Elegoo"): [
                {"id": 3, "name": "Bambu X"}, {"id": 4, "name": "Bambu Y"}]})
        assert client.get_or_create_vendor("Elegoo") == {"id": 3, "name": "Bambu X"}
        assert moonraker.calls == [("GET", "/v1/vendor?name=Elegoo", None, False)]
        assert moonraker.logger.messages == []

    def test_empty_list_creates_vendor(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/vendor?name=New"): [],
            ("POST", "/v1/vendor"): {"id": 9, "name": "New"}})
        assert client.get_or_create_vendor("New") == {"id": 9, "name": "New"}
        assert moonraker.calls == [("GET", "/v1/vendor?name=New", None, False),
                                   ("POST", "/v1/vendor", {"name": "New"}, True)]
        assert moonraker.logger.messages == []

    def test_non_list_response_creates_vendor(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/vendor?name=X"): {"id": 1, "name": "X"},
            ("POST", "/v1/vendor"): {"id": 10}})
        assert client.get_or_create_vendor("X") == {"id": 10}
        assert moonraker.calls == [("GET", "/v1/vendor?name=X", None, False),
                                   ("POST", "/v1/vendor", {"name": "X"}, True)]
        assert moonraker.logger.messages == []

    def test_none_response_creates_vendor(self):
        client, moonraker = rfid_spoolman_client({("POST", "/v1/vendor"): {"id": 10}})
        assert client.get_or_create_vendor("X") == {"id": 10}
        assert moonraker.calls == [("GET", "/v1/vendor?name=X", None, False),
                                   ("POST", "/v1/vendor", {"name": "X"}, True)]
        assert moonraker.logger.messages == []


class TestSpoolmanClientCreateFilament:
    @staticmethod
    def _client() -> Tuple[SpoolmanClient, RfidSpoolmanMoonraker]:
        """
        :return tuple: a client whose filament POST answers {"id": 1}
        """
        return rfid_spoolman_client({("POST", "/v1/filament"): {"id": 1}})

    def test_minimal_only_name(self):
        client, moonraker = self._client()
        assert client.create_filament(name="X") == {"id": 1}
        assert moonraker.calls == [("POST", "/v1/filament", {"name": "X"}, True)]
        assert moonraker.logger.messages == []

    def test_all_scalar_fields(self):
        client, moonraker = self._client()
        client.create_filament(
            name="X", vendor_id=2, material="PLA", density=1.24, diameter=1.75,
            color_hex="#00ff00", settings_extruder_temp=220, settings_bed_temp=60,
            weight=1000, spool_weight=250, article_number="SKU")
        assert moonraker.sent("POST", "/v1/filament") == [{
            "name": "X", "vendor_id": 2, "material": "PLA", "density": 1.24,
            "diameter": 1.75, "color_hex": "00ff00", "settings_extruder_temp": 220,
            "settings_bed_temp": 60, "weight": 1000, "spool_weight": 250,
            "article_number": "SKU"}]
        assert moonraker.logger.messages == []

    def test_multi_color_list_drops_color_hex_default_direction(self):
        client, moonraker = self._client()
        client.create_filament(name="X", color_hex="#aaaaaa",
                               multi_color_hexes=["#aa0000", "00bb00"])
        assert moonraker.sent("POST", "/v1/filament") == [{
            "name": "X", "multi_color_hexes": "aa0000,00bb00",
            "multi_color_direction": "coaxial"}]
        assert moonraker.logger.messages == []

    def test_multi_color_string_and_explicit_direction(self):
        client, moonraker = self._client()
        client.create_filament(name="X", multi_color_hexes="aa0000,00bb00",
                               multi_color_direction="longitudinal")
        assert moonraker.sent("POST", "/v1/filament") == [{
            "name": "X", "multi_color_hexes": "aa0000,00bb00",
            "multi_color_direction": "longitudinal"}]
        assert moonraker.logger.messages == []


class TestSpoolmanClientUpdateFilament:
    def test_empty_fields_noop(self):
        client, moonraker = rfid_spoolman_client()
        assert client.update_filament(5, {}) is None
        assert moonraker.calls == []
        assert moonraker.logger.messages == []

    def test_patches_fields(self):
        client, moonraker = rfid_spoolman_client({("PATCH", "/v1/filament/5"): {"id": 5}})
        assert client.update_filament(5, {"material": "PLA"}) == {"id": 5}
        assert moonraker.calls == [("PATCH", "/v1/filament/5", {"material": "PLA"}, True)]
        assert moonraker.logger.messages == []


class TestSpoolmanClientCreateSpool:
    def test_minimal(self):
        client, moonraker = rfid_spoolman_client({("POST", "/v1/spool"): {"id": 1}})
        assert client.create_spool(filament_id=7) == {"id": 1}
        assert moonraker.calls == [("POST", "/v1/spool", {"filament_id": 7}, True)]
        assert moonraker.logger.messages == []

    def test_all_weights(self):
        client, moonraker = rfid_spoolman_client({("POST", "/v1/spool"): {"id": 1}})
        client.create_spool(filament_id=7, initial_weight=1000,
                            remaining_weight=900, spool_weight=250)
        assert moonraker.sent("POST", "/v1/spool") == [{
            "filament_id": 7, "initial_weight": 1000, "remaining_weight": 900,
            "spool_weight": 250}]
        assert moonraker.logger.messages == []


class TestSpoolmanClientEnsureSpoolFields:
    _GET = ("GET", "/v1/field/spool", None, False)
    _POST = ("POST", "/v1/field/spool/card_uids",
             {"name": "Card UIDs", "field_type": "text"}, True)

    def test_creates_missing_and_caches(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/spool"): [],
            ("POST", "/v1/field/spool/card_uids"): {"ok": 1}})
        assert client._fields_ensured is False
        client._ensure_spool_fields()
        client._ensure_spool_fields()
        assert moonraker.calls == [self._GET, self._POST]
        assert client._fields_ensured is True
        assert moonraker.logger.messages == []

    def test_skips_existing_field(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/field/spool"): [{"key": "card_uids"}]})
        client._ensure_spool_fields()
        assert moonraker.calls == [self._GET]
        assert client._fields_ensured is True
        assert moonraker.logger.messages == []

    def test_non_list_existing_still_creates(self):
        client, moonraker = rfid_spoolman_client(
            {("POST", "/v1/field/spool/card_uids"): {"ok": 1}})
        client._ensure_spool_fields()
        assert moonraker.calls == [self._GET, self._POST]
        assert client._fields_ensured is True
        assert moonraker.logger.messages == []

    def test_ignores_non_dict_entries(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/field/spool"): ["junk", {"key": "card_uids"}]})
        client._ensure_spool_fields()
        assert moonraker.calls == [self._GET]
        assert client._fields_ensured is True
        assert moonraker.logger.messages == []


class TestSpoolmanClientEnsureFlowKField:
    _GET = ("GET", "/v1/field/spool", None, False)
    _POST = ("POST", "/v1/field/spool/flow_k", {"name": "Flow K", "field_type": "float"}, True)

    def test_creates_missing_and_caches(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/spool"): [],
            ("POST", "/v1/field/spool/flow_k"): {"ok": 1}})
        assert client._flow_k_field_ensured is False
        client._ensure_flow_k_field()
        client._ensure_flow_k_field()
        assert moonraker.calls == [self._GET, self._POST]
        assert client._flow_k_field_ensured is True
        assert moonraker.logger.messages == []

    def test_skips_existing_ignoring_non_dict(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/field/spool"): ["junk", {"key": "flow_k"}]})
        client._ensure_flow_k_field()
        assert moonraker.calls == [self._GET]
        assert client._flow_k_field_ensured is True
        assert moonraker.logger.messages == []

    def test_non_list_existing_still_creates(self):
        client, moonraker = rfid_spoolman_client(
            {("POST", "/v1/field/spool/flow_k"): {"ok": 1}})
        client._ensure_flow_k_field()
        assert moonraker.calls == [self._GET, self._POST]
        assert client._flow_k_field_ensured is True
        assert moonraker.logger.messages == []


class TestSpoolmanClientEnsureTrayUidField:
    _GET = ("GET", "/v1/field/spool", None, False)
    _POST = ("POST", "/v1/field/spool/tray_uid",
             {"name": "Tray UID", "field_type": "text"}, True)

    def test_creates_missing_and_caches(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/spool"): [],
            ("POST", "/v1/field/spool/tray_uid"): {"ok": 1}})
        assert client._tray_uid_field_ensured is False
        client._ensure_tray_uid_field()
        client._ensure_tray_uid_field()
        assert moonraker.calls == [self._GET, self._POST]
        assert client._tray_uid_field_ensured is True
        assert moonraker.logger.messages == []

    def test_skips_existing(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/field/spool"): [{"key": "tray_uid"}]})
        client._ensure_tray_uid_field()
        assert moonraker.calls == [self._GET]
        assert client._tray_uid_field_ensured is True
        assert moonraker.logger.messages == []


class TestSpoolmanClientWriteTrayUid:
    def test_writes_the_field_and_keeps_the_others(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/spool"): [{"key": "tray_uid"}],
            ("GET", "/v1/spool/7"): {"id": 7, "extra": {"card_uids": '"AABBCCDD"'}},
            ("PATCH", "/v1/spool/7"): {"id": 7}})
        assert client.write_tray_uid(7, "CF34CF1D212F46B5BC8561E05EB644C8") == {"id": 7}
        # Lower-cased for storage, and card_uids is untouched beside it.
        assert moonraker.calls == [
            ("GET", "/v1/field/spool", None, False),
            ("GET", "/v1/spool/7", None, True),
            ("PATCH", "/v1/spool/7",
             {"extra": {"card_uids": '"AABBCCDD"',
                        "tray_uid": '"cf34cf1d212f46b5bc8561e05eb644c8"'}}, True)]
        assert client._tray_uid_field_ensured is True
        assert moonraker.logger.messages == []

    def test_nothing_to_write_is_a_no_op(self):
        client, moonraker = rfid_spoolman_client()
        assert client.write_tray_uid(7, "") is None
        assert client.write_tray_uid(7, "0" * 32) is None   # an unread field
        assert moonraker.calls == []
        assert client._tray_uid_field_ensured is False
        assert moonraker.logger.messages == []


class TestSpoolmanClientEnsureFilamentFields:
    _GET = ("GET", "/v1/field/filament", None, False)
    _POST = ("POST", "/v1/field/filament/variant",
             {"name": "Variant", "field_type": "text"}, True)

    def test_creates_variant_field_and_caches(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/filament"): [],
            ("POST", "/v1/field/filament/variant"): {"ok": 1}})
        assert client._filament_fields_ensured is False
        client._ensure_filament_fields()
        client._ensure_filament_fields()
        assert moonraker.calls == [self._GET, self._POST]
        assert client._filament_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_skips_existing_ignoring_non_dict(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/field/filament"): ["junk", {"key": "variant"}]})
        client._ensure_filament_fields()
        assert moonraker.calls == [self._GET]
        assert client._filament_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_non_list_existing_still_creates(self):
        client, moonraker = rfid_spoolman_client(
            {("POST", "/v1/field/filament/variant"): {"ok": 1}})
        client._ensure_filament_fields()
        assert moonraker.calls == [self._GET, self._POST]
        assert client._filament_fields_ensured is True
        assert moonraker.logger.messages == []


class TestSpoolmanClientEnsureDryingFields:
    _GET = ("GET", "/v1/field/filament", None, False)
    _POST_TEMP = ("POST", "/v1/field/filament/drying_temp_c",
                  {"name": "Drying temp (C)", "field_type": "integer"}, True)
    _POST_TIME = ("POST", "/v1/field/filament/drying_time_h",
                  {"name": "Drying time (h)", "field_type": "integer"}, True)
    _CREATED = {("POST", "/v1/field/filament/drying_temp_c"): {"ok": 1},
                ("POST", "/v1/field/filament/drying_time_h"): {"ok": 1}}

    def test_creates_only_missing_field(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/field/filament"): [{"key": "drying_time_h"}], **self._CREATED})
        client._ensure_drying_fields()
        assert moonraker.calls == [self._GET, self._POST_TEMP]
        assert client._drying_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_skips_when_both_present_ignoring_non_dict(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/filament"): ["junk", {"key": "drying_temp_c"},
                                            {"key": "drying_time_h"}]})
        client._ensure_drying_fields()
        assert moonraker.calls == [self._GET]
        assert client._drying_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_non_list_existing_creates_both(self):
        client, moonraker = rfid_spoolman_client(dict(self._CREATED))
        client._ensure_drying_fields()
        client._ensure_drying_fields()
        assert moonraker.calls == [self._GET, self._POST_TEMP, self._POST_TIME]
        assert client._drying_fields_ensured is True
        assert moonraker.logger.messages == []


class TestSpoolmanClientWriteFilamentDrying:
    _GET = ("GET", "/v1/field/filament", None, False)
    _POST_TEMP = ("POST", "/v1/field/filament/drying_temp_c",
                  {"name": "Drying temp (C)", "field_type": "integer"}, True)
    _POST_TIME = ("POST", "/v1/field/filament/drying_time_h",
                  {"name": "Drying time (h)", "field_type": "integer"}, True)

    @staticmethod
    def _client(fields: Optional[List[Dict[str, str]]] = None
                ) -> Tuple[SpoolmanClient, RfidSpoolmanMoonraker]:
        """
        :param fields: the filament extra fields Spoolman already has
        :return tuple: a client whose writes all succeed
        """
        return rfid_spoolman_client({
            ("GET", "/v1/field/filament"): list(fields or []),
            ("POST", "/v1/field/filament/drying_temp_c"): {"ok": True},
            ("POST", "/v1/field/filament/drying_time_h"): {"ok": True},
            ("PATCH", "/v1/filament/1"): {"ok": True},
            ("PATCH", "/v1/filament/2"): {"ok": True},
            ("PATCH", "/v1/filament/7"): {"ok": True}})

    def test_noop_when_both_unset(self):
        client, moonraker = self._client()
        assert client.write_filament_drying(1, None, None) is None
        assert moonraker.calls == []                  # not even the ensure GET
        assert client._drying_fields_ensured is False
        assert moonraker.logger.messages == []

    def test_writes_both_fields(self):
        client, moonraker = self._client()
        assert client.write_filament_drying(7, 70, 8) == {"ok": True}
        assert client._drying_fields_ensured is True
        assert moonraker.calls == [
            self._GET, self._POST_TEMP, self._POST_TIME,
            ("PATCH", "/v1/filament/7",
             {"extra": {"drying_temp_c": "70", "drying_time_h": "8"}}, True)]
        assert moonraker.logger.messages == []

    def test_temp_only(self):
        client, moonraker = self._client()
        client.write_filament_drying(7, 65, None)
        assert moonraker.sent("PATCH", "/v1/filament/7") == [
            {"extra": {"drying_temp_c": "65"}}]
        assert client._drying_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_time_only(self):
        client, moonraker = self._client()
        assert client.write_filament_drying(2, None, 6) == {"ok": True}
        assert moonraker.sent("PATCH", "/v1/filament/2") == [
            {"extra": {"drying_time_h": "6"}}]
        assert client._drying_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_noop_when_values_current(self):
        client, moonraker = self._client()
        current = {"drying_temp_c": "70", "drying_time_h": "8"}
        assert client.write_filament_drying(7, 70, 8, current_extra=current) is None
        assert moonraker.calls == [self._GET, self._POST_TEMP, self._POST_TIME]
        assert client._drying_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_ensure_creates_fields_once(self):
        client, moonraker = self._client()
        client.write_filament_drying(1, 70, 8)
        client.write_filament_drying(2, 60, 6)
        assert moonraker.calls == [
            self._GET, self._POST_TEMP, self._POST_TIME,
            ("PATCH", "/v1/filament/1",
             {"extra": {"drying_temp_c": "70", "drying_time_h": "8"}}, True),
            ("PATCH", "/v1/filament/2",
             {"extra": {"drying_temp_c": "60", "drying_time_h": "6"}}, True)]
        assert client._drying_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_ensure_skips_existing_fields(self):
        client, moonraker = self._client(
            [{"key": "drying_temp_c"}, {"key": "drying_time_h"}])
        client.write_filament_drying(1, 70, 8)
        assert moonraker.calls == [
            self._GET,
            ("PATCH", "/v1/filament/1",
             {"extra": {"drying_temp_c": "70", "drying_time_h": "8"}}, True)]
        assert client._drying_fields_ensured is True
        assert moonraker.logger.messages == []


class TestSpoolmanClientWriteFilamentVariant:
    _ROUTES = {("GET", "/v1/field/filament"): [],
               ("POST", "/v1/field/filament/variant"): {"ok": 1},
               ("PATCH", "/v1/filament/5"): {"ok": 1}}
    _ENSURE = [("GET", "/v1/field/filament", None, False),
               ("POST", "/v1/field/filament/variant",
                {"name": "Variant", "field_type": "text"}, True)]

    def test_noop_when_empty(self):
        client, moonraker = rfid_spoolman_client(dict(self._ROUTES))
        assert client.write_filament_variant(5, "") is None
        assert moonraker.calls == []
        assert client._filament_fields_ensured is False
        assert moonraker.logger.messages == []

    def test_noop_when_already_current(self):
        client, moonraker = rfid_spoolman_client(dict(self._ROUTES))
        current = {"variant": '"Matte"'}
        assert client.write_filament_variant(5, "Matte", current_extra=current) is None
        assert moonraker.calls == self._ENSURE
        assert client._filament_fields_ensured is True
        assert moonraker.logger.messages == []

    def test_writes_variant_merged(self):
        client, moonraker = rfid_spoolman_client(dict(self._ROUTES))
        result = client.write_filament_variant(5, "Silk", current_extra={"other": "x"})
        assert result == {"ok": 1}
        assert client._filament_fields_ensured is True
        assert moonraker.calls == self._ENSURE + [
            ("PATCH", "/v1/filament/5", {"extra": {"other": "x", "variant": '"Silk"'}},
             True)]
        assert moonraker.logger.messages == []


class TestSpoolmanClientPatchSpool:
    def test_noop_when_nothing(self):
        client, moonraker = rfid_spoolman_client()
        assert client._patch_spool(5) is None
        assert moonraker.calls == []
        assert moonraker.logger.messages == []

    def test_lot_nr_only(self):
        client, moonraker = rfid_spoolman_client({("PATCH", "/v1/spool/5"): {"ok": 1}})
        assert client._patch_spool(5, lot_nr="2024-01") == {"ok": 1}
        assert moonraker.calls == [("PATCH", "/v1/spool/5", {"lot_nr": "2024-01"}, True)]
        assert moonraker.logger.messages == []

    def test_extra_updates_merges_existing(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/spool/5"): {"id": 5, "extra": {"keep": "1"}},
            ("PATCH", "/v1/spool/5"): {"ok": 1}})
        assert client._patch_spool(5, extra_updates={"new": "2"}) == {"ok": 1}
        assert moonraker.calls == [
            ("GET", "/v1/spool/5", None, True),
            ("PATCH", "/v1/spool/5", {"extra": {"keep": "1", "new": "2"}}, True)]
        assert moonraker.logger.messages == []

    def test_extra_updates_absent_spool(self):
        client, moonraker = rfid_spoolman_client({("PATCH", "/v1/spool/9"): {"ok": 1}})
        client._patch_spool(9, extra_updates={"new": "2"})
        assert moonraker.calls == [
            ("GET", "/v1/spool/9", None, True),
            ("PATCH", "/v1/spool/9", {"extra": {"new": "2"}}, True)]
        assert moonraker.logger.messages == []


class TestSpoolmanClientWriteSpoolMetadata:
    def test_noop_when_nothing(self):
        client, moonraker = rfid_spoolman_client()
        assert client.write_spool_metadata(5, lot_nr=None, uid=None) is None
        assert moonraker.calls == []
        assert moonraker.logger.messages == []

    def test_lot_nr_only_no_uid(self):
        client, moonraker = rfid_spoolman_client({("PATCH", "/v1/spool/5"): {"ok": 1}})
        assert client.write_spool_metadata(5, lot_nr="2024-05", uid="") == {"ok": 1}
        assert moonraker.calls == [("PATCH", "/v1/spool/5", {"lot_nr": "2024-05"}, True)]
        assert client._fields_ensured is False
        assert moonraker.logger.messages == []

    def test_uid_merged_into_card_uids(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/spool"): [],
            ("POST", "/v1/field/spool/card_uids"): {"ok": 1},
            ("GET", "/v1/spool/5"): {"id": 5, "extra": {"card_uids": '"AABB"'}},
            ("PATCH", "/v1/spool/5"): {"ok": 1}})
        assert client.write_spool_metadata(5, lot_nr="2024-05", uid="cc:dd") == {"ok": 1}
        assert moonraker.calls == [
            ("GET", "/v1/field/spool", None, False),
            ("POST", "/v1/field/spool/card_uids",
             {"name": "Card UIDs", "field_type": "text"}, True),
            ("GET", "/v1/spool/5", None, True),
            ("GET", "/v1/spool/5", None, True),
            ("PATCH", "/v1/spool/5",
             {"lot_nr": "2024-05", "extra": {"card_uids": '"AABB,CCDD"'}}, True)]
        assert client._fields_ensured is True
        assert moonraker.logger.messages == []


class TestSpoolmanClientGetSpool:
    def test_reads_through_the_spoolman_proxy(self):
        # Not through AFC_moonraker.get_spool: that one is a queued callback API
        # now and returns None to a synchronous caller.
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool/3"): {"id": 3}})
        assert client.get_spool("3") == {"id": 3}
        assert moonraker.calls == [("GET", "/v1/spool/3", None, True)]
        assert moonraker.fetches == []
        assert moonraker.logger.messages == []

    def test_a_non_dict_answer_is_no_spool(self):
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool/3"): "not a spool"})
        assert client.get_spool(3) is None
        assert moonraker.logger.messages == []


class TestSpoolmanClientReadFlowK:
    @staticmethod
    def _client(spool: Any) -> Tuple[SpoolmanClient, RfidSpoolmanMoonraker]:
        """
        :param spool: what Spoolman answers for spool 1
        :return tuple: (client, moonraker)
        """
        return rfid_spoolman_client({("GET", "/v1/spool/1"): spool})

    def test_none_when_no_spool(self):
        client, moonraker = self._client(None)
        assert client.read_flow_k(1) is None
        assert moonraker.calls == [("GET", "/v1/spool/1", None, True)]
        assert moonraker.logger.messages == []

    def test_reads_value(self):
        client, moonraker = self._client({"id": 1, "extra": {"flow_k": "1.234567"}})
        assert client.read_flow_k(1) == 1.234567
        assert moonraker.logger.messages == []

    def test_none_when_field_absent(self):
        client, moonraker = self._client({"id": 1, "extra": {}})
        assert client.read_flow_k(1) is None
        assert moonraker.logger.messages == []

    def test_none_when_empty_string(self):
        client, moonraker = self._client({"id": 1, "extra": {"flow_k": ""}})
        assert client.read_flow_k(1) is None
        assert moonraker.logger.messages == []

    def test_none_on_invalid_value(self):
        client, moonraker = self._client({"id": 1, "extra": {"flow_k": "not-json{"}})
        assert client.read_flow_k(1) is None
        assert moonraker.logger.messages == []


class TestSpoolmanClientWriteFlowK:
    def test_none_when_no_spool(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/spool"): [],
            ("POST", "/v1/field/spool/flow_k"): {"ok": 1}})
        assert client.write_flow_k(1, 1.5) is None
        assert moonraker.calls == [
            ("GET", "/v1/field/spool", None, False),
            ("POST", "/v1/field/spool/flow_k", {"name": "Flow K", "field_type": "float"},
             True),
            ("GET", "/v1/spool/1", None, True)]
        assert client._flow_k_field_ensured is True
        assert moonraker.logger.messages == []

    def test_writes_rounded_k_merged(self):
        client, moonraker = rfid_spoolman_client({
            ("GET", "/v1/field/spool"): [{"key": "flow_k"}],
            ("GET", "/v1/spool/1"): {"id": 1, "extra": {"keep": "x"}},
            ("PATCH", "/v1/spool/1"): {"ok": 1}})
        assert client.write_flow_k(1, 1.23456789) == {"ok": 1}
        assert moonraker.sent("PATCH", "/v1/spool/1") == [
            {"extra": {"keep": "x", "flow_k": "1.234568"}}]
        assert client._flow_k_field_ensured is True
        assert moonraker.logger.messages == []


class TestBedTempForMaterial:
    def test_an_unknown_material_claims_nothing_rather_than_guessing_pla(self):
        # Density has a defensible generic default; a bed temperature does not.
        assert bed_temp_for_material("PEEK") is None
        assert bed_temp_for_material("Snapmaker Mystery") is None
        assert bed_temp_for_material("") is None

    def test_separator_and_case_handling_matches_the_density_table(self):
        for spelling in ("PETG-CF", "petg cf", "petg_cf", "PETG/CF", "PetgCf"):
            assert bed_temp_for_material(spelling) == 70, spelling
        # Longest prefix, so a variant falls back to its base rather than missing.
        assert bed_temp_for_material("PLA Basic") == 55
        assert bed_temp_for_material("ABS Something") == 90
        assert bed_temp_for_material("PVA") == 45


class TestNormUid:
    def test_norm_uid_separator_and_case_insensitive(self):
        assert _norm_uid("E5:CA:F0:A1") == "E5CAF0A1"
        assert _norm_uid("e5-ca-f0-a1") == "E5CAF0A1"
        assert _norm_uid("e5 ca_f0 a1") == "E5CAF0A1"
        assert _norm_uid("") == ""
        assert _norm_uid(None) == ""

    def test_norm_uid_distinct_uids_differ(self):
        assert _norm_uid("56A36AEA") == "56A36AEA"
        assert _norm_uid("26a36aea") == "26A36AEA"


class TestDecodeExtra:
    def test_absent_returns_none(self):
        assert _decode_extra({}, "k") is None
        assert _decode_extra(None, "k") is None

    def test_empty_string_returns_none(self):
        assert _decode_extra({"k": ""}, "k") is None

    def test_valid_json_decoded(self):
        assert _decode_extra({"k": "[1, 2]"}, "k") == [1, 2]

    def test_non_json_returns_raw(self):
        assert _decode_extra({"k": "not json{"}, "k") == "not json{"


class TestSpoolUids:
    def test_no_card_uids_empty(self):
        assert _spool_uids({"extra": {}}) == set()
        assert _spool_uids({"extra": None}) == set()

    def test_blank_parts_skipped(self):
        spool = {"extra": {"card_uids": '"aa:bb,,  ,CCDD"'}}
        assert _spool_uids(spool) == {"AABB", "CCDD"}


class TestNormTrayUid:
    def test_normalizes_and_rejects(self):
        assert _norm_tray_uid("CF34CF1D") == "cf34cf1d"
        assert _norm_tray_uid(" cf34:cf1d-0000 ") == "cf34cf1d0000"
        # All-zero is an unread field, not a roll, and neither is junk.
        assert _norm_tray_uid("0" * 32) == ""
        assert _norm_tray_uid("nothex") == ""
        assert _norm_tray_uid(" : ") == ""
        assert _norm_tray_uid(None) == ""


class TestFindSpoolByTrayUid:
    _SPOOL_3 = {"id": 3, "extra": {"tray_uid": '"cf34cf1d"',
                                   "card_uids": '"D34E4E39,13F56D32"'}}
    _SPOOL_4 = {"id": 4, "extra": {"tray_uid": '"4e3177c3"'}}

    def test_finds_the_roll_whichever_face_was_read(self):
        # One record taught both faces of the reel, which is the fixed state.
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/spool"): [self._SPOOL_3, self._SPOOL_4]})
        assert find_spool_by_tray_uid(client, "CF34CF1D") == self._SPOOL_3
        assert find_spool_by_tray_uid(client, "4e3177c3") == self._SPOOL_4
        assert moonraker.logger.messages == []

    def test_an_ambiguous_answer_is_no_answer(self):
        # Picking one of two records for one roll would bind consumption to a
        # coin flip.
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool"): [
            {"id": 3, "extra": {"tray_uid": '"cf34cf1d"'}},
            {"id": 9, "extra": {"tray_uid": '"cf34cf1d"'}}]})
        assert find_spool_by_tray_uid(client, "cf34cf1d") is None
        assert moonraker.logger.messages == []

    def test_unknown_and_unusable_are_none(self):
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool"): [self._SPOOL_3]})
        assert find_spool_by_tray_uid(client, "deadbeef") is None
        assert find_spool_by_tray_uid(client, "") is None
        # The unusable id never reached Spoolman.
        assert moonraker.calls == [("GET", "/v1/spool", None, False)]
        assert moonraker.logger.messages == []

    def test_a_broken_listing_does_not_raise(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/spool"): RuntimeError("spoolman down")})
        assert find_spool_by_tray_uid(client, "cf34cf1d") is None
        assert moonraker.logger.messages == []


class TestMatchSpoolForTag:
    @staticmethod
    def _roll(sid: int, uids: List[str], tray: Optional[str] = None) -> Dict[str, Any]:
        """
        :param sid: spool id
        :param uids: chip UIDs in card_uids
        :param tray: roll id in tray_uid, if any
        :return dict: a Spoolman spool record, extra fields JSON-encoded
        """
        extra = {"card_uids": json.dumps(",".join(uids))}
        if tray:
            extra["tray_uid"] = json.dumps(tray)
        return {"id": sid, "extra": extra}

    @staticmethod
    def _client(spools: Any) -> Tuple[SpoolmanClient, RfidSpoolmanMoonraker]:
        """
        :param spools: Spoolman's spool listing
        :return tuple: (client, moonraker)
        """
        return rfid_spoolman_client({("GET", "/v1/spool"): spools})

    def test_match_prefers_the_roll_over_the_tag(self):
        # The reel's other face: a chip UID Spoolman has never seen, on a roll
        # it knows.
        roll = self._roll(132, ["D34E4E39"], "cf34cf1d")
        client, moonraker = self._client([roll, self._roll(163, ["7392020A"], "013d91a7")])
        assert match_spool_for_tag(client, "13f56d32", "CF34CF1D") == (roll, True, None)
        assert moonraker.calls == [("GET", "/v1/spool", None, False)]
        assert moonraker.logger.messages == []

    def test_match_falls_back_to_the_tag(self):
        roll = self._roll(7, ["AABBCCDD"])
        client, moonraker = self._client([roll])
        assert match_spool_for_tag(client, "aabbccdd", "") == (roll, False, None)
        assert moonraker.logger.messages == []

    def test_match_falls_back_when_the_roll_is_unknown(self):
        # The caller stamps the roll on so the other face matches next time.
        roll = self._roll(7, ["AABBCCDD"])
        client, moonraker = self._client([roll])
        assert match_spool_for_tag(client, "AABBCCDD", "deadbeef") == (roll, False, None)
        assert moonraker.logger.messages == []

    def test_match_refuses_an_ambiguous_roll(self):
        # Two records for one roll: the chip UID decides, and the record it
        # could not choose is named so the pair surfaces.
        first = self._roll(124, ["13F56D32"], "cf34cf1d")
        second = self._roll(132, ["D34E4E39"], "cf34cf1d")
        client, moonraker = self._client([first, second])
        assert match_spool_for_tag(client, "D34E4E39", "cf34cf1d") == (second, False, 124)
        assert moonraker.logger.messages == []

    def test_match_with_nothing_to_go_on_is_none(self):
        client, moonraker = self._client([self._roll(7, ["AABBCCDD"], "cf34cf1d")])
        assert match_spool_for_tag(client, "", "") == (None, False, None)
        assert moonraker.logger.messages == []

    def test_match_names_the_duplicate_record_it_walked_past(self):
        # The reel was recorded twice, once per face; the roll id is on one.
        right = self._roll(124, ["13F56D32", "D34E4E39"], "cf34cf1d")
        client, moonraker = self._client([right, self._roll(132, ["D34E4E39"])])
        assert match_spool_for_tag(client, "d34e4e39", "cf34cf1d") == (right, True, 132)
        assert moonraker.logger.messages == []

    def test_a_broken_listing_matches_nothing_and_says_nothing(self):
        client, moonraker = self._client(RuntimeError("spoolman unreachable"))
        assert match_spool_for_tag(client, "AABBCCDD", "cf34cf1d") == (None, False, None)
        assert moonraker.logger.messages == []


class TestCachedSpoolmanClient:
    class _AFC:
        """The AFC core object: its moonraker, and where the client is cached."""

        def __init__(self, moonraker: Any) -> None:
            """
            :param moonraker: AFC_moonraker, or None when it is not loaded
            """
            self.moonraker = moonraker

    def test_no_afc_gives_no_client(self):
        assert _cached_spoolman_client(None) is None

    def test_an_afc_without_moonraker_gives_no_client(self):
        afc = self._AFC(None)
        assert _cached_spoolman_client(afc) is None
        assert not hasattr(afc, "_afc_spoolman_client_cache")

    def test_one_client_is_built_and_cached_per_afc(self):
        moonraker = RfidSpoolmanMoonraker()
        afc = self._AFC(moonraker)
        client = _cached_spoolman_client(afc)
        assert isinstance(client, SpoolmanClient)
        assert client._mr is moonraker
        assert afc._afc_spoolman_client_cache is client
        assert _cached_spoolman_client(afc) is client


class TestFindSpoolByUid:
    @staticmethod
    def _spool(sid: int, uids: List[str]) -> Dict[str, Any]:
        """
        :param sid: spool id
        :param uids: chip UIDs in card_uids
        :return dict: a Spoolman spool record
        """
        return {"id": sid, "extra": {"card_uids": json.dumps(",".join(uids))}}

    def test_empty_uid_returns_none_without_search(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/spool"): [self._spool(1, ["AABB"])]})
        assert find_spool_by_uid(client, "") is None
        assert moonraker.calls == []
        assert moonraker.logger.messages == []

    def test_find_spool_by_uid_matches_regardless_of_case_or_separators(self):
        first = self._spool(1, ["AAAA1111"])
        second = self._spool(2, ["10C7E32F", "7BF0AFFF"])
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool"): [first, second]})
        assert find_spool_by_uid(client, "7bf0afff") == second
        assert find_spool_by_uid(client, "10:C7:E3:2F") == second
        assert find_spool_by_uid(client, "AAAA1111") == first
        assert moonraker.logger.messages == []

    def test_find_spool_by_uid_unknown_uid_returns_none(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/spool"): [self._spool(1, ["AAAA1111"])]})
        assert find_spool_by_uid(client, "DEADBEEF") is None
        assert moonraker.logger.messages == []

    def test_find_spool_by_uid_search_failure_returns_none(self):
        # A failed listing reads as "no match", so the caller leaves the tag's
        # values in place rather than creating a duplicate.
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/spool"): RuntimeError("spoolman unreachable")})
        assert find_spool_by_uid(client, "AAAA1111") is None
        assert moonraker.logger.messages == []


class TestDensityForMaterial:
    def test_density_known_materials(self):
        assert density_for_material("PLA") == 1.24
        assert density_for_material("PETG") == 1.27
        assert density_for_material("abs") == 1.04

    def test_density_separator_and_case_insensitive(self):
        for spelling in ("PLA-CF", "pla cf", "pla_cf", "PLA/CF"):
            assert density_for_material(spelling) == 1.30, spelling

    def test_density_prefix_fallback(self):
        # Longest matching base first: "PETG..." is petg, not pet.
        assert density_for_material("PETG Translucent") == 1.27
        assert density_for_material("ABS-CF Pro") == 1.13
        assert density_for_material("PLA Silk Rainbow") == 1.24

    def test_density_unknown_defaults_to_pla(self):
        assert density_for_material("unobtainium") == 1.24
        assert density_for_material("") == 1.24
        assert density_for_material(None) == 1.24

    def test_a_matte_still_reads_as_pla_until_one_is_measured(self):
        # A single-reel 1.32 was tried and withdrawn; pinned so the next change
        # is deliberate.
        assert density_for_material("PLA Matte") == 1.24
        assert density_for_material("PLA") == 1.24
        assert density_for_material("PLA Basic") == 1.24


class TestBuildFilamentName:
    def test_build_filament_name_full(self):
        assert build_filament_name("Bambu", "PLA", "Basic") == "Bambu PLA Basic"
        assert build_filament_name("Bambu", "PLA", "Matte") == "Bambu PLA Matte"

    def test_build_filament_name_drops_duplicate_material(self):
        # The sub_type already spells out the material, any case.
        assert build_filament_name("Bambu", "PLA", "PLA Basic") == "Bambu PLA Basic"
        assert build_filament_name("Bambu", "pla", "PLA Basic") == "Bambu PLA Basic"

    def test_build_filament_name_skips_empty_parts(self):
        assert build_filament_name("", "PLA", "") == "PLA"
        assert build_filament_name("Bambu", "", "") == "Bambu"
        assert build_filament_name("", "", "Silk") == "Silk"
        assert build_filament_name("", "", "") == ""


class TestMapTagToSlotInfo:
    _DEFAULTS = {"material": "", "color_hex": "", "multi_color": [], "is_dual_color": False,
                 "sku": "", "brand": "", "sub_type": "", "diameter": 1.75,
                 "extruder_temp": None, "bed_temp": None, "mfg_date": "", "uid": "",
                 "weight_g": None}

    def test_map_empty_tag_gives_defaults(self):
        # No optional rich key appears on an empty tag.
        assert map_tag_to_slot_info({}) == self._DEFAULTS

    def test_map_none_tag_gives_defaults(self):
        assert map_tag_to_slot_info(None) == self._DEFAULTS

    def test_map_primary_color_and_opaque_alpha(self):
        info = map_tag_to_slot_info({"filament": {"color_argb": 0xFF12CD56}})
        assert info == {**self._DEFAULTS, "color_hex": "12cd56", "multi_color": ["12cd56"]}

    def test_map_translucent_alpha_surfaced(self):
        info = map_tag_to_slot_info({"filament": {"color_argb": 0x8012CD56}})
        assert info == {**self._DEFAULTS, "color_hex": "12cd56", "multi_color": ["12cd56"],
                        "color_alpha": 128}

    def test_map_colors_argb_dedup_and_none_skip(self):
        info = map_tag_to_slot_info({"filament": {
            "color_argb": 0xFFAA0000,
            "colors_argb": [0xFFAA0000, None, 0xFF00BB00, 0xFFAA0000]}})
        assert info == {**self._DEFAULTS, "color_hex": "aa0000",
                        "multi_color": ["aa0000", "00bb00"], "is_dual_color": True}

    def test_map_temp_midpoint_and_range(self):
        info = map_tag_to_slot_info({"filament": {"hotend_min_c": 210, "hotend_max_c": 231}})
        assert info == {**self._DEFAULTS, "extruder_temp": 220,
                        "extruder_temp_min": 210, "extruder_temp_max": 231}

    def test_map_temp_max_only(self):
        info = map_tag_to_slot_info({"filament": {"hotend_max_c": 240}})
        assert info == {**self._DEFAULTS, "extruder_temp": 240, "extruder_temp_max": 240}

    def test_map_temp_min_only_gives_none_midpoint(self):
        info = map_tag_to_slot_info({"filament": {"hotend_min_c": 200}})
        assert info == {**self._DEFAULTS, "extruder_temp_min": 200}

    def test_map_optional_rich_fields_copied(self):
        fil = {"serial": "S123", "tray_uid": "aa" * 16, "density": 1.24,
               "drying_temp_c": 70, "drying_time_h": 8, "color_count": 2,
               "nozzle_diameter": 0.4, "spool_width_mm": 66.2, "length_m": 330}
        assert map_tag_to_slot_info({"filament": fil}) == {
            **self._DEFAULTS, "serial": "S123", "tray_uid": "aa" * 16, "density": 1.24,
            "drying_temp": 70, "drying_time_h": 8, "color_count": 2,
            "nozzle_diameter": 0.4, "spool_width_mm": 66.2, "length_m": 330}

    def test_map_optional_rich_fields_skip_unset_values(self):
        fil = {"serial": "", "density": None, "drying_temp_c": 0, "length_m": 0}
        assert map_tag_to_slot_info({"filament": fil}) == self._DEFAULTS

    def test_map_tag_type_copied_only_when_set(self):
        assert map_tag_to_slot_info({"uid": "aabb", "tag_type": "MifareClassic1k"}) == {
            **self._DEFAULTS, "uid": "aabb", "tag_type": "MifareClassic1k"}
        assert map_tag_to_slot_info({"uid": "aabb", "tag_type": ""}) == {
            **self._DEFAULTS, "uid": "aabb"}

    def test_map_brand_from_decode_only(self):
        assert map_tag_to_slot_info({"filament": {"manufacturer": "Elegoo"}}) == {
            **self._DEFAULTS, "brand": "Elegoo"}
        # Never guessed from the tag type: other brands use Classic tags too.
        info = map_tag_to_slot_info({"uid": "aa", "tag_type": "MifareClassic1k"})
        assert info["brand"] == ""


class TestMakeTagRecord:
    def test_record_drops_empty_values(self):
        rec = make_tag_record({"material": "PLA", "sku": "", "bed_temp": None,
                               "multi_color": []}, 100.0)
        assert rec == {"material": "PLA", "decoded": True, "scan_time": 100.0}

    def test_record_decoded_flag_and_time(self):
        assert make_tag_record({"material": "PLA"}, 1234.56789) == {
            "material": "PLA", "decoded": True, "scan_time": 1234.568}
        assert make_tag_record({"material": "PLA"}, 7, decoded=0) == {
            "material": "PLA", "decoded": False, "scan_time": 7.0}

    def test_record_failed_decode_uses_uid_and_type_args(self):
        rec = make_tag_record(None, 5.0, decoded=False, uid="AABB",
                              tag_type="MifareUltralight")
        assert rec == {"uid": "AABB", "tag_type": "MifareUltralight", "decoded": False,
                       "scan_time": 5.0}

    def test_record_slot_info_uid_wins_over_arg(self):
        rec = make_tag_record({"uid": "CCDD", "tag_type": "NTAG215"}, 5.0, uid="AABB",
                              tag_type="MifareClassic1k")
        assert rec == {"uid": "CCDD", "tag_type": "NTAG215", "decoded": True,
                       "scan_time": 5.0}


class TestFormatTagSummary:
    def test_format_tag_summary_full(self):
        summary = format_tag_summary({
            "brand": "Bambu", "material": "PLA", "sub_type": "Basic",
            "color_hex": "00ff00", "extruder_temp": 220, "bed_temp": 60,
        }, "ACE2 RFID: read lane1")
        assert summary == ("ACE2 RFID: read lane1\n"
                           "  Name: Bambu PLA Basic\n"
                           "  Brand: Bambu\n"
                           "  Material: PLA\n"
                           "  Color: #00ff00\n"
                           "  Nozzle temp: 220°C\n"
                           "  Bed temp: 60°C")

    def test_format_tag_summary_dual_color_joins_hex(self):
        summary = format_tag_summary(
            {"brand": "Bambu", "material": "PLA", "color_hex": "111111",
             "multi_color": ["e94b3c", "", "#ffffff"]}, "hdr")
        assert summary == ("hdr\n  Name: Bambu PLA\n  Brand: Bambu\n  Material: PLA\n"
                           "  Color: #e94b3c + #ffffff")

    def test_format_tag_summary_bare_uid_is_header_only(self):
        # A UID-only decode is just the header, so callers can skip it.
        assert format_tag_summary({"uid": "AABBCCDD"}, "hdr") == "hdr"

    def test_format_tag_summary_enriched_fields(self):
        # A matched Spoolman name wins; extras render when present.
        summary = format_tag_summary({
            "brand": "Bambu", "material": "PLA", "sub_type": "Basic",
            "display_name": "My Custom Name", "color_hex": "#00ff00",
            "diameter": 1.75, "remaining_weight": 812.6, "spool_id": 42,
        }, "hdr")
        assert summary == ("hdr\n"
                           "  Name: My Custom Name\n"
                           "  Brand: Bambu\n"
                           "  Material: PLA\n"
                           "  Color: #00ff00\n"
                           "  Diameter: 1.75mm\n"
                           "  Remaining: 813g\n"
                           "  Spoolman ID: 42")

    def test_all_new_lines(self):
        summary = format_tag_summary({
            "brand": "BQ Tech", "material": "PET", "color_hex": "12cd56",
            "diameter": 1.75, "density": 1.24,
            "extruder_temp": 220, "extruder_temp_min": 210,
            "extruder_temp_max": 230, "bed_temp": 60,
            "drying_temp": 70, "drying_time_h": 8,
            "weight_g": 750, "length_m": 330,
            "sku": "AC123", "serial": "S99", "mfg_date": "2024-08-12",
            "uid": "AABBCCDD",
        }, "hdr")
        assert summary.splitlines() == [
            "hdr",
            "  Name: BQ Tech PET",
            "  Brand: BQ Tech",
            "  Material: PET",
            "  Color: #12cd56",
            "  Diameter: 1.75mm",
            "  Density: 1.24g/cm³",
            "  Nozzle temp: 220°C (210–230)",
            "  Bed temp: 60°C",
            "  Drying: 70°C for 8h",
            "  Tag weight: 750g",
            "  Length: 330m",
            "  SKU: AC123",
            "  Serial: S99",
            "  Mfg date: 2024-08-12",
            "  Tag UID: AABBCCDD",
        ]

    def test_temp_line_without_range(self):
        expected = "hdr\n  Name: PLA\n  Material: PLA\n  Nozzle temp: 220°C"
        assert format_tag_summary({"material": "PLA", "extruder_temp": 220}, "hdr") == expected
        # One end of the range alone is not a range.
        assert format_tag_summary({"material": "PLA", "extruder_temp": 220,
                                   "extruder_temp_min": 210}, "hdr") == expected
        assert format_tag_summary({"material": "PLA", "extruder_temp": 220,
                                   "extruder_temp_max": 230}, "hdr") == expected

    def test_drying_time_only(self):
        summary = format_tag_summary({"material": "PLA", "drying_time_h": 6}, "hdr")
        assert summary == "hdr\n  Name: PLA\n  Material: PLA\n  Drying: ? for 6h"

    def test_drying_temp_only(self):
        summary = format_tag_summary({"material": "PLA", "drying_temp": 55}, "hdr")
        assert summary == "hdr\n  Name: PLA\n  Material: PLA\n  Drying: 55°C"

    def test_uid_shown_only_with_decoded_fields(self):
        with_fields = format_tag_summary({"material": "PLA", "uid": "AABB"}, "hdr")
        assert with_fields == "hdr\n  Name: PLA\n  Material: PLA\n  Tag UID: AABB"
        assert format_tag_summary({"uid": "AABB"}, "hdr") == "hdr"

    def test_new_lines_absent_when_unset(self):
        assert format_tag_summary({"material": "PLA"}, "hdr") == (
            "hdr\n  Name: PLA\n  Material: PLA")


class TestPromptHoldSpool:
    def test_prompt_hold_spool_emits_action_prompt(self):
        out: List[str] = []
        prompt_hold_spool(out.append, "lane1")
        assert out == [
            "// action:prompt_begin RFID Scan",
            "// action:prompt_text Tag detected on lane1, hold the spool at the reader "
            "until the read completes…",
            "// action:prompt_show",
        ]


class TestDismissPrompt:
    def test_dismiss_prompt_emits_prompt_end(self):
        out: List[str] = []
        dismiss_prompt(out.append)
        assert out == ["// action:prompt_end"]


class TestEnrichFromSpool:
    def test_client_none_returns_copy(self):
        slot = {"material": "PLA"}
        out = enrich_from_spool(None, 5, slot)
        assert out == {"material": "PLA"}
        assert out is not slot

    def test_get_spool_exception_returns_copy(self):
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool/5"): RuntimeError("down")})
        slot = {"material": "PLA"}
        out = enrich_from_spool(client, 5, slot)
        assert out == {"material": "PLA"}
        assert out is not slot
        assert moonraker.calls == [("GET", "/v1/spool/5", None, True)]
        assert moonraker.logger.messages == []

    def test_non_dict_spool_returns_copy(self):
        client, moonraker = rfid_spoolman_client()
        out = enrich_from_spool(client, 5, {"material": "PLA"})
        assert out == {"material": "PLA"}
        assert moonraker.calls == [("GET", "/v1/spool/5", None, True)]
        assert moonraker.logger.messages == []

    def test_only_present_fields_overlaid(self):
        client, moonraker = rfid_spoolman_client(
            {("GET", "/v1/spool/5"): {"id": 5, "filament": {"name": "N"}}})
        out = enrich_from_spool(client, 5, {"material": "PLA", "brand": "B"})
        # No vendor, material, temps or remaining weight on the spool record.
        assert out == {"material": "PLA", "brand": "B", "display_name": "N", "spool_id": 5}
        assert moonraker.logger.messages == []

    def test_vendor_without_filament_name(self):
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool/5"): {
            "id": 5, "filament": {"vendor": {"name": "V"}}, "remaining_weight": 0}})
        out = enrich_from_spool(client, 5, {"brand": "old"})
        assert out == {"brand": "V", "remaining_weight": 0, "spool_id": 5}
        assert moonraker.logger.messages == []

    def test_enrich_from_spool_overlays_record(self):
        slot = {"brand": "Bambu", "material": "PLA", "color_hex": "00ff00"}
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool/7"): {
            "id": 7, "remaining_weight": 640.0,
            "filament": {"name": "Bambu PLA Basic", "material": "PLA+",
                         "settings_extruder_temp": 220, "settings_bed_temp": 60,
                         "diameter": 1.75, "vendor": {"name": "Bambu Lab"},
                         "color_hex": "ffffff"}}})
        out = enrich_from_spool(client, 7, slot)
        # The record wins; the colour stays as the tag decoded it.
        assert out == {"brand": "Bambu Lab", "material": "PLA+", "color_hex": "00ff00",
                       "display_name": "Bambu PLA Basic", "extruder_temp": 220,
                       "bed_temp": 60, "diameter": 1.75, "remaining_weight": 640.0,
                       "spool_id": 7}
        assert slot == {"brand": "Bambu", "material": "PLA", "color_hex": "00ff00"}
        assert moonraker.logger.messages == []

    def test_enrich_from_spool_no_id_returns_copy(self):
        # Spool 0 is readable here, so only the no-id guard keeps it out.
        client, moonraker = rfid_spoolman_client({("GET", "/v1/spool/0"): {
            "id": 0, "remaining_weight": 640.0,
            "filament": {"name": "Bambu PLA Basic", "material": "PLA+",
                         "vendor": {"name": "Bambu Lab"}}}})
        slot = {"brand": "Bambu", "material": "PLA"}
        for spool_id in (0, None):
            out = enrich_from_spool(client, spool_id, slot)
            assert out == {"brand": "Bambu", "material": "PLA"}
            assert out is not slot
        assert moonraker.calls == []
        assert moonraker.logger.messages == []


class TestRgbArrayToHex:
    def test_rgb_array_to_hex(self):
        assert rgb_array_to_hex([255, 0, 0]) == "#ff0000"
        assert rgb_array_to_hex((0, 128, 255)) == "#0080ff"
        assert rgb_array_to_hex(["1", "2", "3", 99]) == "#010203"

    def test_rgb_array_to_hex_invalid_input(self):
        assert rgb_array_to_hex(None) == "#000000"
        assert rgb_array_to_hex([255]) == "#000000"
        assert rgb_array_to_hex("FF0000") == "#000000"


class TestLogNewFilament:
    def test_full_breakdown(self):
        logger = AceLogger()
        log_new_filament(logger, "U1 RFID", {"id": 7}, "Bambu", "PLA",
                         "00ff00", 1.75, 220, 60, "SKU9")
        assert logger.messages == [("info", "U1 RFID: created filament #7 in Spoolman:\n"
                                            "  vendor: Bambu\n"
                                            "  material: PLA\n"
                                            "  color: #00ff00\n"
                                            "  diameter: 1.75mm\n"
                                            "  nozzle temp: 220°C\n"
                                            "  bed temp: 60°C\n"
                                            "  SKU: SKU9")]

    def test_minimal_skips_optionals(self):
        logger = AceLogger()
        log_new_filament(logger, "U1 RFID", {}, "", "", "", 1.75, 0, 0)
        assert logger.messages == [("info", "U1 RFID: created filament #? in Spoolman:\n"
                                            "  diameter: 1.75mm")]


class TestLogNewSpool:
    def test_with_spool_weight(self):
        logger = AceLogger()
        log_new_spool(logger, "U1 RFID", {"id": 5}, 1000, spool_weight=250)
        assert logger.messages == [("info", "U1 RFID: created spool #5 in Spoolman:\n"
                                            "  filament weight: 1000g\n"
                                            "  spool weight (tare): 250g\n"
                                            "  remaining: 1000g")]

    def test_without_spool_weight(self):
        logger = AceLogger()
        log_new_spool(logger, "U1 RFID", {}, 900)
        assert logger.messages == [("info", "U1 RFID: created spool #? in Spoolman:\n"
                                            "  filament weight: 900g\n"
                                            "  remaining: 900g")]


class TestGetAutoSpoolmanCreate:
    def test_unit_opts_in(self):
        lane = rfid_lane()
        lane.unit_obj.auto_spoolman_create = True
        assert get_auto_spoolman_create(lane) is True

    def test_extruder_opts_in_when_unit_off(self):
        lane = rfid_lane()
        assert lane.unit_obj.auto_spoolman_create is False
        lane.extruder_obj.auto_spoolman_create = True
        assert get_auto_spoolman_create(lane) is True

    def test_both_present_but_off_returns_default(self):
        lane = rfid_lane()
        lane.extruder_obj.auto_spoolman_create = False
        assert get_auto_spoolman_create(lane, unit_default=True) is True
        assert get_auto_spoolman_create(lane, unit_default=False) is False

    def test_neither_present_returns_default(self):
        lane = rfid_lane()
        lane.unit_obj = None
        lane.extruder_obj = None
        assert get_auto_spoolman_create(lane, unit_default=True) is True
        assert get_auto_spoolman_create(lane) is False


class TestResolveRfidKeys:
    @staticmethod
    def _printer(values: Optional[Dict[str, str]]) -> AcePrinter:
        """
        :param values: [AFC_rfid_keys] options, None for no section
        :return AcePrinter: a printer with that section registered
        """
        printer = AcePrinter()
        if values is not None:
            keys = AFC_rfid_keys(AceConfig("AFC_rfid_keys", printer, values))
            printer.add_section("AFC_rfid_keys", keys, values)
        return printer

    def test_resolve_rfid_keys_no_shared_section_is_passthrough(self):
        printer = self._printer(None)
        assert resolve_rfid_keys(printer, b"\x01", None, None) == (b"\x01", None, None)

    def test_resolve_rfid_keys_fills_unset_from_shared(self):
        printer = self._printer({"bambu_master_key": "aa", "creality_key": "bb",
                                 "creality_encryption_key": "cc"})
        assert resolve_rfid_keys(printer, None, None, None) == (b"\xaa", b"\xbb", b"\xcc")

    def test_resolve_rfid_keys_own_key_wins(self):
        printer = self._printer({"bambu_master_key": "aa", "creality_key": "bb",
                                 "creality_encryption_key": "cc"})
        assert resolve_rfid_keys(printer, b"\x11", None, b"\x33") == (
            b"\x11", b"\xbb", b"\x33")


class TestDefaultBedTempForMaterial:
    def test_empty_and_none_return_none(self):
        assert default_bed_temp_for_material("") is None
        assert default_bed_temp_for_material(None) is None

    def test_all_symbols_return_none(self):
        assert default_bed_temp_for_material("!!!") is None

    def test_unknown_material_returns_none(self):
        assert default_bed_temp_for_material("xyz") is None

    def test_bed_temp_defaults(self):
        assert default_bed_temp_for_material("PLA") == 60
        assert default_bed_temp_for_material("ABS") == 95
        assert default_bed_temp_for_material("PLA-CF") == 60
        # Longest key first: "PETG HF" is petg (75), not pet (70).
        assert default_bed_temp_for_material("PETG HF") == 75


class TestApplyFilamentDefaults:
    # A fresh AFCLane's filament fields, as its __init__ leaves them.
    _FRESH = {"material": None, "color": "", "extruder_temp": None, "bed_temp": None,
              "weight": 0.0, "sub_type": "", "spool_vendor": "", "multi_color": [],
              "filament_density": 1.24}

    @staticmethod
    def _state(lane: AFCLane) -> Dict[str, Any]:
        """
        :param lane: the lane
        :return dict: every lane field apply_filament_defaults can set
        """
        return {"material": lane.material, "color": lane.color,
                "extruder_temp": lane.extruder_temp, "bed_temp": lane.bed_temp,
                "weight": lane.weight, "sub_type": lane.sub_type,
                "spool_vendor": lane.spool_vendor, "multi_color": lane.multi_color,
                "filament_density": lane.filament_density}

    def test_material_unknown_cleared(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "Unknown"})
        assert self._state(lane) == {**self._FRESH, "weight": 1000}

    def test_material_applied_when_unset(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA"})
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "bed_temp": 60.0,
                                     "weight": 1000}

    def test_material_kept_when_set(self):
        lane = rfid_lane()
        lane.material = "PETG"
        apply_filament_defaults(lane, {"material": "PLA"})
        # The bed default follows the lane's material, not the tag's.
        assert self._state(lane) == {**self._FRESH, "material": "PETG",
                                     "filament_density": 1.23, "bed_temp": 75.0,
                                     "weight": 1000}

    def test_color_hex_gets_hash_prefix(self):
        lane = rfid_lane()
        converter = Recorder(result="#bad000")
        apply_filament_defaults(lane, {"material": "PLA", "color_hex": "00ff00"},
                                color_converter=converter)
        assert converter.calls == []
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "color": "#00ff00",
                                     "bed_temp": 60.0, "weight": 1000}

    def test_color_hex_already_prefixed(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA", "color_hex": "#abcdef"})
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "color": "#abcdef",
                                     "bed_temp": 60.0, "weight": 1000}

    def test_color_from_converter_when_no_hex(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA", "color": [255, 0, 0]},
                                color_converter=rgb_array_to_hex)
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "color": "#ff0000",
                                     "bed_temp": 60.0, "weight": 1000}

    def test_color_converter_skipped_for_black(self):
        lane = rfid_lane()
        converter = Recorder(result="#bad000")
        apply_filament_defaults(lane, {"material": "PLA", "color": [0, 0, 0]},
                                color_converter=converter)
        apply_filament_defaults(lane, {"material": "PLA"}, color_converter=converter)
        assert converter.calls == []
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "bed_temp": 60.0,
                                     "weight": 1000}

    def test_extruder_temp_applied_as_float(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA", "extruder_temp": "220"})
        assert type(lane.extruder_temp) is float
        assert self._state(lane) == {**self._FRESH, "material": "PLA",
                                     "extruder_temp": 220.0, "bed_temp": 60.0,
                                     "weight": 1000}

    def test_extruder_temp_invalid_ignored(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA", "extruder_temp": "x"})
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "bed_temp": 60.0,
                                     "weight": 1000}

    def test_bed_temp_applied_as_float(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA", "bed_temp": "65"})
        assert type(lane.bed_temp) is float
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "bed_temp": 65.0,
                                     "weight": 1000}

    def test_bed_temp_invalid_then_material_default(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA", "bed_temp": "x"})
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "bed_temp": 60.0,
                                     "weight": 1000}

    def test_bed_default_when_tag_has_none(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "ABS"})
        assert self._state(lane) == {**self._FRESH, "material": "ABS",
                                     "filament_density": 1.04, "bed_temp": 95.0,
                                     "weight": 1000}

    def test_no_bed_default_for_unknown_material(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "xyz"})
        assert self._state(lane) == {**self._FRESH, "material": "xyz", "weight": 1000}

    def test_sub_type_stashed(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA", "sub_type": "Matte"})
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "bed_temp": 60.0,
                                     "sub_type": "Matte", "weight": 1000}

    def test_weight_defaulted_when_zero(self):
        lane = rfid_lane()
        assert lane.weight == 0
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.weight == 1000

    def test_weight_kept_when_set(self):
        lane = rfid_lane()
        lane.weight = 500
        apply_filament_defaults(lane, {"material": "PLA"})
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "bed_temp": 60.0,
                                     "weight": 500}

    def test_lane_temps_and_color_kept_when_tag_supplies_them(self):
        lane = rfid_lane()
        lane.color, lane.extruder_temp, lane.bed_temp = "#112233", 210.0, 55.0
        apply_filament_defaults(lane, {"material": "PLA", "color_hex": "00ff00",
                                       "extruder_temp": 220, "bed_temp": 65})
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "color": "#112233",
                                     "extruder_temp": 210.0, "bed_temp": 55.0,
                                     "weight": 1000}

    def test_afc_defaults_fill_material_and_color(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {}, afc_defaults={"default_material_type": "PLA",
                                                        "default_color": "#123456"})
        # The bed default ran before the AFC default material was filled in.
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "color": "#123456",
                                     "weight": 1000}

    def test_afc_defaults_not_used_when_tag_supplied(self):
        lane = rfid_lane()
        apply_filament_defaults(
            lane, {"material": "PETG", "color_hex": "abcdef"},
            afc_defaults={"default_material_type": "PLA", "default_color": "#123456"})
        assert self._state(lane) == {**self._FRESH, "material": "PETG", "color": "#abcdef",
                                     "filament_density": 1.23, "bed_temp": 75.0,
                                     "weight": 1000}

    def test_afc_defaults_present_but_empty_values(self):
        lane = rfid_lane()
        # None, not the fresh "", so writing the empty default would show.
        lane.color = None
        apply_filament_defaults(lane, {}, afc_defaults={"default_material_type": "",
                                                        "default_color": ""})
        assert self._state(lane) == {**self._FRESH, "color": None, "weight": 1000}

    def test_slot_info_none_only_defaults_weight(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, None)
        assert self._state(lane) == {**self._FRESH, "weight": 1000}

    def test_vendor_applied_when_unset(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"brand": "Elegoo"})
        assert self._state(lane) == {**self._FRESH, "spool_vendor": "Elegoo", "weight": 1000}

    def test_vendor_kept_when_already_set(self):
        lane = rfid_lane()
        lane.spool_vendor = "Existing"
        apply_filament_defaults(lane, {"brand": "Elegoo"})
        assert self._state(lane) == {**self._FRESH, "spool_vendor": "Existing",
                                     "weight": 1000}

    def test_vendor_not_applied_when_tag_has_none(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "PLA", "brand": ""})
        assert self._state(lane) == {**self._FRESH, "material": "PLA", "bed_temp": 60.0,
                                     "weight": 1000}

    def test_multi_color_applied_for_dual(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"multi_color": ["#aa0000", "", "00bb00"]})
        assert self._state(lane) == {**self._FRESH, "multi_color": ["aa0000", "00bb00"],
                                     "weight": 1000}

    def test_multi_color_not_applied_for_single(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"multi_color": ["aa0000"]})
        assert self._state(lane) == {**self._FRESH, "weight": 1000}

    def test_multi_color_kept_when_already_set(self):
        lane = rfid_lane()
        lane.multi_color = ["111111", "222222"]
        apply_filament_defaults(lane, {"multi_color": ["aa0000", "00bb00"]})
        assert self._state(lane) == {**self._FRESH, "multi_color": ["111111", "222222"],
                                     "weight": 1000}

    def test_density_applied_when_tag_carries_one(self):
        lane = rfid_lane()
        apply_filament_defaults(lane, {"material": "ABS", "density": 1.31})
        # The tag's density wins over the one the material setter picked.
        assert self._state(lane) == {**self._FRESH, "material": "ABS",
                                     "filament_density": 1.31, "bed_temp": 95.0,
                                     "weight": 1000}

    def test_density_untouched_without_tag_value(self):
        lane = rfid_lane()
        lane.filament_density = 1.5
        apply_filament_defaults(lane, {"material": "xyz"})
        assert self._state(lane) == {**self._FRESH, "material": "xyz",
                                     "filament_density": 1.5, "weight": 1000}

    def test_density_invalid_value_ignored(self):
        lane = rfid_lane()
        lane.filament_density = 1.5
        apply_filament_defaults(lane, {"density": "junk"})
        assert self._state(lane) == {**self._FRESH, "filament_density": 1.5,
                                     "weight": 1000}


class TestMissingFilamentFields:
    def test_all_empty_filament_filled(self):
        slot = {"material": " PLA ", "diameter": 1.75, "extruder_temp": 220,
                "bed_temp": 60, "sku": " S1 ", "multi_color": ["#00FF00"]}
        assert _missing_filament_fields({}, slot) == {
            "material": "PLA", "density": 1.24, "diameter": 1.75,
            "settings_extruder_temp": 220, "settings_bed_temp": 60,
            "article_number": "S1", "color_hex": "00ff00"}

    def test_nothing_when_all_present(self):
        fil = {"material": "PLA", "density": 1.24, "diameter": 1.75,
               "settings_extruder_temp": 220, "settings_bed_temp": 60,
               "article_number": "S1", "color_hex": "abcdef"}
        slot = {"material": "PETG", "diameter": 2.85, "extruder_temp": 250,
                "bed_temp": 90, "sku": "S2", "multi_color": ["112233"], "density": 1.3}
        assert _missing_filament_fields(fil, slot) == {}

    def test_tag_density_wins_over_table(self):
        # The table says 1.24 for PLA; the tag's own figure replaces it.
        assert _missing_filament_fields({}, {"material": "PLA", "density": 1.31}) == {
            "material": "PLA", "density": 1.31}

    def test_material_density_kept_when_filament_has_one(self):
        assert _missing_filament_fields({"density": 1.0}, {"material": "PLA"}) == {
            "material": "PLA"}

    def test_multi_color_two_hexes(self):
        out = _missing_filament_fields({}, {"multi_color": ["#AA0000", "00bb00"]})
        assert out == {"multi_color_hexes": "aa0000,00bb00",
                       "multi_color_direction": "coaxial"}

    def test_color_skipped_when_filament_has_multi(self):
        out = _missing_filament_fields({"multi_color_hexes": "112233,445566"},
                                       {"multi_color": ["aa0000"]})
        assert out == {}

    def test_color_skipped_when_filament_has_single(self):
        out = _missing_filament_fields({"color_hex": "abcdef"},
                                       {"multi_color": ["aa0000", "00bb00"]})
        assert out == {}

    def test_no_color_when_tag_has_none(self):
        assert _missing_filament_fields({}, {"material": "PLA", "multi_color": [""]}) == {
            "material": "PLA", "density": 1.24}


class TestSyncRfidToSpoolman:
    class _Lane:
        """The lane a sync assigns: AFCLane's name, spool_id and send_lane_data."""

        def __init__(self, name: str = "lane1", spool_id: Any = None) -> None:
            """
            :param name: lane name
            :param spool_id: the spool already on the lane
            """
            self.name = name
            self.spool_id = spool_id
            self.sends = 0

        def send_lane_data(self) -> None:
            """Count the pushes to moonraker's lane data."""
            self.sends += 1

    class _Spool:
        """afc.spool: stages the next spool, or assigns one as set_spoolID does."""

        def __init__(self, raises: Optional[BaseException] = None) -> None:
            """
            :param raises: exception set_spoolID raises, if any
            """
            self.next_spool_info: Any = None
            self.next_spool_id: Optional[int] = None
            self.assigned: List[Tuple[str, Any]] = []
            self._raises = raises

        def set_spoolID(self, cur_lane: Any, SpoolID: Any) -> None:
            """
            :param cur_lane: lane to assign
            :param SpoolID: Spoolman spool id
            """
            if self._raises is not None:
                raise self._raises
            self.assigned.append((cur_lane.name, SpoolID))
            cur_lane.spool_id = SpoolID

    class _AFC:
        """The AFC core object as the sync reaches it."""

        def __init__(self, moonraker: Any, spool: Any, spoolman: Any) -> None:
            """
            :param moonraker: AFC_moonraker
            :param spool: afc.spool
            :param spoolman: configured Spoolman URL, None when unset
            """
            self.moonraker = moonraker
            self.spool = spool
            self.spoolman = spoolman

    class _WriteQueue:
        """AFC_moonraker._write_queue: holds jobs until the writer thread runs."""

        def __init__(self) -> None:
            """Start empty."""
            self.items: List[Tuple[Callable[..., None], tuple]] = []

        def put_nowait(self, item: Tuple[Callable[..., None], tuple]) -> None:
            """
            :param item: (function, args)
            """
            self.items.append(item)

        def run(self) -> None:
            """Run the queued jobs, as the writer thread would."""
            items, self.items = self.items, []
            for func, args in items:
                func(*args)

    class _Reactor:
        """Klipper reactor: holds async callbacks until the reactor runs them."""

        def __init__(self) -> None:
            """Start empty."""
            self.callbacks: List[Callable[[float], None]] = []

        def register_async_callback(self, callback: Callable[[float], None]) -> None:
            """
            :param callback: called with the event time on the reactor
            """
            self.callbacks.append(callback)

        def run(self) -> None:
            """Run the pending callbacks."""
            callbacks, self.callbacks = self.callbacks, []
            for callback in callbacks:
                callback(0.0)

    class _KeywordLogger(AceLogger):
        """AceLogger that also records the keywords each call actually passed."""

        def __init__(self) -> None:
            """Start with no messages."""
            super().__init__()
            self.passed: List[Tuple[str, Dict[str, Any]]] = []

        def info(self, message: str, **kwargs: Any) -> None:
            """
            :param message: finished message
            :param kwargs: the keywords as passed
            """
            self.passed.append(("info", kwargs))
            super().info(message, **kwargs)

        def debug(self, message: str, **kwargs: Any) -> None:
            """
            :param message: finished message
            :param kwargs: the keywords as passed
            """
            self.passed.append(("debug", kwargs))
            super().debug(message, **kwargs)

    _INFO = {("GET", "/v1/info"): {"version": "0.22.1"}}
    _FIELDS = {("GET", "/v1/field/spool"): [{"key": "card_uids"}, {"key": "tray_uid"}],
               ("GET", "/v1/field/filament"): [{"key": "variant"}, {"key": "drying_temp_c"},
                                               {"key": "drying_time_h"}]}
    _FULL_FILAMENT = {"id": 88, "name": "Bambu PLA", "material": "PLA", "density": 1.24,
                      "diameter": 1.75, "settings_extruder_temp": 220,
                      "settings_bed_temp": 60, "color_hex": "00ff00", "article_number": "S1"}
    _FULL_TAG = {"uid": "AABB", "material": "PLA", "color_hex": "00ff00", "diameter": 1.75,
                 "extruder_temp": 220, "bed_temp": 60, "sku": "S1",
                 "multi_color": ["00ff00"], "mfg_date": "2024-01"}
    _CREATE_TAG = {"uid": "AABB", "brand": "", "material": "PLA", "color_hex": "00ff00",
                   "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60}
    _BASE_TAG = {"uid": "AABBCCDD", "brand": "BQ Tech", "material": "PLA",
                 "color_hex": "aa0000", "diameter": 1.75}
    # The created-filament log line for _CREATE_TAG.
    _CREATE_TAG_LOG = ("TEST: created filament #99 in Spoolman:\n"
                       "  material: PLA\n"
                       "  color: #00ff00\n"
                       "  diameter: 1.75mm\n"
                       "  nozzle temp: 220°C\n"
                       "  bed temp: 60°C")
    _SPOOL_1000_LOG = ("TEST: created spool #500 in Spoolman:\n"
                       "  filament weight: 1000g\n"
                       "  remaining: 1000g")

    @staticmethod
    def _matched(sid: int, uid: str, remaining: Any,
                 filament: Dict[str, Any]) -> Dict[str, Any]:
        """
        :param sid: spool id
        :param uid: the chip UID in its card_uids
        :param remaining: remaining_weight
        :param filament: the spool's filament record
        :return dict: a Spoolman spool record
        """
        return {"id": sid, "remaining_weight": remaining, "filament": dict(filament),
                "extra": {"card_uids": json.dumps(uid)}}

    def _routes(self, spools: List[Dict[str, Any]], **extra: Any
                ) -> Dict[Tuple[str, str], Any]:
        """
        Spoolman answering: reachable, listing spools, each readable and
        patchable, the extra fields already created.

        :param spools: the spool listing
        :param extra: more routes, keyed "METHOD path"
        :return dict: the routes
        """
        routes: Dict[Tuple[str, str], Any] = {**self._INFO, **self._FIELDS,
                                              ("GET", "/v1/spool"): spools}
        for spool in spools:
            routes[("GET", f"/v1/spool/{spool['id']}")] = spool
            routes[("PATCH", f"/v1/spool/{spool['id']}")] = {"id": spool["id"]}
        for key, value in extra.items():
            method, path = key.split(" ", 1)
            routes[(method, path)] = value
        return routes

    def _create_routes(self, filament: Any, patched: Any = None,
                       spool: Any = None, **extra: Any) -> Dict[Tuple[str, str], Any]:
        """
        Spoolman with no matching spool, creating filament #99 and spool #500.

        :param filament: the POST /v1/filament answer
        :param patched: the PATCH /v1/filament/99 answer
        :param spool: the POST /v1/spool answer, {"id": 500, ...} when None
        :param extra: more routes, keyed "METHOD path"
        :return dict: the routes
        """
        created = spool if spool is not None else {"id": 500, "remaining_weight": 1000}
        routes = self._routes([], **extra)
        routes.setdefault(("POST", "/v1/filament"), filament)
        routes.setdefault(("PATCH", "/v1/filament/99"), patched)
        routes.setdefault(("POST", "/v1/spool"), created)
        routes.setdefault(("GET", "/v1/spool/500"), {"id": 500, "extra": {}})
        routes.setdefault(("PATCH", "/v1/spool/500"), {"id": 500})
        return routes

    def _run(self, slot_info: Any, routes: Dict[Tuple[str, str], Any], *,
             allow_create: Optional[bool] = None, set_next: Optional[bool] = None,
             lane: Any = None, spool: Any = None, spoolman: Any = "http://spoolman:7912",
             reactor: Any = None, queue: Any = None, on_done: Any = None,
             logger: Optional[AceLogger] = None,
             prepare: Optional[Callable[[Any], None]] = None
             ) -> Tuple[Any, Any, AceLogger]:
        """
        Run sync_rfid_to_spoolman with one logger shared with moonraker, as AFC's is.

        allow_create and set_next are passed only when given, so a test that
        leaves them out runs on the sync's own defaults.

        :param prepare: called with the AFC object before the sync runs
        :return tuple: (afc, lane, logger)
        """
        logger = logger if logger is not None else AceLogger()
        moonraker = RfidSpoolmanMoonraker(routes, logger=logger)
        moonraker._write_queue = queue
        afc = self._AFC(moonraker, spool if spool is not None else self._Spool(), spoolman)
        lane = lane if lane is not None else self._Lane()
        flags: Dict[str, bool] = {}
        if allow_create is not None:
            flags["allow_create"] = allow_create
        if set_next is not None:
            flags["set_next"] = set_next
        if prepare is not None:
            prepare(afc)
        sync_rfid_to_spoolman(afc, lane, slot_info, logger, "TEST", reactor=reactor,
                              on_done=on_done, **flags)
        return afc, lane, logger

    def test_set_next_stashes_info_then_returns_no_spoolman(self):
        slot = {"uid": "AA", "material": "PLA"}
        afc, lane, logger = self._run(slot, self._routes([]), set_next=True, spoolman=None)
        assert afc.spool.next_spool_info == {"uid": "AA", "material": "PLA"}
        assert afc.spool.next_spool_info is not slot
        assert afc.moonraker.calls == []
        assert slot == {"uid": "AA", "material": "PLA"}
        assert logger.messages == []
        # No moonraker stops it the same way, before the bed default is written.
        other = self._AFC(None, self._Spool(), "http://spoolman:7912")
        other_logger = AceLogger()
        sync_rfid_to_spoolman(other, self._Lane(), slot, other_logger, "TEST", set_next=True)
        assert other.spool.next_spool_info == {"uid": "AA", "material": "PLA"}
        assert slot == {"uid": "AA", "material": "PLA"}
        assert other_logger.messages == []

    def test_set_next_dict_failure_swallowed(self):
        spool = self._Spool()
        spool.next_spool_info = "orig"
        afc, lane, logger = self._run(123, self._routes([]), set_next=True, spool=spool,
                                      spoolman=None)
        assert afc.spool.next_spool_info == "orig"       # dict(123) raised
        assert afc.moonraker.calls == []
        assert logger.messages == []

    def test_existing_spool_id_skips(self):
        slot = {"uid": "AA", "material": "PLA", "color_hex": "00ff00"}
        afc, lane, logger = self._run(slot, self._routes([]), lane=self._Lane(spool_id=42))
        assert lane.spool_id == 42
        assert afc.moonraker.calls == []
        # Stopped before the bed temp default is written back into the tag.
        assert slot == {"uid": "AA", "material": "PLA", "color_hex": "00ff00"}
        assert logger.messages == []

    def test_unreachable_logs_and_returns(self):
        slot = {"uid": "AA", "material": "PLA", "color_hex": "00ff00"}
        afc, lane, logger = self._run(slot, {})
        assert afc.moonraker.calls == [("GET", "/v1/info", None, False)]
        assert slot["bed_temp"] == 60
        assert lane.spool_id is None
        assert logger.messages == [
            ("info", "TEST: Spoolman unreachable, using the tag's own values on the lane "
                     "(no Spoolman match this scan)")]

    def test_bed_default_none_not_written(self):
        slot = {"uid": "AABB", "material": "xyz", "color_hex": "00ff00"}
        afc, lane, logger = self._run(slot, self._routes([]), allow_create=False)
        assert slot == {"uid": "AABB", "material": "xyz", "color_hex": "00ff00"}
        assert logger.messages == [
            ("info", "TEST: no Spoolman spool matches UID AABB and auto-create is OFF "
                     "(set 'auto_spoolman_create: True' to create one)")]

    def test_no_match_no_uid_on(self):
        slot = {"uid": "", "material": "PLA", "color_hex": "00ff00"}
        afc, lane, logger = self._run(slot, self._routes([]), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == []
        assert logger.messages == [
            ("info", "TEST: no Spoolman spool matches this tag (no UID) and auto-create "
                     "is ON")]

    def test_incomplete_missing_material_and_colour(self):
        slot = {"uid": "AABB", "material": "", "color_hex": ""}
        afc, lane, logger = self._run(slot, self._routes([]), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == []
        assert logger.messages == [
            ("info", "TEST: incomplete tag decode (missing material, colour), applied to "
                     "the lane, not creating a Spoolman entry")]

    def test_incomplete_missing_material_only(self):
        slot = {"uid": "AABB", "material": "", "color_hex": "00ff00"}
        afc, lane, logger = self._run(slot, self._routes([]), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == []
        assert lane.spool_id is None
        assert logger.messages == [
            ("info", "TEST: incomplete tag decode (missing material), applied to the lane, "
                     "not creating a Spoolman entry")]

    def test_matched_spool_no_backfill_assigns(self):
        matched = self._matched(300, "AABBCCDD", 800, self._FULL_FILAMENT)
        slot = {**self._FULL_TAG, "uid": "AABBCCDD"}
        afc, lane, logger = self._run(slot, self._routes([matched]))
        assert afc.moonraker.calls == [
            ("GET", "/v1/info", None, False),
            ("GET", "/v1/spool", None, False),
            ("GET", "/v1/field/spool", None, False),
            ("GET", "/v1/spool/300", None, True),
            ("GET", "/v1/spool/300", None, True),
            ("PATCH", "/v1/spool/300",
             {"lot_nr": "2024-01", "extra": {"card_uids": '"AABBCCDD"'}}, True)]
        assert afc.spool.assigned == [("lane1", 300)]
        assert lane.spool_id == 300
        assert lane.sends == 1
        # One client per AFC, and the set_spoolID signature check, both memoised.
        assert afc._afc_spoolman_client_cache._mr is afc.moonraker
        assert afc._afc_setspoolid_on_done is False
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABBCCDD"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 800g left) assigned to lane1")]

    def _roll_run(self, record_tray: Optional[str], tag_tray: Optional[str],
                  patch: Any = None) -> Tuple[Any, Any, AceLogger]:
        """
        Match spool #300 by its chip UID, with roll ids on the record and the tag.

        :param record_tray: tray_uid already on the spool record, if any
        :param tag_tray: the tag's tray_uid, if any
        :param patch: the PATCH /v1/spool/300 answer, {"id": 300} when None
        :return tuple: (afc, lane, logger)
        """
        matched = self._matched(300, "AABB", 100, self._FULL_FILAMENT)
        if record_tray is not None:
            matched["extra"]["tray_uid"] = json.dumps(record_tray)
        slot = dict(self._FULL_TAG)
        if tag_tray is not None:
            slot["tray_uid"] = tag_tray
        routes = self._routes([matched])
        if patch is not None:
            routes[("PATCH", "/v1/spool/300")] = patch
        return self._run(slot, routes)

    def test_matched_by_chip_keeps_the_roll_already_recorded(self):
        afc, lane, logger = self._roll_run("cf34cf1d", None)
        # Only the metadata stamp, carrying the recorded roll id through unchanged.
        assert afc.moonraker.sent("PATCH", "/v1/spool/300") == [
            {"lot_nr": "2024-01",
             "extra": {"card_uids": '"AABB"', "tray_uid": '"cf34cf1d"'}}]
        assert lane.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane1")]

    def test_matched_by_chip_stamps_the_roll_id(self):
        afc, lane, logger = self._roll_run(None, "CF34CF1D")
        # Stamped once while matching; the later metadata stamp leaves it settled.
        assert afc.moonraker.sent("PATCH", "/v1/spool/300") == [
            {"extra": {"card_uids": '"AABB"', "tray_uid": '"cf34cf1d"'}},
            {"lot_nr": "2024-01", "extra": {"card_uids": '"AABB"'}}]
        assert lane.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane1")]

    def test_matched_by_chip_leaves_a_conflicting_roll_alone(self):
        afc, lane, logger = self._roll_run("aaaa1111", "cf34cf1d")
        assert afc.moonraker.sent("PATCH", "/v1/spool/300") == [
            {"lot_nr": "2024-01",
             "extra": {"card_uids": '"AABB"', "tray_uid": '"aaaa1111"'}}]
        assert lane.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("warning", "TEST: spool #300 is recorded against roll aaaa1111 but this tag "
                        "says cf34cf1d, left alone; one of them is on the wrong spool"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane1")]

    def test_matched_by_chip_roll_id_write_failure_warns(self):
        def patch(body: Dict[str, Any]) -> Dict[str, Any]:
            """
            :param body: the PATCH body
            :return dict: the patched spool, unless the roll id is in the write
            """
            if "tray_uid" in body.get("extra", {}):
                raise RuntimeError("roll down")
            return {"id": 300}

        afc, lane, logger = self._roll_run(None, "cf34cf1d", patch=patch)
        assert afc.moonraker.sent("PATCH", "/v1/spool/300") == [
            {"extra": {"card_uids": '"AABB"', "tray_uid": '"cf34cf1d"'}},
            {"lot_nr": "2024-01", "extra": {"card_uids": '"AABB"'}}]
        assert lane.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("warning", "TEST: recording the roll id on spool #300 failed (roll down); the "
                        "tag on the other side of this spool will not match it yet"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane1")]

    def test_matched_spool_backfill_logs(self):
        matched = self._matched(301, "AABB", 250.4,
                                {"id": 88, "name": "Existing", "color_hex": "abcdef"})
        slot = {"uid": "AABB", "material": "PLA", "color_hex": "00ff00", "diameter": 1.75,
                "multi_color": ["00ff00"]}
        afc, lane, logger = self._run(slot, self._routes([matched]))
        # The bed temp default written into the tag is backfilled too.
        assert afc.moonraker.sent("PATCH", "/v1/filament/88") == [
            {"material": "PLA", "density": 1.24, "diameter": 1.75, "settings_bed_temp": 60}]
        assert lane.spool_id == 301
        # The PATCH failed, so only the client's error is logged, no backfill.
        assert logger.messages == [
            ("info", "TEST: matched spool #301 by tag UID AABB"),
            ("error", 'Spoolman PATCH /v1/filament/88 failed; request body: '
                      '{"material": "PLA", "density": 1.24, "diameter": 1.75, '
                      '"settings_bed_temp": 60}'),
            ("info", "TEST: spool #301 ('Existing', #abcdef, 250g left) assigned to lane1")]

    def test_matched_backfill_exception_debug(self):
        matched = self._matched(300, "AABB", 100,
                                {"id": 88, "name": "N", "color_hex": "00ff00"})
        slot = {"uid": "AABB", "material": "PLA", "color_hex": "00ff00", "diameter": 1.75,
                "multi_color": ["00ff00"]}
        routes = self._routes([matched], **{"PATCH /v1/filament/88": RuntimeError("bf fail")})
        afc, lane, logger = self._run(slot, routes)
        assert lane.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("debug", "TEST: filament backfill skipped: bf fail"),
            ("info", "TEST: spool #300 ('N', #00ff00, 100g left) assigned to lane1")]

    def test_metadata_stamp_failure_warns(self):
        matched = self._matched(300, "AABB", 100, self._FULL_FILAMENT)
        routes = self._routes([matched], **{"PATCH /v1/spool/300": RuntimeError("stamp fail")})
        afc, lane, logger = self._run(dict(self._FULL_TAG), routes)
        assert lane.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("warning", "TEST: stamping UID/lot on new spool #300 failed (stamp fail), next "
                        "scan of this tag may not re-match it"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane1")]

    def test_no_match_no_create_off(self):
        slot = {"uid": "AABB", "material": "PLA", "color_hex": "00ff00"}
        # No allow_create passed: the sync's default is not to create.
        afc, lane, logger = self._run(slot, self._routes([]))
        assert afc.moonraker.sent("POST", "/v1/filament") == []
        assert lane.spool_id is None
        assert logger.messages == [
            ("info", "TEST: no Spoolman spool matches UID AABB and auto-create is OFF "
                     "(set 'auto_spoolman_create: True' to create one)")]

    def test_create_filament_failure_warns(self):
        slot = {"uid": "AABB", "brand": "Bambu", "material": "PLA", "color_hex": "00ff00",
                "sub_type": "Basic"}
        routes = self._routes([], **{"GET /v1/vendor?name=Bambu": [{"id": 7, "name": "Bambu"}]})
        afc, lane, logger = self._run(slot, routes, allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/spool") == []
        assert lane.spool_id is None
        assert logger.messages == [
            ("error", 'Spoolman POST /v1/filament failed; request body: '
                      '{"name": "Bambu PLA Basic", "vendor_id": 7, "material": "PLA", '
                      '"density": 1.24, "diameter": 1.75, "color_hex": "00ff00", '
                      '"settings_bed_temp": 60, "weight": 1000}'),
            ("warning", "TEST: Spoolman create_filament FAILED for 'Bambu PLA Basic', "
                        "check Spoolman/moonraker")]

    def test_created_filament_no_id_warns(self):
        routes = self._create_routes({"name": "X"})
        afc, lane, logger = self._run(dict(self._CREATE_TAG), routes, allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/spool") == []
        assert logger.messages == [
            ("info", self._CREATE_TAG_LOG.replace("#99", "#?")),
            ("warning", "TEST: resolved filament has no id, aborting")]

    def test_create_spool_failure_warns(self):
        # The backfill PATCH fails as well, so no backfill is claimed; the run
        # stops at the spool create.
        routes = self._create_routes({"id": 99, "name": "PLA", "color_hex": "00ff00"})
        routes[("POST", "/v1/spool")] = None
        afc, lane, logger = self._run(dict(self._CREATE_TAG), routes, allow_create=True)
        assert lane.spool_id is None
        assert logger.messages == [
            ("info", self._CREATE_TAG_LOG),
            ("error", 'Spoolman PATCH /v1/filament/99 failed; request body: '
                      '{"material": "PLA", "density": 1.24, "diameter": 1.75, '
                      '"settings_extruder_temp": 220, "settings_bed_temp": 60}'),
            ("error", 'Spoolman POST /v1/spool failed; request body: '
                      '{"filament_id": 99, "initial_weight": 1000, "remaining_weight": 1000}'),
            ("warning", "TEST: Spoolman create_spool FAILED for filament #99, check "
                        "Spoolman/moonraker")]

    def test_create_full_path_logs(self):
        slot = {"uid": "AABBCCDD", "brand": "Bambu", "material": "PLA", "sub_type": "Basic",
                "color_hex": "00ff00", "diameter": 1.75, "extruder_temp": 220,
                "bed_temp": 60, "sku": "SKU9", "multi_color": ["00ff00"],
                "mfg_date": "2024-01", "drying_temp": 70, "drying_time_h": 8,
                "weight_g": 750}
        # The backfill PATCH fails, so no backfill is claimed and the created
        # filament is the one kept: its extra is merged into the later writes
        # and its name ends the summary.
        routes = self._create_routes(
            {"id": 99, "name": "Bambu PLA Basic", "color_hex": "00ff00",
             "extra": {"note": '"x"'}},
            patched=lambda body: None if "extra" not in body else {"id": 99, **body},
            spool={"id": 500, "remaining_weight": 750.0},
            **{"GET /v1/vendor?name=Bambu": [{"id": 7, "name": "Bambu"}]})
        afc, lane, logger = self._run(slot, routes, allow_create=True)
        assert afc.moonraker.calls == [
            ("GET", "/v1/info", None, False),
            ("GET", "/v1/spool", None, False),
            ("GET", "/v1/vendor?name=Bambu", None, False),
            ("POST", "/v1/filament",
             {"name": "Bambu PLA Basic", "vendor_id": 7, "material": "PLA", "density": 1.24,
              "diameter": 1.75, "color_hex": "00ff00", "settings_extruder_temp": 220,
              "settings_bed_temp": 60, "weight": 750.0, "article_number": "SKU9"}, True),
            ("PATCH", "/v1/filament/99",
             {"material": "PLA", "density": 1.24, "diameter": 1.75,
              "settings_extruder_temp": 220, "settings_bed_temp": 60,
              "article_number": "SKU9"}, True),
            ("GET", "/v1/field/filament", None, False),
            ("PATCH", "/v1/filament/99",
             {"extra": {"note": '"x"', "variant": '"Basic"'}}, True),
            ("GET", "/v1/field/filament", None, False),
            ("PATCH", "/v1/filament/99",
             {"extra": {"note": '"x"', "drying_temp_c": "70", "drying_time_h": "8"}},
             True),
            ("POST", "/v1/spool",
             {"filament_id": 99, "initial_weight": 750.0, "remaining_weight": 750.0}, True),
            ("GET", "/v1/field/spool", None, False),
            ("GET", "/v1/spool/500", None, True),
            ("GET", "/v1/spool/500", None, True),
            ("PATCH", "/v1/spool/500",
             {"lot_nr": "2024-01", "extra": {"card_uids": '"AABBCCDD"'}}, True)]
        assert lane.spool_id == 500
        assert logger.messages == [
            ("info", "TEST: created filament #99 in Spoolman:\n"
                     "  vendor: Bambu\n"
                     "  material: PLA\n"
                     "  color: #00ff00\n"
                     "  diameter: 1.75mm\n"
                     "  nozzle temp: 220°C\n"
                     "  bed temp: 60°C\n"
                     "  SKU: SKU9"),
            ("error", 'Spoolman PATCH /v1/filament/99 failed; request body: '
                      '{"material": "PLA", "density": 1.24, "diameter": 1.75, '
                      '"settings_extruder_temp": 220, "settings_bed_temp": 60, '
                      '"article_number": "SKU9"}'),
            ("info", "TEST: created spool #500 in Spoolman:\n"
                     "  filament weight: 750.0g\n"
                     "  remaining: 750.0g"),
            ("info", "TEST: spool #500 ('Bambu PLA Basic', #00ff00, 750g left) assigned to "
                     "lane1")]

    def test_create_variant_and_drying_exceptions_debug(self):
        slot = {**self._CREATE_TAG, "sub_type": "Basic", "drying_temp": 70,
                "drying_time_h": 8}
        routes = self._create_routes(
            {"id": 99, "name": "PLA", "color_hex": "00ff00"},
            patched={"id": 99, "name": "PLA", "color_hex": "00ff00"},
            **{"GET /v1/field/filament": RuntimeError("x")})
        afc, lane, logger = self._run(slot, routes, allow_create=True)
        assert lane.spool_id == 500
        assert logger.messages == [
            ("info", self._CREATE_TAG_LOG),
            ("info", "TEST: backfilled density, diameter, material, settings_bed_temp, "
                     "settings_extruder_temp on filament #99"),
            ("debug", "TEST: filament variant write skipped: x"),
            ("debug", "TEST: filament drying write skipped: x"),
            ("info", self._SPOOL_1000_LOG),
            ("info", "TEST: spool #500 ('PLA', #00ff00, 1000g left) assigned to lane1")]

    def test_set_next_stages_spool_id(self):
        slot = dict(self._CREATE_TAG)
        routes = self._create_routes({"id": 99, "name": "PLA", "color_hex": "00ff00"},
                                     patched={"id": 99, "name": "PLA", "color_hex": "00ff00"})
        afc, lane, logger = self._run(slot, routes, set_next=True,
                                      lane=self._Lane(spool_id=42), allow_create=True)
        assert afc.spool.next_spool_info == self._CREATE_TAG
        assert afc.spool.next_spool_id == 500
        assert afc.spool.assigned == []
        assert lane.spool_id == 42
        assert lane.sends == 0
        assert logger.messages == [
            ("info", self._CREATE_TAG_LOG),
            ("info", "TEST: backfilled density, diameter, material, settings_bed_temp, "
                     "settings_extruder_temp on filament #99"),
            ("info", self._SPOOL_1000_LOG),
            ("info", "TEST: spool #500 ('PLA', #00ff00, 1000g left) staged as next_spool_id")]

    def test_matched_backfill_updated_dict_replaces_filament(self):
        matched = self._matched(300, "AABB", 100, {"id": 88})
        slot = {"uid": "AABB", "material": "PLA", "color_hex": "00ff00", "diameter": 1.75,
                "multi_color": ["00ff00"]}
        routes = self._routes([matched], **{
            "PATCH /v1/filament/88": {"id": 88, "name": "Updated", "color_hex": "112233"}})
        afc, lane, logger = self._run(slot, routes)
        # The PATCH answer replaces the filament, so the summary uses it.
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("info", "TEST: backfilled color_hex, density, diameter, material, "
                     "settings_bed_temp on filament #88"),
            ("info", "TEST: spool #300 ('Updated', #112233, 100g left) assigned to lane1")]

    def test_create_backfill_updated_dict_replaces_filament(self):
        routes = self._create_routes({"id": 99, "name": "Init", "color_hex": "00ff00"},
                                     patched={"id": 99, "name": "Patched",
                                              "color_hex": "#00ff00 "})
        afc, lane, logger = self._run(dict(self._CREATE_TAG), routes, allow_create=True)
        assert logger.messages == [
            ("info", self._CREATE_TAG_LOG),
            ("info", "TEST: backfilled density, diameter, material, settings_bed_temp, "
                     "settings_extruder_temp on filament #99"),
            ("info", self._SPOOL_1000_LOG),
            ("info", "TEST: spool #500 ('Patched', #00ff00, 1000g left) assigned to lane1")]

    def test_create_backfill_exception_debug(self):
        routes = self._create_routes({"id": 99, "name": "PLA", "color_hex": "00ff00"},
                                     patched=RuntimeError("bf"))
        afc, lane, logger = self._run(dict(self._CREATE_TAG), routes, allow_create=True)
        assert lane.spool_id == 500
        assert logger.messages == [
            ("info", self._CREATE_TAG_LOG),
            ("debug", "TEST: filament backfill skipped: bf"),
            ("info", self._SPOOL_1000_LOG),
            ("info", "TEST: spool #500 ('PLA', #00ff00, 1000g left) assigned to lane1")]

    def test_create_no_backfill_when_complete(self):
        routes = self._create_routes({**self._FULL_FILAMENT, "id": 99, "name": "Full"})
        slot = {**self._CREATE_TAG, "sku": "S1", "multi_color": ["00ff00"]}
        afc, lane, logger = self._run(slot, routes, allow_create=True)
        assert afc.moonraker.sent("PATCH", "/v1/filament/99") == []
        assert logger.messages == [
            ("info", self._CREATE_TAG_LOG + "\n  SKU: S1"),
            ("info", self._SPOOL_1000_LOG),
            ("info", "TEST: spool #500 ('Full', #00ff00, 1000g left) assigned to lane1")]

    def test_create_vendor_lookup_none_leaves_vendor_id_unset(self):
        slot = {**self._CREATE_TAG, "brand": "Bambu"}
        routes = self._create_routes(
            {"id": 99, "name": "Bambu PLA", "color_hex": "00ff00"},
            patched={"id": 99, "name": "Bambu PLA", "color_hex": "00ff00"},
            **{"GET /v1/vendor?name=Bambu": []})
        afc, lane, logger = self._run(slot, routes, allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == [
            {"name": "Bambu PLA", "material": "PLA", "density": 1.24, "diameter": 1.75,
             "color_hex": "00ff00", "settings_extruder_temp": 220, "settings_bed_temp": 60,
             "weight": 1000}]
        assert lane.spool_id == 500
        assert logger.messages == [
            ("error", 'Spoolman POST /v1/vendor failed; request body: {"name": "Bambu"}'),
            ("info", "TEST: created filament #99 in Spoolman:\n"
                     "  vendor: Bambu\n"
                     "  material: PLA\n"
                     "  color: #00ff00\n"
                     "  diameter: 1.75mm\n"
                     "  nozzle temp: 220°C\n"
                     "  bed temp: 60°C"),
            ("info", "TEST: backfilled density, diameter, material, settings_bed_temp, "
                     "settings_extruder_temp on filament #99"),
            ("info", self._SPOOL_1000_LOG),
            ("info", "TEST: spool #500 ('Bambu PLA', #00ff00, 1000g left) assigned to lane1")]

    def test_outer_exception_logs_error(self):
        matched = self._matched(300, "AABB", 100, self._FULL_FILAMENT)
        afc, lane, logger = self._run(
            {**self._FULL_TAG, "mfg_date": ""}, self._routes([matched]),
            lane=self._Lane("lane9"), spool=self._Spool(raises=RuntimeError("kaboom")))
        assert lane.spool_id is None
        assert lane.sends == 0
        # "apply", not "sync": the Spoolman round trips worked, the lane did not.
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("error", "TEST Spoolman apply failed for lane9: kaboom")]

    def test_a_reel_recorded_twice_is_named_where_it_is_actionable(self):
        # A spool with a tag on each side, one record per chip UID: the roll
        # match picks the right record and the bind names the other one.
        right = {"id": 124, "remaining_weight": 800,
                 "filament": {"id": 88, "name": "Bambu PLA Matte", "material": "PLA",
                              "density": 1.24, "diameter": 1.75, "color_hex": "757575",
                              "settings_bed_temp": 55},
                 "extra": {"tray_uid": '"cf34cf1d"', "card_uids": '"D34E4E39"'}}
        duplicate = {"id": 132, "remaining_weight": 200, "filament": {"id": 88},
                     "extra": {"card_uids": '"13F56D32"'}}
        slot = {"uid": "13F56D32", "tray_uid": "cf34cf1d", "material": "PLA",
                "color_hex": "757575", "diameter": 1.75}
        afc, lane, logger = self._run(slot, self._routes([right, duplicate]),
                                      lane=self._Lane("lane14"))
        assert lane.spool_id == 124
        # The tag's chip UID is added to the record the roll matched.
        assert afc.moonraker.sent("PATCH", "/v1/spool/124") == [
            {"extra": {"tray_uid": '"cf34cf1d"', "card_uids": '"13F56D32,D34E4E39"'}}]
        assert logger.messages == [
            ("warning", "TEST: spool #132 is the SAME PHYSICAL REEL as #124: a spool with a "
                        "tag on each side, recorded twice. Merge them in Spoolman (keep one, "
                        "add the other's card_uids to it, add the two used weights together) "
                        "or its filament count stays split between them."),
            ("info", "TEST: matched spool #124 by tray UID cf34cf1d"),
            ("info", "TEST: spool #124 ('Bambu PLA Matte', #757575, 800g left) assigned to "
                     "lane14")]

    def test_sync_refuses_incomplete_decode(self):
        slot = {"uid": "AABBCCDD", "brand": "Bambu", "material": "PLA", "color_hex": "",
                "sub_type": "Basic"}
        afc, lane, logger = self._run(slot, self._routes([]), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == []
        assert afc.moonraker.sent("POST", "/v1/spool") == []
        assert logger.messages == [
            ("info", "TEST: incomplete tag decode (missing colour), applied to the lane, "
                     "not creating a Spoolman entry")]

    def test_sync_creates_new_filament_for_new_uid(self):
        # An unseen UID always creates a new filament and spool.
        slot = {"uid": "AABBCCDD", "brand": "Bambu", "material": "PLA",
                "color_hex": "ffffff", "sub_type": "Basic"}
        routes = self._create_routes(
            lambda body: {"id": 99, **body}, patched={"id": 99},
            **{"GET /v1/vendor?name=Bambu": [{"id": 7, "name": "Bambu"}]})
        afc, lane, logger = self._run(slot, routes, allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == [
            {"name": "Bambu PLA Basic", "vendor_id": 7, "material": "PLA", "density": 1.24,
             "diameter": 1.75, "color_hex": "ffffff", "settings_bed_temp": 60,
             "weight": 1000}]
        assert afc.moonraker.sent("POST", "/v1/spool") == [
            {"filament_id": 99, "initial_weight": 1000, "remaining_weight": 1000}]
        assert lane.spool_id == 500
        assert logger.messages == [
            ("info", "TEST: created filament #99 in Spoolman:\n"
                     "  vendor: Bambu\n"
                     "  material: PLA\n"
                     "  color: #ffffff\n"
                     "  diameter: 1.75mm\n"
                     "  bed temp: 60°C"),
            ("info", self._SPOOL_1000_LOG),
            ("info", "TEST: spool #500 ('Bambu PLA Basic', #ffffff, 1000g left) assigned to "
                     "lane1")]

    def test_sync_no_create_without_uid(self):
        # No tag UID means nothing to re-match on, so nothing is created.
        slot = {"uid": "", "brand": "Bambu", "material": "PLA", "color_hex": "ffffff",
                "sub_type": "Basic"}
        afc, lane, logger = self._run(slot, self._routes([]), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == []
        assert afc.moonraker.sent("POST", "/v1/spool") == []
        assert logger.messages == [
            ("info", "TEST: no Spoolman spool matches this tag (no UID) and auto-create "
                     "is ON")]

    def test_with_a_reactor_the_http_runs_on_the_moonraker_thread(self):
        matched = self._matched(300, "AABB", 100, self._FULL_FILAMENT)
        queue, reactor = self._WriteQueue(), self._Reactor()
        afc, lane, logger = self._run(dict(self._FULL_TAG), self._routes([matched]),
                                      reactor=reactor, queue=queue)
        # Nothing reaches Spoolman during the call itself.
        assert afc.moonraker.calls == []
        assert len(queue.items) == 1
        queue.run()
        assert afc.moonraker.calls == [
            ("GET", "/v1/info", None, False),
            ("GET", "/v1/spool", None, False),
            ("GET", "/v1/field/spool", None, False),
            ("GET", "/v1/spool/300", None, True),
            ("GET", "/v1/spool/300", None, True),
            ("PATCH", "/v1/spool/300",
             {"lot_nr": "2024-01", "extra": {"card_uids": '"AABB"'}}, True)]
        # Neither the lane nor the console is touched off the reactor.
        assert lane.spool_id is None
        assert logger.messages == []
        assert len(reactor.callbacks) == 1
        reactor.run()
        # Run on the sync's defaults: assigned, not staged.
        assert lane.spool_id == 300
        assert afc.spool.next_spool_id is None
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane1")]

    def test_without_a_reactor_it_stays_inline(self):
        matched = self._matched(300, "AABB", 100, self._FULL_FILAMENT)
        queue = self._WriteQueue()
        afc, lane, logger = self._run(dict(self._FULL_TAG), self._routes([matched]),
                                      queue=queue)
        assert queue.items == []
        # Run on the sync's defaults: assigned, not staged.
        assert lane.spool_id == 300
        assert afc.spool.next_spool_id is None
        # A reactor without moonraker's writer queue stays inline too.
        other = self._Lane("lane2")
        reactor = self._Reactor()
        afc, other, other_logger = self._run(dict(self._FULL_TAG), self._routes([matched]),
                                             lane=other, reactor=reactor)
        assert reactor.callbacks == []
        assert other.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane1")]
        assert other_logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane2")]

    def test_on_done_fires_on_every_path(self):
        # Callers hang their next step off it, so it cannot be skipped.
        seen: List[Tuple[str, Any]] = []
        lane = self._Lane()
        _, _, early_logger = self._run({}, self._routes([]), spoolman=None, lane=lane,
                                       on_done=lambda: seen.append(("early", lane.spool_id)))
        assert seen == [("early", None)]
        assert early_logger.messages == []
        matched = self._matched(300, "AABB", 100, self._FULL_FILAMENT)
        afc, lane, logger = self._run(dict(self._FULL_TAG), self._routes([matched]),
                                      lane=lane,
                                      on_done=lambda: seen.append(("done", lane.spool_id)))
        # It fires once the lane carries the spool.
        assert seen == [("early", None), ("done", 300)]
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 100g left) assigned to lane1")]

    def test_logging_from_the_thread_is_deferred_to_the_reactor(self):
        # AFC's logger answers the g-code console, which must not happen off
        # the reactor: the worker records and the reactor replays.
        matched = self._matched(300, "AABB", 100,
                                {"id": 88, "name": "N", "color_hex": "00ff00"})
        slot = {"uid": "AABB", "material": "PLA", "color_hex": "00ff00", "diameter": 1.75,
                "multi_color": ["00ff00"]}
        routes = self._routes([matched], **{"PATCH /v1/filament/88": RuntimeError("bf fail")})
        queue, reactor = self._WriteQueue(), self._Reactor()
        logger = self._KeywordLogger()
        afc, lane, _ = self._run(slot, routes, reactor=reactor, queue=queue, logger=logger)
        queue.run()
        assert logger.messages == []
        reactor.run()
        # The thread's lines replay first, then the apply step's.
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("debug", "TEST: filament backfill skipped: bf fail"),
            ("info", "TEST: spool #300 ('N', #00ff00, 100g left) assigned to lane1")]
        # The replay passes the recorded keywords on; the apply step's direct
        # call passes none.
        assert logger.passed == [("info", {"console_only": False}),
                                 ("debug", {"only_debug": False, "traceback": None}),
                                 ("info", {})]
        assert lane.spool_id == 300

    def test_a_failed_request_on_the_thread_is_deferred_in_order(self):
        # The client logs a failed PATCH itself; on the worker it must record
        # into the same deferred lines, not reach AFC's logger from the thread.
        matched = self._matched(300, "AABB", 100,
                                {"id": 88, "name": "N", "color_hex": "00ff00"})
        slot = {"uid": "AABB", "material": "PLA", "color_hex": "00ff00", "diameter": 1.75,
                "multi_color": ["00ff00"]}
        queue, reactor = self._WriteQueue(), self._Reactor()
        afc, lane, logger = self._run(slot, self._routes([matched]), reactor=reactor,
                                      queue=queue)
        queue.run()
        assert logger.messages == []
        reactor.run()
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("error", 'Spoolman PATCH /v1/filament/88 failed; request body: '
                      '{"material": "PLA", "density": 1.24, "diameter": 1.75, '
                      '"settings_bed_temp": 60}'),
            ("info", "TEST: spool #300 ('N', #00ff00, 100g left) assigned to lane1")]
        # The shared client is back on AFC's logger once the worker is done.
        assert afc._afc_spoolman_client_cache._spoolman_proxy("PATCH", "/v1/x") is None
        assert logger.messages[-1] == (
            "error", "Spoolman PATCH /v1/x failed; request body: (none)")
        assert lane.spool_id == 300

    def test_tag_weight_seeds_creation_only(self):
        afc, lane, logger = self._run({**self._BASE_TAG, "weight_g": 750},
                                      self._base_routes(), allow_create=True)
        # The tag's weight is net filament, never the empty spool's tare.
        assert afc.moonraker.sent("POST", "/v1/filament") == [self._base_filament(750.0)]
        assert afc.moonraker.sent("POST", "/v1/spool") == [
            {"filament_id": 99, "initial_weight": 750.0, "remaining_weight": 750.0}]
        assert logger.messages == self._base_logs("750.0", "750")

    def test_no_tag_weight_falls_back_to_1000(self):
        afc, lane, logger = self._run(dict(self._BASE_TAG), self._base_routes(),
                                      allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == [self._base_filament(1000)]
        assert afc.moonraker.sent("POST", "/v1/spool") == [
            {"filament_id": 99, "initial_weight": 1000, "remaining_weight": 1000}]
        assert logger.messages == self._base_logs("1000", "1000")

    def test_invalid_tag_weight_falls_back_to_1000(self):
        afc, lane, logger = self._run({**self._BASE_TAG, "weight_g": "junk"},
                                      self._base_routes(), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == [self._base_filament(1000)]
        assert logger.messages == self._base_logs("1000", "1000")

    def test_tag_density_wins(self):
        afc, lane, logger = self._run({**self._BASE_TAG, "density": 1.31},
                                      self._base_routes(), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == [
            {**self._base_filament(1000), "density": 1.31}]
        assert logger.messages == self._base_logs("1000", "1000")

    def test_material_table_density_without_tag(self):
        afc, lane, logger = self._run({**self._BASE_TAG, "material": "PETG"},
                                      self._base_routes(), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == [
            {**self._base_filament(1000), "name": "BQ Tech PETG", "material": "PETG",
             "density": 1.27, "settings_bed_temp": 75}]
        assert logger.messages == [
            ("info", "TEST: created filament #99 in Spoolman:\n"
                     "  vendor: BQ Tech\n"
                     "  material: PETG\n"
                     "  color: #aa0000\n"
                     "  diameter: 1.75mm\n"
                     "  bed temp: 75°C"),
            ("info", "TEST: created spool #500 in Spoolman:\n"
                     "  filament weight: 1000g\n"
                     "  remaining: 1000g"),
            ("info", "TEST: spool #500 ('BQ Tech PETG', #aa0000, 1000g left) assigned to "
                     "lane1")]

    def test_sku_passed_as_article_number(self):
        afc, lane, logger = self._run({**self._BASE_TAG, "sku": " AC123 "},
                                      self._base_routes(), allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == [
            {**self._base_filament(1000), "article_number": "AC123"}]
        logs = self._base_logs("1000", "1000")
        logs[0] = ("info", logs[0][1] + "\n  SKU: AC123")
        assert logger.messages == logs

    def test_no_sku_passes_none(self):
        afc, lane, logger = self._run({**self._BASE_TAG, "sku": "  "}, self._base_routes(),
                                      allow_create=True)
        assert afc.moonraker.sent("POST", "/v1/filament") == [
            {"name": "BQ Tech PLA", "vendor_id": 7, "material": "PLA", "density": 1.24,
             "diameter": 1.75, "color_hex": "aa0000", "settings_bed_temp": 60,
             "weight": 1000}]
        assert logger.messages == self._base_logs("1000", "1000")

    def test_drying_written_on_create(self):
        afc, lane, logger = self._run({**self._BASE_TAG, "drying_temp": 70,
                                       "drying_time_h": 8}, self._base_routes(),
                                      allow_create=True)
        assert afc.moonraker.sent("PATCH", "/v1/filament/99") == [
            {"extra": {"drying_temp_c": "70", "drying_time_h": "8"}}]
        assert logger.messages == self._base_logs("1000", "1000")

    def test_no_drying_write_without_values(self):
        hooks, prepare = self._hooked_client("write_filament_drying")
        afc, lane, logger = self._run({**self._BASE_TAG, "drying_temp": 0,
                                       "drying_time_h": None}, self._base_routes(),
                                      allow_create=True, prepare=prepare)
        assert hooks["write_filament_drying"].calls == []
        assert afc.moonraker.sent("PATCH", "/v1/filament/99") == []
        assert afc.moonraker.sent("GET", "/v1/field/filament") == []
        assert logger.messages == self._base_logs("1000", "1000")

    def test_create_without_roll_id_writes_none(self):
        hooks, prepare = self._hooked_client("write_tray_uid")
        afc, lane, logger = self._run(dict(self._BASE_TAG), self._base_routes(),
                                      allow_create=True, prepare=prepare)
        assert hooks["write_tray_uid"].calls == []
        assert afc.moonraker.sent("PATCH", "/v1/spool/500") == [
            {"extra": {"card_uids": '"AABBCCDD"'}}]
        assert logger.messages == self._base_logs("1000", "1000")

    def test_create_stamps_the_roll_id_on_the_new_spool(self):
        routes = self._base_routes()
        # The new spool already carries its chip UID by the time the roll id lands.
        routes[("GET", "/v1/spool/500")] = {"id": 500,
                                            "extra": {"card_uids": '"AABBCCDD"'}}
        afc, lane, logger = self._run({**self._BASE_TAG, "tray_uid": "CF34CF1D"}, routes,
                                      allow_create=True)
        assert afc.moonraker.sent("PATCH", "/v1/spool/500") == [
            {"extra": {"card_uids": '"AABBCCDD"'}},
            {"extra": {"card_uids": '"AABBCCDD"', "tray_uid": '"cf34cf1d"'}}]
        assert lane.spool_id == 500
        assert logger.messages == self._base_logs("1000", "1000")

    @staticmethod
    def _hooked_client(*names: str) -> Tuple[Dict[str, Hook], Callable[[Any], None]]:
        """
        Seed the AFC's cached client with a real SpoolmanClient whose named
        methods are wrapped in Hooks, so a test can see whether the sync called them.

        :param names: SpoolmanClient methods to wrap
        :return tuple: (hooks by name, the _run prepare callback)
        """
        hooks: Dict[str, Hook] = {}

        def prepare(afc: Any) -> None:
            """
            :param afc: the AFC object the sync will use
            """
            client = SpoolmanClient(afc.moonraker)
            for name in names:
                hooks[name] = Hook(getattr(client, name))
                setattr(client, name, hooks[name])
            afc._afc_spoolman_client_cache = client

        return hooks, prepare

    def _base_routes(self) -> Dict[Tuple[str, str], Any]:
        """
        :return dict: create routes for _BASE_TAG, Spoolman echoing what it stores
        """
        return self._create_routes(
            lambda body: {"id": 99, **body}, patched={"id": 99},
            spool=lambda body: {"id": 500, **body},
            **{"GET /v1/vendor?name=BQ%20Tech": [{"id": 7, "name": "BQ Tech"}]})

    @staticmethod
    def _base_filament(weight: Any) -> Dict[str, Any]:
        """
        :param weight: the filament weight sent
        :return dict: the filament POST body for _BASE_TAG
        """
        return {"name": "BQ Tech PLA", "vendor_id": 7, "material": "PLA", "density": 1.24,
                "diameter": 1.75, "color_hex": "aa0000", "settings_bed_temp": 60,
                "weight": weight}

    @staticmethod
    def _base_logs(weight: str, left: str) -> List[Tuple[str, str]]:
        """
        :param weight: the spool weight as logged
        :param left: the remaining grams in the assignment line
        :return list: the logs of creating and assigning _BASE_TAG's spool
        """
        return [
            ("info", "TEST: created filament #99 in Spoolman:\n"
                     "  vendor: BQ Tech\n"
                     "  material: PLA\n"
                     "  color: #aa0000\n"
                     "  diameter: 1.75mm\n"
                     "  bed temp: 60°C"),
            ("info", f"TEST: created spool #500 in Spoolman:\n"
                     f"  filament weight: {weight}g\n"
                     f"  remaining: {weight}g"),
            ("info", f"TEST: spool #500 ('BQ Tech PLA', #aa0000, {left}g left) assigned to "
                     f"lane1")]


class TestAFCUnitRFIDResolveAutoCreate:
    class _UnreadableLane:
        """A lane whose unit cannot be read, so the auto-create helper raises."""

        @property
        def unit_obj(self) -> Any:
            """
            :return Any: never returns
            """
            error_str = "lane gone"
            raise RuntimeError(error_str)

    def test_mixin_resolve_auto_create_prefers_lane(self):
        reader, lane = rfid_reader()
        assert reader.auto_create is False
        lane.unit_obj.auto_spoolman_create = True
        assert reader._resolve_auto_create(lane) is True
        # When the helper raises, the unit default stands.
        reader.auto_create = True
        assert reader._resolve_auto_create(self._UnreadableLane()) is True
        reader.auto_create = False
        assert reader._resolve_auto_create(self._UnreadableLane()) is False
        assert reader.logger.messages == []


class TestAFCUnitRFIDRecordTagRead:
    def test_stores_under_string_key_and_returns_record(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        assert not hasattr(reader, "_tag_reads")
        rec = reader.record_tag_read("lane1", {"material": "PLA", "sku": ""})
        assert rec == {"material": "PLA", "decoded": True, "scan_time": 1000.0}
        assert reader._tag_reads == {"lane1": rec}
        assert reader.logger.messages == []

    def test_failed_decode_records_uid(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        rec = reader.record_tag_read(3, None, decoded=False, uid="AABB",
                                     tag_type="MifareClassic1k")
        assert rec == {"uid": "AABB", "tag_type": "MifareClassic1k", "decoded": False,
                       "scan_time": 1000.0}
        assert reader._tag_reads == {"3": rec}
        assert reader.logger.messages == []

    def test_new_read_overwrites_old(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        reader.record_tag_read("lane1", {"material": "PLA"})
        reader.record_tag_read("lane2", {"material": "ABS"})
        rfid_wall_clock[0] = 1005.0
        reader.record_tag_read("lane1", {"material": "PETG"})
        assert reader._tag_reads == {
            "lane1": {"material": "PETG", "decoded": True, "scan_time": 1005.0},
            "lane2": {"material": "ABS", "decoded": True, "scan_time": 1000.0}}
        assert reader.logger.messages == []


class TestAFCUnitRFIDLastReadsStatus:
    def test_empty_before_first_read(self):
        reader, _ = rfid_reader()
        assert reader.last_reads_status() == {}
        assert not hasattr(reader, "_tag_reads")
        assert reader.logger.messages == []

    def test_returns_copy(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        reader.record_tag_read("lane1", {"material": "PLA"})
        status = reader.last_reads_status()
        assert status == {"lane1": {"material": "PLA", "decoded": True, "scan_time": 1000.0}}
        status.clear()
        assert reader.last_reads_status() == {
            "lane1": {"material": "PLA", "decoded": True, "scan_time": 1000.0}}
        assert reader.logger.messages == []


class TestAFCUnitRFIDUndecodedHint:
    def test_hint_for_fresh_undecoded_read(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        reader.record_tag_read("lane1", None, decoded=False, uid="AABB",
                               tag_type="MifareClassic1k")
        assert reader.undecoded_hint("lane1") == (
            " (saw tag UID AABB, MifareClassic1k, no decoder/key matched)")
        assert reader.logger.messages == []

    def test_hint_without_tag_type(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        reader.record_tag_read("lane1", None, decoded=False, uid="AABB")
        assert reader.undecoded_hint("lane1") == " (saw tag UID AABB, no decoder/key matched)"
        assert reader.logger.messages == []

    def test_no_hint_for_decoded_read(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        reader.record_tag_read("lane1", {"material": "PLA", "uid": "AABB"})
        assert reader.undecoded_hint("lane1") == ""
        assert reader.logger.messages == []

    def test_no_hint_without_uid(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        reader.record_tag_read("lane1", None, decoded=False)
        assert reader.undecoded_hint("lane1") == ""
        assert reader.logger.messages == []

    def test_no_hint_when_stale(self, rfid_wall_clock):
        reader, _ = rfid_reader()
        reader.record_tag_read("lane1", None, decoded=False, uid="AABB")
        rfid_wall_clock[0] = 1010.0
        assert reader.undecoded_hint("lane1") == " (saw tag UID AABB, no decoder/key matched)"
        rfid_wall_clock[0] = 1011.0
        assert reader.undecoded_hint("lane1") == ""
        assert reader.undecoded_hint("lane1", max_age=20.0) == (
            " (saw tag UID AABB, no decoder/key matched)")
        assert reader.logger.messages == []

    def test_no_hint_for_unknown_key(self):
        reader, _ = rfid_reader()
        assert reader.undecoded_hint("nope") == ""
        assert reader.logger.messages == []


class TestAFCUnitRFIDApplyToLane:
    # map_tag_to_slot_info's answer for an empty read.
    _DEFAULTS = {"material": "", "color_hex": "", "multi_color": [], "is_dual_color": False,
                 "sku": "", "brand": "", "sub_type": "", "diameter": 1.75,
                 "extruder_temp": None, "bed_temp": None, "mfg_date": "", "uid": "",
                 "weight_g": None}

    def test_sync_exception_warns(self, rfid_wall_clock):
        reader, lane = rfid_reader()
        reader.afc.spoolman = "http://spoolman:7912"
        # A moonraker without a host: building the Spoolman client raises.
        reader.afc.moonraker = object()
        out = reader.apply_to_lane(lane, {"uid": "aa", "filament": {"type": "PLA"}})
        assert out == {**self._DEFAULTS, "material": "PLA", "uid": "aa", "bed_temp": 60}
        assert (lane.material, lane.bed_temp, lane.weight) == ("PLA", 60.0, 1000)
        assert reader.logger.messages == [
            ("warning", "ACE2 RFID Spoolman sync failed: 'object' object has no attribute "
                        "'host'")]
        assert reader.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n"
                     "  Name: PLA\n"
                     "  Material: PLA\n"
                     "  Diameter: 1.75mm\n"
                     "  Bed temp: 60°C\n"
                     "  Tag UID: aa")]
        assert reader.last_reads_status() == {
            "lane1": {"material": "PLA", "is_dual_color": False, "diameter": 1.75,
                      "bed_temp": 60, "uid": "aa", "decoded": True, "scan_time": 1000.0}}

    def test_mixin_apply_to_lane_maps_applies_and_syncs(self, rfid_wall_clock):
        reader, lane = rfid_reader()
        moonraker = RfidSpoolmanMoonraker(
            {("GET", "/v1/info"): {"version": "0.22.1"}, ("GET", "/v1/spool"): []},
            logger=reader.logger)
        reader.afc.moonraker = moonraker
        reader.afc.spoolman = "http://spoolman:7912"
        out = reader.apply_to_lane(
            lane, {"uid": "aa", "filament": {"type": "PLA", "weight_g": 250}})
        # The weight travels inside slot_info; the sync only uses it to create.
        assert out == {**self._DEFAULTS, "material": "PLA", "uid": "aa", "weight_g": 250,
                       "bed_temp": 60}
        assert (lane.material, lane.bed_temp, lane.weight) == ("PLA", 60.0, 1000)
        assert moonraker.calls == [("GET", "/v1/info", None, False),
                                   ("GET", "/v1/spool", None, False)]
        assert reader.logger.messages == [
            ("info", "ACE2 RFID: no Spoolman spool matches UID AA and auto-create is OFF "
                     "(set 'auto_spoolman_create: True' to create one)")]
        assert reader.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n"
                     "  Name: PLA\n"
                     "  Material: PLA\n"
                     "  Diameter: 1.75mm\n"
                     "  Bed temp: 60°C\n"
                     "  Tag weight: 250g\n"
                     "  Tag UID: aa")]
        assert reader.last_reads_status() == {
            "lane1": {"material": "PLA", "is_dual_color": False, "diameter": 1.75,
                      "bed_temp": 60, "uid": "aa", "weight_g": 250, "decoded": True,
                      "scan_time": 1000.0}}

    def test_mixin_apply_to_lane_skips_sync_without_spoolman(self, rfid_wall_clock):
        reader, lane = rfid_reader()
        afc = reader.afc
        # Spoolman answers, so only the missing URL or AFC keeps the sync away.
        moonraker = RfidSpoolmanMoonraker(
            {("GET", "/v1/info"): {"version": "0.22.1"}, ("GET", "/v1/spool"): []},
            logger=reader.logger)
        afc.moonraker = moonraker
        assert afc.spoolman is None
        out = reader.apply_to_lane(lane, {"uid": "aa"})
        assert out == {**self._DEFAULTS, "uid": "aa"}
        assert lane.weight == 1000
        # Without an AFC there is no sync either, and no crash.
        reader.afc = None
        second = reader.apply_to_lane(lane, {"uid": "bb", "filament": {"type": "PLA"}})
        assert second == {**self._DEFAULTS, "material": "PLA", "uid": "bb"}
        assert (lane.material, lane.bed_temp) == ("PLA", 60.0)
        # A blank URL gets past the sync's own None check, so only this guard
        # stops it. A sync would have written ABS's bed default into the read.
        reader.afc = afc
        afc.spoolman = ""
        third = reader.apply_to_lane(lane, {"uid": "cc", "filament": {"type": "ABS"}})
        assert third == {**self._DEFAULTS, "material": "ABS", "uid": "cc"}
        assert (lane.material, lane.bed_temp) == ("PLA", 60.0)
        assert moonraker.calls == []
        assert reader.logger.messages == []
        assert reader.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n  Diameter: 1.75mm\n  Tag UID: aa"),
            ("info", "ACE2 RFID: read spool on lane1\n"
                     "  Name: PLA\n"
                     "  Material: PLA\n"
                     "  Diameter: 1.75mm\n"
                     "  Tag UID: bb"),
            ("info", "ACE2 RFID: read spool on lane1\n"
                     "  Name: ABS\n"
                     "  Material: ABS\n"
                     "  Diameter: 1.75mm\n"
                     "  Tag UID: cc")]


class TestAFCUnitRFIDConsoleReadOut:
    class _BrokenGcode:
        """A gcode object whose console write fails."""

        def respond_info(self, msg: str, log: bool = True) -> None:
            """
            :param msg: console line
            :param log: Klipper's flag
            """
            error_str = "ui down"
            raise RuntimeError(error_str)

    def test_no_gcode_noop(self):
        reader, lane = rfid_reader()
        gcode = reader.gcode
        moonraker = RfidSpoolmanMoonraker({("GET", "/v1/spool/5"): {"id": 5}})
        reader.afc.moonraker = moonraker
        reader.gcode = None
        lane.spool_id = 5
        reader._console_read_out(lane, {"material": "PLA"})
        assert gcode.messages == []
        assert moonraker.calls == []
        assert reader.logger.messages == []

    def test_prints_summary_when_multiline(self):
        reader, lane = rfid_reader()
        moonraker = RfidSpoolmanMoonraker()
        reader.afc.moonraker = moonraker
        reader._console_read_out(lane, {"material": "PLA", "color_hex": "00ff00"})
        assert reader.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n"
                     "  Name: PLA\n"
                     "  Material: PLA\n"
                     "  Color: #00ff00")]
        assert moonraker.calls == []
        assert reader.logger.messages == []

    def test_enriches_when_spool_id_present(self):
        reader, lane = rfid_reader()
        moonraker = RfidSpoolmanMoonraker({("GET", "/v1/spool/7"): {
            "id": 7, "remaining_weight": 512.6,
            "filament": {"name": "Enriched", "vendor": {"name": "Bambu Lab"}}}})
        reader.afc.moonraker = moonraker
        lane.spool_id = 7
        slot = {"material": "PLA"}
        reader._console_read_out(lane, slot)
        assert reader.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n"
                     "  Name: Enriched\n"
                     "  Brand: Bambu Lab\n"
                     "  Material: PLA\n"
                     "  Remaining: 513g\n"
                     "  Spoolman ID: 7")]
        assert moonraker.calls == [("GET", "/v1/spool/7", None, True)]
        assert reader.afc._afc_spoolman_client_cache._mr is moonraker
        assert slot == {"material": "PLA"}
        assert reader.logger.messages == []

    def test_afc_none_skips_enrich(self):
        reader, lane = rfid_reader()
        lane.spool_id = 7
        # An AFC with no moonraker, then no AFC at all: neither enriches.
        assert reader.afc.moonraker is None
        reader._console_read_out(lane, {"material": "PLA"})
        reader.afc = None
        reader._console_read_out(lane, {"material": "PLA"})
        assert reader.gcode.messages == [
            ("info", "ACE2 RFID: read spool on lane1\n  Name: PLA\n  Material: PLA"),
            ("info", "ACE2 RFID: read spool on lane1\n  Name: PLA\n  Material: PLA")]
        assert reader.logger.messages == []

    def test_bare_uid_summary_not_printed(self):
        reader, lane = rfid_reader()
        reader._console_read_out(lane, {"uid": "AABB"})
        assert reader.gcode.messages == []
        assert reader.logger.messages == []

    def test_no_lane_name(self):
        reader, lane = rfid_reader()
        lane.name = ""
        reader._console_read_out(lane, {"material": "PLA"})
        assert reader.gcode.messages == [
            ("info", "ACE2 RFID: read spool\n  Name: PLA\n  Material: PLA")]
        assert reader.logger.messages == []

    def test_exception_logged_debug(self):
        reader, lane = rfid_reader()
        reader.gcode = self._BrokenGcode()
        reader._console_read_out(lane, {"material": "PLA"})
        assert reader.logger.messages == [
            ("debug", "ACE2 RFID console read-out skipped: ui down")]
