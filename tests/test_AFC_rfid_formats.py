"""
The QIDI, TigerTag, OpenSpool, OpenTag3D and SpoolEase decoders in
extras/AFC_rfid_readers.py, run on tags recorded by OpenRFID.

Expected values are OpenRFID's for the same dumps (its test/tags/*.yml), so
a tag reads the same through AFC as on a Snapmaker U1. Where AFC differs on
purpose it says so: OpenRFID fills bed and drying temperatures a tag does not
carry from a per-material table, and AFC leaves them unset for its own
filament defaults to supply.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

from extras.AFC_RFID import map_tag_to_slot_info
from extras.AFC_rfid_readers import (
    NTAG_MAX_BYTES,
    decode_ndef,
    decode_openspool,
    decode_opentag3d,
    decode_qidi,
    decode_spoolease,
    decode_tigertag,
    encode_tag_payload,
    ndef_bytes_needed,
    ndef_records,
    read_tag,
)
from tests.test_AFC_rfid_readers import RfidSimLink, RfidSimTag

TAGS = Path(__file__).parent / "fixtures" / "rfid_tags"


def _dump(name: str) -> bytes:
    """
    :param name: fixture file name
    :return bytes: the recorded tag image, from byte 0
    """
    return (TAGS / name).read_bytes()


def _ntag(image: bytes) -> RfidSimTag:
    """
    An NTAG215 holding a recorded image. NTAG pages 0-2 carry the 7-byte UID
    (BCC0 at byte 3); a dump with a blank UID area gets a placeholder.

    :param image: NTAG image
    :return RfidSimTag: the tag
    """
    uid = image[0:3] + image[4:8]
    if not any(uid):
        uid = bytes.fromhex("04112233445566")
    return RfidSimTag(uid, image, ntag=True, pages=135)


def _ndef_image(*records: bytes) -> bytes:
    """
    An NTAG image whose NDEF message holds the given encoded records.

    :param records: short records, each from _record()
    :return bytes: image from page 0
    """
    msg = b"".join(records)
    length = (bytes([len(msg)]) if len(msg) < 0xFF
              else b"\xff" + struct.pack(">H", len(msg)))
    return (bytes(12) + b"\xe1\x10\x3e\x00" + b"\x03" + length
            + msg + b"\xfe").ljust(540, b"\x00")


def _record(tnf: int, rtype: bytes, payload: bytes, last: bool = True) -> bytes:
    """
    :return bytes: one NDEF record, short form when the payload fits
    """
    hdr = tnf | (0x40 if last else 0)
    if len(payload) < 0x100:
        return bytes([hdr | 0x10, len(rtype), len(payload)]) + rtype + payload
    return (bytes([hdr, len(rtype)]) + struct.pack(">I", len(payload))
            + rtype + payload)


def _openspool(**fields) -> bytes:
    """
    :return bytes: an NTAG image carrying an OpenSpool JSON record
    """
    data = {"protocol": "openspool", "version": "1.0", **fields}
    return _ndef_image(_record(0x02, b"application/json",
                               json.dumps(data).encode()))


# ── the recorded tags, through read_tag and the simulated reader ─────────────

def test_openspool_generic_magenta():
    res = read_tag(RfidSimLink(_ntag(_dump("openspool_generic_pla_magenta.bin"))))
    fil = res["filament"]
    assert fil == {
        "manufacturer": "Generic", "type": "PLA", "detailed": "",
        "color_argb": 0xFFF330F9, "diameter_mm": 1.75,
        "hotend_min_c": 180, "hotend_max_c": 210,
        # Not on this tag. OpenRFID fills weight 1000 g, bed 60 C and drying
        # 50 C / 8 h from its defaults; AFC leaves them unset, as the U1's own
        # OpenSpool reader does for weight.
        "weight_g": None, "bed_temp_c": None,
    }


def test_openspool_spoolpainter():
    fil = read_tag(RfidSimLink(_ntag(_dump("openspool_elegoo_pla_grey.bin"))))["filament"]
    assert (fil["manufacturer"], fil["type"], fil["color_argb"]) == (
        "Elegoo", "PLA", 0xFFA5A5A5)
    assert (fil["hotend_min_c"], fil["hotend_max_c"]) == (200, 230)


def test_tigertag_elegoo_grey():
    fil = read_tag(RfidSimLink(_ntag(_dump("tigertag_elegoo_pla_grey.bin"))))["filament"]
    assert (fil["manufacturer"], fil["type"], fil["detailed"]) == (
        "ELEGOO", "PLA", "Basic")
    assert fil["color_argb"] == 0xFFBCBCBC
    assert fil["weight_g"] == 1000 and fil["diameter_mm"] == 1.75
    assert (fil["hotend_min_c"], fil["hotend_max_c"]) == (190, 240)
    assert (fil["drying_temp_c"], fil["drying_time_h"]) == (50, 8)
    assert fil["production"] == "2026-04-03"


def test_opentag3d_polar_light_blue():
    fil = read_tag(RfidSimLink(_ntag(_dump("opentag3d_polar_pla_light_blue.bin"))))["filament"]
    assert (fil["manufacturer"], fil["type"], fil["detailed"]) == (
        "Polar Filament", "PLA", "Pure")
    assert fil["weight_g"] == 1000
    assert (fil["hotend_min_c"], fil["hotend_max_c"], fil["bed_temp_c"]) == (
        205, 245, 60)
    assert fil["drying_temp_c"] == 65
    assert fil["production"] == "2026-04-03"


def test_spoolease_esun_multicolor():
    res = read_tag(RfidSimLink(_ntag(_dump("spoolease_esun_pla_plus_multi.bin"))))
    info = map_tag_to_slot_info(res)
    assert (info["brand"], info["material"], info["sub_type"]) == (
        "eSUN", "PLA", "Plus")
    assert info["multi_color"] == ["b87333", "50c878", "d4af37"]
    assert info["color_count"] == 3
    assert (info["extruder_temp_min"], info["extruder_temp_max"]) == (190, 240)


def test_qidi_petg_red():
    image = _dump("qidi_petg_red.bin")
    link = RfidSimLink(RfidSimTag(image[:4], image))
    fil = read_tag(link)["filament"]
    assert fil == {"manufacturer": "QIDI", "type": "PETG", "detailed": "",
                   # QIDI's table is bare RRGGBB; read as opaque.
                   "color_argb": 0xFFFF362D, "diameter_mm": 1.75}
    # Snapmaker is turned away at sector 0. BTT shares the default key, so it
    # reads its blocks through 20 before its fingerprint says no. QIDI then
    # reads its one sector.
    assert [e for e in link.trace if e.startswith("AUTH")] == [
        "AUTH 0", "AUTH 0", "AUTH 4", "AUTH 8", "AUTH 16", "AUTH 20", "AUTH 4"]


# ── reading only as far as the message goes ─────────────────────────────────

def test_a_long_ndef_message_is_read_to_its_end_and_no_further():
    image = _dump("openspool_generic_pla_magenta.bin")
    assert ndef_bytes_needed(image) == 161          # past the 128-byte first read
    link = RfidSimLink(_ntag(image))
    assert read_tag(link)["filament"]["manufacturer"] == "Generic"
    # Pages 0-31 (the first read), then 32-40 for the rest of the message.
    # The AFC block (pages 32-39) comes out of those bytes, not a second read.
    assert [e for e in link.trace if e.startswith("READ")] == [
        f"READ {p}" for p in range(0, 44, 4)]


def test_the_first_read_alone_would_have_cut_the_message_short():
    image = _dump("openspool_generic_pla_magenta.bin")
    assert decode_ndef(image[:128]) is None
    assert decode_ndef(image[:161]) is not None


def test_a_short_tag_reads_no_further_than_before():
    link = RfidSimLink(_ntag(_dump("tigertag_elegoo_pla_grey.bin")))
    read_tag(link)
    # The 128-byte read and the AFC block probe, as for any NTAG.
    assert [e for e in link.trace if e.startswith("READ")] == [
        f"READ {p}" for p in range(0, 32, 4)] + ["READ 32", "READ 36"]


@pytest.mark.parametrize("tlvs, needed", [
    (b"\x03\x10", 16 + 2 + 0x10),                      # short length
    (b"\x00\x00\x03\x05", 16 + 4 + 5),                 # NULL TLVs skipped
    (b"\x01\x03\xa0\x0c\x34\x03\x08", 16 + 5 + 2 + 8),  # lock TLV stepped over
    (b"\x03\xff\x01\x00", 16 + 4 + 0x100),             # three-byte length
    (b"\x03\xff\x7f\xff", NTAG_MAX_BYTES),             # capped at NTAG216
    (b"\xfe", None),                                   # terminator first
    (b"\x7b\x00\x65\x00", None),                       # not NDEF (Anycubic)
])
def test_ndef_bytes_needed(tlvs, needed):
    assert ndef_bytes_needed(bytes(16) + tlvs + bytes(16)) == needed


# ── existing formats keep their tags ─────────────────────────────────────────

def test_an_afc_written_tag_still_reads_as_before():
    image = bytes(16) + encode_tag_payload(
        weight_g=750, ftype="PETG", manufacturer="Anycubic",
        color_argb=0xFF00FF00)
    tag = RfidSimTag(bytes.fromhex("04112233445566"), image, ntag=True, pages=135)
    fil = read_tag(RfidSimLink(tag))["filament"]
    assert (fil["manufacturer"], fil["type"], fil["weight_g"]) == (
        "Anycubic", "PETG", 750)


def test_a_blank_classic_tag_decodes_as_nothing():
    fil = read_tag(RfidSimLink(RfidSimTag(bytes.fromhex("01020304"))))["filament"]
    assert fil is None


# ── the decoders on their own ────────────────────────────────────────────────

def test_openspool_u1_extended_fields():
    fil = decode_ndef(_openspool(
        brand="Elegoo", type="petg", subtype="Rapid", color_hex="#AFAFAF",
        alpha="80", additional_color_hexes=["EEFFEE", "nope", "FF00FF"],
        min_temp="230", max_temp="260", bed_min_temp=0, bed_max_temp=80,
        weight=750, diameter="2.85"))
    assert fil == {
        "manufacturer": "Elegoo", "type": "PETG", "detailed": "Rapid",
        "color_argb": 0x80AFAFAF, "diameter_mm": 2.85, "weight_g": 750,
        "hotend_min_c": 230, "hotend_max_c": 260, "bed_temp_c": 80,
        "color_count": 3, "colors_argb": [0x80AFAFAF, 0xFFEEFFEE, 0xFFFF00FF],
    }


def test_openspool_needs_its_protocol_field():
    records = ndef_records(_ndef_image(_record(
        0x02, b"application/json", b'{"protocol": "other", "type": "PLA"}')))
    assert decode_openspool(records) is None


def test_spoolease_url_with_rgba_colours():
    url = b"tag.spoolease.io/S1?M=PLA-S&MS=Matte&B=Acme&CC=FF000080;00FF00&NN=190&NX=220&WL=500"
    fil = decode_spoolease(ndef_records(_ndef_image(_record(0x01, b"U", b"\x04" + url))))
    assert fil["type"] == "PLA"                        # PLA-S is PLA
    assert fil["colors_argb"] == [0x80FF0000, 0xFF00FF00]
    assert (fil["hotend_min_c"], fil["hotend_max_c"], fil["weight_g"]) == (190, 220, 500)


def test_spoolease_ignores_other_urls():
    rec = _record(0x01, b"U", b"\x04example.com/S1?M=PLA")
    assert decode_spoolease(ndef_records(_ndef_image(rec))) is None


def test_opentag3d_v1_is_not_read_with_the_v2_map():
    payload = struct.pack(">H", 1000) + b"PLA"
    rec = _record(0x02, b"application/opentag3d", payload)
    assert decode_opentag3d(ndef_records(_ndef_image(rec))) is None


def test_ndef_finds_a_format_in_any_record():
    other = _record(0x01, b"U", b"\x04example.com", last=False)
    spool = _record(0x02, b"application/json",
                    b'{"protocol": "openspool", "type": "abs", "color_hex": "112233"}')
    fil = decode_ndef(_ndef_image(other, spool))
    assert (fil["type"], fil["color_argb"]) == ("ABS", 0xFF112233)


def test_qidi_needs_known_codes_and_a_clean_sector():
    image = bytearray(1024)
    image[64:67] = bytes([0x29, 0x12, 0x01])
    assert decode_qidi(bytes(image))["type"] == "PETG"
    image[70] = 1                                       # trailing data: not QIDI
    assert decode_qidi(bytes(image)) is None
    image[70] = 0
    image[64] = 0x7F                                    # unknown material
    assert decode_qidi(bytes(image)) is None


def test_tigertag_unknown_ids_still_decode():
    u = bytearray(48)
    struct.pack_into(">I", u, 0, 0x5BF59264)
    struct.pack_into(">H", u, 8, 9999)                 # material not in table
    struct.pack_into(">H", u, 14, 9999)                # brand not in table
    u[20:24] = bytes([0, 1, 0, 35])                    # 1 kg
    fil = decode_tigertag(bytes(16) + bytes(u))
    assert (fil["manufacturer"], fil["type"], fil["weight_g"]) == (
        "Unknown(9999)", "Unknown(9999)", 256000)


def test_tigertag_needs_its_id():
    assert decode_tigertag(bytes(16) + bytes(48)) is None


def test_td_reaches_slot_info():
    info = map_tag_to_slot_info({"uid": "01", "filament": {"type": "PLA", "td": 4.2}})
    assert info["td"] == 4.2
