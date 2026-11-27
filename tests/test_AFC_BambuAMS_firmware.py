"""Unit tests for extras/AFC_BambuAMS_firmware.py."""

from __future__ import annotations

import binascii
import json
import os
import pathlib
import time
import zipfile
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Sequence, Tuple

import pytest

from extras import AFC_BambuAMS_firmware as fw_mod
from extras.AFC_BambuAMS import afcBambuAMS
from extras.AFC_BambuAMS_firmware import (
    AmsFirmware,
    board_name,
    board_of_code,
    bus_frames,
    choices_for,
    crc16,
    crc8,
    load_catalog,
    local_images,
    manager,
    read_json,
    updater_dir,
    version_answer,
    version_key,
    version_query,
)
from tests.bambu_helpers import (
    BambuPrinter,
    FakeBridge,
    FakeGcmd,
    FakeReactor,
    LogLine,
    make_bambu_unit,
    make_printer,
    Recorder,
    use_bridges,
)

#: The printer object the AMS updater's shell command is, and the manager's own name.
SHELL = "gcode_shell_command ams_flash"
MANAGER = "AFC_BambuAMS_firmware"
#: The records the updater keeps beside itself.
CATALOG = "ams_firmware.json"
RESULT = "ams_flash_result.json"
STATE = "ams_flash_state.json"

#: Bus addresses as a frame carries them, little-endian: the printer, a boxed unit, an AMS HT.
HOST_LE = b"\x00\x09"
BOXED_LE = b"\x00\x07"
HT_LE = b"\x00\x18"
#: The version query's op.
VERSION_OP = b"\x03\x01"

#: A made-up offline update package's name.
PACK = "pack-a.zip"


# Synthetic bus frames

def ref_crc8(data: bytes) -> int:
    """
    CRC-8 by polynomial long division: x^8 + x^5 + x^4 + x^3 + 1 (0x39), the 0x66 start folded in
    as 0x66 * x^(8 * len), no final XOR.
    """
    value = (0x66 << (8 * len(data))) ^ (int.from_bytes(data, "big") << 8)
    for shift in range(value.bit_length() - 9, -1, -1):
        if value >> (shift + 8) & 1:
            value ^= 0x139 << shift
    return value


def ref_crc16(data: bytes) -> int:
    """CRC-16/CCITT from 0x913D, as binascii computes it."""
    return binascii.crc_hqx(data, 0x913D)


def bus_frame(body: bytes, counter: int = 0, start: int = 0x3D) -> bytes:
    """
    A long bus frame: its header, the header's CRC-8, the body, then a little-endian CRC-16 of
    everything before it.

    :param body: the bytes after the header
    :param counter: the frame counter
    :param start: the first byte
    """
    size = 7 + len(body) + 2
    head = bytes((start, 0x05, counter & 0xFF, counter >> 8, size, 0))
    frame = head + bytes((ref_crc8(head),)) + body
    return frame + ref_crc16(frame).to_bytes(2, "little")


def answer_frame(unit_id: int, version: Sequence[int], code: bytes, dev: bytes = BOXED_LE,
                 op: bytes = VERSION_OP, counter: int = 0) -> bytes:
    """
    A unit's answer to the version query: to the printer from `dev`, the version lowest byte
    first, the model code padded to 16 bytes with NULs, then the id of the unit answering.
    """
    body = HOST_LE + dev + op + bytes(version) + code.ljust(16, b"\0") + bytes((unit_id,))
    return bus_frame(body, counter)


def query_sent(dev: bytes, unit_id: int, counter: int) -> Dict[str, Any]:
    """The raw bridge command carrying the version query to `unit_id` at `dev`."""
    frame = bus_frame(dev + HOST_LE + VERSION_OP + bytes((unit_id,)), counter)
    return {"cmd": "raw", "hex": frame.hex(), "us": 100000}


def short_frame() -> bytes:
    """
    Eight bytes that check out as a frame but for their length: the header claims 8, and its
    CRC-8 is also the low byte of the CRC-16 over the header.
    """
    for counter in range(0x10000):
        head = bytes((0x3D, 0x05, counter & 0xFF, counter >> 8, 8, 0))
        crc = ref_crc16(head)
        if crc & 0xFF == ref_crc8(head):
            return head + crc.to_bytes(2, "little")
    raise AssertionError("no counter gives a header CRC equal to the frame CRC's low byte")


def cut_off_frame() -> bytes:
    """
    A frame whose header claims two bytes more than there are. The bytes there end in their own
    CRC-16, big-endian, so the CRC-16 over all of them is 0, which the missing two read as.
    """
    body = b"\x01\x02\x03"
    size = 7 + len(body) + 4
    head = bytes((0x3D, 0x05, 0, 0, size, 0))
    content = head + bytes((ref_crc8(head),)) + body
    return content + ref_crc16(content).to_bytes(2, "big")


def short_answer(unit_id: int) -> bytes:
    """
    A 35-byte answer, a byte short: its CRC-16 low byte sits where the unit id goes, and the
    counter is picked so that byte is `unit_id`.
    """
    body = HOST_LE + BOXED_LE + VERSION_OP + bytes((4, 3, 2, 1)) + b"N3F05".ljust(16, b"\0")
    for counter in range(0x10000):
        frame = bus_frame(body, counter)
        if frame[33] == unit_id:
            return frame
    raise AssertionError("no counter puts the unit id at the CRC's low byte")


# The updater and its files

class ShellCommand:
    """[gcode_shell_command ams_flash] as klippy holds it."""

    def __init__(self, command: Any) -> None:
        """:param command: a list of arguments (Klipper) or one string (Kalico)"""
        self.command = command


def add_updater(printer: BambuPrinter, folder: pathlib.Path) -> str:
    """
    Set up the AMS updater in `folder`, its command listed as Klipper's gcode_shell_command does.

    :return str: the folder
    """
    printer.add_object(SHELL, ShellCommand(["/usr/bin/python3", f"{folder}/ams_flash.py"]))
    return str(folder)


def image_name(board: str, version: str) -> str:
    """A made-up file name in Bambu's image naming."""
    return f"{board}-firmware-v{version}-20000101000000.bin.sig"


def catalog_entry(board: str, version: str) -> Dict[str, str]:
    """One image to download, from a made-up address."""
    return {"board": board, "version": version, "file": image_name(board, version),
            "url": f"https://example.invalid/{image_name(board, version)}"}


def write_json(path: pathlib.Path, record: Any) -> None:
    """:param record: written to `path` as JSON"""
    path.write_text(json.dumps(record))


def write_catalog(folder: pathlib.Path, images: Any) -> None:
    """:param images: the catalog's image list"""
    write_json(folder / CATALOG, {"images": images})


def write_zip(path: pathlib.Path, members: Sequence[str]) -> None:
    """:param members: the member names, each holding two made-up bytes"""
    with zipfile.ZipFile(path, "w") as z:
        for member in members:
            z.writestr(member, b"\x00\x01")


class FakeMaster:
    """An AFC_BridgeBox as the manager tells it what a unit runs."""

    def __init__(self) -> None:
        """Start with no reports."""
        self.offers: List[Tuple[Any, Optional[Dict[str, Any]]]] = []

    def offer_firmware(self, unit: Any, choice: Optional[Dict[str, Any]]) -> None:
        """
        :param unit: the unit read
        :param choice: the newer image it can take, or None
        """
        self.offers.append((unit, choice))


class FakeUpdater:
    """The ams_flash shell command: each script run, and the result it leaves behind."""

    def __init__(self, folder: str, leaves: Optional[Dict[str, Any]] = None) -> None:
        """
        :param folder: the updater's folder
        :param leaves: the result each run writes; None writes none
        """
        self.path = os.path.join(folder, RESULT)
        self.leaves = leaves
        #: (script, whether a result file was there when it ran)
        self.runs: List[Tuple[str, bool]] = []

    def __call__(self, script: str) -> None:
        """:param script: the gcode run"""
        self.runs.append((script, os.path.exists(self.path)))
        if self.leaves is not None:
            with open(self.path, "w") as f:
                json.dump(self.leaves, f)


class WatchedLock:
    """A bridge's lock that records the raw reply as it is taken and let go."""

    def __init__(self, bridge: FakeBridge) -> None:
        """:param bridge: the bridge it guards"""
        self.bridge = bridge
        self.seen: List[Tuple[str, str]] = []

    def __enter__(self) -> "WatchedLock":
        """:return WatchedLock: this lock"""
        self.seen.append(("taken", self.bridge._last_raw_reply))
        return self

    def __exit__(self, *exc: Any) -> None:
        """Record the reply as the lock is let go."""
        self.seen.append(("let go", self.bridge._last_raw_reply))


def manager_of(printer: BambuPrinter) -> AmsFirmware:
    """:return AmsFirmware: the manager the printer's first unit made"""
    return printer.lookup_object(MANAGER)


# AFCprompt's lines

def begin(title: str) -> LogLine:
    return ("raw", f"// action:prompt_begin {title}")


def text(line: str) -> LogLine:
    return ("raw", f"// action:prompt_text {line}")


def button(label: str, command: str, style: str) -> LogLine:
    return ("raw", f"// action:prompt_button {label}|{command}|{style}")


def footer(label: str, command: str, style: str) -> LogLine:
    return ("raw", f"// action:prompt_footer_button {label}|{command}|{style}")


CANCEL = footer("Cancel", "RESPOND TYPE=command MSG=action:prompt_end", "warning")
CLOSE = footer("Close", "RESPOND TYPE=command MSG=action:prompt_end", "info")
SHOW = ("raw", "// action:prompt_show")


# Module functions

class TestCrc8:
    def test_no_bytes_leave_the_start_value(self):
        assert crc8(b"") == 0x66

    def test_a_clear_top_bit_only_shifts(self):
        # 0x66 ^ 0x66 leaves 0: no step sees the top bit, so none folds in 0x39.
        assert crc8(b"\x66") == 0x00

    def test_a_set_top_bit_folds_in_the_polynomial(self):
        # 0x66 ^ 0xE6 leaves 0x80; by hand: 39 72 E4 F1 DB 8F 27 4E.
        assert crc8(b"\xe6") == 0x4E

    @pytest.mark.parametrize("data", [bytes((0x3D, 0x05, 0x00, 0x00, 0x10, 0x00)),
                                      bytes(range(6)), b"\xff" * 6, b"\x3d\x05\x01\x00\x24"])
    def test_matches_the_long_division(self, data):
        assert crc8(data) == ref_crc8(data)


class TestCrc16:
    def test_no_bytes_leave_the_start_value(self):
        assert crc16(b"") == 0x913D

    def test_a_clear_top_bit_only_shifts(self):
        # 0x913D ^ 0x9100 leaves 0x003D, whose top bit stays clear for all eight steps.
        assert crc16(b"\x91") == 0x3D00

    def test_a_set_top_bit_folds_in_the_polynomial(self):
        # 0x913D ^ 0x1100 leaves 0x803D: shifting alone would give 0x3D00.
        assert crc16(b"\x11") == ref_crc16(b"\x11")
        assert crc16(b"\x11") != 0x3D00

    @pytest.mark.parametrize("data", [bytes(range(34)), b"\xff" * 9, b"\x3d\x05\x00\x00"])
    def test_matches_ccitt(self, data):
        assert crc16(data) == ref_crc16(data)


class TestVersionQuery:
    def test_a_boxed_unit_is_asked_at_the_boxed_address(self):
        # 3D 05, counter 0, 16 bytes; to 0x0700 from the printer; op 03 01 for unit 2.
        expected = bus_frame(BOXED_LE + HOST_LE + VERSION_OP + b"\x02")
        assert version_query(2) == expected.hex()
        assert len(expected) == 16

    def test_an_ht_unit_is_asked_at_the_ht_address(self):
        expected = bus_frame(HT_LE + HOST_LE + VERSION_OP + b"\x81", counter=0x0203)
        assert version_query(0x81, 0x0203) == expected.hex()

    def test_the_counter_and_the_id_keep_their_low_bytes(self):
        # Counter 0x10203 goes out as 03 02; id 0x180 is an AMS HT's, sent as 0x80.
        expected = bus_frame(HT_LE + HOST_LE + VERSION_OP + b"\x80", counter=0x0203)
        assert version_query(0x180, 0x10203) == expected.hex()


class TestBusFrames:
    def test_too_few_bytes_for_a_header_hold_no_frame(self):
        assert bus_frames(bus_frame(b"\x01\x02")[:6]) == []

    def test_one_frame(self):
        frame = bus_frame(b"\x01\x02\x03")
        assert bus_frames(frame) == [frame]

    def test_frames_among_noise_come_back_in_order(self):
        first, second = bus_frame(b"\x01"), bus_frame(b"\x02\x03", counter=1)
        raw = b"\x00\x11" + first + b"\x22" + second + b"\x33\x44"
        assert bus_frames(raw) == [first, second]

    def test_the_scan_resumes_past_a_whole_frame(self):
        # A whole frame inside another one's body is not read again.
        inner = bus_frame(b"\x01\x02")
        outer = bus_frame(inner)
        assert bus_frames(inner) == [inner]
        assert bus_frames(outer) == [outer]

    def test_the_header_must_start_with_3d(self):
        # Both CRCs are right for the 3E it starts with.
        assert bus_frames(bus_frame(b"\x01\x02\x03", start=0x3E)) == []

    def test_the_header_crc_must_match(self):
        frame = bytearray(bus_frame(b"\x01\x02\x03"))
        frame[6] ^= 0xFF
        # The frame CRC still matches what is there.
        frame[-2:] = ref_crc16(bytes(frame[:-2])).to_bytes(2, "little")
        assert bus_frames(bytes(frame)) == []

    def test_a_length_under_nine_is_no_frame(self):
        raw = short_frame()
        assert raw[6] == ref_crc8(raw[:6])
        assert int.from_bytes(raw[6:8], "little") == ref_crc16(raw[:6])
        assert bus_frames(raw) == []

    def test_a_frame_cut_off_is_no_frame(self):
        raw = cut_off_frame()
        assert raw[4] == len(raw) + 2
        assert ref_crc16(raw) == 0
        assert bus_frames(raw) == []

    def test_the_frame_crc_must_match(self):
        frame = bytearray(bus_frame(b"\x01\x02\x03"))
        frame[-1] ^= 0xFF
        assert bus_frames(bytes(frame)) == []

    def test_a_good_frame_right_after_a_bad_one_is_found(self):
        bad = bytearray(bus_frame(b"\x01"))
        bad[-1] ^= 0xFF
        good = bus_frame(b"\x02", counter=1)
        assert bus_frames(bytes(bad) + good) == [good]


class TestVersionAnswer:
    def test_a_capture_that_is_not_hex_is_no_answer(self):
        assert version_answer("3d0g", 2) is None

    def test_an_empty_capture_is_no_answer(self):
        assert version_answer("", 2) is None

    def test_a_boxed_units_answer(self):
        rx = answer_frame(2, (4, 3, 2, 1), b"N3F05").hex()
        assert version_answer(rx, 2) == ("01.02.03.04", "N3F05")

    def test_an_ht_units_answer(self):
        rx = answer_frame(0x80, (31, 0, 10, 2), b"N3S05", dev=HT_LE).hex()
        assert version_answer(rx, 0x80) == ("02.10.00.31", "N3S05")

    def test_the_answer_is_found_among_other_frames(self):
        noise = bus_frame(b"\x01\x02")
        other = answer_frame(1, (9, 9, 9, 9), b"AMS08")
        mine = answer_frame(2, (4, 3, 2, 1), b"N3F05", counter=1)
        assert version_answer((b"\xaa" + noise + other + mine).hex(), 2) == ("01.02.03.04",
                                                                              "N3F05")

    def test_the_first_answer_wins(self):
        first = answer_frame(2, (4, 3, 2, 1), b"N3F05")
        second = answer_frame(2, (0, 0, 3, 1), b"N3F05", counter=1)
        assert version_answer((first + second).hex(), 2) == ("01.02.03.04", "N3F05")

    def test_a_frame_a_byte_short_is_no_answer(self):
        frame = short_answer(2)
        assert (len(frame), frame[33]) == (35, 2)
        assert version_answer(frame.hex(), 2) is None

    def test_another_op_is_no_answer(self):
        rx = answer_frame(2, (4, 3, 2, 1), b"N3F05", op=b"\x03\x02").hex()
        assert version_answer(rx, 2) is None

    def test_an_answer_from_the_ht_address_is_not_a_boxed_units(self):
        rx = answer_frame(2, (4, 3, 2, 1), b"N3F05", dev=HT_LE).hex()
        assert version_answer(rx, 2) is None

    def test_an_answer_from_the_boxed_address_is_not_an_hts(self):
        rx = answer_frame(0x80, (4, 3, 2, 1), b"N3S05", dev=BOXED_LE).hex()
        assert version_answer(rx, 0x80) is None

    def test_another_units_answer_is_no_answer(self):
        rx = answer_frame(1, (4, 3, 2, 1), b"N3F05").hex()
        assert version_answer(rx, 2) is None

    def test_a_code_filling_its_field_is_kept_whole(self):
        rx = answer_frame(2, (4, 3, 2, 1), b"ABCDEFGHIJKLMNOP").hex()
        assert version_answer(rx, 2) == ("01.02.03.04", "ABCDEFGHIJKLMNOP")

    def test_the_code_ends_at_its_first_nul(self):
        rx = answer_frame(2, (4, 3, 2, 1), b"AMS07\0XY").hex()
        assert version_answer(rx, 2) == ("01.02.03.04", "AMS07")

    def test_a_byte_outside_ascii_is_replaced(self):
        rx = answer_frame(2, (4, 3, 2, 1), b"N3F\xff5").hex()
        assert version_answer(rx, 2) == ("01.02.03.04", "N3F\ufffd5")


class TestBoardOfCode:
    @pytest.mark.parametrize("code, board", [("N3F05", "n3f_rev5"), ("N3S05", "n3s_rev5"),
                                             ("AMS07", "ams_rev7"), ("AMS08", "ams_rev8"),
                                             ("N3F12", "n3f_rev12")])
    def test_a_known_models_code(self, code, board):
        assert board_of_code(code) == board

    @pytest.mark.parametrize("code", [None, "", "XYZ05", "N3F5", "n3f05", "N3F05X", " N3F05"])
    def test_any_other_code_names_no_board(self, code):
        assert board_of_code(code) == ""


class TestBoardName:
    @pytest.mark.parametrize("board, name", [("ams_rev8", "AMS 1"), ("n3f_rev5", "AMS 2 Pro"),
                                             ("n3s_rev5", "AMS HT")])
    def test_a_known_board(self, board, name):
        assert board_name(board) == name

    def test_the_family_is_read_before_the_revision(self):
        assert board_name("n3f") == "AMS 2 Pro"

    @pytest.mark.parametrize("board", ["x1c_rev1", "n3fx_rev5", ""])
    def test_any_other_board_is_named_as_itself(self, board):
        assert board_name(board) == board


class TestVersionKey:
    @pytest.mark.parametrize("version", [None, ""])
    def test_no_version_is_empty(self, version):
        assert version_key(version) == ()

    def test_a_version_as_numbers(self):
        assert version_key("01.02.30.04") == (1, 2, 30, 4)

    def test_versions_order_by_number(self):
        # As text, "1.10" sorts before "1.9".
        assert version_key("1.10") > version_key("1.9")


class TestUpdaterDir:
    def test_no_shell_command_is_no_updater(self):
        assert updater_dir(make_printer()) is None

    def test_a_command_list_names_the_folder(self, tmp_path):
        printer = make_printer()
        printer.add_object(SHELL, ShellCommand(["/usr/bin/python3", f"{tmp_path}/a/ams_flash.py",
                                                "--mode", "go"]))
        assert updater_dir(printer) == f"{tmp_path}/a"

    def test_a_command_string_is_split_as_a_shell_would(self, tmp_path):
        # Kalico keeps the command as one string; a quoted path with a space is one part.
        printer = make_printer()
        printer.add_object(SHELL, ShellCommand(f"/usr/bin/python3 '{tmp_path}/my ams/ams_flash.py'"
                                               f" --mode go"))
        assert updater_dir(printer) == f"{tmp_path}/my ams"

    def test_a_home_relative_path_is_expanded(self, monkeypatch, tmp_path):
        monkeypatch.setenv("HOME", str(tmp_path))
        printer = make_printer()
        printer.add_object(SHELL, ShellCommand(["/usr/bin/python3", "~/ams/ams_flash.py"]))
        assert updater_dir(printer) == f"{tmp_path}/ams"

    def test_a_command_without_the_updater_is_no_updater(self):
        printer = make_printer()
        printer.add_object(SHELL, ShellCommand(["/usr/bin/python3", "/opt/x/my_ams_flash.py"]))
        assert updater_dir(printer) is None

    def test_a_bare_file_name_is_no_updater(self):
        # It names no folder, and Klipper's working folder is not one.
        printer = make_printer()
        printer.add_object(SHELL, ShellCommand(["/usr/bin/python3", "ams_flash.py"]))
        assert updater_dir(printer) is None

    def test_a_command_string_a_shell_could_not_split_is_no_updater(self):
        printer = make_printer()
        printer.add_object(SHELL, ShellCommand('/usr/bin/python3 "/opt/u/ams_flash.py'))
        assert updater_dir(printer) is None

    @pytest.mark.parametrize("command", [None, "", []])
    def test_an_empty_command_is_no_updater(self, command):
        printer = make_printer()
        printer.add_object(SHELL, ShellCommand(command))
        assert updater_dir(printer) is None

    def test_an_object_without_a_command_is_no_updater(self):
        printer = make_printer()
        printer.add_object(SHELL, SimpleNamespace())
        assert updater_dir(printer) is None


class TestLoadCatalog:
    def test_a_missing_catalog_lists_nothing(self, tmp_path):
        assert load_catalog(str(tmp_path)) == []

    def test_a_catalog_that_is_not_json_lists_nothing(self, tmp_path):
        (tmp_path / CATALOG).write_text('{"images": [')
        assert load_catalog(str(tmp_path)) == []

    def test_a_catalog_that_is_not_an_object_lists_nothing(self, tmp_path):
        write_json(tmp_path / CATALOG, [catalog_entry("n3f_rev5", "01.02.03.04")])
        assert load_catalog(str(tmp_path)) == []

    @pytest.mark.parametrize("record", [{}, {"images": {"n3f_rev5": "01.02.03.04"}}])
    def test_images_that_are_not_a_list_list_nothing(self, tmp_path, record):
        write_json(tmp_path / CATALOG, record)
        assert load_catalog(str(tmp_path)) == []

    def test_every_whole_entry_is_listed(self, tmp_path):
        entries = [catalog_entry("n3f_rev5", "01.02.03.04"),
                   catalog_entry("ams_rev8", "01.00.00.01")]
        write_catalog(tmp_path, entries)
        assert load_catalog(str(tmp_path)) == entries

    def test_an_entry_that_is_not_an_object_is_left_out(self, tmp_path):
        good = catalog_entry("n3f_rev5", "01.02.03.04")
        write_catalog(tmp_path, ["n3f_rev5", ["board", "file", "url"], good])
        assert load_catalog(str(tmp_path)) == [good]

    @pytest.mark.parametrize("key", ["board", "file", "url"])
    def test_an_entry_missing_a_field_is_left_out(self, tmp_path, key):
        good = catalog_entry("n3f_rev5", "01.02.03.04")
        missing = {k: v for k, v in good.items() if k != key}
        empty = dict(good, **{key: ""})
        write_catalog(tmp_path, [missing, empty, good])
        assert load_catalog(str(tmp_path)) == [good]

    @pytest.mark.parametrize("version", ["1.2.3.4", "01.02.03", "01.02.03.04.05", "v01.02.03.04",
                                         None, 1.2])
    def test_an_entry_with_a_malformed_version_is_left_out(self, tmp_path, version):
        write_catalog(tmp_path, [dict(catalog_entry("n3f_rev5", "01.02.03.04"), version=version)])
        assert load_catalog(str(tmp_path)) == []


class TestLocalImages:
    def test_a_missing_folder_has_no_images(self, tmp_path):
        assert local_images(str(tmp_path / "gone")) == []

    def test_a_signed_image_is_listed(self, tmp_path):
        name = image_name("n3f_rev5", "01.02.03.04")
        (tmp_path / name).write_bytes(b"\x00\x01")
        assert local_images(str(tmp_path)) == [{"board": "n3f_rev5", "version": "01.02.03.04",
                                                "file": name}]

    def test_a_package_lists_each_image_inside_it(self, tmp_path):
        write_zip(tmp_path / PACK, [f"pkg/{image_name('ams_rev8', '01.00.00.01')}",
                                    "pkg/readme.txt", image_name("ams_rev7", "01.00.00.02")])
        assert local_images(str(tmp_path)) == [
            {"board": "ams_rev8", "version": "01.00.00.01", "file": PACK},
            {"board": "ams_rev7", "version": "01.00.00.02", "file": PACK}]

    def test_a_package_that_is_not_a_zip_is_skipped(self, tmp_path):
        (tmp_path / "a-broken.zip").write_bytes(b"not a zip")
        name = image_name("n3f_rev5", "01.02.03.04")
        (tmp_path / name).write_bytes(b"\x00\x01")
        assert local_images(str(tmp_path)) == [{"board": "n3f_rev5", "version": "01.02.03.04",
                                                "file": name}]

    def test_a_package_that_cannot_be_opened_is_skipped(self, tmp_path):
        (tmp_path / "a-folder.zip").mkdir()
        name = image_name("n3f_rev5", "01.02.03.04")
        (tmp_path / name).write_bytes(b"\x00\x01")
        assert local_images(str(tmp_path)) == [{"board": "n3f_rev5", "version": "01.02.03.04",
                                                "file": name}]

    def test_other_files_are_skipped(self, tmp_path):
        unsigned = image_name("n3f_rev5", "01.02.03.04")[:-len(".sig")]
        for name in ("notes.txt", unsigned, "custom.bin.sig"):
            (tmp_path / name).write_bytes(b"\x00\x01")
        assert local_images(str(tmp_path)) == []

    def test_images_come_in_name_order(self, tmp_path, monkeypatch):
        later = image_name("n3s_rev5", "01.00.00.01")
        sooner = image_name("ams_rev8", "01.00.00.02")
        for name in (later, sooner):
            (tmp_path / name).write_bytes(b"\x00\x01")
        # The folder lists backwards, so only sorting puts the images in name order.
        listdir = os.listdir
        monkeypatch.setattr(os, "listdir", lambda folder: sorted(listdir(folder), reverse=True))
        assert local_images(str(tmp_path)) == [
            {"board": "ams_rev8", "version": "01.00.00.02", "file": sooner},
            {"board": "n3s_rev5", "version": "01.00.00.01", "file": later}]


class TestChoicesFor:
    def test_nothing_beside_the_updater_offers_nothing(self, tmp_path):
        assert choices_for(str(tmp_path), "n3f_rev5") == []

    def test_catalog_images_are_downloads_newest_first(self, tmp_path):
        write_catalog(tmp_path, [catalog_entry("n3f_rev5", "01.02.03.04"),
                                 catalog_entry("n3f_rev5", "01.10.00.00"),
                                 catalog_entry("n3s_rev5", "02.00.00.00")])
        assert choices_for(str(tmp_path), "n3f_rev5") == [
            {"version": "01.10.00.00", "file": "", "newest": True},
            {"version": "01.02.03.04", "file": "", "newest": False}]

    def test_a_file_beside_the_updater_takes_the_place_of_a_download(self, tmp_path):
        write_catalog(tmp_path, [catalog_entry("n3f_rev5", "01.02.03.04"),
                                 catalog_entry("n3f_rev5", "01.03.00.00")])
        listed = image_name("n3f_rev5", "01.02.03.04")
        unlisted = image_name("n3f_rev5", "01.04.00.00")
        other_board = image_name("n3s_rev5", "02.00.00.00")
        for name in (listed, unlisted, other_board):
            (tmp_path / name).write_bytes(b"\x00\x01")
        assert choices_for(str(tmp_path), "n3f_rev5") == [
            {"version": "01.04.00.00", "file": unlisted, "newest": True},
            {"version": "01.03.00.00", "file": "", "newest": False},
            {"version": "01.02.03.04", "file": listed, "newest": False}]


class TestReadJson:
    def test_a_missing_file_is_empty(self, tmp_path):
        assert read_json(str(tmp_path / RESULT)) == {}

    def test_a_file_that_is_not_json_is_empty(self, tmp_path):
        (tmp_path / RESULT).write_text('{"ok": ')
        assert read_json(str(tmp_path / RESULT)) == {}

    def test_json_that_is_not_an_object_is_empty(self, tmp_path):
        write_json(tmp_path / RESULT, ["Bambu_AMS_1", True])
        assert read_json(str(tmp_path / RESULT)) == {}

    def test_an_object_is_read(self, tmp_path):
        write_json(tmp_path / RESULT, {"unit": "Bambu_AMS_1", "running": True})
        assert read_json(str(tmp_path / RESULT)) == {"unit": "Bambu_AMS_1", "running": True}


class TestManager:
    def test_the_first_call_makes_the_manager(self):
        printer = make_printer()
        mgr = manager(printer)
        assert isinstance(mgr, AmsFirmware)
        assert mgr.printer is printer
        assert printer.lookup_object(MANAGER) is mgr

    def test_later_calls_return_the_same_one(self):
        printer = make_printer()
        mgr = manager(printer)
        assert manager(printer) is mgr
        # Made once: one ready handler (a second command registration would have raised).
        assert printer._event_handlers["klippy:ready"] == [mgr._handle_ready]

    def test_an_object_already_there_is_returned_as_it_is(self):
        printer = make_printer()
        held = SimpleNamespace()
        printer.add_object(MANAGER, held)
        assert manager(printer) is held
        assert "AFC_BAMBU_FIRMWARE" not in printer.gcode.ready_gcode_handlers


# AmsFirmware

class TestAmsFirmwareInit:
    DESC = ("Flash AMS: pick a Bambu AMS unit and one of Bambu's firmware images for it, and the "
            "AMS updater writes it while Klipper keeps running. AFC_BAMBU_FIRMWARE [UNIT=<unit> "
            "[VERSION=<version> [FILE=<file>] [CONFIRM=1]]]")

    def test_starts_with_nothing_read(self):
        printer = make_printer()
        mgr = AmsFirmware(printer)
        assert mgr.printer is printer
        assert mgr.reactor is printer.reactor
        assert mgr.afc is printer.afc
        assert mgr.logger is printer.afc.logger
        assert (mgr._read, mgr._asking, mgr._timer, mgr._choices) == ({}, None, None, {})
        assert printer.afc.logger.messages == []

    def test_waits_for_klippy_ready(self):
        printer = make_printer()
        mgr = AmsFirmware(printer)
        assert printer._event_handlers["klippy:ready"] == [mgr._handle_ready]

    def test_registers_the_flash_ams_command(self, monkeypatch):
        printer = make_printer()
        calls: List[Tuple[str, Any, Optional[str]]] = []
        real = printer.gcode.register_command

        def record(cmd: str, func: Any, when_not_ready: bool = False,
                   desc: Optional[str] = None) -> Any:
            calls.append((cmd, func, desc))
            return real(cmd, func, when_not_ready=when_not_ready, desc=desc)

        monkeypatch.setattr(printer.gcode, "register_command", record)
        mgr = AmsFirmware(printer)
        assert calls == [("AFC_BAMBU_FIRMWARE", mgr.cmd_AFC_BAMBU_FIRMWARE, self.DESC)]
        assert printer.gcode.ready_gcode_handlers["AFC_BAMBU_FIRMWARE"] == \
            mgr.cmd_AFC_BAMBU_FIRMWARE


class TestAmsFirmwareHandleReady:
    def test_no_updater_reads_nothing(self):
        printer = make_printer()
        mgr = AmsFirmware(printer)
        before = list(printer.reactor.timers)
        mgr._handle_ready()
        assert mgr._timer is None
        assert printer.reactor.timers == before

    def test_the_updater_starts_the_reads_at_once(self, tmp_path):
        printer = make_printer()
        mgr = AmsFirmware(printer)
        add_updater(printer, tmp_path)
        before = list(printer.reactor.timers)
        mgr._handle_ready()
        assert printer.reactor.timers == before + [mgr._timer]
        assert mgr._timer.callback == mgr._tick
        assert mgr._timer.waketime == FakeReactor.NOW


class TestAmsFirmwareUnits:
    def test_no_units(self):
        assert AmsFirmware(make_printer())._units() == []

    def test_every_unit_in_config_order(self):
        printer = make_printer()
        first = make_bambu_unit("Bambu_AMS_1", printer=printer)
        second = make_bambu_unit("Bambu_AMS_HT_1", printer=printer, model="ht", ams_index=4)
        assert manager_of(printer)._units() == [first, second]


class TestAmsFirmwarePrinting:
    @pytest.mark.parametrize("state, printing", [("printing", True), ("paused", True),
                                                 ("standby", False), ("complete", False)])
    def test_the_print_state(self, state, printing):
        printer = make_printer(print_state=state)
        assert AmsFirmware(printer)._printing() is printing

    def test_no_print_stats_is_no_print(self):
        printer = make_printer(print_state="printing")
        mgr = AmsFirmware(printer)
        del printer._objects["print_stats"]
        assert mgr._printing() is False


class TestAmsFirmwareTick:
    @staticmethod
    def _setup(state: str = "standby") -> Tuple[BambuPrinter, afcBambuAMS, FakeBridge,
                                                AmsFirmware]:
        printer = make_printer(print_state=state)
        bridge = FakeBridge(online=[True])
        unit = make_bambu_unit(printer=printer, bridge=bridge)
        return printer, unit, bridge, manager_of(printer)

    def test_an_unread_unit_is_asked(self):
        printer, unit, bridge, mgr = self._setup()
        assert mgr._tick(100.0) == 100.5
        assert bridge.sent == [query_sent(BOXED_LE, 0, 1)]
        assert mgr._asking == (unit, bridge, 0, 1)
        assert (mgr._read, printer.afc.logger.messages) == ({}, [])

    def test_its_answer_is_taken_on_the_next_tick(self):
        printer, unit, bridge, mgr = self._setup()
        mgr._tick(100.0)
        bridge._last_raw_reply = answer_frame(0, (4, 3, 2, 1), b"N3F05").hex()
        assert mgr._tick(100.5) == 102.5
        assert (unit.fw_version, unit.fw_model) == ("01.02.03.04", "N3F05")
        assert (mgr._read, mgr._asking) == ({"Bambu_AMS_1": (None, 0)}, None)
        assert bridge.sent == [query_sent(BOXED_LE, 0, 1)]
        assert printer.afc.logger.messages == [
            ("debug", "Bambu_AMS_1: AMS 2 Pro (N3F05) firmware 01.02.03.04")]

    def test_a_unit_that_stays_silent_is_asked_again(self):
        printer, unit, bridge, mgr = self._setup()
        mgr._tick(100.0)
        assert mgr._tick(100.5) == 101.0
        assert bridge.sent == [query_sent(BOXED_LE, 0, 1), query_sent(BOXED_LE, 0, 2)]
        assert mgr._asking == (unit, bridge, 0, 2)
        assert (mgr._read, unit.fw_version, printer.afc.logger.messages) == ({}, None, [])

    def test_an_answer_owed_is_taken_even_while_printing(self):
        printer, unit, bridge, mgr = self._setup()
        mgr._tick(100.0)
        printer.set_print_state("printing")
        bridge._last_raw_reply = answer_frame(0, (4, 3, 2, 1), b"N3F05").hex()
        assert mgr._tick(100.5) == 102.5
        assert (unit.fw_version, unit.fw_model) == ("01.02.03.04", "N3F05")
        assert (mgr._read, mgr._asking) == ({"Bambu_AMS_1": (None, 0)}, None)
        assert printer.afc.logger.messages == [
            ("debug", "Bambu_AMS_1: AMS 2 Pro (N3F05) firmware 01.02.03.04")]

    def test_nothing_is_asked_while_printing(self):
        printer, unit, bridge, mgr = self._setup("printing")
        assert mgr._tick(100.0) == 102.0
        assert (bridge.sent, mgr._asking, mgr._read) == ([], None, {})
        assert printer.afc.logger.messages == []

    def test_with_every_unit_read_it_waits_the_long_tick(self):
        printer, unit, bridge, mgr = self._setup()
        mgr._read["Bambu_AMS_1"] = (None, 0)
        assert mgr._tick(100.0) == 102.0
        assert (bridge.sent, mgr._asking) == ([], None)
        assert mgr._read == {"Bambu_AMS_1": (None, 0)}
        assert printer.afc.logger.messages == []


class TestAmsFirmwareNextUnread:
    @staticmethod
    def _setup(uid: Optional[str] = None,
               ams_index: int = 0) -> Tuple[afcBambuAMS, FakeBridge, AmsFirmware]:
        printer = make_printer()
        bridge = FakeBridge(online=[True, True])
        unit = make_bambu_unit(printer=printer, ams_index=ams_index, bridge=bridge)
        unit.unit_uid = uid
        return unit, bridge, manager_of(printer)

    def test_no_units_leave_nothing_to_read(self):
        assert AmsFirmware(make_printer())._next_unread() is None

    def test_an_unread_online_unit_is_next(self):
        unit, bridge, mgr = self._setup()
        assert mgr._next_unread() == (unit, bridge, 0)
        assert mgr._read == {}

    def test_a_unit_read_at_its_uid_and_bus_id_is_done(self):
        # Bus id 1, so a check that ignored the bus id would ask this unit again.
        unit, bridge, mgr = self._setup("uid-a", ams_index=1)
        mgr._read["Bambu_AMS_1"] = ("uid-a", 1)
        assert mgr._next_unread() is None
        assert mgr._read == {"Bambu_AMS_1": ("uid-a", 1)}

    def test_another_uid_on_the_unit_is_read_again(self):
        unit, bridge, mgr = self._setup("uid-b")
        mgr._read["Bambu_AMS_1"] = ("uid-a", 0)
        assert mgr._next_unread() == (unit, bridge, 0)
        assert mgr._read == {"Bambu_AMS_1": ("uid-a", 0)}

    def test_another_bus_id_is_read_again(self):
        unit, bridge, mgr = self._setup()
        mgr._read["Bambu_AMS_1"] = (None, 1)
        assert mgr._next_unread() == (unit, bridge, 0)
        assert mgr._read == {"Bambu_AMS_1": (None, 1)}

    def test_an_offline_unit_forgets_its_read_and_the_next_is_asked(self):
        printer = make_printer()
        bridge = FakeBridge(online=[False, True])
        make_bambu_unit("Bambu_AMS_1", printer=printer, bridge=bridge)
        second = make_bambu_unit("Bambu_AMS_2", printer=printer, ams_index=1, bridge=bridge)
        mgr = manager_of(printer)
        mgr._read["Bambu_AMS_1"] = (None, 0)
        assert mgr._next_unread() == (second, bridge, 1)
        assert mgr._read == {}

    def test_an_offline_unit_alone_leaves_nothing_to_read(self):
        printer = make_printer()
        make_bambu_unit(printer=printer, bridge=FakeBridge(online=[False]))
        mgr = manager_of(printer)
        mgr._read["Bambu_AMS_1"] = (None, 0)
        assert mgr._next_unread() is None
        assert mgr._read == {}


class TestAmsFirmwareAsk:
    def test_sends_the_query_counted_by_the_try(self):
        printer = make_printer()
        bridge = FakeBridge()
        unit = make_bambu_unit(printer=printer, bridge=bridge)
        mgr = manager_of(printer)
        bridge._last_raw_reply = "aa55"
        lock = bridge._lock = WatchedLock(bridge)
        mgr._ask(unit, bridge, 2, 3)
        # The earlier capture is dropped under the lock, before the query goes out.
        assert lock.seen == [("taken", "aa55"), ("let go", "")]
        assert bridge._last_raw_reply == ""
        assert bridge.sent == [query_sent(BOXED_LE, 2, 3)]
        assert mgr._asking == (unit, bridge, 2, 3)
        assert printer.afc.logger.messages == []

    def test_an_ht_unit_is_asked_at_its_address(self):
        printer = make_printer()
        bridge = FakeBridge()
        unit = make_bambu_unit("Bambu_AMS_HT_1", printer=printer, model="ht", ams_index=4,
                               bridge=bridge)
        mgr = manager_of(printer)
        mgr._ask(unit, bridge, 0x80, 1)
        assert bridge.sent == [query_sent(HT_LE, 0x80, 1)]
        assert mgr._asking == (unit, bridge, 0x80, 1)


class TestAmsFirmwareTakeAnswer:
    @staticmethod
    def _setup(monkeypatch: pytest.MonkeyPatch, reply: str = "", name: str = "Bambu_AMS_1",
               **unit_kw: Any) -> Tuple[BambuPrinter, afcBambuAMS, FakeBridge, AmsFirmware,
                                        Recorder]:
        printer = make_printer()
        bridge = FakeBridge()
        unit = make_bambu_unit(name, printer=printer, bridge=bridge, **unit_kw)
        mgr = manager_of(printer)
        offers = Recorder()
        monkeypatch.setattr(mgr, "_offer", offers)
        bridge._last_raw_reply = reply
        return printer, unit, bridge, mgr, offers

    def test_no_answer_on_the_first_try_asks_again(self, monkeypatch):
        # Unit 1's answer is not unit 0's.
        other = answer_frame(1, (4, 3, 2, 1), b"N3F05").hex()
        printer, unit, bridge, mgr, offers = self._setup(monkeypatch, other)
        mgr._take_answer(unit, bridge, 0, 1)
        assert bridge.sent == [query_sent(BOXED_LE, 0, 2)]
        assert bridge._last_raw_reply == ""
        assert mgr._asking == (unit, bridge, 0, 2)
        assert (mgr._read, unit.fw_version, unit.fw_model) == ({}, None, None)
        assert (offers.calls, printer.afc.logger.messages) == ([], [])

    def test_no_answer_on_the_second_try_asks_a_third_time(self, monkeypatch):
        printer, unit, bridge, mgr, offers = self._setup(monkeypatch)
        mgr._take_answer(unit, bridge, 0, 2)
        assert bridge.sent == [query_sent(BOXED_LE, 0, 3)]
        assert mgr._asking == (unit, bridge, 0, 3)
        assert (mgr._read, unit.fw_version, unit.fw_model) == ({}, None, None)
        assert (offers.calls, printer.afc.logger.messages) == ([], [])

    def test_no_answer_on_the_last_try_gives_up(self, monkeypatch):
        printer, unit, bridge, mgr, offers = self._setup(monkeypatch)
        unit.unit_uid = "uid-a"
        mgr._take_answer(unit, bridge, 0, 3)
        assert (bridge.sent, mgr._asking) == ([], None)
        assert mgr._read == {"Bambu_AMS_1": ("uid-a", 0)}
        assert (unit.fw_version, unit.fw_model) == (None, None)
        assert offers.calls == []
        assert printer.afc.logger.messages == [
            ("debug", "Bambu_AMS_1: no answer to the firmware version query")]

    def test_an_answer_on_the_first_try_is_taken(self, monkeypatch):
        reply = answer_frame(0, (4, 3, 2, 1), b"N3F05").hex()
        printer, unit, bridge, mgr, offers = self._setup(monkeypatch, reply)
        mgr._take_answer(unit, bridge, 0, 1)
        assert (bridge.sent, mgr._asking) == ([], None)
        assert mgr._read == {"Bambu_AMS_1": (None, 0)}
        assert (unit.fw_version, unit.fw_model) == ("01.02.03.04", "N3F05")
        assert offers.calls == [((unit,), {})]
        assert printer.afc.logger.messages == [
            ("debug", "Bambu_AMS_1: AMS 2 Pro (N3F05) firmware 01.02.03.04")]

    def test_a_code_of_no_known_model_is_logged_as_unknown(self, monkeypatch):
        reply = answer_frame(0, (4, 3, 2, 1), b"XYZ01").hex()
        printer, unit, bridge, mgr, offers = self._setup(monkeypatch, reply)
        mgr._take_answer(unit, bridge, 0, 1)
        assert (unit.fw_version, unit.fw_model) == ("01.02.03.04", "XYZ01")
        assert offers.calls == [((unit,), {})]
        assert printer.afc.logger.messages == [
            ("debug", "Bambu_AMS_1: unknown model (XYZ01) firmware 01.02.03.04")]

    def test_an_answer_on_the_last_try_is_taken(self, monkeypatch):
        reply = answer_frame(0x80, (31, 0, 10, 2), b"N3S05", dev=HT_LE).hex()
        printer, unit, bridge, mgr, offers = self._setup(monkeypatch, reply, "Bambu_AMS_HT_1",
                                                         model="ht", ams_index=4)
        mgr._take_answer(unit, bridge, 0x80, 3)
        assert (bridge.sent, mgr._asking) == ([], None)
        assert mgr._read == {"Bambu_AMS_HT_1": (None, 0x80)}
        assert (unit.fw_version, unit.fw_model) == ("02.10.00.31", "N3S05")
        assert offers.calls == [((unit,), {})]
        assert printer.afc.logger.messages == [
            ("debug", "Bambu_AMS_HT_1: AMS HT (N3S05) firmware 02.10.00.31")]


class TestAmsFirmwareOffer:
    #: What the chain is told for a unit on 01.02.03.04 with 01.03.00.00 out.
    NEWER = {"version": "01.03.00.00", "file": "", "newest": True}

    @staticmethod
    def _setup(tmp_path: pathlib.Path, images: Sequence[Dict[str, str]] = (),
               master: bool = True, updater: bool = True, have: str = "01.02.03.04",
               code: Optional[str] = "N3F05") -> Tuple[afcBambuAMS, FakeMaster, AmsFirmware]:
        printer = make_printer()
        unit = make_bambu_unit(printer=printer)
        unit.fw_version, unit.fw_model = have, code
        chain = FakeMaster()
        if master:
            unit.set_master(chain)
        if updater:
            add_updater(printer, tmp_path)
            write_catalog(tmp_path, list(images))
        return unit, chain, manager_of(printer)

    def test_an_older_unit_is_offered_the_newest_image(self, tmp_path):
        unit, chain, mgr = self._setup(tmp_path, [catalog_entry("n3f_rev5", "01.02.00.00"),
                                                  catalog_entry("n3f_rev5", "01.03.00.00")])
        mgr._offer(unit)
        assert chain.offers == [(unit, self.NEWER)]

    def test_a_unit_on_the_newest_image_is_reported_with_none(self, tmp_path):
        unit, chain, mgr = self._setup(tmp_path, [catalog_entry("n3f_rev5", "01.03.00.00")],
                                       have="01.03.00.00")
        mgr._offer(unit)
        assert chain.offers == [(unit, None)]

    def test_a_unit_newer_than_every_image_is_reported_with_none(self, tmp_path):
        unit, chain, mgr = self._setup(tmp_path, [catalog_entry("n3f_rev5", "01.03.00.00")],
                                       have="01.04.00.00")
        mgr._offer(unit)
        assert chain.offers == [(unit, None)]

    def test_a_unit_with_no_image_for_its_board_is_reported_with_none(self, tmp_path):
        unit, chain, mgr = self._setup(tmp_path, [catalog_entry("n3s_rev5", "01.03.00.00")])
        mgr._offer(unit)
        assert chain.offers == [(unit, None)]

    def test_a_unit_no_chain_built_tells_nothing(self, monkeypatch, tmp_path):
        unit, chain, mgr = self._setup(tmp_path, master=False)
        looked = Recorder(result=[self.NEWER])
        monkeypatch.setattr(fw_mod, "choices_for", looked)
        mgr._offer(unit)
        assert (looked.calls, chain.offers) == ([], [])

    def test_without_the_updater_the_chain_is_told_nothing(self, monkeypatch, tmp_path):
        unit, chain, mgr = self._setup(tmp_path, updater=False)
        looked = Recorder(result=[self.NEWER])
        monkeypatch.setattr(fw_mod, "choices_for", looked)
        mgr._offer(unit)
        assert (looked.calls, chain.offers) == ([], [])

    @pytest.mark.parametrize("code", [None, "XYZ01"])
    def test_an_unknown_board_tells_the_chain_nothing(self, monkeypatch, tmp_path, code):
        unit, chain, mgr = self._setup(tmp_path, code=code)
        looked = Recorder(result=[self.NEWER])
        monkeypatch.setattr(fw_mod, "choices_for", looked)
        mgr._offer(unit)
        assert (looked.calls, chain.offers) == ([], [])


class TestAmsFirmwareListed:
    def test_no_units_list_nothing(self):
        assert AmsFirmware(make_printer())._listed() == []

    @pytest.mark.parametrize("model", ["ams1", "ams2", "ht", "boxed"])
    def test_a_unit_outside_a_pool_needs_no_uid(self, model):
        printer = make_printer()
        unit = make_bambu_unit(printer=printer, model=model)
        assert (unit.pool, unit.unit_uid) == (False, None)
        assert manager_of(printer)._listed() == [unit]

    def test_an_ams_lite_is_left_out(self):
        printer = make_printer()
        first = make_bambu_unit("Bambu_AMS_1", printer=printer)
        lite = make_bambu_unit("Bambu_AMS_Lite_1", printer=printer, model="lite",
                               values={"serial_port": "/dev/ttyLITE0"})
        assert (lite.pool, lite.unit_uid) == (False, None)
        assert manager_of(printer)._listed() == [first]

    def test_an_unclaimed_spare_is_left_out(self):
        printer = make_printer()
        first = make_bambu_unit("Bambu_AMS_1", printer=printer)
        spare = make_bambu_unit("Bambu_AMS_2", printer=printer, values={"pool": True})
        assert (spare.ams_model, spare.pool, spare.unit_uid) == ("ams2", True, None)
        assert manager_of(printer)._listed() == [first]

    def test_a_spare_holding_a_uid_is_listed(self):
        printer = make_printer()
        first = make_bambu_unit("Bambu_AMS_1", printer=printer)
        spare = make_bambu_unit("Bambu_AMS_2", printer=printer, values={"pool": True})
        spare.unit_uid = "uid-a"
        assert manager_of(printer)._listed() == [first, spare]


class TestAmsFirmwareUnit:
    @staticmethod
    def _setup() -> Tuple[afcBambuAMS, AmsFirmware]:
        printer = make_printer()
        make_bambu_unit("Bambu_AMS_1", printer=printer)
        second = make_bambu_unit("Bambu_AMS_2", printer=printer, ams_index=1)
        return second, manager_of(printer)

    def test_a_unit_by_its_name(self):
        second, mgr = self._setup()
        assert mgr._unit("Bambu_AMS_2") is second

    def test_a_unit_by_its_name_in_any_case(self):
        second, mgr = self._setup()
        assert mgr._unit("bAMBU_ams_2") is second

    def test_an_unknown_name_is_no_unit(self):
        _second, mgr = self._setup()
        assert mgr._unit("Bambu_AMS_9") is None


class TestAmsFirmwareBoard:
    @staticmethod
    def _board(model: str, code: Optional[str]) -> str:
        printer = make_printer()
        unit = make_bambu_unit(printer=printer, model=model)
        unit.fw_model = code
        return manager_of(printer)._board(unit)

    def test_the_model_code_names_the_board(self):
        # An AMS 1 code on a section configured as an AMS 2 Pro.
        assert self._board("ams2", "AMS08") == "ams_rev8"

    @pytest.mark.parametrize("model, board", [("ams2", "n3f_rev5"), ("ht", "n3s_rev5")])
    def test_an_unread_unit_takes_its_models_board(self, model, board):
        assert self._board(model, None) == board

    def test_an_unknown_code_falls_back_to_the_models_board(self):
        assert self._board("ht", "XYZ01") == "n3s_rev5"

    @pytest.mark.parametrize("model", ["ams1", "boxed"])
    def test_a_model_without_one_board_is_unknown(self, model):
        assert self._board(model, None) == ""


class TestAmsFirmwareCmdAFCBambuFirmware:
    PAGES = ("_show_result", "_show_units", "_hold", "_run", "_show_versions", "_show_confirm")
    BAD_VERSION = ("AFC_BAMBU_FIRMWARE: VERSION is like 05.00.22.22 and FILE is a file name "
                   "beside the updater")

    @classmethod
    def _setup(cls, monkeypatch: pytest.MonkeyPatch, tmp_path: Optional[pathlib.Path] = None
               ) -> Tuple[BambuPrinter, afcBambuAMS, AmsFirmware, Optional[str],
                          Dict[str, Recorder]]:
        printer = make_printer()
        unit = make_bambu_unit(printer=printer)
        mgr = manager_of(printer)
        folder = add_updater(printer, tmp_path) if tmp_path is not None else None
        pages = {name: Recorder() for name in cls.PAGES}
        for name, page in pages.items():
            monkeypatch.setattr(mgr, name, page)
        return printer, unit, mgr, folder, pages

    @staticmethod
    def _gcmd(**params: str) -> FakeGcmd:
        line = " ".join(["AFC_BAMBU_FIRMWARE"] + [f"{k}={v}" for k, v in params.items()])
        return FakeGcmd(params, command="AFC_BAMBU_FIRMWARE", commandline=line)

    @staticmethod
    def _called(pages: Dict[str, Recorder]) -> Dict[str, List[Tuple[tuple, dict]]]:
        return {name: page.calls for name, page in pages.items() if page.calls}

    def _refused(self, monkeypatch: pytest.MonkeyPatch, tmp_path: Optional[pathlib.Path],
                 **params: str) -> str:
        """:return str: the error the command raised, once nothing else came of it"""
        printer, _unit, mgr, _folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(**params)
        with pytest.raises(BambuPrinter.command_error) as exc:
            mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {}
        assert (gcmd.messages, printer.afc.logger.messages) == ([], [])
        return str(exc.value)

    def test_show_result_in_any_case_shows_the_result(self, monkeypatch, tmp_path):
        _p, _unit, mgr, folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(SHOW="result", UNIT="Bambu_AMS_1")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_show_result": [((gcmd, folder), {})]}

    def test_show_result_needs_no_updater(self, monkeypatch):
        _p, _unit, mgr, _folder, pages = self._setup(monkeypatch)
        gcmd = self._gcmd(SHOW="RESULT")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_show_result": [((gcmd, None), {})]}

    def test_another_show_is_the_unit_list(self, monkeypatch, tmp_path):
        _p, _unit, mgr, folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(SHOW="UNITS")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_show_units": [((gcmd, folder), {})]}

    def test_no_unit_is_the_unit_list_even_without_the_updater(self, monkeypatch):
        _p, _unit, mgr, _folder, pages = self._setup(monkeypatch)
        gcmd = self._gcmd()
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_show_units": [((gcmd, None), {})]}

    def test_an_unknown_unit_is_refused(self, monkeypatch, tmp_path):
        assert self._refused(monkeypatch, tmp_path, UNIT="Bambu_AMS_9") == \
            "AFC_BAMBU_FIRMWARE: there is no Bambu AMS unit Bambu_AMS_9"

    def test_hold_lends_the_link_even_without_the_updater(self, monkeypatch):
        _p, unit, mgr, _folder, pages = self._setup(monkeypatch)
        gcmd = self._gcmd(UNIT="bambu_ams_1", HOLD="600")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_hold": [((gcmd, unit, 600), {})]}

    @pytest.mark.parametrize("seconds", [0, 3600])
    def test_hold_takes_none_up_to_an_hour(self, monkeypatch, tmp_path, seconds):
        _p, unit, mgr, _folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(UNIT="Bambu_AMS_1", HOLD=str(seconds), VERSION="01.03.00.00")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_hold": [((gcmd, unit, seconds), {})]}

    @pytest.mark.parametrize("seconds, error", [
        ("3601", "HOLD must have maximum of 3600"), ("-1", "HOLD must have minimum of 0")])
    def test_a_hold_out_of_range_is_refused(self, monkeypatch, tmp_path, seconds, error):
        assert self._refused(monkeypatch, tmp_path, UNIT="Bambu_AMS_1", HOLD=seconds) == \
            f"Error on 'AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 HOLD={seconds}': {error}"

    def test_without_the_updater_a_unit_is_refused(self, monkeypatch):
        assert self._refused(monkeypatch, None, UNIT="Bambu_AMS_1") == \
            "AFC_BAMBU_FIRMWARE: the AMS updater is not set up; AMS_FLASH_HOWTO.md says how"

    def test_resume_finishes_a_stopped_update(self, monkeypatch, tmp_path):
        _p, unit, mgr, folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(UNIT="Bambu_AMS_1", RESUME="1", VERSION="01.03.00.00", FILE=PACK)
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_run": [((gcmd, folder, unit, None, None), {})]}

    def test_resume_zero_is_no_resume(self, monkeypatch, tmp_path):
        _p, unit, mgr, folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(UNIT="Bambu_AMS_1", RESUME="0")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_show_versions": [((gcmd, folder, unit), {})]}

    def test_no_version_lists_the_versions(self, monkeypatch, tmp_path):
        _p, unit, mgr, folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(UNIT="Bambu_AMS_1")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {"_show_versions": [((gcmd, folder, unit), {})]}

    @pytest.mark.parametrize("version", ["1.3.0.0", "01.03.00", "01.03.00.00 "])
    def test_a_malformed_version_is_refused(self, monkeypatch, tmp_path, version):
        assert self._refused(monkeypatch, tmp_path, UNIT="Bambu_AMS_1",
                             VERSION=version) == self.BAD_VERSION

    @pytest.mark.parametrize("file", ["../pack-a.zip", "sub/pack-a.zip", "pack a.zip", ""])
    def test_a_file_that_is_not_a_bare_name_is_refused(self, monkeypatch, tmp_path, file):
        assert self._refused(monkeypatch, tmp_path, UNIT="Bambu_AMS_1", VERSION="01.03.00.00",
                             FILE=file) == self.BAD_VERSION

    def test_a_version_to_download_asks_to_confirm(self, monkeypatch, tmp_path):
        _p, unit, mgr, _folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(UNIT="Bambu_AMS_1", VERSION="01.03.00.00")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {
            "_show_confirm": [((gcmd, unit, "01.03.00.00", None), {})]}

    @pytest.mark.parametrize("confirm", [None, "0"])
    def test_a_version_in_a_file_asks_to_confirm(self, monkeypatch, tmp_path, confirm):
        _p, unit, mgr, _folder, pages = self._setup(monkeypatch, tmp_path)
        params = {"UNIT": "Bambu_AMS_1", "VERSION": "01.03.00.00", "FILE": PACK}
        if confirm is not None:
            params["CONFIRM"] = confirm
        gcmd = self._gcmd(**params)
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {
            "_show_confirm": [((gcmd, unit, "01.03.00.00", PACK), {})]}

    def test_confirm_runs_the_update(self, monkeypatch, tmp_path):
        _p, unit, mgr, folder, pages = self._setup(monkeypatch, tmp_path)
        gcmd = self._gcmd(UNIT="Bambu_AMS_1", VERSION="01.03.00.00", FILE=PACK, CONFIRM="1")
        mgr.cmd_AFC_BAMBU_FIRMWARE(gcmd)
        assert self._called(pages) == {
            "_run": [((gcmd, folder, unit, "01.03.00.00", PACK), {})]}


class TestAmsFirmwareHold:
    BACK = "Bambu_AMS_1: Klipper has its bridge link back"
    LENT = ("Bambu_AMS_1: the AMS updater has its bridge link; the units on it read offline "
            "until it is done")

    @staticmethod
    def _setup(monkeypatch: pytest.MonkeyPatch, state: str = "standby"
               ) -> Tuple[BambuPrinter, afcBambuAMS, FakeBridge, AmsFirmware]:
        # The bridge table is the test's own, with the unit's bridge at its port.
        printer = make_printer(print_state=state, monkeypatch=monkeypatch)
        bridge = FakeBridge()
        unit = make_bambu_unit(printer=printer, bridge=bridge)
        return printer, unit, bridge, manager_of(printer)

    def test_a_unit_with_no_bridge_on_its_port_is_refused(self, monkeypatch):
        printer, unit, bridge, mgr = self._setup(monkeypatch)
        use_bridges(monkeypatch, {})
        gcmd = FakeGcmd()
        with pytest.raises(BambuPrinter.command_error) as exc:
            mgr._hold(gcmd, unit, 600)
        assert str(exc.value) == "AFC_BAMBU_FIRMWARE: Bambu_AMS_1 has no bridge"
        assert (bridge.holds, bridge.held_now) == ([], False)
        assert (gcmd.messages, printer.afc.logger.messages) == ([], [])

    def test_the_link_is_not_lent_during_a_print(self, monkeypatch):
        printer, unit, bridge, mgr = self._setup(monkeypatch, "printing")
        gcmd = FakeGcmd()
        with pytest.raises(BambuPrinter.command_error) as exc:
            mgr._hold(gcmd, unit, 600)
        assert str(exc.value) == ("AFC_BAMBU_FIRMWARE: a print is running, so the bridge link "
                                  "stays")
        assert (bridge.holds, bridge.held_now) == ([], False)
        assert (gcmd.messages, printer.afc.logger.messages) == ([], [])

    def test_the_link_is_lent_when_no_print_runs(self, monkeypatch):
        printer, unit, bridge, mgr = self._setup(monkeypatch)
        gcmd = FakeGcmd()
        mgr._hold(gcmd, unit, 600)
        assert (bridge.holds, bridge.held_now) == ([600], True)
        assert gcmd.messages == [("respond_info", self.LENT)]
        assert printer.afc.logger.messages == []

    def test_the_link_is_taken_back_even_during_a_print(self, monkeypatch):
        printer, unit, bridge, mgr = self._setup(monkeypatch, "printing")
        bridge.held_now = True
        gcmd = FakeGcmd()
        mgr._hold(gcmd, unit, 0)
        assert (bridge.holds, bridge.held_now) == ([0], False)
        assert gcmd.messages == [("respond_info", self.BACK)]
        assert printer.afc.logger.messages == []


class TestAmsFirmwareStatusLine:
    @staticmethod
    def _line(tmp_path: Optional[pathlib.Path], have: Optional[str], code: Optional[str],
              images: Sequence[Dict[str, str]] = (), model: str = "ams2",
              name: str = "Bambu_AMS_1") -> str:
        printer = make_printer()
        unit = make_bambu_unit(name, printer=printer, model=model)
        unit.fw_version, unit.fw_model = have, code
        folder = None
        if tmp_path is not None:
            folder = add_updater(printer, tmp_path)
            write_catalog(tmp_path, list(images))
        return manager_of(printer)._status_line(folder, unit)

    def test_an_unread_unit(self, tmp_path):
        images = [catalog_entry("n3f_rev5", "01.03.00.00")]
        assert self._line(tmp_path, None, None, images) == \
            "Bambu_AMS_1: firmware not read yet; it is read while the unit is online"

    def test_without_the_updater_there_is_nothing_to_compare(self):
        assert self._line(None, "01.02.03.04", "N3F05") == \
            "Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04"

    def test_a_newer_image_is_named(self, tmp_path):
        images = [catalog_entry("n3f_rev5", "01.02.03.04"),
                  catalog_entry("n3f_rev5", "01.03.00.00")]
        assert self._line(tmp_path, "01.02.03.04", "N3F05", images) == \
            "Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04, 01.03.00.00 is out"

    @pytest.mark.parametrize("have", ["01.03.00.00", "01.04.00.00"])
    def test_a_unit_on_the_newest_image_says_so(self, tmp_path, have):
        images = [catalog_entry("n3f_rev5", "01.03.00.00")]
        assert self._line(tmp_path, have, "N3F05", images) == \
            f"Bambu_AMS_1: AMS 2 Pro, firmware {have}, the newest"

    def test_no_image_for_its_board_says_nothing_more(self, tmp_path):
        images = [catalog_entry("n3f_rev5", "01.03.00.00")]
        assert self._line(tmp_path, "02.10.00.31", "N3S05", images, model="ht",
                          name="Bambu_AMS_HT_1") == "Bambu_AMS_HT_1: AMS HT, firmware 02.10.00.31"

    def test_a_code_of_no_known_model_reads_as_an_ams(self, tmp_path):
        # As on the versions page; an AMS 1 has no one board to fall back on.
        assert self._line(tmp_path, "01.02.03.04", "XYZ01", model="ams1") == \
            "Bambu_AMS_1: AMS, firmware 01.02.03.04"

    def test_a_code_of_no_known_model_takes_the_board_its_model_shipped_on(self, tmp_path):
        images = [catalog_entry("n3f_rev5", "01.03.00.00")]
        assert self._line(tmp_path, "01.02.03.04", "XYZ01", images) == \
            "Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04, 01.03.00.00 is out"


class TestAmsFirmwareShowUnits:
    HOWTO = "Set up the AMS updater to flash units; AMS_FLASH_HOWTO.md says how."
    UNREAD = "firmware not read yet; it is read while the unit is online"

    @staticmethod
    def _two_units(printer: BambuPrinter) -> Tuple[afcBambuAMS, afcBambuAMS]:
        """Bambu_AMS_1 read on 01.02.03.04; Bambu_AMS_2 not read yet."""
        read = make_bambu_unit("Bambu_AMS_1", printer=printer)
        read.fw_version, read.fw_model = "01.02.03.04", "N3F05"
        return read, make_bambu_unit("Bambu_AMS_2", printer=printer, ams_index=1)

    @staticmethod
    def _unit_button(name: str) -> LogLine:
        return button(name, f"AFC_BAMBU_FIRMWARE UNIT={name}", "primary")

    def test_no_units_and_no_updater(self):
        printer = make_printer()
        AmsFirmware(printer)._show_units(FakeGcmd(), None)
        assert printer.afc.logger.messages == [
            begin("Flash AMS"), text("No Bambu AMS units."), text(self.HOWTO), CANCEL, SHOW]

    def test_without_the_updater_units_are_listed_without_buttons(self):
        printer = make_printer()
        self._two_units(printer)
        manager_of(printer)._show_units(FakeGcmd(), None)
        assert printer.afc.logger.messages == [
            begin("Flash AMS"), text("Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04"),
            text(f"Bambu_AMS_2: {self.UNREAD}"), text(self.HOWTO), CANCEL, SHOW]

    def test_with_the_updater_each_unit_has_a_button(self, tmp_path):
        printer = make_printer()
        self._two_units(printer)
        folder = add_updater(printer, tmp_path)
        write_catalog(tmp_path, [catalog_entry("n3f_rev5", "01.03.00.00")])
        manager_of(printer)._show_units(FakeGcmd(), folder)
        assert printer.afc.logger.messages == [
            begin("Flash AMS"),
            text("Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04, 01.03.00.00 is out"),
            text(f"Bambu_AMS_2: {self.UNREAD}"),
            self._unit_button("Bambu_AMS_1"), self._unit_button("Bambu_AMS_2"), CANCEL, SHOW]

    def test_a_unit_left_in_its_bootloader_is_finished_first(self, tmp_path):
        printer = make_printer()
        self._two_units(printer)
        folder = add_updater(printer, tmp_path)
        write_json(tmp_path / STATE, {"unit": "bambu_ams_2"})
        manager_of(printer)._show_units(FakeGcmd(), folder)
        assert printer.afc.logger.messages == [
            begin("Flash AMS"), text("Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04"),
            text(f"Bambu_AMS_2: {self.UNREAD}"),
            text("Bambu_AMS_2 waits in its bootloader from an update that stopped."),
            button("Finish Bambu_AMS_2", "AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_2 RESUME=1", "error"),
            self._unit_button("Bambu_AMS_1"), self._unit_button("Bambu_AMS_2"), CANCEL, SHOW]

    def test_a_running_update_is_not_offered_as_stopped(self, tmp_path):
        printer = make_printer()
        self._two_units(printer)
        folder = add_updater(printer, tmp_path)
        write_json(tmp_path / STATE, {"unit": "Bambu_AMS_2"})
        write_json(tmp_path / RESULT, {"running": True, "unit": "Bambu_AMS_2"})
        manager_of(printer)._show_units(FakeGcmd(), folder)
        assert printer.afc.logger.messages == [
            begin("Flash AMS"), text("Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04"),
            text(f"Bambu_AMS_2: {self.UNREAD}"),
            text("An AMS update is running; let it finish."),
            self._unit_button("Bambu_AMS_1"), self._unit_button("Bambu_AMS_2"), CANCEL, SHOW]

    @pytest.mark.parametrize("state", [None, {}, {"unit": "Bambu_AMS_9"}, {"unit": 2}])
    def test_a_state_naming_no_unit_adds_nothing(self, tmp_path, state):
        printer = make_printer()
        self._two_units(printer)
        folder = add_updater(printer, tmp_path)
        if state is not None:
            write_json(tmp_path / STATE, state)
        manager_of(printer)._show_units(FakeGcmd(), folder)
        assert printer.afc.logger.messages == [
            begin("Flash AMS"), text("Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04"),
            text(f"Bambu_AMS_2: {self.UNREAD}"),
            self._unit_button("Bambu_AMS_1"), self._unit_button("Bambu_AMS_2"), CANCEL, SHOW]

    def test_only_the_units_the_menu_lists_are_shown(self, monkeypatch, tmp_path):
        printer = make_printer()
        _read, unread = self._two_units(printer)
        mgr = manager_of(printer)
        folder = add_updater(printer, tmp_path)
        monkeypatch.setattr(mgr, "_listed", Recorder(result=[unread]))
        mgr._show_units(FakeGcmd(), folder)
        assert printer.afc.logger.messages == [
            begin("Flash AMS"), text(f"Bambu_AMS_2: {self.UNREAD}"),
            self._unit_button("Bambu_AMS_2"), CANCEL, SHOW]


class TestAmsFirmwareShowVersions:
    BACK = footer("Back", "AFC_BAMBU_FIRMWARE", "info")
    PICK = "Pick the firmware to write."

    @staticmethod
    def _setup(tmp_path: pathlib.Path, model: str = "ams2", have: Optional[str] = None,
               code: Optional[str] = None, name: str = "Bambu_AMS_1"
               ) -> Tuple[BambuPrinter, afcBambuAMS, AmsFirmware, str]:
        """A unit, and beside the updater 01.02.03.04 to download and 01.03.00.00 in a package."""
        printer = make_printer()
        unit = make_bambu_unit(name, printer=printer, model=model)
        unit.fw_version, unit.fw_model = have, code
        folder = add_updater(printer, tmp_path)
        write_catalog(tmp_path, [catalog_entry("n3f_rev5", "01.02.03.04")])
        write_zip(tmp_path / PACK, [image_name("n3f_rev5", "01.03.00.00")])
        return printer, unit, manager_of(printer), folder

    @staticmethod
    def _choice(label: str, version: str, style: str, file: str = "") -> LogLine:
        cmd = f"AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 VERSION={version}"
        return button(label, f"{cmd} FILE={file}" if file else cmd, style)

    def test_an_unread_unit_is_offered_its_models_images(self, tmp_path):
        printer, unit, mgr, folder = self._setup(tmp_path)
        mgr._show_versions(FakeGcmd(), folder, unit)
        assert printer.afc.logger.messages == [
            begin("Flash Bambu_AMS_1"), text("Bambu_AMS_1: AMS 2 Pro, firmware not read yet."),
            text(self.PICK),
            self._choice("01.03.00.00 (newest)", "01.03.00.00", "primary", PACK),
            self._choice("01.02.03.04", "01.02.03.04", "secondary"), self.BACK, CANCEL, SHOW]

    def test_the_installed_image_is_marked(self, tmp_path):
        printer, unit, mgr, folder = self._setup(tmp_path, have="01.02.03.04", code="N3F05")
        mgr._show_versions(FakeGcmd(), folder, unit)
        assert printer.afc.logger.messages == [
            begin("Flash Bambu_AMS_1"), text("Bambu_AMS_1: AMS 2 Pro, firmware 01.02.03.04."),
            text(self.PICK),
            self._choice("01.03.00.00 (newest)", "01.03.00.00", "primary", PACK),
            self._choice("01.02.03.04 (installed)", "01.02.03.04", "secondary"),
            self.BACK, CANCEL, SHOW]

    def test_the_newest_installed_carries_both_marks(self, tmp_path):
        printer, unit, mgr, folder = self._setup(tmp_path, have="01.03.00.00", code="N3F05")
        mgr._show_versions(FakeGcmd(), folder, unit)
        assert printer.afc.logger.messages == [
            begin("Flash Bambu_AMS_1"), text("Bambu_AMS_1: AMS 2 Pro, firmware 01.03.00.00."),
            text(self.PICK),
            self._choice("01.03.00.00 (newest) (installed)", "01.03.00.00", "primary", PACK),
            self._choice("01.02.03.04", "01.02.03.04", "secondary"), self.BACK, CANCEL, SHOW]

    def test_a_board_with_no_image_says_where_one_goes(self, tmp_path):
        printer, unit, mgr, folder = self._setup(tmp_path, model="ht", name="Bambu_AMS_HT_1")
        mgr._show_versions(FakeGcmd(), folder, unit)
        assert printer.afc.logger.messages == [
            begin("Flash Bambu_AMS_HT_1"), text("Bambu_AMS_HT_1: AMS HT, firmware not read yet."),
            text("No image for n3s_rev5: put Bambu's file beside the updater."),
            self.BACK, CANCEL, SHOW]

    def test_an_unknown_board_waits_for_the_read(self, monkeypatch, tmp_path):
        # An AMS 1 has shipped on two boards, so none is assumed.
        printer, unit, mgr, folder = self._setup(tmp_path, model="ams1")
        looked = Recorder(result=[{"version": "01.03.00.00", "file": "", "newest": True}])
        monkeypatch.setattr(fw_mod, "choices_for", looked)
        mgr._show_versions(FakeGcmd(), folder, unit)
        assert looked.calls == []
        assert printer.afc.logger.messages == [
            begin("Flash Bambu_AMS_1"), text("Bambu_AMS_1: AMS, firmware not read yet."),
            text("Its board is read with its version, while it is online."),
            self.BACK, CANCEL, SHOW]


class TestAmsFirmwareShowConfirm:
    TAKES = ("It takes a few minutes, and the units on its bridge read offline until it is done. "
             "Klipper keeps running.")
    POWER = "Keep the unit powered. If an update stops partway, this menu finishes it."
    BACK = footer("Back", "AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1", "info")

    @staticmethod
    def _setup(have: Optional[str]) -> Tuple[BambuPrinter, afcBambuAMS, AmsFirmware]:
        printer = make_printer()
        unit = make_bambu_unit(printer=printer)
        unit.fw_version = have
        return printer, unit, manager_of(printer)

    def test_an_image_in_a_file(self):
        printer, unit, mgr = self._setup("01.02.03.04")
        mgr._show_confirm(FakeGcmd(), unit, "01.03.00.00", PACK)
        assert printer.afc.logger.messages == [
            begin("Flash Bambu_AMS_1 to 01.03.00.00?"),
            text(f"Bambu_AMS_1 runs 01.02.03.04. This writes 01.03.00.00, from {PACK}."),
            text(self.TAKES), text(self.POWER),
            button("Flash 01.03.00.00", "AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 VERSION=01.03.00.00 "
                                        f"CONFIRM=1 FILE={PACK}", "primary"),
            self.BACK, CANCEL, SHOW]

    def test_an_image_to_download_on_an_unread_unit(self):
        printer, unit, mgr = self._setup(None)
        mgr._show_confirm(FakeGcmd(), unit, "01.03.00.00", None)
        assert printer.afc.logger.messages == [
            begin("Flash Bambu_AMS_1 to 01.03.00.00?"),
            text("Bambu_AMS_1 runs unknown firmware. This writes 01.03.00.00, downloaded from "
                 "Bambu."),
            text(self.TAKES), text(self.POWER),
            button("Flash 01.03.00.00", "AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 VERSION=01.03.00.00 "
                                        "CONFIRM=1", "primary"),
            self.BACK, CANCEL, SHOW]


class TestAmsFirmwareRun:
    SCRIPT = 'RUN_SHELL_COMMAND CMD=ams_flash PARAMS="--unit Bambu_AMS_1 --mode go{}"'

    @staticmethod
    def _setup(monkeypatch: pytest.MonkeyPatch, tmp_path: pathlib.Path,
               leaves: Optional[Dict[str, Any]] = None, state: str = "standby"
               ) -> Tuple[BambuPrinter, afcBambuAMS, AmsFirmware, str, FakeUpdater, Recorder]:
        printer = make_printer(print_state=state)
        unit = make_bambu_unit(printer=printer)
        mgr = manager_of(printer)
        folder = add_updater(printer, tmp_path)
        updater = FakeUpdater(folder, leaves)
        monkeypatch.setattr(printer.gcode, "run_script_from_command", updater)
        shown = Recorder()
        monkeypatch.setattr(mgr, "_show_result", shown)
        return printer, unit, mgr, folder, updater, shown

    def test_a_print_keeps_the_updater_from_starting(self, monkeypatch, tmp_path):
        printer, unit, mgr, folder, updater, shown = self._setup(monkeypatch, tmp_path,
                                                                 state="printing")
        write_json(tmp_path / RESULT, {"ok": True})
        gcmd = FakeGcmd()
        with pytest.raises(BambuPrinter.command_error) as exc:
            mgr._run(gcmd, folder, unit, "01.03.00.00", None)
        assert str(exc.value) == ("AFC_BAMBU_FIRMWARE: a print is running; update the AMS once "
                                  "it is done")
        assert (updater.runs, shown.calls) == ([], [])
        assert json.loads((tmp_path / RESULT).read_text()) == {"ok": True}
        assert (gcmd.messages, printer.afc.logger.messages) == ([], [])

    def test_a_running_update_keeps_another_from_starting(self, monkeypatch, tmp_path):
        printer, unit, mgr, folder, updater, shown = self._setup(monkeypatch, tmp_path)
        write_json(tmp_path / RESULT, {"running": True})
        gcmd = FakeGcmd()
        with pytest.raises(BambuPrinter.command_error) as exc:
            mgr._run(gcmd, folder, unit, "01.03.00.00", None)
        assert str(exc.value) == "AFC_BAMBU_FIRMWARE: an AMS update is running; let it finish"
        assert (updater.runs, shown.calls) == ([], [])
        assert json.loads((tmp_path / RESULT).read_text()) == {"running": True}
        assert (gcmd.messages, printer.afc.logger.messages) == ([], [])

    def test_an_image_in_a_file_starts_the_update(self, monkeypatch, tmp_path):
        printer, unit, mgr, folder, updater, shown = self._setup(monkeypatch, tmp_path,
                                                                 {"running": True})
        mgr._run(FakeGcmd(), folder, unit, "01.03.00.00", PACK)
        assert updater.runs == [
            (self.SCRIPT.format(f" --version 01.03.00.00 --file {PACK}"), False)]
        assert shown.calls == []
        assert printer.afc.logger.messages == [
            begin("Updating Bambu_AMS_1"),
            text("Bambu_AMS_1 is updating. This shows how it ended when it is done."), CLOSE, SHOW]

    def test_an_image_to_download_names_no_file(self, monkeypatch, tmp_path):
        printer, unit, mgr, folder, updater, shown = self._setup(monkeypatch, tmp_path,
                                                                 {"ok": True})
        gcmd = FakeGcmd()
        mgr._run(gcmd, folder, unit, "01.03.00.00", None)
        assert updater.runs == [(self.SCRIPT.format(" --version 01.03.00.00"), False)]
        assert shown.calls == [((gcmd, folder), {})]
        assert printer.afc.logger.messages == []

    def test_finishing_a_stopped_update_names_only_the_unit(self, monkeypatch, tmp_path):
        printer, unit, mgr, folder, updater, shown = self._setup(monkeypatch, tmp_path)
        gcmd = FakeGcmd()
        mgr._run(gcmd, folder, unit, None, None)
        assert updater.runs == [(self.SCRIPT.format(""), False)]
        assert not (tmp_path / RESULT).exists()
        assert shown.calls == [((gcmd, folder), {})]
        assert printer.afc.logger.messages == []

    def test_the_last_result_is_cleared_before_the_run(self, monkeypatch, tmp_path):
        # Written two hours ago by a run that never finished: no longer busy, and not this run's.
        printer, unit, mgr, folder, updater, shown = self._setup(monkeypatch, tmp_path)
        old = tmp_path / RESULT
        write_json(old, {"running": True, "message": "Bambu_AMS_1 is at 40%."})
        then = time.time() - 7200
        os.utime(old, (then, then))
        gcmd = FakeGcmd()
        mgr._run(gcmd, folder, unit, "01.03.00.00", None)
        assert updater.runs == [(self.SCRIPT.format(" --version 01.03.00.00"), False)]
        assert not old.exists()
        assert shown.calls == [((gcmd, folder), {})]
        assert printer.afc.logger.messages == []


class TestAmsFirmwareBusy:
    NOW = 2_000_000_000

    @classmethod
    def _result(cls, monkeypatch: pytest.MonkeyPatch, tmp_path: pathlib.Path,
                record: Dict[str, Any], age: int) -> str:
        """:return str: a result written `age` seconds before NOW, which the clock reads"""
        monkeypatch.setattr(fw_mod, "time", SimpleNamespace(time=lambda: cls.NOW))
        path = tmp_path / RESULT
        write_json(path, record)
        os.utime(path, (cls.NOW - age, cls.NOW - age))
        return str(path)

    def test_no_result_is_not_busy(self, tmp_path):
        assert AmsFirmware._busy(str(tmp_path / RESULT)) is False

    @pytest.mark.parametrize("age", [0, 3599])
    def test_a_running_result_under_an_hour_old_is_busy(self, monkeypatch, tmp_path, age):
        assert AmsFirmware._busy(self._result(monkeypatch, tmp_path, {"running": True},
                                              age)) is True

    @pytest.mark.parametrize("age", [3600, 7200])
    def test_a_running_result_an_hour_old_is_not_busy(self, monkeypatch, tmp_path, age):
        assert AmsFirmware._busy(self._result(monkeypatch, tmp_path, {"running": True},
                                              age)) is False

    def test_a_fresh_result_of_a_finished_update_is_not_busy(self, monkeypatch, tmp_path):
        assert AmsFirmware._busy(self._result(monkeypatch, tmp_path,
                                              {"running": False, "ok": True}, 0)) is False


class TestAmsFirmwareShowResult:
    TAIL = [button("Flash AMS", "AFC_BAMBU_FIRMWARE", "primary"), CLOSE, SHOW]
    LOG = "Its log is logs/ams_flash.log."

    @staticmethod
    def _show(tmp_path: Optional[pathlib.Path], record: Optional[Dict[str, Any]] = None
              ) -> List[LogLine]:
        printer = make_printer()
        folder = None
        if tmp_path is not None:
            folder = str(tmp_path)
            if record is not None:
                write_json(tmp_path / RESULT, record)
        AmsFirmware(printer)._show_result(FakeGcmd(), folder)
        return printer.afc.logger.messages

    def test_without_the_updater_nothing_said_how_it_ended(self):
        assert self._show(None) == [
            begin("AMS update stopped"), text("The AMS updater did not say how it ended."),
            text(self.LOG)] + self.TAIL

    def test_no_result_left_said_nothing_either(self, tmp_path):
        assert self._show(tmp_path) == [
            begin("AMS update stopped"), text("The AMS updater did not say how it ended."),
            text(self.LOG)] + self.TAIL

    def test_a_running_update_with_its_message(self, tmp_path):
        record = {"running": True, "ok": True, "message": "Bambu_AMS_1 is at 40%."}
        assert self._show(tmp_path, record) == [
            begin("AMS update running"), text("Bambu_AMS_1 is at 40%.")] + self.TAIL

    def test_a_running_update_without_a_message(self, tmp_path):
        assert self._show(tmp_path, {"running": True, "message": ""}) == [
            begin("AMS update running"), text("An AMS update is running.")] + self.TAIL

    def test_a_finished_update_with_its_message(self, tmp_path):
        record = {"ok": True, "message": "Bambu_AMS_1 runs 01.03.00.00."}
        assert self._show(tmp_path, record) == [
            begin("AMS update done"), text("Bambu_AMS_1 runs 01.03.00.00.")] + self.TAIL

    def test_a_finished_update_without_a_message(self, tmp_path):
        assert self._show(tmp_path, {"ok": True}) == [
            begin("AMS update done"), text("The unit took its new firmware.")] + self.TAIL

    def test_a_stopped_update_with_its_message(self, tmp_path):
        record = {"running": False, "ok": False, "message": "Bambu_AMS_1 did not answer."}
        assert self._show(tmp_path, record) == [
            begin("AMS update stopped"), text("Bambu_AMS_1 did not answer."),
            text(self.LOG)] + self.TAIL


class TestAmsFirmwareCachedChoices:
    FIRST = [{"version": "01.02.03.04", "file": "", "newest": True}]
    LATER = [{"version": "01.03.00.00", "file": "", "newest": True}]

    @classmethod
    def _setup(cls, monkeypatch: pytest.MonkeyPatch) -> Tuple[AmsFirmware, Recorder]:
        """:return tuple: a manager, and the folder reads, which give FIRST until told otherwise"""
        mgr = AmsFirmware(make_printer())
        looked = Recorder(result=cls.FIRST)
        monkeypatch.setattr(fw_mod, "choices_for", looked)
        return mgr, looked

    def test_a_board_not_read_yet_is_read(self, monkeypatch):
        mgr, looked = self._setup(monkeypatch)
        assert mgr._cached_choices("/updater", "n3f_rev5", 100.0) == self.FIRST
        assert looked.calls == [(("/updater", "n3f_rev5"), {})]
        assert mgr._choices == {"n3f_rev5": (100.0, self.FIRST)}

    def test_a_board_read_under_30_s_ago_is_not_read_again(self, monkeypatch):
        mgr, looked = self._setup(monkeypatch)
        mgr._cached_choices("/updater", "n3f_rev5", 100.0)
        looked.result = self.LATER
        assert mgr._cached_choices("/updater", "n3f_rev5", 129.5) == self.FIRST
        assert looked.call_count == 1
        assert mgr._choices == {"n3f_rev5": (100.0, self.FIRST)}

    def test_a_board_read_30_s_ago_is_read_again(self, monkeypatch):
        mgr, looked = self._setup(monkeypatch)
        mgr._cached_choices("/updater", "n3f_rev5", 100.0)
        looked.result = self.LATER
        assert mgr._cached_choices("/updater", "n3f_rev5", 130.0) == self.LATER
        assert looked.call_count == 2
        assert mgr._choices == {"n3f_rev5": (130.0, self.LATER)}

    def test_each_board_is_kept_apart(self, monkeypatch):
        mgr, looked = self._setup(monkeypatch)
        mgr._cached_choices("/updater", "n3f_rev5", 100.0)
        looked.result = self.LATER
        assert mgr._cached_choices("/updater", "n3s_rev5", 101.0) == self.LATER
        assert looked.calls == [(("/updater", "n3f_rev5"), {}), (("/updater", "n3s_rev5"), {})]
        assert mgr._choices == {"n3f_rev5": (100.0, self.FIRST),
                                "n3s_rev5": (101.0, self.LATER)}


class TestAmsFirmwareGetStatus:
    NEWER = [{"version": "01.03.00.00", "file": "", "newest": True}]
    NO_RESULT = {"unit": "", "ok": False, "message": ""}

    @staticmethod
    def _setup(tmp_path: Optional[pathlib.Path]) -> Tuple[BambuPrinter, AmsFirmware]:
        """
        Bambu_AMS_1, an AMS 2 Pro read on 01.02.03.04, and Bambu_AMS_2, an AMS 1 not read yet;
        with `tmp_path`, the updater beside a catalog listing 01.03.00.00 for the AMS 2 Pro.
        """
        printer = make_printer()
        read = make_bambu_unit("Bambu_AMS_1", printer=printer)
        read.fw_version, read.fw_model = "01.02.03.04", "N3F05"
        make_bambu_unit("Bambu_AMS_2", printer=printer, model="ams1", ams_index=1)
        if tmp_path is not None:
            add_updater(printer, tmp_path)
            write_catalog(tmp_path, [catalog_entry("n3f_rev5", "01.03.00.00")])
        return printer, manager_of(printer)

    def _units(self, choices: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
        """:param choices: Bambu_AMS_1's images; Bambu_AMS_2 has no board, so none"""
        return {"Bambu_AMS_1": {"version": "01.02.03.04", "model": "AMS 2 Pro",
                                "choices": choices},
                "Bambu_AMS_2": {"version": "", "model": "", "choices": []}}

    def test_without_the_updater_only_the_units_are_known(self):
        printer, mgr = self._setup(None)
        assert mgr.get_status(100.0) == {"updater": False, "running": False, "waiting": "",
                                         "result": {}, "units": self._units([])}
        # Bambu_AMS_1's board is known, but there is no folder to look in.
        assert mgr._choices == {}
        assert printer.afc.logger.messages == []

    def test_with_the_updater_a_known_board_has_its_images(self, tmp_path):
        printer, mgr = self._setup(tmp_path)
        assert mgr.get_status(100.0) == {"updater": True, "running": False, "waiting": "",
                                         "result": self.NO_RESULT,
                                         "units": self._units(self.NEWER)}
        # Bambu_AMS_2 has no board to look up.
        assert mgr._choices == {"n3f_rev5": (100.0, self.NEWER)}
        assert printer.afc.logger.messages == []

    def test_a_unit_with_no_board_is_given_no_model_name(self, monkeypatch, tmp_path):
        # board_name("") is "" as well, so only whether it is asked tells the two ways apart.
        _printer, mgr = self._setup(tmp_path)
        asked: List[str] = []
        real = fw_mod.board_name

        def spy(board: str) -> str:
            asked.append(board)
            return real(board)

        monkeypatch.setattr(fw_mod, "board_name", spy)
        assert mgr.get_status(100.0)["units"] == self._units(self.NEWER)
        assert asked == ["n3f_rev5"]

    def test_only_the_units_the_menu_lists_are_given(self, monkeypatch, tmp_path):
        printer, mgr = self._setup(tmp_path)
        listed = make_bambu_unit("Bambu_AMS_HT_1", printer=printer, model="ht", ams_index=4)
        monkeypatch.setattr(mgr, "_listed", Recorder(result=[listed]))
        assert mgr.get_status(100.0)["units"] == {
            "Bambu_AMS_HT_1": {"version": "", "model": "AMS HT", "choices": []}}

    def test_a_running_update_and_its_result(self, tmp_path):
        _printer, mgr = self._setup(tmp_path)
        write_json(tmp_path / RESULT, {"running": True, "unit": "Bambu_AMS_1", "ok": False,
                                       "message": "Bambu_AMS_1 is at 40%."})
        assert mgr.get_status(100.0) == {
            "updater": True, "running": True, "waiting": "",
            "result": {"unit": "Bambu_AMS_1", "ok": False, "message": "Bambu_AMS_1 is at 40%."},
            "units": self._units(self.NEWER)}

    def test_the_record_of_a_running_update_is_not_a_stopped_one(self, tmp_path):
        _printer, mgr = self._setup(tmp_path)
        write_json(tmp_path / STATE, {"unit": "Bambu_AMS_1"})
        write_json(tmp_path / RESULT, {"running": True, "unit": "Bambu_AMS_1"})
        status = mgr.get_status(100.0)
        assert (status["running"], status["waiting"]) == (True, "")

    def test_a_result_is_given_as_text_and_a_flag(self, tmp_path):
        _printer, mgr = self._setup(tmp_path)
        write_json(tmp_path / RESULT, {"unit": 2, "ok": 1, "message": 40})
        status = mgr.get_status(100.0)
        assert (status["running"], status["result"]) == (False, {"unit": "2", "ok": True,
                                                                  "message": "40"})
        # 1 == True, so the flag's type is checked on its own.
        assert status["result"]["ok"] is True

    def test_a_result_with_its_fields_empty(self, tmp_path):
        _printer, mgr = self._setup(tmp_path)
        write_json(tmp_path / RESULT, {"unit": None, "ok": None, "message": ""})
        status = mgr.get_status(100.0)
        assert (status["running"], status["result"]) == (False, self.NO_RESULT)

    def test_a_result_from_before_klipper_started_is_left_out(self, tmp_path):
        _printer, mgr = self._setup(tmp_path)
        write_json(tmp_path / RESULT, {"unit": "Bambu_AMS_1", "ok": True,
                                       "message": "Bambu_AMS_1 now runs 01.03.00.00."})
        before = mgr._started - 60
        os.utime(tmp_path / RESULT, (before, before))
        status = mgr.get_status(100.0)
        assert (status["running"], status["result"]) == (False, self.NO_RESULT)

    def test_an_update_running_since_before_klipper_started_still_shows(self, tmp_path):
        _printer, mgr = self._setup(tmp_path)
        write_json(tmp_path / RESULT, {"running": True, "unit": "Bambu_AMS_1",
                                       "message": "Bambu_AMS_1 is at 40%."})
        before = mgr._started - 60
        os.utime(tmp_path / RESULT, (before, before))
        status = mgr.get_status(100.0)
        assert status["running"] is True
        assert status["result"]["message"] == "Bambu_AMS_1 is at 40%."

    def test_a_unit_left_in_its_bootloader_is_named_as_configured(self, tmp_path):
        _printer, mgr = self._setup(tmp_path)
        write_json(tmp_path / STATE, {"unit": "bambu_ams_2"})
        assert mgr.get_status(100.0)["waiting"] == "Bambu_AMS_2"

    @pytest.mark.parametrize("state", [None, {}, {"unit": "Bambu_AMS_9"}, {"unit": 2}])
    def test_a_state_naming_no_unit_leaves_none_waiting(self, tmp_path, state):
        _printer, mgr = self._setup(tmp_path)
        if state is not None:
            write_json(tmp_path / STATE, state)
        assert mgr.get_status(100.0) == {"updater": True, "running": False, "waiting": "",
                                         "result": self.NO_RESULT,
                                         "units": self._units(self.NEWER)}
