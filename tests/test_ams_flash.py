"""Unit tests for Firmwares/Bambu_AMS/ams_flash.py."""

from __future__ import annotations

import builtins
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
from types import ModuleType
from typing import Any, Callable, Dict, Iterator, List, Optional, Tuple

import pytest


AMS_FLASH_DIR = Path(__file__).resolve().parents[1] / "Firmwares" / "Bambu_AMS"


def _load_ams_flash() -> ModuleType:
    """Load the updater from its file; it is a script, not an importable package."""
    spec = importlib.util.spec_from_file_location("ams_flash", AMS_FLASH_DIR / "ams_flash.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ams_flash = _load_ams_flash()


# The real enter-loader frames from the two models' update captures, as
# (device, AMS id, frame). Counters and CRCs are the printer's own.
CAPTURED_CMD1 = {
    "ams2": (0x0700, 0x00, "3d04110a31004800070009060101070000000000000000"
                           "0000000000000000000000000000000000000000000000"
                           "00f0d3"),
    "ht": (0x1800, 0x80, "3d04a60b3100370018000906010118000000008000000000"
                         "00000000000000000000000000000000000000000000004"
                         "7d4"),
}


# The HT's own loader answer (class 0x00, target 0009 = the updater, source
# 0018 = the unit). This is what a real "it is in its loader" looks like.
HT_LOADER_REPLY = ("3d0000002900ea0009001806010118000000008000000000d0020000"
                   "00000000040000000000007179")


# The vendor image name each model's header frame carries (BIMH container).
IMAGE_NAMES = {
    (0x0700, 0x00): "n3f_rev5-firmware-v05.00.22.22-20260702164346.bin.sig",
    (0x1800, 0x80): "n3s_rev5-firmware-v05.00.22.19-20260616201708.bin.sig",
}


AMS1_IMAGE = "ams_rev8-firmware-v01.00.06.87-20260109152259.bin.sig"


# Captured from a real printer asking two boxed units their generation: the
# addressed 0x3702 query to id 0 and to id 1, and the AMS 2's answer from id 1.
# The AMS 1 at id 0 never answered.
QUERY_3702_ID0 = "3d050000120012000700033702000001c4cf"


QUERY_3702_ID1 = "3d050000120012000700033702010001f4f8"


ANSWER_3702_ID1 = ("3d0000002800760003000737020101000000008e151469ae15bc6891"
                   "0da00d930d011900c901eb31")


# The same answer as it comes from id 0: the id byte set to 0, CRC redone.
ANSWER_3702_ID0 = ("3d0000002800760003000737020001000000008e151469ae15bc6891"
                   "0da00d930d011900c9015f74")


# The bridge's own log-drain poll to id 0 and a unit's idle answer, from an
# AMS 1 insert capture; an AMS 2 answers it the same way.
DRAIN_ID0 = "3d05000013008e000700031a0200000000e4ce"


DRAIN_ANSWER_ID0 = "3d0000001500f4000300071a020000000000003473"


def _ams_flash_artifact(target: int, ams_id: int, blocks: int = 2,
                        image: Optional[str] = None) -> Dict[str, Any]:
    """A minimal artifact whose frames carry the routing under test.

    Bodies are frame[7:-2], so body[0:2] is the target and body[12] (frame
    byte 19) is the AMS id. The header's payload holds a BIMH container
    header naming the image, as a real one does.

    :param target: device address the frames are for
    :param ams_id: AMS id the frames are for
    :param blocks: number of data blocks
    :param image: vendor image name, default the real one for that address
    :return dict: the artifact, as load_artifact returns one
    """
    def body(cmd: int) -> bytes:
        frame = bytearray(40)
        frame[0:2] = target.to_bytes(2, "little")
        frame[2:4] = b"\x00\x09"                         # source: the updater
        frame[4:7] = bytes((0x06, 0x01, cmd))
        frame[7] = target >> 8                           # device class
        frame[12] = ams_id
        return bytes(frame)

    name = IMAGE_NAMES[(target, ams_id)] if image is None else image
    header = body(0x02)[:32] + b"BIMH" + bytes(44) + name.encode() + bytes(8)
    return {"header": header, "blocks": [body(0x03)] * blocks}


class _AmsFlashScriptedLink:
    """A bridge link that answers each command from a script.

    Answer lines come back in order, then the port goes quiet (None), as the
    real Link's lines() does.
    """

    def __init__(self, answer: Callable[[Dict[str, Any]], List[str]]) -> None:
        self.answer = answer
        self.sent: List[Dict[str, Any]] = []
        self.queue: List[str] = []
        self.dead = False

    def send(self, obj: Dict[str, Any]) -> None:
        self.sent.append(obj)
        self.queue.extend(self.answer(obj))

    def lines(self) -> Iterator[Optional[str]]:
        while True:
            yield self.queue.pop(0) if self.queue else None


class _AmsFlashClock:
    """The updater's `time`: time() steps on every read, sleep() takes no time
    unless given a stand-in, and strftime() reads a fixed wall clock."""

    WALL = {"%Y-%m-%d %H:%M:%S": "2026-10-05 09:30:00",
            "%Y-%m-%d %H:%M": "2026-10-05 09:30"}

    def __init__(self, step: float,
                 sleep: Optional[Callable[[float], None]] = None) -> None:
        self.now = 1000.0
        self.step = step
        self._sleep = sleep

    def time(self) -> float:
        self.now += self.step
        return self.now

    def sleep(self, seconds: float) -> None:
        if self._sleep is not None:
            self._sleep(seconds)

    def strftime(self, fmt: str) -> str:
        return self.WALL[fmt]


class _AmsFlashPty:
    """A pseudo-terminal standing in for the bridge's USB serial port."""

    def __init__(self) -> None:
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        self._open = [self.master, slave]

    def hang_up(self) -> None:
        """Close both ends, as a bridge pulled off the USB bus does."""
        for fd in self._open:
            os.close(fd)
        self._open = []


@pytest.fixture
def ams_flash_pty() -> Iterator[_AmsFlashPty]:
    port = _AmsFlashPty()
    yield port
    port.hang_up()


class TestBuildCmd1:
    """build_cmd1: the enter-loader poke, addressed per model."""

    @staticmethod
    def _crc(data: bytes, width: int, poly: int, init: int) -> int:
        """Plain MSB-first CRC with no reflection and no final XOR."""
        top, mask = 1 << (width - 1), (1 << width) - 1
        reg = init
        for byte in data:
            reg ^= byte << (width - 8)
            for _ in range(8):
                reg = ((reg << 1) ^ poly) & mask if reg & top else (reg << 1) & mask
        return reg

    @pytest.mark.parametrize("model", sorted(CAPTURED_CMD1))
    def test_standalone_cmd1_reproduces_the_captured_frame(self, model: str) -> None:
        target, ams_id, hexed = CAPTURED_CMD1[model]
        want = bytes.fromhex(hexed)
        counter = int.from_bytes(want[2:4], "little")
        assert ams_flash.build_cmd1(target, ams_id, counter) == want

    def test_the_two_models_differ_in_the_payload_not_only_the_address(self) -> None:
        # Aiming at an HT by changing the target alone would leave the class
        # byte and the AMS id at their AMS 2 values, and the HT ignores that.
        ams2 = ams_flash.build_cmd1(0x0700, 0x00)
        ht = ams_flash.build_cmd1(0x1800, 0x80)
        # (default counter, address, device class, AMS id)
        assert (ams2[2:4], ams2[7:9], ams2[14], ams2[19]) == (
            b"\x77\x00", b"\x00\x07", 0x07, 0x00)
        assert (ht[2:4], ht[7:9], ht[14], ht[19]) == (b"\x77\x00", b"\x00\x18", 0x18, 0x80)

    @pytest.mark.parametrize("target,class_byte", [(0x0700, 0x07), (0x1800, 0x18)])
    def test_the_device_class_byte_tracks_the_address(self, target: int,
                                                        class_byte: int) -> None:
        # [14] is derived from the address, so it cannot drift out of step with it.
        assert ams_flash.build_cmd1(target, 0, 1)[14] == class_byte

    @pytest.mark.parametrize("target,ams_id", [(0x0700, 0x00), (0x1800, 0x80)])
    def test_cmd1_stays_crc_correct_for_both_models(self, target: int, ams_id: int) -> None:
        frame = ams_flash.build_cmd1(target, ams_id, 0x1234)
        assert len(frame) == 49
        assert frame[2:4] == b"\x34\x12"
        assert frame[6] == self._crc(frame[:6], 8, 0x39, 0x66)
        assert int.from_bytes(frame[47:49], "little") == self._crc(
            frame[:47], 16, 0x1021, 0x913D)


class TestLoaderTargetOf:
    """loader_target_of: the (device, AMS id) an artifact's frames are for."""

    @pytest.mark.parametrize("target,ams_id,address,class_byte", [
        (0x0700, 0x00, b"\x00\x07", 0x07),
        (0x1800, 0x80, b"\x00\x18", 0x18),
    ], ids=["ams2", "ht"])
    def test_standalone_cmd1_is_addressed_off_the_artifact(
            self, target: int, ams_id: int, address: bytes, class_byte: int) -> None:
        art = _ams_flash_artifact(target, ams_id)
        assert ams_flash.loader_target_of(art) == (target, ams_id)
        frame = ams_flash.build_cmd1(*ams_flash.loader_target_of(art))
        assert (frame[7:9], frame[14], frame[19]) == (address, class_byte, ams_id)

    def test_standalone_has_no_fixed_loader_target(self) -> None:
        # There is no fixed target to fall back on: whatever the header
        # carries is the target, even an address no model uses.
        art = _ams_flash_artifact(0x2A0B, 0x42, image="")
        assert ams_flash.loader_target_of(art) == (0x2A0B, 0x42)


class TestArtifactMismatch:
    """artifact_mismatch: why an artifact must not go out under a model's command."""

    @pytest.mark.parametrize("model,own,other,why", [
        ("ht", (0x1800, 0x80), (0x0700, 0x00),
         "this artifact is addressed to device 0x0700, AMS id 0x00, not to an AMS HT "
         "(device 0x1800, AMS id 0x80). Wrong file for this command?"),
        ("ams2", (0x0700, 0x00), (0x1800, 0x80),
         "this artifact is addressed to device 0x1800, AMS id 0x80, not to an AMS 2 Pro "
         "(device 0x0700, AMS id 0x00). Wrong file for this command?"),
    ], ids=["ht", "ams2"])
    def test_an_artifact_for_another_device_is_refused(
            self, model: str, own: Tuple[int, int], other: Tuple[int, int],
            why: str) -> None:
        assert ams_flash.artifact_mismatch(_ams_flash_artifact(*own), model) is None
        assert ams_flash.artifact_mismatch(_ams_flash_artifact(*other), model) == why

    @pytest.mark.parametrize("model,why", [
        ("ams2", "this artifact is addressed to device 0x0700, AMS id 0x80, not to an "
                 "AMS 2 Pro (device 0x0700, AMS id 0x00). Wrong file for this command?"),
        ("ht", "this artifact is addressed to device 0x0700, AMS id 0x80, not to an "
               "AMS HT (device 0x1800, AMS id 0x80). Wrong file for this command?"),
    ], ids=["right-device-wrong-id", "right-id-wrong-device"])
    def test_either_half_of_the_address_alone_is_refused(self, model: str, why: str) -> None:
        art = _ams_flash_artifact(0x0700, 0x80, image=IMAGE_NAMES[(0x0700, 0x00)])
        assert ams_flash.artifact_mismatch(art, model) == why

    @pytest.mark.parametrize("model,own,image,other,why", [
        ("ams2", (0x0700, 0x00), IMAGE_NAMES[(0x0700, 0x00)], (0x1800, 0x80),
         "this artifact is addressed to device 0x1800, AMS id 0x80, not to an AMS 2 Pro "
         "(device 0x0700, AMS id 0x00). Wrong file for this command?"),
        ("ams1", (0x0700, 0x00), AMS1_IMAGE, (0x1800, 0x80),
         "this artifact is addressed to device 0x1800, AMS id 0x80, not to an AMS 1 "
         "(device 0x0700, AMS id 0x00). Wrong file for this command?"),
        ("ht", (0x1800, 0x80), IMAGE_NAMES[(0x1800, 0x80)], (0x0700, 0x00),
         "this artifact is addressed to device 0x0700, AMS id 0x00, not to an AMS HT "
         "(device 0x1800, AMS id 0x80). Wrong file for this command?"),
    ], ids=["ams2", "ams1", "ht"])
    def test_each_model_command_has_its_own_address(
            self, model: str, own: Tuple[int, int], image: str,
            other: Tuple[int, int], why: str) -> None:
        assert ams_flash.artifact_mismatch(
            _ams_flash_artifact(*own, image=image), model) is None
        assert ams_flash.artifact_mismatch(
            _ams_flash_artifact(*other, image=image), model) == why

    def test_an_ams2_image_is_refused_by_the_ams1_command_and_back(self) -> None:
        # Same address, so only the image's own name can tell them apart. An
        # AMS 2 image saved as ams1_artifact.json would otherwise erase an
        # AMS 1 with no AMS 1 image to put back.
        ams2 = _ams_flash_artifact(0x0700, 0x00)
        ams1 = _ams_flash_artifact(0x0700, 0x00, image=AMS1_IMAGE)
        assert ams_flash.artifact_mismatch(ams1, "ams1") is None
        assert ams_flash.artifact_mismatch(ams2, "ams1") == (
            "this artifact carries the image "
            "'n3f_rev5-firmware-v05.00.22.22-20260702164346.bin.sig', not an AMS 1 "
            "image (ams_...). Wrong file for this command?")
        assert ams_flash.artifact_mismatch(ams1, "ams2") == (
            "this artifact carries the image "
            "'ams_rev8-firmware-v01.00.06.87-20260109152259.bin.sig', not an AMS 2 Pro "
            "image (n3f_...). Wrong file for this command?")
        unnamed = _ams_flash_artifact(0x0700, 0x00, image="")
        assert ams_flash.artifact_mismatch(unnamed, "ams2") == (
            "this artifact carries the image 'unnamed', not an AMS 2 Pro image "
            "(n3f_...). Wrong file for this command?")

    def test_a_header_that_is_not_an_update_header_is_refused(self) -> None:
        art = _ams_flash_artifact(0x0700, 0x00)
        header = bytearray(art["header"])
        header[6] = 0x03                                 # a data block's command
        art["header"] = bytes(header)
        assert ams_flash.artifact_mismatch(art, "ams2") == (
            "the artifact's first frame is not an update header")

    def test_a_block_addressed_elsewhere_is_refused(self) -> None:
        art = _ams_flash_artifact(0x0700, 0x00, blocks=3)
        stray = bytearray(art["blocks"][1])
        stray[12] = 0x01                                 # another unit's id
        art["blocks"][1] = bytes(stray)
        assert ams_flash.artifact_mismatch(art, "ams2") == (
            "data block 1 is not an update block addressed like the header; the file "
            "is damaged or mixed")

    @pytest.mark.parametrize("offset,value", [(1, 0x18), (6, 0x02)],
                             ids=["device", "not-a-data-block"])
    def test_any_one_stray_field_refuses_the_block(self, offset: int, value: int) -> None:
        art = _ams_flash_artifact(0x0700, 0x00, blocks=3)
        stray = bytearray(art["blocks"][2])
        stray[offset] = value
        art["blocks"][2] = bytes(stray)
        assert ams_flash.artifact_mismatch(art, "ams2") == (
            "data block 2 is not an update block addressed like the header; the file "
            "is damaged or mixed")

    @pytest.mark.parametrize("model,other,why", [
        ("ams2", "ht",
         "this artifact is addressed to device 0x0700, AMS id 0x00, not to an AMS HT "
         "(device 0x1800, AMS id 0x80). Wrong file for this command?"),
        ("ht", "ams2",
         "this artifact is addressed to device 0x1800, AMS id 0x80, not to an AMS 2 Pro "
         "(device 0x0700, AMS id 0x00). Wrong file for this command?"),
    ], ids=["ams2", "ht"])
    def test_the_shipped_artifacts_pass_the_flashers_own_checks(
            self, model: str, other: str, why: str) -> None:
        # The testers flash these files as they are. They must load, verify,
        # add up to their declared image, and be addressed to their own model.
        art = ams_flash.load_artifact(str(AMS_FLASH_DIR / f"{model}_artifact.json"))
        total, summed = ams_flash.artifact_image_bytes(art)
        assert total - summed == 416
        assert ams_flash.artifact_mismatch(art, model) is None
        assert ams_flash.artifact_mismatch(art, other) == why


class TestArtifactImageBytes:
    """artifact_image_bytes: the declared image size and what the blocks carry."""

    @staticmethod
    def _header(total: int) -> bytes:
        """A header body declaring `total` image bytes at frame [23:26]."""
        body = bytearray(40)
        body[16:19] = total.to_bytes(3, "little")
        return bytes(body)

    @staticmethod
    def _block(carried: int) -> bytes:
        """A block body counting `carried` firmware bytes at frame [35:39]."""
        body = bytearray(40)
        body[28:32] = carried.to_bytes(4, "little")
        return bytes(body)

    @pytest.mark.parametrize("nblocks,per", [(157, 1020), (168, 1022), (3, 40)])
    def test_declared_image_equals_carried_bytes_plus_the_container_header(
            self, nblocks: int, per: int) -> None:
        # Completeness is judged against the image the artifact declares, not
        # a block count, so it holds at any block count.
        total = nblocks * per + 416
        art = {"header": self._header(total), "blocks": [self._block(per)] * nblocks}
        assert ams_flash.artifact_image_bytes(art) == (total, nblocks * per)

    def test_a_truncated_artifact_is_caught_however_many_blocks_it_has(self) -> None:
        # The failure a count cannot see: the right number of blocks, the
        # wrong number of bytes (the last block carries half).
        art = {"header": self._header(160556),
               "blocks": [self._block(1020)] * 156 + [self._block(510)]}
        assert ams_flash.artifact_image_bytes(art) == (160556, 159630)


class TestLinkRead:
    """Link._read over a real pseudo-terminal."""

    def test_a_usb_hangup_is_a_dead_link_not_a_quiet_one(
            self, ams_flash_pty: _AmsFlashPty) -> None:
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        assert link.dead is False
        ams_flash_pty.hang_up()
        with pytest.raises(IOError) as raised:
            link._read()
        assert str(raised.value) == "bridge link closed"
        assert link.dead is True
        link.close()

    def test_a_quiet_port_reads_as_nothing(self, ams_flash_pty: _AmsFlashPty) -> None:
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        assert link._read() == b""
        assert link.dead is False
        link.close()

    def test_what_is_buffered_is_returned(self, ams_flash_pty: _AmsFlashPty) -> None:
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        os.write(ams_flash_pty.master, b'{"evt":"info"}\n')
        assert link._read() == b'{"evt":"info"}\n'
        assert link.dead is False
        link.close()


class TestLinkLines:
    """Link.lines over a real pseudo-terminal."""

    def test_the_usb_link_needs_no_pyserial_and_never_blocks(
            self, ams_flash_pty: _AmsFlashPty, monkeypatch: pytest.MonkeyPatch) -> None:
        # A USB bridge is what the testers have. The link must work on the
        # system python3 (no pyserial) and hand control back on a quiet port.
        real_import = builtins.__import__

        def no_serial(name: str, *args: Any, **kwargs: Any) -> ModuleType:
            if name == "serial":
                raise ImportError("no pyserial here")
            return real_import(name, *args, **kwargs)

        monkeypatch.setattr(builtins, "__import__", no_serial)
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        lines = link.lines()
        assert next(lines) is None                       # quiet: yields, no hang
        os.write(ams_flash_pty.master, b'{"evt":"info"}\n')
        assert next(lines) == '{"evt":"info"}'
        assert link.buf == b""
        link.send({"cmd": "info"})
        assert os.read(ams_flash_pty.master, 100) == b'{"cmd": "info"}\n'
        link.close()

    def test_a_line_is_held_until_it_ends(self, ams_flash_pty: _AmsFlashPty) -> None:
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        lines = link.lines()
        os.write(ams_flash_pty.master, b'{"a":1}\r\n')
        assert next(lines) == '{"a":1}'                  # the carriage return is dropped
        os.write(ams_flash_pty.master, b'{"b"')
        assert next(lines) is None                       # half a line: wait for the rest
        assert link.buf == b'{"b"'
        os.write(ams_flash_pty.master, b':2}\n{"c":3}\n')
        assert next(lines) == '{"b":2}'
        assert next(lines) == '{"c":3}'                  # two lines from one read
        assert link.buf == b""
        link.close()


class TestRxOf:
    """_rx_of: the captured bytes in a raw or txsend reply."""

    def test_a_reply_with_a_lost_byte_is_no_answer_not_a_crash(self) -> None:
        assert ams_flash._rx_of('{"evt":"txsend","rx":"ABC"}') == b""
        assert ams_flash._rx_of('{"evt":"txsend","rx":"ABCD"}') == b"\xab\xcd"

    def test_a_reply_without_captured_bytes_is_no_answer(self) -> None:
        assert ams_flash._rx_of('{"evt":"txsend","crc":"bad"}') == b""
        assert ams_flash._rx_of(None) == b""


class TestSendFrame:
    """send_frame: stage a frame over txbuf lines, send it, return the reply."""

    def test_a_damaged_staged_frame_is_staged_again(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        # fw >= 1.75 answers "crc":"bad" and sends nothing when a txbuf line
        # was mangled on the way. The frame is staged again; it must not read
        # as a missing ack, which would cost a whole erase pass.
        txsends: List[Dict[str, Any]] = []

        def answer(obj: Dict[str, Any]) -> List[str]:
            if obj["cmd"] == "txbuf":
                return ['{"evt":"ack","cmd":"txbuf"}']
            txsends.append(obj)
            if len(txsends) == 1:
                return ['{"evt":"txsend","crc":"bad"}']
            return ['{"evt":"txsend","rx":"AABB"}']

        link = _AmsFlashScriptedLink(answer)
        assert ams_flash.send_frame(link, bytes(250)) == b"\xaa\xbb"
        staging = [{"cmd": "txbuf", "off": 0, "hex": "00" * 100},
                   {"cmd": "txbuf", "off": 100, "hex": "00" * 100},
                   {"cmd": "txbuf", "off": 200, "hex": "00" * 50},
                   {"cmd": "txsend", "n": 250, "us": 8000}]
        assert link.sent == staging * 2                  # every line staged again
        assert link.dead is False
        assert capsys.readouterr() == (
            "", "    staged frame arrived damaged -- staging it again (1/4)\n")

        link = _AmsFlashScriptedLink(
            lambda obj: ['{"evt":"ack","cmd":"txbuf"}'] if obj["cmd"] == "txbuf"
            else ['{"evt":"txsend","crc":"bad"}'])
        with pytest.raises(IOError) as raised:
            ams_flash.send_frame(link, bytes(10), stage_tries=2)
        assert str(raised.value) == (
            "staged frame still damaged after re-staging -- the link is dropping bytes")
        assert link.sent == [{"cmd": "txbuf", "off": 0, "hex": "00" * 10},
                             {"cmd": "txsend", "n": 10, "us": 8000}] * 2
        assert link.dead is False
        assert capsys.readouterr() == (
            "", "    staged frame arrived damaged -- staging it again (1/2)\n"
                "    staged frame arrived damaged -- staging it again (2/2)\n")


class TestLoaderHits:
    """loader_hits: the loader's own answer in a capture window."""

    @pytest.mark.parametrize("target,ams_id", [(0x0700, 0x00), (0x1800, 0x80)])
    def test_standalone_gate_ignores_our_own_cmd1(self, target: int, ams_id: int) -> None:
        assert ams_flash.loader_hits(ams_flash.build_cmd1(target, ams_id).hex()) == []

    def test_standalone_gate_recognises_the_real_loader_reply(self) -> None:
        assert ams_flash.loader_hits(HT_LOADER_REPLY) == ["ams-origin/op0601"]
        # The same reply as if it came from us (source 0x0900), CRC redone so
        # only the source rules it out.
        from_us = ("3d0000002900ea0009000906010118000000008000000000d0020000"
                   "00000000040000000000001876")
        assert ams_flash.loader_hits(from_us) == []

    def test_the_loader_gate_needs_a_whole_frame(self) -> None:
        # A damaged echo, a 06 01 inside another frame's payload, and a match
        # on half a byte all used to read as "the loader answered".
        cmd1 = bytearray(ams_flash.build_cmd1(0x0700, 0x00))
        cmd1[10] = 0x0B                                  # source byte hit by noise
        assert ams_flash.loader_hits(bytes(cmd1).hex()) == []
        assert ams_flash.loader_hits("3d00aa0030601f00") == []
        assert ams_flash.loader_hits("3d05a0601b") == []
        assert ams_flash.loader_hits(
            ams_flash.build_cmd1(0x0700, 0x00).hex() + HT_LOADER_REPLY) == [
            "ams-origin/op0601"]

    def test_another_ops_answer_is_not_the_loader(self) -> None:
        assert ams_flash.loader_hits(ANSWER_3702_ID1) == []

    def test_the_loaders_own_text_counts_on_a_byte_boundary(self) -> None:
        banner = b"[MCU_UP] Loader Version: 23".hex()
        assert ams_flash.loader_hits(banner) == ["[MCU_UP]", "Loader Version"]
        assert ams_flash.loader_hits(b"[MCU_UP] ams 128 wait cmd1!".hex().upper()) == [
            "[MCU_UP]", "wait cmd1"]
        # Shifted by half a byte, the same hex digits are no longer that text.
        assert ams_flash.loader_hits("0" + banner + "0") == []

    def test_a_capture_that_is_not_hex_holds_no_frame(self) -> None:
        assert ams_flash.loader_hits("zz" + HT_LOADER_REPLY) == []


class TestBuild3702:
    """build_3702: the printer's addressed generation query."""

    def test_the_generation_query_is_the_captured_frame(self) -> None:
        assert ams_flash.build_3702(0x00).hex() == QUERY_3702_ID0
        assert ams_flash.build_3702(0x01).hex() == QUERY_3702_ID1


class TestBuildDrain:
    """build_drain: the bridge's own log-drain poll."""

    def test_the_control_query_is_the_bridges_own_drain(self) -> None:
        assert ams_flash.build_drain(0x00).hex() == DRAIN_ID0


class TestAnswers3702:
    """answers_3702: a unit's own answer to the generation query."""

    def test_only_the_units_own_answer_counts(self) -> None:
        echo = bytes.fromhex(QUERY_3702_ID1)
        answer = bytes.fromhex(ANSWER_3702_ID1)
        assert ams_flash.answers_3702(echo + answer, 0x01) is True
        assert ams_flash.answers_3702(echo + answer, 0x00) is False   # another unit's
        assert ams_flash.answers_3702(echo, 0x01) is False            # our own query
        damaged = bytearray(answer)
        damaged[20] ^= 0xFF
        assert ams_flash.answers_3702(echo + bytes(damaged), 0x01) is False

    def test_the_units_answer_to_another_op_does_not_count(self) -> None:
        # The drain answer comes from the same device and id; only its op differs.
        assert ams_flash.answers_3702(bytes.fromhex(DRAIN_ANSWER_ID0), 0x00) is False


class TestWireOnline:
    """wire_online: the chain indices the bridge itself sees answering."""

    def test_the_bus_count_comes_from_the_bridge_itself(self) -> None:
        status = ('{"evt":"status","online":true,"units":[{"n":4,"online":true},'
                  '{"n":1,"online":false},{"n":0,"online":true}]}')
        link = _AmsFlashScriptedLink(lambda obj: [status])
        assert ams_flash.wire_online(link) == [0, 4]
        assert link.sent == [{"cmd": "status"}]

    def test_a_damaged_status_line_is_never_half_read(self) -> None:
        bad = ['{"evt":"status","units":[{"n":0,"online":true},{"n":1,"onl']
        link = _AmsFlashScriptedLink(lambda obj: bad)
        with pytest.raises(IOError) as raised:
            ams_flash.wire_online(link)
        assert str(raised.value) == "could not read the unit list from the bridge"
        assert link.sent == [{"cmd": "status"}] * 3


class TestProbeGeneration:
    """probe_generation: how often a boxed unit answers 0x3702 and the drain."""

    ASKS = [{"cmd": "raw", "hex": QUERY_3702_ID0, "us": 100000},
            {"cmd": "raw", "hex": DRAIN_ID0, "us": 100000}] * 8

    @staticmethod
    def _unit(gen_answers: bool, drain_answers: bool) -> Callable[[Dict[str, Any]], List[str]]:
        """A boxed unit at id 0 behind the bridge: it answers the drain while
        listening, and the generation query only if it is an AMS 2."""
        def answer(obj: Dict[str, Any]) -> List[str]:
            rx = bytes.fromhex(obj["hex"])               # the bridge hears its own frame
            if obj["hex"] == QUERY_3702_ID0 and gen_answers:
                rx += bytes.fromhex(ANSWER_3702_ID0)
            if obj["hex"] == DRAIN_ID0 and drain_answers:
                rx += bytes.fromhex(DRAIN_ANSWER_ID0)
            return [f'{{"evt":"raw","tx":18,"rx":"{rx.hex().upper()}"}}']
        return answer

    def test_an_ams2_answers_the_generation_query(self) -> None:
        link = _AmsFlashScriptedLink(self._unit(gen_answers=True, drain_answers=True))
        assert ams_flash.probe_generation(link, 0x00) == (8, 8)
        assert link.sent == self.ASKS                    # interleaved, 8 rounds

    def test_an_ams1_listens_but_stays_silent_to_the_generation_query(self) -> None:
        link = _AmsFlashScriptedLink(self._unit(gen_answers=False, drain_answers=True))
        assert ams_flash.probe_generation(link, 0x00) == (0, 8)
        assert link.sent == self.ASKS

    def test_a_unit_that_answers_nothing_is_not_an_ams1(self) -> None:
        # Silence alone used to read as "AMS 1". A unit in its loader, or a
        # bus that is dropping replies, is silent to both.
        link = _AmsFlashScriptedLink(self._unit(gen_answers=False, drain_answers=False))
        assert ams_flash.probe_generation(link, 0x00) == (0, 0)
        assert link.sent == self.ASKS


class TestLoaderOnBus:
    """loader_on_bus: whether some unit on the bus sits in its bootloader."""

    SNIFF = [{"cmd": "sniff", "on": 1}, {"cmd": "sniff", "on": 0}]

    @staticmethod
    def _bus(heard: str) -> Callable[[Dict[str, Any]], List[str]]:
        """A bridge whose sniff hears `heard` (hex, empty for nothing) once."""
        def answer(obj: Dict[str, Any]) -> List[str]:
            on = "true" if obj["on"] else "false"
            out = [f'{{"evt":"sniff_mode","on":{on}}}']
            if obj["on"] and heard:
                out.append(f'{{"evt":"sniff","us":1,"n":41,"hex":"{heard}"}}')
            return out
        return answer

    def test_a_unit_in_its_loader_is_heard_on_the_bus(self) -> None:
        link = _AmsFlashScriptedLink(self._bus(HT_LOADER_REPLY.upper()))
        assert ams_flash.loader_on_bus(link) is True
        assert link.sent == self.SNIFF                   # always switched off

    def test_a_quiet_bus_has_no_loader(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(ams_flash, "time", _AmsFlashClock(step=1.0))
        link = _AmsFlashScriptedLink(self._bus(""))
        assert ams_flash.loader_on_bus(link) is False
        assert link.sent == self.SNIFF

    def test_the_resumed_units_own_loader_does_not_count(
            self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(ams_flash, "time", _AmsFlashClock(step=1.0))
        link = _AmsFlashScriptedLink(self._bus(HT_LOADER_REPLY.upper()))
        assert ams_flash.loader_on_bus(link, exclude=(0x1800, 0x80)) is False
        assert link.sent == self.SNIFF

    def test_another_units_loader_counts_while_one_is_resumed(self) -> None:
        link = _AmsFlashScriptedLink(self._bus(HT_LOADER_REPLY.upper()))
        assert ams_flash.loader_on_bus(link, exclude=(0x0700, 0x00)) is True
        assert link.sent == self.SNIFF


class TestLoaderAnswer:
    """loader_answer: the addressed unit's loader answering cmd1."""

    def test_the_loader_answer_must_come_from_the_addressed_unit(self) -> None:
        reply = bytes.fromhex(HT_LOADER_REPLY)           # HT: 0x1800, id 0x80
        assert ams_flash.loader_answer(reply, 0x1800, 0x80) is True
        assert ams_flash.loader_answer(reply, 0x0700, 0x00) is False   # another unit's
        assert ams_flash.loader_answer(
            ams_flash.build_cmd1(0x1800, 0x80), 0x1800, 0x80) is False
        banner = b"[MCU_UP] Loader Version: 23"
        assert ams_flash.loader_answer(banner, 0x0700, 0x00) is False  # names no unit
        assert ams_flash.loader_answer(b"..[MCU_UP] 0 resev cmd 0x1..", 0x0700, 0x00) is True
        assert ams_flash.loader_answer(b"[MCU_UP] ams 128 wait cmd1!", 0x1800, 0x80) is True
        assert ams_flash.loader_answer(
            b"[MCU_UP] 128 send cmd 0x1 back", 0x1800, 0x80) is True
        assert ams_flash.loader_answer(b"[MCU_UP] 128 resev cmd 0x1", 0x0700, 0x00) is False
        assert ams_flash.loader_answer(b"[MCU_UP] 10 resev cmd 0x1", 0x0700, 0x00) is False

    @pytest.mark.parametrize("dev,ams_id", [(0x1800, 0x00), (0x0700, 0x80)],
                             ids=["right-device-wrong-id", "right-id-wrong-device"])
    def test_half_the_address_is_another_unit(self, dev: int, ams_id: int) -> None:
        assert ams_flash.loader_answer(bytes.fromhex(HT_LOADER_REPLY), dev, ams_id) is False


class TestDoFlash:
    """do_flash: whole-flash passes until the loader verifies the image."""

    # The loader's answers to one good pass over a two-block artifact: cmd1,
    # the header, block 0, and block 1 with the verdict.
    GOOD_PASS = [b"[MCU_UP] 0 resev cmd 0x1", b"\x06", b"\x06", b"success!"]
    GOOD_PASS_SENDS = ["fwreplay", "txbuf", "txsend", "txbuf", "txbuf", "txsend",
                       "txbuf", "txsend", "txbuf", "txsend", "fwreplay"]
    OPENING = ("    up to 7 attempt(s); only a loader 'success!' ends it.\n"
               "    == flash attempt 1/7 ==\n")
    PASSED = ("    loader confirmed after cmd1 #1\n"
              "    header sent, erase triggered; streaming blocks...\n"
              "    loader: success! -- image verified, resetting into the app\n"
              "    FLASH CONFIRMED: the loader verified the image and reset into the "
              "new firmware.\n")

    @staticmethod
    def _loader(replies: List[bytes],
                events: List[str]) -> Callable[[Dict[str, Any]], List[str]]:
        """A bridge in front of a unit's loader: it acks every staged line and
        answers each txsend with the next of `replies`."""
        def answer(obj: Dict[str, Any]) -> List[str]:
            events.append(obj["cmd"])
            if obj["cmd"] == "fwreplay":
                on = "true" if obj["on"] else "false"
                return [f'{{"evt":"fwreplay","on":{on}}}']
            if obj["cmd"] == "txbuf":
                return ['{"evt":"ack","cmd":"txbuf"}']
            return [f'{{"evt":"txsend","rx":"{replies.pop(0).hex().upper()}"}}']
        return answer

    def test_a_dead_link_is_reopened_before_the_next_pass(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        link = _AmsFlashScriptedLink(self._loader(list(self.GOOD_PASS), events))
        link.dead = True

        def reconnect() -> None:
            events.append("reconnect")
            link.dead = False

        art = _ams_flash_artifact(0x0700, 0x00)
        assert ams_flash.do_flash(link, art, reconnect=reconnect) == 0
        assert events == ["reconnect"] + self.GOOD_PASS_SENDS
        assert link.dead is False
        assert capsys.readouterr() == (
            self.OPENING + "    link reopened\n" + self.PASSED, "")

    def test_a_live_link_is_not_reopened(self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        link = _AmsFlashScriptedLink(self._loader(list(self.GOOD_PASS), events))
        art = _ams_flash_artifact(0x0700, 0x00)
        assert ams_flash.do_flash(
            link, art, reconnect=lambda: events.append("reconnect")) == 0
        assert events == self.GOOD_PASS_SENDS
        assert capsys.readouterr() == (self.OPENING + self.PASSED, "")

    def test_a_dead_link_with_no_way_back_is_used_as_it_is(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        link = _AmsFlashScriptedLink(self._loader(list(self.GOOD_PASS), events))
        link.dead = True
        assert ams_flash.do_flash(link, _ams_flash_artifact(0x0700, 0x00)) == 0
        assert events == self.GOOD_PASS_SENDS
        assert link.dead is True
        assert capsys.readouterr() == (self.OPENING + self.PASSED, "")

    def test_a_link_that_does_not_come_back_ends_the_flash(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        link = _AmsFlashScriptedLink(self._loader(list(self.GOOD_PASS), events))
        link.dead = True

        def reconnect() -> None:
            events.append("reconnect")
            raise IOError("port gone")

        assert ams_flash.do_flash(link, _ams_flash_artifact(0x0700, 0x00),
                                  reconnect=reconnect) == 1
        assert events == ["reconnect"]                   # no pass on a dead link
        assert link.dead is True
        assert capsys.readouterr() == (
            self.OPENING,
            "    the bridge link is gone and did not come back (port gone)\n"
            "    out of attempts: the loader never verified success!. The unit is left "
            "in its loader (recoverable) -- do not power it off; run the same command "
            "with MODE=go again. If every pass acks but reports chunk hash error, the "
            "bus/link is dropping bytes.\n")


class TestBridgeKeys:
    """bridge_keys: every Bambu bridge in the live config, with its link key."""

    def test_the_bridgebox_sections_own_key_wins(self) -> None:
        settings = {"afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                                 "tcp_key": "stale"},
                    "afc_bridgebox chain1": {"serial_port": "tcp://h:8888",
                                             "tcp_key": "real"}}
        assert ams_flash.bridge_keys(settings) == {"tcp://h:8888": "real"}

    def test_a_bridgebox_section_without_a_key_keeps_the_units_key(self) -> None:
        settings = {"afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                                 "tcp_key": "k"},
                    "afc_bridgebox chain1": {"serial_port": "tcp://h:8888"}}
        assert ams_flash.bridge_keys(settings) == {"tcp://h:8888": "k"}


class TestBridgeFromSettings:
    """bridge_from_settings: the one bridge in the live config."""

    def test_the_bridge_comes_from_the_live_config(self) -> None:
        settings = {
            "afc_bridgebox chain1": {"serial_port": "/dev/serial/by-id/usb-Pico",
                                     "tcp_key": None},
            "mcu": {"serial": "/dev/ttyACM0"},
        }
        assert ams_flash.bridge_from_settings(settings) == (
            "/dev/serial/by-id/usb-Pico", None)
        settings = {"afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                                 "tcp_key": "k"}}
        assert ams_flash.bridge_from_settings(settings) == ("tcp://h:8888", "k")
        assert ams_flash.bridge_from_settings({"mcu": {}}) == (None, None)

    def test_a_missing_key_published_as_the_string_none_is_no_key(self) -> None:
        # AFC_BridgeBox writes str(None) into the unit sections it builds, and
        # Klipper publishes that. Sending "None" as a key fails the auth of a
        # keyless bridge after Klipper has already been stopped.
        settings = {"afc_bridgebox chain1": {"serial_port": "tcp://h:8888"},
                    "afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                                 "tcp_key": "None"}}
        assert ams_flash.bridge_from_settings(settings) == ("tcp://h:8888", None)

    def test_two_bridges_in_the_config_do_not_guess(self) -> None:
        settings = {"afc_bridgebox a": {"serial_port": "tcp://a:8888"},
                    "afc_bridgebox b": {"serial_port": "/dev/ttyACM1"}}
        with pytest.raises(ValueError) as raised:
            ams_flash.bridge_from_settings(settings)
        assert str(raised.value) == (
            "more than one bridge in the config (/dev/ttyACM1, tcp://a:8888); name the "
            "one to use with TARGET=<serial_port> on the command")


class TestUnitRows:
    """unit_rows: every AFC_BambuAMS unit Klipper knows, read through Moonraker."""

    class _Moonraker:
        """Moonraker's object list and status queries; an Exception as a unit's
        status is raised by the query."""

        def __init__(self, status: Dict[str, Any]) -> None:
            self.status = status

        def objects(self) -> List[str]:
            return [f"AFC_BambuAMS {name}" for name in self.status] + ["toolhead"]

        def q(self, obj: str) -> Any:
            value = self.status[obj.split(" ", 1)[1]]
            if isinstance(value, Exception):
                raise value
            return value

    def test_a_unit_that_cannot_be_read_stops_the_count(self) -> None:
        # Dropping it would let a second unit that is online pass the one-unit
        # rule because of a network hiccup.
        moon = self._Moonraker({
            "Bambu_AMS_1": {"bridge_online": True, "ams_model": "ams2", "ams_index": 0},
            "Bambu_AMS_HT_1": IOError("timed out")})
        with pytest.raises(IOError) as raised:
            ams_flash.unit_rows(moon)
        assert str(raised.value) == "timed out"
        moon.status["Bambu_AMS_HT_1"] = {}
        with pytest.raises(IOError) as raised:
            ams_flash.unit_rows(moon)
        assert str(raised.value) == "no status for AFC_BambuAMS Bambu_AMS_HT_1"
        moon.status["Bambu_AMS_HT_1"] = {"bridge_online": False, "ams_model": "ht",
                                         "ams_index": 4}
        assert ams_flash.unit_rows(moon) == [
            {"name": "Bambu_AMS_1", "online": True, "model": "ams2", "index": 0,
             "fw": None},
            {"name": "Bambu_AMS_HT_1", "online": False, "model": "ht", "index": 4,
             "fw": None}]

    def test_a_status_that_is_not_a_record_stops_the_count(self) -> None:
        moon = self._Moonraker({"Bambu_AMS_1": ["online"]})
        with pytest.raises(IOError) as raised:
            ams_flash.unit_rows(moon)
        assert str(raised.value) == "no status for AFC_BambuAMS Bambu_AMS_1"

    def test_each_row_is_read_as_klipper_publishes_it(self) -> None:
        moon = self._Moonraker({"Bambu_AMS_1": {
            "bridge_online": 1, "ams_model": " AMS2 ", "ams_index": "0",
            "bridge_fw": "AFC-2.81"}})
        assert ams_flash.unit_rows(moon) == [
            {"name": "Bambu_AMS_1", "online": True, "model": "ams2", "index": None,
             "fw": "AFC-2.81"}]


class TestPickTarget:
    """pick_target: the one unit to flash, or why there is none."""

    @staticmethod
    def _row(name: str, online: bool, model: str,
             index: Optional[int] = None) -> Dict[str, Any]:
        """A unit row as unit_rows builds it; an HT defaults to chain index 4."""
        if index is None:
            index = 4 if model == "ht" else 0
        return {"name": name, "online": online, "model": model, "index": index,
                "fw": "AFC-2.81"}

    def test_the_one_unit_online_is_the_target(self) -> None:
        rows = [self._row("Bambu_AMS_1", True, "ams2"),
                self._row("Bambu_AMS_HT_1", False, "ht")]
        assert ams_flash.pick_target(rows, "ams2", 0x00) == ("Bambu_AMS_1", None, False)
        rows = [self._row("Bambu_AMS_1", False, "ams2"),
                self._row("Bambu_AMS_HT_1", True, "ht")]
        assert ams_flash.pick_target(rows, "ht", 0x80) == ("Bambu_AMS_HT_1", None, False)

    @pytest.mark.parametrize("model,ams_id", [("ams2", 0x00), ("ht", 0x80)])
    def test_a_second_unit_online_refuses_whatever_its_model(
            self, model: str, ams_id: int) -> None:
        rows = [self._row("Bambu_AMS_1", True, "ams2"),
                self._row("Bambu_AMS_HT_1", True, "ht")]
        assert ams_flash.pick_target(rows, model, ams_id) == (
            None, "need exactly one AMS on the bus, found 2 online (Bambu_AMS_1, "
                  "Bambu_AMS_HT_1). Unplug the others and retry.", False)

    def test_no_unit_online_refuses(self) -> None:
        rows = [self._row("Bambu_AMS_1", False, "ams2")]
        assert ams_flash.pick_target(rows, "ams2", 0x00) == (
            None, "need exactly one AMS on the bus, found 0 online (none). Unplug the "
                  "others and retry.", False)

    @pytest.mark.parametrize("cmd,ams_id,index,unit_model,shown,name", [
        ("ams2", 0x00, 0, "ams1", "ams1", "AMS 2 Pro"),
        ("ams1", 0x00, 0, "ams2", "ams2", "AMS 1"),
        ("ht", 0x80, 4, "ams2", "ams2", "AMS HT"),
        ("ams2", 0x00, 0, "ht", "ht", "AMS 2 Pro"),
        ("ams1", 0x00, 0, "", "?", "AMS 1"),
        ("ht", 0x80, 4, "boxed", "boxed", "AMS HT"),
    ])
    def test_the_online_unit_must_be_the_commands_model(
            self, cmd: str, ams_id: int, index: int, unit_model: str, shown: str,
            name: str) -> None:
        rows = [self._row("Bambu_AMS_1", True, unit_model, index)]
        assert ams_flash.pick_target(rows, cmd, ams_id) == (
            None,
            f"Bambu_AMS_1 is configured as ams_model '{shown}', but this command is "
            f"for the {name} ({cmd}). Use that model's command, or fix ams_model: if "
            f"the config is wrong.",
            False)

    @pytest.mark.parametrize("cmd", ["ams1", "ams2"])
    def test_an_unconfirmed_boxed_unit_is_left_to_the_bus_check(self, cmd: str) -> None:
        # BridgeBox calls a unit "boxed" until the bus settles AMS 1 vs AMS 2,
        # and there is no ams_model line to fix. Step 5 asks the unit itself.
        rows = [self._row("Bambu_AMS_1", True, "boxed")]
        assert ams_flash.pick_target(rows, cmd, 0x00) == ("Bambu_AMS_1", None, False)

    def test_a_unit_at_another_bus_address_is_refused(self) -> None:
        # A boxed image is addressed to id 0x00. A unit enrolled second (index
        # 1, id 0x01) would never see the cmd1, or another unit at 0x00 would.
        rows = [self._row("Bambu_AMS_1", True, "ams2", 1)]
        assert ams_flash.pick_target(rows, "ams2", 0x00) == (
            None, "Bambu_AMS_1 is enrolled at chain index 1 (AMS id 0x01), but the "
                  "image is addressed to AMS id 0x00. Power the bridge off and on with "
                  "only this unit connected so it enrolls first, restart Klipper, and "
                  "retry.", False)
        rows = [self._row("Bambu_AMS_HT_1", True, "ht", 5)]
        assert ams_flash.pick_target(rows, "ht", 0x80) == (
            None, "Bambu_AMS_HT_1 is enrolled at chain index 5 (AMS id 0x81), but the "
                  "image is addressed to AMS id 0x80. Power the bridge off and on with "
                  "only this unit connected so it enrolls first, restart Klipper, and "
                  "retry.", False)

    def test_a_unit_with_no_known_index_is_left_to_the_bus_check(self) -> None:
        rows = [dict(self._row("Bambu_AMS_1", True, "ams2"), index=None)]
        assert ams_flash.pick_target(rows, "ams2", 0x00) == ("Bambu_AMS_1", None, False)

    def test_a_unit_left_in_its_loader_is_resumed_by_its_own_command(self) -> None:
        # A unit in its bootloader never shows online, so without the record a
        # stopped update could never be finished from the console.
        state = {"model": "ams2", "unit": "Bambu_AMS_1", "ams_id": 0x00,
                 "when": "2026-09-26 20:00"}
        rows = [self._row("Bambu_AMS_1", False, "ams2")]
        assert ams_flash.pick_target(rows, "ams2", 0x00, state) == (
            "Bambu_AMS_1", None, True)
        assert ams_flash.pick_target(rows, "ams1", 0x00, state) == (
            None, "a unit is waiting in its bootloader from an unfinished AMS 2 Pro "
                  "update (Bambu_AMS_1, 2026-09-26 20:00). Finish it with the AMS 2 Pro "
                  "command.", False)
        assert ams_flash.pick_target(rows, "ht", 0x80, dict(state, model="ht")) == (
            None, "the unit waiting in its bootloader was sent there for a different "
                  "bus address than this image; run the command with the artifact it "
                  "was started with", False)
        # A unit online again means it was power-cycled out of the loader: the
        # ordinary rules apply and the record is ignored.
        rows = [self._row("Bambu_AMS_1", True, "ams2")]
        assert ams_flash.pick_target(rows, "ams2", 0x00, state) == (
            "Bambu_AMS_1", None, False)

    def test_a_record_naming_no_known_model_or_unit_is_still_honoured(self) -> None:
        rows = [self._row("Bambu_AMS_1", False, "ams2")]
        state = {"model": "x1", "unit": "Bambu_AMS_1", "ams_id": 0x00, "when": "then"}
        assert ams_flash.pick_target(rows, "ams2", 0x00, state) == (
            None, "a unit is waiting in its bootloader from an unfinished x1 update "
                  "(Bambu_AMS_1, then). Finish it with the x1 command.", False)
        state = {"model": "ams2", "ams_id": 0x00}
        assert ams_flash.pick_target(rows, "ams2", 0x00, state) == ("?", None, True)


class TestReadState:
    """read_state: the record of a unit a previous run left in its loader."""

    def test_the_state_record_round_trips(self, tmp_path: Path,
                                          capsys: pytest.CaptureFixture[str]) -> None:
        path = str(tmp_path / "ams_flash_state.json")
        assert ams_flash.read_state(path) is None
        assert ams_flash.write_state(path, model="ht", unit="Bambu_AMS_HT_1",
                                     ams_id=0x80, when="t") is True
        assert ams_flash.read_state(path) == {"model": "ht", "unit": "Bambu_AMS_HT_1",
                                              "ams_id": 128, "when": "t"}
        ams_flash.clear_state(path)
        assert ams_flash.read_state(path) is None
        (tmp_path / "ams_flash_state.json").write_text("not json")
        assert ams_flash.read_state(path) is None
        assert capsys.readouterr() == ("", "")

    @pytest.mark.parametrize("text", ['["ams2"]', '{"unit": "Bambu_AMS_1"}'],
                             ids=["not-a-record", "no-model"])
    def test_a_record_without_a_model_is_no_record(self, tmp_path: Path, text: str) -> None:
        path = tmp_path / "ams_flash_state.json"
        path.write_text(text)
        assert ams_flash.read_state(str(path)) is None


class TestMain:
    """main() end to end, against a scripted bridge and Moonraker.

    The updater runs from its own file as if it lived in the test's config
    directory, where it keeps its state file and finds its artifacts. The
    bridge link, Moonraker and the flash itself are scripted; every check up
    to the flash is the real one.
    """

    FW = "[3] bridge fw >= 175\n    bridge_fw = 281\n"
    BUS = ("[5] the bus: one unit answering, at the image's address, and it is "
           "this model\n")
    AMS2_IMAGE = (
        "[2] artifact verifies\n"
        "    OK: header + 168 blocks\n"
        "    image: header declares 172132 bytes, blocks carry 171716 + 416 container "
        "header\n"
        "    n3f_rev5-firmware-v05.00.22.22-20260702164346.bin.sig\n"
        "    addressed to device 0x0700, AMS id 0x00\n")
    AMS1_IMAGE = (
        "[2] artifact verifies\n"
        "    OK: header + 3 blocks\n"
        "    image: header declares 536 bytes, blocks carry 120 + 416 container header\n"
        "    ams_rev8-firmware-v01.00.06.87-x.bin.sig\n"
        "    addressed to device 0x0700, AMS id 0x00\n")
    HT_IMAGE = (
        "[2] artifact verifies\n"
        "    OK: header + 157 blocks\n"
        "    image: header declares 160552 bytes, blocks carry 160136 + 416 container "
        "header\n"
        "    n3s_rev5-firmware-v05.00.22.19-20260616201708.bin.sig\n"
        "    addressed to device 0x1800, AMS id 0x80\n")
    AMS2_UNIT = ("[4] exactly one AMS online, and it is an AMS 2 Pro\n"
                 "    Bambu_AMS_1: online  ams_model=ams2  index=0\n"
                 "    target unit: Bambu_AMS_1  (label: Bambu_AMS_1)\n")
    BOXED_AMS2_UNIT = ("[4] exactly one AMS online, and it is an AMS 2 Pro\n"
                       "    Bambu_AMS_1: online  ams_model=boxed  index=0\n"
                       "    target unit: Bambu_AMS_1  (label: Bambu_AMS_1)\n")
    BOXED_AMS1_UNIT = ("[4] exactly one AMS online, and it is an AMS 1\n"
                       "    Bambu_AMS_1: online  ams_model=boxed  index=0\n"
                       "    target unit: Bambu_AMS_1  (label: Bambu_AMS_1)\n")
    HT_UNIT = ("[4] exactly one AMS online, and it is an AMS HT\n"
               "    Bambu_AMS_HT_1: online  ams_model=ht  index=4\n"
               "    target unit: Bambu_AMS_HT_1  (label: Bambu_AMS_HT_1)\n")
    ONE_ON_THE_BUS = "    answering on the bus: index 0 (AMS id 0x00)\n"
    FLASHED = ("[8] unit reboots and comes back online\n"
               "    Bambu_AMS_1 ONLINE on the new firmware.\n"
               "\n"
               "==> DONE. Start the dryer and load a tray to confirm.\n")
    RECORD = {"model": "ams2", "unit": "Bambu_AMS_1", "ams_id": 0, "when": "earlier"}

    class _Moonraker:
        """Moonraker as main() uses it: the print state, the AMS unit objects,
        the live config, and a Klipper service that stops and starts."""

        def __init__(self, units: Dict[str, Dict[str, Any]]) -> None:
            self.units = units
            self.actions: List[str] = []

        def q(self, obj: str) -> Dict[str, Any]:
            if obj == "print_stats":
                return {"state": "standby"}
            return self.units[obj.split(" ", 1)[1]]

        def objects(self) -> List[str]:
            return [f"AFC_BambuAMS {name}" for name in self.units]

        def settings(self) -> Dict[str, Any]:
            return {}

        def service(self, action: str) -> bool:
            self.actions.append(action)
            return True

        def state(self) -> str:
            return "?" if self.actions[-1:] == ["stop"] else "ready"

    class _BusBridge:
        """One boxed unit at index 0 behind a bridge that behaves like the
        firmware where it matters here: online flags are only refreshed while
        the bridge polls, and neither fwreplay nor sniff polls."""

        def __init__(self, model: str, state: str = "app", fwreplay: bool = False,
                     frozen: Optional[List[int]] = None, drain: bool = True,
                     dryrem: int = 0) -> None:
            self.model, self.state = model, state        # state: app or loader
            self.fwreplay, self.sniff = fwreplay, False
            self.frozen = frozen                         # flags held since polling stopped
            self.drain, self.dryrem = drain, dryrem
            self.sent: List[Dict[str, Any]] = []
            self.queue: List[str] = []
            self.dead = False
            self.closed = False

        def _online(self) -> List[int]:
            if self.fwreplay or self.sniff:
                return list(self.frozen or [])
            return [0] if self.state == "app" else []

        def send(self, obj: Dict[str, Any]) -> None:
            self.sent.append(obj)
            cmd = obj["cmd"]
            if cmd == "fwreplay":
                self.fwreplay = bool(obj["on"])
                if not self.fwreplay:
                    self.frozen = None
                self.queue.append(f'{{"evt":"fwreplay","on":{json.dumps(self.fwreplay)}}}')
            elif cmd == "sniff":
                if obj["on"]:
                    self.frozen = self._online()
                self.sniff = bool(obj["on"])
                self.queue.append(f'{{"evt":"sniff_mode","on":{json.dumps(self.sniff)}}}')
            elif cmd == "status":
                units = [{"n": n, "online": True, "dryrem": self.dryrem}
                         for n in self._online()]
                self.queue.append(json.dumps({"evt": "status", "units": units}))
            elif cmd == "raw":
                tx = bytes.fromhex(obj["hex"])
                rx = tx
                op = tx[11:13]
                if self.state == "app" and op == b"\x37\x02" and self.model == "ams2":
                    rx += bytes.fromhex(ANSWER_3702_ID0)
                elif self.state == "app" and op == b"\x1a\x02" and self.drain:
                    rx += bytes.fromhex(DRAIN_ANSWER_ID0)
                elif op == b"\x06\x01":
                    if self.state == "app":
                        self.state = "loader"            # it jumps; the banner names no unit
                        rx += b"[MCU_UP] Loader Version: 23"
                    else:
                        rx += b"[MCU_UP] 0 resev cmd 0x1"
                self.queue.append(json.dumps(
                    {"evt": "raw", "tx": len(tx), "rx": rx.hex().upper()}))

        def lines(self) -> Iterator[Optional[str]]:
            while True:
                yield self.queue.pop(0) if self.queue else None

        def close(self) -> None:
            self.closed = True

    class _Harness:
        """Runs main() from a config directory of its own, on scripted parts."""

        def __init__(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
                     capsys: pytest.CaptureFixture[str]) -> None:
            self.cfg = tmp_path / "config"
            self.cfg.mkdir()
            (self.cfg / "printer.cfg").write_text("")
            for name in ("ams2_artifact.json", "ht_artifact.json"):
                (self.cfg / name).symlink_to(AMS_FLASH_DIR / name)
            self.ams1_header = TestMain._write_ams1_artifact(self.cfg / "ams1_artifact.json")
            self.port = str(tmp_path / "fakeport")
            Path(self.port).write_text("")
            self.state_path = self.cfg / "ams_flash_state.json"
            self.log_path = tmp_path / "logs" / "ams_flash.log"
            self._monkeypatch = monkeypatch
            self._capsys = capsys
            monkeypatch.setattr(ams_flash, "__file__", str(self.cfg / "ams_flash.py"))

        def read_state(self) -> Optional[Dict[str, Any]]:
            if not self.state_path.exists():
                return None
            return json.loads(self.state_path.read_text())

        def run(self, model: str, bridge: TestMain._BusBridge,
                units: Dict[str, Dict[str, Any]], state: Optional[Dict[str, Any]] = None,
                sleep: Optional[Callable[[float], None]] = None, mode: str = "go") -> Any:
            """Run main() once and keep what it did.

            :param model: the --model the command passes
            :param bridge: the link main() opens
            :param units: Klipper's AFC_BambuAMS status, by unit name
            :param state: a record a previous run left, if any
            :param sleep: what the updater's sleeps do, by default nothing
            :param mode: the --mode the command passes
            :return Any: main()'s exit code, or "exit: <reason>" when it was stopped
            """
            if state is not None:
                self.state_path.write_text(json.dumps(state))
            self.moon = TestMain._Moonraker(units)
            self.opened: List[str] = []
            self.flashed: List[Tuple[Any, Dict[str, Any], Optional[Dict[str, Any]]]] = []

            def open_link(target: str, timeout: float = 1.0) -> TestMain._BusBridge:
                self.opened.append(target)
                return bridge

            def flash(link: Any, art: Dict[str, Any], retries: int = 6,
                      reconnect: Optional[Callable[[], None]] = None) -> int:
                self.flashed.append((link, art, self.read_state()))
                for unit in units.values():              # it reboots into its app
                    unit["bridge_online"] = True
                return 0

            patch = self._monkeypatch
            patch.setattr(ams_flash, "time", _AmsFlashClock(step=0.25, sleep=sleep))
            patch.setattr(ams_flash, "Moon", lambda: self.moon)
            patch.setattr(ams_flash, "Link", open_link)
            patch.setattr(ams_flash, "do_flash", flash)
            patch.setattr(sys, "argv", [
                "ams_flash.py", "--detached", "--cfg-dir", str(self.cfg), "--target",
                self.port, "--model", model, "--mode", mode])
            logged = self.log_path.stat().st_size if self.log_path.exists() else 0
            handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP)}
            streams = sys.stdout, sys.stderr
            try:
                rc: Any = ams_flash.main()
            except SystemExit as stop:
                rc = f"exit: {stop}"
            finally:
                sys.stdout, sys.stderr = streams         # main() tees both into its log
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)
            self.out, self.err = self._capsys.readouterr()
            self.log = self.log_path.read_text()[logged:]
            self.state = self.read_state()
            return rc

    @staticmethod
    def _write_ams1_artifact(path: Path) -> bytes:
        """Write an AMS 1 artifact that passes step 2, and return its header.

        It is addressed to a lone boxed unit, named for an AMS 1, and its three
        40-byte blocks add up to the image the header declares.
        """
        def body(cmd: int) -> bytearray:
            frame = bytearray(40)
            frame[0:2] = b"\x00\x07"                     # device 0x0700
            frame[2:4] = b"\x00\x09"                     # source: the updater
            frame[4:7] = bytes((0x06, 0x01, cmd))
            frame[7] = 0x07                              # device class
            return frame

        header = body(0x02)
        header[16:19] = (3 * 40 + 416).to_bytes(3, "little")
        header += (b"BIMH" + bytes(44) + b"ams_rev8-firmware-v01.00.06.87-x.bin.sig"
                   + bytes(8))
        block = body(0x03)
        block[28:32] = (40).to_bytes(4, "little")
        path.write_text(json.dumps({"header": header.hex(), "blocks": [block.hex()] * 3}))
        return bytes(header)

    @staticmethod
    def _shipped_header(name: str) -> bytes:
        return bytes.fromhex(json.loads((AMS_FLASH_DIR / name).read_text())["header"])

    @staticmethod
    def _opening(name: str, model: str, port: str, mode: str = "GO") -> str:
        """The run's banner and step 1, as main() prints them."""
        return ("\n===== 2026-10-05 09:30:00 =====\n"
                f"=== AMS update: {name} ({model}) label=(auto) mode={mode} "
                f"bridge={port} ===\n"
                "[1] printer not printing\n"
                "    state = standby\n")

    @staticmethod
    def _units(online: bool, model: str = "boxed") -> Dict[str, Dict[str, Any]]:
        return {"Bambu_AMS_1": {"bridge_online": online, "ams_model": model,
                                "ams_index": 0, "bridge_fw": "AFC-2.81"}}

    @staticmethod
    def _record(model: str) -> Dict[str, Any]:
        """The record main() writes before it sends Bambu_AMS_1 into its loader."""
        return {"model": model, "unit": "Bambu_AMS_1", "ams_id": 0,
                "artifact": f"{model}_artifact.json", "when": "2026-10-05 09:30"}

    @pytest.fixture
    def main_run(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
                 capsys: pytest.CaptureFixture[str]) -> TestMain._Harness:
        return TestMain._Harness(tmp_path, monkeypatch, capsys)

    def test_main_updates_a_lone_ams2(self, main_run: TestMain._Harness) -> None:
        bridge = self._BusBridge("ams2")
        assert main_run.run("ams2", bridge, self._units(True, "ams2")) == 0
        expected = (self._opening("AMS 2 Pro", "ams2", main_run.port) + self.AMS2_IMAGE
                    + self.FW + self.AMS2_UNIT + self.BUS + self.ONE_ON_THE_BUS
                    + "    generation query: answered 8 of 8; log drain: answered 8 of 8\n"
                    + "    OK\n"
                    + "[6] cmd1 enters the loader (non-destructive)\n"
                    + "    loader confirmed after cmd1 #2\n"
                    + "[7] FLASH: header (erase) + 168 blocks\n"
                    + self.FLASHED)
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        assert main_run.moon.actions == ["stop", "start"]
        assert main_run.opened == [main_run.port]
        [(link, art, recorded)] = main_run.flashed
        assert link is bridge
        assert art["header"] == self._shipped_header("ams2_artifact.json")
        assert recorded == self._record("ams2")          # recorded before the loader
        assert main_run.state is None                    # cleared on success
        assert bridge.closed is True

    def test_main_refuses_the_ams2_image_for_an_ams1(self, main_run: TestMain._Harness) -> None:
        bridge = self._BusBridge("ams1")
        assert main_run.run("ams2", bridge, self._units(True, "ams2")) == 1
        expected = (self._opening("AMS 2 Pro", "ams2", main_run.port) + self.AMS2_IMAGE
                    + self.FW + self.AMS2_UNIT + self.BUS + self.ONE_ON_THE_BUS
                    + "    generation query: answered 0 of 8; log drain: answered 8 of 8\n"
                    + "    ABORT: the unit answers as an AMS 1. Use the AMS 1 command.\n")
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        assert main_run.flashed == []
        assert main_run.moon.actions == ["stop", "start"]
        assert bridge.state == "app"                     # no cmd1 went out
        assert main_run.state is None
        # Released on the live link, then closed.
        assert bridge.sent[-2:] == [{"cmd": "fwreplay", "on": 0}, {"cmd": "sniff", "on": 0}]
        assert bridge.closed is True

    def test_main_does_not_call_a_silent_unit_an_ams1(self, main_run: TestMain._Harness) -> None:
        unreliable = ("    ABORT: the unit is not answering the bus reliably, so its "
                      "generation cannot be told. Retry; if it keeps happening, check the "
                      "bus wiring.\n")
        head = (self._opening("AMS 1", "ams1", main_run.port) + self.AMS1_IMAGE + self.FW
                + self.BOXED_AMS1_UNIT + self.BUS + self.ONE_ON_THE_BUS)
        # Deaf to the drain: its answers to the generation query cannot be weighed.
        deaf = self._BusBridge("ams2", drain=False)
        assert main_run.run("ams1", deaf, self._units(True)) == 1
        expected = (head
                    + "    generation query: answered 8 of 8; log drain: answered 0 of 8\n"
                    + unreliable)
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        assert main_run.moon.actions == ["stop", "start"]
        assert deaf.closed is True
        # Silent to both: no answer to 0x3702 alone must not make it an AMS 1.
        silent = self._BusBridge("ams1", drain=False)
        assert main_run.run("ams1", silent, self._units(True)) == 1
        expected = (head
                    + "    generation query: answered 0 of 8; log drain: answered 0 of 8\n"
                    + unreliable)
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        assert main_run.flashed == []
        assert main_run.moon.actions == ["stop", "start"]
        assert silent.closed is True

    def test_main_resets_a_bridge_left_in_fwreplay(self, main_run: TestMain._Harness) -> None:
        # A run died in fwreplay with an AMS 2 in its loader, so the bridge
        # still reports it online. Silent to 0x3702, it used to read as an
        # AMS 1 and take the AMS 1 image.
        stuck = self._BusBridge("ams2", state="loader", fwreplay=True, frozen=[0])
        assert main_run.run("ams1", stuck, self._units(True), state=self.RECORD) == 1
        expected = (self._opening("AMS 1", "ams1", main_run.port) + self.AMS1_IMAGE
                    + self.FW + self.BOXED_AMS1_UNIT + self.BUS
                    + "    answering on the bus: none\n"
                    + "    ABORT: need exactly one AMS on the bus, found 0 answering. Unplug "
                    + "the others (the bridge counts units Klipper has no section for too) "
                    + "and retry.\n")
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        off = [{"cmd": "fwreplay", "on": 0}, {"cmd": "sniff", "on": 0}]
        assert stuck.sent == off + [{"cmd": "status"}] + off
        assert main_run.flashed == []
        assert main_run.state == self.RECORD             # left for the AMS 2's own command
        assert main_run.moon.actions == ["stop", "start"]
        assert stuck.closed is True
        # The same bridge, the AMS 2's own command: the unit is resumed.
        stuck = self._BusBridge("ams2", state="loader", fwreplay=True, frozen=[0])
        assert main_run.run("ams2", stuck, self._units(True), state=self.RECORD) == 0
        expected = (self._opening("AMS 2 Pro", "ams2", main_run.port) + self.AMS2_IMAGE
                    + self.FW + self.BOXED_AMS2_UNIT + self.BUS
                    + "    answering on the bus: none\n"
                    + "    nothing answers, but an earlier run (earlier) left Bambu_AMS_1 in "
                    + "its bootloader: resuming it\n"
                    + "    OK\n"
                    + "[6] cmd1 enters the loader (non-destructive)\n"
                    + "    loader confirmed after cmd1 #1\n"
                    + "[7] FLASH: header (erase) + 168 blocks\n"
                    + self.FLASHED)
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        [(_link, art, recorded)] = main_run.flashed
        assert art["header"] == self._shipped_header("ams2_artifact.json")
        assert recorded == self._record("ams2")
        assert main_run.state is None
        assert main_run.moon.actions == ["stop", "start"]
        assert stuck.closed is True

    def test_main_resumes_a_unit_left_in_its_loader(self, main_run: TestMain._Harness) -> None:
        bridge = self._BusBridge("ams2", state="loader")
        assert main_run.run("ams2", bridge, self._units(False), state=self.RECORD) == 0
        expected = (self._opening("AMS 2 Pro", "ams2", main_run.port) + self.AMS2_IMAGE
                    + self.FW
                    + "[4] exactly one AMS online, and it is an AMS 2 Pro\n"
                    + "    Bambu_AMS_1: offline  ams_model=boxed  index=0\n"
                    + "    resuming Bambu_AMS_1: an earlier run (earlier) left it waiting "
                    + "in its bootloader\n"
                    + "    target unit: Bambu_AMS_1  (label: Bambu_AMS_1)\n"
                    + self.BUS
                    + "    answering on the bus: none\n"
                    + "    OK\n"
                    + "[6] cmd1 enters the loader (non-destructive)\n"
                    + "    loader confirmed after cmd1 #1\n"
                    + "[7] FLASH: header (erase) + 168 blocks\n"
                    + self.FLASHED)
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        [(_link, art, recorded)] = main_run.flashed
        assert art["header"] == self._shipped_header("ams2_artifact.json")
        assert recorded == self._record("ams2")
        assert main_run.state is None
        assert main_run.moon.actions == ["stop", "start"]
        assert bridge.closed is True

    def test_main_clears_a_stale_record_when_the_unit_runs_again(
            self, main_run: TestMain._Harness) -> None:
        # Preflighted, then power-cycled back to normal: the record must not
        # outlive it.
        head = (self._opening("AMS 1", "ams1", main_run.port) + self.AMS1_IMAGE + self.FW
                + self.BOXED_AMS1_UNIT + self.BUS + self.ONE_ON_THE_BUS)
        bridge = self._BusBridge("ams1")
        assert main_run.run("ams1", bridge, self._units(True), state=self.RECORD) == 0
        expected = (head
                    + "    generation query: answered 0 of 8; log drain: answered 8 of 8\n"
                    + "    OK\n"
                    + "[6] cmd1 enters the loader (non-destructive)\n"
                    + "    loader confirmed after cmd1 #2\n"
                    + "[7] FLASH: header (erase) + 3 blocks\n"
                    + self.FLASHED)
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        [(_link, art, recorded)] = main_run.flashed
        assert art["header"] == main_run.ams1_header
        assert recorded == self._record("ams1")
        assert main_run.state is None
        assert main_run.moon.actions == ["stop", "start"]
        assert bridge.closed is True
        # The record goes as soon as the unit is seen running, even when a
        # later check stops the run before anything else is recorded.
        bridge = self._BusBridge("ams2")
        assert main_run.run("ams1", bridge, self._units(True), state=self.RECORD) == 1
        expected = (head
                    + "    generation query: answered 8 of 8; log drain: answered 8 of 8\n"
                    + "    ABORT: the unit answers as an AMS 2 Pro. Use the AMS 2 Pro "
                    + "command.\n")
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        assert main_run.flashed == []
        assert main_run.state is None
        assert main_run.moon.actions == ["stop", "start"]
        assert bridge.closed is True

    def test_main_refuses_during_a_dry(self, main_run: TestMain._Harness) -> None:
        bridge = self._BusBridge("ams2", dryrem=600)
        assert main_run.run("ams2", bridge, self._units(True, "ams2")) == 1
        expected = (self._opening("AMS 2 Pro", "ams2", main_run.port) + self.AMS2_IMAGE
                    + self.FW + self.AMS2_UNIT + self.BUS + self.ONE_ON_THE_BUS
                    + "    ABORT: a dry cycle is running. Stop it and retry.\n")
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        off = [{"cmd": "fwreplay", "on": 0}, {"cmd": "sniff", "on": 0}]
        assert bridge.sent == off + [{"cmd": "status"}] + off
        assert main_run.flashed == []
        assert main_run.moon.actions == ["stop", "start"]
        assert bridge.closed is True

    def test_main_does_not_send_an_unrecorded_unit_into_its_loader(
            self, main_run: TestMain._Harness) -> None:
        tmp = f"{main_run.state_path}.tmp"
        Path(tmp).mkdir()                                # the record cannot be written
        bridge = self._BusBridge("ams2")
        assert main_run.run("ams2", bridge, self._units(True, "ams2")) == 1
        assert bridge.state == "app"                     # no cmd1 went out
        head = (self._opening("AMS 2 Pro", "ams2", main_run.port) + self.AMS2_IMAGE
                + self.FW + self.AMS2_UNIT + self.BUS + self.ONE_ON_THE_BUS
                + "    generation query: answered 8 of 8; log drain: answered 8 of 8\n"
                + "    OK\n")
        refused = (f"    cannot record the unit in {main_run.state_path} ([Errno 21] Is a "
                   f"directory: '{tmp}')\n")
        tail = ("    ABORT: without that record a stopped update could not be resumed, so "
                "the unit is not sent into its loader.\n")
        assert (main_run.out, main_run.err, main_run.log) == (
            head + tail, refused, head + refused + tail)
        assert main_run.flashed == []
        assert main_run.state is None
        assert main_run.moon.actions == ["stop", "start"]
        assert bridge.closed is True

    def test_main_restarts_klipper_when_stopped_right_after_the_stop(
            self, main_run: TestMain._Harness) -> None:
        unhandled = signal.getsignal(signal.SIGTERM)

        def stopped(_seconds: float) -> None:
            # systemd stopping the unit: streams flushed, then a SIGTERM that
            # main() itself handles.
            assert signal.getsignal(signal.SIGTERM) is not unhandled
            sys.stdout.flush()
            sys.stderr.flush()
            assert main_run.log_path.read_text().endswith(self.BUS)
            os.kill(os.getpid(), signal.SIGTERM)

        bridge = self._BusBridge("ams2")
        assert main_run.run("ams2", bridge, self._units(True, "ams2"),
                            sleep=stopped) == "exit: stopped by signal 15"
        assert main_run.moon.actions == ["stop", "start"]
        assert main_run.opened == []                     # stopped before the link opened
        expected = (self._opening("AMS 2 Pro", "ams2", main_run.port) + self.AMS2_IMAGE
                    + self.FW + self.AMS2_UNIT + self.BUS)
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        assert main_run.flashed == []

    @pytest.mark.parametrize("model,name,image,units,unit_check", [
        ("ams1", "AMS 1", AMS1_IMAGE,
         {"Bambu_AMS_1": {"bridge_online": True, "ams_model": "boxed", "ams_index": 0,
                          "bridge_fw": "AFC-2.81"}}, BOXED_AMS1_UNIT),
        ("ams2", "AMS 2 Pro", AMS2_IMAGE,
         {"Bambu_AMS_1": {"bridge_online": True, "ams_model": "ams2", "ams_index": 0,
                          "bridge_fw": "AFC-2.81"}}, AMS2_UNIT),
        ("ht", "AMS HT", HT_IMAGE,
         {"Bambu_AMS_HT_1": {"bridge_online": True, "ams_model": "ht", "ams_index": 4,
                             "bridge_fw": "AFC-2.81"}}, HT_UNIT),
    ], ids=["ams1", "ams2", "ht"])
    def test_each_model_command_reads_its_own_artifact(
            self, main_run: TestMain._Harness, model: str, name: str, image: str,
            units: Dict[str, Dict[str, Any]], unit_check: str) -> None:
        assert main_run.run(model, self._BusBridge(model), units, mode="check") == 0
        expected = (self._opening(name, model, main_run.port, mode="CHECK") + image
                    + self.FW + unit_check
                    + "\n==> READ-ONLY CHECKS PASSED. Nothing was changed. Preflight and go "
                    + "also check the bus itself, with Klipper stopped.\n")
        assert (main_run.out, main_run.err, main_run.log) == (expected, "", expected)
        assert main_run.moon.actions == []               # check never stops Klipper
        assert main_run.opened == []
