"""AFC_BAMBU_FLASH must send each bridge an image built for its chip.

A Pico bridge takes a .uf2, flattened; the BridgeBox S3 takes its ESP-IDF app
.bin unchanged. Both arrive through firmware_image(), which also names the
chip so the flash command can refuse a mismatch before a byte is sent. The
S3 firmware reports "ESP32S3" in `info`; that name is not cross-checked here
because this test ships with the module and the firmware tree does not (see
test_repo_boundary.py).
"""
from __future__ import annotations

import struct

import pytest

from extras.AFC_BambuAMS import firmware_image


def _esp_header(chip_id: int) -> bytes:
    """The first 24 bytes of an ESP-IDF app image for `chip_id`."""
    hdr = bytearray(24)
    hdr[0] = 0xE9                        # image magic
    hdr[1] = 4                           # segment count
    struct.pack_into("<H", hdr, 12, chip_id)
    return bytes(hdr)


def test_s3_image_passes_through_unchanged() -> None:
    blob = _esp_header(9) + bytes(range(256)) * 4
    img, chip = firmware_image(blob)
    assert img == blob
    assert chip == "ESP32S3"


def test_esp_image_for_an_unknown_chip_is_refused() -> None:
    with pytest.raises(ValueError, match="chip id 5"):
        firmware_image(_esp_header(5) + bytes(64))


def test_anything_else_goes_to_the_uf2_reader() -> None:
    # Not an ESP image, and not whole UF2 blocks either: the UF2 reader's
    # own complaint, which proves the dispatch took that path.
    with pytest.raises(ValueError, match="not a UF2"):
        firmware_image(b"\x00" * 100)

