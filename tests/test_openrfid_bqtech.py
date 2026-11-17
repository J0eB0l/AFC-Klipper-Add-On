"""Unit tests for contrib/openrfid-bqtech/src/tag/bqtech/processor.py."""

from __future__ import annotations

import enum
import importlib
import logging
import os
import struct
import sys
import types
from typing import Any, Dict, Iterator, List, Tuple

import pytest


BQTECH_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "contrib", "openrfid-bqtech", "src")


BQTECH_VALID_BASE_MATERIALS = {"PLA", "PET", "PETG", "ABS", "ASA", "TPU", "PC", "PA"}


# The staged package's own modules, imported fresh for every test.
BQTECH_PACKAGE_MODULES = ("tag.bqtech", "tag.bqtech.constants", "tag.bqtech.processor")


BQTECH_PROCESSOR_NAME = "bqtech_tag_processor"


class BqTechGenericFilament:
    """OpenRFID's GenericFilament: keeps its fields, refuses an unknown base type."""

    def __init__(self, **fields: Any) -> None:
        """
        Keep every field as an attribute.

        :param fields: the filament's fields by name
        """
        if fields["type"] not in BQTECH_VALID_BASE_MATERIALS:
            error_str = f"Invalid filament type: {fields['type']}"
            raise ValueError(error_str)
        self.__dict__.update(fields)

    @staticmethod
    def generate_unique_id(*parts: Any) -> str:
        """
        Join the parts in order, so a test can read back what the id was built from.

        :param parts: the values the id is built from
        :return str: "uid|" and the parts, "|"-separated
        """
        return "uid|" + "|".join(str(part) for part in parts)


class BqTechTagType(enum.Enum):
    """OpenRFID's TagType."""

    Unknown = 0
    MifareUltralight = 1
    MifareClassic1k = 8


class BqTechScanResult:
    """OpenRFID's ScanResult: the tag the reader found."""

    def __init__(self, tag_type: BqTechTagType, uid: bytes) -> None:
        """
        Keep the tag type and UID.

        :param tag_type: the tag's type
        :param uid: the tag's UID
        """
        self.tag_type = tag_type
        self.uid = uid


class BqTechTagAuthentication:
    """OpenRFID's TagAuthentication: a Key A and a Key B for each sector."""

    def __init__(self, hkdf_key_a: List[List[int]], hkdf_key_b: List[List[int]]) -> None:
        """
        Keep the per-sector keys.

        :param hkdf_key_a: Key A per sector
        :param hkdf_key_b: Key B per sector
        """
        self.hkdf_key_a = hkdf_key_a
        self.hkdf_key_b = hkdf_key_b


class BqTechLogRecorder(logging.Logger):
    """A stdlib logger outside the logging tree that keeps every record it is handed."""

    def __init__(self, name: str) -> None:
        """
        Start with no messages, every level enabled.

        :param name: logger name
        """
        super().__init__(name, logging.DEBUG)
        self.messages: List[Tuple[str, str]] = []

    def handle(self, record: logging.LogRecord) -> None:
        """
        Keep the record as (lowercase level, formatted message).

        :param record: the record to keep
        """
        self.messages.append((record.levelname.lower(), record.getMessage()))


class BqTechMifareClassicTagProcessor:
    """OpenRFID's MifareClassicTagProcessor base: name, enabled flag and logger."""

    def __init__(self, config: Dict[str, Any]) -> None:
        """
        Read the name and enabled flag from the config and set up a recording logger.

        :param config: the processor's config section, its name under "__name"
        """
        self.name = config["__name"]
        self.config = config
        self.enabled = str(config.get("enabled", "true")).lower() == "true"
        self.logger = BqTechLogRecorder(f"openrfid.{self.name}")


def bqtech_extract_string(data: bytes, position: int, length: int) -> str:
    """
    OpenRFID's tag.binary.extract_string.

    :param data: tag image
    :param position: first byte of the field
    :param length: field width in bytes
    :return str: the field's ASCII text up to the first NUL
    """
    return data[position:position + length].split(b"\x00")[0].decode("ascii", "replace")


def bqtech_extract_uint16_le(data: bytes, position: int) -> int:
    """
    OpenRFID's tag.binary.extract_uint16_le.

    :param data: tag image
    :param position: first byte of the field
    :return int: the little-endian u16 there
    """
    return struct.unpack_from("<H", data, position)[0]


def bqtech_stub_module(name: str, **attrs: Any) -> types.ModuleType:
    """
    Build an empty module holding the given attributes.

    :param name: module name
    :param attrs: the module's attributes
    :return types.ModuleType: a new module holding them
    """
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


@pytest.fixture()
def bqtech_processor_class(monkeypatch: pytest.MonkeyPatch) -> Iterator[type]:
    """
    The real BqTechTagProcessor class, imported fresh against the OpenRFID stubs.

    :param monkeypatch: undoes the sys.modules changes after the test
    :return Iterator[type]: the processor class
    """
    # tag.bqtech is the real package on disk; its OpenRFID siblings are stubs.
    tag_package = bqtech_stub_module("tag")
    tag_package.__path__ = [os.path.join(BQTECH_SRC, "tag")]
    stubs = {
        "filament": bqtech_stub_module("filament", GenericFilament=BqTechGenericFilament),
        "filament.valid_materials": bqtech_stub_module(
            "filament.valid_materials", VALID_BASE_MATERIALS=BQTECH_VALID_BASE_MATERIALS),
        "reader": bqtech_stub_module("reader"),
        "reader.scan_result": bqtech_stub_module("reader.scan_result",
                                                 ScanResult=BqTechScanResult),
        "tag": tag_package,
        "tag.binary": bqtech_stub_module("tag.binary", extract_string=bqtech_extract_string,
                                         extract_uint16_le=bqtech_extract_uint16_le),
        "tag.tag_types": bqtech_stub_module("tag.tag_types", TagType=BqTechTagType),
        "tag.mifare_classic_tag_processor": bqtech_stub_module(
            "tag.mifare_classic_tag_processor",
            MifareClassicTagProcessor=BqTechMifareClassicTagProcessor,
            TagAuthentication=BqTechTagAuthentication),
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    for name in BQTECH_PACKAGE_MODULES:
        monkeypatch.delitem(sys.modules, name, raising=False)
    yield importlib.import_module("tag.bqtech.processor").BqTechTagProcessor
    for name in BQTECH_PACKAGE_MODULES:
        sys.modules.pop(name, None)


@pytest.fixture()
def bqtech_processor(bqtech_processor_class: type) -> Any:
    """
    An enabled processor, built through its real __init__.

    :param bqtech_processor_class: the processor class
    :return Any: the processor
    """
    return bqtech_processor_class({"__name": BQTECH_PROCESSOR_NAME})


def bqtech_scan(tag_type: BqTechTagType = BqTechTagType.MifareClassic1k) -> BqTechScanResult:
    """
    A scan of a tag with UID aabbccdd.

    :param tag_type: the type the reader reports
    :return BqTechScanResult: a scan of a tag with UID aabbccdd
    """
    return BqTechScanResult(tag_type, b"\xaa\xbb\xcc\xdd")


class TestBqTechTagProcessorAuthenticateTag:
    def test_authenticate_returns_all_default_ff_keys(self, bqtech_processor):
        auth = bqtech_processor.authenticate_tag(bqtech_scan())
        ff_key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        assert type(auth) is BqTechTagAuthentication
        assert vars(auth) == {"hkdf_key_a": [ff_key] * 16, "hkdf_key_b": [ff_key] * 16}
        assert bqtech_processor.logger.messages == []

    def test_authenticate_rejects_non_classic(self, bqtech_processor):
        scan = bqtech_scan(BqTechTagType.MifareUltralight)
        assert bqtech_processor.authenticate_tag(scan) is None
        assert bqtech_processor.logger.messages == []

    def test_authenticate_disabled_processor_returns_none(self, bqtech_processor_class):
        processor = bqtech_processor_class({"__name": BQTECH_PROCESSOR_NAME, "enabled": "false"})
        # A Classic 1K tag, so only the enabled check can turn it away.
        assert processor.authenticate_tag(bqtech_scan()) is None
        assert processor.logger.messages == []


class TestBqTechTagProcessorProcessTag:
    @staticmethod
    def build_image(version: int = 1000, manufacturer: str = "BIQU Filaments",
                    mfg: str = "20240812_162600", material: str = "PET",
                    detailed: str = "PET (CEP Glossy)", serial: str = "IP243ZCXV6712345",
                    rgb: Tuple[int, int, int] = (0x12, 0x34, 0x56), diameter: int = 1748,
                    weight: int = 750, ptmin: int = 200, ptmax: int = 240, bed: int = 60,
                    bed_max: int = 110, drying_time: int = 8, drying_temp: int = 70) -> bytes:
        """
        Assemble a 1024-byte BQ Tech MIFARE Classic 1K image in BTT's layout:
        block N at byte N*16, little-endian u16 numbers, NUL-padded ASCII.
        Every default differs from the processor's fallbacks, and the default
        manufacturer, detailed type and serial fill their whole fields.

        :param version: tag_version, 1000 on a genuine BQ Tech tag
        :param manufacturer: filament_manufacturer, 14 bytes
        :param mfg: manufacture_datetime, YYYYMMDD_HHMMSS
        :param material: filament_material_type, 16 bytes
        :param detailed: filament_type_detailed, 16 bytes
        :param serial: serial_number, 16 bytes
        :param rgb: color_code, R G B
        :param diameter: filament_diameter in micrometres
        :param weight: spool_weight in grams
        :param ptmin: printing_temperature_min
        :param ptmax: printing_temperature_max
        :param bed: bed_temperature
        :param bed_max: bed_temperature_max
        :param drying_time: drying_time in hours
        :param drying_temp: drying_temp_max
        :return bytes: the tag image
        """
        image = bytearray(1024)

        def put(block: int, offset: int, raw: bytes) -> None:
            """
            Write raw bytes into the image at a block and offset.

            :param block: 16-byte block number
            :param offset: first byte within the block
            :param raw: the bytes to write
            """
            start = block * 16 + offset
            image[start:start + len(raw)] = raw

        put(1, 0, version.to_bytes(2, "little"))
        put(1, 2, manufacturer.encode("ascii"))
        put(2, 0, mfg.encode("ascii"))
        put(4, 0, material.encode("ascii"))
        put(5, 0, detailed.encode("ascii"))
        put(6, 0, serial.encode("ascii"))
        put(8, 0, bytes(rgb))
        put(10, 0, diameter.to_bytes(2, "little"))
        put(17, 0, weight.to_bytes(2, "little"))
        put(18, 0, drying_time.to_bytes(2, "little"))
        put(18, 4, drying_temp.to_bytes(2, "little"))
        put(18, 8, bed_max.to_bytes(2, "little"))
        put(18, 10, ptmin.to_bytes(2, "little"))
        put(18, 12, ptmax.to_bytes(2, "little"))
        put(20, 0, bed.to_bytes(2, "little"))
        return bytes(image)

    @staticmethod
    def decoded(**changes: Any) -> Dict[str, Any]:
        """
        The GenericFilament fields process_tag builds from the default image.

        :param changes: fields that differ from the default image's
        :return Dict[str, Any]: every field by name
        """
        fields = {
            "source_processor": "bqtech_tag_processor",
            # The id names the brand, not the manufacturer field.
            "unique_id": f"uid|BQ Tech|IP243ZCXV6712345|PET|{0xFF123456}|20240812_162600",
            "manufacturer": "BIQU Filaments",
            "type": "PET",
            "modifiers": ["CEP Glossy"],
            "colors": [0xFF123456],
            "diameter_mm": 1.748,
            "weight_grams": 750,
            "hotend_min_temp_c": 200,
            "hotend_max_temp_c": 240,
            "bed_temp_c": 60,
            "drying_temp_c": 70,
            "drying_time_hours": 8,
            "manufacturing_date": "2024-08-12",
        }
        fields.update(changes)
        return fields

    def test_process_tag_decodes_fields(self, bqtech_processor):
        filament = bqtech_processor.process_tag(bqtech_scan(), self.build_image())
        assert type(filament) is BqTechGenericFilament
        assert vars(filament) == self.decoded()
        assert bqtech_processor.logger.messages == []

    def test_process_tag_fingerprint_rejects_non_btt(self, bqtech_processor):
        # tag_version 7: the FF key opened it, but it is not a BQ Tech tag.
        assert bqtech_processor.process_tag(bqtech_scan(), self.build_image(version=7)) is None
        # Past the fingerprint, a blank image would log an unknown material.
        assert bqtech_processor.process_tag(bqtech_scan(), bytes(1024)) is None
        assert bqtech_processor.logger.messages == []

    def test_process_tag_unknown_material_skips(self, bqtech_processor):
        # Fills the 16-byte field, so the message shows the whole field and no more.
        image = self.build_image(material="UNOBTAINIUM GLOW")
        assert bqtech_processor.process_tag(bqtech_scan(), image) is None
        assert bqtech_processor.logger.messages == [
            ("warning", "BqTechTagProcessor: unknown material 'UNOBTAINIUM GLOW', skipping")]

    def test_process_tag_blank_material_skips(self, bqtech_processor):
        assert bqtech_processor.process_tag(bqtech_scan(), self.build_image(material="")) is None
        assert bqtech_processor.logger.messages == [
            ("warning", "BqTechTagProcessor: unknown material '', skipping")]

    def test_process_tag_disabled_processor_returns_none(self, bqtech_processor_class):
        processor = bqtech_processor_class({"__name": BQTECH_PROCESSOR_NAME, "enabled": "false"})
        assert processor.process_tag(bqtech_scan(), self.build_image()) is None
        assert processor.logger.messages == []

    def test_process_tag_rejects_non_classic(self, bqtech_processor):
        scan = bqtech_scan(BqTechTagType.MifareUltralight)
        assert bqtech_processor.process_tag(scan, self.build_image()) is None
        assert bqtech_processor.logger.messages == []

    def test_process_tag_rejects_an_image_that_is_not_1k(self, bqtech_processor):
        image = self.build_image()
        # Both still hold every field, so only the size check turns them away.
        assert bqtech_processor.process_tag(bqtech_scan(), image[:1023]) is None
        assert bqtech_processor.process_tag(bqtech_scan(), image + b"\x00") is None
        assert bqtech_processor.logger.messages == []

    def test_process_tag_blank_manufacturer_reads_as_bq_tech(self, bqtech_processor):
        image = self.build_image(manufacturer="")
        filament = bqtech_processor.process_tag(bqtech_scan(), image)
        assert vars(filament) == self.decoded(manufacturer="BQ Tech")
        assert bqtech_processor.logger.messages == []

    def test_process_tag_zero_diameter_reads_as_1_75_mm(self, bqtech_processor):
        filament = bqtech_processor.process_tag(bqtech_scan(), self.build_image(diameter=0))
        assert vars(filament) == self.decoded(diameter_mm=1.75)
        assert bqtech_processor.logger.messages == []

    def test_process_tag_zero_bed_temperature_falls_back_to_the_maximum(self, bqtech_processor):
        filament = bqtech_processor.process_tag(bqtech_scan(), self.build_image(bed=0))
        assert vars(filament) == self.decoded(bed_temp_c=110)
        assert bqtech_processor.logger.messages == []

    def test_process_tag_short_production_date_reads_as_the_epoch(self, bqtech_processor):
        # Seven digits: one short of a YYYYMMDD day.
        filament = bqtech_processor.process_tag(bqtech_scan(), self.build_image(mfg="2024081"))
        assert vars(filament) == self.decoded(
            unique_id=f"uid|BQ Tech|IP243ZCXV6712345|PET|{0xFF123456}|2024081",
            manufacturing_date="1970-01-01")
        assert bqtech_processor.logger.messages == []

    def test_process_tag_detailed_type_without_the_material_is_kept_whole(self, bqtech_processor):
        image = self.build_image(detailed="Glossy Black")
        filament = bqtech_processor.process_tag(bqtech_scan(), image)
        assert vars(filament) == self.decoded(modifiers=["Glossy Black"])
        assert bqtech_processor.logger.messages == []

    def test_process_tag_detailed_type_of_just_the_material_adds_none(self, bqtech_processor):
        filament = bqtech_processor.process_tag(bqtech_scan(), self.build_image(detailed="PET"))
        assert vars(filament) == self.decoded(modifiers=[])
        assert bqtech_processor.logger.messages == []

    def test_process_tag_material_matches_ignoring_case_and_padding(self, bqtech_processor):
        image = self.build_image(material="pet ", detailed=" pet (cep) ")
        filament = bqtech_processor.process_tag(bqtech_scan(), image)
        # The id keeps the material as the tag spells it.
        assert vars(filament) == self.decoded(
            unique_id=f"uid|BQ Tech|IP243ZCXV6712345|pet |{0xFF123456}|20240812_162600",
            type="PET", modifiers=["cep"])
        assert bqtech_processor.logger.messages == []
