from filament import GenericFilament
from filament.valid_materials import VALID_BASE_MATERIALS
from reader.scan_result import ScanResult
from tag.tag_types import TagType
from tag.mifare_classic_tag_processor import (
    MifareClassicTagProcessor, TagAuthentication)
import tag.binary as binary
from . import constants as Constants


class BqTechTagProcessor(MifareClassicTagProcessor):
    """Decode BigTreeTech MMS / ViViD "BQ Tech" (tag_version 1000) MIFARE Classic
    1K tags. Every sector uses the DEFAULT key FFFFFFFFFFFF (Key A): no config
    key required, so this processor is always enabled. A genuine BQ Tech tag is
    fingerprinted by tag_version == 1000; anything else authenticates but is
    rejected, so this never mis-decodes a Bambu/Creality/etc. tag.
    """

    def __init__(self, config: dict):
        super().__init__(config)
        # No secret key to gate on: the default FF Key A is public. Respect the
        # base `enabled` (config may still switch it off).

    def authenticate_tag(self, scan_result: ScanResult) -> TagAuthentication | None:
        if not self.enabled:
            return None
        if scan_result.tag_type != TagType.MifareClassic1k:
            return None
        default = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        keys_a = [list(default) for _ in range(16)]
        keys_b = [list(default) for _ in range(16)]
        return TagAuthentication(keys_a, keys_b)

    def process_tag(self, scan_result: ScanResult, data: bytes) -> GenericFilament | None:
        if not self.enabled:
            return None
        if scan_result.tag_type != TagType.MifareClassic1k:
            return None
        if len(data) != Constants.TAG_TOTAL_SIZE:
            return None
        # Fingerprint: reject anything that isn't a BQ Tech tag_version-1000 tag.
        if binary.extract_uint16_le(data, Constants.TAG_VERSION_POS) != \
                Constants.EXPECTED_TAG_VERSION:
            return None

        manufacturer = binary.extract_string(
            data, Constants.FILAMENT_MANUFACTURER_POS,
            Constants.FILAMENT_MANUFACTURER_LEN) or "BQ Tech"
        material = binary.extract_string(
            data, Constants.FILAMENT_MATERIAL_TYPE_POS,
            Constants.FILAMENT_MATERIAL_TYPE_LEN)
        detailed = binary.extract_string(
            data, Constants.FILAMENT_TYPE_DETAILED_POS,
            Constants.FILAMENT_TYPE_DETAILED_LEN)
        serial = binary.extract_string(
            data, Constants.SERIAL_NUMBER_POS, Constants.SERIAL_NUMBER_LEN)
        production = binary.extract_string(
            data, Constants.MANUFACTURE_DATETIME_POS,
            Constants.MANUFACTURE_DATETIME_LEN)

        r = data[Constants.COLOR_CODE_POS]
        g = data[Constants.COLOR_CODE_POS + 1]
        b = data[Constants.COLOR_CODE_POS + 2]
        argb_color = (0xFF << 24) | (r << 16) | (g << 8) | b

        diameter_raw = binary.extract_uint16_le(data, Constants.FILAMENT_DIAMETER_POS)
        diameter_mm = round(diameter_raw / 1000.0, 3) if diameter_raw else 1.75
        weight_grams = binary.extract_uint16_le(data, Constants.SPOOL_WEIGHT_POS)
        hotend_min = binary.extract_uint16_le(data, Constants.PRINTING_TEMP_MIN_POS)
        hotend_max = binary.extract_uint16_le(data, Constants.PRINTING_TEMP_MAX_POS)
        bed = (binary.extract_uint16_le(data, Constants.BED_TEMPERATURE_POS)
               or binary.extract_uint16_le(data, Constants.BED_TEMP_MAX_POS))
        drying_temp = binary.extract_uint16_le(data, Constants.DRYING_TEMP_MAX_POS)
        drying_time = binary.extract_uint16_le(data, Constants.DRYING_TIME_POS)

        base_type, modifiers = self.__normalize_material(material, detailed)
        if base_type is None:
            self.logger.warning(
                "BqTechTagProcessor: unknown material %r, skipping", material)
            return None

        return GenericFilament(
            source_processor=self.name,
            unique_id=GenericFilament.generate_unique_id(
                "BQ Tech", serial, material, argb_color, production),
            manufacturer=manufacturer,
            type=base_type,
            modifiers=modifiers,
            colors=[argb_color],
            diameter_mm=diameter_mm,
            weight_grams=weight_grams,
            hotend_min_temp_c=hotend_min,
            hotend_max_temp_c=hotend_max,
            bed_temp_c=bed,
            drying_temp_c=drying_temp,
            drying_time_hours=drying_time,
            manufacturing_date=self.__parse_production_date(production),
        )

    def __normalize_material(self, material: str, detailed: str):
        """Map the tag's material string to a GenericFilament base type (which is
        validated against VALID_BASE_MATERIALS) plus a list of modifiers. Returns
        (base_type, modifiers) or (None, []) if the material can't be recognised."""
        m = (material or "").strip().upper()
        if not m:
            return None, []
        if m in VALID_BASE_MATERIALS:
            mod = (detailed or "").strip()
            # Drop a redundant "PET ..." style prefix; keep the rest as a modifier.
            if mod.upper().startswith(m):
                mod = mod[len(m):].strip(" ()")
            return m, ([mod] if mod else [])
        # Aliases for BQ Tech material codes that don't match a base type 1:1.
        # (Empty for now: PLA/PET/PETG/ABS/ASA/TPU etc. are already valid base
        # materials; extend this as more BQ Tech codes are observed.)
        aliases = {}
        if m in aliases and aliases[m] in VALID_BASE_MATERIALS:
            return aliases[m], []
        return None, []

    def __parse_production_date(self, date_str: str) -> str:
        """"20240812_162600" (YYYYMMDD_HHMMSS) -> ISO "2024-08-12"."""
        try:
            day_part = (date_str or "").split("_")[0]
            if len(day_part) >= 8:
                return "%s-%s-%s" % (day_part[0:4], day_part[4:6], day_part[6:8])
        except Exception:
            pass
        return "1970-01-01"
