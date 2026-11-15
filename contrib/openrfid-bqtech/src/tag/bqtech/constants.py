# BigTreeTech MMS / ViViD "BQ Tech" MIFARE Classic 1K tag layout.
#
# Block N lives at image byte N*16. BTT's source (bigtreetech/BIGTREETECH_MMS
# klippy/extras/mms/hardware/mfrc522.py, RFIDDict._fields) expresses field
# offsets/lengths in HEX CHARACTERS (nibbles); they are halved to bytes here.
# All numeric fields are little-endian u16; strings are ASCII (null-stripped);
# color_code is a raw RRGGBB triple (single colour, no alpha). These offsets
# mirror the tested decode_btt() in the AFC ACE2 reader stack.

TAG_TOTAL_SIZE = 1024

# tag_version fingerprint: a genuine BQ Tech tag carries 1000 here. Since every
# sector uses the default FFFFFFFFFFFF Key A, a foreign tag could authenticate
# too: this version check is what stops it from being mis-decoded.
TAG_VERSION_POS = 1 * 16 + 0        # 16
TAG_VERSION_LEN = 2
EXPECTED_TAG_VERSION = 1000

FILAMENT_MANUFACTURER_POS = 1 * 16 + 2   # 18
FILAMENT_MANUFACTURER_LEN = 14

MANUFACTURE_DATETIME_POS = 2 * 16 + 0    # 32  (e.g. "20240812_162600")
MANUFACTURE_DATETIME_LEN = 16

FILAMENT_MATERIAL_TYPE_POS = 4 * 16 + 0  # 64  (e.g. "PET")
FILAMENT_MATERIAL_TYPE_LEN = 16

FILAMENT_TYPE_DETAILED_POS = 5 * 16 + 0  # 80  (e.g. "PET (CEP)")
FILAMENT_TYPE_DETAILED_LEN = 16

SERIAL_NUMBER_POS = 6 * 16 + 0           # 96
SERIAL_NUMBER_LEN = 16

COLOR_CODE_POS = 8 * 16 + 0              # 128  raw RRGGBB (3 bytes)

FILAMENT_DIAMETER_POS = 10 * 16 + 0      # 160  (1750 -> 1.750 mm)
DENSITY_POS = 10 * 16 + 2                # 162  (1240 -> 1.240 g/cm^3)

SPOOL_WEIGHT_POS = 17 * 16 + 0           # 272  grams

DRYING_TIME_POS = 18 * 16 + 0           # 288  hours
DRYING_TEMP_MAX_POS = 18 * 16 + 4       # 292  C
BED_TEMP_MAX_POS = 18 * 16 + 8          # 296  C
PRINTING_TEMP_MIN_POS = 18 * 16 + 10    # 298  C
PRINTING_TEMP_MAX_POS = 18 * 16 + 12    # 300  C

BED_TEMPERATURE_POS = 20 * 16 + 0       # 320  recommended bed temp C
