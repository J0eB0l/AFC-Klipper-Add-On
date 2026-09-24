# OpenRFID `bqtech` tag processor (BigTreeTech MMS / ViViD "BQ Tech")

Adds decoding of BigTreeTech MMS / ViViD **"BQ Tech"** filament tags
(`tag_version 1000`, MIFARE Classic 1K, default `FFFFFFFFFFFF` Key A) to
[OpenRFID](https://github.com/suchmememanyskill/OpenRFID). Installing it lets a
**Snapmaker U1** (which reads tags through OpenRFID, not host-side) recognise BTT
tags, the last brand our AFC ACE2/ViViD readers already decode natively.

This is staged here for two uses:
1. **Install it on a U1** so `[bqtech_tag_processor]` works today.
2. **Open a PR upstream** to OpenRFID so everyone gets it.

## Files (mirror the OpenRFID `src/` layout)

```
src/tag/bqtech/__init__.py       # from .processor import BqTechTagProcessor
src/tag/bqtech/constants.py      # block/byte offsets (tag_version 1000 layout)
src/tag/bqtech/processor.py      # BqTechTagProcessor(MifareClassicTagProcessor)
```

Copy the `src/tag/bqtech/` directory into your OpenRFID checkout at the same path.

## One-line registration patch (`src/main.py`)

OpenRFID registers processors with a hardcoded `match` in
`create_configurable_entity`. Add the import and a `case`:

```diff
--- a/src/main.py
+++ b/src/main.py
@@ imports
 from tag.bambu import BambuTagProcessor
 from tag.creality import CrealityTagProcessor
 from tag.snapmaker import SnapmakerTagProcessor
+from tag.bqtech import BqTechTagProcessor
@@ def create_configurable_entity(key, config):
         case "snapmaker_tag_processor":
             return SnapmakerTagProcessor(config)
+        case "bqtech_tag_processor":
+            return BqTechTagProcessor(config)
```

## Enable it in `openrfid_user.cfg`

No key is needed: BQ Tech tags use the public default `FFFFFFFFFFFF` Key A:

```ini
[bqtech_tag_processor]
```

That's it. The processor authenticates every sector with the default key, then
**fingerprints the tag by `tag_version == 1000`** before decoding, so it never
mis-reads a Bambu/Creality/Snapmaker tag (whose data sectors won't authenticate
with the default key anyway, and which carry a different version).

## What it decodes

`manufacturer`, `type` (mapped to an OpenRFID `VALID_BASE_MATERIALS` base type +
modifiers), primary `color` (RRGGBB, single-colour; BQ Tech's second colour is
only a text name), `diameter`, `weight`, hotend min/max temps, bed temp, drying
temp/time, `serial_number`, and `manufacturing_date` (`YYYYMMDD_HHMMSS` →
ISO `YYYY-MM-DD`).

## Provenance

Field offsets are ported from
`bigtreetech/BIGTREETECH_MMS` (`klippy/extras/mms/hardware/mfrc522.py`,
`RFIDDict._fields`; offsets there are hex-character counts, halved to bytes here)
and mirror the `decode_btt()` in this repo's `extras/AFC_ACE2_rfid.py`, which is
covered by unit tests (`tests/test_AFC_ACE2_rfid.py::test_decode_btt_*`).
