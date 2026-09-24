# Self-contained AMS updater (drop-in)

Updates a Bambu AMS's own firmware from Klipper through the bridge. Python
standard library only; nothing else from this repository is needed on the
printer. Step-by-step instructions for users are in
[`AMS_FLASH_HOWTO.md`](AMS_FLASH_HOWTO.md).

| file | goes in | what it is |
|---|---|---|
| `ams_flash.py` | your Klipper **config** dir | the whole updater |
| `ams_flash.cfg` | included by your config | the three update commands |
| `ams2_artifact.json` | beside `ams_flash.py` | **AMS 2 Pro** firmware, v05.00.22.22 (included) |
| `ht_artifact.json` | beside `ams_flash.py` | **AMS HT** firmware, v05.00.22.19 (included) |
| `ams1_artifact.json` | beside `ams_flash.py` | **AMS 1** firmware (none exists yet) |

Copy only the artifacts for models you actually have. A missing one is
reported at step [2] and nothing is touched.

## One command per model

    AFC_BAMBU_AMS_UPDATE    ->  ams2_artifact.json   (AMS 2 Pro)
    AFC_BAMBU_AMS1_UPDATE   ->  ams1_artifact.json   (AMS 1)
    AFC_BAMBU_HT_UPDATE     ->  ht_artifact.json     (AMS HT)

Each takes `MODE=check` (read-only), no `MODE` (preflight: probes, no erase),
or `MODE=go` (erases and rewrites), an optional `LABEL=` for the log, and
`TARGET=` for a config with more than one bridge.

The images are not interchangeable. Before anything is erased the updater
checks that:

1. the printer is not printing,
2. the artifact is complete, its frames are addressed to this model (HT:
   device `0x1800`, AMS id `0x80`; AMS 2 and AMS 1: device `0x0700`, AMS id
   `0x00`), and the image it carries is this model's (the vendor file name in
   its header: `n3f_` AMS 2, `n3s_` HT, `ams_` AMS 1),
3. the bridge runs firmware 1.75 or newer,
4. in Klipper, exactly one AMS is online, configured as this model, at the
   bus address the image is for,
5. on the bus itself, with Klipper stopped: exactly one unit answers, at that
   address; asked directly, it answers as this model (only an AMS 2 answers
   the 0x3702 generation query, so this is what keeps an AMS 2 image off an
   AMS 1 and back); and no other unit is sitting in its bootloader,
6. the unit enters its bootloader when asked (non-destructive).

Only `MODE=go`, past all six, erases. A run records the unit it sent into its
bootloader in `ams_flash_state.json` beside `ams_flash.py`, so a stopped
update is resumed by running the same command again: a unit waiting in its
bootloader never shows online, and the record is what lets gates 4 and 5
accept it. A successful flash deletes the record.

## The bridge link

The bridge is found in the live Klipper config (`serial_port:` and `tcp_key:`
of the `[AFC_BridgeBox]` or `[AFC_BambuAMS]` sections), or named with
`TARGET=` on the command (`--target`) when the config has more than one. A
`tcp://host:port` bridge and a USB one (`/dev/serial/by-id/...`) both work.
The USB link opens the port non-blocking with the standard library, so a
bridge that stops draining mid-transfer becomes a retried pass rather than a
hang, and a link that drops (a USB bridge that resets) is reopened before the
next pass.

## Where the artifacts come from

The AMS 2 Pro and AMS HT artifacts ship in this folder. An AMS HT artifact is
packed straight from Bambu's own firmware file. Boxed models (AMS 2, AMS 1)
cannot be packed that way: their data blocks carry a 4-byte per-block tag that
is absent from the vendor image and cannot be derived from it, so a boxed
artifact has to come from a capture of a Bambu printer updating one.
