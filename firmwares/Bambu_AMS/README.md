# AMS updater

Writes Bambu's own firmware into a Bambu AMS 1, AMS 2 Pro or AMS HT through
the BridgeBox, while Klipper keeps running. Python standard library only;
nothing else from this repository is needed on the printer. Step-by-step
instructions are in [`AMS_FLASH_HOWTO.md`](AMS_FLASH_HOWTO.md).

| file | what it is |
|---|---|
| `ams_flash.py` | the updater |
| `ams_flash.cfg` | its `gcode_shell_command`, included from `printer.cfg` |
| `ams_firmware.json` | Bambu's newest two images for each AMS board, and where Bambu serves them |

All three go in one folder in your Klipper config, such as
`~/printer_data/config/ams_firmware/`.

## How it is used

AFC's Flash AMS menu, `AFC_BAMBU_FIRMWARE`, lists the Bambu units with the
firmware each runs, then the images the picked unit can take, newest first,
and runs the updater on the one picked. AFC reads each unit's firmware over
the bus while it is online, and the first time the bridge sees a unit on older
firmware it offers the newest.

The image is downloaded from Bambu when the update starts: the updater reads
only that image out of Bambu's offline update package and checks its MD5
against `ams_firmware.json`. Without internet, put Bambu's `.bin.sig` image or
a Bambu offline update package (`.zip`) in the same folder; the menu lists the
images it finds there.

Other units may stay connected. Klipper lends the updater the unit's bridge
link for the run (`AFC_BAMBU_FIRMWARE UNIT=... HOLD=...`) and takes it back
when it ends, so only the units on that bridge read offline meanwhile.

## Running it by hand

    python3 ams_flash.py --unit Bambu_AMS_1 --version 05.00.22.22
    python3 ams_flash.py --unit Bambu_AMS_1 --version 05.00.22.22 --mode go
    python3 ams_flash.py --unit Bambu_AMS_1 --file <.bin.sig or .zip> --mode go

Without `--mode go` it only runs its checks and changes nothing. With it, the
updater starts a copy of itself that outlives the command, logs to
`logs/ams_flash.log`, writes how the run ended to `ams_flash_result.json` and
shows that in Klipper. A unit an update left in its bootloader is recorded in
`ams_flash_state.json`, and `--mode go` with no `--version` or `--file`
finishes it.
