# Updating an AMS's firmware from the bridge

Update a Bambu AMS's own firmware over the bus from Klipper: no Bambu printer,
no BOOTSEL, no cable to the AMS. There is one command per model:

| model | command | artifact file |
|---|---|---|
| AMS 2 Pro | `AFC_BAMBU_AMS_UPDATE` | `ams2_artifact.json` |
| AMS 1 | `AFC_BAMBU_AMS1_UPDATE` | `ams1_artifact.json` |
| AMS HT | `AFC_BAMBU_HT_UPDATE` | `ht_artifact.json` |

Each command only ever sends its own model's file, and refuses before erasing
anything if the file or the connected unit is not that model. An AMS 1 and an
AMS 2 look the same on the bus, so the command asks the unit which one it is.

Why update: an AMS 2 only loads a tray while its dryer is running once it has
Bambu's updated AMS firmware; before that it accepts the load and never runs
the feeder. An AMS HT on firmware 05.00.22.19 also keeps heating while
printing, with a lane loaded. Units that have not been updated still
decline: on an un-updated AMS 2 a load made while drying stalls and fails, and
an un-updated HT refuses a dry start with a lane loaded, which AFC reports as a
refused dry.

## The one rule

**Only the unit being updated may be connected to the bridge.** Unplug every
other AMS from the bus first. The command checks this twice: in Klipper, and
then on the bus itself with Klipper stopped, where it also sees units Klipper
has no section for and a unit sitting in its bootloader. It refuses if it
finds a second unit either way.

Also:

- The printer must be idle (not printing or paused).
- The AMS must be powered and on the wire as normal, and showing online.

## What you need (one time)

1. **A bridge on firmware AFC-1.75 or newer.** WiFi (`tcp://`) and USB
   bridges both work; the port is read from your Klipper config. To update
   the bridge itself, see
   [BRIDGE_FLASH_HOWTO.md](../BridgeBox/BRIDGE_FLASH_HOWTO.md) in
   `Firmwares/BridgeBox/`.
2. **Klipper's `gcode_shell_command`.** Built into Kalico; on stock Klipper
   install the `gcode_shell_command` extra.
3. **The updater files** from this folder, `Firmwares/Bambu_AMS/`, copied
   into your Klipper config folder (usually `~/printer_data/config/`):
   - `ams_flash.py`
   - `ams_flash.cfg`
   - the artifact for your model, from the same folder: `ams2_artifact.json`
     (AMS 2 Pro) and `ht_artifact.json` (AMS HT) are included. There is no
     AMS 1 artifact yet; see [Artifacts](#artifacts).
4. Keep the artifact in the same folder as `ams_flash.py`, with the name from
   the table above. That is where the commands look for it.
5. **Edit one line in `ams_flash.cfg`**: the `command:` path must be the
   absolute path to `ams_flash.py`. Find it with:

       ls -l ~/printer_data/config/ams_flash.py

   and put exactly that into:

       [gcode_shell_command ams_flash]
       command: /usr/bin/python3 /home/YOURUSER/printer_data/config/ams_flash.py

   Klipper does not expand `~`, so it has to be the full path. If it is wrong,
   the command fails at once with `can't open file` in the Klipper console,
   and nothing is sent to the AMS.
6. Add `[include ams_flash.cfg]` to `printer.cfg`, then `FIRMWARE_RESTART`.

## Update a unit

Use your model's command from the table. The steps are the same for all three;
the examples use the AMS 2 command.

1. **Connect only that unit** to the bridge and wait until it shows online.

2. **Check** (read-only, changes nothing, Klipper keeps running):

       AFC_BAMBU_AMS_UPDATE MODE=check

   This confirms the printer is idle, the artifact is complete and is this
   model's image, the bridge firmware is new enough, and exactly one unit is
   online in Klipper, configured as this model, at the address the image is
   for. A good result ends with `READ-ONLY CHECKS PASSED`.

3. **Preflight** (optional, recommended the first time):

       AFC_BAMBU_AMS_UPDATE

   Everything in check, then Klipper is stopped and the bus itself is
   checked: exactly one unit answering, at the right address, answering as
   this model, and no other unit sitting in its bootloader. Then the unit is
   asked to enter its bootloader. Nothing is erased. It ends with
   `PREFLIGHT PASSED` and Klipper restarts on its own. The unit is left
   waiting in its bootloader and shows offline; step 4 picks it up from
   there. To return it to normal instead, power-cycle the AMS.

4. **Update**:

       AFC_BAMBU_AMS_UPDATE MODE=go

   Klipper stops, the unit's firmware is erased and rewritten, and Klipper
   restarts. It takes a few minutes. Follow it in
   `~/printer_data/logs/ams_flash.log`. The line that means it worked is:

       FLASH CONFIRMED: the loader verified the image and reset into the new firmware.

   That is the AMS's own bootloader checking the whole image, and nothing else
   counts as success. A few `chunk hash error` passes before it are normal:
   the command resends the whole image until the bootloader accepts it, up to
   seven times.

`LABEL=<name>` is optional on any of these and only tags the log (the unit's
serial is a good choice). `TARGET=<port>` is only needed when your config has
more than one bridge: give the `serial_port` of the one to use.

## After

- The unit reboots into the new firmware and comes back online by itself.
  The log ends with `ONLINE on the new firmware`.
- Reconnect any other units and restart Klipper; they re-enroll on their own.
- To confirm on an AMS 2 or HT, start the dryer and load a tray: loading while
  drying is what these updates add.

## If it does not finish

The bootloader is never erased, so a stopped update leaves the unit waiting
in its bootloader, not bricked. **Do not power it off**: it has no firmware to
boot until an update completes. Run the same `MODE=go` command again. The
unit shows offline while it waits, and the command remembers which unit it
left there (in `ams_flash_state.json` beside `ams_flash.py`), so it picks it
up and rewrites it from scratch. Use the same model's command; another
model's refuses while that unit is waiting.

| log says | what to do |
|---|---|
| `out of attempts` | Run `MODE=go` again. If every pass fails with chunk hash errors, the bus wiring is dropping bytes (check termination and ground). |
| `IMG HASH ERROR` | The unit accepted every block and rejected the image, so the artifact is wrong for this unit. It stops instead of erasing again. Get the right artifact and run `MODE=go` again. |
| `need exactly one AMS on the bus` | Unplug the other units and retry. The bus count includes units Klipper has no section for. |
| `is configured as ams_model ...` | The connected unit is not the model this command is for. Use the right command, or fix `ams_model:` if the config is wrong. |
| `does not answer as an AMS 2` | The unit is an AMS 1. Use `AFC_BAMBU_AMS1_UPDATE`. |
| `answers as an AMS 2` | The unit is an AMS 2. Use `AFC_BAMBU_AMS_UPDATE`. |
| `enrolled at ... index` | The unit is not at the address the image is for, because another unit enrolled on the bridge before it. Power the bridge off and on with only this unit connected, restart Klipper, and retry. |
| `sitting in its bootloader` | Another unit on the bus is waiting in its bootloader. Unplug it, or finish its own update first. |
| `unfinished ... update` | A unit of another model is waiting in its bootloader from an earlier run. Finish it with that model's command. |
| `not an ... image` or `not to an ...` (step 2) | The artifact file is for a different model. |
| `unit did not enter its loader` | Nothing was erased. Check the unit is online and that it is the right command. |
| `bridge is below 175` | Update the bridge firmware first ([BRIDGE_FLASH_HOWTO.md](../BridgeBox/BRIDGE_FLASH_HOWTO.md)). |
| `another AMS update is already running` | Wait for the first one to finish; follow it in `logs/ams_flash.log`. |
| `could not launch detached` | See [Detaching](#detaching). |

## Several units

Update them one at a time: finish one, unplug it, connect the next, repeat.

## Artifacts

An artifact is the firmware as it goes out on the bus: one header frame plus
the data blocks. Each model's is different and they are not interchangeable.
The two that exist ship in this folder, `Firmwares/Bambu_AMS/`.

| file | model | firmware | blocks |
|---|---|---|---|
| `ams2_artifact.json` | AMS 2 Pro | v05.00.22.22 | 168 |
| `ht_artifact.json` | AMS HT | v05.00.22.19 | 157 |
| `ams1_artifact.json` | AMS 1 | none yet | |

- **AMS 2 Pro:** carved from a capture of a Bambu printer updating an AMS 2.
  It cannot be built from Bambu's firmware file, because each block carries a
  tag that is not in it. An older file named `ams_artifact.json` beside
  `ams_flash.py` is also picked up.
- **AMS HT:** built from Bambu's own firmware file for the HT. An HT has been
  updated to v05.00.22.19 from this file.
- **AMS 1:** like the AMS 2, it has to come from a capture of an AMS 1 being
  updated. None has been captured yet, so no AMS 1 has been updated this way.
  Until one exists, `AFC_BAMBU_AMS1_UPDATE` stops at the artifact check and
  touches nothing.

## Detaching

Preflight and go stop Klipper, so the command first relaunches itself as its
own system service; otherwise it would be stopped along with Klipper. It
tries, in order:

1. `systemd-run --user`, if lingering is on for your user. Turn it on once
   with `sudo loginctl enable-linger $USER`. Without it this way is skipped,
   because the flash would stop when your last login session closed.
2. Passwordless `sudo`, if you have it (the default on Raspberry Pi OS).
3. A one-time password file. Create `ams_sudo.pw` beside `ams_flash.py`:

       printf '%s' 'YOURPASSWORD' > ~/printer_data/config/ams_sudo.pw

   It is used once and deleted automatically.

`MODE=check` never stops Klipper and needs none of this.
