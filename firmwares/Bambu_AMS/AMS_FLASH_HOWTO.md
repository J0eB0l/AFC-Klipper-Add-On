# Updating an AMS's firmware

Write Bambu's own firmware into a Bambu AMS 1, AMS 2 Pro or AMS HT through the
BridgeBox: no Bambu printer, no cable to the AMS, and Klipper keeps running.
You pick the unit and the version from a menu, and the image comes straight
from Bambu.

Where the firmware comes from: this repository doesn't host, bundle or
redistribute any Bambu Lab firmware. The updater only carries a list of the
versions Bambu Lab has published and the addresses Bambu Lab serves them from.
When you start an update, your printer downloads the image directly from Bambu
Lab's servers and checks it against that list before writing it to the AMS. An
image you download from Bambu Lab yourself and save beside the updater is
checked the same way.

Why update: an AMS 2 Pro only loads a tray while its dryer runs once it has
Bambu's newer AMS firmware; before that it takes the load and never runs the
feeder. An AMS HT on 05.00.22.19 also keeps heating while printing with a lane
loaded. A unit that was not updated still declines: AFC reports a load made
while drying as stalled, and a dry started with a lane loaded as refused.

## One-time setup

1. **Bridge firmware AFC-1.75 or newer.** USB and WiFi bridges both work. To
   update the bridge, see
   [BRIDGE_FLASH_HOWTO.md](../BridgeBox/BRIDGE_FLASH_HOWTO.md).
2. **Klipper's `gcode_shell_command`.** Kalico has it built in; on Klipper,
   install the `gcode_shell_command` extra, as KIAUH does.
3. **Copy the three files from this folder** into one folder in your Klipper
   config, such as `~/printer_data/config/ams_firmware/`:
   - `ams_flash.py`, the updater
   - `ams_flash.cfg`, its shell command
   - `ams_firmware.json`, the list of Bambu's images
4. **Edit one line in `ams_flash.cfg`.** `command:` must be the absolute path
   to `ams_flash.py`, because Klipper does not expand `~`. Find it with:

       ls -l ~/printer_data/config/ams_firmware/ams_flash.py

   and put that path after `/usr/bin/python3 `:

       [gcode_shell_command ams_flash]
       command: /usr/bin/python3 /home/pi/printer_data/config/ams_firmware/ams_flash.py

   A wrong path only makes the update fail at once with `can't open file`;
   nothing is sent to the AMS.
5. Add `[include ams_firmware/ams_flash.cfg]` to `printer.cfg`, then
   `FIRMWARE_RESTART`.
6. **Let the update run on its own.** It outlives the shell command that
   starts it, so a Klipper restart cannot stop it partway. Run this once:

       sudo loginctl enable-linger $USER

   Passwordless `sudo`, the default on Raspberry Pi OS, works too. Otherwise
   put your sudo password in `ams_sudo.pw` beside `ams_flash.py`; it is read
   once and deleted. A Snapmaker U1 needs none of this.

## Update a unit

1. Open the menu: press **Flash AMS** in the BridgeBox display's settings, or
   run:

       AFC_BAMBU_FIRMWARE

   It lists your Bambu units, each with the firmware it runs and whether a
   newer one is out. AFC reads a unit's firmware over the bus while the unit
   is online, so a unit that just came online can show "firmware not read
   yet" for a few seconds.
2. Pick the unit. The menu lists the images it can take, newest first: the
   newest two from Bambu, and any you put in the folder yourself.
3. Pick the version, read the last page, and press **Flash**.

The update takes about two minutes. The units on that bridge read offline
until it is done; units on other bridges keep working. When it ends, a popup
says how it went, such as:

    Bambu_AMS_1 now runs 05.00.22.22 (was 04.00.21.87).

and the unit comes back online on its new firmware. The whole run is logged
in `~/printer_data/logs/ams_flash.log`.

Other units may stay connected. The update refuses while a print runs or is
paused, and while a unit on the same bridge is drying.

The same without the menu:

    AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 VERSION=05.00.22.22 CONFIRM=1

## Offered updates

The first time the bridge sees a unit, AFC reads its firmware. If the unit
runs older firmware than the newest one listed, a popup offers the newest:
**Update to ...** opens the last page of the menu for it, and **Not now**
closes the popup. Each unit is offered an update once; a unit first seen on
the newest firmware is not offered one later. After `AFC_BRIDGEBOX_FORGET`,
the unit is offered again the next time the bridge sees it. During a print the
popup waits until the print ends.

## Without internet

Put Bambu's image in the folder beside `ams_flash.py`, and the menu lists it
with the others. Either kind of file works:

- the image itself, named like
  `n3f_rev5-firmware-v05.00.22.22-20260702164346_product.bin.sig`
- a Bambu printer's offline update package (`offline-ota-....zip`); the menu
  lists every AMS image in it

A file in the folder is used in place of downloading the same version.

## If it does not finish

The bootloader is never erased, so an update that stops partway leaves the
unit waiting in its bootloader, not bricked. **Keep the unit powered**: it has
no firmware to boot until an update completes. Open the menu again; it shows
**Finish ...** for that unit, which writes the same image again from the
start.

| the popup or log says | what to do |
|---|---|
| `a print is active` | Update once the print is done. |
| `... is offline` | Power the unit, wait until it shows online, and retry. |
| `... drying on the same bridge` | Stop the dry and retry. |
| `the bridge firmware is older than 175` | Update the bridge first ([BRIDGE_FLASH_HOWTO.md](../BridgeBox/BRIDGE_FLASH_HOWTO.md)). |
| `the download from Bambu failed` | The printer could not reach Bambu. Put the image in the folder ([Without internet](#without-internet)) and retry. |
| `... is for ..., but ... is ...` | The image is for another model. Pick one the menu lists for this unit. |
| `... is sitting in its bootloader` | Another unit waits in its bootloader. Finish its update, or power-cycle it. |
| `... did not enter its loader` | Nothing was erased. Check the unit shows online and retry. |
| `... stopped answering when ... entered its loader` | Nothing was erased. Power-cycle the units and retry. |
| `the update did not finish ...` | Press **Finish** in the menu. If the log shows chunk hash errors on every pass, the bus is dropping bytes: check the wiring, termination and ground. |
| `... took every block and refused the image` | The image is wrong for this unit. It stops rather than erasing again; update it with the right image. |
| `an AMS update is running` | Wait for it to end. |
| `could not start the update on its own` | See step 6 of [One-time setup](#one-time-setup). |
| `can't open file` | The `command:` path in `ams_flash.cfg` is wrong (step 4). |

## Several units

Update one unit at a time; the menu refuses a second while one runs. The other
units can stay connected.
