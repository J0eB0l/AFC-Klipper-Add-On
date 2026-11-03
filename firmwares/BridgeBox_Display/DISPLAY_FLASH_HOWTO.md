# BridgeBox display

A touchscreen that shows what the BridgeBox's AMS units hold: each bay's
color, material, weight and state, and each unit's humidity and temperature,
with the dryer controls one tap away. It runs on a **BTT K-Touch**. A **BTT
Panda Touch** is the same board under another name, so one left over from a
Bambu printer works too.

The display reads the printer through Moonraker over WiFi. It never touches the
AMS bus, so unplugging it, even mid-print, changes nothing on the printer.

## The files

Both are version AFC-1.46, in this folder, `firmwares/BridgeBox_Display/`.

| file | flash at | keeps the saved WiFi and printer | use it for |
|---|---|---|---|
| `bridgebox-full-0x0.bin` | `0x0` | no | the first flash, and recovery |
| `bridgebox-update-app-0x10000.bin` | `0x10000` | yes | every update after that |

The full image erases the saved network and printer address, so after it the
display starts in setup again.

## First flash

This replaces the stock BTT firmware on the panel. It works from the
printer's Raspberry Pi over SSH, or from any computer with Python. These steps
use esptool 5, whose commands are spelled with dashes (`flash-id`,
`write-flash`); esptool 4 spelled them with underscores.

1. **Get esptool 5.** On a Pi (or any Debian-style Linux), plain
   `pip install` is refused, so put it in its own folder:

       python3 -m venv ~/esptool-venv
       ~/esptool-venv/bin/pip install "esptool>=5,<6"

   The commands below run it as `~/esptool-venv/bin/esptool`. If the Pi
   already has `~/esptool-venv`, skip this step. On Windows or a Mac,
   `pip install "esptool>=5,<6"` and run plain `esptool` instead.

2. **Download the images** (on the Pi, or save them from this folder on
   GitHub):

       mkdir -p ~/bridgebox_display && cd ~/bridgebox_display
       wget -O bridgebox-full-0x0.bin https://raw.githubusercontent.com/J0eB0l/AFC-Klipper-Add-On/BridgeBox/firmwares/BridgeBox_Display/bridgebox-full-0x0.bin
       wget -O bridgebox-update-app-0x10000.bin https://raw.githubusercontent.com/J0eB0l/AFC-Klipper-Add-On/BridgeBox/firmwares/BridgeBox_Display/bridgebox-update-app-0x10000.bin

3. **Connect the panel's USB-C and find its port.** It is a CH340 serial
   port. On Linux list the ports by name:

       ls -l /dev/serial/by-id/

   The panel is `usb-1a86_USB_Serial-if00-port0`. A printer has other USB
   serial boards; if more than one `1a86` port is listed, unplug the panel,
   list again and take the one that went away. Never flash a port Klipper
   uses: that overwrites the board on it. On Windows it is a `COM` port (older
   Windows may need the CH340 driver), on a Mac `/dev/cu.wchusbserial...`.
   Use your port in place of `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
   below.

4. **Check it is the right board.** This only reads:

       ~/esptool-venv/bin/esptool --chip esp32s3 -p /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 flash-id

   Look for `Chip type: ESP32-S3` and `Detected flash size: 16MB`.

5. **Flash the full image.** `--erase-all` clears the whole chip first, so
   nothing of the stock firmware is left behind:

       ~/esptool-venv/bin/esptool --chip esp32s3 -p /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -b 460800 --after watchdog-reset write-flash --erase-all 0x0 bridgebox-full-0x0.bin

   It ends with `Hash of data verified.` and the panel restarts into setup.

## Setup on the panel

The panel walks through this on its first start:

1. **Choose your network.** Tap the network the printer is on.
2. **Password**, then the tick. Leave it blank for an open network. The panel
   joins the network to check the password before going on.
3. **Moonraker address**: the printer the BridgeBox is plugged into, for
   example `http://192.168.1.50`.
4. **Port**: `7125` unless you moved Moonraker.
5. **API key**: usually blank. See the table below if the panel reports
   `HTTP 401 -- API key?`.

It saves, restarts and connects. The units appear by themselves once the
printer reports them.

The screen shows one unit at a time. The **Unit** button in the top bar names
it and says which of how many it is. Tap it for a list of every unit with its
bays, then tap a unit to show it. Each bay's **T# / Runout** button sets its
tool number and its infinite-runout lane, from every lane on the printer.

Tick **All AFC units** in that list to add the printer's other AFC units (Box
Turtle, ACE, OpenAMS, EMU and the rest). Their cards show each lane and its
T# / Runout button, plus temperature and humidity when the unit reports them,
and the unit's buffer when it has one (its state, and a gauge when the buffer
has a position sensor). An ACE or ACE 2 card also has a **Dryer Menu** to start
and stop its dryer, and a lane with an AFC RFID reader has **Scan Tag**, which
reads the tag and applies it. OpenAMS, Box Turtle and ACE 2 lanes pull the
spool in past the reader and back (AFC_OAMS_RFID_SCAN, AFC_BT_RFID_STAGE,
ACE_RFID_RESCAN), so the OpenAMS or Box Turtle lane in the tool shows Tool
loaded instead, and the ACE 2 lane in the tool reads in place (ACE_RFID_READ).
ACE and ViViD lanes read in place.
A unit with more than four lanes has a **Lanes 1-4 of N** button that shows the
next four.

To move the panel to another network or printer, press **Log out** in the top
bar and confirm. It forgets both and starts setup again.

## Updating

Both ways keep the saved network and printer.

**From a browser.** The panel serves its own update page. Its address is on the
Settings sheet (the cog in the top bar). Open that address, choose
`bridgebox-update-app-0x10000.bin` on the **Panel** card and press
**Update panel**. The panel shows `Updating NN%`, then restarts on the new
version.

**Over USB**, with the port and esptool from the first flash:

    ~/esptool-venv/bin/esptool --chip esp32s3 -p /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -b 460800 --after watchdog-reset write-flash 0x10000 bridgebox-update-app-0x10000.bin

Use only the update image on the update page. It refuses the full image,
because the full image cannot boot from there.

If a new version fails to start, the panel goes back to the version it was
running on the next start. This protection comes with the full image, so it
works on any panel that had its first flash from this folder.

## What can go wrong

| the panel shows | meaning |
|---|---|
| `No WiFi` | It cannot join the saved network: out of range, or the password changed. Log out and run setup again. |
| `No answer at that address` | The Moonraker address or port is wrong, or the printer is off. |
| `HTTP 401 -- API key?` | Moonraker does not trust the panel. Either add the panel's network to `trusted_clients` under `[authorization]` in `moonraker.conf`, or log out and enter Moonraker's API key in setup. |
| no units | The panel shows the Bambu AMS units the printer reports online, up to twelve (4 AMS and 8 AMS HT). Check the units are online in Mainsail or Fluidd first. |
| nothing, or it keeps restarting, after a USB flash | Flash the full image again. |

If esptool says `Failed to connect`, try again with `-b 115200`, or another
USB-C cable (some carry power only). `Permission denied` on the port means
your user is not in the `dialout` group: `sudo usermod -aG dialout $USER`,
then log in again.
