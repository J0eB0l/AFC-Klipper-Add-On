# BridgeBox display

A touchscreen that shows what the BridgeBox's AMS units hold: each bay's
colour, material, weight and state, and each unit's humidity and temperature,
with the dryer controls one tap away. It runs on a **BTT K-Touch**. A **BTT
Panda Touch** is the same board under another name, so one left over from a
Bambu printer works too.

The display reads the printer through Moonraker over WiFi. It never touches the
AMS bus, so unplugging it, even mid-print, changes nothing on the printer.

## The files

Both are version AFC-1.46, in this folder, `Firmwares/BridgeBox_Display/`.

| file | flash at | keeps the saved WiFi and printer | use it for |
|---|---|---|---|
| `bridgebox-full-0x0.bin` | `0x0` | no | the first flash, and recovery |
| `bridgebox-update-app-0x10000.bin` | `0x10000` | yes | every update after that |

The full image erases the saved network and printer address, so after it the
display starts in setup again.

## First flash

This replaces the stock BTT firmware on the panel. You need a computer with
Python and esptool (`pip install esptool`).

1. **Connect the panel's USB-C to the computer.** It shows up as a CH340
   serial port: `/dev/ttyUSB0` on Linux, a `COM` port on Windows (older
   Windows may need the CH340 driver), `/dev/cu.wchusbserial...` on a Mac.
   Use your port in place of `/dev/ttyUSB0` below.

2. **Check it is the right board.** This only reads:

       esptool.py -p /dev/ttyUSB0 flash_id

   Look for `Chip is ESP32-S3` and a 16 MB flash.

3. **Flash the full image:**

       esptool.py -p /dev/ttyUSB0 -b 460800 write_flash 0x0 bridgebox-full-0x0.bin

   The panel restarts into setup.

Newer esptool installs name the command `esptool` instead of `esptool.py`; the
rest of the line is the same.

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

**Over USB:**

    esptool.py -p /dev/ttyUSB0 -b 460800 write_flash 0x10000 bridgebox-update-app-0x10000.bin

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
