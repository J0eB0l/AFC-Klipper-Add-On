# BridgeHub

A BridgeHub puts a USB serial unit on the network. It is an ESP32-S3 board
whose own USB port runs as a USB host. Plug a self-powered unit into it, such
as a BoxTurtle running the BoxTurtleUSB firmware, and the hub serves the
unit's serial bytes raw over WiFi on TCP port 4001. AFC then talks to the
unit as if it were plugged into the printer's host.

## What you need

- An ESP32-S3 board with 4 MB of flash or more that brings the S3's own USB
  port out to a connector. On an ESP32-S3-DevKitC-1 that is the port marked
  USB; the other port (UART) powers the board and shows its console.
- A USB cable from that port to the unit. The hub puts no 5 V on the port,
  so the unit needs its own supply (a BoxTurtle on its PSU).

## Flash the hub

`bridgehub.bin` is a plain build of this folder (no secure boot, no flash
encryption). Check it with `sha256sum -c bridgehub.bin.sha256`, then write
it at offset 0 through the board's UART port:

```
pip install esptool
esptool.py --chip esp32s3 -p /dev/ttyUSB0 write_flash 0x0 bridgehub.bin
```

If esptool cannot connect, hold BOOT, tap RESET, and try again.

To build it yourself, with ESP-IDF 5.3: `idf.py build`, then
`idf.py -p <port> flash`. The build fetches espressif/usb_host_cdc_acm 2.0.6.

## Join it to your WiFi

1. Power the hub. With no network saved, it raises an open hotspot named
   `BridgeHub-XXXX`.
2. Join that hotspot with a phone or laptop and open http://192.168.4.1.
3. Pick your network (2.4 GHz), type its password and tap Join. The page
   shows the address the hub got and the line to put in AFC. The hub then
   restarts onto your network and the hotspot goes away.
4. Reserve that address for the hub in your router, so it never changes.

Hold BOOT for 10 seconds to forget the network and the link key and bring
the hotspot back.

The UART port prints the hub's state as JSON lines at 115200 baud: its
address, and each unit as it comes and goes.

## Use it in AFC

Name the hub's address as the unit's `serial_port`:

```
[AFC_BoxTurtleUSB Turtle_1]
serial_port: tcp://192.168.1.50
```

An ACE takes the same line in its `[AFC_ACE ...]` or `[AFC_ACE2 ...]`
section.

AFC takes the hub's port, 4001, when none is given; `tcp://<ip>:<port>`
names another, as for a raw TCP serial server such as ser2net. Use the IP
address, not a host name: AFC checks it at startup, and a name lookup would
stall Klipper. The rest of the unit's config stays as it would be over USB.
If the hub or its WiFi drops for a moment, AFC reconnects on its own.

## The link key

There is nothing to type. The first time AFC reaches a new hub, it makes a
random link key, saves it in `AFC.var.hub` next to AFC's `AFC.var`, and
stores it on the hub. From then on the hub challenges every connection and
lets in only one that proves it holds the key, so nothing else on the
network can talk to the unit or push AFC off the hub.

- To pick the key yourself, set `tcp_key: <key>` in the unit's section
  before AFC first reaches the hub: 1 to 64 characters, with no spaces,
  quotes or backslashes. AFC then gives the hub that key instead of making
  one.
- To move the hub to another printer, or after AFC lost its key (a fresh
  install), hold BOOT for 10 seconds. That clears the key along with the
  network; join the hub to your WiFi again and the next AFC to connect
  stores its own key. Or copy the key from the old printer's `AFC.var.hub`
  into the new printer's `tcp_key`.
- A hub with no key (new, or after BOOT) takes the first key it is given,
  so bring AFC up soon after the hub joins your network.
- The key proves who connects. The unit's bytes still cross the network
  unencrypted, as they would cross a USB cable.

## How it behaves

- One unit per hub, and one TCP client at a time: a new connection that
  passes the key check replaces the old one. Point only one printer at a
  hub.
- Raw TCP carries no baud rate, so the hub picks one: 230400 for a WCH
  (1a86) serial chip, as an ACE uses, and 115200 for anything else. A
  BoxTurtleUSB ignores the speed.
- The connection stays up while the unit is unplugged and plugged back in.

## Update a BoxTurtleUSB through the hub

`hub_katapult.py` flashes the unit's firmware over the hub with Katapult's
flashtool. Stop Klipper first, since AFC holds the hub's port and would take
it back in the middle of the flash:

```
sudo systemctl stop klipper
python3 hub_katapult.py 192.168.1.50 ../BoxTurtleUSB/boxturtle.bin --unit Turtle_1
sudo systemctl start klipper
```

`--unit` names the AFC unit whose link key to use, read from
`~/printer_data/config/AFC/AFC.var.hub` (`--key-file` names another file);
`--key <key>` takes the key as typed instead. A hub with no key needs
neither.

The tool asks the running firmware into Katapult, runs flashtool over the
same connection, then asks the new build for its info, which also passes the
new build's boot guard. A unit already sitting in Katapult is flashed the
same way. It needs pyserial and Katapult's `flashtool.py`, by default at
`~/katapult/scripts/flashtool.py` (`--flashtool` names another).

## Source

`main/main.c` starts the hub, `main/hub.c` runs the USB host and the TCP
port, `main/linkkey.c` checks the link key, and `main/wifi.c` joins the
network or serves the setup page. `make -C test && test/test_linkkey` runs
the link key's host test (it needs OpenSSL). GPLv3.
