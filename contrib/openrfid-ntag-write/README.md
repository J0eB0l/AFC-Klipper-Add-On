# OpenRFID: NTAG/Ultralight page write (`0xA2`)

Adds writing to the FM175xx driver in
[OpenRFID](https://github.com/suchmememanyskill/OpenRFID), so a **Snapmaker U1**
can program blank NTAG stickers with its own built-in reader: the thing needed
to tag a third-party spool.

MIFARE Classic writing is already there (`__reader_a_m1_block_write`, `0xA0`).
NTAG is read-only: `__reader_a_ultralight_page_read` exists, the matching write
does not. The NTAG215 geometry constants are already defined too
(`FM175XX_NTAG215_USER_START_PAGE` / `_USER_END_PAGE`), so this is the missing
half of a feature the driver is otherwise set up for.

## Why we want it

Our AFC fork writes Anycubic-layout records to blank NTAG215 stickers through
any host-side MFRC522-class reader (BoxTurtle, ACE2, ViViD, OpenAMS), so a
generic spool scans like a branded one. The U1's reader is the same chip family,
an FM17580, but it belongs to the OpenRFID daemon rather than to Klipper, so
the U1 is the one unit that can read those stickers and not make them. This
patch closes that gap at the source, rather than us fighting the daemon for the
SPI bus.

## What it adds

`src/reader/mifare_ultralight_reader.py`

- `write_mifare_ultralight(scan_result, start_page, data) -> bool`

  Deliberately **not** `@abstractmethod`: adding an abstract method would make
  every existing `MifareUltralightReader` subclass uninstantiable, including
  `GpioEnabledRfidReader`. The default returns `False`, so a reader that cannot
  write says so and nothing else breaks.

`src/reader/fm175xx/rfid.py`

- `__reader_a_ultralight_page_write(page, buff)`: the wire op. Unlike the
  Classic `0xA0` write, which sends command and data as two transceives, `0xA2`
  carries the page and its 4 bytes in one frame. Both answer with the same 4-bit
  `0x0A` ACK and no CRC, so the reply handling matches the existing Classic
  write exactly.
- `__reader_a_ultralight_write_all_data(start_page, data, retry_times=3)`:
  bounds check, write with retries, then read back and compare.
- `write_mifare_ultralight(...)`: the public override, shaped like
  `read_mifare_ultralight`.

## The write endpoint (what makes AFC_RFID_WRITE reach the U1)

The page write above is the wire op. On its own it lets code *in the daemon*
write a tag; it does not let AFC, which runs as a different user and cannot
touch the SPI bus, ask for one. The rest of this patch is that bridge:

`src/reader/gpio_enabled_rfid_reader.py` -- delegates `write_mifare_ultralight`
to the underlying FM175xx, the way it already delegates the reads. The U1's four
slot readers are these wrappers, so without this the write never reaches the
chip.

`src/runtime.py` -- a `write_ntag(slot, start_page, data)` that scans to confirm
a tag is present and is Ultralight-family, then writes and reads back, all under
a new lock the scan loop also takes so the two never touch the one SPI bus at
once.

`src/controllers/file_write_watch.py` -- a controller that watches a directory
for `req-<token>.json`, hands each to `write_ntag`, and answers with
`res-<token>.json`. The request is removed before the write (so a crash cannot
re-fire it) and results are swept after a TTL (the requester is another user and
cannot delete our files). `src/main.py` registers it as `file_write_watch`.

AFC's side is `extras/AFC_U1_rfid.py`, which registers each U1 channel into
`AFC_RFID_WRITE` with a payload hand-off that drives this file protocol.

### Installing the whole thing on a U1

`install.sh` puts in both halves and wires the config. It needs **root**
(everything under `/usr/local/share/openrfid` is root-owned, no sudo, and the
overlay upper is root-owned too); root is reachable over dropbear with
`/oem/.debug` present:

```sh
sh /oem/printer_data/config/AFC/openrfid-ntag-write/install.sh
```

It refuses unless all five daemon files hash to the exact stock versions it was
built against, backs them up beside itself, drops in the patched files and the
new controller, adds `[file_write_watch afc]` to `openrfid_user.cfg` (the
writable user config the daemon already reads last), creates the shared request
directory, restarts the daemon and confirms it came back. `uninstall.sh`
reverses all of it.

### Testing the endpoint

`test_write_watch.py` drives the controller and `runtime.write_ntag` against a
simulated reader over a temp directory -- the full request/result round trip,
the no-tag and Classic-refusal paths, the crash-safety of removing the request
first, and that the lock serialises against a scan. Run it like
`test_ntag_write.py`, from the OpenRFID checkout. 16 checks, all pass; combined
with the page-write test that is 33 green with no hardware.

## Two things worth reviewing closely

**The bounds check is the point.** Pages below `USER_START_PAGE` are the UID,
the one-way static lock bits and the capability container; pages above
`USER_END_PAGE` are the dynamic lock bits, the configuration and the password.
Those are fuses, not storage. A caller's page arithmetic being off by a little
does not produce a wrong tag, it produces a permanently dead one, so the range
is enforced in the driver rather than trusted from above.

**The read-back is not belt-and-braces.** A tag pulled out of the field
part-way through will ACK some pages and store nothing, and without comparing
the result that is indistinguishable from success.

## Testing

`test_ntag_write.py` runs the driver against a simulated NTAG215 substituted for
`__command_exe`, so it needs no hardware and no `spidev`:

```bash
git clone https://github.com/suchmememanyskill/OpenRFID.git
cd OpenRFID
git apply /path/to/openrfid-ntag-write.patch
cp /path/to/test_ntag_write.py .
python3 test_ntag_write.py
```

It covers a clean 36-page write, the reserved pages being refused with nothing
written, running past the user area, the last legal page being allowed, payloads
that are not whole pages, a tag that NAKs, and a tag that ACKs while storing
nothing. All 17 checks pass.

**Not yet run on real silicon.** The frame is modelled on the driver's own
Classic write and on NTAG21x `WRITE`, and verified against a simulator; it has
not been put in front of an actual sticker on a U1. That is the remaining step
before this is worth proposing upstream as finished.

## Installing it on a U1 right now

`install.sh` / `uninstall.sh` do this in one command. Upload the folder to
`/oem/printer_data/config/AFC/openrfid-ntag-write/` through Moonraker's file
API (that directory is writable by the `lava` user Klipper runs as).

The install itself needs **root**: everything under `/usr/local/share/openrfid`
is root-owned, there is no `sudo` on the box, and the overlay's upper directory
(`/oem/overlay/upper`) is root-owned too, so `lava` has no write path to it at
all. Root is reachable over dropbear (`S50dropbear`, with `/oem/.debug`
present). From a root shell:

```sh
sh /oem/printer_data/config/AFC/openrfid-ntag-write/install.sh
```

The installer refuses to run unless the two files on disk hash to the exact
stock versions it was built against, so it cannot silently downgrade a box
running something else. It backs up to `backup/` beside itself, drops the
patched files in, clears the stale `__pycache__` that would otherwise shadow
them, import-checks the result, restarts the daemon and confirms it came back --
rolling the message back to `uninstall.sh` if it did not.

`/` is an overlay (`upperdir=/oem/overlay/upper`), so the change copies up there
and survives reboots. A firmware update may replace it; re-run after one.

## Applying

```bash
git apply openrfid-ntag-write.patch
```

Against `suchmememanyskill/OpenRFID` at `1a6f605` ("Align RFID reset timeouts
with Snapmaker U1 v1.4.0 firmware (#23)"), which is byte-identical to the copy
shipped in the Snapmaker U1 Extended Firmware at
`/usr/local/share/openrfid/`. Upstream is GPL-3.0.

## Upstream

Related discussion: OpenRFID feedback is collected in
[SnapmakerU1-Extended-Firmware#366](https://github.com/paxx12-snapmaker-u1/SnapmakerU1-Extended-Firmware/issues/366),
and RFID/Spoolman work in
[#401](https://github.com/paxx12-snapmaker-u1/SnapmakerU1-Extended-Firmware/issues/401).
The patch itself belongs to
[suchmememanyskill/OpenRFID](https://github.com/suchmememanyskill/OpenRFID).
