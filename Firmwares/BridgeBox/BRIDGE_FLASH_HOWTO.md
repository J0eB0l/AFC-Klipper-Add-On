# Updating the bridge's firmware

The bridge (the Pico, or the ESP32-S3 on a BridgeBox S3) is updated from
Klipper over the link it already uses, with `AFC_BAMBU_FLASH`. No BOOTSEL button, no cable
swapping and no shell access are needed. Bridges ship already flashed, so this
is only for updates.

## What you need

Two files for your board, which always travel together:

| board | image | manifest |
|---|---|---|
| Pico 2 (USB bridge) | `enc_flash_pico2.uf2` | `enc_flash_pico2.uf2.manifest.json` |
| ESP32-S3 (BridgeBox S3, USB or WiFi) | `signed_s3.bin` | `signed_s3.bin.manifest.json` |

The Pico 2 pair is included in this folder, `Firmwares/BridgeBox/`. It is
AFC-2.82.

The S3 pair is included too. It is AFC-2.83, for a BridgeBox S3 as shipped
(it is signed, and the board runs only signed images). AFC needs to
understand the `.bin`: an AFC too old for it answers `block 0 has no UF2
magic`.

Use the pair made for your board. The manifest is the signature: without it
the bridge refuses the update.

**`bridgebox_flash.cfg`** (in this folder) gives you one command for all three
boards. Upload it to your config folder, add `[include bridgebox_flash.cfg]` to
`printer.cfg` (or put it where your config already includes `*.cfg`), and
RESTART once. It only calls AFC's own `AFC_BAMBU_FLASH`: no shell access, no
paths to edit. Without it, the `AFC_BAMBU_FLASH` lines below do the same.

| command | does |
|---|---|
| `BRIDGEBOX_FLASH BOARD=pico2` | check: transfer and verify, then discard |
| `BRIDGEBOX_FLASH BOARD=pico2 MODE=go` | update: write and reboot |
| `BRIDGEBOX_FLASH_ABORT` | clear a stuck transfer |

`BOARD` is `pico2` or `s3`, and picks the file from the table above
(`FILE=` overrides it). With more than one Bambu AMS unit in your config, add
`UNIT=` with any unit on the bridge you mean, e.g. `UNIT=Bambu_AMS_1`; the
command lists your units if you leave it out.

## Update

1. **Make sure the printer is idle.** The command refuses to run during a
   print.

2. **Note the current version**:

       AFC_BAMBU_UIDS

   The firmware line reads something like `AFC-2.81`.

3. **Upload both files** into your Klipper config folder, the one that holds
   `printer.cfg`. In Mainsail or Fluidd that is the Machine tab's file list;
   upload into the top level, not a subfolder.

4. **Dry run.** This transfers and checks the image, then throws it away. The
   bridge keeps running what it had:

       BRIDGEBOX_FLASH BOARD=pico2 UNIT=Bambu_AMS_1

   or, without the cfg:

       AFC_BAMBU_FLASH UNIT=Bambu_AMS_1 FILE=enc_flash_pico2.uf2 APPLY=0

   On an S3 bridge, `BOARD=s3` (or `FILE=signed_s3.bin`). AFC reads the chip from the file
   and refuses to send a Pico image to an S3, or an S3 image to a Pico.

   `UNIT=` can be any Bambu AMS unit on that bridge; it picks the bridge, not
   the AMS. Look for:

       AFC_BAMBU_FLASH: signed manifest vNNN accepted
       AFC_BAMBU_FLASH: APPLY=0 -- the image transferred and verified, and was then discarded.

5. **Apply**:

       BRIDGEBOX_FLASH BOARD=pico2 UNIT=Bambu_AMS_1 MODE=go

   or, without the cfg, the same `AFC_BAMBU_FLASH` line without `APPLY=0`:

       AFC_BAMBU_FLASH UNIT=Bambu_AMS_1 FILE=enc_flash_pico2.uf2

   The bridge writes its own flash and reboots. The link drops for about ten
   seconds and the AMS units go offline, then come back by themselves. The
   usual last message is `apply sent; the bridge stopped answering without
   confirming`. That is normal: the bridge reboots before its reply gets out.

6. **Confirm** once the units are back:

       AFC_BAMBU_UIDS

   The firmware line should show the new version. That line is the proof;
   nothing else is.

Always pass `FILE=`. Without it the command looks for a file called
`.bridge_firmware.uf2`, which is not what you uploaded.

## What can go wrong

Nothing on the bridge is erased until the whole image has arrived and its
checksum matches, so almost every failure leaves it running the old firmware.

| message | meaning |
|---|---|
| `the bridge rejected the manifest` | The signature does not match, or the image is older than what the bridge runs (it will not go backwards). Nothing changed. |
| `the bridge refused the image` / `would not start` | The bridge turned it down before writing anything. Nothing changed; the message says why. |
| `no .manifest.json beside the image` | The manifest was not uploaded, or has a different name. Upload it next to the image. |
| `the link died mid-transfer` / `the image did not verify` | The transfer was interrupted. Nothing changed; run it again. |
| `busy: a transfer is already running` | A previous attempt is stuck. Clear it with `BRIDGEBOX_FLASH_ABORT` (or `AFC_BAMBU_FLASH UNIT=Bambu_AMS_1 ABORT=1`), then run it again. |
| the version did not change | The apply did not take. Run step 5 again. |

The one risky moment is the second or two while the bridge writes its own
flash in step 5. If power is lost exactly then, the bridge will not start
again by itself. To recover a USB bridge:

1. Unplug it, hold the BOOTSEL button (the case has a hole over it) and plug
   it into a computer.
2. It shows up as a drive named `RP2350`.
3. Copy `enc_flash_pico2.uf2` onto that drive. The bridge restarts on the new
   image by itself.
4. Plug it back into the printer and confirm with `AFC_BAMBU_UIDS`.

An S3 is harder to break this way: the new image goes into its spare slot,
and the old one keeps running until the new one has been checked, so a power
cut mid-write leaves it on the old firmware. An S3 has no BOOTSEL drive; if
one ever will not start, ask for it to be recovered.

## From the BridgeBox display

A BridgeBox display (panel version AFC-1.30 or later) can do the same update
for a Pico 2 on USB from its own update page, with the same two files and a
Check only dry run. It does not update an S3; use the steps
above. See
[DISPLAY_FLASH_HOWTO.md](../BridgeBox_Display/DISPLAY_FLASH_HOWTO.md) in
`Firmwares/BridgeBox_Display/`.

## Updating the AMS units themselves

That is a separate procedure with its own commands: see
[AMS_FLASH_HOWTO.md](../Bambu_AMS/AMS_FLASH_HOWTO.md) in `Firmwares/Bambu_AMS/`.
