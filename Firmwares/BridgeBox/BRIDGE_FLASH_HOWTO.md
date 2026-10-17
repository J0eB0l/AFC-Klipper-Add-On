# Updating the BridgeBox's firmware

The BridgeBox is updated from Klipper over the link it already uses, USB or
WiFi, with `AFC_BAMBU_FLASH`. No buttons, no cable swapping and no shell
access are needed. BridgeBoxes ship already flashed, so this is only for
updates.

## What you need

Two files, which always travel together:

| image | manifest |
|---|---|
| `signed_s3.bin` | `signed_s3.bin.manifest.json` |

Both are included in this folder, `Firmwares/BridgeBox/`. They are AFC-2.83.
The BridgeBox runs only signed images, and the manifest is the signature:
without it the BridgeBox refuses the update. AFC needs to understand the
`.bin`: an AFC too old for it answers `block 0 has no UF2 magic`.

**`bridgebox_flash.cfg`** (in this folder) gives you one command for it.
Upload it to your config folder, add `[include bridgebox_flash.cfg]` to
`printer.cfg` (or put it where your config already includes `*.cfg`), and
RESTART once. It only calls AFC's own `AFC_BAMBU_FLASH`: no shell access, no
paths to edit. Without it, the `AFC_BAMBU_FLASH` lines below do the same.

| command | does |
|---|---|
| `BRIDGEBOX_FLASH` | check: transfer and verify, then discard |
| `BRIDGEBOX_FLASH MODE=go` | update: write and reboot |
| `BRIDGEBOX_FLASH_ABORT` | clear a stuck transfer |

`FILE=` picks a different file. With more than one Bambu AMS unit in your
config, add `UNIT=` with any unit on the BridgeBox you mean, e.g.
`UNIT=Bambu_AMS_1`; the command lists your units if you leave it out.

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
   BridgeBox keeps running what it had:

       BRIDGEBOX_FLASH UNIT=Bambu_AMS_1

   or, without the cfg:

       AFC_BAMBU_FLASH UNIT=Bambu_AMS_1 FILE=signed_s3.bin APPLY=0

   `UNIT=` can be any Bambu AMS unit on that BridgeBox; it picks the
   BridgeBox, not the AMS. Look for:

       AFC_BAMBU_FLASH: signed manifest vNNN accepted
       AFC_BAMBU_FLASH: APPLY=0 -- the image transferred and verified, and was then discarded.

5. **Apply**:

       BRIDGEBOX_FLASH UNIT=Bambu_AMS_1 MODE=go

   or, without the cfg, the same `AFC_BAMBU_FLASH` line without `APPLY=0`:

       AFC_BAMBU_FLASH UNIT=Bambu_AMS_1 FILE=signed_s3.bin

   The BridgeBox writes its own flash and reboots. The link drops for about
   ten seconds and the AMS units go offline, then come back by themselves.
   The usual last message is `apply sent; the bridge stopped answering
   without confirming`. That is normal: the BridgeBox reboots before its
   reply gets out.

6. **Confirm** once the units are back:

       AFC_BAMBU_UIDS

   The firmware line should show the new version. That line is the proof;
   nothing else is.

Always pass `FILE=` to `AFC_BAMBU_FLASH`. Without it the command looks for a
file called `.bridge_firmware.uf2`, which is not what you uploaded.

## What can go wrong

Nothing on the BridgeBox is erased until the whole image has arrived and its
checksum matches, so almost every failure leaves it running the old firmware.

| message | meaning |
|---|---|
| `the bridge rejected the manifest` | The signature does not match, or the image is older than what the BridgeBox runs (it will not go backwards). Nothing changed. |
| `the bridge refused the image` / `would not start` | The BridgeBox turned it down before writing anything. Nothing changed; the message says why. |
| `no .manifest.json beside the image` | The manifest was not uploaded, or has a different name. Upload it next to the image. |
| `the link died mid-transfer` / `the image did not verify` | The transfer was interrupted. Nothing changed; run it again. |
| `busy: a transfer is already running` | A previous attempt is stuck. Clear it with `BRIDGEBOX_FLASH_ABORT` (or `AFC_BAMBU_FLASH UNIT=Bambu_AMS_1 ABORT=1`), then run it again. |
| the version did not change | The apply did not take. Run step 5 again. |

A power cut during step 5 is safe too: the new image goes into the
BridgeBox's spare slot, and the old one keeps running until the new one has
been checked, so it comes back on the old firmware. If a BridgeBox ever will
not start, ask for it to be recovered.

## Updating the AMS units themselves

That is a separate procedure with its own commands: see
[AMS_FLASH_HOWTO.md](../Bambu_AMS/AMS_FLASH_HOWTO.md) in `Firmwares/Bambu_AMS/`.
