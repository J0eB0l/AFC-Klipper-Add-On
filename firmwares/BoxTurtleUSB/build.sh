#!/bin/bash
# Builds the BoxTurtle standalone firmware on top of a pinned Klipper tree.
#   ./build.sh            -> out/boxturtle.bin (app at 0x08020000) and its manifest, .json
#   KLIPPER_DIR=... ./build.sh  uses an existing Klipper checkout instead
# Needs gcc-arm-none-eabi and libnewlib-arm-none-eabi.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
KLIPPER_COMMIT=af81d23
WORK="${WORK:-$HERE/.build}"
KDIR="${KLIPPER_DIR:-$WORK/klipper}"

if [ ! -d "$KDIR/.git" ]; then
    mkdir -p "$WORK"
    git clone -q https://github.com/Klipper3d/klipper.git "$KDIR"
fi
git -C "$KDIR" fetch -q origin 2>/dev/null || true
git -C "$KDIR" checkout -q -f "$KLIPPER_COMMIT"
git -C "$KDIR" clean -qfdx src
git -C "$KDIR" apply "$HERE/klipper.patch"
rm -rf "$KDIR/src/turtle"
cp -r "$HERE/src/turtle" "$KDIR/src/turtle"
cp "$HERE/defconfig" "$KDIR/.config"
make -C "$KDIR" olddefconfig >/dev/null
make -C "$KDIR" clean >/dev/null
make -C "$KDIR" -j"$(nproc)"
mkdir -p "$HERE/out"
cp "$KDIR/out/klipper.bin" "$HERE/out/boxturtle.bin"
cp "$KDIR/out/klipper.elf" "$HERE/out/boxturtle.elf"
# The manifest AFC_TURTLE_FLASH and the OrcaSlicer Update page read: version, protocol, size, hash.
python3 - "$HERE/out/boxturtle.bin" "$HERE/src/turtle/board.h" <<'PY'
import hashlib, json, re, sys
image = open(sys.argv[1], "rb").read()
board = open(sys.argv[2]).read()
manifest = {"format": "BTFW1",
            "version": re.search(r'#define FW_VERSION "([^"]+)"', board).group(1),
            "proto": re.search(r'#define FW_PROTO "([^"]+)"', board).group(1),
            "len": len(image), "sha256": hashlib.sha256(image).hexdigest()}
open(sys.argv[1] + ".json", "w").write(json.dumps(manifest, indent=2) + "\n")
PY
ls -l "$HERE/out/boxturtle.bin" "$HERE/out/boxturtle.bin.json"
