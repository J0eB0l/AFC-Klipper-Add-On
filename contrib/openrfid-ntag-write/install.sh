#!/bin/sh
# Install the OpenRFID NTAG write support onto a Snapmaker U1. RUN AS ROOT.
#
#   sh /oem/printer_data/config/AFC/openrfid-ntag-write/install.sh
#
# Two things go in:
#   1. NTAG/Ultralight page write (0xA2) in the FM175xx driver, plus the write
#      delegation through the gpio reader wrapper.
#   2. A file-watch controller that lets AFC (running as another user) ask the
#      daemon to program a tag: AFC drops a request file, the daemon writes it
#      and answers with a result file.
#
# It refuses to run unless the daemon files on disk are the exact stock versions
# this was built against, so it cannot silently downgrade or break a box running
# something else. / is an overlay (upperdir=/oem/overlay/upper), so the change
# survives reboots. A firmware update may replace it; re-run after one.
set -e
R=/usr/local/share/openrfid
HERE=$(cd "$(dirname "$0")" && pwd)
BAK=$HERE/backup
USER_CFG=/oem/printer_data/config/extended/openrfid_user.cfg
WRITE_DIR=/oem/printer_data/config/.afc_u1_write

# rel-path -> "stock_md5 patched_md5"
STOCK_rfid=46554a825f61e03fccfdedeab2afb5ba
PATCHED_rfid=ee5627632a69372964d791bc69a68399
STOCK_iface=1e89aa90c3eaa86bd2769e381d9e69ef
PATCHED_iface=645c267dde151828d22d01c325b801a0
STOCK_gpio=c25a754fda455641131f24365e8071ce
PATCHED_gpio=4fdb1950276d3e430af0c591e0ff5945
STOCK_runtime=451f486a5ff13b3ade5ff38af77313a4
PATCHED_runtime=0a1f6936883eeea71c071847269edf61
STOCK_main=80c3f7262d93e755aad32f7395d16ca8
PATCHED_main=b29094b544f23b6f28fab824903767b4

md5of() { md5sum "$1" 2>/dev/null | cut -d' ' -f1; }

[ "$(id -u)" = "0" ] || { echo "must run as root (/usr/local is root-owned)"; exit 1; }

# --- gate: every target must be either already-patched or exactly stock ------
check() {  # $1 relpath  $2 stock  $3 patched
    cur=$(md5of "$R/$1")
    if [ "$cur" = "$3" ]; then echo "already"; return 0; fi
    if [ "$cur" = "$2" ]; then echo "stock";   return 0; fi
    echo "OTHER ($cur)"; return 1
}
bad=0
for row in \
    "reader/fm175xx/rfid.py $STOCK_rfid $PATCHED_rfid" \
    "reader/mifare_ultralight_reader.py $STOCK_iface $PATCHED_iface" \
    "reader/gpio_enabled_rfid_reader.py $STOCK_gpio $PATCHED_gpio" \
    "runtime.py $STOCK_runtime $PATCHED_runtime" \
    "main.py $STOCK_main $PATCHED_main"; do
    set -- $row
    state=$(check "$1" "$2" "$3") || bad=1
    echo "  $1: $state"
done
if [ "$bad" = "1" ]; then
    echo "REFUSING: a daemon file is neither stock nor this patch. This build"
    echo "targets OpenRFID at 1a6f605 (shipped U1 firmware). Not touching it."
    exit 1
fi

echo "backing up to $BAK"
mkdir -p "$BAK/reader/fm175xx" "$BAK/controllers"
for f in reader/fm175xx/rfid.py reader/mifare_ultralight_reader.py \
         reader/gpio_enabled_rfid_reader.py runtime.py main.py; do
    cp -p "$R/$f" "$BAK/$f.orig"
done

echo "installing files"
install -m 644 -o root -g root "$HERE/files/reader/fm175xx/rfid.py"            "$R/reader/fm175xx/rfid.py"
install -m 644 -o root -g root "$HERE/files/reader/mifare_ultralight_reader.py" "$R/reader/mifare_ultralight_reader.py"
install -m 644 -o root -g root "$HERE/files/reader/gpio_enabled_rfid_reader.py" "$R/reader/gpio_enabled_rfid_reader.py"
install -m 644 -o root -g root "$HERE/files/runtime.py"                         "$R/runtime.py"
install -m 644 -o root -g root "$HERE/files/main.py"                            "$R/main.py"
install -m 644 -o root -g root "$HERE/files/controllers/file_write_watch.py"    "$R/controllers/file_write_watch.py"
# Stale bytecode would shadow the new source.
find "$R" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true

# --- config: add the write-watch controller to the user cfg (writable) -------
if ! grep -q "^\[file_write_watch" "$USER_CFG" 2>/dev/null; then
    echo "adding [file_write_watch] to $USER_CFG"
    cat >> "$USER_CFG" <<CFG

# Added by contrib/openrfid-ntag-write: lets AFC_RFID_WRITE program blank NTAG
# stickers through this daemon. request_dir must match AFC_U1_rfid's
# openrfid_write_dir (same default).
[file_write_watch afc]
request_dir = $WRITE_DIR
poll_interval_seconds = 0.5
CFG
else
    echo "[file_write_watch] already in $USER_CFG"
fi
# The dir must be writable by the Klipper user (klippy runs as a normal user);
# create it world-writable+sticky so either side can drop files.
mkdir -p "$WRITE_DIR"
chmod 1777 "$WRITE_DIR" 2>/dev/null || true

echo "compile check"
( cd "$R" && python3 -c "
import sys; sys.path.insert(0, '.')
import reader.fm175xx.rfid as m
assert hasattr(m.Fm175xx, 'write_mifare_ultralight'), 'write method missing'
import controllers.file_write_watch as c
assert hasattr(c, 'FileWriteWatchController'), 'controller missing'
import runtime as rt
assert hasattr(rt.Runtime, 'write_ntag'), 'runtime.write_ntag missing'
print('  ok')
" )

echo "restarting openrfid"
/etc/init.d/S99openrfid restart >/dev/null 2>&1 || true
sleep 2
if [ -f /var/run/openrfid.pid ] && kill -0 "$(cat /var/run/openrfid.pid)" 2>/dev/null; then
    echo "openrfid running as pid $(cat /var/run/openrfid.pid)"
else
    echo "WARNING: openrfid did not come back. See /oem/printer_data/logs/openrfid.log"
    echo "Undo with: sh $HERE/uninstall.sh"
    exit 1
fi
echo
echo "installed. The daemon now watches $WRITE_DIR."
echo "Undo with: sh $HERE/uninstall.sh"
