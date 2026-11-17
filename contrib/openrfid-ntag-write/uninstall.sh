#!/bin/sh
# Put the stock OpenRFID daemon files back. RUN AS ROOT.
# Leaves the [file_write_watch] config and the request dir in place (harmless
# without the controller); remove them by hand if you want them gone.
set -e
R=/usr/local/share/openrfid
HERE=$(cd "$(dirname "$0")" && pwd)
BAK=$HERE/backup
[ "$(id -u)" = "0" ] || { echo "must run as root"; exit 1; }
[ -f "$BAK/runtime.py.orig" ] || { echo "no backup at $BAK"; exit 1; }
for f in reader/fm175xx/rfid.py reader/mifare_ultralight_reader.py \
         reader/gpio_enabled_rfid_reader.py runtime.py main.py; do
    install -m 644 -o root -g root "$BAK/$f.orig" "$R/$f"
done
rm -f "$R/controllers/file_write_watch.py"
find "$R" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true
# The daemon will fail to start if the config still names the now-removed
# controller, so drop that section too.
USER_CFG=/oem/printer_data/config/extended/openrfid_user.cfg
if [ -f "$USER_CFG" ]; then
    python3 - "$USER_CFG" <<'PY'
import sys, re
p = sys.argv[1]
s = open(p).read()
# strip our added block: the comment line(s) + [file_write_watch ...] section
s = re.sub(r"\n# Added by contrib/openrfid-ntag-write:.*?(?=\n\[|\Z)", "\n",
           s, flags=re.DOTALL)
s = re.sub(r"\n\[file_write_watch[^\]]*\][^\[]*", "\n", s)
open(p, "w").write(s)
PY
fi
/etc/init.d/S99openrfid restart >/dev/null 2>&1 || true
echo "stock daemon restored"
