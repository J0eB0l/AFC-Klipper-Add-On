#!/usr/bin/env python3
"""
AMS firmware updater for the Bambu-AMS bridge.

Writes one of Bambu's own AMS images into one AMS 1, AMS 2 Pro or AMS HT on the
bus, through the bridge. The image is Bambu's signed *.bin.sig, either
downloaded from Bambu when the update runs (ams_firmware.json beside this script
lists them), or a .bin.sig or Bambu offline update package (.zip) put beside
this script, for a printer without internet. Other units may stay connected,
and Klipper keeps running: it lends this run the unit's bridge link.

  ams_flash.py --unit Bambu_AMS_1 --version 04.00.21.87 --mode go
  ams_flash.py --unit Bambu_AMS_1 --file <.bin.sig or .zip> --mode go

Without --mode go it only runs the checks and changes nothing. Progress goes to
logs/ams_flash.log and how the run ended to ams_flash_result.json.
"""
from __future__ import annotations

import argparse
import configparser
import hashlib
import hmac
import io
import json
import os
import re
import socket
import sys
import time
import urllib.parse
import urllib.request
import zipfile

# Constants (all confirmed against real printer<->AMS captures)
#: Bridge fw floor.
TARGET_FW = 175
CONFIRM = "ERASE-AMS-FW"        # the firmware's fwreplay confirm token
CHUNK = 100                     # hex-payload bytes per txbuf line
CRC8_POLY, CRC8_INIT = 0x39, 0x66
CRC16_POLY, CRC16_INIT = 0x1021, 0x913D
MOON = "http://127.0.0.1:7125"
# The loader's OWN verdict, read from the block-send reply windows (fw >= 1.74).
SUCCESS_HEX = "7375636365737321"       # "success!"
CHUNKERR_HEX = "6368756e6b2068617368"  # "chunk hash" (per-chunk hash failure)
# "img hash": every chunk was accepted and the assembled image still failed the loader's whole-image
# check.
IMGERR_HEX = "696d672068617368"
FLASH_RETRIES = 6               # whole-flash resends after the first attempt (7 total)
BADIMG = 3                      # do_flash: the loader refused the image itself
STATE_NAME = "ams_flash_state.json"    # the unit this run sent into its loader
RESULT_NAME = "ams_flash_result.json"  # how the last run ended, for the Flash AMS menu
CATALOG_NAME = "ams_firmware.json"     # Bambu's images and where Bambu serves them
ONLINE_SETTLE_S = 3.5           # two of the bridge's 1.5 s online timeouts
REJOIN_S = 90                   # a flashed unit boots its new firmware and answers again
#: How long Klipper lends this run the bridge link at most; it takes it back sooner when told.
HOLD_S = 1800
#: The printer's own bus address; its loader and version frames come from here.
HOST = 0x0900
BOXED, HT = 0x0700, 0x1800
#: Bambu's image name prefix -> (model key, model name, bus device).
FAMILIES = {
    "ams": ("ams1", "AMS 1", BOXED),
    "n3f": ("ams2", "AMS 2 Pro", BOXED),
    "n3s": ("ht", "AMS HT", HT),
}
#: The board each model has shipped on, for a unit whose model code was never read. An AMS 1 has
#: two, so it has none here.
DEFAULT_BOARD = {"ams2": "n3f_rev5", "ht": "n3s_rev5"}
REGION = 0x2D000                # a data block's region size, the same in every captured update
BLOCK = 0x400                   # image bytes per data block
USER_AGENT = "ams_flash/2"
#: Bambu's image names, e.g. n3f_rev5-firmware-v05.00.22.22-20260702164346.bin.sig.
IMAGE_NAME = re.compile(r"((ams|n3f|n3s)_rev(\d+))-firmware-v(\d\d\.\d\d\.\d\d\.\d\d)-")


class Abort(Exception):
    """
    A check refused the update; the message says why, for the console and the menu.
    """


def mc_id_for_index(idx: int) -> int:
    """
    The bus address the bridge enrolls chain index `idx` at.
    """
    i = int(idx)
    if i < 0:
        return 0
    return i if i < 4 else (0x80 + (i - 4)) & 0xFF


def device_of(unit_id: int) -> int:
    """
    The bus device a unit id is addressed through: ids from 0x80 up are AMS HTs.
    """
    return HT if unit_id & 0x80 else BOXED


# Bambu's images
def board_of_code(code: str) -> str:
    """
    The image board a unit's model code needs: N3F05 -> n3f_rev5, AMS08 -> ams_rev8.

    :param code: the model code from the unit's version answer
    :return str: the board, or '' for a code that is none of the three models
    """
    m = re.fullmatch(r"(AMS|N3F|N3S)(\d\d)", code or "")
    return f"{m.group(1).lower()}_rev{int(m.group(2))}" if m else ""


def model_of_board(board: str) -> str:
    """
    The model key (ams1, ams2 or ht) a board belongs to, or ''.
    """
    fam = FAMILIES.get((board or "").split("_", 1)[0])
    return fam[0] if fam else ""


def describe(board: str) -> str:
    """
    A board for people: 'AMS 2 Pro (n3f_rev5)'.
    """
    fam = FAMILIES.get((board or "").split("_", 1)[0])
    return f"{fam[1]} ({board})" if fam else (board or "an unknown board")


def parse_image(data: bytes) -> dict:
    """
    Check that `data` is a whole Bambu AMS image (BIMH container) and read what it is for.

    :param data: the .bin.sig file
    :return dict: name, board, model, version and hlen (the container header's length)
    :raises ValueError: when it is not a whole AMS image
    """
    if len(data) < 0x80 or data[:4] != b"BIMH":
        raise ValueError("not a Bambu AMS image (no BIMH header)")

    def u32(off: int) -> int:
        return int.from_bytes(data[off:off + 4], "little")
    if u32(4) != 1:
        raise ValueError(f"unknown image format {u32(4)}")
    if u32(8) != len(data):
        raise ValueError(f"the file is {len(data)} bytes but its header says {u32(8)}; "
                         f"it is cut short or damaged")
    hlen = u32(0x20)
    if not 0x80 <= hlen < len(data) or u32(0x28) != len(data) - hlen:
        raise ValueError("the image's header lengths do not add up")
    name = data[0x30:0x80].split(b"\0", 1)[0].decode("ascii", "replace")
    m = IMAGE_NAME.match(name)
    if not m:
        raise ValueError(f"not an AMS image ({name or 'unnamed'})")
    return {"name": name, "board": m.group(1), "model": FAMILIES[m.group(2)][0],
            "version": m.group(4), "hlen": hlen}


def _body(dev: int, unit_id: int, cmd: int, f16: int, f20: int, f24: int,
          payload: bytes, trailer: bool) -> bytes:
    """
    One update frame body: a 32-byte prefix, the payload, and on a data block a 4-byte trailer.
    """
    return (dev.to_bytes(2, "little") + HOST.to_bytes(2, "little") + b"\x06\x01"
            + bytes((cmd, dev >> 8)) + bytes(4) + unit_id.to_bytes(4, "little")
            + f16.to_bytes(4, "little") + f20.to_bytes(4, "little")
            + f24.to_bytes(4, "little") + len(payload).to_bytes(4, "little")
            + payload + (bytes(4) if trailer else b""))


def pack(image: bytes, unit_id: int) -> dict:
    """
    The update frames for one unit, built from a Bambu image: a header frame carrying the
    image's own container header (the loader erases on it), then the rest in 1 KiB data blocks.

    A printer leaves each data block's trailer zero but on the last block, and the loader takes a
    zero there too, so every trailer here is zero.

    :param image: the .bin.sig file, checked by parse_image
    :param unit_id: the unit's bus id (0-3 boxed, 0x80 up an HT)
    :return dict: {"header": body, "blocks": [body, ...]}
    """
    dev = device_of(unit_id)
    hlen = int.from_bytes(image[0x20:0x24], "little")
    rest = image[hlen:]
    return {"header": _body(dev, unit_id, 0x02, len(image), 0, 0, image[:hlen], False),
            "blocks": [_body(dev, unit_id, 0x03, REGION, seq, BLOCK,
                             rest[seq * BLOCK:(seq + 1) * BLOCK], True)
                       for seq in range((len(rest) + BLOCK - 1) // BLOCK)]}


def load_catalog(here: str) -> list:
    """
    The images ams_firmware.json beside this script lists (none when it is missing).
    """
    try:
        with open(os.path.join(here, CATALOG_NAME)) as f:
            images = json.load(f).get("images") or []
    except (OSError, ValueError, AttributeError):
        return []
    return [e for e in images
            if isinstance(e, dict) and e.get("board") and e.get("version")
            and e.get("file") and e.get("url")]


def md5_of(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


class RemoteFile(io.RawIOBase):
    """
    A seekable, read-only view of a file on a web server, read in blocks with HTTP range
    requests, so zipfile can take one image out of a large Bambu package without downloading
    the whole package.
    """

    BLOCK = 256 * 1024

    def __init__(self, url: str, timeout: float = 30.0, opener=None) -> None:
        super().__init__()
        self.url, self.timeout = url, timeout
        self._open = opener or urllib.request.urlopen
        self._blocks = {}
        self._pos = 0
        self.size = self._range(0, 0)[1]

    def _range(self, start: int, end: int) -> tuple:
        """
        Bytes start..end of the file, and the file's whole size.
        """
        req = urllib.request.Request(self.url, headers={"Range": f"bytes={start}-{end}",
                                                         "User-Agent": USER_AGENT})
        with self._open(req, timeout=self.timeout) as r:
            m = re.match(r"bytes (\d+)-(\d+)/(\d+)", r.headers.get("Content-Range") or "")
            if getattr(r, "status", None) != 206 or not m or int(m.group(1)) != start:
                raise IOError("the server will not send part of the package; download the "
                              "image (.bin.sig) yourself and put it beside ams_flash.py")
            data = r.read()
        if int(m.group(2)) != end or len(data) != end - start + 1:
            raise IOError("part of the package arrived cut short")
        return data, int(m.group(3))

    def _block(self, i: int) -> bytes:
        if i not in self._blocks:
            start = i * self.BLOCK
            self._blocks[i] = self._range(start, min(start + self.BLOCK, self.size) - 1)[0]
        return self._blocks[i]

    def readable(self) -> bool:
        return True

    def seekable(self) -> bool:
        return True

    def tell(self) -> int:
        return self._pos

    def seek(self, offset: int, whence: int = io.SEEK_SET) -> int:
        base = {io.SEEK_SET: 0, io.SEEK_CUR: self._pos, io.SEEK_END: self.size}[whence]
        self._pos = max(0, base + offset)
        return self._pos

    def readinto(self, b) -> int:
        view = memoryview(b).cast("B")
        n = 0
        while n < len(view) and self._pos < self.size:
            i, off = divmod(self._pos, self.BLOCK)
            chunk = self._block(i)[off:off + len(view) - n]
            view[n:n + len(chunk)] = chunk
            n += len(chunk)
            self._pos += len(chunk)
        return n


def download_image(entry: dict, here: str, opener=None) -> str:
    """
    Fetch a listed image from Bambu into the folder beside this script, unless an intact copy is
    already there.

    :param entry: its ams_firmware.json entry: url (a .bin.sig, or a package .zip holding
                  `file`), file, md5
    :param here: the folder beside this script
    :return str: the image's path
    """
    path = os.path.join(here, os.path.basename(entry["file"]))
    want = (entry.get("md5") or "").lower()
    if os.path.exists(path) and (not want or md5_of(path) == want):
        return path
    url = entry["url"]
    if urllib.parse.urlparse(url).path.lower().endswith(".zip"):
        with zipfile.ZipFile(RemoteFile(url, opener=opener)) as z:
            data = z.read(entry["file"])
    else:
        req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        with (opener or urllib.request.urlopen)(req, timeout=60) as r:
            data = r.read()
    got = hashlib.md5(data).hexdigest()
    if want and got != want:
        raise IOError(f"{os.path.basename(path)} arrived damaged (MD5 {got}, "
                      f"expected {want})")
    tmp = path + ".part"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)
    return path


def image_from_file(path: str, board: str = "", version: str = "") -> tuple:
    """
    The image in a local file: a .bin.sig, or the one image for `board` (and `version`) in a
    Bambu offline update package (.zip).

    :return tuple: (the image, a label for it)
    """
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as z:
            names = [n for n in z.namelist() if IMAGE_NAME.match(os.path.basename(n))]
            pick = []
            for n in names:
                m = IMAGE_NAME.match(os.path.basename(n))
                if (not board or m.group(1) == board) and (not version or m.group(4) == version):
                    pick.append(n)
            if len(pick) != 1:
                want = " ".join(x for x in (board, version) if x) or "this unit"
                held = ", ".join(os.path.basename(n) for n in names) or "no AMS images"
                raise ValueError(f"it holds {len(pick)} images for {want} ({held})")
            return z.read(pick[0]), f"{os.path.basename(path)}: {os.path.basename(pick[0])}"
    with open(path, "rb") as f:
        return f.read(), os.path.basename(path)


# CRCs
def crc8(data: bytes) -> int:
    c = CRC8_INIT
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ CRC8_POLY) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c


def crc16(data: bytes) -> int:
    c = CRC16_INIT
    for b in data:
        c = (c ^ (b << 8)) & 0xFFFF
        for _ in range(8):
            c = ((c << 1) ^ CRC16_POLY) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
    return c


def build_cmd1(target: int, ams_id: int, counter: int = 0x0077) -> bytes:
    """
    The enter-loader / loader-handshake frame: op 0601, cmd 01, source 0x0900.
    """
    f = bytearray(49)
    f[0] = 0x3D; f[1] = 0x04
    f[2] = counter & 0xFF; f[3] = (counter >> 8) & 0xFF
    f[4] = 0x31; f[5] = 0x00
    f[6] = crc8(bytes(f[:6]))
    f[7] = target & 0xFF; f[8] = (target >> 8) & 0xFF
    f[9] = 0x00; f[10] = 0x09
    f[11] = 0x06; f[12] = 0x01; f[13] = 0x01
    f[14] = (target >> 8) & 0xFF
    f[19] = ams_id & 0xFF
    c = crc16(bytes(f[:47]))
    f[47] = c & 0xFF; f[48] = (c >> 8) & 0xFF
    assert f[11] == 0x06 and f[12] == 0x01 and f[13] == 0x01, "not a cmd1 frame"
    return bytes(f)


def wrap(cls: int, counter: int, body: bytes) -> bytes:
    """
    Rebuild a full frame from its invariant body with a fresh counter+CRCs.
    """
    fr = bytearray(7 + len(body) + 2)
    fr[0] = 0x3D
    fr[1] = cls
    fr[2] = counter & 0xFF; fr[3] = (counter >> 8) & 0xFF
    n = len(fr)                                     # [4:6] = total frame length
    fr[4] = n & 0xFF; fr[5] = (n >> 8) & 0xFF
    fr[6] = crc8(bytes(fr[:6]))
    fr[7:7 + len(body)] = body
    c = crc16(bytes(fr[:-2]))
    fr[-2] = c & 0xFF; fr[-1] = (c >> 8) & 0xFF
    return bytes(fr)


def loader_target_of(art):
    """
    (device address, AMS id) the packed image's frames are addressed to.
    """
    hdr = art["header"]
    return hdr[0] | (hdr[1] << 8), hdr[19 - 7]


def plan(art, counter0: int = 0):
    """
    The packed image's frames in send order: the header, which erases, then each data block.
    """
    c = counter0
    yield ("header", -1, wrap(0x00, c, art["header"])); c += 1
    for s, body in enumerate(art["blocks"]):
        yield ("data", s, wrap(0x00, c, body)); c += 1


# Bridge link (tcp:// or USB serial) + link-key handshake
class Link:
    """
    A line link to the bridge over TCP or its USB serial port.
    """

    WRITE_TIMEOUT = 15.0

    def __init__(self, target: str, timeout: float = 1.0):
        self.target = target
        self.tcp = target.startswith("tcp://")
        self.timeout = timeout
        self._open()

    def _open(self):
        self.buf = b""
        self.dead = False
        if self.tcp:
            host, _, port = self.target[6:].partition(":")
            self.sock = socket.create_connection((host, int(port or 8888)),
                                                 timeout=5)
            self.sock.settimeout(self.timeout)
        else:
            import termios
            import tty
            self.fd = os.open(self.target, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                tty.setraw(self.fd)
                attrs = termios.tcgetattr(self.fd)
                attrs[2] |= termios.CLOCAL | termios.CREAD
                attrs[4] = attrs[5] = termios.B115200  # ignored by USB-CDC
                termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            except termios.error as e:   # not an OSError
                os.close(self.fd)
                raise IOError(f"cannot set up {self.target} ({e})")
            except Exception:
                os.close(self.fd)
                raise
            try:
                import fcntl
                fcntl.ioctl(self.fd, termios.TIOCEXCL)   # nobody else opens it
            except Exception:
                pass

    def reopen(self, wait_s: float = 20.0):
        """
        Close and open the link again, waiting for the port to come back (a USB bridge that reset
        re-enumerates under the same by-id path).
        """
        self.close()
        deadline = time.time() + wait_s
        while True:
            try:
                self._open()
                return
            except OSError as e:
                if time.time() > deadline:
                    raise IOError(f"could not reopen the bridge link ({e})")
                time.sleep(1.0)

    def send(self, obj: dict):
        b = (json.dumps(obj) + "\n").encode()
        if self.tcp:
            try:
                self.sock.sendall(b)
            except socket.timeout:
                raise IOError("socket write timed out (bridge not draining)")
            except OSError as e:
                self.dead = True
                raise IOError(f"bridge link closed ({e})")
            return
        import select
        deadline = time.time() + self.WRITE_TIMEOUT
        while b:
            try:
                n = os.write(self.fd, b)
                b = b[n:]
                continue
            except BlockingIOError:
                pass
            except OSError as e:
                self.dead = True
                raise IOError(f"bridge link closed ({e})")
            left = deadline - time.time()
            if left <= 0:
                raise IOError("serial write timed out (bridge not draining)")
            select.select([], [self.fd], [], min(left, 0.5))

    def _read(self) -> bytes:
        """
        Whatever is buffered now, waiting at most `timeout` for some.
        """
        if self.tcp:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                raise
            except OSError as e:
                self.dead = True
                raise IOError(f"bridge link closed ({e})")
            if not chunk:
                self.dead = True
                raise IOError("bridge link closed")
            return chunk
        import select
        r, _w, _x = select.select([self.fd], [], [], self.timeout)
        if not r:
            return b""
        try:
            chunk = os.read(self.fd, 4096)
        except BlockingIOError:
            return b""
        except OSError as e:
            self.dead = True
            raise IOError(f"bridge link closed ({e})")
        if not chunk:
            self.dead = True
            raise IOError("bridge link closed")
        return chunk

    def lines(self):
        while True:
            try:
                chunk = self._read()
            except socket.timeout:
                yield None; continue
            if not chunk:
                yield None; continue
            self.buf += chunk
            if b"\n" not in self.buf:
                yield None; continue
            while b"\n" in self.buf:
                line, _, self.buf = self.buf.partition(b"\n")
                yield line.decode("utf-8", "replace").rstrip("\r")

    def close(self):
        try:
            if self.tcp:
                self.sock.close()
            else:
                os.close(self.fd)
        except Exception:
            pass


def open_link(port: str, tries: int = 5) -> Link:
    """
    Open the bridge link Klipper just let go of; the port can take a moment to come free.
    """
    for i in range(tries):
        try:
            return Link(port)
        except OSError:
            if i == tries - 1:
                raise
            time.sleep(1.0)
    raise IOError(f"could not open {port}")


def authenticate(link: Link, key: str, timeout: float = 8.0) -> bool:
    """
    board -> {"nonce"}; host -> HMAC-SHA512(key,nonce)[:32]; board -> {"ok":1}.
    """
    sock = getattr(link, "sock", None)
    if sock is None:
        return not key
    deadline = time.time() + max(timeout, 2.0)
    sock.settimeout(0.2)
    buf = b""
    while time.time() < deadline and b"\n" not in buf:
        try:
            c = sock.recv(256)
        except (socket.timeout, TimeoutError):
            if not key:
                return True
            continue
        if not c:
            return False
        buf += c
    if b"\n" not in buf:
        return not key
    line = buf.split(b"\n", 1)[0].decode("utf-8", "replace")
    if '"nonce"' not in line:
        link.buf = buf
        return True
    if not key:
        return False
    nonce = bytes.fromhex(json.loads(line)["nonce"])
    mac = hmac.new(key.encode(), nonce, hashlib.sha512).digest()[:32]
    sock.sendall(f'{{"cmd":"auth","mac":"{mac.hex()}"}}\n'.encode())
    reply = buf.split(b"\n", 1)[1]
    while time.time() < deadline and b"\n" not in reply:
        try:
            c = sock.recv(256)
        except (socket.timeout, TimeoutError):
            continue
        if not c:
            return False
        reply += c
    rl = reply.split(b"\n", 1)[0].decode("utf-8", "replace")
    link.buf = reply.split(b"\n", 1)[1] if b"\n" in reply else b""
    sock.settimeout(1.0)
    return '"ok":1' in rl.replace(" ", "")


def _await(link: Link, evt: str, timeout: float = 8.0):
    t0 = time.time()
    for line in link.lines():
        if line and f'"{evt}"' in line:
            return line
        if time.time() - t0 > timeout:
            return None


def _await_evt(link: Link, evt: str, timeout: float = 8.0):
    """
    The next line whose event is exactly `evt` (not merely mentioning it: the chain report has a
    "raw" field, the status report a "state" one).
    """
    tag = f'"evt":"{evt}"'
    t0 = time.time()
    for line in link.lines():
        if line and tag in line.replace(" ", ""):
            return line
        if time.time() - t0 > timeout:
            return None


def _rx_of(line: str) -> bytes:
    """
    The captured bytes in a raw/txsend reply; b"" (no answer) if there are none or they arrived
    damaged (an odd-length hex string after a lost byte).
    """
    m = re.search(r'"rx"\s*:\s*"([0-9A-Fa-f]*)"', line or "")
    try:
        return bytes.fromhex(m.group(1)) if m else b""
    except ValueError:
        return b""


def _seqretry(rx: bytes):
    """
    Parse the loader's '[MCU_UP] N seq_num: X, retry: M' out of a reply.
    """
    a = "".join(chr(b) if 32 <= b < 127 else " " for b in rx)
    m = re.findall(r"seq_num:\s*([0-9a-fA-F]+),\s*retry:\s*(\d+)", a)
    if not m:
        return None
    want, rt = m[-1]
    try:
        return int(want, 16), int(rt)
    except ValueError:
        return None


def send_frame(link: Link, frame: bytes, window_us: int = 8000,
               stage_tries: int = 4) -> bytes:
    """
    Stage one frame across txbuf lines, then txsend; return the AMS reply.
    """
    for attempt in range(stage_tries):
        for off in range(0, len(frame), CHUNK):
            link.send({"cmd": "txbuf", "off": off,
                       "hex": frame[off:off + CHUNK].hex()})
            if not _await(link, "txbuf", 4.0):
                link.dead = True        # silent, not closed: reopen it
                raise IOError(f"txbuf not acked at off {off}/{len(frame)}")
        link.send({"cmd": "txsend", "n": len(frame), "us": window_us})
        line = _await(link, "txsend", timeout=max(2.0, window_us / 1e6 + 2))
        if line is None:
            link.dead = True
            raise IOError("txsend not acked")
        if '"crc":"bad"' in line.replace(" ", ""):
            print(f"    staged frame arrived damaged -- staging it again "
                  f"({attempt + 1}/{stage_tries})", file=sys.stderr)
            continue
        return _rx_of(line)
    raise IOError("staged frame still damaged after re-staging -- the link "
                  "is dropping bytes")


SIGS = {"[MCU_UP]": "5b4d43555f55505d",
        "Loader Version": "4c6f61646572205665727369",
        "wait cmd1": "7761697420636d6431"}


def frames(raw: bytes):
    """
    Every whole, CRC-valid long frame in a bus capture, in order.
    """
    out = []
    i, n = 0, len(raw)
    while i + 7 <= n:
        if raw[i] == 0x3D and crc8(raw[i:i + 6]) == raw[i + 6]:
            end = i + (raw[i + 4] | (raw[i + 5] << 8))
            if i + 9 <= end <= n and \
                    crc16(raw[i:end - 2]) == (raw[end - 2] | (raw[end - 1] << 8)):
                out.append(raw[i:end])
                i = end
                continue
        i += 1
    return out


def loader_hits(hexstr: str):
    """
    The loader's own answer in a capture window, if there is one.
    """
    h = hexstr.lower()
    out = []
    for name, pat in SIGS.items():
        i = h.find(pat)
        while i >= 0 and i % 2:
            i = h.find(pat, i + 1)
        if i >= 0:
            out.append(name)
    try:
        raw = bytes.fromhex(h[:len(h) & ~1])
    except ValueError:
        raw = b""
    for f in frames(raw):
        if len(f) >= 13 and f[11:13] == b"\x06\x01" \
                and (f[9] | (f[10] << 8)) != 0x0900:
            out.append("ams-origin/op0601")
            break
    return out


# What a unit says it runs
def build_version_query(unit_id: int, counter: int = 0) -> bytes:
    """
    The printer's version query (op 0301) to one unit, as a printer sends it.
    """
    f = bytearray((0x3D, 0x05, counter & 0xFF, (counter >> 8) & 0xFF, 16, 0, 0))
    f[6] = crc8(bytes(f[:6]))
    f += device_of(unit_id).to_bytes(2, "little") + HOST.to_bytes(2, "little")
    f += bytes((0x03, 0x01, unit_id & 0xFF))
    c = crc16(bytes(f))
    return bytes(f) + bytes((c & 0xFF, c >> 8))


def version_answer(raw: bytes, unit_id: int):
    """
    The unit's own answer to the version query in a capture.

    It carries the version lowest byte first (1d 15 00 03 is 03.00.21.29), then the model code
    (N3F05, N3S05, AMS07 or AMS08), and at [33] the id of the unit answering.

    :return tuple: (version, model code), or None when the unit did not answer
    """
    dev = device_of(unit_id)
    for f in frames(raw):
        if (len(f) >= 36 and f[11:13] == b"\x03\x01"
                and int.from_bytes(f[9:11], "little") == dev and f[33] == unit_id):
            code = f[17:33].split(b"\0", 1)[0].decode("ascii", "replace")
            return "%02d.%02d.%02d.%02d" % tuple(reversed(f[13:17])), code
    return None


def read_version(link: Link, unit_id: int, tries: int = 4):
    """
    Ask one unit for its firmware version and model code.

    :return tuple: (version, model code), or None when it never answers
    """
    for i in range(tries):
        link.send({"cmd": "raw", "hex": build_version_query(unit_id, i).hex(), "us": 100000})
        line = _await_evt(link, "raw", 3.0)
        if line is None:
            raise IOError("bridge did not answer raw")
        got = version_answer(_rx_of(line), unit_id)
        if got:
            return got
    return None


def wire_online(link: Link, tries: int = 3):
    return wire_status(link, tries)[0]


def settle(link: Link, seconds: float):
    """
    Read (and drop) whatever the bridge says for `seconds`.
    """
    t0 = time.time()
    for _line in link.lines():
        if time.time() - t0 >= seconds:
            return


def reset_bridge_mode(link: Link):
    """
    Put the bridge back to polling the bus, whatever an earlier run left.
    """
    for cmd, evt in (({"cmd": "fwreplay", "on": 0}, "fwreplay"),
                     ({"cmd": "sniff", "on": 0}, "sniff_mode")):
        link.send(cmd)
        if _await_evt(link, evt, 4.0) is None:
            raise IOError(f"bridge did not answer {cmd['cmd']} off")
    settle(link, ONLINE_SETTLE_S)


def release_bridge(link: Link):
    """
    Best effort on the way out: never leave the bridge in fwreplay or sniff for Klipper, which
    cannot drive the units in either.
    """
    for cmd, evt in (({"cmd": "fwreplay", "on": 0}, "fwreplay"),
                     ({"cmd": "sniff", "on": 0}, "sniff_mode")):
        try:
            link.send(cmd)
            _await_evt(link, evt, 3.0)
        except OSError:
            return


def wire_status(link: Link, tries: int = 3):
    """
    (online, drying): chain indices the bridge itself sees answering now, and those running a dry
    cycle, from its status report.
    """
    for _ in range(tries):
        link.send({"cmd": "status"})
        line = _await_evt(link, "status", 5.0)
        if line is None:
            continue
        try:
            units = json.loads(line).get("units") or []
            online = sorted(int(u["n"]) for u in units if u.get("online"))
            drying = sorted(int(u["n"]) for u in units
                            if (u.get("dryrem") or 0) > 0)
            return online, drying
        except (ValueError, KeyError, TypeError, AttributeError):
            continue        # a damaged line: never guess from part of it
    raise IOError("could not read the unit list from the bridge")


def loader_on_bus(link: Link, watch_s: float = 6.0, exclude=None) -> bool:
    """
    Whether some unit on the bus is sitting in its bootloader.
    """
    link.send({"cmd": "sniff", "on": 1})
    heard = False
    t0 = time.time()
    try:
        for line in link.lines():
            if line and '"evt":"sniff"' in line.replace(" ", ""):
                m = re.search(r'"hex"\s*:\s*"([0-9A-Fa-f]*)"', line)
                if m and loader_hits(m.group(1)):
                    raw = bytes.fromhex(m.group(1)[:len(m.group(1)) & ~1])
                    if not (exclude and loader_answer(raw, *exclude)):
                        heard = True
                        break
            if time.time() - t0 > watch_s:
                break
    finally:
        link.send({"cmd": "sniff", "on": 0})
        if _await_evt(link, "sniff_mode", 3.0) is None:
            raise IOError("bridge did not answer sniff off")
    return heard


def loader_answer(raw: bytes, dev: int, ams_id: int) -> bool:
    """
    Whether a capture holds the ADDRESSED unit's loader answering cmd1.
    """
    for f in frames(raw):
        if len(f) > 19 and f[11:13] == b"\x06\x01" \
                and (f[9] | (f[10] << 8)) == dev and f[19] == ams_id:
            return True
    text = "".join(chr(b) if 32 <= b < 127 else " " for b in raw)
    n = str(ams_id)
    return re.search(rf"(?<!\d)(?:ams {n} wait cmd1|{n} resev cmd 0x1|"
                     rf"{n} send cmd 0x1 back)", text) is not None


def enter_loader(link: Link, art, tries: int = 6) -> bool:
    """
    cmd1 until the loader announces. cmd1 only -- can never erase.
    """
    dev, ams_id = loader_target_of(art)
    cmd1 = build_cmd1(dev, ams_id)
    for i in range(tries):
        rx = send_frame(link, cmd1, window_us=600000)
        if rx and loader_answer(rx, dev, ams_id):
            print(f"    loader confirmed after cmd1 #{i + 1}")
            return True
    return False


def probe_cmd1(link: Link, art, tries: int = 4) -> bool:
    """
    Non-destructive loader probe, OUTSIDE fwreplay: send cmd1 via `raw`.
    """
    dev, ams_id = loader_target_of(art)
    cmd1 = build_cmd1(dev, ams_id)
    for i in range(tries):
        link.send({"cmd": "raw", "hex": cmd1.hex(), "us": 600000})
        line = _await_evt(link, "raw", timeout=3.0)
        if line is None:
            raise IOError("bridge did not answer raw")
        rx = _rx_of(line)
        if rx and loader_answer(rx, dev, ams_id):
            print(f"    loader confirmed after cmd1 #{i + 1}")
            return True
    return False


def _one_pass(link: Link, art, steps) -> str:
    """
    One erase+rewrite pass, LOADER-DRIVEN: send the block the loader's reply asks for (seq_num),
    resending in place any chunk it will not accept, so a glitched chunk is fixed within the pass
    instead of failing the whole image.
    """
    link.send({"cmd": "fwreplay", "on": 1, "confirm": CONFIRM})
    if not _await(link, "fwreplay", 4.0):
        print("    bridge did not answer fwreplay; reopening the link",
              file=sys.stderr)
        link.dead = True
        return "stall"
    try:
        # cmd1 handshake FIRST.
        if not enter_loader(link, art):
            print("    no loader response to cmd1 -- NOTHING erased", file=sys.stderr)
            return "stall"
        blocks = art["blocks"]
        nblk = len(blocks)
        header = next(f for k, s, f in steps if k == "header")
        bframe = {s: f for k, s, f in steps if k == "data"}

        rx = send_frame(link, header, window_us=3_500_000)
        if not rx:
            print("    header NO ACK -- loader idle; unit left for a re-erase",
                  file=sys.stderr)
            return "stall"
        print("    header sent, erase triggered; streaming blocks...")

        # LOADER-DRIVEN transfer.
        want = 0
        resends = {}
        total = 0
        last_rx = rx
        MAX_PER = 8
        MAX_TOTAL = nblk * 6
        while want < nblk:
            frame = bframe.get(want)
            if frame is None:
                print(f"    loader wants seq {want} (outside 0..{nblk-1}) -- "
                      f"resending the whole pass", file=sys.stderr)
                return "badimage"
            win = 3_000_000 if want == nblk - 1 else 400_000
            rx = send_frame(link, frame, window_us=win)
            if not rx:
                print(f"    data {want}: NO ACK -- stopping (unit left in loader)",
                      file=sys.stderr)
                return "stall"
            last_rx = rx
            total += 1
            if total > MAX_TOTAL:
                print("    too many block sends this pass -- resending the whole "
                      "pass", file=sys.stderr)
                return "badimage"
            rxl = rx.hex().lower()
            if SUCCESS_HEX in rxl:
                print("    loader: success! -- image verified, resetting into the app")
                return "success"
            if CHUNKERR_HEX in rxl:
                print("    loader: chunk hash error -- resending the whole pass",
                      file=sys.stderr)
                return "badimage"
            sr = _seqretry(rx)
            if sr is None:
                want += 1                  # no seq narration: treat as accepted
                continue
            nxt, rt = sr
            if nxt > want:
                want = nxt                 # accepted -- follow the loader forward
            else:
                resends[nxt] = resends.get(nxt, 0) + 1
                print(f"    loader wants seq {nxt} resent "
                      f"(#{resends[nxt]}, retry {rt})", file=sys.stderr)
                if resends[nxt] > MAX_PER:
                    print(f"    seq {nxt} will not take after {MAX_PER} resends -- "
                          f"resending the whole pass", file=sys.stderr)
                    return "badimage"
                want = nxt
            if want and want % 24 == 0:
                print(f"    at block {want}/{nblk - 1}")

        # All blocks accepted.
        rxhex = last_rx.hex().lower()
        if SUCCESS_HEX in rxhex:
            print("    loader: success! -- image verified, resetting into the app")
            return "success"
        if CHUNKERR_HEX in rxhex:
            print("    loader: chunk hash error -- resending the whole pass",
                  file=sys.stderr)
            return "badimage"
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in last_rx)
        if IMGERR_HEX in rxhex:
            print(f"    loader: IMG HASH ERROR after all blocks were accepted -- "
                  f"{ascii_[-120:]}", file=sys.stderr)
            return "badimg"
        print(f"    loader: no success! after all blocks ({len(last_rx)} B) -- "
              f"{ascii_[-120:]}", file=sys.stderr)
        return "badimage"
    finally:
        try:
            link.send({"cmd": "fwreplay", "on": 0})
            _await(link, "fwreplay", 4.0)
        except OSError:
            pass        # the link is gone.


def do_flash(link: Link, art, retries: int = FLASH_RETRIES,
             reconnect=None) -> int:
    """
    Erase + rewrite the AMS, retrying the WHOLE flash until the loader itself reports success!.
    """
    steps = list(plan(art))
    print(f"    up to {retries + 1} attempt(s); only a loader 'success!' ends it.")
    for attempt in range(1, retries + 2):
        print(f"    == flash attempt {attempt}/{retries + 1} ==")
        if getattr(link, "dead", False) and reconnect is not None:
            try:
                reconnect()
                print("    link reopened")
            except OSError as e:
                print(f"    the bridge link is gone and did not come back "
                      f"({e})", file=sys.stderr)
                break
        try:
            verdict = _one_pass(link, art, steps)
        except OSError as e:
            # A staging/link stall (e.g.
            print(f"    attempt {attempt}: link stall -- {e}", file=sys.stderr)
            verdict = "stall"
        if verdict == "success":
            print("    FLASH CONFIRMED: the loader verified the image and reset "
                  "into the new firmware.")
            return 0
        if verdict == "badimg":
            print("    the loader accepted every chunk and rejected the assembled "
                  "image: this image is wrong for this unit (model or build), "
                  "not the link. Stopping rather than erasing again. The unit is "
                  "left in its loader (recoverable): do not power it off; run "
                  "the update again with the right image for this model.",
                  file=sys.stderr)
            return BADIMG
        if attempt <= retries:
            print(f"    attempt {attempt}: {verdict} -- resending the full flash",
                  file=sys.stderr)
    print("    out of attempts: the loader never verified success!. The unit is "
          "left in its loader (recoverable) -- do not power it off; run the "
          "same command with MODE=go again. If every pass acks but reports "
          "chunk hash error, the bus/link is dropping bytes.",
          file=sys.stderr)
    return 1


# Moonraker (localhost): the print state, the units, and g-code to Klipper
class Moon:
    def __init__(self, base=MOON):
        self.base = base

    def _req(self, method, path, timeout=30):
        r = urllib.request.Request(self.base + path, method=method)
        return json.loads(urllib.request.urlopen(r, timeout=timeout).read())

    def q(self, obj):
        p = "/printer/objects/query?" + urllib.parse.quote(obj)
        return self._req("GET", p)["result"]["status"].get(obj, {})

    def objects(self):
        return self._req("GET", "/printer/objects/list")["result"]["objects"]

    def settings(self):
        """
        Klipper's parsed config, section names lower-cased.
        """
        p = "/printer/objects/query?configfile=settings"
        return (self._req("GET", p)["result"]["status"]
                .get("configfile", {}).get("settings", {}))

    def gcode(self, script):
        """
        Run a g-code line in Klipper; False if it could not be sent.
        """
        try:
            self._req("POST", "/printer/gcode/script?"
                      + urllib.parse.urlencode({"script": script}), timeout=15)
            return True
        except Exception:
            return False

    def state(self):
        try:
            return self.q("webhooks").get("state", "?")
        except Exception:
            return "?"


def hold_bridge(moon, unit: str, seconds: int) -> bool:
    """
    Have Klipper let go of the unit's bridge link for this run (it keeps running and takes the
    link back after `seconds`), or take it back now with 0.

    :return bool: False when Klipper could not be told
    """
    return moon.gcode(f"AFC_BAMBU_FIRMWARE UNIT={unit} HOLD={seconds}")


def relaunch_detached() -> None:
    """
    Re-run this script in its own session, so the update outlives the shell command that started
    it (Klipper ends that after its timeout). A Klipper service restart still stops it; the unit
    then waits in its loader and running the update again finishes it.
    """
    import subprocess
    args = [x for x in sys.argv[1:] if x != "--detached"] + ["--detached"]
    subprocess.Popen([sys.executable or "python3", os.path.abspath(__file__)] + args,
                     start_new_session=True, close_fds=True, stdin=subprocess.DEVNULL,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# Config auto-detection (self-locating from this file's directory)
def config_root(start: str) -> str:
    """
    Walk up from `start` to the dir holding printer.cfg (the config root).
    """
    d = os.path.abspath(start)
    for _ in range(6):
        if os.path.exists(os.path.join(d, "printer.cfg")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return start


def find_in_cfg(cfg_dir: str, key: str):
    for root, _dirs, files in os.walk(cfg_dir):
        for fn in files:
            if not fn.endswith(".cfg"):
                continue
            try:
                for ln in open(os.path.join(root, fn), errors="replace"):
                    s = ln.strip()
                    if s.startswith(key + ":"):
                        v = s.split(":", 1)[1]
                        # Drop a Klipper inline comment (# or ;) and take the first token.
                        v = v.split("#", 1)[0].split(";", 1)[0].strip()
                        v = v.split()[0] if v.split() else ""
                        if v:
                            return v
            except Exception:
                pass
    return None


def _clean_key(v):
    """
    A tcp_key as Klipper publishes it, or None for no key.
    """
    s = str(v).strip() if v is not None else ""
    return None if s.lower() in ("", "none") else s


def bridge_keys(settings):
    """
    {serial_port: tcp_key or None} for every Bambu bridge in the live config, read from
    [AFC_BridgeBox ...] and [AFC_BambuAMS ...] sections. serial_port: auto names no port, so
    it is left out.
    """
    found, master = {}, {}
    for sec, opts in (settings or {}).items():
        if not sec.startswith(("afc_bridgebox", "afc_bambuams")):
            continue
        opts = opts or {}
        port = str(opts.get("serial_port") or "").strip()
        if not port.startswith(("tcp://", "/")):
            continue
        key = _clean_key(opts.get("tcp_key"))
        if sec.startswith("afc_bridgebox") and key:
            master[port] = key
        if key and not found.get(port):
            found[port] = key
        else:
            found.setdefault(port, None)
    found.update(master)
    return found


def bridge_from_settings(settings):
    """
    (serial_port, tcp_key) of the one Bambu bridge in the live config.
    """
    found = bridge_keys(settings)
    if len(found) > 1:
        raise ValueError("more than one bridge in the config ("
                         + ", ".join(sorted(found)) + "); name the one to "
                         "use with --target")
    if not found:
        return None, None
    return next(iter(found.items()))


def unit_rows(moon):
    """
    Every AFC_BambuAMS unit Klipper has: name, online, model, chain index, bridge port, bridge fw,
    firmware version and model code once read, and whether it is drying.
    """
    names = [o.split(" ", 1)[1] for o in moon.objects()
             if o.startswith("AFC_BambuAMS ") and " " in o]
    rows = []
    for n in names:
        st = moon.q(f"AFC_BambuAMS {n}")
        if not isinstance(st, dict) or not st:
            raise IOError(f"no status for AFC_BambuAMS {n}")
        idx = st.get("ams_index")
        rows.append({"name": n,
                     "online": bool(st.get("bridge_online")),
                     "model": str(st.get("ams_model") or "").strip().lower(),
                     "index": idx if isinstance(idx, int) else None,
                     "port": st.get("bridge_port") or None,
                     "fw": st.get("bridge_fw"),
                     "version": st.get("fw_version") or "",
                     "code": st.get("fw_model") or "",
                     "drying": bool(st.get("drying"))})
    return rows


def read_state(path):
    """
    The record of a unit a previous run left in its loader, or None.
    """
    try:
        with open(path) as f:
            d = json.load(f)
        return d if isinstance(d, dict) and d.get("unit") else None
    except (OSError, ValueError):
        return None


def write_state(path, **d) -> bool:
    """
    Record the unit about to be sent into its loader; False if it could not be written, in which
    case it must not be sent there.
    """
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(d, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
        return True
    except OSError as e:
        print(f"    cannot record the unit in {path} ({e})", file=sys.stderr)
        return False


def clear_state(path):
    try:
        os.remove(path)
    except OSError:
        pass


def write_result(here: str, **d) -> None:
    """
    Leave how this run ended beside the script, for the Flash AMS menu.
    """
    d.setdefault("when", time.strftime("%Y-%m-%d %H:%M"))
    path = os.path.join(here, RESULT_NAME)
    try:
        with open(path + ".tmp", "w") as f:
            json.dump(d, f)
        os.replace(path + ".tmp", path)
    except OSError as e:
        print(f"note: could not write {RESULT_NAME} ({e})")


def bridge_fw(rows):
    for r in rows:
        v = r.get("fw")
        if v:
            m = re.search(r"(\d+)\.(\d+)", str(v))
            if m:
                return int(m.group(1)) * 100 + int(m.group(2))
    return -1


# Main
def get_image(a, here: str, board: str, state, fetch: bool) -> tuple:
    """
    This run's image: from --file, from a version ams_firmware.json lists (an intact copy beside
    this script, else downloaded from Bambu when `fetch`), or the one the run being resumed used.

    :return tuple: (the image, or None while it is still to be downloaded; a label; its path)
    """
    if a.file or (state and not a.version):
        name = a.file or state.get("file") or ""
        path = name if os.path.isabs(name) or os.sep in name else os.path.join(here, name)
        if not os.path.isfile(path):
            raise Abort(f"there is no file {name} beside ams_flash.py")
        version = a.version or ("" if a.file else state.get("version", ""))
        try:
            image, label = image_from_file(path, board, version)
        except (OSError, ValueError, zipfile.BadZipFile) as e:
            raise Abort(f"{os.path.basename(path)}: {e}")
        return image, label, path
    if not a.version:
        raise Abort("say which image to write: --version (one ams_firmware.json lists) or "
                    "--file")
    listed = [e for e in load_catalog(here)
              if e["version"] == a.version and (not board or e["board"] == board)]
    if not listed:
        raise Abort(f"{CATALOG_NAME} lists no {a.version} for "
                    f"{describe(board) if board else 'this unit'}")
    if len(listed) > 1:
        raise Abort(f"{a.version} comes for {' and '.join(e['board'] for e in listed)}; read "
                    f"the unit's board first (AFC_BAMBU_VERSION) or name it with --board")
    entry = listed[0]
    path = os.path.join(here, os.path.basename(entry["file"]))
    want = (entry.get("md5") or "").lower()
    if not (os.path.isfile(path) and (not want or md5_of(path) == want)):
        if not fetch:
            return (None, f"{entry['version']} for {describe(entry['board'])}: downloaded "
                          f"from Bambu when the update starts", path)
        print(f"    downloading {os.path.basename(entry['file'])} from Bambu")
        try:
            path = download_image(entry, here)
        except Exception as e:
            raise Abort(f"the download from Bambu failed ({e}); to update offline, put the "
                        f"image beside ams_flash.py")
    with open(path, "rb") as f:
        return f.read(), os.path.basename(path), path


STATE_MARK = "#~# --- AFC_BridgeBox managed state -- everything below is auto-written ---"


def wifi_bridge(cfg_dir: str, settings, usb_port: str) -> tuple:
    """
    (address, link key) a chain on connection: wifi saved for its bridge, found by the bridge's
    USB port in the chain's #~# state block, or (None, None).
    """
    files = [str(opts.get("state_file")) for sec, opts in (settings or {}).items()
             if sec.startswith("afc_bridgebox") and (opts or {}).get("state_file")]
    for root, _dirs, names in os.walk(cfg_dir):
        files += [os.path.join(root, n) for n in names if n.endswith(".cfg")]
    for path in files:
        try:
            with open(os.path.expanduser(path), errors="replace") as f:
                text = f.read()
        except OSError:
            continue
        if STATE_MARK not in text:
            continue
        cp = configparser.RawConfigParser(delimiters=(":", "="))
        try:
            cp.read_string("\n".join(ln[4:] for ln in text.split(STATE_MARK, 1)[1].splitlines()
                                     if ln.startswith("#~# ")))
        except configparser.Error:
            continue
        for sec in cp.sections():
            addr = cp.get(sec, "wifi_addr", fallback="").strip()
            if cp.get(sec, "usb_port", fallback="").strip() == usb_port and addr.startswith("tcp://"):
                return addr, _clean_key(cp.get(sec, "wifi_key", fallback=None))
    return None, None


def bridge_of(a, row: dict, settings) -> tuple:
    """
    (port, link key) of the bridge a unit is on.
    """
    port = a.target or row["port"]
    if str(port or "").lower().startswith("auto"):
        # The config files can't say which board auto took.
        error_str = (f"{row['name']}'s bridge is on serial_port: auto, and Klipper names no "
                     f"USB port for it; name the port with --target")
        raise Abort(error_str)
    if not port or not str(port).startswith(("tcp://", "/")):
        try:
            port = bridge_from_settings(settings)[0]
        except ValueError as e:
            raise Abort(str(e))
        port = port or find_in_cfg(a.cfg_dir, "serial_port")
    if not port:
        raise Abort(f"no bridge found for {row['name']}; name it with --target")
    key = a.key if a.key is not None else (
        (bridge_keys(settings).get(port) if settings is not None else None)
        or _clean_key(find_in_cfg(a.cfg_dir, "tcp_key")) or "")
    if not a.target and not port.startswith("tcp://") and not os.path.exists(port):
        # A bridge on connection: wifi keeps its USB name as its port, unplugged or not.
        addr, saved = wifi_bridge(a.cfg_dir, settings, port)
        if addr:
            port, key = addr, a.key if a.key is not None else (saved or "")
    if not port.startswith("tcp://"):
        key = ""        # the link key is a network feature.
        if not os.path.exists(port):
            raise Abort(f"bridge port {port} does not exist; name the port with --target")
    else:
        host, _s, num = port[6:].partition(":")
        try:
            if not host or not 0 < int(num or 8888) < 65536:
                raise ValueError
        except ValueError:
            raise Abort(f"bridge target {port} is not tcp://host:port")
    return port, key


def update(a, here: str, moon, result: dict) -> int:
    """
    Check everything, then (mode go) write the image into the unit; Abort says why it stopped.

    :return int: the exit code
    """
    state_path = os.path.join(here, STATE_NAME)

    print("[1] printer not printing")
    try:
        st = moon.q("print_stats").get("state")
    except Exception as e:
        raise Abort(f"cannot read the print state ({e})")
    print(f"    state = {st}")
    if st in ("printing", "paused"):
        raise Abort("a print is active")

    print("[2] the unit")
    try:
        rows = unit_rows(moon)
    except Exception as e:
        raise Abort(f"cannot read the AMS units ({e})")
    state = read_state(state_path)
    want = a.unit or (state or {}).get("unit")
    if not want:
        raise Abort("name the unit to update, e.g. --unit Bambu_AMS_1")
    row = next((r for r in rows if r["name"].lower() == str(want).lower()), None)
    if row is None:
        raise Abort(f"there is no unit {want} (units: "
                    f"{', '.join(r['name'] for r in rows) or 'none'})")
    unit = result["unit"] = row["name"]
    resuming = bool(state) and state.get("unit") == unit
    if state and not resuming:
        # One record at a time: this run must not replace one that still has a unit to finish.
        other = next((r for r in rows if r["name"] == state["unit"]), None)
        if other is None:
            raise Abort(f"{STATE_NAME} says {state['unit']} waits in its bootloader, and Klipper "
                        f"has no such unit; delete that file if the unit is gone")
        if not other["online"]:
            raise Abort(f"{other['name']} waits in its bootloader from an earlier run; finish "
                        f"its update first")
    if row["model"] not in ("ams1", "ams2", "ht", "boxed"):
        raise Abort(f"{unit} is ams_model {row['model'] or 'unset'}; only an AMS 1, AMS 2 Pro "
                    f"or AMS HT updates here")
    if resuming:
        unit_id = int(state.get("unit_id") or 0)
    elif row["index"] is None:
        raise Abort(f"Klipper has no chain index for {unit}")
    else:
        unit_id = mc_id_for_index(row["index"])
    known = f"  firmware {row['version']} ({row['code']})" if row["version"] else ""
    print(f"    {unit}: {'online' if row['online'] else 'offline'}  "
          f"ams_model={row['model']}  index={row['index']}  bus id 0x{unit_id:02x}{known}")
    if resuming:
        print(f"    resuming: an earlier run ({state.get('when')}) left it waiting in its "
              f"bootloader")
    elif not row["online"]:
        raise Abort(f"{unit} is offline. Power it, wait until it shows online, and retry.")
    drying = [r["name"] for r in rows if r["drying"] and r["port"] == row["port"]]
    if drying:
        raise Abort(f"{', '.join(drying)} {'is' if len(drying) == 1 else 'are'} drying on the "
                    f"same bridge; stop the dry and retry")
    settings = None
    try:
        settings = moon.settings()
    except Exception:
        pass
    port, key = bridge_of(a, row, settings)
    print(f"    bridge: {port}")
    board = (a.board or (state.get("board") if resuming else "")
             or board_of_code(row["code"]) or DEFAULT_BOARD.get(row["model"], ""))
    result["from"] = row["version"]

    print(f"[3] bridge fw >= {TARGET_FW}")
    fw = bridge_fw([row])
    print(f"    bridge_fw = {fw if fw > 0 else 'unknown'}")
    if fw < TARGET_FW:
        raise Abort(f"the bridge firmware is older than {TARGET_FW}; update the bridge first")

    print("[4] the image")
    image, label, path = get_image(a, here, board, state if resuming else None,
                                   fetch=a.mode == "check" or a.detached)
    print(f"    {label}")
    info = None
    if image is None:
        result["to"] = a.version
    else:
        try:
            info = parse_image(image)
        except ValueError as e:
            raise Abort(f"{label}: {e}")
        print(f"    {info['name']}: {describe(info['board'])} {info['version']}, "
              f"{len(image)} bytes, MD5 {hashlib.md5(image).hexdigest()}")
        fits = ("ams1", "ams2") if row["model"] == "boxed" else (row["model"],)
        if info["model"] not in fits:
            raise Abort(f"{label} is for an {describe(info['board'])}, but {unit} is set up as "
                        f"ams_model {row['model']}")
        if board and info["board"] != board:
            raise Abort(f"{label} is for {describe(info['board'])}, but {unit} is "
                        f"{describe(board)}")
        result["to"] = info["version"]

    if a.mode == "check":
        print("\n==> CHECKS PASSED. Nothing was changed. --mode go updates the unit, after "
              "checking the bus itself.")
        return 0

    if not a.detached:
        # The update outlasts the shell command that started it.
        result.update(running=True, message=f"Updating {unit}. The units on its bridge read "
                                             f"offline until it is done.")
        write_result(here, **result)
        try:
            relaunch_detached()
        except OSError as e:
            result["running"] = False
            raise Abort(f"could not start the update on its own: {e}")
        print("launched detached; progress in logs/ams_flash.log")
        return 0

    # One update at a time.
    import fcntl
    import signal
    lock_path = os.path.join(os.path.dirname(a.cfg_dir.rstrip("/")), "logs", "ams_flash.lock")
    try:
        os.makedirs(os.path.dirname(lock_path), exist_ok=True)
        lock = open(lock_path, "w")
    except OSError as e:
        print(f"note: no lock file ({e}); not guarding against a second run")
        lock = None
    if lock is not None:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            raise Abort("another AMS update is already running; let it finish")

    # A stop of this process (systemctl, shutdown, logout) must still run the finally blocks
    # below, which give Klipper the bridge link back.
    def _stopped(signum, _frame):
        raise SystemExit(f"stopped by signal {signum}")
    signal.signal(signal.SIGTERM, _stopped)
    signal.signal(signal.SIGHUP, _stopped)

    art = pack(image, unit_id)
    dev = device_of(unit_id)
    n = row["index"]
    moon.gcode(f'RESPOND MSG="AMS update: {unit} to {info["version"]}. The units on its bridge '
               f'read offline until it is done."')
    print("[5] the bus (Klipper lends this run the bridge link; a bridge takes one client)")
    rc = 1
    link = None
    try:
        if not hold_bridge(moon, unit, HOLD_S):
            raise Abort("Klipper would not let go of the bridge link")
        try:
            link = open_link(port)
            if not authenticate(link, key):
                raise Abort("the bridge refused the link key")
            reset_bridge_mode(link)
            wire, drying = wire_status(link)
            print("    answering on the bus: "
                  + (", ".join(f"index {m} (bus id 0x{mc_id_for_index(m):02x})"
                               for m in wire) or "none"))
            if drying:
                # A boxed dry deafens the bridge's receiver.
                raise Abort("a dry cycle is running on this bridge; stop it and retry")
            if resuming and n in wire and read_version(link, unit_id):
                print(f"    {unit} runs its firmware again, so the record of it waiting in its "
                      f"bootloader is stale")
                clear_state(state_path)
                resuming = False
            if resuming:
                if loader_on_bus(link, exclude=(dev, unit_id)):
                    raise Abort("another unit on the bus is sitting in its bootloader; "
                                "power-cycle it and retry")
            else:
                if n not in wire:
                    raise Abort(f"{unit} (index {n}) is not answering the bridge")
                got = read_version(link, unit_id)
                if not got:
                    raise Abort(f"{unit} did not answer the version query")
                ver, code = got
                result["from"] = ver
                print(f"    {unit}: firmware {ver}, model code {code}")
                if board_of_code(code) != info["board"]:
                    raise Abort(f"{unit} is {describe(board_of_code(code) or code)}, and "
                                f"{label} is for {describe(info['board'])}")
                if ver == info["version"]:
                    print("    it already runs this version; writing it again")
                if loader_on_bus(link):
                    raise Abort("a unit on the bus is sitting in its bootloader; "
                                "power-cycle it, or finish its update first")
            # The sniff stopped the polling.
            settle(link, ONLINE_SETTLE_S)
            others = [m for m in wire_online(link) if m != n]
            print("    OK")

            # Recorded before the unit is sent into its loader, so a run that stops anywhere
            # after this is resumed by running the update again.
            if not write_state(state_path, unit=unit, unit_id=unit_id, board=info["board"],
                               version=info["version"], file=path,
                               when=time.strftime("%Y-%m-%d %H:%M")):
                raise Abort("without a record of it, a stopped update could not be resumed, so "
                            "the unit is not sent into its loader")

            print("[6] cmd1 enters the loader (non-destructive)")
            if not probe_cmd1(link, art):
                raise Abort(f"{unit} did not enter its loader"
                            + ("; if it was power-cycled since, wait until it shows online "
                               "and retry" if resuming else ""))
            if others:
                settle(link, ONLINE_SETTLE_S)
                lost = [m for m in others if m not in wire_online(link)]
                if lost:
                    raise Abort(f"index {', '.join(map(str, lost))} stopped answering when "
                                f"{unit} entered its loader. Nothing was erased; power-cycle "
                                f"the units and retry")

            print(f"[7] FLASH: header (erase) + {len(art['blocks'])} blocks")

            def _reconnect():
                # A Klipper restart drops the hold, and a bridge serves one client.
                if not hold_bridge(moon, unit, HOLD_S):
                    raise IOError("Klipper would not let go of the bridge link again")
                link.reopen()
                if not authenticate(link, key):
                    raise IOError("link auth failed after reopening")
            rc = do_flash(link, art, reconnect=_reconnect)
            if rc == BADIMG:
                raise Abort(f"{unit} took every block and refused the image. Its bootloader is "
                            f"intact: update it again with the right image")
            if rc != 0:
                raise Abort(f"the update did not finish and {unit} waits in its bootloader; "
                            f"run the update again to finish it")
            clear_state(state_path)

            print("[8] the unit boots its new firmware")
            got = None
            t0 = time.time()
            while got is None and time.time() - t0 < REJOIN_S:
                if n in wire_online(link):
                    got = read_version(link, unit_id)
                if got is None:
                    time.sleep(2)
            was = f" (was {result['from']})" if result["from"] else ""
            if got is None:
                result.update(ok=True, to=info["version"],
                              message=f"{unit} took {info['version']}{was}, but did not answer "
                                      f"within {REJOIN_S}s; check that it shows online.")
            elif got[0] != info["version"]:
                raise Abort(f"{unit} took the image but answers {got[0]}, not "
                            f"{info['version']}")
            else:
                print(f"    {unit}: firmware {got[0]}, model code {got[1]}")
                result.update(ok=True, to=got[0], message=f"{unit} now runs {got[0]}{was}.")
            print(f"\n==> DONE. {result['message']}")
            rc = 0
        except OSError as e:
            raise Abort(f"the bridge link failed ({e})")
        finally:
            if link is not None:
                if not link.dead:
                    release_bridge(link)
                link.close()
    finally:
        if not hold_bridge(moon, unit, 0):
            print("    note: could not give Klipper the bridge link back; it takes it back by "
                  "itself within 30 minutes, or at a Klipper restart")
    return rc


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unit", default=None,
                    help="the unit to update, as Klipper names it (e.g. Bambu_AMS_1); default: "
                    "the unit an unfinished update left in its bootloader")
    ap.add_argument("--version", default=None,
                    help="the version to write, one ams_firmware.json lists (downloaded from "
                    "Bambu); with --file, the one to take from a package")
    ap.add_argument("--file", default=None,
                    help="a Bambu .bin.sig or offline update package (.zip), beside this "
                    "script or by path")
    ap.add_argument("--board", default=None,
                    help="the unit's board, e.g. ams_rev8 (default: from the model code "
                    "Klipper read)")
    ap.add_argument("--mode", default="check", choices=["check", "go"],
                    help="check (default) changes nothing; go updates the unit")
    ap.add_argument("--cfg-dir", default=None,
                    help="Klipper config root (default: auto-detected from this file's "
                    "location)")
    ap.add_argument("--target", default=None,
                    help="bridge tcp://host:port or USB serial path (default: the unit's own)")
    ap.add_argument("--key", default=None, help="link key (default: tcp_key from config)")
    ap.add_argument("--detached", action="store_true",
                    help=argparse.SUPPRESS)   # internal: set on the relaunched copy
    a = ap.parse_args()
    if not a.cfg_dir:
        a.cfg_dir = config_root(here)

    # Tee all output to printer_data/logs/ams_flash.log.
    try:
        logdir = os.path.join(os.path.dirname(a.cfg_dir.rstrip("/")), "logs")
        os.makedirs(logdir, exist_ok=True)
        lf = open(os.path.join(logdir, "ams_flash.log"), "a", buffering=1)

        class _Tee:
            def __init__(self, *s): self.s = s
            def write(self, x):
                for f in self.s:
                    try: f.write(x); f.flush()
                    except Exception: pass
            def flush(self):
                for f in self.s:
                    try: f.flush()
                    except Exception: pass
        sys.stdout = _Tee(sys.stdout, lf)
        sys.stderr = _Tee(sys.stderr, lf)
        print(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} =====")
    except Exception:
        pass

    print(f"=== AMS update: {a.unit or '(unfinished update)'} "
          f"{a.version or a.file or ''} mode={a.mode.upper()} ===")
    moon = Moon()
    result = {"unit": a.unit or "", "ok": False, "from": "", "to": a.version or "",
              "message": ""}
    try:
        rc = update(a, here, moon, result)
    except Abort as e:
        print(f"    ABORT: {e}")
        result["message"] = str(e)
        rc = 1
    except SystemExit as e:
        # Stopped by a signal: the launch left "running", and the menu must not keep showing it.
        if a.mode == "go":
            write_result(here, **{**result, "message": f"The update was {e}; run it again."})
        raise
    if a.mode == "go" and not result.get("running"):
        write_result(here, **result)
        if a.detached and moon.state() == "ready":
            moon.gcode("AFC_BAMBU_FIRMWARE SHOW=RESULT")
    return rc


if __name__ == "__main__":
    sys.exit(main())
