#!/usr/bin/env python3
"""
Self-contained AMS firmware updater for the Bambu-AMS bridge.

Drop this file, `ams_flash.cfg` (the gcode_shell_command + macros) and the
artifact for your model into your Klipper config directory, then run from the
console, one command per model:

    AFC_BAMBU_AMS_UPDATE    MODE=check|preflight|go   # AMS 2 Pro  ams2_artifact.json
    AFC_BAMBU_AMS1_UPDATE   MODE=check|preflight|go   # AMS 1      ams1_artifact.json
    AFC_BAMBU_HT_UPDATE     MODE=check|preflight|go   # AMS HT     ht_artifact.json

MODE defaults to preflight (probes, no erase); check is read-only; go erases.
Nothing here imports from the bridge repo -- only the Python standard library --
so these files are all a machine needs. The bridge may be a WiFi (tcp://) or a
USB one; the port is read from the Klipper config.

WHAT IT DOES. Sends the AMS a cmd1 frame, which makes the running unit jump into
its bootloader, then streams the firmware (one header + N data blocks); the
unit erases its app flash, writes the new image, reboots and rejoins the bus.
The bootloader is never erased, so an interrupted transfer leaves a RECOVERABLE
unit in its loader -- just run MODE=go again. The run records which unit it
sent into the loader (ams_flash_state.json beside this file), and a later run
of the same model's command picks that unit up again even though it no longer
shows online.

SAFETY GATES, all of which must pass before the erase:
    1 not printing
    2 artifact verifies, and is addressed to and named for this model
    3 bridge fw >= 1.75
    4 Klipper: exactly one AMS online, this model, at the image's bus address
    5 the bus itself: exactly one unit answering, at that address; asked
      directly, it answers as this model (AMS 1 vs AMS 2); no other unit is
      sitting in its bootloader
    6 cmd1 enters the loader (non-destructive)
Only MODE=go, and only past 1-6, streams the erase+image.

Requirements: the bridge must already run fw >= 1.75. If it is older this
refuses with instructions -- updating the bridge is a separate, one-time step.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import os
import re
import socket
import sys
import time
import urllib.request

# ── constants (all confirmed against real printer<->AMS captures) ─────────────
# Bridge fw floor. 1.68 brought fwreplay/txbuf/txsend, 1.74 the loader's
# verdict in the block reply windows (without it every pass reads as failed
# and a good flash is erased again), 1.75 the staged-frame CRC check.
TARGET_FW = 175
CONFIRM = "ERASE-AMS-FW"        # the firmware's fwreplay confirm token
CHUNK = 100                     # hex-payload bytes per txbuf line
CRC8_POLY, CRC8_INIT = 0x39, 0x66
CRC16_POLY, CRC16_INIT = 0x1021, 0x913D
MOON = "http://127.0.0.1:7125"
# The loader's OWN verdict, read from the block-send reply windows (fw >= 1.74).
# A block ack is only receipt; success! is the whole-image hash passing and the
# loader resetting into the app -- the only trustworthy "it took and rebooted".
SUCCESS_HEX = "7375636365737321"       # "success!"
CHUNKERR_HEX = "6368756e6b2068617368"  # "chunk hash" (per-chunk hash failure)
# "img hash": every chunk was accepted and the assembled image still failed the
# loader's whole-image check. That is the image being wrong for this unit, not
# the link, so resending the same bytes cannot help.
IMGERR_HEX = "696d672068617368"
FLASH_RETRIES = 6               # whole-flash resends after the first attempt (7 total)
BADIMG = 3                      # do_flash: the loader refused the image itself
STATE_NAME = "ams_flash_state.json"  # the unit this run sent into its loader
# The addressed generation query at step 5: an AMS 2 answers it, an AMS 1
# never does. GEN_AMS2_MIN answers out of GEN_ASKS confirm an AMS 2 (the
# bridge's own verdict takes 3); an AMS 1 must stay silent through all of them.
GEN_ASKS = 8
GEN_AMS2_MIN = 3
GEN_CTL_MIN = 4                 # drain answers proving the unit is listening
ONLINE_SETTLE_S = 3.5           # two of the bridge's 1.5 s online timeouts

#: One entry per update command. `dev` and `ams_id` are what the model's update
#: frames are addressed to, and an artifact must carry exactly those (step 2):
#: a boxed AMS is device 0x0700, AMS id 0x00 when it is alone on the bus; an HT
#: is device 0x1800, AMS id 0x80. `image` is how the vendor's own file name for
#: that model's image starts; the header frame carries it, and it is the only
#: thing in an artifact that tells an AMS 1 image from an AMS 2 one.
MODELS = {
    "ams2": {"name": "AMS 2 Pro", "artifact": "ams2_artifact.json",
             "legacy_artifact": "ams_artifact.json",
             "dev": 0x0700, "ams_id": 0x00, "image": "n3f_"},
    "ams1": {"name": "AMS 1", "artifact": "ams1_artifact.json",
             "dev": 0x0700, "ams_id": 0x00, "image": "ams_"},
    "ht":   {"name": "AMS HT", "artifact": "ht_artifact.json",
             "dev": 0x1800, "ams_id": 0x80, "image": "n3s_"},
}


def mc_id_for_index(idx: int) -> int:
    """The bus address the bridge enrolls chain index `idx` at.

    Indices 0-3 map to themselves, above that 0x80 + (index - 4). A copy of
    the rule in extras/AFC_BambuAMS.py, which this file cannot import.
    """
    i = int(idx)
    if i < 0:
        return 0
    return i if i < 4 else (0x80 + (i - 4)) & 0xFF


# ── CRCs ──────────────────────────────────────────────────────────────────────
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
    """The enter-loader / loader-handshake frame: op 0601, cmd 01, source 0x0900.

    Addressed per model: [7:9] is the device (0x0700 boxed, 0x1800 HT), [14]
    is that device's high byte, and [19] is the AMS id (0x00 boxed, 0x80 HT).
    An HT ignores the boxed shape, so both have to follow the artifact.
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
    """Rebuild a full frame from its invariant body with a fresh counter+CRCs."""
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


def load_artifact(path: str):
    """Read + verify the per-model artifact (header + N block bodies)."""
    d = json.load(open(path))
    art = {"header": bytes.fromhex(d["header"]),
           "blocks": [bytes.fromhex(b) for b in d["blocks"]]}
    steps = list(plan(art))
    bad = [s for _k, s, fr in steps
           if crc8(fr[:6]) != fr[6] or crc16(fr[:-2]) != (fr[-2] | (fr[-1] << 8))]
    if bad or len(steps) != 1 + len(art["blocks"]):
        raise ValueError("artifact failed re-verification")
    return art


#: Bytes of BIMH container header counted in the update's declared total but
#: NOT carried in the data blocks' own firmware-byte counts. Measured as
#: exactly 416 on both models (AMS 2: 172132 - 171716; AMS HT: 160552 -
#: 160136).
BIMH_HEADER_LEN = 416


def loader_target_of(art):
    """(device address, AMS id) the artifact's frames are addressed to.

    The artifact stores frame bodies (frame[7:-2]), so the target at frame[7:9]
    is body[0:2] and the AMS id at frame[19] is body[12]. The cmd1 poke is
    built from these; step 5 checks that the one unit on the bus sits there.
    """
    hdr = art["header"]
    return hdr[0] | (hdr[1] << 8), hdr[19 - 7]


def artifact_image_name(art) -> str:
    """The vendor file name the header frame carries for its image, or ''.

    The header's payload is the image's BIMH container header, which names
    the file it came from: n3f_... for an AMS 2, n3s_... for an HT,
    ams_rev... for an AMS 1.
    """
    hdr = art["header"]
    i = hdr.find(b"BIMH")
    if i < 0:
        return ""
    m = re.search(rb"[ -~]+?\.bin\.sig", hdr[i + 4:])
    return m.group(0).decode("ascii", "replace") if m else ""


def artifact_mismatch(art, model: str):
    """Why this artifact must not be sent by `model`'s command, or None.

    The frames carry the device they are for, so an HT image cannot go out
    under a boxed command or the other way round. An AMS 1 and an AMS 2 image
    share an address, so the image's own name has to match too; a file saved
    under the wrong model's name is refused here, before anything is sent.
    """
    m = MODELS[model]
    dev, ams_id = loader_target_of(art)
    if (dev, ams_id) != (m["dev"], m["ams_id"]):
        return (f"this artifact is addressed to device 0x{dev:04x}, AMS id "
                f"0x{ams_id:02x}, not to an {m['name']} (device "
                f"0x{m['dev']:04x}, AMS id 0x{m['ams_id']:02x}). Wrong file "
                f"for this command?")
    name = artifact_image_name(art)
    if not name.startswith(m["image"]):
        return (f"this artifact carries the image '{name or 'unnamed'}', not "
                f"an {m['name']} image ({m['image']}...). Wrong file for this "
                f"command?")
    hdr = art["header"]
    if hdr[4:7] != b"\x06\x01\x02":
        return "the artifact's first frame is not an update header"
    for s, b in enumerate(art["blocks"]):
        if b[0:2] != hdr[0:2] or b[12] != hdr[12] or b[4:7] != b"\x06\x01\x03":
            return (f"data block {s} is not an update block addressed like "
                    f"the header; the file is damaged or mixed")
    return None


def artifact_image_bytes(art):
    """(declared total image bytes, bytes actually carried by the blocks).

    Bodies are captured frame[7:-2], hence the -7 on both offsets: the header
    frame's total image size lives at frame [23:26] and each data block's own
    firmware-byte count at frame [35:39].
    """
    hdr = art["header"]
    total = int.from_bytes(hdr[23 - 7:26 - 7], "little")
    summed = sum(int.from_bytes(b[35 - 7:39 - 7], "little")
                 for b in art["blocks"])
    return total, summed


def plan(artifact, counter0: int = 0):
    c = counter0
    yield ("header", -1, wrap(0x00, c, artifact["header"])); c += 1
    for s, body in enumerate(artifact["blocks"]):
        yield ("data", s, wrap(0x00, c, body)); c += 1


# ── bridge link (tcp:// or USB serial) + link-key handshake ───────────────────
class Link:
    """A line link to the bridge over TCP or its USB serial port.

    The serial side uses the standard library only (no pyserial, so this still
    runs on the system python3), and never makes a blocking call: the port is
    opened non-blocking, reads take only what is already buffered, and writes
    give up after WRITE_TIMEOUT. A USB-CDC bridge busy on the AMS bus can stop
    draining, and a blocking call there would hang the flash with Klipper
    stopped. A timed-out write raises IOError, which do_flash treats as a
    stalled pass (the unit stays in its loader and the pass is resent).

    A link that has gone away (the USB port hung up, the socket closed) is
    marked `dead` and raises IOError instead of reading as a quiet port;
    reopen() then brings it back for the next pass.
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
        """Close and open the link again, waiting for the port to come back
        (a USB bridge that reset re-enumerates under the same by-id path)."""
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
        """Whatever is buffered now, waiting at most `timeout` for some.

        b"" means quiet. A port that reports readable and then has nothing is
        a hangup, not quiet: raise, or every caller spins on it until its own
        deadline and the next pass reuses a dead descriptor."""
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


def authenticate(link: Link, key: str, timeout: float = 8.0) -> bool:
    """board -> {"nonce"}; host -> HMAC-SHA512(key,nonce)[:32]; board -> {"ok":1}."""
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
    """The next line whose event is exactly `evt` (not merely mentioning it:
    the chain report has a "raw" field, the status report a "state" one)."""
    tag = f'"evt":"{evt}"'
    t0 = time.time()
    for line in link.lines():
        if line and tag in line.replace(" ", ""):
            return line
        if time.time() - t0 > timeout:
            return None


def _rx_of(line: str) -> bytes:
    """The captured bytes in a raw/txsend reply; b"" (no answer) if there are
    none or they arrived damaged (an odd-length hex string after a lost byte)."""
    m = re.search(r'"rx"\s*:\s*"([0-9A-Fa-f]*)"', line or "")
    try:
        return bytes.fromhex(m.group(1)) if m else b""
    except ValueError:
        return b""


def _seqretry(rx: bytes):
    """Parse the loader's '[MCU_UP] N seq_num: X, retry: M' out of a reply.

    Returns (want, retry): `want` is the NEXT seq the loader expects (hex on the
    wire -> int), `retry` its attempt counter. After block N is accepted the
    loader answers want == N+1; if it does not advance it is asking for that
    chunk again. None when the reply carries no such narration (e.g. the
    completion frame, which instead carries success! / chunk hash)."""
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
    """Stage one frame across txbuf lines, then txsend; return the AMS reply.

    The bridge checks the staged frame's own CRC16 before transmitting (fw >=
    1.75). A txbuf line mangled on the way (WiFi) draws "crc":"bad" and
    nothing goes out, so this one frame is staged again; otherwise a single
    damaged line would read as "no ack" and cost a whole erase pass."""
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
    """Every whole, CRC-valid long frame in a bus capture, in order.

    A capture is our own echo followed by whatever answered, with no framing
    of its own, so a pattern search over its hex lands across frame
    boundaries, inside payloads and on half bytes. Only a frame that parses
    end to end counts: 0x3D sync, header CRC8 over [0:6], the total length at
    [4:6], and the CRC16 over the whole frame.
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
    """The loader's own answer in a capture window, if there is one.

    Keyed on the SOURCE, never the class byte. Class 0x04 is the shape of our
    own cmd1, so a window holding our echo would otherwise read as "the loader
    answered" and hand off to the erase at a unit that said nothing. An
    op-0601 answer counts only as a whole CRC-valid frame whose source at
    [9:11] is not the updater's 0x0900, so a damaged echo, a stray 06 01 in a
    payload or a match straddling two bytes cannot pass. The loader's own
    text (its banner, "wait cmd1") counts on a byte boundary.
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


# ── step 5: what is actually on the bus ───────────────────────────────────────
def build_3702(ams_id: int) -> bytes:
    """The printer's addressed generation query (op 0x3702, [15] = 0x01).

    Only the unit enrolled at `ams_id` answers it, and only an AMS 2 answers
    at all: an AMS 1 has no answer to 0x3702. Captured from a real printer
    (on a dual-unit cold boot the query to the AMS 1's id drew nothing, the
    one to the AMS 2's id drew the version reply). [13] is the id.
    """
    f = bytearray.fromhex("3d05000012001200070003370200" "0001")
    f[13] = ams_id & 0xFF
    c = crc16(bytes(f))
    return bytes(f) + bytes((c & 0xFF, c >> 8))


def build_drain(ams_id: int) -> bytes:
    """The 1A02 log-drain poll to `ams_id`, byte-identical to the bridge's own
    online poll. Both boxed generations answer it from their app, so it is
    the control for the generation query: a unit that answers this and not
    0x3702 is an AMS 1 that is listening, not an AMS 2 that missed the asks."""
    f = bytearray.fromhex("3d05000013008e000700031a0200000000")
    f[13] = ams_id & 0xFF
    c = crc16(bytes(f))
    return bytes(f) + bytes((c & 0xFF, c >> 8))


def answers_op(raw: bytes, op: bytes, ams_id: int) -> bool:
    """Whether a capture holds a boxed unit's own answer to `op`: a whole
    frame from device 0x0700 with that op and the unit's id at [13] (our
    queries come from 0x0300, so their echo never counts)."""
    return any(len(f) > 13 and f[11:13] == op
               and (f[9] | (f[10] << 8)) == 0x0700 and f[13] == ams_id
               for f in frames(raw))


def answers_3702(raw: bytes, ams_id: int) -> bool:
    """Whether a capture holds a unit's own 0x3702 answer: a whole frame from
    device 0x0700 with op 3702 and the answering unit's id at [13] (our query
    is from 0x0300, so its echo never counts)."""
    return answers_op(raw, b"\x37\x02", ams_id)


def settle(link: Link, seconds: float):
    """Read (and drop) whatever the bridge says for `seconds`. A plain sleep
    would let its status reports pile up unread in the link."""
    t0 = time.time()
    for _line in link.lines():
        if time.time() - t0 >= seconds:
            return


def reset_bridge_mode(link: Link):
    """Put the bridge back to polling the bus, whatever an earlier run left.

    A run that died in fwreplay (or in the sniff at step 5) leaves the bridge
    there: link loss does not end either mode. Neither mode polls, and a
    unit's online flag is only refreshed by polling, so the flags would read
    as they were when the old run stopped -- a unit already in its loader
    still "online". Both modes are switched off and the bus is given time to
    refresh every flag (the bridge ages a silent unit out after 1.5 s)."""
    for cmd, evt in (({"cmd": "fwreplay", "on": 0}, "fwreplay"),
                     ({"cmd": "sniff", "on": 0}, "sniff_mode")):
        link.send(cmd)
        if _await_evt(link, evt, 4.0) is None:
            raise IOError(f"bridge did not answer {cmd['cmd']} off")
    settle(link, ONLINE_SETTLE_S)


def release_bridge(link: Link):
    """Best effort on the way out: never leave the bridge in fwreplay or
    sniff for Klipper, which cannot drive the units in either."""
    for cmd, evt in (({"cmd": "fwreplay", "on": 0}, "fwreplay"),
                     ({"cmd": "sniff", "on": 0}, "sniff_mode")):
        try:
            link.send(cmd)
            _await_evt(link, evt, 3.0)
        except OSError:
            return


def wire_status(link: Link, tries: int = 3):
    """(online, drying): chain indices the bridge itself sees answering now,
    and those running a dry cycle, from its status report. Units Klipper has
    no section for count here too."""
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


def wire_online(link: Link, tries: int = 3):
    return wire_status(link, tries)[0]


def probe_generation(link: Link, ams_id: int, asks: int = GEN_ASKS):
    """(0x3702 answers, drain answers) from the boxed unit at `ams_id`, over
    `asks` rounds of one query each, interleaved so both see the same bus.
    Read-only for the unit; `raw` sends one frame and captures."""
    gen, ctl = build_3702(ams_id).hex(), build_drain(ams_id).hex()
    got = [0, 0]
    for _ in range(asks):
        for k, (q, op) in enumerate(((gen, b"\x37\x02"), (ctl, b"\x1a\x02"))):
            link.send({"cmd": "raw", "hex": q, "us": 100000})
            line = _await_evt(link, "raw", 3.0)
            if line is None:
                raise IOError("bridge did not answer raw")
            if answers_op(_rx_of(line), op, ams_id):
                got[k] += 1
    return got[0], got[1]


def loader_on_bus(link: Link, watch_s: float = 6.0, exclude=None) -> bool:
    """Whether some unit on the bus is sitting in its bootloader.

    Listen only (sniff: the bridge stops transmitting, and a running unit
    says nothing unless polled). A loader talks unasked when it starts, and
    "wait cmd1" has been seen repeating, so a loader heard here is a unit the
    online count cannot see. One that has gone quiet is not caught; the
    id-bound loader answer at step 6 is what keeps it from standing in for
    the target. `exclude` = (device, AMS id): that unit's own loader, when it
    is the one being resumed, is expected and does not count.
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
    """Whether a capture holds the ADDRESSED unit's loader answering cmd1.

    Either its op-0601 reply as a whole frame from `dev` carrying `ams_id` at
    [19] (the HT's: source 0x1800, id 0x80), or the loader's own narration
    naming that id: "ams 128 wait cmd1!", "128 resev cmd 0x1", "128 send cmd
    0x1 back" (the id in decimal). A banner with no id ("Loader Version")
    does not count: it cannot say which unit is talking, and another unit's
    loader must never stand in for the one about to be erased.
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
    """cmd1 until the loader announces. cmd1 only -- can never erase. No answer -> False.

    Uses txbuf/txsend, so it is only valid INSIDE fwreplay mode (do_flash)."""
    dev, ams_id = loader_target_of(art)
    cmd1 = build_cmd1(dev, ams_id)
    for i in range(tries):
        rx = send_frame(link, cmd1, window_us=600000)
        if rx and loader_answer(rx, dev, ams_id):
            print(f"    loader confirmed after cmd1 #{i + 1}")
            return True
    return False


def probe_cmd1(link: Link, art, tries: int = 4) -> bool:
    """Non-destructive loader probe, OUTSIDE fwreplay: send cmd1 via `raw`.

    `raw` sends one frame and captures the bus for the window (no fwreplay
    needed), so this works on a running unit without arming the erase path.
    cmd1 only; it can never carry a header. A running unit answers the first
    cmd1 by jumping, and its loader's banner names no unit, so it is usually
    the second cmd1, answered by the loader itself, that confirms."""
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
    """One erase+rewrite pass, LOADER-DRIVEN: send the block the loader's reply
    asks for (seq_num), resending in place any chunk it will not accept, so a
    glitched chunk is fixed within the pass instead of failing the whole image.
    Returns the loader's OWN verdict, read from the block windows:
      'success'  -- loader hashed the whole image OK; it reset into the app
      'badimage' -- chunk hash error, a stuck chunk, or no success! after all blocks
      'badimg'   -- every chunk accepted, then the whole-image hash failed
      'stall'    -- no loader answer / a block went unacked
    'badimage' and 'stall' both leave the unit IN the loader, so the caller may
    safely resend the whole pass. 'badimg' also leaves it in the loader, but
    resending the same image cannot change the verdict. No probe after the verdict, so a good flash is
    never mistaken for a failure and re-erased."""
    link.send({"cmd": "fwreplay", "on": 1, "confirm": CONFIRM})
    if not _await(link, "fwreplay", 4.0):
        print("    bridge did not answer fwreplay; reopening the link",
              file=sys.stderr)
        link.dead = True
        return "stall"
    try:
        # cmd1 handshake FIRST -- the loader ignores a header until it handshakes;
        # a header before this strands a half-written image. No answer -> no erase.
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

        # LOADER-DRIVEN transfer: each reply names the next seq the loader wants.
        # want == last+1 means accepted; a non-advancing want means resend that
        # chunk -- following it fixes a corrupted chunk WITHIN the pass (the way
        # the printer does) instead of failing the whole image. A corrupt
        # header/handshake still dooms the pass; that falls to the pass-level
        # retry in do_flash, which re-does the handshake.
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

        # All blocks accepted; the verdict is in the final reply's window.
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
            pass        # the link is gone; the pass already failed on that


def do_flash(link: Link, art, retries: int = FLASH_RETRIES,
             reconnect=None) -> int:
    """Erase + rewrite the AMS, retrying the WHOLE flash until the loader itself
    reports success!. The transfer intermittently corrupts a chunk; the loader
    catches it ('chunk hash error') and refuses to boot, so a single pass is a
    coin flip. Every non-success pass leaves the unit safely in its loader; a
    success! pass means it already reset into the app, so the loop returns and
    never re-erases a live unit. Returns 0 only on a loader-verified success.

    `reconnect` reopens a link that died (a USB bridge that reset, a dropped
    socket) before the next pass, instead of spending the remaining attempts
    on a dead descriptor."""
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
            # A staging/link stall (e.g. USB-CDC back-pressure) aborts this pass
            # but leaves the unit in the loader -- safe to re-erase.
            print(f"    attempt {attempt}: link stall -- {e}", file=sys.stderr)
            verdict = "stall"
        if verdict == "success":
            print("    FLASH CONFIRMED: the loader verified the image and reset "
                  "into the new firmware.")
            return 0
        if verdict == "badimg":
            print("    the loader accepted every chunk and rejected the assembled "
                  "image: this artifact is wrong for this unit (model or build), "
                  "not the link. Stopping rather than erasing again. The unit is "
                  "left in its loader (recoverable): do not power it off; run "
                  "the command again with a known-good artifact for this "
                  "model.", file=sys.stderr)
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


# ── Moonraker (localhost) for print-state, klipper control, verify ────────────
class Moon:
    def __init__(self, base=MOON):
        self.base = base

    def _req(self, method, path, timeout=30):
        r = urllib.request.Request(self.base + path, method=method)
        return json.loads(urllib.request.urlopen(r, timeout=timeout).read())

    def q(self, obj):
        import urllib.parse
        p = "/printer/objects/query?" + urllib.parse.quote(obj)
        return self._req("GET", p)["result"]["status"].get(obj, {})

    def objects(self):
        return self._req("GET", "/printer/objects/list")["result"]["objects"]

    def settings(self):
        """Klipper's parsed config, section names lower-cased."""
        p = "/printer/objects/query?configfile=settings"
        return (self._req("GET", p)["result"]["status"]
                .get("configfile", {}).get("settings", {}))

    def state(self):
        try:
            return self.q("webhooks").get("state", "?")
        except Exception:
            return "?"

    def service(self, action, name="klipper"):
        try:
            self._req("POST", f"/machine/services/{action}?service={name}", timeout=20)
            return True
        except Exception:
            return False


def klipper(moon, action):
    if moon.service(action):
        return True
    import subprocess
    for cmd in (["systemctl", action, "klipper"],
                ["sudo", "-n", "systemctl", action, "klipper"]):
        try:
            if subprocess.run(cmd, capture_output=True).returncode == 0:
                return True
        except Exception:
            pass
    return False


def relaunch_detached(pass_file: str) -> tuple:
    """Re-run this script in its OWN transient systemd unit so it survives the
    klipper bounce -- a child of the klipper service is killed when klipper
    stops, which mid-flash would strand a half-written AMS.

    Three ways, tried in order, so no manual setup is needed on a normal Klipper
    Pi:
      1. `systemd-run --user` -- NO sudo, NO password. Used only when the
         user's systemd manager lingers (`loginctl enable-linger`), which any
         Pi already running a --user service has.
         Without lingering that manager ends with the last login session, and
         the flash with it.
      2. `sudo -n systemd-run` -- a passwordless (NOPASSWD) sudoers grant, if one
         happens to be present.
      3. `sudo -S systemd-run` reading the password from `pass_file` (used once,
         then DELETED). Last resort, only if 1 and 2 both fail.

    Returns (rc, output); rc == 0 means the detached unit started.
    """
    import subprocess
    args = [x for x in sys.argv[1:] if x != "--detached"] + ["--detached"]
    unit = ["systemd-run", "--collect", "/usr/bin/python3",
            os.path.abspath(__file__)] + args

    # 1) user manager -- needs XDG_RUNTIME_DIR pointing at the user's runtime dir
    #    (not always set in the gcode_shell_command environment). Only with
    #    lingering on: otherwise the user manager lives only as long as a login
    #    session, and closing an SSH window would stop the flash halfway.
    env = dict(os.environ)
    env.setdefault("XDG_RUNTIME_DIR", "/run/user/%d" % os.getuid())
    if _lingering():
        r1 = subprocess.run(["systemd-run", "--user", "--collect",
                             "/usr/bin/python3", os.path.abspath(__file__)]
                            + args, capture_output=True, env=env)
        if r1.returncode == 0:
            return 0, "detached via systemd-run --user\n" + \
                      (r1.stdout + r1.stderr).decode("utf-8", "replace")
    else:
        r1 = subprocess.CompletedProcess(
            [], 1, b"", b"lingering is off for this user (enable it once with: "
            b"sudo loginctl enable-linger $USER)")

    # 2) passwordless sudo grant, if present.
    r2 = subprocess.run(["sudo", "-n"] + unit, capture_output=True)
    if r2.returncode == 0:
        return 0, "detached via sudo -n\n" + \
                  (r2.stdout + r2.stderr).decode("utf-8", "replace")

    # 3) password file (used once, then deleted so the secret never lingers in
    #    the Moonraker-served config dir).
    pf = os.path.expanduser(pass_file)
    if not os.path.exists(pf):
        u1 = (r1.stderr or r1.stdout).decode("utf-8", "replace").strip()
        u2 = (r2.stderr or r2.stdout).decode("utf-8", "replace").strip()
        return 2, ("could not detach without root:\n"
                   f"  systemd-run --user failed: {u1}\n"
                   f"  sudo -n systemd-run failed: {u2}\n"
                   f"  last resort -- create a sudo password file (chmod 600):\n"
                   f"    printf '%s' 'YOURPASSWORD' > {pf}")
    pw = open(pf).read().rstrip("\n") + "\n"
    r3 = subprocess.run(["sudo", "-S", "-p", ""] + unit,
                        input=pw.encode(), capture_output=True)
    try:
        os.remove(pf)
    except Exception:
        pass
    return r3.returncode, (r3.stdout + r3.stderr).decode("utf-8", "replace")


def _lingering() -> bool:
    """Whether this user's systemd manager outlives login sessions."""
    import subprocess
    try:
        r = subprocess.run(["loginctl", "show-user", str(os.getuid()),
                            "-p", "Linger"], capture_output=True, timeout=5)
    except Exception:
        return False
    return r.stdout.decode("utf-8", "replace").strip().lower() == "linger=yes"


def wait_ready(moon, timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if moon.state() == "ready":
            return True
        time.sleep(3)
    return moon.state() == "ready"


# ── config auto-detection (self-locating from this file's directory) ──────────
def config_root(start: str) -> str:
    """Walk up from `start` to the dir holding printer.cfg (the config root)."""
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
                        # Drop a Klipper inline comment (# or ;) and take the
                        # first token -- serial_port/tcp_key never contain spaces,
                        # and "tcp://host:8888   # note" must not carry the note
                        # into port parsing.
                        v = v.split("#", 1)[0].split(";", 1)[0].strip()
                        v = v.split()[0] if v.split() else ""
                        if v:
                            return v
            except Exception:
                pass
    return None


def _clean_key(v):
    """A tcp_key as Klipper publishes it, or None for no key. A section that
    AFC_BridgeBox builds without one carries the string 'None'."""
    s = str(v).strip() if v is not None else ""
    return None if s.lower() in ("", "none") else s


def bridge_keys(settings):
    """{serial_port: tcp_key or None} for every Bambu bridge in the live
    config, read from [AFC_BridgeBox ...] and [AFC_BambuAMS ...] sections. The
    key on an [AFC_BridgeBox] section itself wins over one copied into the
    unit sections it builds."""
    found, master = {}, {}
    for sec, opts in (settings or {}).items():
        if not sec.startswith(("afc_bridgebox", "afc_bambuams")):
            continue
        opts = opts or {}
        port = str(opts.get("serial_port") or "").strip()
        if not port:
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
    """(serial_port, tcp_key) of the one Bambu bridge in the live config.

    Returns (None, None) when there is none, and raises ValueError when
    sections name different bridges, since the flash must not guess which one
    to use.
    """
    found = bridge_keys(settings)
    if len(found) > 1:
        raise ValueError("more than one bridge in the config ("
                         + ", ".join(sorted(found)) + "); name the one to "
                         "use with TARGET=<serial_port> on the command")
    if not found:
        return None, None
    return next(iter(found.items()))


def unit_rows(moon):
    """Every AFC_BambuAMS unit Klipper knows: name, online, model, chain index,
    bridge fw. A unit whose status cannot be read raises: it could be the
    second unit online, so it must stop the count, not drop out of it."""
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
                     "fw": st.get("bridge_fw")})
    return rows


def pick_target(rows, model, ams_id, state=None):
    """(unit name, None, resuming) for the unit to flash, or (None, why, False).

    Exactly one AMS may be online, whatever its model, and it must be the model
    this command is for and sit at the bus address the image is for (`ams_id`;
    boxed units share device 0x0700 and take their id from the order they
    enrolled in). A unit whose AMS 1 / AMS 2 generation is not confirmed yet
    ("boxed") passes here: step 5 asks the unit itself.

    With no unit online, `state` (the record a previous run left when it sent
    a unit into its loader) resumes that unit, if it was this model. A unit
    in its loader does not answer the bridge's polls, so it never shows
    online, and without this a stopped update could not be finished.
    """
    name = MODELS[model]["name"]
    on = [r for r in rows if r["online"]]
    if not on and state:
        was = state.get("model")
        if was != model:
            was_name = MODELS.get(was, {}).get("name", str(was))
            return None, (f"a unit is waiting in its bootloader from an "
                          f"unfinished {was_name} update ({state.get('unit')}, "
                          f"{state.get('when')}). Finish it with the "
                          f"{was_name} command."), False
        if state.get("ams_id") != ams_id:
            return None, ("the unit waiting in its bootloader was sent there "
                          "for a different bus address than this image; run "
                          "the command with the artifact it was started "
                          "with"), False
        return str(state.get("unit") or "?"), None, True
    if len(on) != 1:
        names = ", ".join(r["name"] for r in on) or "none"
        return None, (f"need exactly one AMS on the bus, found {len(on)} "
                      f"online ({names}). Unplug the others and retry."), False
    r = on[0]
    if r["model"] != model and not (r["model"] == "boxed"
                                    and model in ("ams1", "ams2")):
        return None, (f"{r['name']} is configured as ams_model "
                      f"'{r['model'] or '?'}', but this command is for the "
                      f"{name} ({model}). Use that model's command, or fix "
                      f"ams_model: if the config is wrong."), False
    if r["index"] is not None and mc_id_for_index(r["index"]) != ams_id:
        return None, (f"{r['name']} is enrolled at chain index {r['index']} "
                      f"(AMS id 0x{mc_id_for_index(r['index']):02x}), but the "
                      f"image is addressed to AMS id 0x{ams_id:02x}. Power the "
                      f"bridge off and on with only this unit connected so "
                      f"it enrolls first, restart Klipper, and retry."), False
    return r["name"], None, False


def read_state(path):
    """The record of a unit a previous run left in its loader, or None."""
    try:
        with open(path) as f:
            d = json.load(f)
        return d if isinstance(d, dict) and d.get("model") else None
    except (OSError, ValueError):
        return None


def write_state(path, **d) -> bool:
    """Record the unit about to be sent into its loader; False if it could
    not be written, in which case it must not be sent there."""
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


def bridge_fw(rows):
    for r in rows:
        v = r.get("fw")
        if v:
            m = re.search(r"(\d+)\.(\d+)", str(v))
            if m:
                return int(m.group(1)) * 100 + int(m.group(2))
    return -1


# ── main ──────────────────────────────────────────────────────────────────────
def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default="",
                    help="optional log label for the unit (e.g. its sticker serial); "
                    "not validated -- defaults to the online unit name")
    ap.add_argument("--mode", default="preflight", choices=["check", "preflight", "go"])
    ap.add_argument("--model", default="ams2", choices=sorted(MODELS),
                    help="which AMS this run is for; picks the artifact and is "
                    "checked against the online unit (default: ams2)")
    ap.add_argument("--cfg-dir", default=None,
                    help="Klipper config root (default: auto-detected from this "
                    "file's location)")
    ap.add_argument("--target", default=None,
                    help="bridge tcp://host:port or USB serial path "
                    "(default: serial_port from the config)")
    ap.add_argument("--key", default=None, help="link key (default: tcp_key from config)")
    ap.add_argument("--artifact", default=None,
                    help="firmware artifact json (default: the model's own file "
                    "beside this script, e.g. ht_artifact.json)")
    ap.add_argument("--units", default=None,
                    help=argparse.SUPPRESS)   # retired: every unit is counted
    ap.add_argument("--sudo-pass-file", default=os.path.join(here, "ams_sudo.pw"),
                    help="file holding the sudo password, used once to relaunch "
                    "detached and then DELETED automatically (default: ams_sudo.pw "
                    "beside this file)")
    ap.add_argument("--detached", action="store_true",
                    help=argparse.SUPPRESS)   # internal: set on the relaunched copy
    a = ap.parse_args()
    if not a.cfg_dir:
        a.cfg_dir = config_root(here)

    # Tee all output to printer_data/logs/ams_flash.log -- when launched detached
    # (systemd-run) there is no console, and this log is Moonraker-served.
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

    moon = Moon()
    model = MODELS[a.model]
    state_path = os.path.join(here, STATE_NAME)
    if a.units:
        print("note: --units is no longer used; every unit Klipper has is "
              "counted, since any of them could be the second unit on the bus")
    target, cfg_key = a.target, None
    settings = None
    try:
        settings = moon.settings()
    except Exception:
        pass
    if target:
        cfg_key = (bridge_keys(settings).get(target) if settings is not None
                   else None) or find_in_cfg(a.cfg_dir, "tcp_key")
    else:
        # The live config first: it names the bridge Klipper actually uses. The
        # file scan is the fallback for a Klipper that is not up.
        try:
            target, cfg_key = bridge_from_settings(settings)
        except ValueError as e:
            print(f"=== AMS update: {e} ==="); return 2
        if not target:
            target = find_in_cfg(a.cfg_dir, "serial_port")
            cfg_key = _clean_key(find_in_cfg(a.cfg_dir, "tcp_key"))
    key = a.key if a.key is not None else (cfg_key or "")
    if not str(target or "").startswith("tcp://"):
        key = ""        # the link key is a network feature; USB has no challenge
    # ══ A BARE FILENAME RESOLVES BESIDE THIS SCRIPT. ══
    #
    # So the gcode macro can say `--artifact ht_artifact.json` instead of
    # carrying an absolute path that every recipient has to edit for their own
    # username. The script already knows where it lives; the caller should not
    # have to. An absolute path still wins, and a path with a directory in it
    # (./x, ../x) is left alone so it stays relative to the CWD as before.
    if a.artifact and not os.path.isabs(a.artifact) and os.sep not in a.artifact:
        artifact = os.path.join(here, a.artifact)
    elif a.artifact:
        artifact = a.artifact
    else:
        artifact = os.path.join(here, model["artifact"])
        # The AMS 2 file was ams_artifact.json before there was one per model.
        legacy = model.get("legacy_artifact")
        if legacy and not os.path.exists(artifact) \
                and os.path.exists(os.path.join(here, legacy)):
            artifact = os.path.join(here, legacy)

    print(f"=== AMS update: {model['name']} ({a.model}) "
          f"label={a.label or '(auto)'} mode={a.mode.upper()} bridge={target} ===")
    if not target:
        print("no bridge found in the config (serial_port:); name it with "
              "TARGET=<serial_port> on the command"); return 2
    if target.startswith("tcp://"):
        # Parse now: the link is first opened at step 5, with Klipper stopped.
        _h, _s, _p = target[6:].partition(":")
        try:
            if not _h or not (0 < int(_p or 8888) < 65536):
                raise ValueError
        except ValueError:
            print(f"bridge target {target!r} is not tcp://host:port"); return 2
    elif not os.path.exists(target):
        print(f"bridge port {target!r} does not exist; check serial_port: or "
              f"name the port with TARGET="); return 2

    # Detach: preflight/go must stop klipper, which would kill this process if it
    # stayed a child of the klipper service. Relaunch into our own systemd unit
    # (once). `check` is read-only and never stops klipper, so it runs inline.
    if a.mode != "check" and not a.detached:
        rc, out = relaunch_detached(a.sudo_pass_file)
        if rc != 0:
            print("could not launch detached:\n" + out); return rc
        print("launched detached; progress in logs/ams_flash.log"); return 0

    if a.mode != "check":
        # One update at a time: a second press of the command must not open
        # the same bridge and interleave frames with the first.
        import fcntl
        import signal
        lock_path = os.path.join(os.path.dirname(a.cfg_dir.rstrip("/")),
                                 "logs", "ams_flash.lock")
        try:
            os.makedirs(os.path.dirname(lock_path), exist_ok=True)
            lock = open(lock_path, "w")
        except OSError as e:
            print(f"note: no lock file ({e}); not guarding against a second "
                  f"run")
            lock = None
        if lock is not None:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                print("ABORT: another AMS update is already running; let it "
                      "finish (logs/ams_flash.log)."); return 1

        # A stop of this unit (systemctl, shutdown, logout) must still run the
        # finally blocks below, which restart Klipper.
        def _stopped(signum, _frame):
            raise SystemExit(f"stopped by signal {signum}")
        signal.signal(signal.SIGTERM, _stopped)
        signal.signal(signal.SIGHUP, _stopped)

    # 1 not printing
    print("[1] printer not printing")
    try:
        st = moon.q("print_stats").get("state")
    except Exception as e:
        print(f"    ABORT: cannot read print state ({e})"); return 1
    print(f"    state = {st}")
    if st in ("printing", "paused"):
        print("    ABORT: a print is active."); return 1

    # 2 artifact
    print("[2] artifact verifies")
    try:
        art = load_artifact(artifact)
        print(f"    OK: header + {len(art['blocks'])} blocks")
    except Exception as e:
        print(f"    ABORT: artifact {artifact}: {e}"); return 1
    # ══ THE BLOCKS MUST ADD UP TO THE DECLARED IMAGE -- NOT COUNT 168. ══
    #
    # `len(blocks) != 168` was the AMS 2's block count and nothing else. It
    # refused a perfectly good AMS HT artifact (157 blocks) while reporting it
    # as a verification failure, and it would equally have waved through a
    # 168-block artifact missing half its bytes, because a count says nothing
    # about content.
    #
    # The artifact describes itself instead: the header frame carries the total
    # image size and every data block carries its own firmware-byte count, so
    # the blocks must sum to the declared image less the BIMH container header.
    # True on both models, independent of size, version and block count.
    total, summed = artifact_image_bytes(art)
    print(f"    image: header declares {total} bytes, blocks carry {summed} "
          f"+ {total - summed} container header")
    if total - summed != BIMH_HEADER_LEN:
        print(f"    ABORT: artifact INCOMPLETE -- blocks do not add up to the "
              f"declared image ({total} - {summed} != {BIMH_HEADER_LEN})")
        return 1
    dev, ams_id = loader_target_of(art)
    print(f"    {artifact_image_name(art) or '(unnamed image)'}")
    print(f"    addressed to device 0x{dev:04x}, AMS id 0x{ams_id:02x}")
    why = artifact_mismatch(art, a.model)
    if why:
        print(f"    ABORT: {why}"); return 1

    # 3 bridge fw
    print(f"[3] bridge fw >= {TARGET_FW}")
    try:
        rows = unit_rows(moon)
    except Exception as e:
        print(f"    ABORT: cannot read the AMS units ({e})"); return 1
    fw = bridge_fw(rows)
    print(f"    bridge_fw = {fw if fw > 0 else 'unknown'}")
    if fw < TARGET_FW:
        print(f"    ABORT: bridge is below {TARGET_FW}. Update the bridge first "
              f"(one-time), then retry."); return 1

    # 4 one unit online (Klipper's view)
    print(f"[4] exactly one AMS online, and it is an {model['name']}")
    for r in rows:
        print(f"    {r['name']}: {'online' if r['online'] else 'offline'}"
              f"  ams_model={r['model'] or '?'}  index={r['index']}")
    state = read_state(state_path)
    unit, why, resuming = pick_target(rows, a.model, ams_id, state)
    if unit is None:
        print(f"    ABORT: {why}"); return 1
    if resuming:
        print(f"    resuming {unit}: an earlier run ({state.get('when')}) left "
              f"it waiting in its bootloader")
    label = a.label or unit
    print(f"    target unit: {unit}  (label: {label})")
    row = next((r for r in rows if r["name"] == unit), None)

    if a.mode == "check":
        print("\n==> READ-ONLY CHECKS PASSED. Nothing was changed. Preflight "
              "and go also check the bus itself, with Klipper stopped.")
        return 0

    # 5 the bus itself (klipper down: a bridge takes one client at a time)
    print("[5] the bus: one unit answering, at the image's address, and it "
          "is this model")
    rc = 1
    link = None
    try:
        if not klipper(moon, "stop"):
            print("    ABORT: could not stop klipper to free the board socket.")
            return 1
        time.sleep(3)
        try:
            link = Link(target)
            if not authenticate(link, key):
                print("    ABORT: link auth failed"); return 1
            reset_bridge_mode(link)
            wire, drying = wire_status(link)
            print("    answering on the bus: "
                  + (", ".join(f"index {n} (AMS id 0x{mc_id_for_index(n):02x})"
                               for n in wire) or "none"))
            if drying:
                # A boxed dry deafens the bridge's receiver: answers go
                # unheard and online flags are held, so nothing here can be
                # read while one runs.
                print("    ABORT: a dry cycle is running. Stop it and "
                      "retry."); return 1
            if not resuming and not wire and state \
                    and state.get("model") == a.model \
                    and state.get("ams_id") == ams_id:
                # Klipper's online flag was stale; the bus is the authority.
                print(f"    nothing answers, but an earlier run "
                      f"({state.get('when')}) left {state.get('unit')} in its "
                      f"bootloader: resuming it")
                resuming = True
                unit = str(state.get("unit") or unit)
            if resuming:
                if wire:
                    print("    ABORT: a unit is answering on the bus, but the "
                          "unit being resumed is waiting in its bootloader. "
                          "Unplug every other unit and retry."); return 1
                if loader_on_bus(link, exclude=(dev, ams_id)):
                    print("    ABORT: another unit on the bus is sitting in "
                          "its bootloader. Unplug it and retry."); return 1
            else:
                if len(wire) != 1:
                    print(f"    ABORT: need exactly one AMS on the bus, found "
                          f"{len(wire)} answering. Unplug the others (the "
                          f"bridge counts units Klipper has no section for "
                          f"too) and retry."); return 1
                n = wire[0]
                if mc_id_for_index(n) != ams_id:
                    print(f"    ABORT: the unit is enrolled at index {n} (AMS "
                          f"id 0x{mc_id_for_index(n):02x}), but the image is "
                          f"addressed to AMS id 0x{ams_id:02x}. Power the "
                          f"bridge off and on with only this unit connected "
                          f"so it enrolls first, restart Klipper, and retry.")
                    return 1
                if row and row["index"] is not None and row["index"] != n:
                    print(f"    ABORT: Klipper has {unit} at index "
                          f"{row['index']}, but the unit answering is at index "
                          f"{n}. Restart Klipper so they agree, and retry.")
                    return 1
                # A unit answering the bridge's polls is running its app, so
                # a record of one left in its loader at this address is stale
                # (the unit was power-cycled out of it).
                if state and state.get("ams_id") == ams_id:
                    clear_state(state_path)
                    state = None
                if a.model in ("ams1", "ams2"):
                    # An AMS 1 and an AMS 2 take the same address and neither
                    # image can tell. The unit can: only an AMS 2 answers the
                    # generation query, and both answer the log drain, which
                    # proves the silence is a listening AMS 1. Asked now, not
                    # remembered, so a swap since the bridge booted cannot
                    # fool it.
                    gen, ctl = probe_generation(link, ams_id)
                    print(f"    generation query: answered {gen} of "
                          f"{GEN_ASKS}; log drain: answered {ctl} of "
                          f"{GEN_ASKS}")
                    if ctl < GEN_CTL_MIN:
                        print("    ABORT: the unit is not answering the bus "
                              "reliably, so its generation cannot be told. "
                              "Retry; if it keeps happening, check the bus "
                              "wiring."); return 1
                    if gen >= GEN_AMS2_MIN:
                        is_model = "ams2"
                    elif gen == 0:
                        is_model = "ams1"
                    else:
                        print("    ABORT: too few answers to tell an AMS 1 "
                              "from an AMS 2. Retry."); return 1
                    if is_model != a.model:
                        print(f"    ABORT: the unit answers as an "
                              f"{MODELS[is_model]['name']}. Use the "
                              f"{MODELS[is_model]['name']} command."); return 1
                if loader_on_bus(link):
                    print("    ABORT: a unit on the bus is sitting in its "
                          "bootloader, apart from the one being updated. That "
                          "is a second unit on the bus: unplug it, or finish "
                          "its own update first."); return 1
                # The sniff stopped the polling; let it run again and look
                # once more, right before the unit is sent into its loader.
                settle(link, ONLINE_SETTLE_S)
                if wire_online(link) != [n]:
                    print("    ABORT: the units answering changed during the "
                          "checks; retry."); return 1
            print("    OK")

            # Recorded before the unit is sent into its loader, so a run that
            # stops anywhere after this can be resumed by the same command.
            if not write_state(state_path, model=a.model, unit=unit,
                               ams_id=ams_id,
                               artifact=os.path.basename(artifact),
                               when=time.strftime("%Y-%m-%d %H:%M")):
                print("    ABORT: without that record a stopped update could "
                      "not be resumed, so the unit is not sent into its "
                      "loader."); return 1

            # 6 probe
            print("[6] cmd1 enters the loader (non-destructive)")
            if not probe_cmd1(link, art):
                print("    NO-GO: unit did not enter its loader on cmd1")
                if resuming:
                    print("    (nothing answered from a bootloader either; if "
                          "the unit was power-cycled since, plug it in, wait "
                          "until it shows online, and retry)")
                return 1
            if a.mode == "preflight":
                print("\n==> PREFLIGHT PASSED. Everything up to the erase is "
                      "green; nothing on the AMS changed. The unit is now "
                      "waiting in its bootloader: run MODE=go to update it, "
                      "or power-cycle the AMS to return it to normal.")
                rc = 0
                return 0

            # 7 flash
            print(f"[7] FLASH: header (erase) + {len(art['blocks'])} blocks")

            def _reconnect():
                # A bridge serves one client, and a new one pushes the old one
                # off. If Klipper came back up it holds the link now; taking
                # it back would only push Klipper off in turn.
                if moon.state() != "?":
                    raise IOError("Klipper was started during the update and "
                                  "now holds the bridge link")
                link.reopen()
                if not authenticate(link, key):
                    raise IOError("link auth failed after reopening")
            rc = do_flash(link, art, reconnect=_reconnect)
            if rc == BADIMG:
                print("    FLASH stopped: the unit refused this image. Loader "
                      "intact; get the right artifact before running MODE=go "
                      "again.")
                return 1
            if rc != 0:
                print("    FLASH incomplete; loader intact -- run MODE=go "
                      "again.")
                return rc
            clear_state(state_path)
        except OSError as e:
            print(f"    ABORT: the bridge link failed ({e})"); return 1
        finally:
            if link is not None:
                if not link.dead:
                    release_bridge(link)
                link.close()
    finally:
        if not klipper(moon, "start"):
            print("    WARNING: could not restart Klipper. Start it yourself "
                  "(Mainsail/Fluidd service menu, or: sudo systemctl start "
                  "klipper).")
        wait_ready(moon, 90)

    # 8 verify online
    print("[8] unit reboots and comes back online")
    t0 = time.time()
    while time.time() - t0 < 60:
        try:
            back = any(r["online"] for r in unit_rows(moon) if r["name"] == unit)
        except Exception:
            back = False
        if back:
            print(f"    {unit} ONLINE on the new firmware.")
            print("\n==> DONE. Start the dryer and load a tray to confirm.")
            return 0
        time.sleep(5)
    print("    not seen online within 60s -- check AFC_BAMBU_UIDS / status.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
