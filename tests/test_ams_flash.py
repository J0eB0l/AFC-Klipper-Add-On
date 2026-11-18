"""Unit tests for firmwares/Bambu_AMS/ams_flash.py.

Every image, frame, unit, port and address here is made up: the images are synthetic BIMH
containers named the way Bambu names its own, and no test reaches the network.
"""

from __future__ import annotations

import argparse
import array
import binascii
import builtins
import contextlib
import fcntl
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import socket
import struct
import sys
from types import ModuleType
from typing import Any, Callable, Dict, Iterator, List, Optional, Tuple
import urllib.request
import zipfile

import pytest


AMS_FLASH_DIR = Path(__file__).resolve().parents[1] / "firmwares" / "Bambu_AMS"


def _load_ams_flash() -> ModuleType:
    """Load the updater from its file; it is a script, not an importable package."""
    spec = importlib.util.spec_from_file_location("ams_flash", AMS_FLASH_DIR / "ams_flash.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ams_flash = _load_ams_flash()


# Made-up images, named the way Bambu names its own.
AMS2_FILE = "n3f_rev5-firmware-v01.02.03.04-20000101000000_product.bin.sig"
HT_FILE = "n3s_rev5-firmware-v01.02.03.05-20000101000000_product.bin.sig"
AMS1_FILE = "ams_rev8-firmware-v01.02.03.06-20000101000000_product.bin.sig"
PACKAGE_URL = "https://example.invalid/offline/package.zip"


def _payload(size: int, seed: int) -> bytes:
    """Made-up image bytes."""
    return bytes((i * 7 + seed) & 0xFF for i in range(size))


def _bimh(name: str, payload: bytes, hlen: int = 0x80) -> bytes:
    """
    A synthetic Bambu image: a BIMH container header `hlen` bytes long naming `name`, then
    `payload`.
    """
    total = hlen + len(payload)
    head = bytearray(hlen)
    head[0:4] = b"BIMH"
    head[4:8] = (1).to_bytes(4, "little")                    # container format
    head[8:12] = total.to_bytes(4, "little")                 # the whole file
    head[0x20:0x24] = hlen.to_bytes(4, "little")             # the container header
    head[0x28:0x2C] = (total - hlen).to_bytes(4, "little")   # what follows it
    head[0x30:0x30 + len(name)] = name.encode()              # NUL-padded up to 0x80
    return bytes(head) + payload


AMS2_IMAGE = _bimh(AMS2_FILE, _payload(2500, 1))
HT_IMAGE = _bimh(HT_FILE, _payload(2500, 2))
AMS1_IMAGE = _bimh(AMS1_FILE, _payload(2500, 3))
#: The MD5s of the three images, worked out once apart from the code under test.
AMS2_MD5 = "95e1542519ef10672477e9080902b67c"
HT_MD5 = "1e70101dd0b5fcbbeae768bdf821c853"
AMS1_MD5 = "67054c846cea28d264bdfad7459e9222"


def _crc8(data: bytes) -> int:
    """CRC-8: poly 0x39, init 0x66, MSB first, no reflection and no final XOR."""
    reg = 0x66
    for byte in data:
        reg ^= byte
        for _ in range(8):
            reg = ((reg << 1) ^ 0x39) & 0xFF if reg & 0x80 else (reg << 1) & 0xFF
    return reg


def _crc16(data: bytes) -> int:
    """CRC-16: poly 0x1021 from 0x913D, which is binascii's crc_hqx."""
    return binascii.crc_hqx(data, 0x913D)


def _frame(body: bytes, counter: int = 0) -> bytes:
    """A whole bus frame around `body`: its length, a CRC-8 over the head, a CRC-16 over all."""
    head = bytes((0x3D, 0x00, counter & 0xFF, counter >> 8))
    head += (7 + len(body) + 2).to_bytes(2, "little")
    frame = head + bytes((_crc8(head),)) + body
    return frame + _crc16(frame).to_bytes(2, "little")


def _loader_frame(dev: int, unit_id: int, source: Optional[int] = None) -> bytes:
    """
    A unit's loader answering cmd1: op 06 01 from `source` (the unit's own device unless
    given) to the printer, carrying the unit's id where cmd1 carries it.
    """
    body = bytearray(13)
    body[0:2] = (0x0900).to_bytes(2, "little")
    body[2:4] = (dev if source is None else source).to_bytes(2, "little")
    body[4:7] = b"\x06\x01\x01"
    body[7] = dev >> 8
    body[12] = unit_id
    return _frame(bytes(body))


def _version_frame(dev: int, unit_id: int, version: str, code: str,
                   op: bytes = b"\x03\x01") -> bytes:
    """A unit's answer to the version query: the version lowest byte first, model code, its id."""
    body = bytearray(27)
    body[0:2] = (0x0900).to_bytes(2, "little")
    body[2:4] = dev.to_bytes(2, "little")
    body[4:6] = op
    body[6:10] = bytes(int(part) for part in reversed(version.split(".")))
    body[10:10 + len(code)] = code.encode()
    body[26] = unit_id
    return _frame(bytes(body))


def _version_query(dev: int, unit_id: int, counter: int) -> bytes:
    """The printer's version query to a unit, laid out by hand."""
    head = bytes((0x3D, 0x05, counter & 0xFF, counter >> 8, 16, 0))
    frame = (head + bytes((_crc8(head),)) + dev.to_bytes(2, "little") + b"\x00\x09"
             + bytes((0x03, 0x01, unit_id)))
    return frame + _crc16(frame).to_bytes(2, "little")


def _package(members: Dict[str, bytes]) -> bytes:
    """A made-up offline update package (.zip) holding `members`, stored as they are."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as z:
        for name, data in members.items():
            z.writestr(name, data)
    return buf.getvalue()


def _entry(**kw: Any) -> Dict[str, Any]:
    """An ams_firmware.json entry for the made-up AMS 2 Pro image, with `kw` over it."""
    entry = {"board": "n3f_rev5", "version": "01.02.03.04", "file": AMS2_FILE,
             "url": PACKAGE_URL, "md5": AMS2_MD5}
    entry.update(kw)
    return entry


def _catalog(here: Path, *entries: Any) -> None:
    """Write an ams_firmware.json listing `entries` into `here`."""
    text = json.dumps({"about": "made up", "images": list(entries)})
    (here / "ams_firmware.json").write_text(text)


class _Response:
    """What urlopen returns: a context manager with a status, headers and a body."""

    def __init__(self, body: bytes, status: Optional[int] = 200,
                 headers: Optional[Dict[str, str]] = None) -> None:
        self.body = body
        if status is not None:
            self.status = status
        self.headers = headers or {}

    def __enter__(self) -> _Response:
        return self

    def __exit__(self, *exc: Any) -> None:
        return None

    def read(self) -> bytes:
        return self.body


class _Server:
    """
    A web server holding `files` (URL -> bytes) in memory, in place of urlopen. It answers a
    Range request with 206 and a Content-Range header, any other with 200 and the whole file.
    """

    def __init__(self, files: Dict[str, bytes]) -> None:
        self.files = files
        #: (URL, Range, User-Agent, timeout) of each request.
        self.asked: List[Tuple[str, Optional[str], Optional[str], float]] = []
        self.served = 0

    def __call__(self, req: urllib.request.Request, timeout: float) -> _Response:
        wanted = req.get_header("Range")
        self.asked.append((req.full_url, wanted, req.get_header("User-agent"), timeout))
        data = self.files[req.full_url]
        if wanted is None:
            self.served += len(data)
            return _Response(data)
        start, end = (int(x) for x in wanted[len("bytes="):].split("-"))
        part = data[start:end + 1]
        self.served += len(part)
        return _Response(part, 206, {"Content-Range": f"bytes {start}-{end}/{len(data)}"})


@pytest.fixture(autouse=True)
def _offline(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    """No test reaches the network: urlopen and socket connects are refused, and noticed."""
    tried: List[str] = []

    def refuse(target: Any, *args: Any, **kwargs: Any) -> Any:
        tried.append(str(getattr(target, "full_url", target)))
        raise OSError("the network is off in these tests")

    monkeypatch.setattr(urllib.request, "urlopen", refuse)
    monkeypatch.setattr(socket, "create_connection", refuse)
    yield
    assert tried == []


@contextlib.contextmanager
def _signals_kept() -> Iterator[None]:
    """Put back the SIGTERM and SIGHUP handlers a run installs."""
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP)}
    try:
        yield
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


class _AmsFlashScriptedLink:
    """A bridge link that answers each command from a script."""

    def __init__(self, answer: Callable[[Dict[str, Any]], List[str]]) -> None:
        self.answer = answer
        self.sent: List[Dict[str, Any]] = []
        self.queue: List[str] = []
        self.dead = False

    def send(self, obj: Dict[str, Any]) -> None:
        self.sent.append(obj)
        self.queue.extend(self.answer(obj))

    def lines(self) -> Iterator[Optional[str]]:
        while True:
            yield self.queue.pop(0) if self.queue else None


class _AmsFlashClock:
    """
    The updater's `time`: time() steps on every read, sleep() moves the clock on and is kept,
    and strftime() reads a fixed, made-up wall clock.
    """

    WALL = {"%Y-%m-%d %H:%M:%S": "2000-01-02 03:04:05", "%Y-%m-%d %H:%M": "2000-01-02 03:04"}

    def __init__(self, step: float = 0.25,
                 on_sleep: Optional[Callable[[float], None]] = None) -> None:
        self.now = 1000.0
        self.step = step
        self.slept: List[float] = []
        self.on_sleep = on_sleep

    def time(self) -> float:
        self.now += self.step
        return self.now

    def sleep(self, seconds: float) -> None:
        self.slept.append(seconds)
        self.now += seconds
        if self.on_sleep is not None:
            self.on_sleep(seconds)

    def strftime(self, fmt: str) -> str:
        return self.WALL[fmt]


class _AmsFlashPty:
    """A pseudo-terminal standing in for the bridge's USB serial port."""

    def __init__(self) -> None:
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        self._open = [self.master, slave]

    def hang_up(self) -> None:
        """Close both ends, as a bridge pulled off the USB bus does."""
        for fd in self._open:
            os.close(fd)
        self._open = []


@pytest.fixture
def ams_flash_pty() -> Iterator[_AmsFlashPty]:
    port = _AmsFlashPty()
    yield port
    port.hang_up()


def _tiny_art(dev: int, unit_id: int, blocks: int = 2) -> Dict[str, Any]:
    """A short made-up update for do_flash: a header and `blocks` data blocks for one unit."""
    def body(cmd: int, size: int) -> bytes:
        frame = bytearray(size)
        frame[0:2] = dev.to_bytes(2, "little")
        frame[2:4] = b"\x00\x09"                         # from the printer
        frame[4:7] = bytes((0x06, 0x01, cmd))
        frame[7] = dev >> 8
        frame[12] = unit_id
        return bytes(frame)

    return {"header": body(0x02, 140), "blocks": [body(0x03, 40)] * blocks}


def _raised(value: Any) -> Any:
    """`value`, or raise it when it is an exception."""
    if isinstance(value, BaseException):
        raise value
    return value


def _klipper_unit(port: Optional[str], **kw: Any) -> Dict[str, Any]:
    """An AFC_BambuAMS unit's status as Klipper publishes it, with `kw` over it."""
    status = {"bridge_online": True, "ams_model": "ams2", "ams_index": 0, "bridge_port": port,
              "bridge_fw": "AFC-2.81", "fw_version": "01.02.03.00", "fw_model": "N3F05",
              "drying": False}
    status.update(kw)
    return status


class _Moon:
    """
    Moonraker as update() and main() use it. An exception in place of a value is raised when
    it is read, and each g-code line is kept; `answers` scripts what a line answers.
    """

    def __init__(self, units: Dict[str, Any]) -> None:
        self.units = units
        self.print_state: Any = "standby"
        self.config: Any = {}
        self.klippy = "ready"
        self.scripts: List[str] = []
        self.answers: Dict[str, List[bool]] = {}
        self.state_calls = 0
        self.on_print_stats: Optional[Callable[[], None]] = None

    def q(self, obj: str) -> Any:
        if obj == "print_stats":
            if self.on_print_stats is not None:
                self.on_print_stats()
            return {"state": _raised(self.print_state)}
        return _raised(self.units[obj.split(" ", 1)[1]])

    def objects(self) -> List[str]:
        return [f"AFC_BambuAMS {name}" for name in self.units] + ["toolhead"]

    def settings(self) -> Any:
        return _raised(self.config)

    def gcode(self, script: str) -> bool:
        self.scripts.append(script)
        answers = self.answers.get(script) or []
        return answers.pop(0) if answers else True

    def state(self) -> str:
        self.state_calls += 1
        return self.klippy


class _Unit:
    """One AMS on the made-up bus."""

    def __init__(self, n: int, unit_id: int, dev: int, version: str, code: str,
                 mode: str = "app") -> None:
        self.n, self.unit_id, self.dev = n, unit_id, dev
        self.version, self.code = version, code
        self.mode = mode            # "app", or "loader" while it sits in its bootloader
        self.online = True          # the bridge lists it while it runs its app
        self.answers = True         # it answers the version query in its app
        self.enters = True          # cmd1 sends it into its loader
        self.drops = False          # it goes quiet when another unit enters its loader


class _Bus:
    """
    The bridge link update() borrows, in front of made-up units. It answers fwreplay, sniff,
    status and raw as the bridge firmware does, and logs each request in words.
    """

    NO_CMD1 = "no cmd1 was sent"

    def __init__(self, units: List[_Unit], state_path: Path) -> None:
        self.units = units
        self.state_path = state_path
        self.dryrem = 0
        self.mute = False
        self.log: List[str] = []
        self.queue: List[str] = []
        self.dead = False
        #: The update's record as it stood when the first cmd1 went out.
        self.record_at_cmd1: Any = self.NO_CMD1

    def send(self, obj: Dict[str, Any]) -> None:
        cmd = obj["cmd"]
        tx = bytes.fromhex(obj["hex"]) if cmd == "raw" else b""
        if cmd in ("fwreplay", "sniff"):
            self.log.append(f"{cmd} {'on' if obj['on'] else 'off'}")
        elif cmd == "raw" and tx[11:13] == b"\x03\x01":
            self.log.append(f"version? 0x{tx[13]:02x}")
        elif cmd == "raw":
            self.log.append(f"cmd1 0x{tx[19]:02x}")
            if self.record_at_cmd1 == self.NO_CMD1:
                self.record_at_cmd1 = (json.loads(self.state_path.read_text())
                                       if self.state_path.is_file() else None)
        else:
            self.log.append(cmd)
        if self.mute:
            return
        if cmd == "fwreplay":
            self.queue.append(json.dumps({"evt": "fwreplay", "on": bool(obj["on"])}))
        elif cmd == "sniff":
            self.queue.append(json.dumps({"evt": "sniff_mode", "on": bool(obj["on"])}))
            for unit in self.units:
                if obj["on"] and unit.mode == "loader":
                    hexed = _loader_frame(unit.dev, unit.unit_id).hex().upper()
                    self.queue.append(json.dumps({"evt": "sniff", "us": 1, "hex": hexed}))
        elif cmd == "status":
            units = [{"n": u.n, "online": True, "dryrem": self.dryrem} for u in self.units
                     if u.mode == "app" and u.online]
            self.queue.append(json.dumps({"evt": "status", "units": units}))
        else:
            self.queue.append(json.dumps({"evt": "raw", "rx": self._heard(tx).hex().upper()}))

    def _heard(self, tx: bytes) -> bytes:
        """What the bridge hears after sending `tx`: the frame itself, then any answer."""
        rx = tx
        for unit in self.units:
            if (tx[11:13] == b"\x03\x01"
                and tx[13] == unit.unit_id
                and unit.mode == "app"
                and unit.answers):
                rx += _version_frame(unit.dev, unit.unit_id, unit.version, unit.code)
            elif (tx[11:13] == b"\x06\x01"
                  and tx[19] == unit.unit_id
                  and unit.enters):
                unit.mode = "loader"
                for other in self.units:
                    other.online = other.online and not other.drops
                rx += _loader_frame(unit.dev, unit.unit_id)
        return rx

    def lines(self) -> Iterator[Optional[str]]:
        while True:
            yield self.queue.pop(0) if self.queue else None

    def reopen(self) -> None:
        self.log.append("reopen")
        self.dead = False

    def close(self) -> None:
        self.log.append("close")


class _Rig:
    """
    A printer for update() and main(): a config folder with the updater's folder in it, and
    made-up Moonraker, bridge link, do_flash and detached relaunch.
    """

    def __init__(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
                 capsys: pytest.CaptureFixture[str]) -> None:
        self.tmp_path = tmp_path
        self.root = tmp_path / "printer_data"
        self.cfg = self.root / "config"
        self.here = self.cfg / "Bambu_AMS"
        self.here.mkdir(parents=True)
        (self.cfg / "printer.cfg").write_text("")
        self.port = str(tmp_path / "bridge-port")
        Path(self.port).write_text("")
        self.state_path = self.here / "ams_flash_state.json"
        self.result_path = self.here / "ams_flash_result.json"
        self.log_path = self.root / "logs" / "ams_flash.log"
        (self.here / AMS2_FILE).write_bytes(AMS2_IMAGE)
        _catalog(self.here, _entry())
        self.moon = _Moon({"Bambu_AMS_1": _klipper_unit(self.port)})
        self.unit = _Unit(0, 0x00, 0x0700, "01.02.03.00", "N3F05")
        self.bus = _Bus([self.unit], self.state_path)
        self.clock = _AmsFlashClock()
        self.opened: List[str] = []
        self.open_errors: List[OSError] = []
        self.flashed: List[Dict[str, Any]] = []
        self.flash_rc = 0
        self.flash_hook: Optional[Callable[[Any, Callable[[], None]], int]] = None
        self.new_version = "01.02.03.04"
        self.relaunch: Tuple[int, str] = (0, "detached via systemd-run --user\n")
        self.relaunched: List[str] = []
        self.result: Dict[str, Any] = {}
        self.out = self.err = ""
        self.log: Optional[str] = None
        self._monkeypatch = monkeypatch
        self._capsys = capsys
        monkeypatch.setattr(ams_flash, "time", self.clock)
        monkeypatch.setattr(ams_flash, "Link", self._open)
        monkeypatch.setattr(ams_flash, "do_flash", self._flash)
        monkeypatch.setattr(ams_flash, "relaunch_detached", self._relaunch)

    def _open(self, port: str) -> _Bus:
        self.opened.append(port)
        if self.open_errors:
            raise self.open_errors.pop(0)
        self.bus.log.append("open")
        return self.bus

    def _flash(self, link: Any, art: Dict[str, Any], retries: int = 6,
               reconnect: Optional[Callable[[], None]] = None) -> int:
        self.bus.log.append("flash")
        self.flashed.append(art)
        if self.flash_hook is not None:
            return self.flash_hook(link, reconnect)
        for unit in self.bus.units:
            if self.flash_rc == 0 and unit.mode == "loader":
                unit.mode, unit.version = "app", self.new_version
        return self.flash_rc

    def _relaunch(self, pass_file: str) -> Tuple[int, str]:
        self.relaunched.append(pass_file)
        return self.relaunch

    def args(self, **kw: Any) -> argparse.Namespace:
        """The detached update of Bambu_AMS_1 to 01.02.03.04, with `kw` over it."""
        a = dict(unit="Bambu_AMS_1", version="01.02.03.04", file=None, board=None, mode="go",
                 cfg_dir=str(self.cfg), target=None, key=None,
                 sudo_pass_file=str(self.here / "ams_sudo.pw"), detached=True)
        a.update(kw)
        return argparse.Namespace(**a)

    def record(self, **kw: Any) -> Dict[str, Any]:
        """The record an update of Bambu_AMS_1 writes before cmd1, with `kw` over it."""
        rec = {"unit": "Bambu_AMS_1", "unit_id": 0, "board": "n3f_rev5",
               "version": "01.02.03.04", "file": str(self.here / AMS2_FILE),
               "when": "2000-01-02 03:04"}
        rec.update(kw)
        return rec

    def state(self) -> Optional[Dict[str, Any]]:
        """The record on disk, or None."""
        return json.loads(self.state_path.read_text()) if self.state_path.is_file() else None

    def saved_result(self) -> Optional[Dict[str, Any]]:
        """The result on disk, or None."""
        return json.loads(self.result_path.read_text()) if self.result_path.is_file() else None

    def update(self, a: argparse.Namespace) -> Tuple[str, Any]:
        """Run update() once, as main() would, and keep what it printed."""
        self.result = {"unit": a.unit or "", "ok": False, "from": "", "to": a.version or "",
                       "message": ""}
        with _signals_kept():
            try:
                outcome: Tuple[str, Any] = (
                    "rc", ams_flash.update(a, str(self.here), self.moon, self.result))
            except ams_flash.Abort as e:
                outcome = ("abort", str(e))
            except SystemExit as e:
                outcome = ("exit", str(e))
        self.out, self.err = self._capsys.readouterr()
        return outcome

    def main(self, *argv: str) -> Tuple[str, Any]:
        """Run main() once with `argv`, and keep what it printed and logged."""
        self._monkeypatch.setattr(sys, "argv", ["ams_flash.py", *argv])
        self._monkeypatch.setattr(ams_flash, "__file__", str(self.here / "ams_flash.py"))
        self._monkeypatch.setattr(ams_flash, "Moon", lambda: self.moon)
        logged = self.log_path.stat().st_size if self.log_path.is_file() else 0
        streams = sys.stdout, sys.stderr
        with _signals_kept():
            try:
                outcome: Tuple[str, Any] = ("rc", ams_flash.main())
            except SystemExit as e:
                outcome = ("exit", str(e))
            finally:
                sys.stdout, sys.stderr = streams     # main() tees both into its log
        self.out, self.err = self._capsys.readouterr()
        self.log = self.log_path.read_text()[logged:] if self.log_path.is_file() else None
        return outcome


@pytest.fixture
def rig(tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
        capsys: pytest.CaptureFixture[str]) -> _Rig:
    return _Rig(tmp_path, monkeypatch, capsys)


#: What Klipper is asked over a default update.
RESPOND = ('RESPOND MSG="AMS update: Bambu_AMS_1 to 01.02.03.04. The units on its bridge read '
           'offline until it is done."')
HOLD = "AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 HOLD=1800"
BACK = "AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 HOLD=0"
SHOW = "AFC_BAMBU_FIRMWARE SHOW=RESULT"
#: What the bus hears over a default update: the checks up to the record, then the flash.
CHECKED_BUS = ["open", "fwreplay off", "sniff off", "status", "version? 0x00", "sniff on",
               "sniff off", "status"]
FLASHED_BUS = ["cmd1 0x00", "flash", "status", "version? 0x00", "fwreplay off", "sniff off",
               "close"]
#: A live link handed back.
RELEASED = ["fwreplay off", "sniff off", "close"]


class _Said:
    """What update() prints over a default update, step by step."""

    START = "[1] printer not printing\n    state = standby\n"
    ROW = ("    Bambu_AMS_1: online  ams_model=ams2  index=0  bus id 0x00  firmware 01.02.03.00 "
           "(N3F05)\n")
    OFFLINE = "    Bambu_AMS_1: offline  ams_model=ams2  index=0  bus id 0x00\n"
    RESUMING = ("    resuming: an earlier run (1999-12-31 23:59) left it waiting in its "
                "bootloader\n")
    FW = "[3] bridge fw >= 175\n    bridge_fw = 281\n"
    IMAGE = (f"[4] the image\n    {AMS2_FILE}\n    {AMS2_FILE}: AMS 2 Pro (n3f_rev5) "
             f"01.02.03.04, 2628 bytes, MD5 {AMS2_MD5}\n")
    BUS = "[5] the bus (Klipper lends this run the bridge link; a bridge takes one client)\n"
    ONE = "    answering on the bus: index 0 (bus id 0x00)\n"
    NONE = "    answering on the bus: none\n"
    READ = "    Bambu_AMS_1: firmware 01.02.03.00, model code N3F05\n"
    PROBE = "    OK\n[6] cmd1 enters the loader (non-destructive)\n"
    LOADER = PROBE + "    loader confirmed after cmd1 #1\n[7] FLASH: header (erase) + 3 blocks\n"
    BOOTED = ("[8] the unit boots its new firmware\n"
              "    Bambu_AMS_1: firmware 01.02.03.04, model code N3F05\n")
    DONE = "\n==> DONE. Bambu_AMS_1 now runs 01.02.03.04 (was 01.02.03.00).\n"
    CHECKED = ("\n==> CHECKS PASSED. Nothing was changed. --mode go updates the unit, after "
               "checking the bus itself.\n")
    NOTE = ("    note: could not give Klipper the bridge link back; it takes it back by itself "
            "within 30 minutes, or at a Klipper restart\n")

    @classmethod
    def checks(cls, rig: _Rig, row: Optional[str] = None, image: Optional[str] = None,
               bridge: Optional[str] = None) -> str:
        """Steps 1 to 4 of a run that passed them."""
        return (cls.START + "[2] the unit\n" + (cls.ROW if row is None else row)
                + f"    bridge: {rig.port if bridge is None else bridge}\n" + cls.FW
                + (cls.IMAGE if image is None else image))

    @classmethod
    def flashed(cls) -> str:
        """Steps 5 to 8 of a default update that went through."""
        return cls.BUS + cls.ONE + cls.READ + cls.LOADER + cls.BOOTED + cls.DONE


class TestAbort:
    """Abort: why a check refused the update."""

    def test_an_abort_is_an_error_carrying_its_reason(self) -> None:
        with pytest.raises(Exception) as raised:
            raise ams_flash.Abort("a print is active")
        assert type(raised.value) is ams_flash.Abort
        assert str(raised.value) == "a print is active"


class TestDeviceOf:
    """device_of: the bus device a unit id is addressed through."""

    @pytest.mark.parametrize("unit_id,device", [(0x00, 0x0700), (0x03, 0x0700), (0x7F, 0x0700),
                                                (0x80, 0x1800), (0x83, 0x1800)])
    def test_ids_from_0x80_up_are_ams_hts(self, unit_id: int, device: int) -> None:
        assert ams_flash.device_of(unit_id) == device


class TestBoardOfCode:
    """board_of_code: the image board a unit's model code needs."""

    @pytest.mark.parametrize("code,board", [("N3F05", "n3f_rev5"), ("N3S05", "n3s_rev5"),
                                            ("AMS08", "ams_rev8"), ("AMS10", "ams_rev10")])
    def test_each_models_code_names_its_board(self, code: str, board: str) -> None:
        assert ams_flash.board_of_code(code) == board

    @pytest.mark.parametrize("code", ["", None, "X1C05", "N3F5", "n3f05", "N3F055", " N3F05"])
    def test_anything_else_names_no_board(self, code: Optional[str]) -> None:
        assert ams_flash.board_of_code(code) == ""


class TestModelOfBoard:
    """model_of_board: the model key a board belongs to."""

    @pytest.mark.parametrize("board,model", [("n3f_rev5", "ams2"), ("n3s_rev5", "ht"),
                                             ("ams_rev8", "ams1"), ("n3f", "ams2")])
    def test_the_boards_family_names_its_model(self, board: str, model: str) -> None:
        assert ams_flash.model_of_board(board) == model

    @pytest.mark.parametrize("board", ["x1c_rev1", "rev5_n3f", "", None])
    def test_an_unknown_board_has_no_model(self, board: Optional[str]) -> None:
        assert ams_flash.model_of_board(board) == ""


class TestDescribe:
    """describe: a board for people."""

    @pytest.mark.parametrize("board,said", [("n3f_rev5", "AMS 2 Pro (n3f_rev5)"),
                                            ("n3s_rev5", "AMS HT (n3s_rev5)"),
                                            ("ams_rev7", "AMS 1 (ams_rev7)")])
    def test_a_known_board_is_named_with_its_model(self, board: str, said: str) -> None:
        assert ams_flash.describe(board) == said

    @pytest.mark.parametrize("board,said", [("x1c_rev1", "x1c_rev1"), ("", "an unknown board"),
                                            (None, "an unknown board")])
    def test_an_unknown_board_is_named_as_it_is(self, board: Optional[str], said: str) -> None:
        assert ams_flash.describe(board) == said


def _u32(image: bytes, offset: int, value: int) -> bytes:
    """`image` with the little-endian u32 at `offset` set to `value`."""
    out = bytearray(image)
    out[offset:offset + 4] = value.to_bytes(4, "little")
    return bytes(out)


class TestParseImage:
    """parse_image: a whole Bambu AMS image, and what it is for."""

    @pytest.mark.parametrize("image,name,board,model,version", [
        (AMS2_IMAGE, AMS2_FILE, "n3f_rev5", "ams2", "01.02.03.04"),
        (HT_IMAGE, HT_FILE, "n3s_rev5", "ht", "01.02.03.05"),
        (AMS1_IMAGE, AMS1_FILE, "ams_rev8", "ams1", "01.02.03.06"),
    ], ids=["ams2", "ht", "ams1"])
    def test_an_image_names_its_board_model_and_version(self, image: bytes, name: str,
                                                        board: str, model: str,
                                                        version: str) -> None:
        assert ams_flash.parse_image(image) == {"name": name, "board": board, "model": model,
                                                "version": version, "hlen": 0x80}

    def test_a_longer_container_header_is_measured(self) -> None:
        image = _bimh(AMS2_FILE, _payload(300, 4), hlen=0x100)
        assert ams_flash.parse_image(image)["hlen"] == 0x100

    def test_the_container_header_may_end_one_byte_before_the_file(self) -> None:
        assert ams_flash.parse_image(_bimh(AMS2_FILE, b"\x01"))["hlen"] == 0x80

    @pytest.mark.parametrize("data", [b"BIMH" + bytes(0x7B), b"BIMX" + AMS2_IMAGE[4:]],
                             ids=["too-short", "no-magic"])
    def test_a_file_without_the_container_is_refused(self, data: bytes) -> None:
        with pytest.raises(ValueError) as raised:
            ams_flash.parse_image(data)
        assert str(raised.value) == "not a Bambu AMS image (no BIMH header)"

    def test_an_unknown_container_format_is_refused(self) -> None:
        with pytest.raises(ValueError) as raised:
            ams_flash.parse_image(_u32(AMS2_IMAGE, 4, 2))
        assert str(raised.value) == "unknown image format 2"

    @pytest.mark.parametrize("data,said", [
        (_u32(AMS2_IMAGE, 8, 2629), "the file is 2628 bytes but its header says 2629; it is cut "
                                    "short or damaged"),
        (AMS2_IMAGE[:-1], "the file is 2627 bytes but its header says 2628; it is cut short or "
                          "damaged"),
    ], ids=["damaged", "cut-short"])
    def test_a_file_that_is_not_the_size_it_says_is_refused(self, data: bytes, said: str) -> None:
        with pytest.raises(ValueError) as raised:
            ams_flash.parse_image(data)
        assert str(raised.value) == said

    @pytest.mark.parametrize("hlen,rest", [(0x7F, 2628 - 0x7F), (2628, 0), (0x80, 2499)],
                             ids=["header-too-short", "header-is-the-file", "rest-is-off"])
    def test_header_lengths_that_do_not_add_up_are_refused(self, hlen: int, rest: int) -> None:
        # Each case breaks one of the three conditions and keeps the other two.
        with pytest.raises(ValueError) as raised:
            ams_flash.parse_image(_u32(_u32(AMS2_IMAGE, 0x20, hlen), 0x28, rest))
        assert str(raised.value) == "the image's header lengths do not add up"

    @pytest.mark.parametrize("name,said", [
        ("x1c_rev1-firmware-v01.02.03.04-20000101000000_product.bin.sig",
         "not an AMS image (x1c_rev1-firmware-v01.02.03.04-20000101000000_product.bin.sig)"),
        ("n3f_rev5-firmware-v1.2.3.4-20000101000000_product.bin.sig",
         "not an AMS image (n3f_rev5-firmware-v1.2.3.4-20000101000000_product.bin.sig)"),
        ("", "not an AMS image (unnamed)"),
    ], ids=["another-product", "short-version", "unnamed"])
    def test_an_image_that_is_not_for_an_ams_is_refused(self, name: str, said: str) -> None:
        with pytest.raises(ValueError) as raised:
            ams_flash.parse_image(_bimh(name, _payload(64, 5)))
        assert str(raised.value) == said


class TestBody:
    """_body: one update frame body, laid out byte by byte."""

    def test_a_header_body_carries_no_trailer(self) -> None:
        body = ams_flash._body(0x1800, 0x80, 0x02, 0x00010203, 0, 0, b"\xaa\xbb", False)
        assert body.hex() == ("0018" "0009" "0601" "02" "18" "00000000" "80000000" "03020100"
                              "00000000" "00000000" "02000000" "aabb")

    def test_a_data_block_body_ends_in_a_zero_trailer(self) -> None:
        body = ams_flash._body(0x0700, 0x01, 0x03, 0x2D000, 5, 0x400, b"\x11\x22\x33", True)
        assert body.hex() == ("0007" "0009" "0601" "03" "07" "00000000" "01000000" "00d00200"
                              "05000000" "00040000" "03000000" "112233" "00000000")


class TestPack:
    """pack: a Bambu image cut into one unit's update frames."""

    @staticmethod
    def _body(dev: int, unit_id: int, cmd: int, f16: int, f20: int, f24: int, payload: bytes,
              trailer: bool) -> bytes:
        """A frame body laid out field by field with struct."""
        head = struct.pack("<HH2sBB4xIIIII", dev, 0x0900, b"\x06\x01", cmd, dev >> 8, unit_id,
                           f16, f20, f24, len(payload))
        return head + payload + (bytes(4) if trailer else b"")

    def test_a_boxed_units_update_is_its_header_then_1_kib_blocks(self) -> None:
        rest = AMS2_IMAGE[0x80:]
        assert ams_flash.pack(AMS2_IMAGE, 0x01) == {
            "header": self._body(0x0700, 0x01, 0x02, 2628, 0, 0, AMS2_IMAGE[:0x80], False),
            "blocks": [self._body(0x0700, 0x01, 0x03, 0x2D000, 0, 0x400, rest[:1024], True),
                       self._body(0x0700, 0x01, 0x03, 0x2D000, 1, 0x400, rest[1024:2048], True),
                       self._body(0x0700, 0x01, 0x03, 0x2D000, 2, 0x400, rest[2048:], True)]}
        assert len(rest[2048:]) == 452                   # the last block is short

    def test_an_hts_update_goes_to_its_own_device(self) -> None:
        rest = HT_IMAGE[0x80:]
        assert ams_flash.pack(HT_IMAGE, 0x80) == {
            "header": self._body(0x1800, 0x80, 0x02, 2628, 0, 0, HT_IMAGE[:0x80], False),
            "blocks": [self._body(0x1800, 0x80, 0x03, 0x2D000, 0, 0x400, rest[:1024], True),
                       self._body(0x1800, 0x80, 0x03, 0x2D000, 1, 0x400, rest[1024:2048], True),
                       self._body(0x1800, 0x80, 0x03, 0x2D000, 2, 0x400, rest[2048:], True)]}

    def test_an_image_of_whole_blocks_ends_on_a_whole_block(self) -> None:
        image = _bimh(AMS2_FILE, _payload(2048, 6))
        rest = image[0x80:]
        assert ams_flash.pack(image, 0x00)["blocks"] == [
            self._body(0x0700, 0x00, 0x03, 0x2D000, 0, 0x400, rest[:1024], True),
            self._body(0x0700, 0x00, 0x03, 0x2D000, 1, 0x400, rest[1024:], True)]

    def test_a_longer_container_header_goes_whole_into_the_header_frame(self) -> None:
        image = _bimh(AMS2_FILE, _payload(100, 7), hlen=0x100)
        assert ams_flash.pack(image, 0x00) == {
            "header": self._body(0x0700, 0x00, 0x02, 0x164, 0, 0, image[:0x100], False),
            "blocks": [self._body(0x0700, 0x00, 0x03, 0x2D000, 0, 0x400, image[0x100:], True)]}


class TestLoadCatalog:
    """load_catalog: the images ams_firmware.json lists."""

    def test_the_listed_images_come_back_in_order(self, tmp_path: Path) -> None:
        first = _entry()
        second = _entry(board="n3s_rev5", version="01.02.03.05", file=HT_FILE, md5=None)
        del second["md5"]                                # the MD5 is not required
        _catalog(tmp_path, first, second)
        assert ams_flash.load_catalog(str(tmp_path)) == [first, second]

    @pytest.mark.parametrize("missing", ["board", "version", "file", "url"])
    def test_an_entry_without_a_field_it_needs_is_dropped(self, tmp_path: Path,
                                                          missing: str) -> None:
        broken = _entry()
        del broken[missing]
        _catalog(tmp_path, broken, _entry())
        assert ams_flash.load_catalog(str(tmp_path)) == [_entry()]

    def test_an_empty_field_or_an_entry_that_is_no_record_is_dropped(self,
                                                                      tmp_path: Path) -> None:
        _catalog(tmp_path, "n3f_rev5", ["n3f_rev5"], _entry(url=""), _entry())
        assert ams_flash.load_catalog(str(tmp_path)) == [_entry()]

    @pytest.mark.parametrize("text", [None, "not json", '["images"]', '{"images": null}',
                                      '{"about": "made up"}'],
                             ids=["missing", "not-json", "not-a-record", "null", "no-images"])
    def test_a_catalog_that_cannot_be_read_lists_nothing(self, tmp_path: Path,
                                                         text: Optional[str]) -> None:
        if text is not None:
            (tmp_path / "ams_firmware.json").write_text(text)
        assert ams_flash.load_catalog(str(tmp_path)) == []


class TestMd5Of:
    """md5_of: the MD5 of a file's bytes."""

    @pytest.mark.parametrize("data,md5", [(b"", "d41d8cd98f00b204e9800998ecf8427e"),
                                          (b"abc", "900150983cd24fb0d6963f7d28e17f72")],
                             ids=["empty", "abc"])
    def test_the_md5_of_a_files_bytes(self, tmp_path: Path, data: bytes, md5: str) -> None:
        # RFC 1321's own test values.
        path = tmp_path / "made-up.bin"
        path.write_bytes(data)
        assert ams_flash.md5_of(str(path)) == md5


#: 40 made-up bytes behind PACKAGE_URL in the RemoteFile tests.
DATA40 = bytes(range(100, 140))


def _remote(monkeypatch: pytest.MonkeyPatch, block: int = 16) -> Tuple[Any, _Server]:
    """A RemoteFile reading DATA40 from a made-up server, in blocks of `block` bytes."""
    monkeypatch.setattr(ams_flash.RemoteFile, "BLOCK", block)
    server = _Server({PACKAGE_URL: DATA40})
    return ams_flash.RemoteFile(PACKAGE_URL, opener=server), server


class TestRemoteFileInit:
    """RemoteFile.__init__: a view of a file on a web server, sized by a one-byte probe."""

    def test_a_one_byte_probe_learns_the_files_size(self) -> None:
        server = _Server({PACKAGE_URL: DATA40})
        remote = ams_flash.RemoteFile(PACKAGE_URL, timeout=7.5, opener=server)
        assert (remote.url, remote.timeout, remote.size, remote._pos) == (PACKAGE_URL, 7.5, 40, 0)
        assert remote._open is server
        assert remote._blocks == {}
        assert server.asked == [(PACKAGE_URL, "bytes=0-0", "ams_flash/2", 7.5)]

    def test_without_an_opener_urlopen_serves_it(self, monkeypatch: pytest.MonkeyPatch) -> None:
        server = _Server({PACKAGE_URL: DATA40})
        monkeypatch.setattr(urllib.request, "urlopen", server)
        remote = ams_flash.RemoteFile(PACKAGE_URL)
        assert remote._open is server
        assert remote.size == 40
        assert server.asked == [(PACKAGE_URL, "bytes=0-0", "ams_flash/2", 30.0)]


class TestRemoteFileRange:
    """RemoteFile._range: one part of the file, and the file's whole size."""

    REFUSED = ("the server will not send part of the package; download the image (.bin.sig) "
               "yourself and put it beside ams_flash.py")

    def test_a_part_comes_back_with_the_files_size(self,
                                                   monkeypatch: pytest.MonkeyPatch) -> None:
        remote, server = _remote(monkeypatch)
        assert remote._range(4, 9) == (DATA40[4:10], 40)
        assert server.asked[1:] == [(PACKAGE_URL, "bytes=4-9", "ams_flash/2", 30.0)]

    @pytest.mark.parametrize("answer", [
        _Response(DATA40[:1], 200, {"Content-Range": "bytes 0-0/40"}),
        _Response(DATA40[:1], 206, {}),
        _Response(DATA40[:1], 206, {"Content-Range": "bytes */40"}),
        _Response(DATA40[1:2], 206, {"Content-Range": "bytes 1-1/40"}),
        _Response(DATA40[:1], None, {"Content-Range": "bytes 0-0/40"}),
    ], ids=["whole-file-status", "no-content-range", "unreadable-range", "another-part",
            "no-status"])
    def test_an_answer_that_is_not_the_part_asked_for_is_refused(self,
                                                                 answer: _Response) -> None:
        with pytest.raises(IOError) as raised:
            ams_flash.RemoteFile(PACKAGE_URL, opener=lambda req, timeout: answer)
        assert str(raised.value) == self.REFUSED

    def test_a_part_that_arrives_short_is_refused(self) -> None:
        answer = _Response(b"", 206, {"Content-Range": "bytes 0-0/40"})
        with pytest.raises(IOError) as raised:
            ams_flash.RemoteFile(PACKAGE_URL, opener=lambda req, timeout: answer)
        assert str(raised.value) == "part of the package arrived cut short"


class TestRemoteFileBlock:
    """RemoteFile._block: one block of the file, fetched once."""

    def test_a_block_is_fetched_once_and_kept(self, monkeypatch: pytest.MonkeyPatch) -> None:
        remote, server = _remote(monkeypatch)
        assert remote._block(1) == DATA40[16:32]
        assert remote._block(1) == DATA40[16:32]
        assert [ask[1] for ask in server.asked] == ["bytes=0-0", "bytes=16-31"]
        assert remote._blocks == {1: DATA40[16:32]}

    def test_the_last_block_ends_with_the_file(self, monkeypatch: pytest.MonkeyPatch) -> None:
        remote, server = _remote(monkeypatch)
        assert remote._block(2) == DATA40[32:]
        assert [ask[1] for ask in server.asked] == ["bytes=0-0", "bytes=32-39"]
        assert remote._blocks == {2: DATA40[32:]}


class TestRemoteFileReadable:
    """RemoteFile.readable."""

    def test_it_can_be_read(self, monkeypatch: pytest.MonkeyPatch) -> None:
        remote, server = _remote(monkeypatch)
        assert remote.readable() is True
        assert len(server.asked) == 1


class TestRemoteFileSeekable:
    """RemoteFile.seekable."""

    def test_it_can_seek(self, monkeypatch: pytest.MonkeyPatch) -> None:
        remote, server = _remote(monkeypatch)
        assert remote.seekable() is True
        assert len(server.asked) == 1


class TestRemoteFileTell:
    """RemoteFile.tell: where the next read starts."""

    def test_it_tells_where_the_next_read_starts(self, monkeypatch: pytest.MonkeyPatch) -> None:
        remote, _server = _remote(monkeypatch)
        assert remote.tell() == 0
        assert remote.read(5) == DATA40[:5]
        assert remote.tell() == 5


class TestRemoteFileSeek:
    """RemoteFile.seek: move the read position without reading."""

    @pytest.mark.parametrize("offset,whence,pos", [(3, io.SEEK_SET, 3), (5, io.SEEK_CUR, 17),
                                                   (-4, io.SEEK_END, 36),
                                                   (-50, io.SEEK_CUR, 0)],
                             ids=["from-the-start", "from-here", "from-the-end", "before-0"])
    def test_a_seek_moves_the_read_position(self, monkeypatch: pytest.MonkeyPatch, offset: int,
                                            whence: int, pos: int) -> None:
        remote, server = _remote(monkeypatch)
        assert remote.seek(12) == 12
        assert remote.seek(offset, whence) == pos
        assert remote._pos == pos
        assert len(server.asked) == 1                    # seeking reads nothing


class TestRemoteFileReadinto:
    """RemoteFile.readinto: bytes from the read position, block by block."""

    def test_a_read_across_blocks_fetches_each_once(self,
                                                    monkeypatch: pytest.MonkeyPatch) -> None:
        remote, server = _remote(monkeypatch)
        remote.seek(10)
        buf = bytearray(12)
        assert remote.readinto(buf) == 12
        assert bytes(buf) == DATA40[10:22]
        assert remote._pos == 22
        assert [ask[1] for ask in server.asked] == ["bytes=0-0", "bytes=0-15", "bytes=16-31"]

    def test_a_read_stops_at_the_end_of_the_file(self, monkeypatch: pytest.MonkeyPatch) -> None:
        remote, server = _remote(monkeypatch)
        remote.seek(36)
        buf = bytearray(10)
        assert remote.readinto(buf) == 4
        assert bytes(buf) == DATA40[36:] + bytes(6)
        assert remote._pos == 40
        assert [ask[1] for ask in server.asked] == ["bytes=0-0", "bytes=32-39"]

    @pytest.mark.parametrize("start,size", [(40, 4), (12, 0)], ids=["at-the-end", "no-room"])
    def test_nothing_to_read_reads_nothing(self, monkeypatch: pytest.MonkeyPatch, start: int,
                                           size: int) -> None:
        remote, server = _remote(monkeypatch)
        remote.seek(start)
        assert remote.readinto(bytearray(size)) == 0
        assert remote._pos == start
        assert len(server.asked) == 1

    def test_a_buffer_of_wider_items_is_filled_byte_by_byte(
            self, monkeypatch: pytest.MonkeyPatch) -> None:
        remote, _server = _remote(monkeypatch)
        words = array.array("H", [0, 0])
        assert remote.readinto(words) == 4
        assert words.tobytes() == DATA40[:4]

    def test_a_server_that_sends_less_than_asked_is_an_error_not_a_hang(
            self, monkeypatch: pytest.MonkeyPatch) -> None:
        # A server may answer a range with a shorter one than asked for, saying so in its
        # Content-Range. A read must not spin forever on the block that ends early.
        monkeypatch.setattr(ams_flash.RemoteFile, "BLOCK", 16)

        def stingy(req: urllib.request.Request, timeout: float) -> _Response:
            start, end = (int(x) for x in req.get_header("Range")[6:].split("-"))
            end = min(end, start + 7)                    # never more than 8 bytes
            return _Response(DATA40[start:end + 1], 206,
                             {"Content-Range": f"bytes {start}-{end}/40"})

        class Hung(Exception):
            """The read made no progress."""

        def hung(signum: int, frame: Any) -> None:
            raise Hung("RemoteFile.readinto made no progress")

        remote = ams_flash.RemoteFile(PACKAGE_URL, opener=stingy)
        previous = signal.signal(signal.SIGALRM, hung)
        signal.setitimer(signal.ITIMER_REAL, 2.0)
        try:
            with pytest.raises(IOError) as raised:
                remote.read(16)
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous)
        assert str(raised.value) == "part of the package arrived cut short"


class TestDownloadImage:
    """download_image: a listed image fetched beside the script, unless an intact copy is there."""

    def test_the_image_is_read_out_of_the_package_in_parts(self, tmp_path: Path,
                                                           monkeypatch: pytest.MonkeyPatch,
                                                           capsys: pytest.CaptureFixture[str]
                                                           ) -> None:
        monkeypatch.setattr(ams_flash.RemoteFile, "BLOCK", 4096)
        decoy = _payload(65536, 9)
        server = _Server({PACKAGE_URL: _package({"made-up-other.bin": decoy,
                                                 AMS2_FILE: AMS2_IMAGE})})
        assert ams_flash.download_image(_entry(), str(tmp_path), opener=server) == str(
            tmp_path / AMS2_FILE)
        assert (tmp_path / AMS2_FILE).read_bytes() == AMS2_IMAGE
        assert sorted(p.name for p in tmp_path.iterdir()) == [AMS2_FILE]   # no .part left
        assert server.asked[0] == (PACKAGE_URL, "bytes=0-0", "ams_flash/2", 30.0)
        assert [ask for ask in server.asked if ask[1] is None] == []        # parts only
        assert server.served < len(decoy)                # the other member stayed behind
        assert capsys.readouterr() == ("", "")

    def test_a_package_url_is_known_by_its_path_in_any_case(self, tmp_path: Path) -> None:
        url = "https://example.invalid/offline/PACKAGE.ZIP"
        server = _Server({url: _package({AMS2_FILE: AMS2_IMAGE})})
        ams_flash.download_image(_entry(url=url), str(tmp_path), opener=server)
        assert server.asked[0] == (url, "bytes=0-0", "ams_flash/2", 30.0)
        assert (tmp_path / AMS2_FILE).read_bytes() == AMS2_IMAGE

    def test_an_image_url_is_fetched_whole(self, tmp_path: Path) -> None:
        # Only the URL's path says whether it is a package, not its query.
        url = "https://example.invalid/images/made-up.bin.sig?mirror=package.zip"
        server = _Server({url: AMS2_IMAGE})
        assert ams_flash.download_image(_entry(url=url), str(tmp_path), opener=server) == str(
            tmp_path / AMS2_FILE)
        assert server.asked == [(url, None, "ams_flash/2", 60)]
        assert (tmp_path / AMS2_FILE).read_bytes() == AMS2_IMAGE

    def test_without_an_opener_urlopen_fetches_it(self, tmp_path: Path,
                                                  monkeypatch: pytest.MonkeyPatch) -> None:
        url = "https://example.invalid/images/made-up.bin.sig"
        server = _Server({url: AMS2_IMAGE})
        monkeypatch.setattr(urllib.request, "urlopen", server)
        ams_flash.download_image(_entry(url=url), str(tmp_path))
        assert server.asked == [(url, None, "ams_flash/2", 60)]
        assert (tmp_path / AMS2_FILE).read_bytes() == AMS2_IMAGE

    @pytest.mark.parametrize("md5,copy", [(AMS2_MD5, AMS2_IMAGE),
                                          (AMS2_MD5.upper(), AMS2_IMAGE),
                                          (None, b"a copy with no MD5 to check"),
                                          ("", b"a copy with no MD5 to check")],
                             ids=["md5", "upper-case-md5", "no-md5", "empty-md5"])
    def test_an_intact_copy_is_not_fetched_again(self, tmp_path: Path, md5: Optional[str],
                                                 copy: bytes) -> None:
        (tmp_path / AMS2_FILE).write_bytes(copy)
        server = _Server({})
        assert ams_flash.download_image(_entry(md5=md5), str(tmp_path), opener=server) == str(
            tmp_path / AMS2_FILE)
        assert server.asked == []
        assert (tmp_path / AMS2_FILE).read_bytes() == copy

    def test_a_damaged_copy_is_fetched_again(self, tmp_path: Path) -> None:
        (tmp_path / AMS2_FILE).write_bytes(b"damaged")
        server = _Server({PACKAGE_URL: _package({AMS2_FILE: AMS2_IMAGE})})
        ams_flash.download_image(_entry(), str(tmp_path), opener=server)
        assert server.asked[0] == (PACKAGE_URL, "bytes=0-0", "ams_flash/2", 30.0)
        assert (tmp_path / AMS2_FILE).read_bytes() == AMS2_IMAGE

    def test_an_image_that_arrives_damaged_is_not_kept(self, tmp_path: Path) -> None:
        server = _Server({PACKAGE_URL: _package({AMS2_FILE: AMS2_IMAGE})})
        with pytest.raises(IOError) as raised:
            ams_flash.download_image(_entry(md5="0" * 32), str(tmp_path), opener=server)
        assert str(raised.value) == (f"{AMS2_FILE} arrived damaged (MD5 {AMS2_MD5}, expected "
                                     f"00000000000000000000000000000000)")
        assert list(tmp_path.iterdir()) == []

    def test_an_image_listed_without_an_md5_is_kept_as_it_came(self, tmp_path: Path) -> None:
        server = _Server({PACKAGE_URL: _package({AMS2_FILE: b"whatever came"})})
        ams_flash.download_image(_entry(md5=None), str(tmp_path), opener=server)
        assert (tmp_path / AMS2_FILE).read_bytes() == b"whatever came"

    def test_a_member_in_a_folder_is_saved_by_its_own_name(self, tmp_path: Path) -> None:
        member = f"firmware/ams/{AMS2_FILE}"
        server = _Server({PACKAGE_URL: _package({member: AMS2_IMAGE})})
        assert ams_flash.download_image(_entry(file=member), str(tmp_path),
                                        opener=server) == str(tmp_path / AMS2_FILE)
        assert (tmp_path / AMS2_FILE).read_bytes() == AMS2_IMAGE


class TestImageFromFile:
    """image_from_file: the image in a .bin.sig, or the one wanted out of a package."""

    PACKAGE = {AMS2_FILE: AMS2_IMAGE, HT_FILE: HT_IMAGE, "readme.txt": b"made-up notes"}

    def _zip(self, tmp_path: Path, members: Optional[Dict[str, bytes]] = None) -> str:
        path = tmp_path / "made-up-package.zip"
        path.write_bytes(_package(self.PACKAGE if members is None else members))
        return str(path)

    def test_a_bin_sig_is_the_image(self, tmp_path: Path) -> None:
        path = tmp_path / AMS2_FILE
        path.write_bytes(AMS2_IMAGE)
        # The board and the version only pick out of a package.
        assert ams_flash.image_from_file(str(path), "n3s_rev5", "09.09.09.09") == (
            AMS2_IMAGE, AMS2_FILE)

    @pytest.mark.parametrize("board,version,image,name", [
        ("n3s_rev5", "", HT_IMAGE, HT_FILE),
        ("", "01.02.03.04", AMS2_IMAGE, AMS2_FILE),
        ("n3f_rev5", "01.02.03.04", AMS2_IMAGE, AMS2_FILE),
    ], ids=["board", "version", "both"])
    def test_the_wanted_image_is_taken_out_of_a_package(self, tmp_path: Path, board: str,
                                                        version: str, image: bytes,
                                                        name: str) -> None:
        assert ams_flash.image_from_file(self._zip(tmp_path), board, version) == (
            image, f"made-up-package.zip: {name}")

    def test_a_package_of_one_image_needs_no_board_or_version(self, tmp_path: Path) -> None:
        path = self._zip(tmp_path, {AMS2_FILE: AMS2_IMAGE, "readme.txt": b"made-up notes"})
        assert ams_flash.image_from_file(path) == (AMS2_IMAGE,
                                                   f"made-up-package.zip: {AMS2_FILE}")

    def test_an_image_in_a_folder_of_the_package_is_known_by_its_name(self,
                                                                      tmp_path: Path) -> None:
        path = self._zip(tmp_path, {f"firmware/{HT_FILE}": HT_IMAGE})
        assert ams_flash.image_from_file(path, "n3s_rev5") == (HT_IMAGE,
                                                               f"made-up-package.zip: {HT_FILE}")

    @pytest.mark.parametrize("board,version", [("n3f_rev5", "01.02.03.05"),
                                               ("n3s_rev5", "01.02.03.04")],
                             ids=["right-board-wrong-version", "right-version-wrong-board"])
    def test_the_board_and_the_version_must_both_match(self, tmp_path: Path, board: str,
                                                       version: str) -> None:
        with pytest.raises(ValueError) as raised:
            ams_flash.image_from_file(self._zip(tmp_path), board, version)
        assert str(raised.value) == (f"it holds 0 images for {board} {version} ({AMS2_FILE}, "
                                     f"{HT_FILE})")

    def test_a_package_holding_two_matches_is_refused(self, tmp_path: Path) -> None:
        with pytest.raises(ValueError) as raised:
            ams_flash.image_from_file(self._zip(tmp_path))
        assert str(raised.value) == f"it holds 2 images for this unit ({AMS2_FILE}, {HT_FILE})"

    def test_a_package_without_ams_images_is_refused(self, tmp_path: Path) -> None:
        with pytest.raises(ValueError) as raised:
            ams_flash.image_from_file(self._zip(tmp_path, {"readme.txt": b"made-up notes"}))
        assert str(raised.value) == "it holds 0 images for this unit (no AMS images)"


class TestBuildCmd1:
    """build_cmd1: the enter-loader poke, addressed per model."""

    @pytest.mark.parametrize("target,ams_id,counter", [(0x0700, 0x00, 0x1234),
                                                       (0x1800, 0x80, 0x0042)],
                             ids=["boxed", "ht"])
    def test_cmd1_is_the_enter_loader_frame(self, target: int, ams_id: int,
                                            counter: int) -> None:
        head = bytes((0x3D, 0x04, counter & 0xFF, counter >> 8, 0x31, 0x00))
        body = bytearray(40)
        body[0:2] = target.to_bytes(2, "little")         # to the unit
        body[2:4] = b"\x00\x09"                          # from the printer
        body[4:7] = b"\x06\x01\x01"                      # op 06 01, cmd 01
        body[7] = target >> 8                            # device class
        body[12] = ams_id
        frame = head + bytes((_crc8(head),)) + bytes(body)
        assert ams_flash.build_cmd1(target, ams_id, counter) == frame + _crc16(frame).to_bytes(
            2, "little")

    def test_the_two_models_differ_in_the_payload_not_only_the_address(self) -> None:
        # Aiming at an HT by changing the target alone would leave the class byte and the AMS id at
        # their AMS 2 values, and the HT ignores that.
        ams2 = ams_flash.build_cmd1(0x0700, 0x00)
        ht = ams_flash.build_cmd1(0x1800, 0x80)
        # (default counter, address, device class, AMS id)
        assert (ams2[2:4], ams2[7:9], ams2[14], ams2[19]) == (
            b"\x77\x00", b"\x00\x07", 0x07, 0x00)
        assert (ht[2:4], ht[7:9], ht[14], ht[19]) == (b"\x77\x00", b"\x00\x18", 0x18, 0x80)

    @pytest.mark.parametrize("target,class_byte", [(0x0700, 0x07), (0x1800, 0x18)])
    def test_the_device_class_byte_tracks_the_address(self, target: int,
                                                        class_byte: int) -> None:
        # [14] is derived from the address, so it cannot drift out of step with it.
        assert ams_flash.build_cmd1(target, 0, 1)[14] == class_byte

    @pytest.mark.parametrize("target,ams_id", [(0x0700, 0x00), (0x1800, 0x80)])
    def test_cmd1_stays_crc_correct_for_both_models(self, target: int, ams_id: int) -> None:
        frame = ams_flash.build_cmd1(target, ams_id, 0x1234)
        assert len(frame) == 49
        assert frame[2:4] == b"\x34\x12"
        assert frame[6] == _crc8(frame[:6])
        assert int.from_bytes(frame[47:49], "little") == _crc16(frame[:47])


class TestLoaderTargetOf:
    """loader_target_of: the (device, AMS id) a packed image's frames are for."""

    @pytest.mark.parametrize("image,unit_id,target,address,class_byte", [
        (AMS2_IMAGE, 0x00, 0x0700, b"\x00\x07", 0x07),
        (HT_IMAGE, 0x80, 0x1800, b"\x00\x18", 0x18),
    ], ids=["ams2", "ht"])
    def test_cmd1_is_addressed_off_the_packed_image(self, image: bytes, unit_id: int,
                                                    target: int, address: bytes,
                                                    class_byte: int) -> None:
        art = ams_flash.pack(image, unit_id)
        assert ams_flash.loader_target_of(art) == (target, unit_id)
        frame = ams_flash.build_cmd1(*ams_flash.loader_target_of(art))
        assert (frame[7:9], frame[14], frame[19]) == (address, class_byte, unit_id)

    def test_there_is_no_fixed_loader_target(self) -> None:
        art = {"header": (0x2A0B).to_bytes(2, "little") + bytes(10) + b"\x42", "blocks": []}
        assert ams_flash.loader_target_of(art) == (0x2A0B, 0x42)


class TestPlan:
    """plan: a packed image's frames in send order."""

    ART = {"header": b"\x01\x02\x03", "blocks": [b"\x04\x05", b"\x06"]}

    def test_the_header_goes_first_then_each_block(self) -> None:
        assert list(ams_flash.plan(self.ART)) == [("header", -1, _frame(b"\x01\x02\x03", 0)),
                                                   ("data", 0, _frame(b"\x04\x05", 1)),
                                                   ("data", 1, _frame(b"\x06", 2))]

    def test_the_frame_count_starts_where_it_is_told(self) -> None:
        assert [frame for _kind, _seq, frame in ams_flash.plan(self.ART, 0x01FF)] == [
            _frame(b"\x01\x02\x03", 0x01FF), _frame(b"\x04\x05", 0x0200), _frame(b"\x06", 0x0201)]


class TestLinkRead:
    """Link._read over a real pseudo-terminal."""

    def test_a_usb_hangup_is_a_dead_link_not_a_quiet_one(
            self, ams_flash_pty: _AmsFlashPty) -> None:
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        assert link.dead is False
        ams_flash_pty.hang_up()
        with pytest.raises(IOError) as raised:
            link._read()
        assert str(raised.value) == "bridge link closed"
        assert link.dead is True
        link.close()

    def test_a_quiet_port_reads_as_nothing(self, ams_flash_pty: _AmsFlashPty) -> None:
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        assert link._read() == b""
        assert link.dead is False
        link.close()

    def test_what_is_buffered_is_returned(self, ams_flash_pty: _AmsFlashPty) -> None:
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        os.write(ams_flash_pty.master, b'{"evt":"info"}\n')
        assert link._read() == b'{"evt":"info"}\n'
        assert link.dead is False
        link.close()


class TestLinkLines:
    """Link.lines over a real pseudo-terminal."""

    def test_the_usb_link_needs_no_pyserial_and_never_blocks(
            self, ams_flash_pty: _AmsFlashPty, monkeypatch: pytest.MonkeyPatch) -> None:
        # A USB bridge is what the testers have.
        real_import = builtins.__import__

        def no_serial(name: str, *args: Any, **kwargs: Any) -> ModuleType:
            if name == "serial":
                raise ImportError("no pyserial here")
            return real_import(name, *args, **kwargs)

        monkeypatch.setattr(builtins, "__import__", no_serial)
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        lines = link.lines()
        assert next(lines) is None                       # quiet: yields, no hang
        os.write(ams_flash_pty.master, b'{"evt":"info"}\n')
        assert next(lines) == '{"evt":"info"}'
        assert link.buf == b""
        link.send({"cmd": "info"})
        assert os.read(ams_flash_pty.master, 100) == b'{"cmd": "info"}\n'
        link.close()

    def test_a_line_is_held_until_it_ends(self, ams_flash_pty: _AmsFlashPty) -> None:
        link = ams_flash.Link(ams_flash_pty.path, timeout=0.05)
        lines = link.lines()
        os.write(ams_flash_pty.master, b'{"a":1}\r\n')
        assert next(lines) == '{"a":1}'                  # the carriage return is dropped
        os.write(ams_flash_pty.master, b'{"b"')
        assert next(lines) is None                       # half a line: wait for the rest
        assert link.buf == b'{"b"'
        os.write(ams_flash_pty.master, b':2}\n{"c":3}\n')
        assert next(lines) == '{"b":2}'
        assert next(lines) == '{"c":3}'                  # two lines from one read
        assert link.buf == b""
        link.close()


class TestOpenLink:
    """open_link: the bridge link Klipper let go of, tried again while the port comes free."""

    PORT = "/dev/made-up-bridge"

    @pytest.fixture
    def opener(self, monkeypatch: pytest.MonkeyPatch) -> Dict[str, Any]:
        """Link stood in for: each try is kept, and fails while `failures` lasts."""
        seen: Dict[str, Any] = {"clock": _AmsFlashClock(), "tried": [], "failures": []}

        def link(port: str) -> Tuple[str, str]:
            seen["tried"].append(port)
            if seen["failures"]:
                raise seen["failures"].pop(0)
            return ("link to", port)

        monkeypatch.setattr(ams_flash, "time", seen["clock"])
        monkeypatch.setattr(ams_flash, "Link", link)
        return seen

    def test_a_free_port_opens_at_once(self, opener: Dict[str, Any]) -> None:
        assert ams_flash.open_link(self.PORT) == ("link to", self.PORT)
        assert opener["tried"] == [self.PORT]
        assert opener["clock"].slept == []

    def test_a_busy_port_is_tried_again_each_second(self, opener: Dict[str, Any]) -> None:
        opener["failures"] += [OSError("busy"), OSError("busy")]
        assert ams_flash.open_link(self.PORT) == ("link to", self.PORT)
        assert opener["tried"] == [self.PORT] * 3
        assert opener["clock"].slept == [1.0, 1.0]

    def test_a_port_that_never_comes_free_raises_its_last_error(
            self, opener: Dict[str, Any]) -> None:
        opener["failures"] += [OSError(f"busy {i}") for i in range(3)]
        with pytest.raises(OSError) as raised:
            ams_flash.open_link(self.PORT, tries=3)
        assert str(raised.value) == "busy 2"
        assert opener["tried"] == [self.PORT] * 3
        assert opener["clock"].slept == [1.0, 1.0]

    def test_no_tries_open_nothing(self, opener: Dict[str, Any]) -> None:
        with pytest.raises(OSError) as raised:
            ams_flash.open_link(self.PORT, tries=0)
        assert str(raised.value) == "could not open /dev/made-up-bridge"
        assert opener["tried"] == []


class TestRxOf:
    """_rx_of: the captured bytes in a raw or txsend reply."""

    def test_a_reply_with_a_lost_byte_is_no_answer_not_a_crash(self) -> None:
        assert ams_flash._rx_of('{"evt":"txsend","rx":"ABC"}') == b""
        assert ams_flash._rx_of('{"evt":"txsend","rx":"ABCD"}') == b"\xab\xcd"

    def test_a_reply_without_captured_bytes_is_no_answer(self) -> None:
        assert ams_flash._rx_of('{"evt":"txsend","crc":"bad"}') == b""
        assert ams_flash._rx_of(None) == b""


class TestSendFrame:
    """send_frame: stage a frame over txbuf lines, send it, return the reply."""

    def test_a_damaged_staged_frame_is_staged_again(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        # fw >= 1.75 answers "crc":"bad" and sends nothing when a txbuf line was mangled on the way.
        txsends: List[Dict[str, Any]] = []

        def answer(obj: Dict[str, Any]) -> List[str]:
            if obj["cmd"] == "txbuf":
                return ['{"evt":"ack","cmd":"txbuf"}']
            txsends.append(obj)
            if len(txsends) == 1:
                return ['{"evt":"txsend","crc":"bad"}']
            return ['{"evt":"txsend","rx":"AABB"}']

        link = _AmsFlashScriptedLink(answer)
        assert ams_flash.send_frame(link, bytes(250)) == b"\xaa\xbb"
        staging = [{"cmd": "txbuf", "off": 0, "hex": "00" * 100},
                   {"cmd": "txbuf", "off": 100, "hex": "00" * 100},
                   {"cmd": "txbuf", "off": 200, "hex": "00" * 50},
                   {"cmd": "txsend", "n": 250, "us": 8000}]
        assert link.sent == staging * 2                  # every line staged again
        assert link.dead is False
        assert capsys.readouterr() == (
            "", "    staged frame arrived damaged -- staging it again (1/4)\n")

        link = _AmsFlashScriptedLink(
            lambda obj: ['{"evt":"ack","cmd":"txbuf"}'] if obj["cmd"] == "txbuf"
            else ['{"evt":"txsend","crc":"bad"}'])
        with pytest.raises(IOError) as raised:
            ams_flash.send_frame(link, bytes(10), stage_tries=2)
        assert str(raised.value) == (
            "staged frame still damaged after re-staging -- the link is dropping bytes")
        assert link.sent == [{"cmd": "txbuf", "off": 0, "hex": "00" * 10},
                             {"cmd": "txsend", "n": 10, "us": 8000}] * 2
        assert link.dead is False
        assert capsys.readouterr() == (
            "", "    staged frame arrived damaged -- staging it again (1/2)\n"
                "    staged frame arrived damaged -- staging it again (2/2)\n")


#: An HT's loader answering cmd1, and the same frame as if it came from the printer.
HT_LOADER = _loader_frame(0x1800, 0x80)
HT_LOADER_FROM_US = _loader_frame(0x1800, 0x80, source=0x0900)


class TestLoaderHits:
    """loader_hits: the loader's own answer in a capture window."""

    @pytest.mark.parametrize("target,ams_id", [(0x0700, 0x00), (0x1800, 0x80)])
    def test_the_gate_ignores_our_own_cmd1(self, target: int, ams_id: int) -> None:
        assert ams_flash.loader_hits(ams_flash.build_cmd1(target, ams_id).hex()) == []

    def test_the_gate_recognizes_a_units_loader_frame(self) -> None:
        assert ams_flash.loader_hits(HT_LOADER.hex()) == ["ams-origin/op0601"]
        # The same frame from us (source 0x0900): only the source rules it out.
        assert ams_flash.loader_hits(HT_LOADER_FROM_US.hex()) == []

    def test_the_loader_gate_needs_a_whole_frame(self) -> None:
        # A damaged echo, a 06 01 inside another frame's payload, and a match on half a byte all
        # used to read as "the loader answered".
        cmd1 = bytearray(ams_flash.build_cmd1(0x0700, 0x00))
        cmd1[10] = 0x0B                                  # source byte hit by noise
        assert ams_flash.loader_hits(bytes(cmd1).hex()) == []
        assert ams_flash.loader_hits("3d00aa0030601f00") == []
        assert ams_flash.loader_hits("3d05a0601b") == []
        assert ams_flash.loader_hits(
            ams_flash.build_cmd1(0x0700, 0x00).hex() + HT_LOADER.hex()) == ["ams-origin/op0601"]

    def test_another_ops_answer_is_not_the_loader(self) -> None:
        answer = _version_frame(0x0700, 0x01, "01.02.03.04", "N3F05")
        assert ams_flash.loader_hits(answer.hex()) == []

    def test_the_loaders_own_text_counts_on_a_byte_boundary(self) -> None:
        banner = b"[MCU_UP] Loader Version: made-up".hex()
        assert ams_flash.loader_hits(banner) == ["[MCU_UP]", "Loader Version"]
        assert ams_flash.loader_hits(b"[MCU_UP] ams 128 wait cmd1!".hex().upper()) == [
            "[MCU_UP]", "wait cmd1"]
        # Shifted by half a byte, the same hex digits are no longer that text.
        assert ams_flash.loader_hits("0" + banner + "0") == []

    def test_a_capture_that_is_not_hex_holds_no_frame(self) -> None:
        assert ams_flash.loader_hits("zz" + HT_LOADER.hex()) == []


class TestBuildVersionQuery:
    """build_version_query: the printer's version query (op 03 01) to one unit."""

    @pytest.mark.parametrize("unit_id,dev,counter", [(0x01, 0x0700, 0x0000),
                                                     (0x81, 0x1800, 0x0102)],
                             ids=["boxed", "ht"])
    def test_the_query_goes_to_the_units_device(self, unit_id: int, dev: int,
                                                counter: int) -> None:
        assert ams_flash.build_version_query(unit_id, counter) == _version_query(dev, unit_id,
                                                                                 counter)

    def test_the_counter_is_0_unless_given(self) -> None:
        assert ams_flash.build_version_query(0x02) == _version_query(0x0700, 0x02, 0)


class TestVersionAnswer:
    """version_answer: a unit's own answer to the version query in a capture."""

    #: Our own query, which the bridge hears too.
    QUERY = _version_query(0x0700, 0x01, 0)

    @staticmethod
    def _one_byte_short(unit_id: int) -> bytes:
        """A whole answer frame one byte short, whose byte at [33] is still `unit_id`."""
        body = _version_frame(0x0700, unit_id, "01.02.03.04", "N3F05")[7:-3]
        for counter in range(0x10000):
            frame = _frame(body, counter)
            if frame[33] == unit_id:
                return frame
        raise AssertionError("no counter puts the id at [33]")

    def test_the_units_own_answer_gives_its_version_and_model_code(self) -> None:
        raw = self.QUERY + _version_frame(0x0700, 0x01, "01.02.03.04", "N3F05")
        assert ams_flash.version_answer(raw, 0x01) == ("01.02.03.04", "N3F05")

    def test_an_hts_answer_comes_from_its_own_device(self) -> None:
        raw = _version_frame(0x1800, 0x80, "12.34.56.78", "N3S05")
        assert ams_flash.version_answer(raw, 0x80) == ("12.34.56.78", "N3S05")

    def test_a_model_code_that_fills_its_field_is_read_whole(self) -> None:
        raw = _version_frame(0x0700, 0x01, "01.02.03.04", "ABCDEFGHIJKLMNOP")
        assert ams_flash.version_answer(raw, 0x01) == ("01.02.03.04", "ABCDEFGHIJKLMNOP")

    @pytest.mark.parametrize("raw", [
        QUERY,
        _version_frame(0x0700, 0x01, "01.02.03.04", "N3F05", op=b"\x03\x02"),
        _version_frame(0x1800, 0x01, "01.02.03.04", "N3F05"),
        _version_frame(0x0700, 0x02, "01.02.03.04", "N3F05"),
    ], ids=["our-query-only", "another-op", "another-device", "another-unit"])
    def test_anything_but_the_units_own_answer_is_none(self, raw: bytes) -> None:
        assert ams_flash.version_answer(raw, 0x01) is None

    def test_an_answer_one_byte_short_is_none(self) -> None:
        frame = self._one_byte_short(0x01)
        assert (len(frame), frame[11:13], frame[9:11], frame[33]) == (35, b"\x03\x01",
                                                                      b"\x00\x07", 0x01)
        assert ams_flash.version_answer(frame, 0x01) is None

    def test_a_damaged_answer_is_none(self) -> None:
        frame = bytearray(_version_frame(0x0700, 0x01, "01.02.03.04", "N3F05"))
        frame[20] ^= 0xFF
        assert ams_flash.version_answer(bytes(frame), 0x01) is None


class TestReadVersion:
    """read_version: ask one unit its firmware version and model code."""

    @staticmethod
    def _unit(answer_from: Optional[int]) -> Callable[[Dict[str, Any]], List[str]]:
        """A bridge in front of a boxed unit at id 1 that answers from query `answer_from` on."""
        asked: List[str] = []

        def answer(obj: Dict[str, Any]) -> List[str]:
            asked.append(obj["hex"])
            rx = bytes.fromhex(obj["hex"])               # the bridge hears its own frame
            if answer_from is not None and len(asked) > answer_from:
                rx += _version_frame(0x0700, 0x01, "01.02.03.04", "N3F05")
            return [json.dumps({"evt": "raw", "rx": rx.hex().upper()})]
        return answer

    @staticmethod
    def _asked(count: int) -> List[Dict[str, Any]]:
        return [{"cmd": "raw", "hex": _version_query(0x0700, 0x01, i).hex(), "us": 100000}
                for i in range(count)]

    def test_the_first_answer_is_the_version(self) -> None:
        link = _AmsFlashScriptedLink(self._unit(0))
        assert ams_flash.read_version(link, 0x01) == ("01.02.03.04", "N3F05")
        assert link.sent == self._asked(1)

    def test_the_query_is_asked_again_until_the_unit_answers(self) -> None:
        link = _AmsFlashScriptedLink(self._unit(2))
        assert ams_flash.read_version(link, 0x01) == ("01.02.03.04", "N3F05")
        assert link.sent == self._asked(3)

    def test_a_unit_that_never_answers_has_no_version(self) -> None:
        link = _AmsFlashScriptedLink(self._unit(None))
        assert ams_flash.read_version(link, 0x01, tries=2) is None
        assert link.sent == self._asked(2)

    def test_a_bridge_that_does_not_answer_raw_is_an_error(
            self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(ams_flash, "time", _AmsFlashClock(step=1.0))
        link = _AmsFlashScriptedLink(lambda obj: [])
        with pytest.raises(IOError) as raised:
            ams_flash.read_version(link, 0x01)
        assert str(raised.value) == "bridge did not answer raw"
        assert link.sent == self._asked(1)


class TestWireOnline:
    """wire_online: the chain indices the bridge itself sees answering."""

    def test_the_bus_count_comes_from_the_bridge_itself(self) -> None:
        status = ('{"evt":"status","online":true,"units":[{"n":4,"online":true},'
                  '{"n":1,"online":false},{"n":0,"online":true}]}')
        link = _AmsFlashScriptedLink(lambda obj: [status])
        assert ams_flash.wire_online(link) == [0, 4]
        assert link.sent == [{"cmd": "status"}]

    def test_a_damaged_status_line_is_never_half_read(self) -> None:
        bad = ['{"evt":"status","units":[{"n":0,"online":true},{"n":1,"onl']
        link = _AmsFlashScriptedLink(lambda obj: bad)
        with pytest.raises(IOError) as raised:
            ams_flash.wire_online(link)
        assert str(raised.value) == "could not read the unit list from the bridge"
        assert link.sent == [{"cmd": "status"}] * 3


class TestLoaderOnBus:
    """loader_on_bus: whether some unit on the bus sits in its bootloader."""

    SNIFF = [{"cmd": "sniff", "on": 1}, {"cmd": "sniff", "on": 0}]

    @staticmethod
    def _bus(heard: str) -> Callable[[Dict[str, Any]], List[str]]:
        """A bridge whose sniff hears `heard` (hex, empty for nothing) once."""
        def answer(obj: Dict[str, Any]) -> List[str]:
            on = "true" if obj["on"] else "false"
            out = [f'{{"evt":"sniff_mode","on":{on}}}']
            if obj["on"] and heard:
                out.append(f'{{"evt":"sniff","us":1,"n":22,"hex":"{heard}"}}')
            return out
        return answer

    def test_a_unit_in_its_loader_is_heard_on_the_bus(self) -> None:
        link = _AmsFlashScriptedLink(self._bus(HT_LOADER.hex().upper()))
        assert ams_flash.loader_on_bus(link) is True
        assert link.sent == self.SNIFF                   # always switched off

    def test_a_quiet_bus_has_no_loader(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(ams_flash, "time", _AmsFlashClock(step=1.0))
        link = _AmsFlashScriptedLink(self._bus(""))
        assert ams_flash.loader_on_bus(link) is False
        assert link.sent == self.SNIFF

    def test_the_resumed_units_own_loader_does_not_count(
            self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(ams_flash, "time", _AmsFlashClock(step=1.0))
        link = _AmsFlashScriptedLink(self._bus(HT_LOADER.hex().upper()))
        assert ams_flash.loader_on_bus(link, exclude=(0x1800, 0x80)) is False
        assert link.sent == self.SNIFF

    def test_another_units_loader_counts_while_one_is_resumed(self) -> None:
        link = _AmsFlashScriptedLink(self._bus(HT_LOADER.hex().upper()))
        assert ams_flash.loader_on_bus(link, exclude=(0x0700, 0x00)) is True
        assert link.sent == self.SNIFF


class TestLoaderAnswer:
    """loader_answer: the addressed unit's loader answering cmd1."""

    def test_the_loader_answer_must_come_from_the_addressed_unit(self) -> None:
        assert ams_flash.loader_answer(HT_LOADER, 0x1800, 0x80) is True
        assert ams_flash.loader_answer(HT_LOADER, 0x0700, 0x00) is False   # another unit's
        assert ams_flash.loader_answer(
            ams_flash.build_cmd1(0x1800, 0x80), 0x1800, 0x80) is False
        banner = b"[MCU_UP] Loader Version: made-up"
        assert ams_flash.loader_answer(banner, 0x0700, 0x00) is False  # names no unit
        assert ams_flash.loader_answer(b"..[MCU_UP] 0 resev cmd 0x1..", 0x0700, 0x00) is True
        assert ams_flash.loader_answer(b"[MCU_UP] ams 128 wait cmd1!", 0x1800, 0x80) is True
        assert ams_flash.loader_answer(
            b"[MCU_UP] 128 send cmd 0x1 back", 0x1800, 0x80) is True
        assert ams_flash.loader_answer(b"[MCU_UP] 128 resev cmd 0x1", 0x0700, 0x00) is False
        assert ams_flash.loader_answer(b"[MCU_UP] 10 resev cmd 0x1", 0x0700, 0x00) is False

    @pytest.mark.parametrize("dev,ams_id", [(0x1800, 0x00), (0x0700, 0x80)],
                             ids=["right-device-wrong-id", "right-id-wrong-device"])
    def test_half_the_address_is_another_unit(self, dev: int, ams_id: int) -> None:
        assert ams_flash.loader_answer(HT_LOADER, dev, ams_id) is False


class TestDoFlash:
    """do_flash: whole-flash passes until the loader verifies the image."""

    # The loader's answers to one good pass over a two-block update.
    GOOD_PASS = [_loader_frame(0x0700, 0x00), b"\x06", b"\x06", b"success!"]
    GOOD_PASS_SENDS = ["fwreplay", "txbuf", "txsend", "txbuf", "txbuf", "txsend",
                       "txbuf", "txsend", "txbuf", "txsend", "fwreplay"]
    OPENING = ("    up to 7 attempt(s); only a loader 'success!' ends it.\n"
               "    == flash attempt 1/7 ==\n")
    PASSED = ("    loader confirmed after cmd1 #1\n"
              "    header sent, erase triggered; streaming blocks...\n"
              "    loader: success! -- image verified, resetting into the app\n"
              "    FLASH CONFIRMED: the loader verified the image and reset into the "
              "new firmware.\n")

    @staticmethod
    def _loader(replies: List[bytes],
                events: List[str]) -> Callable[[Dict[str, Any]], List[str]]:
        """
        A bridge in front of a unit's loader: it acks every staged line and answers each txsend with
        the next of `replies`.
        """
        def answer(obj: Dict[str, Any]) -> List[str]:
            events.append(obj["cmd"])
            if obj["cmd"] == "fwreplay":
                on = "true" if obj["on"] else "false"
                return [f'{{"evt":"fwreplay","on":{on}}}']
            if obj["cmd"] == "txbuf":
                return ['{"evt":"ack","cmd":"txbuf"}']
            return [f'{{"evt":"txsend","rx":"{replies.pop(0).hex().upper()}"}}']
        return answer

    def test_a_dead_link_is_reopened_before_the_next_pass(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        link = _AmsFlashScriptedLink(self._loader(list(self.GOOD_PASS), events))
        link.dead = True

        def reconnect() -> None:
            events.append("reconnect")
            link.dead = False

        art = _tiny_art(0x0700, 0x00)
        assert ams_flash.do_flash(link, art, reconnect=reconnect) == 0
        assert events == ["reconnect"] + self.GOOD_PASS_SENDS
        assert link.dead is False
        assert capsys.readouterr() == (
            self.OPENING + "    link reopened\n" + self.PASSED, "")

    def test_a_live_link_is_not_reopened(self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        link = _AmsFlashScriptedLink(self._loader(list(self.GOOD_PASS), events))
        art = _tiny_art(0x0700, 0x00)
        assert ams_flash.do_flash(
            link, art, reconnect=lambda: events.append("reconnect")) == 0
        assert events == self.GOOD_PASS_SENDS
        assert capsys.readouterr() == (self.OPENING + self.PASSED, "")

    def test_a_dead_link_with_no_way_back_is_used_as_it_is(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        link = _AmsFlashScriptedLink(self._loader(list(self.GOOD_PASS), events))
        link.dead = True
        assert ams_flash.do_flash(link, _tiny_art(0x0700, 0x00)) == 0
        assert events == self.GOOD_PASS_SENDS
        assert link.dead is True
        assert capsys.readouterr() == (self.OPENING + self.PASSED, "")

    def test_a_link_that_does_not_come_back_ends_the_flash(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        link = _AmsFlashScriptedLink(self._loader(list(self.GOOD_PASS), events))
        link.dead = True

        def reconnect() -> None:
            events.append("reconnect")
            raise IOError("port gone")

        assert ams_flash.do_flash(link, _tiny_art(0x0700, 0x00), reconnect=reconnect) == 1
        assert events == ["reconnect"]                   # no pass on a dead link
        assert link.dead is True
        assert capsys.readouterr() == (
            self.OPENING,
            "    the bridge link is gone and did not come back (port gone)\n"
            "    out of attempts: the loader never verified success!. The unit is left "
            "in its loader (recoverable) -- do not power it off; run the same command "
            "with MODE=go again. If every pass acks but reports chunk hash error, the "
            "bus/link is dropping bytes.\n")

    def test_an_image_the_loader_refuses_is_not_sent_again(
            self, capsys: pytest.CaptureFixture[str]) -> None:
        events: List[str] = []
        replies = [_loader_frame(0x0700, 0x00), b"\x06", b"\x06", b"img hash error"]
        link = _AmsFlashScriptedLink(self._loader(replies, events))
        assert ams_flash.do_flash(link, _tiny_art(0x0700, 0x00)) == 3
        assert events == self.GOOD_PASS_SENDS            # one pass: no second erase
        assert capsys.readouterr() == (
            self.OPENING + "    loader confirmed after cmd1 #1\n"
                           "    header sent, erase triggered; streaming blocks...\n",
            "    loader: IMG HASH ERROR after all blocks were accepted -- img hash error\n"
            "    the loader accepted every chunk and rejected the assembled image: this image is "
            "wrong for this unit (model or build), not the link. Stopping rather than erasing "
            "again. The unit is left in its loader (recoverable): do not power it off; run the "
            "update again with the right image for this model.\n")


#: A Moonraker that is not there, by a name that cannot resolve.
MOON_BASE = "http://moonraker.invalid:7125"


class TestMoonQ:
    """Moon.q: one Klipper object's status."""

    @staticmethod
    def _moonraker(monkeypatch: pytest.MonkeyPatch,
                   status: Dict[str, Any]) -> List[Tuple[str, str, float]]:
        asked: List[Tuple[str, str, float]] = []

        def urlopen(req: urllib.request.Request, timeout: float) -> _Response:
            asked.append((req.get_method(), req.full_url, timeout))
            return _Response(json.dumps({"result": {"status": status}}).encode())

        monkeypatch.setattr(urllib.request, "urlopen", urlopen)
        return asked

    def test_an_objects_status_is_read(self, monkeypatch: pytest.MonkeyPatch) -> None:
        asked = self._moonraker(monkeypatch,
                                {"AFC_BambuAMS Bambu_AMS_1": {"bridge_online": True}})
        assert ams_flash.Moon(MOON_BASE).q("AFC_BambuAMS Bambu_AMS_1") == {"bridge_online": True}
        assert asked == [("GET", f"{MOON_BASE}/printer/objects/query?AFC_BambuAMS%20Bambu_AMS_1",
                          30)]

    def test_an_object_klipper_does_not_report_is_empty(
            self, monkeypatch: pytest.MonkeyPatch) -> None:
        self._moonraker(monkeypatch, {})
        assert ams_flash.Moon(MOON_BASE).q("webhooks") == {}


class TestMoonGcode:
    """Moon.gcode: run one g-code line in Klipper."""

    def test_a_line_is_posted_to_moonraker(self, monkeypatch: pytest.MonkeyPatch) -> None:
        asked: List[Tuple[str, str, float]] = []

        def urlopen(req: urllib.request.Request, timeout: float) -> _Response:
            asked.append((req.get_method(), req.full_url, timeout))
            return _Response(b'{"result": "ok"}')

        monkeypatch.setattr(urllib.request, "urlopen", urlopen)
        assert ams_flash.Moon(MOON_BASE).gcode(BACK) is True
        assert asked == [("POST", f"{MOON_BASE}/printer/gcode/script?script=AFC_BAMBU_FIRMWARE"
                                  "+UNIT%3DBambu_AMS_1+HOLD%3D0", 15)]

    def test_a_line_that_cannot_be_sent_is_false(self, monkeypatch: pytest.MonkeyPatch) -> None:
        asked: List[str] = []

        def urlopen(req: urllib.request.Request, timeout: float) -> _Response:
            asked.append(req.full_url)
            raise OSError("connection refused")

        monkeypatch.setattr(urllib.request, "urlopen", urlopen)
        assert ams_flash.Moon(MOON_BASE).gcode(SHOW) is False
        assert asked == [f"{MOON_BASE}/printer/gcode/script?script=AFC_BAMBU_FIRMWARE"
                         "+SHOW%3DRESULT"]


class TestMoonState:
    """Moon.state: Klipper's own state, or '?' when it cannot be read."""

    @pytest.mark.parametrize("answer,state", [({"webhooks": {"state": "ready"}}, "ready"),
                                              ({"webhooks": {}}, "?"),
                                              (OSError("refused"), "?")],
                             ids=["ready", "no-state", "unreadable"])
    def test_the_state_comes_from_webhooks(self, monkeypatch: pytest.MonkeyPatch, answer: Any,
                                           state: str) -> None:
        def urlopen(req: urllib.request.Request, timeout: float) -> _Response:
            status = _raised(answer)
            return _Response(json.dumps({"result": {"status": status}}).encode())

        monkeypatch.setattr(urllib.request, "urlopen", urlopen)
        assert ams_flash.Moon(MOON_BASE).state() == state


class TestHoldBridge:
    """hold_bridge: have Klipper lend this run a unit's bridge link, or take it back."""

    class _Gcode:
        """Moonraker's g-code endpoint, answering `ok` to every line."""

        def __init__(self, ok: bool) -> None:
            self.ok = ok
            self.scripts: List[str] = []

        def gcode(self, script: str) -> bool:
            self.scripts.append(script)
            return self.ok

    @pytest.mark.parametrize("seconds,ok", [(1800, True), (0, True), (1800, False)],
                             ids=["hold", "give-back", "not-told"])
    def test_klipper_is_told_through_its_firmware_command(self, seconds: int, ok: bool) -> None:
        moon = self._Gcode(ok)
        assert ams_flash.hold_bridge(moon, "Bambu_AMS_1", seconds) is ok
        assert moon.scripts == [f"AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 HOLD={seconds}"]


class TestBridgeKeys:
    """bridge_keys: every Bambu bridge in the live config, with its link key."""

    def test_the_bridgebox_sections_own_key_wins(self) -> None:
        settings = {"afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                                 "tcp_key": "stale"},
                    "afc_bridgebox chain1": {"serial_port": "tcp://h:8888",
                                             "tcp_key": "real"}}
        assert ams_flash.bridge_keys(settings) == {"tcp://h:8888": "real"}

    def test_a_bridgebox_section_without_a_key_keeps_the_units_key(self) -> None:
        settings = {"afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                                 "tcp_key": "k"},
                    "afc_bridgebox chain1": {"serial_port": "tcp://h:8888"}}
        assert ams_flash.bridge_keys(settings) == {"tcp://h:8888": "k"}

    def test_serial_port_auto_names_no_bridge(self) -> None:
        # A chain on auto writes auto:<chain> into the unit sections it builds.
        settings = {"afc_bridgebox chain0": {"serial_port": "auto"},
                    "afc_bambuams bambu_ams_1": {"serial_port": "auto:chain0"},
                    "afc_bridgebox chain1": {"serial_port": "/dev/made-up-1"}}
        assert ams_flash.bridge_keys(settings) == {"/dev/made-up-1": None}


class TestBridgeFromSettings:
    """bridge_from_settings: the one bridge in the live config."""

    def test_the_bridge_comes_from_the_live_config(self) -> None:
        settings = {
            "afc_bridgebox chain1": {"serial_port": "/dev/serial/by-id/usb-Pico",
                                     "tcp_key": None},
            "mcu": {"serial": "/dev/ttyACM0"},
        }
        assert ams_flash.bridge_from_settings(settings) == (
            "/dev/serial/by-id/usb-Pico", None)
        settings = {"afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                                 "tcp_key": "k"}}
        assert ams_flash.bridge_from_settings(settings) == ("tcp://h:8888", "k")
        assert ams_flash.bridge_from_settings({"mcu": {}}) == (None, None)

    def test_a_missing_key_published_as_the_string_none_is_no_key(self) -> None:
        # AFC_BridgeBox writes str(None) into the unit sections it builds, and Klipper publishes
        # that.
        settings = {"afc_bridgebox chain1": {"serial_port": "tcp://h:8888"},
                    "afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                                 "tcp_key": "None"}}
        assert ams_flash.bridge_from_settings(settings) == ("tcp://h:8888", None)

    def test_two_bridges_in_the_config_do_not_guess(self) -> None:
        settings = {"afc_bridgebox a": {"serial_port": "tcp://a:8888"},
                    "afc_bridgebox b": {"serial_port": "/dev/ttyACM1"}}
        with pytest.raises(ValueError) as raised:
            ams_flash.bridge_from_settings(settings)
        assert str(raised.value) == (
            "more than one bridge in the config (/dev/ttyACM1, tcp://a:8888); name the "
            "one to use with --target")

    def test_a_chain_on_auto_is_not_a_second_bridge(self) -> None:
        settings = {"afc_bridgebox chain0": {"serial_port": "auto"},
                    "afc_bambuams bambu_ams_1": {"serial_port": "auto:chain0"}}
        assert ams_flash.bridge_from_settings(settings) == (None, None)


class TestUnitRows:
    """unit_rows: every AFC_BambuAMS unit Klipper has, read through Moonraker."""

    class _Moonraker:
        """
        Moonraker's object list and status queries; an Exception as a unit's status is raised by the
        query.
        """

        def __init__(self, status: Dict[str, Any]) -> None:
            self.status = status

        def objects(self) -> List[str]:
            return [f"AFC_BambuAMS {name}" for name in self.status] + ["toolhead"]

        def q(self, obj: str) -> Any:
            return _raised(self.status[obj.split(" ", 1)[1]])

    def test_a_unit_that_cannot_be_read_stops_the_list(self) -> None:
        # Dropping it would hide a unit that could share the bus with the one updated.
        moon = self._Moonraker({
            "Bambu_AMS_1": {"bridge_online": True, "ams_model": "ams2", "ams_index": 0},
            "Bambu_AMS_HT_1": IOError("timed out")})
        with pytest.raises(IOError) as raised:
            ams_flash.unit_rows(moon)
        assert str(raised.value) == "timed out"
        moon.status["Bambu_AMS_HT_1"] = {}
        with pytest.raises(IOError) as raised:
            ams_flash.unit_rows(moon)
        assert str(raised.value) == "no status for AFC_BambuAMS Bambu_AMS_HT_1"

    def test_a_status_that_is_not_a_record_stops_the_list(self) -> None:
        moon = self._Moonraker({"Bambu_AMS_1": ["online"]})
        with pytest.raises(IOError) as raised:
            ams_flash.unit_rows(moon)
        assert str(raised.value) == "no status for AFC_BambuAMS Bambu_AMS_1"

    def test_each_row_is_read_as_klipper_publishes_it(self) -> None:
        moon = self._Moonraker({"Bambu_AMS_1": {
            "bridge_online": 1, "ams_model": " AMS2 ", "ams_index": "0", "bridge_fw": "AFC-2.81",
            "bridge_port": "/dev/made-up-bridge", "fw_version": "01.02.03.04",
            "fw_model": "N3F05", "drying": 1}})
        assert ams_flash.unit_rows(moon) == [
            {"name": "Bambu_AMS_1", "online": True, "model": "ams2", "index": None,
             "port": "/dev/made-up-bridge", "fw": "AFC-2.81", "version": "01.02.03.04",
             "code": "N3F05", "drying": True}]

    def test_what_klipper_has_not_read_yet_is_empty(self) -> None:
        moon = self._Moonraker({"Bambu_AMS_HT_1": {
            "bridge_online": False, "ams_model": None, "ams_index": 4, "bridge_port": "",
            "fw_version": None, "fw_model": None}})
        assert ams_flash.unit_rows(moon) == [
            {"name": "Bambu_AMS_HT_1", "online": False, "model": "", "index": 4, "port": None,
             "fw": None, "version": "", "code": "", "drying": False}]

    def test_every_unit_is_listed_whatever_its_bridge(self) -> None:
        moon = self._Moonraker({
            "Bambu_AMS_chain1_1": {"bridge_online": True, "bridge_port": "/dev/made-up-1"},
            "Bambu_AMS_chain2_1": {"bridge_online": True, "bridge_port": "/dev/made-up-2"}})
        assert [(r["name"], r["port"]) for r in ams_flash.unit_rows(moon)] == [
            ("Bambu_AMS_chain1_1", "/dev/made-up-1"), ("Bambu_AMS_chain2_1", "/dev/made-up-2")]


class TestReadState:
    """read_state: the record of a unit a previous run left in its loader."""

    def test_the_state_record_round_trips(self, tmp_path: Path,
                                          capsys: pytest.CaptureFixture[str]) -> None:
        path = str(tmp_path / "ams_flash_state.json")
        assert ams_flash.read_state(path) is None
        assert ams_flash.write_state(path, unit="Bambu_AMS_HT_1", unit_id=0x80, board="n3s_rev5",
                                     version="01.02.03.05", file=HT_FILE, when="t") is True
        assert ams_flash.read_state(path) == {"unit": "Bambu_AMS_HT_1", "unit_id": 128,
                                              "board": "n3s_rev5", "version": "01.02.03.05",
                                              "file": HT_FILE, "when": "t"}
        ams_flash.clear_state(path)
        assert ams_flash.read_state(path) is None
        (tmp_path / "ams_flash_state.json").write_text("not json")
        assert ams_flash.read_state(path) is None
        assert capsys.readouterr() == ("", "")

    @pytest.mark.parametrize("text", ['["Bambu_AMS_1"]', '{"board": "n3f_rev5"}', '{"unit": ""}'],
                             ids=["not-a-record", "no-unit", "empty-unit"])
    def test_a_record_naming_no_unit_is_no_record(self, tmp_path: Path, text: str) -> None:
        path = tmp_path / "ams_flash_state.json"
        path.write_text(text)
        assert ams_flash.read_state(str(path)) is None


class TestWriteResult:
    """write_result: how this run ended, beside the script for the Flash AMS menu."""

    def test_the_result_is_left_with_the_time(self, tmp_path: Path,
                                              monkeypatch: pytest.MonkeyPatch,
                                              capsys: pytest.CaptureFixture[str]) -> None:
        monkeypatch.setattr(ams_flash, "time", _AmsFlashClock())
        ams_flash.write_result(str(tmp_path), unit="Bambu_AMS_1", ok=True, message="done")
        assert json.loads((tmp_path / "ams_flash_result.json").read_text()) == {
            "unit": "Bambu_AMS_1", "ok": True, "message": "done", "when": "2000-01-02 03:04"}
        assert sorted(p.name for p in tmp_path.iterdir()) == ["ams_flash_result.json"]
        assert capsys.readouterr() == ("", "")

    def test_a_time_given_is_kept(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(ams_flash, "time", _AmsFlashClock())
        ams_flash.write_result(str(tmp_path), ok=False, when="1999-12-31 23:59")
        assert json.loads((tmp_path / "ams_flash_result.json").read_text()) == {
            "ok": False, "when": "1999-12-31 23:59"}

    def test_a_folder_that_cannot_take_it_is_noted(self, tmp_path: Path,
                                                   monkeypatch: pytest.MonkeyPatch,
                                                   capsys: pytest.CaptureFixture[str]) -> None:
        monkeypatch.setattr(ams_flash, "time", _AmsFlashClock())
        here = tmp_path / "gone"
        ams_flash.write_result(str(here), ok=False)
        assert capsys.readouterr() == (
            f"note: could not write ams_flash_result.json ([Errno 2] No such file or directory: "
            f"'{here}/ams_flash_result.json.tmp')\n", "")
        assert list(tmp_path.iterdir()) == []


class TestGetImage:
    """get_image: this run's image, from --file, from the catalog, or from the run resumed."""

    @pytest.fixture
    def here(self, tmp_path: Path) -> Path:
        here = tmp_path / "Bambu_AMS"
        here.mkdir()
        _catalog(here, _entry())
        return here

    @staticmethod
    def _get(here: Path, file: Optional[str] = None, version: Optional[str] = None,
             board: str = "n3f_rev5", state: Optional[Dict[str, Any]] = None,
             fetch: bool = False) -> Tuple[Optional[bytes], str, str]:
        a = argparse.Namespace(file=file, version=version)
        return ams_flash.get_image(a, str(here), board, state, fetch)

    @staticmethod
    def _two_versions(path: Path) -> bytes:
        """A package holding two versions for n3f_rev5; returns the newer image."""
        newer = _bimh("n3f_rev5-firmware-v01.02.03.07-20000101000000_product.bin.sig",
                      _payload(500, 8))
        path.write_bytes(_package({AMS2_FILE: AMS2_IMAGE,
                                   "n3f_rev5-firmware-v01.02.03.07-20000101000000_product.bin.sig":
                                   newer}))
        return newer

    def test_a_file_named_bare_is_read_beside_the_script(self, here: Path,
                                                         capsys: pytest.CaptureFixture[str]
                                                         ) -> None:
        (here / AMS2_FILE).write_bytes(AMS2_IMAGE)
        assert self._get(here, file=AMS2_FILE) == (AMS2_IMAGE, AMS2_FILE, str(here / AMS2_FILE))
        assert capsys.readouterr() == ("", "")

    def test_a_file_named_by_absolute_path_is_read_there(self, here: Path,
                                                         tmp_path: Path) -> None:
        path = tmp_path / AMS2_FILE
        path.write_bytes(AMS2_IMAGE)
        assert self._get(here, file=str(path)) == (AMS2_IMAGE, AMS2_FILE, str(path))

    def test_a_file_named_by_relative_path_is_read_from_the_working_folder(
            self, here: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        (tmp_path / "images").mkdir()
        (tmp_path / "images" / AMS2_FILE).write_bytes(AMS2_IMAGE)
        monkeypatch.chdir(tmp_path)
        name = os.path.join("images", AMS2_FILE)
        assert self._get(here, file=name) == (AMS2_IMAGE, AMS2_FILE, name)

    def test_a_missing_file_is_refused(self, here: Path) -> None:
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, file=AMS2_FILE)
        assert str(raised.value) == f"there is no file {AMS2_FILE} beside ams_flash.py"

    def test_a_package_gives_the_image_for_the_units_board(self, here: Path) -> None:
        (here / "made-up-package.zip").write_bytes(_package({AMS2_FILE: AMS2_IMAGE,
                                                             HT_FILE: HT_IMAGE}))
        assert self._get(here, file="made-up-package.zip", board="n3s_rev5") == (
            HT_IMAGE, f"made-up-package.zip: {HT_FILE}", str(here / "made-up-package.zip"))

    def test_with_a_file_only_a_version_given_picks_out_of_the_package(self,
                                                                       here: Path) -> None:
        newer = self._two_versions(here / "made-up-package.zip")
        assert self._get(here, file="made-up-package.zip", version="01.02.03.07")[0] == newer
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, file="made-up-package.zip")
        assert str(raised.value) == (
            f"made-up-package.zip: it holds 2 images for n3f_rev5 ({AMS2_FILE}, "
            f"n3f_rev5-firmware-v01.02.03.07-20000101000000_product.bin.sig)")

    def test_a_package_that_cannot_be_read_is_refused(self, here: Path) -> None:
        damaged = bytearray(_package({AMS2_FILE: AMS2_IMAGE}))
        damaged[0:4] = b"XXXX"                           # the member's own header
        (here / "made-up-package.zip").write_bytes(bytes(damaged))
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, file="made-up-package.zip")
        assert str(raised.value) == "made-up-package.zip: Bad magic number for file header"

    def test_a_file_that_cannot_be_opened_is_refused(self, here: Path,
                                                     monkeypatch: pytest.MonkeyPatch) -> None:
        (here / AMS2_FILE).write_bytes(AMS2_IMAGE)

        def unreadable(path: str, board: str, version: str) -> Tuple[bytes, str]:
            raise PermissionError(13, "Permission denied")

        monkeypatch.setattr(ams_flash, "image_from_file", unreadable)
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, file=AMS2_FILE)
        assert str(raised.value) == f"{AMS2_FILE}: [Errno 13] Permission denied"

    def test_a_resumed_run_reads_the_file_and_version_it_recorded(self, here: Path) -> None:
        path = here / "made-up-package.zip"
        newer = self._two_versions(path)
        state = {"unit": "Bambu_AMS_1", "file": str(path), "version": "01.02.03.07"}
        assert self._get(here, state=state) == (
            newer, "made-up-package.zip: n3f_rev5-firmware-v01.02.03.07-20000101000000_product"
                   ".bin.sig", str(path))

    def test_a_record_without_a_version_takes_any_for_the_board(self, here: Path) -> None:
        path = here / "made-up-package.zip"
        self._two_versions(path)
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, state={"unit": "Bambu_AMS_1", "file": str(path)})
        assert str(raised.value) == (
            f"made-up-package.zip: it holds 2 images for n3f_rev5 ({AMS2_FILE}, "
            f"n3f_rev5-firmware-v01.02.03.07-20000101000000_product.bin.sig)")

    def test_a_record_without_a_file_is_refused(self, here: Path) -> None:
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, state={"unit": "Bambu_AMS_1"})
        assert str(raised.value) == "there is no file  beside ams_flash.py"

    def test_a_version_given_wins_over_the_record(self, here: Path) -> None:
        (here / AMS2_FILE).write_bytes(AMS2_IMAGE)
        state = {"unit": "Bambu_AMS_1", "file": str(here / "made-up-missing.bin.sig")}
        assert self._get(here, version="01.02.03.04", state=state) == (
            AMS2_IMAGE, AMS2_FILE, str(here / AMS2_FILE))

    def test_no_file_and_no_version_is_refused(self, here: Path) -> None:
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here)
        assert str(raised.value) == ("say which image to write: --version (one "
                                     "ams_firmware.json lists) or --file")

    @pytest.mark.parametrize("board,said", [("n3f_rev5", "AMS 2 Pro (n3f_rev5)"),
                                            ("", "this unit")])
    def test_a_version_the_catalog_does_not_list_is_refused(self, here: Path, board: str,
                                                            said: str) -> None:
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, version="09.09.09.09", board=board)
        assert str(raised.value) == f"ams_firmware.json lists no 09.09.09.09 for {said}"

    def test_a_version_listed_for_another_board_is_refused(self, here: Path) -> None:
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, version="01.02.03.04", board="n3s_rev5")
        assert str(raised.value) == "ams_firmware.json lists no 01.02.03.04 for AMS HT (n3s_rev5)"

    def test_a_version_listed_for_two_boards_needs_the_board(self, here: Path) -> None:
        _catalog(here, _entry(), _entry(board="n3s_rev5", file=HT_FILE))
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, version="01.02.03.04", board="")
        assert str(raised.value) == (
            "01.02.03.04 comes for n3f_rev5 and n3s_rev5; read the unit's board first "
            "(AFC_BAMBU_VERSION) or name it with --board")

    def test_the_units_board_picks_between_two_boards(self, here: Path) -> None:
        _catalog(here, _entry(), _entry(board="n3s_rev5", file=HT_FILE, md5=HT_MD5))
        (here / HT_FILE).write_bytes(HT_IMAGE)
        assert self._get(here, version="01.02.03.04", board="n3s_rev5") == (
            HT_IMAGE, HT_FILE, str(here / HT_FILE))

    @pytest.mark.parametrize("md5,copy", [(AMS2_MD5, AMS2_IMAGE),
                                          (None, b"a copy with no MD5 to check")],
                             ids=["intact", "unchecked"])
    def test_a_copy_beside_the_script_is_used(self, here: Path, md5: Optional[str], copy: bytes,
                                              capsys: pytest.CaptureFixture[str]) -> None:
        _catalog(here, _entry(md5=md5))
        (here / AMS2_FILE).write_bytes(copy)
        assert self._get(here, version="01.02.03.04", fetch=True) == (
            copy, AMS2_FILE, str(here / AMS2_FILE))
        assert capsys.readouterr() == ("", "")           # nothing downloaded

    @pytest.mark.parametrize("copy", [None, b"damaged"], ids=["missing", "damaged"])
    def test_a_copy_that_is_not_intact_is_left_for_the_download(
            self, here: Path, copy: Optional[bytes], capsys: pytest.CaptureFixture[str]) -> None:
        if copy is not None:
            (here / AMS2_FILE).write_bytes(copy)
        assert self._get(here, version="01.02.03.04") == (
            None, "01.02.03.04 for AMS 2 Pro (n3f_rev5): downloaded from Bambu when the update "
                  "starts", str(here / AMS2_FILE))
        assert capsys.readouterr() == ("", "")

    def test_a_missing_copy_is_downloaded_when_asked(self, here: Path,
                                                     monkeypatch: pytest.MonkeyPatch,
                                                     capsys: pytest.CaptureFixture[str]) -> None:
        server = _Server({PACKAGE_URL: _package({AMS2_FILE: AMS2_IMAGE})})
        monkeypatch.setattr(urllib.request, "urlopen", server)
        assert self._get(here, version="01.02.03.04", fetch=True) == (
            AMS2_IMAGE, AMS2_FILE, str(here / AMS2_FILE))
        assert (here / AMS2_FILE).read_bytes() == AMS2_IMAGE
        assert server.asked[0] == (PACKAGE_URL, "bytes=0-0", "ams_flash/2", 30.0)
        assert capsys.readouterr() == (f"    downloading {AMS2_FILE} from Bambu\n", "")

    def test_a_download_that_fails_is_refused(self, here: Path, monkeypatch: pytest.MonkeyPatch,
                                              capsys: pytest.CaptureFixture[str]) -> None:
        monkeypatch.setattr(urllib.request, "urlopen",
                            lambda req, timeout: _Response(b"the whole package"))
        with pytest.raises(ams_flash.Abort) as raised:
            self._get(here, version="01.02.03.04", fetch=True)
        assert str(raised.value) == (
            "the download from Bambu failed (the server will not send part of the package; "
            "download the image (.bin.sig) yourself and put it beside ams_flash.py); to update "
            "offline, put the image beside ams_flash.py")
        assert capsys.readouterr() == (f"    downloading {AMS2_FILE} from Bambu\n", "")
        assert not (here / AMS2_FILE).exists()


class TestBridgeOf:
    """bridge_of: the port and link key of the bridge a unit is on."""

    @pytest.fixture
    def cfg(self, tmp_path: Path) -> Path:
        cfg = tmp_path / "config"
        cfg.mkdir()
        (cfg / "printer.cfg").write_text("")
        return cfg

    @pytest.fixture
    def port(self, tmp_path: Path) -> str:
        path = tmp_path / "bridge-port"
        path.write_text("")
        return str(path)

    @staticmethod
    def _of(cfg: Path, port: Optional[str], settings: Any = None, target: Optional[str] = None,
            key: Optional[str] = None) -> Tuple[str, str]:
        a = argparse.Namespace(target=target, key=key, cfg_dir=str(cfg))
        return ams_flash.bridge_of(a, {"name": "Bambu_AMS_1", "port": port}, settings)

    @staticmethod
    def _bridge(port: str, key: Optional[str] = None) -> Dict[str, Any]:
        """Live settings naming one bridge."""
        return {"afc_bridgebox chain1": {"serial_port": port, "tcp_key": key}}

    def test_the_units_own_serial_port_takes_no_key(self, cfg: Path, port: str) -> None:
        assert self._of(cfg, port, self._bridge(port, "made-up-key")) == (port, "")

    def test_a_target_given_wins_over_the_units_port(self, cfg: Path, port: str,
                                                     tmp_path: Path) -> None:
        other = tmp_path / "other-port"
        other.write_text("")
        assert self._of(cfg, port, {}, target=str(other)) == (str(other), "")

    def test_a_network_bridge_takes_its_key_from_the_live_config(self, cfg: Path) -> None:
        target = "tcp://bridge.invalid:8888"
        assert self._of(cfg, target, self._bridge(target, "made-up-key")) == (target,
                                                                              "made-up-key")

    @pytest.mark.parametrize("key", ["", "another-made-up-key"])
    def test_a_key_given_wins(self, cfg: Path, key: str) -> None:
        target = "tcp://bridge.invalid:8888"
        assert self._of(cfg, target, self._bridge(target, "made-up-key"), key=key) == (target,
                                                                                       key)

    @pytest.mark.parametrize("settings", [None, {}], ids=["no-live-config", "no-key-there"])
    def test_otherwise_the_key_comes_from_the_config_files(self, cfg: Path,
                                                           settings: Any) -> None:
        (cfg / "bridge.cfg").write_text("tcp_key: made-up-key  # made up\n")
        target = "tcp://bridge.invalid:8888"
        assert self._of(cfg, target, settings) == (target, "made-up-key")

    @pytest.mark.parametrize("text", ["tcp_key: None\n", ""], ids=["none", "no-key"])
    def test_a_bridge_without_a_key_has_an_empty_one(self, cfg: Path, text: str) -> None:
        (cfg / "bridge.cfg").write_text(text)
        target = "tcp://bridge.invalid:8888"
        assert self._of(cfg, target, None) == (target, "")

    @pytest.mark.parametrize("unit_port", [None, "made-up-port"], ids=["none", "not-a-path"])
    def test_a_unit_without_a_usable_port_uses_the_one_bridge_in_the_config(
            self, cfg: Path, port: str, unit_port: Optional[str]) -> None:
        assert self._of(cfg, unit_port, self._bridge(port)) == (port, "")

    def test_two_bridges_in_the_config_are_not_guessed_between(self, cfg: Path) -> None:
        settings = {"afc_bridgebox a": {"serial_port": "tcp://bridge-a.invalid:8888"},
                    "afc_bridgebox b": {"serial_port": "/dev/made-up-b"}}
        with pytest.raises(ams_flash.Abort) as raised:
            self._of(cfg, None, settings)
        assert str(raised.value) == (
            "more than one bridge in the config (/dev/made-up-b, tcp://bridge-a.invalid:8888); "
            "name the one to use with --target")

    @pytest.mark.parametrize("unit_port", ["auto:chain0", "AUTO"], ids=["auto-key", "auto"])
    def test_a_unit_on_auto_is_never_guessed_from_the_config(self, cfg: Path, port: str,
                                                             unit_port: str) -> None:
        # A config file can name an old board, or another chain's.
        (cfg / "old.cfg").write_text(f"serial_port: {port}\n")
        settings = {"afc_bridgebox chain0": {"serial_port": "auto"},
                    "afc_bambuams bambu_ams_1": {"serial_port": "auto:chain0"}}
        with pytest.raises(ams_flash.Abort) as raised:
            self._of(cfg, unit_port, settings)
        assert str(raised.value) == (
            "Bambu_AMS_1's bridge is on serial_port: auto, and Klipper names no USB port for "
            "it; name the port with --target")

    def test_a_target_given_wins_over_auto(self, cfg: Path, port: str) -> None:
        assert self._of(cfg, "auto:chain0", {}, target=port) == (port, "")

    @pytest.mark.parametrize("settings", [None, {}], ids=["no-live-config", "no-bridge-there"])
    def test_without_a_live_bridge_the_port_comes_from_the_config_files(
            self, cfg: Path, port: str, settings: Any) -> None:
        (cfg / "bridge.cfg").write_text(f"serial_port: {port}  ; made up\n")
        assert self._of(cfg, None, settings) == (port, "")

    def test_no_bridge_anywhere_is_refused(self, cfg: Path) -> None:
        with pytest.raises(ams_flash.Abort) as raised:
            self._of(cfg, None, {})
        assert str(raised.value) == "no bridge found for Bambu_AMS_1; name it with --target"

    def test_a_serial_port_that_is_gone_is_refused(self, cfg: Path, tmp_path: Path) -> None:
        gone = str(tmp_path / "gone")
        with pytest.raises(ams_flash.Abort) as raised:
            self._of(cfg, gone, {})
        assert str(raised.value) == (f"bridge port {gone} does not exist; name the port with "
                                     f"--target")

    @staticmethod
    def _wifi_state(usb_port: str, addr: str = "tcp://bridge.invalid:8888") -> str:
        """A chain on connection: wifi as its #~# block saves it."""
        return ("[AFC_BridgeBox Chain_1]\nserial_port: auto\nconnection: wifi\n\n"
                f"{ams_flash.STATE_MARK}\n#~# [AFC_BridgeBox Chain_1]\n"
                "#~# wifi_key = made-up-key\n#~# lane_base : 12\n"
                f"#~# usb_port : {usb_port}\n#~# wifi_addr : {addr}\n#~# \n")

    def test_a_wifi_bridge_off_usb_is_reached_at_its_saved_address(self, cfg: Path,
                                                                   tmp_path: Path) -> None:
        gone = str(tmp_path / "gone")
        (cfg / "AFC_BridgeBox_1.cfg").write_text(self._wifi_state(gone))
        assert self._of(cfg, gone, {}) == ("tcp://bridge.invalid:8888", "made-up-key")

    def test_a_wifi_bridge_in_a_state_file_is_found_too(self, cfg: Path, tmp_path: Path) -> None:
        gone = str(tmp_path / "gone")
        state = tmp_path / "bridge.state"
        state.write_text(self._wifi_state(gone))
        settings = {"afc_bridgebox chain_1": {"serial_port": "auto", "state_file": str(state)}}
        assert self._of(cfg, gone, settings) == ("tcp://bridge.invalid:8888", "made-up-key")

    def test_a_key_given_wins_over_the_saved_one(self, cfg: Path, tmp_path: Path) -> None:
        gone = str(tmp_path / "gone")
        (cfg / "AFC_BridgeBox_1.cfg").write_text(self._wifi_state(gone))
        assert self._of(cfg, gone, {}, key="") == ("tcp://bridge.invalid:8888", "")

    def test_a_wifi_bridge_on_usb_keeps_its_usb_port(self, cfg: Path, port: str) -> None:
        (cfg / "AFC_BridgeBox_1.cfg").write_text(self._wifi_state(port))
        assert self._of(cfg, port, {}) == (port, "")

    def test_another_bridges_saved_address_is_not_taken(self, cfg: Path, tmp_path: Path) -> None:
        gone = str(tmp_path / "gone")
        (cfg / "AFC_BridgeBox_1.cfg").write_text(self._wifi_state(str(tmp_path / "other")))
        with pytest.raises(ams_flash.Abort, match="does not exist"):
            self._of(cfg, gone, {})

    def test_a_target_given_is_not_swapped_for_a_saved_address(self, cfg: Path,
                                                              tmp_path: Path) -> None:
        gone = str(tmp_path / "gone")
        (cfg / "AFC_BridgeBox_1.cfg").write_text(self._wifi_state(gone))
        with pytest.raises(ams_flash.Abort, match="does not exist"):
            self._of(cfg, None, {}, target=gone)

    @pytest.mark.parametrize("target", ["tcp://bridge.invalid", "tcp://bridge.invalid:1",
                                        "tcp://bridge.invalid:65535"],
                             ids=["default-port", "lowest", "highest"])
    def test_a_network_bridge_address_is_checked(self, cfg: Path, target: str) -> None:
        assert self._of(cfg, target, {}) == (target, "")

    @pytest.mark.parametrize("target", ["tcp://:8888", "tcp://bridge.invalid:0",
                                        "tcp://bridge.invalid:65536",
                                        "tcp://bridge.invalid:port"],
                             ids=["no-host", "port-0", "port-too-high", "port-not-a-number"])
    def test_a_network_bridge_address_that_is_not_one_is_refused(self, cfg: Path,
                                                                 target: str) -> None:
        with pytest.raises(ams_flash.Abort) as raised:
            self._of(cfg, target, {})
        assert str(raised.value) == f"bridge target {target} is not tcp://host:port"


class TestUpdate:
    """update: every check, then the update itself, on a made-up printer."""

    def test_a_unit_takes_the_image_and_answers_with_it(self, rig: _Rig) -> None:
        assert rig.update(rig.args()) == ("rc", 0)
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.flashed(), "")
        assert rig.result == {"unit": "Bambu_AMS_1", "ok": True, "from": "01.02.03.00",
                              "to": "01.02.03.04",
                              "message": "Bambu_AMS_1 now runs 01.02.03.04 (was 01.02.03.00)."}
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]
        assert rig.opened == [rig.port]
        assert rig.bus.log == CHECKED_BUS + FLASHED_BUS
        assert rig.bus.record_at_cmd1 == rig.record()    # recorded before cmd1
        assert rig.state() is None                       # and dropped once it took
        [art] = rig.flashed
        assert (art["header"][:2], art["header"][12], art["header"][32:]) == (
            b"\x00\x07", 0x00, AMS2_IMAGE[:0x80])
        assert len(art["blocks"]) == 3
        assert rig.clock.slept == []
        assert (rig.root / "logs" / "ams_flash.lock").is_file()
        assert rig.saved_result() is None                # main() leaves the result

    def test_an_ht_takes_its_image_out_of_a_package(self, rig: _Rig) -> None:
        rig.moon.units = {"Bambu_AMS_HT_1": _klipper_unit(rig.port, ams_model="ht", ams_index=4,
                                                          fw_model="N3S05")}
        rig.bus.units = [_Unit(4, 0x80, 0x1800, "01.02.03.00", "N3S05")]
        rig.new_version = "01.02.03.05"
        (rig.here / "made-up-package.zip").write_bytes(_package({AMS2_FILE: AMS2_IMAGE,
                                                                 HT_FILE: HT_IMAGE}))
        a = rig.args(unit="Bambu_AMS_HT_1", version=None, file="made-up-package.zip")
        assert rig.update(a) == ("rc", 0)
        row = ("    Bambu_AMS_HT_1: online  ams_model=ht  index=4  bus id 0x80  firmware "
               "01.02.03.00 (N3S05)\n")
        image = (f"[4] the image\n    made-up-package.zip: {HT_FILE}\n    {HT_FILE}: AMS HT "
                 f"(n3s_rev5) 01.02.03.05, 2628 bytes, MD5 {HT_MD5}\n")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=row, image=image) + _Said.BUS
            + "    answering on the bus: index 4 (bus id 0x80)\n"
            + "    Bambu_AMS_HT_1: firmware 01.02.03.00, model code N3S05\n" + _Said.LOADER
            + "[8] the unit boots its new firmware\n"
            + "    Bambu_AMS_HT_1: firmware 01.02.03.05, model code N3S05\n"
            + "\n==> DONE. Bambu_AMS_HT_1 now runs 01.02.03.05 (was 01.02.03.00).\n", "")
        assert rig.result == {"unit": "Bambu_AMS_HT_1", "ok": True, "from": "01.02.03.00",
                              "to": "01.02.03.05",
                              "message": "Bambu_AMS_HT_1 now runs 01.02.03.05 (was 01.02.03.00)."}
        assert rig.moon.scripts == [
            'RESPOND MSG="AMS update: Bambu_AMS_HT_1 to 01.02.03.05. The units on its bridge '
            'read offline until it is done."',
            "AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_HT_1 HOLD=1800",
            "AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_HT_1 HOLD=0"]
        assert rig.bus.log == ["open", "fwreplay off", "sniff off", "status", "version? 0x80",
                               "sniff on", "sniff off", "status", "cmd1 0x80", "flash", "status",
                               "version? 0x80"] + RELEASED
        assert rig.bus.record_at_cmd1 == {"unit": "Bambu_AMS_HT_1", "unit_id": 128,
                                          "board": "n3s_rev5", "version": "01.02.03.05",
                                          "file": str(rig.here / "made-up-package.zip"),
                                          "when": "2000-01-02 03:04"}
        [art] = rig.flashed
        assert (art["header"][:2], art["header"][12]) == (b"\x00\x18", 0x80)
        assert rig.state() is None

    def test_an_unreadable_print_state_stops_it(self, rig: _Rig) -> None:
        rig.moon.print_state = OSError("Moonraker is down")
        assert rig.update(rig.args()) == ("abort", "cannot read the print state (Moonraker is "
                                                   "down)")
        assert (rig.out, rig.err) == ("[1] printer not printing\n", "")
        assert rig.result == {"unit": "Bambu_AMS_1", "ok": False, "from": "", "to": "01.02.03.04",
                              "message": ""}
        assert rig.moon.scripts == []

    @pytest.mark.parametrize("state", ["printing", "paused"])
    def test_an_active_print_stops_it(self, rig: _Rig, state: str) -> None:
        rig.moon.print_state = state
        assert rig.update(rig.args()) == ("abort", "a print is active")
        assert (rig.out, rig.err) == (f"[1] printer not printing\n    state = {state}\n", "")
        assert rig.moon.scripts == []

    def test_units_that_cannot_be_read_stop_it(self, rig: _Rig) -> None:
        rig.moon.units["Bambu_AMS_1"] = {}
        assert rig.update(rig.args()) == ("abort", "cannot read the AMS units (no status for "
                                                   "AFC_BambuAMS Bambu_AMS_1)")
        assert (rig.out, rig.err) == (_Said.START + "[2] the unit\n", "")

    def test_no_unit_named_and_none_waiting_stops_it(self, rig: _Rig) -> None:
        assert rig.update(rig.args(unit=None)) == (
            "abort", "name the unit to update, e.g. --unit Bambu_AMS_1")
        assert (rig.out, rig.err) == (_Said.START + "[2] the unit\n", "")
        assert rig.result["unit"] == ""

    @pytest.mark.parametrize("units,said", [
        (["Bambu_AMS_1", "Bambu_AMS_HT_1"], "Bambu_AMS_1, Bambu_AMS_HT_1"), ([], "none")],
        ids=["others", "none"])
    def test_a_unit_klipper_does_not_have_is_refused(self, rig: _Rig, units: List[str],
                                                     said: str) -> None:
        rig.moon.units = {name: _klipper_unit(rig.port) for name in units}
        assert rig.update(rig.args(unit="Bambu_AMS_9")) == (
            "abort", f"there is no unit Bambu_AMS_9 (units: {said})")
        assert (rig.out, rig.err) == (_Said.START + "[2] the unit\n", "")
        assert rig.result["unit"] == "Bambu_AMS_9"

    def test_the_unit_goes_by_the_name_klipper_gives_it(self, rig: _Rig) -> None:
        assert rig.update(rig.args(unit="bambu_ams_1", mode="check")) == ("rc", 0)
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.CHECKED, "")
        assert rig.result["unit"] == "Bambu_AMS_1"

    @pytest.mark.parametrize("model,shown", [("acme", "acme"), ("", "unset")])
    def test_a_model_that_does_not_update_here_is_refused(self, rig: _Rig, model: str,
                                                          shown: str) -> None:
        rig.moon.units["Bambu_AMS_1"]["ams_model"] = model
        assert rig.update(rig.args()) == (
            "abort", f"Bambu_AMS_1 is ams_model {shown}; only an AMS 1, AMS 2 Pro or AMS HT "
                     f"updates here")
        assert (rig.out, rig.err) == (_Said.START + "[2] the unit\n", "")
        assert rig.result["unit"] == "Bambu_AMS_1"

    def test_a_unit_without_a_chain_index_is_refused(self, rig: _Rig) -> None:
        rig.moon.units["Bambu_AMS_1"]["ams_index"] = None
        assert rig.update(rig.args()) == ("abort", "Klipper has no chain index for Bambu_AMS_1")
        assert (rig.out, rig.err) == (_Said.START + "[2] the unit\n", "")

    def test_an_offline_unit_is_refused(self, rig: _Rig) -> None:
        rig.moon.units["Bambu_AMS_1"].update(bridge_online=False, fw_version="")
        assert rig.update(rig.args()) == (
            "abort", "Bambu_AMS_1 is offline. Power it, wait until it shows online, and retry.")
        assert (rig.out, rig.err) == (_Said.START + "[2] the unit\n" + _Said.OFFLINE, "")
        assert rig.moon.scripts == []

    @pytest.mark.parametrize("drying,said", [
        (["Bambu_AMS_2"], "Bambu_AMS_2 is drying"),
        (["Bambu_AMS_1", "Bambu_AMS_2"], "Bambu_AMS_1, Bambu_AMS_2 are drying")],
        ids=["one", "two"])
    def test_a_dry_on_the_same_bridge_stops_it(self, rig: _Rig, drying: List[str],
                                               said: str) -> None:
        rig.moon.units["Bambu_AMS_2"] = _klipper_unit(rig.port, ams_index=1)
        for name in drying:
            rig.moon.units[name]["drying"] = True
        assert rig.update(rig.args()) == (
            "abort", f"{said} on the same bridge; stop the dry and retry")
        assert (rig.out, rig.err) == (_Said.START + "[2] the unit\n" + _Said.ROW, "")

    def test_only_a_dry_on_the_same_bridge_counts(self, rig: _Rig) -> None:
        rig.moon.units["Bambu_AMS_2"] = _klipper_unit(str(rig.tmp_path / "other-port"),
                                                      drying=True)
        rig.moon.units["Bambu_AMS_3"] = _klipper_unit(rig.port, ams_index=1)
        assert rig.update(rig.args(mode="check")) == ("rc", 0)
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.CHECKED, "")

    def test_without_live_settings_the_bridge_comes_from_the_config_files(self,
                                                                         rig: _Rig) -> None:
        rig.moon.config = OSError("Moonraker is down")
        rig.moon.units["Bambu_AMS_1"]["bridge_port"] = None
        (rig.cfg / "bridge.cfg").write_text(f"serial_port: {rig.port}\n")
        assert rig.update(rig.args(mode="check")) == ("rc", 0)
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.CHECKED, "")

    @pytest.mark.parametrize("status,board,was,row,said", [
        ({}, "n3f_rev7", "01.02.03.00", _Said.ROW, "AMS 2 Pro (n3f_rev7)"),
        ({"fw_model": "N3F06"}, None, "01.02.03.00",
         "    Bambu_AMS_1: online  ams_model=ams2  index=0  bus id 0x00  firmware 01.02.03.00 "
         "(N3F06)\n", "AMS 2 Pro (n3f_rev6)"),
        ({"ams_model": "ht", "ams_index": 4, "fw_version": "", "fw_model": ""}, None, "",
         "    Bambu_AMS_1: online  ams_model=ht  index=4  bus id 0x80\n", "AMS HT (n3s_rev5)"),
        ({"ams_model": "ams1", "fw_version": "", "fw_model": ""}, None, "",
         "    Bambu_AMS_1: online  ams_model=ams1  index=0  bus id 0x00\n", "this unit"),
    ], ids=["given", "model-code", "model-default", "no-default"])
    def test_the_board_comes_from_the_command_then_the_code_then_the_model(
            self, rig: _Rig, status: Dict[str, Any], board: Optional[str], was: str, row: str,
            said: str) -> None:
        rig.moon.units["Bambu_AMS_1"].update(status)
        assert rig.update(rig.args(version="09.09.09.09", board=board)) == (
            "abort", f"ams_firmware.json lists no 09.09.09.09 for {said}")
        assert (rig.out, rig.err) == (_Said.checks(rig, row=row, image="[4] the image\n"), "")
        assert rig.result == {"unit": "Bambu_AMS_1", "ok": False, "from": was,
                              "to": "09.09.09.09", "message": ""}

    @pytest.mark.parametrize("board,said", [("n3f_rev9", "AMS 2 Pro (n3f_rev9)"),
                                            (None, "AMS 2 Pro (n3f_rev5)")],
                             ids=["recorded", "not-recorded"])
    def test_a_resumed_unit_keeps_the_board_it_was_recorded_with(self, rig: _Rig,
                                                                 board: Optional[str],
                                                                 said: str) -> None:
        rig.moon.units["Bambu_AMS_1"]["bridge_online"] = False
        rig.state_path.write_text(json.dumps(rig.record(when="1999-12-31 23:59", board=board)))
        assert rig.update(rig.args(version="09.09.09.09")) == (
            "abort", f"ams_firmware.json lists no 09.09.09.09 for {said}")
        row = ("    Bambu_AMS_1: offline  ams_model=ams2  index=0  bus id 0x00  firmware "
               "01.02.03.00 (N3F05)\n")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=row + _Said.RESUMING, image="[4] the image\n"), "")

    @pytest.mark.parametrize("fw,shown", [(None, "unknown"), ("AFC-1.74", "174")])
    def test_an_old_bridge_firmware_stops_it(self, rig: _Rig, fw: Optional[str],
                                             shown: str) -> None:
        rig.moon.units["Bambu_AMS_1"]["bridge_fw"] = fw
        assert rig.update(rig.args()) == (
            "abort", "the bridge firmware is older than 175; update the bridge first")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, image="").replace("bridge_fw = 281", f"bridge_fw = {shown}"), "")
        assert rig.result["from"] == "01.02.03.00"

    def test_the_oldest_bridge_firmware_that_works_passes(self, rig: _Rig) -> None:
        rig.moon.units["Bambu_AMS_1"]["bridge_fw"] = "AFC-1.75"
        assert rig.update(rig.args(mode="check")) == ("rc", 0)
        assert (rig.out, rig.err) == (
            _Said.checks(rig).replace("bridge_fw = 281", "bridge_fw = 175") + _Said.CHECKED, "")

    def test_a_file_that_is_no_image_stops_it(self, rig: _Rig) -> None:
        (rig.here / "made-up.bin.sig").write_bytes(b"not an image")
        assert rig.update(rig.args(version=None, file="made-up.bin.sig")) == (
            "abort", "made-up.bin.sig: not a Bambu AMS image (no BIMH header)")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, image="[4] the image\n    made-up.bin.sig\n"), "")

    @pytest.mark.parametrize("model", ["ams2", "boxed"])
    def test_an_image_for_another_model_is_refused(self, rig: _Rig, model: str) -> None:
        rig.moon.units["Bambu_AMS_1"].update(ams_model=model, fw_version="", fw_model="")
        (rig.here / HT_FILE).write_bytes(HT_IMAGE)
        assert rig.update(rig.args(version=None, file=HT_FILE)) == (
            "abort", f"{HT_FILE} is for an AMS HT (n3s_rev5), but Bambu_AMS_1 is set up as "
                     f"ams_model {model}")
        row = f"    Bambu_AMS_1: online  ams_model={model}  index=0  bus id 0x00\n"
        image = (f"[4] the image\n    {HT_FILE}\n    {HT_FILE}: AMS HT (n3s_rev5) 01.02.03.05, "
                 f"2628 bytes, MD5 {HT_MD5}\n")
        assert (rig.out, rig.err) == (_Said.checks(rig, row=row, image=image), "")
        assert rig.result["to"] == ""

    @pytest.mark.parametrize("file,image,version,said", [
        (AMS1_FILE, AMS1_IMAGE, "01.02.03.06",
         f"AMS 1 (ams_rev8) 01.02.03.06, 2628 bytes, MD5 {AMS1_MD5}"),
        (AMS2_FILE, AMS2_IMAGE, "01.02.03.04",
         f"AMS 2 Pro (n3f_rev5) 01.02.03.04, 2628 bytes, MD5 {AMS2_MD5}"),
    ], ids=["ams1", "ams2"])
    def test_a_boxed_unit_of_unknown_board_takes_either_boxed_model(self, rig: _Rig, file: str,
                                                                    image: bytes, version: str,
                                                                    said: str) -> None:
        rig.moon.units["Bambu_AMS_1"].update(ams_model="boxed", fw_version="", fw_model="")
        (rig.here / file).write_bytes(image)
        assert rig.update(rig.args(version=None, file=file, mode="check")) == ("rc", 0)
        row = "    Bambu_AMS_1: online  ams_model=boxed  index=0  bus id 0x00\n"
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=row, image=f"[4] the image\n    {file}\n    {file}: {said}\n")
            + _Said.CHECKED, "")
        assert rig.result["to"] == version

    def test_an_image_for_another_board_is_refused(self, rig: _Rig) -> None:
        rig.moon.units["Bambu_AMS_1"]["fw_model"] = "N3F06"
        assert rig.update(rig.args(version=None, file=AMS2_FILE)) == (
            "abort", f"{AMS2_FILE} is for AMS 2 Pro (n3f_rev5), but Bambu_AMS_1 is AMS 2 Pro "
                     f"(n3f_rev6)")
        row = ("    Bambu_AMS_1: online  ams_model=ams2  index=0  bus id 0x00  firmware "
               "01.02.03.00 (N3F06)\n")
        assert (rig.out, rig.err) == (_Said.checks(rig, row=row), "")

    def test_check_mode_changes_nothing(self, rig: _Rig) -> None:
        assert rig.update(rig.args(mode="check")) == ("rc", 0)
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.CHECKED, "")
        assert rig.result == {"unit": "Bambu_AMS_1", "ok": False, "from": "01.02.03.00",
                              "to": "01.02.03.04", "message": ""}
        assert (rig.moon.scripts, rig.bus.log, rig.relaunched) == ([], [], [])
        assert (rig.state(), rig.saved_result()) == (None, None)
        assert not (rig.root / "logs").exists()

    def test_check_mode_downloads_a_missing_image(self, rig: _Rig,
                                                  monkeypatch: pytest.MonkeyPatch) -> None:
        (rig.here / AMS2_FILE).unlink()
        server = _Server({PACKAGE_URL: _package({AMS2_FILE: AMS2_IMAGE})})
        monkeypatch.setattr(urllib.request, "urlopen", server)
        assert rig.update(rig.args(mode="check", detached=False)) == ("rc", 0)
        image = _Said.IMAGE.replace("[4] the image\n",
                                    f"[4] the image\n    downloading {AMS2_FILE} from Bambu\n")
        assert (rig.out, rig.err) == (_Said.checks(rig, image=image) + _Said.CHECKED, "")
        assert (rig.here / AMS2_FILE).read_bytes() == AMS2_IMAGE

    def test_the_update_is_started_on_its_own_and_downloads_there(self, rig: _Rig) -> None:
        (rig.here / AMS2_FILE).unlink()
        assert rig.update(rig.args(detached=False)) == ("rc", 0)
        image = ("[4] the image\n    01.02.03.04 for AMS 2 Pro (n3f_rev5): downloaded from Bambu "
                 "when the update starts\n")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, image=image) + "launched detached; progress in logs/ams_flash.log\n",
            "")
        running = {"unit": "Bambu_AMS_1", "ok": False, "from": "01.02.03.00", "to": "01.02.03.04",
                   "message": "Updating Bambu_AMS_1. The units on its bridge read offline until "
                              "it is done.", "running": True}
        assert rig.result == running
        assert rig.saved_result() == dict(running, when="2000-01-02 03:04")
        assert rig.relaunched == [str(rig.here / "ams_sudo.pw")]
        assert not (rig.here / AMS2_FILE).exists()       # the update it started downloads it
        assert (rig.moon.scripts, rig.bus.log) == ([], [])

    def test_an_update_that_cannot_start_on_its_own_says_why(self, rig: _Rig) -> None:
        rig.relaunch = (2, "could not detach without root:\n  made-up reason")
        assert rig.update(rig.args(detached=False)) == (
            "abort", "could not start the update on its own:\ncould not detach without root:\n"
                     "  made-up reason")
        assert (rig.out, rig.err) == (_Said.checks(rig), "")
        assert rig.result["running"] is False
        assert rig.saved_result()["running"] is True     # left by the launch, main() redoes it
        assert rig.relaunched == [str(rig.here / "ams_sudo.pw")]

    def test_without_a_lock_folder_it_runs_unguarded(self, rig: _Rig) -> None:
        (rig.root / "logs").write_text("")
        assert rig.update(rig.args()) == ("rc", 0)
        note = (f"note: no lock file ([Errno 17] File exists: '{rig.root / 'logs'}'); not "
                f"guarding against a second run\n")
        assert (rig.out, rig.err) == (_Said.checks(rig) + note + _Said.flashed(), "")
        assert rig.bus.log == CHECKED_BUS + FLASHED_BUS

    def test_a_second_update_waits_for_the_first(self, rig: _Rig) -> None:
        (rig.root / "logs").mkdir()
        with open(rig.root / "logs" / "ams_flash.lock", "w") as first:
            fcntl.flock(first, fcntl.LOCK_EX | fcntl.LOCK_NB)
            assert rig.update(rig.args()) == (
                "abort", "another AMS update is already running; let it finish")
        assert (rig.out, rig.err) == (_Said.checks(rig), "")
        assert (rig.moon.scripts, rig.bus.log) == ([], [])

    def test_a_refused_hold_stops_it_and_still_gives_the_link_back(self, rig: _Rig) -> None:
        rig.moon.answers[HOLD] = [False]
        assert rig.update(rig.args()) == ("abort", "Klipper would not let go of the bridge link")
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.BUS, "")
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]
        assert (rig.opened, rig.bus.log) == ([], [])

    def test_a_link_klipper_will_not_take_back_is_noted(self, rig: _Rig) -> None:
        rig.moon.answers[BACK] = [False]
        assert rig.update(rig.args()) == ("rc", 0)
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.flashed() + _Said.NOTE, "")
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]

    def test_a_bridge_link_that_will_not_open_stops_it(self, rig: _Rig) -> None:
        rig.open_errors = [OSError("port busy")] * 5
        assert rig.update(rig.args()) == ("abort", "the bridge link failed (port busy)")
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.BUS, "")
        assert rig.opened == [rig.port] * 5
        assert rig.clock.slept == [1.0] * 4
        assert rig.bus.log == []                         # nothing to hand back or close
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]

    def test_a_refused_link_key_stops_it(self, rig: _Rig) -> None:
        target = "tcp://bridge.invalid:8888"
        assert rig.update(rig.args(target=target, key="made-up-key")) == (
            "abort", "the bridge refused the link key")
        assert (rig.out, rig.err) == (_Said.checks(rig, bridge=target) + _Said.BUS, "")
        assert rig.opened == [target]
        assert rig.bus.log == ["open"] + RELEASED
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]

    def test_a_bridge_that_does_not_answer_stops_it(self, rig: _Rig) -> None:
        rig.bus.mute = True
        assert rig.update(rig.args()) == (
            "abort", "the bridge link failed (bridge did not answer fwreplay off)")
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.BUS, "")
        assert rig.bus.log == ["open", "fwreplay off"] + RELEASED
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]

    def test_a_unit_the_bridge_does_not_see_is_refused(self, rig: _Rig) -> None:
        rig.unit.online = False
        assert rig.update(rig.args()) == (
            "abort", "Bambu_AMS_1 (index 0) is not answering the bridge")
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.BUS + _Said.NONE, "")
        assert rig.bus.log == ["open", "fwreplay off", "sniff off", "status"] + RELEASED

    def test_a_dry_on_the_bridge_stops_it(self, rig: _Rig) -> None:
        rig.bus.dryrem = 600
        assert rig.update(rig.args()) == (
            "abort", "a dry cycle is running on this bridge; stop it and retry")
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.BUS + _Said.ONE, "")
        assert rig.bus.log == ["open", "fwreplay off", "sniff off", "status"] + RELEASED

    def test_a_unit_that_does_not_say_its_version_is_refused(self, rig: _Rig) -> None:
        rig.unit.answers = False
        assert rig.update(rig.args()) == ("abort", "Bambu_AMS_1 did not answer the version query")
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.BUS + _Said.ONE, "")
        assert rig.bus.log == (["open", "fwreplay off", "sniff off", "status"]
                               + ["version? 0x00"] * 4 + RELEASED)

    @pytest.mark.parametrize("code,said", [("N3F06", "AMS 2 Pro (n3f_rev6)"), ("X9Z01", "X9Z01")],
                             ids=["another-board", "unknown-code"])
    def test_a_unit_that_says_it_is_another_board_is_refused(self, rig: _Rig, code: str,
                                                             said: str) -> None:
        rig.unit.version, rig.unit.code = "01.02.03.01", code
        assert rig.update(rig.args()) == (
            "abort", f"Bambu_AMS_1 is {said}, and {AMS2_FILE} is for AMS 2 Pro (n3f_rev5)")
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE
            + f"    Bambu_AMS_1: firmware 01.02.03.01, model code {code}\n", "")
        assert rig.result["from"] == "01.02.03.01"       # what the unit said, not Klipper
        assert rig.bus.log == ["open", "fwreplay off", "sniff off", "status",
                               "version? 0x00"] + RELEASED
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]

    def test_the_version_a_unit_runs_is_written_again(self, rig: _Rig) -> None:
        rig.unit.version = "01.02.03.04"
        assert rig.update(rig.args()) == ("rc", 0)
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE
            + "    Bambu_AMS_1: firmware 01.02.03.04, model code N3F05\n"
            + "    it already runs this version; writing it again\n" + _Said.LOADER
            + _Said.BOOTED + "\n==> DONE. Bambu_AMS_1 now runs 01.02.03.04 (was 01.02.03.04).\n",
            "")
        assert rig.bus.log == CHECKED_BUS + FLASHED_BUS

    def test_a_unit_in_its_loader_elsewhere_on_the_bus_stops_it(self, rig: _Rig) -> None:
        rig.bus.units.append(_Unit(1, 0x01, 0x0700, "01.02.03.00", "N3F05", mode="loader"))
        assert rig.update(rig.args()) == (
            "abort", "a unit on the bus is sitting in its bootloader; power-cycle it, or finish "
                     "its update first")
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE + _Said.READ, "")
        assert rig.bus.log == ["open", "fwreplay off", "sniff off", "status", "version? 0x00",
                               "sniff on", "sniff off"] + RELEASED
        assert rig.state() is None

    def test_a_unit_that_cannot_be_recorded_is_not_sent_into_its_loader(self,
                                                                        rig: _Rig) -> None:
        Path(f"{rig.state_path}.tmp").mkdir()
        assert rig.update(rig.args()) == (
            "abort", "without a record of it, a stopped update could not be resumed, so the "
                     "unit is not sent into its loader")
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE + _Said.READ + "    OK\n",
            f"    cannot record the unit in {rig.state_path} ([Errno 21] Is a directory: "
            f"'{rig.state_path}.tmp')\n")
        assert rig.bus.log == CHECKED_BUS + RELEASED     # no cmd1
        assert rig.state() is None

    def test_a_unit_that_does_not_enter_its_loader_is_refused(self, rig: _Rig) -> None:
        rig.unit.enters = False
        assert rig.update(rig.args()) == ("abort", "Bambu_AMS_1 did not enter its loader")
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE + _Said.READ + _Said.PROBE, "")
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00"] * 4 + RELEASED
        assert rig.state() == rig.record()

    def test_a_unit_that_goes_quiet_when_this_one_enters_its_loader_stops_it(self,
                                                                             rig: _Rig) -> None:
        other = _Unit(1, 0x01, 0x0700, "01.02.03.00", "N3F05")
        other.drops = True
        rig.bus.units.append(other)
        rig.moon.units["Bambu_AMS_2"] = _klipper_unit(rig.port, ams_index=1)
        assert rig.update(rig.args()) == (
            "abort", "index 1 stopped answering when Bambu_AMS_1 entered its loader. Nothing "
                     "was erased; power-cycle the units and retry")
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS
            + "    answering on the bus: index 0 (bus id 0x00), index 1 (bus id 0x01)\n"
            + _Said.READ + _Said.PROBE + "    loader confirmed after cmd1 #1\n", "")
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00", "status"] + RELEASED
        assert rig.state() == rig.record()

    def test_other_units_that_keep_answering_let_it_go_on(self, rig: _Rig) -> None:
        rig.bus.units.append(_Unit(1, 0x01, 0x0700, "01.02.03.00", "N3F05"))
        rig.moon.units["Bambu_AMS_2"] = _klipper_unit(rig.port, ams_index=1)
        assert rig.update(rig.args()) == ("rc", 0)
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS
            + "    answering on the bus: index 0 (bus id 0x00), index 1 (bus id 0x01)\n"
            + _Said.READ + _Said.LOADER + _Said.BOOTED + _Said.DONE, "")
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00", "status"] + FLASHED_BUS[1:]

    @pytest.mark.parametrize("rc,said", [
        (3, "Bambu_AMS_1 took every block and refused the image. Its bootloader is intact: "
            "update it again with the right image"),
        (1, "the update did not finish and Bambu_AMS_1 waits in its bootloader; run the update "
            "again to finish it"),
    ], ids=["image-refused", "unfinished"])
    def test_a_flash_that_did_not_take_keeps_the_record(self, rig: _Rig, rc: int,
                                                        said: str) -> None:
        rig.flash_rc = rc
        assert rig.update(rig.args()) == ("abort", said)
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE + _Said.READ + _Said.LOADER, "")
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00", "flash"] + RELEASED
        assert rig.state() == rig.record()
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]

    def test_a_dropped_link_is_reopened_under_a_new_hold(self, rig: _Rig) -> None:
        def flash(link: Any, reconnect: Callable[[], None]) -> int:
            link.dead = True                             # the bridge dropped the link
            reconnect()
            rig.unit.mode, rig.unit.version = "app", "01.02.03.04"
            return 0

        rig.flash_hook = flash
        assert rig.update(rig.args()) == ("rc", 0)
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.flashed(), "")
        assert rig.moon.scripts == [RESPOND, HOLD, HOLD, BACK]
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00", "flash", "reopen"] + FLASHED_BUS[2:]
        assert rig.bus.dead is False

    def test_a_hold_refused_on_reconnecting_ends_the_flash(self, rig: _Rig) -> None:
        rig.moon.answers[HOLD] = [True, False]
        said: List[str] = []

        def flash(link: Any, reconnect: Callable[[], None]) -> int:
            link.dead = True
            try:
                reconnect()
            except IOError as e:
                said.append(str(e))
            return 1                                     # as do_flash ends then

        rig.flash_hook = flash
        assert rig.update(rig.args()) == (
            "abort", "the update did not finish and Bambu_AMS_1 waits in its bootloader; run "
                     "the update again to finish it")
        assert said == ["Klipper would not let go of the bridge link again"]
        assert rig.moon.scripts == [RESPOND, HOLD, HOLD, BACK]
        # A dead link is closed without handing anything back on it.
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00", "flash", "close"]
        assert rig.state() == rig.record()

    def test_a_link_key_refused_on_reconnecting_ends_the_flash(
            self, rig: _Rig, monkeypatch: pytest.MonkeyPatch) -> None:
        answers, keys = [True, False], []

        def authenticate(link: Any, key: str) -> bool:
            keys.append(key)
            return answers.pop(0)

        said: List[str] = []

        def flash(link: Any, reconnect: Callable[[], None]) -> int:
            link.dead = True
            try:
                reconnect()
            except IOError as e:
                said.append(str(e))
            return 1

        monkeypatch.setattr(ams_flash, "authenticate", authenticate)
        rig.flash_hook = flash
        assert rig.update(rig.args())[0] == "abort"
        assert said == ["link auth failed after reopening"]
        assert keys == ["", ""]
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00", "flash", "reopen"] + RELEASED

    def test_a_unit_that_answers_another_version_after_is_refused(self, rig: _Rig) -> None:
        rig.new_version = "01.02.03.00"
        assert rig.update(rig.args()) == (
            "abort", "Bambu_AMS_1 took the image but answers 01.02.03.00, not 01.02.03.04")
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE + _Said.READ + _Said.LOADER
            + "[8] the unit boots its new firmware\n", "")
        assert rig.bus.log == CHECKED_BUS + FLASHED_BUS
        assert rig.state() is None                       # the flash itself took
        assert rig.result["ok"] is False

    def test_a_unit_slow_to_come_back_is_waited_for(self, rig: _Rig) -> None:
        def boots(seconds: float) -> None:
            rig.unit.mode, rig.unit.version = "app", "01.02.03.04"

        rig.flash_hook = lambda link, reconnect: 0      # still in its loader at first
        rig.clock.on_sleep = boots
        assert rig.update(rig.args()) == ("rc", 0)
        assert (rig.out, rig.err) == (_Said.checks(rig) + _Said.flashed(), "")
        assert rig.clock.slept == [2]
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00", "flash", "status"] + FLASHED_BUS[2:]

    def test_a_unit_that_does_not_come_back_is_reported(self, rig: _Rig) -> None:
        def flash(link: Any, reconnect: Callable[[], None]) -> int:
            rig.unit.mode, rig.unit.answers = "app", False
            return 0

        rig.flash_hook = flash
        assert rig.update(rig.args()) == ("rc", 0)
        message = ("Bambu_AMS_1 took 01.02.03.04 (was 01.02.03.00), but did not answer within "
                   "90s; check that it shows online.")
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE + _Said.READ + _Said.LOADER
            + f"[8] the unit boots its new firmware\n\n==> DONE. {message}\n", "")
        assert rig.result == {"unit": "Bambu_AMS_1", "ok": True, "from": "01.02.03.00",
                              "to": "01.02.03.04", "message": message}
        rounds = len(rig.clock.slept)
        assert rounds > 1
        assert rig.clock.slept == [2] * rounds
        assert rig.bus.log == (CHECKED_BUS + ["cmd1 0x00", "flash"]
                               + (["status"] + ["version? 0x00"] * 4) * rounds + RELEASED)
        assert rig.state() is None

    @pytest.mark.parametrize("sig", [signal.SIGTERM, signal.SIGHUP], ids=["sigterm", "sighup"])
    def test_a_stop_signal_still_gives_the_link_back(self, rig: _Rig, sig: int) -> None:
        def flash(link: Any, reconnect: Callable[[], None]) -> int:
            os.kill(os.getpid(), sig)                    # systemd stopping the update
            return 0

        rig.flash_hook = flash
        assert rig.update(rig.args()) == ("exit", f"stopped by signal {int(sig)}")
        assert (rig.out, rig.err) == (
            _Said.checks(rig) + _Said.BUS + _Said.ONE + _Said.READ + _Said.LOADER, "")
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]
        assert rig.bus.log == CHECKED_BUS + ["cmd1 0x00", "flash"] + RELEASED
        assert rig.state() == rig.record()               # kept, for the next run to finish

    def test_a_unit_left_in_its_loader_is_finished_from_its_record(self, rig: _Rig) -> None:
        # The record of an older run named no bus id; the unit's is 0.
        earlier = rig.record(when="1999-12-31 23:59")
        del earlier["unit_id"]
        rig.state_path.write_text(json.dumps(earlier))
        rig.moon.units["Bambu_AMS_1"].update(bridge_online=False, fw_version="")
        rig.unit.mode = "loader"
        assert rig.update(rig.args(unit=None, version=None)) == ("rc", 0)
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=_Said.OFFLINE + _Said.RESUMING) + _Said.BUS + _Said.NONE
            + _Said.LOADER + _Said.BOOTED + "\n==> DONE. Bambu_AMS_1 now runs 01.02.03.04.\n",
            "")
        assert rig.result == {"unit": "Bambu_AMS_1", "ok": True, "from": "", "to": "01.02.03.04",
                              "message": "Bambu_AMS_1 now runs 01.02.03.04."}
        assert rig.bus.log == ["open", "fwreplay off", "sniff off", "status", "sniff on",
                               "sniff off", "status"] + FLASHED_BUS
        assert rig.bus.record_at_cmd1 == rig.record()
        assert rig.state() is None
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]

    def test_a_resumed_unit_is_addressed_by_its_recorded_id(self, rig: _Rig) -> None:
        rig.moon.units = {"Bambu_AMS_HT_1": _klipper_unit(rig.port, ams_model="ht",
                                                          ams_index=None, bridge_online=False,
                                                          fw_version="", fw_model="N3S05")}
        rig.bus.units = [_Unit(4, 0x80, 0x1800, "01.02.03.00", "N3S05", mode="loader")]
        rig.new_version = "01.02.03.05"
        (rig.here / HT_FILE).write_bytes(HT_IMAGE)
        earlier = {"unit": "Bambu_AMS_HT_1", "unit_id": 128, "board": "n3s_rev5",
                   "version": "01.02.03.05", "file": str(rig.here / HT_FILE),
                   "when": "1999-12-31 23:59"}
        rig.state_path.write_text(json.dumps(earlier))
        assert rig.update(rig.args(unit=None, version=None)) == ("rc", 0)
        message = ("Bambu_AMS_HT_1 took 01.02.03.05, but did not answer within 90s; check that "
                   "it shows online.")
        row = "    Bambu_AMS_HT_1: offline  ams_model=ht  index=None  bus id 0x80\n"
        image = (f"[4] the image\n    {HT_FILE}\n    {HT_FILE}: AMS HT (n3s_rev5) 01.02.03.05, "
                 f"2628 bytes, MD5 {HT_MD5}\n")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=row + _Said.RESUMING, image=image) + _Said.BUS + _Said.NONE
            + _Said.LOADER + f"[8] the unit boots its new firmware\n\n==> DONE. {message}\n", "")
        assert rig.bus.log[:9] == ["open", "fwreplay off", "sniff off", "status", "sniff on",
                                   "sniff off", "status", "cmd1 0x80", "flash"]
        assert rig.bus.record_at_cmd1 == dict(earlier, when="2000-01-02 03:04")

    def test_a_resume_refuses_another_unit_in_its_loader(self, rig: _Rig) -> None:
        earlier = rig.record(when="1999-12-31 23:59")
        rig.state_path.write_text(json.dumps(earlier))
        rig.moon.units["Bambu_AMS_1"].update(bridge_online=False, fw_version="")
        rig.unit.mode = "loader"
        rig.bus.units.append(_Unit(1, 0x01, 0x0700, "01.02.03.00", "N3F05", mode="loader"))
        assert rig.update(rig.args()) == (
            "abort", "another unit on the bus is sitting in its bootloader; power-cycle it and "
                     "retry")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=_Said.OFFLINE + _Said.RESUMING) + _Said.BUS + _Said.NONE, "")
        assert rig.bus.log == ["open", "fwreplay off", "sniff off", "status", "sniff on",
                               "sniff off"] + RELEASED
        assert rig.state() == earlier

    def test_a_record_of_a_unit_running_again_is_dropped(self, rig: _Rig) -> None:
        rig.state_path.write_text(json.dumps(rig.record(when="1999-12-31 23:59")))
        rig.bus.units.append(_Unit(1, 0x01, 0x0700, "01.02.03.00", "N3F05", mode="loader"))
        assert rig.update(rig.args(version=None)) == (
            "abort", "a unit on the bus is sitting in its bootloader; power-cycle it, or finish "
                     "its update first")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=_Said.ROW + _Said.RESUMING) + _Said.BUS + _Said.ONE
            + "    Bambu_AMS_1 runs its firmware again, so the record of it waiting in its "
              "bootloader is stale\n" + _Said.READ, "")
        assert rig.bus.log == ["open", "fwreplay off", "sniff off", "status", "version? 0x00",
                               "version? 0x00", "sniff on", "sniff off"] + RELEASED
        assert rig.state() is None

    def test_a_record_of_a_unit_on_the_bus_that_says_nothing_is_kept(self, rig: _Rig) -> None:
        rig.state_path.write_text(json.dumps(rig.record(when="1999-12-31 23:59")))
        rig.unit.answers = False
        rig.flash_rc = 1
        assert rig.update(rig.args(version=None))[0] == "abort"
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=_Said.ROW + _Said.RESUMING) + _Said.BUS + _Said.ONE
            + _Said.LOADER, "")
        # Asked once as a resume (four tries), then the resumed unit's checks: no second ask.
        assert rig.bus.log == (["open", "fwreplay off", "sniff off", "status"]
                               + ["version? 0x00"] * 4
                               + ["sniff on", "sniff off", "status", "cmd1 0x00", "flash"]
                               + RELEASED)
        assert rig.state() == rig.record()

    def test_a_resumed_unit_that_does_not_enter_its_loader_is_refused(self, rig: _Rig) -> None:
        rig.state_path.write_text(json.dumps(rig.record(when="1999-12-31 23:59")))
        rig.moon.units["Bambu_AMS_1"].update(bridge_online=False, fw_version="")
        rig.unit.mode, rig.unit.enters = "loader", False
        assert rig.update(rig.args()) == (
            "abort", "Bambu_AMS_1 did not enter its loader; if it was power-cycled since, wait "
                     "until it shows online and retry")
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=_Said.OFFLINE + _Said.RESUMING) + _Said.BUS + _Said.NONE
            + _Said.PROBE, "")
        assert rig.bus.log == (["open", "fwreplay off", "sniff off", "status", "sniff on",
                                "sniff off", "status"] + ["cmd1 0x00"] * 4 + RELEASED)

    def test_a_record_of_another_unit_does_not_resume_this_one(self, rig: _Rig) -> None:
        # Bambu_AMS_2 shows online again, so its record is stale and does not stop this run.
        rig.moon.units["Bambu_AMS_2"] = _klipper_unit(rig.port, ams_index=1)
        other = rig.record(unit="Bambu_AMS_2", when="1999-12-31 23:59")
        rig.state_path.write_text(json.dumps(other))
        assert rig.update(rig.args(version=None)) == (
            "abort", "say which image to write: --version (one ams_firmware.json lists) or "
                     "--file")
        assert (rig.out, rig.err) == (_Said.checks(rig, image="[4] the image\n"), "")
        assert rig.state() == other

    def test_a_unit_still_in_its_bootloader_is_finished_first(self, rig: _Rig) -> None:
        rig.moon.units["Bambu_AMS_2"] = _klipper_unit(rig.port, ams_index=1,
                                                      bridge_online=False)
        other = rig.record(unit="Bambu_AMS_2", when="1999-12-31 23:59")
        rig.state_path.write_text(json.dumps(other))
        assert rig.update(rig.args()) == (
            "abort", "Bambu_AMS_2 waits in its bootloader from an earlier run; finish its "
                     "update first")
        assert (rig.out, rig.err) == ("[1] printer not printing\n    state = standby\n"
                                      "[2] the unit\n", "")
        assert rig.state() == other

    def test_a_record_of_a_unit_klipper_does_not_have_says_how_to_clear_it(
            self, rig: _Rig) -> None:
        other = rig.record(unit="Bambu_AMS_9", when="1999-12-31 23:59")
        rig.state_path.write_text(json.dumps(other))
        assert rig.update(rig.args()) == (
            "abort", "ams_flash_state.json says Bambu_AMS_9 waits in its bootloader, and "
                     "Klipper has no such unit; delete that file if the unit is gone")
        assert (rig.out, rig.err) == ("[1] printer not printing\n    state = standby\n"
                                      "[2] the unit\n", "")
        assert rig.state() == other

    def test_a_record_of_a_unit_on_another_bridge_outlives_this_update(self, rig: _Rig) -> None:
        # Bambu_AMS_2 waits in its bootloader on another bridge, where this run cannot hear it;
        # only its record lets a later run finish it.
        other_port = rig.tmp_path / "other-bridge-port"
        other_port.write_text("")
        rig.moon.units["Bambu_AMS_2"] = _klipper_unit(str(other_port), bridge_online=False,
                                                      fw_version="")
        rig.state_path.write_text(json.dumps(rig.record(unit="Bambu_AMS_2",
                                                        when="1999-12-31 23:59")))
        rig.update(rig.args())                           # whether it goes ahead or refuses
        assert rig.update(rig.args(unit="Bambu_AMS_2", version=None, mode="check")) == ("rc", 0)
        row = "    Bambu_AMS_2: offline  ams_model=ams2  index=0  bus id 0x00\n"
        assert (rig.out, rig.err) == (
            _Said.checks(rig, row=row + _Said.RESUMING, bridge=str(other_port))
            + _Said.CHECKED, "")


#: The command line Klipper's Flash AMS menu runs, and the copy it relaunches.
GO = ("--unit", "Bambu_AMS_1", "--version", "01.02.03.04", "--mode", "go")
DETACHED = GO + ("--detached",)
BANNER = ("\n===== 2000-01-02 03:04:05 =====\n"
          "=== AMS update: Bambu_AMS_1 01.02.03.04 mode=GO ===\n")


class TestMain:
    """main() end to end, on a made-up printer."""

    def test_a_detached_update_leaves_its_result_and_shows_it(self, rig: _Rig) -> None:
        assert rig.main(*DETACHED) == ("rc", 0)
        expected = BANNER + _Said.checks(rig) + _Said.flashed()
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert rig.saved_result() == {
            "unit": "Bambu_AMS_1", "ok": True, "from": "01.02.03.00", "to": "01.02.03.04",
            "message": "Bambu_AMS_1 now runs 01.02.03.04 (was 01.02.03.00).",
            "when": "2000-01-02 03:04"}
        assert rig.moon.scripts == [RESPOND, HOLD, BACK, SHOW]
        assert rig.bus.log == CHECKED_BUS + FLASHED_BUS
        assert rig.state() is None

    def test_the_result_is_not_shown_while_klipper_is_not_ready(self, rig: _Rig) -> None:
        rig.moon.klippy = "startup"
        assert rig.main(*DETACHED) == ("rc", 0)
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]
        assert rig.moon.state_calls == 1
        assert rig.saved_result()["ok"] is True

    def test_an_abort_is_the_result(self, rig: _Rig) -> None:
        rig.moon.print_state = "printing"
        assert rig.main(*DETACHED) == ("rc", 1)
        expected = BANNER + "[1] printer not printing\n    state = printing\n" + (
            "    ABORT: a print is active\n")
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert rig.saved_result() == {"unit": "Bambu_AMS_1", "ok": False, "from": "",
                                      "to": "01.02.03.04", "message": "a print is active",
                                      "when": "2000-01-02 03:04"}
        assert rig.moon.scripts == [SHOW]

    def test_a_launched_update_leaves_the_running_result(self, rig: _Rig) -> None:
        assert rig.main(*GO) == ("rc", 0)
        expected = (BANNER + _Said.checks(rig)
                    + "launched detached; progress in logs/ams_flash.log\n")
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert rig.saved_result() == {
            "unit": "Bambu_AMS_1", "ok": False, "from": "01.02.03.00", "to": "01.02.03.04",
            "message": "Updating Bambu_AMS_1. The units on its bridge read offline until it is "
                       "done.", "running": True, "when": "2000-01-02 03:04"}
        assert rig.relaunched == [str(rig.here / "ams_sudo.pw")]
        assert (rig.moon.scripts, rig.moon.state_calls) == ([], 0)

    def test_an_update_that_could_not_start_is_the_result(self, rig: _Rig) -> None:
        rig.relaunch = (2, "could not detach without root:\n  made-up reason")
        assert rig.main(*GO) == ("rc", 1)
        why = ("could not start the update on its own:\n"
               "could not detach without root:\n  made-up reason")
        expected = BANNER + _Said.checks(rig) + f"    ABORT: {why}\n"
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert rig.saved_result() == {"unit": "Bambu_AMS_1", "ok": False, "from": "01.02.03.00",
                                      "to": "01.02.03.04", "message": why, "running": False,
                                      "when": "2000-01-02 03:04"}
        assert (rig.moon.scripts, rig.moon.state_calls) == ([], 0)

    def test_the_launch_leaves_the_result_to_the_update_it_started(
            self, rig: _Rig, monkeypatch: pytest.MonkeyPatch) -> None:
        ended = {"unit": "Bambu_AMS_1", "ok": False, "message": "made-up early end",
                 "when": "1999-12-31 23:59"}

        def relaunch(pass_file: str) -> Tuple[int, str]:
            rig.result_path.write_text(json.dumps(ended))  # it ended before the launch is back
            return 0, ""

        monkeypatch.setattr(ams_flash, "relaunch_detached", relaunch)
        assert rig.main(*GO) == ("rc", 0)
        assert rig.saved_result() == ended
        assert (rig.moon.scripts, rig.moon.state_calls) == ([], 0)

    def test_a_run_naming_no_unit_or_version_leaves_them_empty(self, rig: _Rig) -> None:
        assert rig.main("--mode", "go", "--detached") == ("rc", 1)
        why = "name the unit to update, e.g. --unit Bambu_AMS_1"
        expected = ("\n===== 2000-01-02 03:04:05 =====\n"
                    "=== AMS update: (unfinished update)  mode=GO ===\n"
                    + _Said.START + f"[2] the unit\n    ABORT: {why}\n")
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert rig.saved_result() == {"unit": "", "ok": False, "from": "", "to": "",
                                      "message": why, "when": "2000-01-02 03:04"}
        assert rig.moon.scripts == [SHOW]

    def test_check_mode_leaves_no_result(self, rig: _Rig) -> None:
        assert rig.main("--unit", "Bambu_AMS_1", "--version", "01.02.03.04") == ("rc", 0)
        expected = (BANNER.replace("mode=GO", "mode=CHECK") + _Said.checks(rig)
                    + _Said.CHECKED)
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert rig.saved_result() is None
        assert (rig.moon.scripts, rig.moon.state_calls) == ([], 0)

    def test_the_download_runs_in_the_update_the_launch_starts(
            self, rig: _Rig, monkeypatch: pytest.MonkeyPatch) -> None:
        (rig.here / AMS2_FILE).unlink()
        server = _Server({PACKAGE_URL: _package({AMS2_FILE: AMS2_IMAGE})})
        monkeypatch.setattr(urllib.request, "urlopen", server)
        assert rig.main(*GO) == ("rc", 0)
        assert rig.out == (BANNER + _Said.checks(rig, image=(
            "[4] the image\n    01.02.03.04 for AMS 2 Pro (n3f_rev5): downloaded from Bambu "
            "when the update starts\n")) + "launched detached; progress in logs/ams_flash.log\n")
        assert server.asked == []                        # the launch downloads nothing
        assert rig.main(*DETACHED) == ("rc", 0)
        image = _Said.IMAGE.replace("[4] the image\n",
                                    f"[4] the image\n    downloading {AMS2_FILE} from Bambu\n")
        expected = BANNER + _Said.checks(rig, image=image) + _Said.flashed()
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert server.asked[0] == (PACKAGE_URL, "bytes=0-0", "ams_flash/2", 30.0)
        assert (rig.here / AMS2_FILE).read_bytes() == AMS2_IMAGE
        assert rig.saved_result()["ok"] is True

    def test_an_unfinished_update_is_finished_from_its_record(self, rig: _Rig) -> None:
        rig.state_path.write_text(json.dumps(rig.record(when="1999-12-31 23:59")))
        rig.moon.units["Bambu_AMS_1"].update(bridge_online=False, fw_version="")
        rig.unit.mode = "loader"
        assert rig.main("--mode", "go", "--detached") == ("rc", 0)
        expected = ("\n===== 2000-01-02 03:04:05 =====\n"
                    "=== AMS update: (unfinished update)  mode=GO ===\n"
                    + _Said.checks(rig, row=_Said.OFFLINE + _Said.RESUMING) + _Said.BUS
                    + _Said.NONE + _Said.LOADER + _Said.BOOTED
                    + "\n==> DONE. Bambu_AMS_1 now runs 01.02.03.04.\n")
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert rig.saved_result() == {"unit": "Bambu_AMS_1", "ok": True, "from": "",
                                      "to": "01.02.03.04",
                                      "message": "Bambu_AMS_1 now runs 01.02.03.04.",
                                      "when": "2000-01-02 03:04"}
        assert rig.state() is None

    def test_a_unit_on_another_board_is_the_result(self, rig: _Rig) -> None:
        rig.unit.code = "N3F06"
        assert rig.main(*DETACHED) == ("rc", 1)
        why = f"Bambu_AMS_1 is AMS 2 Pro (n3f_rev6), and {AMS2_FILE} is for AMS 2 Pro (n3f_rev5)"
        expected = (BANNER + _Said.checks(rig) + _Said.BUS + _Said.ONE
                    + "    Bambu_AMS_1: firmware 01.02.03.00, model code N3F06\n"
                    + f"    ABORT: {why}\n")
        assert (rig.out, rig.err, rig.log) == (expected, "", expected)
        assert rig.saved_result() == {"unit": "Bambu_AMS_1", "ok": False, "from": "01.02.03.00",
                                      "to": "01.02.03.04", "message": why,
                                      "when": "2000-01-02 03:04"}
        assert rig.moon.scripts == [RESPOND, HOLD, BACK, SHOW]

    def test_a_unit_on_another_version_after_is_the_result(self, rig: _Rig) -> None:
        rig.new_version = "01.02.03.00"
        assert rig.main(*DETACHED) == ("rc", 1)
        why = "Bambu_AMS_1 took the image but answers 01.02.03.00, not 01.02.03.04"
        assert rig.out.endswith(f"[8] the unit boots its new firmware\n    ABORT: {why}\n")
        assert rig.saved_result() == {"unit": "Bambu_AMS_1", "ok": False, "from": "01.02.03.00",
                                      "to": "01.02.03.04", "message": why,
                                      "when": "2000-01-02 03:04"}
        assert rig.moon.scripts == [RESPOND, HOLD, BACK, SHOW]

    def test_the_banner_names_a_file_given(self, rig: _Rig) -> None:
        assert rig.main("--unit", "Bambu_AMS_1", "--file", AMS2_FILE) == ("rc", 0)
        assert rig.out == ("\n===== 2000-01-02 03:04:05 =====\n"
                           f"=== AMS update: Bambu_AMS_1 {AMS2_FILE} mode=CHECK ===\n"
                           + _Said.checks(rig) + _Said.CHECKED)

    def test_a_config_root_given_is_used(self, rig: _Rig) -> None:
        other = rig.tmp_path / "elsewhere" / "config"
        other.mkdir(parents=True)
        assert rig.main("--unit", "Bambu_AMS_1", "--version", "01.02.03.04", "--cfg-dir",
                        str(other)) == ("rc", 0)
        expected = (BANNER.replace("mode=GO", "mode=CHECK") + _Said.checks(rig)
                    + _Said.CHECKED)
        assert rig.out == expected
        assert (rig.tmp_path / "elsewhere" / "logs" / "ams_flash.log").read_text() == expected
        assert rig.log is None                           # not beside the script's own config

    def test_without_a_log_folder_the_run_goes_to_the_console(self, rig: _Rig) -> None:
        (rig.root / "logs").write_text("")
        assert rig.main(*DETACHED) == ("rc", 0)
        note = (f"note: no lock file ([Errno 17] File exists: '{rig.root / 'logs'}'); not "
                f"guarding against a second run\n")
        assert (rig.out, rig.err, rig.log) == (
            "=== AMS update: Bambu_AMS_1 01.02.03.04 mode=GO ===\n" + _Said.checks(rig) + note
            + _Said.flashed(), "", None)

    def test_a_log_that_breaks_does_not_stop_the_run(self, rig: _Rig) -> None:
        def break_the_log() -> None:
            sys.stdout.s[1].close()                      # the tee's log file
            sys.stdout.flush()                           # shrugged off for the log

        rig.moon.on_print_stats = break_the_log
        assert rig.main("--unit", "Bambu_AMS_1", "--version", "01.02.03.04") == ("rc", 0)
        banner = BANNER.replace("mode=GO", "mode=CHECK")
        assert rig.out == banner + _Said.checks(rig) + _Said.CHECKED
        assert rig.log == banner + "[1] printer not printing\n"

    def test_a_stopped_update_does_not_stay_running(self, rig: _Rig) -> None:
        # The launch leaves "running" for the Flash AMS menu, which refuses to start another
        # update and offers no finish while it says so. A stop of the update it started must
        # replace that, or the menu reports an update running that is long gone.
        assert rig.main(*GO) == ("rc", 0)
        assert rig.saved_result()["running"] is True

        def stopped(link: Any, reconnect: Callable[[], None]) -> int:
            os.kill(os.getpid(), signal.SIGTERM)
            return 1

        rig.flash_hook = stopped
        assert rig.main(*DETACHED) == ("exit", "stopped by signal 15")
        assert rig.moon.scripts == [RESPOND, HOLD, BACK]  # the link went back
        assert rig.state() == rig.record()                # and the record stays to finish it
        saved = rig.saved_result()
        assert not saved.get("running")
        assert saved["ok"] is False
