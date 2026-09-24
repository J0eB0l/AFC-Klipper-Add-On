"""
Daemon-side write path without hardware: a fake reader behind Runtime.write_ntag,
driven through the real FileWriteWatchController over a temp request dir.
Run from repo root: python3 test_write_watch.py
"""
import sys, os, json, threading, types, tempfile, time

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src")
sys.path.insert(0, SRC)
for name in ("spidev", "gpiod", "requests"):
    sys.modules.setdefault(name, types.ModuleType(name))

from reader.mifare_ultralight_reader import MifareUltralightReader   # noqa
from reader.scan_result import ScanResult                            # noqa
from tag.tag_types import TagType                                    # noqa
from runtime import Runtime                                          # noqa
from controllers.file_write_watch import FileWriteWatchController    # noqa


class FakeUltralight(MifareUltralightReader):
    def __init__(self, uid=b"\x04\xa1\xb2\xc3\xd4\xe5\xf6", tag=None, fail=False):
        self.name = "fake"
        self._uid = uid
        self._tag = TagType.MifareUltralight if tag is None else tag
        self._fail = fail
        self.sessions = 0
        self.written = None

    def start_session(self): self.sessions += 1
    def end_session(self): pass
    def scan(self):
        if self._uid is None:
            return None
        return ScanResult(self._tag, bytes(self._uid), b"\x44\x00", b"", b"\x00")
    def read_mifare_ultralight(self, scan_result): return None
    def write_mifare_ultralight(self, scan_result, start_page, data):
        if self._fail: return False
        self.written = (start_page, bytes(data)); return True


def runtime_with(reader):
    r = Runtime.__new__(Runtime)
    r.rfid_readers = [reader]
    r.reader_lock = threading.Lock()
    return r


def controller(dirpath, rt):
    c = FileWriteWatchController.__new__(FileWriteWatchController)
    c.request_dir = dirpath
    c.poll_interval = 0.0
    c.logger = types.SimpleNamespace(info=lambda *a: None, error=lambda *a: None)
    c.runtime = rt
    return c


def req(dirpath, token, obj):
    with open(os.path.join(dirpath, f"req-{token}.json"), "w") as f:
        json.dump(obj, f)


def res(dirpath, token):
    p = os.path.join(dirpath, f"res-{token}.json")
    return json.load(open(p)) if os.path.exists(p) else None


PASS = [True]
def check(name, cond):
    print(("  PASS  " if cond else "  FAIL  ") + name)
    PASS[0] &= bool(cond)


print("a clean write round-trips through the file protocol")
with tempfile.TemporaryDirectory() as d:
    fake = FakeUltralight()
    c = controller(d, runtime_with(fake))
    payload = bytes(range(144))
    req(d, "t1", {"slot": 0, "start_page": 4, "data": payload.hex()})
    c._scan_once()
    r = res(d, "t1")
    check("result ok", r and r["ok"] is True)
    check("uid is the full 7 bytes", r["uid"] == "04a1b2c3d4e5f6")
    check("pages reported", r["pages"] == 36)
    check("bytes reached the reader", fake.written == (4, payload))
    check("request file consumed", not os.path.exists(os.path.join(d, "req-t1.json")))

print("no tag in the field")
with tempfile.TemporaryDirectory() as d:
    fake = FakeUltralight(uid=None)
    c = controller(d, runtime_with(fake))
    req(d, "t2", {"slot": 0, "start_page": 4, "data": (b"\x00"*144).hex()})
    c._scan_once()
    r = res(d, "t2")
    check("reports no tag", r and r["ok"] is False and "no tag" in r["error"])
    check("nothing written", fake.written is None)

print("a MIFARE Classic tag is refused")
with tempfile.TemporaryDirectory() as d:
    fake = FakeUltralight(tag=TagType.MifareClassic1k)
    c = controller(d, runtime_with(fake))
    req(d, "t3", {"slot": 0, "start_page": 4, "data": (b"\x00"*144).hex()})
    c._scan_once()
    r = res(d, "t3")
    check("refused by kind", r and r["ok"] is False and "not an NTAG" in r["error"])
    check("nothing written", fake.written is None)

print("a reader-reported write failure")
with tempfile.TemporaryDirectory() as d:
    fake = FakeUltralight(fail=True)
    c = controller(d, runtime_with(fake))
    req(d, "t4", {"slot": 0, "start_page": 4, "data": (b"\x11"*144).hex()})
    c._scan_once()
    r = res(d, "t4")
    check("failure surfaced", r and r["ok"] is False and "write failed" in r["error"])
    check("uid still reported", r["uid"] == "04a1b2c3d4e5f6")

print("an invalid slot")
with tempfile.TemporaryDirectory() as d:
    c = controller(d, runtime_with(FakeUltralight()))
    req(d, "t5", {"slot": 9, "start_page": 4, "data": (b"\x00"*4).hex()})
    c._scan_once()
    r = res(d, "t5")
    check("invalid slot reported", r and r["ok"] is False and "invalid slot" in r["error"])

print("a malformed request")
with tempfile.TemporaryDirectory() as d:
    c = controller(d, runtime_with(FakeUltralight()))
    req(d, "t6", {"slot": 0, "data": "zz"})     # bad hex, missing start_page
    c._scan_once()
    r = res(d, "t6")
    check("malformed reported, not crashed", r and r["ok"] is False)
    check("request consumed", not os.path.exists(os.path.join(d, "req-t6.json")))

print("the request is removed before the write (no re-fire on crash)")
with tempfile.TemporaryDirectory() as d:
    boom = FakeUltralight()
    def explode(*a): raise RuntimeError("kaboom")
    boom.write_mifare_ultralight = explode
    c = controller(d, runtime_with(boom))
    req(d, "t7", {"slot": 0, "start_page": 4, "data": (b"\x00"*4).hex()})
    try:
        c._scan_once()
    except RuntimeError:
        pass
    check("request already gone", not os.path.exists(os.path.join(d, "req-t7.json")))

print("the lock actually serialises against a scan")
with tempfile.TemporaryDirectory() as d:
    fake = FakeUltralight()
    rt = runtime_with(fake)
    rt.reader_lock.acquire()          # pretend the scan loop holds it
    c = controller(d, runtime_with.__wrapped__(fake) if hasattr(runtime_with,'__wrapped__') else rt)
    req(d, "t8", {"slot": 0, "start_page": 4, "data": (b"\x00"*4).hex()})
    done = []
    th = threading.Thread(target=lambda: (c._scan_once(), done.append(True)))
    th.start(); th.join(timeout=0.3)
    check("write blocked while lock held", not done)
    rt.reader_lock.release()
    th.join(timeout=1.0)
    check("write proceeds once released", bool(done))

print("\nRESULT:", "all passed" if PASS[0] else "FAILURES")
sys.exit(0 if PASS[0] else 1)
