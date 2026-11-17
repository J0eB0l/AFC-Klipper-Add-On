"""
Exercise the NTAG page write without hardware: a simulated tag behind
__command_exe. Run from the repo root: python3 test_ntag_write.py
"""
import sys, os, types

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src")
sys.path.insert(0, SRC)
# spidev/gpiod only exist on the printer; the driver only needs them imported.
for name in ("spidev", "gpiod"):
    m = types.ModuleType(name)
    m.SpiDev = m.Chip = m.LINE_REQ_DIR_OUT = object
    sys.modules.setdefault(name, m)

from reader.fm175xx.rfid import Fm175xx, Fm175xxReturnVal   # noqa: E402
from reader.fm175xx import constants as C                   # noqa: E402

PER = C.FM175XX_NTAG215_BYTES_PER_PAGE


class Tag:
    """An NTAG215's memory, answering the frames the driver sends."""

    def __init__(self, nak_page=None, swallow_page=None):
        self.mem = [[0, 0, 0, 0] for _ in range(C.FM175XX_NTAG215_TOTAL_PAGES)]
        self.nak_page = nak_page          # refuses to ACK
        self.swallow_page = swallow_page  # ACKs, stores nothing
        self.written = []

    def command_exe(self, cmd):
        ret = Fm175xxReturnVal()
        op, page = cmd.send_buff[0], cmd.send_buff[1]
        if op == 0xA2:                                    # WRITE
            if page == self.nak_page:
                cmd.bits_recved, cmd.recv_buff[0] = 4, 0x00   # NAK
                ret.err_code = C.FM175XX_OK
                return ret
            self.written.append(page)
            if page != self.swallow_page:
                self.mem[page] = list(cmd.send_buff[2:6])
            cmd.bits_recved, cmd.recv_buff[0] = 4, 0x0A       # ACK
            ret.err_code = C.FM175XX_OK
            return ret
        if op == 0x30:                                    # READ, 4 pages
            out = []
            for i in range(4):
                out += self.mem[(page + i) % C.FM175XX_NTAG215_TOTAL_PAGES]
            ret.err_code, ret.out_data = C.FM175XX_OK, out
            return ret
        raise AssertionError(f"unexpected opcode 0x{op:02X}")


def reader(tag):
    r = Fm175xx.__new__(Fm175xx)
    r.logger = types.SimpleNamespace(error=lambda *a: None,
                                     info=lambda *a: None)
    r._Fm175xx__command_exe = tag.command_exe
    return r


def write(r, start, data):
    return r._Fm175xx__reader_a_ultralight_write_all_data(start, list(data))


def check(name, cond):
    print(("  PASS  " if cond else "  FAIL  ") + name)
    return cond


ok = True
START = C.FM175XX_NTAG215_USER_START_PAGE
END = C.FM175XX_NTAG215_USER_END_PAGE

print("a clean write")
t = Tag(); r = reader(t)
payload = bytes(range(144))
res = write(r, START, payload)
ok &= check("reports OK", res.err_code == C.FM175XX_OK)
ok &= check("wrote pages 4..39", t.written == list(range(START, START + 36)))
ok &= check("bytes landed", bytes(sum(t.mem[START:START + 36], [])) == payload)

print("the reserved pages are refused")
for page in (0, 1, 2, 3):
    t = Tag(); r = reader(t)
    res = write(r, page, b"\x00" * 4)
    ok &= check(f"page {page} refused, nothing written",
                res.err_code == C.FM175XX_PARAM_ERR and t.written == [])

print("past the user area is refused")
t = Tag(); r = reader(t)
res = write(r, END - 1, b"\x00" * 16)          # would run to END+2
ok &= check("refused", res.err_code == C.FM175XX_PARAM_ERR and t.written == [])
t = Tag(); r = reader(t)
ok &= check("the last legal page is allowed",
            write(r, END, b"\x00" * 4).err_code == C.FM175XX_OK)

print("payload shape")
t = Tag(); r = reader(t)
ok &= check("not whole pages refused",
            write(r, START, b"\x00" * 5).err_code == C.FM175XX_PARAM_ERR)
ok &= check("empty refused",
            write(r, START, b"").err_code == C.FM175XX_PARAM_ERR)

print("a tag that will not take a page")
t = Tag(nak_page=START + 2); r = reader(t)
ok &= check("NAK reported as a write error",
            write(r, START, b"\x11" * 16).err_code == C.FM175XX_CARD_WRITE_ERR)

print("a tag that ACKs and stores nothing")
t = Tag(swallow_page=START + 5); r = reader(t)
ok &= check("caught by the read-back",
            write(r, START, b"\x22" * 32).err_code == C.FM175XX_CARD_WRITE_ERR)

print("a single page")
t = Tag(); r = reader(t)
ok &= check("one page is fine", write(r, START, b"\xAB\xCD\xEF\x01").err_code
            == C.FM175XX_OK and t.mem[START] == [0xAB, 0xCD, 0xEF, 0x01])

print("the public method")
t = Tag(); r = reader(t)
ok &= check("returns True on success",
            r.write_mifare_ultralight(None, START, b"\x01\x02\x03\x04") is True)
t = Tag(); r = reader(t)
ok &= check("returns False on a refusal",
            r.write_mifare_ultralight(None, 0, b"\x01\x02\x03\x04") is False)

print("\nRESULT:", "all passed" if ok else "FAILURES")
sys.exit(0 if ok else 1)
