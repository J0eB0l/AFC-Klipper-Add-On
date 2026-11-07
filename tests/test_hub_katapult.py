"""
Unit tests for firmwares/BridgeHub/hub_katapult.py

Covers every function, with a socketpair standing in for the hub's TCP port,
a fake hub thread answering the unit's JSON commands, and a stub flashtool
script that talks over the real pseudo terminal the tool makes.
"""

from __future__ import annotations

import errno
import importlib.util
import json
import os
import socket
import subprocess
import sys
import termios
import threading
from pathlib import Path
from typing import Callable, Dict, List

import pytest

TOOL = Path(__file__).resolve().parents[1] / "firmwares" / "BridgeHub" / "hub_katapult.py"
_spec = importlib.util.spec_from_file_location("hub_katapult", TOOL)
hk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(hk)

STUB = """
import os, sys, time, tty
args = sys.argv[1:]
dev = args[args.index("-d") + 1]
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY)
tty.setraw(fd)
os.write(fd, b"PING")
got, end = b"", time.time() + 5
while len(got) < 4 and time.time() < end:
    got += os.read(fd, 4 - len(got))
os.close(fd)
sys.exit(int(os.environ.get("STUB_EXIT", "0")) if got == b"PONG" else 9)
"""


class FakeHub(threading.Thread):
    """The unit behind the hub: answers JSON commands and a stub PING."""

    def __init__(self, sock: socket.socket, replies: Dict[str, Callable[[int], dict]]):
        super().__init__(daemon=True)
        self.sock = sock
        self.replies = replies
        self.received = b""

    def run(self) -> None:
        buf = b""
        while True:
            try:
                data = self.sock.recv(4096)
            except OSError:
                return
            if not data:
                return
            self.received += data
            buf += data
            if b"PING" in buf:
                buf = buf.replace(b"PING", b"")
                self.sock.sendall(b"PONG")
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                req = json.loads(line)
                make = self.replies.get(req["cmd"])
                if make is not None:
                    self.sock.sendall(json.dumps(make(req["id"])).encode() + b"\n")


@pytest.fixture
def pair():
    a, b = socket.socketpair()
    yield a, b
    a.close()
    b.close()


@pytest.fixture
def fast(monkeypatch):
    monkeypatch.setattr(hk, "REPLY_WAIT", 0.3)
    monkeypatch.setattr(hk, "BOOT_WAIT", 0.2)
    monkeypatch.setattr(hk, "INFO_TRIES", 2)


@pytest.fixture
def stub(tmp_path):
    path = tmp_path / "flashtool.py"
    path.write_text(STUB)
    return str(path)


@pytest.fixture
def ttys(monkeypatch):
    """Records each tty open_tty makes."""
    made: List[tuple] = []
    real = hk.open_tty

    def wrap():
        out = real()
        made.append(out)
        return out
    monkeypatch.setattr(hk, "open_tty", wrap)
    return made


def lines(capsys) -> List[str]:
    return capsys.readouterr().out.splitlines()


class FakeProc:
    def __init__(self, polls: List, code: int):
        self.polls = list(polls)
        self.code = code
        self.killed = False

    def poll(self):
        return self.polls.pop(0) if self.polls else self.code

    def kill(self):
        self.killed = True

    def wait(self):
        return self.code


# ── log ──────────────────────────────────────────────────────────────────────

class TestLog:
    def test_prints_with_prefix(self, capsys):
        hk.log("hello")
        assert lines(capsys) == ["hub: hello"]


# ── parse_target ─────────────────────────────────────────────────────────────

class TestParseTarget:
    def test_ip_and_port(self):
        assert hk.parse_target("192.168.1.50:4001") == ("192.168.1.50", 4001)

    def test_bracketed_ipv6(self):
        assert hk.parse_target("[::1]:4001") == ("::1", 4001)

    def test_highest_port(self):
        assert hk.parse_target("10.0.0.2:65535") == ("10.0.0.2", 65535)

    @pytest.mark.parametrize("spec", [
        "192.168.1.50",         # no colon
        ":4001",                # no host
        "192.168.1.50:abc",     # port not a number
        "192.168.1.50:",        # no port
        "192.168.1.50:0",       # port too low
        "192.168.1.50:65536",   # port too high
    ])
    def test_rejects(self, spec):
        with pytest.raises(ValueError) as e:
            hk.parse_target(spec)
        assert str(e.value) == f"{spec} is not <host>:<port>"


# ── read_lines ───────────────────────────────────────────────────────────────

class TestReadLines:
    def test_nothing_arrives(self, pair, capsys):
        a, _ = pair
        assert hk.read_lines(a, 0.1) == []
        assert lines(capsys) == []

    def test_json_objects_printed_and_returned(self, pair, capsys):
        a, b = pair
        b.sendall(b'{"id": 1, "ok": true}\n{"evt": "x"}\n')
        assert hk.read_lines(a, 0.2) == [{"id": 1, "ok": True}, {"evt": "x"}]
        assert lines(capsys) == ['hub: < {"id": 1, "ok": true}', 'hub: < {"evt": "x"}']

    def test_skips_text_and_non_objects(self, pair, capsys):
        a, b = pair
        b.sendall(b"boot banner\n5\n[1, 2]\n")
        assert hk.read_lines(a, 0.2) == []
        assert lines(capsys) == []

    def test_partial_line_waits(self, pair, capsys):
        a, b = pair
        b.sendall(b'{"id": 1')
        assert hk.read_lines(a, 0.2) == []
        assert lines(capsys) == []

    def test_closed_connection(self, pair):
        a, b = pair
        b.close()
        with pytest.raises(ConnectionError) as e:
            hk.read_lines(a, 0.5)
        assert str(e.value) == "the hub closed the connection"


# ── ask ──────────────────────────────────────────────────────────────────────

class TestAsk:
    def test_matching_reply(self, pair, capsys):
        a, b = pair
        b.sendall(b'{"id": 7, "ok": true}\n')
        assert hk.ask(a, 7, "info", 0.2) == {"id": 7, "ok": True}
        assert b.recv(100) == b'\n{"id": 7, "cmd": "info"}\n'
        assert lines(capsys) == ['hub: < {"id": 7, "ok": true}']

    def test_other_ids_only(self, pair, capsys):
        a, b = pair
        b.sendall(b'{"id": 3, "ok": true}\n{"evt": "status"}\n')
        assert hk.ask(a, 7, "info", 0.2) is None
        assert lines(capsys) == ['hub: < {"id": 3, "ok": true}', 'hub: < {"evt": "status"}']

    def test_no_reply(self, pair, capsys):
        a, _ = pair
        assert hk.ask(a, 7, "info", 0.1) is None
        assert lines(capsys) == []


# ── open_tty ─────────────────────────────────────────────────────────────────

class TestOpenTty:
    def test_raw_tty_with_only_the_master_open(self):
        master, path = hk.open_tty()
        try:
            assert os.path.exists(path)
            # No slave fd is held: reading the master fails with EIO.
            with pytest.raises(OSError) as e:
                os.read(master, 1)
            assert e.value.errno == errno.EIO
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
            try:
                lflag = termios.tcgetattr(fd)[3]
                assert lflag & (termios.ICANON | termios.ECHO) == 0
            finally:
                os.close(fd)
        finally:
            os.close(master)


# ── relay ────────────────────────────────────────────────────────────────────

class TestRelay:
    def test_carries_bytes_both_ways_until_exit(self, pair, stub):
        a, b = pair
        hub = FakeHub(b, {})
        hub.start()
        master, path = hk.open_tty()
        try:
            proc = subprocess.Popen([sys.executable, stub, "-d", path])
            assert hk.relay(a, master, proc) == 0
        finally:
            os.close(master)
        assert hub.received == b"PING"

    def test_exit_code_passed_on(self, pair, stub, monkeypatch):
        a, b = pair
        FakeHub(b, {}).start()
        monkeypatch.setenv("STUB_EXIT", "4")
        master, path = hk.open_tty()
        try:
            proc = subprocess.Popen([sys.executable, stub, "-d", path])
            assert hk.relay(a, master, proc) == 4
        finally:
            os.close(master)

    def test_already_exited(self, pair):
        a, _ = pair
        proc = FakeProc([], 5)
        master, _ = hk.open_tty()
        try:
            assert hk.relay(a, master, proc) == 5
        finally:
            os.close(master)
        assert proc.killed is False

    def test_hub_closes_during_flash(self, pair, capsys):
        a, b = pair
        b.close()
        proc = FakeProc([None, None, None], -9)
        master, _ = hk.open_tty()
        try:
            assert hk.relay(a, master, proc) == -9
        finally:
            os.close(master)
        assert proc.killed is True
        assert lines(capsys) == ["hub: the hub closed the connection during the flash"]

    def test_other_tty_errors_raise(self, pair, monkeypatch):
        a, _ = pair

        def bad_read(fd, n):
            raise OSError(errno.EBADF, "bad fd")
        master, _ = hk.open_tty()
        monkeypatch.setattr(hk.os, "read", bad_read)
        try:
            with pytest.raises(OSError) as e:
                hk.relay(a, master, FakeProc([None], 0))
            assert e.value.errno == errno.EBADF
        finally:
            monkeypatch.undo()
            os.close(master)


# ── flash ────────────────────────────────────────────────────────────────────

def ok(rid: int) -> dict:
    return {"id": rid, "ok": True}


def info(rid: int) -> dict:
    return {"id": rid, "ok": True, "fw": "0.1.0"}


class TestFlash:
    def test_flashes_and_new_build_answers(self, pair, stub, fast, ttys, capsys):
        a, b = pair
        hub = FakeHub(b, {"bootloader": ok, "info": info})
        hub.start()
        assert hk.flash(a, stub, "fw.bin") == 0
        master, path = ttys[0]
        with pytest.raises(OSError):
            os.fstat(master)            # closed after the flash
        assert lines(capsys) == [
            'hub: < {"id": 1, "ok": true}',
            "hub: the unit is restarting into Katapult",
            f"hub: flashtool on {path}",
            "hub: flashtool exit 0",
            'hub: < {"id": 2, "ok": true, "fw": "0.1.0"}',
            "hub: the new build is running",
        ]
        assert hub.received == (b'\n{"id": 1, "cmd": "bootloader"}\n' + b"PING"
                                + b'\n{"id": 2, "cmd": "info"}\n')

    def test_no_bootloader_reply(self, pair, stub, fast, ttys, capsys):
        a, b = pair
        FakeHub(b, {"info": info}).start()
        assert hk.flash(a, stub, "fw.bin") == 0
        path = ttys[0][1]
        assert lines(capsys) == [
            "hub: no reply to bootloader (the unit may already be in Katapult)",
            f"hub: flashtool on {path}",
            "hub: flashtool exit 0",
            'hub: < {"id": 2, "ok": true, "fw": "0.1.0"}',
            "hub: the new build is running",
        ]

    def test_flashtool_fails(self, pair, stub, fast, ttys, capsys, monkeypatch):
        a, b = pair
        hub = FakeHub(b, {"bootloader": ok, "info": info})
        hub.start()
        monkeypatch.setenv("STUB_EXIT", "3")
        assert hk.flash(a, stub, "fw.bin") == 3
        master, path = ttys[0]
        with pytest.raises(OSError):
            os.fstat(master)
        assert lines(capsys) == [
            'hub: < {"id": 1, "ok": true}',
            "hub: the unit is restarting into Katapult",
            f"hub: flashtool on {path}",
            "hub: flashtool exit 3",
        ]
        assert b'"cmd": "info"' not in hub.received

    def test_new_build_silent(self, pair, stub, fast, ttys, capsys):
        a, b = pair
        hub = FakeHub(b, {"bootloader": ok})
        hub.start()
        assert hk.flash(a, stub, "fw.bin") == 2
        path = ttys[0][1]
        assert lines(capsys) == [
            'hub: < {"id": 1, "ok": true}',
            "hub: the unit is restarting into Katapult",
            f"hub: flashtool on {path}",
            "hub: flashtool exit 0",
            "hub: no reply from the new build",
        ]
        assert hub.received.endswith(b'\n{"id": 2, "cmd": "info"}\n'
                                     b'\n{"id": 3, "cmd": "info"}\n')


# ── main ─────────────────────────────────────────────────────────────────────

@pytest.fixture
def files(tmp_path):
    fw = tmp_path / "boxturtle.bin"
    fw.write_bytes(b"\0" * 16)
    tool = tmp_path / "flashtool.py"
    tool.write_text("")
    return str(fw), str(tool)


@pytest.fixture
def listener():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    yield srv
    srv.close()


class TestMain:
    def test_bad_target(self, files, capsys):
        fw, tool = files
        with pytest.raises(SystemExit) as e:
            hk.main(["hub", fw, "--flashtool", tool])
        assert e.value.code == 2
        assert capsys.readouterr().err.splitlines()[-1].endswith(
            "error: hub is not <host>:<port>")

    def test_missing_firmware(self, files, tmp_path, capsys):
        _, tool = files
        missing = str(tmp_path / "nope.bin")
        assert hk.main(["127.0.0.1:4001", missing, "--flashtool", tool]) == 1
        assert lines(capsys) == [f"hub: {missing} not found"]

    def test_missing_default_flashtool(self, files, tmp_path, monkeypatch, capsys):
        fw, _ = files
        missing = str(tmp_path / "katapult" / "flashtool.py")
        monkeypatch.setattr(hk, "DEFAULT_FLASHTOOL", missing)
        assert hk.main(["127.0.0.1:4001", fw]) == 1
        assert lines(capsys) == [f"hub: {missing} not found"]

    def test_cannot_connect(self, files, listener, capsys):
        fw, tool = files
        port = listener.getsockname()[1]
        listener.close()
        assert hk.main([f"127.0.0.1:{port}", fw, "--flashtool", tool]) == 1
        out = lines(capsys)
        assert len(out) == 1
        assert out[0].startswith(f"hub: cannot connect to 127.0.0.1:{port}: ")
        assert "Connection refused" in out[0]

    def test_connects_and_flashes(self, files, listener, monkeypatch, capsys):
        fw, tool = files
        port = listener.getsockname()[1]
        seen = []

        def fake_flash(sock, flashtool, firmware):
            seen.append((sock.getpeername()[1], flashtool, firmware,
                         sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)))
            return 0
        monkeypatch.setattr(hk, "flash", fake_flash)
        assert hk.main([f"127.0.0.1:{port}", fw, "--flashtool", tool]) == 0
        assert seen == [(port, tool, fw, 1)]
        assert lines(capsys) == [f"hub: connected to 127.0.0.1:{port}"]
        conn, _ = listener.accept()
        conn.settimeout(1)
        assert conn.recv(10) == b""     # the tool closed its end
        conn.close()

    def test_connection_lost(self, files, listener, monkeypatch, capsys):
        fw, tool = files
        port = listener.getsockname()[1]

        def lost(sock, flashtool, firmware):
            raise ConnectionError("the hub closed the connection")
        monkeypatch.setattr(hk, "flash", lost)
        assert hk.main([f"127.0.0.1:{port}", fw, "--flashtool", tool]) == 1
        assert lines(capsys) == [f"hub: connected to 127.0.0.1:{port}",
                                 "hub: the hub closed the connection"]

    def test_run_as_script(self):
        res = subprocess.run([sys.executable, str(TOOL), "hub", "fw.bin"],
                             capture_output=True, text=True, timeout=30)
        assert res.returncode == 2
        assert res.stderr.splitlines()[-1].endswith("error: hub is not <host>:<port>")
