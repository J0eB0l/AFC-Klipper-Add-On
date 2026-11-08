"""
Unit tests for extras/AFC_bhub_link.py

Covers:
  - parse_tcp, check_link_key, bad_tcp_key
  - the link key file (link_key_file, stored_link_key)
  - the exchange's pieces (auth_line, hub_answer, hub_verdict, is_auth)
  - HubSocket over a socketpair
  - open_hub against a fake hub on a loopback port, one thread per hub
"""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import select
import socket
import threading
from typing import Any, List, Optional
from unittest.mock import patch

import pytest

from extras import AFC_bhub_link as hub_link
from extras.AFC_bhub_link import (
    HUB_PORT,
    HubLinkError,
    HubSocket,
    auth_line,
    bad_tcp_key,
    check_link_key,
    hub_answer,
    hub_verdict,
    is_auth,
    link_key_file,
    open_hub,
    parse_tcp,
    stored_link_key,
)
from tests.conftest import MockLogger

# The firmware's host test (firmwares/BridgeHub/test) checks the same answer.
HUB_KEY = "0123456789abcdef0123456789abcdef"
HUB_NONCE = bytes(range(16))
HUB_MAC = "84aa390f3f68d40d3b8f2b5ee2ad3517c13e31649284a43e867a8fa5974500bc"
KEYED = {"evt": "auth", "nonce": HUB_NONCE.hex()}
OPEN = {"evt": "auth", "open": 1}
BOOT = "hold the hub's BOOT button for 10 s to clear it"


def keys_for(saved: Optional[str] = None, new: Optional[str] = "ab" * 16):
    """
    A unit's link_key: the saved key, else (when asked to make one) new.

    :return tuple: (the callable, the list of make flags it was called with)
    """
    calls: List[bool] = []

    def link_key(make: bool) -> Optional[str]:
        calls.append(make)
        return saved or (new if make else None)
    return link_key, calls


# ── parse_tcp ────────────────────────────────────────────────────────────────

class TestParseTcp:
    @pytest.mark.parametrize("spec, expected", [
        ("tcp://192.168.1.50:4001", ("192.168.1.50", 4001)),
        ("tcp://192.168.1.50:4001/", ("192.168.1.50", 4001)),
        ("tcp://[fe80::1]:4002", ("fe80::1", 4002)),
        ("tcp://::1:65535", ("::1", 65535)),
        ("tcp://192.168.1.50", ("192.168.1.50", HUB_PORT)),
        ("tcp://192.168.1.50/", ("192.168.1.50", HUB_PORT)),
        ("tcp://[fe80::1]", ("fe80::1", HUB_PORT)),
        ("tcp://fe80::1", ("fe80::1", HUB_PORT)),
    ])
    def test_good(self, spec, expected):
        assert parse_tcp(spec) == expected

    def test_hub_port(self):
        assert HUB_PORT == 4001

    @pytest.mark.parametrize("spec", [
        "tcp://192.168.1.50:",
        "tcp://[fe80::1]x",
        "tcp://[fe80::1]:",
        "tcp://192.168.1.50:0",
        "tcp://192.168.1.50:65536",
        "tcp://192.168.1.50:abc",
        "tcp://hub.local:4001",
        "tcp://:4001",
    ])
    def test_bad(self, spec):
        with pytest.raises(ValueError) as e:
            parse_tcp(spec)
        assert str(e.value) == f"serial_port {spec} is not tcp://<ip address>[:<port>]"


# ── check_link_key / bad_tcp_key ─────────────────────────────────────────────

class TestCheckLinkKey:
    @pytest.mark.parametrize("key", ["a", "x" * 64, "!~#$%&'()*+,-./09:;<=>?@AZ[]^_`az{|}"])
    def test_good(self, key):
        assert check_link_key(key) is True

    @pytest.mark.parametrize("key", ["", "x" * 65, "a b", 'a"b', "a\\b", "a\tb", "café"])
    def test_bad(self, key):
        assert check_link_key(key) is False


class TestBadTcpKey:
    @pytest.mark.parametrize("key", [None, HUB_KEY])
    def test_usable(self, key):
        assert bad_tcp_key(key) is None

    @pytest.mark.parametrize("key", ["", "has space", "x" * 65])
    def test_unusable(self, key):
        assert bad_tcp_key(key) == ("tcp_key must be 1 to 64 characters with no spaces, "
                                    "quotes or backslashes")


# ── the key file ─────────────────────────────────────────────────────────────

class TestLinkKeyFile:
    def test_next_to_var_file(self, tmp_path):
        assert link_key_file(str(tmp_path / "AFC.var")) == str(tmp_path / "AFC.var.hub")

    def test_expands_home(self):
        assert link_key_file("~/printer_data/config/AFC/AFC.var") == os.path.expanduser(
            "~/printer_data/config/AFC/AFC.var.hub")


class TestStoredLinkKey:
    @pytest.fixture
    def var(self, tmp_path):
        return str(tmp_path / "AFC.var")

    def saved(self, var):
        with open(link_key_file(var)) as f:
            return json.load(f)

    def test_tcp_key_wins(self, var):
        assert stored_link_key(var, "Ace_1", HUB_KEY, True, MockLogger()) == HUB_KEY
        assert not os.path.exists(link_key_file(var))

    def test_none_saved(self, var):
        assert stored_link_key(var, "Ace_1", None, False, MockLogger()) is None
        assert not os.path.exists(link_key_file(var))

    def test_makes_and_saves_beside_other_units(self, var):
        with open(link_key_file(var), "w") as f:
            json.dump({"Turtle_1": "other"}, f)
        with patch.object(hub_link.secrets, "token_hex", return_value=HUB_KEY) as token:
            assert stored_link_key(var, "Ace_1", None, True, MockLogger()) == HUB_KEY
        token.assert_called_once_with(16)
        assert self.saved(var) == {"Turtle_1": "other", "Ace_1": HUB_KEY}
        assert not os.path.exists(f"{link_key_file(var)}.tmp")
        assert stored_link_key(var, "Ace_1", None, False, MockLogger()) == HUB_KEY

    def test_made_key_is_one_the_hub_takes(self, var):
        key = stored_link_key(var, "Ace_1", None, True, MockLogger())
        assert check_link_key(key) and len(key) == 32

    def test_reads_the_saved_key(self, var):
        with open(link_key_file(var), "w") as f:
            json.dump({"Ace_1": HUB_KEY}, f)
        with patch.object(hub_link.secrets, "token_hex") as token:
            assert stored_link_key(var, "Ace_1", None, True, MockLogger()) == HUB_KEY
        token.assert_not_called()

    @pytest.mark.parametrize("text", ["not json", "[1, 2]", '{"Ace_1": 5}',
                                      '{"Ace_1": "has space"}'])
    def test_unusable_file_makes_a_new_key(self, var, text):
        with open(link_key_file(var), "w") as f:
            f.write(text)
        with patch.object(hub_link.secrets, "token_hex", return_value=HUB_KEY):
            assert stored_link_key(var, "Ace_1", None, True, MockLogger()) == HUB_KEY
        assert self.saved(var)["Ace_1"] == HUB_KEY

    def test_cannot_save(self, tmp_path):
        var = str(tmp_path / "missing" / "AFC.var")
        path = link_key_file(var)
        logger = MockLogger()
        assert stored_link_key(var, "Ace_1", None, True, logger) is None
        assert logger.messages == [
            ("warning", f"Ace_1: cannot save the hub's link key in {path}: "
                        f"[Errno 2] No such file or directory: '{path}.tmp'")]


# ── the exchange's pieces ────────────────────────────────────────────────────

class TestAuthLine:
    def test_compact(self):
        assert auth_line({"cmd": "linkkey", "key": "k"}) == b'{"cmd":"linkkey","key":"k"}\n'


class TestHubAnswer:
    def test_answers_the_challenge(self):
        link_key, calls = keys_for(saved=HUB_KEY)
        line, claimed = hub_answer(KEYED, link_key)
        # Byte for byte what the hub's firmware reads.
        assert line == f'{{"cmd":"auth","mac":"{HUB_MAC}"}}\n'.encode()
        assert claimed is False
        assert calls == [False]

    def test_no_key_for_a_keyed_hub(self):
        link_key, calls = keys_for()
        with pytest.raises(HubLinkError) as e:
            hub_answer(KEYED, link_key)
        assert str(e.value) == f"the hub holds a link key this printer doesn't have; {BOOT}"
        assert calls == [False]

    def test_bad_challenge(self):
        link_key, _ = keys_for(saved=HUB_KEY)
        with pytest.raises(HubLinkError) as e:
            hub_answer({"evt": "auth", "nonce": "zz"}, link_key)
        assert str(e.value) == "the hub sent a bad challenge"

    def test_claims_an_open_hub(self):
        link_key, calls = keys_for()
        line, claimed = hub_answer(OPEN, link_key)
        # The hub's firmware takes a claim only in exactly this form.
        assert line == b'{"cmd":"linkkey","key":"' + b"ab" * 16 + b'"}\n'
        assert claimed is True
        assert calls == [True]

    def test_no_key_to_give(self):
        link_key, _ = keys_for(new=None)
        with pytest.raises(HubLinkError) as e:
            hub_answer(OPEN, link_key)
        assert str(e.value) == "no link key to give the hub"

    def test_unknown_greeting(self):
        link_key, calls = keys_for()
        with pytest.raises(HubLinkError) as e:
            hub_answer({"evt": "auth", "open": 0}, link_key)
        assert str(e.value) == "the hub sent an unknown greeting {'evt': 'auth', 'open': 0}"
        assert calls == []


class TestHubVerdict:
    def test_ok(self):
        hub_verdict({"evt": "auth", "ok": 1})

    @pytest.mark.parametrize("reply", [{"evt": "auth", "ok": 0}, {"evt": "auth"}])
    def test_refused(self, reply):
        with pytest.raises(HubLinkError) as e:
            hub_verdict(reply)
        assert str(e.value) == f"the hub refused the link key; {BOOT}"

    def test_no_answer(self):
        with pytest.raises(HubLinkError) as e:
            hub_verdict(None)
        assert str(e.value) == "no answer from the hub to the link key"


class TestIsAuth:
    @pytest.mark.parametrize("msg, expected", [
        ({"evt": "auth", "ok": 1}, True),
        ({"ev": "auth"}, False),
        ({"evt": "hub"}, False),
        ([1], False),
        ("auth", False),
    ])
    def test_is_auth(self, msg, expected):
        assert is_auth(msg) is expected


# ── HubSocket ────────────────────────────────────────────────────────────────

@pytest.fixture
def pair():
    a, b = socket.socketpair()
    yield a, b
    a.close()
    b.close()


class TestHubSocket:
    def test_rest_first_then_the_socket(self, pair):
        a, b = pair
        port = HubSocket(a, "tcp://10.0.0.2", rest=b"abc")
        assert a.gettimeout() == 0.0
        b.sendall(b"def")
        assert port.read(2) == b"ab"
        assert port.read(10) == b"c"
        assert port.read(10) == b"def"

    def test_nothing_yet(self, pair):
        a, _ = pair
        assert HubSocket(a, "tcp://10.0.0.2").read(10) == b""

    def test_closed_reads_as_disconnected(self, pair):
        a, b = pair
        port = HubSocket(a, "tcp://10.0.0.2")
        b.close()
        with pytest.raises(OSError) as e:
            port.read(10)
        assert str(e.value) == "tcp://10.0.0.2: the hub closed the connection (disconnected)"

    def test_write_flush_and_fileno(self, pair):
        a, b = pair
        port = HubSocket(a, "tcp://10.0.0.2")
        assert port.write(b"frame") == 5
        port.flush()
        port.reset_output_buffer()
        assert b.recv(10) == b"frame"
        assert port.fileno() == a.fileno()

    def test_reset_input_drops_what_arrived(self, pair):
        a, b = pair
        port = HubSocket(a, "tcp://10.0.0.2", rest=b"old")
        b.sendall(b"stale")
        assert select.select([a], [], [], 1.0)[0]
        port.reset_input_buffer()
        assert port.read(10) == b""

    def test_write_gives_up_when_the_hub_takes_nothing(self, pair, monkeypatch):
        a, _ = pair
        a.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
        port = HubSocket(a, "tcp://10.0.0.2")
        monkeypatch.setattr(HubSocket, "WRITE_TIMEOUT", 0.1)
        with pytest.raises(OSError) as e:
            port.write(b"x" * 4_000_000)
        assert str(e.value) == "tcp://10.0.0.2: write timeout"

    def test_write_waits_for_room(self, pair, monkeypatch):
        a, b = pair
        a.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
        port = HubSocket(a, "tcp://10.0.0.2")
        got = bytearray()

        def drain():
            while len(got) < 200_000:
                got.extend(b.recv(65536))
        t = threading.Thread(target=drain, daemon=True)
        t.start()
        assert port.write(b"y" * 200_000) == 200_000
        t.join(2)
        assert bytes(got) == b"y" * 200_000

    def test_reset_input_on_a_closed_peer(self, pair):
        a, b = pair
        port = HubSocket(a, "tcp://10.0.0.2")
        b.close()
        port.reset_input_buffer()

    def test_reset_input_after_close(self, pair):
        a, _ = pair
        port = HubSocket(a, "tcp://10.0.0.2")
        port.close()
        port.reset_input_buffer()
        assert a.fileno() == -1


# ── open_hub ─────────────────────────────────────────────────────────────────

class FakeHub(threading.Thread):
    """
    One connection to a BridgeHub: its greeting, its check of the answer or
    the claim, then the unit's bytes.
    """

    def __init__(self, server: socket.socket, greeting: Any = KEYED, key: str = HUB_KEY,
                 answer: bool = True, after: bytes = b"", hang_up: bool = False) -> None:
        super().__init__(daemon=True)
        self.server = server
        self.greeting = greeting
        self.key = key
        self.answer = answer
        self.after = after
        self.hang_up = hang_up
        self.raw: List[bytes] = []
        self.conn: Optional[socket.socket] = None

    def run(self) -> None:
        self.conn, _ = self.server.accept()
        conn = self.conn
        conn.settimeout(3)
        if self.greeting is not None:
            data = self.greeting if isinstance(self.greeting, bytes) else (
                json.dumps(self.greeting).encode() + b"\n")
            conn.sendall(data)
        if self.hang_up:
            conn.close()
            return
        buf = b""
        try:
            while b"\n" not in buf:
                data = conn.recv(4096)
                if not data:
                    return
                buf += data
        except OSError:
            return
        line = buf.split(b"\n")[0]
        self.raw.append(line)
        if not self.answer:
            return
        msg = json.loads(line)
        if msg.get("cmd") == "linkkey":
            self.key, ok = msg["key"], 1
        else:
            want = hmac.new(self.key.encode(), HUB_NONCE, hashlib.sha512).hexdigest()[:64]
            ok = int(msg.get("mac") == want)
        conn.sendall(json.dumps({"evt": "auth", "ok": ok}).encode() + b"\n" + self.after)


@pytest.fixture
def server():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    yield srv, f"tcp://127.0.0.1:{srv.getsockname()[1]}"
    srv.close()


@pytest.fixture
def fast(monkeypatch):
    monkeypatch.setattr(hub_link, "GREETING_TIME", 0.2)
    monkeypatch.setattr(hub_link, "AUTH_REPLY_TIME", 0.3)


def run_hub(server, link_key, logger=None, tcp_key=None, **kw):
    """Starts a fake hub, then open_hub against it."""
    srv, url = server
    hub = FakeHub(srv, **kw)
    hub.start()
    logger = logger or MockLogger()
    try:
        return open_hub(url, link_key, "Ace_1", logger, tcp_key=tcp_key), hub, logger, url
    finally:
        hub.join(3)


class TestOpenHub:
    def test_answers_the_challenge(self, server, fast):
        link_key, calls = keys_for(saved=HUB_KEY)
        port, hub, logger, url = run_hub(server, link_key, after=b"\xff\xaa unit")
        assert hub.raw == [f'{{"cmd":"auth","mac":"{HUB_MAC}"}}'.encode()]
        assert isinstance(port, HubSocket) and port.port == url
        assert port.sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY) != 0
        assert logger.messages == []
        assert calls == [False]
        got = b""
        for _ in range(100):
            got += port.read(100)
            if got:
                break
            threading.Event().wait(0.01)
        assert got == b"\xff\xaa unit"
        port.close()

    def test_claims_an_open_hub(self, server, fast):
        link_key, calls = keys_for()
        port, hub, logger, url = run_hub(server, link_key, greeting=OPEN)
        assert hub.raw == [b'{"cmd":"linkkey","key":"' + b"ab" * 16 + b'"}']
        assert hub.key == "ab" * 16
        assert logger.messages == [("info", f"Ace_1: link key stored on the hub at {url}")]
        port.close()

    def test_refused(self, server, fast):
        link_key, _ = keys_for(saved="not-the-key")
        with pytest.raises(HubLinkError) as e:
            run_hub(server, link_key)
        assert str(e.value) == f"the hub refused the link key; {BOOT}"

    def test_no_way_to_get_a_key(self, server, fast):
        with pytest.raises(HubLinkError) as e:
            run_hub(server, None, greeting=OPEN)
        assert str(e.value) == "no link key to give the hub"

    def test_no_answer(self, server, fast):
        link_key, _ = keys_for(saved=HUB_KEY)
        with pytest.raises(HubLinkError) as e:
            run_hub(server, link_key, answer=False)
        assert str(e.value) == "no answer from the hub to the link key"

    def test_hangs_up(self, server, fast):
        link_key, _ = keys_for(saved=HUB_KEY)
        with pytest.raises(HubLinkError) as e:
            run_hub(server, link_key, greeting=None, hang_up=True)
        assert str(e.value) == "the hub closed the connection"

    def test_no_greeting_is_a_plain_server(self, server, fast):
        link_key, calls = keys_for(saved=HUB_KEY)
        port, hub, logger, _ = run_hub(server, link_key, greeting=None, answer=False)
        assert calls == []
        assert logger.messages == []
        port.write(b"x\n")
        port.close()

    def test_no_greeting_with_tcp_key_warns(self, server, fast):
        link_key, _ = keys_for(saved=HUB_KEY)
        port, _, logger, url = run_hub(server, link_key, tcp_key=HUB_KEY, greeting=None,
                                       answer=False)
        assert logger.messages == [
            ("warning", f"Ace_1: {url} asked for no link key, so tcp_key is not used")]
        port.write(b"x\n")
        port.close()

    def test_unit_bytes_first_are_kept(self, server, fast):
        # A plain server's unit talks first: its bytes are not a greeting and
        # stay for the unit's own code.
        link_key, calls = keys_for(saved=HUB_KEY)
        port, _, _, _ = run_hub(server, link_key, greeting=b'{"id":1,"ok":true}\nmore',
                                answer=False)
        assert calls == []
        assert port.read(100) == b'{"id":1,"ok":true}\nmore'
        port.write(b"x\n")
        port.close()

    def test_binary_without_a_newline_is_kept(self, server, fast, monkeypatch):
        monkeypatch.setattr(hub_link, "AUTH_LINE_MAX", 8)
        link_key, _ = keys_for(saved=HUB_KEY)
        port, _, _, _ = run_hub(server, link_key, greeting=b"\xff\xaa" * 8, answer=False)
        assert port.read(100) == b"\xff\xaa" * 8
        port.write(b"x\n")
        port.close()

    def test_bad_line_is_kept(self, server, fast):
        link_key, _ = keys_for(saved=HUB_KEY)
        port, _, _, _ = run_hub(server, link_key, greeting=b"\xff\xfe\n", answer=False)
        assert port.read(100) == b"\xff\xfe\n"
        port.write(b"x\n")
        port.close()

    def test_cannot_connect(self):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        url = f"tcp://127.0.0.1:{sock.getsockname()[1]}"
        sock.close()
        with pytest.raises(ConnectionRefusedError):
            open_hub(url, None, "Ace_1", MockLogger())

    def test_bad_spec(self):
        with pytest.raises(ValueError):
            open_hub("tcp://hub.local", None, "Ace_1", MockLogger())

    def test_closes_the_socket_on_failure(self, server, fast):
        made: List[socket.socket] = []
        real = socket.create_connection

        def spy(*args, **kwargs):
            sock = real(*args, **kwargs)
            made.append(sock)
            return sock
        link_key, _ = keys_for(saved="not-the-key")
        with patch.object(hub_link.socket, "create_connection", spy):
            with pytest.raises(HubLinkError):
                run_hub(server, link_key)
        assert made[0].fileno() == -1
