"""Unit tests for extras/AFC_BambuAMS_bridge.py."""

from __future__ import annotations

import json
import logging
import logging.handlers
from pathlib import Path
import queue
import socket
import threading
import time
from typing import Any, Callable, Dict, Iterator, List, Optional, Sequence, Set, Tuple
from unittest.mock import patch

import pytest

from extras import AFC_BambuAMS_bridge as bridge_mod
from extras.AFC_BambuAMS_bridge import (
    _ams_is_noise,
    _CHMB_STATE_RE,
    _DBG_AMSTIME_RE,
    _RFID_CYCLE_END_RE,
    _RFID_FOREIGN_TAG_RE,
    _RFID_READ_OK_RE,
    _RFID_TERMINAL_RE,
    _SerialTimeout,
    _STATE_SWITCH_DONE_RE,
    _STEP,
    BambuBridge,
    parse_bridge_line,
    TcpPort,
)
from tests.bambu_helpers import (
    BambuLogger,
    bridge_sent,
    FakeReactor,
    FakeSerial,
    FakeSocket,
    LogLine,
    make_bambu_bridge,
    make_bambu_unit,
    make_printer,
    make_tcp_port,
    patch_module_time,
)


@pytest.fixture
def bambu_bridge(monkeypatch: pytest.MonkeyPatch) -> BambuBridge:
    """:return BambuBridge: a connected bridge from make_bambu_bridge"""
    return make_bambu_bridge(monkeypatch)


def feed_amsdbg(bridge: BambuBridge, text: str, addr: Optional[int] = None,
                unit: Optional[int] = None) -> None:
    """
    Hand the bridge one narration frame, as the reader thread does.

    :param bridge: the bridge
    :param text: the AMS's narration
    :param addr: device address the frame names; omitted when None
    :param unit: chain index the frame names; omitted when None
    """
    frame: Dict[str, Any] = {"evt": "amsdbg", "text": text}
    if addr is not None:
        frame["addr"] = addr
    if unit is not None:
        frame["unit"] = unit
    bridge.handle_line(json.dumps(frame))


def ams_echo(text: str) -> LogLine:
    """
    :param text: a narration line
    :return LogLine: the line handle_line writes to AFC.log for it
    """
    return ("debug", f"AMS: {text}")


def clocked_tcp_socket(monkeypatch: pytest.MonkeyPatch,
                       script: Sequence[bytes] = (),
                       sock_cls: type = FakeSocket) -> FakeSocket:
    """
    A fake socket whose empty reads advance the clock the bridge module reads.

    :param monkeypatch: pytest's monkeypatch fixture (module time)
    :param script: what the board sends, chunk by chunk
    :param sock_cls: FakeSocket or a subclass of it
    :return FakeSocket: the socket, on a new FakeReactor at 100.0
    """
    reactor = FakeReactor()
    patch_module_time(monkeypatch, reactor)
    return sock_cls(script, clock=reactor)


def tcp_port_on(sock: FakeSocket, *, key: Optional[str] = None,
                write_timeout: float = 0.5) -> TcpPort:
    """
    A TcpPort through its real ``__init__``, connected to ``sock``.

    :param sock: the socket the connect hands back
    :param key: the configured tcp_key
    :param write_timeout: the port's write budget
    :return TcpPort: the port, named tcp://test:8888
    """
    def _connect(address: Tuple[str, int],
                 timeout: Optional[float] = None) -> FakeSocket:
        """:return FakeSocket: ``sock``, whatever the address"""
        return sock

    with patch.object(bridge_mod.socket, "create_connection", _connect):
        return TcpPort("test", 8888, timeout=0.1, write_timeout=write_timeout,
                       connect_timeout=0.5, key=key)


class StalledTcpSocket(FakeSocket):
    """A socket the far end never drains: every send times out."""

    def send(self, data: bytes) -> int:
        """
        Let 0.1 s pass on the clock, then time out.

        :param data: the bytes that will not go
        :return int: never returns
        """
        if self.clock is not None:
            self.clock.advance(0.1)
        raise socket.timeout()


class LoopbackBoard:
    """
    The board's end of a real loopback TCP link, on a port the OS picks.

    Module time must be the real clock here: TcpPort's deadlines are waited
    out on a real socket.
    """

    def __init__(self) -> None:
        """Listen on 127.0.0.1."""
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.server.settimeout(5.0)
        self.port: int = self.server.getsockname()[1]
        self.conn: Optional[socket.socket] = None

    def connect(self, write_timeout: float = 0.5) -> TcpPort:
        """
        Open a keyless TcpPort to this board and accept it.

        :param write_timeout: the port's write budget
        :return TcpPort: the host's end, named tcp://127.0.0.1:<port>
        """
        port = TcpPort("127.0.0.1", self.port, timeout=0.05,
                       write_timeout=write_timeout)
        self.conn, _peer = self.server.accept()
        return port

    @staticmethod
    def fill(port: TcpPort) -> _SerialTimeout:
        """
        Write until the kernel's buffers are full and a write times out.

        :param port: a port whose far end never reads
        :return _SerialTimeout: the timeout that ended it
        """
        deadline = time.monotonic() + 10.0
        with pytest.raises(_SerialTimeout) as exc:
            while time.monotonic() < deadline:
                port.write(b"x" * 65536)
        return exc.value

    def close(self) -> None:
        """Close the accepted connection and the listener."""
        for sock in (self.conn, self.server):
            if sock is not None:
                sock.close()


@pytest.fixture
def loopback_board() -> Iterator[LoopbackBoard]:
    """:return LoopbackBoard: a listening board end, closed afterwards"""
    board = LoopbackBoard()
    yield board
    board.close()


#: What handle_line logs when the first status frame of a connection asks
#: the bridge who it is.
REQUEST_INFO_LOG: LogLine = ("info",
                             'AFC bambu: request_info -> sent {"cmd":"info"}')


class UnstartedBridgeThreads:
    """
    ``threading`` for the bridge module, whose threads are built but never run.

    ``start()`` then leaves its reader for the test to run on its own thread
    (``bridge._thread.target()``), so the loop reads the reactor's clock.
    Everything but ``Thread`` is the real module.
    """

    class Thread:
        """A thread that records its target and does not run it."""

        def __init__(self, target: Callable[[], None], name: str = "",
                     daemon: bool = False) -> None:
            """
            :param target: the loop the thread would run
            :param name: the thread's name
            :param daemon: whether it would be a daemon
            """
            self.target = target
            self.name = name
            self.daemon = daemon
            self.started = False

        def start(self) -> None:
            """Note the start; nothing runs."""
            self.started = True

    def __getattr__(self, name: str) -> Any:
        """:return Any: the real threading module's attribute"""
        return getattr(threading, name)


def unstarted_bridge_threads(monkeypatch: pytest.MonkeyPatch
                             ) -> UnstartedBridgeThreads:
    """
    Make the bridge module build threads that never run.

    :param monkeypatch: pytest's monkeypatch fixture
    :return UnstartedBridgeThreads: the stand-in ``threading``
    """
    threads = UnstartedBridgeThreads()
    monkeypatch.setattr(bridge_mod, "threading", threads)
    return threads


AMS_NARRATION_DIR = Path(__file__).parent / "fixtures" / "ams_narration"


#: The marker a fixture line's `expect` column uses for each pattern.
AMS_NARRATION_PATTERNS = {
    "read": _RFID_READ_OK_RE,
    "end": _RFID_CYCLE_END_RE,
    "foreign": _RFID_FOREIGN_TAG_RE,
}


class AmsNarrationLine:
    """One narration line of a capture, with the markers it should match."""

    def __init__(self, capture: str, index: int, offset: str,
                 expect: Set[str], text: str) -> None:
        """
        :param capture: the capture's name
        :param index: the line's position in the capture
        :param offset: the capture's time column for it
        :param expect: the pattern markers the fixture records for it
        :param text: the narration itself
        """
        self.capture = capture
        self.index = index
        self.offset = offset
        self.expect = expect
        self.text = text

    def __repr__(self) -> str:
        """:return str: capture[index], the test id"""
        return f"{self.capture}[{self.index}]"


class AmsNarrationCapture:
    """A fixture file under fixtures/ams_narration: header and lines."""

    def __init__(self, path: Path) -> None:
        """:param path: the fixture file"""
        self.name = path.stem
        self.meta: Dict[str, str] = {}
        self.lines: List[AmsNarrationLine] = []
        for raw in path.read_text().splitlines():
            raw = raw.strip()
            if not raw:
                continue
            if raw.startswith("#"):
                key, sep, value = raw.lstrip("#").strip().partition(":")
                if sep and key in ("model", "outcome", "source", "verbatim"):
                    self.meta.setdefault(key, value.strip())
                continue
            offset, _, rest = raw.partition("|")
            expect, _, text = rest.partition("|")
            expect = expect.strip()
            markers = set() if expect == "." else set(expect.split())
            self.lines.append(AmsNarrationLine(
                self.name, len(self.lines), offset.strip(), markers,
                text.strip()))
        # A typo here would silently drop the capture from every outcome's
        # tests, so a malformed header fails collection instead.
        verbatim = self.meta.get("verbatim", "").split()[:1]
        if (self.meta.get("model") not in ("ams1", "ams2", "ht")
                or self.outcome not in ("read", "notag", "foreign")
                or not self.meta.get("source")
                or verbatim not in (["yes"], ["abbreviated"])):
            error_str = f"{path}: malformed header {self.meta}"
            raise ValueError(error_str)

    @property
    def outcome(self) -> str:
        """:return str: the outcome the header declares"""
        return self.meta.get("outcome", "")

    def first(self, marker: str) -> Optional[int]:
        """
        :param marker: read, end or foreign
        :return Optional[int]: the first line the marker's real pattern
          matches, or None
        """
        pattern = AMS_NARRATION_PATTERNS[marker]
        for line in self.lines:
            if pattern.search(line.text):
                return line.index
        return None

    def __repr__(self) -> str:
        """:return str: the capture's name, the test id"""
        return self.name


AMS_NARRATION_CAPTURES = [AmsNarrationCapture(p)
                          for p in sorted(AMS_NARRATION_DIR.glob("*.txt"))]


def ams_narration_captures_with(outcome: str) -> List[AmsNarrationCapture]:
    """
    The captures whose header declares ``outcome``; never none.

    :param outcome: read, notag or foreign
    :return List[AmsNarrationCapture]: those captures, in file order
    :raises ValueError: when no capture declares it, so a moved or emptied
      fixture directory cannot leave a test with nothing to run
    """
    caps = [c for c in AMS_NARRATION_CAPTURES if c.outcome == outcome]
    if not caps:
        error_str = f"no {outcome} capture under {AMS_NARRATION_DIR}"
        raise ValueError(error_str)
    return caps


AMS_NARRATION_LINES = [ln for cap in AMS_NARRATION_CAPTURES
                       for ln in cap.lines]


class TestParseBridgeLine:
    def test_valid_object(self):
        assert parse_bridge_line('{"evt":"info","fw":"0.1.0"}') == {
            "evt": "info", "fw": "0.1.0"}

    def test_blank_is_none(self):
        assert parse_bridge_line("   ") is None
        assert parse_bridge_line("") is None

    def test_invalid_json_is_none(self):
        assert parse_bridge_line("{not json") is None

    def test_non_object_is_none(self):
        assert parse_bridge_line("[1,2,3]") is None
        assert parse_bridge_line('"a string"') is None

    def test_a_tx_line_parses_as_a_capture_line(self):
        # Every field the capture tooling reads off a sniff line survives.
        line = ('{"evt":"tx","us":10322878545,"n":13,'
                '"hex":"3DC50DF10400077F03000211BC"}\r\n')
        assert parse_bridge_line(line) == {
            "evt": "tx", "us": 10322878545, "n": 13,
            "hex": "3DC50DF10400077F03000211BC"}

    def test_the_length_field_is_the_REAL_length(self):
        # The ring keeps at most 32 bytes but records the true length, so a
        # parsed line still says "44 bytes, 32 kept".
        kept = "AB" * 32
        obj = parse_bridge_line(f'{{"evt":"tx","us":1,"n":44,"hex":"{kept}"}}')
        assert obj == {"evt": "tx", "us": 1, "n": 44, "hex": kept}
        assert len(bytes.fromhex(obj["hex"])) == 32


class TestStep:
    @pytest.mark.parametrize("line", [
        "[AMS_RFID] STEP4,read success",
        "[AMS_RFID]STEP:read success",
        "[AMS_DEV] STEP:read success",
        "[AMS_DEV]STEP2: read success",
        "[AMS_SWITCH] STEP: read success",
    ])
    def test_step_helper_tolerates_every_punctuation_seen(self, line):
        match = _STEP("read success").search(line)
        assert match is not None
        assert match.group(0) == line

    def test_step_helper_does_not_match_a_different_event(self):
        rx = _STEP("read success")
        assert rx.search("[AMS_DEV] STEP:odom search, odo 1.856") is None
        assert rx.search("[AMS_RFID] STEP3,search 1 card") is None


class TestAmsIsNoise:
    #: Print-time chatter, verbatim off a live print.
    PRINT_NOISE = (
        "l [AMS_COMMON]state:4,tray_now:1,tray_exit:15 [AMS_LED]tray 1 loading"
        " [AMS_PMSM]mode:0->2 [AMS_PMSM]mode:2->0",
        "[AMS_PMSM]mode:0->2",
        "C [AMS_LED]tray 1 loading",
        "n [AMS_SWITCH]BUFF,pos:0.10->0.73,det:20mm,i:0.635A [AMS_PMSM]mode:2->0",
        "[AMS_COMMON]en:0,mode:0,idx:1,ref:0 [AMS_COMMON]preload_disable:1,"
        " tmpr:22.0, cd:0 [AMS_COMMON]state:0,tray_now:1,tray_exit:15",
    )

    @pytest.mark.parametrize("line", [
        "[AMS_CALL] ams0 select,select ams1 [AMS_CALL] ams0 select,select ams1",
        "# [AMS_CALL] ams0 select,select ams0",
        "s [AMS_COMMON]mode: 4 -> 0 [AMS_COMMON]ref: 128 -> 128",
        "[AMS_IDLE]set ams state switch",
        "[AMS_COMMON]mode: 0 -> 4 [AMS_COMMON]ref: 128 -> 128 "
        "[AMS_LINK]ams0 select,req ams0",
    ])
    def test_pure_chatter_is_console_suppressed(self, line):
        assert _ams_is_noise(line) is True

    @pytest.mark.parametrize("line", [
        # Chatter bundled with real narration must survive.
        "[AMS_CALL] ams0 select,select ams0 [AMS_DEV] STEP:set 0 tray_preload",
        "g [AMS_DEV] STEP2:pull tray 0 from switch [AMS_DEV] STEP:rfid pull 0",
        "\\ [AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:23.0",
        "[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m",
        "< [AMS_TRAY]tray[0] sw_sta update, 0 -> 1, u_in_out:1,0",
        "[AMS_CHMB]ignore dry_mode:1, ams_state:2",
        "[AMS_DOOR]wind_door[1] closing [AMS_BDC_OFF]BDC offline isr enter",
    ])
    def test_anything_informative_stays_on_the_console(self, line):
        assert _ams_is_noise(line) is False

    def test_text_without_any_bracket_is_not_treated_as_noise(self):
        assert _ams_is_noise("some unstructured reply") is False
        assert _ams_is_noise("") is False

    def test_link_chatter_is_still_visible_not_filtered(self):
        # A repeating get_slot is a stuck re-read; the dedupe handles its
        # repetition, so it must stay reportable.
        assert _ams_is_noise("[AMS_LINK]get_slot ams1 tray0 basic") is False

    def test_the_90s_preload_housekeeping_is_console_suppressed(self):
        assert _ams_is_noise(
            "^ [AMS_COMMON]preload_disable:1, tmpr:25.8, cd:0 "
            "[AMS_COMMON]preload_disable:0, tmpr:25.8, cd:0") is True

    @pytest.mark.parametrize("line", [
        "[AMS_RFID] STEP3,save to flash ,card info valid",
        "[AMS_COMMON]state:6,tray_now:255,tray_exit:1",
        "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1",
    ])
    def test_the_filter_does_not_reach_lines_that_matter(self, line):
        assert _ams_is_noise(line) is False

    def test_print_time_chatter_is_console_suppressed(self):
        assert [_ams_is_noise(line) for line in self.PRINT_NOISE] == [True] * 5

    @pytest.mark.parametrize("line", [
        "[AMS_COMMON]state:6,tray_now:255,tray_exit:15",
        "[AMS_COMMON]state:1,tray_now:1,tray_exit:15",
        "[AMS_COMMON]state:7,tray_now:1,tray_exit:15",
        "[AMS_RFID]STEP:odom C:0.478,R:0.076,P:79%, od:0.491",
        "[AMS_SWITCH]feed finish -1, stall, len_det:1.620 m, tube_len:3.506 m",
    ])
    def test_faults_and_results_are_never_suppressed(self, line):
        assert _ams_is_noise(line) is False

    @pytest.mark.parametrize("line", PRINT_NOISE)
    def test_chatter_carrying_a_heartbeat_is_still_chatter(self, bambu_bridge, line):
        # The AMS bundles its 10 s heartbeat into whatever frame goes out, and
        # the raw line no longer reads as noise; handle_line strips it first.
        beating = f"{line} [DBG] ams time: now=42044054ms diff=10005ms"
        assert _ams_is_noise(beating) is False
        feed_amsdbg(bambu_bridge, beating, addr=0x0700)
        assert bambu_bridge.logger.messages == [("debug", f"AMS: {line}")]
        assert bambu_bridge.logger.file_only == [f"AMS: {line}"]


class TestTcpPortParse:
    @pytest.mark.parametrize("spec, want", [
        ("tcp://192.168.1.50:8888", ("192.168.1.50", 8888)),
        ("TCP://bridgebox.local:9000", ("bridgebox.local", 9000)),
        ("tcp://192.168.1.50", ("192.168.1.50", 8888)),
        ("tcp://192.168.1.50:8888/", ("192.168.1.50", 8888)),
        ("192.168.1.50:8888", ("192.168.1.50", 8888)),
        ("tcp://[fe80::1]:8888", ("fe80::1", 8888)),
        ("tcp://[fe80::1]", ("fe80::1", 8888)),
        ("tcp://fe80::1:2:3", ("fe80::1:2:3", 8888)),
    ])
    def test_parse_accepts_the_forms_people_will_write(self, spec, want):
        assert TcpPort.parse(spec) == want

    @pytest.mark.parametrize("spec, message", [
        ("tcp://", "no host in bridge address 'tcp://'"),
        ("tcp://:8888", "no host in bridge address 'tcp://:8888'"),
        ("tcp://host:nope", "bad port in bridge address 'tcp://host:nope'"),
    ])
    def test_parse_rejects_nonsense(self, spec, message):
        with pytest.raises(ValueError) as exc:
            TcpPort.parse(spec)
        assert str(exc.value) == message


class TestTcpPortInit:
    def test_connect_failure_raises_so_the_reader_backs_off(self):
        # The reader's reconnect loop relies on the factory raising while
        # the bridge is absent. Nothing listens on a port just released.
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind(("127.0.0.1", 0))
        host, port = srv.getsockname()
        srv.close()
        with pytest.raises(OSError):
            TcpPort(host, port, connect_timeout=1.0)

    def test_the_socket_is_set_up_for_the_bridge(self, monkeypatch):
        port = make_tcp_port(monkeypatch=monkeypatch)
        assert port.name == "tcp://test:8888"
        assert (port._timeout, port._write_timeout) == (0.1, 0.5)
        # The handshake's own 0.2 s poll is put back to the read timeout.
        assert port._sock.timeout == 0.1
        assert port._sock.options == [
            (socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),
            (socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1),
            (socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 20),
            (socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 5),
            (socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)]
        assert port._pushback == b""


class TestTcpPortAuthenticate:
    NONCE = "000102030405060708090a0b0c0d0e0f"
    OTHER_NONCE = "0f0e0d0c0b0a09080706050403020100"
    OK = b'{"evt":"auth","ok":1}\n'

    @staticmethod
    def _challenge(nonce: str) -> bytes:
        """:return bytes: the board's challenge line for ``nonce``"""
        return f'{{"evt":"auth","nonce":"{nonce}"}}\n'.encode()

    # Known answers: HMAC-SHA512 of the nonce bytes, first 32 bytes in hex,
    # computed outside Python with openssl dgst -sha512 -mac HMAC.
    ANSWER_HUNTER2 = (b'{"cmd":"auth","mac":"8a61ee1259a6fa07c57a5c98107858ef'
                      b'609ba3bbd3fb2513f93c17299b222a0f"}\n')
    ANSWER_WRONG = (b'{"cmd":"auth","mac":"c914d5412fdb8db3adad0a830b9d1d80'
                    b'8715953b2959969d5edc017f39170e97"}\n')
    ANSWER_K = (b'{"cmd":"auth","mac":"1f8fe778e9b9e5f27dceddaea1537818'
                b'33f20d9786a215889b97812e1ced7aca"}\n')
    ANSWER_K_OTHER = (b'{"cmd":"auth","mac":"9f54865a90f40521b1d8f377d5482afd'
                      b'b724c871b6947b869c66b131057efccc"}\n')

    def test_a_correct_key_authenticates(self, monkeypatch):
        port = make_tcp_port([self._challenge(self.NONCE), self.OK],
                             key="hunter2", monkeypatch=monkeypatch)
        assert port._sock.sent == self.ANSWER_HUNTER2
        assert port._sock.timeout == 0.1
        assert port._pushback == b""

    def test_THE_POINT_a_wrong_key_is_refused(self, monkeypatch):
        sock = clocked_tcp_socket(monkeypatch, [
            self._challenge(self.NONCE), b'{"evt":"auth","ok":0}\n'])
        with pytest.raises(OSError) as exc:
            tcp_port_on(sock, key="wrong")
        assert str(exc.value) == "tcp://test:8888: the bridge rejected the link key"
        assert sock.sent == self.ANSWER_WRONG
        assert sock.timeout == 0.1

    def test_a_keyed_board_with_no_configured_key_is_an_error(self, monkeypatch):
        # Fails loudly rather than hanging on a link that never comes up.
        sock = clocked_tcp_socket(monkeypatch, [self._challenge(self.NONCE)])
        with pytest.raises(OSError) as exc:
            tcp_port_on(sock)
        assert str(exc.value) == (
            "tcp://test:8888: the bridge asked for a link key and none is "
            "configured -- set tcp_key to match the board")
        assert sock.sent == b""

    def test_an_open_board_still_works_with_no_key(self, monkeypatch):
        # Silence means an unkeyed board; with no key there is nothing to
        # wait for, so the first quiet poll ends the handshake.
        reactor = FakeReactor()
        port = make_tcp_port(key=None, monkeypatch=monkeypatch, reactor=reactor)
        assert port._sock.sent == b""
        assert reactor.now == pytest.approx(100.2)

    def test_an_open_board_works_even_when_a_key_IS_configured(self, monkeypatch):
        # Config updated before the board is keyed: the board never
        # challenges, so the handshake waits out its 2 s and sends nothing.
        reactor = FakeReactor()
        port = make_tcp_port(key="hunter2", monkeypatch=monkeypatch,
                             reactor=reactor)
        assert port._sock.sent == b""
        assert reactor.now == pytest.approx(102.0)
        assert port._pushback == b""

    def test_the_key_itself_never_goes_on_the_wire(self, monkeypatch):
        port = make_tcp_port([self._challenge(self.NONCE), self.OK],
                             key="hunter2", monkeypatch=monkeypatch)
        assert b"hunter2" not in port._sock.sent
        assert port._sock.sent == self.ANSWER_HUNTER2

    def test_a_closed_socket_mid_handshake_raises(self, monkeypatch):
        sock = clocked_tcp_socket(monkeypatch, [b""])
        with pytest.raises(OSError) as exc:
            tcp_port_on(sock, key="hunter2")
        assert str(exc.value) == "tcp://test:8888: closed during authentication"
        assert sock.sent == b""

    def test_a_close_while_waiting_for_the_verdict_raises(self, monkeypatch):
        sock = clocked_tcp_socket(monkeypatch, [self._challenge(self.NONCE), b""])
        with pytest.raises(OSError) as exc:
            tcp_port_on(sock, key="hunter2")
        assert str(exc.value) == "tcp://test:8888: closed during authentication"
        assert sock.sent == self.ANSWER_HUNTER2

    def test_a_frame_arriving_instead_of_a_challenge_is_not_eaten(self, monkeypatch):
        # An open board already talking: what the handshake over-read must
        # come back out of read().
        frame = b'{"evt":"status","online":true}\n'
        port = make_tcp_port([frame], key="hunter2", monkeypatch=monkeypatch)
        assert port._sock.sent == b""
        assert port._pushback == frame
        assert port.read(4096) == frame
        assert port._pushback == b""

    @pytest.mark.parametrize("frame", [
        b'{"evt":"auth","ok":1}\n',
        f'{{"evt":"hello","nonce":"{NONCE}"}}\n'.encode(),
    ], ids=["auth without a nonce", "a nonce without auth"])
    def test_a_challenge_needs_both_auth_and_a_nonce(self, monkeypatch, frame):
        # Either word alone is a frame, not a challenge: nothing is answered.
        port = make_tcp_port([frame], key="hunter2", monkeypatch=monkeypatch)
        assert port._sock.sent == b""
        assert port._pushback == frame
        assert port.read(4096) == frame

    def test_the_response_is_bound_to_THIS_nonce(self, monkeypatch):
        # Replay protection: the same key answers a different challenge
        # differently, so a recorded session cannot be reused.
        first = make_tcp_port([self._challenge(self.NONCE), self.OK],
                              key="k", monkeypatch=monkeypatch)
        second = make_tcp_port([self._challenge(self.OTHER_NONCE), self.OK],
                               key="k", monkeypatch=monkeypatch)
        assert first._sock.sent == self.ANSWER_K
        assert second._sock.sent == self.ANSWER_K_OTHER
        assert first._sock.sent != second._sock.sent


class TestTcpPortRead:
    def test_read_returns_what_was_sent(self, monkeypatch):
        port = make_tcp_port(monkeypatch=monkeypatch)
        port._sock.inbox.append(b'{"evt":"ack"}\n')
        assert port.read(64) == b'{"evt":"ack"}\n'
        assert port._sock.inbox == []

    def test_read_returns_empty_on_timeout_not_an_error(self, monkeypatch):
        # A bridge with nothing to say must not look like a broken one: the
        # read gives up after its own 0.1 s timeout.
        reactor = FakeReactor()
        port = make_tcp_port(monkeypatch=monkeypatch, reactor=reactor)
        assert reactor.now == pytest.approx(100.2)
        assert port.read(64) == b""
        assert reactor.now == pytest.approx(100.3)

    def test_read_raises_at_end_of_stream(self, monkeypatch):
        # The reconnect trigger: b"" here would spin the reader on a dead
        # bridge.
        port = make_tcp_port(monkeypatch=monkeypatch)
        port._sock.inbox.append(b"")
        with pytest.raises(OSError) as exc:
            port.read(64)
        assert str(exc.value) == "tcp://test:8888: bridge closed the connection"

    def test_read_raises_at_a_real_end_of_stream(self, loopback_board):
        # The kernel's own end of stream, after the board closes its end.
        port = loopback_board.connect()
        loopback_board.conn.close()
        deadline = time.monotonic() + 3.0
        with pytest.raises(OSError) as exc:
            while time.monotonic() < deadline:
                port.read(64)
        assert str(exc.value) == (
            f"tcp://127.0.0.1:{loopback_board.port}: bridge closed the connection")
        port.close()


class TestTcpPortWrite:
    class _ZeroSendSocket(FakeSocket):
        """A socket that reports sending nothing: the far end is gone."""

        def send(self, data: bytes) -> int:
            """:return int: 0, no bytes taken"""
            return 0

    def test_write_delivers_whole_lines(self, monkeypatch):
        port = make_tcp_port(monkeypatch=monkeypatch)
        assert port.write(b'{"cmd":"prime","slot":2}\n') == 25
        assert port._sock.sent == b'{"cmd":"prime","slot":2}\n'

    def test_write_raises_serial_timeout_when_the_far_end_stops_reading(
            self, monkeypatch):
        # The same exception a stalled Pico raises, so the writer reads it
        # as "busy", not as a broken link.
        sock = clocked_tcp_socket(monkeypatch, sock_cls=StalledTcpSocket)
        port = tcp_port_on(sock, write_timeout=0.2)
        with pytest.raises(_SerialTimeout) as exc:
            port.write(b"x" * 64)
        assert str(exc.value) == "tcp://test:8888: Write timeout"
        assert sock.sent == b""

    def test_a_real_far_end_that_stops_reading_times_the_write_out(
            self, loopback_board):
        # Accepted and never read from: the kernel's full send buffer, not a
        # fake's exception, is what the write gives up on.
        port = loopback_board.connect(write_timeout=0.2)
        exc = loopback_board.fill(port)
        assert str(exc) == f"tcp://127.0.0.1:{loopback_board.port}: Write timeout"
        port.close()

    def test_a_send_of_nothing_is_a_closed_connection(self, monkeypatch):
        sock = clocked_tcp_socket(monkeypatch, sock_cls=self._ZeroSendSocket)
        port = tcp_port_on(sock)
        with pytest.raises(OSError) as exc:
            port.write(b"x")
        assert str(exc.value) == "tcp://test:8888: bridge closed the connection"


class TestTcpPortClose:
    class _DeadSocket(FakeSocket):
        """A socket the OS already tore down: close attempts are counted."""

        def __init__(self, script: Sequence[bytes] = (),
                     clock: Optional[FakeReactor] = None) -> None:
            """
            :param script: the chunks recv returns, in order
            :param clock: the clock an empty recv advances
            """
            super().__init__(script, clock)
            self.close_calls = 0

        def close(self) -> None:
            """Refuse, as closing a dead descriptor does."""
            self.close_calls += 1
            error_str = "Bad file descriptor"
            raise OSError(error_str)

    def test_close_is_idempotent(self, monkeypatch):
        port = make_tcp_port(monkeypatch=monkeypatch)
        port.close()
        port.close()
        assert port._sock.closed is True

    def test_closing_a_real_socket_twice_is_idempotent(self, loopback_board):
        port = loopback_board.connect()
        port.close()
        port.close()
        assert port._sock.fileno() == -1
        # The board sees the link end.
        loopback_board.conn.settimeout(3.0)
        assert loopback_board.conn.recv(16) == b""

    def test_a_dead_socket_is_not_an_error(self, monkeypatch):
        sock = clocked_tcp_socket(monkeypatch, sock_cls=self._DeadSocket)
        port = tcp_port_on(sock)
        port.close()
        # The close was tried and its OSError swallowed.
        assert sock.close_calls == 1
        assert sock.closed is False


class TestBambuBridgeInit:
    def test_a_freshly_constructed_bridge_has_a_name(self, monkeypatch):
        # Narration renders "AFC bambu <name>: ..."; built as production
        # builds it, with nobody assigning a name afterwards.
        reactor = FakeReactor()
        patch_module_time(monkeypatch, reactor)
        bridge = BambuBridge(FakeSerial, reactor, BambuLogger())
        assert bridge.name == "bridge"
        assert bridge.logger.messages == []

    def test_the_constructor_runs_and_seeds_its_records(self, monkeypatch):
        reactor = FakeReactor(now=250.0)
        patch_module_time(monkeypatch, reactor)
        logger = BambuLogger()
        bridge = BambuBridge(FakeSerial, reactor, logger)
        assert (bridge.reactor, bridge.logger) == (reactor, logger)
        assert (bridge._serial, bridge._thread, bridge._wthread) == (None, None, None)
        assert bridge._run is False
        # Stamped "never connected" at construction, on module time.
        assert bridge._down_t == 250.0
        assert bridge.down_since() == 250.0
        assert bridge.last_finish() == (0, False, "")
        assert bridge.last_fault() == (0, "", 0.0)
        assert bridge.last_err_code() == (None, 0.0)
        assert bridge.latest_status() is None
        assert bridge._wq.qsize() == 0
        assert logger.messages == []


class TestBambuBridgeStart:
    class _TimedPort:
        """A serial port the reader blocks on: queued chunks, else a short wait."""

        def __init__(self, chunks: Sequence[bytes] = ()) -> None:
            """:param chunks: what reads hand back, in order"""
            self.chunks: List[bytes] = list(chunks)
            self.written: List[bytes] = []
            self.closed = False
            self.fail_read: Optional[BaseException] = None

        def read(self, size: int = 1) -> bytes:
            """:return bytes: the next chunk, or b"" after a 10 ms wait"""
            if self.fail_read is not None:
                raise self.fail_read
            if self.chunks:
                return self.chunks.pop(0)
            time.sleep(0.01)
            return b""

        def write(self, data: bytes) -> int:
            """:return int: bytes written"""
            self.written.append(bytes(data))
            return len(data)

        def close(self) -> None:
            """Mark the port closed."""
            self.closed = True

    @staticmethod
    def _stop(bridge: BambuBridge) -> None:
        """Stop both threads and wait for them to leave their loops."""
        bridge._run = False
        bridge.stop()
        for thread in (bridge._thread, bridge._wthread):
            if thread is not None:
                thread.join(timeout=5.0)

    @staticmethod
    def _wait_for(check: Callable[[], bool], seconds: float = 5.0) -> bool:
        """:return bool: whether ``check`` came true within ``seconds``"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if check():
                return True
            time.sleep(0.01)
        return check()

    def test_bridge_runs_end_to_end_over_tcp(self):
        # Real threads on a loopback TcpPort: a command goes out, a non-frame
        # line is skipped and a frame split across two segments is dispatched.
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        srv.settimeout(5.0)
        host, port = srv.getsockname()
        logger = BambuLogger()
        bridge = BambuBridge(lambda: TcpPort(host, port, timeout=0.05),
                             FakeReactor(), logger)
        conn = None
        try:
            bridge.start()
            assert bridge.is_connected() is True
            assert bridge.down_since() is None
            assert bridge._thread.is_alive() and bridge._wthread.is_alive()
            conn, _peer = srv.accept()
            conn.settimeout(5.0)
            bridge.send({"cmd": "status"})
            got = b""
            while b"\n" not in got:
                got += conn.recv(256)
            assert got == b'{"cmd": "status"}\n'
            conn.sendall(b"rp2040 boot banner\n")
            conn.sendall(b'{"evt":"ack","cmd":"stat')
            time.sleep(0.1)
            conn.sendall(b'us","slot":-1}\n')
            assert self._wait_for(lambda: bool(logger.messages))
            assert logger.messages == [
                ("debug", "AFC bambu: bridge ack status (slot -1)")]
            assert logger.file_only == []
        finally:
            self._stop(bridge)
            for sock in (conn, srv):
                if sock is not None:
                    sock.close()

    def test_start_defers_a_failed_first_connect_and_the_reader_gets_there(
            self, monkeypatch):
        # A network bridge not up yet at Klipper start must not fail the
        # unit: the reader's backoff loop connects once it is there.
        reactor = FakeReactor()
        patch_module_time(monkeypatch, reactor)
        port = self._TimedPort([b'{"evt":"ack","cmd":"late","slot":0}\n'])
        outcomes: List[Any] = [OSError("no bridge yet"),
                               OSError("no bridge yet"), port]

        def factory() -> Any:
            """:return Any: the next scripted port, or raise the next error"""
            nxt = outcomes.pop(0)
            if isinstance(nxt, BaseException):
                raise nxt
            return nxt

        logger = BambuLogger()
        bridge = BambuBridge(factory, reactor, logger)
        try:
            bridge.start(defer_open=True)
            assert self._wait_for(lambda: len(logger.messages) == 3)
            # start's own attempt, the reader's failed one, then its success.
            assert outcomes == []
            assert bridge._serial is port
            # One failed reader attempt, then its 0.5 s backoff on the clock.
            assert reactor.now == pytest.approx(100.5)
            assert logger.messages == [
                ("warning", "AFC bambu: bridge not reachable yet (no bridge "
                            "yet); the reader will keep trying"),
                ("info", "AFC bambu: bridge reconnected"),
                ("debug", "AFC bambu: bridge ack late (slot 0)")]
        finally:
            self._stop(bridge)

    def test_start_still_raises_for_a_missing_usb_port(self, monkeypatch):
        # A device path that is not there is a configuration fault, not a
        # bridge still booting, so it stays loud.
        def boom() -> Any:
            """Fail as a missing tty does."""
            error_str = "no such device: /dev/serial/by-id/nope"
            raise OSError(error_str)

        patch_module_time(monkeypatch, FakeReactor())
        logger = BambuLogger()
        bridge = BambuBridge(boom, FakeReactor(), logger)
        with pytest.raises(OSError) as exc:
            bridge.start()
        assert str(exc.value) == "no such device: /dev/serial/by-id/nope"
        assert (bridge._thread, bridge._wthread, bridge._serial) == (None, None, None)
        assert bridge._run is False
        assert logger.messages == []

    def test_a_successful_start_clears_the_never_connected_stamp(self, monkeypatch):
        # _down_t is stamped at construction; left set, the first outage
        # would report the object's age instead of its own.
        reactor = FakeReactor()
        patch_module_time(monkeypatch, reactor)
        port = self._TimedPort()
        release = threading.Event()
        calls: List[int] = []

        def factory() -> Any:
            """:return Any: the port once; later calls wait, then fail"""
            calls.append(1)
            if len(calls) == 1:
                return port
            release.wait(timeout=5.0)
            error_str = "bridge gone"
            raise OSError(error_str)

        logger = BambuLogger()
        bridge = BambuBridge(factory, reactor, logger)
        assert bridge._down_t == 100.0
        reactor.advance(200.0)
        try:
            bridge.start()
            assert bridge._down_t is None
            assert bridge.down_since() is None
            assert bridge._connected_t == 300.0
            reactor.advance(10.0)
            port.fail_read = OSError("bridge closed the connection")
            # Close is the last thing _drop_port does: the stamp is set by then.
            assert self._wait_for(lambda: port.closed)
            assert bridge.is_connected() is False
            assert bridge.down_since() == 310.0
            assert logger.messages == [
                ("warning", "AFC bambu: bridge read failed: bridge closed the "
                            "connection; reconnecting")]
        finally:
            release.set()
            self._stop(bridge)


class TestBambuBridgeWriter:
    class _StalledPort:
        """Takes the bytes, then times out: a CDC the Pico stopped reading."""

        write_timeout: Optional[float] = 0.5

        def __init__(self) -> None:
            """Start with nothing written."""
            self.written: List[bytes] = []
            self.closed = False

        def write(self, data: bytes) -> int:
            """Record ``data``, then time out as pyserial does."""
            self.written.append(bytes(data))
            error_str = "Write timeout"
            raise _SerialTimeout(error_str)

        def close(self) -> None:
            """Mark the port closed."""
            self.closed = True

    class _StalledTcpStylePort(_StalledPort):
        """TcpPort's spelling of the same budget."""

        write_timeout = None
        _write_timeout = 0.5

    @staticmethod
    def _run_writer(bridge: BambuBridge) -> None:
        """Run the real writer loop on this thread until its queue is empty."""
        real_get = bridge._wq.get

        def get(block: bool = True, timeout: Optional[float] = None) -> Any:
            """:return Any: the next item; ends the loop once drained"""
            if bridge._wq.qsize() == 0:
                bridge._run = False
                raise queue.Empty
            return real_get(block, timeout)

        bridge._wq.get = get
        bridge._run = True
        bridge._writer()

    def test_the_command_and_the_timeout_are_named(self, bambu_bridge):
        port = self._StalledPort()
        bambu_bridge._serial = port
        bambu_bridge.send({"cmd": "chain"})
        self._run_writer(bambu_bridge)
        assert port.written == [b'{"cmd": "chain"}\n']
        line = ("AFC bambu: bridge busy, write of 'chain' timed out after "
                "0.5 s (may still be delivered)")
        assert bambu_bridge.logger.messages == [("debug", line)]
        # AFC.log only: it nearly always lands late.
        assert bambu_bridge.logger.file_only == [line]
        # A timeout keeps the port, and still stamps and counts.
        assert bambu_bridge._serial is port and port.closed is False
        assert bambu_bridge._write_drop_t == 100.0
        assert bambu_bridge._write_timeouts == 1
        assert bambu_bridge.writes_dropped_since(100.0) is True
        assert bambu_bridge.writes_dropped_since(100.5) is False
        assert bambu_bridge.writes_dropped_since(None) is False

    @pytest.mark.parametrize("item", [b"x", b"\xff\xfe\n", b'{"slot": 1}\n',
                                      b"[1, 2]\n", b'{"cmd": ""}\n'])
    def test_an_unnameable_item_does_not_break_the_writer(self, bambu_bridge, item):
        port = self._StalledPort()
        bambu_bridge._serial = port
        bambu_bridge._wq.put_nowait(item)
        bambu_bridge.send({"cmd": "stop"})
        self._run_writer(bambu_bridge)
        assert port.written == [item, b'{"cmd": "stop"}\n']
        assert bambu_bridge.logger.messages == [
            ("debug", "AFC bambu: bridge busy, write of '?' timed out after "
                      "0.5 s (may still be delivered)"),
            ("debug", "AFC bambu: bridge busy, write of 'stop' timed out after "
                      "0.5 s (may still be delivered)")]
        assert bambu_bridge._write_timeouts == 2

    def test_a_port_that_does_not_say_its_timeout_is_not_guessed(self, bambu_bridge):
        port = self._StalledPort()
        port.write_timeout = None
        bambu_bridge._serial = port
        bambu_bridge.send({"cmd": "chain"})
        self._run_writer(bambu_bridge)
        assert bambu_bridge.logger.messages == [
            ("debug", "AFC bambu: bridge busy, write of 'chain' timed out "
                      "(may still be delivered)")]

    def test_a_zero_timeout_is_not_named(self, bambu_bridge):
        # A number, but no budget: "after 0 s" would be wrong.
        port = self._StalledPort()
        port.write_timeout = 0
        bambu_bridge._serial = port
        bambu_bridge.send({"cmd": "chain"})
        self._run_writer(bambu_bridge)
        assert bambu_bridge.logger.messages == [
            ("debug", "AFC bambu: bridge busy, write of 'chain' timed out "
                      "(may still be delivered)")]
        assert bambu_bridge._write_timeouts == 1

    def test_the_tcp_transport_is_named_too(self, bambu_bridge):
        bambu_bridge._serial = self._StalledTcpStylePort()
        bambu_bridge.send({"cmd": "info"})
        self._run_writer(bambu_bridge)
        assert bambu_bridge.logger.messages == [
            ("debug", "AFC bambu: bridge busy, write of 'info' timed out after "
                      "0.5 s (may still be delivered)")]

    def test_a_timed_out_tcp_write_is_named_and_keeps_the_link(self, bambu_bridge):
        # A real TcpPort whose far end stopped reading.
        sock = StalledTcpSocket(clock=bambu_bridge.reactor)
        port = tcp_port_on(sock, write_timeout=0.2)
        bambu_bridge._serial = port
        bambu_bridge.send({"cmd": "chain"})
        self._run_writer(bambu_bridge)
        assert bambu_bridge.logger.messages == [
            ("debug", "AFC bambu: bridge busy, write of 'chain' timed out after "
                      "0.2 s (may still be delivered)")]
        assert bambu_bridge._serial is port
        assert sock.closed is False

    def test_a_real_socket_that_stopped_draining_keeps_the_link(self, loopback_board):
        # Built without make_bambu_bridge: the real socket's write deadline
        # needs module time on the real clock.
        port = loopback_board.connect(write_timeout=0.2)
        loopback_board.fill(port)
        bridge = BambuBridge(lambda: port, FakeReactor(), BambuLogger())
        bridge._serial = port
        bridge.send({"cmd": "chain"})
        self._run_writer(bridge)
        line = ("AFC bambu: bridge busy, write of 'chain' timed out after "
                "0.2 s (may still be delivered)")
        assert bridge.logger.messages == [("debug", line)]
        assert bridge.logger.file_only == [line]
        assert bridge._serial is port
        assert port._sock.fileno() != -1
        assert bridge._write_timeouts == 1
        port.close()

    def test_a_failed_write_drops_the_port(self, bambu_bridge):
        port = FakeSerial(fail_write=OSError("[Errno 5] Input/output error"))
        bambu_bridge._serial = port
        bambu_bridge.send({"cmd": "chain"})
        self._run_writer(bambu_bridge)
        assert bambu_bridge.logger.messages == [
            ("warning", "AFC bambu: bridge write failed: [Errno 5] Input/output "
                        "error; reconnecting")]
        assert bambu_bridge._serial is None
        assert port.closed is True
        assert bambu_bridge.down_since() == 100.0
        assert bambu_bridge._write_timeouts == 0

    def test_an_item_for_a_dropped_port_is_discarded(self, bambu_bridge):
        bambu_bridge._serial = None
        bambu_bridge._wq.put_nowait(b'{"cmd": "chain"}\n')
        self._run_writer(bambu_bridge)
        assert bambu_bridge._wq.qsize() == 0
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeStop:
    class _GonePort(FakeSerial):
        """A port whose close throws: the device is already gone."""

        close_calls = 0

        def close(self) -> None:
            """Count the attempt, then refuse to close."""
            self.close_calls += 1
            error_str = "already gone"
            raise OSError(error_str)

    def test_stop_closes_the_port(self, bambu_bridge):
        port = bambu_bridge._serial
        bambu_bridge._run = True
        bambu_bridge.stop()
        assert bambu_bridge._run is False
        assert port.closed is True
        assert bambu_bridge.logger.messages == []

    def test_stop_survives_a_close_that_throws(self, bambu_bridge):
        port = self._GonePort()
        bambu_bridge._serial = port
        bambu_bridge._run = True
        bambu_bridge.stop()
        assert bambu_bridge._run is False
        assert port.close_calls == 1
        assert port.closed is False
        assert bambu_bridge._serial is port
        assert bambu_bridge.logger.messages == []

    def test_stop_with_no_port_is_a_noop(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, connected=False)
        bridge._run = True
        bridge.stop()
        assert bridge._run is False
        assert bridge._serial is None
        assert bridge.logger.messages == []


class TestBambuBridgeLastErrCode:
    def test_never_reported_is_none(self, bambu_bridge):
        # None means never heard, which is not the same as healthy.
        assert bambu_bridge.last_err_code() == (None, 0.0)
        assert bambu_bridge.logger.messages == []

    def test_reports_the_level_and_its_time(self, bambu_bridge):
        bambu_bridge.reactor.advance(55.5)
        feed_amsdbg(bambu_bridge, "[AMS_LINK]err_code: 0 -> 23", addr=0x1800)
        assert bambu_bridge.last_err_code() == (23, 155.5)
        # The hex form, back to healthy: zero, not None.
        bambu_bridge.reactor.advance(4.5)
        feed_amsdbg(bambu_bridge, "[AMS_LINK]err_code:0x16->0x00", addr=0x0700)
        assert bambu_bridge.last_err_code() == (0, 160.0)
        assert bambu_bridge.logger.messages == [
            ams_echo("[AMS_LINK]err_code: 0 -> 23"),
            ams_echo("[AMS_LINK]err_code:0x16->0x00")]


class TestBambuBridgeTryClaimBus:
    def test_first_claim_wins_and_records_owner(self, bambu_bridge):
        assert bambu_bridge.try_claim_bus("BambuAMS_1", 10.0) is True
        assert bambu_bridge._bus_owner == "BambuAMS_1"
        assert bambu_bridge._bus_claim_t == 10.0
        assert bambu_bridge.logger.messages == []

    def test_a_second_owner_is_refused_while_busy(self, bambu_bridge):
        bambu_bridge.try_claim_bus("BambuAMS_1", 10.0)
        assert bambu_bridge.try_claim_bus("BambuAMS_2", 20.0) is False
        assert bambu_bridge._bus_owner == "BambuAMS_1"
        assert bambu_bridge._bus_claim_t == 10.0
        assert bambu_bridge.logger.messages == []

    def test_reclaim_by_the_same_owner_refreshes_the_stamp(self, bambu_bridge):
        bambu_bridge.try_claim_bus("BambuAMS_1", 10.0)
        assert bambu_bridge.try_claim_bus("BambuAMS_1", 30.0) is True
        assert bambu_bridge._bus_owner == "BambuAMS_1"
        assert bambu_bridge._bus_claim_t == 30.0
        assert bambu_bridge.logger.messages == []

    def test_a_cycle_end_after_the_claim_releases_it(self, bambu_bridge):
        # The unit's own end marker frees the bus, not a timer.
        bambu_bridge.try_claim_bus("BambuAMS_1", 10.0)
        bambu_bridge._rfid_end_t = 40.0
        assert bambu_bridge.try_claim_bus("BambuAMS_2", 41.0) is True
        assert bambu_bridge._bus_owner == "BambuAMS_2"
        assert bambu_bridge._bus_claim_t == 41.0
        assert bambu_bridge.logger.messages == []

    def test_a_cycle_end_before_the_claim_does_not_release_it(self, bambu_bridge):
        # A stale end marker from an earlier scan must not free a live claim.
        bambu_bridge._rfid_end_t = 5.0
        bambu_bridge.try_claim_bus("BambuAMS_1", 10.0)
        assert bambu_bridge.try_claim_bus("BambuAMS_2", 11.0) is False
        assert bambu_bridge._bus_owner == "BambuAMS_1"
        assert bambu_bridge.logger.messages == []

    def test_the_backstop_expires_an_unannounced_claim(self, bambu_bridge):
        bambu_bridge.try_claim_bus("BambuAMS_1", 10.0)
        # 120 s is the backstop: one short of it still holds, at it expires.
        assert bambu_bridge.try_claim_bus("BambuAMS_2", 129.0) is False
        assert bambu_bridge.try_claim_bus("BambuAMS_2", 130.0) is True
        assert bambu_bridge._bus_owner == "BambuAMS_2"
        assert bambu_bridge._bus_claim_t == 130.0
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeReleaseBus:
    def test_holder_releases(self, bambu_bridge):
        bambu_bridge.try_claim_bus("BambuAMS_1", 10.0)
        bambu_bridge.release_bus("BambuAMS_1")
        assert bambu_bridge._bus_owner is None
        assert bambu_bridge.logger.messages == []

    def test_non_holder_cannot_release(self, bambu_bridge):
        bambu_bridge.try_claim_bus("BambuAMS_1", 10.0)
        bambu_bridge.release_bus("BambuAMS_2")
        assert bambu_bridge._bus_owner == "BambuAMS_1"
        assert bambu_bridge.logger.messages == []

    def test_release_with_no_claim_is_safe(self, bambu_bridge):
        bambu_bridge.release_bus("BambuAMS_1")
        assert bambu_bridge._bus_owner is None
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeBusOwner:
    def test_none_before_any_claim(self, bambu_bridge):
        assert bambu_bridge.bus_owner() is None
        assert bambu_bridge.logger.messages == []

    def test_reports_the_holder(self, bambu_bridge):
        bambu_bridge.try_claim_bus("BambuAMS_1", 10.0)
        assert bambu_bridge.bus_owner() == "BambuAMS_1"
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeLastFault:
    TIMEOUT_SAID = ("info", "AFC bambu bridge: AMS: TIMEOUT -- the unit gave up "
                            "on the move")

    def test_ams1_state_6_is_a_fault(self, bambu_bridge):
        # The AMS 1 gives up in state, not words.
        line = "[AMS_COMMON]state:6,tray_now:255,tray_exit:6"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_fault() == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_ams1_en0_mode7_is_a_fault(self, bambu_bridge):
        line = "[AMS_LINK]en:0,mode:7,idx:255,ref:0"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_fault() == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_the_states_of_a_healthy_load_are_not(self, bambu_bridge):
        # Counted across a load that reached the toolhead: state:4 and
        # state:0 only.
        lines = ["[AMS_COMMON]state:4,tray_now:255,tray_exit:6",
                 "[AMS_COMMON]state:0,tray_now:255,tray_exit:6",
                 "[AMS_DEV] STEP:odom search, odo 0.516"]
        for line in lines:
            feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_fault() == (0, "", 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(x) for x in lines]

    @pytest.mark.parametrize("line, said", [
        ("[AMS_SWITCH]feed finish -1, stall, len_det:1.0 m", []),
        ("[AMS_LED]TIMEOUT error 2", [TIMEOUT_SAID]),
    ])
    def test_the_other_two_dialects_still_fire(self, bambu_bridge, line, said):
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_fault() == (1, line, 0.0)
        assert bambu_bridge.logger.messages == said + [ams_echo(line)]

    def test_a_chain_mates_stall_does_not_move_this_unit(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish -1, stall"
        feed_amsdbg(bambu_bridge, line, addr=0x0700, unit=1)
        assert bambu_bridge.last_fault(unit=0) == (0, "", 0.0)
        assert bambu_bridge.last_fault(unit=1) == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_this_units_stall_moves_it_with_its_own_words(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish -1, stall"
        feed_amsdbg(bambu_bridge, line, addr=0x0700, unit=0)
        assert bambu_bridge.last_fault(unit=0) == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_an_unattributed_stall_counts_for_every_unit(self, bambu_bridge):
        # Unit -1 is the firmware unable to tell; dropping it would let a
        # load ride out a real fault.
        line = "[AMS_LED]TIMEOUT error 2"
        feed_amsdbg(bambu_bridge, line, addr=0x0700, unit=-1)
        assert bambu_bridge.last_fault(unit=0) == (1, line, 0.0)
        assert bambu_bridge.last_fault(unit=1) == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [self.TIMEOUT_SAID, ams_echo(line)]

    def test_a_later_own_stall_outranks_an_older_unattributed_one(self, bambu_bridge):
        timeout = "[AMS_LED]TIMEOUT error 2"
        stall = "[AMS_SWITCH]feed finish -1, stall"
        feed_amsdbg(bambu_bridge, timeout, addr=0x0700, unit=-1)
        feed_amsdbg(bambu_bridge, stall, addr=0x0700, unit=0)
        assert bambu_bridge.last_fault(unit=0) == (2, stall, 0.0)
        assert bambu_bridge.last_fault(unit=1) == (1, timeout, 0.0)
        assert bambu_bridge.logger.messages == [
            self.TIMEOUT_SAID, ams_echo(timeout), ams_echo(stall)]

    def test_stall_is_captured(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m"
        feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.last_fault() == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_bdc_stall_is_captured(self, bambu_bridge):
        line = "[AMS_SWITCH]pull err, bdc stall, mode:1, tray_sw"
        feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.last_fault() == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_rocker_stall_with_tray_cnt_is_not_a_fault(self, monkeypatch, bambu_bridge):
        # tray_cnt is the unit's own retry counter: a retry it already
        # handled. A rocker stall without it still faults.
        benign = ["[AMS_SWITCH]switch_feed rocker stall, tray_cnt:0,0,",
                  "[AMS_SWITCH]odometer_cali rocker stall, tray_cnt:0,1,0,0",
                  "[AMS_SWITCH]feed_with_rfid rocker stall, tray_cnt:0,2,3,0"]
        for line in benign:
            feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.last_fault() == (0, "", 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(x) for x in benign]
        other = make_bambu_bridge(monkeypatch)
        line = "[AMS_SWITCH]switch_feed rocker stall, mode:1"
        feed_amsdbg(other, line)
        assert other.last_fault() == (1, line, 0.0)
        assert other.logger.messages == [ams_echo(line)]

    @pytest.mark.parametrize("line", [
        "[AMS_SWITCH]feed finish -1, tray_cnt:0,1",
        "[AMS_SWITCH]pull err, bdc stall, tray_cnt:0,1",
    ], ids=["finish -1", "bdc stall"])
    def test_tray_cnt_excuses_only_a_rocker_stall(self, bambu_bridge, line):
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_fault() == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_finish_minus_one_without_a_stall_is_a_fault(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish -1, len_det:1.0 m"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_fault() == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_motor_current_is_parsed(self, bambu_bridge):
        line = "[AMS_SWITCH]feed to dw ok, len_det:0.050 m, bldc_i:1.600A"
        feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.last_fault() == (0, "", 1.6)
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS: filament reached the hub after "
                     "0.05 m"),
            ams_echo(line)]

    def test_ams2_timeout_error_is_captured(self, bambu_bridge):
        # A boxed AMS 2 reports a jam as "TIMEOUT error N", never "stall".
        line = ("[AMS_LED]TIMEOUT error 2 [AMS_LED]TIMEOUT error 3 "
                "[AMS_LED]TRAY 3 in five [AMS_LINK]err_code: 0 -> 23")
        feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.last_fault() == (1, line, 0.0)
        assert bambu_bridge.logger.messages == [self.TIMEOUT_SAID, ams_echo(line)]

    def test_assist_err_is_not_a_fault(self, bambu_bridge):
        # assist_err cycles around every successful feed.
        line = "[AMS_LINK]assist_err: 0 -> 65536"
        feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.last_fault() == (0, "", 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_err_code_is_not_a_fault(self, bambu_bridge):
        # err_code 0x16 shows up hundreds of times in normal operation.
        line = "[AMS_LINK]err_code:0x00->0x16"
        feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.last_fault() == (0, "", 0.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]


class TestBambuBridgeSetNarrationLog:
    LOGGERS = ("AFC_BambuAMS_file", "AFC_BambuAMS_file_ttyACM1")

    @pytest.fixture(autouse=True)
    def _fresh_narration_loggers(self) -> Any:
        """Strip the process-wide narration loggers before and after."""
        def clear() -> None:
            """Close and remove every handler on the narration loggers."""
            for name in self.LOGGERS:
                lg = logging.getLogger(name)
                for handler in list(lg.handlers):
                    handler.close()
                    lg.removeHandler(handler)

        clear()
        yield
        clear()

    @staticmethod
    def _file_lines(path: Any, name: str = "AFC_BambuAMS_file") -> List[str]:
        """
        Flush the narration logger and read its file back.

        :param path: the log file
        :param name: the logger writing it
        :return list: each line without its HH:MM:SS stamp
        """
        for handler in logging.getLogger(name).handlers:
            handler.flush()
        out = []
        for line in path.read_text().splitlines():
            stamp, _sep, rest = line.partition(" ")
            assert len(stamp) == 8 and stamp[2] == stamp[5] == ":"
            out.append(rest)
        return out

    @staticmethod
    def _rotating(name: str = "AFC_BambuAMS_file") -> List[Any]:
        """:return list: the logger's RotatingFileHandlers"""
        return [h for h in logging.getLogger(name).handlers
                if isinstance(h, logging.handlers.RotatingFileHandler)]

    def test_it_writes_narration_to_its_own_file(self, bambu_bridge, tmp_path):
        assert bambu_bridge.set_narration_log(str(tmp_path)) is True
        assert bambu_bridge._nar_lg is logging.getLogger("AFC_BambuAMS_file")
        line = "[AMS_SWITCH]feed finish 0, dw_len:3.508 m"
        feed_amsdbg(bambu_bridge, line, addr=0x1800)
        assert self._file_lines(tmp_path / "AFC_BambuAMS.log") == [
            "0x1800 u? [AMS_SWITCH]feed finish 0, dw_len:3.508 m"]
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_it_does_not_propagate_into_afc_log(self, bambu_bridge, tmp_path):
        lg = logging.getLogger("AFC_BambuAMS_file")
        lg.propagate = True
        lg.setLevel(logging.NOTSET)
        bambu_bridge.set_narration_log(str(tmp_path))
        assert lg.propagate is False
        assert lg.level == logging.DEBUG
        assert bambu_bridge.logger.messages == []

    def test_rotation_defaults_to_10mb_with_one_backup(self, bambu_bridge, tmp_path):
        bambu_bridge.set_narration_log(str(tmp_path))
        handlers = self._rotating()
        assert len(handlers) == 1
        assert handlers[0].maxBytes == 10 * 1024 * 1024
        assert handlers[0].backupCount == 1
        assert handlers[0].baseFilename == str(tmp_path / "AFC_BambuAMS.log")
        assert bambu_bridge.logger.messages == []

    def test_it_rotates_and_keeps_exactly_one_backup(self, bambu_bridge, tmp_path):
        # At maxBytes the live file rolls to .log.1 and a fresh one starts:
        # the previous chunk survives, disk use stays bounded.
        bambu_bridge.set_narration_log(str(tmp_path), max_bytes=200)
        lines = [f"[AMS_SWITCH]line {i} padding padding padding" for i in range(60)]
        for line in lines:
            feed_amsdbg(bambu_bridge, line)
        for handler in self._rotating():
            handler.flush()
        assert (tmp_path / "AFC_BambuAMS.log").stat().st_size <= 400
        assert sorted(tmp_path.glob("AFC_BambuAMS.log*")) == [
            tmp_path / "AFC_BambuAMS.log", tmp_path / "AFC_BambuAMS.log.1"]
        assert (tmp_path / "AFC_BambuAMS.log.1").stat().st_size <= 400
        # The last line written is in the live file.
        assert self._file_lines(tmp_path / "AFC_BambuAMS.log")[-1] == (
            "0x---- u? [AMS_SWITCH]line 59 padding padding padding")
        # Sixty lines in one second pass the console's burst budget of 12.
        burst = ("info", "AFC bambu: the AMS is narrating faster than the "
                         "console can take (>12/s); the rest of this burst "
                         "is in AFC.log")
        echoes = [ams_echo(x) for x in lines]
        assert bambu_bridge.logger.messages == echoes[:12] + [burst] + echoes[12:]

    def test_an_unwritable_directory_is_reported_not_raised(self, bambu_bridge):
        assert bambu_bridge.set_narration_log("/nonexistent-dir-xyz") is False
        assert bambu_bridge._nar_lg is None
        assert self._rotating() == []
        assert bambu_bridge.logger.messages == [
            ("warning", "AFC bambu: could not open AFC_BambuAMS.log: [Errno 2] "
                        "No such file or directory: "
                        "'/nonexistent-dir-xyz/AFC_BambuAMS.log'")]

    def test_setup_is_idempotent(self, bambu_bridge, tmp_path):
        first, second = tmp_path / "a", tmp_path / "b"
        first.mkdir()
        second.mkdir()
        bambu_bridge.set_narration_log(str(first))
        assert bambu_bridge.set_narration_log(str(second)) is True
        handlers = self._rotating()
        assert [h.baseFilename for h in handlers] == [str(first / "AFC_BambuAMS.log")]
        assert list(second.iterdir()) == []
        assert bambu_bridge.logger.messages == []

    def test_a_preexisting_unrelated_handler_does_not_defeat_setup(
            self, bambu_bridge, tmp_path):
        # getLogger() is process-global: a handler of another kind must not
        # read as "already set up" and leave the file unwritten.
        lg = logging.getLogger("AFC_BambuAMS_file")
        null = logging.NullHandler()
        lg.addHandler(null)
        assert bambu_bridge.set_narration_log(str(tmp_path)) is True
        assert lg.handlers[0] is null
        assert len(self._rotating()) == 1
        line = "[AMS_SWITCH]feed finish 0, dw_len:3.5 m"
        feed_amsdbg(bambu_bridge, line)
        assert self._file_lines(tmp_path / "AFC_BambuAMS.log") == [
            "0x---- u? [AMS_SWITCH]feed finish 0, dw_len:3.5 m"]
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_second_bridge_on_one_master_shares_the_handler(
            self, monkeypatch, bambu_bridge, tmp_path):
        other = make_bambu_bridge(monkeypatch)
        assert bambu_bridge.set_narration_log(str(tmp_path)) is True
        assert other.set_narration_log(str(tmp_path)) is True
        assert len(self._rotating()) == 1
        assert other._nar_lg is bambu_bridge._nar_lg
        assert bambu_bridge.logger.messages == []
        assert other.logger.messages == []

    def test_no_tag_keeps_the_original_filename(self, bambu_bridge, tmp_path):
        # A single-Pico printer is unchanged.
        assert bambu_bridge.set_narration_log(str(tmp_path)) is True
        assert [h.baseFilename for h in self._rotating()] == [
            str(tmp_path / "AFC_BambuAMS.log")]
        assert self._rotating("AFC_BambuAMS_file_ttyACM1") == []
        assert bambu_bridge.logger.messages == []

    def test_a_tagged_master_writes_its_own_file(self, bambu_bridge, tmp_path):
        assert bambu_bridge.set_narration_log(str(tmp_path), "ttyACM1") is True
        assert bambu_bridge._nar_lg is logging.getLogger("AFC_BambuAMS_file_ttyACM1")
        feed_amsdbg(bambu_bridge, "[AMS_DEV] STEP:odom reset tray 0", addr=0x0700)
        assert self._file_lines(tmp_path / "AFC_BambuAMS_ttyACM1.log",
                                "AFC_BambuAMS_file_ttyACM1") == [
            "0x0700 u? [AMS_DEV] STEP:odom reset tray 0"]
        assert not (tmp_path / "AFC_BambuAMS.log").exists()
        assert bambu_bridge.logger.messages == [
            ams_echo("[AMS_DEV] STEP:odom reset tray 0")]

    def test_two_masters_do_not_share_a_file(self, monkeypatch, tmp_path):
        one = make_bambu_bridge(monkeypatch)
        two = make_bambu_bridge(monkeypatch)
        one.set_narration_log(str(tmp_path))
        two.set_narration_log(str(tmp_path), "ttyACM1")
        feed_amsdbg(one, "bus one speaking", addr=0x0700)
        feed_amsdbg(two, "bus two speaking", addr=0x0700)
        assert self._file_lines(tmp_path / "AFC_BambuAMS.log") == [
            "0x0700 u? bus one speaking"]
        assert self._file_lines(tmp_path / "AFC_BambuAMS_ttyACM1.log",
                                "AFC_BambuAMS_file_ttyACM1") == [
            "0x0700 u? bus two speaking"]
        assert one.logger.messages == [ams_echo("bus one speaking")]
        assert two.logger.messages == [ams_echo("bus two speaking")]


class TestBambuBridgeNarrateToFile:
    def test_writes_the_line_with_its_address_and_unit(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge._narrate_to_file("[AMS_PMSM]mode:0->2", 0x0700, 1)
        assert bridge._nar_lg.messages == [("debug", "0x0700 u1 [AMS_PMSM]mode:0->2")]
        assert bridge.logger.messages == []

    def test_two_units_on_one_address_are_told_apart(self, monkeypatch):
        # Both boxed AMSs answer at 0x0700; only the unit index separates
        # their lines in the file.
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge._narrate_to_file("[AMS_DEV] STEP:rfid pull 1", 0x0700, 0)
        bridge._narrate_to_file("[AMS_DEV] STEP:rfid pull 1", 0x0700, 1)
        assert bridge._nar_lg.messages == [
            ("debug", "0x0700 u0 [AMS_DEV] STEP:rfid pull 1"),
            ("debug", "0x0700 u1 [AMS_DEV] STEP:rfid pull 1")]
        assert bridge.logger.messages == []

    def test_an_unattributed_line_says_so_rather_than_guessing(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge._narrate_to_file("[AMS_PMSM]mode:0->2", 0x0700, None)
        assert bridge._nar_lg.messages == [("debug", "0x0700 u? [AMS_PMSM]mode:0->2")]
        assert bridge.logger.messages == []

    def test_an_unknown_address_is_dashes(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge._narrate_to_file("hello", None)
        assert bridge._nar_lg.messages == [("debug", "0x---- u? hello")]
        assert bridge.logger.messages == []

    def test_no_log_configured_is_a_noop(self, bambu_bridge):
        assert bambu_bridge._nar_lg is None
        bambu_bridge._narrate_to_file("hello", 0x0700)
        assert bambu_bridge._nar_lg is None
        assert bambu_bridge.logger.messages == []

    def test_empty_text_writes_nothing(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge._narrate_to_file("", 0x0700)
        assert bridge._nar_lg.messages == []
        assert bridge.logger.messages == []


class TestBambuBridgeLastBuffPos:
    E_IN = "[AMS_SWITCH]e_in tray:0,buff_pos:-0.34,i:0.566A,len:1.670m"

    def test_e_in_still_yields_its_buffer_reading(self, bambu_bridge):
        # e_in no longer completes a move, but its buff_pos is still read.
        assert bambu_bridge.last_buff_pos() is None
        feed_amsdbg(bambu_bridge, self.E_IN, addr=0x0700)
        assert bambu_bridge.last_buff_pos() == -0.34
        assert bambu_bridge.last_finish() == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(self.E_IN)]

    def test_the_recovered_position_becomes_the_current_one(self, bambu_bridge):
        line = "[AMS_SWITCH]BUFF,pos:0.10->0.74, det:28mm, i:0.521A"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_buff_pos() == 0.74
        assert bambu_bridge.logger.messages == [ams_echo(line)]


class TestBambuBridgeLastBuffRefill:
    def test_nothing_reported_yet_is_none(self, bambu_bridge):
        assert bambu_bridge.last_buff_refill() is None
        assert bambu_bridge.last_buff_pos() is None
        assert bambu_bridge.logger.messages == []

    def test_a_refill_records_sag_recovery_and_distance(self, bambu_bridge):
        line = "[AMS_SWITCH]BUFF,pos:0.09->0.74, det:6mm,  i:0.583A"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_buff_refill() == (0.09, 0.74, 6.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_the_unspaced_form_is_read(self, bambu_bridge):
        line = "[AMS_SWITCH]BUFF,pos:0.09->0.74,det:12mm"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_buff_refill() == (0.09, 0.74, 12.0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_refill_without_det_still_records_the_positions(self, bambu_bridge):
        line = "[AMS_SWITCH]BUFF,pos:0.10->0.74"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_buff_refill() == (0.10, 0.74, None)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_the_distance_varies_while_the_setpoint_does_not(self, bambu_bridge):
        # A unit refilling to a fixed setpoint on demand, which is what
        # makes it usable for ramming.
        lines = ["[AMS_SWITCH]BUFF,pos:0.09->0.74, det:6mm,  i:0.583A",
                 "[AMS_SWITCH]BUFF,pos:0.10->0.73, det:24mm, i:0.740A",
                 "[AMS_SWITCH]BUFF,pos:0.10->0.74, det:28mm, i:0.521A"]
        seen = []
        for line in lines:
            feed_amsdbg(bambu_bridge, line, addr=0x0700)
            seen.append(bambu_bridge.last_buff_refill())
        assert seen == [(0.09, 0.74, 6.0), (0.10, 0.73, 24.0), (0.10, 0.74, 28.0)]
        assert bambu_bridge.logger.messages == [ams_echo(x) for x in lines]

    def test_none_before_any_refill(self, bambu_bridge):
        assert bambu_bridge.last_buff_refill() is None
        assert bambu_bridge.logger.messages == []

    def test_reports_the_tuple(self, bambu_bridge):
        bambu_bridge._buff_refill = (0.31, 0.62, 18.0)
        assert bambu_bridge.last_buff_refill() == (0.31, 0.62, 18.0)
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeNoteDryRefusal:
    LINE = "[AMS_CHMB]err, filament hub load!"

    def test_a_refusal_is_recorded_for_its_unit(self, bambu_bridge):
        bambu_bridge._note_dry_refusal(self.LINE, 0x0700, unit=1)
        assert bambu_bridge._dry_err_u == {1: "filament hub load!"}
        assert bambu_bridge.logger.messages == []

    def test_an_unattributable_refusal_is_dropped(self, bambu_bridge):
        bambu_bridge._note_dry_refusal(self.LINE, 0x0700, unit=None)
        assert bambu_bridge._dry_err_u == {}
        assert bambu_bridge.logger.messages == []

    @pytest.mark.parametrize("clear", [
        "[AMS_CHMB]set state CTC_STATE_HEATING",
        "[AMS_CHMB]set state CTC_STATE_SELF_CHECK, from off, ref:55",
        "[AMS_CHMB]dry_mode:1, check ok!",
        "dry_mode:1, ams-ht shell ok!",
    ])
    def test_heating_clears_that_units_refusal(self, bambu_bridge, clear):
        bambu_bridge._note_dry_refusal(self.LINE, 0x0700, unit=1)
        bambu_bridge._note_dry_refusal(self.LINE, 0x0700, unit=2)
        bambu_bridge._note_dry_refusal(clear, 0x0700, unit=1)
        assert bambu_bridge._dry_err_u == {2: "filament hub load!"}
        assert bambu_bridge.logger.messages == []

    def test_shell_ok_clears_the_hts_open_lid_note(self, bambu_bridge):
        bambu_bridge._note_dry_refusal("[AMS_CHMB]err, ams-ht shell open!",
                                       0x1800, unit=4)
        assert bambu_bridge._dry_err_u == {4: "ams-ht shell open!"}
        bambu_bridge._note_dry_refusal("dry_mode:1, ams-ht shell ok!", 0x1800,
                                       unit=4)
        assert bambu_bridge._dry_err_u == {}
        assert bambu_bridge.logger.messages == []

    def test_an_unattributable_clear_clears_nothing(self, bambu_bridge):
        bambu_bridge._note_dry_refusal(self.LINE, 0x0700, unit=1)
        bambu_bridge._note_dry_refusal("[AMS_CHMB]set state CTC_STATE_HEATING",
                                       0x0700, unit=None)
        assert bambu_bridge._dry_err_u == {1: "filament hub load!"}
        assert bambu_bridge.logger.messages == []

    def test_no_address_records_nothing(self, bambu_bridge):
        bambu_bridge._note_dry_refusal(self.LINE, None, unit=1)
        assert bambu_bridge._dry_err_u == {}
        assert bambu_bridge.logger.messages == []

    def test_no_text_records_nothing(self, bambu_bridge):
        bambu_bridge._note_dry_refusal("", 0x0700, unit=1)
        assert bambu_bridge._dry_err_u == {}
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeNoteDryCfg:
    LINE = "[AMS_CHMB]rotate:1, 0, pw_lim:100, cool_down:0, 0, dur:480, tmpr:55"

    def test_the_echo_is_recorded_by_unit(self, bambu_bridge):
        bambu_bridge._note_dry_cfg(self.LINE, 0x0700, unit=0)
        assert bambu_bridge._dry_cfg_u == {0: {"rotate": 1, "dur": 480, "tmpr": 55}}
        assert bambu_bridge.logger.messages == []

    @pytest.mark.parametrize("pair, rotate", [("1, 0", 1), ("0, 1", 1), ("0, 0", 0)])
    def test_either_rotate_flag_means_rotating(self, bambu_bridge, pair, rotate):
        line = f"[AMS_CHMB]rotate:{pair}, pw_lim:80, cool_down:0, 45, dur:240, tmpr:45"
        bambu_bridge._note_dry_cfg(line, 0x1800, unit=4)
        assert bambu_bridge._dry_cfg_u == {4: {"rotate": rotate, "dur": 240, "tmpr": 45}}
        assert bambu_bridge.logger.messages == []

    def test_a_non_cfg_line_records_nothing(self, bambu_bridge):
        bambu_bridge._note_dry_cfg("[AMS_PMSM]mode:0->2", 0x0700, unit=0)
        assert bambu_bridge._dry_cfg_u == {}
        assert bambu_bridge.logger.messages == []

    def test_unattributable_echo_is_dropped(self, bambu_bridge):
        bambu_bridge._note_dry_cfg(self.LINE, 0x0700, unit=None)
        assert bambu_bridge._dry_cfg_u == {}
        assert bambu_bridge.logger.messages == []

    def test_no_address_records_nothing(self, bambu_bridge):
        bambu_bridge._note_dry_cfg(self.LINE, None, unit=0)
        assert bambu_bridge._dry_cfg_u == {}
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeLastDryCfg:
    LINE = "[AMS_CHMB]rotate:1, 0, pw_lim:100, cool_down:0, 0, dur:480, tmpr:55"

    def test_none_unit_is_none(self, bambu_bridge):
        bambu_bridge._note_dry_cfg(self.LINE, 0x0700, unit=0)
        assert bambu_bridge.last_dry_cfg(None) is None
        assert bambu_bridge.logger.messages == []

    def test_never_echoed_is_none(self, bambu_bridge):
        assert bambu_bridge.last_dry_cfg(0) is None
        assert bambu_bridge.logger.messages == []

    def test_returns_a_copy(self, bambu_bridge):
        bambu_bridge._note_dry_cfg(self.LINE, 0x0700, unit=0)
        got = bambu_bridge.last_dry_cfg(0)
        assert got == {"rotate": 1, "dur": 480, "tmpr": 55}
        got["dur"] = 999
        assert bambu_bridge._dry_cfg_u[0]["dur"] == 480
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeNoteCapMeasure:
    #: (label, narration, tray, circumference, radius, shown percent,
    #:  raw percent, restored)
    CAP_LINES = [
        ("HT live", "[AMS_RFID] STEP4,odom C:0.531,R:0.084,P:107%,od:1.132",
         None, 0.531, 0.084, 100, 107, False),
        ("AMS 1 live", "[AMS_DEV] STEP:odom C:0.480, R:0.076, P:78%, od:0.988",
         None, 0.480, 0.076, 78, 78, False),
        ("AMS 2 restore", "[AMS_RFID]STEP:odom load from flash 2,R:0.072,P:65",
         2, None, 0.072, 65, 65, True),
        ("HT restore", "[AMS_RFID] STEP:odom load from flash 0,R:0.088,P:119",
         0, None, 0.088, 100, 119, True),
    ]
    BATCH = ("[AMS_RFID]STEP:odom detect #2, odo:0.609 "
             "[AMS_RFID]STEP:odom C:0.588,R:0.094,P:142%,N:2,od:0.609 "
             "[AMS_RFID]STEP:odom save tray:1, R:0.093643 "
             "[AMS_PMSM]mode:2->0")

    @staticmethod
    def _record(pct: int, pct_raw: int, radius: float, restored: bool, *,
                circ: Optional[float] = None, tray: Optional[int] = None,
                t: float = 99.0, save_tray: Optional[int] = None,
                save_radius: Optional[float] = None) -> Dict[str, Any]:
        """:return dict: the measurement record expected for these fields"""
        return {"pct": pct, "pct_raw": pct_raw, "radius_m": radius,
                "circumference_m": circ, "tray": tray, "restored": restored,
                "save_tray": save_tray, "save_radius_m": save_radius, "t": t}

    @pytest.mark.parametrize("label, text, tray, circ, radius, pct, raw, restored",
                             CAP_LINES, ids=[c[0] for c in CAP_LINES])
    def test_capacity_narration_parses_every_dialect(
            self, bambu_bridge, label, text, tray, circ, radius, pct, raw, restored):
        bambu_bridge._note_cap_measure(text, 0x0700, 99.0)
        rec = bambu_bridge._cap_measure[0x0700]
        assert rec == self._record(pct, raw, radius, restored, circ=circ, tray=tray)
        assert bambu_bridge.logger.messages == []

    def test_capacity_circumference_agrees_with_radius(self, bambu_bridge):
        # C = 2*pi*R on every live reading: the fields land where they
        # belong rather than three numbers that happen to line up.
        for _label, text, _tray, circ, _radius, _pct, _raw, _restored in self.CAP_LINES:
            if circ is None:
                continue
            bambu_bridge._note_cap_measure(text, 0x0700, 99.0)
            rec = bambu_bridge._cap_measure[0x0700]
            assert abs(rec["circumference_m"] - 6.2832 * rec["radius_m"]) < 0.005
        assert bambu_bridge.logger.messages == []

    def test_calibration_done_matches_both_spellings(self, bambu_bridge):
        # The HT spells it with one 's'; on a boxed address either spelling
        # is the cycle's verdict.
        bambu_bridge._note_cap_measure("[AMS_RFID] STEP4,odom calib sucess",
                                       0x0700, 40.0, unit=0)
        bambu_bridge._note_cap_measure(
            "[AMS_DEV] STEP:odom calib success exit 0,dis:0.989", 0x0700, 41.0,
            unit=1)
        assert bambu_bridge._ht_cali_u == {0: {"rst": 0, "t": 40.0},
                                           1: {"rst": 0, "t": 41.0}}
        assert bambu_bridge.logger.messages == []

    @pytest.mark.parametrize("noise", [
        "[AMS_DEV] STEP:odom search, odo 1.856",
        "[AMS_DEV] STEP:odom reset tray 0",
        "[AMS_RFID] STEP,odom load tray 3 info invailed",
        "[AMS_RFID] STEP,odom save R nan, exit",
    ])
    def test_capacity_pattern_ignores_unrelated_odom_chatter(self, bambu_bridge, noise):
        bambu_bridge._note_cap_measure(noise, 0x0700, 99.0)
        assert bambu_bridge._cap_measure == {}
        assert bambu_bridge.logger.messages == []

    @pytest.mark.parametrize("line", [
        "[AMS_DEV] STEP:odom r:0, dt0.442, R:0.073, P:70%, od:0.741",
        "[AMS_DEV] STEP:odom r:1, dt0.887, R:0.071, P:65%",
        "[AMS_DEV] STEP:odom r:1, dt0.895, R:0.077, P:54%",
    ])
    def test_the_load_time_search_lines_are_NOT_a_measurement(self, bambu_bridge, line):
        # The mid-load radius search does not converge; only the calibration
        # cycle's reading (with a circumference) is a measurement.
        bambu_bridge._note_cap_measure(line, 0x0700, 99.0)
        assert bambu_bridge._cap_measure == {}
        assert bambu_bridge.logger.messages == []

    def test_the_calibrated_line_from_the_same_session_does_parse(self, bambu_bridge):
        bambu_bridge._note_cap_measure(
            "[AMS_DEV] STEP,second detected [AMS_DEV] STEP:odom "
            "C:0.469,R:0.075,P:73%, od:0.724", 0x0700, 99.0)
        assert bambu_bridge._cap_measure[0x0700] == self._record(73, 73, 0.075, False, circ=0.469)
        assert bambu_bridge.logger.messages == []

    @pytest.mark.parametrize("line, pct, raw", [
        ("[AMS_RFID] STEP4,odom C:0.531,R:0.084,P:107%,od:1.132", 100, 107),
        ("[AMS_DEV] STEP:odom C:0.480, R:0.076, P:78%, od:0.988", 78, 78),
        ("[AMS_RFID]STEP:odom load from flash 2,R:0.072,P:65", 65, 65),
    ], ids=["HT", "AMS 1", "AMS 2"])
    def test_capacity_line_parses_on_all_three_units(self, bambu_bridge, line, pct, raw):
        # Clamped for a 0-100 display; the raw figure is kept beside it.
        bambu_bridge._note_cap_measure(line, 0x0700, 99.0)
        rec = bambu_bridge._cap_measure[0x0700]
        assert (rec["pct"], rec["pct_raw"]) == (pct, raw)
        assert bambu_bridge.logger.messages == []

    def test_the_colonless_ht_form_is_a_restore(self, bambu_bridge):
        bambu_bridge._note_cap_measure(
            "[AMS_RFID] STEP,odom r0, dt0.444, R0.075, 82%, od0.751", 0x1800, 99.0)
        assert bambu_bridge._cap_measure == {
            0x1800: self._record(82, 82, 0.075, True, tray=0)}
        assert bambu_bridge.logger.messages == []

    def test_a_live_measurement_is_recorded_with_its_time(self, bambu_bridge):
        bambu_bridge._note_cap_measure("odom C:1.234, R:0.084, P:78%", 0x0700, 99.0)
        assert bambu_bridge._cap_measure == {
            0x0700: self._record(78, 78, 0.084, False, circ=1.234)}
        assert bambu_bridge.logger.messages == []

    def test_an_ht_verdict_is_recorded_by_unit(self, bambu_bridge):
        bambu_bridge._note_cap_measure("Calibration rst:0", 0x1800, 50.0, unit=4)
        assert bambu_bridge._ht_cali_u == {4: {"rst": 0, "t": 50.0}}
        assert bambu_bridge._cap_measure == {}
        assert bambu_bridge.logger.messages == []

    def test_an_unattributed_verdict_is_not_recorded(self, bambu_bridge):
        bambu_bridge._note_cap_measure("Calibration rst:4", 0x1800, 50.0)
        bambu_bridge._note_cap_measure("STEP:odom calib success exit 0", 0x0700, 51.0)
        assert bambu_bridge._ht_cali_u == {}
        assert bambu_bridge.logger.messages == []

    def test_a_plain_line_records_nothing(self, bambu_bridge):
        bambu_bridge._note_cap_measure("[AMS_PMSM]mode:0->2", 0x0700, 99.0)
        assert bambu_bridge._cap_measure == {}
        assert bambu_bridge._ht_cali_u == {}
        assert bambu_bridge.logger.messages == []

    def test_no_address_records_nothing(self, bambu_bridge):
        bambu_bridge._note_cap_measure("odom C:1.234, R:0.084, P:78%", None, 99.0)
        bambu_bridge._note_cap_measure("Calibration rst:0", None, 99.0, unit=4)
        assert bambu_bridge._cap_measure == {}
        assert bambu_bridge._ht_cali_u == {}
        assert bambu_bridge.logger.messages == []

    def test_the_tray_is_taken_from_the_same_batch(self, bambu_bridge):
        # The save line keeps its own precision: at this radius a millimetre
        # is about 2% of the reel.
        bambu_bridge._note_cap_measure(self.BATCH, 0x0700, 99.0)
        assert bambu_bridge._cap_measure[0x0700] == self._record(
            100, 142, 0.094, False, circ=0.588, save_tray=1, save_radius=0.093643)
        assert bambu_bridge.logger.messages == []

    def test_a_later_batch_still_labels_the_reading(self, bambu_bridge):
        bambu_bridge._note_cap_measure("odom C:0.588,R:0.094,P:142%", 0x0700, 99.0)
        bambu_bridge._note_cap_measure("[AMS_RFID]STEP:odom save tray:2, R:0.093643",
                                       0x0700, 99.2)
        assert bambu_bridge._cap_measure[0x0700] == self._record(
            100, 142, 0.094, False, circ=0.588, save_tray=2, save_radius=0.093643)
        assert bambu_bridge.logger.messages == []

    def test_a_save_from_another_cycle_cannot_relabel_it(self, bambu_bridge):
        # Matched on radius, not recency: a leftover save describes a
        # different spool.
        bambu_bridge._note_cap_measure("odom C:0.588,R:0.094,P:142%", 0x0700, 99.0)
        bambu_bridge._note_cap_measure("odom save tray:3, R:0.071200", 0x0700, 99.2)
        assert bambu_bridge._cap_measure[0x0700] == self._record(
            100, 142, 0.094, False, circ=0.588)
        assert bambu_bridge.logger.messages == []

    def test_a_save_alone_records_no_measurement(self, bambu_bridge):
        bambu_bridge._note_cap_measure("odom save tray:1, R:0.093643", 0x0700, 99.0)
        assert bambu_bridge._cap_measure == {}
        assert bambu_bridge.logger.messages == []

    def test_an_unlabelled_measurement_says_so(self, bambu_bridge):
        bambu_bridge._note_cap_measure("odom C:1.234, R:0.084, P:78%", 0x0700, 99.0)
        assert bambu_bridge._cap_measure[0x0700]["save_tray"] is None
        assert bambu_bridge._cap_measure[0x0700]["save_radius_m"] is None
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeLastHtCali:
    HT_MEAS = "[HT-MEAS] fire0 capu1 tun1 dst1 act1 arm0 htm0000 on0 off306 pres0"
    # lane8's reading, verbatim from AFC_BambuAMS.log (0x0700 u1).
    AMS1_ODOM = ("[AMS_DEV] STEP,second detected [AMS_DEV] STEP:odom C:0.449,"
                 "R:0.071,P:63%, od:1.004 [AMS_DEV] STEP:odom save tray:1, "
                 "R:0.071481 [AMS_DEV] STEP:odom calib success exit 0,dis:0.832")

    def test_none_unit_is_none(self, bambu_bridge):
        bambu_bridge._note_cap_measure("Calibration rst:0", 0x1800, 50.0, unit=4)
        assert bambu_bridge.last_ht_cali(None) is None
        assert bambu_bridge.logger.messages == []

    def test_never_reported_is_none(self, bambu_bridge):
        assert bambu_bridge.last_ht_cali(4) is None
        assert bambu_bridge.logger.messages == []

    def test_reports_the_verdict(self, bambu_bridge):
        bambu_bridge._note_cap_measure("Calibration rst:1", 0x1800, 50.0, unit=4)
        got = bambu_bridge.last_ht_cali(4)
        assert got == {"rst": 1, "t": 50.0}
        got["rst"] = 9
        assert bambu_bridge._ht_cali_u[4] == {"rst": 1, "t": 50.0}
        assert bambu_bridge.logger.messages == []

    def test_the_ams1_done_line_still_maps_to_rst_0(self, bambu_bridge):
        # "odom calib success exit 0" is the AMS 1's only cycle-end sentence.
        bambu_bridge._note_cap_measure("STEP:odom calib success exit 0,dis:0.989",
                                       0x0700, 44.0, unit=0)
        assert bambu_bridge.last_ht_cali(0) == {"rst": 0, "t": 44.0}
        assert bambu_bridge.logger.messages == []

    def test_an_ht_done_line_is_not_a_verdict(self, bambu_bridge):
        # One HT cycle end says both lines, ~6 s apart; on an HT the rst line
        # is the verdict and the done line only its preamble.
        bambu_bridge._note_cap_measure("[AMS_RFID] STEP4,odom calib sucess",
                                       0x1800, 44.0, unit=4)
        assert bambu_bridge.last_ht_cali(4) is None
        bambu_bridge._note_cap_measure("Calibration rst:0", 0x1800, 50.0, unit=4)
        assert bambu_bridge.last_ht_cali(4) == {"rst": 0, "t": 50.0}
        assert bambu_bridge.logger.messages == []

    def test_the_ams1_own_success_line_still_gives_rst_0(self, monkeypatch):
        # A firmware diagnostic stops at the file; the AMS 1's own line that
        # follows is still parsed.
        bridge = make_bambu_bridge(monkeypatch, name="u")
        feed_amsdbg(bridge, self.HT_MEAS, addr=0x1800, unit=1)
        bridge.reactor.advance(5.0)
        feed_amsdbg(bridge, self.AMS1_ODOM, addr=0x0700, unit=1)
        assert bridge.last_ht_cali(1) == {"rst": 0, "t": 105.0}
        assert bridge.last_cap_measure(0x0700) == {
            "pct": 63, "pct_raw": 63, "radius_m": 0.071, "circumference_m": 0.449,
            "tray": None, "restored": False, "save_tray": 1,
            "save_radius_m": 0.071481, "t": 105.0}
        diag = f"AFC bambu: bridge diag {self.HT_MEAS}"
        assert bridge.logger.messages == [
            ("debug", diag),
            ("info", "AFC bambu u: AMS finished measuring the spool"),
            ams_echo(self.AMS1_ODOM)]
        assert bridge.logger.file_only == [diag]

    def test_an_ams_line_mentioning_it_later_is_still_parsed(self, monkeypatch):
        # Anchored at the start: only the firmware's own lines begin with it.
        bridge = make_bambu_bridge(monkeypatch, name="u")
        line = "[AMS_DEV] STEP:odom calib success exit 0 [HT-MEAS]"
        feed_amsdbg(bridge, line, addr=0x0700, unit=1)
        assert bridge.last_ht_cali(1) == {"rst": 0, "t": 100.0}
        assert bridge.logger.messages == [
            ("info", "AFC bambu u: AMS finished measuring the spool"),
            ams_echo(line)]


class TestBambuBridgeCapCalibrating:
    """
    The flag is stamped and aged on the reactor clock, which on a printer reads
    seconds since boot while the wall clock reads an epoch time.
    """

    DETECTED = "[AMS_RFID]STEP:first detected"
    DETECTED_SAID = ("info", "AFC bambu bridge: AMS: spool detected")
    WALL = 1_700_000_000.0

    @pytest.fixture
    def wall_bridge(self, monkeypatch: pytest.MonkeyPatch) -> BambuBridge:
        """:return BambuBridge: a bridge whose wall clock is not its reactor's"""
        bridge = make_bambu_bridge(monkeypatch)
        monkeypatch.setattr(bridge_mod.time, "time", lambda: self.WALL)
        return bridge

    def _measuring(self, bridge: BambuBridge) -> None:
        """Have 0x0700 announce a measurement, as handle_line receives it."""
        feed_amsdbg(bridge, self.DETECTED, addr=0x0700)

    def test_no_address_is_false(self, wall_bridge):
        self._measuring(wall_bridge)
        # A live flag under address 0 still reads as no address.
        wall_bridge._meas_live[0] = True
        wall_bridge._meas_live_t[0] = wall_bridge.reactor.now
        assert wall_bridge.cap_calibrating(None) is False
        assert wall_bridge.cap_calibrating(0) is False
        assert wall_bridge._meas_live == {0x0700: True, 0: True}
        assert wall_bridge.logger.messages == [
            self.DETECTED_SAID, ams_echo(self.DETECTED)]

    def test_not_measuring_is_false(self, wall_bridge):
        self._measuring(wall_bridge)
        assert wall_bridge.cap_calibrating(0x1800) is False
        assert wall_bridge._meas_live == {0x0700: True}
        assert wall_bridge.logger.messages == [
            self.DETECTED_SAID, ams_echo(self.DETECTED)]

    def test_a_measurement_handle_line_announced_is_live(self, wall_bridge):
        # Stamped at reactor time 100 s; still live at the 120 s backstop.
        self._measuring(wall_bridge)
        assert wall_bridge._meas_live_t == {0x0700: 100.0}
        assert wall_bridge.cap_calibrating(0x0700) is True
        wall_bridge.reactor.now = 220.0
        assert wall_bridge.cap_calibrating(0x0700) is True
        assert wall_bridge._meas_live == {0x0700: True}
        assert wall_bridge.logger.messages == [
            self.DETECTED_SAID, ams_echo(self.DETECTED)]

    def test_a_stalled_flag_expires(self, wall_bridge):
        # A calibrate that dies at the first edge must not refuse every
        # later calibrate for the rest of the session.
        self._measuring(wall_bridge)
        wall_bridge.reactor.now = 220.5
        assert wall_bridge.cap_calibrating(0x0700) is False
        assert wall_bridge._meas_live == {0x0700: False}
        assert wall_bridge.logger.messages == [
            self.DETECTED_SAID, ams_echo(self.DETECTED)]

    def test_a_reactor_without_a_clock_reads_zero(self, wall_bridge):
        # No monotonic(): the age is taken from 0, so a stamp is never stale.
        self._measuring(wall_bridge)
        wall_bridge.reactor = object()
        assert wall_bridge.cap_calibrating(0x0700) is True
        assert wall_bridge._meas_live == {0x0700: True}


class TestBambuBridgeLastTerminal:
    def test_no_address_is_none(self, bambu_bridge):
        bambu_bridge._rfid_term_by_addr[0x0700] = 77.0
        assert bambu_bridge.last_terminal(None) is None
        assert bambu_bridge.logger.messages == []

    def test_never_finished_is_none(self, bambu_bridge):
        assert bambu_bridge.last_terminal(0x0700) is None
        assert bambu_bridge.logger.messages == []

    def test_reports_the_stamp(self, bambu_bridge):
        bambu_bridge.reactor.advance(77.0)
        line = "[AMS_RFID] STEP7:cali end"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.last_terminal(0x0700) == 177.0
        assert bambu_bridge.last_terminal(0x1800) is None
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS finished its measuring cycle"),
            ams_echo(line)]


class TestBambuBridgeLastCapMeasure:
    def test_no_address_is_none(self, bambu_bridge):
        bambu_bridge._note_cap_measure("odom C:1.234, R:0.084, P:78%", 0x0700, 99.0)
        assert bambu_bridge.last_cap_measure(None) is None
        assert bambu_bridge.logger.messages == []

    def test_never_measured_is_none(self, bambu_bridge):
        assert bambu_bridge.last_cap_measure(0x0700) is None
        assert bambu_bridge.logger.messages == []

    def test_returns_a_copy(self, bambu_bridge):
        bambu_bridge._note_cap_measure("odom C:1.234, R:0.084, P:78%", 0x0700, 99.0)
        got = bambu_bridge.last_cap_measure(0x0700)
        assert (got["pct"], got["radius_m"], got["t"]) == (78, 0.084, 99.0)
        got["pct"] = 1
        assert bambu_bridge._cap_measure[0x0700]["pct"] == 78
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeClearDryError:
    def test_clears_that_unit_only(self, bambu_bridge):
        bambu_bridge._note_dry_refusal("[AMS_CHMB]err, filament hub load!", 0x0700, 1)
        bambu_bridge._note_dry_refusal("[AMS_CHMB]err, ams-ht shell open!", 0x1800, 4)
        bambu_bridge.clear_dry_error(1)
        assert bambu_bridge._dry_err_u == {4: "ams-ht shell open!"}
        assert bambu_bridge.logger.messages == []

    def test_none_unit_clears_nothing(self, bambu_bridge):
        bambu_bridge._note_dry_refusal("[AMS_CHMB]err, filament hub load!", 0x0700, 1)
        bambu_bridge.clear_dry_error(None)
        assert bambu_bridge._dry_err_u == {1: "filament hub load!"}
        assert bambu_bridge.logger.messages == []

    def test_clearing_an_unset_unit_is_safe(self, bambu_bridge):
        bambu_bridge.clear_dry_error(3)
        assert bambu_bridge._dry_err_u == {}
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeLastDryError:
    HUB = "[AMS_CHMB]err, filament hub load!"
    REFUSED = ("info", "AFC bambu bridge: AMS refused the drying command: "
                       "filament hub load!. An AMS will not dry with filament out "
                       "in the hub -- reel the lane back to its bay first "
                       "(LANE_UNLOAD).")
    LID = ("info", "AFC bambu bridge: AMS HT lid is open -- drying continues, "
                   "but the chamber cannot hold temperature until the shell is "
                   "closed.")

    def test_nothing_refused_yet_is_none(self, bambu_bridge):
        assert bambu_bridge.last_dry_error(2) is None
        assert bambu_bridge.last_dry_error(None) is None
        assert bambu_bridge.logger.messages == []

    def test_the_refusal_is_recorded_in_the_units_own_words(self, bambu_bridge):
        # The echo before it proves the command arrived: the unit declined.
        line = ("[AMS_LINK]ret:1,mode:1,temp:55,time:480 "
                "[AMS_CHMB]err, filament hub load! "
                "[AMS_CHMB]update dry_mode:1, ams_state:0")
        feed_amsdbg(bambu_bridge, line, addr=0x1800, unit=2)
        assert bambu_bridge.last_dry_error(2) == "filament hub load!"
        assert bambu_bridge.logger.messages == [self.REFUSED, ams_echo(line)]

    def test_it_is_recorded_against_the_unit_that_said_it(self, bambu_bridge):
        # The address names only the unit class; the chain index names it.
        feed_amsdbg(bambu_bridge, self.HUB, addr=0x1800, unit=2)
        assert bambu_bridge.last_dry_error(0) is None
        assert bambu_bridge.last_dry_error(2) == "filament hub load!"
        assert bambu_bridge.last_dry_error(None) is None
        assert bambu_bridge.logger.messages == [self.REFUSED, ams_echo(self.HUB)]

    def test_heating_clears_it(self, bambu_bridge):
        heating = "[AMS_CHMB]set state CTC_STATE_HEATING"
        feed_amsdbg(bambu_bridge, self.HUB, addr=0x1800, unit=2)
        feed_amsdbg(bambu_bridge, heating, addr=0x1800, unit=2)
        assert bambu_bridge.last_dry_error(2) is None
        assert bambu_bridge.logger.messages == [
            self.REFUSED, ams_echo(self.HUB), ams_echo(heating)]

    def test_a_self_check_clears_it_too(self, bambu_bridge):
        check = "[AMS_CHMB]set state CTC_STATE_SELF_CHECK, from off, ref:55"
        feed_amsdbg(bambu_bridge, self.HUB, addr=0x1800, unit=2)
        feed_amsdbg(bambu_bridge, check, addr=0x1800, unit=2)
        assert bambu_bridge.last_dry_error(2) is None
        assert bambu_bridge.logger.messages == [
            self.REFUSED, ams_echo(self.HUB), ams_echo(check)]

    def test_the_lid_closing_clears_it(self, bambu_bridge):
        opened, closed = "[AMS_CHMB]err, ams-ht shell open!", "[AMS_CHMB]ams-ht shell ok!"
        feed_amsdbg(bambu_bridge, opened, addr=0x1800, unit=2)
        assert bambu_bridge.last_dry_error(2) == "ams-ht shell open!"
        feed_amsdbg(bambu_bridge, closed, addr=0x1800, unit=2)
        assert bambu_bridge.last_dry_error(2) is None
        assert bambu_bridge.logger.messages == [
            self.LID, ams_echo(opened), ams_echo(closed)]

    def test_an_accepted_start_clears_it(self, bambu_bridge):
        # "dry_mode:1, check ok!" announces an accepted start in both
        # dialects.
        opened, accepted = "[AMS_CHMB]err, ams-ht shell open!", "[AMS_CHMB]dry_mode:1, check ok!"
        feed_amsdbg(bambu_bridge, opened, addr=0x1800, unit=2)
        feed_amsdbg(bambu_bridge, accepted, addr=0x1800, unit=2)
        assert bambu_bridge.last_dry_error(2) is None
        assert bambu_bridge.logger.messages == [
            self.LID, ams_echo(opened), ams_echo(accepted)]

    def test_a_repeat_is_still_recorded(self, bambu_bridge):
        # The unit repeats the refusal on every retry: a repeat the console
        # dedupes still records it.
        feed_amsdbg(bambu_bridge, self.HUB, addr=0x1800, unit=2)
        bambu_bridge.clear_dry_error(2)
        feed_amsdbg(bambu_bridge, self.HUB, addr=0x1800, unit=2)
        assert bambu_bridge.last_dry_error(2) == "filament hub load!"
        assert bambu_bridge.logger.messages == [self.REFUSED, ams_echo(self.HUB)]

    def test_an_unattributed_line_is_ignored(self, bambu_bridge):
        # Guessing an owner would put one unit's refusal on another's card.
        feed_amsdbg(bambu_bridge, self.HUB, addr=0x1800)
        assert bambu_bridge._dry_err_u == {}
        assert bambu_bridge.last_dry_error(2) is None
        assert bambu_bridge.logger.messages == [self.REFUSED, ams_echo(self.HUB)]


class TestBambuBridgeFinishSucceeded:
    @pytest.mark.parametrize("line, stored, ok", [
        # No stall at all.
        ("[AMS_SWITCH]feed finish, buff_pos:1.28, bldc_i:1.595A", None, True),
        # The HT's normal end of load: 18 mm short of a 3619 mm path.
        ("[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m, tube_len:3.619 m",
         None, True),
        # The unload that really came up short: 336 mm out.
        ("[AMS_SWITCH]feed finish -1, stall, len_det:3.283 m, tube_len:3.619 m",
         None, False),
        # Either side of the 100 mm arrival tolerance.
        ("[AMS_SWITCH]feed finish -1, stall, len_det:3.520 m, tube_len:3.619 m",
         None, True),
        ("[AMS_SWITCH]feed finish -1, stall, len_det:3.518 m, tube_len:3.619 m",
         None, False),
        # Either word alone opens the judgement.
        ("[AMS_SWITCH]feed finish -1, len_det:3.601 m, tube_len:3.619 m", None, True),
        ("[AMS_SWITCH]feed finish -1, len_det:1.000 m, tube_len:3.619 m", None, False),
        ("[AMS_SWITCH]pull err, bdc stall, len_det:1.000 m, tube_len:3.619 m",
         None, False),
        # A clean completion sharing the line wins.
        ("[AMS_SWITCH]feed finish -1, stall, len_det:1.000 m [AMS_SWITCH]feed "
         "finish, buff_pos:1.28", None, True),
        # Nothing travelled to judge.
        ("[AMS_SWITCH]feed finish -1, stall", 3619.0, False),
        # The unit's stored measurement when the line omits it.
        ("[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m", 3619.0, True),
        ("[AMS_SWITCH]feed finish -1, stall, len_det:1.000 m", 3619.0, False),
        # Nothing to judge against: the stall stands.
        ("[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m", None, False),
    ])
    def test_the_verdict_is_how_far_it_got(self, bambu_bridge, line, stored, ok):
        if stored is not None:
            bambu_bridge._tube_by_addr[0x1800] = stored
        assert bambu_bridge._finish_succeeded(line, line.lower(), 0x1800) is ok
        assert bambu_bridge.logger.messages == []

    def test_tolerance_is_clear_of_both_measured_cases(self, bambu_bridge):
        # A normal end of load stalls 18 mm short; the real short unload was
        # 336 mm short. The tolerance must separate them.
        normal = "[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m, tube_len:3.619 m"
        short = "[AMS_SWITCH]feed finish -1, stall, len_det:3.283 m, tube_len:3.619 m"
        assert bambu_bridge._finish_succeeded(normal, normal.lower(), 0x1800) is True
        assert bambu_bridge._finish_succeeded(short, short.lower(), 0x1800) is False
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeTraceFstate:
    def test_a_change_is_logged_with_the_buffer(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge._trace_fstate({"fstate": 3, "buff": 54})
        assert bridge._nar_lg.messages == [("debug", "HOST-- fstate - -> 3 (buff=54)")]
        assert bridge._fstate_last == 3
        assert bridge.logger.messages == []

    def test_the_same_value_again_is_not_logged(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge._trace_fstate({"fstate": 3, "buff": 54})
        bridge._trace_fstate({"fstate": 3, "buff": 60})
        assert bridge._nar_lg.messages == [("debug", "HOST-- fstate - -> 3 (buff=54)")]
        assert bridge.logger.messages == []

    def test_a_transition_names_both_states(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge._trace_fstate({"fstate": 3, "buff": 54})
        bridge._trace_fstate({"fstate": 0, "buff": 58})
        assert bridge._nar_lg.messages == [
            ("debug", "HOST-- fstate - -> 3 (buff=54)"),
            ("debug", "HOST-- fstate 3 -> 0 (buff=58)")]
        assert bridge._fstate_last == 0
        assert bridge.logger.messages == []

    def test_no_log_configured_is_a_noop(self, bambu_bridge):
        bambu_bridge._trace_fstate({"fstate": 3})
        assert bambu_bridge._fstate_last is bridge_mod._UNSET
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeTubeLen:
    """
    The unit self-calibrates its filament path and narrates the result. It is
    kept per device address, and per chain index while the host names the
    unit it is commanding, because an AMS 1 and an AMS 2 Pro share 0x0700.
    """

    @staticmethod
    def _adopted(addr: str, mm: int) -> LogLine:
        """
        :param addr: the device address as the console line prints it
        :param mm: the measured length
        :return LogLine: the one-time console line for a first measurement
        """
        return ("info", f"AFC bambu bridge: AMS {addr} reports its measured "
                        f"filament path as {mm}mm -- using it to size move "
                        f"timeouts instead of the configured estimate")

    @staticmethod
    def _learned(mm: int) -> LogLine:
        """
        :param mm: the measured length
        :return LogLine: the console line for a bare "new tube_len" report
        """
        return ("info", f"AFC bambu bridge: AMS learned the bay-to-hub path "
                        f"length: {mm} mm")

    def test_the_new_tube_len_form_is_read(self, bambu_bridge):
        # "new tube_len" here against the HT's "old tube_len".
        line = "[AMS_SWITCH]new tube_len:3503 mm, list:3500,3507,0 mm, err:7 mm"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700) == 3503.0
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS measured the PTFE path at 3503mm "
                     "(+/-7mm)"),
            self._adopted("0x0700", 3503),
            ams_echo(line)]

    def test_address_keying_still_works_with_no_active_unit(self, bambu_bridge):
        line = "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700) == 3532.0
        assert bambu_bridge._tube_by_unit == {}
        assert bambu_bridge.logger.messages == [
            self._learned(3532), self._adopted("0x0700", 3532), ams_echo(line)]

    def test_two_units_on_one_address_do_not_collide(self, bambu_bridge):
        first = "[AMS_SWITCH]new tube_len:3000 mm, list:3000,3000,0 mm"
        second = "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm"
        bambu_bridge.set_active_unit(1)
        feed_amsdbg(bambu_bridge, first, addr=0x0700)
        bambu_bridge.set_active_unit(2)
        feed_amsdbg(bambu_bridge, second, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700, unit=1) == 3000.0
        assert bambu_bridge.tube_len(0x0700, unit=2) == 3532.0
        # The address keeps whichever spoke last; the second report inside
        # the console's one-second floor is in AFC.log only.
        assert bambu_bridge.tube_len(0x0700) == 3532.0
        assert bambu_bridge.logger.messages == [
            self._learned(3000), self._adopted("0x0700", 3000), ams_echo(first),
            ams_echo(second)]

    def test_clearing_the_active_unit_stops_attributing(self, bambu_bridge):
        first = "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm"
        second = "[AMS_SWITCH]new tube_len:9999 mm, list:9999,9999,0 mm"
        bambu_bridge.set_active_unit(2)
        feed_amsdbg(bambu_bridge, first, addr=0x0700)
        bambu_bridge.set_active_unit(None)
        feed_amsdbg(bambu_bridge, second, addr=0x0700)
        assert bambu_bridge._active_unit is None
        assert bambu_bridge._tube_by_unit == {2: 3532.0}
        assert bambu_bridge.tube_len(0x0700, unit=2) == 3532.0
        assert bambu_bridge.tube_len(0x0700) == 9999.0
        assert bambu_bridge.logger.messages == [
            self._learned(3532), self._adopted("0x0700", 3532), ams_echo(first),
            ams_echo(second)]

    def test_an_uncalibrated_zero_is_still_dropped(self, bambu_bridge):
        line = "[AMS_SWITCH]old tube_len:0 mm, list:3534,0,0 mm"
        bambu_bridge.set_active_unit(2)
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700, unit=2) is None
        assert bambu_bridge._tube_by_unit == {}
        assert bambu_bridge._tube_by_addr == {}
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_unit_that_never_measured_reads_nothing(self, bambu_bridge):
        # No falling back to the address: on a bus with two boxed units that
        # hands unit 1 whatever unit 2 last measured.
        line = "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm"
        bambu_bridge.set_active_unit(2)
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700, unit=2) == 3532.0
        assert bambu_bridge.tube_len(0x0700, unit=1) is None      # NOT 3532
        assert bambu_bridge.logger.messages == [
            self._learned(3532), self._adopted("0x0700", 3532), ams_echo(line)]

    def test_the_address_fallback_still_works_before_any_attribution(
            self, bambu_bridge):
        # Single-unit bus, or anything that never set an active unit.
        line = "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700, unit=1) == 3532.0
        assert bambu_bridge.tube_len(0x0700) == 3532.0
        assert bambu_bridge.logger.messages == [
            self._learned(3532), self._adopted("0x0700", 3532), ams_echo(line)]

    def test_each_unit_keeps_its_own_once_both_have_measured(self, bambu_bridge):
        first = "[AMS_SWITCH]new tube_len:2900 mm, list:2900,2900,0 mm"
        second = "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm"
        bambu_bridge.set_active_unit(1)
        feed_amsdbg(bambu_bridge, first, addr=0x0700)
        bambu_bridge.reactor.advance(1.1)
        bambu_bridge.set_active_unit(2)
        feed_amsdbg(bambu_bridge, second, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700, unit=1) == 2900.0
        assert bambu_bridge.tube_len(0x0700, unit=2) == 3532.0
        # Past the console floor the second report is said too, but the
        # address is not new, so the one-time adoption line is not repeated.
        assert bambu_bridge.logger.messages == [
            self._learned(2900), self._adopted("0x0700", 2900), ams_echo(first),
            self._learned(3532), ams_echo(second)]

    def test_an_unmeasured_unit_keeps_its_configured_value(self, bambu_bridge):
        # None is the correct answer: _adopt_measured_path leaves the
        # configured length alone rather than adopting a neighbour's.
        line = "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm"
        bambu_bridge.set_active_unit(2)
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(0x1800, unit=0) is None
        assert bambu_bridge.logger.messages == [
            self._learned(3532), self._adopted("0x0700", 3532), ams_echo(line)]

    def test_the_mm_form_is_captured(self, bambu_bridge):
        line = "[AMS_SWITCH]new tube_len:3481 mm, list:3491,3472,0 mm, err:19 mm"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700) == 3481.0
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS measured the PTFE path at 3481mm "
                     "(+/-19mm)"),
            self._adopted("0x0700", 3481),
            ams_echo(line)]

    def test_the_metre_form_is_captured_and_converted(self, bambu_bridge):
        # This form appears on a STALL line, which is exactly when knowing the
        # calibrated length matters most.
        line = ("[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m, "
                "tube_len:2.186 m")
        feed_amsdbg(bambu_bridge, line, addr=0x1800)
        assert bambu_bridge.tube_len(0x1800) == 2186.0
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS: the filament STALLED after 3.71 m "
                     "of a 2.19 m path -- check for a jam between the bay and "
                     "the toolhead"),
            self._adopted("0x1800", 2186),
            ams_echo(line)]

    def test_zero_is_not_adopted(self, bambu_bridge):
        # The unit reports 0 until it has enough samples. Adopting that would
        # set every derived deadline to zero.
        line = "[AMS_SWITCH]new tube_len:0 mm, list:3491,0,0 mm, err:3491 mm"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(0x0700) is None
        assert bambu_bridge._tube_by_addr == {}
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS measured the PTFE path at 0mm "
                     "(+/-3491mm)"),
            ams_echo(line)]

    def test_zero_in_metres_is_not_adopted(self, bambu_bridge):
        line = ("[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m, "
                "tube_len:0.000 m")
        feed_amsdbg(bambu_bridge, line, addr=0x1800)
        assert bambu_bridge.tube_len(0x1800) is None
        assert bambu_bridge._tube_by_addr == {}
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS: the filament STALLED after 3.71 m "
                     "of a 0.00 m path -- check for a jam between the bay and "
                     "the toolhead"),
            ams_echo(line)]

    def test_two_units_do_not_share_a_measurement(self, bambu_bridge):
        # The exact cross-unit attribution bug the chamber telemetry already
        # had: an HT's path length must never be served to an AMS 2.
        feed_amsdbg(bambu_bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        feed_amsdbg(bambu_bridge, "[AMS_SWITCH]new tube_len:1693 mm", addr=0x1800)
        assert bambu_bridge.tube_len(0x0700) == 3481.0
        assert bambu_bridge.tube_len(0x1800) == 1693.0
        assert bambu_bridge.logger.messages == [
            self._learned(3481), self._adopted("0x0700", 3481),
            ams_echo("[AMS_SWITCH]new tube_len:3481 mm"),
            self._adopted("0x1800", 1693),
            ams_echo("[AMS_SWITCH]new tube_len:1693 mm")]

    def test_an_unaddressed_query_is_refused_when_two_units_reported(
            self, bambu_bridge):
        feed_amsdbg(bambu_bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        feed_amsdbg(bambu_bridge, "[AMS_SWITCH]new tube_len:1693 mm", addr=0x1800)
        assert bambu_bridge.tube_len(None) is None
        assert bambu_bridge._tube_by_addr == {0x0700: 3481.0, 0x1800: 1693.0}
        assert bambu_bridge.logger.messages == [
            self._learned(3481), self._adopted("0x0700", 3481),
            ams_echo("[AMS_SWITCH]new tube_len:3481 mm"),
            self._adopted("0x1800", 1693),
            ams_echo("[AMS_SWITCH]new tube_len:1693 mm")]

    def test_an_unaddressed_query_answers_when_only_one_unit_reported(
            self, bambu_bridge):
        line = "[AMS_SWITCH]new tube_len:3481 mm"
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.tube_len(None) == 3481.0
        assert bambu_bridge.logger.messages == [
            self._learned(3481), self._adopted("0x0700", 3481), ams_echo(line)]

    def test_narration_without_an_address_is_not_stored(self, bambu_bridge):
        # Firmware older than 1.0.7.0 does not say who narrated; storing that
        # against a guessed unit is worse than not storing it.
        line = "[AMS_SWITCH]new tube_len:3481 mm"
        feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.tube_len(0x0700) is None
        assert bambu_bridge.tube_len(None) is None
        assert bambu_bridge._tube_by_addr == {}
        # Still said on the console; only the record is withheld.
        assert bambu_bridge.logger.messages == [self._learned(3481), ams_echo(line)]

    def test_a_later_measurement_replaces_the_earlier_one(self, bambu_bridge):
        feed_amsdbg(bambu_bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        feed_amsdbg(bambu_bridge, "[AMS_SWITCH]new tube_len:3502 mm", addr=0x0700)
        assert bambu_bridge.tube_len(0x0700) == 3502.0
        assert bambu_bridge.logger.messages == [
            self._learned(3481), self._adopted("0x0700", 3481),
            ams_echo("[AMS_SWITCH]new tube_len:3481 mm"),
            ams_echo("[AMS_SWITCH]new tube_len:3502 mm")]

    def test_an_unknown_unit_reports_nothing(self, bambu_bridge):
        assert bambu_bridge.tube_len(0x0700) is None
        assert bambu_bridge.tube_len(None) is None
        assert bambu_bridge.logger.messages == []

    def test_a_matching_line_does_not_block_later_parsing(self, monkeypatch):
        # Built as production builds it, with no name assigned: a failure in
        # the _AMS_HUMAN render used to skip the tube_len parse after it.
        reactor = FakeReactor()
        patch_module_time(monkeypatch, reactor)
        logger = BambuLogger()
        bridge = BambuBridge(FakeSerial, reactor, logger)
        line = "[AMS_SWITCH]new tube_len:3481 mm, list:3491,3472,0 mm, err:19 mm"
        feed_amsdbg(bridge, line, addr=0x0700)
        assert bridge.tube_len(0x0700) == 3481.0
        assert logger.messages == [
            ("info", "AFC bambu bridge: AMS measured the PTFE path at 3481mm "
                     "(+/-19mm)"),
            self._adopted("0x0700", 3481),
            ams_echo(line)]


class TestBambuBridgeLastFinish:
    """
    _wait_move returns the instant the finish sequence bumps, so what counts
    as a finish decides where AFC thinks the filament is. Lines are verbatim
    from the units that emitted them.
    """

    ODOM_RESET = "[AMS_DEV] STEP:odom reset tray 0"
    TRAY_GONE = "[AMS_DEV] STEP:odom tray_id error 255"
    STATE_SWITCH = "[AMS_IDLE]set ams state switch"
    NO_TRAY = ("AFC bambu bridge: AMS: asked to move with NO TRAY SELECTED -- "
               "the unit rejected the command")
    HT_PATH = ("info", "AFC bambu bridge: AMS 0x1800 reports its measured "
                       "filament path as 3619mm -- using it to size move "
                       "timeouts instead of the configured estimate")

    @staticmethod
    def _say(bridge: BambuBridge, text: str,
             addr: Optional[int] = 0x0700) -> Tuple[int, bool, str]:
        """
        :param bridge: the bridge
        :param text: the AMS's narration
        :param addr: the device address the frame names
        :return tuple: last_finish() afterwards
        """
        feed_amsdbg(bridge, text, addr=addr)
        return bridge.last_finish()

    @staticmethod
    def _stalled(got: str, path: str) -> LogLine:
        """
        :param got: len_det as the console prints it
        :param path: tube_len as the console prints it
        :return LogLine: the console line for a stalled feed
        """
        return ("info", f"AFC bambu bridge: AMS: the filament STALLED after "
                        f"{got} m of a {path} m path -- check for a jam "
                        f"between the bay and the toolhead")

    # ── what counts as a completion ──

    def test_a_real_feed_completion_counts(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish, buff_pos:1.29, bldc_i:1.593A"
        assert self._say(bambu_bridge, line, 0x1800) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_the_ams2_form_with_an_index_counts(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish 0, dw_len:3.508 m"
        assert self._say(bambu_bridge, line, 0x1800) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_pull_completion_counts(self, bambu_bridge):
        line = "[AMS_SWITCH]pull finish 0, tray_sw:0, len_det:0.265 m"
        assert self._say(bambu_bridge, line, 0x1800) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_state_machine_switch_does_NOT_count(self, bambu_bridge):
        # Emitted ~10 times in the seconds before the feed completes. Counting
        # it called the load done somewhere mid-bowden.
        line = ("[AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, "
                "err_code:0x00")
        for _ in range(10):
            self._say(bambu_bridge, line, 0x1800)
        assert bambu_bridge.last_finish() == (0, False, "")
        # Routine, so AFC.log only, and the repeats are deduped.
        assert bambu_bridge.logger.messages == [ams_echo(line)]
        assert bambu_bridge.logger.file_only == [ams_echo(line)[1]]

    def test_the_follower_dropping_does_NOT_count(self, bambu_bridge):
        line = ("[AMS_COMMON]mode: 4 -> 0 [AMS_SWITCH]assist finish 0, ref:0 "
                "[AMS_LED]other to idle 0")
        assert self._say(bambu_bridge, line, 0x1800) == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_preload_completion_still_counts(self, bambu_bridge):
        line = "[AMS_PRELOAD]preload finish"
        assert self._say(bambu_bridge, line, 0x1800) == (1, True, line)
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS staged the spool at its feeder"),
            ams_echo(line)]

    def test_a_real_finish_in_a_blob_of_noise_still_counts(self, bambu_bridge):
        # Narration arrives as several bracketed segments per line, so the
        # completion routinely shares a line with the noise above.
        line = ("[AMS_SWITCH]feed finish, buff_pos:1.29 [AMS_IDLE]set "
                "ams_state:2 --> 0 [AMS_SWITCH]AMS_CTRL_state_switch finish, "
                "sucessful, err_code:0x00")
        assert self._say(bambu_bridge, line, 0x1800) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_stalled_completion_is_still_reported_but_not_ok(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish -1, stall"
        assert self._say(bambu_bridge, line, 0x1800) == (1, False, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    # ── a stall is judged by how far it got ──

    def test_a_clean_finish_is_ok(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish, buff_pos:1.28, bldc_i:1.595A"
        assert self._say(bambu_bridge, line, 0x1800) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_the_ht_end_of_load_stall_is_ok(self, bambu_bridge):
        # Verbatim: 18 mm short of a 3619 mm path. The HT ends a normal load
        # by stalling against the extruder gear.
        line = ("[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m, "
                "tube_len:3.619 m")
        assert self._say(bambu_bridge, line, 0x1800) == (1, True, line)
        assert bambu_bridge.logger.messages == [
            self._stalled("3.60", "3.62"), self.HT_PATH, ams_echo(line)]

    def test_a_genuinely_short_stall_is_not_ok(self, bambu_bridge):
        # Verbatim shape of the unload that really did come up short: 336 mm
        # out, and it needed its retry.
        line = ("[AMS_SWITCH]feed finish -1, stall, len_det:3.283 m, "
                "tube_len:3.619 m")
        assert self._say(bambu_bridge, line, 0x1800) == (1, False, line)
        assert bambu_bridge.logger.messages == [
            self._stalled("3.28", "3.62"), self.HT_PATH, ams_echo(line)]

    def test_a_stall_at_the_very_start_is_not_ok(self, bambu_bridge):
        line = ("[AMS_SWITCH]feed finish -1, stall, len_det:0.050 m, "
                "tube_len:3.619 m")
        assert self._say(bambu_bridge, line, 0x1800) == (1, False, line)
        assert bambu_bridge.logger.messages == [
            self._stalled("0.05", "3.62"), self.HT_PATH, ams_echo(line)]

    def test_the_stored_measurement_is_used_when_the_line_omits_it(
            self, bambu_bridge):
        # A stall line without tube_len must still be judged against the right
        # distance rather than defaulting to failure.
        measured = "[AMS_SWITCH]old tube_len:3619 mm, list:3617,3645,0 mm"
        arrived = "[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m"
        short = "[AMS_SWITCH]feed finish -1, stall, len_det:1.000 m"
        assert self._say(bambu_bridge, measured, 0x1800) == (0, False, "")
        assert self._say(bambu_bridge, arrived, 0x1800) == (1, True, arrived)
        assert self._say(bambu_bridge, short, 0x1800) == (2, False, short)
        assert bambu_bridge.logger.messages == [
            self.HT_PATH, ams_echo(measured), ams_echo(arrived), ams_echo(short)]

    def test_a_stall_with_nothing_to_judge_against_stays_a_failure(
            self, bambu_bridge):
        # No len_det, no measurement: the safe reading is that it failed.
        line = "[AMS_SWITCH]feed finish -1, stall"
        assert self._say(bambu_bridge, line, 0x1800) == (1, False, line)
        assert bambu_bridge.tube_len(0x1800) is None
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_clean_finish_sharing_the_line_wins(self, bambu_bridge):
        # Exactly what the HT emitted: the stall and the real completion
        # arrive in one narration blob.
        line = ("[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m, "
                "tube_len:3.619 m [AMS_RFID] STEP,odom reset tray 0 "
                "[AMS_SWITCH]feed finish, buff_pos:1.28, bldc_i:1.600A")
        assert self._say(bambu_bridge, line, 0x1800) == (1, True, line)
        assert bambu_bridge.logger.messages == [
            self._stalled("3.60", "3.62"), self.HT_PATH, ams_echo(line)]

    def test_the_minus_one_form_alone_does_not_read_as_clean(self, bambu_bridge):
        # The clean-finish pattern must not match "feed finish -1".
        line = ("[AMS_SWITCH]feed finish -1, stall, len_det:0.100 m, "
                "tube_len:3.619 m")
        assert self._say(bambu_bridge, line, 0x1800) == (1, False, line)
        assert bambu_bridge.logger.messages == [
            self._stalled("0.10", "3.62"), self.HT_PATH, ams_echo(line)]

    def test_a_stalled_completion_still_bumps_the_sequence(self, bambu_bridge):
        # Whatever the verdict, the caller must learn the move ended, or it
        # waits out the deadline it was meant to be spared.
        line = "[AMS_SWITCH]feed finish -1, stall"
        self._say(bambu_bridge, "[AMS_SWITCH]feed finish, buff_pos:1.28", 0x1800)
        assert self._say(bambu_bridge, line, 0x1800) == (2, False, line)
        assert bambu_bridge.logger.messages == [
            ams_echo("[AMS_SWITCH]feed finish, buff_pos:1.28"), ams_echo(line)]

    # ── the [AMS_DEV] dialect's odometer completions ──

    def test_an_odom_reset_completes_a_feed(self, bambu_bridge):
        assert self._say(bambu_bridge, self.ODOM_RESET) == (1, True, self.ODOM_RESET)
        assert bambu_bridge._tray_gone is False
        assert bambu_bridge.logger.messages == [ams_echo(self.ODOM_RESET)]
        assert bambu_bridge.logger.file_only == [ams_echo(self.ODOM_RESET)[1]]

    def test_the_tray_going_away_completes_a_retract(self, bambu_bridge):
        self._say(bambu_bridge, self.ODOM_RESET)     # engaged
        assert self._say(bambu_bridge, self.TRAY_GONE) == (2, True, self.TRAY_GONE)
        assert bambu_bridge._tray_gone is True
        assert bambu_bridge.logger.messages == [
            ams_echo(self.ODOM_RESET), ("debug", self.NO_TRAY),
            ams_echo(self.TRAY_GONE)]
        assert bambu_bridge.logger.file_only == [
            ams_echo(self.ODOM_RESET)[1], self.NO_TRAY]

    def test_the_repeat_does_NOT_keep_completing(self, bambu_bridge):
        # Repeated at ~2 Hz. Counting each one leaves a completion pending, so
        # the next move would report done the instant it starts waiting.
        self._say(bambu_bridge, self.ODOM_RESET)
        self._say(bambu_bridge, self.TRAY_GONE)
        for _ in range(20):
            bambu_bridge.reactor.advance(0.5)
            self._say(bambu_bridge, self.STATE_SWITCH)
            self._say(bambu_bridge, self.TRAY_GONE)
        assert bambu_bridge.last_finish() == (2, True, self.TRAY_GONE)
        assert bambu_bridge.logger.messages == (
            [ams_echo(self.ODOM_RESET), ("debug", self.NO_TRAY),
             ams_echo(self.TRAY_GONE)]
            + [ams_echo(self.STATE_SWITCH), ("debug", self.NO_TRAY),
               ams_echo(self.TRAY_GONE)] * 20)
        assert bambu_bridge.logger.file_only == (
            [ams_echo(self.ODOM_RESET)[1], self.NO_TRAY]
            + [ams_echo(self.STATE_SWITCH)[1], self.NO_TRAY] * 20)

    def test_a_new_tray_re_arms_the_edge(self, bambu_bridge):
        # Load, unload, load, unload must give four completions, not two.
        for line in (self.ODOM_RESET, self.TRAY_GONE) * 2:
            self._say(bambu_bridge, line)
        assert bambu_bridge.last_finish() == (4, True, self.TRAY_GONE)
        assert bambu_bridge.logger.messages == [
            ams_echo(self.ODOM_RESET), ("debug", self.NO_TRAY),
            ams_echo(self.TRAY_GONE)] * 2

    def test_the_interleaved_state_lines_do_not_re_arm_it(self, bambu_bridge):
        # The churn alternates with "set ams state switch"; if that re-armed
        # the latch we would be back to counting every repeat.
        self._say(bambu_bridge, self.TRAY_GONE)
        self._say(bambu_bridge, self.STATE_SWITCH)
        self._say(bambu_bridge, self.TRAY_GONE)
        assert bambu_bridge.last_finish() == (1, True, self.TRAY_GONE)
        assert bambu_bridge.logger.messages == [
            ("debug", self.NO_TRAY), ams_echo(self.TRAY_GONE),
            ams_echo(self.STATE_SWITCH), ("debug", self.NO_TRAY),
            ams_echo(self.TRAY_GONE)]

    def test_a_real_finish_line_also_re_arms_the_edge(self, bambu_bridge):
        # An HT-dialect completion means a tray is engaged again just as much
        # as an odom reset does.
        finish = "[AMS_SWITCH]feed finish, buff_pos:1.28"
        self._say(bambu_bridge, self.TRAY_GONE)
        assert self._say(bambu_bridge, finish, 0x1800) == (2, True, finish)
        assert bambu_bridge._tray_gone is False
        assert self._say(bambu_bridge, self.TRAY_GONE) == (3, True, self.TRAY_GONE)
        assert bambu_bridge.logger.messages == [
            ("debug", self.NO_TRAY), ams_echo(self.TRAY_GONE), ams_echo(finish),
            ("debug", self.NO_TRAY), ams_echo(self.TRAY_GONE)]

    def test_the_ht_blob_is_still_judged_as_a_finish_not_an_odom_reset(
            self, bambu_bridge):
        # The HT emits odom reset INSIDE its finish blob. The finish rule must
        # win, or a stalled-short feed would be scored a success by the reset.
        line = ("[AMS_SWITCH]feed finish -1, stall, len_det:1.000 m, "
                "tube_len:3.619 m [AMS_RFID] STEP,odom reset tray 0")
        assert self._say(bambu_bridge, line, 0x1800) == (1, False, line)
        assert bambu_bridge.logger.messages == [
            self._stalled("1.00", "3.62"), self.HT_PATH, ams_echo(line)]

    def test_ordinary_dev_narration_is_not_a_completion(self, bambu_bridge):
        lines = ["[AMS_DEV] STEP2:feed tray 0 to switch",
                 "[AMS_IDLE]set ams state assist, mode:4",
                 "[AMS_DEV] STEP3:start,read all card"]
        for line in lines:
            self._say(bambu_bridge, line)
        assert bambu_bridge.last_finish() == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(x) for x in lines]

    # ── the AMS 2 Pro's own words ──

    def test_pull_sucess_completes_an_unload(self, bambu_bridge):
        # The unit does NOT say "finish" on the way out. Without this its
        # every unload runs the full watchdog.
        line = "[AMS_SWITCH]pull sucess,cond match,... bdc_i:0.464A;spd:-20.1cm/s"
        assert self._say(bambu_bridge, line) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_the_spaced_spelling_also_completes(self, bambu_bridge):
        line = "[AMS_SWITCH]pull sucess, cond match"
        assert self._say(bambu_bridge, line) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_the_state_machine_sucessful_still_does_NOT_complete(
            self, bambu_bridge):
        # Shares the misspelling and occurs 242 times in one night's log.
        line = ("[AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, "
                "err_code:0x80")
        for _ in range(5):
            self._say(bambu_bridge, line)
        assert bambu_bridge.last_finish() == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_the_feed_completion_form_is_covered(self, bambu_bridge):
        line = ("[AMS_SWITCH]feed finish 0, mode:4, dw_len:3.508 m, idx_set:3, "
                "idx_ref:3")
        assert self._say(bambu_bridge, line) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_e_in_does_NOT_complete_a_move(self, bambu_bridge):
        # In both captures an err_code change follows within a second, so it
        # may be an error report; it does not complete a move.
        line = "[AMS_SWITCH]e_in tray:0,buff_pos:-0.34,i:0.566A,len:1.670m"
        assert self._say(bambu_bridge, line) == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_refill_is_not_mistaken_for_a_motion_completion(self, bambu_bridge):
        # It happens continuously during a print; counting it would report a
        # move finishing every time the extruder pulled.
        line = "[AMS_SWITCH]BUFF,pos:0.10->0.74, det:28mm"
        for _ in range(10):
            self._say(bambu_bridge, line)
        assert bambu_bridge.last_finish() == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    # ── tray_now:255 is not a completion ──

    def test_the_state_line_does_NOT_complete_a_retract(self, bambu_bridge):
        line = "[AMS_COMMON]state:2,tray_now:255,tray_exit:1"
        assert self._say(bambu_bridge, line) == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_it_does_not_complete_while_following_either(self, bambu_bridge):
        # The verbatim lines that disproved the reading: loaded and following.
        lines = ["[AMS_COMMON]state:4,tray_now:255,tray_exit:1",
                 "[AMS_SWITCH]tray:0, bldc slip, dw_pos:-0.000 m"]
        for line in lines:
            self._say(bambu_bridge, line)
        assert bambu_bridge.last_finish() == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(x) for x in lines]

    def test_the_odometer_form_still_completes_one(self, bambu_bridge):
        # A boxed AMS's own marker is unaffected and still works.
        assert self._say(bambu_bridge, self.TRAY_GONE) == (1, True, self.TRAY_GONE)
        assert bambu_bridge.logger.messages == [
            ("debug", self.NO_TRAY), ams_echo(self.TRAY_GONE)]

    # ── unaddressed narration ──

    def test_feed_finish_marks_success(self, bambu_bridge):
        line = "[AMS_SWITCH]feed finish,buff_pos:1.36"
        assert self._say(bambu_bridge, line, None) == (1, True, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_stall_marks_failure(self, bambu_bridge):
        # "-1" and "stall" say the move did not do what it was asked: the case
        # a duration-based wait cannot detect.
        line = "[AMS_SWITCH]feed finish -1, stall, len_det:1.156 m"
        assert self._say(bambu_bridge, line, None) == (1, False, line)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_sequence_increments_so_waiters_see_a_fresh_event(self, bambu_bridge):
        preload = "[AMS_PRELOAD]preload finish, sw_sta:1"
        feed = "[AMS_SWITCH]feed finish,buff_pos:1.2"
        assert self._say(bambu_bridge, preload, None) == (1, True, preload)
        assert self._say(bambu_bridge, feed, None) == (2, True, feed)
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS staged the spool at its feeder"),
            ams_echo(preload), ams_echo(feed)]

    def test_non_finish_narration_leaves_it_alone(self, bambu_bridge):
        line = "[AMS_PMSM]mode:0->2"
        assert self._say(bambu_bridge, line, None) == (0, False, "")
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_a_suppressed_line_still_reaches_the_parsers(self, bambu_bridge):
        # The dedupe hides a byte-identical repeat from the log and nothing
        # else: everything downstream reads the raw line.
        pull = "[AMS_SWITCH]pull sucess, mode change, mode:4"
        assert self._say(bambu_bridge, pull) == (1, True, pull)
        assert self._say(bambu_bridge, pull) == (2, True, pull)
        assert bambu_bridge.logger.messages == [ams_echo(pull)]


class TestBambuBridgeLastTrayRelease:
    """
    ``tray_now`` -> 255 is the AMS 2's real unload completion. It is exposed
    as an edge scoped to a unit, with the tray it left, because the resting
    level IS 255 and an operator handling spools makes the same edge.
    """

    @staticmethod
    def _narrate(bridge: BambuBridge, text: str, unit: Optional[int] = 0) -> None:
        """
        :param bridge: the bridge
        :param text: the AMS's narration, from 0x0700
        :param unit: the chain index the frame names; None leaves it out
        """
        feed_amsdbg(bridge, text, addr=0x0700, unit=unit)

    def test_the_release_edge_is_reported_with_the_tray_it_left(
            self, bambu_bridge):
        # Loaded on tray 1, exactly as the unit says it while unloading.
        loaded = "[AMS_COMMON]state:0,tray_now:1,tray_exit:7"
        # The release, verbatim from 14:22:32.
        release = ("[AMS_TRAY]tray[1] sw_sta update, 3 -> 1, u_in_out:2982,2509 "
                   "[AMS_COMMON]state:2,tray_now:255,tray_exit:7")
        self._narrate(bambu_bridge, loaded)
        assert bambu_bridge.last_tray_release(unit=0) == (0, None)
        self._narrate(bambu_bridge, release)
        assert bambu_bridge.last_tray_release(unit=0) == (1, 1)
        assert bambu_bridge.logger.messages == [ams_echo(loaded), ams_echo(release)]

    def test_the_resting_level_is_not_an_edge(self, bambu_bridge):
        # state:0,tray_now:255 is where the unit rests (120 hits in a day).
        # Read as a level it would end every wait instantly.
        line = "[AMS_COMMON]state:0,tray_now:255,tray_exit:7"
        for _ in range(5):
            self._narrate(bambu_bridge, line)
        assert bambu_bridge.last_tray_release(unit=0) == (0, None)
        assert bambu_bridge._tray_now_by_unit == {0: 255}
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_one_line_carrying_the_whole_transition_still_counts(
            self, bambu_bridge):
        # A frame can hold several [AMS_COMMON] segments; the edge may be
        # between two of them, so the last value alone would miss it.
        line = ("[AMS_COMMON]state:0,tray_now:0,tray_exit:7 "
                "[AMS_COMMON]state:2,tray_now:255,tray_exit:7")
        self._narrate(bambu_bridge, line)
        assert bambu_bridge.last_tray_release(unit=0) == (1, 0)
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_each_unload_advances_the_sequence_once(self, bambu_bridge):
        release = "[AMS_COMMON]state:2,tray_now:255,tray_exit:7"
        expected: List[LogLine] = []
        for tray in (1, 0, 1):
            loaded = f"[AMS_COMMON]state:0,tray_now:{tray},tray_exit:7"
            self._narrate(bambu_bridge, loaded)
            self._narrate(bambu_bridge, release)
            expected += [ams_echo(loaded), ams_echo(release)]
        assert bambu_bridge.last_tray_release(unit=0) == (3, 1)
        assert bambu_bridge.logger.messages == expected

    def test_units_are_kept_apart(self, bambu_bridge):
        # Both boxed units answer on 0x0700, so only the narration's own unit
        # index separates them.
        lines = [("[AMS_COMMON]state:0,tray_now:1,tray_exit:7", 0),
                 ("[AMS_COMMON]state:0,tray_now:3,tray_exit:7", 1),
                 ("[AMS_COMMON]state:2,tray_now:255,tray_exit:7", 1)]
        for text, unit in lines:
            self._narrate(bambu_bridge, text, unit=unit)
        assert bambu_bridge.last_tray_release(unit=0) == (0, None)   # untouched
        assert bambu_bridge.last_tray_release(unit=1) == (1, 3)
        assert bambu_bridge._tray_now_by_unit == {0: 1, 1: 255}
        assert bambu_bridge.logger.messages == [ams_echo(t) for t, _u in lines]

    def test_unattributed_narration_is_dropped_not_applied_broadly(
            self, bambu_bridge):
        # This ends a move. A guess would let a neighbour end our retract.
        lines = ["[AMS_COMMON]state:0,tray_now:1", "[AMS_COMMON]state:2,tray_now:255"]
        for line in lines:
            self._narrate(bambu_bridge, line, unit=None)
        assert bambu_bridge.last_tray_release(unit=0) == (0, None)
        assert bambu_bridge.last_tray_release() == (0, None)
        assert bambu_bridge._tray_now_by_unit == {}
        assert bambu_bridge.logger.messages == [ams_echo(x) for x in lines]

    def test_a_spool_insert_still_produces_an_edge(self, bambu_bridge):
        # Verbatim from 19:19:02: an insert, not an unload. The parser cannot
        # know why the tray changed; rejecting it is the waiter's job.
        loaded = "[AMS_COMMON]state:0,tray_now:2,tray_exit:5"
        insert = ("[AMS_TRAY]tray[2] sw_sta update, 3 -> 1, u_in_out:3074,2508 "
                  "[AMS_COMMON]state:3,tray_now:255,tray_exit:5")
        self._narrate(bambu_bridge, loaded)
        self._narrate(bambu_bridge, insert)
        assert bambu_bridge.last_tray_release(unit=0) == (1, 2)
        assert bambu_bridge.logger.messages == [ams_echo(loaded), ams_echo(insert)]


class TestBambuBridgeLatestStatus:
    def test_latest_status_is_a_copy(self, monkeypatch):
        seen: List[dict] = []
        bridge = make_bambu_bridge(monkeypatch, listener=seen.append)
        assert bridge.latest_status() is None
        bridge.handle_line('{"evt":"status","online":true,"slots":[]}')
        snap = bridge.latest_status()
        assert snap == {"evt": "status", "online": True, "slots": []}
        snap["online"] = False
        assert bridge.latest_status()["online"] is True   # cache untouched
        # The listeners get the frame itself, on the reactor.
        assert seen == []
        bridge.reactor.run_callbacks()
        assert seen == [{"evt": "status", "online": True, "slots": []}]
        # The first status frame of a connection asks for the board's info.
        assert bridge.logger.messages == [
            ("info", 'AFC bambu: request_info -> sent {"cmd":"info"}')]


class TestBambuBridgeSend:
    def test_queues_the_json_line(self, bambu_bridge):
        # send() runs on the reactor and only queues: a 125 ms write block on
        # a busy Pico during a homing move shut an MCU down.
        bambu_bridge.send({"cmd": "select", "slot": 2})
        assert bambu_bridge._serial.written == []
        assert bambu_bridge._wq.get_nowait() == b'{"cmd": "select", "slot": 2}\n'
        assert bambu_bridge._wq.empty()
        assert bambu_bridge.logger.messages == []

    def test_send_without_serial_is_noop(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, connected=False)
        bridge.send({"cmd": "stop"})                # no raise, nothing queued
        assert bridge._wq.empty()
        assert bridge.logger.messages == [
            ("warning", "AFC bambu: link is down -- dropped stop, and anything "
                        "else sent until it is back")]

    def test_first_dropped_command_says_so_once_per_outage(self, bambu_bridge):
        # Silence was the bug: every command issued while the port was gone
        # vanished with no log at all, so a dead link read as an idle one.
        bambu_bridge._drop_port()
        assert bambu_bridge._down_epoch == 1
        for _ in range(5):
            bambu_bridge.send({"cmd": "status"})
        assert bambu_bridge.logger.messages == [
            ("warning", "AFC bambu: link is down -- dropped status, and "
                        "anything else sent until it is back")]
        # Back, then gone again: a NEW outage speaks again.
        bambu_bridge._serial = bambu_bridge._serial_factory()
        bambu_bridge._drop_port()
        assert bambu_bridge._down_epoch == 2
        bambu_bridge.send({"cmd": "feed"})
        assert bambu_bridge._wq.empty()
        assert bambu_bridge.logger.messages == [
            ("warning", "AFC bambu: link is down -- dropped status, and "
                        "anything else sent until it is back"),
            ("warning", "AFC bambu: link is down -- dropped feed, and anything "
                        "else sent until it is back")]


class TestBambuBridgeNarrateHuman:
    """
    The AMS narrates continuously. Only curated lines reach the console, at
    most one a second, and never the same line twice in a row; chamber
    telemetry is recorded per unit.
    """

    AUTH = "[AMS_DEV] STEP:card auth success!"
    AUTH_SAID = ("info", "AFC bambu BambuAMS_1: AMS: tag authenticated")
    NO_TRAY = "[AMS_RFID]STEP3,odom tray_id error 0"
    NO_TRAY_SAID = ("AFC bambu BambuAMS_1: AMS: asked to move with NO TRAY "
                    "SELECTED -- the unit rejected the command")
    LINE = ("[AMS_CHMB]s:2|rf:55,0|vt:44.0|ap:35.3|hts:34,31,00|pw:100"
            "|ad:2|wd:0000|fa:98|t:70")
    LINE_SAID = ("info", "AFC bambu BambuAMS_1: drying -- chamber 44.0C, ad 2, "
                         "humidity 34%, target 55C")

    @pytest.fixture
    def unit_bridge(self, monkeypatch: pytest.MonkeyPatch) -> BambuBridge:
        """:return BambuBridge: a bridge speaking for BambuAMS_1"""
        return make_bambu_bridge(monkeypatch, name="BambuAMS_1")

    # ── the console ──

    def test_a_matched_line_is_rendered_in_english(self, unit_bridge):
        unit_bridge._narrate_human(self.AUTH, 100.0)
        assert unit_bridge.logger.messages == [self.AUTH_SAID]
        assert unit_bridge._last_human == self.AUTH_SAID[1]
        assert unit_bridge._last_human_t == 100.0

    def test_every_dialect_renders_the_SAME_message(self, monkeypatch):
        # One event, one sentence, whichever unit said it. Anchored on the
        # authentication: an HT says "read success" on attempts that then fail.
        for line in (self.AUTH, "[AMS_RFID]STEP:card auth success!",
                     "[AMS_RFID] STEP3,auth card successful"):
            bridge = make_bambu_bridge(monkeypatch, name="BambuAMS_1")
            bridge._narrate_human(line, 100.0)
            assert bridge.logger.messages == [self.AUTH_SAID], line

    @pytest.mark.parametrize("said, lines", [
        ("AMS: tag authenticated",
         ["[AMS_RFID] STEP3,auth card successful",
          "[AMS_RFID]STEP:card auth success!",
          "[AMS_DEV] STEP:card auth success!"]),
        # The HT misspells it.
        ("AMS finished measuring the spool",
         ["[AMS_RFID] STEP4,odom calib sucess",
          "[AMS_RFID]STEP:odom calib success",
          "[AMS_DEV] STEP:odom calib success exit 0,dis:0.989"]),
    ], ids=["tag authenticated", "calibration done"])
    def test_a_rule_fires_on_every_dialect(self, monkeypatch, said, lines):
        # Whatever is said for one unit must be said for all three (HT, AMS 2,
        # AMS 1 forms in that order).
        for line in lines:
            bridge = make_bambu_bridge(monkeypatch, name="BambuAMS_1")
            bridge._narrate_human(line, 100.0)
            assert bridge.logger.messages == [
                ("info", f"AFC bambu BambuAMS_1: {said}")], line

    def test_the_same_line_twice_is_said_once(self, unit_bridge):
        unit_bridge._narrate_human(self.AUTH, 100.0)
        unit_bridge._narrate_human(self.AUTH, 200.0)
        assert unit_bridge.logger.messages == [self.AUTH_SAID]
        assert unit_bridge._last_human_t == 100.0

    def test_a_burst_is_rate_limited_to_one_a_second(self, unit_bridge):
        flash = "[RF] tray0: info write to flash"
        unit_bridge._narrate_human(self.AUTH, 100.0)
        unit_bridge._narrate_human(flash, 100.2)
        assert unit_bridge.logger.messages == [self.AUTH_SAID]
        # Held back, not remembered: a second later the same line is said.
        unit_bridge._narrate_human(flash, 101.2)
        assert unit_bridge.logger.messages == [
            self.AUTH_SAID,
            ("info", "AFC bambu BambuAMS_1: AMS: tag for bay 1 cached in the "
                     "unit's flash (a later read returns it even after a "
                     "swap)")]

    def test_an_unmatched_line_says_nothing(self, unit_bridge):
        unit_bridge._narrate_human("[AMS_FOO] something unremarkable", 100.0)
        assert unit_bridge.logger.messages == []
        assert unit_bridge._last_human is None

    def test_the_no_tray_refusal_stays_off_the_console(self, unit_bridge):
        # A clean unload ends with the unit refusing the next command; on the
        # console that would read as a failure of a move that just worked.
        unit_bridge._narrate_human(self.NO_TRAY, 100.0)
        assert unit_bridge.logger.file_only == [self.NO_TRAY_SAID]
        assert unit_bridge.logger.messages == [("debug", self.NO_TRAY_SAID)]

    def test_the_no_tray_refusal_is_still_written_to_the_log(self, unit_bridge):
        unit_bridge._narrate_human(self.NO_TRAY, 100.0)
        unit_bridge._narrate_human(self.NO_TRAY, 100.1)
        # AFC.log takes every one: it is ahead of the dedupe and the floor.
        assert unit_bridge.logger.messages == [("debug", self.NO_TRAY_SAID)] * 2
        assert unit_bridge.logger.file_only == [self.NO_TRAY_SAID] * 2

    def test_a_log_only_line_does_not_spend_the_console_budget(self, unit_bridge):
        # A line kept off the console spends neither the rate limit nor the
        # dedupe, or it would silence the real message after it.
        unit_bridge._narrate_human(self.NO_TRAY, 100.0)
        unit_bridge._narrate_human(self.AUTH, 100.2)
        assert unit_bridge.logger.messages == [
            ("debug", self.NO_TRAY_SAID), self.AUTH_SAID]
        assert unit_bridge._last_human_t == 100.2

    def test_a_shell_open_err_is_the_lid_and_not_a_refusal(self, unit_bridge):
        # Watched live: this landed six seconds into a cycle that kept heating;
        # the generic refusal text sent the operator to unload an unrelated lane.
        unit_bridge._narrate_human(
            "j [AMS_CHMB]finish! [AMS_CHMB]set state CTC_STATE_HEATING, from "
            "selfcheck [AMS_CHMB]err, ams-ht shell open!", 100.0)
        assert unit_bridge.logger.messages == [
            ("info", "AFC bambu BambuAMS_1: AMS HT lid is open -- drying "
                     "continues, but the chamber cannot hold temperature until "
                     "the shell is closed.")]

    def test_other_err_lines_still_render_as_refusals(self, unit_bridge):
        unit_bridge._narrate_human("[AMS_CHMB]err, filament hub load!", 100.0)
        assert unit_bridge.logger.messages == [
            ("info", "AFC bambu BambuAMS_1: AMS refused the drying command: "
                     "filament hub load!. An AMS will not dry with filament out "
                     "in the hub -- reel the lane back to its bay first "
                     "(LANE_UNLOAD).")]

    # ── chamber telemetry ──

    def test_chamber_telemetry_updates_the_units_record(self, unit_bridge):
        unit_bridge._narrate_human(
            "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:22.0", 100.0, 0x1800, 2)
        assert unit_bridge._chmb_by_unit == {
            2: {"temp": 23.1, "seen": 100.0, "state": 2, "target": 55.0}}
        assert unit_bridge._last_chmb_t == 100.0
        assert unit_bridge.logger.messages == [
            ("info", "AFC bambu BambuAMS_1: drying -- chamber 23.1C, target 55C")]

    def test_humidity_comes_from_ht_not_the_suffix_on_vt(self, unit_bridge):
        # A real AMS HT line. The ",00" after vt never moves, so it is not the
        # humidity; ht carries the real figure.
        unit_bridge._narrate_human(
            "[AMS_CHMB]s:2|rf:55|vt:22.4,00|ap:22.3|ht:60,22|pw:000|ad:2|t:7",
            100.0, 0x1800, 2)
        assert unit_bridge._chmb_by_unit == {
            2: {"temp": 22.4, "seen": 100.0, "state": 2, "target": 55.0,
                "humidity": 60, "ad_n": 2}}
        assert unit_bridge.logger.messages == [
            ("info", "AFC bambu BambuAMS_1: drying -- chamber 22.4C, ad 2, "
                     "humidity 60%, target 55C")]

    def test_humidity_tracks_the_chamber_over_a_cycle(self, unit_bridge):
        # From a real cycle: ht's first value falls as the chamber heats, as
        # relative humidity does when air warms.
        seen: List[int] = []
        for t, vt, ht in ((7, 22.4, 60), (67, 33.2, 55),
                          (127, 55.6, 40), (178, 52.7, 31)):
            unit_bridge._narrate_human(
                f"[AMS_CHMB]s:2|rf:55|vt:{vt},00|ap:30.0|ht:{ht},22|t:{t}",
                100.0 + t, 0x1800, 2)
            seen.append(unit_bridge._chmb_by_unit[2]["humidity"])
        assert seen == [60, 55, 40, 31]
        # The console line is once a minute: the fourth is 51 s after the third.
        assert unit_bridge.logger.messages == [
            ("info", "AFC bambu BambuAMS_1: drying -- chamber 22.4C, humidity "
                     "60%, target 55C"),
            ("info", "AFC bambu BambuAMS_1: drying -- chamber 33.2C, humidity "
                     "55%, target 55C"),
            ("info", "AFC bambu BambuAMS_1: drying -- chamber 55.6C, humidity "
                     "40%, target 55C")]

    def test_the_hts_spelling_is_read_too(self, unit_bridge):
        # Same field under a firmware that appends a third value.
        unit_bridge._narrate_human(
            "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:23.0, hts:46,23,0 pw:100",
            100.0, 0x1800, 2)
        assert unit_bridge._chmb_by_unit == {
            2: {"temp": 23.1, "seen": 100.0, "state": 2, "target": 55.0,
                "humidity": 46}}
        assert unit_bridge.logger.messages == [
            ("info", "AFC bambu BambuAMS_1: drying -- chamber 23.1C, humidity "
                     "46%, target 55C")]

    def test_a_line_without_humidity_records_none(self, unit_bridge):
        # Not every model reports it; absent must stay absent rather than 0,
        # which would read as "bone dry".
        unit_bridge._narrate_human(
            "[AMS_CHMB]s:2|rf:55,0|vt:44.0|ap:35.3|pw:100|ad:2", 100.0,
            0x1800, 2)
        assert unit_bridge._chmb_by_unit == {
            2: {"temp": 44.0, "seen": 100.0, "state": 2, "target": 55.0,
                "ad_n": 2}}
        assert unit_bridge.logger.messages == [
            ("info", "AFC bambu BambuAMS_1: drying -- chamber 44.0C, ad 2, "
                     "target 55C")]

    def test_unparseable_chamber_numbers_leave_the_record_alone(self, unit_bridge):
        unit_bridge._narrate_human("[AMS_CHMB]s:x, rf:y|vt:z", 100.0, 0x1800, 2)
        assert unit_bridge._chmb_by_unit == {}
        assert unit_bridge._last_chmb_t == 0.0
        assert unit_bridge.logger.messages == []

    def test_chamber_numbers_that_match_but_will_not_parse(self, unit_bridge):
        # The pattern captures [0-9.]+, so "1.2.3" matches and float() still
        # fails, on the reader thread. The record already held is kept.
        held = {"temp": 42.0, "seen": 50.0, "state": 2, "target": 55.0}
        unit_bridge._chmb_by_unit[2] = dict(held)
        unit_bridge._narrate_human(
            "[AMS_CHMB]s:2, rf:55, cd:55, vt:1.2.3, ap:22.0", 100.0, 0x1800, 2)
        assert unit_bridge._chmb_by_unit == {2: held}
        # The console line quotes the text as it came.
        assert unit_bridge.logger.messages == [
            ("info", "AFC bambu BambuAMS_1: drying -- chamber 1.2.3C, target "
                     "55C")]

    def test_unit_keyed_record_is_stored(self, unit_bridge):
        unit_bridge._narrate_human(self.LINE, 100.0, 0x1800, 2)
        assert unit_bridge._chmb_by_unit == {
            2: {"temp": 44.0, "seen": 100.0, "state": 2, "target": 55.0,
                "humidity": 34, "ad_n": 2}}
        assert unit_bridge.logger.messages == [self.LINE_SAID]

    def test_two_units_do_not_overwrite_each_other(self, unit_bridge):
        other = self.LINE.replace("vt:44.0", "vt:22.5").replace("rf:55", "rf:65")
        unit_bridge._narrate_human(self.LINE, 100.0, 0x1800, 2)
        unit_bridge._narrate_human(other, 101.0, 0x0700, 0)
        assert unit_bridge._chmb_by_unit == {
            2: {"temp": 44.0, "seen": 100.0, "state": 2, "target": 55.0,
                "humidity": 34, "ad_n": 2},
            0: {"temp": 22.5, "seen": 101.0, "state": 2, "target": 65.0,
                "humidity": 34, "ad_n": 2}}
        # One drying line a minute, whichever unit is speaking.
        assert unit_bridge.logger.messages == [self.LINE_SAID]

    def test_an_unattributed_line_stores_nothing(self, unit_bridge):
        # Unit -1 (None here) is an ambiguous stray; guessing an owner would put
        # one unit's chamber on another's card.
        unit_bridge._narrate_human(self.LINE, 100.0, 0x1800, None)
        assert unit_bridge._chmb_by_unit == {}
        assert unit_bridge.logger.messages == [self.LINE_SAID]


class TestBambuBridgeRfidStamp:
    def test_bridge_wide_when_no_address_given(self, bambu_bridge):
        assert bambu_bridge._rfid_stamp(10.0, {0x0700: 20.0}, None) == 10.0
        assert bambu_bridge.logger.messages == []

    def test_a_devices_own_stamp_wins(self, bambu_bridge):
        assert bambu_bridge._rfid_stamp(10.0, {0x0700: 20.0}, 0x0700) == 20.0
        assert bambu_bridge.logger.messages == []

    def test_a_silent_device_on_an_attributing_bridge_is_none(self, bambu_bridge):
        # Another unit's chatter must not be credited to this one.
        assert bambu_bridge._rfid_stamp(10.0, {0x1800: 20.0}, 0x0700) is None
        assert bambu_bridge.logger.messages == []

    def test_no_attribution_at_all_falls_back_to_wide(self, bambu_bridge):
        # A firmware predating per-device stamps must not read as "never".
        assert bambu_bridge._rfid_stamp(10.0, {}, 0x0700) == 10.0
        assert bambu_bridge._rfid_stamp(None, {}, 0x0700) is None
        assert bambu_bridge.logger.messages == []


class TestBambuBridgeRfidReadSucceededSince:
    """
    A successful tag read must be recognisable from every unit type, and
    credited to the unit that said it: it decides whether a bay's record is
    the new spool's.
    """

    def test_bridge_credits_a_read_to_the_device_that_said_it(self, bambu_bridge):
        # An AMS 1 (0x0700) read during an HT (0x1800) scan must not count as
        # the HT's: an AMS 1 insert once landed 3 s into an HT scan window.
        line = "[AMS_DEV] STEP:read success,valid"
        t0 = bambu_bridge.reactor.monotonic()
        bambu_bridge.reactor.advance(1.0)
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.rfid_read_succeeded_since(t0, addr=0x0700) is True
        assert bambu_bridge.rfid_read_succeeded_since(t0, addr=0x1800) is False
        # Bridge-wide (no addr) keeps its old, deliberately loose behaviour.
        assert bambu_bridge.rfid_read_succeeded_since(t0) is True
        # Answered by time: a window opened after the read, or none, is False.
        assert bambu_bridge.rfid_read_succeeded_since(t0 + 1.5, addr=0x0700) is False
        assert bambu_bridge.rfid_read_succeeded_since(None, addr=0x0700) is False
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_ht_read_is_credited_to_the_ht(self, bambu_bridge):
        line = "[AMS_RFID] STEP3,save to flash ,card info valid"
        t0 = bambu_bridge.reactor.monotonic()
        feed_amsdbg(bambu_bridge, line, addr=0x1800)
        assert bambu_bridge.rfid_read_succeeded_since(t0, addr=0x1800) is True
        assert bambu_bridge.rfid_read_succeeded_since(t0, addr=0x0700) is False
        assert bambu_bridge.logger.messages == [ams_echo(line)]

    def test_unattributed_narration_still_answers_for_any_address(
            self, bambu_bridge):
        # Firmware that reports no address must not read as "this unit has
        # gone silent", the mistake a dead counter once caused.
        line = "[AMS_DEV] STEP:read success,valid"
        t0 = bambu_bridge.reactor.monotonic()
        feed_amsdbg(bambu_bridge, line)
        assert bambu_bridge.rfid_read_succeeded_since(t0, addr=0x1800) is True
        assert bambu_bridge._rfid_ok_by_addr == {}
        assert bambu_bridge.logger.messages == [ams_echo(line)]


class TestBambuBridgeRfidForeignTagSince:
    """
    A chip whose keys are not Bambu's answers anticollision, so its UID is
    readable, and then fails authentication. That is a third-party spool, not
    an empty bay.
    """

    def test_it_is_credited_to_the_unit_that_said_it(self, bambu_bridge):
        line = "[AMS_RFID]STEP:auth fail:-4"
        t0 = bambu_bridge.reactor.monotonic()
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.rfid_foreign_tag_since(t0, addr=0x0700) is True
        assert bambu_bridge.rfid_foreign_tag_since(t0, addr=0x1800) is False
        assert bambu_bridge.rfid_foreign_tag_since(None, addr=0x0700) is False
        # A window opened after the refusal does not see it.
        assert bambu_bridge.rfid_foreign_tag_since(t0 + 1.0, addr=0x0700) is False
        assert bambu_bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS could not authenticate the tag -- "
                     "third-party spool, or the tag is unreadable"),
            ams_echo(line)]

    def test_the_hts_refusal_is_credited_to_the_ht(self, bambu_bridge):
        # Verbatim from printer 1's HT (0x1800, u4): a chip that answered
        # anticollision and was refused, which the colon-only pattern missed.
        line = ("[AMS_RFID] STEP3,select card successful, stop "
                "[AMS_PMSM]mode:2->0 [AMS_RFID] STEP3,auth fail -4")
        t0 = bambu_bridge.reactor.monotonic()
        feed_amsdbg(bambu_bridge, line, addr=0x1800)
        assert bambu_bridge.rfid_foreign_tag_since(t0, addr=0x1800) is True
        assert bambu_bridge.rfid_foreign_tag_since(t0, addr=0x0700) is False
        assert bambu_bridge.logger.messages == [ams_echo(line)]


class TestBambuBridgeGaveUpSince:
    def test_it_is_stamped_per_device_and_answered_by_time(self, bambu_bridge):
        # "AMS_CTRL_state_switch finish, fail" says the unit's own retry budget
        # is spent: the one case where asking again cannot help.
        line = "[AMS_SWITCH]AMS_CTRL_state_switch finish, fail, retry:5"
        now = bambu_bridge.reactor.monotonic()
        assert bambu_bridge.gave_up_since(now, addr=0x0700) is False
        feed_amsdbg(bambu_bridge, line, addr=0x0700)
        assert bambu_bridge.gave_up_since(0.0, addr=0x0700) is True
        assert bambu_bridge.gave_up_since(0.0, addr=0x1800) is False
        assert bambu_bridge.gave_up_since(None, addr=0x0700) is False
        # A feed attempt started after the announcement must not inherit it,
        # or the re-home retry would abort at once.
        assert bambu_bridge.gave_up_since(now + 1.0, addr=0x0700) is False
        assert bambu_bridge.logger.messages == [
            ("info", f"AFC bambu bridge: the AMS reported it has STOPPED "
                     f"retrying ({line})"),
            ams_echo(line)]
        assert bambu_bridge.logger.file_only == [ams_echo(line)[1]]


class TestBambuBridgeTraySwitches:
    """
    The bay switch narration an AMS 2 gives when a spool runs out: the inlet
    clears while the outlet still holds the tail (printer 1, 2026-09-30).
    """

    LINE = ("j [AMS_TRAY]tray[2] sw_sta update, 3 -> 2, u_in_out:2459,3278 "
            "[AMS_COMMON]state:4,tray_now:255,tray_exit:14")

    def test_the_new_state_is_recorded_per_unit_and_bay(self, bambu_bridge):
        assert bambu_bridge.tray_switches(0x0700, 0, 2) is None
        bambu_bridge.reactor.advance(5.0)
        feed_amsdbg(bambu_bridge, self.LINE, addr=0x0700, unit=0)
        assert bambu_bridge.tray_switches(0x0700, 0, 2) == (2, 105.0)
        assert bambu_bridge.tray_switches(0x0700, 1, 2) is None
        assert bambu_bridge.tray_switches(0x0700, 0, 1) is None
        assert bambu_bridge.tray_switches(None, 0, 2) is None
        assert bambu_bridge.tray_switches(0x0700, None, 2) is None
        assert bambu_bridge.logger.messages == [ams_echo(self.LINE)]

    def test_a_line_with_no_unit_is_not_recorded(self, bambu_bridge):
        # Two units of one model share an address, so the bay could be either.
        feed_amsdbg(bambu_bridge, self.LINE, addr=0x0700)
        assert bambu_bridge.tray_switches(0x0700, 0, 2) is None
        assert bambu_bridge._tray_sw == {}
        assert bambu_bridge.logger.messages == [ams_echo(self.LINE)]


class TestBambuBridgeHandleLine:
    # The bridge firmware's own diagnostics, as they appear on the narration
    # channel.
    HT_MEAS = ("[HT-MEAS] fire0 capu1 tun1 dst1 act1 arm0 htm0000 on0 off306 "
               "pres0")
    BB_GATE = "[BB-GATE] u4 iv150 ht1 flw1 iv<1 m0010 f0000"
    CAP_OPEN = "[CAP] open u1 s2"
    # A line the AMS repeats, and the 10 s heartbeat it bundles into frames.
    SLOT_POLL = "[AMS_LINK]get_slot ams1 tray0 basic"
    HEARTBEAT = " [DBG] ams time: now=42044054ms diff=10005ms"
    # The console's line when a capture starts.
    SNIFF_ON: LogLine = ("info", "AFC bambu bridge: bridge sniff mode ON -- "
                                 "listen-only, this bridge is NOT driving "
                                 "the bus")

    @staticmethod
    def _say(bridge: BambuBridge, text: str, addr: Optional[int] = None,
             unit: Optional[int] = None) -> None:
        """
        Hand the bridge one narration frame.

        :param bridge: the bridge
        :param text: the AMS's words
        :param addr: the device address the frame names, if any
        :param unit: the chain index the frame names, if any
        """
        frame: Dict[str, Any] = {"evt": "amsdbg", "text": text}
        if addr is not None:
            frame["addr"] = addr
        if unit is not None:
            frame["unit"] = unit
        bridge.handle_line(json.dumps(frame))

    @pytest.fixture
    def narration_file(self, tmp_path: Path
                       ) -> Iterator[Callable[[], List[str]]]:
        """
        The real narration file, AFC_BambuAMS.log under ``tmp_path``.

        Its logger is process-global, so its handlers are closed and removed
        before and after the test.

        :return Callable: reads the file's lines, each without its timestamp
        """
        lg = logging.getLogger("AFC_BambuAMS_file")

        def _clear() -> None:
            """Close and detach every handler on the narration logger."""
            for handler in list(lg.handlers):
                handler.close()
                lg.removeHandler(handler)

        def _lines() -> List[str]:
            """:return List[str]: the file's lines, timestamps dropped"""
            for handler in lg.handlers:
                handler.flush()
            path = tmp_path / "AFC_BambuAMS.log"
            text = path.read_text() if path.exists() else ""
            return [line.split(" ", 1)[1] for line in text.splitlines()]

        _clear()
        yield _lines
        _clear()

    # ── event dispatch ───────────────────────────────────────────────────

    def test_an_ams1_calibration_closes_the_measurement_not_the_scan(
            self, monkeypatch):
        # The AMS 1 calibrates on the insert edge, about 32 s before its tag
        # read; only "STEP7:finish,cali tray" ends its scan.
        bridge = make_bambu_bridge(monkeypatch)
        detected = "[AMS_DEV] STEP,first detected"
        calib = "[AMS_DEV] STEP:odom calib success exit 0,dis:0.773"
        finish = "[AMS_DEV] STEP:feed with rfid success / STEP7:finish,cali tray"
        feed_amsdbg(bridge, detected, addr=0x0700)
        feed_amsdbg(bridge, calib, addr=0x0700)
        assert bridge._meas_live == {0x0700: False}
        assert (bridge._rfid_end_t, bridge._rfid_end_by_addr) == (None, {})
        assert bridge.rfid_cycle_ended_since(0.0, addr=0x0700) is False
        bridge.reactor.now = 142.4
        feed_amsdbg(bridge, finish, addr=0x0700)
        assert (bridge._rfid_end_t, bridge._rfid_end_by_addr) == (
            142.4, {0x0700: 142.4})
        assert bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS: spool detected"),
            ams_echo(detected), ams_echo(calib), ams_echo(finish)]

    def test_unknown_event_is_surfaced_file_only(self, monkeypatch):
        # Silence would make "the command never landed" and "the reply never
        # came" look the same, so an event nothing consumes is logged.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"nonsense","x":1}')
        line = "AFC bambu: unhandled bridge event {'evt': 'nonsense', 'x': 1}"
        assert bridge.logger.messages == [("debug", line)]
        assert bridge.logger.file_only == [line]

    def test_routine_command_echoes_are_not_surfaced(self, monkeypatch):
        # These arrive on every prep; they would be noise at startup.
        bridge = make_bambu_bridge(monkeypatch)
        for evt in ("mcaddr", "armms", "hb", "mute", "units"):
            bridge.handle_line(json.dumps({"evt": evt}))
        assert bridge.logger.messages == []

    def test_sniff_frames_are_file_only(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"sniff","hex":"3DC5"}')
        assert bridge.logger.messages == [("debug", "SNIFF 3DC5")]
        assert bridge.logger.file_only == ["SNIFF 3DC5"]
        # No sequence number, so no capture counters move.
        assert (bridge._sniff_sq, bridge._sniff_lost) == (None, 0)

    def test_sniff_frames_count_lost_blobs_and_overruns(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"sniff","hex":"AA","sq":4,"ov":10,"rf":2,'
                           '"us":7}')
        bridge.handle_line('{"evt":"sniff","hex":"BB","sq":7,"ov":13,"rf":5,'
                           '"us":9}')
        assert bridge.logger.messages == [
            ("debug", "SNIFF AA sq=4 ov=10 rf=2 us=7"),
            ("debug", "SNIFF BB sq=7 ov=13 rf=5 us=9")]
        # sq 4 -> 7 lost two blobs; ov and rf count from the first frame.
        assert (bridge._sniff_sq, bridge._sniff_lost) == (7, 2)
        assert (bridge._sniff_ovr, bridge._sniff_rf) == (3, 3)

    def test_consecutive_sniff_frames_lose_nothing(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"sniff","hex":"AA","sq":5}')
        bridge.handle_line('{"evt":"sniff","hex":"BB","sq":6}')
        assert bridge.logger.messages == [
            ("debug", "SNIFF AA sq=5 ov=None rf=None us=None"),
            ("debug", "SNIFF BB sq=6 ov=None rf=None us=None")]
        assert (bridge._sniff_sq, bridge._sniff_lost) == (6, 0)
        # No ov or rf in the frames, so neither baseline is taken.
        assert (bridge._sniff_ovr0, bridge._sniff_rf0) == (None, None)

    def test_sniff_mode_ack_is_surfaced(self, monkeypatch):
        # On the console: a listen-only bridge still answers status polls, so
        # nothing else tells it from units that have gone quiet.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"sniff_mode","on":true}')
        bridge.handle_line('{"evt":"sniff_mode","on":false}')
        assert bridge.logger.messages == [
            ("info", "AFC bambu bridge: bridge sniff mode ON -- listen-only, "
                     "this bridge is NOT driving the bus"),
            ("info", "AFC bambu bridge: bridge sniff mode OFF -- bus master "
                     "again -- capture LOSSLESS")]

    def test_sniff_mode_off_reports_an_incomplete_capture(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"sniff_mode","on":true}')
        bridge.handle_line('{"evt":"sniff","hex":"AA","sq":1,"ov":0,"rf":0}')
        bridge.handle_line('{"evt":"sniff","hex":"BB","sq":3,"ov":1,"rf":0}')
        bridge.handle_line('{"evt":"sniff_mode","on":false}')
        # Turning it on again starts a new capture's counters.
        bridge.handle_line('{"evt":"sniff_mode","on":true}')
        assert (bridge._sniff_sq, bridge._sniff_lost, bridge._sniff_ovr,
                bridge._sniff_rf) == (None, 0, 0, 0)
        assert bridge.logger.messages == [
            self.SNIFF_ON,
            ("debug", "SNIFF AA sq=1 ov=0 rf=0 us=None"),
            ("debug", "SNIFF BB sq=3 ov=1 rf=0 us=None"),
            ("info", "AFC bambu bridge: bridge sniff mode OFF -- bus master "
                     "again -- capture INCOMPLETE: 1 blob(s) lost, 1 UART "
                     "overrun(s), 0 ring lap(s). Usable for protocol work, "
                     "NOT for reconstructing a firmware image"),
            self.SNIFF_ON]

    def test_lost_blobs_alone_make_the_capture_incomplete(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"sniff_mode","on":true}')
        bridge.handle_line('{"evt":"sniff","hex":"AA","sq":1,"ov":4,"rf":2}')
        bridge.handle_line('{"evt":"sniff","hex":"BB","sq":3,"ov":4,"rf":2}')
        bridge.handle_line('{"evt":"sniff_mode","on":false}')
        assert (bridge._sniff_lost, bridge._sniff_ovr, bridge._sniff_rf) == (
            1, 0, 0)
        assert bridge.logger.messages == [
            self.SNIFF_ON,
            ("debug", "SNIFF AA sq=1 ov=4 rf=2 us=None"),
            ("debug", "SNIFF BB sq=3 ov=4 rf=2 us=None"),
            ("info", "AFC bambu bridge: bridge sniff mode OFF -- bus master "
                     "again -- capture INCOMPLETE: 1 blob(s) lost, 0 UART "
                     "overrun(s), 0 ring lap(s). Usable for protocol work, "
                     "NOT for reconstructing a firmware image")]

    def test_overruns_alone_make_the_capture_incomplete(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"sniff_mode","on":true}')
        bridge.handle_line('{"evt":"sniff","hex":"AA","sq":1,"ov":0,"rf":2}')
        bridge.handle_line('{"evt":"sniff","hex":"BB","sq":2,"ov":2,"rf":2}')
        bridge.handle_line('{"evt":"sniff_mode","on":false}')
        assert (bridge._sniff_lost, bridge._sniff_ovr, bridge._sniff_rf) == (
            0, 2, 0)
        assert bridge.logger.messages == [
            self.SNIFF_ON,
            ("debug", "SNIFF AA sq=1 ov=0 rf=2 us=None"),
            ("debug", "SNIFF BB sq=2 ov=2 rf=2 us=None"),
            ("info", "AFC bambu bridge: bridge sniff mode OFF -- bus master "
                     "again -- capture INCOMPLETE: 0 blob(s) lost, 2 UART "
                     "overrun(s), 0 ring lap(s). Usable for protocol work, "
                     "NOT for reconstructing a firmware image")]

    def test_ring_laps_alone_make_the_capture_incomplete(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"sniff_mode","on":true}')
        bridge.handle_line('{"evt":"sniff","hex":"AA","sq":1,"ov":4,"rf":0}')
        bridge.handle_line('{"evt":"sniff","hex":"BB","sq":2,"ov":4,"rf":1}')
        bridge.handle_line('{"evt":"sniff_mode","on":false}')
        assert (bridge._sniff_lost, bridge._sniff_ovr, bridge._sniff_rf) == (
            0, 0, 1)
        assert bridge.logger.messages == [
            self.SNIFF_ON,
            ("debug", "SNIFF AA sq=1 ov=4 rf=0 us=None"),
            ("debug", "SNIFF BB sq=2 ov=4 rf=1 us=None"),
            ("info", "AFC bambu bridge: bridge sniff mode OFF -- bus master "
                     "again -- capture INCOMPLETE: 0 blob(s) lost, 0 UART "
                     "overrun(s), 1 ring lap(s). Usable for protocol work, "
                     "NOT for reconstructing a firmware image")]

    def test_txecho_reports_drops_and_tolerates_a_bad_count(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"txecho","drops":0}')
        assert bridge._txecho_drops == 0
        assert bridge.logger.messages == []
        bridge.handle_line('{"evt":"txecho","drops":2}')
        assert bridge._txecho_drops == 2
        bridge.handle_line('{"evt":"txecho","drops":"x"}')
        assert bridge._txecho_drops == 0
        assert bridge.logger.messages == [
            ("info", "AFC bambu bridge: TX echo dropped 2 frame(s) -- the USB "
                     "link fell behind the bus. Frames MISSING from this "
                     "capture were still transmitted; do not read a gap as a "
                     "frame we never sent.")]

    def test_error_event_warns(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"error","msg":"bus down"}')
        assert bridge.logger.messages == [
            ("warning", "AFC bambu: bridge error: bus down")]

    def test_ack_is_logged(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"ack","cmd":"dry","slot":55}')
        assert bridge.logger.messages == [
            ("debug", "AFC bambu: bridge ack dry (slot 55)")]
        # Not a routine ack, so it may reach the console.
        assert bridge.logger.file_only == []

    def test_a_routine_ack_is_file_only(self, monkeypatch):
        # The follower's cadence acks repeat for the length of a print.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"ack","cmd":"assist","slot":1}')
        line = "AFC bambu: bridge ack assist (slot 1)"
        assert bridge.logger.messages == [("debug", line)]
        assert bridge.logger.file_only == [line]

    def test_a_slow_pass_is_logged_to_file_only(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"slow","ms":1460,"rr_ms":1402,"rr_n":31,'
                           '"rr_max_us":32000,"capped":30,"capscan":0}')
        line = ("AFC bambu: bridge loop held 1460 ms (bus reads 1402 ms over "
                "31, longest 32000 us, capped 30, capscans 0)")
        assert bridge.logger.messages == [("debug", line)]
        assert bridge.logger.file_only == [line]

    def test_reply_is_cached_for_the_probe(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"reply","hex":"3D05AA"}')
        assert bridge._last_raw_reply == "3D05AA"
        bridge.handle_line('{"evt":"raw","rx":"3D0700"}')
        assert bridge._last_raw_reply == "3D0700"
        assert bridge.logger.messages == []

    def test_garbage_line_is_ignored(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line("not json at all")
        bridge.handle_line("")
        assert bridge.logger.messages == []
        assert bridge.latest_status() is None

    # ── the chain reply ──────────────────────────────────────────────────

    def test_capmask_is_the_measure_readback(self, monkeypatch):
        # The capen ack names only the unit, so this is the only way to see
        # the value the firmware kept.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","capmask":18}')
        assert bridge._chain_capmask == 18
        assert bridge.logger.messages == []

    def test_no_capmask_is_unknown_not_zero(self, monkeypatch):
        # "No unit measures" and "this build cannot tell" must differ.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","htmask":5}')
        assert bridge._chain_capmask is None
        assert bridge.logger.messages == []

    def test_a_malformed_capmask_reads_unknown(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","capmask":18}')
        bridge.handle_line('{"evt":"chain","uids":"AA","capmask":"x"}')
        assert bridge._chain_capmask is None
        assert bridge.logger.messages == []

    def test_an_all_off_capmask_is_a_real_answer(self, monkeypatch):
        # Zero is the default, so it has to be told apart from absent.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","capmask":0}')
        assert bridge._chain_capmask == 0
        assert bridge.logger.messages == []

    def test_the_capacity_window_verdict_rides_along(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","capn":3,"capdiag":9,'
                           '"capwhy":2,"capwhyu":1,"capwhys":3,"tags":"3A"}')
        assert (bridge._chain_capn, bridge._chain_capdiag,
                bridge._chain_capwhy, bridge._chain_capwhy_unit,
                bridge._chain_capwhy_slot) == (3, 9, 2, 1, 3)
        assert bridge._chain_tags == "3A"
        # Absent is "not said", kept apart from 0.
        bridge.handle_line('{"evt":"chain","uids":"AA"}')
        assert (bridge._chain_capn, bridge._chain_capwhy,
                bridge._chain_capwhy_unit,
                bridge._chain_capwhy_slot) == (0, None, None, None)
        # A malformed field resets the whole group.
        bridge.handle_line('{"evt":"chain","uids":"AA","capn":"x",'
                           '"capwhy":2}')
        assert (bridge._chain_capn, bridge._chain_capdiag,
                bridge._chain_capwhy) == (0, 0, None)
        assert bridge.logger.messages == []

    def test_a_reply_is_stored_under_the_same_lock(self, monkeypatch):
        # The handler takes the bridge's lock to store a reply, so a snapshot
        # holding it can never see a store in progress.
        bridge = make_bambu_bridge(monkeypatch)
        line = '{"evt":"chain","uids":"CC","a2mask":1,"a2asks":"4"}'
        with bridge._lock:
            t = threading.Thread(target=bridge.handle_line, args=(line,))
            t.start()
            t.join(0.1)
            assert t.is_alive()                   # blocked on the lock
            assert (bridge._chain_seq, bridge._chain_uids) == (0, [])
        t.join(5)
        assert bridge.chain_snapshot() == {"seq": 1, "uids": ["CC"],
                                           "htmask": 0, "a2mask": 1,
                                           "a2asks": [4]}
        assert bridge.logger.messages == []

    # ── narration routing ────────────────────────────────────────────────

    def test_pure_chatter_is_file_only(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, name="u")
        self._say(bridge, "[AMS_CALL] ams0 select,select ams1")
        line = "AMS: [AMS_CALL] ams0 select,select ams1"
        assert bridge.logger.messages == [("debug", line)]
        assert bridge.logger.file_only == [line]

    def test_narration_with_content_is_not_file_only(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, name="u")
        self._say(bridge, "[AMS_SWITCH]feed finish -1, stall")
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_SWITCH]feed finish -1, stall")]
        assert bridge.logger.file_only == []

    def test_repeated_lines_are_deduped_then_re_emitted_with_a_count(
            self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, name="u")
        self._say(bridge, "[AMS_TRAY]tray[0] sw_sta update")
        self._say(bridge, "[AMS_TRAY]tray[0] sw_sta update")
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_TRAY]tray[0] sw_sta update")]
        bridge.reactor.advance(61.0)
        self._say(bridge, "[AMS_TRAY]tray[0] sw_sta update")
        repeat = "AMS: (x3 repeated) [AMS_TRAY]tray[0] sw_sta update"
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_TRAY]tray[0] sw_sta update"),
            ("debug", repeat)]
        assert bridge.logger.file_only == [repeat]
        assert (bridge._last_dbg_n, bridge._last_dbg_t) == (3, 161.0)

    def test_motor_current_that_matches_but_will_not_parse(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, name="u")
        bridge._bldc_i = 0.25                     # an earlier good reading
        self._say(bridge, "[AMS_SWITCH]feed bldc_i:1.2.3A")
        assert bridge.last_fault()[2] == 0.25
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_SWITCH]feed bldc_i:1.2.3A")]

    def test_a_valid_motor_current_is_cached(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, name="u")
        self._say(bridge, "[AMS_SWITCH]feed bldc_i:0.319A")
        assert bridge.last_fault()[2] == pytest.approx(0.319)
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_SWITCH]feed bldc_i:0.319A")]

    def test_the_ten_second_heartbeat_line_is_dropped_entirely(
            self, monkeypatch):
        # Its timestamp defeats the dedupe, so it would log six lines a minute.
        bridge = make_bambu_bridge(monkeypatch, name="u")
        self._say(bridge, "[DBG] ams time 12345")
        assert bridge.logger.messages == []
        assert bridge._last_dbg is None

    def test_a_throwing_narrator_does_not_break_the_reader(self, monkeypatch):
        class _ConsoleDownLogger(BambuLogger):
            """AFC's logger with the console line failing."""

            def __init__(self) -> None:
                """Record each console line that was refused."""
                super().__init__()
                self.refused: List[str] = []

            def info(self, message: str, console_only: bool = False) -> None:
                """:raises RuntimeError: always, after recording the line"""
                self.refused.append(message)
                error_str = "console gone"
                raise RuntimeError(error_str)

        logger = _ConsoleDownLogger()
        bridge = make_bambu_bridge(monkeypatch, logger=logger, name="u")
        # A stall the narrator puts into words, so its console line raises.
        text = ("[AMS_SWITCH]feed finish -1, stall, len_det:1.620 m, "
                "tube_len:3.506 m")
        self._say(bridge, text)
        # The narrator did try the console, and that raised.
        assert logger.refused == [
            "AFC bambu u: AMS: the filament STALLED after 1.62 m of a 3.51 m "
            "path -- check for a jam between the bay and the toolhead"]
        # The stall is still recorded, and the line still reaches AFC.log.
        assert bridge.last_fault() == (1, text, 0.0)
        assert logger.messages == [("debug", f"AMS: {text}")]

    # ── the narration file ───────────────────────────────────────────────

    def test_the_address_is_recorded_for_attribution(
            self, monkeypatch, tmp_path, narration_file):
        bridge = make_bambu_bridge(monkeypatch)
        assert bridge.set_narration_log(str(tmp_path)) is True
        self._say(bridge, "[AMS_SWITCH]pull finish 0", addr=0x1800)
        assert narration_file() == ["0x1800 u? [AMS_SWITCH]pull finish 0"]
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_SWITCH]pull finish 0")]

    def test_repeats_are_kept_verbatim(self, monkeypatch, tmp_path,
                                       narration_file):
        # The console dedupes; the file must not. A line repeating hundreds of
        # times is how a stuck loop looks.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.set_narration_log(str(tmp_path))
        for _ in range(5):
            self._say(bridge, "[AMS_IDLE]set ams state assist, mode:4",
                      addr=0x700, unit=1)
        assert narration_file() == [
            "0x0700 u1 [AMS_IDLE]set ams state assist, mode:4"] * 5
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_IDLE]set ams state assist, mode:4")]

    def test_narration_without_a_log_is_a_safe_noop(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, "[AMS_SWITCH]feed finish 0")
        assert bridge._nar_lg is None
        # The rest of the frame was still handled.
        assert bridge.last_finish() == (1, True, "[AMS_SWITCH]feed finish 0")
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_SWITCH]feed finish 0")]

    # ── the mcaddr echo ──────────────────────────────────────────────────

    def test_a_malformed_echo_does_not_take_the_reader_down(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"mcaddr","unit":"x","addr":"y"}')
        assert bridge.mcaddr_ack(0) is None
        assert bridge._mcaddr_ack == {}
        assert bridge.logger.messages == []

    def test_it_is_still_a_known_event_and_not_logged_as_unhandled(
            self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"mcaddr","unit":0,"addr":6144}')
        assert bridge.mcaddr_ack(0) == 6144
        assert bridge.logger.messages == []

    # ── status frames and the fstate trace ───────────────────────────────

    def test_the_first_frame_is_recorded(self, monkeypatch, tmp_path,
                                         narration_file):
        # A unit that comes up in a mode and never leaves it is the finding,
        # so the opening value is not swallowed as "no change".
        bridge = make_bambu_bridge(monkeypatch)
        bridge.set_narration_log(str(tmp_path))
        bridge.handle_line('{"evt":"status","fstate":4,"buff":59}')
        assert narration_file() == ["HOST-- fstate - -> 4 (buff=59)"]
        assert bridge.logger.messages == [REQUEST_INFO_LOG]

    def test_a_change_is_recorded_with_both_ends(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge.handle_line('{"evt":"status","fstate":0}')
        bridge.handle_line('{"evt":"status","fstate":2}')
        assert bridge._nar_lg.messages == [
            ("debug", "HOST-- fstate - -> 0 (buff=None)"),
            ("debug", "HOST-- fstate 0 -> 2 (buff=None)")]
        assert bridge._fstate_last == 2
        assert bridge.logger.messages == [REQUEST_INFO_LOG]

    def test_repeats_are_not_recorded(self, monkeypatch):
        # Several frames a second; logging each would bury the narration.
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        for _ in range(20):
            bridge.handle_line('{"evt":"status","fstate":4}')
        assert bridge._nar_lg.messages == [
            ("debug", "HOST-- fstate - -> 4 (buff=None)")]
        assert bridge.logger.messages == [REQUEST_INFO_LOG]

    def test_the_buffer_reading_rides_along(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, narration=True)
        bridge.handle_line('{"evt":"status","fstate":2,"buff":97}')
        assert bridge._nar_lg.messages == [
            ("debug", "HOST-- fstate - -> 2 (buff=97)")]
        assert bridge.logger.messages == [REQUEST_INFO_LOG]

    def test_no_narration_log_configured_is_a_noop(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"status","fstate":2}')
        assert bridge._fstate_last is bridge_mod._UNSET
        assert bridge.latest_status() == {"evt": "status", "fstate": 2}
        assert bridge.logger.messages == [REQUEST_INFO_LOG]

    def test_status_listeners_still_run(self, monkeypatch):
        # The trace sits in the status path; it must not displace it.
        seen: List[dict] = []
        bridge = make_bambu_bridge(monkeypatch, narration=True,
                                   listener=seen.append)
        bridge.handle_line('{"evt":"status","fstate":2}')
        bridge.reactor.run_callbacks()
        assert seen == [{"evt": "status", "fstate": 2}]
        assert bridge._nar_lg.messages == [
            ("debug", "HOST-- fstate - -> 2 (buff=None)")]
        assert bridge.logger.messages == [REQUEST_INFO_LOG]

    def test_status_caches_and_hops_to_reactor(self, monkeypatch):
        seen: List[dict] = []
        bridge = make_bambu_bridge(monkeypatch, listener=seen.append)
        frame = {"evt": "status", "online": True, "slots": [{"i": 0}]}
        bridge.handle_line(json.dumps(frame))
        assert bridge.latest_status() == frame     # cached synchronously
        assert seen == []                          # not before the hop
        bridge.reactor.run_callbacks()
        assert seen == [frame]                     # delivered on the reactor
        # The first frame of a connection asks the bridge who it is.
        assert bridge_sent(bridge) == [{"cmd": "info"}]
        assert bridge.logger.messages == [REQUEST_INFO_LOG]

    def test_info_is_asked_once_per_connection(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"status","online":true}')
        bridge.handle_line('{"evt":"status","online":true}')
        assert bridge_sent(bridge) == [{"cmd": "info"}]
        assert bridge.logger.messages == [REQUEST_INFO_LOG]
        assert bridge._info_asked is True

    def test_all_listeners_get_status(self, monkeypatch):
        seen: List[dict] = []
        bridge = make_bambu_bridge(monkeypatch, listener=seen.append)
        a: List[dict] = []
        b: List[dict] = []
        bridge.add_listener(a.append)
        bridge.add_listener(b.append)
        frame = {"evt": "status", "online": True, "slots": []}
        bridge.handle_line(json.dumps(frame))
        bridge.reactor.run_callbacks()
        assert (seen, a, b) == ([frame], [frame], [frame])
        assert bridge.logger.messages == [REQUEST_INFO_LOG]

    def test_error_line_logs(self, monkeypatch):
        seen: List[dict] = []
        bridge = make_bambu_bridge(monkeypatch, listener=seen.append)
        bridge.handle_line('{"evt":"error","msg":"feed refused"}')
        bridge.reactor.run_callbacks()
        assert bridge.logger.messages == [
            ("warning", "AFC bambu: bridge error: feed refused")]
        assert seen == []

    def test_junk_ignored(self, monkeypatch):
        seen: List[dict] = []
        bridge = make_bambu_bridge(monkeypatch, listener=seen.append)
        bridge.handle_line("not json")
        bridge.handle_line("")
        bridge.handle_line("[1, 2]")              # JSON, but not an object
        bridge.reactor.run_callbacks()
        assert bridge.latest_status() is None
        assert seen == []
        assert bridge.logger.messages == []

    def test_ack_logged_to_afc_log_and_not_dispatched(self, monkeypatch):
        # Acks are the record of what the bridge was asked to do: AFC.log at
        # debug, and never to the status listeners.
        seen: List[dict] = []
        bridge = make_bambu_bridge(monkeypatch, listener=seen.append)
        bridge.handle_line('{"evt":"ack","cmd":"feed","slot":0}')
        bridge.reactor.run_callbacks()
        assert seen == []
        assert bridge.logger.messages == [
            ("debug", "AFC bambu: bridge ack feed (slot 0)")]
        assert bridge.logger.file_only == []

    def test_chain_caches_uids(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line(
            '{"evt":"chain","ndisc":3,"nexp":3,"uids":'
            '"AAAA00000000000000000001,BBBB00000000000000000002,'
            '0123456789ABCDEF00003331"}')
        assert bridge.chain_uids() == [
            "AAAA00000000000000000001", "BBBB00000000000000000002",
            "0123456789ABCDEF00003331"]
        assert bridge.logger.messages == []

    def test_chain_empty_uids(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA"}')
        bridge.handle_line('{"evt":"chain","uids":""}')
        assert bridge.chain_uids() == []
        assert bridge._chain_seq == 2
        assert bridge.logger.messages == []

    # ── finish judgement ─────────────────────────────────────────────────

    def test_a_stall_line_that_consults_the_measurement_returns(
            self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, "[AMS_SWITCH]old tube_len:3619 mm", addr=0x1800)
        # Would hang, not fail, if the lock were taken twice.
        self._say(bridge, "[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m",
                  addr=0x1800)
        # 3601 mm is within 100 mm of the 3619 mm path: arrived.
        assert bridge.last_finish() == (
            1, True, "[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m")
        assert bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS 0x1800 reports its measured "
                     "filament path as 3619mm -- using it to size move "
                     "timeouts instead of the configured estimate"),
            ("debug", "AMS: [AMS_SWITCH]old tube_len:3619 mm"),
            ("debug", "AMS: [AMS_SWITCH]feed finish -1, stall, "
                      "len_det:3.601 m")]

    def test_the_lock_is_free_afterwards(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, "[AMS_SWITCH]feed finish -1, stall, len_det:1.0 m",
                  addr=0x1800)
        assert bridge._lock.locked() is False
        assert bridge.tube_len(0x1800) is None    # takes the lock again
        # No path length to judge against, so the stall stands.
        assert bridge.last_finish()[:2] == (1, False)
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_SWITCH]feed finish -1, stall, len_det:1.0 m")]

    # ── drying refusals ──────────────────────────────────────────────────

    def test_it_is_said_in_english_on_the_console(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, "[AMS_CHMB]err, filament hub load!", addr=0x1800,
                  unit=2)
        assert bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS refused the drying command: "
                     "filament hub load!. An AMS will not dry with filament "
                     "out in the hub -- reel the lane back to its bay first "
                     "(LANE_UNLOAD)."),
            ("debug", "AMS: [AMS_CHMB]err, filament hub load!")]
        assert bridge.last_dry_error(2) == "filament hub load!"

    # ── the unit giving up ───────────────────────────────────────────────

    def test_the_fail_form_is_recognised(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        text = ("[AMS_SWITCH]AMS_CTRL_state_switch finish, fail, retry:5, "
                "feed_ret:0, err_code:0x12")
        self._say(bridge, text, addr=0x1800)
        assert bridge._gave_up_by_addr == {0x1800: 100.0}
        assert bridge.logger.messages == [
            ("info", "AFC bambu bridge: the AMS reported it has STOPPED "
                     "retrying ([AMS_SWITCH]AMS_CTRL_state_switch finish, "
                     "fail, retry:5, feed_ret:0, err_code:0x12)"),
            ("debug", f"AMS: {text}")]

    def test_a_give_up_with_no_address_is_not_stamped(self, monkeypatch):
        # gave_up_since() is asked per unit, so a line naming none is no use.
        bridge = make_bambu_bridge(monkeypatch)
        text = ("[AMS_SWITCH]AMS_CTRL_state_switch finish, fail, retry:5, "
                "feed_ret:0, err_code:0x12")
        self._say(bridge, text)
        assert bridge._gave_up_by_addr == {}
        assert bridge.logger.messages == [("debug", f"AMS: {text}")]

    def test_a_long_give_up_line_is_cut_to_ninety_characters(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        text = ("  [AMS_SWITCH]AMS_CTRL_state_switch finish, fail, retry:5, "
                "feed_ret:0, err_code:0x12, bldc_i:0.080A  ")
        self._say(bridge, text, addr=0x1800)
        # Stripped first, then the first 90 characters.
        assert bridge.logger.messages == [
            ("info", "AFC bambu bridge: the AMS reported it has STOPPED "
                     "retrying ([AMS_SWITCH]AMS_CTRL_state_switch finish, "
                     "fail, retry:5, feed_ret:0, err_code:0x12, bldc_i)"),
            ("debug", f"AMS: {text}")]

    def test_the_success_form_is_not(self, monkeypatch):
        # Same sentence shape, opposite meaning, and Bambu's own spelling.
        bridge = make_bambu_bridge(monkeypatch)
        text = ("[AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, "
                "err_code:0x00")
        self._say(bridge, text, addr=0x1800)
        assert bridge._gave_up_by_addr == {}
        # It is the retract's completion instead.
        assert (bridge._switch_seq, bridge._switch_text) == (1, text)
        assert bridge.logger.messages == [("debug", f"AMS: {text}")]

    @pytest.mark.parametrize("text,logged", [
        ("[AMS_SWITCH]tray:2, bldc slip, dw_pos:-0.037 m", []),
        ("[AMS_LINK]err_code:0x00->0x16", []),
        ("[AMS_SWITCH]switch_feed rocker stall, tray_cnt:0,0,", []),
        ("[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m", []),
        ("[AMS_RFID]STEP:odom calib success exit 0,dis:0.688",
         [("info", "AFC bambu bridge: AMS finished measuring the spool")]),
    ])
    def test_ordinary_stall_chatter_is_not(self, monkeypatch, text, logged):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, text, addr=0x0700)
        assert bridge._gave_up_by_addr == {}
        assert bridge.logger.messages == logged + [("debug", f"AMS: {text}")]

    # ── firmware diagnostics on the narration channel ────────────────────

    @pytest.mark.parametrize("which", ["HT_MEAS", "BB_GATE", "CAP_OPEN"])
    def test_it_is_file_only(self, monkeypatch, which):
        text = getattr(self, which)
        bridge = make_bambu_bridge(monkeypatch, narration=True, name="u")
        self._say(bridge, text, addr=0x1800, unit=1)
        assert bridge._nar_lg.messages == [("debug", f"0x1800 u1 {text}")]
        assert bridge.logger.messages == [
            ("debug", f"AFC bambu: bridge diag {text}")]
        assert bridge.logger.file_only == [f"AFC bambu: bridge diag {text}"]

    def test_it_does_not_break_the_repeat_collapsing_around_it(
            self, monkeypatch):
        # Its counter changes every second; letting it replace the dedupe's
        # last line printed a repeating AMS line on every repeat.
        bridge = make_bambu_bridge(monkeypatch, name="u")
        line = "+ [AMS_DEV] STEP:odom card in RF,delay check"
        self._say(bridge, line, addr=0x0700, unit=1)
        self._say(bridge, self.HT_MEAS, addr=0x1800, unit=1)
        self._say(bridge, line, addr=0x0700, unit=1)
        assert bridge.logger.messages == [
            ("debug", f"AMS: {line}"),
            ("debug", f"AFC bambu: bridge diag {self.HT_MEAS}")]
        assert (bridge._last_dbg, bridge._last_dbg_n) == (line, 2)

    def test_it_is_never_evidence(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, name="u")
        before = {k: dict(v) for k, v in vars(bridge).items()
                  if isinstance(v, dict)}
        for text in (self.HT_MEAS, self.BB_GATE, self.CAP_OPEN):
            self._say(bridge, text, addr=6144, unit=1)
        after = {k: dict(v) for k, v in vars(bridge).items()
                 if isinstance(v, dict)}
        assert after == before, "a firmware diagnostic reached a parser"
        assert bridge.last_ht_cali(1) is None
        assert bridge.last_cap_measure(0x1800) is None
        assert bridge.rfid_cycle_ended_since(0.0) is False
        assert bridge._last_dbg is None
        assert bridge.logger.messages == [
            ("debug", f"AFC bambu: bridge diag {text}")
            for text in (self.HT_MEAS, self.BB_GATE, self.CAP_OPEN)]

    # ── the calibrate echo ───────────────────────────────────────────────

    def test_it_is_logged_file_only_and_not_as_unhandled(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"cali","unit":1,"slot":2}')
        line = "AFC bambu: bridge answered cali (unit 1, slot 2)"
        assert bridge.logger.messages == [("debug", line)]
        assert bridge.logger.file_only == [line]

    def test_extra_fields_ride_along_uninterpreted(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"cali","unit":1,"slot":2,"ran":0,"why":3}')
        line = "AFC bambu: bridge answered cali (unit 1, slot 2) ran=0 why=3"
        assert bridge.logger.messages == [("debug", line)]
        assert bridge.logger.file_only == [line]

    # ── narration dedupe ─────────────────────────────────────────────────

    def test_first_line_logged(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_RFID] STEP3,feed with rfid fail!")]
        assert bridge.logger.file_only == []

    def test_immediate_repeat_suppressed(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        self._say(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_RFID] STEP3,feed with rfid fail!")]
        assert bridge._last_dbg_n == 2

    def test_repeat_resurfaces_with_a_count(self, monkeypatch):
        # A loop stuck for a minute must reappear, or the fault reads as
        # silence.
        bridge = make_bambu_bridge(monkeypatch)
        for _ in range(6):
            self._say(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        bridge.reactor.advance(61.0)
        self._say(bridge, "[AMS_RFID] STEP3,feed with rfid fail!")
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_RFID] STEP3,feed with rfid fail!"),
            ("debug", "AMS: (x7 repeated) [AMS_RFID] STEP3,feed with rfid "
                      "fail!")]

    def test_a_different_line_resets_the_counter(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, "[AMS_PMSM]mode:0->2")
        self._say(bridge, "[AMS_PMSM]mode:0->2")
        self._say(bridge, "[AMS_PMSM]mode:2->0")
        assert bridge.logger.messages == [
            ("debug", "AMS: [AMS_PMSM]mode:0->2"),
            ("debug", "AMS: [AMS_PMSM]mode:2->0")]
        assert (bridge._last_dbg, bridge._last_dbg_n) == (
            "[AMS_PMSM]mode:2->0", 1)

    def test_a_heartbeat_only_frame_reaches_nobody(self, monkeypatch):
        # The drain reply opens with a stray framing byte, so a heartbeat-only
        # frame strips to "," and would print a bare comma every 10 s.
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, ", [DBG] ams time: now=49107885ms diff=10004ms",
                  addr=1792)
        assert bridge.logger.messages == []
        assert bridge._last_dbg is None

    def test_a_bundled_heartbeat_does_not_reset_the_repeat_run(
            self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, self.SLOT_POLL, addr=1792)
        # The same sentence carrying the heartbeat is a repeat, not a new run.
        self._say(bridge, self.SLOT_POLL + self.HEARTBEAT, addr=1792)
        self._say(bridge, self.SLOT_POLL, addr=1792)
        assert (bridge._last_dbg, bridge._last_dbg_n) == (self.SLOT_POLL, 3)
        assert bridge.logger.messages == [("debug", f"AMS: {self.SLOT_POLL}")]

    # ── narration reaching the console ───────────────────────────────────

    def test_the_first_adoption_is_announced_once(self, monkeypatch):
        # Worth one console line, since it resizes every move timeout, but
        # not one per load.
        bridge = make_bambu_bridge(monkeypatch)
        self._say(bridge, "[AMS_SWITCH]new tube_len:3481 mm", addr=0x0700)
        # Within the console's 1 s floor, so not narrated either.
        self._say(bridge, "[AMS_SWITCH]new tube_len:3502 mm", addr=0x0700)
        bridge.reactor.advance(5.0)
        self._say(bridge, "[AMS_SWITCH]new tube_len:3497 mm", addr=0x0700)
        assert bridge.logger.messages == [
            ("info", "AFC bambu bridge: AMS learned the bay-to-hub path "
                     "length: 3481 mm"),
            ("info", "AFC bambu bridge: AMS 0x0700 reports its measured "
                     "filament path as 3481mm -- using it to size move "
                     "timeouts instead of the configured estimate"),
            ("debug", "AMS: [AMS_SWITCH]new tube_len:3481 mm"),
            ("debug", "AMS: [AMS_SWITCH]new tube_len:3502 mm"),
            ("info", "AFC bambu bridge: AMS learned the bay-to-hub path "
                     "length: 3497 mm"),
            ("debug", "AMS: [AMS_SWITCH]new tube_len:3497 mm")]
        assert bridge.tube_len(0x0700) == 3497.0

    def test_a_matching_line_actually_reaches_the_console(self, monkeypatch):
        # No name given: the bridge speaks as "bridge", its own default.
        reactor = FakeReactor()
        patch_module_time(monkeypatch, reactor)
        logger = BambuLogger()
        bridge = BambuBridge(FakeSerial, reactor, logger)
        self._say(bridge, "[AMS_CHMB]set state CTC_STATE_HEATING")
        assert logger.messages == [
            ("info", "AFC bambu bridge: AMS drying: self-check passed, "
                     "now heating"),
            ("debug", "AMS: [AMS_CHMB]set state CTC_STATE_HEATING")]


class TestBambuBridgeChainUids:
    def test_uids_are_split_and_uppercased(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"aabb, ccdd"}')
        assert bridge.chain_uids() == ["AABB", "CCDD"]
        assert bridge.logger.messages == []

    def test_empty_fields_are_kept_so_indices_do_not_shift(self, monkeypatch):
        # Dropping a blank would renumber every later unit on the wire.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA,,CC"}')
        assert bridge.chain_uids() == ["AA", "", "CC"]
        assert bridge.logger.messages == []

    def test_no_uids_is_an_empty_list(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        assert bridge.chain_uids() == []
        bridge.handle_line('{"evt":"chain","uids":""}')
        assert bridge.chain_uids() == []
        assert bridge.logger.messages == []

    def test_the_list_is_a_copy(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA"}')
        bridge.chain_uids().append("BB")
        assert bridge.chain_uids() == ["AA"]
        assert bridge.logger.messages == []


class TestBambuBridgeChainSnapshot:
    def test_before_any_reply(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        assert bridge.chain_snapshot() == {"seq": 0, "uids": [], "htmask": 0,
                                           "a2mask": 0, "a2asks": []}
        assert bridge.logger.messages == []

    def test_seq_counts_parsed_replies_and_nothing_else(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","a2mask":1,'
                           '"a2asks":"3"}')
        first = bridge.chain_snapshot()
        assert first == {"seq": 1, "uids": ["AA"], "htmask": 0,
                         "a2mask": 1, "a2asks": [3]}
        assert bridge.chain_snapshot()["seq"] == 1   # reading does not advance
        bridge.handle_line("not json at all")
        bridge.handle_line('{"evt":"reply","hex":"3D05AA"}')
        assert bridge.chain_snapshot()["seq"] == 1
        # A reply with malformed fields still replaced the cache, so it counts.
        bridge.handle_line('{"evt":"chain","uids":"BB","htmask":"x",'
                           '"a2mask":"y"}')
        assert bridge.chain_snapshot() == {"seq": 2, "uids": ["BB"],
                                           "htmask": 0, "a2mask": 0,
                                           "a2asks": []}
        assert first["uids"] == ["AA"]              # a copy, not the cache
        assert bridge.logger.messages == []

    def test_a_snapshot_waits_for_a_reply_being_stored(self, monkeypatch):
        # Hold the lock as a store would: the snapshot must wait, then return
        # the new reply whole, never part old and part new.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA,BB","htmask":1,'
                           '"a2mask":2,"a2asks":"0,9"}')
        got: List[Dict[str, Any]] = []
        with bridge._lock:
            t = threading.Thread(
                target=lambda: got.append(bridge.chain_snapshot()))
            t.start()
            t.join(0.1)
            assert t.is_alive() and got == []      # blocked on the lock
            bridge._chain_seq = 2
            bridge._chain_uids = ["CC"]
            bridge._chain_htmask = 0
            bridge._chain_a2mask = 1
            bridge._chain_a2asks = [4]
        t.join(5)
        assert got == [{"seq": 2, "uids": ["CC"], "htmask": 0, "a2mask": 1,
                        "a2asks": [4]}]
        assert bridge.logger.messages == []


class TestBambuBridgeMcaddrAck:
    def test_unacknowledged_unit_is_none(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        assert bridge.mcaddr_ack(0) is None
        assert bridge.logger.messages == []

    def test_the_echo_is_recorded_per_unit(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"mcaddr","unit":0,"addr":6144}')
        bridge.handle_line('{"evt":"mcaddr","unit":1,"addr":1792}')
        assert bridge.mcaddr_ack(0) == 6144           # 0x1800, an HT
        assert bridge.mcaddr_ack(1) == 1792           # 0x0700, a boxed AMS
        assert bridge.mcaddr_ack(2) is None
        assert bridge.logger.messages == []

    def test_an_address_that_did_not_take_records_zero_not_none(
            self, monkeypatch):
        # The firmware refusing and the command never arriving are different
        # faults.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"mcaddr","unit":0,"addr":0}')
        assert bridge.mcaddr_ack(0) == 0
        assert bridge.logger.messages == []

    def test_a_later_echo_replaces_the_earlier_one(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"mcaddr","unit":0,"addr":1792}')
        bridge.handle_line('{"evt":"mcaddr","unit":0,"addr":6144}')
        assert bridge.mcaddr_ack(0) == 6144
        assert bridge.logger.messages == []


class TestBambuBridgeChainMcaddr:
    def test_it_is_read_from_the_chain_reply(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line(json.dumps(
            {"evt": "chain", "uids": "", "mcaddr": [6144, 1792]}))
        assert bridge.chain_mcaddr() == [6144, 1792]
        assert bridge.logger.messages == []

    def test_absent_is_none_not_empty(self, monkeypatch):
        # None is firmware too old to report; zeros are reported and unset.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line(json.dumps(
            {"evt": "chain", "uids": "", "mcaddr": [6144]}))
        bridge.handle_line(json.dumps({"evt": "chain", "uids": ""}))
        assert bridge.chain_mcaddr() is None
        assert bridge.logger.messages == []

    def test_all_zero_is_reported_as_such(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line(json.dumps(
            {"evt": "chain", "uids": "", "mcaddr": [0, 0]}))
        assert bridge.chain_mcaddr() == [0, 0]
        assert bridge.logger.messages == []

    def test_before_any_chain_reply_it_is_none(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        assert bridge.chain_mcaddr() is None
        assert bridge.logger.messages == []


class TestBambuBridgeChainDialect:
    def test_dialect_counters_ride_along(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA,BB","a2mask":2,'
                           '"a2asks":"4,17"}')
        assert bridge.chain_dialect() == (2, [4, 17])
        assert bridge.logger.messages == []

    def test_malformed_dialect_counters_read_as_none(self, monkeypatch):
        # The pair falls back together: a good mask beside counts that did
        # not parse would be judged against the wrong indices.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","a2mask":1,'
                           '"a2asks":"3,x"}')
        assert bridge.chain_dialect() == (0, [])
        bridge.handle_line('{"evt":"chain","uids":"AA","a2mask":"x",'
                           '"a2asks":"3"}')
        assert bridge.chain_dialect() == (0, [])
        assert bridge.logger.messages == []

    def test_absent_dialect_counters_read_as_none(self, monkeypatch):
        # They are the last reply's, not sticky.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","a2mask":1,'
                           '"a2asks":"3"}')
        bridge.handle_line('{"evt":"chain","uids":"AA","htmask":0}')
        assert bridge.chain_dialect() == (0, [])
        assert bridge.logger.messages == []

    def test_dialect_defaults_before_any_chain_reply(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        assert bridge.chain_dialect() == (0, [])
        assert bridge.logger.messages == []


class TestBambuBridgeChainDiag:
    def test_diagnostics_ride_along(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","htmask":5,'
                           '"fw":"1.0.10.6","selid":2,"selsent":7,'
                           '"selack":6}')
        assert bridge.chain_diag() == (5, "1.0.10.6", (2, 7, 6))
        assert bridge.logger.messages == []

    def test_malformed_diagnostics_fall_back_to_defaults(self, monkeypatch):
        # Older firmware omits them; a bad value must not poison the map.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","htmask":5,"selid":2}')
        bridge.handle_line('{"evt":"chain","uids":"AA","htmask":"x",'
                           '"selid":"y"}')
        assert bridge.chain_diag() == (0, "", (-1, 0, 0))
        assert bridge.logger.messages == []

    def test_diag_defaults_before_any_chain_reply(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        assert bridge.chain_diag() == (0, "", (-1, 0, 0))
        assert bridge.logger.messages == []

    def test_the_info_reply_s_build_wins_over_the_chain_s(self, monkeypatch):
        # Right after a flash the chain reply still names the previous build.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.handle_line('{"evt":"chain","uids":"AA","fw":"1.0.10.6"}')
        bridge.handle_line('{"evt":"info","chip":"rp2040","fw":"1.0.11.0"}')
        assert bridge.chain_diag() == (0, "1.0.11.0", (-1, 0, 0))
        # An info reply that names no build leaves the chain's.
        bridge.handle_line('{"evt":"info","chip":"rp2040"}')
        assert bridge.chain_diag()[1] == "1.0.10.6"
        assert bridge.logger.messages == [
            ("info", "AFC bambu: info REPLY chip=rp2040 fw=1.0.11.0 up=None"),
            ("info", "AFC bambu: info REPLY chip=rp2040 fw=None up=None")]


class TestBambuBridgeCheckQuiet:
    def test_silence_watchdog_reports_and_then_holds_its_tongue(
            self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, connected=False)
        # Never connected: unreachable since construction, at 100.0.
        bridge.reactor.advance(46.0)
        bridge._check_quiet()
        unreachable = ("warning", "AFC bambu: bridge has been unreachable for "
                                  "46s; commands are being dropped")
        assert bridge.logger.messages == [unreachable]
        assert bridge._silence_logged_t == 146.0
        bridge.reactor.advance(299.0)             # not again, not yet
        bridge._check_quiet()
        assert bridge.logger.messages == [unreachable]
        bridge.reactor.advance(1.0)               # one repeat interval on
        bridge._check_quiet()
        assert bridge.logger.messages == [
            unreachable,
            ("warning", "AFC bambu: bridge has been unreachable for 346s; "
                        "commands are being dropped")]
        assert bridge._silence_logged_t == 446.0

    def test_an_open_link_saying_nothing_is_silent_not_unreachable(
            self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge._last_frame_t = 100.0
        bridge.reactor.advance(46.0)
        bridge._check_quiet()
        assert bridge.logger.messages == [
            ("warning", "AFC bambu: bridge has been silent for 46s")]
        assert bridge._silence_logged_t == 146.0

    def test_a_quiet_healthy_link_says_nothing(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge._last_frame_t = 100.0              # a frame just arrived
        bridge.reactor.advance(44.0)
        bridge._check_quiet()
        assert bridge.logger.messages == []
        assert bridge._silence_logged_t is None

    def test_a_link_that_has_never_spoken_is_not_timed(self, monkeypatch):
        # Open, but no frame yet: there is nothing to measure from.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.reactor.advance(1000.0)
        bridge._check_quiet()
        assert bridge.logger.messages == []
        assert bridge._silence_logged_t is None


class TestBambuBridgeDropIfSilent:
    DROP_LOG: LogLine = ("warning", "AFC bambu: bridge silent for 30s on an "
                                    "open link -- dropping it to force a "
                                    "reconnect")

    @staticmethod
    def _spoke(bridge: BambuBridge) -> None:
        """A frame arrives now, as the reader stamps it."""
        bridge._last_frame_t = bridge.reactor.now
        bridge._spoke_since_connect = True

    @staticmethod
    def _reconnect(bridge: BambuBridge) -> FakeSerial:
        """
        The link comes back, as the reader opens it.

        :return FakeSerial: the new port
        """
        bridge._serial = FakeSerial()
        bridge._mark_connected()
        return bridge._serial

    def test_a_quiet_moment_is_not_a_drop(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        port = bridge._serial
        self._spoke(bridge)
        bridge.reactor.advance(5.0)
        assert bridge._drop_if_silent() is False
        assert (bridge._serial, port.closed) == (port, False)
        assert bridge.logger.messages == []

    def test_THE_ONE_THAT_BIT_long_silence_forces_a_reconnect(
            self, monkeypatch):
        # A bridge that vanishes without a FIN leaves read() timing out
        # forever on a socket to nothing.
        bridge = make_bambu_bridge(monkeypatch)
        port = bridge._serial
        self._spoke(bridge)
        bridge.reactor.advance(31.0)
        assert bridge._drop_if_silent() is True
        assert (bridge._serial, port.closed) == (None, True)
        assert (bridge.down_since(), bridge._down_epoch) == (131.0, 1)
        assert bridge.logger.messages == [self.DROP_LOG]

    def test_never_during_a_firmware_transfer(self, monkeypatch):
        # The board is silent while it counts image bytes; dropping the link
        # would abort the flash.
        bridge = make_bambu_bridge(monkeypatch)
        port = bridge._serial
        self._spoke(bridge)
        bridge._fw_raw = True
        bridge.reactor.advance(90.0)
        assert bridge._drop_if_silent() is False
        assert (bridge._serial, port.closed) == (port, False)
        assert bridge.logger.messages == []

    def test_the_threshold_leaves_room_for_a_normal_poll_gap(self, monkeypatch):
        # The firmware streams status continuously: ten quiet seconds is a
        # slow moment, a minute is a bridge that is gone.
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)
        bridge.reactor.advance(10.0)
        assert bridge._drop_if_silent() is False
        assert bridge.logger.messages == []
        bridge.reactor.advance(50.0)
        assert bridge._drop_if_silent() is True
        assert bridge.logger.messages == [self.DROP_LOG]

    def test_nothing_to_drop_when_already_disconnected(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)
        bridge._drop_port()
        bridge.reactor.advance(9999.0)
        assert bridge._drop_if_silent() is False
        # Still the one outage, stamped when the port went.
        assert (bridge.down_since(), bridge._down_epoch) == (100.0, 1)
        assert bridge.logger.messages == []

    def test_a_link_with_neither_a_frame_nor_an_open_stamp_is_kept(
            self, monkeypatch):
        # Nothing to measure from: a port set up outside start() or the
        # reader never stamped its open.
        bridge = make_bambu_bridge(monkeypatch, connected=False)
        bridge._serial = FakeSerial()
        bridge.reactor.advance(9999.0)
        assert bridge._drop_if_silent() is False
        assert bridge.logger.messages == []

    def test_an_open_link_that_never_spoke_is_dropped_on_its_own_age(
            self, monkeypatch):
        # No frame at all: the open stamp alone is what silence counts from.
        bridge = make_bambu_bridge(monkeypatch)
        port = bridge._serial
        assert (bridge._connected_t, bridge._last_frame_t) == (100.0, None)
        bridge.reactor.advance(29.0)
        assert bridge._drop_if_silent() is False
        assert bridge.logger.messages == []
        bridge.reactor.advance(1.0)
        assert bridge._drop_if_silent() is True
        assert (bridge._serial, port.closed) == (None, True)
        assert bridge.logger.messages == [self.DROP_LOG]

    def test_silence_counts_from_the_last_frame_without_an_open_stamp(
            self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch, connected=False)
        bridge._serial = FakeSerial()
        self._spoke(bridge)
        bridge.reactor.advance(29.0)
        assert bridge._drop_if_silent() is False
        bridge.reactor.advance(1.0)
        assert bridge._drop_if_silent() is True
        assert bridge.logger.messages == [self.DROP_LOG]

    def test_a_fresh_link_is_never_dropped_for_older_silence(self, monkeypatch):
        # Without the floor a reconnect after a long outage would inherit the
        # old stamp and be dropped on its first tick.
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)
        bridge.reactor.advance(90.0)              # a long outage
        port = self._reconnect(bridge)            # back up
        bridge.reactor.advance(1.0)
        assert bridge._drop_if_silent() is False
        assert (bridge._serial, port.closed) == (port, False)
        assert bridge.logger.messages == []

    def test_an_open_link_that_stays_quiet_is_still_dropped(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        port = bridge._serial
        self._spoke(bridge)
        bridge.reactor.advance(31.0)
        assert bridge._drop_if_silent() is True
        assert (bridge._serial, port.closed) == (None, True)
        assert bridge.logger.messages == [self.DROP_LOG]

    def test_a_quiet_link_is_dropped_on_its_own_age_after_a_reconnect(
            self, monkeypatch):
        # The floor delays the watchdog, it does not disable it.
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)
        bridge.reactor.advance(100.0)
        port = self._reconnect(bridge)            # opened at 200.0
        bridge.reactor.advance(29.0)
        assert bridge._drop_if_silent() is False
        assert bridge.logger.messages == []
        bridge.reactor.advance(2.0)
        assert bridge._drop_if_silent() is True
        assert (bridge._serial, port.closed) == (None, True)
        assert bridge.logger.messages == [self.DROP_LOG]

    def test_nothing_is_dropped_during_a_firmware_transfer(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        port = bridge._serial
        self._spoke(bridge)
        bridge._fw_raw = True
        bridge.reactor.advance(60.0)
        assert bridge._drop_if_silent() is False
        assert (bridge._serial, port.closed) == (port, False)
        # The same silence drops the link once the transfer is over.
        bridge._fw_raw = False
        assert bridge._drop_if_silent() is True
        assert bridge.logger.messages == [self.DROP_LOG]


class TestBambuBridgeSilentFor:
    @staticmethod
    def _spoke(bridge: BambuBridge) -> None:
        """A frame arrives now, as the reader stamps it."""
        bridge._last_frame_t = bridge.reactor.now
        bridge._spoke_since_connect = True

    @staticmethod
    def _reconnect(bridge: BambuBridge) -> None:
        """The link comes back, as the reader opens it."""
        bridge._serial = FakeSerial()
        bridge._mark_connected()

    def test_none_before_anything_has_ever_arrived(self, monkeypatch):
        # Opening a socket is not the bridge speaking.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.reactor.advance(10.0)
        assert bridge.silent_for() is None
        assert bridge.logger.messages == []

    def test_it_counts_from_the_last_frame(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge.reactor.advance(5.0)               # past the open's grace
        self._spoke(bridge)
        bridge.reactor.advance(3.0)
        assert bridge.silent_for() == 3.0
        assert bridge.logger.messages == []

    def test_a_reconnect_after_speaking_gets_the_grace(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)                       # this connection spoke
        bridge.reactor.advance(30.0)              # ... then the link died
        self._reconnect(bridge)                   # and came back
        bridge.reactor.advance(0.5)
        # Held down to the link's age, not the 30.5 s of real silence.
        assert bridge.silent_for() == 0.5
        assert bridge.logger.messages == []

    def test_the_grace_expires_and_the_real_silence_shows_through(
            self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)
        bridge.reactor.advance(30.0)
        self._reconnect(bridge)
        bridge.reactor.advance(2.5)               # the 2 s grace, and more
        # Not a reset: the clock underneath kept running.
        assert bridge.silent_for() == 32.5
        assert bridge.logger.messages == []

    def test_a_frame_older_than_the_link_is_not_held_up_by_the_grace(
            self, monkeypatch):
        # Inside the grace the smaller of the two counts wins.
        bridge = make_bambu_bridge(monkeypatch)
        bridge.reactor.advance(1.5)
        self._spoke(bridge)
        bridge.reactor.advance(0.25)
        assert bridge.silent_for() == 0.25
        assert bridge.logger.messages == []

    def test_a_mute_connection_earns_no_second_grace(self, monkeypatch):
        # The flapper: the first reconnect gets its grace; because that
        # connection never spoke, the next one does not.
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)
        bridge.reactor.advance(10.0)
        self._reconnect(bridge)                   # grace earned by the frame
        assert bridge.silent_for() == 0.0
        bridge.reactor.advance(1.0)               # says nothing, reconnects
        self._reconnect(bridge)
        assert bridge.silent_for() == 11.0
        assert bridge.logger.messages == []

    def test_speaking_again_re_earns_the_grace(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)
        bridge.reactor.advance(10.0)
        self._reconnect(bridge)
        self._spoke(bridge)                       # this one did speak
        bridge.reactor.advance(5.0)
        self._reconnect(bridge)
        bridge.reactor.advance(0.5)
        assert bridge.silent_for() == 0.5
        assert bridge.logger.messages == []

    def test_a_dropped_link_is_measured_plainly(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        self._spoke(bridge)
        bridge.reactor.advance(4.0)
        self._reconnect(bridge)
        bridge._drop_port()
        bridge.reactor.advance(1.0)
        assert bridge._connected_t is None
        assert bridge.silent_for() == 5.0
        assert bridge.logger.messages == []


class TestBambuBridgeDownSince:
    def test_down_since_and_is_connected_track_the_outage(self, monkeypatch):
        reactor = FakeReactor()
        patch_module_time(monkeypatch, reactor)
        threads = unstarted_bridge_threads(monkeypatch)
        ports: List[TcpPort] = []

        def factory() -> TcpPort:
            """:return TcpPort: a TCP link once; then the bridge is gone"""
            if ports:
                bridge._run = False               # one retry is enough here
                error_str = "connection refused"
                raise OSError(error_str)
            ports.append(make_tcp_port())
            return ports[0]

        logger = BambuLogger()
        bridge = BambuBridge(factory, reactor, logger)
        bridge.start()
        assert threads is bridge_mod.threading and bridge._thread.started
        assert bridge.is_connected() is True
        assert bridge.down_since() is None
        # The far end closes; the open's handshake wait took the clock to
        # 100.2, and the drop is stamped there.
        ports[0]._sock.inbox.append(b"")
        bridge._thread.target()
        assert bridge.is_connected() is False
        assert bridge.down_since() == pytest.approx(100.2)
        assert ports[0]._sock.closed is True
        bridge.stop()                             # nothing open to close
        assert (bridge._run, bridge.down_since()) == (
            False, pytest.approx(100.2))
        assert logger.messages == [
            ("warning", "AFC bambu: bridge read failed: tcp://test:8888: "
                        "bridge closed the connection; reconnecting")]

    def test_an_open_port_is_up_even_with_a_stale_outage_stamp(
            self, monkeypatch):
        # The port decides, not whatever outage stamp is left over.
        bridge = make_bambu_bridge(monkeypatch, connected=False)
        assert bridge.down_since() == 100.0
        bridge._serial = FakeSerial()
        assert bridge._down_t == 100.0
        assert bridge.down_since() is None
        assert bridge.logger.messages == []


class TestBambuBridgeDropPort:
    def test_drop_port_clears_and_closes(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        port = bridge._serial
        bridge.reactor.advance(5.0)
        bridge._drop_port()
        assert (bridge._serial, port.closed) == (None, True)
        assert bridge._connected_t is None
        assert (bridge._down_t, bridge._down_epoch) == (105.0, 1)
        assert bridge.logger.messages == []

    def test_drop_port_survives_a_close_that_throws(self, monkeypatch):
        class _DeadPort(FakeSerial):
            """A port whose close fails: the device is already gone."""

            def close(self) -> None:
                """:raises OSError: always"""
                error_str = "gone"
                raise OSError(error_str)

        bridge = make_bambu_bridge(monkeypatch)
        bridge._serial = _DeadPort()
        bridge._drop_port()
        assert bridge._serial is None
        assert (bridge._down_t, bridge._down_epoch) == (100.0, 1)
        assert bridge.logger.messages == []

    def test_queued_writes_go_with_the_dropped_port(self, monkeypatch):
        # Anything still queued was for the dead link and goes with it.
        bridge = make_bambu_bridge(monkeypatch)
        port = bridge._serial
        bridge.send({"cmd": "stop"})
        bridge.send({"cmd": "status"})
        bridge._drop_port()
        assert bridge_sent(bridge) == []
        assert (bridge._serial, port.closed) == (None, True)
        assert bridge.logger.messages == []

    def test_a_second_outage_gets_its_own_stamp_and_epoch(self, monkeypatch):
        bridge = make_bambu_bridge(monkeypatch)
        bridge._drop_port()
        assert (bridge._down_t, bridge._down_epoch) == (100.0, 1)
        bridge.reactor.advance(5.0)
        bridge._serial = FakeSerial()              # the link came back...
        bridge._drop_port()                        # ...and died again
        # A new outage, newly timed, so send() speaks again.
        assert (bridge._down_t, bridge._down_epoch) == (105.0, 2)
        assert bridge.logger.messages == []

    def test_dropping_again_while_down_keeps_the_outage(self, monkeypatch):
        # One outage, one stamp: a later drop with no port changes nothing.
        bridge = make_bambu_bridge(monkeypatch)
        bridge._drop_port()
        bridge.reactor.advance(5.0)
        bridge._drop_port()
        assert (bridge._down_t, bridge._down_epoch) == (100.0, 1)
        assert bridge.logger.messages == []

    def test_a_drop_before_any_outage_was_stamped_stamps_one(
            self, monkeypatch):
        # No port and no stamp: the stamp alone is enough to record one.
        bridge = make_bambu_bridge(monkeypatch, connected=False)
        bridge._down_t = None
        bridge.reactor.advance(5.0)
        bridge._drop_port()
        assert (bridge._down_t, bridge._down_epoch) == (105.0, 1)
        assert bridge.logger.messages == []


class TestBambuBridgeReader:
    HB = b'{"evt":"hb"}\n'

    class _ScriptedPort:
        """
        A port whose reads follow a script of (seconds_later, what).

        Each read moves the reactor's clock first; ``what`` is a chunk to
        return, an exception to raise, or a callable run with the bridge (as
        the writer thread would act meanwhile). The script running out stops
        the reader, so the real loop runs on the test's thread.
        """

        def __init__(self, bridge: BambuBridge,
                     steps: Sequence[Tuple[float, Any]] = ()) -> None:
            """
            :param bridge: the bridge reading it
            :param steps: the script
            """
            self.bridge = bridge
            self.steps: List[Tuple[float, Any]] = list(steps)
            self.closed = False

        def read(self, size: int = 1) -> bytes:
            """:return bytes: the next chunk, or b"" once the script is done"""
            while self.steps:
                dt, what = self.steps.pop(0)
                self.bridge.reactor.advance(dt)
                if isinstance(what, BaseException):
                    raise what
                if callable(what):
                    what(self.bridge)
                    continue
                return what
            self.bridge._run = False
            return b""

        def write(self, data: bytes) -> int:
            """:return int: bytes written"""
            return len(data)

        def close(self) -> None:
            """Mark the port closed."""
            self.closed = True

    class _GapSocket(FakeSocket):
        """
        A TCP socket whose script may hold read timeouts (None).

        A None advances the clock by the socket's timeout and times out, as
        a quiet link does; the script running out calls ``on_end``.
        """

        def __init__(self, script: Sequence[Optional[bytes]],
                     clock: FakeReactor,
                     on_end: Callable[[], None]) -> None:
            """
            :param script: chunks and timeouts, in order
            :param clock: the clock a timeout advances
            :param on_end: run when the script is exhausted
            """
            super().__init__((), clock=clock)
            self.script: List[Optional[bytes]] = list(script)
            self.on_end = on_end

        def recv(self, size: int) -> bytes:
            """:return bytes: the next chunk; raises socket.timeout for None"""
            if self.script and self.script[0] is not None:
                return self.script.pop(0)
            if self.script:
                self.script.pop(0)
            else:
                self.on_end()
            self.clock.advance(self.timeout or 0.0)
            raise socket.timeout()

    @staticmethod
    def _bridge(monkeypatch: pytest.MonkeyPatch,
                factory: Callable[[], Any],
                reactor: Optional[FakeReactor] = None) -> BambuBridge:
        """
        A bridge through its real ``__init__``, on module time.

        :param monkeypatch: pytest's monkeypatch fixture
        :param factory: its serial factory
        :param reactor: its reactor; a new FakeReactor at 100.0 when None
        :return BambuBridge: the bridge, not connected
        """
        reactor = reactor or FakeReactor()
        patch_module_time(monkeypatch, reactor)
        return BambuBridge(factory, reactor, BambuLogger())

    @staticmethod
    def _run(bridge: BambuBridge) -> None:
        """Run the reader loop on this thread until a script stops it."""
        bridge._run = True
        bridge._reader()

    @staticmethod
    def _two_timeouts(bridge: BambuBridge) -> None:
        """What the writer does for two timed-out writes."""
        bridge._write_timeouts += 2

    def test_lines_are_split_on_newlines_across_chunks(self, monkeypatch):
        ports: List[Any] = []
        bridge = self._bridge(monkeypatch, lambda: ports.pop(0))
        ports.append(self._ScriptedPort(bridge, [
            (0.0, b'{"evt":"ack","cmd":"a","slot":1}\n{"evt":"ac'),
            (0.0, b'k","cmd":"b","slot":2}\n')]))
        self._run(bridge)
        assert bridge.logger.messages == [
            ("info", "AFC bambu: bridge reconnected"),
            ("debug", "AFC bambu: bridge ack a (slot 1)"),
            ("debug", "AFC bambu: bridge ack b (slot 2)")]
        # Each chunk is a frame: the link spoke at the clock's time.
        assert (bridge._last_frame_t, bridge._spoke_since_connect) == (
            100.0, True)

    def test_a_read_error_drops_the_port_and_reconnects(self, monkeypatch):
        opened: List[Any] = []

        def factory() -> Any:
            """:return Any: a port that fails on read, then a quiet one"""
            if opened:
                port: Any = self._ScriptedPort(bridge)
            else:
                port = FakeSerial(fail_read=OSError("input/output error"))
            opened.append(port)
            return port

        bridge = self._bridge(monkeypatch, factory)
        self._run(bridge)
        assert len(opened) == 2
        assert opened[0].closed is True
        assert bridge._serial is opened[1]
        assert bridge._down_epoch == 1
        assert bridge.logger.messages == [
            ("info", "AFC bambu: bridge reconnected"),
            ("warning", "AFC bambu: bridge read failed: input/output error; "
                        "reconnecting"),
            ("info", "AFC bambu: bridge reconnected")]

    def test_a_requested_reset_is_not_reported_as_a_failure(self, monkeypatch):
        opened: List[Any] = []

        def factory() -> Any:
            """:return Any: a port that drops on read, then a quiet one"""
            if opened:
                port: Any = self._ScriptedPort(bridge)
            else:
                port = FakeSerial(fail_read=OSError("device disconnected"))
            opened.append(port)
            return port

        bridge = self._bridge(monkeypatch, factory)
        bridge._expect_reset = True               # {"cmd":"reset"} was sent
        self._run(bridge)
        assert bridge._expect_reset is False
        assert bridge.logger.messages == [
            ("info", "AFC bambu: bridge reconnected"),
            ("info", "AFC bambu: bridge resetting as asked; reconnecting"),
            ("info", "AFC bambu: bridge reconnected")]

    def test_reconnect_notifies_listeners_on_the_reactor(self, monkeypatch):
        # A reconnect usually means the Pico rebooted, so units re-push their
        # config, on the reactor rather than this thread.
        calls: List[int] = []
        bridge = self._bridge(monkeypatch, lambda: self._ScriptedPort(bridge))
        bridge.add_reconnect_listener(lambda: calls.append(1))
        bridge._silence_logged_t = 50.0
        bridge.reactor.advance(3.0)
        self._run(bridge)
        assert calls == []
        bridge.reactor.run_callbacks()
        assert calls == [1]
        assert (bridge._down_t, bridge._silence_logged_t) == (None, None)
        assert bridge._connected_t == 103.0
        assert bridge.logger.messages == [
            ("info", "AFC bambu: bridge reconnected after 3s down")]

    def test_a_failing_factory_backs_off_instead_of_spinning(self, monkeypatch):
        tries: List[float] = []

        def factory() -> Any:
            """Fail every open; stop the loop after the sixth."""
            tries.append(bridge.reactor.now)
            if len(tries) == 6:
                bridge._run = False
            error_str = "no such port"
            raise OSError(error_str)

        bridge = self._bridge(monkeypatch, factory)
        self._run(bridge)
        # 0.5 s, doubling, capped at 5 s.
        assert tries == [100.0, 100.5, 101.5, 103.5, 107.5, 112.5]
        assert bridge.reactor.now == 117.5
        assert bridge.logger.messages == []

    def test_a_failing_reactor_schedule_is_swallowed(self, monkeypatch):
        class _ClosingReactor(FakeReactor):
            """A reactor that refuses callbacks: Klipper shutting down."""

            def __init__(self) -> None:
                """Count the refusals."""
                super().__init__()
                self.refused = 0

            def register_async_callback(self, callback: Callable[[float], Any],
                                        waketime: float = 0.0) -> None:
                """:raises RuntimeError: always"""
                self.refused += 1
                error_str = "reactor gone"
                raise RuntimeError(error_str)

        reactor = _ClosingReactor()
        bridge = self._bridge(monkeypatch, lambda: self._ScriptedPort(
            bridge, [(0.0, b'{"evt":"ack","cmd":"x","slot":0}\n')]),
            reactor=reactor)
        bridge.add_reconnect_listener(lambda: None)
        self._run(bridge)
        assert reactor.refused == 1               # it tried to schedule
        # ...and the reader carried on reading.
        assert bridge.logger.messages == [
            ("info", "AFC bambu: bridge reconnected"),
            ("debug", "AFC bambu: bridge ack x (slot 0)")]

    def test_one_line_per_gap_with_the_timeouts_in_it(self, monkeypatch):
        # Written when the gap ends, so it carries the length.
        ports: List[Any] = []
        bridge = self._bridge(monkeypatch, lambda: ports.pop(0))
        ports.append(self._ScriptedPort(bridge, [
            (0.0, self.HB), (0.1, self.HB), (0.2, self.HB),
            (5.0, self._two_timeouts), (5.1, self.HB),   # a 10.1 s burst
            (0.1, self.HB), (0.1, self.HB),
            (2.4, self.HB),                              # under the threshold
            (3.0, self.HB)]))                            # a WiFi-sized gap
        self._run(bridge)
        want = ["AFC bambu: bridge was silent 10.1 s; 2 write(s) timed out "
                "meanwhile",
                "AFC bambu: bridge was silent 3.0 s; 0 write(s) timed out "
                "meanwhile"]
        assert bridge.logger.messages == (
            [("info", "AFC bambu: bridge reconnected")]
            + [("debug", line) for line in want])
        assert bridge.logger.file_only == want
        assert bridge._gap_timeouts_seen == 2

    def test_a_port_with_no_open_stamp_still_reports_its_gaps(
            self, monkeypatch):
        # A port handed in without the reconnect path has no open stamp; its
        # frames are still one connection, so a gap between them counts.
        opens: List[float] = []
        bridge = self._bridge(monkeypatch, lambda: opens.append(1.0))
        bridge._serial = self._ScriptedPort(bridge, [(0.0, self.HB),
                                                     (3.0, self.HB)])
        assert bridge._connected_t is None
        self._run(bridge)
        assert opens == []
        assert bridge.logger.messages == [
            ("debug", "AFC bambu: bridge was silent 3.0 s; 0 write(s) timed "
                      "out meanwhile")]
        assert bridge.logger.file_only == [
            "AFC bambu: bridge was silent 3.0 s; 0 write(s) timed out "
            "meanwhile"]

    def test_the_first_frame_of_a_connection_is_not_a_gap(self, monkeypatch):
        ports: List[Any] = []
        bridge = self._bridge(monkeypatch, lambda: ports.pop(0))
        ports.append(self._ScriptedPort(bridge, [(9.0, self.HB),
                                                 (0.1, self.HB)]))
        self._run(bridge)
        assert bridge.logger.messages == [
            ("info", "AFC bambu: bridge reconnected")]

    def test_a_reconnect_starts_over(self, monkeypatch):
        # The first frame after a drop follows an outage, not a gap on this
        # link, and the timeouts from before the drop are written off.
        ports: List[Any] = []
        bridge = self._bridge(monkeypatch, lambda: ports.pop(0))
        ports.append(self._ScriptedPort(bridge, [
            (0.0, self.HB), (0.1, self.HB), (1.0, self._two_timeouts),
            (1.0, OSError("input/output error"))]))
        ports.append(self._ScriptedPort(bridge, [
            (6.0, self.HB),                      # first frame on the new link
            (0.1, self.HB), (4.0, self.HB)]))    # a gap on it, from zero
        self._run(bridge)
        assert bridge.logger.messages == [
            ("info", "AFC bambu: bridge reconnected"),
            ("warning", "AFC bambu: bridge read failed: input/output error; "
                        "reconnecting"),
            ("info", "AFC bambu: bridge reconnected"),
            ("debug", "AFC bambu: bridge was silent 4.0 s; 0 write(s) timed "
                      "out meanwhile")]

    def test_reader_reconnects_after_read_error(self, monkeypatch):
        unstarted_bridge_threads(monkeypatch)
        made: List[Any] = []

        def factory() -> Any:
            """:return Any: a port that errors once, then a quiet one"""
            steps = [] if made else [(0.0, RuntimeError("io error"))]
            made.append(self._ScriptedPort(bridge, steps))
            return made[-1]

        bridge = self._bridge(monkeypatch, factory)
        bridge.start()
        bridge._thread.target()
        bridge.stop()
        assert len(made) == 2                     # re-opened, did not die
        assert made[0].closed is True             # dropped the broken port
        assert made[1].closed is True             # and stop closed the new one
        assert bridge.logger.messages == [
            ("warning", "AFC bambu: bridge read failed: io error; "
                        "reconnecting"),
            ("info", "AFC bambu: bridge reconnected")]

    def test_the_watchdog_fires_from_the_READER_while_the_link_is_down(
            self, monkeypatch):
        # A disconnected reader never reaches the read path, so a watchdog
        # called only from there could never fire in the state it is for.
        unstarted_bridge_threads(monkeypatch)
        calls: List[int] = []

        def never() -> Any:
            """Fail every open; stop after start's and five of the reader's."""
            calls.append(1)
            if len(calls) == 6:
                bridge._run = False
            error_str = "nothing is listening"
            raise OSError(error_str)

        bridge = self._bridge(monkeypatch, never)
        bridge.start(defer_open=True)
        # Unreachable since construction at 100.0; the reader's attempts then
        # come 0.5, 1, 2 and 4 s apart, the last at 47.75 s.
        bridge.reactor.advance(40.25)
        bridge._thread.target()
        assert bridge.logger.messages == [
            ("warning", "AFC bambu: bridge not reachable yet (nothing is "
                        "listening); the reader will keep trying"),
            ("warning", "AFC bambu: bridge has been unreachable for 48s; "
                        "commands are being dropped")]
        assert bridge._silence_logged_t == 147.75

    def test_a_gap_on_a_tcp_link_is_logged_once_and_file_only(
            self, monkeypatch):
        # WiFi gaps are ordinary, so the line stays off the console and says
        # each gap once, when it ends.
        unstarted_bridge_threads(monkeypatch)
        reactor = FakeReactor()
        socks: List[FakeSocket] = []

        def factory() -> TcpPort:
            """:return TcpPort: the link, its handshake timing out (no key)"""
            sock = self._GapSocket(
                [None, self.HB, self.HB] + [None] * 30 + [self.HB],
                clock=reactor,
                on_end=lambda: setattr(bridge, "_run", False))
            socks.append(sock)
            with patch.object(bridge_mod.socket, "create_connection",
                              lambda address, timeout=None: sock):
                return TcpPort("test", 8888, timeout=0.1, connect_timeout=0.5)

        bridge = self._bridge(monkeypatch, factory, reactor=reactor)
        bridge.start()
        bridge._thread.target()
        line = ("AFC bambu: bridge was silent 3.0 s; 0 write(s) timed out "
                "meanwhile")
        assert bridge.logger.messages == [("debug", line)]
        assert bridge.logger.file_only == [line]
        assert len(socks) == 1                    # never dropped


class TestBambuBridgeQuietDropS:
    def test_the_default_threshold_is_far_below_the_transport_timers(
            self, monkeypatch):
        # A unit pauses the print after link_loss_pause_s of silence; the
        # transport must not drop the link until well after that.
        printer = make_printer(monkeypatch=monkeypatch)
        bridge = make_bambu_bridge(monkeypatch, printer=printer)
        unit = make_bambu_unit(printer=printer, bridge=bridge)
        assert unit.link_loss_pause_s == 5.0
        bridge._last_frame_t = bridge.reactor.now
        bridge.reactor.advance(2 * unit.link_loss_pause_s)
        assert bridge._drop_if_silent() is False
        bridge.reactor.advance(19.9)              # 29.9 s of silence
        assert bridge._drop_if_silent() is False
        bridge.reactor.advance(0.1)               # 30 s of silence
        assert bridge._drop_if_silent() is True
        assert bridge.logger.messages == [
            ("warning", "AFC bambu: bridge silent for 30s on an open link -- "
                        "dropping it to force a reconnect")]


class TestAMSHuman:
    @staticmethod
    def _said(monkeypatch: pytest.MonkeyPatch, *lines: str) -> List[LogLine]:
        """
        What the console narration says for each line, two seconds apart.

        :param monkeypatch: pytest's monkeypatch fixture
        :param lines: the AMS's words
        :return List[LogLine]: everything the bridge logged
        """
        bridge = make_bambu_bridge(monkeypatch)
        for i, text in enumerate(lines):
            bridge._narrate_human(text, 100.0 + 2.0 * i)
        return bridge.logger.messages

    @pytest.mark.parametrize("line,sentence", [
        ("[AMS_DEV] STEP,first detected", "AMS: spool detected"),
        ("[AMS_DEV] STEP:card auth success!", "AMS: tag authenticated"),
        ("[RF] tray0: info write to flash",
         "AMS: tag for bay 1 cached in the unit's flash (a later read "
         "returns it even after a swap)"),
    ])
    def test_every_real_ams1_line_narrates(self, monkeypatch, line, sentence):
        # An AMS 1 says [AMS_DEV] and puts a space after the bracket.
        assert self._said(monkeypatch, line) == [
            ("info", f"AFC bambu bridge: {sentence}")]

    def test_flash_cache_line_names_the_bay_one_based(self, monkeypatch):
        assert self._said(monkeypatch, "[RF] tray0: info write to flash",
                          "[RF] tray3: info write to flash") == [
            ("info", "AFC bambu bridge: AMS: tag for bay 1 cached in the "
                     "unit's flash (a later read returns it even after a "
                     "swap)"),
            ("info", "AFC bambu bridge: AMS: tag for bay 4 cached in the "
                     "unit's flash (a later read returns it even after a "
                     "swap)")]

    def test_the_ams2_rules_still_work(self, monkeypatch):
        # The [AMS_DEV] rules sit ahead of the [AMS_RFID] ones and must not
        # shadow them.
        assert self._said(monkeypatch, "[AMS_RFID]STEP:card auth success!") == [
            ("info", "AFC bambu bridge: AMS: tag authenticated")]

    @pytest.mark.parametrize("line", [
        "[AMS_DEV] STEP:read success,valid",
        "[AMS_DEV] STEP:feed with rfid success",
        "[AMS_RFID] STEP3,read success ,goto Cali",
        "[AMS_RFID] STEP3,feed with rfid success",
    ])
    def test_a_mid_cycle_read_claim_is_not_narrated(self, monkeypatch, line):
        # An HT says these on an attempt that then fails and retries;
        # narrating them announced a read ten seconds before it happened.
        assert self._said(monkeypatch, line) == []

    @pytest.mark.parametrize("line", [
        "[AMS_DEV] STEP:card auth success!",
        "[AMS_RFID]STEP:card auth success!",
        "[AMS_RFID] STEP3,auth card successful",
    ])
    def test_the_authentication_is_narrated_in_every_dialect(
            self, monkeypatch, line):
        assert self._said(monkeypatch, line) == [
            ("info", "AFC bambu bridge: AMS: tag authenticated")]

    def test_a_measurement_reads_as_a_sentence(self, monkeypatch):
        assert self._said(
            monkeypatch,
            "[AMS_RFID]STEP:odom C:0.478,R:0.076,P:79%, od:0.491") == [
            ("info", "AFC bambu bridge: AMS measured the spool: about 79% "
                     "left (spool radius 76 mm)")]

    def test_the_stored_flash_value_is_surfaced(self, monkeypatch):
        # The only place the unit says what it remembers, at power-up.
        assert self._said(
            monkeypatch,
            "[AMS_RFID] STEP,odom load from flash 0,R:0.075,P:75") == [
            ("info", "AFC bambu bridge: AMS: bay 1 remembers about 75% left "
                     "from its last measurement")]

    def test_a_stall_names_both_distances(self, monkeypatch):
        assert self._said(
            monkeypatch,
            "[AMS_SWITCH]feed finish -1, stall, len_det:1.620 m, "
            "tube_len:3.506 m") == [
            ("info", "AFC bambu bridge: AMS: the filament STALLED after "
                     "1.62 m of a 3.51 m path -- check for a jam between the "
                     "bay and the toolhead")]

    def test_an_error_code_is_named_not_dumped(self, monkeypatch):
        assert self._said(monkeypatch, "[AMS_LINK]err_code:0x00->0x17",
                          "[AMS_LINK]err_code:0x17->0x00") == [
            ("info", "AFC bambu bridge: AMS raised error 0x17"),
            ("info", "AFC bambu bridge: AMS cleared its error")]

    def test_bays_are_one_based_for_humans(self, monkeypatch):
        # The wire counts from 0; an operator counts bays from 1.
        assert self._said(monkeypatch,
                          "[AMS_RFID]STEP:odom save tray:1, R:0.0765",
                          "[AMS_IDLE]tray 0 out,clear magic_num") == [
            ("info", "AFC bambu bridge: AMS stored a new measurement for "
                     "bay 2"),
            ("info", "AFC bambu bridge: AMS: bay 1 is now empty")]


class TestBridgeEventsKnown:
    @staticmethod
    def _logged(monkeypatch: pytest.MonkeyPatch,
                *frames: Dict[str, Any]) -> List[LogLine]:
        """
        What handle_line logs for each frame.

        :param monkeypatch: pytest's monkeypatch fixture
        :param frames: the bridge's frames, in order
        :return List[LogLine]: everything the bridge logged
        """
        bridge = make_bambu_bridge(monkeypatch)
        for frame in frames:
            bridge.handle_line(json.dumps(frame))
        return bridge.logger.messages

    def test_scan_echoes_are_not_unhandled_events(self, monkeypatch):
        assert self._logged(monkeypatch, {"evt": "scan"}, {"evt": "reid"},
                            {"evt": "reread"}, {"evt": "prime"}) == []

    def test_the_announce_no_longer_sends_selfc(self, monkeypatch):
        # Retired: an echo of it now is unexplained, so it is logged.
        assert self._logged(monkeypatch, {"evt": "selfc"}) == [
            ("debug", "AFC bambu: unhandled bridge event {'evt': 'selfc'}")]

    def test_the_event_is_known_to_the_bridge(self, monkeypatch):
        # The unknown-event catch-all runs before the per-event branches, so
        # an unlisted txecho would never reach its own handler.
        assert self._logged(monkeypatch, {"evt": "txecho", "drops": 0}) == []

    def test_the_per_round_echoes_are_known(self, monkeypatch):
        # Unlisted, each would be logged every status round.
        assert self._logged(monkeypatch, {"evt": "bind"},
                            {"evt": "htuid"}) == []

    def test_every_command_echo_we_send_is_known(self, monkeypatch):
        # Each lands in its own handler, never the catch-all.
        assert self._logged(monkeypatch, {"evt": "chain"}, {"evt": "status"},
                            {"evt": "ack"}, {"evt": "units"},
                            {"evt": "htunit"}) == [
            REQUEST_INFO_LOG,
            ("debug", "AFC bambu: bridge ack  (slot None)")]


class TestChmbStateRe:
    # Verbatim drying telemetry, both separators.
    HT = ("[AMS_CHMB]s:2|rf:55,0|vt:44.0|ap:35.3|hts:34,31,00|pw:100|ad:2"
          "|wd:0000|fa:98|t:70")
    AMS2 = ("[AMS_CHMB]s:2|rf:65,0|vt:24.1|ap:22.0|hts:52,22,00|pw:100|ad:2"
            "|wd:0000|fa:99|t:40")
    COMMA = ("[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:23.0, hts:46,23,0 "
             "pw:100, ad:2, wd:0,0,0,0, fa:102")

    @staticmethod
    def _fields(line: str) -> Tuple[Optional[str], ...]:
        """
        :param line: a telemetry line
        :return tuple: (state, target, chamber C, humidity %)
        """
        m = _CHMB_STATE_RE.search(line)
        assert m is not None, f"did not match: {line}"
        return m.group(1), m.group(2), m.group(3), m.group(4)

    def test_ams_ht_line(self):
        # hts:34,... is 34 %RH: not None and not the 00 after vt.
        assert self._fields(self.HT) == ("2", "55", "44.0", "34")

    def test_ams2_pro_line(self):
        assert self._fields(self.AMS2) == ("2", "65", "24.1", "52")

    def test_line_with_a_leading_prefix(self):
        # The drain interleaves other text ahead of the record.
        assert self._fields("R " + self.HT) == ("2", "55", "44.0", "34")

    def test_the_suffix_after_vt_is_not_humidity(self):
        # On a live HT that field is 00 every sample while the chamber goes
        # from 22C to 55C. With no ht: there is no humidity to report.
        assert self._fields("[AMS_CHMB]s:2|rf:55|vt:29.8,38") == (
            "2", "55", "29.8", None)

    def test_humidity_comes_from_ht(self):
        assert self._fields(
            "[AMS_CHMB]s:2|rf:55|vt:22.4,00|ap:22.3|ht:60,22|t:7") == (
            "2", "55", "22.4", "60")

    def test_humidity_falls_as_the_chamber_heats(self):
        # The correlation that identified the field, from one real cycle.
        got = [self._fields(
            f"[AMS_CHMB]s:2|rf:55|vt:{vt},00|ap:30.0|ht:{ht},22|t:{t}")[3]
            for t, vt, ht in ((7, 22.4, 60), (67, 33.2, 55),
                              (127, 55.6, 40), (178, 52.7, 31))]
        assert got == ["60", "55", "40", "31"]

    @pytest.mark.parametrize("line", [
        "[AMS_CHMB]dry_mode:1, check ok! dur:480,tmpr:55,pre_check:1",
        "[AMS_CHMB]s:off->wind_res1",
        "[AMS_CHMB]finish!",
        "[AMS_CHMB]set state CTC_STATE_HEATING, from selfcheck",
    ])
    def test_non_telemetry_chatter_is_ignored(self, line):
        assert _CHMB_STATE_RE.search(line) is None

    def test_comma_separated_form(self):
        assert self._fields(self.COMMA) == ("2", "55", "23.1", "46")

    def test_comma_form_with_a_leading_framing_byte(self):
        # The drain reply is often prefixed by one stray rendered byte.
        assert self._fields("\\ " + self.COMMA) == ("2", "55", "23.1", "46")

    def test_cd_field_is_not_mistaken_for_the_chamber_probe(self):
        # cd sits between rf and vt and here equals the target; the chamber
        # reading still comes from vt.
        assert self._fields(self.COMMA)[2] == "23.1"

    def test_both_separators_still_parse(self):
        # Which form a unit emits follows the addressing, not the model.
        assert self._fields(self.HT)[1] == "55"
        assert self._fields(self.COMMA)[1] == "55"


class TestDbgAmstimeRe:
    LINE = "[AMS_LINK]get_slot ams1 tray0 basic"
    BEAT = " [DBG] ams time: now=42044054ms diff=10005ms"

    def test_the_heartbeat_segment_is_stripped(self):
        assert _DBG_AMSTIME_RE.sub("", self.LINE + self.BEAT).strip() == (
            "[AMS_LINK]get_slot ams1 tray0 basic")

    def test_a_heartbeat_riding_with_real_narration_keeps_the_narration(self):
        # The heartbeat may come first, too; only its own segment goes.
        assert _DBG_AMSTIME_RE.sub("", self.BEAT + " " + self.LINE).strip() == (
            "[AMS_LINK]get_slot ams1 tray0 basic")


class TestRFIDCycleEndRe:
    def test_ht_cycle_end_still_matches(self):
        assert _RFID_CYCLE_END_RE.search(
            "[AMS_RFID] STEP4,Calibration rst:0") is not None

    @pytest.mark.parametrize("cap", ams_narration_captures_with("notag"),
                             ids=repr)
    def test_an_empty_bay_narrates_an_end_and_never_a_read(self, cap):
        # Without an end, "no tag" never becomes a fact; with a read, the lane
        # would take the previous spool's record.
        assert cap.first("end") is not None, cap.name
        assert cap.first("read") is None, cap.name

    @pytest.mark.parametrize("cap", ams_narration_captures_with("read"),
                             ids=repr)
    def test_the_terminal_marker_does_not_fire_before_the_read(self, cap):
        # _scan_verdict asks "did it read?" before "did it finish?", so an end
        # ahead of the read resolves a tagged scan as notag. The AMS 1 says
        # "odom calib success" on the insert edge, about 32 s before its read,
        # which is why that line is not an end.
        first_read, first_end = cap.first("read"), cap.first("end")
        if first_end is None:
            return                          # nothing terminal in this capture
        assert first_read is not None and first_read <= first_end, cap.name


class TestRFIDForeignTagRe:
    @pytest.mark.parametrize("line", [
        "[AMS_RFID]STEP:auth fail:-4",
        # The HT's spelling: a space and no colon.
        "[AMS_RFID] STEP3,auth fail -4",
    ])
    def test_the_refusal_is_recognised(self, line):
        assert _RFID_FOREIGN_TAG_RE.search(line) is not None

    @pytest.mark.parametrize("line", [
        "[AMS_RFID]STEP7:info_valid 0 or bbl:-1",
        "[AMS_RFID] STEP4,info_valid 0 or bbl:1",
    ])
    def test_info_valid_zero_is_not_foreign_evidence(self, line):
        # It shows on empty-bay cycles and mid-retry on HT reads that then
        # succeed: "no valid record right now", not "chip refused".
        assert _RFID_FOREIGN_TAG_RE.search(line) is None

    @pytest.mark.parametrize("line", [
        "[AMS_RFID] STEP3,save to flash ,card info valid",
        "[AMS_DEV] STEP:read success,valid",
        "[AMS_RFID]STEP0:checking",
        "[AMS_RFID]STEP:tray pull over 880 mm, but no card detected",
    ])
    def test_a_good_read_or_an_empty_bay_is_not_a_refusal(self, line):
        assert _RFID_FOREIGN_TAG_RE.search(line) is None

    @pytest.mark.parametrize("cap", ams_narration_captures_with("foreign"),
                             ids=repr)
    def test_a_foreign_chip_is_told_apart_from_an_empty_bay(self, cap):
        assert cap.first("foreign") is not None, cap.name
        assert cap.first("read") is None, cap.name


class TestRFIDReadOkRe:
    # One complete successful read per model, verbatim.
    HT_OK = [
        # Authenticated and committed to its own flash.
        "[AMS_RFID] STEP3,auth card successful [RF] tray0: info write to "
        "flash [AMS_RFID] STEP3,save to flash ,card info valid",
        # And said the read landed.
        "[AMS_RFID] STEP3,feed with rfid success [AMS_RFID] STEP3,read "
        "success ,goto Cali",
    ]
    BOXED_OK = [
        "[AMS_DEV] STEP:read success,valid",       # AMS 1: space, colon
        "[AMS_RFID]STEP:read success,valid",       # AMS 2: no space, colon
        "[AMS_DEV] STEP:read_done=1",
    ]
    # A read that runs is not a read that lands.
    NOT_OK = [
        "[AMS_RFID] STEP2,search 0 card",
        "[AMS_RFID] STEP3,empty to read,feed with rfid",
        "[AMS_DEV] STEP5:no card in RF",
        "[AMS_DEV] STEP:search finished, found 0 card",
        "[AMS_DEV] STEP:tray pull over 790 mm, but no card detected",
        "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1",
    ]

    @pytest.mark.parametrize("line", HT_OK + BOXED_OK)
    def test_every_dialect_reports_a_landed_read(self, line):
        assert _RFID_READ_OK_RE.search(line) is not None

    @pytest.mark.parametrize("line", NOT_OK)
    def test_running_or_failed_is_not_a_landed_read(self, line):
        assert _RFID_READ_OK_RE.search(line) is None

    @pytest.mark.parametrize("line", AMS_NARRATION_LINES, ids=repr)
    def test_a_line_matches_exactly_what_it_claims(self, line):
        # The drift lock, both ways: a pattern that stops firing on a model it
        # served, and one that starts firing where it should not.
        assert line.text, f"{line!r} has no narration text"
        got = {name for name, pat in AMS_NARRATION_PATTERNS.items()
               if pat.search(line.text)}
        assert got == line.expect, (
            f"{line!r} ({line.offset}): {line.text}\n"
            f"  fixture expects: {sorted(line.expect) or ['.']}\n"
            f"  patterns give:   {sorted(got) or ['.']}")

    @pytest.mark.parametrize("cap", ams_narration_captures_with("read"),
                             ids=repr)
    def test_a_successful_scan_narrates_a_read(self, cap):
        # Otherwise _scan_verdict can only wait for its fallback.
        assert cap.first("read") is not None, cap.name

    @pytest.mark.parametrize("model,line", [
        ("ams1", "[AMS_DEV] STEP:read success,valid"),
        ("ams2", "[AMS_RFID]STEP:feed with rfid success"),
        ("ht", "[AMS_RFID] STEP3,read success ,goto Cali"),
    ])
    def test_every_model_has_a_read_the_pattern_recognises(self, model, line):
        # A model the pattern is blind to can only resolve a scan by timing
        # out.
        assert _RFID_READ_OK_RE.search(line) is not None, model

    @pytest.mark.parametrize("model,line", [
        ("ams1", "[RF] tray0: info write to flash"),
        ("ams2", "[RF] tray0: info write to flash"),
        ("ht", "[AMS_RFID] STEP3,save to flash ,card info valid"),
    ])
    def test_every_models_commit_sentence_counts_as_a_read(self, model, line):
        # A commit is the unit writing the record it will serve: the read is
        # recognised there, not at some later sentence.
        assert _RFID_READ_OK_RE.search(line) is not None, model

    @pytest.mark.parametrize("model,line", [
        ("ams1", "[AMS_DEV] STEP:card auth success!"),
        ("ams2", "[AMS_RFID]STEP:card auth success!"),
        ("ht", "[AMS_RFID] STEP3,auth card successful"),
    ])
    def test_no_model_treats_authentication_alone_as_a_read(self, model, line):
        # The HT can authenticate a chip and still serve its flash cache.
        assert _RFID_READ_OK_RE.search(line) is None, model


class TestRFIDTerminalRe:
    @pytest.mark.parametrize("line", [
        # One per model. The HT never says STEP7; it ends on Calibration rst.
        "0x1800 [AMS_RFID] STEP4,Calibration rst:0",          # HT
        "0x1800 [AMS_RFID] STEP4,Calibration rst:4",          # HT, stalled
        "0x0700 [AMS_RFID]STEP7:cali end",                    # AMS 2
        "0x0700 [AMS_DEV] STEP7:finish,cali tray",            # AMS 1
        "0x1800 [AMS_RFID] STEP4,tray capacity no en",        # HT, no measure
    ])
    def test_every_model_narrates_an_ending_we_recognise(self, line):
        assert _RFID_TERMINAL_RE.search(line) is not None

    @pytest.mark.parametrize("line", [
        "[AMS_RFID]STEP7:ready to cali tray",
        "[AMS_RFID]STEP7:info_valid 0 or bbl:-1",
        "[AMS_RFID]STEP7:cali read tray 1",
    ])
    def test_a_mid_cycle_step7_is_not_an_ending(self, line):
        # STEP7 is a phase, not a full stop.
        assert _RFID_TERMINAL_RE.search(line) is None


class TestStateSwitchDoneRe:
    @pytest.mark.parametrize("line", [
        # An AMS 2 unload and an AMS HT unload: the prefix differs, which is
        # why the pattern is not anchored.
        "[AMS_SWITCH]SRL_state_switch finish, sucessful, err_code:0x00",
        "[AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, err_code:0x00",
    ])
    def test_the_regex_matches_a_real_completed_retract(self, line):
        assert _STATE_SWITCH_DONE_RE.search(line) is not None

    def test_a_nonzero_err_code_is_not_a_completion(self):
        # Pre-update firmware says this after a feed finish that already
        # ended the wait; it says nothing about this move.
        assert _STATE_SWITCH_DONE_RE.search(
            "[AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, "
            "err_code:0x25") is None
