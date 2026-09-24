# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# ACE 2 Pro RFID integration.
#
# Contains, bottom-to-top:
#   Ace2Link      ACE2 V2 serial frame + the firmware passthrough commands:
#                 MFRC522 register read / write and reader power.
#   _Ace2RegLink  the same passthrough spoken over the live ACE2 serial object.
#   AFC_ACE2_RFID  the Klipper [AFC_ACE2_rfid] object: read a lane/slot tag and
#                 apply the decoded filament to an AFC lane (+ optional Spoolman).
#
# The MFRC522 primitives, MIFARE access and multi-brand tag decoders this
# transport feeds live in extras/AFC_rfid_readers.py, shared with the
# ViViD/OpenAMS readers.
#
# The firmware only ever exposes reg_read(reg)->byte, reg_write(reg,val) and
# reader_power(on); every bit of MIFARE/crypto/brand logic stays host-side and
# editable without reflashing.
from __future__ import annotations
import struct
import traceback
from typing import (TYPE_CHECKING, Any, Callable, Dict, List, Optional, Tuple)

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC import afc
    from extras.AFC_lane import AFCLane

# Shared MFRC522/MIFARE primitives + multi-brand decoders. This module supplies
# the ACE2 transport they run on.
from extras.AFC_rfid_readers import read_tag, Mfrc522, MifareClassic
from extras.AFC_rfid_write import StageError, register_reader
from extras.AFC_RFID import (sync_rfid_to_spoolman, apply_filament_defaults,
                             build_filament_name, prompt_hold_spool,
                             dismiss_prompt, AFCUnitRFID,
                             map_tag_to_slot_info, SpoolmanClient)
try:
    from extras.AFC_RFID import get_auto_spoolman_create
except ImportError:                                    # older AFC without the helper
    get_auto_spoolman_create = None
try:
    from extras.AFC_RFID import resolve_rfid_keys
except ImportError:                                    # older AFC without the helper
    resolve_rfid_keys = None


# ═════════════════════════════
# ACE2 frame + passthrough, pure logic, unit-testable in isolation; the
# shared register/crypto/brand stack it feeds lives in AFC_rfid_readers.
# ═════════════════════════════

# ── ACE2 V2 frame ──────────────────────
PREAMBLE = b"\xff\xaa"
END = b"\xfe"


def crc16_kermit(data: bytes, init: int = 0xFFFF) -> int:
    """
    CRC-16/KERMIT over a frame body.

    :param data: bytes to checksum
    :param init: starting register value
    :return int: 16-bit CRC
    """
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if (c & 1) else c >> 1
    return c & 0xFFFF


# Opcodes for the passthrough commands, registered in firmware.
CMD_MFRC522_REG_READ = 0x50
CMD_MFRC522_REG_WRITE = 0x51
CMD_MFRC522_READER_POWER = 0x52     # v2: host-owned reader power (Bambu reads)


def _varint(n: int) -> bytes:
    """
    Encode an unsigned protobuf base-128 varint.

    :param n: value to encode
    :return bytes: the varint bytes
    """
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | 0x80 if n else b)
        if not n:
            return bytes(out)


def _varint_decode(data: bytes, i: int = 0) -> Tuple[int, int]:
    """
    Decode one protobuf varint.

    :param data: buffer to read from
    :param i: offset of the first byte
    :return tuple: (value, offset after the varint)
    """
    result = shift = 0
    while True:
        b = data[i]
        i += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, i
        shift += 7


class Ace2Link:
    """Frames the ACE2 protocol and issues the passthrough commands.

    `transport(frame:bytes) -> bytes` sends a framed request and returns the
    raw response frame (whatever your serial layer does). For unit tests, a
    fake transport / a fake Mfrc522 is injected higher up instead.
    """
    def __init__(self, transport: Callable[[bytes], bytes], slot: int = 0,
                 ftype: int = 0x0000) -> None:
        """
        Frame builder for an MFRC522 register link over a raw transport.

        :param transport: callable that sends a frame and returns the reply
        :param slot: ACE2 reader slot this link drives
        :param ftype: frame type word
        """
        self._tx = transport
        self._ftype = ftype
        self._seq = 0
        self.slot = slot            # which ACE2 reader slot (0..3) this link drives

    def _next_seq(self) -> int:
        """
        Advance the 8-bit frame sequence number.

        :return int: the next sequence value
        """
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def build_frame(self, cmd: int, payload: bytes) -> bytes:
        """
        Wrap a command and payload in a framed, CRC'd packet.

        :param cmd: command byte
        :param payload: payload bytes
        :return bytes: the complete frame
        """
        body = struct.pack(">H", self._ftype) + bytes([cmd, len(payload)]) + payload
        crc = crc16_kermit(body)
        return PREAMBLE + bytes([self._next_seq()]) + body + struct.pack("<H", crc) + END

    # Firmware request/response are a single protobuf field #1 (varint):
    #   reg-read     req arg = (slot<<16) | reg               -> resp{val}
    #   reg-write    req arg = (slot<<16) | (reg<<8) | val     -> resp{} (empty)
    #   reader-power req arg = (reader_index<<16) | on          -> resp{} (empty)
    def reg_read(self, reg: int) -> int:
        """
        Read one MFRC522 register on this slot's reader.

        :param reg: register address
        :return int: register value
        """
        arg = (self.slot << 16) | (reg & 0xFF)
        resp = self._tx(self.build_frame(CMD_MFRC522_REG_READ, b"\x08" + _varint(arg)))
        return self._parse_field1(resp)

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one MFRC522 register on this slot's reader.

        :param reg: register address
        :param val: value to write
        """
        arg = (self.slot << 16) | ((reg & 0xFF) << 8) | (val & 0xFF)
        self._tx(self.build_frame(CMD_MFRC522_REG_WRITE, b"\x08" + _varint(arg)))

    def reader_power(self, on: bool) -> None:
        """
        Switch the slot's reader power.

        :param on: True to power the reader
        """
        arg = (self.slot << 16) | (1 if on else 0)
        self._tx(self.build_frame(CMD_MFRC522_READER_POWER, b"\x08" + _varint(arg)))

    @staticmethod
    def _parse_field1(resp: bytes) -> int:
        """
        Pull field 1 (a varint) out of a response frame's payload.

        Frame: FF AA seq TYPE(2) CMD LEN PAYLOAD CRC(2) FE, where PAYLOAD is
        protobuf {field1: varint val} = 08 <varint>.

        :param resp: raw response frame
        :return int: decoded value
        """
        if not resp or resp[:2] != PREAMBLE:
            error_str = "bad ACE2 response preamble"
            raise IOError(error_str)
        ln = resp[6]
        payload = resp[7:7 + ln]
        if len(payload) >= 2 and payload[0] == 0x08:
            return _varint_decode(payload, 1)[0] & 0xFF
        return 0


# ═════════════════════════════
# Klipper integration, [AFC_ACE2_rfid]
# ═════════════════════════════

class _Ace2RegLink:
    """Adapts an ACE2 serial object to the reg_read/reg_write reader contract.

    Each read/write is one passthrough command over the existing ACE2 link:
      reg-read     -> send_command('mfrc522_reg_read',     {'arg': (slot<<16)|reg})
      reg-write    -> send_command('mfrc522_reg_write',    {'arg': (slot<<16)|(reg<<8)|val})
      reader-power -> send_command('mfrc522_reader_power', {'arg': (slot<<16)|on})
    """
    def __init__(self, ace2: Any, slot: int, power_index: Optional[int] = None,
                 reg_timeout: float = 2.0) -> None:
        """
        Register link that routes MFRC522 access through the ACE2 serial link.

        :param ace2: the afcACE2 printer object
        :param slot: reader chip-select index
        :param power_index: slot index for the power command, defaults to slot
        :param reg_timeout: seconds to wait for a register reply
        """
        self._ace2 = ace2            # the afcACE2 printer object
        # reg r/w chip-select index (each slot has its own MFRC522 on the shared
        # SPI2 bus, selected by this field).
        self.slot = slot & 0xFF
        # reader-power index. Power is per reader pair (PD12/PD13 cover slots
        # 0-1 / 2-3), so it can differ from the per-slot reg index above.
        self.power_index = (self.slot if power_index is None
                            else power_index & 0xFF)
        # Per-register serial timeout. A cold reader can stall a reg op, so each
        # one is bounded to keep a long read from wedging the link.
        self._reg_timeout = reg_timeout

    def _conn(self) -> Any:
        """
        Fetch the live ACE2 serial connection at call time.

        The connection (afcACE._ace) may be recreated, so it is never cached.

        :return Any: the serial connection object
        """
        conn = getattr(self._ace2, "_ace", None)
        if conn is None:
            error_str = "ACE2 serial not connected"
            raise IOError(error_str)
        return conn

    def reg_read(self, reg: int) -> int:
        """
        Read one MFRC522 register through send_command.

        :param reg: register address
        :return int: register value
        """
        arg = (self.slot << 16) | (reg & 0xFF)
        res = self._conn().send_command(
            "mfrc522_reg_read", {"arg": arg}, timeout=self._reg_timeout)
        return int((res or {}).get("val", 0)) & 0xFF

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one MFRC522 register through send_command.

        :param reg: register address
        :param val: value to write
        """
        arg = (self.slot << 16) | ((reg & 0xFF) << 8) | (val & 0xFF)
        self._conn().send_command(
            "mfrc522_reg_write", {"arg": arg}, timeout=self._reg_timeout)

    def reader_power(self, on: bool) -> None:
        """
        Switch the reader power (firmware cmd 0x52).

        Indexed by the per-pair power_index, which may differ from the per-slot
        reg index.

        :param on: True to power the reader
        """
        arg = (self.power_index << 16) | (1 if on else 0)
        self._conn().send_command(
            "mfrc522_reader_power", {"arg": arg}, timeout=self._reg_timeout)

    def set_rfid_enable(self, phys_slot: int, enable: bool) -> None:
        """
        Enable or disable the firmware identify loop for a physical slot.

        Firmware cmd 0x0E; the firmware maps slot>>1 to the shared reader.
        Disabling frees the reader and gates its power off so the host can drive
        it. Sent fire-and-forget so a missing ack can't stall the read; wire
        order still puts it ahead of the following synchronous commands.

        :param phys_slot: physical slot index
        :param enable: True to hand the reader back to the firmware
        """
        conn = self._conn()
        conn.send_command_async(
            "set_rfid_enable", {"index": phys_slot & 0xFF, "enable": bool(enable)})


class _StageCmd:
    """Stands in for the g-code command in the rescan motion when a tag write
    runs it: a failure becomes a StageError, which the write command reports
    as its own error."""

    @staticmethod
    def error(msg: str) -> StageError:
        """
        Build the error a failed rescan move raises.

        :param msg: what went wrong
        :return StageError: the exception to raise
        """
        return StageError(msg)

    @staticmethod
    def respond_info(msg: str) -> None:
        """
        Nothing to answer; the write command reports the outcome.

        :param msg: ignored
        """


class AFC_ACE2_RFID(AFCUnitRFID):
    """Reads ACE 2 Pro spool tags via the firmware passthrough and applies them
    to AFC lanes. Configure with ``[AFC_ACE2_rfid]``."""

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Set up the ACE2 RFID section: keys, options, lane map and commands.

        :param config: the [AFC_ACE2_rfid] section
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        # Take AFC's logger now (load_object builds AFC if needed). self.afc
        # stays None until klippy:ready on purpose: the guards use it as the
        # ready marker.
        self.logger = self.printer.load_object(config, 'AFC').logger
        # One scan at a time; see cmd_ACE_RFID_SCAN.
        self._scan_running = False
        self.gcode = self.printer.lookup_object("gcode")

        key = (config.get("bambu_master_key", "") or "").strip()
        self.bambu_master_key = bytes.fromhex(key) if key else None
        # Creality CFS keys (both required to decode Creality tags): key = the
        # UID->MIFARE-key AES key (u_key), encryption_key = the AES-CBC payload
        # key (d_key). Every other format needs no config key.
        ck = (config.get("creality_key", "") or "").strip()
        cek = (config.get("creality_encryption_key", "") or "").strip()
        self.creality_key = bytes.fromhex(ck) if ck else None
        self.creality_encryption_key = bytes.fromhex(cek) if cek else None
        self.auto_create = config.getboolean("auto_spoolman_create", False)
        self.log_prefix = "ACE2 RFID"          # AFCUnitRFID.apply_to_lane / Spoolman
        # Auto-read the tag when a spool is inserted (the insert preload/feed
        # spins the spool, sweeping the tag past the reader). Retries a few times
        # to catch it as it settles.
        self.read_on_insert = config.getboolean("read_on_insert", True)
        self.read_on_insert_attempts = config.getint(
            "read_on_insert_attempts", 3, minval=1)
        self.read_on_insert_delay = config.getfloat(
            "read_on_insert_delay", 1.0, minval=0.0)
        # Per-register serial timeout (see _Ace2RegLink). A healthy reg op takes
        # a few ms, so a short bound makes a wedged passthrough fail fast.
        self.read_reg_timeout = config.getfloat(
            "read_reg_timeout", 0.8, minval=0.2)
        # Reg chip-select index. This unit has two MFRC522 readers, one per slot
        # pair, so reg r/w uses slot>>1 unless per-slot is opted in.
        self.reader_reg_per_slot = config.getboolean("reader_reg_per_slot", False)
        # disable_rfid: no automatic tag reads. Inserts still stage to the hub
        # as a plain feed; ACE_RFID_READ still works.
        self.disable_rfid = config.getboolean("disable_rfid", False)
        # stage_read: poll the reader during the staging feed and stop on the
        # first UID so the tag is read at rest. Feed assist is suppressed for the
        # read, or its stuck-spool check would trip while the feed pauses.
        self.stage_read = config.getboolean("stage_read", True)
        # Scan feed speed (mm/s). Slower coasts less past the antenna after the
        # stop; the remainder is fed at full feed_speed.
        self.stage_scan_speed = min(90.0, config.getfloat(
            "stage_scan_speed", 25.0, minval=1.0))
        # Parked-read attempts after stop-on-detect, before the re-center sweep.
        self.stage_read_hold_attempts = config.getint(
            "stage_read_hold_attempts", 3, minval=1)
        # Re-center sweep: if the parked read misses, retract in small steps (the
        # tag coasts past the read zone) up to this total; 0 disables it. The
        # retract is fed back with the remainder.
        self.stage_recenter_max = config.getfloat(
            "stage_recenter_max", 42.0, minval=0.0)
        self.stage_recenter_step = config.getfloat(
            "stage_recenter_step", 3.0, minval=0.5)
        # Deprecated (chunked staging is gone), still read so existing configs
        # that set it don't raise an "unused option" error at startup.
        self.stage_read_chunk = config.getfloat(
            "stage_read_chunk", 25.0, minval=2.0)
        # auto_tag_adjust: when the shared-reader sister's tag blocks a read,
        # retract that sibling to turn its tag off the coil, then restore it. Only
        # an idle, hub-staged sibling is moved, never during a print. When off, the
        # console asks the user to turn the sibling spool by hand.
        self.auto_tag_adjust = config.getboolean("auto_tag_adjust", True)
        self.auto_tag_adjust_dist = config.getfloat(
            "auto_tag_adjust_dist", 75.0, minval=1.0)
        # Initial load fed before dist_hub when the factory autostage is skipped
        # (it replaces the factory's own staging). Net load = this + dist_hub.
        self.stage_initial_dist = config.getfloat(
            "stage_initial_dist", 500.0, minval=0.0)
        # Scan window: how far to feed at scan speed while polling. It must exceed
        # one spool revolution so the tag sweeps past; it adds no net distance.
        self.stage_scan_dist = config.getfloat(
            "stage_scan_dist", 600.0, minval=0.0)
        # Two slots share one reader, so a read can return the neighbour's parked
        # tag. With dedup on, a UID matching the sibling's known tag (while that
        # sibling holds a spool) is rejected and the scan keeps looking.
        self.shared_reader_dedup = config.getboolean("shared_reader_dedup", True)
        # Last UID successfully read per PHYSICAL slot, used for the dedup above.
        self._slot_uid: Dict[int, str] = {}
        # Skip the ACE's slow (~30-45s) factory autostage by disabling identify
        # on every slot at boot, so our own staging runs instead.
        self.skip_factory_autostage = config.getboolean(
            "skip_factory_autostage", True)
        # The boot-time identify-disable races the ACE's own deferred serial
        # connect (which re-enables identify), so poll until connected then
        # disable. These bound that poll (default ~30s window).
        self._identify_disable_delay = config.getfloat(
            "identify_disable_retry", 0.5, minval=0.05)
        self._identify_disable_max = config.getint(
            "identify_disable_max_tries", 60, minval=1)
        self._identify_disable_tries = 0
        # Restore factory identify after each read. Needed only when factory
        # autostaging is in use, or the next insert won't autostage.
        self.probe_restore_identify = config.getboolean(
            "restore_identify", not self.skip_factory_autostage)
        self.probe_power_off = config.getboolean("probe_power_off", True)
        self.probe_settle = config.getfloat("probe_settle", 0.2, minval=0.0)
        # Name of the ACE2 printer object exposing send_command (afcACE subclass).
        self.ace2_name = config.get("ace2_object", "AFC_ACE2")
        # "lane1:0, lane2:1" -> {lane_name: physical slot}
        self._lane_slot: Dict[str, int] = {}
        for pair in (config.get("lane_slot_map", "") or "").split(","):
            pair = pair.strip()
            if not pair:
                continue
            name, _, slot = pair.partition(":")
            self._lane_slot[name.strip()] = int(slot or 0)

        # Scanner lanes: ACE_RFID_SCAN reads a presented tag on the lane's reader
        # and stages it as the next spool id. Each needs a slot in lane_slot_map.
        self._scanner_lanes = set()
        for n in (config.get("scanner_lanes", "") or "").split(","):
            n = n.strip()
            if n:
                self._scanner_lanes.add(n)
        # How long ACE_RFID_SCAN reads for (seconds) and the dwell between read
        # attempts. Present the spool's tag to the reader during this window.
        self.scan_seconds = config.getfloat("scan_seconds", 30.0, minval=1.0)
        self.scan_interval = config.getfloat("scan_interval", 0.3, minval=0.05)
        # A scan accepts a UID only after this many consecutive matching decodes,
        # so a stray read can't pick the wrong Spoolman spool.
        self.scanner_confirm_reads = config.getint(
            "scanner_confirm_reads", 2, minval=1)

        self.afc: Optional["afc"] = None
        self.ace2: Optional[Any] = None
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler("afc_ace:post_insert",
                                            self._on_post_insert)
        self.printer.register_event_handler("afc_ace:connected",
                                            self._on_ace_connected)
        # Continuous read during staging (see stage_read).
        self._probe: Optional[dict] = None
        # slots whose feed assist we suppressed
        self._assist_slots: Optional[Tuple[int, ...]] = None
        # (sibling_slot, dist, speed) when a shared-reader sister was retracted to
        # clear its tag off the antenna during this stage; None otherwise. Restored
        # (re-fed the same distance) in _stage_probe_end.
        self._sister_retracted: Optional[Tuple[Any, float, float]] = None
        # True once we've told the user (this stage) to nudge the sister spool by
        # hand, so the console isn't spammed with repeat hints during one stage.
        self._sister_hint_shown = False
        # The staging feed fires begin (the whole scan-and-read runs there) and
        # end (teardown).
        self.printer.register_event_handler("afc_ace:stage_probe_begin",
                                            self._stage_probe_begin)
        self.printer.register_event_handler("afc_ace:stage_probe_end",
                                            self._stage_probe_end)
        # No digit in the command names: Klipper parses a token like "ACE2" as a
        # G/M-style command ("Unknown command ACE2").
        self.gcode.register_command(
            "ACE_RFID_READ", self.cmd_ACE_RFID_READ,
            desc="Read the ACE 2 Pro RFID tag for a lane (LANE=) or slot (SLOT=)")
        self.gcode.register_command(
            "ACE_RFID_SCAN", self.cmd_ACE_RFID_SCAN,
            desc="Scan a scanner lane's RFID for a presented tag and stage it as "
                 "the next spool id (LANE=, optional SECONDS=)")
        self.gcode.register_command(
            "ACE_RFID_BLOCKS", self.cmd_ACE_RFID_BLOCKS,
            desc="Dump raw MIFARE blocks of a Bambu tag (LANE=/SLOT=, optional "
                 "BLOCKS=5,16), shows color count + second color for dual-color")

    def _register_writers(self) -> None:
        """
        Offer each slot's reader to the shared tag writer (AFC_RFID_WRITE).

        Registered per slot, since a blank sticker is held against one slot's
        antenna. prepare and release mirror the managed read: stop the firmware
        identify loop on both slots of the pair (it would tear a multi-page write
        apart), take the power, and put both back afterwards.
        """
        n = int(getattr(self.ace2, "slot_count", 4) or 4)

        def _open(slot: int) -> Optional[_Ace2RegLink]:
            """
            Build a link for a slot, or None when the unit is gone.

            :param slot: physical slot
            :return Optional[_Ace2RegLink]: the link, or None
            """
            if self.ace2 is None:
                return None
            return self._new_link(self._reg_index(slot) & 0xFF,
                                  power_index=(slot >> 1) & 0xFF)

        def _prepare(link: _Ace2RegLink, slot: int) -> None:
            """
            Take the reader off the firmware and power it, as a read does.

            :param link: the slot's register link
            :param slot: physical slot
            """
            pair = (slot >> 1) & 0xFF
            for s in (pair * 2, pair * 2 + 1):
                link.set_rfid_enable(s, False)
            self._dwell(0.1)                    # let the firmware stop
            link.reader_power(True)
            self._dwell(0.1)                    # oscillator/power-on settle

        def _release(link: _Ace2RegLink, slot: int) -> None:
            """
            Hand the reader back.

            Best-effort: a wedged serial here must not mask the write's own
            result.

            :param link: the slot's register link
            :param slot: physical slot
            """
            try:
                link.reader_power(False)
            except Exception:
                self.logger.error("ACE2 RFID: reader power-off failed",
                                  traceback=traceback.format_exc())
            if not self.probe_restore_identify:
                return
            pair = (slot >> 1) & 0xFF
            for s in (pair * 2, pair * 2 + 1):
                try:
                    link.set_rfid_enable(s, True)
                except Exception:
                    self.logger.error(
                        "ACE2 RFID: re-enable identify failed",
                        traceback=traceback.format_exc())

        for slot in range(n):
            register_reader(
                self.printer, f"ace2:slot{slot}", f"ACE2 Pro slot {slot}",
                self, lambda s=slot: _open(s),
                prepare=lambda link, s=slot: _prepare(link, s),
                release=lambda link, s=slot: _release(link, s),
                stage_around=lambda ln, body, s=slot: self._stage_write_around(
                    ln, s, body),
                serves=lambda ln, s=slot: self._physical_slot(ln) == s)

    def _on_ready(self) -> None:
        """
        Resolve AFC and the ACE2 unit at klippy:ready and finish wiring.
        """
        self.afc = self.printer.lookup_object("AFC", None)
        # Fall back to the shared [AFC_rfid_keys] section for any brand key this
        # section didn't set.
        if resolve_rfid_keys is not None:
            (self.bambu_master_key, self.creality_key,
             self.creality_encryption_key) = resolve_rfid_keys(
                self.printer, self.bambu_master_key, self.creality_key,
                self.creality_encryption_key)
        # Mark configured scanner lanes so scanner-aware code (and any bridge
        # hooks) sees lane.spool_scanner, mirroring the U1.
        if self.afc is not None and hasattr(self.afc, "lanes"):
            for n in self._scanner_lanes:
                lane = self.afc.lanes.get(n)
                if lane is not None:
                    try:
                        lane.spool_scanner = True
                    except Exception:
                        pass
        # The ACE2 serial object is a named section: "AFC_ACE2 <name>" (e.g.
        # "AFC_ACE2 Ace2_1"). Try the configured/bare name, else auto-discover
        # the first AFC_ACE2* object.
        self.ace2 = (self.printer.lookup_object(self.ace2_name, None)
                     or self.printer.lookup_object("AFC_ACE2", None))
        if self.ace2 is None:
            for name, obj in self.printer.lookup_objects():
                if name == self.ace2_name or name.startswith("AFC_ACE2 "):
                    self.ace2 = obj
                    break
        if self.ace2 is None:
            self.logger.warning("ACE2 object not found; RFID disabled")
        else:
            self.logger.info(f"ACE2 RFID bound to {getattr(self.ace2, 'name', self.ace2)}")
            self._register_writers()
            # Identify is set once the ACE connects (_on_ace_connected), at boot
            # and after every late connect or reset.

    def _on_ace_connected(self, unit: Any, reconnect: bool) -> None:
        """
        Set the factory identify whenever the unit connects or reconnects.

        Identify off skips the slow factory autostage (AFC's staging takes
        over). A unit that reset on a USB drop, or came up late, has identify
        back on and would autostage the next insert, so this runs on every
        connect.

        :param unit: the ACE unit that connected
        :param reconnect: True for a reconnect after a drop (unused)
        """
        if unit is not self.ace2 or not self.skip_factory_autostage:
            return
        if getattr(self, "_identify_retry_pending", False):
            return                  # a retry is already waiting for it
        self._identify_disable_tries = 0
        self._identify_retry_pending = True
        self.reactor.register_callback(self._retry_disable_identify)

    def _retry_disable_identify(self, eventtime: float) -> None:
        """
        Disable factory identify once the ACE serial is connected.

        The ACE's deferred connect races klippy:ready and re-enables identify as
        it comes up, so poll until connected, then disable so it sticks.

        :param eventtime: reactor event time (unused)
        """
        conn = getattr(self.ace2, "_ace", None) if self.ace2 is not None else None
        if conn is not None and getattr(conn, "connected", False):
            self._identify_retry_pending = False
            self._disable_factory_identify()
            return
        self._identify_disable_tries += 1
        if self._identify_disable_tries >= self._identify_disable_max:
            self._identify_retry_pending = False
            self.logger.warning(
                f"ACE2 RFID: ACE serial not ready after {self._identify_disable_tries} tries; "
                f"factory identify not set (autostage may run on insert)")
            return
        self.reactor.register_callback(
            self._retry_disable_identify,
            self.reactor.monotonic() + self._identify_disable_delay)

    def _disable_factory_identify(self, slots: Optional[Any] = None) -> None:
        """
        Disable the firmware identify so the ACE stops driving the reader.

        This also stops its autonomous autostage. Best-effort; each insert's
        stage read disables it again for its own reader pair.

        :param slots: slots to disable, defaults to all of the unit's slots
        """
        if self.ace2 is None:
            return
        if slots is None:
            n = int(getattr(self.ace2, "slot_count", 4) or 4)
            slots = range(n)
        link = self._new_link(0)
        for s in slots:
            try:
                link.set_rfid_enable(int(s), False)
                self.logger.info(f"ACE2 RFID: factory identify disabled on slot {s}")
            except Exception:
                self.logger.info(
                    f"ACE2 RFID: could not disable factory identify on slot {s} yet (serial not "
                    f"ready?)")

    # ── mapping ───────────────────────
    def _slot_for_lane(self, lane_name: str) -> Optional[int]:
        """
        Resolve the physical reader slot for a lane.

        Prefers the ACE unit's own slot map, so a stale lane_slot_map can't
        point a read at the wrong reader.

        :param lane_name: AFC lane name
        :return Optional[int]: slot index, None when unmapped
        """
        return self._physical_slot(lane_name)

    def _map(self, tag: dict) -> dict:
        """
        Map a read_tag() result to AFC's shared slot_info shape.

        :param tag: raw tag dict from read_tag
        :return dict: slot_info for AFC_RFID
        """
        return map_tag_to_slot_info(tag)

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        Report the lane->slot map and the per-lane last-read records.

        :param eventtime: Reactor event time (unused; kept for the status API).
        :return dict: The lane_slot_map and last_reads records.
        """
        return {
            "lane_slot_map": dict(self._lane_slot),
            "last_reads": self.last_reads_status(),
        }

    # ── read + apply ─────────────────────
    def _new_link(self, reg_idx: int,
                  power_index: Optional[int] = None) -> _Ace2RegLink:
        """
        Build a register link to one of this unit's readers.

        :param reg_idx: reader chip-select index
        :param power_index: reader-pair index for the power command
        :return _Ace2RegLink: the link
        """
        return _Ace2RegLink(self.ace2, reg_idx, power_index=power_index,
                            reg_timeout=self.read_reg_timeout)

    def _probe_uid(self, link: Any) -> Optional[str]:
        """
        Return the UID of the tag in the reader's field, without decoding it.

        :param link: reader link
        :return Optional[str]: UID hex, None when no tag
        """
        try:
            uid, _sak = MifareClassic(Mfrc522(link)).activate()
        except Exception:
            return None
        return uid.hex() if uid is not None else None

    def _read_tag(self, link: Any,
                  is_excluded: Optional[Callable[[str], bool]] = None,
                  seen: Optional[List[Any]] = None,
                  dump_blocks: Any = None) -> Optional[dict]:
        """
        Run read_tag with all configured brand keys (Bambu and Creality).

        :param link: register link to the reader
        :param is_excluded: predicate for UIDs to halt instead of decoding
        :param seen: list that collects every UID seen, when given
        :param dump_blocks: MIFARE blocks to dump raw, when given
        :return Optional[dict]: decoded tag, or None when nothing read
        """
        return read_tag(link, bambu_master_key=self.bambu_master_key,
                        creality_key=self.creality_key,
                        creality_encryption_key=self.creality_encryption_key,
                        is_excluded=is_excluded, seen=seen,
                        dump_blocks=dump_blocks)

    def read_slot(self, slot: int, manage_power: bool = True,
                  reg_slot: Optional[int] = None,
                  dump_blocks: Any = None) -> Optional[dict]:
        """
        Read the tag on a physical slot (0..3).

        Two readers cover the four slots, so the reader index is ``slot >> 1``.
        With ``manage_power`` the host owns the reader for the read: it stops the
        firmware identify loop (which power-cycles the reader and breaks an
        encrypted Bambu read), powers the reader via the firmware command
        (0x52), reads, then restores power-off and identify.

        :param slot: physical slot
        :param manage_power: take the reader from the firmware; False reads
            without taking it, best-effort
        :param reg_slot: diagnostic reg index override, powering that same reader
        :param dump_blocks: MIFARE blocks to dump raw, when given
        :return Optional[dict]: raw tag dict, or None when nothing read
        """
        if self.ace2 is None:
            error_str = "ACE2 not available"
            raise self.printer.command_error(error_str)
        # reg chip-select index (per-slot); REG= overrides it for diagnostics.
        # Power is per-reader-pair (slot>>1), independent of the reg index.
        if reg_slot is None:
            reg_idx = self._reg_index(slot) & 0xFF
            power_idx = (slot >> 1) & 0xFF
        else:
            # Diagnostic: REG is the reader index; power the same reader.
            reg_idx = int(reg_slot) & 0xFF
            power_idx = reg_idx
        link = self._new_link(reg_idx, power_index=power_idx)
        if not manage_power:
            return self._read_tag(link, dump_blocks=dump_blocks)
        # Disable identify on both slots of the pair: it frees the reader and
        # suppresses the firmware autostage. Re-enabling is optional.
        pair = (slot >> 1) & 0xFF
        sib = pair * 2
        shared = (sib, sib + 1)
        for s in shared:
            link.set_rfid_enable(s, False)
        sibling = sib if sib + 1 == slot else sib + 1
        self._dwell(0.1)                        # let the firmware stop identifying
        link.reader_power(True)                 # host powers the reader itself
        self._dwell(0.1)                        # MFRC522 oscillator/power-on settle
        try:
            # Halt the sibling's parked tag so this slot's own tag is read.
            excluder = self._probe_excluder(slot, sibling)
            tag = self._read_tag(link, is_excluded=excluder,
                                 dump_blocks=dump_blocks)
            uid = (tag or {}).get("uid")
            if uid is not None:
                if self._is_sibling_tag(slot, sibling, uid):
                    self.logger.warning(
                        f"ACE2 RFID: slot {slot} read matches shared sibling slot {sibling}'s tag ("
                        f"uid={uid}), check the spool is in slot {slot}, not {sibling}")
                self._slot_uid[slot] = uid
            return tag
        finally:
            # Best-effort restore, a wedged serial during restore must not mask
            # the result or add another hard failure.
            try:
                link.reader_power(False)
            except Exception:
                self.logger.error(
                    "ACE2 RFID: reader power-off failed", traceback=traceback.format_exc())
            # Re-enable identify only when not keeping it off for
            # skip_factory_autostage (else the next insert would autostage).
            if self.probe_restore_identify:
                for s in shared:
                    try:
                        link.set_rfid_enable(s, True)
                    except Exception:
                        self.logger.error(
                            "ACE2 RFID: re-enable identify failed",
                                traceback=traceback.format_exc())

    def _dwell(self, seconds: float) -> None:
        """
        Yield to the reactor for a short settle delay (no-op without one).

        :param seconds: how long to wait
        """
        r = getattr(self, "reactor", None)
        if r is not None:
            r.pause(r.monotonic() + seconds)

    # apply_to_lane() comes from AFCUnitRFID. The unit's auto_spoolman_create wins;
    # the [AFC_ACE2_rfid] value is only the fallback default.

    def read_lane(self, lane_name: str) -> Optional[dict]:
        """
        Read the tag on the reader slot mapped to the given lane and, when
        that lane exists in AFC, apply the tag's filament to it. Raises a command
        error if the lane has no slot mapped (set lane_slot_map).

        :param lane_name: Name of the AFC lane to read the tag for
        :return dict: Tag data that was read, or None when no tag is present
        """
        slot = self._slot_for_lane(lane_name)
        if slot is None:
            error_str = f"lane {lane_name!r} has no ACE2 reader slot (set lane_slot_map)"
            raise self.printer.command_error(error_str)
        tag = self.read_slot(slot)
        if not tag:
            self.logger.info(f"ACE2 RFID: no tag read on slot {slot}")
            return None
        lane = None
        if self.afc is not None:
            lane = self.afc.lanes.get(lane_name) if hasattr(self.afc, "lanes") else None
        if lane is not None:
            self.apply_to_lane(lane, tag)
        return tag

    # ── scanner lane: read a presented tag, stage it as the next spool id ─────
    def _scan_slot(self, slot: int, duration: float,
                   lane_name: str = "") -> Optional[dict]:
        """
        Power the slot's reader and read repeatedly for up to ``duration`` seconds.

        The tag is read at rest, so it must be presented within the antenna's
        range. The reader stays powered for the whole scan, since no feed runs.
        The first sight of the tag shows a "hold the spool" popup, dismissed on
        a timeout or replaced by the result notification.

        :param slot: physical slot
        :param duration: how long to scan, seconds
        :param lane_name: label for the popup
        :return Optional[dict]: the confirmed tag, or None
        """
        reg_idx = self._reg_index(slot) & 0xFF
        power_idx = (slot >> 1) & 0xFF
        link = self._new_link(reg_idx, power_index=power_idx)
        pair = (slot >> 1) & 0xFF
        sib = pair * 2
        shared = (sib, sib + 1)
        sibling = sib if sib + 1 == slot else sib + 1
        # Free the reader for host control (and suppress firmware autostage);
        # reader_power(OFF) is the sync barrier that flushes the async disable.
        for s in shared:
            link.set_rfid_enable(s, False)
        link.reader_power(False)
        self._dwell(0.1)
        link.reader_power(True)
        self._dwell(0.1)
        deadline = self.reactor.monotonic() + max(0.0, duration)
        excluder = self._probe_excluder(slot, sibling)
        seen_uids: Dict[str, bool] = {}   # uid_hex -> whether it was decoded
        confirm = max(1, self.scanner_confirm_reads)
        cand_uid = None         # UID of the current confirmation candidate
        streak = 0              # consecutive full decodes of cand_uid
        detected_shown = False  # "hold the spool" popup shown once per scan
        # Fresh scan: no sister retracted, no manual-nudge hint shown yet.
        self._sister_retracted = None
        self._sister_hint_shown = False
        try:
            while self.reactor.monotonic() < deadline:
                seen: List[Any] = []
                try:
                    tag = self._read_tag(link, is_excluded=excluder, seen=seen)
                except Exception:
                    self.logger.error(
                        "ACE2 RFID scan: read error", traceback=traceback.format_exc())
                    tag = None
                for h, _s, _e in seen:
                    seen_uids.setdefault(h, False)
                # First sight of the presented tag: ask the user to hold it
                # steady while the confirmation reads complete. Once per scan.
                if not detected_shown and (
                        any(not e for _h, _s, e in seen)
                        or (tag and tag.get("uid"))):
                    prompt_hold_spool(self.gcode.respond_raw,
                                      lane_name or (f"slot {slot}"))
                    detected_shown = True
                # Only the sister's tag in the field: it blocks the coil. Clear
                # it if allowed, otherwise ask the user once to nudge it.
                if (any(e for _h, _s, e in seen)
                        and not any(not e for _h, _s, e in seen)):
                    self._handle_sister_domination(sibling)
                # Only trust a full decode, and only once the same UID has
                # decoded `confirm` times in a row, so a stray tag can't win.
                if tag and tag.get("filament"):
                    uid = tag.get("uid")
                    if uid == cand_uid:
                        streak += 1
                    else:
                        cand_uid, streak = uid, 1
                    if streak >= confirm:
                        if uid is not None:
                            self._slot_uid[slot] = uid
                        return tag
                    # decoded but not yet confirmed, already in seen_uids (False)
                    self._dwell(self.scan_interval)
                    continue
                # A miss/no-decode breaks the confirmation streak.
                cand_uid, streak = None, 0
                if tag and tag.get("uid"):
                    seen_uids[tag["uid"]] = True   # read a UID but it didn't decode
                self._dwell(self.scan_interval)
            # Diagnostic: report exactly what the antenna saw so a misread UID
            # (differs from another reader) or a decode-that-never-completes is
            # visible instead of a bare "no tag".
            if seen_uids:
                saw = ", ".join(
                    f"{h}{' (uid-only, no decode)' if v else ''}"
                    for h, v in sorted(seen_uids.items()))
                self.logger.info(
                    f"ACE2 RFID scan: no full decode; saw UID(s) {saw}")
            # Nothing to report -> close the "hold the spool" popup (on success
            # the result notification replaces it instead).
            if detected_shown:
                dismiss_prompt(self.gcode.respond_raw)
            return None
        finally:
            try:
                link.reader_power(False)
            except Exception:
                self.logger.error(
                    "ACE2 RFID scan: reader power-off failed", traceback=traceback.format_exc())
            # Put any sister we retracted to clear the antenna back where it was.
            self._restore_sister()
            if self.probe_restore_identify:
                for s in shared:
                    try:
                        link.set_rfid_enable(s, True)
                    except Exception:
                        self.logger.error(
                            "ACE2 RFID scan: re-enable identify failed",
                                traceback=traceback.format_exc())

    def scan_lane(self, lane_name: str,
                  seconds: Optional[float] = None) -> Optional[dict]:
        """
        Scan the lane's reader for a presented tag and stage it as the next spool id.

        The next lane load consumes it, like the U1 scanner; it is not applied to
        the scanner lane itself.

        :param lane_name: the scanner lane
        :param seconds: how long to scan, defaults to scan_seconds
        :return Optional[dict]: the tag read, or None
        """
        if self.ace2 is None:
            error_str = "ACE2 not available"
            raise self.printer.command_error(error_str)
        slot = self._slot_for_lane(lane_name)
        if slot is None:
            error_str = f"lane {lane_name!r} has no ACE2 reader slot (set lane_slot_map)"
            raise self.printer.command_error(error_str)
        duration = self.scan_seconds if seconds is None else float(seconds)
        tag = self._scan_slot(int(slot), duration, lane_name=lane_name)
        if not tag:
            self.logger.info(f"ACE2 RFID scan: no tag on lane {lane_name} in {duration:.0f}s")
            return None
        lane = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane = self.afc.lanes.get(lane_name)
        slot_info = self._map(tag)
        self.record_tag_read(lane_name, slot_info)
        allow_create = self.auto_create
        if get_auto_spoolman_create is not None and lane is not None:
            try:
                allow_create = get_auto_spoolman_create(lane, self.auto_create)
            except Exception:
                pass
        if self.afc is not None and getattr(self.afc, "spoolman", None):
            try:
                sync_rfid_to_spoolman(
                    self.afc, lane, slot_info, self.logger, "ACE2 RFID scan",
                    allow_create=allow_create, set_next=True)
            except Exception as e:
                self.logger.warning(f"ACE2 RFID scan Spoolman sync failed: {e}")
        # Pop a UI notification (Mainsail/Fluidd action:prompt) with what we read,
        # like the U1 scanner does on a match/create.
        spool = getattr(self.afc, "spool", None) if self.afc is not None else None
        self._notify_scan(slot_info, lane_name,
                          getattr(spool, "next_spool_id", None))
        return tag

    def _spool_details(self, spool_id: Any, slot_info: dict) -> dict:
        """
        Build the richest spool description available for the scan popup.

        Prefers the matched Spoolman spool's record (complete even when the tag
        read only got the UID), falling back to the tag decode.

        :param spool_id: matched Spoolman spool id, if any
        :param slot_info: the decoded tag
        :return dict: brand, material, color, diameter, temps, weight and name
        """
        d = {
            "brand": slot_info.get("brand", ""),
            "material": slot_info.get("material", ""),
            "color": (slot_info.get("color_hex", "") or "").lstrip("#"),
            "diameter": slot_info.get("diameter"),
            "ext": slot_info.get("extruder_temp"),
            "bed": slot_info.get("bed_temp"),
            "weight": slot_info.get("weight_g"),
            "name": "",
        }
        mr = getattr(self.afc, "moonraker", None) if self.afc is not None else None
        if not spool_id or mr is None:
            return d
        try:
            # SpoolmanClient's, not moonraker's: upstream's get_spool is a
            # fire-and-forget queue that takes a callback and returns None,
            # and this needs the record on the next line to build the popup.
            sp = SpoolmanClient(mr).get_spool(spool_id)
        except Exception:
            sp = None
        if not isinstance(sp, dict):
            return d
        fil = sp.get("filament") or {}
        vendor = (fil.get("vendor") or {}).get("name")
        d["name"] = fil.get("name") or d["name"]
        d["brand"] = vendor or d["brand"]
        d["material"] = fil.get("material") or d["material"]
        d["color"] = (fil.get("color_hex") or d["color"] or "").lstrip("#")
        d["diameter"] = fil.get("diameter") or d["diameter"]
        d["ext"] = fil.get("settings_extruder_temp") or d["ext"]
        d["bed"] = fil.get("settings_bed_temp") or d["bed"]
        # Prefer remaining weight (what's actually left on the spool).
        rem = sp.get("remaining_weight")
        d["weight"] = rem if rem is not None else (
            fil.get("weight") or d["weight"])
        return d

    def _notify_scan(self, slot_info: dict, lane_name: str,
                     spool_id: Any) -> None:
        """
        Show a popup and console summary of the scanned spool, like the U1 scanner.

        Enriched from the matched Spoolman spool. Best-effort: the popup or the
        Spoolman lookup never faults the scan.

        :param slot_info: the decoded tag
        :param lane_name: the scanner lane, for the title
        :param spool_id: matched Spoolman spool id, if any
        """
        try:
            d = self._spool_details(spool_id, slot_info)
            color = d["color"]
            # Prefer the matched Spoolman name; fall back to the shared rich
            # "<brand> <material> <sub_type>" builder so an unmatched tag still
            # reads e.g. "Bambu PLA Basic".
            name = d["name"] or build_filament_name(
                d["brand"], d["material"], slot_info.get("sub_type", ""))
            lines = []
            if name:
                lines.append(f"Name: {name}")
            if d["brand"]:
                lines.append(f'Brand: {d["brand"]}')
            if d["material"]:
                lines.append(f'Material: {d["material"]}')
            if color:
                lines.append(f"Color: #{color}")
            if d["diameter"]:
                lines.append(f'Diameter: {d["diameter"]}mm')
            if d["ext"]:
                lines.append(f'Nozzle temp: {d["ext"]}°C')
            if d["bed"]:
                lines.append(f'Bed temp: {d["bed"]}°C')
            if d["weight"]:
                lines.append(f'Remaining: {round(float(d["weight"]))}g')
            if spool_id:
                lines.append(f"Spoolman ID: {spool_id}")
            if not lines:
                lines.append(f'uid: {slot_info.get("uid", "")}')
            title = f"Spool Scanned on {lane_name}" if lane_name else \
                "Spool Scanned"
            body = "\n  ".join(lines)
            self.gcode.respond_info(f"{title}:\n  {body}")
            respond = self.gcode.respond_raw
            respond(f"// action:prompt_begin {title}")
            for pl in lines:
                respond(f"// action:prompt_text {pl}")
            respond("// action:prompt_footer_button "
                    "OK|RESPOND TYPE=command MSG=action:prompt_end|info")
            respond("// action:prompt_show")
            # Auto-dismiss after 10s so it doesn't linger if unattended.
            self.reactor.register_callback(
                lambda e: self.gcode.respond_raw("// action:prompt_end"),
                self.reactor.monotonic() + 10.0)
        except Exception as e:
            self.logger.warning(f"ACE2 RFID scan: notification error: {e}")

    # ── auto-read on insert ───────────────────
    def _on_post_insert(self, lane: "AFCLane") -> None:
        """
        Auto-read the tag after ``afc_ace:post_insert`` fed a new spool to the hub.

        The feed spun the tag past the reader. The read is deferred onto the
        reactor and best-effort, so it never blocks or faults the insert path.

        :param lane: the lane that was inserted
        """
        # RFID off: no automatic insert read.
        if self.disable_rfid:
            return
        # When stage_read is on, the tag is read during staging (stage_probe*),
        # so skip the after-staging read to avoid a redundant reader-power cycle.
        if self.stage_read or not self.read_on_insert:
            return
        name = getattr(lane, "name", None)
        if name is None or not self._rfid_enabled(name):
            return
        self.reactor.register_callback(lambda et, n=name: self._auto_read(n))

    def _apply_untagged_defaults(self, lane: Any) -> None:
        """
        Give a spool with no readable tag AFC's default material and colour.

        On an ACE 2 the insert leaves the lane's filament to this module:
        AFC_ACE clears it and skips the V1 path's defaults, since the tag read
        here is meant to fill it. So every way an insert can end without a
        decoded tag has to land here, or the lane is left blank. Only fields
        still unset are filled, so a remembered spool's values stand.

        :param lane: the lane, or its name
        """
        afc = self.afc
        if afc is None:
            return
        if isinstance(lane, str):
            lane = getattr(afc, "lanes", {}).get(lane)
        elif hasattr(afc, "lanes"):
            lane = afc.lanes.get(getattr(lane, "name", None), lane)
        if lane is None:
            return
        try:
            before = (getattr(lane, "material", None), getattr(lane, "color", None))
            apply_filament_defaults(lane, {}, afc_defaults={
                "default_material_type": getattr(afc, "default_material_type", None),
                "default_color": getattr(afc, "default_color", None)})
            after = (getattr(lane, "material", None), getattr(lane, "color", None))
            if after == before:
                return                  # nothing configured, or nothing unset
            unit = getattr(lane, "unit_obj", None)
            if unit is not None and hasattr(unit, "lane_illuminate_spool"):
                unit.lane_illuminate_spool(lane)
            afc.save_vars()
            self.logger.info(
                f"ACE2 RFID: {lane.name} has no readable tag; AFC defaults applied "
                f"(material {after[0] or '-'}, color {after[1] or '-'})")
        except Exception:
            self.logger.error("ACE2 RFID: applying defaults failed",
                              traceback=traceback.format_exc())

    def _auto_read(self, name: str) -> None:
        """
        Read the lane's tag, retrying a few times to catch the tag as the
        spool settles. Stops on the first successful read.

        :param name: lane name
        """
        for attempt in range(self.read_on_insert_attempts):
            try:
                tag = self.read_lane(name)
            except self.printer.command_error as e:
                self.logger.info(f"ACE2 RFID auto-read {name}: {e}")
                return
            except Exception:
                self.logger.error(
                    f"ACE2 RFID auto-read failed for {name}", traceback=traceback.format_exc())
                tag = None
            if tag:
                if not tag.get("filament"):
                    # A tag, but nothing it says could be decoded.
                    self._apply_untagged_defaults(name)
                return
            if attempt < self.read_on_insert_attempts - 1 and self.read_on_insert_delay:
                self.reactor.pause(
                    self.reactor.monotonic() + self.read_on_insert_delay)
        self.logger.info(
            f"ACE2 RFID auto-read {name}: no tag after {self.read_on_insert_attempts} attempts")
        self._apply_untagged_defaults(name)

    # ── chunked read during staging ────────────────
    def _physical_slot(self, name: str) -> Optional[int]:
        """
        Return the lane's ACE slot.

        Uses the ACE's own map when available (correct for motion and reader),
        else the lane_slot_map override.

        :param name: lane name
        :return Optional[int]: physical slot, or None when unmapped
        """
        sm = getattr(self.ace2, "_slot_map", None)
        if isinstance(sm, dict) and name in sm:
            return sm[name]
        return self._lane_slot.get(name)

    def _reg_index(self, slot: int) -> int:
        """
        Return the MFRC522 chip-select index for a physical slot's reg r/w.

        The reader pair (slot>>1) by default; the slot itself when
        reader_reg_per_slot is set.

        :param slot: physical slot
        :return int: chip-select index
        """
        return int(slot) if self.reader_reg_per_slot else (int(slot) >> 1)

    def _rfid_enabled(self, name: str) -> bool:
        """
        Whether RFID reads run for this lane.

        A configured lane_slot_map is an explicit allow-list; with no map every
        lane the ACE unit knows about is enabled.

        :param name: lane name
        :return bool: True when reads run for the lane
        """
        if self._lane_slot:
            return name in self._lane_slot
        sm = getattr(self.ace2, "_slot_map", None)
        return isinstance(sm, dict) and name in sm

    def _lane_at_slot(self, slot: int) -> Optional["AFCLane"]:
        """
        Return the AFC lane mapped to a physical slot.

        Checks the ACE's slot map, then lane_slot_map.

        :param slot: physical slot
        :return Optional[AFCLane]: the lane, or None
        """
        if self.afc is None or not hasattr(self.afc, "lanes"):
            return None
        sm = getattr(self.ace2, "_slot_map", None)
        maps = [sm] if isinstance(sm, dict) else []
        maps.append(self._lane_slot)
        for m in maps:
            for lname, s in m.items():
                if int(s) == int(slot):
                    lane = self.afc.lanes.get(lname)
                    if lane is not None:
                        return lane
        return None

    def _sibling_present(self, slot: int) -> bool:
        """
        Whether the shared-reader sibling slot has a spool inserted.

        Assumes present when unknown, so dedup still guards against duplicates.

        :param slot: the sibling's physical slot
        :return bool: True when a spool is, or may be, present
        """
        lane = self._lane_at_slot(slot)
        if lane is None:
            return True
        return bool(getattr(lane, "prep_state", True))

    def _is_sibling_tag(self, active_slot: int, sibling_slot: Optional[int],
                        uid: Optional[str]) -> bool:
        """
        Whether ``uid`` belongs to the shared sibling slot, not the active lane.

        True when this session already read that UID on the sibling slot and the
        sibling still holds a spool, so assigning it here would duplicate it.

        :param active_slot: slot being read
        :param sibling_slot: slot sharing the reader, if any
        :param uid: UID hex just read
        :return bool: True when the UID is the sibling's
        """
        if (not self.shared_reader_dedup
                or not uid
                or sibling_slot is None):
            return False
        if not self._sibling_present(sibling_slot):
            return False
        return self._slot_uid.get(sibling_slot) == uid

    def _stage_probe_begin(self, lane: "AFCLane", ctx: dict) -> None:
        """
        Run the scan-and-read when ``afc_ace:stage_probe_begin`` fires.

        Arms the reader, feeds at scan speed while polling, stops on the first
        UID, reads the tag at rest and applies it. Sets ``ctx['fed']`` (net
        distance fed) and ``ctx['initial']`` so the feeder stages the remainder
        at full speed, and ``ctx['done']`` on a successful read.

        :param lane: the lane being staged
        :param ctx: shared staging context, updated in place
        """
        try:
            if not self.stage_read or self.ace2 is None:
                return
            name = getattr(lane, "name", None)
            if name is None:
                return
            # RFID off: stage as a plain feed, no chunk-probe, no reads. Still
            # request the mandatory initial load so a skip_factory_autostage
            # insert reaches the hub (fed in one move, not chunked).
            if self.disable_rfid:
                ctx["initial"] = (self.stage_initial_dist
                                  if self.skip_factory_autostage else 0.0)
                return
            if not self._rfid_enabled(name):
                return
            slot = self._physical_slot(name)
            if slot is None:
                return
            slot = int(slot)
            pair = (slot >> 1) & 0xFF          # per-pair power domain
            link = self._new_link(self._reg_index(slot) & 0xFF,
                                  power_index=pair)
            shared = (pair * 2, pair * 2 + 1)
            sibling = shared[0] if shared[1] == slot else shared[1]
            # Disable identify on the pair: frees the reader and suppresses the
            # firmware autostage for this fresh insert (restore stays optional).
            for s in shared:
                link.set_rfid_enable(s, False)
            # set_rfid_enable is fire-and-forget; a synchronous reader_power(off)
            # makes sure the disable lands before the feed, or the firmware would
            # autostage the insert. _run_stage_scan powers the reader back on.
            link.reader_power(False)
            self._dwell(0.1)
            # Fresh spool in the active slot: forget its old UID so its own new
            # tag is never mistaken for stale history.
            self._slot_uid.pop(slot, None)
            self._probe = {"link": link, "shared": shared, "lane": lane,
                           "slot": slot, "sibling": sibling}
            # Fresh stage: no sister has been retracted, no manual-nudge hint
            # shown yet.
            self._sister_retracted = None
            self._sister_hint_shown = False
            # Stop + suppress the ACE feed assist on these slots for the whole
            # stage read: the read pauses the feed for seconds, and if assist is
            # watching the encoder it trips the stuck-spool error (fast flash).
            self._suppress_assist(shared)
            ctx["active"] = True
            # Mandatory initial load, only when we bypassed the factory initial
            # staging (otherwise the factory already loaded it).
            ctx["initial"] = (self.stage_initial_dist
                              if self.skip_factory_autostage else 0.0)
            # Run the scan-and-read now on a continuous feed; report how far it fed
            # so the feeder stages the remainder at full speed.
            ctx["fed"] = self._run_stage_scan(lane, ctx)
        except Exception:
            self.logger.error(
                "ACE2 RFID: stage_probe_begin failed", traceback=traceback.format_exc())
            self._safe_probe_teardown()
            self._restore_assist()

    def _suppress_assist(self, slots: Any) -> None:
        """
        Stop feed assist on the given slots and suppress the watchdog so it
        stays off until we restore it (mirrors ACE_FEED_ASSIST ENABLE=0).

        :param slots: physical slots to stop assist on
        """
        ace2 = self.ace2
        if ace2 is None:
            return
        self._assist_slots = tuple(slots)
        sup = getattr(ace2, "_assist_suppressed", None)
        active = getattr(ace2, "_feed_assist_active", None)
        conn = getattr(ace2, "_ace", None)
        for s in slots:
            try:
                if isinstance(sup, set):
                    sup.add(s)
                if isinstance(active, set) and s in active:
                    ace2._stop_feed_assist(s)
                elif conn is not None:
                    conn.stop_feed_assist_sync(s)   # force stop; tracking drifted
                self.logger.info(f"ACE2 RFID: feed assist stopped on slot {s}")
            except Exception:
                self.logger.error(
                    f"ACE2 RFID: failed to stop feed assist on slot {s}",
                        traceback=traceback.format_exc())

    def _restore_assist(self) -> None:
        """
        Un-suppress feed assist on the slots we stopped, letting the ACE's
        watchdog reconcile it normally again.
        """
        slots = self._assist_slots
        self._assist_slots = None
        if not slots or self.ace2 is None:
            return
        sup = getattr(self.ace2, "_assist_suppressed", None)
        for s in slots:
            try:
                if isinstance(sup, set):
                    sup.discard(s)
                self.logger.info(f"ACE2 RFID: feed assist restored on slot {s}")
            except Exception:
                self.logger.error(
                    f"ACE2 RFID: failed to restore feed assist on slot {s}",
                        traceback=traceback.format_exc())

    def _probe_excluder(self, active_slot: int,
                        sibling_slot: Optional[int]
                        ) -> Optional[Callable[[str], bool]]:
        """
        Build a ``uid_hex -> bool`` predicate that flags the sibling's tag.

        Passed into read_tag so a neighbour tag is halted, not decoded, and this
        lane's own tag is read instead.

        :param active_slot: slot being read
        :param sibling_slot: slot sharing the reader, if any
        :return Optional[Callable[[str], bool]]: the predicate, or None when
            there is nothing to exclude
        """
        if not self.shared_reader_dedup or sibling_slot is None:
            return None
        return lambda uid: self._is_sibling_tag(active_slot, sibling_slot, uid)

    def _run_stage_scan(self, lane: "AFCLane", ctx: dict) -> float:
        """
        Feed the lane at scan speed while polling, and read its own tag.

        Stops on the first of this lane's tags, reads it at rest (re-centering
        if it coasted past) and applies it. A sister tag dominating the antenna
        is retracted out of the way (auto_tag_adjust). Sets ``ctx['done']`` on a
        successful read and ``ctx['removed']`` if the spool was pulled mid-scan.

        :param lane: the lane being staged
        :param ctx: shared staging context, updated in place
        :return float: net distance fed, less any re-centering retract
        """
        p = self._probe
        if not p:
            return 0.0
        link, slot, sibling = p["link"], p["slot"], p["sibling"]
        conn = getattr(self.ace2, "_ace", None)
        if conn is None:
            return 0.0
        speed = float(self.stage_scan_speed)
        # Scan window, bounded to the net load so a tagless spool never overshoots
        # the hub (the remainder is what's left of initial + dist_hub).
        initial = float(ctx.get("initial") or 0.0)
        total = initial + float(getattr(lane, "dist_hub", 0.0) or 0.0)
        scan_dist = float(self.stage_scan_dist)
        if total > 0:
            scan_dist = min(scan_dist, total)
        if scan_dist <= 0:
            return 0.0
        sib_lane = self._lane_at_slot(sibling)
        sib_has_spool = (sib_lane is not None
                         and bool(getattr(sib_lane, "prep_state", False)))
        # Host owns the reader for the whole scan, power is held across the feed
        # (the continuous scan keeps it powered and polls throughout).
        link.reader_power(True)
        self._dwell(0.1)
        try:
            self.ace2._wait_for_ace_ready(timeout=self.read_reg_timeout + 20.0)
        except Exception:
            pass
        # Probe the antenna once before our spool spins: any tag seen now is the
        # sister's, parked. Retract a movable sister, else exclude that UID for
        # the scan (dedup alone misses a sister staged in an earlier session).
        baseline = None
        blocked_sib = None            # a present sibling we couldn't move off the coil
        if sib_has_spool:
            parked = None
            for _ in range(3):                       # a couple retries, read is
                parked = self._probe_uid(link)   # reliable for a still tag
                if parked is not None:
                    break
                self._dwell(0.05)
            if parked is not None:
                self.logger.info(
                    f"ACE2 RFID: stage scan slot {slot}, tag {parked} "
                    f"parked on the shared reader (sibling slot {sibling}); "
                    f"clearing it before the read")
                # Unmovable: exclude its tag and try anyway; the user is only
                # asked to move it if the read comes up empty.
                self._maybe_retract_sister(sibling)
                if self._sister_retracted is None:   # couldn't move it
                    baseline = parked
                    blocked_sib = sibling
        t0 = self.reactor.monotonic()
        try:
            conn.feed_filament(slot, scan_dist, speed)
        except Exception:
            self.logger.error("ACE2 RFID: stage scan feed failed", traceback=traceback.format_exc())
            self._reader_power_off(link)
            return 0.0
        eff = min(max(speed, 1.0), 90.0)
        expected = scan_dist / eff
        deadline = self.reactor.monotonic() + expected * 1.5 + 15.0
        slot_moving = getattr(self.ace2, "_slot_is_moving", None)
        slot_empty = getattr(self.ace2, "_slot_reports_empty", None)
        get_status = getattr(conn, "get_status", None)
        detected = None
        net_fed = 0.0
        started = False
        idle = 0
        while self.reactor.monotonic() < deadline:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
            # Spool pulled back out mid-scan (fumbled insert), stop and bail so
            # the feeder resets prep instead of ramming an empty slot.
            if callable(slot_empty):
                try:
                    if slot_empty(slot):
                        try:
                            conn.stop_feed_filament(slot)
                        except Exception:
                            pass
                        ctx["removed"] = True
                        self._reader_power_off(link)
                        return min(scan_dist,
                                   (self.reactor.monotonic() - t0) * eff)
                except Exception:
                    pass
            uidhex = self._probe_uid(link)
            if uidhex is not None:
                # The stationary parked sibling tag captured at baseline, never
                # this lane's; keep feeding so our own tag sweeps in.
                if baseline is not None and uidhex == baseline:
                    continue
                # A neighbour's tag on the antenna: retract the idle sibling and
                # keep feeding so our tag sweeps in. Restored in _stage_probe_end.
                if self._is_sibling_tag(slot, sibling, uidhex):
                    self.logger.info(
                        f"ACE2 RFID: stage scan slot {slot} saw sibling slot {sibling}'s tag (uid="
                        f"{uidhex}), clearing it and continuing")
                    self._maybe_retract_sister(sibling)
                    if self._sister_retracted is None:
                        blocked_sib = sibling
                    continue
                detected = uidhex
                net_fed = min(scan_dist, (self.reactor.monotonic() - t0) * eff)
                try:
                    conn.stop_feed_filament(slot)
                except Exception as e:
                    self.logger.info(f"ACE2 RFID: stage scan stop_feed err: {e}")
                break
            # End the scan when the feed finishes with no tag seen.
            if callable(slot_moving) and callable(get_status):
                try:
                    moving = slot_moving(get_status(timeout=2.0), slot)
                except Exception:
                    moving = True
                if moving:
                    started = True
                    idle = 0
                elif started:
                    idle += 1
                    if idle >= 2:
                        net_fed = scan_dist
                        break
        if not detected and net_fed == 0.0:
            net_fed = scan_dist
        # Parked read (+ re-center sweep) now that we've stopped on our own tag.
        if detected:
            tag, extra_retract = self._parked_read(link, slot, sibling, baseline)
            net_fed -= extra_retract
            if tag and tag.get("filament"):
                uid = tag.get("uid")
                if uid is not None:
                    self._slot_uid[int(slot)] = uid
                ctx["done"] = True
                # Release the reader before the feeder stages the remainder.
                self._safe_probe_teardown()
                try:
                    if self.afc is not None and hasattr(self.afc, "lanes"):
                        lane = self.afc.lanes.get(getattr(lane, "name", None), lane)
                    self.apply_to_lane(lane, tag)
                    f = tag.get("filament") or {}
                    self.logger.info(
                        f"ACE2 RFID: read {getattr(lane, 'name', '?')} during staging, uid="
                        f"{tag.get('uid', '')} type={tag.get('tag_type', '')} {f.get('type', '')}")
                except Exception:
                    self.logger.error(
                        "ACE2 RFID: apply after stage read failed",
                            traceback=traceback.format_exc())
            else:
                self.logger.info(
                    f"ACE2 RFID: stage scan slot {slot} detected uid={detected} but the tag did "
                    f"not decode, staging without RFID")
                self._reader_power_off(link)
                self._sister_blocked_hint(lane, blocked_sib)
                self._apply_untagged_defaults(lane)
        else:
            self.logger.info(
                f"ACE2 RFID: stage scan slot {slot}, no tag seen in {scan_dist:.0f}mm; staging "
                f"without RFID")
            self._reader_power_off(link)
            self._sister_blocked_hint(lane, blocked_sib)
            self._apply_untagged_defaults(lane)
        return max(0.0, net_fed)

    def _parked_read(self, link: Any, slot: int, sibling: Optional[int],
                     baseline: Optional[str] = None
                     ) -> Tuple[Optional[dict], float]:
        """
        Read the just-stopped tag at rest.

        The tag coasts past the antenna after the stop, so if the first reads
        miss, small retracts step it back into the read zone, reading at each
        step. The retract is fed back when the remainder stages.

        :param link: register link to the reader
        :param slot: physical slot being read
        :param sibling: slot sharing the reader, if any
        :param baseline: the parked sibling UID, never returned as this lane's
        :return Tuple[Optional[dict], float]: (tag or None, total mm retracted)
        """
        conn = getattr(self.ace2, "_ace", None)
        sib_excluder = self._probe_excluder(slot, sibling)

        def excluder(u: str) -> bool:
            """
            Return true when a read UID should be ignored, either the parked
            baseline UID or a UID the stationary sibling lane already owns.

            :param u: UID read from the tag being evaluated
            :return bool: True when the UID should be excluded from this lane's read
            """
            if baseline is not None and u == baseline:
                return True
            return bool(sib_excluder and sib_excluder(u))

        def _try() -> Optional[dict]:
            """
            One parked-read attempt; errors are logged and become None.

            :return Optional[dict]: decoded tag or None
            """
            try:
                t = self._read_tag(link, is_excluded=excluder)
            except Exception:
                self.logger.error("ACE2 RFID: parked read error", traceback=traceback.format_exc())
                return None
            return t if (t and t.get("filament")) else None

        for _ in range(max(1, self.stage_read_hold_attempts)):
            tag = _try()
            if tag:
                return tag, 0.0
            self._dwell(0.12)
        retracted = 0.0
        step = max(0.5, float(self.stage_recenter_step))
        rspeed = float(getattr(self.ace2, "retract_speed", self.stage_scan_speed))
        while retracted + 1e-6 < self.stage_recenter_max and conn is not None:
            try:
                conn.unwind_filament(slot, step, rspeed)
            except Exception:
                self.logger.error(
                    "ACE2 RFID: re-center retract failed", traceback=traceback.format_exc())
                break
            retracted += step
            self._dwell(0.15)
            tag = _try()
            if tag:
                self.logger.info(f"ACE2 RFID: re-centered tag after {retracted:.0f}mm retract")
                return tag, retracted
        return None, retracted

    def _stage_probe_end(self, lane: "AFCLane", ctx: dict) -> None:
        """
        Tear down after ``afc_ace:stage_probe_end`` (the whole staging feed).

        Ensures the reader is down and puts a retracted sister back, then
        restores feed assist, which stays off through the remaining feed.

        :param lane: the lane that was staged (unused)
        :param ctx: shared staging context (unused)
        """
        self._safe_probe_teardown()
        # Put any sister we retracted to clear the antenna back where it was
        # (re-fed the same distance) before restoring assist.
        self._restore_sister()
        self._restore_assist()

    def _is_printing(self) -> bool:
        """
        Whether AFC reports a print in progress (best-effort).

        Gates the sister retract: never move loaded filament mid-print.

        :return bool: True while printing
        """
        try:
            return bool(self.afc.function.is_printing())
        except Exception:
            return False

    def _maybe_retract_sister(self, sibling_slot: Optional[int]) -> str:
        """
        Retract the shared-reader sister spool so its tag leaves the antenna.

        Only when enabled and the sibling is idle and hub-staged: never one
        loaded to the toolhead, and never while printing. _restore_sister feeds
        it back the same distance.

        :param sibling_slot: the sister's physical slot, if any
        :return str: "retracted" (moved now or earlier this stage) or "blocked"
            (not movable, so the user should nudge it by hand)
        """
        if sibling_slot is None:
            return "blocked"
        if self._sister_retracted is not None:
            return "retracted"          # already cleared it this stage
        if not self.auto_tag_adjust or self.ace2 is None:
            return "blocked"            # feature off → manual nudge
        lane = self._lane_at_slot(sibling_slot)
        if lane is None:
            return "blocked"
        if getattr(lane, "tool_loaded", False):
            return "blocked"            # loaded to toolhead, never move
        if not getattr(lane, "loaded_to_hub", False):
            return "blocked"            # not staged at hub, don't guess its pos
        if self._is_printing():
            return "blocked"            # print in progress, never move
        dist = float(self.auto_tag_adjust_dist)
        speed = float(getattr(self.ace2, "feed_speed", 80.0))
        conn = getattr(self.ace2, "_ace", None)
        if conn is None:
            return "blocked"
        try:
            conn.unwind_filament(sibling_slot, dist, speed)
            self.ace2._wait_for_feed_complete(sibling_slot, dist, speed)
        except Exception:
            self.logger.error(
                f"ACE2 RFID: sister retract on slot {sibling_slot} failed",
                    traceback=traceback.format_exc())
            return "blocked"
        self._sister_retracted = (sibling_slot, dist, speed)
        self.logger.info(
            f"ACE2 RFID: retracted sister slot {sibling_slot} by {dist:.0f}mm to clear its tag off "
            f"the shared antenna")
        return "retracted"

    def _restore_sister(self) -> None:
        """
        Re-feed a sister spool we retracted during the stage back to its
        staged position (mirrors the retract). Best-effort, a failure just
        leaves the sibling slightly short, which the next load corrects.
        """
        info = self._sister_retracted
        self._sister_retracted = None
        if not info or self.ace2 is None:
            return
        slot, dist, speed = info
        conn = getattr(self.ace2, "_ace", None)
        if conn is None:
            return
        try:
            conn.feed_filament(slot, dist, speed)
            self.ace2._wait_for_feed_complete(slot, dist, speed)
            self.logger.info(
                f"ACE2 RFID: restored sister slot {slot} (+{dist:.0f}mm) after stage read")
        except Exception:
            self.logger.error(
                f"ACE2 RFID: sister restore on slot {slot} failed",
                    traceback=traceback.format_exc())

    def _handle_sister_domination(self, sibling_slot: Optional[int]) -> None:
        """
        Clear a sibling tag that blocks the read, or ask the user to.

        Retracts it when allowed; otherwise tells the user once per stage to
        nudge that spool by hand. Used by the manual scanner; the stage read
        uses _sister_blocked_hint after a failed read instead.

        :param sibling_slot: the sister's physical slot, if any
        """
        if self._maybe_retract_sister(sibling_slot) != "blocked":
            return
        if self._sister_hint_shown:
            return
        self._sister_hint_shown = True
        lane = self._lane_at_slot(sibling_slot)
        lname = getattr(lane, "name", None) or (f"slot {sibling_slot}")
        self.gcode.respond_info(
            f"AFC ACE2: the tag on lane {lname} is sitting on the shared reader and blocking this "
                f"read. Give that spool about a quarter turn by hand to move its tag off the "
                f"reader, then it should read.")

    def _sister_blocked_hint(self, active_lane: "AFCLane",
                             sibling_slot: Optional[int]) -> None:
        """
        Tell the user a parked sibling probably blocked an empty stage read.

        Once per stage, asks them to move that sister spool and re-stage, or set
        the spool id by hand. No-op without a blocking sibling.

        :param active_lane: the lane whose read failed
        :param sibling_slot: the sibling that could not be moved, if any
        """
        if sibling_slot is None or self._sister_hint_shown:
            return
        self._sister_hint_shown = True
        sib_lane = self._lane_at_slot(sibling_slot)
        sib_name = getattr(sib_lane, "name", None) or (f"slot {sibling_slot}")
        act = getattr(active_lane, "name", None) or "this lane"
        self.gcode.respond_info(
            f"AFC ACE2: couldn't read {act}'s RFID, lane {sib_name}'s spool is on the shared "
                f"reader blocking it. Manually move lane {sib_name}'s spool a little and re-stage "
                f"{act}, or set {act}'s spool id by hand.")

    def _reader_power_off(self, link: Any) -> None:
        """
        Power the reader down without tearing the probe down.

        Best-effort: a wedged power-off must not add another hard failure.

        :param link: register link to the reader
        """
        try:
            link.reader_power(False)
        except Exception:
            self.logger.error(
                "ACE2 RFID: stage read reader power-off failed", traceback=traceback.format_exc())

    def _safe_probe_teardown(self) -> None:
        """
        Release the active probe: settle, power the reader off, and restore
        firmware identify when configured. Best-effort and instrumented.
        """
        p = self._probe
        self._probe = None
        if not p:
            return
        link, shared = p["link"], p["shared"]
        # Instrumented so the klippy.log shows exactly which teardown step ran
        # last before a lock-up. Each step is logged before and after.
        if self.probe_settle:
            self._dwell(self.probe_settle)
        if self.probe_power_off:
            self.logger.info("ACE2 RFID teardown: reader_power(off)...")
            try:
                link.reader_power(False)
                self.logger.info("ACE2 RFID teardown: reader_power(off) done")
            except Exception:
                self.logger.error(
                    "ACE2 RFID: probe power-off failed", traceback=traceback.format_exc())
        if self.probe_restore_identify:
            for s in shared:
                self.logger.info(f"ACE2 RFID teardown: set_rfid_enable({s},on)...")
                try:
                    link.set_rfid_enable(s, True)
                    self.logger.info(f"ACE2 RFID teardown: set_rfid_enable({s},on) done")
                except Exception:
                    self.logger.error(
                        "ACE2 RFID: probe re-enable identify failed",
                            traceback=traceback.format_exc())
        else:
            self.logger.info("ACE2 RFID teardown: identify NOT restored (config)")

    # ── gcode ───────────────────────
    def cmd_ACE_RFID_READ(self, gcmd: "GCodeCommand") -> None:
        """
        Read the tag for a lane, or a raw slot for diagnostics, and report it.

        Responds with the decoded uid, type, brand, material and colour. REG=
        overrides the MFRC522 chip-select index. Reader glitches are caught so a
        bad read never shuts Klipper down.

        Usage
        -------
        `ACE_RFID_READ [LANE=<name>] [SLOT=<n>] [REG=<n>]`

        Example
        -------
        ```
        ACE_RFID_READ LANE=lane1
        ```
        """
        try:
            lane_name = gcmd.get("LANE", None)
            # REG= overrides the MFRC522 chip-select index for diagnostics (map
            # which reg index reads which physical slot's tag). -1 = no override.
            reg_slot = gcmd.get_int("REG", -1)
            reg_slot = None if reg_slot < 0 else reg_slot
            if lane_name is not None:
                tag = self.read_lane(lane_name)
            else:
                slot = gcmd.get_int("SLOT", 0)
                tag = self.read_slot(slot, reg_slot=reg_slot)
        except self.printer.command_error:
            raise                                 # normal user error (no shutdown)
        except Exception as e:
            # A serial timeout or reader glitch must never shut Klipper down.
            self.logger.error("ACE2 RFID read failed", traceback=traceback.format_exc())
            gcmd.respond_info(f"ACE2 RFID: read error: {e}")
            return
        if not tag:
            gcmd.respond_info("ACE2 RFID: no tag found")
            return
        f = tag.get("filament") or {}
        color = ""
        if f.get("color_argb") is not None:
            color = f'{(f.get("color_argb") or 0) & 0xFFFFFF:06x}'
        gcmd.respond_info(
            f'ACE2 RFID: uid={tag.get("uid", "")} type={tag.get("tag_type", "")} '
            f'brand={f.get("manufacturer", "")} material={f.get("type", "")} '
            f'color={color}')

    def cmd_ACE_RFID_BLOCKS(self, gcmd: "GCodeCommand") -> None:
        """
        Dump raw MIFARE blocks of a Bambu tag to confirm dual colour.

        Reads block 5 (primary colour) and block 16 (colour count @258, second
        colour @260 in reversed ABGR) by default. Named to avoid the V1 unit's
        firmware ACE_RFID_DUMP.

        Usage
        -------
        `ACE_RFID_BLOCKS [LANE=<name>] [SLOT=<n>] [BLOCKS=5,16]`

        Example
        -------
        ```
        ACE_RFID_BLOCKS LANE=lane1 BLOCKS=5,16
        ```
        """
        lane_name = gcmd.get("LANE", None)
        blocks_str = gcmd.get("BLOCKS", "5,16")
        try:
            blocks = tuple(int(b) for b in blocks_str.replace(" ", "").split(",")
                           if b != "")
        except ValueError:
            gcmd.respond_info(f"ACE2 RFID DUMP: bad BLOCKS={blocks_str!r}")
            return
        try:
            if lane_name is not None:
                slot = self._slot_for_lane(lane_name)
                if slot is None:
                    error_str = (f"lane {lane_name!r} has no ACE2 reader slot "
                                 f"(set lane_slot_map)")
                    raise self.printer.command_error(error_str)
            else:
                slot = gcmd.get_int("SLOT", 0)
            tag = self.read_slot(int(slot), dump_blocks=blocks)
        except self.printer.command_error:
            raise
        except Exception as e:
            self.logger.error("ACE2 RFID dump failed", traceback=traceback.format_exc())
            gcmd.respond_info(f"ACE2 RFID DUMP: error: {e}")
            return
        if not tag:
            gcmd.respond_info("ACE2 RFID DUMP: no tag found")
            return
        raw = tag.get("raw_blocks") or {}
        lines = [f'ACE2 RFID DUMP: uid={tag.get("uid", "")} type={tag.get("tag_type", "")}']
        b5 = bytes.fromhex(raw[5]) if raw.get(5) else b""
        b16 = bytes.fromhex(raw[16]) if raw.get(16) else b""
        if len(b5) >= 4:
            lines.append(f"  block5 primary -> #{b5[0]:02x}{b5[1]:02x}{b5[2]:02x} (a={b5[3]:02x})")
        if len(b16) >= 8:
            count = b16[2] | (b16[3] << 8)
            a2, bb2, g2, r2 = b16[4], b16[5], b16[6], b16[7]
            lines.append(f"  block16 fmt={b16[0] | (b16[1] << 8):04x} color_count={count} "
                f"second(ABGR bytes)={a2:02x}{bb2:02x}{g2:02x}{r2:02x} -> #{r2:02x}{g2:02x}"
                f"{bb2:02x}")
            verdict = (f"DUAL-COLOR (count={count})" if count >= 2
                       else f"single color (count={count})")
            lines.append(f"  VERDICT: {verdict}")
        for b in blocks:
            lines.append(f'  raw block {b}: {raw.get(b, "(not read)")}')
        f = tag.get("filament") or {}
        if f.get("colors_argb"):
            lines.append("  decoded colors: "
                         + ", ".join(f"#{c & 0xFFFFFF:06x}"
                                     for c in f["colors_argb"]))
        gcmd.respond_info("\n".join(lines))

    #: What a rescan leaves between the tip and the slot when it winds back, so
    #: the slot never reads empty under it.
    RESCAN_MARGIN_MM = 50.0
    #: The parked read's search around the aimed tag position: step size and
    #: how far either way it looks (mm of filament).
    RESCAN_SEEK_STEP_MM = 5.0
    RESCAN_SEEK_SPAN_MM = 30.0

    def rescan_lane(self, lane_name: str, gcmd: "GCodeCommand") -> None:
        """
        ACE_RFID_RESCAN on an ACE 2 lane: turn the spool past the reader
        and apply the tag it reads to the lane.

        Every move runs to completion, so the lane's position is known exactly
        and it ends where it started; the ACE takes a second or two to start a
        move, so a move stopped part-way cannot be measured by time.

        1. Wind the filament back DIST (default 500, never more than staging
           fed less RESCAN_MARGIN_MM).
        2. Feed it all back at SPEED (default stage_scan_speed) while polling
           the reader, noting where in the move this lane's tag went past. The
           lane is now where it started.
        3. Wind back to where the tag was seen, read it at rest (searching
           RESCAN_SEEK_SPAN_MM either way in RESCAN_SEEK_STEP_MM steps), feed
           back what was wound, and apply the tag.

        The shared-reader sibling's known tag is passed over. Refused while
        printing, on a lane loaded to the toolhead, on a lane that is not
        staged, and while the ACE is loading or unloading.

        :param lane_name: the lane to rescan
        :param gcmd: the command, for DIST=/SPEED= and the console
        """
        lane, slot = self._rescan_target(lane_name, gcmd.error)
        dist = self._rescan_dist(lane, lane_name,
                                 gcmd.get_float("DIST", 500.0, minval=10.0),
                                 gcmd.error)
        speed = gcmd.get_float("SPEED", float(self.stage_scan_speed),
                               minval=1.0, maxval=140.0)
        gcmd.respond_info(f"ACE2 RFID: winding {lane_name} back {dist:.0f}mm "
                          f"and feeding it past the reader...")
        tag, seen = self._rescan_run(lane_name, slot, dist, speed, gcmd)
        if not tag:
            gcmd.respond_info(
                f"ACE2 RFID: no tag read on {lane_name}"
                + ("" if seen is not None else " (none came past the reader)")
                + "; the lane is back where it was")
            return
        uid = tag.get("uid")
        if uid is not None:
            self._slot_uid[slot] = uid
        self.apply_to_lane(lane, tag)

    def _rescan_target(self, lane_name: str,
                       fail: Callable[[str], Exception]) -> Tuple[Any, int]:
        """
        The lane and slot a rescan or a staged tag write turns, or ``fail``
        raised with why it may not.

        :param lane_name: the lane to turn
        :param fail: builds the exception to raise (gcmd.error, StageError)
        :return tuple: (lane, physical slot)
        """
        ace2 = self.ace2
        if ace2 is None:
            error_str = "ACE2 not available"
            raise fail(error_str)
        slot = self._physical_slot(lane_name)
        if slot is None:
            error_str = f"lane {lane_name!r} has no ACE2 slot (set lane_slot_map)"
            raise fail(error_str)
        if self.disable_rfid or not self._rfid_enabled(lane_name):
            error_str = f"ACE2 RFID is off for {lane_name}"
            raise fail(error_str)
        lane = (getattr(self.afc, "lanes", None) or {}).get(lane_name)
        if lane is None:
            error_str = f"{lane_name} is not an AFC lane"
            raise fail(error_str)
        if getattr(lane, "tool_loaded", False):
            error_str = (f"{lane_name} is loaded to the toolhead; unload it "
                         f"before its spool is turned past the reader")
            raise fail(error_str)
        if self._is_printing():
            error_str = ("turning the spool past the reader moves filament; "
                         "not while printing")
            raise fail(error_str)
        if not getattr(lane, "loaded_to_hub", False):
            error_str = (f"{lane_name} is not staged, so there is no filament "
                         f"to turn the spool with")
            raise fail(error_str)
        conn = getattr(ace2, "_ace", None)
        if conn is None or not getattr(conn, "connected", False):
            error_str = "ACE serial not connected"
            raise fail(error_str)
        if getattr(ace2, "_operation_active", False):
            error_str = ("the ACE is loading or unloading; try again when it "
                         "is done")
            raise fail(error_str)
        return lane, int(slot)

    def _rescan_dist(self, lane: Any, lane_name: str, dist: float,
                     fail: Callable[[str], Exception]) -> float:
        """
        How far a rescan winds back: ``dist``, never more than staging fed
        less RESCAN_MARGIN_MM.

        :param lane: the lane
        :param lane_name: its name, for the message
        :param dist: the distance wanted, mm
        :param fail: builds the exception to raise when too little is staged
        :return float: the distance to wind back, mm
        """
        staged = float(getattr(lane, "dist_hub", 0.0) or 0.0)
        if self.skip_factory_autostage:
            staged += float(self.stage_initial_dist)
        if staged > 0:
            dist = min(dist, staged - self.RESCAN_MARGIN_MM)
        if dist < 10.0:
            error_str = (f"{lane_name} has too little filament staged to turn "
                         f"its spool past the reader")
            raise fail(error_str)
        return dist

    def _rescan_run(self, lane_name: str, slot: int, dist: float, speed: float,
                    gcmd: Any,
                    hold: Optional[Callable[[Optional[Callable[[str], bool]]],
                                            Any]] = None
                    ) -> Tuple[Optional[dict], Optional[float]]:
        """
        The rescan's motion: wind back, feed past the reader noting where
        the tag went by, wind back to it and read it at rest, feed back.

        With ``hold`` (a staged tag write), the tag at rest only has to answer,
        not decode (a blank tag has nothing to decode), and hold(excluded)
        runs right there, the reader handed over, before the filament goes
        back.

        :param lane_name: the lane, for the log
        :param slot: its physical slot
        :param dist: how far to wind back, mm
        :param speed: the feed speed past the reader, mm/s
        :param gcmd: gives .error for a move that does not complete
        :param hold: run with the tag at rest in the field
        :return tuple: (the tag read at rest, or None; mm into the feed where
            the tag went past, or None)
        """
        ace2 = self.ace2
        pair = (slot >> 1) * 2
        sibling = pair if pair + 1 == slot else pair + 1
        excluded = self._probe_excluder(slot, sibling)
        link = self._new_link(self._reg_index(slot) & 0xFF,
                              power_index=pair >> 1)

        def log(msg: str) -> None:
            """
            Log one rescan step to AFC.log.

            :param msg: step text
            """
            self.logger.info(f"ACE2 RFID rescan {lane_name}: {msg}")

        # Keep the heartbeat from re-staging, runout-checking or re-asserting
        # feed assist while the lane is wound back, and assist off on the pair.
        ace2._operation_active = True
        self._suppress_assist((pair, pair + 1))
        pos = 0.0                         # mm from where the lane started
        tag = None
        seen = None
        hold_error: Optional[Exception] = None
        try:
            self._rescan_move(slot, -dist, gcmd)
            pos = -dist
            # Host owns the reader for the pass and the read (firmware identify
            # off on both slots of the pair, then power it ourselves).
            for s in (pair, pair + 1):
                link.set_rfid_enable(s, False)
            self._dwell(0.1)
            link.reader_power(True)
            self._dwell(0.1)
            try:
                seen = self._rescan_pass(slot, link, dist, speed, excluded, log,
                                         gcmd)
                pos = 0.0
                if seen is not None:
                    back = dist - seen
                    log(f"tag went past ~{seen:.0f}mm into the {dist:.0f}mm "
                        f"feed; winding back {back:.0f}mm to it")
                    if back > 0.5:
                        self._rescan_move(slot, -back, gcmd)
                        pos = -back
                    tag, pos = self._rescan_seek(slot, link, pos, excluded, log,
                                                 gcmd,
                                                 need_decode=hold is None)
                    if tag and hold is not None:
                        log(f"holding tag {tag.get('uid')} at the reader for "
                            f"the write")
                        try:
                            hold(excluded)
                        except Exception as e:
                            # Raised once the lane is back, so a failed
                            # write still leaves it where it started.
                            hold_error = e
            finally:
                self._reader_power_off(link)
                if self.probe_restore_identify:
                    for s in (pair, pair + 1):
                        try:
                            link.set_rfid_enable(s, True)
                        except Exception:
                            pass
            if pos < -0.5:
                self._rescan_move(slot, -pos, gcmd)
                pos = 0.0
        finally:
            ace2._operation_active = False
            self._restore_assist()
        if hold_error is not None:
            raise hold_error
        return tag, seen

    def _stage_write_around(self, lane_name: str, slot: int,
                            body: Callable[..., Any]) -> None:
        """
        AFC_RFID_WRITE / AFC_RFID_ENROLL LANE= on an ACE 2 lane: run the
        rescan's motion and write the tag where it comes to rest, then put the
        lane back where it was. Runs on the reactor.

        :param lane_name: the lane whose spool carries the tag
        :param slot: the slot of the reader the write was sent to
        :param body: the write, called with the sibling excluder
        :raises StageError: when the spool must not or cannot be turned
        """
        lane, lane_slot = self._rescan_target(lane_name, StageError)
        if lane_slot != slot:
            error_str = (f"{lane_name} is on slot {lane_slot}; write it "
                         f"with READER=ace2:slot{lane_slot}")
            raise StageError(error_str)
        dist = self._rescan_dist(lane, lane_name, 500.0, StageError)
        tag, seen = self._rescan_run(lane_name, slot, dist,
                                     float(self.stage_scan_speed),
                                     _StageCmd(), hold=body)
        if not tag:
            error_str = (f"no tag on {lane_name} "
                         + ("came to rest at the reader" if seen is not None
                            else "came past the reader")
                         + "; the lane is back where it was")
            raise StageError(error_str)

    def apply_written_tag(self, lane_name: str, tag: dict) -> None:
        """
        Give a lane the tag AFC_RFID_WRITE / AFC_RFID_ENROLL just wrote to
        its spool, as a rescan would, and note its UID for the slot.

        :param lane_name: the lane whose spool carries the tag
        :param tag: the written tag, shaped like a read
        """
        lane = (getattr(self.afc, "lanes", None) or {}).get(lane_name)
        if lane is None:
            error_str = f"{lane_name} is not an AFC lane"
            raise RuntimeError(error_str)
        slot = self._physical_slot(lane_name)
        if slot is not None and tag.get("uid") is not None:
            self._slot_uid[int(slot)] = tag["uid"]
        self.apply_to_lane(lane, tag)

    def _rescan_move(self, slot: int, mm: float, gcmd: "GCodeCommand",
                     speed: Optional[float] = None,
                     poll: Optional[Callable[[], None]] = None
                     ) -> Tuple[float, float]:
        """
        Move a rescan lane ``mm`` (negative winds back) and wait until the
        ACE reports the move started and finished, calling ``poll`` between
        status reads. A move that never starts or never ends leaves the lane's
        position unknown, so the rescan stops there and says so rather than
        guess.

        :param slot: physical ACE2 slot
        :param mm: distance; negative winds back, positive feeds
        :param gcmd: the command, for the error
        :param speed: mm/s; defaults to the unit's retract or feed speed
        :param poll: called on every pass of the wait (the reader poll)
        :return tuple: when the ACE was first seen moving, and first seen
            stopped again (reactor time)
        """
        ace2 = self.ace2
        conn = ace2._ace
        if speed is None:
            speed = float(getattr(ace2, "retract_speed" if mm < 0
                                  else "feed_speed", 80.0))
        send = conn.unwind_filament if mm < 0 else conn.feed_filament
        dist = abs(mm)
        what = "wind back" if mm < 0 else "feed"
        try:
            ace2._wait_for_ace_ready(timeout=self.read_reg_timeout + 20.0)
            send(slot, dist, speed)
        except Exception as e:
            error_str = f"ACE2 RFID: the {dist:.0f}mm {what} was refused: {e}"
            raise gcmd.error(error_str)
        now = self.reactor.monotonic
        deadline = now() + dist / min(max(speed, 1.0), 90.0) * 1.5 + 15.0
        # The ACE takes a second or two to start a move.
        depart = now() + float(getattr(ace2, "feed_departure_timeout", 3.0)) + 3.0
        # A move cannot end sooner than this, so a busy blip before it starts
        # (the unit finishing the previous command) is not taken for the move.
        shortest = 0.5 * dist / min(max(speed, 1.0), 140.0)
        t_start = t_end = None
        idle = 0
        while now() < deadline:
            self.reactor.pause(now() + 0.05)
            if poll is not None:
                poll()
            try:
                moving = ace2._slot_is_moving(conn.get_status(timeout=2.0), slot)
            except Exception:
                continue
            t = now()
            if moving:
                if t_start is None:
                    t_start = t
                t_end, idle = None, 0
            elif t_start is not None:
                idle += 1
                if t_end is None:
                    t_end = t
                if idle >= 2:
                    if t_end - t_start >= shortest or shortest < 1.0:
                        return t_start, t_end
                    t_start = t_end = None
                    idle = 0
            elif t > depart:
                # A short move can start and end between two status reads.
                if dist / min(max(speed, 1.0), 90.0) <= 2.0:
                    return t, t
                break
        error_str = (f"ACE2 RFID: the {dist:.0f}mm {what} on slot {slot} was "
                     f"not seen to run to completion, so the lane's position "
                     f"is unknown; check it before loading")
        raise gcmd.error(error_str)

    def _rescan_pass(self, slot: int, link: Any, dist: float, speed: float,
                     excluded: Optional[Callable[[str], bool]],
                     log: Callable[[str], None],
                     gcmd: "GCodeCommand") -> Optional[float]:
        """
        Feed ``dist`` at ``speed`` to completion while polling the reader,
        and say where in the move this slot's tag last went past.

        The position is the middle of the last run of sightings, as a share of
        the time the ACE reported the move running, times ``dist``: both ends
        are seen the same way, so the ACE's start-up delay drops out.

        :param slot: physical slot
        :param link: register link to the reader
        :param dist: how far to feed, mm
        :param speed: feed speed, mm/s
        :param excluded: predicate for UIDs to ignore (the sibling's)
        :param log: step logger
        :param gcmd: gives .error for a move that does not complete
        :return Optional[float]: mm into the move where the tag was last seen,
            or None when no tag of this slot went past
        """
        sightings: List[float] = []

        def poll() -> None:
            """
            Note the time whenever this slot's tag is in the reader's field.
            """
            uidhex = self._probe_uid(link)
            if uidhex is not None and not (excluded and excluded(uidhex)):
                sightings.append(self.reactor.monotonic())

        t_start, t_end = self._rescan_move(slot, dist, gcmd, speed=speed,
                                           poll=poll)
        sightings = [t for t in sightings if t_start <= t <= t_end]
        if not sightings:
            log(f"no tag went past in {dist:.0f}mm")
            return None
        # The last run of sightings: consecutive polls are well under a second
        # apart, a new pass of the tag is a revolution later.
        last = first = sightings[-1]
        for t in reversed(sightings[:-1]):
            if first - t > 1.0:
                break
            first = t
        run = max(t_end - t_start, 1e-3)
        seen = ((first + last) / 2.0 - t_start) / run * dist
        return max(0.0, min(dist, seen))

    def _rescan_seek(self, slot: int, link: Any, pos: float,
                     excluded: Optional[Callable[[str], bool]],
                     log: Callable[[str], None],
                     gcmd: "GCodeCommand",
                     need_decode: bool = True) -> Tuple[Optional[dict], float]:
        """
        Read the tag at rest where the pass saw it, then step either way
        around that spot (RESCAN_SEEK_STEP_MM up to RESCAN_SEEK_SPAN_MM) until
        it reads. Every step is a whole move, so the position stays exact.

        :param slot: physical slot
        :param link: register link to the reader
        :param pos: the lane's position (mm from its start) at the aimed spot
        :param excluded: predicate for UIDs to ignore (the sibling's)
        :param log: step logger
        :param gcmd: gives .error for a move that does not complete
        :param need_decode: False to settle for a tag that answers without
            decoding, such as a blank one about to be written
        :return tuple: (tag or None, the lane's position afterwards)
        """
        def _read() -> Optional[dict]:
            """
            A few read attempts at rest; errors become None.

            :return Optional[dict]: decoded tag or None
            """
            for _ in range(max(1, self.stage_read_hold_attempts)):
                try:
                    t = self._read_tag(link, is_excluded=excluded)
                except Exception:
                    t = None
                if (t
                    and (t.get("filament")
                         or (not need_decode
                             and t.get("uid")))):
                    return t
                self._dwell(0.12)
            return None

        aim = pos
        tag = _read()
        step = self.RESCAN_SEEK_STEP_MM
        k = 1
        while not tag and k * step <= self.RESCAN_SEEK_SPAN_MM + 1e-6:
            for off in (-k * step, k * step):
                target = aim + off
                # never feed past where the lane started
                if target > 0.0:
                    continue
                self._rescan_move(slot, target - pos, gcmd)
                pos = target
                tag = _read()
                if tag:
                    log(f"read the tag {off:+.0f}mm from where it was seen")
                    break
            k += 1
        if not tag:
            log("the tag was seen going past but did not read at rest")
        return tag, pos

    def cmd_ACE_RFID_SCAN(self, gcmd: "GCodeCommand") -> None:
        """
        Watch a scanner lane for a presented tag and stage it as the next spool id.

        LANE= defaults to the only configured scanner lane. The scan runs behind
        the command and reports to the console; reader glitches are caught so a
        bad scan never shuts Klipper down.

        Usage
        -------
        `ACE_RFID_SCAN [LANE=<name>] [SECONDS=<n>]`

        Example
        -------
        ```
        ACE_RFID_SCAN LANE=scanner1 SECONDS=30
        ```
        """
        lane_name = gcmd.get("LANE", None)
        if lane_name is None:
            if len(self._scanner_lanes) == 1:
                lane_name = next(iter(self._scanner_lanes))
            else:
                error_str = "ACE_RFID_SCAN requires LANE= (no single scanner_lanes lane)"
                raise self.printer.command_error(error_str)
        seconds = gcmd.get_float("SECONDS", self.scan_seconds,
                                 minval=1.0, maxval=600.0)
        if self._scan_running:
            error_str = "ACE2 RFID scan: a scan is already running"
            raise self.printer.command_error(error_str)
        gcmd.respond_info(
            f"ACE2 RFID scan: present the tag to {lane_name} (scanning {seconds:.0f}s)...")
        # The command returns now; the scan runs behind it in a reactor callback, where
        # pause() is legal. Holding the g-code queue for the whole scan read as a frozen
        # printer. Results go to the console since there is no command left to answer.
        self._scan_running = True
        self.reactor.register_callback(
            lambda et: self._scan_in_background(lane_name, seconds))

    def _scan_in_background(self, lane_name: str, seconds: float) -> None:
        """
        Run one scan off the g-code queue and report to the console.

        :param lane_name: the scanner lane to watch
        :param seconds: how long to watch for
        """
        say = self.gcode.respond_info
        try:
            tag = self.scan_lane(lane_name, seconds)
        except Exception as e:
            # A reader glitch must never shut Klipper down, and there is no
            # command left to raise to, so even a command_error is reported.
            self.logger.error("ACE2 RFID scan failed",
                              traceback=traceback.format_exc())
            say(f"ACE2 RFID scan: error: {e}")
            return
        finally:
            self._scan_running = False
        if not tag:
            say(f"ACE2 RFID scan: no tag found on {lane_name}")
            return
        f = tag.get("filament") or {}
        spool = getattr(self.afc, "spool", None) if self.afc is not None else None
        nid = getattr(spool, "next_spool_id", None)
        say(
            f'ACE2 RFID scan: staged next spool from {lane_name}, uid={tag.get("uid", "")} type='
                f'{f.get("type", "")}{((f" (spool #{nid})") if nid else "")}')


def load_config(config: "ConfigWrapper") -> AFC_ACE2_RFID:
    """
    Klipper entry point that builds the ACE2 RFID coordinator when this
    module's config section is loaded from the printer config.

    :param config: Klipper config section for this module
    :return AFC_ACE2_RFID: The configured RFID coordinator object
    """
    return AFC_ACE2_RFID(config)
