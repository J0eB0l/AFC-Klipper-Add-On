# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Shared host-side RFID reader stack: transport-agnostic MFRC522 primitives,
# MIFARE Classic access, and the multi-brand tag decoders (Bambu, Anycubic,
# Snapmaker, Creality CFS, Elegoo, BTT/ViViD "BQ Tech", QIDI, TigerTag,
# OpenSpool, OpenTag3D, SpoolEase).
#
# Everything runs on a minimal link contract, an object exposing
# reg_read(reg) -> byte and reg_write(reg, val), plus an optional
# reader_power(on), so the same stack serves every reader transport: the
# ACE2 firmware serial passthrough, the ViViD/OpenAMS MCU-SPI links, and the
# OpenAMS i2c-hook shim. read_tag() is the unified entry point.
#
# Nothing here talks to Klipper: no printer object, no config, no reactor.
#
# Credits: the per-brand key derivation and tag layouts are not original work.
# They come from the projects and write-ups below, ported to the link contract
# above. Each section banner names its own source as well.
#
#   OpenRFID: https://github.com/suchmememanyskill/OpenRFID
#       Bambu: the HKDF-SHA256 key derivation (salt -> per-sector Key A) and
#       the block field layout. Anycubic: the NTAG page layout, jointly with
#       DnG-Crafts. QIDI, TigerTag, SpoolEase and the NDEF record parsing.
#       The bulk of what this module knows about reading a tag.
#   paxx12/SnapmakerU1-Extended-Firmware
#       OpenSpool, with the U1's extended fields (docs/design/openspool.md).
#   OpenTag3D: https://opentag3d.info/spec.html
#       The v2 memory map (spec.json 2.001).
#   DnG-Crafts
#       Anycubic (with OpenRFID) and the Snapmaker U1 per-tag sector keys.
#   Bambu-Research-Group/RFID-Tag-Guide
#       Bambu block 16: colour count and the second colour's reversed ABGR.
#   ELEGOO-3D/ELEGOO-RFID-Tag-Guide
#       Elegoo NTAG213 EPC-256 field offsets.
#   bigtreetech/BIGTREETECH_MMS
#       BTT/ViViD "BQ Tech" tag layout, from its mfrc522.py.

from __future__ import annotations
import hashlib
import hmac
import json
import struct
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Tuple


# ── MFRC522 register map (subset used) ───────────────
CommandReg, ComIrqReg, ErrorReg, Status2Reg = 0x01, 0x04, 0x06, 0x08
FIFODataReg, FIFOLevelReg, ControlReg, BitFramingReg, CollReg = 0x09, 0x0A, 0x0C, 0x0D, 0x0E
TxControlReg = 0x14
# commands
PCD_IDLE, PCD_AUTHENT, PCD_TRANSCEIVE, PCD_MFAUTHENT = 0x00, 0x0E, 0x0C, 0x0E
# PICC (card) commands
PICC_REQA, PICC_WUPA = 0x26, 0x52
PICC_ANTICOLL, PICC_SELECT = 0x93, 0x93          # cascade level 1
PICC_ANTICOLL_CL2, PICC_SELECT_CL2 = 0x95, 0x95  # cascade level 2
SAK_UID_INCOMPLETE = 0x04              # "keep cascading", not a tag type
CASCADE_TAG = 0x88                     # stands in for the UID bytes CL1 lacks
PICC_AUTH_KEY_A, PICC_AUTH_KEY_B = 0x60, 0x61
PICC_READ = 0x30
PICC_WRITE_PAGE = 0xA2                 # NTAG / Ultralight 4-byte page write
PICC_WRITE_CLASSIC = 0xA0              # MIFARE Classic 16-byte block write
NTAG_ACK = 0x0A                        # 4-bit reply; anything else is a NAK
# The writable window. Pages 0-3 and the pages past the user area hold lock
# bits, config and password, and writing them is irreversible. A tag whose
# capability container will not parse is held to the NTAG213 bound (see
# MifareClassic.user_last_page).
NTAG_USER_FIRST_PAGE = 4
NTAG213_LAST_PAGE = 39

#: Capability-container size byte -> chip name, for the NTAG21x family. The CC
#: at page 3 reads "E1 10 <size> <access>", size in 8-byte units of user memory.
NTAG_CC_CHIPS = {0x12: "NTAG213", 0x3E: "NTAG215", 0x6D: "NTAG216"}


def ntag_chip(data: bytes) -> Tuple[Optional[str], Optional[int]]:
    """
    Name an Ultralight-family chip from its capability container.

    A plain MIFARE Ultralight has no CC (page 3 is its OTP area) and an
    unformatted NTAG can be blank there too. Both return (None, None) rather
    than a guess, since SAK 0x00 alone does not identify the chip.

    :param data: a tag read starting at page 0 (needs the first 16 bytes)
    :return tuple: (chip name or None, user-memory bytes or None)
    """
    if len(data) < 16 or data[12] != 0xE1 or not data[14]:
        return None, None
    return NTAG_CC_CHIPS.get(data[14]), data[14] * 8


class Mfrc522:
    """MFRC522 primitives built on any link object with reg_read/reg_write."""
    def __init__(self, link: Any) -> None:
        """
        MFRC522 driver over a register link.

        :param link: the register link
        """
        self.l = link

    def _set(self, reg: int, mask: int) -> None:
        """
        Set bits in a register.

        :param reg: register address
        :param mask: bits to set
        """
        self.l.reg_write(reg, self.l.reg_read(reg) | mask)

    def _clr(self, reg: int, mask: int) -> None:
        """
        Clear bits in a register.

        :param reg: register address
        :param mask: bits to clear
        """
        self.l.reg_write(reg, self.l.reg_read(reg) & (~mask & 0xFF))

    def reset(self) -> None:
        """
        Soft-reset and apply a minimal timer/ASK/mode config.

        A host-powered MFRC522 comes up cold and can hang the first register
        read without this bring-up. Writes only, so a cold chip cannot stall it;
        the caller dwells briefly afterwards for the oscillator.
        """
        self.l.reg_write(CommandReg, 0x0F)     # SoftReset
        self.l.reg_write(0x2A, 0x8D)           # TModeReg  (auto timer, TAuto)
        self.l.reg_write(0x2B, 0x3E)           # TPrescalerReg
        self.l.reg_write(0x2D, 30)             # TReloadReg low
        self.l.reg_write(0x2C, 0)              # TReloadReg high
        self.l.reg_write(0x15, 0x40)           # TxASKReg  (Force100ASK)
        self.l.reg_write(0x11, 0x3D)           # ModeReg   (CRC preset 0x6363)

    def stop_crypto(self) -> None:
        """
        End any MIFARE crypto session so a fresh anticoll/select works.

        Clears MFCrypto1On in Status2Reg without a SoftReset, keeping the RF
        field up so a marginal at-rest tag stays powered between decode schemes.
        """
        self._clr(Status2Reg, 0x08)

    def antenna_on(self) -> None:
        """
        Enable the TX antenna drivers if they are off.
        """
        if not (self.l.reg_read(TxControlReg) & 0x03):
            self._set(TxControlReg, 0x03)

    def _to_card(self, command: int, send: bytes,
                 tx_last_bits: int = 0,
                 err_mask: int = 0x1B) -> Tuple[bool, bytes, int]:
        """
        Execute a Transceive or MFAuthent command.

        ``err_mask`` selects which ErrorReg bits abort the exchange, default
        BufferOvfl|CollErr|ParityErr|ProtocolErr. A 4-bit ACK raises ProtocolErr,
        so that path passes a narrower mask and checks the ACK value instead.

        :param command: PCD command (PCD_TRANSCEIVE or PCD_MFAUTHENT)
        :param send: bytes to load into the FIFO
        :param tx_last_bits: valid bits in the last sent byte (unused here)
        :param err_mask: ErrorReg bits that abort the exchange
        :return Tuple[bool, bytes, int]: (ok, rx_bytes, rx_last_bits)
        """
        irq_en, wait_irq = (0x12, 0x10) if command == PCD_TRANSCEIVE else (0x12, 0x10)
        self.l.reg_write(CommandReg, PCD_IDLE)
        self._set(FIFOLevelReg, 0x80)                      # flush FIFO
        for b in send:
            self.l.reg_write(FIFODataReg, b)
        self.l.reg_write(CommandReg, command)
        if command == PCD_TRANSCEIVE:
            self._set(BitFramingReg, 0x80)                 # StartSend
        for _ in range(2000):                              # poll ComIrq
            n = self.l.reg_read(ComIrqReg)
            if n & 0x30:                                   # Rx or Idle
                break
            if n & 0x01:                                   # Timer
                self._clr(BitFramingReg, 0x80)
                return False, b"", 0
        self._clr(BitFramingReg, 0x80)
        if self.l.reg_read(ErrorReg) & err_mask:            # see err_mask above
            return False, b"", 0
        rx = b""
        if command == PCD_TRANSCEIVE:
            n = self.l.reg_read(FIFOLevelReg)
            rx = bytes(self.l.reg_read(FIFODataReg) for _ in range(n))
            last = self.l.reg_read(ControlReg) & 0x07
            return True, rx, last
        return True, rx, 0

    def request(self, req: int = PICC_REQA) -> Optional[bytes]:
        """
        Send REQA/WUPA to wake a tag.

        :param req: request command byte
        :return Optional[bytes]: the 2-byte ATQA, None when nothing answered
        """
        self.l.reg_write(BitFramingReg, 0x07)              # 7-bit frame
        ok, atqa, _ = self._to_card(PCD_TRANSCEIVE, bytes([req]), 7)
        return atqa if ok and len(atqa) == 2 else None

    def anticoll(self, cmd: int = PICC_ANTICOLL) -> Optional[bytes]:
        """
        Run one anticollision cascade level.

        :param cmd: PICC_ANTICOLL for level 1, PICC_ANTICOLL_CL2 for level 2
        :return Optional[bytes]: the 4 UID bytes this level carries, None on
            failure. At level 1 on a 7-byte-UID tag the first of those is the
            cascade tag, not a UID byte.
        """
        self.l.reg_write(BitFramingReg, 0x00)
        ok, uid5, _ = self._to_card(PCD_TRANSCEIVE, bytes([cmd, 0x20]))
        if not ok or len(uid5) != 5:
            return None
        if uid5[0] ^ uid5[1] ^ uid5[2] ^ uid5[3] != uid5[4]:  # BCC
            return None
        return uid5[:4]

    def select(self, uid4: bytes, cmd: int = PICC_SELECT) -> Optional[int]:
        """
        Select a tag on one cascade level.

        SAK 0x00 = Ultralight/NTAG, 0x08 = Classic 1K, and bit 2
        (SAK_UID_INCOMPLETE) means another cascade level follows.

        :param uid4: the 4 bytes this cascade level selects on
        :param cmd: PICC_SELECT for level 1, PICC_SELECT_CL2 for level 2
        :return Optional[int]: the SAK byte, None on failure
        """
        buf = bytes([cmd, 0x70]) + uid4 + bytes([uid4[0]^uid4[1]^uid4[2]^uid4[3]])
        buf += struct.pack("<H", self._crc_a(buf))
        ok, sak, _ = self._to_card(PCD_TRANSCEIVE, buf)
        return sak[0] if ok and len(sak) >= 1 else None

    def auth(self, key_type: int, block: int, key6: bytes, uid4: bytes) -> bool:
        """
        Authenticate a sector with a key.

        :param key_type: PICC_AUTHENT1A or 1B
        :param block: block number
        :param key6: 6-byte key
        :param uid4: 4-byte UID
        :return bool: True when Crypto1 is on
        """
        send = bytes([key_type, block]) + key6 + uid4
        ok, _, _ = self._to_card(PCD_MFAUTHENT, send)
        return ok and bool(self.l.reg_read(Status2Reg) & 0x08)  # MFCrypto1On

    def read_block(self, block: int) -> Optional[bytes]:
        """
        Read one 16-byte block.

        :param block: block number
        :return Optional[bytes]: the data, None on failure
        """
        buf = bytes([PICC_READ, block])
        buf += struct.pack("<H", self._crc_a(buf))
        ok, data, _ = self._to_card(PCD_TRANSCEIVE, buf)
        return data[:16] if ok and len(data) >= 16 else None

    def _short_ack(self) -> bool:
        """
        Read the 4-bit ACK that a MIFARE Classic write phase returns.

        Same shape as the NTAG write ACK: wait for RxIRq, let FIFOLevel settle
        over a slow bridge, then check for exactly one byte, 4 bits, 0x0A.

        :return bool: True on a valid ACK
        """
        level = 0
        for _ in range(8):
            level = self.l.reg_read(FIFOLevelReg)
            if level:
                break
        last = self.l.reg_read(ControlReg) & 0x07
        if level != 1 or last != 4:
            return False
        return (self.l.reg_read(FIFODataReg) & 0x0F) == NTAG_ACK

    def _classic_phase(self, frame: bytes) -> bool:
        """
        Send one CRC'd frame in an active Crypto1 session and read its 4-bit
        ACK. Used for both halves of the MIFARE Classic write.

        :param frame: bytes to send (CRC appended here)
        :return bool: True when the tag ACKed
        """
        buf = frame + struct.pack("<H", self._crc_a(frame))
        self.l.reg_write(CommandReg, PCD_IDLE)
        self._set(FIFOLevelReg, 0x80)
        for b in buf:
            self.l.reg_write(FIFODataReg, b)
        self.l.reg_write(CommandReg, PCD_TRANSCEIVE)
        self._set(BitFramingReg, 0x80)
        for _ in range(4000):
            n = self.l.reg_read(ComIrqReg)
            if n & 0x20:                                   # RxIRq
                break
            if n & 0x01:                                   # TimerIRq
                self._clr(BitFramingReg, 0x80)
                return False
        self._clr(BitFramingReg, 0x80)
        if self.l.reg_read(ErrorReg) & 0x12:               # BufferOvfl|ParityErr
            return False
        return self._short_ack()

    def write_classic_block(self, block: int, data16: bytes) -> bool:
        """
        MIFARE Classic WRITE (0xA0): a two-phase write of one 16-byte block.

        The sector must already be authenticated (Crypto1 on). The command
        frame (0xA0, block) gets a 4-bit ACK, then the 16 data bytes get another.

        Callers guard which block this touches: a sector trailer or the
        manufacturer block are not written here (see classic_write_block).

        :param block: absolute block number
        :param data16: exactly 16 bytes
        :return bool: True when both phases ACKed
        """
        if len(data16) != 16:
            error_str = f"Classic block write needs 16 bytes, got {len(data16)}"
            raise ValueError(error_str)
        if not self._classic_phase(bytes([PICC_WRITE_CLASSIC, block])):
            return False
        return self._classic_phase(bytes(data16))

    def write_page(self, page: int, data4: bytes) -> bool:
        """
        NTAG / Ultralight WRITE: one 4-byte page, no authentication.

        Single-phase, unlike the MIFARE Classic 0xA0 write: the whole frame
        goes out at once and the tag answers a 4-bit ACK. Callers must have
        bounds-checked the page (see MifareClassic.write_ntag): this raw wire op
        will happily program a lock page.

        :param page: page number
        :param data4: exactly 4 bytes
        :return bool: True when the tag ACKed
        """
        if len(data4) != 4:
            error_str = f"NTAG page write needs 4 bytes, got {len(data4)}"
            raise ValueError(error_str)
        buf = bytes([PICC_WRITE_PAGE, page]) + bytes(data4)
        buf += struct.pack("<H", self._crc_a(buf))
        # The 4-bit ACK arrives ~4ms after TX, but IdleIRq fires as TX ends, so
        # drive the transceive here and wait for RxIRq instead of using _to_card.
        self.l.reg_write(CommandReg, PCD_IDLE)
        self._set(FIFOLevelReg, 0x80)                      # flush FIFO
        for b in buf:
            self.l.reg_write(FIFODataReg, b)
        self.l.reg_write(CommandReg, PCD_TRANSCEIVE)
        self._set(BitFramingReg, 0x80)                     # StartSend
        got = False
        for _ in range(4000):
            n = self.l.reg_read(ComIrqReg)
            if n & 0x20:                                   # RxIRq: the ACK
                got = True
                break
            if n & 0x01:                                   # TimerIRq: no answer
                break
        self._clr(BitFramingReg, 0x80)
        err = self.l.reg_read(ErrorReg)
        # FIFOLevel can lag RxIRq over a slow serial-bridged link, so re-read it
        # a few times before treating the FIFO as empty.
        level = 0
        for _ in range(8):
            level = self.l.reg_read(FIFOLevelReg)
            if level:
                break
        last = self.l.reg_read(ControlReg) & 0x07
        ack = self.l.reg_read(FIFODataReg) if level else None
        # Raw response for a failed write: (got_rx, ErrorReg, FIFOlevel,
        # RxLastBits, ack). Empty is a timeout; a 4-bit 0x00/0x01/0x05 is a NAK
        # (locked page / bad CRC / invalid arg); 0x0a is the ACK.
        self.last_write_ack = (got, hex(err), level, last,
                               None if ack is None else hex(ack))
        # BufferOvfl or ParityErr is a real failure; ProtocolErr is expected on
        # a 4-bit frame and ignored. A valid ACK is exactly one byte, 4 bits,
        # value 0x0A.
        if err & 0x12:
            return False
        return (got
                and level == 1
                and last == 4
                and ack is not None
                and (ack & 0x0F) == NTAG_ACK)

    def halt(self) -> None:
        """
        Send HLTA so the selected tag stops answering REQA (only WUPA wakes it).

        Silences a shared-reader neighbour tag so this lane's own tag can
        respond. HLTA has no reply, so the result is ignored.
        """
        buf = bytes([0x50, 0x00])
        buf += struct.pack("<H", self._crc_a(buf))
        self._to_card(PCD_TRANSCEIVE, buf)

    @staticmethod
    def _crc_a(data: bytes) -> int:
        """
        ISO14443-A CRC (poly 0x8408, init 0x6363), computed in software.

        :param data: bytes to checksum
        :return int: the CRC
        """
        c = 0x6363
        for b in data:
            b ^= c & 0xFF
            b = (b ^ (b << 4)) & 0xFF
            c = ((c >> 8) ^ (b << 8) ^ (b << 3) ^ (b >> 4)) & 0xFFFF
        return c


class MifareClassic:
    """Full-tag read: activate → per-sector auth+read → 1024-byte image."""
    def __init__(self, mfrc: Mfrc522) -> None:
        """
        MIFARE Classic reader over an Mfrc522.

        :param mfrc: the MFRC522 driver
        """
        self.m = mfrc

    def activate(self, is_excluded: Optional[Callable[[str], bool]] = None,
                 seen: Optional[List[Any]] = None,
                 reset: bool = True) -> Tuple[Optional[bytes], Optional[int]]:
        """
        Activate an ISO14443-A tag and return its UID and SAK.

        On a shared reader, an excluded tag (a neighbour's) is HALTed and the
        request repeated until this lane's tag answers or the field is empty.
        The reader is reset only once up front, since a reset would drop the RF
        field and wake the halted neighbour.

        :param is_excluded: ``uid_hex -> bool`` predicate for tags to skip
        :param seen: list to append ``(uid_hex, sak, excluded)`` to for every
            tag detected, so callers can tell a halted neighbour from an empty
            field
        :param reset: False re-selects the same card without a SoftReset,
            keeping the RF field up between decode schemes
        :return Tuple[Optional[bytes], Optional[int]]: (uid, sak), or
            (None, None) when no tag answers
        """
        if reset:
            self.m.reset()          # bring up a cold, host-powered reader
        else:
            self.m.stop_crypto()    # re-select without dropping the RF field
        self.m.antenna_on()
        if is_excluded is None:
            uid, sak = self._activate_once(wake=True)
            if seen is not None and uid is not None:
                seen.append((uid.hex(), sak, False))
            return uid, sak
        for i in range(MAX_FIELD_TAGS):
            uid, sak = self._activate_once(wake=(i == 0))
            if uid is None:
                return None, None
            excluded = bool(is_excluded(uid.hex()))
            if seen is not None:
                seen.append((uid.hex(), sak, excluded))
            if excluded:
                self.m.halt()       # sleep the neighbour; next REQA skips it
                continue
            return uid, sak
        return None, None

    def _activate_once(self, wake: bool) -> Tuple[Optional[bytes], Optional[int]]:
        """
        Run one REQA/anticoll/select, following a second cascade level if needed.

        :param wake: use WUPA (wakes halted tags too) for first detection;
            otherwise REQA, which a halted neighbour ignores
        :return Tuple[Optional[bytes], Optional[int]]: (uid, sak), or
            (None, None) on failure
        """
        if wake:
            if (self.m.request(PICC_WUPA) is None
                    and self.m.request(PICC_REQA) is None):
                return None, None
        elif self.m.request(PICC_REQA) is None:
            return None, None
        uid = self.m.anticoll()
        if uid is None:
            return None, None
        sak = self.m.select(uid)
        if sak is None:
            return None, None
        if not sak & SAK_UID_INCOMPLETE:
            return uid, sak                    # 4-byte UID, one level, done
        # A 7-byte UID (every NTAG21x) needs cascade level 2 for bytes 3-6.
        # Stopping at level 1 leaves the tag half selected with a UID of "88"
        # plus three bytes, which collides within a roll of stickers.
        if uid[0] != CASCADE_TAG:
            return None, None
        rest = self.m.anticoll(PICC_ANTICOLL_CL2)
        if rest is None:
            return None, None
        sak = self.m.select(rest, PICC_SELECT_CL2)
        if sak is None:
            return None, None
        return uid[1:4] + rest, sak

    def read_ntag(self, nbytes: int = 128,
                  start_page: int = 0) -> Optional[bytes]:
        """
        MIFARE Ultralight / NTAG read (no auth). Each READ returns 4 pages.

        :param nbytes: how many bytes to read
        :param start_page: page to start at
        :return Optional[bytes]: the bytes, None if any read failed
        """
        data = bytearray()
        page = start_page
        while len(data) < nbytes:
            blk = self.m.read_block(page)      # NTAG uses the same 0x30 READ
            if blk is None:
                return None
            data += blk
            page += 4                          # each read returns 4 pages (16 B)
        return bytes(data[:nbytes])

    def user_last_page(self) -> int:
        """
        Highest page a writer may touch, read from the tag's own capability
        container rather than assumed from the chip we hope is in the field.

        Page 3 byte 2 is the user-memory size in 8-byte units, so the last user
        page is ``3 + size/4``. On NTAG215/216 this under-estimates slightly,
        which is the safe direction. A tag with no readable CC falls back to the
        NTAG213 bound.

        :return int: last writable page number
        """
        blk = self.m.read_block(3)
        if blk is None or blk[0] != 0xE1 or not blk[2]:
            return NTAG213_LAST_PAGE
        return 3 + blk[2] * 2

    def write_ntag(self, start_page: int, payload: bytes) -> Optional[str]:
        """
        Write a payload across consecutive NTAG pages and read it back.

        Refuses anything outside the user area, since a stray write to the lock,
        config or password pages kills the tag for good. Every page is read back
        and compared before this reports success.

        :param start_page: first page to write
        :param payload: bytes to write, a multiple of 4
        :return Optional[str]: None on success, else why it failed
        """
        if len(payload) % 4:
            return f"payload is {len(payload)} bytes, not a multiple of 4"
        pages = len(payload) // 4
        last = self.user_last_page()
        if start_page < NTAG_USER_FIRST_PAGE:
            return (f"page {start_page} is in the reserved area; the first "
                    f"writable page is {NTAG_USER_FIRST_PAGE}")
        if start_page + pages - 1 > last:
            return (f"{pages} pages from {start_page} runs past the last "
                    f"writable page ({last})")
        for i in range(pages):
            chunk = payload[i * 4:i * 4 + 4]
            # Retry a page: over a slow bridge the ACK can be missed although
            # the write took. Re-writing is idempotent; the read-back verifies.
            if not (self.m.write_page(start_page + i, chunk)
                    or self.m.write_page(start_page + i, chunk)
                    or self.m.write_page(start_page + i, chunk)):
                dbg = getattr(self.m, "last_write_ack", None)
                return (f"tag did not ACK the write to page {start_page + i} "
                        f"[resp {dbg}]")
        return self._verify_ntag(start_page, payload)

    def _verify_ntag(self, start_page: int, payload: bytes) -> Optional[str]:
        """
        Read written pages back and compare them to what was sent.

        :param start_page: first page written
        :param payload: the bytes that were written
        :return Optional[str]: None when the tag matches, else the first
            page that does not
        """
        pages = len(payload) // 4
        for i in range(0, pages, 4):                   # READ returns 4 pages
            blk = self.m.read_block(start_page + i)
            if blk is None:
                return f"could not read page {start_page + i} back"
            want = payload[i * 4:i * 4 + 16]
            if blk[:len(want)] != want:
                # One READ covers 4 pages: name the first one that differs.
                j = next(k for k in range(0, len(want), 4)
                         if blk[k:k + 4] != want[k:k + 4])
                return (f"page {start_page + i + j // 4} reads back as "
                        f"{blk[j:j + 4].hex()}, not {want[j:j + 4].hex()}")
        return None

    def read_all(self, uid: bytes, keys_a: List[List[int]],
                 sectors: int = 16) -> Optional[bytes]:
        """
        Read the first ``sectors`` sectors (auth + 4 blocks each).

        Reading only the sectors a decoder needs saves serial round-trips.

        :param uid: tag UID
        :param keys_a: 16 per-sector Key A entries
        :param sectors: number of sectors to read (clamped to 1-16)
        :return Optional[bytes]: ``sectors*64`` bytes, None when any read fails
        """
        sectors = max(1, min(int(sectors), 16))
        data = bytearray(sectors * 64)
        for sector in range(sectors):
            if not self.m.auth(PICC_AUTH_KEY_A, sector * 4, bytes(keys_a[sector]), uid):
                return None
            for b in range(4):
                block = sector * 4 + b
                blk = self.m.read_block(block)
                if blk is None:
                    return None
                data[block * 16:block * 16 + 16] = blk
        return bytes(data)

    def read_blocks(self, uid: bytes, keys_a: List[List[int]],
                    blocks: Any) -> Optional[bytes]:
        """
        Read only the given blocks into a 1024-byte image (rest zero).

        Each sector is authenticated once. Reading only needed blocks saves
        round-trips over the ACE serial passthrough.

        :param uid: tag UID
        :param keys_a: 16 per-sector Key A entries
        :param blocks: iterable of absolute block numbers
        :return Optional[bytes]: the 1024-byte image, None when any read fails
        """
        data = bytearray(1024)
        by_sector: Dict[int, List[int]] = {}
        for b in blocks:
            by_sector.setdefault(b // 4, []).append(b)
        for sector in sorted(by_sector):
            if not self.m.auth(PICC_AUTH_KEY_A, sector * 4, bytes(keys_a[sector]), uid):
                return None
            for b in sorted(by_sector[sector]):
                blk = self.m.read_block(b)
                if blk is None:
                    return None
                data[b * 16:b * 16 + 16] = blk
        return bytes(data)


# ── Bambu (HKDF key derivation + decode), port of OpenRFID ───────
def hkdf_sha256(salt: bytes, ikm: bytes, info: bytes, length: int) -> bytes:
    """
    HKDF-SHA256 extract-and-expand.

    :param salt: HKDF salt
    :param ikm: input keying material
    :param info: context bytes
    :param length: bytes to derive
    :return bytes: the derived key material
    """
    prk = hmac.new(salt, ikm, hashlib.sha256).digest()
    okm, t, i = b"", b"", 0
    while len(okm) < length:
        i += 1
        t = hmac.new(prk, t + info + bytes([i]), hashlib.sha256).digest()
        okm += t
    return okm[:length]

BAMBU_SALT_HASH = "19cc3c63cb8802668800c3b3bf3fee05b3c59bf59fc5fd256b68e868084ec304"


def bambu_keys(uid: bytes, master_key: bytes) -> List[List[int]]:
    """
    Derive the 16 sector A keys for a Bambu tag.

    :param uid: 4-byte tag UID
    :param master_key: Bambu master key
    :return list: 16 six-byte keys
    """
    okm = hkdf_sha256(master_key, uid, b"RFID-A\x00", 6 * 16)
    return [list(okm[s * 6:(s + 1) * 6]) for s in range(16)]


def _s(d: bytes, p: int, n: int) -> str:
    """
    NUL-terminated ASCII string at an offset.

    :param d: tag image
    :param p: offset
    :param n: field width
    :return str: the decoded string
    """
    return d[p:p + n].split(b"\x00")[0].decode("ascii", "replace")


def _u16(d: bytes, p: int) -> int:
    """
    Little-endian u16 at an offset.

    :param d: tag image
    :param p: offset
    :return int: the value
    """
    return struct.unpack_from("<H", d, p)[0]


def _f32(d: bytes, p: int) -> float:
    """
    Little-endian float32 at an offset.

    :param d: tag image
    :param p: offset
    :return float: the value
    """
    return struct.unpack_from("<f", d, p)[0]


def decode_bambu(d: bytes) -> dict:
    """
    Decode a Bambu MIFARE Classic 1K tag image.

    :param d: 1024-byte tag image
    :return dict: decoded filament fields
    """
    if len(d) != 1024:
        error_str = "expected 1024-byte MIFARE Classic 1K image"
        raise ValueError(error_str)
    r, g, b, a = d[80], d[81], d[82], d[83]
    nozzle = round(_f32(d, 140), 2)                   # blk8 @140
    spool_width = _u16(d, 164)                        # blk10 @164, mm*100
    return dict(
        manufacturer="Bambu",
        type=_s(d, 32, 16), detailed=_s(d, 64, 16),
        color_argb=(a << 24) | (r << 16) | (g << 8) | b,
        weight_g=_u16(d, 84), diameter_mm=round(_f32(d, 88), 3),
        drying_temp_c=_u16(d, 96), drying_time_h=_u16(d, 98),
        bed_temp_c=_u16(d, 102), hotend_max_c=_u16(d, 104), hotend_min_c=_u16(d, 106),
        nozzle_diameter=nozzle if 0 < nozzle < 2 else None,
        spool_width_mm=round(spool_width / 100.0, 2) if spool_width else None,
        length_m=_u16(d, 228) or None,                # blk14 @228, metres
        tray_uid=d[144:160].hex(), production=_s(d, 192, 16),
    )


# ── Anycubic (MIFARE Ultralight/NTAG, no auth), port of OpenRFID/DnG-Crafts ──
ANYCUBIC_MAGIC = b"\x7b\x00\x65\x00"
_ANY_WEIGHT = {330: 1000, 247: 750, 198: 600, 165: 500, 82: 250}


def decode_anycubic(d: bytes) -> Optional[dict]:
    """
    Decode an Anycubic tag image.

    :param d: tag image
    :return Optional[dict]: decoded fields, None without the Anycubic magic
    """
    if len(d) < 0x7C or d[0x10:0x14] != ANYCUBIC_MAGIC:
        return None
    sku = _s(d, 0x14, 16)
    brand = _s(d, 0x28, 16)
    ftype = _s(d, 0x3C, 16)
    a, b, g, r = d[0x50], d[0x51], d[0x52], d[0x53]
    length_m = _u16(d, 0x7A)
    return dict(
        manufacturer=brand or "Anycubic", sku=sku, type=ftype,
        color_argb=(a << 24) | (r << 16) | (g << 8) | b,
        diameter_mm=round(_u16(d, 0x78) / 100.0, 2),
        weight_g=_ANY_WEIGHT.get(length_m, 1000),
        length_m=length_m or None,
        hotend_min_c=_u16(d, 0x60), hotend_max_c=_u16(d, 0x62),
        bed_temp_c=_u16(d, 0x76),
    )


# The image bytes encode_anycubic fills, and the NTAG page they land on. The
# magic sits at 0x10 (page 4, the first user page), so it fits an NTAG213.
ANYCUBIC_FIRST_PAGE = NTAG_USER_FIRST_PAGE
ANYCUBIC_IMAGE_START = 0x10
ANYCUBIC_IMAGE_END = 0x80              # exclusive; 112 bytes, 28 pages
# weight -> length_m, inverted from _ANY_WEIGHT. Only these five spool sizes
# survive a round trip; anything else reads back as the 1000g default.
_ANY_LENGTH = {g: m for m, g in _ANY_WEIGHT.items()}


def _put_s(buf: bytearray, p: int, n: int, s: str) -> None:
    """
    Write a NUL-padded ASCII string into an image, truncating to fit.

    :param buf: image being built
    :param p: offset
    :param n: field width
    :param s: text to write
    """
    raw = (s or "").encode("ascii", "replace")[:n]
    buf[p:p + n] = raw + b"\x00" * (n - len(raw))


def _put_u16(buf: bytearray, p: int, v: Optional[int]) -> None:
    """
    Write a little-endian u16 into an image, clamped to the field.

    :param buf: image being built
    :param p: offset
    :param v: value, None or negative writes zero
    """
    struct.pack_into("<H", buf, p, max(0, min(int(v or 0), 0xFFFF)))


def anycubic_length_for_weight(weight_g: Optional[int]) -> int:
    """
    The length that makes decode_anycubic report a given spool weight.

    :param weight_g: desired weight in grams
    :return int: length in metres, 0 when the weight has no encoding
    """
    return _ANY_LENGTH.get(int(weight_g or 0), 0)


def encode_anycubic(manufacturer: str = "", sku: str = "", ftype: str = "",
                    color_argb: int = 0xFFFFFFFF, diameter_mm: float = 1.75,
                    weight_g: Optional[int] = None,
                    length_m: Optional[int] = None,
                    hotend_min_c: Optional[int] = None,
                    hotend_max_c: Optional[int] = None,
                    bed_temp_c: Optional[int] = None) -> bytes:
    """
    Build the Anycubic-layout bytes for pages 4-31 of a blank NTAG.

    The exact inverse of decode_anycubic, so a written tag reads back through
    the normal scan path. Only fields that decoder reads are populated; the rest
    stay zero. ``manufacturer`` is free text, so a spool keeps its real brand.

    :param manufacturer: brand string, 16 bytes
    :param sku: SKU / part number, 16 bytes
    :param ftype: material name such as PLA or PETG, 16 bytes
    :param color_argb: packed ARGB colour
    :param diameter_mm: filament diameter
    :param weight_g: spool weight; stored as the length that decodes back to
        it when one of the five standard sizes, otherwise left blank here and
        carried exactly by the AFC block (see encode_tag_payload)
    :param length_m: filament length, overriding the weight lookup
    :param hotend_min_c: minimum hotend temperature
    :param hotend_max_c: maximum hotend temperature
    :param bed_temp_c: bed temperature
    :return bytes: 112 bytes to write at page 4
    """
    if length_m is None:
        length_m = anycubic_length_for_weight(weight_g)
    buf = bytearray(ANYCUBIC_IMAGE_END)
    buf[0x10:0x14] = ANYCUBIC_MAGIC
    _put_s(buf, 0x14, 16, sku)
    _put_s(buf, 0x28, 16, manufacturer)
    _put_s(buf, 0x3C, 16, ftype)
    # Byte order on the tag is A, B, G, R (decode_anycubic reads it that way).
    buf[0x50] = (color_argb >> 24) & 0xFF
    buf[0x51] = color_argb & 0xFF
    buf[0x52] = (color_argb >> 8) & 0xFF
    buf[0x53] = (color_argb >> 16) & 0xFF
    _put_u16(buf, 0x60, hotend_min_c)
    _put_u16(buf, 0x62, hotend_max_c)
    _put_u16(buf, 0x76, bed_temp_c)
    _put_u16(buf, 0x78, round(float(diameter_mm) * 100))
    _put_u16(buf, 0x7A, length_m)
    return bytes(buf[ANYCUBIC_IMAGE_START:ANYCUBIC_IMAGE_END])


# ── AFC extension block ─────────────────────────────────────────────────────
# The Anycubic layout cannot hold an exact weight or a Spoolman id, so those go
# in our own block at page 32: past everything decode_anycubic reads and inside
# an NTAG213's user area. Genuine Anycubic tags lack the magic and are unaffected.
AFC_BLOCK_MAGIC = b"AFC1"
AFC_BLOCK_VERSION = 1
AFC_BLOCK_PAGE = 32
AFC_BLOCK_OFFSET = AFC_BLOCK_PAGE * 4          # 0x80
AFC_BLOCK_LEN = 32                             # pages 32-39
#: Bytes a full NTAG scan reads: through the end of the block.
NTAG_SCAN_BYTES = AFC_BLOCK_OFFSET + AFC_BLOCK_LEN


def encode_afc_block(weight_g: Optional[int] = None,
                     spool_id: Optional[int] = None,
                     density: Optional[float] = None,
                     drying_temp_c: Optional[int] = None,
                     drying_time_h: Optional[int] = None) -> bytes:
    """
    Build the AFC extension block: what the Anycubic layout cannot hold.

    :param weight_g: exact net filament weight in grams
    :param spool_id: Spoolman spool id this tag belongs to
    :param density: g/cm^3
    :param drying_temp_c: drying temperature
    :param drying_time_h: drying time in hours
    :return bytes: AFC_BLOCK_LEN bytes, to be written at AFC_BLOCK_PAGE
    """
    buf = bytearray(AFC_BLOCK_LEN)
    buf[0:4] = AFC_BLOCK_MAGIC
    buf[4] = AFC_BLOCK_VERSION
    _put_u16(buf, 6, weight_g)
    struct.pack_into("<I", buf, 8, max(0, min(int(spool_id or 0), 0xFFFFFFFF)))
    _put_u16(buf, 12, round(float(density) * 1000) if density else 0)
    _put_u16(buf, 14, drying_temp_c)
    _put_u16(buf, 16, drying_time_h)
    return bytes(buf)


def decode_afc_block(d: bytes, offset: int = AFC_BLOCK_OFFSET
                     ) -> Optional[dict]:
    """
    Decode the AFC extension block out of a tag image.

    Only fields that were actually set come back, so a caller can overlay the
    result onto a brand decode without blanking what the brand layout knew.

    :param d: tag image, or just the block when ``offset`` is 0
    :param offset: where the block starts in ``d``
    :return Optional[dict]: the fields that are set, None without the magic
    """
    p = offset
    if len(d) < p + AFC_BLOCK_LEN or d[p:p + 4] != AFC_BLOCK_MAGIC:
        return None
    if d[p + 4] > AFC_BLOCK_VERSION:
        return None                            # written by something newer
    out: Dict[str, Any] = {}
    if _u16(d, p + 6):
        out["weight_g"] = _u16(d, p + 6)
    spool_id = struct.unpack_from("<I", d, p + 8)[0]
    if spool_id:
        out["spool_id"] = spool_id
    if _u16(d, p + 12):
        out["density"] = round(_u16(d, p + 12) / 1000.0, 3)
    if _u16(d, p + 14):
        out["drying_temp_c"] = _u16(d, p + 14)
    if _u16(d, p + 16):
        out["drying_time_h"] = _u16(d, p + 16)
    return out


def encode_tag_payload(weight_g: Optional[int] = None,
                       spool_id: Optional[int] = None,
                       density: Optional[float] = None,
                       drying_temp_c: Optional[int] = None,
                       drying_time_h: Optional[int] = None,
                       **anycubic: Any) -> bytes:
    """
    Build the whole record for a blank NTAG: Anycubic layout plus the AFC block.

    One contiguous 144-byte payload written from page 4 to page 39. The Anycubic
    half keeps an approximate weight so a reader unaware of the AFC block still
    gets the nearest standard spool size.

    :param weight_g: exact net filament weight in grams
    :param spool_id: Spoolman spool id this tag belongs to
    :param density: g/cm^3
    :param drying_temp_c: drying temperature
    :param drying_time_h: drying time in hours
    :param anycubic: remaining encode_anycubic keyword arguments
    :return bytes: 144 bytes to write at ANYCUBIC_FIRST_PAGE
    """
    return (encode_anycubic(weight_g=weight_g, **anycubic)
            + encode_afc_block(weight_g=weight_g, spool_id=spool_id,
                               density=density, drying_temp_c=drying_temp_c,
                               drying_time_h=drying_time_h))


# ── Snapmaker U1 (MIFARE Classic, HKDF-derived keys), port of DnG-Crafts U1 ──
# Per-tag sector keys: PRK = HMAC-SHA256(salt, UID); sector key A =
# HMAC-SHA256(PRK, b"key_a_<sector>\x01")[:6], one HKDF-expand block, exactly
# what hkdf_sha256() computes for length 6. (RSA signature check is skipped.)
SNAPMAKER_SALT_A = b"Snapmaker_qwertyuiop[,.;]"
SNAPMAKER_SALT_B = b"Snapmaker_qwertyuiop[,.;]_1q2w3e"
SNAPMAKER_MAIN_TYPE = {1: "PLA", 2: "PETG", 3: "ABS", 4: "TPU", 5: "PVA"}
SNAPMAKER_SUB_TYPE = {1: "Basic", 2: "Matte", 3: "SnapSpeed", 4: "Silk",
                      5: "Support", 6: "HF", 7: "95A", 8: "95A-HF"}
# Blocks holding the fields decode_snapmaker reads (vendor@16=blk1,
# manufacturer@32=blk2, version/type/color-nums/alpha@64=blk4, RGB@80=blk5,
# SKU@96=blk6, diameter/weight@128=blk8, temps@144=blk9, mfg-date@160=blk10).
SNAPMAKER_READ_BLOCKS = (1, 2, 4, 5, 6, 8, 9, 10)


def snapmaker_keys(uid: bytes) -> List[List[int]]:
    """
    Derive the 16 sector A keys for a Snapmaker tag.

    :param uid: 4-byte tag UID
    :return list: 16 six-byte keys
    """
    return [list(hkdf_sha256(SNAPMAKER_SALT_A, uid,
                             (f"key_a_{s}").encode(), 6)) for s in range(16)]


def decode_snapmaker(d: bytes) -> Optional[dict]:
    """
    Decode a Snapmaker U1 MIFARE Classic 1K image.

    Offsets are linear into the 1024-byte image (sector*64 + block*16 + byte).

    :param d: tag image
    :return Optional[dict]: decoded fields, None if the main filament type is
        not a known Snapmaker code
    """
    if len(d) < 176:
        return None
    main = SNAPMAKER_MAIN_TYPE.get(_u16(d, 66))
    if main is None:                                 # wrong keys / not Snapmaker
        return None
    alpha = 0xFF - d[73]
    rgb1 = (d[80] << 16) | (d[81] << 8) | d[82]
    sku = d[96] | (d[97] << 8) | (d[98] << 16) | (d[99] << 24)
    diameter = _u16(d, 128)
    hot_max, hot_min = _u16(d, 148), _u16(d, 150)
    return dict(
        manufacturer=_s(d, 16, 16) or "Snapmaker",
        type=main, detailed=SNAPMAKER_SUB_TYPE.get(_u16(d, 68), ""),
        color_argb=(alpha << 24) | rgb1,
        weight_g=_u16(d, 130) or 1000,
        diameter_mm=round(diameter / 100.0, 2) if diameter else 1.75,
        hotend_max_c=hot_max or None, hotend_min_c=hot_min or None,
        bed_temp_c=_u16(d, 154) or None,
        sku=str(sku) if sku else "", production=_s(d, 160, 8),
    )


# ── AES-128 (pure Python, dependency-free), for Creality CFS tags ─────
# klippy-env has no working crypto lib, so bundle a small AES-128: Creality
# derives the MIFARE key as AES(UID) and stores the payload as AES-128-CBC.
_AES_SBOX = bytes.fromhex(
    "637c777bf26b6fc53001672bfed7ab76ca82c97dfa5947f0add4a2af9ca472c0"
    "b7fd9326363ff7cc34a5e5f171d8311504c723c31896059a071280e2eb27b275"
    "09832c1a1b6e5aa0523bd6b329e32f8453d100ed20fcb15b6acbbe394a4c58cf"
    "d0efaafb434d338545f9027f503c9fa851a3408f929d38f5bcb6da2110fff3d2"
    "cd0c13ec5f974417c4a77e3d645d197360814fdc222a908846eeb814de5e0bdb"
    "e0323a0a4906245cc2d3ac629195e479e7c8376d8dd54ea96c56f4ea657aae08"
    "ba78252e1ca6b4c6e8dd741f4bbd8b8a703eb5664803f60e613557b986c11d9e"
    "e1f8981169d98e949b1e87e9ce5528df8ca1890dbfe6426841992d0fb054bb16")
_AES_INV_SBOX = bytes(_AES_SBOX.index(i) for i in range(256))
_AES_RCON = (0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36)


def _aes_gmul(a: int, b: int) -> int:
    """
    GF(2^8) multiply for AES MixColumns.

    :param a: first operand
    :param b: second operand
    :return int: the product
    """
    p = 0
    for _ in range(8):
        if b & 1:
            p ^= a
        hi = a & 0x80
        a = (a << 1) & 0xFF
        if hi:
            a ^= 0x1B
        b >>= 1
    return p & 0xFF


def _aes_expand(key: bytes) -> List[List[int]]:
    """
    AES-128 key schedule.

    :param key: 16-byte key
    :return list: 44 round-key words
    """
    w = [list(key[i:i + 4]) for i in range(0, 16, 4)]
    for i in range(4, 44):
        t = list(w[i - 1])
        if i % 4 == 0:
            t = [_AES_SBOX[b] for b in t[1:] + t[:1]]
            t[0] ^= _AES_RCON[i // 4 - 1]
        w.append([w[i - 4][j] ^ t[j] for j in range(4)])
    return w


def _aes_ark(s: List[List[int]], w: List[List[int]], rnd: int) -> None:
    """
    AES AddRoundKey, in place.

    :param s: state columns
    :param w: expanded key
    :param rnd: round number
    """
    for c in range(4):
        for r in range(4):
            s[r][c] ^= w[rnd * 4 + c][r]


def _aes_encrypt_block(pt: bytes, key: bytes) -> bytes:
    """
    AES-128 encrypt one block.

    :param pt: 16-byte plaintext
    :param key: 16-byte key
    :return bytes: the ciphertext block
    """
    w = _aes_expand(key)
    s = [[pt[r + 4 * c] for c in range(4)] for r in range(4)]
    _aes_ark(s, w, 0)
    for rnd in range(1, 10):
        for r in range(4):
            for c in range(4):
                s[r][c] = _AES_SBOX[s[r][c]]
        for r in range(1, 4):
            s[r] = s[r][r:] + s[r][:r]
        for c in range(4):
            a = [s[r][c] for r in range(4)]
            s[0][c] = _aes_gmul(a[0], 2) ^ _aes_gmul(a[1], 3) ^ a[2] ^ a[3]
            s[1][c] = a[0] ^ _aes_gmul(a[1], 2) ^ _aes_gmul(a[2], 3) ^ a[3]
            s[2][c] = a[0] ^ a[1] ^ _aes_gmul(a[2], 2) ^ _aes_gmul(a[3], 3)
            s[3][c] = _aes_gmul(a[0], 3) ^ a[1] ^ a[2] ^ _aes_gmul(a[3], 2)
        _aes_ark(s, w, rnd)
    for r in range(4):
        for c in range(4):
            s[r][c] = _AES_SBOX[s[r][c]]
    for r in range(1, 4):
        s[r] = s[r][r:] + s[r][:r]
    _aes_ark(s, w, 10)
    return bytes(s[r][c] for c in range(4) for r in range(4))


def _aes_decrypt_block(ct: bytes, key: bytes) -> bytes:
    """
    AES-128 decrypt one block.

    :param ct: 16-byte ciphertext
    :param key: 16-byte key
    :return bytes: the plaintext block
    """
    w = _aes_expand(key)
    s = [[ct[r + 4 * c] for c in range(4)] for r in range(4)]
    _aes_ark(s, w, 10)
    for rnd in range(9, 0, -1):
        for r in range(1, 4):
            s[r] = s[r][-r:] + s[r][:-r]
        for r in range(4):
            for c in range(4):
                s[r][c] = _AES_INV_SBOX[s[r][c]]
        _aes_ark(s, w, rnd)
        for c in range(4):
            a = [s[r][c] for r in range(4)]
            s[0][c] = (_aes_gmul(a[0], 14) ^ _aes_gmul(a[1], 11)
                       ^ _aes_gmul(a[2], 13) ^ _aes_gmul(a[3], 9))
            s[1][c] = (_aes_gmul(a[0], 9) ^ _aes_gmul(a[1], 14)
                       ^ _aes_gmul(a[2], 11) ^ _aes_gmul(a[3], 13))
            s[2][c] = (_aes_gmul(a[0], 13) ^ _aes_gmul(a[1], 9)
                       ^ _aes_gmul(a[2], 14) ^ _aes_gmul(a[3], 11))
            s[3][c] = (_aes_gmul(a[0], 11) ^ _aes_gmul(a[1], 13)
                       ^ _aes_gmul(a[2], 9) ^ _aes_gmul(a[3], 14))
    for r in range(1, 4):
        s[r] = s[r][-r:] + s[r][:-r]
    for r in range(4):
        for c in range(4):
            s[r][c] = _AES_INV_SBOX[s[r][c]]
    _aes_ark(s, w, 0)
    return bytes(s[r][c] for c in range(4) for r in range(4))


def _aes_cbc_decrypt(data: bytes, key: bytes, iv: bytes = b"\x00" * 16) -> bytes:
    """
    AES-128-CBC decrypt the whole blocks of a buffer.

    :param data: ciphertext
    :param key: 16-byte key
    :param iv: initialisation vector
    :return bytes: the plaintext
    """
    out, prev = bytearray(), iv
    for i in range(0, len(data) - len(data) % 16, 16):
        blk = data[i:i + 16]
        out += bytes(d ^ p for d, p in zip(_aes_decrypt_block(blk, key), prev))
        prev = blk
    return bytes(out)


# ── Creality CFS (MIFARE Classic, UID-derived key + AES-CBC payload) ─────
# The 48-byte CFS payload is
# in sector 1 (blocks 4-6); the MIFARE Key A is AES(UID-repeated-to-16) with the
# u_key; the payload is AES-128-CBC (zero IV) with the d_key. Both keys are set
# in config (creality_key = u_key, creality_encryption_key = d_key).
CREALITY_READ_BLOCKS = (4, 5, 6)
CREALITY_FILM_ID = {"101001": "PLA", "101002": "PETG", "101003": "ABS",
                    "101004": "TPU", "101005": "PA", "E00003": "PLA"}
CREALITY_LENGTH_WEIGHT = {130: 250, 357: 500, 408: 600, 583: 750, 816: 1000}


def creality_mifare_key(uid: bytes, u_key: bytes) -> bytes:
    """
    Derive the 6-byte MIFARE Key A: AES(UID repeated to 16 bytes) with u_key.

    :param uid: tag UID
    :param u_key: Creality UID key
    :return bytes: the first 6 bytes of the ciphertext
    """
    uid16 = bytes(uid[i % len(uid)] for i in range(16))
    return _aes_encrypt_block(uid16, bytes(u_key))[:6]


def decode_creality(data48: bytes) -> Optional[dict]:
    """
    Decode the 48-byte decrypted CFS payload (ASCII fields).

    :param data48: decrypted payload
    :return Optional[dict]: decoded fields, None when it does not look like a
        Creality tag (wrong keys give garbage)
    """
    if len(data48) < 34:
        return None
    s = data48.decode("ascii", "replace")
    vendor = s[5:9]
    film = s[11:17]
    material = CREALITY_FILM_ID.get(film, "")
    if vendor != "0276" and not material:            # not a Creality payload
        return None
    color = s[17:24]                                 # "0RRGGBB"
    argb = None
    try:
        argb = 0xFF000000 | (int(color[1:7], 16) & 0xFFFFFF)
    except ValueError:
        pass
    try:
        length_m = int(s[24:28], 16)
    except ValueError:
        length_m = 0
    return dict(
        manufacturer="Creality", type=material, detailed="",
        color_argb=argb, sku=film.strip("\x00 ") or "",
        diameter_mm=1.75,                            # not on the tag
        weight_g=CREALITY_LENGTH_WEIGHT.get(length_m),
        length_m=length_m or None,
        serial=s[28:34], production=s[0:5],
    )


# ── Elegoo (NTAG213, plain EPC-256, no auth), ELEGOO-3D/ELEGOO-RFID-Tag-Guide ─
# User memory starts at NTAG page 4 (byte 16 of a page-0 read). Fields are packed
# big-endian from there: header@16, mfr@17, code@21, material@23, subtype@27,
# color@31, diameter@34, weight@36, date@38.
ELEGOO_HEADER = 0x36
ELEGOO_MFR = b"\xee\xee\xee\xee"


def decode_elegoo(d: bytes) -> Optional[dict]:
    """
    Decode an Elegoo tag image.

    :param d: tag image
    :return Optional[dict]: decoded fields, None without the Elegoo header
    """
    if len(d) < 40 or d[16] != ELEGOO_HEADER or d[17:21] != ELEGOO_MFR:
        return None
    rgb = (d[31] << 16) | (d[32] << 8) | d[33]
    return dict(
        manufacturer="Elegoo",
        type=_s(d, 23, 4).strip(), detailed=_s(d, 27, 4).strip(),
        color_argb=0xFF000000 | rgb,
        diameter_mm=round(((d[34] << 8) | d[35]) / 100.0, 2),
        weight_g=(d[36] << 8) | d[37],
        production=f"{(d[38] << 8) | d[39]:04d}",
    )


# ── unified entry: read UID for ANY tag, decode brand when we can ──────
SAK_CLASSIC_1K = 0x08
SAK_ULTRALIGHT = 0x00
# The exact blocks decode_bambu reads (type@32=blk2, detailed@64=blk4,
# color/weight/diameter@80=blk5, temps@96=blk6, nozzle@140=blk8, tray_uid@144=blk9,
# spool-width@164=blk10, production@192=blk12, length@228=blk14). Reading only
# these roughly halves the transceives over the slow serial passthrough.
BAMBU_READ_BLOCKS = (2, 4, 5, 6, 8, 9, 10, 12, 14)
# Max tags to enumerate/halt in one shared-reader activation (2 slots per
# reader, so at most the neighbour + this lane's tag; a little slack).
MAX_FIELD_TAGS = 4


def _bambu_apply_multicolor(fil: dict, block16: Optional[bytes]) -> None:
    """
    Fill fil['color_count'] and fil['colors_argb'] (list, ARGB) from block 16.

    Per the Bambu-Research-Group RFID-Tag-Guide, block 16 holds the colour count
    @258 (u16 LE) and the second colour @260 in reversed ABGR order. A single
    colour or missing block leaves just the primary.

    :param fil: decoded filament dict, updated in place
    :param block16: 1024-byte image with block 16 populated, or None
    """
    primary = fil.get("color_argb")
    colors = [primary] if primary is not None else []
    count = 1
    if block16 and len(block16) >= 264:
        count = _u16(block16, 258) or 1
        if count >= 2:
            a2, b2, g2, r2 = (block16[260], block16[261],
                              block16[262], block16[263])
            if a2 | b2 | g2 | r2:                 # block 16 actually present
                colors.append((a2 << 24) | (r2 << 16) | (g2 << 8) | b2)
            else:
                # No second colour to list, so count it as the single colour
                # it is; color_count must match colors_argb.
                count = 1
    fil["color_count"] = count
    fil["colors_argb"] = colors


def _classic_bambu(mc: MifareClassic, uid: bytes, master_key: bytes,
                   blocks: Any) -> Optional[dict]:
    """
    Read and decode a Bambu Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :param master_key: Bambu master key
    :param blocks: blocks to read
    :return Optional[dict]: decoded fields, None when the read fails
    """
    keys = bambu_keys(uid, master_key)
    data = mc.read_blocks(uid, keys, blocks)
    if not data:
        return None
    fil = decode_bambu(data)
    # A dual/gradient spool's second colour is in block 16 (sector 4). Read it
    # separately, best-effort, so an auth hiccup cannot break the primary decode.
    try:
        block16 = mc.read_blocks(uid, keys, (16,))
    except Exception:
        block16 = None
    _bambu_apply_multicolor(fil, block16)
    return fil


def _classic_snapmaker(mc: MifareClassic, uid: bytes) -> Optional[dict]:
    """
    Read and decode a Snapmaker Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :return Optional[dict]: decoded fields, None when the read fails
    """
    data = mc.read_blocks(uid, snapmaker_keys(uid), SNAPMAKER_READ_BLOCKS)
    return decode_snapmaker(data) if data else None


def _classic_creality(mc: MifareClassic, uid: bytes, u_key: bytes,
                      d_key: bytes) -> Optional[dict]:
    """
    Read and decode a Creality Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :param u_key: Creality UID key
    :param d_key: Creality data key
    :return Optional[dict]: decoded fields, None when the read fails
    """
    keyA = creality_mifare_key(uid, u_key)
    data = mc.read_blocks(uid, [list(keyA)] * 16, CREALITY_READ_BLOCKS)
    if not data:
        return None
    return decode_creality(_aes_cbc_decrypt(data[64:112], bytes(d_key)))


# ── BigTreeTech MMS / ViViD "BQ Tech" (MIFARE Classic, default FF Key A) ──────
# Port of bigtreetech/BIGTREETECH_MMS klippy/extras/mms/hardware/mfrc522.py:
# MIFARE Classic 1K, tag_version 1000, every sector under the default key
# FFFFFFFFFFFF. Fields are ASCII and little-endian u16 (BTT's nibble offsets
# halved to bytes); color_code is a raw RRGGBB triple. tag_version==1000 is
# the fingerprint, so trying this scheme cannot mis-decode other tags.
# Shared by the ACE2 and ViViD readers.
BTT_DEFAULT_KEY = (0xFF,) * 6
# Blocks decode_btt reads: version+manufacturer@blk1, mfg-date@blk2, material@blk4,
# detailed@blk5, serial@blk6, colour@blk8, diameter/density@blk10, spool
# weight@blk17, temps@blk18, bed temp@blk20.
BTT_READ_BLOCKS = (1, 2, 4, 5, 6, 8, 10, 17, 18, 20)


def btt_keys() -> List[List[int]]:
    """
    Return the 16 per-sector Key-A entries for a BQ Tech tag, all default FF.

    :return List[List[int]]: 16 six-byte keys
    """
    return [list(BTT_DEFAULT_KEY) for _ in range(16)]


def decode_btt(d: bytes) -> Optional[dict]:
    """
    Decode a BigTreeTech MMS "BQ Tech" MIFARE Classic 1K image.

    Block N is at image byte N*16. Fields are LE u16 ints / ASCII strings;
    color_code is a raw RRGGBB triple (single colour, no alpha).

    :param d: tag image
    :return Optional[dict]: decoded fields, None unless tag_version (block 1,
        LE u16) is the 1000 fingerprint
    """
    if len(d) < 336:                                  # need through block 20
        return None
    if _u16(d, 16) != 1000:                           # tag_version fingerprint
        return None
    r, g, b = d[128], d[129], d[130]                  # color_code RRGGBB @ blk8
    diameter = _u16(d, 160)                            # 1750 -> 1.750 mm
    density = _u16(d, 162)                             # 1240 -> 1.240 g/cm^3
    hot_min = _u16(d, 298) or None                     # printing_temperature_min
    hot_max = _u16(d, 300) or None                     # printing_temperature_max
    bed = _u16(d, 320) or _u16(d, 296) or None         # bed_temperature / _max
    return dict(
        manufacturer=_s(d, 18, 14) or "BQ Tech",       # filament_manufacturer
        type=_s(d, 64, 16),                            # filament_material_type
        detailed=_s(d, 80, 16),                        # filament_type_detailed
        color_argb=(0xFF << 24) | (r << 16) | (g << 8) | b,
        weight_g=_u16(d, 272) or None,                 # spool_weight
        diameter_mm=round(diameter / 1000.0, 3) if diameter else 1.75,
        density=round(density / 1000.0, 3) if density else None,
        hotend_min_c=hot_min, hotend_max_c=hot_max,
        bed_temp_c=bed,
        drying_time_h=_u16(d, 288) or None,            # drying_time (hours)
        drying_temp_c=_u16(d, 292) or None,            # drying_temp_max (C)
        sku=_s(d, 96, 16),                             # serial_number
        production=_s(d, 32, 16),                       # manufacture_datetime
    )


def _classic_btt(mc: MifareClassic, uid: bytes) -> Optional[dict]:
    """
    Read and decode a BTT Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :return Optional[dict]: decoded fields, None when the read fails
    """
    data = mc.read_blocks(uid, btt_keys(), BTT_READ_BLOCKS)
    return decode_btt(data) if data else None


# ── QIDI (MIFARE Classic, default key), port of OpenRFID ─────────────
# Sector 1 holds three code bytes, material / colour / manufacturer, and
# zeros after them; the codes index QIDI's tables (AFC_rfid_tables). The
# default FF key is shared with BTT, which is tried first and identified by
# its own fingerprint.
QIDI_READ_BLOCKS = (4, 5, 6)


def decode_qidi(d: bytes) -> Optional[dict]:
    """
    Decode a QIDI MIFARE Classic 1K image.

    :param d: tag image with blocks 4-6 populated
    :return Optional[dict]: decoded fields, None unless sector 1 holds three
        known codes and nothing else
    """
    from extras.AFC_rfid_tables import QIDI_COLORS, QIDI_MATERIALS
    if len(d) < 112:
        return None
    sector = d[64:112]
    material, color, maker = sector[0], sector[1], sector[2]
    if not (material and color and maker) or any(sector[3:]):
        return None
    if material not in QIDI_MATERIALS or color not in QIDI_COLORS:
        return None
    ftype, modifiers = QIDI_MATERIALS[material]
    return dict(manufacturer="QIDI", type=ftype, detailed=modifiers,
                color_argb=0xFF000000 | QIDI_COLORS[color], diameter_mm=1.75)


def _classic_qidi(mc: MifareClassic, uid: bytes) -> Optional[dict]:
    """
    Read and decode a QIDI Classic tag.

    :param mc: the Classic reader
    :param uid: tag UID
    :return Optional[dict]: decoded fields, None when the read fails
    """
    data = mc.read_blocks(uid, btt_keys(), QIDI_READ_BLOCKS)
    return decode_qidi(data) if data else None


# ── TigerTag (NTAG, binary), port of OpenRFID ────────────────────────
# A fixed big-endian layout from page 4. Brand and material are ids into the
# TigerTag registry (AFC_rfid_tables). Dates count seconds from 2000-01-01.
TIGERTAG_IDS = (0x5BF59264, 0xBC0FCB97)          # TigerTag, TigerTag+
_TIGERTAG_EPOCH = 946684800
#: unit id -> grams per unit (g, kg, mg; anything else is read as grams)
_TIGERTAG_UNITS = {1: 1.0, 21: 1.0, 2: 1000.0, 35: 1000.0, 10: 0.001}


def decode_tigertag(d: bytes) -> Optional[dict]:
    """
    Decode a TigerTag / TigerTag+ NTAG image read from page 0.

    :param d: tag image from page 0, at least 52 bytes
    :return Optional[dict]: decoded fields, None without a TigerTag id
    """
    from extras.AFC_rfid_tables import (
        TIGERTAG_ASPECTS, TIGERTAG_BRANDS, TIGERTAG_DIAMETERS,
        TIGERTAG_MATERIALS)
    u = d[16:]
    if len(u) < 36 or struct.unpack_from(">I", u, 0)[0] not in TIGERTAG_IDS:
        return None
    material_id = struct.unpack_from(">H", u, 8)[0]
    brand_id = struct.unpack_from(">H", u, 14)[0]
    ftype, name, density = TIGERTAG_MATERIALS.get(
        material_id, (f"Unknown({material_id})", "", None))
    aspects = [TIGERTAG_ASPECTS[a] for a in (u[10], u[11]) if a in TIGERTAG_ASPECTS]
    detailed = ([name] if name and name != ftype else []) + aspects
    r, g, b, a = u[16], u[17], u[18], u[19]
    amount = (u[20] << 16) | (u[21] << 8) | u[22]
    weight = amount * _TIGERTAG_UNITS.get(u[23], 1.0)
    stamp = struct.unpack_from(">I", u, 32)[0]
    production = ""
    if stamp:
        try:
            production = datetime.fromtimestamp(
                stamp + _TIGERTAG_EPOCH, tz=timezone.utc).strftime("%Y-%m-%d")
        except (OSError, OverflowError, ValueError):
            production = ""
    td = struct.unpack_from(">H", u, 44)[0] / 10.0 if len(u) >= 46 else 0.0
    return dict(
        manufacturer=TIGERTAG_BRANDS.get(brand_id, f"Unknown({brand_id})"),
        type=ftype, detailed=" ".join(detailed),
        color_argb=(a << 24) | (r << 16) | (g << 8) | b,
        weight_g=int(round(weight)) or None,
        diameter_mm=TIGERTAG_DIAMETERS.get(u[13], 1.75),
        density=density,
        hotend_min_c=_u16be(u, 24) or None, hotend_max_c=_u16be(u, 26) or None,
        drying_temp_c=u[28] or None, drying_time_h=u[29] or None,
        bed_temp_c=u[30] or u[31] or None,
        production=production, td=td or None,
    )


def _u16be(d: bytes, p: int) -> int:
    """
    Big-endian u16 at p (TigerTag; the other layouts here are little-endian).

    :param d: buffer
    :param p: offset
    :return int: the value
    """
    return struct.unpack_from(">H", d, p)[0]


# ── NDEF (NTAG): OpenSpool, OpenTag3D, SpoolEase ─────────────────────
# These carry their data as an NDEF message from page 4: OpenSpool as JSON
# (what PrintTag-Web, SpoolPainter and the U1 apps write), OpenTag3D as a
# binary MIME record, SpoolEase as a URL. A message can run past AFC's
# 128-byte first read, so ndef_bytes_needed says how much to read.
#: Largest NTAG read for an NDEF message: NTAG216's 888 user bytes and the
#: 16 bytes before them, whatever a damaged length field claims.
NTAG_MAX_BYTES = 16 + 888
#: NDEF URI prefix codes (NFC Forum RTD-URI), the ones a tag URL uses.
_URI_PREFIXES = {0x00: "", 0x01: "http://www.", 0x02: "https://www.",
                 0x03: "http://", 0x04: "https://"}


def _ndef_tlv(d: bytes) -> Optional[Tuple[int, int]]:
    """
    Find the NDEF message TLV from the first user page.

    NULL TLVs are skipped and lock / memory-control TLVs stepped over; the
    length is one byte, or 0xFF and two more.

    :param d: tag image from page 0
    :return Optional[Tuple[int, int]]: (message offset, length), None when
        the tag has no NDEF message
    """
    i = 16
    while i < len(d):
        t = d[i]
        if t == 0x00:
            i += 1
            continue
        if t == 0xFE or i + 1 >= len(d):
            return None
        length, head = d[i + 1], 2
        if length == 0xFF:
            if i + 3 >= len(d):
                return None
            length, head = (d[i + 2] << 8) | d[i + 3], 4
        if t == 0x03:
            return i + head, length
        i += head + length
    return None


def ndef_bytes_needed(d: bytes) -> Optional[int]:
    """
    How much of an NTAG, read from page 0, holds its NDEF message.

    :param d: tag image from page 0, at least the first TLV header
    :return Optional[int]: bytes from page 0 to the message's end (capped at
        NTAG_MAX_BYTES), None without an NDEF message
    """
    tlv = _ndef_tlv(d)
    return min(sum(tlv), NTAG_MAX_BYTES) if tlv else None


def ndef_records(d: bytes) -> List[Tuple[int, str, bytes]]:
    """
    The records of an NTAG's NDEF message.

    :param d: tag image from page 0, read through the message
    :return List[Tuple[int, str, bytes]]: (TNF, type, payload) per record;
        empty without a message, and a truncated record ends the list
    """
    tlv = _ndef_tlv(d)
    if not tlv:
        return []
    msg = d[tlv[0]:tlv[0] + tlv[1]]
    out, i = [], 0
    while i + 3 <= len(msg):
        hdr, type_len = msg[i], msg[i + 1]
        i += 2
        if hdr & 0x10:                                # short record
            payload_len = msg[i]
            i += 1
        else:
            if i + 4 > len(msg):
                break
            payload_len = struct.unpack_from(">I", msg, i)[0]
            i += 4
        id_len = 0
        if hdr & 0x08:
            id_len = msg[i]
            i += 1
        end = i + type_len + id_len + payload_len
        if end > len(msg):
            break
        rtype = msg[i:i + type_len].decode("ascii", "ignore")
        out.append((hdr & 0x07, rtype, bytes(msg[i + type_len + id_len:end])))
        i = end
        if hdr & 0x40:                                # message end
            break
    return out


def _int_or(v: Any, default: Any = None) -> Any:
    """
    :param v: a JSON or query value
    :param default: returned when v is not a number
    :return Any: v as an int, else default
    """
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return default


def _hex_rgb(v: Any) -> Optional[int]:
    """
    :param v: "RRGGBB" or "#RRGGBB"
    :return Optional[int]: the colour, None when not six hex digits
    """
    s = str(v or "").strip().lstrip("#")
    if len(s) != 6:
        return None
    try:
        return int(s, 16)
    except ValueError:
        return None


def decode_openspool(records: List[Tuple[int, str, bytes]]) -> Optional[dict]:
    """
    Decode an OpenSpool JSON record, with the U1 extended firmware's fields
    (subtype, alpha, additional_color_hexes, bed_min/max_temp, weight,
    diameter), so a tag reads the same here as on a U1.

    :param records: ndef_records() output
    :return Optional[dict]: decoded fields, None without an OpenSpool record
    """
    for tnf, rtype, payload in records:
        if tnf != 0x02 or rtype != "application/json":
            continue
        try:
            data = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            continue
        if not isinstance(data, dict) or data.get("protocol") != "openspool":
            continue
        alpha = data.get("alpha", "FF")
        try:
            alpha = int(alpha, 16) if isinstance(alpha, str) else int(alpha)
        except (TypeError, ValueError):
            alpha = 0xFF
        alpha = max(0, min(0xFF, alpha))
        rgb = _hex_rgb(data.get("color_hex"))
        colors = [(alpha << 24) | (0xFFFFFF if rgb is None else rgb)]
        for extra in list(data.get("additional_color_hexes") or [])[:4]:
            c = _hex_rgb(extra)
            if c is not None:
                colors.append(0xFF000000 | c)
        try:
            diameter = float(data.get("diameter", 1.75))
        except (TypeError, ValueError):
            diameter = 1.75
        bed_min = _int_or(data.get("bed_min_temp"), 0)
        bed_max = _int_or(data.get("bed_max_temp"), 0)
        fil = dict(
            manufacturer=str(data.get("brand") or "Generic"),
            type=str(data.get("type") or "PLA").upper(),
            detailed=str(data.get("subtype") or ""),
            color_argb=colors[0],
            diameter_mm=diameter or 1.75,
            weight_g=_int_or(data.get("weight")) or None,
            hotend_min_c=_int_or(data.get("min_temp")) or None,
            hotend_max_c=_int_or(data.get("max_temp")) or None,
            bed_temp_c=bed_min or bed_max or None,
        )
        if len(colors) > 1:
            fil["color_count"] = len(colors)
            fil["colors_argb"] = colors
        return fil
    return None


#: SpoolEase material codes for AMS-only variants, as their base material.
_SPOOLEASE_ALIASES = {"ABS-S": "ABS", "PA-S": "PA", "PLA-S": "PLA",
                      "TPU-AMS": "TPU"}


def decode_spoolease(records: List[Tuple[int, str, bytes]]) -> Optional[dict]:
    """
    Decode a SpoolEase tag: a URI record to tag.spoolease.io/S1?... whose query
    carries M (material), MS (subtype), B (brand), CC (colours, RRGGBB or
    RRGGBBAA, ';'-separated), NN / NX (nozzle min / max) and WL (weight).

    :param records: ndef_records() output
    :return Optional[dict]: decoded fields, None without a SpoolEase record
    """
    from urllib.parse import parse_qs, urlparse
    for tnf, rtype, payload in records:
        if tnf != 0x01 or rtype != "U" or not payload:
            continue
        url = (_URI_PREFIXES.get(payload[0], "")
               + payload[1:].decode("utf-8", "ignore")).strip()
        parsed = urlparse(url)
        if (parsed.netloc.lower() != "tag.spoolease.io"
                or not parsed.path.startswith("/S1")):
            continue
        q = {k: v[0] for k, v in parse_qs(parsed.query).items() if v}
        material = (q.get("M") or "PLA").upper()
        colors = []
        for code in (q.get("CC") or "").split(";")[:5]:
            code = code.strip().lstrip("#")
            try:
                if len(code) == 6:
                    colors.append(0xFF000000 | int(code, 16))
                elif len(code) == 8:
                    colors.append((int(code[6:], 16) << 24) | int(code[:6], 16))
            except ValueError:
                continue
        fil = dict(
            manufacturer=q.get("B") or "Generic",
            type=_SPOOLEASE_ALIASES.get(material, material),
            detailed=q.get("MS") or "",
            color_argb=colors[0] if colors else 0xFFFFFFFF,
            diameter_mm=1.75,
            weight_g=_int_or(q.get("WL")) or None,
            hotend_min_c=_int_or(q.get("NN")) or None,
            hotend_max_c=_int_or(q.get("NX")) or None,
        )
        if len(colors) > 1:
            fil["color_count"] = len(colors)
            fil["colors_argb"] = colors
        return fil
    return None


#: OpenTag3D v2 fields AFC uses: id -> (offset, length, kind, scale). From
#: the official memory map (opentag3d.info/spec.json, 2.001). Offsets are into
#: the record's payload; bytes past its end read as zero, as the spec says.
_OT3D_FIELDS = {
    "material": (0x02, 5, "str", 1), "material_mod": (0x07, 5, "str", 1),
    "manufacturer": (0x0C, 16, "str", 1), "serial": (0x4C, 32, "str", 1),
    "sku": (0x6C, 16, "str", 1), "mfg_date": (0x84, 4, "date", 1),
    "diameter": (0x8C, 2, "int", 0.001), "print_temp": (0x90, 1, "int", 5),
    "min_print_temp": (0x91, 1, "int", 5), "max_print_temp": (0x92, 1, "int", 5),
    "bed_temp": (0x94, 1, "int", 5), "max_dry_temp": (0x9A, 1, "int", 5),
    "dry_time": (0x9B, 1, "int", 1), "density": (0x9C, 2, "int", 0.001),
    "weight": (0x9E, 2, "int", 1), "td": (0xA7, 1, "int", 0.1),
}
_OT3D_COLORS = (0x3C, 0x40, 0x44, 0x48)


def decode_opentag3d(records: List[Tuple[int, str, bytes]]) -> Optional[dict]:
    """
    Decode an OpenTag3D record (MIME application/opentag3d, major version 2).

    :param records: ndef_records() output
    :return Optional[dict]: decoded fields, None without a v2 record
    """
    for tnf, rtype, payload in records:
        if tnf != 0x02 or rtype != "application/opentag3d" or len(payload) < 2:
            continue
        if struct.unpack_from(">H", payload, 0)[0] // 1000 != 2:
            continue                               # v1 is a different map
        p = payload.ljust(0xE0, b"\x00")
        v: Dict[str, Any] = {}
        for key, (off, n, kind, scale) in _OT3D_FIELDS.items():
            raw = p[off:off + n]
            if kind == "str":
                v[key] = raw.split(b"\x00", 1)[0].decode("utf-8", "ignore").strip()
            elif kind == "date":
                y, m, dd = struct.unpack_from(">H", raw, 0)[0], raw[2], raw[3]
                v[key] = f"{y:04d}-{m:02d}-{dd:02d}" if y and m and dd else ""
            else:
                v[key] = int.from_bytes(raw, "big") * scale
        colors = []
        for i, off in enumerate(_OT3D_COLORS):
            r, g, b, a = p[off:off + 4]
            if i == 0 or (r | g | b | a):
                colors.append((a << 24) | (r << 16) | (g << 8) | b)
        fil = dict(
            manufacturer=v["manufacturer"], type=v["material"],
            detailed=v["material_mod"], color_argb=colors[0],
            diameter_mm=round(v["diameter"], 3) or 1.75,
            density=round(v["density"], 3) or None,
            weight_g=int(v["weight"]) or None,
            hotend_min_c=int(v["min_print_temp"] or v["print_temp"]) or None,
            hotend_max_c=int(v["max_print_temp"] or v["print_temp"]) or None,
            bed_temp_c=int(v["bed_temp"]) or None,
            drying_temp_c=int(v["max_dry_temp"]) or None,
            drying_time_h=int(v["dry_time"]) or None,
            serial=v["serial"], sku=v["sku"], production=v["mfg_date"],
            td=round(v["td"], 1) or None,
        )
        if len(colors) > 1:
            fil["color_count"] = len(colors)
            fil["colors_argb"] = colors
        return fil
    return None


def decode_ndef(d: bytes) -> Optional[dict]:
    """
    Decode whichever NDEF format an NTAG carries.

    :param d: tag image from page 0, read through the message
    :return Optional[dict]: decoded fields, None when no format matched
    """
    records = ndef_records(d)
    if not records:
        return None
    return (decode_openspool(records) or decode_opentag3d(records)
            or decode_spoolease(records))


def _ntag_ndef(mc: MifareClassic, data: bytes) -> Tuple[Optional[dict], bytes]:
    """
    Read on to the end of an NDEF message, then decode it.

    The first read covers 128 bytes. An OpenSpool message often runs past
    that, so the rest is read only when the message's own length says so.

    :param mc: the reader
    :param data: image already read from page 0
    :return tuple: (decoded fields or None, the image as far as it was read)
    """
    need = ndef_bytes_needed(data)
    if need and need > len(data):
        more = mc.read_ntag(need - len(data), start_page=len(data) // 4)
        if more is None:
            return None, data
        data = data + more
    return decode_ndef(data), data


def read_tag(link: Any, bambu_master_key: Optional[bytes] = None,
             bambu_blocks: Any = BAMBU_READ_BLOCKS,
             is_excluded: Optional[Callable[[str], bool]] = None,
             creality_key: Optional[bytes] = None,
             creality_encryption_key: Optional[bytes] = None,
             seen: Optional[List[Any]] = None,
             dump_blocks: Any = None) -> Optional[dict]:
    """
    Activate an ISO14443-A tag and decode its filament when the layout is known.

    MIFARE Classic is tried as Bambu (if a master key is set), Snapmaker,
    Creality (if its keys are set), BTT, then QIDI; NTAG as Anycubic, Elegoo,
    TigerTag, then an NDEF message (OpenSpool, OpenTag3D, SpoolEase), with the
    AFC block overlaid.

    :param link: reader link exposing reg_read / reg_write
    :param bambu_master_key: Bambu master key, None to skip Bambu
    :param bambu_blocks: blocks to read for a Bambu decode
    :param is_excluded: uid_hex -> bool, halts neighbour tags on a shared reader
    :param creality_key: Creality UID key
    :param creality_encryption_key: Creality data key
    :param seen: list collecting every detected (uid, sak, excluded)
    :param dump_blocks: Classic blocks to dump raw into ``raw_blocks``
    :return Optional[dict]: uid, sak, tag_type and filament, None without a tag
    """
    mc = MifareClassic(Mfrc522(link))
    uid, sak = mc.activate(is_excluded=is_excluded, seen=seen)
    if uid is None:
        return None
    res = {"uid": uid.hex(), "sak": sak, "tag_type": None, "filament": None}

    if sak & 0x08:                                   # MIFARE Classic
        res["tag_type"] = "MifareClassic1k"
        schemes = []
        if bambu_master_key:
            schemes.append(lambda u: _classic_bambu(mc, u, bambu_master_key,
                                                    bambu_blocks))
        schemes.append(lambda u: _classic_snapmaker(mc, u))
        if creality_key and creality_encryption_key:
            schemes.append(lambda u: _classic_creality(
                mc, u, creality_key, creality_encryption_key))
        # BTT "BQ Tech" needs no config key, so it is always tried, and its
        # tag_version fingerprint prevents mis-decoding other brands. QIDI
        # shares the default key and goes last: its check is the weaker one.
        schemes.append(lambda u: _classic_btt(mc, u))
        schemes.append(lambda u: _classic_qidi(mc, u))
        # A failed auth deauthenticates the card, so re-select before each later
        # scheme, pinned to this tag's UID. Re-running is_excluded here could
        # halt the active tag itself; pinning still halts any neighbour.
        first_uid_hex = uid.hex()

        def _not_this_tag(h: str) -> bool:
            """
            Whether a UID differs from the first one seen.

            :param h: UID as hex
            :return bool: True when it is a different tag
            """
            return h != first_uid_hex

        for i, scheme in enumerate(schemes):
            u = uid
            if i:
                # Re-select without a SoftReset so the RF field stays up and a
                # marginal at-rest tag is not unpowered between schemes.
                u = mc.activate(is_excluded=_not_this_tag, seen=seen,
                                reset=False)[0]
                if u is None:
                    break
                res["uid"] = u.hex()
            fil = scheme(u)
            if fil:
                res["filament"] = fil
                break
        # Best-effort raw block dump for ACE_RFID_BLOCKS: re-select and attach
        # {block: hex} read with the Bambu keys.
        if dump_blocks and bambu_master_key:
            try:
                u2 = mc.activate(reset=True)[0]
                if u2 is not None:
                    img = mc.read_blocks(u2, bambu_keys(u2, bambu_master_key),
                                         dump_blocks)
                    if img:
                        res["raw_blocks"] = {
                            b: img[b * 16:b * 16 + 16].hex() for b in dump_blocks}
            except Exception:
                pass
    else:                                            # Ultralight / NTAG
        res["tag_type"] = "MifareUltralight"
        data = mc.read_ntag(0x80)
        if data:
            # Which chip, and how much room a write has. Free here: the read
            # starts at page 0, so the CC is already in hand.
            res["chip"], res["user_bytes"] = ntag_chip(data)
            fil = (decode_anycubic(data) or decode_elegoo(data)
                   or decode_tigertag(data))
            if fil is None:
                fil, data = _ntag_ndef(mc, data)
            # Probe the AFC block separately: a plain Ultralight has only 80
            # bytes, so a 160-byte main read would fail on it. A long NDEF
            # read may already hold it.
            start = AFC_BLOCK_PAGE * 4
            if len(data) >= start + AFC_BLOCK_LEN:
                ext = data[start:start + AFC_BLOCK_LEN]
            else:
                ext = mc.read_ntag(AFC_BLOCK_LEN, start_page=AFC_BLOCK_PAGE)
            afc = decode_afc_block(ext, offset=0) if ext else None
            if afc:
                # Ours: the exact weight and the Spoolman id override what the
                # brand layout could only approximate.
                fil = dict(fil or {})
                fil.update(afc)
            res["filament"] = fil
    return res


def _is_writable_classic_block(block: int) -> bool:
    """
    Whether a Classic block is safe to write.

    Sector trailers (every 4th block, keys + access bits) can lock the sector
    for good, and block 0 is the manufacturer block.

    :param block: absolute block number
    :return bool: True for a data block other than block 0
    """
    return block > 0 and (block % 4) != 3


CLASSIC_DEFAULT_KEY = b"\xff" * 6      # a blank / magic card ships with this


def classic_write_block(link: Any, block: int, data16: bytes,
                        key6: bytes = CLASSIC_DEFAULT_KEY,
                        key_type: int = PICC_AUTH_KEY_A
                        ) -> Tuple[Optional[str], Optional[str]]:
    """
    Write one 16-byte data block of a MIFARE Classic tag and read it back.

    Authenticates the block's sector with ``key6`` (default the blank-card
    key), writes, then reads to confirm. Refuses a sector trailer or the
    manufacturer block, since a wrong write there locks the sector for good.

    :param link: reader link exposing reg_read / reg_write
    :param block: absolute block number (a data block)
    :param data16: exactly 16 bytes
    :param key6: 6-byte sector key
    :param key_type: PICC_AUTH_KEY_A or PICC_AUTH_KEY_B
    :return tuple: (uid hex or None, error or None)
    """
    if len(data16) != 16:
        return None, f"data is {len(data16)} bytes, must be 16"
    if not _is_writable_classic_block(block):
        return None, (f"block {block} is a sector trailer or block 0; refusing "
                      f"to write it")
    mfrc = Mfrc522(link)
    mc = MifareClassic(mfrc)
    uid, sak = mc.activate()
    if uid is None:
        return None, "no tag in the reader's field"
    if not sak & SAK_CLASSIC_1K:
        return uid.hex(), (f"that is not a MIFARE Classic tag "
                           f"(sak 0x{sak:02X})")
    if not mfrc.auth(key_type, (block // 4) * 4, bytes(key6), uid):
        mfrc.stop_crypto()
        return uid.hex(), (f"key did not authenticate sector {block // 4} "
                           f"(wrong key for this card?)")
    ok = mfrc.write_classic_block(block, data16)
    back = mfrc.read_block(block)
    mfrc.stop_crypto()
    if not ok:
        return uid.hex(), f"tag did not ACK the write to block {block}"
    if back != data16:
        return uid.hex(), (f"block {block} reads back {back.hex() if back else None}"
                           f", not what was written")
    return uid.hex(), None


def bambu_classic_write_test(link: Any, master_key: bytes,
                             block: int = 13) -> dict:
    """
    Prove the MIFARE Classic write works on a genuine Bambu tag without
    permanently changing it.

    Writes a data block with one byte flipped, confirms it, then restores the
    original and confirms that. Defaults to block 13, which no Bambu decoder
    reads, so even a failed restore is harmless.

    :param link: reader link exposing reg_read / reg_write
    :param master_key: the Bambu master key
    :param block: absolute block to test on (a data block)
    :return dict: {uid, block, wrote, changed, restored, orig, modified, ...}
    """
    if not _is_writable_classic_block(block):
        return {"error": f"block {block} is a sector trailer or block 0; "
                         f"refusing to write it"}
    mfrc = Mfrc522(link)
    mc = MifareClassic(mfrc)
    uid, sak = mc.activate()
    if uid is None:
        return {"error": "no tag in the field"}
    if not sak & SAK_CLASSIC_1K:
        return {"error": f"not a MIFARE Classic tag (sak 0x{sak:02X})",
                "uid": uid.hex()}
    sector = block // 4
    key6 = bytes(bambu_keys(uid, master_key)[sector])
    if not mfrc.auth(PICC_AUTH_KEY_A, sector * 4, key6, uid):
        mfrc.stop_crypto()
        return {"error": f"Key A did not authenticate sector {sector}",
                "uid": uid.hex()}
    orig = mfrc.read_block(block)
    if orig is None:
        mfrc.stop_crypto()
        return {"error": f"could not read block {block}", "uid": uid.hex()}
    # The trailer's access bytes (6-8) show whether a write NAK is a lock
    # rather than a bad frame.
    trailer = mfrc.read_block(sector * 4 + 3)
    access = trailer[6:9].hex() if trailer is not None else None
    modified = bytes([orig[0] ^ 0xFF]) + orig[1:]
    out = {"uid": uid.hex(), "block": block, "orig": orig.hex(),
           "modified": modified.hex(), "access": access}
    # Write the flipped copy and confirm it landed.
    out["wrote"] = mfrc.write_classic_block(block, modified)
    back = mfrc.read_block(block)
    out["changed"] = back == modified
    out["read_after_write"] = back.hex() if back is not None else None
    # Restore the original, always, and confirm.
    out["restored_ack"] = mfrc.write_classic_block(block, orig)
    final = mfrc.read_block(block)
    out["restored"] = final == orig
    out["read_after_restore"] = final.hex() if final is not None else None
    mfrc.stop_crypto()
    return out


def read_bambu(link: Any, master_key: bytes) -> Optional[dict]:
    """
    Convenience Bambu-only read (used by tests).

    :param link: reader link exposing reg_read / reg_write
    :param master_key: Bambu master key
    :return Optional[dict]: decoded fields plus uid, None on failure
    """
    mc = MifareClassic(Mfrc522(link))
    uid, sak = mc.activate()
    if uid is None:
        return None
    data = mc.read_all(uid, bambu_keys(uid, master_key))
    if data is None:
        return None
    out = decode_bambu(data)
    out["uid"] = uid.hex()
    return out


def write_tag(link: Any, payload: bytes,
              is_excluded: Optional[Callable[[str], bool]] = None,
              ) -> Tuple[Optional[str], Optional[str]]:
    """
    Write an Anycubic-layout record to a blank NTAG in the reader's field.

    The counterpart to read_tag: activate whatever is there, refuse anything
    that is not an NTAG, then write and verify. A MIFARE Classic tag is
    rejected by name rather than attempted, because the Classic write is a
    different opcode behind Crypto1 and the keys for a Bambu or Snapmaker tag
    are not ours to forge.

    :param link: reader link exposing reg_read / reg_write
    :param payload: bytes from encode_anycubic
    :param is_excluded: uid_hex -> bool, halts a neighbour tag on a shared
        reader so this station's own tag is the one written
    :return tuple: (uid hex or None, error message or None)
    """
    mc = MifareClassic(Mfrc522(link))
    uid, sak = mc.activate(is_excluded=is_excluded)
    if uid is None:
        return None, "no tag in the reader's field"
    if sak & SAK_CLASSIC_1K:
        return uid.hex(), (
            "that is a MIFARE Classic tag, not an NTAG; this writer only "
            "programs blank NTAG stickers")
    return uid.hex(), mc.write_ntag(ANYCUBIC_FIRST_PAGE, payload)
