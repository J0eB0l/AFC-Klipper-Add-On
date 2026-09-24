"""Unit tests for extras/AFC_rfid_readers.py."""

from __future__ import annotations

import hashlib
import hmac
import struct
from typing import Callable, Dict, Iterable, List, Optional, Tuple

import pytest

from extras.AFC_rfid_readers import (
    _aes_cbc_decrypt,
    _aes_decrypt_block,
    _aes_encrypt_block,
    _bambu_apply_multicolor,
    _classic_btt,
    _is_writable_classic_block,
    bambu_classic_write_test,
    classic_write_block,
    creality_mifare_key,
    decode_afc_block,
    decode_anycubic,
    decode_bambu,
    decode_btt,
    decode_creality,
    decode_elegoo,
    decode_snapmaker,
    encode_anycubic,
    encode_tag_payload,
    hkdf_sha256,
    Mfrc522,
    MifareClassic,
    read_bambu,
    read_tag,
    snapmaker_keys,
    write_tag,
)


# MFRC522 register addresses, from the NXP datasheet.
RFID_REG_COMMAND = 0x01


RFID_REG_COM_IRQ = 0x04


RFID_REG_ERROR = 0x06


RFID_REG_STATUS2 = 0x08


RFID_REG_FIFO_DATA = 0x09


RFID_REG_FIFO_LEVEL = 0x0A


RFID_REG_CONTROL = 0x0C


RFID_REG_BIT_FRAMING = 0x0D


RFID_REG_TX_CONTROL = 0x14


RFID_CMD_TRANSCEIVE = 0x0C


RFID_CMD_MFAUTHENT = 0x0E


RFID_CMD_SOFT_RESET = 0x0F


RFID_ACK = (b"\x0a", 4)


RFID_NAK = (b"\x00", 4)


RFID_UID4 = bytes.fromhex("04a1b2c3")


RFID_UID7 = bytes.fromhex("04a1b2c3d4e5f6")


RFID_BAMBU_MASTER = bytes.fromhex("9A759CF2C4F7CAFF222CB9769B41BC96")


RFID_FF_KEYS = [b"\xff" * 6] * 16


def rfid_crc_a(data: bytes) -> bytes:
    """
    ISO14443-A CRC computed one bit at a time, as the two bytes sent on air.

    :param data: frame body
    :return bytes: CRC low byte, then high byte
    """
    crc = 0x6363
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


def rfid_bambu_keys(uid: bytes, master: bytes) -> List[bytes]:
    """
    Bambu sector keys: 96 bytes of HKDF-SHA256 output cut into 16 keys.

    :param uid: tag UID
    :param master: Bambu master key
    :return List[bytes]: 16 six-byte keys
    """
    prk = hmac.new(master, uid, hashlib.sha256).digest()
    okm, block = b"", b""
    for counter in (1, 2, 3):
        block = hmac.new(prk, block + b"RFID-A\x00" + bytes([counter]),
                         hashlib.sha256).digest()
        okm += block
    return [okm[i:i + 6] for i in range(0, 96, 6)]


def rfid_snapmaker_keys(uid: bytes) -> List[bytes]:
    """
    Snapmaker sector keys, derived straight from the published recipe.

    :param uid: tag UID
    :return List[bytes]: 16 six-byte keys
    """
    prk = hmac.new(b"Snapmaker_qwertyuiop[,.;]", uid, hashlib.sha256).digest()
    return [hmac.new(prk, f"key_a_{s}".encode() + b"\x01", hashlib.sha256).digest()[:6]
            for s in range(16)]


def rfid_cbc_encrypt(data: bytes, key: bytes, iv: bytes = b"\x00" * 16) -> bytes:
    """
    AES-128-CBC encrypt whole blocks with the module's block cipher.

    :param data: plaintext, a multiple of 16 bytes
    :param key: 16-byte key
    :param iv: initialisation vector
    :return bytes: the ciphertext
    """
    out, prev = bytearray(), iv
    for i in range(0, len(data), 16):
        block = _aes_encrypt_block(bytes(x ^ y for x, y in zip(data[i:i + 16], prev)), key)
        out += block
        prev = block
    return bytes(out)


def rfid_bambu_image(nozzle: float = 0.4, spool_width: int = 6620,
                     length: int = 330) -> bytes:
    """
    A Bambu MIFARE Classic 1K image with known field values.

    :param nozzle: nozzle diameter float in block 8
    :param spool_width: spool width u16 in block 10, mm*100
    :param length: filament length u16 in block 14, metres
    :return bytes: the 1024-byte image
    """
    d = bytearray(1024)
    d[32:35] = b"PLA"
    d[64:73] = b"PLA Basic"
    d[80:84] = bytes([0x12, 0x34, 0x56, 0xFF])          # R, G, B, A
    d[84:86] = (1000).to_bytes(2, "little")
    d[88:92] = struct.pack("<f", 1.75)
    d[96:98] = (55).to_bytes(2, "little")
    d[98:100] = (8).to_bytes(2, "little")
    d[102:104] = (55).to_bytes(2, "little")
    d[104:106] = (230).to_bytes(2, "little")
    d[106:108] = (190).to_bytes(2, "little")
    d[140:144] = struct.pack("<f", nozzle)
    d[144:160] = bytes.fromhex("00112233445566778899aabbccddeeff")
    d[164:166] = spool_width.to_bytes(2, "little")
    d[192:208] = b"2026_03_20_11_48"
    d[228:230] = length.to_bytes(2, "little")
    return bytes(d)


def rfid_bambu_fields(**changes: object) -> dict:
    """
    What decode_bambu should report for rfid_bambu_image().

    :param changes: fields that differ from the default image
    :return dict: the expected decode
    """
    fields = {
        "manufacturer": "Bambu", "type": "PLA", "detailed": "PLA Basic",
        "color_argb": 0xFF123456, "weight_g": 1000, "diameter_mm": 1.75,
        "drying_temp_c": 55, "drying_time_h": 8, "bed_temp_c": 55,
        "hotend_max_c": 230, "hotend_min_c": 190, "nozzle_diameter": 0.4,
        "spool_width_mm": 66.2, "length_m": 330,
        "tray_uid": "00112233445566778899aabbccddeeff",
        "production": "2026_03_20_11_48",
    }
    fields.update(changes)
    return fields


def rfid_anycubic_image(length: int = 330, cc: bytes = b"\x00" * 4) -> bytes:
    """
    An Anycubic NTAG image, pages 0-31.

    :param length: filament length u16 at 0x7A
    :param cc: capability container bytes for page 3
    :return bytes: the 128-byte image
    """
    d = bytearray(0x80)
    d[12:16] = cc
    d[0x10:0x14] = b"\x7b\x00\x65\x00"
    d[0x14:0x1D] = b"HPL19-102"
    d[0x28:0x30] = b"Anycubic"
    d[0x3C:0x3F] = b"PLA"
    d[0x50:0x54] = bytes([0xFF, 0xD6, 0x86, 0x00])     # A, B, G, R
    d[0x60:0x62] = (190).to_bytes(2, "little")
    d[0x62:0x64] = (230).to_bytes(2, "little")
    d[0x76:0x78] = (60).to_bytes(2, "little")
    d[0x78:0x7A] = (175).to_bytes(2, "little")
    d[0x7A:0x7C] = length.to_bytes(2, "little")
    return bytes(d)


def rfid_anycubic_fields(**changes: object) -> dict:
    """
    What decode_anycubic should report for rfid_anycubic_image().

    :param changes: fields that differ from the default image
    :return dict: the expected decode
    """
    fields = {
        "manufacturer": "Anycubic", "sku": "HPL19-102", "type": "PLA",
        "color_argb": 0xFF0086D6, "diameter_mm": 1.75, "weight_g": 1000,
        "length_m": 330, "hotend_min_c": 190, "hotend_max_c": 230, "bed_temp_c": 60,
    }
    fields.update(changes)
    return fields


def rfid_snapmaker_image(main_type: int = 1) -> bytes:
    """
    A Snapmaker U1 MIFARE Classic 1K image, no field at its decoder default.

    :param main_type: MAIN_TYPE code at byte 66
    :return bytes: the 1024-byte image
    """
    d = bytearray(1024)
    d[16:24] = b"SnapTest"
    d[66] = main_type
    d[68] = 2                                           # SUB_TYPE Matte
    d[73] = 0x40                                        # alpha 0xFF - 0x40
    d[80:83] = bytes([0x12, 0x34, 0x56])                # R, G, B
    d[96:100] = bytes([0x0D, 0x0C, 0x0B, 0x0A])         # SKU 0x0A0B0C0D, LE
    d[128:130] = (285).to_bytes(2, "little")
    d[130:132] = (750).to_bytes(2, "little")
    d[148:150] = (220).to_bytes(2, "little")
    d[150:152] = (190).to_bytes(2, "little")
    d[154:156] = (60).to_bytes(2, "little")
    d[160:168] = b"20250101"
    return bytes(d)


RFID_SNAPMAKER_FIELDS = {
    "manufacturer": "SnapTest", "type": "PLA", "detailed": "Matte",
    "color_argb": 0xBF123456, "weight_g": 750, "diameter_mm": 2.85,
    "hotend_max_c": 220, "hotend_min_c": 190, "bed_temp_c": 60,
    "sku": "168496141", "production": "20250101",
}


def rfid_btt_image(version: int = 1000, material: str = "PET", diameter: int = 1750,
                   density: int = 0, drying_time: int = 0, drying_temp: int = 0,
                   bed_max: int = 60, bed: int = 60, manufacturer: bytes = b"BQ Tech",
                   weight: int = 1000, hot_min: int = 200, hot_max: int = 240) -> bytes:
    """
    A BTT "BQ Tech" MIFARE Classic 1K image, block N at byte N*16.

    :param version: tag_version fingerprint
    :param material: filament material string
    :param diameter: diameter u16, mm*1000
    :param density: density u16, g/cm^3*1000
    :param drying_time: drying time u16, hours
    :param drying_temp: drying temperature u16
    :param bed_max: bed temperature max u16
    :param bed: bed temperature u16
    :param manufacturer: filament manufacturer string
    :param weight: spool weight u16, grams
    :param hot_min: printing temperature min u16
    :param hot_max: printing temperature max u16
    :return bytes: the 1024-byte image
    """
    d = bytearray(1024)

    def put(block: int, offset: int, raw: bytes) -> None:
        d[block * 16 + offset:block * 16 + offset + len(raw)] = raw

    put(1, 0, version.to_bytes(2, "little"))
    put(1, 2, manufacturer)
    put(2, 0, b"20240812_162600")
    put(4, 0, material.encode())
    put(5, 0, b"PET (CEP)")
    put(6, 0, b"IP243ZCXV67")
    put(8, 0, bytes([0x12, 0x34, 0x56]))
    put(10, 0, diameter.to_bytes(2, "little"))
    put(10, 2, density.to_bytes(2, "little"))
    put(17, 0, weight.to_bytes(2, "little"))
    put(18, 0, drying_time.to_bytes(2, "little"))
    put(18, 4, drying_temp.to_bytes(2, "little"))
    put(18, 8, bed_max.to_bytes(2, "little"))
    put(18, 10, hot_min.to_bytes(2, "little"))
    put(18, 12, hot_max.to_bytes(2, "little"))
    put(20, 0, bed.to_bytes(2, "little"))
    return bytes(d)


def rfid_btt_fields(**changes: object) -> dict:
    """
    What decode_btt should report for rfid_btt_image().

    :param changes: fields that differ from the default image
    :return dict: the expected decode
    """
    fields = {
        "manufacturer": "BQ Tech", "type": "PET", "detailed": "PET (CEP)",
        "color_argb": 0xFF123456, "weight_g": 1000, "diameter_mm": 1.75,
        "density": None, "hotend_min_c": 200, "hotend_max_c": 240, "bed_temp_c": 60,
        "drying_time_h": None, "drying_temp_c": None, "sku": "IP243ZCXV67",
        "production": "20240812_162600",
    }
    fields.update(changes)
    return fields


def rfid_creality_payload(vendor: bytes = b"0276", film: bytes = b"101001",
                          color: bytes = b"0FF5F0B", length: bytes = b"0165",
                          serial: bytes = b"736314") -> bytes:
    """
    A decrypted 48-byte Creality CFS payload.

    :param vendor: 4-char vendor code
    :param film: 6-char film id
    :param color: 7-char colour, "0RRGGBB"
    :param length: 4 hex chars of length in metres
    :param serial: 6-char serial
    :return bytes: the payload
    """
    return b"ABC21" + vendor + b"01" + film + color + length + serial + b"\x00" * 14


def rfid_creality_fields(**changes: object) -> dict:
    """
    What decode_creality should report for rfid_creality_payload().

    :param changes: fields that differ from the default payload
    :return dict: the expected decode
    """
    fields = {
        "manufacturer": "Creality", "type": "PLA", "detailed": "",
        "color_argb": 0xFFFF5F0B, "sku": "101001", "diameter_mm": 1.75,
        "weight_g": 500, "length_m": 357, "serial": "736314", "production": "ABC21",
    }
    fields.update(changes)
    return fields


def rfid_afc_block(version: int = 1, weight: int = 0, spool: int = 0, density: int = 0,
                   dry_temp: int = 0, dry_hours: int = 0) -> bytes:
    """
    An AFC extension block laid out by hand.

    :param version: format version byte
    :param weight: weight u16, grams
    :param spool: Spoolman spool id u32
    :param density: density u16, g/cm^3*1000
    :param dry_temp: drying temperature u16
    :param dry_hours: drying time u16, hours
    :return bytes: the 32-byte block
    """
    return (b"AFC1" + bytes([version, 0]) + weight.to_bytes(2, "little")
            + spool.to_bytes(4, "little") + density.to_bytes(2, "little")
            + dry_temp.to_bytes(2, "little") + dry_hours.to_bytes(2, "little") + bytes(14))


class RfidScriptedLink:
    """Register link that answers each frame sent with the next scripted reply."""

    def __init__(self, *replies: Optional[Tuple[bytes, int]], error: int = 0x00,
                 comirq: int = 0x30, regs: Optional[Dict[int, int]] = None) -> None:
        """
        Script a run of replies.

        :param replies: (rx bytes, rx last bits) per frame, None for no answer
        :param error: ErrorReg value after every frame
        :param comirq: ComIrqReg value after a frame that got a reply
        :param regs: initial register values
        """
        self.replies = list(replies)
        self.error = error
        self.comirq = comirq
        self.regs: Dict[int, int] = dict(regs or {})
        self.fifo = bytearray()
        self.frames: List[bytes] = []
        self.writes: List[Tuple[int, int]] = []
        self.reads: Dict[int, int] = {}

    def reg_read(self, reg: int) -> int:
        """
        Read a register.

        :param reg: register address
        :return int: its value
        """
        self.reads[reg] = self.reads.get(reg, 0) + 1
        if reg == RFID_REG_FIFO_LEVEL:
            return len(self.fifo)
        if reg == RFID_REG_FIFO_DATA:
            return self.fifo.pop(0) if self.fifo else 0
        return self.regs.get(reg, 0)

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write a register; starting a command plays the next reply.

        :param reg: register address
        :param val: value written
        """
        self.writes.append((reg, val))
        if reg == RFID_REG_FIFO_LEVEL:
            if val & 0x80:
                self.fifo.clear()
            return
        if reg == RFID_REG_FIFO_DATA:
            self.fifo.append(val)
            return
        self.regs[reg] = val
        if ((reg == RFID_REG_COMMAND
             and val == RFID_CMD_MFAUTHENT)
            or (reg == RFID_REG_BIT_FRAMING
                and val & 0x80
                and self.regs.get(RFID_REG_COMMAND) == RFID_CMD_TRANSCEIVE)):
            self._answer()

    def _answer(self) -> None:
        """Take the sent frame and load the next scripted reply."""
        self.frames.append(bytes(self.fifo))
        self.fifo.clear()
        reply = self.replies.pop(0) if self.replies else None
        self.regs[RFID_REG_ERROR] = self.error
        if reply is None:
            self.regs[RFID_REG_COM_IRQ] = 0x01
            self.regs[RFID_REG_CONTROL] = 0
            return
        data, bits = reply
        self.fifo = bytearray(data)
        self.regs[RFID_REG_COM_IRQ] = self.comirq
        self.regs[RFID_REG_CONTROL] = bits


class RfidSimTag:
    """An ISO14443-A tag for RfidSimLink: a MIFARE Classic 1K or an NTAG."""

    def __init__(self, uid: bytes, image: bytes = b"", *, ntag: bool = False,
                 pages: int = 45, keys_a: Optional[List[bytes]] = None,
                 sak: Optional[int] = None, ignores: Iterable[str] = (),
                 nak_writes: Iterable[int] = (), unreadable: Iterable[int] = (),
                 corrupt: Optional[Dict[int, bytes]] = None,
                 lost_acks: Optional[Dict[int, int]] = None,
                 wake_limit: Optional[int] = None) -> None:
        """
        Describe one tag.

        :param uid: 4- or 7-byte UID
        :param image: initial memory from byte 0
        :param ntag: True for an NTAG (pages, no auth), False for a Classic 1K
        :param pages: NTAG page count
        :param keys_a: Classic per-sector Key A, default all FF
        :param sak: final SAK, default 0x00 for NTAG and 0x08 for Classic
        :param ignores: frames the tag never answers (reqa, wupa, anticoll1,
            select1, anticoll2, select2)
        :param nak_writes: pages or blocks whose write is NAKed
        :param unreadable: pages or blocks whose READ is NAKed
        :param corrupt: page or block -> bytes stored instead of the write
        :param lost_acks: page -> writes that store but send no ACK
        :param wake_limit: REQA/WUPA answers before the tag leaves the field
        """
        self.uid = bytes(uid)
        self.ntag = ntag
        self.pages = pages
        self.memory = bytearray(pages * 4 if ntag else 1024)
        self.memory[:len(image)] = image
        self.keys_a = list(keys_a) if keys_a is not None else list(RFID_FF_KEYS)
        self.sak = sak if sak is not None else (0x00 if ntag else 0x08)
        self.ignores = set(ignores)
        self.nak_writes = set(nak_writes)
        self.unreadable = set(unreadable)
        self.corrupt = dict(corrupt or {})
        self.lost_acks = dict(lost_acks or {})
        self.wake_limit = wake_limit
        self.wakes = 0
        self.state = "idle"
        self.level = 1
        self.crypto_sector: Optional[int] = None
        self.pending_block: Optional[int] = None

    def cascade_bytes(self, level: int) -> bytes:
        """
        The four UID bytes a cascade level carries.

        :param level: 1 or 2
        :return bytes: those bytes, the cascade tag first at level 1 of a 7-byte UID
        """
        if len(self.uid) == 4:
            return self.uid
        return b"\x88" + self.uid[:3] if level == 1 else self.uid[3:7]

    def wake(self) -> None:
        """Answer a REQA/WUPA: back to READY at cascade level 1."""
        self.wakes += 1
        self.state = "ready"
        self.level = 1
        self.crypto_sector = None
        self.pending_block = None

    def read(self, arg: int) -> Optional[Tuple[bytes, int]]:
        """
        Answer a READ.

        :param arg: page (NTAG) or block (Classic)
        :return Optional[Tuple[bytes, int]]: 16 bytes + CRC, or a NAK
        """
        if self.ntag:
            if (arg >= self.pages
                or arg in self.unreadable):
                return RFID_NAK
            data = b"".join(bytes(self.memory[(p % self.pages) * 4:(p % self.pages) * 4 + 4])
                            for p in range(arg, arg + 4))
        else:
            if (self.crypto_sector != arg // 4
                or arg in self.unreadable):
                self.state = "idle"
                self.crypto_sector = None
                return RFID_NAK
            data = bytes(self.memory[arg * 16:arg * 16 + 16])
        return data + rfid_crc_a(data), 0

    def write_page(self, page: int, data: bytes) -> Optional[Tuple[bytes, int]]:
        """
        Answer an NTAG WRITE (0xA2).

        :param page: page number
        :param data: 4 bytes
        :return Optional[Tuple[bytes, int]]: ACK, NAK, or None when the ACK is lost
        """
        if not self.ntag:
            return None
        if (page < 2
            or page >= self.pages
            or page in self.nak_writes):
            return RFID_NAK
        self.memory[page * 4:page * 4 + 4] = self.corrupt.get(page, data)
        if self.lost_acks.get(page, 0):
            self.lost_acks[page] -= 1
            return None
        return RFID_ACK

    def command_phase(self, block: int) -> Optional[Tuple[bytes, int]]:
        """
        Answer the first half of a Classic WRITE (0xA0).

        :param block: block number
        :return Optional[Tuple[bytes, int]]: ACK or NAK
        """
        if self.ntag:
            return None
        if (self.crypto_sector != block // 4
            or block in self.nak_writes):
            self.state = "idle"
            self.crypto_sector = None
            return RFID_NAK
        self.pending_block = block
        return RFID_ACK

    def data_phase(self, data: bytes) -> Tuple[bytes, int]:
        """
        Answer the 16 data bytes of a Classic WRITE.

        :param data: the block contents
        :return Tuple[bytes, int]: ACK
        """
        block = self.pending_block or 0
        self.pending_block = None
        self.memory[block * 16:block * 16 + 16] = self.corrupt.get(block, data)
        return RFID_ACK


class RfidSimLink:
    """
    Register-level MFRC522 with tags in its RF field.

    Commands run when started (Transceive on StartSend, MFAuthent on the
    CommandReg write). ``trace`` names every reader event in order.
    """

    def __init__(self, *tags: RfidSimTag) -> None:
        """
        Power up with the antenna off.

        :param tags: tags in the field, in anticollision order
        """
        self.tags = list(tags)
        self.regs: Dict[int, int] = {RFID_REG_TX_CONTROL: 0x80}
        self.fifo = bytearray()
        self.frames: List[bytes] = []
        self.trace: List[str] = []

    def field_on(self) -> bool:
        """
        Whether an antenna driver is on.

        :return bool: True when the RF field is up
        """
        return bool(self.regs.get(RFID_REG_TX_CONTROL, 0) & 0x03)

    def power_on(self) -> None:
        """Turn the field on without logging it, as scenario setup."""
        self.regs[RFID_REG_TX_CONTROL] = 0x83

    def power_and_select(self, tag: RfidSimTag) -> None:
        """
        Field on and one tag already selected, as scenario setup.

        :param tag: the tag to leave ACTIVE
        """
        self.power_on()
        tag.state = "active"

    def active(self) -> Optional[RfidSimTag]:
        """
        The selected tag.

        :return Optional[RfidSimTag]: the ACTIVE tag, None when none is
        """
        return next((t for t in self.tags if t.state == "active"), None)

    def reg_read(self, reg: int) -> int:
        """
        Read a register.

        :param reg: register address
        :return int: its value
        """
        if reg == RFID_REG_FIFO_LEVEL:
            return len(self.fifo)
        if reg == RFID_REG_FIFO_DATA:
            return self.fifo.pop(0) if self.fifo else 0
        return self.regs.get(reg, 0)

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write a register and run whatever that starts.

        :param reg: register address
        :param val: value written
        """
        if reg == RFID_REG_FIFO_LEVEL:
            if val & 0x80:
                self.fifo.clear()
            return
        if reg == RFID_REG_FIFO_DATA:
            self.fifo.append(val)
            return
        was_on = self.field_on()
        self.regs[reg] = val
        if (reg == RFID_REG_TX_CONTROL
            and self.field_on()
            and not was_on):
            self.trace.append("FIELD-ON")
        elif (reg == RFID_REG_COMMAND
              and val == RFID_CMD_SOFT_RESET):
            self._soft_reset()
        elif (reg == RFID_REG_COMMAND
              and val == RFID_CMD_MFAUTHENT):
            self._authenticate(self._take_frame())
        elif (reg == RFID_REG_BIT_FRAMING
              and val & 0x80
              and self.regs.get(RFID_REG_COMMAND) == RFID_CMD_TRANSCEIVE):
            self._transceive(self._take_frame(), val & 0x07)
        elif (reg == RFID_REG_STATUS2
              and not val & 0x08):
            self.trace.append("CRYPTO-OFF")
            for tag in self.tags:
                tag.crypto_sector = None

    def _take_frame(self) -> bytes:
        """
        Empty the FIFO into a sent frame.

        :return bytes: the frame
        """
        frame = bytes(self.fifo)
        self.fifo.clear()
        self.frames.append(frame)
        return frame

    def _soft_reset(self) -> None:
        """SoftReset: registers to defaults, antenna off, every tag unpowered."""
        self.trace.append("SOFT-RESET")
        self.regs = {RFID_REG_TX_CONTROL: 0x80}
        self.fifo.clear()
        for tag in self.tags:
            tag.state = "idle"
            tag.crypto_sector = None
            tag.pending_block = None

    def _authenticate(self, frame: bytes) -> None:
        """
        MFAuthent: start Crypto1 when the key matches the active Classic tag.

        :param frame: key type, block, 6-byte key, 4-byte UID
        """
        block = frame[1] if len(frame) > 1 else 0
        self.trace.append(f"AUTH {block}" if frame[:1] == b"\x60" else f"AUTH-B {block}")
        self.regs[RFID_REG_COM_IRQ] = 0x10
        self.regs[RFID_REG_ERROR] = 0x00
        tag = self.active() if self.field_on() else None
        status = self.regs.get(RFID_REG_STATUS2, 0)
        if (tag is not None
            and not tag.ntag
            and len(frame) == 12
            and frame[0] == 0x60
            and frame[2:8] == tag.keys_a[block // 4]
            and frame[8:12] == tag.uid[:4]):
            tag.crypto_sector = block // 4
            self.regs[RFID_REG_STATUS2] = status | 0x08
            return
        self.regs[RFID_REG_STATUS2] = status & 0xF7
        if tag is not None:
            tag.state = "idle"
            tag.crypto_sector = None

    def _name(self, frame: bytes, tx_bits: int) -> str:
        """
        Name a sent frame for the trace.

        :param frame: the frame
        :param tx_bits: valid bits in its last byte, 0 meaning 8
        :return str: the event name
        """
        tag = self.active()
        crc_ok = (len(frame) >= 3
                  and rfid_crc_a(frame[:-2]) == frame[-2:])
        if (tag is not None
            and tag.pending_block is not None
            and len(frame) == 18):
            return f"DATA {tag.pending_block}" if crc_ok else f"BAD-FRAME {frame.hex()}"
        if (tx_bits == 7
            and frame in (b"\x26", b"\x52")):
            return "REQA" if frame == b"\x26" else "WUPA"
        if frame in (b"\x93\x20", b"\x95\x20"):
            return "ANTICOLL1" if frame[0] == 0x93 else "ANTICOLL2"
        if not crc_ok:
            return f"BAD-FRAME {frame.hex()}"
        body = frame[:-2]
        if (len(body) == 7
            and body[0] in (0x93, 0x95)
            and body[1] == 0x70):
            return f"SELECT{1 if body[0] == 0x93 else 2} {body[2:6].hex()}"
        if body == b"\x50\x00":
            return "HLTA"
        if (len(body) == 2
            and body[0] in (0x30, 0xA0)):
            return f"{'READ' if body[0] == 0x30 else 'WRITE-CLASSIC'} {body[1]}"
        if (len(body) == 6
            and body[0] == 0xA2):
            return f"WRITE {body[1]}"
        return f"UNKNOWN {frame.hex()}"

    def _transceive(self, frame: bytes, tx_bits: int) -> None:
        """
        Transceive: deliver a frame to the field and load the answer.

        :param frame: the frame sent
        :param tx_bits: valid bits in its last byte
        """
        name = self._name(frame, tx_bits)
        self.trace.append(name)
        reply = self._respond(name, frame) if self.field_on() else None
        if reply is None:
            self.regs[RFID_REG_COM_IRQ] = 0x01
            self.regs[RFID_REG_CONTROL] = 0
            self.regs[RFID_REG_ERROR] = 0x00
            return
        data, bits = reply
        self.fifo = bytearray(data)
        self.regs[RFID_REG_COM_IRQ] = 0x30
        self.regs[RFID_REG_CONTROL] = bits
        # A 4-bit ACK/NAK carries no parity, so the chip flags ProtocolErr.
        self.regs[RFID_REG_ERROR] = 0x01 if bits else 0x00

    def _respond(self, name: str, frame: bytes) -> Optional[Tuple[bytes, int]]:
        """
        The field's answer to a named frame.

        :param name: the event name from _name
        :param frame: the frame sent
        :return Optional[Tuple[bytes, int]]: (rx bytes, rx last bits), None for silence
        """
        verb, _, arg = name.partition(" ")
        if verb in ("REQA", "WUPA"):
            return self._wake(verb == "WUPA")
        if verb.startswith("ANTICOLL"):
            return self._anticoll(int(verb[-1]))
        if verb.startswith("SELECT"):
            return self._select(int(verb[-1]), frame[2:6], frame[6])
        tag = self.active()
        if tag is None:
            return None
        if verb == "HLTA":
            tag.state = "halt"
            tag.crypto_sector = None
            return None
        if verb == "READ":
            return tag.read(int(arg))
        if verb == "WRITE":
            return tag.write_page(int(arg), frame[2:6])
        if verb == "WRITE-CLASSIC":
            return tag.command_phase(int(arg))
        if verb == "DATA":
            return tag.data_phase(frame[:16])
        return None

    def _wake(self, wupa: bool) -> Optional[Tuple[bytes, int]]:
        """
        REQA wakes tags that are not halted; WUPA wakes halted ones too.

        :param wupa: True for WUPA
        :return Optional[Tuple[bytes, int]]: the ATQA, None when nothing woke
        """
        word = "wupa" if wupa else "reqa"
        woken = [t for t in self.tags
                 if ((wupa
                      or t.state != "halt")
                     and word not in t.ignores
                     and (t.wake_limit is None
                          or t.wakes < t.wake_limit))]
        for tag in woken:
            tag.wake()
        if not woken:
            return None
        return (b"\x44\x00" if len(woken[0].uid) == 7 else b"\x04\x00"), 0

    def _anticoll(self, level: int) -> Optional[Tuple[bytes, int]]:
        """
        The first READY tag at this level answers with its UID bytes and BCC.

        :param level: cascade level
        :return Optional[Tuple[bytes, int]]: four UID bytes and BCC, None for silence
        """
        for tag in self.tags:
            if (tag.state == "ready"
                and tag.level == level):
                if f"anticoll{level}" in tag.ignores:
                    return None
                part = tag.cascade_bytes(level)
                return part + bytes([part[0] ^ part[1] ^ part[2] ^ part[3]]), 0
        return None

    def _select(self, level: int, uid4: bytes, bcc: int) -> Optional[Tuple[bytes, int]]:
        """
        SELECT: the matching tag answers its SAK, other READY tags drop to IDLE.

        :param level: cascade level
        :param uid4: the UID bytes selected
        :param bcc: the BCC sent
        :return Optional[Tuple[bytes, int]]: SAK + CRC, None for silence
        """
        if bcc != uid4[0] ^ uid4[1] ^ uid4[2] ^ uid4[3]:
            return None
        chosen: Optional[RfidSimTag] = None
        for tag in self.tags:
            if (tag.state != "ready"
                or tag.level != level):
                continue
            if (chosen is None
                and tag.cascade_bytes(level) == uid4
                and f"select{level}" not in tag.ignores):
                chosen = tag
            else:
                tag.state = "idle"
        if chosen is None:
            return None
        if (len(chosen.uid) == 7
            and level == 1):
            chosen.level = 2
            sak = 0x04
        else:
            chosen.state = "active"
            sak = chosen.sak
        return bytes([sak]) + rfid_crc_a(bytes([sak])), 0


class TestMfrc522AntennaOn:
    def test_turns_on_when_off(self):
        link = RfidScriptedLink(regs={RFID_REG_TX_CONTROL: 0x80})
        Mfrc522(link).antenna_on()
        assert link.writes == [(RFID_REG_TX_CONTROL, 0x83)]
        assert link.regs[RFID_REG_TX_CONTROL] == 0x83

    @pytest.mark.parametrize("tx_control", [0x01, 0x02, 0x03])
    def test_noop_when_already_on(self, tx_control):
        link = RfidScriptedLink(regs={RFID_REG_TX_CONTROL: tx_control})
        Mfrc522(link).antenna_on()
        assert link.writes == []
        assert link.regs[RFID_REG_TX_CONTROL] == tx_control


class TestMfrc522ToCard:
    SENT_REQA = [(RFID_REG_COMMAND, 0x00), (RFID_REG_FIFO_LEVEL, 0x80),
                 (RFID_REG_FIFO_DATA, 0x26), (RFID_REG_COMMAND, RFID_CMD_TRANSCEIVE),
                 (RFID_REG_BIT_FRAMING, 0x80)]

    def test_timer_irq_returns_failure(self):
        link = RfidScriptedLink(None)
        result = Mfrc522(link)._to_card(RFID_CMD_TRANSCEIVE, b"\x26", 7)
        assert result == (False, b"", 0)
        # The timer exit clears StartSend, then returns before ErrorReg is read.
        assert link.writes == self.SENT_REQA + [(RFID_REG_BIT_FRAMING, 0x00)]
        assert link.reads[RFID_REG_COM_IRQ] == 1
        assert RFID_REG_ERROR not in link.reads

    @pytest.mark.parametrize("error", [0x01, 0x02, 0x08, 0x10])
    def test_error_reg_returns_failure(self, error):
        link = RfidScriptedLink((b"\x04\x00", 0), error=error)
        result = Mfrc522(link)._to_card(RFID_CMD_TRANSCEIVE, b"\x26", 7)
        assert result == (False, b"", 0)
        assert link.writes == self.SENT_REQA + [(RFID_REG_BIT_FRAMING, 0x00)]
        assert link.fifo == bytearray(b"\x04\x00")

    def test_error_bits_outside_the_mask_are_ignored(self):
        link = RfidScriptedLink((b"\x04\x00", 0), error=0x04)
        result = Mfrc522(link)._to_card(RFID_CMD_TRANSCEIVE, b"\x26", 7)
        assert result == (True, b"\x04\x00", 0)

    def test_a_narrower_err_mask_lets_protocol_error_through(self):
        link = RfidScriptedLink((b"\x0a", 4), error=0x01)
        result = Mfrc522(link)._to_card(RFID_CMD_TRANSCEIVE, b"\xa2", err_mask=0x12)
        assert result == (True, b"\x0a", 4)

    def test_poll_times_out_then_reads_fifo(self):
        link = RfidScriptedLink((b"\xab\xcd", 3), comirq=0x00)
        result = Mfrc522(link)._to_card(RFID_CMD_TRANSCEIVE, b"\x26", 7)
        assert result == (True, b"\xab\xcd", 3)
        assert link.reads[RFID_REG_COM_IRQ] == 2000
        assert link.fifo == bytearray()

    @pytest.mark.parametrize("comirq", [0x10, 0x20])
    def test_idle_or_rx_irq_alone_ends_the_poll(self, comirq):
        link = RfidScriptedLink((b"\x04\x00", 0), comirq=comirq)
        result = Mfrc522(link)._to_card(RFID_CMD_TRANSCEIVE, b"\x26", 7)
        assert result == (True, b"\x04\x00", 0)
        assert link.reads[RFID_REG_COM_IRQ] == 1

    def test_authent_returns_no_rx(self):
        link = RfidScriptedLink((b"\x99", 0))
        result = Mfrc522(link)._to_card(RFID_CMD_MFAUTHENT, b"\x60\x00")
        assert result == (True, b"", 0)
        assert link.writes == [
            (RFID_REG_COMMAND, 0x00), (RFID_REG_FIFO_LEVEL, 0x80),
            (RFID_REG_FIFO_DATA, 0x60), (RFID_REG_FIFO_DATA, 0x00),
            (RFID_REG_COMMAND, RFID_CMD_MFAUTHENT), (RFID_REG_BIT_FRAMING, 0x00)]
        assert link.frames == [b"\x60\x00"]
        assert link.fifo == bytearray(b"\x99")


class TestMfrc522Anticoll:
    def test_transceive_failure_returns_none(self):
        # Two tags answering at once: a well-formed reply flagged CollErr.
        link = RfidScriptedLink((b"\x01\x02\x03\x04\x04", 0), error=0x08)
        assert Mfrc522(link).anticoll() is None
        assert link.frames == [b"\x93\x20"]

    def test_bad_bcc_returns_none(self):
        link = RfidScriptedLink((b"\x01\x02\x03\x04\x05", 0))
        assert Mfrc522(link).anticoll() is None

    def test_good_bcc_returns_uid(self):
        link = RfidScriptedLink((b"\x01\x02\x03\x04\x04", 0))
        assert Mfrc522(link).anticoll() == b"\x01\x02\x03\x04"
        assert link.writes[0] == (RFID_REG_BIT_FRAMING, 0x00)
        assert link.frames == [b"\x93\x20"]

    def test_a_short_reply_returns_none(self):
        link = RfidScriptedLink((b"\x01\x02\x03\x00", 0))
        assert Mfrc522(link).anticoll() is None

    def test_cascade_level_two_sends_its_own_command(self):
        link = RfidScriptedLink((b"\xd4\xe5\xf6\x07\xc0", 0))
        assert Mfrc522(link).anticoll(0x95) == b"\xd4\xe5\xf6\x07"
        assert link.frames == [b"\x95\x20"]


class TestMfrc522WriteClassicBlock:
    COMMAND_13 = b"\xa0\x0d" + rfid_crc_a(b"\xa0\x0d")

    def test_two_acked_phases_is_a_success(self):
        link = RfidScriptedLink(RFID_ACK, RFID_ACK)
        assert Mfrc522(link).write_classic_block(13, b"\x11" * 16) is True
        assert link.frames == [b"\xa0\x0d" + rfid_crc_a(b"\xa0\x0d"),
                               b"\x11" * 16 + rfid_crc_a(b"\x11" * 16)]

    def test_the_command_frame_is_a0_block(self):
        link = RfidScriptedLink(RFID_ACK, RFID_ACK)
        Mfrc522(link).write_classic_block(9, b"\x22" * 16)
        assert link.frames[0] == b"\xa0\x09" + rfid_crc_a(b"\xa0\x09")

    def test_a_naked_command_phase_fails(self):
        link = RfidScriptedLink(RFID_NAK)
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is False
        assert link.frames == [b"\xa0\x0d" + rfid_crc_a(b"\xa0\x0d")]

    def test_a_naked_data_phase_fails(self):
        link = RfidScriptedLink(RFID_ACK, RFID_NAK)
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is False
        assert link.frames == [self.COMMAND_13, b"\x33" * 16 + rfid_crc_a(b"\x33" * 16)]

    def test_sixteen_bytes_required(self):
        link = RfidScriptedLink(RFID_ACK, RFID_ACK)
        with pytest.raises(ValueError) as err:
            Mfrc522(link).write_classic_block(13, b"\x00" * 15)
        assert str(err.value) == "Classic block write needs 16 bytes, got 15"
        assert link.frames == []

    def test_a_silent_tag_fails_and_clears_start_send(self):
        link = RfidScriptedLink(None)
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is False
        assert link.regs[RFID_REG_BIT_FRAMING] == 0x00
        assert link.frames == [self.COMMAND_13]
        # The TimerIRq exit returns before ErrorReg is read.
        assert link.reads[RFID_REG_COM_IRQ] == 1
        assert RFID_REG_ERROR not in link.reads

    @pytest.mark.parametrize("error", [0x02, 0x10])
    def test_a_parity_or_buffer_error_fails(self, error):
        link = RfidScriptedLink(RFID_ACK, RFID_ACK, error=error)
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is False
        assert link.frames == [self.COMMAND_13]

    def test_a_whole_byte_reply_is_not_an_ack(self):
        # The data phase would ACK, so only the command-phase check can fail it.
        link = RfidScriptedLink((b"\x0a", 0), RFID_ACK)
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is False
        assert link.frames == [self.COMMAND_13]

    def test_a_high_nibble_on_the_ack_is_ignored(self):
        link = RfidScriptedLink((b"\xfa", 4), (b"\xfa", 4))
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is True
        assert link.frames == [self.COMMAND_13, b"\x33" * 16 + rfid_crc_a(b"\x33" * 16)]

    def test_a_two_byte_reply_is_not_an_ack(self):
        link = RfidScriptedLink((b"\x0a\x0a", 4), RFID_ACK)
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is False
        assert link.frames == [self.COMMAND_13]

    def test_a_reply_without_rx_irq_polls_out_then_reads_the_ack(self):
        # Neither RxIRq nor TimerIRq: both phases poll to the end, then read the FIFO.
        link = RfidScriptedLink(RFID_ACK, RFID_ACK, comirq=0x10)
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is True
        assert link.frames == [self.COMMAND_13, b"\x33" * 16 + rfid_crc_a(b"\x33" * 16)]
        assert link.reads[RFID_REG_COM_IRQ] == 8000
        # One read for the FIFO flush and one for the ACK level, per phase.
        assert link.reads[RFID_REG_FIFO_LEVEL] == 4

    def test_an_empty_fifo_after_rx_irq_is_not_an_ack(self):
        link = RfidScriptedLink((b"", 4), RFID_ACK)
        assert Mfrc522(link).write_classic_block(13, b"\x33" * 16) is False
        assert link.frames == [self.COMMAND_13]
        assert link.reads[RFID_REG_COM_IRQ] == 1
        # One read for the FIFO flush, then eight waiting for a level.
        assert link.reads[RFID_REG_FIFO_LEVEL] == 9
        assert RFID_REG_FIFO_DATA not in link.reads


class TestMfrc522WritePage:
    def test_the_frame_is_a2_page_data_crc(self):
        link = RfidScriptedLink(RFID_ACK)
        mfrc = Mfrc522(link)
        assert mfrc.write_page(7, b"\x01\x02\x03\x04") is True
        assert link.frames == [b"\xa2\x07\x01\x02\x03\x04"
                               + rfid_crc_a(b"\xa2\x07\x01\x02\x03\x04")]
        assert mfrc.last_write_ack == (True, "0x0", 1, 4, "0xa")

    def test_the_crc_is_the_one_the_reader_computes(self):
        link = RfidScriptedLink(RFID_ACK)
        mfrc = Mfrc522(link)
        assert mfrc.write_page(9, b"\xde\xad\xbe\xef") is True
        body = b"\xa2\x09\xde\xad\xbe\xef"
        assert link.frames[0][6:] == rfid_crc_a(body)
        assert mfrc.last_write_ack == (True, "0x0", 1, 4, "0xa")

    @pytest.mark.parametrize("reply,want_ack", [(b"\x0a", "0xa"), (b"\xfa", "0xfa")])
    def test_only_the_4_bit_ack_counts_as_written(self, reply, want_ack):
        mfrc = Mfrc522(RfidScriptedLink((reply, 4)))
        assert mfrc.write_page(7, b"\x00" * 4) is True
        assert mfrc.last_write_ack == (True, "0x0", 1, 4, want_ack)

    @pytest.mark.parametrize("reply,comirq,want", [
        ((b"\x00", 4), 0x30, (True, "0x0", 1, 4, "0x0")),          # NAK
        ((b"\x01", 4), 0x30, (True, "0x0", 1, 4, "0x1")),          # NAK, bad arg
        ((b"\x0a", 0), 0x30, (True, "0x0", 1, 0, "0xa")),          # a whole byte
        ((b"\x0a\x0a", 4), 0x30, (True, "0x0", 2, 4, "0xa")),      # too long
        ((b"", 0), 0x30, (True, "0x0", 0, 0, None)),                # nothing came back
        ((b"\x0a", 4), 0x01, (False, "0x0", 1, 4, "0xa")),         # no RxIRq
        (None, 0x30, (False, "0x0", 0, 0, None)),                   # timed out
    ])
    def test_anything_else_is_a_failed_write(self, reply, comirq, want):
        mfrc = Mfrc522(RfidScriptedLink(reply, comirq=comirq))
        assert mfrc.write_page(7, b"\x00" * 4) is False
        assert mfrc.last_write_ack == want

    def test_a_reply_without_rx_or_timer_irq_polls_out(self):
        link = RfidScriptedLink(RFID_ACK, comirq=0x10)
        mfrc = Mfrc522(link)
        assert mfrc.write_page(7, b"\x00" * 4) is False
        assert link.reads[RFID_REG_COM_IRQ] == 4000
        assert mfrc.last_write_ack == (False, "0x0", 1, 4, "0xa")

    def test_a_short_frame_protocol_error_is_not_treated_as_failure(self):
        mfrc = Mfrc522(RfidScriptedLink(RFID_ACK, error=0x01))
        assert mfrc.write_page(7, b"\x00" * 4) is True
        assert mfrc.last_write_ack == (True, "0x1", 1, 4, "0xa")

    @pytest.mark.parametrize("error,want_err", [(0x10, "0x10"), (0x02, "0x2")])
    def test_a_real_buffer_error_still_fails(self, error, want_err):
        mfrc = Mfrc522(RfidScriptedLink(RFID_ACK, error=error))
        assert mfrc.write_page(7, b"\x00" * 4) is False
        assert mfrc.last_write_ack == (True, want_err, 1, 4, "0xa")

    @pytest.mark.parametrize("bad", [b"", b"\x01\x02\x03", b"\x01\x02\x03\x04\x05"])
    def test_a_page_must_be_exactly_four_bytes(self, bad):
        link = RfidScriptedLink(RFID_ACK)
        mfrc = Mfrc522(link)
        with pytest.raises(ValueError) as err:
            mfrc.write_page(7, bad)
        assert str(err.value) == f"NTAG page write needs 4 bytes, got {len(bad)}"
        assert link.frames == []
        assert not hasattr(mfrc, "last_write_ack")


class TestMfrc522Halt:
    def test_halt_sends_hlta_frame(self):
        link = RfidScriptedLink(None)
        assert Mfrc522(link).halt() is None
        assert link.frames == [b"\x50\x00\x57\xcd"]

    def test_halt_puts_the_selected_tag_to_sleep(self):
        tag = RfidSimTag(RFID_UID4)
        link = RfidSimLink(tag)
        link.power_and_select(tag)
        Mfrc522(link).halt()
        assert tag.state == "halt"
        assert link.trace == ["HLTA"]


class TestMifareClassicActivate:
    @staticmethod
    def field(*uids: bytes, ntag: bool = False) -> Tuple[RfidSimLink, List[RfidSimTag]]:
        """
        A reader with one tag per UID, in anticollision order.

        :param uids: tag UIDs
        :param ntag: True for NTAG tags
        :return Tuple[RfidSimLink, List[RfidSimTag]]: the link and its tags
        """
        tags = [RfidSimTag(uid, ntag=ntag) for uid in uids]
        return RfidSimLink(*tags), tags

    @staticmethod
    def excludes(uid_hex: str) -> Callable[[str], bool]:
        """
        A predicate excluding one UID.

        :param uid_hex: the UID to exclude
        :return Callable[[str], bool]: the predicate
        """
        return lambda h: h == uid_hex

    def test_an_ntag_reports_its_whole_uid(self):
        link, (tag,) = self.field(RFID_UID7, ntag=True)
        uid, sak = MifareClassic(Mfrc522(link)).activate()
        assert uid == b"\x04\xa1\xb2\xc3\xd4\xe5\xf6"
        assert uid.hex() == "04a1b2c3d4e5f6"
        assert sak == 0x00
        assert tag.state == "active"

    def test_it_runs_both_cascade_levels(self):
        link, _ = self.field(RFID_UID7, ntag=True)
        MifareClassic(Mfrc522(link)).activate()
        assert link.trace == ["SOFT-RESET", "FIELD-ON", "WUPA", "ANTICOLL1",
                              "SELECT1 8804a1b2", "ANTICOLL2", "SELECT2 c3d4e5f6"]

    def test_the_cascade_tag_is_not_part_of_the_uid(self):
        link, _ = self.field(bytes.fromhex("04112233445566"), ntag=True)
        uid, _ = MifareClassic(Mfrc522(link)).activate()
        assert uid == bytes.fromhex("04112233445566")
        assert uid[0] != 0x88

    def test_a_four_byte_uid_still_takes_one_level(self):
        link, _ = self.field(b"\x01\x02\x03\x04")
        uid, sak = MifareClassic(Mfrc522(link)).activate()
        assert uid == b"\x01\x02\x03\x04"
        assert sak == 0x08
        assert link.trace == ["SOFT-RESET", "FIELD-ON", "WUPA", "ANTICOLL1",
                              "SELECT1 01020304"]

    def test_two_stickers_from_one_roll_are_told_apart(self):
        link_a, _ = self.field(bytes.fromhex("04112233445566"), ntag=True)
        link_b, _ = self.field(bytes.fromhex("04112233445577"), ntag=True)
        uid_a, _ = MifareClassic(Mfrc522(link_a)).activate()
        uid_b, _ = MifareClassic(Mfrc522(link_b)).activate()
        assert (uid_a.hex(), uid_b.hex()) == ("04112233445566", "04112233445577")

    def test_activate_halts_excluded_neighbour_and_returns_own(self):
        link, (neigh, own) = self.field(b"\xaa" * 4, b"\xbb" * 4)
        uid, sak = MifareClassic(Mfrc522(link)).activate(
            is_excluded=self.excludes("aaaaaaaa"))
        assert (uid, sak) == (b"\xbb" * 4, 0x08)
        assert (neigh.state, own.state) == ("halt", "active")
        assert link.trace == ["SOFT-RESET", "FIELD-ON", "WUPA", "ANTICOLL1",
                              "SELECT1 aaaaaaaa", "HLTA", "REQA", "ANTICOLL1",
                              "SELECT1 bbbbbbbb"]

    def test_activate_returns_none_when_only_excluded_present(self):
        link, (neigh,) = self.field(b"\xaa" * 4)
        uid, sak = MifareClassic(Mfrc522(link)).activate(
            is_excluded=self.excludes("aaaaaaaa"))
        assert (uid, sak) == (None, None)
        assert neigh.state == "halt"
        assert link.trace[-2:] == ["HLTA", "REQA"]

    def test_activate_without_excluder_returns_first_tag(self):
        link, (neigh, own) = self.field(b"\xaa" * 4, b"\xbb" * 4)
        uid, sak = MifareClassic(Mfrc522(link)).activate()
        assert (uid, sak) == (b"\xaa" * 4, 0x08)
        assert (neigh.state, own.state) == ("active", "idle")
        assert link.trace == ["SOFT-RESET", "FIELD-ON", "WUPA", "ANTICOLL1",
                              "SELECT1 aaaaaaaa"]

    def test_activate_seen_records_halted_neighbour_and_own(self):
        link, _ = self.field(b"\xaa" * 4, b"\xbb" * 4)
        seen = []
        # A truthy non-bool predicate: seen still records a real bool.
        MifareClassic(Mfrc522(link)).activate(
            is_excluded=lambda h: h if h == "aaaaaaaa" else "", seen=seen)
        assert seen == [("aaaaaaaa", 0x08, True), ("bbbbbbbb", 0x08, False)]

    def test_activate_seen_empty_when_field_empty(self):
        link, _ = self.field()
        seen = []
        uid, sak = MifareClassic(Mfrc522(link)).activate(is_excluded=lambda h: False,
                                                         seen=seen)
        assert (uid, sak) == (None, None)
        assert seen == []
        assert link.trace == ["SOFT-RESET", "FIELD-ON", "WUPA", "REQA"]

    def test_activate_seen_without_excluder_records_the_tag(self):
        link, _ = self.field(b"\xbb" * 4)
        seen = []
        MifareClassic(Mfrc522(link)).activate(seen=seen)
        assert seen == [("bbbbbbbb", 0x08, False)]

    def test_activate_seen_without_excluder_stays_empty_on_an_empty_field(self):
        link, _ = self.field()
        seen = []
        assert MifareClassic(Mfrc522(link)).activate(seen=seen) == (None, None)
        assert seen == []

    def test_activate_gives_up_after_four_excluded_tags(self):
        uids = [bytes([n]) * 4 for n in (1, 2, 3, 4, 5)]
        link, tags = self.field(*uids)
        seen = []
        uid, sak = MifareClassic(Mfrc522(link)).activate(is_excluded=lambda h: True,
                                                         seen=seen)
        assert (uid, sak) == (None, None)
        assert [t.state for t in tags] == ["halt", "halt", "halt", "halt", "idle"]
        assert seen == [("01010101", 8, True), ("02020202", 8, True),
                        ("03030303", 8, True), ("04040404", 8, True)]

    def test_activate_reset_false_keeps_field_no_soft_reset(self):
        link, (own,) = self.field(b"\xbb" * 4)
        mc = MifareClassic(Mfrc522(link))
        mc.activate(reset=True)
        assert link.trace[:2] == ["SOFT-RESET", "FIELD-ON"]
        own.crypto_sector = 1
        link.regs[RFID_REG_STATUS2] = 0x08
        link.trace.clear()
        assert mc.activate(reset=False) == (b"\xbb" * 4, 0x08)
        assert link.trace == ["CRYPTO-OFF", "WUPA", "ANTICOLL1", "SELECT1 bbbbbbbb"]
        assert link.regs[RFID_REG_STATUS2] == 0x00


class TestMifareClassicActivateOnce:
    @staticmethod
    def powered(tag: RfidSimTag) -> Tuple[RfidSimLink, MifareClassic]:
        """
        A field-on reader holding one idle tag.

        :param tag: the tag
        :return Tuple[RfidSimLink, MifareClassic]: the link and the reader
        """
        link = RfidSimLink(tag)
        link.power_on()
        return link, MifareClassic(Mfrc522(link))

    def test_wake_success(self):
        tag = RfidSimTag(b"\x01\x02\x03\x04")
        link, mc = self.powered(tag)
        assert mc._activate_once(wake=True) == (b"\x01\x02\x03\x04", 0x08)
        assert link.trace == ["WUPA", "ANTICOLL1", "SELECT1 01020304"]
        assert tag.state == "active"

    def test_wake_falls_back_to_reqa(self):
        tag = RfidSimTag(b"\x01\x02\x03\x04", ignores=["wupa"])
        link, mc = self.powered(tag)
        assert mc._activate_once(wake=True) == (b"\x01\x02\x03\x04", 0x08)
        assert link.trace == ["WUPA", "REQA", "ANTICOLL1", "SELECT1 01020304"]

    def test_wake_both_none_returns_none(self):
        link = RfidSimLink()
        link.power_on()
        assert MifareClassic(Mfrc522(link))._activate_once(wake=True) == (None, None)
        assert link.trace == ["WUPA", "REQA"]

    def test_no_wake_reqa_none_returns_none(self):
        tag = RfidSimTag(b"\x01\x02\x03\x04")
        link, mc = self.powered(tag)
        tag.state = "halt"
        assert mc._activate_once(wake=False) == (None, None)
        assert link.trace == ["REQA"]
        assert tag.state == "halt"

    def test_anticoll_none_returns_none(self):
        link, mc = self.powered(RfidSimTag(b"\x01\x02\x03\x04", ignores=["anticoll1"]))
        assert mc._activate_once(wake=False) == (None, None)
        assert link.trace == ["REQA", "ANTICOLL1"]

    def test_select_none_returns_none(self):
        tag = RfidSimTag(b"\x01\x02\x03\x04", ignores=["select1"])
        link, mc = self.powered(tag)
        assert mc._activate_once(wake=False) == (None, None)
        assert link.trace == ["REQA", "ANTICOLL1", "SELECT1 01020304"]
        assert tag.state == "idle"

    def test_seven_byte_uid_takes_the_second_level(self):
        tag = RfidSimTag(RFID_UID7, ntag=True)
        link, mc = self.powered(tag)
        assert mc._activate_once(wake=False) == (RFID_UID7, 0x00)
        assert link.trace == ["REQA", "ANTICOLL1", "SELECT1 8804a1b2", "ANTICOLL2",
                              "SELECT2 c3d4e5f6"]

    def test_incomplete_sak_without_the_cascade_tag_returns_none(self):
        link, mc = self.powered(RfidSimTag(b"\x01\x02\x03\x04", sak=0x0C))
        assert mc._activate_once(wake=False) == (None, None)
        assert link.trace == ["REQA", "ANTICOLL1", "SELECT1 01020304"]

    def test_second_level_anticoll_none_returns_none(self):
        link, mc = self.powered(RfidSimTag(RFID_UID7, ntag=True, ignores=["anticoll2"]))
        assert mc._activate_once(wake=False) == (None, None)
        assert link.trace[-1] == "ANTICOLL2"

    def test_second_level_select_none_returns_none(self):
        tag = RfidSimTag(RFID_UID7, ntag=True, ignores=["select2"])
        link, mc = self.powered(tag)
        assert mc._activate_once(wake=False) == (None, None)
        assert link.trace[-1] == "SELECT2 c3d4e5f6"
        assert tag.state == "idle"


class TestMifareClassicReadNtag:
    IMAGE = bytes(n & 0xFF for n in range(180))

    def reader(self, **tag_kw: object) -> Tuple[RfidSimLink, MifareClassic]:
        """
        A reader with an NTAG213 holding IMAGE already selected.

        :param tag_kw: extra RfidSimTag options
        :return Tuple[RfidSimLink, MifareClassic]: the link and the reader
        """
        tag = RfidSimTag(RFID_UID7, self.IMAGE, ntag=True, **tag_kw)
        link = RfidSimLink(tag)
        link.power_and_select(tag)
        return link, MifareClassic(Mfrc522(link))

    def test_reads_until_nbytes(self):
        link, mc = self.reader()
        assert mc.read_ntag(128) == self.IMAGE[:128]
        assert link.trace == ["READ 0", "READ 4", "READ 8", "READ 12", "READ 16",
                              "READ 20", "READ 24", "READ 28"]

    def test_a_partial_read_from_a_start_page_is_trimmed(self):
        link, mc = self.reader()
        assert mc.read_ntag(20, start_page=8) == self.IMAGE[32:52]
        assert link.trace == ["READ 8", "READ 12"]

    def test_missing_page_returns_none(self):
        link, mc = self.reader(unreadable=[8])
        assert mc.read_ntag(128) is None
        assert link.trace == ["READ 0", "READ 4", "READ 8"]


class TestMifareClassicUserLastPage:
    @staticmethod
    def reader(cc: bytes, **tag_kw: object) -> Tuple[RfidSimLink, MifareClassic]:
        """
        A reader with an NTAG whose page 3 is ``cc`` already selected.

        :param cc: capability container bytes
        :param tag_kw: extra RfidSimTag options
        :return Tuple[RfidSimLink, MifareClassic]: the link and the reader
        """
        tag = RfidSimTag(RFID_UID7, bytes(12) + cc, ntag=True, pages=240, **tag_kw)
        link = RfidSimLink(tag)
        link.power_and_select(tag)
        return link, MifareClassic(Mfrc522(link))

    @pytest.mark.parametrize("cc,last", [
        (b"\xe1\x10\x12\x00", 39),      # NTAG213
        (b"\xe1\x10\x3e\x00", 127),     # NTAG215
        (b"\xe1\x10\x6d\x00", 221),     # NTAG216
    ])
    def test_it_reads_the_bound_off_the_tag(self, cc, last):
        link, mc = self.reader(cc)
        assert mc.user_last_page() == last
        assert link.trace == ["READ 3"]

    def test_the_bound_never_exceeds_the_real_user_area(self):
        # An NTAG215's user pages run 4-129; its CC says 496 bytes, page 127.
        _, mc = self.reader(b"\xe1\x10\x3e\x00")
        assert mc.user_last_page() == 127

    @pytest.mark.parametrize("cc", [b"\x00\x00\x00\x00", b"\x00\x10\x3e\x00"])
    def test_a_tag_with_no_readable_cc_falls_back_to_the_smallest_chip(self, cc):
        _, mc = self.reader(cc)
        assert mc.user_last_page() == 39

    def test_an_unreadable_page_3_falls_back_too(self):
        link, mc = self.reader(b"\xe1\x10\x3e\x00", unreadable=[3])
        assert mc.user_last_page() == 39
        assert link.trace == ["READ 3"]

    def test_a_zero_size_cc_falls_back_too(self):
        _, mc = self.reader(b"\xe1\x10\x00\x00")
        assert mc.user_last_page() == 39


class TestMifareClassicWriteNtag:
    @staticmethod
    def reader(cc: bytes = b"\xe1\x10\x3e\x00", pages: int = 135,
               **tag_kw: object) -> Tuple[RfidSimTag, RfidSimLink, MifareClassic]:
        """
        A reader with a blank NTAG already selected.

        :param cc: capability container bytes for page 3
        :param pages: page count
        :param tag_kw: extra RfidSimTag options
        :return Tuple[RfidSimTag, RfidSimLink, MifareClassic]: tag, link, reader
        """
        tag = RfidSimTag(RFID_UID7, bytes(12) + cc, ntag=True, pages=pages, **tag_kw)
        link = RfidSimLink(tag)
        link.power_and_select(tag)
        return tag, link, MifareClassic(Mfrc522(link))

    @staticmethod
    def writes(link: RfidSimLink) -> List[str]:
        """
        The page writes in a trace.

        :param link: the reader link
        :return List[str]: its WRITE events
        """
        return [event for event in link.trace if event.startswith("WRITE")]

    def test_a_clean_write_reports_no_error(self):
        tag, link, mc = self.reader()
        assert mc.write_ntag(4, b"\xaa" * 112) is None
        assert self.writes(link) == [f"WRITE {page}" for page in range(4, 32)]
        assert tag.memory[16:128] == b"\xaa" * 112
        assert link.trace[0] == "READ 3"
        assert link.trace[29:] == ["READ 4", "READ 8", "READ 12", "READ 16", "READ 20",
                                   "READ 24", "READ 28"]

    def test_the_bytes_actually_land_where_the_decoder_looks(self):
        tag, _, mc = self.reader()
        payload = encode_anycubic(manufacturer="Sunlu", ftype="PETG",
                                  color_argb=0xFF00FF00, weight_g=1000)
        assert mc.write_ntag(4, payload) is None
        assert tag.memory[16:128] == payload
        decoded = decode_anycubic(bytes(tag.memory[:128]))
        assert (decoded["manufacturer"], decoded["type"], decoded["color_argb"]) == (
            "Sunlu", "PETG", 0xFF00FF00)

    @pytest.mark.parametrize("page", [0, 1, 2, 3])
    def test_the_reserved_pages_are_refused(self, page):
        tag, link, mc = self.reader()
        before = bytes(tag.memory)
        assert mc.write_ntag(page, b"\x00" * 4) == (
            f"page {page} is in the reserved area; the first writable page is 4")
        assert link.trace == ["READ 3"]
        assert bytes(tag.memory) == before

    def test_running_past_the_user_area_is_refused(self):
        _, link, mc = self.reader(cc=b"\xe1\x10\x12\x00", pages=45)
        assert mc.write_ntag(30, b"\x00" * 112) == (
            "28 pages from 30 runs past the last writable page (39)")
        assert link.trace == ["READ 3"]

    def test_the_last_legal_page_is_allowed(self):
        tag, link, mc = self.reader(cc=b"\xe1\x10\x12\x00", pages=48)
        assert mc.write_ntag(39, b"\x01\x02\x03\x04") is None
        assert link.trace == ["READ 3", "WRITE 39", "READ 39"]
        assert tag.memory[156:160] == b"\x01\x02\x03\x04"

    def test_one_page_past_the_last_is_refused(self):
        _, link, mc = self.reader(cc=b"\xe1\x10\x12\x00", pages=48)
        assert mc.write_ntag(40, b"\x01\x02\x03\x04") == (
            "1 pages from 40 runs past the last writable page (39)")
        assert link.trace == ["READ 3"]

    def test_a_payload_that_is_not_whole_pages_is_refused(self):
        _, link, mc = self.reader()
        assert mc.write_ntag(4, b"\x00" * 5) == "payload is 5 bytes, not a multiple of 4"
        assert link.trace == []

    def test_a_tag_that_will_not_take_a_page_is_reported(self):
        tag, link, mc = self.reader(nak_writes=[6])
        assert mc.write_ntag(4, b"\x11" * 112) == (
            "tag did not ACK the write to page 6 [resp (True, '0x1', 1, 4, '0x0')]")
        assert self.writes(link) == ["WRITE 4", "WRITE 5", "WRITE 6", "WRITE 6", "WRITE 6"]
        assert tag.memory[24:28] == b"\x00" * 4

    def test_a_missed_ack_is_retried(self):
        tag, link, mc = self.reader(lost_acks={5: 2})
        assert mc.write_ntag(4, b"\x11" * 8) is None
        assert self.writes(link) == ["WRITE 4", "WRITE 5", "WRITE 5", "WRITE 5"]
        assert tag.memory[16:24] == b"\x11" * 8

    def test_a_page_that_reads_back_wrong_fails_the_write(self):
        tag, _, mc = self.reader(corrupt={10: b"\x00" * 4})
        # The READ of page 8 covers 8-11; the error names page 10, the bad one.
        assert mc.write_ntag(4, b"\x55" * 112) == (
            "page 10 reads back as 00000000, not 55555555")
        assert tag.memory[40:44] == b"\x00" * 4

    def test_the_first_bad_page_of_a_read_is_named(self):
        _, _, mc = self.reader(corrupt={9: b"\x01" * 4, 11: b"\x02" * 4})
        assert mc.write_ntag(4, b"\x55" * 112) == (
            "page 9 reads back as 01010101, not 55555555")

    def test_a_page_that_cannot_be_read_back_fails_the_write(self):
        _, link, mc = self.reader(unreadable=[12])
        assert mc.write_ntag(4, b"\x22" * 112) == "could not read page 12 back"
        assert link.trace[-3:] == ["READ 4", "READ 8", "READ 12"]


class TestMifareClassicReadAll:
    IMAGE = b"".join(bytes([n]) * 16 for n in range(64))
    UID = b"\x01\x02\x03\x04"

    def reader(self, **tag_kw: object) -> Tuple[RfidSimTag, RfidSimLink, MifareClassic]:
        """
        A reader with a Classic tag holding IMAGE (block n is n repeated) selected.

        :param tag_kw: extra RfidSimTag options
        :return Tuple[RfidSimTag, RfidSimLink, MifareClassic]: tag, link, reader
        """
        tag = RfidSimTag(self.UID, self.IMAGE, **tag_kw)
        link = RfidSimLink(tag)
        link.power_and_select(tag)
        return tag, link, MifareClassic(Mfrc522(link))

    def test_auth_failure_returns_none(self):
        tag, link, mc = self.reader()
        assert mc.read_all(self.UID, [[0] * 6] * 16, sectors=1) is None
        assert link.trace == ["AUTH 0"]
        assert tag.state == "idle"

    def test_read_block_failure_returns_none(self):
        _, link, mc = self.reader(unreadable=[1])
        assert mc.read_all(self.UID, [[0xFF] * 6] * 16, sectors=1) is None
        assert link.trace == ["AUTH 0", "READ 0", "READ 1"]

    def test_success_returns_image(self):
        _, link, mc = self.reader()
        assert mc.read_all(self.UID, [[0xFF] * 6] * 16, sectors=1) == self.IMAGE[:64]
        assert link.trace == ["AUTH 0", "READ 0", "READ 1", "READ 2", "READ 3"]

    def test_the_sector_count_is_clamped(self):
        _, link, mc = self.reader()
        assert mc.read_all(self.UID, [[0xFF] * 6] * 16, sectors=0) == self.IMAGE[:64]
        assert mc.read_all(self.UID, [[0xFF] * 6] * 16, sectors=40) == self.IMAGE
        auths = [event for event in link.trace if event.startswith("AUTH")]
        assert auths == ["AUTH 0"] + [f"AUTH {sector * 4}" for sector in range(16)]


class TestMifareClassicReadBlocks:
    IMAGE = b"".join(bytes([n]) * 16 for n in range(64))
    UID = b"\x01\x02\x03\x04"

    def reader(self, **tag_kw: object) -> Tuple[RfidSimLink, MifareClassic]:
        """
        A reader with a Classic tag holding IMAGE (block n is n repeated) selected.

        :param tag_kw: extra RfidSimTag options
        :return Tuple[RfidSimLink, MifareClassic]: the link and the reader
        """
        tag = RfidSimTag(self.UID, self.IMAGE, **tag_kw)
        link = RfidSimLink(tag)
        link.power_and_select(tag)
        return link, MifareClassic(Mfrc522(link))

    def test_auth_failure_returns_none(self):
        link, mc = self.reader()
        assert mc.read_blocks(self.UID, [[0] * 6] * 16, (4,)) is None
        assert link.trace == ["AUTH 4"]

    def test_read_block_failure_returns_none(self):
        link, mc = self.reader(unreadable=[4])
        assert mc.read_blocks(self.UID, [[0xFF] * 6] * 16, (4,)) is None
        assert link.trace == ["AUTH 4", "READ 4"]

    def test_success_places_blocks(self):
        link, mc = self.reader()
        out = mc.read_blocks(self.UID, [[0xFF] * 6] * 16, (5,))
        assert out == bytes(80) + b"\x05" * 16 + bytes(1024 - 96)
        assert link.trace == ["AUTH 4", "READ 5"]

    def test_each_sector_is_authenticated_once_in_order(self):
        link, mc = self.reader()
        out = mc.read_blocks(self.UID, [[0xFF] * 6] * 16, (9, 5, 4))
        want = bytearray(1024)
        want[64:96] = b"\x04" * 16 + b"\x05" * 16
        want[144:160] = b"\x09" * 16
        assert out == bytes(want)
        assert link.trace == ["AUTH 4", "READ 4", "READ 5", "AUTH 8", "READ 9"]


class TestHkdfSha256:
    def test_hkdf_rfc5869_vector(self):
        got = hkdf_sha256(bytes.fromhex("000102030405060708090a0b0c"),
                          bytes.fromhex("0b" * 22),
                          bytes.fromhex("f0f1f2f3f4f5f6f7f8f9"), 42)
        assert got.hex() == ("3cb25f25faacd57a90434f64d0362f2a2d2d0a90cf1a5a4c5"
                             "db02d56ecc4c5bf34007208d5b887185865")


class TestDecodeBambu:
    @pytest.mark.parametrize("size", [100, 1023, 1025])
    def test_wrong_length_raises(self, size):
        with pytest.raises(ValueError) as err:
            decode_bambu(bytes(size))
        assert str(err.value) == "expected 1024-byte MIFARE Classic 1K image"

    def test_optional_fields_present(self):
        image = rfid_bambu_image(nozzle=0.4, spool_width=6000, length=250)
        assert decode_bambu(image) == rfid_bambu_fields(spool_width_mm=60.0, length_m=250)

    def test_optional_fields_absent(self):
        image = rfid_bambu_image(nozzle=5.0, spool_width=0, length=0)
        assert decode_bambu(image) == rfid_bambu_fields(
            nozzle_diameter=None, spool_width_mm=None, length_m=None)

    def test_nozzle_spool_width_length_parsed(self):
        f = decode_bambu(rfid_bambu_image())
        assert (f["nozzle_diameter"], f["spool_width_mm"], f["length_m"]) == (0.4, 66.2, 330)

    def test_nozzle_zero_is_none(self):
        assert decode_bambu(rfid_bambu_image(nozzle=0.0))["nozzle_diameter"] is None

    @pytest.mark.parametrize("nozzle", [87.0, 2.0])
    def test_nozzle_out_of_range_is_none(self, nozzle):
        assert decode_bambu(rfid_bambu_image(nozzle=nozzle))["nozzle_diameter"] is None

    def test_spool_width_zero_is_none(self):
        assert decode_bambu(rfid_bambu_image(spool_width=0))["spool_width_mm"] is None

    def test_length_zero_is_none(self):
        assert decode_bambu(rfid_bambu_image(length=0))["length_m"] is None


class TestDecodeAnycubic:
    def test_short_returns_none(self):
        assert decode_anycubic(rfid_anycubic_image()[:0x7B]) is None
        assert decode_anycubic(rfid_anycubic_image()[:0x7C]) == rfid_anycubic_fields()

    def test_wrong_magic_returns_none(self):
        assert decode_anycubic(b"\x11" * 0x80) is None

    def test_length_kept(self):
        assert decode_anycubic(rfid_anycubic_image(length=330)) == rfid_anycubic_fields()

    def test_length_zero_is_none(self):
        # The weight falls back to a 1 kg spool when no length is set.
        f = decode_anycubic(rfid_anycubic_image(length=0))
        assert (f["length_m"], f["weight_g"]) == (None, 1000)

    def test_an_unlisted_length_keeps_the_length_and_defaults_the_weight(self):
        f = decode_anycubic(rfid_anycubic_image(length=165))
        assert (f["length_m"], f["weight_g"]) == (165, 500)
        f = decode_anycubic(rfid_anycubic_image(length=412))
        assert (f["length_m"], f["weight_g"]) == (412, 1000)

    def test_a_blank_brand_reads_as_anycubic(self):
        image = bytearray(rfid_anycubic_image())
        image[0x28:0x30] = bytes(8)
        assert decode_anycubic(bytes(image))["manufacturer"] == "Anycubic"


class TestEncodeAnycubic:
    def test_it_round_trips_through_the_decoder(self):
        p = encode_anycubic(manufacturer="Polymaker", sku="PM-PLA-BLK",
                            ftype="PLA", color_argb=0xFF1A2B3C,
                            diameter_mm=1.75, weight_g=1000,
                            hotend_min_c=190, hotend_max_c=230, bed_temp_c=60)
        assert decode_anycubic(bytes(16) + p) == {
            "manufacturer": "Polymaker", "sku": "PM-PLA-BLK", "type": "PLA",
            "color_argb": 0xFF1A2B3C, "diameter_mm": 1.75, "weight_g": 1000,
            "length_m": 330, "hotend_min_c": 190, "hotend_max_c": 230, "bed_temp_c": 60}

    def test_the_numbers_sit_at_their_offsets(self):
        p = encode_anycubic(diameter_mm=2.85, hotend_min_c=200, hotend_max_c=0x1234,
                            bed_temp_c=-5, length_m=70000)
        # Image offsets 0x60, 0x62, 0x76, 0x78, 0x7A, less the 0x10 before page 4.
        assert p[0x50:0x54] == b"\xc8\x00\x34\x12"
        assert p[0x66:0x6C] == b"\x00\x00\x1d\x01\xff\xff"

    def test_the_diameter_is_rounded_not_truncated(self):
        # 1.15 * 100 is 114.99999999999999 in floating point.
        p = encode_anycubic(diameter_mm=1.15)
        assert p[0x68:0x6A] == b"\x73\x00"

    def test_it_fills_exactly_pages_4_to_31(self):
        assert len(encode_anycubic(ftype="PLA")) == 112

    def test_the_magic_lands_on_the_first_writable_page(self):
        assert encode_anycubic(ftype="PLA")[:4] == b"\x7b\x00\x65\x00"

    def test_the_brand_is_ours_to_set(self):
        """The magic is a format fingerprint, not a claim about the maker."""
        p = encode_anycubic(manufacturer="Overture")
        assert decode_anycubic(bytes(16) + p)["manufacturer"] == "Overture"

    @pytest.mark.parametrize("grams,metres", [(1000, 330), (750, 247), (600, 198),
                                              (500, 165), (250, 82)])
    def test_every_encodable_weight_round_trips(self, grams, metres):
        p = encode_anycubic(weight_g=grams)
        assert p[0x6A:0x6C] == metres.to_bytes(2, "little")
        d = decode_anycubic(bytes(16) + p)
        assert (d["length_m"], d["weight_g"]) == (metres, grams)

    def test_an_unencodable_weight_leaves_the_length_blank(self):
        p = encode_anycubic(weight_g=800)
        assert p[0x6A:0x6C] == b"\x00\x00"
        assert decode_anycubic(bytes(16) + p)["length_m"] is None

    def test_a_length_can_be_set_directly(self):
        assert decode_anycubic(bytes(16) + encode_anycubic(length_m=412))["length_m"] == 412
        p = encode_anycubic(weight_g=1000, length_m=412)
        assert decode_anycubic(bytes(16) + p)["length_m"] == 412

    def test_no_weight_and_no_length_is_left_blank(self):
        p = encode_anycubic(ftype="PLA")
        assert decode_anycubic(bytes(16) + p)["length_m"] is None

    def test_the_colour_byte_order_is_abgr_on_the_tag(self):
        """decode_anycubic reads a, b, g, r in that order."""
        p = encode_anycubic(color_argb=0x11804020)
        assert p[0x40:0x44] == b"\x11\x20\x40\x80"

    def test_a_long_string_is_truncated_not_overflowed(self):
        p = encode_anycubic(manufacturer="A" * 40, sku="B" * 40, ftype="C" * 40)
        assert len(p) == 112
        assert p[0x04:0x18] == b"B" * 16 + bytes(4)
        assert p[0x18:0x2C] == b"A" * 16 + bytes(4)
        assert p[0x2C:0x40] == b"C" * 16 + bytes(4)
        d = decode_anycubic(bytes(16) + p)
        assert (d["manufacturer"], d["sku"], d["type"]) == ("A" * 16, "B" * 16, "C" * 16)

    def test_non_ascii_does_not_raise(self):
        p = encode_anycubic(manufacturer="Prusa\u00e9")
        assert decode_anycubic(bytes(16) + p)["manufacturer"] == "Prusa?"


class TestDecodeAfcBlock:
    def test_a_genuine_anycubic_tag_has_no_block(self):
        image = rfid_anycubic_image() + bytes(32)
        assert decode_afc_block(image) is None
        assert decode_anycubic(image)["weight_g"] == 1000

    def test_a_short_read_is_not_mistaken_for_a_block(self):
        assert decode_afc_block(rfid_anycubic_image()) is None
        assert decode_afc_block(rfid_anycubic_image() + rfid_afc_block(weight=500)[:31]) is None

    def test_a_newer_version_is_ignored_rather_than_misread(self):
        image = rfid_anycubic_image()
        assert decode_afc_block(image + rfid_afc_block(version=2, weight=500)) is None
        assert decode_afc_block(image + rfid_afc_block(version=1, weight=500)) == {
            "weight_g": 500}

    def test_unset_fields_do_not_appear(self):
        """So an overlay never blanks what the brand layout did know."""
        image = rfid_anycubic_image() + rfid_afc_block(weight=500)
        assert decode_afc_block(image) == {"weight_g": 500}

    def test_every_set_field_is_decoded(self):
        block = rfid_afc_block(weight=823, spool=136, density=1240, dry_temp=55, dry_hours=8)
        assert decode_afc_block(rfid_anycubic_image() + block) == {
            "weight_g": 823, "spool_id": 136, "density": 1.24,
            "drying_temp_c": 55, "drying_time_h": 8}

    def test_offset_zero_reads_a_bare_block(self):
        assert decode_afc_block(rfid_afc_block(spool=9), offset=0) == {"spool_id": 9}


class TestEncodeTagPayload:
    def test_an_arbitrary_weight_round_trips_exactly(self):
        """Spoolman's 823g is 823g on the tag, not 1000."""
        payload = encode_tag_payload(ftype="PLA", weight_g=823)
        assert payload[112:] == rfid_afc_block(weight=823)
        assert decode_afc_block(bytes(16) + payload) == {"weight_g": 823}

    def test_it_carries_the_spoolman_spool_id(self):
        payload = encode_tag_payload(ftype="PLA", spool_id=136)
        assert payload[112:] == rfid_afc_block(spool=136)
        assert decode_afc_block(bytes(16) + payload) == {"spool_id": 136}

    def test_a_big_spool_id_survives(self):
        payload = encode_tag_payload(ftype="PLA", spool_id=4000000000)
        assert payload[120:124] == b"\x00\x28\x6b\xee"
        assert decode_afc_block(bytes(16) + payload) == {"spool_id": 4000000000}

    def test_it_carries_density_and_drying(self):
        payload = encode_tag_payload(ftype="PLA", density=1.24,
                                     drying_temp_c=55, drying_time_h=8)
        assert payload[112:] == rfid_afc_block(density=1240, dry_temp=55, dry_hours=8)
        assert decode_afc_block(bytes(16) + payload) == {
            "density": 1.24, "drying_temp_c": 55, "drying_time_h": 8}

    def test_it_sits_where_the_anycubic_decoder_never_looks(self):
        # 823g has no Anycubic length, so only the AFC block carries it.
        payload = encode_tag_payload(manufacturer="Sunlu", ftype="PETG",
                                     weight_g=823, spool_id=7)
        anycubic = bytearray(112)
        anycubic[0x00:0x04] = b"\x7b\x00\x65\x00"
        anycubic[0x18:0x1D] = b"Sunlu"
        anycubic[0x2C:0x30] = b"PETG"
        anycubic[0x40:0x44] = b"\xff\xff\xff\xff"          # default white, A B G R
        anycubic[0x68:0x6A] = b"\xaf\x00"                  # 175, 1.75 mm
        assert payload[:112] == bytes(anycubic)
        assert payload[112:] == rfid_afc_block(weight=823, spool=7)
        assert payload[118:120] == b"\x37\x03"
        assert decode_anycubic(bytes(16) + payload) == {
            "manufacturer": "Sunlu", "sku": "", "type": "PETG", "color_argb": 0xFFFFFFFF,
            "diameter_mm": 1.75, "weight_g": 1000, "length_m": None, "hotend_min_c": 0,
            "hotend_max_c": 0, "bed_temp_c": 0}

    def test_a_spool_id_past_32_bits_is_clamped(self):
        payload = encode_tag_payload(ftype="PLA", spool_id=2 ** 32 + 5)
        assert payload[120:124] == b"\xff\xff\xff\xff"

    def test_the_density_is_rounded_not_truncated(self):
        # 1.001 * 1000 is 1000.9999999999999 in floating point.
        payload = encode_tag_payload(ftype="PLA", density=1.001)
        assert payload[124:126] == b"\xe9\x03"

    def test_the_whole_record_fits_an_ntag213(self):
        # 144 bytes is 36 pages: page 4 through page 39, the NTAG213's last.
        assert len(encode_tag_payload(ftype="PLA", weight_g=823, spool_id=1)) == 144

    def test_the_magic_is_there(self):
        # Payload byte 112 is image byte 0x80, page 32.
        assert encode_tag_payload(ftype="PLA")[112:117] == b"AFC1\x01"


class TestSnapmakerKeys:
    def test_snapmaker_key_derivation_matches_spec(self):
        uid = bytes.fromhex("80dcf43e")
        keys = snapmaker_keys(uid)
        assert [bytes(key) for key in keys] == rfid_snapmaker_keys(uid)
        assert len(keys) == 16


class TestDecodeSnapmaker:
    def test_short_returns_none(self):
        assert decode_snapmaker(rfid_snapmaker_image()[:175]) is None
        assert decode_snapmaker(rfid_snapmaker_image()[:176]) == RFID_SNAPMAKER_FIELDS

    def test_decode_snapmaker_fields(self):
        assert decode_snapmaker(rfid_snapmaker_image()) == RFID_SNAPMAKER_FIELDS

    @pytest.mark.parametrize("main_type", [0, 6])
    def test_decode_snapmaker_rejects_non_snapmaker(self, main_type):
        assert decode_snapmaker(rfid_snapmaker_image(main_type)) is None

    def test_blank_optional_fields_take_defaults(self):
        image = bytearray(1024)
        image[66] = 2
        image[68] = 9
        assert decode_snapmaker(bytes(image)) == {
            "manufacturer": "Snapmaker", "type": "PETG", "detailed": "",
            "color_argb": 0xFF000000, "weight_g": 1000, "diameter_mm": 1.75,
            "hotend_max_c": None, "hotend_min_c": None, "bed_temp_c": None,
            "sku": "", "production": ""}


class TestAesEncryptBlock:
    def test_aes128_fips197_vector(self):
        key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
        pt = bytes.fromhex("00112233445566778899aabbccddeeff")
        ct = bytes.fromhex("69c4e0d86a7b0430d8cdb78070b4c55a")
        assert _aes_encrypt_block(pt, key) == ct
        assert _aes_decrypt_block(ct, key) == pt


class TestAesCbcDecrypt:
    def test_aes_cbc_decrypt_roundtrip(self):
        key = bytes.fromhex("484043466b526e7a404b4174424a7032")
        data = bytes(range(48))
        assert _aes_cbc_decrypt(rfid_cbc_encrypt(data, key), key) == data

    def test_nist_sp800_38a_cbc_vector(self):
        key = bytes.fromhex("2b7e151628aed2a6abf7158809cf4f3c")
        iv = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
        ct = bytes.fromhex("7649abac8119b246cee98e9b12e9197d"
                           "5086cb9b507219ee95db113a917678b2")
        pt = bytes.fromhex("6bc1bee22e409f96e93d7e117393172a"
                           "ae2d8a571e03ac9c9eb76fac45af8e51")
        assert _aes_cbc_decrypt(ct, key, iv) == pt

    def test_a_trailing_partial_block_is_dropped(self):
        key = bytes.fromhex("2b7e151628aed2a6abf7158809cf4f3c")
        iv = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
        ct = bytes.fromhex("7649abac8119b246cee98e9b12e9197d") + b"\x01\x02\x03"
        assert _aes_cbc_decrypt(ct, key, iv) == bytes.fromhex(
            "6bc1bee22e409f96e93d7e117393172a")


class TestCrealityMifareKey:
    def test_creality_key_derivation_matches_reference(self):
        u_key = bytes.fromhex("713362755e74316e71665a2870662431")
        key_a = creality_mifare_key(bytes.fromhex("60EA1221"), u_key)
        assert key_a.hex() == "1f1e83a97182"


class TestDecodeCreality:
    def test_short_returns_none(self):
        assert decode_creality(rfid_creality_payload()[:33]) is None
        assert decode_creality(rfid_creality_payload()[:34]) == rfid_creality_fields()

    def test_bad_hex_color_and_length_default(self):
        payload = rfid_creality_payload(color=b"ZZZZZZZ", length=b"YYYY", serial=b"SERIAL")
        assert decode_creality(payload) == rfid_creality_fields(
            color_argb=None, length_m=None, weight_g=None, serial="SERIAL")

    def test_length_kept(self):
        f = decode_creality(rfid_creality_payload(length=b"0165"))
        assert (f["length_m"], f["weight_g"]) == (357, 500)

    def test_length_zero_is_none(self):
        f = decode_creality(rfid_creality_payload(length=b"0000"))
        assert (f["length_m"], f["weight_g"]) == (None, None)

    def test_decode_creality_fields(self):
        assert decode_creality(rfid_creality_payload()) == rfid_creality_fields()

    def test_decode_creality_rejects_garbage(self):
        assert decode_creality(b"\x00" * 48) is None

    def test_the_vendor_code_alone_identifies_the_payload(self):
        payload = rfid_creality_payload(film=b"999999")
        assert decode_creality(payload) == rfid_creality_fields(type="", sku="999999")

    def test_a_padded_film_id_is_stripped_for_the_sku(self):
        payload = rfid_creality_payload(film=b" 12AB ")
        assert decode_creality(payload) == rfid_creality_fields(type="", sku="12AB")

    def test_a_blank_film_id_gives_an_empty_sku(self):
        payload = rfid_creality_payload(film=b"\x00" * 6)
        assert decode_creality(payload) == rfid_creality_fields(type="", sku="")

    def test_a_known_film_alone_identifies_the_payload(self):
        assert decode_creality(rfid_creality_payload(vendor=b"1234")) == rfid_creality_fields()

    def test_decode_creality_end_to_end_encrypted(self):
        d_key = bytes.fromhex("484043466b526e7a404b4174424a7032")
        payload = rfid_creality_payload(film=b"101002", color=b"000FF00", length=b"0330",
                                        serial=b"000123")
        f = decode_creality(_aes_cbc_decrypt(rfid_cbc_encrypt(payload, d_key), d_key))
        assert f == rfid_creality_fields(type="PETG", sku="101002", color_argb=0xFF00FF00,
                                         weight_g=1000, length_m=816, serial="000123")


class TestDecodeElegoo:
    @staticmethod
    def image(header: int = 0x36, mfr: bytes = b"\xee\xee\xee\xee") -> bytearray:
        """
        An Elegoo NTAG image, pages 0-15.

        :param header: header byte at 16
        :param mfr: manufacturer bytes at 17-20
        :return bytearray: the image
        """
        d = bytearray(64)
        d[16] = header
        d[17:21] = mfr
        d[21:23] = b"\x00\x01"
        d[23:27] = b"PLA "
        d[27:31] = b"CF20"
        d[31:34] = bytes([0xFF, 0x37, 0x10])
        d[34:36] = (285).to_bytes(2, "big")
        d[36:38] = (1000).to_bytes(2, "big")
        d[38:40] = (502).to_bytes(2, "big")
        return d

    def test_decode_elegoo_fields(self):
        assert decode_elegoo(bytes(self.image())) == {
            "manufacturer": "Elegoo", "type": "PLA", "detailed": "CF20",
            "color_argb": 0xFFFF3710, "diameter_mm": 2.85, "weight_g": 1000,
            "production": "0502"}

    def test_decode_elegoo_rejects_non_elegoo(self):
        assert decode_elegoo(bytes(64)) is None
        assert decode_elegoo(bytes(self.image(header=0x37))) is None
        assert decode_elegoo(bytes(self.image(mfr=b"\xee\xee\xee\xef"))) is None
        assert decode_elegoo(bytes(self.image())[:39]) is None


class TestBambuApplyMulticolor:
    PRIMARY = 0xFFE94B3C

    @staticmethod
    def block16(count: int, abgr: bytes = bytes(4), size: int = 1024) -> bytes:
        """
        An image with block 16 holding a colour count and a second colour.

        :param count: colour count u16 at 258
        :param abgr: second colour bytes at 260, A B G R
        :param size: image length
        :return bytes: the image
        """
        img = bytearray(1024)
        img[258:260] = count.to_bytes(2, "little")
        img[260:264] = abgr
        return bytes(img[:size])

    def test_bambu_apply_multicolor_dual(self):
        fil = {"color_argb": self.PRIMARY}
        _bambu_apply_multicolor(fil, self.block16(2, b"\xff\x33\x22\x11"))
        assert fil == {"color_argb": self.PRIMARY, "color_count": 2,
                       "colors_argb": [self.PRIMARY, 0xFF112233]}

    def test_bambu_apply_multicolor_single_count(self):
        fil = {"color_argb": self.PRIMARY}
        _bambu_apply_multicolor(fil, self.block16(1, b"\xff\x33\x22\x11"))
        assert fil == {"color_argb": self.PRIMARY, "color_count": 1,
                       "colors_argb": [self.PRIMARY]}

    def test_bambu_apply_multicolor_missing_block16(self):
        fil = {"color_argb": self.PRIMARY}
        _bambu_apply_multicolor(fil, None)
        assert fil == {"color_argb": self.PRIMARY, "color_count": 1,
                       "colors_argb": [self.PRIMARY]}

    def test_bambu_apply_multicolor_count2_but_empty_second_stays_single(self):
        # No second colour is added, so the count drops to match the list.
        fil = {"color_argb": self.PRIMARY}
        _bambu_apply_multicolor(fil, self.block16(2))
        assert fil == {"color_argb": self.PRIMARY, "color_count": 1,
                       "colors_argb": [self.PRIMARY]}

    def test_a_short_block16_image_is_ignored(self):
        fil = {"color_argb": self.PRIMARY}
        _bambu_apply_multicolor(fil, self.block16(2, b"\xff\x33\x22\x11", size=263))
        assert fil == {"color_argb": self.PRIMARY, "color_count": 1,
                       "colors_argb": [self.PRIMARY]}

    @pytest.mark.parametrize("abgr,second", [
        (b"\x80\x00\x00\x00", 0x80000000),     # alpha alone
        (b"\x00\x33\x00\x00", 0x00000033),     # blue alone
        (b"\x00\x00\x22\x00", 0x00002200),     # green alone
        (b"\x00\x00\x00\x11", 0x00110000),     # red alone
    ])
    def test_any_one_non_zero_channel_adds_the_second_colour(self, abgr, second):
        fil = {"color_argb": self.PRIMARY}
        _bambu_apply_multicolor(fil, self.block16(2, abgr))
        assert fil == {"color_argb": self.PRIMARY, "color_count": 2,
                       "colors_argb": [self.PRIMARY, second]}

    def test_a_block16_image_of_exactly_264_bytes_is_read(self):
        fil = {"color_argb": self.PRIMARY}
        _bambu_apply_multicolor(fil, self.block16(2, b"\xff\x33\x22\x11", size=264))
        assert fil == {"color_argb": self.PRIMARY, "color_count": 2,
                       "colors_argb": [self.PRIMARY, 0xFF112233]}

    def test_a_zero_count_reads_as_one(self):
        fil = {"color_argb": self.PRIMARY}
        _bambu_apply_multicolor(fil, self.block16(0, b"\xff\x33\x22\x11"))
        assert fil == {"color_argb": self.PRIMARY, "color_count": 1,
                       "colors_argb": [self.PRIMARY]}

    def test_no_primary_colour_leaves_only_the_second(self):
        fil: dict = {}
        _bambu_apply_multicolor(fil, self.block16(2, b"\x80\x33\x22\x11"))
        assert fil == {"color_count": 2, "colors_argb": [0x80112233]}


class TestDecodeBtt:
    def test_decode_btt_fields(self):
        assert decode_btt(rfid_btt_image()) == rfid_btt_fields()

    def test_decode_btt_density_parsed(self):
        assert decode_btt(rfid_btt_image(density=1240))["density"] == 1.24

    def test_decode_btt_density_zero_is_none(self):
        assert decode_btt(rfid_btt_image(density=0))["density"] is None

    def test_decode_btt_drying_parsed(self):
        f = decode_btt(rfid_btt_image(drying_time=8, drying_temp=70))
        assert (f["drying_time_h"], f["drying_temp_c"]) == (8, 70)

    def test_decode_btt_drying_zero_is_none(self):
        f = decode_btt(rfid_btt_image(drying_time=0, drying_temp=0))
        assert (f["drying_time_h"], f["drying_temp_c"]) == (None, None)

    def test_decode_btt_fingerprint_rejects_non_btt(self):
        assert decode_btt(rfid_btt_image(version=2)) is None
        assert decode_btt(bytes(1024)) is None

    def test_short_returns_none(self):
        assert decode_btt(rfid_btt_image()[:335]) is None
        assert decode_btt(rfid_btt_image()[:336]) == rfid_btt_fields()

    def test_the_bed_temperature_falls_back_to_the_bed_maximum(self):
        assert decode_btt(rfid_btt_image(bed=0, bed_max=70))["bed_temp_c"] == 70
        assert decode_btt(rfid_btt_image(bed=0, bed_max=0))["bed_temp_c"] is None

    def test_the_manufacturer_is_read_and_a_blank_one_reads_as_bq_tech(self):
        assert decode_btt(rfid_btt_image(manufacturer=b"Acme")) == rfid_btt_fields(
            manufacturer="Acme")
        assert decode_btt(rfid_btt_image(manufacturer=b"")) == rfid_btt_fields()

    def test_a_blank_weight_is_none(self):
        assert decode_btt(rfid_btt_image(weight=0)) == rfid_btt_fields(weight_g=None)

    def test_blank_hotend_temperatures_are_none(self):
        assert decode_btt(rfid_btt_image(hot_min=0, hot_max=0)) == rfid_btt_fields(
            hotend_min_c=None, hotend_max_c=None)

    def test_the_bed_temperature_wins_over_the_bed_maximum(self):
        assert decode_btt(rfid_btt_image(bed=65, bed_max=70)) == rfid_btt_fields(
            bed_temp_c=65)

    def test_a_blank_diameter_reads_as_1_75(self):
        assert decode_btt(rfid_btt_image(diameter=0))["diameter_mm"] == 1.75
        assert decode_btt(rfid_btt_image(diameter=2850))["diameter_mm"] == 2.85


class TestClassicBtt:
    UID = b"\xaa\xbb\xcc\xdd"

    def reader(self, keys_a: List[bytes]) -> Tuple[RfidSimLink, MifareClassic]:
        """
        A reader with a selected BTT tag protected by ``keys_a``.

        :param keys_a: the tag's sector keys
        :return Tuple[RfidSimLink, MifareClassic]: the link and the reader
        """
        tag = RfidSimTag(self.UID, rfid_btt_image(material="PLA"), keys_a=keys_a)
        link = RfidSimLink(tag)
        link.power_and_select(tag)
        return link, MifareClassic(Mfrc522(link))

    def test_classic_btt_reads_and_decodes(self):
        link, mc = self.reader(RFID_FF_KEYS)
        assert _classic_btt(mc, self.UID) == rfid_btt_fields(type="PLA")
        assert [event for event in link.trace if event.startswith("READ")] == [
            "READ 1", "READ 2", "READ 4", "READ 5", "READ 6", "READ 8", "READ 10",
            "READ 17", "READ 18", "READ 20"]

    def test_classic_btt_none_when_read_fails(self):
        link, mc = self.reader([b"\x01" * 6] * 16)
        assert _classic_btt(mc, self.UID) is None
        assert link.trace == ["AUTH 0"]


class TestReadTag:
    NEIGH = b"\xaa\xaa\xaa\xaa"
    OWN = b"\xbb\xbb\xbb\xbb"

    @staticmethod
    def bambu_tag(**tag_kw: object) -> RfidSimTag:
        """
        A Bambu Classic tag keyed to RFID_BAMBU_MASTER.

        :param tag_kw: extra RfidSimTag options
        :return RfidSimTag: the tag
        """
        return RfidSimTag(RFID_UID4, rfid_bambu_image(),
                          keys_a=rfid_bambu_keys(RFID_UID4, RFID_BAMBU_MASTER), **tag_kw)

    @staticmethod
    def ntag(image: bytes, **tag_kw: object) -> RfidSimTag:
        """
        An NTAG213 holding ``image`` from page 0.

        :param image: tag memory
        :param tag_kw: extra RfidSimTag options
        :return RfidSimTag: the tag
        """
        return RfidSimTag(RFID_UID7, image, ntag=True, **tag_kw)

    def test_no_tag_returns_none(self):
        link = RfidSimLink()
        assert read_tag(link) is None
        assert link.trace == ["SOFT-RESET", "FIELD-ON", "WUPA", "REQA"]

    def test_dump_blocks_attaches_raw_blocks(self):
        tag = self.bambu_tag()
        tag.memory[208:224] = bytes(range(0xD0, 0xE0))
        res = read_tag(RfidSimLink(tag), bambu_master_key=RFID_BAMBU_MASTER,
                       dump_blocks=(5, 13))
        assert res == {
            "uid": "04a1b2c3", "sak": 0x08, "tag_type": "MifareClassic1k",
            "filament": rfid_bambu_fields(color_count=1, colors_argb=[0xFF123456]),
            "raw_blocks": {5: "123456ffe80300000000e03f00000000",
                           13: "d0d1d2d3d4d5d6d7d8d9dadbdcdddedf"}}

    def test_dump_blocks_need_the_bambu_key(self):
        link = RfidSimLink(self.bambu_tag())
        res = read_tag(link, dump_blocks=(5, 13))
        assert res == {"uid": "04a1b2c3", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": None}
        assert link.trace.count("SOFT-RESET") == 1

    def test_ntag_without_decode(self):
        link = RfidSimLink(self.ntag(b"\x11" * 128, unreadable=[32]))
        assert read_tag(link) == {
            "uid": "04a1b2c3d4e5f6", "sak": 0x00, "tag_type": "MifareUltralight",
            "filament": None, "chip": None, "user_bytes": None}
        assert link.trace[-2:] == ["READ 28", "READ 32"]

    def test_a_failed_ntag_read_leaves_no_filament(self):
        # A plain Ultralight has 16 pages, so the 128-byte read runs off its end.
        link = RfidSimLink(self.ntag(b"\x11" * 64, pages=16))
        assert read_tag(link) == {"uid": "04a1b2c3d4e5f6", "sak": 0x00,
                                  "tag_type": "MifareUltralight", "filament": None}
        assert link.trace[-6:] == ["SELECT2 c3d4e5f6", "READ 0", "READ 4", "READ 8",
                                   "READ 12", "READ 16"]

    def test_anycubic_ntag_end_to_end(self):
        image = rfid_anycubic_image(cc=b"\xe1\x10\x12\x00")
        assert read_tag(RfidSimLink(self.ntag(image))) == {
            "uid": "04a1b2c3d4e5f6", "sak": 0x00, "tag_type": "MifareUltralight",
            "filament": rfid_anycubic_fields(), "chip": "NTAG213", "user_bytes": 144}

    def test_an_elegoo_ntag_is_decoded_when_anycubic_is_not(self):
        image = bytearray(128)
        image[16:21] = b"\x36\xee\xee\xee\xee"
        image[23:26] = b"PLA"
        res = read_tag(RfidSimLink(self.ntag(bytes(image))))
        assert res["filament"] == {
            "manufacturer": "Elegoo", "type": "PLA", "detailed": "",
            "color_argb": 0xFF000000, "diameter_mm": 0.0, "weight_g": 0,
            "production": "0000"}

    def test_the_afc_block_overlays_the_brand_decode(self):
        image = rfid_anycubic_image() + rfid_afc_block(weight=823, spool=136)
        res = read_tag(RfidSimLink(self.ntag(image)))
        assert res["filament"] == rfid_anycubic_fields(weight_g=823, spool_id=136)

    def test_the_afc_block_alone_still_gives_a_filament(self):
        image = bytes(128) + rfid_afc_block(weight=823)
        res = read_tag(RfidSimLink(self.ntag(image)))
        assert res["filament"] == {"weight_g": 823}

    def test_read_tag_classic_bambu_with_key(self):
        link = RfidSimLink(self.bambu_tag())
        res = read_tag(link, RFID_BAMBU_MASTER)
        assert res == {"uid": "04a1b2c3", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": rfid_bambu_fields(color_count=1,
                                                     colors_argb=[0xFF123456])}
        assert [event for event in link.trace if event.startswith("AUTH")] == [
            "AUTH 0", "AUTH 4", "AUTH 8", "AUTH 12", "AUTH 16"]

    def test_read_tag_returns_uid_even_without_key(self):
        link = RfidSimLink(self.bambu_tag())
        res = read_tag(link, bambu_master_key=None)
        assert res == {"uid": "04a1b2c3", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": None}
        # Snapmaker then BTT, each turned away at sector 0, then QIDI at
        # sector 1.
        assert [event for event in link.trace if event.startswith("AUTH")] == [
            "AUTH 0", "AUTH 0", "AUTH 4"]

    def test_a_creality_tag_is_read_with_both_keys(self):
        uid = bytes.fromhex("60EA1221")
        u_key = bytes.fromhex("713362755e74316e71665a2870662431")
        d_key = bytes.fromhex("484043466b526e7a404b4174424a7032")
        image = bytes(64) + rfid_cbc_encrypt(rfid_creality_payload(), d_key)
        tag = RfidSimTag(uid, image, keys_a=[bytes.fromhex("1f1e83a97182")] * 16)
        res = read_tag(RfidSimLink(tag), creality_key=u_key, creality_encryption_key=d_key)
        assert res == {"uid": "60ea1221", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": rfid_creality_fields()}

    @pytest.mark.parametrize("keys", ["creality_key", "creality_encryption_key"])
    def test_creality_needs_both_keys(self, keys):
        uid = bytes.fromhex("60EA1221")
        given = {"creality_key": bytes.fromhex("713362755e74316e71665a2870662431"),
                 "creality_encryption_key": bytes.fromhex("484043466b526e7a404b4174424a7032")}
        image = bytes(64) + rfid_cbc_encrypt(rfid_creality_payload(),
                                             given["creality_encryption_key"])
        tag = RfidSimTag(uid, image, keys_a=[bytes.fromhex("1f1e83a97182")] * 16)
        link = RfidSimLink(tag)
        assert read_tag(link, **{keys: given[keys]})["filament"] is None
        # Snapmaker and BTT both try sector 0 and QIDI sector 1. Creality runs
        # between Snapmaker and BTT, so it would show as an AUTH 4 there.
        assert [event for event in link.trace if event.startswith("AUTH")] == [
            "AUTH 0", "AUTH 0", "AUTH 4"]

    def test_a_tag_gone_before_a_later_scheme_ends_the_search(self):
        link = RfidSimLink(self.bambu_tag(wake_limit=1))
        res = read_tag(link)
        assert res == {"uid": "04a1b2c3", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": None}
        # Snapmaker fails at sector 0; the BTT re-select finds an empty field.
        assert link.trace[-4:] == ["AUTH 0", "CRYPTO-OFF", "WUPA", "REQA"]

    def test_a_tag_gone_before_the_dump_skips_it(self):
        link = RfidSimLink(self.bambu_tag(wake_limit=1))
        res = read_tag(link, RFID_BAMBU_MASTER, dump_blocks=(5,))
        assert res == {"uid": "04a1b2c3", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": rfid_bambu_fields(color_count=1,
                                                     colors_argb=[0xFF123456])}
        assert link.trace[-5:] == ["READ 16", "SOFT-RESET", "FIELD-ON", "WUPA", "REQA"]

    def test_a_failed_dump_read_is_skipped(self):
        link = RfidSimLink(self.bambu_tag(unreadable=[13]))
        res = read_tag(link, RFID_BAMBU_MASTER, dump_blocks=(13,))
        assert res == {"uid": "04a1b2c3", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": rfid_bambu_fields(color_count=1,
                                                     colors_argb=[0xFF123456])}
        assert link.trace[-7:] == ["SOFT-RESET", "FIELD-ON", "WUPA", "ANTICOLL1",
                                   "SELECT1 04a1b2c3", "AUTH 12", "READ 13"]

    def test_a_link_error_during_the_dump_is_swallowed(self):
        class DroppingLink(RfidSimLink):
            """A link whose second SoftReset fails, as a dropped serial link would."""

            def _soft_reset(self) -> None:
                """SoftReset, then fail the second time."""
                super()._soft_reset()
                if self.trace.count("SOFT-RESET") == 2:
                    raise OSError("link dropped")

        link = DroppingLink(self.bambu_tag())
        res = read_tag(link, RFID_BAMBU_MASTER, dump_blocks=(5,))
        assert res == {"uid": "04a1b2c3", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": rfid_bambu_fields(color_count=1,
                                                     colors_argb=[0xFF123456])}
        assert link.trace[-1] == "SOFT-RESET"

    def test_reselect_pins_to_active_uid_not_sibling_predicate(self):
        # A sibling predicate that only misfires on calls after the first
        # activation must not halt this lane's own tag on the re-select.
        own = RfidSimTag(self.OWN, rfid_snapmaker_image(),
                         keys_a=rfid_snapmaker_keys(self.OWN))
        neigh = RfidSimTag(self.NEIGH)
        link = RfidSimLink(neigh, own)
        asked: List[str] = []

        def sibling_pred(uid_hex: str) -> bool:
            asked.append(uid_hex)
            return (uid_hex == "aaaaaaaa"
                    or len(asked) > 2)

        seen: list = []
        res = read_tag(link, bambu_master_key=RFID_BAMBU_MASTER, is_excluded=sibling_pred,
                       seen=seen)
        assert res == {"uid": "bbbbbbbb", "sak": 0x08, "tag_type": "MifareClassic1k",
                       "filament": RFID_SNAPMAKER_FIELDS}
        assert asked == ["aaaaaaaa", "bbbbbbbb"]
        assert seen == [("aaaaaaaa", 0x08, True), ("bbbbbbbb", 0x08, False),
                        ("aaaaaaaa", 0x08, True), ("bbbbbbbb", 0x08, False)]
        assert neigh.state == "halt"
        # The re-select ends Crypto1 and keeps the field up: one SoftReset only.
        assert link.trace.count("SOFT-RESET") == 1
        assert link.trace.count("CRYPTO-OFF") == 1


class TestIsWritableClassicBlock:
    def test_trailers_and_block0_are_refused(self):
        blocks = (0, 3, 7, 11, 15, 63)
        assert [_is_writable_classic_block(b) for b in blocks] == [False] * len(blocks)

    def test_data_blocks_are_allowed(self):
        blocks = (1, 2, 4, 5, 6, 13, 62)
        assert [_is_writable_classic_block(b) for b in blocks] == [True] * len(blocks)


class TestClassicWriteBlock:
    UID = b"\x01\x02\x03\x04"

    def field(self, **tag_kw: object) -> Tuple[RfidSimTag, RfidSimLink]:
        """
        A reader with one blank-keyed Classic tag, block n holding n repeated.

        :param tag_kw: extra RfidSimTag options
        :return Tuple[RfidSimTag, RfidSimLink]: the tag and the link
        """
        image = b"".join(bytes([n]) * 16 for n in range(64))
        tag = RfidSimTag(self.UID, image, **tag_kw)
        return tag, RfidSimLink(tag)

    def test_a_clean_write_verifies(self):
        # Block 5 is not first in its sector, so auth must go to block 4.
        tag, link = self.field()
        assert classic_write_block(link, 5, b"\xab" * 16) == ("01020304", None)
        assert tag.memory[80:96] == b"\xab" * 16
        assert tag.memory[64:80] == b"\x04" * 16
        assert link.trace == ["SOFT-RESET", "FIELD-ON", "WUPA", "ANTICOLL1",
                              "SELECT1 01020304", "AUTH 4", "WRITE-CLASSIC 5", "DATA 5",
                              "READ 5", "CRYPTO-OFF"]

    @pytest.mark.parametrize("block", [0, 7])
    def test_a_trailer_is_refused(self, block):
        tag, link = self.field()
        before = bytes(tag.memory)
        assert classic_write_block(link, block, b"\x00" * 16) == (
            None, f"block {block} is a sector trailer or block 0; refusing to write it")
        assert link.trace == []
        assert bytes(tag.memory) == before

    def test_wrong_key_is_reported(self):
        tag, link = self.field()
        assert classic_write_block(link, 4, b"\x00" * 16, key6=b"\x00" * 6) == (
            "01020304", "key did not authenticate sector 1 (wrong key for this card?)")
        assert link.trace[-2:] == ["AUTH 4", "CRYPTO-OFF"]
        assert tag.memory[64:80] == b"\x04" * 16

    def test_the_key_type_reaches_the_auth_frame(self):
        # The sim only accepts Key A, so a Key B auth is refused.
        tag, link = self.field()
        assert classic_write_block(link, 4, b"\x00" * 16, key_type=0x61) == (
            "01020304", "key did not authenticate sector 1 (wrong key for this card?)")
        assert link.trace[-2:] == ["AUTH-B 4", "CRYPTO-OFF"]
        assert tag.memory[64:80] == b"\x04" * 16

    def test_a_locked_block_nak_is_reported(self):
        tag, link = self.field(nak_writes=[4])
        assert classic_write_block(link, 4, b"\x00" * 16) == (
            "01020304", "tag did not ACK the write to block 4")
        assert tag.memory[64:80] == b"\x04" * 16

    def test_a_block_that_reads_back_wrong_is_reported(self):
        _, link = self.field(corrupt={4: b"\xee" * 16})
        assert classic_write_block(link, 4, b"\x00" * 16) == (
            "01020304", f"block 4 reads back {'ee' * 16}, not what was written")

    def test_a_block_that_cannot_be_read_back_is_reported(self):
        _, link = self.field(unreadable=[4])
        assert classic_write_block(link, 4, b"\x00" * 16) == (
            "01020304", "block 4 reads back None, not what was written")

    def test_an_ntag_is_refused(self):
        link = RfidSimLink(RfidSimTag(RFID_UID7, ntag=True))
        assert classic_write_block(link, 4, b"\x00" * 16) == (
            "04a1b2c3d4e5f6", "that is not a MIFARE Classic tag (sak 0x00)")
        assert link.trace[-1] == "SELECT2 c3d4e5f6"

    def test_an_empty_field_is_reported(self):
        link = RfidSimLink()
        assert classic_write_block(link, 4, b"\x00" * 16) == (
            None, "no tag in the reader's field")

    @pytest.mark.parametrize("size", [8, 17])
    def test_data_must_be_16_bytes(self, size):
        _, link = self.field()
        assert classic_write_block(link, 4, b"\x00" * size) == (
            None, f"data is {size} bytes, must be 16")
        assert link.trace == []


class TestBambuClassicWriteTest:
    ORIG = bytes(range(0xD0, 0xE0))
    FLIPPED = bytes([0xD0 ^ 0xFF]) + bytes(range(0xD1, 0xE0))

    def field(self, **tag_kw: object) -> Tuple[RfidSimTag, RfidSimLink]:
        """
        A reader with a Bambu tag: block 13 known, trailer 15 with access bytes.

        :param tag_kw: extra RfidSimTag options
        :return Tuple[RfidSimTag, RfidSimLink]: the tag and the link
        """
        tag = RfidSimTag(RFID_UID4, rfid_bambu_image(),
                         keys_a=rfid_bambu_keys(RFID_UID4, RFID_BAMBU_MASTER), **tag_kw)
        tag.memory[208:224] = self.ORIG
        tag.memory[240:256] = bytes(6) + b"\xff\x07\x80\x69" + bytes(6)
        return tag, RfidSimLink(tag)

    def test_write_test_refuses_a_trailer(self):
        link = RfidSimLink()
        assert bambu_classic_write_test(link, b"k" * 16, block=3) == {
            "error": "block 3 is a sector trailer or block 0; refusing to write it"}
        assert link.trace == []

    def test_a_write_is_proved_and_undone(self):
        tag, link = self.field()
        assert bambu_classic_write_test(link, RFID_BAMBU_MASTER) == {
            "uid": "04a1b2c3", "block": 13, "orig": self.ORIG.hex(),
            "modified": self.FLIPPED.hex(), "access": "ff0780",
            "wrote": True, "changed": True, "read_after_write": self.FLIPPED.hex(),
            "restored_ack": True, "restored": True, "read_after_restore": self.ORIG.hex()}
        assert tag.memory[208:224] == self.ORIG
        assert link.trace[5:] == ["AUTH 12", "READ 13", "READ 15", "WRITE-CLASSIC 13",
                                  "DATA 13", "READ 13", "WRITE-CLASSIC 13", "DATA 13",
                                  "READ 13", "CRYPTO-OFF"]

    def test_a_locked_block_reports_every_step_failed(self):
        tag, _ = self.field(nak_writes=[13])
        res = bambu_classic_write_test(RfidSimLink(tag), RFID_BAMBU_MASTER)
        assert res == {
            "uid": "04a1b2c3", "block": 13, "orig": self.ORIG.hex(),
            "modified": self.FLIPPED.hex(), "access": "ff0780",
            "wrote": False, "changed": False, "read_after_write": None,
            "restored_ack": False, "restored": False, "read_after_restore": None}

    def test_an_unreadable_trailer_leaves_access_unknown(self):
        _, link = self.field(unreadable=[15])
        # The trailer NAK drops the tag out of its Crypto1 session, so every later
        # step fails too.
        assert bambu_classic_write_test(link, RFID_BAMBU_MASTER) == {
            "uid": "04a1b2c3", "block": 13, "orig": self.ORIG.hex(),
            "modified": self.FLIPPED.hex(), "access": None,
            "wrote": False, "changed": False, "read_after_write": None,
            "restored_ack": False, "restored": False, "read_after_restore": None}
        assert link.trace[5:] == ["AUTH 12", "READ 13", "READ 15", "WRITE-CLASSIC 13",
                                  "READ 13", "WRITE-CLASSIC 13", "READ 13", "CRYPTO-OFF"]

    def test_an_empty_field_is_reported(self):
        assert bambu_classic_write_test(RfidSimLink(), RFID_BAMBU_MASTER) == {
            "error": "no tag in the field"}

    def test_an_ntag_is_refused(self):
        link = RfidSimLink(RfidSimTag(RFID_UID7, ntag=True))
        assert bambu_classic_write_test(link, RFID_BAMBU_MASTER) == {
            "error": "not a MIFARE Classic tag (sak 0x00)", "uid": "04a1b2c3d4e5f6"}

    def test_a_wrong_master_key_is_reported(self):
        _, link = self.field()
        assert bambu_classic_write_test(link, bytes(16)) == {
            "error": "Key A did not authenticate sector 3", "uid": "04a1b2c3"}
        assert link.trace[-2:] == ["AUTH 12", "CRYPTO-OFF"]

    def test_an_unreadable_block_is_reported(self):
        _, link = self.field(unreadable=[13])
        assert bambu_classic_write_test(link, RFID_BAMBU_MASTER) == {
            "error": "could not read block 13", "uid": "04a1b2c3"}
        assert link.trace[-1] == "CRYPTO-OFF"


class TestReadBambu:
    @staticmethod
    def field() -> RfidSimLink:
        """
        A reader with a Bambu tag keyed to RFID_BAMBU_MASTER.

        :return RfidSimLink: the link
        """
        return RfidSimLink(RfidSimTag(RFID_UID4, rfid_bambu_image(),
                                      keys_a=rfid_bambu_keys(RFID_UID4, RFID_BAMBU_MASTER)))

    def test_full_read_and_decode_end_to_end(self):
        link = self.field()
        assert read_bambu(link, RFID_BAMBU_MASTER) == rfid_bambu_fields(uid="04a1b2c3")
        assert [event for event in link.trace if event.startswith("AUTH")] == [
            f"AUTH {sector * 4}" for sector in range(16)]

    def test_wrong_master_key_fails_auth(self):
        link = self.field()
        assert read_bambu(link, bytes(16)) is None
        assert link.trace[-1] == "AUTH 0"

    def test_an_empty_field_returns_none(self):
        link = RfidSimLink()
        assert read_bambu(link, RFID_BAMBU_MASTER) is None
        assert link.trace == ["SOFT-RESET", "FIELD-ON", "WUPA", "REQA"]


class TestWriteTag:
    PAYLOAD = bytes(range(112))

    @staticmethod
    def blank_ntag(uid: bytes = RFID_UID7, **tag_kw: object) -> RfidSimTag:
        """
        A blank NTAG213 with its capability container.

        :param uid: tag UID
        :param tag_kw: extra RfidSimTag options
        :return RfidSimTag: the tag
        """
        return RfidSimTag(uid, bytes(12) + b"\xe1\x10\x12\x00", ntag=True, **tag_kw)

    @staticmethod
    def writes(link: RfidSimLink) -> List[str]:
        """
        The page writes in a trace.

        :param link: the reader link
        :return List[str]: its WRITE events
        """
        return [event for event in link.trace if event.startswith("WRITE")]

    def test_an_ntag_is_written_at_page_four(self):
        tag = self.blank_ntag()
        link = RfidSimLink(tag)
        assert write_tag(link, self.PAYLOAD) == ("04a1b2c3d4e5f6", None)
        assert tag.memory[16:128] == self.PAYLOAD
        assert self.writes(link) == [f"WRITE {page}" for page in range(4, 32)]

    def test_an_empty_field_is_reported_not_written(self):
        link = RfidSimLink()
        assert write_tag(link, self.PAYLOAD) == (None, "no tag in the reader's field")
        assert self.writes(link) == []

    def test_a_mifare_classic_is_refused_by_name(self):
        """A Bambu or Snapmaker tag in the field is turned away, not attempted."""
        tag = RfidSimTag(b"\x01\x02\x03\x04")
        link = RfidSimLink(tag)
        assert write_tag(link, self.PAYLOAD) == (
            "01020304", "that is a MIFARE Classic tag, not an NTAG; this writer only "
            "programs blank NTAG stickers")
        assert link.trace[-1] == "SELECT1 01020304"
        assert bytes(tag.memory) == bytes(1024)

    def test_a_write_failure_is_passed_through(self):
        link = RfidSimLink(self.blank_ntag(nak_writes=[9]))
        assert write_tag(link, self.PAYLOAD) == (
            "04a1b2c3d4e5f6",
            "tag did not ACK the write to page 9 [resp (True, '0x1', 1, 4, '0x0')]")

    def test_a_neighbour_tag_is_halted_and_left_unwritten(self):
        neigh = self.blank_ntag(bytes.fromhex("04111111111111"))
        own = self.blank_ntag()
        link = RfidSimLink(neigh, own)
        assert write_tag(link, self.PAYLOAD, is_excluded=lambda h: h == "04111111111111") == (
            "04a1b2c3d4e5f6", None)
        assert (neigh.state, neigh.memory[16:128]) == ("halt", bytes(112))
        assert own.memory[16:128] == self.PAYLOAD
