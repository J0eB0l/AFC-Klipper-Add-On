# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# OpenAMS RFID integration for AFC.
#
# The OpenAMS mainboard has two RFID antenna boards (RFID A / RFID B), each an
# FM17580 reader: a Fudan MFRC522-class chip (13.56 MHz, ISO14443A, MIFARE
# Classic) on an 8-pin MFRC522 header. RFID A is on SPI1 and RFID B on SPI2, so
# each reader section gets its own `spi_bus:` plus a `cs_pin`. A shared bus with
# per-reader CS pins also works.
#
# The stock OpenAMS firmware is built without SPI, so this module only works once
# the firmware is rebuilt with SPI on the RFID pins. Until then a reader section
# fails to build its SPI at config time.
#
# This is transport and wiring only: each reader's MCU_SPI is adapted to the
# reg_read/reg_write contract of the shared reader stack (read_tag) used by the
# ACE2/ViViD, so every format it decodes (see AFC_rfid_readers) reads here and
# syncs to Spoolman through AFC_RFID.
from __future__ import annotations
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional

# The shared transport-agnostic MFRC522 + MIFARE + multi-manufacturer
# decode stack. read_tag() only needs an object exposing reg_read/reg_write.
from extras.AFC_rfid_readers import read_tag, Mfrc522, MifareClassic
from extras.AFC_rfid_write import StageError, register_reader
from extras.AFC_RFID import (AFCUnitRFID,
                             map_tag_to_slot_info)
try:
    from extras.AFC_RFID import get_auto_spoolman_create
except ImportError:                                    # older AFC without the helper
    get_auto_spoolman_create = None
try:
    from extras.AFC_RFID import resolve_rfid_keys
except ImportError:                                    # older AFC without the helper
    resolve_rfid_keys = None

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand

# MFRC522 SPI: mode 0. The address byte is (reg<<1); read sets the MSB (0x80).
# The FM17580 is register-compatible with the MFRC522, so this framing is shared
# with the ViViD/ACE2. 5 MHz is a safe default; the FM17580 SPI tops out at 10.
_MFRC522_SPI_MODE = 0
_MFRC522_SPI_SPEED = 5000000


class _OamsSpiRegLink:
    """
    Adapt a Klipper ``MCU_SPI`` to the reg_read/reg_write reader contract the
    shared Mfrc522/MifareClassic/read_tag stack expects.

    MFRC522 SPI framing: write = ``[(reg<<1), val]``; read = ``[0x80|(reg<<1),
    0x00]`` with the value in the second returned byte.

    :param spi: Klipper ``MCU_SPI`` for this reader.
    """

    def __init__(self, spi: Any) -> None:
        """
        Register link over a Klipper SPI object.

        :param spi: the MCU SPI object
        """
        self.spi = spi

    def reg_read(self, reg: int) -> int:
        """
        Read one MFRC522 register over SPI (address byte 0x80|(reg<<1)).

        :param reg: MFRC522 register address.
        :return int: The register value.
        """
        addr = 0x80 | ((reg << 1) & 0x7E)
        params = self.spi.spi_transfer([addr, 0x00])
        resp = bytearray(params['response'])
        return resp[1] if len(resp) > 1 else 0

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one MFRC522 register over SPI (address byte (reg<<1)).

        :param reg: MFRC522 register address.
        :param val: Byte value to write.
        """
        self.spi.spi_send([(reg << 1) & 0x7E, val & 0xFF])

    def reader_power(self, on: bool) -> None:
        """
        No-op RF-field toggle (the OpenAMS readers have no coil-enable line).

        The field is driven through the MFRC522 TxControlReg by the Mfrc522 class.

        :param on: Requested power state (ignored).
        """
        return None


class AFC_OpenAMS_rfid_reader:
    """
    One FM17580 reader on the OpenAMS (RFID A or RFID B), wrapping a Klipper
    ``MCU_SPI``.

    Configured as ``[AFC_OpenAMS_rfid <name>]`` with ``cs_pin`` (+ ``spi_bus`` or
    software-SPI pins), an optional ``reset_pin``, and the ``slots`` it serves.
    Both readers share one SPI bus and differ only by their CS pin.

    :param config: Klipper config wrapper for the named reader section.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Set up one OpenAMS RFID reader section.

        :param config: the [AFC_OpenAMS_rfid <name>] section
        """
        self.printer = config.get_printer()
        # Section is "AFC_OpenAMS_rfid <name>"; keep the trailing name.
        self.name = config.get_name().split()[-1]
        # Take AFC's logger at construction; load_object builds AFC if needed.
        self.logger = self.printer.load_object(config, "AFC").logger
        # Build the SPI at config time. `bus` (klippy/bus.py) is imported here so
        # the module still loads without it in unit tests.
        try:
            from . import bus            # hardware: extras.bus
        except (ImportError, ValueError):
            import bus                   # top-level import fallback (tests)
        # transport: only "spi" (the stock firmware's native SPI). Still read
        # so existing sections that spell it out keep loading.
        transport = config.get("transport", "spi").strip().lower()
        if transport != "spi":
            error_str = (f"AFC_OpenAMS_rfid {config.get_name().split()[-1]}: "
                         f"unknown transport {transport!r} (spi)")
            raise config.error(error_str)
        self.spi = bus.MCU_SPI_from_config(
            config, _MFRC522_SPI_MODE, pin_option="cs_pin",
            default_speed=_MFRC522_SPI_SPEED, cs_active_high=False)
        self.link = _OamsSpiRegLink(self.spi)
        # Optional hardware reset / power-down line (MFRC522 NRSTPD). Held HIGH
        # to keep the chip enabled; the Mfrc522 class does a soft reset on read.
        self._reset_pin = None
        reset_pin = config.get("reset_pin", None)
        if reset_pin:
            ppins = self.printer.lookup_object("pins")
            self._reset_pin = ppins.setup_pin("digital_out", reset_pin)
            self._reset_pin.setup_start_value(1, 1)     # enabled at boot, stays high
        # Physical slots this reader's antenna covers.
        self.slots: List[int] = []
        for s in (config.get("slots", "") or "").split(","):
            s = s.strip()
            if not s:
                continue
            try:
                self.slots.append(int(s))
            except ValueError:
                error_str = f"AFC_OpenAMS_rfid {self.name}: bad slot number {s!r} in 'slots'"
                raise config.error(error_str)


class AFC_OpenAMS_rfid(AFCUnitRFID):
    """
    Coordinator for the OpenAMS RFID readers. Configured as
    ``[AFC_OpenAMS_rfid]``.

    Ties the per-reader ``[AFC_OpenAMS_rfid <name>]`` sections to lanes via a
    ``lane_slot_map`` and drives a read: pick the lane's slot, read that slot's
    reader, decode and apply to the lane through the shared AFC_RFID path.

    :param config: Klipper config wrapper for the bare ``[AFC_OpenAMS_rfid]``
        section.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Set up the OpenAMS RFID coordinator: readers, keys and commands.

        :param config: the [AFC_OpenAMS_rfid] section
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        # Take AFC's logger at construction; load_object builds AFC if needed.
        # self.afc stays None until klippy:connect: guards use it as the ready marker.
        self.logger = self.printer.load_object(config, "AFC").logger
        self.afc = None
        self.log_prefix = "OpenAMS RFID"       # AFCUnitRFID.apply_to_lane / Spoolman
        self.auto_create = config.getboolean("auto_spoolman_create", False)

        # Shared decode keys: a reader section may override, else fall back to the
        # printer-wide [AFC_rfid_keys].
        bk = config.get("bambu_master_key", None)
        ck = config.get("creality_key", None)
        cek = config.get("creality_encryption_key", None)
        self.bambu_master_key = bytes.fromhex(bk) if bk else None
        self.creality_key = bytes.fromhex(ck) if ck else None
        self.creality_encryption_key = bytes.fromhex(cek) if cek else None
        if resolve_rfid_keys is not None:
            self.bambu_master_key, self.creality_key, self.creality_encryption_key = \
                resolve_rfid_keys(self.printer, self.bambu_master_key,
                                  self.creality_key, self.creality_encryption_key)
        # Warn loudly: a missing key makes every Bambu tag uid-only, which looks
        # like an antenna/positioning problem.
        if resolve_rfid_keys is None:
            self.logger.warning(
                "AFC_OpenAMS_rfid: AFC_RFID.py is too old to resolve "
                "[AFC_rfid_keys]. Update it or Bambu keys are ignored")
        elif self.bambu_master_key is None:
            self.logger.warning(
                "AFC_OpenAMS_rfid: no bambu_master_key resolved (section "
                "option or [AFC_rfid_keys]). Bambu tags will not decode")

        # lane -> physical slot (e.g. "lane4:0, lane5:1, ...").
        self._lane_slot: Dict[str, int] = {}
        for pair in (config.get("lane_slot_map", "") or "").split(","):
            pair = pair.strip()
            if not pair:
                continue
            try:
                lane, slot = pair.split(":")
                self._lane_slot[lane.strip()] = int(slot)
            except ValueError:
                error_str = ("AFC_OpenAMS_rfid: 'lane_slot_map' entries must be "
                             f"'lane:slot'. Bad entry {pair!r}")
                raise config.error(error_str)

        self._slot_reader: Dict[int, AFC_OpenAMS_rfid_reader] = {}
        self._last: Dict[int, dict] = {}       # slot -> last raw tag
        # Slots already warned as having no reader, so a scan's rapid polling
        # logs it once.
        self._no_reader_warned: set[int] = set()

        self.gcode.register_command(
            "OAMS_RFID_READ", self.cmd_OAMS_RFID_READ,
            desc="Read an OpenAMS RFID tag: OAMS_RFID_READ LANE=<name> | SLOT=<n>")
        self.printer.register_event_handler("klippy:connect", self._on_connect)

    def _on_connect(self) -> None:
        """
        Resolve the AFC object and index each reader by the slots it serves.

        Indexing by physical slot lets a lane's slot map straight to its reader.
        """
        self.afc = self.printer.lookup_object("AFC", None)
        for name, obj in self.printer.lookup_objects():
            if (name.startswith("AFC_OpenAMS_rfid ")
                and isinstance(obj, AFC_OpenAMS_rfid_reader)):
                for slot in obj.slots:
                    if slot in self._slot_reader:
                        self.logger.warning(
                            f"AFC_OpenAMS_rfid: slot {slot} served by "
                            f"more than one reader "
                            f"({self._slot_reader[slot].name} and "
                            f"{obj.name})")
                    self._slot_reader[slot] = obj
        # Offer every reader to the shared tag writer (AFC_RFID_WRITE). The SPI
        # link exists from config, so it is always available.
        for rdr in dict.fromkeys(self._slot_reader.values()):
            register_reader(
                self.printer, f"oams:{rdr.name}",
                f"OpenAMS {rdr.name} (slots "
                f"{', '.join(str(s) for s in rdr.slots)})",
                self, lambda r=rdr: r.link,
                stage_around=lambda ln, body, r=rdr: self._stage_write_around(
                    ln, r, body),
                serves=lambda ln, r=rdr: self._lane_slot.get(ln) in r.slots)
        if not self._slot_reader:
            self.logger.warning(
                "AFC_OpenAMS_rfid: no [AFC_OpenAMS_rfid <name>] reader sections "
                "found. RFID reads will no-op")

    def _stage_write_around(self, lane_name: str, rdr: Any,
                            body: Callable[..., Any]) -> None:
        """
        AFC_RFID_WRITE / AFC_RFID_ENROLL LANE= on an OpenAMS lane: run the
        unit's scan feed (AFC_OAMS_RFID_SCAN), write the tag where the scan
        stops on it, then unwind back to the bay. Runs on the reactor.

        :param lane_name: the lane whose spool carries the tag
        :param rdr: the reader the write was sent to
        :param body: the write, called with the sister-tag excluder
        :raises StageError: when the spool must not or cannot be turned
        """
        lanes = getattr(self.afc, "lanes", None) or {}
        lane = lanes.get(lane_name)
        unit = getattr(lane, "unit_obj", None) if lane is not None else None
        if lane is None or not hasattr(unit, "_do_rfid_scan"):
            error_str = f"{lane_name} is not an OpenAMS lane"
            raise StageError(error_str)
        rfid_slot = unit._rfid_scan_slot(self, lane)
        if rfid_slot not in getattr(rdr, "slots", ()):
            owner = self._slot_reader.get(rfid_slot)
            hint = (f"; write it with READER=oams:{owner.name}"
                    if owner is not None else "")
            error_str = f"{lane_name} is not on {rdr.name}{hint}"
            raise StageError(error_str)
        if unit._operation_active:
            error_str = ("the OpenAMS is busy; try again once the current "
                         "operation completes")
            raise StageError(error_str)
        reason = unit._rfid_scan_blocked_reason(lane)
        if reason is not None:
            error_str = f"the scan feed is blocked: {reason}"
            raise StageError(error_str)
        slot = unit._spool_map.get(lane_name)
        if slot is None or unit.oams is None or not unit.oams.is_bay_ready(slot):
            error_str = (f"no filament inserted in {lane_name} (the write "
                         f"feeds the lane to turn the spool)")
            raise StageError(error_str)
        held: List[bool] = []

        def _hold(excluded: Callable[[str], bool]) -> None:
            """
            Run the write with the tag stopped at the reader.

            :param excluded: sister-tag excluder passed through to the write
            """
            held.append(True)
            body(excluded)

        unit._rfid_scanned.discard(lane_name)
        unit._do_rfid_scan(lane, hold=_hold)
        if not held:
            error_str = (f"no tag on {lane_name} came to rest at the "
                         f"reader; the lane is back in its bay")
            raise StageError(error_str)

    def _get_slot(self, lane_name: str) -> Optional[int]:
        """
        Return the physical slot a lane maps to, or None if unmapped.

        :param lane_name: AFC lane name.
        :return int: The mapped physical slot, or None if unmapped.
        """
        return self._lane_slot.get(lane_name)

    def read_slot(self, slot: int) -> Optional[dict]:
        """
        Read the tag on a physical slot via its reader; return the raw tag
        dict (uid + decoded filament), or None.

        The tag is only reliably in range while the spool is spinning during a
        feed, so callers read repeatedly across a feed (stage-read integration).

        :param slot: Physical OpenAMS slot index.
        :return dict: Raw read_tag() result, or None if no reader / no tag.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            if slot not in self._no_reader_warned:
                self._no_reader_warned.add(slot)
                self.logger.warning(f"AFC_OpenAMS_rfid: no reader configured for slot {slot}")
            return None
        tag = read_tag(
            reader.link, bambu_master_key=self.bambu_master_key,
            creality_key=self.creality_key,
            creality_encryption_key=self.creality_encryption_key)
        if tag:
            self._last[slot] = tag
        return tag

    def scan_slot_uids(self, slot: int) -> List[str]:
        """
        Enumerate every tag UID currently in a slot's reader field.

        Used before a scan feed to take the sister-tag baseline: on the shared
        antenna a seated neighbour spool's tag may already be in range, and
        only a new uid appearing during the feed belongs to the moving spool.
        Each found tag is HALTed so the next request reaches the one behind it.

        :param slot: Physical OpenAMS slot index.
        :return list: uid hex strings for every tag seen (empty field = []).
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return []
        seen: List[Any] = []
        try:
            MifareClassic(Mfrc522(reader.link)).activate(
                is_excluded=lambda h: True, seen=seen)
        except Exception as e:
            self.logger.debug(f"scan_slot_uids failed on slot {slot}: {e}")
        return [u for (u, _sak, _ex) in seen]

    def detect_slot_tag(self, slot: int, exclude: Any) -> Optional[str]:
        """
        Light presence probe: REQA/anticollision/select only, with no auth or
        block reads, so it is fast enough to poll while the spool turns. Sister
        tags (uids in ``exclude``) are HALTed per the shared-reader protocol so
        the moving spool's own tag can answer.

        :param slot: Physical OpenAMS slot index.
        :param exclude: container of uid hex strings to ignore (sisters).
        :return str: a NEW tag's uid hex, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        try:
            uid, _sak = MifareClassic(Mfrc522(reader.link)).activate(
                is_excluded=lambda h: h in exclude)
        except Exception as e:
            self.logger.debug(f"detect_slot_tag failed on slot {slot}: {e}")
            return None
        return uid.hex() if uid is not None else None

    def read_slot_excluding(self, slot: int, exclude: Any) -> Optional[dict]:
        """
        Full tag read on a slot with sister uids excluded (HALTed), so the
        stationary read after a scan-feed detection can't decode the
        neighbour's seated tag by mistake.

        :param slot: Physical OpenAMS slot index.
        :param exclude: container of uid hex strings to ignore (sisters).
        :return dict: Raw read_tag() result, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        tag = read_tag(
            reader.link, bambu_master_key=self.bambu_master_key,
            creality_key=self.creality_key,
            creality_encryption_key=self.creality_encryption_key,
            is_excluded=lambda h: h in exclude)
        if tag:
            self._last[slot] = tag
        return tag

    def read_lane(self, lane_name: str) -> Optional[dict]:
        """
        Read a lane's tag (by its mapped slot) and apply it. Returns slot_info
        or None.

        :param lane_name: AFC lane name.
        :return dict: The applied slot_info, or None if unmapped / no tag.
        """
        slot = self._get_slot(lane_name)
        if slot is None:
            self.logger.warning(
                f"AFC_OpenAMS_rfid: lane {lane_name!r} has no slot "
                f"(set lane_slot_map)")
            return None
        tag = self.read_slot(slot)
        if not tag or not tag.get("filament"):
            # A tag may still have been seen (e.g. missing decode key): record
            # its UID/type so get_status shows what was detected.
            if tag:
                self.record_tag_read(lane_name, None, decoded=False,
                                     uid=tag.get("uid", "") or "",
                                     tag_type=tag.get("tag_type", "") or "")
            return None
        lane = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane = self.afc.lanes.get(lane_name)
        if lane is None:
            slot_info = self._map(tag)
            self.record_tag_read(lane_name, slot_info)
            return slot_info
        return self.apply_to_lane(lane, tag)

    def _map(self, tag: dict) -> dict:
        """
        Map a read_tag() result to AFC slot_info (shared rich shape, AFC_RFID).

        :param tag: Raw read_tag() dict.
        :return dict: AFC slot_info (uid, brand, material, color_hex, ...).
        """
        return map_tag_to_slot_info(tag)

    # apply_to_lane() is inherited from AFCUnitRFID (shared AFC_RFID path); it
    # uses self._map, self.log_prefix ("OpenAMS RFID") and self.auto_create.

    def get_status(self, eventtime: Optional[float] = None) -> dict:
        """
        Report the configured reader slots and the lane->slot map.

        :param eventtime: Reactor event time (unused; kept for the status API).
        :return dict: The reader slots and lane_slot_map.
        """
        return {
            "slots": sorted(self._slot_reader.keys()),
            "lane_slot_map": dict(self._lane_slot),
            "last_reads": self.last_reads_status(),
        }

    def cmd_OAMS_RFID_READ(self, gcmd: GCodeCommand) -> None:
        """
        Read an OpenAMS RFID tag and apply it to the lane.

        With SLOT= the raw slot is read and reported without applying it.

        Usage
        -------
        `OAMS_RFID_READ LANE=<name> | SLOT=<n>`

        Example
        -------
        ```
        OAMS_RFID_READ LANE=lane4
        ```
        """
        lane_name = gcmd.get("LANE", None)
        slot = gcmd.get_int("SLOT", None)
        if lane_name is None and slot is None:
            error_str = "OAMS_RFID_READ requires LANE= or SLOT="
            raise gcmd.error(error_str)
        if lane_name is not None:
            si = self.read_lane(lane_name)
            if si is None:
                hint = self.undecoded_hint(lane_name)
                gcmd.respond_info(
                    f"OpenAMS RFID: no tag decoded on {lane_name}{hint}")
            else:
                gcmd.respond_info(
                    "OpenAMS RFID: %s -> %s %s" % (
                        lane_name, si.get("brand", ""), si.get("material", "")))
            return
        tag = self.read_slot(slot)
        gcmd.respond_info(
            "OpenAMS RFID: slot %d -> %s" % (slot, self._map(tag) if tag else None))


def load_config(config: ConfigWrapper) -> AFC_OpenAMS_rfid:
    """
    Klipper entry point for the bare ``[AFC_OpenAMS_rfid]`` coordinator.

    :param config: Klipper config wrapper for the section.
    :return AFC_OpenAMS_rfid: The coordinator object.
    """
    return AFC_OpenAMS_rfid(config)


def load_config_prefix(config: ConfigWrapper) -> AFC_OpenAMS_rfid_reader:
    """
    Klipper entry point for a named ``[AFC_OpenAMS_rfid <name>]`` reader.

    :param config: Klipper config wrapper for the named reader section.
    :return AFC_OpenAMS_rfid_reader: The reader object.
    """
    return AFC_OpenAMS_rfid_reader(config)
