# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# BigTreeTech MMS / ViViD RFID integration for AFC.
#
# The ViViD carries two MFRC522 (MIFARE Classic 1K, 13.56 MHz) readers on a
# shared Klipper SPI bus, told apart by their CS pin; each serves a pair of slots.
# There is no coil-enable GPIO: the RF field is toggled per read through the
# MFRC522 TxControlReg (handled by the shared Mfrc522 class).
#
# This module is transport and wiring only: each reader's MCU_SPI is adapted to
# the reg_read/reg_write contract of the shared reader stack (read_tag), so
# every format it decodes (see AFC_rfid_readers) reads here and syncs to
# Spoolman through AFC_RFID like the ACE2/U1 paths.
from __future__ import annotations
from typing import Any, Callable, Dict, List, Optional, TYPE_CHECKING

# Klipper's core `bus` module is imported lazily in AFC_Vivid_rfid_reader.__init__
# so this module imports without the full Klipper tree (e.g. in unit tests).

# The shared MFRC522 + MIFARE + multi-manufacturer decode stack. Mfrc522 and
# MifareClassic also give a fast UID-only detect (activate) for the stage poll.
from extras.AFC_rfid_readers import read_tag, Mfrc522, MifareClassic
from extras.AFC_rfid_write import register_reader
from extras.AFC_RFID import (format_tag_summary, AFCUnitRFID,
                             map_tag_to_slot_info)
try:
    from extras.AFC_RFID import get_auto_spoolman_create
except ImportError:                                    # older AFC without the helper
    get_auto_spoolman_create = None
try:
    from extras.AFC_RFID import resolve_rfid_keys
except ImportError:                                    # older AFC without the helper
    resolve_rfid_keys = None
try:                                                   # AFC move enums (sister nudge)
    from extras.AFC_lane import SpeedMode, AssistActive, MoveDirection
except ImportError:                                    # older AFC / import-time absence
    SpeedMode = AssistActive = MoveDirection = None
if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC_lane import AFCLane

# MFRC522 SPI: mode 0, address byte (reg << 1) with 0x80 set for a read.
# 5 MHz matches the ViViD firmware default.
_MFRC522_SPI_MODE = 0
_MFRC522_SPI_SPEED = 5000000


class _VividSpiRegLink:
    """Adapt a Klipper ``MCU_SPI`` to the reg_read/reg_write reader contract that
    the shared Mfrc522/MifareClassic/read_tag stack expects.

    MFRC522 SPI framing: write = ``[(reg<<1), val]``; read = ``[0x80|(reg<<1),
    0x00]`` with the value in the second returned byte.
    """

    def __init__(self, spi: Any) -> None:
        """
        Store the Klipper ``MCU_SPI`` used for MFRC522 register access.

        :param spi: The Klipper ``MCU_SPI`` transport for this reader.
        """
        self.spi = spi

    def reg_read(self, reg: int) -> int:
        """
        Read one MFRC522 register over SPI (address byte 0x80|(reg<<1)).

        :param reg: The MFRC522 register index to read.
        :return int: The register value.
        """
        addr = 0x80 | ((reg << 1) & 0x7E)
        params = self.spi.spi_transfer([addr, 0x00])
        resp = bytearray(params['response'])
        return resp[1] if len(resp) > 1 else 0

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one MFRC522 register over SPI (address byte (reg<<1)).

        :param reg: The MFRC522 register index to write.
        :param val: The byte value to store.
        """
        self.spi.spi_send([(reg << 1) & 0x7E, val & 0xFF])

    def reader_power(self, on: bool) -> None:
        """
        No-op: the ViViD has no reader-power line; the RF field is driven
        through the MFRC522 TxControlReg by the Mfrc522 class.

        :param on: Requested power state (ignored).
        """
        return None


class AFC_Vivid_rfid_reader:
    """One MFRC522 reader on the ViViD, wrapping a Klipper ``MCU_SPI``.

    Configured as ``[AFC_Vivid_rfid <name>]`` with ``cs_pin`` (+ ``spi_bus`` or
    software-SPI pins) and the ``slots`` it serves. Two readers share one SPI bus
    and are told apart only by their CS pin.
    """

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Build the MFRC522 SPI transport and parse the slots this reader serves.

        :param config: Klipper config wrapper for the named reader section.
        """
        self.printer = config.get_printer()
        # Section is "AFC_Vivid_rfid <name>"; keep the trailing name.
        self.name = config.get_name().split()[-1]
        # Take AFC's logger at construction; load_object builds AFC if needed.
        self.logger = self.printer.load_object(config, "AFC").logger
        # Build the SPI at config time: mode 0, active-low CS, 5 MHz. `bus`
        # (klippy/bus.py) is imported here so the module loads without it in tests.
        try:
            from . import bus            # hardware: extras.bus
        except (ImportError, ValueError):
            import bus                   # top-level import fallback (tests)
        self.spi = bus.MCU_SPI_from_config(
            config, _MFRC522_SPI_MODE, pin_option="cs_pin",
            default_speed=_MFRC522_SPI_SPEED, cs_active_high=False)
        self.link = _VividSpiRegLink(self.spi)
        # Physical slots this reader covers (the selector positions one at a time).
        self.slots: List[int] = []
        for s in (config.get("slots", "") or "").split(","):
            s = s.strip()
            if not s:
                continue
            try:
                self.slots.append(int(s))
            except ValueError:
                error_str = f"AFC_Vivid_rfid {self.name}: bad slot number {s!r} in 'slots'"
                raise config.error(error_str)


class AFC_Vivid_rfid(AFCUnitRFID):
    """Coordinator for the ViViD RFID readers. Configured as ``[AFC_Vivid_rfid]``.

    Maps AFC lanes to physical slots, resolves each slot to its reader, and reads
    a lane's tag through the shared multi-manufacturer ``read_tag`` stack, syncing
    the result to the lane + Spoolman.

    A tag is only in range while the spool spins during a feed, and each
    antenna sees both of its slots. So on afc_vivid:stage_read_begin/end this
    module polls the reader on a reactor timer during the unit's normal load
    feed, with a shared-antenna sibling dedup; the first confirmed read wins.
    ``VIVID_RFID_READ`` also does an on-demand read.
    """

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Parse the coordinator config, build the lane/slot maps and register the
        stage-read event handlers and the ``VIVID_RFID_READ`` command.

        :param config: Klipper config wrapper for the ``[AFC_Vivid_rfid]`` section.
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        # Take AFC's logger at construction; load_object builds AFC if needed.
        # self.afc stays None until klippy:ready: guards use it as the ready marker.
        self.logger = self.printer.load_object(config, "AFC").logger

        # Brand keys (optional): Bambu master key + Creality CFS keys enable those
        # decoders; every other format needs no config key.
        bmk = (config.get("bambu_master_key", "") or "").strip()
        self.bambu_master_key = bytes.fromhex(bmk) if bmk else None
        ck = (config.get("creality_key", "") or "").strip()
        cek = (config.get("creality_encryption_key", "") or "").strip()
        self.creality_key = bytes.fromhex(ck) if ck else None
        self.creality_encryption_key = bytes.fromhex(cek) if cek else None
        self.auto_create = config.getboolean("auto_spoolman_create", False)
        self.log_prefix = "ViViD RFID"         # AFCUnitRFID.apply_to_lane / Spoolman

        # Stage-read: poll the reader on a reactor timer while the unit's normal
        # load feed spins the spool. The first confirmed read wins.
        self.stage_read = config.getboolean("stage_read", True)
        # Seconds between reader polls during the feed. The tag is in the antenna's
        # arc only briefly per revolution, so poll often enough to catch a pass.
        self.stage_poll_interval = config.getfloat(
            "stage_poll_interval", 0.15, minval=0.02)
        # Require the same decoded UID on this many consecutive in-place reads,
        # so a stray read cannot match or create the wrong spool.
        self.stage_confirm_reads = config.getint(
            "stage_confirm_reads", 2, minval=1)
        # Max feed aborts before giving up on an undecodable tag.
        self.stage_max_aborts = config.getint("stage_max_aborts", 3, minval=1)
        # Sister-retract: like the ACE2, retract an idle shared-antenna sibling
        # off the antenna for the read, then feed it back (default 75 mm).
        self.auto_tag_adjust = config.getboolean("auto_tag_adjust", True)
        self.auto_tag_adjust_dist = config.getfloat(
            "auto_tag_adjust_dist", 75.0, minval=1.0)

        # lane_slot_map: "lane0:0, lane1:1, lane2:2, lane3:3" -> {lane: slot}
        self._lane_slot: Dict[str, int] = {}
        for pair in (config.get("lane_slot_map", "") or "").split(","):
            pair = pair.strip()
            if not pair:
                continue
            name, sep, slot = pair.partition(":")
            if not sep:
                error_str = ("AFC_Vivid_rfid: 'lane_slot_map' entries must be "
                             f"'lane:slot', got {pair!r}")
                raise config.error(error_str)
            try:
                self._lane_slot[name.strip()] = int(slot.strip())
            except ValueError:
                error_str = f"AFC_Vivid_rfid: bad slot number in {pair!r}"
                raise config.error(error_str)

        self.afc: Any = None
        self._slot_reader: Dict[int, AFC_Vivid_rfid_reader] = {}
        self._slot_lane: Dict[int, str] = {}           # slot -> lane (reverse map)
        self._last: Dict[int, dict] = {}               # slot -> last slot_info
        # Last decoded UID per physical slot, used to halt a shared-antenna
        # sibling's parked tag so a read returns this slot's own tag.
        self._slot_uid: Dict[int, str] = {}
        self._probe: Optional[dict] = None             # active stage-read state
        self._poll_timer: Any = None                   # reactor timer during feed

        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler(
            "afc_vivid:stage_read_begin", self._stage_read_begin)
        self.printer.register_event_handler(
            "afc_vivid:stage_read_end", self._stage_read_end)
        self.gcode.register_command(
            "VIVID_RFID_READ", self.cmd_VIVID_RFID_READ,
            desc="Read the ViViD RFID tag for a lane (LANE=) or slot (SLOT=) and "
                 "apply it to the lane / Spoolman")

    # ── wiring ──────────────────────
    def _on_ready(self) -> None:
        """
        Resolve the AFC object and shared brand keys, then discover every reader
        section and index it by physical slot (klippy:ready handler).
        """
        self.afc = self.printer.lookup_object("AFC", None)
        # Fall back to the shared [AFC_rfid_keys] for any key not set here.
        if resolve_rfid_keys is not None:
            (self.bambu_master_key, self.creality_key,
             self.creality_encryption_key) = resolve_rfid_keys(
                self.printer, self.bambu_master_key, self.creality_key,
                self.creality_encryption_key)
        # Discover every [AFC_Vivid_rfid <name>] reader and index by physical slot.
        self._slot_reader = {}
        for name, obj in self.printer.lookup_objects():
            if (name.startswith("AFC_Vivid_rfid ")
                and isinstance(obj, AFC_Vivid_rfid_reader)):
                for slot in obj.slots:
                    if slot in self._slot_reader:
                        self.logger.warning(
                            f"AFC_Vivid_rfid: slot {slot} served by more than one reader; keeping "
                            f"{self._slot_reader[slot].name}")
                        continue
                    self._slot_reader[slot] = obj
        # Offer every reader to the shared tag writer (AFC_RFID_WRITE). The SPI
        # link exists from config, so it is always available.
        for rdr in dict.fromkeys(self._slot_reader.values()):
            register_reader(
                self.printer, f"vivid:{rdr.name}",
                f"ViViD {rdr.name} (slots "
                f"{', '.join(str(s) for s in rdr.slots)})",
                self, lambda r=rdr: r.link,
                serves=lambda ln, r=rdr: self._get_slot(ln) in r.slots)
        # Reverse map: slot -> lane, for sibling-present checks during dedup.
        self._slot_lane = {slot: name for name, slot in self._lane_slot.items()}
        if not self._slot_reader:
            self.logger.warning(
                "AFC_Vivid_rfid: no [AFC_Vivid_rfid <name>] reader sections "
                "found, RFID reads disabled")
        else:
            readers = len({id(r) for r in self._slot_reader.values()})
            self.logger.info(
                f"AFC_Vivid_rfid: {len(self._slot_reader)} slot(s) mapped "
                f"across {readers} reader(s)")

    def _get_slot(self, lane_name: str) -> Optional[int]:
        """
        Map a lane name to its configured physical slot.

        :param lane_name: The AFC lane name.
        :return Optional[int]: The mapped slot, or None if the lane is unmapped.
        """
        return self._lane_slot.get(lane_name)

    def _sibling_slot(self, slot: int) -> Optional[int]:
        """
        Return the other slot served by the same reader (shared antenna).

        :param slot: The physical slot to find the sibling of.
        :return Optional[int]: The sibling slot, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        others = [s for s in reader.slots if s != slot]
        return others[0] if len(others) == 1 else None

    def _sibling_has_spool(self, sib: int) -> bool:
        """
        Whether the sibling slot still has filament present (so its last-known
        tag is worth halting). Unknown -> assume present (safer to dedup).

        :param sib: The sibling physical slot.
        :return bool: True if the sibling likely still holds a spool.
        """
        lane_name = self._slot_lane.get(sib)
        if not lane_name or self.afc is None or not hasattr(self.afc, "lanes"):
            return True
        lane = self.afc.lanes.get(lane_name)
        if lane is None:
            return True
        return bool(getattr(lane, "prep_state", True))

    def _sibling_excluder(self, slot: int) -> Optional[Callable[[str], bool]]:
        """
        Build a ``uid_hex -> bool`` predicate that halts the shared-antenna
        sibling's last-known tag, so a read returns this slot's own tag. None
        when the sibling has no known tag or its spool has been removed.

        :param slot: The physical slot being read.
        :return Optional[Callable[[str], bool]]: The excluder predicate, or None.
        """
        sib = self._sibling_slot(slot)
        if sib is None:
            return None
        sib_uid = self._slot_uid.get(sib)
        if not sib_uid or not self._sibling_has_spool(sib):
            return None
        sib_uid = sib_uid.lower()

        def _excluded(uid_hex: str) -> bool:
            """
            Whether a UID is the sibling's parked tag.

            :param uid_hex: UID as hex
            :return bool: True to exclude it
            """
            return (uid_hex or "").lower() == sib_uid
        return _excluded

    def _excluder_with(
            self, slot: int, extra: Optional[str] = None
    ) -> Optional[Callable[[str], bool]]:
        """
        Combine the sibling excluder with an optional extra UID to halt (a
        parked sibling tag captured at the stage baseline). Either may be None.

        :param slot: The physical slot being read.
        :param extra: An extra UID hex to also halt, or None.
        :return Optional[Callable[[str], bool]]: The combined predicate, or None.
        """
        base = self._sibling_excluder(slot)
        if not extra:
            return base
        ex = extra.lower()

        def _c(uid_hex: str) -> bool:
            """
            Combined excluder: this UID or whatever the base excluder rejects.

            :param uid_hex: UID as hex
            :return bool: True to exclude it
            """
            u = (uid_hex or "").lower()
            return u == ex or (base is not None and base(u))
        return _c

    def _parked_tag(self, slot: int) -> Optional[str]:
        """
        Raw tag probe without an excluder. Called before the spool spins, so
        anything read is a parked tag, usually the stationary sibling's.

        :param slot: The physical slot to probe.
        :return Optional[str]: The parked tag UID hex, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        try:
            uid, _sak = MifareClassic(Mfrc522(reader.link)).activate()
        except Exception:
            return None
        return uid.hex() if uid is not None else None

    # ── read ──────────────────────
    def read_slot(
            self, slot: int, extra_excluded: Optional[str] = None
    ) -> Optional[dict]:
        """
        Read the tag on a physical slot via its reader; return the raw tag dict
        (uid + decoded filament) or None.

        The sibling excluder halts the neighbour slot's known tag on the shared
        antenna. A successful read's UID is remembered for the sibling's dedup.

        :param slot: The physical slot to read.
        :param extra_excluded: An extra parked-sibling UID hex to halt, or None.
        :return Optional[dict]: The raw tag dict, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            self.logger.warning(f"AFC_Vivid_rfid: no reader configured for slot {slot}")
            return None
        tag = read_tag(
            reader.link, bambu_master_key=self.bambu_master_key,
            creality_key=self.creality_key,
            creality_encryption_key=self.creality_encryption_key,
            is_excluded=self._excluder_with(slot, extra_excluded))
        uid = (tag or {}).get("uid")
        if uid:
            self._slot_uid[slot] = uid
        return tag

    def _map(self, tag: dict) -> dict:
        """
        Convert a read_tag() result to AFC slot_info (shared rich shape, AFC_RFID).

        :param tag: The raw tag dict from read_tag().
        :return dict: The AFC slot_info dict.
        """
        return map_tag_to_slot_info(tag)

    # apply_to_lane() is inherited from AFCUnitRFID (shared AFC_RFID path); it
    # uses self._map, self.log_prefix ("ViViD RFID") and self.auto_create.

    def read_lane(self, lane_name: str) -> Optional[dict]:
        """
        Read a lane's tag (by its mapped slot) and apply it. The caller must have
        the lane's slot selected at the reader.

        :param lane_name: The AFC lane name to read.
        :return Optional[dict]: The applied slot_info, or None.
        """
        slot = self._get_slot(lane_name)
        if slot is None:
            self.logger.warning(
                f"AFC_Vivid_rfid: lane {lane_name!r} has no slot (set lane_slot_map)")
            return None
        tag = self.read_slot(slot)
        if not tag or not tag.get("filament"):
            # A tag may still have been seen (e.g. Bambu without a decode key):
            # record its UID/type so get_status shows what was detected.
            if tag:
                self.record_tag_read(lane_name, None, decoded=False,
                                     uid=tag.get("uid", "") or "",
                                     tag_type=tag.get("tag_type", "") or "")
            return None
        self._last[slot] = tag
        lane = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane = self.afc.lanes.get(lane_name)
        if lane is None:
            slot_info = self._map(tag)
            self.record_tag_read(lane_name, slot_info)
            return slot_info
        return self.apply_to_lane(lane, tag)

    # ── stage-read (poll the reader while the unit's load feed spins the spool) ──
    def _stage_read_begin(self, lane: "AFCLane") -> None:
        """
        Start polling the reader on a reactor timer as the unit's load feed
        begins for this lane. The feed itself is not touched.

        :param lane: The AFC lane whose load feed is starting.
        """
        self._cancel_poll()
        if not self.stage_read:
            return
        lane_name = getattr(lane, "name", None)
        slot = self._get_slot(lane_name) if lane_name else None
        if slot is None or slot not in self._slot_reader:
            return                                     # not a lane we read
        self._probe = {"slot": slot, "lane": lane_name, "uid": None,
                       "count": 0, "done": False, "sib_lane": None,
                       "sib_dist": 0.0, "baseline": None, "blocked_sib": None,
                       "read_ok": False}
        # Probe once before the spool spins: a tag read now is parked on the
        # shared antenna. Retract a movable idle sibling, or else exclude that UID
        # for the whole read (dedup alone misses a sibling with no recorded UID).
        try:
            sib = self._sibling_slot(slot)
            sib_name = self._slot_lane.get(sib) if sib is not None else None
            sib_lane = (self.afc.lanes.get(sib_name)
                        if (sib_name
                            and self.afc is not None
                            and hasattr(self.afc, "lanes"))
                        else None)
            if sib_lane is not None and getattr(sib_lane, "prep_state", False):
                parked = self._parked_tag(slot)
                # The parked tag may be this slot's own, so compare against
                # _slot_uid (as ACE2's _is_sibling_tag does). An unknown UID
                # stays a collision on purpose.
                mine = (self._slot_uid.get(slot) or "").lower()
                if parked and mine and parked.lower() == mine:
                    self.logger.info(
                        f"ViViD RFID: tag {parked} on the shared reader for slot {slot} is this "
                        f"slot's own, no collision, leaving the sibling alone")
                    parked = None
                if parked:
                    self.logger.info(
                        f"ViViD RFID: tag {parked} parked on the shared reader for slot {slot}, "
                        f"clearing the sibling before the read")
                    self._retract_sibling(self._probe)
                    if self._probe.get("sib_lane") is None:
                        # Could not move it: exclude its UID for the whole read
                        # (see _stage_read_end for the empty-read hint).
                        self._probe["baseline"] = parked
                        self._probe["blocked_sib"] = sib_name
        except Exception as e:
            self.logger.warning(f"ViViD RFID: sibling pre-check failed: {e}")
        try:
            self._poll_timer = self.reactor.register_timer(
                self._stage_poll,
                self.reactor.monotonic() + self.stage_poll_interval)
        except Exception as e:
            self.logger.warning(f"ViViD RFID: could not start stage poll: {e}")
            self._restore_sibling(self._probe)
            self._probe = None

    def _stage_poll(self, eventtime: float) -> float:
        """
        Reactor-timer tick during the load feed.

        Runs a cheap UID detect; once a tag is in range, stops the feed so the
        spool parks with the tag held there, then does the full read and confirm.

        :param eventtime: The reactor event time for this tick.
        :return float: The next wake time, or NEVER once done/torn down.
        """
        p = self._probe
        if p is None or p.get("done"):
            return self.reactor.NEVER
        try:
            uid = self._detect_uid(p["slot"])
        except Exception as e:
            self.logger.warning(f"ViViD RFID: detect error on slot {p['slot']}: {e}")
            uid = None
        if not uid:
            return eventtime + self.stage_poll_interval   # nothing in range yet
        if uid == p.get("baseline"):
            # The parked sibling tag that could not be moved: keep polling.
            return eventtime + self.stage_poll_interval

        # Any sibling was cleared up front, so this is this lane's tag. Abort the
        # feed so the spool parks with the tag in range, then read in place.
        self._abort_feed(p["lane"])
        p["aborts"] = p.get("aborts", 0) + 1
        tag = self._read_confirmed(p["slot"], p.get("baseline"))
        if tag:
            p["done"] = True
            p["read_ok"] = True
            self._apply_staged(p, tag)
            return self.reactor.NEVER
        # Detected but not decoded: give up after stage_max_aborts attempts.
        if p["aborts"] >= self.stage_max_aborts:
            p["done"] = True
            self.logger.info(
                f"ViViD RFID: gave up reading {p['lane']} after {p['aborts']} attempts")
            return self.reactor.NEVER
        return eventtime + self.stage_poll_interval

    def _detect_uid(self, slot: int) -> Optional[str]:
        """
        Fast tag presence check: activate only (REQA/anticoll/select), with the
        sibling excluder applied so a halted neighbour tag does not count.

        :param slot: The physical slot to detect on.
        :return Optional[str]: The detected UID hex, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        mc = MifareClassic(Mfrc522(reader.link))
        uid, _sak = mc.activate(is_excluded=self._sibling_excluder(slot))
        return uid.hex() if uid is not None else None

    def _read_confirmed(
            self, slot: int, baseline: Optional[str] = None
    ) -> Optional[dict]:
        """
        Read the parked tag stage_confirm_reads times and require a consistent
        decoded UID. ``baseline`` (an unmovable parked sibling) is halted.

        :param slot: The physical slot to read.
        :param baseline: A parked-sibling UID hex to halt, or None.
        :return Optional[dict]: The confirmed tag dict, or None.
        """
        tag = None
        for _ in range(self.stage_confirm_reads):
            try:
                t = self.read_slot(slot, extra_excluded=baseline)
            except Exception as e:
                self.logger.warning(f"ViViD RFID: stage read error on slot {slot}: {e}")
                return None
            if not t or not t.get("filament"):
                return None
            if tag is not None and (t.get("uid") or "") != (tag.get("uid") or ""):
                return None                            # inconsistent: reject
            tag = t
        return tag

    def _abort_feed(self, lane_name: str) -> None:
        """
        Stop the in-progress load feed by faking the load-sensor trigger.

        Force-completes the endstop's MCU trsync so the homing move halts; the
        real sensor is untriggered, so the unit re-issues the feed. Best-effort:
        without the trsync the feed just runs to the real sensor.

        :param lane_name: The AFC lane whose load feed to abort.
        """
        lane = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane = self.afc.lanes.get(lane_name)
        endstop_name = getattr(lane, "load_endstop_name", None) if lane else None
        if not endstop_name:
            return
        try:
            qe = self.printer.lookup_object("query_endstops", None)
            mcu_endstop = None
            for es, name in (getattr(qe, "endstops", []) or []):
                if name == endstop_name:
                    mcu_endstop = es
                    break
            if mcu_endstop is None:
                return
            dispatch = getattr(mcu_endstop, "_dispatch", None)
            trsyncs = getattr(dispatch, "_trsyncs", None) if dispatch else None
            if not trsyncs:
                return
            trsync = trsyncs[0]
            trsync._trsync_trigger_cmd.send(
                [trsync._oid, trsync.REASON_HOST_REQUEST])
        except Exception as e:
            self.logger.debug(
                f"ViViD RFID: fake-trigger of {endstop_name} not "
                f"available ({e}), feeding to the "
                f"real sensor instead")

    def _apply_staged(self, p: dict, tag: dict) -> None:
        """
        Cache and apply a staged read to its lane.

        :param p: The active probe state.
        :param tag: The confirmed tag dict to apply.
        """
        self._last[p["slot"]] = tag
        lane_obj = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane_obj = self.afc.lanes.get(p["lane"])
        try:
            if lane_obj is not None:
                self.apply_to_lane(lane_obj, tag)
        except Exception as e:
            self.logger.warning(f"ViViD RFID: applying {p['lane']} read failed: {e}")

    def _retract_sibling(self, p: dict) -> None:
        """
        Move the shared-antenna sibling off the reader before the read.

        Mirrors the ACE2's auto_tag_adjust. Only an idle, present sibling moves,
        never tool-loaded or mid-print. The distance is recorded on ``p`` for
        _restore_sibling.

        :param p: The active probe state; sib_lane/sib_dist are filled if moved.
        """
        if not self.auto_tag_adjust or MoveDirection is None:
            return
        sib = self._sibling_slot(p["slot"])
        sib_lane_name = self._slot_lane.get(sib) if sib is not None else None
        sib_lane = None
        if sib_lane_name and self.afc is not None and hasattr(self.afc, "lanes"):
            sib_lane = self.afc.lanes.get(sib_lane_name)
        if sib_lane is None:
            return
        # Nothing to move if the sibling is empty; refuse to move a loaded or
        # printing sibling (moving loaded/printing filament is a hard no).
        if not getattr(sib_lane, "prep_state", False):
            return
        if getattr(sib_lane, "tool_loaded", False) or self._afc_is_printing():
            self.logger.info(
                f"ViViD RFID: sibling {sib_lane_name} is "
                f"loaded/printing, not moving it; relying "
                f"on the HALT dedup (nudge by hand if the read misses)")
            return
        # Require the load switch, not just prep: otherwise a blind retract can
        # walk a badly staged lane out of itself.
        if not self._seated(sib_lane):
            self.logger.info(
                f"ViViD RFID: sibling {sib_lane_name} is not on its load "
                f"switch, not moving it, since there may be nothing to give "
                f"back")
            return
        want = self.auto_tag_adjust_dist
        self.logger.info(
            f"ViViD RFID: retracting sibling {sib_lane_name} up to {want:.0f}mm to clear the "
            f"antenna before reading slot {p['slot']}")
        # Stepped so the distance is a maximum: stop before the sibling would
        # come off its load switch.
        step = getattr(sib_lane, "short_move_dis", None) or 10.0
        moved = 0.0
        while moved < want and self._seated(sib_lane):
            hop = min(step, want - moved)
            # Assist on for the retract so the spooler winds the filament back.
            sib_lane.move_to(hop * MoveDirection.NEG, SpeedMode.SHORT,
                             assist_active=AssistActive.YES, use_homing=False)
            moved += hop
            # Recorded per hop, so a later hop that raises still leaves the
            # restore knowing how far the sibling went.
            p["sib_lane"], p["sib_dist"] = sib_lane, moved
        if moved <= 0.0:
            return
        if moved < want:
            self.logger.info(
                f"ViViD RFID: sibling {sib_lane_name} stopped early at "
                f"{moved:.0f}mm, its load switch was about to release")

    def _homing_available(self, lane: Any) -> bool:
        """
        Whether AFC's own homing moves can be used on this lane.

        :param lane: the AFC lane
        :return bool: True when the enums, move_to, the endstop and AFC's
                      homing setting are all present
        """
        return (SpeedMode is not None
                and MoveDirection is not None
                and AssistActive is not None
                and callable(getattr(lane, "move_to", None))
                and getattr(lane, "load_es", None) is not None
                and bool(getattr(self.afc, "homing_enabled", False)))

    def _seated(self, lane: Any) -> bool:
        """
        Whether filament still covers the lane's LOAD switch.

        Prefers the raw reading where the lane exposes one: the debounced
        value can lag a switch that has just released, and this is used to
        decide whether it is safe to keep retracting.

        :param lane: the AFC lane
        :return bool: True while the load switch reads filament
        """
        raw = getattr(lane, "raw_load_state", None)
        if raw is not None:
            return bool(raw)
        return bool(getattr(lane, "load_state", False))

    def _restore_sibling(self, p: Optional[dict]) -> None:
        """
        Put the sibling back on the load switch it started from.

        Gear slip makes a blind give-back under-deliver, so after it this homes
        onto the load switch (or creeps forward without homing). Slip on a
        sibling that never left its switch is not corrected; one that ends up
        off its switch is logged as an error.

        :param p: The active probe state, or None.
        """
        if not p:
            return
        sib_lane, dist = p.get("sib_lane"), p.get("sib_dist") or 0.0
        if sib_lane is None or dist <= 0.0 or MoveDirection is None:
            return
        p["sib_lane"], p["sib_dist"] = None, 0.0       # once
        try:
            # Plain move: a homing move would stop at once on a switch that
            # never released.
            sib_lane.move_to(dist * MoveDirection.POS, SpeedMode.SHORT,
                             assist_active=AssistActive.NO, use_homing=False)
            step = getattr(sib_lane, "short_move_dis", None) or 10.0
            unit = getattr(sib_lane, "unit_obj", None)
            if self._seated(sib_lane):
                pass                      # already back on it, nothing to do
            elif (self._homing_available(sib_lane)
                  and callable(getattr(unit, "move_to_load", None))):
                # The switch released, so home forward onto it as prep_load does.
                unit.move_to_load(sib_lane, dist + 2.0 * step,
                                  MoveDirection.POS)
                if self._seated(sib_lane):
                    self.logger.info(
                        f"ViViD RFID: homed sibling {sib_lane.name} forward "
                        f"onto its load switch after the give-back landed "
                        f"short")
                else:
                    self.logger.warning(
                        f"ViViD RFID: homing sibling {sib_lane.name} forward "
                        f"after the give-back landed short did not reach its "
                        f"load switch")
            else:
                crept = 0.0
                for _ in range(int(dist / step) + 4):
                    if self._seated(sib_lane):
                        break
                    sib_lane.move_to(step * MoveDirection.POS, SpeedMode.SHORT,
                                     assist_active=AssistActive.NO,
                                     use_homing=False)
                    crept += step
                if crept:
                    self.logger.info(
                        f"ViViD RFID: fed sibling {sib_lane.name} a further "
                        f"{crept:.0f}mm to put it back on its load switch")
            if not self._seated(sib_lane):
                self.logger.error(
                    f"ViViD RFID: sibling {sib_lane.name} is NOT back on its "
                    f"load switch after the read; its filament may have come "
                    f"out of the lane. Re-seat it and check the spool.")
        except Exception as e:
            self.logger.error(f"ViViD RFID: FAILED to restore sibling by {dist:.0f}mm: {e}")

    def _afc_is_printing(self) -> bool:
        """
        Best-effort 'are we printing' guard: never move a sibling mid-print.

        :return bool: True if the printer reports it is printing.
        """
        try:
            ps = self.printer.lookup_object("print_stats", None)
            if ps is not None:
                st = ps.get_status(self.reactor.monotonic())
                return st.get("state") == "printing"
        except Exception:
            pass
        return False

    def _sister_blocked_hint(
            self, active_name: Optional[str], sib_name: Optional[str]
    ) -> None:
        """
        Tell the user an unmovable sibling parked on the shared reader likely
        blocked the stage read, and to move it and re-read or set the id by hand.

        :param active_name: The lane we were trying to read, or None.
        :param sib_name: The blocking sibling lane name, or None.
        """
        act = active_name or "this lane"
        self.gcode.respond_info(
            f"ViViD RFID: couldn't read {act}'s RFID, lane {sib_name}'s spool is on the "
            f"shared reader blocking it. Manually move lane {sib_name}'s spool and re-run "
            f"VIVID_RFID_READ, or set {act}'s spool id by hand.")

    def _stage_read_end(self, lane: "AFCLane") -> None:
        """
        Stop polling when the feed finishes and restore a moved sibling.

        If no tag was decoded and an unmovable sibling was blocking the reader,
        tell the user; otherwise it is just a tagless or foreign spool.

        :param lane: The AFC lane whose load feed finished.
        """
        p = self._probe
        if p is not None and not p.get("read_ok"):
            self.logger.info(f"ViViD RFID: no tag decoded on {p.get('lane')} during staging")
            if p.get("blocked_sib"):
                self._sister_blocked_hint(p.get("lane"), p.get("blocked_sib"))
        self._restore_sibling(p)
        self._cancel_poll()

    def _cancel_poll(self) -> None:
        """
        Tear down the poll timer + probe state, restoring a still-retracted
        sibling first so it's never left short (defensive for aborted reads).
        """
        self._restore_sibling(self._probe)
        if self._poll_timer is not None:
            try:
                self.reactor.unregister_timer(self._poll_timer)
            except Exception:
                pass
            self._poll_timer = None
        self._probe = None

    # ── gcode ──────────────────────
    def cmd_VIVID_RFID_READ(self, gcmd: GCodeCommand) -> None:
        """
        Read a ViViD RFID tag for a lane or slot and apply it to the lane/Spoolman.

        Usage
        -------
        `VIVID_RFID_READ LANE=<name> | SLOT=<n>`

        Example
        -------
        ```
        VIVID_RFID_READ LANE=lane0
        ```
        """
        lane_name = gcmd.get("LANE", None)
        slot = gcmd.get_int("SLOT", None)
        if lane_name is None and slot is None:
            error_str = "VIVID_RFID_READ requires LANE= or SLOT="
            raise gcmd.error(error_str)
        if lane_name is not None:
            # read_lane -> apply_to_lane already prints the console read-out.
            si = self.read_lane(lane_name)
            if si is None:
                hint = self.undecoded_hint(lane_name)
                gcmd.respond_info(f"ViViD RFID: no tag decoded on {lane_name}{hint}")
            return
        tag = self.read_slot(slot)
        if not tag or not tag.get("filament"):
            hint = ""
            if tag and tag.get("uid"):
                hint = f" (saw tag UID {tag['uid']}"
                if tag.get("tag_type"):
                    hint += f", {tag['tag_type']}"
                hint += ", no decoder/key matched)"
            gcmd.respond_info(f"ViViD RFID: no tag decoded on slot {slot}{hint}")
            return
        self._respond_tag(gcmd, self._map(tag), "slot %d" % slot)

    def _respond_tag(self, gcmd: GCodeCommand, si: dict, where: str) -> None:
        """
        Echo a decoded tag to the console using the shared U1-style scan format
        (header + indented Name/Brand/Material/Color/temp lines), so every RFID
        surface reads the same. Colour is hex only ("#hex + #hex" for dual).

        :param gcmd: The gcode command to respond on.
        :param si: The AFC slot_info dict.
        :param where: A label for the source (lane or slot).
        """
        gcmd.respond_info(
            format_tag_summary(si, "ViViD RFID tag on %s:" % where))

    def get_status(self, eventtime: Optional[float] = None) -> dict:
        """
        Report which slots have a reader and the lane->slot map, so the GUI /
        macros can see the ViViD RFID wiring.

        :param eventtime: Reactor event time (unused; kept for the status API).
        :return dict: The configured reader slots and the lane_slot_map.
        """
        return {
            "slots": sorted(self._slot_reader.keys()),
            "lane_slot_map": dict(self._lane_slot),
            "last_reads": self.last_reads_status(),
        }


def load_config(config: "ConfigWrapper") -> AFC_Vivid_rfid:
    """
    Klipper entry point for the bare ``[AFC_Vivid_rfid]`` section, the one
    coordinator that ties the per-reader sections to lanes.

    :param config: Klipper config wrapper for the section.
    :return AFC_Vivid_rfid: The coordinator object.
    """
    return AFC_Vivid_rfid(config)


def load_config_prefix(config: "ConfigWrapper") -> AFC_Vivid_rfid_reader:
    """
    Klipper entry point for a named ``[AFC_Vivid_rfid <name>]`` section: one
    physical MFRC522 reader serving a pair of slots.

    :param config: Klipper config wrapper for the named reader section.
    :return AFC_Vivid_rfid_reader: The reader object.
    """
    return AFC_Vivid_rfid_reader(config)
