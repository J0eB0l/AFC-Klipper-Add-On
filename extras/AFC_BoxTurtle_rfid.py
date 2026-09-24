# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# BoxTurtle lane RFID: read a spool's tag by turning the spool.
#
# Two RC522-family readers share one Pico running the rfid_bridge firmware,
# one reader per pair of lanes:
#
#   [AFC_BoxTurtle_rfid]           coordinator: serial port, keys, knobs
#   [AFC_BoxTurtle_rfid reader0]   bus: 0   lanes: lane8, lane9
#   [AFC_BoxTurtle_rfid reader1]   bus: 1   lanes: lane10, lane11
#
# The Pico is not a Klipper [mcu] but a plain USB-CDC device this module opens
# itself, so an absent or wedged reader only reports offline and can never take
# the printer down.
#
# The firmware is a register wire ("r<bus> <reg>" / "w<bus> <reg> <val>", hex,
# one op per line), so the shared reader stack runs host side unchanged and
# results reach Spoolman through AFC_RFID. Bus 0 is GP4/GP5, bus 1 is GP6/GP7.
# A read takes about a second over USB, so reads run on a worker thread.
#
# The tag sits at one angle on the spool, so reading it means turning the
# spool: feed while polling, stop when the tag answers, put the filament back.
# The sweep is bounded in spool revolutions. A sister lane's parked tag on the
# shared antenna is rolled clear first (see _clear_sibling).
#
# This runs on insert from afc:lane_prep_loaded, after the filament is homed to
# the load switch and before it feeds the hub. AFC_BT_RFID_READ and
# AFC_BT_RFID_STAGE do it on demand.
from __future__ import annotations
import math
import threading
import chelper
from typing import (TYPE_CHECKING, Any, Callable, Dict, List, Optional,
                    Tuple)

# pyserial is only needed when the bridge actually opens a port; the tests
# and any host without a reader Pico must import cleanly without it.
try:                                        # pragma: no cover - import shim
    import serial
except Exception:                           # pragma: no cover
    serial = None

try:                                        # pragma: no cover - import shim
    from extras.AFC_lane import AssistActive, MoveDirection, SpeedMode
except Exception:                           # pragma: no cover
    AssistActive = MoveDirection = SpeedMode = None

from extras.AFC_rfid_readers import read_tag
from extras.AFC_rfid_write import StageError, register_reader
from extras.AFC_RFID import (AFCUnitRFID,
                             map_tag_to_slot_info,
                             resolve_rfid_keys)

if TYPE_CHECKING:
    from configfile import ConfigWrapper

#: VersionReg, the probe register. 0x91/0x92 = genuine MFRC522 v1/v2;
#: clones (FM17522 and friends) answer other non-0x00/0xFF values.
_VERSION_REG = 0x37

#: Per-op reply deadline. A healthy bridge answers in a few ms, so a slow
#: reply is a failed reply.
_OP_TIMEOUT_S = 0.25

#: How often the connect timer retries an absent bridge.
_RETRY_S = 5.0


class _BridgeSerial:
    """The rfid_bridge Pico's USB-CDC port, opened and owned by this module.

    Fail-soft: every method returns a failure value instead of raising, and a
    port error closes the port so the connect timer retries. One lock keeps
    the protocol to one outstanding op across threads.
    """

    def __init__(self, port: str, logger: Any) -> None:
        """
        Store the port path; the port is opened later by connect().

        :param port: the /dev/serial/by-id path of the rfid_bridge Pico
        :param logger: module logger for connect/drop notes
        """
        self.port = port
        self.logger = logger
        self._ser: Optional[Any] = None
        self._lock = threading.Lock()

    def connected(self) -> bool:
        """
        Whether the bridge port is open.

        :return bool: True while the port is open
        """
        return self._ser is not None

    def connect(self) -> bool:
        """
        Open the port if it is not already open. Never raises.

        :return bool: True when the port is open after the attempt
        """
        if serial is None:
            return False
        with self._lock:
            if self._ser is not None:
                return True
            try:
                self._ser = serial.Serial(
                    self.port, 115200, timeout=_OP_TIMEOUT_S,
                    write_timeout=_OP_TIMEOUT_S)
                self.logger.info(
                    f"BT RFID: bridge connected on {self.port}")
                return True
            except Exception:
                self._ser = None
                return False

    def _drop(self) -> None:
        """
        Close the port after an error; the connect timer takes it from here.

        Called with the lock held.
        """
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        if self._ser is not None:
            self.logger.warning(
                f"BT RFID: bridge dropped on {self.port}, retrying in "
                f"the background")
        self._ser = None

    def request(self, line: str) -> Optional[str]:
        """
        Run one register op: send the line, return the "="-reply payload.

        The firmware's JSON events in the stream are skipped. None means the
        op failed (port absent, I2C NAK "!", or timeout).

        :param line: the op, e.g. "r0 37" or "w1 2A 8D"
        :return Optional[str]: reply payload ("" for a write ack, hex for a
            read), or None
        """
        with self._lock:
            if self._ser is None:
                return None
            try:
                self._ser.reset_input_buffer()
                self._ser.write((line + "\n").encode())
                for _ in range(8):          # skip interleaved JSON events
                    resp = self._ser.readline().decode(errors="replace")
                    if not resp:
                        return None         # timeout
                    resp = resp.strip()
                    if resp.startswith("="):
                        return resp[1:]
                    if resp.startswith("!"):
                        return None
                return None
            except Exception:
                self._drop()
                return None


class _SerialRegLink:
    """reg_read/reg_write over the rfid_bridge: the link contract the
    shared Mfrc522 driver expects, with the wire being a serial line."""

    def __init__(self, bridge: _BridgeSerial, bus: int) -> None:
        """
        Bind a register link to one bus of the shared bridge.

        :param bridge: the shared serial bridge
        :param bus: which I2C controller on the Pico (0 or 1)
        """
        self.bridge = bridge
        self.bus = bus

    def reg_read(self, reg: int) -> int:
        """
        Read one register through the bridge.

        :param reg: MFRC522 register index
        :return int: the register value
        """
        resp = self.bridge.request(f"r{self.bus} {reg & 0xFF:02X}")
        if not resp:
            error_str = f"rfid bridge bus {self.bus}: no answer reading " \
                        f"reg 0x{reg:02X}"
            raise OSError(error_str)
        return int(resp, 16)

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one register through the bridge.

        :param reg: MFRC522 register index
        :param val: value to write
        """
        resp = self.bridge.request(
            f"w{self.bus} {reg & 0xFF:02X} {val & 0xFF:02X}")
        if resp is None:
            error_str = f"rfid bridge bus {self.bus}: no answer writing " \
                        f"reg 0x{reg:02X}"
            raise OSError(error_str)


class AFC_BoxTurtle_rfid_reader:
    """One RC522 reader section: which bridge bus it lives on and the lanes
    its antenna covers. Configured as ``[AFC_BoxTurtle_rfid <name>]``."""

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Parse the bus number and the lanes this reader serves.

        No hardware is touched at config time; the coordinator wires the link.

        :param config: Klipper config wrapper for the named reader section
        """
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Take AFC's logger at construction; load_object builds AFC if needed.
        self.logger = self.printer.load_object(config, "AFC").logger
        self.bus = config.getint("bus", 0, minval=0, maxval=1)
        self.link: Optional[_SerialRegLink] = None
        self.lanes: List[str] = []
        for ln in (config.get("lanes", "") or "").split(","):
            ln = ln.strip()
            if ln:
                self.lanes.append(ln)
        if not self.lanes:
            error_str = (f"AFC_BoxTurtle_rfid {self.name}: 'lanes' must name at "
                         f"least one lane")
            raise config.error(error_str)
        # Filled by the coordinator's connect probe; None = offline.
        self.version: Optional[int] = None


class AFC_BoxTurtle_rfid(AFCUnitRFID):
    """Coordinator for the BoxTurtle RFID readers.

    Maps AFC lanes to readers (each reader's fixed antenna covers two adjacent
    lanes), reads a lane's tag through the shared multi-manufacturer
    ``read_tag`` stack, and applies the result to the lane + Spoolman.

    A shared antenna means the other lane's at-rest tag can answer instead of
    the one being read: the partner lane's last-known UID is passed to
    ``read_tag`` as excluded, so the stack halts on the sibling and keeps
    hunting for the lane's own tag.
    """

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Parse the coordinator config and register the g-code commands.

        The serial bridge is opened by a background timer after ready, so a
        missing reader Pico can never fail startup.

        :param config: Klipper config wrapper for ``[AFC_BoxTurtle_rfid]``
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        # Take AFC's logger at construction; load_object builds AFC if needed.
        # self.afc stays None until klippy:ready: guards use it as the ready marker.
        self.logger = self.printer.load_object(config, "AFC").logger
        self.afc: Any = None
        self.log_prefix = "BT RFID"

        self.serial_port = config.get("serial")
        self.bridge = _BridgeSerial(self.serial_port, self.logger)

        # Brand keys: this section wins, [AFC_rfid_keys] fills the gaps.
        bmk = (config.get("bambu_master_key", "") or "").strip()
        ck = (config.get("creality_key", "") or "").strip()
        cek = (config.get("creality_encryption_key", "") or "").strip()
        self.bambu_master_key = bytes.fromhex(bmk) if bmk else None
        self.creality_key = bytes.fromhex(ck) if ck else None
        self.creality_encryption_key = bytes.fromhex(cek) if cek else None
        self.auto_create = config.getboolean("auto_spoolman_create", False)

        # Tag-homing defaults (all overridable per AFC_BT_RFID_STAGE call). The
        # sweep is bounded in spool revolutions, since the tag passes the reader
        # once per turn (~630mm of filament on a full 200mm spool).
        self.spool_diameter_mm = config.getfloat(
            "spool_diameter_mm", 200.0, above=0.0)
        self.tag_sweep_revs = config.getfloat(
            "tag_sweep_revs", 2.0, above=0.0)
        # Limited by dwell: the tag must stay in the coil's field for a whole
        # read. 100 is measured (20-100 all read). Too fast and tags are missed,
        # which shows up as lanes needing extra turns, not as an error.
        self.tag_sweep_speed = config.getfloat(
            "tag_sweep_speed", 100.0, above=0.0)
        # Chunk size, the abort granularity. 0 (default) sizes it from the
        # lane's speed and acceleration (see _sweep_step_for).
        self.tag_sweep_step_mm = config.getfloat(
            "tag_sweep_step_mm", 0.0, minval=0.0)
        self.read_timeout_s = config.getfloat(
            "read_timeout_s", 6.0, above=0.0)
        self.retract_after_read = config.getboolean(
            "retract_after_read", True)
        # Speed for the restoring retract; 0 means the lane's long-move speed.
        self.tag_retract_speed = config.getfloat(
            "tag_retract_speed", 0.0, minval=0.0)
        # Scan the tag automatically when a spool is inserted (reader lanes only).
        self.scan_on_insert = config.getboolean("scan_on_insert", True)
        # Roll a sister lane back so its parked tag leaves the shared antenna
        # during the scan, like ACE2's auto_tag_adjust.
        self.sibling_tag_adjust = config.getboolean(
            "sibling_tag_adjust", True)
        # 45mm is measured: the tag clears the coil well before 75mm.
        self.sibling_tag_adjust_dist = config.getfloat(
            "sibling_tag_adjust_dist", 45.0, minval=1.0)
        # Set while a sweep owns the lane. See _refuse_if_busy.
        self._sweeping = False
        # One probe thread at a time; see _connect_timer.
        self._probing = False
        self.printer.register_event_handler(
            "afc:lane_prep_loaded", self._on_lane_prep_loaded)

        self._readers: List[AFC_BoxTurtle_rfid_reader] = []
        self._reader_by_lane: Dict[str, AFC_BoxTurtle_rfid_reader] = {}
        self._last_uid_by_lane: Dict[str, str] = {}
        # ACE2's `baseline`: the parked sibling UID to exclude from a read when
        # the sister could not be moved off the antenna. See _clear_sibling.
        self._baseline_uid: Optional[str] = None

        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)
        self.gcode.register_command(
            "AFC_BT_RFID_READ", self.cmd_AFC_BT_RFID_READ,
            desc="Read the tag on a BoxTurtle lane's reader and apply it. "
                 "AFC_BT_RFID_READ LANE=<lane>")
        self.gcode.register_command(
            "AFC_BT_RFID_STATUS", self.cmd_AFC_BT_RFID_STATUS,
            desc="Report the BoxTurtle RFID readers: bridge link, chip "
                 "versions, lane map, last reads. AFC_BT_RFID_STATUS")
        self.gcode.register_command(
            "AFC_BT_RFID_STAGE", self.cmd_AFC_BT_RFID_STAGE,
            desc="Experiment feed: advance the lane so the spool spins the "
                 "tag past the antenna, poll for a read while it moves, then "
                 "retract. AFC_BT_RFID_STAGE LANE=<lane> [ADVANCE=mm] "
                 "[SPEED=] [TIMEOUT=s] [RETRACT=0|1]")

    def _register_writers(self) -> None:
        """
        Offer every reader to the shared tag writer (AFC_RFID_WRITE).

        The link is fetched per call, so an unplugged bridge shows as offline.
        """
        for rdr in self._readers:
            register_reader(
                self.printer, f"bt:{rdr.name}",
                f"BoxTurtle {rdr.name} (bus {rdr.bus}, "
                f"lanes {', '.join(rdr.lanes)})", self,
                lambda r=rdr: (r.link if r.link is not None
                               and self.bridge.connected() else None),
                threaded=True,
                stage=lambda ln, r=rdr: self._stage_for_write(ln, r),
                unstage=self._unstage_after_write,
                exclude=self._write_excluder,
                serves=lambda ln, r=rdr: ln in r.lanes)

    def _write_excluder(self, token: tuple) -> Optional[Callable[[str], bool]]:
        """
        Return the tags a staged write must pass over: the sister lane's.

        :param token: the token _stage_for_write returned
        :return Optional[Callable[[str], bool]]: uid_hex -> bool, or None when
            no sister tag is known
        """
        lane = token[0]
        rdr = self._reader_by_lane.get(lane.name)
        return self._excluder_for(lane.name, rdr) if rdr is not None else None

    def apply_written_tag(self, lane_name: str, tag: dict) -> None:
        """
        Give a lane the tag AFC_RFID_WRITE / AFC_RFID_ENROLL just wrote to its
        spool, as a scan would, and note its UID for the shared antenna.

        :param lane_name: the lane whose spool carries the tag
        :param tag: the written tag, shaped like a read
        """
        lane = self._lane_obj(lane_name)
        if lane is None:
            error_str = f"{lane_name} is not an AFC lane"
            raise RuntimeError(error_str)
        self._last_uid_by_lane[lane_name] = (tag.get("uid") or "").lower()
        self.apply_to_lane(lane, tag)

    def _stage_for_write(self, lane_name: str,
                         target: Optional[AFC_BoxTurtle_rfid_reader] = None
                         ) -> Optional[tuple]:
        """
        Spin the lane's spool until its tag reaches the antenna and hold it,
        so a write lands on a tag the reader can actually see.

        Uses the read sweep, which stops with the tag parked at the coil. Runs
        on the reactor. Staged like cmd_AFC_BT_RFID_STAGE: a hub-staged lane is
        homed back first and re-staged afterwards, the sister's parked tag is
        rolled off, and a busy hub bounds the sweep. A tool-loaded lane is
        refused.

        :param lane_name: the lane whose spool carries the tag
        :param target: the reader the write was sent to, when known
        :return Optional[tuple]: (lane, fed_mm, staged, sibling token), or None
            when there is nothing to spin
        :raises StageError: when the filament must not be moved for the write
        """
        lane = self._lane_obj(lane_name)
        rdr = self._reader_by_lane.get(lane_name)
        if target is not None and rdr is not None and rdr is not target:
            error_str = (f"{lane_name} is not on {target.name}; write it "
                         f"with READER=bt:{rdr.name}")
            raise StageError(error_str)
        if lane is None or rdr is None or rdr.version is None:
            self.logger.info(
                f"AFC_BT_RFID: stage-for-write skipped {lane_name} "
                f"(lane={lane is not None}, rdr={rdr is not None}, "
                f"ver={None if rdr is None else rdr.version})")
            return None
        if self._sweeping:
            self.logger.info(
                f"AFC_BT_RFID: stage-for-write busy, skipping {lane_name}")
            return None
        if getattr(lane, "tool_loaded", False):
            error_str = (f"{lane_name} is loaded in the toolhead. The write "
                         f"turns the spool by moving its filament, so unload "
                         f"it first.")
            raise StageError(error_str)
        staged = bool(getattr(lane, "loaded_to_hub", False))
        if staged and not self._homing_available(lane):
            error_str = (f"{lane_name} is staged at the hub and homing is "
                         f"off, so there is no load switch to bring it back "
                         f"to. Eject it and insert it again to write it.")
            raise StageError(error_str)
        advance = math.pi * self.spool_diameter_mm * self.tag_sweep_revs
        # Room is judged from the load switch, where the sweep will start.
        advance, cut = self._sweep_room(lane, advance, staged=False)
        if advance <= 0.0:
            error_str = (f"{cut}, and {lane_name}'s dist_hub leaves no room "
                         f"before it. Unload that lane first to write this "
                         f"one.")
            raise StageError(error_str)
        if cut:
            self.logger.info(f"AFC_BT_RFID: stage-for-write limited: {cut}.")
        self._sweeping = True
        step = (self.tag_sweep_step_mm
                or self._sweep_step_for(lane, self.tag_sweep_speed))
        sib = None
        try:
            if staged:
                try:
                    self._unstage_for_scan(lane)
                except RuntimeError as e:
                    raise StageError(str(e))
            sib = self._clear_sibling(lane_name, rdr)
            tag, fed = self._sweep_for_tag(lane, lane_name, rdr, advance,
                                           step, self.tag_sweep_speed)
            settled = 0.0
            if tag is not None:
                settled = self._settle_on_tag(lane, lane_name, rdr,
                                              step + 25.0)
                fed += settled
        except Exception:
            # Not re-staged: where the tip is after a failed move is not
            # known, and its next load re-homes it from the load switch.
            try:
                self._restore_sibling(sib)
            finally:
                self._sweeping = False
            raise
        self.logger.info(
            f"AFC_BT_RFID: staged {lane_name} for write: tag "
            f"{'found ' + str(tag.get('uid')) if tag else 'NOT found'} after "
            f"{fed:.0f}mm (re-centred {settled:.0f}mm); holding for the write.")
        return (lane, fed, staged, sib)

    def _settle_on_tag(self, lane: Any, lane_name: str,
                       rdr: AFC_BoxTurtle_rfid_reader,
                       back_limit: float) -> float:
        """
        Ease a just-swept tag back to mid-field so a write couples.

        The sweep stops on a chunk boundary, up to one step past the tag. Back
        off in short steps until the reader answers again, then keep easing
        while the read holds and step back onto the last good spot. Best
        effort: a tag that never re-reads is left where the search stopped.
        Either way the net can come out behind the load switch, which
        _restore homes back onto after the write.

        :param lane: the AFC lane
        :param lane_name: the lane's name
        :param rdr: the reader watching it
        :param back_limit: the most to retract, mm
        :return float: net mm moved (<= 0, a retract), for the restore
        """
        move = 3.0
        speed = getattr(lane, "short_moves_speed", None) or self.tag_sweep_speed
        moved = 0.0
        # The overshot tag re-enters at the far edge of the field.
        found = self._read_once(lane_name, rdr) is not None
        while (not found
               and -moved < back_limit):
            self._lane_move(lane, -move, speed, assist=True)
            moved -= move
            found = self._read_once(lane_name, rdr) is not None
        if not found:
            return moved
        # A write needs the tag nearer the centre than a read does.
        for _ in range(6):
            self._lane_move(lane, -move, speed, assist=True)
            moved -= move
            if self._read_once(lane_name, rdr) is None:
                self._lane_move(lane, move, speed, assist=True)
                moved += move
                break
        return moved

    def _unstage_after_write(self, token: Optional[tuple]) -> None:
        """
        Put the filament back where _stage_for_write found it, then release the
        sweep guard: the sister lane rolled back, this lane's filament returned
        to its load switch, and re-staged at the hub if it was staged.

        :param token: the (lane, fed, staged, sibling token) _stage_for_write
            returned
        """
        try:
            if not token:
                return
            lane, fed, staged, sib = token
            back = (self.tag_retract_speed
                    or getattr(lane, "long_moves_speed", None)
                    or self.tag_sweep_speed)
            try:
                self._restore_sibling(sib)
            finally:
                self._restore(lane, fed, back)
                if staged:
                    self._restage_after_scan(lane)
        finally:
            self._sweeping = False

    def _handle_ready(self) -> None:
        """
        Collect the reader sections, resolve keys, wire each reader's
        serial link, and start the connect timer.

        No hardware I/O here: the timer owns the port.
        """
        self.afc = self.printer.lookup_object("AFC", None)
        # The bridge is a plain object, so hand it AFC's logger here.
        self.bridge.logger = self.logger
        (self.bambu_master_key, self.creality_key,
         self.creality_encryption_key) = resolve_rfid_keys(
            self.printer, self.bambu_master_key, self.creality_key,
            self.creality_encryption_key)
        self._readers = [
            obj for name, obj in self.printer.lookup_objects("AFC_BoxTurtle_rfid")
            if isinstance(obj, AFC_BoxTurtle_rfid_reader)]
        self._reader_by_lane = {}
        for rdr in self._readers:
            rdr.link = _SerialRegLink(self.bridge, rdr.bus)
            for ln in rdr.lanes:
                self._reader_by_lane[ln] = rdr
        self._register_writers()
        self.reactor.register_timer(
            self._connect_timer, self.reactor.monotonic() + 1.0)

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        Report which reader covers each lane.

        This also lists the module in Moonraker, which is how the BridgeBox
        display learns to offer Scan Tag on these lanes.

        :param eventtime: Reactor event time (unused; kept for the status API).
        :return dict: lane_slot_map (lane -> reader name) and each reader's
            online state.
        """
        return {
            "lane_slot_map": {ln: rdr.name
                              for ln, rdr in self._reader_by_lane.items()},
            "readers": {rdr.name: rdr.version is not None
                        for rdr in self._readers},
        }

    def _probe_all(self) -> Tuple[bool, List[Tuple[Any, bool, Optional[int],
                                                   Optional[Exception]]]]:
        """
        Open the port and probe readers. Blocking, so never on the reactor.

        Returns the results rather than applying them; _apply_probe latches
        them on the reactor, where the scan gates read rdr.version.

        :return tuple: (bridge is connected, [(reader, fresh, version, error),
            ...] probed)
        """
        was = self.bridge.connected()
        if not self.bridge.connect():
            return (False, [])
        fresh = not was
        # Do not probe during a sweep on the same link; the next tick will do.
        if not fresh and self._sweeping:
            return (True, [])
        probed = []
        for rdr in self._readers:
            if not (fresh or rdr.version is None):
                continue                  # answering already: leave it alone
            try:
                ver = rdr.link.reg_read(_VERSION_REG)
                probed.append((rdr, fresh, ver, None))
            except Exception as e:
                probed.append((rdr, fresh, None, e))
        return (True, probed)

    def _probe_worker(self) -> None:
        """
        Run _probe_all off-reactor and hand the answer back to it.
        """
        try:
            thread_name = threading.current_thread().name
            chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
        except Exception:
            pass
        try:
            try:
                ok, probed = self._probe_all()
            except Exception:
                ok, probed = (False, [])
            # Register before clearing the flag, so the next tick cannot start
            # a probe while these results are still in flight.
            self.reactor.register_async_callback(
                lambda et: self._apply_probe(ok, probed))
        finally:
            self._probing = False

    def _apply_probe(self, ok: bool, probed: List[Any]) -> None:
        """
        Latch probe results on the reactor, where the scan gates read them.

        :param ok: whether the bridge is connected
        :param probed: (reader, fresh, version, error) from _probe_all
        """
        if not ok:
            for rdr in self._readers:
                rdr.version = None
            return
        for rdr, fresh, ver, err in probed:
            was = rdr.version
            rdr.version = ver
            # Edge-driven, so an absent reader does not warn every retry.
            if err is not None:
                if fresh or was is not None:
                    self.logger.warning(
                        f"BT RFID {rdr.name}: bridge is up but bus "
                        f"{rdr.bus} has no reader ({err}), check wiring")
            elif fresh or was is None:
                self.logger.info(
                    f"BT RFID {rdr.name}: version reg "
                    f"0x{ver:02X} (bus {rdr.bus}, lanes "
                    f"{', '.join(rdr.lanes)})")

    def _connect_timer(self, eventtime: float) -> float:
        """
        Keep the bridge connected and keep re-probing offline readers.

        Touches no hardware itself: it starts a probe thread and returns. A
        reader already answering is left alone, nothing is probed during a
        sweep, and only one probe runs at a time.

        :param eventtime: the reactor time the timer fired at
        :return float: the next wake time
        """
        if not self._probing:
            self._probing = True
            threading.Thread(target=self._probe_worker, daemon=True,
                             name="afc_bt_rfid_probe").start()
        return eventtime + _RETRY_S

    def _lane_obj(self, lane_name: str) -> Any:
        """
        Resolve an AFC lane object by name, or None.

        :param lane_name: the AFC lane name
        :return Any: the lane object or None
        """
        if self.afc is None:
            return None
        return getattr(self.afc, "lanes", {}).get(lane_name)

    def _excluder_for(self, lane_name: str,
                      rdr: AFC_BoxTurtle_rfid_reader) -> Optional[
                          Callable[[str], bool]]:
        """
        Build the shared-antenna exclusion: the partner lane's last-known UID
        answers "excluded", so the read stack keeps hunting past the sibling.

        :param lane_name: the lane being read
        :param rdr: the reader both lanes share
        :return Optional[Callable[[str], bool]]: uid_hex -> bool, or None when
            no sibling UID is known
        """
        sibling_uids = {
            uid.lower() for ln, uid in self._last_uid_by_lane.items()
            if ln != lane_name and ln in rdr.lanes}
        # Plus the parked tag the probe saw when the sister could not be moved,
        # which has no last-known UID yet.
        if self._baseline_uid:
            sibling_uids.add(self._baseline_uid.lower())
        if not sibling_uids:
            return None
        return lambda uid_hex: uid_hex.lower() in sibling_uids

    def _parked_tag(self, rdr: AFC_BoxTurtle_rfid_reader) -> Optional[str]:
        """
        Raw probe with no excluder: whatever tag is on the antenna right now.

        Run before this lane's spool turns, so anything that answers is a
        parked tag. Worker thread only, like _read_blocking.

        :param rdr: the reader whose antenna both lanes share
        :return Optional[str]: the parked tag's UID hex, or None if the antenna
            is clear
        """
        if rdr.link is None or not self.bridge.connected():
            return None
        try:
            tag = read_tag(
                rdr.link,
                bambu_master_key=self.bambu_master_key,
                creality_key=self.creality_key,
                creality_encryption_key=self.creality_encryption_key)
        except Exception:
            return None                   # a probe that fails is "no collision"
        return (tag or {}).get("uid") or None

    def _read_blocking(self, lane_name: str,
                       rdr: AFC_BoxTurtle_rfid_reader) -> Optional[dict]:
        """
        Run one pass of the shared read stack. Worker thread only.

        A read takes ~1s of serial round-trips; _read_once is the reactor-safe
        wrapper.

        :param lane_name: the lane the read is for (drives sibling exclusion)
        :param rdr: the reader to poll
        :return Optional[dict]: the raw read_tag dict, or None
        """
        if rdr.link is None or not self.bridge.connected():
            return None
        try:
            return read_tag(
                rdr.link,
                bambu_master_key=self.bambu_master_key,
                is_excluded=self._excluder_for(lane_name, rdr),
                creality_key=self.creality_key,
                creality_encryption_key=self.creality_encryption_key)
        except Exception as e:
            self.logger.warning(
                f"BT RFID {rdr.name}: read failed: {e}")
            return None

    def _read_once(self, lane_name: str,
                   rdr: AFC_BoxTurtle_rfid_reader) -> Optional[dict]:
        """
        Run one read on a worker thread and wait reactor-friendly: the
        calling g-code blocks, the printer does not.

        :param lane_name: the lane the read is for
        :param rdr: the reader to poll
        :return Optional[dict]: the raw read_tag dict, or None
        """
        completion = self.reactor.completion()
        result: List[Any] = [None]

        def worker() -> None:
            """
            Background read; completes the reactor completion when done.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            result[0] = self._read_blocking(lane_name, rdr)
            self.reactor.register_async_callback(
                lambda et: completion.complete(None))

        threading.Thread(target=worker, daemon=True,
                         name="afc_bt_rfid_rd").start()
        completion.wait(self.reactor.monotonic() + 30.0)
        return result[0]

    def _parked_tag_once(self, rdr: AFC_BoxTurtle_rfid_reader) -> Optional[str]:
        """
        Run _parked_tag on a worker thread, waited for reactor-friendly.

        _clear_sibling runs on the reactor, and the probe takes as long as a
        read, so it must not run there directly.

        :param rdr: the reader whose antenna both lanes share
        :return Optional[str]: the parked tag's UID hex, or None
        """
        completion = self.reactor.completion()
        result: List[Any] = [None]

        def worker() -> None:
            """
            Background parked-tag read; completes the reactor completion.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            result[0] = self._parked_tag(rdr)
            self.reactor.register_async_callback(
                lambda et: completion.complete(None))

        threading.Thread(target=worker, daemon=True,
                         name="afc_bt_rfid_rd").start()
        completion.wait(self.reactor.monotonic() + 30.0)
        return result[0]

    def _apply(self, lane_name: str, tag: dict, gcmd: Any) -> None:
        """
        Apply a successful read to the lane through the shared framework.

        :param lane_name: the lane the tag belongs to
        :param tag: the raw read_tag result
        :param gcmd: the invoking command, for console output
        """
        self._last_uid_by_lane[lane_name] = (tag.get("uid") or "").lower()
        lane = self._lane_obj(lane_name)
        if lane is None:
            gcmd.respond_info(
                f"AFC_BT_RFID: read uid {tag.get('uid')} but lane {lane_name} "
                f"is not an AFC lane, nothing applied.")
            return
        self.apply_to_lane(lane, tag)

    def _map(self, tag: dict) -> dict:
        """
        Map a raw read to the shared slot_info shape (AFCUnitRFID hook).

        :param tag: the raw read_tag result
        :return dict: AFC slot_info
        """
        return map_tag_to_slot_info(tag)

    def _require_lane_reader(
            self, gcmd: Any) -> Tuple[str, AFC_BoxTurtle_rfid_reader]:
        """
        Resolve LANE= to (lane_name, reader) or raise the g-code error.

        :param gcmd: the invoking command
        :return tuple: (lane_name, reader)
        """
        lane_name = gcmd.get("LANE")
        rdr = self._reader_by_lane.get(lane_name)
        if rdr is None:
            error_str = (f"AFC_BT_RFID: no reader serves lane {lane_name}. "
                         f"Configured: "
                         f"{sorted(self._reader_by_lane) or 'none'}")
            raise gcmd.error(error_str)
        return lane_name, rdr

    def cmd_AFC_BT_RFID_READ(self, gcmd: Any) -> None:
        """
        Read the tag in a lane's antenna right now and apply it.

        The no-motion probe: a spool at rest may hold its tag outside the
        antenna's arc, which is what AFC_BT_RFID_STAGE's feed is for.

        Usage
        -------
        `AFC_BT_RFID_READ LANE=<lane>`

        Example
        -------
        ```
        AFC_BT_RFID_READ LANE=lane8
        ```
        """
        lane_name, rdr = self._require_lane_reader(gcmd)
        tag = self._read_once(lane_name, rdr)
        if tag is None:
            gcmd.respond_info(
                f"AFC_BT_RFID_READ: nothing readable in {rdr.name}'s field for "
                f"{lane_name}. If the tag rides the spool, use AFC_BT_RFID_STAGE "
                f"to spin it past the antenna.")
            return
        self._apply(lane_name, tag, gcmd)

    def cmd_AFC_BT_RFID_STATUS(self, gcmd: Any) -> None:
        """
        Report readers, chip versions, the lane map and last reads.

        Usage
        -------
        `AFC_BT_RFID_STATUS`

        Example
        -------
        ```
        AFC_BT_RFID_STATUS
        ```
        """
        if not self._readers:
            gcmd.respond_info(
                "AFC_BT_RFID: no reader sections configured "
                "([AFC_BoxTurtle_rfid <name>] with bus + lanes).")
            return
        lines = [f"bridge: "
                 f"{'connected' if self.bridge.connected() else 'OFFLINE'} "
                 f"({self.serial_port})"]
        for rdr in self._readers:
            # "retrying": the connect timer keeps probing offline readers.
            ver = ("offline (retrying)" if rdr.version is None
                   else f"version reg 0x{rdr.version:02X}")
            lines.append(f"{rdr.name}: {ver}  lanes: {', '.join(rdr.lanes)}")
        for ln, uid in sorted(self._last_uid_by_lane.items()):
            lines.append(f"{ln}: last uid {uid}")
        gcmd.respond_info("AFC_BT_RFID status\n" + "\n".join(lines))

    def _on_lane_prep_loaded(self, lane: Any) -> None:
        """
        Scan the inserted spool's tag, once AFC has seated the filament.

        Runs after prep_load has homed the tip to the load sensor (a known
        position) and before prep_post_load feeds to the hub (so the bowden is
        empty for the sweep). It also runs before a TD-1 capture, so the
        measurement refines the tag's record. A failed read never fails the
        insert.

        :param lane: the AFCLane being inserted
        """
        name = getattr(lane, "name", None)
        if not self.scan_on_insert or name is None:
            return
        rdr = self._reader_by_lane.get(name)
        if rdr is None:
            return                    # no reader watches this lane
        if self._sweeping:
            self.logger.warning(
                f"AFC_BT_RFID: {name} inserted while a sweep was already "
                f"running, not scanning this one.")
            return
        if rdr.version is None:
            self.logger.warning(
                f"AFC_BT_RFID: {name} inserted but {rdr.name} is offline, "
                f"skipping the tag scan. Check AFC_BT_RFID_STATUS.")
            return
        advance = math.pi * self.spool_diameter_mm * self.tag_sweep_revs
        advance, cut = self._sweep_room(lane, advance)
        if cut:
            self.logger.warning(
                f"AFC_BT_RFID: {name} inserted while {cut}"
                + (f"; scanning only {advance:.0f}mm." if advance > 0.0 else
                   "; not scanning. AFC_BT_RFID_STAGE can scan it once the "
                   "hub is clear."))
            if advance <= 0.0:
                return
        back = self.tag_retract_speed or getattr(
            lane, "long_moves_speed", None) or self.tag_sweep_speed
        self._sweeping = True
        token = None
        try:
            token = self._clear_sibling(name, rdr)
            step = (self.tag_sweep_step_mm
                    or self._sweep_step_for(lane, self.tag_sweep_speed))
            tag, fed = self._sweep_for_tag(lane, name, rdr, advance,
                                           step, self.tag_sweep_speed)
            self._restore(lane, fed, back)
            if tag is None:
                self.logger.info(
                    f"AFC_BT_RFID: no tag on {name} after {fed:.0f}mm "
                    f"({fed / (math.pi * self.spool_diameter_mm):.1f} turns); "
                    f"filament restored.")
                return
            self.apply_to_lane(lane, tag)
            self._last_uid_by_lane[name] = (tag.get("uid") or "").lower()
            self.logger.info(
                f"AFC_BT_RFID: {name} tag {tag.get('uid')} read after "
                f"{fed:.0f}mm at {self.tag_sweep_speed:.0f}mm/s in "
                f"{step:.0f}mm chunks; filament restored.")
        except Exception as e:
            self.logger.warning(
                f"AFC_BT_RFID: tag scan failed for {name} ({e}), the insert "
                f"carries on without it.")
        finally:
            self._restore_sibling(token)
            self._sweeping = False

    def _sibling_lane(self, lane_name: str,
                      rdr: AFC_BoxTurtle_rfid_reader) -> Optional[Any]:
        """
        Return the other lane on this reader's antenna, if AFC knows it.

        :param lane_name: the lane being scanned
        :param rdr: the reader both lanes share
        :return Optional[Any]: the sibling's AFCLane, or None
        """
        for ln in rdr.lanes:
            if ln == lane_name:
                continue
            obj = self._lane_obj(ln)
            if obj is not None:
                return obj
        return None

    def _clear_sibling(self, lane_name: str,
                       rdr: AFC_BoxTurtle_rfid_reader) -> Optional[tuple]:
        """
        Roll the sister lane back so its parked tag leaves the shared antenna.

        UID exclusion cannot reject a sister tag seen for the first time, so
        the tag is moved out of the field before the sweep instead. Same gates
        as ACE2: only an idle, hub-staged sibling moves, never one loaded to
        the toolhead or during a print.

        :param lane_name: the lane being scanned
        :param rdr: the reader both lanes share
        :return Optional[tuple]: a token for _restore_sibling, or None if
            nothing was moved
        """
        if not self.sibling_tag_adjust:
            return None
        sib = self._sibling_lane(lane_name, rdr)
        if sib is None:
            return None
        # Only move the sister when a tag is actually parked on the antenna,
        # as ACE2 and ViViD do.
        self._baseline_uid = None
        parked = self._parked_tag_once(rdr)
        if parked is None:
            return None
        # The parked tag may be this lane's own, so compare against
        # _last_uid_by_lane (as ACE2's _is_sibling_tag does). An unknown UID
        # stays a collision.
        mine = (self._last_uid_by_lane.get(lane_name) or "").lower()
        if mine and parked.lower() == mine:
            self.logger.info(
                f"AFC_BT_RFID: tag {parked} on {rdr.name}'s antenna is "
                f"{lane_name}'s own, no collision, leaving {sib.name} "
                f"alone.")
            return None
        # An empty or unseated sister cannot own the parked tag, so it is this
        # lane's: set no baseline and let the sweep read it.
        if not getattr(sib, "prep_state", False) or not self._seated(sib):
            self.logger.info(
                f"AFC_BT_RFID: tag {parked} on {rdr.name}'s antenna, but "
                f"{sib.name} holds no seated spool: it is {lane_name}'s own, "
                f"reading it.")
            return None
        # The tag is the seated sister's: exclude it as a baseline, since any
        # gate below may leave it on the coil. Cleared once she is rolled clear.
        self._baseline_uid = parked
        self.logger.info(
            f"AFC_BT_RFID: tag {parked} is parked on {rdr.name}'s antenna "
            f"while {lane_name} is read, clearing {sib.name} off it.")
        if getattr(sib, "tool_loaded", False):
            return None                   # loaded to the toolhead: never move
        # loaded_to_hub is remembered, not measured; _seated above checked the
        # load switch, which has the last word.
        if not getattr(sib, "loaded_to_hub", False):
            return None                   # position unknown: do not guess
        try:
            if self.afc.function.is_printing():
                return None               # never mid-print
        except Exception:
            pass
        want = self.sibling_tag_adjust_dist
        speed = getattr(sib, "long_moves_speed", None) or self.tag_sweep_speed
        step = getattr(sib, "short_move_dis", None) or 10.0
        moved = 0.0
        try:
            # Stepped so the distance is a maximum: stop before the sister
            # would come off its load switch.
            while moved < want and self._seated(sib):
                hop = min(step, want - moved)
                self._lane_move(sib, -hop, speed, assist=True)
                moved += hop
        except Exception as e:
            self.logger.warning(
                f"AFC_BT_RFID: could not roll {sib.name} back off the shared "
                f"antenna ({e}); reading {lane_name} anyway.")
            return (sib, moved, speed) if moved > 0.0 else None
        if moved <= 0.0:
            return None
        self._baseline_uid = None     # it is off the antenna now
        short = " (stopped early at its load switch)" if moved < want else ""
        self.logger.info(
            f"AFC_BT_RFID: rolled {sib.name} back {moved:.0f}mm to clear its "
            f"tag off {rdr.name}'s antenna while {lane_name} is read{short}.")
        return (sib, moved, speed)

    def _restore_sibling(self, token: Optional[tuple]) -> None:
        """
        Put a rolled-back sister lane back where a normal insert leaves it.

        Gear slip makes a blind give-back land short, so after it the sister is
        brought back onto its load switch and re-staged by _restage_sibling.
        If she does not come back to her load switch, ``loaded_to_hub`` is
        cleared so the next load re-stages her.

        :param token: the value _clear_sibling returned
        """
        if not token:
            return
        sib, dist, speed = token
        try:
            self._lane_move(sib, dist, speed, assist=False)
            # If the roll-back came off the load switch, creep back onto it
            # first; behind it the lane reads as "spool removed".
            step = getattr(sib, "short_move_dis", None) or 10.0
            slowf = getattr(sib, "short_moves_speed", None) or speed
            crept = 0.0
            for _ in range(int(dist / step) + 4):
                if self._seated(sib):
                    break
                self._lane_move(sib, step, slowf, assist=False)
                crept += step
            if crept:
                self.logger.info(
                    f"AFC_BT_RFID: fed {sib.name} a further {crept:.0f}mm to "
                    f"put it back on its load switch.")
            if not self._seated(sib):
                # Tell the operator this move is why the lane reads as removed.
                sib.loaded_to_hub = False
                self.logger.warning(
                    f"AFC_BT_RFID: {sib.name} is NOT back on its load switch "
                    f"after the read; its filament may have come out of the "
                    f"lane. Re-seat it and check the spool.")
                return
            self._restage_sibling(sib, dist)
        except Exception as e:
            sib.loaded_to_hub = False
            self.logger.warning(
                f"AFC_BT_RFID: could not restore {sib.name} after the read "
                f"({e}); its next load will re-home it.")

    def _home_back_to_load(self, lane: Any, dist: float) -> None:
        """
        Retract to the load switch with the espooler running.

        Same homing move as the unit's move_to_load, but with assist YES
        rather than DYNAMIC: DYNAMIC is off below 200mm, and without the
        spooler retracted filament piles up loose on the reel.

        :param lane: the AFC lane to retract
        :param dist: how far to retract, mm; the endstop is what stops it
        """
        lane.move_to(dist * MoveDirection.NEG, SpeedMode.LONG,
                     endstop=lane.load_es, assist_active=AssistActive.YES,
                     use_homing=True)

    def _homing_available(self, lane: Any) -> bool:
        """
        Whether AFC's own homing moves can be used on this lane.

        :param lane: the AFC lane
        :return bool: True when move_to, the enums and homing are all present
        """
        return (SpeedMode is not None
                and MoveDirection is not None
                and AssistActive is not None
                and callable(getattr(lane, "move_to", None))
                and bool(getattr(self.afc, "homing_enabled", False)))

    def _restage_sibling(self, lane: Any, given_back: float) -> None:
        """
        Put the sibling back where a normal insert leaves it, using the same
        two moves: home onto the load sensor, then prep_post_load feeds
        dist_hub, which stops short of the hub. Hunting for the hub switch
        instead parks the tip in the shared hub bore and jams other lanes.

        Gated like prep_post_load (load_to_hub set, not direct-hub). Without
        homing it keeps the plain give-back and clears loaded_to_hub so the
        next load re-stages it.

        :param lane: the sibling lane to re-stage
        :param given_back: distance already re-fed, mm, for the log only
        """
        hub = getattr(lane, "hub_obj", None)
        if hub is None or getattr(lane, "is_direct_hub", lambda: False)():
            return                        # nothing between the lane and the tool
        if not getattr(lane, "load_to_hub", False):
            return                        # config says this lane does not stage
        unit = getattr(lane, "unit_obj", None)
        dist_hub = getattr(lane, "dist_hub", None)
        if (unit is None
            or not dist_hub
            or not callable(getattr(unit, "prep_post_load", None))
            or not callable(getattr(unit, "prep_load", None))):
            return
        if not self._homing_available(lane):
            lane.loaded_to_hub = False
            self.logger.warning(
                f"AFC_BT_RFID: homing is off, so {lane.name} was only re-fed "
                f"{given_back:.0f}mm rather than re-staged. Its next load "
                f"will put it back at the hub.")
            return
        # 1. Back to the load switch, the way a load and an unload do it.
        #    First the reverse home eject_lane makes...
        self._home_back_to_load(lane, dist_hub)
        # ...then forward onto it with prep_load, since the reverse home stops
        # where the switch releases and prep_post_load needs load_state.
        unit.prep_load(lane)
        if not self._seated(lane):
            # The homing moves should have made the switch; something is wrong.
            lane.loaded_to_hub = False
            self.logger.warning(
                f"AFC_BT_RFID: {lane.name} would not come back to its load "
                f"switch, so it is not staged. Check the spool. Its next "
                f"load will re-home it.")
            return
        # 2. Feed dist_hub. prep_post_load's own job, and it sets
        #    loaded_to_hub itself; it only runs when the flag is clear.
        lane.loaded_to_hub = False
        unit.prep_post_load(lane)
        self.logger.info(
            f"AFC_BT_RFID: re-staged {lane.name} the way a load does: "
            f"homed to its load switch, then fed dist_hub "
            f"({dist_hub:.0f}mm). The hub is untouched.")

    def _seated(self, lane: Any) -> bool:
        """
        Whether the filament is still at the sensor the sweep started from.

        The load sensor, not prep: prep_load homes the filament there, so it is
        a real reference, whereas prep trips before the gears take hold.

        :param lane: the AFC lane
        :return bool: True while filament covers the load sensor
        """
        raw = getattr(lane, "raw_load_state", None)
        if raw is not None:
            return bool(raw)
        return bool(getattr(lane, "load_state", False))

    def _restore(self, lane: Any, fed: float, speed: float) -> float:
        """
        Put the filament back on the load sensor the sweep started from.

        Gear slip makes a blind ``-fed`` retract walk the tip out of the lane,
        so this homes on the load sensor with AFC's own moves (as prep_load and
        eject_lane do). Without homing it gives back the bulk, steps the tail
        while checking the sensor, then creeps forward if it went past.

        :param lane: the AFC lane to move
        :param fed: distance the sweep fed, mm; negative when the settle
            backed up further than the sweep fed
        :param speed: speed for the bulk of the return, mm/s
        :return float: distance actually given back, mm (negative when it
            fed forward instead)
        """
        if fed <= 0.0:
            # A tip left behind its load switch reads as an empty lane, so
            # home it forward onto the switch.
            if fed < 0.0 and not self._seated(lane):
                return -self._home_onto_load(lane, -fed, speed)
            return 0.0
        # Nothing on the sensor to home against: give back the measured distance.
        if not self._seated(lane):
            self._lane_move(lane, -fed, speed, assist=True)
            return fed
        unit = getattr(lane, "unit_obj", None)
        if (self._homing_available(lane)
            and callable(getattr(unit, "prep_load", None))):
            # A reverse home stops where the switch releases, leaving the tip
            # behind it, so follow it with prep_load's forward home onto it.
            self._home_back_to_load(lane, fed + 2.0 * (
                getattr(lane, "short_move_dis", None) or 10.0))
            unit.prep_load(lane)
            return fed
        short = getattr(lane, "short_move_dis", None) or 10.0
        slow = getattr(lane, "short_moves_speed", None) or speed
        tail = min(fed, max(3.0 * short, 30.0))
        back = 0.0
        bulk = fed - tail
        if bulk > 0.0:
            self._lane_move(lane, -bulk, speed, assist=True)
            back += bulk
        # Step the tail so the sensor is checked between moves; the step size
        # is the overshoot bound.
        while self._seated(lane) and back < fed + tail:
            self._lane_move(lane, -short, slow, assist=True)
            back += short
        # Off the sensor now: creep forward until it catches again, which is
        # where the operator left the filament.
        for _ in range(int(tail / short) + 4):
            if self._seated(lane):
                break
            self._lane_move(lane, short, slow, assist=False)
            back -= short
        return back

    def _home_onto_load(self, lane: Any, behind: float, speed: float) -> float:
        """
        Bring a tip that ended behind its load switch forward onto it.

        With homing this is the unit's prep_load, the forward home onto the
        load switch that an insert uses. Without it, short feeds that check
        the switch between moves, as calibration does.

        :param lane: the AFC lane to move
        :param behind: how far behind the switch the tip should be, mm
        :param speed: feed speed when the lane has no short-move speed, mm/s
        :return float: distance fed, mm; ``behind`` for a home, whose own
            distance is not reported
        """
        unit = getattr(lane, "unit_obj", None)
        if (self._homing_available(lane)
            and callable(getattr(unit, "prep_load", None))):
            unit.prep_load(lane)
            return behind
        short = getattr(lane, "short_move_dis", None) or 10.0
        slow = getattr(lane, "short_moves_speed", None) or speed
        fed = 0.0
        for _ in range(int(behind / short) + 4):
            if self._seated(lane):
                break
            self._lane_move(lane, short, slow, assist=False)
            fed += short
        return fed

    #: Kept between a sweeping tip and a hub another lane's filament is in.
    HUB_MARGIN_MM = 25.0

    def _hub_busy(self, lane: Any) -> Optional[str]:
        """
        Say what is using this lane's hub, or None while it is clear.

        A sweep can feed over a metre, past dist_hub, into a hub another lane's
        filament is in.

        :param lane: the lane about to be swept
        :return Optional[str]: what occupies the hub, or None
        """
        hub = getattr(lane, "hub_obj", None)
        if hub is None:
            return None
        lanes = getattr(self.afc, "lanes", None) or {}
        for other in lanes.values():
            if (other is not lane
                and getattr(other, "hub_obj", None) is hub
                and getattr(other, "tool_loaded", False)):
                return f"{getattr(other, 'name', 'another lane')} is loaded through it"
        # A virtual hub's "switch" is every lane's load sensor, this one's
        # included, so only a real switch says anything here.
        virtual = getattr(hub, "is_virtual_pin", None)
        try:
            if not (callable(virtual) and virtual()) and getattr(hub, "state", False):
                return "its switch reads filament"
        except Exception:
            pass
        return None

    def _sweep_room(self, lane: Any, advance: float,
                    staged: Optional[bool] = None) -> Tuple[float, Optional[str]]:
        """
        Bound a sweep so it never drives the tip into a hub in use.

        The sweep starts from the load switch, or from the staged position
        when the lane is loaded_to_hub, which is already dist_hub along.

        :param lane: the lane about to be swept
        :param advance: the sweep wanted, mm
        :param staged: whether the sweep starts from the staged position;
            None reads the lane's loaded_to_hub
        :return Tuple[float, Optional[str]]: (the sweep allowed, mm; why it was
            cut, or None)
        """
        busy = self._hub_busy(lane)
        if busy is None:
            return advance, None
        dist_hub = float(getattr(lane, "dist_hub", 0.0) or 0.0)
        if staged is None:
            staged = bool(getattr(lane, "loaded_to_hub", False))
        start = dist_hub if staged else 0.0
        room = max(0.0, dist_hub - start - self.HUB_MARGIN_MM)
        if room >= advance:
            return advance, None
        hub = getattr(getattr(lane, "hub_obj", None), "name", "the hub")
        return room, (f"hub {hub} is in use ({busy}), so the sweep stops "
                      f"{self.HUB_MARGIN_MM:.0f}mm short of it")

    def _unstage_for_scan(self, lane: Any) -> None:
        """
        Bring a lane staged at the hub back onto its load switch, so a scan
        starts where the insert scan does, with the path to the hub empty.

        The same two homing moves a restore makes: a reverse home that stops
        where the switch releases, then prep_load's forward home onto it.

        :param lane: the staged lane
        :raises RuntimeError: when the filament does not come back onto the
            load switch
        """
        dist_hub = float(getattr(lane, "dist_hub", 0.0) or 0.0)
        short = getattr(lane, "short_move_dis", None) or 10.0
        self._home_back_to_load(lane, dist_hub + 2.0 * short)
        lane.unit_obj.prep_load(lane)
        lane.loaded_to_hub = False
        if not self._seated(lane):
            error_str = (f"{lane.name} did not come back onto its load "
                         f"switch. Check the spool; its next load will "
                         f"re-home it")
            raise RuntimeError(error_str)

    def _restage_after_scan(self, lane: Any) -> bool:
        """
        Put a scanned lane back at the hub the way an insert leaves it: feed
        dist_hub from the load switch (the unit's own prep_post_load).

        :param lane: the lane, back on its load switch after the restore
        :return bool: True when it is staged again
        """
        lane.loaded_to_hub = False
        if not self._seated(lane):
            self.logger.warning(
                f"AFC_BT_RFID: {lane.name} is not on its load switch after "
                f"the scan, so it was not re-staged. Its next load will "
                f"re-home it.")
            return False
        lane.unit_obj.prep_post_load(lane)
        if not getattr(lane, "loaded_to_hub", False):
            # The unit's prep_post_load skips its feed without a word (no
            # load_to_hub, a direct hub, a sensor off), so say it here.
            self.logger.warning(
                f"AFC_BT_RFID: {lane.name} is back on its load switch, but "
                f"re-staging it at the hub did not take, so it was left at "
                f"the load switch.")
            return False
        return True

    def _refuse_if_busy(self, gcmd: Any) -> None:
        """
        Refuse to start a sweep while anything else is moving filament.

        Two overlapping lane moves on one stepper corrupt step generation and
        shut down every MCU. AFC has no lane-moving flag, so this checks
        idle_timeout, which reads "Printing" whenever motion is queued.

        :param gcmd: the command to refuse through
        """
        if self._sweeping:
            error_str = ("AFC_BT_RFID_STAGE: a sweep is already running. "
                         "Wait for it to finish or restart the firmware.")
            raise gcmd.error(error_str)
        idle = self.printer.lookup_object("idle_timeout", None)
        if idle is None:
            return
        try:
            state = idle.get_status(self.reactor.monotonic()).get("state")
        except Exception:
            return
        if state == "Printing":
            error_str = ("AFC_BT_RFID_STAGE: the printer is moving something "
                         "already (idle_timeout says Printing): a TD-1 "
                         "capture, a toolchange or a print. Two filament "
                         "moves at once corrupt step generation and shut down "
                         "every MCU, so this waits rather than joining in. "
                         "Try again once it is idle.")
            raise gcmd.error(error_str)

    def _lane_move(self, lane: Any, distance: float, speed: float,
                   assist: bool) -> None:
        """
        Move the lane, driving the espooler only when asked.

        Feeding free-wheels and needs no assist; a retract needs the espooler
        to wind the slack back onto the reel.

        :param lane: the AFC lane to move
        :param distance: signed distance, mm (negative retracts)
        :param speed: feed speed, mm/s
        :param assist: run the espooler for this move
        """
        accel = getattr(lane, "long_moves_accel", 400)
        active = False
        if assist:
            active = True
            getter = getattr(lane, "get_active_assist", None)
            if callable(getter) and AssistActive is not None:
                try:
                    active = getter(distance, AssistActive.YES)
                except Exception:
                    active = True
        lane.move(distance, speed, accel, active)

    # Fraction of each chunk spent at speed rather than ramping; 0.6 reads as
    # continuous motion.
    SWEEP_CRUISE_FRACTION = 0.6
    SWEEP_STEP_MIN = 20.0
    SWEEP_STEP_MAX = 150.0

    def _sweep_step_for(self, lane: Any, speed: float) -> float:
        """
        Choose a sweep chunk long enough to actually reach ``speed``.

        Each chunk is a complete move, and one shorter than v**2/a never
        reaches speed. So the chunk leaves SWEEP_CRUISE_FRACTION at constant
        velocity, bounded by SWEEP_STEP_MIN and SWEEP_STEP_MAX.

        :param lane: the AFC lane, for its acceleration
        :param speed: sweep speed, mm/s
        :return float: chunk length, mm
        """
        accel = getattr(lane, "long_moves_accel", None) or 400.0
        ramps = (speed * speed) / accel          # both ramps together
        step = ramps / max(0.05, 1.0 - self.SWEEP_CRUISE_FRACTION)
        return min(self.SWEEP_STEP_MAX, max(self.SWEEP_STEP_MIN, step))

    def _sweep_for_tag(self, lane: Any, lane_name: str,
                       rdr: AFC_BoxTurtle_rfid_reader, advance: float,
                       step: float, speed: float) -> tuple:
        """
        Home on the tag: feed the lane while polling the reader continuously
        and stop when a tag answers; the distance bound is the failsafe.

        The poll runs on its own thread, since a read takes too long to fit
        between moves. Motion is issued in chunks, the abort granularity.

        :param lane: the AFC lane object to move
        :param lane_name: the lane's name (drives sibling exclusion)
        :param rdr: the reader whose antenna watches this lane
        :param advance: maximum distance to sweep, mm
        :param step: chunk size, mm
        :param speed: feed speed, mm/s
        :return tuple: (tag or None, distance fed when it answered)
        """
        stop = threading.Event()
        found: List[Any] = [None, 0.0]
        fed = [0.0]

        def poller() -> None:
            """
            Poll the reader until a tag is read or stop is set.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            while not stop.is_set():
                tag = self._read_blocking(lane_name, rdr)
                if tag:
                    found[0] = tag
                    found[1] = fed[0]
                    stop.set()
                    return

        th = threading.Thread(target=poller, daemon=True, name="afc_bt_rfid_sw")
        th.start()
        try:
            while fed[0] < advance and not stop.is_set():
                hop = min(step, advance - fed[0])
                self._lane_move(lane, hop, speed, assist=False)
                fed[0] += hop
        finally:
            stop.set()
            th.join(timeout=10.0)
        # One more read covers the final chunk, via _read_once since this runs
        # on the reactor.
        if found[0] is None:
            tag = self._read_once(lane_name, rdr)
            if tag:
                found[0], found[1] = tag, fed[0]
        return found[0], fed[0]

    def cmd_AFC_BT_RFID_STAGE(self, gcmd: Any) -> None:
        """
        Home on the lane's tag: feed slowly while polling the reader, stop as
        soon as the tag answers, and give up after a bounded number of spool
        revolutions.

        REVS bounds the search in spool turns (via ``spool_diameter_mm``);
        ADVANCE overrides it with a distance. Refuses while anything else is
        moving filament, or for a lane loaded in the toolhead. A hub-staged
        lane is homed back to its load switch, swept, and re-staged (needs
        homing). A busy shared hub shortens the sweep. RETRACT=1 (default)
        puts the filament back at RETRACT_SPEED (default: long-move speed).

        Usage
        -------
        `AFC_BT_RFID_STAGE LANE=<lane> [REVS=2] [ADVANCE=mm] [SPEED=mm/s] [STEP=mm]
        [RETRACT=0|1] [RETRACT_SPEED=mm/s]`

        Example
        -------
        ```
        AFC_BT_RFID_STAGE LANE=lane8 REVS=2
        ```
        """
        self._refuse_if_busy(gcmd)
        lane_name, rdr = self._require_lane_reader(gcmd)
        lane = self._lane_obj(lane_name)
        if lane is None:
            error_str = f"AFC_BT_RFID_STAGE: {lane_name} is not an AFC lane"
            raise gcmd.error(error_str)
        if rdr.version is None:
            error_str = (f"AFC_BT_RFID_STAGE: {rdr.name} is offline: no "
                         f"point moving filament at a reader that cannot "
                         f"answer. Check the bridge with AFC_BT_RFID_STATUS.")
            raise gcmd.error(error_str)
        if getattr(lane, "tool_loaded", False):
            error_str = (f"AFC_BT_RFID_STAGE: {lane_name} is loaded in the "
                         f"toolhead. The scan turns the spool by moving its "
                         f"filament, so unload it first.")
            raise gcmd.error(error_str)
        # A lane staged at the hub is brought back to its load switch first
        # and re-staged afterwards, so the scan runs from where the insert
        # scan does. Both ends are homing moves, so homing has to be on.
        staged = bool(getattr(lane, "loaded_to_hub", False))
        if staged and not self._homing_available(lane):
            error_str = (f"AFC_BT_RFID_STAGE: {lane_name} is staged at the "
                         f"hub and homing is off, so there is no load switch "
                         f"to bring it back to for the scan. Eject it and "
                         f"insert it again to scan it.")
            raise gcmd.error(error_str)
        revs = gcmd.get_float("REVS", self.tag_sweep_revs, above=0.0)
        default_mm = math.pi * self.spool_diameter_mm * revs
        advance = gcmd.get_float("ADVANCE", default_mm, minval=0.0)
        speed = gcmd.get_float("SPEED", self.tag_sweep_speed, above=0.0)
        step = gcmd.get_float("STEP", self.tag_sweep_step_mm, minval=0.0)
        if step <= 0.0:
            step = self._sweep_step_for(lane, speed)
        retract = gcmd.get_int("RETRACT", 1 if self.retract_after_read else 0,
                               minval=0, maxval=1)
        if staged:
            retract = 1        # it goes back to the hub, via its load switch
        back = gcmd.get_float("RETRACT_SPEED", self.tag_retract_speed,
                              minval=0.0)
        if back <= 0.0:
            back = getattr(lane, "long_moves_speed", None) or speed
        # Room is judged from the load switch, where the sweep will start.
        advance, cut = self._sweep_room(lane, advance, staged=False)
        if advance <= 0.0:
            error_str = (f"AFC_BT_RFID_STAGE: {cut}, and {lane_name}'s "
                         f"dist_hub leaves no room before it. Unload that "
                         f"lane first to scan this one.")
            raise gcmd.error(error_str)

        gcmd.respond_info(
            f"AFC_BT_RFID_STAGE: homing on {lane_name}'s tag: up to "
            f"{advance:.0f}mm ({advance / (math.pi * self.spool_diameter_mm):.1f} "
            f"turns of a {self.spool_diameter_mm:.0f}mm spool) at "
            f"{speed:.0f}mm/s, polling {rdr.name} throughout. Coming back at "
            f"{back:.0f}mm/s."
            + (f" {lane_name} comes back to its load switch first and is "
               f"re-staged at the hub afterwards." if staged else "")
            + (f" Limited: {cut}." if cut else ""))
        self._sweeping = True
        token = None
        tag, fed = None, 0.0
        restaged = None
        try:
            if staged:
                try:
                    self._unstage_for_scan(lane)
                except RuntimeError as e:
                    error_str = f"AFC_BT_RFID_STAGE: {e}"
                    raise gcmd.error(error_str)
            try:
                token = self._clear_sibling(lane_name, rdr)
                tag, fed = self._sweep_for_tag(lane, lane_name, rdr, advance,
                                               step, speed)
            finally:
                self._restore_sibling(token)
                if retract:
                    # Homes on the load sensor when there is filament there
                    # to home against; falls back to the measured distance
                    # when there is not.
                    self._restore(lane, fed, back)
                if staged:
                    restaged = self._restage_after_scan(lane)
        finally:
            self._sweeping = False
        where = ""
        if staged:
            where = (" and re-staged at the hub" if restaged else
                     ", but it is not staged at the hub; see the warning")
        restored = (" (filament restored" + where + ")") if retract else ""
        one_turn = math.pi * self.spool_diameter_mm
        if tag is None:
            if cut:
                verdict = (f"Only {fed / one_turn:.1f} of a turn fitted "
                           f"because {cut}. Unload that lane to give the "
                           f"scan a full turn.")
            elif fed >= one_turn:
                verdict = ("That is a full pass of the spool, so the tag "
                           "never enters this antenna's field; move the "
                           "reader rather than sweeping further.")
            else:
                verdict = (f"That is only {fed / one_turn:.1f} of a turn, so "
                           f"the tag may simply not have come round yet; "
                           f"raise REVS before suspecting the mounting.")
            gcmd.respond_info(
                f"AFC_BT_RFID_STAGE: no tag in {fed:.0f}mm of sweep on "
                f"{rdr.name}{restored}. {verdict}")
            return
        self._apply(lane_name, tag, gcmd)
        # Not a number to tune with: the tag's starting angle is arbitrary.
        gcmd.respond_info(
            f"AFC_BT_RFID_STAGE: tag answered after {fed:.0f}mm of feed"
            f"{restored}.")


def load_config(config: "ConfigWrapper") -> AFC_BoxTurtle_rfid:
    """
    Klipper entry point for the bare ``[AFC_BoxTurtle_rfid]`` coordinator.

    :param config: Klipper config wrapper
    :return AFC_BoxTurtle_rfid: the coordinator
    """
    return AFC_BoxTurtle_rfid(config)


def load_config_prefix(config: "ConfigWrapper") -> AFC_BoxTurtle_rfid_reader:
    """
    Klipper entry point for ``[AFC_BoxTurtle_rfid <name>]`` reader sections.

    :param config: Klipper config wrapper
    :return AFC_BoxTurtle_rfid_reader: the reader
    """
    return AFC_BoxTurtle_rfid_reader(config)
