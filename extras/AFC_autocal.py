# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC AutoCal: per-spool pressure advance, keyed off a lane's spool_id and stored in the Spoolman
# extra field 'flow_k', so a K calibrated on one printer is used by every printer sharing that
# Spoolman.


from __future__ import annotations
import threading
import chelper
from typing import Any, Callable, Dict, Optional, Set, Tuple, TYPE_CHECKING

from extras.AFC_RFID import SpoolmanClient

if TYPE_CHECKING:
    from extras.AFC_utils import AFCGcmd
    from configfile import ConfigWrapper
    from gcode import GCodeCommand


class AFC_autocal:
    """
    Per-spool flow calibration (K): applies a spool's stored K on load and optionally
    auto-calibrates spools that have none, persisting the result to the spool's 'flow_k' Spoolman
    extra field.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Sets up the autocal module: wires up the two toggles (apply_stored_k and auto_calibrate, or
        the ``enabled`` back-compat master that turns both on), registers the AFC/homing event
        handlers, and adds the two manual gcode commands.

        :param config: The Klipper config object for the [AFC_autocal] section
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        # self.afc stays None until klippy:ready.
        self.logger = self.printer.load_object(config, 'AFC').logger
        self.afc: Optional[Any] = None
        self._lane_flow_k: Dict[str, Tuple[Optional[int], float]] = {}
        self._k_fetch_inflight: Set[str] = set()
        self._staged_handled: Dict[str, int] = {}
        self._staged_pending: Set[str] = set()
        self._cal_pending: Set[str] = set()
        # Consumer mode only (no flow_calibrator).
        self._managed_extruders: Set[str] = set()
        self._wrapped_extruders: Set[str] = set()

        # 'enabled' is a back-compat master that defaults both on.
        master = config.getboolean('enabled', None)
        dflt = master if master is not None else False
        self.apply_stored_k = config.getboolean('apply_stored_k', dflt)
        self.auto_calibrate = config.getboolean('auto_calibrate', dflt)
        self.calibrate_gcode = config.get('calibrate_gcode', 'FLOW_CALIBRATE')
        # Suppress auto-calibration for this many seconds after klippy:ready so the startup prep
        # reconcile (which fires once prep_done flips) doesn't kick off a calibration.
        self._startup_cal_grace = config.getfloat('startup_cal_grace', 30.0)
        self._ready_time: Optional[float] = None

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('afc:tool_loaded', self._handle_tool_loaded)
        self.printer.register_event_handler('afc:spool_assigned', self._handle_spool_assigned)
        self.printer.register_event_handler('homing:home_rails_end', self._handle_home_rails_end)
        self.printer.register_event_handler('extruder:activate_extruder',
                                            self._handle_activate_extruder)

        self.gcode.register_command('AFC_APPLY_LANE_FLOW_K', self.cmd_APPLY_LANE_FLOW_K,
                                    desc="Apply stored flow K for the current lane")
        self.gcode.register_command('AFC_CALIBRATE_LANE_FLOW_K', self.cmd_CALIBRATE_LANE_FLOW_K,
                                    desc="Run flow calibration on the current lane and store K")

    def _handle_ready(self) -> None:
        """
        Runs once Klipper is ready.
        """
        self._ready_time = self.reactor.monotonic()
        self.afc = self.printer.lookup_object('AFC', None)
        if self.afc is None:
            self.logger.warning("AFC_autocal: AFC not loaded; disabled")
            return
        # Consumer mode only.
        if self._flow_calibrator() is None:
            self._wrap_pa_handlers()
            self._load_all_spoolman_k()

    def _spoolman(self) -> Optional[SpoolmanClient]:
        """
        Build a SpoolmanClient from the AFC moonraker handle.

        :return Optional[SpoolmanClient]: the client, or None if unavailable
        """
        mr = getattr(self.afc, 'moonraker', None) if self.afc else None
        if (mr is None
            or getattr(self.afc, 'spoolman', None) is None):
            return None
        return SpoolmanClient(mr)

    def _flow_calibrator(self) -> Optional[Any]:
        """
        Return the flow_calibrator object, or None on a consumer printer.

        :return Optional[Any]: the flow_calibrator, or None
        """
        return self.printer.lookup_object('flow_calibrator', None)

    def _can_calibrate(self) -> bool:
        """
        True only where auto-calibration is both enabled and possible (U1).

        :return bool: True if auto_calibrate is on and a flow_calibrator exists
        """
        return self.auto_calibrate and self._flow_calibrator() is not None

    def _apply_gate_ok(self, lane: Any) -> bool:
        """
        Whether it's safe to apply K to this lane right now.

        :param lane: The lane object to check
        :return bool: True if K may be applied for this lane now
        """
        if self._flow_calibrator() is None:
            return True
        return self._lane_on_active_toolhead(lane)

    @staticmethod
    def _norm_spool_id(sid: Any) -> Optional[int]:
        """
        Normalize a raw spool id to an int, or None when it's empty/invalid.

        :param sid: The raw spool id value to normalize
        :return Optional[int]: the id as an int, or None
        """
        if sid in (None, "", 0, "0"):
            return None
        try:
            return int(sid)
        except (ValueError, TypeError):
            return None

    def _set_lane_k(self, lane: Any, k: float) -> None:
        """
        Cache a lane's K keyed to its current spool id.

        :param lane: The lane object to cache K for
        :param k: The flow K value to store
        """
        self._lane_flow_k[lane.name] = (self._norm_spool_id(getattr(lane, 'spool_id', None)), k)

    def _get_lane_k(self, lane: Any) -> Optional[float]:
        """
        Return a lane's cached K, dropping it if the spool has since changed.

        :param lane: The lane object to look up
        :return Optional[float]: the cached K, or None
        """
        entry = self._lane_flow_k.get(lane.name)
        if entry is None:
            return None
        stored, k = entry
        if stored != self._norm_spool_id(getattr(lane, 'spool_id', None)):
            del self._lane_flow_k[lane.name]
            return None
        return k

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        Report each lane's cached K, so a slicer can show the K of a lane with no Spoolman spool.

        :param eventtime: The Klipper event time (unused)
        :return Dict[str, Any]: {"lane_k": {lane name: K}}
        """
        lanes = getattr(self.afc, 'lanes', None) or {}
        lane_k: Dict[str, float] = {}
        for name, (stored, k) in self._lane_flow_k.items():
            lane = lanes.get(name)
            if (lane is not None
                and stored == self._norm_spool_id(getattr(lane, 'spool_id', None))):
                lane_k[name] = k
        return {"lane_k": lane_k}

    def _read_k_from_spoolman(self, lane: Any) -> Optional[float]:
        """
        Read a lane's stored K from Spoolman (synchronous HTTP).

        :param lane: The lane object whose spool to read
        :return Optional[float]: the stored K, or None
        """
        sid = self._norm_spool_id(getattr(lane, 'spool_id', None))
        if sid is None:
            return None
        client = self._spoolman()
        return client.read_flow_k(sid) if client else None

    def _write_k_to_spoolman(self, lane: Any, k: float) -> None:
        """
        Persist a lane's K to its Spoolman spool.

        :param lane: The lane object whose spool to write
        :param k: The flow K value to persist
        """
        sid = self._norm_spool_id(getattr(lane, 'spool_id', None))
        if sid is None:
            return
        client = self._spoolman()
        if client is not None:
            client.write_flow_k(sid, k)

    def _apply_lane_k(self, lane_name: str) -> Optional[str]:
        """
        Apply a lane's cached K, dispatching on the printer mode.

        :param lane_name: The name of the lane whose K to apply
        :return Optional[str]: a status message, or None if nothing was applied
        """
        entry = self._lane_flow_k.get(lane_name)
        if entry is None:
            return None
        _, k = entry
        return self._put_k(lane_name, k)

    def _put_k(self, lane_name: str, k: float, what: str = "K") -> Optional[str]:
        """
        Set K for a lane, dispatching on the printer mode.

        :param lane_name: The name of the lane the K is for
        :param k: The flow K value to set
        :param what: What the K is, for the log ("K" or "default K")
        :return Optional[str]: a status message, or None if nothing was applied
        """
        flow_cal = self._flow_calibrator()
        if flow_cal is None:
            return self._apply_k_stepper(lane_name, k, what)
        ext = self.printer.lookup_object('toolhead').get_extruder()
        ext_name = ext.get_name()
        flow_cal._set_pressure_advance(ext, k)
        flow_cal._current_k[ext_name] = k
        msg = f"AFC autocal: applied {what}={k:.6f} for {lane_name} on {ext_name}"
        self.logger.debug(msg)
        return msg

    def _default_k(self, lane: Any) -> Optional[float]:
        """
        The K for a lane with none stored.

        :param lane: The lane object to find a default K for
        :return Optional[float]: the K, or None when there is none to find
        """
        if self._flow_calibrator() is None:
            ext_obj = getattr(lane, 'extruder_obj', None)
            ext_name = getattr(ext_obj, 'name', None) or 'extruder'
            printer_ext = self.printer.lookup_object(ext_name, None)
            estepper = getattr(printer_ext, 'extruder_stepper', None)
            k = getattr(estepper, 'config_pa', None)
            return float(k) if isinstance(k, (int, float)) else None
        params = self.printer.lookup_object('filament_parameters', None)
        ext = self.printer.lookup_object('toolhead').get_extruder()
        if (params is None
            or ext is None):
            return None
        k = params.get_flow_k(
            getattr(lane, 'spool_vendor', None) or "Generic",
            (getattr(lane, 'material', None)
             or getattr(self.afc, 'default_material_type', None) or "NONE"),
            getattr(lane, 'sub_type', "") or "NONE",
            ext.nozzle_diameter,
            getattr(ext, 'nozzle_volume_type', 'standard'))
        return float(k) if isinstance(k, (int, float)) else None

    def _apply_default_k(self, lane: Any) -> Optional[str]:
        """
        Set the default K for a lane with no stored K, so it doesn't print with the K of the lane
        before it on that toolhead.

        :param lane: The lane object that was just loaded
        :return Optional[str]: a status message, or None if nothing was applied
        """
        if (not self.apply_stored_k
            or not self._apply_gate_ok(lane)):
            return None
        try:
            k = self._default_k(lane)
        except Exception as e:
            self.logger.warning(f"AFC autocal: no default K for {lane.name}: {e}")
            return None
        if k is None:
            return None
        return self._put_k(lane.name, k, "default K")

    def _apply_k_stepper(self, lane_name: str, k: float, what: str = "K") -> Optional[str]:
        """
        Consumer-mode apply: set pressure advance on the lane's extruder stepper.

        :param lane_name: The name of the lane whose K to apply
        :param k: The flow K value to apply
        :param what: What the K is, for the log ("K" or "default K")
        :return Optional[str]: a status message, or None if nothing was applied
        """
        lane = self.afc.lanes.get(lane_name) if self.afc else None
        ext_obj = getattr(lane, 'extruder_obj', None) if lane is not None else None
        ext_name = getattr(ext_obj, 'name', None) or 'extruder'
        printer_ext = self.printer.lookup_object(ext_name, None)
        if printer_ext is None:
            self.logger.warning(f"AFC autocal: extruder {ext_name} not found")
            return None
        estepper = getattr(printer_ext, 'extruder_stepper', None)
        if estepper is None:
            self.logger.warning(f"AFC autocal: extruder {ext_name} has no extruder_stepper")
            return None
        smooth_time = getattr(estepper, 'config_smooth_time', estepper.pressure_advance_smooth_time)
        estepper._set_pressure_advance(k, smooth_time)
        self._managed_extruders.add(ext_name)
        msg = f"AFC autocal: applied {what}={k:.6f} for {lane_name} on {ext_name}"
        self.logger.debug(msg)
        return msg

    def _wrap_pa_handlers(self) -> None:
        """
        Wrap each per-extruder SET_PRESSURE_ADVANCE handler (consumer mode).
        """
        mux = getattr(self.gcode, 'mux_commands', {}).get("SET_PRESSURE_ADVANCE")
        if mux is None:
            return
        _key, values = mux
        for ext_name, orig_func in list(values.items()):
            if ext_name in self._wrapped_extruders:
                continue
            values[ext_name] = self._make_pa_wrapper(orig_func, ext_name)
            self._wrapped_extruders.add(ext_name)

    def _make_pa_wrapper(self, original: Callable, name: str) -> Callable:
        """
        Build a SET_PRESSURE_ADVANCE wrapper that blocks slicer PA while printing.

        :param original: The original mux handler for this extruder
        :param name: The extruder name this handler serves, None for the default handler that acts
        on the active extruder
        :return Callable: the wrapping handler
        """
        def wrapper(gcmd: GCodeCommand) -> None:
            """
            Ignore slicer PA for a managed extruder while printing, else run the original.

            :param gcmd: the gcode command
            """
            target = name if name is not None else self._active_extruder_name()
            if (target in self._managed_extruders
                and self.afc is not None
                and self._is_printing()):
                self.logger.debug(f"AFC autocal: slicer PA change ignored for {target} "
                                 f"(flow K managed)")
                gcmd.respond_info("AFC flow K active, slicer pressure advance ignored")
                return
            original(gcmd)
        return wrapper

    def _load_all_spoolman_k(self) -> None:
        """
        Pre-load stored K for all lanes at startup (consumer mode).
        """
        if (not self.apply_stored_k
            or self.afc is None):
            return
        if getattr(self.afc, 'moonraker', None) is None:
            return
        for lane in self.afc.lanes.values():
            try:
                if self._get_lane_k(lane) is not None:
                    continue
                k = self._read_k_from_spoolman(lane)
                if k is not None:
                    self._set_lane_k(lane, k)
            except Exception as e:
                self.logger.debug(f"AFC autocal: startup K load failed for {lane.name}: {e}")

    def _ensure_k_loaded(self, lane: Any) -> Optional[float]:
        """
        Return K for a lane: cached, else from Spoolman.

        :param lane: The lane object to resolve K for
        :return Optional[float]: the resolved K, or None
        """
        k = self._get_lane_k(lane)
        if k is None:
            k = self._read_k_from_spoolman(lane)
            if k is not None:
                self._set_lane_k(lane, k)
        return k

    def _calibrate(self, cur_lane: Any, gcmd: AFCGcmd,
                   runner: Optional[Callable] = None) -> Optional[float]:
        """
        Run the calibration command, then store/apply/persist the new K.

        :param cur_lane: The lane to calibrate
        :param gcmd: The gcode command the calibration runs through and answers on
        :param runner: Callable used to run ``calibrate_gcode``
        :return Optional[float]: the measured K, or None if none produced
        """
        flow_cal = self.printer.lookup_object('flow_calibrator', None)
        if flow_cal is None:
            msg = "AFC_autocal: flow_calibrator not found"
            if gcmd == "callback":
                self.logger.warning(msg)
            else:
                gcmd.respond_info(msg)
            return None
        # flow_calibrator keys _current_k by the Klipper extruder name, which differs from the
        # AFC_extruder name when `extruder_name` renames it.
        ext_obj = cur_lane.extruder_obj
        ext_name = getattr(ext_obj, 'th_extruder_name', None) or ext_obj.name
        k_before = flow_cal._current_k.get(ext_name)
        run = runner if runner is not None else (lambda s: self.afc.run_script(s, gcmd))
        run(self.calibrate_gcode)
        k_after = flow_cal._current_k.get(ext_name)
        if (k_after is None
            or k_after == k_before):
            self.logger.info(f"{cur_lane.name}: flow calibration gave no new K")
            return None
        self._set_lane_k(cur_lane, k_after)
        self._apply_lane_k(cur_lane.name)
        self._write_k_to_spoolman(cur_lane, k_after)
        self.logger.info(f"{cur_lane.name}: flow K {k_after:.4f} stored")
        return k_after

    def _current_lane(self) -> Optional[Any]:
        """
        Return the currently loaded lane object, or None.

        :return Optional[Any]: the current lane, or None
        """
        if self.afc is None:
            return None
        try:
            return self.afc.function.get_current_lane_obj()
        except Exception:
            return None

    def _handle_tool_loaded(self, cur_lane: Any) -> None:
        """
        Fired whenever a lane's tool is loaded.

        :param cur_lane: The lane object that was just loaded
        """
        if (self.afc is None
            or cur_lane is None):
            return
        if not (self.apply_stored_k or self._can_calibrate()):
            return
        # The gcode mutex is held mid-toolchange, so defer to the reactor where _do_tool_loaded can
        # use run_script once the load releases it.
        self.reactor.register_callback(
            lambda et, lane=cur_lane: self._do_tool_loaded(lane, "callback"))

    def _do_tool_loaded(self, cur_lane: Any,
                        gcmd: AFCGcmd) -> None:
        """
        Deferred body of the tool-loaded handler.

        :param cur_lane: The lane object that was just loaded
        :param gcmd: command context a calibration runs through
        """
        try:
            if (self.afc is None
                or cur_lane is None):
                return
            if not (self.apply_stored_k or self._can_calibrate()):
                return
            # A cached K needs no I/O, so apply it now subject to the mode's apply gate.
            if self._get_lane_k(cur_lane) is not None:
                if (self.apply_stored_k
                    and self._apply_gate_ok(cur_lane)):
                    self._apply_lane_k(cur_lane.name)
                return
            # Uncached: the Spoolman read is blocking HTTP, so run it off the reactor to avoid
            # "Timer too close".
            self._fetch_k_async(cur_lane, gcmd)
        except Exception as e:
            self.logger.warning(f"AFC_autocal: tool_loaded error: {e}")

    def _handle_spool_assigned(self, cur_lane: Any) -> None:
        """
        Fired when a spool is assigned to a lane.

        :param cur_lane: The lane the spool was assigned to
        """
        if (self.afc is None
            or cur_lane is None
            or not self._can_calibrate()):
            return
        # set_spoolID may run with the gcode mutex held, defer to the reactor.
        self.reactor.register_callback(lambda et, lane=cur_lane: self._do_spool_assigned(lane))

    def _do_spool_assigned(self, cur_lane: Any, attempts: int = 0) -> None:
        """
        Auto-load a staged, uncalibrated lane so tool_loaded calibrates it.

        :param cur_lane: The lane the spool was assigned to
        :param attempts: Retry counter for the bounded idle-wait chain
        """
        try:
            if (self.afc is None
                or cur_lane is None
                or not self._can_calibrate()):
                return
            name = cur_lane.name
            sid = self._norm_spool_id(getattr(cur_lane, 'spool_id', None))
            if sid is None:
                self._staged_handled.pop(name, None)
                self._staged_pending.discard(name)
                return
            if getattr(cur_lane, 'tool_loaded', False):
                self._staged_pending.discard(name)
                return
            if not getattr(cur_lane, 'load_state', False):
                self._staged_pending.discard(name)
                return
            if self._staged_handled.get(name) == sid:
                return
            if (attempts == 0
                and name in self._staged_pending):
                return
            # Don't auto-load spools already staged at boot.
            now = self.reactor.monotonic()
            if (self._ready_time is not None
                and (now - self._ready_time) < self._startup_cal_grace):
                self._staged_pending.discard(name)
                return
            # The assignment often fires mid staging-load, so wait (bounded) for idle and for any
            # extruder load in flight, which becomes tool_loaded itself.
            if (not self._safe_to_calibrate()
                or self._extruder_load_in_flight(cur_lane)):
                if attempts < 30:
                    self._staged_pending.add(name)
                    self.reactor.register_callback(
                        lambda et, ln=cur_lane, a=attempts + 1:
                            self._do_spool_assigned(ln, a), now + 1.0)
                else:
                    self._staged_pending.discard(name)
                return
            self._staged_pending.discard(name)
            self._staged_handled[name] = sid
            # The K read is a blocking HTTP call, so do it off the reactor.
            self._check_staged_k_async(name, sid)
        except Exception as e:
            self.logger.warning(f"AFC_autocal: spool_assigned error: {e}")

    def _check_staged_k_async(self, lane_name: str, sid: int) -> None:
        """
        Read a staged lane's K off-thread, then decide on the reactor.

        :param lane_name: The name of the staged lane
        :param sid: The normalized spool id being checked
        """
        def worker() -> None:
            """
            Background: read the staged spool's K, then hop back to the reactor.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            k = None
            read_ok = False
            try:
                client = self._spoolman()
                if client is not None:
                    k = client.read_flow_k(sid)
                    read_ok = True
            except Exception as e:
                self.logger.debug(f"AFC_autocal: staged K read failed: {e}")
            finally:
                self.reactor.register_async_callback(
                    lambda et: self._staged_k_ready(lane_name, sid, k, read_ok, "callback"))

        threading.Thread(target=worker, name="afc_autocal_stg", daemon=True).start()

    def _staged_k_ready(self, lane_name: str, sid: int, k: Optional[float], read_ok: bool,
                        gcmd: AFCGcmd) -> None:
        """
        Runs on the reactor with the staged spool's K.

        :param lane_name: The name of the staged lane
        :param sid: The normalized spool id the read was for
        :param k: The K read from Spoolman, or None
        :param read_ok: Whether the Spoolman read completed successfully
        :param gcmd: command context the auto-load runs through
        """
        try:
            lane = self.afc.lanes.get(lane_name) if self.afc else None
            if lane is None:
                return
            if self._norm_spool_id(getattr(lane, 'spool_id', None)) != sid:
                return
            if getattr(lane, 'tool_loaded', False):
                return
            if k is not None:
                self._set_lane_k(lane, k)
                self.logger.debug(f"AFC autocal: {lane_name} spool {sid} already has "
                                 f"K={k:.6f}, not auto-loading")
                return
            if not read_ok:
                self.logger.debug(f"AFC autocal: {lane_name} spool {sid} K unknown (Spoolman "
                                 f"read failed), not auto-loading")
                return  # couldn't confirm there's no K, be conservative
            if not self._safe_to_calibrate():
                return
            self.logger.info(f"{lane_name}: no flow K for this spool, loading it to measure one")
            self.afc.run_script(f"CHANGE_TOOL LANE={lane_name}", gcmd)
        except Exception as e:
            self.logger.warning(f"AFC_autocal: staged load error: {e}")

    def _fetch_k_async(self, cur_lane: Any, gcmd: AFCGcmd) -> None:
        """
        Read this lane's K from Spoolman in a worker thread, then hand the result to the reactor.

        :param cur_lane: The lane whose K to fetch
        :param gcmd: command context a calibration started here runs through
        """
        lane_name = cur_lane.name
        if lane_name in self._k_fetch_inflight:
            return
        sid = self._norm_spool_id(getattr(cur_lane, 'spool_id', None))
        if sid is None:
            # _calibrate_when_loaded waits for idle and skips during a print.
            self._apply_default_k(cur_lane)
            if self._can_calibrate():
                self._calibrate_when_loaded(cur_lane, gcmd)
            return
        if not self.apply_stored_k:
            # Only auto_calibrate is on, and that needs idle, defer to _k_applied with k=None so it
            # takes the (idle-gated) calibrate path.
            self._k_applied(lane_name, sid, None, gcmd)
            return

        self._k_fetch_inflight.add(lane_name)

        def worker() -> None:
            """
            Background: read the spool's K, then hop back to the reactor to apply it.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            k = None
            try:
                client = self._spoolman()
                if client is not None:
                    k = client.read_flow_k(sid)
            except Exception as e:
                self.logger.debug(f"AFC_autocal: async K read failed: {e}")
            finally:
                # Hop back onto the reactor.
                self.reactor.register_async_callback(
                    lambda et: self._k_applied(lane_name, sid, k, "callback"))

        threading.Thread(target=worker, name="afc_autocal_k", daemon=True).start()

    def _k_applied(self, lane_name: str, sid: int, k: Optional[float],
                   gcmd: AFCGcmd) -> None:
        """
        Runs on the reactor with the K read off-thread.

        :param lane_name: The name of the lane the read was for
        :param sid: The normalized spool id the read was for
        :param k: The K read from Spoolman, or None
        :param gcmd: command context a calibration started here runs through
        """
        self._k_fetch_inflight.discard(lane_name)
        try:
            lane = self.afc.lanes.get(lane_name) if self.afc else None
            if lane is None:
                return
            if self._norm_spool_id(getattr(lane, 'spool_id', None)) != sid:
                return
            if k is not None:
                self._set_lane_k(lane, k)
                # Cache it regardless, but only apply per the mode's gate (U1: lane's tool mounted.
                if (self.apply_stored_k
                    and self._apply_gate_ok(lane)):
                    self._apply_lane_k(lane_name)
                return
            # No stored K: the default K, then optionally calibrate (stores + applies), gated to
            # prep-done, idle and past the startup grace.
            self._apply_default_k(lane)
            if self._can_calibrate():
                self._calibrate_when_loaded(lane, gcmd)
        except Exception as e:
            self.logger.warning(f"AFC_autocal: K apply error: {e}")

    def _extruder_load_in_flight(self, lane: Any) -> bool:
        """
        True while the lane's extruder is mid async-load.

        :param lane: The lane whose extruder to check
        :return bool: True if an extruder load is in flight
        """
        ext = getattr(lane, 'extruder_obj', None)
        return bool(getattr(ext, 'load_active', False))

    def _is_printing(self) -> bool:
        """
        True when AFC reports a print in progress.

        :return bool: True if printing
        """
        try:
            return bool(self.afc.function.is_printing())
        except Exception:
            return False

    def _calibrate_when_loaded(self, lane: Any, gcmd: AFCGcmd,
                               attempts: int = 0) -> None:
        """
        Run the flow calibration once the printer settles to idle after the load.

        :param lane: The lane to calibrate
        :param gcmd: command context the calibration runs through
        :param attempts: Retry counter for the bounded idle-wait chain
        """
        name = lane.name
        if attempts == 0:
            if name in self._cal_pending:
                return
            self._cal_pending.add(name)
        try:
            if self._is_printing():
                self.logger.debug(f"AFC autocal: {name} calibration skipped, printing")
                self._cal_pending.discard(name)
                return
            reason = self._cal_block_reason()
            if (reason == "prep not done"
                or (reason is not None and reason.startswith("within startup"))):
                self.logger.debug(f"AFC autocal: {name} calibration skipped, {reason}")
                self._cal_pending.discard(name)
                return
            # Wait (bounded, about 4 min) for the tool change or extruder load to settle.
            if (reason is not None
                or self._extruder_load_in_flight(lane)):
                if attempts < 240:
                    self.reactor.register_callback(
                        lambda et, ln=lane, a=attempts + 1:
                            self._calibrate_when_loaded(ln, "callback", a),
                        self.reactor.monotonic() + 1.0)
                else:
                    self.logger.debug(f"AFC autocal: {name} calibration gave up waiting to "
                                     f"settle ({reason or 'load in flight'})")
                    self._cal_pending.discard(name)
                return
            self._cal_pending.discard(name)
            if not getattr(lane, 'tool_loaded', False):
                return
            # FLOW_CALIBRATE only measures the active toolhead extruder, so only calibrate when this
            # lane's tool is the one mounted.
            if not self._lane_on_active_toolhead(lane):
                try:
                    active = self.printer.lookup_object('toolhead').get_extruder().get_name()
                except Exception:
                    active = '?'
                self.logger.debug(f"AFC autocal: {name} calibration skipped, its tool is not "
                                 f"on the toolhead (active extruder={active}); pick up/load "
                                 f"this lane's tool to calibrate it")
                return
            self.logger.info(f"{name}: measuring flow K")
            self._calibrate(lane, gcmd)
        except Exception as e:
            self._cal_pending.discard(name)
            self.logger.warning(f"AFC_autocal: deferred calibrate error: {e}")

    def _active_extruder_name(self) -> Optional[str]:
        """
        Klipper name of the toolhead's active extruder.

        :return Optional[str]: the name, or None when it can't be read
        """
        try:
            active = self.printer.lookup_object('toolhead').get_extruder()
            return active.get_name() if active is not None else None
        except Exception:
            return None

    def _lane_on_active_toolhead(self, lane: Any) -> bool:
        """
        True when this lane's extruder is the one currently mounted.

        :param lane: The lane whose extruder to check
        :return bool: True if the lane's tool is on the toolhead
        """
        try:
            ext_obj = getattr(lane, 'extruder_obj', None)
            if ext_obj is None:
                return False
            active = self.printer.lookup_object('toolhead').get_extruder()
            if active is None:
                return False
            # An AFC_extruder section can be renamed (`extruder_name`), so match either name.
            return active.get_name() in (getattr(ext_obj, 'name', None),
                                         getattr(ext_obj, 'th_extruder_name', None))
        except Exception:
            return False

    def _safe_to_calibrate(self) -> bool:
        """
        True when there's no reason blocking a calibration right now.

        :return bool: True if safe to calibrate
        """
        return self._cal_block_reason() is None

    def _cal_block_reason(self) -> Optional[str]:
        """
        None when it's safe to calibrate, else a short reason string.

        :return Optional[str]: the blocking reason, or None
        """
        if not getattr(self.afc, 'prep_done', False):
            return "prep not done"
        if (self._ready_time is not None
            and ((self.reactor.monotonic() - self._ready_time) < self._startup_cal_grace)):
            return "within startup grace"
        state = getattr(self.afc, 'current_state', None)
        if (state is not None
            and str(state).split('.')[-1].lower() != 'idle'):
            return f"state={str(state).split('.')[-1]} (not idle)"
        return None

    def _reapply_current_k(self) -> None:
        """
        Re-apply the current lane's cached K on the active toolhead.
        """
        if (not self.apply_stored_k
            or self.afc is None):
            return
        if not getattr(self.afc, 'prep_done', False):
            return
        state = getattr(self.afc, 'current_state', None)
        if (state is not None
            and str(state).split('.')[-1].lower() != 'idle'):
            return
        cur_lane = self._current_lane()
        if cur_lane is None:
            return
        # Only the cached K: no Spoolman read on this event path.
        if self._get_lane_k(cur_lane) is not None:
            self._apply_lane_k(cur_lane.name)

    def _handle_home_rails_end(self, homing_state: Any, rails: Any) -> None:
        """
        Re-applies the current lane's stored K after homing finishes, since a home can reset
        pressure advance on the toolhead.

        :param homing_state: The homing state object from the event
        :param rails: The rails that were just homed
        """
        try:
            self._reapply_current_k()
        except Exception as e:
            self.logger.warning(f"AFC_autocal: home reapply error: {e}")

    def _handle_activate_extruder(self) -> None:
        """
        Re-applies the current lane's stored K whenever an extruder is activated, so an off-shuttle
        lane's cached K is restored once its tool is mounted.
        """
        try:
            self._reapply_current_k()
        except Exception as e:
            self.logger.warning(f"AFC_autocal: activate reapply error: {e}")

    def cmd_APPLY_LANE_FLOW_K(self, gcmd: GCodeCommand) -> None:
        """
        Manual command that applies the stored flow K for the current lane, pulling it from the
        cache or Spoolman first.
        """
        cur_lane = self._current_lane()
        if cur_lane is None:
            gcmd.respond_info("AFC_autocal: no current lane")
            return
        # U1 mode applies to the active extruder, so refuse when this lane's tool is docked.
        if not self._apply_gate_ok(cur_lane):
            gcmd.respond_info(f"AFC_autocal: {cur_lane.name}'s tool is not on the toolhead, "
                              f"pick up/load its tool first")
            return
        if self._ensure_k_loaded(cur_lane) is None:
            gcmd.respond_info(f"AFC_autocal: no stored K for {cur_lane.name}")
            return
        msg = self._apply_lane_k(cur_lane.name)
        if msg is None:
            # The K is cached, so a None here means the lane's extruder could not take it (missing,
            # or no stepper).
            gcmd.respond_info(f"AFC_autocal: K for {cur_lane.name} was not applied, see the log")
            return
        gcmd.respond_info(msg)

    def cmd_CALIBRATE_LANE_FLOW_K(self, gcmd: GCodeCommand) -> None:
        """
        Manual command that runs a flow calibration on the current lane and stores the resulting K
        on its Spoolman spool.
        """
        cur_lane = self._current_lane()
        if cur_lane is None:
            gcmd.respond_info("AFC_autocal: no current lane")
            return
        k = self._calibrate(cur_lane, gcmd)
        if k is not None:
            gcmd.respond_info(f"AFC_autocal: stored K={k:.6f} for {cur_lane.name}")
        else:
            gcmd.respond_info("AFC_autocal: calibration produced no new K")


def load_config(config: ConfigWrapper) -> AFC_autocal:
    """
    Klipper entry point that builds the AFC_autocal module from its config section.

    :param config: The Klipper config object for the [AFC_autocal] section
    :return AFC_autocal: The configured module instance
    """
    return AFC_autocal(config)
