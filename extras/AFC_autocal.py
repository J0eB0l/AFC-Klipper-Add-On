# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC AutoCal, per-spool flow calibration (K), read + apply + (optionally)
# calibrate. One module, two modes, picked automatically by whether a
# ``flow_calibrator`` object is present:
#
#   U1 mode (flow_calibrator present), the printer that measures K:
#     applies K through the flow_calibrator on the active toolhead and, when
#     auto_calibrate is on, calibrates spools that have none and stores the
#     result. The U1 handles its own slicer pressure-advance blocking, so this
#     module does not wrap SET_PRESSURE_ADVANCE here.
#
#   Consumer mode (no flow_calibrator), any other printer that consumes K:
#     looks up a spool's stored K and applies it directly on that lane's
#     extruder stepper, and wraps the per-extruder SET_PRESSURE_ADVANCE handler
#     so a slicer's mid-print PA change can't clobber the calibrated value. It
#     never calibrates (there's nothing to measure with).
#
# Decoupled from any RFID reader: it keys purely off a lane's spool_id, so it
# works no matter how the spool was identified (RFID, scanner, or manual
# SET_SPOOL_ID). K is persisted per-spool in a single 'flow_k' Spoolman extra
# field (created lazily on first write) and read/written via the shared
# SpoolmanClient, so a spool the U1 calibrates is found, unchanged, by every
# consumer printer sharing that Spoolman.
#
# The same two toggles apply on every printer, U1 or consumer:
#   apply_stored_k -> if the spool has a stored K, apply it (and re-apply after
#                     homing / extruder activation).
#   auto_calibrate -> U1 only (needs a flow_calibrator): if the spool has NO
#                     stored K, run a calibration (``calibrate_gcode``, default
#                     FLOW_CALIBRATE) and store it. A no-op in consumer mode.
# Use apply_stored_k alone to apply saved K without ever auto-calibrating.
#
# ── Configuration ───────────────────────────────────────────────────
#   [AFC_autocal]
#   apply_stored_k: True              # apply a spool's saved K on load
#   auto_calibrate: False             # U1 only: calibrate + store when no K
#   # enabled: True                   # back-compat master: turns both on
#   calibrate_gcode: FLOW_CALIBRATE   # command run to measure K (default)
# On a toolchanger this module is also auto-loaded (no section needed), but it
# stays dormant until apply_stored_k is set, the same knob used on the U1.

from __future__ import annotations
import inspect
import threading
import chelper
from typing import Any, Callable, Dict, Optional, Set, Tuple, TYPE_CHECKING

from extras.AFC_RFID import SpoolmanClient

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC_lane import AFCLane
    from extras.AFC_spool import AFCSpool


class AFC_autocal:
    """Per-spool flow calibration (K): applies a spool's stored K on load and
    optionally auto-calibrates spools that have none, persisting the result to
    the spool's 'flow_k' Spoolman extra field."""

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Sets up the autocal module: wires up the two toggles (apply_stored_k and
        auto_calibrate, or the ``enabled`` back-compat master that turns both on),
        registers the AFC/homing event handlers, and adds the two manual gcode
        commands.

        :param config: The Klipper config object for the [AFC_autocal] section
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        # load_object constructs AFC if needed, as AFC_lane/AFC_buffer do. self.afc
        # stays None until klippy:ready on purpose; the guards use it as the ready marker.
        self.logger = self.printer.load_object(config, 'AFC').logger
        self.afc: Optional[Any] = None
        self._lane_flow_k: Dict[str, Tuple[Optional[int], float]] = {}
        self._k_fetch_inflight: Set[str] = set()   # lanes with a pending read
        self._staged_handled: Dict[str, int] = {}   # lane_name -> staged spool_id
        self._staged_pending: Set[str] = set()   # lanes with a retry awaiting idle
        self._cal_pending: Set[str] = set()   # lanes with calibrate-when-idle
        # Consumer mode (no flow_calibrator): extruders we've applied K to, and
        # extruders whose SET_PRESSURE_ADVANCE handler we've wrapped.
        self._managed_extruders: Set[str] = set()
        self._wrapped_extruders: Set[str] = set()

        # Two independent toggles. 'enabled' is a back-compat master that
        # defaults both on when set.
        #   apply_stored_k - apply a spool's saved K on load + re-apply on homing
        #   auto_calibrate - if a loaded spool has NO saved K, calibrate & store
        master = config.getboolean('enabled', None)
        dflt = master if master is not None else False
        self.apply_stored_k = config.getboolean('apply_stored_k', dflt)
        self.auto_calibrate = config.getboolean('auto_calibrate', dflt)
        self.calibrate_gcode = config.get('calibrate_gcode', 'FLOW_CALIBRATE')
        # Suppress auto-calibration for this many seconds after klippy:ready so
        # the startup prep reconcile (which fires once prep_done flips) doesn't
        # kick off a calibration; genuine spool inserts later still calibrate.
        self._startup_cal_grace = config.getfloat('startup_cal_grace', 30.0)
        self._ready_time: Optional[float] = None

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('afc:tool_loaded',
                                            self._handle_tool_loaded)
        self.printer.register_event_handler('afc:spool_assigned',
                                            self._handle_spool_assigned)
        self.printer.register_event_handler('homing:home_rails_end',
                                            self._handle_home_rails_end)
        self.printer.register_event_handler('extruder:activate_extruder',
                                            self._handle_activate_extruder)

        self.gcode.register_command(
            'AFC_APPLY_LANE_FLOW_K', self.cmd_APPLY_LANE_FLOW_K,
            desc="Apply stored flow K for the current lane")
        self.gcode.register_command(
            'AFC_CALIBRATE_LANE_FLOW_K', self.cmd_CALIBRATE_LANE_FLOW_K,
            desc="Run flow calibration on the current lane and store K")

    # ── Lifecycle ───────────────────────────────────────────────────

    def _handle_ready(self) -> None:
        """
        Runs once Klipper is ready. Grabs the AFC object and, when a toggle is on,
        patches the lane/spool paths so ordinary loads and spool assignments
        actually reach this module.
        """
        self._ready_time = self.reactor.monotonic()
        self.afc = self.printer.lookup_object('AFC', None)
        if self.afc is None:
            self.logger.warning("AFC_autocal: AFC not loaded; disabled")
            return
        if self.apply_stored_k or self.auto_calibrate:
            self._patch_set_tool_loaded_emit()
        if self._can_calibrate():
            self._patch_set_spoolid_emit()
        # Consumer mode only: guard the calibrated PA against slicer overrides
        # and pre-load stored K for opted-in lanes (the U1 does both itself).
        if self._flow_calibrator() is None:
            self._wrap_pa_handlers()
            self._load_all_spoolman_k()

    def _patch_set_tool_loaded_emit(self) -> None:
        """
        Make every normal toolchange emit ``afc:tool_loaded``.

        Upstream ``AFCLane.set_tool_loaded()`` doesn't fire the event; only the
        ACE/OpenAMS/U1 units emit it, so a plain lane load never triggers autocal.
        Patch the lane base class here so upstream files stay untouched. The patch
        is idempotent (guarded by a class flag).

        The wrapper forwards ``*args``/``**kwargs`` verbatim so an upstream
        signature change cannot make it raise TypeError.
        """
        try:
            from extras.AFC_lane import AFCLane
        except Exception as e:
            self.logger.warning(
                f"AFC_autocal: cannot patch set_tool_loaded: {e}")
            return
        if getattr(AFCLane, '_afc_autocal_emit_patched', False):
            return
        _orig = AFCLane.set_tool_loaded

        def set_tool_loaded(self: AFCLane, *args: Any,
                            _orig: Callable = _orig, **kwargs: Any) -> None:
            """
            Wrapped AFCLane.set_tool_loaded: run the original, then announce the load.

            :param _orig: the original method
            :param args: forwarded to the original
            :param kwargs: forwarded to the original
            """
            _orig(self, *args, **kwargs)
            # Emit on every load, not just normal_toolchange: the direct
            # "load to extruder" path (AFC_extruder.temp_check_cb) and the
            # OpenAMS load paths call set_tool_loaded() with the default
            # (normal_toolchange=False), and those loads need autocal too.
            try:
                self.printer.send_event("afc:tool_loaded", self)
            except Exception:
                pass

        AFCLane.set_tool_loaded = set_tool_loaded
        AFCLane._afc_autocal_emit_patched = True
        self.logger.info(
            "AFC_autocal: set_tool_loaded now emits afc:tool_loaded on load")

    def _patch_set_spoolid_emit(self) -> None:
        """
        Make spool assignment emit ``afc:spool_assigned``.

        A spool staged into a lane is assigned via ``AFCSpool.set_spoolID`` but
        isn't loaded to the toolhead, so it never fires ``afc:tool_loaded`` and
        autocal has nothing to hook. Patch set_spoolID here (idempotent) to emit
        the lane on every call; _handle_spool_assigned then auto-loads an
        uncalibrated staged lane so the normal tool_loaded path calibrates it.
        Heavy gating in the handler keeps startup reconcile / clears from firing.

        The wrapper forwards arguments rather than restating upstream's signature,
        which previously raised TypeError on ``on_done=``. It binds against the real
        signature and chains the event onto ``on_done``, because set_spoolID returns
        before the async Spoolman fetch sets ``cur_lane.spool_id``. Upstream's
        ``finally`` runs on_done on every exit path, so the event cannot be lost; a
        caller's own on_done runs first and still propagates its exceptions.
        """
        try:
            from extras.AFC_spool import AFCSpool
        except Exception as e:
            self.logger.warning(
                f"AFC_autocal: cannot patch set_spoolID: {e}")
            return
        if getattr(AFCSpool, '_afc_autocal_spoolid_patched', False):
            return
        _orig = AFCSpool.set_spoolID
        try:
            _sig: Optional[inspect.Signature] = inspect.signature(_orig)
        except (TypeError, ValueError):
            _sig = None
        # Whether upstream has on_done to chain onto; without it, emit on return.
        _chainable = _sig is not None and 'on_done' in _sig.parameters

        def set_spoolID(self: AFCSpool, *args: Any, _orig: Callable = _orig,
                        _sig: Optional[inspect.Signature] = _sig,
                        **kwargs: Any) -> None:
            """
            Wrapped AFCSpool.set_spoolID: run the original, then announce the assignment.

            :param _orig: the original method
            :param _sig: the original's signature, for resolving the lane argument
            :param args: forwarded to the original
            :param kwargs: forwarded to the original
            """
            def emit(cur_lane: Any) -> None:
                """
                Announce the assignment; never let a handler break the call.

                :param cur_lane: the lane the spool was assigned to
                """
                try:
                    self.printer.send_event("afc:spool_assigned", cur_lane)
                except Exception:
                    pass

            if not _chainable:
                _orig(self, *args, **kwargs)
                emit(args[0] if args else kwargs.get('cur_lane'))
                return
            try:
                bound = _sig.bind(self, *args, **kwargs)
            except TypeError:
                # Not a call upstream accepts either. Let IT raise the error, so
                # the traceback names the real signature and not this wrapper.
                _orig(self, *args, **kwargs)
                return
            cur_lane = bound.arguments.get('cur_lane')
            caller_done = bound.arguments.get('on_done')

            def chained() -> None:
                """
                Caller's completion first, then ours, which runs regardless.
                """
                try:
                    if caller_done is not None:
                        caller_done()
                finally:
                    emit(cur_lane)

            bound.arguments['on_done'] = chained
            _orig(*bound.args, **bound.kwargs)

        AFCSpool.set_spoolID = set_spoolID
        AFCSpool._afc_autocal_spoolid_patched = True
        self.logger.info(
            "AFC_autocal: set_spoolID now emits afc:spool_assigned")

    def _spoolman(self) -> Optional[SpoolmanClient]:
        """
        Build a SpoolmanClient from the AFC moonraker handle.

        :return Optional[SpoolmanClient]: the client, or None if unavailable
        """
        mr = getattr(self.afc, 'moonraker', None) if self.afc else None
        if mr is None or getattr(self.afc, 'spoolman', None) is None:
            return None
        return SpoolmanClient(mr)

    # ── Mode detection (U1 vs consumer) ─────────────────────────────

    def _flow_calibrator(self) -> Optional[Any]:
        """
        Return the flow_calibrator object, or None on a consumer printer.

        Its presence is the mode signal: present -> U1 (measures + applies via
        the calibrator); absent -> consumer (applies on the extruder stepper).

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

        U1 mode targets the ACTIVE toolhead extruder, so we only apply when the
        lane's tool is mounted. Consumer mode addresses the lane's own extruder by
        name, so it can apply regardless of which tool is active.

        :param lane: The lane object to check
        :return bool: True if K may be applied for this lane now
        """
        if self._flow_calibrator() is None:
            return True
        return self._lane_on_active_toolhead(lane)

    # ── K cache (spool-validated) ───────────────────────────────────

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
        self._lane_flow_k[lane.name] = (
            self._norm_spool_id(getattr(lane, 'spool_id', None)), k)

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

    # ── Spoolman read/write (via the shared client) ─────────────────

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

    # ── Apply via flow_calibrator ───────────────────────────────────

    def _apply_lane_k(self, lane_name: str) -> Optional[str]:
        """
        Apply a lane's cached K, dispatching on the printer mode.

        U1 mode applies through flow_calibrator on the active toolhead extruder;
        consumer mode applies directly on the lane's own extruder stepper.

        :param lane_name: The name of the lane whose K to apply
        :return Optional[str]: a status message, or None if nothing was applied
        """
        entry = self._lane_flow_k.get(lane_name)
        if entry is None:
            return None
        _, k = entry
        flow_cal = self._flow_calibrator()
        if flow_cal is None:
            return self._apply_k_stepper(lane_name, k)
        ext = self.printer.lookup_object('toolhead').get_extruder()
        ext_name = ext.get_name()
        flow_cal._set_pressure_advance(ext, k)
        flow_cal._current_k[ext_name] = k
        msg = f"AFC autocal: applied K={k:.6f} for {lane_name} on {ext_name}"
        self.logger.info(msg)
        return msg

    def _apply_k_stepper(self, lane_name: str, k: float) -> Optional[str]:
        """
        Consumer-mode apply: set pressure advance on the lane's extruder stepper.

        Addresses the lane's own extruder by name (not the active toolhead), so a
        toolchanger applies K to the right extruder even when another tool is
        mounted, and records it in ``_managed_extruders`` so the wrapped
        SET_PRESSURE_ADVANCE handler blocks slicer overrides for it mid-print.

        :param lane_name: The name of the lane whose K to apply
        :param k: The flow K value to apply
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
            self.logger.warning(
                f"AFC autocal: extruder {ext_name} has no extruder_stepper")
            return None
        smooth_time = getattr(estepper, 'config_smooth_time',
                              estepper.pressure_advance_smooth_time)
        estepper._set_pressure_advance(k, smooth_time)
        self._managed_extruders.add(ext_name)
        msg = f"AFC autocal: applied K={k:.6f} for {lane_name} on {ext_name}"
        self.logger.info(msg)
        return msg

    # ── Consumer-mode: block slicer PA overrides, startup preload ───

    def _wrap_pa_handlers(self) -> None:
        """
        Wrap each per-extruder SET_PRESSURE_ADVANCE handler (consumer mode).

        Once a lane's K is applied, a slicer PA change mid-print would silently
        undo it; the wrapper swallows such changes for managed extruders while
        printing and passes everything else through. Idempotent per extruder.
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
        :param name: The extruder name this handler serves
        :return Callable: the wrapping handler
        """
        def wrapper(gcmd: GCodeCommand) -> None:
            """
            Ignore slicer PA for a managed extruder while printing, else run the original.

            :param gcmd: the gcode command
            """
            if (name in self._managed_extruders
                and self.afc is not None
                and self._is_printing()):
                self.logger.info(
                    f"AFC autocal: slicer PA change ignored for {name} "
                    f"(flow K managed)")
                gcmd.respond_info(
                    "AFC flow K active, slicer pressure advance ignored")
                return
            original(gcmd)
        return wrapper

    def _load_all_spoolman_k(self) -> None:
        """
        Pre-load stored K for all lanes at startup (consumer mode).

        A printer already loaded at boot emits no tool_loaded event, so seed the
        cache; the reapply-on-activate/home handlers then put the value on the
        extruder. No-op unless apply_stored_k is set.
        """
        if not self.apply_stored_k or self.afc is None:
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
                self.logger.debug(
                    f"AFC autocal: startup K load failed for {lane.name}: {e}")

    def _ensure_k_loaded(self, lane: Any) -> Optional[float]:
        """
        Return K for a lane: cached, else from Spoolman. None if neither.

        :param lane: The lane object to resolve K for
        :return Optional[float]: the resolved K, or None
        """
        k = self._get_lane_k(lane)
        if k is None:
            k = self._read_k_from_spoolman(lane)
            if k is not None:
                self._set_lane_k(lane, k)
        return k

    def _calibrate(self, cur_lane: Any, gcmd: Optional[GCodeCommand] = None,
                   runner: Optional[Callable] = None) -> Optional[float]:
        """
        Run the calibration command, then store/apply/persist the new K.

        :param cur_lane: The lane to calibrate
        :param gcmd: The gcode command for responses, or None
        :param runner: Callable used to run ``calibrate_gcode``. Defaults to
          ``run_script_from_command`` (assumes the gcode mutex is held, i.e.
          we're inside a command). Pass ``self.gcode.run_script`` when calling
          from a deferred/reactor context so the mutex is acquired safely.
        :return Optional[float]: the measured K, or None if none produced
        """
        flow_cal = self.printer.lookup_object('flow_calibrator', None)
        if flow_cal is None:
            msg = "AFC_autocal: flow_calibrator not found"
            if gcmd is not None:
                gcmd.respond_info(msg)
            else:
                self.logger.warning(msg)
            return None
        # flow_calibrator keys _current_k by the Klipper toolhead extruder name,
        # which differs from the AFC_extruder section name when the extruder is
        # renamed (v1.1.22 'extruder_name'). Use the Klipper name.
        ext_obj = cur_lane.extruder_obj
        ext_name = getattr(ext_obj, 'th_extruder_name', None) or ext_obj.name
        k_before = flow_cal._current_k.get(ext_name)
        run = runner if runner is not None else self.gcode.run_script_from_command
        run(self.calibrate_gcode)
        k_after = flow_cal._current_k.get(ext_name)
        if k_after is None or k_after == k_before:
            self.logger.info(
                f"AFC autocal: calibration produced no new K for {cur_lane.name}")
            return None
        self._set_lane_k(cur_lane, k_after)
        self._apply_lane_k(cur_lane.name)
        self._write_k_to_spoolman(cur_lane, k_after)
        self.logger.info(
            f"AFC autocal: calibrated and stored K={k_after:.6f} for {cur_lane.name}")
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

    # ── Event handlers ──────────────────────────────────────────────

    def _handle_tool_loaded(self, cur_lane: Any) -> None:
        """
        Fired whenever a lane's tool is loaded. This runs mid-toolchange with the
        gcode mutex held, so we just hand the real work off to the reactor.

        :param cur_lane: The lane object that was just loaded
        """
        if self.afc is None or cur_lane is None:
            return
        if not (self.apply_stored_k or self._can_calibrate()):
            return
        # The gcode mutex is held mid-toolchange, so defer to the reactor where
        # _do_tool_loaded can use run_script once the load releases it.
        self.reactor.register_callback(
            lambda et, lane=cur_lane: self._do_tool_loaded(lane))

    def _do_tool_loaded(self, cur_lane: Any) -> None:
        """
        Deferred body of the tool-loaded handler. Applies an already-cached K right
        away when the lane's tool is the active one, otherwise kicks off an
        off-thread Spoolman read to fetch (and maybe calibrate) it.

        :param cur_lane: The lane object that was just loaded
        """
        try:
            if self.afc is None or cur_lane is None:
                return
            if not (self.apply_stored_k or self._can_calibrate()):
                return
            # A cached K needs no I/O, so apply it now subject to the mode's apply
            # gate. An off-shuttle lane is re-applied when its tool activates.
            if self._get_lane_k(cur_lane) is not None:
                if self.apply_stored_k and self._apply_gate_ok(cur_lane):
                    self._apply_lane_k(cur_lane.name)
                return
            # Uncached: the Spoolman read is blocking HTTP, so run it off the reactor
            # to avoid "Timer too close"; _k_applied applies the result.
            self._fetch_k_async(cur_lane)
        except Exception as e:
            self.logger.warning(f"AFC_autocal: tool_loaded error: {e}")

    def _handle_spool_assigned(self, cur_lane: Any) -> None:
        """
        Fired when a spool is assigned to a lane. Like the tool-loaded handler this
        may run with the gcode mutex held, so we defer to the reactor.

        :param cur_lane: The lane the spool was assigned to
        """
        if self.afc is None or cur_lane is None or not self._can_calibrate():
            return
        # set_spoolID may run with the gcode mutex held, defer to the reactor.
        self.reactor.register_callback(
            lambda et, lane=cur_lane: self._do_spool_assigned(lane))

    def _do_spool_assigned(self, cur_lane: Any, attempts: int = 0) -> None:
        """
        Auto-load a staged, uncalibrated lane so tool_loaded calibrates it.

        If the lane is staged (filament present, has a spool, but isn't in the
        toolhead) and uncalibrated, auto-load it once it's safe. A lane already in
        the toolhead is handled by the tool_loaded path; a direct extruder
        auto-loading is skipped because it becomes tool_loaded (or is still
        load-in-flight) while we wait. Spools staged at boot are left alone.

        :param cur_lane: The lane the spool was assigned to
        :param attempts: Retry counter for the bounded idle-wait chain
        """
        try:
            if self.afc is None or cur_lane is None or not self._can_calibrate():
                return
            name = cur_lane.name
            sid = self._norm_spool_id(getattr(cur_lane, 'spool_id', None))
            if sid is None:
                # Spool cleared/removed, allow a later re-insert to retry.
                self._staged_handled.pop(name, None)
                self._staged_pending.discard(name)
                return
            if getattr(cur_lane, 'tool_loaded', False):
                self._staged_pending.discard(name)
                return  # in the toolhead, tool_loaded path handles it
            if not getattr(cur_lane, 'load_state', False):
                self._staged_pending.discard(name)
                return  # no filament present at the lane, nothing to load
            if self._staged_handled.get(name) == sid:
                return  # already auto-loaded this spool from staging
            if attempts == 0 and name in self._staged_pending:
                return  # a retry chain is already waiting for this lane
            # Don't auto-load spools already staged at boot: inside the startup
            # grace we neither act nor retry (genuine inserts happen later).
            now = self.reactor.monotonic()
            if (self._ready_time is not None
                and (now - self._ready_time) < self._startup_cal_grace):
                self._staged_pending.discard(name)
                return
            # The assignment often fires mid staging-load, so wait (bounded) for idle
            # and for any extruder load in flight, which becomes tool_loaded itself.
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
            # Only auto-load when it actually needs calibration (no stored K).
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
                    lambda et: self._staged_k_ready(lane_name, sid, k, read_ok))

        threading.Thread(target=worker, name="afc_autocal_stg",
                         daemon=True).start()

    def _staged_k_ready(self, lane_name: str, sid: int, k: Optional[float],
                        read_ok: bool) -> None:
        """
        Runs on the reactor with the staged spool's K.

        Auto-load (to calibrate) ONLY when we positively confirmed the spool has
        no stored K: a spool that already has a K, or one whose K we couldn't
        read, is left staged.

        :param lane_name: The name of the staged lane
        :param sid: The normalized spool id the read was for
        :param k: The K read from Spoolman, or None
        :param read_ok: Whether the Spoolman read completed successfully
        """
        try:
            lane = self.afc.lanes.get(lane_name) if self.afc else None
            if lane is None:
                return
            if self._norm_spool_id(getattr(lane, 'spool_id', None)) != sid:
                return  # spool changed since we kicked off the read
            if getattr(lane, 'tool_loaded', False):
                return  # got loaded in the meantime
            if k is not None:
                self._set_lane_k(lane, k)
                self.logger.info(
                    f"AFC autocal: {lane_name} spool {sid} already has "
                    f"K={k:.6f}, not auto-loading")
                return  # already calibrated, don't force a load
            if not read_ok:
                self.logger.info(
                    f"AFC autocal: {lane_name} spool {sid} K unknown (Spoolman "
                    f"read failed), not auto-loading")
                return  # couldn't confirm there's no K, be conservative
            if not self._safe_to_calibrate():
                return
            self.logger.info(
                f"AFC autocal: {lane_name} spool {sid} has no stored K, "
                f"loading to calibrate")
            self.gcode.run_script(f"CHANGE_TOOL LANE={lane_name}")
        except Exception as e:
            self.logger.warning(f"AFC_autocal: staged load error: {e}")

    def _fetch_k_async(self, cur_lane: Any) -> None:
        """
        Read this lane's K from Spoolman in a worker thread, then hand the result
        to the reactor. The worker only does the (stateless) HTTP read; all state
        changes / applies happen on the reactor in _k_applied.

        :param cur_lane: The lane whose K to fetch
        """
        lane_name = cur_lane.name
        if lane_name in self._k_fetch_inflight:
            return
        sid = self._norm_spool_id(getattr(cur_lane, 'spool_id', None))
        if sid is None:
            # No spool to look up; calibrate if possible (U1). _calibrate_when_loaded
            # waits for idle and skips during a print.
            if self._can_calibrate():
                self._calibrate_when_loaded(cur_lane)
            return
        if not self.apply_stored_k:
            # Only auto_calibrate is on, and that needs idle, defer to _k_applied
            # with k=None so it takes the (idle-gated) calibrate path.
            self._k_applied(lane_name, sid, None)
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
                # Hop back onto the reactor; in a finally so a worker crash can't
                # leave the lane stuck in _k_fetch_inflight.
                self.reactor.register_async_callback(
                    lambda et: self._k_applied(lane_name, sid, k))

        threading.Thread(target=worker, name="afc_autocal_k",
                         daemon=True).start()

    def _k_applied(self, lane_name: str, sid: int, k: Optional[float]) -> None:
        """
        Runs on the reactor with the K read off-thread. Re-validate the lane still
        carries that spool (it may have changed during the read), then apply the
        stored K or fall back to an idle-gated calibration.

        :param lane_name: The name of the lane the read was for
        :param sid: The normalized spool id the read was for
        :param k: The K read from Spoolman, or None
        """
        self._k_fetch_inflight.discard(lane_name)
        try:
            lane = self.afc.lanes.get(lane_name) if self.afc else None
            if lane is None:
                return
            if self._norm_spool_id(getattr(lane, 'spool_id', None)) != sid:
                return  # spool changed since we kicked off the read, stale
            if k is not None:
                self._set_lane_k(lane, k)
                # Cache it regardless, but only apply per the mode's gate (U1:
                # lane's tool mounted; consumer: always, on the lane's extruder).
                if self.apply_stored_k and self._apply_gate_ok(lane):
                    self._apply_lane_k(lane_name)
                return
            # No stored K, optionally calibrate (stores + applies), gated to prep-done,
            # idle and past the startup grace.
            if self._can_calibrate():
                self._calibrate_when_loaded(lane)
        except Exception as e:
            self.logger.warning(f"AFC_autocal: K apply error: {e}")

    def _extruder_load_in_flight(self, lane: Any) -> bool:
        """
        True while the lane's extruder is mid async-load.

        The U1 direct-load (AFC_extruder.move_extruder) schedules a deferred
        cleanup timer (extruder_move_cb) that calls flush_step_generation(); if a
        calibration's homing/drip move overlaps that cleanup, the flush raises
        DripModeEndSignal and shuts Klipper down. Non-U1 loads don't set this
        flag, so they calibrate immediately.

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

    def _calibrate_when_loaded(self, lane: Any, attempts: int = 0) -> None:
        """
        Run the flow calibration once the printer settles to idle after the load.

        The auto-load issues a full CHANGE_TOOL, so the AFC state stays non-idle
        ('Loading') for the whole sequence (~90s), wait (bounded) for it to finish
        rather than skipping on the first non-idle read. Never calibrate during a
        print, before prep, or in the startup grace, and wait out any in-flight
        extruder load so its cleanup can't overlap the calibration's homing/drip
        move (see _extruder_load_in_flight).

        :param lane: The lane to calibrate
        :param attempts: Retry counter for the bounded idle-wait chain
        """
        name = lane.name
        if attempts == 0:
            if name in self._cal_pending:
                return  # a calibrate chain is already running for this lane
            self._cal_pending.add(name)
        try:
            if self._is_printing():
                self.logger.info(
                    f"AFC autocal: {name} calibration skipped, printing")
                self._cal_pending.discard(name)
                return
            reason = self._cal_block_reason()
            # Hard stops we don't wait out: boot prep / startup grace.
            if (reason == "prep not done"
                or (reason is not None
                    and reason.startswith("within startup"))):
                self.logger.info(
                    f"AFC autocal: {name} calibration skipped, {reason}")
                self._cal_pending.discard(name)
                return
            # Transient: the tool change / load hasn't settled to idle yet, or an
            # extruder load is still in flight. Wait for it (bounded ~4min).
            if reason is not None or self._extruder_load_in_flight(lane):
                if attempts < 240:
                    self.reactor.register_callback(
                        lambda et, ln=lane, a=attempts + 1:
                            self._calibrate_when_loaded(ln, a),
                        self.reactor.monotonic() + 1.0)
                else:
                    self.logger.info(
                        f"AFC autocal: {name} calibration gave up waiting to "
                        f"settle ({reason or 'load in flight'})")
                    self._cal_pending.discard(name)
                return
            self._cal_pending.discard(name)
            if not getattr(lane, 'tool_loaded', False):
                return  # lane was unloaded while we waited
            # FLOW_CALIBRATE only measures the active toolhead extruder, so only
            # calibrate when this lane's tool is the one mounted.
            if not self._lane_on_active_toolhead(lane):
                try:
                    active = self.printer.lookup_object(
                        'toolhead').get_extruder().get_name()
                except Exception:
                    active = '?'
                self.logger.info(
                    f"AFC autocal: {name} calibration skipped, its tool is not "
                    f"on the toolhead (active extruder={active}); pick up/load "
                    f"this lane's tool to calibrate it")
                return
            self.logger.info(
                f"AFC autocal: running flow calibration for {name}")
            self._calibrate(lane, runner=self.gcode.run_script)
        except Exception as e:
            self._cal_pending.discard(name)
            self.logger.warning(f"AFC_autocal: deferred calibrate error: {e}")

    def _lane_on_active_toolhead(self, lane: Any) -> bool:
        """
        True when this lane's extruder is the one currently mounted.

        Applying stored K sets pressure advance on the active extruder, so we only
        apply a lane's K when its tool is actually mounted, never on a sibling tool
        on the shuttle (e.g. T0 during prep). An off-shuttle lane's cached K is
        re-applied when its tool is activated.

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
            # The toolhead reports the Klipper extruder name; an AFC_extruder
            # section can be renamed (v1.1.22 'extruder_name'), so match either
            # the section name or the Klipper name (th_extruder_name).
            return active.get_name() in (
                getattr(ext_obj, 'name', None),
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
            and ((self.reactor.monotonic() - self._ready_time)
                 < self._startup_cal_grace)):
            return "within startup grace"
        state = getattr(self.afc, 'current_state', None)
        if state is not None and str(state).split('.')[-1].lower() != 'idle':
            return f"state={str(state).split('.')[-1]} (not idle)"
        return None

    def _reapply_current_k(self) -> None:
        """
        Re-apply the current lane's cached K on the active toolhead.
        """
        # Re-applying only matters when we apply stored K.
        if not self.apply_stored_k or self.afc is None:
            return
        if not getattr(self.afc, 'prep_done', False):
            return
        state = getattr(self.afc, 'current_state', None)
        if state is not None and str(state).split('.')[-1].lower() != 'idle':
            return
        cur_lane = self._current_lane()
        if cur_lane is None:
            return
        # Re-apply only the already-cached K (no Spoolman read on this event
        # path). The K was cached at load; if it wasn't, the load's async fetch
        # applies it, no need to block here.
        if self._get_lane_k(cur_lane) is not None:
            self._apply_lane_k(cur_lane.name)

    def _handle_home_rails_end(self, homing_state: Any, rails: Any) -> None:
        """
        Re-applies the current lane's stored K after homing finishes, since a home
        can reset pressure advance on the toolhead.

        :param homing_state: The homing state object from the event
        :param rails: The rails that were just homed
        """
        try:
            self._reapply_current_k()
        except Exception as e:
            self.logger.warning(f"AFC_autocal: home reapply error: {e}")

    def _handle_activate_extruder(self) -> None:
        """
        Re-applies the current lane's stored K whenever an extruder is activated, so
        an off-shuttle lane's cached K is restored once its tool is mounted.
        """
        try:
            self._reapply_current_k()
        except Exception as e:
            self.logger.warning(f"AFC_autocal: activate reapply error: {e}")

    # ── GCode commands (manual overrides, work regardless of enabled) ─

    def cmd_APPLY_LANE_FLOW_K(self, gcmd: GCodeCommand) -> None:
        """
        Manual command that applies the stored flow K for the current lane, pulling
        it from the cache or Spoolman first. Works regardless of the toggles.

        Usage
        -------
        `AFC_APPLY_LANE_FLOW_K`

        Example
        -------
        ```
        AFC_APPLY_LANE_FLOW_K
        ```
        """
        cur_lane = self._current_lane()
        if cur_lane is None:
            gcmd.respond_info("AFC_autocal: no current lane")
            return
        # U1 mode applies to the active extruder, so refuse when this lane's tool
        # is docked; consumer mode addresses the lane's own extruder by name.
        if not self._apply_gate_ok(cur_lane):
            gcmd.respond_info(
                f"AFC_autocal: {cur_lane.name}'s tool is not on the toolhead, "
                f"pick up/load its tool first")
            return
        if self._ensure_k_loaded(cur_lane) is None:
            gcmd.respond_info(f"AFC_autocal: no stored K for {cur_lane.name}")
            return
        msg = self._apply_lane_k(cur_lane.name)
        if msg is None:
            # The K is cached, so a None here means the lane's extruder could not
            # take it (missing, or no stepper); _apply_k_stepper logged which.
            gcmd.respond_info(
                f"AFC_autocal: K for {cur_lane.name} was not applied, see the log")
            return
        gcmd.respond_info(msg)

    def cmd_CALIBRATE_LANE_FLOW_K(self, gcmd: GCodeCommand) -> None:
        """
        Manual command that runs a flow calibration on the current lane and stores
        the resulting K on its Spoolman spool. Works regardless of the toggles.

        Usage
        -------
        `AFC_CALIBRATE_LANE_FLOW_K`

        Example
        -------
        ```
        AFC_CALIBRATE_LANE_FLOW_K
        ```
        """
        cur_lane = self._current_lane()
        if cur_lane is None:
            gcmd.respond_info("AFC_autocal: no current lane")
            return
        k = self._calibrate(cur_lane, gcmd=gcmd)
        if k is not None:
            gcmd.respond_info(
                f"AFC_autocal: stored K={k:.6f} for {cur_lane.name}")
        else:
            gcmd.respond_info("AFC_autocal: calibration produced no new K")


def load_config(config: ConfigWrapper) -> AFC_autocal:
    """
    Klipper entry point that builds the AFC_autocal module from its config section.

    :param config: The Klipper config object for the [AFC_autocal] section
    :return AFC_autocal: The configured module instance
    """
    return AFC_autocal(config)
