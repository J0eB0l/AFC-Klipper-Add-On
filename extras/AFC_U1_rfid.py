# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# U1 RFID integration: reads spool data from Snapmaker U1's filament_detect
# Klipper module and syncs to AFC lanes / Spoolman.

from __future__ import annotations
import json
import os
import re
import time
from typing import TYPE_CHECKING, Any, Dict, Optional, Tuple

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from extras.AFC import afc
    from extras.AFC_lane import AFCLane

from extras.AFC_RFID import (
    build_filament_name,
    get_auto_spoolman_create, apply_filament_defaults,
    sync_rfid_to_spoolman, make_tag_record,
)
from extras.AFC_rfid_write import register_reader

POLL_INTERVAL = 2.0
#: NTAG page the tag record starts at. encode_tag_payload() lays the Anycubic
#: layout down from here (the first writable NTAG page); the U1's OpenRFID
#: reads it back through its own Anycubic processor.
_WRITE_START_PAGE = 4
#: How long a write may take end to end: the daemon has to pause its scan loop,
#: grab the reader, write ~150 pages and read them back. Roomy on purpose.
_WRITE_TIMEOUT_S = 45.0
_WRITE_POLL_STEP = 0.15
_MAX_CONSECUTIVE_FAILURES = 5
_BACKOFF_INTERVAL = 10.0
_BACKOFF_RESET_CYCLES = 18  # ~3 min at 10s intervals before retrying normal speed
_FORCE_READ_TIMEOUT = 1.0
_FORCE_READ_POLL_STEP = 0.05


class AFC_U1_RFID:
    """Polls the Snapmaker U1 filament_detect Klipper object for RFID tag data
    and applies it to AFC lanes (material, color) and Spoolman."""

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Configure the reader from its ``[AFC_U1_rfid]`` section.

        Parses lane->channel maps, standalone scanner channels, auto-create
        options and the OpenRFID webhook grace, registers the
        ``afc/u1_rfid`` webhook endpoint, and defers wiring to ``klippy:ready``.

        :param config: Klipper ConfigWrapper for the ``[AFC_U1_rfid]`` section.
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        # Take AFC's logger at construction (load_object builds AFC if needed)
        # so early messages reach AFC.log and the console.
        # self.afc stays None until klippy:ready: guards use it as the ready marker.
        self.logger = self.printer.load_object(config, 'AFC').logger
        self.afc: Optional["afc"] = None
        self._filament_detect: Optional[Any] = None
        self._lane_channel_map: Dict[str, int] = {}
        self._lane_objects: Dict[str, "AFCLane"] = {}
        self._last_uid: Dict[int, Optional[list]] = {}
        self._poll_timer: Optional[Any] = None
        self._scanner_channels: set = set()
        self._channel_to_lane: Dict[int, str] = {}
        self._consecutive_failures: Dict[int, int] = {}
        self._tag_reads: Dict[str, dict] = {}
        self._backed_off: bool = False
        self._backoff_cycles: int = 0
        self._fd_cb_registered: bool = False
        # The lane->channel map and scanner channels are configured in this
        # section, and the reader wires up its own polling:
        #   [AFC_U1_rfid]
        #   lane_channels: lane4:1, lane5:2, lane6:3   # tag -> assign to lane
        #   scanner_channels: 0                         # tag -> stage next spool
        #   scanner_auto_create: True   # opt-in (default False): create scanned
        #                               # spools in Spoolman when no match exists
        # lane_channels: the tag read on a channel is assigned to that lane.
        # (Alias: 'channels'.)
        self._cfg_channels: Dict[str, int] = {}        # lane_name -> channel
        lane_chan_str = config.get('lane_channels', None)
        if lane_chan_str is None:
            lane_chan_str = config.get('channels', '')
        for pair in lane_chan_str.split(','):
            pair = pair.strip()
            if not pair:
                continue
            name, sep, ch = pair.partition(':')
            if not sep:
                error_str = ("AFC_U1_rfid: 'lane_channels' entries must be "
                             f"'lane:channel', got '{pair}'")
                raise config.error(error_str)
            try:
                self._cfg_channels[name.strip()] = int(ch.strip())
            except ValueError:
                error_str = f"AFC_U1_rfid: bad channel number in '{pair}'"
                raise config.error(error_str)
        # scanner_channels: standalone spool-scanner channels with no lane. A
        # scan stages the spool as next_spool_id for whichever lane loads next.
        self._cfg_scanner_channels: set = set()
        for ch in config.get('scanner_channels', '').split(','):
            ch = ch.strip()
            if not ch:
                continue
            try:
                self._cfg_scanner_channels.add(int(ch))
            except ValueError:
                error_str = f"AFC_U1_rfid: bad scanner channel '{ch}'"
                raise config.error(error_str)
        # Auto-create scanned spools in Spoolman (opt-in). Scanner channels have
        # no lane, so the lane-based auto-create lookup does not apply.
        self._scanner_auto_create = config.getboolean(
            'scanner_auto_create', False)
        # Scanner channels only: require the same UID on N consecutive reads, so
        # one corrupt read cannot create a junk spool. Default 1 acts at once.
        self._scanner_confirm_reads = config.getint(
            'scanner_confirm_reads', 1, minval=1)
        self._pending_confirm: Dict[int, tuple] = {}  # channel -> (uid, count)
        # Default auto-create for lane reads; a lane's unit/extruder
        # auto_spoolman_create still overrides this.
        self._lane_auto_create = config.getboolean(
            'auto_spoolman_create', False)
        # scanner_lanes: a loadable lane that also acts as a scanner (rare).
        self._cfg_scanners: set = {s.strip() for s in
                                   config.get('scanner_lanes', '').split(',')
                                   if s.strip()}
        # OpenRFID full-colour webhook: filament_detect only carries the primary
        # colour, so a webhook POST to /printer/afc/u1_rfid supplies the full
        # list. The first webhook on a channel makes it authoritative there;
        # filament_detect still handles tag removal.
        self._webhook_channels_seen: set = set()
        # webhook_grace: seconds to defer a filament_detect read of a new tag so
        # the full-colour webhook can land first. Default 0; only set it (~1.0)
        # with the OpenRFID webhook exporter configured, or it just delays scans.
        self._webhook_grace = config.getfloat('webhook_grace', 0.0, minval=0.0)
        self._pending_defer: Dict[int, list] = {}  # channel -> uid awaiting grace
        # Where AFC_RFID_WRITE drops requests for the OpenRFID daemon; the
        # default is resolved lazily in _write_dir.
        self._cfg_write_dir = config.get('openrfid_write_dir', None)
        try:
            webhooks = self.printer.lookup_object('webhooks')
            webhooks.register_endpoint('afc/u1_rfid', self._handle_webhook_scan)
        except Exception as e:
            self.logger.warning(
                f"AFC_U1_rfid: failed to register webhook endpoint: {e}")
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self) -> None:
        """
        Resolve configured lanes against AFC and start polling.
        """
        self.afc = self.printer.lookup_object('AFC', None)
        if self.afc is None:
            self.logger.warning("AFC_U1_rfid: AFC not loaded; reader disabled")
            return
        for lane_name, channel in self._cfg_channels.items():
            lane, extruder = self._resolve_lane(lane_name)
            if lane is None and extruder is not None:
                # Several lanes share this extruder, so there is no single lane
                # to attribute a read to: act as a standalone scanner instead.
                self._cfg_scanner_channels.add(channel)
                n = len(getattr(extruder, 'lanes', {}))
                self.logger.info(
                    f"U1 RFID: '{lane_name}' is a combined extruder ({n} lanes), "
                    f"ch{channel} acts as a spool scanner (stages next spool)")
                continue
            if lane is None:
                self.logger.warning(
                    f"U1 RFID: configured lane '{lane_name}' not found in AFC "
                    f"(neither a lane name nor a single-lane extruder)")
                continue
            if lane_name in self._cfg_scanners:
                # Mark the lane so any scanner-aware code (incl. bridge hooks
                # that read lane.spool_scanner) sees it.
                try:
                    lane.spool_scanner = True
                except Exception:
                    pass
            self.register_lane(lane, channel)
        # Standalone scanner channels register with a None lane.
        for channel in self._cfg_scanner_channels:
            self._channel_to_lane[channel] = None
            self._last_uid[channel] = None
            self._consecutive_failures[channel] = 0
        self.start()
        self._patch_scanner_rfid_update()

    def _patch_scanner_rfid_update(self) -> None:
        """
        Stop a spool-scanner read from overwriting the U1 display (and
        resetting flow K) for the extruder the antenna sits on.

        The U1's native ``print_task_config._rfid_filament_info_update_cb``
        writes a scanned tag into print_task_config and runs FLOW_RESET_K,
        clobbering the loaded lane's filament. This suppresses it for standalone
        scanner channels. No-op on non-U1.
        """
        ptc = self.printer.lookup_object("print_task_config", None)
        fd = self.printer.lookup_object("filament_detect", None)
        if (ptc is None
            or fd is None
            or not hasattr(fd, "_notify_data_update_cb")):
            return
        original_cb = getattr(ptc, "_rfid_filament_info_update_cb", None)
        if original_cb is None:
            return
        scanner_channels = set(self._cfg_scanner_channels)
        if not scanner_channels:
            return

        def patched_rfid_cb(*args: Any, **kwargs: Any) -> None:
            """
            filament_detect callback wrapper that protects scanner channels.

            Suppresses the native print_task_config write for configured scanner
            channels (keeping the loaded lane's display + flow K); every other
            channel passes straight through to the original callback unchanged.

            The signature is deliberately generic: the U1 firmware calls this,
            and a TypeError from an added argument would shut Klipper down. Only
            the first argument (the channel) is read.

            :param args: positional arguments from filament_detect, forwarded.
            :param kwargs: keyword arguments from filament_detect, forwarded.
            """
            channel = args[0] if args else kwargs.get('channel')
            # Suppress the native write for scanner channels only.
            if isinstance(channel, int) and channel in scanner_channels:
                return
            original_cb(*args, **kwargs)

        for i, cb in enumerate(fd._notify_data_update_cb):
            if cb == original_cb:
                fd._notify_data_update_cb[i] = patched_rfid_cb
                self.logger.info(
                    "U1 RFID: protecting scanner channels %s from U1 display "
                    "overwrite" % sorted(scanner_channels))
                return
        self.logger.warning(
            "U1 RFID: could not locate print_task_config RFID callback to patch")

    def _resolve_lane(self, name: str) -> Tuple[Any, Any]:
        """
        Resolve a configured name to a lane or a combined extruder.

        Tries the AFC lane registry first, then the lane(s) driving an extruder
        of that name (AFC_extruder section name or th_extruder_name).

        Returns a ``(lane, extruder)`` pair:
          * ``(lane, None)``: resolved to a single AFC lane.
          * ``(None, ext)``: an extruder fed by multiple lanes; the caller
            treats the channel as a spool scanner.
          * ``(None, None)``: unresolved; the available names are logged.

        :param name: configured name, an AFC lane name or an extruder name.
        :return tuple: ``(lane, extruder)`` per the cases above.
        """
        lane = self.afc.lanes.get(name)
        if lane is not None:
            return lane, None

        def _ext_names(lane_obj: Any) -> set:
            """
            Return the set of extruder names associated with a lane.

            :param lane_obj: an AFC lane object.
            :return set: the lane's AFC_extruder section name and toolhead
                extruder name (th_extruder_name), for matching by either.
            """
            e = getattr(lane_obj, 'extruder_obj', None)
            return {getattr(e, 'name', None),
                    getattr(e, 'th_extruder_name', None)}

        matches = [ln for ln in self.afc.lanes.values()
                   if name in _ext_names(ln)]
        ext_obj = getattr(matches[0], 'extruder_obj', None) if matches else None
        # Also consult the extruder registry directly, in case a lane's
        # extruder_obj linkage isn't reflected in the scan above.
        if not matches:
            tools = getattr(self.afc, 'tools', {}) or {}
            ext = tools.get(name)
            if ext is None:
                for e in tools.values():
                    if name in {getattr(e, 'name', None),
                                getattr(e, 'th_extruder_name', None)}:
                        ext = e
                        break
            if ext is not None:
                ext_obj = ext
                matches = list(getattr(ext, 'lanes', {}).values())

        if len(matches) == 1:
            return matches[0], None
        if len(matches) > 1:
            # Several lanes share one extruder: signal the caller to treat the
            # channel as a spool scanner.
            return None, ext_obj

        # Nothing matched: log what is registered so the config can be fixed.
        # A standalone toolhead lane needs 'standalone: True' to appear here.
        avail_lanes = sorted(self.afc.lanes.keys())
        avail_ext = sorted(
            {n for ln in self.afc.lanes.values() for n in _ext_names(ln) if n})
        self.logger.warning(
            f"U1 RFID: '{name}' resolved to no lanes. Available lanes="
            f"{avail_lanes}; extruders={avail_ext}. If '{name}' is a standalone "
            f"toolhead, ensure its [AFC_stepper] has 'standalone: True' and the "
            f"[AFC_extruder {name}] section exists.")
        return None, None

    def register_lane(self, lane: "AFCLane", channel: int) -> None:
        """
        Register a lane to monitor a specific filament_detect channel.

        :param lane: AFC lane instance to associate with the channel.
        :param channel: U1 filament_detect channel index.
        """
        self._lane_channel_map[lane.name] = channel
        self._lane_objects[lane.name] = lane
        self._last_uid[channel] = None
        self._channel_to_lane[channel] = lane.name
        self._consecutive_failures[channel] = 0

    def start(self) -> None:
        """
        Start polling filament_detect for RFID data.
        """
        if not self._lane_channel_map and not self._cfg_scanner_channels:
            return
        self._gcode = self.afc.gcode
        channels = list(self._lane_channel_map.items())
        self._scanner_channels = {ch for name, ch in channels
                                   if name in self._cfg_scanners}
        self._scanner_channels |= self._cfg_scanner_channels
        if channels:
            self.logger.info(
                f"U1 RFID: monitoring {len(channels)} lane channel(s): "
                + ", ".join(f"{name}=ch{ch}" for name, ch in channels))
        if self._cfg_scanner_channels:
            self.logger.info(
                "U1 RFID: standalone spool scanner channel(s): "
                + ", ".join(f"ch{ch}" for ch in sorted(self._cfg_scanner_channels)))
        lane_scanner_names = [name for name, ch in channels
                              if ch in self._scanner_channels]
        if lane_scanner_names:
            self.logger.info(
                f"U1 RFID: lane-attached scanner(s): {', '.join(lane_scanner_names)}")
        self._try_attach_filament_detect()
        self._register_writers()
        self._poll_timer = self.reactor.register_timer(
            self._poll_cb, self.reactor.monotonic() + POLL_INTERVAL)

    def _write_dir(self) -> str:
        """
        Return where write requests are dropped for the OpenRFID daemon.

        Defaults to a hidden directory beside the config, which both Klipper and
        the daemon can reach; ``openrfid_write_dir`` overrides it.

        :return str: the write-request directory
        """
        if self._cfg_write_dir:
            return self._cfg_write_dir
        start_args = getattr(self.printer, "get_start_args", lambda: {})()
        cfg = start_args.get("config_file", "") if start_args else ""
        return os.path.join(os.path.dirname(cfg), ".afc_u1_write")

    def _register_writers(self) -> None:
        """
        Offer each U1 RFID channel to the shared writer (AFC_RFID_WRITE).

        The reader is the OpenRFID daemon's, not ours, so these targets carry a
        payload hand-off instead of a register link: the write goes out as a
        file the daemon answers. Registered per channel, named for a standalone
        scanner or the lane it feeds.
        """
        seen: set = set()

        def add(name: str, channel: int, what: str) -> None:
            """
            Register one channel as a write target, once.

            :param name: reader name suffix
            :param channel: OpenRFID channel index
            :param what: human description of the channel
            """
            if channel in seen:
                return
            seen.add(channel)
            register_reader(
                self.printer, f"u1:{name}",
                f"U1 OpenRFID {what} (channel {channel})", self,
                open_link=lambda: True,
                threaded=True,
                write_payload=lambda payload, ch=channel: self._openrfid_write(
                    ch, payload))

        for ch in sorted(self._cfg_scanner_channels):
            add(f"scanner{ch}", ch, "spool scanner")
        for lane_name, ch in sorted(self._lane_channel_map.items()):
            add(lane_name, ch, f"lane {lane_name}")

    def _openrfid_write(self, channel: int,
                        payload: bytes) -> Tuple[Optional[str], Optional[str]]:
        """
        Hand a tag payload to the OpenRFID daemon and wait for its verdict.

        Worker thread only: it blocks on the result file. The request is
        written to a temp name and renamed so the daemon never reads it half
        written; the result comes back as res-<token>.json.

        :param channel: the OpenRFID slot the sticker is held at
        :param payload: the encoded tag bytes, written from page 4
        :return tuple: (uid, error); error is None on success
        """
        d = self._write_dir()
        try:
            os.makedirs(d, exist_ok=True)
        except OSError as e:
            return None, f"cannot reach the write directory {d}: {e}"
        token = f"{os.getpid()}-{time.time_ns()}"
        req = os.path.join(d, f"req-{token}.json")
        res = os.path.join(d, f"res-{token}.json")
        tmp = req + ".tmp"
        body = {"slot": channel, "start_page": _WRITE_START_PAGE,
                "data": payload.hex()}
        try:
            with open(tmp, "w") as f:
                json.dump(body, f)
            os.replace(tmp, req)
        except OSError as e:
            return None, f"could not queue the write request: {e}"
        deadline = time.time() + _WRITE_TIMEOUT_S
        while time.time() < deadline:
            if os.path.exists(res):
                try:
                    with open(res) as f:
                        result = json.load(f)
                except (ValueError, OSError):
                    time.sleep(_WRITE_POLL_STEP)   # still being written
                    continue
                uid = result.get("uid")
                if result.get("ok"):
                    return uid, None
                return uid, result.get("error", "the write failed")
            time.sleep(_WRITE_POLL_STEP)
        try:
            os.remove(req)
        except OSError:
            pass
        return None, (
            f"no answer from OpenRFID within {_WRITE_TIMEOUT_S:.0f}s. Is the "
            f"write-watch controller installed and the daemon running? "
            f"(contrib/openrfid-ntag-write)")

    def _try_attach_filament_detect(self) -> bool:
        """
        Look up filament_detect and register the push callback.

        :return bool: True if filament_detect is available, False otherwise.
        """
        if self._filament_detect is not None:
            return True
        fd = self.printer.lookup_object("filament_detect", None)
        if fd is None:
            return False
        self._filament_detect = fd
        # Log the API surface once: breakages are usually a method-name mismatch.
        known = ('_notify_data_update_cb', 'register_cb_2_update_filament_info',
                 'get_a_filament_info', 'get_all_filament_info', 'get_status',
                 'update_filament_info', 'request_update')
        present = [n for n in known if hasattr(fd, n)]
        self.logger.info(
            "U1 RFID: filament_detect attached (api: %s)"
            % (", ".join(present) or "none recognized"))
        self._register_fd_callback(fd)
        return True

    def _register_fd_callback(self, fd: Any) -> None:
        """
        Register our push callback with filament_detect.

        Prefers register_cb_2_update_filament_info() (the proven path, paired
        with FILAMENT_DT_UPDATE); other firmware revs fall back to appending to
        the raw _notify_data_update_cb list.

        :param fd: the filament_detect Klipper object to register the callback on.
        """
        if self._fd_cb_registered:
            return
        if hasattr(fd, 'register_cb_2_update_filament_info'):
            try:
                fd.register_cb_2_update_filament_info(
                    self._on_filament_info_update)
                self._fd_cb_registered = True
                self.logger.info(
                    "U1 RFID: push callback registered via "
                    "register_cb_2_update_filament_info")
                return
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: failed to register info callback: {e}")
        cb_list = getattr(fd, '_notify_data_update_cb', None)
        if isinstance(cb_list, list):
            if self._on_filament_info_update not in cb_list:
                cb_list.append(self._on_filament_info_update)
            self._fd_cb_registered = True
            self.logger.info(
                "U1 RFID: push callback registered via _notify_data_update_cb")
            return
        self.logger.warning(
            "U1 RFID: no recognized filament_detect push-callback API; "
            "scanner will rely on polling only")

    def _on_filament_info_update(self, *args: Any) -> None:
        """
        Callback fired by filament_detect with (channel, info_dict, official).

        :param args: Variable positional arguments forwarded by filament_detect.
        """
        if len(args) >= 2 and isinstance(args[0], int) and isinstance(args[1], dict):
            channel = args[0]
            info = args[1]
            # Registered channels map to a lane name, or None for standalone
            # scanner channels; dispatch on membership.
            if channel in self._channel_to_lane:
                lane_name = self._channel_to_lane.get(channel)
                try:
                    self._check_channel(lane_name, channel, info=info)
                except Exception as e:
                    self.logger.warning(
                        f"U1 RFID: _on_filament_info_update error ch{channel}: {e}")
            return
        for lane_name, channel in self._lane_channel_map.items():
            try:
                self._check_channel(lane_name, channel)
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: _on_filament_info_update error {lane_name}: {e}")

    def stop(self) -> None:
        """
        Stop polling.
        """
        if self._poll_timer is not None:
            self.reactor.update_timer(self._poll_timer, self.reactor.NEVER)

    def _trigger_channel_update(self, channel: int) -> bool:
        """
        Trigger a fresh read from hardware for a scanner channel.

        :param channel: U1 filament_detect channel index.
        :return bool: True on success, False on failure.
        """
        fd = self._filament_detect
        if fd is None:
            return False
        # Prefer FILAMENT_DT_UPDATE: it reads the channel and fires the notify
        # callback, whereas update_filament_info() refreshes without notifying.
        try:
            self._gcode.run_script_from_command(
                f"FILAMENT_DT_UPDATE CHANNEL={channel}")
            return True
        except Exception as e:
            self.logger.warning(
                f"U1 RFID: FILAMENT_DT_UPDATE failed ch{channel}: {e}")
        if hasattr(fd, 'update_filament_info'):
            try:
                fd.update_filament_info(channel)
                return True
            except Exception:
                pass
        if hasattr(fd, 'request_update'):
            try:
                fd.request_update(channel)
                return True
            except Exception:
                pass
        return False

    def _poll_cb(self, eventtime: float) -> float:
        """
        Periodic check for new RFID data on registered channels.

        :param eventtime: Current reactor monotonic time.
        :return float: Next poll time.
        """
        if not self._try_attach_filament_detect():
            return eventtime + _BACKOFF_INTERVAL

        # FILAMENT_DT_UPDATE's internal M400 shuts Klipper down if run from this
        # callback mid-move, so defer the read while the machine is busy.
        idle = self.printer.lookup_object('idle_timeout', None)
        if (idle is not None
                and idle.get_status(eventtime).get('state') == 'Printing'):
            return eventtime + POLL_INTERVAL

        for ch in self._scanner_channels:
            if not self._trigger_channel_update(ch):
                self._consecutive_failures[ch] = \
                    self._consecutive_failures.get(ch, 0) + 1
                if self._consecutive_failures[ch] == _MAX_CONSECUTIVE_FAILURES:
                    self.logger.error(
                        f"U1 RFID: ch{ch} failed {_MAX_CONSECUTIVE_FAILURES} "
                        f"times consecutively, backing off")
                    self._backed_off = True
            else:
                self._consecutive_failures[ch] = 0

        # Poll standalone scanner channels directly: the lane loop below skips
        # them and the push callback may not fire for a held spool.
        for ch in self._cfg_scanner_channels:
            try:
                self._check_channel(None, ch)
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: poll error on scanner ch{ch}: {e}")

        for lane_name, channel in self._lane_channel_map.items():
            try:
                self._check_channel(lane_name, channel)
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: poll error on {lane_name} ch{channel}: {e}")

        if self._backed_off:
            self._backoff_cycles += 1
            if self._backoff_cycles >= _BACKOFF_RESET_CYCLES:
                self._backed_off = False
                self._backoff_cycles = 0
                for ch in self._scanner_channels:
                    self._consecutive_failures[ch] = 0
                self.logger.info("U1 RFID: backoff reset, retrying normal polling")
            else:
                all_recovered = all(
                    self._consecutive_failures.get(ch, 0) < _MAX_CONSECUTIVE_FAILURES
                    for ch in self._scanner_channels)
                if all_recovered:
                    self._backed_off = False
                    self._backoff_cycles = 0
            return eventtime + _BACKOFF_INTERVAL
        return eventtime + POLL_INTERVAL

    _LOCKED_STATES = frozenset({
        "Loaded", "Tooled", "Tool Loaded", "Tool Loading", "Tool Unloading",
        "HUB Loading",
    })

    def _send_lane_data(self, lane: Any) -> None:
        """
        Push lane data to moonraker, guarded.

        AFCLane.send_lane_data() assumes moonraker is connected, which it may
        not be yet after klippy:ready, so skip the push until it is up; the data
        is re-sent on the next read / save_vars.

        :param lane: the AFC lane whose data to push to moonraker.
        """
        if getattr(self.afc, 'moonraker', None) is None:
            return
        try:
            lane.send_lane_data()
        except Exception as e:
            self.logger.debug(
                f"U1 RFID: send_lane_data skipped for "
                f"{getattr(lane, 'name', '?')}: {e}")

    def _handle_webhook_scan(self, web_request: Any) -> None:
        """
        Receive a full tag read pushed by the OpenRFID daemon.

        The daemon's GenericFilament carries the complete colour list, which
        filament_detect drops. Re-pack it into the info schema _map_to_slot_info
        expects (COLOR_NUMS = colour count, RGB_n = each ARGB colour) and run it
        through the normal scan path as the authoritative source.

        Expected JSON body (see the [webhook_exporter] config block):
            {"channel": int, "manufacturer": str, "type": str,
             "sub_type": str, "colors": [argb_int, ...],
             "hotend_min_temp": int, "hotend_max_temp": int,
             "bed_temp": int, "weight_grams": int,
             "card_uid": [int, ...] OR "hex" (OpenRFID scan.uid is a hex string),
             "manufacturing_date": str}   # -> Spoolman lot_nr
        Optional keys newer daemons push: diameter_mm, density, serial_number,
        sku, drying_temp_c, drying_time_hours.

        :param web_request: Klipper webhook request carrying the JSON body
            described above (read via its get_int/get accessors).
        """
        try:
            channel = web_request.get_int('channel')
        except Exception:
            return
        if channel not in self._channel_to_lane:
            return  # not a channel we monitor
        colors = web_request.get('colors', [])
        if not isinstance(colors, (list, tuple)):
            colors = []
        info = {
            'VENDOR': web_request.get('manufacturer', ''),
            'MAIN_TYPE': web_request.get('type', ''),
            'SUB_TYPE': web_request.get('sub_type', ''),
            'HOTEND_MIN_TEMP': web_request.get_int('hotend_min_temp', 0),
            'HOTEND_MAX_TEMP': web_request.get_int('hotend_max_temp', 0),
            'BED_TEMP': web_request.get_int('bed_temp', 0),
            'WEIGHT': web_request.get_int('weight_grams', 0),
            'COLOR_NUMS': len(colors),
            'CARD_UID': web_request.get('card_uid', None),
            'MF_DATE': web_request.get('manufacturing_date', ''),
        }
        # Optional rich fields newer daemons push; absent keys stay unset so
        # _map_to_slot_info simply skips them.
        for json_key, info_key in (('diameter_mm', 'DIAMETER'),
                                   ('density', 'DENSITY'),
                                   ('serial_number', 'SERIAL'),
                                   ('sku', 'SKU'),
                                   ('drying_temp_c', 'DRYING_TEMP'),
                                   ('drying_time_hours', 'DRYING_TIME')):
            val = web_request.get(json_key, None)
            if val not in (None, '', 0):
                info[info_key] = val
        for idx, c in enumerate(colors, start=1):
            try:
                info['RGB_%d' % idx] = int(c)
            except (ValueError, TypeError):
                continue
        # The first webhook makes this channel webhook-authoritative. Keep
        # _last_uid so the UID dedup still suppresses a duplicate of this tag.
        if channel not in self._webhook_channels_seen:
            self._webhook_channels_seen.add(channel)
        lane_name = self._channel_to_lane.get(channel)
        try:
            self._check_channel(lane_name, channel, info=info, source='webhook')
        except Exception as e:
            self.logger.warning(
                f"U1 RFID: webhook scan error ch{channel}: {e}")

    def _check_channel(self, lane_name: Optional[str], channel: int,
                       info: Optional[dict] = None,
                       source: str = 'poll') -> None:
        """
        Check a single channel for new or changed RFID data.

        For spool_scanner lanes the spool is assigned directly to the lane
        (bypassing the ``next_spool_id`` staging mechanism) so that Spoolman
        flow K sync triggers immediately on scan.

        :param lane_name: AFC lane name mapped to this RFID channel.
        :param channel: U1 filament_detect channel index.
        :param info: Pre-fetched RFID info dict, or *None* to read live.
        :param source: 'poll'/'push' (filament_detect) or 'webhook'.
        """
        if info is None:
            if self._filament_detect is None:
                return
            info = self._get_channel_info(channel)
        # Standalone scanner channel: no lane; the scan stages next_spool_id.
        scanner_only = channel in self._cfg_scanner_channels
        lane = None if scanner_only else self._lane_objects.get(lane_name)
        is_scanner = scanner_only or (
            lane is not None and getattr(lane, 'spool_scanner', False))

        if info is None:
            return

        card_uid = info.get("CARD_UID")
        if not card_uid or card_uid == 0:
            self._pending_confirm.pop(channel, None)
            if self._last_uid.get(channel) not in (None, 0):
                if not is_scanner:
                    self._last_uid[channel] = 0
                    if lane is not None and getattr(lane, "status", "") not in self._LOCKED_STATES:
                        self._clear_lane(lane, lane_name)
                # Scanner channels keep _last_uid so the same spool does not
                # re-fire; a different uid still triggers.
            return

        # Once a webhook has been seen on this channel, ignore the colour-lossy
        # filament_detect read (removal above still runs).
        if source != 'webhook' and channel in self._webhook_channels_seen:
            return

        if card_uid == self._last_uid.get(channel):
            return

        # Scanner stable-read gate for filament_detect reads: require the same
        # UID on N consecutive reads. A webhook acts immediately.
        if (is_scanner
            and self._scanner_confirm_reads > 1
            and source != 'webhook'):
            pending, count = self._pending_confirm.get(channel, (None, 0))
            count = count + 1 if card_uid == pending else 1
            self._pending_confirm[channel] = (card_uid, count)
            if count < self._scanner_confirm_reads:
                self.logger.debug(
                    f"U1 RFID: ch{channel} UID {self._fmt_uid(card_uid)} seen "
                    f"{count}/{self._scanner_confirm_reads}, waiting for a "
                    f"stable read")
                return
            self._pending_confirm.pop(channel, None)

        if not scanner_only and lane is None:
            return

        if not is_scanner and getattr(lane, "status", "") in self._LOCKED_STATES:
            return

        # Defer a new tag's 'poll' read by webhook_grace so a webhook can land
        # first; _grace_expired re-checks with 'poll-final' if none arrived.
        if (self._webhook_grace > 0
            and source == 'poll'
            and channel not in self._webhook_channels_seen):
            if self._pending_defer.get(channel) != card_uid:
                self._pending_defer[channel] = card_uid
                self.reactor.register_callback(
                    lambda et, ln=lane_name, ch=channel, uid=card_uid:
                        self._grace_expired(ln, ch, uid),
                    self.reactor.monotonic() + self._webhook_grace)
            return

        self._last_uid[channel] = card_uid

        # Dump the raw tag dict so field-mapping issues (colour, temps, vendor)
        # are diagnosable without guessing filament_detect's schema.
        self.logger.debug(f"U1 RFID: ch{channel} raw tag info: {info}")

        main_type = info.get("MAIN_TYPE", "")
        if not main_type or main_type.upper() == "NONE":
            return

        slot_info = self._map_to_slot_info(info)
        record_key = lane_name or f"scanner-ch{channel}"
        # Lazy-init so tests that build the reader without running __init__
        # (and older pickled state) still record cleanly.
        if not hasattr(self, "_tag_reads"):
            self._tag_reads = {}
        self._tag_reads[record_key] = make_tag_record(slot_info, time.time())
        brand = slot_info.get('brand', '')
        material = slot_info.get('material', '')
        color = slot_info.get('color_hex', '')
        multi_color = slot_info.get('multi_color', [color] if color else [])
        tag_desc = f"{brand} {material}".strip() or "Unknown"
        clabel = " + ".join(f"#{c.lstrip('#')}" for c in multi_color if c)
        if clabel:
            tag_desc += f" ({clabel})"

        if is_scanner:
            self.logger.info(f"U1 RFID: spool scanned: {tag_desc}")
            # Scanner-only channels use the configured scanner default.
            allow_create = (self._scanner_auto_create if scanner_only
                            else get_auto_spoolman_create(
                                lane, self._lane_auto_create))
            # reactor=: the Spoolman round trips go off the reactor onto
            # moonraker's writer thread. on_done=: the notification reads
            # lane.spool_id, which is no longer set by the time this returns.
            def self_notify() -> None:
                """
                Notify the scan once the Spoolman sync has landed.
                """
                self._notify_scan(
                    brand, material, color, slot_info,
                    lane_name=(lane_name or f"scanner-ch{channel}"),
                    is_scanner=True)
            sync_rfid_to_spoolman(
                self.afc, lane, slot_info, self.logger, "U1 RFID",
                allow_create=allow_create, set_next=True,
                reactor=self.reactor, on_done=self_notify)
            return

        self.logger.info(f"U1 RFID: tag detected on {lane_name}: {tag_desc}")
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            self.afc.spool.set_spoolID(lane, "")
        apply_filament_defaults(lane, slot_info)
        allow_create = get_auto_spoolman_create(lane, self._lane_auto_create)
        def _after_sync() -> None:
            """
            Notify and push lane data once the lane carries the spool (reactor).
            """
            self._notify_scan(brand, material, color, slot_info,
                              lane_name=lane_name)
            self._send_lane_data(lane)

        sync_rfid_to_spoolman(
            self.afc, lane, slot_info, self.logger, "U1 RFID",
            allow_create=allow_create, reactor=self.reactor,
            on_done=_after_sync)
        self.afc.save_vars()
        if getattr(lane, 'tool_loaded', False):
            self.printer.send_event("afc:tool_loaded", lane)

    def _grace_expired(self, lane_name: Optional[str], channel: int,
                       card_uid: Any) -> None:
        """
        webhook_grace timer: process the deferred filament_detect read only if
        no webhook arrived for the channel during the grace window.

        :param lane_name: AFC lane name for the channel (None for scanner-only).
        :param channel: U1 filament_detect channel index.
        :param card_uid: the tag UID the deferral was armed for; the read is
            skipped if a newer tag has since superseded it.
        """
        if self._pending_defer.get(channel) != card_uid:
            return  # superseded by a newer tag, or already cleared
        self._pending_defer.pop(channel, None)
        if channel in self._webhook_channels_seen:
            return  # a webhook landed during the grace and handled the tag
        try:
            self._check_channel(lane_name, channel, source='poll-final')
        except Exception as e:
            self.logger.warning(
                f"U1 RFID: deferred read error ch{channel}: {e}")

    def _clear_lane(self, lane: Any, lane_name: str) -> None:
        """
        Clear RFID data from a lane when tag is removed.

        :param lane: AFC lane instance to clear.
        :param lane_name: Name of the lane being cleared.
        """
        lane.material = ""
        lane.color = ""
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            try:
                self.afc.spool.set_spoolID(lane, "")
            except Exception as e:
                self.logger.warning(
                    f"U1 RFID: failed to clear spool_id on {lane_name}: {e}")
        self._send_lane_data(lane)
        self.afc.save_vars()

    def _get_channel_info(self, channel: int) -> Optional[dict]:
        """
        Read filament info for a channel from filament_detect.

        :param channel: U1 filament_detect channel index.
        :return Optional[dict]: RFID info dict for the channel, or None.
        """
        fd = self._filament_detect
        if hasattr(fd, 'get_a_filament_info'):
            try:
                info = fd.get_a_filament_info(channel)
                if isinstance(info, dict):
                    return info
            except Exception:
                pass
        if hasattr(fd, 'get_all_filament_info'):
            try:
                all_info = fd.get_all_filament_info()
                if isinstance(all_info, (list, tuple)) and channel < len(all_info):
                    entry = all_info[channel]
                    if isinstance(entry, dict):
                        return entry
                elif isinstance(all_info, dict):
                    entry = all_info.get(channel) or all_info.get(str(channel))
                    if isinstance(entry, dict):
                        return entry
            except Exception:
                pass
        if hasattr(fd, 'get_status'):
            try:
                status = fd.get_status()
                if isinstance(status, dict):
                    info_list = status.get('info')
                    if info_list and channel < len(info_list):
                        entry = info_list[channel]
                        if isinstance(entry, dict) and entry.get("CARD_UID"):
                            return entry
            except Exception:
                pass
        return None

    def _tag_color_count(self, info: dict) -> Optional[int]:
        """
        Return the tag's declared colour count, or None if not present.

        The U1's filament_detect schema isn't fully known here, so match any
        key that means "colour count" (contains COLOR/COLOUR and COUNT/NUM/NUMS)
        rather than hard-coding one name. The OpenRFID Bambu processor decodes
        this as "Color Count", so the forwarded info dict should expose it.

        :param info: raw RFID info dict from filament_detect / the webhook.
        :return Optional[int]: the declared colour count (>= 1), or None.
        """
        for k, v in info.items():
            ku = str(k).upper()
            if (("COLOR" in ku or "COLOUR" in ku)
                    and ("COUNT" in ku or "NUM" in ku)):
                try:
                    n = int(v)
                except (ValueError, TypeError):
                    continue
                if n >= 1:
                    return n
        return None

    def _map_to_slot_info(self, info: dict) -> dict:
        """
        Map filament_detect fields to AFC RFID slot_info format.

        :param info: Raw RFID info dict from filament_detect.
        :return dict: Normalized slot info dict for AFC use.
        """
        # Gather colours in tag order (RGB_1, RGB_2, ...), masking the ARGB
        # alpha byte (-> RRGGBB).
        ordered = []
        rgb_keys = sorted((k for k in info if re.fullmatch(r"RGB_\d+", str(k))),
                          key=lambda k: int(str(k).split("_")[1]))
        for key in rgb_keys:
            raw = info.get(key)
            if raw is None or raw == "":
                continue
            try:
                raw_int = int(raw)
            except (ValueError, TypeError):
                continue
            ordered.append((raw_int, f"{raw_int & 0xFFFFFF:06x}"))

        # Prefer the tag's own colour count; without it, drop the U1's unused-
        # slot white sentinel (0xFFFFFFFF) on secondary slots.
        color_count = self._tag_color_count(info)
        multi_color = []
        if color_count is not None and color_count >= 1:
            for _raw_int, hx in ordered[:color_count]:
                if hx not in multi_color:
                    multi_color.append(hx)
            _src = f"tag count={color_count}"
        else:
            for raw_int, hx in ordered:
                if multi_color and raw_int == 0xFFFFFFFF:
                    continue  # unused secondary slot, not a real colour
                if hx not in multi_color:
                    multi_color.append(hx)
            _src = "no tag count field; white-sentinel heuristic"
        color_hex = multi_color[0] if multi_color else ""
        self.logger.debug(
            f"U1 RFID: parsed {len(multi_color)} colour(s) {multi_color} "
            f"from RGB slots {[hx for _, hx in ordered]} ({_src})")
        ext_max = info.get("HOTEND_MAX_TEMP")
        ext_min = info.get("HOTEND_MIN_TEMP")
        bed_max = info.get("BED_TEMP")
        sku_raw = info.get("SKU", "")
        sku = "" if (not sku_raw or sku_raw == 0) else str(sku_raw)
        vendor = info.get("VENDOR", "")
        if vendor.upper() == "NONE":
            vendor = ""
        if ext_max and ext_min:
            ext_temp = (int(ext_max) + int(ext_min)) // 2
        elif ext_max:
            ext_temp = int(ext_max)
        else:
            ext_temp = None
        # Tag diameter when the daemon supplies one; the U1 default otherwise.
        try:
            diameter = float(info.get("DIAMETER") or 0) or 1.75
        except (TypeError, ValueError):
            diameter = 1.75
        slot_info = {
            "material": info.get("MAIN_TYPE", ""),
            "color_hex": color_hex,
            "multi_color": multi_color,
            "is_dual_color": len(multi_color) > 1,
            "sku": sku,
            "brand": vendor,
            "sub_type": info.get("SUB_TYPE", ""),
            "diameter": diameter,
            "extruder_temp": ext_temp,
            "bed_temp": int(bed_max) if bed_max else None,
            # Tag metadata for Spoolman: manufacturing date -> lot_nr, card UID
            # -> card_uids extra field. (MF_DATE is reliable on the OpenRFID
            # webhook path; the filament_detect path often reports it unset.)
            "mfg_date": self._fmt_mfg_date(info.get("MF_DATE")),
            "uid": self._fmt_uid(info.get("CARD_UID")),
        }
        # Optional rich fields, uniform with map_tag_to_slot_info, only when
        # the tag/daemon carried them.
        if ext_min:
            slot_info["extruder_temp_min"] = int(ext_min)
        if ext_max:
            slot_info["extruder_temp_max"] = int(ext_max)
        try:
            weight = int(info.get("WEIGHT") or 0)
        except (TypeError, ValueError):
            weight = 0
        if weight > 0:
            slot_info["weight_g"] = weight
        for src, dst in (("SERIAL", "serial"), ("DENSITY", "density"),
                         ("DRYING_TEMP", "drying_temp"),
                         ("DRYING_TIME", "drying_time_h"),
                         ("COLOR_NUMS", "color_count")):
            val = info.get(src)
            if val not in (None, "", 0, "0"):
                slot_info[dst] = val
        return slot_info

    @staticmethod
    def _fmt_mfg_date(raw: Any) -> Optional[str]:
        """
        Normalize a tag manufacturing date to a clean string (lot_nr), or
        None if unset. Accepts YYYYMMDD (filament_detect) or ISO YYYY-MM-DD
        (OpenRFID webhook); treats epoch/1970 as 'unset'.

        :param raw: the tag's raw manufacturing-date value (str/int/None).
        :return Optional[str]: a normalised 'YYYY-MM-DD' string, or None.
        """
        if not raw:
            return None
        s = str(raw).strip()
        if not s or s.startswith("1970") or s in ("0", "00000000"):
            return None
        if len(s) == 8 and s.isdigit():
            return f"{s[0:4]}-{s[4:6]}-{s[6:8]}"
        return s

    @staticmethod
    def _fmt_uid(raw: Any) -> Optional[str]:
        """
        Format a card UID (list of byte ints, e.g. [123,240,175,255]) as an
        uppercase hex string ('7BF0AFFF'); pass through a non-empty string.

        :param raw: a card UID as a list/tuple of byte ints, or a string.
        :return Optional[str]: the uppercase-hex UID string, or None if empty.
        """
        if not raw:
            return None
        if isinstance(raw, (list, tuple)):
            try:
                return "".join("%02X" % (int(b) & 0xFF) for b in raw)
            except (ValueError, TypeError):
                return None
        # A hex string (e.g. the webhook's scan.uid): uppercase to match the
        # byte-list form filament_detect sends.
        s = str(raw).strip().upper()
        return s or None

    def _notify_scan(self, brand: str, material: str, color: str,
                     slot_info: dict, lane_name: str = "",
                     is_scanner: bool = False) -> None:
        """
        Send a user-visible notification when RFID reads a spool.

        Sends to three targets: Klipper console (respond_info), Mainsail/
        Octoprint (action:prompt), and U1 factory display (exception_manager).

        :param brand: Filament brand name.
        :param material: Filament material type (e.g. "PLA").
        :param color: Hex color string (without '#').
        :param slot_info: Full slot info dict from RFID tag.
        :param lane_name: Lane name; empty string if unknown.
        :param is_scanner: True when this is a spool_scanner read.
        """
        try:
            # Rich "<brand> <material> <sub_type>" name (e.g. "Bambu PLA Basic");
            # shared builder so every scanner shows the same naming. Colour is
            # shown as hex only (no hex->name mapping).
            name = build_filament_name(brand, material,
                                       slot_info.get("sub_type", ""))
            ext = slot_info.get("extruder_temp")
            bed = slot_info.get("bed_temp")
            raw = self.logger.raw
            lane = self._lane_objects.get(lane_name) if lane_name else None
            spool_id = getattr(lane, "spool_id", None) if lane else None

            # Enrich from the lane, which set_spoolID has just populated from a
            # fresh Spoolman fetch (on_done guarantees it has landed).
            if spool_id is not None and lane is not None:
                name = getattr(lane, "filament_name", "") or name
                brand = getattr(lane, "spool_vendor", "") or brand
                material = getattr(lane, "material", "") or material
                ext = getattr(lane, "extruder_temp", None) or ext
                bed = getattr(lane, "bed_temp", None) or bed

            if is_scanner:
                title = "Spool Scanned"
                header = "Spool scanned on %s:" % lane_name if lane_name else "Spool scanned:"
            else:
                title = "Spool Loaded: %s" % lane_name if lane_name else "Spool Loaded"
                header = "Spool loaded on %s:" % lane_name if lane_name else "Spool loaded:"

            lines = [header]
            if name:
                lines.append(f"  Name: {name}")
            if brand:
                lines.append(f"  Brand: {brand}")
            if material:
                lines.append(f"  Material: {material}")
            if color:
                lines.append(f"  Color: #{color}")
            if ext:
                lines.append(f"  Nozzle temp: {ext}°C")
            if bed:
                lines.append(f"  Bed temp: {bed}°C")
            if spool_id:
                lines.append(f"  Spoolman ID: {spool_id}")
            self.afc.gcode.respond_info("\n".join(lines))

            # Mainsail/Fluidd popup + U1 factory display are reserved for the
            # spool_scanner. A normal lane load only prints the console read-out
            # above (no popup), so loads don't spam a dialog on every insert.
            if is_scanner:
                prompt_lines = []
                if name:
                    prompt_lines.append(f"Name: {name}")
                if brand:
                    prompt_lines.append(f"Brand: {brand}")
                if material:
                    prompt_lines.append(f"Material: {material}")
                if color:
                    prompt_lines.append(f"Color: #{color}")
                if ext:
                    prompt_lines.append(f"Nozzle: {ext}°C")
                if bed:
                    prompt_lines.append(f"Bed: {bed}°C")
                if spool_id:
                    prompt_lines.append(f"Spoolman ID: {spool_id}")
                raw(f"// action:prompt_begin {title}")
                for pl in prompt_lines:
                    raw(f"// action:prompt_text {pl}")
                raw("// action:prompt_footer_button "
                    "OK|RESPOND TYPE=command MSG=action:prompt_end|info")
                raw("// action:prompt_show")
                self.reactor.register_callback(
                    lambda e: self.logger.raw("// action:prompt_end"),
                    self.reactor.monotonic() + 10.0)

                em = self.printer.lookup_object("exception_manager", None)
                if em is not None:
                    label = name or " ".join(p for p in (brand, material) if p)
                    msg = "%s: %s" % (title, label) if label else title
                    channel = self._lane_channel_map.get(lane_name, 0)
                    em.raise_exception_async(
                        id=529, index=channel, code=99,
                        message=msg, oneshot=1, level=1)
        except Exception as e:
            self.logger.warning(f"U1 RFID: notification error: {e}")

    def force_read(self, lane_name: str) -> None:
        """
        Force an RFID re-read for a specific lane.

        :param lane_name: Name of the lane to re-read.
        """
        channel = self._lane_channel_map.get(lane_name)
        if channel is None:
            return
        self._last_uid[channel] = None
        if not self._trigger_channel_update(channel):
            self.logger.warning(
                f"U1 RFID: force_read failed to trigger update for {lane_name}")
            return
        deadline = self.reactor.monotonic() + _FORCE_READ_TIMEOUT
        while self.reactor.monotonic() < deadline:
            info = self._get_channel_info(channel)
            if info is not None and info.get("CARD_UID"):
                self._check_channel(lane_name, channel, info=info)
                return
            self.reactor.pause(
                self.reactor.monotonic() + _FORCE_READ_POLL_STEP)
        self._check_channel(lane_name, channel)

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        Report the lane->channel wiring and the per-lane/scanner last-read
        records (uniform with the ACE2/ViViD/OpenAMS readers).

        :param eventtime: Reactor event time (unused; kept for the status API).
        :return dict: The lane_channel_map, scanner_channels and last_reads.
        """
        return {
            "lane_channel_map": dict(self._lane_channel_map),
            "scanner_channels": sorted(self._scanner_channels),
            "last_reads": dict(getattr(self, "_tag_reads", {}) or {}),
        }


def load_config(config: "ConfigWrapper") -> AFC_U1_RFID:
    """
    Klipper config hook: instantiate the U1 RFID reader.

    :param config: Klipper config wrapper for the [AFC_U1_rfid] section.
    :return AFC_U1_RFID: the instantiated reader.
    """
    return AFC_U1_RFID(config)
