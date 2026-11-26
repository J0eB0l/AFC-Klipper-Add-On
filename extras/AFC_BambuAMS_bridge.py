# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 J0eB0l. All Rights Reserved.
#
# LIMITED USE LICENSE
#
# This file is NOT distributed under the GNU GPL.
#
# Permission is granted, free of charge, to download, install and execute this
# file solely as an add-on component of the official, unmodified AFC
# (Automated Filament Control) distribution hosted at:
#
#     https://github.com/AFCProject/AFC-Klipper-Add-On
#
# The AFC Project is granted permission to host and distribute this file as
# part of that repository, unmodified and with this notice intact.
#
# RESTRICTIONS
#   1. You may NOT modify, reverse-engineer, decompile or create derivative
#      works of this file.
#   2. You may NOT bundle, redistribute, re-host or include this file in any
#      third-party software, installer or package manager without express
#      written consent of the copyright holder.
#   3. You may NOT use this file with modified forks or unauthorised
#      distributions of the AFC ecosystem.
#
# THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED. THE COPYRIGHT HOLDER IS NOT LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY ARISING FROM ITS USE.

from __future__ import annotations
import glob
import hashlib
import hmac
import json
import logging
import logging.handlers
import os
import queue
import re
import traceback
import socket
import threading
import chelper

import time
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

_MEAS_LIVE_MAX_S = 120.0

WRITE_PACE_DEPTH = 8
WRITE_PACE_S = 0.02

CONNECT_GRACE_S = 2.0

SILENCE_LOG_S = 2.5


def _queued_cmd_name(data: Any) -> str:
    try:
        if isinstance(data, (bytes, bytearray, memoryview)):
            data = bytes(data).decode("utf-8", "replace")
        obj = json.loads(data)
        cmd = obj.get("cmd") if isinstance(obj, dict) else None
        return str(cmd) if cmd not in (None, "") else "?"
    except Exception:
        return "?"


def parse_bridge_line(line: str) -> Optional[dict]:
    line = line.strip()
    if not line:
        return None
    try:
        obj = json.loads(line)
    except (ValueError, TypeError):
        return None
    return obj if isinstance(obj, dict) else None

def routine_info(logger: Any, message: str) -> None:
    if getattr(logger, "print_debug_console", False):
        logger.info(message)
    else:
        logger.debug(message)


_BLDC_I_RE = re.compile(r"bldc_i:([0-9.]+)A")


_CHMB_AD_RE = re.compile(r"\[AMS_CHMB\].*?\bad:(\d+)(?:\s*,\s*([0-9.]+))?")

_CHMB_STATE_RE = re.compile(
    r"\[AMS_CHMB\]s:(\d+)(?:,\d+)?\s*[|,]\s*"
    r"rf:(\d+)(?:,\d+)?\s*[|,]\s*"
    r"(?:cd:\d+\s*[|,]\s*)?"
    r"vt:([0-9.]+)(?:\s*,\s*\d+)?"
    r"(?:.*?\bhts?:(\d+)\s*,)?")
_MOTION_FINISH_RE = re.compile(r"\b(?:(?:feed|pull|preload)\s+finish|pull\s+suc+ess"
                               r"|feed\s+to\s+(?:normal|dw\s+ok))\b", re.IGNORECASE)

_STATE_SWITCH_DONE_RE = re.compile(r"state_switch\s+finish\s*,\s*\w+\s*,\s*err_code:0x00\b",
                                   re.IGNORECASE)

TRAY_NONE = 255

#: How many narration lines are kept for a later report.
NARRATION_KEPT = 24

_TRAY_NOW_RE = re.compile(r"tray_now:(\d+)")

_HT_CALI_RST_RE = re.compile(r"Calibration\s+rst:(\d+)", re.IGNORECASE)


_BUFF_POS_RE = re.compile(r"buff_pos:(-?[0-9]+\.[0-9]+)")

_BUFF_REFILL_RE = re.compile(r"BUFF,\s*pos:(-?[0-9.]+)\s*->\s*(-?[0-9.]+)"
                             r"(?:[^\n]*?det:(\d+)\s*mm)?", re.IGNORECASE)

_CLEAN_FINISH_RE = re.compile(r"\b(?:feed|pull|preload)\s+finish\s*(?:,|$)", re.IGNORECASE)

FINISH_ARRIVAL_TOLERANCE_MM = 100.0

_ODOM_RESET_RE = re.compile(r"odom\s+reset\s+tray\s*\d+", re.IGNORECASE)
_ERR_CODE_RE = re.compile(r"err_code:\s*(0x[0-9A-Fa-f]+|\d+)\s*->\s*(0x[0-9A-Fa-f]+|\d+)")

_ODOM_NO_TRAY_RE = re.compile(r"odom\s+tray_id\s+error\s*255", re.IGNORECASE)


_LEN_DET_M_RE = re.compile(r"len_det:([0-9]+\.[0-9]+)\s*m\b")

_ASSIST_DONE_RE = re.compile(r"assist finish\s*-?\d*")

_TUBE_LEN_MM_RE = re.compile(r"tube_len:(\d+)\s*mm")
_TUBE_LEN_M_RE = re.compile(r"tube_len:([0-9]+\.[0-9]+)\s*m\b")
_DW_LEN_M_RE = re.compile(r"dw_len:([0-9]+\.[0-9]+)\s*m\b")


_DRY_REFUSED_RE = re.compile(r"\[AMS_CHMB\]\s*err,\s*([^\[\r\n]{1,60})")


_CAP_MEASURE_RE = re.compile(
    r"odom\s+"
    r"(?:load\s+from\s+flash\s+(\d+)\s*,\s*"
    r"|C:([0-9.]+)\s*,\s*)?"
    r"R:([0-9.]+)\s*,\s*"
    r"P:(\d+)\s*%?",
    re.IGNORECASE)

_CAP_MEASURE_ALT_RE = re.compile(
    r"odom\s+r(\d+)\s*,\s*"
    r"dt[0-9.]+\s*,\s*"
    r"R([0-9.]+)\s*,\s*"
    r"(\d+)\s*%",
    re.IGNORECASE)

_CAP_SAVE_RE = re.compile(r"odom\s+save\s+tray:(\d+)\s*,\s*R:([0-9.]+)", re.IGNORECASE)

_CALI_DONE_RE = re.compile(r"odom\s+calib\s+succ?ess(?:\s+exit\s+(\d+))?", re.IGNORECASE)

_DRY_CFG_RE = re.compile(r"\[AMS_CHMB\]\s*rotate:(\d+),\s*(\d+),\s*pw_lim:(\d+),"
                         r"\s*cool_down:\d+,\s*(\d+),\s*dur:(\d+),\s*tmpr:(\d+)")

_STEP_SEP = r"STEP\d*\s*[,:]\s*"


def _STEP(tail: str) -> "re.Pattern":
    return re.compile(r"\[AMS_[A-Z_]+\]\s*" + _STEP_SEP + tail, re.IGNORECASE)


_AMS_HUMAN = (
    (re.compile(r"\[AMS_CHMB\]\s*err,\s*ams-ht shell open", re.IGNORECASE),
     lambda m: ("AMS HT lid is open -- drying continues, but the chamber "
                "cannot hold temperature until the shell is closed.")),
    (_DRY_REFUSED_RE,
     lambda m: (f"AMS refused the drying command: {m.group(1).strip()}. "
                f"An AMS will not dry with filament out in the hub -- reel "
                f"the lane back to its bay first (LANE_UNLOAD).")),
    (re.compile(r"\[AMS_CHMB\]ignore[^,]*,\s*ams_state:(\d+)"),
     lambda m: (f"AMS refused the drying command -- it was busy "
                f"(state {m.group(1)}). Wait for it to settle and try again.")),
    (re.compile(r"\[AMS_CHMB\]set state CTC_STATE_SELF_CHECK.*?ref:(\d+)"),
     lambda m: f"AMS drying: self-check started, target {m.group(1)}C",
     True),
    (re.compile(r"\[AMS_CHMB\]set state CTC_STATE_HEATING"),
     lambda m: "AMS drying: self-check passed, now heating",
     True),
    (re.compile(r"\[AMS_CHMB\]set state CTC_STATE_OFF"),
     lambda m: "AMS drying: heater off",
     True),
    (re.compile(r"\[AMS_SWITCH\]new tube_len:(\d+) mm.*?err:(-?\d+) mm"),
     lambda m: (f"AMS measured the PTFE path at {m.group(1)}mm "
                f"(+/-{m.group(2)}mm)"),
     True),
    (_STEP("card auth success"),
     lambda m: "AMS: tag authenticated",
     True),
    (_STEP("auth card successful"),
     lambda m: "AMS: tag authenticated",
     True),
    (_STEP("first detected"),
     lambda m: "AMS: spool detected",
     True),
    (_STEP(r"select card fail, err (\d+)"),
     lambda m: f"AMS could not read the spool tag (err {m.group(1)})",
     True),
    (_STEP(r"odom calib succ?ess(?:\s+exit\s+(\d+))?"),
     lambda m: "AMS finished measuring the spool",
     True),
    (re.compile(r"\[RF\]\s*tray(\d+): info write to flash"),
     lambda m: (f"AMS: tag for bay {int(m.group(1)) + 1} cached in the unit's "
                f"flash (a later read returns it even after a swap)"),
     True),
    (re.compile(r"preload\s+finish", re.IGNORECASE),
     lambda m: "AMS staged the spool at its feeder",
     True),
    (re.compile(r"odom\s+C:[0-9.]+\s*,\s*R:([0-9.]+)\s*,\s*P:(\d+)%"),
     lambda m: (f"AMS measured the spool: about {m.group(2)}% left "
                f"(spool radius {float(m.group(1)) * 1000:.0f} mm)"),
     True),
    (re.compile(r"odom\s+load\s+from\s+flash\s*(\d+)\s*,\s*"
                r"R:[0-9.]+\s*,\s*P:(\d+)"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} remembers about "
                f"{m.group(2)}% left from its last measurement"),
     True),
    (re.compile(r"odom\s+save\s+tray:(\d+)"),
     lambda m: f"AMS stored a new measurement for bay {int(m.group(1)) + 1}",
     True),
    (re.compile(r"feed\s+finish\s+-?\d+\s*,\s*stall\s*,\s*"
                r"len_det:([0-9.]+)\s*m\s*,\s*tube_len:([0-9.]+)\s*m"),
     lambda m: (f"AMS: the filament STALLED after {float(m.group(1)):.2f} m "
                f"of a {float(m.group(2)):.2f} m path -- check for a jam "
                f"between the bay and the toolhead")),
    (re.compile(r"err_code:0x00\s*->\s*0x16\b"), None),
    (re.compile(r"err_code:0x16\s*->\s*0x0+\b"), None),
    (re.compile(r"err_code:0x00\s*->\s*0x17\b"), None),
    (re.compile(r"err_code:0x17\s*->\s*0x0+\b"), None),
    (re.compile(r"err_code:0x00\s*->\s*0x80\b"), None),
    (re.compile(r"err_code:0x80\s*->\s*0x0+\b"), None),
    (re.compile(r"err_code:0x00\s*->\s*0x([0-9A-Fa-f]+)"),
     lambda m: f"AMS raised error 0x{m.group(1).upper()}",
     "routine"),
    (re.compile(r"err_code:0x[0-9A-Fa-f]+\s*->\s*0x0+\b"),
     lambda m: "AMS cleared its error",
     "routine"),
    (re.compile(r"tray\s+(\d+)\s+out\s*,\s*clear\s+magic_num"),
     lambda m: f"AMS: bay {int(m.group(1)) + 1} is now empty",
     True),
    (re.compile(r"ams\s+pmsm\s+cali\s+finish", re.IGNORECASE),
     lambda m: "AMS finished its power-up self-check",
     True),
    (re.compile(r"pmsm\s+self\s+check\s+good", re.IGNORECASE),
     lambda m: "AMS motor self-check passed",
     True),
    (_STEP("second detected"),
     lambda m: "AMS: spool turned a full revolution (measuring)",
     True),
    (_STEP(r"odom calib\s*,\s*tray (\d+)"),
     lambda m: f"AMS started measuring bay {int(m.group(1)) + 1}",
     True),
    (_STEP(r"cali end"),
     lambda m: "AMS finished its measuring cycle",
     True),
    (_STEP(r"Calibration rst:(\d+)"),
     lambda m: f"AMS finished measuring (result {m.group(1)})",
     True),
    (_STEP(r"auth fail:-?(\d+)"),
     lambda m: ("AMS could not authenticate the tag -- third-party spool, "
                "or the tag is unreadable"),
     True),
    (re.compile(r"\[RF\]\s*tray(\d+): info same as last read"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} holds the same spool as "
                f"before"),
     True),
    (_STEP(r"odom invalid tray (\d+)"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} has no stored measurement "
                f"yet"),
     True),
    (_STEP(r"odom load tray (\d+) info invailed"),
     lambda m: f"AMS: bay {int(m.group(1)) + 1} has no stored measurement yet",
     True),
    (_STEP(r"odom tray_id error (\d+)"),
     lambda m: ("AMS: asked to move with NO TRAY SELECTED -- the unit "
                "rejected the command"),
     True),
    (re.compile(r"\[AMS_LED\]\s*TIMEOUT error (\d+)"),
     lambda m: "AMS: TIMEOUT -- the unit gave up on the move"),
    (re.compile(r"\bfeed tray:(\d+)\s*,"),
     lambda m: f"AMS: feeding from bay {int(m.group(1)) + 1}",
     True),
    (re.compile(r"assist finish stall!\s*pos:(-?[0-9.]+)->(-?[0-9.]+)"),
     lambda m: (f"AMS stopped its assist (buffer {m.group(1)} -> "
                f"{m.group(2)})"),
     True),
    (re.compile(r"feed to dw ok\s*,\s*len_det:([0-9.]+)\s*m"),
     lambda m: f"AMS: filament reached the hub after {float(m.group(1)):.2f} m",
     True),
    (re.compile(r"feed finish\s*,\s*buff_pos:[0-9.]+\s*,\s*bldc_i:[0-9.]+A"
                r"\s*,\s*t:([0-9.]+)s"),
     lambda m: f"AMS finished feeding ({float(m.group(1)):.1f} s)",
     True),
    (re.compile(r"new tube_len:(\d+)\s*mm"),
     lambda m: f"AMS learned the bay-to-hub path length: {m.group(1)} mm",
     True),
    (re.compile(r"preload start", re.IGNORECASE),
     lambda m: "AMS is staging the spool at its feeder",
     True),
    (re.compile(r"\[AMS_ADA\]\s*init", re.IGNORECASE),
     lambda m: "AMS powered up (any follower state it held is gone)",
     True),
)

_RFID_READ_OK_RE = re.compile(
    _STEP_SEP + r"read success"
    r"|" + _STEP_SEP + r"read_done=1"
    r"|" + _STEP_SEP + r"feed with rfid success"
    r"|" + _STEP_SEP + r"save to flash"
    r"|card info valid"
    r"|info write to flash"
    r"|info same as last read")

_RFID_READ_OK_HT_RE = re.compile(
    _STEP_SEP + r"save to flash|card info valid|info write to flash|info same as last read")

_RFID_FOREIGN_TAG_RE = re.compile(
    r"auth fail\s*:?\s*-?\d+",
    re.IGNORECASE)

_RFID_CYCLE_END_RE = re.compile(
    r"Calibration\s+rst:\d+"
    r"|STEP7:\s*(?:finish|cali\s+end)"
    r"|tray\s+capacity\s+no\s+en"
    r"|odom\s+pull\s+back",
    re.IGNORECASE)

_MEAS_END_RE = re.compile(r"odom\s+calib\s+succ?ess", re.IGNORECASE)

_AMS_GAVE_UP_RE = re.compile(r"state_switch\s+finish\s*,\s*fail\b", re.IGNORECASE)

_AMS_NO_TRAY_RE = _STEP(r"odom tray_id error (\d+)")

_AMS_TRAY_SW_RE = re.compile(
    r"\[AMS_TRAY\]\s*tray\[(\d+)\]\s*sw_sta\s*update\s*,\s*(\d+)\s*->\s*(\d+)")

_RFID_TERMINAL_RE = re.compile(r"STEP7:\s*(?:finish|cali\s+end)|Calibration\s+rst:\d+"
                               r"|tray\s+capacity\s+no\s+en", re.IGNORECASE)

_AMS_NOISE_RE = re.compile(
    r"^(?:\s*(?:"
    r"\[AMS_CALL\]\s*ams\d+\s+select,\s*select\s+ams\d+"
    r"|\[AMS_LINK\]\s*ams\d+\s+select,\s*req\s+ams\d+"
    r"|\[AMS_COMMON\]\s*(?:mode|ref):\s*-?\d+\s*->\s*-?\d+"
    r"|\[AMS_SWITCH\]\s*tray:\d+,\s*bldc slip,\s*dw_pos:\S+\s*m"
    r"|\[AMS_SWITCH\]\s*assist finish 0, ref:0"
    r"|\[AMS_PMSM\]\s*mode:\d+\s*->\s*\d+"
    r"|\[AMS_LINK\]\s*err_code:\s*0x[0-9A-Fa-f]+\s*->\s*0x[0-9A-Fa-f]+"
    r"|\[AMS_IDLE\]\s*set ams state switch"
    r"|\[AMS_COMMON\]\s*preload_disable:\d+\s*,\s*tmpr:[0-9.]+\s*,\s*cd:\d+"
    r"|\[AMS_COMMON\]\s*state:[03]\s*,\s*tray_now:\d+\s*,\s*tray_exit:\d+"
    r"|\[AMS_COMMON\]\s*en:\d+\s*,\s*mode:\d+\s*,\s*idx:\d+\s*,\s*ref:\d+"
    r"|\[AMS_RFID\]\s*STEP0:\s*checking"
    r"|\[AMS_RFID\]\s*STEP0:\s*idx\s+\d+\s*>\s*\d+"
    r"|\[AMS_PMSM\]\s*mode:\s*\d+\s*->\s*\d+"
    r"|\[AMS_LED\]\s*tray\s+\d+\s+\w+"
    r"|\[AMS_SWITCH\]\s*BUFF\s*,[^\[]*"
    r"|\[AMS_COMMON\]\s*state:4\s*,\s*tray_now:\d+\s*,\s*tray_exit:\d+"
    r"|\[AMS_IDLE\]\s*set ams state assist[^\[]*"
    r"|\[AMS_LINK\]\s*en:\d+\s*,\s*mode:\d+\s*,\s*idx:\d+\s*,\s*ref:\d+"
    r"|\[AMS_(?:LINK|CALL)\]\s*ams-?(?:0x)?[0-9A-Fa-f]+\s+select[^\[]*"
    r"|\[AMS_TRAY\]\s*tray\[?\d*\]?\s*sw_sta\s*update\s*,\s*"
    r"(?:3\s*->\s*\d+|\d+\s*->\s*3)[^\[]*"
    r"|\[AMS_BDC\]\s*tray lock:[^\[]*"
    r"|\[AMS_LED\]\s*mc set tray[^\[]*"
    r"|\[AMS_ENC\]\s*clc[^\[]*"
    r"|\[AMS_LINK\]\s*assist_err:[^\[]*"
    r"|\[AMS_SWITCH\]\s*(?:assist finish|reset dw length|retry:|AMS_CTRL_"
    r"|SWITCH_pull ignore|SWITHC_feed ignore|need to pull tray|feed tray:"
    r"|pull tray:|pull sucess)[^\[]*"
    r"|\[AMS_(?:RFID|DEV)\]\s*STEP\d*[:,]?\s*(?:odom search|set \d+ tray_readid"
    r"|rfid pull|time_reset|stop goto auth|check pass|checking|open_PCD"
    r"|reader \d+ enable|pull tension|goS\d|done\d|search \d+ card"
    r"|anticoll get UID|direct read card|empty to read|select card success"
    r"|pull back|start,read all card|ready to cali tray|no card in RF"
    r"|after tension|confirm RF have no card|odom select rslt"
    r"|odom reset tray|cali read tray)[^\[]*"
    r"|\[AMS_PMSM(?:_[A-Z])?\]\s*(?:adc\d|timeout, retry|get ams_id"
    r"|has ams_id|P cali init|table_xy)[^\[]*"
    r"))+\s*$")

_DBG_AMSTIME_RE = re.compile(r"\[DBG\]\s*ams time\b[^\[]*")


def _ams_is_noise(text: str) -> bool:
    i = text.find("[")
    return bool(text) and i >= 0 and bool(_AMS_NOISE_RE.match(text[i:]))

_FW_DIAG_RE = re.compile(r"^\s*\[(?:HT-MEAS|BB-GATE|CAP)\]")

_ACK_ROUTINE = frozenset(("assist", "select", "stop", "hold", "follow"))

_UNSET = object()

_BRIDGE_EVENTS_KNOWN = frozenset((
    "status", "mot", "reply", "error", "ack", "amsdbg", "sniff", "chain", "info",
    "sniff_mode", "slow",
    "dry", "mon", "resync", "mcaddr", "armms", "arrivems", "hb",
    "fwreplay", "txbuf", "txsend",
    "htunit", "mute", "units", "variant", "baud",
    "parity", "replay", "unload", "relink", "rehome",
    "capscan", "ht0fhold", "pathload",
    "insskip",
    "arrived", "txecho", "fw",
    "scan", "reid", "reread", "prime",
    "tx", "loops",
    "bind", "htuid",
    "wifi",
    "cali",
))

try:                                        # pragma: no cover
    from serial import SerialTimeoutException as _SerialTimeout
except Exception:                           # pragma: no cover
    class _SerialTimeout(Exception):
        pass


def split_tcp_key(raw: Optional[str]) -> Tuple[Optional[str], bool]:
    if not raw:
        return None, False
    parts = raw.split()
    store = bool(parts) and parts[-1] == "-store"
    if store:
        parts = parts[:-1]
    return (" ".join(parts) or None), store


class LinkKeyError(OSError):
    pass


class TcpPort:
    DEFAULT_PORT = 8888

    @staticmethod
    def parse(spec: str) -> Tuple[str, int]:
        raw = str(spec or "").strip()
        if "://" in raw:
            raw = raw.split("://", 1)[1]
        raw = raw.rstrip("/")
        if raw.startswith("["):
            host, _, tail = raw[1:].partition("]")
            port = tail.lstrip(":")
        elif raw.count(":") > 1:
            host, port = raw, ""
        else:
            host, _, port = raw.partition(":")
        host = host.strip()
        if not host:
            error_str = f"no host in bridge address {spec!r}"
            raise ValueError(error_str)
        try:
            return host, int(port) if port else TcpPort.DEFAULT_PORT
        except ValueError:
            error_str = f"bad port in bridge address {spec!r}"
            raise ValueError(error_str)

    def __init__(self, host: str, port: int, timeout: float = 0.1, write_timeout: float = 0.5,
                 connect_timeout: float = 3.0, key: Optional[str] = None) -> None:
        self._sock = socket.create_connection((host, int(port)), timeout=connect_timeout)
        self._timeout = float(timeout)
        self._pushback = b""
        self._sock.settimeout(timeout)
        for level, opt in ((socket.IPPROTO_TCP, socket.TCP_NODELAY),
                           (socket.SOL_SOCKET, socket.SO_KEEPALIVE)):
            try:
                self._sock.setsockopt(level, opt, 1)
            except OSError:
                pass
        for opt, val in (("TCP_KEEPIDLE", 20), ("TCP_KEEPINTVL", 5), ("TCP_KEEPCNT", 3)):
            num = getattr(socket, opt, None)
            if num is None:
                continue
            try:
                self._sock.setsockopt(socket.IPPROTO_TCP, num, val)
            except OSError:
                pass
        self._write_timeout = float(write_timeout)
        self.name = f"tcp://{host}:{int(port)}"
        self.challenged = False
        self._authenticate(key, connect_timeout)

    def _authenticate(self, key: Optional[str], timeout: float) -> None:
        deadline = time.monotonic() + max(float(timeout), 2.0)
        buf = b""
        self._sock.settimeout(0.2)
        try:
            while (time.monotonic() < deadline
                   and b"\n" not in buf):
                try:
                    chunk = self._sock.recv(256)
                except (socket.timeout, TimeoutError):
                    if not key:
                        return
                    continue
                if not chunk:
                    error_str = f"{self.name}: closed during authentication"
                    raise OSError(error_str)
                buf += chunk
            if b"\n" not in buf:
                return
            line = buf.split(b"\n", 1)[0].decode("utf-8", "replace")
            if ('"auth"' not in line
                or '"nonce"' not in line):
                self._pushback = buf
                return
            if not key:
                error_str = (f"{self.name}: the bridge asked for a link key and none is "
                             f"configured -- set tcp_key to match the board")
                raise LinkKeyError(error_str)
            self.challenged = True
            nonce = bytes.fromhex(json.loads(line)["nonce"])
            mac = hmac.new(key.encode(), nonce, hashlib.sha512).digest()[:32]
            self._sock.sendall(f'{{"cmd":"auth","mac":"{mac.hex()}"}}\n'.encode())
            reply = buf.split(b"\n", 1)[1]
            while (time.monotonic() < deadline
                   and b"\n" not in reply):
                try:
                    chunk = self._sock.recv(256)
                except (socket.timeout, TimeoutError):
                    continue
                if not chunk:
                    error_str = f"{self.name}: closed during authentication"
                    raise OSError(error_str)
                reply += chunk
            rl = reply.split(b"\n", 1)[0].decode("utf-8", "replace")
            if '"ok":1' not in rl.replace(" ", ""):
                error_str = f"{self.name}: the bridge rejected the link key"
                raise LinkKeyError(error_str)
        finally:
            self._sock.settimeout(self._timeout)

    def read(self, size: int = 1) -> bytes:
        if self._pushback:
            data, self._pushback = self._pushback[:size], self._pushback[size:]
            return data
        try:
            data = self._sock.recv(size)
        except (socket.timeout, TimeoutError, BlockingIOError):
            return b""
        if not data:
            error_str = f"{self.name}: bridge closed the connection"
            raise OSError(error_str)
        return data

    def write(self, data: bytes) -> int:
        view = memoryview(data)
        deadline = time.monotonic() + self._write_timeout
        while view:
            if time.monotonic() >= deadline:
                error_str = f"{self.name}: Write timeout"
                raise _SerialTimeout(error_str)
            try:
                sent = self._sock.send(view)
            except (socket.timeout, TimeoutError, BlockingIOError):
                continue
            if not sent:
                error_str = f"{self.name}: bridge closed the connection"
                raise OSError(error_str)
            view = view[sent:]
        return len(data)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


BB_RAW_WRITE_TIMEOUT = 5.0


_BRIDGES: Dict[str, "BambuBridge"] = {}

_WIFI_LINKS: Dict[str, "WifiLink"] = {}

_WIFI_LINK_UP = 3

_AUTO_FINDERS: Dict[str, Callable[[bool], Optional[str]]] = {}

BRIDGE_USB_GLOBS = (
    "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00",
    "/dev/serial/by-id/usb-Raspberry_Pi_Pico_*-if00",
)

PROBE_READS = 15

PROBE_ASK_EVERY = 5


def usb_port(spec: str) -> str:
    finder = _AUTO_FINDERS.get(spec)
    if finder is None:
        return spec
    path = finder(threading.current_thread() is not threading.main_thread())
    if path is None:
        error_str = "no BridgeBox found on USB yet (serial_port: auto)"
        raise OSError(error_str)
    return path


def bridge_candidates() -> List[str]:
    return sorted(p for pattern in BRIDGE_USB_GLOBS
                  for p in glob.glob(pattern))


def probe_bridge(path: str) -> Optional[dict]:
    try:
        import serial
        port = serial.Serial(path, 115200, timeout=0.1, write_timeout=0.5)
    except Exception:
        return None
    try:
        port.reset_input_buffer()
        buf = b""
        for i in range(PROBE_READS):
            if i % PROBE_ASK_EVERY == 0:
                port.write(b'\n{"cmd":"info"}\n')
            buf += port.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                obj = parse_bridge_line(raw.decode("utf-8", "replace"))
                if (obj is not None
                    and obj.get("evt") == "info"
                    and obj.get("proto") == "bambubus-1"):
                    return obj
        return None
    except Exception:
        return None
    finally:
        try:
            port.close()
        except Exception:
            pass


class WifiLink:
    RETRY_S = 60.0

    def __init__(self, key: str, save: Callable[[str], None], label: str,
                 usb_path: Optional[str] = None, addr: Optional[str] = None,
                 ssid: Optional[str] = None, password: str = "") -> None:
        self.key = key
        self._save = save
        self.label = label
        self.usb_path = usb_path
        self.addr = addr
        self.ssid = ssid
        self._password = password
        self._creds_sent = False
        self.on_wifi = False
        self._bridge: Optional["BambuBridge"] = None
        self._usb_open: Optional[Callable[[], Any]] = None
        self._failed_addr: Optional[str] = None
        self._retry_at = 0.0
        self._pending: Optional[Tuple[str, str]] = None
        self._said: Optional[str] = None

    def attach(self, bridge: "BambuBridge", usb_open: Callable[[], Any]) -> None:
        self._bridge = bridge
        self._usb_open = usb_open

    def forget(self, key: str) -> None:
        self.key = key
        if self.usb_path:
            self.addr = None
        self._failed_addr = None
        self._retry_at = 0.0
        self._pending = None
        self._said = None
        self._creds_sent = False

    def _say(self, state: str, message: str, warning: bool = False, console: bool = True,
             routine: bool = False) -> None:
        if (state == self._said
            or self._bridge is None):
            return
        self._said = state
        if not console:
            self._bridge.logger.debug(f"{self.label}: {message}")
            return
        if routine:
            routine_info(self._bridge.logger, f"{self.label}: {message}")
            return
        log = self._bridge.logger.warning if warning \
            else self._bridge.logger.info
        log(f"{self.label}: {message}")

    def open(self) -> Any:
        usb_in = bool(self.usb_path) and os.path.exists(self.usb_path)
        if (self.addr
            and (not usb_in or time.monotonic() >= self._retry_at)):
            host, port = TcpPort.parse(self.addr)
            try:
                link = TcpPort(host, port, timeout=0.1, write_timeout=0.5, key=self.key)
            except Exception as e:
                self._tcp_failed(e, usb_in)
                if not usb_in:
                    raise
            else:
                self.on_wifi = True
                self._failed_addr = None
                self._say("wifi", f"BridgeBox connected over WiFi at {self.addr}", routine=True)
                return link
        elif (not self.addr
              and not usb_in
              and self.usb_path not in _AUTO_FINDERS):
            self._say("no-addr", "connection: wifi needs the BridgeBox on "
                      "this printer's USB once, to learn its WiFi address", warning=True)
        self.on_wifi = False
        return self._usb_open()

    def _tcp_failed(self, error: Exception, usb_in: bool) -> None:
        self._failed_addr = self.addr
        self._retry_at = time.monotonic() + self.RETRY_S
        if (isinstance(error, LinkKeyError)
            and usb_in):
            self._say("key", f"the BridgeBox at {self.addr} holds another "
                      f"printer's link key; staying on USB, which "
                      f"gives it this printer's key", warning=True)
        elif isinstance(error, LinkKeyError):
            self._say("key", f"the BridgeBox at {self.addr} holds another "
                      f"printer's link key. Plug it into this "
                      f"printer's USB to give it this printer's key, "
                      f"or run AFC_BRIDGEBOX_RESET_WIFI on the printer "
                      f"that set it up.", warning=True)
        elif usb_in:
            self._say("usb", f"can't reach the BridgeBox on WiFi at "
                      f"{self.addr} ({error}); staying on USB", warning=True)
        else:
            self._say("down", f"can't reach the BridgeBox at {self.addr} "
                      f"({error}). If it is new, finish its WiFi "
                      f"setup from its bb-bridge hotspot; Klipper keeps trying.", warning=True)

    def on_connect(self, port: Any) -> None:
        self._pending = None
        if (self.on_wifi
            and self._bridge is not None
            and not getattr(port, "challenged", False)):
            self._pending = ("claim", str(self.addr))
            self._bridge.send({"cmd": "linkkey", "key": self.key})

    def on_frame(self, obj: dict) -> None:
        if obj.get("evt") == "linkkey":
            self._key_stored(obj)
        elif obj.get("cleared") == 1:
            self._cleared(obj)
        elif (not self.on_wifi
              and self.usb_path):
            self._usb_wifi_state(obj)

    def _cleared(self, obj: dict) -> None:
        if (obj.get("by") != "button"
            or self._bridge is None
            or self._said == "cleared"):
            return
        usb_in = bool(self.usb_path) and os.path.exists(self.usb_path)
        addr = self.addr
        self.forget(self.key)
        if usb_in:
            if addr:
                self._bridge.reactor.register_async_callback(lambda et: self._save(""))
            self._say("cleared", "the BridgeBox's BOOT button cleared its "
                      "WiFi and key; the link is on USB until its WiFi is set up again")
        else:
            self.addr = addr
            self._say("cleared", "the BridgeBox's BOOT button cleared its "
                      "WiFi and key. Set it up again from its "
                      "bb-bridge hotspot; Klipper keeps trying to reach it.", warning=True)
        if self.on_wifi:
            self._bridge._drop_port()

    def _usb_wifi_state(self, obj: dict) -> None:
        is_info = obj.get("evt") == "info"
        if (is_info
            and obj.get("link") != "wifi"):
            self._say("no-radio", "this BridgeBox's firmware has no WiFi, so "
                      "it stays on USB", warning=True)
            return
        if (not is_info
            and "saved" in obj):
            saved = obj.get("saved")
            if (isinstance(saved, int)
                and saved < 0):
                self._say("creds-failed", f"the BridgeBox did not store the "
                          f"WiFi network ({obj})", warning=True)
            return
        if (is_info
            and self._send_creds(obj)):
            return
        up = obj.get("wlink") if is_info else obj.get("link")
        ip = str(obj.get("ip") or "0.0.0.0")
        if (up != _WIFI_LINK_UP
            or ip == "0.0.0.0"
            or obj.get("portal")):
            if (self._creds_sent
                and obj.get("portal")):
                self._say("join-failed", f"the BridgeBox could not join "
                          f"{self.ssid}. Check wifi_ssid and wifi_password, then "
                          f"FIRMWARE_RESTART, or set it up "
                          f"from its bb-bridge hotspot.", warning=True)
            elif (obj.get("portal")
                  or (is_info and not obj.get("ssid"))):
                self._say("setup", "the BridgeBox isn't on your WiFi yet. "
                          "Join its bb-bridge hotspot from a phone "
                          "and choose your network; it moves to "
                          "WiFi by itself and stays on USB until then.")
            return
        addr = f"tcp://{ip}:{TcpPort.DEFAULT_PORT}"
        if (is_info
            and addr == self._failed_addr
            and time.monotonic() < self._retry_at):
            return
        if (self._pending == ("move", addr)
            or self._bridge is None):
            return
        self._pending = ("move", addr)
        self._bridge.send({"cmd": "linkkey", "key": self.key})

    def _send_creds(self, obj: dict) -> bool:
        if (not self.ssid
            or self._creds_sent
            or self._bridge is None):
            return False
        if (obj.get("ssid") == self.ssid
            and not obj.get("portal")):
            return False
        self._creds_sent = True
        self._bridge.send({"cmd": "wifi", "ssid": self.ssid, "pass": self._password,
                           "key": self.key})
        self._say("creds", f"giving the BridgeBox the WiFi network "
                  f"{self.ssid} from the config; it restarts to join it")
        return True

    def _key_stored(self, obj: dict) -> None:
        pending, self._pending = self._pending, None
        if (pending is None
            or self._bridge is None):
            return
        kind, addr = pending
        if obj.get("saved") not in (0, 1):
            self._say("key-failed", f"the BridgeBox did not store its link "
                      f"key ({obj})", warning=True)
            return
        if kind == "claim":
            self._say("claimed", "the BridgeBox now holds this printer's link "
                      "key, so only this printer can connect to it")
            return
        self.addr = addr
        self._failed_addr = None
        self._retry_at = 0.0
        self._bridge.reactor.register_async_callback(lambda et, a=addr: self._save(a))
        self._say("moving", f"the BridgeBox joined WiFi at {addr}; moving the link there",
                  console=False)
        self._bridge._drop_port()


def attach_wifi(serial_port: Optional[str], bridge: "BambuBridge") -> None:
    link = _WIFI_LINKS.get(str(serial_port))
    if link is not None:
        bridge.use_wifi(link)


class BambuBridge:
    NARRATION_CONSOLE_MAX_PER_S = 12
    RAW_NARRATION_CONSOLE = False
    RECONNECT_SAY_S = 10.0

    STALE_ERROR_S = 1.0

    def __init__(self, serial_factory: Callable[[], Any], reactor: Any, logger: Any) -> None:
        self._serial_factory = serial_factory
        self.reactor = reactor
        self.logger = logger
        self.store_key: Optional[str] = None
        self.wifi: Optional[WifiLink] = None
        self.name = "bridge"
        self._unit_names: Dict[int, str] = {}
        self._listeners: List[Callable[[dict], None]] = []
        self._diag_prev: Optional[Tuple[str, int, int]] = None
        self._bus_said: Optional[str] = None
        self._serial: Optional[Any] = None
        self._thread: Optional[threading.Thread] = None
        self._run = False
        self._latest: Optional[dict] = None
        self._lock = threading.Lock()
        self._last_dbg: Optional[str] = None
        self._last_human: Optional[str] = None
        self._last_chmb_t: float = 0.0
        self._last_human_t: float = 0.0
        self._chmb_by_unit: Dict[int, dict] = {}
        self._meas_live: Dict[int, bool] = {}
        self._meas_live_t: Dict[int, float] = {}
        self._rfid_term_by_addr: Dict[int, float] = {}
        self._dry_err_u: Dict[int, str] = {}
        self._dry_cfg_u: Dict[int, Dict[str, int]] = {}
        self._ht_cali_u: Dict[int, Dict[str, Any]] = {}
        self._cap_measure: Dict[int, Dict[str, Any]] = {}
        self._nar_lg: Optional[Any] = None
        self._tube_by_addr: Dict[int, float] = {}
        self._tube_by_unit: Dict[int, float] = {}
        self._dw_by_unit: Dict[int, float] = {}
        self._dw_n_by_unit: Dict[int, int] = {}
        self._dw_addr_by_unit: Dict[int, int] = {}
        self._active_unit: Optional[int] = None
        self._last_dbg_n: int = 0
        self._last_dbg_t: float = 0.0
        self._rfid_ok_t: Optional[float] = None
        self._rfid_end_t: Optional[float] = None
        self._rfid_ok_by_addr: Dict[int, float] = {}
        self._rfid_end_by_addr: Dict[int, float] = {}
        self._gave_up_by_addr: Dict[int, float] = {}
        self._no_tray_by_addr: Dict[int, float] = {}
        self._tray_sw: Dict[Tuple[int, int, int], Tuple[int, float]] = {}
        self._write_drop_t: Optional[float] = None
        self._write_timeouts: int = 0
        self._gap_timeouts_seen: int = 0
        self._down_t: Optional[float] = time.monotonic()
        self._down_epoch: int = 0
        self._last_frame_t: Optional[float] = None
        self._connected_t: Optional[float] = None
        self._grace_this_conn: bool = False
        self._spoke_since_connect: bool = True
        self._silence_logged_t: Optional[float] = None
        self._drop_logged_epoch: int = -1
        self._rfid_foreign_t: Optional[float] = None
        self._rfid_foreign_by_addr: Dict[int, float] = {}
        self._finish_seq: int = 0
        self._switch_seq: int = 0
        self._switch_text: str = ""
        self._assist_seq: int = 0
        self._finish_ok: bool = False
        self._finish_text: str = ""
        self._fault_seq: int = 0
        self._fault_text: str = ""
        self._fault_seq_by_unit: Dict[int, int] = {}
        self._fault_text_by_unit: Dict[int, str] = {}
        self._fault_seq_unattr: int = 0
        self._fault_text_unattr: str = ""
        self._tray_now_by_unit: Dict[int, int] = {}
        self._tray_release_seq_by_unit: Dict[int, int] = {}
        self._tray_release_from_by_unit: Dict[int, int] = {}
        self._err_code: Optional[int] = None
        self._err_code_t: float = 0.0
        self._err_code_by_unit: Dict[int, Tuple[int, float]] = {}
        self._bldc_i: float = 0.0
        self._chain_uids: List[str] = []
        self._chain_seq: int = 0
        self._last_raw_reply: str = ""
        self._last_idsave: Optional[tuple] = None
        self._last_fw: Optional[tuple] = None
        self._fw_raw: bool = False
        # The link stays let go until then, for the AMS updater.
        self._held_until: float = 0.0
        self._hold_lock = threading.Lock()
        self._wq: "queue.Queue" = queue.Queue(maxsize=512)
        self._wthread: Optional[threading.Thread] = None
        self._info: Optional[dict] = None
        self._sniff_sq: Optional[int] = None
        self._sniff_lost: int = 0
        self._sniff_ovr: int = 0
        self._sniff_ovr0: Optional[int] = None
        self._sniff_rf: int = 0
        self._sniff_rf0: Optional[int] = None
        self._expect_reset: bool = False
        self._bus_owner: Optional[str] = None
        self._bus_claim_t: float = 0.0
        self._mcaddr_ack: Dict[int, int] = {}
        self._fstate_last: Any = _UNSET
        self._tray_gone: bool = False
        self._said: Deque[List[Any]] = deque(maxlen=NARRATION_KEPT)
        self._buff_pos: Optional[float] = None
        self._buff_refill: Optional[Tuple[float, float, Optional[float]]] = None
        self._buff_refill_seq: int = 0
        self._mot_unit: Optional[int] = None
        self._reconnect_cbs: List[Callable[[], None]] = []
        self._fw_up_ms: Optional[int] = None
        self._fw_rebooted = False
        self._info_asked = False
        self._info_due: Optional[float] = None

    def add_listener(self, cb: Callable[[dict], None]) -> None:
        if cb not in self._listeners:
            self._listeners.append(cb)

    def remove_listener(self, cb: Callable[[dict], None]) -> None:
        try:
            self._listeners.remove(cb)
        except ValueError:
            pass

    def consume_reboot(self) -> bool:
        if not self._fw_rebooted:
            return False
        self._fw_rebooted = False
        return True

    def replay_reconnect_listeners(self) -> None:
        for cb in list(self._reconnect_cbs):
            try:
                self.reactor.register_async_callback(lambda et, c=cb: c())
            except Exception:
                try:
                    cb()
                except Exception:
                    pass

    def add_reconnect_listener(self, cb: Callable[[], None]) -> None:
        if cb not in self._reconnect_cbs:
            self._reconnect_cbs.append(cb)

    def remove_reconnect_listener(self, cb: Callable[[], None]) -> None:
        try:
            self._reconnect_cbs.remove(cb)
        except ValueError:
            pass

    def request_info(self) -> None:
        try:
            self.send({"cmd": "info"})
            self.logger.debug("BridgeBox: request_info -> sent {\"cmd\":\"info\"}")
        except Exception:
            self.logger.warning("BridgeBox: request_info FAILED to send",
                                traceback=traceback.format_exc())

    def use_wifi(self, link: WifiLink) -> None:
        link.attach(self, self._serial_factory)
        self.wifi = link
        self._serial_factory = link.open

    def start(self, defer_open: bool = False) -> None:
        try:
            self._serial = self._serial_factory()
            self._down_t = None
            self._mark_connected()
        except Exception as e:
            if (not defer_open
                and self.wifi is None):
                raise
            self._serial = None
            routine_info(self.logger, f"BridgeBox: bridge not reachable yet ({e}); the reader will "
                                      f"keep trying")
        self._run = True
        self._thread = threading.Thread(target=self._reader, name="afc_bambu_read", daemon=True)
        self._thread.start()
        self._wthread = threading.Thread(target=self._writer, name="afc_bambu_write", daemon=True)
        self._wthread.start()

    def _writer(self) -> None:
        try:
            thread_name = threading.current_thread().name
            chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
        except Exception:
            pass
        while self._run:
            try:
                data = self._wq.get(timeout=0.25)
            except queue.Empty:
                continue
            s = self._serial
            if s is None:
                continue
            try:
                s.write(data)
            except _SerialTimeout:
                self._write_drop_t = time.monotonic()
                self._write_timeouts += 1
                try:
                    _tmo = getattr(s, "_write_timeout", None)
                    if _tmo is None:
                        _tmo = getattr(s, "write_timeout", None)
                    _after = (f" after {float(_tmo):g} s"
                              if isinstance(_tmo, (int, float)) and _tmo > 0
                              else "")
                except Exception:
                    _after = ""
                self.logger.debug(f"BridgeBox: bridge busy, write of "
                                  f"'{_queued_cmd_name(data)}' timed out{_after} "
                                  f"(may still be delivered)", only_debug=True)
            except Exception as e:
                self.logger.warning(f"BridgeBox: bridge write failed: {e}; reconnecting")
                self._drop_port()
            if (not self._fw_raw
                and self._wq.qsize() > WRITE_PACE_DEPTH):
                time.sleep(WRITE_PACE_S)

    def _drain_writes(self, timeout: float = 2.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self._wq.empty():
                return True
            time.sleep(0.005)
        return self._wq.empty()

    def stop(self) -> None:
        self._run = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass

    def hold(self, seconds: float) -> None:
        """
        Let go of the link so the AMS updater can use the bridge, or take it back.

        :param seconds: how long to stay off the link at most; 0 takes it back now
        """
        if seconds <= 0:
            self._held_until = 0.0
            return
        with self._hold_lock:
            self._held_until = time.monotonic() + seconds
            self._drop_port()

    def held(self) -> bool:
        """
        Whether the link is let go for the AMS updater.

        :return bool: True while held
        """
        return time.monotonic() < self._held_until

    def last_err_code(self, unit: Optional[int] = None
                      ) -> Tuple[Optional[int], float]:
        with self._lock:
            if unit is None:
                return (self._err_code, self._err_code_t)
            return self._err_code_by_unit.get(int(unit), (None, 0.0))

    def last_scan_end(self) -> Optional[float]:
        with self._lock:
            return self._rfid_end_t

    BUS_CLAIM_MAX_S = 120.0

    def try_claim_bus(self, owner: str, now: float) -> bool:
        with self._lock:
            cur = getattr(self, "_bus_owner", None)
            if (cur is not None
                and cur != owner):
                claimed = getattr(self, "_bus_claim_t", 0.0)
                ended = self._rfid_end_t
                done = (ended is not None and ended >= claimed)
                if (not done
                    and (now - claimed) < self.BUS_CLAIM_MAX_S):
                    return False
            self._bus_owner = owner
            self._bus_claim_t = now
            return True

    def release_bus(self, owner: str) -> None:
        with self._lock:
            if getattr(self, "_bus_owner", None) == owner:
                self._bus_owner = None

    def bus_owner(self) -> Optional[str]:
        with self._lock:
            return getattr(self, "_bus_owner", None)

    def last_fault(self, unit: Optional[int] = None) -> Tuple[int, str, float]:
        with self._lock:
            if unit is None:
                return (self._fault_seq, self._fault_text, self._bldc_i)
            _u = int(unit)
            mine = self._fault_seq_by_unit.get(_u, 0)
            if mine >= self._fault_seq_unattr:
                return (mine, self._fault_text_by_unit.get(_u, ""), self._bldc_i)
            return (self._fault_seq_unattr, self._fault_text_unattr, self._bldc_i)

    def set_narration_log(self, log_dir: str, tag: str = "",
                          max_bytes: int = 10 * 1024 * 1024) -> bool:
        if self._nar_lg is not None:
            return True
        suffix = ("_" + tag) if tag else ""
        lg = logging.getLogger("AFC_BambuAMS_file" + suffix)
        if not any(isinstance(h, logging.handlers.RotatingFileHandler)
                   for h in lg.handlers):
            try:
                fh = logging.handlers.RotatingFileHandler(
                    os.path.join(log_dir, f"AFC_BambuAMS{suffix}.log"), maxBytes=max_bytes,
                    backupCount=1)
                fh.setFormatter(logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S"))
                lg.addHandler(fh)
            except Exception as e:
                self.logger.warning(f"BridgeBox: could not open AFC_BambuAMS{suffix}.log: {e}")
                return False
        lg.setLevel(logging.DEBUG)
        lg.propagate = False
        self._nar_lg = lg
        return True

    def _narrate_to_file(self, text: str, addr: Optional[int], unit: Optional[int] = None,
                         always: bool = False) -> None:
        lg = self._nar_lg
        if (lg is None
            or not text
            or not (always or self._verbose())):
            return
        try:
            lg.debug(f'{((f"0x{int(addr):04X}") if addr else "0x----")}'
                     f'{(f" u{int(unit)}" if unit is not None else " u?")} {text}')
        except Exception:
            pass

    def _verbose(self) -> bool:
        return bool(getattr(self.logger, "print_debug_console", False))

    def last_buff_pos(self) -> Optional[float]:
        with self._lock:
            return self._buff_pos

    def last_buff_refill(self) -> Optional[tuple]:
        with self._lock:
            return self._buff_refill

    def buff_refill_seq(self) -> int:
        with self._lock:
            return self._buff_refill_seq

    def motion_unit(self) -> Optional[int]:
        """
        The unit the last motion report was for, whose readings are newer than the status frame's.

        :return Optional[int]: its chain index, or None once a status frame has followed the report
        """
        with self._lock:
            return self._mot_unit

    def _note_dry_refusal(self, text: Optional[str], addr: Optional[int],
                          unit: Optional[int] = None) -> None:
        if (not text
            or not addr):
            return
        try:
            m = _DRY_REFUSED_RE.search(text)
            if m:
                if unit is None:
                    return
                with self._lock:
                    self._dry_err_u[unit] = m.group(1).strip()
            elif ("CTC_STATE_HEATING" in text
                  or "CTC_STATE_SELF_CHECK" in text
                  or "check ok!" in text
                  or "shell ok!" in text):
                if unit is None:
                    return
                with self._lock:
                    self._dry_err_u.pop(unit, None)
        except Exception:
            pass

    def _note_dry_cfg(self, text: Optional[str], addr: Optional[int],
                      unit: Optional[int] = None) -> None:
        if (not text
            or not addr):
            return
        try:
            m = _DRY_CFG_RE.search(text)
            if not m:
                return
            if unit is None:
                return
            with self._lock:
                self._dry_cfg_u[unit] = {
                    "rotate": 1 if (int(m.group(1)) or int(m.group(2))) else 0,
                    "dur":    int(m.group(5)),
                    "tmpr":   int(m.group(6)),
                }
        except Exception:
            pass

    def last_dry_cfg(self, unit: Optional[int]) -> Optional[Dict[str, int]]:
        if unit is None:
            return None
        with self._lock:
            cfg = self._dry_cfg_u.get(int(unit))
            return dict(cfg) if cfg else None

    def _attach_save_tray(self, sv: Optional[Any], addr: Optional[int]) -> None:
        if (not sv
            or not addr):
            return
        try:
            with self._lock:
                rec = self._cap_measure.get(int(addr))
                if (rec is not None
                    and rec.get("radius_m") is not None
                    and abs(round(float(sv.group(2)), 3) - float(rec["radius_m"])) < 0.0005):
                    rec["save_tray"] = int(sv.group(1))
                    rec["save_radius_m"] = float(sv.group(2))
        except Exception:
            pass

    def _note_cap_measure(self, text: Optional[str], addr: Optional[int], now: float,
                          unit: Optional[int] = None) -> None:
        if (not text
            or not addr):
            return
        try:
            c = _HT_CALI_RST_RE.search(text)
            if c:
                if unit is not None:
                    with self._lock:
                        self._ht_cali_u[unit] = {"rst": int(c.group(1)), "t": now}
            else:
                d = _CALI_DONE_RE.search(text)
                if (d
                    and unit is not None
                    and int(addr) != 0x1800):
                    with self._lock:
                        self._ht_cali_u[unit] = {"rst": 0, "t": now}
            sv = _CAP_SAVE_RE.search(text)
            m = _CAP_MEASURE_RE.search(text)
            if m:
                tray, circ, radius, pct = m.groups()
            else:
                m = _CAP_MEASURE_ALT_RE.search(text)
                if not m:
                    self._attach_save_tray(sv, addr)
                    return
                tray, radius, pct = m.groups()
                circ = None
            with self._lock:
                self._cap_measure[int(addr)] = {
                    "pct": min(100, int(pct)),
                    "pct_raw": int(pct),
                    "radius_m": float(radius),
                    "circumference_m": float(circ) if circ else None,
                    "tray": int(tray) if tray is not None else None,
                    "restored": tray is not None,
                    "save_tray": None,
                    "save_radius_m": None,
                    "t": now,
                }
            self._attach_save_tray(sv, addr)
        except Exception:
            pass

    def last_ht_cali(self, unit: Optional[int]) -> Optional[Dict[str, Any]]:
        if unit is None:
            return None
        with self._lock:
            m = self._ht_cali_u.get(int(unit))
            return dict(m) if m else None

    def cap_calibrating(self, addr: Optional[int]) -> bool:
        if not addr:
            return False
        with self._lock:
            if not self._meas_live.get(int(addr)):
                return False
            t0 = self._meas_live_t.get(int(addr))
            mono = getattr(self.reactor, "monotonic", None)
            now = mono() if callable(mono) else 0.0
            if (t0 is not None
                and (now - t0) > _MEAS_LIVE_MAX_S):
                self._meas_live[int(addr)] = False
                return False
            return True

    def last_terminal(self, addr: Optional[int]) -> Optional[float]:
        if not addr:
            return None
        with self._lock:
            return self._rfid_term_by_addr.get(int(addr))

    def last_cap_measure(self, addr: Optional[int]) -> Optional[Dict[str, Any]]:
        if not addr:
            return None
        with self._lock:
            m = self._cap_measure.get(int(addr))
            return dict(m) if m else None

    def clear_dry_error(self, unit: Optional[int]) -> None:
        if unit is None:
            return
        with self._lock:
            self._dry_err_u.pop(int(unit), None)

    def last_dry_error(self, unit: Optional[int]) -> Optional[str]:
        if unit is None:
            return None
        with self._lock:
            return self._dry_err_u.get(int(unit))

    def _finish_succeeded(self, text: str, low: str, addr: Optional[int]) -> bool:
        if ("finish -1" not in low
            and "stall" not in low):
            return True
        if _CLEAN_FINISH_RE.search(text):
            return True
        det = _LEN_DET_M_RE.search(text)
        if not det:
            return False
        travelled = float(det.group(1)) * 1000.0
        tube = _TUBE_LEN_M_RE.search(text)
        target = (float(tube.group(1)) * 1000.0 if tube
                  else self.tube_len(addr))
        if not target:
            return False
        return travelled >= target - FINISH_ARRIVAL_TOLERANCE_MM

    def _trace_fstate(self, obj: dict) -> None:
        if (self._nar_lg is None
            or not self._verbose()):
            return
        try:
            v = obj.get("fstate")
            if v == self._fstate_last:
                return
            prev, self._fstate_last = self._fstate_last, v
            self._nar_lg.debug(f'HOST-- fstate {("-" if prev is _UNSET else prev)} -> {v} (buff='
                               f'{obj.get("buff")})')
        except Exception:
            pass

    def name_unit(self, unit: int, name: str) -> None:
        self._unit_names[int(unit)] = str(name)

    def _speaker(self, unit: Optional[int]) -> str:
        if unit is None:
            unit = self._active_unit
        if unit is None:
            return self.name
        return self._unit_names.get(int(unit), self.name)

    def set_active_unit(self, unit: Optional[int]) -> None:
        with self._lock:
            self._active_unit = None if unit is None else int(unit)

    def dw_len(self, unit: Optional[int] = None) -> tuple:
        with self._lock:
            if unit is not None:
                u = int(unit)
                return (self._dw_by_unit.get(u), self._dw_n_by_unit.get(u, 0),
                        self._dw_addr_by_unit.get(u, 0))
            if not self._dw_by_unit:
                return (None, 0, 0)
            u = max(self._dw_n_by_unit, key=self._dw_n_by_unit.get)
            return (self._dw_by_unit.get(u), self._dw_n_by_unit.get(u, 0),
                    self._dw_addr_by_unit.get(u, 0))

    def tube_len(self, addr: Optional[int] = None, unit: Optional[int] = None) -> Optional[float]:
        with self._lock:
            if unit is not None:
                if int(unit) in self._tube_by_unit:
                    return self._tube_by_unit[int(unit)]
                if self._tube_by_unit:
                    return None
            if addr is not None:
                v = self._tube_by_addr.get(int(addr))
                if v:
                    return v
                return None
            vals = list(self._tube_by_addr.values())
            return vals[0] if len(vals) == 1 else None

    def last_finish(self) -> Tuple[int, bool, str]:
        with self._lock:
            return (self._finish_seq, self._finish_ok, self._finish_text)

    def chip(self) -> Optional[str]:
        with self._lock:
            v = (self._info or {}).get("chip")
        return str(v) if v else None

    def last_switch_finish(self) -> Tuple[int, str]:
        with self._lock:
            return (self._switch_seq, self._switch_text)

    def last_tray_release(self, unit: Optional[int] = None) -> Tuple[int, Optional[int]]:
        if unit is None:
            return (0, None)
        with self._lock:
            _u = int(unit)
            return (self._tray_release_seq_by_unit.get(_u, 0),
                    self._tray_release_from_by_unit.get(_u))

    def narrates_tray(self, unit: Optional[int]) -> bool:
        """
        Whether a unit reports its selected tray in its own narration.

        :param unit: the unit's chain index
        :return bool: True once a `tray_now` report was heard from that unit
        """
        if unit is None:
            return False
        with self._lock:
            return int(unit) in self._tray_now_by_unit

    def _keep_said(self, text: str, unit: Optional[int], now: float) -> None:
        """
        Keep a narration line for a later report, folding a repeat into its entry.

        :param text: the unit's line
        :param unit: the chain index the line was attributed to, when known
        :param now: the reactor time it arrived
        """
        with self._lock:
            last = self._said[-1] if self._said else None
            if (last is not None
                and last[1] == unit
                and last[2] == text):
                last[0] = now
                last[3] += 1
                return
            self._said.append([now, unit, text, 1])

    def said_since(self, since: float, unit: Optional[int] = None) -> List[Tuple[str, int]]:
        """
        The narration lines last heard at or after a time, oldest first.

        :param since: the reactor time to report from
        :param unit: leave out the lines attributed to another unit
        :return list: (line, times heard in a row) per entry
        """
        out: List[Tuple[str, int]] = []
        with self._lock:
            for when, who, text, count in self._said:
                if when < since:
                    continue
                if (unit is not None
                    and who is not None
                    and who != int(unit)):
                    continue
                out.append((text, count))
        return out

    def last_assist_done(self) -> int:
        with self._lock:
            return self._assist_seq

    def latest_status(self) -> Optional[dict]:
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def write_raw(self, data: bytes) -> bool:
        s = self._serial
        if s is None:
            return False
        if not self._drain_writes():
            self.logger.warning("BridgeBox: queued writes did not drain; not starting the image")
            return False
        _tmo_attr = ("_write_timeout" if hasattr(s, "_write_timeout")
                     else ("write_timeout" if hasattr(s, "write_timeout")
                           else None))
        _tmo_prev = getattr(s, _tmo_attr) if _tmo_attr else None
        if _tmo_attr:
            try:
                setattr(s, _tmo_attr, BB_RAW_WRITE_TIMEOUT)
            except Exception:
                _tmo_attr = None
        try:
            view = memoryview(data)
            step = 4096
            while view:
                n = min(step, len(view))
                s.write(view[:n])
                view = view[n:]
            return True
        except _SerialTimeout as e:
            self.logger.warning(f"BridgeBox: bridge busy mid-transfer: {e}")
            return False
        except Exception as e:
            self.logger.warning(f"BridgeBox: bridge raw write failed: {e}; reconnecting")
            self._drop_port()
            return False
        finally:
            if _tmo_attr:
                try:
                    setattr(s, _tmo_attr, _tmo_prev)
                except Exception:
                    pass

    def send(self, obj: dict) -> None:
        s = self._serial
        if s is None:
            if self.held():
                return
            if self._drop_logged_epoch != self._down_epoch:
                self._drop_logged_epoch = self._down_epoch
                log = self.logger.info if self._down_epoch == 0 \
                    else self.logger.warning
                log(f"BridgeBox: link is down -- dropped "
                    f"{obj.get('cmd', '?')}, and anything else sent until it is back")
            return
        if (self._fw_raw
            and not str(obj.get("cmd", "")).startswith("fw")):
            self.logger.debug(f"BridgeBox: dropped {obj.get('cmd')} during a firmware transfer")
            return
        if obj.get("cmd") in ("reset", "bootsel"):
            self._expect_reset = True
        try:
            self._wq.put_nowait((json.dumps(obj) + "\n").encode())
            return
        except queue.Full:
            self.logger.warning(f"BridgeBox: write queue full, dropped {obj.get('cmd')}")

    def _narrate_human(self, text: str, now: float, addr: Optional[int] = None,
                       unit: Optional[int] = None) -> None:
        for entry in _AMS_HUMAN:
            pattern, render = entry[0], entry[1]
            log_only = len(entry) > 2 and entry[2] is True
            if (len(entry) > 2
                and entry[2] == "routine"
                and not self._verbose()):
                log_only = True
            m = pattern.search(text)
            if not m:
                continue
            if render is None:
                return
            msg = f"{self._speaker(unit)}: {render(m)}"
            if log_only:
                if self._verbose():
                    self.logger.debug(msg, only_debug=True)
                continue
            if msg == getattr(self, "_last_human", None):
                continue
            if now - getattr(self, "_last_human_t", 0.0) < 1.0:
                continue
            self._last_human = msg
            self._last_human_t = now
            self.logger.info(msg)
        mm = None
        tl = _TUBE_LEN_MM_RE.search(text)
        if tl:
            mm = float(tl.group(1))
        else:
            tl = _TUBE_LEN_M_RE.search(text)
            if tl:
                mm = float(tl.group(1)) * 1000.0
        if (mm
            and mm > 0.0
            and addr):
            with self._lock:
                prev = self._tube_by_addr.get(int(addr))
                self._tube_by_addr[int(addr)] = mm
                if self._active_unit is not None:
                    self._tube_by_unit[int(self._active_unit)] = mm
            if prev is None:
                self.logger.debug(f"{self.name}: AMS 0x{int(addr):04X} reports its "
                                  f"measured filament path as {mm:.0f}mm -- using it to size "
                                  f"move timeouts instead of the configured estimate")

        dw = _DW_LEN_M_RE.search(text)
        if dw:
            dw_mm = float(dw.group(1)) * 1000.0
            if (dw_mm > 0.0
                and self._active_unit is not None):
                with self._lock:
                    u = int(self._active_unit)
                    self._dw_by_unit[u] = dw_mm
                    self._dw_n_by_unit[u] = self._dw_n_by_unit.get(u, 0) + 1
                    self._dw_addr_by_unit[u] = int(addr or 0)

        if ("[AMS_CHMB]" in text
            and ("res:" in text or "i_avr:" in text or " i:" in text)):
            self.logger.debug(f"{self.name}: selfcheck | "
                             + text.split("[AMS_CHMB]", 1)[-1].strip()[:150])
        m = _CHMB_STATE_RE.search(text)
        if m:
            try:
                rec = {"temp": float(m.group(3)), "seen": now, "state": int(m.group(1)),
                       "target": float(m.group(2))}
                if m.group(4) is not None:
                    rec["humidity"] = int(m.group(4))
            except (TypeError, ValueError):
                rec = None
            if (rec is not None
                and unit is not None):
                ma = _CHMB_AD_RE.search(text)
                if ma:
                    rec["ad_n"] = int(ma.group(1))
                    if ma.group(2) is not None:
                        try:
                            rec["ad_v"] = float(ma.group(2))
                        except ValueError:
                            pass
                self._chmb_by_unit[int(unit)] = rec
        if (m
            and now - getattr(self, "_last_chmb_t", 0.0) >= 60.0):
            self._last_chmb_t = now
            hum = f", humidity {int(m.group(4))}%" if m.group(4) else ""
            ma = _CHMB_AD_RE.search(text)
            ad = ""
            if ma:
                ad = (f", ad {ma.group(1)}"
                      + (f"/{ma.group(2)}" if ma.group(2) else ""))
            self.logger.debug(f"{self.name}: drying -- chamber {m.group(3)}C{ad}"
                              f"{hum}, target {m.group(2)}C")

    def _rfid_stamp(self, wide: Optional[float], by_addr: Dict[int, float],
                    addr: Optional[int]) -> Optional[float]:
        if addr:
            got = by_addr.get(int(addr))
            if got is not None:
                return got
            if by_addr:
                return None
        return wide

    def rfid_read_succeeded_since(self, since: Optional[float], addr: Optional[int] = None) -> bool:
        ok = self._rfid_stamp(self._rfid_ok_t, self._rfid_ok_by_addr, addr)
        return ok is not None and since is not None and ok >= since

    def rfid_foreign_tag_since(self, since: Optional[float], addr: Optional[int] = None) -> bool:
        t = self._rfid_stamp(self._rfid_foreign_t, self._rfid_foreign_by_addr, addr)
        return t is not None and since is not None and t >= since

    def gave_up_since(self, since: Optional[float], addr: Optional[int] = None) -> bool:
        if since is None:
            return False
        t = self._rfid_stamp(None, self._gave_up_by_addr, addr)
        return t is not None and t >= since

    def no_tray_since(self, since: Optional[float], addr: Optional[int] = None) -> bool:
        if since is None:
            return False
        t = self._rfid_stamp(None, self._no_tray_by_addr, addr)
        return t is not None and t >= since

    def tray_switches(self, addr: Optional[int], unit: Optional[int],
                      slot: int) -> Optional[Tuple[int, float]]:
        if (not addr
            or unit is None):
            return None
        return self._tray_sw.get((int(addr), int(unit), int(slot)))

    def writes_dropped_since(self, since: Optional[float]) -> bool:
        if since is None:
            return False
        t = self._write_drop_t
        return t is not None and t >= since

    def rfid_cycle_ended_since(self, since: Optional[float], addr: Optional[int] = None) -> bool:
        end = self._rfid_stamp(self._rfid_end_t, self._rfid_end_by_addr, addr)
        return end is not None and since is not None and end >= since

    @staticmethod
    def _motion_entry(unit: Any, obj: dict) -> Any:
        """
        A status frame's unit entry with a motion report folded in.

        :param unit: the entry as the status frame listed it
        :param obj: the motion report
        :return Any: the entry, updated when the report is for that unit
        """
        if (not isinstance(unit, dict)
            or unit.get("n") != obj.get("u")):
            return unit
        out = dict(unit)
        if "odom" in obj:
            out["odom"] = obj["odom"]
        # The report carries the unit's own reading once it has one, which is when its entry does.
        if ("buff" in obj
            and "buff" in unit):
            out["buff"] = obj["buff"]
        return out

    def _merge_motion(self, obj: dict) -> None:
        """
        Fold a motion report into the latest status frame.

        :param obj: the motion report
        """
        with self._lock:
            latest = self._latest
            if not isinstance(latest, dict):
                return
            merged = dict(latest)
            for key in ("buff", "buffn"):
                if key in obj:
                    merged[key] = obj[key]
            units = latest.get("units")
            if isinstance(units, list):
                merged["units"] = [self._motion_entry(u, obj) for u in units]
            self._latest = merged
            self._mot_unit = obj.get("u")

    def handle_line(self, line: str) -> None:
        obj = parse_bridge_line(line)
        if obj is None:
            return
        if (self.wifi is not None
            and obj.get("evt") in ("info", "wifi", "linkkey")):
            self.wifi.on_frame(obj)
        if obj.get("evt") == "status":
            with self._lock:
                self._latest = obj
                self._mot_unit = None
            if (not self._info_asked
                or (self._info_due is not None
                    and time.monotonic() >= self._info_due)):
                self._info_asked = True
                self._info_due = None
                self.request_info()
            self._trace_fstate(obj)
            for cb in self._listeners:
                self.reactor.register_async_callback(lambda et, o=obj, c=cb: c(o))
        elif obj.get("evt") == "mot":
            self._merge_motion(obj)
        elif obj.get("evt") in ("reply", "raw"):
            with self._lock:
                self._last_raw_reply = str(obj.get("hex") or obj.get("rx") or "")
        elif obj.get("evt") == "error":
            since = (time.monotonic() - self._connected_t
                     if self._connected_t is not None else None)
            if (obj.get("msg") == "unknown command"
                and since is not None
                and since < self.STALE_ERROR_S):
                self.logger.debug(f"BridgeBox: bridge error {since:.2f}s after connect, "
                                  f"from a line sent before it (ignored): {obj.get('msg')}",
                                  only_debug=True)
            else:
                self.logger.warning(f"BridgeBox: bridge error: {obj.get('msg')}")
        elif obj.get("evt") == "uartdiag":
            self._note_uartdiag(obj)
        elif obj.get("evt") == "fw":
            state = str(obj.get("state") or "")
            detail = obj.get("detail")
            if (detail is None
                and "want" in obj):
                detail = (f"{obj.get('got')}/{obj.get('len')} bytes, "
                          f"computed 0x{int(obj.get('crc', 0)):08X}, "
                          f"expected 0x{int(obj.get('want', 0)):08X}, head {obj.get('head', '?')}"
                          f", tail {obj.get('tail', '?')}")
            elif (detail is None
                  and "got" in obj):
                detail = f"{obj.get('got')}/{obj.get('len')} bytes"
            with self._lock:
                self._last_fw = (state, str(detail or ""))
            if state in ("crc_bad", "stalled", "refused", "busy"):
                self.logger.warning(f"BridgeBox: firmware transfer {state}: {detail}")
            elif state == "applying":
                self._expect_reset = True
                # An S3 on USB restarts without its port closing, so no reconnect asks.
                self._info_due = time.monotonic() + self.INFO_AFTER_APPLY_S
                self.logger.info("BridgeBox: bridge is writing its own flash and will "
                                 "reboot; the link drops for a few seconds")
            else:
                self.logger.debug(f"BridgeBox: firmware transfer {state}: {detail}")
        elif obj.get("evt") == "idsave":
            state = obj.get("state")
            n = obj.get("n")
            with self._lock:
                self._last_idsave = (str(state or ""), n)
            if state == "wiped":
                self.logger.info("BridgeBox: bridge ERASED its stored unit identities; the "
                                 "next power-up will enroll from announce order until prep "
                                 "writes a fresh record")
            elif state == "written":
                routine_info(self.logger, f"BridgeBox: bridge stored {n} unit identities -- the next "
                                          f"restart will enroll from them")
            elif state == "failed":
                self.logger.warning("BridgeBox: bridge could NOT store the unit identities; "
                                    "the chain will keep enrolling from announce order")
            else:
                self.logger.debug(f"BridgeBox: bridge identities already stored ({n} units)")
        elif obj.get("evt") == "ack":
            _ack_cmd = str(obj.get("cmd") or "")
            if (_ack_cmd not in _ACK_ROUTINE
                or self._verbose()):
                self.logger.debug(f"BridgeBox: bridge ack {_ack_cmd} (slot {obj.get('slot')})",
                                  only_debug=_ack_cmd in _ACK_ROUTINE)
        elif obj.get("evt") == "amsdbg":
            text = obj.get("text")
            mono = getattr(self.reactor, "monotonic", None)
            now = mono() if callable(mono) else 0.0
            _dbg_unit = obj.get("unit")
            _dbg_unit = (int(_dbg_unit)
                         if isinstance(_dbg_unit, int) and _dbg_unit >= 0
                         else None)
            self._narrate_to_file(text, obj.get("addr"), _dbg_unit)
            if (isinstance(text, str)
                and _FW_DIAG_RE.match(text)):
                if self._verbose():
                    self.logger.debug(f"BridgeBox: bridge diag {text}", only_debug=True)
                return
            self._note_dry_refusal(text, obj.get("addr"), _dbg_unit)
            self._note_dry_cfg(text, obj.get("addr"), _dbg_unit)
            self._note_cap_measure(text, obj.get("addr"), now, _dbg_unit)
            _addr = obj.get("addr")
            _addr = int(_addr) if isinstance(_addr, int) and _addr else None
            _read_re = (_RFID_READ_OK_HT_RE if _addr == 0x1800
                        else _RFID_READ_OK_RE)
            if (text
                and _read_re.search(text)):
                self._rfid_ok_t = now
                if _addr:
                    self._rfid_ok_by_addr[_addr] = now
            if (text
                and _addr
                and "first detected" in text):
                self._meas_live[_addr] = True
                self._meas_live_t[_addr] = now
            if (text
                and _RFID_CYCLE_END_RE.search(text)):
                self._rfid_end_t = now
                if _addr:
                    self._rfid_end_by_addr[_addr] = now
                    self._meas_live[_addr] = False
            if (text
                and _addr
                and _MEAS_END_RE.search(text)):
                self._meas_live[_addr] = False
            if (text
                and _addr
                and _RFID_TERMINAL_RE.search(text)):
                self._rfid_term_by_addr[_addr] = now
            if (text
                and _addr
                and _AMS_GAVE_UP_RE.search(text)):
                self._gave_up_by_addr[_addr] = now
                routine_info(self.logger, f"BridgeBox: the AMS reported it has STOPPED "
                                          f"retrying ({text.strip()[:90]})")
            if (text
                and _addr
                and _AMS_NO_TRAY_RE.search(text)):
                self._no_tray_by_addr[_addr] = now
            if (text
                and _addr
                and _dbg_unit is not None):
                for m in _AMS_TRAY_SW_RE.finditer(text):
                    self._tray_sw[(_addr, _dbg_unit, int(m.group(1)))] = (int(m.group(3)), now)
            if (text
                and _addr
                and ("second detected" in text or "odom save" in text)):
                self._meas_live[_addr] = False
            if (text
                and _RFID_FOREIGN_TAG_RE.search(text)):
                self._rfid_foreign_t = now
                if _addr:
                    self._rfid_foreign_by_addr[_addr] = now
            if (text
                and "[DBG] ams time" in text):
                stripped = _DBG_AMSTIME_RE.sub("", text).strip()
                text = stripped if "[" in stripped else None
            raw = text
            if (text
                and text == self._last_dbg):
                self._last_dbg_n += 1
                if now - self._last_dbg_t >= 60.0:
                    self._last_dbg_t = now
                    n = self._last_dbg_n
                    self.logger.debug(f"AMS: (x{n} repeated) {text}", only_debug=True)
                text = None
            elif text:
                self._last_dbg = text
                self._last_dbg_n = 1
                self._last_dbg_t = now
            if raw:
                self._keep_said(raw.strip(), _dbg_unit, now)
                bp = _BUFF_POS_RE.search(raw)
                if bp:
                    with self._lock:
                        self._buff_pos = float(bp.group(1))
                rf = _BUFF_REFILL_RE.search(raw)
                if rf:
                    det = float(rf.group(3)) if rf.group(3) else None
                    with self._lock:
                        self._buff_pos = float(rf.group(2))
                        self._buff_refill = (float(rf.group(1)), float(rf.group(2)), det)
                        self._buff_refill_seq += 1
                low = raw.lower()
                if _ASSIST_DONE_RE.search(raw):
                    with self._lock:
                        self._assist_seq += 1
                if _STATE_SWITCH_DONE_RE.search(raw):
                    with self._lock:
                        self._switch_seq += 1
                        self._switch_text = raw
                if _dbg_unit is not None:
                    _tn = _TRAY_NOW_RE.findall(raw)
                    if _tn:
                        _u = int(_dbg_unit)
                        with self._lock:
                            _prev = self._tray_now_by_unit.get(_u)
                            for _v in (int(x) for x in _tn):
                                if (_prev is not None
                                    and _prev != TRAY_NONE
                                    and _v == TRAY_NONE):
                                    self._tray_release_seq_by_unit[_u] = (
                                        self._tray_release_seq_by_unit
                                        .get(_u, 0) + 1)
                                    self._tray_release_from_by_unit[_u] = _prev
                                _prev = _v
                            self._tray_now_by_unit[_u] = _prev
                if _MOTION_FINISH_RE.search(raw):
                    ok = self._finish_succeeded(raw, low, obj.get("addr"))
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = ok
                        self._finish_text = raw
                    self._tray_gone = False
                elif _ODOM_RESET_RE.search(raw):
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = True
                        self._finish_text = raw
                    self._tray_gone = False
                elif (_ODOM_NO_TRAY_RE.search(raw)
                      and not self._tray_gone):
                    self._tray_gone = True
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = True
                        self._finish_text = raw
                try:
                    self._narrate_human(raw, now, obj.get("addr"), _dbg_unit)
                except Exception:
                    pass
                mi = _BLDC_I_RE.search(raw)
                if mi:
                    try:
                        with self._lock:
                            self._bldc_i = float(mi.group(1))
                    except ValueError:
                        pass
                if ("rocker stall" in low
                    and "tray_cnt" in low):
                    pass
                elif ("stall" in low
                      or "finish -1" in low
                      or "timeout error" in low
                      or "state:6" in low
                      or "en:0,mode:7" in low):
                    with self._lock:
                        self._fault_seq += 1
                        self._fault_text = raw.strip()
                        if _dbg_unit is not None:
                            _u = int(_dbg_unit)
                            self._fault_seq_by_unit[_u] = self._fault_seq
                            self._fault_text_by_unit[_u] = raw.strip()
                        else:
                            self._fault_seq_unattr = self._fault_seq
                            self._fault_text_unattr = raw.strip()
                me = _ERR_CODE_RE.search(raw)
                if me:
                    try:
                        raw = me.group(2)
                        val = int(raw, 16) if raw.lower().startswith("0x") \
                            else int(raw, 10)
                        with self._lock:
                            self._err_code = val
                            self._err_code_t = now
                            if _dbg_unit is not None:
                                self._err_code_by_unit[int(_dbg_unit)] = (val, now)
                    except ValueError:
                        pass
                if text:
                    _quiet = (not self.RAW_NARRATION_CONSOLE
                              or _ams_is_noise(
                                  _DBG_AMSTIME_RE.sub("", text).strip()))
                    if not _quiet:
                        _now = time.monotonic()
                        _win = getattr(self, "_narr_win", 0.0)
                        if _now - _win >= 1.0:
                            self._narr_win = _now
                            self._narr_n = 0
                            self._narr_said = False
                        self._narr_n = getattr(self, "_narr_n", 0) + 1
                        if self._narr_n > self.NARRATION_CONSOLE_MAX_PER_S:
                            _quiet = True
                            if not getattr(self, "_narr_said", False):
                                self._narr_said = True
                                self.logger.debug("BridgeBox: the AMS is narrating faster "
                                                  "than the console can take "
                                                  f"(>{self.NARRATION_CONSOLE_MAX_PER_S}/s); "
                                                  "the rest of this burst is in AFC.log")
                    if self._verbose():
                        self.logger.debug(f"AMS: {text}", only_debug=_quiet)
        elif obj.get("evt") == "linkkey":
            saved = obj.get("saved")
            if saved == 1:
                self.logger.debug("BridgeBox: link key stored on the bridge; it is asked "
                                  "for from the next connection")
            elif saved == 0:
                self.logger.debug("BridgeBox: link key already stored on the bridge",
                                  only_debug=True)
            else:
                self.logger.warning(f"BridgeBox: the bridge did not store the link key {obj}")
        elif obj.get("evt") == "mcaddr":
            try:
                with self._lock:
                    self._mcaddr_ack[int(obj.get("unit", 0))] = \
                        int(obj.get("addr", 0))
            except Exception:
                pass
        elif obj.get("evt") not in _BRIDGE_EVENTS_KNOWN:
            self.logger.debug(f"BridgeBox: unhandled bridge event {obj}", only_debug=True)
        elif obj.get("evt") == "txecho":
            try:
                self._txecho_drops = int(obj.get("drops", 0) or 0)
            except Exception:
                self._txecho_drops = 0
            if self._txecho_drops:
                self.logger.info(f"BridgeBox: TX echo dropped "
                                 f"{self._txecho_drops} frame(s) -- the USB link fell "
                                 f"behind the bus. Frames MISSING from this capture were "
                                 f"still transmitted; do not read a gap as a frame we never sent.")
        elif obj.get("evt") == "loops":
            try:
                self._narrate_to_file(f'{{"evt":"loops","n":{int(obj.get("n", 0))},'
                                      f'"us":{int(obj.get("us", 0))}}}', None)
            except Exception:
                pass
        elif obj.get("evt") == "slow":
            if not self._verbose():
                return
            try:
                self.logger.debug(f"BridgeBox: bridge loop held {int(obj.get('ms', 0))} ms "
                                  f"(bus reads {int(obj.get('rr_ms', 0))} ms over "
                                  f"{int(obj.get('rr_n', 0))}, longest "
                                  f"{int(obj.get('rr_max_us', 0))} us, capped "
                                  f"{int(obj.get('capped', 0))}, capscans "
                                  f"{int(obj.get('capscan', 0))})", only_debug=True)
            except Exception:
                pass
        elif obj.get("evt") == "tx":
            try:
                _dir = str(obj.get("dir", "tx"))
                self._narrate_to_file(
                    f'{{"evt":"tx","dir":"{_dir}",'
                    f'"us":{int(obj.get("us", 0))},'
                    f'"n":{int(obj.get("n", 0))},'
                    + (f'"s":"{obj.get("s")}",' if obj.get("s") else "")
                    + f'"hex":"{obj.get("hex", "")}"}}', None, always=True)
            except Exception:
                pass
        elif obj.get("evt") == "sniff":
            _sq, _ov, _rf = obj.get("sq"), obj.get("ov"), obj.get("rf")
            _us = obj.get("us")
            self.logger.debug(
                f"SNIFF {obj.get('hex')}"
                + (f" sq={_sq} ov={_ov} rf={_rf} us={_us}" if _sq is not None
                   else ""),
                only_debug=True)
            if _sq is not None:
                prev = self._sniff_sq
                self._sniff_sq = int(_sq)
                if (prev is not None
                    and int(_sq) != prev + 1):
                    self._sniff_lost += max(0, int(_sq) - prev - 1)
            if _ov is not None:
                if self._sniff_ovr0 is None:
                    self._sniff_ovr0 = int(_ov)
                self._sniff_ovr = int(_ov) - self._sniff_ovr0
            if _rf is not None:
                if self._sniff_rf0 is None:
                    self._sniff_rf0 = int(_rf)
                self._sniff_rf = int(_rf) - self._sniff_rf0
        elif obj.get("evt") == "sniff_mode":
            on = bool(obj.get("on"))
            with self._lock:
                if on:
                    self._sniff_sq = None
                    self._sniff_lost = 0
                    self._sniff_ovr = 0
                    self._sniff_ovr0 = None
                    self._sniff_rf = 0
                    self._sniff_rf0 = None
                    verdict = ""
                else:
                    verdict = (
                        " -- capture LOSSLESS"
                        if (not self._sniff_lost
                            and not self._sniff_ovr
                            and not self._sniff_rf) else
                        f" -- capture INCOMPLETE: {self._sniff_lost} blob(s) "
                        f"lost, {self._sniff_ovr} UART overrun(s), "
                        f"{self._sniff_rf} ring lap(s). Usable for "
                        f"protocol work, NOT for reconstructing a firmware "
                        f"image")
            mode_str = ("ON -- listen-only, this bridge is NOT driving the bus"
                        if on else "OFF -- bus master again")
            self.logger.info(f"{self.name}: bridge sniff mode {mode_str}{verdict}")
        elif obj.get("evt") == "info":
            with self._lock:
                self._info = dict(obj)
            self.logger.debug(f"BridgeBox: info REPLY chip={obj.get('chip')} "
                              f"fw={obj.get('fw')} up={obj.get('up')}")
            try:
                up = obj.get("up")
                up = None if up is None else int(up)
            except Exception:
                up = None
            if up is not None:
                prev = self._fw_up_ms
                if (prev is not None
                    and up < prev
                    and self._expect_reset):
                    # Restarted in place: the port stayed open, so no reconnect follows.
                    self._expect_reset = False
                    self.logger.info(f"BridgeBox: the BridgeBox restarted on {obj.get('fw')}")
                elif (prev is not None
                      and up < prev):
                    self._fw_rebooted = True
                    self.logger.debug(f"BridgeBox: bridge REBOOTED (uptime {prev} -> {up} "
                                      f"ms); units on the wire lost their master")
                    self.logger.info("BridgeBox: the BridgeBox restarted; "
                                     "reconnecting its AMS units")
                self._fw_up_ms = up
        elif obj.get("evt") == "wifi":
            try:
                if "cleared" in obj:
                    msg = (f"{self.name}: bridge WiFi wiped, "
                           f"cleared={obj.get('cleared')} by={obj.get('by', 'command')}")
                else:
                    msg = (f"{self.name}: pico wifi "
                           f"link={obj.get('link')} ip={obj.get('ip')} "
                           f"assoc={obj.get('assoc')} tries={obj.get('tries')}")
                self.logger.debug(msg)
            except Exception:
                pass
        elif obj.get("evt") == "cali":
            try:
                _extra = " ".join(f"{k}={v}" for k, v in obj.items()
                                  if k not in ("evt", "unit", "slot"))
                self.logger.debug(
                    f"BridgeBox: bridge answered cali "
                    f"(unit {obj.get('unit')}, slot {obj.get('slot')})"
                    + (f" {_extra}" if _extra else ""),
                    only_debug=True)
            except Exception:
                pass
        elif obj.get("evt") == "chain":
            raw = (obj.get("uids") or "").strip().upper()
            uids = [] if not raw else [u.strip() for u in raw.split(",")]
            with self._lock:
                self._chain_seq = getattr(self, "_chain_seq", 0) + 1
                self._chain_uids = uids
                try:
                    self._chain_htmask = int(obj.get("htmask") or 0)
                except Exception:
                    self._chain_htmask = 0
                try:
                    cm = obj.get("capmask")
                    self._chain_capmask = None if cm is None else int(cm)
                except Exception:
                    self._chain_capmask = None
                self._chain_fw = str(obj.get("fw") or "")
                try:
                    self._chain_a2mask = int(obj.get("a2mask") or 0)
                    self._chain_a2asks = [
                        int(x) for x in
                        str(obj.get("a2asks") or "").split(",") if x]
                except Exception:
                    self._chain_a2mask = 0
                    self._chain_a2asks = []
                self._chain_tags = str(obj.get("tags") or "")
                try:
                    self._chain_capn = int(obj.get("capn") or 0)
                    self._chain_capdiag = int(obj.get("capdiag") or 0)
                    _cw = obj.get("capwhy")
                    self._chain_capwhy = None if _cw is None else int(_cw)
                    _u = obj.get("capwhyu")
                    _s = obj.get("capwhys")
                    self._chain_capwhy_unit = None if _u is None else int(_u)
                    self._chain_capwhy_slot = None if _s is None else int(_s)
                except Exception:
                    self._chain_capn = 0
                    self._chain_capdiag = 0
                    self._chain_capwhy = None
                    self._chain_capwhy_unit = None
                    self._chain_capwhy_slot = None
                mc = obj.get("mcaddr")
                self._chain_mcaddr = ([int(x) for x in mc] if isinstance(mc, list) else None)
                try:
                    self._chain_sel = (int(obj.get("selid", -1)), int(obj.get("selsent", 0)),
                                       int(obj.get("selack", 0)))
                except Exception:
                    self._chain_sel = (-1, 0, 0)

    def chain_uids(self) -> List[str]:
        with self._lock:
            return list(self._chain_uids)

    def chain_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {"seq": getattr(self, "_chain_seq", 0), "uids": list(self._chain_uids),
                    "htmask": getattr(self, "_chain_htmask", 0),
                    "a2mask": getattr(self, "_chain_a2mask", 0),
                    "a2asks": list(getattr(self, "_chain_a2asks", []))}

    def mcaddr_ack(self, unit: int) -> Optional[int]:
        with self._lock:
            return self._mcaddr_ack.get(int(unit))

    def chain_mcaddr(self) -> Optional[List[int]]:
        with self._lock:
            return getattr(self, "_chain_mcaddr", None)

    def chain_dialect(self) -> tuple:
        with self._lock:
            return (getattr(self, "_chain_a2mask", 0), list(getattr(self, "_chain_a2asks", [])))

    def chain_diag(self) -> tuple:
        with self._lock:
            fw = str((getattr(self, "_info", None) or {}).get("fw") or "")
            return (getattr(self, "_chain_htmask", 0), fw or getattr(self, "_chain_fw", ""),
                    getattr(self, "_chain_sel", (-1, 0, 0)))

    QUIET_WARN_S = 45.0
    QUIET_REPEAT_S = 300.0
    QUIET_DROP_S = 30.0
    INFO_AFTER_APPLY_S = 5.0
    RESET_QUIET_S = 2.0

    def _check_quiet(self) -> None:
        now = time.monotonic()
        if self._serial is None:
            since, what = self._down_t, "unreachable"
        else:
            since, what = self._last_frame_t, "silent"
        if since is None:
            return
        quiet_for = now - since
        if quiet_for < self.QUIET_WARN_S:
            return
        last = self._silence_logged_t
        if (last is not None
            and (now - last) < self.QUIET_REPEAT_S):
            return
        self._silence_logged_t = now
        self.logger.warning(f"BridgeBox: bridge has been {what} for {quiet_for:.0f}s"
                            f"{'; commands are being dropped' if self._serial is None else ''}")

    def _note_uartdiag(self, obj: dict) -> None:
        self.logger.debug(f"BridgeBox: bridge uartdiag {obj}", only_debug=True)
        why = str(obj.get("why") or "")
        try:
            now = (why, int(obj.get("edges")), int(obj.get("frames")))
        except (TypeError, ValueError):
            return
        prev, self._diag_prev = self._diag_prev, now
        label = self.wifi.label if self.wifi is not None else "BridgeBox"
        if why == "frames":
            if self._bus_said is not None:
                self._bus_said = None
                self.logger.info(f"{label}: the AMS bus is talking again")
            return
        if (why != "still"
            or prev is None
            or prev[0] not in ("silent", "still")
            or now[1] < 0
            or now[2] != prev[2]):
            return
        state = "no_signal" if now[1] == prev[1] else "deaf"
        if state == self._bus_said:
            return
        self._bus_said = state
        if state == "no_signal":
            self.logger.warning(f"{label}: no signal on the AMS bus. Check the bus cable to "
                                f"the BridgeBox and that the AMS has power.")
        else:
            self.logger.warning(f"{label}: the BridgeBox hears the AMS bus but no message "
                                f"gets through. Press RST on the BridgeBox; it reconnects by "
                                f"itself.")

    def _mark_connected(self) -> None:
        self._connected_t = time.monotonic()
        self._grace_this_conn = self._spoke_since_connect
        self._spoke_since_connect = False
        self._info_asked = False
        if (self.store_key
            and not getattr(self._serial, "challenged", False)):
            self.send({"cmd": "linkkey", "key": self.store_key})
        if self.wifi is not None:
            self.wifi.on_connect(self._serial)

    def _drop_if_silent(self) -> bool:
        if (self._serial is None
            or self._fw_raw):
            return False
        since = self._last_frame_t
        if self._connected_t is not None:
            since = self._connected_t if since is None else max(since, self._connected_t)
        if since is None:
            return False
        quiet_for = time.monotonic() - since
        if (self._expect_reset
            and isinstance(self._serial, TcpPort)
            and quiet_for >= self.RESET_QUIET_S):
            self._expect_reset = False
            self.logger.info("BridgeBox: bridge resetting as asked; reconnecting")
            self._drop_port()
            return True
        if quiet_for < self.QUIET_DROP_S:
            return False
        self.logger.warning(f"BridgeBox: bridge silent for {self.QUIET_DROP_S:.0f}s on an open "
                            f"link -- dropping it to force a reconnect")
        self._drop_port()
        return True

    def is_connected(self) -> bool:
        return self._serial is not None

    def link(self) -> Optional[str]:
        """
        How the host reaches the bridge now, for a panel to show.

        :return Optional[str]: "wifi" over its TCP link, "usb" over its serial port, None while
        no port is open
        """
        port = self._serial
        if port is None:
            return None
        return "wifi" if isinstance(port, TcpPort) else "usb"

    def silent_for(self) -> Optional[float]:
        if self._last_frame_t is None:
            return None
        now = time.monotonic()
        quiet = now - self._last_frame_t
        if (self._grace_this_conn
            and self._connected_t is not None
            and (now - self._connected_t) < CONNECT_GRACE_S):
            return min(quiet, now - self._connected_t)
        return quiet

    def _note_frame_gap(self, now: float) -> None:
        total = self._write_timeouts
        n = total - self._gap_timeouts_seen
        self._gap_timeouts_seen = total
        prev = self._last_frame_t
        if (prev is None
            or (self._connected_t is not None and prev < self._connected_t)):
            return
        gap = now - prev
        if gap <= SILENCE_LOG_S:
            return
        self.logger.debug(f"BridgeBox: bridge was silent {gap:.1f} s; {n} write(s) timed "
                          f"out meanwhile", only_debug=True)

    def down_since(self) -> Optional[float]:
        return None if self._serial is not None else self._down_t

    def _drop_port(self) -> None:
        s = self._serial
        self._serial = None
        self._connected_t = None
        if (s is not None
            or self._down_t is None):
            self._down_t = time.monotonic()
            self._down_epoch += 1
        while True:
            try:
                self._wq.get_nowait()
            except queue.Empty:
                break
        if s is not None:
            try:
                s.close()
            except Exception:
                pass

    def _reader(self) -> None:
        try:
            thread_name = threading.current_thread().name
            chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
        except Exception:
            pass
        buf = b""
        backoff = 0.5
        while self._run:
            if self._serial is None:
                if self.held():
                    time.sleep(0.25)
                    continue
                try:
                    port = self._serial_factory()
                    # The hold lock keeps a hold from landing between the check
                    # and the reconnect being announced for a port it closed.
                    with self._hold_lock:
                        self._serial = port
                        # A hold that came while the port was opening.
                        if self.held():
                            self._drop_port()
                            continue
                        down_for = (time.monotonic() - self._down_t
                                    if self._down_t is not None else 0.0)
                        self._down_t = None
                        self._silence_logged_t = None
                        self._mark_connected()
                        if down_for >= self.RECONNECT_SAY_S:
                            self.logger.info(f"BridgeBox: the BridgeBox link is back "
                                             f"after {down_for:.0f}s")
                        else:
                            self.logger.debug(
                                f"BridgeBox: bridge reconnected"
                                f"{f' after {down_for:.0f}s down' if down_for >= 1 else ''}")
                        for cb in self._reconnect_cbs:
                            try:
                                self.reactor.register_async_callback(lambda et, c=cb: c())
                            except Exception:
                                pass
                    buf = b""
                    backoff = 0.5
                except Exception:
                    self._check_quiet()
                    time.sleep(min(backoff, 5.0))
                    backoff = min(backoff * 2, 5.0)
                continue
            try:
                chunk = self._serial.read(64)
            except Exception as e:
                if self.held():
                    self._drop_port()
                    continue
                if self._expect_reset:
                    self._expect_reset = False
                    self.logger.info("BridgeBox: bridge resetting as asked; reconnecting")
                else:
                    self.logger.warning(f"BridgeBox: bridge read failed: {e}; reconnecting")
                self._drop_port()
                continue
            if not chunk:
                self._check_quiet()
                self._drop_if_silent()
                continue
            _now = time.monotonic()
            self._note_frame_gap(_now)
            self._last_frame_t = _now
            self._spoke_since_connect = True
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.handle_line(raw.decode(errors="replace"))
