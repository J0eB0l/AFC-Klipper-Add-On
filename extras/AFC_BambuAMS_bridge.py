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
#
# Bambu AMS bridge transport: the link to the Pi Pico that masters the AMS's
# RS-485 bus (threaded reader with reconnect, newline-JSON protocol, narration
# parsing). Import-safe and drivable with a fake serial port; AFC_BambuAMS.py
# imports BambuBridge and TcpPort from here.
from __future__ import annotations
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
from typing import Any, Callable, Dict, List, Optional, Tuple

# How long an in-flight measurement may stay declared before it is treated as
# abandoned. A real one takes 8-20s; this matches the follower backstop so the
# two cannot disagree about whether a calibrate is still running.
_MEAS_LIVE_MAX_S = 120.0

#: Writer pacing: queue depth at which the writer starts spacing writes, and
#: the gap between them. A reconnect re-announce (~115 commands on a
#: twelve-unit chain) would otherwise make a WiFi bridge drop the link; normal
#: traffic never reaches this depth.
WRITE_PACE_DEPTH = 8
WRITE_PACE_S = 0.02

#: How long a new link may stay quiet before that counts against the link-loss
#: pause. Covers only the gap to the first frame; the real clock keeps running
#: underneath.
CONNECT_GRACE_S = 2.0

#: A gap between frames longer than this is logged once, when the next frame
#: arrives (see _note_frame_gap). Typical causes: an AMS 1 capscan blocking the
#: bridge for 10-15 s, or a WiFi gap on a TCP bridge.
SILENCE_LOG_S = 2.5



# ── Pure helpers (unit-tested; no Klipper/hardware needed) ──────────────────────

def _queued_cmd_name(data: Any) -> str:
    """
    Name the command inside one queued write, for a log line.

    The writer queue holds encoded JSON lines, so a timed-out write can only
    be named by decoding it again. This runs on the writer thread, inside its
    error path, so it must never raise: anything that is not a JSON object
    with a "cmd" answers "?".

    :param data: one item from the writer queue (normally bytes)
    :return str: the command name, or "?" when there is none to give
    """
    try:
        if isinstance(data, (bytes, bytearray, memoryview)):
            data = bytes(data).decode("utf-8", "replace")
        obj = json.loads(data)
        cmd = obj.get("cmd") if isinstance(obj, dict) else None
        return str(cmd) if cmd not in (None, "") else "?"
    except Exception:
        return "?"


def parse_bridge_line(line: str) -> Optional[dict]:
    """
    Parse one newline-JSON line from the bridge into an event dict.

    :param line: A single line of text from the Pico (without the newline)
    :return Optional[dict]: the decoded object, or None if blank/invalid
    """
    line = line.strip()
    if not line:
        return None
    try:
        obj = json.loads(line)
    except (ValueError, TypeError):
        return None
    return obj if isinstance(obj, dict) else None

# "bldc_i:0.319A": the AMS's motor current, from feed/switch narration.
_BLDC_I_RE = re.compile(r"bldc_i:([0-9.]+)A")


# ── Plain-English narration ──────────────────────────────────────────────────
# Narration table (_AMS_HUMAN): only matched entries reach the console;
# everything goes to AFC.log verbatim. An entry is (pattern, render) or
# (pattern, render, log_only), where log_only keeps the sentence off the
# console.
#
# Drying telemetry, every ~10 s while a cycle runs (both separators occur):
#   [AMS_CHMB]s:2|rf:55,0|vt:44.0|ap:35.3|hts:34,31,00|pw:100|ad:2|...
#   [AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:23.0, hts:46,23,0 pw:100, ...
# Groups: state, target, chamber, humidity (the first ht:/hts: value).
#: The `ad:` field, the unit's external power adapter:
#:   AMS 2 Pro   ... pw:32,73,66,30,35,29,34, ad:4,23.9 wd:0,0,0,0, fa:104,99
#:   AMS HT      ... pw:100, ad:2, wd:0,0,0,0, fa:99
#: On an AMS 2 the second number is the 24 V jack voltage (~23.8 with the
#: adapter, ~0.5 without). _bus_supply_conflict uses it: two AMS 2s drying off
#: bus power collapse the supply and reset every unit on the wire.
_CHMB_AD_RE = re.compile(r"\[AMS_CHMB\].*?\bad:(\d+)(?:\s*,\s*([0-9.]+))?")

_CHMB_STATE_RE = re.compile(
    r"\[AMS_CHMB\]s:(\d+)(?:,\d+)?\s*[|,]\s*"
    r"rf:(\d+)(?:,\d+)?\s*[|,]\s*"
    r"(?:cd:\d+\s*[|,]\s*)?"
    r"vt:([0-9.]+)(?:\s*,\s*\d+)?"          # the ,00 suffix is not humidity
    r"(?:.*?\bhts?:(\d+)\s*,)?")            # ht:<RH>,<sensor C>  (or hts:)
# tube_len is the AMS's self-calibrated path length, in mm or metres; 0 until
# it has enough samples, which the caller drops.
#: A motion completion, and only that. Each generation has its own wording:
#:   [AMS_SWITCH]feed finish, buff_pos:1.29, bldc_i:1.593A       (HT)
#:   [AMS_SWITCH]feed finish 0, dw_len:3.508 m                   (AMS 2 Pro)
#:   [AMS_SWITCH]pull finish 0, tray_sw:0, len_det:0.265 m
#:   [AMS_PRELOAD]preload finish
#:   [AMS_SWITCH]pull sucess,cond match,...                      (AMS 2 Pro)
#:   [AMS_SWITCH]feed to dw ok / feed to normal, len_det:...     (HT)
#: "pull" must precede "sucess" so the per-feed "state_switch finish,
#: sucessful" does not match.
_MOTION_FINISH_RE = re.compile(
    r"\b(?:(?:feed|pull|preload)\s+finish|pull\s+suc+ess"
    r"|feed\s+to\s+(?:normal|dw\s+ok))\b",
    re.IGNORECASE)

#: The end of a retract, which _MOTION_FINISH_RE does not see:
#:   [AMS_SWITCH]SRL_state_switch finish, sucessful, err_code:0x00
#:   [AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, err_code:0x00
#: The prefix varies; err_code:0x00 is required, so a unit that never says it
#: falls back to the deadline.
_STATE_SWITCH_DONE_RE = re.compile(
    r"state_switch\s+finish\s*,\s*\w+\s*,\s*err_code:0x00\b",
    re.IGNORECASE)

#: The value ``tray_now`` carries when no tray is the current one.
TRAY_NONE = 255

#: The unit's current tray, from [AMS_COMMON]state:N,tray_now:M,tray_exit:K.
#: The tray_now -> 255 edge is the AMS 2's most reliable unload completion; it
#: also fires on preload/insert, so a waiter requires the retract window and
#: the commanded tray.
_TRAY_NOW_RE = re.compile(r"tray_now:(\d+)")

#: The AMS HT's capacity calibration verdict (the HT does not narrate "odom
#: C:..,P:NN%"):
#:   [AMS_RFID] STEP4,Calibration rst:0   completed; 1 refused; 4 aborted
#: The percent is read back from the tag afterwards.
_HT_CALI_RST_RE = re.compile(r"Calibration\s+rst:(\d+)", re.IGNORECASE)

#: "[AMS_SWITCH]e_in tray:N,..." has no pattern on purpose: an err_code
#: transition follows it, so it means "error in tray". _BUFF_POS_RE still reads
#: its buff_pos.

#: Buffer position as an instantaneous reading, from e_in during a load or feed
#: finish at the end of one.
_BUFF_POS_RE = re.compile(r"buff_pos:(-?[0-9]+\.[0-9]+)")

#: A buffer refill, the ramming event itself:
#:   [AMS_SWITCH]BUFF,pos:0.09->0.74, det:6mm,  i:0.583A
#: pos before -> after, det the filament fed. "BUFF,pos:" differs from
#: "buff_pos:", so it needs its own pattern.
_BUFF_REFILL_RE = re.compile(
    r"BUFF,\s*pos:(-?[0-9.]+)\s*->\s*(-?[0-9.]+)"
    r"(?:[^\n]*?det:(\d+)\s*mm)?",
    re.IGNORECASE)

#: A clean completion, e.g. the HT's "feed finish, buff_pos:1.28". Requires a
#: comma or end-of-line after "finish" so "finish -1" cannot match.
_CLEAN_FINISH_RE = re.compile(r"\b(?:feed|pull|preload)\s+finish\s*(?:,|$)",
                              re.IGNORECASE)

#: How far short of its measured path (mm) a stall still counts as arrived: a
#: normal load stalls tens of mm short, a short move hundreds.
FINISH_ARRIVAL_TOLERANCE_MM = 100.0

#: Completion in the [AMS_DEV] dialect, which never says "finish": "STEP:odom
#: reset tray 0" when a feed arrives. The reset line is cross-dialect.
_ODOM_RESET_RE = re.compile(r"odom\s+reset\s+tray\s*\d+", re.IGNORECASE)
#: The unit's current error level, in either form:
#:   [AMS_LINK]err_code: 0 -> 23        decimal, spaced (HT)
#:   [AMS_LINK]err_code:0x00->0x80      hex, unspaced (AMS 2)
#: The value after the arrow is the new level; 0 means no error. Read as a
#: level, not an edge (see handle_line).
_ERR_CODE_RE = re.compile(
    r"err_code:\s*(0x[0-9A-Fa-f]+|\d+)\s*->\s*(0x[0-9A-Fa-f]+|\d+)")

_ODOM_NO_TRAY_RE = re.compile(r"odom\s+tray_id\s+error\s*255", re.IGNORECASE)

#: Not a completion marker: tray_now:255 also appears while loaded and
#: following (state:4), so it would end a move early. Left unmatched.

#: Distance the AMS says it actually moved, in metres.
_LEN_DET_M_RE = re.compile(r"len_det:([0-9]+\.[0-9]+)\s*m\b")

#: The second half of the unit's seating cycle ("pull sucess" is the first):
#:   [AMS_SWITCH]assist finish 0, ref:0
#: The extruder must wait for this too, or it fights the unit's push.
_ASSIST_DONE_RE = re.compile(r"assist finish\s*-?\d*")

_TUBE_LEN_MM_RE = re.compile(r"tube_len:(\d+)\s*mm")
_TUBE_LEN_M_RE = re.compile(r"tube_len:([0-9]+\.[0-9]+)\s*m\b")
# dw_len, reported at the end of a feed ("feed finish 0, dw_len:3.508 m"), is a
# per-load measurement (0.000 on a failure). Reported only.
_DW_LEN_M_RE = re.compile(r"dw_len:([0-9]+\.[0-9]+)\s*m\b")


#: A drying command the AMS refused, in its own words:
#:   [AMS_LINK]ret:1,mode:1,temp:55,time:480      the command, echoed
#:   [AMS_CHMB]err, filament hub load!            and refused
#: The echo proves delivery, so the reason is reported to the panel.
_DRY_REFUSED_RE = re.compile(r"\[AMS_CHMB\]\s*err,\s*([^\[\r\n]{1,60})")

#: The unit's echo of its drying settings, e.g. "[AMS_CHMB]rotate:0,0,
#: pw_lim:100, cool_down:0,45, dur:480, tmpr:45" (minutes, C). The only source
#: for a cycle this host did not start.

#: The spool capacity measurement at the end of the insert calibration:
#:   HT     [AMS_RFID] STEP4,odom C:0.531,R:0.084,P:107%,od:1.132
#:   AMS 1  [AMS_DEV]  STEP:odom C:0.480, R:0.076, P:78%, od:0.988
#:   AMS 2  [AMS_RFID]STEP:odom load from flash 2,R:0.072,P:65
#: Requiring R: close after `odom` excludes the load-time radius search lines
#: ("STEP:odom r:0, dt0.442, R:0.073, P:70%"), which do not converge.
_CAP_MEASURE_RE = re.compile(
    r"odom\s+"
    r"(?:load\s+from\s+flash\s+(\d+)\s*,\s*"     # 1 tray (restore form)
    r"|C:([0-9.]+)\s*,\s*)?"                       # 2 circumference (live)
    r"R:([0-9.]+)\s*,\s*"                          # 3 radius, metres
    r"P:(\d+)\s*%?",                               # 4 remaining percent
    re.IGNORECASE)

#: The same reading without colons, tried after _CAP_MEASURE_RE:
#:     [AMS_RFID] STEP,odom r0, dt0.444, R0.075, 82%, od0.751
#: Separate so a bare "NN%" near any odom line is not a measurement. It names
#: its tray, so _note_cap_measure records it as a restore, not a live measure.
_CAP_MEASURE_ALT_RE = re.compile(
    r"odom\s+r(\d+)\s*,\s*"                        # 1 tray
    r"dt[0-9.]+\s*,\s*"                            #   distance travelled
    r"R([0-9.]+)\s*,\s*"                           # 2 radius, metres, no colon
    r"(\d+)\s*%",                                  # 3 remaining percent, bare
    re.IGNORECASE)

#: The unit naming the bay it just measured:
#:   [AMS_RFID]STEP:odom C:0.588,R:0.094,P:142%,N:2,od:0.609
#:   [AMS_RFID]STEP:odom save tray:1, R:0.093643
#: The save line carries the same radius at full precision, tying it to the
#: reading. It resolves the bay within a unit only (boxed units share an
#: address), so it is used as a veto, never a new attribution.
_CAP_SAVE_RE = re.compile(
    r"odom\s+save\s+tray:(\d+)\s*,\s*R:([0-9.]+)", re.IGNORECASE)

#: The calibration verdict. The HT misspells it, hence succ?ess:
#:   HT     [AMS_RFID] STEP4,odom calib sucess
#:   AMS 1  [AMS_DEV]  STEP:odom calib success exit 0,dis:0.989
_CALI_DONE_RE = re.compile(
    r"odom\s+calib\s+succ?ess(?:\s+exit\s+(\d+))?", re.IGNORECASE)

_DRY_CFG_RE = re.compile(
    r"\[AMS_CHMB\]\s*rotate:(\d+),\s*(\d+),\s*pw_lim:(\d+),"
    r"\s*cool_down:\d+,\s*(\d+),\s*dur:(\d+),\s*tmpr:(\d+)")

#: The STEP marker's punctuation in all dialects: "STEP:" on boxed units,
#: "STEP3," on the HT. A literal "STEP:" never matches the HT.
_STEP_SEP = r"STEP\d*\s*[,:]\s*"


def _STEP(tail: str) -> "re.Pattern":
    """
    Build a pattern for a STEP event that ignores which dialect said it.

    The three units tag and punctuate the same event differently:

        HT     [AMS_RFID] STEP4,odom calib sucess
        AMS 2  [AMS_RFID]STEP:read success
        AMS 1  [AMS_DEV]  STEP:odom calib success exit 0

    The pattern therefore accepts any [AMS_*] tag, an optional space, STEP with
    an optional step number, ',' or ':' as the separator, then the shared
    wording. Anchoring on one dialect matches nothing on the other two.

    :param tail: the event wording, as a regex fragment
    :return Pattern: a compiled, dialect-tolerant pattern
    """
    return re.compile(r"\[AMS_[A-Z_]+\]\s*" + _STEP_SEP + tail, re.IGNORECASE)


_AMS_HUMAN = (
    # Before the generic err, rule: "shell open" is the HT's lid, not a
    # refusal. The dry keeps running; the chamber just cannot hold
    # temperature until the lid closes.
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
     lambda m: f"AMS drying: self-check started, target {m.group(1)}C"),
    (re.compile(r"\[AMS_CHMB\]set state CTC_STATE_HEATING"),
     lambda m: "AMS drying: self-check passed, now heating"),
    (re.compile(r"\[AMS_CHMB\]set state CTC_STATE_OFF"),
     lambda m: "AMS drying: heater off"),
    (re.compile(r"\[AMS_SWITCH\]new tube_len:(\d+) mm.*?err:(-?\d+) mm"),
     lambda m: (f"AMS measured the PTFE path at {m.group(1)}mm "
                f"(+/-{m.group(2)}mm)")),
    # Prefix-agnostic from here: the generations share content, not bracket
    # tags. "read success" and "feed with rfid success" are left out because a
    # failed HT attempt emits both before retrying.
    (_STEP("card auth success"),
     lambda m: "AMS: tag authenticated"),
    (_STEP("auth card successful"),
     lambda m: "AMS: tag authenticated"),
    (_STEP("first detected"),
     lambda m: "AMS: spool detected"),
    (_STEP(r"select card fail, err (\d+)"),
     lambda m: f"AMS could not read the spool tag (err {m.group(1)})"),
    (_STEP(r"odom calib succ?ess(?:\s+exit\s+(\d+))?"),
     lambda m: "AMS finished measuring the spool"),
    (re.compile(r"\[RF\]\s*tray(\d+): info write to flash"),
     lambda m: (f"AMS: tag for bay {int(m.group(1)) + 1} cached in the unit's "
                f"flash (a later read returns it even after a swap)")),
    (re.compile(r"preload\s+finish", re.IGNORECASE),
     lambda m: "AMS staged the spool at its feeder"),
    # Plain-English renderings of the events an operator acts on. First, the
    # measurement result: of "odom C:..,R:..,P:79%" only the percent matters.
    (re.compile(r"odom\s+C:[0-9.]+\s*,\s*R:([0-9.]+)\s*,\s*P:(\d+)%"),
     lambda m: (f"AMS measured the spool: about {m.group(2)}% left "
                f"(spool radius {float(m.group(1)) * 1000:.0f} mm)")),
    # The stored per-tray value, read back at power-up from the unit's flash;
    # the only place the unit states it.
    (re.compile(r"odom\s+load\s+from\s+flash\s*(\d+)\s*,\s*"
                r"R:[0-9.]+\s*,\s*P:(\d+)"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} remembers about "
                f"{m.group(2)}% left from its last measurement")),
    # The unit committing a fresh measure to flash.
    (re.compile(r"odom\s+save\s+tray:(\d+)"),
     lambda m: f"AMS stored a new measurement for bay {int(m.group(1)) + 1}"),
    # A stalled feed: len_det is how far it got, tube_len how far it should
    # have gone.
    (re.compile(r"feed\s+finish\s+-?\d+\s*,\s*stall\s*,\s*"
                r"len_det:([0-9.]+)\s*m\s*,\s*tube_len:([0-9.]+)\s*m"),
     lambda m: (f"AMS: the filament STALLED after {float(m.group(1)):.2f} m "
                f"of a {float(m.group(2)):.2f} m path -- check for a jam "
                f"between the bay and the toolhead")),
    # The error register changing: 0x00 -> anything raises, -> 0x00 clears.
    # 0x16 is the assist slipping on a tray it cannot grip, routine while the
    # follower holds an unloaded tray, so it stays off the console.
    (re.compile(r"err_code:0x00\s*->\s*0x16\b"), None),
    (re.compile(r"err_code:0x16\s*->\s*0x0+\b"), None),
    # 0x80 is a status bit raised at motor start, not a fault; log only.
    (re.compile(r"err_code:0x00\s*->\s*0x80\b"), None),
    (re.compile(r"err_code:0x80\s*->\s*0x0+\b"), None),
    (re.compile(r"err_code:0x00\s*->\s*0x([0-9A-Fa-f]+)"),
     lambda m: f"AMS raised error 0x{m.group(1).upper()}"),
    (re.compile(r"err_code:0x[0-9A-Fa-f]+\s*->\s*0x0+\b"),
     lambda m: "AMS cleared its error"),
    # A spool leaving the bay, and the calibration that goes with it.
    (re.compile(r"tray\s+(\d+)\s+out\s*,\s*clear\s+magic_num"),
     lambda m: f"AMS: bay {int(m.group(1)) + 1} is now empty"),
    # Power-up. The self-check is the unit's boot, which is worth one line
    # because it means everything it knew about follower state is gone.
    (re.compile(r"ams\s+pmsm\s+cali\s+finish", re.IGNORECASE),
     lambda m: "AMS finished its power-up self-check"),
    (re.compile(r"pmsm\s+self\s+check\s+good", re.IGNORECASE),
     lambda m: "AMS motor self-check passed"),
    # The second odometer edge, one full turn of the spool; its absence is the
    # fast-path.
    (_STEP("second detected"),
     lambda m: "AMS: spool turned a full revolution (measuring)"),
    (_STEP(r"odom calib\s*,\s*tray (\d+)"),
     lambda m: f"AMS started measuring bay {int(m.group(1)) + 1}"),
    (_STEP(r"cali end"),
     lambda m: "AMS finished its measuring cycle"),
    (_STEP(r"Calibration rst:(\d+)"),
     lambda m: f"AMS finished measuring (result {m.group(1)})"),
    # A tag the unit cannot open, almost always a non-Bambu spool.
    (_STEP(r"auth fail:-?(\d+)"),
     lambda m: ("AMS could not authenticate the tag -- third-party spool, "
                "or the tag is unreadable")),
    (re.compile(r"\[RF\]\s*tray(\d+): info same as last read"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} holds the same spool as "
                f"before")),
    # No stored calibration for this bay, so no fast-path.
    (_STEP(r"odom invalid tray (\d+)"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} has no stored measurement "
                f"yet")),
    (_STEP(r"odom load tray (\d+) info invailed"),
     lambda m: f"AMS: bay {int(m.group(1)) + 1} has no stored measurement yet"),
    # Log only: the unit drops its tray selection after every bay, so a clean
    # unload ends with it refusing the next command. The load path reports the
    # case that matters (see _AMS_NO_TRAY_RE).
    (_STEP(r"odom tray_id error (\d+)"),
     lambda m: ("AMS: asked to move with NO TRAY SELECTED -- the unit "
                "rejected the command"),
     True),
    (re.compile(r"\[AMS_LED\]\s*TIMEOUT error (\d+)"),
     lambda m: "AMS: TIMEOUT -- the unit gave up on the move"),
    # Feed milestones, with the distance that makes them meaningful.
    (re.compile(r"feed to dw ok\s*,\s*len_det:([0-9.]+)\s*m"),
     lambda m: f"AMS: filament reached the hub after {float(m.group(1)):.2f} m"),
    (re.compile(r"feed finish\s*,\s*buff_pos:[0-9.]+\s*,\s*bldc_i:[0-9.]+A"
                r"\s*,\s*t:([0-9.]+)s"),
     lambda m: f"AMS finished feeding ({float(m.group(1)):.1f} s)"),
    (re.compile(r"new tube_len:(\d+)\s*mm"),
     lambda m: f"AMS learned the bay-to-hub path length: {m.group(1)} mm"),
    # Staging the spool at the feeder, and the bay lock that goes with it.
    (re.compile(r"preload start", re.IGNORECASE),
     lambda m: "AMS is staging the spool at its feeder"),
    # Power-up of the whole unit: its follower state is gone.
    (re.compile(r"\[AMS_ADA\]\s*init", re.IGNORECASE),
     lambda m: "AMS powered up (any follower state it held is gone)"),
)

# The terminal success markers of a tag read. Every alternative goes through
# _STEP_SEP, never a literal "STEP:", or the HT never matches.
_RFID_READ_OK_RE = re.compile(
    _STEP_SEP + r"read success"
    r"|" + _STEP_SEP + r"read_done=1"
    r"|" + _STEP_SEP + r"feed with rfid success"
    # The HT's commit sentence: it has authenticated the chip and written the
    # record to its own flash. Stronger than "read success": the unit is
    # stating that the tag it now serves belongs to the spool in the bay.
    r"|" + _STEP_SEP + r"save to flash"
    r"|card info valid"
    # The [RF]-prefixed commit lines: "trayN: info write to flash" on a fresh
    # read, "trayN: info same as last read" on a re-insert of the same spool.
    r"|info write to flash"
    r"|info same as last read")

# The HT's terminal-only subset: its "read success" lines also fire on failed
# sub-cycles, so only the flash-commit family counts there.
_RFID_READ_OK_HT_RE = re.compile(
    _STEP_SEP + r"save to flash"
    r"|card info valid"
    r"|info write to flash"
    r"|info same as last read")

#: A foreign tag: a non-Bambu Mifare chip answers anticollision, then "auth
#: fail -4" / "info_valid 0 or bbl:-1". Tells a non-Bambu spool from an empty
#: bay; the colon after "fail" is optional.
_RFID_FOREIGN_TAG_RE = re.compile(
    # Only the auth refusal. "info_valid 0 or bbl:N" reads like foreign-tag
    # wording but also appears on empty-bay cycles and mid-retry on HT reads
    # that then succeed, so it would word an empty bay as a refused chip.
    r"auth fail\s*:?\s*-?\d+",
    re.IGNORECASE)

# End of the scan cycle, whatever the outcome; the moment to say a tag did not
# read (a clock cannot, since a bay can go quiet for 11 s). Per dialect:
#   HT     [AMS_RFID] STEP4,Calibration rst:0
#   AMS 1  [AMS_DEV]  STEP7:finish,cali tray
#   AMS 2  [AMS_RFID] STEP7:cali end
# Bare "STEP7:" is mid-cycle. Not "odom calib success": that ends the
# measurement (_MEAS_END_RE), and an AMS 1 says it on the insert edge, about
# 30 s before its tag read.
_RFID_CYCLE_END_RE = re.compile(
    r"Calibration\s+rst:\d+"
    r"|STEP7:\s*(?:finish|cali\s+end)"
    # An HT with capacity measure disabled ends on "tray capacity no en" and
    # never says "Calibration rst:".
    r"|tray\s+capacity\s+no\s+en"
    # An HT that misses its second RFID pass ends on "odom pull back" with no
    # "Calibration rst:". Anchored on odom: "STEP4,pull back to SW" comes
    # earlier.
    r"|odom\s+pull\s+back",
    # Not "tray pull over N mm, no card detected": that line is in-flight;
    # the unit is still finishing after it and the true terminal
    # "STEP7:finish,cali tray" arrives next.
    re.IGNORECASE)

# End of the unit's odometer measurement ("odom calib sucess" on an HT). It
# closes the "first detected" window, and every cycle end closes it too.
_MEAS_END_RE = re.compile(r"odom\s+calib\s+succ?ess", re.IGNORECASE)

# The unit's own retries are spent: "AMS_CTRL_state_switch finish, fail,
# retry:5, ...". Unlike a stall, which the unit recovers from. An AMS 1 emits
# no AMS_CTRL line and keeps its deadline.
_AMS_GAVE_UP_RE = re.compile(
    r"state_switch\s+finish\s*,\s*fail\b", re.IGNORECASE)

# "Not pointed at a bay": after a fault latches the unit rejects every move
# ("odom tray_id error N"), so a retry is pointless. Stamped like the give-up
# above; see no_tray_since().
_AMS_NO_TRAY_RE = _STEP(r"odom tray_id error (\d+)")

# A bay's two switches changing: "[AMS_TRAY]tray[2] sw_sta update, 3 -> 2".
# Bit 0 is the inlet by the spool, bit 1 the outlet toward the hub
# ("u_in_out:" gives their voltages). Recorded per unit and bay; see
# tray_switches().
_AMS_TRAY_SW_RE = re.compile(
    r"\[AMS_TRAY\]\s*tray\[(\d+)\]\s*sw_sta\s*update\s*,\s*(\d+)\s*->\s*(\d+)")

# The true terminal ("STEP7:cali end" / "STEP7:finish") for callers that must
# not overlap the unit's cycle; "odom calib success" can arrive with the next
# move.
_RFID_TERMINAL_RE = re.compile(
    r"STEP7:\s*(?:finish|cali\s+end)"
    r"|Calibration\s+rst:\d+"
    r"|tray\s+capacity\s+no\s+en",
    re.IGNORECASE)

# Link-layer chatter, kept in AFC.log but off the console, and only when every
# bracketed segment in the line is noise.
_AMS_NOISE_RE = re.compile(
    r"^(?:\s*(?:"
    r"\[AMS_CALL\]\s*ams\d+\s+select,\s*select\s+ams\d+"
    r"|\[AMS_LINK\]\s*ams\d+\s+select,\s*req\s+ams\d+"
    r"|\[AMS_COMMON\]\s*(?:mode|ref):\s*-?\d+\s*->\s*-?\d+"
    # An assist slipping on an unloaded tray, twice a second; routine.
    r"|\[AMS_SWITCH\]\s*tray:\d+,\s*bldc slip,\s*dw_pos:\S+\s*m"
    r"|\[AMS_SWITCH\]\s*assist finish 0, ref:0"
    r"|\[AMS_PMSM\]\s*mode:\d+\s*->\s*\d+"
    # err_code toggling between 0x16 (assist slip against no tray) and 0x00
    # rides the same flood; a code that stays raised still reaches the
    # console through the fault path, so the toggle segments are chatter.
    r"|\[AMS_LINK\]\s*err_code:\s*0x[0-9A-Fa-f]+\s*->\s*0x[0-9A-Fa-f]+"
    r"|\[AMS_IDLE\]\s*set ams state switch"
    # Boxed-unit housekeeping every 90 s ("preload_disable:1, tmpr:..").
    # Console only: the log keeps it and every parser has already run.
    r"|\[AMS_COMMON\]\s*preload_disable:\d+\s*,\s*tmpr:[0-9.]+\s*,\s*cd:\d+"
    # The RFID poller idling (states 0 and 3, any tray). States 1, 6 and 7 stay
    # visible.
    r"|\[AMS_COMMON\]\s*state:[03]\s*,\s*tray_now:\d+\s*,\s*tray_exit:\d+"
    r"|\[AMS_COMMON\]\s*en:\d+\s*,\s*mode:\d+\s*,\s*idx:\d+\s*,\s*ref:\d+"
    r"|\[AMS_RFID\]\s*STEP0:\s*checking"
    r"|\[AMS_RFID\]\s*STEP0:\s*idx\s+\d+\s*>\s*\d+"
    # Print-time chatter: [AMS_PMSM] mode cycling, [AMS_LED] loading,
    # [AMS_SWITCH]BUFF (on the buffer card) and [AMS_COMMON]state:4. Console
    # only; states 1/6/7 stay visible.
    r"|\[AMS_PMSM\]\s*mode:\s*\d+\s*->\s*\d+"
    r"|\[AMS_LED\]\s*tray\s+\d+\s+\w+"
    r"|\[AMS_SWITCH\]\s*BUFF\s*,[^\[]*"
    r"|\[AMS_COMMON\]\s*state:4\s*,\s*tray_now:\d+\s*,\s*tray_exit:\d+"
    # Mechanism chatter: follower assist, RFID steps, select acks and move
    # bookkeeping. Decisions, outcomes and faults get sentences in _AMS_HUMAN.
    r"|\[AMS_IDLE\]\s*set ams state assist[^\[]*"
    r"|\[AMS_LINK\]\s*en:\d+\s*,\s*mode:\d+\s*,\s*idx:\d+\s*,\s*ref:\d+"
    # The select ack, named by bus address ("ams-0x00", dash and 0x optional).
    r"|\[AMS_(?:LINK|CALL)\]\s*ams-?(?:0x)?[0-9A-Fa-f]+\s+select[^\[]*"
    # [AMS_LINK]get_slot is not listed: the dedupe loop handles its repetition,
    # and muting it would hide a stuck unit (pinned by
    # TestTheHeartbeatCannotBreakTheDedupe). Only the feed-cycle sw_sta
    # transitions are noise; "0 -> 1" and "1 -> 0" (spool in/out) stay visible.
    r"|\[AMS_TRAY\]\s*tray\[?\d*\]?\s*sw_sta\s*update\s*,\s*"
    r"(?:3\s*->\s*\d+|\d+\s*->\s*3)[^\[]*"
    r"|\[AMS_BDC\]\s*tray lock:[^\[]*"
    r"|\[AMS_LED\]\s*mc set tray[^\[]*"
    r"|\[AMS_ENC\]\s*clc[^\[]*"
    r"|\[AMS_LINK\]\s*assist_err:[^\[]*"
    r"|\[AMS_SWITCH\]\s*(?:assist finish|reset dw length|retry:|AMS_CTRL_"
    r"|SWITCH_pull ignore|SWITHC_feed ignore|need to pull tray|feed tray:"
    r"|pull tray:|pull sucess)[^\[]*"
    # The RFID step machine's stages; the results are translated and stay.
    r"|\[AMS_(?:RFID|DEV)\]\s*STEP\d*[:,]?\s*(?:odom search|set \d+ tray_readid"
    r"|rfid pull|time_reset|stop goto auth|check pass|checking|open_PCD"
    r"|reader \d+ enable|pull tension|goS\d|done\d|search \d+ card"
    r"|anticoll get UID|direct read card|empty to read|select card success"
    r"|pull back|start,read all card|ready to cali tray|no card in RF"
    r"|after tension|confirm RF have no card|odom select rslt"
    r"|odom reset tray|cali read tray)[^\[]*"
    # Motor/encoder self-test internals at power-up. "self check good" and
    # "cali finish" are translated; the ADC dumps behind them are not.
    r"|\[AMS_PMSM(?:_[A-Z])?\]\s*(?:adc\d|timeout, retry|get ams_id"
    r"|has ams_id|P cali init|table_xy)[^\[]*"
    r"))+\s*$")

#: The AMS's 10 s heartbeat, removable mid-line since the unit bundles it into
#: any frame. The colon after "time" is optional.
_DBG_AMSTIME_RE = re.compile(r"\[DBG\]\s*ams time\b[^\[]*")


def _ams_is_noise(text: str) -> bool:
    """
    Whether a narration line is pure link-layer chatter (console-suppressed).

    :param text: One raw narration line from the AMS
    :return bool: True if nothing in the line is worth an operator's attention
    """
    # Leading junk: the drain reply often starts with one stray byte rendered as
    # a character ("\\ [AMS_CHMB]...", "q [AMS_DEV]..."), which is framing, not
    # content. Strip up to the first bracket before judging the line.
    i = text.find("[")
    return bool(text) and i >= 0 and bool(_AMS_NOISE_RE.match(text[i:]))

# Bridge firmware diagnostics on the narration channel, not AMS speech:
# [HT-MEAS] and [BB-GATE] (always stamped 0x1800) and [CAP]. Logged verbatim,
# then dropped: no parser may use them, and their changing counters would break
# the dedupe.
_FW_DIAG_RE = re.compile(r"^\s*\[(?:HT-MEAS|BB-GATE|CAP)\]")

#: Motion acks that ride the follower's own cadence rather than marking an
#: operator-visible decision. Console-suppressed (AFC.log keeps them all):
#: during a print these repeat every few seconds for the length of the job.
_ACK_ROUTINE = frozenset(("assist", "select", "stop", "hold", "follow"))

#: Sentinel for "no value yet", where None is itself meaningful.
_UNSET = object()

# Events the reader consumes or ignores on purpose; anything else is logged
# (see handle_line). Command echoes are listed as uninteresting.
_BRIDGE_EVENTS_KNOWN = frozenset((
    "status", "reply", "error", "ack", "amsdbg", "sniff", "chain", "info",
    "sniff_mode", "slow",
    # command echoes
    "dry", "mon", "resync", "mcaddr", "armms", "arrivems", "hb",
    # AMS firmware-update replay echoes (driven by the AMS updater, not this
    # module; listed so a connected module does not flood the log if the
    # updater runs while it is attached).
    "fwreplay", "txbuf", "txsend",
    "htunit", "mute", "units", "variant", "baud",
    # ("unload" is bridge_unload()'s echo; it is unit/slot-addressed.)
    "parity", "replay", "unload", "relink", "rehome",
    "capscan", "ht0fhold",
    "arrived", "txecho", "fw",
    # Scan-path echoes of host commands; unlisted, each would log a line and
    # could starve the MCU.
    "scan", "reid", "reread", "prime",
    # "tx" is the event, "txecho" only the echo. Both must be listed: the
    # unknown-event catch-all runs before the per-event branches.
    "tx", "loops",     # both also need a handler in handle_line: "known"
                       # without one means the catch-all stops logging it
                       # and it vanishes.
    # Enrollment echoes (bind, htuid) every status round; listed so they stay
    # silent.
    "bind", "htuid",
    # A BB_WIFI firmware's radio state, printed by the Pico on change only.
    # Has a handler in handle_line as well (see "tx"/"loops"): membership alone
    # would silence the one event that says why a board never appeared.
    "wifi",
    # The calibrate command's echo, also with a handler: when an AMS 1 runs
    # the calibrate it prints only once the firmware's blocking capacity burst
    # returns, so it marks the end of that stall.
    "cali",
))

# A write timeout, told apart from a broken link. Falls back to a class that
# never matches when pyserial is not loaded.
try:                                        # pragma: no cover - import shim
    from serial import SerialTimeoutException as _SerialTimeout
except Exception:                           # pragma: no cover
    class _SerialTimeout(Exception):
        pass


class TcpPort:
    """
    A pyserial-shaped TCP transport, for a bridge reached over the network.

    Opt-in. USB-CDC is the primary transport; this exists for a
    WiFi-capable Pico (or a socat/ser2net relay in front of a USB one) and is
    selected only by writing ``serial_port: tcp://host:port``. Nothing changes
    for a serial_port that is a device path.

    The bridge asks a port for exactly three things (``read(n)``,
    ``write(b)`` and ``close()``), so that is the whole surface here, with
    pyserial's semantics preserved because the reader and writer threads rely
    on them:

    - ``read`` returns b"" on timeout (a quiet link is not an error) and
      raises at end of stream, which is what makes the reader drop the port
      and reconnect with backoff. Returning b"" there instead would spin.
    - ``write`` raises ``_SerialTimeout`` when it cannot get the bytes out in
      time, so a busy far end lands in the writer's "bridge busy, write of
      ... timed out" path, as a Pico that stopped draining its CDC does,
      rather than killing the link.

    The socket timeout is never changed after connect. The reader thread sits
    in recv while the writer thread sends, so a settimeout() around a write
    would change the timeout out from under a blocked read. The write deadline is enforced by
    looping on a short fixed timeout instead. recv and send from two threads
    on one socket is otherwise fine: the directions are independent.
    """

    #: Wire default when a tcp:// spec omits the port.
    DEFAULT_PORT = 8888

    @staticmethod
    def parse(spec: str) -> Tuple[str, int]:
        """
        Split ``tcp://host:port`` into its parts.

        Accepts a bracketed IPv6 literal (``tcp://[fe80::1]:8888``) and a
        missing port (defaults to ``DEFAULT_PORT``).

        :param spec: the serial_port value, with or without the tcp:// scheme
        :return tuple: (host, port)
        :raises ValueError: if no host can be read out of it
        """
        raw = str(spec or "").strip()
        if "://" in raw:
            raw = raw.split("://", 1)[1]
        raw = raw.rstrip("/")
        if raw.startswith("["):                       # [v6]:port
            host, _, tail = raw[1:].partition("]")
            port = tail.lstrip(":")
        elif raw.count(":") > 1:                      # bare v6, no port
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

    def __init__(self, host: str, port: int, timeout: float = 0.1,
                 write_timeout: float = 0.5,
                 connect_timeout: float = 3.0,
                 key: Optional[str] = None) -> None:
        """
        Open a TCP link to the bridge.

        :param host: bridge hostname or address
        :param port: bridge TCP port
        :param timeout: read timeout, seconds (b"" is returned on expiry)
        :param write_timeout: how long write() may take before it gives up
        :param connect_timeout: how long the initial connect may take
        :param key: the link key, or None/"" for an unauthenticated link
        """
        self._sock = socket.create_connection((host, int(port)),
                                              timeout=connect_timeout)
        self._timeout = float(timeout)
        self._pushback = b""
        self._sock.settimeout(timeout)
        for level, opt in ((socket.IPPROTO_TCP, socket.TCP_NODELAY),
                           (socket.SOL_SOCKET, socket.SO_KEEPALIVE)):
            # NODELAY for latency; KEEPALIVE so a bridge that vanishes without
            # a FIN eventually errors instead of hanging.
            try:
                self._sock.setsockopt(level, opt, 1)
            except OSError:
                pass
        # Probe after 20 s idle, every 5 s, give up after 3 (~35 s instead of
        # the two-hour default). Linux-only options; the silence watchdog
        # covers the rest.
        for opt, val in (("TCP_KEEPIDLE", 20), ("TCP_KEEPINTVL", 5),
                         ("TCP_KEEPCNT", 3)):
            num = getattr(socket, opt, None)
            if num is None:
                continue
            try:
                self._sock.setsockopt(socket.IPPROTO_TCP, num, val)
            except OSError:
                pass
        self._write_timeout = float(write_timeout)
        self.name = f"tcp://{host}:{int(port)}"
        self._authenticate(key, connect_timeout)

    # Proves the host holds the link key during connect:
    #   board -> {"evt":"auth","nonce":"<32 hex>"}
    #   host  -> {"cmd":"auth","mac":"<64 hex>"}    HMAC-SHA512(key, nonce)[:32]
    #   board -> {"evt":"auth","ok":1}
    # A board with no key sends no challenge.
    def _authenticate(self, key: Optional[str], timeout: float) -> None:
        """
        Answer the bridge's link-key challenge on a freshly opened socket.

        :param key: the link key, or None/"" when none is configured
        :param timeout: seconds to allow for the exchange
        """
        deadline = time.monotonic() + max(float(timeout), 2.0)
        buf = b""
        self._sock.settimeout(0.2)
        try:
            while time.monotonic() < deadline and b"\n" not in buf:
                try:
                    chunk = self._sock.recv(256)
                except (socket.timeout, TimeoutError):
                    if not key:
                        return          # no key configured: nothing to wait for
                    continue
                if not chunk:
                    error_str = f"{self.name}: closed during authentication"
                    raise OSError(error_str)
                buf += chunk
            if b"\n" not in buf:
                # No challenge: an open board with nothing buffered.
                return
            line = buf.split(b"\n", 1)[0].decode("utf-8", "replace")
            if '"auth"' not in line or '"nonce"' not in line:
                # Not a challenge: this board is open and has started talking.
                # Push it back so the reader sees the frame just consumed.
                self._pushback = buf
                return
            if not key:
                error_str = (
                    f"{self.name}: the bridge asked for a link key and none is "
                    f"configured -- set tcp_key to match the board")
                raise OSError(error_str)
            nonce = bytes.fromhex(json.loads(line)["nonce"])
            mac = hmac.new(key.encode(), nonce, hashlib.sha512).digest()[:32]
            self._sock.sendall(
                f'{{"cmd":"auth","mac":"{mac.hex()}"}}\n'.encode())
            reply = buf.split(b"\n", 1)[1]
            while time.monotonic() < deadline and b"\n" not in reply:
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
                raise OSError(error_str)
        finally:
            self._sock.settimeout(self._timeout)

    def read(self, size: int = 1) -> bytes:
        """
        Read up to ``size`` bytes.

        :param size: maximum bytes to return
        :return bytes: the bytes read, or b"" if the read timed out
        :raises OSError: if the far end closed the connection
        """
        # The handshake reads by the bufferful and can take a byte of the
        # first real frame with it; serving that back here keeps the first
        # status line on an open board.
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
        """
        Send all of ``data``, or give up after ``write_timeout``.

        :param data: bytes to send
        :return int: bytes written
        :raises _SerialTimeout: if the far end would not take them in time
        """
        view = memoryview(data)
        deadline = time.monotonic() + self._write_timeout
        while view:
            if time.monotonic() >= deadline:
                error_str = f"{self.name}: Write timeout"
                raise _SerialTimeout(error_str)
            try:
                sent = self._sock.send(view)
            except (socket.timeout, TimeoutError, BlockingIOError):
                continue                    # the deadline above ends this
            if not sent:
                error_str = f"{self.name}: bridge closed the connection"
                raise OSError(error_str)
            view = view[sent:]
        return len(data)

    def close(self) -> None:
        """
        Close the socket, ignoring an already-dead one.
        """
        try:
            self._sock.close()
        except OSError:
            pass


# ── Bridge connection (threaded reader, reactor hop) ────────────────────────────

# Per-chunk write timeout during a firmware transfer: the bridge stops
# answering while it programs a staging sector.
BB_RAW_WRITE_TIMEOUT = 5.0


# One BambuBridge per serial port, shared by every unit on that Pico. Read it
# as `<module>._BRIDGES`, not an imported name, so a rebinding (e.g. in tests)
# is always seen.
_BRIDGES: Dict[str, "BambuBridge"] = {}


class BambuBridge:
    """
    Serial link to the Pico bridge: background reader + JSON command writer.

    One bridge per physical Pico. Multiple AFC units (daisy-chained AMS on the
    same bus) share it and each register a status listener via add_listener().
    """

    # Console narration lines per second before the rest of a burst goes to
    # AFC.log only, so a retry storm cannot stall the reactor.
    NARRATION_CONSOLE_MAX_PER_S = 12

    def __init__(self, serial_factory: Callable[[], Any], reactor: Any,
                 logger: Any) -> None:
        """
        Set up the bridge connection; the reader starts on connect.

        :param serial_factory: Zero-arg callable returning an open pyserial-like
          port (injectable so tests can supply a fake)
        :param reactor: The Klipper reactor (for register_async_callback)
        :param logger: AFC logger
        """
        self._serial_factory = serial_factory
        self.reactor = reactor
        self.logger = logger
        # Interpolated by _narrate_human. Not a unit name: one bridge can carry
        # several units and the narration is bus-wide; the address in each
        # message identifies the unit.
        self.name = "bridge"
        self._listeners: List[Callable[[dict], None]] = []
        self._serial: Optional[Any] = None
        self._thread: Optional[threading.Thread] = None
        self._run = False
        self._latest: Optional[dict] = None
        self._lock = threading.Lock()
        self._last_dbg: Optional[str] = None
        self._last_human: Optional[str] = None
        self._last_chmb_t: float = 0.0
        self._last_human_t: float = 0.0
        # Drying records keyed by chain index, which the firmware stamps on
        # every line; keying by device address would merge chain-mates.
        #: unit -> {"temp","target","state","seen"}: the [AMS_CHMB] telemetry.
        self._chmb_by_unit: Dict[int, dict] = {}
        #: addr -> True between "first detected" and the unit's cycle end. A
        #: calibrate must not cut across a live measurement.
        self._meas_live: Dict[int, bool] = {}
        # When each in-flight measurement was declared, so a stalled one cannot
        # latch forever. See cap_calibrating.
        self._meas_live_t: Dict[int, float] = {}
        # When each device last said it finished (see _RFID_TERMINAL_RE).
        self._rfid_term_by_addr: Dict[int, float] = {}
        #: unit -> the last drying refusal, so the panel can say why a start
        #: did nothing. Cleared when that unit actually begins a cycle.
        self._dry_err_u: Dict[int, str] = {}
        #: unit -> {"rotate","dur","tmpr"}, as the unit last echoed them.
        self._dry_cfg_u: Dict[int, Dict[str, int]] = {}
        #: unit -> {"rst","t"}; the AMS HT's calibration verdict, which it
        #: reports instead of the boxed units' percent line.
        self._ht_cali_u: Dict[int, Dict[str, Any]] = {}
        # Scan-window scoped (one scan runs at a time, globally enforced), so
        # the address is unambiguous for the window that is open.
        self._cap_measure: Dict[int, Dict[str, Any]] = {}
        # Dedicated narration log; see set_narration_log().
        self._nar_lg: Optional[Any] = None
        # The AMS's self-calibrated PTFE path length (mm) per narrating
        # address; only positive values are stored. Preferred over any
        # configured value.
        self._tube_by_addr: Dict[int, float] = {}
        # Per-unit path length, keyed by the commanded unit: an AMS 1 and an
        # AMS 2 Pro share 0x0700. The address map is the single-unit fallback.
        self._tube_by_unit: Dict[int, float] = {}
        # Per-unit dw_len and how often it was seen, keyed by unit because
        # boxed units share 0x0700.
        self._dw_by_unit: Dict[int, float] = {}
        self._dw_n_by_unit: Dict[int, int] = {}
        # The device address each value arrived on, so a consumer can refuse a
        # value from another unit's device (HT 0x1800 vs boxed 0x0700).
        # _active_unit names the last loader, which is wrong after the load.
        self._dw_addr_by_unit: Dict[int, int] = {}
        self._active_unit: Optional[int] = None
        # Repeat tracking for the narration dedupe. An identical line is
        # re-emitted periodically with a count, so a repeating fault stays
        # visible as a fault rather than being suppressed into silence.
        self._last_dbg_n: int = 0
        self._last_dbg_t: float = 0.0
        # Monotonic time of the last successful tag read (see
        # _RFID_READ_OK_RE). None means never seen, since 0.0 can be a real
        # reactor time.
        self._rfid_ok_t: Optional[float] = None
        # Monotonic time the last scan cycle ended (see _RFID_CYCLE_END_RE),
        # success or failure alike. Bridge-wide, as above.
        self._rfid_end_t: Optional[float] = None
        # The same stamps per device, so an AMS 1 read cannot credit an HT
        # mid-scan. The address cannot separate two boxed units.
        self._rfid_ok_by_addr: Dict[int, float] = {}
        self._rfid_end_by_addr: Dict[int, float] = {}
        # When each device last said its own retries were spent. See
        # _AMS_GAVE_UP_RE and gave_up_since().
        self._gave_up_by_addr: Dict[int, float] = {}
        # When each device last refused a move for having no tray, and when the
        # writer last dropped a command: together, "the unit is not listening".
        self._no_tray_by_addr: Dict[int, float] = {}
        # The last switch state each bay narrated, and when, keyed by
        # (address, chain index, bay). See tray_switches().
        self._tray_sw: Dict[Tuple[int, int, int], Tuple[int, float]] = {}
        self._write_drop_t: Optional[float] = None
        # Timed-out writes, counted for the reader's silence line: the writer
        # only ever increments the first, the reader only ever moves the
        # second, and the difference is how many went unanswered in a gap.
        self._write_timeouts: int = 0
        self._gap_timeouts_seen: int = 0
        # Link state, so "down" stops being indistinguishable from "quiet": when the
        # outage started, how many there have been (one message per outage), and when
        # a frame last arrived (silence on a nominally-up link is noticed too).
        self._down_t: Optional[float] = time.monotonic()
        self._down_epoch: int = 0
        self._last_frame_t: Optional[float] = None
        # When the current link opened, and whether it may discount its first
        # CONNECT_GRACE_S of quiet. Grace is granted only if the previous link
        # spoke, so a mute flapping bridge cannot re-arm the pause forever.
        self._connected_t: Optional[float] = None
        self._grace_this_conn: bool = False
        self._spoke_since_connect: bool = True
        self._silence_logged_t: Optional[float] = None
        self._drop_logged_epoch: int = -1
        # The unit said a chip is present but its keys are not Bambu's
        # ("auth fail -4"). Distinguishes a third-party tag from an empty bay.
        self._rfid_foreign_t: Optional[float] = None
        self._rfid_foreign_by_addr: Dict[int, float] = {}
        # Last motion completion the AMS reported, as (sequence, ok, text). The
        # only reliable end-of-move signal; the sequence tells fresh from
        # stale.
        self._finish_seq: int = 0
        # Kept apart from _finish_seq: this token trails the `feed finish` of a
        # load and would end the next wait early. Only the retract asks for it
        # (see _wait_move's accept_switch_finish).
        self._switch_seq: int = 0
        self._switch_text: str = ""
        # Bumped when the unit finishes the push-forward half.
        self._assist_seq: int = 0
        self._finish_ok: bool = False
        self._finish_text: str = ""
        # The AMS's own named fault reports ("feed finish -1, stall", ...),
        # more reliable than inferring from buffer position. The sequence tells
        # fresh faults from handled ones.
        self._fault_seq: int = 0
        self._fault_text: str = ""
        # Per unit, the _fault_seq at that unit's last fault, so one unit's
        # fault does not fail another's load. Keyed on the narration's unit
        # index.
        self._fault_seq_by_unit: Dict[int, int] = {}
        self._fault_text_by_unit: Dict[int, str] = {}
        # The last fault no unit could be credited with (the firmware sends
        # unit -1 when it cannot tell), as a stamp of _fault_seq.
        self._fault_seq_unattr: int = 0
        self._fault_text_unattr: str = ""
        # The unit's current tray and its release edge, the AMS 2's unload
        # completion (see _TRAY_NOW_RE). Per unit, as boxed units share 0x0700.
        self._tray_now_by_unit: Dict[int, int] = {}
        self._tray_release_seq_by_unit: Dict[int, int] = {}
        self._tray_release_from_by_unit: Dict[int, int] = {}
        # The unit's last reported error level (0 = healthy) and when.
        # None means it has never said, which is not the same as zero.
        self._err_code: Optional[int] = None
        self._err_code_t: float = 0.0
        self._bldc_i: float = 0.0
        self._chain_uids: List[str] = []       # index -> 24-hex UID (from `chain`)
        # Parsed `chain` replies so far; chain_snapshot hands it out so a
        # reader can tell two reads of the same reply from a new one.
        self._chain_seq: int = 0
        self._last_raw_reply: str = ""         # last `reply` frame (diagnostic)
        # Last idsave outcome as (state, n), or None; AFC_BAMBU_SAVEIDS waits
        # on it instead of a clock.
        self._last_idsave: Optional[tuple] = None
        # Last fw state as (state, detail); AFC_BAMBU_FLASH waits on these
        # answers (ready, crc_ok, crc_bad, stalled).
        self._last_fw: Optional[tuple] = None
        # True during a raw firmware transfer: send() then writes only fw*
        # commands, since a raw stream cannot survive interleaved writes.
        self._fw_raw: bool = False
        # Outbound bytes to the writer thread. Bounded, but sized for a whole
        # reconnect re-announce (see WRITE_PACE_DEPTH).
        self._wq: "queue.Queue" = queue.Queue(maxsize=512)
        self._wthread: Optional[threading.Thread] = None
        # Last {"evt":"info"} payload. Older firmware omits "chip"; callers
        # must treat that as "unknown", never as a particular chip.
        self._info: Optional[dict] = None
        #: Capture integrity from the firmware's counters: host pipe drops and
        #: UART overruns. A capture is complete only if both stay 0.
        self._sniff_sq: Optional[int] = None
        self._sniff_lost: int = 0
        self._sniff_ovr: int = 0
        self._sniff_ovr0: Optional[int] = None
        self._sniff_rf: int = 0
        self._sniff_rf0: Optional[int] = None
        # True between sending {"cmd":"reset"} and the disconnect it causes, so
        # the reader can tell "the Pico is rebooting on request" from
        # "the link died".
        self._expect_reset: bool = False
        # Bus-wide spool-operation ownership (see try_claim_bus).
        self._bus_owner: Optional[str] = None
        self._bus_claim_t: float = 0.0
        # unit -> the MC address the firmware read back: the receipt that an
        # mcaddr command arrived and was applied.
        self._mcaddr_ack: Dict[int, int] = {}
        # Last fstate seen, for the change-only trace. _UNSET (not None) so the
        # very first frame is recorded (a unit that comes up in a mode and
        # never leaves it is still logged), and None is a legitimate value.
        self._fstate_last: Any = _UNSET
        # Latch for the tray-gone edge. The unit repeats "odom tray_id error
        # 255" for as long as it is asked, so only the rising edge is a
        # completion; re-armed when a tray is engaged again.
        self._tray_gone: bool = False
        # Last buffer position the unit reported; None until it says one.
        self._buff_pos: Optional[float] = None
        # Last buffer refill as (sagged_to, recovered_to, mm_fed). mm is None
        # when the line omitted det.
        self._buff_refill: Optional[Tuple[float, float, Optional[float]]] = None
        self._reconnect_cbs: List[Callable[[], None]] = []
        #: Bridge uptime (ms) from the last `info`, and whether it went
        #: backwards (a reboot, not a blip). None until known or on firmware
        #: older than 1.88, which means "cannot tell".
        self._fw_up_ms: Optional[int] = None
        self._fw_rebooted = False
        #: Whether `info` has been asked for on this connection. See
        #: _mark_connected: the ask rides the first status frame, because that
        #: is the first moment the link is provably authenticated.
        self._info_asked = False

    def add_listener(self, cb: Callable[[dict], None]) -> None:
        """
        Register a callback invoked (on the reactor) with each status frame.

        :param cb: Callable taking one decoded status dict
        """
        if cb not in self._listeners:      # idempotent: a re-claim must not
            self._listeners.append(cb)     # stack a second copy of the same cb

    def remove_listener(self, cb: Callable[[dict], None]) -> None:
        """
        Detach a status callback (no-op if it was never added).

        :param cb: the callback add_listener was given
        """
        try:
            self._listeners.remove(cb)
        except ValueError:
            pass

    def consume_reboot(self) -> bool:
        """
        Whether the bridge has rebooted since this was last asked.

        One-shot: the recovery it gates (a chain relink) is chain-wide, and
        every claimed unit gets its own reconnect callback, so without this a
        bridge reboot on a two-unit bus would fire two relinks, the second
        landing in the middle of the first.

        :return bool: True exactly once per reboot
        """
        if not self._fw_rebooted:
            return False
        self._fw_rebooted = False
        return True

    def replay_reconnect_listeners(self) -> None:
        """
        Tell every claimed unit to re-push its config, without a reconnect.

        A relink is a chain-wide teardown: the firmware deregisters everything
        and the units re-enroll, so their unit count, HT flags, MC addressing
        and model all have to be sent again, the work the reconnect
        listeners already do. A relink issued for one unit therefore has to
        re-announce every other unit on the bus too.

        Runs on the reactor, because callers are on the reader thread or in a
        command, and the listeners announce.
        """
        for cb in list(self._reconnect_cbs):
            try:
                self.reactor.register_async_callback(lambda et, c=cb: c())
            except Exception:
                try:
                    cb()
                except Exception:
                    pass

    def add_reconnect_listener(self, cb: Callable[[], None]) -> None:
        """
        Register a callback invoked (on the reactor) after the serial link is
        re-established.

        A reconnect usually means the Pico rebooted (reflash, power-cycle,
        replug), which resets the firmware's per-unit state (polled unit count,
        HT flags) and may reshuffle enrollment; units re-push their config from
        here.

        :param cb: Zero-arg callable
        """
        if cb not in self._reconnect_cbs:
            self._reconnect_cbs.append(cb)

    def remove_reconnect_listener(self, cb: Callable[[], None]) -> None:
        """
        Detach a reconnect callback (no-op if it was never added).

        :param cb: the callback add_reconnect_listener was given
        """
        try:
            self._reconnect_cbs.remove(cb)
        except ValueError:
            pass

    def request_info(self) -> None:
        """
        Ask the bridge to describe itself.

        `info` does not arrive unprompted, and chip() stays None until it
        does. Cheap, and the answer is needed before any firmware image is
        sent.
        """
        try:
            self.send({"cmd": "info"})
            self.logger.info("AFC bambu: request_info -> sent {\"cmd\":\"info\"}")
        except Exception:
            self.logger.warning("AFC bambu: request_info FAILED to send",
                                traceback=traceback.format_exc())

    def start(self, defer_open: bool = False) -> None:
        """
        Open the port, and spin up the reader and writer threads.

        :param defer_open: keep going when the first open fails, leaving the
          reader's backoff loop to connect. A network bridge is legitimately
          not up yet at Klipper start (still booting, no address from the AP,
          relay not started), and the reconnect path already handles exactly
          that, so raising here would refuse a link that works seconds later
          and would need a Klipper restart to pick it up. A USB port missing
          at boot is a different claim (wrong device path, Pico unplugged), so
          that still raises, which is why this defaults to off.
        """
        try:
            self._serial = self._serial_factory()
            # Cleared here too: _down_t is stamped at construction and start()
            # skips the reconnect path, so the first outage would report the
            # construction age.
            self._down_t = None
            self._mark_connected()
        except Exception as e:
            if not defer_open:
                raise
            self._serial = None
            self.logger.warning(
                f"AFC bambu: bridge not reachable yet ({e}); the reader will "
                f"keep trying")
        self._run = True
        self._thread = threading.Thread(target=self._reader,
                                        name="afc_bambu_read", daemon=True)
        self._thread.start()
        self._wthread = threading.Thread(target=self._writer,
                                         name="afc_bambu_write", daemon=True)
        self._wthread.start()

    # A write to a busy Pico blocks, and a reactor stall causes "Timer too
    # close", so reactor callbacks only queue bytes and this thread waits.
    def _writer(self) -> None:
        """
        Writer-thread loop: drain the queue onto the port and absorb stalls.
        """
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
                continue                  # dropped mid-flight; the reader reconnects
            try:
                s.write(data)
            except _SerialTimeout:
                # Busy is not gone, so debug, not a warning. Stamped so the
                # load path can tell "still working" from "not listening"; the
                # writer owns it.
                self._write_drop_t = time.monotonic()
                self._write_timeouts += 1
                # Timed out is not dropped: on USB the line usually goes out
                # late and in order. Named so a late feed or stop stands out
                # from a poll.
                try:
                    _tmo = getattr(s, "_write_timeout", None)
                    if _tmo is None:
                        _tmo = getattr(s, "write_timeout", None)
                    _after = (f" after {float(_tmo):g} s"
                              if isinstance(_tmo, (int, float)) and _tmo > 0
                              else "")
                except Exception:
                    _after = ""
                # AFC.log only, even with the console debug printout on: the
                # write nearly always lands late and its ack follows, so it is
                # not a console-worthy failure.
                self.logger.debug(
                    f"AFC bambu: bridge busy, write of "
                    f"'{_queued_cmd_name(data)}' timed out{_after} "
                    f"(may still be delivered)", only_debug=True)
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu: bridge write failed: {e}; reconnecting")
                self._drop_port()
            # Space out a backlog so a reconnect's re-announce does not go out
            # as one burst (see WRITE_PACE_DEPTH). Never during a firmware
            # transfer: that path drains the queue against a deadline.
            if not self._fw_raw and self._wq.qsize() > WRITE_PACE_DEPTH:
                time.sleep(WRITE_PACE_S)

    def _drain_writes(self, timeout: float = 2.0) -> bool:
        """
        Wait for the queued writes to reach the wire.

        The firmware transfer writes its image directly: it is driven from a
        g-code handler that already paces itself, and it needs the error return
        that a queue cannot give. So it has to know that the fwbegin queued
        ahead of it has actually gone out, or the image lands in front of the
        command that arms it.

        :param timeout: seconds to wait
        :return bool: True if the queue emptied
        """
        end = time.time() + timeout
        while time.time() < end:
            if self._wq.empty():
                return True
            time.sleep(0.005)
        return self._wq.empty()

    def stop(self) -> None:
        """
        Signal the reader to stop and close the port.
        """
        self._run = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass

    def last_err_code(self) -> Tuple[Optional[int], float]:
        """
        Return the unit's last reported error level and when it said it.

        This is a level, not an event: 0 means the unit currently reports no
        error, and None means it has never reported one at all, which is not
        the same thing and must not be treated as healthy.

        Use it to answer "is this unit still in error", e.g. before resuming a
        print. Do not use it to detect a fault occurring: err_code cycles
        during healthy operation; the stall detector keys on the unit's words
        instead.

        :return tuple: (err_code or None, monotonic time it was reported)
        """
        with self._lock:
            return (self._err_code, self._err_code_t)

    def last_scan_end(self) -> Optional[float]:
        """
        When the AMS last said its scan/measure cycle finished.

        The unit announces this ("Calibration rst:0" on an HT,
        "STEP7:finish,cali tray" on an AMS 1, "STEP7:cali end" on an AMS 2),
        so a caller can wait for the real end instead of guessing with a
        timer.

        :return float: monotonic time of the last cycle end, or None
        """
        with self._lock:
            return self._rfid_end_t

    # One spool operation on the bus at a time, across every unit, as a real
    # printer does. Released by the unit's cycle-end marker; this timer is only
    # a backstop (a real scan-and-measure takes ~60 s).
    BUS_CLAIM_MAX_S = 120.0

    def try_claim_bus(self, owner: str, now: float) -> bool:
        """
        Claim the bus for a spool operation, or report that someone else has it.

        :param owner: a stable name for the claimant (the unit's name)
        :param now: reactor monotonic time
        :return bool: True if the caller may proceed
        """
        with self._lock:
            cur = getattr(self, "_bus_owner", None)
            if cur is not None and cur != owner:
                claimed = getattr(self, "_bus_claim_t", 0.0)
                ended = self._rfid_end_t
                done = (ended is not None and ended >= claimed)
                if not done and (now - claimed) < self.BUS_CLAIM_MAX_S:
                    return False                  # genuinely busy elsewhere
            self._bus_owner = owner
            self._bus_claim_t = now
            return True

    def release_bus(self, owner: str) -> None:
        """
        Drop the claim if this owner holds it. Safe to call otherwise.

        :param owner: the claimant name that was passed to claim_bus
        """
        with self._lock:
            if getattr(self, "_bus_owner", None) == owner:
                self._bus_owner = None

    def bus_owner(self) -> Optional[str]:
        """
        Who holds the bus for a spool operation, if anyone.

        :return str: the holder, or None when the bus is free
        """
        with self._lock:
            return getattr(self, "_bus_owner", None)

    def last_fault(self, unit: Optional[int] = None) -> Tuple[int, str, float]:
        """
        Return the AMS's last self-reported stall.

        With ``unit`` given, answers for that unit only. Both boxed units share
        device address 0x0700, so the narration's unit index is the only thing
        that separates them; without it a stall on one surfaces as a fault on
        the other's load.

        Omitting ``unit`` answers for the bridge as a whole.

        :param unit: chain index, or None for the bridge as a whole
        :return tuple: (sequence, text, last motor current in A)
        """
        with self._lock:
            if unit is None:
                return (self._fault_seq, self._fault_text, self._bldc_i)
            # The newer of this unit's last fault and the last unattributed
            # one; an unattributed stall counts for every unit.
            _u = int(unit)
            mine = self._fault_seq_by_unit.get(_u, 0)
            if mine >= self._fault_seq_unattr:
                return (mine, self._fault_text_by_unit.get(_u, ""),
                        self._bldc_i)
            return (self._fault_seq_unattr, self._fault_text_unattr,
                    self._bldc_i)

    def set_narration_log(self, log_dir: str, tag: str = "",
                          max_bytes: int = 10 * 1024 * 1024) -> bool:
        """
        Send the AMS's own narration to its own file.

        Narration gets its own file and handler: always written, never on the
        console, and independent of AFC's `debug` flag, so every STEP, finish,
        stall and measured length stays on record even with debug off.
        Rotates at 10 MB keeping one backup, so on disk it tops out at ~20 MB
        (the live file plus one rolled copy) and the previous chunk survives a
        rollover rather than being wiped.

        :param log_dir: directory to write into (Klipper's log directory)
        :param max_bytes: rotate at this size; 0 disables rotation
        :param tag: short suffix distinguishing a second bus master's file
        :return bool: True if the log is ready
        """
        if self._nar_lg is not None:
            return True
        # One log per bus master: two Picos in one file cannot be untangled,
        # since boxed units on different buses both narrate as 0x0700. The
        # first master's tag is empty, so a single Pico writes plain
        # AFC_BambuAMS.log.
        suffix = ("_" + tag) if tag else ""
        lg = logging.getLogger("AFC_BambuAMS_file" + suffix)
        # Test for the handler actually needed: getLogger() is process-global,
        # so another handler may already be attached.
        if not any(isinstance(h, logging.handlers.RotatingFileHandler)
                   for h in lg.handlers):
            try:
                fh = logging.handlers.RotatingFileHandler(
                    os.path.join(log_dir, f"AFC_BambuAMS{suffix}.log"),
                    maxBytes=max_bytes, backupCount=1)
                fh.setFormatter(logging.Formatter(
                    "%(asctime)s %(message)s", datefmt="%H:%M:%S"))
                lg.addHandler(fh)
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu: could not open AFC_BambuAMS{suffix}.log: {e}")
                return False
        lg.setLevel(logging.DEBUG)
        # Never propagate: this would otherwise duplicate every narration line
        # into AFC.log, which is the flooding this file exists to avoid.
        lg.propagate = False
        self._nar_lg = lg
        return True

    def _narrate_to_file(self, text: str, addr: Optional[int],
                         unit: Optional[int] = None) -> None:
        """
        Record one narration line verbatim.

        Written before the console dedupe, so a line repeating hundreds of
        times still shows the shape of a stuck loop. The address is included
        so a bus carrying several units stays attributable.

        The address alone does not attribute a line when two units share a
        class: both boxed AMSs answer at 0x0700. So the chain index the
        firmware stamps on each line (it refuses to guess across a class
        mismatch) is logged too, as "uN", or "u?" when absent.

        :param text: the raw narration line
        :param addr: device address that produced it, if known
        :param unit: chain index the firmware attributed it to, if it could
        """
        lg = self._nar_lg
        if lg is None or not text:
            return
        try:
            lg.debug(f'{((f"0x{int(addr):04X}") if addr else "0x----")}'
                     f'{(f" u{int(unit)}" if unit is not None else " u?")}'
                     f' {text}')
        except Exception:
            pass          # a log must never take the reader thread down

    def last_buff_pos(self) -> Optional[float]:
        """
        The buffer position the unit last reported, or None.

        Reported at two moments: "e_in" as the filament enters the extruder,
        and "feed finish" when the load completes. End-of-load reads ~1.28 on
        an HT (hard compressed), which is the reference for buffer ramming.

        :return Optional[float]: the reading, or None if it has said none
        """
        with self._lock:
            return self._buff_pos

    def last_buff_refill(self) -> Optional[tuple]:
        """
        The unit's last buffer refill: (sagged_to, recovered_to, mm_fed).

        This is the ramming event as the AMS measures it: how far the buffer
        sagged when the extruder pulled, and how much filament it fed to bring
        it back. mm_fed is None if the line carried no `det`.

        :return Optional[Tuple[float, float, Optional[float]]]: the refill
        """
        with self._lock:
            return self._buff_refill

    def _note_dry_refusal(self, text: Optional[str], addr: Optional[int],
                          unit: Optional[int] = None) -> None:
        """
        Record, or clear, a drying refusal for the device that narrated it.

        Recorded before the dedupe, because the AMS repeats the refusal on
        every retry and a deduped repeat still means "still refusing". Cleared
        as soon as that unit reports heating or self-checking, so a stale
        reason cannot outlive the condition.

        :param text: the narration line
        :param addr: device address that produced it
        :param unit: chain index when the caller already knows it
        """
        if not text or not addr:
            return
        try:
            m = _DRY_REFUSED_RE.search(text)
            if m:
                if unit is None:
                    return          # unattributable stray
                with self._lock:
                    self._dry_err_u[unit] = m.group(1).strip()
            elif ("CTC_STATE_HEATING" in text
                  or "CTC_STATE_SELF_CHECK" in text
                  or "check ok!" in text
                  or "shell ok!" in text):
                # "dry_mode:1, check ok!" is an accepted start in both
                # dialects, and "ams-ht shell ok!" is the HT's lid closing;
                # either clears the shell-open note.
                if unit is None:
                    return
                with self._lock:
                    self._dry_err_u.pop(unit, None)
        except Exception:
            pass          # a diagnostic must never take the reader thread down

    def _note_dry_cfg(self, text: Optional[str], addr: Optional[int],
                      unit: Optional[int] = None) -> None:
        """
        Record the drying settings a device echoes back.

        Kept even when the command is refused: the echo reports what the unit
        is holding, which a refusal does not erase. Cleared only by a cycle
        finishing with dur:0, which the unit emits itself.

        :param text: the narration line
        :param addr: device address that produced it
        :param unit: chain index when the caller already knows it
        """
        if not text or not addr:
            return
        try:
            m = _DRY_CFG_RE.search(text)
            if not m:
                return
            if unit is None:
                return              # unattributable stray
            with self._lock:
                self._dry_cfg_u[unit] = {
                    "rotate": 1 if (int(m.group(1)) or int(m.group(2))) else 0,
                    "dur":    int(m.group(5)),
                    "tmpr":   int(m.group(6)),
                }
        except Exception:
            pass          # a diagnostic must never take the reader thread down

    def last_dry_cfg(self, unit: Optional[int]) -> Optional[Dict[str, int]]:
        """
        The drying settings this unit last echoed, or None.

        :param unit: chain index
        :return dict: dict with rotate/dur/tmpr, or None if it has never echoed
        """
        if unit is None:
            return None
        with self._lock:
            cfg = self._dry_cfg_u.get(int(unit))
            return dict(cfg) if cfg else None

    def _attach_save_tray(self, sv: Optional[Any], addr: Optional[int]) -> None:
        """
        Label the current measurement with the bay the unit named for it.

        :param sv: a _CAP_SAVE_RE match, or None
        :param addr: device address the narration came from
        """
        if not sv or not addr:
            return
        try:
            with self._lock:
                rec = self._cap_measure.get(int(addr))
                # The save must belong to this reading: matched on radius (six
                # decimals against three), so a save from an earlier cycle
                # cannot re-label it.
                if (rec is not None
                    and rec.get("radius_m") is not None
                    and abs(round(float(sv.group(2)), 3)
                            - float(rec["radius_m"])) < 0.0005):
                    rec["save_tray"] = int(sv.group(1))
                    # Keep the full-precision radius too: near a full spool one
                    # millimetre of radius is ~2% of the mass. The rounded one
                    # is what the percent came from.
                    rec["save_radius_m"] = float(sv.group(2))
        except Exception:
            pass          # a label is a diagnostic; it may never raise

    def _note_cap_measure(self, text: Optional[str], addr: Optional[int],
                          now: float, unit: Optional[int] = None) -> None:
        """
        Record a narrated capacity measurement (see _CAP_MEASURE_RE).

        :param text: the narration line
        :param addr: device address that produced it
        :param now: reactor-monotonic receive time, so consumers can tell a
            fresh measurement from a stale one
        :param unit: chain index when the caller already knows it
        """
        if not text or not addr:
            return
        try:
            # The AMS HT's calibration verdict, which carries no percent (see
            # _HT_CALI_RST_RE). Recorded so a caller can tell "the cycle
            # finished" from "the cycle was never heard from".
            c = _HT_CALI_RST_RE.search(text)
            if c:
                if unit is not None:
                    with self._lock:
                        self._ht_cali_u[unit] = {"rst": int(c.group(1)),
                                                 "t": now}
            else:
                # Map the AMS 1's "odom calib success exit 0" onto rst 0. Not
                # for the HT, which emits both lines and would record two
                # verdicts.
                d = _CALI_DONE_RE.search(text)
                if d and unit is not None and int(addr) != 0x1800:
                    with self._lock:
                        self._ht_cali_u[unit] = {"rst": 0, "t": now}
            # Parsed here, applied after the store below: the save line and its
            # reading usually share a batch, so applying first would tag the
            # previous record.
            sv = _CAP_SAVE_RE.search(text)
            m = _CAP_MEASURE_RE.search(text)
            if m:
                tray, circ, radius, pct = m.groups()
            else:
                # The colon-less HT form. Tried only after the primary, so a
                # line that matches both is read by the primary pattern.
                m = _CAP_MEASURE_ALT_RE.search(text)
                if not m:
                    self._attach_save_tray(sv, addr)
                    return
                tray, radius, pct = m.groups()
                circ = None          # this form states no circumference
            with self._lock:
                self._cap_measure[int(addr)] = {
                    # Clamped for callers that drive a 0-100 display, but the
                    # raw value is kept: a fresh spool can legitimately read
                    # over 100.
                    "pct": min(100, int(pct)),
                    "pct_raw": int(pct),
                    "radius_m": float(radius),
                    # Present only on a live measurement; a restore from flash
                    # reports no circumference. None means "not stated", which
                    # is not the same as zero.
                    "circumference_m": float(circ) if circ else None,
                    # The restore form names its tray; the live form does not,
                    # because it can only be the tray just measured.
                    "tray": int(tray) if tray is not None else None,
                    # Distinguishes "the unit just measured this" from "the
                    # unit recalled it at power-on". Both are valid readings,
                    # but only the first means a spool was physically pulled.
                    "restored": tray is not None,
                    # Filled by _attach_save_tray from the following "odom save
                    # tray:N" line: the bay the unit says it measured, and that
                    # line's radius. None if unsaid.
                    "save_tray": None,
                    "save_radius_m": None,
                    "t": now,
                }
            self._attach_save_tray(sv, addr)
        except Exception:
            pass          # a diagnostic must never take the reader thread down

    def last_ht_cali(self, unit: Optional[int]) -> Optional[Dict[str, Any]]:
        """
        The most recent AMS HT calibration verdict, or None.

        :param unit: chain index
        :return dict: dict with rst / t; rst 0 = completed, 1 = refused
            ("capacity no en"), 4 = aborted (stall during calib)
        """
        if unit is None:
            return None
        with self._lock:
            m = self._ht_cali_u.get(int(unit))
            return dict(m) if m else None

    def cap_calibrating(self, addr: Optional[int]) -> bool:
        """
        Whether this device is mid-measurement right now.

        True from the unit's "first detected" until it publishes a
        measurement or ends its cycle. Used to keep a commanded calibrate
        from interrupting a pull already under way.

        :param addr: device address
        :return bool: True while a measurement is in flight
        """
        if not addr:
            return False
        with self._lock:
            if not self._meas_live.get(int(addr)):
                return False
            # Set on "first detected" and cleared on a terminal sentence. A calibrate
            # that stalls before its terminal sentence would leave the flag set and
            # refuse every later calibrate, so it expires after _MEAS_LIVE_MAX_S.
            # Aged on the reactor clock handle_line stamps it with.
            t0 = self._meas_live_t.get(int(addr))
            mono = getattr(self.reactor, "monotonic", None)
            now = mono() if callable(mono) else 0.0
            if t0 is not None and (now - t0) > _MEAS_LIVE_MAX_S:
                self._meas_live[int(addr)] = False
                return False
            return True

    def last_terminal(self, addr: Optional[int]) -> Optional[float]:
        """
        When this device last said it had finished, or None.

        Strictly 'STEP7:cali end' / 'STEP7:finish' and the two
        equivalents, not 'odom calib success', which the unit
        emits while still moving. Use this before commanding
        anything that must not overlap the unit's cycle.

        :param addr: device address
        :return float: monotonic timestamp, or None
        """
        if not addr:
            return None
        with self._lock:
            return self._rfid_term_by_addr.get(int(addr))

    def last_cap_measure(self, addr: Optional[int]) -> Optional[Dict[str, Any]]:
        """
        The most recent capacity measurement this device narrated, or None.

        :param addr: device address (an HT is 0x1800, a boxed AMS 0x0700)
        :return dict: dict with pct / radius_m / t, or None
        """
        if not addr:
            return None
        with self._lock:
            m = self._cap_measure.get(int(addr))
            return dict(m) if m else None

    def clear_dry_error(self, unit: Optional[int]) -> None:
        """
        Forget why this device last refused, because a new attempt is starting.

        The error describes the last attempt, so a new attempt is what resets
        it. Called as a dry is commanded, before anything reaches the bus.

        Narration alone is not enough: the only lines that clear it are
        CTC_STATE_HEATING and CTC_STATE_SELF_CHECK, and an AMS HT emits
        neither, so without this one refusal would read as "refused" forever.

        :param unit: chain index
        """
        if unit is None:
            return
        with self._lock:
            self._dry_err_u.pop(int(unit), None)

    def last_dry_error(self, unit: Optional[int]) -> Optional[str]:
        """
        Why this unit last refused to dry, or None if it has not.

        :param unit: chain index
        :return Optional[str]: the AMS's own wording, e.g. "filament hub load!"
        """
        if unit is None:
            return None
        with self._lock:
            return self._dry_err_u.get(int(unit))

    def _finish_succeeded(self, text: str, low: str,
                          addr: Optional[int]) -> bool:
        """
        Whether a motion completion means the filament arrived.

        "finish -1, stall" is not a failure on every unit: an AMS HT ends a
        normal load by feeding to the end of its PTFE and stalling against the
        extruder gear, which is how it knows it has arrived. Reading the word
        "stall" as failure marks a good load failed. What matters is how far
        it got, not that it stalled:

        1. no stall reported at all -> success
        2. stalled, but len_det reached tube_len (within tolerance) -> success
        3. the same line also carries a clean finish -> success
        4. otherwise -> failure

        tube_len comes from the line when present, else from this unit's last
        reported measurement, so a stall line that omits it is still judged
        against the right distance rather than defaulting to failure.

        :param text: the narration line
        :param low: the same line, lowercased (already computed by the caller)
        :param addr: device address that narrated, if known
        :return bool: True if the move achieved what it was asked to
        """
        if "finish -1" not in low and "stall" not in low:
            return True
        # A clean completion can share the line with the stall.
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
            return False          # nothing to judge against; stall stands
        return travelled >= target - FINISH_ARRIVAL_TOLERANCE_MM

    def _trace_fstate(self, obj: dict) -> None:
        """
        Record every CHANGE of the AMS's own mode into the narration log.

        `fstate` is what the move-completion wait keys on, so the trace sits
        next to the narration on the same clock. Changes only: the field rides
        every status frame, several a second, and logging all of them would
        bury the narration beside it.

        :param obj: a decoded status event
        """
        if self._nar_lg is None:
            return
        try:
            v = obj.get("fstate")
            if v == self._fstate_last:
                return
            prev, self._fstate_last = self._fstate_last, v
            self._nar_lg.debug(f'HOST-- fstate {("-" if prev is _UNSET else prev)} -> {v} (buff='
                f'{obj.get("buff")})')
        except Exception:
            pass          # a trace must never take the reader thread down

    def set_active_unit(self, unit: Optional[int]) -> None:
        """
        Name the unit currently being commanded, so narration it produces is
        attributed to it rather than to its device address.

        Two units of the same class share an address (an AMS 1 and an AMS 2 Pro
        are both 0x0700), so the address alone cannot say which one spoke,
        but the host knows, having issued the move. Set around a load, cleared
        after.

        :param unit: chain index, or None to clear
        """
        with self._lock:
            self._active_unit = None if unit is None else int(unit)

    def dw_len(self, unit: Optional[int] = None) -> tuple:
        """
        The last dw_len a unit reported at the end of a feed, and how many.

        Reported, never used (see _DW_LEN_M_RE). The count is returned with
        the value so a caller can tell one reading from a consistent series.

        :param unit: chain index; None returns the most recent from any unit
        :return tuple: (mm, count, addr), or (None, 0, 0) if never reported
        """
        with self._lock:
            if unit is not None:
                u = int(unit)
                return (self._dw_by_unit.get(u),
                        self._dw_n_by_unit.get(u, 0),
                        self._dw_addr_by_unit.get(u, 0))
            if not self._dw_by_unit:
                return (None, 0, 0)
            u = max(self._dw_n_by_unit, key=self._dw_n_by_unit.get)
            return (self._dw_by_unit.get(u),
                    self._dw_n_by_unit.get(u, 0),
                    self._dw_addr_by_unit.get(u, 0))

    def tube_len(self, addr: Optional[int] = None,
                 unit: Optional[int] = None) -> Optional[float]:
        """
        The AMS's own measured PTFE path length in mm, if it has told us.

        The unit learns this from consecutive feed measurements and narrates
        it, so it is the real distance on this machine and beats a configured
        estimate. Only available once it has enough samples; it reports 0
        until then.

        :param addr: Device address to look up (0x0700 AMS, 0x1800 HT); None
            returns the most recent from any unit
        :param unit: chain index when the caller already knows it
        :return Optional[float]: length in mm, or None if never reported
        """
        with self._lock:
            if unit is not None:
                if int(unit) in self._tube_by_unit:
                    return self._tube_by_unit[int(unit)]
                if self._tube_by_unit:
                    # This unit has not measured yet. Do not fall back to the
                    # address, which same-class units share; unknown keeps the
                    # configured value.
                    return None
            if addr is not None:
                v = self._tube_by_addr.get(int(addr))
                if v:
                    return v
                return None
            # No address: only safe to answer when exactly one unit has
            # reported. With two units on the bridge, "the most recent" could
            # hand an HT's path length to an AMS 2.
            vals = list(self._tube_by_addr.values())
            return vals[0] if len(vals) == 1 else None

    def last_finish(self) -> Tuple[int, bool, str]:
        """
        Return the AMS's last reported motion completion.

        :return tuple: (sequence, ok, text); sequence increments per event
        """
        with self._lock:
            return (self._finish_seq, self._finish_ok, self._finish_text)

    def chip(self) -> Optional[str]:
        """
        The silicon the bridge reports running on ("RP2040"/"RP2350").

        None when the firmware predates the field. That is not a licence to
        assume one: a wrong-chip image leaves a board that only BOOTSEL can
        recover, so an unknown chip means "cannot check", not "RP2040".

        :return str: chip name, or None if the bridge has not said
        """
        with self._lock:
            v = (self._info or {}).get("chip")
        return str(v) if v else None

    def last_switch_finish(self) -> Tuple[int, str]:
        """
        Return the AMS's last completed state switch (the end of a retract).

        Counted apart from last_finish() so that only a caller that wants it
        (the unload) can be ended by it. See _STATE_SWITCH_DONE_RE.

        :return tuple: (sequence, text); sequence increments per event
        """
        with self._lock:
            return (self._switch_seq, self._switch_text)

    def last_tray_release(self, unit: Optional[int] = None
                          ) -> Tuple[int, Optional[int]]:
        """
        The unit's last tray release: ``tray_now`` leaving a tray for 255.

        This is the AMS 2's most reliable unload-completion signal (see
        _TRAY_NOW_RE). It fires the instant the tray switch releases, ahead
        of the deadline the unload otherwise waits out.

        Returns the tray it left, not just that something happened, because
        the edge is not unload-specific: an operator preloading or inserting a
        spool produces the same transition on whichever tray they touched.
        A caller must require the tray it commanded.

        :param unit: chain index. Omitted or unknown answers (0, None), which
          means "nothing to report" and leaves a waiter on its deadline;
          there is no bridge-wide answer here on purpose, because both boxed
          units share device address 0x0700.
        :return tuple: (sequence, tray index released); the sequence
          increments once per release, so compare against a mark taken before
          commanding the retract
        """
        if unit is None:
            return (0, None)
        with self._lock:
            _u = int(unit)
            return (self._tray_release_seq_by_unit.get(_u, 0),
                    self._tray_release_from_by_unit.get(_u))

    def last_assist_done(self) -> int:
        """
        Sequence of the last completed assist cycle (the push-forward half).

        The unit's seating cycle is pull back, then push forward; this marks
        the end of the push. Releasing the extruder on the pull alone just
        moves the fight later.

        :return int: increments once per "assist finish"
        """
        with self._lock:
            return self._assist_seq

    def latest_status(self) -> Optional[dict]:
        """
        Return the most recent status frame (thread-safe copy).

        :return Optional[dict]: the last status dict, or None if none yet
        """
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def write_raw(self, data: bytes) -> bool:
        """
        Write bytes to the bridge with no framing at all.

        Only ever valid between {"cmd":"fwbegin"} and the firmware's answer:
        in that window the firmware is not parsing lines, it is counting an
        image. Anything sent here at any other time is read as a command and
        will not be one.

        :param data: Raw bytes to put on the wire
        :return bool: False when there is no link, or the write failed
        """
        s = self._serial
        if s is None:
            return False
        # fwbegin went through send() and may still be queued; the image must
        # not overtake it.
        if not self._drain_writes():
            self.logger.warning("AFC bambu: queued writes did not drain; "
                                "not starting the image")
            return False
        # The image gets its own write budget (a chunk waits on flash writes);
        # restored in the finally so later commands still fail fast.
        _tmo_attr = ("_write_timeout" if hasattr(s, "_write_timeout")
                     else ("write_timeout" if hasattr(s, "write_timeout")
                           else None))
        _tmo_prev = getattr(s, _tmo_attr) if _tmo_attr else None
        if _tmo_attr:
            try:
                setattr(s, _tmo_attr, BB_RAW_WRITE_TIMEOUT)
            except Exception:
                _tmo_attr = None            # pyserial can refuse mid-flight
        try:
            # Chunked so the write timeout applies per chunk; the bridge
            # throttles while it programs each staging sector.
            view = memoryview(data)
            step = 4096                     # one staging sector's worth
            while view:
                n = min(step, len(view))
                s.write(view[:n])
                view = view[n:]
            return True
        except _SerialTimeout as e:
            # Fatal to this transfer and nothing else. The caller aborts and
            # the bridge discards its partial image; the link itself is fine,
            # so it does not get torn down.
            self.logger.warning(f"AFC bambu: bridge busy mid-transfer: {e}")
            return False
        except Exception as e:
            self.logger.warning(
                f"AFC bambu: bridge raw write failed: {e}; reconnecting")
            self._drop_port()
            return False
        finally:
            if _tmo_attr:
                try:
                    setattr(s, _tmo_attr, _tmo_prev)
                except Exception:
                    pass

    def send(self, obj: dict) -> None:
        """
        Write one JSON command line to the bridge.

        :param obj: The command object to serialize and send
        """
        s = self._serial
        if s is None:
            # Log the first dropped command of each outage only, so a dead link
            # is visible without flooding the log at poll rate.
            if self._drop_logged_epoch != self._down_epoch:
                self._drop_logged_epoch = self._down_epoch
                self.logger.warning(
                    f"AFC bambu: link is down -- dropped "
                    f"{obj.get('cmd', '?')}, and anything else sent until it "
                    f"is back")
            return
        # A raw firmware transfer owns the port. Anything else here is a timer
        # poll that is free to skip and would corrupt the image.
        if self._fw_raw and not str(obj.get("cmd", "")).startswith("fw"):
            self.logger.debug(
                f"AFC bambu: dropped {obj.get('cmd')} during a firmware "
                "transfer")
            return
        # Arm the expected-disconnect flag before the write: a reset can take
        # the port down before this call even returns.
        if obj.get("cmd") in ("reset", "bootsel"):
            self._expect_reset = True
        # Queued, never written here: this runs on the reactor (see _writer).
        try:
            self._wq.put_nowait((json.dumps(obj) + "\n").encode())
            return
        except queue.Full:
            # A full queue means the link is not draining. Everything here is a
            # poll or re-arm the next tick resends, so dropping the newest is
            # cheapest.
            self.logger.warning(
                f"AFC bambu: write queue full, dropped {obj.get('cmd')}")
        # Link errors (Errno 5 after a glitch, another process taking the
        # port) happen on the write, which is the writer thread's job; it
        # drops the port for them there.

    def _narrate_human(self, text: str, now: float,
                       addr: Optional[int] = None,
                       unit: Optional[int] = None) -> None:
        """
        Surface the AMS's own words on the console, in English.

        Everything the unit says already reaches AFC.log verbatim; this picks
        the handful an operator wants to see and renders them plainly. A
        refused dry is the case that matters: the AMS answers
        "[AMS_CHMB]ignore dry_mode:1, ams_state:2" and AFC_BAMBU_HEATER_START
        reports success either way, so the refusal is otherwise invisible.

        :param text: One narration line from the AMS
        :param now: Reactor monotonic time, for rate limiting
        :param addr: Device address the line came from (0x0700 = boxed AMS,
            0x1800 = HT)
        :param unit: chain index when the caller already knows it, so chamber
            telemetry is attributed to the unit that produced it instead of
            shared across the bridge
        """
        for entry in _AMS_HUMAN:
            pattern, render = entry[0], entry[1]
            log_only = len(entry) > 2 and entry[2]
            m = pattern.search(text)
            if not m:
                continue
            # render None = matched deliberately to say nothing (see the
            # 0x16 assist-slip pair). Stop here rather than falling through
            # to a later, more general pattern that would speak for it.
            if render is None:
                return
            msg = f"AFC bambu {self.name}: {render(m)}"
            if log_only:
                # Log-only lines go to AFC.log before the console dedupe and
                # rate limit, and still reach the tube_len/dw_len/chamber
                # parsing below.
                self.logger.debug(msg, only_debug=True)
                continue
            # Consecutive duplicates say nothing; the AMS repeats state lines.
            if msg == getattr(self, "_last_human", None):
                continue
            # Hard floor between console lines. The AMS narrates continuously
            # and a burst (a load, a dry start) can match several patterns in
            # under a second; the console is the operator's, not a log tail.
            if now - getattr(self, "_last_human_t", 0.0) < 1.0:
                continue
            self._last_human = msg
            self._last_human_t = now
            self.logger.info(msg)
        # The unit's own PTFE measurement, stored per address so an AMS 2 and
        # an HT on one bridge keep their own path lengths.
        mm = None
        tl = _TUBE_LEN_MM_RE.search(text)
        if tl:
            mm = float(tl.group(1))
        else:
            tl = _TUBE_LEN_M_RE.search(text)
            if tl:
                mm = float(tl.group(1)) * 1000.0
        # > 0 only: the unit reports 0.000 m until it has calibrated, and
        # adopting that would set every deadline to zero.
        if mm and mm > 0.0 and addr:
            with self._lock:
                prev = self._tube_by_addr.get(int(addr))
                self._tube_by_addr[int(addr)] = mm
                if self._active_unit is not None:
                    self._tube_by_unit[int(self._active_unit)] = mm
            if prev is None:
                self.logger.info(
                    f"AFC bambu {self.name}: AMS 0x{int(addr):04X} reports its "
                    f"measured filament path as {mm:.0f}mm -- using it to size "
                    f"move timeouts instead of the configured estimate")

        # dw_len: recorded, not used (see _DW_LEN_M_RE). Same > 0 rule as
        # tube_len: a load that fails reports dw_len:0.000, meaning "this feed
        # measured nothing", which must never reach a deadline.
        dw = _DW_LEN_M_RE.search(text)
        if dw:
            dw_mm = float(dw.group(1)) * 1000.0
            if dw_mm > 0.0 and self._active_unit is not None:
                with self._lock:
                    u = int(self._active_unit)
                    self._dw_by_unit[u] = dw_mm
                    self._dw_n_by_unit[u] = self._dw_n_by_unit.get(u, 0) + 1
                    self._dw_addr_by_unit[u] = int(addr or 0)

        # Drying telemetry is reported on its own slow cadence, not per line.
        # The heater self-check lines (resistance per element, before heating
        # starts) are logged verbatim until their fields are understood:
        #   setR]state[1]:0 -> 1, pwm:0.57, i:0.75 A
        #   wind_door[0] res ok,i_0:3, i_sum:125, i_avr:627, cnt:200, res:7968
        #   PTC[0] ok! i_0:1, res:9174
        if ("[AMS_CHMB]" in text
            and ("res:" in text
                 or "i_avr:" in text
                 or " i:" in text)):
            self.logger.info(f"AFC bambu {self.name}: selfcheck | "
                             + text.split("[AMS_CHMB]", 1)[-1].strip()[:150])
        m = _CHMB_STATE_RE.search(text)
        if m:
            # Chamber temperature, streamed only here while drying (the binary
            # protocol hardcodes -1). vt is the chamber probe; ap runs ~4C
            # higher and is not published. Keyed by unit.
            try:
                rec = {"temp": float(m.group(3)), "seen": now,
                       "state": int(m.group(1)), "target": float(m.group(2))}
                # %RH alongside the temperature, when the model reports it.
                if m.group(4) is not None:
                    rec["humidity"] = int(m.group(4))
            except (TypeError, ValueError):
                rec = None
            if rec is not None and unit is not None:
                # ad: when the line carried it (see _CHMB_AD_RE), stored beside
                # the temperature it came with.
                ma = _CHMB_AD_RE.search(text)
                if ma:
                    rec["ad_n"] = int(ma.group(1))
                    if ma.group(2) is not None:
                        try:
                            rec["ad_v"] = float(ma.group(2))
                        except ValueError:
                            pass
                self._chmb_by_unit[int(unit)] = rec
        if m and now - getattr(self, "_last_chmb_t", 0.0) >= 60.0:
            self._last_chmb_t = now
            # Humidity rides along only on models that report ht:.
            hum = f", humidity {int(m.group(4))}%" if m.group(4) else ""
            ma = _CHMB_AD_RE.search(text)
            ad = ""
            if ma:
                ad = (f", ad {ma.group(1)}"
                      + (f"/{ma.group(2)}" if ma.group(2) else ""))
            self.logger.info(
                f"AFC bambu {self.name}: drying -- chamber {m.group(3)}C{ad}"
                f"{hum}, target {m.group(2)}C")

    def _rfid_stamp(self, wide: Optional[float], by_addr: Dict[int, float],
                    addr: Optional[int]) -> Optional[float]:
        """
        Pick the narration stamp for ``addr``, or the bridge-wide one.

        An address the bridge has never heard narrate falls back to the shared
        stamp rather than reading as "never", so a unit whose firmware predates
        per-device attribution is not reported silent. Once it HAS narrated its
        own stamp wins, and another unit's chatter cannot be credited to it.

        :param wide: the bridge-wide stamp
        :param by_addr: per-device stamps
        :param addr: device address to resolve, or None for bridge-wide
        :return float: the stamp to compare against, or None if there is none
        """
        if addr:
            got = by_addr.get(int(addr))
            if got is not None:
                return got
            if by_addr:
                # This bridge attributes narration and this device has said
                # nothing of the kind. That is an answer, not a gap.
                return None
        return wide

    def rfid_read_succeeded_since(self, since: Optional[float],
                                  addr: Optional[int] = None) -> bool:
        """
        Whether a tag read has landed since ``since``.

        The caller needs this to decide whether a slot's profile record
        belongs to the spool now in the bay or to the one before it: an AMS
        reports its stored record for a bay from the moment a spool goes in,
        long before the reader has seen the new tag.

        Scoped to ``addr`` when one is given: this decides whether a bay's
        record is the new spool's, so crediting it to the wrong unit applies
        wrong data. Two boxed units still share 0x0700 and cannot be separated
        this way.

        :param since: Reactor monotonic time to compare against (None = never)
        :param addr: device address to scope the answer to (0x1800 = an HT);
                     None answers for the bridge as a whole
        :return bool: True if a successful read was narrated at or after ``since``
        """
        ok = self._rfid_stamp(self._rfid_ok_t, self._rfid_ok_by_addr, addr)
        return ok is not None and since is not None and ok >= since

    def rfid_foreign_tag_since(self, since: Optional[float],
                               addr: Optional[int] = None) -> bool:
        """
        Whether the unit refused a chip it could not authenticate since ``since``.

        A Mifare tag whose keys are not Bambu's still answers anticollision, so
        its UID is readable; only the profile is locked. That is a third-party
        spool, not an empty bay, and the two need different words.

        :param since: Reactor monotonic time to compare against (None = never)
        :param addr: device address to scope the answer to
        :return bool: True if a foreign tag was refused at or after ``since``
        """
        t = self._rfid_stamp(self._rfid_foreign_t,
                             self._rfid_foreign_by_addr, addr)
        return t is not None and since is not None and t >= since

    def gave_up_since(self, since: Optional[float],
                      addr: Optional[int] = None) -> bool:
        """
        Has this device said its own retries are spent, at or after ``since``?

        The unit announces it: "AMS_CTRL_state_switch finish, fail, retry:5".
        That is a different statement from a stall: a stall is trouble the
        unit is still working through, and the printer keeps feeding through
        one on purpose. This is the retry budget being spent, after which
        continuing to ask cannot help.

        :param since: Reactor monotonic time to compare against (None = never)
        :param addr: device address to scope the answer to
        :return bool: True if the unit gave up at or after ``since``
        """
        if since is None:
            return False
        t = self._rfid_stamp(None, self._gave_up_by_addr, addr)
        return t is not None and t >= since

    def no_tray_since(self, since: Optional[float],
                      addr: Optional[int] = None) -> bool:
        """
        Has this device refused a move for having no tray selected since ``since``?

        The companion to ``gave_up_since``. That one says the unit stopped
        trying; this one says it will not accept being asked again, because it
        is no longer pointed at a bay. See ``_AMS_NO_TRAY_RE``.

        :param since: Reactor monotonic time to compare against (None = never)
        :param addr: device address to scope the answer to
        :return bool: True if the unit rejected a move at or after ``since``
        """
        if since is None:
            return False
        t = self._rfid_stamp(None, self._no_tray_by_addr, addr)
        return t is not None and t >= since

    def tray_switches(self, addr: Optional[int], unit: Optional[int],
                      slot: int) -> Optional[Tuple[int, float]]:
        """
        The last switch state a bay narrated, and when.

        The AMS narrates a change only, so this is the state since then: bit 0
        the inlet by the spool, bit 1 the outlet toward the hub.

        :param addr: device address of the unit
        :param unit: the unit's chain index
        :param slot: 0-based bay index
        :return tuple: (switch state, reactor time), or None if never narrated
        """
        if not addr or unit is None:
            return None
        return self._tray_sw.get((int(addr), int(unit), int(slot)))

    def writes_dropped_since(self, since: Optional[float]) -> bool:
        """
        Has the writer thread dropped a command since ``since``?

        "Dropped" here means a write that timed out. Bridge-wide by design: the
        Pico stopped reading, which says nothing about any one device, and
        on USB the bytes are usually delivered late rather than lost. A timeout
        on its own is ordinary: a Pico mid-recovery stops draining its CDC
        for a moment. It only means something alongside a unit that has
        already given up.

        :param since: Reactor monotonic time to compare against (None = never)
        :return bool: True if a command was dropped at or after ``since``
        """
        if since is None:
            return False
        t = self._write_drop_t
        return t is not None and t >= since

    def rfid_cycle_ended_since(self, since: Optional[float],
                               addr: Optional[int] = None) -> bool:
        """
        Whether a scan cycle has run to completion since ``since``.

        The unit narrates ``STEP7:finish,cali tray`` at the end of a scan
        whatever the outcome, so this is the moment, and the only moment,
        at which "no tag read" is a fact rather than a guess against a clock.

        Scoped to ``addr`` when one is given, like the stamps above.

        :param since: Reactor monotonic time to compare against (None = never)
        :param addr: device address to scope the answer to (0x1800 = an HT);
                     None answers for the bridge as a whole
        :return bool: True if a cycle ended at or after ``since``
        """
        end = self._rfid_stamp(self._rfid_end_t, self._rfid_end_by_addr, addr)
        return end is not None and since is not None and end >= since

    def handle_line(self, line: str) -> None:
        """
        Decode one bridge line and, for a status frame, cache + hop to reactor.

        Split out from the reader loop so it's unit-testable.

        :param line: One line of text from the bridge
        """
        obj = parse_bridge_line(line)
        if obj is None:
            return
        if obj.get("evt") == "status":
            with self._lock:
                self._latest = obj
            # The first status frame proves the link is up and authenticated,
            # so ask the bridge's identity here, once per connection.
            if not self._info_asked:
                self._info_asked = True
                self.request_info()
            self._trace_fstate(obj)
            for cb in self._listeners:
                self.reactor.register_async_callback(
                    lambda et, o=obj, c=cb: c(o))
        elif obj.get("evt") in ("reply", "raw"):
            # Raw AMS frame, held for diagnostics: {"evt":"reply","hex":...}
            # from AFC_BAMBU_BUFFER_PROBE, {"evt":"raw","rx":...} from
            # {"cmd":"raw"}.
            with self._lock:
                self._last_raw_reply = str(
                    obj.get("hex") or obj.get("rx") or "")
        elif obj.get("evt") == "error":
            self.logger.warning(f"AFC bambu: bridge error: {obj.get('msg')}")
        elif obj.get("evt") == "fw":
            # Firmware transfer, one line per stage. AFC_BAMBU_FLASH waits on
            # each answer; the CRC answer is the only receipt for a raw stream.
            state = str(obj.get("state") or "")
            detail = obj.get("detail")
            if detail is None and "want" in obj:
                # Both CRCs, because "crc_bad" alone cannot tell a stream that
                # arrived wrong from an expected value that arrived wrong.
                detail = (f"{obj.get('got')}/{obj.get('len')} bytes, "
                          f"computed 0x{int(obj.get('crc', 0)):08X}, "
                          f"expected 0x{int(obj.get('want', 0)):08X}"
                          f", head {obj.get('head', '?')}"
                          f", tail {obj.get('tail', '?')}")
            elif detail is None and "got" in obj:
                detail = f"{obj.get('got')}/{obj.get('len')} bytes"
            with self._lock:
                self._last_fw = (state, str(detail or ""))
            if state in ("crc_bad", "stalled", "refused", "busy"):
                self.logger.warning(
                    f"AFC bambu: firmware transfer {state}: {detail}")
            elif state == "applying":
                # The last thing the Pico says. Everything after this is a
                # blank link until it comes back on the new image.
                self._expect_reset = True
                self.logger.info(
                    "AFC bambu: bridge is writing its own flash and will "
                    "reboot; the link drops for a few seconds")
            else:
                self.logger.debug(f"AFC bambu: firmware transfer {state}: {detail}")
        elif obj.get("evt") == "idsave":
            # The identity table persisted on the Pico. "match" is every
            # ordinary boot; "written" should appear once after a change, so a
            # record rewritten every boot shows at info.
            state = obj.get("state")
            n = obj.get("n")
            # Latched for AFC_BAMBU_SAVEIDS, which must not reboot the Pico
            # until the firmware has actually said the record is down.
            with self._lock:
                self._last_idsave = (str(state or ""), n)
            if state == "wiped":
                self.logger.info(
                    "AFC bambu: bridge ERASED its stored unit identities; the "
                    "next power-up will enroll from announce order until prep "
                    "writes a fresh record")
            elif state == "written":
                self.logger.info(
                    f"AFC bambu: bridge stored {n} unit identities -- the next "
                    f"restart will enroll from them")
            elif state == "failed":
                self.logger.warning(
                    "AFC bambu: bridge could NOT store the unit identities; "
                    "the chain will keep enrolling from announce order")
            else:
                self.logger.debug(
                    f"AFC bambu: bridge identities already stored ({n} units)")
        elif obj.get("evt") == "ack":
            # Motion-command acks through the AFC logger. Routine ones are
            # log-only; feed and retract stay on the console.
            _ack_cmd = str(obj.get("cmd") or "")
            self.logger.debug(
                f"AFC bambu: bridge ack {_ack_cmd} (slot {obj.get('slot')})",
                only_debug=_ack_cmd in _ACK_ROUTINE)
        elif obj.get("evt") == "amsdbg":
            # The AMS's own narration. Repeats are deduplicated, not suppressed
            # forever, since a repeating line is how a stuck loop shows. The
            # "[DBG] ams time" heartbeat is dropped.
            text = obj.get("text")
            # Defensive: this runs on the reader thread, and a diagnostic must
            # never be able to take the bridge down.
            mono = getattr(self.reactor, "monotonic", None)
            now = mono() if callable(mono) else 0.0
            # The chain index of the unit whose drain pulled this text
            # (-1/absent = unknown). The line then goes verbatim to the
            # narration file, unfiltered.
            _dbg_unit = obj.get("unit")
            _dbg_unit = (int(_dbg_unit)
                         if isinstance(_dbg_unit, int) and _dbg_unit >= 0
                         else None)
            self._narrate_to_file(text, obj.get("addr"), _dbg_unit)
            # Firmware diagnostics stop here, after the file and before every
            # parser (see _FW_DIAG_RE).
            if isinstance(text, str) and _FW_DIAG_RE.match(text):
                self.logger.debug(f"AFC bambu: bridge diag {text}",
                                  only_debug=True)
                return
            self._note_dry_refusal(text, obj.get("addr"), _dbg_unit)
            self._note_dry_cfg(text, obj.get("addr"), _dbg_unit)
            self._note_cap_measure(text, obj.get("addr"), now, _dbg_unit)
            # RFID/measurement stamps from the raw line, before the dedupe
            # blanks a repeat: a repeat still shows the read is alive.
            _addr = obj.get("addr")
            _addr = int(_addr) if isinstance(_addr, int) and _addr else None
            _read_re = (_RFID_READ_OK_HT_RE if _addr == 0x1800
                        else _RFID_READ_OK_RE)
            if text and _read_re.search(text):
                self._rfid_ok_t = now
                if _addr:
                    self._rfid_ok_by_addr[_addr] = now
            if text and _addr and "first detected" in text:
                self._meas_live[_addr] = True
                self._meas_live_t[_addr] = now
            if text and _RFID_CYCLE_END_RE.search(text):
                self._rfid_end_t = now
                if _addr:
                    self._rfid_end_by_addr[_addr] = now
                    self._meas_live[_addr] = False
            if text and _addr and _MEAS_END_RE.search(text):
                self._meas_live[_addr] = False
            # Same shape and the same guards as its neighbours above, keyed on
            # the stricter pattern. Recorded, never acted on here.
            if text and _addr and _RFID_TERMINAL_RE.search(text):
                self._rfid_term_by_addr[_addr] = now
            # The unit's own "I am done retrying". Stamped here like every
            # other narration fact; the load path asks about it via
            # gave_up_since() rather than parsing text itself.
            if text and _addr and _AMS_GAVE_UP_RE.search(text):
                self._gave_up_by_addr[_addr] = now
                self.logger.info(
                    f"AFC bambu bridge: the AMS reported it has STOPPED "
                    f"retrying ({text.strip()[:90]})")
            # Same shape, same guards: the unit rejecting moves because it has
            # no bay selected. Recorded here, acted on by the load path.
            if text and _addr and _AMS_NO_TRAY_RE.search(text):
                self._no_tray_by_addr[_addr] = now
            # Bay switch changes, only when the line names its unit: two units
            # of one model share an address.
            if text and _addr and _dbg_unit is not None:
                for m in _AMS_TRAY_SW_RE.finditer(text):
                    self._tray_sw[(_addr, _dbg_unit, int(m.group(1)))] = (
                        int(m.group(3)), now)
            # A published measurement ends the in-flight state too: the unit
            # narrates the percent and, on the boxed models, may go quiet
            # afterwards without a terminal sentence at all.
            if (text
                and _addr
                and ("second detected" in text
                     or "odom save" in text)):
                self._meas_live[_addr] = False
            # A chip the unit cannot authenticate, stamped above the dedupe
            # too.
            if text and _RFID_FOREIGN_TAG_RE.search(text):
                self._rfid_foreign_t = now
                if _addr:
                    self._rfid_foreign_by_addr[_addr] = now
            # Strip the heartbeat before the dedupe: its changing timestamp
            # would break every run of identical lines.
            if text and "[DBG] ams time" in text:
                stripped = _DBG_AMSTIME_RE.sub("", text).strip()
                # Nothing left but the frame's junk byte means nothing to report: a
                # heartbeat-only frame strips down to a bare "," and would reach the console
                # every 10 seconds. Keep the line only if real narration rode along.
                text = stripped if "[" in stripped else None
            # The parsers get the raw line: two identical completions for two
            # moves must bump the sequence twice.
            raw = text
            if text and text == self._last_dbg:
                # Same line again: count it, and re-emit once a minute so a
                # loop that is going nowhere still shows up.
                self._last_dbg_n += 1
                if now - self._last_dbg_t >= 60.0:
                    self._last_dbg_t = now
                    n = self._last_dbg_n
                    self.logger.debug(f"AMS: (x{n} repeated) {text}",
                                      only_debug=True)
                text = None
            elif text:
                self._last_dbg = text
                self._last_dbg_n = 1
                self._last_dbg_t = now
            if raw:
                # Buffer position, from whichever line carries it. Recorded
                # before the completion branches below so a line that is both
                # (a feed finish carrying buff_pos) contributes both.
                bp = _BUFF_POS_RE.search(raw)
                if bp:
                    with self._lock:
                        self._buff_pos = float(bp.group(1))
                rf = _BUFF_REFILL_RE.search(raw)
                if rf:
                    det = float(rf.group(3)) if rf.group(3) else None
                    with self._lock:
                        # The position after recovery is the current one.
                        self._buff_pos = float(rf.group(2))
                        self._buff_refill = (float(rf.group(1)),
                                             float(rf.group(2)), det)
                # Motion completion.
                low = raw.lower()
                # Checked apart from the finish chain: one line can carry an
                # assist and a finish.
                if _ASSIST_DONE_RE.search(raw):
                    with self._lock:
                        self._assist_seq += 1
                # Asked separately from the finish chain below, and not part
                # of it: this must not end an ordinary load's wait.
                if _STATE_SWITCH_DONE_RE.search(raw):
                    with self._lock:
                        self._switch_seq += 1
                        self._switch_text = raw
                # tray_now leaving a tray for 255, the AMS 2's unload
                # completion. Unattributed narration is dropped, or a neighbour
                # sharing the address could end this unit's retract.
                if _dbg_unit is not None:
                    _tn = _TRAY_NOW_RE.findall(raw)
                    if _tn:
                        _u = int(_dbg_unit)
                        with self._lock:
                            _prev = self._tray_now_by_unit.get(_u)
                            # Walk every [AMS_COMMON] segment in order; the
                            # edge can fall between two.
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
                    # Judged before taking the lock: it reads tube_len(), which
                    # takes the same non-reentrant lock, so doing this inside
                    # the with-block would deadlock the reader thread.
                    ok = self._finish_succeeded(raw, low, obj.get("addr"))
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = ok
                        self._finish_text = raw
                    self._tray_gone = False
                elif _ODOM_RESET_RE.search(raw):
                    # A tray engaged and its odometer zeroed: the feed arrived.
                    # The only completion an [AMS_DEV] unit gives; it also
                    # re-arms the tray-gone edge.
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = True
                        self._finish_text = raw
                    self._tray_gone = False
                elif _ODOM_NO_TRAY_RE.search(raw) and not self._tray_gone:
                    # The odometer has no tray, so a retract completed.
                    # Edge-triggered: the unit repeats it at ~2 Hz, and only an
                    # odom reset re-arms it.
                    self._tray_gone = True
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = True
                        self._finish_text = raw
                # Say the useful ones out loud, in English.
                try:
                    # addr identifies which AMS class narrated (0x0700 boxed
                    # AMS, 0x1800 HT); may be absent.
                    self._narrate_human(raw, now, obj.get("addr"), _dbg_unit)
                except Exception:
                    pass          # a nicety must never break the reader thread
                # Motor current, when the AMS reports it. It clamps at its
                # limiter (~1.6A) against a ~0.07A typical draw, so this is a
                # threshold signal rather than a proportional one.
                mi = _BLDC_I_RE.search(raw)
                if mi:
                    try:
                        with self._lock:
                            self._bldc_i = float(mi.group(1))
                    except ValueError:
                        pass
                # An explicit stall: "feed finish -1, stall" / "pull err, bdc
                # stall" (AMS 2 Pro) or "TIMEOUT error N" (HT). Not assist_err
                # or err_code, which cycle normally, and not the
                # self-recovering "rocker stall, tray_cnt:..." family.
                if "rocker stall" in low and "tray_cnt" in low:
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
                # err_code's current level, which a resume guard needs ("is
                # this unit in error now"); an edge is not a fault:
                #   [AMS_LINK]err_code: 0 -> 23      HT declaring a stall
                #   [AMS_LINK]err_code: 18 -> 0      HT accepting the clear
                #   [AMS_LINK]err_code:0x00->0x80    AMS 2, hex form
                me = _ERR_CODE_RE.search(raw)
                if me:
                    try:
                        raw = me.group(2)
                        # 0x-prefixed is hex, bare is decimal; both occur.
                        val = int(raw, 16) if raw.lower().startswith("0x") \
                            else int(raw, 10)
                        with self._lock:
                            self._err_code = val
                            self._err_code_t = now
                    except ValueError:
                        pass
                # The operator's console, the only consumer of the deduped
                # text: a run of identical lines re-emits once a minute with a
                # count. Bus chatter stays off the console (see _AMS_NOISE_RE).
                if text:
                    # The console is rate-limited: filling Klipper's gcode pipe
                    # stalls the reactor and takes the MCUs down. AFC.log keeps
                    # everything.
                    _quiet = _ams_is_noise(_DBG_AMSTIME_RE.sub("", text).strip())
                    if not _quiet:
                        _now = time.monotonic()
                        _win = getattr(self, "_narr_win", 0.0)
                        if _now - _win >= 1.0:
                            self._narr_win = _now
                            self._narr_n = 0
                            self._narr_said = False
                        self._narr_n = getattr(self, "_narr_n", 0) + 1
                        if self._narr_n > self.NARRATION_CONSOLE_MAX_PER_S:
                            _quiet = True            # log-only from here
                            if not getattr(self, "_narr_said", False):
                                self._narr_said = True
                                self.logger.info(
                                    "AFC bambu: the AMS is narrating faster "
                                    "than the console can take "
                                    f"(>{self.NARRATION_CONSOLE_MAX_PER_S}/s); "
                                    "the rest of this burst is in AFC.log")
                    self.logger.debug(f"AMS: {text}", only_debug=_quiet)
        elif obj.get("evt") == "mcaddr":
            # The firmware echoes the address it applied, so zero means it did
            # not take and the log drain falls back to 0x0700.
            try:
                with self._lock:
                    self._mcaddr_ack[int(obj.get("unit", 0))] = \
                        int(obj.get("addr", 0))
            except Exception:
                pass
        elif obj.get("evt") not in _BRIDGE_EVENTS_KNOWN:
            # Anything no handler consumes, logged to file so a lost command
            # and a lost reply are distinguishable.
            self.logger.debug(f"AFC bambu: unhandled bridge event {obj}",
                              only_debug=True)
        elif obj.get("evt") == "txecho":
            # Echo lines the firmware dropped because USB was behind; shown on
            # the console so the capture can be re-taken.
            try:
                self._txecho_drops = int(obj.get("drops", 0) or 0)
            except Exception:
                self._txecho_drops = 0
            if self._txecho_drops:
                self.logger.info(
                    f"AFC bambu bridge: TX echo dropped "
                    f"{self._txecho_drops} frame(s) -- the USB link fell "
                    f"behind the bus. Frames MISSING from this capture were "
                    f"still transmitted; do not read a gap as a frame we "
                    f"never sent.")
        elif obj.get("evt") == "loops":
            # Main-loop iterations + a timestamp. Everything the master can do
            # is bounded by this rate: the 21ms drive channel needs 48 passes a
            # second. Straight to the narration file, like tx.
            try:
                self._narrate_to_file(
                    f'{{"evt":"loops","n":{int(obj.get("n", 0))},'
                    f'"us":{int(obj.get("us", 0))}}}', None)
            except Exception:
                pass
        elif obj.get("evt") == "slow":
            # One bus pass held the bridge's main loop and USB went unread:
            # rr_* is time in the reply reader, capped the reads cut short,
            # capscan the blocking scans. AFC.log only.
            try:
                self.logger.debug(
                    f"AFC bambu: bridge loop held {int(obj.get('ms', 0))} ms "
                    f"(bus reads {int(obj.get('rr_ms', 0))} ms over "
                    f"{int(obj.get('rr_n', 0))}, longest "
                    f"{int(obj.get('rr_max_us', 0))} us, capped "
                    f"{int(obj.get('capped', 0))}, capscans "
                    f"{int(obj.get('capscan', 0))})", only_debug=True)
            except Exception:
                pass
        elif obj.get("evt") == "tx":
            # Frames the bridge transmits, in the "sniff" shape so capture
            # tools read them unchanged. Narration file only.
            try:
                # dir distinguishes what the bridge sent from what the AMS
                # sent back.
                _dir = str(obj.get("dir", "tx"))
                self._narrate_to_file(
                    # Rebuilt from a fixed list of keys, not passed through:
                    # a new firmware field must be added here or it is dropped.
                    f'{{"evt":"tx","dir":"{_dir}",'
                    f'"us":{int(obj.get("us", 0))},'
                    f'"n":{int(obj.get("n", 0))},'
                    + (f'"s":"{obj.get("s")}",' if obj.get("s") else "")
                    + f'"hex":"{obj.get("hex", "")}"}}', None)
            except Exception:
                pass
        elif obj.get("evt") == "sniff":
            # Passive bus-sniffer frame, logged verbatim to file (hundreds a
            # second). sq gaps are dropped blobs, ov growth is UART loss, and
            # us is the firmware's microsecond stamp, the only clock fine
            # enough for sub-second silences.
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
                if prev is not None and int(_sq) != prev + 1:
                    self._sniff_lost += max(0, int(_sq) - prev - 1)
            if _ov is not None:
                # ov counts from the Pico's boot, so baseline it per capture.
                if self._sniff_ovr0 is None:
                    self._sniff_ovr0 = int(_ov)
                self._sniff_ovr = int(_ov) - self._sniff_ovr0
            if _rf is not None:
                # A ring lap (reader behind the DMA) is counted apart from an
                # overrun (DMA behind the wire); either means lost bytes.
                if self._sniff_rf0 is None:
                    self._sniff_rf0 = int(_rf)
                self._sniff_rf = int(_rf) - self._sniff_rf0
        elif obj.get("evt") == "sniff_mode":
            # The firmware's ack of {"cmd":"sniff"}. On the console: a
            # listen-only bridge is otherwise indistinguishable from quiet
            # units.
            on = bool(obj.get("on"))
            with self._lock:
                if on:
                    # A capture's integrity is per-capture. Carrying the last
                    # one's counters into this one would answer for the wrong
                    # run.
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
            self.logger.info(
                f"AFC bambu {self.name}: bridge sniff mode "
                f"{mode_str}"
                f"{verdict}")
        elif obj.get("evt") == "info":
            # The bridge's own identity. `chip` is the one field the host
            # cannot work out for itself and must not guess: it decides
            # whether a firmware image is even loadable on this board.
            with self._lock:
                self._info = dict(obj)
            self.logger.info(
                f"AFC bambu: info REPLY chip={obj.get('chip')} "
                f"fw={obj.get('fw')} up={obj.get('up')}")
            # Reboot or link hiccup: uptime going backwards is a reboot (every
            # AMS is left parked); a WiFi blip leaves them working. The first
            # reading is only a baseline. Must live here: `chain` has "fw" but
            # not "up".
            try:
                up = obj.get("up")
                up = None if up is None else int(up)
            except Exception:
                up = None
            if up is not None:
                prev = self._fw_up_ms
                if prev is not None and up < prev:
                    self._fw_rebooted = True
                    self.logger.info(
                        f"AFC bambu: bridge REBOOTED (uptime {prev} -> {up} "
                        f"ms); units on the wire lost their master")
                self._fw_up_ms = up
        elif obj.get("evt") == "wifi":
            # A BB_WIFI board reporting its radio, logged when it changes:
            # tells a wrong password from a board that never booted.
            try:
                self.logger.info(
                    f"AFC bambu {self.name}: pico wifi link={obj.get('link')} "
                    f"ip={obj.get('ip')} assoc={obj.get('assoc')} "
                    f"tries={obj.get('tries')}")
            except Exception:
                pass
        elif obj.get("evt") == "cali":
            # The bridge's ack of a calibrate, not its outcome: echoed whether
            # it ran, was deferred or ignored. On an AMS 1 it follows the 10-15
            # s blocking burst. File only; extra fields are carried through
            # uninterpreted.
            try:
                _extra = " ".join(f"{k}={v}" for k, v in obj.items()
                                  if k not in ("evt", "unit", "slot"))
                self.logger.debug(
                    f"AFC bambu: bridge answered cali "
                    f"(unit {obj.get('unit')}, slot {obj.get('slot')})"
                    + (f" {_extra}" if _extra else ""),
                    only_debug=True)
            except Exception:
                pass
        elif obj.get("evt") == "chain":
            # Enrollment map: comma-separated 24-hex UIDs by polling index.
            # Keep empty fields, or later units shift index.
            raw = (obj.get("uids") or "").strip().upper()
            uids = [] if not raw else [u.strip() for u in raw.split(",")]
            with self._lock:
                self._chain_seq = getattr(self, "_chain_seq", 0) + 1
                self._chain_uids = uids
                # Diagnostics riding on the chain reply: which indices the
                # firmware has HT-flagged, and the exact build running on the
                # Pico (see chain_diag for why `info` is preferred for fw).
                try:
                    self._chain_htmask = int(obj.get("htmask") or 0)
                except Exception:
                    self._chain_htmask = 0
                # Which indices may measure, as the firmware holds it. None
                # when absent: "none measures" and "cannot tell" differ.
                try:
                    cm = obj.get("capmask")
                    self._chain_capmask = None if cm is None else int(cm)
                except Exception:
                    self._chain_capmask = None
                self._chain_fw = str(obj.get("fw") or "")
                # 0x3702 dialect verdicts: a2mask = units that ever answered
                # the version query (ams2); a2asks = queries per index, so many
                # unanswered means ams1.
                try:
                    self._chain_a2mask = int(obj.get("a2mask") or 0)
                    self._chain_a2asks = [
                        int(x) for x in
                        str(obj.get("a2asks") or "").split(",") if x]
                except Exception:
                    self._chain_a2mask = 0
                    self._chain_a2asks = []
                # Announce-reply tag byte per discovered unit, in chain order.
                # Diagnostic only: printed with the UID list, nothing acts on
                # it.
                self._chain_tags = str(obj.get("tags") or "")
                try:
                    self._chain_capn = int(obj.get("capn") or 0)
                    self._chain_capdiag = int(obj.get("capdiag") or 0)
                    # Why the last capacity window did or did not open.
                    # None (field absent) stays distinct from 0 (no window
                    # requested since boot).
                    _cw = obj.get("capwhy")
                    self._chain_capwhy = None if _cw is None else int(_cw)
                    # ...and the bay that verdict was about.
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
                # Per-unit MC addressing as the firmware holds it. None
                # (field absent, unknown) is kept distinct from [] (known-empty).
                mc = obj.get("mcaddr")
                self._chain_mcaddr = (
                    [int(x) for x in mc] if isinstance(mc, list) else None)
                try:
                    self._chain_sel = (int(obj.get("selid", -1)),
                                       int(obj.get("selsent", 0)),
                                       int(obj.get("selack", 0)))
                except Exception:
                    self._chain_sel = (-1, 0, 0)

    def chain_uids(self) -> List[str]:
        """
        The cached chain UID list, index -> UID.

        :return list: the UIDs; empty until known
        """
        with self._lock:
            return list(self._chain_uids)

    def chain_snapshot(self) -> Dict[str, Any]:
        """
        One chain reply's enrollment map and dialect counters, read together.

        The reader thread replaces all of them under the lock when a reply
        lands, so separate getter calls can straddle two replies: uids from
        one and a2mask/a2asks from the next, crediting a unit that just took
        an index with the counters of the one that held it before (or the
        reverse). A caller that judges a UID by its index's counters takes
        this instead.

        :return dict: seq (parsed chain replies so far, 0 before any), uids
            (index -> UID, empties kept), htmask, a2mask, a2asks
        """
        with self._lock:
            return {"seq": getattr(self, "_chain_seq", 0),
                    "uids": list(self._chain_uids),
                    "htmask": getattr(self, "_chain_htmask", 0),
                    "a2mask": getattr(self, "_chain_a2mask", 0),
                    "a2asks": list(getattr(self, "_chain_a2asks", []))}

    def mcaddr_ack(self, unit: int) -> Optional[int]:
        """
        What the firmware read back the last time this unit was told an MC
        address, or None if it has never acknowledged one.

        None and 0 mean different things and both are failures worth telling
        apart: None is "the command never reached the Pico" (announce dropped,
        link not up yet, JSON malformed); 0 is "the Pico got it and the address
        still is not set".

        :param unit: AMS chain index
        :return Optional[int]: the acknowledged address, or None
        """
        with self._lock:
            return self._mcaddr_ack.get(int(unit))

    def chain_mcaddr(self) -> Optional[List[int]]:
        """
        Per-unit MC device addresses as the firmware holds them.

        None means the firmware did not report them, which is
        distinct from an empty/zero list meaning "reported, and nothing is
        set". The distinction matters: an unset address drops the narration
        log drain back to the default 0x0700 pair, which never asks an AMS HT
        at 0x1800.

        :return Optional[List[int]]: addresses by unit index, or None
        """
        with self._lock:
            return getattr(self, "_chain_mcaddr", None)

    def chain_dialect(self) -> tuple:
        """
        Which units have answered the 0x3702 version query, from the last chain reply.

        :return tuple: (a2mask, a2asks): the answered-units mask and how many
                       queries each unit has seen; (0, []) until known
        """
        with self._lock:
            return (getattr(self, "_chain_a2mask", 0),
                    list(getattr(self, "_chain_a2asks", [])))

    def chain_diag(self) -> tuple:
        """
        Chain diagnostics from the last chain reply.

        `fw` prefers the `info` reply. Both events carry a version, but they are
        refreshed on different clocks: `info` is asked on every connection, while
        `chain` only arrives once the chain answers again, after a reboot has
        dropped the link, re-enrolled the units and polled them. In that gap the
        cached `_chain_fw` is the previous build, which would misreport the version
        right after a flash.

        :return tuple: (htmask, fw, (selid, selsent, selack)); (0, '', (-1, 0, 0))
                       until known
        """
        with self._lock:
            fw = str((getattr(self, "_info", None) or {}).get("fw") or "")
            return (getattr(self, "_chain_htmask", 0),
                    fw or getattr(self, "_chain_fw", ""),
                    getattr(self, "_chain_sel", (-1, 0, 0)))

    #: Silence before a bridge is called out, and the repeat interval.
    #: Generous, since a firmware transfer or a long AMS cycle can be quiet.
    QUIET_WARN_S = 45.0
    QUIET_REPEAT_S = 300.0
    #: Silence this long on an open link means the far end is gone: drop it and
    #: reconnect (see _check_quiet).
    QUIET_DROP_S = 30.0

    def _check_quiet(self) -> None:
        """
        Say so when the bridge has gone silent, or has stayed unreachable.

        The reconnect loop logs only on success, so without this a bridge that
        never came back would be indistinguishable from a quiet one. Fires
        once when silence passes QUIET_WARN_S and then every QUIET_REPEAT_S,
        so a long outage leaves a trail without filling the log.

        Called from the reader on every idle pass, which is the one place that
        is running whether or not anything is arriving.
        """
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
        if last is not None and (now - last) < self.QUIET_REPEAT_S:
            return
        self._silence_logged_t = now
        self.logger.warning(
            f"AFC bambu: bridge has been {what} for {quiet_for:.0f}s"
            f"{'; commands are being dropped' if self._serial is None else ''}")

    def _mark_connected(self) -> None:
        """
        Stamp a freshly opened link and decide whether it gets a grace.

        _last_frame_t is deliberately not touched here: an opened socket is
        not the bridge having spoken. The grace is granted only when the
        previous connection actually delivered a frame, so a bridge that keeps
        accepting and going mute cannot renew its allowance on every
        reconnect.
        """
        self._connected_t = time.monotonic()
        self._grace_this_conn = self._spoke_since_connect
        self._spoke_since_connect = False
        # Ask the bridge's identity on every connection, including the first;
        # the wrong-chip flash guard needs it. Asked on the first status frame,
        # after auth.
        self._info_asked = False

    def _drop_if_silent(self) -> bool:
        """
        Force a reconnect when an open link has gone quiet. :return: dropped

        read() returns b"" on a timeout, because a quiet link is not an error,
        and raises at end of stream, which makes the reader drop and
        reconnect. That covers a clean disconnect, but not a bridge that
        vanishes silently (BOOTSEL, a power cut, an AP drop): with no FIN or
        RST, read keeps timing out and the reader would sit on a dead socket
        indefinitely.

        SO_KEEPALIVE (tuned to ~35s) is the socket-level backstop, but it is
        Linux-specific and only applies to the TCP transport. This is the
        transport-agnostic one: the firmware streams status continuously, so
        QUIET_DROP_S of nothing on an open link means the far end is gone.

        Skipped during a firmware transfer: the board is legitimately silent
        while it receives an image, and dropping the link mid-flash would
        abort it.
        """
        if self._serial is None or self._fw_raw:
            return False
        # Measured from the later of last frame and link open, or a reconnect
        # after an outage would drop itself on its first tick.
        since = self._last_frame_t
        if self._connected_t is not None:
            since = self._connected_t if since is None else max(since, self._connected_t)
        if since is None or (time.monotonic() - since) < self.QUIET_DROP_S:
            return False
        self.logger.warning(
            f"AFC bambu: bridge silent for {self.QUIET_DROP_S:.0f}s on an open "
            f"link -- dropping it to force a reconnect")
        self._drop_port()
        return True

    def is_connected(self) -> bool:
        """
        Whether the link to the bridge is up right now.

        :return bool: True while a port is open
        """
        return self._serial is not None

    def silent_for(self) -> Optional[float]:
        """
        How long since anything arrived from the bridge, in seconds.

        down_since() only describes a closed port. A bridge that vanished
        without closing anything (BOOTSEL, a power cut, an AP drop) leaves
        is_connected() true and down_since() None while nothing gets through.
        This covers both: the firmware streams status many times a second, so
        any meaningful silence means the far end is gone regardless of what
        the socket believes.

        :return float: seconds since the last frame, or None if never connected
        """
        if self._last_frame_t is None:
            return None
        now = time.monotonic()
        quiet = now - self._last_frame_t
        # A new link gets CONNECT_GRACE_S of discount; the count underneath
        # keeps running.
        if (self._grace_this_conn
            and self._connected_t is not None
            and (now - self._connected_t) < CONNECT_GRACE_S):
            return min(quiet, now - self._connected_t)
        return quiet

    def _note_frame_gap(self, now: float) -> None:
        """
        Say how long an open link was quiet, once, when a frame ends the gap.

        The reader calls this for every chunk, before it moves _last_frame_t,
        so the previous stamp is still there to measure against. A gap only
        counts between two frames of the same connection: the first frame
        after a reconnect follows an outage, which the reconnect line already
        reports, and the timed-out writes counted before it are written off
        with it.

        File only (only_debug): a WiFi bridge has gaps of its own, and this is
        a line for reading a capscan back, not for the console.

        :param now: monotonic time the new frame arrived
        """
        total = self._write_timeouts
        n = total - self._gap_timeouts_seen
        self._gap_timeouts_seen = total
        prev = self._last_frame_t
        if (prev is None
            or (self._connected_t is not None
                and prev < self._connected_t)):
            return
        gap = now - prev
        if gap <= SILENCE_LOG_S:
            return
        self.logger.debug(
            f"AFC bambu: bridge was silent {gap:.1f} s; {n} write(s) timed "
            f"out meanwhile", only_debug=True)

    def down_since(self) -> Optional[float]:
        """
        When the link went away, or None if it is up.

        A moment, not a duration, so a caller can ask "has it been down long
        enough to act on" without the bridge having to guess what long enough
        means for them. A load wants seconds; an operator warning wants longer.

        :return float: monotonic time the port was dropped, else None
        """
        return None if self._serial is not None else self._down_t

    def _drop_port(self) -> None:
        """
        Close the current serial and mark it gone so the reader reconnects.
        """
        s = self._serial
        self._serial = None
        # No link, no connection stamp: silence is measured plainly while down.
        self._connected_t = None
        # Stamp the outage on the transition; _down_epoch counts outages so
        # send() logs one dropped command per outage.
        if s is not None or self._down_t is None:
            self._down_t = time.monotonic()
            self._down_epoch += 1
        # Drop anything queued for the dead link: the Pico has reset and the
        # units re-push their config on reconnect.
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
        """
        Reader-thread loop: reconnect, split lines, dispatch each frame.
        """
        try:
            thread_name = threading.current_thread().name
            chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
        except Exception:
            pass
        buf = b""
        backoff = 0.5
        while self._run:
            # Reconnect if the port is gone (first-open failure, a read/write
            # error, or a Pico re-plug). Back off so the loop doesn't spin while it's
            # absent, and reset the backoff once reading again.
            if self._serial is None:
                try:
                    self._serial = self._serial_factory()
                    down_for = (time.monotonic() - self._down_t
                                if self._down_t is not None else 0.0)
                    self._down_t = None
                    self._silence_logged_t = None
                    self._mark_connected()
                    self.logger.info(
                        f"AFC bambu: bridge reconnected"
                        f"{f' after {down_for:.0f}s down' if down_for >= 1 else ''}")
                    buf = b""
                    backoff = 0.5
                    # The firmware likely just booted (reflash/power-cycle): its
                    # unit count and HT flags are factory-fresh. Let each unit
                    # re-push its config (on the reactor, not this thread).
                    for cb in self._reconnect_cbs:
                        try:
                            self.reactor.register_async_callback(
                                lambda et, c=cb: c())
                        except Exception:
                            pass
                except Exception:
                    # The watchdog runs here too, not only on the read path: a disconnected
                    # reader never reaches the read, so a link that stays down would otherwise
                    # never be reported.
                    self._check_quiet()
                    time.sleep(min(backoff, 5.0))
                    backoff = min(backoff * 2, 5.0)
                continue
            try:
                chunk = self._serial.read(64)
            except Exception as e:
                # Never die on a read error: drop the port and reconnect so a
                # transient glitch self-heals.
                if self._expect_reset:
                    # A requested reset drops the USB CDC endpoint, so the read
                    # failing is the command working, not a fault; logged at
                    # info rather than as a warning.
                    self._expect_reset = False
                    self.logger.info(
                        "AFC bambu: bridge resetting as asked; reconnecting")
                else:
                    self.logger.warning(
                        f"AFC bambu: bridge read failed: {e}; reconnecting")
                self._drop_port()
                continue
            if not chunk:
                self._check_quiet()
                # A read timeout on an open link is normal; a long run of them
                # is not. This turns a silently vanished bridge into a
                # reconnect.
                self._drop_if_silent()
                continue
            _now = time.monotonic()
            self._note_frame_gap(_now)           # reads the previous stamp
            self._last_frame_t = _now
            self._spoke_since_connect = True     # earns the next grace
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.handle_line(raw.decode(errors="replace"))
