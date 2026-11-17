# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""
Bambu AMS firmware: each unit's firmware version and board, read off the bus when it comes online,
and the Flash AMS menu, which picks a unit and one of Bambu's images for it and runs the AMS
updater, lending it the unit's bridge link.
"""
from __future__ import annotations

import json
import os
import re
import shlex
import time
import zipfile
from typing import Any, Dict, List, Optional, Tuple

from extras import AFC_BambuAMS_bridge as _bridge_mod
from extras.AFC_respond import AFCprompt

#: The AMS updater's shell command, and the files beside the updater.
SHELL_SECTION = "gcode_shell_command ams_flash"
UPDATER = "ams_flash.py"
CATALOG_NAME = "ams_firmware.json"
RESULT_NAME = "ams_flash_result.json"
STATE_NAME = "ams_flash_state.json"
OBJECT_NAME = "AFC_BambuAMS_firmware"

#: The printer's own bus address, and the two device addresses units answer on.
HOST = 0x0900
BOXED = 0x0700
HT = 0x1800
CRC8_POLY = 0x39
CRC8_INIT = 0x66
CRC16_POLY = 0x1021
CRC16_INIT = 0x913D

#: Bambu's image name prefix -> the model's name.
FAMILIES = {"ams": "AMS 1", "n3f": "AMS 2 Pro", "n3s": "AMS HT"}
#: The board each model has shipped on, for a unit whose model code was not read. An AMS 1 has two.
DEFAULT_BOARD = {"ams2": "n3f_rev5", "ht": "n3s_rev5"}
IMAGE_NAME = re.compile(r"((ams|n3f|n3s)_rev(\d+))-firmware-v(\d\d\.\d\d\.\d\d\.\d\d)-")
VERSION = re.compile(r"\d\d\.\d\d\.\d\d\.\d\d")
FILE_NAME = re.compile(r"[\w.\-]+")

TICK_S = 2.0
ANSWER_S = 0.5
READ_TRIES = 3
#: The longest the updater may hold a bridge link.
HOLD_MAX_S = 3600
#: How long the images found for a board are kept before the updater's folder is read again.
CHOICES_S = 30.0
CLOSE = "RESPOND TYPE=command MSG=action:prompt_end"


def crc8(data: bytes) -> int:
    """
    A bus frame header's CRC-8.

    :param data: the header's first six bytes
    :return int: its CRC
    """
    c = CRC8_INIT
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ CRC8_POLY) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def crc16(data: bytes) -> int:
    """
    A bus frame's CRC-16.

    :param data: the frame up to its CRC
    :return int: its CRC
    """
    c = CRC16_INIT
    for b in data:
        c = (c ^ (b << 8)) & 0xFFFF
        for _ in range(8):
            c = ((c << 1) ^ CRC16_POLY) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def version_query(unit_id: int, counter: int = 0) -> str:
    """
    The printer's firmware version query (op 03 01) to one unit.

    :param unit_id: the unit's bus id: 0-3 boxed, 0x80 and up an AMS HT
    :param counter: the frame counter
    :return str: the whole frame in hex, as the bridge's raw command takes it
    """
    dev = HT if unit_id & 0x80 else BOXED
    head = bytes((0x3D, 0x05, counter & 0xFF, (counter >> 8) & 0xFF, 16, 0))
    frame = head + bytes((crc8(head),)) + dev.to_bytes(2, "little")
    frame += HOST.to_bytes(2, "little") + bytes((0x03, 0x01, unit_id & 0xFF))
    return (frame + crc16(frame).to_bytes(2, "little")).hex()


def bus_frames(raw: bytes) -> List[bytes]:
    """
    Every whole, CRC-valid long frame in a bus capture, in order.

    :param raw: the captured bytes
    :return list: the frames
    """
    out: List[bytes] = []
    i = 0
    while i + 7 <= len(raw):
        if (raw[i] == 0x3D
            and crc8(raw[i:i + 6]) == raw[i + 6]):
            end = i + (raw[i + 4] | (raw[i + 5] << 8))
            if (end >= i + 9
                and end <= len(raw)
                and crc16(raw[i:end - 2]) == int.from_bytes(raw[end - 2:end], "little")):
                out.append(raw[i:end])
                i = end
                continue
        i += 1
    return out


def version_answer(rx_hex: str, unit_id: int) -> Optional[Tuple[str, str]]:
    """
    A unit's answer to the version query in a bridge capture. It carries the version lowest byte
    first (1d 15 00 03 is 03.00.21.29), the model code (N3F05, N3S05, AMS07 or AMS08) and, at byte
    33, the id of the unit answering.

    :param rx_hex: the capture, in hex
    :param unit_id: the unit asked
    :return tuple: (version, model code), or None when it did not answer
    """
    try:
        raw = bytes.fromhex(rx_hex)
    except ValueError:
        return None
    dev = HT if unit_id & 0x80 else BOXED
    for f in bus_frames(raw):
        if (len(f) >= 36
            and f[11:13] == b"\x03\x01"
            and int.from_bytes(f[9:11], "little") == dev
            and f[33] == unit_id):
            code = f[17:33].split(b"\0", 1)[0].decode("ascii", "replace")
            return ".".join(f"{b:02d}" for b in reversed(f[13:17])), code
    return None


def board_of_code(code: Optional[str]) -> str:
    """
    The image board a unit's model code takes: N3F05 is n3f_rev5, AMS08 is ams_rev8.

    :param code: the model code from the unit's version answer
    :return str: the board, or "" for a code of none of the three models
    """
    m = re.fullmatch(r"(AMS|N3F|N3S)(\d\d)", code or "")
    if m is None:
        return ""
    return f"{m.group(1).lower()}_rev{int(m.group(2))}"


def board_name(board: str) -> str:
    """
    A board's model, for people.

    :param board: an image board such as n3f_rev5
    :return str: AMS 1, AMS 2 Pro or AMS HT; the board itself when it is none of them
    """
    return FAMILIES.get(board.split("_", 1)[0], board)


def version_key(version: Optional[str]) -> Tuple[int, ...]:
    """
    A version as numbers, so versions sort in release order.

    :param version: such as 05.00.22.22
    :return tuple: its numbers; () for no version
    """
    if not version:
        return ()
    return tuple(int(p) for p in version.split("."))


def updater_dir(printer: Any) -> Optional[str]:
    """
    The folder the AMS updater runs from, read off its shell command.

    :param printer: Klipper's printer object
    :return str: the folder, or None when the updater is not set up
    """
    shell = printer.lookup_object(SHELL_SECTION, None)
    cmd = getattr(shell, "command", None) or []
    try:
        parts = shlex.split(cmd) if isinstance(cmd, str) else cmd
    except ValueError:
        return None
    for part in parts:
        if os.path.basename(part) == UPDATER:
            return os.path.dirname(os.path.expanduser(part)) or None
    return None


def load_catalog(folder: str) -> List[Dict[str, Any]]:
    """
    The images the catalog beside the updater lists.

    :param folder: the updater's folder
    :return list: entries with a board, version, file and url; [] when it is missing or unreadable
    """
    try:
        with open(os.path.join(folder, CATALOG_NAME)) as f:
            images = json.load(f).get("images")
    except (OSError, ValueError, AttributeError):
        return []
    if not isinstance(images, list):
        return []
    return [e for e in images
            if isinstance(e, dict)
            and all(e.get(k) for k in ("board", "file", "url"))
            and VERSION.fullmatch(str(e.get("version")))]


def local_images(folder: str) -> List[Dict[str, str]]:
    """
    The images in files beside the updater: Bambu's .bin.sig files and offline update packages.

    :param folder: the updater's folder
    :return list: a board, version and file name for each image found
    """
    found: List[Dict[str, str]] = []
    try:
        names = sorted(os.listdir(folder))
    except OSError:
        return found
    for name in names:
        members = [name]
        if name.endswith(".zip"):
            try:
                with zipfile.ZipFile(os.path.join(folder, name)) as z:
                    members = [os.path.basename(n) for n in z.namelist()]
            except (OSError, zipfile.BadZipFile):
                continue
        elif not name.endswith(".bin.sig"):
            continue
        for member in members:
            m = IMAGE_NAME.match(member)
            if m is not None:
                found.append({"board": m.group(1), "version": m.group(4), "file": name})
    return found


def choices_for(folder: str, board: str) -> List[Dict[str, Any]]:
    """
    The images a board can take, newest first: the catalog's, and any in files beside the updater,
    which are used in place of a download.

    :param folder: the updater's folder
    :param board: the unit's board
    :return list: a version, the file holding it ("" to download it) and whether it is the newest
    """
    listed = {e["version"]: "" for e in load_catalog(folder) if e["board"] == board}
    for img in local_images(folder):
        if img["board"] == board:
            listed[img["version"]] = img["file"]
    ordered = sorted(listed, key=version_key, reverse=True)
    return [{"version": v, "file": listed[v], "newest": i == 0} for i, v in enumerate(ordered)]


def read_json(path: str) -> Dict[str, Any]:
    """
    A small JSON record the updater left.

    :param path: the file
    :return dict: the record; {} when it is missing or unreadable
    """
    try:
        with open(path) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return {}
    return d if isinstance(d, dict) else {}


class AmsFirmware:
    """
    One per printer: reads each Bambu AMS unit's firmware when it comes online, offers an update the
    first time a unit is seen, and serves the Flash AMS menu.
    """

    def __init__(self, printer: Any) -> None:
        """
        :param printer: Klipper's printer object
        """
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.afc = printer.lookup_object("AFC")
        self.logger = self.afc.logger
        # Unit name -> the (uid, bus id) its version was read for.
        self._read: Dict[str, Tuple[Optional[str], int]] = {}
        # The read waiting on an answer: (unit, bridge, bus id, tries).
        self._asking: Optional[Tuple[Any, Any, int, int]] = None
        self._timer: Any = None
        # Board -> (when it was read, the images it can take), for get_status.
        self._choices: Dict[str, Tuple[float, List[Dict[str, Any]]]] = {}
        # When this Klipper run began: get_status leaves out an update that ended before it.
        self._started = time.time()
        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.lookup_object("gcode").register_command(
            "AFC_BAMBU_FIRMWARE", self.cmd_AFC_BAMBU_FIRMWARE,
            desc="Flash AMS: pick a Bambu AMS unit and one of Bambu's firmware images for it, "
                 "and the AMS updater writes it while Klipper keeps running. "
                 "AFC_BAMBU_FIRMWARE [UNIT=<unit> [VERSION=<version> [FILE=<file>] [CONFIRM=1]]]")

    def _handle_ready(self) -> None:
        """
        Start reading versions, when the updater is set up to use them.
        """
        if updater_dir(self.printer) is not None:
            self._timer = self.reactor.register_timer(self._tick, self.reactor.NOW)

    def _units(self) -> List[Any]:
        """
        Every Bambu AMS unit, in config order.

        :return list: the AFC_BambuAMS objects
        """
        return [unit for _name, unit in self.printer.lookup_objects("AFC_BambuAMS")]

    def _printing(self) -> bool:
        """
        Whether a print is running or paused.

        :return bool: True while one is
        """
        stats = self.printer.lookup_object("print_stats", None)
        if stats is None:
            return False
        state = stats.get_status(self.reactor.monotonic()).get("state")
        return state in ("printing", "paused")

    def _tick(self, eventtime: float) -> float:
        """
        Read one unit's firmware at a time: ask on one tick, take the answer on the next.

        :param eventtime: reactor time
        :return float: when to run next
        """
        asking = self._asking
        if asking is not None:
            self._asking = None
            self._take_answer(*asking)
        elif not self._printing():
            pick = self._next_unread()
            if pick is not None:
                self._ask(*pick, 1)
        return eventtime + (ANSWER_S if self._asking is not None else TICK_S)

    def _next_unread(self) -> Optional[Tuple[Any, Any, int]]:
        """
        The next unit whose version is still to read; one that cannot be asked now forgets its
        last read, so it is read again when it can be.

        :return tuple: (unit, bridge, bus id), or None
        """
        for unit in self._units():
            target = unit.firmware_target()
            if target is None:
                self._read.pop(unit.name, None)
                continue
            bridge, bus_id = target
            if self._read.get(unit.name) != (unit.unit_uid, bus_id):
                return unit, bridge, bus_id
        return None

    def _ask(self, unit: Any, bridge: Any, bus_id: int, tries: int) -> None:
        """
        Send one unit the version query.

        :param unit: the unit
        :param bridge: its bridge
        :param bus_id: its bus id
        :param tries: which try this is
        """
        with bridge._lock:
            bridge._last_raw_reply = ""
        bridge.send({"cmd": "raw", "hex": version_query(bus_id, tries), "us": 100000})
        self._asking = (unit, bridge, bus_id, tries)

    def _take_answer(self, unit: Any, bridge: Any, bus_id: int, tries: int) -> None:
        """
        Take a unit's answer to the version query, or ask again.

        :param unit: the unit asked
        :param bridge: its bridge
        :param bus_id: its bus id
        :param tries: how many times it was asked
        """
        with bridge._lock:
            rx = bridge._last_raw_reply
        got = version_answer(rx, bus_id)
        if (got is None
            and tries < READ_TRIES):
            self._ask(unit, bridge, bus_id, tries + 1)
            return
        self._read[unit.name] = (unit.unit_uid, bus_id)
        if got is None:
            self.logger.debug(f"{unit.name}: no answer to the firmware version query")
            return
        unit.fw_version, unit.fw_model = got
        model = board_name(board_of_code(got[1])) or "unknown model"
        self.logger.debug(f"{unit.name}: {model} ({got[1]}) firmware {got[0]}")
        self._offer(unit)

    def _offer(self, unit: Any) -> None:
        """
        Tell the unit's chain what the unit was read to run: the chain offers the newest image the
        first time it sees the unit on an older one.

        :param unit: a unit whose version was just read
        """
        master = getattr(unit, "_master", None)
        folder = updater_dir(self.printer)
        board = board_of_code(unit.fw_model)
        if (master is None
            or folder is None
            or not board):
            return
        choices = choices_for(folder, board)
        newer = None
        if (choices
            and version_key(choices[0]["version"]) > version_key(unit.fw_version)):
            newer = choices[0]
        master.offer_firmware(unit, newer)

    def _listed(self) -> List[Any]:
        """
        The units the Flash AMS menu lists: a pool's spare sections hold no unit until one is
        claimed, and an AMS Lite takes no image.

        :return list: the units, in config order
        """
        return [u for u in self._units()
                if u.ams_model != "lite"
                and (u.unit_uid is not None
                     or not u.pool)]

    def _unit(self, name: str) -> Optional[Any]:
        """
        A unit by name, any case.

        :param name: the unit's name
        :return Any: the unit, or None
        """
        return next((u for u in self._units() if u.name.lower() == name.lower()), None)

    def _board(self, unit: Any) -> str:
        """
        The image board a unit takes: from its model code, else the one its model shipped on.

        :param unit: the unit
        :return str: the board, or "" when it is not known
        """
        return board_of_code(unit.fw_model) or DEFAULT_BOARD.get(unit.ams_model, "")

    def cmd_AFC_BAMBU_FIRMWARE(self, gcmd: Any) -> None:
        """
        Flash AMS: pick a unit, then one of Bambu's images for it, then confirm, and the AMS updater
        writes it while Klipper keeps running. The updater borrows the unit's bridge link with HOLD
        and shows how the update ended with SHOW=RESULT.

        Usage
        -------
        `AFC_BAMBU_FIRMWARE [UNIT=<unit> [VERSION=<version> [FILE=<file>] [CONFIRM=1]]]`

        Example
        -------
        ```
        AFC_BAMBU_FIRMWARE UNIT=Bambu_AMS_1 VERSION=05.00.22.22
        ```
        """
        folder = updater_dir(self.printer)
        if gcmd.get("SHOW", "").upper() == "RESULT":
            self._show_result(gcmd, folder)
            return
        name = gcmd.get("UNIT", None)
        if name is None:
            self._show_units(gcmd, folder)
            return
        unit = self._unit(name)
        if unit is None:
            error_str = f"AFC_BAMBU_FIRMWARE: there is no Bambu AMS unit {name}"
            raise gcmd.error(error_str)
        hold = gcmd.get_int("HOLD", None, minval=0, maxval=HOLD_MAX_S)
        if hold is not None:
            self._hold(gcmd, unit, hold)
            return
        if folder is None:
            error_str = ("AFC_BAMBU_FIRMWARE: the AMS updater is not set up; "
                         "AMS_FLASH_HOWTO.md says how")
            raise gcmd.error(error_str)
        if gcmd.get_int("RESUME", 0):
            self._run(gcmd, folder, unit, None, None)
            return
        version = gcmd.get("VERSION", None)
        if version is None:
            self._show_versions(gcmd, folder, unit)
            return
        file = gcmd.get("FILE", None)
        bad_file = (file is not None
                    and not FILE_NAME.fullmatch(file))
        if (not VERSION.fullmatch(version)
            or bad_file):
            error_str = ("AFC_BAMBU_FIRMWARE: VERSION is like 05.00.22.22 and FILE is a file "
                         "name beside the updater")
            raise gcmd.error(error_str)
        if gcmd.get_int("CONFIRM", 0):
            self._run(gcmd, folder, unit, version, file)
            return
        self._show_confirm(gcmd, unit, version, file)

    def _hold(self, gcmd: Any, unit: Any, seconds: int) -> None:
        """
        Let go of a unit's bridge link for the AMS updater, or take it back.

        :param gcmd: the command
        :param unit: the unit being updated
        :param seconds: how long to let go of it at most; 0 takes it back
        """
        bridge = _bridge_mod._BRIDGES.get(unit.serial_port)
        if bridge is None:
            error_str = f"AFC_BAMBU_FIRMWARE: {unit.name} has no bridge"
            raise gcmd.error(error_str)
        if (seconds
            and self._printing()):
            error_str = "AFC_BAMBU_FIRMWARE: a print is running, so the bridge link stays"
            raise gcmd.error(error_str)
        bridge.hold(seconds)
        if seconds:
            gcmd.respond_info(f"{unit.name}: the AMS updater has its bridge link; the units on "
                              f"it read offline until it is done")
        else:
            gcmd.respond_info(f"{unit.name}: Klipper has its bridge link back")

    def _status_line(self, folder: Optional[str], unit: Any) -> str:
        """
        One unit's line in the unit list.

        :param folder: the updater's folder
        :param unit: the unit
        :return str: its model, firmware and whether a newer one is out
        """
        if unit.fw_version is None:
            return f"{unit.name}: firmware not read yet; it is read while the unit is online"
        board = self._board(unit)
        choices = choices_for(folder, board) if folder is not None else []
        note = ""
        if (choices
            and version_key(choices[0]["version"]) > version_key(unit.fw_version)):
            note = f", {choices[0]['version']} is out"
        elif choices:
            note = ", the newest"
        model = board_name(board) if board else "AMS"
        return f"{unit.name}: {model}, firmware {unit.fw_version}{note}"

    def _show_units(self, gcmd: Any, folder: Optional[str]) -> None:
        """
        The first page: every unit, with a button for each.

        :param gcmd: the command
        :param folder: the updater's folder
        """
        prompt = AFCprompt(gcmd, self.logger)
        units = self._listed()
        lines = [self._status_line(folder, u) for u in units] or ["No Bambu AMS units."]
        buttons = [(u.name, f"AFC_BAMBU_FIRMWARE UNIT={u.name}", "primary") for u in units]
        if folder is None:
            lines.append("Set up the AMS updater to flash units; AMS_FLASH_HOWTO.md says how.")
            buttons = []
        else:
            stuck = read_json(os.path.join(folder, STATE_NAME)).get("unit")
            waiting = self._unit(stuck) if isinstance(stuck, str) else None
            # A running update keeps the same record of its own unit.
            if self._busy(os.path.join(folder, RESULT_NAME)):
                lines.append("An AMS update is running; let it finish.")
            elif waiting is not None:
                lines.append(f"{waiting.name} waits in its bootloader from an update that "
                             f"stopped.")
                buttons.insert(0, (f"Finish {waiting.name}",
                                   f"AFC_BAMBU_FIRMWARE UNIT={waiting.name} RESUME=1", "error"))
        prompt.create_custom_p("Flash AMS", "\n".join(lines), buttons, True, None)

    def _show_versions(self, gcmd: Any, folder: str, unit: Any) -> None:
        """
        The second page: the images a unit can take, newest first.

        :param gcmd: the command
        :param folder: the updater's folder
        :param unit: the unit
        """
        prompt = AFCprompt(gcmd, self.logger)
        board = self._board(unit)
        have = unit.fw_version
        lines = [f"{unit.name}: {board_name(board) if board else 'AMS'}, "
                 f"firmware {have or 'not read yet'}."]
        buttons = []
        for c in choices_for(folder, board) if board else []:
            label = c["version"]
            if c["newest"]:
                label += " (newest)"
            if c["version"] == have:
                label += " (installed)"
            cmd = f"AFC_BAMBU_FIRMWARE UNIT={unit.name} VERSION={c['version']}"
            if c["file"]:
                cmd += f" FILE={c['file']}"
            buttons.append((label, cmd, "primary" if c["newest"] else "secondary"))
        if not board:
            lines.append("Its board is read with its version, while it is online.")
        elif buttons:
            lines.append("Pick the firmware to write.")
        else:
            lines.append(f"No image for {board}: put Bambu's file beside the updater.")
        back = [("Back", "AFC_BAMBU_FIRMWARE", "info")]
        prompt.create_custom_p(f"Flash {unit.name}", "\n".join(lines), buttons, True, None, back)

    def _show_confirm(self, gcmd: Any, unit: Any, version: str, file: Optional[str]) -> None:
        """
        The last page: what the update does, and the button that starts it.

        :param gcmd: the command
        :param unit: the unit
        :param version: the version picked
        :param file: the file holding it, or None to download it
        """
        prompt = AFCprompt(gcmd, self.logger)
        source = f"from {file}" if file else "downloaded from Bambu"
        text = (f"{unit.name} runs {unit.fw_version or 'unknown firmware'}. This writes "
                f"{version}, {source}.\n"
                f"It takes a few minutes, and the units on its bridge read offline until it is "
                f"done. Klipper keeps running.\n"
                f"Keep the unit powered. If an update stops partway, this menu finishes it.")
        cmd = f"AFC_BAMBU_FIRMWARE UNIT={unit.name} VERSION={version} CONFIRM=1"
        if file:
            cmd += f" FILE={file}"
        back = [("Back", f"AFC_BAMBU_FIRMWARE UNIT={unit.name}", "info")]
        prompt.create_custom_p(f"Flash {unit.name} to {version}?", text,
                               [(f"Flash {version}", cmd, "primary")], True, None, back)

    def _run(self, gcmd: Any, folder: str, unit: Any, version: Optional[str],
             file: Optional[str]) -> None:
        """
        Start the AMS updater on a unit, and show how its start went.

        :param gcmd: the command
        :param folder: the updater's folder
        :param unit: the unit
        :param version: the version to write; None finishes a stopped update
        :param file: the file holding it, or None to download it
        """
        if self._printing():
            error_str = "AFC_BAMBU_FIRMWARE: a print is running; update the AMS once it is done"
            raise gcmd.error(error_str)
        result = os.path.join(folder, RESULT_NAME)
        if self._busy(result):
            error_str = "AFC_BAMBU_FIRMWARE: an AMS update is running; let it finish"
            raise gcmd.error(error_str)
        params = f"--unit {unit.name} --mode go"
        if version is not None:
            params += f" --version {version}"
        if file is not None:
            params += f" --file {file}"
        # Whatever result is there after the run is this run's.
        try:
            os.remove(result)
        except OSError:
            pass
        self.afc.run_script(f'RUN_SHELL_COMMAND CMD=ams_flash PARAMS="{params}"', gcmd)
        if read_json(result).get("running"):
            prompt = AFCprompt(gcmd, self.logger)
            prompt.create_custom_p(f"Updating {unit.name}",
                                   f"{unit.name} is updating. This shows how it ended when it "
                                   f"is done.", None, False, None, [("Close", CLOSE, "info")])
            return
        self._show_result(gcmd, folder)

    @staticmethod
    def _busy(result: str) -> bool:
        """
        Whether an update is running: its result says so and was written within the longest hold.

        :param result: the result file
        :return bool: True while one runs
        """
        try:
            age = time.time() - os.path.getmtime(result)
        except OSError:
            return False
        return (age < HOLD_MAX_S
                and bool(read_json(result).get("running")))

    def _since_start(self, result: str) -> bool:
        """
        Whether a result was written since this Klipper run began, so a finished update's
        result shows until Klipper restarts.

        :param result: the result file
        :return bool: True when it was
        """
        try:
            return os.path.getmtime(result) >= self._started
        except OSError:
            return False

    def _show_result(self, gcmd: Any, folder: Optional[str]) -> None:
        """
        How the last update ended.

        :param gcmd: the command
        :param folder: the updater's folder
        """
        result = read_json(os.path.join(folder, RESULT_NAME)) if folder else {}
        if result.get("running"):
            title = "AMS update running"
            text = result.get("message") or "An AMS update is running."
        elif result.get("ok"):
            title = "AMS update done"
            text = result.get("message") or "The unit took its new firmware."
        else:
            title = "AMS update stopped"
            text = (result.get("message") or "The AMS updater did not say how it ended.")
            text += "\nIts log is logs/ams_flash.log."
        prompt = AFCprompt(gcmd, self.logger)
        prompt.create_custom_p(title, text, [("Flash AMS", "AFC_BAMBU_FIRMWARE", "primary")],
                               False, None, [("Close", CLOSE, "info")])

    def _cached_choices(self, folder: str, board: str,
                        eventtime: float) -> List[Dict[str, Any]]:
        """
        The images a board can take, reading the updater's folder at most every CHOICES_S seconds,
        since Klipper asks for the status several times a second.

        :param folder: the updater's folder
        :param board: the board
        :param eventtime: reactor time
        :return list: the images, as choices_for gives them
        """
        hit = self._choices.get(board)
        if (hit is None
            or eventtime - hit[0] >= CHOICES_S):
            hit = (eventtime, choices_for(folder, board))
            self._choices[board] = hit
        return hit[1]

    def get_status(self, eventtime: float) -> Dict[str, Any]:
        """
        What the BridgeBox display's Flash AMS pages show: each unit's firmware and the images it
        can take, and whether an update runs, or how the last one ended.

        :param eventtime: reactor time
        :return dict: the status
        """
        folder = updater_dir(self.printer)
        units = {}
        for unit in self._listed():
            board = self._board(unit)
            choices: List[Dict[str, Any]] = []
            if (folder is not None
                and board):
                choices = self._cached_choices(folder, board, eventtime)
            units[unit.name] = {"version": unit.fw_version or "",
                                "model": board_name(board) if board else "",
                                "choices": choices}
        status: Dict[str, Any] = {"updater": folder is not None, "running": False,
                                  "waiting": "", "result": {}, "units": units}
        if folder is None:
            return status
        result_path = os.path.join(folder, RESULT_NAME)
        result = read_json(result_path)
        status["running"] = self._busy(result_path)
        if (not status["running"]
                and not self._since_start(result_path)):
            result = {}
        status["result"] = {"unit": str(result.get("unit") or ""),
                            "ok": bool(result.get("ok")),
                            "message": str(result.get("message") or "")}
        stuck = read_json(os.path.join(folder, STATE_NAME)).get("unit")
        waiting = self._unit(stuck) if isinstance(stuck, str) else None
        if (waiting is not None
                and not status["running"]):
            status["waiting"] = waiting.name
        return status


def manager(printer: Any) -> AmsFirmware:
    """
    The printer's AMS firmware manager, made with the first Bambu AMS unit.

    :param printer: Klipper's printer object
    :return AmsFirmware: the one manager
    """
    mgr = printer.lookup_object(OBJECT_NAME, None)
    if mgr is None:
        mgr = AmsFirmware(printer)
        printer.add_object(OBJECT_NAME, mgr)
    return mgr
