# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""BoxTurtle 4-Lane, 8-Lane and USB."""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Dict, List, Tuple

from installer.config_editor.document import ConfigDocument
from installer.config_editor.edits import replace_text
from installer.platform_core.console import NC, RED, Console, Prompter
from installer.platform_core.context import Context
from installer.platform_core.errors import InstallerExit
from installer.platform_core.fsops import safe_copy
from installer.unit_registry.base import BufferTarget, UnitType
from installer.unit_registry.options import NameOption, UnitOption
from installer.unit_registry.selection import UnitSelection

SERIAL_BY_ID = Path("/dev/serial/by-id")
_PORT_PREFIX = "usb-AFC_BoxTurtle_"
_PORT_SUFFIX = "-if00"
# A typed chip serial: letters and digits, long enough never to be A or a number from the list.
_CHIP_SERIAL = re.compile(r"[A-Za-z0-9]{8,}")
_NUMBER = re.compile(r"[0-9]+")
# An uncommented lane section header; the lane number is the first group.
_LANE_SECTION = re.compile(r"\s*\[AFC_\w+\s+lane([0-9]+)\s*\]\s*")
_USB_SECTION = re.compile(r"\[AFC_BoxTurtleUSB\s+([^\s\]]+)\s*\]")
_SERIAL_UID = re.compile(r"serial_uid\s*[:=]\s*(\S+)")
_SERIAL_UID_LINE = re.compile(r"#serial_uid: <chip serial>")


def list_turtle_serials() -> List[str]:
    """
    The chip serials of the USB BoxTurtles plugged in, from their ``/dev/serial/by-id`` names.

    :return List[str]: the serials, sorted
    """
    names = (port.name for port in SERIAL_BY_ID.glob(f"{_PORT_PREFIX}*{_PORT_SUFFIX}"))
    return sorted(serial for serial in (name[len(_PORT_PREFIX):-len(_PORT_SUFFIX)]
                                        for name in names) if serial)


def turtle_owners(ctx: Context) -> Dict[str, str]:
    """
    Which unit each known BoxTurtle belongs to: the board a unit keeps (saved next to
    ``AFC.var``) or the one its ``serial_uid`` names.

    :param ctx: installer context
    :return Dict[str, str]: chip serial -> unit name
    """
    owners: Dict[str, str] = {}
    try:
        saved = json.loads((ctx.afc_config_dir / "AFC.var.turtle").read_text())
    except (OSError, ValueError):
        saved = {}
    if isinstance(saved, dict):
        owners.update({serial: unit for unit, serial in saved.items() if isinstance(serial, str)})
    for path in sorted(ctx.afc_config_dir.glob("*.cfg")):
        unit = ""
        for line in path.read_text(errors="replace").splitlines():
            body = line.split("#", 1)[0].strip()
            if body.startswith("["):
                section = _USB_SECTION.fullmatch(body)
                unit = section.group(1) if section else ""
            elif unit:
                uid = _SERIAL_UID.fullmatch(body)
                if uid:
                    owners[uid.group(1)] = unit
    return owners


def next_lane_number(config_dir: Path, skip: Path) -> int:
    """
    The number after the highest ``lane<N>`` section in the AFC config files.

    :param config_dir: the AFC config directory
    :param skip: a file to leave out (the unit file being written)
    :return int: the first free lane number, 1 when there are no lanes
    """
    highest = 0
    for path in config_dir.glob("*.cfg"):
        if path == skip:
            continue
        for line in path.read_text(errors="replace").splitlines():
            lane = _LANE_SECTION.fullmatch(line)
            if lane:
                highest = max(highest, int(lane.group(1)))
    return highest + 1


def _number_lanes(path: Path, first: int) -> None:
    """
    Renumber a unit file's lanes from ``first`` on, keeping their order.

    :param path: the unit file, with its lanes numbered from 1
    :param first: the first lane's new number
    """
    doc = ConfigDocument.load(path)
    for index, line in enumerate(doc.lines):
        lane = _LANE_SECTION.fullmatch(line.rstrip("\n"))
        if lane:
            number = int(lane.group(1)) + first - 1
            doc.lines[index] = line.replace(f"lane{lane.group(1)}", f"lane{number}", 1)
    doc.save()


def _name_board(path: Path, serial: str) -> None:
    """
    Name the unit's board by its chip serial (``serial_uid``); ``auto`` leaves it commented out.

    :param path: the unit file
    :param serial: the chip serial, or ``auto``
    """
    if serial == "auto":
        return
    doc = ConfigDocument.load(path)
    for index, line in enumerate(doc.lines):
        if _SERIAL_UID_LINE.match(line):
            doc.lines[index] = _SERIAL_UID_LINE.sub(f"serial_uid: {serial}", line, count=1)
    doc.save()


class _BoxTurtle(UnitType):
    """Behavior shared by the BoxTurtle types; they differ mainly in their file names."""

    additional_name_prefix = "Turtle"
    additional_buffer_supported = True
    mcu_name: str  # empty: the unit has no mcu file
    template: str
    section_prefix = "AFC_BoxTurtle"

    def copy_files(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the mcu file, the shared hardware file and the named unit file.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        cfg = ctx.afc_config_dir
        if self.mcu_name:
            mcu = f"{self.mcu_name}.cfg"
            safe_copy(
                ctx, console, prompter, ctx.afc_path / "config" / "mcu" / mcu, cfg / "mcu" / mcu
            )
        hardware = ctx.afc_path / "templates" / "AFC_Hardware-AFC.cfg"
        safe_copy(ctx, console, prompter, hardware, cfg / "AFC_Hardware.cfg")
        unit_template = ctx.afc_path / "templates" / self.template
        safe_copy(ctx, console, prompter, unit_template, cfg / f"AFC_{sel.unit_name}.cfg")

    def install_additional(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the unit file under its new name with its own mcu file.

        The edits run even if the user chose to keep an existing file.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        cfg = ctx.afc_config_dir
        name = sel.unit_name
        unit_file = cfg / f"AFC_{name}.cfg"
        unit_template = ctx.afc_path / "templates" / self.template
        safe_copy(ctx, console, prompter, unit_template, unit_file)
        replace_text(unit_file, "Turtle_1", name)
        if not self.mcu_name:
            return
        mcu = ctx.afc_path / "config" / "mcu" / f"{self.mcu_name}.cfg"
        safe_copy(ctx, console, prompter, mcu, cfg / "mcu" / f"AFC_{name}_mcu.cfg")
        replace_text(
            unit_file, f"include mcu/{self.mcu_name}.cfg", f"include mcu/AFC_{name}_mcu.cfg"
        )

    def buffer_target(self, ctx: Context, sel: UnitSelection) -> BufferTarget:
        """
        The unit's own file and section.

        :param ctx: installer context
        :param sel: the user's unit choices
        :return BufferTarget: where the buffer goes
        """
        return BufferTarget(
            unit_name=sel.unit_name,
            unit_section_prefix=self.section_prefix,
            extruder_file=ctx.afc_config_dir / f"AFC_{sel.unit_name}.cfg",
            section_name=sel.unit_name,
        )

    def message(self, ctx: Context, sel: UnitSelection) -> str:
        """
        Remind the user to enter CAN bus or serial information.

        :param ctx: installer context
        :param sel: the user's unit choices
        :return str: the message, indented to line up with the install message
        """
        return (
            "\n- Ensure you enter either your CAN bus or serial information in the "
            f"{ctx.afc_config_dir}/AFC_{sel.unit_name}.cfg file\n  "
        )

    def install_options(self) -> List[UnitOption]:
        """
        The name option of the install screen.

        :return List[UnitOption]: the options
        """
        return [NameOption("BoxTurtle Name")]

    def additional_options(self) -> List[UnitOption]:
        """
        The name option of the additional-unit screen.

        :return List[UnitOption]: the options
        """
        return [NameOption("BoxTurtle Name", additional_prompt=True)]





class BoxTurtle4Lane(_BoxTurtle):
    """BoxTurtle with four lanes."""

    display_name = "BoxTurtle (4-Lane)"
    key = "BoxTurtle_4Lane"
    mcu_name = "AFC_Lite"
    template = "AFC_Turtle_1.cfg"


class BoxTurtle8Lane(_BoxTurtle):
    """BoxTurtle with eight lanes."""

    display_name = "BoxTurtle (8-Lane)"
    key = "BoxTurtle_8Lane"
    mcu_name = "AFC_Pro"
    template = "AFC_Pro_Turtle_1.cfg"


class ChipSerialOption(UnitOption):
    """
    The board a USB BoxTurtle unit takes: ``auto`` (the first one plugged in that no other unit
    has, kept from then on) or one named by its chip serial.
    """

    label = "BoxTurtle serial"

    def value(self, sel: UnitSelection) -> str:
        """
        The chip serial, or ``auto``.

        :param sel: the user's unit choices
        :return str: the value
        """
        return sel.turtle_serial

    def activate(self, ctx: Context, console: Console, prompter: Prompter,
                 sel: UnitSelection) -> str:
        """
        List the BoxTurtles plugged in, with the unit each one already belongs to, and take
        ``A``, a number or a typed chip serial.

        :param ctx: installer context
        :param console: where the list prints
        :param prompter: where answers come from
        :param sel: the user's unit choices, updated in place
        :return str: the status message
        """
        serials = list_turtle_serials()
        owners = turtle_owners(ctx)
        console.print_msg(
            "OTHER", "A. auto: the first BoxTurtle plugged in that no other unit has, kept "
                     "from then on.")
        for number, serial in enumerate(serials, 1):
            owner = f" ({owners[serial]})" if serial in owners else ""
            console.print_msg("OTHER", f"{number}. {serial}{owner}")
        answer = prompter.ask(
            "BoxTurtle for this unit (A, a number, or its chip serial; "
            f"Enter keeps {sel.turtle_serial}): ")
        if answer is None:
            raise InstallerExit(1)
        choice = answer.strip()
        if choice.lower() == "a":
            sel.turtle_serial = "auto"
        elif _NUMBER.fullmatch(choice) and 1 <= int(choice) <= len(serials):
            sel.turtle_serial = serials[int(choice) - 1]
        elif _CHIP_SERIAL.fullmatch(choice):
            sel.turtle_serial = choice
        elif choice:
            return f"{RED}Enter A, a number from the list, or a chip serial.{NC}"
        return f"BoxTurtle serial: {sel.turtle_serial}"


_USB_BUFFER_ORDER: Tuple[str, ...] = (
    "Internal", "TurtleNeck", "TurtleNeckV2", "TurtleNeck2Remastered", "Compensator", "FPS_PSF"
)


class BoxTurtleUSB(_BoxTurtle):
    """
    BoxTurtle on its own USB firmware: an AFC-Lite that is not a Klipper MCU.

    There is no mcu file and there are no pins. The board runs the TurtleNeck wired to
    it, so the buffer is that TurtleNeck (`Internal`) unless the user picks a buffer
    wired to a Klipper MCU, which the unit then names in its `buffer:` option.
    """

    display_name = "BoxTurtle (USB) Serial Driver"
    key = "BoxTurtle_USB"
    mcu_name = ""
    template = "AFC_Turtle_USB_1.cfg"
    section_prefix = "AFC_BoxTurtleUSB"
    additional_buffer_options = _USB_BUFFER_ORDER
    additional_buffer_default = "Internal"
    force_additional_buffer_default = True

    def copy_files(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the hardware file and the named unit file, naming its board if one was picked.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        super().copy_files(ctx, console, prompter, sel)
        _name_board(ctx.afc_config_dir / f"AFC_{sel.unit_name}.cfg", sel.turtle_serial)

    def install_additional(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the unit file under its new name, number its lanes after the lanes already in the
        AFC config, and name its board if one was picked.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        unit_file = ctx.afc_config_dir / f"AFC_{sel.unit_name}.cfg"
        first = next_lane_number(ctx.afc_config_dir, unit_file)
        super().install_additional(ctx, console, prompter, sel)
        _number_lanes(unit_file, first)
        _name_board(unit_file, sel.turtle_serial)

    def install_options(self) -> List[UnitOption]:
        """
        The name and board options of the install screen.

        :return List[UnitOption]: the options
        """
        return [NameOption("BoxTurtle Name"), ChipSerialOption()]

    def additional_options(self) -> List[UnitOption]:
        """
        The name and board options of the additional-unit screen.

        :return List[UnitOption]: the options
        """
        return [NameOption("BoxTurtle Name", additional_prompt=True), ChipSerialOption()]

    def message(self, ctx: Context, sel: UnitSelection) -> str:
        """
        Point the user at the firmware and at the settings to check in the unit file.

        :param ctx: installer context
        :param sel: the user's unit choices
        :return str: the message, indented to line up with the install message
        """
        return (
            "\n- Flash firmwares/BoxTurtleUSB/boxturtle.bin on the AFC-Lite, then check hub, "
            "extruder and each lane's dist_hub in the "
            f"{ctx.afc_config_dir}/AFC_{sel.unit_name}.cfg file\n  "
        )

    def on_select(self, sel: UnitSelection) -> None:
        """
        Start with the TurtleNeck on the board as the buffer.

        :param sel: the user's unit choices, updated in place
        """
        sel.buffer_type = "Internal"

    def cycle_buffer(self, sel: UnitSelection) -> None:
        """
        Step to the next buffer type: `Internal`, then the buffers on a Klipper MCU.

        :param sel: the user's unit choices, `buffer_type` is updated
        """
        if sel.buffer_type in _USB_BUFFER_ORDER:
            position = (_USB_BUFFER_ORDER.index(sel.buffer_type) + 1) % len(_USB_BUFFER_ORDER)
        else:
            position = 0
        sel.buffer_type = _USB_BUFFER_ORDER[position]
