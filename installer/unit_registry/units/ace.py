# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""Anycubic ACE Pro (V1) and ACE 2 Pro (V2_Pro)."""
from __future__ import annotations

from typing import Dict, List, Tuple

from installer.config_editor.edits import replace_text
from installer.platform_core.console import Console, Prompter
from installer.platform_core.context import Context
from installer.platform_core.fsops import safe_copy
from installer.unit_registry.base import BufferTarget, UnitType
from installer.unit_registry.options import ACE_UNIT_TYPES, ChoiceOption, NameOption, UnitOption
from installer.unit_registry.selection import UnitSelection

_V2_PRO = "V2_Pro"
# The buffer cycle: the ACE buffers internally, so Internal is the default and None is
# offered as well, since ramming is never active and FPS_PSF has nothing to fall back to.
_NEXT_BUFFER: Dict[str, str] = {
    "Internal": "TurtleNeck",
    "TurtleNeck": "TurtleNeckV2",
    "TurtleNeckV2": "TurtleNeck2Remastered",
    "TurtleNeck2Remastered": "FPS_PSF",
    "FPS_PSF": "None",
}


class Ace(UnitType):
    """
    The ACE is not an MCU: no board or motor type, no pins or steppers file.

    The two generations are separate Klipper modules (``AFC_ACE`` and ``AFC_ACE2``) with
    their own templates and default names, chosen with the unit type option rather than a
    second entry in the type picker. Its load pulses filament until the toolhead sensor
    triggers, so ramming is not offered.
    """

    display_name = "ACE"
    key = "ACE"
    additional_name_prefix = "Ace"
    additional_buffer_supported = True
    additional_buffer_options: Tuple[str, ...] = (
        "None", "Internal", "TurtleNeck", "TurtleNeckV2", "TurtleNeck2Remastered", "FPS_PSF"
    )
    additional_buffer_default = "Internal"

    def additional_prefix(self, sel: UnitSelection) -> str:
        """
        ``Ace2`` for an ACE 2 Pro, ``Ace`` for an ACE Pro.

        :param sel: the user's unit choices
        :return str: the prefix
        """
        return "Ace2" if sel.ace_unit_type == _V2_PRO else "Ace"

    def _first_name(self, sel: UnitSelection) -> str:
        return f"{self.additional_prefix(sel)}_1"

    def _section_prefix(self, sel: UnitSelection) -> str:
        return "AFC_ACE2" if sel.ace_unit_type == _V2_PRO else "AFC_ACE"

    def _template(self, sel: UnitSelection) -> str:
        return "AFC_ACE2_1.cfg" if sel.ace_unit_type == _V2_PRO else "AFC_ACE_1.cfg"

    def copy_files(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the hardware file and the generation's unit file under its fixed name.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        templates = ctx.afc_path / "templates"
        cfg = ctx.afc_config_dir
        hardware = templates / "AFC_Hardware-AFC.cfg"
        safe_copy(ctx, console, prompter, hardware, cfg / "AFC_Hardware.cfg")
        template = self._template(sel)
        safe_copy(ctx, console, prompter, templates / template, cfg / template)

    def install_additional(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the generation's unit file under the new name and rename the unit inside it.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        unit_file = ctx.afc_config_dir / f"AFC_{sel.unit_name}.cfg"
        source = ctx.afc_path / "templates" / self._template(sel)
        safe_copy(ctx, console, prompter, source, unit_file)
        replace_text(unit_file, self._first_name(sel), sel.unit_name)

    def buffer_target(self, ctx: Context, sel: UnitSelection) -> BufferTarget:
        """
        The generation's fixed first unit, in the file ``copy_files`` wrote, or the named
        additional unit.

        :param ctx: installer context
        :param sel: the user's unit choices
        :return BufferTarget: where the buffer goes
        """
        if sel.is_additional_unit:
            name = sel.unit_name
            unit_file = ctx.afc_config_dir / f"AFC_{name}.cfg"
        else:
            name = self._first_name(sel)
            unit_file = ctx.afc_config_dir / self._template(sel)
        return BufferTarget(name, self._section_prefix(sel), unit_file, name)

    def message(self, ctx: Context, sel: UnitSelection) -> str:
        """
        Point the user at the serial port settings, and the RFID section of an ACE 2 Pro.

        :param ctx: installer context
        :param sel: the user's unit choices
        :return str: the message
        """
        unit_file = ctx.afc_config_dir / self._template(sel)
        message = (
            f"\n- Ensure you check the serial_port and ace_index settings in the {unit_file} "
            "file.\n"
        )
        if sel.ace_unit_type == _V2_PRO:
            message += "\n- Review the [AFC_ACE2_rfid] section if your unit has an RFID reader.\n"
        return message + "  "

    def install_options(self) -> List[UnitOption]:
        """
        The generation option of the install screen.

        :return List[UnitOption]: the options
        """
        return [ChoiceOption("ACE Unit Type", "ace_unit_type", ACE_UNIT_TYPES)]

    def additional_options(self) -> List[UnitOption]:
        """
        The name and generation options of the additional-unit screen.

        :return List[UnitOption]: the options
        """
        return [NameOption("ACE Name", additional_prompt=True), *self.install_options()]

    def on_select(self, sel: UnitSelection) -> None:
        """
        An ACE loads to a toolhead sensor and buffers internally.

        :param sel: the user's unit choices, ``toolhead_sensor`` and ``buffer_type`` are set
        """
        sel.toolhead_sensor = "Sensor"
        sel.buffer_type = "Internal"

    def after_sensor_change(self, sel: UnitSelection) -> str:
        """
        Keep the toolhead sensor: ramming is not a load strategy the ACE supports.

        :param sel: the user's unit choices, ``toolhead_sensor`` is set back
        :return str: the status message
        """
        sel.toolhead_sensor = "Sensor"
        return "ACE requires a toolhead sensor; ramming is not supported"

    def cycle_buffer(self, sel: UnitSelection) -> None:
        """
        Step through Internal, the TurtleNecks, FPS_PSF and None, then back to Internal.

        :param sel: the user's unit choices, ``buffer_type`` is updated
        """
        sel.buffer_type = _NEXT_BUFFER.get(sel.buffer_type, "Internal")
