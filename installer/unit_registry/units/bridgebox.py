# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""BridgeBox: stock Bambu Lab AMS units on one pooled chain."""
from __future__ import annotations

import re
from pathlib import Path
from typing import List, Pattern, Tuple

from installer.config_editor.blocks import section_names
from installer.config_editor.document import ConfigDocument
from installer.platform_core.console import NC, RED, Console, Prompter
from installer.platform_core.context import Context
from installer.platform_core.errors import InstallerExit
from installer.platform_core.fsops import safe_copy
from installer.unit_registry.base import UnitType
from installer.unit_registry.options import NameOption, UnitOption
from installer.unit_registry.selection import UnitSelection

# Where the USB serial ports named like a BridgeBox are listed from.
SERIAL_BY_ID = Path("/dev/serial/by-id")
_PORT_PATTERNS = (
    "usb-Espressif_USB_JTAG_serial_debug_unit_*-if00",
    "usb-Raspberry_Pi_Pico_*-if00",
)
MAX_AMS = 4
MAX_HT = 8
_TCP_PORT = 8888
_TEMPLATE = "AFC_BridgeBox.cfg"
_FIRST_CHAIN = "chain1"
_ADDRESS = re.compile(r"[A-Za-z0-9.-]+(:[0-9]+)?")
_DEV_PATH = re.compile(r"/dev/[A-Za-z0-9._:/+-]+")
_NUMBER = re.compile(r"[0-9]+")
_EXTRUDER_NAME = re.compile(r"[A-Za-z0-9_-]+")

_LineEdit = Tuple[Pattern[str], str]


def _answer(prompter: Prompter, prompt: str) -> str:
    """
    One trimmed answer; end of input stops the installer.

    :param prompter: where answers come from
    :param prompt: prompt text
    :return str: the answer without surrounding whitespace
    """
    answer = prompter.ask(prompt)
    if answer is None:
        raise InstallerExit(1)
    return answer.strip()


def _count_ok(text: str, maximum: int) -> bool:
    """
    Is this a unit count from 0 to ``maximum``?

    :param text: the answer
    :param maximum: the highest count allowed, a single digit
    :return bool: True if allowed
    """
    return re.fullmatch(f"[0-{maximum}]", text) is not None


def connection_label(sel: UnitSelection) -> str:
    """
    How the chain reaches its BridgeBox, as the menu shows it.

    :param sel: the user's unit choices
    :return str: ``USB``, ``USB, then WiFi`` or ``WiFi at <address>``
    """
    if sel.bridgebox_connection == "usb_wifi":
        return "USB, then WiFi"
    if sel.bridgebox_connection == "wifi":
        return f"WiFi at {sel.bridgebox_address}"
    return "USB"


def list_bridgebox_ports() -> List[str]:
    """
    The ``/dev/serial/by-id`` ports named like a BridgeBox (an ESP32-S3 or a Pico), sorted.

    :return List[str]: the port paths
    """
    return sorted(str(port) for pattern in _PORT_PATTERNS for port in SERIAL_BY_ID.glob(pattern))


def _edit_lines(path: Path, edits: List[_LineEdit]) -> None:
    """
    Rewrite the lines of a config file that match a pattern whole, in the order given.

    Each edit applies to a line's current text, so an earlier edit can take a line out of
    a later one's reach, as the ``sed`` lines this ports did.

    :param path: config file
    :param edits: ``(pattern, replacement)`` pairs; the pattern must match a whole line
    """
    doc = ConfigDocument.load(path)
    for index, line in enumerate(doc.lines):
        body = line[:-1] if line.endswith("\n") else line
        for pattern, replacement in edits:
            if pattern.fullmatch(body):
                body = pattern.sub(replacement, body)
        doc.lines[index] = f"{body}\n"
    doc.save()


def _chain_edits(sel: UnitSelection) -> List[_LineEdit]:
    """
    The edits that turn the template into this chain: its buffer, counts, link and extruder.

    :param sel: the user's unit choices
    :return List[_LineEdit]: the edits, in the order to apply them
    """
    edits: List[_LineEdit] = [
        (re.compile(r"#buffer_type: FPS_PSF"), "buffer_type: bambu"),
        (re.compile(r"pool_ams: .*"), f"pool_ams: {sel.bridgebox_pool_ams}"),
        (re.compile(r"pool_ht: .*"), f"pool_ht: {sel.bridgebox_pool_ht}"),
    ]
    if sel.bridgebox_connection == "usb_wifi":
        edits.append((re.compile(r"#connection: wifi"), "connection: wifi"))
    elif sel.bridgebox_connection == "wifi":
        address = sel.bridgebox_address
        if ":" not in address:
            address = f"{address}:{_TCP_PORT}"
        edits.append((re.compile(r"serial_port: auto"), f"serial_port: tcp://{address}"))
    edits.append((re.compile(r"serial_port: auto"), f"serial_port: {sel.bridgebox_serial}"))
    edits.append((re.compile(r"extruder: extruder"), f"extruder: {sel.bridgebox_extruder}"))
    return edits


class PoolOption(UnitOption):
    """How many AMS and AMS HT bays the chain builds (``pool_ams`` and ``pool_ht``)."""

    label = "Units on this chain"

    def value(self, sel: UnitSelection) -> str:
        """
        The counts.

        :param sel: the user's unit choices
        :return str: ``<n> AMS, <m> AMS HT``
        """
        return f"{sel.bridgebox_pool_ams} AMS, {sel.bridgebox_pool_ht} AMS HT"

    def activate(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> str:
        """
        Ask for both counts; an empty answer keeps a count, and at least one bay is needed.

        :param ctx: installer context
        :param console: where the explanation goes
        :param prompter: where answers come from
        :param sel: the user's unit choices, updated in place when both counts are valid
        :return str: the status message
        """
        console.print_msg(
            "OTHER",
            "Count the most units this chain will run: raising the AMS count later moves "
            "every AMS HT lane.",
        )
        ams = _answer(
            prompter,
            f"How many AMS or AMS 2 Pro units on this chain? (0-{MAX_AMS}, Enter keeps "
            f"{sel.bridgebox_pool_ams}): ",
        ) or str(sel.bridgebox_pool_ams)
        ht = _answer(
            prompter,
            f"How many AMS HT units on this chain? (0-{MAX_HT}, Enter keeps "
            f"{sel.bridgebox_pool_ht}): ",
        ) or str(sel.bridgebox_pool_ht)
        if not _count_ok(ams, MAX_AMS) or not _count_ok(ht, MAX_HT):
            return f"{RED}Enter 0 to {MAX_AMS} AMS units and 0 to {MAX_HT} AMS HT units.{NC}"
        if int(ams) == 0 and int(ht) == 0:
            return f"{RED}A chain needs at least one AMS or AMS HT.{NC}"
        sel.bridgebox_pool_ams = int(ams)
        sel.bridgebox_pool_ht = int(ht)
        return f"Units on this chain: {ams} AMS, {ht} AMS HT"


class ConnectionOption(UnitOption):
    """How Klipper reaches the BridgeBox, and for WiFi only the address its setup page showed."""

    label = "Connection"

    def value(self, sel: UnitSelection) -> str:
        """
        The connection as the menu shows it.

        :param sel: the user's unit choices
        :return str: the label
        """
        return connection_label(sel)

    def activate(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> str:
        """
        Offer USB, USB then WiFi, and WiFi only; the last one asks for the address.

        :param ctx: installer context
        :param console: where the choices print
        :param prompter: where answers come from
        :param sel: the user's unit choices, updated in place
        :return str: the status message
        """
        console.print_msg("OTHER", "1. USB.")
        console.print_msg(
            "OTHER",
            "2. USB, then WiFi: once the BridgeBox joins your WiFi from its bb-bridge hotspot, "
            "Klipper moves over by itself.",
        )
        console.print_msg(
            "OTHER",
            "3. WiFi only: join the bb-bridge hotspot from a phone, choose your network, and "
            "note the address the page shows.",
        )
        choice = _answer(
            prompter,
            f"How does Klipper reach this BridgeBox? (1-3, Enter keeps {connection_label(sel)}): ",
        )
        if choice == "1":
            sel.bridgebox_connection = "usb"
        elif choice == "2":
            sel.bridgebox_connection = "usb_wifi"
        elif choice == "3":
            address = _answer(
                prompter,
                "The address the BridgeBox's setup page showed (for example 192.168.1.50): ",
            )
            if _ADDRESS.fullmatch(address) is None:
                return (
                    f"{RED}Enter the address the way the setup page shows it, like "
                    f"192.168.1.50.{NC}"
                )
            sel.bridgebox_connection = "wifi"
            sel.bridgebox_address = address
        elif choice:
            return f"{RED}Enter 1, 2 or 3.{NC}"
        return f"Connection: {connection_label(sel)}"


class SerialOption(UnitOption):
    """The USB ``serial_port`` the chain is written with: ``auto`` or a named port."""

    label = "USB serial port"

    def value(self, sel: UnitSelection) -> str:
        """
        The port, or a note that a WiFi only chain has none.

        :param sel: the user's unit choices
        :return str: the value
        """
        if sel.bridgebox_connection == "wifi":
            return "none (WiFi only)"
        return sel.bridgebox_serial

    def activate(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> str:
        """
        List the ports named like a BridgeBox and take ``A``, a number or a ``/dev`` path.

        :param ctx: installer context
        :param console: where the list prints
        :param prompter: where answers come from
        :param sel: the user's unit choices, updated in place
        :return str: the status message
        """
        if sel.bridgebox_connection == "wifi":
            return "A WiFi only BridgeBox has no USB serial port"
        ports = list_bridgebox_ports()
        console.print_msg("OTHER", "A. auto: Klipper finds the BridgeBox on USB by itself.")
        for number, port in enumerate(ports, 1):
            console.print_msg("OTHER", f"{number}. {port}")
        choice = _answer(
            prompter,
            "USB serial port for this BridgeBox (A, a number, or a /dev/serial/by-id path; "
            f"Enter keeps {sel.bridgebox_serial}): ",
        )
        if choice.lower() == "a":
            sel.bridgebox_serial = "auto"
        elif _DEV_PATH.fullmatch(choice):
            sel.bridgebox_serial = choice
        elif _NUMBER.fullmatch(choice) and 1 <= int(choice) <= len(ports):
            sel.bridgebox_serial = ports[int(choice) - 1]
        elif choice:
            return f"{RED}Enter A, a number from the list, or a /dev/serial/by-id path.{NC}"
        return f"USB serial port: {sel.bridgebox_serial}"


class ExtruderOption(UnitOption):
    """The ``[AFC_extruder]`` the chain feeds."""

    label = "Extruder"

    def value(self, sel: UnitSelection) -> str:
        """
        The extruder name.

        :param sel: the user's unit choices
        :return str: the name
        """
        return sel.bridgebox_extruder

    def activate(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> str:
        """
        Ask for the extruder's section name; an empty answer keeps the current one.

        :param ctx: installer context
        :param console: unused
        :param prompter: where answers come from
        :param sel: the user's unit choices, updated in place when the name is valid
        :return str: the status message
        """
        name = _answer(
            prompter,
            "Extruder this chain feeds (for example extruder or extruder1; Enter keeps "
            f"{sel.bridgebox_extruder}): ",
        ) or sel.bridgebox_extruder
        if _EXTRUDER_NAME.fullmatch(name) is None:
            return (
                f"{RED}Enter the extruder's name as its [AFC_extruder] section names it, with "
                f"no spaces.{NC}"
            )
        sel.bridgebox_extruder = name
        return f"Extruder: {name}"


class BridgeBox(UnitType):
    """
    A BridgeBox chain: stock Bambu Lab AMS, AMS 2 Pro and AMS HT units on one RS-485 bus.

    Neither the BridgeBox nor the AMS units are an MCU. Each chain builds its own
    ``[AFC_buffer]`` from the AMS's buffer, so the buffer is not a choice, and a second
    BridgeBox is a second chain named ``chainN``.
    """

    display_name = "BridgeBox"
    key = "BridgeBox"
    additional_name_prefix = "chain"
    buffer_selectable = False
    ramming_hardware = "the Bambu AMS buffer"

    @property
    def default_name(self) -> str:
        """
        The first chain's name, fixed by the template.

        :return str: ``chain1``
        """
        return _FIRST_CHAIN

    def copy_files(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the hardware file and the chain file, then write the chain's choices into it.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        templates = ctx.afc_path / "templates"
        cfg = ctx.afc_config_dir
        hardware = templates / "AFC_Hardware-AFC.cfg"
        safe_copy(ctx, console, prompter, hardware, cfg / "AFC_Hardware.cfg")
        safe_copy(ctx, console, prompter, templates / _TEMPLATE, cfg / _TEMPLATE)
        _edit_lines(cfg / _TEMPLATE, _chain_edits(sel))

    def install_additional(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the chain file under the new chain's name and rename the section.

        A second chain's units name themselves after the chain, so the first chain's unit
        names are commented out.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        unit_file = ctx.afc_config_dir / f"AFC_{sel.unit_name}.cfg"
        safe_copy(ctx, console, prompter, ctx.afc_path / "templates" / _TEMPLATE, unit_file)
        edits: List[_LineEdit] = [
            (re.compile(rf"\[AFC_BridgeBox {_FIRST_CHAIN}\]"), f"[AFC_BridgeBox {sel.unit_name}]"),
            (re.compile(r"(ams_names|ht_names):.*"), r"#\g<0>"),
        ]
        _edit_lines(unit_file, edits + _chain_edits(sel))

    def lowest_free_name(self, ctx: Context, sel: UnitSelection) -> str:
        """
        The first ``chainN`` no ``[AFC_BridgeBox chainN]`` section in the AFC config uses.

        ``chain1`` when the system has other units but no BridgeBox yet.

        :param ctx: installer context
        :param sel: the user's unit choices
        :return str: the chain name
        """
        taken = section_names(sorted(ctx.afc_config_dir.glob("*.cfg")), "AFC_BridgeBox")
        number = 1
        while f"chain{number}" in taken:
            number += 1
        return f"chain{number}"

    def message(self, ctx: Context, sel: UnitSelection) -> str:
        """
        Remind the user that the extruder the chain feeds needs its own section.

        :param ctx: installer context
        :param sel: the user's unit choices
        :return str: the message
        """
        return (
            f"\n- This chain feeds [AFC_extruder {sel.bridgebox_extruder}]. Check that your AFC "
            "config has that section.\n  "
        )

    def install_options(self) -> List[UnitOption]:
        """
        The chain's counts, connection, USB port and extruder.

        :return List[UnitOption]: the options
        """
        return [PoolOption(), ConnectionOption(), SerialOption(), ExtruderOption()]

    def additional_options(self) -> List[UnitOption]:
        """
        The chain name, then the same options as the install screen.

        :return List[UnitOption]: the options
        """
        return [NameOption("BridgeBox Chain Name", additional_prompt=True), *self.install_options()]

    def on_select(self, sel: UnitSelection) -> None:
        """
        The AMS's own buffer is built into each chain and cannot be swapped.

        :param sel: the user's unit choices, ``buffer_type`` is set
        """
        sel.buffer_type = "Bambu"
