from pathlib import Path
from typing import List, Sequence

import pytest

from installer.platform_core.console import NC, RED
from installer.platform_core.context import Context
from installer.platform_core.errors import InstallerExit
from installer.unit_registry.registry import (
    additional_buffer_supported,
    additional_default_name,
    unit_for,
)
from installer.unit_registry.selection import UnitSelection
from installer.unit_registry.units import bridgebox
from installer.unit_registry.units.bridgebox import (
    BridgeBox,
    ConnectionOption,
    ExtruderOption,
    PoolOption,
    SerialOption,
    connection_label,
    list_bridgebox_ports,
)
from tests.installer.fakes import PrinterTree, RecordingConsole, ScriptedPrompter
from tests.installer.unit_rows import additional_rows, install_rows

BRIDGEBOX = "BridgeBox"
TEMPLATE = "AFC_BridgeBox.cfg"


def make_ctx(tree: PrinterTree) -> Context:
    return Context.create(tree.afc_path, home=tree.home)


def install(tree: PrinterTree, sel: UnitSelection, additional: bool = False) -> Context:
    ctx = make_ctx(tree)
    ctx.afc_config_dir.mkdir(parents=True, exist_ok=True)
    action = BridgeBox().install_additional if additional else BridgeBox().copy_files
    action(ctx, RecordingConsole(), ScriptedPrompter([]), sel)
    return ctx


def files(ctx: Context) -> List[str]:
    return sorted(p.name for p in ctx.afc_config_dir.iterdir() if p.is_file())


def settings(ctx: Context, name: str = TEMPLATE) -> List[str]:
    """The uncommented ``key: value`` lines of a chain file, without their comments."""
    out = []
    for line in (ctx.afc_config_dir / name).read_text().splitlines():
        if line and not line.startswith(("#", "[")) and ":" in line:
            out.append(line.split("#")[0].rstrip())
    return out


def selection(**kwargs: object) -> UnitSelection:
    return UnitSelection(installation_type=BRIDGEBOX, **kwargs)  # type: ignore[arg-type]


def activate(option, tree: PrinterTree, sel: UnitSelection, answers: Sequence[str]):  # type: ignore
    console, prompter = RecordingConsole(), ScriptedPrompter(answers)
    message = option.activate(make_ctx(tree), console, prompter, sel)
    return message, console, prompter


class TestFirstInstall:
    def test_copies_the_hardware_and_chain_files(self, printer_tree: PrinterTree) -> None:
        ctx = install(printer_tree, selection())
        assert files(ctx) == [TEMPLATE, "AFC_Hardware.cfg"]
        source = printer_tree.afc_path / "templates" / "AFC_Hardware-AFC.cfg"
        assert (ctx.afc_config_dir / "AFC_Hardware.cfg").read_bytes() == source.read_bytes()

    def test_the_defaults_write_the_bambu_buffer_and_leave_the_rest_as_the_template(
        self, printer_tree: PrinterTree
    ) -> None:
        ctx = install(printer_tree, selection())
        lines = settings(ctx)
        assert "buffer_type: bambu" in lines
        assert "pool_ams: 4" in lines
        assert "pool_ht: 8" in lines
        assert "serial_port: auto" in lines
        assert "extruder: extruder" in lines
        assert "connection: wifi" not in lines
        template = (printer_tree.afc_path / "templates" / TEMPLATE).read_text()
        changed = template.replace("#buffer_type: FPS_PSF", "buffer_type: bambu")
        assert (ctx.afc_config_dir / TEMPLATE).read_text() == changed

    def test_the_counts_the_port_and_the_extruder_are_written(
        self, printer_tree: PrinterTree
    ) -> None:
        sel = selection(
            bridgebox_pool_ams=2, bridgebox_pool_ht=0,
            bridgebox_serial="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_AA-if00",
            bridgebox_extruder="extruder1",
        )
        lines = settings(install(printer_tree, sel))
        assert "pool_ams: 2" in lines
        assert "pool_ht: 0" in lines
        assert "serial_port: /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_AA-if00" in lines
        assert "extruder: extruder1" in lines
        assert "extruder: extruder" not in lines

    def test_usb_then_wifi_turns_the_connection_line_on(self, printer_tree: PrinterTree) -> None:
        lines = settings(install(printer_tree, selection(bridgebox_connection="usb_wifi")))
        assert "connection: wifi" in lines
        assert "serial_port: auto" in lines

    def test_wifi_only_points_serial_port_at_the_address_with_the_default_port(
        self, printer_tree: PrinterTree
    ) -> None:
        sel = selection(bridgebox_connection="wifi", bridgebox_address="192.168.1.50")
        lines = settings(install(printer_tree, sel))
        assert "serial_port: tcp://192.168.1.50:8888" in lines
        assert "serial_port: auto" not in lines
        assert "connection: wifi" not in lines

    def test_wifi_only_keeps_a_port_the_user_gave(self, printer_tree: PrinterTree) -> None:
        sel = selection(bridgebox_connection="wifi", bridgebox_address="bridge.local:9000")
        assert "serial_port: tcp://bridge.local:9000" in settings(install(printer_tree, sel))

    def test_a_file_without_a_final_newline_gets_one(self, printer_tree: PrinterTree) -> None:
        template = printer_tree.afc_path / "templates" / TEMPLATE
        template.write_text(template.read_text().rstrip("\n") + "\npool_ht: 8")
        ctx = install(printer_tree, selection(bridgebox_pool_ht=3))
        text = (ctx.afc_config_dir / TEMPLATE).read_text()
        assert text.endswith("\npool_ht: 3\n")


class TestAdditionalChain:
    def test_copies_the_template_under_the_chain_name_and_renames_the_section(
        self, printer_tree: PrinterTree
    ) -> None:
        sel = selection(unit_name="chain2", is_additional_unit=True, bridgebox_pool_ams=1)
        ctx = install(printer_tree, sel, additional=True)
        assert files(ctx) == ["AFC_chain2.cfg"]
        text = (ctx.afc_config_dir / "AFC_chain2.cfg").read_text()
        assert "\n[AFC_BridgeBox chain2]\n" in text
        assert "\n[AFC_BridgeBox chain1]\n" not in text
        assert "# [AFC_BridgeBox chain1] below" in text, "comments are left alone"
        lines = settings(ctx, "AFC_chain2.cfg")
        assert "buffer_type: bambu" in lines
        assert "pool_ams: 1" in lines

    def test_the_first_chains_unit_names_are_commented_out(
        self, printer_tree: PrinterTree
    ) -> None:
        sel = selection(unit_name="chain2", is_additional_unit=True)
        ctx = install(printer_tree, sel, additional=True)
        text = (ctx.afc_config_dir / "AFC_chain2.cfg").read_text()
        template = (printer_tree.afc_path / "templates" / TEMPLATE).read_text()
        for key in ("ams_names", "ht_names"):
            original = [line for line in template.splitlines() if line.startswith(f"{key}:")]
            assert original, key
            assert f"#{original[0]}\n" in text
            assert f"\n{original[0]}\n" not in text

    def test_no_buffer_target_the_chain_builds_its_own(self, printer_tree: PrinterTree) -> None:
        ctx = make_ctx(printer_tree)
        assert BridgeBox().buffer_target(ctx, selection()) is None
        assert additional_buffer_supported(BRIDGEBOX) is False


class TestChainNames:
    def write(self, ctx: Context, name: str, text: str) -> None:
        ctx.afc_config_dir.mkdir(parents=True, exist_ok=True)
        (ctx.afc_config_dir / name).write_text(text)

    def test_chain1_when_no_chain_exists(self, printer_tree: PrinterTree) -> None:
        ctx = make_ctx(printer_tree)
        self.write(ctx, "AFC_Turtle_1.cfg", "[AFC_BoxTurtle Turtle_1]\n")
        assert additional_default_name(ctx, selection()) == "chain1"

    def test_the_next_free_chain_after_the_sections_found_whatever_the_file_names(
        self, printer_tree: PrinterTree
    ) -> None:
        ctx = make_ctx(printer_tree)
        self.write(ctx, TEMPLATE, "[AFC_BridgeBox chain1]\nserial_port: auto\n")
        self.write(ctx, "AFC_Second.cfg", "# [AFC_BridgeBox chain9]\n[AFC_BridgeBox chain2]\n")
        assert additional_default_name(ctx, selection()) == "chain3"

    def test_a_gap_is_filled_first(self, printer_tree: PrinterTree) -> None:
        ctx = make_ctx(printer_tree)
        self.write(ctx, "AFC_chain2.cfg", "[AFC_BridgeBox chain2]\n")
        assert additional_default_name(ctx, selection()) == "chain1"

    def test_the_additional_screen_offers_that_name(self, printer_tree: PrinterTree) -> None:
        unit = unit_for(BRIDGEBOX)
        assert unit is not None
        ctx = make_ctx(printer_tree)
        self.write(ctx, TEMPLATE, "[AFC_BridgeBox chain1]\n")
        sel = selection(unit_name="stale")
        assert additional_rows(unit, ctx, sel)[0] == "BridgeBox Chain Name : chain2"

    def test_the_first_chain_is_always_chain1(self) -> None:
        assert BridgeBox().default_name == "chain1"


class TestMessageAndRows:
    def test_the_message_names_the_extruder_section(self, printer_tree: PrinterTree) -> None:
        sel = selection(bridgebox_extruder="extruder2")
        assert BridgeBox().message(make_ctx(printer_tree), sel) == (
            "\n- This chain feeds [AFC_extruder extruder2]. Check that your AFC config has that "
            "section.\n  "
        )

    def test_install_rows_show_every_choice(self) -> None:
        unit = unit_for(BRIDGEBOX)
        assert unit is not None
        sel = selection(bridgebox_connection="wifi", bridgebox_address="10.0.0.9")
        assert install_rows(unit, sel) == [
            "Units on this chain : 4 AMS, 8 AMS HT",
            "Connection : WiFi at 10.0.0.9",
            "USB serial port : none (WiFi only)",
            "Extruder : extruder",
        ]

    @pytest.mark.parametrize(
        "connection, address, label",
        [("usb", "", "USB"), ("usb_wifi", "", "USB, then WiFi"), ("wifi", "a.b", "WiFi at a.b")],
    )
    def test_connection_labels(self, connection: str, address: str, label: str) -> None:
        sel = selection(bridgebox_connection=connection, bridgebox_address=address)
        assert connection_label(sel) == label

    def test_selecting_a_bridgebox_fixes_the_buffer(self) -> None:
        sel = UnitSelection(buffer_type="TurtleNeck")
        BridgeBox().on_select(sel)
        assert sel.buffer_type == "Bambu"


class TestPoolOption:
    def test_both_counts_are_asked_and_stored(self, printer_tree: PrinterTree) -> None:
        sel = selection()
        message, console, prompter = activate(PoolOption(), printer_tree, sel, ["2", "1"])
        assert message == "Units on this chain: 2 AMS, 1 AMS HT"
        assert (sel.bridgebox_pool_ams, sel.bridgebox_pool_ht) == (2, 1)
        assert prompter.prompts == [
            "How many AMS or AMS 2 Pro units on this chain? (0-4, Enter keeps 4): ",
            "How many AMS HT units on this chain? (0-8, Enter keeps 8): ",
        ]
        assert console.messages[0][0] == "OTHER"

    def test_enter_keeps_a_count(self, printer_tree: PrinterTree) -> None:
        sel = selection(bridgebox_pool_ams=3, bridgebox_pool_ht=2)
        message, _, _ = activate(PoolOption(), printer_tree, sel, ["", " 0 "])
        assert message == "Units on this chain: 3 AMS, 0 AMS HT"
        assert (sel.bridgebox_pool_ams, sel.bridgebox_pool_ht) == (3, 0)

    @pytest.mark.parametrize("answers", [["5", "1"], ["1", "9"], ["x", "1"], ["1", "-1"], ["10", "1"]])
    def test_out_of_range_counts_are_refused_together(
        self, printer_tree: PrinterTree, answers: List[str]
    ) -> None:
        sel = selection()
        message, _, _ = activate(PoolOption(), printer_tree, sel, answers)
        assert message == f"{RED}Enter 0 to 4 AMS units and 0 to 8 AMS HT units.{NC}"
        assert (sel.bridgebox_pool_ams, sel.bridgebox_pool_ht) == (4, 8)

    def test_an_empty_chain_is_refused(self, printer_tree: PrinterTree) -> None:
        sel = selection()
        message, _, _ = activate(PoolOption(), printer_tree, sel, ["0", "0"])
        assert message == f"{RED}A chain needs at least one AMS or AMS HT.{NC}"
        assert (sel.bridgebox_pool_ams, sel.bridgebox_pool_ht) == (4, 8)

    def test_end_of_input_stops_the_installer(self, printer_tree: PrinterTree) -> None:
        with pytest.raises(InstallerExit) as info:
            activate(PoolOption(), printer_tree, selection(), ["1"])
        assert info.value.code == 1

    def test_value_shows_both_counts(self) -> None:
        assert PoolOption().value(selection(bridgebox_pool_ht=0)) == "4 AMS, 0 AMS HT"


class TestConnectionOption:
    def test_the_three_ways_are_explained_before_the_prompt(
        self, printer_tree: PrinterTree
    ) -> None:
        sel = selection()
        message, console, prompter = activate(ConnectionOption(), printer_tree, sel, ["1"])
        assert [kind for kind, _ in console.messages] == ["OTHER"] * 3
        assert [text[:2] for _, text in console.messages] == ["1.", "2.", "3."]
        assert prompter.prompts == [
            "How does Klipper reach this BridgeBox? (1-3, Enter keeps USB): "
        ]
        assert (message, sel.bridgebox_connection) == ("Connection: USB", "usb")

    def test_two_is_usb_then_wifi(self, printer_tree: PrinterTree) -> None:
        sel = selection()
        message, _, _ = activate(ConnectionOption(), printer_tree, sel, ["2"])
        assert (message, sel.bridgebox_connection) == ("Connection: USB, then WiFi", "usb_wifi")

    def test_three_asks_for_the_address(self, printer_tree: PrinterTree) -> None:
        sel = selection()
        message, _, prompter = activate(ConnectionOption(), printer_tree, sel, ["3", "192.168.1.50"])
        assert prompter.prompts[1] == (
            "The address the BridgeBox's setup page showed (for example 192.168.1.50): "
        )
        assert message == "Connection: WiFi at 192.168.1.50"
        assert (sel.bridgebox_connection, sel.bridgebox_address) == ("wifi", "192.168.1.50")

    @pytest.mark.parametrize("address", ["", "192.168.1.50 extra", "http://x", "a:b"])
    def test_a_bad_address_changes_nothing(self, printer_tree: PrinterTree, address: str) -> None:
        sel = selection()
        message, _, _ = activate(ConnectionOption(), printer_tree, sel, ["3", address])
        assert message == (
            f"{RED}Enter the address the way the setup page shows it, like 192.168.1.50.{NC}"
        )
        assert (sel.bridgebox_connection, sel.bridgebox_address) == ("usb", "")

    def test_enter_keeps_the_connection(self, printer_tree: PrinterTree) -> None:
        sel = selection(bridgebox_connection="usb_wifi")
        message, _, _ = activate(ConnectionOption(), printer_tree, sel, [""])
        assert (message, sel.bridgebox_connection) == ("Connection: USB, then WiFi", "usb_wifi")

    @pytest.mark.parametrize("answer", ["4", "0", "usb", "1 2"])
    def test_anything_else_is_refused(self, printer_tree: PrinterTree, answer: str) -> None:
        sel = selection()
        message, _, _ = activate(ConnectionOption(), printer_tree, sel, [answer])
        assert message == f"{RED}Enter 1, 2 or 3.{NC}"
        assert sel.bridgebox_connection == "usb"

    def test_value_is_the_label(self) -> None:
        assert ConnectionOption().value(selection(bridgebox_connection="usb_wifi")) == (
            "USB, then WiFi"
        )


@pytest.fixture
def ports(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> List[str]:
    """Two BridgeBox-like ports and one that is not, under a fake /dev/serial/by-id."""
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    names = [
        "usb-Raspberry_Pi_Pico_E66-if00",
        "usb-Espressif_USB_JTAG_serial_debug_unit_D9:94:9C-if00",
        "usb-Klipper_stm32_1234-if00",
    ]
    for name in names:
        (by_id / name).write_text("")
    monkeypatch.setattr(bridgebox, "SERIAL_BY_ID", by_id)
    return [str(by_id / names[1]), str(by_id / names[0])]


class TestSerialOption:
    def test_lists_the_bridgebox_ports_sorted_and_skips_other_boards(self, ports: List[str]) -> None:
        assert list_bridgebox_ports() == ports

    def test_no_directory_means_no_ports(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(bridgebox, "SERIAL_BY_ID", tmp_path / "missing")
        assert list_bridgebox_ports() == []

    def test_the_list_is_shown_with_auto_first(
        self, printer_tree: PrinterTree, ports: List[str]
    ) -> None:
        sel = selection()
        message, console, prompter = activate(SerialOption(), printer_tree, sel, ["2"])
        assert [text for _, text in console.messages] == [
            "A. auto: Klipper finds the BridgeBox on USB by itself.",
            f"1. {ports[0]}",
            f"2. {ports[1]}",
        ]
        assert prompter.prompts == [
            "USB serial port for this BridgeBox (A, a number, or a /dev/serial/by-id path; "
            "Enter keeps auto): "
        ]
        assert (message, sel.bridgebox_serial) == (f"USB serial port: {ports[1]}", ports[1])

    @pytest.mark.parametrize("answer", ["A", "a"])
    def test_a_is_auto(self, printer_tree: PrinterTree, ports: List[str], answer: str) -> None:
        sel = selection(bridgebox_serial=ports[0])
        message, _, _ = activate(SerialOption(), printer_tree, sel, [answer])
        assert (message, sel.bridgebox_serial) == ("USB serial port: auto", "auto")

    def test_a_dev_path_is_taken_as_typed(self, printer_tree: PrinterTree, ports: List[str]) -> None:
        sel = selection()
        path = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_AA:BB-if00"
        message, _, _ = activate(SerialOption(), printer_tree, sel, [f" {path} "])
        assert (message, sel.bridgebox_serial) == (f"USB serial port: {path}", path)

    def test_enter_keeps_the_port(self, printer_tree: PrinterTree, ports: List[str]) -> None:
        sel = selection(bridgebox_serial=ports[0])
        message, _, _ = activate(SerialOption(), printer_tree, sel, [""])
        assert (message, sel.bridgebox_serial) == (f"USB serial port: {ports[0]}", ports[0])

    @pytest.mark.parametrize("answer", ["0", "3", "x", "dev/ttyACM0", "/dev/tty ACM0"])
    def test_anything_else_is_refused(
        self, printer_tree: PrinterTree, ports: List[str], answer: str
    ) -> None:
        sel = selection()
        message, _, _ = activate(SerialOption(), printer_tree, sel, [answer])
        assert message == f"{RED}Enter A, a number from the list, or a /dev/serial/by-id path.{NC}"
        assert sel.bridgebox_serial == "auto"

    def test_a_wifi_only_chain_has_no_port_to_pick(self, printer_tree: PrinterTree) -> None:
        sel = selection(bridgebox_connection="wifi", bridgebox_address="10.0.0.9")
        message, console, prompter = activate(SerialOption(), printer_tree, sel, ["1"])
        assert message == "A WiFi only BridgeBox has no USB serial port"
        assert (console.messages, prompter.prompts) == ([], [])
        assert SerialOption().value(sel) == "none (WiFi only)"

    def test_value_is_the_port_otherwise(self) -> None:
        assert SerialOption().value(selection(bridgebox_serial="/dev/x")) == "/dev/x"


class TestExtruderOption:
    def test_a_name_is_stored(self, printer_tree: PrinterTree) -> None:
        sel = selection()
        message, _, prompter = activate(ExtruderOption(), printer_tree, sel, ["extruder1"])
        assert prompter.prompts == [
            "Extruder this chain feeds (for example extruder or extruder1; Enter keeps extruder): "
        ]
        assert (message, sel.bridgebox_extruder) == ("Extruder: extruder1", "extruder1")

    def test_enter_keeps_the_extruder(self, printer_tree: PrinterTree) -> None:
        sel = selection(bridgebox_extruder="extruder3")
        message, _, _ = activate(ExtruderOption(), printer_tree, sel, [""])
        assert (message, sel.bridgebox_extruder) == ("Extruder: extruder3", "extruder3")

    @pytest.mark.parametrize("answer", ["two words", "[AFC_extruder extruder]", "ext:1"])
    def test_a_name_that_is_not_a_section_name_is_refused(
        self, printer_tree: PrinterTree, answer: str
    ) -> None:
        sel = selection()
        message, _, _ = activate(ExtruderOption(), printer_tree, sel, [answer])
        assert message == (
            f"{RED}Enter the extruder's name as its [AFC_extruder] section names it, with no "
            f"spaces.{NC}"
        )
        assert sel.bridgebox_extruder == "extruder"

    def test_value_is_the_extruder(self) -> None:
        assert ExtruderOption().value(selection(bridgebox_extruder="e2")) == "e2"
