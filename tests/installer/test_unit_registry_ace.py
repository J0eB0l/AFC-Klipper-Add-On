from pathlib import Path
from typing import List

import pytest

from installer.platform_core.context import Context
from installer.unit_registry.base import BufferTarget
from installer.unit_registry.registry import (
    additional_buffer_default,
    additional_buffer_options,
    additional_buffer_supported,
    additional_default_name,
    unit_for,
)
from installer.unit_registry.selection import UnitSelection
from installer.unit_registry.units.ace import Ace
from tests.installer.fakes import PrinterTree, RecordingConsole, ScriptedPrompter
from tests.installer.unit_rows import additional_rows, install_rows

ACE = "ACE"
V1 = "V1"
V2 = "V2_Pro"


def make_ctx(tree: PrinterTree) -> Context:
    return Context.create(tree.afc_path, home=tree.home)


def install(tree: PrinterTree, sel: UnitSelection, additional: bool = False) -> Context:
    ctx = make_ctx(tree)
    ctx.afc_config_dir.mkdir(parents=True, exist_ok=True)
    action = Ace().install_additional if additional else Ace().copy_files
    action(ctx, RecordingConsole(), ScriptedPrompter([]), sel)
    return ctx


def files(ctx: Context) -> List[str]:
    return sorted(p.name for p in ctx.afc_config_dir.iterdir() if p.is_file())


class TestFirstInstall:
    @pytest.mark.parametrize("generation, template", [(V1, "AFC_ACE_1.cfg"), (V2, "AFC_ACE2_1.cfg")])
    def test_copies_the_generations_template_under_its_own_name_whatever_the_unit_name(
        self, printer_tree: PrinterTree, generation: str, template: str
    ) -> None:
        sel = UnitSelection(installation_type=ACE, unit_name="MyAce", ace_unit_type=generation)
        ctx = install(printer_tree, sel)
        assert files(ctx) == [template, "AFC_Hardware.cfg"]
        source = (printer_tree.afc_path / "templates" / template).read_bytes()
        assert (ctx.afc_config_dir / template).read_bytes() == source

    def test_the_hardware_file_is_the_shared_afc_one(self, printer_tree: PrinterTree) -> None:
        ctx = install(printer_tree, UnitSelection(installation_type=ACE))
        source = printer_tree.afc_path / "templates" / "AFC_Hardware-AFC.cfg"
        assert (ctx.afc_config_dir / "AFC_Hardware.cfg").read_bytes() == source.read_bytes()


class TestAdditionalUnit:
    @pytest.mark.parametrize(
        "generation, first, section", [(V1, "Ace_1", "AFC_ACE"), (V2, "Ace2_1", "AFC_ACE2")]
    )
    def test_renames_the_first_unit_throughout_the_copied_template(
        self, printer_tree: PrinterTree, generation: str, first: str, section: str
    ) -> None:
        sel = UnitSelection(
            installation_type=ACE, unit_name="Second", ace_unit_type=generation,
            is_additional_unit=True,
        )
        ctx = install(printer_tree, sel, additional=True)
        text = (ctx.afc_config_dir / "AFC_Second.cfg").read_text()
        assert files(ctx) == ["AFC_Second.cfg"]
        assert f"[{section} Second]" in text
        assert first not in text
        assert "UNIT=Second" in text

    def test_it_does_not_touch_the_hardware_file(self, printer_tree: PrinterTree) -> None:
        sel = UnitSelection(installation_type=ACE, unit_name="Ace_2", is_additional_unit=True)
        ctx = install(printer_tree, sel, additional=True)
        assert "AFC_Hardware.cfg" not in files(ctx)


class TestBufferTarget:
    @pytest.mark.parametrize(
        "generation, name, section, template",
        [(V1, "Ace_1", "AFC_ACE", "AFC_ACE_1.cfg"), (V2, "Ace2_1", "AFC_ACE2", "AFC_ACE2_1.cfg")],
    )
    def test_the_first_unit_is_the_section_in_the_file_copy_files_wrote(
        self, printer_tree: PrinterTree, generation: str, name: str, section: str, template: str
    ) -> None:
        ctx = make_ctx(printer_tree)
        sel = UnitSelection(installation_type=ACE, unit_name="Ignored", ace_unit_type=generation)
        assert Ace().buffer_target(ctx, sel) == BufferTarget(
            name, section, ctx.afc_config_dir / template, name
        )

    @pytest.mark.parametrize("generation, section", [(V1, "AFC_ACE"), (V2, "AFC_ACE2")])
    def test_an_additional_unit_is_its_own_file_and_section(
        self, printer_tree: PrinterTree, generation: str, section: str
    ) -> None:
        ctx = make_ctx(printer_tree)
        sel = UnitSelection(
            installation_type=ACE, unit_name="Mine", ace_unit_type=generation,
            is_additional_unit=True,
        )
        assert Ace().buffer_target(ctx, sel) == BufferTarget(
            "Mine", section, ctx.afc_config_dir / "AFC_Mine.cfg", "Mine"
        )


class TestMessage:
    def test_v1_points_at_the_serial_settings_only(self, printer_tree: PrinterTree) -> None:
        ctx = make_ctx(printer_tree)
        message = Ace().message(ctx, UnitSelection(installation_type=ACE))
        assert message == (
            "\n- Ensure you check the serial_port and ace_index settings in the "
            f"{ctx.afc_config_dir}/AFC_ACE_1.cfg file.\n  "
        )

    def test_v2_adds_the_rfid_section(self, printer_tree: PrinterTree) -> None:
        ctx = make_ctx(printer_tree)
        message = Ace().message(ctx, UnitSelection(installation_type=ACE, ace_unit_type=V2))
        assert "AFC_ACE2_1.cfg file.\n" in message
        assert message.endswith(
            "\n- Review the [AFC_ACE2_rfid] section if your unit has an RFID reader.\n  "
        )


class TestOptions:
    def test_install_screen_shows_the_generation(self) -> None:
        unit = unit_for(ACE)
        assert unit is not None
        assert install_rows(unit, UnitSelection()) == ["ACE Unit Type : V1"]

    def test_the_generation_cycles_between_the_two(self, printer_tree: PrinterTree) -> None:
        option = Ace().install_options()[0]
        sel = UnitSelection(installation_type=ACE)
        ctx = make_ctx(printer_tree)
        assert option.activate(ctx, RecordingConsole(), ScriptedPrompter([]), sel) == (  # type: ignore
            "ACE Unit Type: V2_Pro"
        )
        assert option.activate(ctx, RecordingConsole(), ScriptedPrompter([]), sel) == (  # type: ignore
            "ACE Unit Type: V1"
        )

    @pytest.mark.parametrize("generation, default", [(V1, "Ace_1"), (V2, "Ace2_1")])
    def test_additional_screen_names_the_unit_after_its_generation(
        self, printer_tree: PrinterTree, generation: str, default: str
    ) -> None:
        unit = unit_for(ACE)
        assert unit is not None
        sel = UnitSelection(installation_type=ACE, unit_name="stale", ace_unit_type=generation)
        rows = additional_rows(unit, make_ctx(printer_tree), sel)
        assert rows == [f"ACE Name : {default}", f"ACE Unit Type : {generation}"]

    def test_the_first_unit_is_always_ace_1(self) -> None:
        assert Ace().default_name == "Ace_1"


class TestDefaultNames:
    def seed(self, tree: PrinterTree, *names: str) -> Context:
        ctx = make_ctx(tree)
        ctx.afc_config_dir.mkdir(parents=True)
        for name in names:
            (ctx.afc_config_dir / name).write_text("placeholder\n")
        return ctx

    def test_each_generation_probes_its_own_files(self, printer_tree: PrinterTree) -> None:
        ctx = self.seed(printer_tree, "AFC_Ace_1.cfg", "AFC_Ace_2.cfg", "AFC_Ace2_1.cfg")
        v1 = UnitSelection(installation_type=ACE, ace_unit_type=V1)
        v2 = UnitSelection(installation_type=ACE, ace_unit_type=V2)
        assert additional_default_name(ctx, v1) == "Ace_3"
        assert additional_default_name(ctx, v2) == "Ace2_2"

    def test_a_first_ace_2_pro_is_ace2_1_whatever_ace_pros_exist(
        self, printer_tree: PrinterTree
    ) -> None:
        ctx = self.seed(printer_tree, "AFC_Ace_1.cfg")
        sel = UnitSelection(installation_type=ACE, ace_unit_type=V2)
        assert additional_default_name(ctx, sel) == "Ace2_1"

    def test_the_generation_prefix(self) -> None:
        unit = Ace()
        assert unit.additional_prefix(UnitSelection(ace_unit_type=V1)) == "Ace"
        assert unit.additional_prefix(UnitSelection(ace_unit_type=V2)) == "Ace2"


class TestSelectionRules:
    def test_selecting_an_ace_means_a_sensor_and_the_internal_buffer(self) -> None:
        sel = UnitSelection(toolhead_sensor="Ramming", buffer_type="FPS_PSF")
        Ace().on_select(sel)
        assert (sel.toolhead_sensor, sel.buffer_type) == ("Sensor", "Internal")

    def test_ramming_is_refused(self) -> None:
        sel = UnitSelection(toolhead_sensor="Ramming", buffer_type="Internal")
        message = Ace().after_sensor_change(sel)
        assert message == "ACE requires a toolhead sensor; ramming is not supported"
        assert sel.toolhead_sensor == "Sensor"

    @pytest.mark.parametrize(
        "current, expected",
        [
            ("Internal", "TurtleNeck"),
            ("TurtleNeck", "TurtleNeckV2"),
            ("TurtleNeckV2", "TurtleNeck2Remastered"),
            ("TurtleNeck2Remastered", "FPS_PSF"),
            ("FPS_PSF", "None"),
            ("None", "Internal"),
            ("Unknown", "Internal"),
        ],
    )
    def test_the_buffer_cycle_includes_internal_and_none(self, current: str, expected: str) -> None:
        sel = UnitSelection(buffer_type=current)
        Ace().cycle_buffer(sel)
        assert sel.buffer_type == expected

    def test_additional_units_offer_the_same_buffers_with_internal_as_the_default(self) -> None:
        assert additional_buffer_supported(ACE) is True
        assert additional_buffer_default(ACE) == "Internal"
        assert additional_buffer_options(ACE) == (
            "None", "Internal", "TurtleNeck", "TurtleNeckV2", "TurtleNeck2Remastered", "FPS_PSF"
        )
