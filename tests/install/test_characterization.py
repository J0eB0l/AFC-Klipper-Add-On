"""Characterization tests for the AFC installer's unit-handling bash functions.

These tests do NOT assert anything about "correct" behavior. They run the
installer's per-unit-type functions (copy_unit_files, install_additional_unit,
get_unit_buffer_target, build_install_message, name_additional_unit) through
tests/install/run_case.sh for a matrix of unit types/board/motor/buffer
variants, and diff the output against committed fixtures in
tests/install/fixtures/.

If a fixture ever needs to change, that must be a deliberate, reviewed behavior
change -- not a side effect of refactoring -- regenerate it with:

    python3 tests/install/test_characterization.py --regenerate
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUN_CASE = Path(__file__).resolve().parent / "run_case.sh"
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"

BUFFER_TYPES = ["TurtleNeck", "TurtleNeckV2", "FPS_PSF", "None"]

UNIT_VARIANTS: dict[str, list[dict[str, str]]] = {
    "BoxTurtle (4-Lane)": [{}],
    "BoxTurtle (8-Lane)": [{}],
    "NightOwl": [{}],
    "ViViD": [{}],
    "OpenAMS": [{}],
    "BridgeBox": [{}],
    "HTLF": [{"htlf_board_type": b} for b in ["ERB", "MMB_1.1", "MMB_1.0"]],
    "Claymore": [{"htlf2_board_type": "AFC_Lite"}],
    "QuattroBox": [
        {"qb_motor_type": m, "qb_board_type": b}
        for m in ["NEMA_17", "NEMA_14"]
        for b in ["MMB_1.0", "MMB_1.1", "MMB_2.0"]
    ],
    "EMU": [
        {"emu_num_lanes": str(n), "emu_board_type": bt}
        for n in (1, 3, 8)
        for bt in ("EBB", "SLB")
    ],
}

# The name_additional_unit() default per type (mirrors unit_functions.sh).
ADDITIONAL_UNIT_DEFAULT_NAME = {
    "BoxTurtle (4-Lane)": "Turtle_2",
    "BoxTurtle (8-Lane)": "Turtle_2",
    "NightOwl": "NightOwl_1",
    "ViViD": "Vivid_1",
    "QuattroBox": "QuattroBox_1",
    "OpenAMS": "AMS_1",
    "BridgeBox": "chain2",
    "EMU": "EMU_1",
    "HTLF": "HTLF_1",
    "Claymore": "Claymore_1",
}


def slug(s: str) -> str:
    return (
        s.replace(" ", "_")
        .replace("(", "")
        .replace(")", "")
        .replace("-", "")
        .replace(".", "")
    )


def run_case(mode: str, env_overrides: dict[str, str]) -> str:
    env = os.environ.copy()
    env["MODE"] = mode
    for k, v in env_overrides.items():
        env[f"CASE_{k}"] = v
    result = subprocess.run(
        ["bash", str(RUN_CASE)],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"run_case.sh failed (mode={mode}, overrides={env_overrides}):\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    return result.stdout


def iter_cases():
    for itype, variants in UNIT_VARIANTS.items():
        for variant in variants:
            variant_suffix = "__".join(f"{k}-{v}" for k, v in sorted(variant.items()))
            base_name = slug(itype) + (f"__{variant_suffix}" if variant_suffix else "")

            # copy_unit_files: default boxturtle_name (as constants.sh sets it)
            # and a custom one, to exercise the per-type renaming quirks.
            for name_label, name_value in (("defaultname", "Turtle_1"), ("customname", "CustomName1")):
                overrides = {"installation_type": itype, "boxturtle_name": name_value, **variant}
                yield f"copy_unit_files__{base_name}__{name_label}", "copy_unit_files", overrides

            # install_additional_unit: type-specific default name and a custom one.
            default_add_name = ADDITIONAL_UNIT_DEFAULT_NAME[itype]
            for name_label, name_value in (
                ("defaultname", default_add_name),
                ("customname", "CustomName2"),
            ):
                overrides = {"installation_type": itype, "boxturtle_name": name_value, **variant}
                yield f"install_additional_unit__{base_name}__{name_label}", "install_additional_unit", overrides

            # get_unit_buffer_target: doesn't depend on buffer_type, one case per variant.
            overrides = {"installation_type": itype, "boxturtle_name": "Turtle_1", **variant}
            yield f"buffer_target__{base_name}", "buffer_target", overrides

            # install_menu.sh's type-specific option rows: doesn't depend on
            # turtle_renamed (that only matters for the additional-unit menu).
            overrides = {"installation_type": itype, "boxturtle_name": "Foo_1", **variant}
            yield f"install_menu_options__{base_name}", "install_menu_options", overrides

            # additional_system_menu.sh's name + option row(s): depends on
            # turtle_renamed (whether a prior rename sticks) and can mutate
            # boxturtle_name as a side effect (see e.g. NightOwl/Claymore/EMU).
            for renamed in ("True", "False"):
                overrides = {
                    "installation_type": itype,
                    "boxturtle_name": "Foo_1",
                    "turtle_renamed": renamed,
                    **variant,
                }
                yield (
                    f"additional_menu_row__{base_name}__renamed-{renamed}",
                    "additional_menu_row",
                    overrides,
                )

            # build_install_message: depends on buffer_type too.
            for buffer_type in BUFFER_TYPES:
                overrides = {
                    "installation_type": itype,
                    "boxturtle_name": "Turtle_1",
                    "buffer_type": buffer_type,
                    **variant,
                }
                yield f"message__{base_name}__buffer-{buffer_type}", "message", overrides

        # name_additional_unit: only depends on installation_type.
        yield f"name_additional_unit__{slug(itype)}", "name_additional_unit", {"installation_type": itype}


def all_cases():
    return list(iter_cases())


def fixture_path(case_name: str) -> Path:
    return FIXTURES_DIR / f"{case_name}.txt"


def regenerate():
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    for case_name, mode, overrides in all_cases():
        output = run_case(mode, overrides)
        fixture_path(case_name).write_text(output)
        print(f"wrote {case_name}")


def _pytest_cases():
    return [(name, mode, overrides) for name, mode, overrides in all_cases()]


try:
    import pytest

    @pytest.mark.parametrize(
        "case_name,mode,overrides",
        _pytest_cases(),
        ids=[c[0] for c in _pytest_cases()],
    )
    def test_characterization(case_name, mode, overrides):
        expected_path = fixture_path(case_name)
        assert expected_path.exists(), f"missing fixture {expected_path}; run with --regenerate first"
        expected = expected_path.read_text()
        actual = run_case(mode, overrides)
        assert actual == expected

    def test_installation_options_matches_registry_order():
        # installation_options is now derived from the registry's
        # UNIT_TYPE_ORDER rather than a literal array in constants.sh --
        # pin its content and order so that derivation can't silently drift.
        actual = run_case("installation_options", {}).splitlines()
        assert actual == [
            "BoxTurtle (4-Lane)",
            "BoxTurtle (8-Lane)",
            "NightOwl",
            "HTLF",
            "Claymore",
            "QuattroBox",
            "OpenAMS",
            "BridgeBox",
            "ViViD",
            "EMU",
        ]

    def test_no_adapter_dispatchers_survive_under_set_e():
        # install-afc.sh runs under `set -e` and calls each unit_* dispatcher
        # (and print_unit_art) as a bare statement. A type with no registered
        # adapter must make the dispatcher return 0, the same way an
        # unmatched branch in the old if/elif chains fell through with
        # nothing to do -- otherwise the whole script dies mid-menu the
        # first time someone adds a type that's missing one adapter (see
        # registry.sh's "how to add a new unit type" steps 3-4).
        actual = run_case("no_adapter_survival", {})
        assert actual == "still alive\n"

    @pytest.mark.parametrize("mode", ["copy_unit_files", "install_additional_unit"])
    def test_bridgebox_chain_gets_the_unit_counts(mode):
        actual = run_case(mode, {
            "installation_type": "BridgeBox",
            "boxturtle_name": "chain2",
            "bridgebox_pool_ams": "2",
            "bridgebox_pool_ht": "1",
        })
        assert "\npool_ams: 2\npool_ht: 1\n" in actual

    @pytest.mark.parametrize("answers,ams,ht,message", [
        ("2\\n1\\n", "2", "1", "Units on this chain: 2 AMS, 1 AMS HT"),
        ("\\n\\n", "4", "8", "Units on this chain: 4 AMS, 8 AMS HT"),
        ("5\\n1\\n", "4", "8", "Enter 0 to 4 AMS units and 0 to 8 AMS HT units."),
        ("0\\n0\\n", "4", "8", "A chain needs at least one AMS or AMS HT."),
    ])
    def test_bridgebox_ask_pool(answers, ams, ht, message):
        lines = run_case("bridgebox_ask_pool", {"answers": answers}).splitlines()
        assert lines[:2] == [f"bridgebox_pool_ams={ams}", f"bridgebox_pool_ht={ht}"]
        assert message in lines[2]

except ImportError:
    pass


if __name__ == "__main__":
    if "--regenerate" in sys.argv:
        regenerate()
    else:
        print("Use --regenerate to (re)generate fixtures, or run under pytest to compare.")
