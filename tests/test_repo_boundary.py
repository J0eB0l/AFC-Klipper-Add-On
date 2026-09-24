"""Tests of the repository layout itself."""

from __future__ import annotations

import ast
import importlib
import pathlib
import re
from typing import Any, Dict, List, Optional, Tuple

import pytest

from extras import AFC_BambuAMS as unit_mod, AFC_BambuAMS_bridge as bridge_mod
from extras.AFC_BambuAMS import _BambuBufferChip, afcBambuAMS
from tests.bambu_helpers import (
    BambuConfig,
    FakeBridge,
    make_bambu_unit,
    make_bridgebox,
    make_printer,
    use_bridges,
)


class TestRepoLayout:
    """The release boundary and the name-joined seams between modules."""

    ROOT = pathlib.Path(__file__).resolve().parents[1]

    #: The firmware and flashing-tool tree. Anything naming this path is on
    #: the wrong side of the release boundary if it also ships.
    PRIVATE = "Firmwares"

    #: Firmwares/ tools whose own tests may reach across: module name ->
    #: its path under the repo root.
    TOOLS = {"ams_flash": pathlib.Path(PRIVATE, "Bambu_AMS", "ams_flash.py")}

    HT_UID = "0123456789ABCDEF00003331"
    HT_UNIT = "Bambu_AMS_HT_1"
    CHAIN_PORT = "/dev/serial/by-id/usb-chain1-if00"

    # ── helpers ──────────────────────────────────────────────────────────────

    @classmethod
    def _depends_on_private(cls, path: pathlib.Path) -> bool:
        """
        Whether this file depends on Firmwares/, as opposed to mentioning it.

        This guard itself is skipped, as it names the tree to look for it.
        Comments and docstrings do not count: a released module explaining
        what it talks to is not one that cannot be built without it. Imports
        and string literals do, because those break once the tree is gone.

        :param path: a Python file
        :return bool: True when it imports from or names a path in Firmwares/
        """
        if (path.name == "test_repo_boundary.py"
            or path.resolve() == pathlib.Path(__file__).resolve()):
            return False
        try:
            tree = ast.parse(path.read_text(errors="replace"))
        except SyntaxError:
            return False
        docstrings = set()
        for node in ast.walk(tree):
            if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                                 ast.AsyncFunctionDef)):
                body = node.body
                if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                    docstrings.add(id(body[0].value))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                if any(cls.PRIVATE in alias.name for alias in node.names):
                    return True
            elif isinstance(node, ast.ImportFrom):
                if cls.PRIVATE in (node.module or ""):
                    return True
            elif isinstance(node, ast.Constant) and isinstance(node.value, str):
                if id(node) not in docstrings and cls.PRIVATE in node.value:
                    return True
        return False

    @classmethod
    def _crossing_tests(cls) -> List[str]:
        """:return list: names of the test files that depend on Firmwares/"""
        return sorted(p.name for p in (cls.ROOT / "tests").glob("*.py")
                      if cls._depends_on_private(p))

    @classmethod
    def _tool_of(cls, test_name: str) -> Optional[str]:
        """
        The tool's tests are named for it: ``test_<tool>``
        or ``test_<tool>_<topic>``.

        :param test_name: a test file's name
        :return str: the Firmwares/ tool it tests, None for none
        """
        stem = pathlib.Path(test_name).stem
        for tool in cls.TOOLS:
            if re.fullmatch(rf"test_{re.escape(tool)}(_\w+)?", stem):
                return tool
        return None

    class _Pins:
        """Klipper's pin registry: the chips registered with it, by name."""

        def __init__(self) -> None:
            self.chips: Dict[str, Any] = {}

        def register_chip(self, name: str, chip: Any) -> None:
            """
            :param name: the chip's name
            :param chip: the chip
            """
            self.chips[name] = chip

    class _Bridge(FakeBridge):
        """BambuBridge as ready and the scout build it, recording the start."""

        def __init__(self, opener: Any, reactor: Any, logger: Any) -> None:
            """
            :param opener: opens the bridge's port
            :param reactor: the reactor it runs on
            :param logger: where it logs
            """
            super().__init__()
            self.starts: List[bool] = []
            self.narration: List[Tuple[str, str]] = []

        def start(self, defer_open: bool = False) -> None:
            """:param defer_open: keep retrying a port that is not up yet"""
            self.starts.append(defer_open)

        def set_narration_log(self, log_dir: str, tag: str) -> None:
            """
            :param log_dir: the directory of klippy's log
            :param tag: the narration file's tag
            """
            self.narration.append((log_dir, tag))

    @staticmethod
    def _pool_unit(name: str, printer: Any, port: str) -> afcBambuAMS:
        """
        :param name: the unit's name
        :param printer: the printer it loads on
        :param port: its serial_port
        :return afcBambuAMS: an idle pool unit on ``port`` with no bridge yet
        """
        return make_bambu_unit(name, printer=printer, offline=True,
                               values={"serial_port": port, "pool": True})

    # ── the release boundary ─────────────────────────────────────────────────

    def test_no_shipped_module_references_the_firmware_tree(
            self, tmp_path: pathlib.Path) -> None:
        # The scan has to see each way a module can depend on the tree, and
        # not a docstring that only explains it, or it passes vacuously.
        planted = {
            "imports.py": "import Firmwares.Bambu_AMS.ams_flash\n",
            "from_import.py": "from Firmwares.Bambu_AMS import ams_flash\n",
            "literal.py": "TOOL = 'Firmwares/Bambu_AMS/ams_flash.py'\n",
            "docstring.py": '"""Talks to what Firmwares/ flashes."""\n'
                            "# Firmwares/ in a comment\nX = 1\n",
            "broken.py": "Firmwares = (\n"}
        for name, text in planted.items():
            (tmp_path / name).write_text(text)
        flagged = sorted(name for name in planted
                         if self._depends_on_private(tmp_path / name))
        assert flagged == ["from_import.py", "imports.py", "literal.py"]

        offenders = sorted(p.relative_to(self.ROOT).as_posix()
                           for p in (self.ROOT / "extras").glob("*.py")
                           if self._depends_on_private(p))
        assert offenders == []

    def test_only_the_known_tests_cross_the_boundary(self) -> None:
        # Tests may cross, but only those of a Firmwares/ tool, named for it:
        # test_<tool> or test_<tool>_<topic>.
        names = {"test_ams_flash.py": "ams_flash",
                 "test_ams_flash_addressing.py": "ams_flash",
                 "test_AFC_BambuAMS.py": None,
                 "test_bambu_ams_flash_shim.py": None,
                 "test_not_ams_flash_related.py": None,
                 "test_ams_flashing.py": None}
        assert {name: self._tool_of(name) for name in names} == names

        crossing = self._crossing_tests()
        unexpected = [name for name in crossing if self._tool_of(name) is None]
        assert unexpected == []

    def test_the_allowlist_is_not_stale(self) -> None:
        # An allowance nothing uses would hide the next test that starts
        # crossing, so each tool must still exist and its tests still cross.
        crossing = self._crossing_tests()
        for tool, path in self.TOOLS.items():
            assert (self.ROOT / path).is_file(), path
            assert [n for n in crossing if self._tool_of(n) == tool] != [], tool

    # ── seams AFC_BridgeBox reaches by name ──────────────────────────────────

    def test_the_fabricated_section_prefix_resolves_to_the_module_checked(
            self, tmp_path: pathlib.Path) -> None:
        # klippy maps [<prefix> name] to extras/<prefix>.py by filename, so
        # the prefix BridgeBox writes decides which module reads its keys.
        master = make_bridgebox(tmp_path, buffer="Bamb_1")
        loaded = dict(master.printer.loaded)
        assert list(loaded) == [f"AFC_BambuAMS {self.HT_UNIT}",
                                "AFC_lane lane24",
                                f"AFC_hub {self.HT_UNIT}",
                                f"temperature_sensor {self.HT_UNIT}"]
        section = f"AFC_BambuAMS {self.HT_UNIT}"
        prefix = section.split()[0]
        module = importlib.import_module(f"extras.{prefix}")
        assert module is unit_mod
        assert (pathlib.Path(module.__file__)
                == self.ROOT / "extras" / "AFC_BambuAMS.py")

        # The wrapper klippy received; built on BambuConfig for its getters.
        fileconfig = loaded[section].fileconfig
        config = BambuConfig(section, master.printer,
                             dict(fileconfig.items(section)), fileconfig)
        unit = module.load_config_prefix(config)
        assert type(unit) is afcBambuAMS
        assert unit.name == self.HT_UNIT
        assert unit.unit_uid == self.HT_UID
        assert unit.ams_model == "ht"
        assert unit.serial_port == self.CHAIN_PORT
        assert unit.pool is True
        assert unit.logger.messages == []

    def test_private_symbols_bridgebox_reaches_for_still_exist(
            self, tmp_path: pathlib.Path,
            monkeypatch: pytest.MonkeyPatch) -> None:
        # Each is reached by name inside a best-effort path, so a rename
        # would degrade to "no bridge", "no chip" or "defaults" unraised.
        for sub in ("scout", "chain"):
            (tmp_path / sub).mkdir()
        printer = make_printer()
        pins = self._Pins()
        printer.add_object("pins", pins)
        scout = make_bridgebox(tmp_path / "scout", printer=printer, roster="")
        chip = printer._bambu_buffer_chips["bambu_buffer"]
        assert list(printer._bambu_buffer_chips) == ["bambu_buffer"]
        assert type(chip) is _BambuBufferChip
        assert pins.chips == {"bambu_buffer": chip}
        assert chip._unit.scout_stub is True
        assert chip._unit.fps_buffer_value() is None
        assert scout.logger.messages == []

        master = make_bridgebox(tmp_path / "chain", buffer="Bamb_1")
        bridge = FakeBridge()
        use_bridges(monkeypatch, {self.CHAIN_PORT: bridge})
        assert master._chain_bridge() is bridge

        unit = make_bambu_unit("Bambu_AMS_1", printer=master.printer,
                               model="ht", offline=True)
        assert unit.afc_bowden_length != 1234.0
        master._state_set({f"AFC_BridgeBox chain1 learned {self.HT_UID}":
                           {"afc_bowden_length": "1234.0"}})
        master.printer.afc.logger.messages.clear()
        master._apply_learned(unit, self.HT_UID, "ht")
        assert unit.afc_bowden_length == 1234.0
        assert unit.afc_unload_bowden_length == 1234.0
        assert unit._path_wait_load is True
        assert master.logger.messages == [
            ("info", f"AFC_BridgeBox chain1: Bambu_AMS_1 takes the "
                     f"afc_bowden_length 1234mm that UID {self.HT_UID} learned.")]

    def test_the_bridge_registry_has_exactly_one_home(
            self, tmp_path: pathlib.Path,
            monkeypatch: pytest.MonkeyPatch) -> None:
        # A site holding its own binding of the table would not see a rebind
        # of AFC_BambuAMS_bridge._BRIDGES: every store and read must see it.
        # raising=True: a renamed registry fails here instead of being added.
        table: Dict[str, Any] = {}
        monkeypatch.setattr(bridge_mod, "_BRIDGES", table)
        monkeypatch.setattr(unit_mod, "BambuBridge", self._Bridge)
        monkeypatch.setattr(bridge_mod, "BambuBridge", self._Bridge)
        port = "/dev/serial/by-id/usb-chain2-if00"
        master = make_bridgebox(tmp_path, buffer="Bamb_1")
        assert table == {}
        assert unit_mod._bridge_log_tag(port) == ""
        assert master._chain_bridge() is None

        # BridgeBox's scout stores the bridge it opens; the units' read sees it.
        master.logger.messages.clear()
        opened = master._ensure_bridge()
        assert type(opened) is self._Bridge
        assert table == {self.CHAIN_PORT: opened}
        assert master._chain_bridge() is opened
        assert opened.starts == [False]
        assert master.logger.messages == []

        # The first pool unit's ready stores the bridge it brings up.
        monkeypatch.setitem(master.printer.start_args, "log_file",
                            str(tmp_path / "klippy.log"))
        owner = self._pool_unit("Bambu_AMS_1", master.printer, port)
        owner.logger.messages.clear()
        owner._handle_ready()
        bridge = owner._bridge
        assert type(bridge) is self._Bridge
        assert table == {self.CHAIN_PORT: opened, port: bridge}
        assert bridge.starts == [False]
        assert bridge.sent == [{"cmd": "status"}]
        assert bridge.narration == [(str(tmp_path), "usb-chain2-if00")]
        assert owner.logger.messages == [
            ("info", "AFC bambu Bambu_AMS_1: brought the bridge up as the pool "
                     "owner on /dev/serial/by-id/usb-chain2-if00 "
                     "(variant=auto); units claim onto it live.")]

        # A second pool unit's ready reuses it rather than opening another.
        spare = self._pool_unit("Bambu_AMS_2", master.printer, port)
        spare.logger.messages.clear()
        spare._handle_ready()
        assert spare._bridge is bridge
        assert table == {self.CHAIN_PORT: opened, port: bridge}
        assert bridge.starts == [False]
        assert spare.logger.messages == []

        # A claim on a unit with no bridge wired looks it up there too.
        late = self._pool_unit("Bambu_AMS_3", master.printer, port)
        late.logger.messages.clear()
        assert late.claim(self.HT_UID, "ht") is True
        assert late._bridge is bridge
        assert table == {self.CHAIN_PORT: opened, port: bridge}
        assert late.logger.messages == [
            ("debug", f"AFC bambu Bambu_AMS_3: chain index not resolved yet "
                      f"(UID {self.HT_UID}); holding this unit's "
                      f"registrations until the chain map arrives"),
            ("info", f"AFC bambu Bambu_AMS_3: claimed UID {self.HT_UID} as ht "
                     f"and brought online live (ams_index=0).")]

        assert unit_mod._bridge_log_tag(port) == "usb-chain2-if00"
        owner.logger.messages.clear()
        owner._handle_disconnect()
        assert table == {self.CHAIN_PORT: opened}
        assert bridge.stopped is True
        assert owner._bridge is None
        assert owner.logger.messages == []
