from pathlib import Path

import pytest

from installer.platform_core import system
from tests.installer.fakes import (
    FakeCommandRunner,
    FakeLocalPosts,
    PrinterTree,
    RecordingConsole,
    ScriptedPrompter,
    build_printer_tree,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def printer_tree(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> PrinterTree:
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    return build_printer_tree(tmp_path, REPO_ROOT)


@pytest.fixture
def console() -> RecordingConsole:
    return RecordingConsole()


@pytest.fixture
def runner() -> FakeCommandRunner:
    return FakeCommandRunner()


@pytest.fixture
def prompter() -> ScriptedPrompter:
    return ScriptedPrompter([])


@pytest.fixture(autouse=True)
def local_posts(monkeypatch: pytest.MonkeyPatch) -> FakeLocalPosts:
    """No test asks a real Moonraker or U1 page on this machine to restart anything:
    every such POST is recorded and refused unless the test sets `ok`. No test inherits
    AFC_CONFIGS_ONLY from the shell either."""
    posts = FakeLocalPosts()
    monkeypatch.setattr(system, "_post_ok", posts)
    monkeypatch.delenv("AFC_CONFIGS_ONLY", raising=False)
    return posts
