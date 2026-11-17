"""Unit tests for extras/AFC_installer.py."""

from __future__ import annotations

import errno
import io
import os
import pathlib
import signal
import subprocess
import sys
import time
from types import ModuleType, SimpleNamespace
from typing import Any, Callable, Dict, List, Optional, Tuple

import pytest

from extras import AFC_installer as mod
from extras.AFC_installer import (
    BRANCH_NAME,
    COLUMNS,
    COPY_BRANCH,
    IDLE_S,
    LOG_NAME,
    ROWS,
    STOP_GRACE_S,
    TICK_S,
    afcInstaller,
    checkout_branch,
    class_file,
    group_kwargs,
    has_scripts,
    is_checkout,
    klipper_dir,
    load_config,
    repo_dir,
    session_env,
)
from tests.bambu_helpers import BambuLogger, FakeGcmd, FakeReactor
from tests.conftest import CommandError

#: The runners make_installer made, for close_terminals.
MADE: List["afcInstaller"] = []
#: Every log line a session start leaves, by mode.
STARTED = {"update": ("info", "AFC update started (session 1)"),
           "install": ("info", "AFC install started (session 1)")}


class InstallerReactor(FakeReactor):
    """FakeReactor with file descriptor callbacks."""

    def __init__(self, now: float = 100.0) -> None:
        """:param now: the starting time"""
        super().__init__(now=now)
        self.fds: Dict[int, Callable[[float], None]] = {}

    def register_fd(self, fd: int, callback: Callable[[float], None]) -> int:
        """
        :param fd: the descriptor to watch
        :param callback: called with eventtime when it is readable
        :return int: the handle, the descriptor itself here
        """
        self.fds[fd] = callback
        return fd

    def unregister_fd(self, handle: int) -> None:
        """:param handle: what register_fd returned"""
        del self.fds[handle]


class FakeGcode:
    """Records registered commands."""

    def __init__(self) -> None:
        """Start with none."""
        self.commands: Dict[str, Tuple[Callable[[Any], None], str]] = {}

    def register_command(self, name: str, func: Callable[[Any], None], desc: str = "") -> None:
        """
        :param name: the command
        :param func: its handler
        :param desc: its help
        """
        self.commands[name] = (func, desc)


class FakePrinter:
    """The printer calls the installer runner makes."""

    def __init__(self, start_args: Dict[str, Any], afc: Any) -> None:
        """
        :param start_args: Klipper's start arguments
        :param afc: the AFC object
        """
        self.reactor = InstallerReactor()
        self.gcode = FakeGcode()
        self.start_args = start_args
        self.objects: Dict[str, Any] = {"gcode": self.gcode, "AFC": afc}
        self.handlers: Dict[str, Callable[[], None]] = {}

    def get_reactor(self) -> InstallerReactor:
        """:return InstallerReactor: the reactor"""
        return self.reactor

    def get_start_args(self) -> Dict[str, Any]:
        """:return dict: the start arguments"""
        return self.start_args

    def lookup_object(self, name: str) -> Any:
        """
        :param name: the object
        :return Any: it
        """
        return self.objects[name]

    def register_event_handler(self, event: str, callback: Callable[[], None]) -> None:
        """
        :param event: the event
        :param callback: its handler
        """
        self.handlers[event] = callback


class FakeProc:
    """A Popen result that exits when told."""

    def __init__(self, pid: int = 4321) -> None:
        """:param pid: its process id"""
        self.pid = pid
        self.code: Optional[int] = None

    def poll(self) -> Optional[int]:
        """:return Optional[int]: its exit status, None while it runs"""
        return self.code


def make_afc(printing: bool = False, paused: bool = False) -> SimpleNamespace:
    """
    :param printing: what is_printing answers
    :param paused: what is_paused answers
    :return SimpleNamespace: AFC with a logger, its print checks and Moonraker settings
    """
    function = SimpleNamespace(is_printing=lambda: printing, is_paused=lambda: paused)
    return SimpleNamespace(logger=BambuLogger(), function=function,
                           moonraker_host="http://localhost", moonraker_port=7125)


def make_repo(root: pathlib.Path, scripts: Tuple[str, ...] = ("update-afc.sh", "install-afc.sh"),
              body: str = "exit 0\n") -> pathlib.Path:
    """
    :param root: where to make it
    :param scripts: the installer scripts it holds
    :param body: each script's text
    :return pathlib.Path: an AFC checkout with an extras folder
    """
    repo = root / "AFC-Klipper-Add-On"
    (repo / "extras").mkdir(parents=True)
    for name in scripts:
        (repo / name).write_text(body)
    return repo


def make_head(repo: str, head: str) -> None:
    """
    Make a folder a git checkout.

    :param repo: the folder
    :param head: what its HEAD holds
    """
    os.makedirs(os.path.join(repo, ".git"), exist_ok=True)
    with open(os.path.join(repo, ".git", "HEAD"), "w") as f:
        f.write(head + "\n")


def afc_from(path: Optional[pathlib.Path], monkeypatch: pytest.MonkeyPatch) -> Any:
    """
    :param path: the file AFC's module was loaded from, None for a module with no file
    :param monkeypatch: to put the module in sys.modules
    :return Any: an idle AFC whose class comes from that module
    """
    module = ModuleType("afc_test_module")
    if path is not None:
        module.__file__ = str(path)
    monkeypatch.setitem(sys.modules, module.__name__, module)
    afc = type("afc", (), {"__module__": module.__name__})()
    vars(afc).update(vars(make_afc()))
    return afc


def link_afc(root: pathlib.Path, folder: pathlib.Path) -> pathlib.Path:
    """
    :param root: where Klipper's extras go
    :param folder: the AFC folder AFC.py links into
    :return pathlib.Path: AFC.py in Klipper's extras, a link to the folder's
    """
    (folder / "extras" / "AFC.py").write_text("")
    extras = root / "klipper" / "klippy" / "extras"
    extras.mkdir(parents=True)
    (extras / "AFC.py").symlink_to(folder / "extras" / "AFC.py")
    return extras / "AFC.py"


def make_installer(tmp_path: pathlib.Path, monkeypatch: pytest.MonkeyPatch,
                   afc: Optional[SimpleNamespace] = None, log: bool = True,
                   connect: bool = True, root: bool = False) -> afcInstaller:
    """
    :param tmp_path: the test's folder
    :param monkeypatch: to set the user id
    :param afc: AFC, an idle one by default
    :param log: whether Klipper was started with a log file
    :param connect: run the connect handler
    :param root: whether Klipper runs as root
    :return afcInstaller: a runner whose checkout is a fresh folder in tmp_path
    """
    monkeypatch.setattr(os, "geteuid", lambda: 0 if root else 1000)
    (tmp_path / "logs").mkdir(exist_ok=True)
    args = {"config_file": str(tmp_path / "config" / "printer.cfg")}
    if log:
        args["log_file"] = str(tmp_path / "logs" / "klippy.log")
    printer = FakePrinter(args, afc if afc is not None else make_afc())
    inst = afcInstaller(SimpleNamespace(get_printer=lambda: printer))
    MADE.append(inst)
    inst.repo = str(make_repo(tmp_path))
    inst.klipper = str(tmp_path / "klipper")
    if connect:
        printer.handlers["klippy:connect"]()
    return inst


def logged(inst: afcInstaller) -> List[Tuple[str, str]]:
    """:return list: AFC's log lines"""
    return inst.printer.objects["AFC"].logger.messages


def run_started(inst: afcInstaller, monkeypatch: pytest.MonkeyPatch) -> SimpleNamespace:
    """
    Start a session with a fake process, on a real terminal whose session side is closed
    again, as it is once only the script holds it.

    :param inst: the runner
    :param monkeypatch: to replace Popen and watch openpty
    :return SimpleNamespace: proc, the fake process, and slave, the session side's path
    """
    proc = FakeProc()
    names: List[str] = []
    real = os.openpty

    def openpty() -> Tuple[int, int]:
        master, slave = real()
        names.append(os.ttyname(slave))
        return master, slave

    monkeypatch.setattr(mod.pty, "openpty", openpty)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: proc)
    inst._start(["bash", "update-afc.sh"])
    return SimpleNamespace(proc=proc, slave=names[0])


def open_slave(started: SimpleNamespace) -> int:
    """
    :param started: what run_started returned
    :return int: the session side opened again, as the script would hold it
    """
    return os.open(started.slave, os.O_RDWR | os.O_NOCTTY)


def popen_recording(argvs: List[List[str]]) -> Callable[..., FakeProc]:
    """
    :param argvs: where each command goes
    :return Callable: a Popen that records its command and returns a FakeProc
    """
    def popen(argv: List[str], **kwargs: Any) -> FakeProc:
        argvs.append(argv)
        return FakeProc()
    return popen


@pytest.fixture
def signals(monkeypatch: pytest.MonkeyPatch) -> List[Tuple[int, int]]:
    """:return list: (pid, signal) for each os.killpg call, which sends nothing"""
    sent: List[Tuple[int, int]] = []
    monkeypatch.setattr(os, "killpg", lambda pid, signum: sent.append((pid, signum)))
    return sent


@pytest.fixture(autouse=True)
def close_terminals():
    """Close any terminal or log a test's runners left open."""
    yield
    while MADE:
        inst = MADE.pop()
        if inst._master is not None:
            os.close(inst._master)
        if inst._log is not None:
            inst._log.close()


def closed(fd: int) -> bool:
    """:return bool: True when the descriptor is not open"""
    try:
        os.fstat(fd)
    except OSError:
        return True
    return False


# repo_dir

class TestRepoDir:
    def test_follows_the_extras_link_to_the_checkout(self, tmp_path):
        repo = make_repo(tmp_path)
        real = repo / "extras" / "AFC_installer.py"
        real.write_text("")
        extras = tmp_path / "klipper" / "klippy" / "extras"
        extras.mkdir(parents=True)
        (extras / "AFC_installer.py").symlink_to(real)
        assert repo_dir(str(extras / "AFC_installer.py")) == str(repo)


# has_scripts

class TestHasScripts:
    def test_every_script_there(self, tmp_path):
        assert has_scripts(str(make_repo(tmp_path))) is True

    @pytest.mark.parametrize("held", [("update-afc.sh",), ("install-afc.sh",), ()])
    def test_a_script_missing(self, tmp_path, held):
        assert has_scripts(str(make_repo(tmp_path, scripts=held))) is False


# class_file

class TestClassFile:
    def test_the_file_its_class_was_loaded_from(self, tmp_path, monkeypatch):
        assert class_file(afc_from(tmp_path / "AFC.py", monkeypatch)) == str(tmp_path / "AFC.py")

    def test_none_for_a_module_with_no_file(self, monkeypatch):
        assert class_file(afc_from(None, monkeypatch)) is None


# klipper_dir

class TestKlipperDir:
    def test_is_the_folder_above_klippy_on_the_link_path(self, tmp_path):
        repo = make_repo(tmp_path)
        real = repo / "extras" / "AFC_installer.py"
        real.write_text("")
        extras = tmp_path / "klipper" / "klippy" / "extras"
        extras.mkdir(parents=True)
        (extras / "AFC_installer.py").symlink_to(real)
        assert klipper_dir(str(extras / "AFC_installer.py")) == str(tmp_path / "klipper")


# checkout_branch

class TestCheckoutBranch:
    def test_the_branch_head_names(self, tmp_path):
        (tmp_path / ".git").mkdir()
        (tmp_path / ".git" / "HEAD").write_text("ref: refs/heads/BridgeBox\n")
        assert checkout_branch(str(tmp_path)) == "BridgeBox"

    def test_a_worktree_reads_the_head_its_git_file_points_at(self, tmp_path):
        gitdir = tmp_path / "main" / ".git" / "worktrees" / "wt"
        gitdir.mkdir(parents=True)
        (gitdir / "HEAD").write_text("ref: refs/heads/BridgeBox-turtle\n")
        wt = tmp_path / "wt"
        wt.mkdir()
        (wt / ".git").write_text(f"gitdir: {gitdir}\n")
        assert checkout_branch(str(wt)) == "BridgeBox-turtle"

    def test_a_worktree_path_may_be_relative(self, tmp_path):
        gitdir = tmp_path / "main" / ".git" / "worktrees" / "wt"
        gitdir.mkdir(parents=True)
        (gitdir / "HEAD").write_text("ref: refs/heads/DEV\n")
        wt = tmp_path / "wt"
        wt.mkdir()
        (wt / ".git").write_text("gitdir: ../main/.git/worktrees/wt\n")
        assert checkout_branch(str(wt)) == "DEV"

    def test_none_when_head_is_detached(self, tmp_path):
        (tmp_path / ".git").mkdir()
        (tmp_path / ".git" / "HEAD").write_text("6d551cd0000000000000000000000000000000ab\n")
        assert checkout_branch(str(tmp_path)) is None

    def test_none_when_head_cannot_be_read(self, tmp_path):
        (tmp_path / ".git").mkdir()
        assert checkout_branch(str(tmp_path)) is None


# is_checkout

class TestIsCheckout:
    def test_a_git_folder(self, tmp_path):
        make_head(str(tmp_path), "ref: refs/heads/DEV")
        assert is_checkout(str(tmp_path))

    def test_a_worktrees_git_file(self, tmp_path):
        (tmp_path / ".git").write_text("gitdir: /elsewhere\n")
        assert is_checkout(str(tmp_path))

    def test_no_git(self, tmp_path):
        assert not is_checkout(str(tmp_path))


# BRANCH_NAME

class TestBranchName:
    @pytest.mark.parametrize("name", ["DEV", "BridgeBox-turtle", "feature/two_spools", "v1.2", "a",
                                      "a" * 100])
    def test_takes_a_branch(self, name):
        assert BRANCH_NAME.fullmatch(name)

    @pytest.mark.parametrize("name", ["", "-b", ".hidden", "/DEV", "DEV/", "DEV.", "a..b", "a//b",
                                      "a b", "a;b", "a#b", "a\nb", "a" * 101])
    def test_refuses_what_is_no_branch_or_would_break_the_command(self, name):
        assert not BRANCH_NAME.fullmatch(name)


# group_kwargs

class TestGroupKwargs:
    def test_process_group_from_python_3_11(self):
        assert group_kwargs((3, 11, 2)) == {"process_group": 0}

    def test_setpgrp_before_it(self):
        assert group_kwargs((3, 10, 12)) == {"preexec_fn": os.setpgrp}


# session_env

class TestSessionEnv:
    def test_sets_the_terminal_and_quiets_pagers_and_git_prompts(self):
        klipper = {"HOME": "/home/pi", "TERM": "dumb", "PATH": "/usr/bin"}
        env = session_env(klipper)
        assert env == {"HOME": "/home/pi", "PATH": "/usr/bin", "TERM": "xterm-256color",
                       "COLUMNS": str(COLUMNS), "LINES": str(ROWS), "PAGER": "cat",
                       "GIT_PAGER": "cat", "GIT_TERMINAL_PROMPT": "0"}
        assert klipper["TERM"] == "dumb"


# load_config

class TestLoadConfig:
    def test_makes_the_runner(self, tmp_path, monkeypatch):
        printer = FakePrinter({}, make_afc())
        inst = load_config(SimpleNamespace(get_printer=lambda: printer))
        assert isinstance(inst, afcInstaller)
        assert inst.printer is printer


# afcInstaller.__init__

class TestAfcInstallerInit:
    def test_paths_from_the_start_arguments_and_the_module(self, tmp_path):
        printer = FakePrinter({"config_file": "/home/pi/printer_data/config/printer.cfg",
                               "log_file": "/home/pi/printer_data/logs/klippy.log"}, make_afc())
        inst = afcInstaller(SimpleNamespace(get_printer=lambda: printer))
        assert inst.repo == repo_dir(mod.__file__)
        assert inst.klipper == klipper_dir(mod.__file__)
        assert inst.config_dir == "/home/pi/printer_data/config"
        assert inst.log_path == "/home/pi/printer_data/logs/" + LOG_NAME
        assert (inst.why, inst.mode, inst.session, inst.size, inst.result) == ("", "", 0, 0, "")
        assert (inst.git, inst.branch) == (False, "")
        assert inst.exit_code is None and inst._proc is None and inst._master is None
        assert sorted(printer.handlers) == ["klippy:connect", "klippy:disconnect"]
        assert sorted(printer.gcode.commands) == ["AFC_INSTALLER", "AFC_INSTALLER_ANSWER",
                                                  "AFC_INSTALLER_STOP"]

    def test_no_log_file_no_log_path(self):
        printer = FakePrinter({"config_file": "/c/printer.cfg"}, make_afc())
        assert afcInstaller(SimpleNamespace(get_printer=lambda: printer)).log_path == ""

    def test_no_config_file_no_config_dir(self):
        printer = FakePrinter({"config_file": None}, make_afc())
        assert afcInstaller(SimpleNamespace(get_printer=lambda: printer)).config_dir == ""


# afcInstaller._handle_connect

class TestAfcInstallerHandleConnect:
    def test_finds_afc_and_its_logger_and_says_whether_it_can_run(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, connect=False, root=True)
        afc = inst.printer.objects["AFC"]
        inst.printer.handlers["klippy:connect"]()
        assert inst.afc is afc and inst.logger is afc.logger
        assert inst.why == "Klipper runs as root here, and AFC's installer refuses root"
        assert inst.printer.reactor.timers[0].waketime == FakeReactor.NEVER
        assert logged(inst) == []

    def test_reads_the_branch_of_afcs_checkout(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, connect=False)
        make_head(inst.repo, "ref: refs/heads/BridgeBox")
        inst.printer.handlers["klippy:connect"]()
        assert (inst.git, inst.branch) == (True, "BridgeBox")

    def test_keeps_its_own_folder_when_the_scripts_are_there(self, tmp_path, monkeypatch):
        other = make_repo(tmp_path / "home")
        afc = afc_from(link_afc(tmp_path, other), monkeypatch)
        inst = make_installer(tmp_path, monkeypatch, afc=afc)
        assert inst.repo == str(tmp_path / "AFC-Klipper-Add-On") and inst.why == ""

    def test_follows_afc_py_to_the_folder_the_scripts_are_in(self, tmp_path, monkeypatch):
        folder = make_repo(tmp_path / "home")
        afc = afc_from(link_afc(tmp_path, folder), monkeypatch)
        inst = make_installer(tmp_path, monkeypatch, afc=afc, connect=False)
        for script in ("update-afc.sh", "install-afc.sh"):
            os.remove(os.path.join(inst.repo, script))
        inst.printer.handlers["klippy:connect"]()
        assert inst.repo == str(folder) and inst.why == ""

    def test_afc_py_leads_to_no_scripts_either(self, tmp_path, monkeypatch):
        folder = make_repo(tmp_path / "home", scripts=())
        afc = afc_from(link_afc(tmp_path, folder), monkeypatch)
        inst = make_installer(tmp_path, monkeypatch, afc=afc, connect=False)
        own = inst.repo
        os.remove(os.path.join(own, "install-afc.sh"))
        inst.printer.handlers["klippy:connect"]()
        assert inst.repo == own
        assert inst.why == "AFC's installer scripts are not beside AFC's Klipper files"

    def test_afc_from_no_file(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, afc=afc_from(None, monkeypatch),
                              connect=False)
        own = inst.repo
        os.remove(os.path.join(own, "update-afc.sh"))
        inst.printer.handlers["klippy:connect"]()
        assert inst.repo == own
        assert inst.why == "AFC's installer scripts are not beside AFC's Klipper files"


# afcInstaller._why_not

class TestAfcInstallerWhyNot:
    def test_root(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, root=True)
        assert inst._why_not() == "Klipper runs as root here, and AFC's installer refuses root"

    @pytest.mark.parametrize("missing", ["update-afc.sh", "install-afc.sh"])
    def test_a_script_missing(self, tmp_path, monkeypatch, missing):
        inst = make_installer(tmp_path, monkeypatch)
        os.remove(os.path.join(inst.repo, missing))
        assert inst._why_not() == "AFC's installer scripts are not beside AFC's Klipper files"

    def test_no_log_file(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, log=False)
        assert inst._why_not() == ("Klipper writes no log file here, so the installer's output "
                                   "has nowhere to go")

    def test_nothing_in_the_way(self, tmp_path, monkeypatch):
        assert make_installer(tmp_path, monkeypatch)._why_not() == ""


# afcInstaller._read_branch

class TestAfcInstallerReadBranch:
    def test_a_checkout_on_a_branch(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "ref: refs/heads/DEV")
        inst._read_branch()
        assert (inst.git, inst.branch) == (True, "DEV")

    def test_a_detached_checkout(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "6d551cd0000000000000000000000000000000ab")
        inst._read_branch()
        assert (inst.git, inst.branch) == (True, "")

    def test_a_copy_without_git_shows_main(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        inst.git, inst.branch = True, "DEV"
        inst._read_branch()
        assert (inst.git, inst.branch) == (False, COPY_BRANCH) == (False, "main")


# afcInstaller._handle_disconnect

class TestAfcInstallerHandleDisconnect:
    def test_ends_a_running_session(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        master, log = inst._master, inst._log
        inst.printer.handlers["klippy:disconnect"]()
        assert signals == [(started.proc.pid, signal.SIGTERM)]
        assert inst._proc is None and inst._master is None and inst._log is None
        assert closed(master) and log.closed
        assert inst.printer.reactor.fds == {}
        assert inst.result == "stopped"

    def test_nothing_running_nothing_to_end(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        inst.printer.handlers["klippy:disconnect"]()
        assert signals == []
        assert inst.result == ""


# afcInstaller._argv

class TestAfcInstallerArgv:
    def test_a_git_checkout_stays_on_its_branch(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "ref: refs/heads/BridgeBox")
        assert inst._argv("update") == [
            "bash", os.path.join(inst.repo, "update-afc.sh"), "-b", "BridgeBox",
            "-k", inst.klipper, "-p", str(tmp_path / "config"),
            "-y", os.path.dirname(sys.executable), "-a", "http://localhost", "-n", "7125"]

    def test_a_detached_checkout_gets_none(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "6d551cd0000000000000000000000000000000ab")
        assert inst._argv("update") is None

    def test_a_branch_asked_for_comes_first(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "ref: refs/heads/BridgeBox")
        assert inst._argv("install", "DEV")[2:4] == ["-b", "DEV"]

    def test_a_branch_asked_for_on_a_detached_checkout(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "6d551cd0000000000000000000000000000000ab")
        assert inst._argv("update", "DEV")[2:4] == ["-b", "DEV"]

    def test_a_copy_without_git_shows_main(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        assert inst._argv("install") == [
            "bash", os.path.join(inst.repo, "install-afc.sh"), "-b", "main",
            "-k", inst.klipper, "-p", str(tmp_path / "config"),
            "-y", os.path.dirname(sys.executable), "-a", "http://localhost", "-n", "7125"]

    def test_a_copy_without_git_shows_the_branch_asked_for(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        assert inst._argv("install", "BridgeBox")[2:4] == ["-b", "BridgeBox"]


# afcInstaller._start

class TestAfcInstallerStart:
    def test_runs_the_command_in_a_terminal_of_its_own(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        with open(inst.log_path, "wb") as f:
            f.write(b"the last session")
        calls: List[Tuple[tuple, dict]] = []
        proc = FakeProc()

        def popen(*args, **kwargs):
            calls.append((args, kwargs))
            return proc

        monkeypatch.setattr(subprocess, "Popen", popen)
        inst.size, inst.result, inst.exit_code, inst._stop_at = 9, "failed", 1, 5.0
        inst._start(["bash", "update-afc.sh"])
        (argv,), kwargs = calls[0]
        assert argv == ["bash", "update-afc.sh"]
        assert kwargs["cwd"] == inst.repo and "start_new_session" not in kwargs
        assert {k: kwargs[k] for k in group_kwargs(sys.version_info)} == group_kwargs(
            sys.version_info)
        assert kwargs["env"] == session_env(os.environ)
        assert kwargs["stdin"] == kwargs["stdout"] == kwargs["stderr"]
        assert closed(kwargs["stdin"])
        assert inst._proc is proc and not os.get_blocking(inst._master)
        assert inst.printer.reactor.fds == {inst._master: inst._handle_read}
        assert (inst.session, inst.size, inst.result, inst.exit_code) == (1, 0, "running", None)
        assert inst._stop_at is None and inst._active == inst.printer.reactor.now
        assert inst.printer.reactor.timers[0].waketime == FakeReactor.NOW
        assert pathlib.Path(inst.log_path).read_bytes() == b""

    def test_the_terminal_has_the_screen_size(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        sizes: List[Tuple[int, int]] = []

        def popen(*args, **kwargs):
            sizes.append(os.get_terminal_size(kwargs["stdout"]))
            return FakeProc()

        monkeypatch.setattr(subprocess, "Popen", popen)
        inst._start(["bash", "update-afc.sh"])
        assert sizes == [os.terminal_size((COLUMNS, ROWS))]

    def test_a_log_that_cannot_open_closes_the_terminal(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        inst.log_path = str(tmp_path / "missing" / LOG_NAME)
        opened: List[Tuple[int, int]] = []
        real = os.openpty

        def openpty():
            fds = real()
            opened.append(fds)
            return fds

        monkeypatch.setattr(mod.pty, "openpty", openpty)
        with pytest.raises(FileNotFoundError):
            inst._start(["bash", "update-afc.sh"])
        assert closed(opened[0][0]) and closed(opened[0][1])
        assert inst._proc is None and inst.session == 0

    def test_a_command_that_cannot_start_closes_the_log_and_terminal(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        opened: List[Tuple[int, int]] = []
        real = os.openpty
        logs: List[Any] = []
        real_open = open

        def openpty():
            fds = real()
            opened.append(fds)
            return fds

        def tracked_open(path, *args, **kwargs):
            handle = real_open(path, *args, **kwargs)
            logs.append(handle)
            return handle

        def popen(*args, **kwargs):
            raise PermissionError(errno.EACCES, "Permission denied")

        monkeypatch.setattr(mod.pty, "openpty", openpty)
        monkeypatch.setattr(mod, "open", tracked_open, raising=False)
        monkeypatch.setattr(subprocess, "Popen", popen)
        with pytest.raises(PermissionError):
            inst._start(["bash", "update-afc.sh"])
        assert closed(opened[0][0]) and closed(opened[0][1])
        assert logs[0].closed
        assert inst._proc is None and inst.session == 0


# afcInstaller._handle_read

class TestAfcInstallerHandleRead:
    def test_copies_what_the_terminal_shows(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        read_end, write_end = os.pipe()
        inst._master, inst._log = read_end, io.BytesIO()
        os.write(write_end, b"Enter your choice: ")
        inst._handle_read(150.0)
        os.close(write_end)
        assert inst._log.getvalue() == b"Enter your choice: "


# afcInstaller._read_once

class TestAfcInstallerReadOnce:
    def test_writes_what_it_read_to_the_log(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        slave = open_slave(started)
        os.write(slave, b"U. Update AFC Klipper Add-On\n")
        assert inst._read_once(150.0) is True
        os.close(slave)
        inst._log.flush()
        assert pathlib.Path(inst.log_path).read_bytes() == b"U. Update AFC Klipper Add-On\r\n"
        assert inst.size == 30 and inst._active == 150.0

    def test_nothing_waiting(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        slave = open_slave(run_started(inst, monkeypatch))
        master = inst._master
        assert inst._read_once(150.0) is False
        os.close(slave)
        assert inst._master == master and inst.size == 0 and inst._active == 100.0

    def test_the_terminal_ends_when_its_other_side_closes(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        run_started(inst, monkeypatch)
        master = inst._master
        assert inst._read_once(150.0) is False  # the session's side closed after Popen
        assert inst._master is None and closed(master)
        assert inst.printer.reactor.fds == {}

    def test_an_empty_read_ends_it_too(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        read_end, write_end = os.pipe()
        os.close(write_end)
        inst._master, inst._fd, inst._log = read_end, read_end, io.BytesIO()
        inst.printer.reactor.fds[read_end] = inst._handle_read
        assert inst._read_once(150.0) is False
        assert inst._master is None and closed(read_end) and inst._log.getvalue() == b""


# afcInstaller._close_terminal

class TestAfcInstallerCloseTerminal:
    def test_stops_watching_and_closes_it(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        run_started(inst, monkeypatch)
        master = inst._master
        inst._close_terminal()
        assert inst._master is None and inst._fd is None and closed(master)
        assert inst.printer.reactor.fds == {}

    def test_already_closed(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        inst.printer.reactor.fds[99] = inst._handle_read
        inst._close_terminal()
        assert inst.printer.reactor.fds == {99: inst._handle_read}


# afcInstaller._close_log

class TestAfcInstallerCloseLog:
    def test_closes_it(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        log = io.BytesIO()
        inst._log = log
        inst._close_log()
        assert log.closed and inst._log is None

    def test_none_open(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        inst._close_log()
        assert inst._log is None


# afcInstaller._tick

class TestAfcInstallerTick:
    def test_nothing_running(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        assert inst._tick(150.0) == FakeReactor.NEVER

    def test_a_script_that_exited_ends_the_session(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        inst.mode = "update"
        started.proc.code = 0
        assert inst._tick(150.0) == FakeReactor.NEVER
        assert inst._proc is None and inst.result == "done"
        assert signals == []
        assert logged(inst) == [("info", "AFC update ended: done (exit status 0)")]

    def test_kills_it_when_stops_grace_ran_out(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        inst._stop_at = 150.0
        assert inst._tick(150.0) == 150.0 + TICK_S
        assert signals == [(started.proc.pid, signal.SIGKILL)]

    def test_waits_out_stops_grace(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        run_started(inst, monkeypatch)
        inst._stop_at = 150.0
        inst._active = 0.0  # stopping, so not idle-stopped again
        assert inst._tick(149.0) == 149.0 + TICK_S
        assert signals == [] and inst._stop_at == 150.0

    def test_stops_a_session_nothing_happened_in(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        inst.mode = "update"
        now = inst._active + IDLE_S + 1.0
        assert inst._tick(now) == now + TICK_S
        assert signals == [(started.proc.pid, signal.SIGINT)]
        assert inst._stop_at == now + STOP_GRACE_S
        assert logged(inst) == [("info", "AFC update stopped: nothing happened in it for "
                                         "30 minutes")]

    def test_a_session_still_in_use(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        run_started(inst, monkeypatch)
        now = inst._active + IDLE_S
        assert inst._tick(now) == now + TICK_S
        assert signals == [] and inst._stop_at is None and logged(inst) == []


# afcInstaller._finish

class TestAfcInstallerFinish:
    def test_keeps_what_it_showed_last(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        inst.mode = "update"
        slave = open_slave(started)
        os.write(slave, b"Restarting Klipper service to apply changes...\n")
        os.close(slave)
        master, log = inst._master, inst._log
        inst._finish(0, 150.0)
        assert pathlib.Path(inst.log_path).read_bytes() == (b"Restarting Klipper service to "
                                                             b"apply changes...\r\n")
        assert inst._master is None and closed(master) and log.closed and inst._log is None
        assert inst._proc is None and (inst.exit_code, inst.result) == (0, "done")
        assert logged(inst) == [("info", "AFC update ended: done (exit status 0)")]

    def test_a_terminal_already_closed(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        run_started(inst, monkeypatch)
        inst.mode = "install"
        inst._close_terminal()
        inst._finish(2, 150.0)
        assert (inst.exit_code, inst.result) == (2, "failed")
        assert logged(inst) == [("info", "AFC install ended: failed (exit status 2)")]

    def test_a_stopped_session(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        run_started(inst, monkeypatch)
        inst.mode = "update"
        inst._stop_at = 140.0
        inst._finish(0, 150.0)
        assert (inst.exit_code, inst.result) == (0, "stopped")
        assert logged(inst) == [("info", "AFC update ended: stopped (exit status 0)")]

    def test_reads_the_branch_the_script_left(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "ref: refs/heads/BridgeBox")
        inst._read_branch()
        run_started(inst, monkeypatch)
        inst.mode = "update"
        make_head(inst.repo, "ref: refs/heads/DEV")
        inst._finish(0, 150.0)
        assert inst.branch == "DEV"


# afcInstaller._stop

class TestAfcInstallerStop:
    def test_interrupts_it_and_arms_the_kill(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        inst._stop(150.0)
        assert inst._stop_at == 150.0 + STOP_GRACE_S
        assert signals == [(started.proc.pid, signal.SIGINT)]


# afcInstaller._signal

class TestAfcInstallerSignal:
    def test_signals_the_whole_group(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        inst._signal(signal.SIGTERM)
        assert signals == [(started.proc.pid, signal.SIGTERM)]

    def test_a_group_already_gone(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        run_started(inst, monkeypatch)
        tried: List[int] = []

        def killpg(pid, signum):
            tried.append(signum)
            raise ProcessLookupError(errno.ESRCH, "No such process")

        monkeypatch.setattr(os, "killpg", killpg)
        inst._signal(signal.SIGKILL)
        assert tried == [signal.SIGKILL]


# afcInstaller._printing

class TestAfcInstallerPrinting:
    @pytest.mark.parametrize("printing, paused, expected", [
        (True, False, True), (False, True, True), (False, False, False)])
    def test_printing_or_paused(self, tmp_path, monkeypatch, printing, paused, expected):
        inst = make_installer(tmp_path, monkeypatch, afc=make_afc(printing, paused))
        assert inst._printing() is expected


# afcInstaller.cmd_AFC_INSTALLER

class TestAfcInstallerCmdAfcInstaller:
    def test_starts_the_updater_by_default(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        argvs: List[List[str]] = []
        monkeypatch.setattr(subprocess, "Popen", popen_recording(argvs))
        inst.cmd_AFC_INSTALLER(FakeGcmd({}))
        assert argvs == [inst._argv("update")]
        assert inst.mode == "update" and inst.result == "running"
        assert logged(inst) == [STARTED["update"]]

    def test_starts_the_installer(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        argvs: List[List[str]] = []
        monkeypatch.setattr(subprocess, "Popen", popen_recording(argvs))
        inst.cmd_AFC_INSTALLER(FakeGcmd({"MODE": "install"}))
        assert argvs == [inst._argv("install")]
        assert inst.mode == "install"
        assert logged(inst) == [STARTED["install"]]

    def test_starts_on_the_branch_asked_for(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "ref: refs/heads/BridgeBox")
        argvs: List[List[str]] = []
        monkeypatch.setattr(subprocess, "Popen", popen_recording(argvs))
        inst.cmd_AFC_INSTALLER(FakeGcmd({"MODE": "install", "BRANCH": "DEV"}))
        assert argvs == [inst._argv("install", "DEV")]
        assert logged(inst) == [("info", "AFC install started on branch DEV (session 1)")]

    def test_a_branch_name_it_cannot_take(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "ref: refs/heads/BridgeBox")
        with pytest.raises(CommandError, match="^BRANCH must be a branch name, not -x$"):
            inst.cmd_AFC_INSTALLER(FakeGcmd({"BRANCH": "-x"}))
        assert inst._proc is None and logged(inst) == []

    def test_a_copy_without_git_on_the_branch_asked_for(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        argvs: List[List[str]] = []
        monkeypatch.setattr(subprocess, "Popen", popen_recording(argvs))
        inst.cmd_AFC_INSTALLER(FakeGcmd({"BRANCH": "BridgeBox"}))
        assert argvs == [inst._argv("update", "BridgeBox")]
        assert logged(inst) == [("info", "AFC update started on branch BridgeBox (session 1)")]

    def test_an_unknown_mode(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        with pytest.raises(CommandError, match="^MODE must be UPDATE or INSTALL, not UNINSTALL$"):
            inst.cmd_AFC_INSTALLER(FakeGcmd({"MODE": "uninstall"}))
        assert inst._proc is None and logged(inst) == []

    def test_not_where_it_cannot_run(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, root=True)
        with pytest.raises(CommandError, match="^Klipper runs as root here"):
            inst.cmd_AFC_INSTALLER(FakeGcmd({}))
        assert inst._proc is None and logged(inst) == []

    def test_one_at_a_time(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        inst.mode = "install"
        with pytest.raises(CommandError, match="^AFC install is already running$"):
            inst.cmd_AFC_INSTALLER(FakeGcmd({"MODE": "update"}))
        assert inst._proc is started.proc and inst.mode == "install" and inst.session == 1

    def test_not_while_printing(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, afc=make_afc(printing=True))
        with pytest.raises(CommandError, match="^AFC's installer cannot run while printing$"):
            inst.cmd_AFC_INSTALLER(FakeGcmd({}))
        assert inst._proc is None and logged(inst) == []

    def test_not_on_a_detached_checkout(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        make_head(inst.repo, "6d551cd0000000000000000000000000000000ab")
        with pytest.raises(CommandError, match="^AFC's checkout is on no branch, and the "
                                               "installer would switch it to its default "
                                               "branch$"):
            inst.cmd_AFC_INSTALLER(FakeGcmd({}))
        assert inst._proc is None and logged(inst) == []

    def test_a_script_that_cannot_start(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)

        def popen(*args, **kwargs):
            raise FileNotFoundError(errno.ENOENT, "No such file or directory", "bash")

        monkeypatch.setattr(subprocess, "Popen", popen)
        with pytest.raises(CommandError, match="^AFC update could not start: .*No such file"):
            inst.cmd_AFC_INSTALLER(FakeGcmd({}))
        assert inst.mode == "" and inst._proc is None and logged(inst) == []


# afcInstaller.cmd_AFC_INSTALLER_ANSWER

class TestAfcInstallerCmdAfcInstallerAnswer:
    def test_sends_the_answer_and_enter(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        slave = open_slave(started)
        inst.printer.reactor.now = 160.0
        inst.cmd_AFC_INSTALLER_ANSWER(FakeGcmd({"HEX": "553b23"}))
        assert os.read(slave, 64) == b"U;#\n"
        os.close(slave)
        assert inst._active == 160.0

    def test_no_answer_is_just_enter(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        slave = open_slave(started)
        inst.cmd_AFC_INSTALLER_ANSWER(FakeGcmd({}))
        assert os.read(slave, 64) == b"\n"
        os.close(slave)

    def test_not_running(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        with pytest.raises(CommandError, match="^AFC's installer is not running$"):
            inst.cmd_AFC_INSTALLER_ANSWER(FakeGcmd({"HEX": "55"}))

    def test_not_hex(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        run_started(inst, monkeypatch)
        with pytest.raises(CommandError, match="^HEX must be the answer's bytes as hex digits$"):
            inst.cmd_AFC_INSTALLER_ANSWER(FakeGcmd({"HEX": "U"}))
        assert inst._active == 100.0

    def test_an_answer_the_terminal_did_not_take(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch)
        read_end, write_end = os.pipe()
        inst._master = read_end  # a write to a pipe's read end fails
        with pytest.raises(CommandError, match="^AFC's installer did not take the answer: "):
            inst.cmd_AFC_INSTALLER_ANSWER(FakeGcmd({"HEX": "55"}))
        os.close(write_end)
        assert inst._active == 0.0


# afcInstaller.cmd_AFC_INSTALLER_STOP

class TestAfcInstallerCmdAfcInstallerStop:
    def test_stops_the_session(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        started = run_started(inst, monkeypatch)
        inst.mode = "update"
        gcmd = FakeGcmd({})
        inst.cmd_AFC_INSTALLER_STOP(gcmd)
        assert signals == [(started.proc.pid, signal.SIGINT)]
        assert inst._stop_at == inst.printer.reactor.now + STOP_GRACE_S
        assert logged(inst) == [("info", "AFC update stopping")] and gcmd.messages == []

    def test_nothing_to_stop(self, tmp_path, monkeypatch, signals):
        inst = make_installer(tmp_path, monkeypatch)
        gcmd = FakeGcmd({})
        inst.cmd_AFC_INSTALLER_STOP(gcmd)
        assert gcmd.messages == [("respond_info", "AFC's installer is not running")]
        assert signals == [] and inst._stop_at is None and logged(inst) == []


# afcInstaller.get_status

class TestAfcInstallerGetStatus:
    def test_before_any_session(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, root=True)
        assert inst.get_status(150.0) == {
            "available": False,
            "why": "Klipper runs as root here, and AFC's installer refuses root", "git": False,
            "branch": "main", "running": False, "mode": "", "session": 0, "log": LOG_NAME,
            "size": 0, "result": "", "exit_code": None}

    def test_while_a_session_runs(self, tmp_path, monkeypatch):
        inst = make_installer(tmp_path, monkeypatch, connect=False)
        make_head(inst.repo, "ref: refs/heads/BridgeBox")
        inst.printer.handlers["klippy:connect"]()
        monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: FakeProc())
        inst.cmd_AFC_INSTALLER(FakeGcmd({}))
        inst.size = 512
        assert inst.get_status() == {
            "available": True, "why": "", "git": True, "branch": "BridgeBox", "running": True,
            "mode": "update", "session": 1, "log": LOG_NAME, "size": 512, "result": "running",
            "exit_code": None}


# A real session

class TestARealSession:
    def test_menus_answered_over_a_terminal_that_cannot_ask_for_passwords(self, tmp_path,
                                                                          monkeypatch):
        script = ("if (exec 3</dev/tty) 2>/dev/null; then echo HAS_TTY; else echo NO_TTY; fi\n"
                  "printf 'Enter your choice: '\n"
                  "read answer\n"
                  "echo \"chose $answer\"\n"
                  "exit 3\n")
        inst = make_installer(tmp_path, monkeypatch)
        for name in ("update-afc.sh", "install-afc.sh"):
            pathlib.Path(inst.repo, name).write_text(script)
        inst.cmd_AFC_INSTALLER(FakeGcmd({}))
        deadline = time.monotonic() + 10.0
        log = pathlib.Path(inst.log_path)
        while b"Enter your choice: " not in log.read_bytes() and time.monotonic() < deadline:
            inst._handle_read(inst.printer.reactor.now)
            time.sleep(0.01)
        inst.cmd_AFC_INSTALLER_ANSWER(FakeGcmd({"HEX": "U".encode().hex()}))
        while inst._proc is not None and time.monotonic() < deadline:
            inst._tick(inst.printer.reactor.now)
            time.sleep(0.01)
        text = log.read_bytes().decode()
        assert "NO_TTY" in text and "HAS_TTY" not in text
        assert "Enter your choice: U\r\nchose U\r\n" in text
        assert (inst.result, inst.exit_code, inst.size) == ("failed", 3, len(text.encode()))
        assert logged(inst) == [STARTED["update"],
                                ("info", "AFC update ended: failed (exit status 3)")]
