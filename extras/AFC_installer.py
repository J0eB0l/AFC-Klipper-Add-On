# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""
AFC's installer and updater, run on the printer for a panel such as OrcaSlicer's Update page: the
script runs in a terminal of its own, what it shows goes to AFC_installer.log beside Klipper's log,
and the panel sends the answers to its menus back with AFC_INSTALLER_ANSWER.

The script runs in a process group of its own inside Klipper's session, which has no controlling
terminal and which it cannot take one into, since only a session's leader can. So nothing in it
can ask for a password: a sudo that wants one fails instead of waiting, and no password ever
passes through G-code.
"""
from __future__ import annotations

import fcntl
import os
import pty
import signal
import struct
import subprocess
import sys
import termios
from typing import IO, Any, Dict, List, Mapping, Optional, Tuple

LOG_NAME = "AFC_installer.log"
#: MODE -> the script beside AFC's extras that it runs.
SCRIPTS = {"update": "update-afc.sh", "install": "install-afc.sh"}
#: The terminal's size, as the installer's screens are drawn for it.
ROWS = 40
COLUMNS = 100
TICK_S = 0.5
#: A session that shows nothing and gets no answer this long is stopped.
IDLE_S = 1800.0
#: How long Stop waits after its interrupt before the session is killed.
STOP_GRACE_S = 5.0
READ_SIZE = 65536


def repo_dir(module_file: str) -> str:
    """
    The AFC checkout a module file belongs to: Klipper's extras hold a link to each AFC extra.

    :param module_file: the module's `__file__`
    :return str: the folder above the real file's extras folder
    """
    return os.path.dirname(os.path.dirname(os.path.realpath(module_file)))


def klipper_dir(module_file: str) -> str:
    """
    The Klipper checkout a module was loaded from, through the link in its extras.

    :param module_file: the module's `__file__`
    :return str: the folder above klippy
    """
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(module_file))))


def checkout_branch(repo: str) -> Optional[str]:
    """
    The branch a git checkout is on, read from its HEAD, also for a worktree.

    :param repo: the checkout
    :return Optional[str]: the branch, None when HEAD is detached or cannot be read
    """
    git = os.path.join(repo, ".git")
    try:
        if os.path.isfile(git):
            with open(git) as f:
                git = os.path.join(repo, f.read().strip().split("gitdir:", 1)[-1].strip())
        with open(os.path.join(git, "HEAD")) as f:
            head = f.read().strip()
    except OSError:
        return None
    prefix = "ref: refs/heads/"
    return head[len(prefix):] if head.startswith(prefix) else None


def group_kwargs(version: Tuple[int, ...]) -> Dict[str, Any]:
    """
    Popen's arguments that put a session in a process group of its own, still in Klipper's
    session: a new session would make bash its leader, and bash takes its terminal as the
    session's controlling terminal, where sudo could ask for a password.

    :param version: Python's version
    :return Dict[str, Any]: process_group from Python 3.11, setpgrp in the child before it
    """
    if version >= (3, 11):
        return {"process_group": 0}
    return {"preexec_fn": os.setpgrp}


def session_env(environ: Mapping[str, str]) -> Dict[str, str]:
    """
    The installer's environment: Klipper's, with the terminal's size and no pager or git
    password prompt to wait on.

    :param environ: Klipper's environment
    :return Dict[str, str]: the environment the script runs with
    """
    env = dict(environ)
    env.update(TERM="xterm-256color", COLUMNS=str(COLUMNS), LINES=str(ROWS), PAGER="cat",
               GIT_PAGER="cat", GIT_TERMINAL_PROMPT="0")
    return env


class afcInstaller:
    """
    Runs one installer or updater session at a time for a panel to drive.
    """

    def __init__(self, config: Any) -> None:
        """
        :param config: the [AFC_installer] section AFC loads
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.afc: Any = None
        self.logger: Any = None
        self.repo = repo_dir(__file__)
        self.klipper = klipper_dir(__file__)
        start_args = self.printer.get_start_args()
        self.config_dir = os.path.dirname(start_args.get("config_file") or "")
        log_file = start_args.get("log_file")
        self.log_path = os.path.join(os.path.dirname(log_file), LOG_NAME) if log_file else ""
        #: Why no session can start here, empty when one can.
        self.why = ""
        self.mode = ""
        #: Sessions started since Klipper started, so a panel can tell one from the next.
        self.session = 0
        #: Bytes the running or last session wrote to its log.
        self.size = 0
        #: "", running, done, failed or stopped.
        self.result = ""
        self.exit_code: Optional[int] = None
        self._proc: Optional[subprocess.Popen] = None
        self._master: Optional[int] = None
        self._fd: Any = None
        self._log: Optional[IO[bytes]] = None
        self._timer: Any = None
        self._active = 0.0
        self._stop_at: Optional[float] = None
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("AFC_INSTALLER", self.cmd_AFC_INSTALLER,
                               desc="Start AFC's updater or installer for a panel to answer: "
                                    "AFC_INSTALLER MODE=<UPDATE|INSTALL>")
        gcode.register_command("AFC_INSTALLER_ANSWER", self.cmd_AFC_INSTALLER_ANSWER,
                               desc="Answer the running AFC installer: "
                                    "AFC_INSTALLER_ANSWER HEX=<answer as hex>")
        gcode.register_command("AFC_INSTALLER_STOP", self.cmd_AFC_INSTALLER_STOP,
                               desc="Stop the running AFC installer")

    def _handle_connect(self) -> None:
        """
        Look up AFC and decide whether a session can run here.
        """
        self.afc = self.printer.lookup_object("AFC")
        self.logger = self.afc.logger
        self._timer = self.reactor.register_timer(self._tick, self.reactor.NEVER)
        self.why = self._why_not()

    def _why_not(self) -> str:
        """
        Why no session can start on this printer.

        :return str: the reason, empty when one can
        """
        if os.geteuid() == 0:
            return "Klipper runs as root here, and AFC's installer refuses root"
        if not all(os.path.isfile(os.path.join(self.repo, s)) for s in SCRIPTS.values()):
            return "AFC's installer scripts are not beside AFC's Klipper files"
        if not self.log_path:
            return "Klipper writes no log file here, so the installer's output has nowhere to go"
        return ""

    def _handle_disconnect(self) -> None:
        """
        End a running session with Klipper: nothing would read its terminal after this.
        """
        if self._proc is not None:
            self._signal(signal.SIGTERM)
            self._close_terminal()
            self._close_log()
            self._proc = None
            self.result = "stopped"

    def _argv(self, mode: str) -> Optional[List[str]]:
        """
        The command a session runs: the mode's script on the branch the checkout is on, with
        Klipper's own folders and AFC's Moonraker.

        :param mode: update or install
        :return Optional[List[str]]: the command, None when the checkout is on no branch, since
            the installer would then switch it to its default branch
        """
        argv = ["bash", os.path.join(self.repo, SCRIPTS[mode])]
        if os.path.exists(os.path.join(self.repo, ".git")):
            branch = checkout_branch(self.repo)
            if branch is None:
                return None
            argv += ["-b", branch]
        return argv + ["-k", self.klipper, "-p", self.config_dir,
                       "-y", os.path.dirname(sys.executable),
                       "-a", str(self.afc.moonraker_host), "-n", str(self.afc.moonraker_port)]

    def _start(self, argv: List[str]) -> None:
        """
        Start a session: the command in a new terminal of its own, its log truncated.

        :param argv: the command
        :raises OSError: when the terminal, the log or the command cannot be opened
        """
        master, slave = pty.openpty()
        try:
            log = open(self.log_path, "wb")
            try:
                fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", ROWS, COLUMNS, 0, 0))
                proc = subprocess.Popen(argv, cwd=self.repo, stdin=slave, stdout=slave,
                                        stderr=slave, env=session_env(os.environ),
                                        **group_kwargs(sys.version_info))
            except OSError:
                log.close()
                raise
        except OSError:
            os.close(master)
            raise
        finally:
            os.close(slave)
        os.set_blocking(master, False)
        self._proc, self._master, self._log = proc, master, log
        self._fd = self.reactor.register_fd(master, self._handle_read)
        self.session += 1
        self.size = 0
        self.result = "running"
        self.exit_code = None
        self._stop_at = None
        self._active = self.reactor.monotonic()
        self.reactor.update_timer(self._timer, self.reactor.NOW)

    def _handle_read(self, eventtime: float) -> None:
        """
        Copy what the installer shows to its log.

        :param eventtime: reactor time
        """
        self._read_once(eventtime)

    def _read_once(self, eventtime: float) -> bool:
        """
        Read the terminal once; at its end, close it.

        :param eventtime: reactor time
        :return bool: True when something was read
        """
        try:
            data = os.read(self._master, READ_SIZE)
        except BlockingIOError:
            return False
        except OSError:
            data = b""
        if not data:
            self._close_terminal()
            return False
        self._log.write(data)
        self._log.flush()
        self.size += len(data)
        self._active = eventtime
        return True

    def _close_terminal(self) -> None:
        """
        Stop reading the terminal and close it.
        """
        if self._master is not None:
            self.reactor.unregister_fd(self._fd)
            os.close(self._master)
            self._master = None
            self._fd = None

    def _close_log(self) -> None:
        """
        Close the session's log.
        """
        if self._log is not None:
            self._log.close()
            self._log = None

    def _tick(self, eventtime: float) -> float:
        """
        Watch the running session: end it when its script exits, kill it when Stop's grace runs
        out, stop it when nothing happened in it for IDLE_S.

        :param eventtime: reactor time
        :return float: when to look again
        """
        if self._proc is None:
            return self.reactor.NEVER
        code = self._proc.poll()
        if code is not None:
            self._finish(code, eventtime)
            return self.reactor.NEVER
        if self._stop_at is not None:
            if eventtime >= self._stop_at:
                self._signal(signal.SIGKILL)
        elif eventtime - self._active > IDLE_S:
            self.logger.info(f"AFC {self.mode} stopped: nothing happened in it for "
                             f"{IDLE_S / 60:.0f} minutes")
            self._stop(eventtime)
        return eventtime + TICK_S

    def _finish(self, code: int, eventtime: float) -> None:
        """
        End a session whose script exited: keep what it showed last, then close up.

        :param code: its exit status
        :param eventtime: reactor time
        """
        while self._master is not None and self._read_once(eventtime):
            pass
        self._close_terminal()
        self._close_log()
        self._proc = None
        self.exit_code = code
        if self._stop_at is not None:
            self.result = "stopped"
        else:
            self.result = "done" if code == 0 else "failed"
        self.logger.info(f"AFC {self.mode} ended: {self.result} (exit status {code})")

    def _stop(self, eventtime: float) -> None:
        """
        Interrupt the running session, as CTRL+C would, and kill it after STOP_GRACE_S.

        :param eventtime: reactor time
        """
        self._stop_at = eventtime + STOP_GRACE_S
        self._signal(signal.SIGINT)

    def _signal(self, signum: int) -> None:
        """
        Signal the session's whole process group.

        :param signum: the signal
        """
        try:
            os.killpg(self._proc.pid, signum)
        except ProcessLookupError:
            pass

    def _printing(self) -> bool:
        """
        Whether a print runs or waits paused.

        :return bool: True while printing or paused
        """
        return self.afc.function.is_printing() or self.afc.function.is_paused()

    def cmd_AFC_INSTALLER(self, gcmd: Any) -> None:
        """
        Start AFC's updater or installer for a panel to answer.

        It runs on the branch AFC's checkout is on, with Klipper's own folders, and writes what it
        shows to AFC_installer.log beside Klipper's log. One at a time, and not while printing.

        Usage
        -------
        `AFC_INSTALLER MODE=<UPDATE|INSTALL>`

        Example
        -------
        ```
        AFC_INSTALLER MODE=UPDATE
        ```
        """
        mode = gcmd.get("MODE", "UPDATE").lower()
        error_str = ""
        if mode not in SCRIPTS:
            error_str = f"MODE must be UPDATE or INSTALL, not {mode.upper()}"
        elif self.why:
            error_str = self.why
        elif self._proc is not None:
            error_str = f"AFC {self.mode} is already running"
        elif self._printing():
            error_str = "AFC's installer cannot run while printing"
        if error_str:
            raise gcmd.error(error_str)
        argv = self._argv(mode)
        if argv is None:
            error_str = ("AFC's checkout is on no branch, and the installer would switch it to "
                         "its default branch")
            raise gcmd.error(error_str)
        try:
            self._start(argv)
        except OSError as e:
            error_str = f"AFC {mode} could not start: {e}"
            raise gcmd.error(error_str)
        self.mode = mode
        self.logger.info(f"AFC {mode} started (session {self.session})")

    def cmd_AFC_INSTALLER_ANSWER(self, gcmd: Any) -> None:
        """
        Answer the running AFC installer's question or menu: the answer and Enter.

        The answer comes as hex, since G-code drops everything after a ; or # and a menu answer
        may hold either.

        Usage
        -------
        `AFC_INSTALLER_ANSWER HEX=<the answer's UTF-8 bytes as hex>`

        Example
        -------
        ```
        AFC_INSTALLER_ANSWER HEX=55
        ```
        """
        if self._master is None:
            error_str = "AFC's installer is not running"
            raise gcmd.error(error_str)
        try:
            answer = bytes.fromhex(gcmd.get("HEX", ""))
        except ValueError:
            error_str = "HEX must be the answer's bytes as hex digits"
            raise gcmd.error(error_str)
        try:
            os.write(self._master, answer + b"\n")
        except OSError as e:
            error_str = f"AFC's installer did not take the answer: {e}"
            raise gcmd.error(error_str)
        self._active = self.reactor.monotonic()

    def cmd_AFC_INSTALLER_STOP(self, gcmd: Any) -> None:
        """
        Stop the running AFC installer: interrupted as CTRL+C would, killed if it has not ended
        5 seconds later.

        Usage
        -------
        `AFC_INSTALLER_STOP`

        Example
        -------
        ```
        AFC_INSTALLER_STOP
        ```
        """
        if self._proc is None:
            gcmd.respond_info("AFC's installer is not running")
            return
        self._stop(self.reactor.monotonic())
        self.logger.info(f"AFC {self.mode} stopping")

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        The session for a panel: whether one can start, the running or last one's state, and how
        much of its log is written.

        :param eventtime: reactor time
        :return Dict[str, Any]: available, why, running, mode, session, log (the file beside
            Klipper's log), size, result and exit_code
        """
        return {"available": not self.why, "why": self.why, "running": self._proc is not None,
                "mode": self.mode, "session": self.session, "log": LOG_NAME, "size": self.size,
                "result": self.result, "exit_code": self.exit_code}


def load_config(config: Any) -> afcInstaller:
    """
    :param config: the [AFC_installer] section AFC loads
    :return afcInstaller: the printer's installer runner
    """
    return afcInstaller(config)
