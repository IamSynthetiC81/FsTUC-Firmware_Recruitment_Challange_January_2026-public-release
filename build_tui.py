#!/usr/bin/env python3
"""
Interactive TUI for building and testing FsTUC Firmware project.
Requires: pip install textual

Supports cross-platform CMake configuration for Windows, Linux, and macOS.
"""

import subprocess
import os
import sys
import platform
import shutil
import threading
from pathlib import Path
from datetime import datetime

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import (
    Footer,
    Header,
    Label,
    ListItem,
    ListView,
    RichLog,
    Static,
)
from textual.reactive import reactive

# ---------------------------------------------------------------------------
# Platform helpers
# ---------------------------------------------------------------------------

def _detect_platform() -> str:
    """Return a short platform identifier: 'Windows', 'Linux', or 'Darwin'."""
    system = platform.system()
    if system in ("Windows", "Linux", "Darwin"):
        return system
    return "Linux"  # JPL: safe default


def _cmake_generator() -> str:
    """Choose the best CMake generator for the current host."""
    system = _detect_platform()
    if system == "Windows":
        # Prefer Ninja; fall back to MinGW Makefiles or NMake Makefiles
        if shutil.which("ninja"):
            return "Ninja"
        if shutil.which("mingw32-make") or shutil.which("mingw64-make"):
            return "MinGW Makefiles"
        return "NMake Makefiles"
    # Linux / macOS
    if shutil.which("ninja"):
        return "Ninja"
    return "Unix Makefiles"


def _exe(name: str) -> str:
    """Append .exe on Windows, nothing elsewhere."""
    return name + ".exe" if _detect_platform() == "Windows" else name


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

WORKSPACE = Path(__file__).parent.resolve()
BUILD_DIR = WORKSPACE / "build"

# Source files to watch for uncommitted changes after a build
_WATCHED_SOURCES: list[Path] = [
    WORKSPACE / "lib" / "src" / "contactor.c",
    WORKSPACE / "lib" / "src" / "BatteryPack.c",
    WORKSPACE / "lib" / "src" / "FSM.c",
]

# Build artefacts whose mtime marks the last successful build
_BUILD_STAMPS: list[str] = [
    "CMakeCache.txt",
    "test_contactor",
    "test_contactor.exe",
    "test_battery_pack",
    "test_battery_pack.exe",
    "test_adc",
    "test_eeprom",
]

# ---------------------------------------------------------------------------
# Build-info helpers
# ---------------------------------------------------------------------------

def _last_build_time() -> datetime | None:
    """
    Return the mtime of the newest build artefact, or None if the build
    directory does not exist / no known artefact is present.
    """
    best: float | None = None
    for name in _BUILD_STAMPS:
        p = BUILD_DIR / name
        if p.exists():
            t = p.stat().st_mtime
            if best is None or t > best:
                best = t
    return datetime.fromtimestamp(best) if best is not None else None


def _dirty_sources(since: datetime | None) -> list[str]:
    """
    Return a list of source-file names whose mtime is newer than *since*.
    If *since* is None (no build yet), all watched files are considered dirty.
    """
    dirty: list[str] = []
    for src in _WATCHED_SOURCES:
        if not src.exists():
            continue
        src_mtime = datetime.fromtimestamp(src.stat().st_mtime)
        if since is None or src_mtime > since:
            dirty.append(src.name)
    return dirty


# ---------------------------------------------------------------------------
# Task definitions
# ---------------------------------------------------------------------------


def _make_tasks() -> list:
    """Build the ordered list of actions the TUI exposes."""
    generator = _cmake_generator()

    configure_cmd = [
        "cmake", "-B", str(BUILD_DIR),
        "-G", generator,
    ]
    build_cmd = ["cmake", "--build", str(BUILD_DIR), "--config", "Debug"]
    rebuild_steps = [
        ["cmake", "--build", str(BUILD_DIR), "--target", "clean"],
        ["cmake", "-B", str(BUILD_DIR), "-G", generator],
        ["cmake", "--build", str(BUILD_DIR), "--config", "Debug"],
    ]
    clean_cmd = ["cmake", "--build", str(BUILD_DIR), "--target", "clean"]
    ctest_cmd = ["ctest", "--output-on-failure", "--parallel", "4"]

    contactor_bin = str(BUILD_DIR / _exe("test_contactor"))
    batterypack_bin = str(BUILD_DIR / _exe("test_battery_pack"))

    return [
        {
            "label": "Setup CMake",
            "description": f"Configure build system  [{generator}]",
            "steps": [configure_cmd],
            "cwd": str(WORKSPACE),
        },
        {
            "label": "Build",
            "description": "Compile all targets",
            "steps": [build_cmd],
            "cwd": str(WORKSPACE),
        },
        {
            "label": "Rebuild  (clean + build)",
            "description": "Full clean, reconfigure, and build",
            "steps": rebuild_steps,
            "cwd": str(WORKSPACE),
        },
        {
            "label": "Clean",
            "description": "Remove compiled artifacts",
            "steps": [clean_cmd],
            "cwd": str(WORKSPACE),
        },
        {
            "label": "Run Contactor Tests",
            "description": "Execute test_contactor binary",
            "steps": [[contactor_bin]],
            "cwd": str(BUILD_DIR),
        },
        {
            "label": "Run BatteryPack Tests",
            "description": "Execute test_battery_pack binary",
            "steps": [[batterypack_bin]],
            "cwd": str(BUILD_DIR),
        },
        {
            "label": "Run All Tests",
            "description": "Run full test suite with CTest",
            "steps": [ctest_cmd],
            "cwd": str(BUILD_DIR),
        },
    ]


# ---------------------------------------------------------------------------
# CSS
# ---------------------------------------------------------------------------

CSS = """
Screen {
    background: $surface;
}

#sidebar {
    width: 44;
    border: solid $primary;
    padding: 0 1;
}

#sidebar-title {
    text-align: center;
    background: $primary;
    color: $text;
    padding: 0 1;
    margin-bottom: 1;
}

#platform-label {
    text-align: center;
    color: $text-muted;
    margin-bottom: 1;
}

#build-info {
    border: solid $primary-darken-2;
    padding: 0 1;
    margin-bottom: 1;
    color: $text-muted;
    height: auto;
}

ListView {
    background: transparent;
    border: none;
}

ListItem {
    padding: 0 1;
}

#output-pane {
    border: solid $accent;
    padding: 0 1;
}

#output-title {
    text-align: center;
    background: $accent;
    color: $text;
    padding: 0 1;
    margin-bottom: 1;
}

#status-bar {
    height: 1;
    background: $panel;
    padding: 0 1;
    color: $text-muted;
}
"""


# ---------------------------------------------------------------------------
# Main App
# ---------------------------------------------------------------------------

class BuildTUI(App):
    """FsTUC Firmware interactive build and test TUI."""

    TITLE = "FsTUC Firmware — Build & Test"
    CSS = CSS

    BINDINGS = [
        Binding("q", "quit", "Quit", show=True),
    ]

    def __init__(self) -> None:
        super().__init__()
        self._tasks: list = _make_tasks()
        self._running: bool = False
        # Guard against spurious ListView.Selected events that Textual 8.x
        # fires during the compose phase before the UI is ready.
        self._app_ready: bool = False
        
    # ------------------------------------------------------------------
    # Compose
    # ------------------------------------------------------------------

    def compose(self) -> ComposeResult:
        """Build the TUI widget tree."""
        yield Header(show_clock=True)

        with Horizontal():
            with Vertical(id="sidebar"):
                yield Static("  Actions", id="sidebar-title")
                yield Static(
                    f"  {_detect_platform()}  |  {_cmake_generator()}",
                    id="platform-label",
                )
                yield Static("", id="build-info")
                items = [ListItem(Label(t["label"])) for t in self._tasks]
                yield ListView(*items)

            with Vertical(id="output-pane"):
                yield Static("  Output", id="output-title")
                yield RichLog(highlight=True, markup=True, wrap=True, id="log")

        yield Static("", id="status-bar")
        yield Footer()

    def on_mount(self) -> None:
        """Set initial focus, print welcome banner, and refresh build info."""
        self.query_one(ListView).focus()
        # Schedule _app_ready after the first full render cycle so any
        # compose-phase Selected events (Textual 8.x) are discarded first.
        self._running = False  # reset in case compose fired a spurious Selected
        self.call_after_refresh(self._mark_ready)
        log: RichLog = self.query_one("#log", RichLog)
        log.write(
            f"[bold cyan]FsTUC Firmware Build TUI[/bold cyan]  "
            f"[dim]{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}[/dim]\n"
        )
        log.write(
            f"[dim]Workspace : {WORKSPACE}[/dim]\n"
            f"[dim]Build dir : {BUILD_DIR}[/dim]\n"
            f"[dim]Platform  : {_detect_platform()}[/dim]\n"
            f"[dim]Generator : {_cmake_generator()}[/dim]\n"
        )
        log.write(
            "[dim]Select an action and press [bold]Enter[/bold] to run.[/dim]\n"
        )
        self._refresh_build_info(log)

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------

    def _mark_ready(self) -> None:
        """Called after the first render; enables task execution."""
        self._app_ready = True

    def _set_status(self, msg: str) -> None:
        """Update the one-line status bar."""
        self.query_one("#status-bar", Static).update(msg)

    def _refresh_build_info(self, log: RichLog | None = None) -> None:
        """
        Recompute last-build time and dirty-file list, update the sidebar
        widget, and optionally emit a warning to the log.
        """
        last = _last_build_time()
        dirty = _dirty_sources(last)

        if last is None:
            time_line = "[dim]Last build : [yellow]never[/yellow][/dim]"
        else:
            time_line = (
                f"[dim]Last build : "
                f"[cyan]{last.strftime('%Y-%m-%d %H:%M:%S')}[/cyan][/dim]"
            )

        if dirty:
            dirty_line = (
                "[bold yellow]⚠ Modified since build:[/bold yellow]\n"
                + "\n".join(f"  [yellow]• {f}[/yellow]" for f in dirty)
            )
        else:
            dirty_line = "[dim green]✓ Sources up to date[/dim green]"

        self.query_one("#build-info", Static).update(
            f"{time_line}\n{dirty_line}"
        )

        if log is not None and dirty:
            log.write(
                "[bold yellow]⚠  Source files changed since last build:[/bold yellow]\n"
                + "".join(f"   [yellow]• {f}[/yellow]\n" for f in dirty)
            )

    # ------------------------------------------------------------------
    # Enter key: fired by ListView when the user presses Enter
    # ------------------------------------------------------------------

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        """Execute the highlighted task when Enter is pressed in the list."""
        event.stop()
        # Ignore events fired during the Textual compose/mount phase.
        if not self._app_ready:
            return
        lv: ListView = self.query_one(ListView)
        idx = lv.index
        if idx is None or idx < 0 or idx >= len(self._tasks):
            return
        if self._running:
            self._set_status(
                "[yellow]A task is already running — please wait.[/yellow]"
            )
            return
        self._run_task_async(self._tasks[idx])

    def _run_task_async(self, task: dict) -> None:
        """Launch the task in a daemon thread so the TUI stays responsive.

        The RichLog widget reference is resolved here on the main thread;
        Textual DOM queries are not safe to call from worker threads.
        A try/finally guarantees _running is cleared if thread startup fails.
        """
        self._running = True
        try:
            log: RichLog = self.query_one("#log", RichLog)
            thread = threading.Thread(
                target=self._run_task, args=(task, log), daemon=True
            )
            thread.start()
        except Exception:  # noqa: BLE001
            self._running = False
            raise

    def _run_task(self, task: dict, log: RichLog) -> None:
        """Execute all command steps, streaming output to the log widget.

        A try/finally guarantees _running is cleared even if the thread
        exits unexpectedly (e.g. an unhandled exception in a step).
        """
        label: str = task["label"]
        steps: list = task["steps"]

        try:
            start = datetime.now()
            self.call_from_thread(self._set_status, f"Running: {label} …")
            self.call_from_thread(
                log.write,
                f"\n[bold yellow]{'─' * 60}[/bold yellow]\n"
                f"[bold cyan]▶ {label}[/bold cyan]  "
                f"[dim]{start.strftime('%H:%M:%S')}[/dim]\n"
                f"[dim]{task['description']}[/dim]\n",
            )

            overall_ok = True
            for step_idx, cmd in enumerate(steps, 1):
                if len(steps) > 1:
                    self.call_from_thread(
                        log.write,
                        f"[dim]Step {step_idx}/{len(steps)}: "
                        f"{' '.join(str(c) for c in cmd)}[/dim]\n",
                    )
                ok = self._stream_command(cmd, task["cwd"], log)
                if not ok:
                    overall_ok = False
                    break  # JPL: abort pipeline on first failure

            elapsed = (datetime.now() - start).total_seconds()
            if overall_ok:
                result_line = (
                    f"[bold green]✓ PASSED[/bold green]  {label}  "
                    f"[dim]({elapsed:.1f} s)[/dim]"
                )
                status_line = f"[green]✓ Done: {label}  ({elapsed:.1f} s)[/green]"
            else:
                result_line = (
                    f"[bold red]✗ FAILED[/bold red]  {label}  "
                    f"[dim]({elapsed:.1f} s)[/dim]"
                )
                status_line = f"[red]✗ Failed: {label}  ({elapsed:.1f} s)[/red]"

            self.call_from_thread(log.write, result_line + "\n")
            self.call_from_thread(self._set_status, status_line)
            self.call_from_thread(self._refresh_build_info)

        except Exception as exc:  # noqa: BLE001 — catch-all safety net
            self.call_from_thread(
                log.write,
                f"[bold red]Internal error in task thread: {exc}[/bold red]\n",
            )
            self.call_from_thread(
                self._set_status,
                f"[red]Error running {label}: {exc}[/red]",
            )
        finally:
            self._running = False

    def _stream_command(
        self,
        cmd: list,
        cwd: str,
        log: RichLog,
    ) -> bool:
        """
        Run one command, streaming every output line to the RichLog.

        Returns True on exit-code 0, False otherwise.
        """
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError:
            self.call_from_thread(
                log.write,
                f"[red]Error: executable not found — {cmd[0]}[/red]\n",
            )
            return False
        except OSError as exc:
            self.call_from_thread(
                log.write,
                f"[red]OS Error: {exc}[/red]\n",
            )
            return False

        assert proc.stdout is not None
        for line in proc.stdout:
            self.call_from_thread(log.write, line.rstrip("\n"))

        proc.wait()
        return proc.returncode == 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """Entry point — also callable as ``python build_tui.py``."""
    app = BuildTUI()
    app.run()


if __name__ == "__main__":
    main()
