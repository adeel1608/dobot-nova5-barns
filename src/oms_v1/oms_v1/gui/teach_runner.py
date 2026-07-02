"""
gui/teach_runner.py

Thin orchestration layer around the patched
``ros2 run pickn_place tool_mount_teach`` and
``ros2 run pickn_place machine_mount_teach`` commands.

These nodes use ``input()`` blocking calls to drive the teach flow. The
runner launches them as subprocesses, feeds prompts via stdin, streams
stdout/stderr into the GUI trace buffer, and parses the success messages
the patched nodes emit (``Tool offset save target: <path>`` and
``Machine offset save target: <path>``) to report to the operator.

The runner intentionally does NOT modify the teach algorithms -- only the
I/O layer.
"""

from __future__ import annotations

import os
import re
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

from .trace import gui_log, gui_log_error


_SAVED_LINE = re.compile(
    r"(?:Tool offset|Machine offset|Offsets) (?:saved|save target).*?->\s*(.+)$"
)


@dataclass
class TeachSession:
    label: str
    cmd: List[str]
    proc: Optional[subprocess.Popen] = None
    log_lines: List[str] = field(default_factory=list)
    saved_paths: List[str] = field(default_factory=list)
    finished: threading.Event = field(default_factory=threading.Event)
    return_code: Optional[int] = None
    log_lock: threading.Lock = field(default_factory=threading.Lock)

    def append(self, line: str) -> None:
        with self.log_lock:
            self.log_lines.append(line)
            if len(self.log_lines) > 2000:
                del self.log_lines[:-2000]

    def tail(self, n: int = 80) -> List[str]:
        with self.log_lock:
            return list(self.log_lines[-n:])

    def is_running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None


def _start_subprocess(cmd: Sequence[str]) -> subprocess.Popen:
    env = os.environ.copy()
    return subprocess.Popen(
        list(cmd),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        text=True,
        env=env,
    )


def _spawn_reader(session: TeachSession) -> None:
    def reader() -> None:
        try:
            assert session.proc is not None
            assert session.proc.stdout is not None
            for raw in session.proc.stdout:
                line = raw.rstrip("\n")
                gui_log(f"GUI-TEACH[{session.label}]", line)
                session.append(line)
                m = _SAVED_LINE.search(line)
                if m:
                    session.saved_paths.append(m.group(1).strip())
        except Exception as exc:
            gui_log_error(
                f"GUI-TEACH[{session.label}]", "stdout reader crashed", exc
            )
        finally:
            if session.proc is not None:
                session.proc.wait()
                session.return_code = session.proc.returncode
            session.finished.set()

    threading.Thread(
        target=reader, name=f"gui-teach-{session.label}", daemon=True
    ).start()


def write_line(session: TeachSession, text: str) -> bool:
    """Send a single newline-terminated line to the teach subprocess."""
    if session.proc is None or session.proc.stdin is None:
        return False
    try:
        gui_log(f"GUI-TEACH[{session.label}]", f"<<< {text!r}")
        session.proc.stdin.write(text + "\n")
        session.proc.stdin.flush()
        return True
    except Exception as exc:
        gui_log_error(f"GUI-TEACH[{session.label}]", "stdin write failed", exc)
        return False


def wait_for_text(
    session: TeachSession,
    needle: str,
    timeout_sec: float = 60.0,
) -> bool:
    """Block until ``needle`` appears in the session's log, or timeout."""
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if not session.is_running():
            return False
        with session.log_lock:
            joined = "\n".join(session.log_lines)
        if needle in joined:
            return True
        time.sleep(0.1)
    return False


def start_tool_teach() -> TeachSession:
    cmd = ["ros2", "run", "pickn_place", "tool_mount_teach"]
    session = TeachSession(label="tool", cmd=list(cmd))
    session.proc = _start_subprocess(cmd)
    _spawn_reader(session)
    gui_log("GUI-TEACH", f"tool_mount_teach started pid={session.proc.pid}")
    return session


def start_machine_teach() -> TeachSession:
    cmd = ["ros2", "run", "pickn_place", "machine_mount_teach"]
    session = TeachSession(label="machine", cmd=list(cmd))
    session.proc = _start_subprocess(cmd)
    _spawn_reader(session)
    gui_log("GUI-TEACH", f"machine_mount_teach started pid={session.proc.pid}")
    return session


def end_tool_session(session: TeachSession, send_blank: bool = False) -> None:
    """Best-effort termination of a tool teach session."""
    if not session.is_running():
        return
    try:
        if session.proc and session.proc.stdin and not session.proc.stdin.closed:
            session.proc.stdin.close()
    except Exception:
        pass
    try:
        assert session.proc is not None
        session.proc.terminate()
    except Exception:
        pass


def end_machine_session(session: TeachSession) -> None:
    """Send blank input lines to make machine_mount_teach exit cleanly."""
    if session.is_running():
        write_line(session, "")
    end_tool_session(session)


def run_tool_teach(
    marker_name: str,
    timeout_sec: float = 180.0,
) -> TeachSession:
    """High-level helper: start the teach node, feed the marker name once,
    and wait until either a save line is emitted or the process exits.

    The returned session contains ``saved_paths`` and the full log tail for
    the GUI to display.
    """
    session = start_tool_teach()
    if not wait_for_text(session, "Enter tool tf name", timeout_sec=20.0):
        gui_log("GUI-TEACH", "tool teach: prompt not seen, sending marker name anyway")
    write_line(session, marker_name)
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if session.saved_paths or not session.is_running():
            break
        time.sleep(0.5)
    return session


def run_machine_marker_capture(
    machine_name: str,
    timeout_sec: float = 180.0,
) -> TeachSession:
    """Start ``machine_mount_teach`` and feed the machine name. After this
    returns the operator should drive subsequent prompts (point name, TCP
    frames) via ``feed_machine_point`` so each point can be retried/discarded
    individually.
    """
    session = start_machine_teach()
    if not wait_for_text(session, "Enter machine TF name", timeout_sec=20.0):
        gui_log("GUI-TEACH", "machine teach: prompt not seen, sending name anyway")
    write_line(session, machine_name)
    if not wait_for_text(session, "Machine averaged", timeout_sec=timeout_sec):
        gui_log("GUI-TEACH", "machine teach: averaging did not finish in time")
    return session


###############################################################################
# Per-step machine teach helpers.
#
# machine_mount_teach uses bare `input(prompt)` calls whose prompts have NO
# trailing newline. Our reader thread iterates `proc.stdout` line by line, so
# such prompts stay buffered in the parent's read buffer until something with
# a newline finishes the line -- typically the success log line printed AFTER
# the operator's response is processed.
#
# That means we can't reliably "wait for the prompt" before sending. Instead,
# each helper:
#   1. Sends the response immediately (the subprocess is already blocked at
#      input() because the previous step finished and the next step's input
#      call is the next thing to run).
#   2. Waits for the post-action log marker that only appears after the
#      teach node's success path runs.
#
# Markers used (all emitted via rclpy logger -> always end with a newline):
#   - per-point capture: "<point>_approach_test: offset" / "_mount_test: offset"
#   - finalize:          "Offsets saved -> ..." (one per save target)
#
# Operator can do anything in the GUI between helpers (toggle drag, move
# gripper, refresh, etc.) without affecting the teach subprocess -- it's
# just blocked at input() waiting for our next stdin write.
###############################################################################


def send_machine_point_name(
    session: TeachSession, point_name: str
) -> bool:
    """Reply to ``input("Enter point name (blank=finish): ")``.

    There is no log line emitted between this input() returning and the
    next prompt_and_sample's input() call, so we just send and return.
    The next helper (``capture_machine_approach``) will be the one that
    waits for a post-action marker.
    """
    if not session.is_running():
        gui_log_error("GUI-TEACH", "send_machine_point_name: session not running", None)
        return False
    return write_line(session, point_name)


def capture_machine_approach(
    session: TeachSession,
    point_name: str,
    tcp_frame: str = "Link6",
    timeout_sec: float = 60.0,
) -> bool:
    """Send the approach-pose TCP frame and wait for the offset log.

    The teach node calls sample_offset(...) right after input() returns,
    which prints ``<point>_approach_test: offset t=[...]`` via rclpy
    logger. That line has a newline so our reader yields it; we wait
    on it as the success marker.
    """
    if not session.is_running():
        gui_log_error("GUI-TEACH", "capture_machine_approach: session not running", None)
        return False
    if not write_line(session, tcp_frame):
        return False
    marker = f"{point_name}_approach_test: offset"
    if wait_for_text(session, marker, timeout_sec=timeout_sec):
        return True
    gui_log_error(
        "GUI-TEACH",
        f"capture_machine_approach: marker {marker!r} not seen within {timeout_sec}s",
        None,
    )
    return False


def capture_machine_mount(
    session: TeachSession,
    point_name: str,
    tcp_frame: str = "Link6",
    timeout_sec: float = 60.0,
) -> bool:
    """Send the mount-pose TCP frame and wait for the offset log."""
    if not session.is_running():
        gui_log_error("GUI-TEACH", "capture_machine_mount: session not running", None)
        return False
    if not write_line(session, tcp_frame):
        return False
    marker = f"{point_name}_mount_test: offset"
    if wait_for_text(session, marker, timeout_sec=timeout_sec):
        return True
    gui_log_error(
        "GUI-TEACH",
        f"capture_machine_mount: marker {marker!r} not seen within {timeout_sec}s",
        None,
    )
    return False


def finalize_machine_teach(
    session: TeachSession, timeout_sec: float = 120.0
) -> bool:
    """Send a blank input to end the machine teach point loop and wait for
    the YAML save lines.

    The patched machine_mount_teach emits at least
    ``Offsets saved -> <path>`` and ``Machine offset save target: <path>``
    log lines for each save destination. ``saved_paths`` accumulates them
    as the reader yields them; we declare success as soon as at least
    one path is captured (mirrors tool teach finalize behaviour).
    """
    if session.is_running():
        write_line(session, "")
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if session.saved_paths:
            return True
        if not session.is_running():
            # process exited; saved_paths may still have populated via the
            # reader-thread tail flush before set(finished) -- give it a beat.
            time.sleep(0.3)
            return bool(session.saved_paths)
        time.sleep(0.4)
    return False


def feed_machine_point(
    session: TeachSession,
    point_name: str,
    approach_tcp_frame: str = "Link6",
    mount_tcp_frame: str = "Link6",
    per_step_timeout_sec: float = 90.0,
) -> bool:
    """Legacy single-shot helper. Kept for backward compat / non-interactive
    callers; new code should use the per-step helpers above so the operator
    has a chance to physically reposition the robot between approach and
    mount captures.
    """
    if not send_machine_point_name(session, point_name):
        return False
    if not capture_machine_approach(
        session, point_name, approach_tcp_frame, timeout_sec=per_step_timeout_sec
    ):
        return False
    if not capture_machine_mount(
        session, point_name, mount_tcp_frame, timeout_sec=per_step_timeout_sec
    ):
        return False
    return True
