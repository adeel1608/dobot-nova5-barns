"""
gui/process_manager.py

Light-weight wrapper around the long-lived ROS2 processes that the marker
training flows expect to be alive (camera + perception + visualization).

The manager will:
    - start each process if it is not already running
    - reuse running processes that match (it greps `pgrep -f` so it does not
      stomp on processes started by ``robot1-startup.sh``)
    - tee stdout/stderr into the GUI trace buffer
    - terminate only the processes it itself started, on explicit operator
      request

It does NOT kill anything during normal exit; the parent BARNS startup
script is responsible for the production lifecycle.
"""

from __future__ import annotations

import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .trace import gui_log, gui_log_error


CAMERA_PROCESS_KEY = "orbbec_camera"
ARUCO_PROCESS_KEY = "aruco_perception"
RVIZ_PROCESS_KEY = "rviz2"


@dataclass
class ManagedProcess:
    name: str
    cmd: List[str]
    proc: Optional[subprocess.Popen] = None
    reader_thread: Optional[threading.Thread] = None
    started_by_us: bool = False
    last_lines: List[str] = field(default_factory=list)
    last_lines_lock: threading.Lock = field(default_factory=threading.Lock)


def _pgrep_running(pattern: str) -> bool:
    try:
        out = subprocess.run(
            ["pgrep", "-f", pattern],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
        )
    except Exception:
        return False
    return out.returncode == 0 and out.stdout.strip() != b""


def _which(program: str) -> Optional[str]:
    return shutil.which(program)


class ProcessManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._procs: Dict[str, ManagedProcess] = {}

    def _spawn(self, key: str, cmd: List[str]) -> ManagedProcess:
        gui_log("GUI-PROC", f"spawn {key}: {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                text=True,
            )
        except FileNotFoundError as exc:
            gui_log_error("GUI-PROC", f"{key} executable not found", exc)
            raise

        managed = ManagedProcess(
            name=key, cmd=list(cmd), proc=proc, started_by_us=True
        )

        def reader() -> None:
            try:
                if proc.stdout is None:
                    return
                for line in proc.stdout:
                    line = line.rstrip("\n")
                    gui_log(f"GUI-PROC[{key}]", line)
                    with managed.last_lines_lock:
                        managed.last_lines.append(line)
                        if len(managed.last_lines) > 500:
                            del managed.last_lines[:-500]
            except Exception as exc:
                gui_log_error(f"GUI-PROC[{key}]", "stdout reader crashed", exc)

        managed.reader_thread = threading.Thread(
            target=reader, name=f"gui-proc-{key}", daemon=True
        )
        managed.reader_thread.start()
        return managed

    def ensure(
        self, key: str, cmd: Sequence[str], match_pattern: Optional[str] = None
    ) -> ManagedProcess:
        match_pattern = match_pattern or key
        with self._lock:
            existing = self._procs.get(key)
            if existing and existing.proc and existing.proc.poll() is None:
                return existing
            if _pgrep_running(match_pattern):
                gui_log(
                    "GUI-PROC",
                    f"{key} already running externally (pgrep matched {match_pattern!r}), reusing",
                )
                managed = ManagedProcess(
                    name=key, cmd=list(cmd), proc=None, started_by_us=False
                )
                self._procs[key] = managed
                return managed
            managed = self._spawn(key, list(cmd))
            self._procs[key] = managed
            return managed

    def ensure_camera(self) -> ManagedProcess:
        cmd = [
            "ros2", "launch", "orbbec_camera",
            "gemini_330_series.launch.py", "depth_registration:=true",
            "__log_level:=info",
        ]
        return self.ensure(CAMERA_PROCESS_KEY, cmd, match_pattern="orbbec_camera")

    def ensure_perception(self, visualization: bool = True) -> ManagedProcess:
        cmd = ["ros2", "run", "pickn_place", "aruco_perception"]
        if visualization:
            cmd += ["--ros-args", "-p", "visualize:=true"]
        cmd += ["__log_level:=info"]
        return self.ensure(
            ARUCO_PROCESS_KEY, cmd, match_pattern="aruco_perception"
        )

    def ensure_rviz(self, config_path: Optional[str] = None) -> Optional[ManagedProcess]:
        if _which("rviz2") is None:
            gui_log("GUI-PROC", "rviz2 not installed, skipping visualization spawn")
            return None
        cmd = ["rviz2"]
        if config_path:
            cmd += ["-d", config_path]
        return self.ensure(RVIZ_PROCESS_KEY, cmd, match_pattern="rviz2")

    def status(self) -> Dict[str, Dict[str, object]]:
        with self._lock:
            out: Dict[str, Dict[str, object]] = {}
            for key, mp in self._procs.items():
                running = False
                if mp.proc is not None:
                    running = mp.proc.poll() is None
                else:
                    running = _pgrep_running(key)
                with mp.last_lines_lock:
                    tail = list(mp.last_lines[-25:])
                out[key] = {
                    "running": running,
                    "started_by_us": mp.started_by_us,
                    "cmd": mp.cmd,
                    "tail": tail,
                }
            return out

    def stop_owned(self, key: str, timeout_sec: float = 10.0) -> None:
        with self._lock:
            mp = self._procs.get(key)
            if mp is None or mp.proc is None or not mp.started_by_us:
                return
            if mp.proc.poll() is not None:
                return
            try:
                mp.proc.terminate()
            except Exception as exc:
                gui_log_error("GUI-PROC", f"terminate {key} raised", exc)
                return
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if mp.proc.poll() is not None:
                gui_log("GUI-PROC", f"{key} stopped cleanly")
                return
            time.sleep(0.2)
        try:
            mp.proc.kill()
            gui_log("GUI-PROC", f"{key} killed after timeout")
        except Exception as exc:
            gui_log_error("GUI-PROC", f"kill {key} raised", exc)


_singleton: Optional[ProcessManager] = None
_singleton_lock = threading.Lock()


def get_process_manager() -> ProcessManager:
    global _singleton
    if _singleton is None:
        with _singleton_lock:
            if _singleton is None:
                _singleton = ProcessManager()
    return _singleton
