"""
gui/robot_bridge.py

Hybrid client used by the production-point GUI:

    1. Direct ROS2 path -- imports oms_v1.manipulate_node.run_skill and calls it
       in-process. This is the same path the sequences themselves use, so any
       drag/gripper/joint readback produced through this bridge is identical
       to what a running sequence sees.

    2. RabbitMQ fallback -- when rclpy or oms_v1.manipulate_node is not
       importable in the current environment (e.g. the GUI is running outside
       the robot container), the bridge talks to the existing oms_v1.app
       service via shared.rabbitmq_client.RabbitMQClient.execute_action.

The bridge intentionally exposes a small, stable API that hides which
transport is active so the Streamlit layer does not branch on it.
"""

from __future__ import annotations

import asyncio
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from .trace import gui_log, gui_log_error


_DIRECT_BACKEND = "direct"
_RABBITMQ_BACKEND = "rabbitmq"
_UNAVAILABLE_BACKEND = "unavailable"


def _try_import_direct() -> Optional[Callable[..., Any]]:
    """Return oms_v1.manipulate_node.run_skill if importable, else None."""
    try:
        from oms_v1.manipulate_node import run_skill  # type: ignore
    except Exception as exc:
        gui_log("GUI-BRIDGE", f"direct backend unavailable: {type(exc).__name__}: {exc}")
        return None
    return run_skill


class _RabbitMQRunner:
    """Owns a background asyncio loop so the synchronous Streamlit thread can
    issue execute_action calls without re-creating the RabbitMQClient on each
    call."""

    def __init__(self, robot_id: int) -> None:
        self.robot_id = robot_id
        self.service_name = f"robot_container_{robot_id}"
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._client: Any = None
        self._ready = threading.Event()
        self._start_lock = threading.Lock()
        self._available: Optional[bool] = None

    def _start_loop(self) -> None:
        if self._loop is not None:
            return
        with self._start_lock:
            if self._loop is not None:
                return

            def runner() -> None:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                self._loop = loop
                self._ready.set()
                try:
                    loop.run_forever()
                finally:
                    loop.close()

            self._loop_thread = threading.Thread(
                target=runner, name="oms-gui-rmq", daemon=True
            )
            self._loop_thread.start()
            self._ready.wait(timeout=5.0)

    async def _ensure_client(self) -> Any:
        if self._client is not None:
            return self._client
        try:
            from shared.rabbitmq_client import RabbitMQClient  # type: ignore
        except Exception as exc:
            gui_log_error("GUI-BRIDGE", "shared.rabbitmq_client not importable", exc)
            raise
        client = RabbitMQClient(f"oms_gui_{os.getpid()}")
        await client.connect()
        self._client = client
        return client

    def is_available(self) -> bool:
        if self._available is not None:
            return self._available
        try:
            from shared.rabbitmq_client import RabbitMQClient  # noqa: F401
        except Exception:
            self._available = False
            return False
        self._available = True
        return True

    def execute_action(
        self,
        action_name: str,
        params: Dict[str, Any] | None = None,
        timeout_sec: float = 30.0,
    ) -> Dict[str, Any]:
        if not self.is_available():
            return {
                "success": False,
                "error": "rabbitmq client not importable in this environment",
            }
        self._start_loop()
        if self._loop is None:
            return {"success": False, "error": "rabbitmq loop did not start"}

        async def _call() -> Dict[str, Any]:
            client = await self._ensure_client()
            return await client.execute_action(
                self.service_name,
                action_name,
                params or {},
                timeout=timeout_sec,
            )

        future = asyncio.run_coroutine_threadsafe(_call(), self._loop)
        try:
            result = future.result(timeout=timeout_sec + 5.0)
        except Exception as exc:
            gui_log_error(
                "GUI-BRIDGE", f"rabbitmq execute_action {action_name} failed", exc
            )
            return {"success": False, "error": f"{type(exc).__name__}: {exc}"}
        if not isinstance(result, dict):
            return {"success": True, "result": result}
        return result


class RobotBridge:
    """Singleton-like façade used by the Streamlit layer.

    All GUI code goes through this object so the transport (direct vs
    RabbitMQ) stays a runtime detail.
    """

    def __init__(self, robot_id: Optional[int] = None) -> None:
        self.robot_id = int(
            robot_id if robot_id is not None else os.getenv("ROBOT_ID", "1")
        )
        self._direct_run_skill: Optional[Callable[..., Any]] = _try_import_direct()
        self._rmq = _RabbitMQRunner(self.robot_id)
        self._control_lock = threading.Lock()
        self._last_state_lock = threading.Lock()
        self._last_state: Dict[str, Any] = {
            "joints": None,
            "pose": None,
            "joints_stamp": 0.0,
            "pose_stamp": 0.0,
            "drag_enabled": None,
            "gripper": None,
        }
        gui_log(
            "GUI-BRIDGE",
            f"initialized robot_id={self.robot_id} backend={self.backend()}",
        )

    def backend(self) -> str:
        if self._direct_run_skill is not None:
            return _DIRECT_BACKEND
        if self._rmq.is_available():
            return _RABBITMQ_BACKEND
        return _UNAVAILABLE_BACKEND

    def is_direct(self) -> bool:
        return self._direct_run_skill is not None

    def reset_direct_backend(self) -> None:
        """Re-attempt the direct import. Useful after sourcing the workspace."""
        self._direct_run_skill = _try_import_direct()

    def _ok(self, result: Any) -> bool:
        return result not in (False, None)

    def _direct(self, *args, **kwargs) -> Any:
        if self._direct_run_skill is None:
            raise RuntimeError("direct backend not available")
        return self._direct_run_skill(*args, **kwargs)

    def _begin_control(self, name: str) -> bool:
        acquired = self._control_lock.acquire(blocking=False)
        if not acquired:
            gui_log("GUI-BRIDGE", f"{name} rejected: robot control already running")
        return acquired

    def _end_control(self) -> None:
        try:
            self._control_lock.release()
        except RuntimeError:
            pass

    def call_skill(
        self,
        skill: str,
        *args: Any,
        timeout_sec: float = 30.0,
    ) -> Tuple[bool, Any]:
        """Run an arbitrary manipulate_node skill via whichever backend is up."""
        gui_log("GUI-BRIDGE", f"call_skill {skill} args={args}")
        if self._direct_run_skill is not None:
            try:
                result = self._direct(skill, *args)
                return self._ok(result), result
            except Exception as exc:
                gui_log_error("GUI-BRIDGE", f"direct {skill} raised", exc)
                return False, {"error": str(exc)}

        action_name = self._skill_to_action(skill)
        if action_name is None:
            return False, {
                "error": f"skill {skill!r} has no rabbitmq fallback action"
            }
        params = self._skill_to_params(skill, args)
        resp = self._rmq.execute_action(action_name, params, timeout_sec=timeout_sec)
        ok = bool(resp.get("success"))
        return ok, resp

    def _skill_to_action(self, skill: str) -> Optional[str]:
        mapping = {
            "toggle_drag_mode": "toggle_drag_mode",
            "open_gripper": "open_gripper",
            "close_gripper": "close_gripper",
        }
        return mapping.get(skill)

    def _skill_to_params(self, skill: str, args: Tuple[Any, ...]) -> Dict[str, Any]:
        return {}

    def _ros2_service_call_result(
        self,
        service: str,
        srv_type: str,
        args_yaml: str = "{}",
        timeout_sec: float = 15.0,
    ) -> Tuple[bool, str]:
        """Invoke a dobot_bringup_v3 service via ``ros2 service call``."""
        import shutil
        import subprocess

        if shutil.which("ros2") is None:
            gui_log_error("GUI-BRIDGE", "ros2 CLI not on PATH", None)
            return False, "ros2 CLI not on PATH"
        cmd = ["ros2", "service", "call", service, srv_type, args_yaml]
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_sec,
                text=True,
            )
        except subprocess.TimeoutExpired as exc:
            gui_log_error(
                "GUI-BRIDGE", f"{service} timed out after {timeout_sec}s", exc
            )
            return False, f"timeout after {timeout_sec}s"
        except Exception as exc:
            gui_log_error("GUI-BRIDGE", f"{service} raised", exc)
            return False, f"{type(exc).__name__}: {exc}"
        if result.returncode != 0:
            stderr = (result.stderr or "").strip()
            gui_log(
                "GUI-BRIDGE",
                f"{service} exit={result.returncode} stderr={stderr[:200]}",
            )
            return False, stderr or f"exit={result.returncode}"
        return True, (result.stdout or "").strip()

    def _ros2_service_call(
        self,
        service: str,
        srv_type: str,
        args_yaml: str = "{}",
        timeout_sec: float = 15.0,
    ) -> bool:
        ok, _ = self._ros2_service_call_result(
            service,
            srv_type,
            args_yaml=args_yaml,
            timeout_sec=timeout_sec,
        )
        return ok

    def _init_step(
        self,
        name: str,
        fn: Callable[[], bool],
        pause_before_sec: float = 1.0,
    ) -> bool:
        if pause_before_sec > 0:
            time.sleep(pause_before_sec)
        gui_log("GUI-BRIDGE", f"initialize_robot step {name} start")
        ok = bool(fn())
        gui_log(
            "GUI-BRIDGE",
            f"initialize_robot step {name} {'success' if ok else 'failed'}",
        )
        return ok

    def _joint1_startup_home(self) -> Optional[float]:
        angles = self.current_angles()
        if not angles:
            return None
        a1 = float(angles[0])
        ranges = [
            (-22.49, 22.49, 0.0),
            (22.51, 67.49, 45.0),
            (67.51, 112.49, 90.0),
            (112.51, 157.49, 135.0),
            (157.51, 202.49, 180.0),
            (202.51, 247.49, -135.0),
            (247.51, 292.49, -90.0),
            (292.51, 337.49, -45.0),
            (337.51, 360.0, 0.0),
            (-67.49, -22.51, -45.0),
            (-112.49, -67.51, -90.0),
            (-157.49, -112.51, -135.0),
            (-202.49, -157.51, -180.0),
            (-247.49, -202.51, 135.0),
            (-292.49, -247.51, 90.0),
            (-337.49, -292.51, 45.0),
            (-360.0, -337.51, 0.0),
        ]
        for low, high, value in ranges:
            if low <= a1 <= high:
                return value
        gui_log("GUI-BRIDGE", f"initialize_robot current joint1 outside ranges: {a1}")
        return None

    def _run_dobot_init_once(self, attempt: int) -> Tuple[bool, str]:
        srv_root = "/dobot_bringup_v3/srv"
        gui_log("GUI-BRIDGE", f"initialize_robot attempt {attempt}: dobot init start")
        steps: List[Tuple[str, Callable[[], bool], float]] = [
            (
                "ClearError",
                lambda: self._ros2_service_call(
                    f"{srv_root}/ClearError",
                    "dobot_msgs_v3/srv/ClearError",
                    "{}",
                ),
                5.0,
            ),
            (
                "DisableRobot",
                lambda: self._ros2_service_call(
                    f"{srv_root}/DisableRobot",
                    "dobot_msgs_v3/srv/DisableRobot",
                    "{}",
                ),
                1.0,
            ),
            (
                "EnableRobot",
                lambda: self._ros2_service_call(
                    f"{srv_root}/EnableRobot",
                    "dobot_msgs_v3/srv/EnableRobot",
                    "{load: 2.0}",
                ),
                1.0,
            ),
            (
                "CP",
                lambda: self._ros2_service_call(
                    f"{srv_root}/CP",
                    "dobot_msgs_v3/srv/CP",
                    "{ r: 100 }",
                ),
                1.0,
            ),
            (
                "SetGripperPosition(open)",
                lambda: self._ros2_service_call(
                    f"{srv_root}/SetGripperPosition",
                    "dobot_msgs_v3/srv/SetGripperPosition",
                    "{position: 0, speed: 255, force: 255}",
                ),
                1.0,
            ),
            (
                "StartDrag",
                lambda: self._ros2_service_call(
                    f"{srv_root}/StartDrag",
                    "dobot_msgs_v3/srv/StartDrag",
                    "{}",
                ),
                1.0,
            ),
            (
                "StopDrag",
                lambda: self._ros2_service_call(
                    f"{srv_root}/StopDrag",
                    "dobot_msgs_v3/srv/StopDrag",
                    "{}",
                ),
                6.0,
            ),
        ]
        for name, fn, pause in steps:
            if not self._init_step(name, fn, pause_before_sec=pause):
                return False, name
        return True, ""

    def _reset_after_failed_init_attempt(self) -> None:
        srv_root = "/dobot_bringup_v3/srv"
        gui_log("GUI-BRIDGE", "initialize_robot reset before retry start")
        self._ros2_service_call(
            f"{srv_root}/ClearError",
            "dobot_msgs_v3/srv/ClearError",
            "{}",
        )
        time.sleep(1.0)
        self._ros2_service_call(
            f"{srv_root}/DisableRobot",
            "dobot_msgs_v3/srv/DisableRobot",
            "{}",
        )
        time.sleep(3.0)
        gui_log("GUI-BRIDGE", "initialize_robot reset before retry done")

    def _run_post_dobot_startup_steps(self) -> Tuple[bool, str]:
        srv_root = "/dobot_bringup_v3/srv"

        def move_to_startup_home() -> bool:
            j1_val = self._joint1_startup_home()
            if j1_val is None:
                return False
            return self._ros2_service_call(
                f"{srv_root}/JointMovJ",
                "dobot_msgs_v3/srv/JointMovJ",
                (
                    "{"
                    f"j1: {j1_val}, j2: 30.0, j3: -130.0, "
                    "j4: -100.0, j5: -90.0, j6: 0.0"
                    "}"
                ),
            )

        steps: List[Tuple[str, Callable[[], bool], float]] = [
            ("Move startup home", move_to_startup_home, 1.0),
            (
                "ModbusClose",
                lambda: self._ros2_service_call(
                    f"{srv_root}/ModbusClose",
                    "dobot_msgs_v3/srv/ModbusClose",
                    "{index: 0}",
                ),
                1.0,
            ),
            (
                "ModbusCreate",
                lambda: self._ros2_service_call(
                    f"{srv_root}/ModbusCreate",
                    "dobot_msgs_v3/srv/ModbusCreate",
                    '{ip: "127.0.0.1", port: 60000, slave_id: 9, is_rtu: 1}',
                ),
                1.0,
            ),
            (
                "SetHoldRegs(reset)",
                lambda: self._ros2_service_call(
                    f"{srv_root}/SetHoldRegs",
                    "dobot_msgs_v3/srv/SetHoldRegs",
                    '{index: 0, addr: 1000, count: 3, val_tab: "0,0,0", val_type: "int"}',
                ),
                1.0,
            ),
            (
                "SetHoldRegs(enable)",
                lambda: self._ros2_service_call(
                    f"{srv_root}/SetHoldRegs",
                    "dobot_msgs_v3/srv/SetHoldRegs",
                    '{index: 0, addr: 1000, count: 3, val_tab: "256,0,0", val_type: "int"}',
                ),
                0.0,
            ),
        ]
        for name, fn, pause in steps:
            if not self._init_step(name, fn, pause_before_sec=pause):
                return False, name
        return True, ""

    def initialize_robot(self) -> Tuple[bool, str]:
        """Run the startup-script Dobot init and post-init setup."""
        if not self._begin_control("initialize_robot"):
            return False, "robot control action already in progress"
        try:
            gui_log("GUI-BRIDGE", "initialize_robot start")
            if self._direct_run_skill is None:
                return False, "initialize requires direct ROS2 backend"

            max_attempts = 3
            failures: List[str] = []
            for attempt in range(1, max_attempts + 1):
                ok, failed_step = self._run_dobot_init_once(attempt)
                if ok:
                    failures = []
                    break
                failures.append(f"attempt {attempt}: {failed_step}")
                gui_log(
                    "GUI-BRIDGE",
                    f"initialize_robot attempt {attempt} failed at {failed_step}",
                )
                if attempt < max_attempts:
                    self._reset_after_failed_init_attempt()
            else:
                msg = "initialize failed: " + "; ".join(failures)
                gui_log("GUI-BRIDGE", msg)
                return False, msg

            ok, failed_step = self._run_post_dobot_startup_steps()
            if not ok:
                msg = f"initialize failed after Dobot init at {failed_step}"
                gui_log("GUI-BRIDGE", msg)
                return False, msg

            gui_log("GUI-BRIDGE", "initialize_robot success")
            return True, "robot initialized"
        finally:
            self._end_control()

    def disable_robot(self) -> Tuple[bool, str]:
        """Disable the arm via the dobot driver DisableRobot service."""
        if not self._begin_control("disable_robot"):
            return False, "robot control action already in progress"
        try:
            gui_log("GUI-BRIDGE", "disable_robot start")
            if self._direct_run_skill is None:
                return False, "disable requires direct ROS2 (dobot driver services)"
            srv_root = "/dobot_bringup_v3/srv"
            ok = self._ros2_service_call(
                f"{srv_root}/DisableRobot",
                "dobot_msgs_v3/srv/DisableRobot",
                "{}",
            )
            gui_log("GUI-BRIDGE", f"disable_robot {'success' if ok else 'failed'}")
            if ok:
                return True, "robot disabled"
            return False, "DisableRobot service call failed"
        finally:
            self._end_control()

    def enable_robot(self) -> Tuple[bool, str]:
        """Enable the arm via the dobot driver EnableRobot service."""
        if not self._begin_control("enable_robot"):
            return False, "robot control action already in progress"
        try:
            gui_log("GUI-BRIDGE", "enable_robot start")
            if self._direct_run_skill is None:
                return False, "enable requires direct ROS2 (dobot driver services)"
            srv_root = "/dobot_bringup_v3/srv"
            ok = self._ros2_service_call(
                f"{srv_root}/EnableRobot",
                "dobot_msgs_v3/srv/EnableRobot",
                "{load: 2.0}",
            )
            gui_log("GUI-BRIDGE", f"enable_robot {'success' if ok else 'failed'}")
            if ok:
                return True, "robot enabled"
            return False, "EnableRobot service call failed"
        finally:
            self._end_control()

    def toggle_drag(self) -> Tuple[bool, Any]:
        if not self._begin_control("toggle_drag"):
            return False, {"error": "robot control action already in progress"}
        try:
            ok, info = self.call_skill("toggle_drag_mode")
            with self._last_state_lock:
                if ok:
                    cur = self._last_state.get("drag_enabled")
                    self._last_state["drag_enabled"] = not bool(cur)
            return ok, info
        finally:
            self._end_control()

    def open_gripper(self) -> Tuple[bool, Any]:
        if not self._begin_control("open_gripper"):
            return False, {"error": "robot control action already in progress"}
        try:
            return self._open_gripper_unlocked()
        finally:
            self._end_control()

    def _open_gripper_unlocked(self) -> Tuple[bool, Any]:
        if self._direct_run_skill is not None:
            try:
                self._direct("sync")
                result = self._direct("set_gripper_position", 255, 0, 255)
                ok = self._ok(result)
                if ok:
                    self._update_gripper(0)
                return ok, result
            except Exception as exc:
                gui_log_error("GUI-BRIDGE", "open_gripper direct failed", exc)
                return False, {"error": str(exc)}
        return self.call_skill("open_gripper")

    def close_gripper(self) -> Tuple[bool, Any]:
        if not self._begin_control("close_gripper"):
            return False, {"error": "robot control action already in progress"}
        try:
            return self._close_gripper_unlocked()
        finally:
            self._end_control()

    def _close_gripper_unlocked(self) -> Tuple[bool, Any]:
        if self._direct_run_skill is not None:
            try:
                self._direct("sync")
                result = self._direct("set_gripper_position", 255, 255, 255)
                ok = self._ok(result)
                if ok:
                    self._update_gripper(255)
                return ok, result
            except Exception as exc:
                gui_log_error("GUI-BRIDGE", "close_gripper direct failed", exc)
                return False, {"error": str(exc)}
        return self.call_skill("close_gripper")

    def set_gripper_position(self, value: int) -> Tuple[bool, Any]:
        if not self._begin_control("set_gripper_position"):
            return False, {"error": "robot control action already in progress"}
        try:
            return self._set_gripper_position_unlocked(value)
        finally:
            self._end_control()

    def _set_gripper_position_unlocked(self, value: int) -> Tuple[bool, Any]:
        value = max(0, min(255, int(value)))
        gui_log("GUI-BRIDGE", f"set_gripper_position value={value}")
        if self._direct_run_skill is not None:
            try:
                self._direct("sync")
                result = self._direct(
                    "set_gripper_position",
                    255,
                    value,
                    255,
                    verify_position=True,
                )
                ok = self._ok(result)
                if ok:
                    self._update_gripper(value)
                return ok, result
            except Exception as exc:
                gui_log_error("GUI-BRIDGE", "set_gripper_position direct failed", exc)
                return False, {"error": str(exc)}
        return False, {
            "error": (
                "set_gripper_position requires direct ROS2 backend; "
                "no rabbitmq action exposes a numeric value"
            )
        }

    def _update_gripper(self, value: int) -> None:
        with self._last_state_lock:
            self._last_state["gripper"] = int(value)

    def current_angles(self) -> Optional[Tuple[float, ...]]:
        if self._direct_run_skill is None:
            return None
        try:
            angles = self._direct("current_angles")
        except Exception as exc:
            gui_log_error("GUI-BRIDGE", "current_angles raised", exc)
            return None
        if not angles:
            return None
        try:
            tup = tuple(float(v) for v in angles)
        except Exception:
            return None
        if len(tup) < 6:
            return None
        with self._last_state_lock:
            self._last_state["joints"] = tup
            self._last_state["joints_stamp"] = time.time()
        return tup

    def current_pose(self) -> Optional[Tuple[float, ...]]:
        if self._direct_run_skill is None:
            return None
        try:
            pose = self._direct("current_pose")
        except Exception as exc:
            gui_log_error("GUI-BRIDGE", "current_pose raised", exc)
            return None
        if not pose:
            return None
        try:
            tup = tuple(float(v) for v in pose)
        except Exception:
            return None
        if len(tup) < 6:
            return None
        with self._last_state_lock:
            self._last_state["pose"] = tup
            self._last_state["pose_stamp"] = time.time()
        return tup

    def last_state(self) -> Dict[str, Any]:
        with self._last_state_lock:
            return dict(self._last_state)


_singleton: Optional[RobotBridge] = None
_singleton_lock = threading.Lock()


def get_bridge() -> RobotBridge:
    global _singleton
    if _singleton is None:
        with _singleton_lock:
            if _singleton is None:
                _singleton = RobotBridge()
    return _singleton
