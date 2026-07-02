"""
espresso.py

Defines the espresso-making sequence for different ports and cups.
This module provides comprehensive functions for managing the complete espresso
workflow including portafilter handling, grinding, tamping, mounting, and milk operations.
"""

import logging
import csv
import math
import statistics
import sys
import inspect
from pathlib import Path
import time
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill, init_motion_node
from oms_v1.params import (
    PULL_ESPRESSO_PARAMS,
    ESPRESSO_HOME,
    ESPRESSO_GRINDER_HOME,
    ESPRESSO_GRINDER_PARAMS,
    ESPRESSO_PITCHER_FLOW_POSES, ESPRESSO_ANGLED_TRANSFER_POSES,
    ESPRESSO_PITCHER_PARAMS,
    ESPRESSO_HOT_WATER_PARAMS,
    ESPRESSO_SPEEDS, ESPRESSO_PITCHER_GRIPPER, ESPRESSO_PORTAFILTER_GRIPPER,
    ESPRESSO_MOVEMENT_OFFSETS, ESPRESSO_DELAYS,
    GRIPPER_FULL, GRIPPER_OPEN, SPEED_FAST, SPEED_SUPER_SLOW, SPEED_SLOW_POURING,
    _extract_cup_position
)
from oms_v1.sequences.computer_vision import detect_cup_gripper as _raw_detect_cup_gripper


# -----------------------------------------------------------------------------
# Runtime trace helpers
# -----------------------------------------------------------------------------
# Set ESPRESSO_TRACE_DEBUG = False to disable these debug prints globally.
# The helper prints immediately with flush=True so ROS/stdout logs show the exact
# function and run_skill step that executed before/after each robot command.
ESPRESSO_TRACE_DEBUG = True

def _trace_step(scope: str, msg: str) -> None:
    """Lightweight visible trace for high-level function steps."""
    if globals().get("ESPRESSO_TRACE_DEBUG", True):
        print(f"[TRACE:{scope}] {msg}", flush=True)

def _trace_format_result(result: Any) -> str:
    """Keep trace results short so logging stays readable."""
    if isinstance(result, (tuple, list)):
        if len(result) <= 6 and all(isinstance(x, (int, float)) for x in result):
            return "(" + ", ".join(f"{float(x):.3f}" for x in result) + ")"
        return f"{type(result).__name__}(len={len(result)})"
    return repr(result)

def _trace_run_skill(scope: str, skill_name: str, *args: Any) -> Any:
    """Trace every run_skill call without changing its return value."""
    if globals().get("ESPRESSO_TRACE_DEBUG", True):
        print(f"[TRACE:{scope}] run_skill('{skill_name}') START args={args}", flush=True)
    result = run_skill(skill_name, *args)
    if globals().get("ESPRESSO_TRACE_DEBUG", True):
        print(
            f"[TRACE:{scope}] run_skill('{skill_name}') DONE result={_trace_format_result(result)}",
            flush=True,
        )
    return result


def _fail(reason: str = "") -> bool:
    """Drop-in replacement for `return False` that logs the caller and the
    triggering source line, so silent guard/skill failures are visible in stdout.

    Use as: `return _fail()` or `return _fail("custom reason")`.
    Always returns False so the calling function's contract is preserved.
    """
    if not globals().get("ESPRESSO_TRACE_DEBUG", True):
        return False
    try:
        frame = sys._getframe(1)
        scope = frame.f_code.co_name
        line_no = frame.f_lineno
        cause = ""
        try:
            src_lines, start = inspect.getsourcelines(frame)
            # `line_no` is 1-based file line; `start` is the function's first line.
            # The line that triggered the failure is usually the `if ...:` immediately
            # above the `return _fail()` line.
            rel = line_no - start
            for back in (1, 2):
                idx = rel - back
                if 0 <= idx < len(src_lines):
                    candidate = src_lines[idx].strip()
                    if candidate and not candidate.startswith("return"):
                        cause = candidate
                        break
        except Exception:
            pass
        msg = f"[TRACE:{scope}] FAIL line={line_no}"
        if cause:
            msg += f" cause=`{cause}`"
        if reason:
            msg += f" reason={reason}"
        print(msg, flush=True)
    except Exception:
        pass
    return False


# Backward-compatible globals retained for any external references.
below_espresso_port: Optional[Tuple[float, ...]] = None
mount_espresso_port: Optional[Tuple[float, ...]] = None
approach_pitcher: Optional[Tuple[float, ...]] = None
pick_pitcher: Optional[Tuple[float, ...]] = None

# Per-port angle cache: populated on first unmount, reused on subsequent calls.
_port_angle_cache: Dict[str, Dict[str, Tuple[float, ...]]] = {}
_pitcher_clean_cache: Dict[str, list] = {}
_pitcher_pick_cache: Dict[str, Dict[str, Tuple[float, ...]]] = {}
_pitcher_return_cache: Dict[str, Dict[str, Tuple[float, ...]]] = {}

# Per-tool pose cache captured at the end of grinder and reused by grinder/tamper.
_tool_pick_pose_cache: Dict[str, Tuple[float, ...]] = {}

# BEST: dedicated grinder cache so grinder mount replay does not share namespace with tool-pick cache.
_grinder_post_mount_cache: Dict[str, Tuple[float, ...]] = {}

# Per-port runtime state so mount/unmount does not cross-contaminate between ports.
_mount_runtime_cache: Dict[str, Dict[str, Tuple[float, ...]]] = {}

# Generic machine alignment caches captured after live approach_machine / mount_machine.
# Using shared caches keeps behavior consistent across normal + angled flows.
_machine_approach_pose_cache: Dict[str, Tuple[float, ...]] = {}
_machine_mount_pose_cache: Dict[str, Tuple[float, ...]] = {}

# After grab_tool on unmount: first live pass caches joint pose; later passes replay with gotoJ_deg.
_unmount_post_grab_joints_cache: Dict[str, Tuple[float, ...]] = {}


_gripper_log = logging.getLogger(__name__)

_GRIPPER_OPEN_MAX_POS = 20
_GRIPPER_OPEN_RETRIES = 3

# Acceptable gripper position after a correct close on the portafilter (Dobot
# decoded value from register 2002). Center ~146; allow small variation.
_PORTAFILTER_GRIP_POS_MIN = 134
_PORTAFILTER_GRIP_POS_MAX = 150

# After release_tension on uncached unmount grab: Link6 Z from current_pose (mm).
_UNMOUNT_POST_TENSION_Z_TARGET_MM = 200.0
_UNMOUNT_POST_TENSION_Z_TARGET_ANGL_MM = 140.0
_UNMOUNT_POST_TENSION_Z_TOL_MM = 5.0

# Live unmount (no _port_angle_cache): Z drop after arc -> after tension sets clear_up Z (mm) for mount.
_portafilter_clear_up_z_mm_by_port: Dict[str, float] = {}

# After live tamper tool acquisition: first pass caches post-grab joint pose;
# later passes replay with gotoJ_deg + sync before closing gripper.
_tamper_post_grab_joints_cache: Dict[str, Tuple[float, ...]] = {}
_angled_tamper_post_grab_joints_cache: Dict[str, Tuple[float, ...]] = {}
_portafilter_arc_cmd_by_port: Dict[str, float] = {}
_portafilter_mount_arc_cmd_by_port: Dict[str, float] = {}

# Angled per-port learned arc commands.
# First uncached angled_unmount calculates these from current_pose RZ.
# Later cached angled_unmount and angled_mount reuse them.
angled__portafilter_arc_cmd_by_port: Dict[str, float] = {}
angled__portafilter_mount_arc_cmd_by_port: Dict[str, float] = {}
angled__portafilter_arc_cmd_by_port_2: Dict[str, float] = {}

# From measured angled data:
# after enforce_rxry_angled rz ~= 84.781235
# after arc -37.0 rz ~= 47.781235
_ANGLED_PORTAFILTER_ARC_TARGET_RZ = 48.0

# Current angled mount hardcode was 42.5 while measured unmount delta was 37.0.
# So mount return arc = arc_delta + 5.5.
_ANGLED_PORTAFILTER_MOUNT_ARC_EXTRA = 2.5

# Runtime tracing helpers.
# Set PAPER_CUPS_TRACE_DEBUG = False to disable these step prints without changing behavior.
PAPER_CUPS_TRACE_DEBUG = True

def _trace_format_value(value, max_len: int = 140) -> str:
    """Return a compact printable representation for trace logs."""
    try:
        text = repr(value)
    except Exception:
        text = f"<{type(value).__name__}>"
    if len(text) > max_len:
        text = text[:max_len - 3] + "..."
    return text

def _trace_step(scope: str, message: str) -> None:
    """Print a visible, low-overhead runtime trace message."""
    if globals().get("PAPER_CUPS_TRACE_DEBUG", True):
        print(f"[{scope}] {message}", flush=True)

def detect_cup_gripper(*args, **kwargs):
    """Trace wrapper around detect_cup_gripper(...)."""
    _trace_step("PAPER-CUPS", f"detect_cup_gripper START args={_trace_format_value(args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_detect_cup_gripper(*args, **kwargs)
    _trace_step("PAPER-CUPS", f"detect_cup_gripper DONE result={_trace_format_value(result)}")
    return result

def ok(r):
    return r not in (False, None)

def _portafilter_clear_up_offset(port: str) -> Tuple[float, float, float, float, float, float]:
    """Use per-port learned Z from last live unmount, else params default."""
    _trace_step("_portafilter_clear_up_offset", "START")
    z = _portafilter_clear_up_z_mm_by_port.get(str(port))
    if z is not None:
        return (0.0, 0.0, float(z), 0.0, 0.0, 0.0)
    base = ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_up"]
    return tuple(float(x) for x in base)

def _portafilter_clear_up_angled_offset(port: str) -> Tuple[float, float, float, float, float, float]:
    """Angled mount clear-up: learned Z from last live angled unmount, else params default."""
    _trace_step("_portafilter_clear_up_angled_offset", "START")
    z = _portafilter_clear_up_z_mm_by_port.get(str(port))
    base = ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_up_angled"]
    if z is not None:
        return (float(base[0]), float(base[1]), float(z), float(base[3]), float(base[4]), float(base[5]))
    return tuple(float(x) for x in base)

def _angled_unmount_grab_tool_name(port: str) -> str:
    """Portafilter tool frame for angled unmount grab (per robot teach)."""
    _trace_step("_angled_unmount_grab_tool_name", "START")
    if str(port) == "angled_portafilter_2":
        return "single_portafilter_angled"
    return "double_portafilter_angled"

def _open_gripper_with_verify(speed=255, force=255):
    """Send gripper-open (position 0) and verify it actually reached a low position.

    Retries up to _GRIPPER_OPEN_RETRIES times if the reported position is above
    _GRIPPER_OPEN_MAX_POS (gripper did not physically open).
    """
    _trace_step("_open_gripper_with_verify", "START")
    node = init_motion_node()
    for attempt in range(1, _GRIPPER_OPEN_RETRIES + 1):
        success, actual_pos = node.set_gripper_position(speed=speed, position=0, force=force)
        if not success:
            _gripper_log.error(f"[GRIPPER-OPEN] command failed (attempt {attempt}/{_GRIPPER_OPEN_RETRIES})")
            time.sleep(0.3)
            continue
        if actual_pos is not None and actual_pos <= _GRIPPER_OPEN_MAX_POS:
            return True
        _gripper_log.warning(
            f"[GRIPPER-OPEN] position {actual_pos} > {_GRIPPER_OPEN_MAX_POS}, "
            f"retrying (attempt {attempt}/{_GRIPPER_OPEN_RETRIES})"
        )
        time.sleep(0.3)
    _gripper_log.error(f"[GRIPPER-OPEN] failed to open after {_GRIPPER_OPEN_RETRIES} attempts")
    return _fail()

def invalidate_port_cache():
    _trace_step("invalidate_port_cache", "START")
    _port_angle_cache.clear()
    _pitcher_clean_cache.clear()
    _pitcher_pick_cache.clear()
    _pitcher_return_cache.clear()
    _tool_pick_pose_cache.clear()
    _grinder_post_mount_cache.clear()
    _mount_runtime_cache.clear()
    _machine_approach_pose_cache.clear()
    _machine_mount_pose_cache.clear()
    _unmount_post_grab_joints_cache.clear()
    _portafilter_clear_up_z_mm_by_port.clear()
    _tamper_post_grab_joints_cache.clear()
    _angled_tamper_post_grab_joints_cache.clear()
    _portafilter_arc_cmd_by_port.clear()
    _portafilter_mount_arc_cmd_by_port.clear()

def _is_valid_angles(angles: Any) -> bool:
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6

def _run_cached_machine_approach(cache_key: str, machine_name: str, target_name: str) -> bool:
    """
    Replay a cached pose captured immediately after a successful approach_machine(...).
    If no cache exists yet, run the live approach, sync, capture current_angles, and cache them.
    """
    _trace_step("_run_cached_machine_approach", "START")
    cached_angles = _machine_approach_pose_cache.get(cache_key)
    if _is_valid_angles(cached_angles):
        _trace_step("_run_cached_machine_approach", f"cache HIT key={cache_key}")
        if _trace_run_skill("_run_cached_machine_approach", "gotoJ_deg", *cached_angles) in (False, None):
            return _fail()
        return True

    _trace_step("_run_cached_machine_approach", f"cache MISS key={cache_key}; live approach {machine_name}/{target_name}")
    if _trace_run_skill("_run_cached_machine_approach", "approach_machine", machine_name, target_name) in (False, None):
        return _fail()
    if _trace_run_skill("_run_cached_machine_approach", "sync") in (False, None):
        return _fail()
    captured_angles = _trace_run_skill("_run_cached_machine_approach", "current_angles")
    if not _is_valid_angles(captured_angles):
        return _fail()
    _machine_approach_pose_cache[cache_key] = tuple(captured_angles)
    return True

def _run_cached_machine_mount(cache_key: str, machine_name: str, target_name: str) -> bool:
    """
    Replay a cached pose captured immediately after a successful mount_machine(...).
    If no cache exists yet, run the live mount, sync, capture current_angles, and cache them.
    """
    _trace_step("_run_cached_machine_mount", "START")
    cached_angles = _machine_mount_pose_cache.get(cache_key)
    if _is_valid_angles(cached_angles):
        _trace_step("_run_cached_machine_mount", f"cache HIT key={cache_key}")
        if _trace_run_skill("_run_cached_machine_mount", "gotoJ_deg", *cached_angles) in (False, None):
            return _fail()
        return True

    _trace_step("_run_cached_machine_mount", f"cache MISS key={cache_key}; live mount {machine_name}/{target_name}")
    if _trace_run_skill("_run_cached_machine_mount", "mount_machine", machine_name, target_name) in (False, None):
        return _fail()
    if _trace_run_skill("_run_cached_machine_mount", "sync") in (False, None):
        return _fail()
    captured_angles = _trace_run_skill("_run_cached_machine_mount", "current_angles")
    if not _is_valid_angles(captured_angles):
        return _fail()
    _machine_mount_pose_cache[cache_key] = tuple(captured_angles)
    return True

def _normalize_espresso_shot(espresso_dict: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    _trace_step("_normalize_espresso_shot", "START")
    try:
        if not espresso_dict or not isinstance(espresso_dict, dict):
            return None

        espresso_key = next(iter(espresso_dict.keys()), None)
        if not espresso_key:
            return None

        espresso_key_lower = str(espresso_key).lower()

        if 'single' in espresso_key_lower:
            return {
                "port": "angled_portafilter_2",
                "positioning_time": 1.2,
                "portafilter_tool": "single_portafilter_angled",
                "angled": True,
            }
        elif 'double' in espresso_key_lower:
            value = espresso_dict.get(espresso_key)
            if value is not None and float(value) == 2.0:
                return {
                    "port": "port_1",
                    "positioning_time": 2.4,
                    "portafilter_tool": "double_portafilter",
                }
            return {
                "port": "port_1",
                "positioning_time": 2.4,
                "portafilter_tool": "double_portafilter",
            }
        else:
            value = espresso_dict.get(espresso_key)
            if value is not None:
                shots = float(value)
                if shots <= 1.0:
                    return {
                        "port": "angled_portafilter_2",
                        "positioning_time": 1.2,
                        "portafilter_tool": "single_portafilter_angled",
                        "angled": True,
                    }
                elif shots == 2.0:
                    return {
                        "port": "port_1",
                        "positioning_time": 2.4,
                        "portafilter_tool": "double_portafilter",
                    }
                else:
                    return {
                        "port": "port_1",
                        "positioning_time": 2.4,
                        "portafilter_tool": "double_portafilter",
                    }
    except Exception as e:
        print(f"[WARNING] Error parsing espresso parameters: {e}")
        return None

    return None

def unmount(**params) -> bool:
    global below_espresso_port, mount_espresso_port

    _trace_step("unmount", "START")

    # -------------------------------------------------------------------------
    # HARD RUNTIME FINGERPRINT
    # -------------------------------------------------------------------------
    # If this exact line does not appear in your runtime log, the robot is still
    # not running this edited function.
    UNMOUNT_CODE_VERSION = "UNMOUNT_RETRY_DEBUG_V4_2026_06_09_ACCEPT_134_FULL_RECOVERY_3X"

    # Use local values so this function does not depend on stale/global constants.
    GRIP_POS_MIN = 134
    GRIP_POS_MAX = 150
    CLOSE_VERIFY_ATTEMPTS = 3
    FULL_RECOVERY_ATTEMPTS = 3

    print(
        f"[UNMOUNT-CODE-VERSION] {UNMOUNT_CODE_VERSION} "
        f"file={globals().get('__file__', '<unknown>')} "
        f"grip_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}] "
        f"close_verify_attempts={CLOSE_VERIFY_ATTEMPTS} "
        f"full_recovery_attempts={FULL_RECOVERY_ATTEMPTS}",
        flush=True,
    )

    def ok(r):
        return r not in (False, None)

    def _grip_ok(pos) -> bool:
        return pos is not None and GRIP_POS_MIN <= int(pos) <= GRIP_POS_MAX

    def _close_and_verify_grip(max_attempts: int = CLOSE_VERIFY_ATTEMPTS):
        """
        Close gripper and return (gripped_ok, reported_position).

        This function performs up to max_attempts close/read attempts.
        pos=134 is valid.
        """
        _trace_step("unmount._close_and_verify_grip", "START")

        node = init_motion_node()
        last_pos = None

        for attempt in range(1, max_attempts + 1):
            print(
                f"[PORTAFILTER-GRIP] close/verify attempt {attempt}/{max_attempts} START "
                f"target_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]",
                flush=True,
            )

            success, pos = node.set_gripper_position(
                speed=255,
                position=255,
                force=255,
            )
            last_pos = pos

            if not success:
                _gripper_log.warning(
                    f"[PORTAFILTER-GRIP] close/verify attempt {attempt}/{max_attempts} "
                    f"failed to command gripper"
                )
                continue

            if _grip_ok(pos):
                _gripper_log.info(
                    f"[PORTAFILTER-GRIP] close/verify attempt {attempt}/{max_attempts} "
                    f"OK pos={pos}, accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]"
                )
                return True, pos

            _gripper_log.warning(
                f"[PORTAFILTER-GRIP] close/verify attempt {attempt}/{max_attempts} "
                f"pos={pos} outside accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]"
            )

        return False, last_pos

    def _grab_then_close():
        """
        Perform grab_tool + close + read.
        Returns (gripped_ok, pos).
        """
        _trace_step("unmount._grab_then_close", "START")

        if not ok(_trace_run_skill("unmount._grab_then_close", "sync")):
            return False, None

        if not ok(_trace_run_skill("unmount._grab_then_close", "grab_tool", "double_portafilter")):
            return False, None

        if not ok(_trace_run_skill("unmount._grab_then_close", "sync")):
            return False, None

        return _close_and_verify_grip()

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    if shot_cfg and shot_cfg.get("angled"):
        return angled_unmount(**params)

    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_1")

    if not port:
        return _fail()

    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return _fail()

    if not ok(_trace_run_skill("unmount", "gotoJ_deg", *port_params["home"])):
        return _fail()

    if port in ("port_1", "port_3"):
        if not _run_cached_machine_approach(
            f"unmount:{port}:approach:{port_params['portafilter_number']}",
            "three_group_espresso",
            port_params["portafilter_number"],
        ):
            return _fail()

    grab_cache_key = f"unmount:post_grab:{port}"
    cached_grab_joints = _unmount_post_grab_joints_cache.get(grab_cache_key)

    if _is_valid_angles(cached_grab_joints):
        print(
            f"[PORTAFILTER-GRIP] cached post-grab pose HIT key={grab_cache_key}",
            flush=True,
        )

        if not ok(_trace_run_skill("unmount", "gotoJ_deg", *cached_grab_joints)):
            return _fail()

        if not ok(_trace_run_skill("unmount", "sync")):
            return _fail()

        if not ok(_trace_run_skill("unmount", "set_gripper_position", 255, 255, 255)):
            return _fail()

    else:
        print(
            f"[PORTAFILTER-GRIP] cached post-grab pose MISS key={grab_cache_key}; "
            f"running live grab recovery routine",
            flush=True,
        )

        # ---------------------------------------------------------------------
        # Attempt 1: normal grab_tool + close/verify
        # ---------------------------------------------------------------------
        gripped, pos = _grab_then_close()

        # ---------------------------------------------------------------------
        # Attempt 2: nudge down 2.5 mm and retry close/verify
        # ---------------------------------------------------------------------
        if not gripped:
            _gripper_log.warning(
                f"[PORTAFILTER-GRIP] recovery stage 2: pos={pos} "
                f"outside accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]; "
                f"nudging down 2.5 mm and retrying close"
            )

            if not ok(_trace_run_skill("unmount", "moveEE_movJ", 0, 0, -2.5, 0, 0, 0)):
                return _fail()

            gripped, pos = _close_and_verify_grip()

        # ---------------------------------------------------------------------
        # Attempt 3: nudge up 5 mm and retry close/verify
        # ---------------------------------------------------------------------
        if not gripped:
            _gripper_log.warning(
                f"[PORTAFILTER-GRIP] recovery stage 3: pos={pos} "
                f"outside accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]; "
                f"nudging up 5 mm and retrying close"
            )

            if not ok(_trace_run_skill("unmount", "moveEE_movJ", 0, 0, 5, 0, 0, 0)):
                return _fail()

            gripped, pos = _close_and_verify_grip()

        # ---------------------------------------------------------------------
        # Full recovery: open gripper, re-run approach, re-grab.
        # This MUST run 1/3, 2/3, 3/3 unless one succeeds.
        # ---------------------------------------------------------------------
        if not gripped:
            for recovery_attempt in range(1, FULL_RECOVERY_ATTEMPTS + 1):
                _gripper_log.warning(
                    f"[PORTAFILTER-GRIP] FULL RECOVERY "
                    f"{recovery_attempt}/{FULL_RECOVERY_ATTEMPTS} START "
                    f"last_pos={pos}, accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]"
                )

                if not ok(_trace_run_skill("unmount", "set_gripper_position", 255, 0, 255)):
                    _gripper_log.warning(
                        f"[PORTAFILTER-GRIP] FULL RECOVERY "
                        f"{recovery_attempt}/{FULL_RECOVERY_ATTEMPTS} "
                        f"failed while opening gripper; continuing"
                    )
                    continue

                if not ok(_trace_run_skill("unmount", "sync")):
                    _gripper_log.warning(
                        f"[PORTAFILTER-GRIP] FULL RECOVERY "
                        f"{recovery_attempt}/{FULL_RECOVERY_ATTEMPTS} "
                        f"failed while syncing after open; continuing"
                    )
                    continue

                if port in ("port_1", "port_3"):
                    if not _run_cached_machine_approach(
                        f"unmount:{port}:approach:{port_params['portafilter_number']}",
                        "three_group_espresso",
                        port_params["portafilter_number"],
                    ):
                        _gripper_log.warning(
                            f"[PORTAFILTER-GRIP] FULL RECOVERY "
                            f"{recovery_attempt}/{FULL_RECOVERY_ATTEMPTS} "
                            f"failed during cached machine approach; continuing"
                        )
                        continue

                gripped, pos = _grab_then_close()

                if gripped:
                    _gripper_log.info(
                        f"[PORTAFILTER-GRIP] FULL RECOVERY "
                        f"{recovery_attempt}/{FULL_RECOVERY_ATTEMPTS} SUCCESS "
                        f"pos={pos}, accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]"
                    )
                    break

                _gripper_log.warning(
                    f"[PORTAFILTER-GRIP] FULL RECOVERY "
                    f"{recovery_attempt}/{FULL_RECOVERY_ATTEMPTS} FAILED "
                    f"pos={pos}, accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]"
                )

        if not gripped:
            _gripper_log.error(
                f"[PORTAFILTER-GRIP] FINAL FAIL pos={pos} "
                f"accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]; "
                f"aborting unmount for {port}"
            )
            return _fail()

        _gripper_log.info(
            f"[PORTAFILTER-GRIP] gripped OK pos={pos}, "
            f"accepted_range=[{GRIP_POS_MIN}, {GRIP_POS_MAX}]"
        )

        if not ok(_trace_run_skill("unmount", "release_tension")):
            return _fail()

        if not ok(_trace_run_skill("unmount", "sync")):
            return _fail()

        if not ok(_trace_run_skill("unmount", "enforce_rxry")):
            return _fail()

        if not ok(_trace_run_skill("unmount", "sync")):
            return _fail()

        # Z height check after release_tension.
        z_lo = _UNMOUNT_POST_TENSION_Z_TARGET_MM - _UNMOUNT_POST_TENSION_Z_TOL_MM
        z_hi = _UNMOUNT_POST_TENSION_Z_TARGET_MM + _UNMOUNT_POST_TENSION_Z_TOL_MM

        pose_z = _trace_run_skill("unmount", "current_pose")
        if not ok(pose_z) or not isinstance(pose_z, (tuple, list)) or len(pose_z) < 3:
            return _fail()

        z_mm = float(pose_z[2])

        if not (z_lo <= z_mm <= z_hi):
            dz = (_UNMOUNT_POST_TENSION_Z_TARGET_MM - z_mm) / 1.0

            _gripper_log.warning(
                f"[UNMOUNT-Z] after release_tension z={z_mm:.2f} mm "
                f"outside [{z_lo:.1f}, {z_hi:.1f}]; "
                f"moveEE_movJ dz={dz:.2f} mm"
            )

            if not ok(_trace_run_skill("unmount", "set_gripper_position", 25, 100, 25)):
                return _fail()

            if not ok(_trace_run_skill("unmount", "moveEE_movJ", -0.25, 0, dz, 0, 0, 0)):
                return _fail()

            if not ok(_trace_run_skill("unmount", "set_gripper_position", 255, 255, 255)):
                return _fail()

            if not ok(_trace_run_skill("unmount", "sync")):
                return _fail()

            if not ok(_trace_run_skill("unmount", "release_tension")):
                return _fail()

            if not ok(_trace_run_skill("unmount", "sync")):
                return _fail()

        angles = _trace_run_skill("unmount", "current_angles")
        if not ok(angles) or not _is_valid_angles(angles):
            return _fail()

        _unmount_post_grab_joints_cache[grab_cache_key] = tuple(angles)

        print(
            f"[PORTAFILTER-GRIP] cached post-grab joints key={grab_cache_key} "
            f"angles={tuple(angles)}",
            flush=True,
        )

        if not ok(_trace_run_skill("unmount", "sync")):
            return _fail()

    if not ok(_trace_run_skill("unmount", "enforce_rxry")):
        return _fail()

    if not ok(_trace_run_skill("unmount", "sync")):
        return _fail()

    cached_port_angle = _port_angle_cache.get(port)

    if cached_port_angle:
        arc_cmd = _portafilter_arc_cmd_by_port.get(str(port))
        if arc_cmd is None:
            return _fail()

        if not ok(_trace_run_skill("unmount", "move_portafilter_arc_movJ", arc_cmd)):
            return _fail()

    else:
        pose_before_arc = _trace_run_skill("unmount", "current_pose")
        if (
            not ok(pose_before_arc)
            or not isinstance(pose_before_arc, (tuple, list))
            or len(pose_before_arc) < 6
        ):
            return _fail()

        current_rz = float(pose_before_arc[5])
        desired_rz = 44.0

        arc_delta = current_rz - desired_rz
        arc_cmd = -arc_delta
        arc_delta_mount = arc_delta + 3.5

        _portafilter_arc_cmd_by_port[str(port)] = float(arc_cmd)
        _portafilter_mount_arc_cmd_by_port[str(port)] = float(arc_delta_mount)

        _gripper_log.info(
            f"[PORTAFILTER-ARC] port={port} current_rz={current_rz:.3f}, "
            f"desired_rz={desired_rz:.3f}, arc_cmd={arc_cmd:.3f}, "
            f"arc_delta_mount={arc_delta_mount:.3f}"
        )

        if not ok(_trace_run_skill("unmount", "move_portafilter_arc_movJ", arc_cmd)):
            return _fail()

    cached = _port_angle_cache.get(port)

    if cached:
        if not ok(_trace_run_skill("unmount", "release_tension")):
            return _fail()

        mount_pose = cached["mount"]
        below_pose = cached["below"]

        if not _is_valid_angles(below_pose):
            return _fail()

        if not ok(_trace_run_skill("unmount", "moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_down"])):
            return _fail()

    else:
        pose_after_arc = _trace_run_skill("unmount", "current_pose")
        if not ok(pose_after_arc) or not isinstance(pose_after_arc, (tuple, list)) or len(pose_after_arc) < 3:
            return _fail()

        z_after_arc_mm = float(pose_after_arc[2])

        if not ok(_trace_run_skill("unmount", "release_tension")):
            return _fail()

        if not ok(_trace_run_skill("unmount", "sync")):
            return _fail()

        mount_pose = _trace_run_skill("unmount", "current_angles")

        pose_after_tension = _trace_run_skill("unmount", "current_pose")
        if (
            not ok(pose_after_tension)
            or not isinstance(pose_after_tension, (tuple, list))
            or len(pose_after_tension) < 3
        ):
            return _fail()

        z_after_tension_mm = float(pose_after_tension[2])
        dz_drop_mm = z_after_arc_mm - z_after_tension_mm + 0.1

        if dz_drop_mm > 0.0:
            learned_clear_up_z = float(math.ceil(dz_drop_mm))
        else:
            learned_clear_up_z = float(ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_up"][2])

        _portafilter_clear_up_z_mm_by_port[str(port)] = learned_clear_up_z

        _gripper_log.info(
            f"[CLEAR-UP-Z] port={port} "
            f"z_arc={z_after_arc_mm:.2f} "
            f"z_after_tension={z_after_tension_mm:.2f} "
            f"drop={dz_drop_mm:.2f} mm -> "
            f"portafilter_clear_up z={learned_clear_up_z} mm"
        )

        if not _is_valid_angles(mount_pose):
            return _fail()

        if not ok(_trace_run_skill("unmount", "moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_down"])):
            return _fail()

        below_pose = _trace_run_skill("unmount", "current_angles")
        if not _is_valid_angles(below_pose):
            return _fail()

        mount_pose = tuple(mount_pose)
        below_pose = tuple(below_pose)

        _port_angle_cache[port] = {
            "mount": mount_pose,
            "below": below_pose,
        }

    _mount_runtime_cache[port] = {
        "mount": tuple(mount_pose),
        "below": tuple(below_pose),
    }

    mount_espresso_port = tuple(mount_pose)
    below_espresso_port = tuple(below_pose)

    if not ok(_trace_run_skill("unmount", "gotoJ_deg", *port_params["move_back"])):
        return _fail()

    if port in ("port_2", "port_3"):
        if not ok(_trace_run_skill("unmount", "gotoJ_deg", *ESPRESSO_GRINDER_PARAMS["nav1"])):
            return _fail()

        if not ok(_trace_run_skill("unmount", "gotoJ_deg", *ESPRESSO_GRINDER_PARAMS["nav2"])):
            return _fail()

    return True

def grinder(**params) -> bool:
    _trace_step("grinder", "START")

    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    if shot_cfg and shot_cfg.get("angled"):
        return angled_grinder(**params)

    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_1")

    positioning_time = params.get("positioning_time")
    if positioning_time is None:
        positioning_time = shot_cfg.get("positioning_time") if shot_cfg else 2.4

    portafilter_tool = (
        params.get("portafilter_tool")
        or (shot_cfg.get("portafilter_tool") if shot_cfg else "double_portafilter")
    )

    if not port or portafilter_tool not in ("single_portafilter", "double_portafilter"):
        return _fail()

    if port == "port_1":
        if not ok(_trace_run_skill("grinder", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
            return _fail()

    if not _run_cached_machine_approach(
        f"grinder:{port}:approach:grinder",
        "espresso_grinder",
        "grinder",
    ):
        return _fail()

    grinder_cache_key = f"{port}_grinder_post_mount"
    cached_grinder_mount_pose = _grinder_post_mount_cache.get(grinder_cache_key)

    if _is_valid_angles(cached_grinder_mount_pose):
        if not ok(_trace_run_skill("grinder", "gotoJ_deg", *cached_grinder_mount_pose)):
            return _fail()
    else:
        if not _run_cached_machine_mount(
            f"grinder:{port}:mount:grinder",
            "espresso_grinder",
            "grinder",
        ):
            return _fail()

        grinder_mount_pose = _trace_run_skill("grinder", "current_angles")
        if not _is_valid_angles(grinder_mount_pose):
            return _fail()

        _grinder_post_mount_cache[grinder_cache_key] = tuple(grinder_mount_pose)

    if not _run_cached_machine_approach(
        f"grinder:{port}:approach:tamper",
        "espresso_grinder",
        "tamper",
    ):
        return _fail()

    if not ok(_trace_run_skill("grinder", "sync")):
        return _fail()

    time.sleep(positioning_time)

    if not _run_cached_machine_mount(
        f"grinder:{port}:mount:grinder:final",
        "espresso_grinder",
        "grinder",
    ):
        return _fail()

    if not _run_cached_machine_mount(
        f"grinder:{port}:mount:tamper",
        "espresso_grinder",
        "tamper",
    ):
        return _fail()

    if not ok(_trace_run_skill("grinder", "sync")):
        return _fail()

    if not ok(_trace_run_skill("grinder", "set_gripper_position", 255, 0, 255)):
        return _fail()

    cached_tool_pick_pose = _tool_pick_pose_cache.get(portafilter_tool)

    if _is_valid_angles(cached_tool_pick_pose):
        if not ok(_trace_run_skill("grinder", "gotoJ_deg", *cached_tool_pick_pose)):
            return _fail()
    else:
        # IMPORTANT:
        # This cache must remain a pre-grab / pre-approach-tool pose.
        # Do not add approach_tool() here, otherwise tamper() will replay
        # the wrong cache semantics.
        if not ok(_trace_run_skill("grinder", "moveEE_movJ", -50, 50, 50, 15, 0, 0)):
            return _fail()

        tool_pick_pose = _trace_run_skill("grinder", "current_angles")
        if not _is_valid_angles(tool_pick_pose):
            return _fail()

        _tool_pick_pose_cache[portafilter_tool] = tuple(tool_pick_pose)

    if not ok(_trace_run_skill("grinder", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    return True

def single_grinder(**params) -> bool:
    _trace_step("single_grinder", "START")
    params["portafilter_tool"] = "single_portafilter"
    return grinder(**params)

def double_grinder(**params) -> bool:
    _trace_step("double_grinder", "START")
    params["portafilter_tool"] = "double_portafilter"
    return grinder(**params)

def tamper(**params) -> bool:
    _trace_step("tamper", "START")

    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    if shot_cfg and shot_cfg.get("angled"):
        return angled_tamper(**params)

    portafilter_tool = (
        params.get("portafilter_tool")
        or (shot_cfg.get("portafilter_tool") if shot_cfg else "double_portafilter")
    )

    if portafilter_tool not in ("single_portafilter", "double_portafilter"):
        return _fail()

    # Normal/non-angled portafilter expected gripper reading.
    # These constants already exist in espresso.py and are used by unmount().
    TAMPER_GRIP_POS_MIN = _PORTAFILTER_GRIP_POS_MIN
    TAMPER_GRIP_POS_MAX = _PORTAFILTER_GRIP_POS_MAX
    TAMPER_GRIP_RETRIES = 5

    def _close_and_verify_grip():
        """
        Close the gripper and verify the stabilized gripper reading.

        Do not compare against commanded position 255 here. When holding a
        portafilter, the real stabilized register should be around the known
        portafilter grip range, not 255.
        """
        _trace_step("tamper._close_and_verify_grip", "START")

        node = init_motion_node()
        success, pos = node.set_gripper_position(
            speed=255,
            position=255,
            force=255,
        )

        if not success:
            return False, None

        gripped_ok = (
            pos is not None
            and TAMPER_GRIP_POS_MIN <= pos <= TAMPER_GRIP_POS_MAX
        )

        return gripped_ok, pos

    def _move_to_tool_pick_pose() -> bool:
        """
        Move to the same pre-grab position that the original tamper() used.

        Cached path:
            gotoJ_deg(_tool_pick_pose_cache[tool])

        Uncached path:
            sync -> move_to(tool, 0.22) -> sync -> approach_tool(tool)

        This preserves the original movement order.
        """
        cached_tool_pick_pose = _tool_pick_pose_cache.get(portafilter_tool)

        if _is_valid_angles(cached_tool_pick_pose):
            if not ok(_trace_run_skill(
                "tamper._move_to_tool_pick_pose",
                "gotoJ_deg",
                *cached_tool_pick_pose,
            )):
                return False
            return True

        if not ok(_trace_run_skill("tamper._move_to_tool_pick_pose", "sync")):
            return False

        if not ok(_trace_run_skill(
            "tamper._move_to_tool_pick_pose",
            "move_to",
            portafilter_tool,
            0.22,
        )):
            return False

        if not ok(_trace_run_skill("tamper._move_to_tool_pick_pose", "sync")):
            return False

        if not ok(_trace_run_skill(
            "tamper._move_to_tool_pick_pose",
            "approach_tool",
            portafilter_tool,
        )):
            return False

        return True

    def _grab_then_close_and_verify():
        """
        Run the original grab sequence, then close and verify the gripper.

        IMPORTANT:
        approach_tool() is not called here. It belongs in
        _move_to_tool_pick_pose() for the uncached path only. This avoids
        duplicate approach_tool() calls.
        """
        _trace_step("tamper._grab_then_close_and_verify", "START")

        if not ok(_trace_run_skill("tamper._grab_then_close_and_verify", "sync")):
            return False, None

        if not ok(_trace_run_skill(
            "tamper._grab_then_close_and_verify",
            "grab_tool",
            portafilter_tool,
        )):
            return False, None

        if not ok(_trace_run_skill("tamper._grab_then_close_and_verify", "sync")):
            return False, None

        return _close_and_verify_grip()

    if not ok(_trace_run_skill("tamper", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    if not _move_to_tool_pick_pose():
        return _fail()

    post_grab_pose = _tamper_post_grab_joints_cache.get(portafilter_tool)

    if _is_valid_angles(post_grab_pose):
        if not ok(_trace_run_skill("tamper", "gotoJ_deg", *post_grab_pose)):
            return _fail()

        if not ok(_trace_run_skill("tamper", "sync")):
            return _fail()

        gripped, pos = _close_and_verify_grip()

        if not gripped:
            _gripper_log.warning(
                f"[TAMPER-GRIP] cached post-grab verification failed: "
                f"tool={portafilter_tool}, pos={pos}, "
                f"expected=[{TAMPER_GRIP_POS_MIN}, {TAMPER_GRIP_POS_MAX}]. "
                f"Invalidating post-grab cache and retrying live pickup."
            )

            _tamper_post_grab_joints_cache.pop(portafilter_tool, None)
            post_grab_pose = None

            if not ok(_trace_run_skill("tamper", "set_gripper_position", 255, 0, 255)):
                return _fail()

            if not ok(_trace_run_skill("tamper", "sync")):
                return _fail()

    if not _is_valid_angles(post_grab_pose):
        gripped = False
        pos = None

        for attempt in range(1, TAMPER_GRIP_RETRIES + 1):
            if attempt > 1:
                _gripper_log.warning(
                    f"[TAMPER-GRIP] retry {attempt}/{TAMPER_GRIP_RETRIES}: "
                    f"tool={portafilter_tool}, previous_pos={pos}, "
                    f"expected=[{TAMPER_GRIP_POS_MIN}, {TAMPER_GRIP_POS_MAX}]"
                )

                if not ok(_trace_run_skill("tamper", "set_gripper_position", 255, 0, 255)):
                    return _fail()

                if not ok(_trace_run_skill("tamper", "sync")):
                    return _fail()

                if not _move_to_tool_pick_pose():
                    return _fail()

            gripped, pos = _grab_then_close_and_verify()

            if gripped:
                _gripper_log.info(
                    f"[TAMPER-GRIP] grip verified: "
                    f"tool={portafilter_tool}, pos={pos}, "
                    f"expected=[{TAMPER_GRIP_POS_MIN}, {TAMPER_GRIP_POS_MAX}]"
                )
                break

        if not gripped:
            _gripper_log.error(
                f"[TAMPER-GRIP] FINAL FAIL: "
                f"tool={portafilter_tool}, pos={pos}, "
                f"expected=[{TAMPER_GRIP_POS_MIN}, {TAMPER_GRIP_POS_MAX}]"
            )
            return _fail()

        post_grab_angles = _trace_run_skill("tamper", "current_angles")
        if not _is_valid_angles(post_grab_angles):
            return _fail()

        _tamper_post_grab_joints_cache[portafilter_tool] = tuple(post_grab_angles)

    if not ok(_trace_run_skill("tamper", "moveEE", 0, 0, 40, 0, 0, 0)):
        return _fail()

    if not _run_cached_machine_mount(
        f"tamper:{portafilter_tool}:mount:grinder",
        "espresso_grinder",
        "grinder",
    ):
        return _fail()

    if not _run_cached_machine_approach(
        f"tamper:{portafilter_tool}:approach:grinder",
        "espresso_grinder",
        "grinder",
    ):
        return _fail()

    if not ok(_trace_run_skill("tamper", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    return True

def single_tamper(**params) -> bool:
    _trace_step("single_tamper", "START")
    params["portafilter_tool"] = "single_portafilter"
    return tamper(**params)

def double_tamper(**params) -> bool:
    _trace_step("double_tamper", "START")
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_tamper(**params)
    params["portafilter_tool"] = "double_portafilter"
    return tamper(**params)

def mount(**params) -> bool:
    global below_espresso_port, mount_espresso_port

    _trace_step("mount", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_mount(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_1")

    if not port:
        return _fail()

    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return _fail()

    if port in ('port_2', 'port_3'):
        if not ok(_trace_run_skill("mount", "gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])):
            return _fail()

    if not ok(_trace_run_skill("mount", "gotoJ_deg", *port_params['move_back'])):
        return _fail()

    runtime_cached = _mount_runtime_cache.get(port)
    if runtime_cached:
        below_pose = runtime_cached.get('below')
        mount_pose = runtime_cached.get('mount')
    else:
        below_pose = below_espresso_port
        mount_pose = mount_espresso_port

    if not _is_valid_angles(below_pose):
        return _fail()

    if not ok(_trace_run_skill("mount", "gotoJ_deg", *below_pose)):
        return _fail()

    if not _is_valid_angles(mount_pose):
        return _fail()

    if not ok(_trace_run_skill("mount", "gotoJ_deg", *mount_pose)):
        return _fail()

    if not ok(_trace_run_skill("mount", "moveEE_movJ", *_portafilter_clear_up_offset(port))):
        return _fail()

    if not ok(_trace_run_skill("mount", "enforce_rxry")):
        return _fail()

    if not ok(_trace_run_skill("mount", "sync")):
        return _fail()

    arc_delta_mount = _portafilter_mount_arc_cmd_by_port.get(str(port))
    if arc_delta_mount is None:
        return _fail()

    if not ok(_trace_run_skill("mount", "move_portafilter_arc_movJ", arc_delta_mount)):
        return _fail()
    
    if not ok(_trace_run_skill("mount", "sync")):
        return _fail()
    
    if not ok(_trace_run_skill("mount", "release_tension")):
        return _fail()

    if not _open_gripper_with_verify():
        return _fail()

    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return _fail()

    if not ok(_trace_run_skill("mount", "sync")):
        return _fail()

    if port in ('port_1', 'port_3'):
        if not _run_cached_machine_approach(
            f"mount:{port}:approach:{port_params['portafilter_number']}",
            "three_group_espresso",
            port_params['portafilter_number'],
        ):
            return _fail()

    if not ok(_trace_run_skill("mount", "gotoJ_deg", *port_params['home'])):
        return _fail()

    return True

def grab_espresso_pitcher(**params) -> bool:
    """
    Grab the espresso pitcher and stop right after closing the gripper.
    """
    _trace_step("grab_espresso_pitcher", "START")

    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    if shot_cfg and shot_cfg.get("angled"):
        return angled_grab_espresso_pitcher(**params)

    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_1")

    if not port or port not in ("port_1", "port_2", "port_3"):
        return _fail()

    if not ok(_trace_run_skill("grab_espresso_pitcher", "gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()

    cached = _pitcher_pick_cache.get(port)

    pick2 = cached.get("pick2_approach") if cached else None
    if _is_valid_angles(pick2):
        if not ok(_trace_run_skill("grab_espresso_pitcher", "gotoJ_deg", *pick2)):
            return _fail()
    else:
        if not _run_cached_machine_approach(
            f"grab_pitcher:{port}:approach:pick_pitcher_2",
            "three_group_espresso",
            "pick_pitcher_2",
        ):
            return _fail()

        if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
            return _fail()

        pick2_angles = _trace_run_skill("grab_espresso_pitcher", "current_angles")
        if _is_valid_angles(pick2_angles):
            _pitcher_pick_cache.setdefault(port, {})["pick2_approach"] = tuple(pick2_angles)

    cached = _pitcher_pick_cache.get(port)

    if port == "port_1":
        if cached and cached.get("approach") and cached.get("mount"):
            if not ok(_trace_run_skill("grab_espresso_pitcher", "gotoJ_deg", *cached["approach"])):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "gotoJ_deg", *cached["mount"])):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "set_gripper_position", 255, 115, 255)):
                return _fail()

        else:
            if not _run_cached_machine_approach(
                f"grab_pitcher:{port}:approach:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
                return _fail()

            approach_angles = _trace_run_skill("grab_espresso_pitcher", "current_angles")
            if _is_valid_angles(approach_angles):
                _pitcher_pick_cache.setdefault(port, {})["approach"] = tuple(approach_angles)

            if not _run_cached_machine_mount(
                f"grab_pitcher:{port}:mount:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "set_gripper_position", 255, 115, 255)):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "release_tension")):
                return _fail()

            mount_angles = _trace_run_skill("grab_espresso_pitcher", "current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_pick_cache.setdefault(port, {})["mount"] = tuple(mount_angles)

    elif port == "port_2":
        if cached and cached.get("mount"):
            if not ok(_trace_run_skill("grab_espresso_pitcher", "gotoJ_deg", *cached["mount"])):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
                return _fail()

        else:
            if not _run_cached_machine_mount(
                f"grab_pitcher:{port}:mount:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
                return _fail()

            mount_angles = _trace_run_skill("grab_espresso_pitcher", "current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_pick_cache.setdefault(port, {})["mount"] = tuple(mount_angles)

        if not ok(_trace_run_skill("grab_espresso_pitcher", "set_gripper_position", 255, 115, 255)):
            return _fail()

    elif port == "port_3":
        if cached and cached.get("approach") and cached.get("mount"):
            if not ok(_trace_run_skill("grab_espresso_pitcher", "gotoJ_deg", *cached["approach"])):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "gotoJ_deg", *cached["mount"])):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
                return _fail()

        else:
            if not _run_cached_machine_approach(
                f"grab_pitcher:{port}:approach:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
                return _fail()

            approach_angles = _trace_run_skill("grab_espresso_pitcher", "current_angles")
            if _is_valid_angles(approach_angles):
                _pitcher_pick_cache.setdefault(port, {})["approach"] = tuple(approach_angles)

            if not _run_cached_machine_mount(
                f"grab_pitcher:{port}:mount:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()

            if not ok(_trace_run_skill("grab_espresso_pitcher", "sync")):
                return _fail()

            mount_angles = _trace_run_skill("grab_espresso_pitcher", "current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_pick_cache.setdefault(port, {})["mount"] = tuple(mount_angles)

        if not ok(_trace_run_skill("grab_espresso_pitcher", "set_gripper_position", 255, 115, 255)):
            return _fail()

    else:
        return _fail()
    
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return _fail()

    return True

def pick_espresso_pitcher(**params) -> bool:
    """
    Complete pitcher pickup after grab_espresso_pitcher().
    """
    _trace_step("pick_espresso_pitcher", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_pick_espresso_pitcher(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_1")

    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return _fail()

    cached = _pitcher_pick_cache.get(port)

    if not ok(_trace_run_skill("pick_espresso_pitcher", "sync")):
        return _fail()
    if not ok(_trace_run_skill("pick_espresso_pitcher", "set_speed_factor", 35)):#25
        return _fail()

    if port == 'port_1':
        if cached and cached.get('retreat'):
            if not ok(_trace_run_skill("pick_espresso_pitcher", "gotoJ_deg", *cached['retreat'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"pick_pitcher:{port}:retreat:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()
            if not ok(_trace_run_skill("pick_espresso_pitcher", "sync")):
                return _fail()
            retreat_angles = _trace_run_skill("pick_espresso_pitcher", "current_angles")
            if _is_valid_angles(retreat_angles):
                _pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_2':
        if cached and cached.get('retreat'):
            if not ok(_trace_run_skill("pick_espresso_pitcher", "gotoJ_deg", *cached['retreat'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"pick_pitcher:{port}:retreat:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return _fail()
            if not ok(_trace_run_skill("pick_espresso_pitcher", "sync")):
                return _fail()
            retreat_angles = _trace_run_skill("pick_espresso_pitcher", "current_angles")
            if _is_valid_angles(retreat_angles):
                _pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_3':
        if cached and cached.get('retreat'):
            if not ok(_trace_run_skill("pick_espresso_pitcher", "gotoJ_deg", *cached['retreat'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"pick_pitcher:{port}:retreat:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()
            if not ok(_trace_run_skill("pick_espresso_pitcher", "sync")):
                return _fail()
            retreat_angles = _trace_run_skill("pick_espresso_pitcher", "current_angles")
            if _is_valid_angles(retreat_angles):
                _pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    if port in ('port_1', 'port_2'):
        if not ok(_trace_run_skill("pick_espresso_pitcher", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
            return _fail()

    return True

def pour_espresso_pitcher_cup_station(**params) -> bool:
    _trace_step("pour_espresso_pitcher_cup_station", "START")
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"

    if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_FLOW_POSES['cup_station_entry'])):
        return _fail()

    if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return _fail()

    if stage == 'stage_1':
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos1'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "set_speed_factor", SPEED_SLOW_POURING)):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.1'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.2'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.3'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "set_speed_factor", 100)):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral1'])):
            return _fail()
    elif stage == 'stage_2':
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos2'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "set_speed_factor", SPEED_SLOW_POURING)):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.1'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.2'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.3'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "set_speed_factor", 100)):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral2'])):
            return _fail()
    elif stage == 'stage_3':
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos3'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "set_speed_factor", SPEED_SLOW_POURING)):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.1'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.2'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.3'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "set_speed_factor", 100)):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral3'])):
            return _fail()
    else:
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos4'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "set_speed_factor", SPEED_SLOW_POURING)):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.1'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.2'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.3'])):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "set_speed_factor", 100)):
            return _fail()
        if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral4'])):
            return _fail()

    if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return _fail()

    if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_FLOW_POSES['cup_station_entry'])):
        return _fail()

    if not ok(_trace_run_skill("pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
        return _fail()

    return True

def get_hot_water(**params) -> bool:
    _trace_step("get_hot_water", "START")
    def ok(r):
        return r not in (False, None)

    if not _run_cached_machine_approach(
        "get_hot_water:approach:hot_water",
        "three_group_espresso",
        "hot_water",
    ):
        return _fail()

    if not _run_cached_machine_mount(
        "get_hot_water:mount:hot_water",
        "three_group_espresso",
        "hot_water",
    ):
        return _fail()

    if not ok(_trace_run_skill("get_hot_water", "moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_move'])):
        return _fail()
    return True

def with_hot_water(**params) -> bool:
    _trace_step("with_hot_water", "START")
    def ok(r):
        return r not in (False, None)

    if not ok(_trace_run_skill("with_hot_water", "set_speed_factor", ESPRESSO_SPEEDS['hot_water_pour'])):
        return _fail()

    if not ok(_trace_run_skill("with_hot_water", "moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return _fail()

    return True

def return_espresso_pitcher(**params) -> bool:
    global approach_pitcher, pick_pitcher

    _trace_step("return_espresso_pitcher", "START")

    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    if shot_cfg and shot_cfg.get("angled"):
        return angled_return_espresso_pitcher(**params)

    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_1")

    if not port or port not in ("port_1", "port_2", "port_3"):
        return _fail()

    # Important:
    # This is separate from _pitcher_pick_cache used by grab_espresso_pitcher().
    # So grab_espresso_pitcher() being called first does not automatically make
    # return_espresso_pitcher() cached.
    return_cache = _pitcher_return_cache.setdefault(port, {})

    if port == "port_1":
        if _is_valid_angles(return_cache.get("approach")):
            if not ok(_trace_run_skill("return_espresso_pitcher", "gotoJ_deg", *return_cache["approach"])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"return_pitcher:{port}:approach:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()

            if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
                return _fail()

            approach_angles = _trace_run_skill("return_espresso_pitcher", "current_angles")
            if _is_valid_angles(approach_angles):
                return_cache["approach"] = tuple(approach_angles)

        if _is_valid_angles(return_cache.get("mount")):
            if not ok(_trace_run_skill("return_espresso_pitcher", "gotoJ_deg", *return_cache["mount"])):
                return _fail()
        else:
            if not _run_cached_machine_mount(
                f"return_pitcher:{port}:mount:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()

            # Runs only the first time this return mount is learned.
            if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
                return _fail()

            if not ok(_trace_run_skill("return_espresso_pitcher", "release_tension")):
                return _fail()

            if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
                return _fail()

            mount_angles = _trace_run_skill("return_espresso_pitcher", "current_angles")
            if _is_valid_angles(mount_angles):
                return_cache["mount"] = tuple(mount_angles)

        if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
            return _fail()

        if not ok(_trace_run_skill("return_espresso_pitcher", "set_gripper_position", 35, 0, 255)):
            return _fail()

        if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
            return _fail()

        if _is_valid_angles(return_cache.get("retreat")):
            if not ok(_trace_run_skill("return_espresso_pitcher", "gotoJ_deg", *return_cache["retreat"])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"return_pitcher:{port}:retreat:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()

            if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
                return _fail()

            retreat_angles = _trace_run_skill("return_espresso_pitcher", "current_angles")
            if _is_valid_angles(retreat_angles):
                return_cache["retreat"] = tuple(retreat_angles)

    elif port == "port_2":
        if _is_valid_angles(return_cache.get("mount")):
            if not ok(_trace_run_skill("return_espresso_pitcher", "gotoJ_deg", *return_cache["mount"])):
                return _fail()
        else:
            if not _run_cached_machine_mount(
                f"return_pitcher:{port}:mount:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return _fail()

            if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
                return _fail()

            mount_angles = _trace_run_skill("return_espresso_pitcher", "current_angles")
            if _is_valid_angles(mount_angles):
                return_cache["mount"] = tuple(mount_angles)

        if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
            return _fail()

        if not ok(_trace_run_skill("return_espresso_pitcher", "set_gripper_position", 35, 0, 255)):
            return _fail()

        if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
            return _fail()

    elif port == "port_3":
        if _is_valid_angles(return_cache.get("approach")):
            if not ok(_trace_run_skill("return_espresso_pitcher", "gotoJ_deg", *return_cache["approach"])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"return_pitcher:{port}:approach:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()

            if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
                return _fail()

            approach_angles = _trace_run_skill("return_espresso_pitcher", "current_angles")
            if _is_valid_angles(approach_angles):
                return_cache["approach"] = tuple(approach_angles)

        if _is_valid_angles(return_cache.get("mount")):
            if not ok(_trace_run_skill("return_espresso_pitcher", "gotoJ_deg", *return_cache["mount"])):
                return _fail()
        else:
            if not _run_cached_machine_mount(
                f"return_pitcher:{port}:mount:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()

            if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
                return _fail()

            mount_angles = _trace_run_skill("return_espresso_pitcher", "current_angles")
            if _is_valid_angles(mount_angles):
                return_cache["mount"] = tuple(mount_angles)

        if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
            return _fail()

        if not ok(_trace_run_skill("return_espresso_pitcher", "set_gripper_position", 35, 0, 255)):
            return _fail()

        if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
            return _fail()

        if _is_valid_angles(return_cache.get("retreat")):
            if not ok(_trace_run_skill("return_espresso_pitcher", "gotoJ_deg", *return_cache["retreat"])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"return_pitcher:{port}:retreat:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()

            if not ok(_trace_run_skill("return_espresso_pitcher", "sync")):
                return _fail()

            retreat_angles = _trace_run_skill("return_espresso_pitcher", "current_angles")
            if _is_valid_angles(retreat_angles):
                return_cache["retreat"] = tuple(retreat_angles)

    if not _run_cached_machine_approach(
        f"return_pitcher:{port}:final_approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return _fail()

    if not ok(_trace_run_skill("return_espresso_pitcher", "gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()

    return True

def return_cleaned_espresso_pitcher(**params) -> bool:
    _trace_step("return_cleaned_espresso_pitcher", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_return_cleaned_espresso_pitcher(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_1")

    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return _fail()

    if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()

    if not _run_cached_machine_approach(
        f"return_clean_pitcher:{port}:approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return _fail()

    cached = _pitcher_clean_cache.get(port)

    if port == 'port_1':
        if not _run_cached_machine_approach(
            f"return_clean_pitcher:{port}:approach:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return _fail()
        if not _run_cached_machine_mount(
            f"return_clean_pitcher:{port}:mount:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "set_gripper_position", 255,115,255)):
            return _fail()
        if cached:
            for angles in cached:
                if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "gotoJ_deg", *angles)):
                    return _fail()
        else:
            waypoints = []
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, 20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, -170)):
                return _fail()
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, 170)):
                return _fail()
            _trace_run_skill("return_cleaned_espresso_pitcher", "sync")
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, -20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if all(_is_valid_angles(w) for w in waypoints):
                _pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "set_gripper_position", 35,0,255)):
            return _fail()
        if not _run_cached_machine_approach(
            f"return_clean_pitcher:{port}:final_approach:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return _fail()

    elif port == 'port_2':
        if not _run_cached_machine_mount(
            f"return_clean_pitcher:{port}:mount:pick_pitcher_2",
            "three_group_espresso",
            "pick_pitcher_2",
        ):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "set_gripper_position", 255,115,255)):
            return _fail()
        if cached:
            for angles in cached:
                if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "gotoJ_deg", *angles)):
                    return _fail()
        else:
            waypoints = []
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, 20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, -170)):
                return _fail()
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, 170)):
                return _fail()
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, -20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if all(_is_valid_angles(w) for w in waypoints):
                _pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "set_gripper_position", 35,0,255)):
            return _fail()

    elif port == 'port_3':
        if not _run_cached_machine_approach(
            f"return_clean_pitcher:{port}:approach:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return _fail()
        if not _run_cached_machine_mount(
            f"return_clean_pitcher:{port}:mount:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "set_gripper_position", 255,115,255)):
            return _fail()
        if cached:
            for angles in cached:
                if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "gotoJ_deg", *angles)):
                    return _fail()
        else:
            waypoints = []
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, 20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, -135)):
                return _fail()
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, 135)):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, -20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("return_cleaned_espresso_pitcher", "current_angles"))
            if all(_is_valid_angles(w) for w in waypoints):
                _pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "set_gripper_position", 35,0,255)):
            return _fail()
        if not _run_cached_machine_approach(
            f"return_clean_pitcher:{port}:final_approach:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return _fail()

    if not _run_cached_machine_approach(
        f"return_clean_pitcher:{port}:final_approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return _fail()

    if not ok(_trace_run_skill("return_cleaned_espresso_pitcher", "gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()

    return True

def unmount_single(**params) -> bool:
    _trace_step("unmount_single", "START")
    params["port"] = "port_2"
    return unmount(**params)

def unmount_double(**params) -> bool:
    _trace_step("unmount_double", "START")
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_unmount(**params)
    params["port"] = "port_1"
    return unmount(**params)

def mount_single(**params) -> bool:
    _trace_step("mount_single", "START")
    params["port"] = "port_2"
    return mount(**params)

def mount_double(**params) -> bool:
    _trace_step("mount_double", "START")
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_mount(**params)
    params["port"] = "port_1"
    return mount(**params)

def single_grab_espresso_pitcher(**params) -> bool:
    _trace_step("single_grab_espresso_pitcher", "START")
    params["port"] = "port_2"
    return grab_espresso_pitcher(**params)

def double_grab_espresso_pitcher(**params) -> bool:
    _trace_step("double_grab_espresso_pitcher", "START")
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_grab_espresso_pitcher(**params)
    params["port"] = "port_1"
    return grab_espresso_pitcher(**params)

def single_pick_espresso_pitcher(**params) -> bool:
    _trace_step("single_pick_espresso_pitcher", "START")
    params["port"] = "port_2"
    return pick_espresso_pitcher(**params)

def double_pick_espresso_pitcher(**params) -> bool:
    _trace_step("double_pick_espresso_pitcher", "START")
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_pick_espresso_pitcher(**params)
    params["port"] = "port_1"
    return pick_espresso_pitcher(**params)

def single_pour_espresso_pitcher_cup_station(**params) -> bool:
    _trace_step("single_pour_espresso_pitcher_cup_station", "START")
    params["port"] = "port_2"
    return pour_espresso_pitcher_cup_station(**params)

def double_pour_espresso_pitcher_cup_station(**params) -> bool:
    _trace_step("double_pour_espresso_pitcher_cup_station", "START")
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_pour_espresso_pitcher_cup_station(**params)
    params["port"] = "port_1"
    return pour_espresso_pitcher_cup_station(**params)

def single_return_espresso_pitcher(**params) -> bool:
    _trace_step("single_return_espresso_pitcher", "START")
    params["port"] = "port_2"
    return return_espresso_pitcher(**params)

def double_return_espresso_pitcher(**params) -> bool:
    _trace_step("double_return_espresso_pitcher", "START")
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_return_espresso_pitcher(**params)
    params["port"] = "port_1"
    return return_espresso_pitcher(**params)

def single_return_cleaned_espresso_pitcher(**params) -> bool:
    _trace_step("single_return_cleaned_espresso_pitcher", "START")
    params["port"] = "port_2"
    return return_cleaned_espresso_pitcher(**params)

def double_return_cleaned_espresso_pitcher(**params) -> bool:
    _trace_step("double_return_cleaned_espresso_pitcher", "START")
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_return_cleaned_espresso_pitcher(**params)
    params["port"] = "port_1"
    return return_cleaned_espresso_pitcher(**params)

"""
angled_espresso.py

Parallel definitions prefixed with angled_; same behavior as the espresso section above until customized.
"""

# Global variables to store captured positions during angled_unmount sequence
angled_below_espresso_port: Optional[Tuple[float, ...]] = None
angled_mount_espresso_port: Optional[Tuple[float, ...]] = None
angled_mount_espresso_pose: Optional[Tuple[float, ...]] = None  # Cartesian pose at angled_mount position
angled_approach_pitcher: Optional[Tuple[float, ...]] = None
angled_pick_pitcher: Optional[Tuple[float, ...]] = None

angled__port_angle_cache: Dict[str, Any] = {}
# OLD / remove if you want later:
# angled_grinder_mount_pose_cached: Dict[str, Any] = {}
angled__pitcher_clean_cache: Dict[str, Any] = {}
angled__pitcher_pick_cache: Dict[str, Any] = {}
angled__pitcher_return_cache: Dict[str, Any] = {}
angled__tool_pick_pose_cache: Dict[str, Any] = {}
# BEST: dedicated angled grinder cache so grinder mount replay stays port-specific
# and does not share namespace with tool-pick poses.
angled__grinder_post_mount_cache: Dict[str, Any] = {}
angled__mount_runtime_cache: Dict[str, Any] = {}

def angled_invalidate_port_cache():
    _trace_step("angled_invalidate_port_cache", "START")
    angled__port_angle_cache.clear()
    angled__pitcher_clean_cache.clear()
    angled__pitcher_pick_cache.clear()
    angled__pitcher_return_cache.clear()
    angled__tool_pick_pose_cache.clear()
    angled__grinder_post_mount_cache.clear()
    angled__mount_runtime_cache.clear()
    _machine_approach_pose_cache.clear()
    _machine_mount_pose_cache.clear()
    _tamper_post_grab_joints_cache.clear()
    _angled_tamper_post_grab_joints_cache.clear()
    angled__portafilter_arc_cmd_by_port.clear()
    angled__portafilter_mount_arc_cmd_by_port.clear()
    angled__portafilter_arc_cmd_by_port_2.clear()
    for _k in list(_unmount_post_grab_joints_cache.keys()):
        if str(_k).startswith("angled_unmount:"):
            del _unmount_post_grab_joints_cache[_k]
    for _k in list(_portafilter_clear_up_z_mm_by_port.keys()):
        if str(_k).startswith("angled_portafilter"):
            del _portafilter_clear_up_z_mm_by_port[_k]
    # OLD / remove if you want later:
    # angled_grinder_mount_pose_cached.clear()

def angled__is_valid_angles(angles: Any) -> bool:
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6
    
def angled__normalize_espresso_shot(espresso_dict: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    _trace_step("angled__normalize_espresso_shot", "START")
    try:
        if not espresso_dict or not isinstance(espresso_dict, dict):
            return None

        espresso_key = next(iter(espresso_dict.keys()), None)
        if not espresso_key:
            return None

        espresso_key_lower = str(espresso_key).lower()

        if 'single' in espresso_key_lower:
            return {
                "port": "angled_portafilter_2",
                "positioning_time": 1.2,
                "portafilter_tool": "single_portafilter_angled",
            }
        elif 'double' in espresso_key_lower:
            return {
                "port": "angled_portafilter_1",
                "positioning_time": 2.4,
                "portafilter_tool": "double_portafilter_angled",
            }
        else:
            value = espresso_dict.get(espresso_key)
            if value is not None:
                shots = float(value)
                if shots <= 1.0:
                    return {
                        "port": "angled_portafilter_2",
                        "positioning_time": 1.2,
                        "portafilter_tool": "single_portafilter_angled",
                    }
                else:
                    return {
                        "port": "angled_portafilter_1",
                        "positioning_time": 2.4,
                        "portafilter_tool": "double_portafilter_angled",
                    }
    except Exception as e:
        print(f"[WARNING] Error parsing espresso parameters: {e}")
        return None

    return None

def angled_unmount(**params) -> bool:
    global angled_below_espresso_port, angled_mount_espresso_port

    _trace_step("angled_unmount", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "angled_portafilter_2")
    grab_tool_name = (
        params.get("portafilter_tool")
        or (shot_cfg.get("portafilter_tool") if shot_cfg else None)
        or _angled_unmount_grab_tool_name(port)
    )

    if not port:
        return _fail()

    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return _fail()

    if not ok(_trace_run_skill("angled_unmount", "gotoJ_deg", *port_params['home'])):
        return _fail()

    if port == 'angled_portafilter_2':
        if not _run_cached_machine_approach(
            f"angled_unmount:{port}:approach:{port_params['portafilter_number']}",
            "three_group_espresso",
            port_params['portafilter_number'],
        ):
            return _fail()

    grab_cache_key = f"angled_unmount:post_grab:{port}"
    cached_grab_joints = _unmount_post_grab_joints_cache.get(grab_cache_key)

    if _is_valid_angles(cached_grab_joints):
        if not ok(_trace_run_skill("angled_unmount", "gotoJ_deg", *cached_grab_joints)):
            return _fail()

        if not ok(_trace_run_skill("angled_unmount", "sync")):
            return _fail()

        if not ok(_trace_run_skill("angled_unmount", "set_gripper_position", 255, 255, 255)):
            return _fail()

    else:
        ANGLED_UNMOUNT_GRIP_POS_MIN = 138
        ANGLED_UNMOUNT_GRIP_POS_MAX = 143
        ANGLED_UNMOUNT_GRIP_RETRIES = 15

        def trace_grip(msg: str):
            print(f"[ANGLED-UNMOUNT-GRIP] {msg}", flush=True)

        def _close_and_verify_grip_angled():
            _trace_step("angled_unmount._close_and_verify_grip_angled", "START")
            node = init_motion_node()
            success, pos = node.set_gripper_position(speed=255, position=255, force=255)
            if not success:
                return False, None

            ok_reading = (
                pos is not None
                and ANGLED_UNMOUNT_GRIP_POS_MIN <= pos <= ANGLED_UNMOUNT_GRIP_POS_MAX
            )
            return ok_reading, pos

        def _grab_then_close_angled(num_samples: int = 9, max_wait: float = 25.0):
            _trace_step(
                "angled_unmount._grab_then_close_angled",
                f"START num_samples={num_samples} max_wait={max_wait:.1f}",
            )
            if not ok(_trace_run_skill("angled_unmount._grab_then_close_angled", "sync")):
                return False, None

            # Bypass run_skill for grab_tool so we can pass num_samples/max_wait as
            # kwargs (run_skill only forwards positional args). Tracing is preserved
            # via explicit prints to keep the log shape consistent with other steps.
            print(
                f"[TRACE:angled_unmount._grab_then_close_angled] "
                f"grab_tool('{grab_tool_name}', num_samples={num_samples}, "
                f"max_wait={max_wait:.1f}) START",
                flush=True,
            )
            node = init_motion_node()
            grab_ok = node.grab_tool(
                grab_tool_name,
                num_samples=num_samples,
                max_wait=max_wait,
            )
            print(
                f"[TRACE:angled_unmount._grab_then_close_angled] "
                f"grab_tool DONE result={grab_ok}",
                flush=True,
            )
            if not ok(grab_ok):
                return False, None

            if not ok(_trace_run_skill("angled_unmount._grab_then_close_angled", "sync")):
                return False, None
            return _close_and_verify_grip_angled()

        def _rerun_approach_for_retry(attempt_idx: int) -> bool:
            _trace_step("angled_unmount._rerun_approach_for_retry", "START")
            trace_grip(f"full routine attempt {attempt_idx}: open gripper START")
            if not ok(_trace_run_skill("angled_unmount._rerun_approach_for_retry", "set_gripper_position", 255, 0, 255)):
                trace_grip(f"full routine attempt {attempt_idx}: FAIL open gripper")
                return _fail()
            trace_grip(f"full routine attempt {attempt_idx}: open gripper DONE")

            if not ok(_trace_run_skill("angled_unmount._rerun_approach_for_retry", "sync")):
                return _fail()
            trace_grip(f"full routine attempt {attempt_idx}: sync after open DONE")

            if port == 'angled_portafilter_2':
                trace_grip(
                    f"full routine attempt {attempt_idx}: re-run cached machine approach START "
                    f"target={port_params['portafilter_number']}"
                )

                if not _run_cached_machine_approach(
                    f"angled_unmount:{port}:approach:{port_params['portafilter_number']}",
                    "three_group_espresso",
                    port_params['portafilter_number'],
                ):
                    trace_grip(f"full routine attempt {attempt_idx}: FAIL cached machine approach")
                    return _fail()

                trace_grip(f"full routine attempt {attempt_idx}: re-run cached machine approach DONE")
            else:
                trace_grip(
                    f"full routine attempt {attempt_idx}: machine approach SKIPPED for port={port}"
                )

            return True

        routine_success = False
        final_pos = None
        final_angles = None

        # attempt 0 = first try, attempts 1..15 = retries
        for attempt_idx in range(0, ANGLED_UNMOUNT_GRIP_RETRIES + 1):
            display_attempt = attempt_idx + 1
            total_attempts = ANGLED_UNMOUNT_GRIP_RETRIES + 1

            trace_grip(
                f"full routine attempt {display_attempt}/{total_attempts} START "
                f"tool={grab_tool_name}, port={port}, "
                f"target_pos=[{ANGLED_UNMOUNT_GRIP_POS_MIN}, {ANGLED_UNMOUNT_GRIP_POS_MAX}]"
            )

            if attempt_idx > 0:
                if not _rerun_approach_for_retry(display_attempt):
                    return _fail()

            # Per-attempt sample ramp for the angled single portafilter only.
            # 9 + 3*x grows the consecutive-stable-window requirement on each retry
            # so a marginally jittery marker eventually averages to a settled pose
            # without relaxing trans/rot tolerances. max_wait scales with samples
            # because acquire_target_transform clears the buffer on instability and
            # therefore needs more time to find a longer continuous stable window.
            if grab_tool_name == "single_portafilter_angled":
                this_num_samples = 9 + 3 * attempt_idx
                this_max_wait = max(25.0, this_num_samples * 2.5)
            else:
                this_num_samples = 9
                this_max_wait = 25.0

            trace_grip(
                f"full routine attempt {display_attempt}: grab_then_close START "
                f"num_samples={this_num_samples} max_wait={this_max_wait:.1f}"
            )
            gripped, pos = _grab_then_close_angled(
                num_samples=this_num_samples,
                max_wait=this_max_wait,
            )
            trace_grip(
                f"full routine attempt {display_attempt}: grab_then_close RESULT "
                f"gripped={gripped}, pos={pos}"
            )

            if not gripped:
                _gripper_log.warning(
                    f"[ANGLED-PORTAFILTER-GRIP] full routine attempt "
                    f"{display_attempt}/{total_attempts} failed at initial grip "
                    f"pos={pos} "
                    f"(want in [{ANGLED_UNMOUNT_GRIP_POS_MIN}, {ANGLED_UNMOUNT_GRIP_POS_MAX}]); "
                    f"will retry whole grab routine"
                )
                continue

            trace_grip(
                f"full routine attempt {display_attempt}: initial grip OK pos={pos}; "
                f"release_tension START"
            )

            if not ok(_trace_run_skill("angled_unmount", "release_tension")):
                trace_grip(f"full routine attempt {display_attempt}: FAIL release_tension")
                return _fail()

            if not ok(_trace_run_skill("angled_unmount", "sync")):
                return _fail()
            trace_grip(f"full routine attempt {display_attempt}: release_tension DONE")

            z_tgt = _UNMOUNT_POST_TENSION_Z_TARGET_ANGL_MM
            z_lo = z_tgt - _UNMOUNT_POST_TENSION_Z_TOL_MM
            z_hi = z_tgt + _UNMOUNT_POST_TENSION_Z_TOL_MM

            pose_z = _trace_run_skill("angled_unmount", "current_pose")
            if not ok(pose_z) or not isinstance(pose_z, (tuple, list)) or len(pose_z) < 3:
                trace_grip(f"full routine attempt {display_attempt}: FAIL current_pose for Z check")
                return _fail()

            z_mm = float(pose_z[2])

            trace_grip(
                f"full routine attempt {display_attempt}: Z check "
                f"z={z_mm:.2f}, target={z_tgt:.2f}, range=[{z_lo:.2f}, {z_hi:.2f}]"
            )

            if not (z_lo <= z_mm <= z_hi):
                dz = (z_tgt - z_mm) / 5.0
                dx = -0.315298 * dz

                _gripper_log.warning(
                    f"[ANGLED-UNMOUNT-Z] attempt={display_attempt} "
                    f"after release_tension z={z_mm:.2f} mm outside [{z_lo:.1f}, {z_hi:.1f}]; "
                    f"moveEE_movJ dz={dz:.2f} mm, dx={dx:.2f} mm"
                )

                trace_grip(
                    f"full routine attempt {display_attempt}: Z correction START "
                    f"dx={dx:.2f}, dz={dz:.2f}"
                )

                if not ok(_trace_run_skill("angled_unmount", "set_gripper_position", 25, 100, 25)):
                    trace_grip(f"full routine attempt {display_attempt}: FAIL loosen gripper before Z correction")
                    return _fail()

                if not ok(_trace_run_skill("angled_unmount", "moveEE_movJ", dx, 0, dz, 0, 0, 0)):
                    trace_grip(f"full routine attempt {display_attempt}: FAIL moveEE_movJ Z correction")
                    return _fail()

                if not ok(_trace_run_skill("angled_unmount", "set_gripper_position", 255, 255, 255)):
                    trace_grip(f"full routine attempt {display_attempt}: FAIL re-close gripper after Z correction")
                    return _fail()

                if not ok(_trace_run_skill("angled_unmount", "sync")):
                    return _fail()

                if not ok(_trace_run_skill("angled_unmount", "release_tension")):
                    trace_grip(f"full routine attempt {display_attempt}: FAIL release_tension after Z correction")
                    return _fail()

                if not ok(_trace_run_skill("angled_unmount", "sync")):
                    return _fail()
                trace_grip(f"full routine attempt {display_attempt}: Z correction DONE")
            else:
                trace_grip(f"full routine attempt {display_attempt}: Z correction SKIPPED")

            trace_grip(
                f"full routine attempt {display_attempt}: final post-release grip verify START"
            )

            final_gripped, final_pos = _close_and_verify_grip_angled()

            trace_grip(
                f"full routine attempt {display_attempt}: final post-release grip verify RESULT "
                f"gripped={final_gripped}, pos={final_pos}"
            )

            if not final_gripped:
                _gripper_log.warning(
                    f"[ANGLED-PORTAFILTER-GRIP] full routine attempt "
                    f"{display_attempt}/{total_attempts} failed at final post-release verify "
                    f"pos={final_pos} "
                    f"(want in [{ANGLED_UNMOUNT_GRIP_POS_MIN}, {ANGLED_UNMOUNT_GRIP_POS_MAX}]); "
                    f"retrying whole grab routine"
                )
                continue

            angles = _trace_run_skill("angled_unmount", "current_angles")
            if not ok(angles) or not _is_valid_angles(angles):
                trace_grip(f"full routine attempt {display_attempt}: FAIL current_angles after final verify")
                return _fail()

            final_angles = tuple(angles)
            routine_success = True

            trace_grip(
                f"full routine attempt {display_attempt}: SUCCESS final_pos={final_pos}; "
                f"ready to cache post-grab joints"
            )

            break

        if not routine_success:
            trace_grip(
                f"FINAL FAIL after {ANGLED_UNMOUNT_GRIP_RETRIES + 1} full routine attempts: "
                f"tool={grab_tool_name}, port={port}, last_pos={final_pos}"
            )

            _gripper_log.error(
                f"[ANGLED-PORTAFILTER-GRIP] FINAL FAIL after "
                f"{ANGLED_UNMOUNT_GRIP_RETRIES + 1} full routine attempts "
                f"port={port} tool={grab_tool_name} last_pos={final_pos}; "
                f"aborting angled_unmount before caching"
            )
            return _fail()

        _unmount_post_grab_joints_cache[grab_cache_key] = final_angles

        trace_grip(
            f"CACHED post-grab joints for key={grab_cache_key}, final_pos={final_pos}"
        )

        if not ok(_trace_run_skill("angled_unmount", "sync")):
            return _fail()

    if not ok(_trace_run_skill("angled_unmount", "sync")):
        return _fail()

    cached_port_angle = angled__port_angle_cache.get(port)

    if cached_port_angle:
        arc_cmd_by_2 = angled__portafilter_arc_cmd_by_port_2.get(str(port))
        if arc_cmd_by_2 is None:
            return _fail()

        for i in range(2):
            if not ok(_trace_run_skill("angled_unmount", "move_portafilter_arc_tool_angled", arc_cmd_by_2)):
                return _fail()
        if not ok(_trace_run_skill("angled_unmount", "sync")):
            return _fail()

    else:
        pose_before_arc = _trace_run_skill("angled_unmount", "current_pose")
        if (
            not ok(pose_before_arc)
            or not isinstance(pose_before_arc, (tuple, list))
            or len(pose_before_arc) < 6
        ):
            return _fail()

        current_rz = float(pose_before_arc[5])
        desired_rz = _ANGLED_PORTAFILTER_ARC_TARGET_RZ

        arc_delta = current_rz - desired_rz
        arc_cmd = -arc_delta
        arc_cmd_by_2 = arc_cmd / 2
        arc_delta_mount = arc_delta + _ANGLED_PORTAFILTER_MOUNT_ARC_EXTRA

        angled__portafilter_arc_cmd_by_port[str(port)] = float(arc_cmd)
        angled__portafilter_mount_arc_cmd_by_port[str(port)] = float(arc_delta_mount)
        angled__portafilter_arc_cmd_by_port_2[str(port)] = float(arc_cmd_by_2)

        _gripper_log.info(
            f"[ANGLED-PORTAFILTER-ARC] port={port} current_rz={current_rz:.3f}, "
            f"desired_rz={desired_rz:.3f}, arc_cmd={arc_cmd:.3f}, "
            f"mount_arc_cmd={arc_delta_mount:.3f}"
        )

        for i in range(2):
            if not ok(_trace_run_skill("angled_unmount", "move_portafilter_arc_tool_angled", arc_cmd_by_2)):
                return _fail()
        if not ok(_trace_run_skill("angled_unmount", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_unmount", "moveJ_deg", 0, 0, 0, 0, 0, 1)):
            return _fail()
        if not ok(_trace_run_skill("angled_unmount", "sync")):
            return _fail()

    cached = angled__port_angle_cache.get(port)

    if cached:
        mount_pose = cached['angled_mount']
        below_pose = cached['below']

        if not angled__is_valid_angles(below_pose):
            return _fail()

        if not ok(_trace_run_skill("angled_unmount", "set_speed_factor", 25)):
            return _fail()

        if not ok(_trace_run_skill("angled_unmount", "moveEE_movJ", 0.5, -4.0, -30, 0, 0, 0)):
            return _fail()

        if not ok(_trace_run_skill("angled_unmount", "set_speed_factor", 100)):
            return _fail()

    else:
        pose_after_arc = _trace_run_skill("angled_unmount", "current_pose")
        if not ok(pose_after_arc) or not isinstance(pose_after_arc, (tuple, list)) or len(pose_after_arc) < 3:
            return _fail()

        z_after_arc_mm = float(pose_after_arc[2])

        if not ok(_trace_run_skill("angled_unmount", "sync")):
            return _fail()

        mount_pose = _trace_run_skill("angled_unmount", "current_angles")

        pose_after_tension = _trace_run_skill("angled_unmount", "current_pose")
        if not ok(pose_after_tension) or not isinstance(pose_after_tension, (tuple, list)) or len(pose_after_tension) < 3:
            return _fail()

        z_after_tension_mm = float(pose_after_tension[2])
        dz_drop_mm = z_after_arc_mm - z_after_tension_mm + 1.0

        base_z = float(ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_up_angled"][2])

        if dz_drop_mm > 0.0:
            learned_clear_up_z = float(math.ceil(dz_drop_mm))
        else:
            learned_clear_up_z = base_z

        _portafilter_clear_up_z_mm_by_port[str(port)] = learned_clear_up_z

        _gripper_log.info(
            f"[ANGLED-CLEAR-UP-Z] port={port} z_arc={z_after_arc_mm:.2f} "
            f"z_after_tension={z_after_tension_mm:.2f} "
            f"drop={dz_drop_mm:.2f} mm -> portafilter_clear_up_angled z={learned_clear_up_z} mm"
        )

        if not angled__is_valid_angles(mount_pose):
            return _fail()

        if not ok(_trace_run_skill("angled_unmount", "set_speed_factor", 25)):
            return _fail()

        if not ok(_trace_run_skill("angled_unmount", "moveEE_movJ", 0.5, -4.0, -30, 0, 0, 0)):
            return _fail()

        if not ok(_trace_run_skill("angled_unmount", "set_speed_factor", 100)):
            return _fail()

        below_pose = _trace_run_skill("angled_unmount", "current_angles")
        if not angled__is_valid_angles(below_pose):
            return _fail()

        mount_pose = tuple(mount_pose)
        below_pose = tuple(below_pose)

        angled__port_angle_cache[port] = {
            'angled_mount': mount_pose,
            'below': below_pose,
        }

    angled__mount_runtime_cache[port] = {
        'angled_mount': tuple(mount_pose),
        'below': tuple(below_pose),
    }

    angled_mount_espresso_port = tuple(mount_pose)
    angled_below_espresso_port = tuple(below_pose)

    if not ok(_trace_run_skill("angled_unmount", "gotoJ_deg", *ESPRESSO_ANGLED_TRANSFER_POSES['machine_exit_1'])):
        return _fail()
    if not ok(_trace_run_skill("angled_unmount", "gotoJ_deg", *ESPRESSO_ANGLED_TRANSFER_POSES['machine_exit_2'])):
        return _fail()
    if not ok(_trace_run_skill("angled_unmount", "gotoJ_deg", *ESPRESSO_ANGLED_TRANSFER_POSES['grinder_entry'])):
        return _fail()
    if not ok(_trace_run_skill("angled_unmount", "gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])):
        return _fail()
    if not ok(_trace_run_skill("angled_unmount", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    return True

def angled_grinder(**params) -> bool:
    _trace_step("angled_grinder", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "angled_portafilter_2")
    positioning_time = params.get("positioning_time")
    if positioning_time is None:
        positioning_time = (shot_cfg.get("positioning_time") if shot_cfg else 1.2)
    portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "single_portafilter_angled")

    if not port or portafilter_tool not in ("double_portafilter_angled", "single_portafilter_angled"):
        return _fail()

    if port in ('port_1', 'angled_portafilter_1', 'angled_portafilter_2'):
        if not ok(_trace_run_skill("angled_grinder", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
            return _fail()

    if not _run_cached_machine_approach(
        f"angled_grinder:{port}:approach:grinder",
        "espresso_grinder",
        "angled_grinder",
    ):
        return _fail()

    grinder_cache_key = f"{port}_angled_grinder_post_mount"
    cached_grinder_mount_pose = angled__grinder_post_mount_cache.get(grinder_cache_key)

    if angled__is_valid_angles(cached_grinder_mount_pose):
        if not ok(_trace_run_skill("angled_grinder", "gotoJ_deg", *cached_grinder_mount_pose)):
            return _fail()
    else:
        if not _run_cached_machine_mount(
            f"angled_grinder:{port}:mount:grinder",
            "espresso_grinder",
            "angled_grinder",
        ):
            return _fail()

        grinder_mount_pose = _trace_run_skill("angled_grinder", "current_angles")
        if not angled__is_valid_angles(grinder_mount_pose):
            return _fail()
        angled__grinder_post_mount_cache[grinder_cache_key] = tuple(grinder_mount_pose)

    if not _run_cached_machine_approach(
        f"angled_grinder:{port}:approach:tamper",
        "espresso_grinder",
        "angled_tamper",
    ):
        return _fail()
    if not ok(_trace_run_skill("angled_grinder", "sync")):
        return _fail()
    time.sleep(positioning_time)

    if not _run_cached_machine_mount(
        f"angled_grinder:{port}:mount:grinder:final",
        "espresso_grinder",
        "angled_grinder",
    ):
        return _fail()

    if not _run_cached_machine_mount(
        f"angled_grinder:{port}:mount:tamper",
        "espresso_grinder",
        "angled_tamper",
    ):
        return _fail()

    if not ok(_trace_run_skill("angled_grinder", "sync")):
        return _fail()
    if not ok(_trace_run_skill("angled_grinder", "set_gripper_position", 255, 0, 255)):
        return _fail()

    cached_tool_pick_pose = angled__tool_pick_pose_cache.get(portafilter_tool)
    if angled__is_valid_angles(cached_tool_pick_pose):
        if not ok(_trace_run_skill("angled_grinder", "gotoJ_deg", *cached_tool_pick_pose)):
            return _fail()
    else:
        if not ok(_trace_run_skill("angled_grinder", "moveEE_movJ", -50, 50, 50, 15, 0, 0)):
            return _fail()
        if not ok(_trace_run_skill("angled_grinder", "approach_tool", portafilter_tool)):
            return _fail()
        if not ok(_trace_run_skill("angled_grinder", "sync")):
            return _fail()
        tool_pick_pose = _trace_run_skill("angled_grinder", "current_angles")
        if not angled__is_valid_angles(tool_pick_pose):
            return _fail()
        angled__tool_pick_pose_cache[portafilter_tool] = tuple(tool_pick_pose)

    if not ok(_trace_run_skill("angled_grinder", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    return True

def angled_single_grinder(**params) -> bool:
    _trace_step("angled_single_grinder", "START")
    params.setdefault("portafilter_tool", "single_portafilter_angled")
    params.setdefault("port", "angled_portafilter_2")
    return angled_grinder(**params)

def angled_double_grinder(**params) -> bool:
    _trace_step("angled_double_grinder", "START")
    params.setdefault("portafilter_tool", "double_portafilter_angled")
    params.setdefault("port", "angled_portafilter_1")
    return angled_grinder(**params)

def angled_tamper(**params) -> bool:
    _trace_step("angled_tamper", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    portafilter_tool = params.get("portafilter_tool") or (
        shot_cfg.get("portafilter_tool") if shot_cfg else "single_portafilter_angled"
    )

    if portafilter_tool not in ("double_portafilter_angled", "single_portafilter_angled"):
        return _fail()

    if not ok(_trace_run_skill("angled_tamper", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    cached_tool_pick_pose = angled__tool_pick_pose_cache.get(portafilter_tool)
    if angled__is_valid_angles(cached_tool_pick_pose):
        if not ok(_trace_run_skill("angled_tamper", "gotoJ_deg", *cached_tool_pick_pose)):
            return _fail()
        if not ok(_trace_run_skill("angled_tamper", "sync")):
            return _fail()
    else:
        if not ok(_trace_run_skill("angled_tamper", "move_to", portafilter_tool, 0.22)):
            return _fail()
        if not ok(_trace_run_skill("angled_tamper", "sync")):
            return _fail()

        if not ok(_trace_run_skill("angled_tamper", "approach_tool", portafilter_tool)):
            return _fail()

    # Local tolerance for angled tamper grip.
    # Your latest code was actually checking 139..143, so keep that explicit.
    ANGLED_TAMPER_GRIP_POS_MIN = 139
    ANGLED_TAMPER_GRIP_POS_MAX = 143
    ANGLED_TAMPER_GRIP_RETRIES = 5

    def _close_and_verify_grip_angled():
        _trace_step("angled_tamper._close_and_verify_grip_angled", "START")
        node = init_motion_node()
        success, pos = node.set_gripper_position(speed=255, position=255, force=255)
        if not success:
            return False, None

        ok_reading = (
            pos is not None
            and ANGLED_TAMPER_GRIP_POS_MIN <= pos <= ANGLED_TAMPER_GRIP_POS_MAX
        )
        return ok_reading, pos

    def _grab_then_close_angled():
        _trace_step("angled_tamper._grab_then_close_angled", "START")
        if not ok(_trace_run_skill("angled_tamper._grab_then_close_angled", "sync")):
            return False, None

        if not ok(_trace_run_skill("angled_tamper._grab_then_close_angled", "grab_tool", portafilter_tool)):
            return False, None

        if not ok(_trace_run_skill("angled_tamper._grab_then_close_angled", "sync")):
            return False, None
        return _close_and_verify_grip_angled()

    used_uncached_post_grab = False

    post_grab_pose = _angled_tamper_post_grab_joints_cache.get(portafilter_tool)

    if angled__is_valid_angles(post_grab_pose):
        if not ok(_trace_run_skill("angled_tamper", "gotoJ_deg", *post_grab_pose)):
            return _fail()

        if not ok(_trace_run_skill("angled_tamper", "sync")):
            return _fail()

        if not ok(_trace_run_skill("angled_tamper", "set_gripper_position", 255, 255, 255)):
            return _fail()

        if not ok(_trace_run_skill("angled_tamper", "sync")):
            return _fail()

    else:
        used_uncached_post_grab = True

        gripped, pos = _grab_then_close_angled()

        for attempt in range(1, ANGLED_TAMPER_GRIP_RETRIES + 1):
            if gripped:
                break

            _gripper_log.warning(
                f"[ANGLED-TAMPER-GRIP] attempt {attempt} pos={pos} "
                f"(want in [{ANGLED_TAMPER_GRIP_POS_MIN}, {ANGLED_TAMPER_GRIP_POS_MAX}]); "
                f"opening gripper and re-running tool approach"
            )

            if not ok(_trace_run_skill("angled_tamper", "set_gripper_position", 255, 0, 255)):
                return _fail()

            if not ok(_trace_run_skill("angled_tamper", "sync")):
                return _fail()

            retry_tool_pick_pose = angled__tool_pick_pose_cache.get(portafilter_tool)
            if angled__is_valid_angles(retry_tool_pick_pose):
                if not ok(_trace_run_skill("angled_tamper", "gotoJ_deg", *retry_tool_pick_pose)):
                    return _fail()
                if not ok(_trace_run_skill("angled_tamper", "sync")):
                    return _fail()
            else:
                if not ok(_trace_run_skill("angled_tamper", "move_to", portafilter_tool, 0.22)):
                    return _fail()
                if not ok(_trace_run_skill("angled_tamper", "sync")):
                    return _fail()

            if not ok(_trace_run_skill("angled_tamper", "approach_tool", portafilter_tool)):
                return _fail()

            if not ok(_trace_run_skill("angled_tamper", "sync")):
                return _fail()

            gripped, pos = _grab_then_close_angled()

        if not gripped:
            _gripper_log.error(
                f"[ANGLED-TAMPER-GRIP] FINAL FAIL tool={portafilter_tool} "
                f"pos_read={pos}; aborting angled_tamper"
            )
            return _fail()

        _gripper_log.info(f"[ANGLED-TAMPER-GRIP] gripped OK, pos={pos}")

        if not ok(_trace_run_skill("angled_tamper", "sync")):
            return _fail()

        post_grab_angles = _trace_run_skill("angled_tamper", "current_angles")
        if not angled__is_valid_angles(post_grab_angles):
            return _fail()

        _angled_tamper_post_grab_joints_cache[portafilter_tool] = tuple(post_grab_angles)

    if not ok(_trace_run_skill("angled_tamper", "moveEE", 0, 0, -2.5, 0, 0, 0)):
        return _fail()

    if not ok(_trace_run_skill("angled_tamper", "release_tension")):
        return _fail()

    # Only verify after release_tension on uncached/live grab path.
    if used_uncached_post_grab:
        gripped_after_tension, pos_after_tension = _close_and_verify_grip_angled()

        if not gripped_after_tension:
            _gripper_log.error(
                f"[ANGLED-TAMPER-GRIP] after release_tension pos={pos_after_tension} "
                f"outside [{ANGLED_TAMPER_GRIP_POS_MIN}, {ANGLED_TAMPER_GRIP_POS_MAX}]; "
                f"aborting angled_tamper"
            )
            return _fail()

        _gripper_log.info(
            f"[ANGLED-TAMPER-GRIP] after release_tension gripped OK, "
            f"pos={pos_after_tension}"
        )

        if not ok(_trace_run_skill("angled_tamper", "sync")):
            return _fail()

    if not ok(_trace_run_skill("angled_tamper", "moveEE", 0, 0, 40, 0, 0, 0)):
        return _fail()

    if not _run_cached_machine_mount(
        f"angled_tamper:{portafilter_tool}:mount:grinder",
        "espresso_grinder",
        "angled_grinder",
    ):
        return _fail()

    if not _run_cached_machine_approach(
        f"angled_tamper:{portafilter_tool}:approach:grinder",
        "espresso_grinder",
        "angled_grinder",
    ):
        return _fail()

    if not ok(_trace_run_skill("angled_tamper", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    return True

def angled_single_tamper(**params) -> bool:
    _trace_step("angled_single_tamper", "START")
    params.setdefault("portafilter_tool", "single_portafilter_angled")
    return angled_tamper(**params)

def angled_double_tamper(**params) -> bool:
    _trace_step("angled_double_tamper", "START")
    params.setdefault("portafilter_tool", "double_portafilter_angled")
    return angled_tamper(**params)

ANGLED_MOUNT_DEBUG = True
def angled_mount(**params) -> bool:
    global angled_below_espresso_port, angled_mount_espresso_port

    _trace_step("angled_mount", "START")
    def ok(r):
        return r not in (False, None)

    debug_enabled = globals().get("ANGLED_MOUNT_DEBUG", True)

    def trace(msg: str):
        if debug_enabled:
            print(f"[ANGLED-MOUNT] {msg}", flush=True)

    trace("START")

    espresso_dict = params.get("espresso")
    trace(f"espresso_dict={espresso_dict}")

    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    trace(f"shot_cfg={shot_cfg}")

    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "angled_portafilter_1")
    trace(f"resolved port={port}")

    if not port:
        trace("FAIL: port is empty")
        return _fail()

    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    trace(f"port_params exists={bool(port_params)}")

    if not port_params:
        trace(f"FAIL: missing PULL_ESPRESSO_PARAMS for port={port}")
        return _fail()

    trace("nav waypoint 1 START")
    if not ok(_trace_run_skill("angled_mount", "gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        trace("FAIL: nav waypoint 1")
        return _fail()
    trace("nav waypoint 1 DONE")

    trace("nav waypoint 2 START")
    if not ok(_trace_run_skill("angled_mount", "gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])):
        trace("FAIL: nav waypoint 2")
        return _fail()
    trace("nav waypoint 2 DONE")

    trace("nav waypoint 3 START")
    if not ok(_trace_run_skill("angled_mount", "gotoJ_deg", *ESPRESSO_ANGLED_TRANSFER_POSES['grinder_entry'])):
        trace("FAIL: nav waypoint 3")
        return _fail()
    trace("nav waypoint 3 DONE")

    trace("nav waypoint 4 START")
    if not ok(_trace_run_skill("angled_mount", "gotoJ_deg", *ESPRESSO_ANGLED_TRANSFER_POSES['machine_exit_2'])):
        trace("FAIL: nav waypoint 4")
        return _fail()
    trace("nav waypoint 4 DONE")

    trace("nav waypoint 5 START")
    if not ok(_trace_run_skill("angled_mount", "gotoJ_deg", *ESPRESSO_ANGLED_TRANSFER_POSES['machine_exit_1'])):
        trace("FAIL: nav waypoint 5")
        return _fail()
    trace("nav waypoint 5 DONE")

    trace("checking angled__mount_runtime_cache")
    runtime_cached = angled__mount_runtime_cache.get(port)

    if runtime_cached:
        trace("runtime cache FOUND")
        below_pose = runtime_cached.get("below")
        mount_pose = runtime_cached.get("angled_mount")
    else:
        trace("runtime cache MISSING; using global angled poses")
        below_pose = angled_below_espresso_port
        mount_pose = angled_mount_espresso_port

    trace(f"below_pose={below_pose}")
    trace(f"below_pose valid={angled__is_valid_angles(below_pose)}")

    if not angled__is_valid_angles(below_pose):
        trace("FAIL: invalid below_pose")
        return _fail()

    trace("goto below_pose START")
    if not ok(_trace_run_skill("angled_mount", "gotoJ_deg", *below_pose)):
        trace("FAIL: goto below_pose")
        return _fail()
    trace("goto below_pose DONE")

    trace(f"mount_pose={mount_pose}")
    trace(f"mount_pose valid={angled__is_valid_angles(mount_pose)}")

    if not angled__is_valid_angles(mount_pose):
        trace("FAIL: invalid mount_pose")
        return _fail()

    trace("sync before slow mount START")
    if not ok(_trace_run_skill("angled_mount", "sync")):
        return _fail()
    trace("sync before slow mount DONE")

    trace("set_speed_factor 25 START")
    if not ok(_trace_run_skill("angled_mount", "set_speed_factor", 25)):
        trace("FAIL: set_speed_factor 25")
        return _fail()
    trace("set_speed_factor 25 DONE")

    trace("goto mount_pose START")
    if not ok(_trace_run_skill("angled_mount", "gotoJ_deg", *mount_pose)):
        trace("FAIL: goto mount_pose")
        return _fail()
    trace("goto mount_pose DONE")

    trace("sync after mount_pose START")
    if not ok(_trace_run_skill("angled_mount", "sync")):
        return _fail()
    trace("sync after mount_pose DONE")

    trace("set_speed_factor 100 START")
    if not ok(_trace_run_skill("angled_mount", "set_speed_factor", 100)):
        trace("FAIL: set_speed_factor 100 before arc")
        return _fail()
    trace("set_speed_factor 100 DONE")

    trace("reading arc_delta_mount START")
    arc_delta_mount = angled__portafilter_mount_arc_cmd_by_port.get(str(port))
    trace(f"arc_delta_mount={arc_delta_mount}")

    if arc_delta_mount is None:
        trace(f"FAIL: missing arc_delta_mount for port={port}")
        return _fail()

    trace("move_portafilter_arc_tool_angled START")
    if not ok(_trace_run_skill("angled_mount", "move_portafilter_arc_tool_angled", arc_delta_mount)):
        trace("FAIL: move_portafilter_arc_tool_angled")
        return _fail()
    trace("move_portafilter_arc_tool_angled DONE")

    trace("sync after arc START")
    if not ok(_trace_run_skill("angled_mount", "sync")):
        return _fail()
    trace("sync after arc DONE")

    trace("release_tension START")
    if not ok(_trace_run_skill("angled_mount", "release_tension")):
        trace("FAIL: release_tension")
        return _fail()
    trace("release_tension DONE")

    trace("sync after release_tension START")
    if not ok(_trace_run_skill("angled_mount", "sync")):
        return _fail()
    trace("sync after release_tension DONE")

    trace("_open_gripper_with_verify START")
    if not _open_gripper_with_verify():
        trace("FAIL: _open_gripper_with_verify")
        return _fail()
    trace("_open_gripper_with_verify DONE")

    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return _fail()

    trace("sync after gripper open START")
    if not ok(_trace_run_skill("angled_mount", "sync")):
        return _fail()
    trace("sync after gripper open DONE")

    trace(f"checking final machine approach condition for port={port}")
    if port in ("angled_portafilter_2",):
        trace("final machine approach START")
        if not _run_cached_machine_approach(
            f"angled_unmount:{port}:approach:{port_params['portafilter_number']}",
            "three_group_espresso",
            port_params["portafilter_number"],
        ):
            trace("FAIL: final machine approach")
            return _fail()
        trace("final machine approach DONE")
    else:
        trace("final machine approach SKIPPED")

    trace("goto port home START")
    if not ok(_trace_run_skill("angled_mount", "gotoJ_deg", *port_params["home"])):
        trace("FAIL: goto port home")
        return _fail()
    trace("goto port home DONE")

    trace("SUCCESS")
    return True

def angled_grab_espresso_pitcher(**params) -> bool:
    """
    Grab the espresso pitcher and stop right after closing the gripper.
    """
    _trace_step("angled_grab_espresso_pitcher", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3', 'angled_portafilter_1', 'angled_portafilter_2'):
        return _fail()
    if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()
    cached = angled__pitcher_pick_cache.get(port)
    pick2 = cached.get("pick2_approach") if cached else None
    if angled__is_valid_angles(pick2):
        if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "gotoJ_deg", *pick2)):
            return _fail()
        if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
            return _fail()
    else:
        if not _run_cached_machine_approach(
            f"angled_grab_pitcher:{port}:approach:pick_pitcher_2",
            "three_group_espresso",
            "pick_pitcher_2",
        ):
            return _fail()
        if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
            return _fail()
        pick2_angles = _trace_run_skill("angled_grab_espresso_pitcher", "current_angles")
        if angled__is_valid_angles(pick2_angles):
            angled__pitcher_pick_cache.setdefault(port, {})["pick2_approach"] = tuple(pick2_angles)

    cached = angled__pitcher_pick_cache.get(port)

    if port in ('port_1', 'angled_portafilter_1'):
        if cached and cached.get('approach') and cached.get('mount'):
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "gotoJ_deg", *cached['approach'])):
                return _fail()
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "gotoJ_deg", *cached['mount'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_grab_pitcher:{port}:approach:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
                return _fail()
            approach_angles = _trace_run_skill("angled_grab_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(approach_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

            if not _run_cached_machine_mount(
                f"angled_grab_pitcher:{port}:mount:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
                return _fail()
            mount_angles = _trace_run_skill("angled_grab_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['mount'] = tuple(mount_angles)
            
        if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "set_gripper_position", 255, 115, 255)):
            return _fail()

    elif port in ('port_2', 'angled_portafilter_2'):
        pick_cache = angled__pitcher_pick_cache.setdefault(port, {})

        if angled__is_valid_angles(pick_cache.get('mount')):
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "gotoJ_deg", *pick_cache['mount'])):
                return _fail()

            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
                return _fail()

            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "set_gripper_position", 255, 115, 255)):
                return _fail()

        else:
            if not _run_cached_machine_mount(
                f"angled_grab_pitcher:{port}:mount:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return _fail()

            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
                return _fail()

            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "set_gripper_position", 255, 115, 255)):
                return _fail()

            # Runs only the first time this angled grab mount is learned.
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "release_tension")):
                return _fail()

            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
                return _fail()

            mount_angles = _trace_run_skill("angled_grab_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(mount_angles):
                pick_cache['mount'] = tuple(mount_angles)

    elif port == 'port_3':
        if cached and cached.get('approach') and cached.get('mount'):
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "gotoJ_deg", *cached['approach'])):
                return _fail()
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "gotoJ_deg", *cached['mount'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_grab_pitcher:{port}:approach:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
                return _fail()
            approach_angles = _trace_run_skill("angled_grab_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(approach_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

            if not _run_cached_machine_mount(
                f"angled_grab_pitcher:{port}:mount:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
                return _fail()
            mount_angles = _trace_run_skill("angled_grab_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['mount'] = tuple(mount_angles)
        
        if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_grab_espresso_pitcher", "set_gripper_position", 255, 115, 255)):
            return _fail()

    else:
        return _fail()

    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return _fail()
        
    return True

def angled_pick_espresso_pitcher(**params) -> bool:
    """
    Complete pitcher pickup after angled_grab_espresso_pitcher().
    """
    _trace_step("angled_pick_espresso_pitcher", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3', 'angled_portafilter_1', 'angled_portafilter_2'):
        return _fail()

    cached = angled__pitcher_pick_cache.get(port)

    if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "sync")):
        return _fail()
    if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])):
        return _fail()

    if port in ('port_1', 'angled_portafilter_1'):
        if cached and cached.get('retreat'):
            if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "gotoJ_deg", *cached['retreat'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_pick_pitcher:{port}:retreat:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "sync")):
                return _fail()
            retreat_angles = _trace_run_skill("angled_pick_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port in ('port_2', 'angled_portafilter_2'):
        if cached and cached.get('retreat'):
            if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "gotoJ_deg", *cached['retreat'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_pick_pitcher:{port}:retreat:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "sync")):
                return _fail()
            retreat_angles = _trace_run_skill("angled_pick_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_3':
        if cached and cached.get('retreat'):
            if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "gotoJ_deg", *cached['retreat'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_pick_pitcher:{port}:retreat:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "sync")):
                return _fail()
            retreat_angles = _trace_run_skill("angled_pick_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    if port in ('port_1', 'angled_portafilter_1', 'port_2', 'angled_portafilter_2'):
        if not ok(_trace_run_skill("angled_pick_espresso_pitcher", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
            return _fail()

    return True

def angled_pour_espresso_pitcher_cup_station(**params) -> bool:
    _trace_step("angled_pour_espresso_pitcher_cup_station", "START")
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"

    if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_FLOW_POSES['cup_station_entry'])):
        return _fail()

    if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return _fail()

    if stage == 'stage_1':
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos1'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "set_speed_factor", SPEED_SLOW_POURING)):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.1'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.2'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.3'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "set_speed_factor", 100)):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral1'])):
            return _fail()
    elif stage == 'stage_2':
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos2'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "set_speed_factor", SPEED_SLOW_POURING)):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.1'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.2'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.3'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "set_speed_factor", 100)):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral2'])):
            return _fail()
    elif stage == 'stage_3':
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos3'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "set_speed_factor", SPEED_SLOW_POURING)):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.1'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.2'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.3'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "set_speed_factor", 100)):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral3'])):
            return _fail()
    else:
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos4'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "set_speed_factor", SPEED_SLOW_POURING)):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.1'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.2'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.3'])):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "set_speed_factor", 100)):
            return _fail()
        if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral4'])):
            return _fail()

    if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return _fail()

    if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_FLOW_POSES['cup_station_entry'])):
        return _fail()

    if not ok(_trace_run_skill("angled_pour_espresso_pitcher_cup_station", "gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
        return _fail()

    return True

def angled_get_hot_water(**params) -> bool:
    _trace_step("angled_get_hot_water", "START")
    def ok(r):
        return r not in (False, None)

    if not _run_cached_machine_approach(
        "angled_get_hot_water:approach:hot_water",
        "three_group_espresso",
        "hot_water",
    ):
        return _fail()

    if not _run_cached_machine_mount(
        "angled_get_hot_water:mount:hot_water",
        "three_group_espresso",
        "hot_water",
    ):
        return _fail()

    if not ok(_trace_run_skill("angled_get_hot_water", "moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_move'])):
        return _fail()
    return True

def angled_with_hot_water(**params) -> bool:
    _trace_step("angled_with_hot_water", "START")
    def ok(r):
        return r not in (False, None)

    if not ok(_trace_run_skill("angled_with_hot_water", "set_speed_factor", ESPRESSO_SPEEDS['hot_water_pour'])):
        return _fail()

    if not ok(_trace_run_skill("angled_with_hot_water", "moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return _fail()

    return True

def angled_return_espresso_pitcher(**params) -> bool:
    global angled_approach_pitcher, angled_pick_pitcher

    _trace_step("angled_return_espresso_pitcher", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3', 'angled_portafilter_1', 'angled_portafilter_2'):
        return _fail()

    cached = angled__pitcher_return_cache.get(port)

    if port in ('port_1', 'angled_portafilter_1'):
        if cached and cached.get('approach'):
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *cached['approach'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_return_pitcher:{port}:approach:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
                return _fail()
            approach_angles = _trace_run_skill("angled_return_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(approach_angles):
                angled__pitcher_return_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

        if cached and cached.get('mount'):
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *cached['mount'])):
                return _fail()
        else:
            if not _run_cached_machine_mount(
                f"angled_return_pitcher:{port}:mount:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
                return _fail()
            mount_angles = _trace_run_skill("angled_return_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_return_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_return_espresso_pitcher", "set_gripper_position", 35,0,255)):
            return _fail()
        if cached and cached.get('retreat'):
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *cached['retreat'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_return_pitcher:{port}:retreat:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
                return _fail()
            retreat_angles = _trace_run_skill("angled_return_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_return_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port in ('port_2', 'angled_portafilter_2'):
        return_cache = angled__pitcher_return_cache.setdefault(port, {})

        pick_cached = angled__pitcher_pick_cache.get(port)
        pick_retreat = pick_cached.get('retreat') if pick_cached else None

        if not angled__is_valid_angles(pick_retreat):
            return _fail()

        if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *pick_retreat)):
            return _fail()

        if angled__is_valid_angles(return_cache.get('mount')):
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *return_cache['mount'])):
                return _fail()

        else:
            if not _run_cached_machine_mount(
                f"angled_return_pitcher:{port}:mount:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return _fail()

            # Runs only the first time this angled return mount is learned.
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
                return _fail()

            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "release_tension")):
                return _fail()

            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
                return _fail()

            mount_angles = _trace_run_skill("angled_return_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(mount_angles):
                return_cache['mount'] = tuple(mount_angles)

        if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
            return _fail()

        if not ok(_trace_run_skill("angled_return_espresso_pitcher", "set_gripper_position", 35, 0, 255)):
            return _fail()

    elif port == 'port_3':
        if cached and cached.get('approach'):
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *cached['approach'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_return_pitcher:{port}:approach:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
                return _fail()
            approach_angles = _trace_run_skill("angled_return_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(approach_angles):
                angled__pitcher_return_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

        if cached and cached.get('mount'):
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *cached['mount'])):
                return _fail()
        else:
            if not _run_cached_machine_mount(
                f"angled_return_pitcher:{port}:mount:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
                return _fail()
            mount_angles = _trace_run_skill("angled_return_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_return_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_return_espresso_pitcher", "set_gripper_position", 35,0,255)):
            return _fail()
        if cached and cached.get('retreat'):
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *cached['retreat'])):
                return _fail()
        else:
            if not _run_cached_machine_approach(
                f"angled_return_pitcher:{port}:retreat:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return _fail()
            if not ok(_trace_run_skill("angled_return_espresso_pitcher", "sync")):
                return _fail()
            retreat_angles = _trace_run_skill("angled_return_espresso_pitcher", "current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_return_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    if not _run_cached_machine_approach(
        f"angled_return_pitcher:{port}:final_approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return _fail()

    if not ok(_trace_run_skill("angled_return_espresso_pitcher", "gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()

    return True

def angled_return_cleaned_espresso_pitcher(**params) -> bool:
    _trace_step("angled_return_cleaned_espresso_pitcher", "START")
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3', 'angled_portafilter_1', 'angled_portafilter_2'):
        return _fail()

    if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()

    if not _run_cached_machine_approach(
        f"angled_return_clean_pitcher:{port}:approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return _fail()

    cached = angled__pitcher_clean_cache.get(port)

    if port in ('port_1', 'angled_portafilter_1'):
        if not _run_cached_machine_approach(
            f"angled_return_clean_pitcher:{port}:approach:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return _fail()
        if not _run_cached_machine_mount(
            f"angled_return_clean_pitcher:{port}:mount:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "set_gripper_position", 255,115,255)):
            return _fail()
        if cached:
            for angles in cached:
                if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "gotoJ_deg", *angles)):
                    return _fail()
        else:
            waypoints = []
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, 20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, -170)):
                return _fail()
            _trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, 170)):
                return _fail()
            _trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, -20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if all(angled__is_valid_angles(w) for w in waypoints):
                angled__pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "set_gripper_position", 35,0,255)):
            return _fail()
        if not _run_cached_machine_approach(
            f"angled_return_clean_pitcher:{port}:final_approach:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return _fail()

    elif port in ('port_2', 'angled_portafilter_2'):
        if not _run_cached_machine_mount(
            f"angled_return_clean_pitcher:{port}:mount:pick_pitcher_2",
            "three_group_espresso",
            "pick_pitcher_2",
        ):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "set_gripper_position", 255,115,255)):
            return _fail()
        if cached:
            for angles in cached:
                if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "gotoJ_deg", *angles)):
                    return _fail()
        else:
            waypoints = []
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, 20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, -170)):
                return _fail()
            _trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, 170)):
                return _fail()
            _trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, -20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if all(angled__is_valid_angles(w) for w in waypoints):
                angled__pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "set_gripper_position", 35,0,255)):
            return _fail()

    elif port == 'port_3':
        if not _run_cached_machine_approach(
            f"angled_return_clean_pitcher:{port}:approach:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return _fail()
        if not _run_cached_machine_mount(
            f"angled_return_clean_pitcher:{port}:mount:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "set_gripper_position", 255,115,255)):
            return _fail()
        if cached:
            for angles in cached:
                if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "gotoJ_deg", *angles)):
                    return _fail()
        else:
            waypoints = []
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, 20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, -135)):
                return _fail()
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")):
                return _fail()
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveJ_deg", 0, 0, 0, 0, 0, 135)):
                return _fail()
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")):
                return _fail()
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "moveEE_movJ", 0, 0, -20, 0, 0, 0)):
                return _fail()
            waypoints.append(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "current_angles"))
            if all(angled__is_valid_angles(w) for w in waypoints):
                angled__pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "sync")):
            return _fail()
        if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "set_gripper_position", 35,0,255)):
            return _fail()
        if not _run_cached_machine_approach(
            f"angled_return_clean_pitcher:{port}:final_approach:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return _fail()

    if not _run_cached_machine_approach(
        f"angled_return_clean_pitcher:{port}:final_approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return _fail()

    if not ok(_trace_run_skill("angled_return_cleaned_espresso_pitcher", "gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()

    return True

def angled_unmount_single(**params) -> bool:
    _trace_step("angled_unmount_single", "START")
    params["port"] = "port_3"
    return angled_unmount(**params)

def angled_unmount_double(**params) -> bool:
    _trace_step("angled_unmount_double", "START")
    params["port"] = "angled_portafilter_1"
    return angled_unmount(**params)

def angled_mount_single(**params) -> bool:
    _trace_step("angled_mount_single", "START")
    params["port"] = "port_3"
    return angled_mount(**params)

def angled_mount_double(**params) -> bool:
    _trace_step("angled_mount_double", "START")
    params["port"] = "angled_portafilter_1"
    return angled_mount(**params)

def angled_single_grab_espresso_pitcher(**params) -> bool:
    _trace_step("angled_single_grab_espresso_pitcher", "START")
    params["port"] = "port_2"
    return angled_grab_espresso_pitcher(**params)

def angled_double_grab_espresso_pitcher(**params) -> bool:
    _trace_step("angled_double_grab_espresso_pitcher", "START")
    params["port"] = "port_1"
    return angled_grab_espresso_pitcher(**params)

def angled_single_pick_espresso_pitcher(**params) -> bool:
    _trace_step("angled_single_pick_espresso_pitcher", "START")
    params["port"] = "port_2"
    return angled_pick_espresso_pitcher(**params)

def angled_double_pick_espresso_pitcher(**params) -> bool:
    _trace_step("angled_double_pick_espresso_pitcher", "START")
    params["port"] = "port_1"
    return angled_pick_espresso_pitcher(**params)

def angled_single_pour_espresso_pitcher_cup_station(**params) -> bool:
    _trace_step("angled_single_pour_espresso_pitcher_cup_station", "START")
    params["port"] = "port_3"
    return angled_pour_espresso_pitcher_cup_station(**params)

def angled_double_pour_espresso_pitcher_cup_station(**params) -> bool:
    _trace_step("angled_double_pour_espresso_pitcher_cup_station", "START")
    params["port"] = "port_1"
    return angled_pour_espresso_pitcher_cup_station(**params)

def angled_single_return_espresso_pitcher(**params) -> bool:
    _trace_step("angled_single_return_espresso_pitcher", "START")
    params["port"] = "port_3"
    return angled_return_espresso_pitcher(**params)

def angled_double_return_espresso_pitcher(**params) -> bool:
    _trace_step("angled_double_return_espresso_pitcher", "START")
    params["port"] = "port_1"
    return angled_return_espresso_pitcher(**params)

def angled_single_return_cleaned_espresso_pitcher(**params) -> bool:
    _trace_step("angled_single_return_cleaned_espresso_pitcher", "START")
    params["port"] = "port_3"
    return angled_return_cleaned_espresso_pitcher(**params)

def angled_double_return_cleaned_espresso_pitcher(**params) -> bool:
    _trace_step("angled_double_return_cleaned_espresso_pitcher", "START")
    params["port"] = "angled_portafilter_1"
    return angled_return_cleaned_espresso_pitcher(**params)

SEQUENCES = {
    'unmount': unmount,
    'grinder': grinder,
    'mount': mount,
    'grab_espresso_pitcher': grab_espresso_pitcher,
    'pick_espresso_pitcher': pick_espresso_pitcher,
    'pour_espresso_pitcher_cup_station': pour_espresso_pitcher_cup_station,
    'get_hot_water': get_hot_water,
    'with_hot_water': with_hot_water,
    'return_espresso_pitcher': return_espresso_pitcher,
    'return_cleaned_espresso_pitcher': return_cleaned_espresso_pitcher,
    'tamper': tamper,
    'single_tamper': single_tamper,
    'double_tamper': double_tamper,
    'unmount_single': unmount_single,
    'unmount_double': unmount_double,
    'mount_single': mount_single,
    'mount_double': mount_double,
    'single_pick_espresso_pitcher': single_pick_espresso_pitcher,
    'double_pick_espresso_pitcher': double_pick_espresso_pitcher,
    'single_pour_espresso_pitcher_cup_station': single_pour_espresso_pitcher_cup_station,
    'double_pour_espresso_pitcher_cup_station': double_pour_espresso_pitcher_cup_station,
    'single_return_espresso_pitcher': single_return_espresso_pitcher,
    'double_return_espresso_pitcher': double_return_espresso_pitcher,
    'single_return_cleaned_espresso_pitcher': single_return_cleaned_espresso_pitcher,
    'double_return_cleaned_espresso_pitcher': double_return_cleaned_espresso_pitcher,
    'single_grab_espresso_pitcher': single_grab_espresso_pitcher,
    'double_grab_espresso_pitcher': double_grab_espresso_pitcher,
    'single_grinder': single_grinder,
    'double_grinder': double_grinder,

    'angled_single_grinder': angled_single_grinder,
    'angled_double_grinder': angled_double_grinder,
    'angled_single_tamper': angled_single_tamper,
    'angled_double_tamper': angled_double_tamper,
    'angled_mount_single': angled_mount_single,
    'angled_mount_double': angled_mount_double,
    'angled_unmount_single': angled_unmount_single,
    'angled_unmount_double': angled_unmount_double,
    'angled_single_grab_espresso_pitcher': angled_single_grab_espresso_pitcher,
    'angled_double_grab_espresso_pitcher': angled_double_grab_espresso_pitcher,
    'angled_single_pick_espresso_pitcher': angled_single_pick_espresso_pitcher,
    'angled_double_pick_espresso_pitcher': angled_double_pick_espresso_pitcher,
    'angled_single_pour_espresso_pitcher_cup_station': angled_single_pour_espresso_pitcher_cup_station,
    'angled_double_pour_espresso_pitcher_cup_station': angled_double_pour_espresso_pitcher_cup_station,
    'angled_single_return_espresso_pitcher': angled_single_return_espresso_pitcher,
    'angled_double_return_espresso_pitcher': angled_double_return_espresso_pitcher,
    'angled_single_return_cleaned_espresso_pitcher': angled_single_return_cleaned_espresso_pitcher,
    'angled_double_return_cleaned_espresso_pitcher': angled_double_return_cleaned_espresso_pitcher,
}