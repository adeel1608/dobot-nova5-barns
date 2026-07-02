"""
home.py

Defines the 'home' positioning routine using compass directions.
This module provides functions for robot positioning, machine calibration,
and system diagnostics for the BARNS coffee automation system.
"""
#NOTE: Gripper commands can opt into strict register verification via run_skill(..., verify_position=True).


import logging
import subprocess
import sys
import inspect
import time
from typing import Dict, Any, Union

_log = logging.getLogger(__name__)
from oms_v1.params import (
    HOME_ANGLES, ESPRESSO_HOME, ESPRESSO_GRINDER_HOME,
    HOME_CALIBRATION_PARAMS, HOME_CALIBRATION_CONSTANTS,
    GRIPPER_FULL, GRIPPER_OPEN, SPEED_FAST
)
from oms_v1.manipulate_node import run_skill as _raw_run_skill

MAX_CALIBRATION_RETRIES = 5

# Runtime trace helpers. Home/calibration calls are high-level route anchors, so
# START/DONE logs make it easier to locate failures in long drink sequences.
HOME_TRACE_DEBUG = True

def _trace_format_value(value, max_len: int = 140) -> str:
    try:
        text = repr(value)
    except Exception:
        text = f"<{type(value).__name__}>"
    return text if len(text) <= max_len else text[:max_len - 3] + "..."

def _trace_step(scope: str, message: str) -> None:
    if globals().get("HOME_TRACE_DEBUG", True):
        print(f"[HOME:{scope}] {message}", flush=True)

def run_skill(*args, **kwargs):
    skill_name = args[0] if args else "<missing>"
    skill_args = args[1:] if len(args) > 1 else ()
    _trace_step("run_skill", f"{skill_name} START args={_trace_format_value(skill_args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_run_skill(*args, **kwargs)
    _trace_step("run_skill", f"{skill_name} DONE result={_trace_format_value(result)}")
    return result

def _fail(reason: str = "") -> bool:
    if globals().get("HOME_TRACE_DEBUG", True):
        try:
            frame = sys._getframe(1)
            msg = f"FAIL in {frame.f_code.co_name} line={frame.f_lineno}"
            if reason:
                msg += f" reason={reason}"
            _trace_step("fail", msg)
        except Exception:
            pass
    return False


def _calibrate_marker(marker_name, prep_fn, ok):
    """Retry a single marker calibration up to MAX_CALIBRATION_RETRIES times.

    prep_fn must move the arm into the correct approach pose and call sync.
    Returns True on the first successful get_machine_position, False if all
    attempts are exhausted.
    """
    _trace_step("_calibrate_marker", "START")
    for attempt in range(1, MAX_CALIBRATION_RETRIES + 1):
        if not prep_fn():
            _log.warning(f"[CALIBRATION] {marker_name} prep failed (attempt {attempt}/{MAX_CALIBRATION_RETRIES})")
            time.sleep(1.0)
            continue
        result = run_skill("get_machine_position", marker_name)
        if ok(result):
            return True
        _log.warning(f"[CALIBRATION] {marker_name} failed (attempt {attempt}/{MAX_CALIBRATION_RETRIES})")
        time.sleep(1.0)
    _log.error(f"[CALIBRATION] {marker_name} failed after {MAX_CALIBRATION_RETRIES} attempts")
    return False


def home(**params) -> bool:
    """
    Move robot to a predefined home position.
    """
    def ok(r):
        return r not in (False, None)

    position = params.get("position", "north")
    if not position:
        return _fail("missing home position")

    angles = HOME_ANGLES.get(str(position))
    if not angles:
        return _fail(f"unknown home position={position}")

    if not ok(run_skill("gotoJ_deg", *angles)):
        return False

    return True


def return_back_to_home() -> bool:
    """
    Return the robot to a safe home position based on current angle.
    """
    def ok(r):
        return r not in (False, None)

    release_result = run_skill("release_tension")
    if not ok(release_result):
        if not ok(run_skill("toggle_drag_mode")):
            return False

    run_skill("set_speed_factor", 100)
    run_skill("sync")
    run_skill("set_gripper_position", 255,0,255)
    angles = run_skill("current_angles")

    if not ok(angles) or len(angles) < 6:
        return False

    a1 = float(angles[0])
    j1_val = None

    if -22.49 <= a1 <= 22.49:
        j1_val = 0.0
    elif 22.51 <= a1 <= 67.49:
        j1_val = 45.0
    elif 67.51 <= a1 <= 112.49:
        j1_val = 90.0
    elif 112.51 <= a1 <= 157.49:
        j1_val = 135.0
    elif 157.51 <= a1 <= 202.49:
        j1_val = 180.0
    elif 202.51 <= a1 <= 247.49:
        j1_val = -135.0
    elif 247.51 <= a1 <= 292.49:
        j1_val = -90.0
    elif 292.51 <= a1 <= 337.49:
        j1_val = -45.0
    elif 337.51 <= a1 <= 360.0:
        j1_val = 0.0
    elif -67.49 <= a1 <= -22.51:
        j1_val = -45.0
    elif -112.49 <= a1 <= -67.51:
        j1_val = -90.0
    elif -157.49 <= a1 <= -112.51:
        j1_val = -135.0
    elif -202.49 <= a1 <= -157.51:
        j1_val = -180.0
    elif -247.49 <= a1 <= -202.51:
        j1_val = 135.0
    elif -292.49 <= a1 <= -247.51:
        j1_val = 90.0
    elif -337.49 <= a1 <= -292.51:
        j1_val = 45.0
    elif -360.0 <= a1 <= -337.51:
        j1_val = 0.0

    if j1_val is None:
        return _fail(f"current joint1 outside return-home compass ranges: {a1}")

    home_j2_j6 = HOME_CALIBRATION_PARAMS['return_home_position']
    if not ok(run_skill("gotoJ_deg", j1_val, *home_j2_j6)):
        return False

    return True


def get_machine_position(**params) -> bool:
    """
    Calibrate and record machine positions for all coffee equipment.
    """
    from oms_v1.sequences.espresso import invalidate_port_cache, angled_invalidate_port_cache
    from oms_v1.sequences.cleaning import (
        invalidate_cleaning_cache,
        angled_invalidate_cleaning_cache,
    )
    from oms_v1.sequences.milk_frothing import invalidate_milk_frothing_cache

    def ok(r):
        return r not in (False, None)

    invalidate_port_cache()
    invalidate_cleaning_cache()
    angled_invalidate_cleaning_cache()
    angled_invalidate_port_cache()
    run_skill("set_speed_factor", SPEED_FAST)

    if not return_back_to_home():
        return False

    def _prep_cleaner():
        if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['portafilter_cleaner']['prep_position'])):
            return False
        cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
        for _ in range(cycles):
            time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
            if not ok(run_skill("move_to", "portafilter_cleaner", 0.22)):
                return False
        run_skill("sync")
        return True

    if not _calibrate_marker("portafilter_cleaner", _prep_cleaner, ok):
        return False

    def _prep_grinder():
        if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep2'])):
            return False
        cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
        for _ in range(cycles):
            time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
            if not ok(run_skill("move_to", "espresso_grinder", 0.22)):
                return False
        run_skill("sync")
        return True

    if not _calibrate_marker("espresso_grinder", _prep_grinder, ok):
        return False

    def _prep_espresso():
        if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['three_group_espresso_calibration']['prep1'])):
            return False
        if not ok(run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)):
            return False
        cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
        for _ in range(cycles):
            time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
            if not ok(run_skill("move_to", "three_group_espresso", 0.22)):
                return False
        run_skill("sync")
        return True

    if not _calibrate_marker("three_group_espresso", _prep_espresso, ok):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False

    check_saved_data()

    return True


def check_saved_data() -> Dict[str, Any]:
    """
    Check and display currently saved machine position data.
    """
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory

    try:
        pkg_share = get_package_share_directory("pickn_place")
        mem_path = os.path.join(pkg_share, "machine_pose_data_memory.yaml")

        if not os.path.exists(mem_path):
            return {}

        with open(mem_path, "r") as f:
            data = yaml.safe_load(f) or {}

        machines = data.get("machines", {})
        return machines

    except Exception:
        return {}


def check_aruco_status(**params) -> bool:
    """
    Check current ArUco marker detection status and help diagnose calibration issues.
    """
    check_saved_data()
    return True


def solution(j1, j2, j3, j4, j5, j6, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
    """
    Convert joint values to cartesian, apply offsets, and convert back to joints.
    """
    print(f"Input joints: [{j1}, {j2}, {j3}, {j4}, {j5}, {j6}]")

    pos_result = run_skill("positive_solution", j1, j2, j3, j4, j5, j6)

    if not pos_result or not hasattr(pos_result, 'pose'):
        print("Failed to get positive solution")
        return None

    try:
        pose_values = [float(v) for v in pos_result.pose.strip("{}").split(",")[:6]]
        current_x, current_y, current_z, current_rx, current_ry, current_rz = pose_values
    except (ValueError, IndexError) as e:
        print(f"Failed to parse pose string: {e}")
        return None

    print(f"Current cartesian: x={current_x:.3f}, y={current_y:.3f}, z={current_z:.3f}, rx={current_rx:.3f}, ry={current_ry:.3f}, rz={current_rz:.3f}")

    new_x = current_x + x
    new_y = current_y + y
    new_z = current_z + z
    new_rx = current_rx + rx
    new_ry = current_ry + ry
    new_rz = current_rz + rz

    print(f"Offsets applied: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    print(f"New cartesian: x={new_x:.3f}, y={new_y:.3f}, z={new_z:.3f}, rx={new_rx:.3f}, ry={new_ry:.3f}, rz={new_rz:.3f}")

    inv_result = run_skill("inverse_solution", new_x, new_y, new_z, new_rx, new_ry, new_rz)

    if inv_result and hasattr(inv_result, 'angle'):
        try:
            angle_values = [float(v) for v in inv_result.angle.strip("{}").split(",")[:6]]
            res_j1, res_j2, res_j3, res_j4, res_j5, res_j6 = angle_values
            print(f"Resulting joints: [{res_j1:.3f}, {res_j2:.3f}, {res_j3:.3f}, {res_j4:.3f}, {res_j5:.3f}, {res_j6:.3f}]")
        except (ValueError, IndexError) as e:
            print(f"Failed to parse angle string: {e}")
            return None
    else:
        print("Failed to get inverse solution")
        return None

    return inv_result


def solution_interactive(*params):
    """
    Interactive wrapper for solution function that prompts for input.
    """
    print("\n=== Joint to Cartesian Offset Solution ===")
    print("Enter joint values (j1-j6) and optional cartesian offsets (x,y,z,rx,ry,rz)")
    print("Press Enter to use default value of 0.0 for any parameter\n")

    try:
        j1 = float(input("j1 (degrees): ") or 0.0)
        j2 = float(input("j2 (degrees): ") or 0.0)
        j3 = float(input("j3 (degrees): ") or 0.0)
        j4 = float(input("j4 (degrees): ") or 0.0)
        j5 = float(input("j5 (degrees): ") or 0.0)
        j6 = float(input("j6 (degrees): ") or 0.0)

        print("\nCartesian offsets (optional - press Enter for 0.0):")
        x = float(input("x offset (mm): ") or 0.0)
        y = float(input("y offset (mm): ") or 0.0)
        z = float(input("z offset (mm): ") or 0.0)
        rx = float(input("rx offset (degrees): ") or 0.0)
        ry = float(input("ry offset (degrees): ") or 0.0)
        rz = float(input("rz offset (degrees): ") or 0.0)

        print("\n" + "=" * 50)
        return solution(j1, j2, j3, j4, j5, j6, x, y, z, rx, ry, rz)

    except ValueError as e:
        print(f"Invalid input: {e}")
        return None
    except KeyboardInterrupt:
        print("\nCancelled")
        return None


def open_gripper(**params):
    run_skill("sync")
    run_skill("set_gripper_position", 255, 0, 255)
    return True


def close_gripper(**params):
    run_skill("sync")
    run_skill("set_gripper_position", 255, 255, 255)
    return True


def toggle_drag_mode(**params):
    run_skill("toggle_drag_mode")
    return True


def _call_dobot_driver_service(
    service_name: str,
    srv_type: str,
    args_yaml: str = "{}",
    timeout_sec: float = 15.0,
) -> bool:
    """Call a raw Dobot ROS2 driver service from a sequence action."""
    cmd = [
        "ros2",
        "service",
        "call",
        f"/dobot_bringup_v3/srv/{service_name}",
        srv_type,
        args_yaml,
    ]
    _trace_step(
        "dobot_service",
        f"{service_name} START args={_trace_format_value(args_yaml)}",
    )
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired:
        _trace_step("dobot_service", f"{service_name} FAILED timeout")
        return False
    except FileNotFoundError as exc:
        _trace_step("dobot_service", f"{service_name} FAILED ros2 not found: {exc}")
        return False
    except Exception as exc:
        _trace_step("dobot_service", f"{service_name} FAILED {type(exc).__name__}: {exc}")
        return False

    if result.returncode != 0:
        _trace_step(
            "dobot_service",
            f"{service_name} FAILED exit={result.returncode} "
            f"stderr={(result.stderr or '').strip()[:180]}",
        )
        return False

    _trace_step("dobot_service", f"{service_name} DONE")
    return True


def enable_robot(**params):
    """Enable the Dobot arm through the driver EnableRobot service."""
    load = float(params.get("load", 2.0))
    return _call_dobot_driver_service(
        "EnableRobot",
        "dobot_msgs_v3/srv/EnableRobot",
        f"{{load: {load}}}",
    )


def disable_robot(**params):
    """Disable the Dobot arm through the driver DisableRobot service."""
    return _call_dobot_driver_service(
        "DisableRobot",
        "dobot_msgs_v3/srv/DisableRobot",
        "{}",
    )


_RESET_SSH_HOST = "192.168.200.254"
_RESET_SSH_USER = "qss"
_RESET_SSH_PASS = "123"


def _run_kubectl_rollout_restart_on_nuc(deployment: str) -> bool:
    """Run kubectl rollout restart on the NUC via SSH. Returns True on success."""
    cmd = f"kubectl rollout restart deployment {deployment} -n barns"
    ssh_cmd = [
        "sshpass", "-p", _RESET_SSH_PASS,
        "ssh", "-o", "StrictHostKeyChecking=no",
        f"{_RESET_SSH_USER}@{_RESET_SSH_HOST}",
        cmd,
    ]
    try:
        result = subprocess.run(
            ssh_cmd, capture_output=True, text=True, timeout=30
        )
        if result.returncode == 0:
            return True
        _log.error(
            "reset_robot: SSH/kubectl failed (exit %s): stderr=%s stdout=%s",
            result.returncode,
            (result.stderr or "").strip(),
            (result.stdout or "").strip(),
        )
        return False
    except FileNotFoundError as e:
        _log.error(
            "reset_robot: ssh or sshpass not found (install openssh-client sshpass in container): %s",
            e,
        )
        return False
    except subprocess.TimeoutExpired:
        _log.error("reset_robot: SSH to %s timed out (check network)", _RESET_SSH_HOST)
        return False


def reset_robot1(**params):
    """Restart robot1 deployment via kubectl on NUC (qss@192.168.200.254)."""
    return _run_kubectl_rollout_restart_on_nuc("robot1")


def reset_robot2(**params):
    """Restart robot2 deployment via kubectl on NUC (qss@192.168.200.254)."""
    return _run_kubectl_rollout_restart_on_nuc("robot2")


SEQUENCES = {
    'home': home,
    'return_back_to_home': return_back_to_home,
    'get_machine_position': get_machine_position,
    'check_saved_data': check_saved_data,
    'check_aruco_status': check_aruco_status,
    'open_gripper': open_gripper,
    'close_gripper': close_gripper,
    'toggle_drag_mode': toggle_drag_mode,
    'enable_robot': enable_robot,
    'disable_robot': disable_robot,
    'reset_robot1': reset_robot1,
    'reset_robot2': reset_robot2,
}
