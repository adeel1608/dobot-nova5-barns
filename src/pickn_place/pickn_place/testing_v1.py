#!/usr/bin/env python3
import time
import math
import argparse
import signal
import atexit
import sys
import threading
from typing import Dict, Any, Optional, Union, Tuple, List
import logging
import subprocess
_log = logging.getLogger(__name__)

# Handle both direct execution and module imports
try:
    from .computer_vision import detect_cup_gripper
except ImportError:
    from computer_vision import detect_cup_gripper

# Import all parameters and helper functions from params module
try:
    from .params import *
except ImportError:
    from params import *

#####################################################################
# 🔧 VERSION SELECTOR - Change this to switch between versions
#####################################################################
USE_VERSION = "v4"  # Change to "v1", "v2", "v3", or "v4"

# Import run_skill using direct module loading to avoid conflicts with oms_v1 package
import importlib.util
import os

def _get_local_run_skill():
    """Get run_skill function from the selected manipulate_node version"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    if USE_VERSION == "v1":
        module_path = os.path.join(current_dir, 'manipulate_node_v1.py')
        module_name = "manipulate_node_v1_local"
        class_name = "DirectTfMotionNode"  # v1 uses DirectTfMotionNode
    elif USE_VERSION == "v2":
        module_path = os.path.join(current_dir, 'manipulate_node_v2.py')
        module_name = "manipulate_node_v2_local"
        class_name = "robot_motion"  # v2 uses robot_motion
    elif USE_VERSION == "v3":
        module_path = os.path.join(current_dir, 'manipulate_node_v3.py')
        module_name = "manipulate_node_v3_local"
        class_name = "robot_motion"  # v3 uses robot_motion (hybrid of v1+v2)
    elif USE_VERSION == "v4":
        module_path = os.path.join(current_dir, 'manipulate_node_v4.py')
        module_name = "manipulate_node_v4_local"
        class_name = "robot_motion"  # v4 uses robot_motion (complete with all functions)
    else:
        raise ValueError(f"Invalid version: {USE_VERSION}. Use 'v1', 'v2', 'v3', or 'v4'")
    
    print(f"🔧 Loading {USE_VERSION} ({class_name}) from {module_path}")
    
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Failed to load {module_path}")
    
    local_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(local_module)
    
    # Get the correct class based on version
    motion_class = getattr(local_module, class_name)
    
    return local_module.run_skill, motion_class

# Get the correct run_skill function and motion class
run_skill_old, robot_motion_class = _get_local_run_skill()

import rclpy
from rclpy.node import Node
from dobot_msgs_v3.srv import (
    ServoJ,
    InverseSolution,
    PositiveSolution,
    GetPose,
    GetAngle,
    StartDrag,
    StopDrag,
    SetGripperPosition,
    GetGripperPosition,
    ClearError,
    DisableRobot,
    EnableRobot,
    ModbusClose,
    ModbusCreate,
    SetHoldRegs,
    CP,
)
import re

# Global persistent motion node (more efficient approach)
_global_motion_node = None
_rclpy_initialized = False

# Global variables for cleanup tracking
_cleanup_called = False
_cleanup_lock = threading.Lock()

def get_motion_node():
    """Get or create the global motion node instance"""
    global _global_motion_node, _rclpy_initialized
    
    if not _rclpy_initialized:
        # rclpy may already be initialized elsewhere in this process (e.g. by a
        # different module / previous sequence). Calling init() twice raises:
        # "Context.init() must only be called once".
        if not rclpy.ok():
            rclpy.init(args=None)
        _rclpy_initialized = True
    
    if _global_motion_node is None:
        _global_motion_node = robot_motion_class()
        print(f"✅ Initialized {USE_VERSION} motion node: {robot_motion_class.__name__}")
    
    return _global_motion_node

_DATA_RETURN_SKILLS = ("current_angles", "current_pose", "get_machine_position")

def run_skill(fn_name: str, *args):
    """
    Efficient run_skill that reuses a persistent motion node.
    Falls back to old approach if there are issues.

    Skills like set_gripper_position return a tuple (success, payload). The
    raw tuple is truthy even when success is False, which used to bypass
    every `if not ok(run_skill(...))` check downstream and let sequences
    keep marching past a failed gripper command. Unwrap such returns to a
    bool here so a single source of truth handles failure detection, and
    keep data-returning skills (current_angles, current_pose,
    get_machine_position) returning their raw payload.
    """
    try:
        motion_node = get_motion_node()
        fn = getattr(motion_node, fn_name)
        result = fn(*args)

        if fn_name in _DATA_RETURN_SKILLS:
            if result is None or result is False:
                motion_node.get_logger().error(f"{fn_name}{args} failed – aborting")
                cleanup_motion_node()
                raise RuntimeError(f"Skill {fn_name} failed")
            return result

        if isinstance(result, tuple):
            success = result[0] if len(result) > 0 else False
        else:
            success = result

        if success is False or success is None:
            motion_node.get_logger().error(f"{fn_name}{args} failed – aborting")
            cleanup_motion_node()
            raise RuntimeError(f"Skill {fn_name} failed")

        return success
    except Exception as e:
        print(f"Error with persistent node approach: {e}")
        print("Falling back to old run_skill approach...")
        return run_skill_old(fn_name, *args)

def cleanup_motion_node():
    """Clean up the global motion node with proper thread management"""
    global _global_motion_node, _rclpy_initialized, _cleanup_called
    
    with _cleanup_lock:
        if _cleanup_called:
            return
        _cleanup_called = True
    
    print("🧹 Starting cleanup process...")
    
    try:
        if _global_motion_node is not None:
            # Check if node has executor threads to join
            if hasattr(_global_motion_node, 'executor_thread'):
                print("   Stopping executor thread...")
                if _global_motion_node.executor_thread.is_alive():
                    _global_motion_node.executor_thread.join(timeout=2.0)
            
            print("   Destroying motion node...")
            _global_motion_node.destroy_node()
            _global_motion_node = None

        if _rclpy_initialized:
            print("   Shutting down rclpy...")
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"   Warning: rclpy shutdown error: {e}")
            _rclpy_initialized = False
            
        print("✅ Cleanup complete!")
        
    except Exception as e:
        print(f"❌ Error during cleanup: {e}")

def show_version_info():
    """Display current version and how to switch"""
    print(f"""
{'='*60}
🔧 MANIPULATE NODE VERSION SELECTOR
{'='*60}
Currently using: {USE_VERSION.upper()}
Class: {robot_motion_class.__name__}

To switch versions:
1. Edit line 16 in testing_v1.py
2. Change USE_VERSION = "{USE_VERSION}" to:
   - USE_VERSION = "v1"  # for manipulate_node_v1.py (DirectTfMotionNode)
   - USE_VERSION = "v2"  # for manipulate_node_v2.py (robot_motion)
   - USE_VERSION = "v3"  # for manipulate_node_v3.py (robot_motion)
   - USE_VERSION = "v4"  # for manipulate_node_v4.py (robot_motion)

Version Differences:
• v1: Original implementation with DirectTfMotionNode class
• v2: Updated implementation with robot_motion class and fixes
• v3: 🌟 HYBRID - Best of both worlds! 🌟
     ├── From v1: enforce_rxry, sync, move_portafilter_arc (working methods)
     └── From v2: All other methods with improvements and fixes
• v4: 🌟 COMPLETE - All functions included! 🌟
     ├── From v3: All methods with improvements and fixes
     └── NEW: move_portafilter_arc_movJ, move_portafilter_arc_tool, enforce_rxry_moveit, gotoEE_movJ
{'='*60}
""")

def switch_version():
    """Interactive version switcher"""
    global USE_VERSION
    current = USE_VERSION
    
    print(f"\nCurrent version: {current}")
    print("Available versions:")
    print("  1. v1 (manipulate_node_v1.py) - Original implementation")
    print("  2. v2 (manipulate_node_v2.py) - Updated with fixes")
    print("  3. v3 (manipulate_node_v3.py) - 🌟 HYBRID: Best methods from v1 + v2")
    print("  4. v4 (manipulate_node_v4.py) - 🌟 COMPLETE - All functions included!")
    
    choice = input("Enter version (v1/v2/v3/v4) or press Enter to keep current: ").strip().lower()
    
    if choice in ['v1', 'v2', 'v3', 'v4']:
        if choice != current:
            print(f"\n⚠️  To switch from {current} to {choice}:")
            print(f"   Edit line 9 in testing_v1.py:")
            print(f"   Change USE_VERSION = \"{current}\" to USE_VERSION = \"{choice}\"")
            print("   Then restart the script.")
            if choice == "v4":
                print("\n🌟 V4 BENEFITS:")
                print("   ✅ Working enforce_rxry from v1")
                print("   ✅ Reliable sync from v1") 
                print("   ✅ Stable move_portafilter_arc from v1")
                print("   ✅ All other improved methods from v2")
                print("   ✅ NEW: move_portafilter_arc_movJ")
                print("   ✅ NEW: move_portafilter_arc_tool")
                print("   ✅ NEW: enforce_rxry_moveit")
                print("   ✅ NEW: gotoEE_movJ")
        else:
            print(f"✅ Already using {choice}")
    elif choice == "":
        print(f"✅ Keeping current version: {current}")
    else:
        print("❌ Invalid choice. Use 'v1', 'v2', 'v3', or 'v4'")


"""
home.py

Defines the 'home' positioning routine using compass directions.
This module provides functions for robot positioning, machine calibration,
and system diagnostics for the BARNS coffee automation system.
"""

_log = logging.getLogger(__name__)
# from oms_v1.params import (
#     HOME_ANGLES, ESPRESSO_HOME, ESPRESSO_GRINDER_HOME,
#     HOME_CALIBRATION_PARAMS, HOME_CALIBRATION_CONSTANTS,
#     GRIPPER_FULL, GRIPPER_OPEN, SPEED_FAST
# )
# from oms_v1.manipulate_node import run_skill

MAX_CALIBRATION_RETRIES = 5


def _calibrate_marker(marker_name, prep_fn, ok):
    """Retry a single marker calibration up to MAX_CALIBRATION_RETRIES times.

    prep_fn must move the arm into the correct approach pose and call sync.
    Returns True on the first successful get_machine_position, False if all
    attempts are exhausted.
    """
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
        return False

    angles = HOME_ANGLES.get(str(position))
    if not angles:
        return False

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
        return False

    home_j2_j6 = HOME_CALIBRATION_PARAMS['return_home_position']
    if not ok(run_skill("gotoJ_deg", j1_val, *home_j2_j6)):
        return False

    return True

def get_machine_position(**params) -> bool:
    """
    Calibrate and record machine positions for all coffee equipment.
    """
    # from oms_v1.sequences.espresso import invalidate_port_cache, angled_invalidate_port_cache
    # from oms_v1.sequences.cleaning import (
    #     invalidate_cleaning_cache,
    #     angled_invalidate_cleaning_cache,
    # )
    # from oms_v1.sequences.milk_frothing import invalidate_milk_frothing_cache

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


"""
paper_cups.py

Defines the paper cup handling sequences for coffee service automation.
This module provides comprehensive functions for grabbing, placing, and serving
paper cups in the BARNS coffee automation system, including size-based handling
and staging area management.
"""

# from oms_v1.params import (
#     GRAB_PAPER_CUP_PARAMS, PLACE_PAPER_CUP_PARAMS,
#     PAPER_CUPS_NAVIGATION_PARAMS, PAPER_CUPS_STATION_PARAMS,
#     PAPER_CUP_GRIPPER_POSITIONS, PAPER_CUP_MOVEMENT_OFFSETS,
#     ESPRESSO_MOVEMENT_OFFSETS,
#     ESPRESSO_HOME, GRIPPER_OPEN, GRIPPER_RELEASE, GRIPPER_FULL,
#     _extract_cup_position, _extract_cups_dict, _normalize_cup_size
# )
# from oms_v1.manipulate_node import run_skill
# from oms_v1.sequences.home import home, return_back_to_home
# from oms_v1.sequences.computer_vision import detect_cup_gripper

def _normalize_paper_cup_size(cups_dict: Any) -> str:
    """
    Universal cup size normalizer for paper cup operations.
    Accepts BOTH H-codes AND C-codes regardless of prefix.
    Extracts the numeric size and returns standardized format.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz')
        
    Examples:
        cup_H9 → '9oz'
        cup_C9 → '9oz'
        cup_h12 → '12oz'
        cup_c7 → '7oz'
    """
    # if not cups_dict:
    #     from oms_v1.params import DEFAULT_PAPER_CUP_SIZE
    #     return DEFAULT_PAPER_CUP_SIZE
    
    # Extract the cup code (case-insensitive)
    if isinstance(cups_dict, dict):
        cup_key = next(iter(cups_dict.keys()), None)
        if cup_key:
            # Convert to uppercase for parsing
            cup_key_str = str(cup_key).upper()
            if 'CUP_' in cup_key_str:
                cup_code = cup_key_str.split('CUP_', 1)[1]
            else:
                cup_code = cup_key_str
            
            # Extract numeric size from code (works with both H and C prefixes)
            # H7, H9, H12, C7, C9, C12, C16 → extract the number
            if cup_code and len(cup_code) >= 2:
                # Remove H or C prefix if present
                if cup_code[0] in ('H', 'C'):
                    size_num = cup_code[1:]
                else:
                    size_num = cup_code
                
                # Validate and return standardized size (paper cups: 7, 9, 12)
                if size_num in ('7', '9', '12'):
                    return f"{size_num}oz"
    
    # If parsing failed, try the standard normalizers
    # Try paper first (since this is paper cup function)
    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result
    
    # Try plastic as fallback
    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '' and result in ('7oz', '9oz', '12oz'):
        return result
    
    # Final fallback
    # from oms_v1.params import DEFAULT_PAPER_CUP_SIZE
    # return DEFAULT_PAPER_CUP_SIZE

def grab_paper_cup(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
    if not size:
        return False
    
    cup_params = GRAB_PAPER_CUP_PARAMS.get(str(size))
    if not cup_params:
        cup_params = GRAB_PAPER_CUP_PARAMS.get("7oz")
        if not cup_params:
            return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['espresso_avoid'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['dispenser_area'])):
        return False
    
    attempt_count = 0
    while attempt_count < 15:
        if size == "7oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_7oz'])):
                return False
        elif size == "9oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_9oz'])):
                return False
        elif size == "12oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_12oz'])):
                return False
        else:
            return False
        
        if 'approach' in cup_params:
            if not ok(run_skill("moveEE", *cup_params['approach'])):
                return False
        
        if 'grip_width' not in cup_params:
            return False
        
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, cup_params['grip_width'])):
            return False
        
        if 'retreat' in cup_params:
            if not ok(run_skill("moveEE", *cup_params['retreat'])):
                return False
        
        cup_detected = detect_cup_gripper()
        if cup_detected:
            break
        
        attempt_count += 1
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)):
            return False
        if attempt_count == 15:
            return False
    
    return True

def place_paper_cup(**params) -> bool:
    """
    Place a paper cup at the specified staging area.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"
    
    stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
    if not stage_params:
        return False
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['intermediate'])):
        return False
    
    if 'twist' in stage_params:
        if not ok(run_skill("moveJ_deg", *stage_params['twist'])):
            return False
    
    if 'pose' not in stage_params:
        return False
    
    if not ok(run_skill("gotoJ_deg", *stage_params['pose'])):
        return False
    
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", 25, 100, 255)):
        return False
    if not ok(run_skill("set_gripper_position", 255, 0, 255)):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])):
        return False
    
    if 'stage_home' in stage_params:
        if not ok(run_skill("gotoJ_deg", *stage_params['stage_home'])):
            return False
    
    if 'twist_back' in stage_params:
        if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_back_machine'])):
            return False
    
    return True

def grab_paper_cup_arm1(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.

    First attempt does the full size-specific approach.
    If detection fails, retries only:
      1) open gripper
      2) move back up
      3) close gripper with size-specific width
      4) move back down
    """
    def ok(r):
        return r not in (False, None)

    cups_dict = _extract_cups_dict(params)
    size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
    if not size:
        return False

    size_cfg = {
        "7oz": {
            "home": "south_west",
            "grip": 165,
            "up_down_z": 200,
            "pose1": (191.646150,74.117401,-112.223690,-142.814608,11.629319,0.888704),
            "pose2": (191.667596,49.107588,-70.637578,-159.483294,11.665208,0.983008),
        },
        "9oz": {
            "home": "south_west",
            "grip": 165,
            "up_down_z": 200,
            "pose1": (181.710022,-5.584718,-88.052513,-93.715889,1.744494,7.348929),
            "pose2": (181.232376,-15.066946,-40.633915,-135.281052,1.234581,10.979085),
        },
        "12oz": {
            "home": "south_west",
            "grip": 165,
            "up_down_z": 200,
            "pose1": (175.946579,32.215965,-120.514900,-88.941734,-4.002728,-2.748568),
            "pose2": (175.906542,23.482150,-80.123117,-120.455698,-4.057709,-2.892895),
        },
    }

    cfg = size_cfg.get(size)
    if not cfg:
        return False

    if not ok(home(position=cfg["home"])):
        return False
    if not ok(run_skill("gotoJ_deg", *cfg["pose1"])):
        return False
    if not ok(run_skill("gotoJ_deg", *cfg["pose2"])):
        return False
    if not ok(run_skill("sync")):
        return False

    attempt_count = 0
    while attempt_count < 15:
        if attempt_count > 0:
            if not ok(run_skill("set_gripper_position", 255, 0, 255)):
                return False
            if not ok(run_skill("moveEE_movJ", 0, 0, cfg["up_down_z"], 0, 0, 0)):
                return False

        if not ok(run_skill("set_gripper_position", 255, cfg["grip"], 255)):
            return False
        if not ok(run_skill("moveEE_movJ", 0, 0, -cfg["up_down_z"], 0, 0, 0)):
            return False

        cup_detected = detect_cup_gripper()
        if cup_detected:
            break

        attempt_count += 1
        run_skill("sync")

        if attempt_count == 15:
            return False

    if not ok(home(position=cfg["home"])):
        return False
    return True
    # run_skill("set_gripper_position", 255, 0, 255)
    # run_skill("moveEE_movJ", 0, 0, -0.25, 0, 0, 0)
    # run_skill("set_gripper_position", 255, 165, 255)

def place_paper_cup_arm1(**params) -> bool:
    """
    Place a paper cup at the specified staging area.
    """
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = str(cup_position)

    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}

    size_mapped = _normalize_paper_cup_size(cups_dict)

    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')

    if stage not in valid_stages or size_mapped not in valid_sizes:
        return False

    stage_params_map = {
        "1": PLACE_PAPER_CUP_PARAMS['stage_1'],
        "2": PLACE_PAPER_CUP_PARAMS['stage_2'],
        "3": PLACE_PAPER_CUP_PARAMS['stage_3'],
        "4": PLACE_PAPER_CUP_PARAMS['stage_4'],
    }

    stage_params = stage_params_map[stage]

    if stage == "1":
        if not ok(run_skill("gotoJ_deg", 112.5, 30, -130, -90, -90, 0)):
            return False
    elif stage == "2":
        if not ok(home(position="south_west")):
            return False
    elif stage == "4":
        if not ok(home(position="south")):
            return False
    elif stage == "3":
        if not ok(home(position="south_west")):
            return False

    if 'pose' not in stage_params:
        return False

    if not ok(run_skill("gotoJ_deg", *stage_params['pose'])):
        return False

    run_skill("sync")

    if not ok(run_skill("set_gripper_position", 25, 100, 255)):
        return False

    if not ok(run_skill("set_gripper_position", 255, 0, 255)):
        return False

    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])):
        return False

    if not ok(run_skill("set_speed_factor", 100)):
        return False

    if 'stage_home' in stage_params:
        if not ok(run_skill("gotoJ_deg", *stage_params['stage_home'])):
            return False

    return True

def dispense_paper_arm1_cup_station(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
    """
    if not grab_paper_cup_arm1(**params):
        return False
    if not place_paper_cup_arm1(**params):
        return False
    return True

def grab_paper_arm2_cup_station(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.

    First attempt does the full size-specific approach.
    If detection fails, retries only:
      1) open gripper
      2) move back up
      3) close gripper with size-specific width
      4) move back down
    """
    def ok(r):
        return r not in (False, None)

    cups_dict = _extract_cups_dict(params)
    size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
    if not size:
        return False

    size_cfg = {
        "7oz": {
            "home": "north_east",
            "grip": 155,
            "up_down_z": 200,
            "pose1": (-52.257668,-36.013535,-91.807312,127.588043,52.159515,180.128571),
            "pose2": (-52.257133,-24.955849,-72.251076,96.955688,52.174652,180.158661),
            "visitfix":(0,0,-2,0,0,0),
        },
        "9oz": {
            "home": "south_east",
            "grip": 150,
            "up_down_z": 200,
            "pose1": (-108.257324,-43.628250,-75.009590,118.425400,108.173233,179.912598),
            "pose2": (-108.522827,-34.824387,-55.117859,89.734695,108.454109,179.928329),
            "visitfix":(0,8.5,-4.75,0,0,0),
        },
        "12oz": {
            "home": "east",
            "grip": 139,
            "up_down_z": 200,
            "pose1": (-82.175354,-30.120581,-95.428894,125.354034,82.074257,180.017426),
            "pose2": (-82.143822,-21.311819,-81.286293,102.399956,82.054932,180.037231),
            "visitfix":(0,0,-1.5,0,0,0),
        },
    }

    cfg = size_cfg.get(size)
    if not cfg:
        return False

    if not ok(home(position=cfg["home"])):
        return False
    if not ok(run_skill("gotoJ_deg", *cfg["pose1"])):
        return False
    if not ok(run_skill("gotoJ_deg", *cfg["pose2"])):
        return False
    if not ok(run_skill("moveEE_movJ", *cfg["visitfix"])):
        return False
    if not ok(run_skill("sync")):
        return False

    attempt_count = 0
    while attempt_count < 15:
        if attempt_count > 0:
            if not ok(run_skill("set_gripper_position", 255, 0, 255)):
                return False
            if not ok(run_skill("moveEE", 0, 0, cfg["up_down_z"], 0, 0, 0)):
                return False

        if not ok(run_skill("set_gripper_position", 255, cfg["grip"], 255)):
            return False
        if not ok(run_skill("moveEE", 0, 0, -cfg["up_down_z"], 0, 0, 0)):
            return False

        cup_detected = detect_cup_gripper()
        if cup_detected:
            break

        attempt_count += 1
        run_skill("sync")

        if attempt_count == 15:
            return False

    if not ok(home(position=cfg["home"])):
        return False
    if not ok(home(position="north_east")):
        return False
    return True
    # run_skill("gotoJ_deg", -82.176834,-21.495697,-80.396042,101.693619,82.088623,180.036987)
    # run_skill("set_gripper_position", 255, 0, 255)
    # run_skill("moveEE_movJ", 0, 1, -1, 0, 0, 0)
    # run_skill("set_gripper_position", 255, 139, 255)

def place_paper__arm2_cup_station(**params) -> bool:
    """
    Place a paper cup at specified staging area.
    """
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = str(cup_position)

    valid_stages = ("1", "2", "3", "4")
    if stage not in valid_stages:
        return False

    if stage in ("1", "2"):
        if not ok(home(position="east")):
            return False
    if stage in ("3", "4"):
        if not ok(home(position="south_east")):
            return False

    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS["staging"]["place_1"],
        "2": PAPER_CUPS_STATION_PARAMS["staging"]["place_2"],
        "3": PAPER_CUPS_STATION_PARAMS["staging"]["place_3"],
        "4": PAPER_CUPS_STATION_PARAMS["staging"]["place_4"],
    }

    stage_home_map = {
        "1": "east",
        "2": "east",
        "3": "south_east",
        "4": "south_east",
    }

    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    if not ok(run_skill("sync")):
        return False

    if not ok(run_skill("set_gripper_position", 25, 100, 255)):
        return False

    if not ok(run_skill("set_gripper_position", 255, 0, 255)):
        return False

    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS["place_up"])):
        return False

    if not ok(run_skill("set_speed_factor", 100)):
        return False

    if not ok(home(position=stage_home_map[stage])):
        return False

    return True

def dispense_paper_arm2_cup_station(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
    """
    if not grab_paper_arm2_cup_station(**params):
        return False
    if not place_paper__arm2_cup_station(**params):
        return False
    return True

def dispense_paper_cup_station(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
    """
    if not grab_paper_cup(**params):
        return False
    if not place_paper_cup(**params):
        return False
    return True

def pick_paper_cup_station(**params) -> bool:
    """
    Pick up a paper cup from a specific stage.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return False
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['pickup_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['pickup_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['pickup_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['pickup_4']
    }
    
    gripper_positions = PAPER_CUP_GRIPPER_POSITIONS
    
    if not home(position="east"):
        return False
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return False
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_down'])):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[size_mapped])):
        return False
    
    run_skill("moveEE_movJ", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_up'])
    
    if not home(position="east"):
        return False
    
    if not home(position="north_east"):
        return False
    
    return True

def place_paper_cup_station(**params) -> bool:
    """
    Place a paper cup at specified staging area.

    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return False
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['place_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['place_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['place_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['place_4']
    }
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_return_up'])):
        return False
    if not home(position="east"):
        return False
    
    return True

def place_paper_cup_sauces(**params) -> bool:
    """
    Place the paper cup at the sauces station.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position3'])):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", 255, 90, 255)):
        return False
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return False
    return True

def pick_paper_cup_sauces(**params) -> bool:
    """
    Pick the paper cup from the sauces station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    cup_size = _normalize_paper_cup_size(cups_dict)
    valid_sizes = ("7oz", "9oz", "12oz")
    if cup_size not in valid_sizes:
        return False
    
    gripper_position = 140
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return False
    if not ok(run_skill("moveEE", -1,0,0,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_position)):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position1'])):
        return False
    return True

def place_paper_cup_milk(**params) -> bool:
    """
    Place the paper cup at the milk station.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position1'])):
        return False
    # if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position2'])):
    #     return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position3'])):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", 255, 90, 255)):
        return False
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return False
    return True

def pick_paper_cup_milk(**params) -> bool:
    """
    Pick the paper cup from the milk station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    cup_size = _normalize_paper_cup_size(cups_dict)
    valid_sizes = ("7oz", "9oz", "12oz")
    if cup_size not in valid_sizes:
        return False
    
    gripper_position = 140
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return False
    if not ok(run_skill("moveEE", -1,0,-5,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_position)):
        return False
    # if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position2'])):
    #     return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position1'])):
        return False
    return True

def pick_cup_for_hot_water(**params) -> bool:
    """
    Pick up a paper cup from a specific stage for hot water.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return False
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_4']
    }
    
    gripper_positions = PAPER_CUP_GRIPPER_POSITIONS
    
    if not home(position="south_west"):
        return False
    if stage in ("3", "4"):
        if not home(position="south"):
            return False
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_hot_water_down'])):
        return False
    if size_mapped == '12oz':
        if not ok(run_skill("set_gripper_position", 255,120,255)):
            return False
    else:
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255,130,255)):
            return False
    
    run_skill("sync")
    
    if not ok(run_skill("set_speed_factor", 75)):
        return False
    
    if not ok(run_skill("moveEE_movJ", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_up'])):
        return False
    
    if not home(position="west"):
        return False

    if not ok(run_skill("approach_machine", "three_group_espresso", "hot_water")):
        return False
    
    if not ok(run_skill("mount_machine", "three_group_espresso", "hot_water")):
        return False

    run_skill("sync")    
    
    return True

def return_cup_with_hot_water(**params) -> bool:
    """
    Complete hot water dispensing sequence and return to holding position.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return False
    
    stage_params_map = {
        "1": PLACE_PAPER_CUP_PARAMS['stage_1'],
        "2": PLACE_PAPER_CUP_PARAMS['stage_2'],
        "3": PLACE_PAPER_CUP_PARAMS['stage_3'],
        "4": PLACE_PAPER_CUP_PARAMS['stage_4'],
    }
    
    stage_params = stage_params_map.get(stage, {})

    if not ok(run_skill("set_speed_factor",25)):
        return False

    if not ok(run_skill("moveEE", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return False

    if stage in ("1"):
        if not run_skill("gotoJ_deg", 112.5,30,-130,-90,-90,0):
            return False
    if stage in ("2","3", "4"):
        if not home(position="south_west"):
            return False
    if stage in ("3", "4"):
        if not home(position="south"):
            return False
    
    if 'pose' not in stage_params:
        return False
    
    if not ok(run_skill("gotoJ_deg", *stage_params['pose'])):
        return False
    
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])):
        return False

    if not ok(run_skill("set_speed_factor", 100)):
        return False
    
    if 'stage_home' in stage_params:
        if not ok(run_skill("gotoJ_deg", *stage_params['stage_home'])):
            return False
    
    if 'twist_back' in stage_params:
        if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_back_machine'])):
            return False
    
    return True

"""
espresso.py

Defines the espresso-making sequence for different ports and cups.
This module provides comprehensive functions for managing the complete espresso
workflow including portafilter handling, grinding, tamping, mounting, and milk operations.
"""

# from oms_v1.manipulate_node import run_skill, init_motion_node
# from oms_v1.params import (
#     PULL_ESPRESSO_PARAMS,
#     ESPRESSO_HOME,
#     ESPRESSO_GRINDER_HOME,
#     ESPRESSO_GRINDER_PARAMS,
#     ESPRESSO_PITCHER_PARAMS,
#     ESPRESSO_HOT_WATER_PARAMS,
#     ESPRESSO_SPEEDS, ESPRESSO_PITCHER_GRIPPER, ESPRESSO_PORTAFILTER_GRIPPER,
#     ESPRESSO_MOVEMENT_OFFSETS, ESPRESSO_DELAYS,
#     GRIPPER_FULL, GRIPPER_OPEN, SPEED_FAST, SPEED_SUPER_SLOW, SPEED_SLOW_POURING,
#     _extract_cup_position
# )

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
_PORTAFILTER_GRIP_POS_MIN = 135
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

def _portafilter_clear_up_offset(port: str) -> Tuple[float, float, float, float, float, float]:
    """Use per-port learned Z from last live unmount, else params default."""
    z = _portafilter_clear_up_z_mm_by_port.get(str(port))
    if z is not None:
        return (0.0, 0.0, float(z), 0.0, 0.0, 0.0)
    base = ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_up"]
    return tuple(float(x) for x in base)

def _portafilter_clear_up_angled_offset(port: str) -> Tuple[float, float, float, float, float, float]:
    """Angled mount clear-up: learned Z from last live angled unmount, else params default."""
    z = _portafilter_clear_up_z_mm_by_port.get(str(port))
    base = ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_up_angled"]
    if z is not None:
        return (float(base[0]), float(base[1]), float(z), float(base[3]), float(base[4]), float(base[5]))
    return tuple(float(x) for x in base)

def _angled_unmount_grab_tool_name(port: str) -> str:
    """Portafilter tool frame for angled unmount grab (per robot teach)."""
    if str(port) == "angled_portafilter_2":
        return "single_portafilter_angled"
    return "double_portafilter_angled"

def _open_gripper_with_verify(speed=255, force=255):
    """Send gripper-open (position 0) and verify it actually reached a low position.

    Retries up to _GRIPPER_OPEN_RETRIES times if the reported position is above
    _GRIPPER_OPEN_MAX_POS (gripper did not physically open).
    """
    node = get_motion_node()
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
    return False

def invalidate_port_cache():
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
    cached_angles = _machine_approach_pose_cache.get(cache_key)
    if _is_valid_angles(cached_angles):
        if run_skill("gotoJ_deg", *cached_angles) in (False, None):
            return False
        # run_skill("sync")
        return True

    if run_skill("approach_machine", machine_name, target_name) in (False, None):
        return False
    run_skill("sync")
    captured_angles = run_skill("current_angles")
    if not _is_valid_angles(captured_angles):
        return False
    _machine_approach_pose_cache[cache_key] = tuple(captured_angles)
    return True

def _run_cached_machine_mount(cache_key: str, machine_name: str, target_name: str) -> bool:
    """
    Replay a cached pose captured immediately after a successful mount_machine(...).
    If no cache exists yet, run the live mount, sync, capture current_angles, and cache them.
    """
    cached_angles = _machine_mount_pose_cache.get(cache_key)
    if _is_valid_angles(cached_angles):
        if run_skill("gotoJ_deg", *cached_angles) in (False, None):
            return False
        # run_skill("sync")
        return True

    if run_skill("mount_machine", machine_name, target_name) in (False, None):
        return False
    run_skill("sync")
    captured_angles = run_skill("current_angles")
    if not _is_valid_angles(captured_angles):
        return False
    _machine_mount_pose_cache[cache_key] = tuple(captured_angles)
    return True

def _normalize_espresso_shot(espresso_dict: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    try:
        if not espresso_dict or not isinstance(espresso_dict, dict):
            return None

        espresso_key = next(iter(espresso_dict.keys()), None)
        if not espresso_key:
            return None

        espresso_key_lower = str(espresso_key).lower()

        if 'single' in espresso_key_lower:
            return {
                "port": "port_2",
                "positioning_time": 1.2,
                "portafilter_tool": "single_portafilter",
            }
        elif 'double' in espresso_key_lower:
            value = espresso_dict.get(espresso_key)
            if value is not None and float(value) == 2.0:
                return {
                    "port": "angled_portafilter_1",
                    "positioning_time": 2.4,
                    "portafilter_tool": "double_portafilter_angled",
                    "angled": True,
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
                        "port": "port_2",
                        "positioning_time": 1.2,
                        "portafilter_tool": "single_portafilter",
                    }
                elif shots == 2.0:
                    return {
                        "port": "angled_portafilter_1",
                        "positioning_time": 2.4,
                        "portafilter_tool": "double_portafilter_angled",
                        "angled": True,
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

    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_unmount(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port:
        return False

    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return False

    if not ok(run_skill("gotoJ_deg", *port_params['home'])):
        return False

    if port in ('port_1', 'port_3'):
        if not _run_cached_machine_approach(
            f"unmount:{port}:approach:{port_params['portafilter_number']}",
            "three_group_espresso",
            port_params['portafilter_number'],
        ):
            return False

    grab_cache_key = f"unmount:post_grab:{port}"
    cached_grab_joints = _unmount_post_grab_joints_cache.get(grab_cache_key)
    if _is_valid_angles(cached_grab_joints):
        if not ok(run_skill("gotoJ_deg", *cached_grab_joints)):
            return False
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255, 255, 255)):
            return False
    else:
        def _close_and_verify_grip():
            """Close gripper and return (gripped_ok, reported_position)."""
            node = get_motion_node()
            success, pos = node.set_gripper_position(speed=255, position=255, force=255)
            if not success:
                return False, None
            ok_reading = (
                pos is not None
                and _PORTAFILTER_GRIP_POS_MIN <= pos <= _PORTAFILTER_GRIP_POS_MAX
            )
            return ok_reading, pos

        def _grab_then_close():
            """Perform grab_tool + close + read. Returns (gripped_ok, pos)."""
            run_skill("sync")
            if not ok(run_skill("grab_tool", "double_portafilter")):
                return False, None
            run_skill("sync")
            return _close_and_verify_grip()

        gripped, pos = _grab_then_close()

        # Attempt 2: nudge -5 mm in Z and re-close.
        if not gripped:
            _gripper_log.warning(
                f"[PORTAFILTER-GRIP] attempt 1 pos={pos} "
                f"(want in [{_PORTAFILTER_GRIP_POS_MIN}, {_PORTAFILTER_GRIP_POS_MAX}]); "
                f"nudging down 5 mm and retrying close"
            )
            if not ok(run_skill("moveEE_movJ", 0, 0, -2.5, 0, 0, 0)):
                return False
            gripped, pos = _close_and_verify_grip()

        # Attempt 3: nudge +10 mm in Z and re-close.
        if not gripped:
            _gripper_log.warning(
                f"[PORTAFILTER-GRIP] attempt 2 pos={pos} "
                f"(want in [{_PORTAFILTER_GRIP_POS_MIN}, {_PORTAFILTER_GRIP_POS_MAX}]); "
                f"nudging up 10 mm and retrying close"
            )
            if not ok(run_skill("moveEE_movJ", 0, 0, 5, 0, 0, 0)):
                return False
            gripped, pos = _close_and_verify_grip()

        # Attempt 4: full recovery — open gripper, re-run cached approach, re-grab.
        if not gripped:
            _gripper_log.warning(
                f"[PORTAFILTER-GRIP] attempt 3 pos={pos} "
                f"(want in [{_PORTAFILTER_GRIP_POS_MIN}, {_PORTAFILTER_GRIP_POS_MAX}]); "
                f"opening gripper and re-running approach"
            )
            if not ok(run_skill("set_gripper_position", 255, 0, 255)):
                return False
            if port in ('port_1', 'port_3'):
                if not _run_cached_machine_approach(
                    f"unmount:{port}:approach:{port_params['portafilter_number']}",
                    "three_group_espresso",
                    port_params['portafilter_number'],
                ):
                    return False
            gripped, pos = _grab_then_close()

        if not gripped:
            _gripper_log.error(
                f"[PORTAFILTER-GRIP] FINAL FAIL pos={pos} "
                f"(wanted in [{_PORTAFILTER_GRIP_POS_MIN}, {_PORTAFILTER_GRIP_POS_MAX}]); "
                f"aborting unmount for {port}"
            )
            return False

        _gripper_log.info(f"[PORTAFILTER-GRIP] gripped OK, pos={pos}")

        if not ok(run_skill("release_tension")):
            return False
        run_skill("sync")
        run_skill("enforce_rxry")
        run_skill("sync")

        # Z height check (current_pose is mm per manipulate_node_v4.current_pose).
        z_lo = _UNMOUNT_POST_TENSION_Z_TARGET_MM - _UNMOUNT_POST_TENSION_Z_TOL_MM
        z_hi = _UNMOUNT_POST_TENSION_Z_TARGET_MM + _UNMOUNT_POST_TENSION_Z_TOL_MM
        pose_z = run_skill("current_pose")
        if not ok(pose_z) or not isinstance(pose_z, (tuple, list)) or len(pose_z) < 3:
            return False
        z_mm = float(pose_z[2])
        if not (z_lo <= z_mm <= z_hi):
            dz = (_UNMOUNT_POST_TENSION_Z_TARGET_MM - z_mm)/1.0
            _gripper_log.warning(
                f"[UNMOUNT-Z] after release_tension z={z_mm:.2f} mm outside [{z_lo:.1f}, {z_hi:.1f}]; "
                f"moveEE_movJ dz={dz:.2f} mm"
            )
            if not ok(run_skill("set_gripper_position", 25, 100, 25)):
                return False
            if not ok(run_skill("moveEE_movJ", -0.25, 0, dz, 0, 0, 0)):
                return False
            if not ok(run_skill("set_gripper_position", 255, 255, 255)):
                return False
            run_skill("sync")
            if not ok(run_skill("release_tension")):
                return False            
            run_skill("sync")
        angles = run_skill("current_angles")
        if not ok(angles) or not _is_valid_angles(angles):
            return False
        _unmount_post_grab_joints_cache[grab_cache_key] = tuple(angles)
        run_skill("sync")

    if not ok(run_skill("enforce_rxry")):
        return False
    run_skill("sync")

    cached_port_angle = _port_angle_cache.get(port)

    if cached_port_angle:
        arc_cmd = _portafilter_arc_cmd_by_port.get(str(port))
        if arc_cmd is None:
            return False

        if not ok(run_skill("move_portafilter_arc_movJ", arc_cmd)):
            return False

    else:
        pose_before_arc = run_skill("current_pose")
        if not ok(pose_before_arc) or not isinstance(pose_before_arc, (tuple, list)) or len(pose_before_arc) < 6:
            return False

        current_rz = float(pose_before_arc[5])
        desired_rz = 44.0

        arc_delta = current_rz - desired_rz
        arc_cmd = -arc_delta
        arc_delta_mount = arc_delta + 3.5

        _portafilter_arc_cmd_by_port[str(port)] = float(arc_cmd)
        _portafilter_mount_arc_cmd_by_port[str(port)] = float(arc_delta_mount)

        _gripper_log.info(
            f"[PORTAFILTER-ARC] port={port} current_rz={current_rz:.3f}, "
            f"desired_rz={desired_rz:.3f}, arc_cmd={arc_cmd:.3f}"
        )

        if not ok(run_skill("move_portafilter_arc_movJ", arc_cmd)):
            return False

    cached = _port_angle_cache.get(port)
    if cached:
        if not ok(run_skill("release_tension")):
            return False
        mount_pose = cached['mount']
        below_pose = cached['below']
        if not _is_valid_angles(below_pose):
            return False
        if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down'])):
            return False
    else:
        pose_after_arc = run_skill("current_pose")
        if not ok(pose_after_arc) or not isinstance(pose_after_arc, (tuple, list)) or len(pose_after_arc) < 3:
            return False
        z_after_arc_mm = float(pose_after_arc[2])

        if not ok(run_skill("release_tension")):
            return False

        run_skill("sync")
        mount_pose = run_skill("current_angles")
        pose_after_tension = run_skill("current_pose")
        if not ok(pose_after_tension) or not isinstance(pose_after_tension, (tuple, list)) or len(pose_after_tension) < 3:
            return False
        z_after_tension_mm = float(pose_after_tension[2])
        dz_drop_mm = z_after_arc_mm - z_after_tension_mm + 0.1
        if dz_drop_mm > 0.0:
            learned_clear_up_z = float(math.ceil(dz_drop_mm))
        else:
            learned_clear_up_z = float(ESPRESSO_MOVEMENT_OFFSETS["portafilter_clear_up"][2])
        _portafilter_clear_up_z_mm_by_port[str(port)] = learned_clear_up_z
        _gripper_log.info(
            f"[CLEAR-UP-Z] port={port} z_arc={z_after_arc_mm:.2f} z_after_tension={z_after_tension_mm:.2f} "
            f"drop={dz_drop_mm:.2f} mm -> portafilter_clear_up z={learned_clear_up_z} mm"
        )
        if not _is_valid_angles(mount_pose):
            return False
        if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down'])):
            return False
        below_pose = run_skill("current_angles")
        if not _is_valid_angles(below_pose):
            return False
        mount_pose = tuple(mount_pose)
        below_pose = tuple(below_pose)
        _port_angle_cache[port] = {'mount': mount_pose, 'below': below_pose}

    _mount_runtime_cache[port] = {'mount': tuple(mount_pose), 'below': tuple(below_pose)}
    mount_espresso_port = tuple(mount_pose)
    below_espresso_port = tuple(below_pose)

    if not ok(run_skill("gotoJ_deg", *port_params['move_back'])):
        return False

    if port in ('port_2', 'port_3'):
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav2'])):
            return False

    return True

def grinder(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_grinder(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    positioning_time = params.get("positioning_time")
    if positioning_time is None:
        positioning_time = (shot_cfg.get("positioning_time") if shot_cfg else 2.4)
    portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "double_portafilter")

    if not port or portafilter_tool not in ('single_portafilter', 'double_portafilter'):
        return False

    if port == 'port_1':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
            return False

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "espresso_grinder", "grinder")):
    #     return False
    if not _run_cached_machine_approach(
        f"grinder:{port}:approach:grinder",
        "espresso_grinder",
        "grinder",
    ):
        return False

    # BEST: cache the post-mount pose per port.
    # OLD shared-style idea kept here for reference:
    # cached_grinder_mount_pose = _tool_pick_pose_cache.get("grinder_post_mount")
    grinder_cache_key = f"{port}_grinder_post_mount"
    cached_grinder_mount_pose = _grinder_post_mount_cache.get(grinder_cache_key)

    if _is_valid_angles(cached_grinder_mount_pose):
        if not ok(run_skill("gotoJ_deg", *cached_grinder_mount_pose)):
            return False
        # run_skill("sync")
    else:
        # OLD live call kept for rollback:
        # if not ok(run_skill("mount_machine", "espresso_grinder", "grinder")):
        #     return False
        if not _run_cached_machine_mount(
            f"grinder:{port}:mount:grinder",
            "espresso_grinder",
            "grinder",
        ):
            return False
        grinder_mount_pose = run_skill("current_angles")
        if not _is_valid_angles(grinder_mount_pose):
            return False
        _grinder_post_mount_cache[grinder_cache_key] = tuple(grinder_mount_pose)

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "espresso_grinder", "tamper")):
    #     return False
    if not _run_cached_machine_approach(
        f"grinder:{port}:approach:tamper",
        "espresso_grinder",
        "tamper",
    ):
        return False
    run_skill("sync")
    time.sleep(positioning_time)

    # OLD live call kept for rollback:
    # if not ok(run_skill("mount_machine", "espresso_grinder", "grinder")):
    #     return False
    if not _run_cached_machine_mount(
        f"grinder:{port}:mount:grinder:final",
        "espresso_grinder",
        "grinder",
    ):
        return False

    # OLD live call kept for rollback:
    # if not ok(run_skill("mount_machine", "espresso_grinder", "tamper")):
    #     return False
    if not _run_cached_machine_mount(
        f"grinder:{port}:mount:tamper",
        "espresso_grinder",
        "tamper",
    ):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", 255,0,255)):
        return False

    cached_tool_pick_pose = _tool_pick_pose_cache.get(portafilter_tool)
    if _is_valid_angles(cached_tool_pick_pose):
        if not ok(run_skill("gotoJ_deg", *cached_tool_pick_pose)):
            return False
    else:
        if not ok(run_skill("moveEE_movJ", -50, 50, 50, 15, 0, 0)):
            return False
        tool_pick_pose = run_skill("current_angles")
        if not _is_valid_angles(tool_pick_pose):
            return False
        _tool_pick_pose_cache[portafilter_tool] = tuple(tool_pick_pose)

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    return True

def single_grinder(**params) -> bool:
    params["portafilter_tool"] = "single_portafilter"
    return grinder(**params)

def double_grinder(**params) -> bool:
    params["portafilter_tool"] = "double_portafilter"
    return grinder(**params)

def tamper(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_tamper(**params)
    portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "single_portafilter")

    if portafilter_tool not in ('single_portafilter', 'double_portafilter'):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    cached_tool_pick_pose = _tool_pick_pose_cache.get(portafilter_tool)
    if _is_valid_angles(cached_tool_pick_pose):
        if not ok(run_skill("gotoJ_deg", *cached_tool_pick_pose)):
            return False
    else:
        run_skill("sync")
        if not ok(run_skill("move_to", portafilter_tool, 0.22)):
            return False
        run_skill("sync")
        if not ok(run_skill("approach_tool", portafilter_tool)):
            return False

    post_grab_pose = _tamper_post_grab_joints_cache.get(portafilter_tool)

    if _is_valid_angles(post_grab_pose):
        if not ok(run_skill("gotoJ_deg", *post_grab_pose)):
            return False
        run_skill("sync")
    else:
        run_skill("sync")
        if not ok(run_skill("grab_tool", portafilter_tool)):
            return False
        run_skill("sync")
        post_grab_angles = run_skill("current_angles")
        if not _is_valid_angles(post_grab_angles):
            return False
        _tamper_post_grab_joints_cache[portafilter_tool] = tuple(post_grab_angles)

    if not ok(run_skill("set_gripper_position", 255,255,255)):
        return False

    if not ok(run_skill("moveEE", 0, 0, 40, 0, 0, 0)):
        return False

    if not _run_cached_machine_mount(
        f"tamper:{portafilter_tool}:mount:grinder",
        "espresso_grinder",
        "grinder",
    ):
        return False

    if not _run_cached_machine_approach(
        f"tamper:{portafilter_tool}:approach:grinder",
        "espresso_grinder",
        "grinder",
    ):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    return True

def single_tamper(**params) -> bool:
    params["portafilter_tool"] = "single_portafilter"
    return tamper(**params)

def double_tamper(**params) -> bool:
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_tamper(**params)
    params["portafilter_tool"] = "double_portafilter"
    return tamper(**params)

def mount(**params) -> bool:
    global below_espresso_port, mount_espresso_port

    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_mount(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port:
        return False

    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return False

    if port in ('port_2', 'port_3'):
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])):
            return False

    if not ok(run_skill("gotoJ_deg", *port_params['move_back'])):
        return False

    runtime_cached = _mount_runtime_cache.get(port)
    if runtime_cached:
        below_pose = runtime_cached.get('below')
        mount_pose = runtime_cached.get('mount')
    else:
        below_pose = below_espresso_port
        mount_pose = mount_espresso_port

    if not _is_valid_angles(below_pose):
        return False

    if not ok(run_skill("gotoJ_deg", *below_pose)):
        return False

    if not _is_valid_angles(mount_pose):
        return False

    if not ok(run_skill("gotoJ_deg", *mount_pose)):
        return False

    if not ok(run_skill("moveEE_movJ", *_portafilter_clear_up_offset(port))):
        return False

    if not ok(run_skill("enforce_rxry")):
        return False

    run_skill("sync")

    arc_delta_mount = _portafilter_mount_arc_cmd_by_port.get(str(port))
    if arc_delta_mount is None:
        return False

    if not ok(run_skill("move_portafilter_arc_movJ", arc_delta_mount)):
        return False
    run_skill("sync")
    run_skill("release_tension")

    if not _open_gripper_with_verify():
        return False

    run_skill("sync")

    if port in ('port_1', 'port_3'):
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'])):
        #     return False
        if not _run_cached_machine_approach(
            f"mount:{port}:approach:{port_params['portafilter_number']}",
            "three_group_espresso",
            port_params['portafilter_number'],
        ):
            return False

    if not ok(run_skill("gotoJ_deg", *port_params['home'])):
        return False

    return True

def grab_espresso_pitcher(**params) -> bool:
    """
    Grab the espresso pitcher and stop right after closing the gripper.
    """
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_grab_espresso_pitcher(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False

    cached = _pitcher_pick_cache.get(port)
    pick2 = cached.get("pick2_approach") if cached else None
    if _is_valid_angles(pick2):
        if not ok(run_skill("gotoJ_deg", *pick2)):
            return False
        # run_skill("sync")
    else:
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        #     return False
        if not _run_cached_machine_approach(
            f"grab_pitcher:{port}:approach:pick_pitcher_2",
            "three_group_espresso",
            "pick_pitcher_2",
        ):
            return False
        run_skill("sync")
        pick2_angles = run_skill("current_angles")
        if _is_valid_angles(pick2_angles):
            _pitcher_pick_cache.setdefault(port, {})["pick2_approach"] = tuple(pick2_angles)

    cached = _pitcher_pick_cache.get(port)

    if port == 'port_1':
        if cached and cached.get('approach') and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['approach'])):
                return False
            # run_skill("sync")
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
            run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            #     return False
            if not _run_cached_machine_approach(
                f"grab_pitcher:{port}:approach:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            approach_angles = run_skill("current_angles")
            if _is_valid_angles(approach_angles):
                _pitcher_pick_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

            # OLD live call kept for rollback:
            # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
            #     return False
            if not _run_cached_machine_mount(
                f"grab_pitcher:{port}:mount:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_pick_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        if not ok(run_skill("set_gripper_position", 255, 115, 255)):
            return False

    elif port == 'port_2':
        if cached and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
            run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
            #     return False
            if not _run_cached_machine_mount(
                f"grab_pitcher:{port}:mount:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_pick_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        if not ok(run_skill("set_gripper_position", 255, 115, 255)):
            return False

    elif port == 'port_3':
        if cached and cached.get('approach') and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['approach'])):
                return False
            # run_skill("sync")
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
            run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            #     return False
            if not _run_cached_machine_approach(
                f"grab_pitcher:{port}:approach:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            approach_angles = run_skill("current_angles")
            if _is_valid_angles(approach_angles):
                _pitcher_pick_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

            # OLD live call kept for rollback:
            # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
            #     return False
            if not _run_cached_machine_mount(
                f"grab_pitcher:{port}:mount:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_pick_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        if not ok(run_skill("set_gripper_position", 255, 115, 255)):
            return False

    else:
        return False

    return True

def pick_espresso_pitcher(**params) -> bool:
    """
    Complete pitcher pickup after grab_espresso_pitcher().
    """
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_pick_espresso_pitcher(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False

    cached = _pitcher_pick_cache.get(port)

    run_skill("sync")
    run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])

    if port == 'port_1':
        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            #     return False
            if not _run_cached_machine_approach(
                f"pick_pitcher:{port}:retreat:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if _is_valid_angles(retreat_angles):
                _pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_2':
        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
            #     return False
            if not _run_cached_machine_approach(
                f"pick_pitcher:{port}:retreat:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if _is_valid_angles(retreat_angles):
                _pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_3':
        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            #     return False
            if not _run_cached_machine_approach(
                f"pick_pitcher:{port}:retreat:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if _is_valid_angles(retreat_angles):
                _pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    if port in ('port_1', 'port_2'):
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
            return False

    return True

def pour_espresso_pitcher_cup_station(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"

    run_skill("gotoJ_deg", 103.201965, -21.933174, -150.611664, -10.398072, -23.882843, 0.127716)

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return False

    if stage == 'stage_1':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos1'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.2'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral1'])):
            return False
    elif stage == 'stage_2':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos2'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.2'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral2'])):
            return False
    elif stage == 'stage_3':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.2'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral3'])):
            return False
    else:
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos4'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.2'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral4'])):
            return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return False

    run_skill("gotoJ_deg", 103.201965, -21.933174, -150.611664, -10.398072, -23.882843, 0.127716)
    # run_skill("sync")

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
        return False

    return True

def get_hot_water(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "three_group_espresso", "hot_water")):
    #     return False
    if not _run_cached_machine_approach(
        "get_hot_water:approach:hot_water",
        "three_group_espresso",
        "hot_water",
    ):
        return False

    # OLD live call kept for rollback:
    # if not ok(run_skill("mount_machine", "three_group_espresso", "hot_water")):
    #     return False
    if not _run_cached_machine_mount(
        "get_hot_water:mount:hot_water",
        "three_group_espresso",
        "hot_water",
    ):
        return False

    run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_move'])
    return True

def with_hot_water(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    run_skill("set_speed_factor", ESPRESSO_SPEEDS['hot_water_pour'])

    if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return False

    return True

def return_espresso_pitcher(**params) -> bool:
    global approach_pitcher, pick_pitcher

    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_return_espresso_pitcher(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False

    cached = _pitcher_return_cache.get(port)

    if port == 'port_1':
        if cached and cached.get('approach'):
            if not ok(run_skill("gotoJ_deg", *cached['approach'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            #     return False
            if not _run_cached_machine_approach(
                f"return_pitcher:{port}:approach:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            approach_angles = run_skill("current_angles")
            if _is_valid_angles(approach_angles):
                _pitcher_return_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

        if cached and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
            #     return False
            if not _run_cached_machine_mount(
                f"return_pitcher:{port}:mount:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_return_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        run_skill("sync")

        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            #     return False
            if not _run_cached_machine_approach(
                f"return_pitcher:{port}:retreat:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if _is_valid_angles(retreat_angles):
                _pitcher_return_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_2':
        if cached and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
            #     return False
            if not _run_cached_machine_mount(
                f"return_pitcher:{port}:mount:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_return_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        run_skill("sync")

    elif port == 'port_3':
        if cached and cached.get('approach'):
            if not ok(run_skill("gotoJ_deg", *cached['approach'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            #     return False
            if not _run_cached_machine_approach(
                f"return_pitcher:{port}:approach:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            approach_angles = run_skill("current_angles")
            if _is_valid_angles(approach_angles):
                _pitcher_return_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

        if cached and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
            #     return False
            if not _run_cached_machine_mount(
                f"return_pitcher:{port}:mount:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if _is_valid_angles(mount_angles):
                _pitcher_return_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        run_skill("sync")

        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
            # run_skill("sync")
        else:
            # OLD live call kept for rollback:
            # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            #     return False
            if not _run_cached_machine_approach(
                f"return_pitcher:{port}:retreat:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if _is_valid_angles(retreat_angles):
                _pitcher_return_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
    #     return False
    if not _run_cached_machine_approach(
        f"return_pitcher:{port}:final_approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False

    return True

def return_cleaned_espresso_pitcher(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    if shot_cfg and shot_cfg.get("angled"):
        return angled_return_cleaned_espresso_pitcher(**params)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
    #     return False
    if not _run_cached_machine_approach(
        f"return_clean_pitcher:{port}:approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return False

    cached = _pitcher_clean_cache.get(port)

    if port == 'port_1':
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
        #     return False
        if not _run_cached_machine_approach(
            f"return_clean_pitcher:{port}:approach:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return False
        # OLD live call kept for rollback:
        # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
        #     return False
        if not _run_cached_machine_mount(
            f"return_clean_pitcher:{port}:mount:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return False
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255,115,255)):
            return False
        if cached:
            for angles in cached:
                if not ok(run_skill("gotoJ_deg", *angles)):
                    return False
        else:
            waypoints = []
            run_skill("moveEE_movJ", 0, 0, 20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, -170)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, 170)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveEE_movJ", 0, 0, -20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            if all(_is_valid_angles(w) for w in waypoints):
                _pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        # run_skill("sync")
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
        #     return False
        if not _run_cached_machine_approach(
            f"return_clean_pitcher:{port}:final_approach:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return False

    elif port == 'port_2':
        # OLD live call kept for rollback:
        # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
        #     return False
        if not _run_cached_machine_mount(
            f"return_clean_pitcher:{port}:mount:pick_pitcher_2",
            "three_group_espresso",
            "pick_pitcher_2",
        ):
            return False
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255,115,255)):
            return False
        if cached:
            for angles in cached:
                if not ok(run_skill("gotoJ_deg", *angles)):
                    return False
        else:
            waypoints = []
            run_skill("moveEE_movJ", 0, 0, 20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, -170)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, 170)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveEE_movJ", 0, 0, -20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            if all(_is_valid_angles(w) for w in waypoints):
                _pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False

    elif port == 'port_3':
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
        #     return False
        if not _run_cached_machine_approach(
            f"return_clean_pitcher:{port}:approach:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return False
        # OLD live call kept for rollback:
        # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
        #     return False
        if not _run_cached_machine_mount(
            f"return_clean_pitcher:{port}:mount:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return False
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255,115,255)):
            return False
        # run_skill("sync")
        if cached:
            for angles in cached:
                if not ok(run_skill("gotoJ_deg", *angles)):
                    return False
        else:
            waypoints = []
            run_skill("moveEE_movJ", 0, 0, 20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, -135)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, 135)
            waypoints.append(run_skill("current_angles"))
            run_skill("moveEE_movJ", 0, 0, -20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            if all(_is_valid_angles(w) for w in waypoints):
                _pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        # run_skill("sync")
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
        #     return False
        if not _run_cached_machine_approach(
            f"return_clean_pitcher:{port}:final_approach:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return False

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
    #     return False
    if not _run_cached_machine_approach(
        f"return_clean_pitcher:{port}:final_approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False

    return True

def unmount_single(**params) -> bool:
    params["port"] = "port_2"
    return unmount(**params)

def unmount_double(**params) -> bool:
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_unmount(**params)
    params["port"] = "port_1"
    return unmount(**params)

def mount_single(**params) -> bool:
    params["port"] = "port_2"
    return mount(**params)

def mount_double(**params) -> bool:
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_mount(**params)
    params["port"] = "port_1"
    return mount(**params)

def single_grab_espresso_pitcher(**params) -> bool:
    params["port"] = "port_2"
    return grab_espresso_pitcher(**params)

def double_grab_espresso_pitcher(**params) -> bool:
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_grab_espresso_pitcher(**params)
    params["port"] = "port_1"
    return grab_espresso_pitcher(**params)

def single_pick_espresso_pitcher(**params) -> bool:
    params["port"] = "port_2"
    return pick_espresso_pitcher(**params)

def double_pick_espresso_pitcher(**params) -> bool:
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_pick_espresso_pitcher(**params)
    params["port"] = "port_1"
    return pick_espresso_pitcher(**params)

def single_pour_espresso_pitcher_cup_station(**params) -> bool:
    params["port"] = "port_2"
    return pour_espresso_pitcher_cup_station(**params)

def double_pour_espresso_pitcher_cup_station(**params) -> bool:
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_pour_espresso_pitcher_cup_station(**params)
    params["port"] = "port_1"
    return pour_espresso_pitcher_cup_station(**params)

def single_return_espresso_pitcher(**params) -> bool:
    params["port"] = "port_2"
    return return_espresso_pitcher(**params)

def double_return_espresso_pitcher(**params) -> bool:
    shot_cfg = _normalize_espresso_shot(params.get("espresso"))
    if shot_cfg and shot_cfg.get("angled"):
        return angled_return_espresso_pitcher(**params)
    params["port"] = "port_1"
    return return_espresso_pitcher(**params)

def single_return_cleaned_espresso_pitcher(**params) -> bool:
    params["port"] = "port_2"
    return return_cleaned_espresso_pitcher(**params)

def double_return_cleaned_espresso_pitcher(**params) -> bool:
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
        return False

    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return False

    if not ok(run_skill("gotoJ_deg", *port_params['home'])):
        return False

    if port == 'angled_portafilter_2':
        if not _run_cached_machine_approach(
            f"angled_unmount:{port}:approach:{port_params['portafilter_number']}",
            "three_group_espresso",
            port_params['portafilter_number'],
        ):
            return False

    grab_cache_key = f"angled_unmount:post_grab:{port}"
    cached_grab_joints = _unmount_post_grab_joints_cache.get(grab_cache_key)

    if _is_valid_angles(cached_grab_joints):
        if not ok(run_skill("gotoJ_deg", *cached_grab_joints)):
            return False

        run_skill("sync")

        if not ok(run_skill("set_gripper_position", 255, 255, 255)):
            return False

    else:
        ANGLED_UNMOUNT_GRIP_POS_MIN = 138
        ANGLED_UNMOUNT_GRIP_POS_MAX = 143
        ANGLED_UNMOUNT_GRIP_RETRIES = 15

        def trace_grip(msg: str):
            print(f"[ANGLED-UNMOUNT-GRIP] {msg}", flush=True)

        def _close_and_verify_grip_angled():
            node = get_motion_node()
            success, pos = node.set_gripper_position(speed=255, position=255, force=255)
            if not success:
                return False, None

            ok_reading = (
                pos is not None
                and ANGLED_UNMOUNT_GRIP_POS_MIN <= pos <= ANGLED_UNMOUNT_GRIP_POS_MAX
            )
            return ok_reading, pos

        def _grab_then_close_angled():
            run_skill("sync")

            if not ok(run_skill("grab_tool", grab_tool_name)):
                return False, None

            run_skill("sync")
            return _close_and_verify_grip_angled()

        def _rerun_approach_for_retry(attempt_idx: int) -> bool:
            trace_grip(f"full routine attempt {attempt_idx}: open gripper START")
            if not ok(run_skill("set_gripper_position", 255, 0, 255)):
                trace_grip(f"full routine attempt {attempt_idx}: FAIL open gripper")
                return False
            trace_grip(f"full routine attempt {attempt_idx}: open gripper DONE")

            run_skill("sync")
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
                    return False

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
                    return False

            trace_grip(f"full routine attempt {display_attempt}: grab_then_close START")
            gripped, pos = _grab_then_close_angled()
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

            if not ok(run_skill("release_tension")):
                trace_grip(f"full routine attempt {display_attempt}: FAIL release_tension")
                return False

            run_skill("sync")
            trace_grip(f"full routine attempt {display_attempt}: release_tension DONE")

            z_tgt = _UNMOUNT_POST_TENSION_Z_TARGET_ANGL_MM
            z_lo = z_tgt - _UNMOUNT_POST_TENSION_Z_TOL_MM
            z_hi = z_tgt + _UNMOUNT_POST_TENSION_Z_TOL_MM

            pose_z = run_skill("current_pose")
            if not ok(pose_z) or not isinstance(pose_z, (tuple, list)) or len(pose_z) < 3:
                trace_grip(f"full routine attempt {display_attempt}: FAIL current_pose for Z check")
                return False

            z_mm = float(pose_z[2])

            trace_grip(
                f"full routine attempt {display_attempt}: Z check "
                f"z={z_mm:.2f}, target={z_tgt:.2f}, range=[{z_lo:.2f}, {z_hi:.2f}]"
            )

            if not (z_lo <= z_mm <= z_hi):
                dz = (z_tgt - z_mm) / 1.0
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

                if not ok(run_skill("set_gripper_position", 25, 100, 25)):
                    trace_grip(f"full routine attempt {display_attempt}: FAIL loosen gripper before Z correction")
                    return False

                if not ok(run_skill("moveEE_movJ", dx, 0, dz, 0, 0, 0)):
                    trace_grip(f"full routine attempt {display_attempt}: FAIL moveEE_movJ Z correction")
                    return False

                if not ok(run_skill("set_gripper_position", 255, 255, 255)):
                    trace_grip(f"full routine attempt {display_attempt}: FAIL re-close gripper after Z correction")
                    return False

                run_skill("sync")

                if not ok(run_skill("release_tension")):
                    trace_grip(f"full routine attempt {display_attempt}: FAIL release_tension after Z correction")
                    return False

                run_skill("sync")
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

            angles = run_skill("current_angles")
            if not ok(angles) or not _is_valid_angles(angles):
                trace_grip(f"full routine attempt {display_attempt}: FAIL current_angles after final verify")
                return False

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
            return False

        _unmount_post_grab_joints_cache[grab_cache_key] = final_angles

        trace_grip(
            f"CACHED post-grab joints for key={grab_cache_key}, final_pos={final_pos}"
        )

        run_skill("sync")

    run_skill("sync")

    cached_port_angle = angled__port_angle_cache.get(port)

    if cached_port_angle:
        arc_cmd_by_2 = angled__portafilter_arc_cmd_by_port_2.get(str(port))
        if arc_cmd_by_2 is None:
            return False

        for i in range(2):
            if not ok(run_skill("move_portafilter_arc_tool_angled", arc_cmd_by_2)):
                return False
        run_skill("sync")

    else:
        pose_before_arc = run_skill("current_pose")
        if (
            not ok(pose_before_arc)
            or not isinstance(pose_before_arc, (tuple, list))
            or len(pose_before_arc) < 6
        ):
            return False

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
            if not ok(run_skill("move_portafilter_arc_tool_angled", arc_cmd_by_2)):
                return False
        run_skill("sync")
        if not ok(run_skill("moveJ_deg", 0, 0, 0, 0, 0, 1)):
            return False
        run_skill("sync")

    # run_skill("sync")

    cached = angled__port_angle_cache.get(port)

    if cached:
        mount_pose = cached['angled_mount']
        below_pose = cached['below']

        if not angled__is_valid_angles(below_pose):
            return False

        if not ok(run_skill("set_speed_factor", 25)):
            return False

        if not ok(run_skill("moveEE_movJ", 0.5, -4.0, -30, 0, 0, 0)):
            return False

        if not ok(run_skill("set_speed_factor", 100)):
            return False

    else:
        pose_after_arc = run_skill("current_pose")
        if not ok(pose_after_arc) or not isinstance(pose_after_arc, (tuple, list)) or len(pose_after_arc) < 3:
            return False

        z_after_arc_mm = float(pose_after_arc[2])

        run_skill("sync")

        mount_pose = run_skill("current_angles")

        pose_after_tension = run_skill("current_pose")
        if not ok(pose_after_tension) or not isinstance(pose_after_tension, (tuple, list)) or len(pose_after_tension) < 3:
            return False

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
            return False

        run_skill("set_speed_factor", 25)

        if not ok(run_skill("moveEE_movJ", 0.5, -4.0, -30, 0, 0, 0)):
            return False

        run_skill("set_speed_factor", 100)

        below_pose = run_skill("current_angles")
        if not angled__is_valid_angles(below_pose):
            return False

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

    run_skill("gotoJ_deg", 41.352814, -59.951000, -116.487274, 72.716667, -14.247798, -75.843781)
    run_skill("gotoJ_deg", 59.183862, -61.661050, -115.736025, 72.719532, -13.555987, -74.867439)
    run_skill("gotoJ_deg", 56.791147, -36.511041, -128.165316, -14.824074, -33.131641, -0.433359)
    run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("gotoJ_deg", -32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)

    return True

def angled_grinder(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "angled_portafilter_1")
    positioning_time = params.get("positioning_time")
    if positioning_time is None:
        positioning_time = (shot_cfg.get("positioning_time") if shot_cfg else 2.4)
    portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "double_portafilter_angled")

    if not port or portafilter_tool not in ("double_portafilter_angled", "single_portafilter_angled"):
        return False

    if port in ('port_1', 'angled_portafilter_1', 'angled_portafilter_2'):
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
            return False

    if not _run_cached_machine_approach(
        f"angled_grinder:{port}:approach:grinder",
        "espresso_grinder",
        "angled_grinder",
    ):
        return False

    grinder_cache_key = f"{port}_angled_grinder_post_mount"
    cached_grinder_mount_pose = angled__grinder_post_mount_cache.get(grinder_cache_key)

    if angled__is_valid_angles(cached_grinder_mount_pose):
        if not ok(run_skill("gotoJ_deg", *cached_grinder_mount_pose)):
            return False
    else:
        if not _run_cached_machine_mount(
            f"angled_grinder:{port}:mount:grinder",
            "espresso_grinder",
            "angled_grinder",
        ):
            return False

        grinder_mount_pose = run_skill("current_angles")
        if not angled__is_valid_angles(grinder_mount_pose):
            return False
        angled__grinder_post_mount_cache[grinder_cache_key] = tuple(grinder_mount_pose)

    if not _run_cached_machine_approach(
        f"angled_grinder:{port}:approach:tamper",
        "espresso_grinder",
        "angled_tamper",
    ):
        return False
    run_skill("sync")
    time.sleep(positioning_time)

    if not _run_cached_machine_mount(
        f"angled_grinder:{port}:mount:grinder:final",
        "espresso_grinder",
        "angled_grinder",
    ):
        return False

    if not _run_cached_machine_mount(
        f"angled_grinder:{port}:mount:tamper",
        "espresso_grinder",
        "angled_tamper",
    ):
        return False

    run_skill("sync")
    if not ok(run_skill("set_gripper_position", 255, 0, 255)):
        return False

    cached_tool_pick_pose = angled__tool_pick_pose_cache.get(portafilter_tool)
    if angled__is_valid_angles(cached_tool_pick_pose):
        if not ok(run_skill("gotoJ_deg", *cached_tool_pick_pose)):
            return False
    else:
        if not ok(run_skill("moveEE_movJ", -50, 50, 50, 15, 0, 0)):
            return False
        if not ok(run_skill("approach_tool", portafilter_tool)):
            return False
        run_skill("sync")
        tool_pick_pose = run_skill("current_angles")
        if not angled__is_valid_angles(tool_pick_pose):
            return False
        angled__tool_pick_pose_cache[portafilter_tool] = tuple(tool_pick_pose)

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    return True

def angled_single_grinder(**params) -> bool:
    params.setdefault("portafilter_tool", "single_portafilter_angled")
    params.setdefault("port", "angled_portafilter_2")
    return angled_grinder(**params)

def angled_double_grinder(**params) -> bool:
    params.setdefault("portafilter_tool", "double_portafilter_angled")
    params.setdefault("port", "angled_portafilter_1")
    return angled_grinder(**params)

def angled_tamper(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    portafilter_tool = params.get("portafilter_tool") or (
        shot_cfg.get("portafilter_tool") if shot_cfg else "single_portafilter_angled"
    )

    if portafilter_tool not in ("double_portafilter_angled", "single_portafilter_angled"):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    cached_tool_pick_pose = angled__tool_pick_pose_cache.get(portafilter_tool)
    if angled__is_valid_angles(cached_tool_pick_pose):
        if not ok(run_skill("gotoJ_deg", *cached_tool_pick_pose)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("move_to", portafilter_tool, 0.22)):
            return False
        run_skill("sync")

        if not ok(run_skill("approach_tool", portafilter_tool)):
            return False

    # Local tolerance for angled tamper grip.
    # Your latest code was actually checking 139..143, so keep that explicit.
    ANGLED_TAMPER_GRIP_POS_MIN = 139
    ANGLED_TAMPER_GRIP_POS_MAX = 143
    ANGLED_TAMPER_GRIP_RETRIES = 5

    def _close_and_verify_grip_angled():
        node = get_motion_node()
        success, pos = node.set_gripper_position(speed=255, position=255, force=255)
        if not success:
            return False, None

        ok_reading = (
            pos is not None
            and ANGLED_TAMPER_GRIP_POS_MIN <= pos <= ANGLED_TAMPER_GRIP_POS_MAX
        )
        return ok_reading, pos

    def _grab_then_close_angled():
        run_skill("sync")

        if not ok(run_skill("grab_tool", portafilter_tool)):
            return False, None

        run_skill("sync")
        return _close_and_verify_grip_angled()

    used_uncached_post_grab = False

    post_grab_pose = _angled_tamper_post_grab_joints_cache.get(portafilter_tool)

    if angled__is_valid_angles(post_grab_pose):
        if not ok(run_skill("gotoJ_deg", *post_grab_pose)):
            return False

        run_skill("sync")

        if not ok(run_skill("set_gripper_position", 255, 255, 255)):
            return False

        run_skill("sync")

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

            if not ok(run_skill("set_gripper_position", 255, 0, 255)):
                return False

            run_skill("sync")

            retry_tool_pick_pose = angled__tool_pick_pose_cache.get(portafilter_tool)
            if angled__is_valid_angles(retry_tool_pick_pose):
                if not ok(run_skill("gotoJ_deg", *retry_tool_pick_pose)):
                    return False
                run_skill("sync")
            else:
                if not ok(run_skill("move_to", portafilter_tool, 0.22)):
                    return False
                run_skill("sync")

            if not ok(run_skill("approach_tool", portafilter_tool)):
                return False

            run_skill("sync")

            gripped, pos = _grab_then_close_angled()

        if not gripped:
            _gripper_log.error(
                f"[ANGLED-TAMPER-GRIP] FINAL FAIL tool={portafilter_tool} "
                f"pos_read={pos}; aborting angled_tamper"
            )
            return False

        _gripper_log.info(f"[ANGLED-TAMPER-GRIP] gripped OK, pos={pos}")

        run_skill("sync")

        post_grab_angles = run_skill("current_angles")
        if not angled__is_valid_angles(post_grab_angles):
            return False

        _angled_tamper_post_grab_joints_cache[portafilter_tool] = tuple(post_grab_angles)

    if not ok(run_skill("moveEE", 0, 0, -2.5, 0, 0, 0)):
        return False

    if not ok(run_skill("release_tension")):
        return False

    # Only verify after release_tension on uncached/live grab path.
    if used_uncached_post_grab:
        gripped_after_tension, pos_after_tension = _close_and_verify_grip_angled()

        if not gripped_after_tension:
            _gripper_log.error(
                f"[ANGLED-TAMPER-GRIP] after release_tension pos={pos_after_tension} "
                f"outside [{ANGLED_TAMPER_GRIP_POS_MIN}, {ANGLED_TAMPER_GRIP_POS_MAX}]; "
                f"aborting angled_tamper"
            )
            return False

        _gripper_log.info(
            f"[ANGLED-TAMPER-GRIP] after release_tension gripped OK, "
            f"pos={pos_after_tension}"
        )

        run_skill("sync")

    if not ok(run_skill("moveEE", 0, 0, 40, 0, 0, 0)):
        return False

    if not _run_cached_machine_mount(
        f"angled_tamper:{portafilter_tool}:mount:grinder",
        "espresso_grinder",
        "angled_grinder",
    ):
        return False

    if not _run_cached_machine_approach(
        f"angled_tamper:{portafilter_tool}:approach:grinder",
        "espresso_grinder",
        "angled_grinder",
    ):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    return True

def angled_single_tamper(**params) -> bool:
    params.setdefault("portafilter_tool", "single_portafilter_angled")
    return angled_tamper(**params)

def angled_double_tamper(**params) -> bool:
    params.setdefault("portafilter_tool", "double_portafilter_angled")
    return angled_tamper(**params)

ANGLED_MOUNT_DEBUG = True
def angled_mount(**params) -> bool:
    global angled_below_espresso_port, angled_mount_espresso_port

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
        return False

    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    trace(f"port_params exists={bool(port_params)}")

    if not port_params:
        trace(f"FAIL: missing PULL_ESPRESSO_PARAMS for port={port}")
        return False

    trace("nav waypoint 1 START")
    if not ok(run_skill("gotoJ_deg", -32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)):
        trace("FAIL: nav waypoint 1")
        return False
    trace("nav waypoint 1 DONE")

    trace("nav waypoint 2 START")
    if not ok(run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)):
        trace("FAIL: nav waypoint 2")
        return False
    trace("nav waypoint 2 DONE")

    trace("nav waypoint 3 START")
    if not ok(run_skill("gotoJ_deg", 56.791147, -36.511041, -128.165316, -14.824074, -33.131641, -0.433359)):
        trace("FAIL: nav waypoint 3")
        return False
    trace("nav waypoint 3 DONE")

    trace("nav waypoint 4 START")
    if not ok(run_skill("gotoJ_deg", 59.183862, -61.661050, -115.736025, 72.719532, -13.555987, -74.867439)):
        trace("FAIL: nav waypoint 4")
        return False
    trace("nav waypoint 4 DONE")

    trace("nav waypoint 5 START")
    if not ok(run_skill("gotoJ_deg", 41.352814, -59.951000, -116.487274, 72.716667, -14.247798, -75.843781)):
        trace("FAIL: nav waypoint 5")
        return False
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
        return False

    trace("goto below_pose START")
    if not ok(run_skill("gotoJ_deg", *below_pose)):
        trace("FAIL: goto below_pose")
        return False
    trace("goto below_pose DONE")

    trace(f"mount_pose={mount_pose}")
    trace(f"mount_pose valid={angled__is_valid_angles(mount_pose)}")

    if not angled__is_valid_angles(mount_pose):
        trace("FAIL: invalid mount_pose")
        return False

    trace("sync before slow mount START")
    run_skill("sync")
    trace("sync before slow mount DONE")

    trace("set_speed_factor 25 START")
    if not ok(run_skill("set_speed_factor", 25)):
        trace("FAIL: set_speed_factor 25")
        return False
    trace("set_speed_factor 25 DONE")

    trace("goto mount_pose START")
    if not ok(run_skill("gotoJ_deg", *mount_pose)):
        trace("FAIL: goto mount_pose")
        return False
    trace("goto mount_pose DONE")

    trace("sync after mount_pose START")
    run_skill("sync")
    trace("sync after mount_pose DONE")

    trace("set_speed_factor 100 START")
    if not ok(run_skill("set_speed_factor", 100)):
        trace("FAIL: set_speed_factor 100 before arc")
        return False
    trace("set_speed_factor 100 DONE")

    trace("reading arc_delta_mount START")
    arc_delta_mount = angled__portafilter_mount_arc_cmd_by_port.get(str(port))
    trace(f"arc_delta_mount={arc_delta_mount}")

    if arc_delta_mount is None:
        trace(f"FAIL: missing arc_delta_mount for port={port}")
        return False

    trace("move_portafilter_arc_tool_angled START")
    if not ok(run_skill("move_portafilter_arc_tool_angled", arc_delta_mount)):
        trace("FAIL: move_portafilter_arc_tool_angled")
        return False
    trace("move_portafilter_arc_tool_angled DONE")

    trace("sync after arc START")
    run_skill("sync")
    trace("sync after arc DONE")

    trace("release_tension START")
    if not ok(run_skill("release_tension")):
        trace("FAIL: release_tension")
        return False
    trace("release_tension DONE")

    trace("sync after release_tension START")
    run_skill("sync")
    trace("sync after release_tension DONE")

    trace("_open_gripper_with_verify START")
    if not _open_gripper_with_verify():
        trace("FAIL: _open_gripper_with_verify")
        return False
    trace("_open_gripper_with_verify DONE")

    trace("sync after gripper open START")
    run_skill("sync")
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
            return False
        trace("final machine approach DONE")
    else:
        trace("final machine approach SKIPPED")

    trace("goto port home START")
    if not ok(run_skill("gotoJ_deg", *port_params["home"])):
        trace("FAIL: goto port home")
        return False
    trace("goto port home DONE")

    trace("SUCCESS")
    return True

def angled_grab_espresso_pitcher(**params) -> bool:
    """
    Grab the espresso pitcher and stop right after closing the gripper.
    """
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3', 'angled_portafilter_1'):
        return False
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    cached = angled__pitcher_pick_cache.get(port)
    pick2 = cached.get("pick2_approach") if cached else None
    if angled__is_valid_angles(pick2):
        if not ok(run_skill("gotoJ_deg", *pick2)):
            return False
        run_skill("sync")
    else:
        if not _run_cached_machine_approach(
            f"angled_grab_pitcher:{port}:approach:pick_pitcher_2",
            "three_group_espresso",
            "pick_pitcher_2",
        ):
            return False
        run_skill("sync")
        pick2_angles = run_skill("current_angles")
        if angled__is_valid_angles(pick2_angles):
            angled__pitcher_pick_cache.setdefault(port, {})["pick2_approach"] = tuple(pick2_angles)

    cached = angled__pitcher_pick_cache.get(port)

    if port in ('port_1', 'angled_portafilter_1'):
        if cached and cached.get('approach') and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['approach'])):
                return False
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_grab_pitcher:{port}:approach:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            approach_angles = run_skill("current_angles")
            if angled__is_valid_angles(approach_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

            if not _run_cached_machine_mount(
                f"angled_grab_pitcher:{port}:mount:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['mount'] = tuple(mount_angles)
            
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255, 115, 255)):
            return False

    elif port == 'port_2':
        if cached and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
        else:
            if not _run_cached_machine_mount(
                f"angled_grab_pitcher:{port}:mount:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['mount'] = tuple(mount_angles)
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255, 115, 255)):
            return False

    elif port == 'port_3':
        if cached and cached.get('approach') and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['approach'])):
                return False
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_grab_pitcher:{port}:approach:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            approach_angles = run_skill("current_angles")
            if angled__is_valid_angles(approach_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

            if not _run_cached_machine_mount(
                f"angled_grab_pitcher:{port}:mount:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['mount'] = tuple(mount_angles)
        
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255, 115, 255)):
            return False

    else:
        return False

    return True

def angled_pick_espresso_pitcher(**params) -> bool:
    """
    Complete pitcher pickup after angled_grab_espresso_pitcher().
    """
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3', 'angled_portafilter_1'):
        return False

    cached = angled__pitcher_pick_cache.get(port)

    run_skill("sync")
    run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])

    if port in ('port_1', 'angled_portafilter_1'):
        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_pick_pitcher:{port}:retreat:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_2':
        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_pick_pitcher:{port}:retreat:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_3':
        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_pick_pitcher:{port}:retreat:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_pick_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    if port in ('port_1', 'angled_portafilter_1', 'port_2'):
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
            return False

    return True

def angled_pour_espresso_pitcher_cup_station(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"

    run_skill("gotoJ_deg", 103.201965, -21.933174, -150.611664, -10.398072, -23.882843, 0.127716)

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return False

    if stage == 'stage_1':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos1'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.2'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1.3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral1'])):
            return False
    elif stage == 'stage_2':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos2'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.2'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2.3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral2'])):
            return False
    elif stage == 'stage_3':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.2'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3.3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral3'])):
            return False
    else:
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos4'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.2'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4.3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral4'])):
            return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return False

    run_skill("gotoJ_deg", 103.201965, -21.933174, -150.611664, -10.398072, -23.882843, 0.127716)

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
        return False

    return True

def angled_get_hot_water(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "three_group_espresso", "hot_water")):
    #     return False
    if not _run_cached_machine_approach(
        "angled_get_hot_water:approach:hot_water",
        "three_group_espresso",
        "hot_water",
    ):
        return False

    # OLD live call kept for rollback:
    # if not ok(run_skill("mount_machine", "three_group_espresso", "hot_water")):
    #     return False
    if not _run_cached_machine_mount(
        "angled_get_hot_water:mount:hot_water",
        "three_group_espresso",
        "hot_water",
    ):
        return False

    run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_move'])
    return True

def angled_with_hot_water(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    run_skill("set_speed_factor", ESPRESSO_SPEEDS['hot_water_pour'])

    if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return False

    return True

def angled_return_espresso_pitcher(**params) -> bool:
    global angled_approach_pitcher, angled_pick_pitcher

    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3', 'angled_portafilter_1'):
        return False

    cached = angled__pitcher_return_cache.get(port)

    if port in ('port_1', 'angled_portafilter_1'):
        if cached and cached.get('approach'):
            if not ok(run_skill("gotoJ_deg", *cached['approach'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_return_pitcher:{port}:approach:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            approach_angles = run_skill("current_angles")
            if angled__is_valid_angles(approach_angles):
                angled__pitcher_return_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

        if cached and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
        else:
            if not _run_cached_machine_mount(
                f"angled_return_pitcher:{port}:mount:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_return_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_return_pitcher:{port}:retreat:pick_pitcher_1",
                "three_group_espresso",
                "pick_pitcher_1",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_return_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    elif port == 'port_2':
        if cached and cached.get('mount'):
            pick_cached = angled__pitcher_pick_cache.get(port)
            pick_retreat = pick_cached.get('retreat') if pick_cached else None

            if not angled__is_valid_angles(pick_retreat):
                return False

            if not ok(run_skill("gotoJ_deg", *pick_retreat)):
                return False

            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
        else:
            pick_cached = angled__pitcher_pick_cache.get(port)
            pick_retreat = pick_cached.get('retreat') if pick_cached else None

            if not angled__is_valid_angles(pick_retreat):
                return False

            if not ok(run_skill("gotoJ_deg", *pick_retreat)):
                return False

            if not _run_cached_machine_mount(
                f"angled_return_pitcher:{port}:mount:pick_pitcher_2",
                "three_group_espresso",
                "pick_pitcher_2",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_return_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False

    elif port == 'port_3':
        if cached and cached.get('approach'):
            if not ok(run_skill("gotoJ_deg", *cached['approach'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_return_pitcher:{port}:approach:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            approach_angles = run_skill("current_angles")
            if angled__is_valid_angles(approach_angles):
                angled__pitcher_return_cache.setdefault(port, {})['approach'] = tuple(approach_angles)

        if cached and cached.get('mount'):
            if not ok(run_skill("gotoJ_deg", *cached['mount'])):
                return False
        else:
            if not _run_cached_machine_mount(
                f"angled_return_pitcher:{port}:mount:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            mount_angles = run_skill("current_angles")
            if angled__is_valid_angles(mount_angles):
                angled__pitcher_return_cache.setdefault(port, {})['mount'] = tuple(mount_angles)

        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        if cached and cached.get('retreat'):
            if not ok(run_skill("gotoJ_deg", *cached['retreat'])):
                return False
        else:
            if not _run_cached_machine_approach(
                f"angled_return_pitcher:{port}:retreat:pick_pitcher_3",
                "three_group_espresso",
                "pick_pitcher_3",
            ):
                return False
            run_skill("sync")
            retreat_angles = run_skill("current_angles")
            if angled__is_valid_angles(retreat_angles):
                angled__pitcher_return_cache.setdefault(port, {})['retreat'] = tuple(retreat_angles)

    if not _run_cached_machine_approach(
        f"angled_return_pitcher:{port}:final_approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False

    return True

def angled_return_cleaned_espresso_pitcher(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")

    if not port or port not in ('port_1', 'port_2', 'port_3', 'angled_portafilter_1'):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
    #     return False
    if not _run_cached_machine_approach(
        f"angled_return_clean_pitcher:{port}:approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return False

    cached = angled__pitcher_clean_cache.get(port)

    if port in ('port_1', 'angled_portafilter_1'):
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
        #     return False
        if not _run_cached_machine_approach(
            f"angled_return_clean_pitcher:{port}:approach:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return False
        # OLD live call kept for rollback:
        # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
        #     return False
        if not _run_cached_machine_mount(
            f"angled_return_clean_pitcher:{port}:mount:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return False
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255,115,255)):
            return False
        if cached:
            for angles in cached:
                if not ok(run_skill("gotoJ_deg", *angles)):
                    return False
        else:
            waypoints = []
            run_skill("moveEE_movJ", 0, 0, 20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, -170)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, 170)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveEE_movJ", 0, 0, -20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            if all(angled__is_valid_angles(w) for w in waypoints):
                angled__pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        # run_skill("sync")
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
        #     return False
        if not _run_cached_machine_approach(
            f"angled_return_clean_pitcher:{port}:final_approach:pick_pitcher_1",
            "three_group_espresso",
            "pick_pitcher_1",
        ):
            return False

    elif port == 'port_2':
        # OLD live call kept for rollback:
        # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
        #     return False
        if not _run_cached_machine_mount(
            f"angled_return_clean_pitcher:{port}:mount:pick_pitcher_2",
            "three_group_espresso",
            "pick_pitcher_2",
        ):
            return False
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255,115,255)):
            return False
        if cached:
            for angles in cached:
                if not ok(run_skill("gotoJ_deg", *angles)):
                    return False
        else:
            waypoints = []
            run_skill("moveEE_movJ", 0, 0, 20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, -170)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, 170)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveEE_movJ", 0, 0, -20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            if all(angled__is_valid_angles(w) for w in waypoints):
                angled__pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False

    elif port == 'port_3':
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
        #     return False
        if not _run_cached_machine_approach(
            f"angled_return_clean_pitcher:{port}:approach:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return False
        # OLD live call kept for rollback:
        # if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
        #     return False
        if not _run_cached_machine_mount(
            f"angled_return_clean_pitcher:{port}:mount:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return False
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 255,115,255)):
            return False
        # run_skill("sync")
        if cached:
            for angles in cached:
                if not ok(run_skill("gotoJ_deg", *angles)):
                    return False
        else:
            waypoints = []
            run_skill("moveEE_movJ", 0, 0, 20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, -135)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveJ_deg", 0, 0, 0, 0, 0, 135)
            run_skill("sync")
            waypoints.append(run_skill("current_angles"))
            run_skill("moveEE_movJ", 0, 0, -20, 0, 0, 0)
            waypoints.append(run_skill("current_angles"))
            if all(angled__is_valid_angles(w) for w in waypoints):
                angled__pitcher_clean_cache[port] = [tuple(w) for w in waypoints]
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", 35,0,255)):
            return False
        # run_skill("sync")
        # OLD live call kept for rollback:
        # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
        #     return False
        if not _run_cached_machine_approach(
            f"angled_return_clean_pitcher:{port}:final_approach:pick_pitcher_3",
            "three_group_espresso",
            "pick_pitcher_3",
        ):
            return False

    # OLD live call kept for rollback:
    # if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
    #     return False
    if not _run_cached_machine_approach(
        f"angled_return_clean_pitcher:{port}:final_approach:pick_pitcher_2",
        "three_group_espresso",
        "pick_pitcher_2",
    ):
        return False

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False

    return True

def angled_unmount_single(**params) -> bool:
    params["port"] = "port_3"
    return angled_unmount(**params)

def angled_unmount_double(**params) -> bool:
    params["port"] = "angled_portafilter_1"
    return angled_unmount(**params)

def angled_mount_single(**params) -> bool:
    params["port"] = "port_3"
    return angled_mount(**params)

def angled_mount_double(**params) -> bool:
    params["port"] = "angled_portafilter_1"
    return angled_mount(**params)

def angled_single_grab_espresso_pitcher(**params) -> bool:
    params["port"] = "port_2"
    return angled_grab_espresso_pitcher(**params)

def angled_double_grab_espresso_pitcher(**params) -> bool:
    params["port"] = "port_1"
    return angled_grab_espresso_pitcher(**params)

def angled_single_pick_espresso_pitcher(**params) -> bool:
    params["port"] = "port_2"
    return angled_pick_espresso_pitcher(**params)

def angled_double_pick_espresso_pitcher(**params) -> bool:
    params["port"] = "port_1"
    return angled_pick_espresso_pitcher(**params)

def angled_single_pour_espresso_pitcher_cup_station(**params) -> bool:
    params["port"] = "port_3"
    return angled_pour_espresso_pitcher_cup_station(**params)

def angled_double_pour_espresso_pitcher_cup_station(**params) -> bool:
    params["port"] = "port_1"
    return angled_pour_espresso_pitcher_cup_station(**params)

def angled_single_return_espresso_pitcher(**params) -> bool:
    params["port"] = "port_3"
    return angled_return_espresso_pitcher(**params)

def angled_double_return_espresso_pitcher(**params) -> bool:
    params["port"] = "port_1"
    return angled_return_espresso_pitcher(**params)

def angled_single_return_cleaned_espresso_pitcher(**params) -> bool:
    params["port"] = "port_3"
    return angled_return_cleaned_espresso_pitcher(**params)

def angled_double_return_cleaned_espresso_pitcher(**params) -> bool:
    params["port"] = "angled_portafilter_1"
    return angled_return_cleaned_espresso_pitcher(**params)


"""
cleaning.py

Defines the cleaning routine for machine ports using parameterized configurations.
This module provides comprehensive portafilter cleaning functionality for the BARNS
coffee automation system, including hard brush and soft brush cleaning sequences
with precise positioning and error handling.
"""

# from oms_v1.manipulate_node import run_skill
# from oms_v1.params import (
#     ESPRESSO_GRINDER_HOME, CLEANING_PARAMS, DEFAULT_PORT,
# )

_hard_brush_clean_cache: Dict[str, List[Tuple[float, ...]]] = {}
_soft_brush_clean_cache: Dict[str, List[Tuple[float, ...]]] = {}


def invalidate_cleaning_cache():
    _hard_brush_clean_cache.clear()
    _soft_brush_clean_cache.clear()

def _is_valid_angles(angles: Any) -> bool:
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6

def _capture_and_cache_current_angles(cache_list: List[Tuple[float, ...]]) -> bool:
    angles = run_skill("current_angles")
    if not _is_valid_angles(angles):
        return False
    cache_list.append(tuple(angles))
    return True

def clean_portafilter(**params) -> bool:
    """
    Keep approach/mount live, cache only the post-mount motion sequence.
    """
    # from oms_v1.sequences.espresso import _normalize_espresso_shot

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    if shot_cfg and shot_cfg.get("angled"):
        return angled_clean_portafilter(**params)

    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else DEFAULT_PORT)

    def ok(r):
        return r not in (False, None)

    hard_cached = _hard_brush_clean_cache.get(port)
    soft_cached = _soft_brush_clean_cache.get(port)

    if not ok(run_skill("gotoJ_deg", -35.223076, -2.939468, -128.314575, -47.896400, -73.999352, 1.973845)):
        return False

    if not ok(run_skill("approach_machine", "portafilter_cleaner", "hard_brush")):
        return False
    if not ok(run_skill("gotoJ_deg", *CLEANING_PARAMS['hard_brush_adjust'])):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "hard_brush")):
        return False

    if hard_cached:
        if len(hard_cached) != 5:
            return False
        run_skill("sync")  
        if not ok(run_skill("gotoJ_deg", *hard_cached[0])):
            return False
        if not ok(run_skill("gotoJ_deg", *hard_cached[1])):
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *hard_cached[2])):
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *hard_cached[3])):
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *hard_cached[4])):
            return False
    else:
        hard_capture: List[Tuple[float, ...]] = []
        if not ok(run_skill("moveEE_movJ", 0, 0, 50, 0, 0, 0)):
            return False
        if not _capture_and_cache_current_angles(hard_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 0, 0, -30, -2.5, 0, 0)):
            return False
        if not _capture_and_cache_current_angles(hard_capture):
            return False
        if not ok(run_skill("moveEE_movJ", -7.5, 7.5, 0, 0, 0, 0)):
            return False
        if not _capture_and_cache_current_angles(hard_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 15.0, -15.0, 0, 0, 0, 0)):
            return False
        if not _capture_and_cache_current_angles(hard_capture):
            return False
        if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_hard'])):
            return False
        if not _capture_and_cache_current_angles(hard_capture):
            return False
        _hard_brush_clean_cache[port] = hard_capture

    if not ok(run_skill("approach_machine", "portafilter_cleaner", "soft_brush")):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "soft_brush")):
        return False

    if soft_cached:
        if len(soft_cached) != 6:
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *soft_cached[0])):
            return False
        if not ok(run_skill("gotoJ_deg", *soft_cached[1])):
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *soft_cached[2])):
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *soft_cached[3])):
            return False
        if not ok(run_skill("gotoJ_deg", *soft_cached[4])):
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *soft_cached[5])):
            return False
    else:
        soft_capture: List[Tuple[float, ...]] = []
        if not ok(run_skill("moveEE_movJ", 0, 0, 50, 0, 0, 0)):
            return False
        if not _capture_and_cache_current_angles(soft_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 0, 0, -35, -2.5, 0, 0)):
            return False
        if not _capture_and_cache_current_angles(soft_capture):
            return False
        if not ok(run_skill("moveEE_movJ", -2.5, 2.5, -5, 0, 0, 0)):
            return False
        if not _capture_and_cache_current_angles(soft_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 0.0, 0.0, 47.5, 0.0, 0.0, 0.0)):
            return False
        if not _capture_and_cache_current_angles(soft_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 0.0, 30.0, -17.5, 0.0, 0.0, 0.0)):
            return False
        if not _capture_and_cache_current_angles(soft_capture):
            return False
        if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_soft'])):
            return False
        if not _capture_and_cache_current_angles(soft_capture):
            return False
        _soft_brush_clean_cache[port] = soft_capture

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    return True

def clean_portafilter_single(**params) -> bool:
    params["port"] = "port_2"
    return clean_portafilter(**params)

def clean_portafilter_double(**params) -> bool:
    params["port"] = "port_1"
    return clean_portafilter(**params)

"""
angled_cleaning.py

Parallel definitions prefixed with angled_; same behavior as cleaning above until customized.
"""

angled__hard_brush_clean_cache: Dict[str, Any] = {}
angled__soft_brush_clean_cache: Dict[str, Any] = {}

def angled_invalidate_cleaning_cache():
    angled__hard_brush_clean_cache.clear()
    angled__soft_brush_clean_cache.clear()

def angled_cleaning_is_valid_angles(angles: Any) -> bool:
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6

def angled_cleaning_capture_current_angles(cache_list: List[Tuple[float, ...]]) -> bool:
    angles = run_skill("current_angles")
    if not angled_cleaning_is_valid_angles(angles):
        return False
    cache_list.append(tuple(angles))
    return True

def angled_clean_portafilter(**params) -> bool:
    """
    Keep approach/mount live, cache only the post-mount motion sequence.
    """
    # from oms_v1.sequences.espresso import angled__normalize_espresso_shot

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else DEFAULT_PORT)

    def ok(r):
        return r not in (False, None)

    hard_cached = angled__hard_brush_clean_cache.get(port)
    soft_cached = angled__soft_brush_clean_cache.get(port)

    if not ok(run_skill("gotoJ_deg", -35.223076, -2.939468, -128.314575, -47.896400, -73.999352, 1.973845)):
        return False

    if not ok(run_skill("approach_machine", "portafilter_cleaner", "angled_hard_brush")):
        return False
    if not ok(run_skill("gotoJ_deg", *CLEANING_PARAMS['hard_brush_adjust'])):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "angled_hard_brush")):
        return False

    if hard_cached:
        if len(hard_cached) != 4:
            return False
        run_skill("sync")  
        if not ok(run_skill("gotoJ_deg", *hard_cached[0])):
            return False
        if not ok(run_skill("gotoJ_deg", *hard_cached[1])):
            return False
        if not ok(run_skill("gotoJ_deg", *hard_cached[2])):
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *hard_cached[3])):
            return False
    else:
        hard_capture: List[Tuple[float, ...]] = []
        if not ok(run_skill("moveEE_movJ", 0, 0, 50, 0, 0, 0)):
            return False
        if not angled_cleaning_capture_current_angles(hard_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 0, 0, -37.5, -2.5, 0, 0)):
            return False
        if not angled_cleaning_capture_current_angles(hard_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 0, 0, -7.5, 0, 0, 0)):
            return False
        if not angled_cleaning_capture_current_angles(hard_capture):
            return False
        if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_hard'])):
            return False
        if not angled_cleaning_capture_current_angles(hard_capture):
            return False
        angled__hard_brush_clean_cache[port] = hard_capture

    if not ok(run_skill("approach_machine", "portafilter_cleaner", "angled_soft_brush")):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "angled_soft_brush")):
        return False

    if soft_cached:
        if len(soft_cached) != 4:
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *soft_cached[0])):
            return False
        if not ok(run_skill("gotoJ_deg", *soft_cached[1])):
            return False
        if not ok(run_skill("gotoJ_deg", *soft_cached[2])):
            return False
        run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *soft_cached[3])):
            return False
    else:
        soft_capture: List[Tuple[float, ...]] = []
        if not ok(run_skill("moveEE_movJ", 0, 0, 50, 0, 0, 0)):
            return False
        if not angled_cleaning_capture_current_angles(soft_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 0, 0, -37.5, -2.5, 0, 0)):
            return False
        if not angled_cleaning_capture_current_angles(soft_capture):
            return False
        if not ok(run_skill("moveEE_movJ", 0, 0, -7.5, 0, 0, 0)):
            return False
        if not angled_cleaning_capture_current_angles(soft_capture):
            return False
        if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_soft'])):
            return False
        if not angled_cleaning_capture_current_angles(soft_capture):
            return False
        angled__soft_brush_clean_cache[port] = soft_capture

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    return True

def angled_clean_portafilter_single(**params) -> bool:
    params["port"] = "angled_portafilter_1"
    return angled_clean_portafilter(**params)

def angled_clean_portafilter_double(**params) -> bool:
    params["port"] = "angled_portafilter_1"
    return angled_clean_portafilter(**params)

"""
milk_frothing.py

Defines the milk frothing sequences for coffee preparation automation.
This module provides comprehensive functions for handling milk frothing operations
in the BARNS coffee automation system, including frother positioning, mounting,
steam activation, milk pouring, and cleaning procedures.
"""

_log = logging.getLogger(__name__)
MAX_FROTHER_CALIBRATION_RETRIES = 5


_place_frother_milk_station_cache: Dict[str, Tuple[float, ...]] = {}
_pick_frother_milk_station_cache: Optional[Tuple[float, ...]] = None
_mount_frother_cache: Dict[str, Tuple[float, ...]] = {}
_pour_milk_cup_station_cache: Dict[str, Dict[str, Tuple[float, ...]]] = {}
_clean_milk_pitcher_cache: Optional[Tuple[float, ...]] = None
_return_frother_cache: Dict[str, Tuple[float, ...]] = {}
_get_frother_position_done: bool = False
_milk_frother_position_done: bool = False
_pick_frother_cache: Dict[str, Tuple[float, ...]] = {}
_go_home_with_ice_cache: Dict[str, Tuple[float, ...]] = {}
_place_plastic_cup_station_cache: Dict[str, Tuple[float, ...]] = {}
_pick_plastic_cup_station_cache: Dict[str, Tuple[float, ...]] = {}
_place_plastic_cup_sauces_cache: Optional[Tuple[float, ...]] = None
_pick_plastic_cup_sauces_cache: Dict[str, Tuple[float, ...]] = {}
_pick_plastic_cup_milk_cache: Dict[str, Tuple[float, ...]] = {}


def invalidate_milk_frothing_cache():
    global _pick_frother_milk_station_cache, _clean_milk_pitcher_cache
    global _get_frother_position_done, _milk_frother_position_done

    _place_frother_milk_station_cache.clear()
    _pick_frother_milk_station_cache = None
    _mount_frother_cache.clear()
    _pour_milk_cup_station_cache.clear()
    _clean_milk_pitcher_cache = None
    _return_frother_cache.clear()
    _pick_frother_cache.clear()
    _get_frother_position_done = False
    _milk_frother_position_done = False

def _is_valid_angles(angles: Any) -> bool:
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6

def _capture_current_angles() -> Optional[Tuple[float, ...]]:
    angles = run_skill("current_angles")
    if not _is_valid_angles(angles):
        return None
    return tuple(angles)

def _capture_current_position() -> Optional[Tuple[float, ...]]:
    position = run_skill("current_pose")
    if not _is_valid_position(position):
        return None
    return tuple(position)

def _is_valid_position(position: Any) -> bool:
    return bool(position) and isinstance(position, (tuple, list)) and len(position) == 6

def _cache_key_from_z_adjustment(z_adjustment: float) -> str:
    return f"{round(float(z_adjustment), 3):.3f}"

def get_frother_position(**params) -> bool:
    """
    Calibrate and record the milk frother position for future operations.
    After first successful calibration, skip re-reading machine position on later runs.
    """
    def ok(r):
        return r not in (False, None)

    global _get_frother_position_done, _milk_frother_position_done

    invalidate_plastic_cup_cache()
    invalidate_milk_frothing_cache()

    run_skill("set_speed_factor", 100)
    if not return_back_to_home():
        return False

    if not home(position="north_east"):
        return False

    if not _get_frother_position_done:
        steam_calibrated = False
        for attempt in range(1, MAX_FROTHER_CALIBRATION_RETRIES + 1):
            prep_ok = True
            cycles = 3
            for _ in range(cycles):
                time.sleep(CALIBRATION_SETTLE_TIME)
                if not ok(run_skill("move_to", "left_steam_wand", 0.29)):
                    prep_ok = False
                    break
            if not prep_ok:
                _log.warning(f"[CALIBRATION] left_steam_wand prep failed (attempt {attempt}/{MAX_FROTHER_CALIBRATION_RETRIES})")
                time.sleep(1.0)
                continue
            if ok(run_skill("get_machine_position", "left_steam_wand")):
                steam_calibrated = True
                break
            _log.warning(f"[CALIBRATION] left_steam_wand failed (attempt {attempt}/{MAX_FROTHER_CALIBRATION_RETRIES})")
            time.sleep(1.0)
        if not steam_calibrated:
            _log.error(f"[CALIBRATION] left_steam_wand failed after {MAX_FROTHER_CALIBRATION_RETRIES} attempts")
            return False
        _get_frother_position_done = True

    if not home(position="north_east"):
        return False
    
    if not ok(run_skill("gotoJ_deg", 27.975568,-34.583337,-124.996051,-68.785292,-70.522135,20.578546)):
        return False

    if not _milk_frother_position_done:
        frother_calibrated = False
        for attempt in range(1, MAX_FROTHER_CALIBRATION_RETRIES + 1):
            prep_ok = True
            cycles = 3
            for _ in range(cycles):
                time.sleep(CALIBRATION_SETTLE_TIME)
                if not ok(run_skill("move_to", "milk_frother_2", 0.29)):
                    prep_ok = False
                    break
            if not prep_ok:
                _log.warning(f"[CALIBRATION] milk_frother_2 prep failed (attempt {attempt}/{MAX_FROTHER_CALIBRATION_RETRIES})")
                time.sleep(1.0)
                continue
            if ok(run_skill("get_machine_position", "milk_frother_2")):
                frother_calibrated = True
                break
            _log.warning(f"[CALIBRATION] milk_frother_2 failed (attempt {attempt}/{MAX_FROTHER_CALIBRATION_RETRIES})")
            time.sleep(1.0)
        if not frother_calibrated:
            _log.error(f"[CALIBRATION] milk_frother_2 failed after {MAX_FROTHER_CALIBRATION_RETRIES} attempts")
            return False
        _milk_frother_position_done = True

    
    if not home(position="north"):
        return False

    return True

def pick_frother(**params) -> bool:
    """
    Pick up the milk frother for milk frothing operations.
    """

    def ok(r):
        return r not in (False, None)

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area'])):
        return False

    if not ok(run_skill("approach_machine", 'milk_frother_2', 'milk_frother_1')):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'])):
        return False
    if not ok(run_skill("mount_machine", 'milk_frother_2', 'milk_frother_1')):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure'])):
        return False
    return True

def place_frother_milk_station(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    cached_lift = _place_frother_milk_station_cache.get('lift_after_place')
    cached_nudge = _place_frother_milk_station_cache.get('final_nudge')

    if _is_valid_position(cached_lift):
        if not ok(run_skill("gotoEE", *cached_lift)):
            return False
    else:
        if not ok(run_skill("moveEE", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_place'])):
            return False
        lift_pose = _capture_current_position()
        if not _is_valid_position(lift_pose):
            return False
        _place_frother_milk_station_cache['lift_after_place'] = lift_pose

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_approach'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_final'])):
        return False

    if _is_valid_angles(cached_nudge):
        if not ok(run_skill("gotoJ_deg", *cached_nudge)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", 5, 0, 0, 0, 0, 0)):
            return False
        nudge_pose = _capture_current_angles()
        if not _is_valid_angles(nudge_pose):
            return False
        _place_frother_milk_station_cache['final_nudge'] = nudge_pose

    if not ok(run_skill("set_gripper_position", 50, MILK_FROTHER_GRIPPER_POSITIONS['place'])):
        return False
    return True

def pick_frother_milk_station(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    global _pick_frother_milk_station_cache

    if not ok(run_skill("set_gripper_position", 255, 255, 255)):
        return False

    if _is_valid_angles(_pick_frother_milk_station_cache):
        if not ok(run_skill("gotoJ_deg", *_pick_frother_milk_station_cache)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_pick'])):
            return False
        lift_pose = _capture_current_angles()
        if not _is_valid_angles(lift_pose):
            return False
        _pick_frother_milk_station_cache = lift_pose

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat2'])):
        return False
    return True

def mount_frother(**params) -> bool:
    """
    Mount the milk frother to the steam wand for frothing preparation.
    Applies a Z adjustment based on milk volume and cup size.
    """
    def ok(r):
        return r not in (False, None)

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['mount'])
    if not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth")):
        return False
    if not ok(run_skill("mount_machine", "left_steam_wand", "deep_froth")):
        return False

    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if not cup_size:
        cup_size = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if not cup_size:
        cup_size = 'default'

    milk_data = params.get('milk') or params.get('ingredients', {}).get('milk', {}) or {}
    try:
        volume_ml = float(next(iter(milk_data.values()), 150)) if isinstance(milk_data, dict) else float(milk_data or 150)
    except (TypeError, ValueError):
        volume_ml = 150.0

    factor = MILK_VOLUME_Z_ADJUSTMENT_FACTOR_BY_CUP_SIZE.get(
        cup_size,
        MILK_VOLUME_Z_ADJUSTMENT_FACTOR_BY_CUP_SIZE['default']
    )
    z_adjustment = factor * volume_ml

    if not ok(run_skill("moveEE_movJ", 12.5, 12.5, -z_adjustment, 7.5, 0, 0)):
        return False

    return True

def unmount_and_swirl_milk(**params) -> bool:
    """
    Swirl frothed milk in a circular motion for latte art preparation.
    """
    def ok(r):
        return r not in (False, None)

    time.sleep(MILK_FROTHING_DELAYS['swirl_delay'])

    if not ok(run_skill("gotoJ_deg", -42.145525,-86.603679,-32.009425,-57.409276,-74.353990,2.000000)): # if not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth")):
        return False

    # run_skill("sync")
    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['swirl'])

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['intermediate1'])):
        return False

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['swirl_pos'])):
        return False

    run_skill("sync")

    run_skill(
        "move_circle",
        MILK_SWIRL_CIRCLE_PARAMS['cycles'],
        MILK_SWIRL_CIRCLE_PARAMS['point1_offset'],
        MILK_SWIRL_CIRCLE_PARAMS['point2_offset'],
        MILK_SWIRL_CIRCLE_PARAMS['options']
    )

    return True

def pour_milk_cup_station(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    stage_cache = _pour_milk_cup_station_cache.setdefault(stage, {})
    stage_cfg = MILK_FROTHING_PARAMS['pouring'][f'stage{stage}']
    stage_offsets = MILK_POURING_OFFSETS[f'stage{stage}']

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
    if not ok(run_skill("gotoJ_deg", *stage_cfg['position'])):
        return False
    run_skill("sync")

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
    if not ok(run_skill("gotoJ_deg", *stage_cfg['adjust1'])):
        return False

    if _is_valid_angles(stage_cache.get('forward')):
        if not ok(run_skill("gotoJ_deg", *stage_cache['forward'])):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", *stage_offsets['move_forward'])):
            return False
        forward_pose = _capture_current_angles()
        if not _is_valid_angles(forward_pose):
            return False
        stage_cache['forward'] = forward_pose

    if _is_valid_angles(stage_cache.get('up')):
        if not ok(run_skill("gotoJ_deg", *stage_cache['up'])):
            return False
    else:
        if not ok(run_skill("moveEE_movJ", *stage_offsets['move_up'])):
            return False
        up_pose = _capture_current_angles()
        if not _is_valid_angles(up_pose):
            return False
        stage_cache['up'] = up_pose

    if not ok(run_skill("gotoJ_deg", *stage_cfg['position'])):
        return False

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    return True

def clean_milk_pitcher(**params) -> bool:
    def ok(r):
        return r not in (False, None)

    global _clean_milk_pitcher_cache

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose3'])):
        return False

    if _is_valid_angles(_clean_milk_pitcher_cache):
        if not ok(run_skill("gotoJ_deg", *_clean_milk_pitcher_cache)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['cleaning_motion'])):
            return False
        clean_pose = _capture_current_angles()
        if not _is_valid_angles(clean_pose):
            return False
        _clean_milk_pitcher_cache = clean_pose

    return True

def return_frother(**params) -> bool:
    """
    Return the frother to its original location using recorded approach/grab angles.
    """

    def ok(r):
        return r not in (False, None)

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return3'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return4'])):
        return False

    cached_lift = _place_frother_milk_station_cache.get('lift_after_place')
    if not _is_valid_position(cached_lift):
        return False
    if not ok(run_skill("gotoEE", *cached_lift)):
        return False
    if not ok(run_skill("moveEE", 0, 0, -145, 0, 0, 0)):
        return False
    time.sleep(0.5)
    if not ok(run_skill("set_gripper_position", 75, 165, 255)):
        return False
    time.sleep(0.5)
    if not ok(run_skill("moveEE", *MILK_FROTHER_MOVEMENT_OFFSETS['final_approach'])):
        return False
    if not ok(run_skill("approach_machine", "milk_frother_2", "milk_frother_1")):
        return False
    if not ok(home(position="north")):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open'])):
        return False
    return True

"""
plastic_cups.py

Defines the plastic cup handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for grabbing and placing plastic cups
in the BARNS coffee automation system, supporting multiple cup sizes for cold beverages
like slushes, iced drinks, and cold brews.
"""

def invalidate_plastic_cup_cache():
    _go_home_with_ice_cache.clear()
    _place_plastic_cup_station_cache.clear()
    _pick_plastic_cup_station_cache.clear()
    _place_plastic_cup_sauces_cache = None
    _pick_plastic_cup_sauces_cache.clear()
    _pick_plastic_cup_milk_cache.clear()

def _is_valid_angles(angles: Any) -> bool:
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6

def _capture_current_angles() -> Optional[Tuple[float, ...]]:
    angles = run_skill("current_angles")
    if not _is_valid_angles(angles):
        return None
    return tuple(angles)

def _normalize_plastic_cup_size(cups_dict: Any) -> str:
    # if not cups_dict:
    #     from oms_v1.params import DEFAULT_PLASTIC_CUP_SIZE
    #     return DEFAULT_PLASTIC_CUP_SIZE

    if isinstance(cups_dict, dict):
        cup_key = next(iter(cups_dict.keys()), None)
        if cup_key:
            cup_key_str = str(cup_key).upper()
            if 'CUP_' in cup_key_str:
                cup_code = cup_key_str.split('CUP_', 1)[1]
            else:
                cup_code = cup_key_str

            if cup_code and len(cup_code) >= 2:
                if cup_code[0] in ('H', 'C'):
                    size_num = cup_code[1:]
                else:
                    size_num = cup_code

                if size_num in ('7', '9', '12', '16'):
                    return f"{size_num}oz"

    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '':
        return result

    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result

    # from oms_v1.params import DEFAULT_PLASTIC_CUP_SIZE
    # return DEFAULT_PLASTIC_CUP_SIZE

def dispense_plastic_cup(**params) -> bool:
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_PLASTIC_CUP_SIZE)
    if not cup_size or not validate_cup_size(cup_size):
        return False

    CUP_CONFIG = {
        '16oz': {'home': 'west', 'coords': PLASTIC_CUPS_PARAMS['dispenser']['16oz_coords']},
        '12oz': {'home': 'west', 'coords': PLASTIC_CUPS_PARAMS['dispenser']['12oz_coords']},
        '9oz': {'home': 'south_west', 'coords': PLASTIC_CUPS_PARAMS['dispenser']['9oz_coords']},
        '7oz': {'home': 'south_west', 'coords': PLASTIC_CUPS_PARAMS['dispenser']['7oz_coords']},
    }
    if cup_size not in CUP_CONFIG:
        return False

    DISPENSE_PARAMS = {
        '16oz': {'gripper_pos': 110, 'do_index': 2},
        '12oz': {'gripper_pos': 135, 'do_index': 1},
        '9oz':  {'gripper_pos': 140, 'do_index': 3},
        '7oz':  {'gripper_pos': 130, 'do_index': 1},
    }

    config = CUP_CONFIG[cup_size]
    dp = DISPENSE_PARAMS[cup_size]
    attempt_count = 0
    while attempt_count < 15:
        home(position=config['home'])
        run_skill("set_gripper_position", 255, 0, 255)
        run_skill("gotoJ_deg", *config['coords'])
        run_skill("moveEE", 0.0, 328.0, 10.0, 0, 0, 0)
        run_skill("set_gripper_position", 255, dp['gripper_pos'], 255)
        run_skill("sync")
        run_skill("set_DO", dp['do_index'], 1)
        time.sleep(1.5)
        run_skill("set_DO", dp['do_index'], 0)
        run_skill("moveEE", 0, 0, -150, 0, 0, 0)
        run_skill("moveEE", 0, -328.0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", *config['coords'])
        home(position=config['home'])
        home(position="north")
        run_skill("sync")
        cup_detected = detect_cup_gripper()
        if cup_detected:
            break
        attempt_count += 1
        if attempt_count == 15:
            return False

    _set_cup_dispensed()
    return True

def go_to_ice(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    if not cup_size:
        return False
    if cup_size not in ('16oz', '12oz', '9oz', '7oz'):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position2'])):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    # run_skill("sync")
    return True

def go_home_with_ice(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_PLASTIC_CUP_SIZE)
    if not cup_size or not validate_cup_size(cup_size):
        return False

    cached_retreat = _go_home_with_ice_cache.get(cup_size)
    if _is_valid_angles(cached_retreat):
        if not ok(run_skill("gotoJ_deg", *cached_retreat)):
            return False
        # run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", -10, 0, 0, 0, 0, 0)):
            return False
        retreat_pose = _capture_current_angles()
        if not _is_valid_angles(retreat_pose):
            return False
        _go_home_with_ice_cache[cup_size] = retreat_pose

    gripper_position = {"7oz": 140, "9oz": 145, "12oz": 140, "16oz": 130}.get(cup_size)
    if gripper_position is None:
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_position)):
        return False
    # run_skill("sync")
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position3'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])):
        return False
    if not home(position="north"):
        return False
    _set_cup_dispensed()
    return True

def place_plastic_cup_station(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    if not cup_size or cup_size not in ('7oz', '9oz', '12oz', '16oz'):
        return False

    _check_and_clear_cup_dispensed()
    run_skill("set_speed_factor", SPEED_NORMAL)

    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False

    if stage == "1":
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_1'])
    elif stage == "2":
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_2'])
    elif stage == "3":
        home(position="south_east")
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_3'])
    elif stage == "4":
        home(position="south_east")
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_4'])
    else:
        return False

    if not ok(stage_result):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", 25, 100, 25)):
        return False
    if not ok(run_skill("set_gripper_position", 255, 0, 255)):
        return False
    run_skill("sync")
    run_skill("set_speed_factor", SPEED_FAST)

    cached_up = _place_plastic_cup_station_cache.get(stage)
    if _is_valid_angles(cached_up):
        if not ok(run_skill("gotoJ_deg", *cached_up)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['place_return_up'])):
            return False
        up_pose = _capture_current_angles()
        if not _is_valid_angles(up_pose):
            return False
        _place_plastic_cup_station_cache[stage] = up_pose

    if not home(position="east"):
        return False
    return True

def pick_plastic_cup_station(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    if not cup_size or cup_size not in ('7oz', '9oz', '12oz', '16oz'):
        return False

    stage_positions = {
        "1": PLASTIC_CUPS_PARAMS['staging']['pickup_1'],
        "2": PLASTIC_CUPS_PARAMS['staging']['pickup_2'],
        "3": PLASTIC_CUPS_PARAMS['staging']['pickup_3'],
        "4": PLASTIC_CUPS_PARAMS['staging']['pickup_4']
    }
    gripper_positions = {"7oz": 145, "9oz": 125, "12oz": 140, "16oz": 118}

    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return False

    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    run_skill("sync")

    cache_key = (stage, cup_size)
    cached_down = _pick_plastic_cup_station_cache.get(cache_key)
    if _is_valid_angles(cached_down):
        if not ok(run_skill("gotoJ_deg", *cached_down)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['pickup_down'])):
            return False
        down_pose = _capture_current_angles()
        if not _is_valid_angles(down_pose):
            return False
        _pick_plastic_cup_station_cache[cache_key] = down_pose

    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")

    if cup_position in (3, 4):
        if not home(position="south_east"):
            return False
    if not home(position="east"):
        return False
    if not home(position="north_east"):
        return False
    return True

def place_plastic_cup_sauces(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    if not cup_size or cup_size not in ("7oz", "9oz", "12oz", "16oz"):
        return False

    after_dispense = _check_and_clear_cup_dispensed()
    if not after_dispense:
        run_skill("set_speed_factor", SPEED_NORMAL)

    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position2'])):
        return False
    # run_skill("sync")

    global _place_plastic_cup_sauces_cache
    if _is_valid_angles(_place_plastic_cup_sauces_cache):
        if not ok(run_skill("gotoJ_deg", *_place_plastic_cup_sauces_cache)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE", -5, 0, 0, 0, 0, 0)):
            return False
        nudge_pose = _capture_current_angles()
        if not _is_valid_angles(nudge_pose):
            return False
        _place_plastic_cup_sauces_cache = nudge_pose

    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    # run_skill("sync")
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return False
    return True

def pick_plastic_cup_sauces(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    if not cup_size or cup_size not in ("7oz", "9oz", "12oz", "16oz"):
        return False

    _check_and_clear_cup_dispensed()
    gripper_positions = {"7oz": 140, "9oz": 140, "12oz": 140, "16oz": 130}
    run_skill("set_speed_factor", SPEED_NORMAL)
    if not detect_cup_gripper():
        return False

    cached_lift = _pick_plastic_cup_sauces_cache.get(cup_size)
    if _is_valid_angles(cached_lift):
        if not ok(run_skill("gotoJ_deg", *cached_lift)):
            return False
        # run_skill("sync")
    else:
        if not ok(run_skill("moveEE", 0, 0, 1, 0, 0, 0)):
            return False
        lift_pose = _capture_current_angles()
        if not _is_valid_angles(lift_pose):
            return False
        _pick_plastic_cup_sauces_cache[cup_size] = lift_pose

    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
    # run_skill("sync")
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1'])):
        return False
    return True

def place_plastic_cup_milk(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    if not cup_size or cup_size not in ("7oz", "9oz", "12oz", "16oz"):
        return False

    after_dispense = _check_and_clear_cup_dispensed()
    if not after_dispense:
        run_skill("set_speed_factor", SPEED_NORMAL)

    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position2'])):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    # run_skill("sync")
    if not detect_cup_gripper():
        return False
    return True

def pick_plastic_cup_milk(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    if not cup_size or cup_size not in ("7oz", "9oz", "12oz", "16oz"):
        return False

    _check_and_clear_cup_dispensed()
    gripper_positions = {"7oz": 140, "9oz": 140, "12oz": 140, "16oz": 130}
    run_skill("set_speed_factor", SPEED_NORMAL)
    if not detect_cup_gripper():
        return False

    cached_lift = _pick_plastic_cup_milk_cache.get(cup_size)
    if _is_valid_angles(cached_lift):
        if not ok(run_skill("gotoJ_deg", *cached_lift)):
            return False
        # run_skill("sync")
    else:
        if not ok(run_skill("moveEE", -5, 0, 1, 0, 0, 0)):
            return False
        lift_pose = _capture_current_angles()
        if not _is_valid_angles(lift_pose):
            return False
        _pick_plastic_cup_milk_cache[cup_size] = lift_pose

    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
    # run_skill("sync")
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1'])):
        return False
    return True
 
"""
slush.py

Defines the slush handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for dispensing and placing slush drinks
in the BARNS coffee automation system, supporting multiple dispensers and staging areas
for frozen beverage preparation.
"""

def _normalize_slush_cup_size(cups_dict: Any) -> str:
    if not cups_dict:
        return DEFAULT_PLASTIC_CUP_SIZE
    if isinstance(cups_dict, dict):
        cup_key = next(iter(cups_dict.keys()), None)
        if cup_key:
            cup_key_str = str(cup_key).upper()
            if 'CUP_' in cup_key_str:
                cup_code = cup_key_str.split('CUP_', 1)[1]
            else:
                cup_code = cup_key_str
            if cup_code and len(cup_code) >= 2:
                if cup_code[0] in ('H', 'C'):
                    size_num = cup_code[1:]
                else:
                    size_num = cup_code
                if size_num in ('7', '9', '12', '16'):
                    return f"{size_num}oz"
    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '':
        return result
    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result
    return DEFAULT_PLASTIC_CUP_SIZE


def get_slush(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_slush_cup_size(cups_dict)
    dispenser = params.get("dispenser")
    if not dispenser:
        premixes = params.get("premixes", {})
        if premixes:
            premix_name = list(premixes.keys())[0] if premixes else ""
            dispenser = "2" if ("chocolate" in premix_name.lower() or "choco" in premix_name.lower()) else "1"
        else:
            dispenser = "1"
    if cup_size not in ("7oz", "9oz", "12oz", "16oz") or dispenser not in ("1", "2"):
        return False
    cup_code = f"cup_C{cup_size.replace('oz', '')}"
    if not dispense_plastic_cup(cups={cup_code: 1.0}):
        return False
    if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['intermediate'])):
        return False
    if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])):
        return False
    if dispenser == "2":
        dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['dispense'])
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['intermediate'])):
            return False
        dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['dispense'])
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
            return False
        run_skill("sync")
    if not ok(dispenser_result):
        return False
    return True


def place_slush(**params) -> bool:
    def ok(r):
        return r not in (False, None)
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_slush_cup_size(cups_dict)
    dispenser = params.get("dispenser")
    if not dispenser:
        premixes = params.get("premixes", {})
        if premixes:
            premix_name = list(premixes.keys())[0] if premixes else ""
            dispenser = "2" if ("chocolate" in premix_name.lower() or "choco" in premix_name.lower()) else "1"
        else:
            dispenser = "1"
    if cup_size not in ("7oz", "9oz", "12oz", "16oz") or dispenser not in ("1", "2"):
        return False
    run_skill("set_speed_factor", SPEED_NORMAL)
    gripper_positions = {"7oz": 140, "9oz": 140, "12oz": 140, "16oz": 130}
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, gripper_positions[cup_size])):
        return False
    run_skill("sync")
    if dispenser == "2":
        retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['retreat'])
    else:
        retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['retreat'])
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['intermediate'])):
            return False
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])):
            return False
    if not ok(retreat_result):
        return False
    if not home(position="north"):
        return False
    cup_code = f"cup_C{cup_size.replace('oz', '')}"
    if not place_plastic_cup_station(position={'cup_position': int(stage)}, cups={cup_code: 1.0}):
        return False
    return True

'''
automation calls
'''
def call_tamper(**params):
    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_tamper.py"
    ]

    if "calibration_ms" in params:
        command += ["--calibration_ms", str(params["calibration_ms"])]

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Tamper failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True
    
def call_coffee_machine(**params):
    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_coffee_machine.py"
    ]

    if "coffee_type" in params:
        command += ["--coffee_type", str(params["coffee_type"])]

    if "slot_number" in params:
        command += ["--slot_number", str(params["slot_number"])]

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Coffee failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True

def call_hot_water(**params):

    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_coffee_hot_water.py"
    ]

    if "calibration" in params:
        command += ["--calibration", str(params["calibration"])]

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Hot water failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True

def call_coffee_purge(**params):

    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_coffee_purge.py"
    ]

    if "slot_number" in params:
        command += ["--slot_number", str(params["slot_number"])]

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Purge failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True

def call_frother(**params):

    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_frother.py"
    ]

    # command type: init / froth / clean / status
    if "command" in params:
        command.append(params["command"])

    if "temp" in params:
        command += ["--temp", str(params["temp"])]

    if "init_time" in params:
        command += ["--init-time", str(params["init_time"])]

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Frother failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True

def call_grinder(**params):

    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_grinder.py"
    ]

    if "shots_number" in params:
        command += ["--shots_number", str(params["shots_number"])]

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Grinder failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True

def call_ice(**params):

    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_ice.py"
    ]

    if "weight" in params:
        command += ["--weight", str(params["weight"])]

    if "no_calibration" in params and params["no_calibration"]:
        command.append("--no-calibration")

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Ice failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True

def call_milk_syrup(**params):

    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_milks_syrups_v3.py"
    ]

    if "device" in params:
        command.append(params["device"])  # milk / syrup / rinse / purge

    if "motor" in params:
        command += ["--motor", str(params["motor"])]

    if "amount" in params:
        command += ["--amount", str(params["amount"])]

    if "seconds" in params:
        command += ["--seconds", str(params["seconds"])]

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Milk/Syrup failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True

def call_slush(**params):

    command = [
        "python3",
        "/home/adeel/Downloads/barns_low_level-main/Test_Machines/test_slush.py"
    ]

    if "type" in params:
        command += ["--type", params["type"]]

    if "weight" in params:
        command += ["--weight", str(params["weight"])]

    if "difference" in params:
        command += ["--difference", str(params["difference"])]

    result = subprocess.run(command, capture_output=True, text=True)

    if result.returncode != 0:
        print("Slush failed")
        print(result.stderr)
        return False

    print(result.stdout)
    return True
'''
RECIPES
'''
def espresso(**params):
    """
    Complete espresso preparation sequence using port_3.
    
    Workflow:
    3. Unmount portafilter from port_3
    2. Grind and tamp coffee
    3. Mount portafilter back to port_3
    4. Prepare paper cup at stage_3
    5. Pour espresso from port_3
    6. Return pitcher
    
    
    Returns:
        bool: True if espresso prepared successfully, False otherwise
    """
    print("☕ Starting espresso preparation sequence...")
    
    try:
        # Step 3: Unmount portafilter from port_3
        print("🔧 Unmounting portafilter from port_3...")
        if not unmount(port="port_3"):
            print("[ERROR] Failed to unmount portafilter from port_3")
            return False
        
        # Step 2: Grind coffee
        print("⚙️ Grinding coffee...")
        if not grinder(portafilter_tool="single_portafilter"):
            print("[ERROR] Failed to grind coffee")
            return False
        
        # Step 3: Tamp coffee
        print("🫸 Tamping coffee...")
        if not tamper(portafilter_tool="single_portafilter"):
            print("[ERROR] Failed to tamp coffee")
            return False
        
        # Step 4: Mount portafilter back to port_3
        print("🔧 Mounting portafilter to port_3...")
        if not mount(port="port_3"):
            print("[ERROR] Failed to mount portafilter to port_3")
            return False
        
        # Step 5: Grab paper cup
        print("🥤 Grabbing 7oz paper cup...")
        if not grab_paper_cup(size="7oz"):
            print("[ERROR] Failed to grab paper cup")
            return False
        
        # Step 6: Place cup at stage_3
        print("📍 Placing cup at stage_3...")
        if not place_paper_cup(position={'cup_position': 3}):
            print("[ERROR] Failed to place paper cup")
            return False
        
        # Step 7: Pick espresso pitcher for port_3
        print("🥛 Picking espresso pitcher for port_3...")
        if not pick_espresso_pitcher(port="port_3"):
            print("[ERROR] Failed to pick espresso pitcher")
            return False
        
        # Step 8: Pour espresso at stage_3
        print("☕ Pouring espresso at stage_3...")
        if not pour_espresso_pitcher_cup_station(position={'cup_position': 3}):
            print("[ERROR] Failed to pour espresso")
            return False
        
        # Step 9: Return espresso pitcher
        print("🔄 Returning espresso pitcher for port_3...")
        if not return_espresso_pitcher(port="port_3"):
            print("[ERROR] Failed to return espresso pitcher")
            return False
        
        unmount(port="port_3")
        
        # Step 30: Clean portafilter
        print("🧹 Cleaning portafilter...")
        if not clean_portafilter(port="port_3"):
            print("[ERROR] Failed to clean portafilter")
            return False
        
        # Step 33: Mount portafilter
        print("🔧 Mounting portafilter...")
        if not mount(port="port_3"):
            print("[ERROR] Failed to mount portafilter")
            return False
        
        print("✅ Espresso preparation completed successfully!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Espresso preparation failed with exception: {e}")
        return False

def espresso_angled(**params):
    """
    Angled toolhead espresso: machine port key angled_portafilter_1, tool double_portafilter_angled,
    then paper cup arm1 7oz at stage 1, standard pitcher on port_1, pour and return at stage 1.

    Steps: angled_unmount, clean_portafilter, angled_grinder, return_cleaned_espresso_pitcher,
    angled_tamper, angled_mount, dispense_paper_arm1_cup_station, grab/pick/pour/return pitcher.
    """
    angled_port = "angled_portafilter_1"
    tool = "double_portafilter_angled"
    pitcher_port = "port_1"
    cup_stage = 1

    print("Starting espresso_angled sequence...")

    try:
        print("[1/11] angled_unmount")
        if not angled_unmount(port=angled_port):
            print("[ERROR] angled_unmount failed")
            return False

        print("[2/11] clean_portafilter")
        if not clean_portafilter(port=angled_port):
            print("[ERROR] clean_portafilter failed")
            return False

        # print("[3/11] angled_grinder")
        # if not angled_grinder(port=angled_port, portafilter_tool=tool):
        #     print("[ERROR] angled_grinder failed")
        #     return False

        # print("[4/11] return_cleaned_espresso_pitcher")
        # if not return_cleaned_espresso_pitcher(port=pitcher_port):
        #     print("[ERROR] return_cleaned_espresso_pitcher failed")
        #     return False

        # print("[5/11] angled_tamper")
        # if not angled_tamper(portafilter_tool=tool):
        #     print("[ERROR] angled_tamper failed")
        #     return False

        print("[6/11] angled_mount")
        if not angled_mount(port=angled_port):
            print("[ERROR] angled_mount failed")
            return False

        # print("[7/11] dispense_paper_arm1_cup_station 7oz stage 1")
        # if not dispense_paper_arm1_cup_station(size="7oz", position={'cup_position': cup_stage}):
        #     print("[ERROR] dispense_paper_arm1_cup_station failed")
        #     return False

        # print("[8/11] grab_espresso_pitcher")
        # if not grab_espresso_pitcher(port=pitcher_port):
        #     print("[ERROR] grab_espresso_pitcher failed")
        #     return False

        # print("[9/11] pick_espresso_pitcher")
        # if not pick_espresso_pitcher(port=pitcher_port):
        #     print("[ERROR] pick_espresso_pitcher failed")
        #     return False

        # print("[10/11] pour_espresso_pitcher_cup_station stage 1")
        # if not pour_espresso_pitcher_cup_station(position={'cup_position': cup_stage}):
        #     print("[ERROR] pour_espresso_pitcher_cup_station failed")
        #     return False

        # print("[11/11] return_espresso_pitcher")
        # if not return_espresso_pitcher(port=pitcher_port):
        #     print("[ERROR] return_espresso_pitcher failed")
        #     return False

        print("espresso_angled completed successfully.")
        return True

    except Exception as e:
        print(f"[ERROR] espresso_angled failed: {e}")
        return False


def americano(**params):
    """
    Complete americano preparation sequence using port_1.
    
    Workflow:
    1. Unmount portafilter from port_1
    2. Grind and tamp coffee
    3. Mount portafilter back to port_1
    4. Prepare paper cup at stage_1
    5. Get hot water and prepare
    6. Pour espresso from port_1
    7. Return pitcher
    
    Returns:
        bool: True if americano prepared successfully, False otherwise
    """
    print("☕ Starting americano preparation sequence...")
    
    try:
        # Step 1: Unmount portafilter from port_1
        print("🔧 Unmounting portafilter from port_1...")
        if not unmount(port="port_1"):
            print("[ERROR] Failed to unmount portafilter from port_1")
            return False
        
        # Step 2: Grind coffee
        print("⚙️ Grinding coffee...")
        if not grinder(portafilter_tool="double_portafilter"):
            print("[ERROR] Failed to grind coffee")
            return False
        
        # Step 3: Tamp coffee
        print("🫸 Tamping coffee...")
        if not tamper(portafilter_tool="double_portafilter"):
            print("[ERROR] Failed to tamp coffee")
            return False
        
        # Step 4: Mount portafilter back to port_1
        print("🔧 Mounting portafilter to port_1...")
        if not mount(port="port_1"):
            print("[ERROR] Failed to mount portafilter to port_1")
            return False
        
        # Step 5: Grab paper cup
        print("🥤 Grabbing 7oz paper cup...")
        if not grab_paper_cup(size="7oz"):
            print("[ERROR] Failed to grab paper cup")
            return False
        
        # Step 6: Place cup at stage_1
        print("📍 Placing cup at stage_1...")
        if not place_paper_cup(position={'cup_position': 1}):
            print("[ERROR] Failed to place paper cup")
            return False
        
        # Step 7: Pick espresso pitcher for port_1
        print("🥛 Picking espresso pitcher for port_1...")
        if not pick_espresso_pitcher(port="port_1"):
            print("[ERROR] Failed to pick espresso pitcher")
            return False
        
        # Step 8: Get hot water
        print("♨️ Getting hot water...")
        if not get_hot_water():
            print("[ERROR] Failed to get hot water")
            return False
        
        # Step 9: Prepare with hot water
        print("♨️ Preparing with hot water...")
        if not with_hot_water():
            print("[ERROR] Failed to prepare with hot water")
            return False
        
        # Step 10: Pour espresso at stage_1
        print("☕ Pouring americano at stage_1...")
        if not pour_espresso_pitcher_cup_station(position={'cup_position': 1}):
            print("[ERROR] Failed to pour americano")
            return False
        
        # Step 11: Return espresso pitcher
        print("🔄 Returning espresso pitcher for port_1...")
        if not return_espresso_pitcher(port="port_1"):
            print("[ERROR] Failed to return espresso pitcher")
            return False
        
        print("✅ Americano preparation completed successfully!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Americano preparation failed with exception: {e}")
        return False

def multi_espresso(**params):
    try:
        positions = [1, 2, 3, 4]
        
        for position in positions:
            if not unmount(port="port_1"):
                return False
            if not grinder(portafilter_tool="double_portafilter"):
                return False
            if not tamper(portafilter_tool="double_portafilter"):
                return False
            if not mount(port="port_1"):
                return False
            if not grab_paper_cup(size="7oz"):
                return False
            if not place_paper_cup(position={'cup_position': position}):
                return False
            if not pick_espresso_pitcher(port="port_1"):
                return False
            if not pour_espresso_pitcher_cup_station(position={'cup_position': position}):
                return False
            if not return_espresso_pitcher(port="port_1"):
                return False
        
        return True
        
    except Exception as e:
        return False

def milk_frothing(**params):
    import time
    import json
    import logging
    import traceback
    from datetime import datetime

    # -----------------------------
    # Config
    # -----------------------------
    run_id = params.get("run_id", f"milk_frothing_{int(time.time())}")
    cup_size = params.get("cup_size", "9oz")
    cups_per_batch = params.get("cups_per_batch", 4)
    froth_temp = params.get("froth_temp", 60)

    # Safer than the original while True by default.
    # Set repeat_forever=True to match the original infinite loop behavior.
    repeat_forever = params.get("repeat_forever", False)
    max_batches = params.get("max_batches", 1)

    interactive = params.get("interactive", True)
    dry_run = params.get("dry_run", False)
    fun_logs = params.get("fun_logs", True)
    log_file = params.get("log_file", f"{run_id}.log")

    milk_dose_1 = params.get("milk_dose_1", 20)
    milk_dose_2 = params.get("milk_dose_2", 20)
    pitcher_milk_amount = params.get("pitcher_milk_amount", 150)

    total_milk_per_cup = milk_dose_1 + milk_dose_2 + pitcher_milk_amount

    # -----------------------------
    # Logger setup
    # -----------------------------
    logger = logging.getLogger(run_id)
    logger.setLevel(logging.INFO)
    logger.propagate = False

    if not logger.handlers:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
        logger.addHandler(file_handler)

    # -----------------------------
    # Local state
    # -----------------------------
    events = []
    batch_summaries = []
    event_counter = 0
    last_step = None

    # -----------------------------
    # Internal helpers
    # -----------------------------
    def now_iso():
        return datetime.now().isoformat(timespec="seconds")

    def robot_say(message, mood="robot"):
        if not fun_logs:
            return

        icons = {
            "robot": "🤖",
            "start": "🚀",
            "cup": "🥤",
            "milk": "🥛",
            "froth": "🫧",
            "pour": "☕",
            "clean": "🧼",
            "done": "✅",
            "warning": "⚠️",
            "error": "💥",
            "sync": "🔄",
            "stats": "📊",
        }

        print(f"{icons.get(mood, '🤖')} {message}")

    def log_event(event_name, level="info", **data):
        nonlocal event_counter

        event_counter += 1

        event = {
            "event_index": event_counter,
            "timestamp": now_iso(),
            "run_id": run_id,
            "event": event_name,
            **data,
        }

        events.append(event)

        line = json.dumps(event, default=str)

        if level == "warning":
            logger.warning(line)
        elif level == "error":
            logger.error(line)
        else:
            logger.info(line)

        return event

    def validate_config():
        if not isinstance(cups_per_batch, int) or cups_per_batch <= 0:
            raise ValueError(f"cups_per_batch must be a positive integer. Got: {cups_per_batch}")

        if not isinstance(froth_temp, (int, float)):
            raise ValueError(f"froth_temp must be numeric. Got: {froth_temp}")

        if froth_temp < 40:
            log_event(
                "low_froth_temperature_warning",
                level="warning",
                froth_temp=froth_temp,
                message="Froth temperature is unusually low.",
            )
            robot_say(f"Froth temp looks low: {froth_temp}°C", "warning")

        if froth_temp > 75:
            log_event(
                "high_froth_temperature_warning",
                level="warning",
                froth_temp=froth_temp,
                message="Froth temperature is unusually high.",
            )
            robot_say(f"Froth temp looks high: {froth_temp}°C", "warning")

        if total_milk_per_cup <= 0:
            raise ValueError(f"Total milk amount must be positive. Got: {total_milk_per_cup}")

        if dry_run:
            robot_say("Dry run enabled: robot actions will be logged but not executed.", "warning")

    def timed_step(step_name, fn, *args, **kwargs):
        nonlocal last_step

        last_step = step_name
        start = time.perf_counter()

        log_event(
            "step_started",
            step=step_name,
            args=[str(arg) for arg in args],
            kwargs=kwargs,
            dry_run=dry_run,
        )

        robot_say(f"Starting: {step_name}", "robot")

        try:
            if dry_run:
                result = None
                time.sleep(params.get("dry_run_step_delay", 0.05))
            else:
                result = fn(*args, **kwargs)

            duration_sec = time.perf_counter() - start

            log_event(
                "step_completed",
                step=step_name,
                duration_sec=round(duration_sec, 3),
            )

            return result

        except Exception as exc:
            duration_sec = time.perf_counter() - start

            log_event(
                "step_failed",
                level="error",
                step=step_name,
                duration_sec=round(duration_sec, 3),
                error_type=type(exc).__name__,
                error=str(exc),
                traceback=traceback.format_exc(),
            )

            robot_say(f"Failed at step: {step_name}", "error")
            raise

    def print_cup_summary(cup_summary):
        robot_say(
            (
                f"Cup {cup_summary['cup_number']}/{cups_per_batch} finished "
                f"in {cup_summary['duration_sec']}s "
                f"| position={cup_summary['cup_position']} "
                f"| milk={cup_summary['milk_total_amount']} "
                f"| temp={cup_summary['froth_temp']}°C"
            ),
            "done",
        )

    def print_batch_summary(batch_summary):
        robot_say("Batch summary", "stats")

        print("\n========== MILK FROTHING BATCH SUMMARY ==========")
        print(f"Run ID: {run_id}")
        print(f"Log file: {log_file}")
        print(f"Cup size: {cup_size}")
        print(f"Cups completed: {len(batch_summary['cups'])}")
        print(f"Batch duration: {batch_summary['duration_sec']}s")
        print(f"Total milk used: {batch_summary['total_milk_used']}")

        print("\nPer cup:")
        for cup in batch_summary["cups"]:
            print(
                f"  Cup {cup['cup_number']} "
                f"| position {cup['cup_position']} "
                f"| {cup['duration_sec']}s "
                f"| milk {cup['milk_total_amount']} "
                f"| temp {cup['froth_temp']}°C"
            )

        print("=================================================\n")

    # -----------------------------
    # Main function execution
    # -----------------------------
    validate_config()

    function_start = time.perf_counter()

    log_event(
        "function_started",
        cup_size=cup_size,
        cups_per_batch=cups_per_batch,
        froth_temp=froth_temp,
        repeat_forever=repeat_forever,
        max_batches=max_batches,
        interactive=interactive,
        dry_run=dry_run,
        milk_dose_1=milk_dose_1,
        milk_dose_2=milk_dose_2,
        pitcher_milk_amount=pitcher_milk_amount,
        total_milk_per_cup=total_milk_per_cup,
    )

    robot_say(f"Milk frothing mission started: {run_id}", "start")

    try:
        timed_step("get frother position", get_frother_position)

        batch_number = 0

        while repeat_forever or batch_number < max_batches:
            batch_number += 1
            batch_start = time.perf_counter()
            cup_summaries = []

            log_event(
                "batch_started",
                batch_number=batch_number,
                cups_per_batch=cups_per_batch,
            )

            robot_say(f"Starting batch {batch_number}", "start")

            for i in range(cups_per_batch):
                cup_number = i + 1
                cup_position = i + 1
                cup_start = time.perf_counter()

                print(f"i: {i}")

                log_event(
                    "cup_started",
                    batch_number=batch_number,
                    cup_number=cup_number,
                    cup_position=cup_position,
                    cup_size=cup_size,
                )

                robot_say(
                    f"Preparing cup {cup_number}/{cups_per_batch} at position {cup_position}",
                    "cup",
                )

                timed_step(
                    "grab paper cup from arm2 cup station",
                    grab_paper_arm2_cup_station,
                    size=cup_size,
                )

                timed_step(
                    "place paper cup at milk station",
                    place_paper_cup_milk,
                    cup_size=cup_size,
                )

                timed_step(
                    "milk dose 1",
                    call_milk_syrup,
                    device="milk",
                    motor=20,
                    amount=milk_dose_1,
                )

                timed_step(
                    "pick paper cup from milk station",
                    pick_paper_cup_milk,
                    cup_size=cup_size,
                )

                timed_step(
                    "place paper cup at sauces station",
                    place_paper_cup_sauces,
                    cup_size=cup_size,
                )

                timed_step(
                    "sync",
                    run_skill,
                    "sync",
                )

                timed_step(
                    "milk dose 2",
                    call_milk_syrup,
                    device="milk",
                    motor=23,
                    amount=milk_dose_2,
                )

                timed_step(
                    "pick paper cup from sauces station",
                    pick_paper_cup_sauces,
                    cup_size=cup_size,
                )

                timed_step(
                    "place paper cup at arm2 cup station",
                    place_paper__arm2_cup_station,
                    position={"cup_position": cup_position},
                )

                timed_step(
                    "pick frother",
                    pick_frother,
                )

                timed_step(
                    "place frother at milk station",
                    place_frother_milk_station,
                )

                timed_step(
                    "fill milk pitcher",
                    call_milk_syrup,
                    device="milk",
                    motor=20,
                    amount=pitcher_milk_amount,
                )

                timed_step(
                    "pick frother from milk station",
                    pick_frother_milk_station,
                )

                timed_step(
                    "mount frother",
                    mount_frother,
                )

                timed_step(
                    "froth milk",
                    call_frother,
                    command="froth",
                    temp=froth_temp,
                )

                timed_step(
                    "unmount and swirl milk",
                    unmount_and_swirl_milk,
                )

                timed_step(
                    "pour milk into cup at cup station",
                    pour_milk_cup_station,
                    position={"cup_position": cup_position},
                )

                timed_step(
                    "clean milk pitcher",
                    clean_milk_pitcher,
                )

                timed_step(
                    "return frother",
                    return_frother,
                )

                cup_duration = time.perf_counter() - cup_start

                cup_summary = {
                    "batch_number": batch_number,
                    "cup_number": cup_number,
                    "cup_position": cup_position,
                    "cup_size": cup_size,
                    "duration_sec": round(cup_duration, 3),
                    "milk_dose_1": milk_dose_1,
                    "milk_dose_2": milk_dose_2,
                    "pitcher_milk_amount": pitcher_milk_amount,
                    "milk_total_amount": total_milk_per_cup,
                    "froth_temp": froth_temp,
                }

                cup_summaries.append(cup_summary)

                log_event(
                    "cup_completed",
                    **cup_summary,
                )

                print_cup_summary(cup_summary)

                if interactive:
                    input(f"Finished iteration {cup_number}/{cups_per_batch}. Press Enter to continue...")

            batch_duration = time.perf_counter() - batch_start

            batch_summary = {
                "run_id": run_id,
                "batch_number": batch_number,
                "duration_sec": round(batch_duration, 3),
                "cups": cup_summaries,
                "cups_completed": len(cup_summaries),
                "total_milk_used": total_milk_per_cup * len(cup_summaries),
                "average_cup_duration_sec": round(
                    sum(cup["duration_sec"] for cup in cup_summaries) / len(cup_summaries),
                    3,
                ),
                "fastest_cup_sec": min(cup["duration_sec"] for cup in cup_summaries),
                "slowest_cup_sec": max(cup["duration_sec"] for cup in cup_summaries),
            }

            batch_summaries.append(batch_summary)

            log_event(
                "batch_completed",
                **batch_summary,
            )

            print_batch_summary(batch_summary)

        total_duration = time.perf_counter() - function_start

        final_summary = {
            "run_id": run_id,
            "status": "completed",
            "duration_sec": round(total_duration, 3),
            "batches_completed": len(batch_summaries),
            "cups_completed": sum(batch["cups_completed"] for batch in batch_summaries),
            "total_milk_used": sum(batch["total_milk_used"] for batch in batch_summaries),
            "log_file": log_file,
            "events_recorded": len(events),
            "batches": batch_summaries,
        }

        log_event(
            "function_completed",
            **final_summary,
        )

        robot_say("Milk frothing mission complete.", "done")

        return final_summary

    except KeyboardInterrupt:
        total_duration = time.perf_counter() - function_start

        interrupted_summary = {
            "run_id": run_id,
            "status": "interrupted",
            "duration_sec": round(total_duration, 3),
            "last_step": last_step,
            "log_file": log_file,
            "events_recorded": len(events),
            "batches": batch_summaries,
        }

        log_event(
            "function_interrupted",
            level="warning",
            **interrupted_summary,
        )

        robot_say("Milk frothing mission interrupted by operator.", "warning")

        return interrupted_summary

    except Exception as exc:
        total_duration = time.perf_counter() - function_start

        failure_summary = {
            "run_id": run_id,
            "status": "failed",
            "duration_sec": round(total_duration, 3),
            "last_step": last_step,
            "error_type": type(exc).__name__,
            "error": str(exc),
            "log_file": log_file,
            "events_recorded": len(events),
            "batches": batch_summaries,
        }

        log_event(
            "function_failed",
            level="error",
            **failure_summary,
        )

        robot_say(f"Milk frothing mission failed at: {last_step}", "error")

        raise
    # input()
    # pick_frother()
    # place_frother_milk_station()
    # call_milk_syrup(device="milk", motor=20, amount=200)
    # pick_frother_milk_station()
    # mount_frother()
    # input()
    # call_frother(command="froth", temp=60)
    # input()
    # unmount_and_swirl_milk()
    # input()
    # pour_milk_cup_station(position={'cup_position': 1})
    # clean_milk_pitcher()
    # return_frother()

def milk_1(**params):
    """Pour milk at cup position 1 - simplified raw commands"""
    run_skill("gotoJ_deg", -96.038133,-16.779600,-104.255018,-62.483484,-81.154254,-37.125368)  # stage1 position
    run_skill("sync")
    run_skill("set_speed_factor", 9)  # precise pouring speed
    run_skill("sync")
    run_skill("gotoJ_deg", -80.742955,-20.628923,-107.592269,-56.447117,-75.887376,-108.203675)  # adjust1
    run_skill("moveEE_movJ", 20, 0, 0, 0, 0, 0)  # final pouring motion
    run_skill("sync")
    time.sleep(3.0)  # pour completion time
    run_skill("moveEE_movJ", 0, 0, 100, 0, 0, 0)  # lift up
    run_skill("gotoJ_deg", -96.038133,-16.779600,-104.255018,-62.483484,-81.154254,-37.125368)  # return to position
    run_skill("set_speed_factor", 100)  # restore normal speed
    return True

def milk_2(**params):
    """Pour milk at cup position 2 - simplified raw commands"""
    run_skill("set_speed_factor", 20)  # initial speed
    run_skill("gotoJ_deg", -110.625841,-24.341315,-92.985758,-66.165623,-95.718015,-38.024363)  # stage2 position
    run_skill("sync")
    run_skill("set_speed_factor", 9)  # precise pouring speed
    run_skill("sync")
    run_skill("gotoJ_deg", -99.369913,-24.209976,-97.719323,-62.612256,-94.456415,-109.700141)  # adjust1
    run_skill("moveEE_movJ", 25, 0, 0, 0, 0, 0)  # final pouring motion
    run_skill("sync")
    time.sleep(3.0)  # pour completion time
    run_skill("moveEE_movJ", 0, 0, 100, 0, 0, 0)  # lift up
    run_skill("gotoJ_deg", -110.625841,-24.341315,-92.985758,-66.165623,-95.718015,-38.024363)  # return to position
    run_skill("set_speed_factor", 100)  # restore normal speed
    return True

def milk_3(**params):
    """Pour milk at cup position 3 - simplified raw commands"""
    run_skill("set_speed_factor", 20)  # initial speed
    run_skill("gotoJ_deg", -122.820854,-36.136456,-73.316114,-74.204156,-107.900885,-38.813036)  # stage3 position
    run_skill("sync")
    run_skill("set_speed_factor", 9)  # precise pouring speed
    run_skill("sync")
    run_skill("gotoJ_deg", -113.296795,-33.403078,-82.373956,-68.997752,-108.344130,-110.861340)  # adjust1
    run_skill("moveEE_movJ", 25, 0, 0, 0, 0, 0)  # final pouring motion
    run_skill("sync")
    time.sleep(3.0)  # pour completion time
    run_skill("moveEE_movJ", 0, 0, 100, 0, 0, 0)  # lift up
    run_skill("gotoJ_deg", -122.820854,-36.136456,-73.316114,-74.204156,-107.900885,-38.813036)  # return to position
    run_skill("set_speed_factor", 100)  # restore normal speed
    return True

def milk_4(**params):
    """Pour milk at cup position 4 - simplified raw commands"""
    run_skill("set_speed_factor", 20)  # initial speed
    run_skill("gotoJ_deg", -132.368953,-52.469645,-42.776132,-88.685925,-117.453260,-39.514638)  # stage4 position
    run_skill("sync")
    run_skill("set_speed_factor", 9)  # precise pouring speed
    run_skill("sync")
    run_skill("gotoJ_deg", -125.360518,-47.083181,-57.256980,-80.924584,-120.379033,-112.033018)  # adjust1
    run_skill("moveEE_movJ", 25, 0, 0, 0, 0, 0)  # final pouring motion
    run_skill("sync")
    time.sleep(3.0)  # pour completion time
    run_skill("moveEE_movJ", 0, 0, 100, 0, 0, 0)  # lift up
    run_skill("gotoJ_deg", -132.368953,-52.469645,-42.776132,-88.685925,-117.453260,-39.514638)  # return to position
    run_skill("set_speed_factor", 100)  # restore normal speed
    return True

def slushie(**params):
    """
    Complete slushie preparation sequence.
    
    Workflow:
    1. Dispense plastic cup (if needed, handled by get_slush internally)
    2. Get slush from dispenser
    3. Place slush cup at designated position
    
    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C16': 1.0} (defaults to 16oz)
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        dispenser (str): Dispenser number ('1' or '2'), required parameter
        
    Returns:
        bool: True if slushie preparation completed successfully, False otherwise
        
    Example:
        params = {
            'cups': {'cup_C16': 1.0},
            'position': {'cup_position': 1.0},
            'dispenser': '1'
        }
        success = slushie(**params)
    """
    cup_position = 2
    dispenser = "2"
    cup_size = "16oz"
    
    print(f"🧊 Starting slushie preparation: {cup_size}, Position {cup_position}, Dispenser {dispenser}")
    
    # Get slush from dispenser
    if not get_slush(position={'cup_position': cup_position}, dispenser=dispenser, cup_size=cup_size):
        print("[ERROR] Failed to get slush")
        return False
    
    # Place slush cup at position
    if not place_slush(position={'cup_position': cup_position}, dispenser=dispenser, cup_size=cup_size):
        print("[ERROR] Failed to place slush")
        return False
    
    print(f"✅ Slushie completed successfully at position {cup_position}")
    return True
"""
train.py
"""
def espresso_training(**params):
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.22)
    run_skill("get_machine_position", "three_group_espresso")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("gotoJ_deg", -28.755102,-16.240370,-145.875793,-15.083625,-114.523071,0.660176)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", -17.232647,-27.268740,-120.223114,-31.566103,-104.572914,-0.586924)#run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", -28.755102,-16.240370,-145.875793,-15.083625,-114.523071,0.660176)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    input()
    run_skill("gotoJ_deg", 24.911945,-21.074497,-135.128052,-22.276201,-61.762913,0)#run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    input()
    run_skill("gotoJ_deg", 78.049049,-11.560322,-133.106522,-29.895899,-7.475047,-5.520638)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", 59.782707,-31.320202,-112.962959,-35.808193,-20.976677,0.945422)#run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", 78.049049,-11.560322,-133.106522,-29.895899,-7.475047,-5.520638)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    input()
    run_skill("gotoJ_deg",16.182545,-45.977921,-119.918640,-12.260736,-73.420769,0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    input()
    run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("gotoJ_deg", -22.378635,-25.306602,-137.820709,-24.454021,-108.835793,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
    input()
    run_skill("gotoJ_deg", -14.879210,-48.115944,-114.257034,-13.916231,-101.903206,0.0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
    input()
    run_skill("gotoJ_deg", -22.378635,-25.306602,-137.820709,-24.454021,-108.835793,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
    run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("gotoJ_deg",72.861008,-37.369293,-141.719101,8.783054,-15.383393,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
    input()
    run_skill("gotoJ_deg",48.463070,-49.133530,-111.629639,-15.658930,-40.472858,0.0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3", True)
    input()
    run_skill("gotoJ_deg",72.861008,-37.369293,-141.719101,8.783054,-15.383393,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
    run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "espresso_grinder", 0.26)
    run_skill("get_machine_position", "espresso_grinder")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    run_skill("gotoJ_deg",-45.427513,-58.889004,-102.710777,-19.257256,-101.516602,0.0)#approach the grinder#run_skill("approach_machine", "espresso_grinder", "grinder", True)
    input()
    run_skill("gotoJ_deg", -43.257053,-67.649300,-83.081116,-27.657471,-97.851700,0.0) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    input()
    run_skill("gotoJ_deg",-44.728165,-67.766884,-72.699600,-43.691830,-102.070465,0.0)#touch the button#run_skill("approach_machine", "espresso_grinder", "tamper", True)
    input()
    run_skill("gotoJ_deg", -43.257053,-67.649300,-83.081116,-27.657471,-97.851700,0.0) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    input()
    run_skill("gotoJ_deg", -43.257011,-71.458015,-81.109985,-25.820198,-97.854370,0.0) #in the tamper#run_skill("mount_machine", "espresso_grinder", "tamper", True)
    input()
    run_skill("gotoJ_deg", -43.257053,-67.649300,-83.081116,-27.657471,-97.851700,0.0) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg",-45.427513,-58.889004,-102.710777,-19.257256,-101.516602,0.0)#approach the grinder#run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "portafilter_cleaner", 0.26)
    run_skill("get_machine_position", "portafilter_cleaner")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
    run_skill("gotoJ_deg", -79.218399,-0.662163,-126.990761,-53.961704,-81.920143,-1.976189)# run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
    input()
    run_skill("moveEE_movJ", -88, 0, 0, 0, 0, -135)
    run_skill("gotoJ_deg", -95.707840,-18.450220,-129.333282,-34.286320,-96.467575,-179.530777)#run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
    input()
    run_skill("moveEE_movJ", 0, 0, 100, 0, 0, 0)
    run_skill("gotoJ_deg", -79.218399,-0.662163,-126.990761,-53.961708,-81.920143,-179.457260)# run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
    input()
    run_skill("gotoJ_deg", -79.220100,-15.084805,-135.632080,-30.894413,-81.913452,-179.481079)#run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
    input()
    run_skill("moveEE_movJ", 0, 0, 150, 0, 0, 0)
    run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)

def milk_training(**params):
    # home(position="north_east")
    # for i in range(5):
    #     time.sleep(1.0)
    #     run_skill("move_to", "left_steam_wand", 0.26)
    # run_skill("get_machine_position", "left_steam_wand")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    # input()
    # run_skill("gotoJ_deg", -45.731773,-48.647770,-90.804688,-52.454651,-90.259727,10.531549)
    # run_skill("gotoJ_deg",-47.825047,-86.633034,-28.265034,-67.004753,-96.852402,10.037631)#run_skill("approach_machine", "left_steam_wand", "deep_froth", True)
    # input()
    # run_skill("gotoJ_deg", -47.823650,-85.286758,-16.991077,-70.719292,-92.739357,16.378487)#run_skill("mount_machine", "left_steam_wand", "deep_froth", True)
    ##############################
    run_skill("gotoJ_deg",27.975568,-34.583337,-124.996051,-68.785292,-70.522135,20.578546)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "milk_frother_2", 0.26)
    run_skill("get_machine_position", "milk_frother_2")
    input()
    run_skill("gotoJ_deg",27.975568,-34.583337,-124.996051,-68.785292,-70.522135,20.578546)
    run_skill("sync")
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "milk_frother_1", 0.26)
    run_skill("sync")
    run_skill("approach_tool", 'milk_frother_1')
    input()
    run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'])    
    run_skill("sync")
    run_skill("grab_tool", 'milk_frother_1', 100, 100,-5,-10.5)
    input()
    run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure'])

def angled_espresso_training(**params):
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.22)
    run_skill("get_machine_position", "three_group_espresso")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("sync")
    run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_1", True)
    input()
    run_skill("grab_tool", "double_portafilter_angled")#run_skill("mount_machine", "three_group_espresso", "angled_portafilter_1", True)
    input()
    run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_1", True)
    run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    # input()
    # run_skill("gotoJ_deg", 35.770206,4.758220,-128.029175,-84.061310,-57.503407,15.573264)
    # run_skill("sync")
    # run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_2", True)
    # input()
    # run_skill("grab_tool", "double_portafilter_angled")#run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    # input()
    # run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_1", True)
    # run_skill("gotoJ_deg", 35.770206,4.758220,-128.029175,-84.061310,-57.503407,15.573264)
    # run_skill("gotoJ_deg", *ESPRESSO_HOME)
    # input()
    # run_skill("gotoJ_deg", 50.518208,-1.260620,-120.952957,-81.799118,-62.782921,11.558730)
    # run_skill("sync")
    # run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_1", True)
    # input()
    # run_skill("grab_tool", "double_portafilter_angled")#run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    # input()
    # run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_1", True)
    # run_skill("gotoJ_deg", 50.518208,-1.260620,-120.952957,-81.799118,-62.782921,11.558730)
    # run_skill("gotoJ_deg", *ESPRESSO_HOME)
    # for i in range(5):
    #     time.sleep(1.0)
    #     run_skill("move_to", "portafilter_cleaner", 0.26)
    # run_skill("get_machine_position", "portafilter_cleaner")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    # input()
    # run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    # run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
    # run_skill("gotoJ_deg", -79.218399,-0.662163,-126.990761,-53.961704,-81.920143,-1.976189)# run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
    # input()
    # run_skill("moveEE_movJ", -88, 0, 0, 0, 0, -135)
    # run_skill("gotoJ_deg", -95.707840,-18.450220,-129.333282,-34.286320,-96.467575,-179.530777)#run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
    # input()
    # run_skill("moveEE_movJ", 0, 0, 100, 0, 0, 0)
    # run_skill("gotoJ_deg", -79.218399,-0.662163,-126.990761,-53.961708,-81.920143,-179.457260)# run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
    # input()
    # run_skill("gotoJ_deg", -79.220100,-15.084805,-135.632080,-30.894413,-81.913452,-179.481079)#run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
    # input()
    # run_skill("moveEE_movJ", 0, 0, 150, 0, 0, 0)
    # run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    # run_skill("gotoJ_deg", *ESPRESSO_HOME)

def espresso_port_1_training(**params):
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.22)
    run_skill("get_machine_position", "three_group_espresso")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("sync")
    run_skill("move_to", "double_portafilter", 0.32)#run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("sync")
    run_skill("approach_tool", "double_portafilter")#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("grab_tool", "double_portafilter")#run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    run_skill("move_to", "double_portafilter", 0.32)#run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)

def espresso_port_2_training(**params):
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.22)
    run_skill("get_machine_position", "three_group_espresso")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("sync")
    run_skill("move_to", "single_portafilter", 0.22)#run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("sync")
    run_skill("approach_tool", "single_portafilter")#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    input()
    run_skill("grab_tool", "single_portafilter")#run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    input()
    run_skill("approach_tool", "single_portafilter")#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    run_skill("move_to", "single_portafilter", 0.22)#run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)

def angled_espresso_port_1_training(**params):
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.22)
    run_skill("get_machine_position", "three_group_espresso")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("sync")
    run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_1", True)
    input()
    run_skill("grab_tool", "double_portafilter_angled")#run_skill("mount_machine", "three_group_espresso", "angled_portafilter_1", True)
    input()
    run_skill("approach_tool", "double_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_1", True)
    run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)

def angled_espresso_port_2_training(**params):
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.22)
    run_skill("get_machine_position", "three_group_espresso")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("moveEE_movJ", 0, 0, -100, 0, 0, 0)
    run_skill("sync")
    run_skill("move_to", "single_portafilter_angled", 0.32)#run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("moveEE_movJ", 0, 0, -100, 0, 0, 0)
    run_skill("sync")
    run_skill("approach_tool", "single_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_2", True)
    input()
    run_skill("grab_tool", "single_portafilter_angled")#run_skill("mount_machine", "three_group_espresso", "angled_portafilter_2", True)
    input()
    run_skill("approach_tool", "single_portafilter_angled")#run_skill("approach_machine", "three_group_espresso", "angled_portafilter_2", True)
    run_skill("move_to", "single_portafilter_angled", 0.22)#run_skill("gotoJ_deg", 8.629592,-2.545630,-124.964149,-77.018211,-61.934883,12.157166)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)

def angled_grinder_training(**params):
    run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "espresso_grinder", 0.22)
    run_skill("get_machine_position", "espresso_grinder")
    input()
    run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    run_skill("gotoJ_deg", -37.955273,-73.296745,-105.604561,15.800714,-86.329605,-2.033083)#angled_grinder - approach
    input()
    run_skill("gotoJ_deg", -38.635151,-77.742249,-77.560051,-7.811272,-86.984062,-1.820310)#angled_grinder - mount
    input()
    run_skill("gotoJ_deg", -38.643422,-70.657534,-70.963554,-38.126536,-86.680101,-1.819383)#angled_tamper - approach
    input()
    run_skill("gotoJ_deg", -38.635151,-77.742249,-77.560051,-7.811272,-86.984062,-1.820310)#angled_tamper - mount
    input()
    run_skill("gotoJ_deg", -38.634125,-83.270256,-73.245041,-6.598258,-86.987930,-1.820723)
    input()
    run_skill("gotoJ_deg", -38.635151,-77.742249,-77.560051,-7.811272,-86.984062,-1.820310)
    input()
    run_skill("gotoJ_deg", -37.955273,-73.296745,-105.604561,15.800714,-86.329605,-2.033083)
    input()
    run_skill("gotoJ_deg", -32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    input()

def angled_cleaner_training(**params):
    home(position="east")
    run_skill("gotoJ_deg", -54.471272,-22.616722,-132.696136,-55.483162,-49.198364,24.286514)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "portafilter_cleaner", 0.22)
    run_skill("get_machine_position", "portafilter_cleaner")
    input()
    home(position="east")
    run_skill("gotoJ_deg", -35.223076, -2.939468, -128.314575, -47.896400, -73.999352, 1.973845)
    run_skill("gotoJ_deg", -75.2280066975024, -0.45636946506625187, -126.99544702752844, -56.18573222959352, -77.38542785813024, -1.305376569915049)
    #run_skill("approach_machine", "portafilter_cleaner", "angled_hard_brush")
    input()
    run_skill("gotoJ_deg", -102.563631,-4.349989,-116.815596,-58.573670,-102.493498,-149.923394)
    run_skill("gotoJ_deg", -92.144516,-15.545860,-116.165016,-62.150635,-91.587204,-179.165939)
    #run_skill("mount_machine", "portafilter_cleaner", "angled_hard_brush")
    input()
    run_skill("gotoJ_deg", -75.22800768895691, -0.456370203882616, -126.9954452304246, -56.18573675081598, -77.38543183310222, -178.80755111776034)
    # run_skill("approach_machine", "portafilter_cleaner", "angled_soft_brush")
    input()
    run_skill("gotoJ_deg", -79.643417,-13.190117,-118.912910,-63.592567,-83.353699,-179.155838)
    # run_skill("mount_machine", "portafilter_cleaner", "angled_soft_brush")

def test(**params):
    timings = []

    for outer_idx in range(1):
        get_machine_position()

        for inner_idx in range(10):
            start = time.perf_counter()
            pos = float(inner_idx%4 + 1)

            unmount(port="port_1")
            clean_portafilter(port="port_1")

            # 🔥 --- PARALLEL BLOCK 1 ---
            # t1 = threading.Thread(target=call_coffee_purge, kwargs={"slot_number": 1})
            # t2 = threading.Thread(target=call_grinder, kwargs={"shots_number": 2})
            t3 = threading.Thread(target=grinder, kwargs={"portafilter_tool": "double_portafilter"})

            # t1.start()
            # t2.start()
            # input()
            t3.start()

            # t1.join()
            # t2.join()
            t3.join()
            # 🔥 ------------------------

            # 🔥 --- PARALLEL BLOCK 2 ---
            t4 = threading.Thread(target=call_tamper, kwargs={"calibration_ms": 2000})
            t5 = threading.Thread(target=return_cleaned_espresso_pitcher, kwargs={"port": "port_1"})

            t4.start()
            t5.start()

            t4.join()
            t5.join()
            # 🔥 ------------------------

            tamper(portafilter_tool="double_portafilter")
            mount(port="port_1")

            # t6 = threading.Thread(target=call_coffee_machine, kwargs={"coffee_type": 2, "slot_number": 1})
            t7 = threading.Thread(target=dispense_paper_arm1_cup_station, kwargs={"size": "7oz", "position": {'cup_position': pos}})

            # t6.start()
            t7.start()

            
            t7.join()

            grab_espresso_pitcher(port="port_1")
            # t6.join()
            pick_espresso_pitcher(port="port_1")
            pour_espresso_pitcher_cup_station(position={'cup_position': pos})
            return_espresso_pitcher(port="port_1")

            if pos == 4.0:
                input()

            end = time.perf_counter()
            timings.append(end - start)

    print("Done")
    return timings

def test_arm1(**params):
    timings = []

    for outer_idx in range(3):
        get_machine_position()

        for inner_idx in range(3):
            start = time.perf_counter()
            pos = float(inner_idx%4 + 1)

            angled_unmount(port="angled_portafilter_2")
            angled_clean_portafilter(port="port_2")

            # 🔥 --- PARALLEL BLOCK 1 ---
            # t1 = threading.Thread(target=call_coffee_purge, kwargs={"slot_number": 2})
            # t2 = threading.Thread(target=call_grinder, kwargs={"shots_number": 1})
            t3 = threading.Thread(target=angled_grinder, kwargs={"portafilter_tool": "single_portafilter_angled"})

            # t1.start()
            # t2.start()
            t3.start()

            # t1.join()
            # t2.join()
            t3.join()
            # 🔥 ------------------------

            # 🔥 --- PARALLEL BLOCK 2 ---
            t4 = threading.Thread(target=call_tamper, kwargs={"calibration_ms": 2000})
            t5 = threading.Thread(target=angled_return_cleaned_espresso_pitcher, kwargs={"port": "port_2"})

            t4.start()
            t5.start()

            t4.join()
            t5.join()
            # 🔥 ------------------------

            angled_tamper(portafilter_tool="single_portafilter_angled")
            angled_mount(port="angled_portafilter_2")

            # t6 = threading.Thread(target=call_coffee_machine, kwargs={"coffee_type": 1, "slot_number": 2})
            # t7 = threading.Thread(target=dispense_paper_arm1_cup_station, kwargs={"size": "7oz", "position": {'cup_position': pos}})

            # # t6.start()
            # t7.start()

            
            # t7.join()

            # angled_grab_espresso_pitcher(port="port_2")
            # # t6.join()
            # angled_pick_espresso_pitcher(port="port_2")
            # angled_pour_espresso_pitcher_cup_station(position={'cup_position': pos})
            # angled_return_espresso_pitcher(port="port_2")

            # if pos == 4.0:
            #     input()

            run_skill("sync")

            end = time.perf_counter()
            timings.append(end - start)

    print("Done")
    return timings

def test_both_port(**params):
    timings = []

    outer_total = 7
    inner_total = 7
    total_planned_iterations = outer_total * inner_total

    attempted_iterations = 0
    completed_iterations = 0
    failed_iterations = 0

    def _run_sequence(label_steps, iteration_number):
        for step_number, (step_label, step_fn) in enumerate(label_steps, start=1):
            print(
                f"[ITER {iteration_number}/{total_planned_iterations}] "
                f"starting step {step_number}/{len(label_steps)}: {step_label}"
            )

            result = step_fn()

            if result is False or result is None:
                print(
                    f"[ITER {iteration_number}/{total_planned_iterations}] "
                    f"step {step_label!r} failed; aborting iteration"
                )
                return False, step_label

        return True, None

    for i in range(outer_total):
        get_machine_position()

        for j in range(inner_total):
            attempted_iterations += 1
            current_iteration = attempted_iterations

            print(
                f"\n[ITERATION START] "
                f"iteration={current_iteration}/{total_planned_iterations}, "
                f"i={i}, j={j}, "
                f"completed={completed_iterations}, failed={failed_iterations}"
            )

            # loop 1 timing
            t0 = time.perf_counter()

            # loop1_steps = [
            #     ("sync", lambda: run_skill("sync")),
            #     ("unmount", lambda: unmount(port="port_1")),
            #     ("clean_portafilter", lambda: clean_portafilter(port="port_1")),
            #     ("grinder", lambda: grinder(portafilter_tool="double_portafilter")),
            #     ("return_cleaned_espresso_pitcher",
            #      lambda: return_cleaned_espresso_pitcher(port="port_1")),
            #     ("tamper", lambda: tamper(portafilter_tool="double_portafilter")),
            #     ("mount", lambda: mount(port="port_1")),
            #     ("sync_end", lambda: run_skill("sync")),
            # ]
            # loop1_ok, loop1_failed_step = _run_sequence(loop1_steps, current_iteration)

            loop1_time = time.perf_counter() - t0

            # loop 2 timing
            t0 = time.perf_counter()

            loop2_steps = [
                ("sync", lambda: run_skill("sync")),
                ("angled_unmount", lambda: angled_unmount(port="angled_portafilter_2")),
                ("angled_clean_portafilter",
                 lambda: angled_clean_portafilter(port="angled_portafilter_2")),
                ("angled_grinder",
                 lambda: angled_grinder(portafilter_tool="single_portafilter_angled")),
                # ("angled_return_cleaned_espresso_pitcher",
                #  lambda: angled_return_cleaned_espresso_pitcher(port="port_2")),
                ("call_tamper", lambda: call_tamper(calibration_ms=2000)),
                ("angled_tamper",
                 lambda: angled_tamper(portafilter_tool="single_portafilter_angled")),
                ("angled_mount", lambda: angled_mount(port="angled_portafilter_2")),
                ("sync_end", lambda: run_skill("sync")),
            ]

            loop2_ok, loop2_failed_step = _run_sequence(
                loop2_steps,
                current_iteration
            )

            loop2_time = time.perf_counter() - t0

            if loop2_ok:
                completed_iterations += 1
            else:
                failed_iterations += 1

            timings.append({
                "iteration": current_iteration,
                "total_planned_iterations": total_planned_iterations,
                "completed_iterations": completed_iterations,
                "failed_iterations": failed_iterations,
                "outer_loop": i,
                "inner_loop": j,
                "loop_1_seconds": loop1_time,
                "loop_2_seconds": loop2_time,
                "loop_2_ok": loop2_ok,
                "loop_2_failed_step": loop2_failed_step,
            })

            status = "OK" if loop2_ok else f"FAIL@{loop2_failed_step}"

            print(
                f"[ITERATION END] "
                f"iteration={current_iteration}/{total_planned_iterations}, "
                f"i={i}, j={j} | "
                f"loop 1: {loop1_time:.3f}s | "
                f"loop 2: {loop2_time:.3f}s | "
                f"status={status} | "
                f"completed={completed_iterations}, "
                f"failed={failed_iterations}"
            )

            if not loop2_ok:
                print(
                    f"[test_both_port] aborting outer/inner loops at "
                    f"i={i}, j={j}, "
                    f"iteration={current_iteration}/{total_planned_iterations}"
                )
                print(
                    f"[SUMMARY] attempted={attempted_iterations}, "
                    f"completed={completed_iterations}, "
                    f"failed={failed_iterations}, "
                    f"remaining={total_planned_iterations - attempted_iterations}"
                )
                return timings

    print(
        f"[SUMMARY] attempted={attempted_iterations}, "
        f"completed={completed_iterations}, "
        f"failed={failed_iterations}, "
        f"remaining=0"
    )

    return timings

def test_plastic_cup(**params):
    timings = []
    invalidate_plastic_cup_cache()
    run_skill("sync")
    #loop 1
    t0 = time.perf_counter()
    run_skill("sync")
    dispense_plastic_cup(cup_size="12oz")
    go_to_ice(cup_size="12oz")  
    call_ice(weight=100) 
    go_home_with_ice(position={'cup_position': 1})
    place_plastic_cup_milk(cups={'cup_C12': 1.0})
    call_milk_syrup(device="milk", motor=19, amount=50)
    pick_plastic_cup_milk(cups={'cup_C12': 1.0})
    place_plastic_cup_sauces(cups={'cup_C12': 1.0})
    call_milk_syrup(device="syrup", motor=9, amount=50)
    pick_plastic_cup_sauces(cups={'cup_C12': 1.0})
    place_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C12': 1.0})
    run_skill("sync")
    loop1_time = time.perf_counter() - t0
    t0 = time.perf_counter()
    run_skill("sync")
    #loop 2
    dispense_plastic_cup(cup_size="12oz")
    go_to_ice(cup_size="12oz")  
    call_ice(weight=100) 
    go_home_with_ice(position={'cup_position': 1})
    place_plastic_cup_milk(cups={'cup_C12': 1.0})
    call_milk_syrup(device="milk", motor=19, amount=50)
    pick_plastic_cup_milk(cups={'cup_C12': 1.0})
    place_plastic_cup_sauces(cups={'cup_C12': 1.0})
    call_milk_syrup(device="syrup", motor=9, amount=50)
    pick_plastic_cup_sauces(cups={'cup_C12': 1.0})
    place_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C12': 1.0})
    run_skill("sync")
    loop2_time = time.perf_counter() - t0
    timings.append({
        "loop_1_seconds": loop1_time,
        "loop_2_seconds": loop2_time,
    })
    print(f"loop 1: {loop1_time:.3f}s | loop 2: {loop2_time:.3f}s")
    return timings

def test_paper_cup(**params):
    run_skill("gotoJ_deg", -160.082268,-37.525959,-134.136169,7.954679,-70.319756,-5.921233)
    run_skill("sync")
    run_skill("move_portafilter_arc_tool_angled", -8.2)
    run_skill("move_portafilter_arc_tool_angled", -8.2)
    run_skill("move_portafilter_arc_tool_angled", -8.2)
    run_skill("move_portafilter_arc_tool_angled", -8.2)
    run_skill("move_portafilter_arc_tool_angled", -8.2)
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 1)
    run_skill("move_portafilter_arc_tool_angled", 8.2)
    run_skill("move_portafilter_arc_tool_angled", 8.2)
    run_skill("move_portafilter_arc_tool_angled", 8.2)
    run_skill("move_portafilter_arc_tool_angled", 8.2)
    run_skill("move_portafilter_arc_tool_angled", 8.2)
    run_skill("sync")


    
        

import csv
import statistics
from pathlib import Path


START_JOINTS = (
    -127.669303, -38.953130, -30.134748,
    -110.679805, -127.621770, 0.127810
)

TOTAL_TRAVEL_MM = 900

# Step sizes to test. Each one will try to cover ~900 mm total.
STEP_SIZES_MM = [1, 5, 10, 25, 50, 90, 100, 150, 225, 300]

# Gripper commands
GRIPPER_OPEN = (255, 255, 255)
GRIPPER_CLOSE = (255, 0, 255)

# Output folder
OUT_DIR = Path.home() / "robot_timing_logs"


def now_ms() -> float:
    return time.perf_counter() * 1000.0

def summarize(values, label, skip_first=False):
    if skip_first and len(values) > 1:
        data = values[1:]
        suffix = " (excluding first)"
    else:
        data = values
        suffix = ""

    if not data:
        print(f"\n{label}{suffix}: no data")
        return None

    avg = statistics.mean(data)
    mn = min(data)
    mx = max(data)
    std = statistics.pstdev(data) if len(data) > 1 else 0.0
    med = statistics.median(data)

    print(f"\n{label}{suffix}:")
    print(f"  count = {len(data)}")
    print(f"  avg   = {avg:.3f} ms")
    print(f"  min   = {mn:.3f} ms")
    print(f"  max   = {mx:.3f} ms")
    print(f"  med   = {med:.3f} ms")
    print(f"  std   = {std:.3f} ms")

    # 3-sigma outliers
    if std > 0:
        lower = avg - 3 * std
        upper = avg + 3 * std
        outliers = [(i + 1, v) for i, v in enumerate(data) if v < lower or v > upper]
    else:
        outliers = []

    print(f"  outliers (3-sigma) = {len(outliers)}")
    for idx, v in outliers[:10]:
        print(f"    sample {idx:03d}: {v:.3f} ms")

    return {
        "count": len(data),
        "avg_ms": avg,
        "min_ms": mn,
        "max_ms": mx,
        "median_ms": med,
        "std_ms": std,
        "outliers_3sigma": len(outliers),
    }

def print_chunk_stats(values, label, chunk_size=10):
    if not values:
        return
    print(f"\n{label} chunk stats (chunk={chunk_size}):")
    for start in range(0, len(values), chunk_size):
        block = values[start:start + chunk_size]
        print(
            f"  {start+1:03d}-{start+len(block):03d}: "
            f"avg={statistics.mean(block):.3f} ms, "
            f"min={min(block):.3f}, "
            f"max={max(block):.3f}, "
            f"std={statistics.pstdev(block) if len(block) > 1 else 0.0:.3f}"
        )

def safe_run_skill(skill_name, *args):
    t0 = now_ms()
    ok = True
    err = ""
    try:
        run_skill(skill_name, *args)
    except Exception as e:
        ok = False
        err = repr(e)
    t1 = now_ms()
    return ok, (t1 - t0), err

def go_to_start():
    ok, dt, err = safe_run_skill("gotoJ_deg", *START_JOINTS)
    print(f"gotoJ_deg to start: {dt:.3f} ms")
    if not ok:
        print(f"ERROR in gotoJ_deg: {err}")
    return ok

def warmup():
    print("\n--- Warm-up ---")
    safe_run_skill("gotoJ_deg", *START_JOINTS)
    safe_run_skill("moveEE", 1, 0, 0, 0, 0, 0)
    safe_run_skill("set_gripper_position", *GRIPPER_OPEN)
    safe_run_skill("set_gripper_position", *GRIPPER_CLOSE)
    safe_run_skill("gotoJ_deg", *START_JOINTS)
    print("Warm-up done.")

def run_motion_series(motion_name, dx_mm, repeats, writer):
    """
    motion_name: 'moveEE' or 'moveEE_movJ'
    dx_mm: step size in +X
    repeats: number of repetitions
    """
    timings = []
    print(f"\n--- Testing {motion_name}: step={dx_mm} mm, repeats={repeats}, total={dx_mm * repeats} mm ---")

    # Always reset to same start pose before each series
    if not go_to_start():
        return timings

    for i in range(repeats):
        ok, dt, err = safe_run_skill(motion_name, dx_mm, 0, 0, 0, 0, 0)
        timings.append(dt)

        writer.writerow({
            "category": "motion",
            "test_name": motion_name,
            "sample_index": i + 1,
            "step_mm": dx_mm,
            "repeats": repeats,
            "total_travel_mm": dx_mm * repeats,
            "command": motion_name,
            "arg1": dx_mm,
            "arg2": 0,
            "arg3": 0,
            "arg4": 0,
            "arg5": 0,
            "arg6": 0,
            "dt_ms": f"{dt:.3f}",
            "ok": ok,
            "error": err,
        })

        print(f"{motion_name} {i+1:03d}/{repeats:03d}: {dt:.3f} ms" + ("" if ok else f"  ERROR={err}"))

        if not ok:
            print("Stopping this series because of error.")
            break

    summarize(timings, f"{motion_name} step={dx_mm} mm")
    summarize(timings, f"{motion_name} step={dx_mm} mm", skip_first=True)
    print_chunk_stats(timings, f"{motion_name} step={dx_mm} mm", chunk_size=min(10, max(1, len(timings) // 5 or 1)))
    return timings

def run_gripper_series(cycles, writer):
    print(f"\n--- Testing gripper: {cycles} open/close cycles ---")

    open_times = []
    close_times = []
    cycle_times = []

    # optional reset to start before gripper test
    go_to_start()

    for i in range(cycles):
        c0 = now_ms()

        ok1, dt_open, err1 = safe_run_skill("set_gripper_position", *GRIPPER_OPEN)
        open_times.append(dt_open)
        writer.writerow({
            "category": "gripper",
            "test_name": "gripper_open",
            "sample_index": i + 1,
            "step_mm": "",
            "repeats": cycles,
            "total_travel_mm": "",
            "command": "set_gripper_position",
            "arg1": GRIPPER_OPEN[0],
            "arg2": GRIPPER_OPEN[1],
            "arg3": GRIPPER_OPEN[2],
            "arg4": "",
            "arg5": "",
            "arg6": "",
            "dt_ms": f"{dt_open:.3f}",
            "ok": ok1,
            "error": err1,
        })

        ok2, dt_close, err2 = safe_run_skill("set_gripper_position", *GRIPPER_CLOSE)
        close_times.append(dt_close)
        writer.writerow({
            "category": "gripper",
            "test_name": "gripper_close",
            "sample_index": i + 1,
            "step_mm": "",
            "repeats": cycles,
            "total_travel_mm": "",
            "command": "set_gripper_position",
            "arg1": GRIPPER_CLOSE[0],
            "arg2": GRIPPER_CLOSE[1],
            "arg3": GRIPPER_CLOSE[2],
            "arg4": "",
            "arg5": "",
            "arg6": "",
            "dt_ms": f"{dt_close:.3f}",
            "ok": ok2,
            "error": err2,
        })

        c1 = now_ms()
        cycle_dt = c1 - c0
        cycle_times.append(cycle_dt)

        print(
            f"cycle {i+1:03d}/{cycles:03d}: "
            f"open={dt_open:.3f} ms, close={dt_close:.3f} ms, cycle={cycle_dt:.3f} ms"
        )

        if not ok1 or not ok2:
            print("Stopping gripper series because of error.")
            break

    summarize(open_times, "Gripper open")
    summarize(open_times, "Gripper open", skip_first=True)
    summarize(close_times, "Gripper close")
    summarize(close_times, "Gripper close", skip_first=True)
    summarize(cycle_times, "Gripper open+close cycle")
    summarize(cycle_times, "Gripper open+close cycle", skip_first=True)

    return open_times, close_times, cycle_times

def test_arm2(**params) -> bool:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    ts = time.strftime("%Y%m%d_%H%M%S")
    csv_path = OUT_DIR / f"robot_full_timing_{ts}.csv"

    print(f"Writing CSV to: {csv_path}")

    with open(csv_path, "w", newline="") as f:
        fieldnames = [
            "category",
            "test_name",
            "sample_index",
            "step_mm",
            "repeats",
            "total_travel_mm",
            "command",
            "arg1", "arg2", "arg3", "arg4", "arg5", "arg6",
            "dt_ms",
            "ok",
            "error",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        # One-time warm-up
        warmup()

        # Motion tests
        all_results = {}

        for step_mm in STEP_SIZES_MM:
            repeats = TOTAL_TRAVEL_MM // step_mm
            if repeats <= 0:
                continue

            total = repeats * step_mm
            if total > TOTAL_TRAVEL_MM:
                continue

            # Linear EE
            timings_lin = run_motion_series("moveEE", step_mm, repeats, writer)
            all_results[f"moveEE_{step_mm}"] = timings_lin

            # Reset before next type
            go_to_start()

            # Joint-style move
            timings_j = run_motion_series("moveEE_movJ", step_mm, repeats, writer)
            all_results[f"moveEE_movJ_{step_mm}"] = timings_j

        # Gripper tests
        # 20 cycles is already pretty informative; increase if you want more
        open_times, close_times, cycle_times = run_gripper_series(cycles=20, writer=writer)
        all_results["gripper_open"] = open_times
        all_results["gripper_close"] = close_times
        all_results["gripper_cycle"] = cycle_times

    print("\n=== FINAL COMPARISON ===")
    for key, vals in all_results.items():
        if not vals:
            continue
        use = vals[1:] if len(vals) > 1 else vals
        avg = statistics.mean(use)
        std = statistics.pstdev(use) if len(use) > 1 else 0.0
        print(f"{key:20s}  count={len(vals):4d}  avg(excl first)={avg:9.3f} ms  std={std:8.3f} ms")

    print(f"\nCSV saved: {csv_path}")
    return True

def robot_arm_test(**params):
    """
    Deterministic timing test:
    - Go to zero position
    - Move joint 4: +90 / -90 (3 cycles)
    - Repeat 10 times
    - Log timing for each run
    """

    def ok(r):
        return r not in (False, None)

    TOTAL_RUNS = 10
    CYCLES_PER_RUN = 3

    results = []

    print("\n===== ROBOT ARM TIMING TEST START =====\n")

    for run in range(1, TOTAL_RUNS + 1):
        print(f"\n--- RUN {run}/{TOTAL_RUNS} ---")

        # Ensure consistent speed
        run_skill("set_speed_factor", 100)
        run_skill("sync")

        # Move to zero/home
        if not ok(run_skill("gotoJ_deg", 0, 0, 0, 0, 0, 0)):
            print("[ERROR] Failed to move to zero position")
            return False

        run_skill("sync")
        time.sleep(0.5)  # small settle time

        start_time = time.time()
        print(f"[START] {start_time:.6f}")

        # Perform oscillation cycles
        for cycle in range(1, CYCLES_PER_RUN + 1):
            print(f"  Cycle {cycle}/{CYCLES_PER_RUN}")

            if not ok(run_skill("moveJ_deg", 0, 0, 0, 90, 0, 0)):
                print("[ERROR] Failed at +90")
                return False
            
            run_skill("sync")

            if not ok(run_skill("moveJ_deg", 0, 0, 0, -90, 0, 0)):
                print("[ERROR] Failed at -90")
                return False

            run_skill("sync")

        run_skill("sync")

        end_time = time.time()
        duration = end_time - start_time

        print(f"[END]   {end_time:.6f}")
        print(f"[DURATION] {duration:.4f} sec")

        results.append(duration)

        time.sleep(1.0)  # pause between runs

    # Summary
    print("\n===== TEST SUMMARY =====")
    for i, t in enumerate(results, 1):
        print(f"Run {i}: {t:.4f} sec")

    avg = sum(results) / len(results)
    print(f"\nAverage: {avg:.4f} sec")
    print(f"Min: {min(results):.4f} sec")
    print(f"Max: {max(results):.4f} sec")

    print("\n===== TEST COMPLETE =====\n")

    return True

def hello(**params):
    start_time = time.perf_counter()
    for i in range(3):
        run_skill("gotoJ_deg", 180.145874,31.742138,-120.866173,19.811152,-92.696129,10.536575) #P108
        for i in range(1):
            run_skill("gotoJ_deg", 210.743909,27.533575,-145.620954,39.206884,-92.263737,14.811911) #P109
            run_skill("gotoJ_deg", 180.145874,31.742138,-120.866173,19.811152,-92.696129,10.536575) #P108
            run_skill("gotoJ_deg", 164.649656,15.710394,-137.946233,38.069673,-92.976630,10.570545) #P110
            run_skill("gotoJ_deg", 180.145874,31.742138,-120.866173,19.811152,-92.696129,10.536575) #P108
        for i in range(3):
            run_skill("gotoJ_deg", 202.423675,31.972815,-130.894621,27.789850,-128.921782,10.127392) #P111
            run_skill("gotoJ_deg", 163.461313,31.625278,-130.333863,28.158369,-70.999501,10.265249) #P112
        run_skill("gotoJ_deg", 180.145874,31.742138,-120.866173,19.811152,-92.696129,10.536575) #P108
        run_skill("gotoJ_deg", 180.000000,14.953760,-117.783821,-76.976467,-88.599347,0.053953) #P1
    run_skill("sync")
    end_time = time.perf_counter()
    print(f"Total execution time: {end_time - start_time:.3f} seconds")
        # run_skill("gotoJ_deg", 181.706474,-15.556648,-147.169896,72.921334,-90.016973,-1.835663)
        # run_skill("set_gripper_position", 255, 255, 255)
        # run_skill("moveEE", 0, 0, 500, 0, 0, 0) 
        # run_skill("set_gripper_position", 255, 0, 255) 

# ──────────────────────────────────────────────────────────────────
# 2)  LOOK-UP TABLE  (function-name ↔︎ human-friendly key)
# ──────────────────────────────────────────────────────────────────
SEQUENCES = {
    "hello": lambda: hello(),
    # ═══════════════════════════════════════════════════════════════
    # 🏠 HOME POSITIONS
    # ═══════════════════════════════════════════════════════════════
    "home_north": lambda: home(position="north"),
    "home_south": lambda: home(position="south"),
    "home_east": lambda: home(position="east"),
    "home_west": lambda: home(position="west"),
    "home_north_east": lambda: home(position="north_east"),
    "home_north_west": lambda: home(position="north_west"),
    "home_south_east": lambda: home(position="south_east"),
    "home_south_west": lambda: home(position="south_west"),
    "home_zero": lambda: home(position="zero"),
    "return_back_to_home": lambda: return_back_to_home(),
    
    # ═══════════════════════════════════════════════════════════════
    # 🔧 CALIBRATION & SETUP
    # ═══════════════════════════════════════════════════════════════
    "get_machine_position": lambda: get_machine_position(),
    "get_frother_position": lambda: get_frother_position(),
    "check_saved_data": lambda: check_saved_data(),
    "check_aruco_status": lambda: check_aruco_status(),
    
    # ═══════════════════════════════════════════════════════════════
    # ☕ COMPLETE DRINK RECIPES
    # ═══════════════════════════════════════════════════════════════
    "espresso": lambda: espresso(),
    "americano": lambda: americano(),
    "multi_espresso": lambda: multi_espresso(),
    "milk_frothing": lambda: milk_frothing(),
    "slushie": lambda: slushie(),
    
    # ═══════════════════════════════════════════════════════════════
    # 🥤 PAPER CUP OPERATIONS
    # ═══════════════════════════════════════════════════════════════
    "grab_paper_cup_7oz": lambda: grab_paper_cup(size="7oz"),
    "grab_paper_cup_9oz": lambda: grab_paper_cup(size="9oz"),
    "grab_paper_cup_12oz": lambda: grab_paper_cup(size="12oz"),
    "grab_paper_cup_arm1_7oz": lambda: grab_paper_cup_arm1(size="7oz"),
    "grab_paper_cup_arm1_9oz": lambda: grab_paper_cup_arm1(size="9oz"),
    "grab_paper_cup_arm1_12oz": lambda: grab_paper_cup_arm1(size="12oz"),
    "grab_paper_arm2_cup_station_7oz": lambda: grab_paper_arm2_cup_station(size="7oz"),
    "grab_paper_arm2_cup_station_9oz": lambda: grab_paper_arm2_cup_station(size="9oz"),
    "grab_paper_arm2_cup_station_12oz": lambda: grab_paper_arm2_cup_station(size="12oz"),
    "place_paper_cup_stage_1": lambda: place_paper_cup(position={'cup_position': 1}),
    "place_paper_cup_stage_2": lambda: place_paper_cup(position={'cup_position': 2}),
    "place_paper_cup_stage_3": lambda: place_paper_cup(position={'cup_position': 3}),
    "place_paper_cup_stage_4": lambda: place_paper_cup(position={'cup_position': 4}),
    "place_paper_cup_arm1_stage_1": lambda: place_paper_cup_arm1(position={'cup_position': 1}),
    "place_paper_cup_arm1_stage_2": lambda: place_paper_cup_arm1(position={'cup_position': 2}),
    "place_paper_cup_arm1_stage_3": lambda: place_paper_cup_arm1(position={'cup_position': 3}),
    "place_paper_cup_arm1_stage_4": lambda: place_paper_cup_arm1(position={'cup_position': 4}),
    "place_paper__arm2_cup_station_stage_1": lambda: place_paper__arm2_cup_station(position={'cup_position': 1}),
    "place_paper__arm2_cup_station_stage_2": lambda: place_paper__arm2_cup_station(position={'cup_position': 2}),
    "place_paper__arm2_cup_station_stage_3": lambda: place_paper__arm2_cup_station(position={'cup_position': 3}),
    "place_paper__arm2_cup_station_stage_4": lambda: place_paper__arm2_cup_station(position={'cup_position': 4}),
    # Dispense paper cup (grab + place combined) - all size and stage combinations
    "dispense_paper_cup_7oz_stage_1": lambda: dispense_paper_cup_station(size="7oz", position={'cup_position': 1}),
    "dispense_paper_cup_7oz_stage_2": lambda: dispense_paper_cup_station(size="7oz", position={'cup_position': 2}),
    "dispense_paper_cup_7oz_stage_3": lambda: dispense_paper_cup_station(size="7oz", position={'cup_position': 3}),
    "dispense_paper_cup_7oz_stage_4": lambda: dispense_paper_cup_station(size="7oz", position={'cup_position': 4}),
    "dispense_paper_cup_9oz_stage_1": lambda: dispense_paper_cup_station(size="9oz", position={'cup_position': 1}),
    "dispense_paper_cup_9oz_stage_2": lambda: dispense_paper_cup_station(size="9oz", position={'cup_position': 2}),
    "dispense_paper_cup_9oz_stage_3": lambda: dispense_paper_cup_station(size="9oz", position={'cup_position': 3}),
    "dispense_paper_cup_9oz_stage_4": lambda: dispense_paper_cup_station(size="9oz", position={'cup_position': 4}),
    "dispense_paper_cup_12oz_stage_1": lambda: dispense_paper_cup_station(size="12oz", position={'cup_position': 1}),
    "dispense_paper_cup_12oz_stage_2": lambda: dispense_paper_cup_station(size="12oz", position={'cup_position': 2}),
    "dispense_paper_cup_12oz_stage_3": lambda: dispense_paper_cup_station(size="12oz", position={'cup_position': 3}),
    "dispense_paper_cup_12oz_stage_4": lambda: dispense_paper_cup_station(size="12oz", position={'cup_position': 4}),
    "dispense_paper_cup_arm1_7oz_stage_1": lambda: dispense_paper_arm1_cup_station(size="7oz", position={'cup_position': 1}),
    "dispense_paper_cup_arm1_7oz_stage_2": lambda: dispense_paper_arm1_cup_station(size="7oz", position={'cup_position': 2}),
    "dispense_paper_cup_arm1_7oz_stage_3": lambda: dispense_paper_arm1_cup_station(size="7oz", position={'cup_position': 3}),
    "dispense_paper_cup_arm1_7oz_stage_4": lambda: dispense_paper_arm1_cup_station(size="7oz", position={'cup_position': 4}),
    "dispense_paper_cup_arm1_9oz_stage_1": lambda: dispense_paper_arm1_cup_station(size="9oz", position={'cup_position': 1}),
    "dispense_paper_cup_arm1_9oz_stage_2": lambda: dispense_paper_arm1_cup_station(size="9oz", position={'cup_position': 2}),
    "dispense_paper_cup_arm1_9oz_stage_3": lambda: dispense_paper_arm1_cup_station(size="9oz", position={'cup_position': 3}),
    "dispense_paper_cup_arm1_9oz_stage_4": lambda: dispense_paper_arm1_cup_station(size="9oz", position={'cup_position': 4}),
    "dispense_paper_cup_arm1_12oz_stage_1": lambda: dispense_paper_arm1_cup_station(size="12oz", position={'cup_position': 1}),
    "dispense_paper_cup_arm1_12oz_stage_2": lambda: dispense_paper_arm1_cup_station(size="12oz", position={'cup_position': 2}),
    "dispense_paper_cup_arm1_12oz_stage_3": lambda: dispense_paper_arm1_cup_station(size="12oz", position={'cup_position': 3}),
    "dispense_paper_cup_arm1_12oz_stage_4": lambda: dispense_paper_arm1_cup_station(size="12oz", position={'cup_position': 4}),
    "dispense_paper_cup_arm2_7oz_stage_1": lambda: dispense_paper_arm2_cup_station(size="7oz", position={'cup_position': 1}),
    "dispense_paper_cup_arm2_7oz_stage_2": lambda: dispense_paper_arm2_cup_station(size="7oz", position={'cup_position': 2}),
    "dispense_paper_cup_arm2_7oz_stage_3": lambda: dispense_paper_arm2_cup_station(size="7oz", position={'cup_position': 3}),
    "dispense_paper_cup_arm2_7oz_stage_4": lambda: dispense_paper_arm2_cup_station(size="7oz", position={'cup_position': 4}),
    "dispense_paper_cup_arm2_9oz_stage_1": lambda: dispense_paper_arm2_cup_station(size="9oz", position={'cup_position': 1}),
    "dispense_paper_cup_arm2_9oz_stage_2": lambda: dispense_paper_arm2_cup_station(size="9oz", position={'cup_position': 2}),
    "dispense_paper_cup_arm2_9oz_stage_3": lambda: dispense_paper_arm2_cup_station(size="9oz", position={'cup_position': 3}),
    "dispense_paper_cup_arm2_9oz_stage_4": lambda: dispense_paper_arm2_cup_station(size="9oz", position={'cup_position': 4}),
    "dispense_paper_cup_arm2_12oz_stage_1": lambda: dispense_paper_arm2_cup_station(size="12oz", position={'cup_position': 1}),
    "dispense_paper_cup_arm2_12oz_stage_2": lambda: dispense_paper_arm2_cup_station(size="12oz", position={'cup_position': 2}),
    "dispense_paper_cup_arm2_12oz_stage_3": lambda: dispense_paper_arm2_cup_station(size="12oz", position={'cup_position': 3}),
    "dispense_paper_cup_arm2_12oz_stage_4": lambda: dispense_paper_arm2_cup_station(size="12oz", position={'cup_position': 4}),
    # ═══════════════════════════════════════════════════════════════
    # 🧊 PLASTIC CUP & ICE OPERATIONS
    # ═══════════════════════════════════════════════════════════════
    "dispense_plastic_cup_7oz": lambda: dispense_plastic_cup(cup_size="7oz"),
    "dispense_plastic_cup_9oz": lambda: dispense_plastic_cup(cup_size="9oz"),
    "dispense_plastic_cup_12oz": lambda: dispense_plastic_cup(cup_size="12oz"),
    "dispense_plastic_cup_16oz": lambda: dispense_plastic_cup(cup_size="16oz"),
    "go_to_ice_16oz": lambda: go_to_ice(cup_size="16oz"),
    "go_to_ice_12oz": lambda: go_to_ice(cup_size="12oz"),
    "go_to_ice_9oz": lambda: go_to_ice(cup_size="9oz"),
    "go_to_ice_7oz": lambda: go_to_ice(cup_size="7oz"),
    "go_home_with_ice": lambda: go_home_with_ice(position={'cup_position': 1}),
    # Place/Pick plastic cup at milk station - auto-detects after_dispense
    "place_plastic_cup_milk": lambda: place_plastic_cup_milk(cups={'cup_C16': 1.0}),
    "place_plastic_cup_milk_7oz": lambda: place_plastic_cup_milk(cups={'cup_C7': 1.0}),
    "place_plastic_cup_milk_9oz": lambda: place_plastic_cup_milk(cups={'cup_C9': 1.0}),
    "place_plastic_cup_milk_12oz": lambda: place_plastic_cup_milk(cups={'cup_C12': 1.0}),
    "place_plastic_cup_milk_16oz": lambda: place_plastic_cup_milk(cups={'cup_C16': 1.0}),
    # Pick from milk station – provide size-specific variants and a default (16oz)
    "pick_plastic_cup_milk": lambda: pick_plastic_cup_milk(cups={'cup_C16': 1.0}),
    "pick_plastic_cup_milk_7oz": lambda: pick_plastic_cup_milk(cups={'cup_C7': 1.0}),
    "pick_plastic_cup_milk_9oz": lambda: pick_plastic_cup_milk(cups={'cup_C9': 1.0}),
    "pick_plastic_cup_milk_12oz": lambda: pick_plastic_cup_milk(cups={'cup_C12': 1.0}),
    "pick_plastic_cup_milk_16oz": lambda: pick_plastic_cup_milk(cups={'cup_C16': 1.0}),
    # Place/Pick plastic cup at sauces station - auto-detects after_dispense
    "place_plastic_cup_sauces": lambda: place_plastic_cup_sauces(cups={'cup_C16': 1.0}),
    "place_plastic_cup_sauces_7oz": lambda: place_plastic_cup_sauces(cups={'cup_C7': 1.0}),
    "place_plastic_cup_sauces_9oz": lambda: place_plastic_cup_sauces(cups={'cup_C9': 1.0}),
    "place_plastic_cup_sauces_12oz": lambda: place_plastic_cup_sauces(cups={'cup_C12': 1.0}),
    "place_plastic_cup_sauces_16oz": lambda: place_plastic_cup_sauces(cups={'cup_C16': 1.0}),
    # Pick from sauces station – provide size-specific variants and a default (16oz)
    "pick_plastic_cup_sauces": lambda: pick_plastic_cup_sauces(cups={'cup_C16': 1.0}),
    "pick_plastic_cup_sauces_7oz": lambda: pick_plastic_cup_sauces(cups={'cup_C7': 1.0}),
    "pick_plastic_cup_sauces_9oz": lambda: pick_plastic_cup_sauces(cups={'cup_C9': 1.0}),
    "pick_plastic_cup_sauces_12oz": lambda: pick_plastic_cup_sauces(cups={'cup_C12': 1.0}),
    "pick_plastic_cup_sauces_16oz": lambda: pick_plastic_cup_sauces(cups={'cup_C16': 1.0}),
    "place_plastic_cup_station_stage_1_7oz": lambda: place_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C7': 1.0}),
    "place_plastic_cup_station_stage_1_9oz": lambda: place_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C9': 1.0}),
    "place_plastic_cup_station_stage_1_12oz": lambda: place_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C12': 1.0}),
    "place_plastic_cup_station_stage_1_16oz": lambda: place_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C16': 1.0}),
    "place_plastic_cup_station_stage_2_7oz": lambda: place_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C7': 1.0}),
    "place_plastic_cup_station_stage_2_9oz": lambda: place_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C9': 1.0}),
    "place_plastic_cup_station_stage_2_12oz": lambda: place_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C12': 1.0}),
    "place_plastic_cup_station_stage_2_16oz": lambda: place_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C16': 1.0}),
    "place_plastic_cup_station_stage_3_7oz": lambda: place_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C7': 1.0}),
    "place_plastic_cup_station_stage_3_9oz": lambda: place_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C9': 1.0}),
    "place_plastic_cup_station_stage_3_12oz": lambda: place_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C12': 1.0}),
    "place_plastic_cup_station_stage_3_16oz": lambda: place_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C16': 1.0}),
    "place_plastic_cup_station_stage_4_7oz": lambda: place_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C7': 1.0}),
    "place_plastic_cup_station_stage_4_9oz": lambda: place_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C9': 1.0}),
    "place_plastic_cup_station_stage_4_12oz": lambda: place_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C12': 1.0}),
    "place_plastic_cup_station_stage_4_16oz": lambda: place_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C16': 1.0}),
    # Pick from station (for ice) – provide all stage x size combinations - updated with cups dict format
    "pick_plastic_cup_station_stage_1_7oz": lambda: pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C7': 1.0}),
    "pick_plastic_cup_station_stage_1_9oz": lambda: pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C9': 1.0}),
    "pick_plastic_cup_station_stage_1_12oz": lambda: pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C12': 1.0}),
    "pick_plastic_cup_station_stage_1_16oz": lambda: pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C16': 1.0}),
    "pick_plastic_cup_station_stage_2_7oz": lambda: pick_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C7': 1.0}),
    "pick_plastic_cup_station_stage_2_9oz": lambda: pick_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C9': 1.0}),
    "pick_plastic_cup_station_stage_2_12oz": lambda: pick_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C12': 1.0}),
    "pick_plastic_cup_station_stage_2_16oz": lambda: pick_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C16': 1.0}),
    "pick_plastic_cup_station_stage_3_7oz": lambda: pick_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C7': 1.0}),
    "pick_plastic_cup_station_stage_3_9oz": lambda: pick_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C9': 1.0}),
    "pick_plastic_cup_station_stage_3_12oz": lambda: pick_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C12': 1.0}),
    "pick_plastic_cup_station_stage_3_16oz": lambda: pick_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C16': 1.0}),
    "pick_plastic_cup_station_stage_4_7oz": lambda: pick_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C7': 1.0}),
    "pick_plastic_cup_station_stage_4_9oz": lambda: pick_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C9': 1.0}),
    "pick_plastic_cup_station_stage_4_12oz": lambda: pick_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C12': 1.0}),
    "pick_plastic_cup_station_stage_4_16oz": lambda: pick_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C16': 1.0}),
    # Backward-compatible aliases (map legacy keys to correct function)
    "pick_plastic_cup_for_ice_stage_1_7oz": lambda: pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C7': 1.0}),
    "pick_plastic_cup_for_ice_stage_1_9oz": lambda: pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C9': 1.0}),
    "pick_plastic_cup_for_ice_stage_1_12oz": lambda: pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C12': 1.0}),
    "pick_plastic_cup_for_ice_stage_1_16oz": lambda: pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C16': 1.0}),
    "pick_plastic_cup_for_ice_stage_2_7oz": lambda: pick_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C7': 1.0}),
    "pick_plastic_cup_for_ice_stage_2_9oz": lambda: pick_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C9': 1.0}),
    "pick_plastic_cup_for_ice_stage_2_12oz": lambda: pick_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C12': 1.0}),
    "pick_plastic_cup_for_ice_stage_2_16oz": lambda: pick_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C16': 1.0}),
    "pick_plastic_cup_for_ice_stage_3_7oz": lambda: pick_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C7': 1.0}),
    "pick_plastic_cup_for_ice_stage_3_9oz": lambda: pick_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C9': 1.0}),
    "pick_plastic_cup_for_ice_stage_3_12oz": lambda: pick_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C12': 1.0}),
    "pick_plastic_cup_for_ice_stage_3_16oz": lambda: pick_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C16': 1.0}),
    "pick_plastic_cup_for_ice_stage_4_7oz": lambda: pick_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C7': 1.0}),
    "pick_plastic_cup_for_ice_stage_4_9oz": lambda: pick_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C9': 1.0}),
    "pick_plastic_cup_for_ice_stage_4_12oz": lambda: pick_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C12': 1.0}),
    "pick_plastic_cup_for_ice_stage_4_16oz": lambda: pick_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C16': 1.0}),
    
    # ═══════════════════════════════════════════════════════════════
    # 🧊 SLUSH OPERATIONS (16oz only - working combinations)
    # ═══════════════════════════════════════════════════════════════
    "get_slush_d1_s1": lambda: get_slush(dispenser="1", position={'cup_position': 1}, cup_size="9oz"),
    "get_slush_d1_s2": lambda: get_slush(dispenser="1", position={'cup_position': 2}, cup_size="12oz"),
    "get_slush_d1_s3": lambda: get_slush(dispenser="1", position={'cup_position': 3}, cup_size="16oz"),
    "get_slush_d1_s4": lambda: get_slush(dispenser="1", position={'cup_position': 4}, cup_size="9oz"),
    "get_slush_d2_s1": lambda: get_slush(dispenser="2", position={'cup_position': 1}, cup_size="9oz"),
    "get_slush_d2_s2": lambda: get_slush(dispenser="2", position={'cup_position': 2}, cup_size="12oz"),
    "get_slush_d2_s3": lambda: get_slush(dispenser="2", position={'cup_position': 3}, cup_size="16oz"),
    "get_slush_d2_s4": lambda: get_slush(dispenser="2", position={'cup_position': 4}, cup_size="9oz"),
    "place_slush_d1_s1": lambda: place_slush(dispenser="1", position={'cup_position': 1}, cup_size="9oz"),
    "place_slush_d1_s2": lambda: place_slush(dispenser="1", position={'cup_position': 2}, cup_size="12oz"),
    "place_slush_d1_s3": lambda: place_slush(dispenser="1", position={'cup_position': 3}, cup_size="16oz"),
    "place_slush_d1_s4": lambda: place_slush(dispenser="1", position={'cup_position': 4}, cup_size="9oz"),
    "place_slush_d2_s1": lambda: place_slush(dispenser="2", position={'cup_position': 1}, cup_size="9oz"),
    "place_slush_d2_s2": lambda: place_slush(dispenser="2", position={'cup_position': 2}, cup_size="12oz"),
    "place_slush_d2_s3": lambda: place_slush(dispenser="2", position={'cup_position': 3}, cup_size="16oz"),
    "place_slush_d2_s4": lambda: place_slush(dispenser="2", position={'cup_position': 4}, cup_size="9oz"),
    
    # ═══════════════════════════════════════════════════════════════
    # ⚙️ ESPRESSO MACHINE OPERATIONS
    # ═══════════════════════════════════════════════════════════════
    "unmount_port_1": lambda: unmount(port="port_1"),
    "unmount_port_2": lambda: unmount(port="port_2"),
    "unmount_port_3": lambda: unmount(port="port_3"),
    "mount_port_1": lambda: mount(port="port_1"),
    "mount_port_2": lambda: mount(port="port_2"),
    "mount_port_3": lambda: mount(port="port_3"),
    "grinder": lambda: grinder(),
    "tamper": lambda: tamper(),
    "pick_espresso_pitcher_port_1": lambda: pick_espresso_pitcher(port="port_1"),
    "pick_espresso_pitcher_port_2": lambda: pick_espresso_pitcher(port="port_2"),
    "pick_espresso_pitcher_port_3": lambda: pick_espresso_pitcher(port="port_3"),
    # Return pitcher per port
    "pour_espresso_pitcher_stage_1": lambda: pour_espresso_pitcher_cup_station(position={'cup_position': 1}),
    "pour_espresso_pitcher_stage_2": lambda: pour_espresso_pitcher_cup_station(position={'cup_position': 2}),
    "pour_espresso_pitcher_stage_3": lambda: pour_espresso_pitcher_cup_station(position={'cup_position': 3}),
    "pour_espresso_pitcher_stage_4": lambda: pour_espresso_pitcher_cup_station(position={'cup_position': 4}),
    "return_espresso_pitcher_port_1": lambda: return_espresso_pitcher(port="port_1"),
    "return_espresso_pitcher_port_2": lambda: return_espresso_pitcher(port="port_2"),
    "return_espresso_pitcher_port_3": lambda: return_espresso_pitcher(port="port_3"),
    "return_cleaned_espresso_pitcher_port_1": lambda: return_cleaned_espresso_pitcher(port="port_1"),
    "return_cleaned_espresso_pitcher_port_2": lambda: return_cleaned_espresso_pitcher(port="port_2"),
    "return_cleaned_espresso_pitcher_port_3": lambda: return_cleaned_espresso_pitcher(port="port_3"),
    # Unmount/mount for explicit ports
    "unmount_p1": lambda: unmount(port="port_1"),
    "unmount_p2": lambda: unmount(port="port_2"),
    "unmount_p3": lambda: unmount(port="port_3"),
    "mount_p1": lambda: mount(port="port_1"),
    "mount_p2": lambda: mount(port="port_2"),
    "mount_p3": lambda: mount(port="port_3"),
    "get_hot_water": lambda: get_hot_water(),
    "with_hot_water": lambda: with_hot_water(),
    "pick_cup_hot_water_stage_1_7oz": lambda: pick_cup_for_hot_water(position={'cup_position': 1}, cups_dict={"cup_H7": 1.0}),
    "pick_cup_hot_water_stage_1_9oz": lambda: pick_cup_for_hot_water(position={'cup_position': 1}, cups_dict={"cup_H9": 1.0}),
    "pick_cup_hot_water_stage_1_12oz": lambda: pick_cup_for_hot_water(position={'cup_position': 1}, cups_dict={"cup_H12": 1.0}),
    "pick_cup_hot_water_stage_2_7oz": lambda: pick_cup_for_hot_water(position={'cup_position': 2}, cups_dict={"cup_H7": 1.0}),
    "pick_cup_hot_water_stage_2_9oz": lambda: pick_cup_for_hot_water(position={'cup_position': 2}, cups_dict={"cup_H9": 1.0}),
    "pick_cup_hot_water_stage_2_12oz": lambda: pick_cup_for_hot_water(position={'cup_position': 2}, cups_dict={"cup_H12": 1.0}),
    "pick_cup_hot_water_stage_3_7oz": lambda: pick_cup_for_hot_water(position={'cup_position': 3}, cups_dict={"cup_H7": 1.0}),
    "pick_cup_hot_water_stage_3_9oz": lambda: pick_cup_for_hot_water(position={'cup_position': 3}, cups_dict={"cup_H9": 1.0}),
    "pick_cup_hot_water_stage_3_12oz": lambda: pick_cup_for_hot_water(position={'cup_position': 3}, cups_dict={"cup_H12": 1.0}),
    "pick_cup_hot_water_stage_4_7oz": lambda: pick_cup_for_hot_water(position={'cup_position': 4}, cups_dict={"cup_H7": 1.0}),
    "pick_cup_hot_water_stage_4_9oz": lambda: pick_cup_for_hot_water(position={'cup_position': 4}, cups_dict={"cup_H9": 1.0}),
    "pick_cup_hot_water_stage_4_12oz": lambda: pick_cup_for_hot_water(position={'cup_position': 4}, cups_dict={"cup_H12": 1.0}),
    "return_cup_hot_water_stage_1": lambda: return_cup_with_hot_water(position={'cup_position': 1}),
    "return_cup_hot_water_stage_2": lambda: return_cup_with_hot_water(position={'cup_position': 2}),
    "return_cup_hot_water_stage_3": lambda: return_cup_with_hot_water(position={'cup_position': 3}),
    "return_cup_hot_water_stage_4": lambda: return_cup_with_hot_water(position={'cup_position': 4}),

    # Angled espresso/cleaning (separate caches and globals; same motion as defaults until customized)
    "angled_unmount_port_1": lambda: angled_unmount(port="angled_portafilter_1"),
    "angled_unmount_port_2": lambda: angled_unmount(port="angled_portafilter_2"),
    "angled_unmount_port_3": lambda: angled_unmount(port="angled_portafilter_3"),
    "angled_mount_port_1": lambda: angled_mount(port="angled_portafilter_1"),
    "angled_mount_port_2": lambda: angled_mount(port="angled_portafilter_2"),
    "angled_mount_port_3": lambda: angled_mount(port="angled_portafilter_3"),
    "angled_grinder": lambda: angled_grinder(),
    "angled_tamper": lambda: angled_tamper(),
    "angled_pick_espresso_pitcher_port_1": lambda: angled_pick_espresso_pitcher(port="port_1"),
    "angled_pick_espresso_pitcher_port_2": lambda: angled_pick_espresso_pitcher(port="port_2"),
    "angled_pick_espresso_pitcher_port_3": lambda: angled_pick_espresso_pitcher(port="port_3"),
    "angled_pour_espresso_pitcher_stage_1": lambda: angled_pour_espresso_pitcher_cup_station(position={'cup_position': 1}),
    "angled_pour_espresso_pitcher_stage_2": lambda: angled_pour_espresso_pitcher_cup_station(position={'cup_position': 2}),
    "angled_pour_espresso_pitcher_stage_3": lambda: angled_pour_espresso_pitcher_cup_station(position={'cup_position': 3}),
    "angled_pour_espresso_pitcher_stage_4": lambda: angled_pour_espresso_pitcher_cup_station(position={'cup_position': 4}),
    "angled_return_espresso_pitcher_port_1": lambda: angled_return_espresso_pitcher(port="port_1"),
    "angled_return_espresso_pitcher_port_2": lambda: angled_return_espresso_pitcher(port="port_2"),
    "angled_return_espresso_pitcher_port_3": lambda: angled_return_espresso_pitcher(port="port_3"),
    "angled_return_cleaned_espresso_pitcher_port_1": lambda: angled_return_cleaned_espresso_pitcher(port="port_1"),
    "angled_return_cleaned_espresso_pitcher_port_2": lambda: angled_return_cleaned_espresso_pitcher(port="port_2"),
    "angled_return_cleaned_espresso_pitcher_port_3": lambda: angled_return_cleaned_espresso_pitcher(port="port_3"),
    "angled_unmount_p1": lambda: angled_unmount(port="angled_portafilter_1"),
    "angled_unmount_p2": lambda: angled_unmount(port="angled_portafilter_2"),
    "angled_unmount_p3": lambda: angled_unmount(port="angled_portafilter_3"),
    "angled_mount_p1": lambda: angled_mount(port="angled_portafilter_1"),
    "angled_mount_p2": lambda: angled_mount(port="angled_portafilter_2"),
    "angled_mount_p3": lambda: angled_mount(port="angled_portafilter_3"),
    "angled_get_hot_water": lambda: angled_get_hot_water(),
    "angled_with_hot_water": lambda: angled_with_hot_water(),
    "angled_clean_port_1": lambda: angled_clean_portafilter(port="port_1"),
    "angled_clean_port_2": lambda: angled_clean_portafilter(port="port_2"),
    "angled_clean_port_3": lambda: angled_clean_portafilter(port="port_3"),
    "angled_clean": lambda: angled_clean_portafilter(port="port_2"),
    "angled_espresso_training": lambda: angled_espresso_training(),
    "angled_cleaner_training": lambda: angled_cleaner_training(),
    # ═══════════════════════════════════════════════════════════════
    # 🥛 MILK FROTHING OPERATIONS
    # ═══════════════════════════════════════════════════════════════
    "pick_frother": lambda: pick_frother(),
    "place_frother_milk_station": lambda: place_frother_milk_station(),
    "pick_frother_milk_station": lambda: pick_frother_milk_station(),
    "mount_frother": lambda: mount_frother(),
    # Replace undefined operations with implemented ones
    "unmount_and_swirl_milk": lambda: unmount_and_swirl_milk(),
    "pour_milk_stage_1": lambda: pour_milk_cup_station(position={'cup_position': 1}),
    "pour_milk_stage_2": lambda: pour_milk_cup_station(position={'cup_position': 2}),
    "pour_milk_stage_3": lambda: pour_milk_cup_station(position={'cup_position': 3}),
    "pour_milk_stage_4": lambda: pour_milk_cup_station(position={'cup_position': 4}),
    "return_frother": lambda: return_frother(),
    "clean_milk_pitcher": lambda: clean_milk_pitcher(),

    # ═══════════════════════════════════════════════════════════════
    # 🥤 PAPER CUP STATION / SAUCES / MILK (expanded variants)
    # ═══════════════════════════════════════════════════════════════
    "place_paper_cup_station_stage_1": lambda: place_paper_cup_station(position={'cup_position': 1}),
    "place_paper_cup_station_stage_2": lambda: place_paper_cup_station(position={'cup_position': 2}),
    "place_paper_cup_station_stage_3": lambda: place_paper_cup_station(position={'cup_position': 3}),
    "place_paper_cup_station_stage_4": lambda: place_paper_cup_station(position={'cup_position': 4}),
    "pick_paper_cup_station_stage_1_7oz": lambda: pick_paper_cup_station(position={'cup_position': 1}, cup_size="7oz"),
    "pick_paper_cup_station_stage_1_9oz": lambda: pick_paper_cup_station(position={'cup_position': 1}, cup_size="9oz"),
    "pick_paper_cup_station_stage_1_12oz": lambda: pick_paper_cup_station(position={'cup_position': 1}, cup_size="12oz"),
    "pick_paper_cup_station_stage_2_7oz": lambda: pick_paper_cup_station(position={'cup_position': 2}, cup_size="7oz"),
    "pick_paper_cup_station_stage_2_9oz": lambda: pick_paper_cup_station(position={'cup_position': 2}, cup_size="9oz"),
    "pick_paper_cup_station_stage_2_12oz": lambda: pick_paper_cup_station(position={'cup_position': 2}, cup_size="12oz"),
    "pick_paper_cup_station_stage_3_7oz": lambda: pick_paper_cup_station(position={'cup_position': 3}, cup_size="7oz"),
    "pick_paper_cup_station_stage_3_9oz": lambda: pick_paper_cup_station(position={'cup_position': 3}, cup_size="9oz"),
    "pick_paper_cup_station_stage_3_12oz": lambda: pick_paper_cup_station(position={'cup_position': 3}, cup_size="12oz"),
    "pick_paper_cup_station_stage_4_7oz": lambda: pick_paper_cup_station(position={'cup_position': 4}, cup_size="7oz"),
    "pick_paper_cup_station_stage_4_9oz": lambda: pick_paper_cup_station(position={'cup_position': 4}, cup_size="9oz"),
    "pick_paper_cup_station_stage_4_12oz": lambda: pick_paper_cup_station(position={'cup_position': 4}, cup_size="12oz"),
    "place_paper_cup_sauces_7oz": lambda: place_paper_cup_sauces(cup_size="7oz"),
    "place_paper_cup_sauces_9oz": lambda: place_paper_cup_sauces(cup_size="9oz"),
    "place_paper_cup_sauces_12oz": lambda: place_paper_cup_sauces(cup_size="12oz"),
    "pick_paper_cup_sauces_7oz": lambda: pick_paper_cup_sauces(cup_size="7oz"),
    "pick_paper_cup_sauces_9oz": lambda: pick_paper_cup_sauces(cup_size="9oz"),
    "pick_paper_cup_sauces_12oz": lambda: pick_paper_cup_sauces(cup_size="12oz"),
    "place_paper_cup_milk_7oz": lambda: place_paper_cup_milk(cup_size="7oz"),
    "place_paper_cup_milk_9oz": lambda: place_paper_cup_milk(cup_size="9oz"),
    "place_paper_cup_milk_12oz": lambda: place_paper_cup_milk(cup_size="12oz"),
    "pick_paper_cup_milk_7oz": lambda: pick_paper_cup_milk(cup_size="7oz"),
    "pick_paper_cup_milk_9oz": lambda: pick_paper_cup_milk(cup_size="9oz"),
    "pick_paper_cup_milk_12oz": lambda: pick_paper_cup_milk(cup_size="12oz"),
    
    # ═══════════════════════════════════════════════════════════════
    # 🧹 CLEANING OPERATIONS
    # ═══════════════════════════════════════════════════════════════
    "clean_port_1": lambda: clean_portafilter(port="port_1"),
    "clean_port_2": lambda: clean_portafilter(port="port_2"),
    "clean_port_3": lambda: clean_portafilter(port="port_3"),
    "clean": lambda: clean_portafilter(port="port_2"),
    
    # ═══════════════════════════════════════════════════════════════
    # 🧪 TRAINING & TESTING
    # ═══════════════════════════════════════════════════════════════
    "espresso_training": lambda: espresso_training(),
    "milk_training": lambda: milk_training(),
    "test": lambda: test(),
    "test_plastic_cup": lambda: test_plastic_cup(),
    "test_paper_cup": lambda: test_paper_cup(),
    "test_arm2": lambda: test_arm2(),
    "test_arm1": lambda: test_arm1(),
    "test_both_port": lambda: test_both_port(),
    "milk_1": lambda: milk_1(),
    "milk_2": lambda: milk_2(),
    "milk_3": lambda: milk_3(),
    "milk_4": lambda: milk_4(),
    "espresso_angled": lambda: espresso_angled(),
    # ═══════════════════════════════════════════════════════════════
    # 📸 COMPUTER VISION & DETECTION
    # ═══════════════════════════════════════════════════════════════
    "detect_cup_gripper": lambda: detect_cup_gripper(),
    
    # ═══════════════════════════════════════════════════════════════
    # 🔧 UTILITY FUNCTIONS
    # ═══════════════════════════════════════════════════════════════
    "show_version_info": lambda: show_version_info(),
    "switch_version": lambda: switch_version(),
    "robot_arm_test": lambda: robot_arm_test(),
    "espresso_port_1_training": lambda: espresso_port_1_training(),
    "espresso_port_2_training": lambda: espresso_port_2_training(),
    "angled_espresso_port_1_training": lambda: angled_espresso_port_1_training(),
    "angled_espresso_port_2_training": lambda: angled_espresso_port_2_training(),
    "tamper_3500": lambda: call_tamper(calibration_ms=2075),
    "coffee_single": lambda: call_coffee_machine(coffee_type=1, slot_number=2),
    "coffee_double": lambda: call_coffee_machine(coffee_type=2, slot_number=1),
    "hot_water": lambda: call_hot_water(calibration=2),
    "coffee_purge_single": lambda: call_coffee_purge(slot_number=2),
    "coffee_purge_double": lambda: call_coffee_purge(slot_number=1),
    "grind_single": lambda: call_grinder(shots_number=1),
    "grind_double": lambda: call_grinder(shots_number=2),
    "ice_small": lambda: call_ice(weight=80),
    "ice_large": lambda: call_ice(weight=150),
    "slush": lambda: call_slush(type="slush_2", weight=150, difference=25),
    "milk": lambda: call_milk_syrup(device="milk", motor=8, amount=50),
    "syrup": lambda: call_milk_syrup(device="syrup", motor=7, amount=20),
    "froth": lambda: call_frother(command="froth", temp=70),
    "frother_clean": lambda: call_frother(command="clean"),
}

# ------------------------------------------------------------------
#  Dobot kinematics / bringup tools (same behavior as ~/kinemtaics_solutions.py)
#  Invoked from the main CLI; options 1-10 are only inside this submenu.
# ------------------------------------------------------------------
def _print_kinematics_srv_response(res):
    try:
        fields = res._fields_and_field_types
    except AttributeError:
        fields = {slot: None for slot in res.__slots__}
    print("Service response:")
    for field in fields:
        val = getattr(res, field)
        if isinstance(val, (float, int)):
            print(f"  {field}={val:.6f}")
        else:
            print(f"  {field}={val}")


def run_kinematics_tools_menu():
    """Interactive 1-10 menu mirroring kinemtaics_solutions.py; returns to sequence CLI on q."""
    if not rclpy.ok():
        rclpy.init(args=None)

    knode = rclpy.create_node("kinematics_tools")
    try:
        services = {
            "inverse": knode.create_client(InverseSolution, "/dobot_bringup_v3/srv/InverseSolution"),
            "forward": knode.create_client(PositiveSolution, "/dobot_bringup_v3/srv/PositiveSolution"),
            "pose": knode.create_client(GetPose, "/dobot_bringup_v3/srv/GetPose"),
            "angle": knode.create_client(GetAngle, "/dobot_bringup_v3/srv/GetAngle"),
            "start_drag": knode.create_client(StartDrag, "/dobot_bringup_v3/srv/StartDrag"),
            "stop_drag": knode.create_client(StopDrag, "/dobot_bringup_v3/srv/StopDrag"),
            "set_gripper": knode.create_client(SetGripperPosition, "/dobot_bringup_v3/srv/SetGripperPosition"),
            "get_gripper": knode.create_client(GetGripperPosition, "/dobot_bringup_v3/srv/GetGripperPosition"),
            "clear_error": knode.create_client(ClearError, "/dobot_bringup_v3/srv/ClearError"),
            "disable_robot": knode.create_client(DisableRobot, "/dobot_bringup_v3/srv/DisableRobot"),
            "enable_robot": knode.create_client(EnableRobot, "/dobot_bringup_v3/srv/EnableRobot"),
            "modbus_close": knode.create_client(ModbusClose, "/dobot_bringup_v3/srv/ModbusClose"),
            "modbus_create": knode.create_client(ModbusCreate, "/dobot_bringup_v3/srv/ModbusCreate"),
            "set_hold_regs": knode.create_client(SetHoldRegs, "/dobot_bringup_v3/srv/SetHoldRegs"),
            "cp": knode.create_client(CP, "/dobot_bringup_v3/srv/CP"),
        }

        service_wait_sec = 10.0
        for name, cli in services.items():
            knode.get_logger().info(f"Waiting for {name} service...")
            if not cli.wait_for_service(timeout_sec=service_wait_sec):
                knode.get_logger().warn(
                    f"Service '{name}' not ready after {service_wait_sec}s (calls may fail)."
                )

        menu = [
            "1. Inverse Kinematics",
            "2. Forward Kinematics",
            "3. Get Current Pose",
            "4. Get Current Angles",
            "5. Start Drag",
            "6. Stop Drag",
            "7. Open Gripper",
            "8. Close Gripper",
            "9. Get Gripper Position",
            "10. Initialize",
            "Q. Back to sequence menu",
        ]

        def call(name, req):
            future = services[name].call_async(req)
            rclpy.spin_until_future_complete(knode, future, timeout_sec=30.0)
            if future.done() and future.result() is not None:
                _print_kinematics_srv_response(future.result())
            else:
                print("Service call failed.")

        while True:
            print("\nSelect mode (kinemtaics_solutions.py):")
            for item in menu:
                print(item)
            choice = input("Enter choice: ").strip().lower()

            if choice in ("q", "quit", "exit", "back"):
                print("Returning to sequence menu.\n")
                break

            if choice in ("1", "inverse", "i"):
                vals = input("Enter x,y,z,rx,ry,rz: ").split(",")
                if len(vals) != 6:
                    print("Please enter 6 comma-separated values.")
                    continue
                try:
                    req = InverseSolution.Request(
                        x=float(vals[0]),
                        y=float(vals[1]),
                        z=float(vals[2]),
                        rx=float(vals[3]),
                        ry=float(vals[4]),
                        rz=float(vals[5]),
                    )
                except ValueError:
                    print("Invalid numeric input.")
                    continue
                call("inverse", req)

            elif choice in ("2", "forward", "f"):
                vals = input("Enter j1,j2,j3,j4,j5,j6: ").split(",")
                if len(vals) != 6:
                    print("Please enter 6 comma-separated values.")
                    continue
                try:
                    req = PositiveSolution.Request(
                        j1=float(vals[0]),
                        j2=float(vals[1]),
                        j3=float(vals[2]),
                        j4=float(vals[3]),
                        j5=float(vals[4]),
                        j6=float(vals[5]),
                    )
                except ValueError:
                    print("Invalid numeric input.")
                    continue
                call("forward", req)

            elif choice in ("3", "pose", "p"):
                gp = GetPose.Request()
                gp.user = 0
                gp.tool = 0
                call("pose", gp)

            elif choice in ("4", "angle", "a"):
                call("angle", GetAngle.Request())

            elif choice in ("5", "start drag", "sd"):
                call("start_drag", StartDrag.Request())

            elif choice in ("6", "stop drag", "td"):
                call("stop_drag", StopDrag.Request())

            elif choice in ("7", "open gripper", "og"):
                call(
                    "set_gripper",
                    SetGripperPosition.Request(position=0, speed=255, force=255),
                )

            elif choice in ("8", "close gripper", "cg"):
                call(
                    "set_gripper",
                    SetGripperPosition.Request(position=255, speed=255, force=255),
                )

            elif choice in ("9", "get gripper", "gg"):
                call("get_gripper", GetGripperPosition.Request())

            elif choice in ("10", "initialize", "init"):
                print("Initializing sequence...")
                call("clear_error", ClearError.Request())
                time.sleep(0.1)
                call("disable_robot", DisableRobot.Request())
                time.sleep(0.1)
                call("enable_robot", EnableRobot.Request(load=2.0))
                time.sleep(0.1)
                call("start_drag", StartDrag.Request())
                time.sleep(0.1)
                call("stop_drag", StopDrag.Request())
                time.sleep(0.1)
                call("modbus_close", ModbusClose.Request(index=0))
                time.sleep(0.1)
                call(
                    "modbus_create",
                    ModbusCreate.Request(ip="127.0.0.1", port=60000, slave_id=9, is_rtu=1),
                )
                time.sleep(0.1)
                call(
                    "set_hold_regs",
                    SetHoldRegs.Request(
                        index=0, addr=1000, count=3, val_tab="0,0,0", val_type="int"
                    ),
                )
                time.sleep(0.1)
                call(
                    "set_hold_regs",
                    SetHoldRegs.Request(
                        index=0, addr=1000, count=3, val_tab="256,0,0", val_type="int"
                    ),
                )
                time.sleep(0.1)
                call("cp", CP.Request(r=100))
                time.sleep(0.1)
                print("Initialization complete.")

            else:
                print("Invalid choice, please try again.")
    finally:
        knode.destroy_node()


# ------------------------------------------------------------------
#  CLI – interactive menu that keeps prompting until you quit
# ------------------------------------------------------------------
def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
    cleanup_motion_node()
    sys.exit(0)

def _main():
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Register cleanup function to run at exit
    atexit.register(cleanup_motion_node)
    
    print("🔧  Pick-and-Place Interactive Menu")
    print(f"📌 Currently using: {USE_VERSION.upper()} ({robot_motion_class.__name__})")
    print("Type sequence name to run, 'list' to show all, 'q' to quit.")
    print("Type 'kin' or 'kinematics' for Dobot tools menu (options 1-10, same as kinemtaics_solutions.py).")
    print("For solution: solution(j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz)\n")

    try:
        while True:
            # Prompt the user
            choice = input("Sequence? ").strip()

            if choice.lower() in ("q", "quit", "exit"):
                print("Bye!")
                break

            if choice.lower() in ("kin", "kinematics", "ktools"):
                try:
                    run_kinematics_tools_menu()
                except KeyboardInterrupt:
                    print("\nInterrupted. Returning to sequence menu.\n")
                except Exception as e:
                    print(f"\nKinematics menu error: {e}\n")
                continue

            if choice.lower() in ("list", "help", "ls", "l"):
                print("\nAvailable sequences:")
                for name in SEQUENCES:
                    print(f"  • {name}")
                print("Also: kin / kinematics -> Dobot service menu (options 1-10).\n")
                continue

            # Check if it's a function call with parameters (e.g., solution(...))
            if "(" in choice and choice.endswith(")"):
                func_name = choice[:choice.index("(")].strip().lower()
                params_str = choice[choice.index("(")+1:-1].strip()
                
                # Special handling for solution function
                if func_name == "solution":
                    try:
                        # Parse comma-separated parameters
                        params = [float(p.strip()) for p in params_str.split(",") if p.strip()]
                        
                        # Ensure we have between 6 and 12 parameters
                        if len(params) < 6:
                            print(f"❌  Not enough parameters. Need at least 6 joint values (j1-j6).\n")
                            continue
                        elif len(params) > 12:
                            print(f"❌  Too many parameters. Max 12 (j1-j6, x,y,z,rx,ry,rz).\n")
                            continue
                        
                        # Pad with zeros if less than 12 parameters
                        while len(params) < 12:
                            params.append(0.0)
                        
                        # Call solution function directly
                        result = solution(*params)
                        print()
                        
                    except ValueError as e:
                        print(f"❌  Invalid parameters: {e}\n")
                        continue
                    except Exception as e:
                        print(f"❌  Error running solution: {e}\n")
                        continue
                else:
                    print(f"❌  Function call syntax only supported for 'solution'.\n")
                    continue
            else:
                # Normal sequence lookup
                choice_lower = choice.lower()
                if choice_lower not in SEQUENCES:
                    print(f"❌  '{choice}' not found. Type 'list' to see all sequences.\n")
                    continue

                # Run the chosen sequence
                try:
                    SEQUENCES[choice_lower]()
                except KeyboardInterrupt:
                    print("\n⏹️  Interrupted. Returning to menu.\n")
                except Exception as e:
                    print(f"\n❌  Error running sequence: {e}\n")
                else:
                    print("\n✅  Finished. Back to menu.\n")
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        # Clean up only once
        if not _cleanup_called:
            cleanup_motion_node()


if __name__ == "__main__":
    _main()
