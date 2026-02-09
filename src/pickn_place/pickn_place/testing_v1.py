#!/usr/bin/env python3
import time
import argparse
import signal
import atexit
import sys
import threading
from typing import Dict, Any, Optional, Union, Tuple

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
from dobot_msgs_v3.srv import ServoJ
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
        rclpy.init(args=None)
        _rclpy_initialized = True
    
    if _global_motion_node is None:
        _global_motion_node = robot_motion_class()
        print(f"✅ Initialized {USE_VERSION} motion node: {robot_motion_class.__name__}")
    
    return _global_motion_node

def run_skill(fn_name: str, *args):
    """
    Efficient run_skill that reuses a persistent motion node.
    Falls back to old approach if there are issues.
    """
    try:
        motion_node = get_motion_node()
        fn = getattr(motion_node, fn_name)
        ok = fn(*args)
        if not ok:
            motion_node.get_logger().error(f"{fn_name}{args} failed – aborting")
            cleanup_motion_node()
            raise RuntimeError(f"Skill {fn_name} failed")
        return ok
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
    
    run_skill("set_speed_factor", SPEED_FAST)
    run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)
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
    def ok(r):
        return r not in (False, None)
    
    run_skill("set_speed_factor", SPEED_FAST)
    
    if not return_back_to_home():
        return False
    
    if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['portafilter_cleaner']['prep_position'])):
        return False
    
    cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
    for i in range(cycles):
        time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
        if not ok(run_skill("move_to", "portafilter_cleaner", 0.26)):
            return False
    
    run_skill("sync")
    
    cleaner_record_result = run_skill("get_machine_position", "portafilter_cleaner")
    if not ok(cleaner_record_result):
        return False
    
    if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep1'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep2'])):
        return False
    
    cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
    for i in range(cycles):
        time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
        if not ok(run_skill("move_to", "espresso_grinder", 0.26)):
            return False
    
    run_skill("sync")
    
    grinder_record_result = run_skill("get_machine_position", "espresso_grinder")
    if not ok(grinder_record_result):
        return False
    
    if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['three_group_espresso_calibration']['prep1'])):
        return False
    
    if not ok(run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)):
        return False
    
    cycles = 15
    for i in range(cycles):
        time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
        if not ok(run_skill("move_to", "three_group_espresso", 0.26)):
            return False
    
    run_skill("sync")
    
    espresso_record_result = run_skill("get_machine_position", "three_group_espresso")
    if not ok(espresso_record_result):
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
        
    except Exception as e:
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
    
    Args:
        j1-j6: Joint values in degrees
        x, y, z: Position offsets in mm (default: 0.0)
        rx, ry, rz: Rotation offsets in degrees (default: 0.0)
    
    Returns:
        Result from inverse_solution with the offset cartesian pose
    """
    print(f"Input joints: [{j1}, {j2}, {j3}, {j4}, {j5}, {j6}]")
    
    # Get current cartesian position from joint values
    pos_result = run_skill("positive_solution", j1, j2, j3, j4, j5, j6)
    
    if not pos_result or not hasattr(pos_result, 'pose'):
        print("Failed to get positive solution")
        return None
    
    # Parse pose string: "{x,y,z,rx,ry,rz,...}"
    try:
        pose_values = [float(v) for v in pos_result.pose.strip("{}").split(",")[:6]]
        current_x, current_y, current_z, current_rx, current_ry, current_rz = pose_values
    except (ValueError, IndexError) as e:
        print(f"Failed to parse pose string: {e}")
        return None
    
    print(f"Current cartesian: x={current_x:.3f}, y={current_y:.3f}, z={current_z:.3f}, rx={current_rx:.3f}, ry={current_ry:.3f}, rz={current_rz:.3f}")
    
    # Apply offsets
    new_x = current_x + x
    new_y = current_y + y
    new_z = current_z + z
    new_rx = current_rx + rx
    new_ry = current_ry + ry
    new_rz = current_rz + rz
    
    print(f"Offsets applied: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    print(f"New cartesian: x={new_x:.3f}, y={new_y:.3f}, z={new_z:.3f}, rx={new_rx:.3f}, ry={new_ry:.3f}, rz={new_rz:.3f}")
    
    # Convert back to joint values
    inv_result = run_skill("inverse_solution", new_x, new_y, new_z, new_rx, new_ry, new_rz)
    
    if inv_result and hasattr(inv_result, 'angle'):
        # Parse angle string: "{j1,j2,j3,j4,j5,j6,...}"
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

def solution_interactive():
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
        
        print("\n" + "="*50)
        return solution(j1, j2, j3, j4, j5, j6, x, y, z, rx, ry, rz)
        
    except ValueError as e:
        print(f"Invalid input: {e}")
        return None
    except KeyboardInterrupt:
        print("\nCancelled")
        return None
    
"""
paper_cups.py

Defines the paper cup handling sequences for coffee service automation.
This module provides comprehensive functions for grabbing, placing, and serving
paper cups in the BARNS coffee automation system, including size-based handling
and staging area management.
"""

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
    if not cups_dict:
        from oms_v1.params import DEFAULT_PAPER_CUP_SIZE
        return DEFAULT_PAPER_CUP_SIZE
    
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
    from oms_v1.params import DEFAULT_PAPER_CUP_SIZE
    return DEFAULT_PAPER_CUP_SIZE

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
    while attempt_count < 5:
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
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)):
            return False
        if attempt_count == 3:
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
    
    if not ok(run_skill("set_gripper_position", 25, 0, 255)):
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
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)):
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
    
    gripper_position = 145
    if not ok(run_skill("moveEE", 0,0,5,0,0,0)):
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
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position3'])):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)):
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
    
    gripper_position = 145
    if not ok(run_skill("moveEE", 0,0,5,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_position)):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position2'])):
        return False
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
        if not ok(run_skill("set_gripper_position", 255,100,255)):
            return False
    else:
        if not ok(run_skill("set_gripper_position", 255,120,255)):
            return False
    
    run_skill("set_speed_factor", 50)
    
    run_skill("moveEE_movJ", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_up'])
    
    if not home(position="west"):
        return False

    if not ok(run_skill("approach_machine", "three_group_espresso", "hot_water")):
        return False
    
    if not ok(run_skill("mount_machine", "three_group_espresso", "hot_water")):
        return False
    
    # run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_move'])
    
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
    
    if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
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

# Global variables to store captured positions during unmount sequence
below_espresso_port: Optional[Tuple[float, ...]] = None
mount_espresso_port: Optional[Tuple[float, ...]] = None
mount_espresso_pose: Optional[Tuple[float, ...]] = None  # Cartesian pose at mount position
approach_pitcher: Optional[Tuple[float, ...]] = None
pick_pitcher: Optional[Tuple[float, ...]] = None

def _normalize_espresso_shot(espresso_dict: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """
    Parse espresso parameters from new JSON format.

    Expected format: {'espresso_shot_single': 1.0} or {'espresso_shot_double': 2.0}

    Rules:
      - 'espresso_shot_single' -> single shot → port_3, positioning_time=5.0, portafilter_tool=single_portafilter
      - 'espresso_shot_double' -> double shot → port_1, positioning_time=5.0, portafilter_tool=double_portafilter
    """
    try:
        if not espresso_dict or not isinstance(espresso_dict, dict):
            return None
        
        # Get the first key from the espresso dictionary
        espresso_key = next(iter(espresso_dict.keys()), None)
        if not espresso_key:
            return None
        
        # Parse the key to determine shot type
        espresso_key_lower = str(espresso_key).lower()
        
        if 'single' in espresso_key_lower:
            return {
                "port": "port_3",
                "positioning_time": 5.0,
                "portafilter_tool": "single_portafilter",
            }
        elif 'double' in espresso_key_lower:
            return {
                "port": "port_1",
                "positioning_time": 5.0,
                "portafilter_tool": "double_portafilter",
            }
        else:
            # Fallback: try to parse as numeric value
            value = espresso_dict.get(espresso_key)
            if value is not None:
                shots = float(value)
                if shots <= 1.0:
                    return {
                        "port": "port_3",
                        "positioning_time": 5.0,
                        "portafilter_tool": "single_portafilter",
                    }
                else:
                    return {
                        "port": "port_1",
                        "positioning_time": 5.0,
                        "portafilter_tool": "double_portafilter",
                    }
    except Exception as e:
        print(f"[WARNING] Error parsing espresso parameters: {e}")
        return None
    
    return None

def unmount(**params) -> bool:
    """
    Unmount portafilter from espresso group for cleaning or grinding.
    """
    global below_espresso_port, mount_espresso_port, mount_espresso_pose
    
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port:
        return False
    
    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return False
    
    if not ok(run_skill("gotoJ_deg", *port_params['home'])):
        return False
    
    if port == 'port_1' or port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'])):
            return False
    
    if not ok(run_skill("mount_machine", "three_group_espresso", port_params['portafilter_number'])):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['grip'])):
        return False
    
    if not ok(run_skill("release_tension")):
        return False
    
    if not ok(run_skill("enforce_rxry")):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("move_portafilter_arc_movJ", -42.0)):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("release_tension")):
        return False
    
    time.sleep(ESPRESSO_DELAYS['orientation_settle'])
    
    mount_espresso_port = run_skill("current_angles")
    if not ok(mount_espresso_port) or not isinstance(mount_espresso_port, (tuple, list)) or len(mount_espresso_port) != 6:
        return False
    
    mount_espresso_pose = run_skill("current_pose")
    if not ok(mount_espresso_pose) or not isinstance(mount_espresso_pose, (tuple, list)) or len(mount_espresso_pose) != 6:
        return False
    
    if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down'])):
        return False
    
    below_espresso_port = run_skill("current_angles")
    if not ok(below_espresso_port) or not isinstance(below_espresso_port, (tuple, list)) or len(below_espresso_port) != 6:
        return False
    
    if not ok(run_skill("gotoJ_deg", *port_params['move_back'])):
        return False
    
    if port in ('port_2', 'port_3'):
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav2'])):
            return False
    
    return True

def grinder(**params) -> bool:
    """
    Grind coffee and tamp portafilter at the grinder station.
    """
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    positioning_time = params.get("positioning_time")
    if positioning_time is None:
        positioning_time = (shot_cfg.get("positioning_time") if shot_cfg else 5.0)
    portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "double_portafilter")
    
    if not port or portafilter_tool not in ('single_portafilter', 'double_portafilter'):
        return False
    
    if port == 'port_1':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
            return False
    
    if not ok(run_skill("approach_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("mount_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("approach_machine", "espresso_grinder", "tamper")):
        return False
    
    time.sleep(positioning_time)
    
    if not ok(run_skill("mount_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("mount_machine", "espresso_grinder", "tamper")):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['release'])):
        return False
    
    run_skill("moveEE_movJ", -50, 50, 50, 15, 0, 0)
    
    # approach_tool_result = run_skill("approach_tool", portafilter_tool)
    # if not ok(approach_tool_result):
    #     fallback_tool = "double_portafilter" if portafilter_tool == "single_portafilter" else "single_portafilter"
    #     if not ok(run_skill("approach_tool", fallback_tool)):
    #         return False
    
    return True
    
def tamper(**params) -> bool:
    """
    Tamp coffee at the tamper station using portafilter tool.
    """
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "single_portafilter")
    
    if portafilter_tool not in ('single_portafilter', 'double_portafilter'):
        return False
    
    # run_skill("sync")
    
    # approach_tool_result = run_skill("approach_tool", portafilter_tool)
    # if not ok(approach_tool_result):
    #     fallback_tool = "double_portafilter" if portafilter_tool == "single_portafilter" else "single_portafilter"
    #     if not ok(run_skill("approach_tool", fallback_tool)):
    #         return False
    #     portafilter_tool = fallback_tool
    
    run_skill("sync")
    
    if not ok(run_skill("grab_tool", portafilter_tool)):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['grip'])):
        return False
    
    run_skill("moveEE_movJ", 0, 0, -5, 0, 0, 0)
    
    if not ok(run_skill("moveEE", 0, 0, 45, 0, 0, 0)):
        return False
    
    if not ok(run_skill("mount_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("approach_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False
    
    return True

def mount(**params) -> bool:
    """
    Mount portafilter back to espresso group after grinding.
    """
    global mount_espresso_pose, below_espresso_port
    
    def ok(r):
        return r not in (False, None)
    
    attempt_count = params.get("attempt_count", 0)
    if attempt_count >= 3:
        return False
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
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
    
    if below_espresso_port is None or not isinstance(below_espresso_port, (tuple, list)) or len(below_espresso_port) != 6:
        return False
    
    if not ok(run_skill("gotoJ_deg", *below_espresso_port)):
        return False
    
    if mount_espresso_port is None or not isinstance(mount_espresso_port, (tuple, list)) or len(mount_espresso_port) != 6:
        return False
    
    if not ok(run_skill("gotoJ_deg", *mount_espresso_port)):
        return False
    
    run_skill("sync")
    
    current_mount_pose = run_skill("current_pose")
    if current_mount_pose is not None and mount_espresso_pose is not None:
        z_original = float(mount_espresso_pose[2])
        z_current = float(current_mount_pose[2])
        z_difference = abs(z_current - z_original)
        
        if z_difference > PORTAFILTER_Z_THRESHOLD_MM:
            if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down'])):
                return False
            
            below_espresso_port = run_skill("current_angles")
            if not ok(below_espresso_port) or not isinstance(below_espresso_port, (tuple, list)) or len(below_espresso_port) != 6:
                return False
            
            if port_params and 'move_back' in port_params:
                if not ok(run_skill("gotoJ_deg", *port_params['move_back'])):
                    return False
            
            if not clean_portafilter(port=port):
                return False
            
            if not mount(port=port, attempt_count=attempt_count + 1):
                return False
            
            return True
        else:
            if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_up'])):
                return False
    
    run_skill("sync")
    
    if not ok(run_skill("enforce_rxry")):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("move_portafilter_arc_movJ", 44.0)):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['release'])):
        return False
    
    if port == 'port_1' or port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'])):
            return False
    
    if not ok(run_skill("gotoJ_deg", *port_params['home'])):
        return False
    
    return True

def pick_espresso_pitcher(**params) -> bool:
    """
    Pick up espresso pitcher for the specified port.
    """
    global approach_pitcher, pick_pitcher
    
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        return False
    
    if port == 'port_1':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_1'])):
            return False
        run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
        run_skill("sync")
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
    elif port == 'port_2':
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_2'])):
            return False
        run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
        run_skill("sync")
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
            return False
    elif port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_3'])):
            return False
        run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
        run_skill("sync")
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
    
    if port == 'port_1' or port == 'port_2':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
            return False
    
    return True    

def pour_espresso_pitcher_cup_station(**params) -> bool:
    """
    Pour milk from espresso pitcher into cup at specified position.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"
    
    run_skill("gotoJ_deg", 103.201965,-21.933174,-150.611664,-10.398072,-23.882843,0.127716)
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return False
    
    if stage == 'stage_1':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos1'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1'])):
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
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2'])):
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
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral3'])):
            return False
    else:  # stage_4
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos4'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral4'])):
            return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return False
    
    run_skill("gotoJ_deg", 103.201965,-21.933174,-150.611664,-10.398072,-23.882843,0.127716)
    run_skill("sync")
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
        return False
    
    return True

def get_hot_water(**params) -> bool:
    """
    Position espresso pitcher under hot water dispenser.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "hot_water")):
        return False
    
    if not ok(run_skill("mount_machine", "three_group_espresso", "hot_water")):
        return False
    
    run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_move'])
    
    return True

def with_hot_water(**params) -> bool:
    """
    Complete hot water dispensing sequence and return to holding position.
    """
    def ok(r):
        return r not in (False, None)
    
    run_skill("set_speed_factor", ESPRESSO_SPEEDS['hot_water_pour'])
    
    if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return False
    
    return True

def return_espresso_pitcher(**params) -> bool:
    """
    Return espresso pitcher to its home position after use.
    """
    global approach_pitcher, pick_pitcher
    
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False
    
    if port == 'port_1':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
    elif port == 'port_2':
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
            return False
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
    elif port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    return True

def return_cleaned_espresso_pitcher(**params) -> bool:
    global approach_pitcher, pick_pitcher
    
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        return False
    
    if port == 'port_1':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_1'])):
            return False
        run_skill("moveEE_movJ", 0,250,10,0,0,0)
        run_skill("moveJ_deg", 0,0,0,0,0,-135)
        run_skill("sync")
        run_skill("moveJ_deg", 0,0,0,0,0,135)
        run_skill("moveEE_movJ", 0,-250,-10,0,0,0)
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
    elif port == 'port_2':
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_2'])):
            return False
        run_skill("moveEE_movJ", 0,0,10,0,0,0)
        run_skill("moveJ_deg", 0,0,0,0,0,-135)
        run_skill("sync")
        run_skill("moveJ_deg", 0,0,0,0,0,135)
        run_skill("moveEE_movJ", 0,0,-10,0,0,0)
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
    elif port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_3'])):
            return False
        run_skill("moveEE_movJ", 0,-250,10,0,0,0)
        run_skill("moveJ_deg", 0,0,0,0,0,-135)
        run_skill("moveJ_deg", 0,0,0,0,0,135)
        run_skill("moveEE_movJ", 0,250,-10,0,0,0)
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    return True


"""
cleaning.py

Defines the cleaning routine for machine ports using parameterized configurations.
This module provides comprehensive portafilter cleaning functionality for the BARNS
coffee automation system, including hard brush and soft brush cleaning sequences
with precise positioning and error handling.
"""

def clean_portafilter(**params) -> bool:
    """
    Very simple cleaning flow:
      1) unmount
      2) grinder home
      3) hard brush: approach → adjust → mount → motion1 → motion2 → retreat_hard
      4) soft brush: approach → mount → motion1 → motion2 → retreat_soft
      5) grinder home
    """
    # Import here to avoid circular import with espresso.py
    # from oms_v1.sequences.espresso import _normalize_espresso_shot
    
    # Normalize from espresso shot if provided
    # New format: {'espresso': {'espresso_shot_double': 2.0}}
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    # Extract and validate port parameter (derived from shot when not explicitly provided)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else DEFAULT_PORT)

    def ok(r):  # minimal check: treat False/None as failure
        return r not in (False, None)

    # # # 1) Unmount - pass all params to maintain espresso context
    # if not ok(unmount(**params)):
    #     return False

    # 2) Go to cleaning station home
    if not ok(run_skill("gotoJ_deg", -35.223076,-2.939468,-128.314575,-47.896400,-73.999352,1.973845)):
        return False

    # 3) Hard brush
    if not ok(run_skill("approach_machine", "portafilter_cleaner", "hard_brush")):
        return False
    if not ok(run_skill("gotoJ_deg", *CLEANING_PARAMS['hard_brush_adjust'])):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "hard_brush")):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", 0,0,50,0,0,0)):
        return False
    if not ok(run_skill("moveEE_movJ", 0,0,-35,-2.5,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_hard'])):
        return False

    # 4) Soft brush
    if not ok(run_skill("approach_machine", "portafilter_cleaner", "soft_brush")):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "soft_brush")):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", 0,0,50,0,0,0)):
        return False
    if not ok(run_skill("moveEE_movJ", 0,0,-55,0,0,0)):
        return False
    if not ok(run_skill("moveEE_movJ", 0,10,0,0,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", 10,0,0,0,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", -20,0,0,0,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", 0,-10,10,-2.5,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE", *CLEANING_PARAMS['retreat_soft'])):
        return False

    # 5) Return to cleaning station home
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    return True
    
"""
milk_frothing.py

Defines the milk frothing sequences for coffee preparation automation.
This module provides comprehensive functions for handling milk frothing operations
in the BARNS coffee automation system, including frother positioning, mounting,
steam activation, milk pouring, and cleaning procedures.
"""
# Global variables to store robot positions during milk frothing operations
# These are used to remember positions between function calls for safe return operations
approach_angles: Optional[Tuple[float, ...]] = None
grab_angles: Optional[Tuple[float, ...]] = None

def get_frother_position(**params) -> bool:
    """
    Calibrate and record the milk frother position for future operations.
    """
    def ok(r):
        return r not in (False, None)
    
    run_skill("set_speed_factor", 100)
    
    if not home(position="north_east"):
        return False
    
    run_skill("move_to", "left_steam_wand", 0.29)
    run_skill("sync")
    run_skill("grab_tool", "left_steam_wand")
    run_skill("set_gripper_position", 255, 255, 255)
    run_skill("gotoJ_deg", -43.788200,-68.169113,-36.695751,-72.909874,-90.922371,6.132053)
    run_skill("set_gripper_position", 255, 0,255)
    
    cycles = 3
    for i in range(cycles):
        time.sleep(CALIBRATION_SETTLE_TIME)
        if not ok(run_skill("move_to", "left_steam_wand", 0.29)):
            return False
    
    if not ok(run_skill("get_machine_position", "left_steam_wand")):
        return False
    
    if not home(position="north_east"):
        return False

    return True

def pick_frother(**params) -> bool:
    """
    Pick up the milk frother for milk frothing operations.
    """
    global approach_angles, grab_angles
    
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area'])):
        return False
    
    if not ok(run_skill("move_to", 'milk_frother_1', 0.29)):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("approach_tool", 'milk_frother_1')):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'])):
        return False
    
    run_skill("sync")
    
    approach_angles = run_skill("current_angles")
    
    run_skill("sync")
    
    if not ok(run_skill("grab_tool", 'milk_frother_1', 100, 100,-5,-10.5)):
        return False
    
    run_skill("sync")
    
    grab_angles = run_skill("current_angles")
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure'])):
        return False
    
    return True
        
def place_frother_milk_station(**params) -> bool:
    """
    Place the milk frother at the milk station safely.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_place'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_approach'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_final'])):
        return False
    if not ok(run_skill("set_gripper_position", 50, MILK_FROTHER_GRIPPER_POSITIONS['place'])):
        return False
    return True

def pick_frother_milk_station(**params) -> bool:
    """
    Pick the milk frother up from the milk station safely.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure'])):
        return False
    if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_pick'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat2'])):
        return False
    return True

def mount_frother(**params) -> bool:
    """
    Mount the milk frother to the steam wand for frothing preparation.
    """
    def ok(r):
        return r not in (False, None)
    
    run_skill("sync")
    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['mount'])
    
    if not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth")):
        return False
    
    if not ok(run_skill("mount_machine", "left_steam_wand", "deep_froth")):
        return False
    
    milk_data = params.get('milk', {})
    volume_ml = next(iter(milk_data.values()), 0) if milk_data else 0
    z_adjustment = MILK_VOLUME_Z_ADJUSTMENT_FACTOR * volume_ml
    run_skill("moveEE_movJ", 0, 20, -z_adjustment, 0, 0, 0)
    
    if not ok(run_skill("sync")):
        return False
    
    return True

def unmount_and_swirl_milk(**params) -> bool:
    """
    Swirl frothed milk in a circular motion for latte art preparation.
    """
    def ok(r):
        return r not in (False, None)
    
    time.sleep(MILK_FROTHING_DELAYS['swirl_delay'])
    
    if not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth")):
        return False
    
    run_skill("sync")
    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['swirl'])
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['intermediate1'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['swirl_pos'])):
        return False
    
    run_skill("sync")
    
    run_skill("move_circle", 
        MILK_SWIRL_CIRCLE_PARAMS['cycles'],
        MILK_SWIRL_CIRCLE_PARAMS['point1_offset'],
        MILK_SWIRL_CIRCLE_PARAMS['point2_offset'],
        MILK_SWIRL_CIRCLE_PARAMS['options'])
    
    return True

def pour_milk_cup_station(**params) -> bool:
    """
    Pour frothed milk into cup at specified stage.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    if stage == '1':
        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['position'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
        run_skill("sync")
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['adjust1'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage1']['move_forward'])
        run_skill("sync")
        time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage1']['move_up'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['position'])
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    elif stage == '2':
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['position'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['adjust1'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage2']['move_forward'])
        time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage2']['move_up'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['position'])
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    elif stage == '3':
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['position'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['adjust1'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage3']['move_forward'])
        time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage3']['move_up'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['position'])
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    else:  # stage == '4'
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['position'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['adjust1'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage4']['move_forward'])
        time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage4']['move_up'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['position'])
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    
    return True

def clean_milk_pitcher(**params) -> bool:
    """
    Perform a cleaning motion for the frother tool.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose3'])):
        return False
    if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['cleaning_motion'])):
        return False
    return True

def return_frother(**params) -> bool:
    """
    Return the frother to its original location using recorded approach/grab angles.
    """
    global approach_angles, grab_angles
    
    def ok(r):
        return r not in (False, None)
    
    if grab_angles is None or approach_angles is None:
        return False
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return3'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return4'])):
        return False
    if not ok(run_skill("gotoJ_deg", *grab_angles)):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['release'])):
        return False
    if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['final_approach'])):
        return False
    if not ok(run_skill("gotoJ_deg", *approach_angles)):
        return False
    home(position="north")
    run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open'])
    return True

"""
plastic_cups.py

Defines the plastic cup handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for grabbing and placing plastic cups
in the BARNS coffee automation system, supporting multiple cup sizes for cold beverages
like slushes, iced drinks, and cold brews.
"""

def _normalize_plastic_cup_size(cups_dict: Any) -> str:
    """
    Universal cup size normalizer for plastic cup operations.
    Accepts BOTH H-codes AND C-codes regardless of prefix.
    Extracts the numeric size and returns standardized format.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz', '16oz')
        
    Examples:
        cup_H9 → '9oz'
        cup_C9 → '9oz'
        cup_h12 → '12oz'
        cup_c16 → '16oz'
    """
    # if not cups_dict:
    #     from oms_v1.params import DEFAULT_PLASTIC_CUP_SIZE
    #     return DEFAULT_PLASTIC_CUP_SIZE
    
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
                
                # Validate and return standardized size
                if size_num in ('7', '9', '12', '16'):
                    return f"{size_num}oz"
    
    # If parsing failed, try the standard normalizers
    # Try plastic first
    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '':
        return result
    
    # Try paper
    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result
    
    # Final fallback
    # from oms_v1.params import DEFAULT_PLASTIC_CUP_SIZE
    # return DEFAULT_PLASTIC_CUP_SIZE

def dispense_plastic_cup(**params) -> bool:
    """
    Dispense a plastic cup of specified size from the dispenser.
    """
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_PLASTIC_CUP_SIZE)
    if not cup_size or not validate_cup_size(cup_size):
        return False
    
    CUP_CONFIG = {
        '16oz': {
            'home': 'west',
            'coords': PLASTIC_CUPS_PARAMS['dispenser']['16oz_coords'],
            'gripper': PLASTIC_CUP_DISPENSE_GRIPPER['16oz'],
            'speed': PLASTIC_CUP_DISPENSE_SPEEDS['16oz'],
            'extract_z': PLASTIC_CUP_EXTRACT_OFFSETS['16oz']['z1'],
            'extract_z2': PLASTIC_CUP_EXTRACT_OFFSETS['16oz']['z2'],
        },
        '12oz': {
            'home': 'west',
            'coords': PLASTIC_CUPS_PARAMS['dispenser']['12oz_coords'],
            'gripper': PLASTIC_CUP_DISPENSE_GRIPPER['12oz'],
            'speed': PLASTIC_CUP_DISPENSE_SPEEDS['12oz'],
            'extract_z': PLASTIC_CUP_EXTRACT_OFFSETS['12oz']['z1'],
            'extract_z2': PLASTIC_CUP_EXTRACT_OFFSETS['12oz']['z2'],
        },
        '9oz': {
            'home': 'south_west',
            'coords': PLASTIC_CUPS_PARAMS['dispenser']['9oz_coords'],
            'gripper': PLASTIC_CUP_DISPENSE_GRIPPER['9oz'],
            'speed': PLASTIC_CUP_DISPENSE_SPEEDS['9oz'],
            'extract_z': PLASTIC_CUP_EXTRACT_OFFSETS['9oz']['z1'],
            'extract_z2': PLASTIC_CUP_EXTRACT_OFFSETS['9oz']['z2'],
        },
        '7oz': {
            'home': 'south_west',
            'coords': PLASTIC_CUPS_PARAMS['dispenser']['7oz_coords'],
            'gripper': PLASTIC_CUP_DISPENSE_GRIPPER['7oz'],
            'speed': PLASTIC_CUP_DISPENSE_SPEEDS['7oz'],
            'extract_z': PLASTIC_CUP_EXTRACT_OFFSETS['7oz']['z1'],
            'extract_z2': PLASTIC_CUP_EXTRACT_OFFSETS['7oz']['z2'],
        }
    }
    
    if cup_size not in CUP_CONFIG:
        return False
    
    config = CUP_CONFIG[cup_size]
    attempt_count = 0
    while attempt_count < 5:
            if cup_size == "16oz":
                home(position=config['home'])
                run_skill("set_gripper_position", 255, 0, 255)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 380.0, -50.0, 0, 0, 0)
                time.sleep(2)
                run_skill("set_gripper_position", GRIPPER_FULL, config['gripper'])
                run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            elif cup_size == "12oz":
                home(position=config['home'])
                run_skill("set_gripper_position", 255, 0, 255)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 328.0, 10.0, 0, 0, 0)
                run_skill("set_gripper_position", 255,130,255)
                run_skill("set_DO", 1, 1)
                time.sleep(1.1)
                run_skill("set_DO", 1, 0)
                run_skill("moveEE", 0, 0, -150, 0, 0, 0)
                run_skill("moveEE", 0, -328.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            if cup_size == "9oz":
                home(position=config['home'])
                run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 25.0, 395.0, -32.0, 0, 0, 0)  
                run_skill("set_gripper_position", GRIPPER_FULL, config['gripper'])
                run_skill("set_speed_factor", config['speed'])
                run_skill("sync")
                run_skill("moveEE", 0, 0.0, config['extract_z'], 0, 0, 0)
                run_skill("set_speed_factor", SPEED_FAST)
                run_skill("sync")
                run_skill("moveEE", 0, 0.0, config['extract_z2'], 0, 0, 0)
                run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            if cup_size == "7oz":
                home(position=config['home'])
                run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 403.0, 5.0, 0, 0, 0)  
                run_skill("set_gripper_position", GRIPPER_FULL, config['gripper'])
                run_skill("set_speed_factor", config['speed'])
                run_skill("sync")
                run_skill("moveEE", 0, 0, config['extract_z'], 0, 0, 0)
                run_skill("set_speed_factor", SPEED_FAST)
                run_skill("sync")
                run_skill("moveEE", 0, 0.0, config['extract_z2'], 0, 0, 0)
                run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            
            run_skill("sync")
            cup_detected = detect_cup_gripper()
            if cup_detected:
                break
            attempt_count += 1
            if attempt_count == 3:
                return False
    
    _set_cup_dispensed()
    return True

def go_to_ice(**params) -> bool:
    """
    Get ice for the specified cup size.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    if not cup_size:
        return False
    
    valid_sizes = ('16oz', '12oz', '9oz', '7oz')
    if cup_size not in valid_sizes:
        return False
    
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position2'])):
        return False

    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    
    run_skill("sync")
    return True

def go_home_with_ice(**params) -> bool:
    """
    Return home with ice-filled cup.
    """
    def ok(r):
        return r not in (False, None)

    if not ok(run_skill("moveEE_movJ", 0,0,5,0,0,0)):
        return False

    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, 145)):
        return False

    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])):
        return False
    
    if not home(position="north"):
        return False
    
    _set_cup_dispensed()
    return True

def place_plastic_cup_station(**params) -> bool:
    """
    Place a plastic cup at specified staging area.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    
    if not cup_size:
        return False
    
    valid_sizes = ('7oz', '9oz', '12oz', '16oz')
    if cup_size not in valid_sizes:
        return False
    
    _check_and_clear_cup_dispensed()
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")
    
    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False
    
    stage_result = False
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
    
    if not ok(stage_result):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)):
        return False
    
    run_skill("set_speed_factor", SPEED_FAST)
    
    if not ok(run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['place_return_up'])):
        return False
    if not home(position="east"):
        return False
    
    return True

def pick_plastic_cup_station(**params) -> bool:
    """
    Pick up a plastic cup from a specific stage and add ice to it.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    
    if not cup_size:
        return False
    
    valid_sizes = ('7oz', '9oz', '12oz', '16oz')
    if cup_size not in valid_sizes:
        return False
    
    stage_positions = {
        "1": PLASTIC_CUPS_PARAMS['staging']['pickup_1'],
        "2": PLASTIC_CUPS_PARAMS['staging']['pickup_2'],
        "3": PLASTIC_CUPS_PARAMS['staging']['pickup_3'],
        "4": PLASTIC_CUPS_PARAMS['staging']['pickup_4']
    }
    
    gripper_positions = {
        "7oz": 145,
        "9oz": 125, 
        "12oz": 140,
        "16oz": 118
    }
    
    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return False
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    
    if not ok(run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['pickup_down'])):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")
    
    if cup_position == 3 or cup_position == 4:
        if not home(position="south_east"):
            return False
    
    if not home(position="east"):
        return False
    if not home(position="north_east"):
        return False
    
    return True

def place_plastic_cup_sauces(**params) -> bool:
    """
    Place the plastic cup at the sauces station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    
    if not cup_size:
        return False
    
    valid_sizes = ("7oz", "9oz", "12oz", "16oz")
    if cup_size not in valid_sizes:
        return False
    
    after_dispense = _check_and_clear_cup_dispensed()
    
    if not after_dispense:
        run_skill("set_speed_factor", SPEED_NORMAL)
        run_skill("sync")
    
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position2'])):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    return True

def pick_plastic_cup_sauces(**params) -> bool:
    """
    Pick the plastic cup from the sauces station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    
    if not cup_size:
        return False
    
    valid_sizes = ("7oz", "9oz", "12oz", "16oz")
    if cup_size not in valid_sizes:
        return False
    
    _check_and_clear_cup_dispensed()
    
    gripper_positions = {
        "7oz": 145,
        "9oz": 145,
        "12oz": 150,
        "16oz": PLASTIC_CUP_GRIPPER_POSITIONS['16oz'],
    }
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")
    
    if not ok(run_skill("moveEE", 0,0,5,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1'])):
        return False
    
    return True

def place_plastic_cup_milk(**params) -> bool:
    """
    Place the plastic cup at the milk station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    
    if not cup_size:
        return False
    
    valid_sizes = ("7oz", "9oz", "12oz", "16oz")
    if cup_size not in valid_sizes:
        return False
    
    after_dispense = _check_and_clear_cup_dispensed()
    
    if not after_dispense:
        run_skill("set_speed_factor", SPEED_NORMAL)
        run_skill("sync")
    
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position2'])):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    return True

def pick_plastic_cup_milk(**params) -> bool:
    """
    Pick the plastic cup from the milk station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    
    if not cup_size:
        return False
    
    valid_sizes = ("7oz", "9oz", "12oz", "16oz")
    if cup_size not in valid_sizes:
        return False
    
    _check_and_clear_cup_dispensed()
    
    gripper_positions = {
        "7oz": 145,
        "9oz": 145,
        "12oz": 150,
        "16oz": PLASTIC_CUP_GRIPPER_POSITIONS['16oz'],
    }
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")
    
    if not ok(run_skill("moveEE", 0,0,5,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
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
    """
    Universal cup size normalizer for slush operations.
    Accepts BOTH H-codes AND C-codes regardless of prefix.
    Extracts the numeric size and returns standardized format.
    
    Args:
        cups_dict: Dictionary containing cup information
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz', '16oz')
        
    Examples:
        cup_H9 → '9oz'
        cup_C9 → '9oz'
        cup_h12 → '12oz'
        cup_c16 → '16oz'
    """
    if not cups_dict:
        return DEFAULT_PLASTIC_CUP_SIZE
    
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
                
                # Validate and return standardized size
                if size_num in ('7', '9', '12', '16'):
                    return f"{size_num}oz"
    
    # If parsing failed, try the standard normalizers
    # Try plastic first
    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '':
        return result
    
    # Try paper
    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result
    
    # Final fallback
    return DEFAULT_PLASTIC_CUP_SIZE

def get_slush(**params) -> bool:
    """
    Get slush from specified dispenser and prepare for serving.
    """
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
            if "chocolate" in premix_name.lower() or "choco" in premix_name.lower():
                dispenser = "2"
            else:
                dispenser = "1"
        else:
            dispenser = "1"
    
    valid_cup_sizes = ("7oz", "9oz", "12oz", "16oz")
    valid_dispensers = ("1", "2")
    
    if cup_size not in valid_cup_sizes or dispenser not in valid_dispensers:
        return False
    
    cup_code = f"cup_C{cup_size.replace('oz', '')}"
    if not dispense_plastic_cup(cups={cup_code: 1.0}):
        return False
    
    if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['intermediate'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])):
        return False
    
    if dispenser == "1":
        dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['dispense'])
    else:
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['intermediate'])):
            return False
        dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['dispense'])
    
    if not ok(dispenser_result):
        return False
    
    return True

def place_slush(**params) -> bool:
    """
    Place slush-filled cup at specified staging area after dispensing.
    """
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
            if "chocolate" in premix_name.lower() or "choco" in premix_name.lower():
                dispenser = "2"
            else:
                dispenser = "1"
        else:
            dispenser = "1"
    
    valid_cup_sizes = ("7oz", "9oz", "12oz", "16oz")
    valid_dispensers = ("1", "2")
    
    if cup_size not in valid_cup_sizes or dispenser not in valid_dispensers:
        return False
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    
    if dispenser == "1":
        retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['retreat'])
    else:
        retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['retreat'])
    
    if not ok(retreat_result):
        return False
    
    if not home(position="north"):
        return False
    
    cup_code = f"cup_C{cup_size.replace('oz', '')}"
    if not place_plastic_cup_station(position={'cup_position': int(stage)}, cups={cup_code: 1.0}):
        return False
    
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

def milk(**params):
    """
    Complete milk frothing and pouring sequence.
    
    Workflow:
    1. Get frother position and pick up frother
    2. Mount frother to steam wand
    3. Froth milk for specified duration
    4. Pour frothed milk at positions 1, 2, 3, 4
    5. Return frother and clean steam wand
    
    Args:
        duration (float): Frothing duration in seconds (default: 10)
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
                        Note: This parameter is ignored as the function loops through all positions 1-4
    
    Returns:
        bool: True if milk preparation completed successfully, False otherwise
    """
    duration = params.get("duration", 10)
    
    print(f"🥛 Starting milk preparation sequence (duration: {duration}s, pouring at positions 1-4)...")
    
    try:
        # Step 1: Get frother position
        print("📍 Getting frother position...")
        if not get_frother_position():
            print("[ERROR] Failed to get frother position")
            return False
        
        # Step 2: Pick up frother
        print("🤏 Picking up frother...")
        if not pick_frother():
            print("[ERROR] Failed to pick frother")
            return False

        # Step 3: Place frother at milk station
        print("📍 Placing frother at milk station...")
        if not place_frother_milk_station():
            print("[ERROR] Failed to place frother at milk station")
            return False
        
        # Step 4: Pick frother from milk station
        print("🤏 Picking frother from milk station...")
        if not pick_frother_milk_station():
            print("[ERROR] Failed to pick frother from milk station")
            return False

        run_skill("sync")

        # Step 5: Mount frother to steam wand
        print("🔧 Mounting frother to steam wand...")
        if not mount_frother():
            print("[ERROR] Failed to mount frother")
            return False
        
        # Step 6: Unmount and swirl milk
        print("🥛 Unmounting and swirling milk...")
        if not unmount_and_swirl_milk():
            print("[ERROR] Failed to unmount and swirl milk")
            return False

        # Step 7: Pour milk at positions 1, 2, 3, 4
        for cup_position in [1, 2, 3, 4]:
            print(f"🥛 Pouring milk at position {cup_position}...")
            if not pour_milk(position={'cup_position': cup_position}):
                print(f"[ERROR] Failed to pour milk at position {cup_position}")
                return False
        
        # Step 8: Clean frother
        print("🧹 Cleaning frother...")
        if not clean_milk_pitcher():
            print("[ERROR] Failed to clean frother")
            return False
        
        # Step 10: Return frother
        print("🔄 Returning frother...")
        if not return_frother():
            print("[ERROR] Failed to return frother")
            return False
        
        print("✅ Milk preparation completed successfully for all positions!")
        return True
        
    except Exception as e:
        print(f"[ERROR] Milk preparation failed with exception: {e}")
        return False

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
        run_skill("move_to", "three_group_espresso", 0.26)
    run_skill("get_machine_position", "three_group_espresso")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    run_skill("gotoJ_deg", -28.755102,-16.240370,-145.875793,-15.083625,-114.523071,0.660176)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", -16.564388,-27.443169,-121.383263,-29.049856,-103.556274,-0.997243)#run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", -28.755102,-16.240370,-145.875793,-15.083625,-114.523071,0.660176)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    # # input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    # input()
    # run_skill("gotoJ_deg", 24.911945,-21.074497,-135.128052,-22.276201,-61.762913,0)#run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    # input()
    # run_skill("gotoJ_deg", *ESPRESSO_HOME)#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    # input()
    run_skill("gotoJ_deg", 78.049049,-11.560322,-133.106522,-29.895899,-7.475047,-5.520638)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", 57.893150,-29.455135,-118.186241,-29.553265,-27.351288,-1.391830)#run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", 78.049049,-11.560322,-133.106522,-29.895899,-7.475047,-5.520638)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    # run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    # # input()
    # # run_skill("gotoJ_deg",16.182545,-45.977921,-119.918640,-12.260736,-73.420769,0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    # # input()
    # run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    # run_skill("gotoJ_deg", -22.378635,-25.306602,-137.820709,-24.454021,-108.835793,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
    # input()
    # run_skill("gotoJ_deg", -14.879210,-48.115944,-114.257034,-13.916231,-101.903206,0.0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
    # input()
    # run_skill("gotoJ_deg", -22.378635,-25.306602,-137.820709,-24.454021,-108.835793,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
    # run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    # run_skill("gotoJ_deg",72.861008,-37.369293,-141.719101,8.783054,-15.383393,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
    # input()
    # run_skill("gotoJ_deg",48.463070,-49.133530,-111.629639,-15.658930,-40.472858,0.0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3", True)
    # input()
    # run_skill("gotoJ_deg",72.861008,-37.369293,-141.719101,8.783054,-15.383393,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
    # run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    # run_skill("gotoJ_deg", *ESPRESSO_HOME)
    # run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    # run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
    # for i in range(5):
    #     time.sleep(1.0)
    #     run_skill("move_to", "espresso_grinder", 0.26)
    # run_skill("get_machine_position", "espresso_grinder")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    # input()
    # run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    # run_skill("gotoJ_deg",-45.427513,-58.889004,-102.710777,-19.257256,-101.516602,0.0)#approach the grinder#run_skill("approach_machine", "espresso_grinder", "grinder", True)
    # input()
    # run_skill("gotoJ_deg", -43.257053,-67.649300,-83.081116,-27.657471,-97.851700,0.0) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    # input()
    # run_skill("gotoJ_deg",-44.728165,-67.766884,-72.699600,-43.691830,-102.070465,0.0)#touch the button#run_skill("approach_machine", "espresso_grinder", "tamper", True)
    # input()
    # run_skill("gotoJ_deg", -43.257053,-67.649300,-83.081116,-27.657471,-97.851700,0.0) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    # input()
    # run_skill("gotoJ_deg", -43.257011,-71.458015,-81.109985,-25.820198,-97.854370,0.0) #in the tamper#run_skill("mount_machine", "espresso_grinder", "tamper", True)
    # input()
    # run_skill("gotoJ_deg", -43.257053,-67.649300,-83.081116,-27.657471,-97.851700,0.0) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    # run_skill("gotoJ_deg",-45.427513,-58.889004,-102.710777,-19.257256,-101.516602,0.0)#approach the grinder#run_skill("approach_machine", "espresso_grinder", "grinder", True)
    # run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
    # run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    # for i in range(5):
    #     time.sleep(1.0)
    #     run_skill("move_to", "portafilter_cleaner", 0.26)
    # run_skill("get_machine_position", "portafilter_cleaner")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    # input()
    # run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    # run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
    # run_skill("gotoJ_deg", -79.218399,-0.662163,-126.990761,-53.961704,-81.920143,-1.976189)# run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
    # input()
    # # run_skill("moveEE_movJ", -88, 0, 0, 0, 0, -135)
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

def milk_training(**params):
    home(position="north_east")
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "left_steam_wand", 0.26)
    run_skill("get_machine_position", "left_steam_wand")#42.507626,8.389988,-122.460335,-74.151726,-59.384083,4.208460
    input()
    run_skill("gotoJ_deg", -45.731773,-48.647770,-90.804688,-52.454651,-90.259727,10.531549)
    run_skill("gotoJ_deg",-47.825047,-86.633034,-28.265034,-67.004753,-96.852402,10.037631)#run_skill("approach_machine", "left_steam_wand", "deep_froth", True)
    input()
    run_skill("gotoJ_deg", -47.823650,-85.286758,-16.991077,-70.719292,-92.739357,16.378487)#run_skill("mount_machine", "left_steam_wand", "deep_froth", True)

def test(**params):
    dispense_paper_cup_station(cups={'cup_H12': 1.0}, position={'cup_position': 1})
    dispense_paper_cup_station(cups={'cup_H12': 1.0}, position={'cup_position': 2})
    dispense_paper_cup_station(cups={'cup_H12': 1.0}, position={'cup_position': 3})
    dispense_paper_cup_station(cups={'cup_H12': 1.0}, position={'cup_position': 4})
    pick_cup_for_hot_water(cups={'cup_H12': 1.0}, position={'cup_position': 1})
    return_cup_with_hot_water(cups={'cup_H12': 1.0}, position={'cup_position': 1})
    pick_cup_for_hot_water(cups={'cup_H12': 1.0}, position={'cup_position': 2})
    return_cup_with_hot_water(cups={'cup_H12': 1.0}, position={'cup_position': 2})
    pick_cup_for_hot_water(cups={'cup_H12': 1.0}, position={'cup_position': 3})
    return_cup_with_hot_water(cups={'cup_H12': 1.0}, position={'cup_position': 3})
    pick_cup_for_hot_water(cups={'cup_H12': 1.0}, position={'cup_position': 4})
    return_cup_with_hot_water(cups={'cup_H12': 1.0}, position={'cup_position': 4})


def test_arm1(**params):
    run_skill("gotoJ_deg", 112.5, 30, -130, -90,  -90,  0)

def test_arm2(**params) -> bool:
    try:
        # Cup 1 - Stage 1
        if not dispense_plastic_cup(cup_size="12oz"):
            return False
        if not go_to_ice(cup_size="12oz"):
            return False
        if not go_home_with_ice():
            return False
        if not place_plastic_cup_sauces(cups={'cup_C12': 1.0}):
            return False
        if not pick_plastic_cup_sauces(cups={'cup_C12': 1.0}):
            return False
        if not place_plastic_cup_milk(cups={'cup_C12': 1.0}):
            return False
        if not pick_plastic_cup_milk(cups={'cup_C12': 1.0}):
            return False
        if not place_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C12': 1.0}):
            return False
        
        # Cup 2 - Stage 2
        if not dispense_plastic_cup(cup_size="12oz"):
            return False
        if not go_to_ice(cup_size="12oz"):
            return False
        if not go_home_with_ice():
            return False
        if not place_plastic_cup_milk(cups={'cup_C12': 1.0}):
            return False
        if not pick_plastic_cup_milk(cups={'cup_C12': 1.0}):
            return False
        if not place_plastic_cup_station(position={'cup_position': 2}, cups={'cup_C12': 1.0}):
            return False
        
        # Cup 3 - Stage 3
        if not dispense_plastic_cup(cup_size="12oz"):
            return False
        if not go_to_ice(cup_size="12oz"):
            return False
        if not go_home_with_ice():
            return False
        if not place_plastic_cup_sauces(cups={'cup_C12': 1.0}):
            return False
        if not pick_plastic_cup_sauces(cups={'cup_C12': 1.0}):
            return False
        if not place_plastic_cup_station(position={'cup_position': 3}, cups={'cup_C12': 1.0}):
            return False
        
        # Cup 4 - Stage 4
        if not dispense_plastic_cup(cup_size="12oz"):
            return False
        if not go_to_ice(cup_size="12oz"):
            return False
        if not go_home_with_ice():
            return False
        if not place_plastic_cup_milk(cups={'cup_C12': 1.0}):
            return False
        if not pick_plastic_cup_milk(cups={'cup_C12': 1.0}):
            return False
        if not place_plastic_cup_sauces(cups={'cup_C12': 1.0}):
            return False
        if not pick_plastic_cup_sauces(cups={'cup_C12': 1.0}):
            return False
        if not place_plastic_cup_station(position={'cup_position': 4}, cups={'cup_C12': 1.0}):
            return False
        
        print("✅ test_arm2 completed successfully!")
        return True
        
    except Exception as e:
        print(f"[ERROR] test_arm2 failed with exception: {e}")
        return False


def hello(**params):
    start_time = time.perf_counter()
    for i in range(1):
        run_skill("gotoJ_deg", 180.145874,31.742138,-120.866173,19.811152,-92.696129,10.536575) #P108
        for i in range(1):
            run_skill("gotoJ_deg", 210.743909,27.533575,-145.620954,39.206884,-92.263737,14.811911) #P109
            run_skill("gotoJ_deg", 180.145874,31.742138,-120.866173,19.811152,-92.696129,10.536575) #P108
            run_skill("gotoJ_deg", 164.649656,15.710394,-137.946233,38.069673,-92.976630,10.570545) #P110
            run_skill("gotoJ_deg", 180.145874,31.742138,-120.866173,19.811152,-92.696129,10.536575) #P108
        for i in range(1):
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
    "milk": lambda: milk(),
    "slushie": lambda: slushie(),
    
    # ═══════════════════════════════════════════════════════════════
    # 🥤 PAPER CUP OPERATIONS
    # ═══════════════════════════════════════════════════════════════
    "grab_paper_cup_7oz": lambda: grab_paper_cup(size="7oz"),
    "grab_paper_cup_9oz": lambda: grab_paper_cup(size="9oz"),
    "grab_paper_cup_12oz": lambda: grab_paper_cup(size="12oz"),
    "place_paper_cup_stage_1": lambda: place_paper_cup(position={'cup_position': 1}),
    "place_paper_cup_stage_2": lambda: place_paper_cup(position={'cup_position': 2}),
    "place_paper_cup_stage_3": lambda: place_paper_cup(position={'cup_position': 3}),
    "place_paper_cup_stage_4": lambda: place_paper_cup(position={'cup_position': 4}),
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
    "get_slush_d1_s1": lambda: get_slush(dispenser="1", position={'cup_position': 1}, cup_size="16oz"),
    "get_slush_d1_s2": lambda: get_slush(dispenser="1", position={'cup_position': 2}, cup_size="16oz"),
    "get_slush_d1_s3": lambda: get_slush(dispenser="1", position={'cup_position': 3}, cup_size="16oz"),
    "get_slush_d1_s4": lambda: get_slush(dispenser="1", position={'cup_position': 4}, cup_size="16oz"),
    "get_slush_d2_s1": lambda: get_slush(dispenser="2", position={'cup_position': 1}, cup_size="16oz"),
    "get_slush_d2_s2": lambda: get_slush(dispenser="2", position={'cup_position': 2}, cup_size="16oz"),
    "get_slush_d2_s3": lambda: get_slush(dispenser="2", position={'cup_position': 3}, cup_size="16oz"),
    "get_slush_d2_s4": lambda: get_slush(dispenser="2", position={'cup_position': 4}, cup_size="16oz"),
    "place_slush_d1_s1": lambda: place_slush(dispenser="1", position={'cup_position': 1}, cup_size="16oz"),
    "place_slush_d1_s2": lambda: place_slush(dispenser="1", position={'cup_position': 2}, cup_size="16oz"),
    "place_slush_d1_s3": lambda: place_slush(dispenser="1", position={'cup_position': 3}, cup_size="16oz"),
    "place_slush_d1_s4": lambda: place_slush(dispenser="1", position={'cup_position': 4}, cup_size="16oz"),
    "place_slush_d2_s1": lambda: place_slush(dispenser="2", position={'cup_position': 1}, cup_size="16oz"),
    "place_slush_d2_s2": lambda: place_slush(dispenser="2", position={'cup_position': 2}, cup_size="16oz"),
    "place_slush_d2_s3": lambda: place_slush(dispenser="2", position={'cup_position': 3}, cup_size="16oz"),
    "place_slush_d2_s4": lambda: place_slush(dispenser="2", position={'cup_position': 4}, cup_size="16oz"),
    
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
    "test_arm2": lambda: test_arm2(),
    "test_arm1": lambda: test_arm1(),
    "milk_1": lambda: milk_1(),
    "milk_2": lambda: milk_2(),
    "milk_3": lambda: milk_3(),
    "milk_4": lambda: milk_4(),
    
    # ═══════════════════════════════════════════════════════════════
    # 📸 COMPUTER VISION & DETECTION
    # ═══════════════════════════════════════════════════════════════
    "detect_cup_gripper": lambda: detect_cup_gripper(),
    
    # ═══════════════════════════════════════════════════════════════
    # 🔧 UTILITY FUNCTIONS
    # ═══════════════════════════════════════════════════════════════
    "show_version_info": lambda: show_version_info(),
    "switch_version": lambda: switch_version(),
}

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
    print("For solution: solution(j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz)\n")

    try:
        while True:
            # Prompt the user
            choice = input("Sequence? ").strip()

            if choice.lower() in ("q", "quit", "exit"):
                print("Bye!")
                break

            if choice.lower() in ("list", "help", "ls", "l"):
                print("\nAvailable sequences:")
                for name in SEQUENCES:
                    print(f"  • {name}")
                print()
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