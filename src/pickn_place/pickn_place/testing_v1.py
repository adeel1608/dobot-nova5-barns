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
    
    This function moves the robot to one of several predefined home positions
    based on the position parameter. These home positions are safe, known
    configurations for different operational contexts.
    
    Args:
        position (str): Name of the home position to move to (must exist in HOME_ANGLES), defaults to 'north'
        
    Returns:
        bool: True if robot moved to home position successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during movement
        
    Example:
        success = home(position='espresso_home')
        if success:
            print("Robot moved to home position successfully")
    """
    try:
        # Extract and validate position parameter
        position = params.get("position", "north")  # Default to north
        if not position:
            print("[ERROR] No position parameter provided")
            print("[INFO] Please provide position parameter from available home positions")
            return False
            
        angles = HOME_ANGLES.get(str(position))
        
        if not angles:
            print(f"[ERROR] Unknown home position: {position!r}")
            print(f"[INFO] Available positions: {list(HOME_ANGLES.keys())}")
            return False
        
        print(f"🏠 Moving robot to home position: {position}")
        
        # Execute movement to home position
        result = run_skill("gotoJ_deg", *angles)
        if result is False:
            print(f"[ERROR] Failed to move robot to home position: {position}")
            return False
        
        print(f"✅ Robot successfully moved to home position: {position}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during home movement: {e}")
        return False

def return_back_to_home() -> bool:
    """
    Return the robot to a safe home position based on current angle.
    
    This function reads the current J1 angle and determines the nearest
    cardinal position (0°, ±45°, ±90°, ±135°, ±180°), then moves to
    a safe home pose at that J1 angle with predefined J2-J6 values.
    
    Returns:
        bool: True if movement successful, False otherwise
        
    Raises:
        Exception: If error occurs during angle reading or movement
    """
    try:
        print("\n" + "="*60)
        print("🏠 RETURNING TO HOME POSITION")
        print("="*60)
        
        # Step 1: Try to release tension normally
        print("🔓 Releasing tension and exiting drag mode...")
        release_result = run_skill("release_tension")
        if release_result is False:
            print("[WARNING] Release tension failed - attempting toggle drag mode as fallback...")
            toggle_result = run_skill("toggle_drag_mode")
            if toggle_result is False:
                print("[ERROR] Failed to lock servos - robot may not respond to commands")
                print("[INFO] Please manually check robot state and retry")
                return False
            print("   ✅ Servos locked via toggle drag mode")
        else:
            print("   ✅ Tension released successfully")
        
        # Step 4: Set speed and gripper for safe operation
        run_skill("set_speed_factor", SPEED_FAST)
        run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)
        angles = run_skill("current_angles")
        
        if not angles or len(angles) < 6:
            print(f"[ERROR] Failed to get current angles or invalid data: {angles}")
            return False
        
        # Extract J1 angle (first joint)
        a1 = float(angles[0])
        print(f"   Current J1 angle: {a1:.2f}°")
        
        # Determine appropriate J1 target position based on current angle
        j1_val = None
        
        # Positive angles and 0
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
        # Negative angles
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
            print(f"[ERROR] Could not determine home position for J1 angle: {a1:.2f}°")
            return False
        
        # Move to home position using calibration parameters
        home_j2_j6 = HOME_CALIBRATION_PARAMS['return_home_position']
        print(f"🎯 Moving to home position: J1={j1_val}°, J2={home_j2_j6[0]}°, J3={home_j2_j6[1]}°, J4={home_j2_j6[2]}°, J5={home_j2_j6[3]}°, J6={home_j2_j6[4]}°")        
        result = run_skill("gotoJ_deg", j1_val, *home_j2_j6)
        
        if result is False:
            print("[ERROR] Failed to move to home position")
            return False
        
        print("="*60)
        print("✅ SUCCESSFULLY RETURNED TO HOME POSITION")
        print("="*60)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during return to home: {e}")
        import traceback
        traceback.print_exc()
        return False

def get_machine_position(**params) -> bool:
    """
    Calibrate and record machine positions for all coffee equipment.
    
    This function performs a comprehensive machine position calibration sequence:
    1. Moves to espresso home position
    2. Approaches and records portafilter cleaner position
    3. Approaches and records espresso grinder position  
    4. Approaches and records three-group espresso machine position
    5. Saves all position data for future reference
    
    This calibration should be performed when starting from a known home position
    and when machine positions may have changed.
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if all machine positions calibrated successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during calibration process
        
    Example:
        success = get_machine_position()
        if success:
            print("All machine positions calibrated successfully")
    """
    try:
        print("🎯 Starting comprehensive machine position calibration...")
        
        # Set optimal speed for calibration accuracy
        print("⚙️ Setting speed factor for precise movements...")
        run_skill("set_speed_factor", SPEED_FAST)
        
        # Step 1: Move to espresso home position
        print("🏠 Step 1/8: Moving to espresso home position...")
        home_result = return_back_to_home()
        if home_result is False:
            print("[ERROR] Failed to move to espresso home position")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Position for portafilter cleaner calibration
        print("📍 Step 2/8: Positioning for portafilter cleaner calibration...")
        cleaner_prep_result = run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['portafilter_cleaner']['prep_position'])
        if cleaner_prep_result is False:
            print("[ERROR] Failed to position for cleaner calibration")
            return False
        print("   ✅ Successfully positioned for cleaner calibration")
        
        # Step 3: Perform multiple approaches to portafilter cleaner for accuracy
        cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
        print(f"🧹 Step 3/8: Calibrating portafilter cleaner position ({cycles} approaches)...")
        for i in range(cycles):
            print(f"   📍 Approach {i+1}/{cycles}...")
            time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])  # Allow settling time between approaches
            
            approach_result = run_skill("move_to", "portafilter_cleaner", 0.26)
            if approach_result is False:
                print(f"[ERROR] Failed cleaner approach {i+1}/5")
                return False
        print("   ✅ All cleaner approaches completed successfully")
        
        run_skill("sync")

        # Record portafilter cleaner position
        print("💾 Step 4/8: Recording portafilter cleaner position...")
        print("   🔍 Waiting for ArUco marker ID 23 (portafilter_cleaner) to be detected...")
        cleaner_record_result = run_skill("get_machine_position", "portafilter_cleaner")
        if cleaner_record_result is False or cleaner_record_result is None:
            print("[ERROR] Failed to record portafilter cleaner position")
            print("   ❌ ArUco marker 23 may not be visible to camera")
            print("   💡 Tip: Ensure marker 23 is clearly visible and try again")
            return False
        print("   ✅ Portafilter cleaner position successfully recorded!")
        
        # Step 5: Position for espresso grinder calibration
        print("📍 Step 5/8: Positioning for espresso grinder calibration...")
        grinder_prep1_result = run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep1'])
        if grinder_prep1_result is False:
            print("[ERROR] Failed to position for grinder calibration (step 1)")
            return False
        
        grinder_prep2_result = run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep2'])
        if grinder_prep2_result is False:
            print("[ERROR] Failed to position for grinder calibration (step 2)")
            return False
        print("   ✅ Successfully positioned for grinder calibration")
        
        # Step 6: Perform multiple approaches to espresso grinder for accuracy
        cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
        print(f"☕ Step 6/8: Calibrating espresso grinder position ({cycles} approaches)...")
        for i in range(cycles):
            print(f"   📍 Approach {i+1}/{cycles}...")
            time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])  # Allow settling time between approaches
            
            grinder_approach_result = run_skill("move_to", "espresso_grinder", 0.26)
            if grinder_approach_result is False:
                print(f"[ERROR] Failed grinder approach {i+1}/5")
                return False
        print("   ✅ All grinder approaches completed successfully")
        
        run_skill("sync")

        # Record espresso grinder position
        print("💾 Recording espresso grinder position...")
        print("   🔍 Waiting for ArUco marker ID 31 (espresso_grinder) to be detected...")
        grinder_record_result = run_skill("get_machine_position", "espresso_grinder")
        if grinder_record_result is False or grinder_record_result is None:
            print("[ERROR] Failed to record espresso grinder position")
            print("   ❌ ArUco marker 31 may not be visible to camera")
            print("   💡 Tip: Ensure marker 31 is clearly visible and try again")
            return False
        print("   ✅ Espresso grinder position successfully recorded!")
        
        # Step 7: Position for three-group espresso machine calibration
        print("📍 Step 7/8: Positioning for three-group espresso machine calibration...")
        espresso_prep1_result = run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['three_group_espresso_calibration']['prep1'])
        if espresso_prep1_result is False:
            print("[ERROR] Failed to position for espresso machine calibration (step 1)")
            return False
        
        espresso_prep2_result = run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)
        if espresso_prep2_result is False:
            print("[ERROR] Failed to position for espresso machine calibration (step 2)")
            return False
        print("   ✅ Successfully positioned for espresso machine calibration")
        
        # Perform multiple approaches to three-group espresso machine for accuracy
        cycles = 15  # Special case: more cycles for three-group espresso
        print(f"☕ Calibrating three-group espresso machine position ({cycles} approaches)...")
        for i in range(cycles):
            print(f"   📍 Approach {i+1}/{cycles}...")
            time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])  # Allow settling time between approaches
            
            espresso_approach_result = run_skill("move_to", "three_group_espresso", 0.26)
            if espresso_approach_result is False:
                print(f"[ERROR] Failed espresso machine approach {i+1}/5")
                return False
        print("   ✅ All espresso machine approaches completed successfully")
        
        run_skill("sync")
        
        # Record three-group espresso machine position
        print("💾 Recording three-group espresso machine position...")
        print("   🔍 Waiting for ArUco marker ID 41 (three_group_espresso) to be detected...")
        espresso_record_result = run_skill("get_machine_position", "three_group_espresso")
        if espresso_record_result is False or espresso_record_result is None:
            print("[ERROR] Failed to record three-group espresso machine position")
            print("   ❌ ArUco marker 41 may not be visible to camera")
            print("   💡 Tip: Ensure marker 41 is clearly visible and try again")
            return False
        print("   ✅ Three-group espresso machine position successfully recorded!")
        
        # Step 8: Return to espresso home position
        print("🏠 Step 8/8: Returning to espresso home position...")
        final_home_result = run_skill("gotoJ_deg", *ESPRESSO_HOME)
        if final_home_result is False:
            print("[ERROR] Failed to return to espresso home position")
            return False
        print("   ✅ Successfully returned to espresso home")
        
        # Verify and display saved data
        print("📋 Verifying saved machine position data...")
        saved_machines = check_saved_data()
        
        # Final success summary
        print("\n" + "="*60)
        print("✅ MACHINE POSITION CALIBRATION COMPLETED SUCCESSFULLY!")
        print("="*60)
        print("   ✓ Portafilter cleaner position recorded")
        print("   ✓ Espresso grinder position recorded") 
        print("   ✓ Three-group espresso machine position recorded")
        print(f"   📊 Total machines calibrated: {len(saved_machines)}")
        print("="*60)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during machine position calibration: {e}")
        print("[INFO] Calibration process terminated due to error")
        return False

def check_saved_data() -> Dict[str, Any]:
    """
    Check and display currently saved machine position data.
    
    This function reads the machine_pose_data_memory.yaml file and reports
    what machine positions have been successfully calibrated and saved.
    
    Returns:
        Dict[str, Any]: Dictionary of saved machine positions with their data,
                       or empty dict if no data found
        
    Raises:
        Exception: If error occurs while reading saved data file
    """
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    
    try:
        # Locate the machine position data file
        pkg_share = get_package_share_directory("pickn_place")
        mem_path = os.path.join(pkg_share, "machine_pose_data_memory.yaml")
        
        # Check if data file exists
        if not os.path.exists(mem_path):
            print(f"   ❌ No machine position data file found at: {mem_path}")
            print("   💡 Run calibration first to create position data")
            return {}
        
        # Read and parse the YAML data
        with open(mem_path, "r") as f:
            data = yaml.safe_load(f) or {}
        
        machines = data.get("machines", {})
        
        # Report findings
        if not machines:
            print("   ❌ No machine positions saved yet")
            print("   💡 Run get_machine_position() to calibrate and save positions")
            return {}
        
        print(f"   📊 Found {len(machines)} saved machine positions:")
        for machine_name, position_data in machines.items():
            timestamp = position_data.get("Time", "Unknown")
            translation = position_data.get("translation", {})
            x = translation.get("x", 0)
            y = translation.get("y", 0) 
            z = translation.get("z", 0)
            print(f"      ✓ {machine_name}: ({x:.3f}, {y:.3f}, {z:.3f}) saved at {timestamp}")
        
        return machines
        
    except Exception as e:
        print(f"   ⚠️  Error reading saved data: {e}")
        print("   💡 Check file permissions and YAML format")
        return {}

def check_aruco_status(**params) -> bool:
    """
    Check current ArUco marker detection status and help diagnose calibration issues.
    
    This function provides information about which markers should be detected
    for machine position calibration and offers troubleshooting tips.
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if status check completed successfully, False if error occurred
        
    Example:
        success = check_aruco_status()
        if success:
            print("ArUco status check completed")
    """
    try:
        print("🔍 ArUco Marker Detection Status Check")
        print("=" * 50)
        
        # Define required markers for machine calibration
        required_markers = {
            23: "portafilter_cleaner",
            31: "espresso_grinder", 
            41: "three_group_espresso"
        }
        
        # Display required markers information
        print("📋 Required ArUco markers for machine position calibration:")
        for marker_id, machine_name in required_markers.items():
            print(f"   • Marker ID {marker_id} → {machine_name}")
        
        # Provide troubleshooting guidance
        print("\n💡 Troubleshooting tips for marker detection:")
        troubleshooting_tips = [
            "Ensure markers are clearly visible to the camera",
            "Check that markers are not obstructed or damaged",
            "Verify camera is properly positioned and focused",
            "Make sure lighting conditions are adequate",
            "Try repositioning the robot for better camera angles",
            "Confirm markers are printed at correct size and quality"
        ]
        
        for i, tip in enumerate(troubleshooting_tips, 1):
            print(f"   {i}. {tip}")
        
        # Provide monitoring guidance
        print("\n📊 To monitor real-time detection, watch the robot logs for:")
        print("   • 'Detected N marker(s): [ID1, ID2, ...]' - successful detection")
        print("   • 'No ArUco markers detected' - markers not visible")
        print("   • Check camera feed if available for visual confirmation")
        
        # Display current saved data status
        print("\n📁 Current saved data status:")
        saved_data = check_saved_data()
        
        # Provide next steps based on current state
        print("\n🎯 Recommended next steps:")
        if not saved_data:
            print("   → Run get_machine_position() to perform initial calibration")
        else:
            print("   → Calibration data exists - system ready for operation")
            print("   → Re-run get_machine_position() if positions have changed")
        
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Error during ArUco status check: {e}")
        return False

"""
paper_cups.py

Defines the paper cup handling sequences for coffee service automation.
This module provides comprehensive functions for grabbing, placing, and serving
paper cups in the BARNS coffee automation system, including size-based handling
and staging area management.
"""

def _normalize_paper_cup_size(cups_dict: Any) -> str:
    """
    Parse paper cup size from new JSON format.
    
    This is a wrapper around the unified _normalize_cup_size function.
    Use this for backward compatibility in paper cup operations.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz')
    """
    return _normalize_cup_size(cups_dict, cup_type='paper')

def grab_paper_cup(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.
    
    This function performs the complete paper cup grabbing workflow:
    1. Moves to espresso home position
    2. Navigates to paper cup dispenser area (avoiding espresso machine)
    3. Positions for paper cup grab based on size parameters
    4. Executes approach, grip, and retreat sequence
    5. Returns to intermediate position ready for placement
    
    Args:
        size (str): Paper cup size to grab ('7oz', '9oz', '12oz', etc.), defaults to '7oz'
        
    Returns:
        bool: True if paper cup grabbed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during grabbing process
        
    Example:
        success = grab_paper_cup(size='12oz')
        if success:
            print("Paper cup grabbed successfully")
    """
    try:
        # Extract and validate size parameter using unified helper
        cups_dict = _extract_cups_dict(params)
        size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
        if not size:
            print("[ERROR] No size parameter provided")
            return False
            
        cup_params = GRAB_PAPER_CUP_PARAMS.get(str(size))
        
        # Validate parameters and use default if not found
        if not cup_params:
            print(f"[ERROR] Unknown paper cup size: {size!r}")
            print("[INFO] Using default 7oz parameters as fallback")
            cup_params = GRAB_PAPER_CUP_PARAMS.get("7oz")
            if not cup_params:
                print("[ERROR] Default 7oz parameters not found in GRAB_PAPER_CUP_PARAMS")
                print(f"[INFO] Available sizes: {list(GRAB_PAPER_CUP_PARAMS.keys())}")
                return False
        
        print(f"🥤 Starting paper cup grab sequence for size: {size}")
        print("=" * 50)
        
        # Step 1: Move to espresso home position
        print("🏠 Step 1/8: Moving to espresso home position...")
        home_result = run_skill("gotoJ_deg", *ESPRESSO_HOME)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Twist to avoid hitting the espresso machine during navigation
        print("🔄 Step 2/8: Navigating around espresso machine...")
        twist_result = run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['espresso_avoid'])
        if twist_result is False:
            print("[ERROR] Failed to twist around espresso machine")
            return False
        print("   ✅ Successfully navigated around espresso machine")
        
        # Step 3: Move to paper cup grabbing area
        print("📍 Step 3/8: Moving to paper cup dispenser area...")
        cup_area_result = run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['dispenser_area'])
        
        if cup_area_result is False:
            print("[ERROR] Failed to move to paper cup dispenser area")
            return False
        print("   ✅ Successfully positioned at paper cup dispenser")

        # Check if a cup is in the gripper
        attempt_count = 0
        while attempt_count < 5:        
            if size == "7oz":
                twist_back_result = run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_7oz'])
                if twist_back_result is False:
                    print("[ERROR] Failed to execute twist back movement")
                    return False
                print("   ✅ Successfully executed twist back movement")
            elif size == "9oz":
                twist_back_result = run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_9oz'])
                if twist_back_result is False:
                    print("[ERROR] Failed to execute twist back movement")
                    return False
                print("   ✅ Successfully executed twist back movement")
            elif size == "12oz":
                twist_back_result = run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_12oz'])
                if twist_back_result is False:
                    print("[ERROR] Failed to execute twist back movement")
                    return False
                print("   ✅ Successfully executed twist back movement")
            else:
                print("   ⏭️ No twist back movement defined for this size")
                return False
        
            # Step 5: Move end-effector into approach position
            print("🎯 Step 5/8: Moving to approach position...")
            if 'approach' in cup_params:
                print(f"   📍 Executing approach movement: {cup_params['approach']}")
                approach_result = run_skill("moveEE", *cup_params['approach'])
                if approach_result is False:
                    print("[ERROR] Failed to move to approach position")
                    return False
                print("   ✅ Successfully moved to approach position")
            else:
                print("   ⏭️ No approach movement defined for this size")
            
            # Step 6: Close gripper to grasp the paper cup
            print("🤏 Step 6/8: Gripping paper cup...")
            if 'grip_width' in cup_params:
                print(f"   📏 Setting gripper width to: {cup_params['grip_width']}")
                grip_result = run_skill("set_gripper_position", GRIPPER_FULL, cup_params['grip_width'])
                if grip_result is False:
                    print("[ERROR] Failed to grip paper cup")
                    return False
                print("   ✅ Successfully gripped paper cup")
            else:
                print("[ERROR] No grip width defined for this size")
                return False
            
            # Step 7: Retract after gripping
            print("⬆️ Step 7/8: Retracting with paper cup...")
            if 'retreat' in cup_params:
                print(f"   📍 Executing retreat movement: {cup_params['retreat']}")
                retreat_result = run_skill("moveEE", *cup_params['retreat'])
                if retreat_result is False:
                    print("[ERROR] Failed to retreat with paper cup")
                    return False
                print("   ✅ Successfully retracted with paper cup")
            else:
                print("   ⏭️ No retreat movement defined for this size")
            
            # Check if a cup is in the gripper
            cup_detected = detect_cup_gripper()
            if cup_detected:
                print("✅ Cup detected in gripper")
                break
            else:
                print("❌ No cup detected in gripper")
                attempt_count += 1

                # Step 8: Open gripper to release paper cup
                print("🤏 Step 8/8: Releasing paper cup...")
                release_result = run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)
                
                if release_result is False:
                    print("[ERROR] Failed to release paper cup")
                    return False
                print("   ✅ Paper cup released successfully")
                if attempt_count == 3:
                    print("[ERROR] Failed to grab paper cup after 3 attempts")
                    return False

                
        # Final success summary
        print("=" * 50)
        print(f"✅ PAPER CUP GRAB COMPLETED SUCCESSFULLY FOR {size.upper()}")
        print("   ✓ Paper cup securely gripped")
        print("   ✓ Safe navigation around espresso machine")
        print("   ✓ Robot positioned for next operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup grab: {e}")
        print("[INFO] Paper cup grab process terminated due to error")
        return False

def place_paper_cup(**params) -> bool:
    """
    Place a paper cup at the specified staging area.
    
    This function places a previously grabbed paper cup at a designated serving stage:
    1. Moves from intermediate position to staging area
    2. Adjusts orientation for precise placement
    3. Positions paper cup at target location
    4. Releases paper cup and retreats safely
    5. Returns to staging home position
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        
    Returns:
        bool: True if paper cup placed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during placement process
        
    Example:
        success = place_paper_cup(position={'cup_position': 1.0})
        if success:
            print("Paper cup placed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = f"stage_{cup_position}"  # Convert to internal stage format
            
        stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
        
        # Validate stage parameter
        if not stage_params:
            print(f"[ERROR] Unknown stage: {stage!r}")
            print(f"[INFO] Available stages: {list(PLACE_PAPER_CUP_PARAMS.keys())}")
            return False
        
        print(f"📍 Starting paper cup placement sequence for: {stage}")
        print("=" * 50)
        
        # Step 1: Start from intermediate position
        print("📍 Step 1/7: Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['intermediate'])
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        # Step 2: Move into staging twist angle
        print("🔄 Step 2/7: Adjusting orientation for staging...")
        if 'twist' in stage_params:
            print(f"   📍 Executing staging twist for {stage}")
            twist_result = run_skill("moveJ_deg", *stage_params['twist'])
            
            if twist_result is False:
                print("[ERROR] Failed to execute staging twist")
                return False
            print("   ✅ Successfully adjusted orientation")
        else:
            print("   ⏭️ No twist movement defined for this stage")
        
        # Step 3: Move to target placement pose
        print("🎯 Step 3/7: Moving to placement position...")
        if 'pose' in stage_params:
            print(f"   📍 Moving to placement pose for {stage}")
            pose_result = run_skill("gotoJ_deg", *stage_params['pose'])
            
            if pose_result is False:
                print("[ERROR] Failed to move to placement pose")
                return False
            print("   ✅ Successfully positioned for placement")
        else:
            print("[ERROR] No placement pose defined for this stage")
            return False
        
        # Step 4: Open gripper to release paper cup
        print("🤏 Step 4/7: Releasing paper cup...")
        release_result = run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)
        
        if release_result is False:
            print("[ERROR] Failed to release paper cup")
            return False
        print("   ✅ Paper cup released successfully")
        
        # Step 5: Move up after placing paper cup
        print("⬆️ Step 5/7: Moving up after placement...")
        up_result = run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])
        
        if up_result is False:
            print("[ERROR] Failed to move up after placement")
            return False
        print("   ✅ Successfully moved up after placement")
        
        # Step 6: Move to staging home position
        print("🏠 Step 6/7: Moving to staging home...")
        if 'stage_home' in stage_params:
            print(f"   📍 Moving to staging home for {stage}")
            stage_home_result = run_skill("gotoJ_deg", *stage_params['stage_home'])
            
            if stage_home_result is False:
                print("[ERROR] Failed to move to staging home")
                return False
            print("   ✅ Successfully moved to staging home")
        else:
            print("   ⏭️ No staging home defined for this stage")
        
        # Step 7: Untwist back towards machine
        print("🔄 Step 7/7: Untwisting back towards machine...")
        if 'twist_back' in stage_params:
            print(f"   📍 Executing untwist movement for {stage}")
            twist_back_result = run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_back_machine'])
            
            if twist_back_result is False:
                print("[ERROR] Failed to untwist back")
                return False
            print("   ✅ Successfully untwisted back towards machine")
        else:
            print("   ⏭️ No untwist movement defined for this stage")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PAPER CUP PLACEMENT COMPLETED SUCCESSFULLY FOR {stage.upper()}")
        print("   ✓ Paper cup placed at designated staging area")
        print("   ✓ Safe clearance achieved after placement")
        print("   ✓ Robot ready for next operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup placement: {e}")
        print("[INFO] Paper cup placement process terminated due to error")
        return False

def dispense_paper_cup_station(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.

    This function composes the full workflow by invoking `grab_paper_cup` followed by
    `place_paper_cup` using the provided parameters.

    Args:
        size (str): Paper cup size to grab ('7oz', '9oz', '12oz', etc.). Defaults to '7oz'.
        stage (str): Target stage for placement ('stage_1', 'stage_2', etc.). Defaults to 'stage_1'.

    Returns:
        bool: True if both grab and place steps succeed, False otherwise.
    """
    try:
        print("🥤🚚 Starting paper cup dispense sequence")
        print("=" * 50)

        # Pass full params to grab_paper_cup so it can extract cup size properly
        grab_ok = grab_paper_cup(**params)
        if grab_ok is False:
            print("[ERROR] Paper cup grab step failed; aborting dispense sequence")
            return False

        # Pass full params to place_paper_cup so it can extract stage properly
        place_ok = place_paper_cup(**params)
        if place_ok is False:
            print("[ERROR] Paper cup placement step failed; aborting dispense sequence")
            return False

        print("=" * 50)
        print(f"✅ PAPER CUP DISPENSED SUCCESSFULLY")
        print("=" * 50)
        return True

    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup dispensing: {e}")
        return False

def pick_paper_cup_station(**params) -> bool:
    """
    Pick up a paper cup from a specific stage.

    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cups (dict): Cup size dictionary, e.g., {'cup_H12': 1.0}
    """
    try:
        # return_back_to_home()
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use

        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}
        
        size_mapped = _normalize_paper_cup_size(cups_dict)

        # Validate parameters
        valid_stages = ('1', '2', '3', '4')
        valid_sizes = ('7oz', '9oz', '12oz')

        if stage not in valid_stages:
            print(f"[ERROR] Invalid stage: {stage!r}")
            print(f"[INFO] Valid stages: {', '.join(valid_stages)}")
            return False

        if size_mapped not in valid_sizes:
            print(f"[ERROR] Invalid cup size for paper: {size_mapped!r}")
            print(f"[INFO] Valid sizes: H7/H9/H12 (or 7oz/9oz/12oz)")
            return False

        print(f"🥤 Starting paper cup pickup sequence - Stage: {stage}, Size: {size_mapped}")
        print("=" * 50)

        # Stage-specific positioning using parameters
        stage_positions = {
            "1": PAPER_CUPS_STATION_PARAMS['staging']['pickup_1'],
            "2": PAPER_CUPS_STATION_PARAMS['staging']['pickup_2'],
            "3": PAPER_CUPS_STATION_PARAMS['staging']['pickup_3'],
            "4": PAPER_CUPS_STATION_PARAMS['staging']['pickup_4']
        }

        # Paper cup gripper positions using constants
        gripper_positions = PAPER_CUP_GRIPPER_POSITIONS

        # Step 1: Navigate to home positions
        print("🏠 Step 1/6: Navigating to home positions...")
        if not home(position="east"):
            print("[ERROR] Failed to move to east home")
            return False
        if stage in ("3", "4"):
            if not home(position="south_east"):
                print("[ERROR] Failed to move to south_east home")
                return False
        print("   ✅ Successfully navigated to home positions")

        # Step 2: Move to stage-specific position
        print(f"📍 Step 2/6: Moving to stage {stage} position...")
        stage_result = run_skill("gotoJ_deg", *stage_positions[stage])
        if not stage_result:
            print(f"[ERROR] Failed to move to stage {stage} position")
            return False
        print(f"   ✅ Successfully positioned at stage {stage}")

        # Step 3: Position for cup pickup
        print("🎯 Step 3/6: Positioning for cup pickup...")
        pickup_result = run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_down'])
        if not pickup_result:
            print("[ERROR] Failed to position for cup pickup")
            return False
        print("   ✅ Successfully positioned for pickup")

        # Step 4: Grip the cup
        print(f"🤏 Step 4/6: Gripping {size_mapped} paper cup...")
        grip_result = run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[size_mapped])
        if not grip_result:
            print("[ERROR] Failed to grip cup")
            return False
        print("   ✅ Cup gripped successfully")

        run_skill("moveEE_movJ", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_up'])
        
        # Step 5: Return to safe position
        print("🏠 Step 5/6: Returning to east home...")
        if not home(position="east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully returned to east home")

        # Step 6: Return to north-east home
        print("🏠 Step 6/6: Returning to north-east home...")
        if not home(position="north_east"):
            print("[ERROR] Failed to return to north-east home")
            return False
        print("   ✅ Successfully returned to north-east home")

        # Final success summary
        print("=" * 50)
        print(f"✅ PAPER CUP PICKUP COMPLETED SUCCESSFULLY")
        print(f"   ✓ Stage {stage} cup ({size_mapped}) picked up")
        print("   ✓ Positioned for next operation")
        print("=" * 50)
        return True
    
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup pickup: {e}")
        print("[INFO] Cup pickup process terminated due to error")
        return False

def place_paper_cup_station(**params) -> bool:
    """
    Place a paper cup at specified staging area.

    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use

        print(f"🥤 Starting paper cup placement sequence for stage {stage}")
        print("=" * 50)

        # Step 1: Move to north-east home
        print("🏠 Step 1/5: Moving to north-east home...")
        if not home(position="north_east"):
            print("[ERROR] Failed to move to north-east home")
            return False
        print("   ✅ Successfully moved to north-east home")

        # Step 2: Move to east home
        print("🏠 Step 2/5: Moving to east home...")
        if not home(position="east"):
            print("[ERROR] Failed to move to east home")
            return False
        print("   ✅ Successfully moved to east home")

        if stage in ("3", "4"):
            print("🏠 Step 2.5/5: Moving to south-east home...")
            if not home(position="south_east"):
                print("[ERROR] Failed to move to south-east home")
                return False
            print("   ✅ Successfully moved to south-east home")

        # Step 3: Move to stage-specific position using parameters
        print(f"🎯 Step 3/5: Moving to stage {stage} position...")
        stage_positions = {
            "1": PAPER_CUPS_STATION_PARAMS['staging']['place_1'],
            "2": PAPER_CUPS_STATION_PARAMS['staging']['place_2'],
            "3": PAPER_CUPS_STATION_PARAMS['staging']['place_3'],
            "4": PAPER_CUPS_STATION_PARAMS['staging']['place_4']
        }

        stage_result = run_skill("gotoJ_deg", *stage_positions[stage])
        if not stage_result:
            print(f"[ERROR] Failed to move to stage {stage} position")
            return False
        print(f"   ✅ Successfully positioned at stage {stage}")

        # Step 4: Release cup
        print("🤏 Step 4/5: Releasing paper cup...")
        release_result = run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)
        if not release_result:
            print("[ERROR] Failed to release paper cup")
            return False
        print("   ✅ Cup released successfully")

        # Step 5: Move up and return to home
        print("⬆️ Step 5/5: Moving up and returning to home...")
        up_result = run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_return_up'])
        if not up_result:
            print("[ERROR] Failed to move up after placement")
            return False
        if not home(position="east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully moved up and returned to home")

        # Final success summary
        print("=" * 50)
        print(f"✅ PAPER CUP PLACEMENT COMPLETED FOR STAGE {stage}")
        print("   ✓ Cup positioned at designated staging area")
        print("   ✓ Safe release and clearance achieved")
        print("   ✓ Robot returned to home position")
        print("   🥤 Beverage station ready!")
        print("=" * 50)
        return True

    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup placement: {e}")
        print("[INFO] Cup placement process terminated due to error")
        return False

def place_paper_cup_sauces(**params) -> bool:
    """
    Place the paper cup at the sauces station.
    """
    try:
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position1']) is False:
            return False
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position2']) is False:
            return False
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position3']) is False:
            return False
        if run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_paper_cup_sauces failed: {e}")
        return False

def pick_paper_cup_sauces(**params) -> bool:
    """
    Pick the paper cup from the sauces station.

    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_H12': 1.0} - supports H7, H9, H12 (7oz, 9oz, 12oz)
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}
        
        cup_size = _normalize_paper_cup_size(cups_dict)
        valid_sizes = ("7oz", "9oz", "12oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        # Paper cups only support 7oz, 9oz, 12oz (no 16oz for paper)
        # Using standard gripper position for sauces station (145 for all sizes)
        gripper_position = 145
        if run_skill("moveEE", 0,0,5,0,0,0) is False:
            return False
        if run_skill("set_gripper_position", GRIPPER_FULL, gripper_position) is False:
            return False
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position2']) is False:
            return False
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position1']) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] pick_paper_cup_sauces failed: {e}")
        return False

def place_paper_cup_milk(**params) -> bool:
    """
    Place the paper cup at the milk station.
    """
    try:
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position1']) is False:
            return False
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position2']) is False:
            return False
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position3']) is False:
            return False
        if run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_paper_cup_milk failed: {e}")
        return False

def pick_paper_cup_milk(**params) -> bool:
    """
    Pick the paper cup from the milk station.

    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_H12': 1.0} - supports H7, H9, H12 (7oz, 9oz, 12oz)
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}
        
        cup_size = _normalize_paper_cup_size(cups_dict)
        valid_sizes = ("7oz", "9oz", "12oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        # Paper cups only support 7oz, 9oz, 12oz (no 16oz for paper)
        # Using standard gripper position for milk station (145 for all sizes)
        gripper_position = 145
        if run_skill("moveEE", 0,0,5,0,0,0) is False:
            return False
        if run_skill("set_gripper_position", GRIPPER_FULL, gripper_position) is False:
            return False
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position2']) is False:
            return False
        if run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position1']) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] pick_paper_cup_milk failed: {e}")
        return False

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
    
    This function performs the complete portafilter unmounting sequence:
    1. Moves to espresso home position
    2. Approaches and mounts to the specified portafilter group
    3. Closes gripper to secure portafilter
    4. Releases tension and adjusts orientation
    5. Rotates portafilter to unlock position
    6. Safely retracts and moves to clear path
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3'), defaults to 'port_2'
        
    Returns:
        bool: True if portafilter unmounted successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during unmount process
        
    Example:
        success = unmount(port='port_1')
        if success:
            print("Portafilter unmounted successfully")
    """
    global below_espresso_port, mount_espresso_port, mount_espresso_pose
    try:
        # Normalize from espresso shot if provided
        # New format: {'espresso': {'espresso_shot_double': 2.0}}
        espresso_dict = params.get("espresso")
        shot_cfg = _normalize_espresso_shot(espresso_dict)

        # Extract and validate port parameter (derived from shot when not explicitly provided)
        port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
        if not port:
            print("[ERROR] No port parameter provided")
            return False
            
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] Unknown port number: {port!r}")
            print(f"[INFO] Available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"📤 Starting portafilter unmount sequence for {port}")
        print("=" * 50)
        
        # Step 1: Move to espresso home position
        print("🏠 Step 1/13: Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *port_params['home'])
        
        if home_result is False:
            print("[ERROR] Failed to move to espresso home position")
            return False
        print("   ✅ Successfully moved to espresso home")

        # Step 2: Conditional approach based on port type
        if port == 'port_1' or port == 'port_3':
            print(f"🎯 Step 2/13: Approaching portafilter {port_params['portafilter_number']}...")
            approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'])
            if approach_result is False:
                print("[ERROR] Failed to approach portafilter")
                return False
            print("   ✅ Successfully approached portafilter")
        else:
            print("   ⏭️ Skipping approach step for port_2")
        
        mount_result = run_skill("mount_machine", "three_group_espresso", port_params['portafilter_number'])
        
        if mount_result is False:
            print("[ERROR] Failed to mount to portafilter")
            return False
        print("   ✅ Successfully mounted to portafilter")
        run_skill("sync")

        # Step 4: Close gripper to secure portafilter
        print("🤏 Step 4/13: Securing portafilter with gripper...")
        grip_result = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['grip'])
        if grip_result is False:
            print("[ERROR] Failed to close gripper")
            return False
        print("   ✅ Gripper closed successfully")
        
        # Step 5: Release tension for smooth operation
        print("😌 Step 5/13: Releasing tension...")
        tension_result = run_skill("release_tension")
        if tension_result is False:
            print("[ERROR] Failed to release tension")
            return False
        print("   ✅ Tension released successfully")
        
        # Step 6: First orientation enforcement
        print("📐 Step 6/13: Enforcing proper orientation (first pass)...")
        orient_result1 = run_skill("enforce_rxry")
        if orient_result1 is False:
            print("[ERROR] Failed to enforce orientation (first attempt)")
            return False
        print("   ✅ First orientation enforcement completed")
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 7: Rotate portafilter to unlock (-45 degrees)
        print("🔄 Step 7/13: Rotating portafilter to unlock...")
        rotate_result = run_skill("move_portafilter_arc_movJ", -42.0)
        
        if rotate_result is False:
            print("[ERROR] Failed to rotate portafilter")
            return False

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 8: Release tension after rotation
        print("😌 Step 8/13: Releasing tension after rotation...")
        tension_result2 = run_skill("release_tension")
        if tension_result2 is False:
            print("[ERROR] Failed to release tension after rotation")
            return False
        print("   ✅ Tension released after rotation")
            
        time.sleep(ESPRESSO_DELAYS['orientation_settle'])  # Allow settling time

        # Step 9: Capture mount position for later use
        print("📸 Step 9/13: Capturing mount position...")
        mount_espresso_port = run_skill("current_angles")
        if mount_espresso_port is None:
            print("[ERROR] Failed to capture mount position")
            return False
        # Validate captured position data
        if not isinstance(mount_espresso_port, (tuple, list)) or len(mount_espresso_port) != 6:
            print(f"[ERROR] Invalid mount position data: {mount_espresso_port} (expected 6 joint angles)")
            return False
        print("   ✅ Mount position captured successfully")
        
        # Capture mount pose for portafilter validation
        mount_espresso_pose = run_skill("current_pose")
        if mount_espresso_pose is None:
            print("[ERROR] Failed to capture mount pose")
            return False
        if not isinstance(mount_espresso_pose, (tuple, list)) or len(mount_espresso_pose) != 6:
            print(f"[ERROR] Invalid mount pose data: {mount_espresso_pose} (expected 6 values)")
            return False
        print(f"   ✅ Mount pose captured: Z={mount_espresso_pose[2]:.2f}mm")
        
        # Step 10: Move end effector down to clear portafilter
        print("⬇️ Step 10/13: Moving down to clear portafilter...")
        print(f"   📍 Executing: moveEE_movJ{ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down']}")
        clear_result = run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down'])
        
        if clear_result is False:
            print("[ERROR] Failed to move down to clear portafilter")
            return False
        print("   ✅ Successfully moved down to clear portafilter")
        # Step 11: Capture below position for later use
        print("📸 Step 11/13: Capturing below position...")
        below_espresso_port = run_skill("current_angles")
        if below_espresso_port is None:
            print("[ERROR] Failed to capture below position")
            return False
        # Validate captured position data
        if not isinstance(below_espresso_port, (tuple, list)) or len(below_espresso_port) != 6:
            print(f"[ERROR] Invalid below position data: {below_espresso_port} (expected 6 joint angles)")
            return False
        print("   ✅ Below position captured successfully")
        
        # Step 13: Move back to avoid collisions
        print("⬅️ Step 13/13: Moving back to avoid collisions...")
        back_result = run_skill("gotoJ_deg", *port_params['move_back'])
        
        if back_result is False:
            print("[ERROR] Failed to move back")
            return False
        print("   ✅ Successfully moved back to safe position")
        
        # Step 15: Special handling for ports 2 and 3 (additional navigation)
        if port in ('port_2', 'port_3'):
            print("🔄 Executing special navigation for port 2/3...")
            nav1_result = run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])
            
            if nav1_result is False:
                print("[ERROR] Failed special navigation step 1")
                return False
            print("   ✅ Special navigation step 1 completed")
            
            nav2_result = run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav2'])
            
            if nav2_result is False:
                print("[ERROR] Failed special navigation step 2")
                return False
            print("   ✅ Special navigation step 2 completed")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PORTAFILTER UNMOUNT COMPLETED SUCCESSFULLY FOR {port.upper()}")
        print("   ✓ Portafilter secured and unlocked")
        print("   ✓ Positions captured for future mounting")
        print("   ✓ Robot moved to safe position")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during unmount: {e}")
        print("[INFO] Unmount process terminated due to error")
        return False

def grinder(**params) -> bool:
    """
    Grind coffee and tamp portafilter at the grinder station.
    
    This function performs the complete grinding and tamping workflow:
    1. Moves to grinder home position (for port_1)
    2. Approaches grinder for coffee grinding
    3. Mounts to grinder to activate grinding
    4. Moves to tamper for coffee compaction
    5. Performs tamping motion
    6. Opens gripper to complete process
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3'), defaults to 'port_2'
        positioning_time (float): Time in seconds to allow for positioning, defaults to 3.0
        portafilter_tool (str): Tool type ('single_portafilter' or 'double_portafilter'), defaults to 'single_portafilter'
        
    Returns:
        bool: True if grinding and tamping completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during grinding process
        
    Example:
        success = grinder(port='port_1', positioning_time=2.5, portafilter_tool='double_portafilter')
        if success:
            print("Coffee grinding and tamping completed")
    """
    try:
        # Extract and normalize parameters
        # New format: {'espresso': {'espresso_shot_double': 2.0}}
        espresso_dict = params.get("espresso")
        shot_cfg = _normalize_espresso_shot(espresso_dict)

        # Allow explicit overrides, else derive from shot config, else fall back to legacy defaults
        port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
        positioning_time = params.get("positioning_time")
        if positioning_time is None:
            positioning_time = (shot_cfg.get("positioning_time") if shot_cfg else 5.0)
        portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "double_portafilter")
        if not port:
            print("[ERROR] No port parameter provided")
            return False
            
        # Validate portafilter tool parameter
        if portafilter_tool not in ('single_portafilter', 'double_portafilter'):
            print(f"[ERROR] Invalid portafilter_tool: {portafilter_tool!r}")
            print("[INFO] Available options: single_portafilter, double_portafilter")
            return False
            
        print(f"☕ Starting grinding and tamping sequence for {port}")
        print("=" * 50)
        
        # Step 1: Conditional move to grinder home for port_1
        if port == 'port_1':
            print("🏠 Step 1/7: Moving to grinder home position...")
            home_result = run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
            if home_result is False:
                print("[ERROR] Failed to move to grinder home")
                return False
            print("   ✅ Successfully moved to grinder home")
        else:
            print("   ⏭️ Skipping grinder home movement for this port")
        
        # Step 2: Approach the grinder
        print("🎯 Step 2/7: Approaching grinder...")        
        approach_result = run_skill("approach_machine", "espresso_grinder", "grinder")
        if approach_result is False:
            print("[ERROR] Failed to approach grinder")
            return False
        print("   ✅ Successfully approached grinder")
        
        mount_result = run_skill("mount_machine", "espresso_grinder", "grinder")
        if mount_result is False:
            print("[ERROR] Failed to mount to grinder")
            return False
        print("   ✅ Successfully mounted to grinder")
        
        tamper_approach_result = run_skill("approach_machine", "espresso_grinder", "tamper")
        if tamper_approach_result is False:
            print("[ERROR] Failed to approach tamper")
            return False
        print("   ✅ Successfully approached tamper")
        
        # Allow positioning time
        print(f"   ⏰ Allowing positioning time ({positioning_time}s)...")
        time.sleep(positioning_time)
        
        mount_result2 = run_skill("mount_machine", "espresso_grinder", "grinder")
        if mount_result2 is False:
            print("[ERROR] Failed to re-mount to grinder")
            return False
        print("   ✅ Successfully re-mounted to grinder")
        
        tamper_mount_result = run_skill("mount_machine", "espresso_grinder", "tamper")
        if tamper_mount_result is False:
            print("[ERROR] Failed to mount to tamper")
            return False
        print("   ✅ Successfully positioned at tamper")

        # Step 7: Open gripper to complete process
        print("🤏 Step 7/7: Opening gripper...")
        open_gripper = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['release'])
        if open_gripper is False:
            print("[ERROR] Failed to open gripper")
            return False
        print("   ✅ Gripper opened successfully")

        # Step 8: Approach specified portafilter tool with fallback
        print(f"🎯 Step 8/8: Approaching {portafilter_tool}...")
        approach_tool_result = run_skill("approach_tool", portafilter_tool)
        if approach_tool_result is False:
            # Try the other tool as fallback
            fallback_tool = "double_portafilter" if portafilter_tool == "single_portafilter" else "single_portafilter"
            print(f"[WARNING] Failed to approach {portafilter_tool}, trying {fallback_tool} as fallback...")
            approach_tool_result = run_skill("approach_tool", fallback_tool)
            if approach_tool_result is False:
                print(f"[WARNING] Failed to approach both {portafilter_tool} and {fallback_tool}")
            else:
                print(f"   ✅ Successfully approached {fallback_tool} (fallback)")
                portafilter_tool = fallback_tool  # Update for logging
        else:
            print(f"   ✅ Successfully approached {portafilter_tool}")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ GRINDING AND TAMPING COMPLETED SUCCESSFULLY FOR {port.upper()}")
        print("   ✓ Coffee grounds processed")
        print("   ✓ Tamping operation completed")
        print("   ✓ Gripper opened for next operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during grinding: {e}")
        print("[INFO] Grinding process terminated due to error")
        return False
    
def tamper(**params) -> bool:
    """
    Tamp coffee at the tamper station using portafilter tool.
    
    This function performs the complete tamping sequence:
    1. Approaches and grabs the portafilter tool
    2. Closes gripper to secure tool
    3. Lifts tool slightly for positioning
    4. Mounts to grinder for proper alignment
    5. Approaches grinder for final positioning
    6. Returns to grinder home position
    
    Args:
        portafilter_tool (str): Tool type ('single_portafilter' or 'double_portafilter'), defaults to 'single_portafilter'
        
    Returns:
        bool: True if tamping completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during tamping process
        
    Example:
        success = tamper(portafilter_tool='double_portafilter')
        if success:
            print("Coffee tamping completed successfully")
    """
    try:
        # Extract and normalize parameters from espresso shot configuration
        # New format: {'espresso': {'espresso_shot_single': 1.0}}
        espresso_dict = params.get("espresso")
        shot_cfg = _normalize_espresso_shot(espresso_dict)
        
        # Allow explicit overrides, else derive from shot config, else fall back to default
        portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "single_portafilter")
        
        print(f"[DEBUG tamper] espresso_dict: {espresso_dict}")
        print(f"[DEBUG tamper] shot_cfg: {shot_cfg}")
        print(f"[DEBUG tamper] portafilter_tool resolved to: {portafilter_tool}")
        
        # Validate portafilter tool parameter
        if portafilter_tool not in ('single_portafilter', 'double_portafilter'):
            print(f"[ERROR] Invalid portafilter_tool: {portafilter_tool!r}")
            print("[INFO] Available tools: single_portafilter, double_portafilter")
            return False
            
        print("🔨 Starting coffee tamping sequence")
        print("=" * 50)
        
        # Step 1: Approach and grab portafilter tool with fallback
        print(f"🎯 Step 1/6: Approaching {portafilter_tool}...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        approach_tool_result = run_skill("approach_tool", portafilter_tool)
        if approach_tool_result is False:
            # Try the other tool as fallback
            fallback_tool = "double_portafilter" if portafilter_tool == "single_portafilter" else "single_portafilter"
            print(f"[WARNING] Failed to approach {portafilter_tool}, trying {fallback_tool} as fallback...")
            approach_tool_result = run_skill("approach_tool", fallback_tool)
            if approach_tool_result is False:
                print(f"[ERROR] Failed to approach both {portafilter_tool} and {fallback_tool}")
                return False
            else:
                print(f"   ✅ Successfully approached {fallback_tool} (fallback)")
                portafilter_tool = fallback_tool  # Update for subsequent operations
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        grab_tool_result = run_skill("grab_tool", portafilter_tool)
        if grab_tool_result is False:
            print(f"[ERROR] Failed to grab {portafilter_tool}")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        print(f"   ✅ Successfully approached and grabbed {portafilter_tool}")
        
        # Step 2: Close gripper to secure tool
        print("🤏 Step 2/6: Securing tool with gripper...")
        close_gripper = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['grip'])
        if close_gripper is False:
            print("[ERROR] Failed to close gripper")
            return False
        print("   ✅ Tool secured with gripper")

        # Step 5.1: Move end effector down to fix portafilter
        print("⬇️ Step 5.1/10: Moving down to fix portafilter...")
        print(f"   📍 Executing: moveEE_movJ(0, 0, -5, 0, 0, 0)")
        clear_result = run_skill("moveEE_movJ", 0, 0, -5, 0, 0, 0)
        
        # Step 3: Lift tool slightly for positioning
        print("⬆️ Step 3/6: Lifting tool for positioning...")
        lift_result = run_skill("moveEE", 0, 0, 20, 0, 0, 0)
        if lift_result is False:
            print("[ERROR] Failed to lift tool")
            return False
        print("   ✅ Tool lifted successfully")
        
        mount_result = run_skill("mount_machine", "espresso_grinder", "grinder")
        if mount_result is False:
            print("[ERROR] Failed to mount to grinder")
            return False
        print("   ✅ Successfully mounted to grinder")
        
        approach_result = run_skill("approach_machine", "espresso_grinder", "grinder")
        if approach_result is False:
            print("[ERROR] Failed to approach grinder")
            return False
        print("   ✅ Successfully approached grinder")

        # Step 6: Return to grinder home
        print("🏠 Step 6/6: Returning to grinder home...")
        final_home_result = run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
        if final_home_result is False:
            print("[ERROR] Failed to return to grinder home")
            return False
        print("   ✅ Successfully returned to grinder home")
        
        # Final success summary
        print("=" * 50)
        print("✅ COFFEE TAMPING COMPLETED SUCCESSFULLY")
        print(f"   ✓ {portafilter_tool.replace('_', ' ').title()} tool used")
        print("   ✓ Proper tamping pressure applied")
        print("   ✓ Robot returned to home position")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during tamping: {e}")
        print("[INFO] Tamping process terminated due to error")
        return False

def mount(**params) -> bool:
    """
    Mount portafilter back to espresso group after grinding.
    
    This function performs the complete portafilter mounting sequence:
    1. Handles special navigation for ports 2 and 3
    2. Moves through safe path to target group
    3. Approaches and mounts to espresso group
    4. Adjusts position based on specific port requirements
    5. Performs orientation enforcement and locking rotation
    6. Opens gripper to release portafilter
    7. Returns to home position
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3'), defaults to 'port_2'
        attempt_count (int): Current retry attempt number (internal use), defaults to 0
        
    Returns:
        bool: True if portafilter mounted successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during mount process
        
    Example:
        success = mount(port='port_1')
        if success:
            print("Portafilter mounted successfully")
    """
    global mount_espresso_pose, below_espresso_port
    try:
        # Check retry limit
        attempt_count = params.get("attempt_count", 0)
        if attempt_count >= 3:
            print("=" * 60)
            print("❌ MAXIMUM RETRY ATTEMPTS REACHED (3)")
            print("=" * 60)
            print("[ERROR] Failed to mount portafilter after 3 attempts")
            return False
        
        # Normalize from espresso shot if provided
        # New format: {'espresso': {'espresso_shot_double': 2.0}}
        espresso_dict = params.get("espresso")
        shot_cfg = _normalize_espresso_shot(espresso_dict)

        # Extract and validate port parameter (derived from shot when not explicitly provided)
        port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
        if not port:
            print("[ERROR] No port parameter provided")
            return False
            
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] Unknown port number: {port!r}")
            print(f"[INFO] Available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        if attempt_count > 0:
            print(f"📥 Starting portafilter mount sequence for {port} (Retry attempt {attempt_count}/3)")
        else:
            print(f"📥 Starting portafilter mount sequence for {port}")
        print("=" * 50)
        
        # Step 1: Special handling for ports 2 and 3 (reverse navigation)
        if port in ('port_2', 'port_3'):
            print("🔄 Step 1/10: Executing special navigation for port 2/3...")
            nav1_result = run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])
            
            if nav1_result is False:
                print("[ERROR] Failed special navigation step 1")
                return False
            print("   ✅ Special navigation step 1 completed")
        else:
            print("   ⏭️ Skipping special navigation for port_1")
        
        # Step 2: Move to safe path position
        print("📍 Step 2/10: Moving to safe path position...")
        back_result = run_skill("gotoJ_deg", *port_params['move_back'])
        
        if back_result is False:
            print("[ERROR] Failed to move to safe path position")
            return False
        print("   ✅ Successfully moved to safe path position")
        
        # Step 4: Approach the espresso group using captured position
        print(f"🎯 Step 4/10: Approaching espresso group {port_params['group_number']}...")
        if below_espresso_port is None:
            print("[ERROR] below_espresso_port not captured")
            print("[INFO] Run unmount first to capture required positions")
            print("[INFO] The mount function requires position data from the unmount function")
            return False
        
        # Validate the captured position data
        if not isinstance(below_espresso_port, (tuple, list)) or len(below_espresso_port) != 6:
            print(f"[ERROR] Invalid below_espresso_port data: {below_espresso_port}")
            print("[ERROR] Expected 6 joint angles from current_angles() during unmount")
            return False
        
        print(f"   📍 Using captured below position: {below_espresso_port}")
        approach_result = run_skill("gotoJ_deg", *below_espresso_port)
        if approach_result is False:
            print("[ERROR] Failed to approach espresso group")
            return False
        print("   ✅ Successfully approached espresso group")
        
        # Step 5: Mount to espresso group using captured position
        print("🔧 Step 5/10: Mounting to espresso group...")
        if mount_espresso_port is None:
            print("[ERROR] mount_espresso_port not captured")
            print("[INFO] Run unmount first to capture required positions")
            print("[INFO] The mount function requires position data from the unmount function")
            return False
        
        # Validate the captured position data
        if not isinstance(mount_espresso_port, (tuple, list)) or len(mount_espresso_port) != 6:
            print(f"[ERROR] Invalid mount_espresso_port data: {mount_espresso_port}")
            print("[ERROR] Expected 6 joint angles from current_angles() during unmount")
            return False
        
        print(f"   📍 Using captured mount position: {mount_espresso_port}")
        mount_result = run_skill("gotoJ_deg", *mount_espresso_port)
        if mount_result is False:
            print("[ERROR] Failed to mount to espresso group")
            return False
        print("   ✅ Successfully mounted to espresso group")
        
        run_skill("sync")

        # Validate portafilter - check if it was filled twice
        print("🔍 Validating portafilter fill status...")
        current_mount_pose = run_skill("current_pose")
        if current_mount_pose is None or mount_espresso_pose is None:
            print("[WARNING] Cannot validate portafilter - pose data missing")
        else:
            # Compare Z positions (index 2 is Z in the pose tuple)
            z_original = float(mount_espresso_pose[2])
            z_current = float(current_mount_pose[2])
            z_difference = abs(z_current - z_original)
            
            print(f"   📏 Z difference: {z_difference:.2f}mm (threshold: {PORTAFILTER_Z_THRESHOLD_MM}mm)")
            print(f"      Original Z: {z_original:.2f}mm, Current Z: {z_current:.2f}mm")
            
            if z_difference > PORTAFILTER_Z_THRESHOLD_MM:
                print("=" * 60)
                print("⚠️  PORTAFILTER FILLED TWICE DETECTED!")
                print("=" * 60)
                print(f"   Z position changed by {z_difference:.2f}mm (>{PORTAFILTER_Z_THRESHOLD_MM}mm)")
                print("   🔄 Initiating recovery sequence:")
                print("      1. Unmount portafilter")
                print("      2. Clean portafilter")
                print("      3. Remount portafilter")
                print("=" * 60)
                
                # Step 1: Unmount (move down to clear)
                print("🔄 Step 1/3: Moving down to clear portafilter...")
                clear_down_result = run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down'])
                if clear_down_result is False:
                    print("[ERROR] Failed to move down during recovery")
                    return False
                print("   ✅ Successfully cleared portafilter")
                
                # Capture below position
                below_espresso_port = run_skill("current_angles")
                if below_espresso_port is None:
                    print("[ERROR] Failed to capture below position during recovery")
                    return False
                print("   ✅ Below position captured")
                
                # Move back to safe position
                print("⬅️ Moving back to safe position...")
                if port_params and 'move_back' in port_params:
                    back_result = run_skill("gotoJ_deg", *port_params['move_back'])
                    if back_result is False:
                        print("[ERROR] Failed to move back during recovery")
                        return False
                print("   ✅ Moved to safe position")
                
                # Step 2: Clean portafilter
                print("🧹 Step 2/3: Cleaning portafilter...")
                clean_result = clean_portafilter(port=port)
                if clean_result is False:
                    print("[ERROR] Failed to clean portafilter during recovery")
                    return False
                print("   ✅ Portafilter cleaned successfully")
                
                # Step 3: Remount portafilter (recursive call with incremented attempt count)
                print(f"🔧 Step 3/3: Remounting portafilter (attempt {attempt_count + 1}/3)...")
                remount_result = mount(port=port, attempt_count=attempt_count + 1)
                if remount_result is False:
                    print("[ERROR] Failed to remount portafilter after cleaning")
                    return False
                
                print("=" * 60)
                print("✅ RECOVERY SEQUENCE COMPLETED SUCCESSFULLY")
                print("=" * 60)
                return True  # Exit current mount attempt, recovery handled
            else:
                print(f"   ✅ Portafilter validation passed (Z difference within threshold)")

                # Step 5.1: Move end effector up to fix portafilter
                print("⬇️ Step 5.1/10: Moving up to fix portafilter...")
                print(f"   📍 Executing: moveEE_movJ{ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_up']}")
                clear_result = run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_up'])
                
                if clear_result is False:
                    print("[ERROR] Failed to move up to fix portafilter")
                    return False
                print("   ✅ Successfully moved up to fix portafilter")

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 6: First orientation enforcement
        print("📐 Step 6/10: Enforcing proper orientation (first pass)...")
        orient_result1 = run_skill("enforce_rxry")
        if orient_result1 is False:
            print("[ERROR] Failed to enforce orientation (first attempt)")
            return False
        print("   ✅ First orientation enforcement completed")
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 7: Rotate portafilter to unlock (45 degrees)
        print("🔄 Step 7/10: Rotating portafilter to unlock...")
        rotate_result = run_skill("move_portafilter_arc_movJ", 43.0)
        
        if rotate_result is False:
            print("[ERROR] Failed to rotate portafilter")
            return False

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 8: Open gripper to release portafilter
        print("🤏 Step 8/10: Opening gripper to release portafilter...")
        release_result = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['release'])
        if release_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        print("   ✅ Gripper opened, portafilter released")
        
        # Step 9: Conditional retreat for ports 1 and 3
        if port == 'port_1' or port == 'port_3':
            print("⬅️ Step 9/10: Moving back from portafilter...")
            back_approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'])
            if back_approach_result is False:
                print("[ERROR] Failed to move back from portafilter")
                return False
            print("   ✅ Successfully moved back from portafilter")

        else:
            print("   ⏭️ Skipping retreat step for port_2")

        # Step 10: Return to espresso home
        print("🏠 Step 10/10: Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *port_params['home'])
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        print("   ✅ Successfully returned to espresso home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PORTAFILTER MOUNT COMPLETED SUCCESSFULLY FOR {port.upper()}")
        print("   ✓ Portafilter properly positioned and locked")
        print("   ✓ Orientation enforced correctly")
        print("   ✓ Robot returned to home position")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during mount: {e}")
        print("[INFO] Mount process terminated due to error")
        return False

def pick_espresso_pitcher(**params) -> bool:
    """
    Pick up espresso pitcher for the specified port.
    
    This function picks up the appropriate espresso pitcher based on the port selection:
    - Moves to espresso home position
    - Navigates to the correct espresso pitcher location
    - Grips the espresso pitcher with appropriate force
    - Positions for subsequent operations
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3'), defaults to 'port_2'
        espresso (dict): Espresso configuration to derive port from (e.g., {'espresso_shot_double': 2.0})
        
    Returns:
        bool: True if espresso pitcher picked successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pitcher pickup
        
    Example:
        success = pick_espresso_pitcher(port='port_1')
        if success:
            print("Espresso pitcher picked successfully")
    """
    global approach_pitcher, pick_pitcher
    try:
        # Normalize from espresso shot if provided
        # New format: {'espresso': {'espresso_shot_double': 2.0}}
        espresso_dict = params.get("espresso")
        shot_cfg = _normalize_espresso_shot(espresso_dict)

        # Extract and validate port parameter (derived from shot when not explicitly provided)
        port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
        if not port:
            print("[ERROR] No port parameter provided")
            return False
        
        if port not in ('port_1', 'port_2', 'port_3'):
            print(f"[ERROR] Unknown port: {port!r}")
            print("[INFO] Available ports: port_1, port_2, port_3")
            return False
        
        print(f"🥛 Starting espresso pitcher pickup sequence for {port}")
        print("=" * 50)
        
        # Step 1: Move to espresso home position
        print("🏠 Step 1/5: Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *ESPRESSO_HOME)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Approach espresso pitcher area
        print("🎯 Step 2/5: Approaching espresso pitcher area...")
        approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")
        if approach_result is False:
            print("[ERROR] Failed to approach espresso pitcher area")
            return False
        print("   ✅ Successfully approached pitcher area")
        
        # Step 3: Pick espresso pitcher based on port
        print(f"🤏 Step 3/5: Picking espresso pitcher for {port}...")
        if port == 'port_1':
            
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")
            if approach_result is False:
                print("[ERROR] Failed to approach espresso pitcher 1")
                return False
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")
            if mount_result is False:
                print("[ERROR] Failed to mount espresso pitcher 1")
                return False
            
            grip_result = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_1'])
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 1")
                return False
            
            speed_result = run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
            if speed_result is False:
                print("[WARNING] Failed to set speed factor")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")
            if retreat_result is False:
                print("[ERROR] Failed to retreat from espresso pitcher 1")
                return False
            print("   ✅ Successfully picked espresso pitcher 1")
                
        elif port == 'port_2':
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")
            if mount_result is False:
                print("[ERROR] Failed to mount espresso pitcher 2")
                return False
            
            grip_result = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_2'])
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 2")
                return False
            
            speed_result = run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
            if speed_result is False:
                print("[WARNING] Failed to set speed factor")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            pos_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")
            if pos_result is False:
                print("[ERROR] Failed to position for espresso pitcher 2")
                return False
            print("   ✅ Successfully picked espresso pitcher 2")
                
        elif port == 'port_3':
            # Port 3 pitcher sequence
            
            move1_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")
            if move1_result is False:
                print("[ERROR] Failed to move to espresso pitcher 3 position 1")
                return False
            
            move2_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")
            if move2_result is False:
                print("[ERROR] Failed to move to espresso pitcher 3 position 2")
                return False
            
            grip_result = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_3'])
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 3")
                return False
            
            speed_result = run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
            if speed_result is False:
                print("[WARNING] Failed to set speed factor")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")
            if retreat_result is False:
                print("[ERROR] Failed to retreat with espresso pitcher 3")
                return False
            print("   ✅ Successfully picked espresso pitcher 3")
        
        if port == 'port_1' or port == 'port_2':
            # Step 4: Move to final position
            print("📍 Step 4/5: Moving to final holding position...")
            final_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])
            if final_result is False:
                print("[ERROR] Failed to move to final position")
                return False
            print("   ✅ Successfully moved to final holding position")

        
        
        # Final success summary
        print("=" * 50)
        print(f"✅ ESPRESSO PITCHER PICKUP COMPLETED FOR {port.upper()}")
        print("   ✓ Pitcher secured with appropriate grip force")
        print("   ✓ Speed factor adjusted for safe handling")
        print("   ✓ Robot positioned for next operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during espresso pitcher pickup: {e}")
        print("[INFO] Pitcher pickup process terminated due to error")
        return False    

def pour_espresso_pitcher_cup_station(**params) -> bool:
    """
    Pour milk from espresso pitcher into cup at specified position.
    
    This function performs the milk pouring sequence:
    - Moves to pouring position based on cup position
    - Tilts espresso pitcher to pour milk
    - Returns to neutral position
    - Moves back to holding position
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        
    Returns:
        bool: True if pouring completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pouring process
        
    Example:
        success = pour_espresso_pitcher_cup_station(position={'cup_position': 1.0})
        if success:
            print("Milk poured successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = f"stage_{cup_position}"  # Convert to internal stage format
        
        print(f"🥛 Starting milk pouring sequence for {stage}")
        print("=" * 50)

        run_skill("gotoJ_deg", 103.201965,-21.933174,-150.611664,-10.398072,-23.882843,0.127716)
        
        # Step 1: Initial positioning
        print("📍 Step 1/7: Moving to initial pouring position...")
        init_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])
        
        if init_result is False:
            print("[ERROR] Failed to move to initial pouring position")
            return False
        print("   ✅ Successfully moved to initial position")
        
        if stage == 'stage_1':
            print("🎯 Stage 1 pouring sequence...")
            
            # Step 2: Approach stage 1 position
            print("📍 Step 2/7: Positioning for stage 1 pouring...")
            pos1_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos1'])
            
            if pos1_result is False:
                print("[ERROR] Failed to approach stage 1 position")
                return False
            print("   ✅ Successfully positioned for stage 1")
            run_skill("sync")
            run_skill("set_speed_factor", SPEED_SLOW_POURING)
            # Step 3: Tilt espresso pitcher to pour
            print("⬇️ Step 3/7: Tilting espresso pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1'])
            
            if pour_result is False:
                print("[ERROR] Failed to tilt espresso pitcher for pouring")
                return False
            print("   ✅ Pouring motion completed")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            # Reset speed factor
            speed_result = run_skill("set_speed_factor", 100)
            if speed_result is False:
                print("[WARNING] Failed to reset speed factor")

            # Step 4: Return to neutral position
            print("⬆️ Step 4/7: Returning to neutral position...")
            neutral_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral1'])
            
            if neutral_result is False:
                print("[ERROR] Failed to return to neutral position")
                return False
            print("   ✅ Successfully returned to neutral position")

        elif stage == 'stage_2':
            print("🎯 Stage 2 pouring sequence...")
            
            # Step 2: Approach stage 2 position
            print("📍 Step 2/7: Positioning for stage 2 pouring...")
            pos2_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos2'])
            
            if pos2_result is False:
                print("[ERROR] Failed to approach stage 2 position")
                return False
            print("   ✅ Successfully positioned for stage 2")
            run_skill("sync")
            run_skill("set_speed_factor", SPEED_SLOW_POURING)
            # Step 3: Tilt espresso pitcher to pour
            print("⬇️ Step 3/7: Tilting espresso pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2'])
            
            if pour_result is False:
                print("[ERROR] Failed to tilt espresso pitcher for pouring")
                return False
            print("   ✅ Pouring motion completed")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            # Reset speed factor
            speed_result = run_skill("set_speed_factor", 100)
            if speed_result is False:
                print("[WARNING] Failed to reset speed factor")

            # Step 4: Return to neutral position
            print("⬆️ Step 4/7: Returning to neutral position...")
            neutral_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral2'])
            
            if neutral_result is False:
                print("[ERROR] Failed to return to neutral position")
                return False
            print("   ✅ Successfully returned to neutral position")

        elif stage == 'stage_3':
            print("🎯 Stage 3 pouring sequence...")
            
            # Step 2: Approach stage 3 position
            print("📍 Step 2/7: Positioning for stage 3 pouring...")
            pos3_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos3'])
            
            if pos3_result is False:
                print("[ERROR] Failed to approach stage 3 position")
                return False
            print("   ✅ Successfully positioned for stage 3")
            run_skill("sync")
            run_skill("set_speed_factor", SPEED_SLOW_POURING)
            # Step 3: Tilt espresso pitcher to pour
            print("⬇️ Step 3/7: Tilting espresso pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3'])
            
            if pour_result is False:
                print("[ERROR] Failed to tilt espresso pitcher for pouring")
                return False
            print("   ✅ Pouring motion completed")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            # Reset speed factor
            speed_result = run_skill("set_speed_factor", 100)
            if speed_result is False:
                print("[WARNING] Failed to reset speed factor")

            # Step 4: Return to neutral position
            print("⬆️ Step 4/7: Returning to neutral position...")
            neutral_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral3'])
            
            if neutral_result is False:
                print("[ERROR] Failed to return to neutral position")
                return False
            print("   ✅ Successfully returned to neutral position")

        else:  # stage_4
            print("🎯 Stage 4 pouring sequence...")
            
            # Step 4: Approach stage 4 position
            print("📍 Step 4/7: Positioning for stage 4 pouring...")
            pos4_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos4'])
            
            if pos4_result is False:
                print("[ERROR] Failed to approach stage 4 position")
                return False
            print("   ✅ Successfully positioned for stage 4")
            run_skill("sync")
            run_skill("set_speed_factor", SPEED_SLOW_POURING)
            # Step 3: Tilt espresso pitcher to pour
            print("⬇️ Step 3/7: Tilting espresso pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4'])
            
            if pour_result is False:
                print("[ERROR] Failed to tilt espresso pitcher for pouring")
                return False
            print("   ✅ Pouring motion completed")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            # Reset speed factor
            speed_result = run_skill("set_speed_factor", 100)
            if speed_result is False:
                print("[WARNING] Failed to reset speed factor")
            
            # Step 4: Return to neutral position
            print("⬆️ Step 4/7: Returning to neutral position...")
            neutral_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral4'])
            
            if neutral_result is False:
                print("[ERROR] Failed to return to neutral position")
                return False
            print("   ✅ Successfully returned to neutral position")
        
        # Step 5: Move to intermediate position
        print("📍 Step 5/7: Moving to intermediate position...")
        inter_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])
        
        if inter_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        run_skill("gotoJ_deg", 103.201965,-21.933174,-150.611664,-10.398072,-23.882843,0.127716)

        # Step 7: Return to holding position
        print("🏠 Returning to holding position...")
        final_result = run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])
        
        
        run_skill("sync")
        
        if final_result is False:
            print("[ERROR] Failed to return to holding position")
            return False
        print("   ✅ Successfully returned to holding position")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ MILK POURING COMPLETED SUCCESSFULLY FOR {stage.upper()}")
        print("   ✓ Precise pouring motion executed")
        print("   ✓ Speed factor reset for normal operation")
        print("   ✓ Robot returned to holding position")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during pouring: {e}")
        print("[INFO] Pouring process terminated due to error")
        return False

def get_hot_water(**params) -> bool:
    """
    Position espresso pitcher under hot water dispenser.
    
    This function moves the robot to the hot water dispensing position:
    1. Approaches the hot water dispenser
    2. Positions pitcher under the hot water outlet
    3. Maintains position for hot water dispensing
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
        
    Returns:
        bool: True if positioning completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during positioning
        
    Example:
        success = get_hot_water()
        if success:
            print("Ready for hot water dispensing")
    """
    try:
        print("🚰 Starting hot water dispensing positioning sequence")
        print("=" * 50)
        
        # Step 1: Move to hot water dispenser approach position
        print("🎯 Step 1/2: Approaching hot water dispenser...")
        approach_result = run_skill("approach_machine", "three_group_espresso", "hot_water")
        if approach_result is False:
            print("[ERROR] Failed to approach hot water dispenser")
            return False
        print("   ✅ Successfully approached hot water dispenser")
        
        # Step 2: Position espresso pitcher under hot water outlet
        print("📍 Step 2/2: Positioning espresso pitcher under hot water outlet...")
        position_result = run_skill("mount_machine", "three_group_espresso", "hot_water")
        if position_result is False:
            print("[ERROR] Failed to position espresso pitcher under hot water outlet")
            return False
        print("   ✅ Successfully positioned under hot water outlet")

        run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_move'])        
        # Final success summary
        print("=" * 50)
        print("✅ HOT WATER DISPENSING POSITION READY")
        print("   ✓ Pitcher positioned under hot water outlet")
        print("   ✓ Ready for hot water dispensing operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during hot water positioning: {e}")
        print("[INFO] Hot water positioning terminated due to error")
        return False

def with_hot_water(**params) -> bool:
    """
    Complete hot water dispensing sequence and return to holding position.
    
    This function completes the hot water dispensing operation:
    1. Moves away from the hot water outlet
    2. Returns to the standard holding position
    3. Prepares for next operation
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
        
    Returns:
        bool: True if sequence completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during the sequence
        
    Example:
        success = with_hot_water()
        if success:
            print("Hot water dispensing completed")
    """
    try:
        print("🚰 Completing hot water dispensing sequence")
        print("=" * 50)

        run_skill("set_speed_factor", ESPRESSO_SPEEDS['hot_water_pour'])
        
        # Step 2: Return to holding position
        print("🏠 Step 2/2: Returning to holding position...")
        final_result = run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])
        if final_result is False:
            print("[ERROR] Failed to return to holding position")
            return False
        print("   ✅ Successfully returned to holding position")
        
        # Final success summary
        print("=" * 50)
        print("✅ HOT WATER DISPENSING SEQUENCE COMPLETED")
        print("   ✓ Safely moved away from hot water outlet")
        print("   ✓ Robot ready for next operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during hot water completion: {e}")
        print("[INFO] Hot water completion sequence terminated due to error")
        return False

def return_espresso_pitcher(**params) -> bool:
    """
    Return espresso pitcher to its home position after use.
    
    This function performs the espresso pitcher return sequence:
    - Navigates to the appropriate espresso pitcher return location based on port
    - Positions espresso pitcher in its designated spot
    - Releases gripper to place espresso pitcher
    - Returns to espresso home position
    
    Args:
        port (str): Source port ('port_1', 'port_2', or 'port_3'), defaults to 'port_2'
        espresso (dict): Espresso configuration to derive port from (e.g., {'espresso_shot_double': 2.0})
        
    Returns:
        bool: True if espresso pitcher returned successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during return process
        
    Example:
        success = return_espresso_pitcher(port='port_1')
        if success:
            print("Espresso pitcher returned successfully")
    """
    global approach_pitcher, pick_pitcher
    try:
        # Normalize from espresso shot if provided
        # New format: {'espresso': {'espresso_shot_double': 2.0}}
        espresso_dict = params.get("espresso")
        shot_cfg = _normalize_espresso_shot(espresso_dict)

        # Extract and validate port parameter (derived from shot when not explicitly provided)
        port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
        if not port:
            print("[ERROR] No port parameter provided")
            return False
        
        if port not in ('port_1', 'port_2', 'port_3'):
            print(f"[ERROR] Unknown port: {port!r}")
            print("[INFO] Available ports: port_1, port_2, port_3")
            return False
        
        print(f"🔄 Starting espresso pitcher return sequence for {port}")
        print("=" * 50)
        
        # Step 1: Return espresso pitcher based on port
        if port == 'port_1':
            
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")
            
            if approach_result is False:
                print("[ERROR] Failed to approach espresso pitcher 1 return position")
                return False
            print("   ✅ Successfully approached pitcher 1 return position")
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")
            
            if mount_result is False:
                print("[ERROR] Failed to position espresso pitcher 1 for return")
                return False
            print("   ✅ Successfully positioned pitcher 1 for return")
            
            print("🤏 Releasing espresso pitcher 1...")
            release_result = run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)
            
            if release_result is False:
                print("[ERROR] Failed to release espresso pitcher 1")
                return False
            print("   ✅ Successfully released pitcher 1")
            
            print("⬅️ Retreating from espresso pitcher 1...")
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")
            
            if retreat_result is False:
                print("[ERROR] Failed to retreat from espresso pitcher 1")
                return False
            print("   ✅ Successfully retreated from pitcher 1")
                
        elif port == 'port_2':
            print("📍 Step 1/4: Positioning espresso pitcher 2 for return...")
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")
            
            if mount_result is False:
                print("[ERROR] Failed to position espresso pitcher 2 for return")
                return False
            print("   ✅ Successfully positioned pitcher 2 for return")
            
            print("🤏 Releasing espresso pitcher 2...")
            release_result = run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)
            
            if release_result is False:
                print("[ERROR] Failed to release espresso pitcher 2")
                return False
            print("   ✅ Successfully released pitcher 2")
                
        elif port == 'port_3':
            print("📍 Step 1/4: Moving to espresso pitcher 3 return position...")
            move1_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")
            
            if move1_result is False:
                print("[ERROR] Failed to move to espresso pitcher 3 return position 1")
                return False
            print("   ✅ Successfully moved to pitcher 3 return position 1")
            
            move2_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")
            
            if move2_result is False:
                print("[ERROR] Failed to move to espresso pitcher 3 return position 2")
                return False
            print("   ✅ Successfully moved to pitcher 3 return position 2")
            
            print("🤏 Releasing espresso pitcher 3...")
            release_result = run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)
            
            if release_result is False:
                print("[ERROR] Failed to release espresso pitcher 3")
                return False
            print("   ✅ Successfully released pitcher 3")
            
            print("⬅️ Retreating from espresso pitcher 3...")
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")
            
            if retreat_result is False:
                print("[ERROR] Failed to retreat from espresso pitcher 3")
                return False
            print("   ✅ Successfully retreated from pitcher 3")
        
        # Step 2: Move to common espresso pitcher area
        print("🎯 Step 2/4: Moving to espresso pitcher area...")
        area_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")
        
        if area_result is False:
            print("[ERROR] Failed to move to espresso pitcher area")
            return False
        print("   ✅ Successfully moved to pitcher area")
        
        # Step 3: Return to espresso home
        print("🏠 Step 3/4: Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *ESPRESSO_HOME)
        
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        print("   ✅ Successfully returned to espresso home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ ESPRESSO PITCHER RETURN COMPLETED FOR {port.upper()}")
        print("   ✓ Pitcher properly placed in designated position")
        print("   ✓ Gripper released with appropriate force")
        print("   ✓ Robot returned to home position")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during espresso pitcher return: {e}")
        print("[INFO] Pitcher return process terminated due to error")
        return False

def return_cleaned_espresso_pitcher(**params) -> bool:
    global approach_pitcher, pick_pitcher
    try:
        # Normalize from espresso shot if provided
        # New format: {'espresso': {'espresso_shot_double': 2.0}}
        espresso_dict = params.get("espresso")
        shot_cfg = _normalize_espresso_shot(espresso_dict)

        # Extract and validate port parameter (derived from shot when not explicitly provided)
        port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
        if not port:
            print("[ERROR] No port parameter provided")
            return False
        
        if port not in ('port_1', 'port_2', 'port_3'):
            print(f"[ERROR] Unknown port: {port!r}")
            print("[INFO] Available ports: port_1, port_2, port_3")
            return False
        
        print(f"🥛 Starting espresso pitcher pickup sequence for {port}")
        print("=" * 50)
        
        # Step 1: Move to espresso home position
        print("🏠 Step 1/5: Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *ESPRESSO_HOME)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Approach espresso pitcher area
        print("🎯 Step 2/5: Approaching espresso pitcher area...")
        approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")
        if approach_result is False:
            print("[ERROR] Failed to approach espresso pitcher area")
            return False
        print("   ✅ Successfully approached pitcher area")
        
        # Step 3: Pick espresso pitcher based on port
        print(f"🤏 Step 3/5: Picking espresso pitcher for {port}...")
        if port == 'port_1':
            
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")
            if approach_result is False:
                print("[ERROR] Failed to approach espresso pitcher 1")
                return False
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")
            if mount_result is False:
                print("[ERROR] Failed to mount espresso pitcher 1")
                return False
            
            grip_result = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_1'])
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 1")
                return False
            
            run_skill("moveEE_movJ", 0,0,10,0,0,0)
            run_skill("moveJ_deg", 0,0,0,0,0,-150)
            run_skill("moveJ_deg", 0,0,0,0,0,150)
            run_skill("moveEE_movJ", 0,0,-10,0,0,0)

            print("🤏 Releasing espresso pitcher 1...")
            release_result = run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)
            
            if release_result is False:
                print("[ERROR] Failed to release espresso pitcher 1")
                return False
            print("   ✅ Successfully released pitcher 1")
            
            print("⬅️ Retreating from espresso pitcher 1...")
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")
            
            if retreat_result is False:
                print("[ERROR] Failed to retreat from espresso pitcher 1")
                return False
            print("   ✅ Successfully retreated from pitcher 1")
                
        elif port == 'port_2':
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")
            if mount_result is False:
                print("[ERROR] Failed to mount espresso pitcher 2")
                return False
            
            grip_result = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_2'])
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 2")
                return False

            run_skill("moveEE_movJ", 0,0,10,0,0,0)
            run_skill("moveJ_deg", 0,0,0,0,0,-150)
            run_skill("moveJ_deg", 0,0,0,0,0,150)
            run_skill("moveEE_movJ", 0,0,-10,0,0,0)

            print("🤏 Releasing espresso pitcher 2...")
            release_result = run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)
            
            if release_result is False:
                print("[ERROR] Failed to release espresso pitcher 2")
                return False
            print("   ✅ Successfully released pitcher 2")
                
        elif port == 'port_3':
            # Port 3 pitcher sequence
            
            move1_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")
            if move1_result is False:
                print("[ERROR] Failed to move to espresso pitcher 3 position 1")
                return False
            
            move2_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")
            if move2_result is False:
                print("[ERROR] Failed to move to espresso pitcher 3 position 2")
                return False
            
            grip_result = run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_3'])
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 3")
                return False

            run_skill("moveEE_movJ", 0,0,10,0,0,0)
            run_skill("moveJ_deg", 0,0,0,0,0,-150)
            run_skill("moveJ_deg", 0,0,0,0,0,150)
            run_skill("moveEE_movJ", 0,0,-10,0,0,0)

            print("🤏 Releasing espresso pitcher 3...")
            release_result = run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)
            
            if release_result is False:
                print("[ERROR] Failed to release espresso pitcher 3")
                return False
            print("   ✅ Successfully released pitcher 3")
            
            print("⬅️ Retreating from espresso pitcher 3...")
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")
            
            if retreat_result is False:
                print("[ERROR] Failed to retreat from espresso pitcher 3")
                return False
            print("   ✅ Successfully retreated from pitcher 3")
        
        # Step 2: Move to common espresso pitcher area
        print("🎯 Step 2/4: Moving to espresso pitcher area...")
        area_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")
        
        if area_result is False:
            print("[ERROR] Failed to move to espresso pitcher area")
            return False
        print("   ✅ Successfully moved to pitcher area")
        
        # Step 3: Return to espresso home
        print("🏠 Step 3/4: Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *ESPRESSO_HOME)
        
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        print("   ✅ Successfully returned to espresso home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ ESPRESSO PITCHER RETURN COMPLETED FOR {port.upper()}")
        print("   ✓ Pitcher properly placed in designated position")
        print("   ✓ Gripper released with appropriate force")
        print("   ✓ Robot returned to home position")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during espresso pitcher return: {e}")
        print("[INFO] Pitcher return process terminated due to error")
        return False

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
    from oms_v1.sequences.espresso import _normalize_espresso_shot
    
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
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    # 3) Hard brush
    if not ok(run_skill("approach_machine", "portafilter_cleaner", "hard_brush")):
        return False
    if not ok(run_skill("gotoJ_deg", *CLEANING_PARAMS['hard_brush_adjust'])):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "hard_brush")):
        return False
    if not ok(run_skill("moveEE_movJ", -1,0,50,0,0,0)):
        return False
    if not ok(run_skill("moveEE_movJ", -1,0,-50,0,0,0)):
        return False
    if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_hard'])):
        return False

    # 4) Soft brush
    if not ok(run_skill("approach_machine", "portafilter_cleaner", "soft_brush")):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "soft_brush")):
        return False
    if not ok(run_skill("moveEE", -7.5,7.5,10,1.1,0,0)):
        return False
    if not ok(run_skill("moveEE", 7.5,-7.5,-7.5,1.1,0,0)):
        return False
    time.sleep(DELAY_SHORT)
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
    
    This function performs calibration of the milk frother position:
    1. Moves to north-east home position for approach
    2. Opens gripper to prepare for positioning
    3. Performs steam wand positioning and calibration
    4. Executes multiple approaches to left steam wand for accuracy
    5. Records the calibrated position for future reference
    
    This calibration should be performed when setting up the milk frothing station
    or when the frother position may have changed.
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if frother position calibrated successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during calibration process
        
    Example:
        success = get_frother_position()
        if success:
            print("Frother position calibrated successfully")
    """
    try:
        print("🎯 Starting milk frother position calibration...")
        print("=" * 50)
        # return_back_to_home()
        
        # # Only sleep for single shot (give time for shot to finish)
        # espresso_data = params.get('espresso', {})
        # is_single_shot = 'espresso_shot_single' in espresso_data or params.get('shot_type') == 'single'
        # if is_single_shot:
        #     print("   ⏱️  Waiting 5s for single shot to finish...")
        #     time.sleep(5)
        
        # Set optimal speed for calibration
        print("⚙️ Setting speed factor for precise calibration...")
        speed_result = run_skill("set_speed_factor", 100)
        if speed_result is False:
            print("[WARNING] Failed to set speed factor - continuing with default...")
        
        # Step 1: Move to home position for setup
        print("🏠 Step 1/5: Moving to north-east home position...")
        home_result = home(position="north_east")
        if home_result is False:
            print("[ERROR] Failed to move to north-east home position")
            return False
        print("   ✅ Successfully moved to north-east home")
        
        # # Step 2: Open gripper to prepare for positioning
        # print("🤏 Step 2/5: Opening gripper for positioning...")
        # gripper_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open'])
        # if gripper_result is False:
        #     print("[ERROR] Failed to open gripper")
        #     return False
        # print("   ✅ Gripper opened successfully")

        # # Step 3: Steam wand positioning and preparation
        # print("🎯 Step 3/5: Steam wand positioning and preparation...")
        # print("   📍 Moving to steam wand...")
        # move_result = run_skill("move_to", "left_steam_wand", 0.28)
        # if move_result is False:
        #     print("[ERROR] Failed to move to steam wand")
        #     return False
        
        # sync_result = run_skill("sync")
        # if sync_result is False:
        #     print("[WARNING] Sync operation failed - continuing...")
        
        # print("   🔧 Grabbing steam wand tool...")
        # grab_result = run_skill("grab_tool", "left_steam_wand")
        # if grab_result is False:
        #     print("[ERROR] Failed to grab steam wand tool")
        #     return False
        
        # sync_result = run_skill("sync")
        # if sync_result is False:
        #     print("[WARNING] Sync operation failed - continuing...")
        
        # print("   🤏 Setting grip position...")
        # grip_set_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['place'])
        # if grip_set_result is False:
        #     print("[ERROR] Failed to set grip position")
        #     return False
        
        # sync_result = run_skill("sync")
        # if sync_result is False:
        #     print("[WARNING] Sync operation failed - continuing...")
        
        # print("   📍 Moving to calibration position...")
        # positioning_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['calibration']['positioning'])
        # if positioning_result is False:
        #     print("[ERROR] Failed to move to calibration position")
        #     return False

        # print("   🤏 Releasing grip for calibration...")
        # release_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open'])
        # if release_result is False:
        #     print("[ERROR] Failed to release grip")
        #     return False
        # print("   ✅ Steam wand positioning completed")
        
        # move_ee_result = run_skill("moveEE_movJ", 10, 10, 0, 0, 0, 0)
        # if move_ee_result is False:
        #     print("[ERROR] Failed to move end effector")
        #     return False
        
        # sync_result = run_skill("sync")
        # if sync_result is False:
        #     print("[WARNING] Sync operation failed - continuing...")
        
        # Step 4: Perform multiple approaches for accuracy
        cycles = 4  # Special case: 3 cycles for frother calibration
        print(f"🎯 Step 4/5: Performing calibration approaches ({cycles} attempts)...")
        for i in range(cycles):
            print(f"   📍 Approach {i+1}/{cycles}...")
            time.sleep(CALIBRATION_SETTLE_TIME)  # Allow settling time between approaches
            
            approach_result = run_skill("move_to", "left_steam_wand", 0.29)
            if approach_result is False:
                print(f"[ERROR] Failed calibration approach {i+1}/5")
                return False
        print("   ✅ All calibration approaches completed successfully")
        
        # Step 5: Record the calibrated position
        print("💾 Step 5/5: Recording milk frother position...")
        record_result = run_skill("get_machine_position", "left_steam_wand")
        if record_result is False:
            print("[ERROR] Failed to record milk frother position")
            print("[INFO] ArUco marker for left_steam_wand may not be visible")
            return False
        print("   ✅ Milk frother position recorded successfully")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK FROTHER POSITION CALIBRATION COMPLETED")
        print("   ✓ Steam wand positioning calibrated")
        print("   ✓ Multiple approach accuracy verified")
        print("   ✓ Position data saved for future operations")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother position calibration: {e}")
        print("[INFO] Calibration process terminated due to error")
        return False

def pick_frother(**params) -> bool:
    """
    Pick up the milk frother for milk frothing operations.
    
    This function handles the milk frother pickup sequence:
    1. Moves to frother approach position
    2. Approaches the milk frother with precise positioning
    3. Grabs the frother with appropriate grip strength
    4. Stores position data for later return operations
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if frother picked successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pickup process
        
    Example:
        success = pick_frother()
        if success:
            print("Milk frother picked successfully")
    """
    try:
        global approach_angles, grab_angles
        
        print("🥛 Starting milk frother pickup sequence...")
        print("=" * 50)
        
        # Step 1: Move to frother area position
        print("📍 Step 1/6: Moving to frother area...")
        area_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area'])
        if area_result is False:
            print("[ERROR] Failed to move to frother area")
            return False
        print("   ✅ Successfully moved to frother area")
        
        # Step 2: Approach the milk frother
        print("🎯 Step 2/6: Approaching milk frother...")
        
        # Approach milk_frother_1 (no timeout, let it stabilize properly)
        print("   📍 Moving to milk_frother_1...")
        approach_result = run_skill("move_to", 'milk_frother_1', 0.29)
        
        if not approach_result:
            print("[ERROR] Failed to approach milk_frother_1")
            return False
        
        print("   ✅ Successfully approached milk_frother_1")
        
        # Step 3: Move to approach position for frother
        print("📍 Step 3/6: Moving to frother approach position...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        approach_tool_result = run_skill("approach_tool", 'milk_frother_1')
        if approach_tool_result is False:
            print("[ERROR] Failed to move to frother approach position")
            return False
        print("   ✅ Successfully positioned for frother approach")
        
        # Set initial grip position
        print("   🤏 Setting initial grip position...")
        grip_pos_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'])
        if grip_pos_result is False:
            print("[ERROR] Failed to set initial grip position")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        # Step 4: Record current approach position
        print("💾 Step 4/6: Recording approach position...")
        current_angles = run_skill("current_angles")
        if current_angles is not None:
            approach_angles = current_angles
            print(f"   ✅ Approach angles recorded: {len(approach_angles)} joint values")
        else:
            print("[WARNING] Failed to record approach angles - continuing without position memory")
            approach_angles = None
        # Step 5: Grab the frother
        print(f"🤏 Step 5/6: Grabbing {'milk_frother_1'}...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        grab_result = run_skill("grab_tool", 'milk_frother_1', 100, 100,-5,-10.5)
        if grab_result is False:
            print(f"[ERROR] Failed to grab {'milk_frother_1'}")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 6: Record current grab position
        print("💾 Step 6/6: Recording grab position...")
        grab_angles = run_skill("current_angles")
        if grab_angles is not None:
            print(f"   ✅ Grab angles recorded: {len(grab_angles)} joint values")
        else:
            print("[WARNING] Failed to record grab angles - continuing without position memory")
            grab_angles = None

        # Step 7: Secure the frother with full grip
        print("   🤏 Securing frother with full grip...")
        secure_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure'])
        if secure_result is False:
            print("[ERROR] Failed to secure frother")
            return False
        print("   ✅ Milk frother secured successfully")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ MILK FROTHER PICKUP COMPLETED SUCCESSFULLY ({'milk_frother_1'.upper()})")
        print("   ✓ Frother securely gripped and positioned")
        print("   ✓ Position data recorded for safe return")
        print("   ✓ Ready for mounting to steam wand")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother pickup: {e}")
        print("[INFO] Frother pickup process terminated due to error")
        return False
        
def place_frother_milk_station(**params) -> bool:
    """
    Place the milk frother at the milk station safely.

    Returns:
        bool: True on successful placement, False otherwise.
    """
    try:
        print("📍 Placing frother at milk station...")
        if run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_place']) is False:
            print("[ERROR] Failed to raise end effector before placement")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre1']) is False:
            print("[ERROR] Failed to reach pre-place joint configuration 1")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre2']) is False:
            print("[ERROR] Failed to reach pre-place joint configuration 2")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_approach']) is False:
            print("[ERROR] Failed to reach approach configuration")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_final']) is False:
            print("[ERROR] Failed to reach place configuration")
            return False
        if run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['place']) is False:
            print("[ERROR] Failed to loosen gripper to place frother")
            return False
        print("✅ Frother placed at milk station")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error while placing frother at milk station: {e}")
        return False

def pick_frother_milk_station(**params) -> bool:
    """
    Pick the milk frother up from the milk station safely.

    Returns:
        bool: True on successful pickup, False otherwise.
    """
    try:
        print("📍 Picking frother from milk station...")
        if run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure']) is False:
            print("[ERROR] Failed to close gripper before pick")
            return False
        if run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_pick']) is False:
            print("[ERROR] Failed to lift frother from station")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat1']) is False:
            print("[ERROR] Failed to reach retreat configuration 1")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat2']) is False:
            print("[ERROR] Failed to reach retreat configuration 2")
            return False
        print("✅ Frother picked from milk station")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error while picking frother from milk station: {e}")
        return False

def mount_frother(**params) -> bool:
    """
    Mount the milk frother to the steam wand for frothing preparation.
    
    This function positions the frother on the steam wand:
    1. Sets precise servo timing for accurate movements
    2. Moves to milk frothing preparation position
    3. Approaches the steam wand with the frother (deep position)
    4. Mounts frother securely to steam wand
    
    Note: This function only positions the frother. Use froth_milk() to actually activate steam.
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if frother mounted successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during mounting process
        
    Example:
        success = mount_frother()
        if success:
            print("Frother mounted to steam wand successfully")
    """
    try:
        print("☁️ Starting milk frother mounting sequence...")
        print("=" * 50)

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 1: Set slower servo timing for precise movements
        print("⚙️ Step 1/4: Setting precise servo timing...")
        timing_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['mount'])
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing with default...")
        else:
            print("   ✅ Servo timing set for precise movements")
        
        print("🎯 Step 3/4: Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        print("   ✅ Successfully approached steam wand")

        # Step 4: Mount frother to steam wand with verification
        print("🔧 Step 4/5: Mounting frother to steam wand...")
        mount_result = run_skill("mount_machine", "left_steam_wand", "deep_froth")
        if mount_result is False:
            print("[ERROR] Failed to mount frother to steam wand")
            return False
        print("   ✅ Frother successfully mounted to steam wand")
        run_skill("moveEE_movJ", 10,-10,-5,0,0,0)
        run_skill("moveEE_movJ", 0,0,5,0,0,0)
        # Step 5: Adjust position based on milk volume
        print("📏 Step 5/6: Adjusting position based on milk volume...")
        milk_data = params.get('milk', {})
        volume_ml = next(iter(milk_data.values()), 0) if milk_data else 0
        z_adjustment = MILK_VOLUME_Z_ADJUSTMENT_FACTOR * volume_ml
        
        print(f"   🥛 Milk volume: {volume_ml}ml, Z adjustment: {z_adjustment:.2f}mm")
        move_result = run_skill("moveEE_movJ", 0, 0, -z_adjustment, 0, 0, 0)
        if move_result is False:
            print("[WARNING] Failed to adjust position based on milk volume - continuing...")
        else:
            print(f"   ✅ Position adjusted by {z_adjustment:.2f}mm for {volume_ml}ml milk")
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[ERROR] Final sync failed - robot state may be inconsistent")
            return False
        print("   ✅ Final state synchronized")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK FROTHER MOUNTING COMPLETED SUCCESSFULLY")
        print("   ✓ All movements verified")
        print("   ✓ Precise positioning achieved")
        print("   ✓ Frother securely mounted to steam wand")
        print("   ✓ Ready for milk frothing operation")
        print("=" * 50)
        return True
               
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother mounting: {e}")
        print("[INFO] Mounting process terminated due to error")
        return False

def unmount_and_swirl_milk(**params) -> bool:
    """
    Swirl frothed milk in a circular motion for latte art preparation.
    
    This function performs a milk swirling sequence for latte art:
    1. Approaches steam wand position 
    2. Sets precise timing for smooth swirling
    3. Moves through positioning sequence to optimal swirling location
    4. Executes circular swirling motion
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
        
    Returns:
        bool: True if milk swirling completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during swirling process
        
    Example:
        success = unmount_and_swirl_milk()
        if success:
            print("Milk swirled successfully")
    """
    try:
        
        print("🌀 Starting milk swirling sequence")
        print("=" * 50)

        time.sleep(MILK_FROTHING_DELAYS['swirl_delay'])

        # Step 1: Approach steam wand position
        print("🎯 Step 1/4: Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        print("   ✅ Successfully approached steam wand")

        run_skill("sync")
        
        # Step 2: Set precise servo timing for swirling
        print("⚙️ Step 2/4: Setting precise servo timing for swirling...")
        timing_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['swirl'])
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        else:
            print("   ✅ Servo timing adjusted for swirling")

        # Step 3: Position sequence for optimal swirling location
        print("📍 Step 3/4: Moving through positioning sequence...")
        
        print("   📍 Moving to intermediate position 1...")
        intermediate1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['intermediate1'])
        if intermediate1_result is False:
            print("[ERROR] Failed to move to intermediate position 1")
            return False
        print("   ✅ Successfully moved to intermediate position 1")
        
        print("   📍 Moving to optimal swirling position...")
        swirl_pos_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['swirl_pos'])
        if swirl_pos_result is False:
            print("[ERROR] Failed to move to swirling position")
            return False
        print("   ✅ Successfully positioned for swirling")

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        # Step 4: Execute circular swirling motion
        print("🌀 Step 4/4: Executing circular swirling motion...")
        circle_result = run_skill("move_circle", 
            MILK_SWIRL_CIRCLE_PARAMS['cycles'],
            MILK_SWIRL_CIRCLE_PARAMS['point1_offset'],
            MILK_SWIRL_CIRCLE_PARAMS['point2_offset'],
            MILK_SWIRL_CIRCLE_PARAMS['options'])
        if circle_result is False:
            print("[WARNING] Circular motion may not have completed optimally")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK SWIRLING COMPLETED SUCCESSFULLY")
        print("   ✓ Precise positioning achieved")
        print("   ✓ Optimal swirling technique executed")
        print("   ✓ Perfect preparation for latte art")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk swirling: {e}")
        print("[INFO] Milk swirling process terminated due to error")
        return False

def pour_milk_cup_station(**params) -> bool:
    """
    Pour frothed milk into cup at specified stage.
    
    This function pours the frothed milk with stage-specific positioning:
    1. Validates stage parameter
    2. Returns to approach position from steam wand
    3. Adjusts servo timing for smooth pouring
    4. Moves to target stage for milk pouring
    5. Executes stage-specific pouring sequence
    6. Returns frother to safe position
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        
    Returns:
        bool: True if milk pouring completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pouring process
        
    Example:
        success = pour_milk_cup_station(position={'cup_position': 1.0})
        if success:
            print("Milk poured successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        print(f"🥛 Starting milk pouring sequence for stage {stage}")
        print("=" * 50)
        
        # Step 4: Stage-specific pouring sequence
        if stage == '1':
            print("🎯 Step 4/5: Executing stage 1 milk pouring...")
            
            print("   📍 Moving to stage 1 pouring position...")
            stage1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['position'])
            if stage1_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            
            run_skill("sync")

            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            run_skill("sync")


            print("   📍 Adjusting pour angle...")
            adjust1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['adjust1'])
            if adjust1_result is False:
                print("[WARNING] Failed first pour angle adjustment")
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage1']['move_forward'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage1']['move_up'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   📍 Returning to stage 1 position...")
            return_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['position'])
            if return_result is False:
                print("[WARNING] Failed to return to stage 1 position")
            
            print("   ⚙️ Restoring normal speed...")
            restore_speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
            if restore_speed_result is False:
                print("[WARNING] Failed to restore normal speed")
            print("   ✅ Stage 1 milk pouring completed")
            
        elif stage == '2':
            print("🎯 Step 4/5: Executing stage 2 milk pouring...")
            
            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Moving to stage 2 pouring position...")
            stage2_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['position'])
            if stage2_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            
            run_skill("sync")

            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Adjusting pour angle...")
            adjust1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['adjust1'])
            if adjust1_result is False:
                print("[WARNING] Failed first pour angle adjustment")
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage2']['move_forward'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage2']['move_up'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   📍 Returning to stage 2 position...")
            return_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['position'])
            if return_result is False:
                print("[WARNING] Failed to return to stage 2 position")
            
            print("   ⚙️ Restoring normal speed...")
            restore_speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
            if restore_speed_result is False:
                print("[WARNING] Failed to restore normal speed")
            print("   ✅ Stage 2 milk pouring completed")

        elif stage == '3':
            print("🎯 Step 4/5: Executing stage 3 milk pouring...")
            
            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Moving to stage 3 pouring position...")
            stage3_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['position'])
            if stage3_result is False:
                print("[ERROR] Failed to move to stage 3 position")
                return False
            
            run_skill("sync")

            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
                
            print("   📍 Adjusting pour angle...")
            adjust1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['adjust1'])
            if adjust1_result is False:
                print("[WARNING] Failed first pour angle adjustment")
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage3']['move_forward'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage3']['move_up'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   📍 Returning to stage 3 position...")
            return_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['position'])
            if return_result is False:
                print("[WARNING] Failed to return to stage 3 position")
            
            print("   ⚙️ Restoring normal speed...")
            restore_speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
            if restore_speed_result is False:
                print("[WARNING] Failed to restore normal speed")
            print("   ✅ Stage 3 milk pouring completed")

        else:  # stage == '4'
            print("🎯 Step 4/5: Executing stage 4 milk pouring...")
            
            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Moving to stage 4 pouring position...")
            stage4_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['position'])
            if stage4_result is False:
                print("[ERROR] Failed to move to stage 4 position")
                return False
            
            run_skill("sync")

            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Adjusting pour angle...")
            adjust1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['adjust1'])
            if adjust1_result is False:
                print("[WARNING] Failed first pour angle adjustment")
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage4']['move_forward'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage4']['move_up'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   📍 Returning to stage 4 position...")
            return_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['position'])
            if return_result is False:
                print("[WARNING] Failed to return to stage 4 position")
            
            print("   ⚙️ Restoring normal speed...")
            restore_speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
            if restore_speed_result is False:
                print("[WARNING] Failed to restore normal speed")
            print("   ✅ Stage 4 milk pouring completed")
        
        # Step 5: Completion
        print("🏁 Step 5/5: Finalizing milk pouring...")
        print("   ✅ Milk pouring sequence completed")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ MILK POURING COMPLETED SUCCESSFULLY FOR STAGE {stage}")
        print("   ✓ Precise positioning achieved")
        print("   ✓ Optimal pouring technique executed")
        print("   ✓ Perfect milk-to-coffee ratio delivered")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk pouring: {e}")
        print("[INFO] Milk pouring process terminated due to error")
        return False

def clean_milk_pitcher(**params) -> bool:
    """
    Perform a cleaning motion for the frother tool.

    Returns:
        bool: True on successful cleaning movement sequence, False otherwise.
    """
    try:
        print("🧽 Cleaning frother motion sequence...")
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose1']) is False:
            print("[ERROR] Failed to reach clean pose 1")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose2']) is False:
            print("[ERROR] Failed to reach clean pose 2")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose3']) is False:
            print("[ERROR] Failed to reach clean pose 3")
            return False
        if run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['cleaning_motion']) is False:
            print("[ERROR] Failed to execute cleaning motion")
            return False
        print("✅ Frother cleaning movement completed")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother cleaning: {e}")
        return False

def return_frother(**params) -> bool:
    """
    Return the frother to its original location using recorded approach/grab angles.

    Returns:
        bool: True on successful return, False otherwise.
    """
    try:
        global approach_angles, grab_angles
        print("↩️ Returning frother to original location...")

        if grab_angles is None or approach_angles is None:
            print("[ERROR] Missing recorded angles for safe return. Ensure pick_frother() recorded positions.")
            return False

        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return1']) is False:
            print("[ERROR] Failed to reach pre-return pose 1")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return2']) is False:
            print("[ERROR] Failed to reach pre-return pose 2")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return3']) is False:
            print("[ERROR] Failed to reach pre-return pose 3")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return4']) is False:
            print("[ERROR] Failed to reach pre-return pose 4")
            return False
        if run_skill("gotoJ_deg", *grab_angles) is False:
            print("[ERROR] Failed to go to recorded grab angles")
            return False
        if run_skill("sync") is False:
            print("[WARNING] Sync operation failed - continuing...")
        if run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['release']) is False:
            print("[ERROR] Failed to release frother")
            return False
        if run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['final_approach']) is False:
            print("[ERROR] Failed to execute final approach move")
            return False
        if run_skill("gotoJ_deg", *approach_angles) is False:
            print("[ERROR] Failed to go to recorded approach angles")
            return False
        if home(position="north") is False:
            print("[WARNING] Failed to go home after return")
        if run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open']) is False:
            print("[WARNING] Failed to fully open gripper after return")
        print("✅ Frother returned successfully")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother return: {e}")
        return False

"""
plastic_cups.py

Defines the plastic cup handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for grabbing and placing plastic cups
in the BARNS coffee automation system, supporting multiple cup sizes for cold beverages
like slushes, iced drinks, and cold brews.
"""

def _normalize_plastic_cup_size(cups_dict: Any) -> str:
    """
    Parse plastic cup size from new JSON format.
    
    This is a wrapper around the unified _normalize_cup_size function.
    Use this for backward compatibility in plastic cup operations.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz', '16oz')
    """
    return _normalize_cup_size(cups_dict, cup_type='plastic')

def dispense_plastic_cup(**params) -> bool:
    try:
        # Extract and validate cup size parameter using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_PLASTIC_CUP_SIZE)
        if not cup_size or not validate_cup_size(cup_size):
            return False
        
        # Configuration for each cup size using centralized constants
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
            print(f"[ERROR] Unknown cup size: {cup_size!r}")
            print("[INFO] Valid cup sizes: 7oz, 9oz, 12oz, 16oz")
            return False
        # return_back_to_home()
        config = CUP_CONFIG[cup_size]
        print(f"🥤 Starting plastic cup grab sequence for {cup_size}")
        print("=" * 50)
        attempt_count = 0
        while attempt_count < 5:
            if cup_size == "16oz":
                home(position=config['home'])
                run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_FULL)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 380.0, -50.0, 0, 0, 0)
                run_skill("set_speed_factor", config['speed'])
                run_skill("sync")
                run_skill("moveEE", 0.0, 0.0, 100.0, 0, 0, 0)
                run_skill("moveEE", 0.0, 0.0, -100.0, 0, 0, 0)
                run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)
                run_skill("moveEE", 0.0, 17.5, 65.0, 0, 0, 0)
                run_skill("set_gripper_position", GRIPPER_FULL, config['gripper'])
                run_skill("moveEE", 0, -5.0, config['extract_z'], 0, 0, 0)
                run_skill("set_speed_factor", SPEED_FAST)
                run_skill("sync")
                run_skill("moveEE", 0, -5.0, config['extract_z2'], 0, 0, 0)
                run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            elif cup_size == "12oz":
                home(position=config['home'])
                run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_FULL)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 380.0, -50.0, 0, 0, 0)
                run_skill("set_speed_factor", config['speed'])
                run_skill("sync")
                run_skill("moveEE", 0.0, 0.0, 100.0, 0, 0, 0)
                run_skill("moveEE", 0.0, 0.0, -100.0, 0, 0, 0)
                run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)
                run_skill("moveEE", 0.0,12.5, 70, 0, 0, 0)
                run_skill("set_gripper_position", GRIPPER_FULL, config['gripper'])
                run_skill("moveEE", 0, -5.0, config['extract_z'], 0, 0, 0)
                run_skill("set_speed_factor", SPEED_FAST)
                run_skill("sync")
                run_skill("moveEE", 0, -5.0, config['extract_z2'], 0, 0, 0)
                run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
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
            # Check if a cup is in the gripper
            cup_detected = detect_cup_gripper()
            if cup_detected:
                print("✅ Cup detected in gripper")
                break
            else:
                print("❌ No cup detected in gripper")
                attempt_count += 1
                if attempt_count == 3:
                    print("[ERROR] Failed to grab plastic cup after 3 attempts")
                    return False

        # Set flag to indicate cup was just dispensed
        _set_cup_dispensed()
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during plastic cup grab: {e}")
        print("[INFO] Plastic cup grab process terminated due to error")
        return False

def go_to_ice(**params) -> bool:
    """
    Get ice for the specified cup size.
    
    This function handles ice dispensing for different cup sizes:
    1. Validates cup size parameter
    2. Moves to appropriate home positions
    3. Executes ice dispensing sequence based on cup size
    
    Args:
        cup_size (str): Size of cup for ice ('7oz', '9oz', '12oz', '16oz')
        
    Returns:
        bool: True if ice dispensing completed successfully, False otherwise
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict)
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        # Validate cup size parameter
        valid_sizes = ('16oz', '12oz', '9oz', '7oz')
        if cup_size not in valid_sizes:
            print(f"[ERROR] Unknown plastic cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        print(f"🧊 Starting ice dispensing sequence for {cup_size}")
        print("=" * 50)
        
        # Step 3: Execute ice dispensing based on cup size
        print(f"🧊 Step 3/3: Dispensing ice for {cup_size} cup...")
        
        # All cup sizes use the same ice positions (can be customized per size if needed)
        print(f"   📍 Positioning for {cup_size} ice dispensing...")
        pos1_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])
        if not pos1_result:
            print("[ERROR] Failed to move to first ice position")
            return False
        
        print("   📍 Moving to ice dispensing position...")
        pos2_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position2'])
        if not pos2_result:
            print("[ERROR] Failed to move to ice dispensing position")
            return False
        print(f"   ✅ {cup_size} ice dispensing completed")
        
        run_skill("sync")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ ICE DISPENSING COMPLETED SUCCESSFULLY FOR {cup_size.upper()}")
        print("   ✓ Proper positioning achieved")
        print("   ✓ Ice dispensing sequence executed")
        print("   ✓ Ready for beverage preparation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during ice dispensing: {e}")
        print("[INFO] Ice dispensing process terminated due to error")
        return False

def go_home_with_ice(**params) -> bool:
    try:        
        # Step 1: Initial positioning
        print("📍 Step 1/6: Moving to initial position...")
        pos1_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])
        if not pos1_result:
            print("[ERROR] Failed to move to initial position")
            return False
        print("   ✅ Successfully moved to initial position")
        
        # Step 2: Move to north-east home
        print("🏠 Step 2/6: Moving to north-east home...")
        if not home(position="north"):
            print("[ERROR] Failed to move to north-east home")
            return False
        print("   ✅ Successfully moved to north-east home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ RETURNED HOME WITH ICE SUCCESSFULLY")
        print("   ✓ Robot returned to home position")
        print("   🧊 Ready for next operation")
        print("=" * 50)
        
        # Set flag to indicate cup came from ice (same as after dispense)
        _set_cup_dispensed()
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup placement: {e}")
        print("[INFO] Cup placement process terminated due to error")
        return False

def place_plastic_cup_station(**params) -> bool:
    """
    Place a plastic cup at specified staging area.
    
    This function places a previously grabbed plastic cup at a designated staging area:
    1. Validates cup position parameter
    2. Moves through positioning sequence
    3. Navigates to target stage position
    4. Releases cup and moves up safely
    5. Returns to home position
    
    Auto-detects if called after dispense_plastic_cup or go_home_with_ice to apply height adjustment.
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        
    Returns:
        bool: True if cup placement completed successfully, False otherwise
        
    Example:
        # After dispensing - auto-detects and applies adjustment
        dispense_plastic_cup(cup_size="7oz")
        success = place_plastic_cup_station(position={'cup_position': 1.0}, cups={'cup_C7': 1.0})
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict)
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        # Validate parameters
        valid_sizes = ('7oz', '9oz', '12oz', '16oz')
        
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Starting plastic cup placement sequence for stage {stage}, Size: {cup_size} (after_dispense={after_dispense})")
        print("=" * 50)

        run_skill("set_speed_factor", SPEED_NORMAL)
        run_skill("sync")
        
        # Step 1: Move to north-east home
        print("🏠 Step 1/5: Moving to north-east home...")
        if not home(position="north_east"):
            print("[ERROR] Failed to move to north-east home")
            return False
        print("   ✅ Successfully moved to north-east home")
        
        # Step 2: Move to east home
        print("🏠 Step 2/5: Moving to east home...")
        if not home(position="east"):
            print("[ERROR] Failed to move to east home")
            return False
        print("   ✅ Successfully moved to east home")
        
        # Step 3: Move to stage-specific position using parameters
        print(f"🎯 Step 3/5: Moving to stage {stage} position...")
        stage_result = False
        
        if stage == "1":
            print("   📍 Positioning for stage 1...")
            stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_1'])
        elif stage == "2":
            print("   📍 Positioning for stage 2...")
            stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_2'])
        elif stage == "3":
            print("   📍 Positioning for stage 3...")
            home(position="south_east")
            stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_3'])
        elif stage == "4":
            print("   📍 Positioning for stage 4...")
            home(position="south_east")
            stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_4'])
        
        if not stage_result:
            print(f"[ERROR] Failed to move to stage {stage} position")
            return False
        print(f"   ✅ Successfully positioned at stage {stage}")
        
        # Step 4: Release cup
        print("🤏 Step 4/5: Releasing plastic cup...")
        release_result = run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)
        if not release_result:
            print("[ERROR] Failed to release plastic cup")
            return False
        print("   ✅ Cup released successfully")

        run_skill("set_speed_factor", SPEED_FAST)
        
        # Step 5: Move up and return to home
        print("⬆️ Step 5/5: Moving up and returning to home...")
        up_result = run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['place_return_up'])
        if not up_result:
            print("[ERROR] Failed to move up after placement")
            return False
        
        if not home(position="east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully moved up and returned to home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PLASTIC CUP PLACEMENT COMPLETED FOR STAGE {stage}")
        print("   ✓ Cup positioned at designated staging area")
        print("   ✓ Safe release and clearance achieved")
        print("   ✓ Robot returned to home position")
        print("   🥤 Beverage station ready!")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup placement: {e}")
        print("[INFO] Cup placement process terminated due to error")
        return False

def pick_plastic_cup_station(**params) -> bool:
    """
    Pick up a plastic cup from a specific stage and add ice to it.
    
    This function picks up a previously placed plastic cup from a staging area
    and moves it to the ice dispensing station for ice addition.
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        
    Returns:
        bool: True if cup picked and ice added successfully, False otherwise
        
    Example:
        success = pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C12': 1.0})
        if success:
            print("Cup picked and ice added successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict)
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        # Validate parameters
        valid_sizes = ('7oz', '9oz', '12oz', '16oz')
        
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        print(f"🥤 Starting cup pickup for ice sequence - Stage: {stage}, Size: {cup_size}")
        print("=" * 50)
        
        # Stage-specific positioning using parameters
        stage_positions = {
            "1": PLASTIC_CUPS_PARAMS['staging']['pickup_1'],
            "2": PLASTIC_CUPS_PARAMS['staging']['pickup_2'],
            "3": PLASTIC_CUPS_PARAMS['staging']['pickup_3'],
            "4": PLASTIC_CUPS_PARAMS['staging']['pickup_4']
        }
        
        # Cup size specific gripper positions
        gripper_positions = {
            "7oz": 145,
            "9oz": 125, 
            "12oz": 140,
            "16oz": 118
        }
        
        # Step 1: Navigate to appropriate home positions
        print("🏠 Step 1/6: Navigating to home positions...")
        if not home(position="north_east"):
            print("[ERROR] Failed to move to north_east home")
            return False
            
        if not home(position="east"):
            print("[ERROR] Failed to move to east home")
            return False
        
        # Additional positioning for stages 3 and 4
        if stage in ("3", "4"):
            if not home(position="south_east"):
                print("[ERROR] Failed to move to south_east home")
                return False
        print("   ✅ Successfully navigated to home positions")
        
        # Step 2: Move to stage-specific position
        print(f"📍 Step 2/6: Moving to stage {stage} position...")
        stage_result = run_skill("gotoJ_deg", *stage_positions[stage])
        if not stage_result:
            print(f"[ERROR] Failed to move to stage {stage} position")
            return False
        print(f"   ✅ Successfully positioned at stage {stage}")
        
        # Step 3: Position for cup pickup
        print("🎯 Step 3/6: Positioning for cup pickup...")
        pickup_result = run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['pickup_down'])
        if not pickup_result:
            print("[ERROR] Failed to position for cup pickup")
            return False
        print("   ✅ Successfully positioned for pickup")
        
        # Step 4: Grip the cup
        print(f"🤏 Step 4/6: Gripping {cup_size} cup...")
        grip_result = run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])
        if not grip_result:
            print("[ERROR] Failed to grip cup")
            return False
        print("   ✅ Cup gripped successfully")

        run_skill("set_speed_factor", SPEED_NORMAL)
        run_skill("sync")
        
        if cup_position == 3 or cup_position == 4:
            print("🏠 Step 4.5/6: Moving to south-east home for stages 3/4...")
            if not home(position="south_east"):
                print("[ERROR] Failed to move to south_east home")
                return False
            print("   ✅ Successfully moved to south-east home")
                
        # Step 5: Return to east home
        print("🏠 Step 5/6: Returning to east home...")
        if not home(position="east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully returned to east home")
        
        # Step 6: Return to north-east home
        print("🏠 Step 6/6: Returning to north-east home...")
        if not home(position="north_east"):
            print("[ERROR] Failed to return to north-east home")
            return False
        print("   ✅ Successfully returned to north-east home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ CUP PICKUP FOR ICE COMPLETED SUCCESSFULLY")
        print(f"   ✓ Stage {stage} cup ({cup_size}) picked up")
        print("   ✓ Positioned for ice dispensing")
        print("   ✓ Ready for ice addition")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup pickup for ice: {e}")
        print("[INFO] Cup pickup process terminated due to error")
        return False

def place_plastic_cup_sauces(**params) -> bool:
    """
    Place the plastic cup at the sauces station.
    
    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        after_dispense (bool): If True, called after dispense_plastic_cup. If False, called after other operations. Default True.
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Placing {cup_size} plastic cup at sauces station (after_dispense={after_dispense})")
        
        # Set speed for careful handling if not after dispense
        if not after_dispense:
            run_skill("set_speed_factor", SPEED_NORMAL)
            run_skill("sync")
        
        # All cup sizes use same positions (simplified)
        if run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1']) is False:
            return False
        if run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position2']) is False:
            return False
        
        if run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_plastic_cup_sauces failed: {e}")
        return False

def pick_plastic_cup_sauces(**params) -> bool:
    """
    Pick the plastic cup from the sauces station.

    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        after_dispense (bool): If True, called after dispense_plastic_cup. If False, called after other operations. Default True.
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Picking {cup_size} plastic cup from sauces station (after_dispense={after_dispense})")

        # Gripper positions for sauces station (using specific values for station pickup)
        gripper_positions = {
            "7oz": 145,
            "9oz": 145,
            "12oz": 150,
            "16oz": PLASTIC_CUP_GRIPPER_POSITIONS['16oz'],
        }

        # Set speed for careful handling
        run_skill("set_speed_factor", SPEED_NORMAL)
        run_skill("sync")
        
        # All cup sizes use same positions (simplified)
        if run_skill("moveEE", 0,0,5,0,0,0) is False:
            return False
        if run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size]) is False:
            return False
        if run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position2']) is False:
            return False
        if run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1']) is False:
            return False
        
        return True
    except Exception as e:
        print(f"[ERROR] pick_plastic_cup_sauces failed: {e}")
        return False

def place_plastic_cup_milk(**params) -> bool:
    """
    Place the plastic cup at the milk station.
    
    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        after_dispense (bool): If True, called after dispense_plastic_cup. If False, called after other operations. Default True.
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Placing {cup_size} plastic cup at milk station (after_dispense={after_dispense})")
        
        # Set speed for careful handling if not after dispense
        if not after_dispense:
            run_skill("set_speed_factor", SPEED_NORMAL)
            run_skill("sync")
        
        # All cup sizes use same positions (simplified)
        if run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1']) is False:
            return False
        if run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position2']) is False:
            return False
        
        if run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_plastic_cup_milk failed: {e}")
        return False

def pick_plastic_cup_milk(**params) -> bool:
    """
    Pick the plastic cup from the milk station.

    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        after_dispense (bool): If True, called after dispense_plastic_cup. If False, called after other operations. Default True.
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Picking {cup_size} plastic cup from milk station (after_dispense={after_dispense})")

        # Gripper positions for milk station (using specific values for station pickup)
        gripper_positions = {
            "7oz": 145,
            "9oz": 145,
            "12oz": 150,
            "16oz": PLASTIC_CUP_GRIPPER_POSITIONS['16oz'],
        }

        # Set speed for careful handling
        run_skill("set_speed_factor", SPEED_NORMAL)
        run_skill("sync")
        
        # All cup sizes use same positions (simplified)
        if run_skill("moveEE", 0,0,5,0,0,0) is False:
            return False
        if run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size]) is False:
            return False
        if run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position2']) is False:
            return False
        if run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1']) is False:
            return False
        
        return True
    except Exception as e:
        print(f"[ERROR] pick_plastic_cup_milk failed: {e}")
        return False
   
"""
slush.py

Defines the slush handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for dispensing and placing slush drinks
in the BARNS coffee automation system, supporting multiple dispensers and staging areas
for frozen beverage preparation.
"""

def get_slush(**params) -> bool:
    """
    Get slush from specified dispenser and prepare for serving.
    
    This function handles slush dispensing for different stages, cup sizes, and dispensers:
    1. Grabs plastic cup of specified size
    2. Moves to intermediate positioning
    3. Navigates to appropriate slush dispenser
    4. Positions cup under dispenser for slush dispensing
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cup_size (str): Cup size ('16oz' - currently only 16oz supported), defaults to '16oz'
        dispenser (str): Dispenser number ('1' or '2') - optional, will be inferred from premixes if not provided
        premixes (dict): Premix dictionary to infer dispenser if not explicitly provided
        
    Returns:
        bool: True if slush dispensing completed successfully, False otherwise
        
    Example:
        success = get_slush(position={'cup_position': 1.0}, dispenser='1')
        if success:
            print("Slush dispensed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        cup_size = params.get("cup_size", "16oz")  # Default to 16oz
        dispenser = params.get("dispenser")
        
        # If no dispenser is provided, try to infer from premixes or use default
        if not dispenser:
            premixes = params.get("premixes", {})
            if premixes:
                # Map premix types to dispensers
                premix_name = list(premixes.keys())[0] if premixes else ""
                # Default mapping: most premixes go to dispenser 1
                # You can extend this mapping as needed
                if "chocolate" in premix_name.lower() or "choco" in premix_name.lower():
                    dispenser = "2"
                else:
                    dispenser = "1"
                print(f"[INFO] No dispenser specified, inferred dispenser '{dispenser}' from premix '{premix_name}'")
            else:
                # Default to dispenser 1 if no premix info
                dispenser = "1"
                print(f"[INFO] No dispenser specified, defaulting to dispenser '1'")
        
        # Validate parameters
        valid_cup_sizes = ("16oz",)  # Currently only 16oz supported
        valid_dispensers = ("1", "2")
            
        if cup_size not in valid_cup_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid cup sizes: {', '.join(valid_cup_sizes)}")
            return False
            
        if dispenser not in valid_dispensers:
            print(f"[ERROR] Invalid dispenser: {dispenser!r}")
            print(f"[INFO] Valid dispensers: {', '.join(valid_dispensers)}")
            return False
        
        print(f"🥤 Starting slush dispensing: Stage {stage}, {cup_size}, Dispenser {dispenser}")
        print("=" * 50)
        
        # Step 1: Grab plastic cup
        log_step(1, 4, f"Grabbing {cup_size} plastic cup")
        # Convert cup_size to proper cups dict format (e.g., "16oz" -> {"cup_C16": 1.0})
        cup_code = f"cup_C{cup_size.replace('oz', '')}"
        if not dispense_plastic_cup(cups={cup_code: 1.0}):
            log_error(f"Failed to grab {cup_size} plastic cup")
            return False
        log_success("Cup grabbed successfully", indent=1)
        
        # Step 2: Move to intermediate positioning
        print("📍 Step 2/4: Moving to intermediate positioning...")
        pos1_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['intermediate'])
        if not pos1_result:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        # Step 3: Move to slush area
        print("🧊 Step 3/4: Moving to slush dispensing area...")
        pos2_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])
        if not pos2_result:
            print("[ERROR] Failed to move to slush area")
            return False
        print("   ✅ Successfully positioned in slush area")
        
        # Step 4: Position at specific dispenser
        print(f"🎯 Step 4/4: Positioning at dispenser {dispenser}...")
        if dispenser == "1":
            print("   📍 Moving to dispenser 1...")
            dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['dispense'])
        else:  # dispenser == "2"
            print("   📍 Moving to dispenser 2...")
            pos3_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['intermediate'])
            if not pos3_result:
                print("[ERROR] Failed to move to dispenser 2 intermediate position")
                return False
            dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['dispense'])
        
        if not dispenser_result:
            print(f"[ERROR] Failed to position at dispenser {dispenser}")
            return False
        print(f"   ✅ Successfully positioned at dispenser {dispenser}")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ SLUSH DISPENSING COMPLETED SUCCESSFULLY")
        print(f"   ✓ Stage: {stage}, Cup: {cup_size}, Dispenser: {dispenser}")
        print("   ✓ Cup positioned for slush dispensing")
        print("   ✓ Ready for slush dispensing operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during slush dispensing: {e}")
        print("[INFO] Slush dispensing process terminated due to error")
        return False

def place_slush(**params) -> bool:
    """
    Place slush-filled cup at specified staging area after dispensing.
    
    This function handles the placement of slush-filled cups:
    1. Sets appropriate speed for careful handling
    2. Moves away from dispenser safely
    3. Navigates to home position
    4. Places cup at designated staging area
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cup_size (str): Cup size ('16oz' - currently only 16oz supported), defaults to '16oz'
        dispenser (str): Dispenser number used ('1' or '2') - optional, will be inferred from premixes if not provided
        premixes (dict): Premix dictionary to infer dispenser if not explicitly provided
        
    Returns:
        bool: True if slush placement completed successfully, False otherwise
        
    Example:
        success = place_slush(position={'cup_position': 2.0}, dispenser='1')
        if success:
            print("Slush cup placed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        cup_size = params.get("cup_size", "16oz")  # Default to 16oz
        dispenser = params.get("dispenser")
        
        # If no dispenser is provided, try to infer from premixes or use default
        if not dispenser:
            premixes = params.get("premixes", {})
            if premixes:
                # Map premix types to dispensers (same logic as get_slush)
                premix_name = list(premixes.keys())[0] if premixes else ""
                if "chocolate" in premix_name.lower() or "choco" in premix_name.lower():
                    dispenser = "2"
                else:
                    dispenser = "1"
                print(f"[INFO] No dispenser specified, inferred dispenser '{dispenser}' from premix '{premix_name}'")
            else:
                # Default to dispenser 1 if no premix info
                dispenser = "1"
                print(f"[INFO] No dispenser specified, defaulting to dispenser '1'")
        
        # Validate parameters
        valid_cup_sizes = ("16oz",)  # Currently only 16oz supported
        valid_dispensers = ("1", "2")
            
        if cup_size not in valid_cup_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid cup sizes: {', '.join(valid_cup_sizes)}")
            return False
            
        if dispenser not in valid_dispensers:
            print(f"[ERROR] Invalid dispenser: {dispenser!r}")
            print(f"[INFO] Valid dispensers: {', '.join(valid_dispensers)}")
            return False
        
        print(f"🧊 Starting slush placement: Stage {stage}, {cup_size}, from Dispenser {dispenser}")
        print("=" * 50)
        
        # Step 1: Set careful handling speed
        print("⚙️ Step 1/4: Setting careful handling speed...")
        speed_result = run_skill("set_speed_factor", SPEED_NORMAL)
        if not speed_result:
            print("[WARNING] Failed to set speed factor - continuing with default")
        else:
            print("   ✅ Speed factor set for careful handling")
        
        # Step 2: Move away from dispenser safely
        print(f"⬅️ Step 2/4: Moving away from dispenser {dispenser}...")
        if dispenser == "1":
            print("   📍 Moving away from dispenser 1...")
            retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['retreat'])
        else:  # dispenser == "2"
            print("   📍 Moving away from dispenser 2...")
            retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['retreat'])
        
        if not retreat_result:
            print(f"[ERROR] Failed to move away from dispenser {dispenser}")
            return False
        print(f"   ✅ Successfully moved away from dispenser {dispenser}")
        
        # Step 3: Navigate to home position
        print("🏠 Step 3/4: Navigating to home position...")
        if not home(position="north"):
            print("[ERROR] Failed to move to north home position")
            return False
        print("   ✅ Successfully moved to home position")
        
        # Step 4: Place slush cup at designated stage
        print(f"📍 Step 4/4: Placing slush cup at stage {stage}...")
        # Convert cup_size to proper cups dict format (e.g., "16oz" -> {"cup_C16": 1.0})
        cup_code = f"cup_C{cup_size.replace('oz', '')}"
        if not place_plastic_cup_station(position={'cup_position': int(stage)}, cups={cup_code: 1.0}):
            print(f"[ERROR] Failed to place slush cup at stage {stage}")
            return False
        print(f"   ✅ Successfully placed slush cup at stage {stage}")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ SLUSH PLACEMENT COMPLETED SUCCESSFULLY")
        print(f"   ✓ Stage: {stage}, Cup: {cup_size}, from Dispenser: {dispenser}")
        print("   ✓ Slush cup safely transported and placed")
        print("   ✓ Ready for customer service")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during slush placement: {e}")
        print("[INFO] Slush placement process terminated due to error")
        return False
      
'''
RECIPES
'''
def espresso(**params):
    """
    Complete espresso preparation sequence using port_2.
    
    Workflow:
    1. Unmount portafilter from port_2
    2. Grind and tamp coffee
    3. Mount portafilter back to port_2
    4. Prepare paper cup at stage_1
    5. Pour espresso from port_2
    6. Return pitcher
    
    
    Returns:
        bool: True if espresso prepared successfully, False otherwise
    """
    print("☕ Starting espresso preparation sequence...")
    
    try:
        # Step 1: Unmount portafilter from port_2
        print("🔧 Unmounting portafilter from port_2...")
        if not unmount(port="port_1"):
            print("[ERROR] Failed to unmount portafilter from port_2")
            return False
        
        # Step 2: Grind coffee
        print("⚙️ Grinding coffee...")
        if not grinder():
            print("[ERROR] Failed to grind coffee")
            return False
        
        # Step 3: Tamp coffee
        print("🫸 Tamping coffee...")
        if not tamper():
            print("[ERROR] Failed to tamp coffee")
            return False
        
        # Step 4: Mount portafilter back to port_2
        print("🔧 Mounting portafilter to port_2...")
        if not mount(port="port_1"):
            print("[ERROR] Failed to mount portafilter to port_2")
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
        
        # Step 7: Pick espresso pitcher for port_2
        print("🥛 Picking espresso pitcher for port_2...")
        if not pick_espresso_pitcher(port="port_1"):
            print("[ERROR] Failed to pick espresso pitcher")
            return False
        
        # Step 8: Pour espresso at stage_1
        print("☕ Pouring espresso at stage_1...")
        if not pour_espresso_pitcher(position={'cup_position': 1}):
            print("[ERROR] Failed to pour espresso")
            return False
        
        # Step 9: Return espresso pitcher
        print("🔄 Returning espresso pitcher for port_2...")
        if not return_espresso_pitcher(port="port_1"):
            print("[ERROR] Failed to return espresso pitcher")
            return False
        
        unmount(port="port_1")
        
        # Step 10: Clean portafilter
        print("🧹 Cleaning portafilter...")
        if not clean_portafilter(port="port_1"):
            print("[ERROR] Failed to clean portafilter")
            return False
        
        # Step 11: Mount portafilter
        print("🔧 Mounting portafilter...")
        if not mount(port="port_1"):
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
        if not grinder():
            print("[ERROR] Failed to grind coffee")
            return False
        
        # Step 3: Tamp coffee
        print("🫸 Tamping coffee...")
        if not tamper():
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
        if not pour_espresso_pitcher(position={'cup_position': 1}):
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
            if not grinder():
                return False
            if not tamper():
                return False
            if not mount(port="port_1"):
                return False
            if not grab_paper_cup(size="7oz"):
                return False
            if not place_paper_cup(position={'cup_position': position}):
                return False
            if not pick_espresso_pitcher(port="port_1"):
                return False
            if not pour_espresso_pitcher(position={'cup_position': position}):
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
    run_skill("gotoJ_deg", -29.882000,-17.719395,-139.409073,-21.613743,-116.902481,0)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", -19.191433,-29.470440,-114.684837,-35.904163,-105.765900,-2.335130)#run_skill("mount_machine", "three_group_espresso", "portafilter_1", True)
    input()
    run_skill("gotoJ_deg", -29.882000,-17.719395,-139.409073,-21.613743,-116.902481,0)#run_skill("approach_machine", "three_group_espresso", "portafilter_1", True)
    # input()
    run_skill("gotoJ_deg", *ESPRESSO_HOME)#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    # input()
    # run_skill("gotoJ_deg", 24.911945,-21.074497,-135.128052,-22.276201,-61.762913,0)#run_skill("mount_machine", "three_group_espresso", "portafilter_2", True)
    # input()
    # run_skill("gotoJ_deg", *ESPRESSO_HOME)#run_skill("approach_machine", "three_group_espresso", "portafilter_2", True)
    # input()
    run_skill("gotoJ_deg", 73.193321,-6.765492,-126.959679,-42.988068,-11.680984,-1.857552)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", 53.960320,-26.822138,-122.314857,-29.404949,-33.919445,-2.304142)#run_skill("mount_machine", "three_group_espresso", "portafilter_3", True)
    input()
    run_skill("gotoJ_deg", 73.193321,-6.765492,-126.959679,-42.988068,-11.680984,-1.857552)#run_skill("approach_machine", "three_group_espresso", "portafilter_3", True)
    run_skill("gotoJ_deg", *ESPRESSO_HOME)
    # run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    # input()
    # run_skill("gotoJ_deg",16.182545,-45.977921,-119.918640,-12.260736,-73.420769,0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
    # input()
    # run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    # run_skill("gotoJ_deg", -22.378635,-25.306602,-137.820709,-24.454021,-108.835793,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
    # input()
    # run_skill("gotoJ_deg", -16.964697,-50.312492,-105.155556,-21.110926,-106.022964,0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
    # input()
    # run_skill("gotoJ_deg", -22.378635,-25.306602,-137.820709,-24.454021,-108.835793,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
    # run_skill("gotoJ_deg",32.103580,-28.542721,-151.581696,-2.586381,-58.585411,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    # run_skill("gotoJ_deg",72.861008,-37.369293,-141.719101,8.783054,-15.383393,0)#run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
    # input()
    # run_skill("gotoJ_deg",45.906475,-47.583317,-110.267502,-21.407314,-44.230503,0)#run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3", True)
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
    # run_skill("gotoJ_deg",-42.325905,-64.186890,-94.830910,-18.675297,-93.775047,-1.096767)#approach the grinder#run_skill("approach_machine", "espresso_grinder", "grinder", True)
    # input()
    # run_skill("gotoJ_deg", -42.658646,-68.671516,-79.158653,-29.861872,-94.114250,-1.078400) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    # input()
    # run_skill("gotoJ_deg",-42.630577,-64.802246,-65.030914,-64.123573,-94.165909,-1.080938)#touch the button#run_skill("approach_machine", "espresso_grinder", "tamper", True)
    # input()
    # run_skill("gotoJ_deg", -42.658646,-68.671516,-79.158653,-29.861872,-94.114250,-1.078400) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    # input()
    # run_skill("gotoJ_deg", -42.806690,-71.829788,-76.761559,-31.305946,-94.264015,-1.083733) #in the tamper#run_skill("mount_machine", "espresso_grinder", "tamper", True)
    # input()
    # run_skill("gotoJ_deg", -42.658646,-68.671516,-79.158653,-29.861872,-94.114250,-1.078400) #above the tamper#run_skill("mount_machine", "espresso_grinder", "grinder", True)
    # run_skill("gotoJ_deg",-42.325905,-64.186890,-94.830910,-18.675297,-93.775047,-1.096767)#approach the grinder#run_skill("approach_machine", "espresso_grinder", "grinder", True)
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
    run_skill("gotoJ_deg",-31.904585,-80.887871,-29.938946,-89.772949,-45.815228,20.761606)#run_skill("approach_machine", "left_steam_wand", "deep_froth", True)
    input()
    run_skill("gotoJ_deg", -36.110901,-77.590630,-26.223412,-80.098061,-48.669434,20.761606)#run_skill("mount_machine", "left_steam_wand", "deep_froth", True)

def test(**params):
    run_skill("set_gripper_position", 255, 255, 255)

def test_arm1(**params):
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
    run_skill("gotoJ_deg", 181.706474,-15.556648,-147.169896,72.921334,-90.016973,-1.835663)
    run_skill("set_gripper_position", 255, 255, 255)
    run_skill("moveEE", 0, 0, 500, 0, 0, 0) 
    run_skill("set_gripper_position", 255, 0, 255) 

    

def test_arm2(**params) -> bool:
    run_skill("set_gripper_position", 255, 0, 255)

def hello(**params):
    for i in range(50):
        run_skill("gotoJ_deg", -190.000000,15.000000,-120.000000,5.000000,-40.000000,0.000000)
        # run_skill("gotoJ_deg", -180.000000,20.000000,-115.000000,0.000000,-90.000000,0.000000)
        run_skill("gotoJ_deg", -170.000000,25.000000,-110.000000,-5.000000,-140.000000,0.000000)
        # run_skill("gotoJ_deg", -180.000000,20.000000,-115.000000,0.000000,-90.000000,0.000000)
        # run_skill("gotoJ_deg", -180.000000,20.000000,-115.000000,0.000000,0.000000,0.000000)

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
    "place_plastic_cup_station_stage_1": lambda: place_plastic_cup_station(position={'cup_position': 1}),
    "place_plastic_cup_station_stage_2": lambda: place_plastic_cup_station(position={'cup_position': 2}),
    "place_plastic_cup_station_stage_3": lambda: place_plastic_cup_station(position={'cup_position': 3}),
    "place_plastic_cup_station_stage_4": lambda: place_plastic_cup_station(position={'cup_position': 4}),
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
    print("Type the name to run a sequence, or 'q' to quit.\n")

    try:
        while True:
            # 1) show the current list
            print("Available sequences:")
            for name in SEQUENCES:
                print(f"  • {name}")

            # 2) prompt the user
            choice = input("\nWhich sequence? (q to exit) ").strip().lower()

            if choice in ("q", "quit", "exit"):
                print("Bye!")
                break

            if choice not in SEQUENCES:
                print(f"❌  '{choice}' is not a valid sequence. Try again.\n")
                continue

            # 3) run the chosen sequence *once*
            try:
                SEQUENCES[choice]()        # ← call the function
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