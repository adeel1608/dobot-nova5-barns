"""
espresso.py

Defines the espresso-making sequence for different ports and cups.
This module provides comprehensive functions for managing the complete espresso
workflow including portafilter handling, grinding, tamping, mounting, and milk operations.
"""

import time
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill
from oms_v1.params import (
    PULL_ESPRESSO_PARAMS, 
    ESPRESSO_HOME, 
    ESPRESSO_GRINDER_HOME,
    ESPRESSO_GRINDER_PARAMS,
    ESPRESSO_PITCHER_PARAMS,
    ESPRESSO_HOT_WATER_PARAMS,
    _extract_cup_position
)
from oms_v1.sequences.cleaning import clean_portafilter

# Global variables to store captured positions during unmount sequence
below_espresso_port: Optional[Tuple[float, ...]] = None
mount_espresso_port: Optional[Tuple[float, ...]] = None
mount_espresso_pose: Optional[Tuple[float, ...]] = None  # Cartesian pose at mount position
approach_pitcher: Optional[Tuple[float, ...]] = None
pick_pitcher: Optional[Tuple[float, ...]] = None

# Portafilter validation threshold (in millimeters)
PORTAFILTER_Z_THRESHOLD_MM = 10.0  # If Z difference > this, portafilter was filled twice


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

        # Step 4: Close gripper to secure portafilter
        print("🤏 Step 4/13: Securing portafilter with gripper...")
        grip_result = run_skill("set_gripper_position", 255, 255)
        if grip_result is False:
            print("[ERROR] Failed to close gripper")
            return False
        print("   ✅ Gripper closed successfully")

        run_skill("set_speed_factor", 100)
        
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
        rotate_result = run_skill("move_portafilter_arc_movJ", -45.0)
        
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
            
        time.sleep(0.2)  # Allow settling time

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
        print(f"   📍 Executing: moveEE_movJ(0, 0, -35, 0, 0, 0)")
        clear_result = run_skill("moveEE_movJ", 0, 0, -35, 0, 0, 0)
        
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
        open_gripper = run_skill("set_gripper_position", 255, 0)
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
        close_gripper = run_skill("set_gripper_position", 255, 255)
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
                clear_down_result = run_skill("moveEE_movJ", 0, 0, -35, 0, 0, 0)
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
                print(f"   📍 Executing: moveEE_movJ(0, 0, 5, 0, 0, 0)")
                clear_result = run_skill("moveEE_movJ", 0, 0, 5, 0, 0, 0)
                
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
        rotate_result = run_skill("move_portafilter_arc_movJ", 47.0)
        
        if rotate_result is False:
            print("[ERROR] Failed to rotate portafilter")
            return False

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 8: Open gripper to release portafilter
        print("🤏 Step 8/10: Opening gripper to release portafilter...")
        release_result = run_skill("set_gripper_position", 255, 0)
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
        
        if port == 'port_3':
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            run_skill("moveEE_movJ", -20, 0, 0, 0, 0, 0)

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
            
            grip_result = run_skill("set_gripper_position", 255, 110)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 1")
                return False
            
            speed_result = run_skill("set_speed_factor", 25)
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
            
            grip_result = run_skill("set_gripper_position", 255, 105)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 2")
                return False
            
            speed_result = run_skill("set_speed_factor", 25)
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
            
            grip_result = run_skill("set_gripper_position", 255, 105)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 3")
                return False
            
            speed_result = run_skill("set_speed_factor", 25)
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
            run_skill("set_speed_factor", 15)
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
            run_skill("set_speed_factor", 15)
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
            run_skill("set_speed_factor", 15)
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
            run_skill("set_speed_factor", 5)
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

        run_skill("moveEE_movJ", -35, 0, 0, 0, 0, 0)        
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

        run_skill("set_speed_factor", 15)
        
        # Step 2: Return to holding position
        print("🏠 Step 2/2: Returning to holding position...")
        final_result = run_skill("moveEE_movJ", -150, 0, 0, 0, 0, 0)
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
            release_result = run_skill("set_gripper_position", 75, 0)
            
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
            release_result = run_skill("set_gripper_position", 75, 0)
            
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
            release_result = run_skill("set_gripper_position", 75, 0)
            
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

# Register functions for CLI discovery and external access
SEQUENCES = {
    'unmount': unmount,
    'grinder': grinder,
    'mount': mount,
    'pick_espresso_pitcher': pick_espresso_pitcher,
    'pour_espresso_pitcher_cup_station': pour_espresso_pitcher_cup_station,
    'get_hot_water': get_hot_water,
    'with_hot_water': with_hot_water,
    'return_espresso_pitcher': return_espresso_pitcher,
    'tamper': tamper,
}