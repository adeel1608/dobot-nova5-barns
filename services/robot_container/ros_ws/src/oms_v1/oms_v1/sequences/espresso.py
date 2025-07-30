"""
espresso.py

Defines the espresso-making sequence for different ports and cups.
This module provides comprehensive functions for managing the complete espresso
workflow including portafilter handling, grinding, tamping, mounting, and milk operations.
"""

import time
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill
from oms_v1.params import PULL_ESPRESSO_PARAMS

# Predefined home positions for espresso operations
Espresso_home = (42.159162,16.269149,-135.156441,-81.822150,-49.784457,13.771214)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)

# Global variables to store captured positions during unmount sequence
below_espresso_port: Optional[Tuple[float, ...]] = None
mount_espresso_port: Optional[Tuple[float, ...]] = None


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
    global below_espresso_port, mount_espresso_port
    try:
        # Extract and validate port parameter
        port = params.get("port", "port_2")  # Default to port_2
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
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'])
            if approach_result is False:
                print("[ERROR] Failed to approach portafilter")
                return False
            print("   ✅ Successfully approached portafilter")
        else:
            print("   ⏭️ Skipping approach step for port_2")
        
        # Step 3: Mount to the portafilter for secure grip
        print("🔧 Step 3/13: Mounting to portafilter...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        mount_result = run_skill("mount_machine", "three_group_espresso", port_params['portafilter_number'])
        
        if mount_result is False:
            print("[ERROR] Failed to mount to portafilter")
            return False
        print("   ✅ Successfully mounted to portafilter")
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 4: Close gripper to secure portafilter
        print("🤏 Step 4/13: Securing portafilter with gripper...")
        grip_result = run_skill("set_gripper_position", 255, 255)
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
        
        if port == 'port_1':
            # Step 7: Rotate portafilter to unlock (-45 degrees)
            print("🔄 Step 7/13: Rotating portafilter to unlock...")
            rotate_result = run_skill("move_portafilter_arc_movL", -45.0)
            
            if rotate_result is False:
                print("[ERROR] Failed to rotate portafilter")
                return False
            print("   ✅ Portafilter rotated to unlock position")
        
        elif port == 'port_2':
            # Step 7: Rotate portafilter to unlock (-35 degrees then -10 degrees)
            print("🔄 Step 7/13: Rotating portafilter to unlock (first rotation)...")
            rotate_result = run_skill("move_portafilter_arc_tool", -35.0)
            
            if rotate_result is False:
                print("[ERROR] Failed to rotate portafilter")
                return False

            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")

            # Step 7 continued: Second rotation
            print("🔄 Step 7/13: Rotating portafilter to unlock (second rotation)...")
            rotate_result = run_skill("move_portafilter_arc_movJ", -10.0)
            
            if rotate_result is False:
                print("[ERROR] Failed to rotate portafilter")
                return False
        
        elif port == 'port_3':
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
        
        # Step 10: Move end effector down to clear portafilter
        print("⬇️ Step 10/13: Moving down to clear portafilter...")
        print(f"   📍 Executing: moveEE(0, 0, -35, 0, 0, 0)")
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
        
        # Step 12: Move to position below port
        print("📍 Step 12/13: Moving to position below port...")
        below_result = run_skill("gotoJ_deg", *port_params['below_port'])
        
        if below_result is False:
            print("[ERROR] Failed to move to position below port")
            return False
        print("   ✅ Successfully moved to below port position")
        
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
            nav1_result = run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
            
            if nav1_result is False:
                print("[ERROR] Failed special navigation step 1")
                return False
            print("   ✅ Special navigation step 1 completed")
            
            nav2_result = run_skill("gotoJ_deg", -32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
            
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
        
    Returns:
        bool: True if grinding and tamping completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during grinding process
        
    Example:
        success = grinder(port='port_1')
        if success:
            print("Coffee grinding and tamping completed")
    """
    try:
        # Extract and validate port parameter
        port = params.get("port", "port_2")  # Default to port_2
        if not port:
            print("[ERROR] No port parameter provided")
            return False
            
        print(f"☕ Starting grinding and tamping sequence for {port}")
        print("=" * 50)
        
        # Step 1: Conditional move to grinder home for port_1
        if port == 'port_1':
            print("🏠 Step 1/7: Moving to grinder home position...")
            home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
            if home_result is False:
                print("[ERROR] Failed to move to grinder home")
                return False
            print("   ✅ Successfully moved to grinder home")
        else:
            print("   ⏭️ Skipping grinder home movement for this port")
        
        # Step 2: Approach the grinder
        print("🎯 Step 2/7: Approaching grinder...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        approach_result = run_skill("approach_machine", "espresso_grinder", "grinder")
        if approach_result is False:
            print("[ERROR] Failed to approach grinder")
            return False
        print("   ✅ Successfully approached grinder")
        
        # Step 3: Mount to grinder to activate grinding
        print("⚙️ Step 3/7: Mounting to grinder for grinding...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        mount_result = run_skill("mount_machine", "espresso_grinder", "grinder")
        if mount_result is False:
            print("[ERROR] Failed to mount to grinder")
            return False
        print("   ✅ Successfully mounted to grinder")
        
        # Step 4: Approach tamper station
        print("🎯 Step 4/7: Approaching tamper...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        tamper_approach_result = run_skill("approach_machine", "espresso_grinder", "tamper")
        if tamper_approach_result is False:
            print("[ERROR] Failed to approach tamper")
            return False
        print("   ✅ Successfully approached tamper")
        
        # Allow positioning time
        print("   ⏰ Allowing positioning time...")
        time.sleep(2.5)

        # Step 5: Mount to grinder again for consistency
        print("⚙️ Step 5/7: Re-mounting to grinder...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        mount_result2 = run_skill("mount_machine", "espresso_grinder", "grinder")
        if mount_result2 is False:
            print("[ERROR] Failed to re-mount to grinder")
            return False
        print("   ✅ Successfully re-mounted to grinder")
        
        # Step 6: Mount to tamper for positioning
        print("📍 Step 6/7: Positioning at tamper...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        tamper_mount_result = run_skill("mount_machine", "espresso_grinder", "tamper")
        if tamper_mount_result is False:
            print("[ERROR] Failed to mount to tamper")
            return False
        print("   ✅ Successfully positioned at tamper")

        # Step 7: Open gripper to complete process
        print("🤏 Step 7/7: Opening gripper...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        open_gripper = run_skill("set_gripper_position", 255, 0)
        if open_gripper is False:
            print("[ERROR] Failed to open gripper")
            return False
        print("   ✅ Gripper opened successfully")

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 8: Approach double portafilter tool
        print("🎯 Step 8/8: Approaching double portafilter tool...")
        approach_tool_result = run_skill("approach_tool", "double_portafilter")
        if approach_tool_result is False:
            print("[WARNING] Failed to approach double portafilter tool")
        print("   ✅ Successfully approached tool")
        
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
    Tamp coffee at the tamper station using double portafilter tool.
    
    This function performs the complete tamping sequence:
    1. Approaches and grabs the double portafilter tool
    2. Closes gripper to secure tool
    3. Lifts tool slightly for positioning
    4. Mounts to grinder for proper alignment
    5. Approaches grinder for final positioning
    6. Returns to grinder home position
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
        
    Returns:
        bool: True if tamping completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during tamping process
        
    Example:
        success = tamper()
        if success:
            print("Coffee tamping completed successfully")
    """
    try:
        print("🔨 Starting coffee tamping sequence")
        print("=" * 50)
        
        # Step 1: Approach and grab double portafilter tool
        print("🎯 Step 1/6: Approaching double portafilter tool...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        approach_tool_result = run_skill("approach_tool", "double_portafilter")
        if approach_tool_result is False:
            print("[ERROR] Failed to approach double portafilter tool")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        grab_tool_result = run_skill("grab_tool", "double_portafilter")
        if grab_tool_result is False:
            print("[ERROR] Failed to grab double portafilter tool")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        print("   ✅ Successfully approached and grabbed tool")
        
        # Step 2: Close gripper to secure tool
        print("🤏 Step 2/6: Securing tool with gripper...")
        close_gripper = run_skill("set_gripper_position", 255, 255)
        if close_gripper is False:
            print("[ERROR] Failed to close gripper")
            return False
        print("   ✅ Tool secured with gripper")
        
        # Step 3: Lift tool slightly for positioning
        print("⬆️ Step 3/6: Lifting tool for positioning...")
        lift_result = run_skill("moveEE", 0, 0, 10, 0, 0, 0)
        if lift_result is False:
            print("[ERROR] Failed to lift tool")
            return False
        print("   ✅ Tool lifted successfully")
        
        # Step 4: Mount to grinder for alignment
        print("⚙️ Step 4/6: Mounting to grinder for alignment...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        mount_result = run_skill("mount_machine", "espresso_grinder", "grinder")
        if mount_result is False:
            print("[ERROR] Failed to mount to grinder")
            return False
        print("   ✅ Successfully mounted to grinder")
        
        # Step 5: Approach grinder for final positioning
        print("🎯 Step 5/6: Approaching grinder for final positioning...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        approach_result = run_skill("approach_machine", "espresso_grinder", "grinder")
        if approach_result is False:
            print("[ERROR] Failed to approach grinder")
            return False
        print("   ✅ Successfully approached grinder")

        # Step 6: Return to grinder home
        print("🏠 Step 6/6: Returning to grinder home...")
        final_home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        if final_home_result is False:
            print("[ERROR] Failed to return to grinder home")
            return False
        print("   ✅ Successfully returned to grinder home")
        
        # Final success summary
        print("=" * 50)
        print("✅ COFFEE TAMPING COMPLETED SUCCESSFULLY")
        print("   ✓ Double portafilter tool used")
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
        
    Returns:
        bool: True if portafilter mounted successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during mount process
        
    Example:
        success = mount(port='port_1')
        if success:
            print("Portafilter mounted successfully")
    """
    try:
        # Extract and validate port parameter
        port = params.get("port", "port_2")  # Default to port_2
        if not port:
            print("[ERROR] No port parameter provided")
            return False
            
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] Unknown port number: {port!r}")
            print(f"[INFO] Available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"📥 Starting portafilter mount sequence for {port}")
        print("=" * 50)
        
        # Step 1: Special handling for ports 2 and 3 (reverse navigation)
        if port in ('port_2', 'port_3'):
            print("🔄 Step 1/10: Executing special navigation for port 2/3...")
            nav1_result = run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
            
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
        
        # Step 3: Move to position below port
        print("📍 Step 3/10: Moving to position below port...")
        below_result = run_skill("gotoJ_deg", *port_params['below_port'])
        
        if below_result is False:
            print("[ERROR] Failed to move to position below port")
            return False
        print("   ✅ Successfully moved to below port position")
        
        # Step 4: Approach the espresso group using captured position
        print(f"🎯 Step 4/10: Approaching espresso group {port_params['group_number']}...")
        if below_espresso_port is None:
            print("[ERROR] below_espresso_port not captured")
            print("[INFO] Run unmount first to capture required positions")
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
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 5.1: Move end effector up to fix portafilter
        print("⬇️ Step 5.1/10: Moving up to fix portafilter...")
        print(f"   📍 Executing: moveEE(0, 0, 10, 0, 0, 0)")
        clear_result = run_skill("moveEE_movJ", 0, 0, 10, 0, 0, 0)
        
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

        if port == 'port_1':
            # Step 7: Rotate portafilter to unlock (45 degrees)
            print("🔄 Step 7/10: Rotating portafilter to unlock...")
            rotate_result = run_skill("move_portafilter_arc_movL", 47.0)
            
            if rotate_result is False:
                print("[ERROR] Failed to rotate portafilter")
                return False
            print("   ✅ Portafilter rotated to unlock position")
        
        elif port == 'port_2':
            # Step 7: Rotate portafilter to unlock (10 degrees then 37 degrees)
            print("🔄 Step 7/10: Rotating portafilter to unlock (first rotation)...")
            rotate_result = run_skill("move_portafilter_arc_movJ", 10.0)
            
            if rotate_result is False:
                print("[ERROR] Failed to rotate portafilter")
                return False

            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")

            # Step 7 continued: Second rotation
            print("🔄 Step 7/10: Rotating portafilter to unlock (second rotation)...")
            rotate_result = run_skill("move_portafilter_arc_tool", 37.0)
            
            if rotate_result is False:
                print("[ERROR] Failed to rotate portafilter")
                return False
        
        elif port == 'port_3':
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
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
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
        
    Returns:
        bool: True if espresso pitcher picked successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pitcher pickup
        
    Example:
        success = pick_espresso_pitcher(port='port_1')
        if success:
            print("Espresso pitcher picked successfully")
    """
    try:
        # Extract and validate port parameter
        port = params.get("port", "port_2")  # Default to port_2
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
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Approach espresso pitcher area
        print("🎯 Step 2/5: Approaching espresso pitcher area...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")
        if approach_result is False:
            print("[ERROR] Failed to approach espresso pitcher area")
            return False
        print("   ✅ Successfully approached pitcher area")
        
        # Step 3: Pick espresso pitcher based on port
        print(f"🤏 Step 3/5: Picking espresso pitcher for {port}...")
        if port == 'port_1':
            # Port 1 pitcher sequence
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")
            if approach_result is False:
                print("[ERROR] Failed to approach espresso pitcher 1")
                return False
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")
            if mount_result is False:
                print("[ERROR] Failed to mount espresso pitcher 1")
                return False
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 1")
                return False
            
            speed_result = run_skill("set_speed_factor", 50)
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
            # Port 2 pitcher sequence
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")
            if mount_result is False:
                print("[ERROR] Failed to mount espresso pitcher 2")
                return False
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 2")
                return False
            
            speed_result = run_skill("set_speed_factor", 50)
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
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            move1_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")
            if move1_result is False:
                print("[ERROR] Failed to move to espresso pitcher 3 position 1")
                return False
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            move2_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")
            if move2_result is False:
                print("[ERROR] Failed to move to espresso pitcher 3 position 2")
                return False
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso pitcher 3")
                return False
            
            speed_result = run_skill("set_speed_factor", 50)
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
        
        # Step 4: Move to final position
        print("📍 Step 4/5: Moving to final holding position...")
        final_result = run_skill("gotoJ_deg", 31.076585, -40.253154, -136.313679, -3.210446, -58.931112, -0.206957)
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


def pour_espresso_pitcher(**params) -> bool:
    """
    Pour milk from espresso pitcher into cup at specified stage.
    
    This function performs the milk pouring sequence:
    - Moves to pouring position based on stage
    - Tilts espresso pitcher to pour milk
    - Returns to neutral position
    - Moves back to holding position
    
    Args:
        stage (str): Target stage ('stage_1' or 'stage_2'), defaults to 'stage_1'
        
    Returns:
        bool: True if pouring completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pouring process
        
    Example:
        success = pour_espresso_pitcher(stage='stage_1')
        if success:
            print("Milk poured successfully")
    """
    try:
        # Extract and validate stage parameter
        stage = params.get("stage", "stage_1")  # Default to stage_1
        if not stage:
            print("[ERROR] No stage parameter provided")
            return False
        
        if stage not in ('stage_1', 'stage_2'):
            print(f"[ERROR] Unknown stage: {stage!r}")
            print("[INFO] Available stages: stage_1, stage_2")
            return False
        
        print(f"🥛 Starting milk pouring sequence for {stage}")
        print("=" * 50)
        
        # Step 1: Initial positioning
        print("📍 Step 1/7: Moving to initial pouring position...")
        init_result = run_skill("moveJ_deg", 90.160210, 10.716150, 0.203157, -10.883145, -0.001922, 0.060433)
        
        if init_result is False:
            print("[ERROR] Failed to move to initial pouring position")
            return False
        print("   ✅ Successfully moved to initial position")
        
        if stage == 'stage_1':
            print("🎯 Stage 1 pouring sequence...")
            
            # Step 2: Approach stage 1 position
            print("📍 Step 2/7: Positioning for stage 1 pouring...")
            pos1_result = run_skill("gotoJ_deg", 138.578024, -22.115324, -126.765855, -38.694157, -57.964712, 3.173887)
            
            if pos1_result is False:
                print("[ERROR] Failed to approach stage 1 position")
                return False
            print("   ✅ Successfully positioned for stage 1")
            
            # Step 3: Tilt espresso pitcher to pour
            print("⬇️ Step 3/7: Tilting espresso pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", 142.020347, -26.797349, -119.286076, -47.654450, -56.933253, -102.391535)
            
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
            neutral_result = run_skill("gotoJ_deg", 138.578024, -22.115324, -126.765855, -38.694157, -57.964712, 3.173887)
            
            if neutral_result is False:
                print("[ERROR] Failed to return to neutral position")
                return False
            print("   ✅ Successfully returned to neutral position")
            
        else:  # stage_2
            print("🎯 Stage 2 pouring sequence...")
            
            # Step 2: Approach stage 2 position
            print("📍 Step 2/7: Positioning for stage 2 pouring...")
            pos2_result = run_skill("gotoJ_deg", 145.885393, -26.409357, -118.242817, -43.978546, -58.850444, -0.109599)
            
            if pos2_result is False:
                print("[ERROR] Failed to approach stage 2 position")
                return False
            print("   ✅ Successfully positioned for stage 2")
            
            # Step 3: Tilt espresso pitcher to pour
            print("⬇️ Step 3/7: Tilting espresso pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", 145.462132, -32.355488, -108.068238, -53.572020, -58.845487, -105.385536)
            
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
            neutral_result = run_skill("gotoJ_deg", 145.885393, -26.409357, -118.242817, -43.978546, -58.850444, -0.109599)
            
            if neutral_result is False:
                print("[ERROR] Failed to return to neutral position")
                return False
            print("   ✅ Successfully returned to neutral position")
        
        # Step 5: Move to intermediate position
        print("📍 Step 5/7: Moving to intermediate position...")
        inter_result = run_skill("gotoJ_deg", 121.236795, -29.537004, -136.110522, -14.093591, -58.933034, -0.146524)
        
        if inter_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        # Step 6: Rotate back (first rotation)
        print("🔄 Step 6/7: Rotating back to original orientation (first rotation)...")
        rotate_result = run_skill("moveJ_deg", -45, 0, -15, -30, 0, 0)
        
        if rotate_result is False:
            print("[ERROR] Failed to rotate back")
            return False
        print("   ✅ Successfully rotated back")
        
        # Step 7: Rotate back (second rotation)
        print("🔄 Step 7/7: Rotating back to original orientation (second rotation)...")
        rotate_result = run_skill("moveJ_deg", -15, 0, 15, 30, 0, 0)
        
        if rotate_result is False:
            print("[ERROR] Failed to rotate back")
            return False
        print("   ✅ Successfully rotated back")

        # Step 7: Return to holding position
        print("🏠 Returning to holding position...")
        final_result = run_skill("gotoJ_deg", 31.076585, -40.253154, -136.313679, -3.210446, -58.931112, -0.206957)
        
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
        approach_result = run_skill("gotoJ_deg", 67.341492, -49.798077, -99.061142, -30.809935, -22.613216, -0.457894)
        if approach_result is False:
            print("[ERROR] Failed to approach hot water dispenser")
            return False
        print("   ✅ Successfully approached hot water dispenser")
        
        # Step 2: Position espresso pitcher under hot water outlet
        print("📍 Step 2/2: Positioning espresso pitcher under hot water outlet...")
        position_result = run_skill("gotoJ_deg", 61.759601, -52.680407, -91.236745, -35.814578, -28.197699, -0.388979)
        if position_result is False:
            print("[ERROR] Failed to position espresso pitcher under hot water outlet")
            return False
        print("   ✅ Successfully positioned under hot water outlet")
        
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
        
        # Step 1: Move away from hot water outlet
        print("⬆️ Step 1/2: Moving away from hot water outlet...")
        retreat_result = run_skill("gotoJ_deg", 67.341492, -49.798077, -99.061142, -30.809935, -22.613216, -0.457894)
        if retreat_result is False:
            print("[ERROR] Failed to move away from hot water outlet")
            return False
        print("   ✅ Successfully moved away from outlet")
        
        # Step 2: Return to holding position
        print("🏠 Step 2/2: Returning to holding position...")
        final_result = run_skill("gotoJ_deg", 31.076585, -40.253154, -136.313679, -3.210446, -58.931112, -0.206957)
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
        
    Returns:
        bool: True if espresso pitcher returned successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during return process
        
    Example:
        success = return_espresso_pitcher(port='port_1')
        if success:
            print("Espresso pitcher returned successfully")
    """
    try:
        # Extract and validate port parameter
        port = params.get("port", "port_2")  # Default to port_2
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
            print("🎯 Step 1/4: Approaching espresso pitcher 1 return position...")
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")
            
            if approach_result is False:
                print("[ERROR] Failed to approach espresso pitcher 1 return position")
                return False
            print("   ✅ Successfully approached pitcher 1 return position")
            
            print("📍 Positioning espresso pitcher 1 for return...")
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")
            
            if mount_result is False:
                print("[ERROR] Failed to position espresso pitcher 1 for return")
                return False
            print("   ✅ Successfully positioned pitcher 1 for return")
            
            print("🤏 Releasing espresso pitcher 1...")
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            release_result = run_skill("set_gripper_position", 75, 0)
            
            if release_result is False:
                print("[ERROR] Failed to release espresso pitcher 1")
                return False
            print("   ✅ Successfully released pitcher 1")
            
            print("⬅️ Retreating from espresso pitcher 1...")
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
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
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        
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
    'pour_espresso_pitcher': pour_espresso_pitcher,
    'get_hot_water': get_hot_water,
    'with_hot_water': with_hot_water,
    'return_espresso_pitcher': return_espresso_pitcher,
    'tamper': tamper,
}