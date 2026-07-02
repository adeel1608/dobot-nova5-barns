"""
test.py - Robot Arm Test Sequences

This module contains test sequences for validating robot arm functionality,
including tool manipulation, positioning, and gripper operations.

Functions:
    test(**params): Comprehensive milk frother tool manipulation test
    test_1(): Espresso machine interaction test sequence
    
Author: BARNS Development Team
Version: 1.2.0
Last Modified: 2025-01-07
"""

import time
import traceback
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill
from oms_v1.params import HOME_ANGLES, PULL_ESPRESSO_PARAMS
from oms_v1.sequences.home import home

# Global variables for storing positions
below_espresso_port = None
mount_espresso_port = None


def test(**params) -> bool:
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
        port (str): Target port ('port_1', 'port_2', or 'port_3')
        
    Returns:
        bool: True if portafilter unmounted successfully, False otherwise
        
    Example:
        success = test(port='port_1')
        if success:
            print("Portafilter unmounted successfully")
    """
    global below_espresso_port, mount_espresso_port
    
    try:
        port = params.get("port")
        if not port:
            print("[ERROR] No port specified. Please provide 'port' parameter.")
            print("[INFO] Valid ports: 'port_1', 'port_2', 'port_3'")
            return False
            
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] Unknown port number: {port!r}")
            print(f"[INFO] Available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"📤 Starting portafilter unmount sequence for {port}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *port_params['home'])
        
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False

        # Step 2: Approach the portafilter group (only for port_1 and port_3)
        if port in ['port_1', 'port_3']:
            print(f"🎯 Approaching portafilter {port_params['portafilter_number']}...")
            approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'], True)
            if approach_result is False:
                print("[ERROR] Failed to approach portafilter")
                return False
        
        # Step 3: Mount to the portafilter for secure grip
        print("🔧 Mounting to portafilter...")
        mount_result = run_skill("mount_machine", "three_group_espresso", port_params['portafilter_number'], True)
        
        if mount_result is False:
            print("[ERROR] Failed to mount to portafilter")
            return False
        
        # Step 4: Close gripper to secure portafilter
        print("🤏 Securing portafilter with gripper...")
        run_skill("sync")
        grip_result = run_skill("set_gripper_position", 255, 255)
        if grip_result is False:
            print("[ERROR] Failed to close gripper")
            return False
        
        # Step 5: Release tension for smooth operation
        print("😌 Releasing tension...")
        tension_result = run_skill("release_tension")
        if tension_result is False:
            print("[ERROR] Failed to release tension")
            return False
        
        # Step 6: Enforce proper orientation
        print("📐 Enforcing proper orientation...")
        orient_result = run_skill("enforce_rxry")
        if orient_result is False:
            print("[ERROR] Failed to enforce orientation")
            return False
        
        # Synchronization point
        run_skill("sync")
        
        # Step 7: Rotate portafilter to unlock (-45 degrees)
        print("🔄 Rotating portafilter to unlock...")
        rotate_unlock_result = run_skill("move_portafilter_arc", -45)
        
        if rotate_unlock_result is False:
            print("[ERROR] Failed to rotate portafilter to unlock")
            return False
        
        # Step 8: Release tension after unlock rotation
        print("😌 Releasing tension after unlock...")
        tension_unlock_result = run_skill("release_tension")
        
        # Capture current position for reference
        mount_espresso_port = run_skill("current_angles")
        
        if tension_unlock_result is False:
            print("[ERROR] Failed to release tension after unlock rotation")
            return False
        
        # Step 9: Rotate portafilter to lock position (+47 degrees from unlock)
        print("🔄 Rotating portafilter to lock...")
        rotate_lock_result = run_skill("move_portafilter_arc", 47)
        if rotate_lock_result is False:
            print("[ERROR] Failed to rotate portafilter to lock")
            return False
        
        # Step 10: Open gripper to release portafilter
        print("🤏 Opening gripper to release portafilter...")
        run_skill("sync")
        release_result = run_skill("set_gripper_position", 255, 0)
        if release_result is False:
            print("[ERROR] Failed to open gripper")
            return False
            
        # Step 11: Move back to approach position (only for port_1 and port_3)
        if port in ['port_1', 'port_3']:
            print("⬅️ Moving back from portafilter...")
            back_approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'], True)
            if back_approach_result is False:
                print("[ERROR] Failed to move back from portafilter")
                return False
        
        # Step 12: Return to espresso home
        print("🏠 Returning to espresso home...")
        final_home_result = run_skill("gotoJ_deg", *port_params['home'])
        if final_home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ Portafilter mount sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during portafilter operation: {e}")
        print("Stack trace:")
        traceback.print_exc()
        return False


def test_1(**params) -> bool:
    """
    Espresso machine interaction test sequence.
    
    This function performs a specialized test for espresso machine operations:
    - Positioning near espresso machine group
    - Gripper engagement and manipulation
    - Tension control and force application
    - Multi-cycle operation validation
    
    The test runs 5 iterations with espresso-specific movements and controls.
    Each iteration tests the complete espresso preparation workflow.
    
    Returns:
        bool: Test execution result
            - True: All espresso test cycles completed successfully  
            - False: Test failed during execution
            
    Raises:
        Exception: Captures any errors during espresso test execution
        
    Example:
        >>> # Execute espresso machine test
        >>> success = test_1()
        >>> if success:
        ...     print("☕ Espresso test passed")
        ... else:
        ...     print("❌ Espresso test failed")
    """
    
    print("☕ Initializing espresso machine interaction test")
    print("📋 Test Type: Three-Group Espresso Machine Operation")
    print("🔄 Iterations: 5 cycles")
    print("=" * 50)
    
    try:
        for iteration in range(1, 6):
            print(f"\n☕ ESPRESSO CYCLE {iteration}/5")
            
            # Step 1: Move to espresso machine position
            print("📍 Step 1: Positioning at espresso machine...")
            espresso_angles = (42.159162,16.269149,-135.156441,-81.822150,-49.784457,13.771214)
            position_result = run_skill("gotoJ_deg", *espresso_angles)
            
            if position_result is False:
                print(f"❌ [CYCLE {iteration}] Step 1 FAILED: Cannot reach espresso position")
                return False
            
            print("✅ Step 1 SUCCESS: Positioned at espresso machine")
            
            # Step 2: Execute controlled movement
            print("🎯 Step 2: Executing precision movement...")
            move_result = run_skill("moveJ_deg", 2.0, 2.0, 2.0, 2.0, 2.0, 2.0)
            
            if move_result is False:
                print(f"❌ [CYCLE {iteration}] Step 2 FAILED: Precision movement failed")
                return False
            
            print("✅ Step 2 SUCCESS: Precision movement completed")
            
            # Step 3: Return to exact espresso position with speed control
            print("⚡ Step 3: High-precision positioning...")
            precise_result = run_skill("gotoJ_deg", *espresso_angles, 1.0, 0.2)
            
            if precise_result is False:
                print(f"❌ [CYCLE {iteration}] Step 3 FAILED: High-precision positioning failed")
                return False
            
            print("✅ Step 3 SUCCESS: High-precision position achieved")
            time.sleep(0.25)
            
            # Step 4: Approach espresso group
            print("☕ Step 4: Approaching three-group espresso...")
            approach_result = run_skill("move_to", "three_group_espresso", 0.12)
            
            if approach_result is False:
                print(f"❌ [CYCLE {iteration}] Step 4 FAILED: Cannot approach espresso group")
                return False
            
            print("✅ Step 4 SUCCESS: Espresso group approached")
            
            # Step 5: Return to operating position
            print("↩️ Step 5: Returning to operating position...")
            return_result = run_skill("gotoJ_deg", *espresso_angles)
            
            if return_result is False:
                print(f"❌ [CYCLE {iteration}] Step 5 FAILED: Cannot return to operating position")
                return False
            
            print("✅ Step 5 SUCCESS: Operating position restored")
            
            # Step 6: Engage gripper at maximum force
            print("🤏 Step 6: Engaging gripper at maximum force...")
            run_skill("sync")
            grip_result = run_skill("set_gripper_position", 255, 255)
            
            if grip_result is False:
                print(f"❌ [CYCLE {iteration}] Step 6 FAILED: Gripper engagement failed")
                return False
            
            print("✅ Step 6 SUCCESS: Gripper fully engaged")
            
            # Step 7: Release tension for safety
            print("🔧 Step 7: Releasing mechanical tension...")
            tension_result = run_skill("release_tension")
            
            if tension_result is False:
                print(f"❌ [CYCLE {iteration}] Step 7 FAILED: Tension release failed")
                return False
            
            print("✅ Step 7 SUCCESS: Tension released")
            
            # Step 8: First force application cycle
            print("💪 Step 8: Applying operational force (Cycle 1)...")
            force1_result = run_skill("enforce_rxry")
            
            if force1_result is False:
                print(f"❌ [CYCLE {iteration}] Step 8 FAILED: Force application cycle 1 failed")
                return False
            
            print("✅ Step 8 SUCCESS: Force cycle 1 completed")
            time.sleep(0.6)  # Operational delay
            
            # Step 9: Second force application cycle
            print("💪 Step 9: Applying operational force (Cycle 2)...")
            force2_result = run_skill("enforce_rxry")
            
            if force2_result is False:
                print(f"❌ [CYCLE {iteration}] Step 9 FAILED: Force application cycle 2 failed")
                return False
            
            print("✅ Step 9 SUCCESS: Force cycle 2 completed")
            time.sleep(0.6)  # Operational delay
            
            # Step 10: Final tension release
            print("🔧 Step 10: Final tension release...")
            final_tension_result = run_skill("release_tension")
            
            if final_tension_result is False:
                print(f"❌ [CYCLE {iteration}] Step 10 FAILED: Final tension release failed")
                return False
            
            print("✅ Step 10 SUCCESS: Final tension released")
            
            # Step 11: Release gripper completely
            print("🤏 Step 11: Releasing gripper...")
            run_skill("sync")
            release_result = run_skill("set_gripper_position", 255, 0)
            
            if release_result is False:
                print(f"❌ [CYCLE {iteration}] Step 11 FAILED: Gripper release failed")
                return False
            
            print("✅ Step 11 SUCCESS: Gripper released")
            
            # Step 12: Final position confirmation
            print("📍 Step 12: Final position confirmation...")
            final_result = run_skill("gotoJ_deg", *espresso_angles)
            
            if final_result is False:
                print(f"❌ [CYCLE {iteration}] Step 12 FAILED: Final position confirmation failed")
                return False
            
            print("✅ Step 12 SUCCESS: Final position confirmed")
            print(f"🎉 ESPRESSO CYCLE {iteration}/5 COMPLETED SUCCESSFULLY")
            
            # Brief pause between cycles
            if iteration < 5:
                time.sleep(0.5)
        
        print("\n" + "=" * 50)
        print("🏆 ALL 5 ESPRESSO CYCLES COMPLETED SUCCESSFULLY!")
        print("✅ Espresso test validation: PASSED")
        print("☕ Machine interaction: VERIFIED")
        print("🤖 Robot precision: OPTIMAL")
        return True
        
    except Exception as e:
        error_msg = f"💥 CRITICAL ERROR: Espresso test execution failed"
        print(error_msg)
        print(f"   Error Type: {type(e).__name__}")
        print(f"   Error Message: {str(e)}")
        print("   Stack Trace:")
        traceback.print_exc()
        print("🚨 Espresso test sequence aborted")
        return False


# Test sequence registry for CLI discovery and execution
# This dictionary enables dynamic test discovery and execution
SEQUENCES = {
    'test': test,           # Comprehensive milk frother tool manipulation test
    'test_1': test_1,       # Espresso machine interaction test sequence
}
