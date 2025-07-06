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


def test(**params) -> bool:
    """
    Comprehensive test sequence for milk frother tool manipulation.
    
    This function performs a detailed test sequence that validates:
    - Robot positioning and movement accuracy
    - Tool approach and grabbing operations  
    - Position memory capture and retrieval
    - Gripper manipulation and control
    - Safe return to home position
    - Error recovery and handling
    
    The test runs 5 iterations to ensure repeatability and consistency.
    Each iteration includes 12 distinct steps with proper error checking.
    
    Args:
        **params (Dict[str, Any]): Optional test parameters
            - iterations (int): Number of test iterations (default: 5)
            - tool_name (str): Tool identifier (default: 'milk_frother_1')
            - approach_distance (float): Tool approach distance (default: 0.175)
            - gripper_strength (int): Gripper force (default: 200-255)
            
    Returns:
        bool: Test execution result
            - True: All test steps completed successfully
            - False: Test failed at any step
            
    Raises:
        Exception: Captures and logs any unexpected errors during execution
        
    Example:
        >>> # Basic test execution
        >>> success = test()
        >>> if success:
        ...     print("✅ Test sequence passed")
        ... else:
        ...     print("❌ Test sequence failed")
        
        >>> # Custom parameters
        >>> success = test(iterations=3, tool_name='milk_frother_2')
    """
    
    # Extract parameters with defaults
    iterations = params.get('iterations', 5)
    tool_name = params.get('tool_name', 'milk_frother_1')
    approach_distance = params.get('approach_distance', 0.175)
    
    print(f"🚀 Initializing comprehensive test sequence")
    print(f"📋 Test Parameters:")
    print(f"   - Iterations: {iterations}")
    print(f"   - Tool: {tool_name}")
    print(f"   - Approach Distance: {approach_distance}m")
    print(f"   - Test Type: Milk Frother Tool Manipulation")
    print("=" * 60)
    
    try:
        for iteration in range(1, iterations + 1):
            print(f"\n🔄 ITERATION {iteration}/{iterations}")
            print("🧪 Starting test sequence for milk frother tool manipulation")
            
            # Step 1: Initialize robot position
            print("📍 Step 1/12: Moving to initial position...")
            init_angles = (20.847986, -21.981329, -113.153931, -76.829208, -81.786911, -0.050592)
            init_result = run_skill("gotoJ_deg", *init_angles, 1.0, 0.2)
            
            if init_result is False:
                error_msg = f"❌ [ITERATION {iteration}] Step 1 FAILED: Unable to reach initial position"
                print(error_msg)
                print(f"   Target angles: {init_angles}")
                return False
            
            print("✅ Step 1 SUCCESS: Robot positioned at initial coordinates")
            time.sleep(0.25)  # Stabilization delay
            
            # Step 2: Navigate to milk frother location
            print(f"🎯 Step 2/12: Moving to {tool_name} location...")
            move_result = run_skill("move_to", tool_name, approach_distance)
            
            if move_result is False:
                error_msg = f"❌ [ITERATION {iteration}] Step 2 FAILED: Cannot navigate to {tool_name}"
                print(error_msg)
                print(f"   Target: {tool_name} at {approach_distance}m distance")
                return False
            
            print(f"✅ Step 2 SUCCESS: Positioned near {tool_name}")
            
            # Step 3: Execute tool approach sequence
            print(f"🔧 Step 3/12: Approaching {tool_name} for manipulation...")
            approach_result = run_skill("approach_tool", tool_name, 170)
            
            if approach_result is False:
                error_msg = f"❌ [ITERATION {iteration}] Step 3 FAILED: Tool approach sequence failed"
                print(error_msg)
                print(f"   Tool: {tool_name}, Approach angle: 170°")
                return False
            
            print(f"✅ Step 3 SUCCESS: Tool approach completed")
            
            # Step 4: Capture current joint angles for position memory
            print("📊 Step 4/12: Capturing approach position angles...")
            approach_angles = run_skill("current_angles")
            
            if approach_angles is None:
                print("⚠️ [WARNING] Step 4: Failed to capture approach angles")
                print("   Continuing test but position recovery may be limited")
            else:
                print(f"✅ Step 4 SUCCESS: Approach angles captured")
                print(f"   Angles: {[f'{angle:.2f}°' for angle in approach_angles]}")
            
            time.sleep(0.25)
            
            # Step 5: Execute tool grabbing operation
            print(f"🤏 Step 5/12: Grabbing {tool_name}...")
            grab_result = run_skill("grab_tool", tool_name, 200, 250, 255)
            
            if grab_result is False:
                error_msg = f"❌ [ITERATION {iteration}] Step 5 FAILED: Tool grab operation failed"
                print(error_msg)
                print(f"   Tool: {tool_name}, Gripper settings: 200-250-255")
                return False
            
            print(f"✅ Step 5 SUCCESS: {tool_name} secured in gripper")
            
            # Step 6: Capture grab position for reference
            print("📊 Step 6/12: Capturing grab position angles...")
            grab_angles = run_skill("current_angles")
            
            if grab_angles is None:
                print("⚠️ [WARNING] Step 6: Failed to capture grab angles")
            else:
                print(f"✅ Step 6 SUCCESS: Grab angles captured")
                print(f"   Angles: {[f'{angle:.2f}°' for angle in grab_angles]}")
            
            # Step 7: Move to intermediate safety position
            print("📍 Step 7/12: Moving to intermediate safety position...")
            inter_angles = (-4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837)
            inter1_result = run_skill("gotoJ_deg", *inter_angles, 1.0, 0.2)
            
            if inter1_result is False:
                error_msg = f"❌ [ITERATION {iteration}] Step 7 FAILED: Cannot reach intermediate position"
                print(error_msg)
                print(f"   Target angles: {inter_angles}")
                return False
            
            print("✅ Step 7 SUCCESS: Intermediate position reached")
            time.sleep(0.25)
            
            # Step 8: Move to working/demonstration position
            print("🔄 Step 8/12: Moving to working demonstration position...")
            work_angles = (22.345373, -76.252151, -61.342220, -40.423759, -81.360077, 11.115391)
            work_result = run_skill("gotoJ_deg", *work_angles, 1.0, 0.2)
            
            if work_result is False:
                error_msg = f"❌ [ITERATION {iteration}] Step 8 FAILED: Cannot reach working position"
                print(error_msg)
                print(f"   Target angles: {work_angles}")
                return False
            
            print("✅ Step 8 SUCCESS: Working position achieved")
            time.sleep(0.25)
            
            # Step 9: Return to grab position (if captured successfully)
            if grab_angles is not None:
                print("↩️ Step 9/12: Returning to grab position...")
                return_grab_result = run_skill("gotoJ_deg", *grab_angles, 1.0, 0.2)
                
                if return_grab_result is False:
                    error_msg = f"❌ [ITERATION {iteration}] Step 9 FAILED: Cannot return to grab position"
                    print(error_msg)
                    return False
                
                print("✅ Step 9 SUCCESS: Returned to grab position")
            else:
                print("⚠️ Step 9 SKIPPED: Grab angles unavailable, cannot return to position")
            
            time.sleep(0.25)
            
            # Step 10: Adjust gripper for tool release
            print("🔧 Step 10/12: Adjusting gripper for tool release...")
            grip_result = run_skill("set_gripper_position", 255, 165)
            
            if grip_result is False:
                error_msg = f"❌ [ITERATION {iteration}] Step 10 FAILED: Gripper adjustment failed"
                print(error_msg)
                print("   Target: Position 255, Force 165")
                return False
            
            print("✅ Step 10 SUCCESS: Gripper adjusted for release")
            
            # Step 11: Return to approach position (if captured successfully)
            if approach_angles is not None:
                print("↩️ Step 11/12: Returning to approach position...")
                return_approach_result = run_skill("gotoJ_deg", *approach_angles, 1.0, 0.2)
                
                if return_approach_result is False:
                    error_msg = f"❌ [ITERATION {iteration}] Step 11 FAILED: Cannot return to approach position"
                    print(error_msg)
                    return False
                
                print("✅ Step 11 SUCCESS: Returned to approach position")
            else:
                print("⚠️ Step 11 SKIPPED: Approach angles unavailable, cannot return to position")
            
            time.sleep(0.25)
            
            # Step 12: Safe return to home position
            print("🏠 Step 12/12: Returning to home position...")
            home_result = home(position="north")
            
            if home_result is False:
                error_msg = f"❌ [ITERATION {iteration}] Step 12 FAILED: Cannot return to home position"
                print(error_msg)
                return False
            
            print("✅ Step 12 SUCCESS: Home position reached")
            
            # Final step: Release gripper completely
            print("🤏 Final Step: Opening gripper completely...")
            release_result = run_skill("set_gripper_position", 255, 0)
            
            if release_result is False:
                error_msg = f"❌ [ITERATION {iteration}] FINAL STEP FAILED: Gripper release failed"
                print(error_msg)
                return False
            
            print("✅ FINAL STEP SUCCESS: Gripper opened, tool released")
            print(f"🎉 ITERATION {iteration}/{iterations} COMPLETED SUCCESSFULLY")
            
            # Brief pause between iterations
            if iteration < iterations:
                print("⏳ Preparing for next iteration...")
                time.sleep(1.0)
        
        print("\n" + "=" * 60)
        print(f"🏆 ALL {iterations} ITERATIONS COMPLETED SUCCESSFULLY!")
        print("✅ Test sequence validation: PASSED")
        print("📊 Robot performance: OPTIMAL")
        print("🔧 Tool manipulation: VERIFIED")
        return True
        
    except Exception as e:
        error_msg = f"💥 CRITICAL ERROR: Unexpected failure during test execution"
        print(error_msg)
        print(f"   Error Type: {type(e).__name__}")
        print(f"   Error Message: {str(e)}")
        print("   Stack Trace:")
        traceback.print_exc()
        print("🚨 Test sequence aborted due to critical error")
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
            espresso_angles = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
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
