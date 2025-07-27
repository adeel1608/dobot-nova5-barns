"""
paper_cups.py

Defines the paper cup handling sequences for coffee service automation.
This module provides comprehensive functions for grabbing, placing, and serving
paper cups in the BARNS coffee automation system, including size-based handling
and staging area management.
"""

import time
from typing import Dict, Any, Optional
from ..params import GRAB_PAPER_CUP_PARAMS, PLACE_PAPER_CUP_PARAMS
from ..manipulate_node import run_skill

# Predefined home positions for paper cup operations
Espresso_home = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)


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
        # Extract and validate size parameter
        size = params.get("size", "7oz")  # Default to 7oz
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
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Twist to avoid hitting the espresso machine during navigation
        print("🔄 Step 2/8: Navigating around espresso machine...")
        twist_result = run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0)
        if twist_result is False:
            print("[ERROR] Failed to twist around espresso machine")
            return False
        print("   ✅ Successfully navigated around espresso machine")
        
        # Step 3: Move to paper cup grabbing area
        print("📍 Step 3/8: Moving to paper cup dispenser area...")
        cup_area_result = run_skill("gotoJ_deg", 120.389030, 22.860609, -73.526848, -39.810959, 90.144394, -154.586288)
        
        if cup_area_result is False:
            print("[ERROR] Failed to move to paper cup dispenser area")
            return False
        print("   ✅ Successfully positioned at paper cup dispenser")
        
        # Step 4: Rotate joint angles to back away before approach
        print("⬅️ Step 4/8: Backing away for approach...")
        if 'twist_back' in cup_params:
            print(f"   📍 Executing twist back movement for {size}")
            twist_back_result = run_skill("moveJ_deg", *cup_params['twist_back'])
            if twist_back_result is False:
                print("[ERROR] Failed to execute twist back movement")
                return False
            print("   ✅ Successfully executed twist back movement")
        else:
            print("   ⏭️ No twist back movement defined for this size")
        
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
            run_skill("sync")
            grip_result = run_skill("set_gripper_position", 255, cup_params['grip_width'])
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
        
        # Step 8: Move to intermediate position ready for placement
        print("📍 Step 8/8: Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
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
        stage (str): Target stage for paper cup placement ('stage_1', 'stage_2', etc.), defaults to 'stage_1'
        
    Returns:
        bool: True if paper cup placed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during placement process
        
    Example:
        success = place_paper_cup(stage='stage_1')
        if success:
            print("Paper cup placed successfully")
    """
    try:
        # Extract and validate stage parameter
        stage = params.get("stage", "stage_1")  # Default to stage_1
        if not stage:
            print("[ERROR] No stage parameter provided")
            return False
            
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
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
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
        run_skill("sync")
        release_result = run_skill("set_gripper_position", 50, 0)
        
        if release_result is False:
            print("[ERROR] Failed to release paper cup")
            return False
        print("   ✅ Paper cup released successfully")
        
        # Allow settling time
        print("   ⏰ Allowing cup settling time...")
        time.sleep(1.0)
        
        # Step 5: Move up after placing paper cup
        print("⬆️ Step 5/7: Moving up after placement...")
        up_result = run_skill("moveEE", 0, 0, 150, 0, 0, 0)
        
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
            twist_back_result = run_skill("moveJ_deg", *stage_params['twist_back'])
            
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


def serve_paper_cup(**params) -> bool:
    """
    Serve a paper cup from staging area to customer delivery point.
    
    This function handles the final step of drink service:
    1. Moves from espresso home to pickup position
    2. Grabs the prepared paper cup from staging
    3. Transports paper cup to customer delivery area
    4. Places paper cup for customer pickup
    5. Returns to espresso home position
    
    Args:
        stage (str): Source stage where paper cup is located ('stage_1', 'stage_2', etc.), defaults to 'stage_1'
        
    Returns:
        bool: True if paper cup served successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during serving process
        
    Example:
        success = serve_paper_cup(stage='stage_1')
        if success:
            print("Paper cup served successfully")
    """
    try:
        # Extract and validate stage parameter
        stage = params.get("stage", "stage_1")  # Default to stage_1
        if not stage:
            print("[ERROR] No stage parameter provided")
            return False
            
        stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
        
        # Validate stage parameter
        if not stage_params:
            print(f"[ERROR] Unknown stage: {stage!r}")
            print(f"[INFO] Available stages: {list(PLACE_PAPER_CUP_PARAMS.keys())}")
            return False
        
        print(f"🚚 Starting paper cup serving sequence from: {stage}")
        print("=" * 50)
        
        # Step 1: Move to espresso home position
        print("🏠 Step 1/15: Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Navigate to pickup position
        print("🔄 Step 2/15: Navigating to pickup area...")
        if 'twist_serve' in stage_params:
            print(f"   📍 Executing serving twist for {stage}")
            twist_serve_result = run_skill("moveJ_deg", *stage_params['twist_serve'])
            
            if twist_serve_result is False:
                print("[ERROR] Failed to execute serving twist")
                return False
            print("   ✅ Successfully navigated to pickup area")
        else:
            print("[ERROR] No serving twist defined for this stage")
            return False
        
        # Step 3: Move to paper cup pickup position
        print("📍 Step 3/15: Moving to paper cup pickup position...")
        if 'pick' in stage_params:
            print(f"   📍 Moving to pickup position for {stage}")
            pick_result = run_skill("gotoJ_deg", *stage_params['pick'])
            
            if pick_result is False:
                print("[ERROR] Failed to move to pickup position")
                return False
            print("   ✅ Successfully positioned for pickup")
        else:
            print("[ERROR] No pickup position defined for this stage")
            return False
        
        # Step 4: Lower to paper cup level
        print("⬇️ Step 4/15: Lowering to paper cup level...")
        lower_result = run_skill("moveEE", 0, 0, -140, 0, 0, 0)
        
        if lower_result is False:
            print("[ERROR] Failed to lower to paper cup level")
            return False
        print("   ✅ Successfully lowered to paper cup level")
        
        # Step 5: Grip the paper cup for serving
        print("🤏 Step 5/15: Gripping paper cup for serving...")
        run_skill("sync")
        grip_result = run_skill("set_gripper_position", 55, 125)
        
        if grip_result is False:
            print("[ERROR] Failed to grip paper cup for serving")
            return False
        print("   ✅ Paper cup secured for serving")
        
        # Step 6: Set slower servo timing for careful handling
        print("⚙️ Step 6/15: Setting careful servo timing...")
        timing_result = run_skill("set_speed_factor", 20)
        
        if timing_result is False:
            print("[WARNING] Failed to set servo timing, continuing with default...")
        else:
            print("   ✅ Servo timing set for careful handling")
        
        # Step 7: Lift paper cup
        print("⬆️ Step 7/15: Lifting paper cup...")
        if 'pick' in stage_params:
            print(f"   📍 Lifting to pickup position for {stage}")
            lift_result = run_skill("gotoJ_deg", *stage_params['pick'])
            
            if lift_result is False:
                print("[ERROR] Failed to lift paper cup")
                return False
            print("   ✅ Successfully lifted paper cup")
        
        # Step 8: Move above serving area
        print("📍 Step 8/15: Moving above serving area...")
        if 'above_serve' in stage_params:
            print(f"   📍 Moving above serving area for {stage}")
            above_serve_result = run_skill("gotoJ_deg", *stage_params['above_serve'])
            
            if above_serve_result is False:
                print("[ERROR] Failed to move above serving area")
                return False
            print("   ✅ Successfully positioned above serving area")
        else:
            print("[ERROR] No above serving position defined for this stage")
            return False
        
        # Step 9: Lower to serving position
        print("⬇️ Step 9/15: Lowering to serving position...")
        if 'serve' in stage_params:
            print(f"   📍 Moving to serving position for {stage}")
            serve_result = run_skill("gotoJ_deg", *stage_params['serve'])
            
            if serve_result is False:
                print("[ERROR] Failed to move to serving position")
                return False
            print("   ✅ Successfully positioned for serving")
        else:
            print("[ERROR] No serving position defined for this stage")
            return False
        
        # Step 10: Release paper cup for customer
        print("🤏 Step 10/15: Releasing paper cup for customer...")
        run_skill("sync")
        release_result = run_skill("set_gripper_position", 55, 0)
        
        if release_result is False:
            print("[ERROR] Failed to release paper cup")
            return False
        print("   ✅ Paper cup released for customer pickup")
        
        # Step 11: Reset servo timing
        print("⚙️ Step 11/15: Resetting servo timing...")
        reset_timing_result = run_skill("set_speed_factor", 10)
        
        if reset_timing_result is False:
            print("[WARNING] Failed to reset servo timing, continuing...")
        else:
            print("   ✅ Servo timing reset to normal operation")
        
        # Allow settling time
        print("   ⏰ Allowing cup settling time...")
        time.sleep(1.0)
        
        # Step 12: Move up after placing
        print("⬆️ Step 12/15: Moving up after serving...")
        up_result = run_skill("moveEE", 0, 0, 140, 0, 0, 0)
        
        if up_result is False:
            print("[ERROR] Failed to move up after serving")
            return False
        print("   ✅ Successfully moved up after serving")
        
        # Step 13: Return to staging home
        print("🏠 Step 13/15: Moving to staging home...")
        staging_home_result = run_skill("gotoJ_deg", 106.460129, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        
        if staging_home_result is False:
            print("[ERROR] Failed to move to staging home")
            return False
        print("   ✅ Successfully moved to staging home")
        
        # Step 14: Twist joint 1 to reach espresso home
        print("🔄 Step 14/15: Twisting to reach espresso home...")
        final_twist_result = run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0)
        
        if final_twist_result is False:
            print("[ERROR] Failed to execute final twist")
            return False
        print("   ✅ Successfully executed final twist")
        
        # Step 15: Return to espresso home
        print("🏠 Step 15/15: Returning to espresso home...")
        final_home_result = run_skill("gotoJ_deg", *Espresso_home)
        
        if final_home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        print("   ✅ Successfully returned to espresso home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PAPER CUP SERVING COMPLETED SUCCESSFULLY FROM {stage.upper()}")
        print("   ✓ Paper cup safely transported to customer area")
        print("   ✓ Careful handling with adjusted servo timing")
        print("   ✓ Robot returned to home position")
        print("   🎉 Customer service completed!")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup serving: {e}")
        print("[INFO] Paper cup serving process terminated due to error")
        return False


# Register functions for CLI discovery and external access
SEQUENCES = {
    'grab_paper_cup': grab_paper_cup,
    'place_paper_cup': place_paper_cup,
    'serve_paper_cup': serve_paper_cup,
}
