import time
from ..params import GRAB_PAPER_CUP_PARAMS, PLACE_PAPER_CUP_PARAMS
from ..manipulate_node import run_skill

# Import home positions from espresso module
Espresso_home = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)


def grab_paper_cup(**params):
    """
    Grab a paper cup of specified size from the paper cup dispenser.
    
    This function performs the complete paper cup grabbing workflow:
    1. Moves to espresso home position
    2. Navigates to paper cup dispenser area (avoiding espresso machine)
    3. Positions for paper cup grab based on size parameters
    4. Executes approach, grip, and retreat sequence
    5. Returns to intermediate position ready for placement
    
    Args:
        size (str): Paper cup size to grab ('9oz', '12oz', etc.)
        
    Returns:
        bool: True if paper cup grabbed successfully, False otherwise
        
    Example:
        success = grab_paper_cup(size='12oz')
        if success:
            print("Paper cup grabbed successfully")
    """
    try:
        size = params.get("size")
        cup_params = GRAB_PAPER_CUP_PARAMS.get(str(size))
        
        # Validate parameters and use default if not found
        if not cup_params:
            print(f"[ERROR] unknown paper cup size: {size!r}, using default 12oz")
            cup_params = GRAB_PAPER_CUP_PARAMS.get("12oz")
            if not cup_params:
                print("[ERROR] Default 12oz parameters not found in GRAB_PAPER_CUP_PARAMS")
                return False
        
        print(f"🥤 Starting paper cup grab sequence for size: {size}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home position...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Twist to avoid hitting the espresso machine during navigation
        print("🔄 Navigating around espresso machine...")
        twist_result = run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0)
        if twist_result is False:
            print("[ERROR] Failed to twist around espresso machine")
            return False
        
        # Step 3: Move to paper cup grabbing area
        print("📍 Moving to paper cup dispenser area...")
        cup_area_result = run_skill("gotoJ_deg", 120.389030, 22.860609, -73.526848, -39.810959, 90.144394, -154.586288)
        if cup_area_result is False:
            print("[ERROR] Failed to move to paper cup dispenser area")
            return False
        
        # Step 4: Rotate joint angles to back away before approach
        print("⬅️ Backing away for approach...")
        if 'twist_back' in cup_params:
            twist_back_result = run_skill("moveJ_deg", *cup_params['twist_back'])
            if twist_back_result is False:
                print("[ERROR] Failed to execute twist back movement")
                return False
        
        # Step 5: Move end-effector into approach position
        print("🎯 Moving to approach position...")
        if 'approach' in cup_params:
            approach_result = run_skill("moveEE", *cup_params['approach'])
            if approach_result is False:
                print("[ERROR] Failed to move to approach position")
                return False
        
        # Step 6: Close gripper to grasp the paper cup
        print("🤏 Gripping paper cup...")
        if 'grip_width' in cup_params:
            grip_result = run_skill("set_gripper_position", 255, cup_params['grip_width'])
            if grip_result is False:
                print("[ERROR] Failed to grip paper cup")
                return False
        
        # Step 7: Retract after gripping
        print("⬆️ Retracting with paper cup...")
        if 'retreat' in cup_params:
            retreat_result = run_skill("moveEE", *cup_params['retreat'])
            if retreat_result is False:
                print("[ERROR] Failed to retreat with paper cup")
                return False
        
        # Step 8: Move to intermediate position ready for placement
        print("📍 Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        print(f"✅ Paper cup grab sequence completed successfully for size: {size}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup grab: {e}")
        return False

def place_paper_cup(**params):
    """
    Place a paper cup at the specified staging area.
    
    This function places a previously grabbed paper cup at a designated serving stage:
    1. Moves from intermediate position to staging area
    2. Adjusts orientation for precise placement
    3. Positions paper cup at target location
    4. Releases paper cup and retreats safely
    5. Returns to espresso home position
    
    Args:
        stage (str): Target stage for paper cup placement ('stage_1', 'stage_2', etc.)
        
    Returns:
        bool: True if paper cup placed successfully, False otherwise
        
    Example:
        success = place_cup(stage='stage_1')
        if success:
            print("Paper cup placed successfully")
    """
    try:
        stage = params.get("stage")
        stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
        
        # Validate stage parameter
        if not stage_params:
            print(f"[ERROR] unknown stage: {stage!r}, available stages: {list(PLACE_PAPER_CUP_PARAMS.keys())}")
            return False
        
        print(f"📍 Starting paper cup placement sequence for: {stage}")
        
        # Step 1: Start from intermediate position
        print("📍 Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        # Step 2: Move into staging twist angle
        print("🔄 Adjusting orientation for staging...")
        if 'twist' in stage_params:
            twist_result = run_skill("moveJ_deg", *stage_params['twist'])
            if twist_result is False:
                print("[ERROR] Failed to execute staging twist")
                return False
        
        # Step 3: Move to target placement pose
        print("🎯 Moving to placement position...")
        if 'pose' in stage_params:
            pose_result = run_skill("gotoJ_deg", *stage_params['pose'])
            if pose_result is False:
                print("[ERROR] Failed to move to placement pose")
                return False
        
        # Step 4: Open gripper to release paper cup
        print("🤏 Releasing paper cup...")
        release_result = run_skill("set_gripper_position", 50, 0)
        if release_result is False:
            print("[ERROR] Failed to release paper cup")
            return False
        
        # Small delay to ensure paper cup is properly released
        time.sleep(1.0)
        
        # Step 5: Move up after placing paper cup
        print("⬆️ Moving up after placement...")
        up_result = run_skill("moveEE", 0, 0, 150, 0, 0, 0)
        if up_result is False:
            print("[ERROR] Failed to move up after placement")
            return False
        
        # Step 6: Move to staging home position
        print("🏠 Moving to staging home...")
        if 'stage_home' in stage_params:
            stage_home_result = run_skill("gotoJ_deg", *stage_params['stage_home'])
            if stage_home_result is False:
                print("[ERROR] Failed to move to staging home")
                return False
        
        # Step 7: Untwist back towards machine
        print("🔄 Untwisting back towards machine...")
        if 'twist_back' in stage_params:
            twist_back_result = run_skill("moveJ_deg", *stage_params['twist_back'])
            if twist_back_result is False:
                print("[ERROR] Failed to untwist back")
                return False
        
        # Step 8: Return to espresso home
        print("🏠 Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ Paper cup placement sequence completed successfully for: {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup placement: {e}")
        return False

def serve_paper_cup(**params):
    """
    Serve a paper cup from staging area to customer delivery point.
    
    This function handles the final step of drink service:
    1. Moves from espresso home to pickup position
    2. Grabs the prepared paper cup from staging
    3. Transports paper cup to customer delivery area
    4. Places paper cup for customer pickup
    5. Returns to espresso home position
    
    Args:
        stage (str): Source stage where paper cup is located ('stage_1', 'stage_2', etc.)
        
    Returns:
        bool: True if paper cup served successfully, False otherwise
        
    Example:
        success = serve(stage='stage_1')
        if success:
            print("Paper cup served successfully")
    """
    try:
        stage = params.get("stage")
        stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
        
        # Validate stage parameter
        if not stage_params:
            print(f"[ERROR] unknown stage {stage!r}, available stages: {list(PLACE_PAPER_CUP_PARAMS.keys())}")
            return False
        
        print(f"🚚 Starting paper cup serving sequence from: {stage}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Navigate to pickup position
        print("🔄 Navigating to pickup area...")
        if 'twist_serve' in stage_params:
            twist_serve_result = run_skill("moveJ_deg", *stage_params['twist_serve'])
            if twist_serve_result is False:
                print("[ERROR] Failed to execute serving twist")
                return False
        
        # Step 3: Move to paper cup pickup position
        print("📍 Moving to paper cup pickup position...")
        if 'pick' in stage_params:
            pick_result = run_skill("gotoJ_deg", *stage_params['pick'])
            if pick_result is False:
                print("[ERROR] Failed to move to pickup position")
                return False
        
        # Step 4: Lower to paper cup level
        print("⬇️ Lowering to paper cup level...")
        lower_result = run_skill("moveEE", 0, 0, -140, 0, 0, 0)
        if lower_result is False:
            print("[ERROR] Failed to lower to paper cup level")
            return False
        
        # Step 5: Grip the paper cup for serving
        print("🤏 Gripping paper cup for serving...")
        grip_result = run_skill("set_gripper_position", 55, 125)
        if grip_result is False:
            print("[ERROR] Failed to grip paper cup for serving")
            return False
        
        # Step 6: Set slower servo timing for careful handling
        print("⚙️ Setting careful servo timing...")
        timing_result = run_skill("set_servo_timing", 0.20)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing, continuing...")
        
        # Step 7: Lift paper cup
        print("⬆️ Lifting paper cup...")
        if 'pick' in stage_params:
            lift_result = run_skill("gotoJ_deg", *stage_params['pick'])
            if lift_result is False:
                print("[ERROR] Failed to lift paper cup")
                return False
        
        # Step 8: Move above serving area
        print("📍 Moving above serving area...")
        if 'above_serve' in stage_params:
            above_serve_result = run_skill("gotoJ_deg", *stage_params['above_serve'])
            if above_serve_result is False:
                print("[ERROR] Failed to move above serving area")
                return False
        
        # Step 9: Lower to serving position
        print("⬇️ Lowering to serving position...")
        if 'serve' in stage_params:
            serve_result = run_skill("gotoJ_deg", *stage_params['serve'])
            if serve_result is False:
                print("[ERROR] Failed to move to serving position")
                return False
        
        # Step 10: Release paper cup for customer
        print("🤏 Releasing paper cup for customer...")
        release_result = run_skill("set_gripper_position", 55, 0)
        if release_result is False:
            print("[ERROR] Failed to release paper cup")
            return False
        
        # Step 11: Reset servo timing
        print("⚙️ Resetting servo timing...")
        reset_timing_result = run_skill("set_servo_timing", 0.10)
        if reset_timing_result is False:
            print("[WARNING] Failed to reset servo timing, continuing...")
        
        # Small delay to ensure paper cup is properly placed
        time.sleep(1.0)
        
        # Step 12: Move up after placing
        print("⬆️ Moving up after serving...")
        up_result = run_skill("moveEE", 0, 0, 140, 0, 0, 0)
        if up_result is False:
            print("[ERROR] Failed to move up after serving")
            return False
        
        # Step 13: Return to staging home
        print("🏠 Moving to staging home...")
        staging_home_result = run_skill("gotoJ_deg", 106.460129, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if staging_home_result is False:
            print("[ERROR] Failed to move to staging home")
            return False
        
        # Step 14: Twist joint 1 to reach espresso home
        print("🔄 Twisting to reach espresso home...")
        final_twist_result = run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0)
        if final_twist_result is False:
            print("[ERROR] Failed to execute final twist")
            return False
        
        # Step 15: Return to espresso home
        print("🏠 Returning to espresso home...")
        final_home_result = run_skill("gotoJ_deg", *Espresso_home)
        if final_home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ Paper cup serving sequence completed successfully from: {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup serving: {e}")
        return False

# Register for CLI discovery
SEQUENCES = {
    'grab_paper_cup': grab_paper_cup,
    'place_paper_cup': place_paper_cup,
    'serve_paper_cup': serve_paper_cup,
}
