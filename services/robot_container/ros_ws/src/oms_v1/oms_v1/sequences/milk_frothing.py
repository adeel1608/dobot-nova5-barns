import time
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home

# Global variables to store robot positions during milk frothing operations
# These are used to remember positions between function calls
approach_angles = None
grab_angles = None
froth_approach_angles = None
froth_mount_angles = None

def get_frother_position(**params):
    """
    Calibrate and record the milk frother position for future operations.
    
    This function performs calibration of the milk frother position:
    1. Moves to north-east home position for approach
    2. Opens gripper to prepare for positioning
    3. Performs multiple approaches to left steam wand for accuracy
    4. Records the calibrated position for future reference
    
    This calibration should be performed when setting up the milk frothing station
    or when the frother position may have changed.
    
    Returns:
        bool: True if frother position calibrated successfully, False otherwise
        
    Example:
        success = get_frother_position()
        if success:
            print("Frother position calibrated successfully")
    """
    try:
        print("🎯 Starting milk frother position calibration...")
        
        # Step 1: Move to home position for setup
        print("🏠 Moving to north-east home position...")
        home_result = home(position="north_east")
        if home_result is False:
            print("[ERROR] Failed to move to north-east home position")
            return False
        
        # Step 2: Open gripper to prepare for positioning
        print("🤏 Opening gripper for positioning...")
        gripper_result = run_skill("set_gripper_position", 255, 0)
        if gripper_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        # Step 3: Perform multiple approaches for accuracy
        print("🎯 Performing calibration approaches (5 attempts)...")
        for i in range(5):
            print(f"   Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            approach_result = run_skill("move_to", "left_steam_wand", 0.15, -10, -10)
            if approach_result is False:
                print(f"[ERROR] Failed calibration approach {i+1}/5")
                return False
        
        # Step 4: Record the calibrated position
        print("💾 Recording milk frother position...")
        record_result = run_skill("get_machine_position", "left_steam_wand")
        if record_result is False:
            print("[ERROR] Failed to record milk frother position")
            return False
        
        print("✅ Milk frother position calibration completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother position calibration: {e}")
        return False

def pick_frother(**params):
    """
    Pick up the milk frother for milk frothing operations.
    
    This function handles the milk frother pickup sequence:
    1. Moves to frother approach position
    2. Approaches the milk frother with precise positioning
    3. Grabs the frother with appropriate grip strength
    4. Stores position data for later return operations
    
    Returns:
        bool: True if frother picked successfully, False otherwise
        
    Example:
        success = pick_frother()
        if success:
            print("Milk frother picked successfully")
    """
    try:
        global approach_angles, grab_angles
        
        print("🥛 Starting milk frother pickup sequence...")
        
        # Step 1: Move to frother area position
        print("📍 Moving to frother area...")
        area_result = run_skill("gotoJ_deg", 20.847986, -21.981329, -113.153931, -76.829208, -81.786911, -0.050592)
        if area_result is False:
            print("[ERROR] Failed to move to frother area")
            return False
        
        # Step 2: Approach the milk frother
        print("🎯 Approaching milk frother...")
        approach_result = run_skill("move_to", 'milk_frother_1', 0.175)
        if approach_result is False:
            print("[ERROR] Failed to approach milk frother")
            return False
        
        # Step 3: Move to approach position for frother
        print("📍 Moving to frother approach position...")
        approach_tool_result = run_skill("approach_tool", 'milk_frother_1', 170)
        if approach_tool_result is False:
            print("[ERROR] Failed to move to frother approach position")
            return False
        
        # Step 4: Record current approach position
        print("💾 Recording approach position...")
        current_angles = run_skill("current_angles")
        if current_angles is not None:
            approach_angles = current_angles
            print(f"   Approach angles recorded: {approach_angles}")
        else:
            print("[WARNING] Failed to record approach angles - continuing without position memory")
            approach_angles = None
        
        # Step 5: Grab the frother
        print("🤏 Grabbing milk frother...")
        grab_result = run_skill("grab_tool", 'milk_frother_1', 200, 250, 255)
        if grab_result is False:
            print("[ERROR] Failed to grab milk frother")
            return False
        
        # Step 6: Record current grab position
        print("💾 Recording grab position...")
        grab_angles = run_skill("current_angles")
        if grab_angles is not None:
            print(f"   Grab angles recorded: {grab_angles}")
        else:
            print("[WARNING] Failed to record grab angles - continuing without position memory")
            grab_angles = None
        
        print("✅ Milk frother pickup completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother pickup: {e}")
        return False
    
def froth_milk(**params):
    """
    Perform milk frothing operation using the steam wand.
    
    This function executes the milk frothing process:
    1. Moves to milk frothing preparation position
    2. Approaches the steam wand with the frother
    3. Mounts frother to steam wand for proper positioning
    4. Activates steam via digital output for frothing
    5. Allows frothing time then deactivates steam
    
    Returns:
        bool: True if milk frothing completed successfully, False otherwise
        
    Example:
        success = froth_milk()
        if success:
            print("Milk frothed successfully")
    """
    try:
        global froth_approach_angles, froth_mount_angles
        
        print("☁️ Starting milk frothing sequence...")
        
        # Step 1: Move to frothing preparation position
        print("📍 Moving to frothing preparation position...")
        prep_result = run_skill("gotoJ_deg", -4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837, 1.0, 0.2)
        if prep_result is False:
            print("[ERROR] Failed to move to frothing preparation position")
            return False
        
        time.sleep(0.2)  # Allow settling time
        
        # Step 2: Set slower servo timing for precise movements
        print("⚙️ Setting precise servo timing...")
        timing_result = run_skill("set_servo_timing", 0.2)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        
        # Step 3: Approach steam wand
        print("🎯 Approaching steam wand...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "milk_frother")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        
        # Record approach position
        froth_approach_result = run_skill("current_angles")
        if froth_approach_result is not None:
            froth_approach_angles = froth_approach_result
        
        time.sleep(0.2)
        
        # Step 4: Mount to steam wand for frothing
        print("🔧 Mounting to steam wand...")
        mount_result = run_skill("mount_machine", "left_steam_wand", "milk_frother")
        if mount_result is False:
            print("[ERROR] Failed to mount to steam wand")
            return False
        
        # Record mount position
        froth_mount_result = run_skill("current_angles")
        if froth_mount_result is not None:
            froth_mount_angles = froth_mount_result
        
        # Step 5: Activate steam for frothing
        print("💨 Activating steam for milk frothing...")
        steam_on_result = run_skill("set_DO", 2, 1)
        if steam_on_result is False:
            print("[ERROR] Failed to activate steam")
            return False
        
        # Step 6: Allow frothing time
        print("☁️ Frothing milk (10 seconds)...")
        time.sleep(10)
        
        # Step 7: Deactivate steam
        print("💨 Deactivating steam...")
        steam_off_result = run_skill("set_DO", 2, 0)
        if steam_off_result is False:
            print("[ERROR] Failed to deactivate steam")
            return False
        
        # Step 8: Allow settling time
        time.sleep(2)
        
        print("✅ Milk frothing completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk frothing: {e}")
        return False

def pour_milk(**params):
    """
    Pour frothed milk into cup at specified stage.
    
    This function pours the frothed milk with stage-specific positioning:
    1. Returns to approach position from steam wand
    2. Adjusts servo timing for smooth pouring
    3. Moves to target stage for milk pouring
    4. Tilts frother for controlled milk pour
    5. Returns frother to upright position
    
    Args:
        stage (str): Target stage for pouring ('1' or '2')
        
    Returns:
        bool: True if milk pouring completed successfully, False otherwise
        
    Example:
        success = pour_milk(stage='1')
        if success:
            print("Milk poured successfully")
    """
    try:
        stage = params.get("stage")
        
        # Validate stage parameter
        if stage not in ('1', '2'):
            print(f"[ERROR] unknown stage: {stage!r}, must be '1' or '2'")
            return False
        
        print(f"🥛 Starting milk pouring sequence for stage {stage}")
        
        # Step 1: Return to approach position from steam wand
        print("⬅️ Moving back from steam wand...")
        back_result = run_skill("approach_machine", "left_steam_wand", "milk_frother")
        if back_result is False:
            print("[ERROR] Failed to move back from steam wand")
            return False
        
        # Step 2: Set normal servo timing
        print("⚙️ Setting normal servo timing...")
        timing_result = run_skill("set_servo_timing", 0.1)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        
        time.sleep(0.2)
        
        # Step 3: Move to intermediate pouring position
        print("📍 Moving to intermediate pouring position...")
        intermediate_result = run_skill("gotoJ_deg", -53.498047, -56.063831, -104.329971, -23.914228, -67.359390, 3.238193, 1.0, 0.2)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        time.sleep(0.2)
        
        # Step 4: Stage-specific pouring sequence
        if stage == '1':
            print("🎯 Executing stage 1 milk pouring...")
            
            # Move to stage 1 position
            stage1_pos_result = run_skill("gotoJ_deg", -117.542499, -27.877248, -91.553736, -69.510481, -86.519990, 1.649929, 1.0, 0.2)
            if stage1_pos_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            
            time.sleep(0.2)
            
            # Tilt for pouring
            pour_result = run_skill("gotoJ_deg", -102.517232, -32.497384, -90.464555, -66.071945, -85.480721, -96.398044, 1.0, 0.075)
            if pour_result is False:
                print("[ERROR] Failed to tilt for pouring")
                return False
            
            # Allow pouring time
            print("🥛 Pouring milk (2 seconds)...")
            time.sleep(2)
            
            # Return to upright
            upright_result = run_skill("gotoJ_deg", -117.542499, -27.877248, -91.553736, -69.510481, -86.519990, 1.649929, 1.0, 0.2)
            if upright_result is False:
                print("[ERROR] Failed to return to upright position")
                return False
            
        elif stage == '2':
            print("🎯 Executing stage 2 milk pouring...")
            
            # Move to stage 2 position
            stage2_pos_result = run_skill("gotoJ_deg", -127.319954, -37.857658, -74.250511, -76.871681, -96.185387, 0.116908, 1.0, 0.2)
            if stage2_pos_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            
            time.sleep(0.2)
            
            # Tilt for pouring
            pour_result = run_skill("gotoJ_deg", -113.384514, -39.535606, -77.602524, -71.924614, -96.217064, -98.112167, 1.0, 0.075)
            if pour_result is False:
                print("[ERROR] Failed to tilt for pouring")
                return False
            
            # Allow pouring time
            print("🥛 Pouring milk (2 seconds)...")
            time.sleep(2)
            
            # Return to upright
            upright_result = run_skill("gotoJ_deg", -127.319954, -37.857658, -74.250511, -76.871681, -96.185387, 0.116908, 1.0, 0.2)
            if upright_result is False:
                print("[ERROR] Failed to return to upright position")
                return False
        
        time.sleep(0.2)
        
        print(f"✅ Milk pouring completed successfully for stage {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk pouring: {e}")
        return False
    
def return_frother(**params):
    """
    Return the milk frother to its storage position after use.
    
    This function returns the frother using stored position data:
    1. Moves to intermediate return position
    2. Uses stored grab position if available
    3. Opens gripper to release frother
    4. Uses stored approach position for safe withdrawal
    5. Returns to home position
    
    Returns:
        bool: True if frother returned successfully, False otherwise
        
    Example:
        success = return_frother()
        if success:
            print("Milk frother returned successfully")
    """
    try:
        global approach_angles, grab_angles
        
        print("🔄 Starting milk frother return sequence...")
        
        # Step 1: Move to intermediate return position
        print("📍 Moving to intermediate return position...")
        intermediate_result = run_skill("gotoJ_deg", -4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate return position")
            return False
        
        time.sleep(0.2)
        
        # Step 2: Move to return preparation position
        print("📍 Moving to return preparation position...")
        prep_result = run_skill("gotoJ_deg", 22.345373, -76.252151, -61.342220, -40.423759, -81.360077, 11.115391)
        if prep_result is False:
            print("[ERROR] Failed to move to return preparation position")
            return False
        
        # Step 3: Use stored grab position if available
        if grab_angles is not None and len(grab_angles) >= 6:
            print("📍 Moving to stored grab position...")
            grab_pos_result = run_skill("gotoJ_deg", *grab_angles)
            if grab_pos_result is False:
                print("[ERROR] Failed to move to stored grab position")
                return False
        else:
            print("[WARNING] No stored grab position available - using default positioning")
        
        # Step 4: Open gripper to release frother
        print("🤏 Opening gripper to release frother...")
        release_result = run_skill("set_gripper_position", 255, 165)
        if release_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        # Step 5: Use stored approach position if available
        if approach_angles is not None and len(approach_angles) >= 6:
            print("⬅️ Moving to stored approach position...")
            approach_pos_result = run_skill("gotoJ_deg", *approach_angles)
            if approach_pos_result is False:
                print("[ERROR] Failed to move to stored approach position")
                return False
        else:
            print("[WARNING] No stored approach position available - using default positioning")
        
        # Step 6: Return to home position
        print("🏠 Returning to home position...")
        home_result = home(position="north")
        if home_result is False:
            print("[ERROR] Failed to return to home position")
            return False
        
        # Step 7: Final gripper opening
        print("🤏 Final gripper opening...")
        final_grip_result = run_skill("set_gripper_position", 255, 0)
        if final_grip_result is False:
            print("[WARNING] Failed final gripper opening - frother may still be released")
        
        print("✅ Milk frother return completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother return: {e}")
        return False
        
# Register for CLI discovery
SEQUENCES = {
    'get_frother_position': get_frother_position,
    'pick_frother': pick_frother,
    'froth_milk': froth_milk,
    'pour_milk': pour_milk,
    'return_frother': return_frother,
}
