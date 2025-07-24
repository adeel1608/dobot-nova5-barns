'''
milk_frother.py
'''
import time
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home

# Global variables to store robot positions during milk frothing operations
# These are used to remember positions between function calls
approach_angles = None
grab_angles = None

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
        run_skill("set_speed_factor", 100)
        # Step 1: Move to home position for setup
        print("🏠 Moving to north-east home position...")
        home(position="north_east")
        home_result = run_skill("gotoJ_deg", -67.357964,-23.709629,-89.522377,-84.038696,-113.021690,8.468687)
        if home_result is False:
            print("[ERROR] Failed to move to north-east home position")
            return False
        
        # Step 2: Open gripper to prepare for positioning
        print("🤏 Opening gripper for positioning...")
        gripper_result = run_skill("set_gripper_position", 255, 0)
        if gripper_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        run_skill("move_to", "left_steam_wand", 0.28)
        run_skill("sync")
        run_skill("grab_tool", "left_steam_wand")
        run_skill("sync")
        run_skill("set_gripper_position", 255, 200)
        run_skill("sync")
        run_skill("gotoJ_deg", -42.886719,-74.454857,-15.463737,-96.987885,-87.116272,7.834893)
        run_skill("sync")
        run_skill("set_gripper_position", 255, 0)
        
        # Step 3: Perform multiple approaches for accuracy
        print("🎯 Performing calibration approaches (5 attempts)...")
        for i in range(5):
            print(f"   Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            approach_result = run_skill("move_to", "left_steam_wand", 0.29)
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
        approach_result = run_skill("move_to", 'milk_frother_2', 0.29)
        if not approach_result:
            print("[WARNING] Failed to approach milk_frother_2, trying milk_frother_1...")
            approach_result = run_skill("move_to", 'milk_frother_1', 0.29)
            if not approach_result:
                print("[ERROR] Failed to approach both milk_frother_2 and milk_frother_1")
                return False
        
        # Step 3: Move to approach position for frother
        print("📍 Moving to frother approach position...")
        run_skill("sync")
        approach_tool_result = run_skill("approach_tool", 'milk_frother_2')
        if approach_tool_result is False:
            print("[ERROR] Failed to move to frother approach position")
            return False
        
        run_skill("set_gripper_position",255,169)
        run_skill("sync")
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
        run_skill("sync")
        grab_result = run_skill("grab_tool", 'milk_frother_2',100,100,-6,-5)
        if grab_result is False:
            print("[ERROR] Failed to grab milk frother")
            return False
        run_skill("sync")
        run_skill("set_gripper_position",255,255)
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
    
def mount_frother(**params):
    """
    Mount the milk frother to the steam wand for frothing preparation.
    
    This function positions the frother on the steam wand:
    1. Moves to milk frothing preparation position
    2. Sets precise servo timing for accurate movements
    3. Approaches the steam wand with the frother (deep position)
    4. Fine approaches the steam wand (light position)
    5. Mounts frother securely to steam wand
    
    Note: This function only positions the frother. Use froth_milk() to actually activate steam.
    
    Returns:
        bool: True if frother mounted successfully, False otherwise
        
    Example:
        success = mount_frother()
        if success:
            print("Frother mounted to steam wand successfully")
    """
    try:
        
        print("☁️ Starting milk frothing sequence...")

        # Step 2: Set slower servo timing for precise movements
        print("⚙️ Setting precise servo timing...")
        timing_result = run_skill("set_speed_factor", 50)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        
        # Step 1: Move to frothing preparation position
        print("📍 Moving to frothing preparation position...")
        prep_result = run_skill("gotoJ_deg", -4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837)
        if prep_result is False:
            print("[ERROR] Failed to move to frothing preparation position")
            return False
        
        # Step 3: Approach steam wand (deep position)
        print("🎯 Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False

        # Step 5: Mount frother to steam wand
        print("🔧 Mounting frother to steam wand...")
        mount_result = run_skill("mount_machine", "left_steam_wand", "deep_froth")
        if mount_result is False:
            print("[ERROR] Failed to mount frother to steam wand")
            return False
        
        print("✅ Milk frother mounted to steam wand successfully")
        return True
               
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk frothing: {e}")
        return False

def froth_milk(**params):
    try:
        duration = params.get("duration", 7.5)
        print(f"🥛 Frothing milk for {duration} seconds...")
        run_skill("sync")
        # Step 5: Activate steam for frothing
        print("💨 Activating steam for milk frothing...")
        steam_on_result = run_skill("set_DO", 1, 1)
        if steam_on_result is False:
            print("[ERROR] Failed to activate steam")
            return False
        
        # Step 6: Allow frothing time
        print("☁️ Frothing milk (10 seconds)...")
        time.sleep(duration)
        
        # Step 7: Deactivate steam
        print("💨 Deactivating steam...")
        steam_off_result = run_skill("set_DO", 1, 0)
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


        # Step 3: Approach steam wand (deep position)
        print("🎯 Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        # Step 2: Set normal servo timing
        print("⚙️ Setting normal servo timing...")
        timing_result = run_skill("set_speed_factor", 50)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        
        
        # Step 3: Move to intermediate pouring position
        print("📍 Moving to intermediate pouring position...")
        intermediate_result = run_skill("gotoJ_deg", -53.498047, -56.063831, -104.329971, -23.914228, -67.359390, 3.238193)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        run_skill("sync")
        
        # Step 4: Stage-specific pouring sequence
        if stage == '1':
            print("🎯 Executing stage 1 milk pouring...")
            
            run_skill("set_speed_factor", 10) 
            run_skill("gotoJ_deg", -116.497627,-30.926678,-99.204826,-44.998241,-116.716164,3.239676)
            run_skill("sync")
            run_skill("move_circle", 3,
                ( -30.0,  0.0, 0.0, 0.0, 0.0, 0.0),    # point1  offset1
                ( -15.0,-15.0, 0.0, 0.0, 0.0, 0.0),    # point2  offset2
                ["tool=0"])
            run_skill("sync")
            run_skill("gotoJ_deg", -117.388901,-41.489887,-93.944572,-48.045307,-115.074539,-31.141111)
            run_skill("gotoJ_deg", -109.472954,-37.621704,-91.996796,-56.920311,-120.970390,-95.325432)
            run_skill("moveEE", 50, 0, 0, 0, 0, 0)
            run_skill("sync")
            time.sleep(3.0)
            run_skill("sync")
            run_skill("gotoJ_deg", -116.497627,-30.926678,-99.204826,-44.998241,-116.716164,3.239676)
            run_skill("set_speed_factor", 100) 
            
        elif stage == '2':
            print("🎯 Executing stage 2 milk pouring...")
            
            # Move to stage 2 position
            stage2_pos_result = run_skill("gotoJ_deg", -127.319954, -37.857658, -74.250511, -76.871681, -96.185387, 0.116908)
            if stage2_pos_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            
            
            
            # Tilt for pouring
            pour_result = run_skill("gotoJ_deg", -113.384514, -39.535606, -77.602524, -71.924614, -96.217064, -98.112167)
            if pour_result is False:
                print("[ERROR] Failed to tilt for pouring")
                return False
            
            # Allow pouring time
            print("🥛 Pouring milk (2 seconds)...")
            time.sleep(2)
            
            # Return to upright
            upright_result = run_skill("gotoJ_deg", -127.319954, -37.857658, -74.250511, -76.871681, -96.185387, 0.116908)
            if upright_result is False:
                print("[ERROR] Failed to return to upright position")
                return False
        
        
        
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
        
        
        
        # Step 2: Move to return preparation position
        print("📍 Moving to return preparation position...")
        prep_result = run_skill("gotoJ_deg", 22.402670,-79.481049,-59.269863,-39.324520,-81.417374,11.115391)
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
        run_skill("sync")
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

def clean_steam_wand(**params):
    """
    Clean the steam wand by running steam through it.
    
    This function cleans the steam wand after use:
    1. Activates steam for cleaning
    2. Runs steam for specified duration to clear residue
    3. Deactivates steam
    4. Allows settling time
    
    Args:
        duration (int): Duration in seconds to run cleaning steam (default: 10)
        
    Returns:
        bool: True if steam wand cleaned successfully, False otherwise
        
    Example:
        success = clean_steam_wand(duration=15)
        if success:
            print("Steam wand cleaned successfully")
    """
    try:
        duration = params.get("duration", 10)
        print(f"🧽 Cleaning steam wand for {duration} seconds...")
        
        # Step 1: Activate steam for cleaning
        print("💨 Activating steam for cleaning...")
        steam_on_result = run_skill("set_DO", 2, 1)
        if steam_on_result is False:
            print("[ERROR] Failed to activate steam for cleaning")
            return False
        
        # Step 2: Allow cleaning time
        print(f"🧽 Running cleaning steam ({duration} seconds)...")
        time.sleep(duration)
        
        # Step 3: Deactivate steam
        print("💨 Deactivating steam...")
        steam_off_result = run_skill("set_DO", 2, 0)
        if steam_off_result is False:
            print("[ERROR] Failed to deactivate steam")
            return False
        
        # Step 4: Allow settling time
        time.sleep(2)
        
        print("✅ Steam wand cleaning completed successfully")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error during steam wand cleaning: {e}")
        return False
        
# Register for CLI discovery
SEQUENCES = {
    'get_frother_position': get_frother_position,
    'pick_frother': pick_frother,
    'froth_milk': froth_milk,
    'pour_milk': pour_milk,
    'return_frother': return_frother,
    'mount_frother': mount_frother,
    'clean_steam_wand': clean_steam_wand,
}
