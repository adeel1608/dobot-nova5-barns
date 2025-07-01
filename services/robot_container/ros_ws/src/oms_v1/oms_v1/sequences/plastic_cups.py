import time
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home

def grab_plastic_cup(**params):
    """
    Grab a plastic cup of specified size from the plastic cup dispenser.
    
    This function handles plastic cup pickup for beverages like slushes and iced drinks:
    1. Moves to home position for safe approach
    2. Navigates to plastic cup dispenser area
    3. Positions for plastic cup grab based on size
    4. Grabs plastic cup with appropriate grip strength
    5. Moves plastic cup to safe position ready for beverage preparation
    
    The function supports multiple plastic cup sizes with consistent grabbing sequence:
    - 7oz: Small plastic cups for smaller portions
    - 9oz: Medium plastic cups for standard servings  
    - 12oz: Large plastic cups for generous servings
    - 16oz: Extra large plastic cups for maximum capacity
    
    Args:
        cup_size (str): Size of plastic cup to grab ('7oz', '9oz', '12oz', '16oz')
        
    Returns:
        bool: True if plastic cup grabbed successfully, False otherwise
        
    Example:
        success = grab_plastic_cup(cup_size='12oz')
        if success:
            print("12oz plastic cup grabbed successfully")
    """
    try:
        cup_size = params.get("cup_size")
        
        # Validate cup size parameter
        if cup_size not in ('16oz', '12oz', '9oz', '7oz'):
            print(f"[ERROR] unknown plastic cup size: {cup_size!r}, must be '7oz', '9oz', '12oz', or '16oz'")
            return False
        
        print(f"🥤 Starting plastic cup grab sequence for {cup_size}")
        
        # Step 1: Move to home position for safe approach
        print("🏠 Moving to west home position...")
        home_result = home(position="west")
        if home_result is False:
            print("[ERROR] Failed to move to west home position")
            return False
        
        # Step 2: Open gripper to prepare for plastic cup grab
        print("🤏 Opening gripper for plastic cup grab...")
        gripper_open_result = run_skill("set_gripper_position", 255, 0)
        if gripper_open_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        # Step 3: Move to plastic cup dispenser area
        print("📍 Moving to plastic cup dispenser area...")
        dispenser_result = run_skill("gotoJ_deg", 137.406860, 3.501065, -134.504471, -48.814426, -42.387501, -0.108438)
        if dispenser_result is False:
            print("[ERROR] Failed to move to plastic cup dispenser area")
            return False
        
        # Step 4: Execute plastic cup grabbing sequence (same for all sizes in this setup)
        print(f"🎯 Positioning for {cup_size} plastic cup grab...")
        
        # Move to grab position
        grab_position_result = run_skill("moveEE", 5, 210, 10, 0, 0, 0)
        if grab_position_result is False:
            print("[ERROR] Failed to move to plastic cup grab position")
            return False
        
        # Close gripper to grab plastic cup
        print("🤏 Gripping plastic cup...")
        grip_result = run_skill("set_gripper_position", 255, 140)
        if grip_result is False:
            print("[ERROR] Failed to grip plastic cup")
            return False
        
        # Move down to extract plastic cup from dispenser
        print("⬇️ Extracting plastic cup from dispenser...")
        extract_result = run_skill("moveEE", 0, 0, -205, 0, 0, 0)
        if extract_result is False:
            print("[ERROR] Failed to extract plastic cup from dispenser")
            return False
        
        # Step 5: Return to safe position with plastic cup
        print("📍 Moving to safe position with plastic cup...")
        safe_position_result = run_skill("gotoJ_deg", 137.406860, 3.501065, -134.504471, -48.814426, -42.387501, -0.108438)
        if safe_position_result is False:
            print("[ERROR] Failed to move to safe position with plastic cup")
            return False
        
        print(f"✅ Plastic cup grab completed successfully for {cup_size}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during plastic cup grab: {e}")
        return False
    
def place_plastic_cup(**params):
    """
    Place a plastic cup at specified staging area for cold beverage preparation.
    
    This function places a previously grabbed plastic cup at a designated staging area:
    1. Moves to target staging position
    2. Lowers plastic cup to placement level
    3. Releases plastic cup with controlled opening
    4. Retracts safely after placement
    5. Returns to home position or continues workflow
    
    Args:
        stage (str): Target staging area ('1' or '2')
        
    Returns:
        bool: True if plastic cup placed successfully, False otherwise
        
    Example:
        success = place_plastic_cup(stage='1')
        if success:
            print("Plastic cup placed at stage 1 successfully")
    """
    try:
        stage = params.get("stage")
        
        # Validate staging area parameter
        if stage not in ('1', '2'):
            print(f"[ERROR] unknown cold stage: {stage!r}, must be '1' or '2'")
            return False
        
        print(f"📍 Starting plastic cup placement sequence for stage {stage}")
        
        if stage == '1':
            print("🎯 Executing stage 1 placement...")
            
            # Step 1: Move to stage 1 placement position
            print("📍 Moving to stage 1 position...")
            stage1_result = run_skill("gotoJ_deg", -111.215927, -19.601524, -91.144157, -68.881447, -114.195343, 0.046140)
            if stage1_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            
            # Step 2: Lower plastic cup to placement level
            print("⬇️ Lowering plastic cup to placement level...")
            lower_result = run_skill("moveEE", 0, 0, -315, 0, 0, 0)
            if lower_result is False:
                print("[ERROR] Failed to lower plastic cup to placement level")
                return False
            
            # Step 3: Release plastic cup
            print("🤏 Releasing plastic cup...")
            release_result = run_skill("set_gripper_position", 60, 0)
            if release_result is False:
                print("[ERROR] Failed to release plastic cup")
                return False
            
            # Step 4: Allow settling time
            time.sleep(0.5)
            
            # Step 5: Raise after placement
            print("⬆️ Moving up after placement...")
            raise_result = run_skill("moveEE", 0, 0, 315, 0, 0, 0)
            if raise_result is False:
                print("[ERROR] Failed to move up after placement")
                return False
            
            # Step 6: Return to home position
            print("🏠 Returning to east home position...")
            home_result = home(position="east")
            if home_result is False:
                print("[ERROR] Failed to return to east home position")
                return False
            
        elif stage == '2':
            print("🎯 Executing stage 2 placement...")
            
            # Step 1: Move to stage 2 placement position
            print("📍 Moving to stage 2 position...")
            stage2_result = run_skill("gotoJ_deg", -121.922080, -29.170114, -76.511387, -73.910323, -124.911454, 0.116947)
            if stage2_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            
            # Step 2: Lower plastic cup to placement level
            print("⬇️ Lowering plastic cup to placement level...")
            lower_result = run_skill("moveEE", 0, 0, -315, 0, 0, 0)
            if lower_result is False:
                print("[ERROR] Failed to lower plastic cup to placement level")
                return False
            
            # Step 3: Release plastic cup
            print("🤏 Releasing plastic cup...")
            release_result = run_skill("set_gripper_position", 60, 0)
            if release_result is False:
                print("[ERROR] Failed to release plastic cup")
                return False
            
            # Step 4: Allow settling time
            time.sleep(0.5)
            
            # Step 5: Raise after placement
            print("⬆️ Moving up after placement...")
            raise_result = run_skill("moveEE", 0, 0, 315, 0, 0, 0)
            if raise_result is False:
                print("[ERROR] Failed to move up after placement")
                return False
            
            # Step 6: Return to home position
            print("🏠 Returning to east home position...")
            home_result = home(position="east")
            if home_result is False:
                print("[ERROR] Failed to return to east home position")
                return False
        
        print(f"✅ Plastic cup placement completed successfully for stage {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during plastic cup placement: {e}")
        return False
        
# Register for CLI discovery
SEQUENCES = {
    'grab_plastic_cup': grab_plastic_cup,
    'place_plastic_cup': place_plastic_cup,
}
