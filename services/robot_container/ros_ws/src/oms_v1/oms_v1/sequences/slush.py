import time
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home

def get_slush(**params):
    """
    Dispense slush from one of two available dispensers.
    
    This function controls the robot to operate slush dispensers for cold drinks:
    1. Moves to approach position for selected dispenser
    2. Positions for dispensing operation
    3. Activates dispenser through positioning sequence
    4. Returns to safe position after dispensing
    
    The robot supports two different slush dispensers with distinct positioning:
    - Dispenser 1: Standard positioning sequence
    - Dispenser 2: Extended reach positioning sequence
    
    Args:
        dispenser (str): Dispenser selection ('1' or '2')
        
    Returns:
        bool: True if slush dispensing completed successfully, False otherwise
        
    Example:
        success = get_slush(dispenser='1')
        if success:
            print("Slush dispensed successfully from dispenser 1")
    """
    try:
        dispenser = params.get("dispenser")
        
        # Validate dispenser parameter
        if dispenser not in ('1', '2'):
            print(f"[ERROR] Unknown dispenser: {dispenser!r}, must be '1' or '2'")
            return False
        
        print(f"🧊 Starting slush dispensing sequence for dispenser {dispenser}")
        
        if dispenser == '1':
            print("📍 Using dispenser 1 positioning sequence...")
            
            # Step 1: Move to approach position for dispenser 1
            print("🎯 Moving to dispenser 1 approach position...")
            approach_result = run_skill("gotoJ_deg", 27.351225, -41.244622, -131.515709, -6.958192, -152.611468, 0.0)
            if approach_result is False:
                print("[ERROR] Failed to move to dispenser 1 approach position")
                return False
            
            # Step 2: Move to dispensing position
            print("⬇️ Moving to dispensing position...")
            dispense_result = run_skill("gotoJ_deg", 63.173412, -59.329338, -104.639252, -17.729839, -112.244003, 0.0, 1.0, 0.2)
            if dispense_result is False:
                print("[ERROR] Failed to move to dispensing position")
                return False
            
        elif dispenser == '2':
            print("📍 Using dispenser 2 positioning sequence...")
            
            # Step 1: Move to approach position for dispenser 2
            print("🎯 Moving to dispenser 2 approach position...")
            approach_result = run_skill("gotoJ_deg", 27.351225, -41.244622, -131.515709, -6.958192, -152.611468, 0.0)
            if approach_result is False:
                print("[ERROR] Failed to move to dispenser 2 approach position")
                return False
            
            # Step 2: Move to intermediate position
            print("📍 Moving to intermediate position...")
            intermediate_result = run_skill("gotoJ_deg", 16.886827, -53.178346, -90.576631, -35.737992, -163.024408, 0.0)
            if intermediate_result is False:
                print("[ERROR] Failed to move to intermediate position")
                return False
            
            # Step 3: Move to dispensing position (extended reach)
            print("⬇️ Moving to extended dispensing position...")
            dispense_result = run_skill("gotoJ_deg", 45.045738, -70.610674, -67.497199, -43.970287, -130.385694, -0.683135, 1.0, 0.2)
            if dispense_result is False:
                print("[ERROR] Failed to move to extended dispensing position")
                return False
        
        print(f"✅ Slush dispensing completed successfully from dispenser {dispenser}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during slush dispensing: {e}")
        return False

def place_slush(**params):
    """
    Place slush in the specified staging area.
    
    This function places a previously dispensed slush at a designated staging area:
    1. Moves to target staging position
    2. Lowers slush to placement level
    3. Releases slush with controlled opening
    4. Returns to safe position after placement
    
    Args:
        dispenser (str): Dispenser that was used ('1' or '2')
        stage (str): Target stage for slush placement ('1' or '2')
        
    Returns:
        bool: True if slush placement completed successfully, False otherwise
        
    Example:
        success = place_slush(dispenser='1', stage='1')
        if success:
            print("Slush placed successfully at stage 1")
    """
    try:
        dispenser = params.get("dispenser")
        stage = params.get("stage")
        
        # Validate parameters
        if dispenser not in ('1', '2'):
            print(f"[ERROR] Unknown dispenser: {dispenser!r}, must be '1' or '2'")
            return False
            
        if stage not in ('1', '2'):
            print(f"[ERROR] Unknown stage: {stage!r}, must be '1' or '2'")
            return False
        
        print(f"🧊 Starting slush placement sequence for dispenser {dispenser}, stage {stage}")
        
        # Return to approach position based on dispenser
        if dispenser == '1':
            print("📍 Using dispenser 1 return sequence...")
            print("⬆️ Returning to approach position...")
            return_result = run_skill("gotoJ_deg", 27.351225, -41.244622, -131.515709, -6.958192, -152.611468, 0.0, 1.0, 0.2)
            if return_result is False:
                print("[ERROR] Failed to return to approach position")
                return False
                
        elif dispenser == '2':
            print("📍 Using dispenser 2 return sequence...")
            print("⬆️ Returning to intermediate position...")
            return_intermediate_result = run_skill("gotoJ_deg", 16.886827, -53.178346, -90.576631, -35.737992, -163.024408, 0.0, 1.0, 0.2)
            if return_intermediate_result is False:
                print("[ERROR] Failed to return to intermediate position")
                return False
        
        # Final settling time
        time.sleep(0.2)
        
        # Execute placement based on stage
        print(f"📍 Starting slush placement sequence for stage {stage}")
        
        if stage == '1':
            print("🎯 Executing stage 1 placement...")
            
            # Step 1: Move to stage 1 placement position
            print("📍 Moving to stage 1 position...")
            stage1_result = run_skill("gotoJ_deg", -111.215927, -19.601524, -91.144157, -68.881447, -114.195343, 0.046140, 1.0, 0.2)
            if stage1_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
                
        elif stage == '2':
            print("🎯 Executing stage 2 placement...")
            
            # Step 1: Move to stage 2 placement position
            print("📍 Moving to stage 2 position...")
            stage2_result = run_skill("gotoJ_deg", -121.922080, -29.170114, -76.511387, -73.910323, -124.911454, 0.116947, 1.0, 0.2)
            if stage2_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
        
        # Common placement steps for both stages
        # Step 2: Lower slush to placement level
        print("⬇️ Lowering slush to placement level...")
        run_skill("set_servo_timing", 0.2)
        lower_result = run_skill("moveEE", 0, 0, -315, 0, 0, 0)
        if lower_result is False:
            print("[ERROR] Failed to lower slush to placement level")
            return False
        
        # Step 3: Release slush
        print("🤏 Releasing slush...")
        release_result = run_skill("set_gripper_position", 60, 0)
        if release_result is False:
            print("[ERROR] Failed to release slush")
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
        run_skill("set_servo_timing", 0.1)
        home_result = home(position="east")
        if home_result is False:
            print("[ERROR] Failed to return to east home position")
            return False
        
        print(f"✅ Slush placement completed successfully for stage {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during slush placement: {e}")
        return False

# Register for CLI discovery
SEQUENCES = {
    'get_slush': get_slush,
    'place_slush': place_slush,
}
