"""
slush.py

Defines the slush handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for dispensing and placing slush drinks
in the BARNS coffee automation system, supporting multiple dispensers and staging areas
for frozen beverage preparation.
"""

import time
from typing import Dict, Any, Optional
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home
from oms_v1.params import SLUSH_PARAMS


def get_slush(**params) -> bool:
    """
    Dispense slush from one of two available dispensers.
    
    This function controls the robot to operate slush dispensers for cold drinks:
    1. Validates dispenser parameter
    2. Moves to approach position for selected dispenser
    3. Executes dispenser-specific positioning sequence
    4. Activates dispenser through precise positioning
    5. Maintains position for optimal dispensing
    
    The robot supports two different slush dispensers with distinct positioning:
    - Dispenser 1: Standard positioning sequence with direct approach
    - Dispenser 2: Extended reach positioning sequence with intermediate steps
    
    Args:
        dispenser (str): Dispenser selection ('1' or '2')
        
    Returns:
        bool: True if slush dispensing completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during slush dispensing process
        
    Example:
        success = get_slush(dispenser='1')
        if success:
            print("Slush dispensed successfully from dispenser 1")
    """
    try:
        # Extract and validate dispenser parameter
        dispenser = params.get("dispenser")
        if not dispenser:
            print("[ERROR] No dispenser parameter provided")
            return False
        
        # Validate dispenser parameter
        valid_dispensers = ('1', '2')
        if dispenser not in valid_dispensers:
            print(f"[ERROR] Unknown dispenser: {dispenser!r}")
            print(f"[INFO] Valid dispensers: {', '.join(valid_dispensers)}")
            return False
        
        print(f"🧊 Starting slush dispensing sequence for dispenser {dispenser}")
        print("=" * 50)
        
        if dispenser == '1':
            print("📍 Using dispenser 1 positioning sequence...")
            
            # Step 1: Move to approach position for dispenser 1
            print("🎯 Step 1/2: Moving to dispenser 1 approach position...")
            approach_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['approach'])
            if approach_result is False:
                print("[ERROR] Failed to move to dispenser 1 approach position")
                return False
            print("   ✅ Successfully approached dispenser 1")
            
            # Step 2: Move to dispensing position
            print("⬇️ Step 2/2: Moving to dispensing position...")
            print("   📍 Positioning for optimal slush flow...")
            dispense_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['dispense'])
            if dispense_result is False:
                print("[ERROR] Failed to move to dispensing position")
                return False
            print("   ✅ Successfully positioned for dispensing")
            
        elif dispenser == '2':
            print("📍 Using dispenser 2 positioning sequence...")
            
            # Step 1: Move to approach position for dispenser 2
            print("🎯 Step 1/3: Moving to dispenser 2 approach position...")
            approach_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['approach'])
            if approach_result is False:
                print("[ERROR] Failed to move to dispenser 2 approach position")
                return False
            print("   ✅ Successfully approached dispenser 2")
            
            # Step 2: Move to intermediate position
            print("📍 Step 2/3: Moving to intermediate position...")
            print("   📍 Navigating to extended reach position...")
            intermediate_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['intermediate'])
            if intermediate_result is False:
                print("[ERROR] Failed to move to intermediate position")
                return False
            print("   ✅ Successfully reached intermediate position")
            
            # Step 3: Move to dispensing position (extended reach)
            print("⬇️ Step 3/3: Moving to extended dispensing position...")
            print("   📍 Positioning for optimal slush flow with extended reach...")
            dispense_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['dispense'])
            if dispense_result is False:
                print("[ERROR] Failed to move to extended dispensing position")
                return False
            print("   ✅ Successfully positioned for extended dispensing")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ SLUSH DISPENSING COMPLETED SUCCESSFULLY FROM DISPENSER {dispenser}")
        print("   ✓ Optimal positioning achieved")
        print("   ✓ Slush dispenser activated")
        print("   ✓ Ready for slush collection and placement")
        print("   🧊 Frozen beverage dispensing complete!")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during slush dispensing: {e}")
        print("[INFO] Slush dispensing process terminated due to error")
        return False


def place_slush(**params) -> bool:
    """
    Place slush in the specified staging area.
    
    This function places a previously dispensed slush at a designated staging area:
    1. Validates dispenser and stage parameters
    2. Returns from dispensing position based on dispenser type
    3. Moves to target staging position
    4. Lowers slush to placement level with controlled speed
    5. Releases slush with precise gripper control
    6. Allows settling time for stable placement
    7. Retracts safely and returns to home position
    
    Args:
        dispenser (str): Dispenser that was used ('1' or '2')
        stage (str): Target stage for slush placement ('1' or '2')
        
    Returns:
        bool: True if slush placement completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during slush placement process
        
    Example:
        success = place_slush(dispenser='1', stage='1')
        if success:
            print("Slush placed successfully at stage 1")
    """
    try:
        # Extract and validate parameters
        dispenser = params.get("dispenser")
        stage = params.get("stage")
        
        if not dispenser:
            print("[ERROR] No dispenser parameter provided")
            return False
            
        if not stage:
            print("[ERROR] No stage parameter provided")
            return False
        
        # Validate parameters
        valid_dispensers = ('1', '2')
        valid_stages = ('1', '2')
        
        if dispenser not in valid_dispensers:
            print(f"[ERROR] Unknown dispenser: {dispenser!r}")
            print(f"[INFO] Valid dispensers: {', '.join(valid_dispensers)}")
            return False
            
        if stage not in valid_stages:
            print(f"[ERROR] Unknown stage: {stage!r}")
            print(f"[INFO] Valid stages: {', '.join(valid_stages)}")
            return False
        
        print(f"🧊 Starting slush placement sequence for dispenser {dispenser}, stage {stage}")
        print("=" * 50)
        
        # Step 1: Return from dispensing position based on dispenser
        if dispenser == '1':
            print("📍 Step 1/8: Using dispenser 1 return sequence...")
            print("⬆️ Returning to approach position...")
            return_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['approach'])
            if return_result is False:
                print("[ERROR] Failed to return to approach position")
                return False
            print("   ✅ Successfully returned from dispenser 1")
                
        elif dispenser == '2':
            print("📍 Step 1/8: Using dispenser 2 return sequence...")
            print("⬆️ Returning to intermediate position...")
            return_intermediate_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['intermediate'])
            if return_intermediate_result is False:
                print("[ERROR] Failed to return to intermediate position")
                return False
            print("   ✅ Successfully returned from dispenser 2")
        
        # Step 2: Allow settling time
        print("⏰ Step 2/8: Allowing settling time...")
        time.sleep(0.2)
        print("   ✅ Settling time completed")
        
        # Step 3: Move to staging position based on stage
        print(f"📍 Step 3/8: Moving to stage {stage} position...")
        
        if stage == '1':
            print("🎯 Executing stage 1 positioning...")
            print("   📍 Moving to stage 1 position...")
            stage1_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['staging']['stage_1'])
            if stage1_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            print("   ✅ Successfully positioned at stage 1")
                
        elif stage == '2':
            print("🎯 Executing stage 2 positioning...")
            print("   📍 Moving to stage 2 position...")
            stage2_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['staging']['stage_2'])
            if stage2_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            print("   ✅ Successfully positioned at stage 2")
        
        # Step 4: Set slower speed for careful placement
        print("⚙️ Step 4/8: Setting careful placement speed...")
        run_skill("set_speed_factor", 20)
        print("   ✅ Speed adjusted for careful handling")
        
        # Step 5: Lower slush to placement level
        print("⬇️ Step 5/8: Lowering slush to placement level...")
        print("   📍 Descending 315mm to placement level...")
        lower_result = run_skill("moveEE", 0, 0, -315, 0, 0, 0)
        if lower_result is False:
            print("[ERROR] Failed to lower slush to placement level")
            return False
        print("   ✅ Successfully lowered to placement level")
        
        # Step 6: Release slush
        print("🤏 Step 6/8: Releasing slush...")
        print("   📏 Setting gripper to release position...")
        release_result = run_skill("set_gripper_position", 60, 0)
        if release_result is False:
            print("[ERROR] Failed to release slush")
            return False
        print("   ✅ Slush released successfully")
        
        # Step 7: Allow settling time
        print("⏰ Step 7/8: Allowing settling time...")
        time.sleep(0.5)
        print("   ✅ Settling time completed")
        
        # Step 8: Raise after placement
        print("⬆️ Step 8/8: Moving up after placement...")
        print("   📍 Ascending 315mm to clear slush...")
        raise_result = run_skill("moveEE", 0, 0, 315, 0, 0, 0)
        if raise_result is False:
            print("[ERROR] Failed to move up after placement")
            return False
        print("   ✅ Successfully moved up after placement")
        
        # Step 9: Reset speed and return to home position
        print("🏠 Step 9/8: Returning to east home position...")
        print("   ⚙️ Resetting speed factor...")
        run_skill("set_speed_factor", 10)
        print("   🏠 Moving to east home...")
        home_result = home(position="east")
        if home_result is False:
            print("[ERROR] Failed to return to east home position")
            return False
        print("   ✅ Successfully returned to east home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ SLUSH PLACEMENT COMPLETED SUCCESSFULLY FOR STAGE {stage}")
        print(f"   ✓ Slush from dispenser {dispenser} placed optimally")
        print("   ✓ Stable placement achieved")
        print("   ✓ Speed control utilized for careful handling")
        print("   ✓ Robot returned to home position")
        print("   🧊 Frozen beverage ready for service!")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during slush placement: {e}")
        print("[INFO] Slush placement process terminated due to error")
        return False


# Register functions for CLI discovery and external access
SEQUENCES = {
    'get_slush': get_slush,
    'place_slush': place_slush,
}
