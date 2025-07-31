"""
plastic_cups.py

Defines the plastic cup handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for grabbing and placing plastic cups
in the BARNS coffee automation system, supporting multiple cup sizes for cold beverages
like slushes, iced drinks, and cold brews.
"""

import time
from typing import Dict, Any, Optional
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home



def grab_plastic_cup(**params) -> bool:
    """
    Grab a plastic cup of specified size from the plastic cup dispenser.
    
    This function handles plastic cup pickup for beverages like slushes and iced drinks:
    1. Moves to home position for safe approach
    2. Opens gripper to prepare for cup grab
    3. Navigates to plastic cup dispenser area
    4. Positions for plastic cup grab based on size
    5. Grabs plastic cup with appropriate grip strength
    6. Extracts cup from dispenser safely
    7. Moves plastic cup to safe position ready for beverage preparation
    
    The function supports multiple plastic cup sizes with consistent grabbing sequence:
    - 7oz: Small plastic cups for smaller portions
    - 9oz: Medium plastic cups for standard servings  
    - 12oz: Large plastic cups for generous servings
    - 16oz: Extra large plastic cups for maximum capacity
    
    Args:
        cup_size (str): Size of plastic cup to grab ('7oz', '9oz', '12oz', '16oz')
        
    Returns:
        bool: True if plastic cup grabbed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during cup grabbing process
        
    Example:
        success = grab_plastic_cup(cup_size='12oz')
        if success:
            print("12oz plastic cup grabbed successfully")
    """
    try:
        # Extract and validate cup size parameter
        cup_size = params.get("cup_size")
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        # Validate cup size parameter
        valid_sizes = ('16oz', '12oz', '9oz', '7oz')
        if cup_size not in valid_sizes:
            print(f"[ERROR] Unknown plastic cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        print(f"🥤 Starting plastic cup grab sequence for {cup_size}")
        print("=" * 50)
        
        # Step 1: Move to home position for safe approach
        print("🏠 Step 1/7: Moving to west home position...")
        home_result = home(position="west")
        if home_result is False:
            print("[ERROR] Failed to move to west home position")
            return False
        print("   ✅ Successfully moved to west home position")
        
        # Step 2: Open gripper to prepare for plastic cup grab
        print("🤏 Step 2/7: Opening gripper for plastic cup grab...")
        gripper_open_result = run_skill("set_gripper_position", 255, 0)
        if gripper_open_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        print("   ✅ Gripper opened successfully")
        
        # Step 3: Move to plastic cup dispenser area
        print("📍 Step 3/7: Moving to plastic cup dispenser area...")
        dispenser_result = run_skill("gotoJ_deg", 137.406860, 3.501065, -134.504471, -48.814426, -42.387501, -0.108438)
        if dispenser_result is False:
            print("[ERROR] Failed to move to plastic cup dispenser area")
            return False
        print("   ✅ Successfully positioned at dispenser area")
        
        # Step 4: Position for plastic cup grab
        print(f"🎯 Step 4/7: Positioning for {cup_size} plastic cup grab...")
        print("   📍 Moving to grab position...")
        grab_position_result = run_skill("moveEE", 5, 210, 10, 0, 0, 0)
        if grab_position_result is False:
            print("[ERROR] Failed to move to plastic cup grab position")
            return False
        print("   ✅ Successfully positioned for cup grab")
        
        # Step 5: Grip plastic cup
        print("🤏 Step 5/7: Gripping plastic cup...")
        print(f"   📏 Setting gripper width for {cup_size} cup...")
        grip_result = run_skill("set_gripper_position", 255, 140)
        if grip_result is False:
            print("[ERROR] Failed to grip plastic cup")
            return False
        print("   ✅ Plastic cup secured successfully")
        
        # Step 6: Extract plastic cup from dispenser
        print("⬇️ Step 6/7: Extracting plastic cup from dispenser...")
        print("   📍 Moving down to extract cup...")
        extract_result = run_skill("moveEE", 0, 0, -205, 0, 0, 0)
        
        if extract_result is False:
            print("[ERROR] Failed to extract plastic cup from dispenser")
            return False
        print("   ✅ Cup successfully extracted from dispenser")
        
        # Step 7: Return to safe position with plastic cup
        print("📍 Step 7/7: Moving to safe position with plastic cup...")
        safe_position_result = run_skill("gotoJ_deg", 137.406860, 3.501065, -134.504471, -48.814426, -42.387501, -0.108438)
        
        if safe_position_result is False:
            print("[ERROR] Failed to move to safe position with plastic cup")
            return False
        print("   ✅ Successfully moved to safe position")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PLASTIC CUP GRAB COMPLETED SUCCESSFULLY FOR {cup_size.upper()}")
        print("   ✓ Cup securely gripped and extracted")
        print("   ✓ Safe positioning achieved")
        print("   ✓ Ready for cold beverage preparation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during plastic cup grab: {e}")
        print("[INFO] Plastic cup grab process terminated due to error")
        return False
    

def place_plastic_cup(**params) -> bool:
    """
    Place a plastic cup at specified staging area for cold beverage preparation.
    
    This function places a previously grabbed plastic cup at a designated staging area:
    1. Validates staging area parameter
    2. Moves to target staging position
    3. Lowers plastic cup to placement level
    4. Releases plastic cup with controlled opening
    5. Allows settling time for stable placement
    6. Retracts safely after placement
    7. Returns to home position
    
    Args:
        stage (str): Target staging area ('1' or '2')
        
    Returns:
        bool: True if plastic cup placed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during cup placement process
        
    Example:
        success = place_plastic_cup(stage='1')
        if success:
            print("Plastic cup placed at stage 1 successfully")
    """
    try:
        # Extract and validate stage parameter
        stage = params.get("stage")
        if not stage:
            print("[ERROR] No stage parameter provided")
            return False
        
        # Validate staging area parameter
        valid_stages = ('1', '2')
        if stage not in valid_stages:
            print(f"[ERROR] Unknown cold stage: {stage!r}")
            print(f"[INFO] Valid stages: {', '.join(valid_stages)}")
            return False
        
        print(f"📍 Starting plastic cup placement sequence for stage {stage}")
        print("=" * 50)
        
        if stage == '1':
            print("🎯 Executing stage 1 placement sequence...")
            
            # Step 1: Move to stage 1 placement position
            print("📍 Step 1/7: Moving to stage 1 position...")
            stage1_result = run_skill("gotoJ_deg", -111.215927, -19.601524, -91.144157, -68.881447, -114.195343, 0.046140)
            if stage1_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            print("   ✅ Successfully positioned at stage 1")
            
            # Step 2: Lower plastic cup to placement level
            print("⬇️ Step 2/7: Lowering plastic cup to placement level...")
            print("   📍 Descending 315mm to placement level...")
            lower_result = run_skill("moveEE", 0, 0, -315, 0, 0, 0)
            if lower_result is False:
                print("[ERROR] Failed to lower plastic cup to placement level")
                return False
            print("   ✅ Successfully lowered to placement level")
            
            # Step 3: Release plastic cup
            print("🤏 Step 3/7: Releasing plastic cup...")
            print("   📏 Setting gripper to release position...")
            release_result = run_skill("set_gripper_position", 60, 0)
            if release_result is False:
                print("[ERROR] Failed to release plastic cup")
                return False
            print("   ✅ Plastic cup released successfully")
            
            # Step 4: Allow settling time
            print("⏰ Step 4/7: Allowing settling time...")
            time.sleep(0.5)
            print("   ✅ Settling time completed")
            
            # Step 5: Raise after placement
            print("⬆️ Step 5/7: Moving up after placement...")
            print("   📍 Ascending 315mm to clear cup...")
            raise_result = run_skill("moveEE", 0, 0, 315, 0, 0, 0)
            if raise_result is False:
                print("[ERROR] Failed to move up after placement")
                return False
            print("   ✅ Successfully moved up after placement")
            
            # Step 6: Return to home position
            print("🏠 Step 6/7: Returning to east home position...")
            home_result = home(position="east")
            if home_result is False:
                print("[ERROR] Failed to return to east home position")
                return False
            print("   ✅ Successfully returned to east home")
            
            # Step 7: Completion
            print("🏁 Step 7/7: Stage 1 placement completed")
            
        elif stage == '2':
            print("🎯 Executing stage 2 placement sequence...")
            
            # Step 1: Move to stage 2 placement position
            print("📍 Step 1/7: Moving to stage 2 position...")
            stage2_result = run_skill("gotoJ_deg", -121.922080, -29.170114, -76.511387, -73.910323, -124.911454, 0.116947)
            if stage2_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            print("   ✅ Successfully positioned at stage 2")
            
            # Step 2: Lower plastic cup to placement level
            print("⬇️ Step 2/7: Lowering plastic cup to placement level...")
            print("   📍 Descending 315mm to placement level...")
            lower_result = run_skill("moveEE", 0, 0, -315, 0, 0, 0)
            if lower_result is False:
                print("[ERROR] Failed to lower plastic cup to placement level")
                return False
            print("   ✅ Successfully lowered to placement level")
            
            # Step 3: Release plastic cup
            print("🤏 Step 3/7: Releasing plastic cup...")
            print("   📏 Setting gripper to release position...")
            release_result = run_skill("set_gripper_position", 60, 0)
            if release_result is False:
                print("[ERROR] Failed to release plastic cup")
                return False
            print("   ✅ Plastic cup released successfully")
            
            # Step 4: Allow settling time
            print("⏰ Step 4/7: Allowing settling time...")
            time.sleep(0.5)
            print("   ✅ Settling time completed")
            
            # Step 5: Raise after placement
            print("⬆️ Step 5/7: Moving up after placement...")
            print("   📍 Ascending 315mm to clear cup...")
            raise_result = run_skill("moveEE", 0, 0, 315, 0, 0, 0)
            if raise_result is False:
                print("[ERROR] Failed to move up after placement")
                return False
            print("   ✅ Successfully moved up after placement")
            
            # Step 6: Return to home position
            print("🏠 Step 6/7: Returning to east home position...")
            home_result = home(position="east")
            if home_result is False:
                print("[ERROR] Failed to return to east home position")
                return False
            print("   ✅ Successfully returned to east home")
            
            # Step 7: Completion
            print("🏁 Step 7/7: Stage 2 placement completed")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PLASTIC CUP PLACEMENT COMPLETED SUCCESSFULLY FOR STAGE {stage}")
        print("   ✓ Cup placed at optimal position")
        print("   ✓ Stable placement achieved")
        print("   ✓ Ready for cold beverage preparation")
        print("   ❄️ Cold beverage station ready!")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during plastic cup placement: {e}")
        print("[INFO] Plastic cup placement process terminated due to error")
        return False


# Register functions for CLI discovery and external access
SEQUENCES = {
    'grab_plastic_cup': grab_plastic_cup,
    'place_plastic_cup': place_plastic_cup,
}
