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
from oms_v1.sequences.plastic_cups import dispense_plastic_cup, place_plastic_cup
from oms_v1.params import (
    SLUSH_PARAMS, VALID_STAGES, VALID_CUP_SIZES, VALID_DISPENSERS,
    validate_stage, validate_cup_size, log_step, log_success, log_error, log_info,
    SPEED_NORMAL
)


def get_slush(**params) -> bool:
    """
    Get slush from specified dispenser and prepare for serving.
    
    This function handles slush dispensing for different stages, cup sizes, and dispensers:
    1. Grabs plastic cup of specified size
    2. Moves to intermediate positioning
    3. Navigates to appropriate slush dispenser
    4. Positions cup under dispenser for slush dispensing
    
    Args:
        stage (str): Target stage ('1', '2', '3', or '4'), defaults to '1'
        cup_size (str): Cup size ('16oz' - currently only 16oz supported), defaults to '16oz'
        dispenser (str): Dispenser number ('1' or '2') - required parameter
        
    Returns:
        bool: True if slush dispensing completed successfully, False otherwise
        
    Example:
        success = get_slush(dispenser='1')  # Uses defaults for stage and cup_size
        if success:
            print("Slush dispensed successfully")
    """
    try:
        # Extract and validate parameters with defaults
        stage = params.get("stage", "1")  # Default to stage 1
        cup_size = params.get("cup_size", "16oz")  # Default to 16oz
        dispenser = params.get("dispenser")
        
        if not dispenser:
            print("[ERROR] Missing required parameter: dispenser")
            return False
        
        # Validate parameters
        valid_stages = ("1", "2", "3", "4")
        valid_cup_sizes = ("16oz",)  # Currently only 16oz supported
        valid_dispensers = ("1", "2")
        
        if stage not in valid_stages:
            print(f"[ERROR] Invalid stage: {stage!r}")
            print(f"[INFO] Valid stages: {', '.join(valid_stages)}")
            return False
            
        if cup_size not in valid_cup_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid cup sizes: {', '.join(valid_cup_sizes)}")
            return False
            
        if dispenser not in valid_dispensers:
            print(f"[ERROR] Invalid dispenser: {dispenser!r}")
            print(f"[INFO] Valid dispensers: {', '.join(valid_dispensers)}")
            return False
        
        print(f"🥤 Starting slush dispensing: Stage {stage}, {cup_size}, Dispenser {dispenser}")
        print("=" * 50)
        
        # Step 1: Grab plastic cup
        log_step(1, 4, f"Grabbing {cup_size} plastic cup")
        if not dispense_plastic_cup(cup_size=cup_size):
            log_error(f"Failed to grab {cup_size} plastic cup")
            return False
        log_success("Cup grabbed successfully", indent=1)
        
        # Step 2: Move to intermediate positioning
        print("📍 Step 2/4: Moving to intermediate positioning...")
        pos1_result = run_skill("gotoJ_deg", 106.212090, -43.618443, -136.693954, 1.223362, -23.919476, -0.124173)
        if not pos1_result:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        # Step 3: Move to slush area
        print("🧊 Step 3/4: Moving to slush dispensing area...")
        pos2_result = run_skill("gotoJ_deg", 45.785095, -64.636208, -119.745956, 10.442498, -127.393181, -0.156864)
        if not pos2_result:
            print("[ERROR] Failed to move to slush area")
            return False
        print("   ✅ Successfully positioned in slush area")
        
        # Step 4: Position at specific dispenser
        print(f"🎯 Step 4/4: Positioning at dispenser {dispenser}...")
        if dispenser == "1":
            print("   📍 Moving to dispenser 1...")
            dispenser_result = run_skill("gotoJ_deg", 53.272518, -67.612831, -88.370926, -23.156694, -119.473190, -0.214796)
        else:  # dispenser == "2"
            print("   📍 Moving to dispenser 2...")
            pos3_result = run_skill("gotoJ_deg", 22.607571, -74.011971, -51.206032, -51.210812, -148.363144, 0.484628)
            if not pos3_result:
                print("[ERROR] Failed to move to dispenser 2 intermediate position")
                return False
            dispenser_result = run_skill("gotoJ_deg", 39.953514, -80.013031, -46.382458, -53.143692, -129.994720, -0.156480)
        
        if not dispenser_result:
            print(f"[ERROR] Failed to position at dispenser {dispenser}")
            return False
        print(f"   ✅ Successfully positioned at dispenser {dispenser}")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ SLUSH DISPENSING COMPLETED SUCCESSFULLY")
        print(f"   ✓ Stage: {stage}, Cup: {cup_size}, Dispenser: {dispenser}")
        print("   ✓ Cup positioned for slush dispensing")
        print("   ✓ Ready for slush dispensing operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during slush dispensing: {e}")
        print("[INFO] Slush dispensing process terminated due to error")
        return False

def place_slush(**params) -> bool:
    """
    Place slush-filled cup at specified staging area after dispensing.
    
    This function handles the placement of slush-filled cups:
    1. Sets appropriate speed for careful handling
    2. Moves away from dispenser safely
    3. Navigates to home position
    4. Places cup at designated staging area
    
    Args:
        stage (str): Target stage ('1', '2', '3', or '4'), defaults to '1'
        cup_size (str): Cup size ('16oz' - currently only 16oz supported), defaults to '16oz'
        dispenser (str): Dispenser number used ('1' or '2') - required parameter
        
    Returns:
        bool: True if slush placement completed successfully, False otherwise
        
    Example:
        success = place_slush(dispenser='1', stage='2')  # Uses default cup_size
        if success:
            print("Slush cup placed successfully")
    """
    try:
        # Extract and validate parameters with defaults
        stage = params.get("stage", "1")  # Default to stage 1
        cup_size = params.get("cup_size", "16oz")  # Default to 16oz
        dispenser = params.get("dispenser")
        
        if not dispenser:
            print("[ERROR] Missing required parameter: dispenser")
            return False
        
        # Validate parameters
        valid_stages = ("1", "2", "3", "4")
        valid_cup_sizes = ("16oz",)  # Currently only 16oz supported
        valid_dispensers = ("1", "2")
        
        if stage not in valid_stages:
            print(f"[ERROR] Invalid stage: {stage!r}")
            print(f"[INFO] Valid stages: {', '.join(valid_stages)}")
            return False
            
        if cup_size not in valid_cup_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid cup sizes: {', '.join(valid_cup_sizes)}")
            return False
            
        if dispenser not in valid_dispensers:
            print(f"[ERROR] Invalid dispenser: {dispenser!r}")
            print(f"[INFO] Valid dispensers: {', '.join(valid_dispensers)}")
            return False
        
        print(f"🧊 Starting slush placement: Stage {stage}, {cup_size}, from Dispenser {dispenser}")
        print("=" * 50)
        
        # Step 1: Set careful handling speed
        print("⚙️ Step 1/4: Setting careful handling speed...")
        speed_result = run_skill("set_speed_factor", 50)
        if not speed_result:
            print("[WARNING] Failed to set speed factor - continuing with default")
        else:
            print("   ✅ Speed factor set for careful handling")
        
        # Step 2: Move away from dispenser safely
        print(f"⬅️ Step 2/4: Moving away from dispenser {dispenser}...")
        if dispenser == "1":
            print("   📍 Moving away from dispenser 1...")
            retreat_result = run_skill("gotoJ_deg", 45.785095, -64.636208, -119.745956, 10.442498, -127.393181, -0.156864)
        else:  # dispenser == "2"
            print("   📍 Moving away from dispenser 2...")
            retreat_result = run_skill("gotoJ_deg", 22.607571, -74.011971, -51.206032, -51.210812, -148.363144, 0.484628)
        
        if not retreat_result:
            print(f"[ERROR] Failed to move away from dispenser {dispenser}")
            return False
        print(f"   ✅ Successfully moved away from dispenser {dispenser}")
        
        # Step 3: Navigate to home position
        print("🏠 Step 3/4: Navigating to home position...")
        if not home(position="north"):
            print("[ERROR] Failed to move to north home position")
            return False
        print("   ✅ Successfully moved to home position")
        
        # Step 4: Place slush cup at designated stage
        print(f"📍 Step 4/4: Placing slush cup at stage {stage}...")
        if not place_plastic_cup(stage=stage):
            print(f"[ERROR] Failed to place slush cup at stage {stage}")
            return False
        print(f"   ✅ Successfully placed slush cup at stage {stage}")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ SLUSH PLACEMENT COMPLETED SUCCESSFULLY")
        print(f"   ✓ Stage: {stage}, Cup: {cup_size}, from Dispenser: {dispenser}")
        print("   ✓ Slush cup safely transported and placed")
        print("   ✓ Ready for customer service")
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
