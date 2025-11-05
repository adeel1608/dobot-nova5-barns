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
from oms_v1.sequences.plastic_cups import dispense_plastic_cup, place_plastic_cup_station
from oms_v1.params import (
    log_step, log_success, log_error, _extract_cup_position
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
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cup_size (str): Cup size ('16oz' - currently only 16oz supported), defaults to '16oz'
        dispenser (str): Dispenser number ('1' or '2') - optional, will be inferred from premixes if not provided
        premixes (dict): Premix dictionary to infer dispenser if not explicitly provided
        
    Returns:
        bool: True if slush dispensing completed successfully, False otherwise
        
    Example:
        success = get_slush(position={'cup_position': 1.0}, dispenser='1')
        if success:
            print("Slush dispensed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        cup_size = params.get("cup_size", "16oz")  # Default to 16oz
        dispenser = params.get("dispenser")
        
        # If no dispenser is provided, try to infer from premixes or use default
        if not dispenser:
            premixes = params.get("premixes", {})
            if premixes:
                # Map premix types to dispensers
                premix_name = list(premixes.keys())[0] if premixes else ""
                # Default mapping: most premixes go to dispenser 1
                # You can extend this mapping as needed
                if "chocolate" in premix_name.lower() or "choco" in premix_name.lower():
                    dispenser = "2"
                else:
                    dispenser = "1"
                print(f"[INFO] No dispenser specified, inferred dispenser '{dispenser}' from premix '{premix_name}'")
            else:
                # Default to dispenser 1 if no premix info
                dispenser = "1"
                print(f"[INFO] No dispenser specified, defaulting to dispenser '1'")
        
        # Validate parameters
        valid_cup_sizes = ("16oz",)  # Currently only 16oz supported
        valid_dispensers = ("1", "2")
            
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
        if not dispense_plastic_cup(**params):
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
        pos2_result = run_skill("gotoJ_deg", 33.380177,-65.448544,-125.305906,18.179613,-139.723057,1.841451)
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
            pos3_result = run_skill("gotoJ_deg", 17.117330,-72.397126,-55.180654,-48.179137,-153.839501,1.256867)
            if not pos3_result:
                print("[ERROR] Failed to move to dispenser 2 intermediate position")
                return False
            dispenser_result = run_skill("gotoJ_deg", 39.080325,-80.227702,-48.030111,-51.277060,-130.866959,-0.148558)
        
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
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cup_size (str): Cup size ('16oz' - currently only 16oz supported), defaults to '16oz'
        dispenser (str): Dispenser number used ('1' or '2') - optional, will be inferred from premixes if not provided
        premixes (dict): Premix dictionary to infer dispenser if not explicitly provided
        
    Returns:
        bool: True if slush placement completed successfully, False otherwise
        
    Example:
        success = place_slush(position={'cup_position': 2.0}, dispenser='1')
        if success:
            print("Slush cup placed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        cup_size = params.get("cup_size", "16oz")  # Default to 16oz
        dispenser = params.get("dispenser")
        
        # If no dispenser is provided, try to infer from premixes or use default
        if not dispenser:
            premixes = params.get("premixes", {})
            if premixes:
                # Map premix types to dispensers (same logic as get_slush)
                premix_name = list(premixes.keys())[0] if premixes else ""
                if "chocolate" in premix_name.lower() or "choco" in premix_name.lower():
                    dispenser = "2"
                else:
                    dispenser = "1"
                print(f"[INFO] No dispenser specified, inferred dispenser '{dispenser}' from premix '{premix_name}'")
            else:
                # Default to dispenser 1 if no premix info
                dispenser = "1"
                print(f"[INFO] No dispenser specified, defaulting to dispenser '1'")
        
        # Validate parameters
        valid_cup_sizes = ("16oz",)  # Currently only 16oz supported
        valid_dispensers = ("1", "2")
            
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
            retreat_result = run_skill("gotoJ_deg", 22.607694,-78.770437,-48.740179,-48.922226,-148.368363,0.479901)
        
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
        if not place_plastic_cup_station(**params):
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
