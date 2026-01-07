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
    SLUSH_PARAMS, SPEED_NORMAL, DEFAULT_PLASTIC_CUP_SIZE,
    log_step, log_success, log_error, _extract_cup_position, _extract_cups_dict, _normalize_cup_size
)


def _normalize_slush_cup_size(cups_dict: Any) -> str:
    """
    Universal cup size normalizer for slush operations.
    Accepts BOTH H-codes AND C-codes regardless of prefix.
    Extracts the numeric size and returns standardized format.
    
    Args:
        cups_dict: Dictionary containing cup information
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz', '16oz')
        
    Examples:
        cup_H9 → '9oz'
        cup_C9 → '9oz'
        cup_h12 → '12oz'
        cup_c16 → '16oz'
    """
    if not cups_dict:
        return DEFAULT_PLASTIC_CUP_SIZE
    
    # Extract the cup code (case-insensitive)
    if isinstance(cups_dict, dict):
        cup_key = next(iter(cups_dict.keys()), None)
        if cup_key:
            # Convert to uppercase for parsing
            cup_key_str = str(cup_key).upper()
            if 'CUP_' in cup_key_str:
                cup_code = cup_key_str.split('CUP_', 1)[1]
            else:
                cup_code = cup_key_str
            
            # Extract numeric size from code (works with both H and C prefixes)
            # H7, H9, H12, C7, C9, C12, C16 → extract the number
            if cup_code and len(cup_code) >= 2:
                # Remove H or C prefix if present
                if cup_code[0] in ('H', 'C'):
                    size_num = cup_code[1:]
                else:
                    size_num = cup_code
                
                # Validate and return standardized size
                if size_num in ('7', '9', '12', '16'):
                    return f"{size_num}oz"
    
    # If parsing failed, try the standard normalizers
    # Try plastic first
    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '':
        return result
    
    # Try paper
    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result
    
    # Final fallback
    return DEFAULT_PLASTIC_CUP_SIZE


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
        cups (dict): Cup dictionary, e.g., {'cup_C9': 1.0} or {'cup_H9': 1.0}
        dispenser (str): Dispenser number ('1' or '2') - optional, will be inferred from premixes if not provided
        premixes (dict): Premix dictionary to infer dispenser if not explicitly provided
        
    Returns:
        bool: True if slush dispensing completed successfully, False otherwise
        
    Example:
        success = get_slush(position={'cup_position': 1.0}, cups={'cup_C16': 1.0}, dispenser='1')
        if success:
            print("Slush dispensed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        # Extract and validate cup size parameter using flexible slush normalizer
        # This accepts both H-codes (H7, H9, H12) and C-codes (C7, C9, C12, C16)
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_slush_cup_size(cups_dict)
        
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
                    dispenser = "1"
                else:
                    dispenser = "2"
                print(f"[INFO] No dispenser specified, inferred dispenser '{dispenser}' from premix '{premix_name}'")
            else:
                # Default to dispenser 1 if no premix info
                dispenser = "1"
                print(f"[INFO] No dispenser specified, defaulting to dispenser '1'")
        
        # Validate parameters
        valid_cup_sizes = ("7oz", "9oz", "12oz", "16oz")  # All plastic cup sizes supported
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
        # Convert cup_size to proper cups dict format (e.g., "16oz" -> {"cup_C16": 1.0})
        cup_code = f"cup_C{cup_size.replace('oz', '')}"
        if not dispense_plastic_cup(cups={cup_code: 1.0}):
            log_error(f"Failed to grab {cup_size} plastic cup")
            return False
        log_success("Cup grabbed successfully", indent=1)
        
        # Step 2: Move to intermediate positioning
        print("📍 Step 2/4: Moving to intermediate positioning...")
        pos1_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['intermediate'])
        if not pos1_result:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        # Step 3: Move to slush area
        print("🧊 Step 3/4: Moving to slush dispensing area...")
        pos2_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])
        if not pos2_result:
            print("[ERROR] Failed to move to slush area")
            return False
        print("   ✅ Successfully positioned in slush area")
        
        # Step 4: Position at specific dispenser
        print(f"🎯 Step 4/4: Positioning at dispenser {dispenser}...")
        if dispenser == "1":
            print("   📍 Moving to dispenser 1...")
            dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['dispense'])
        else:  # dispenser == "2"
            print("   📍 Moving to dispenser 2...")
            pos3_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['intermediate'])
            if not pos3_result:
                print("[ERROR] Failed to move to dispenser 2 intermediate position")
                return False
            dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['dispense'])
        
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
        cups (dict): Cup dictionary, e.g., {'cup_C9': 1.0} or {'cup_H9': 1.0}
        dispenser (str): Dispenser number used ('1' or '2') - optional, will be inferred from premixes if not provided
        premixes (dict): Premix dictionary to infer dispenser if not explicitly provided
        
    Returns:
        bool: True if slush placement completed successfully, False otherwise
        
    Example:
        success = place_slush(position={'cup_position': 2.0}, cups={'cup_C16': 1.0}, dispenser='1')
        if success:
            print("Slush cup placed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        # Extract and validate cup size parameter using flexible slush normalizer
        # This accepts both H-codes (H7, H9, H12) and C-codes (C7, C9, C12, C16)
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_slush_cup_size(cups_dict)
        
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
        valid_cup_sizes = ("7oz", "9oz", "12oz", "16oz")  # All plastic cup sizes supported
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
        speed_result = run_skill("set_speed_factor", SPEED_NORMAL)
        if not speed_result:
            print("[WARNING] Failed to set speed factor - continuing with default")
        else:
            print("   ✅ Speed factor set for careful handling")
        
        # Step 2: Move away from dispenser safely
        print(f"⬅️ Step 2/4: Moving away from dispenser {dispenser}...")
        if dispenser == "1":
            print("   📍 Moving away from dispenser 1...")
            retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['retreat'])
        else:  # dispenser == "2"
            print("   📍 Moving away from dispenser 2...")
            retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['retreat'])
        
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
        # Convert cup_size to proper cups dict format (e.g., "16oz" -> {"cup_C16": 1.0})
        cup_code = f"cup_C{cup_size.replace('oz', '')}"
        if not place_plastic_cup_station(position={'cup_position': int(stage)}, cups={cup_code: 1.0}):
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
