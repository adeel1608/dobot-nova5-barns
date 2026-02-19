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
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size
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
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_slush_cup_size(cups_dict)
    
    dispenser = params.get("dispenser")
    
    if not dispenser:
        premixes = params.get("premixes", {})
        if premixes:
            premix_name = list(premixes.keys())[0] if premixes else ""
            if "chocolate" in premix_name.lower() or "choco" in premix_name.lower():
                dispenser = "2"
            else:
                dispenser = "1"
        else:
            dispenser = "1"
    
    valid_cup_sizes = ("7oz", "9oz", "12oz", "16oz")
    valid_dispensers = ("1", "2")
    
    if cup_size not in valid_cup_sizes or dispenser not in valid_dispensers:
        return False
    
    cup_code = f"cup_C{cup_size.replace('oz', '')}"
    if not dispense_plastic_cup(cups={cup_code: 1.0}):
        return False
    
    if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['intermediate'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])):
        return False
    
    if dispenser == "2":
        dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['dispense'])
        if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
            return False
    else:
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['intermediate'])):
            return False
        dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['dispense'])
        if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
            return False
    
    if not ok(dispenser_result):
        return False
    
    return True

def place_slush(**params) -> bool:
    """
    Place slush-filled cup at specified staging area after dispensing.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_slush_cup_size(cups_dict)
    
    dispenser = params.get("dispenser")
    
    if not dispenser:
        premixes = params.get("premixes", {})
        if premixes:
            premix_name = list(premixes.keys())[0] if premixes else ""
            if "chocolate" in premix_name.lower() or "choco" in premix_name.lower():
                dispenser = "2"
            else:
                dispenser = "1"
        else:
            dispenser = "1"
    
    valid_cup_sizes = ("7oz", "9oz", "12oz", "16oz")
    valid_dispensers = ("1", "2")
    
    if cup_size not in valid_cup_sizes or dispenser not in valid_dispensers:
        return False
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    gripper_positions = {
        "7oz": 140,
        "9oz": 140,
        "12oz": 140,
        "16oz": 130,
    }
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, gripper_positions[cup_size])):
        return False
    
    if dispenser == "2":
        retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['retreat'])
    else:
        retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['retreat'])
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['intermediate'])):
            return False
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])):
            return False
    
    if not ok(retreat_result):
        return False
    
    if not home(position="north"):
        return False
    
    cup_code = f"cup_C{cup_size.replace('oz', '')}"
    if not place_plastic_cup_station(position={'cup_position': int(stage)}, cups={cup_code: 1.0}):
        return False
    
    return True 
# Register functions for CLI discovery and external access
SEQUENCES = {
    'get_slush': get_slush,
    'place_slush': place_slush,
}
