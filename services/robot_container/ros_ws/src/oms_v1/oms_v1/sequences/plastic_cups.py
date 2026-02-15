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
from oms_v1.sequences.home import home, return_back_to_home
from oms_v1.params import (
    DEFAULT_PLASTIC_CUP_SIZE, validate_cup_size,
    PLASTIC_CUPS_PARAMS, PLASTIC_CUP_GRIPPER_POSITIONS,
    PLASTIC_CUP_DISPENSE_GRIPPER, PLASTIC_CUP_DISPENSE_SPEEDS,
    PLASTIC_CUP_EXTRACT_OFFSETS, PLASTIC_CUP_MOVEMENT_OFFSETS,
    GRIPPER_FULL, GRIPPER_OPEN, GRIPPER_RELEASE_GENTLE,
    GRIPPER_RELEASE, GRIPPER_HOLD_LOOSE, SPEED_NORMAL, SPEED_FAST,
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size,
    _set_cup_dispensed, _check_and_clear_cup_dispensed
)
from oms_v1.sequences.computer_vision import detect_cup_gripper

def _normalize_plastic_cup_size(cups_dict: Any) -> str:
    """
    Universal cup size normalizer for plastic cup operations.
    Accepts BOTH H-codes AND C-codes regardless of prefix.
    Extracts the numeric size and returns standardized format.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz', '16oz')
        
    Examples:
        cup_H9 → '9oz'
        cup_C9 → '9oz'
        cup_h12 → '12oz'
        cup_c16 → '16oz'
    """
    if not cups_dict:
        from oms_v1.params import DEFAULT_PLASTIC_CUP_SIZE
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
    from oms_v1.params import DEFAULT_PLASTIC_CUP_SIZE
    return DEFAULT_PLASTIC_CUP_SIZE

def dispense_plastic_cup(**params) -> bool:
    """
    Dispense a plastic cup of specified size from the dispenser.
    """
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_PLASTIC_CUP_SIZE)
    if not cup_size or not validate_cup_size(cup_size):
        return False
    
    CUP_CONFIG = {
        '16oz': {
            'home': 'west',
            'coords': PLASTIC_CUPS_PARAMS['dispenser']['16oz_coords'],
            'gripper': PLASTIC_CUP_DISPENSE_GRIPPER['16oz'],
            'speed': PLASTIC_CUP_DISPENSE_SPEEDS['16oz'],
            'extract_z': PLASTIC_CUP_EXTRACT_OFFSETS['16oz']['z1'],
            'extract_z2': PLASTIC_CUP_EXTRACT_OFFSETS['16oz']['z2'],
        },
        '12oz': {
            'home': 'west',
            'coords': PLASTIC_CUPS_PARAMS['dispenser']['12oz_coords'],
            'gripper': PLASTIC_CUP_DISPENSE_GRIPPER['12oz'],
            'speed': PLASTIC_CUP_DISPENSE_SPEEDS['12oz'],
            'extract_z': PLASTIC_CUP_EXTRACT_OFFSETS['12oz']['z1'],
            'extract_z2': PLASTIC_CUP_EXTRACT_OFFSETS['12oz']['z2'],
        },
        '9oz': {
            'home': 'south_west',
            'coords': PLASTIC_CUPS_PARAMS['dispenser']['9oz_coords'],
            'gripper': PLASTIC_CUP_DISPENSE_GRIPPER['9oz'],
            'speed': PLASTIC_CUP_DISPENSE_SPEEDS['9oz'],
            'extract_z': PLASTIC_CUP_EXTRACT_OFFSETS['9oz']['z1'],
            'extract_z2': PLASTIC_CUP_EXTRACT_OFFSETS['9oz']['z2'],
        },
        '7oz': {
            'home': 'south_west',
            'coords': PLASTIC_CUPS_PARAMS['dispenser']['7oz_coords'],
            'gripper': PLASTIC_CUP_DISPENSE_GRIPPER['7oz'],
            'speed': PLASTIC_CUP_DISPENSE_SPEEDS['7oz'],
            'extract_z': PLASTIC_CUP_EXTRACT_OFFSETS['7oz']['z1'],
            'extract_z2': PLASTIC_CUP_EXTRACT_OFFSETS['7oz']['z2'],
        }
    }
    
    if cup_size not in CUP_CONFIG:
        return False
    
    config = CUP_CONFIG[cup_size]
    attempt_count = 0
    while attempt_count < 15:
            if cup_size == "16oz":
                home(position=config['home'])
                run_skill("set_gripper_position", 255, 0, 255)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 328.0, 10.0, 0, 0, 0)
                run_skill("set_gripper_position", 255,130,255)
                run_skill("set_DO", 1, 1)
                time.sleep(1.5)
                run_skill("set_DO", 1, 0)
                run_skill("moveEE", 0, 0, -150, 0, 0, 0)
                run_skill("moveEE", 0, -328.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            elif cup_size == "12oz":
                home(position=config['home'])
                run_skill("set_gripper_position", 255, 0, 255)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 328.0, 10.0, 0, 0, 0)
                run_skill("set_gripper_position", 255,130,255)
                run_skill("set_DO", 1, 1)
                time.sleep(1.5)
                run_skill("set_DO", 1, 0)
                run_skill("moveEE", 0, 0, -150, 0, 0, 0)
                run_skill("moveEE", 0, -328.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            if cup_size == "9oz":
                home(position=config['home'])
                run_skill("set_gripper_position", 255, 0, 255)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 328.0, 10.0, 0, 0, 0)
                run_skill("set_gripper_position", 255,130,255)
                run_skill("set_DO", 1, 1)
                time.sleep(1.5)
                run_skill("set_DO", 1, 0)
                run_skill("moveEE", 0, 0, -150, 0, 0, 0)
                run_skill("moveEE", 0, -328.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            if cup_size == "7oz":
                home(position=config['home'])
                run_skill("set_gripper_position", 255, 0, 255)
                run_skill("gotoJ_deg", *config['coords'])
                run_skill("moveEE", 0.0, 328.0, 10.0, 0, 0, 0)
                run_skill("set_gripper_position", 255,130,255)
                run_skill("set_DO", 1, 1)
                time.sleep(1.5)
                run_skill("set_DO", 1, 0)
                run_skill("moveEE", 0, 0, -150, 0, 0, 0)
                run_skill("moveEE", 0, -328.0, 0, 0, 0, 0)
                run_skill("gotoJ_deg", *config['coords'])
                home(position=config['home'])
                home(position="north")
            
            run_skill("sync")
            cup_detected = detect_cup_gripper()
            if cup_detected:
                break
            attempt_count += 1
            if attempt_count == 15:
                return False
    
    _set_cup_dispensed()
    return True

def go_to_ice(**params) -> bool:
    """
    Get ice for the specified cup size.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    if not cup_size:
        return False
    
    valid_sizes = ('16oz', '12oz', '9oz', '7oz')
    if cup_size not in valid_sizes:
        return False
    
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position2'])):
        return False

    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    
    run_skill("sync")
    return True

def go_home_with_ice(**params) -> bool:
    """
    Return home with ice-filled cup.
    """
    def ok(r):
        return r not in (False, None)

    if not ok(run_skill("moveEE_movJ", 0,0,5,0,0,0)):
        return False

    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, 145)):
        return False

    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])):
        return False
    
    if not home(position="north"):
        return False
    
    _set_cup_dispensed()
    return True

def place_plastic_cup_station(**params) -> bool:
    """
    Place a plastic cup at specified staging area.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    
    if not cup_size:
        return False
    
    valid_sizes = ('7oz', '9oz', '12oz', '16oz')
    if cup_size not in valid_sizes:
        return False
    
    _check_and_clear_cup_dispensed()
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")
    
    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False
    
    stage_result = False
    if stage == "1":
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_1'])
    elif stage == "2":
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_2'])
    elif stage == "3":
        home(position="south_east")
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_3'])
    elif stage == "4":
        home(position="south_east")
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_4'])
    
    if not ok(stage_result):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)):
        return False
    
    run_skill("set_speed_factor", SPEED_FAST)
    
    if not ok(run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['place_return_up'])):
        return False
    if not home(position="east"):
        return False
    
    return True

def pick_plastic_cup_station(**params) -> bool:
    """
    Pick up a plastic cup from a specific stage and add ice to it.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    
    if not cup_size:
        return False
    
    valid_sizes = ('7oz', '9oz', '12oz', '16oz')
    if cup_size not in valid_sizes:
        return False
    
    stage_positions = {
        "1": PLASTIC_CUPS_PARAMS['staging']['pickup_1'],
        "2": PLASTIC_CUPS_PARAMS['staging']['pickup_2'],
        "3": PLASTIC_CUPS_PARAMS['staging']['pickup_3'],
        "4": PLASTIC_CUPS_PARAMS['staging']['pickup_4']
    }
    
    gripper_positions = {
        "7oz": 145,
        "9oz": 125, 
        "12oz": 140,
        "16oz": 118
    }
    
    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return False
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    
    if not ok(run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['pickup_down'])):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")
    
    if cup_position == 3 or cup_position == 4:
        if not home(position="south_east"):
            return False
    
    if not home(position="east"):
        return False
    if not home(position="north_east"):
        return False
    
    return True

def place_plastic_cup_sauces(**params) -> bool:
    """
    Place the plastic cup at the sauces station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    
    if not cup_size:
        return False
    
    valid_sizes = ("7oz", "9oz", "12oz", "16oz")
    if cup_size not in valid_sizes:
        return False
    
    after_dispense = _check_and_clear_cup_dispensed()
    
    if not after_dispense:
        run_skill("set_speed_factor", SPEED_NORMAL)
        run_skill("sync")
    
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position2'])):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    return True

def pick_plastic_cup_sauces(**params) -> bool:
    """
    Pick the plastic cup from the sauces station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    
    if not cup_size:
        return False
    
    valid_sizes = ("7oz", "9oz", "12oz", "16oz")
    if cup_size not in valid_sizes:
        return False
    
    _check_and_clear_cup_dispensed()
    
    gripper_positions = {
        "7oz": 145,
        "9oz": 145,
        "12oz": 150,
        "16oz": PLASTIC_CUP_GRIPPER_POSITIONS['16oz'],
    }
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")
    
    if not ok(run_skill("moveEE", 0,0,5,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1'])):
        return False
    
    return True

def place_plastic_cup_milk(**params) -> bool:
    """
    Place the plastic cup at the milk station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    
    if not cup_size:
        return False
    
    valid_sizes = ("7oz", "9oz", "12oz", "16oz")
    if cup_size not in valid_sizes:
        return False
    
    after_dispense = _check_and_clear_cup_dispensed()
    
    if not after_dispense:
        run_skill("set_speed_factor", SPEED_NORMAL)
        run_skill("sync")
    
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position2'])):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
        return False
    return True

def pick_plastic_cup_milk(**params) -> bool:
    """
    Pick the plastic cup from the milk station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    
    if not cup_size:
        return False
    
    valid_sizes = ("7oz", "9oz", "12oz", "16oz")
    if cup_size not in valid_sizes:
        return False
    
    _check_and_clear_cup_dispensed()
    
    gripper_positions = {
        "7oz": 145,
        "9oz": 145,
        "12oz": 150,
        "16oz": PLASTIC_CUP_GRIPPER_POSITIONS['16oz'],
    }
    
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")
    
    if not ok(run_skill("moveEE", 0,0,5,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1'])):
        return False
    
    return True
   
# Register functions for CLI discovery and external access
SEQUENCES = {
    'dispense_plastic_cup': dispense_plastic_cup,
    'go_to_ice': go_to_ice,
    'go_home_with_ice': go_home_with_ice,
    'place_plastic_cup_station': place_plastic_cup_station,
    'pick_plastic_cup_station': pick_plastic_cup_station,
    'place_plastic_cup_sauces': place_plastic_cup_sauces,
    'pick_plastic_cup_sauces': pick_plastic_cup_sauces,
    'place_plastic_cup_milk': place_plastic_cup_milk,
    'pick_plastic_cup_milk': pick_plastic_cup_milk,
}
