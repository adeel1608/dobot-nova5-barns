"""
paper_cups.py

Defines the paper cup handling sequences for coffee service automation.
This module provides comprehensive functions for grabbing, placing, and serving
paper cups in the BARNS coffee automation system, including size-based handling
and staging area management.
"""

import time
from typing import Dict, Any, Optional
from oms_v1.params import (
    GRAB_PAPER_CUP_PARAMS, PLACE_PAPER_CUP_PARAMS,
    PAPER_CUPS_NAVIGATION_PARAMS, PAPER_CUPS_STATION_PARAMS,
    PAPER_CUP_GRIPPER_POSITIONS, PAPER_CUP_MOVEMENT_OFFSETS,
    ESPRESSO_MOVEMENT_OFFSETS,
    ESPRESSO_HOME, GRIPPER_OPEN, GRIPPER_RELEASE, GRIPPER_FULL,
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size
)
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home, return_back_to_home
from oms_v1.sequences.computer_vision import detect_cup_gripper

def _normalize_paper_cup_size(cups_dict: Any) -> str:
    """
    Universal cup size normalizer for paper cup operations.
    Accepts BOTH H-codes AND C-codes regardless of prefix.
    Extracts the numeric size and returns standardized format.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz')
        
    Examples:
        cup_H9 → '9oz'
        cup_C9 → '9oz'
        cup_h12 → '12oz'
        cup_c7 → '7oz'
    """
    if not cups_dict:
        from oms_v1.params import DEFAULT_PAPER_CUP_SIZE
        return DEFAULT_PAPER_CUP_SIZE
    
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
                
                # Validate and return standardized size (paper cups: 7, 9, 12)
                if size_num in ('7', '9', '12'):
                    return f"{size_num}oz"
    
    # If parsing failed, try the standard normalizers
    # Try paper first (since this is paper cup function)
    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result
    
    # Try plastic as fallback
    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '' and result in ('7oz', '9oz', '12oz'):
        return result
    
    # Final fallback
    from oms_v1.params import DEFAULT_PAPER_CUP_SIZE
    return DEFAULT_PAPER_CUP_SIZE

def grab_paper_cup(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
    if not size:
        return False
    
    cup_params = GRAB_PAPER_CUP_PARAMS.get(str(size))
    if not cup_params:
        cup_params = GRAB_PAPER_CUP_PARAMS.get("7oz")
        if not cup_params:
            return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['espresso_avoid'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['dispenser_area'])):
        return False
    
    attempt_count = 0
    while attempt_count < 5:
        if size == "7oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_7oz'])):
                return False
        elif size == "9oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_9oz'])):
                return False
        elif size == "12oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_12oz'])):
                return False
        else:
            return False
        
        if 'approach' in cup_params:
            if not ok(run_skill("moveEE", *cup_params['approach'])):
                return False
        
        if 'grip_width' not in cup_params:
            return False
        
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, cup_params['grip_width'])):
            return False
        
        if 'retreat' in cup_params:
            if not ok(run_skill("moveEE", *cup_params['retreat'])):
                return False
        
        cup_detected = detect_cup_gripper()
        if cup_detected:
            break
        
        attempt_count += 1
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)):
            return False
        if attempt_count == 15:
            return False
    
    return True

def place_paper_cup(**params) -> bool:
    """
    Place a paper cup at the specified staging area.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"
    
    stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
    if not stage_params:
        return False
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['intermediate'])):
        return False
    
    if 'twist' in stage_params:
        if not ok(run_skill("moveJ_deg", *stage_params['twist'])):
            return False
    
    if 'pose' not in stage_params:
        return False
    
    if not ok(run_skill("gotoJ_deg", *stage_params['pose'])):
        return False
    
    if not ok(run_skill("set_gripper_position", 25, 0, 255)):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])):
        return False
    
    if 'stage_home' in stage_params:
        if not ok(run_skill("gotoJ_deg", *stage_params['stage_home'])):
            return False
    
    if 'twist_back' in stage_params:
        if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_back_machine'])):
            return False
    
    return True

def dispense_paper_cup_station(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
    """
    if not grab_paper_cup(**params):
        return False
    if not place_paper_cup(**params):
        return False
    return True

def pick_paper_cup_station(**params) -> bool:
    """
    Pick up a paper cup from a specific stage.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return False
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['pickup_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['pickup_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['pickup_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['pickup_4']
    }
    
    gripper_positions = PAPER_CUP_GRIPPER_POSITIONS
    
    if not home(position="east"):
        return False
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return False
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_down'])):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[size_mapped])):
        return False
    
    run_skill("moveEE_movJ", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_up'])
    
    if not home(position="east"):
        return False
    
    if not home(position="north_east"):
        return False
    
    return True

def place_paper_cup_station(**params) -> bool:
    """
    Place a paper cup at specified staging area.

    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return False
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['place_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['place_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['place_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['place_4']
    }
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_return_up'])):
        return False
    if not home(position="east"):
        return False
    
    return True

def place_paper_cup_sauces(**params) -> bool:
    """
    Place the paper cup at the sauces station.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position3'])):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)):
        return False
    return True

def pick_paper_cup_sauces(**params) -> bool:
    """
    Pick the paper cup from the sauces station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    cup_size = _normalize_paper_cup_size(cups_dict)
    valid_sizes = ("7oz", "9oz", "12oz")
    if cup_size not in valid_sizes:
        return False
    
    gripper_position = 145
    if not ok(run_skill("moveEE", 0,0,5,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_position)):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position1'])):
        return False
    return True

def place_paper_cup_milk(**params) -> bool:
    """
    Place the paper cup at the milk station.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position3'])):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)):
        return False
    return True

def pick_paper_cup_milk(**params) -> bool:
    """
    Pick the paper cup from the milk station.
    """
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    cup_size = _normalize_paper_cup_size(cups_dict)
    valid_sizes = ("7oz", "9oz", "12oz")
    if cup_size not in valid_sizes:
        return False
    
    gripper_position = 145
    if not ok(run_skill("moveEE", 0,0,5,0,0,0)):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_position)):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position1'])):
        return False
    return True

def pick_cup_for_hot_water(**params) -> bool:
    """
    Pick up a paper cup from a specific stage for hot water.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return False
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_4']
    }
    
    gripper_positions = PAPER_CUP_GRIPPER_POSITIONS
    
    if not home(position="south_west"):
        return False
    if stage in ("3", "4"):
        if not home(position="south"):
            return False
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_hot_water_down'])):
        return False
    if size_mapped == '12oz':
        if not ok(run_skill("set_gripper_position", 255,120,255)):
            return False
    else:
        if not ok(run_skill("set_gripper_position", 255,125,255)):
            return False
    
    run_skill("set_speed_factor", 75)
    
    run_skill("moveEE_movJ", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_up'])
    
    if not home(position="west"):
        return False

    if not ok(run_skill("approach_machine", "three_group_espresso", "hot_water")):
        return False
    
    if not ok(run_skill("mount_machine", "three_group_espresso", "hot_water")):
        return False

    run_skill("sync")    
    
    return True

def return_cup_with_hot_water(**params) -> bool:
    """
    Complete hot water dispensing sequence and return to holding position.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return False
    
    stage_params_map = {
        "1": PLACE_PAPER_CUP_PARAMS['stage_1'],
        "2": PLACE_PAPER_CUP_PARAMS['stage_2'],
        "3": PLACE_PAPER_CUP_PARAMS['stage_3'],
        "4": PLACE_PAPER_CUP_PARAMS['stage_4'],
    }
    
    stage_params = stage_params_map.get(stage, {})

    if not ok(run_skill("set_speed_factor",25)):
        return False

    if not ok(run_skill("moveEE", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return False

    if stage in ("1"):
        if not run_skill("gotoJ_deg", 112.5,30,-130,-90,-90,0):
            return False
    if stage in ("2","3", "4"):
        if not home(position="south_west"):
            return False
    if stage in ("3", "4"):
        if not home(position="south"):
            return False
    
    if 'pose' not in stage_params:
        return False
    
    if not ok(run_skill("gotoJ_deg", *stage_params['pose'])):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)):
        return False
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])):
        return False

    if not ok(run_skill("set_speed_factor", 100)):
        return False
    
    if 'stage_home' in stage_params:
        if not ok(run_skill("gotoJ_deg", *stage_params['stage_home'])):
            return False
    
    if 'twist_back' in stage_params:
        if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_back_machine'])):
            return False
    
    return True

# Register functions for CLI discovery and external access
SEQUENCES = {
    'grab_paper_cup': grab_paper_cup,
    'place_paper_cup': place_paper_cup,
    'dispense_paper_cup_station': dispense_paper_cup_station,
    'pick_paper_cup_station': pick_paper_cup_station,
    'place_paper_cup_station': place_paper_cup_station,
    'place_paper_cup_sauces': place_paper_cup_sauces,
    'pick_paper_cup_sauces': pick_paper_cup_sauces,
    'place_paper_cup_milk': place_paper_cup_milk,
    'pick_paper_cup_milk': pick_paper_cup_milk,
    'pick_cup_for_hot_water': pick_cup_for_hot_water,
    'return_cup_with_hot_water': return_cup_with_hot_water,
}
