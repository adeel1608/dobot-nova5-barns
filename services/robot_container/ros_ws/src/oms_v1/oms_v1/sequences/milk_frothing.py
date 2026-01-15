"""
milk_frothing.py

Defines the milk frothing sequences for coffee preparation automation.
This module provides comprehensive functions for handling milk frothing operations
in the BARNS coffee automation system, including frother positioning, mounting,
steam activation, milk pouring, and cleaning procedures.
"""

import time
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home, return_back_to_home
from oms_v1.params import (
    MILK_FROTHING_PARAMS, MILK_FROTHER_SPEEDS, MILK_FROTHER_GRIPPER_POSITIONS,
    MILK_FROTHER_MOVEMENT_OFFSETS, MILK_POURING_OFFSETS, MILK_FROTHING_DELAYS,
    MILK_SWIRL_CIRCLE_PARAMS, MILK_VOLUME_Z_ADJUSTMENT_FACTOR,
    GRIPPER_FULL, GRIPPER_OPEN, CALIBRATION_SETTLE_TIME,
    _extract_cup_position
)

# Global variables to store robot positions during milk frothing operations
# These are used to remember positions between function calls for safe return operations
approach_angles: Optional[Tuple[float, ...]] = None
grab_angles: Optional[Tuple[float, ...]] = None


def get_frother_position(**params) -> bool:
    """
    Calibrate and record the milk frother position for future operations.
    """
    def ok(r):
        return r not in (False, None)
    
    run_skill("set_speed_factor", 100)
    
    if not home(position="north_east"):
        return False
    
    cycles = 4
    for i in range(cycles):
        time.sleep(CALIBRATION_SETTLE_TIME)
        if not ok(run_skill("move_to", "left_steam_wand", 0.29)):
            return False
    
    if not ok(run_skill("get_machine_position", "left_steam_wand")):
        return False
    
    return True

def pick_frother(**params) -> bool:
    """
    Pick up the milk frother for milk frothing operations.
    """
    global approach_angles, grab_angles
    
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area'])):
        return False
    
    if not ok(run_skill("move_to", 'milk_frother_1', 0.29)):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("approach_tool", 'milk_frother_1')):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'])):
        return False
    
    run_skill("sync")
    
    approach_angles = run_skill("current_angles")
    
    run_skill("sync")
    
    if not ok(run_skill("grab_tool", 'milk_frother_1', 100, 100,-5,-10.5)):
        return False
    
    run_skill("sync")
    
    grab_angles = run_skill("current_angles")
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure'])):
        return False
    
    return True
        
def place_frother_milk_station(**params) -> bool:
    """
    Place the milk frother at the milk station safely.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_place'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_approach'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_final'])):
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['place'])):
        return False
    return True

def pick_frother_milk_station(**params) -> bool:
    """
    Pick the milk frother up from the milk station safely.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure'])):
        return False
    if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_pick'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat2'])):
        return False
    return True

def mount_frother(**params) -> bool:
    """
    Mount the milk frother to the steam wand for frothing preparation.
    """
    def ok(r):
        return r not in (False, None)
    
    run_skill("sync")
    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['mount'])
    
    if not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth")):
        return False
    
    if not ok(run_skill("mount_machine", "left_steam_wand", "deep_froth")):
        return False
    
    run_skill("sync")
    run_skill("release_tension")
    run_skill("moveEE_movJ", 10,-10,-5,0,0,0)
    run_skill("moveEE_movJ", 0,0,5,0,0,0)
    
    milk_data = params.get('milk', {})
    volume_ml = next(iter(milk_data.values()), 0) if milk_data else 0
    z_adjustment = MILK_VOLUME_Z_ADJUSTMENT_FACTOR * volume_ml
    run_skill("moveEE_movJ", 0, 0, -z_adjustment, 0, 0, 0)
    
    if not ok(run_skill("sync")):
        return False
    
    return True

def unmount_and_swirl_milk(**params) -> bool:
    """
    Swirl frothed milk in a circular motion for latte art preparation.
    """
    def ok(r):
        return r not in (False, None)
    
    time.sleep(MILK_FROTHING_DELAYS['swirl_delay'])
    
    if not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth")):
        return False
    
    run_skill("sync")
    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['swirl'])
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['intermediate1'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['swirl_pos'])):
        return False
    
    run_skill("sync")
    
    run_skill("move_circle", 
        MILK_SWIRL_CIRCLE_PARAMS['cycles'],
        MILK_SWIRL_CIRCLE_PARAMS['point1_offset'],
        MILK_SWIRL_CIRCLE_PARAMS['point2_offset'],
        MILK_SWIRL_CIRCLE_PARAMS['options'])
    
    return True

def pour_milk_cup_station(**params) -> bool:
    """
    Pour frothed milk into cup at specified stage.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    if stage == '1':
        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['position'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
        run_skill("sync")
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['adjust1'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage1']['move_forward'])
        run_skill("sync")
        time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage1']['move_up'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['position'])
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    elif stage == '2':
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['position'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['adjust1'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage2']['move_forward'])
        time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage2']['move_up'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['position'])
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    elif stage == '3':
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['position'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['adjust1'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage3']['move_forward'])
        time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage3']['move_up'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['position'])
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    else:  # stage == '4'
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['position'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['adjust1'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage4']['move_forward'])
        time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
        run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage4']['move_up'])
        run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['position'])
        run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    
    return True

def clean_milk_pitcher(**params) -> bool:
    """
    Perform a cleaning motion for the frother tool.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose3'])):
        return False
    if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['cleaning_motion'])):
        return False
    return True

def return_frother(**params) -> bool:
    """
    Return the frother to its original location using recorded approach/grab angles.
    """
    global approach_angles, grab_angles
    
    def ok(r):
        return r not in (False, None)
    
    if grab_angles is None or approach_angles is None:
        return False
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return3'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return4'])):
        return False
    if not ok(run_skill("gotoJ_deg", *grab_angles)):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['release'])):
        return False
    if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['final_approach'])):
        return False
    if not ok(run_skill("gotoJ_deg", *approach_angles)):
        return False
    home(position="north")
    run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open'])
    return True

# Register functions for CLI discovery and external access
SEQUENCES = {
    'get_frother_position': get_frother_position,
    'pick_frother': pick_frother,
    'unmount_and_swirl_milk': unmount_and_swirl_milk,
    'pour_milk_cup_station': pour_milk_cup_station,
    'mount_frother': mount_frother,
    'clean_milk_pitcher': clean_milk_pitcher,
    'return_frother': return_frother,
    'place_frother_milk_station': place_frother_milk_station,
    'pick_frother_milk_station': pick_frother_milk_station,
}

