"""
home.py

Defines the 'home' positioning routine using compass directions.
This module provides functions for robot positioning, machine calibration,
and system diagnostics for the BARNS coffee automation system.
"""

import time
from typing import Dict, Any, Union
from oms_v1.params import (
    HOME_ANGLES, ESPRESSO_HOME, ESPRESSO_GRINDER_HOME, 
    HOME_CALIBRATION_PARAMS, HOME_CALIBRATION_CONSTANTS,
    GRIPPER_FULL, GRIPPER_OPEN, SPEED_FAST
)
from oms_v1.manipulate_node import run_skill


def home(**params) -> bool:
    """
    Move robot to a predefined home position.
    """
    def ok(r):
        return r not in (False, None)
    
    position = params.get("position", "north")
    if not position:
        return False
    
    angles = HOME_ANGLES.get(str(position))
    if not angles:
        return False
    
    if not ok(run_skill("gotoJ_deg", *angles)):
        return False
    
    return True

def return_back_to_home() -> bool:
    """
    Return the robot to a safe home position based on current angle.
    """
    def ok(r):
        return r not in (False, None)
    
    release_result = run_skill("release_tension")
    if not ok(release_result):
        if not ok(run_skill("toggle_drag_mode")):
            return False
    
    run_skill("set_speed_factor", SPEED_FAST)
    run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN)
    angles = run_skill("current_angles")
    
    if not ok(angles) or len(angles) < 6:
        return False
    
    a1 = float(angles[0])
    j1_val = None
    
    if -22.49 <= a1 <= 22.49:
        j1_val = 0.0
    elif 22.51 <= a1 <= 67.49:
        j1_val = 45.0
    elif 67.51 <= a1 <= 112.49:
        j1_val = 90.0
    elif 112.51 <= a1 <= 157.49:
        j1_val = 135.0
    elif 157.51 <= a1 <= 202.49:
        j1_val = 180.0
    elif 202.51 <= a1 <= 247.49:
        j1_val = -135.0
    elif 247.51 <= a1 <= 292.49:
        j1_val = -90.0
    elif 292.51 <= a1 <= 337.49:
        j1_val = -45.0
    elif 337.51 <= a1 <= 360.0:
        j1_val = 0.0
    elif -67.49 <= a1 <= -22.51:
        j1_val = -45.0
    elif -112.49 <= a1 <= -67.51:
        j1_val = -90.0
    elif -157.49 <= a1 <= -112.51:
        j1_val = -135.0
    elif -202.49 <= a1 <= -157.51:
        j1_val = -180.0
    elif -247.49 <= a1 <= -202.51:
        j1_val = 135.0
    elif -292.49 <= a1 <= -247.51:
        j1_val = 90.0
    elif -337.49 <= a1 <= -292.51:
        j1_val = 45.0
    elif -360.0 <= a1 <= -337.51:
        j1_val = 0.0
    
    if j1_val is None:
        return False
    
    home_j2_j6 = HOME_CALIBRATION_PARAMS['return_home_position']
    if not ok(run_skill("gotoJ_deg", j1_val, *home_j2_j6)):
        return False
    
    return True

def get_machine_position(**params) -> bool:
    """
    Calibrate and record machine positions for all coffee equipment.
    """
    def ok(r):
        return r not in (False, None)
    
    run_skill("set_speed_factor", SPEED_FAST)
    
    if not return_back_to_home():
        return False
    
    if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['portafilter_cleaner']['prep_position'])):
        return False
    
    cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
    for i in range(cycles):
        time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
        if not ok(run_skill("move_to", "portafilter_cleaner", 0.26)):
            return False
    
    run_skill("sync")
    
    cleaner_record_result = run_skill("get_machine_position", "portafilter_cleaner")
    if not ok(cleaner_record_result):
        return False
    
    if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep1'])):
        return False
    
    if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['espresso_grinder_calibration']['prep2'])):
        return False
    
    cycles = HOME_CALIBRATION_CONSTANTS['approach_cycles']
    for i in range(cycles):
        time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
        if not ok(run_skill("move_to", "espresso_grinder", 0.26)):
            return False
    
    run_skill("sync")
    
    grinder_record_result = run_skill("get_machine_position", "espresso_grinder")
    if not ok(grinder_record_result):
        return False
    
    if not ok(run_skill("gotoJ_deg", *HOME_CALIBRATION_PARAMS['three_group_espresso_calibration']['prep1'])):
        return False
    
    if not ok(run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)):
        return False
    
    cycles = 15
    for i in range(cycles):
        time.sleep(HOME_CALIBRATION_CONSTANTS['settle_time'])
        if not ok(run_skill("move_to", "three_group_espresso", 0.26)):
            return False
    
    run_skill("sync")
    
    espresso_record_result = run_skill("get_machine_position", "three_group_espresso")
    if not ok(espresso_record_result):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    check_saved_data()
    
    return True

def check_saved_data() -> Dict[str, Any]:
    """
    Check and display currently saved machine position data.
    """
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    
    try:
        pkg_share = get_package_share_directory("pickn_place")
        mem_path = os.path.join(pkg_share, "machine_pose_data_memory.yaml")
        
        if not os.path.exists(mem_path):
            return {}
        
        with open(mem_path, "r") as f:
            data = yaml.safe_load(f) or {}
        
        machines = data.get("machines", {})
        return machines
        
    except Exception as e:
        return {}

def check_aruco_status(**params) -> bool:
    """
    Check current ArUco marker detection status and help diagnose calibration issues.
    """
    check_saved_data()
    return True

# Register functions for CLI discovery and external access
SEQUENCES = {
    'home': home,
    'return_back_to_home': return_back_to_home,
    'get_machine_position': get_machine_position,
    'check_saved_data': check_saved_data,
    'check_aruco_status': check_aruco_status,
}
