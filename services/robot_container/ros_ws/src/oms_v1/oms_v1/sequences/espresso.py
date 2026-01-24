"""
espresso.py

Defines the espresso-making sequence for different ports and cups.
This module provides comprehensive functions for managing the complete espresso
workflow including portafilter handling, grinding, tamping, mounting, and milk operations.
"""

import time
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill
from oms_v1.params import (
    PULL_ESPRESSO_PARAMS, 
    ESPRESSO_HOME, 
    ESPRESSO_GRINDER_HOME,
    ESPRESSO_GRINDER_PARAMS,
    ESPRESSO_PITCHER_PARAMS,
    ESPRESSO_HOT_WATER_PARAMS,
    ESPRESSO_SPEEDS, ESPRESSO_PITCHER_GRIPPER, ESPRESSO_PORTAFILTER_GRIPPER,
    ESPRESSO_MOVEMENT_OFFSETS, ESPRESSO_DELAYS, PORTAFILTER_Z_THRESHOLD_MM,
    GRIPPER_FULL, GRIPPER_OPEN, SPEED_FAST, SPEED_SUPER_SLOW, SPEED_SLOW_POURING,
    _extract_cup_position
)
from oms_v1.sequences.cleaning import clean_portafilter

# Global variables to store captured positions during unmount sequence
below_espresso_port: Optional[Tuple[float, ...]] = None
mount_espresso_port: Optional[Tuple[float, ...]] = None
mount_espresso_pose: Optional[Tuple[float, ...]] = None  # Cartesian pose at mount position
approach_pitcher: Optional[Tuple[float, ...]] = None
pick_pitcher: Optional[Tuple[float, ...]] = None


def _normalize_espresso_shot(espresso_dict: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """
    Parse espresso parameters from new JSON format.

    Expected format: {'espresso_shot_single': 1.0} or {'espresso_shot_double': 2.0}

    Rules:
      - 'espresso_shot_single' -> single shot → port_3, positioning_time=5.0, portafilter_tool=single_portafilter
      - 'espresso_shot_double' -> double shot → port_1, positioning_time=5.0, portafilter_tool=double_portafilter
    """
    try:
        if not espresso_dict or not isinstance(espresso_dict, dict):
            return None
        
        # Get the first key from the espresso dictionary
        espresso_key = next(iter(espresso_dict.keys()), None)
        if not espresso_key:
            return None
        
        # Parse the key to determine shot type
        espresso_key_lower = str(espresso_key).lower()
        
        if 'single' in espresso_key_lower:
            return {
                "port": "port_3",
                "positioning_time": 5.0,
                "portafilter_tool": "single_portafilter",
            }
        elif 'double' in espresso_key_lower:
            return {
                "port": "port_1",
                "positioning_time": 5.0,
                "portafilter_tool": "double_portafilter",
            }
        else:
            # Fallback: try to parse as numeric value
            value = espresso_dict.get(espresso_key)
            if value is not None:
                shots = float(value)
                if shots <= 1.0:
                    return {
                        "port": "port_3",
                        "positioning_time": 5.0,
                        "portafilter_tool": "single_portafilter",
                    }
                else:
                    return {
                        "port": "port_1",
                        "positioning_time": 5.0,
                        "portafilter_tool": "double_portafilter",
                    }
    except Exception as e:
        print(f"[WARNING] Error parsing espresso parameters: {e}")
        return None
    
    return None

def unmount(**params) -> bool:
    """
    Unmount portafilter from espresso group for cleaning or grinding.
    """
    global below_espresso_port, mount_espresso_port, mount_espresso_pose
    
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port:
        return False
    
    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return False
    
    if not ok(run_skill("gotoJ_deg", *port_params['home'])):
        return False
    
    if port == 'port_1' or port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'])):
            return False
    
    if not ok(run_skill("mount_machine", "three_group_espresso", port_params['portafilter_number'])):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['grip'])):
        return False
    
    if not ok(run_skill("release_tension")):
        return False
    
    if not ok(run_skill("enforce_rxry")):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("move_portafilter_arc_movJ", -42.0)):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("release_tension")):
        return False
    
    time.sleep(ESPRESSO_DELAYS['orientation_settle'])
    
    mount_espresso_port = run_skill("current_angles")
    if not ok(mount_espresso_port) or not isinstance(mount_espresso_port, (tuple, list)) or len(mount_espresso_port) != 6:
        return False
    
    mount_espresso_pose = run_skill("current_pose")
    if not ok(mount_espresso_pose) or not isinstance(mount_espresso_pose, (tuple, list)) or len(mount_espresso_pose) != 6:
        return False
    
    if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down'])):
        return False
    
    below_espresso_port = run_skill("current_angles")
    if not ok(below_espresso_port) or not isinstance(below_espresso_port, (tuple, list)) or len(below_espresso_port) != 6:
        return False
    
    if not ok(run_skill("gotoJ_deg", *port_params['move_back'])):
        return False
    
    if port in ('port_2', 'port_3'):
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])):
            return False
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav2'])):
            return False
    
    return True

def grinder(**params) -> bool:
    """
    Grind coffee and tamp portafilter at the grinder station.
    """
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    positioning_time = params.get("positioning_time")
    if positioning_time is None:
        positioning_time = (shot_cfg.get("positioning_time") if shot_cfg else 5.0)
    portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "double_portafilter")
    
    if not port or portafilter_tool not in ('single_portafilter', 'double_portafilter'):
        return False
    
    if port == 'port_1':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
            return False
    
    if not ok(run_skill("approach_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("mount_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("approach_machine", "espresso_grinder", "tamper")):
        return False
    
    time.sleep(positioning_time)
    
    if not ok(run_skill("mount_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("mount_machine", "espresso_grinder", "tamper")):
        return False
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['release'])):
        return False
    
    approach_tool_result = run_skill("approach_tool", portafilter_tool)
    if not ok(approach_tool_result):
        fallback_tool = "double_portafilter" if portafilter_tool == "single_portafilter" else "single_portafilter"
        if not ok(run_skill("approach_tool", fallback_tool)):
            return False
    
    return True
    
def tamper(**params) -> bool:
    """
    Tamp coffee at the tamper station using portafilter tool.
    """
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    portafilter_tool = params.get("portafilter_tool") or (shot_cfg.get("portafilter_tool") if shot_cfg else "single_portafilter")
    
    if portafilter_tool not in ('single_portafilter', 'double_portafilter'):
        return False
    
    run_skill("sync")
    
    approach_tool_result = run_skill("approach_tool", portafilter_tool)
    if not ok(approach_tool_result):
        fallback_tool = "double_portafilter" if portafilter_tool == "single_portafilter" else "single_portafilter"
        if not ok(run_skill("approach_tool", fallback_tool)):
            return False
        portafilter_tool = fallback_tool
    
    run_skill("sync")
    
    if not ok(run_skill("grab_tool", portafilter_tool)):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['grip'])):
        return False
    
    run_skill("moveEE_movJ", 0, 0, -5, 0, 0, 0)
    
    if not ok(run_skill("moveEE", 0, 0, 20, 0, 0, 0)):
        return False
    
    if not ok(run_skill("mount_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("approach_machine", "espresso_grinder", "grinder")):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False
    
    return True

def mount(**params) -> bool:
    """
    Mount portafilter back to espresso group after grinding.
    """
    global mount_espresso_pose, below_espresso_port
    
    def ok(r):
        return r not in (False, None)
    
    attempt_count = params.get("attempt_count", 0)
    if attempt_count >= 3:
        return False
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port:
        return False
    
    port_params = PULL_ESPRESSO_PARAMS.get(str(port))
    if not port_params:
        return False
    
    if port in ('port_2', 'port_3'):
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_PARAMS['nav1'])):
            return False
    
    if not ok(run_skill("gotoJ_deg", *port_params['move_back'])):
        return False
    
    if below_espresso_port is None or not isinstance(below_espresso_port, (tuple, list)) or len(below_espresso_port) != 6:
        return False
    
    if not ok(run_skill("gotoJ_deg", *below_espresso_port)):
        return False
    
    if mount_espresso_port is None or not isinstance(mount_espresso_port, (tuple, list)) or len(mount_espresso_port) != 6:
        return False
    
    if not ok(run_skill("gotoJ_deg", *mount_espresso_port)):
        return False
    
    run_skill("sync")
    
    current_mount_pose = run_skill("current_pose")
    if current_mount_pose is not None and mount_espresso_pose is not None:
        z_original = float(mount_espresso_pose[2])
        z_current = float(current_mount_pose[2])
        z_difference = abs(z_current - z_original)
        
        if z_difference > PORTAFILTER_Z_THRESHOLD_MM:
            if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_down'])):
                return False
            
            below_espresso_port = run_skill("current_angles")
            if not ok(below_espresso_port) or not isinstance(below_espresso_port, (tuple, list)) or len(below_espresso_port) != 6:
                return False
            
            if port_params and 'move_back' in port_params:
                if not ok(run_skill("gotoJ_deg", *port_params['move_back'])):
                    return False
            
            if not clean_portafilter(port=port):
                return False
            
            if not mount(port=port, attempt_count=attempt_count + 1):
                return False
            
            return True
        else:
            if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['portafilter_clear_up'])):
                return False
    
    run_skill("sync")
    
    if not ok(run_skill("enforce_rxry")):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("move_portafilter_arc_movJ", 44.0)):
        return False
    
    run_skill("sync")
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PORTAFILTER_GRIPPER['release'])):
        return False
    
    if port == 'port_1' or port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'])):
            return False
    
    if not ok(run_skill("gotoJ_deg", *port_params['home'])):
        return False
    
    return True

def pick_espresso_pitcher(**params) -> bool:
    """
    Pick up espresso pitcher for the specified port.
    """
    global approach_pitcher, pick_pitcher
    
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        return False
    
    if port == 'port_1':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_1'])):
            return False
        run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
        run_skill("sync")
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
    elif port == 'port_2':
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_2'])):
            return False
        run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
        run_skill("sync")
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
            return False
    elif port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_3'])):
            return False
        run_skill("set_speed_factor", ESPRESSO_SPEEDS['pitcher_handling'])
        run_skill("sync")
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
    
    if port == 'port_1' or port == 'port_2':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
            return False
    
    return True    

def pour_espresso_pitcher_cup_station(**params) -> bool:
    """
    Pour milk from espresso pitcher into cup at specified position.
    """
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"
    
    run_skill("gotoJ_deg", 103.201965,-21.933174,-150.611664,-10.398072,-23.882843,0.127716)
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return False
    
    if stage == 'stage_1':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos1'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour1'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral1'])):
            return False
    elif stage == 'stage_2':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos2'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour2'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral2'])):
            return False
    elif stage == 'stage_3':
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour3'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral3'])):
            return False
    else:  # stage_4
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pos4'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", SPEED_SLOW_POURING)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['pour4'])):
            return False
        run_skill("sync")
        run_skill("set_speed_factor", 100)
        if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['neutral4'])):
            return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['inter'])):
        return False
    
    run_skill("gotoJ_deg", 103.201965,-21.933174,-150.611664,-10.398072,-23.882843,0.127716)
    run_skill("sync")
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_PITCHER_PARAMS['home'])):
        return False
    
    return True

def get_hot_water(**params) -> bool:
    """
    Position espresso pitcher under hot water dispenser.
    """
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "hot_water")):
        return False
    
    if not ok(run_skill("mount_machine", "three_group_espresso", "hot_water")):
        return False
    
    run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_move'])
    
    return True

def with_hot_water(**params) -> bool:
    """
    Complete hot water dispensing sequence and return to holding position.
    """
    def ok(r):
        return r not in (False, None)
    
    run_skill("set_speed_factor", ESPRESSO_SPEEDS['hot_water_pour'])
    
    if not ok(run_skill("moveEE_movJ", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return False
    
    return True

def return_espresso_pitcher(**params) -> bool:
    """
    Return espresso pitcher to its home position after use.
    """
    global approach_pitcher, pick_pitcher
    
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False
    
    if port == 'port_1':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
    elif port == 'port_2':
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
            return False
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
    elif port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    return True

def return_cleaned_espresso_pitcher(**params) -> bool:
    global approach_pitcher, pick_pitcher
    
    def ok(r):
        return r not in (False, None)
    
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else "port_2")
    
    if not port or port not in ('port_1', 'port_2', 'port_3'):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        return False
    
    if port == 'port_1':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_1'])):
            return False
        run_skill("moveEE_movJ", 0,0,10,0,0,0)
        run_skill("moveJ_deg", 0,0,0,0,0,-130)
        run_skill("sync")
        run_skill("moveJ_deg", 0,0,0,0,0,130)
        run_skill("moveEE_movJ", 0,0,-10,0,0,0)
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1")):
            return False
    elif port == 'port_2':
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_2'])):
            return False
        run_skill("moveEE_movJ", 0,0,10,0,0,0)
        run_skill("moveJ_deg", 0,0,0,0,0,-130)
        run_skill("sync")
        run_skill("moveJ_deg", 0,0,0,0,0,130)
        run_skill("moveEE_movJ", 0,0,-10,0,0,0)
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
    elif port == 'port_3':
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, ESPRESSO_PITCHER_GRIPPER['port_3'])):
            return False
        run_skill("moveEE_movJ", 0,0,10,0,0,0)
        run_skill("moveJ_deg", 0,0,0,0,0,-150)
        run_skill("moveJ_deg", 0,0,0,0,0,150)
        run_skill("moveEE_movJ", 0,0,-10,0,0,0)
        if not ok(run_skill("set_gripper_position", ESPRESSO_PITCHER_GRIPPER['release'], GRIPPER_OPEN)):
            return False
        if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3")):
            return False
    
    if not ok(run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2")):
        return False
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return False
    
    return True

# Register functions for CLI discovery and external access
SEQUENCES = {
    'unmount': unmount,
    'grinder': grinder,
    'mount': mount,
    'pick_espresso_pitcher': pick_espresso_pitcher,
    'pour_espresso_pitcher_cup_station': pour_espresso_pitcher_cup_station,
    'get_hot_water': get_hot_water,
    'with_hot_water': with_hot_water,
    'return_espresso_pitcher': return_espresso_pitcher,
    'return_cleaned_espresso_pitcher': return_cleaned_espresso_pitcher,
    'tamper': tamper,
}