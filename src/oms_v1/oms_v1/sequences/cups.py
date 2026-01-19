"""
cups.py

Defines cup-grabbing and placing sequences using parameterized configurations.
"""

import time
from oms_v1.params import GRAB_CUP_PARAMS, PLACE_CUP_PARAMS
from oms_v1.manipulate_node import run_skill

def _prep_for_grab():
    """Perform common refresh and initial positioning steps before grabbing a cup."""
    run_skill("refresh_position")
    time.sleep(2.0)

def grab_cup(size):
    """
    Approaches and picks up the specified cup.
    
    Args:
        size (str): Cup size key, e.g. '7oz', '9oz', '12oz'.
    """
    params = GRAB_CUP_PARAMS.get(size)
    if not params:
        print(f"[ERROR] unknown cup size: {size!r}")
        return
    _prep_for_grab()
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chosen for grabbing the cup
    # Rotate joint angles to back away before approach
    run_skill("moveJ_deg", *params['twist_back'])
    # Move end-effector into approach position
    run_skill("moveEE", *params['approach'])
    # Close gripper to grasp the cup
    run_skill("set_gripper_position", 255, params['grip_width'])
    # Retract after gripping
    run_skill("moveEE", *params['retreat'])
    run_skill("gotoJ_deg", 88.657143,21.041538,-74.451630,-36.522381,90.145508,-91.183128)

def place_cup(stage):
    """
    Places the currently held cup at the specified stage.
    
    Args:
        stage (str): Placement stage key, e.g. 'stage_1', 'stage_2'.
    """
    params = PLACE_CUP_PARAMS.get(stage)
    if not params:
        print(f"[ERROR] unknown stage: {stage!r}")
        return
    run_skill("gotoJ_deg", 88.657143,21.041538,-74.451630,-36.522381,90.145508,-91.183128)
    # Move into staging twist angle
    run_skill("moveJ_deg", *params['twist'])
    # Move to target placement pose
    run_skill("gotoJ_deg", *params['pose'])
    # Open gripper to release cup
    run_skill("set_gripper_position", 50, 0)
    time.sleep(1.0)
    run_skill("moveEE", 0,0,150,0,0,0) # Move up after placing cup
    run_skill("gotoJ_deg", *params['stage_home']) # Move to staging home position
    run_skill("moveJ_deg", *params['twist_back']) # Untwist back towards machine
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home

def pick_cup(size, stage):
    """
    Full pick-and-place sequence for cups.
    
    Args:
        size (str): Cup size key (e.g., '7oz', '9oz', '12oz').
        stage (str): Placement stage key ('stage_1' or 'stage_2').
    """
    grab_cup(size)
    place_cup(stage)

def serve(stage):
    params = PLACE_CUP_PARAMS.get(stage)
    if not params:
        print(f"[ERROR] unknown stage {stage!r}")
        return
    #Pickup from staging place in delivery
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("moveJ_deg", *params['twist_serve'])#run_skill("moveJ_deg", 31.983258, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", *params['pick'])#run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478)
    run_skill("moveEE", 0, 0, -140, 0, 0, 0)
    run_skill("set_gripper_position", 55, 125)
    run_skill("set_servo_timing", 0.20)
    run_skill("gotoJ_deg", *params['pick'])#run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478, 1.0, 0.2)
    run_skill("gotoJ_deg", *params['above_serve'])
    run_skill("gotoJ_deg", *params['serve'])
    run_skill("set_gripper_position", 55, 0)
    run_skill("set_servo_timing", 0.10)
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 140, 0, 0, 0)
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home
    
# Register for CLI discovery
SEQUENCES = {
    'grab_cup': grab_cup,
    'place_cup': place_cup,
    'pick_cup': pick_cup,
    'serve': serve,
}
