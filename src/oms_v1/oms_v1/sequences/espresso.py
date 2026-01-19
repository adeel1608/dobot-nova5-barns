"""
espresso.py

Defines the espresso-making sequence for different ports and cups.
"""

import time
from oms_v1.manipulate_node import run_skill
from oms_v1.params import PULL_ESPRESSO_PARAMS, POUR_PARAMS, Espresso_home, Espresso_grinder_home

def _prep_for_espresso():
    """Perform common refresh and initial positioning steps before grabbing a cup."""
    run_skill("refresh_position")
    time.sleep(2.0)

def unmount(port):
    """Unmount portafilter"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return
    _prep_for_espresso()
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", *params['home']) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("mount_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("set_gripper_position", 255, 255) #close gripper #Close gripper
    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)  
    run_skill("move_portafilter_arc",-45)
    run_skill("release_tension")
    run_skill("moveEE", 0, 0, -35, 0, 0, 0)
    run_skill("gotoJ_deg", *params['below_port']) #A point below the port chosen for the motion plan
    run_skill("gotoJ_deg", *params['move_back']) #A point in the plan chosen to avoid hitting anything in the path
    if port in ('port_2', 'port_3'):
        run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
        run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)

def grinder(port):
    """Grinder port"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return
    run_skill("gotoJ_deg", *Espresso_grinder_home) #Espresso grinder home
    run_skill("approach_machine", "espresso_grinder", "grinder", True) 
    run_skill("mount_machine", "espresso_grinder", "grinder", True)
    run_skill("approach_machine", "espresso_grinder", "tamper", True)
    time.sleep(1.0)
    run_skill("mount_machine", "espresso_grinder", "tamper", True)
    run_skill("moveEE", 0, 0, 45, 0, 0, 0) #go up to tamp #go up to tamp
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, -95, 0, 0, 0) #go below tamper #go below tamper
    run_skill("approach_machine", "espresso_grinder", "grinder", True)
    run_skill("gotoJ_deg", *Espresso_grinder_home)

def mount(port, skip_initial_move=False):
    """Mount portafilter"""
    params = PULL_ESPRESSO_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port number: {port!r}")
        return

    if port in ('port_2', 'port_3'):
        if not skip_initial_move:
            run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)

    run_skill("gotoJ_deg", *params['move_back'])  # A point in the plan chosen to avoid hitting anything in the path
    run_skill("gotoJ_deg", *params['below_port'])  # A point below the port chosen for the motion plan
    run_skill("approach_machine", "three_group_espresso", params['group_number'], True)
    run_skill("mount_machine", "three_group_espresso", params['group_number'], True)

    if port == 'port_1':
        run_skill("moveEE", 0, 0, 3.5, 0, 0, 0)
    elif port == 'port_2':
        run_skill("moveEE", 0, 0, 5, 0, 0, 0)
    elif port == 'port_3':
        run_skill("moveEE", 0, 0, 7, 0, 0, 0)

    run_skill("release_tension")
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("enforce_rxry")
    time.sleep(0.6)
    run_skill("move_portafilter_arc", 47)
    run_skill("set_gripper_position", 255, 0)  # Open gripper
    run_skill("approach_machine", "three_group_espresso", params['portafilter_number'], True)
    run_skill("gotoJ_deg", *params['home'])  # Espresso home


def pull_espresso(port):
    """
    Executes the full pull-espresso sequence on the given port:
    1) unmount → 2) grind & tamp → 3) remount.
    """
    # Validate port up front
    if port not in PULL_ESPRESSO_PARAMS:
        print(f"[ERROR] unknown port: {port!r}")
        return

    for fn in (unmount, grinder, mount):
        print(f"--- running {fn.__name__}({port}) ---")
        fn(port)

def pull_espresso(port):
    """
    Executes the full pull-espresso sequence on the given port:
    1) unmount → 2) grind & tamp → 3) remount.
    """
    # Validate port up front
    if port not in PULL_ESPRESSO_PARAMS:
        print(f"[ERROR] unknown port: {port!r}")
        return

    for fn in (unmount, grinder, mount):
        print(f"--- running {fn.__name__}({port}) ---")
        fn(port)

def pick_pitcher(port):
    """Pick pitcher"""
    #---Pick the pitcher---#
    run_skill("gotoJ_deg", *Espresso_home) #Espresso home 
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    if port == 'port_1':
        run_skill("moveEE", 0, -240, 0, 0, 0, 0)
        run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
        run_skill("set_gripper_position", 255, 100)
        run_skill("moveEE", -103.413977, 0, 10, 0, 0, 0)
    elif port == 'port_2':
        run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
        run_skill("set_gripper_position", 255, 100)
        run_skill("gotoJ_deg", 23.034803,-44.195574,-116.188958,-19.410888,-66.975725,-0.169034, 1.0, 0.2)
    elif port == 'port_3':
        run_skill("moveEE", 0, 240, 0, 0, 0, 0)
        run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
        run_skill("set_gripper_position", 255, 100)
        run_skill("moveEE", -103.413977, 0, 10, 0, 0, 0)
    else:
        print(f"[ERROR] unknown port: {port!r}")
        return
    run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2) 

def pour_pitcher(stage):
    """
    Tilt the pitcher over the specified cup target.
    Cups must match keys in POUR_PARAMS: e.g. 'stage_1', 'stage_2'.
    """
    if stage not in ('stage_1', 'stage_2'):
        print(f"[ERROR] unknown stage: {stage!r}")
        return
    elif stage == 'stage_1':        
        run_skill("moveJ_deg", 90.160210, 10.716150, 0.203157, -10.883145, -0.001922, 0.060433, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 138.578024,-22.115324,-126.765855,-38.694157,-57.964712,3.173887, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 140.953509,-26.271451,-120.302336,-47.003274,-57.967889,-102.690158, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 138.578024,-22.115324,-126.765855,-38.694157,-57.964712,3.173887, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 121.236795,-29.537004,-136.110522,-14.093591,-58.933034,-0.146524)
        run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)
    else:
        run_skill("moveJ_deg", 90.160210, 10.716150, 0.203157, -10.883145, -0.001922, 0.060433, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 145.885393,-26.409357,-118.242817,-43.978546,-58.850444,-0.109599, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 145.462132,-32.355488,-108.068238,-53.572020,-58.845487,-105.385536, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 145.885393,-26.409357,-118.242817,-43.978546,-58.850444,-0.109599, 1.0, 0.2)
        time.sleep(0.3)
        run_skill("gotoJ_deg", 121.236795,-29.537004,-136.110522,-14.093591,-58.933034,-0.146524)
        run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)

def hot_water():
    """Get hot water in pitcher"""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Get hot water in pitcher 1---#
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint
    run_skill("moveEE", 0, 360, 15, 0, 0, 0) #move left to the hot water point
    run_skill("moveEE", 100, 0, 0, 0, 0, 0) #Move in
    time.sleep(8.0)
    run_skill("set_servo_timing", 0.20) #Reduce robot speed
    run_skill("moveEE", -100, 0, 0, 0, 0, 0) #Move out
    run_skill("gotoJ_deg", 34.671350,-38.978448,-145.993787,5.428431,-55.282231,-0.268345) #Move to midpoint

def return_pitcher(port):
    """Return pitcher to its home position."""
    #---Return the pitcher---#
    if port == 'port_1':
        run_skill("moveEE", 0, -240, 10, 0, 0, 0)
        run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
        run_skill("set_gripper_position", 75, 0)
        run_skill("moveEE", -103.413977, 0, -10, 0, 0, 0)
    elif port == 'port_2':
        run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
        run_skill("set_gripper_position", 75, 0) #pitcher gripper grip
    elif port == 'port_3':
        run_skill("moveEE", 0, 240, 10, 0, 0, 0)
        run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
        run_skill("set_gripper_position", 75, 0)
        run_skill("moveEE", -103.413977, 0, -10, 0, 0, 0)
    else:
        print(f"[ERROR] unknown port: {port!r}")
        return
    run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
    run_skill("gotoJ_deg", *Espresso_home) #Espresso home

# Register for CLI discovery
SEQUENCES = {
    'unmount': unmount,
    'grinder': grinder,
    'mount': mount,
    'pull_espresso': pull_espresso,
    'pick_pitcher': pick_pitcher,
    'pour_pitcher': pour_pitcher,
    'hot_water': hot_water,
    'return_pitcher': return_pitcher,
}