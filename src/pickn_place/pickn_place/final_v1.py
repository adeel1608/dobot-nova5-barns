#!/usr/bin/env python3
"""
sequences.py  –  high-level routines ***and*** a tiny CLI to run one.

Usage examples
--------------
python3 sequences.py teach
python3 sequences.py mount --loop      # repeat until Ctrl-C
python3 sequences.py --list            # show available names
"""
import time
import argparse
from manipulate_move_new import run_skill   # ← the only import you need
import rclpy
from rclpy.node import Node  # Add this import
from dobot_msgs_v3.srv import ServoJ
import re

def load_commands_from_file(path):
    """
    Read `say_hi.txt`, extract every sequence of seven floats from
    ServoJ(j1,j2,j3,j4,j5,j6,t=...), and return a list of tuples.
    """
    pattern = re.compile(r'ServoJ\(\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*t=([-\d\.]+)\)')
    cmds = []
    with open(path, 'r') as f:
        for line in f:
            m = pattern.search(line)
            if m:
                vals = tuple(float(m.group(i)) for i in range(1,8))
                cmds.append(vals)
    return cmds

class ServoJCommander(Node):
    def __init__(self):
        super().__init__('servoj_commander')
        self.cli = self.create_client(ServoJ, '/dobot_bringup_v3/srv/ServoJ')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for ServoJ service...')
        self.get_logger().info('Service ready.')

    def send_one(self, j1,j2,j3,j4,j5,j6,t):
        req = ServoJ.Request()
        req.j1, req.j2, req.j3 = j1, j2, j3
        req.j4, req.j5, req.j6 = j4, j5, j6
        req.t = t
        self.get_logger().info(f'Sending: {j1:.3f},{j2:.3f},{j3:.3f},{j4:.3f},{j5:.3f},{j6:.3f} @ t={t}')
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error(f'Failed: {fut.exception()}')

# ──────────────────────────────────────────────────────────────────
# 1)  PUT EACH ROUTINE INTO A FUNCTION
# ──────────────────────────────────────────────────────────────────
HOME_ANGLES = {
    'north':       (   0, 30, -130, -100,  -90,    0),
    'north_east': ( -45, 30, -130, -100,  -90,    0),
    'east':       ( -90, 30, -130, -100,  -90,    0),
    'south_east': (-135, 30, -130, -100,  -90,    0),
    'south':      ( 180, 30, -130, -100,  -90,    0),
    'south_west': ( 135, 30, -130, -100,  -90,    0),
    'west':       (  90, 30, -130, -100,  -90,    0),
    'north_west': (  45, 30, -130, -100,  -90,    0),
}
def home(position):
    """
    Move the robot to one of the predefined home orientations.
    
    Args:
        position (str): One of 'north', 'north_east', 'east',
                        'south_east', 'south', 'south_west',
                        'west', 'north_west'.
    """
    angles = HOME_ANGLES.get(position)
    if not angles:
        print(f"[ERROR] unknown home position: {position!r}")
        return

    print(f"--- refreshing and moving to {position} home ---")
    run_skill("refresh_position")
    time.sleep(2.0)
    run_skill("gotoJ_deg", *angles)
    print(f"--- arrived at {position} home ---")

def say_hi():
    #---Extra waving mode lol---#
    # Create and initialize the ServoJ commander
    rclpy.init()
    node = ServoJCommander()
    
    # Load and execute the ServoJ commands
    commands = load_commands_from_file('/home/adeel/barns_ws/src/pickn_place/pickn_place/say_hi.txt')
    for j1,j2,j3,j4,j5,j6,t in commands:
        node.send_one(j1,j2,j3,j4,j5,j6,0.1)
        time.sleep(0.06)
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

GRAB_CUP_PARAMS = {
    '12oz': {
        'twist_back':   (-142.260873, -17.875853,  10.033241,   8.226858,  -0.089241, -47.128327),
        'approach':     ( 200,        0,          0,          0,          0,         0),
        'grip_width':   139,
        'retreat':      (-350,        0,          0,          0,          0,         0),
    },
    '9oz': {
        'twist_back':   ( -94.810739,  -3.781208,   0.171860,   4.133488,  -0.141411,   0.198394),
        'approach':     ( 250,         5,           -10,          0,          0,         0),
        'grip_width':   144,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
    '7oz': {
        'twist_back':   ( -65.440372, -10.652569,   4.188843,   6.867561,   0.095261,  29.626037),
        'approach':     ( 215,         15,           -10,          0,          0,         0),
        'grip_width':   140,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
}
# ─── CUP PLACEMENT / SERVE PARAMETERS ────────────────────────────────────────────
# twist:       initial joint-1 twist into staging area
# pose:        joint targets to move into place over cup
# stage_home:  safe “staging” home pose
# twist_back:  untwist back towards machine
# twist_serve: joint-1 twist to present cup
# pick/above_serve/serve:
#   intermediate poses for the serve_cup() routine
PLACE_CUP_PARAMS = {
    'stage_1': {
        'twist':        (  66,   0,   0,   0,    0,    0),
        'pose':         ( 154.125778,-52.670326,-116.881065,-10.357515,-25.897715,0.057296),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
    'stage_2': {
        # identical to stage_1 for now; tweak if you need a different trajectory
        'twist':        (  69,   0,   0,   0,    0,    0),
        'pose':         ( 159.539737,-56.095770,-101.541641,-22.208326,-20.521090,0.048888),
        'stage_home':   (106.460129,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379),
        'twist_back':   (-64.032688,   0,        0,         0,         0,         0),
        'twist_serve':  ( 31.983258,   0,        0,         0,         0,         0),
        'pick':         ( 74.423628, -24.537379, -125.119809, -29.957357, -105.529320,   0.111478),
        'above_serve':  (114.123176, -17.424376, -139.079935, -23.095763,  -65.826354,  -0.154358),
        'serve':        (114.103052, -48.360801, -140.051323,   8.823533,  -65.854705,  -0.182434),
    },
}

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

def milk_frothing():
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0, 1.0, 0.2)
    for i in range(3):
        time.sleep(1.0)
        run_skill("move_to", "left_steam_wand", 0.12)
    run_skill("get_machine_position", "left_steam_wand")
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0)
    run_skill("gotoJ_deg", 10.067266,-47.009254,-129.773193,-3.069591,-79.829887,-0.112283)
    run_skill("moveEE", 0, 20, -160, 0, 0, 0)
    run_skill("moveEE", 88, 0, 0, 0, 0, 0)
    run_skill("set_gripper_position", 255, 95)
    # run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("moveEE", -77, 0, 20, 0, 0, 0)
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0, 1.0, 0.2)
    run_skill("approach_machine", "left_steam_wand", "deep_froth", True)
    run_skill("mount_machine", "left_steam_wand", "deep_froth", True)
    run_skill("mount_machine", "left_steam_wand", "light_froth", True)
    run_skill("approach_machine", "left_steam_wand", "light_froth", True)
    run_skill("moveEE", -30, 30, -30, 0, 0, 0)
    run_skill("gotoJ_deg", -42.960346,-27.362787,-119.737602,-32.588718,-42.844364, 0.0, 1.0, 0.2)
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, 100, 1.0, 0.2) #Tilt the pitcher back
    time.sleep(5.0) #Random wait time
    run_skill("moveJ_deg", 0, 0, 0, 0, 0, -100)
    run_skill("gotoJ_deg", -45, 30, -130, -100, -90, 0)
    run_skill("gotoJ_deg", 10.067266,-47.009254,-129.773193,-3.069591,-79.829887,-0.112283)
    run_skill("moveEE", 0, 20, -160, 0, 0, 0)
    run_skill("moveEE", 88, 0, 0, 0, 0, 0)
    # run_skill("set_gripper_position", 255, 95)
    run_skill("set_gripper_position", 75, 0) #Release pitcher slowly
    run_skill("moveEE", -77, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 0, 30, -130, -100, -90, 0)

def serve_cup():
     #Pickup from staging place in delivery
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("moveJ_deg", 31.983258, 0, 0, 0, 0, 0)
    run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478)
    run_skill("moveEE", 0, 0, -140, 0, 0, 0)
    run_skill("set_gripper_position", 55, 125)
    time.sleep(1.0)
    run_skill("gotoJ_deg", 74.423628,-24.537379,-125.119809,-29.957357,-105.529320,0.111478, 1.0, 0.2)
    run_skill("gotoJ_deg", 114.123176,-17.424376,-139.079935,-23.095763,-65.826354,-0.154358, 1.0, 0.2)
    run_skill("gotoJ_deg", 114.103052,-48.360801,-140.051323,8.823533,-65.854705,-0.182434, 1.0, 0.2)
    run_skill("set_gripper_position", 55, 0)
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 140, 0, 0, 0)
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def grab_cup_1():
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #-----------------------------------------------------Get a cup-----------------------------------#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -142.260873, -17.875853, 10.033241, 8.226858, -0.089241, -47.128327) #Twist back to align with the cups
    run_skill("moveEE", 200, 0, 0, 0, 0, 0)
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -350, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 130, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    # # #--------------------------------------------------Grab pitcher----------------------------------#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def grab_cup_2():
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #-----------------------------------------------------Get a cup-----------------------------------#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -94.891786, -3.735069, 0.159065, 4.110916, -0.085106, 0.174636) #Twist back to align with the cups
    run_skill("gotoJ_deg", 14.608274,-10.505052,-46.947823,-32.527233,90.108307,-165.236038) #Current 12oz cup position
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -200, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    # # #--------------------------------------------------Grab pitcher----------------------------------#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def grab_cup_3():
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    #---Unmount the portafilter---#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    #-----------------------------------------------------Get a cup-----------------------------------#
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chose for grabbing the cup
    run_skill("moveJ_deg", -59.968582, -15.674384, 7.995781, 8.130459, 0.019112, 35.111396) #Twist back to align with the cups
    run_skill("moveEE", 200, 0, 0, 0, 0, 0)
    run_skill("set_gripper_position", 255, 139) #12oz gripper size
    run_skill("moveEE", -350, 0, 0, 0, 0, 0) #Remove the cup
    run_skill("moveJ_deg", 50, 0, 0, 0, 0, 0) #Twist joint 1 to go to staging area
    run_skill("gotoJ_deg", 74.410699,-48.038242,-125.889557,-5.691980,-105.547663,0.091964) #Cup placing position
    run_skill("set_gripper_position", 50, 0) #Open gripper slowly
    time.sleep(1.0)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0) #Go up
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Staging home
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) #Twist joint 1 to reach espresso home
    # # #--------------------------------------------------Grab pitcher----------------------------------#
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def get_machine_position():
    """Machine position updates here (when starting from home position)."""
    run_skill("refresh_position")
    time.sleep(5.0) #Random wait time
    run_skill("set_servo_timing", 0.1)
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("moveJ_deg", -39, 0, 0, 0, 0, 0)
    for i in range(3):
        time.sleep(1.0)
        run_skill("move_to", "espresso_grinder", 0.12)
    run_skill("get_machine_position", "espresso_grinder")
    run_skill("gotoJ_deg", 7.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)
    for i in range(3):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.12)
    run_skill("get_machine_position", "three_group_espresso")
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 

def random_tests():
    run_skill("refresh_position")
    time.sleep(2.0) #Random wait time
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chosen for grabbing the cup
    # Rotate joint angles to back away before approach
    run_skill("moveJ_deg", -65.440372, -10.652569,   4.188843,   6.867561,   0.095261,  29.626037)
    # Move end-effector into approach position
    run_skill("moveEE", 215,         15,           -10,          0,          0,         0)
    # Close gripper to grasp the cup
    run_skill("set_gripper_position", 255, 140)
    # Retract after gripping
    run_skill("moveEE", -300,         0,           0,          0,          0,         0)
    run_skill("gotoJ_deg", 88.657143,21.041538,-74.451630,-36.522381,90.145508,-91.183128)
    run_skill("moveJ_deg", 69.012928,   0,   0,   0,    0,    0)
    # Move to target placement pose
    run_skill("gotoJ_deg", 159.539737,-56.095770,-101.541641,-22.208326,-20.521090,0.048888)
    # Open gripper to release cup
    run_skill("set_gripper_position", 50, 0)
    time.sleep(1.0)
    run_skill("moveEE", 0,0,150,0,0,0) # Move up after placing cup
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) # Move to staging home position
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) # Untwist back towards machine
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home
    ####################################################################################################################################
    run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0) #Twist to avoid hitting the espresso machine
    run_skill("gotoJ_deg", 120.389030,22.860609,-73.526848,-39.810959,90.144394,-154.586288) #Pose chosen for grabbing the cup
    # Rotate joint angles to back away before approach
    run_skill("moveJ_deg", -94.810739,  -3.781208,   0.171860,   4.133488,  -0.141411,   0.198394)
    # Move end-effector into approach position
    run_skill("moveEE", 250,         5,           -10,          0,          0,         0)
    # Close gripper to grasp the cup
    run_skill("set_gripper_position", 255, 144)
    # Retract after gripping
    run_skill("moveEE", -300,         0,           0,          0,          0,         0)
    run_skill("gotoJ_deg", 88.657143,21.041538,-74.451630,-36.522381,90.145508,-91.183128)
    run_skill("moveJ_deg", 66.012928,   0,   0,   0,    0,    0)
    # Move to target placement pose
    run_skill("gotoJ_deg", 154.125778,-52.670326,-116.881065,-10.357515,-25.897715,0.057296)
    # Open gripper to release cup
    run_skill("set_gripper_position", 50, 0)
    time.sleep(1.0)
    run_skill("moveEE", 0,0,150,0,0,0) # Move up after placing cup
    run_skill("gotoJ_deg", 106.460129,13.883821,-133.648376,-81.024788,-49.533218,13.894379) # Move to staging home position
    run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0) # Untwist back towards machine
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home
# Add more sequences here as plain Python functions…

# ──────────────────────────────────────────────────────────────────
# 2)  LOOK-UP TABLE  (function-name ↔︎ human-friendly key)
# ──────────────────────────────────────────────────────────────────
SEQUENCES = {
    "home": home_north,
    "home_north_east": home_north_east,
    "home_west":home_west,
    "espresso_port_1": espresso_port_1,
    "espresso_port_2": espresso_port_2,
    "espresso_port_3": espresso_port_3,
    "americano_port_1": americano_port_1,
    "americano_port_2": americano_port_2,
    "americano_port_3": americano_port_3,
    "clean_port_1": clean_port_1,
    "clean_port_2": clean_port_2,
    "clean_port_3": clean_port_3,
    "milk_frothing": milk_frothing,
    "grab_cup_1": grab_cup_1,
    "grab_cup_2": grab_cup_2,
    "grab_cup_3": grab_cup_3,
    "serve_cup": serve_cup,
    "get_machine_position": get_machine_position,
    "random_tests": random_tests,
    # "demo": demo_raw,
}

# ------------------------------------------------------------------
#  CLI – interactive menu that keeps prompting until you quit
# ------------------------------------------------------------------
def _main():
    print("🔧  Pick-and-Place Interactive Menu")
    print("Type the name to run a sequence, or 'q' to quit.\n")

    while True:
        # 1) show the current list
        print("Available sequences:")
        for name in SEQUENCES:
            print(f"  • {name}")

        # 2) prompt the user
        choice = input("\nWhich sequence? (q to exit) ").strip().lower()

        if choice in ("q", "quit", "exit"):
            print("Bye!")
            break

        if choice not in SEQUENCES:
            print(f"❌  '{choice}' is not a valid sequence. Try again.\n")
            continue

        # 3) run the chosen sequence *once*
        try:
            SEQUENCES[choice]()        # ← call the function
        except KeyboardInterrupt:
            print("\n⏹️  Interrupted. Returning to menu.\n")
        else:
            print("\n✅  Finished. Back to menu.\n")


if __name__ == "__main__":
    _main()
