#!/usr/bin/env python3
import time
import argparse
from turtle import position
from manipulate_move_v1 import run_skill   # ← the only import you need
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

def home(**params):
    """
    Move robot to a predefined home position.
    
    This function moves the robot to one of several predefined home positions
    based on the position parameter. These home positions are safe, known
    configurations for different operational contexts.
    
    Args:
        position (str): Name of the home position to move to (must exist in HOME_ANGLES)
        
    Returns:
        bool: True if robot moved to home position successfully, False otherwise
        
    Example:
        success = home(position='espresso_home')
        if success:
            print("Robot moved to home position successfully")
    """
    try:
        position = params.get("position")
        angles = HOME_ANGLES.get(str(position))
        
        if not angles:
            print(f"[ERROR] unknown home position: {position!r}, available positions: {list(HOME_ANGLES.keys())}")
            return False
        
        print(f"🏠 Moving robot to home position: {position}")
        
        # Execute movement to home position
        result = run_skill("gotoJ_deg", *angles)
        if result is False:
            print(f"[ERROR] Failed to move robot to home position: {position}")
            return False
        
        print(f"✅ Robot successfully moved to home position: {position}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during home movement: {e}")
        return False

def get_machine_position(**params):
    """
    Calibrate and record machine positions for all coffee equipment.
    
    This function performs a comprehensive machine position calibration sequence:
    1. Moves to espresso home position
    2. Approaches and records portafilter cleaner position
    3. Approaches and records espresso grinder position  
    4. Approaches and records three-group espresso machine position
    5. Saves all position data for future reference
    
    This calibration should be performed when starting from a known home position
    and when machine positions may have changed.
    
    Returns:
        bool: True if all machine positions calibrated successfully, False otherwise
        
    Example:
        success = get_machine_position()
        if success:
            print("All machine positions calibrated successfully")
    """
    try:
        print("🎯 Starting comprehensive machine position calibration...")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home position...")
        home_result = run_skill("gotoJ_deg", 42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Position for portafilter cleaner calibration
        print("📍 Positioning for portafilter cleaner calibration...")
        cleaner_prep_result = run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
        if cleaner_prep_result is False:
            print("[ERROR] Failed to position for cleaner calibration")
            return False
        
        # Step 3: Perform multiple approaches to portafilter cleaner for accuracy
        print("🧹 Calibrating portafilter cleaner position (5 approaches)...")
        for i in range(5):
            print(f"   Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            approach_result = run_skill("move_to", "portafilter_cleaner", 0.12)
            if approach_result is False:
                print(f"[ERROR] Failed cleaner approach {i+1}/5")
                return False
        
        # Record portafilter cleaner position
        print("💾 Recording portafilter cleaner position...")
        cleaner_record_result = run_skill("get_machine_position", "portafilter_cleaner")
        if cleaner_record_result is False:
            print("[ERROR] Failed to record portafilter cleaner position")
            return False
        
        # Step 4: Position for espresso grinder calibration
        print("📍 Positioning for espresso grinder calibration...")
        grinder_prep1_result = run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
        if grinder_prep1_result is False:
            print("[ERROR] Failed to position for grinder calibration (step 1)")
            return False
        
        grinder_prep2_result = run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if grinder_prep2_result is False:
            print("[ERROR] Failed to position for grinder calibration (step 2)")
            return False
        
        # Step 5: Perform multiple approaches to espresso grinder for accuracy
        print("☕ Calibrating espresso grinder position (5 approaches)...")
        for i in range(5):
            print(f"   Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            grinder_approach_result = run_skill("move_to", "espresso_grinder", 0.12)
            if grinder_approach_result is False:
                print(f"[ERROR] Failed grinder approach {i+1}/5")
                return False
        
        # Record espresso grinder position
        print("💾 Recording espresso grinder position...")
        grinder_record_result = run_skill("get_machine_position", "espresso_grinder")
        if grinder_record_result is False:
            print("[ERROR] Failed to record espresso grinder position")
            return False
        
        # Step 6: Position for three-group espresso machine calibration
        print("📍 Positioning for three-group espresso machine calibration...")
        espresso_prep1_result = run_skill("gotoJ_deg", 7.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if espresso_prep1_result is False:
            print("[ERROR] Failed to position for espresso machine calibration (step 1)")
            return False
        
        espresso_prep2_result = run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)
        if espresso_prep2_result is False:
            print("[ERROR] Failed to position for espresso machine calibration (step 2)")
            return False
        
        # Step 7: Perform multiple approaches to three-group espresso machine for accuracy
        print("☕ Calibrating three-group espresso machine position (5 approaches)...")
        for i in range(5):
            print(f"   Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            espresso_approach_result = run_skill("move_to", "three_group_espresso", 0.12)
            if espresso_approach_result is False:
                print(f"[ERROR] Failed espresso machine approach {i+1}/5")
                return False
        
        # Record three-group espresso machine position
        print("💾 Recording three-group espresso machine position...")
        espresso_record_result = run_skill("get_machine_position", "three_group_espresso")
        if espresso_record_result is False:
            print("[ERROR] Failed to record three-group espresso machine position")
            return False
        
        # Step 8: Return to espresso home position
        print("🏠 Returning to espresso home position...")
        final_home_result = run_skill("gotoJ_deg", 42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if final_home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print("✅ Machine position calibration completed successfully!")
        print("   ✓ Portafilter cleaner position recorded")
        print("   ✓ Espresso grinder position recorded") 
        print("   ✓ Three-group espresso machine position recorded")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during machine position calibration: {e}")
        return False

GRAB_CUP_PARAMS = {
    '12oz': {
        'twist_back':   (-142.260873, -17.875853,  10.033241,   8.226858,  -0.089241, -47.128327),
        'approach':     ( 200,        0,          0,          0,          0,         0),
        'grip_width':   139,
        'retreat':      (-350,        0,          0,          0,          0,         0),
    },
    '9oz': {
        'twist_back':   ( -94.810739,  -3.781208,   0.171860,   4.133488,  -0.141411,   0.198394),
        'approach':     ( 284,         5,           -10,          0,          0,         0),
        'grip_width':   150,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
    '7oz': {
        'twist_back':   ( -65.440372, -10.652569,   4.188843,   6.867561,   0.095261,  29.626037),
        'approach':     ( 260,         15,           -10,          0,          0,         0),
        'grip_width':   150,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
}

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

def grab_cup(**params):
    """
    Grab a cup of specified size from the cup dispenser.
    
    This function performs the complete cup grabbing workflow:
    1. Moves to espresso home position
    2. Navigates to cup dispenser area (avoiding espresso machine)
    3. Positions for cup grab based on size parameters
    4. Executes approach, grip, and retreat sequence
    5. Returns to intermediate position ready for placement
    
    Args:
        size (str): Cup size to grab ('9oz', '12oz', etc.)
        
    Returns:
        bool: True if cup grabbed successfully, False otherwise
        
    Example:
        success = grab_cup(size='12oz')
        if success:
            print("Cup grabbed successfully")
    """
    try:
        size = params.get("size")
        cup_params = GRAB_CUP_PARAMS.get(str(size))
        
        # Validate parameters and use default if not found
        if not cup_params:
            print(f"[ERROR] unknown cup size: {size!r}, using default 12oz")
            cup_params = GRAB_CUP_PARAMS.get("12oz")
            if not cup_params:
                print("[ERROR] Default 12oz parameters not found in GRAB_CUP_PARAMS")
                return False
        
        print(f"🥤 Starting cup grab sequence for size: {size}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home position...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Twist to avoid hitting the espresso machine during navigation
        print("🔄 Navigating around espresso machine...")
        twist_result = run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0)
        if twist_result is False:
            print("[ERROR] Failed to twist around espresso machine")
            return False
        
        # Step 3: Move to cup grabbing area
        print("📍 Moving to cup dispenser area...")
        cup_area_result = run_skill("gotoJ_deg", 120.389030, 22.860609, -73.526848, -39.810959, 90.144394, -154.586288)
        if cup_area_result is False:
            print("[ERROR] Failed to move to cup dispenser area")
            return False
        
        # Step 4: Rotate joint angles to back away before approach
        print("⬅️ Backing away for approach...")
        if 'twist_back' in cup_params:
            twist_back_result = run_skill("moveJ_deg", *cup_params['twist_back'])
            if twist_back_result is False:
                print("[ERROR] Failed to execute twist back movement")
                return False
        
        # Step 5: Move end-effector into approach position
        print("🎯 Moving to approach position...")
        if 'approach' in cup_params:
            approach_result = run_skill("moveEE", *cup_params['approach'])
            if approach_result is False:
                print("[ERROR] Failed to move to approach position")
                return False
        
        # Step 6: Close gripper to grasp the cup
        print("🤏 Gripping cup...")
        if 'grip_width' in cup_params:
            grip_result = run_skill("set_gripper_position", 255, cup_params['grip_width'])
            if grip_result is False:
                print("[ERROR] Failed to grip cup")
                return False
        
        # Step 7: Retract after gripping
        print("⬆️ Retracting with cup...")
        if 'retreat' in cup_params:
            retreat_result = run_skill("moveEE", *cup_params['retreat'])
            if retreat_result is False:
                print("[ERROR] Failed to retreat with cup")
                return False
        
        # Step 8: Move to intermediate position ready for placement
        print("📍 Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        print(f"✅ Cup grab sequence completed successfully for size: {size}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup grab: {e}")
        return False

def place_cup(**params):
    """
    Place a cup at the specified staging area.
    
    This function places a previously grabbed cup at a designated serving stage:
    1. Moves from intermediate position to staging area
    2. Adjusts orientation for precise placement
    3. Positions cup at target location
    4. Releases cup and retreats safely
    5. Returns to espresso home position
    
    Args:
        stage (str): Target stage for cup placement ('stage_1', 'stage_2', etc.)
        
    Returns:
        bool: True if cup placed successfully, False otherwise
        
    Example:
        success = place_cup(stage='stage_1')
        if success:
            print("Cup placed successfully")
    """
    try:
        stage = params.get("stage")
        stage_params = PLACE_CUP_PARAMS.get(str(stage))
        
        # Validate stage parameter
        if not stage_params:
            print(f"[ERROR] unknown stage: {stage!r}, available stages: {list(PLACE_CUP_PARAMS.keys())}")
            return False
        
        print(f"📍 Starting cup placement sequence for: {stage}")
        
        # Step 1: Start from intermediate position
        print("📍 Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        # Step 2: Move into staging twist angle
        print("🔄 Adjusting orientation for staging...")
        if 'twist' in stage_params:
            twist_result = run_skill("moveJ_deg", *stage_params['twist'])
            if twist_result is False:
                print("[ERROR] Failed to execute staging twist")
                return False
        
        # Step 3: Move to target placement pose
        print("🎯 Moving to placement position...")
        if 'pose' in stage_params:
            pose_result = run_skill("gotoJ_deg", *stage_params['pose'])
            if pose_result is False:
                print("[ERROR] Failed to move to placement pose")
                return False
        
        # Step 4: Open gripper to release cup
        print("🤏 Releasing cup...")
        release_result = run_skill("set_gripper_position", 50, 0)
        if release_result is False:
            print("[ERROR] Failed to release cup")
            return False
        
        # Small delay to ensure cup is properly released
        time.sleep(1.0)
        
        # Step 5: Move up after placing cup
        print("⬆️ Moving up after placement...")
        up_result = run_skill("moveEE", 0, 0, 150, 0, 0, 0)
        if up_result is False:
            print("[ERROR] Failed to move up after placement")
            return False
        
        # Step 6: Move to staging home position
        print("🏠 Moving to staging home...")
        if 'stage_home' in stage_params:
            stage_home_result = run_skill("gotoJ_deg", *stage_params['stage_home'])
            if stage_home_result is False:
                print("[ERROR] Failed to move to staging home")
                return False
        
        # Step 7: Untwist back towards machine
        print("🔄 Untwisting back towards machine...")
        if 'twist_back' in stage_params:
            twist_back_result = run_skill("moveJ_deg", *stage_params['twist_back'])
            if twist_back_result is False:
                print("[ERROR] Failed to untwist back")
                return False
        
        # Step 8: Return to espresso home
        print("🏠 Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ Cup placement sequence completed successfully for: {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup placement: {e}")
        return False

def serve(**params):
    """
    Serve a cup from staging area to customer delivery point.
    
    This function handles the final step of drink service:
    1. Moves from espresso home to pickup position
    2. Grabs the prepared cup from staging
    3. Transports cup to customer delivery area
    4. Places cup for customer pickup
    5. Returns to espresso home position
    
    Args:
        stage (str): Source stage where cup is located ('stage_1', 'stage_2', etc.)
        
    Returns:
        bool: True if cup served successfully, False otherwise
        
    Example:
        success = serve(stage='stage_1')
        if success:
            print("Cup served successfully")
    """
    try:
        stage = params.get("stage")
        stage_params = PLACE_CUP_PARAMS.get(str(stage))
        
        # Validate stage parameter
        if not stage_params:
            print(f"[ERROR] unknown stage {stage!r}, available stages: {list(PLACE_CUP_PARAMS.keys())}")
            return False
        
        print(f"🚚 Starting cup serving sequence from: {stage}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Navigate to pickup position
        print("🔄 Navigating to pickup area...")
        if 'twist_serve' in stage_params:
            twist_serve_result = run_skill("moveJ_deg", *stage_params['twist_serve'])
            if twist_serve_result is False:
                print("[ERROR] Failed to execute serving twist")
                return False
        
        # Step 3: Move to cup pickup position
        print("📍 Moving to cup pickup position...")
        if 'pick' in stage_params:
            pick_result = run_skill("gotoJ_deg", *stage_params['pick'])
            if pick_result is False:
                print("[ERROR] Failed to move to pickup position")
                return False
        
        # Step 4: Lower to cup level
        print("⬇️ Lowering to cup level...")
        lower_result = run_skill("moveEE", 0, 0, -140, 0, 0, 0)
        if lower_result is False:
            print("[ERROR] Failed to lower to cup level")
            return False
        
        # Step 5: Grip the cup for serving
        print("🤏 Gripping cup for serving...")
        grip_result = run_skill("set_gripper_position", 55, 125)
        if grip_result is False:
            print("[ERROR] Failed to grip cup for serving")
            return False
        
        # Step 6: Set slower servo timing for careful handling
        print("⚙️ Setting careful servo timing...")
        timing_result = run_skill("set_servo_timing", 0.20)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing, continuing...")
        
        # Step 7: Lift cup
        print("⬆️ Lifting cup...")
        if 'pick' in stage_params:
            lift_result = run_skill("gotoJ_deg", *stage_params['pick'])
            if lift_result is False:
                print("[ERROR] Failed to lift cup")
                return False
        
        # Step 8: Move above serving area
        print("📍 Moving above serving area...")
        if 'above_serve' in stage_params:
            above_serve_result = run_skill("gotoJ_deg", *stage_params['above_serve'])
            if above_serve_result is False:
                print("[ERROR] Failed to move above serving area")
                return False
        
        # Step 9: Lower to serving position
        print("⬇️ Lowering to serving position...")
        if 'serve' in stage_params:
            serve_result = run_skill("gotoJ_deg", *stage_params['serve'])
            if serve_result is False:
                print("[ERROR] Failed to move to serving position")
                return False
        
        # Step 10: Release cup for customer
        print("🤏 Releasing cup for customer...")
        release_result = run_skill("set_gripper_position", 55, 0)
        if release_result is False:
            print("[ERROR] Failed to release cup")
            return False
        
        # Step 11: Reset servo timing
        print("⚙️ Resetting servo timing...")
        reset_timing_result = run_skill("set_servo_timing", 0.10)
        if reset_timing_result is False:
            print("[WARNING] Failed to reset servo timing, continuing...")
        
        # Small delay to ensure cup is properly placed
        time.sleep(1.0)
        
        # Step 12: Move up after placing
        print("⬆️ Moving up after serving...")
        up_result = run_skill("moveEE", 0, 0, 140, 0, 0, 0)
        if up_result is False:
            print("[ERROR] Failed to move up after serving")
            return False
        
        # Step 13: Return to staging home
        print("🏠 Moving to staging home...")
        staging_home_result = run_skill("gotoJ_deg", 106.460129, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if staging_home_result is False:
            print("[ERROR] Failed to move to staging home")
            return False
        
        # Step 14: Twist joint 1 to reach espresso home
        print("🔄 Twisting to reach espresso home...")
        final_twist_result = run_skill("moveJ_deg", -64.032688, 0, 0, 0, 0, 0)
        if final_twist_result is False:
            print("[ERROR] Failed to execute final twist")
            return False
        
        # Step 15: Return to espresso home
        print("🏠 Returning to espresso home...")
        final_home_result = run_skill("gotoJ_deg", *Espresso_home)
        if final_home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ Cup serving sequence completed successfully from: {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup serving: {e}")
        return False

def cold_cup(**params):
    """
    Grab a cold cup of specified size from the cold beverage cup dispenser.
    
    This function handles cold cup pickup for beverages like slushes and iced drinks:
    1. Moves to home position for safe approach
    2. Navigates to cold cup dispenser area
    3. Positions for cup grab based on size
    4. Grabs cup with appropriate grip strength
    5. Moves cup to safe position ready for beverage preparation
    
    The function supports multiple cup sizes with consistent grabbing sequence:
    - 7oz: Small cold cups for smaller portions
    - 9oz: Medium cold cups for standard servings  
    - 12oz: Large cold cups for generous servings
    - 16oz: Extra large cold cups for maximum capacity
    
    Args:
        cup_size (str): Size of cold cup to grab ('7oz', '9oz', '12oz', '16oz')
        
    Returns:
        bool: True if cold cup grabbed successfully, False otherwise
        
    Example:
        success = cold_cup(cup_size='12oz')
        if success:
            print("12oz cold cup grabbed successfully")
    """
    try:
        cup_size = params.get("cup_size")
        
        # Validate cup size parameter
        if cup_size not in ('16oz', '12oz', '9oz', '7oz'):
            print(f"[ERROR] unknown cup size: {cup_size!r}, must be '7oz', '9oz', '12oz', or '16oz'")
            return False
        
        print(f"🥤 Starting cold cup grab sequence for {cup_size}")
        
        # Step 1: Move to home position for safe approach
        print("🏠 Moving to west home position...")
        home_result = home(position="west")
        if home_result is False:
            print("[ERROR] Failed to move to west home position")
            return False
        
        # Step 2: Open gripper to prepare for cup grab
        print("🤏 Opening gripper for cup grab...")
        gripper_open_result = run_skill("set_gripper_position", 255, 0)
        if gripper_open_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        # Step 3: Move to cold cup dispenser area
        print("📍 Moving to cold cup dispenser area...")
        dispenser_result = run_skill("gotoJ_deg", 137.406860, 3.501065, -134.504471, -48.814426, -42.387501, -0.108438)
        if dispenser_result is False:
            print("[ERROR] Failed to move to cold cup dispenser area")
            return False
        
        # Step 4: Execute cup grabbing sequence (same for all sizes in this setup)
        print(f"🎯 Positioning for {cup_size} cup grab...")
        
        # Move to grab position
        grab_position_result = run_skill("moveEE", 5, 210, 10, 0, 0, 0)
        if grab_position_result is False:
            print("[ERROR] Failed to move to cup grab position")
            return False
        
        # Close gripper to grab cup
        print("🤏 Gripping cold cup...")
        grip_result = run_skill("set_gripper_position", 255, 140)
        if grip_result is False:
            print("[ERROR] Failed to grip cold cup")
            return False
        
        # Move down to extract cup from dispenser
        print("⬇️ Extracting cup from dispenser...")
        extract_result = run_skill("moveEE", 0, 0, -205, 0, 0, 0)
        if extract_result is False:
            print("[ERROR] Failed to extract cup from dispenser")
            return False
        
        # Step 5: Return to safe position with cup
        print("📍 Moving to safe position with cup...")
        safe_position_result = run_skill("gotoJ_deg", 137.406860, 3.501065, -134.504471, -48.814426, -42.387501, -0.108438)
        if safe_position_result is False:
            print("[ERROR] Failed to move to safe position with cup")
            return False
        
        print(f"✅ Cold cup grab completed successfully for {cup_size}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cold cup grab: {e}")
        return False
    
def place_cold_cup(**params):
    """
    Place a cold cup at specified staging area for cold beverage preparation.
    
    This function places a previously grabbed cold cup at a designated staging area:
    1. Moves to target staging position
    2. Lowers cup to placement level
    3. Releases cup with controlled opening
    4. Retracts safely after placement
    5. Returns to home position or continues workflow
    
    Args:
        cold_stage (str): Target staging area ('1' or '2')
        
    Returns:
        bool: True if cold cup placed successfully, False otherwise
        
    Example:
        success = place_cold_cup(cold_stage='1')
        if success:
            print("Cold cup placed at stage 1 successfully")
    """
    try:
        cold_stage = params.get("cold_stage")
        
        # Validate staging area parameter
        if cold_stage not in ('1', '2'):
            print(f"[ERROR] unknown cold stage: {cold_stage!r}, must be '1' or '2'")
            return False
        
        print(f"📍 Starting cold cup placement sequence for stage {cold_stage}")
        
        if cold_stage == '1':
            print("🎯 Executing stage 1 placement...")
            
            # Step 1: Move to stage 1 placement position
            print("📍 Moving to stage 1 position...")
            stage1_result = run_skill("gotoJ_deg", -111.215927, -19.601524, -91.144157, -68.881447, -114.195343, 0.046140)
            if stage1_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            
            # Step 2: Lower cup to placement level
            print("⬇️ Lowering cup to placement level...")
            lower_result = run_skill("moveEE", 0, 0, -315, 0, 0, 0)
            if lower_result is False:
                print("[ERROR] Failed to lower cup to placement level")
                return False
            
            # Step 3: Release cup
            print("🤏 Releasing cold cup...")
            release_result = run_skill("set_gripper_position", 60, 0)
            if release_result is False:
                print("[ERROR] Failed to release cold cup")
                return False
            
            # Step 4: Allow settling time
            time.sleep(0.5)
            
            # Step 5: Raise after placement
            print("⬆️ Moving up after placement...")
            raise_result = run_skill("moveEE", 0, 0, 315, 0, 0, 0)
            if raise_result is False:
                print("[ERROR] Failed to move up after placement")
                return False
            
            # Step 6: Return to home position
            print("🏠 Returning to east home position...")
            home_result = home(position="east")
            if home_result is False:
                print("[ERROR] Failed to return to east home position")
                return False
            
        elif cold_stage == '2':
            print("🎯 Executing stage 2 placement...")
            
            # Step 1: Move to stage 2 placement position
            print("📍 Moving to stage 2 position...")
            stage2_result = run_skill("gotoJ_deg", -121.922080, -29.170114, -76.511387, -73.910323, -124.911454, 0.116947)
            if stage2_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            
            # Step 2: Lower cup to placement level
            print("⬇️ Lowering cup to placement level...")
            lower_result = run_skill("moveEE", 0, 0, -315, 0, 0, 0)
            if lower_result is False:
                print("[ERROR] Failed to lower cup to placement level")
                return False
            
            # Step 3: Release cup
            print("🤏 Releasing cold cup...")
            release_result = run_skill("set_gripper_position", 60, 0)
            if release_result is False:
                print("[ERROR] Failed to release cold cup")
                return False
            
            # Step 4: Allow settling time
            time.sleep(0.5)
            
            # Step 5: Raise after placement
            print("⬆️ Moving up after placement...")
            raise_result = run_skill("moveEE", 0, 0, 315, 0, 0, 0)
            if raise_result is False:
                print("[ERROR] Failed to move up after placement")
                return False
            
            # Note: Stage 2 does not return to home - ready for next operation
        
        print(f"✅ Cold cup placement completed successfully for stage {cold_stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cold cup placement: {e}")
        return False

PULL_ESPRESSO_PARAMS = {
    'port_1': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_1",
        'group_number':         "group_1",
        'below_port':   (23.433992,-34.688675,-112.293900,-33.728462,-24.236135,0.000000),
        'move_back':   (-5.931169,-7.389562,-137.818634,-49.226776,-94.418251,0.010815),
    },
    'port_2': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_2",
        'group_number':         "group_2",
        'below_port':   (48.987930,-42.926491,-94.531250,-42.794937,4.182595,0.000000),
        'move_back':   (88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554),
    },
    'port_3': {
        'home':        ( 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379),
        'portafilter_number':         "portafilter_3",
        'group_number':         "group_3",
        'below_port':   (63.598370,-58.609657,-61.130093,-60.226448,15.355281,0.000000),
        'move_back':   (88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554),
    },
}

# Pre-defined home positions for espresso operations
Espresso_home = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
below_espresso_port = None
mount_espresso_port = None


def unmount(**params):
    """
    Unmount portafilter from espresso group for cleaning or grinding.
    
    This function performs the complete portafilter unmounting sequence:
    1. Moves to espresso home position
    2. Approaches and mounts to the specified portafilter group
    3. Closes gripper to secure portafilter
    4. Releases tension and adjusts orientation
    5. Rotates portafilter to unlock position
    6. Safely retracts and moves to clear path
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3')
        
    Returns:
        bool: True if portafilter unmounted successfully, False otherwise
        
    Example:
        success = unmount(port='port_1')
        if success:
            print("Portafilter unmounted successfully")
    """
    global below_espresso_port, mount_espresso_port
    try:
        port = params.get("port")
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] unknown port number: {port!r}, available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"📤 Starting portafilter unmount sequence for {port}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *port_params['home'])
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Approach the portafilter group
        print(f"🎯 Approaching portafilter {port_params['portafilter_number']}...")
        approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'], True)
        if approach_result is False:
            print("[ERROR] Failed to approach portafilter")
            return False
        
        # Step 3: Mount to the portafilter for secure grip
        print("🔧 Mounting to portafilter...")
        mount_result = run_skill("mount_machine", "three_group_espresso", port_params['portafilter_number'], True)
        if mount_result is False:
            print("[ERROR] Failed to mount to portafilter")
            return False
        
        # Step 4: Close gripper to secure portafilter
        print("🤏 Securing portafilter with gripper...")
        grip_result = run_skill("set_gripper_position", 255, 255)
        if grip_result is False:
            print("[ERROR] Failed to close gripper")
            return False
        
        # Step 5: Release tension for smooth operation
        print("😌 Releasing tension...")
        tension_result = run_skill("release_tension")
        if tension_result is False:
            print("[ERROR] Failed to release tension")
            return False
        
        # Step 6: Enforce proper orientation (first time)
        print("📐 Enforcing proper orientation...")
        orient_result1 = run_skill("enforce_rxry")
        if orient_result1 is False:
            print("[ERROR] Failed to enforce orientation (first attempt)")
            return False
        
        time.sleep(0.6)  # Allow settling time
        
        # Step 7: Enforce proper orientation (second time for stability)
        print("📐 Re-enforcing orientation for stability...")
        orient_result2 = run_skill("enforce_rxry")
        if orient_result2 is False:
            print("[ERROR] Failed to re-enforce orientation")
            return False
        
        time.sleep(0.6)  # Allow settling time
        
        # Step 8: Rotate portafilter to unlock (-45 degrees)
        print("🔄 Rotating portafilter to unlock...")
        rotate_result = run_skill("move_portafilter_arc", -45)
        if rotate_result is False:
            print("[ERROR] Failed to rotate portafilter")
            return False
        
        # Step 9: Release tension after rotation
        print("😌 Releasing tension after rotation...")
        tension_result2 = run_skill("release_tension")
        mount_espresso_port = run_skill("current_angles")  # Capture mount position
        if tension_result2 is False:
            print("[ERROR] Failed to release tension after rotation")
            return False
        
        # Step 10: Move end effector down to clear portafilter
        print("⬇️ Moving down to clear portafilter...")
        clear_result = run_skill("moveEE", 0, 0, -35, 0, 0, 0)
        below_espresso_port = run_skill("current_angles")  # Capture below position
        if clear_result is False:
            print("[ERROR] Failed to move down to clear portafilter")
            return False
        
        # Step 11: Move to position below port
        print("📍 Moving to position below port...")
        below_result = run_skill("gotoJ_deg", *port_params['below_port'])
        if below_result is False:
            print("[ERROR] Failed to move to position below port")
            return False
        
        # Step 12: Move back to avoid collisions
        print("⬅️ Moving back to avoid collisions...")
        back_result = run_skill("gotoJ_deg", *port_params['move_back'])
        if back_result is False:
            print("[ERROR] Failed to move back")
            return False
        
        # Step 13: Special handling for ports 2 and 3 (additional navigation)
        if port in ('port_2', 'port_3'):
            print("🔄 Executing special navigation for port 2/3...")
            nav1_result = run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
            if nav1_result is False:
                print("[ERROR] Failed special navigation step 1")
                return False
            
            nav2_result = run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
            if nav2_result is False:
                print("[ERROR] Failed special navigation step 2")
                return False
        
        print(f"✅ Portafilter unmount sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during unmount: {e}")
        return False

def grinder(**params):
    """
    Grind coffee and tamp portafilter at the grinder station.
    
    This function performs the complete grinding and tamping workflow:
    1. Moves to grinder home position
    2. Approaches grinder for coffee grinding
    3. Mounts to grinder to activate grinding
    4. Moves to tamper for coffee compaction
    5. Performs tamping motion
    6. Returns to grinder area and then home
    
    Args:
        port (str): Source port identifier (for parameter validation)
        
    Returns:
        bool: True if grinding and tamping completed successfully, False otherwise
        
    Example:
        success = grinder(port='port_1')
        if success:
            print("Coffee ground and tamped successfully")
    """
    try:
        port = params.get("port")
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] unknown port number: {port!r}, available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"☕ Starting grinding and tamping sequence for {port}")
        
        # Step 1: Move to grinder home position
        print("🏠 Moving to grinder home position...")
        home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        if home_result is False:
            print("[ERROR] Failed to move to grinder home")
            return False
        
        # Step 2: Approach the grinder
        print("🎯 Approaching grinder...")
        approach_result = run_skill("approach_machine", "espresso_grinder", "grinder", True)
        if approach_result is False:
            print("[ERROR] Failed to approach grinder")
            return False
        
        # Step 3: Mount to grinder to activate grinding
        print("⚙️ Mounting to grinder for grinding...")
        mount_result = run_skill("mount_machine", "espresso_grinder", "grinder", True)
        if mount_result is False:
            print("[ERROR] Failed to mount to grinder")
            return False
        
        # Step 4: Approach tamper station
        print("🎯 Approaching tamper...")
        tamper_approach_result = run_skill("approach_machine", "espresso_grinder", "tamper", True)
        if tamper_approach_result is False:
            print("[ERROR] Failed to approach tamper")
            return False
        
        time.sleep(1.0)  # Allow positioning time
        
        # Step 5: Mount to tamper for positioning
        print("📍 Positioning at tamper...")
        tamper_mount_result = run_skill("mount_machine", "espresso_grinder", "tamper", True)
        if tamper_mount_result is False:
            print("[ERROR] Failed to mount to tamper")
            return False
        
        # Step 6: Move up to prepare for tamping
        print("⬆️ Moving up to prepare for tamping...")
        up_result = run_skill("moveEE", 0, 0, 45, 0, 0, 0)
        if up_result is False:
            print("[ERROR] Failed to move up for tamping")
            return False
        
        time.sleep(1.0)  # Allow settling time
        
        # Step 7: Perform tamping motion (move down)
        print("🔨 Performing tamping motion...")
        tamp_result = run_skill("moveEE", 0, 0, -95, 0, 0, 0)
        if tamp_result is False:
            print("[ERROR] Failed to perform tamping motion")
            return False
        
        # Step 8: Return to grinder area
        print("🔄 Returning to grinder area...")
        return_grinder_result = run_skill("approach_machine", "espresso_grinder", "grinder", True)
        if return_grinder_result is False:
            print("[ERROR] Failed to return to grinder area")
            return False
        
        # Step 9: Return to grinder home
        print("🏠 Returning to grinder home...")
        final_home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        if final_home_result is False:
            print("[ERROR] Failed to return to grinder home")
            return False
        
        print(f"✅ Grinding and tamping sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during grinding: {e}")
        return False

def mount(**params):
    """
    Mount portafilter back to espresso group after grinding.
    
    This function performs the complete portafilter mounting sequence:
    1. Handles special navigation for ports 2 and 3
    2. Moves through safe path to target group
    3. Approaches and mounts to espresso group
    4. Adjusts position based on specific port requirements
    5. Performs orientation enforcement and locking rotation
    6. Opens gripper to release portafilter
    7. Returns to home position
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3')
        
    Returns:
        bool: True if portafilter mounted successfully, False otherwise
        
    Example:
        success = mount(port='port_1')
        if success:
            print("Portafilter mounted successfully")
    """
    try:
        port = params.get("port")
        port_params = PULL_ESPRESSO_PARAMS.get(str(port))
        
        if not port_params:
            print(f"[ERROR] unknown port number: {port!r}, available ports: {list(PULL_ESPRESSO_PARAMS.keys())}")
            return False
        
        print(f"📥 Starting portafilter mount sequence for {port}")
        
        # Step 1: Special handling for ports 2 and 3 (reverse navigation)
        if port in ('port_2', 'port_3'):
            print("🔄 Executing special navigation for port 2/3...")
            nav1_result = run_skill("moveJ_deg", 90, 0, 0, 0, 0, 0)
            if nav1_result is False:
                print("[ERROR] Failed special navigation step 1")
                return False
            
            nav2_result = run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
            if nav2_result is False:
                print("[ERROR] Failed special navigation step 2")
                return False
        
        # Step 2: Move to safe path position
        print("📍 Moving to safe path position...")
        back_result = run_skill("gotoJ_deg", *port_params['move_back'])
        if back_result is False:
            print("[ERROR] Failed to move to safe path position")
            return False
        
        # Step 3: Move to position below port
        print("📍 Moving to position below port...")
        below_result = run_skill("gotoJ_deg", *port_params['below_port'])
        if below_result is False:
            print("[ERROR] Failed to move to position below port")
            return False
        
        # Step 4: Approach the espresso group
        print(f"🎯 Approaching espresso group {port_params['group_number']}...")
        if below_espresso_port is None:
            print("[ERROR] below_espresso_port not captured, run unmount first")
            return False
        approach_result = run_skill("gotoJ_deg", *below_espresso_port)  # Use captured below position
        if approach_result is False:
            print("[ERROR] Failed to approach espresso group")
            return False
        
        # Step 5: Mount to espresso group
        print("🔧 Mounting to espresso group...")
        if mount_espresso_port is None:
            print("[ERROR] mount_espresso_port not captured, run unmount first")
            return False
        mount_result = run_skill("gotoJ_deg", *mount_espresso_port)  # Use captured mount position
        if mount_result is False:
            print("[ERROR] Failed to mount to espresso group")
            return False
        
        # Step 6: Adjust position based on specific port (fine-tuning)
        print(f"📐 Adjusting position for {port}...")
        if port == 'port_1':
            adjust_result = run_skill("moveEE", 0, 0, 3.5, 0, 0, 0)
        elif port == 'port_2':
            adjust_result = run_skill("moveEE", 0, 0, 5, 0, 0, 0)
        elif port == 'port_3':
            adjust_result = run_skill("moveEE", 0, 0, 7, 0, 0, 0)
        else:
            adjust_result = True  # No adjustment needed
        
        if adjust_result is False:
            print(f"[ERROR] Failed to adjust position for {port}")
            return False
        
        # Step 7: Release tension for smooth operation
        print("😌 Releasing tension...")
        tension_result = run_skill("release_tension")
        if tension_result is False:
            print("[ERROR] Failed to release tension")
            return False
        
        # Step 8: Enforce proper orientation (first time)
        print("📐 Enforcing proper orientation...")
        orient_result1 = run_skill("enforce_rxry")
        if orient_result1 is False:
            print("[ERROR] Failed to enforce orientation (first attempt)")
            return False
        
        time.sleep(0.6)  # Allow settling time
        
        # Step 9: Enforce proper orientation (second time for stability)
        print("📐 Re-enforcing orientation for stability...")
        orient_result2 = run_skill("enforce_rxry")
        if orient_result2 is False:
            print("[ERROR] Failed to re-enforce orientation")
            return False
        
        time.sleep(0.6)  # Allow settling time
        
        # Step 10: Rotate portafilter to lock position (+47 degrees)
        print("🔄 Rotating portafilter to lock...")
        rotate_result = run_skill("move_portafilter_arc", 47)
        if rotate_result is False:
            print("[ERROR] Failed to rotate portafilter to lock")
            return False
        
        # Step 11: Open gripper to release portafilter
        print("🤏 Opening gripper to release portafilter...")
        release_result = run_skill("set_gripper_position", 255, 0)
        if release_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        # Step 12: Move back to portafilter approach position
        print("⬅️ Moving back from portafilter...")
        back_approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'], True)
        if back_approach_result is False:
            print("[ERROR] Failed to move back from portafilter")
            return False
        
        # Step 13: Return to espresso home
        print("🏠 Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *port_params['home'])
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ Portafilter mount sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during mount: {e}")
        return False

def pick_pitcher(**params):
    """
    Pick up the milk pitcher for espresso operations.
    
    This function handles pitcher pickup with port-specific positioning:
    1. Moves to espresso home position
    2. Approaches pitcher pickup area
    3. Adjusts position based on the target port
    4. Grips pitcher with appropriate force
    5. Moves to intermediate position ready for pouring
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3') for position adjustment
        
    Returns:
        bool: True if pitcher picked successfully, False otherwise
        
    Example:
        success = pick_pitcher(port='port_1')
        if success:
            print("Pitcher picked successfully")
    """
    try:
        port = params.get("port")
        
        if port not in ('port_1', 'port_2', 'port_3'):
            print(f"[ERROR] unknown port: {port!r}, must be 'port_1', 'port_2', or 'port_3'")
            return False
        
        print(f"🥛 Starting pitcher pickup sequence for {port}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Approach pitcher pickup area
        print("🎯 Approaching pitcher pickup area...")
        approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
        if approach_result is False:
            print("[ERROR] Failed to approach pitcher pickup area")
            return False
        
        # Step 3: Port-specific positioning and pickup
        print(f"📍 Executing {port}-specific pickup sequence...")
        if port == 'port_1':
            # Port 1: Move to side position, extend, grip, retract
            move1_result = run_skill("moveEE", 0, -240, 0, 0, 0, 0)
            if move1_result is False:
                print("[ERROR] Failed to move to side position")
                return False
            
            extend_result = run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
            if extend_result is False:
                print("[ERROR] Failed to extend to pitcher")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip pitcher")
                return False
            
            retract_result = run_skill("moveEE", -103.413977, 0, 10, 0, 0, 0)
            if retract_result is False:
                print("[ERROR] Failed to retract with pitcher")
                return False
                
        elif port == 'port_2':
            # Port 2: Direct mount and grip
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
            if mount_result is False:
                print("[ERROR] Failed to mount to pitcher position")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip pitcher")
                return False
            
            lift_result = run_skill("gotoJ_deg", 23.034803, -44.195574, -116.188958, -19.410888, -66.975725, -0.169034, 1.0, 0.2)
            if lift_result is False:
                print("[ERROR] Failed to lift pitcher")
                return False
                
        elif port == 'port_3':
            # Port 3: Move to opposite side position, extend, grip, retract
            move1_result = run_skill("moveEE", 0, 240, 0, 0, 0, 0)
            if move1_result is False:
                print("[ERROR] Failed to move to opposite side position")
                return False
            
            extend_result = run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
            if extend_result is False:
                print("[ERROR] Failed to extend to pitcher")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip pitcher")
                return False
            
            retract_result = run_skill("moveEE", -103.413977, 0, 10, 0, 0, 0)
            if retract_result is False:
                print("[ERROR] Failed to retract with pitcher")
                return False
        
        # Step 4: Move to intermediate position ready for pouring
        print("📍 Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 31.076585, -40.253154, -136.313679, -3.210446, -58.931112, -0.206957, 1.0, 0.2)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        print(f"✅ Pitcher pickup sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during pitcher pickup: {e}")
        return False

def pour_pitcher(**params):
    """
    Pour contents from pitcher to cup at specified stage.
    
    This function performs the pouring sequence with stage-specific positioning:
    1. Moves to pouring preparation position
    2. Navigates to target stage area
    3. Tilts pitcher for controlled pouring
    4. Returns pitcher to upright position
    5. Moves back to intermediate position
    
    Args:
        stage (str): Target stage for pouring ('stage_1' or 'stage_2')
        
    Returns:
        bool: True if pouring completed successfully, False otherwise
        
    Example:
        success = pour_pitcher(stage='stage_1')
        if success:
            print("Pouring completed successfully")
    """
    try:
        stage = params.get("stage")
        
        if stage not in ('stage_1', 'stage_2'):
            print(f"[ERROR] unknown stage: {stage!r}, must be 'stage_1' or 'stage_2'")
            return False
        
        print(f"☕ Starting pouring sequence for {stage}")
        
        # Step 1: Move to pouring preparation position
        print("📍 Moving to pouring preparation position...")
        prep_result = run_skill("moveJ_deg", 90.160210, 10.716150, 0.203157, -10.883145, -0.001922, 0.060433, 1.0, 0.2)
        if prep_result is False:
            print("[ERROR] Failed to move to pouring preparation position")
            return False
        
        time.sleep(0.3)  # Allow settling time
        
        # Step 2: Stage-specific pouring sequence
        if stage == 'stage_1':
            print("🎯 Executing stage_1 pouring sequence...")
            
            # Move to stage_1 position
            pos1_result = run_skill("gotoJ_deg", 138.578024, -22.115324, -126.765855, -38.694157, -57.964712, 3.173887, 1.0, 0.2)
            if pos1_result is False:
                print("[ERROR] Failed to move to stage_1 position")
                return False
            
            time.sleep(0.3)
            
            # Tilt pitcher for pouring
            pour_result = run_skill("gotoJ_deg", 140.953509, -26.271451, -120.302336, -47.003274, -57.967889, -102.690158, 1.0, 0.2)
            if pour_result is False:
                print("[ERROR] Failed to tilt pitcher for pouring")
                return False
            
            time.sleep(0.3)
            
            # Return to upright position
            upright_result = run_skill("gotoJ_deg", 138.578024, -22.115324, -126.765855, -38.694157, -57.964712, 3.173887, 1.0, 0.2)
            if upright_result is False:
                print("[ERROR] Failed to return pitcher to upright")
                return False
            
        else:  # stage == 'stage_2'
            print("🎯 Executing stage_2 pouring sequence...")
            
            # Move to stage_2 position
            pos2_result = run_skill("gotoJ_deg", 145.885393, -26.409357, -118.242817, -43.978546, -58.850444, -0.109599, 1.0, 0.2)
            if pos2_result is False:
                print("[ERROR] Failed to move to stage_2 position")
                return False
            
            time.sleep(0.3)
            
            # Tilt pitcher for pouring
            pour2_result = run_skill("gotoJ_deg", 145.462132, -32.355488, -108.068238, -53.572020, -58.845487, -105.385536, 1.0, 0.2)
            if pour2_result is False:
                print("[ERROR] Failed to tilt pitcher for pouring")
                return False
            
            time.sleep(0.3)
            
            # Return to upright position
            upright2_result = run_skill("gotoJ_deg", 145.885393, -26.409357, -118.242817, -43.978546, -58.850444, -0.109599, 1.0, 0.2)
            if upright2_result is False:
                print("[ERROR] Failed to return pitcher to upright")
                return False
        
        time.sleep(0.3)
        
        # Step 3: Move to safe withdrawal position
        print("⬅️ Moving to safe withdrawal position...")
        withdraw_result = run_skill("gotoJ_deg", 121.236795, -29.537004, -136.110522, -14.093591, -58.933034, -0.146524)
        if withdraw_result is False:
            print("[ERROR] Failed to move to withdrawal position")
            return False
        
        # Step 4: Execute joint rotation
        print("🔄 Executing joint rotation...")
        rotate_result = run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
        if rotate_result is False:
            print("[ERROR] Failed to execute joint rotation")
            return False
        
        # Step 5: Return to intermediate position
        print("📍 Returning to intermediate position...")
        final_result = run_skill("gotoJ_deg", 31.076585, -40.253154, -136.313679, -3.210446, -58.931112, -0.206957, 1.0, 0.2)
        if final_result is False:
            print("[ERROR] Failed to return to intermediate position")
            return False
        
        print(f"✅ Pouring sequence completed successfully for {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during pouring: {e}")
        return False

def hot_water(**params):
    """
    Add hot water to pitcher for americano preparation.
    
    This function dispenses hot water into the pitcher:
    1. Moves to hot water dispenser area
    2. Positions pitcher under hot water nozzle
    3. Activates hot water dispenser (via timing delay)
    4. Returns to intermediate position with hot water added
    
    Returns:
        bool: True if hot water added successfully, False otherwise
        
    Example:
        success = hot_water()
        if success:
            print("Hot water added successfully")
    """
    try:
        print("🔥 Starting hot water addition sequence...")
        
        # Step 1: Move to hot water dispenser approach position
        print("📍 Moving to hot water dispenser approach...")
        approach_result = run_skill("gotoJ_deg", 67.341492, -49.798077, -99.061142, -30.809935, -22.613216, -0.457894)
        if approach_result is False:
            print("[ERROR] Failed to move to hot water approach position")
            return False
        
        # Step 2: Position pitcher under hot water nozzle
        print("🎯 Positioning pitcher under hot water nozzle...")
        position_result = run_skill("gotoJ_deg", 61.759601, -52.680407, -91.236745, -35.814578, -28.197699, -0.388979)
        if position_result is False:
            print("[ERROR] Failed to position pitcher under nozzle")
            return False
        
        # Step 3: Dispense hot water (timing-based activation)
        print("💧 Dispensing hot water (8 second duration)...")
        time.sleep(8.0)  # Hot water dispensing time
        
        # Step 4: Move back to approach position
        print("⬅️ Moving back from hot water nozzle...")
        back_result = run_skill("gotoJ_deg", 67.341492, -49.798077, -99.061142, -30.809935, -22.613216, -0.457894, 1.0, 0.2)
        if back_result is False:
            print("[ERROR] Failed to move back from hot water nozzle")
            return False
        
        # Step 5: Return to intermediate position
        print("📍 Returning to intermediate position...")
        final_result = run_skill("gotoJ_deg", 31.076585, -40.253154, -136.313679, -3.210446, -58.931112, -0.206957, 1.0, 0.2)
        if final_result is False:
            print("[ERROR] Failed to return to intermediate position")
            return False
        
        print("✅ Hot water addition sequence completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during hot water addition: {e}")
        return False

def return_pitcher(**params):
    """
    Return pitcher to its storage position after use.
    
    This function returns the pitcher with port-specific positioning:
    1. Adjusts position based on the source port
    2. Moves to pitcher return area
    3. Positions pitcher in storage location
    4. Releases pitcher grip
    5. Returns to espresso home position
    
    Args:
        port (str): Source port ('port_1', 'port_2', or 'port_3') for position adjustment
        
    Returns:
        bool: True if pitcher returned successfully, False otherwise
        
    Example:
        success = return_pitcher(port='port_1')
        if success:
            print("Pitcher returned successfully")
    """
    try:
        port = params.get("port")
        
        if port not in ('port_1', 'port_2', 'port_3'):
            print(f"[ERROR] unknown port: {port!r}, must be 'port_1', 'port_2', or 'port_3'")
            return False
        
        print(f"🔄 Starting pitcher return sequence for {port}")
        
        # Step 1: Port-specific return positioning
        print(f"📍 Executing {port}-specific return positioning...")
        if port == 'port_1':
            # Port 1: Move to side position, extend, release, retract
            move1_result = run_skill("moveEE", 0, -240, 10, 0, 0, 0)
            if move1_result is False:
                print("[ERROR] Failed to move to side position")
                return False
            
            extend_result = run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
            if extend_result is False:
                print("[ERROR] Failed to extend to return position")
                return False
            
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release pitcher")
                return False
            
            retract_result = run_skill("moveEE", -103.413977, 0, -10, 0, 0, 0)
            if retract_result is False:
                print("[ERROR] Failed to retract from pitcher")
                return False
                
        elif port == 'port_2':
            # Port 2: Direct mount and release
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
            if mount_result is False:
                print("[ERROR] Failed to mount to return position")
                return False
            
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release pitcher")
                return False
                
        elif port == 'port_3':
            # Port 3: Move to opposite side position, extend, release, retract
            move1_result = run_skill("moveEE", 0, 240, 10, 0, 0, 0)
            if move1_result is False:
                print("[ERROR] Failed to move to opposite side position")
                return False
            
            extend_result = run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
            if extend_result is False:
                print("[ERROR] Failed to extend to return position")
                return False
            
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release pitcher")
                return False
            
            retract_result = run_skill("moveEE", -103.413977, 0, -10, 0, 0, 0)
            if retract_result is False:
                print("[ERROR] Failed to retract from pitcher")
                return False
        
        # Step 2: Move to pitcher pickup approach area
        print("📍 Moving to pitcher approach area...")
        approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
        if approach_result is False:
            print("[ERROR] Failed to approach pitcher area")
            return False
        
        # Step 3: Return to espresso home
        print("🏠 Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ Pitcher return sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during pitcher return: {e}")
        return False

def clean(**params):
    """
    Complete portafilter cleaning sequence.
    
    This function performs a comprehensive cleaning workflow:
    1. Unmounts the portafilter from the specified espresso group
    2. Moves to the cleaning station
    3. Performs hard brush cleaning
    4. Performs soft brush cleaning  
    5. Remounts the portafilter back to the espresso group
    
    Args:
        port (str): The portafilter port to clean ('port_1', 'port_2', or 'port_3')
        
    Returns:
        bool: True if cleaning sequence completed successfully, False otherwise
        
    Example:
        success = clean(port='port_1')
        if success:
            print("Cleaning completed successfully")
    """
    try:
        port = params.get("port")
        
        # Validate port parameter
        if not port or port not in ('port_1', 'port_2', 'port_3'):
            print(f"[ERROR] invalid port: {port!r}, must be 'port_1', 'port_2', or 'port_3'")
            return False
        
        print(f"🧹 Starting cleaning sequence for {port}")
        
        # Step 1: Unmount the portafilter from espresso group
        print(f"📤 Unmounting portafilter from {port}...")
        unmount_result = unmount(port=port)
        if unmount_result is False:
            print(f"[ERROR] Failed to unmount portafilter from {port}")
            return False
        
        # Step 2: Move to cleaning station home position
        print("🏠 Moving to cleaning station...")
        home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        if home_result is False:
            print("[ERROR] Failed to move to cleaning station home")
            return False
        
        # Step 3: Perform hard brush cleaning
        print("🪥 Starting hard brush cleaning...")
        approach_result = run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
        if approach_result is False:
            print("[ERROR] Failed to approach hard brush")
            return False
            
        # Adjust position for better cleaning angle
        move_result = run_skill("moveEE", -88, 0, 0, 0, 0, -135)
        if move_result is False:
            print("[ERROR] Failed to adjust position for hard brush")
            return False
            
        # Mount to hard brush for cleaning
        mount_result = run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
        if mount_result is False:
            print("[ERROR] Failed to mount to hard brush")
            return False
            
        # Move up after hard brush cleaning
        up_result = run_skill("moveEE", 0, 0, 100, 0, 0, 0)
        if up_result is False:
            print("[ERROR] Failed to move up after hard brush")
            return False
        
        # Step 4: Perform soft brush cleaning
        print("🧽 Starting soft brush cleaning...")
        soft_approach_result = run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
        if soft_approach_result is False:
            print("[ERROR] Failed to approach soft brush")
            return False
            
        # Mount to soft brush for cleaning
        soft_mount_result = run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
        if soft_mount_result is False:
            print("[ERROR] Failed to mount to soft brush")
            return False
            
        # Move up after soft brush cleaning
        soft_up_result = run_skill("moveEE", 0, 0, 150, 0, 0, 0)
        if soft_up_result is False:
            print("[ERROR] Failed to move up after soft brush")
            return False
        
        # Step 5: Return to cleaning station home
        print("🏠 Returning to cleaning station home...")
        return_home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        if return_home_result is False:
            print("[ERROR] Failed to return to cleaning station home")
            return False
        
        # Step 6: Mount the portafilter back to espresso group
        print(f"📥 Remounting portafilter to {port}...")
        mount_result = mount(port=port)
        if mount_result is False:
            print(f"[ERROR] Failed to remount portafilter to {port}")
            return False
            
        print(f"✅ Cleaning sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cleaning: {e}")
        return False

# Global variables to store robot positions during milk frothing operations
# These are used to remember positions between function calls
approach_angles = None
grab_angles = None
froth_approach_angles = None
froth_mount_angles = None

def get_frother_position(**params):
    """
    Calibrate and record the milk frother position for future operations.
    
    This function performs calibration of the milk frother position:
    1. Moves to north-east home position for approach
    2. Opens gripper to prepare for positioning
    3. Performs multiple approaches to left steam wand for accuracy
    4. Records the calibrated position for future reference
    
    This calibration should be performed when setting up the milk frothing station
    or when the frother position may have changed.
    
    Returns:
        bool: True if frother position calibrated successfully, False otherwise
        
    Example:
        success = get_frother_position()
        if success:
            print("Frother position calibrated successfully")
    """
    try:
        print("🎯 Starting milk frother position calibration...")
        
        # Step 1: Move to home position for setup
        print("🏠 Moving to north-east home position...")
        home_result = home(position="north_east")
        if home_result is False:
            print("[ERROR] Failed to move to north-east home position")
            return False
        
        # Step 2: Open gripper to prepare for positioning
        print("🤏 Opening gripper for positioning...")
        gripper_result = run_skill("set_gripper_position", 255, 0)
        if gripper_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        # Step 3: Perform multiple approaches for accuracy
        print("🎯 Performing calibration approaches (5 attempts)...")
        for i in range(5):
            print(f"   Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            approach_result = run_skill("move_to", "left_steam_wand", 0.15, -10, -10)
            if approach_result is False:
                print(f"[ERROR] Failed calibration approach {i+1}/5")
                return False
        
        # Step 4: Record the calibrated position
        print("💾 Recording milk frother position...")
        record_result = run_skill("get_machine_position", "left_steam_wand")
        if record_result is False:
            print("[ERROR] Failed to record milk frother position")
            return False
        
        print("✅ Milk frother position calibration completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother position calibration: {e}")
        return False

def pick_frother(**params):
    """
    Pick up the milk frother for milk frothing operations.
    
    This function handles the milk frother pickup sequence:
    1. Moves to frother approach position
    2. Approaches the milk frother with precise positioning
    3. Grabs the frother with appropriate grip strength
    4. Stores position data for later return operations
    
    Returns:
        bool: True if frother picked successfully, False otherwise
        
    Example:
        success = pick_frother()
        if success:
            print("Milk frother picked successfully")
    """
    try:
        global approach_angles, grab_angles
        
        print("🥛 Starting milk frother pickup sequence...")
        
        # Step 1: Move to frother area position
        print("📍 Moving to frother area...")
        area_result = run_skill("gotoJ_deg", 20.847986, -21.981329, -113.153931, -76.829208, -81.786911, -0.050592)
        if area_result is False:
            print("[ERROR] Failed to move to frother area")
            return False
        
        # Step 2: Approach the milk frother
        print("🎯 Approaching milk frother...")
        approach_result = run_skill("move_to", 'milk_frother_1', 0.175)
        if approach_result is False:
            print("[ERROR] Failed to approach milk frother")
            return False
        
        # Step 3: Move to approach position for frother
        print("📍 Moving to frother approach position...")
        approach_tool_result = run_skill("approach_tool", 'milk_frother_1', 160)
        if approach_tool_result is False:
            print("[ERROR] Failed to move to frother approach position")
            return False
        
        # Step 4: Record current approach position
        print("💾 Recording approach position...")
        current_angles_result = run_skill("current_angles")
        if current_angles_result is not None:
            approach_angles = current_angles_result
            print(f"   Approach angles recorded: {approach_angles}")
        else:
            print("[WARNING] Failed to record approach angles - continuing without position memory")
            approach_angles = None
        
        # Step 5: Grab the frother
        print("🤏 Grabbing milk frother...")
        grab_result = run_skill("grab_tool", 'milk_frother_1', 200, 250, 255)
        if grab_result is False:
            print("[ERROR] Failed to grab milk frother")
            return False
        
        # Step 6: Record current grab position
        print("💾 Recording grab position...")
        grab_angles_result = run_skill("current_angles")
        if grab_angles_result is not None:
            grab_angles = grab_angles_result
            print(f"   Grab angles recorded: {grab_angles}")
        else:
            print("[WARNING] Failed to record grab angles - continuing without position memory")
            grab_angles = None
        
        print("✅ Milk frother pickup completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother pickup: {e}")
        return False
    
def froth_milk(**params):
    """
    Perform milk frothing operation using the steam wand.
    
    This function executes the milk frothing process:
    1. Moves to milk frothing preparation position
    2. Approaches the steam wand with the frother
    3. Mounts frother to steam wand for proper positioning
    4. Activates steam via digital output for frothing
    5. Allows frothing time then deactivates steam
    
    Returns:
        bool: True if milk frothing completed successfully, False otherwise
        
    Example:
        success = froth_milk()
        if success:
            print("Milk frothed successfully")
    """
    try:
        global froth_approach_angles, froth_mount_angles
        
        print("☁️ Starting milk frothing sequence...")
        
        # Step 1: Move to frothing preparation position
        print("📍 Moving to frothing preparation position...")
        prep_result = run_skill("gotoJ_deg", -4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837, 1.0, 0.2)
        if prep_result is False:
            print("[ERROR] Failed to move to frothing preparation position")
            return False
        
        time.sleep(0.2)  # Allow settling time
        
        # Step 2: Set slower servo timing for precise movements
        print("⚙️ Setting precise servo timing...")
        timing_result = run_skill("set_servo_timing", 0.2)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        
        # Step 3: Approach steam wand
        print("🎯 Approaching steam wand...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "milk_frother")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        
        # Record approach position
        froth_approach_result = run_skill("current_angles")
        if froth_approach_result is not None:
            froth_approach_angles = froth_approach_result
        
        time.sleep(0.2)
        
        # Step 4: Mount to steam wand for frothing
        print("🔧 Mounting to steam wand...")
        mount_result = run_skill("mount_machine", "left_steam_wand", "milk_frother")
        if mount_result is False:
            print("[ERROR] Failed to mount to steam wand")
            return False
        
        # Record mount position
        froth_mount_result = run_skill("current_angles")
        if froth_mount_result is not None:
            froth_mount_angles = froth_mount_result
        
        # Step 5: Activate steam for frothing
        print("💨 Activating steam for milk frothing...")
        steam_on_result = run_skill("set_DO", 2, 1)
        if steam_on_result is False:
            print("[ERROR] Failed to activate steam")
            return False
        
        # Step 6: Allow frothing time
        print("☁️ Frothing milk (10 seconds)...")
        time.sleep(10)
        
        # Step 7: Deactivate steam
        print("💨 Deactivating steam...")
        steam_off_result = run_skill("set_DO", 2, 0)
        if steam_off_result is False:
            print("[ERROR] Failed to deactivate steam")
            return False
        
        # Step 8: Allow settling time
        time.sleep(2)
        
        print("✅ Milk frothing completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk frothing: {e}")
        return False

def pour_milk(**params):
    """
    Pour frothed milk into cup at specified stage.
    
    This function pours the frothed milk with stage-specific positioning:
    1. Returns to approach position from steam wand
    2. Adjusts servo timing for smooth pouring
    3. Moves to target stage for milk pouring
    4. Tilts frother for controlled milk pour
    5. Returns frother to upright position
    
    Args:
        stage (str): Target stage for pouring ('1' or '2')
        
    Returns:
        bool: True if milk pouring completed successfully, False otherwise
        
    Example:
        success = pour_milk(stage='1')
        if success:
            print("Milk poured successfully")
    """
    try:
        stage = params.get("stage")
        
        # Validate stage parameter
        if stage not in ('1', '2'):
            print(f"[ERROR] unknown stage: {stage!r}, must be '1' or '2'")
            return False
        
        print(f"🥛 Starting milk pouring sequence for stage {stage}")
        
        # Step 1: Return to approach position from steam wand
        print("⬅️ Moving back from steam wand...")
        back_result = run_skill("approach_machine", "left_steam_wand", "milk_frother")
        if back_result is False:
            print("[ERROR] Failed to move back from steam wand")
            return False
        
        # Step 2: Set normal servo timing
        print("⚙️ Setting normal servo timing...")
        timing_result = run_skill("set_servo_timing", 0.1)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        
        time.sleep(0.2)
        
        # Step 3: Move to intermediate pouring position
        print("📍 Moving to intermediate pouring position...")
        intermediate_result = run_skill("gotoJ_deg", -53.498047, -56.063831, -104.329971, -23.914228, -67.359390, 3.238193, 1.0, 0.2)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        time.sleep(0.2)
        
        # Step 4: Stage-specific pouring sequence
        if stage == '1':
            print("🎯 Executing stage 1 milk pouring...")
            
            # Move to stage 1 position
            stage1_pos_result = run_skill("gotoJ_deg", -117.542499, -27.877248, -91.553736, -69.510481, -86.519990, 1.649929, 1.0, 0.2)
            if stage1_pos_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            
            time.sleep(0.2)
            
            # Tilt for pouring
            pour_result = run_skill("gotoJ_deg", -102.517232, -32.497384, -90.464555, -66.071945, -85.480721, -96.398044, 1.0, 0.075)
            if pour_result is False:
                print("[ERROR] Failed to tilt for pouring")
                return False
            
            # Allow pouring time
            print("🥛 Pouring milk (2 seconds)...")
            time.sleep(2)
            
            # Return to upright
            upright_result = run_skill("gotoJ_deg", -117.542499, -27.877248, -91.553736, -69.510481, -86.519990, 1.649929, 1.0, 0.2)
            if upright_result is False:
                print("[ERROR] Failed to return to upright position")
                return False
            
        elif stage == '2':
            print("🎯 Executing stage 2 milk pouring...")
            
            # Move to stage 2 position
            stage2_pos_result = run_skill("gotoJ_deg", -127.319954, -37.857658, -74.250511, -76.871681, -96.185387, 0.116908, 1.0, 0.2)
            if stage2_pos_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            
            time.sleep(0.2)
            
            # Tilt for pouring
            pour_result = run_skill("gotoJ_deg", -113.384514, -39.535606, -77.602524, -71.924614, -96.217064, -98.112167, 1.0, 0.075)
            if pour_result is False:
                print("[ERROR] Failed to tilt for pouring")
                return False
            
            # Allow pouring time
            print("🥛 Pouring milk (2 seconds)...")
            time.sleep(2)
            
            # Return to upright
            upright_result = run_skill("gotoJ_deg", -127.319954, -37.857658, -74.250511, -76.871681, -96.185387, 0.116908, 1.0, 0.2)
            if upright_result is False:
                print("[ERROR] Failed to return to upright position")
                return False
        
        time.sleep(0.2)
        
        print(f"✅ Milk pouring completed successfully for stage {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk pouring: {e}")
        return False
    
def return_frother(**params):
    """
    Return the milk frother to its storage position after use.
    
    This function returns the frother using stored position data:
    1. Moves to intermediate return position
    2. Uses stored grab position if available
    3. Opens gripper to release frother
    4. Uses stored approach position for safe withdrawal
    5. Returns to home position
    
    Returns:
        bool: True if frother returned successfully, False otherwise
        
    Example:
        success = return_frother()
        if success:
            print("Milk frother returned successfully")
    """
    try:
        global approach_angles, grab_angles
        
        print("🔄 Starting milk frother return sequence...")
        
        # Step 1: Move to intermediate return position
        print("📍 Moving to intermediate return position...")
        intermediate_result = run_skill("gotoJ_deg", -4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate return position")
            return False
        
        time.sleep(0.2)
        
        # Step 2: Move to return preparation position
        print("📍 Moving to return preparation position...")
        prep_result = run_skill("gotoJ_deg", 22.345373, -76.252151, -61.342220, -40.423759, -81.360077, 11.115391)
        if prep_result is False:
            print("[ERROR] Failed to move to return preparation position")
            return False
        
        # Step 3: Use stored grab position if available
        if grab_angles is not None and len(grab_angles) >= 6:
            print("📍 Moving to stored grab position...")
            grab_pos_result = run_skill("gotoJ_deg", *grab_angles)
            if grab_pos_result is False:
                print("[ERROR] Failed to move to stored grab position")
                return False
        else:
            print("[WARNING] No stored grab position available - using default positioning")
        
        # Step 4: Open gripper to release frother
        print("🤏 Opening gripper to release frother...")
        release_result = run_skill("set_gripper_position", 255, 160)
        if release_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        # Step 5: Use stored approach position if available
        if approach_angles is not None and len(approach_angles) >= 6:
            print("⬅️ Moving to stored approach position...")
            approach_pos_result = run_skill("gotoJ_deg", *approach_angles)
            if approach_pos_result is False:
                print("[ERROR] Failed to move to stored approach position")
                return False
        else:
            print("[WARNING] No stored approach position available - using default positioning")
        
        # Step 6: Return to home position
        print("🏠 Returning to home position...")
        home_result = home(position="north")
        if home_result is False:
            print("[ERROR] Failed to return to home position")
            return False
        
        # Step 7: Final gripper opening
        print("🤏 Final gripper opening...")
        final_grip_result = run_skill("set_gripper_position", 255, 0)
        if final_grip_result is False:
            print("[WARNING] Failed final gripper opening - frother may still be released")
        
        print("✅ Milk frother return completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother return: {e}")
        return False

def slush(**params):
    """
    Dispense slush from one of two available dispensers.
    
    This function controls the robot to operate slush dispensers for cold drinks:
    1. Moves to approach position for selected dispenser
    2. Positions for dispensing operation
    3. Activates dispenser through positioning sequence
    4. Returns to safe position after dispensing
    
    The robot supports two different slush dispensers with distinct positioning:
    - Dispenser 1: Standard positioning sequence
    - Dispenser 2: Extended reach positioning sequence
    
    Args:
        dispence (str): Dispenser selection ('1' or '2')
        
    Returns:
        bool: True if slush dispensing completed successfully, False otherwise
        
    Example:
        success = slush(dispence='1')
        if success:
            print("Slush dispensed successfully from dispenser 1")
    """
    try:
        dispence = params.get("dispence")
        
        # Validate dispenser parameter
        if dispence not in ('1', '2'):
            print(f"[ERROR] unknown dispence: {dispence!r}, must be '1' or '2'")
            return False
        
        print(f"🧊 Starting slush dispensing sequence for dispenser {dispence}")
        
        if dispence == '1':
            print("📍 Using dispenser 1 positioning sequence...")
            
            # Step 1: Move to approach position for dispenser 1
            print("🎯 Moving to dispenser 1 approach position...")
            approach_result = run_skill("gotoJ_deg", 27.351225, -41.244622, -131.515709, -6.958192, -152.611468, 0.0)
            if approach_result is False:
                print("[ERROR] Failed to move to dispenser 1 approach position")
                return False
            
            # Step 2: Move to dispensing position
            print("⬇️ Moving to dispensing position...")
            dispense_result = run_skill("gotoJ_deg", 63.173412, -59.329338, -104.639252, -17.729839, -112.244003, 0.0, 1.0, 0.2)
            if dispense_result is False:
                print("[ERROR] Failed to move to dispensing position")
                return False
            
            # Step 3: Allow dispensing time
            print("🧊 Dispensing slush (0.2 seconds)...")
            time.sleep(0.2)
            
            # Step 4: Return to approach position
            print("⬆️ Returning to approach position...")
            return_result = run_skill("gotoJ_deg", 27.351225, -41.244622, -131.515709, -6.958192, -152.611468, 0.0, 1.0, 0.2)
            if return_result is False:
                print("[ERROR] Failed to return to approach position")
                return False
            
            # Step 5: Final settling time
            time.sleep(0.2)
            
        elif dispence == '2':
            print("📍 Using dispenser 2 positioning sequence...")
            
            # Step 1: Move to approach position for dispenser 2
            print("🎯 Moving to dispenser 2 approach position...")
            approach_result = run_skill("gotoJ_deg", 27.351225, -41.244622, -131.515709, -6.958192, -152.611468, 0.0)
            if approach_result is False:
                print("[ERROR] Failed to move to dispenser 2 approach position")
                return False
            
            # Step 2: Move to intermediate position
            print("📍 Moving to intermediate position...")
            intermediate_result = run_skill("gotoJ_deg", 16.886827, -53.178346, -90.576631, -35.737992, -163.024408, 0.0)
            if intermediate_result is False:
                print("[ERROR] Failed to move to intermediate position")
                return False
            
            # Step 3: Move to dispensing position (extended reach)
            print("⬇️ Moving to extended dispensing position...")
            dispense_result = run_skill("gotoJ_deg", 45.045738, -70.610674, -67.497199, -43.970287, -130.385694, -0.683135, 1.0, 0.2)
            if dispense_result is False:
                print("[ERROR] Failed to move to extended dispensing position")
                return False
            
            # Step 4: Allow dispensing time
            print("🧊 Dispensing slush (0.2 seconds)...")
            time.sleep(0.2)
            
            # Step 5: Return to intermediate position
            print("⬆️ Returning to intermediate position...")
            return_intermediate_result = run_skill("gotoJ_deg", 16.886827, -53.178346, -90.576631, -35.737992, -163.024408, 0.0, 1.0, 0.2)
            if return_intermediate_result is False:
                print("[ERROR] Failed to return to intermediate position")
                return False
            
            # Step 6: Final settling time
            time.sleep(0.2)
        
        print(f"✅ Slush dispensing completed successfully from dispenser {dispence}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during slush dispensing: {e}")
        return False

def test(**params):
    get_machine_position()
    grab_cup(size="7oz")
    place_cup(stage="stage_1")
    unmount(port="port_1")
    grinder(port="port_1")
    mount(port="port_1")
    time.sleep(30)
    pick_pitcher(port="port_1")
    time.sleep(0.2)
    pour_pitcher(stage="stage_1")
    return_pitcher(port="port_1")
    clean(port="port_1")

# Add more sequences here as plain Python functions…


# ──────────────────────────────────────────────────────────────────
# 2)  LOOK-UP TABLE  (function-name ↔︎ human-friendly key)
# ──────────────────────────────────────────────────────────────────
SEQUENCES = {
    "home_north": lambda: home(position="north"),
    "home_north_east": lambda: home(position="north_east"),
    "home_east": lambda: home(position="east"),
    "home_south_east": lambda: home(position="south_east"),
    "home_south": lambda: home(position="south"),
    "home_south_west": lambda: home(position="south_west"),
    "home_west": lambda: home(position="west"),
    "home_north_west": lambda: home(position="north_west"),
    "get_machine_position": get_machine_position,
    
    # Unmount operations
    "unmount_port_1": lambda: unmount(port="port_1"),
    "unmount_port_2": lambda: unmount(port="port_2"),
    "unmount_port_3": lambda: unmount(port="port_3"),
    
    # Grinder operations 
    "grinder_port_1": lambda: grinder(port="port_1"),
    "grinder_port_2": lambda: grinder(port="port_2"),
    "grinder_port_3": lambda: grinder(port="port_3"),
    
    # Mount operations
    "mount_port_1": lambda: mount(port="port_1"),
    "mount_port_2": lambda: mount(port="port_2"),
    "mount_port_3": lambda: mount(port="port_3"),
    
    # Pitcher operations
    "pick_pitcher_port_1": lambda: pick_pitcher(port="port_1"),
    "pick_pitcher_port_2": lambda: pick_pitcher(port="port_2"),
    "pick_pitcher_port_3": lambda: pick_pitcher(port="port_3"),
    "pour_pitcher_cup_1": lambda: pour_pitcher(stage="stage_1"),
    "pour_pitcher_cup_2": lambda: pour_pitcher(stage="stage_2"),
    "return_pitcher_port_1": lambda: return_pitcher(port="port_1"),
    "return_pitcher_port_2": lambda: return_pitcher(port="port_2"),
    "return_pitcher_port_3": lambda: return_pitcher(port="port_3"),
    
    # Cleaning operations
    "clean_port_1": lambda: clean(port="port_1"),
    "clean_port_2": lambda: clean(port="port_2"),
    "clean_port_3": lambda: clean(port="port_3"),
    
    # Hot water
    "hot_water": hot_water,
    
    # Cup operations
    "grab_cup_12": lambda: grab_cup(size="12oz"),
    "grab_cup_9": lambda: grab_cup(size="9oz"),
    "grab_cup_7": lambda: grab_cup(size="7oz"),
    "place_cup_1": lambda: place_cup(stage="stage_1"),
    "place_cup_2": lambda: place_cup(stage="stage_2"),
    "serve_cup_1": lambda: serve(stage="stage_1"),
    "serve_cup_2": lambda: serve(stage="stage_2"),
    
    # Cold cup operations  
    "cold_cup_7oz": lambda: cold_cup(cup_size="7oz"),
    "cold_cup_9oz": lambda: cold_cup(cup_size="9oz"),
    "cold_cup_12oz": lambda: cold_cup(cup_size="12oz"),
    "cold_cup_16oz": lambda: cold_cup(cup_size="16oz"),
    "place_cold_cup_1": lambda: place_cold_cup(cold_stage="1"),
    "place_cold_cup_2": lambda: place_cold_cup(cold_stage="2"),
    
    # Slush operations
    "slush_1": lambda: slush(dispence="1"),
    "slush_2": lambda: slush(dispence="2"),
    
    # Milk frother operations
    "get_frother_position": get_frother_position,
    "pick_frother": pick_frother,
    "froth_milk": froth_milk,
    "pour_milk_stage_1": lambda: pour_milk(stage="1"),
    "pour_milk_stage_2": lambda: pour_milk(stage="2"),
    "return_frother": return_frother,
    
    # Test and utility
    "test": lambda: test(port="port_1", stage="stage_1", size="7oz"),
    "say_hi": say_hi,

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
