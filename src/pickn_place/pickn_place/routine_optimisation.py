#!/usr/bin/env python3
import time
from manipulate_move_v2 import run_skill   # ← the only import you need

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
Espresso_home = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)

def home(**params):
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
        for i in range(3):
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
        for i in range(3):
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
        for i in range(3):
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

GRAB_PAPER_CUP_PARAMS = {
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
        'approach':     ( 263.5,         13.5,           -10,          0,          0,         0),
        'grip_width':   153,
        'retreat':      (-300,         0,           0,          0,          0,         0),
    },
}

PLACE_PAPER_CUP_PARAMS = {
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

def grab_paper_cup(**params):
    """
    Grab a paper cup of specified size from the paper cup dispenser.
    
    This function performs the complete paper cup grabbing workflow:
    1. Moves to espresso home position
    2. Navigates to paper cup dispenser area (avoiding espresso machine)
    3. Positions for paper cup grab based on size parameters
    4. Executes approach, grip, and retreat sequence
    5. Returns to intermediate position ready for placement
    
    Args:
        size (str): Paper cup size to grab ('9oz', '12oz', etc.)
        
    Returns:
        bool: True if paper cup grabbed successfully, False otherwise
        
    Example:
        success = grab_paper_cup(size='12oz')
        if success:
            print("Paper cup grabbed successfully")
    """
    try:
        size = params.get("size")
        cup_params = GRAB_PAPER_CUP_PARAMS.get(str(size))
        
        # Validate parameters and use default if not found
        if not cup_params:
            print(f"[ERROR] unknown paper cup size: {size!r}, using default 12oz")
            cup_params = GRAB_PAPER_CUP_PARAMS.get("7oz")
            if not cup_params:
                print("[ERROR] Default 12oz parameters not found in GRAB_PAPER_CUP_PARAMS")
                return False
        
        print(f"🥤 Starting paper cup grab sequence for size: {size}")
        
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
        
        # Step 3: Move to paper cup grabbing area
        print("📍 Moving to paper cup dispenser area...")
        cup_area_result = run_skill("gotoJ_deg", 120.389030, 22.860609, -73.526848, -39.810959, 90.144394, -154.586288)
        if cup_area_result is False:
            print("[ERROR] Failed to move to paper cup dispenser area")
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
        
        # Step 6: Close gripper to grasp the paper cup
        print("🤏 Gripping paper cup...")
        if 'grip_width' in cup_params:
            grip_result = run_skill("set_gripper_position", 255, cup_params['grip_width'])
            if grip_result is False:
                print("[ERROR] Failed to grip paper cup")
                return False
        
        # Step 7: Retract after gripping
        print("⬆️ Retracting with paper cup...")
        if 'retreat' in cup_params:
            retreat_result = run_skill("moveEE", *cup_params['retreat'])
            if retreat_result is False:
                print("[ERROR] Failed to retreat with paper cup")
                return False
        
        # Step 8: Move to intermediate position ready for placement
        print("📍 Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        print(f"✅ Paper cup grab sequence completed successfully for size: {size}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup grab: {e}")
        return False

def place_paper_cup(**params):
    try:
        stage = params.get("stage")
        stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
        
        # Validate stage parameter
        if not stage_params:
            print(f"[ERROR] unknown stage: {stage!r}, available stages: {list(PLACE_PAPER_CUP_PARAMS.keys())}")
            return False
        
        print(f"📍 Starting paper cup placement sequence for: {stage}")
        
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
        
        # Step 4: Open gripper to release paper cup
        print("🤏 Releasing paper cup...")
        release_result = run_skill("set_gripper_position", 50, 0)
        if release_result is False:
            print("[ERROR] Failed to release paper cup")
            return False
        
        # Small delay to ensure paper cup is properly released
        time.sleep(1.0)
        
        # Step 5: Move up after placing paper cup
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
        
        print(f"✅ Paper cup placement sequence completed successfully for: {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup placement: {e}")
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
        
        # # Step 2: Approach the portafilter group
        # print(f"🎯 Approaching portafilter {port_params['portafilter_number']}...")
        # approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'], True)
        # if approach_result is False:
        #     print("[ERROR] Failed to approach portafilter")
        #     return False
        
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
    """
    try:        
        print(f"☕ Starting grinding and tamping sequence")
        
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
        
        time.sleep(2.0)  # Allow positioning time

        # Step 5: Mount to grinder to activate grinding
        print("⚙️ Mounting to grinder for grinding...")
        mount_result = run_skill("mount_machine", "espresso_grinder", "grinder", True)
        if mount_result is False:
            print("[ERROR] Failed to mount to grinder")
            return False
        
        # Step 6: Mount to tamper for positioning
        print("📍 Positioning at tamper...")
        tamper_mount_result = run_skill("mount_machine", "espresso_grinder", "tamper", True)
        if tamper_mount_result is False:
            print("[ERROR] Failed to mount to tamper")
            return False

        # Step 7: Open gripper
        open_gripper = run_skill("set_gripper_position", 255, 0)
        if open_gripper is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        print(f"✅ Grinding and tamping sequence completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during grinding: {e}")
        return False
    
def tamper(**params):
    """
    Tamp coffee at the tamper station.
    
    This function performs the complete tamping sequence:
    1. Moves to tamper home position
    2. Approaches tamper station
    3. Mounts to tamper for positioning
    4. Returns to grinder home
    """
    try:
        # Step 1: Close gripper
        close_gripper = run_skill("set_gripper_position", 255, 255)
        run_skill("moveEE", 0, 0, 10, 0, 0, 0)
        if close_gripper is False:
            print("[ERROR] Failed to close gripper")
            return False
        # Step 2
        mount_result = run_skill("mount_machine", "espresso_grinder", "grinder", True)
        if mount_result is False:
            print("[ERROR] Failed to mount to grinder")
            return False
        # Step 3:
        approach_result = run_skill("approach_machine", "espresso_grinder", "grinder", True)
        if approach_result is False:
            print("[ERROR] Failed to approach grinder")
            return False

        # Step 3: Return to grinder home
        print("🏠 Returning to grinder home...")
        final_home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        if final_home_result is False:
            print("[ERROR] Failed to return to grinder home")
            return False
        
        print(f"✅ Grinding and tamping sequence completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during tamping: {e}")
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
        
        # # Step 12: Move back to portafilter approach position
        # print("⬅️ Moving back from portafilter...")
        # back_approach_result = run_skill("approach_machine", "three_group_espresso", port_params['portafilter_number'], True)
        # if back_approach_result is False:
        #     print("[ERROR] Failed to move back from portafilter")
        #     return False
        
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

def pick_espresso_pitcher(**params):
    try:
        port = params.get("port")
        
        if not port:
            print("[ERROR] No port specified for espresso pitcher selection")
            return False
        
        print(f"🥛 Starting espresso pitcher pickup sequence for {port}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Approach espresso pitcher area
        print("🎯 Approaching espresso pitcher area...")
        approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
        if approach_result is False:
            print("[ERROR] Failed to approach espresso pitcher area")
            return False
        
        # Step 3: Pick espresso_pitcher based on port
        print(f"🤏 Picking espresso_pitcher for {port}...")
        if port == 'port_1':
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
            if approach_result is False:
                print("[ERROR] Failed to approach espresso_pitcher 1")
                return False
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
            if mount_result is False:
                print("[ERROR] Failed to mount espresso_pitcher 1")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            run_skill("set_servo_timing",0.2)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso_pitcher 1")
                return False
            
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
            if retreat_result is False:
                print("[ERROR] Failed to retreat from espresso_pitcher 1")
                return False
                
        elif port == 'port_2':
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
            if mount_result is False:
                print("[ERROR] Failed to mount espresso_pitcher 2")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            run_skill("set_servo_timing",0.2)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso_pitcher 2")
                return False
            
            pos_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
            if pos_result is False:
                print("[ERROR] Failed to position for espresso_pitcher 2")
                return False
                
        elif port == 'port_3':
            move1_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
            if move1_result is False:
                print("[ERROR] Failed to move to espresso_pitcher 3 position 1")
                return False
            
            move2_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3", True)
            if move2_result is False:
                print("[ERROR] Failed to move to espresso_pitcher 3 position 2")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            run_skill("set_servo_timing",0.2)
            if grip_result is False:
                print("[ERROR] Failed to grip espresso_pitcher 3")
                return False
            
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
            if retreat_result is False:
                print("[ERROR] Failed to retreat with espresso_pitcher 3")
                return False
        else:
            print(f"[ERROR] unknown port: {port!r}, available ports: port_1, port_2, port_3")
            return False
        
        # Step 4: Move to final position
        print("📍 Moving to final position...")
        run_skill("set_servo_timing",0.1)
        final_result = run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)
        time.sleep(0.2)
        if final_result is False:
            print("[ERROR] Failed to move to final position")
            return False
        
        print(f"✅ espresso_pitcher pickup sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during espresso_pitcher pickup: {e}")
        return False    

def pour_espresso_pitcher(**params):
    try:
        stage = params.get("stage")
        
        if not stage:
            print("[ERROR] No stage specified for pouring")
            return False
        
        if stage not in ('stage_1', 'stage_2'):
            print(f"[ERROR] unknown stage: {stage!r}, available stages: stage_1, stage_2")
            return False
        
        print(f"🥛 Starting milk pouring sequence for {stage}")
        
        # Step 1: Initial positioning
        print("📍 Moving to initial pouring position...")
        init_result = run_skill("moveJ_deg", 90.160210, 10.716150, 0.203157, -10.883145, -0.001922, 0.060433, 1.0, 0.2)
        if init_result is False:
            print("[ERROR] Failed to move to initial pouring position")
            return False
        
        time.sleep(0.3)
        
        if stage == 'stage_1':
            print("🎯 Positioning for stage 1 pouring...")
            
            # Step 2: Approach stage 1 position
            pos1_result = run_skill("gotoJ_deg", 138.578024,-22.115324,-126.765855,-38.694157,-57.964712,3.173887, 1.0, 0.2)
            if pos1_result is False:
                print("[ERROR] Failed to approach stage 1 position")
                return False
            
            time.sleep(0.3)
            
            # Step 3: Tilt espresso_pitcher to pour
            print("⬇️ Tilting espresso_pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", 140.953509,-26.271451,-120.302336,-47.003274,-57.967889,-102.690158, 1.0, 0.2)
            if pour_result is False:
                print("[ERROR] Failed to tilt espresso_pitcher for pouring")
                return False
            
            time.sleep(0.3)
            
            # Step 4: Return to neutral position
            print("⬆️ Returning to neutral position...")
            neutral_result = run_skill("gotoJ_deg", 138.578024,-22.115324,-126.765855,-38.694157,-57.964712,3.173887, 1.0, 0.2)
            if neutral_result is False:
                print("[ERROR] Failed to return to neutral position")
                return False
            
            time.sleep(0.3)
            
        else:  # stage_2
            print("🎯 Positioning for stage 2 pouring...")
            
            # Step 2: Approach stage 2 position
            pos2_result = run_skill("gotoJ_deg", 145.885393,-26.409357,-118.242817,-43.978546,-58.850444,-0.109599, 1.0, 0.2)
            if pos2_result is False:
                print("[ERROR] Failed to approach stage 2 position")
                return False
            
            time.sleep(0.3)
            
            # Step 3: Tilt espresso_pitcher to pour
            print("⬇️ Tilting espresso_pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", 145.462132,-32.355488,-108.068238,-53.572020,-58.845487,-105.385536, 1.0, 0.2)
            if pour_result is False:
                print("[ERROR] Failed to tilt espresso_pitcher for pouring")
                return False
            
            time.sleep(0.3)
            
            # Step 4: Return to neutral position
            print("⬆️ Returning to neutral position...")
            neutral_result = run_skill("gotoJ_deg", 145.885393,-26.409357,-118.242817,-43.978546,-58.850444,-0.109599, 1.0, 0.2)
            if neutral_result is False:
                print("[ERROR] Failed to return to neutral position")
                return False
            
            time.sleep(0.3)
        
        # Step 5: Move to intermediate position
        print("📍 Moving to intermediate position...")
        inter_result = run_skill("gotoJ_deg", 121.236795,-29.537004,-136.110522,-14.093591,-58.933034,-0.146524)
        if inter_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        # Step 7: Return to holding position
        print("🏠 Returning to holding position...")
        final_result = run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957)
        if final_result is False:
            print("[ERROR] Failed to return to holding position")
            return False
        
        print(f"✅ Milk pouring sequence completed successfully for {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during pouring: {e}")
        return False

def get_hot_water(**params):
    try:      
        # Step 1: Move to hot water dispenser approach position
        print("🎯 Approaching hot water dispenser...")
        approach_result = run_skill("gotoJ_deg", 67.341492,-49.798077,-99.061142,-30.809935,-22.613216,-0.457894)
        if approach_result is False:
            print("[ERROR] Failed to approach hot water dispenser")
            return False
        
        # Step 2: Position espresso_pitcher under hot water outlet
        print("📍 Positioning espresso_pitcher under hot water outlet...")
        position_result = run_skill("gotoJ_deg", 61.759601,-52.680407,-91.236745,-35.814578,-28.197699,-0.388979)
        if position_result is False:
            print("[ERROR] Failed to position espresso_pitcher under hot water outlet")
            return False
        
        print(f"✅ Hot water dispensing sequence completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during hot water dispensing: {e}")
        return False

def with_hot_water(**params):
    try:
        # Step 1: Move away from hot water outlet
        print("⬆️ Moving away from hot water outlet...")
        retreat_result = run_skill("gotoJ_deg", 67.341492,-49.798077,-99.061142,-30.809935,-22.613216,-0.457894, 1.0, 0.2)
        if retreat_result is False:
            print("[ERROR] Failed to move away from hot water outlet")
            return False
        
        # Step 2: Return to holding position
        print("🏠 Returning to holding position...")
        final_result = run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)
        if final_result is False:
            print("[ERROR] Failed to return to holding position")
            return False
        
        print(f"✅ Hot water dispensing sequence completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during hot water dispensing: {e}")
        return False


def return_espresso_pitcher(**params):
    try:
        port = params.get("port")
        
        if not port:
            print("[ERROR] No port specified for espresso_pitcher return")
            return False
        
        print(f"🔄 Starting espresso_pitcher return sequence for {port}")
        
        # Step 1: Return espresso_pitcher based on port
        if port == 'port_1':
            print("🎯 Approaching espresso_pitcher 1 return position...")
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
            if approach_result is False:
                print("[ERROR] Failed to approach espresso_pitcher 1 return position")
                return False
            
            print("📍 Positioning espresso_pitcher 1 for return...")
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
            if mount_result is False:
                print("[ERROR] Failed to position espresso_pitcher 1 for return")
                return False
            
            print("🤏 Releasing espresso_pitcher 1...")
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release espresso_pitcher 1")
                return False
            
            print("⬅️ Retreating from espresso_pitcher 1...")
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
            if retreat_result is False:
                print("[ERROR] Failed to retreat from espresso_pitcher 1")
                return False
                
        elif port == 'port_2':
            print("📍 Positioning espresso_pitcher 2 for return...")
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
            if mount_result is False:
                print("[ERROR] Failed to position espresso_pitcher 2 for return")
                return False
            
            print("🤏 Releasing espresso_pitcher 2...")
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release espresso_pitcher 2")
                return False
                
        elif port == 'port_3':
            print("📍 Moving to espresso_pitcher 3 return position...")
            move1_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
            if move1_result is False:
                print("[ERROR] Failed to move to espresso_pitcher 3 return position 1")
                return False
            
            move2_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_3", True)
            if move2_result is False:
                print("[ERROR] Failed to move to espresso_pitcher 3 return position 2")
                return False
            
            print("🤏 Releasing espresso_pitcher 3...")
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release espresso_pitcher 3")
                return False
            
            print("⬅️ Retreating from espresso_pitcher 3...")
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_3", True)
            if retreat_result is False:
                print("[ERROR] Failed to retreat from espresso_pitcher 3")
                return False
        else:
            print(f"[ERROR] unknown port: {port!r}, available ports: port_1, port_2, port_3")
            return False
        
        # Step 2: Move to common espresso_pitcher area
        print("🎯 Moving to espresso_pitcher area...")
        area_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
        if area_result is False:
            print("[ERROR] Failed to move to espresso_pitcher area")
            return False
        
        # Step 3: Return to espresso home
        print("🏠 Returning to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        print(f"✅ espresso_pitcher return sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during espresso_pitcher return: {e}")
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

approach_angles = None
grab_angles = None

def get_frother_position(**params):
    try:
        print("🎯 Starting milk frother position calibration...")
        
        # Step 1: Move to home position for setup
        print("🏠 Moving to north-east home position...")
        home(position="north_east")
        home_result = run_skill("gotoJ_deg", -67.357964,-23.709629,-89.522377,-84.038696,-113.021690,8.468687)
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
        for i in range(3):
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
        approach_tool_result = run_skill("approach_tool", 'milk_frother_1', 169)
        if approach_tool_result is False:
            print("[ERROR] Failed to move to frother approach position")
            return False
        
        # Step 4: Record current approach position
        print("💾 Recording approach position...")
        current_angles = run_skill("current_angles")
        if current_angles is not None:
            approach_angles = current_angles
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
        grab_angles = run_skill("current_angles")
        if grab_angles is not None:
            print(f"   Grab angles recorded: {grab_angles}")
        else:
            print("[WARNING] Failed to record grab angles - continuing without position memory")
            grab_angles = None
        
        print("✅ Milk frother pickup completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother pickup: {e}")
        return False
    
def mount_frother(**params):
    """
    Mount the milk frother to the steam wand for frothing preparation.
    
    This function positions the frother on the steam wand:
    1. Moves to milk frothing preparation position
    2. Sets precise servo timing for accurate movements
    3. Approaches the steam wand with the frother (deep position)
    4. Fine approaches the steam wand (light position)
    5. Mounts frother securely to steam wand
    
    Note: This function only positions the frother. Use froth_milk() to actually activate steam.
    
    Returns:
        bool: True if frother mounted successfully, False otherwise
        
    Example:
        success = mount_frother()
        if success:
            print("Frother mounted to steam wand successfully")
    """
    try:
        
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
        
        # Step 3: Approach steam wand (deep position)
        print("🎯 Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth", True)
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
                
        # Step 4: Fine approach to steam wand (light position) 
        # print("🎯 Fine approaching steam wand (light position)...")
        # fine_approach_result = run_skill("approach_machine", "left_steam_wand", "light_froth", True)
        # if fine_approach_result is False:
        #     print("[ERROR] Failed to fine approach steam wand")
        #     return False

        # Step 5: Mount frother to steam wand
        print("🔧 Mounting frother to steam wand...")
        mount_result = run_skill("mount_machine", "left_steam_wand", "deep_froth", True)
        if mount_result is False:
            print("[ERROR] Failed to mount frother to steam wand")
            return False
        
        print("✅ Milk frother mounted to steam wand successfully")
        return True
               
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk frothing: {e}")
        return False

def froth_milk(**params):
    try:
        duration = params.get("duration", 10)
        print(f"🥛 Frothing milk for {duration} seconds...")
        # Step 5: Activate steam for frothing
        print("💨 Activating steam for milk frothing...")
        steam_on_result = run_skill("set_DO", 2, 1)
        if steam_on_result is False:
            print("[ERROR] Failed to activate steam")
            return False
        
        # Step 6: Allow frothing time
        print("☁️ Frothing milk (10 seconds)...")
        time.sleep(duration)
        
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
         # Step 5: Mount frother to steam wand
        print("🔧 Mounting frother to steam wand...")
        mount_result = run_skill("mount_machine", "left_steam_wand", "deep_froth", True)
        if mount_result is False:
            print("[ERROR] Failed to mount frother to steam wand")
            return False
        # Step 4: Fine approach to steam wand (light position) 
        # print("🎯 Fine approaching steam wand (light position)...")
        # fine_approach_result = run_skill("approach_machine", "left_steam_wand", "light_froth", True)
        # if fine_approach_result is False:
        #     print("[ERROR] Failed to fine approach steam wand")
        #     return False
        # Step 3: Approach steam wand (deep position)
        print("🎯 Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth", True)
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        # Step 2: Set normal servo timing
        print("⚙️ Setting normal servo timing...")
        timing_result = run_skill("set_servo_timing", 0.1)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        
        # print("📍 Moving to intermediate pouring position...")
        # intermediate_result = run_skill("gotoJ_deg", -58.476021,-51.300709,-101.058250,-42.526070,-61.983448,8.514315, 1.0, 0.2)
        # if intermediate_result is False:
        #     print("[ERROR] Failed to move to intermediate position")
        #     return False
        # time.sleep(0.2)
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
            pour_result = run_skill("gotoJ_deg", -102.517232, -32.497384, -90.464555, -66.071945, -85.480721, -96.398044, 1.0, 0.005)
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
        release_result = run_skill("set_gripper_position", 255, 165)
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

def espresso_sequence(**params):
    grab_paper_cup(size="7oz")
    place_paper_cup(stage="stage_1")
    unmount(port="port_2")
    grinder()
    time.sleep(27.2)
    tamper()
    mount(port="port_2")
    time.sleep(3.0)
    pick_espresso_pitcher(port="port_2")
    pour_espresso_pitcher(stage="stage_1")
    return_espresso_pitcher(port="port_2")

def milk_sequence(**params):
    get_frother_position()
    pick_frother()
    mount_frother()
    froth_milk(duration=10)
    pour_milk(stage="1")
    return_frother()

def random_shi():
    run_skill("gotoJ_deg", 22.402670,-79.481049,-59.269863,-39.324520,-81.417374,11.115391)
    

# ──────────────────────────────────────────────────────────────────
# 2)  LOOK-UP TABLE  (function-name ↔︎ human-friendly key)
# ──────────────────────────────────────────────────────────────────
SEQUENCES = {
    'get_machine_position': get_machine_position,
    'espresso_sequence': espresso_sequence,
    'milk_sequence': milk_sequence,
    'random_shi': random_shi,
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
