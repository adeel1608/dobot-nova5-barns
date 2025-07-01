"""
espresso.py

Defines the espresso-making sequence for different ports and cups.
"""

import time
from oms_v1.manipulate_node import run_skill
from oms_v1.params import PULL_ESPRESSO_PARAMS, POUR_PARAMS

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
    Pick up milk pitcher for the specified port.
    
    This function picks up the appropriate milk pitcher based on the port selection:
    - Moves to espresso home position
    - Navigates to the correct pitcher location
    - Grips the pitcher with appropriate force
    - Positions for subsequent operations
    
    Args:
        port (str): Target port ('port_1', 'port_2', or 'port_3')
        
    Returns:
        bool: True if pitcher picked successfully, False otherwise
        
    Example:
        success = pick_pitcher(port='port_1')
        if success:
            print("Pitcher picked successfully")
    """
    try:
        port = params.get("port")
        
        if not port:
            print("[ERROR] No port specified for pitcher selection")
            return False
        
        print(f"🥛 Starting pitcher pickup sequence for {port}")
        
        # Step 1: Move to espresso home position
        print("🏠 Moving to espresso home...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        
        # Step 2: Approach pitcher area
        print("🎯 Approaching pitcher area...")
        approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
        if approach_result is False:
            print("[ERROR] Failed to approach pitcher area")
            return False
        
        # Step 3: Pick pitcher based on port
        print(f"🤏 Picking pitcher for {port}...")
        if port == 'port_1':
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
            if approach_result is False:
                print("[ERROR] Failed to approach pitcher 1")
                return False
            
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
            if mount_result is False:
                print("[ERROR] Failed to mount pitcher 1")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip pitcher 1")
                return False
            
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
            if retreat_result is False:
                print("[ERROR] Failed to retreat from pitcher 1")
                return False
                
        elif port == 'port_2':
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
            if mount_result is False:
                print("[ERROR] Failed to mount pitcher 2")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip pitcher 2")
                return False
            
            pos_result = run_skill("gotoJ_deg", 23.034803,-44.195574,-116.188958,-19.410888,-66.975725,-0.169034, 1.0, 0.2)
            if pos_result is False:
                print("[ERROR] Failed to position for pitcher 2")
                return False
                
        elif port == 'port_3':
            move1_result = run_skill("moveEE", 0, 240, 0, 0, 0, 0)
            if move1_result is False:
                print("[ERROR] Failed to move to pitcher 3 position 1")
                return False
            
            move2_result = run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
            if move2_result is False:
                print("[ERROR] Failed to move to pitcher 3 position 2")
                return False
            
            grip_result = run_skill("set_gripper_position", 255, 100)
            if grip_result is False:
                print("[ERROR] Failed to grip pitcher 3")
                return False
            
            retreat_result = run_skill("moveEE", -103.413977, 0, 10, 0, 0, 0)
            if retreat_result is False:
                print("[ERROR] Failed to retreat with pitcher 3")
                return False
        else:
            print(f"[ERROR] unknown port: {port!r}, available ports: port_1, port_2, port_3")
            return False
        
        # Step 4: Move to final position
        print("📍 Moving to final position...")
        final_result = run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)
        if final_result is False:
            print("[ERROR] Failed to move to final position")
            return False
        
        print(f"✅ Pitcher pickup sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during pitcher pickup: {e}")
        return False    

def pour_pitcher(**params):
    """
    Pour milk from pitcher into cup at specified stage.
    
    This function performs the milk pouring sequence:
    - Moves to pouring position based on stage
    - Tilts pitcher to pour milk
    - Returns to neutral position
    - Moves back to holding position
    
    Args:
        stage (str): Target stage ('stage_1' or 'stage_2')
        
    Returns:
        bool: True if pouring completed successfully, False otherwise
        
    Example:
        success = pour_pitcher(stage='stage_1')
        if success:
            print("Milk poured successfully")
    """
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
            
            # Step 3: Tilt pitcher to pour
            print("⬇️ Tilting pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", 140.953509,-26.271451,-120.302336,-47.003274,-57.967889,-102.690158, 1.0, 0.2)
            if pour_result is False:
                print("[ERROR] Failed to tilt pitcher for pouring")
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
            
            # Step 3: Tilt pitcher to pour
            print("⬇️ Tilting pitcher to pour...")
            pour_result = run_skill("gotoJ_deg", 145.462132,-32.355488,-108.068238,-53.572020,-58.845487,-105.385536, 1.0, 0.2)
            if pour_result is False:
                print("[ERROR] Failed to tilt pitcher for pouring")
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
        
        # Step 6: Rotate back
        print("🔄 Rotating back...")
        rotate_result = run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
        if rotate_result is False:
            print("[ERROR] Failed to rotate back")
            return False
        
        # Step 7: Return to holding position
        print("🏠 Returning to holding position...")
        final_result = run_skill("gotoJ_deg", 31.076585,-40.253154,-136.313679,-3.210446,-58.931112,-0.206957, 1.0, 0.2)
        if final_result is False:
            print("[ERROR] Failed to return to holding position")
            return False
        
        print(f"✅ Milk pouring sequence completed successfully for {stage}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during pouring: {e}")
        return False

def hot_water(**params):
    """
    Fill pitcher with hot water from the hot water dispenser.
    
    This function performs the hot water filling sequence:
    - Moves to hot water dispenser approach position
    - Positions pitcher under hot water outlet
    - Activates hot water flow for specified duration
    - Returns to safe position
    
    Args:
        duration (float, optional): Duration to dispense hot water in seconds (default: 8.0)
        
    Returns:
        bool: True if hot water dispensed successfully, False otherwise
        
    Example:
        success = hot_water(duration=10.0)
        if success:
            print("Hot water dispensed successfully")
    """
    try:
        duration = params.get("duration", 8.0)  # Default 8 seconds
        
        print(f"🔥 Starting hot water dispensing sequence (duration: {duration}s)")
        
        # Step 1: Move to hot water dispenser approach position
        print("🎯 Approaching hot water dispenser...")
        approach_result = run_skill("gotoJ_deg", 67.341492,-49.798077,-99.061142,-30.809935,-22.613216,-0.457894)
        if approach_result is False:
            print("[ERROR] Failed to approach hot water dispenser")
            return False
        
        # Step 2: Position pitcher under hot water outlet
        print("📍 Positioning pitcher under hot water outlet...")
        position_result = run_skill("gotoJ_deg", 61.759601,-52.680407,-91.236745,-35.814578,-28.197699,-0.388979)
        if position_result is False:
            print("[ERROR] Failed to position pitcher under hot water outlet")
            return False
        
        # Step 3: Dispense hot water
        print(f"💧 Dispensing hot water for {duration} seconds...")
        time.sleep(duration)
        
        # Step 4: Move away from hot water outlet
        print("⬆️ Moving away from hot water outlet...")
        retreat_result = run_skill("gotoJ_deg", 67.341492,-49.798077,-99.061142,-30.809935,-22.613216,-0.457894, 1.0, 0.2)
        if retreat_result is False:
            print("[ERROR] Failed to move away from hot water outlet")
            return False
        
        # Step 5: Return to holding position
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

def return_pitcher(**params):
    """
    Return pitcher to its home position after use.
    
    This function performs the pitcher return sequence:
    - Navigates to the appropriate pitcher return location based on port
    - Positions pitcher in its designated spot
    - Releases gripper to place pitcher
    - Returns to espresso home position
    
    Args:
        port (str): Source port ('port_1', 'port_2', or 'port_3')
        
    Returns:
        bool: True if pitcher returned successfully, False otherwise
        
    Example:
        success = return_pitcher(port='port_1')
        if success:
            print("Pitcher returned successfully")
    """
    try:
        port = params.get("port")
        
        if not port:
            print("[ERROR] No port specified for pitcher return")
            return False
        
        print(f"🔄 Starting pitcher return sequence for {port}")
        
        # Step 1: Return pitcher based on port
        if port == 'port_1':
            print("🎯 Approaching pitcher 1 return position...")
            approach_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
            if approach_result is False:
                print("[ERROR] Failed to approach pitcher 1 return position")
                return False
            
            print("📍 Positioning pitcher 1 for return...")
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_1", True)
            if mount_result is False:
                print("[ERROR] Failed to position pitcher 1 for return")
                return False
            
            print("🤏 Releasing pitcher 1...")
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release pitcher 1")
                return False
            
            print("⬅️ Retreating from pitcher 1...")
            retreat_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_1", True)
            if retreat_result is False:
                print("[ERROR] Failed to retreat from pitcher 1")
                return False
                
        elif port == 'port_2':
            print("📍 Positioning pitcher 2 for return...")
            mount_result = run_skill("mount_machine", "three_group_espresso", "pick_pitcher_2", True)
            if mount_result is False:
                print("[ERROR] Failed to position pitcher 2 for return")
                return False
            
            print("🤏 Releasing pitcher 2...")
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release pitcher 2")
                return False
                
        elif port == 'port_3':
            print("📍 Moving to pitcher 3 return position...")
            move1_result = run_skill("moveEE", 0, 240, 10, 0, 0, 0)
            if move1_result is False:
                print("[ERROR] Failed to move to pitcher 3 return position 1")
                return False
            
            move2_result = run_skill("moveEE", 103.413977, 0, 0, 0, 0, 0)
            if move2_result is False:
                print("[ERROR] Failed to move to pitcher 3 return position 2")
                return False
            
            print("🤏 Releasing pitcher 3...")
            release_result = run_skill("set_gripper_position", 75, 0)
            if release_result is False:
                print("[ERROR] Failed to release pitcher 3")
                return False
            
            print("⬅️ Retreating from pitcher 3...")
            retreat_result = run_skill("moveEE", -103.413977, 0, -10, 0, 0, 0)
            if retreat_result is False:
                print("[ERROR] Failed to retreat from pitcher 3")
                return False
        else:
            print(f"[ERROR] unknown port: {port!r}, available ports: port_1, port_2, port_3")
            return False
        
        # Step 2: Move to common pitcher area
        print("🎯 Moving to pitcher area...")
        area_result = run_skill("approach_machine", "three_group_espresso", "pick_pitcher_2", True)
        if area_result is False:
            print("[ERROR] Failed to move to pitcher area")
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


# Register for CLI discovery
SEQUENCES = {
    'unmount': unmount,
    'grinder': grinder,
    'mount': mount,
    'pick_pitcher': pick_pitcher,
    'pour_pitcher': pour_pitcher,
    'hot_water': hot_water,
    'return_pitcher': return_pitcher,
}