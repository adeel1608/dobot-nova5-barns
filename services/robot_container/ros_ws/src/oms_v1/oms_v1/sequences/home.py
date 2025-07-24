"""
home.py

Defines the 'home' positioning routine using compass directions.
"""

import time
from oms_v1.params import HOME_ANGLES
from oms_v1.manipulate_node import run_skill  # REMOVED - using local import instead
Espresso_home = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)


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
        run_skill("set_speed_factor",100)
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
            
            approach_result = run_skill("move_to", "portafilter_cleaner", 0.26)
            if approach_result is False:
                print(f"[ERROR] Failed cleaner approach {i+1}/5")
                return False
        
        # Record portafilter cleaner position
        print("💾 Recording portafilter cleaner position...")
        print("   🔍 Waiting for ArUco marker ID 23 (portafilter_cleaner) to be detected...")
        cleaner_record_result = run_skill("get_machine_position", "portafilter_cleaner")
        if cleaner_record_result is False or cleaner_record_result is None:
            print("[ERROR] Failed to record portafilter cleaner position - ArUco marker 23 may not be visible")
            print("   💡 Tip: Ensure marker 23 is clearly visible to the camera and try again")
            return False
        print("   ✅ Portafilter cleaner position successfully recorded!")
        
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
            
            grinder_approach_result = run_skill("move_to", "espresso_grinder", 0.26)
            if grinder_approach_result is False:
                print(f"[ERROR] Failed grinder approach {i+1}/5")
                return False
        
        # Record espresso grinder position
        print("💾 Recording espresso grinder position...")
        print("   🔍 Waiting for ArUco marker ID 31 (espresso_grinder) to be detected...")
        grinder_record_result = run_skill("get_machine_position", "espresso_grinder")
        if grinder_record_result is False or grinder_record_result is None:
            print("[ERROR] Failed to record espresso grinder position - ArUco marker 31 may not be visible")
            print("   💡 Tip: Ensure marker 31 is clearly visible to the camera and try again")
            return False
        print("   ✅ Espresso grinder position successfully recorded!")
        
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
            
            espresso_approach_result = run_skill("move_to", "three_group_espresso", 0.26)
            if espresso_approach_result is False:
                print(f"[ERROR] Failed espresso machine approach {i+1}/5")
                return False
        
        # Record three-group espresso machine position
        print("💾 Recording three-group espresso machine position...")
        print("   🔍 Waiting for ArUco marker ID 41 (three_group_espresso) to be detected...")
        espresso_record_result = run_skill("get_machine_position", "three_group_espresso")
        if espresso_record_result is False or espresso_record_result is None:
            print("[ERROR] Failed to record three-group espresso machine position - ArUco marker 41 may not be visible")
            print("   💡 Tip: Ensure marker 41 is clearly visible to the camera and try again")
            return False
        print("   ✅ Three-group espresso machine position successfully recorded!")
        
        # Step 8: Return to espresso home position
        print("🏠 Returning to espresso home position...")
        final_home_result = run_skill("gotoJ_deg", 42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if final_home_result is False:
            print("[ERROR] Failed to return to espresso home")
            return False
        
        # Check and display saved data
        print("📋 Verifying saved machine position data...")
        check_saved_data()
        
        print("✅ Machine position calibration completed successfully!")
        print("   ✓ Portafilter cleaner position recorded")
        print("   ✓ Espresso grinder position recorded") 
        print("   ✓ Three-group espresso machine position recorded")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during machine position calibration: {e}")
        return False

def check_saved_data():
    """
    Check and display currently saved machine position data.
    
    This function reads the machine_pose_data_memory.yaml file and reports
    what machine positions have been successfully calibrated and saved.
    
    Returns:
        dict: Dictionary of saved machine positions, or empty dict if no data
    """
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    
    try:
        pkg_share = get_package_share_directory("pickn_place")
        mem_path = os.path.join(pkg_share, "machine_pose_data_memory.yaml")
        
        if not os.path.exists(mem_path):
            print(f"   ❌ No machine position data file found at: {mem_path}")
            return {}
        
        with open(mem_path, "r") as f:
            data = yaml.safe_load(f) or {}
        
        machines = data.get("machines", {})
        
        if not machines:
            print("   ❌ No machine positions saved yet")
            return {}
        
        print(f"   📊 Found {len(machines)} saved machine positions:")
        for machine_name, position_data in machines.items():
            timestamp = position_data.get("Time", "Unknown")
            translation = position_data.get("translation", {})
            x, y, z = translation.get("x", 0), translation.get("y", 0), translation.get("z", 0)
            print(f"      ✓ {machine_name}: ({x:.3f}, {y:.3f}, {z:.3f}) at {timestamp}")
        
        return machines
        
    except Exception as e:
        print(f"   ⚠️  Error reading saved data: {e}")
        return {}

def check_aruco_status(**params):
    """
    Check current ArUco marker detection status and help diagnose calibration issues.
    
    This function provides information about which markers should be detected
    for machine position calibration and offers troubleshooting tips.
    
    Returns:
        bool: True if status check completed successfully
    """
    print("🔍 ArUco Marker Detection Status Check")
    print("=" * 50)
    
    # Show required markers for machine calibration
    required_markers = {
        23: "portafilter_cleaner",
        31: "espresso_grinder", 
        41: "three_group_espresso"
    }
    
    print("📋 Required ArUco markers for machine position calibration:")
    for marker_id, machine_name in required_markers.items():
        print(f"   • Marker ID {marker_id} → {machine_name}")
    
    print("\n💡 Troubleshooting tips:")
    print("   1. Ensure markers are clearly visible to the camera")
    print("   2. Check that markers are not obstructed or damaged")
    print("   3. Verify camera is properly positioned and focused")
    print("   4. Make sure lighting conditions are adequate")
    print("   5. Try repositioning the robot for better camera angles")
    
    print("\n📊 To monitor real-time detection, watch the robot logs for:")
    print("   • 'Detected N marker(s): [ID1, ID2, ...]' - successful detection")
    print("   • 'No ArUco markers detected' - markers not visible")
    
    # Check current saved data status
    print("\n📁 Current saved data status:")
    check_saved_data()
    
    return True

# Register for CLI discovery
SEQUENCES = {
    'home': home,
    'get_machine_position' : get_machine_position,
    'check_saved_data': check_saved_data,
    'check_aruco_status': check_aruco_status,
}
