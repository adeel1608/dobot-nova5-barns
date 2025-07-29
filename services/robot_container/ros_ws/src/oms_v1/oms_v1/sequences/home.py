"""
home.py

Defines the 'home' positioning routine using compass directions.
This module provides functions for robot positioning, machine calibration,
and system diagnostics for the BARNS coffee automation system.
"""

import time
from typing import Dict, Any, Union
from oms_v1.params import HOME_ANGLES
from oms_v1.manipulate_node import run_skill

# Predefined home positions for specific operational contexts
Espresso_home = (42.159162,16.269149,-135.156441,-81.822150,-49.784457,13.771214)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)


def home(**params) -> bool:
    """
    Move robot to a predefined home position.
    
    This function moves the robot to one of several predefined home positions
    based on the position parameter. These home positions are safe, known
    configurations for different operational contexts.
    
    Args:
        position (str): Name of the home position to move to (must exist in HOME_ANGLES), defaults to 'north'
        
    Returns:
        bool: True if robot moved to home position successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during movement
        
    Example:
        success = home(position='espresso_home')
        if success:
            print("Robot moved to home position successfully")
    """
    try:
        # Extract and validate position parameter
        position = params.get("position", "north")  # Default to north
        if not position:
            print("[ERROR] No position parameter provided")
            return False
            
        angles = HOME_ANGLES.get(str(position))
        
        if not angles:
            print(f"[ERROR] Unknown home position: {position!r}")
            print(f"[INFO] Available positions: {list(HOME_ANGLES.keys())}")
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


def get_machine_position(**params) -> bool:
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
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if all machine positions calibrated successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during calibration process
        
    Example:
        success = get_machine_position()
        if success:
            print("All machine positions calibrated successfully")
    """
    try:
        print("🎯 Starting comprehensive machine position calibration...")
        
        # Set optimal speed for calibration accuracy
        print("⚙️ Setting speed factor for precise movements...")
        run_skill("set_speed_factor", 100)
        
        # Step 1: Move to espresso home position
        print("🏠 Step 1/8: Moving to espresso home position...")
        home_result = run_skill("gotoJ_deg", 42.159162,16.269149,-135.156441,-81.822150,-49.784457,13.771214)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home position")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Position for portafilter cleaner calibration
        print("📍 Step 2/8: Positioning for portafilter cleaner calibration...")
        cleaner_prep_result = run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
        if cleaner_prep_result is False:
            print("[ERROR] Failed to position for cleaner calibration")
            return False
        print("   ✅ Successfully positioned for cleaner calibration")
        
        # Step 3: Perform multiple approaches to portafilter cleaner for accuracy
        print("🧹 Step 3/8: Calibrating portafilter cleaner position (5 approaches)...")
        for i in range(5):
            print(f"   📍 Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            approach_result = run_skill("move_to", "portafilter_cleaner", 0.26)
            if approach_result is False:
                print(f"[ERROR] Failed cleaner approach {i+1}/5")
                return False
        print("   ✅ All cleaner approaches completed successfully")
        
        # Record portafilter cleaner position
        print("💾 Step 4/8: Recording portafilter cleaner position...")
        print("   🔍 Waiting for ArUco marker ID 23 (portafilter_cleaner) to be detected...")
        cleaner_record_result = run_skill("get_machine_position", "portafilter_cleaner")
        if cleaner_record_result is False or cleaner_record_result is None:
            print("[ERROR] Failed to record portafilter cleaner position")
            print("   ❌ ArUco marker 23 may not be visible to camera")
            print("   💡 Tip: Ensure marker 23 is clearly visible and try again")
            return False
        print("   ✅ Portafilter cleaner position successfully recorded!")
        
        # Step 5: Position for espresso grinder calibration
        print("📍 Step 5/8: Positioning for espresso grinder calibration...")
        grinder_prep1_result = run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
        if grinder_prep1_result is False:
            print("[ERROR] Failed to position for grinder calibration (step 1)")
            return False
        
        grinder_prep2_result = run_skill("gotoJ_deg", 0.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if grinder_prep2_result is False:
            print("[ERROR] Failed to position for grinder calibration (step 2)")
            return False
        print("   ✅ Successfully positioned for grinder calibration")
        
        # Step 6: Perform multiple approaches to espresso grinder for accuracy
        print("☕ Step 6/8: Calibrating espresso grinder position (5 approaches)...")
        for i in range(5):
            print(f"   📍 Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            grinder_approach_result = run_skill("move_to", "espresso_grinder", 0.26)
            if grinder_approach_result is False:
                print(f"[ERROR] Failed grinder approach {i+1}/5")
                return False
        print("   ✅ All grinder approaches completed successfully")
        
        # Record espresso grinder position
        print("💾 Recording espresso grinder position...")
        print("   🔍 Waiting for ArUco marker ID 31 (espresso_grinder) to be detected...")
        grinder_record_result = run_skill("get_machine_position", "espresso_grinder")
        if grinder_record_result is False or grinder_record_result is None:
            print("[ERROR] Failed to record espresso grinder position")
            print("   ❌ ArUco marker 31 may not be visible to camera")
            print("   💡 Tip: Ensure marker 31 is clearly visible and try again")
            return False
        print("   ✅ Espresso grinder position successfully recorded!")
        
        # Step 7: Position for three-group espresso machine calibration
        print("📍 Step 7/8: Positioning for three-group espresso machine calibration...")
        espresso_prep1_result = run_skill("gotoJ_deg", 7.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
        if espresso_prep1_result is False:
            print("[ERROR] Failed to position for espresso machine calibration (step 1)")
            return False
        
        espresso_prep2_result = run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)
        if espresso_prep2_result is False:
            print("[ERROR] Failed to position for espresso machine calibration (step 2)")
            return False
        print("   ✅ Successfully positioned for espresso machine calibration")
        
        # Perform multiple approaches to three-group espresso machine for accuracy
        print("☕ Calibrating three-group espresso machine position (5 approaches)...")
        for i in range(5):
            print(f"   📍 Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            espresso_approach_result = run_skill("move_to", "three_group_espresso", 0.26)
            if espresso_approach_result is False:
                print(f"[ERROR] Failed espresso machine approach {i+1}/5")
                return False
        print("   ✅ All espresso machine approaches completed successfully")
        
        # Record three-group espresso machine position
        print("💾 Recording three-group espresso machine position...")
        print("   🔍 Waiting for ArUco marker ID 41 (three_group_espresso) to be detected...")
        espresso_record_result = run_skill("get_machine_position", "three_group_espresso")
        if espresso_record_result is False or espresso_record_result is None:
            print("[ERROR] Failed to record three-group espresso machine position")
            print("   ❌ ArUco marker 41 may not be visible to camera")
            print("   💡 Tip: Ensure marker 41 is clearly visible and try again")
            return False
        print("   ✅ Three-group espresso machine position successfully recorded!")
        
        # Step 8: Return to espresso home position
        print("🏠 Step 8/8: Returning to espresso home position...")
        final_home_result = run_skill("gotoJ_deg", 42.159162,16.269149,-135.156441,-81.822150,-49.784457,13.771214)
        if final_home_result is False:
            print("[ERROR] Failed to return to espresso home position")
            return False
        print("   ✅ Successfully returned to espresso home")
        
        # Verify and display saved data
        print("📋 Verifying saved machine position data...")
        saved_machines = check_saved_data()
        
        # Final success summary
        print("\n" + "="*60)
        print("✅ MACHINE POSITION CALIBRATION COMPLETED SUCCESSFULLY!")
        print("="*60)
        print("   ✓ Portafilter cleaner position recorded")
        print("   ✓ Espresso grinder position recorded") 
        print("   ✓ Three-group espresso machine position recorded")
        print(f"   📊 Total machines calibrated: {len(saved_machines)}")
        print("="*60)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during machine position calibration: {e}")
        print("[INFO] Calibration process terminated due to error")
        return False


def check_saved_data() -> Dict[str, Any]:
    """
    Check and display currently saved machine position data.
    
    This function reads the machine_pose_data_memory.yaml file and reports
    what machine positions have been successfully calibrated and saved.
    
    Returns:
        Dict[str, Any]: Dictionary of saved machine positions with their data,
                       or empty dict if no data found
        
    Raises:
        Exception: If error occurs while reading saved data file
    """
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    
    try:
        # Locate the machine position data file
        pkg_share = get_package_share_directory("pickn_place")
        mem_path = os.path.join(pkg_share, "machine_pose_data_memory.yaml")
        
        # Check if data file exists
        if not os.path.exists(mem_path):
            print(f"   ❌ No machine position data file found at: {mem_path}")
            print("   💡 Run calibration first to create position data")
            return {}
        
        # Read and parse the YAML data
        with open(mem_path, "r") as f:
            data = yaml.safe_load(f) or {}
        
        machines = data.get("machines", {})
        
        # Report findings
        if not machines:
            print("   ❌ No machine positions saved yet")
            print("   💡 Run get_machine_position() to calibrate and save positions")
            return {}
        
        print(f"   📊 Found {len(machines)} saved machine positions:")
        for machine_name, position_data in machines.items():
            timestamp = position_data.get("Time", "Unknown")
            translation = position_data.get("translation", {})
            x = translation.get("x", 0)
            y = translation.get("y", 0) 
            z = translation.get("z", 0)
            print(f"      ✓ {machine_name}: ({x:.3f}, {y:.3f}, {z:.3f}) saved at {timestamp}")
        
        return machines
        
    except Exception as e:
        print(f"   ⚠️  Error reading saved data: {e}")
        print("   💡 Check file permissions and YAML format")
        return {}


def check_aruco_status(**params) -> bool:
    """
    Check current ArUco marker detection status and help diagnose calibration issues.
    
    This function provides information about which markers should be detected
    for machine position calibration and offers troubleshooting tips.
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if status check completed successfully, False if error occurred
        
    Example:
        success = check_aruco_status()
        if success:
            print("ArUco status check completed")
    """
    try:
        print("🔍 ArUco Marker Detection Status Check")
        print("=" * 50)
        
        # Define required markers for machine calibration
        required_markers = {
            23: "portafilter_cleaner",
            31: "espresso_grinder", 
            41: "three_group_espresso"
        }
        
        # Display required markers information
        print("📋 Required ArUco markers for machine position calibration:")
        for marker_id, machine_name in required_markers.items():
            print(f"   • Marker ID {marker_id} → {machine_name}")
        
        # Provide troubleshooting guidance
        print("\n💡 Troubleshooting tips for marker detection:")
        troubleshooting_tips = [
            "Ensure markers are clearly visible to the camera",
            "Check that markers are not obstructed or damaged",
            "Verify camera is properly positioned and focused",
            "Make sure lighting conditions are adequate",
            "Try repositioning the robot for better camera angles",
            "Confirm markers are printed at correct size and quality"
        ]
        
        for i, tip in enumerate(troubleshooting_tips, 1):
            print(f"   {i}. {tip}")
        
        # Provide monitoring guidance
        print("\n📊 To monitor real-time detection, watch the robot logs for:")
        print("   • 'Detected N marker(s): [ID1, ID2, ...]' - successful detection")
        print("   • 'No ArUco markers detected' - markers not visible")
        print("   • Check camera feed if available for visual confirmation")
        
        # Display current saved data status
        print("\n📁 Current saved data status:")
        saved_data = check_saved_data()
        
        # Provide next steps based on current state
        print("\n🎯 Recommended next steps:")
        if not saved_data:
            print("   → Run get_machine_position() to perform initial calibration")
        else:
            print("   → Calibration data exists - system ready for operation")
            print("   → Re-run get_machine_position() if positions have changed")
        
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Error during ArUco status check: {e}")
        return False


# Register functions for CLI discovery and external access
SEQUENCES = {
    'home': home,
    'get_machine_position': get_machine_position,
    'check_saved_data': check_saved_data,
    'check_aruco_status': check_aruco_status,
}
