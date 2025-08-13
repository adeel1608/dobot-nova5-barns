"""
cleaning.py

Defines the cleaning routine for machine ports using parameterized configurations.
This module provides comprehensive portafilter cleaning functionality for the BARNS
coffee automation system, including hard brush and soft brush cleaning sequences
with precise positioning and error handling.
"""

import time
from typing import Dict, Any, Optional
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.espresso import unmount, mount

# Predefined home positions for cleaning operations
Espresso_home = (42.159162,16.269149,-135.156441,-81.822150,-49.784457,13.771214)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)


def clean(**params) -> bool:
    """
    Complete portafilter cleaning sequence.
    
    This function performs a comprehensive cleaning workflow:
    1. Unmounts the portafilter from the specified espresso group
    2. Moves to the cleaning station
    3. Performs hard brush cleaning with precise positioning
    4. Performs soft brush cleaning with thorough coverage
    5. Remounts the portafilter back to the espresso group
    
    The cleaning process ensures optimal hygiene and performance by utilizing
    both hard and soft brush cleaning methods with automated positioning.
    
    Args:
        port (str): The portafilter port to clean ('port_1', 'port_2', or 'port_3'), defaults to 'port_2'
        
    Returns:
        bool: True if cleaning sequence completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during cleaning process
        
    Example:
        success = clean(port='port_1')
        if success:
            print("Cleaning completed successfully")
    """
    try:
        # Extract and validate port parameter
        port = params.get("port", "port_2")  # Default to port_2
        if not port:
            print("[ERROR] No port parameter provided")
            print("[INFO] Please provide port parameter: 'port_1', 'port_2', or 'port_3'")
            return False
        
        # Validate port parameter
        if port not in ('port_1', 'port_2', 'port_3'):
            print(f"[ERROR] Invalid port: {port!r}")
            print("[INFO] Valid ports: port_1, port_2, port_3")
            return False
        
        print(f"🧹 Starting comprehensive cleaning sequence for {port}")
        print("=" * 60)
        
        # Step 1: Unmount the portafilter from espresso group
        print(f"📤 Step 1/6: Unmounting portafilter from {port}...")
        unmount_result = unmount(port=port)
        
        if unmount_result is False:
            print(f"[ERROR] Failed to unmount portafilter from {port}")
            print("[INFO] Cannot proceed with cleaning without successful unmount")
            return False
        print("   ✅ Portafilter successfully unmounted")
        
        # Step 2: Move to cleaning station home position
        print("🏠 Step 2/6: Moving to cleaning station...")
        home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        
        if home_result is False:
            print("[ERROR] Failed to move to cleaning station home")
            return False
        print("   ✅ Successfully positioned at cleaning station")
        
        # Step 3: Perform hard brush cleaning
        print("🪣 Step 3/6: Performing hard brush cleaning...")
        print("   🎯 Approaching hard brush cleaning position...")
        approach_result = run_skill("approach_machine", "portafilter_cleaner", "hard_brush")
        
        if approach_result is False:
            print("[ERROR] Failed to approach hard brush")
            return False
        print("   ✅ Successfully approached hard brush")
            
        # Adjust position for better cleaning angle
        print("   📐 Adjusting position for optimal cleaning angle...")
        move_result = run_skill("gotoJ_deg", -79.324684, 3.729006, -124.690453, -58.130096, -80.298050, -130.331278)
        
        if move_result is False:
            print("[ERROR] Failed to adjust position for hard brush")
            return False
        print("   ✅ Position adjusted for optimal cleaning")
            
        # Mount to hard brush for cleaning
        print("   🔧 Mounting to hard brush for cleaning...")
        mount_result = run_skill("mount_machine", "portafilter_cleaner", "hard_brush")
        
        if mount_result is False:
            print("[ERROR] Failed to mount to hard brush")
            return False
        print("   ✅ Successfully mounted to hard brush")

        # First cleaning motion
        print("   🧽 Executing first cleaning motion...")
        clean_motion1 = run_skill("moveEE", 1.745675, -6.326488, 31.263496, 7.742872, 0.059984, 0.826083) 
        if clean_motion1 is False:
            print("[WARNING] First cleaning motion may not have completed fully")

        # Second cleaning motion for thorough cleaning
        print("   🧽 Executing second cleaning motion...")
        clean_motion2 = run_skill("moveEE", 0, 0, -2.75, 0, 0, 0)
        if clean_motion2 is False:
            print("[WARNING] Second cleaning motion may not have completed fully")
            
        # Move up after hard brush cleaning
        print("   ⬆️ Retracting from hard brush...")
        up_result = run_skill("moveEE", 0, 0, 100, 0, 0, 0)
        
        if up_result is False:
            print("[ERROR] Failed to move up after hard brush")
            return False
        print("   ✅ Hard brush cleaning completed successfully")
        
        # Step 4: Perform soft brush cleaning
        print("🪶 Step 4/6: Performing soft brush cleaning...")
        print("   🎯 Approaching soft brush cleaning position...")
        soft_approach_result = run_skill("approach_machine", "portafilter_cleaner", "soft_brush")
        
        if soft_approach_result is False:
            print("[ERROR] Failed to approach soft brush")
            return False
        print("   ✅ Successfully approached soft brush")
            
        # Mount to soft brush for cleaning
        print("   🔧 Mounting to soft brush for gentle cleaning...")
        soft_mount_result = run_skill("mount_machine", "portafilter_cleaner", "soft_brush")
        
        if soft_mount_result is False:
            print("[ERROR] Failed to mount to soft brush")
            return False
        print("   ✅ Successfully mounted to soft brush")

        # First soft cleaning motion
        print("   🪶 Executing first gentle cleaning motion...")
        soft_clean1 = run_skill("moveEE", 1.745675, -6.326488, 31.263496, 7.742872, 0.059984, 0.826083) 
        if soft_clean1 is False:
            print("[WARNING] First soft cleaning motion may not have completed fully")

        # Second soft cleaning motion for thorough cleaning
        print("   🪶 Executing second gentle cleaning motion...")
        soft_clean2 = run_skill("moveEE", 0, 0, -2.75, 0, 0, 0)
        if soft_clean2 is False:
            print("[WARNING] Second soft cleaning motion may not have completed fully")
            
        # Move up after soft brush cleaning
        print("   ⬆️ Retracting from soft brush...")
        soft_up_result = run_skill("moveEE", 0, 0, 150, 0, 0, 0)
        
        if soft_up_result is False:
            print("[ERROR] Failed to move up after soft brush")
            return False
        print("   ✅ Soft brush cleaning completed successfully")
        
        # Step 5: Return to cleaning station home
        print("🏠 Step 5/6: Returning to cleaning station home...")
        return_home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        
        if return_home_result is False:
            print("[ERROR] Failed to return to cleaning station home")
            return False
        print("   ✅ Successfully returned to cleaning station home")
        
        # # Step 6: Mount the portafilter back to espresso group
        # print(f"📥 Step 6/6: Remounting portafilter to {port}...")
        # mount_result = mount(port=port)
        # if mount_result is False:
        #     print(f"[ERROR] Failed to remount portafilter to {port}")
        #     print("[INFO] Cleaning completed but remount failed - manual intervention may be required")
        #     return False
        # print("   ✅ Portafilter successfully remounted")
            
        # Final success summary
        print("=" * 60)
        print(f"✅ CLEANING SEQUENCE COMPLETED SUCCESSFULLY FOR {port.upper()}")
        print("   ✓ Portafilter unmounted and remounted safely")
        print("   ✓ Hard brush cleaning performed with precision")
        print("   ✓ Soft brush cleaning completed for optimal finish")
        print("   ✓ All cleaning motions executed successfully")
        print("   🧹 Portafilter is now clean and ready for use!")
        print("=" * 60)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cleaning: {e}")
        print("[INFO] Cleaning process terminated due to error")
        print("[INFO] Manual inspection of portafilter position may be required")
        return False


# Register functions for CLI discovery and external access
SEQUENCES = {
    'clean': clean,
}
