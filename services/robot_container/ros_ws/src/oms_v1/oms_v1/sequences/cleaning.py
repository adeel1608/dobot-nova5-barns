"""
cleaning.py

Defines the cleaning routine for machine ports using parameterized configurations.
"""

import time
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.espresso import unmount, mount
Espresso_home = (42.427441, 13.883821, -133.648376, -81.024788, -49.533218, 13.894379)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)

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
        time.sleep(0.2)
        if unmount_result is False:
            print(f"[ERROR] Failed to unmount portafilter from {port}")
            return False
        
        # Step 2: Move to cleaning station home position
        print("🏠 Moving to cleaning station...")
        home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        time.sleep(0.2)
        if home_result is False:
            print("[ERROR] Failed to move to cleaning station home")
            return False
        
        # Step 3: Perform hard brush cleaning
        print("🪥 Starting hard brush cleaning...")
        approach_result = run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
        time.sleep(0.2)
        if approach_result is False:
            print("[ERROR] Failed to approach hard brush")
            return False
            
        # Adjust position for better cleaning angle
        move_result = run_skill("moveEE", -88, 0, 0, 0, 0, -135)
        time.sleep(0.2)
        if move_result is False:
            print("[ERROR] Failed to adjust position for hard brush")
            return False
            
        # Mount to hard brush for cleaning
        mount_result = run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
        time.sleep(0.2)
        if mount_result is False:
            print("[ERROR] Failed to mount to hard brush")
            return False
            
        # Move up after hard brush cleaning
        up_result = run_skill("moveEE", 0, 0, 100, 0, 0, 0)
        time.sleep(0.2)
        if up_result is False:
            print("[ERROR] Failed to move up after hard brush")
            return False
        
        # Step 4: Perform soft brush cleaning
        print("🧽 Starting soft brush cleaning...")
        soft_approach_result = run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
        time.sleep(0.2)
        if soft_approach_result is False:
            print("[ERROR] Failed to approach soft brush")
            return False
            
        # Mount to soft brush for cleaning
        soft_mount_result = run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
        time.sleep(0.2)
        if soft_mount_result is False:
            print("[ERROR] Failed to mount to soft brush")
            return False
            
        # Move up after soft brush cleaning
        soft_up_result = run_skill("moveEE", 0, 0, 150, 0, 0, 0)
        time.sleep(0.2)
        if soft_up_result is False:
            print("[ERROR] Failed to move up after soft brush")
            return False
        
        # Step 5: Return to cleaning station home
        print("🏠 Returning to cleaning station home...")
        return_home_result = run_skill("gotoJ_deg", *Espresso_grinder_home)
        time.sleep(0.2)
        if return_home_result is False:
            print("[ERROR] Failed to return to cleaning station home")
            return False
        
        # Step 6: Mount the portafilter back to espresso group
        print(f"📥 Remounting portafilter to {port}...")
        mount_result = mount(port=port)
        time.sleep(0.2)
        if mount_result is False:
            print(f"[ERROR] Failed to remount portafilter to {port}")
            return False
            
        print(f"✅ Cleaning sequence completed successfully for {port}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cleaning: {e}")
        return False

# Register for CLI discovery
SEQUENCES = {
    'clean': clean,
}
