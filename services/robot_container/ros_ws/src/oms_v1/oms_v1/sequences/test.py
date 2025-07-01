"""
test.py
"""

import time
from oms_v1.manipulate_node import run_skill
from oms_v1.params import PULL_ESPRESSO_PARAMS, POUR_PARAMS, HOME_ANGLES, GRAB_CUP_PARAMS, PLACE_CUP_PARAMS
from oms_v1.sequences.home import home


def test(**params):
    """
    Test sequence for milk frother tool manipulation.
    
    This function performs a test sequence that demonstrates:
    - Moving to various positions
    - Tool approach and grabbing operations
    - Position memory and return operations
    - Gripper manipulation
    - Home position return
    
    Args:
        **params: Optional parameters (currently unused)
        
    Returns:
        bool: True if test sequence completed successfully, False otherwise
        
    Example:
        success = test()
        if success:
            print("Test sequence completed successfully")
    """
    try:
        print("🧪 Starting test sequence for milk frother tool manipulation")
        
        # Step 1: Move to initial position
        print("📍 Moving to initial position...")
        init_result = run_skill("gotoJ_deg", 20.847986, -21.981329, -113.153931, -76.829208, -81.786911, -0.050592, 1.0, 0.2)
        if init_result is False:
            print("[ERROR] Failed to move to initial position")
            return False
        
        time.sleep(0.25)
        
        # Step 2: Move to milk frother
        print("🎯 Moving to milk frother...")
        move_result = run_skill("move_to", 'milk_frother_1', 0.175)
        if move_result is False:
            print("[ERROR] Failed to move to milk frother")
            return False
        
        # Step 3: Approach milk frother tool
        print("🔧 Approaching milk frother tool...")
        approach_result = run_skill("approach_tool", 'milk_frother_1', 170)
        if approach_result is False:
            print("[ERROR] Failed to approach milk frother tool")
            return False
        
        # Step 4: Capture approach angles
        print("📊 Capturing approach angles...")
        approach_angles = run_skill("current_angles")
        time.sleep(0.25)
        
        # Step 5: Grab the tool
        print("🤏 Grabbing milk frother tool...")
        grab_result = run_skill("grab_tool", 'milk_frother_1', 200, 250, 255)
        if grab_result is False:
            print("[ERROR] Failed to grab milk frother tool")
            return False
        
        # Step 6: Capture grab angles
        print("📊 Capturing grab angles...")
        grab_angles = run_skill("current_angles")
        
        # Step 7: Move to intermediate position
        print("📍 Moving to intermediate position...")
        inter1_result = run_skill("gotoJ_deg", -4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837, 1.0, 0.2)
        if inter1_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        
        time.sleep(0.25)
        
        # Step 8: Move to working position
        print("🔄 Moving to working position...")
        work_result = run_skill("gotoJ_deg", 22.345373, -76.252151, -61.342220, -40.423759, -81.360077, 11.115391, 1.0, 0.2)
        if work_result is False:
            print("[ERROR] Failed to move to working position")
            return False
        
        time.sleep(0.25)
        
        # Step 9: Return to grab position (if angles were captured)
        if grab_angles is not None:
            print("↩️ Returning to grab position...")
            return_grab_result = run_skill("gotoJ_deg", *grab_angles, 1.0, 0.2)
            if return_grab_result is False:
                print("[ERROR] Failed to return to grab position")
                return False
        else:
            print("[WARNING] Grab angles not captured, skipping return to grab position")
        
        time.sleep(0.25)
        
        # Step 10: Adjust gripper position
        print("🔧 Adjusting gripper position...")
        grip_result = run_skill("set_gripper_position", 255, 165)
        if grip_result is False:
            print("[ERROR] Failed to adjust gripper position")
            return False
        
        # Step 11: Return to approach position (if angles were captured)
        if approach_angles is not None:
            print("↩️ Returning to approach position...")
            return_approach_result = run_skill("gotoJ_deg", *approach_angles, 1.0, 0.2)
            if return_approach_result is False:
                print("[ERROR] Failed to return to approach position")
                return False
        else:
            print("[WARNING] Approach angles not captured, skipping return to approach position")
        
        time.sleep(0.25)
        
        # Step 12: Return to home position
        print("🏠 Returning to home position...")
        home_result = home(position="north")
        if home_result is False:
            print("[ERROR] Failed to return to home position")
            return False
        
        # Step 13: Open gripper
        print("🤏 Opening gripper...")
        release_result = run_skill("set_gripper_position", 255, 0)
        if release_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        
        print("✅ Test sequence completed successfully")
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during test sequence: {e}")
        return False
    

# Register for CLI discovery
SEQUENCES = {
    'test': test,
}
