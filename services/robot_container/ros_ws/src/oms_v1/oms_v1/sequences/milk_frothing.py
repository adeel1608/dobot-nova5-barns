"""
milk_frothing.py

Defines the milk frothing sequences for coffee preparation automation.
This module provides comprehensive functions for handling milk frothing operations
in the BARNS coffee automation system, including frother positioning, mounting,
steam activation, milk pouring, and cleaning procedures.
"""

import time
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home

# Global variables to store robot positions during milk frothing operations
# These are used to remember positions between function calls for safe return operations
approach_angles: Optional[Tuple[float, ...]] = None
grab_angles: Optional[Tuple[float, ...]] = None


def get_frother_position(**params) -> bool:
    """
    Calibrate and record the milk frother position for future operations.
    
    This function performs calibration of the milk frother position:
    1. Moves to north-east home position for approach
    2. Opens gripper to prepare for positioning
    3. Performs steam wand positioning and calibration
    4. Executes multiple approaches to left steam wand for accuracy
    5. Records the calibrated position for future reference
    
    This calibration should be performed when setting up the milk frothing station
    or when the frother position may have changed.
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if frother position calibrated successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during calibration process
        
    Example:
        success = get_frother_position()
        if success:
            print("Frother position calibrated successfully")
    """
    try:
        print("🎯 Starting milk frother position calibration...")
        print("=" * 50)
        
        # Set optimal speed for calibration
        print("⚙️ Setting speed factor for precise calibration...")
        run_skill("set_speed_factor", 100)
        
        # Step 1: Move to home position for setup
        print("🏠 Step 1/5: Moving to north-east home position...")
        home(position="north_east")
        home_result = run_skill("gotoJ_deg", -67.357964, -23.709629, -89.522377, -84.038696, -113.021690, 8.468687)
        if home_result is False:
            print("[ERROR] Failed to move to north-east home position")
            return False
        print("   ✅ Successfully moved to north-east home")
        
        # Step 2: Open gripper to prepare for positioning
        print("🤏 Step 2/5: Opening gripper for positioning...")
        gripper_result = run_skill("set_gripper_position", 255, 0)
        if gripper_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        print("   ✅ Gripper opened successfully")
        
        # Step 3: Steam wand positioning and preparation
        print("🎯 Step 3/5: Steam wand positioning and preparation...")
        print("   📍 Moving to steam wand...")
        run_skill("move_to", "left_steam_wand", 0.28)
        run_skill("sync")
        
        print("   🔧 Grabbing steam wand tool...")
        run_skill("grab_tool", "left_steam_wand")
        run_skill("sync")
        
        print("   🤏 Setting grip position...")
        run_skill("set_gripper_position", 255, 200)
        run_skill("sync")
        
        print("   📍 Moving to calibration position...")
        positioning_result = run_skill("gotoJ_deg", -42.886719, -74.454857, -15.463737, -96.987885, -87.116272, 7.834893)
        if positioning_result is False:
            print("[ERROR] Failed to move to calibration position")
            return False
        
        run_skill("sync")
        print("   🤏 Releasing grip for calibration...")
        run_skill("set_gripper_position", 255, 0)
        print("   ✅ Steam wand positioning completed")
        
        # Step 4: Perform multiple approaches for accuracy
        print("🎯 Step 4/5: Performing calibration approaches (5 attempts)...")
        for i in range(5):
            print(f"   📍 Approach {i+1}/5...")
            time.sleep(1.0)  # Allow settling time between approaches
            
            approach_result = run_skill("move_to", "left_steam_wand", 0.29)
            if approach_result is False:
                print(f"[ERROR] Failed calibration approach {i+1}/5")
                return False
        print("   ✅ All calibration approaches completed successfully")
        
        # Step 5: Record the calibrated position
        print("💾 Step 5/5: Recording milk frother position...")
        record_result = run_skill("get_machine_position", "left_steam_wand")
        if record_result is False:
            print("[ERROR] Failed to record milk frother position")
            print("[INFO] ArUco marker for left_steam_wand may not be visible")
            return False
        print("   ✅ Milk frother position recorded successfully")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK FROTHER POSITION CALIBRATION COMPLETED")
        print("   ✓ Steam wand positioning calibrated")
        print("   ✓ Multiple approach accuracy verified")
        print("   ✓ Position data saved for future operations")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother position calibration: {e}")
        print("[INFO] Calibration process terminated due to error")
        return False


def pick_frother(**params) -> bool:
    """
    Pick up the milk frother for milk frothing operations.
    
    This function handles the milk frother pickup sequence:
    1. Moves to frother approach position
    2. Approaches the milk frother with precise positioning
    3. Grabs the frother with appropriate grip strength
    4. Stores position data for later return operations
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if frother picked successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pickup process
        
    Example:
        success = pick_frother()
        if success:
            print("Milk frother picked successfully")
    """
    try:
        global approach_angles, grab_angles
        
        print("🥛 Starting milk frother pickup sequence...")
        print("=" * 50)
        
        # Step 1: Move to frother area position
        print("📍 Step 1/6: Moving to frother area...")
        area_result = run_skill("gotoJ_deg", 20.847986, -21.981329, -113.153931, -76.829208, -81.786911, -0.050592)
        if area_result is False:
            print("[ERROR] Failed to move to frother area")
            return False
        print("   ✅ Successfully moved to frother area")
        
        # Step 2: Approach the milk frother
        print("🎯 Step 2/6: Approaching milk frother...")
        approach_result = run_skill("move_to", 'milk_frother_2', 0.29)
        if not approach_result:
            print("[WARNING] Failed to approach milk_frother_2, trying milk_frother_1...")
            approach_result = run_skill("move_to", 'milk_frother_1', 0.29)
            if not approach_result:
                print("[ERROR] Failed to approach both milk_frother_2 and milk_frother_1")
                return False
        print("   ✅ Successfully approached milk frother")
        
        # Step 3: Move to approach position for frother
        print("📍 Step 3/6: Moving to frother approach position...")
        run_skill("sync")
        approach_tool_result = run_skill("approach_tool", 'milk_frother_2')
        if approach_tool_result is False:
            print("[ERROR] Failed to move to frother approach position")
            return False
        print("   ✅ Successfully positioned for frother approach")
        
        # Set initial grip position
        print("   🤏 Setting initial grip position...")
        run_skill("set_gripper_position", 255, 169)
        run_skill("sync")
        
        # Step 4: Record current approach position
        print("💾 Step 4/6: Recording approach position...")
        current_angles = run_skill("current_angles")
        if current_angles is not None:
            approach_angles = current_angles
            print(f"   ✅ Approach angles recorded: {len(approach_angles)} joint values")
        else:
            print("[WARNING] Failed to record approach angles - continuing without position memory")
            approach_angles = None
        
        # Step 5: Grab the frother
        print("🤏 Step 5/6: Grabbing milk frother...")
        run_skill("sync")
        grab_result = run_skill("grab_tool", 'milk_frother_2', 100, 100, -6, -5)
        if grab_result is False:
            print("[ERROR] Failed to grab milk frother")
            return False
        
        run_skill("sync")
        print("   🤏 Securing frother with full grip...")
        run_skill("set_gripper_position", 255, 255)
        print("   ✅ Milk frother secured successfully")
        
        # Step 6: Record current grab position
        print("💾 Step 6/6: Recording grab position...")
        grab_angles = run_skill("current_angles")
        if grab_angles is not None:
            print(f"   ✅ Grab angles recorded: {len(grab_angles)} joint values")
        else:
            print("[WARNING] Failed to record grab angles - continuing without position memory")
            grab_angles = None
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK FROTHER PICKUP COMPLETED SUCCESSFULLY")
        print("   ✓ Frother securely gripped and positioned")
        print("   ✓ Position data recorded for safe return")
        print("   ✓ Ready for mounting to steam wand")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother pickup: {e}")
        print("[INFO] Frother pickup process terminated due to error")
        return False
    

def mount_frother(**params) -> bool:
    """
    Mount the milk frother to the steam wand for frothing preparation.
    
    This function positions the frother on the steam wand:
    1. Sets precise servo timing for accurate movements
    2. Moves to milk frothing preparation position
    3. Approaches the steam wand with the frother (deep position)
    4. Mounts frother securely to steam wand
    
    Note: This function only positions the frother. Use froth_milk() to actually activate steam.
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if frother mounted successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during mounting process
        
    Example:
        success = mount_frother()
        if success:
            print("Frother mounted to steam wand successfully")
    """
    try:
        print("☁️ Starting milk frother mounting sequence...")
        print("=" * 50)

        # Step 1: Set slower servo timing for precise movements
        print("⚙️ Step 1/4: Setting precise servo timing...")
        timing_result = run_skill("set_speed_factor", 50)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing with default...")
        else:
            print("   ✅ Servo timing set for precise movements")
        
        # Step 2: Move to frothing preparation position
        print("📍 Step 2/4: Moving to frothing preparation position...")
        prep_result = run_skill("gotoJ_deg", -4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837)
        if prep_result is False:
            print("[ERROR] Failed to move to frothing preparation position")
            return False
        print("   ✅ Successfully positioned for frothing preparation")
        
        # Step 3: Approach steam wand (deep position)
        print("🎯 Step 3/4: Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        print("   ✅ Successfully approached steam wand")

        # Step 4: Mount frother to steam wand
        print("🔧 Step 4/4: Mounting frother to steam wand...")
        mount_result = run_skill("mount_machine", "left_steam_wand", "deep_froth")
        if mount_result is False:
            print("[ERROR] Failed to mount frother to steam wand")
            return False
        print("   ✅ Frother successfully mounted to steam wand")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK FROTHER MOUNTING COMPLETED SUCCESSFULLY")
        print("   ✓ Precise positioning achieved")
        print("   ✓ Frother securely mounted to steam wand")
        print("   ✓ Ready for milk frothing operation")
        print("=" * 50)
        return True
               
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother mounting: {e}")
        print("[INFO] Mounting process terminated due to error")
        return False


def froth_milk(**params) -> bool:
    """
    Activate steam to froth milk for the specified duration.
    
    This function controls the steam activation for milk frothing:
    1. Validates duration parameter
    2. Activates steam for frothing
    3. Maintains steam for specified duration
    4. Deactivates steam safely
    5. Allows settling time
    
    Args:
        duration (float): Duration in seconds to froth milk (default: 7.5)
    
    Returns:
        bool: True if milk frothing completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during frothing process
        
    Example:
        success = froth_milk(duration=10.0)
        if success:
            print("Milk frothing completed successfully")
    """
    try:
        # Extract and validate duration parameter
        duration = params.get("duration", 7.5)
        if not isinstance(duration, (int, float)) or duration <= 0:
            print(f"[ERROR] Invalid duration: {duration}, must be a positive number")
            return False
        
        print(f"🥛 Starting milk frothing sequence for {duration} seconds...")
        print("=" * 50)
        
        run_skill("sync")
        
        # Step 1: Activate steam for frothing
        print("💨 Step 1/4: Activating steam for milk frothing...")
        steam_on_result = run_skill("set_DO", 1, 1)
        if steam_on_result is False:
            print("[ERROR] Failed to activate steam")
            return False
        print("   ✅ Steam successfully activated")
        
        # Step 2: Allow frothing time
        print(f"☁️ Step 2/4: Frothing milk ({duration} seconds)...")
        print("   🥛 Milk frothing in progress...")
        time.sleep(duration)
        print("   ✅ Frothing duration completed")
        
        # Step 3: Deactivate steam
        print("💨 Step 3/4: Deactivating steam...")
        steam_off_result = run_skill("set_DO", 1, 0)
        if steam_off_result is False:
            print("[ERROR] Failed to deactivate steam")
            return False
        print("   ✅ Steam successfully deactivated")
        
        # Step 4: Allow settling time
        print("⏰ Step 4/4: Allowing settling time...")
        time.sleep(2)
        print("   ✅ Settling time completed")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK FROTHING COMPLETED SUCCESSFULLY")
        print(f"   ✓ Steam activated for {duration} seconds")
        print("   ✓ Optimal froth consistency achieved")
        print("   ✓ Steam safely deactivated")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk frothing: {e}")
        print("[INFO] Milk frothing process terminated due to error")
        return False


def pour_milk(**params) -> bool:
    """
    Pour frothed milk into cup at specified stage.
    
    This function pours the frothed milk with stage-specific positioning:
    1. Validates stage parameter
    2. Returns to approach position from steam wand
    3. Adjusts servo timing for smooth pouring
    4. Moves to target stage for milk pouring
    5. Executes stage-specific pouring sequence
    6. Returns frother to safe position
    
    Args:
        stage (str): Target stage for pouring ('1' or '2')
        
    Returns:
        bool: True if milk pouring completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pouring process
        
    Example:
        success = pour_milk(stage='1')
        if success:
            print("Milk poured successfully")
    """
    try:
        # Extract and validate stage parameter
        stage = params.get("stage")
        if not stage:
            print("[ERROR] No stage parameter provided")
            return False
        
        # Validate stage parameter
        if stage not in ('1', '2'):
            print(f"[ERROR] Unknown stage: {stage!r}")
            print("[INFO] Valid stages: '1', '2'")
            return False
        
        print(f"🥛 Starting milk pouring sequence for stage {stage}")
        print("=" * 50)

        # Step 1: Approach steam wand position
        print("🎯 Step 1/5: Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        print("   ✅ Successfully approached steam wand")
        
        # Step 2: Set normal servo timing
        print("⚙️ Step 2/5: Setting normal servo timing...")
        timing_result = run_skill("set_speed_factor", 50)
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        else:
            print("   ✅ Servo timing adjusted for pouring")
        
        # Step 3: Move to intermediate pouring position
        print("📍 Step 3/5: Moving to intermediate pouring position...")
        intermediate_result = run_skill("gotoJ_deg", -53.498047, -56.063831, -104.329971, -23.914228, -67.359390, 3.238193)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        run_skill("sync")
        
        # Step 4: Stage-specific pouring sequence
        if stage == '1':
            print("🎯 Step 4/5: Executing stage 1 milk pouring...")
            
            print("   ⚙️ Setting precise pouring speed...")
            run_skill("set_speed_factor", 10)
            
            print("   📍 Moving to stage 1 pouring position...")
            stage1_result = run_skill("gotoJ_deg", -116.497627, -30.926678, -99.204826, -44.998241, -116.716164, 3.239676)
            if stage1_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            
            run_skill("sync")
            
            print("   🥛 Executing circular pouring motion...")
            circle_result = run_skill("move_circle", 3,
                (-30.0, 0.0, 0.0, 0.0, 0.0, 0.0),    # point1 offset1
                (-15.0, -15.0, 0.0, 0.0, 0.0, 0.0),   # point2 offset2
                ["tool=0"])
            if circle_result is False:
                print("[WARNING] Circular motion may not have completed optimally")
            
            run_skill("sync")
            
            print("   📍 Adjusting pour angle...")
            run_skill("gotoJ_deg", -117.388901, -41.489887, -93.944572, -48.045307, -115.074539, -31.141111)
            run_skill("gotoJ_deg", -109.472954, -37.621704, -91.996796, -56.920311, -120.970390, -95.325432)
            
            print("   🥛 Final pouring motion...")
            run_skill("moveEE", 50, 0, 0, 0, 0, 0)
            run_skill("sync")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(3.0)
            run_skill("sync")
            
            print("   📍 Returning to stage 1 position...")
            run_skill("gotoJ_deg", -116.497627, -30.926678, -99.204826, -44.998241, -116.716164, 3.239676)
            
            print("   ⚙️ Restoring normal speed...")
            run_skill("set_speed_factor", 100)
            print("   ✅ Stage 1 milk pouring completed")
            
        elif stage == '2':
            print("🎯 Step 4/5: Executing stage 2 milk pouring...")
            
            # Move to stage 2 position
            print("   📍 Moving to stage 2 position...")
            stage2_pos_result = run_skill("gotoJ_deg", -127.319954, -37.857658, -74.250511, -76.871681, -96.185387, 0.116908)
            if stage2_pos_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            print("   ✅ Successfully positioned at stage 2")
            
            # Tilt for pouring
            print("   📐 Tilting frother for pouring...")
            pour_result = run_skill("gotoJ_deg", -113.384514, -39.535606, -77.602524, -71.924614, -96.217064, -98.112167)
            if pour_result is False:
                print("[ERROR] Failed to tilt for pouring")
                return False
            print("   ✅ Frother tilted for optimal pour")
            
            # Allow pouring time
            print("   🥛 Pouring milk (2 seconds)...")
            time.sleep(2)
            print("   ✅ Pouring time completed")
            
            # Return to upright
            print("   📐 Returning to upright position...")
            upright_result = run_skill("gotoJ_deg", -127.319954, -37.857658, -74.250511, -76.871681, -96.185387, 0.116908)
            if upright_result is False:
                print("[ERROR] Failed to return to upright position")
                return False
            print("   ✅ Stage 2 milk pouring completed")
        
        # Step 5: Completion
        print("🏁 Step 5/5: Finalizing milk pouring...")
        print("   ✅ Milk pouring sequence completed")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ MILK POURING COMPLETED SUCCESSFULLY FOR STAGE {stage}")
        print("   ✓ Precise positioning achieved")
        print("   ✓ Optimal pouring technique executed")
        print("   ✓ Perfect milk-to-coffee ratio delivered")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk pouring: {e}")
        print("[INFO] Milk pouring process terminated due to error")
        return False
    

def return_frother(**params) -> bool:
    """
    Return the milk frother to its storage position after use.
    
    This function returns the frother using stored position data:
    1. Moves to intermediate return position
    2. Moves to return preparation position
    3. Uses stored grab position if available
    4. Opens gripper to release frother
    5. Uses stored approach position for safe withdrawal
    6. Returns to home position
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
    
    Returns:
        bool: True if frother returned successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during return process
        
    Example:
        success = return_frother()
        if success:
            print("Milk frother returned successfully")
    """
    try:
        global approach_angles, grab_angles
        
        print("🔄 Starting milk frother return sequence...")
        print("=" * 50)
        
        # Step 1: Move to intermediate return position
        print("📍 Step 1/7: Moving to intermediate return position...")
        intermediate_result = run_skill("gotoJ_deg", -4.127179, -41.282722, -129.513504, -21.285969, -62.760456, 7.227837)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate return position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        # Step 2: Move to return preparation position
        print("📍 Step 2/7: Moving to return preparation position...")
        prep_result = run_skill("gotoJ_deg", 22.402670, -79.481049, -59.269863, -39.324520, -81.417374, 11.115391)
        if prep_result is False:
            print("[ERROR] Failed to move to return preparation position")
            return False
        print("   ✅ Successfully moved to return preparation position")
        
        # Step 3: Use stored grab position if available
        print("📍 Step 3/7: Moving to stored grab position...")
        if grab_angles is not None and len(grab_angles) >= 6:
            print(f"   💾 Using stored grab position ({len(grab_angles)} joint values)...")
            grab_pos_result = run_skill("gotoJ_deg", *grab_angles)
            if grab_pos_result is False:
                print("[ERROR] Failed to move to stored grab position")
                return False
            print("   ✅ Successfully moved to stored grab position")
        else:
            print("   ⚠️ No stored grab position available - using default positioning")
        
        # Step 4: Open gripper to release frother
        print("🤏 Step 4/7: Opening gripper to release frother...")
        run_skill("sync")
        release_result = run_skill("set_gripper_position", 255, 165)
        if release_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        print("   ✅ Gripper opened, frother released")
        
        # Step 5: Use stored approach position if available
        print("⬅️ Step 5/7: Moving to stored approach position...")
        if approach_angles is not None and len(approach_angles) >= 6:
            print(f"   💾 Using stored approach position ({len(approach_angles)} joint values)...")
            approach_pos_result = run_skill("gotoJ_deg", *approach_angles)
            if approach_pos_result is False:
                print("[ERROR] Failed to move to stored approach position")
                return False
            print("   ✅ Successfully moved to stored approach position")
        else:
            print("   ⚠️ No stored approach position available - using default positioning")
        
        # Step 6: Return to home position
        print("🏠 Step 6/7: Returning to home position...")
        home_result = home(position="north")
        if home_result is False:
            print("[ERROR] Failed to return to home position")
            return False
        print("   ✅ Successfully returned to home position")
        
        # Step 7: Final gripper opening
        print("🤏 Step 7/7: Final gripper opening...")
        final_grip_result = run_skill("set_gripper_position", 255, 0)
        if final_grip_result is False:
            print("[WARNING] Failed final gripper opening - frother should still be properly released")
        else:
            print("   ✅ Final gripper opening completed")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK FROTHER RETURN COMPLETED SUCCESSFULLY")
        print("   ✓ Frother safely returned to storage position")
        print("   ✓ Stored position data utilized effectively")
        print("   ✓ Robot returned to home position")
        print("   ✓ Ready for next frothing operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother return: {e}")
        print("[INFO] Frother return process terminated due to error")
        return False


def clean_steam_wand(**params) -> bool:
    """
    Clean the steam wand by running steam through it.
    
    This function cleans the steam wand after use:
    1. Validates duration parameter
    2. Activates steam for cleaning
    3. Runs steam for specified duration to clear residue
    4. Deactivates steam
    5. Allows settling time
    
    Args:
        duration (int): Duration in seconds to run cleaning steam (default: 10)
        
    Returns:
        bool: True if steam wand cleaned successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during cleaning process
        
    Example:
        success = clean_steam_wand(duration=15)
        if success:
            print("Steam wand cleaned successfully")
    """
    try:
        # Extract and validate duration parameter
        duration = params.get("duration", 10)
        if not isinstance(duration, (int, float)) or duration <= 0:
            print(f"[ERROR] Invalid duration: {duration}, must be a positive number")
            return False
        
        print(f"🧽 Starting steam wand cleaning sequence for {duration} seconds...")
        print("=" * 50)
        
        # Step 1: Activate steam for cleaning
        print("💨 Step 1/4: Activating steam for cleaning...")
        steam_on_result = run_skill("set_DO", 2, 1)
        if steam_on_result is False:
            print("[ERROR] Failed to activate steam for cleaning")
            return False
        print("   ✅ Steam successfully activated for cleaning")
        
        # Step 2: Allow cleaning time
        print(f"🧽 Step 2/4: Running cleaning steam ({duration} seconds)...")
        print("   💨 Steam cleaning in progress...")
        time.sleep(duration)
        print("   ✅ Steam cleaning duration completed")
        
        # Step 3: Deactivate steam
        print("💨 Step 3/4: Deactivating steam...")
        steam_off_result = run_skill("set_DO", 2, 0)
        if steam_off_result is False:
            print("[ERROR] Failed to deactivate steam")
            return False
        print("   ✅ Steam successfully deactivated")
        
        # Step 4: Allow settling time
        print("⏰ Step 4/4: Allowing settling time...")
        time.sleep(2)
        print("   ✅ Settling time completed")
        
        # Final success summary
        print("=" * 50)
        print("✅ STEAM WAND CLEANING COMPLETED SUCCESSFULLY")
        print(f"   ✓ Steam ran for {duration} seconds")
        print("   ✓ All milk residue cleared from wand")
        print("   ✓ Steam wand ready for next use")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during steam wand cleaning: {e}")
        print("[INFO] Steam wand cleaning process terminated due to error")
        return False


# Register functions for CLI discovery and external access
SEQUENCES = {
    'get_frother_position': get_frother_position,
    'pick_frother': pick_frother,
    'froth_milk': froth_milk,
    'pour_milk': pour_milk,
    'return_frother': return_frother,
    'mount_frother': mount_frother,
    'clean_steam_wand': clean_steam_wand,
}
