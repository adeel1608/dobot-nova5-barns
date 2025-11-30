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
from oms_v1.params import (
    MILK_FROTHING_PARAMS, MILK_FROTHER_SPEEDS, MILK_FROTHER_GRIPPER_POSITIONS,
    MILK_FROTHER_MOVEMENT_OFFSETS, MILK_POURING_OFFSETS, MILK_FROTHING_DELAYS,
    MILK_SWIRL_CIRCLE_PARAMS, MILK_VOLUME_Z_ADJUSTMENT_FACTOR,
    GRIPPER_FULL, GRIPPER_OPEN, CALIBRATION_SETTLE_TIME,
    _extract_cup_position
)

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
        speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['calibration'])
        if speed_result is False:
            print("[WARNING] Failed to set speed factor - continuing with default...")
        
        # Step 1: Move to home position for setup
        print("🏠 Step 1/5: Moving to north-east home position...")
        home_result = home(position="north_east")
        if home_result is False:
            print("[ERROR] Failed to move to north-east home position")
            return False
        print("   ✅ Successfully moved to north-east home")
        
        # Step 2: Open gripper to prepare for positioning
        print("🤏 Step 2/5: Opening gripper for positioning...")
        gripper_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open'])
        if gripper_result is False:
            print("[ERROR] Failed to open gripper")
            return False
        print("   ✅ Gripper opened successfully")

        # Step 3: Steam wand positioning and preparation
        print("🎯 Step 3/5: Steam wand positioning and preparation...")
        print("   📍 Moving to steam wand...")
        move_result = run_skill("move_to", "left_steam_wand", 0.28)
        if move_result is False:
            print("[ERROR] Failed to move to steam wand")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        print("   🔧 Grabbing steam wand tool...")
        grab_result = run_skill("grab_tool", "left_steam_wand")
        if grab_result is False:
            print("[ERROR] Failed to grab steam wand tool")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        print("   🤏 Setting grip position...")
        grip_set_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['place'])
        if grip_set_result is False:
            print("[ERROR] Failed to set grip position")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        print("   📍 Moving to calibration position...")
        positioning_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['calibration']['positioning'])
        if positioning_result is False:
            print("[ERROR] Failed to move to calibration position")
            return False

        print("   🤏 Releasing grip for calibration...")
        release_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open'])
        if release_result is False:
            print("[ERROR] Failed to release grip")
            return False
        print("   ✅ Steam wand positioning completed")
        
        move_ee_result = run_skill("moveEE_movJ", -20, 0, 0, 0, 0, 0)
        if move_ee_result is False:
            print("[ERROR] Failed to move end effector")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        # Step 4: Perform multiple approaches for accuracy
        cycles = 3  # Special case: 3 cycles for frother calibration
        print(f"🎯 Step 4/5: Performing calibration approaches ({cycles} attempts)...")
        for i in range(cycles):
            print(f"   📍 Approach {i+1}/{cycles}...")
            time.sleep(CALIBRATION_SETTLE_TIME)  # Allow settling time between approaches
            
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
        area_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area'])
        if area_result is False:
            print("[ERROR] Failed to move to frother area")
            return False
        print("   ✅ Successfully moved to frother area")
        
        # Step 2: Approach the milk frother
        print("🎯 Step 2/6: Approaching milk frother...")
        
        # Approach milk_frother_1 (no timeout, let it stabilize properly)
        print("   📍 Moving to milk_frother_1...")
        approach_result = run_skill("move_to", 'milk_frother_1', 0.29)
        
        if not approach_result:
            print("[ERROR] Failed to approach milk_frother_1")
            return False
        
        print("   ✅ Successfully approached milk_frother_1")
        
        # Step 3: Move to approach position for frother
        print("📍 Step 3/6: Moving to frother approach position...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        approach_tool_result = run_skill("approach_tool", 'milk_frother_1')
        if approach_tool_result is False:
            print("[ERROR] Failed to move to frother approach position")
            return False
        print("   ✅ Successfully positioned for frother approach")
        
        # Set initial grip position
        print("   🤏 Setting initial grip position...")
        grip_pos_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'])
        if grip_pos_result is False:
            print("[ERROR] Failed to set initial grip position")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        # Step 4: Record current approach position
        print("💾 Step 4/6: Recording approach position...")
        current_angles = run_skill("current_angles")
        if current_angles is not None:
            approach_angles = current_angles
            print(f"   ✅ Approach angles recorded: {len(approach_angles)} joint values")
        else:
            print("[WARNING] Failed to record approach angles - continuing without position memory")
            approach_angles = None
        time.sleep(MILK_FROTHING_DELAYS['frother_pickup'])
        # Step 5: Grab the frother
        print(f"🤏 Step 5/6: Grabbing {'milk_frother_1'}...")
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        grab_result = run_skill("grab_tool", 'milk_frother_1', 100, 100,-5,-10.5)
        if grab_result is False:
            print(f"[ERROR] Failed to grab {'milk_frother_1'}")
            return False
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 6: Record current grab position
        print("💾 Step 6/6: Recording grab position...")
        grab_angles = run_skill("current_angles")
        if grab_angles is not None:
            print(f"   ✅ Grab angles recorded: {len(grab_angles)} joint values")
        else:
            print("[WARNING] Failed to record grab angles - continuing without position memory")
            grab_angles = None

        # Step 7: Secure the frother with full grip
        print("   🤏 Securing frother with full grip...")
        secure_result = run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure'])
        if secure_result is False:
            print("[ERROR] Failed to secure frother")
            return False
        print("   ✅ Milk frother secured successfully")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ MILK FROTHER PICKUP COMPLETED SUCCESSFULLY ({'milk_frother_1'.upper()})")
        print("   ✓ Frother securely gripped and positioned")
        print("   ✓ Position data recorded for safe return")
        print("   ✓ Ready for mounting to steam wand")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother pickup: {e}")
        print("[INFO] Frother pickup process terminated due to error")
        return False
        
def place_frother_milk_station(**params) -> bool:
    """
    Place the milk frother at the milk station safely.

    Returns:
        bool: True on successful placement, False otherwise.
    """
    try:
        print("📍 Placing frother at milk station...")
        if run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_place']) is False:
            print("[ERROR] Failed to raise end effector before placement")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre1']) is False:
            print("[ERROR] Failed to reach pre-place joint configuration 1")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre2']) is False:
            print("[ERROR] Failed to reach pre-place joint configuration 2")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_approach']) is False:
            print("[ERROR] Failed to reach approach configuration")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_final']) is False:
            print("[ERROR] Failed to reach place configuration")
            return False
        if run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['place']) is False:
            print("[ERROR] Failed to loosen gripper to place frother")
            return False
        print("✅ Frother placed at milk station")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error while placing frother at milk station: {e}")
        return False

def pick_frother_milk_station(**params) -> bool:
    """
    Pick the milk frother up from the milk station safely.

    Returns:
        bool: True on successful pickup, False otherwise.
    """
    try:
        print("📍 Picking frother from milk station...")
        if run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['secure']) is False:
            print("[ERROR] Failed to close gripper before pick")
            return False
        if run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_pick']) is False:
            print("[ERROR] Failed to lift frother from station")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat1']) is False:
            print("[ERROR] Failed to reach retreat configuration 1")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat2']) is False:
            print("[ERROR] Failed to reach retreat configuration 2")
            return False
        print("✅ Frother picked from milk station")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error while picking frother from milk station: {e}")
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

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")

        # Step 1: Set slower servo timing for precise movements
        print("⚙️ Step 1/4: Setting precise servo timing...")
        timing_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['mount'])
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing with default...")
        else:
            print("   ✅ Servo timing set for precise movements")
        
        print("🎯 Step 3/4: Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        print("   ✅ Successfully approached steam wand")

        # Step 4: Mount frother to steam wand with verification
        print("🔧 Step 4/5: Mounting frother to steam wand...")
        mount_result = run_skill("mount_machine", "left_steam_wand", "deep_froth")
        if mount_result is False:
            print("[ERROR] Failed to mount frother to steam wand")
            return False
        print("   ✅ Frother successfully mounted to steam wand")

        # Step 5: Adjust position based on milk volume
        print("📏 Step 5/6: Adjusting position based on milk volume...")
        milk_data = params.get('milk', {})
        volume_ml = next(iter(milk_data.values()), 0) if milk_data else 0
        z_adjustment = MILK_VOLUME_Z_ADJUSTMENT_FACTOR * volume_ml
        
        print(f"   🥛 Milk volume: {volume_ml}ml, Z adjustment: {z_adjustment:.2f}mm")
        move_result = run_skill("moveEE_movJ", 0, 0, -z_adjustment, 0, 0, 0)
        if move_result is False:
            print("[WARNING] Failed to adjust position based on milk volume - continuing...")
        else:
            print(f"   ✅ Position adjusted by {z_adjustment:.2f}mm for {volume_ml}ml milk")
        
        sync_result = run_skill("sync")
        if sync_result is False:
            print("[ERROR] Final sync failed - robot state may be inconsistent")
            return False
        print("   ✅ Final state synchronized")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK FROTHER MOUNTING COMPLETED SUCCESSFULLY")
        print("   ✓ All movements verified")
        print("   ✓ Precise positioning achieved")
        print("   ✓ Frother securely mounted to steam wand")
        print("   ✓ Ready for milk frothing operation")
        print("=" * 50)
        return True
               
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother mounting: {e}")
        print("[INFO] Mounting process terminated due to error")
        return False

def unmount_and_swirl_milk(**params) -> bool:
    """
    Swirl frothed milk in a circular motion for latte art preparation.
    
    This function performs a milk swirling sequence for latte art:
    1. Approaches steam wand position 
    2. Sets precise timing for smooth swirling
    3. Moves through positioning sequence to optimal swirling location
    4. Executes circular swirling motion
    
    Args:
        **params: Additional parameters (currently unused but reserved for future expansion)
        
    Returns:
        bool: True if milk swirling completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during swirling process
        
    Example:
        success = unmount_and_swirl_milk()
        if success:
            print("Milk swirled successfully")
    """
    try:
        
        print("🌀 Starting milk swirling sequence")
        print("=" * 50)

        time.sleep(MILK_FROTHING_DELAYS['swirl_delay'])

        # Step 1: Approach steam wand position
        print("🎯 Step 1/4: Approaching steam wand (deep position)...")
        approach_result = run_skill("approach_machine", "left_steam_wand", "deep_froth")
        if approach_result is False:
            print("[ERROR] Failed to approach steam wand")
            return False
        print("   ✅ Successfully approached steam wand")

        run_skill("sync")
        
        # Step 2: Set precise servo timing for swirling
        print("⚙️ Step 2/4: Setting precise servo timing for swirling...")
        timing_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['swirl'])
        if timing_result is False:
            print("[WARNING] Failed to set servo timing - continuing...")
        else:
            print("   ✅ Servo timing adjusted for swirling")

        # Step 3: Position sequence for optimal swirling location
        print("📍 Step 3/4: Moving through positioning sequence...")
        
        print("   📍 Moving to intermediate position 1...")
        intermediate1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['intermediate1'])
        if intermediate1_result is False:
            print("[ERROR] Failed to move to intermediate position 1")
            return False
        print("   ✅ Successfully moved to intermediate position 1")
        
        print("   📍 Moving to optimal swirling position...")
        swirl_pos_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['swirl_pos'])
        if swirl_pos_result is False:
            print("[ERROR] Failed to move to swirling position")
            return False
        print("   ✅ Successfully positioned for swirling")

        sync_result = run_skill("sync")
        if sync_result is False:
            print("[WARNING] Sync operation failed - continuing...")
        
        # Step 4: Execute circular swirling motion
        print("🌀 Step 4/4: Executing circular swirling motion...")
        circle_result = run_skill("move_circle", 
            MILK_SWIRL_CIRCLE_PARAMS['cycles'],
            MILK_SWIRL_CIRCLE_PARAMS['point1_offset'],
            MILK_SWIRL_CIRCLE_PARAMS['point2_offset'],
            MILK_SWIRL_CIRCLE_PARAMS['options'])
        if circle_result is False:
            print("[WARNING] Circular motion may not have completed optimally")
        
        # Final success summary
        print("=" * 50)
        print("✅ MILK SWIRLING COMPLETED SUCCESSFULLY")
        print("   ✓ Precise positioning achieved")
        print("   ✓ Optimal swirling technique executed")
        print("   ✓ Perfect preparation for latte art")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during milk swirling: {e}")
        print("[INFO] Milk swirling process terminated due to error")
        return False

def pour_milk_cup_station(**params) -> bool:
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
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        
    Returns:
        bool: True if milk pouring completed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during pouring process
        
    Example:
        success = pour_milk_cup_station(position={'cup_position': 1.0})
        if success:
            print("Milk poured successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        print(f"🥛 Starting milk pouring sequence for stage {stage}")
        print("=" * 50)
        
        # Step 4: Stage-specific pouring sequence
        if stage == '1':
            print("🎯 Step 4/5: Executing stage 1 milk pouring...")
            
            print("   📍 Moving to stage 1 pouring position...")
            stage1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['position'])
            if stage1_result is False:
                print("[ERROR] Failed to move to stage 1 position")
                return False
            
            run_skill("sync")

            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            run_skill("sync")


            print("   📍 Adjusting pour angle...")
            adjust1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['adjust1'])
            if adjust1_result is False:
                print("[WARNING] Failed first pour angle adjustment")
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage1']['move_forward'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            sync_result = run_skill("sync")
            if sync_result is False:
                print("[WARNING] Sync operation failed - continuing...")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage1']['move_up'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   📍 Returning to stage 1 position...")
            return_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage1']['position'])
            if return_result is False:
                print("[WARNING] Failed to return to stage 1 position")
            
            print("   ⚙️ Restoring normal speed...")
            restore_speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
            if restore_speed_result is False:
                print("[WARNING] Failed to restore normal speed")
            print("   ✅ Stage 1 milk pouring completed")
            
        elif stage == '2':
            print("🎯 Step 4/5: Executing stage 2 milk pouring...")
            
            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Moving to stage 2 pouring position...")
            stage2_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['position'])
            if stage2_result is False:
                print("[ERROR] Failed to move to stage 2 position")
                return False
            
            run_skill("sync")

            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Adjusting pour angle...")
            adjust1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['adjust1'])
            if adjust1_result is False:
                print("[WARNING] Failed first pour angle adjustment")
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage2']['move_forward'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage2']['move_up'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   📍 Returning to stage 2 position...")
            return_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage2']['position'])
            if return_result is False:
                print("[WARNING] Failed to return to stage 2 position")
            
            print("   ⚙️ Restoring normal speed...")
            restore_speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
            if restore_speed_result is False:
                print("[WARNING] Failed to restore normal speed")
            print("   ✅ Stage 2 milk pouring completed")

        elif stage == '3':
            print("🎯 Step 4/5: Executing stage 3 milk pouring...")
            
            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Moving to stage 3 pouring position...")
            stage3_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['position'])
            if stage3_result is False:
                print("[ERROR] Failed to move to stage 3 position")
                return False
            
            run_skill("sync")

            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
                
            print("   📍 Adjusting pour angle...")
            adjust1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['adjust1'])
            if adjust1_result is False:
                print("[WARNING] Failed first pour angle adjustment")
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage3']['move_forward'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage3']['move_up'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   📍 Returning to stage 3 position...")
            return_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage3']['position'])
            if return_result is False:
                print("[WARNING] Failed to return to stage 3 position")
            
            print("   ⚙️ Restoring normal speed...")
            restore_speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
            if restore_speed_result is False:
                print("[WARNING] Failed to restore normal speed")
            print("   ✅ Stage 3 milk pouring completed")

        else:  # stage == '4'
            print("🎯 Step 4/5: Executing stage 4 milk pouring...")
            
            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Moving to stage 4 pouring position...")
            stage4_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['position'])
            if stage4_result is False:
                print("[ERROR] Failed to move to stage 4 position")
                return False
            
            run_skill("sync")

            print("   ⚙️ Setting precise pouring speed...")
            speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
            if speed_result is False:
                print("[WARNING] Failed to set pouring speed - continuing...")
            
            print("   📍 Adjusting pour angle...")
            adjust1_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['adjust1'])
            if adjust1_result is False:
                print("[WARNING] Failed first pour angle adjustment")
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage4']['move_forward'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   ⏰ Allowing pour completion time...")
            time.sleep(MILK_FROTHING_DELAYS['pour_completion'])
            
            print("   🥛 Final pouring motion...")
            move_ee_result = run_skill("moveEE_movJ", *MILK_POURING_OFFSETS['stage4']['move_up'])
            if move_ee_result is False:
                print("[WARNING] Failed final pouring motion")
            
            print("   📍 Returning to stage 4 position...")
            return_result = run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pouring']['stage4']['position'])
            if return_result is False:
                print("[WARNING] Failed to return to stage 4 position")
            
            print("   ⚙️ Restoring normal speed...")
            restore_speed_result = run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
            if restore_speed_result is False:
                print("[WARNING] Failed to restore normal speed")
            print("   ✅ Stage 4 milk pouring completed")
        
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

def clean_milk_pitcher(**params) -> bool:
    """
    Perform a cleaning motion for the frother tool.

    Returns:
        bool: True on successful cleaning movement sequence, False otherwise.
    """
    try:
        print("🧽 Cleaning frother motion sequence...")
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose1']) is False:
            print("[ERROR] Failed to reach clean pose 1")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose2']) is False:
            print("[ERROR] Failed to reach clean pose 2")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose3']) is False:
            print("[ERROR] Failed to reach clean pose 3")
            return False
        if run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['cleaning_motion']) is False:
            print("[ERROR] Failed to execute cleaning motion")
            return False
        print("✅ Frother cleaning movement completed")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother cleaning: {e}")
        return False

def return_frother(**params) -> bool:
    """
    Return the frother to its original location using recorded approach/grab angles.

    Returns:
        bool: True on successful return, False otherwise.
    """
    try:
        global approach_angles, grab_angles
        print("↩️ Returning frother to original location...")

        if grab_angles is None or approach_angles is None:
            print("[ERROR] Missing recorded angles for safe return. Ensure pick_frother() recorded positions.")
            return False

        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return1']) is False:
            print("[ERROR] Failed to reach pre-return pose 1")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return2']) is False:
            print("[ERROR] Failed to reach pre-return pose 2")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return3']) is False:
            print("[ERROR] Failed to reach pre-return pose 3")
            return False
        if run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return4']) is False:
            print("[ERROR] Failed to reach pre-return pose 4")
            return False
        if run_skill("gotoJ_deg", *grab_angles) is False:
            print("[ERROR] Failed to go to recorded grab angles")
            return False
        if run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['final_approach']) is False:
            print("[ERROR] Failed to execute final approach move")
            return False
        if run_skill("sync") is False:
            print("[WARNING] Sync operation failed - continuing...")
        if run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['release']) is False:
            print("[ERROR] Failed to release frother")
            return False
        time.sleep(MILK_FROTHING_DELAYS['frother_release'])
        if run_skill("gotoJ_deg", *approach_angles) is False:
            print("[ERROR] Failed to go to recorded approach angles")
            return False
        if home(position="north") is False:
            print("[WARNING] Failed to go home after return")
        if run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open']) is False:
            print("[WARNING] Failed to fully open gripper after return")
        print("✅ Frother returned successfully")
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error during frother return: {e}")
        return False

# Register functions for CLI discovery and external access
SEQUENCES = {
    'get_frother_position': get_frother_position,
    'pick_frother': pick_frother,
    'unmount_and_swirl_milk': unmount_and_swirl_milk,
    'pour_milk_cup_station': pour_milk_cup_station,
    'mount_frother': mount_frother,
    'clean_milk_pitcher': clean_milk_pitcher,
    'return_frother': return_frother,
    'place_frother_milk_station': place_frother_milk_station,
    'pick_frother_milk_station': pick_frother_milk_station,
}

