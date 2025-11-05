"""
plastic_cups.py

Defines the plastic cup handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for grabbing and placing plastic cups
in the BARNS coffee automation system, supporting multiple cup sizes for cold beverages
like slushes, iced drinks, and cold brews.
"""

import time
from typing import Dict, Any, Optional
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home
from oms_v1.params import (
    DEFAULT_PLASTIC_CUP_SIZE, validate_cup_size, log_step, log_success, log_error,
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size
)


def _normalize_plastic_cup_size(cups_dict: Any) -> str:
    """
    Parse plastic cup size from new JSON format.
    
    This is a wrapper around the unified _normalize_cup_size function.
    Use this for backward compatibility in plastic cup operations.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz', '16oz')
    """
    return _normalize_cup_size(cups_dict, cup_type='plastic')

def dispense_plastic_cup(**params) -> bool:
    try:
        # Extract and validate cup size parameter using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_PLASTIC_CUP_SIZE)
        if not cup_size or not validate_cup_size(cup_size):
            return False
        
        # Configuration for each cup size
        CUP_CONFIG = {
            '16oz': {
                'home': 'west',
                'coords': (79.037093,-39.242251,-132.377459,-8.192847,-100.849547,-0.020315),
                'gripper': 140,
                'extract_z': -10,
                'extract_z2': -140,
            },
            '12oz': {
                'home': 'west',
                'coords': (117.584348,-38.509737,-144.016102,2.779971,-62.300929,-0.175795),
                'gripper': 160,
                'extract_z': -10,
                'extract_z2': -150,
            },
            '9oz': {
                'home': 'south_west',
                'coords': (145.520738,-21.027729,-128.794233,-29.809187,-34.358134,-0.313671),
                'gripper': 150,
                'extract_z': -10,
                'extract_z2': -110,
            },
            '7oz': {
                'home': 'south_west',
                'coords': (157.029014,-29.678097,-117.343398,-32.416276,-22.856233,-0.528079),
                'gripper': 162,
                'extract_z': -10.0,
                'extract_z2': -90,
            }
        }
        
        if cup_size not in CUP_CONFIG:
            print(f"[ERROR] Unknown cup size: {cup_size!r}")
            print("[INFO] Valid cup sizes: 7oz, 9oz, 12oz, 16oz")
            return False
        
        config = CUP_CONFIG[cup_size]
        print(f"🥤 Starting plastic cup grab sequence for {cup_size}")
        print("=" * 50)
        
        if cup_size == "16oz":
            home(position=config['home'])
            run_skill("set_gripper_position", 255, 255)
            run_skill("gotoJ_deg", *config['coords'])
            run_skill("moveEE", 0.0, 380.0, -50.0, 0, 0, 0)
            run_skill("set_speed_factor", 7)
            run_skill("sync")
            run_skill("moveEE", 0.0, 0.0, 100.0, 0, 0, 0)
            run_skill("moveEE", 0.0, 0.0, -100.0, 0, 0, 0)
            run_skill("set_gripper_position", 255, 0)
            run_skill("moveEE", 0.0, 17.5, 65.0, 0, 0, 0)
            run_skill("set_gripper_position", 255, config['gripper'])
            run_skill("moveEE", 0, -5.0, config['extract_z'], 0, 0, 0)
            run_skill("set_speed_factor", 100)
            run_skill("sync")
            run_skill("moveEE", 0, -5.0, config['extract_z2'], 0, 0, 0)
            run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
            run_skill("gotoJ_deg", *config['coords'])
            home(position=config['home'])
        elif cup_size == "12oz":
            home(position=config['home'])
            run_skill("set_gripper_position", 255, 255)
            run_skill("gotoJ_deg", *config['coords'])
            run_skill("moveEE", 0.0, 380.0, -50.0, 0, 0, 0)
            run_skill("set_speed_factor", 5)
            run_skill("sync")
            run_skill("moveEE", 0.0, 0.0, 100.0, 0, 0, 0)
            run_skill("moveEE", 0.0, 0.0, -100.0, 0, 0, 0)
            run_skill("set_gripper_position", 255, 0)
            run_skill("moveEE", 0.0, 22.5, 59.5, 0, 0, 0)
            run_skill("set_gripper_position", 255, config['gripper'])
            run_skill("moveEE", 0, -5.0, config['extract_z'], 0, 0, 0)
            run_skill("set_speed_factor", 100)
            run_skill("sync")
            run_skill("moveEE", 0, -5.0, config['extract_z2'], 0, 0, 0)
            run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
            run_skill("gotoJ_deg", *config['coords'])
            home(position=config['home'])
        if cup_size == "9oz":
            home(position=config['home'])
            run_skill("set_gripper_position", 255, 0)
            run_skill("gotoJ_deg", *config['coords'])
            run_skill("moveEE", 25.0, 395.0, -32.0, 0, 0, 0)  
            run_skill("set_gripper_position", 255, config['gripper'])
            run_skill("set_speed_factor", 3)
            run_skill("sync")
            run_skill("moveEE", 0, 0.0, config['extract_z'], 0, 0, 0)
            run_skill("set_speed_factor", 100)
            run_skill("sync")
            run_skill("moveEE", 0, 0.0, config['extract_z2'], 0, 0, 0)
            run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
            run_skill("gotoJ_deg", *config['coords'])
            home(position=config['home'])
        if cup_size == "7oz":
            home(position=config['home'])
            run_skill("set_gripper_position", 255, 0)
            run_skill("gotoJ_deg", *config['coords'])
            run_skill("moveEE", 0.0, 403.0, 5.0, 0, 0, 0)  
            run_skill("set_gripper_position", 255, config['gripper'])
            run_skill("set_speed_factor", 7)
            run_skill("sync")
            run_skill("moveEE", 0, 0, config['extract_z'], 0, 0, 0)
            run_skill("set_speed_factor", 100)
            run_skill("sync")
            run_skill("moveEE", 0, 0.0, config['extract_z2'], 0, 0, 0)
            run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
            run_skill("gotoJ_deg", *config['coords'])
            home(position=config['home'])
        
        # Set flag to indicate cup was just dispensed
        _set_cup_dispensed()
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during plastic cup grab: {e}")
        print("[INFO] Plastic cup grab process terminated due to error")
        return False

def go_to_ice(**params) -> bool:
    """
    Get ice for the specified cup size.
    
    This function handles ice dispensing for different cup sizes:
    1. Validates cup size parameter
    2. Moves to appropriate home positions
    3. Executes ice dispensing sequence based on cup size
    
    Args:
        cup_size (str): Size of cup for ice ('7oz', '9oz', '12oz', '16oz')
        
    Returns:
        bool: True if ice dispensing completed successfully, False otherwise
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict)
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        # Validate cup size parameter
        valid_sizes = ('16oz', '12oz', '9oz', '7oz')
        if cup_size not in valid_sizes:
            print(f"[ERROR] Unknown plastic cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        print(f"🧊 Starting ice dispensing sequence for {cup_size}")
        print("=" * 50)
        
        print("🏠 Step 2/3: Moving to north-east home position...")
        if not home(position="north_east"):
            print("[ERROR] Failed to move to north-east home position")
            return False
        print("   ✅ Successfully moved to north-east home")
        
        # Step 3: Execute ice dispensing based on cup size
        print(f"🧊 Step 3/3: Dispensing ice for {cup_size} cup...")
        
        if cup_size == "16oz":
            print("   📍 Positioning for 16oz ice dispensing...")
            pos1_result = run_skill("gotoJ_deg", -34.285637,-96.143585,-85.111305,-52.668182,-77.955963,40.874359)
            if not pos1_result:
                print("[ERROR] Failed to move to first ice position")
                return False
            
            print("   📍 Moving to ice dispensing position...")
            pos2_result = run_skill("gotoJ_deg", -40.901531,-124.078323,-25.869335,-68.229462,-77.965225,40.874393)
            if not pos2_result:
                print("[ERROR] Failed to move to ice dispensing position")
                return False
            print("   ✅ 16oz ice dispensing completed")
            
        elif cup_size == "12oz":
            print("   📍 Positioning for 12oz ice dispensing...")
            # Add 12oz specific positioning here
            pos1_result = run_skill("gotoJ_deg", -34.285637,-96.143585,-85.111305,-52.668182,-77.955963,40.874359)
            if not pos1_result:
                print("[ERROR] Failed to move to first ice position")
                return False
            
            print("   📍 Moving to ice dispensing position...")
            pos2_result = run_skill("gotoJ_deg", -40.901531,-124.078323,-25.869335,-68.229462,-77.965225,40.874393)
            if not pos2_result:
                print("[ERROR] Failed to move to ice dispensing position")
                return False
            print("   ✅ 12oz ice dispensing completed")
            
        elif cup_size == "9oz":
            print("   📍 Positioning for 9oz ice dispensing...")
            # Add 9oz specific positioning here
            pos1_result = run_skill("gotoJ_deg", -34.285637,-96.143585,-85.111305,-52.668182,-77.955963,40.874359)
            if not pos1_result:
                print("[ERROR] Failed to move to first ice position")
                return False
            
            print("   📍 Moving to ice dispensing position...")
            pos2_result = run_skill("gotoJ_deg", -40.901531,-124.078323,-25.869335,-68.229462,-77.965225,40.874393)
            if not pos2_result:
                print("[ERROR] Failed to move to ice dispensing position")
                return False
            print("   ✅ 9oz ice dispensing completed")
            
        elif cup_size == "7oz":
            print("   📍 Positioning for 7oz ice dispensing...")
            # Add 7oz specific positioning here
            pos1_result = run_skill("gotoJ_deg", -34.285637,-96.143585,-85.111305,-52.668182,-77.955963,40.874359)
            if not pos1_result:
                print("[ERROR] Failed to move to first ice position")
                return False
            
            print("   📍 Moving to ice dispensing position...")
            pos2_result = run_skill("gotoJ_deg", -40.901531,-124.078323,-25.869335,-68.229462,-77.965225,40.874393)
            if not pos2_result:
                print("[ERROR] Failed to move to ice dispensing position")
                return False
            print("   ✅ 7oz ice dispensing completed")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ ICE DISPENSING COMPLETED SUCCESSFULLY FOR {cup_size.upper()}")
        print("   ✓ Proper positioning achieved")
        print("   ✓ Ice dispensing sequence executed")
        print("   ✓ Ready for beverage preparation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during ice dispensing: {e}")
        print("[INFO] Ice dispensing process terminated due to error")
        return False

def go_home_with_ice(**params) -> bool:
    try:        
        # Step 1: Initial positioning
        print("📍 Step 1/6: Moving to initial position...")
        pos1_result = run_skill("gotoJ_deg", -34.285637,-96.143585,-85.111305,-52.668182,-77.955963,40.874359)
        if not pos1_result:
            print("[ERROR] Failed to move to initial position")
            return False
        print("   ✅ Successfully moved to initial position")
        
        # Step 2: Move to north-east home
        print("🏠 Step 2/6: Moving to north-east home...")
        if not home(position="north_east"):
            print("[ERROR] Failed to move to north-east home")
            return False
        print("   ✅ Successfully moved to north-east home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ RETURNED HOME WITH ICE SUCCESSFULLY")
        print("   ✓ Robot returned to home position")
        print("   🧊 Ready for next operation")
        print("=" * 50)
        
        # Set flag to indicate cup came from ice (same as after dispense)
        _set_cup_dispensed()
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup placement: {e}")
        print("[INFO] Cup placement process terminated due to error")
        return False

def place_plastic_cup_station(**params) -> bool:
    """
    Place a plastic cup at specified staging area.
    
    This function places a previously grabbed plastic cup at a designated staging area:
    1. Validates cup position parameter
    2. Moves through positioning sequence
    3. Navigates to target stage position
    4. Releases cup and moves up safely
    5. Returns to home position
    
    Auto-detects if called after dispense_plastic_cup or go_home_with_ice to apply height adjustment.
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        
    Returns:
        bool: True if cup placement completed successfully, False otherwise
        
    Example:
        # After dispensing - auto-detects and applies adjustment
        dispense_plastic_cup(cup_size="7oz")
        success = place_plastic_cup_station(position={'cup_position': 1.0}, cups={'cup_C7': 1.0})
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict)
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        # Validate parameters
        valid_sizes = ('7oz', '9oz', '12oz', '16oz')
        
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Starting plastic cup placement sequence for stage {stage}, Size: {cup_size} (after_dispense={after_dispense})")
        print("=" * 50)
        
        # Step 1: Move to north-east home
        print("🏠 Step 1/5: Moving to north-east home...")
        if not home(position="north_east"):
            print("[ERROR] Failed to move to north-east home")
            return False
        print("   ✅ Successfully moved to north-east home")
        
        # Step 2: Move to east home
        print("🏠 Step 2/5: Moving to east home...")
        if not home(position="east"):
            print("[ERROR] Failed to move to east home")
            return False
        print("   ✅ Successfully moved to east home")
        
        # Step 3: Move to stage-specific position
        print(f"🎯 Step 3/5: Moving to stage {stage} position...")
        stage_result = False
        
        if stage == "1":
            print("   📍 Positioning for stage 1...")
            stage_result = run_skill("gotoJ_deg", -82.350214,-52.505954,-126.196674,-1.090475,-82.250159,-0.103364)
        elif stage == "2":
            print("   📍 Positioning for stage 2...")
            stage_result = run_skill("gotoJ_deg", -102.385087,-53.365893,-116.767921,-9.682041,-102.286850,-0.028823)
        elif stage == "3":
            print("   📍 Positioning for stage 3...")
            home(position="south_east")
            stage_result = run_skill("gotoJ_deg", -118.186155,-56.698592,-101.309618,-21.804371,-118.094850,0.033187)
        elif stage == "4":
            print("   📍 Positioning for stage 4...")
            home(position="south_east")
            stage_result = run_skill("gotoJ_deg", -129.712707,-63.412577,-80.074947,-36.306427,-129.636056,0.087909)
        
        if not stage_result:
            print(f"[ERROR] Failed to move to stage {stage} position")
            return False
        print(f"   ✅ Successfully positioned at stage {stage}")
        
        # Apply height adjustment for freshly dispensed cups (7oz and 12oz only)
        if cup_size in ("7oz", "12oz") and after_dispense:
            print(f"   📏 Applying {cup_size} height adjustment for freshly dispensed cup...")
            run_skill("sync")
            run_skill("moveEE", 0.0, 12.5, 0.0, 0, 0, 0)
            run_skill("sync")
        # Step 4: Release cup
        print("🤏 Step 4/5: Releasing plastic cup...")
        release_result = run_skill("set_gripper_position", 50, 0)
        if not release_result:
            print("[ERROR] Failed to release plastic cup")
            return False
        print("   ✅ Cup released successfully")
        
        # Step 5: Move up and return to home
        print("⬆️ Step 5/5: Moving up and returning to home...")
        up_result = run_skill("moveEE", 0, 100, 0, 0, 0, 0)
        if not up_result:
            print("[ERROR] Failed to move up after placement")
            return False
        
        if not home(position="east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully moved up and returned def testto home")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PLASTIC CUP PLACEMENT COMPLETED FOR STAGE {stage}")
        print("   ✓ Cup positioned at designated staging area")
        print("   ✓ Safe release and clearance achieved")
        print("   ✓ Robot returned to home position")
        print("   🥤 Beverage station ready!")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup placement: {e}")
        print("[INFO] Cup placement process terminated due to error")
        return False

def pick_plastic_cup_station(**params) -> bool:
    """
    Pick up a plastic cup from a specific stage and add ice to it.
    
    This function picks up a previously placed plastic cup from a staging area
    and moves it to the ice dispensing station for ice addition.
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        
    Returns:
        bool: True if cup picked and ice added successfully, False otherwise
        
    Example:
        success = pick_plastic_cup_station(position={'cup_position': 1}, cups={'cup_C12': 1.0})
        if success:
            print("Cup picked and ice added successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict)
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        # Validate parameters
        valid_sizes = ('7oz', '9oz', '12oz', '16oz')
        
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        print(f"🥤 Starting cup pickup for ice sequence - Stage: {stage}, Size: {cup_size}")
        print("=" * 50)
        
        # Stage-specific positioning
        stage_positions = {
            "1": (-78.932080,-56.045250,-144.146053,20.408964,-78.839216,-0.137490),
            "2": (-106.650221,-52.696527,-132.105571,4.981028,-106.550667,-0.027144),
            "3": (-125.266858,-53.712242,-114.610245,-11.487462,-125.169459,0.053606),
            "4": (-137.175466,-59.183730,-92.762705,-27.831726,-137.089599,0.121883)
        }
        
        # Cup size specific gripper positions
        gripper_positions = {
            "7oz": 145,
            "9oz": 125, 
            "12oz": 140,
            "16oz": 118
        }
        
        # Ice dispensing positions (common for all stages)
        ice_pos1 = (-34.285637,-96.143585,-85.111305,-52.668182,-77.955963,40.874359)
        ice_pos2 = (-40.901531,-124.078323,-25.869335,-68.229462,-77.965225,40.874393)
        
        # Step 1: Navigate to appropriate home positions
        print("🏠 Step 1/6: Navigating to home positions...")
        if not home(position="north_east"):
            print("[ERROR] Failed to move to north_east home")
            return False
            
        if not home(position="east"):
            print("[ERROR] Failed to move to east home")
            return False
        
        # Additional positioning for stages 3 and 4
        if stage in ("3", "4"):
            if not home(position="south_east"):
                print("[ERROR] Failed to move to south_east home")
                return False
        print("   ✅ Successfully navigated to home positions")
        
        # Step 2: Move to stage-specific position
        print(f"📍 Step 2/6: Moving to stage {stage} position...")
        stage_result = run_skill("gotoJ_deg", *stage_positions[stage])
        if not stage_result:
            print(f"[ERROR] Failed to move to stage {stage} position")
            return False
        print(f"   ✅ Successfully positioned at stage {stage}")
        
        # Step 3: Position for cup pickup
        print("🎯 Step 3/6: Positioning for cup pickup...")
        pickup_result = run_skill("moveEE", 0, -100, 0, 0, 0, 0)
        if not pickup_result:
            print("[ERROR] Failed to position for cup pickup")
            return False
        print("   ✅ Successfully positioned for pickup")
        
        # Step 4: Grip the cup
        print(f"🤏 Step 4/6: Gripping {cup_size} cup...")
        grip_result = run_skill("set_gripper_position", 255, gripper_positions[cup_size])
        if not grip_result:
            print("[ERROR] Failed to grip cup")
            return False
        print("   ✅ Cup gripped successfully")
        
        if cup_position == 3 or cup_position == 4:
            if not home(position="south_east"):
                print("[ERROR] Failed to move to south_east home")
                return False
                
        # Step 5: Return to safe position
        print("🏠 Step 5/6: Returning to safe position...")
        if not home(position="east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully returned to safe position")
        
        # Step 5: Return to safe position
        print("🏠 Step 5/6: Returning to safe position...")
        if not home(position="north_east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully returned to safe position")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ CUP PICKUP FOR ICE COMPLETED SUCCESSFULLY")
        print(f"   ✓ Stage {stage} cup ({cup_size}) picked up")
        print("   ✓ Positioned for ice dispensing")
        print("   ✓ Ready for ice addition")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during cup pickup for ice: {e}")
        print("[INFO] Cup pickup process terminated due to error")
        return False

def place_plastic_cup_sauces(**params) -> bool:
    """
    Place the plastic cup at the sauces station.
    
    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        after_dispense (bool): If True, called after dispense_plastic_cup. If False, called after other operations. Default True.
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Placing {cup_size} plastic cup at sauces station (after_dispense={after_dispense})")
        
        # Scenario branching: 4 sizes × 2 conditions = 8 scenarios
        # Using current implementation for all scenarios (user will update individually)
        if cup_size == "7oz" and after_dispense:
            # 7oz after dispense_plastic_cup/go_home_with_ice
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
        elif cup_size == "7oz" and not after_dispense:
            # 7oz NOT after dispense
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
        elif cup_size == "9oz" and after_dispense:
            # 9oz after dispense
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
            if run_skill("gotoJ_deg", -38.389327,-71.489342,-68.340179,-36.755781,-127.231722,-0.946228) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
        elif cup_size == "9oz" and not after_dispense:
            # 9oz NOT after dispense
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
            if run_skill("gotoJ_deg", -38.389327,-71.489342,-68.340179,-36.755781,-127.231722,-0.946228) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
        elif cup_size == "12oz" and after_dispense:
            # 12oz after dispense
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
        elif cup_size == "12oz" and not after_dispense:
            # 12oz NOT after dispense
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
        elif cup_size == "16oz" and after_dispense:
            # 16oz after dispense
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
        elif cup_size == "16oz" and not after_dispense:
            # 16oz NOT after dispense
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
        
        if run_skill("set_gripper_position", 255, 0) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_plastic_cup_sauces failed: {e}")
        return False

def pick_plastic_cup_sauces(**params) -> bool:
    """
    Pick the plastic cup from the sauces station.

    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        after_dispense (bool): If True, called after dispense_plastic_cup. If False, called after other operations. Default True.
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Picking {cup_size} plastic cup from sauces station (after_dispense={after_dispense})")

        gripper_positions = {
            "7oz": 145,
            "9oz": 145,
            "12oz": 145,
            "16oz": 118,
        }

        # Scenario branching: 4 sizes × 2 conditions = 8 scenarios
        # Using current implementation for all scenarios (user will update individually)
        if cup_size == "7oz" and after_dispense:
            # 7oz after dispense_plastic_cup/go_home_with_ice
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
            
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
        elif cup_size == "7oz" and not after_dispense:
            # 7oz NOT after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
            
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
        elif cup_size == "9oz" and after_dispense:
            # 9oz after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
            if run_skill("gotoJ_deg", -38.389327,-71.489342,-68.340179,-36.755781,-127.231722,-0.946228) is False:
                return False
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
        elif cup_size == "9oz" and not after_dispense:
            # 9oz NOT after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
            if run_skill("gotoJ_deg", -38.389327,-71.489342,-68.340179,-36.755781,-127.231722,-0.946228) is False:
                return False
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
        elif cup_size == "12oz" and after_dispense:
            # 12oz after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
        elif cup_size == "12oz" and not after_dispense:
            # 12oz NOT after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
        elif cup_size == "16oz" and after_dispense:
            # 16oz after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
        elif cup_size == "16oz" and not after_dispense:
            # 16oz NOT after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -38.389671,-77.361420,-64.930023,-34.296524,-127.239227,-0.950627) is False:
                return False
            if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
                return False
        
        return True
    except Exception as e:
        print(f"[ERROR] pick_plastic_cup_sauces failed: {e}")
        return False

def place_plastic_cup_milk(**params) -> bool:
    """
    Place the plastic cup at the milk station.
    
    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        after_dispense (bool): If True, called after dispense_plastic_cup. If False, called after other operations. Default True.
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False
        
        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Placing {cup_size} plastic cup at milk station (after_dispense={after_dispense})")
        
        # Scenario branching: 4 sizes × 2 conditions = 8 scenarios
        # Using current implementation for all scenarios (user will update individually)
        if cup_size == "7oz" and after_dispense:
            # 7oz after dispense_plastic_cup/go_home_with_ice
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
        elif cup_size == "7oz" and not after_dispense:
            # 7oz NOT after dispense
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
        elif cup_size == "9oz" and after_dispense:
            # 9oz after dispense
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
            if run_skill("gotoJ_deg", -25.011840,-64.131473,-90.574055,-23.267948,-115.822112,-2.385243) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
        elif cup_size == "9oz" and not after_dispense:
            # 9oz NOT after dispense
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
            if run_skill("gotoJ_deg", -25.011840,-64.131473,-90.574055,-23.267948,-115.822112,-2.385243) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
        elif cup_size == "12oz" and after_dispense:
            # 12oz after dispense
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
        elif cup_size == "12oz" and not after_dispense:
            # 12oz NOT after dispense
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
        elif cup_size == "16oz" and after_dispense:
            # 16oz after dispense
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
        elif cup_size == "16oz" and not after_dispense:
            # 16oz NOT after dispense
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
        
        if run_skill("set_gripper_position", 255, 0) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_plastic_cup_milk failed: {e}")
        return False

def pick_plastic_cup_milk(**params) -> bool:
    """
    Pick the plastic cup from the milk station.

    Args:
        cups (dict): Cup size dictionary, e.g., {'cup_C12': 1.0}
        after_dispense (bool): If True, called after dispense_plastic_cup. If False, called after other operations. Default True.
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        # Auto-detect if coming from dispense_plastic_cup or go_home_with_ice
        after_dispense = _check_and_clear_cup_dispensed()
        
        print(f"🥤 Picking {cup_size} plastic cup from milk station (after_dispense={after_dispense})")

        gripper_positions = {
            "7oz": 145,
            "9oz": 145,
            "12oz": 145,
            "16oz": 118,
        }

        # Scenario branching: 4 sizes × 2 conditions = 8 scenarios
        # Using current implementation for all scenarios (user will update individually)
        if cup_size == "7oz" and after_dispense:
            # 7oz after dispense_plastic_cup/go_home_with_ice
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
        elif cup_size == "7oz" and not after_dispense:
            # 7oz NOT after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
        elif cup_size == "9oz" and after_dispense:
            # 9oz after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
            if run_skill("gotoJ_deg", -25.011840,-64.131473,-90.574055,-23.267948,-115.822112,-2.385243) is False:
                return False
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
        elif cup_size == "9oz" and not after_dispense:
            # 9oz NOT after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
            if run_skill("gotoJ_deg", -25.011840,-64.131473,-90.574055,-23.267948,-115.822112,-2.385243) is False:
                return False
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
        elif cup_size == "12oz" and after_dispense:
            # 12oz after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
        elif cup_size == "12oz" and not after_dispense:
            # 12oz NOT after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
        elif cup_size == "16oz" and after_dispense:
            # 16oz after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
        elif cup_size == "16oz" and not after_dispense:
            # 16oz NOT after dispense
            if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
                return False
            if run_skill("gotoJ_deg", -25.749681,-69.288589,-90.343018,-18.333176,-116.565109,-2.363535) is False:
                return False
            if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
                return False
        
        return True
    except Exception as e:
        print(f"[ERROR] pick_plastic_cup_milk failed: {e}")
        return False

# Register functions for CLI discovery and external access
SEQUENCES = {
    'dispense_plastic_cup': dispense_plastic_cup,
    'go_to_ice': go_to_ice,
    'go_home_with_ice': go_home_with_ice,
    'place_plastic_cup_station': place_plastic_cup_station,
    'pick_plastic_cup_station': pick_plastic_cup_station,
    'place_plastic_cup_sauces': place_plastic_cup_sauces,
    'pick_plastic_cup_sauces': pick_plastic_cup_sauces,
    'place_plastic_cup_milk': place_plastic_cup_milk,
    'pick_plastic_cup_milk': pick_plastic_cup_milk,
}
