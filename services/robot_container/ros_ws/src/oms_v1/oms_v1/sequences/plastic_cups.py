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
    PLASTIC_CUPS_PARAMS, VALID_CUP_SIZES, DEFAULT_CUP_SIZE, GRIPPER_OPEN, GRIPPER_FULL,
    SPEED_CAREFUL, SPEED_FAST, validate_cup_size, log_step, log_success, log_error, log_info,
    _extract_cup_position,
)


def _normalize_plastic_cup_size(cups_dict: Any) -> str:
    """
    Parse plastic cup size from new JSON format.
    
    Expected format: {'cup_C16': 1.0} or {'cup_C7': 1.0}
    Extracts C7/C9/C12/C16 and maps to plastic cup sizes (7oz/9oz/12oz/16oz).
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value for backward compatibility
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz', '16oz')
    """
    # Handle new dictionary format
    if isinstance(cups_dict, dict):
        # Get the first key from the cups dictionary
        cup_key = next(iter(cups_dict.keys()), None)
        if not cup_key:
            return DEFAULT_CUP_SIZE
        
        # Extract cup code from key like 'cup_C16' -> 'C16'
        cup_key_str = str(cup_key).upper()
        if 'CUP_' in cup_key_str:
            # Extract code after 'CUP_'
            cup_code = cup_key_str.split('CUP_', 1)[1] if 'CUP_' in cup_key_str else cup_key_str
        else:
            cup_code = cup_key_str
        
        # Check if it's a plastic cup (starts with C)
        if not cup_code.startswith('C'):
            return DEFAULT_CUP_SIZE
        
        # Map to size
        size = cup_code
    else:
        # Backward compatibility: handle direct string/value
        if not cups_dict:
            return DEFAULT_CUP_SIZE
        size = str(cups_dict).strip().upper()
    
    # Normalize the size string
    s = str(size).strip().lower()
    mapping = {
        "c7": "7oz",
        "c9": "9oz",
        "c12": "12oz",
        "c16": "16oz",
        "7oz": "7oz",
        "9oz": "9oz",
        "12oz": "12oz",
        "16oz": "16oz",
    }
    return mapping.get(s, DEFAULT_CUP_SIZE)

def _normalize_stage(stage_value: Any) -> Optional[str]:
    if stage_value is None:
        return None
    if isinstance(stage_value, str) and stage_value.startswith("stage_"):
        return stage_value
    try:
        n = int(float(stage_value))
        if n in (1, 2, 3, 4):
            return f"stage_{n}"
    except Exception:
        pass
    return None

def dispense_plastic_cup(**params) -> bool:
    """
    Grab a plastic cup of specified size from the plastic cup dispenser.
    
    This function handles plastic cup pickup for beverages like slushes and iced drinks:
    1. Moves to home position for safe approach
    2. Opens gripper to prepare for cup grab
    3. Navigates to plastic cup dispenser area
    4. Positions for plastic cup grab based on size
    5. Grabs plastic cup with appropriate grip strength
    6. Extracts cup from dispenser safely
    7. Moves plastic cup to safe position ready for beverage preparation
    
    The function supports multiple plastic cup sizes with consistent grabbing sequence:
    - 7oz: Small plastic cups for smaller portions
    - 9oz: Medium plastic cups for standard servings  
    - 12oz: Large plastic cups for generous servings
    - 16oz: Extra large plastic cups for maximum capacity
    
    Args:
        cup_size (str): Size of plastic cup to grab ('7oz', '9oz', '12oz', '16oz')
        
    Returns:
        bool: True if plastic cup grabbed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during cup grabbing process
        
    Example:
        success = dispnese_plastic_cup(cup_size='12oz')
        if success:
            print("12oz plastic cup grabbed successfully")
    """
    try:
        # Extract and validate cup size parameter
        cups_dict = params.get("cups", params.get("cup_size"))
        cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_CUP_SIZE)
        if not cup_size or not validate_cup_size(cup_size):
            return False
        
        # Configuration for each cup size
        CUP_CONFIG = {
            '16oz': {
                'home': 'west',
                'coords': (86.091174,-38.559416,-137.820425,0.249348,-85.974565,-0.271408),
                'gripper': 136,
                'extract_z': -160,
            },
            '12oz': {
                'home': 'west',
                'coords': (126.231637, -41.187070, -144.717680, 11.342104, -45.944142, -3.776550),
                'gripper': 160,
                'extract_z': -160,
            },
            '9oz': {
                'home': 'south_west',
                'coords': (147.070245,-21.439266,-130.331893,-19.117116,-25.228377,-8.200656),
                'gripper': 120,
                'extract_z': -160,
            },
            '7oz': {
                'home': 'south_west',
                'coords': (158.864438,-31.888748,-118.469490,-13.032610,-13.686490,-16.105547),
                'gripper': 161,
                'extract_z': -100.0,
            }
        }
        
        if cup_size not in CUP_CONFIG:
            print(f"[ERROR] Unknown cup size: {cup_size!r}")
            print("[INFO] Valid cup sizes: 7oz, 9oz, 12oz, 16oz")
            return False
        
        config = CUP_CONFIG[cup_size]
        print(f"🥤 Starting plastic cup grab sequence for {cup_size}")
        print("=" * 50)
        
        # Step 1: Move to home position
        log_step(1, 7, f"Moving to {config['home']} home position")
        if not home(position=config['home']):
            log_error(f"Failed to move to {config['home']} home position")
            return False
        log_success(f"Successfully moved to {config['home']} home position", indent=1)
        
        # Step 2: Open gripper fully
        log_step(2, 7, "Opening gripper fully")
        if not run_skill("set_gripper_position", 255, 0):
            log_error("Failed to open gripper")
            return False
        log_success("Gripper opened successfully", indent=1)
        
        # Step 3: Move to plastic cup dispenser area
        print("📍 Step 3/7: Moving to plastic cup dispenser area...")
        if not run_skill("gotoJ_deg", *config['coords']):
            print("[ERROR] Failed to move to plastic cup dispenser area")
            return False
        print("   ✅ Successfully positioned at dispenser area")
        
        # Step 4: Position for plastic cup grab
        print(f"🎯 Step 4/7: Positioning for {cup_size} plastic cup grab...")
        if not run_skill("moveEE", 0.0, 400.0, 0.0, 0, 0, 0):
            print("[ERROR] Failed to move to plastic cup grab position")
            return False
        print("   ✅ Successfully positioned for cup grab")
        
        # Step 5: Grip plastic cup
        print("🤏 Step 5/7: Gripping plastic cup...")
        if not run_skill("set_gripper_position", 255, config['gripper']):
            print("[ERROR] Failed to grip plastic cup")
            return False
        print("   ✅ Plastic cup secured successfully")
        
        run_skill("set_speed_factor", 15)
        run_skill("sync")
        
        # Step 6: Extract plastic cup from dispenser
        print("⬇️ Step 6/7: Extracting plastic cup from dispenser...")
        if not run_skill("moveEE", 0, 0, config['extract_z'], 0, 0, 0):
            print("[ERROR] Failed to extract plastic cup from dispenser")
            return False
        
        run_skill("set_speed_factor", 100)
        run_skill("sync")
        run_skill("moveEE", 0, -400.0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", *config['coords'])
        
        # Step 7: Return to home position
        print(f"📍 Step 7/7: Returning to {config['home']} home position...")
        if not home(position=config['home']):
            print(f"[ERROR] Failed to return to {config['home']} home position")
            return False
        print("   ✅ Successfully moved to home position")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PLASTIC CUP GRAB COMPLETED SUCCESSFULLY FOR {cup_size.upper()}")
        print("   ✓ Cup securely gripped and extracted")
        print("   ✓ Safe positioning achieved")
        print("   ✓ Ready for cold beverage preparation")
        print("=" * 50)
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
        # New format: {'cups': {'cup_C16': 1.0}}
        cups_dict = params.get("cups", params.get("cup_size"))  # Fallback to old format for compatibility
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
        # Extract and validate stage parameter
        stage = params.get("stage")
        if not stage:
            print("[ERROR] No stage parameter provided")
            return False
        
        # Validate stage parameter
        valid_stages = ('1', '2', '3', '4')
        if stage not in valid_stages:
            print(f"[ERROR] Unknown stage: {stage!r}")
            print(f"[INFO] Valid stages: {', '.join(valid_stages)}")
            return False
        
        print(f"🧊 Starting plastic cup with ice placement sequence for stage {stage}")
        print("=" * 50)
        
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
        print(f"✅ PLASTIC CUP WITH ICE PLACEMENT COMPLETED FOR STAGE {stage}")
        print("   ✓ Cup positioned at designated staging area")
        print("   ✓ Safe release and clearance achieved")
        print("   ✓ Robot returned to home position")
        print("   🧊 Iced beverage station ready!")
        print("=" * 50)
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
    
    Args:
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        
    Returns:
        bool: True if cup placement completed successfully, False otherwise
        
    Example:
        success = place_plastic_cup_station(position={'cup_position': 1.0})
        if success:
            print("Plastic cup placed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use
        
        print(f"🥤 Starting plastic cup placement sequence for stage {stage}")
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
            stage_result = run_skill("gotoJ_deg", -87.693741,-44.670253,-121.345483,-21.690320,-92.259001,-0.994262)
        elif stage == "2":
            print("   📍 Positioning for stage 2...")
            stage_result = run_skill("gotoJ_deg", -106.609709,-47.304585,-111.218162,-29.742663,-110.987446,-3.660784)
        elif stage == "3":
            print("   📍 Positioning for stage 3...")
            home(position="south_east")
            stage_result = run_skill("gotoJ_deg", -122.196826,-53.987263,-91.310199,-44.315117,-126.392533,-6.411539)
        elif stage == "4":
            print("   📍 Positioning for stage 4...")
            home(position="south_east")
            stage_result = run_skill("gotoJ_deg", -131.750384,-63.383616,-67.416760,-60.325379,-135.803904,-8.692966)
        
        if not stage_result:
            print(f"[ERROR] Failed to move to stage {stage} position")
            return False
        print(f"   ✅ Successfully positioned at stage {stage}")
        
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
        print("   ✅ Successfully moved up and returned to home")
        
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

#ADD NEW FUNCTION: pick_plastic_cup_station
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
        
        # New format: {'cups': {'cup_C16': 1.0}}
        cups_dict = params.get("cups", params.get("cup_size"))  # Fallback to old format for compatibility
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
            "1": (-86.736873,-42.694084,-139.481187,-5.526080,-91.306231,-0.876971),
            "2": (-111.873309,-43.726202,-126.359794,-18.529009,-116.187576,-4.511501),
            "3": (-129.476135,-49.513914,-104.372032,-36.817109,-133.546833,-8.091070),
            "4": (-139.873113,-57.852132,-81.476933,-53.832194,-143.728644,-11.340573)
        }
        
        # Cup size specific gripper positions
        gripper_positions = {
            "7oz": 145,
            "9oz": 145, 
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

#ADD NEW FUNCTION: place_plastic_cup_sauces
def place_plastic_cup_sauces(**params) -> bool:
    """
    Place the plastic cup at the sauces station.
    """
    try:
        if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
            return False
        if run_skill("gotoJ_deg", -38.389633,-75.079689,-66.372528,-35.134846,-127.236320,-0.949134) is False:
            return False
        if run_skill("gotoJ_deg", -38.389671,-76.306572,-65.615051,-34.665958,-127.237885,-0.949974) is False:
            return False
        if run_skill("set_gripper_position", 255, 0) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_plastic_cup_sauces failed: {e}")
        return False

#ADD NEW FUNCTION: pick_plastic_cup_sauces
def pick_plastic_cup_sauces(**params) -> bool:
    """
    Pick the plastic cup from the sauces station.

    Args:
        cup_size (str): One of '7oz', '9oz', '12oz', '16oz' (required)
    """
    try:
        # Extract cup size from new format: {'cups': {'cup_C16': 1.0}}
        cups_dict = params.get("cups", params.get("cup_size"))  # Fallback to old format for compatibility
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        gripper_positions = {
            "7oz": 145,
            "9oz": 145,
            "12oz": 145,
            "16oz": 118,
        }

        if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
            return False
        if run_skill("gotoJ_deg", -38.389671,-76.306572,-65.615051,-34.665958,-127.237885,-0.949974) is False:
            return False
        if run_skill("gotoJ_deg", -38.389633,-75.079689,-66.372528,-35.134846,-127.236320,-0.949134) is False:
            return False
        if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] pick_plastic_cup_sauces failed: {e}")
        return False

#ADD NEW FUNCTION: place_plastic_cup_milk
def place_plastic_cup_milk(**params) -> bool:
    """
    Place the plastic cup at the milk station.
    """
    try:
        if run_skill("gotoJ_deg", -38.902538,-62.473824,-116.293251,1.105230,-129.698776,-1.780196) is False:
            return False
        if run_skill("gotoJ_deg", -25.013091,-67.535421,-89.029513,-21.410146,-115.827363,-2.388913) is False:
            return False
        if run_skill("gotoJ_deg", -25.013315,-68.191483,-88.700539,-21.083347,-115.828384,-2.389561) is False:
            return False
        if run_skill("set_gripper_position", 255, 0) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_plastic_cup_milk failed: {e}")
        return False

#ADD NEW FUNCTION: pick_plastic_cup_milk
def pick_plastic_cup_milk(**params) -> bool:
    """
    Pick the plastic cup from the milk station.

    Args:
        cup_size (str): One of '7oz', '9oz', '12oz', '16oz' (required)
    """
    try:
        # Extract cup size from new format: {'cups': {'cup_C16': 1.0}}
        cups_dict = params.get("cups", params.get("cup_size"))  # Fallback to old format for compatibility
        cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
        
        if not cup_size:
            print("[ERROR] No cup_size parameter provided")
            return False
        valid_sizes = ("7oz", "9oz", "12oz", "16oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        gripper_positions = {
            "7oz": 145,
            "9oz": 145,
            "12oz": 145,
            "16oz": 118,
        }

        if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
            return False
        if run_skill("gotoJ_deg", -25.013315,-68.191483,-88.700539,-21.083347,-115.828384,-2.389561) is False:
            return False
        if run_skill("gotoJ_deg", -25.013091,-67.535421,-89.029513,-21.410146,-115.827363,-2.388913) is False:
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
