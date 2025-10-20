"""
paper_cups.py

Defines the paper cup handling sequences for coffee service automation.
This module provides comprehensive functions for grabbing, placing, and serving
paper cups in the BARNS coffee automation system, including size-based handling
and staging area management.
"""

import time
from typing import Dict, Any, Optional
from oms_v1.params import GRAB_PAPER_CUP_PARAMS, PLACE_PAPER_CUP_PARAMS
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home

# Predefined home positions for paper cup operations
Espresso_home = (42.159162,16.269149,-135.156441,-81.822150,-49.784457,13.771214)
Espresso_grinder_home = (-32.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)


def _normalize_paper_cup_size(cups_dict: Any) -> str:
    """
    Parse paper cup size from new JSON format.
    
    Expected format: {'cup_H12': 1.0} or {'cup_H7': 1.0}
    Extracts H7/H9/H12 and maps to paper cup sizes (7oz/9oz/12oz).
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value for backward compatibility
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz')
    """
    print(f"[DEBUG _normalize] Input cups_dict: {cups_dict}, type: {type(cups_dict)}")
    
    # Handle new dictionary format
    if isinstance(cups_dict, dict):
        # Get the first key from the cups dictionary
        cup_key = next(iter(cups_dict.keys()), None)
        print(f"[DEBUG _normalize] Extracted cup_key: {cup_key}")
        if not cup_key:
            print("[DEBUG _normalize] No cup_key found, defaulting to 7oz")
            return "7oz"
        
        # Extract cup code from key like 'cup_H12' -> 'H12'
        cup_key_str = str(cup_key).upper()
        print(f"[DEBUG _normalize] cup_key_str (uppercase): {cup_key_str}")
        
        if 'CUP_' in cup_key_str:
            # Extract code after 'CUP_'
            cup_code = cup_key_str.split('CUP_', 1)[1] if 'CUP_' in cup_key_str else cup_key_str
        else:
            cup_code = cup_key_str
        
        print(f"[DEBUG _normalize] Extracted cup_code: {cup_code}")
        
        # Check if it's a paper cup (starts with H)
        if not cup_code.startswith('H'):
            print(f"[DEBUG _normalize] Cup code doesn't start with 'H', defaulting to 7oz")
            return "7oz"
        
        # Map to size
        size = cup_code
    else:
        # Backward compatibility: handle direct string/value
        if not cups_dict:
            print("[DEBUG _normalize] cups_dict is empty/None, defaulting to 7oz")
            return "7oz"
        size = str(cups_dict).strip().upper()
        print(f"[DEBUG _normalize] Backward compatibility mode, size: {size}")
    
    # Normalize the size string
    s = str(size).strip().lower()
    print(f"[DEBUG _normalize] Normalizing '{size}' -> '{s}'")
    
    mapping = {
        "h7": "7oz",
        "h9": "9oz",
        "h12": "12oz",
        "7oz": "7oz",
        "9oz": "9oz",
        "12oz": "12oz",
    }
    result = mapping.get(s, "7oz")
    print(f"[DEBUG _normalize] Final result: {result}")
    return result

def _normalize_stage(stage_value: str) -> str:
    """Return 'stage_1'..'stage_4' from flexible input like 1/1.0/'1'/stage_1."""
    if stage_value is None:
        return "stage_1"
    if isinstance(stage_value, str) and stage_value.startswith("stage_"):
        return stage_value
    try:
        n = int(float(stage_value))
        if n in (1, 2, 3, 4):
            return f"stage_{n}"
    except Exception:
        pass
    return stage_value

def grab_paper_cup(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.
    
    This function performs the complete paper cup grabbing workflow:
    1. Moves to espresso home position
    2. Navigates to paper cup dispenser area (avoiding espresso machine)
    3. Positions for paper cup grab based on size parameters
    4. Executes approach, grip, and retreat sequence
    5. Returns to intermediate position ready for placement
    
    Args:
        size (str): Paper cup size to grab ('7oz', '9oz', '12oz', etc.), defaults to '7oz'
        
    Returns:
        bool: True if paper cup grabbed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during grabbing process
        
    Example:
        success = grab_paper_cup(size='12oz')
        if success:
            print("Paper cup grabbed successfully")
    """
    try:
        # Extract and validate size parameter
        # Handle multiple parameter formats:
        # 1. New format with ingredients: {'ingredients': {'cups': {'cup_H9': 1.0}}}
        # 2. Direct cups dict: {'cups': {'cup_H9': 1.0}}
        # 3. Old format: {'size': '9oz'}
        
        cups_dict = None
        
        # First check if ingredients exists (nested format from scheduler)
        if 'ingredients' in params and isinstance(params['ingredients'], dict):
            cups_dict = params['ingredients'].get('cups')
            print(f"[DEBUG] Extracted cups from ingredients: {cups_dict}")
        
        # Otherwise check for direct cups parameter
        if not cups_dict:
            cups_dict = params.get("cups")
            # If cups is a list (top-level array), try to extract from first item
            if isinstance(cups_dict, list) and len(cups_dict) > 0:
                first_cup = cups_dict[0]
                if isinstance(first_cup, dict) and 'ingredients' in first_cup:
                    cups_dict = first_cup['ingredients'].get('cups')
                    print(f"[DEBUG] Extracted cups from array ingredients: {cups_dict}")
                elif isinstance(first_cup, dict) and 'size' in first_cup:
                    cups_dict = first_cup.get('size')
                    print(f"[DEBUG] Extracted size from array: {cups_dict}")
        
        # Fallback to old 'size' parameter
        if not cups_dict or not isinstance(cups_dict, dict):
            cups_dict = params.get("size")
        
        size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
        print(f"[DEBUG] Final normalized size: {size}")
        if not size:
            print("[ERROR] No size parameter provided")
            return False
            
        cup_params = GRAB_PAPER_CUP_PARAMS.get(str(size))
        
        # Validate parameters and use default if not found
        if not cup_params:
            print(f"[ERROR] Unknown paper cup size: {size!r}")
            print("[INFO] Using default 7oz parameters as fallback")
            cup_params = GRAB_PAPER_CUP_PARAMS.get("7oz")
            if not cup_params:
                print("[ERROR] Default 7oz parameters not found in GRAB_PAPER_CUP_PARAMS")
                print(f"[INFO] Available sizes: {list(GRAB_PAPER_CUP_PARAMS.keys())}")
                return False
        
        print(f"🥤 Starting paper cup grab sequence for size: {size}")
        print("=" * 50)
        
        # Step 1: Move to espresso home position
        print("🏠 Step 1/8: Moving to espresso home position...")
        home_result = run_skill("gotoJ_deg", *Espresso_home)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Twist to avoid hitting the espresso machine during navigation
        print("🔄 Step 2/8: Navigating around espresso machine...")
        twist_result = run_skill("moveJ_deg", 64.012928, 0, 0, 0, 0, 0)
        if twist_result is False:
            print("[ERROR] Failed to twist around espresso machine")
            return False
        print("   ✅ Successfully navigated around espresso machine")
        
        # Step 3: Move to paper cup grabbing area
        print("📍 Step 3/8: Moving to paper cup dispenser area...")
        cup_area_result = run_skill("gotoJ_deg", 120.389030, 22.860609, -73.526848, -39.810959, 90.144394, -154.586288)
        
        if cup_area_result is False:
            print("[ERROR] Failed to move to paper cup dispenser area")
            return False
        print("   ✅ Successfully positioned at paper cup dispenser")
        
        # Step 4: Rotate joint angles to back away before approach
        print("⬅️ Step 4/8: Backing away for approach...")
        if 'twist_back' in cup_params:
            print(f"   📍 Executing twist back movement for {size}")
            twist_back_result = run_skill("moveJ_deg", *cup_params['twist_back'])
            if twist_back_result is False:
                print("[ERROR] Failed to execute twist back movement")
                return False
            print("   ✅ Successfully executed twist back movement")
        else:
            print("   ⏭️ No twist back movement defined for this size")
        
        # Step 5: Move end-effector into approach position
        print("🎯 Step 5/8: Moving to approach position...")
        if 'approach' in cup_params:
            print(f"   📍 Executing approach movement: {cup_params['approach']}")
            approach_result = run_skill("moveEE", *cup_params['approach'])
            if approach_result is False:
                print("[ERROR] Failed to move to approach position")
                return False
            print("   ✅ Successfully moved to approach position")
        else:
            print("   ⏭️ No approach movement defined for this size")
        
        # Step 6: Close gripper to grasp the paper cup
        print("🤏 Step 6/8: Gripping paper cup...")
        if 'grip_width' in cup_params:
            print(f"   📏 Setting gripper width to: {cup_params['grip_width']}")
            run_skill("sync")
            grip_result = run_skill("set_gripper_position", 255, cup_params['grip_width'])
            if grip_result is False:
                print("[ERROR] Failed to grip paper cup")
                return False
            print("   ✅ Successfully gripped paper cup")
        else:
            print("[ERROR] No grip width defined for this size")
            return False
        
        # Step 7: Retract after gripping
        print("⬆️ Step 7/8: Retracting with paper cup...")
        if 'retreat' in cup_params:
            print(f"   📍 Executing retreat movement: {cup_params['retreat']}")
            retreat_result = run_skill("moveEE", *cup_params['retreat'])
            if retreat_result is False:
                print("[ERROR] Failed to retreat with paper cup")
                return False
            print("   ✅ Successfully retracted with paper cup")
        else:
            print("   ⏭️ No retreat movement defined for this size")
        
        # Step 8: Move to intermediate position ready for placement
        print("📍 Step 8/8: Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PAPER CUP GRAB COMPLETED SUCCESSFULLY FOR {size.upper()}")
        print("   ✓ Paper cup securely gripped")
        print("   ✓ Safe navigation around espresso machine")
        print("   ✓ Robot positioned for next operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup grab: {e}")
        print("[INFO] Paper cup grab process terminated due to error")
        return False

def place_paper_cup(**params) -> bool:
    """
    Place a paper cup at the specified staging area.
    
    This function places a previously grabbed paper cup at a designated serving stage:
    1. Moves from intermediate position to staging area
    2. Adjusts orientation for precise placement
    3. Positions paper cup at target location
    4. Releases paper cup and retreats safely
    5. Returns to staging home position
    
    Args:
        stage (str): Target stage for paper cup placement ('stage_1', 'stage_2', etc.), defaults to 'stage_1'
        
    Returns:
        bool: True if paper cup placed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during placement process
        
    Example:
        success = place_paper_cup(stage='stage_1')
        if success:
            print("Paper cup placed successfully")
    """
    try:
        # Extract and validate stage parameter
        try:
            stage = _normalize_stage(params.get("stage", "1"))  # Default to stage 1
        except Exception as e:
            print(f"[ERROR] Failed to normalize stage: {e}")
            return False
        if not stage:
            stage = "stage_1"  # Final fallback
            
        stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
        
        # Validate stage parameter
        if not stage_params:
            print(f"[ERROR] Unknown stage: {stage!r}")
            print(f"[INFO] Available stages: {list(PLACE_PAPER_CUP_PARAMS.keys())}")
            return False
        
        print(f"📍 Starting paper cup placement sequence for: {stage}")
        print("=" * 50)
        
        # Step 1: Start from intermediate position
        print("📍 Step 1/7: Moving to intermediate position...")
        intermediate_result = run_skill("gotoJ_deg", 88.657143, 21.041538, -74.451630, -36.522381, 90.145508, -91.183128)
        if intermediate_result is False:
            print("[ERROR] Failed to move to intermediate position")
            return False
        print("   ✅ Successfully moved to intermediate position")
        
        # Step 2: Move into staging twist angle
        print("🔄 Step 2/7: Adjusting orientation for staging...")
        if 'twist' in stage_params:
            print(f"   📍 Executing staging twist for {stage}")
            twist_result = run_skill("moveJ_deg", *stage_params['twist'])
            
            if twist_result is False:
                print("[ERROR] Failed to execute staging twist")
                return False
            print("   ✅ Successfully adjusted orientation")
        else:
            print("   ⏭️ No twist movement defined for this stage")
        
        # Step 3: Move to target placement pose
        print("🎯 Step 3/7: Moving to placement position...")
        if 'pose' in stage_params:
            print(f"   📍 Moving to placement pose for {stage}")
            pose_result = run_skill("gotoJ_deg", *stage_params['pose'])
            
            if pose_result is False:
                print("[ERROR] Failed to move to placement pose")
                return False
            print("   ✅ Successfully positioned for placement")
        else:
            print("[ERROR] No placement pose defined for this stage")
            return False
        
        # Step 4: Open gripper to release paper cup
        print("🤏 Step 4/7: Releasing paper cup...")
        run_skill("sync")
        release_result = run_skill("set_gripper_position", 50, 0)
        
        if release_result is False:
            print("[ERROR] Failed to release paper cup")
            return False
        print("   ✅ Paper cup released successfully")
        
        # Allow settling time
        print("   ⏰ Allowing cup settling time...")
        time.sleep(1.0)
        
        # Step 5: Move up after placing paper cup
        print("⬆️ Step 5/7: Moving up after placement...")
        up_result = run_skill("moveEE", 0, 0, 150, 0, 0, 0)
        
        if up_result is False:
            print("[ERROR] Failed to move up after placement")
            return False
        print("   ✅ Successfully moved up after placement")
        
        # Step 6: Move to staging home position
        print("🏠 Step 6/7: Moving to staging home...")
        if 'stage_home' in stage_params:
            print(f"   📍 Moving to staging home for {stage}")
            stage_home_result = run_skill("gotoJ_deg", *stage_params['stage_home'])
            
            if stage_home_result is False:
                print("[ERROR] Failed to move to staging home")
                return False
            print("   ✅ Successfully moved to staging home")
        else:
            print("   ⏭️ No staging home defined for this stage")
        
        # Step 7: Untwist back towards machine
        print("🔄 Step 7/7: Untwisting back towards machine...")
        if 'twist_back' in stage_params:
            print(f"   📍 Executing untwist movement for {stage}")
            twist_back_result = run_skill("moveJ_deg", *stage_params['twist_back'])
            
            if twist_back_result is False:
                print("[ERROR] Failed to untwist back")
                return False
            print("   ✅ Successfully untwisted back towards machine")
        else:
            print("   ⏭️ No untwist movement defined for this stage")
        
        # Final success summary
        print("=" * 50)
        print(f"✅ PAPER CUP PLACEMENT COMPLETED SUCCESSFULLY FOR {stage.upper()}")
        print("   ✓ Paper cup placed at designated staging area")
        print("   ✓ Safe clearance achieved after placement")
        print("   ✓ Robot ready for next operation")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup placement: {e}")
        print("[INFO] Paper cup placement process terminated due to error")
        return False

def dispense_paper_cup(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.

    This function composes the full workflow by invoking `grab_paper_cup` followed by
    `place_paper_cup` using the provided parameters.

    Args:
        size (str): Paper cup size to grab ('7oz', '9oz', '12oz', etc.). Defaults to '7oz'.
        stage (str): Target stage for placement ('stage_1', 'stage_2', etc.). Defaults to 'stage_1'.

    Returns:
        bool: True if both grab and place steps succeed, False otherwise.
    """
    try:
        print("🥤🚚 Starting paper cup dispense sequence")
        print(f"[DEBUG dispense] Received params: {params}")
        print("=" * 50)

        # Pass full params to grab_paper_cup so it can extract cup size properly
        grab_ok = grab_paper_cup(**params)
        if grab_ok is False:
            print("[ERROR] Paper cup grab step failed; aborting dispense sequence")
            return False

        # Pass full params to place_paper_cup so it can extract stage properly
        place_ok = place_paper_cup(**params)
        if place_ok is False:
            print("[ERROR] Paper cup placement step failed; aborting dispense sequence")
            return False

        print("=" * 50)
        print(f"✅ PAPER CUP DISPENSED SUCCESSFULLY")
        print("=" * 50)
        return True

    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup dispensing: {e}")
        return False

def pick_paper_cup_station(**params) -> bool:
    """
    Pick up a paper cup from a specific stage.

    Args:
        stage (str|int|float): Target stage to pick cup from ('1', '2', '3', or '4', also accepts numeric 1.0, etc.)
        cup_size (str): One of 'H7', 'H9', 'H12' (also accepts '7oz','9oz','12oz')
    """
    try:
        # Extract and validate parameters
        raw_stage = params.get("stage", "1")  # Default to stage 1
        if raw_stage is None:
            raw_stage = "1"

        # Normalize stage to '1'..'4'
        stage = None
        if isinstance(raw_stage, str) and raw_stage.startswith("stage_"):
            try:
                stage = str(int(raw_stage.split("_", 1)[1]))
            except Exception:
                stage = raw_stage
        else:
            try:
                stage = str(int(float(raw_stage)))
            except Exception:
                stage = str(raw_stage)

        # Extract cup size from multiple possible formats
        cups_dict = None
        
        # First check if ingredients exists (nested format from scheduler)
        if 'ingredients' in params and isinstance(params['ingredients'], dict):
            cups_dict = params['ingredients'].get('cups')
            print(f"[DEBUG pick_station] Extracted cups from ingredients: {cups_dict}")
        
        # Otherwise check for direct cups parameter
        if not cups_dict:
            cups_dict = params.get("cups")
            # If cups is a list, try to extract from first item
            if isinstance(cups_dict, list) and len(cups_dict) > 0:
                first_cup = cups_dict[0]
                if isinstance(first_cup, dict) and 'ingredients' in first_cup:
                    cups_dict = first_cup['ingredients'].get('cups')
                    print(f"[DEBUG pick_station] Extracted cups from array ingredients: {cups_dict}")
        
        # Fallback to old 'cup_size' parameter
        if not cups_dict:
            cups_dict = params.get("cup_size")
        
        # Default to 12oz if no cup size provided
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}

        # Map H-codes to legacy sizes
        size_mapped = _normalize_paper_cup_size(cups_dict)  # H7/H9/H12 -> 7oz/9oz/12oz
        print(f"[DEBUG pick_station] Final normalized size: {size_mapped}")

        # Validate parameters
        valid_stages = ('1', '2', '3', '4')
        valid_sizes = ('7oz', '9oz', '12oz')

        if stage not in valid_stages:
            print(f"[ERROR] Invalid stage: {stage!r}")
            print(f"[INFO] Valid stages: {', '.join(valid_stages)}")
            return False

        if size_mapped not in valid_sizes:
            print(f"[ERROR] Invalid cup size for paper: {size_mapped!r}")
            print(f"[INFO] Valid sizes: H7/H9/H12 (or 7oz/9oz/12oz)")
            return False

        print(f"🥤 Starting paper cup pickup sequence - Stage: {stage}, Size: {size_mapped}")
        print("=" * 50)

        # Stage-specific positioning (replicated from paper station)
        stage_positions = {
            "1": (-88.246268,-41.336257,-138.612498,-7.761506,-92.800418,-1.075949),
            "2": (-112.183193,-43.094282,-124.253581,-21.289005,-116.494060,-4.560061),
            "3": (-128.808820,-49.036269,-103.446187,-38.102756,-132.891871,-7.919876),
            "4": (-139.460001,-59.117620,-76.669262,-57.246818,-143.330627,-11.182390)
        }

        # Paper cup gripper positions (align with 7/9/12oz used for paper)
        gripper_positions = {
            "7oz": 140,
            "9oz": 135,
            "12oz": 125,
        }

        # Step 1: Navigate to home positions
        print("🏠 Step 1/6: Navigating to home positions...")
        if not home(position="north_east"):
            print("[ERROR] Failed to move to north_east home")
            return False
        if not home(position="east"):
            print("[ERROR] Failed to move to east home")
            return False
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
        print(f"🤏 Step 4/6: Gripping {size_mapped} paper cup...")
        grip_result = run_skill("set_gripper_position", 255, gripper_positions[size_mapped])
        if not grip_result:
            print("[ERROR] Failed to grip cup")
            return False
        print("   ✅ Cup gripped successfully")

        run_skill("moveEE_movJ", 0, 0, 200, 0, 0, 0)
        
        # Step 5: Return to safe position
        print("🏠 Step 5/6: Returning to safe position...")
        if not home(position="east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully returned to safe position")

        print("🏠 Step 5/6: Returning to safe position...")
        if not home(position="north_east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully returned to safe position")

        # Final success summary
        print("=" * 50)
        print(f"✅ PAPER CUP PICKUP COMPLETED SUCCESSFULLY")
        print(f"   ✓ Stage {stage} cup ({size_mapped}) picked up")
        print("   ✓ Positioned for next operation")
        print("=" * 50)
        return True
    
    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup pickup: {e}")
        print("[INFO] Cup pickup process terminated due to error")
        return False

def place_paper_cup_station(**params) -> bool:
    """
    Place a paper cup at specified staging area.

    Args:
        stage (str|int|float): Target staging area ('1','2','3','4', also accepts numeric 1.0 etc.)
    """
    try:
        raw_stage = params.get("stage", "1")  # Default to stage 1
        if raw_stage is None:
            raw_stage = "1"

        # Normalize stage to '1'..'4'
        stage = None
        if isinstance(raw_stage, str) and raw_stage.startswith("stage_"):
            try:
                stage = str(int(raw_stage.split("_", 1)[1]))
            except Exception:
                stage = raw_stage
        else:
            try:
                stage = str(int(float(raw_stage)))
            except Exception:
                stage = str(raw_stage)

        valid_stages = ('1', '2', '3', '4')
        if stage not in valid_stages:
            print(f"[ERROR] Unknown stage: {stage!r}")
            print(f"[INFO] Valid stages: {', '.join(valid_stages)}")
            return False

        print(f"🥤 Starting paper cup placement sequence for stage {stage}")
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

        if stage in ("3", "4"):
            if not home(position="south_east"):
                print("[ERROR] Failed to move to south-east home")
                return False
        print("   ✅ Successfully moved to south-east home")

        # Step 3: Move to stage-specific position (re-using paper station positions)
        print(f"🎯 Step 3/5: Moving to stage {stage} position...")
        stage_positions = {
            "1": (-88.048157,-43.392315,-122.686608,-21.629447,-92.608930,-1.040377),
            "2": (-105.795646,-47.724084,-107.146702,-33.353721,-110.187785,-3.536224),
            "3": (-120.111216,-54.092813,-89.175854,-46.102535,-124.341571,-5.992437),
            "4": (-130.631169,-64.634756,-63.002881,-63.277542,-134.713357,-8.394235)
        }

        stage_result = run_skill("gotoJ_deg", *stage_positions[stage])
        if not stage_result:
            print(f"[ERROR] Failed to move to stage {stage} position")
            return False
        print(f"   ✅ Successfully positioned at stage {stage}")

        # Step 4: Release cup
        print("🤏 Step 4/5: Releasing paper cup...")
        release_result = run_skill("set_gripper_position", 50, 0)
        if not release_result:
            print("[ERROR] Failed to release paper cup")
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
        print(f"✅ PAPER CUP PLACEMENT COMPLETED FOR STAGE {stage}")
        print("   ✓ Cup positioned at designated staging area")
        print("   ✓ Safe release and clearance achieved")
        print("   ✓ Robot returned to home position")
        print("   🥤 Beverage station ready!")
        print("=" * 50)
        return True

    except Exception as e:
        print(f"[ERROR] Unexpected error during paper cup placement: {e}")
        print("[INFO] Cup placement process terminated due to error")
        return False

def place_paper_cup_sauces(**params) -> bool:
    """
    Place the paper cup at the sauces station.
    """
    try:
        if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
            return False
        if run_skill("gotoJ_deg", -38.389633,-75.079689,-66.372528,-35.134846,-127.236320,-0.949134) is False:
            return False
        if run_skill("gotoJ_deg", -38.295812,-75.434312,-68.188894,-32.969245,-127.142100,-0.958200) is False:
            return False
        if run_skill("set_gripper_position", 255, 0) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] place_paper_cup_sauces failed: {e}")
        return False

def pick_paper_cup_sauces(**params) -> bool:
    """
    Pick the paper cup from the sauces station.

    Args:
        cup_size (str): One of '7oz', '9oz', '12oz', '16oz' (required)
    """
    try:
        # Extract cup size from multiple possible formats
        cups_dict = None
        
        # First check if ingredients exists (nested format from scheduler)
        if 'ingredients' in params and isinstance(params['ingredients'], dict):
            cups_dict = params['ingredients'].get('cups')
            print(f"[DEBUG pick_sauces] Extracted cups from ingredients: {cups_dict}")
        
        # Otherwise check for direct cups parameter
        if not cups_dict:
            cups_dict = params.get("cups")
            # If cups is a list, try to extract from first item
            if isinstance(cups_dict, list) and len(cups_dict) > 0:
                first_cup = cups_dict[0]
                if isinstance(first_cup, dict) and 'ingredients' in first_cup:
                    cups_dict = first_cup['ingredients'].get('cups')
                    print(f"[DEBUG pick_sauces] Extracted cups from array ingredients: {cups_dict}")
        
        # Fallback to old 'cup_size' parameter
        if not cups_dict:
            cups_dict = params.get("cup_size")
        
        # Default to 12oz if no cup size provided
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}
        
        cup_size = _normalize_paper_cup_size(cups_dict)
        print(f"[DEBUG pick_sauces] Final normalized size: {cup_size}")
        valid_sizes = ("7oz", "9oz", "12oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        # Paper cups only support 7oz, 9oz, 12oz (no 16oz for paper)
        gripper_positions = {
            "7oz": 145,
            "9oz": 145,
            "12oz": 145,
        }

        if run_skill("set_gripper_position", 255, gripper_positions[cup_size]) is False:
            return False
        if run_skill("gotoJ_deg", -38.295812,-75.434312,-68.188894,-32.969245,-127.142100,-0.958200) is False:
            return False
        if run_skill("gotoJ_deg", -38.389633,-75.079689,-66.372528,-35.134846,-127.236320,-0.949134) is False:
            return False
        if run_skill("gotoJ_deg", -53.449154,-67.421219,-92.044746,-16.125631,-142.249084,0.477525) is False:
            return False
        return True
    except Exception as e:
        print(f"[ERROR] pick_paper_cup_sauces failed: {e}")
        return False

def place_paper_cup_milk(**params) -> bool:
    """
    Place the paper cup at the milk station.
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
        print(f"[ERROR] place_paper_cup_milk failed: {e}")
        return False

def pick_paper_cup_milk(**params) -> bool:
    """
    Pick the paper cup from the milk station.

    Args:
        cup_size (str): One of '7oz', '9oz', '12oz', '16oz' (required)
    """
    try:
        # Extract cup size from multiple possible formats
        cups_dict = None
        
        # First check if ingredients exists (nested format from scheduler)
        if 'ingredients' in params and isinstance(params['ingredients'], dict):
            cups_dict = params['ingredients'].get('cups')
            print(f"[DEBUG pick_milk] Extracted cups from ingredients: {cups_dict}")
        
        # Otherwise check for direct cups parameter
        if not cups_dict:
            cups_dict = params.get("cups")
            # If cups is a list, try to extract from first item
            if isinstance(cups_dict, list) and len(cups_dict) > 0:
                first_cup = cups_dict[0]
                if isinstance(first_cup, dict) and 'ingredients' in first_cup:
                    cups_dict = first_cup['ingredients'].get('cups')
                    print(f"[DEBUG pick_milk] Extracted cups from array ingredients: {cups_dict}")
        
        # Fallback to old 'cup_size' parameter
        if not cups_dict:
            cups_dict = params.get("cup_size")
        
        # Default to 12oz if no cup size provided
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}
        
        cup_size = _normalize_paper_cup_size(cups_dict)
        print(f"[DEBUG pick_milk] Final normalized size: {cup_size}")
        valid_sizes = ("7oz", "9oz", "12oz")
        if cup_size not in valid_sizes:
            print(f"[ERROR] Invalid cup size: {cup_size!r}")
            print(f"[INFO] Valid sizes: {', '.join(valid_sizes)}")
            return False

        # Paper cups only support 7oz, 9oz, 12oz (no 16oz for paper)
        gripper_positions = {
            "7oz": 145,
            "9oz": 145,
            "12oz": 145,
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
        print(f"[ERROR] pick_paper_cup_milk failed: {e}")
        return False


# Register functions for CLI discovery and external access
SEQUENCES = {
    'grab_paper_cup': grab_paper_cup,
    'place_paper_cup': place_paper_cup,
    'dispense_paper_cup': dispense_paper_cup,
    'pick_paper_cup_station': pick_paper_cup_station,
    'place_paper_cup_station': place_paper_cup_station,
    'place_paper_cup_sauces': place_paper_cup_sauces,
    'pick_paper_cup_sauces': pick_paper_cup_sauces,
    'place_paper_cup_milk': place_paper_cup_milk,
    'pick_paper_cup_milk': pick_paper_cup_milk,
}
