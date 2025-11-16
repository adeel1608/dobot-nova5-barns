"""
paper_cups.py

Defines the paper cup handling sequences for coffee service automation.
This module provides comprehensive functions for grabbing, placing, and serving
paper cups in the BARNS coffee automation system, including size-based handling
and staging area management.
"""

import time
from typing import Dict, Any, Optional
from oms_v1.params import (
    GRAB_PAPER_CUP_PARAMS, PLACE_PAPER_CUP_PARAMS,
    ESPRESSO_HOME,
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size
)
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.home import home
from oms_v1.sequences.computer_vision import detect_cup_gripper

def _normalize_paper_cup_size(cups_dict: Any) -> str:
    """
    Parse paper cup size from new JSON format.
    
    This is a wrapper around the unified _normalize_cup_size function.
    Use this for backward compatibility in paper cup operations.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz')
    """
    return _normalize_cup_size(cups_dict, cup_type='paper')

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
        # Extract and validate size parameter using unified helper
        cups_dict = _extract_cups_dict(params)
        size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
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
        home_result = run_skill("gotoJ_deg", *ESPRESSO_HOME)
        if home_result is False:
            print("[ERROR] Failed to move to espresso home")
            return False
        print("   ✅ Successfully moved to espresso home")
        
        # Step 2: Twist to avoid hitting the espresso machine during navigation
        print("🔄 Step 2/8: Navigating around espresso machine...")
        twist_result = run_skill("gotoJ_deg", 106.17209, 16.269149, -135.156441, -81.822150, -49.784457, 13.771214)
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

        # Check if a cup is in the gripper
        attempt_count = 0
        while attempt_count < 3:        
            if size == "7oz":
                twist_back_result = run_skill("gotoJ_deg", 54.948658, 12.208040, -69.338005, -32.943398, 90.239655, -124.960251)
                if twist_back_result is False:
                    print("[ERROR] Failed to execute twist back movement")
                    return False
                print("   ✅ Successfully executed twist back movement")
            elif size == "9oz":
                twist_back_result = run_skill("gotoJ_deg", 27.502762, 30.098457, -80.283768, -36.358551, 88.971786, -154.398346)
                if twist_back_result is False:
                    print("[ERROR] Failed to execute twist back movement")
                    return False
                print("   ✅ Successfully executed twist back movement")
            elif size == "12oz":
                twist_back_result = run_skill("gotoJ_deg", -21.871843, 4.984756, -63.493607, -31.584101, 90.055153, -201.714615)
                if twist_back_result is False:
                    print("[ERROR] Failed to execute twist back movement")
                    return False
                print("   ✅ Successfully executed twist back movement")
            else:
                print("   ⏭️ No twist back movement defined for this size")
                return False
        
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
            
            # Check if a cup is in the gripper
            cup_detected = detect_cup_gripper()
            if cup_detected:
                print("✅ Cup detected in gripper")
                break
            else:
                print("❌ No cup detected in gripper")
                attempt_count += 1

                # Step 8: Open gripper to release paper cup
                print("🤏 Step 8/8: Releasing paper cup...")
                release_result = run_skill("set_gripper_position", 255, 0)
                
                if release_result is False:
                    print("[ERROR] Failed to release paper cup")
                    return False
                print("   ✅ Paper cup released successfully")
                if attempt_count == 3:
                    print("[ERROR] Failed to grab paper cup after 3 attempts")
                    return False

                
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
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        
    Returns:
        bool: True if paper cup placed successfully, False otherwise
        
    Raises:
        Exception: If unexpected error occurs during placement process
        
    Example:
        success = place_paper_cup(position={'cup_position': 1.0})
        if success:
            print("Paper cup placed successfully")
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = f"stage_{cup_position}"  # Convert to internal stage format
            
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
        release_result = run_skill("set_gripper_position", 50, 0)
        
        if release_result is False:
            print("[ERROR] Failed to release paper cup")
            return False
        print("   ✅ Paper cup released successfully")
        
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
            twist_back_result = run_skill("gotoJ_deg", 42.427441,  13.883821, -133.648376, -81.024788,  -49.533218,  13.894379)
            
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

def dispense_paper_cup_station(**params) -> bool:
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
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
        cups (dict): Cup size dictionary, e.g., {'cup_H12': 1.0}
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use

        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}
        
        size_mapped = _normalize_paper_cup_size(cups_dict)

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
        print("🏠 Step 5/6: Returning to east home...")
        if not home(position="east"):
            print("[ERROR] Failed to return to east home")
            return False
        print("   ✅ Successfully returned to east home")

        # Step 6: Return to north-east home
        print("🏠 Step 6/6: Returning to north-east home...")
        if not home(position="north_east"):
            print("[ERROR] Failed to return to north-east home")
            return False
        print("   ✅ Successfully returned to north-east home")

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
        position (dict): Position dictionary with 'cup_position' key (1-4), e.g., {'cup_position': 1.0}
    """
    try:
        # Extract cup position from new format: {'position': {'cup_position': 1.0}}
        cup_position = _extract_cup_position(params)
        stage = str(cup_position)  # Convert to string for internal use

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
            print("🏠 Step 2.5/5: Moving to south-east home...")
            if not home(position="south_east"):
                print("[ERROR] Failed to move to south-east home")
                return False
            print("   ✅ Successfully moved to south-east home")

        # Step 3: Move to stage-specific position (re-using paper station positions)
        print(f"🎯 Step 3/5: Moving to stage {stage} position...")
        stage_positions = {
            "1": (-82.350214,-52.505954,-126.196674,-1.090475,-82.250159,-0.103364),
            "2": (-102.385087,-53.365893,-116.767921,-9.682041,-102.286850,-0.028823),
            "3": (-118.186155,-56.698592,-101.309618,-21.804371,-118.094850,0.033187),
            "4": (-129.712707,-63.412577,-80.074947,-36.306427,-129.636056,0.087909)
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
        cups (dict): Cup size dictionary, e.g., {'cup_H12': 1.0} - supports H7, H9, H12 (7oz, 9oz, 12oz)
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}
        
        cup_size = _normalize_paper_cup_size(cups_dict)
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
        cups (dict): Cup size dictionary, e.g., {'cup_H12': 1.0} - supports H7, H9, H12 (7oz, 9oz, 12oz)
    """
    try:
        # Extract cup size using unified helper
        cups_dict = _extract_cups_dict(params)
        if not cups_dict:
            print("[INFO] No cup_size parameter provided, defaulting to 12oz")
            cups_dict = {"cup_H12": 1.0}
        
        cup_size = _normalize_paper_cup_size(cups_dict)
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
    'dispense_paper_cup_station': dispense_paper_cup_station,
    'pick_paper_cup_station': pick_paper_cup_station,
    'place_paper_cup_station': place_paper_cup_station,
    'place_paper_cup_sauces': place_paper_cup_sauces,
    'pick_paper_cup_sauces': pick_paper_cup_sauces,
    'place_paper_cup_milk': place_paper_cup_milk,
    'pick_paper_cup_milk': pick_paper_cup_milk,
}
