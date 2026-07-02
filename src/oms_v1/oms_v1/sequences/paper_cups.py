"""
paper_cups.py

Defines the paper cup handling sequences for coffee service automation.
This module provides comprehensive functions for grabbing, placing, and serving
paper cups in the BARNS coffee automation system, including size-based handling
and staging area management.
"""
# NOTE: Gripper commands can opt into strict register verification via run_skill(..., verify_position=True).


import time
import sys
import inspect
from typing import Dict, Any, Optional
from oms_v1.params import (
    GRAB_PAPER_CUP_PARAMS, PLACE_PAPER_CUP_PARAMS,
    PAPER_CUPS_NAVIGATION_PARAMS, PAPER_CUP_ARM1_NAVIGATION_POSES, PAPER_CUP_ARM1_DISPENSER_PARAMS, PAPER_CUPS_STATION_PARAMS, PAPER_CUP_ARM2_DISPENSER_PARAMS,
    PAPER_CUP_GRIPPER_POSITIONS, PAPER_CUP_MOVEMENT_OFFSETS,
    ESPRESSO_MOVEMENT_OFFSETS,
    ESPRESSO_HOME, GRIPPER_OPEN, GRIPPER_RELEASE, GRIPPER_FULL,
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size
)
from oms_v1.manipulate_node import run_skill as _raw_run_skill
from oms_v1.sequences.home import home as _raw_home, return_back_to_home as _raw_return_back_to_home
from oms_v1.sequences.computer_vision import detect_cup_gripper as _raw_detect_cup_gripper


# Runtime tracing helpers.
# Set PAPER_CUPS_TRACE_DEBUG = False to disable these step prints without changing behavior.
PAPER_CUPS_TRACE_DEBUG = True

def _trace_format_value(value, max_len: int = 140) -> str:
    """Return a compact printable representation for trace logs."""
    try:
        text = repr(value)
    except Exception:
        text = f"<{type(value).__name__}>"
    if len(text) > max_len:
        text = text[:max_len - 3] + "..."
    return text

def _trace_step(scope: str, message: str) -> None:
    """Print a visible, low-overhead runtime trace message."""
    if globals().get("PAPER_CUPS_TRACE_DEBUG", True):
        print(f"[{scope}] {message}", flush=True)

def run_skill(*args, **kwargs):
    """Trace wrapper around manipulate_node.run_skill."""
    skill_name = args[0] if args else "<missing>"
    skill_args = args[1:] if len(args) > 1 else ()
    _trace_step("PAPER-CUPS", f"run_skill {skill_name} START args={_trace_format_value(skill_args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_run_skill(*args, **kwargs)
    _trace_step("PAPER-CUPS", f"run_skill {skill_name} DONE result={_trace_format_value(result)}")
    return result


def _fail(reason: str = "") -> bool:
    """Log caller + nearby source line, then return False.

    This keeps the original bool contract while making silent guard failures
    visible in stdout. Use as: return _fail() or return _fail("reason").
    """
    if not globals().get("PAPER_CUPS_TRACE_DEBUG", True):
        return False

    try:
        frame = sys._getframe(1)
        scope = frame.f_code.co_name
        line_no = frame.f_lineno
        cause = ""

        try:
            src_lines, start = inspect.getsourcelines(frame)
            rel = line_no - start

            for back in (1, 2):
                idx = rel - back
                if 0 <= idx < len(src_lines):
                    candidate = src_lines[idx].strip()
                    if candidate and not candidate.startswith("return"):
                        cause = candidate
                        break
        except Exception:
            pass

        msg = f"[{scope}] FAIL line={line_no}"
        if cause:
            msg += f" cause=`{cause}`"
        if reason:
            msg += f" reason={reason}"

        _trace_step("PAPER-CUPS", msg)

    except Exception:
        pass

    return False

def home(*args, **kwargs):
    """Trace wrapper around home(...)."""
    _trace_step("PAPER-CUPS", f"home START args={_trace_format_value(args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_home(*args, **kwargs)
    _trace_step("PAPER-CUPS", f"home DONE result={_trace_format_value(result)}")
    return result

def return_back_to_home(*args, **kwargs):
    """Trace wrapper around return_back_to_home(...)."""
    _trace_step("PAPER-CUPS", f"return_back_to_home START args={_trace_format_value(args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_return_back_to_home(*args, **kwargs)
    _trace_step("PAPER-CUPS", f"return_back_to_home DONE result={_trace_format_value(result)}")
    return result

def detect_cup_gripper(*args, **kwargs):
    """Trace wrapper around detect_cup_gripper(...)."""
    _trace_step("PAPER-CUPS", f"detect_cup_gripper START args={_trace_format_value(args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_detect_cup_gripper(*args, **kwargs)
    _trace_step("PAPER-CUPS", f"detect_cup_gripper DONE result={_trace_format_value(result)}")
    return result

def ok(r):
    _trace_step("PAPER-CUPS", "ok START")
    return r not in (False, None)

def _normalize_paper_cup_size(cups_dict: Any) -> str:
    """
    Universal cup size normalizer for paper cup operations.
    Accepts BOTH H-codes AND C-codes regardless of prefix.
    Extracts the numeric size and returns standardized format.
    
    Args:
        cups_dict: Dictionary containing cup information, or a simple string/value
        
    Returns:
        str: Normalized cup size (e.g., '7oz', '9oz', '12oz')
        
    Examples:
        cup_H9 → '9oz'
        cup_C9 → '9oz'
        cup_h12 → '12oz'
        cup_c7 → '7oz'
    """
    _trace_step("PAPER-CUPS", "_normalize_paper_cup_size START")
    if not cups_dict:
        from oms_v1.params import DEFAULT_PAPER_CUP_SIZE
        return DEFAULT_PAPER_CUP_SIZE
    
    # Extract the cup code (case-insensitive)
    if isinstance(cups_dict, dict):
        cup_key = next(iter(cups_dict.keys()), None)
        if cup_key:
            # Convert to uppercase for parsing
            cup_key_str = str(cup_key).upper()
            if 'CUP_' in cup_key_str:
                cup_code = cup_key_str.split('CUP_', 1)[1]
            else:
                cup_code = cup_key_str
            
            # Extract numeric size from code (works with both H and C prefixes)
            # H7, H9, H12, C7, C9, C12, C16 → extract the number
            if cup_code and len(cup_code) >= 2:
                # Remove H or C prefix if present
                if cup_code[0] in ('H', 'C'):
                    size_num = cup_code[1:]
                else:
                    size_num = cup_code
                
                # Validate and return standardized size (paper cups: 7, 9, 12)
                if size_num in ('7', '9', '12'):
                    return f"{size_num}oz"
    
    # If parsing failed, try the standard normalizers
    # Try paper first (since this is paper cup function)
    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result
    
    # Try plastic as fallback
    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '' and result in ('7oz', '9oz', '12oz'):
        return result
    
    # Final fallback
    from oms_v1.params import DEFAULT_PAPER_CUP_SIZE
    return DEFAULT_PAPER_CUP_SIZE

def grab_paper_cup(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.
    """
    _trace_step("PAPER-CUPS", "grab_paper_cup START")
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
    if not size:
        return _fail()
    
    cup_params = GRAB_PAPER_CUP_PARAMS.get(str(size))
    if not cup_params:
        cup_params = GRAB_PAPER_CUP_PARAMS.get("7oz")
        if not cup_params:
            return _fail()
    
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_HOME)):
        return _fail()
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['espresso_avoid'])):
        return _fail()
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['dispenser_area'])):
        return _fail()
    
    attempt_count = 0
    while attempt_count < 15:
        if size == "7oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_7oz'])):
                return _fail()
        elif size == "9oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_9oz'])):
                return _fail()
        elif size == "12oz":
            if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_12oz'])):
                return _fail()
        else:
            return _fail()
        
        if 'approach' in cup_params:
            if not ok(run_skill("moveEE", *cup_params['approach'])):
                return _fail()
        
        if 'grip_width' not in cup_params:
            return _fail()
        
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, cup_params['grip_width'], verify_position=True)):
            return _fail()
        
        if 'retreat' in cup_params:
            if not ok(run_skill("moveEE", *cup_params['retreat'])):
                return _fail()
        
        cup_detected = detect_cup_gripper()
        _trace_step("PAPER-CUPS", f"grab_paper_cup_arm1 detection result={cup_detected}")
        if cup_detected:
            break
        
        attempt_count += 1
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("set_gripper_position", GRIPPER_FULL, GRIPPER_OPEN, verify_position=True)):
            return _fail()
        if attempt_count == 15:
            return _fail()
    
    return True

def place_paper_cup(**params) -> bool:
    """
    Place a paper cup at the specified staging area.
    """
    _trace_step("PAPER-CUPS", "place_paper_cup START")
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = f"stage_{cup_position}"
    
    stage_params = PLACE_PAPER_CUP_PARAMS.get(str(stage))
    if not stage_params:
        return _fail()
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['intermediate'])):
        return _fail()
    
    if 'twist' in stage_params:
        if not ok(run_skill("moveJ_deg", *stage_params['twist'])):
            return _fail()
    
    if 'pose' not in stage_params:
        return _fail()
    
    if not ok(run_skill("gotoJ_deg", *stage_params['pose'])):
        return _fail()
    
    if not ok(run_skill("sync")):
        return _fail()
    if not ok(run_skill("set_gripper_position", 25, 100, 255, verify_position=True)):
        return _fail()
    if not ok(run_skill("set_gripper_position", 255, 0, 255, verify_position=True)):
        return _fail()
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])):
        return _fail()
    
    if 'stage_home' in stage_params:
        if not ok(run_skill("gotoJ_deg", *stage_params['stage_home'])):
            return _fail()
    
    if 'twist_back' in stage_params:
        if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_back_machine'])):
            return _fail()
    
    return True

def grab_paper_cup_arm1(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.

    First attempt does the full size-specific approach.
    If detection fails, retries only:
      1) open gripper
      2) move back up
      3) close gripper with size-specific width
      4) move back down
    """
    _trace_step("PAPER-CUPS", "grab_paper_cup_arm1 START")
    def ok(r):
        return r not in (False, None)

    cups_dict = _extract_cups_dict(params)
    size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
    if not size:
        return _fail()

    # Size-specific arm-1 dispenser poses live in params.py so deployment
    # calibration does not require editing sequence logic.
    cfg = PAPER_CUP_ARM1_DISPENSER_PARAMS.get(size)
    if not cfg:
        return _fail()

    if not ok(home(position=cfg["home"])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *cfg["pose1"])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *cfg["pose2"])):
        return _fail()
    if not ok(run_skill("sync")):
        return _fail()

    attempt_count = 0
    while attempt_count < 15:
        _trace_step("PAPER-CUPS", f"grab_paper_cup_arm1 attempt={attempt_count + 1}/15 size={size}")
        if attempt_count > 0:
            if not ok(run_skill("set_gripper_position", 255, 0, 255, verify_position=True)):
                return _fail()
            if not ok(run_skill("moveEE_movJ", 0, 0, cfg["up_down_z"], 0, 0, 0)):
                return _fail()

        if not ok(run_skill("set_gripper_position", 255, cfg["grip"], 255, verify_position=True)):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0, 0, -cfg["up_down_z"], 0, 0, 0)):
            return _fail()

        cup_detected = detect_cup_gripper()
        _trace_step("PAPER-CUPS", f"grab_paper_cup_arm1 detection result={cup_detected}")
        if cup_detected:
            break

        attempt_count += 1
        if not ok(run_skill("sync")):
            return _fail()

        if attempt_count == 15:
            return _fail()

    if not ok(home(position=cfg["home"])):
        return _fail()
    return True

def place_paper_cup_arm1(**params) -> bool:
    """
    Place a paper cup at the specified staging area.
    """
    _trace_step("PAPER-CUPS", "place_paper_cup_arm1 START")
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = str(cup_position)

    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}

    size_mapped = _normalize_paper_cup_size(cups_dict)

    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')

    if stage not in valid_stages or size_mapped not in valid_sizes:
        return _fail()

    stage_params_map = {
        "1": PLACE_PAPER_CUP_PARAMS['stage_1'],
        "2": PLACE_PAPER_CUP_PARAMS['stage_2'],
        "3": PLACE_PAPER_CUP_PARAMS['stage_3'],
        "4": PLACE_PAPER_CUP_PARAMS['stage_4'],
    }

    stage_params = stage_params_map[stage]

    if stage == "1":
        if not ok(run_skill("gotoJ_deg", *PAPER_CUP_ARM1_NAVIGATION_POSES['stage_1_entry'])):
            return _fail()
    elif stage == "2":
        if not ok(home(position="south_west")):
            return _fail()
    elif stage == "4":
        if not ok(home(position="south")):
            return _fail()
    elif stage == "3":
        if not ok(home(position="south_west")):
            return _fail()

    if 'pose' not in stage_params:
        return _fail()

    if not ok(run_skill("gotoJ_deg", *stage_params['pose'])):
        return _fail()

    if not ok(run_skill("sync")):
        return _fail()

    if not ok(run_skill("set_gripper_position", 25, 100, 255, verify_position=True)):
        return _fail()

    if not ok(run_skill("set_gripper_position", 255, 0, 255, verify_position=True)):
        return _fail()

    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])):
        return _fail()

    if not ok(run_skill("set_speed_factor", 100)):
        return _fail()

    if 'stage_home' in stage_params:
        if not ok(run_skill("gotoJ_deg", *stage_params['stage_home'])):
            return _fail()

    return True

def dispense_paper_arm1_cup_station(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
    """
    _trace_step("PAPER-CUPS", "dispense_paper_arm1_cup_station START")
    if not grab_paper_cup_arm1(**params):
        return _fail()
    if not place_paper_cup_arm1(**params):
        return _fail()
    return True

def grab_paper_arm2_cup_station(**params) -> bool:
    """
    Grab a paper cup of specified size from the paper cup dispenser.

    First attempt does the full size-specific approach.
    If detection fails, retries only:
      1) open gripper
      2) move back up
      3) close gripper with size-specific width
      4) move back down
    """
    _trace_step("PAPER-CUPS", "grab_paper_arm2_cup_station START")
    def ok(r):
        return r not in (False, None)

    cups_dict = _extract_cups_dict(params)
    size = _normalize_paper_cup_size(cups_dict if cups_dict else "7oz")
    if not size:
        return _fail()

    # Size-specific arm-2 dispenser poses live in params.py so deployment
    # calibration does not require editing sequence logic.
    cfg = PAPER_CUP_ARM2_DISPENSER_PARAMS.get(size)
    if not cfg:
        return _fail()

    if not ok(home(position=cfg["home"])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *cfg["pose1"])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *cfg["pose2"])):
        return _fail()
    # if not ok(run_skill("moveEE_movJ", *cfg["visitfix"])):
    #     return False
    if not ok(run_skill("sync")):
        return _fail()

    attempt_count = 0
    while attempt_count < 15:
        _trace_step("PAPER-CUPS", f"grab_paper_cup_arm2 attempt={attempt_count + 1}/15 size={size}")
        if attempt_count > 0:
            if not ok(run_skill("set_gripper_position", 255, 0, 255, verify_position=True)):
                return _fail()
            if not ok(run_skill("moveEE_movJ", 0, 0, cfg["up_down_z"], 0, 0, 0)):
                return _fail()

        if not ok(run_skill("set_gripper_position", 255, cfg["grip"], 255, verify_position=True)):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0, 0, -cfg["up_down_z"], 0, 0, 0)):
            return _fail()

        cup_detected = detect_cup_gripper()
        _trace_step("PAPER-CUPS", f"grab_paper_cup_arm2 detection result={cup_detected}")
        if cup_detected:
            break

        attempt_count += 1
        if not ok(run_skill("sync")):
            return _fail()

        if attempt_count == 15:
            return _fail()

    if not ok(home(position=cfg["home"])):
        return _fail()
    if not ok(home(position="north_east")):
        return _fail()
    return True

def place_paper_arm2_cup_station(**params) -> bool:
    """
    Place a paper cup at specified staging area.
    """
    _trace_step("PAPER-CUPS", "place_paper_arm2_cup_station START")
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = str(cup_position)

    valid_stages = ("1", "2", "3", "4")
    if stage not in valid_stages:
        return _fail()

    if stage in ("1", "2"):
        if not ok(home(position="east")):
            return _fail()
    if stage in ("3", "4"):
        if not ok(home(position="south_east")):
            return _fail()

    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS["staging"]["place_1"],
        "2": PAPER_CUPS_STATION_PARAMS["staging"]["place_2"],
        "3": PAPER_CUPS_STATION_PARAMS["staging"]["place_3"],
        "4": PAPER_CUPS_STATION_PARAMS["staging"]["place_4"],
    }

    stage_home_map = {
        "1": "east",
        "2": "east",
        "3": "south_east",
        "4": "south_east",
    }

    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return _fail()
    if not ok(run_skill("sync")):
        return _fail()

    if not ok(run_skill("set_gripper_position", 25, 100, 255, verify_position=True)):
        return _fail()

    if not ok(run_skill("set_gripper_position", 255, 0, 255, verify_position=True)):
        return _fail()

    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS["place_up"])):
        return _fail()

    if not ok(run_skill("set_speed_factor", 100)):
        return _fail()

    if not ok(home(position=stage_home_map[stage])):
        return _fail()

    return True

def dispense_paper_arm2_cup_station(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
    """
    _trace_step("PAPER-CUPS", "dispense_paper_arm2_cup_station START")
    if not grab_paper_arm2_cup_station(**params):
        return _fail()
    if not place_paper_arm2_cup_station(**params):
        return _fail()
    return True

def dispense_paper_cup_station(**params) -> bool:
    """
    Dispense a paper cup by grabbing it from the dispenser and placing it at the requested stage.
    """
    _trace_step("PAPER-CUPS", "dispense_paper_cup_station START")
    if not grab_paper_cup(**params):
        return _fail()
    if not place_paper_cup(**params):
        return _fail()
    return True

def pick_paper_cup_station(**params) -> bool:
    """
    Pick up a paper cup from a specific stage.
    """
    _trace_step("PAPER-CUPS", "pick_paper_cup_station START")
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return _fail()
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['pickup_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['pickup_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['pickup_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['pickup_4']
    }
    
    gripper_positions = PAPER_CUP_GRIPPER_POSITIONS
    
    if not home(position="east"):
        return _fail()
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return _fail()
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return _fail()
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_down'])):
        return _fail()
    
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[size_mapped])):
        return _fail()
    
    if not ok(run_skill("moveEE_movJ", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_up'])):
        return _fail()
    
    if not home(position="east"):
        return _fail()
    
    if not home(position="north_east"):
        return _fail()
    
    return True

def place_paper_cup_station(**params) -> bool:
    """
    Place a paper cup at specified staging area.

    """
    _trace_step("PAPER-CUPS", "place_paper_cup_station START")
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    if not home(position="north_east"):
        return _fail()
    if not home(position="east"):
        return _fail()
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return _fail()
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['place_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['place_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['place_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['place_4']
    }
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return _fail()
    if not ok(run_skill("sync")):
        return _fail()
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN)):
        return _fail()
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_return_up'])):
        return _fail()
    if not home(position="east"):
        return _fail()
    
    return True

def place_paper_cup_sauces(**params) -> bool:
    """
    Place the paper cup at the sauces station.
    """
    _trace_step("PAPER-CUPS", "place_paper_cup_sauces START")
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position1'])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position2'])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position3'])):
        return _fail()
    if not ok(run_skill("sync")):
        return _fail()
    if not ok(run_skill("set_gripper_position", 255, 75, 255, verify_position=True)):
        return _fail()
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return _fail()
    return True

def pick_paper_cup_sauces(**params) -> bool:
    """
    Pick the paper cup from the sauces station.
    """
    _trace_step("PAPER-CUPS", "pick_paper_cup_sauces START")
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    cup_size = _normalize_paper_cup_size(cups_dict)
    valid_sizes = ("7oz", "9oz", "12oz")
    if cup_size not in valid_sizes:
        return _fail()
    
    gripper_positions = {"7oz": 145, "9oz": 125, "12oz": 135, "16oz": 118}
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return _fail()
    if not ok(run_skill("moveEE", -1,0,0,0,0,0)):
        return _fail()
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position2'])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['sauces_station']['position1'])):
        return _fail()
    return True

def place_paper_cup_milk(**params) -> bool:
    """
    Place the paper cup at the milk station.
    """
    _trace_step("PAPER-CUPS", "place_paper_cup_milk START")
    def ok(r):
        return r not in (False, None)
    
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position1'])):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position3'])):
        return _fail()
    if not ok(run_skill("sync")):
        return _fail()
    if not ok(run_skill("set_gripper_position", 255, 75, 255, verify_position=True)):
        return _fail()
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return _fail()
    return True

def pick_paper_cup_milk(**params) -> bool:
    """
    Pick the paper cup from the milk station.
    """
    _trace_step("PAPER-CUPS", "pick_paper_cup_milk START")
    def ok(r):
        return r not in (False, None)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    cup_size = _normalize_paper_cup_size(cups_dict)
    valid_sizes = ("7oz", "9oz", "12oz")
    if cup_size not in valid_sizes:
        return _fail()
    
    gripper_positions = {"7oz": 145, "9oz": 125, "12oz": 135, "16oz": 118}
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return _fail()
    if not ok(run_skill("moveEE", -1,0,-5,0,0,0)):
        return _fail()
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size], verify_position=True)):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_STATION_PARAMS['milk_station']['position1'])):
        return _fail()
    return True

def pick_cup_for_hot_water(**params) -> bool:
    """
    Pick up a paper cup from a specific stage for hot water.
    """
    _trace_step("PAPER-CUPS", "pick_cup_for_hot_water START")
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return _fail()
    
    stage_positions = {
        "1": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_1'],
        "2": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_2'],
        "3": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_3'],
        "4": PAPER_CUPS_STATION_PARAMS['staging']['pickup_hot_water_4']
    }
    
    gripper_positions = PAPER_CUP_GRIPPER_POSITIONS
    
    if not home(position="south_west"):
        return _fail()
    if stage in ("3", "4"):
        if not home(position="south"):
            return _fail()
    
    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return _fail()
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_hot_water_down'])):
        return _fail()
    if size_mapped == '12oz':
        if not ok(run_skill("set_gripper_position", 255,120,255, verify_position=True)):
            return _fail()
    else:
        if not ok(run_skill("set_gripper_position", 255,130,255, verify_position=True)):
            return _fail()
    
    if not ok(run_skill("sync")):
        return _fail()
    
    if not ok(run_skill("set_speed_factor", 75)):
        return _fail()
    
    if not ok(run_skill("moveEE_movJ", *PAPER_CUP_MOVEMENT_OFFSETS['pickup_up'])):
        return _fail()
    
    if not home(position="west"):
        return _fail()

    if not ok(run_skill("approach_machine", "three_group_espresso", "hot_water")):
        return _fail()
    
    if not ok(run_skill("mount_machine", "three_group_espresso", "hot_water")):
        return _fail()

    if not ok(run_skill("sync")):
        return _fail()
    
    return True

def return_cup_with_hot_water(**params) -> bool:
    """
    Complete hot water dispensing sequence and return to holding position.
    """
    _trace_step("PAPER-CUPS", "return_cup_with_hot_water START")
    def ok(r):
        return r not in (False, None)
    
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    
    cups_dict = _extract_cups_dict(params)
    if not cups_dict:
        cups_dict = {"cup_H12": 1.0}
    
    size_mapped = _normalize_paper_cup_size(cups_dict)
    
    valid_stages = ('1', '2', '3', '4')
    valid_sizes = ('7oz', '9oz', '12oz')
    
    if stage not in valid_stages or size_mapped not in valid_sizes:
        return _fail()
    
    stage_params_map = {
        "1": PLACE_PAPER_CUP_PARAMS['stage_1'],
        "2": PLACE_PAPER_CUP_PARAMS['stage_2'],
        "3": PLACE_PAPER_CUP_PARAMS['stage_3'],
        "4": PLACE_PAPER_CUP_PARAMS['stage_4'],
    }
    
    stage_params = stage_params_map.get(stage, {})

    if not ok(run_skill("set_speed_factor",25)):
        return _fail()

    if not ok(run_skill("moveEE", *ESPRESSO_MOVEMENT_OFFSETS['hot_water_retreat'])):
        return _fail()

    if stage in ("1"):
        if not run_skill("gotoJ_deg", *PAPER_CUP_ARM1_NAVIGATION_POSES['stage_1_entry']):
            return _fail()
    if stage in ("2","3", "4"):
        if not home(position="south_west"):
            return _fail()
    if stage in ("3", "4"):
        if not home(position="south"):
            return _fail()
    
    if 'pose' not in stage_params:
        return _fail()
    
    if not ok(run_skill("gotoJ_deg", *stage_params['pose'])):
        return _fail()
    
    if not ok(run_skill("sync")):
        return _fail()
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE, GRIPPER_OPEN, verify_position=True)):
        return _fail()
    
    if not ok(run_skill("moveEE", *PAPER_CUP_MOVEMENT_OFFSETS['place_up'])):
        return _fail()

    if not ok(run_skill("set_speed_factor", 100)):
        return _fail()
    
    if 'stage_home' in stage_params:
        if not ok(run_skill("gotoJ_deg", *stage_params['stage_home'])):
            return _fail()
    
    if 'twist_back' in stage_params:
        if not ok(run_skill("gotoJ_deg", *PAPER_CUPS_NAVIGATION_PARAMS['twist_back_machine'])):
            return _fail()
    
    return True

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
    'pick_cup_for_hot_water': pick_cup_for_hot_water,
    'return_cup_with_hot_water': return_cup_with_hot_water,
    'dispense_paper_arm1_cup_station': dispense_paper_arm1_cup_station,
    'dispense_paper_arm2_cup_station': dispense_paper_arm2_cup_station,
    'grab_paper_cup_arm1': grab_paper_cup_arm1,
    'place_paper_cup_arm1': place_paper_cup_arm1,
    'grab_paper_arm2_cup_station': grab_paper_arm2_cup_station,
    'place_paper_arm2_cup_station': place_paper_arm2_cup_station,
}