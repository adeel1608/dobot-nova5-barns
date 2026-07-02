"""
plastic_cups.py

Defines the plastic cup handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for grabbing and placing plastic cups
in the BARNS coffee automation system, supporting multiple cup sizes for cold beverages
like slushes, iced drinks, and cold brews.
"""
# NOTE: Gripper commands can opt into strict register verification via run_skill(..., verify_position=True).


import time
import sys
import inspect
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill as _raw_run_skill
from oms_v1.sequences.home import home as _raw_home, return_back_to_home as _raw_return_back_to_home
from oms_v1.params import (
    DEFAULT_PLASTIC_CUP_SIZE, validate_cup_size,
    PLASTIC_CUPS_PARAMS, PLASTIC_CUP_GRIPPER_POSITIONS,
    PLASTIC_CUP_DISPENSE_GRIPPER, PLASTIC_CUP_DISPENSE_SPEEDS,
    PLASTIC_CUP_EXTRACT_OFFSETS, PLASTIC_CUP_MOVEMENT_OFFSETS,
    GRIPPER_FULL, GRIPPER_OPEN, GRIPPER_RELEASE_GENTLE,
    GRIPPER_RELEASE, GRIPPER_HOLD_LOOSE, SPEED_NORMAL, SPEED_FAST,
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size,
    _set_cup_dispensed, _check_and_clear_cup_dispensed
)
from oms_v1.sequences.computer_vision import detect_cup_gripper as _raw_detect_cup_gripper

# Runtime trace helpers. Keep movement ordering unchanged while making cup flow
# visible in ROS/stdout logs.
PLASTIC_CUPS_TRACE_DEBUG = True

def _trace_format_value(value, max_len: int = 140) -> str:
    try:
        text = repr(value)
    except Exception:
        text = f"<{type(value).__name__}>"
    return text if len(text) <= max_len else text[:max_len - 3] + "..."

def _trace_step(scope: str, message: str) -> None:
    if globals().get("PLASTIC_CUPS_TRACE_DEBUG", True):
        print(f"[PLASTIC-CUPS:{scope}] {message}", flush=True)

def run_skill(*args, **kwargs):
    skill_name = args[0] if args else "<missing>"
    skill_args = args[1:] if len(args) > 1 else ()
    _trace_step("run_skill", f"{skill_name} START args={_trace_format_value(skill_args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_run_skill(*args, **kwargs)
    _trace_step("run_skill", f"{skill_name} DONE result={_trace_format_value(result)}")
    return result

def home(*args, **kwargs):
    _trace_step("home", f"START args={_trace_format_value(args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_home(*args, **kwargs)
    _trace_step("home", f"DONE result={_trace_format_value(result)}")
    return result

def return_back_to_home(*args, **kwargs):
    _trace_step("return_back_to_home", f"START args={_trace_format_value(args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_return_back_to_home(*args, **kwargs)
    _trace_step("return_back_to_home", f"DONE result={_trace_format_value(result)}")
    return result

def detect_cup_gripper(*args, **kwargs):
    _trace_step("detect_cup_gripper", "START")
    result = _raw_detect_cup_gripper(*args, **kwargs)
    _trace_step("detect_cup_gripper", f"DONE result={_trace_format_value(result)}")
    return result

def _fail(reason: str = "") -> bool:
    if globals().get("PLASTIC_CUPS_TRACE_DEBUG", True):
        try:
            frame = sys._getframe(1)
            msg = f"FAIL in {frame.f_code.co_name} line={frame.f_lineno}"
            if reason:
                msg += f" reason={reason}"
            _trace_step("fail", msg)
        except Exception:
            pass
    return False

_go_home_with_ice_cache: Dict[str, Tuple[float, ...]] = {}
_place_plastic_cup_station_cache: Dict[str, Tuple[float, ...]] = {}
_pick_plastic_cup_station_cache: Dict[Tuple[str, str], Tuple[float, ...]] = {}
_place_plastic_cup_sauces_cache: Optional[Tuple[float, ...]] = None
_pick_plastic_cup_sauces_cache: Dict[str, Tuple[float, ...]] = {}
_pick_plastic_cup_milk_cache: Dict[str, Tuple[float, ...]] = {}


def invalidate_plastic_cup_cache():
    _trace_step("invalidate_plastic_cup_cache", "START")
    _go_home_with_ice_cache.clear()
    _place_plastic_cup_station_cache.clear()
    _pick_plastic_cup_station_cache.clear()
    global _place_plastic_cup_sauces_cache
    _place_plastic_cup_sauces_cache = None
    _pick_plastic_cup_sauces_cache.clear()
    _pick_plastic_cup_milk_cache.clear()


def _is_valid_angles(angles: Any) -> bool:
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6


def _capture_current_angles() -> Optional[Tuple[float, ...]]:
    _trace_step("_capture_current_angles", "START")
    angles = run_skill("current_angles")
    if not _is_valid_angles(angles):
        return None
    return tuple(angles)


def _normalize_plastic_cup_size(cups_dict: Any) -> str:
    _trace_step("_normalize_plastic_cup_size", "START")
    if not cups_dict:
        from oms_v1.params import DEFAULT_PLASTIC_CUP_SIZE
        return DEFAULT_PLASTIC_CUP_SIZE

    if isinstance(cups_dict, dict):
        cup_key = next(iter(cups_dict.keys()), None)
        if cup_key:
            cup_key_str = str(cup_key).upper()
            if 'CUP_' in cup_key_str:
                cup_code = cup_key_str.split('CUP_', 1)[1]
            else:
                cup_code = cup_key_str

            if cup_code and len(cup_code) >= 2:
                if cup_code[0] in ('H', 'C'):
                    size_num = cup_code[1:]
                else:
                    size_num = cup_code

                if size_num in ('7', '9', '12', '16'):
                    return f"{size_num}oz"

    result = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if result and result != '':
        return result

    result = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if result and result != '':
        return result

    from oms_v1.params import DEFAULT_PLASTIC_CUP_SIZE
    return DEFAULT_PLASTIC_CUP_SIZE


def dispense_plastic_cup(**params) -> bool:
    _trace_step("dispense_plastic_cup", "START")
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_PLASTIC_CUP_SIZE)
    if not cup_size or not validate_cup_size(cup_size):
        return False

    CUP_CONFIG = {
        '16oz': {'home': 'west', 'coords': PLASTIC_CUPS_PARAMS['dispenser']['16oz_coords']},
        '12oz': {'home': 'west', 'coords': PLASTIC_CUPS_PARAMS['dispenser']['12oz_coords']},
        '9oz': {'home': 'south_west', 'coords': PLASTIC_CUPS_PARAMS['dispenser']['9oz_coords']},
        '7oz': {'home': 'south_west', 'coords': PLASTIC_CUPS_PARAMS['dispenser']['7oz_coords']},
    }
    if cup_size not in CUP_CONFIG:
        return False

    DISPENSE_PARAMS = {
        '16oz': {'gripper_pos': 110, 'do_index': 2},
        '12oz': {'gripper_pos': 135, 'do_index': 1},
        '9oz':  {'gripper_pos': 140, 'do_index': 3},
        '7oz':  {'gripper_pos': 130, 'do_index': 1},
    }

    config = CUP_CONFIG[cup_size]
    dp = DISPENSE_PARAMS[cup_size]
    attempt_count = 0
    while attempt_count < 15:
        _trace_step("dispense_plastic_cup", f"attempt={attempt_count + 1}/15 size={cup_size} home={config['home']}")
        home(position=config['home'])
        run_skill("set_gripper_position", 255, 0, 255, verify_position=True)
        run_skill("gotoJ_deg", *config['coords'])
        run_skill("moveEE", 0.0, 328.0, 10.0, 0, 0, 0)
        run_skill("set_gripper_position", 255, dp['gripper_pos'], 255, verify_position=True)
        run_skill("sync")
        run_skill("set_DO", dp['do_index'], 1)
        time.sleep(1.5)
        run_skill("set_DO", dp['do_index'], 0)
        run_skill("moveEE", 0, 0, -150, 0, 0, 0)
        run_skill("moveEE", 0, -328.0, 0, 0, 0, 0)
        run_skill("gotoJ_deg", *config['coords'])
        home(position=config['home'])
        home(position="north")
        run_skill("sync")
        cup_detected = detect_cup_gripper()
        _trace_step("dispense_plastic_cup", f"cup detection result={cup_detected}")
        if cup_detected:
            break
        attempt_count += 1
        if attempt_count == 15:
            return False

    _set_cup_dispensed()
    return True


def go_to_ice(**params) -> bool:
    _trace_step("go_to_ice", "START")
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    if not cup_size:
        return False
    if cup_size not in ('16oz', '12oz', '9oz', '7oz'):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position2'])):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE, verify_position=True)):
        return False
    # run_skill("sync")
    return True


def go_home_with_ice(**params) -> bool:
    _trace_step("go_home_with_ice", "START")
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict if cups_dict else DEFAULT_PLASTIC_CUP_SIZE)
    if not cup_size or not validate_cup_size(cup_size):
        return False

    cached_retreat = _go_home_with_ice_cache.get(cup_size)
    if _is_valid_angles(cached_retreat):
        _trace_step("go_home_with_ice", f"retreat cache HIT size={cup_size}")
        if not ok(run_skill("gotoJ_deg", *cached_retreat)):
            return False
        run_skill("sync")
    else:
        _trace_step("go_home_with_ice", f"retreat cache MISS size={cup_size}; recording current route")
        if not ok(run_skill("moveEE_movJ", -10, 0, 0, 0, 0, 0)):
            return False
        retreat_pose = _capture_current_angles()
        if not _is_valid_angles(retreat_pose):
            return False
        _go_home_with_ice_cache[cup_size] = retreat_pose

    gripper_position = {"7oz": 140, "9oz": 145, "12oz": 135, "16oz": 130}.get(cup_size)
    if gripper_position is None:
        return False
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_position, verify_position=True)):
        return False
    # run_skill("sync")
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position3'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['ice_positions']['position1'])):
        return False
    if not home(position="north"):
        return False
    _set_cup_dispensed()
    return True


def place_plastic_cup_station(**params) -> bool:
    _trace_step("place_plastic_cup_station", "START")
    def ok(r):
        return r not in (False, None)
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    if not cup_size or cup_size not in ('7oz', '9oz', '12oz', '16oz'):
        return False

    _check_and_clear_cup_dispensed()
    run_skill("set_speed_factor", SPEED_NORMAL)

    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False

    if stage == "1":
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_1'])
    elif stage == "2":
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_2'])
    elif stage == "3":
        home(position="south_east")
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_3'])
    elif stage == "4":
        home(position="south_east")
        stage_result = run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['staging']['place_4'])
    else:
        return False

    if not ok(stage_result):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", 25, 100, 25, verify_position=True)):
        return False
    if not ok(run_skill("set_gripper_position", 255, 0, 255, verify_position=True)):
        return False
    # run_skill("sync")
    run_skill("set_speed_factor", SPEED_FAST)

    cached_up = _place_plastic_cup_station_cache.get(stage)
    if _is_valid_angles(cached_up):
        _trace_step("place_plastic_cup_station", f"return-up cache HIT stage={stage}")
        if not ok(run_skill("gotoJ_deg", *cached_up)):
            return False
        # run_skill("sync")
    else:
        _trace_step("place_plastic_cup_station", f"return-up cache MISS stage={stage}; recording pose")
        if not ok(run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['place_return_up'])):
            return False
        up_pose = _capture_current_angles()
        if not _is_valid_angles(up_pose):
            return False
        _place_plastic_cup_station_cache[stage] = up_pose

    if not home(position="east"):
        return False
    return True


def pick_plastic_cup_station(**params) -> bool:
    _trace_step("pick_plastic_cup_station", "START")
    def ok(r):
        return r not in (False, None)
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict)
    if not cup_size or cup_size not in ('7oz', '9oz', '12oz', '16oz'):
        return False

    stage_positions = {
        "1": PLASTIC_CUPS_PARAMS['staging']['pickup_1'],
        "2": PLASTIC_CUPS_PARAMS['staging']['pickup_2'],
        "3": PLASTIC_CUPS_PARAMS['staging']['pickup_3'],
        "4": PLASTIC_CUPS_PARAMS['staging']['pickup_4']
    }
    gripper_positions = {"7oz": 145, "9oz": 125, "12oz": 135, "16oz": 118}

    if not home(position="north_east"):
        return False
    if not home(position="east"):
        return False
    if stage in ("3", "4"):
        if not home(position="south_east"):
            return False

    if not ok(run_skill("gotoJ_deg", *stage_positions[stage])):
        return False
    run_skill("sync")

    cache_key = (stage, cup_size)
    cached_down = _pick_plastic_cup_station_cache.get(cache_key)
    if _is_valid_angles(cached_down):
        _trace_step("pick_plastic_cup_station", f"pickup-down cache HIT stage={stage} size={cup_size}")
        if not ok(run_skill("gotoJ_deg", *cached_down)):
            return False
        run_skill("sync")
    else:
        _trace_step("pick_plastic_cup_station", f"pickup-down cache MISS stage={stage} size={cup_size}; recording pose")
        if not ok(run_skill("moveEE", *PLASTIC_CUP_MOVEMENT_OFFSETS['pickup_down'])):
            return False
        down_pose = _capture_current_angles()
        if not _is_valid_angles(down_pose):
            return False
        _pick_plastic_cup_station_cache[cache_key] = down_pose

    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size], verify_position=True)):
        return False
    run_skill("set_speed_factor", SPEED_NORMAL)
    run_skill("sync")

    if cup_position in (3, 4):
        if not home(position="south_east"):
            return False
    if not home(position="east"):
        return False
    if not home(position="north_east"):
        return False
    return True


def place_plastic_cup_sauces(**params) -> bool:
    _trace_step("place_plastic_cup_sauces", "START")
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    if not cup_size or cup_size not in ("7oz", "9oz", "12oz", "16oz"):
        return False

    after_dispense = _check_and_clear_cup_dispensed()
    if not after_dispense:
        run_skill("set_speed_factor", SPEED_NORMAL)

    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position2'])):
        return False
    # run_skill("sync")

    global _place_plastic_cup_sauces_cache
    if _is_valid_angles(_place_plastic_cup_sauces_cache):
        if not ok(run_skill("gotoJ_deg", *_place_plastic_cup_sauces_cache)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE", -5, 0, 0, 0, 0, 0)):
            return False
        nudge_pose = _capture_current_angles()
        if not _is_valid_angles(nudge_pose):
            return False
        _place_plastic_cup_sauces_cache = nudge_pose

    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE, verify_position=True)):
        return False
    # run_skill("sync")
    cup_detected = detect_cup_gripper()
    if not cup_detected:
        return False
    return True


def pick_plastic_cup_sauces(**params) -> bool:
    _trace_step("pick_plastic_cup_sauces", "START")
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    if not cup_size or cup_size not in ("7oz", "9oz", "12oz", "16oz"):
        return False

    _check_and_clear_cup_dispensed()
    gripper_positions = {"7oz": 140, "9oz": 140, "12oz": 135, "16oz": 130}
    run_skill("set_speed_factor", SPEED_NORMAL)
    if not detect_cup_gripper():
        return False

    cached_lift = _pick_plastic_cup_sauces_cache.get(cup_size)
    if _is_valid_angles(cached_lift):
        if not ok(run_skill("gotoJ_deg", *cached_lift)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE", 0, 0, 1, 0, 0, 0)):
            return False
        lift_pose = _capture_current_angles()
        if not _is_valid_angles(lift_pose):
            return False
        _pick_plastic_cup_sauces_cache[cup_size] = lift_pose

    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size], verify_position=True)):
        return False
    # run_skill("sync")
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['sauces_station']['position1'])):
        return False
    return True


def place_plastic_cup_milk(**params) -> bool:
    _trace_step("place_plastic_cup_milk", "START")
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    if not cup_size or cup_size not in ("7oz", "9oz", "12oz", "16oz"):
        return False

    after_dispense = _check_and_clear_cup_dispensed()
    if not after_dispense:
        run_skill("set_speed_factor", SPEED_NORMAL)

    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position2'])):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE, verify_position=True)):
        return False
    # run_skill("sync")
    if not detect_cup_gripper():
        return False
    return True


def pick_plastic_cup_milk(**params) -> bool:
    _trace_step("pick_plastic_cup_milk", "START")
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_plastic_cup_size(cups_dict) if cups_dict else None
    if not cup_size or cup_size not in ("7oz", "9oz", "12oz", "16oz"):
        return False

    _check_and_clear_cup_dispensed()
    gripper_positions = {"7oz": 140, "9oz": 140, "12oz": 135, "16oz": 130}
    run_skill("set_speed_factor", SPEED_NORMAL)
    if not detect_cup_gripper():
        return False

    cached_lift = _pick_plastic_cup_milk_cache.get(cup_size)
    if _is_valid_angles(cached_lift):
        if not ok(run_skill("gotoJ_deg", *cached_lift)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE", -5, 0, 1, 0, 0, 0)):
            return False
        lift_pose = _capture_current_angles()
        if not _is_valid_angles(lift_pose):
            return False
        _pick_plastic_cup_milk_cache[cup_size] = lift_pose

    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, gripper_positions[cup_size], verify_position=True)):
        return False
    # run_skill("sync")
    if not ok(run_skill("gotoJ_deg", *PLASTIC_CUPS_PARAMS['milk_station']['position1'])):
        return False
    return True


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
    'invalidate_plastic_cup_cache': invalidate_plastic_cup_cache,
}