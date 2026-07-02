"""
slush.py

Defines the slush handling sequences for cold beverage preparation automation.
This module provides comprehensive functions for dispensing and placing slush drinks
in the BARNS coffee automation system, supporting multiple dispensers and staging areas
for frozen beverage preparation.
"""
# NOTE: Gripper commands can opt into strict register verification via run_skill(..., verify_position=True).


import sys
import inspect
from typing import Dict, Any, Optional
from oms_v1.manipulate_node import run_skill as _raw_run_skill
from oms_v1.sequences.home import home as _raw_home
from oms_v1.sequences.plastic_cups import dispense_plastic_cup, place_plastic_cup_station

# Runtime trace helpers for cold/slush flow.
SLUSH_TRACE_DEBUG = True

def _trace_format_value(value, max_len: int = 140) -> str:
    try:
        text = repr(value)
    except Exception:
        text = f"<{type(value).__name__}>"
    return text if len(text) <= max_len else text[:max_len - 3] + "..."

def _trace_step(scope: str, message: str) -> None:
    if globals().get("SLUSH_TRACE_DEBUG", True):
        print(f"[SLUSH:{scope}] {message}", flush=True)

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

def _fail(reason: str = "") -> bool:
    if globals().get("SLUSH_TRACE_DEBUG", True):
        try:
            frame = sys._getframe(1)
            msg = f"FAIL in {frame.f_code.co_name} line={frame.f_lineno}"
            if reason:
                msg += f" reason={reason}"
            _trace_step("fail", msg)
        except Exception:
            pass
    return False

from oms_v1.params import (
    SLUSH_PARAMS, SPEED_NORMAL, DEFAULT_PLASTIC_CUP_SIZE,
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size,
    GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE
)


def _normalize_slush_cup_size(cups_dict: Any) -> str:
    _trace_step("_normalize_slush_cup_size", "START")
    if not cups_dict:
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
    return DEFAULT_PLASTIC_CUP_SIZE


def get_slush(**params) -> bool:
    _trace_step("get_slush", "START")
    def ok(r):
        return r not in (False, None)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_slush_cup_size(cups_dict)
    dispenser = params.get("dispenser")
    if not dispenser:
        _trace_step("get_slush", "no dispenser provided; deriving from premix")
        premixes = params.get("premixes", {})
        if premixes:
            premix_name = list(premixes.keys())[0] if premixes else ""
            dispenser = "2" if ("chocolate" in premix_name.lower() or "choco" in premix_name.lower()) else "1"
        else:
            dispenser = "1"
    if cup_size not in ("7oz", "9oz", "12oz", "16oz") or dispenser not in ("1", "2"):
        return _fail(f"invalid cup_size={cup_size} dispenser={dispenser}")
    cup_code = f"cup_C{cup_size.replace('oz', '')}"
    if not dispense_plastic_cup(cups={cup_code: 1.0}):
        return False
    if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['intermediate'])):
        return False
    if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])):
        return False
    if dispenser == "2":
        dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['dispense'])
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['intermediate'])):
            return False
        dispenser_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['dispense'])
        run_skill("sync")
        if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, GRIPPER_HOLD_LOOSE)):
            return False
        run_skill("sync")
    if not ok(dispenser_result):
        return False
    return True


def place_slush(**params) -> bool:
    _trace_step("place_slush", "START")
    def ok(r):
        return r not in (False, None)
    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_slush_cup_size(cups_dict)
    dispenser = params.get("dispenser")
    if not dispenser:
        _trace_step("place_slush", "no dispenser provided; deriving from premix")
        premixes = params.get("premixes", {})
        if premixes:
            premix_name = list(premixes.keys())[0] if premixes else ""
            dispenser = "2" if ("chocolate" in premix_name.lower() or "choco" in premix_name.lower()) else "1"
        else:
            dispenser = "1"
    if cup_size not in ("7oz", "9oz", "12oz", "16oz") or dispenser not in ("1", "2"):
        return _fail(f"invalid cup_size={cup_size} dispenser={dispenser}")
    run_skill("set_speed_factor", SPEED_NORMAL)
    gripper_positions = {"7oz": 140, "9oz": 140, "12oz": 140, "16oz": 130}
    if not ok(run_skill("set_gripper_position", GRIPPER_RELEASE_GENTLE, gripper_positions[cup_size])):
        return False
    run_skill("sync")
    if dispenser == "2":
        retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_2']['retreat'])
    else:
        retreat_result = run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['retreat'])
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['dispenser_1']['intermediate'])):
            return False
        if not ok(run_skill("gotoJ_deg", *SLUSH_PARAMS['navigation']['slush_area'])):
            return False
    if not ok(retreat_result):
        return False
    if not home(position="north"):
        return False
    cup_code = f"cup_C{cup_size.replace('oz', '')}"
    if not place_plastic_cup_station(position={'cup_position': int(stage)}, cups={cup_code: 1.0}):
        return False
    return True


SEQUENCES = {
    'get_slush': get_slush,
    'place_slush': place_slush,
}