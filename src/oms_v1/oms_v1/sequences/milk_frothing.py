"""
milk_frothing.py

Defines the milk frothing sequences for coffee preparation automation.
This module provides comprehensive functions for handling milk frothing operations
in the BARNS coffee automation system, including frother positioning, mounting,
steam activation, milk pouring, and cleaning procedures.
"""
# NOTE: Gripper commands can opt into strict register verification via run_skill(..., verify_position=True).


import logging
import time
import sys
import inspect
from typing import Dict, Any, Optional, Tuple
from oms_v1.manipulate_node import run_skill as _raw_run_skill, init_motion_node
from oms_v1.sequences.home import home, return_back_to_home

_log = logging.getLogger(__name__)
MAX_FROTHER_CALIBRATION_RETRIES = 5
from oms_v1.params import (
    MILK_FROTHING_PARAMS, MILK_FROTHER_SPEEDS, MILK_FROTHER_GRIPPER_POSITIONS,
    MILK_FROTHER_MOVEMENT_OFFSETS, MILK_POURING_OFFSETS, MILK_FROTHING_DELAYS,
    MILK_SWIRL_CIRCLE_PARAMS, MILK_VOLUME_Z_ADJUSTMENT_FACTOR_BY_CUP_SIZE,
    MILK_FROTHING_EXTRA_POSES,
    GRIPPER_FULL, GRIPPER_OPEN, CALIBRATION_SETTLE_TIME,
    _extract_cup_position, _extract_cups_dict, _normalize_cup_size
)
from oms_v1.sequences.plastic_cups import invalidate_plastic_cup_cache

# Runtime trace helpers. These mirror espresso.py/paper_cups.py and keep robot
# flow observable without changing command order or return values.
MILK_FROTHING_TRACE_DEBUG = True

def _trace_format_value(value, max_len: int = 140) -> str:
    try:
        text = repr(value)
    except Exception:
        text = f"<{type(value).__name__}>"
    return text if len(text) <= max_len else text[:max_len - 3] + "..."

def _trace_step(scope: str, message: str) -> None:
    if globals().get("MILK_FROTHING_TRACE_DEBUG", True):
        print(f"[MILK-FROTHING:{scope}] {message}", flush=True)

def run_skill(*args, **kwargs):
    """Trace wrapper around manipulate_node.run_skill."""
    skill_name = args[0] if args else "<missing>"
    skill_args = args[1:] if len(args) > 1 else ()
    _trace_step("run_skill", f"{skill_name} START args={_trace_format_value(skill_args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_run_skill(*args, **kwargs)
    _trace_step("run_skill", f"{skill_name} DONE result={_trace_format_value(result)}")
    return result

def _fail(reason: str = "") -> bool:
    """Log failure context while preserving the existing False return contract."""
    if globals().get("MILK_FROTHING_TRACE_DEBUG", True):
        try:
            frame = sys._getframe(1)
            msg = f"FAIL in {frame.f_code.co_name} line={frame.f_lineno}"
            if reason:
                msg += f" reason={reason}"
            _trace_step("fail", msg)
        except Exception:
            pass
    return False

_place_frother_milk_station_cache: Dict[str, Tuple[float, ...]] = {}
_pick_frother_milk_station_cache: Optional[Tuple[float, ...]] = None
_mount_frother_cache: Dict[str, Tuple[float, ...]] = {}
_pour_milk_cup_station_cache: Dict[str, Dict[str, Tuple[float, ...]]] = {}
_clean_milk_pitcher_cache: Optional[Tuple[float, ...]] = None
_return_frother_cache: Dict[str, Tuple[float, ...]] = {}
_get_frother_position_done: bool = False
_milk_frother_position_done: bool = False
_pick_frother_cache: Dict[str, Tuple[float, ...]] = {}

# Gripper reading expected after securing the milk frother.
# Accept +/- tolerance because the decoded gripper register can stabilize at
# 228/229/231/232 while the physical grip is still correct.
PICK_FROTHER_GRIP_VERIFY_TARGET = 230
PICK_FROTHER_GRIP_VERIFY_TOLERANCE = 2
PICK_FROTHER_GRIP_VERIFY_RETRIES = 15


def invalidate_milk_frothing_cache():
    _trace_step("invalidate_milk_frothing_cache", "START")
    global _pick_frother_milk_station_cache, _clean_milk_pitcher_cache
    global _get_frother_position_done, _milk_frother_position_done

    _place_frother_milk_station_cache.clear()
    _pick_frother_milk_station_cache = None
    _mount_frother_cache.clear()
    _pour_milk_cup_station_cache.clear()
    _clean_milk_pitcher_cache = None
    _return_frother_cache.clear()
    _pick_frother_cache.clear()
    _get_frother_position_done = False
    _milk_frother_position_done = False

def _is_valid_angles(angles: Any) -> bool:
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6


def _capture_current_angles() -> Optional[Tuple[float, ...]]:
    angles = run_skill("current_angles")
    if not _is_valid_angles(angles):
        return None
    return tuple(angles)


def _capture_current_position() -> Optional[Tuple[float, ...]]:
    position = run_skill("current_pose")
    if not _is_valid_position(position):
        return None
    return tuple(position)


def _is_valid_position(position: Any) -> bool:
    return bool(position) and isinstance(position, (tuple, list)) and len(position) == 6


def _cache_key_from_z_adjustment(z_adjustment: float) -> str:
    return f"{round(float(z_adjustment), 3):.3f}"


def get_frother_position(**params) -> bool:
    """
    Calibrate and record the milk frother position for future operations.
    After first successful calibration, skip re-reading machine position on later runs.
    """
    _trace_step("get_frother_position", "START")
    def ok(r):
        return r not in (False, None)

    global _get_frother_position_done, _milk_frother_position_done

    invalidate_plastic_cup_cache()
    invalidate_milk_frothing_cache()

    run_skill("set_speed_factor", 100)
    if not return_back_to_home():
        return False

    if not home(position="north_east"):
        return False

    if not _get_frother_position_done:
        steam_calibrated = False
        for attempt in range(1, MAX_FROTHER_CALIBRATION_RETRIES + 1):
            prep_ok = True
            cycles = 3
            for _ in range(cycles):
                time.sleep(CALIBRATION_SETTLE_TIME)
                if not ok(run_skill("move_to", "left_steam_wand", 0.29)):
                    prep_ok = False
                    break
            if not prep_ok:
                _log.warning(f"[CALIBRATION] left_steam_wand prep failed (attempt {attempt}/{MAX_FROTHER_CALIBRATION_RETRIES})")
                time.sleep(1.0)
                continue
            if ok(run_skill("get_machine_position", "left_steam_wand")):
                steam_calibrated = True
                break
            _log.warning(f"[CALIBRATION] left_steam_wand failed (attempt {attempt}/{MAX_FROTHER_CALIBRATION_RETRIES})")
            time.sleep(1.0)
        if not steam_calibrated:
            _log.error(f"[CALIBRATION] left_steam_wand failed after {MAX_FROTHER_CALIBRATION_RETRIES} attempts")
            return False
        _get_frother_position_done = True

    if not home(position="north_east"):
        return False
    
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_EXTRA_POSES['frother_calibration_transition'])):
        return False

    if not _milk_frother_position_done:
        frother_calibrated = False
        for attempt in range(1, MAX_FROTHER_CALIBRATION_RETRIES + 1):
            prep_ok = True
            cycles = 3
            for _ in range(cycles):
                time.sleep(CALIBRATION_SETTLE_TIME)
                if not ok(run_skill("move_to", "milk_frother_2", 0.29)):
                    prep_ok = False
                    break
            if not prep_ok:
                _log.warning(f"[CALIBRATION] milk_frother_2 prep failed (attempt {attempt}/{MAX_FROTHER_CALIBRATION_RETRIES})")
                time.sleep(1.0)
                continue
            if ok(run_skill("get_machine_position", "milk_frother_2")):
                frother_calibrated = True
                break
            _log.warning(f"[CALIBRATION] milk_frother_2 failed (attempt {attempt}/{MAX_FROTHER_CALIBRATION_RETRIES})")
            time.sleep(1.0)
        if not frother_calibrated:
            _log.error(f"[CALIBRATION] milk_frother_2 failed after {MAX_FROTHER_CALIBRATION_RETRIES} attempts")
            return False
        _milk_frother_position_done = True

    
    if not home(position="north"):
        return False

    return True


def pick_frother(**params) -> bool:
    """
    Pick up the milk frother for milk frothing operations.

    Live/no-cache pickup verification:
    - After the secure gripper close, the reported gripper position must be
      within PICK_FROTHER_GRIP_VERIFY_TARGET +/- PICK_FROTHER_GRIP_VERIFY_TOLERANCE.
    - If verification fails, reverse back to the pickup area, refresh only the
      milk_frother_2 machine position, and retry the whole live pickup routine.

    Important: do NOT call the full get_frother_position() inside this recovery.
    That function runs home/return routines and can trigger drag/tension-release
    behavior while the robot is still in the pickup flow.
    """
    _trace_step("pick_frother", "START")

    def ok(r):
        return r not in (False, None)

    def _secure_gripper_and_verify():
        """
        Close the gripper to the secure position and verify the reported value.

        This replaces the separate run_skill("set_gripper_position", ...) +
        second read/close call, so we do not issue duplicate secure closes.
        """
        node = init_motion_node()
        success, pos = node.set_gripper_position(
            speed=GRIPPER_FULL,
            position=MILK_FROTHER_GRIPPER_POSITIONS['secure'],
            force=255,
        )
        if not success:
            return False, None

        verified = (
            pos is not None
            and abs(float(pos) - float(PICK_FROTHER_GRIP_VERIFY_TARGET))
            <= float(PICK_FROTHER_GRIP_VERIFY_TOLERANCE)
        )
        return verified, pos

    def _refresh_milk_frother_position_only(attempt_idx: int) -> bool:
        """
        Re-acquire only milk_frother_2 position without calling full
        get_frother_position().

        Full get_frother_position() calls invalidate/home/return logic and can
        trigger drag mode / gripper opening. For retry recovery we only need the
        frother target refreshed before re-running approach/mount.
        """
        global _milk_frother_position_done

        _log.warning(
            f"[PICK-FROTHER-GRIP] retry recovery {attempt_idx}/"
            f"{PICK_FROTHER_GRIP_VERIFY_RETRIES}: refreshing milk_frother_2 "
            f"position only; not running full get_frother_position()"
        )

        frother_calibrated = False
        for calib_attempt in range(1, MAX_FROTHER_CALIBRATION_RETRIES + 1):
            prep_ok = True
            for _ in range(3):
                time.sleep(CALIBRATION_SETTLE_TIME)
                if not ok(run_skill("move_to", "milk_frother_2", 0.29)):
                    prep_ok = False
                    break

            if not prep_ok:
                _log.warning(
                    f"[PICK-FROTHER-GRIP] milk_frother_2 prep failed during retry "
                    f"{attempt_idx}, calibration attempt "
                    f"{calib_attempt}/{MAX_FROTHER_CALIBRATION_RETRIES}"
                )
                time.sleep(1.0)
                continue

            if ok(run_skill("get_machine_position", "milk_frother_2")):
                frother_calibrated = True
                break

            _log.warning(
                f"[PICK-FROTHER-GRIP] milk_frother_2 get_machine_position failed "
                f"during retry {attempt_idx}, calibration attempt "
                f"{calib_attempt}/{MAX_FROTHER_CALIBRATION_RETRIES}"
            )
            time.sleep(1.0)

        if not frother_calibrated:
            _log.error(
                f"[PICK-FROTHER-GRIP] milk_frother_2 refresh failed after "
                f"{MAX_FROTHER_CALIBRATION_RETRIES} attempts during retry {attempt_idx}"
            )
            return False

        _milk_frother_position_done = True
        return True

    def _reverse_to_pickup_area_and_recalibrate(attempt_idx: int) -> bool:
        """
        Recovery path after a failed secure-grip verification.

        Uses the reverse sequence requested:
        sync -> mount_machine -> pickup_initial close -> sync -> approach_machine
        -> pickup area -> refresh frother target -> retry live pickup.
        """
        _log.warning(
            f"[PICK-FROTHER-GRIP] retry recovery {attempt_idx}/"
            f"{PICK_FROTHER_GRIP_VERIFY_RETRIES}: reversing to pickup area"
        )

        run_skill("sync")

        if not ok(run_skill("mount_machine", 'milk_frother_2', 'milk_frother_1')):
            return False

        if not ok(run_skill(
            "set_gripper_position",
            GRIPPER_FULL,
            MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'],
        )):
            return False

        run_skill("sync")

        if not ok(run_skill("approach_machine", 'milk_frother_2', 'milk_frother_1')):
            return False

        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area'])):
            return False

        if not _refresh_milk_frother_position_only(attempt_idx):
            return False

        return True

    total_attempts = PICK_FROTHER_GRIP_VERIFY_RETRIES + 1

    # This function currently performs the live/no-cache pickup path.
    # Keep the gripper verification and recovery inside this live path only.
    for attempt_idx in range(1, total_attempts + 1):
        _log.info(
            f"[PICK-FROTHER-GRIP] pickup attempt {attempt_idx}/{total_attempts}"
        )

        if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['pickup']['area'])):
            return False

        if not ok(run_skill("approach_machine", 'milk_frother_2', 'milk_frother_1')):
            return False

        run_skill("sync")

        if not ok(run_skill(
            "set_gripper_position",
            GRIPPER_FULL,
            MILK_FROTHER_GRIPPER_POSITIONS['pickup_initial'],
        )):
            return False

        if not ok(run_skill("mount_machine", 'milk_frother_2', 'milk_frother_1')):
            return False

        run_skill("sync")

        gripper_verified, gripper_pos = _secure_gripper_and_verify()

        if gripper_verified:
            _log.info(
                f"[PICK-FROTHER-GRIP] secure grip verified: pos={gripper_pos}, "
                f"target={PICK_FROTHER_GRIP_VERIFY_TARGET}, "
                f"tol=+/-{PICK_FROTHER_GRIP_VERIFY_TOLERANCE}"
            )
            return True

        _log.warning(
            f"[PICK-FROTHER-GRIP] secure grip verify failed on attempt "
            f"{attempt_idx}/{total_attempts}: pos={gripper_pos}, "
            f"required={PICK_FROTHER_GRIP_VERIFY_TARGET} "
            f"+/-{PICK_FROTHER_GRIP_VERIFY_TOLERANCE}"
        )

        if attempt_idx >= total_attempts:
            _log.error(
                f"[PICK-FROTHER-GRIP] FINAL FAIL after {total_attempts} attempts: "
                f"last_pos={gripper_pos}, required={PICK_FROTHER_GRIP_VERIFY_TARGET} "
                f"+/-{PICK_FROTHER_GRIP_VERIFY_TOLERANCE}"
            )
            return False

        if not _reverse_to_pickup_area_and_recalibrate(attempt_idx):
            return False

    return False

def place_frother_milk_station(**params) -> bool:
    _trace_step("place_frother_milk_station", "START")
    def ok(r):
        return r not in (False, None)

    cached_lift = _place_frother_milk_station_cache.get('lift_after_place')
    cached_nudge = _place_frother_milk_station_cache.get('final_nudge')

    if _is_valid_position(cached_lift):
        if not ok(run_skill("gotoEE", *cached_lift)):
            return False
    else:
        if not ok(run_skill("moveEE", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_place'])):
            return False
        lift_pose = _capture_current_position()
        if not _is_valid_position(lift_pose):
            return False
        _place_frother_milk_station_cache['lift_after_place'] = lift_pose

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_pre2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_approach'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['place_final'])):
        return False

    if _is_valid_angles(cached_nudge):
        if not ok(run_skill("gotoJ_deg", *cached_nudge)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", 5, 0, 0, 0, 0, 0)):
            return False
        nudge_pose = _capture_current_angles()
        if not _is_valid_angles(nudge_pose):
            return False
        _place_frother_milk_station_cache['final_nudge'] = nudge_pose

    if not ok(run_skill("set_gripper_position", 25, 220, 255)):
        return False
    return True


def pick_frother_milk_station(**params) -> bool:
    _trace_step("pick_frother_milk_station", "START")
    def ok(r):
        return r not in (False, None)

    global _pick_frother_milk_station_cache

    if not ok(run_skill("set_gripper_position", 255, 255, 255)):
        return False

    if _is_valid_angles(_pick_frother_milk_station_cache):
        if not ok(run_skill("gotoJ_deg", *_pick_frother_milk_station_cache)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['lift_after_pick'])):
            return False
        lift_pose = _capture_current_angles()
        if not _is_valid_angles(lift_pose):
            return False
        _pick_frother_milk_station_cache = lift_pose

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['milk_station']['pick_retreat2'])):
        return False
    return True


def mount_frother(**params) -> bool:
    """
    Mount the milk frother to the steam wand for frothing preparation.
    Applies a Z adjustment based on milk volume and cup size.
    """
    def ok(r):
        return r not in (False, None)

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['mount'])
    if not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth")):
        return False
    if not ok(run_skill("mount_machine", "left_steam_wand", "deep_froth")):
        return False

    cups_dict = _extract_cups_dict(params)
    cup_size = _normalize_cup_size(cups_dict, cup_type='paper', default_size='')
    if not cup_size:
        cup_size = _normalize_cup_size(cups_dict, cup_type='plastic', default_size='')
    if not cup_size:
        cup_size = 'default'

    milk_data = params.get('milk') or params.get('ingredients', {}).get('milk', {}) or {}
    try:
        volume_ml = float(next(iter(milk_data.values()), 0)) if isinstance(milk_data, dict) else float(milk_data or 0)
    except (TypeError, ValueError):
        volume_ml = 0.0

    factor = MILK_VOLUME_Z_ADJUSTMENT_FACTOR_BY_CUP_SIZE.get(
        cup_size,
        MILK_VOLUME_Z_ADJUSTMENT_FACTOR_BY_CUP_SIZE['default']
    )
    z_adjustment = factor * volume_ml

    if not ok(run_skill("moveEE_movJ", 12.5, 12.5, -z_adjustment, 7.5, 0, 0)):
        return False

    return True


def unmount_and_swirl_milk(**params) -> bool:
    """
    Swirl frothed milk in a circular motion for latte art preparation.
    """
    def ok(r):
        return r not in (False, None)

    time.sleep(MILK_FROTHING_DELAYS['swirl_delay'])

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_EXTRA_POSES['deep_froth_approach'])): # if not ok(run_skill("approach_machine", "left_steam_wand", "deep_froth")):
        return False

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['swirl'])

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['intermediate1'])):
        return False

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['swirling']['swirl_pos'])):
        return False

    run_skill("sync")

    run_skill(
        "move_circle",
        MILK_SWIRL_CIRCLE_PARAMS['cycles'],
        MILK_SWIRL_CIRCLE_PARAMS['point1_offset'],
        MILK_SWIRL_CIRCLE_PARAMS['point2_offset'],
        MILK_SWIRL_CIRCLE_PARAMS['options']
    )

    return True


def pour_milk_cup_station(**params) -> bool:
    _trace_step("pour_milk_cup_station", "START")
    def ok(r):
        return r not in (False, None)

    cup_position = _extract_cup_position(params)
    stage = str(cup_position)
    stage_cache = _pour_milk_cup_station_cache.setdefault(stage, {})
    stage_cfg = MILK_FROTHING_PARAMS['pouring'][f'stage{stage}']
    stage_offsets = MILK_POURING_OFFSETS[f'stage{stage}']

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour_approach'])
    if not ok(run_skill("gotoJ_deg", *stage_cfg['position'])):
        return False
    run_skill("sync")

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['pour'])
    if not ok(run_skill("gotoJ_deg", *stage_cfg['adjust1'])):
        return False

    if _is_valid_angles(stage_cache.get('forward')):
        if not ok(run_skill("gotoJ_deg", *stage_cache['forward'])):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", *stage_offsets['move_forward'])):
            return False
        forward_pose = _capture_current_angles()
        if not _is_valid_angles(forward_pose):
            return False
        stage_cache['forward'] = forward_pose

    if _is_valid_angles(stage_cache.get('up')):
        if not ok(run_skill("gotoJ_deg", *stage_cache['up'])):
            return False
    else:
        if not ok(run_skill("moveEE_movJ", *stage_offsets['move_up'])):
            return False
        up_pose = _capture_current_angles()
        if not _is_valid_angles(up_pose):
            return False
        stage_cache['up'] = up_pose

    if not ok(run_skill("gotoJ_deg", *stage_cfg['position'])):
        return False

    run_skill("set_speed_factor", MILK_FROTHER_SPEEDS['return'])
    return True


def clean_milk_pitcher(**params) -> bool:
    _trace_step("clean_milk_pitcher", "START")
    def ok(r):
        return r not in (False, None)

    global _clean_milk_pitcher_cache

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['cleaning']['pose3'])):
        return False

    if _is_valid_angles(_clean_milk_pitcher_cache):
        if not ok(run_skill("gotoJ_deg", *_clean_milk_pitcher_cache)):
            return False
        run_skill("sync")
    else:
        if not ok(run_skill("moveEE_movJ", *MILK_FROTHER_MOVEMENT_OFFSETS['cleaning_motion'])):
            return False
        clean_pose = _capture_current_angles()
        if not _is_valid_angles(clean_pose):
            return False
        _clean_milk_pitcher_cache = clean_pose

    return True


def return_frother(**params) -> bool:
    """
    Return the frother to its original location using recorded approach/grab angles.
    """

    def ok(r):
        return r not in (False, None)

    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return1'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return2'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return3'])):
        return False
    if not ok(run_skill("gotoJ_deg", *MILK_FROTHING_PARAMS['return']['pre_return4'])):
        return False

    cached_lift = _place_frother_milk_station_cache.get('lift_after_place')
    if not _is_valid_position(cached_lift):
        return False
    if not ok(run_skill("gotoEE", *cached_lift)):
        return False
    if not ok(run_skill("moveEE", 0, 0, -145, 0, 0, 0)):
        return False
    time.sleep(0.5)
    if not ok(run_skill("set_gripper_position", 75, 165, 255)):
        return False
    time.sleep(0.5)
    if not ok(run_skill("moveEE", *MILK_FROTHER_MOVEMENT_OFFSETS['final_approach'])):
        return False
    if not ok(run_skill("approach_machine", "milk_frother_2", "milk_frother_1")):
        return False
    if not ok(home(position="north")):
        return False
    run_skill("sync")
    if not ok(run_skill("set_gripper_position", GRIPPER_FULL, MILK_FROTHER_GRIPPER_POSITIONS['open'])):
        return False
    return True


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
    'invalidate_milk_frothing_cache': invalidate_milk_frothing_cache,
}