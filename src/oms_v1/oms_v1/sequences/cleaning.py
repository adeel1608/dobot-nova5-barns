"""
cleaning.py

Defines the cleaning routine for machine ports using parameterized configurations.
This module provides comprehensive portafilter cleaning functionality for the BARNS
coffee automation system, including hard brush and soft brush cleaning sequences
with precise positioning and error handling.
"""
# NOTE: Gripper commands can opt into strict register verification via run_skill(..., verify_position=True).


import sys
import inspect
from typing import Dict, Any, List, Tuple
from oms_v1.manipulate_node import run_skill as _raw_run_skill
from oms_v1.params import (
    ESPRESSO_GRINDER_HOME, CLEANING_PARAMS, CLEANING_POSES, DEFAULT_PORT,
)


# Runtime tracing helpers.
# Set CLEANING_TRACE_DEBUG = False to disable these step prints without changing behavior.
CLEANING_TRACE_DEBUG = True

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
    if globals().get("CLEANING_TRACE_DEBUG", True):
        print(f"[{scope}] {message}", flush=True)

def run_skill(*args, **kwargs):
    """Trace wrapper around manipulate_node.run_skill."""
    skill_name = args[0] if args else "<missing>"
    skill_args = args[1:] if len(args) > 1 else ()
    _trace_step("CLEANING", f"run_skill {skill_name} START args={_trace_format_value(skill_args)} kwargs={_trace_format_value(kwargs)}")
    result = _raw_run_skill(*args, **kwargs)
    _trace_step("CLEANING", f"run_skill {skill_name} DONE result={_trace_format_value(result)}")
    return result


def _fail(reason: str = "") -> bool:
    """Log caller + nearby source line, then return False.

    This keeps the original bool contract while making silent guard failures
    visible in stdout. Use as: return _fail() or return _fail("reason").
    """
    if not globals().get("CLEANING_TRACE_DEBUG", True):
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

        _trace_step("CLEANING", msg)

    except Exception:
        pass

    return False

_hard_brush_clean_cache: Dict[str, List[Tuple[float, ...]]] = {}
_soft_brush_clean_cache: Dict[str, List[Tuple[float, ...]]] = {}

def ok(r):
    _trace_step("CLEANING", "ok START")
    return r not in (False, None)

def invalidate_cleaning_cache():
    _trace_step("CLEANING", "invalidate_cleaning_cache START")
    _hard_brush_clean_cache.clear()
    _soft_brush_clean_cache.clear()

def _is_valid_angles(angles: Any) -> bool:
    _trace_step("CLEANING", "_is_valid_angles START")
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6

def _capture_and_cache_current_angles(cache_list: List[Tuple[float, ...]]) -> bool:
    _trace_step("CLEANING", "_capture_and_cache_current_angles START")
    angles = run_skill("current_angles")
    if not _is_valid_angles(angles):
        return _fail()
    cache_list.append(tuple(angles))
    return True

def clean_portafilter(**params) -> bool:
    """
    Keep approach/mount live, cache only the post-mount motion sequence.
    """
    _trace_step("CLEANING", "clean_portafilter START")
    from oms_v1.sequences.espresso import _normalize_espresso_shot

    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    if shot_cfg and shot_cfg.get("angled"):
        return angled_clean_portafilter(**params)

    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else DEFAULT_PORT)

    def ok(r):
        return r not in (False, None)

    hard_cached = _hard_brush_clean_cache.get(port)
    soft_cached = _soft_brush_clean_cache.get(port)

    if not ok(run_skill("gotoJ_deg", *CLEANING_POSES['pre_clean_home'])):
        return _fail()

    if not ok(run_skill("approach_machine", "portafilter_cleaner", "hard_brush")):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *CLEANING_PARAMS['hard_brush_adjust'])):
        return _fail()
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "hard_brush")):
        return _fail()

    if hard_cached:
        _trace_step("CLEANING", f"hard brush cache HIT port={port} len={len(hard_cached)}")
        if len(hard_cached) != 5:
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[0])):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[1])):
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[2])):
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[3])):
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[4])):
            return _fail()
    else:
        _trace_step("CLEANING", f"hard brush cache MISS port={port}; recording live sequence")
        hard_capture: List[Tuple[float, ...]] = []
        if not ok(run_skill("moveEE_movJ", 0, 0, 50, 0, 0, 0)):
            return _fail()
        if not _capture_and_cache_current_angles(hard_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0, 0, -30, -2.5, 0, 0)):
            return _fail()
        if not _capture_and_cache_current_angles(hard_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", -7.5, 7.5, 0, 0, 0, 0)):
            return _fail()
        if not _capture_and_cache_current_angles(hard_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 15.0, -15.0, 0, 0, 0, 0)):
            return _fail()
        if not _capture_and_cache_current_angles(hard_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_hard'])):
            return _fail()
        if not _capture_and_cache_current_angles(hard_capture):
            return _fail()
        _hard_brush_clean_cache[port] = hard_capture

    if not ok(run_skill("approach_machine", "portafilter_cleaner", "soft_brush")):
        return _fail()
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "soft_brush")):
        return _fail()

    if soft_cached:
        _trace_step("CLEANING", f"soft brush cache HIT port={port} len={len(soft_cached)}")
        if len(soft_cached) != 6:
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[0])):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[1])):
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[2])):
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[3])):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[4])):
            return _fail()
        # run_skill("sync")
        if not ok(run_skill("gotoJ_deg", *soft_cached[5])):
            return _fail()
    else:
        _trace_step("CLEANING", f"soft brush cache MISS port={port}; recording live sequence")
        soft_capture: List[Tuple[float, ...]] = []
        if not ok(run_skill("moveEE_movJ", 0, 0, 50, 0, 0, 0)):
            return _fail()
        if not _capture_and_cache_current_angles(soft_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0, 0, -35, -2.5, 0, 0)):
            return _fail()
        if not _capture_and_cache_current_angles(soft_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", -2.5, 2.5, -5, 0, 0, 0)):
            return _fail()
        if not _capture_and_cache_current_angles(soft_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0.0, 0.0, 47.5, 0.0, 0.0, 0.0)):
            return _fail()
        if not _capture_and_cache_current_angles(soft_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0.0, 30.0, -15.5, 0.0, 0.0, 0.0)):
            return _fail()
        if not _capture_and_cache_current_angles(soft_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_soft'])):
            return _fail()
        if not _capture_and_cache_current_angles(soft_capture):
            return _fail()
        _soft_brush_clean_cache[port] = soft_capture

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    return True

def clean_portafilter_single(**params) -> bool:
    _trace_step("CLEANING", "clean_portafilter_single START")
    params["port"] = "port_2"
    return clean_portafilter(**params)

def clean_portafilter_double(**params) -> bool:
    _trace_step("CLEANING", "clean_portafilter_double START")
    params["port"] = "port_1"
    return clean_portafilter(**params)

"""
angled_cleaning.py

Parallel definitions prefixed with angled_; same behavior as cleaning above until customized.
"""

angled__hard_brush_clean_cache: Dict[str, Any] = {}
angled__soft_brush_clean_cache: Dict[str, Any] = {}

def angled_invalidate_cleaning_cache():
    _trace_step("CLEANING", "angled_invalidate_cleaning_cache START")
    angled__hard_brush_clean_cache.clear()
    angled__soft_brush_clean_cache.clear()

def angled_cleaning_is_valid_angles(angles: Any) -> bool:
    _trace_step("CLEANING", "angled_cleaning_is_valid_angles START")
    return bool(angles) and isinstance(angles, (tuple, list)) and len(angles) == 6

def angled_cleaning_capture_current_angles(cache_list: List[Tuple[float, ...]]) -> bool:
    _trace_step("CLEANING", "angled_cleaning_capture_current_angles START")
    angles = run_skill("current_angles")
    if not angled_cleaning_is_valid_angles(angles):
        return _fail()
    cache_list.append(tuple(angles))
    return True

def angled_clean_portafilter(**params) -> bool:
    """
    Keep approach/mount live, cache only the post-mount motion sequence.
    """
    _trace_step("CLEANING", "angled_clean_portafilter START")
    from oms_v1.sequences.espresso import angled__normalize_espresso_shot

    espresso_dict = params.get("espresso")
    shot_cfg = angled__normalize_espresso_shot(espresso_dict)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else DEFAULT_PORT)

    def ok(r):
        return r not in (False, None)

    hard_cached = angled__hard_brush_clean_cache.get(port)
    soft_cached = angled__soft_brush_clean_cache.get(port)

    if not ok(run_skill("gotoJ_deg", *CLEANING_POSES['pre_clean_home'])):
        return _fail()

    if not ok(run_skill("approach_machine", "portafilter_cleaner", "angled_hard_brush")):
        return _fail()
    if not ok(run_skill("gotoJ_deg", *CLEANING_PARAMS['hard_brush_adjust'])):
        return _fail()
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "angled_hard_brush")):
        return _fail()

    if hard_cached:
        _trace_step("CLEANING", f"hard brush cache HIT port={port} len={len(hard_cached)}")
        if len(hard_cached) != 4:
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[0])):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[1])):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[2])):
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *hard_cached[3])):
            return _fail()
    else:
        _trace_step("CLEANING", f"hard brush cache MISS port={port}; recording live sequence")
        hard_capture: List[Tuple[float, ...]] = []
        if not ok(run_skill("moveEE_movJ", 0, 0, 50, 0, 0, 0)):
            return _fail()
        if not angled_cleaning_capture_current_angles(hard_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0, 0, -37.5, -2.5, 0, 0)):
            return _fail()
        if not angled_cleaning_capture_current_angles(hard_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0, 0, -7.5, 0, 0, 0)):
            return _fail()
        if not angled_cleaning_capture_current_angles(hard_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_hard'])):
            return _fail()
        if not angled_cleaning_capture_current_angles(hard_capture):
            return _fail()
        angled__hard_brush_clean_cache[port] = hard_capture

    if not ok(run_skill("approach_machine", "portafilter_cleaner", "angled_soft_brush")):
        return _fail()
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "angled_soft_brush")):
        return _fail()

    if soft_cached:
        _trace_step("CLEANING", f"soft brush cache HIT port={port} len={len(soft_cached)}")
        if len(soft_cached) != 4:
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[0])):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[1])):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[2])):
            return _fail()
        if not ok(run_skill("sync")):
            return _fail()
        if not ok(run_skill("gotoJ_deg", *soft_cached[3])):
            return _fail()
    else:
        _trace_step("CLEANING", f"soft brush cache MISS port={port}; recording live sequence")
        soft_capture: List[Tuple[float, ...]] = []
        if not ok(run_skill("moveEE_movJ", 0, 0, 50, 0, 0, 0)):
            return _fail()
        if not angled_cleaning_capture_current_angles(soft_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0, 0, -37.5, -2.5, 0, 0)):
            return _fail()
        if not angled_cleaning_capture_current_angles(soft_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", 0, 0, -7.5, 0, 0, 0)):
            return _fail()
        if not angled_cleaning_capture_current_angles(soft_capture):
            return _fail()
        if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_soft'])):
            return _fail()
        if not angled_cleaning_capture_current_angles(soft_capture):
            return _fail()
        angled__soft_brush_clean_cache[port] = soft_capture

    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return _fail()

    return True

def angled_clean_portafilter_single(**params) -> bool:
    _trace_step("CLEANING", "angled_clean_portafilter_single START")
    params["port"] = "angled_portafilter_1"
    return angled_clean_portafilter(**params)

def angled_clean_portafilter_double(**params) -> bool:
    _trace_step("CLEANING", "angled_clean_portafilter_double START")
    params["port"] = "angled_portafilter_1"
    return angled_clean_portafilter(**params)


SEQUENCES = {
    'clean_portafilter': clean_portafilter,
    'invalidate_cleaning_cache': invalidate_cleaning_cache,
    'angled_clean_portafilter': angled_clean_portafilter,
    'angled_invalidate_cleaning_cache': angled_invalidate_cleaning_cache,
    'clean_portafilter_single': clean_portafilter_single,
    'angled_clean_portafilter_single': angled_clean_portafilter_single,
    'angled_clean_portafilter_double': angled_clean_portafilter_double,
}