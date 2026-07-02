"""
gui/sequence_registry.py

Reuse the same sequence merging that ``oms_v1/app.py`` performs so the GUI
sees the exact same set of callable actions production sees.

The registry is loaded lazily; if ROS2 is not sourced and the sequence
modules cannot be imported, the registry returns an empty dict and the GUI
surfaces a clear "no sequences available" message instead of crashing.
"""

from __future__ import annotations

import importlib
from typing import Callable, Dict, List, Optional, Tuple

from .trace import gui_log, gui_log_error


_SEQ_MODULES: Tuple[str, ...] = (
    "oms_v1.sequences.home",
    "oms_v1.sequences.espresso",
    "oms_v1.sequences.cleaning",
    "oms_v1.sequences.test",
    "oms_v1.sequences.paper_cups",
    "oms_v1.sequences.plastic_cups",
    "oms_v1.sequences.slush",
    "oms_v1.sequences.milk_frothing",
    "oms_v1.sequences.computer_vision",
)


_cache: Optional[Dict[str, Callable]] = None
_load_errors: List[Tuple[str, str]] = []

# Sequence modules deliberately publish a few cache reset helpers in SEQUENCES
# so production code can call them. They are not useful point-editing or
# operator-run entries in the GUI, so keep that policy explicit here instead
# of hiding functions by accident through the point catalog.
_GUI_EXCLUDED_SEQUENCES = {
    "invalidate_port_cache",
    "angled_invalidate_port_cache",
    "invalidate_cleaning_cache",
    "angled_invalidate_cleaning_cache",
    "invalidate_milk_frothing_cache",
    "invalidate_plastic_cup_cache",
}

_GUI_TEST_PLAY_BLOCKED_SEQUENCES = {
    # Dedicated robot-control buttons own these flows and apply their own
    # busy-state/logging guards. Do not expose them through generic point tests.
    "enable_robot",
    "disable_robot",
    "toggle_drag_mode",
    "open_gripper",
    "close_gripper",
    # These restart deployments or run broad debug routines rather than a
    # point-teaching sequence.
    "reset_robot1",
    "reset_robot2",
    "test",
    "test_1",
}


def load_sequences(force: bool = False) -> Dict[str, Callable]:
    """Import all sequence modules and merge their SEQUENCES dicts."""
    global _cache, _load_errors
    if _cache is not None and not force:
        return _cache
    merged: Dict[str, Callable] = {}
    errors: List[Tuple[str, str]] = []
    for mod_name in _SEQ_MODULES:
        try:
            module = importlib.import_module(mod_name)
        except Exception as exc:
            gui_log_error("GUI-REG", f"failed to import {mod_name}", exc)
            errors.append((mod_name, f"{type(exc).__name__}: {exc}"))
            continue
        seq = getattr(module, "SEQUENCES", None)
        if not isinstance(seq, dict):
            errors.append((mod_name, "module has no SEQUENCES dict"))
            continue
        merged.update(seq)
    gui_log(
        "GUI-REG",
        f"loaded {len(merged)} sequences from {len(_SEQ_MODULES)} modules; "
        f"{len(errors)} errors",
    )
    _cache = merged
    _load_errors = errors
    return merged


def load_errors() -> List[Tuple[str, str]]:
    """Return any per-module import failures from the last load_sequences()."""
    return list(_load_errors)


def public_sequence_names(force: bool = False) -> List[str]:
    """Return sequence names intended to be visible in the operator GUI."""
    sequences = load_sequences(force=force)
    return sorted(
        name
        for name in sequences.keys()
        if not name.startswith("_") and name not in _GUI_EXCLUDED_SEQUENCES
    )


def can_test_play_sequence(name: Optional[str]) -> bool:
    """Return whether the generic GUI test-play button may execute a sequence."""
    return bool(name) and name not in _GUI_TEST_PLAY_BLOCKED_SEQUENCES


def test_play_block_reason(name: Optional[str]) -> str:
    """Human-readable reason for disabling generic test-play execution."""
    if not name:
        return "No sequence selected."
    if name in _GUI_TEST_PLAY_BLOCKED_SEQUENCES:
        return (
            f"{name} is available in the catalog but is blocked from generic "
            "test play. Use its dedicated control flow or run it from the "
            "backend intentionally."
        )
    return ""


def reset() -> None:
    """Drop the cache so the next load reflects re-imported sequence modules."""
    global _cache
    _cache = None
