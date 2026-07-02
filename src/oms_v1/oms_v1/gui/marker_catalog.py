"""
gui/marker_catalog.py

Lists machine markers and tool markers actually referenced by the production
sequence code in ``oms_v1/sequences/``. The lists are intentionally
hand-curated (not auto-discovered every load) so that the GUI never displays
markers the operator does not actually need to teach.

If a new marker is introduced in production code, add it here and document
which sequence(s) use it.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Tuple

from .trace import gui_log


_SEQUENCES_DIR = (
    Path(__file__).resolve().parent.parent / "sequences"
)


_MACHINE_MARKERS: Tuple[str, ...] = (
    "three_group_espresso",
    "espresso_grinder",
    "portafilter_cleaner",
    "left_steam_wand",
    "milk_frother_2",
)


_TOOL_MARKERS: Tuple[str, ...] = (
    "single_portafilter",
    "double_portafilter",
    "single_portafilter_angled",
    "double_portafilter_angled",
)


_MACHINE_POINTS: Dict[str, Tuple[str, ...]] = {
    "three_group_espresso": (
        "group_1",
        "group_2",
        "group_3",
        "hot_water",
        "portafilter_1",
        "portafilter_2",
        "portafilter_3",
        "angled_portafilter_1",
        "angled_portafilter_2",
    ),
    "espresso_grinder": (
        "grinder",
        "tamper",
    ),
    "portafilter_cleaner": (
        "hard_brush",
        "soft_brush",
        "angled_hard_brush",
        "angled_soft_brush",
    ),
    "left_steam_wand": (
        "deep_froth",
    ),
    "milk_frother_2": (
        "milk_frother_1",
    ),
}


def machine_markers() -> List[str]:
    return list(_MACHINE_MARKERS)


def tool_markers() -> List[str]:
    return list(_TOOL_MARKERS)


def machine_points(marker: str) -> List[str]:
    return list(_MACHINE_POINTS.get(marker, ()))


def discover_referenced_markers() -> Dict[str, List[str]]:
    """Best-effort scan of ``sequences/*.py`` for any marker strings that
    look like the second positional arg to ``approach_machine`` /
    ``mount_machine`` / ``move_to`` / ``get_machine_position``. Returned for
    informational display only -- the canonical list above is what the GUI
    actually offers.
    """
    pattern = re.compile(
        r'(?:approach_machine|mount_machine|move_to|get_machine_position)\s*"\s*,\s*"([\w]+)"',
    )
    found_machines: Dict[str, List[str]] = {}
    if not _SEQUENCES_DIR.exists():
        return found_machines
    for py in _SEQUENCES_DIR.glob("*.py"):
        try:
            text = py.read_text(encoding="utf-8")
        except Exception as exc:
            gui_log("GUI-MARKER", f"failed to read {py}: {exc}")
            continue
        for match in pattern.finditer(text):
            name = match.group(1)
            found_machines.setdefault(name, []).append(py.name)
    return found_machines
