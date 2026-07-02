"""
gui/point_catalog.py

Hand-curated metadata describing which params.py keys are static joint
points (used through ``gotoJ`` / ``gotoJ_deg`` / ``run_skill("gotoJ_deg",
...)``) for each sequence function -- and which gripper constants are
considered teachable.

The catalog deliberately excludes:
    - any value populated at runtime (``current_angles``, ``current_pose``,
      cached / learned poses)
    - move offsets used purely as relative deltas (``moveEE`` / ``moveJ_deg``
      offsets) unless those offsets are explicitly intended as teachable
      production parameters
    - speeds, delays, thresholds, and other non-pose tunables

Each catalog entry has the shape:

    {
        "summary": str,
        "variants": [(label, options)],   # 0 or more parameter axes
        "editable_points": [
            EditablePoint(name, description, params_path, kind),
            ...
        ],
        "build_call_params": Callable[[Mapping], dict],
    }

``params_path`` is a tuple ``(top_level_name, *parts)`` understood by
``params_editor.update_value``. ``kind`` is either ``"joint"`` (a 6-tuple of
joint angles) or ``"gripper"`` (an integer 0..255).

Variant options are resolved lazily against the live ``params.py`` so adding
a new cup size or port to params.py automatically appears in the GUI.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from . import params_editor
from .trace import gui_log


@dataclass(frozen=True)
class EditablePoint:
    name: str
    description: str
    params_path: Tuple[Any, ...]
    kind: str = "joint"


@dataclass(frozen=True)
class FunctionMetadata:
    name: str
    summary: str
    variant_axes: Tuple[Tuple[str, str], ...]
    points_factory: Callable[[Dict[str, Any]], List[EditablePoint]]
    call_params_factory: Callable[[Dict[str, Any]], Dict[str, Any]]
    options_factory: Optional[Callable[[str], List[str]]] = None


def _options_from_keys(top_name: str) -> List[str]:
    data = params_editor.get_top_level_dict(top_name)
    if not isinstance(data, dict):
        return []
    return [str(k) for k in data.keys()]


def _grab_paper_cup_options(axis: str) -> List[str]:
    if axis != "size":
        return []
    return [k for k in _options_from_keys("GRAB_PAPER_CUP_PARAMS") if not k.startswith("_")]


def _place_paper_cup_options(axis: str) -> List[str]:
    if axis != "stage":
        return []
    return [k for k in _options_from_keys("PLACE_PAPER_CUP_PARAMS") if not k.startswith("_")]


def _grab_paper_cup_points(choice: Dict[str, Any]) -> List[EditablePoint]:
    size = choice["size"]
    twist_key = {
        "7oz": "twist_7oz",
        "9oz": "twist_9oz",
        "12oz": "twist_12oz",
    }.get(size)
    points: List[EditablePoint] = []
    if twist_key:
        points.append(
            EditablePoint(
                name=f"navigation/{twist_key}",
                description=f"Twist navigation pose used immediately before grabbing a {size} paper cup.",
                params_path=("PAPER_CUPS_NAVIGATION_PARAMS", twist_key),
                kind="joint",
            )
        )
    points.append(
        EditablePoint(
            name="navigation/dispenser_area",
            description="Approach pose for the paper cup dispenser area (shared across sizes).",
            params_path=("PAPER_CUPS_NAVIGATION_PARAMS", "dispenser_area"),
            kind="joint",
        )
    )
    points.append(
        EditablePoint(
            name=f"grip_width:{size}",
            description=f"Gripper width used when closing on a {size} paper cup at the dispenser.",
            params_path=("GRAB_PAPER_CUP_PARAMS", size, "grip_width"),
            kind="gripper",
        )
    )
    return points


def _grab_paper_cup_call(choice: Dict[str, Any]) -> Dict[str, Any]:
    size = choice["size"]
    code = {"7oz": "cup_H7", "9oz": "cup_H9", "12oz": "cup_H12"}.get(size, "cup_H7")
    return {"ingredients": {"cups": {code: 1.0}}}


def _place_paper_cup_points(choice: Dict[str, Any]) -> List[EditablePoint]:
    stage = choice["stage"]
    return [
        EditablePoint(
            name=f"{stage}/pose",
            description=f"Final placement pose for {stage} on the staging shelf.",
            params_path=("PLACE_PAPER_CUP_PARAMS", stage, "pose"),
            kind="joint",
        ),
        EditablePoint(
            name=f"{stage}/stage_home",
            description=f"Stage-home pose for {stage} (shared template; may be safe to skip).",
            params_path=("PLACE_PAPER_CUP_PARAMS", stage, "stage_home"),
            kind="joint",
        ),
        EditablePoint(
            name="navigation/intermediate",
            description="Intermediate pose used between machine and staging shelf.",
            params_path=("PAPER_CUPS_NAVIGATION_PARAMS", "intermediate"),
            kind="joint",
        ),
    ]


def _place_paper_cup_call(choice: Dict[str, Any]) -> Dict[str, Any]:
    stage = choice["stage"]
    cup_position = int(stage.split("_")[1])
    return {"position": {"cup_position": float(cup_position)}}


def _home_options(axis: str) -> List[str]:
    if axis != "position":
        return []
    keys = _options_from_keys("HOME_ANGLES")
    return keys or [
        "north", "north_east", "east", "south_east",
        "south", "south_west", "west", "north_west",
    ]


def _home_points(choice: Dict[str, Any]) -> List[EditablePoint]:
    pos = choice["position"]
    return [
        EditablePoint(
            name=f"HOME_ANGLES[{pos!r}]",
            description=f"Joint pose for the {pos} compass home.",
            params_path=("HOME_ANGLES", pos),
            kind="joint",
        ),
    ]


def _home_call(choice: Dict[str, Any]) -> Dict[str, Any]:
    return {"position": choice["position"]}


def _pull_espresso_options(axis: str) -> List[str]:
    if axis != "port":
        return []
    return [
        k
        for k in _options_from_keys("PULL_ESPRESSO_PARAMS")
        if k.startswith("port_") or k.startswith("angled_")
    ]


def _pull_espresso_points(choice: Dict[str, Any]) -> List[EditablePoint]:
    port = choice["port"]
    return [
        EditablePoint(
            name=f"{port}/home",
            description=f"Home pose for {port} pull-espresso flow.",
            params_path=("PULL_ESPRESSO_PARAMS", port, "home"),
            kind="joint",
        ),
        EditablePoint(
            name=f"{port}/move_back",
            description=f"Move-back pose for {port} after machine interaction.",
            params_path=("PULL_ESPRESSO_PARAMS", port, "move_back"),
            kind="joint",
        ),
        EditablePoint(
            name="ESPRESSO_HOME",
            description="Top-level espresso home (shared across all ports).",
            params_path=("ESPRESSO_HOME",),
            kind="joint",
        ),
    ]


def _pull_espresso_call(choice: Dict[str, Any]) -> Dict[str, Any]:
    return {"port": choice["port"]}


def _milk_options(axis: str) -> List[str]:
    if axis != "stage":
        return []
    pouring = params_editor.get_value("MILK_FROTHING_PARAMS", ("pouring",))
    if isinstance(pouring, dict):
        return [k for k in pouring.keys() if k.startswith("stage")]
    return ["stage1", "stage2", "stage3", "stage4"]


def _milk_points(choice: Dict[str, Any]) -> List[EditablePoint]:
    stage = choice["stage"]
    return [
        EditablePoint(
            name=f"pouring/{stage}/position",
            description=f"Milk pour position for {stage}.",
            params_path=("MILK_FROTHING_PARAMS", "pouring", stage, "position"),
            kind="joint",
        ),
        EditablePoint(
            name=f"pouring/{stage}/adjust1",
            description=f"Pour-angle adjustment pose for {stage}.",
            params_path=("MILK_FROTHING_PARAMS", "pouring", stage, "adjust1"),
            kind="joint",
        ),
        EditablePoint(
            name="mounting/prep",
            description="Frother mounting prep pose (shared across stages).",
            params_path=("MILK_FROTHING_PARAMS", "mounting", "prep"),
            kind="joint",
        ),
    ]


def _milk_call(choice: Dict[str, Any]) -> Dict[str, Any]:
    return {}


def _slush_options(axis: str) -> List[str]:
    if axis != "dispenser":
        return []
    keys = _options_from_keys("SLUSH_PARAMS")
    return [k for k in keys if k.startswith("dispenser_")]


def _slush_points(choice: Dict[str, Any]) -> List[EditablePoint]:
    disp = choice["dispenser"]
    return [
        EditablePoint(
            name=f"{disp}/dispense",
            description=f"Dispense pose for slush {disp}.",
            params_path=("SLUSH_PARAMS", disp, "dispense"),
            kind="joint",
        ),
        EditablePoint(
            name=f"{disp}/retreat",
            description=f"Retreat pose for slush {disp}.",
            params_path=("SLUSH_PARAMS", disp, "retreat"),
            kind="joint",
        ),
        EditablePoint(
            name="navigation/intermediate",
            description="Intermediate navigation pose for slush flows.",
            params_path=("SLUSH_PARAMS", "navigation", "intermediate"),
            kind="joint",
        ),
        EditablePoint(
            name="navigation/slush_area",
            description="Main slush area pose.",
            params_path=("SLUSH_PARAMS", "navigation", "slush_area"),
            kind="joint",
        ),
    ]


def _slush_call(choice: Dict[str, Any]) -> Dict[str, Any]:
    suffix = choice["dispenser"].split("_")[-1]
    return {"dispenser": suffix}


def _no_variant_points_paper_cup_station(_choice: Dict[str, Any]) -> List[EditablePoint]:
    return [
        EditablePoint(
            name="staging/place_1",
            description="Stage 1 placement pose at the paper-cup staging shelf.",
            params_path=("PAPER_CUPS_STATION_PARAMS", "staging", "place_1"),
            kind="joint",
        ),
        EditablePoint(
            name="staging/place_2",
            description="Stage 2 placement pose at the paper-cup staging shelf.",
            params_path=("PAPER_CUPS_STATION_PARAMS", "staging", "place_2"),
            kind="joint",
        ),
        EditablePoint(
            name="staging/place_3",
            description="Stage 3 placement pose at the paper-cup staging shelf.",
            params_path=("PAPER_CUPS_STATION_PARAMS", "staging", "place_3"),
            kind="joint",
        ),
        EditablePoint(
            name="staging/place_4",
            description="Stage 4 placement pose at the paper-cup staging shelf.",
            params_path=("PAPER_CUPS_STATION_PARAMS", "staging", "place_4"),
            kind="joint",
        ),
    ]


def _no_call(_choice: Dict[str, Any]) -> Dict[str, Any]:
    return {}


_CATALOG: Dict[str, FunctionMetadata] = {
    "home": FunctionMetadata(
        name="home",
        summary="Move robot to a predefined compass home position (north, east, ...).",
        variant_axes=(("position", "Home position"),),
        points_factory=_home_points,
        call_params_factory=_home_call,
        options_factory=_home_options,
    ),
    "grab_paper_cup": FunctionMetadata(
        name="grab_paper_cup",
        summary="Grab a paper cup from the dispenser for the selected size.",
        variant_axes=(("size", "Paper cup size"),),
        points_factory=_grab_paper_cup_points,
        call_params_factory=_grab_paper_cup_call,
        options_factory=_grab_paper_cup_options,
    ),
    "place_paper_cup": FunctionMetadata(
        name="place_paper_cup",
        summary="Place the held paper cup on the chosen staging shelf stage.",
        variant_axes=(("stage", "Staging stage"),),
        points_factory=_place_paper_cup_points,
        call_params_factory=_place_paper_cup_call,
        options_factory=_place_paper_cup_options,
    ),
    "place_paper_cup_station": FunctionMetadata(
        name="place_paper_cup_station",
        summary="Update placement poses on the paper-cup staging shelf.",
        variant_axes=(),
        points_factory=_no_variant_points_paper_cup_station,
        call_params_factory=_no_call,
        options_factory=None,
    ),
    "unmount": FunctionMetadata(
        name="unmount",
        summary="Update home / move_back pose around the espresso group for the selected port.",
        variant_axes=(("port", "Espresso port"),),
        points_factory=_pull_espresso_points,
        call_params_factory=_pull_espresso_call,
        options_factory=_pull_espresso_options,
    ),
    "mount": FunctionMetadata(
        name="mount",
        summary="Update home / move_back pose around the espresso group for the selected port.",
        variant_axes=(("port", "Espresso port"),),
        points_factory=_pull_espresso_points,
        call_params_factory=_pull_espresso_call,
        options_factory=_pull_espresso_options,
    ),
    "pour_milk_cup_station": FunctionMetadata(
        name="pour_milk_cup_station",
        summary="Update milk-frothing pour positions for the selected stage.",
        variant_axes=(("stage", "Milk pouring stage"),),
        points_factory=_milk_points,
        call_params_factory=_milk_call,
        options_factory=_milk_options,
    ),
    "get_slush": FunctionMetadata(
        name="get_slush",
        summary="Update navigation + dispense poses for the selected slush dispenser.",
        variant_axes=(("dispenser", "Slush dispenser"),),
        points_factory=_slush_points,
        call_params_factory=_slush_call,
        options_factory=_slush_options,
    ),
}


def known_functions() -> List[str]:
    return list(_CATALOG.keys())


def metadata(name: str) -> Optional[FunctionMetadata]:
    return _CATALOG.get(name)


def variant_options(name: str, axis: str) -> List[str]:
    meta = metadata(name)
    if meta is None or meta.options_factory is None:
        return []
    try:
        return meta.options_factory(axis)
    except Exception as exc:
        gui_log("GUI-CAT", f"options_factory({name},{axis}) failed: {exc}")
        return []


def points_for(name: str, choice: Dict[str, Any]) -> List[EditablePoint]:
    meta = metadata(name)
    if meta is None:
        return []
    try:
        return meta.points_factory(choice)
    except Exception as exc:
        gui_log("GUI-CAT", f"points_factory({name}) failed: {exc}")
        return []


def call_params_for(name: str, choice: Dict[str, Any]) -> Dict[str, Any]:
    meta = metadata(name)
    if meta is None:
        return {}
    try:
        return meta.call_params_factory(choice)
    except Exception as exc:
        gui_log("GUI-CAT", f"call_params_factory({name}) failed: {exc}")
        return {}
