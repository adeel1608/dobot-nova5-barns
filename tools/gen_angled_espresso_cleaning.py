#!/usr/bin/env python3
"""One-off: build angled_* espresso/cleaning blocks for testing_v1.py.

Do not re-run after a successful insert without removing the existing
angled_espresso / angled_cleaning sections from testing_v1.py first.
"""
from __future__ import annotations

import re as regex
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "src/pickn_place/pickn_place/testing_v1.py"


def replace_words(text: str, pairs: list[tuple[str, str]]) -> str:
    pairs = sorted(pairs, key=lambda x: len(x[0]), reverse=True)
    for old, new in pairs:
        text = regex.sub(r"\b" + regex.escape(old) + r"\b", new, text)
    return text


def main() -> None:
    lines = PATH.read_text().splitlines(keepends=True)
    s = "".join(lines)

    # Espresso body: from "# Global variables to store captured" through double_return_cleaned end
    m_esp = regex.search(
        r"(# Global variables to store captured positions during unmount sequence\n"
        r".*?^def double_return_cleaned_espresso_pitcher\(\*\*params\).*?\n    return return_cleaned_espresso_pitcher\(\*\*params\)\n)",
        s,
        regex.MULTILINE | regex.DOTALL,
    )
    if not m_esp:
        raise SystemExit("Could not find espresso block")
    esp_src = m_esp.group(1)

    esp_pairs = [
        ("double_return_cleaned_espresso_pitcher", "angled_double_return_cleaned_espresso_pitcher"),
        ("single_return_cleaned_espresso_pitcher", "angled_single_return_cleaned_espresso_pitcher"),
        ("double_return_espresso_pitcher", "angled_double_return_espresso_pitcher"),
        ("single_return_espresso_pitcher", "angled_single_return_espresso_pitcher"),
        ("double_pour_espresso_pitcher_cup_station", "angled_double_pour_espresso_pitcher_cup_station"),
        ("single_pour_espresso_pitcher_cup_station", "angled_single_pour_espresso_pitcher_cup_station"),
        ("double_pick_espresso_pitcher", "angled_double_pick_espresso_pitcher"),
        ("single_pick_espresso_pitcher", "angled_single_pick_espresso_pitcher"),
        ("double_grab_espresso_pitcher", "angled_double_grab_espresso_pitcher"),
        ("single_grab_espresso_pitcher", "angled_single_grab_espresso_pitcher"),
        ("return_cleaned_espresso_pitcher", "angled_return_cleaned_espresso_pitcher"),
        ("return_espresso_pitcher", "angled_return_espresso_pitcher"),
        ("pour_espresso_pitcher_cup_station", "angled_pour_espresso_pitcher_cup_station"),
        ("with_hot_water", "angled_with_hot_water"),
        ("get_hot_water", "angled_get_hot_water"),
        ("pick_espresso_pitcher", "angled_pick_espresso_pitcher"),
        ("grab_espresso_pitcher", "angled_grab_espresso_pitcher"),
        ("_normalize_espresso_shot", "angled__normalize_espresso_shot"),
        ("invalidate_port_cache", "angled_invalidate_port_cache"),
        ("_tool_pick_pose_cache", "angled__tool_pick_pose_cache"),
        ("_pitcher_return_cache", "angled__pitcher_return_cache"),
        ("_pitcher_pick_cache", "angled__pitcher_pick_cache"),
        ("_pitcher_clean_cache", "angled__pitcher_clean_cache"),
        ("_mount_runtime_cache", "angled__mount_runtime_cache"),
        ("_port_angle_cache", "angled__port_angle_cache"),
        ("mount_espresso_port", "angled_mount_espresso_port"),
        ("below_espresso_port", "angled_below_espresso_port"),
        ("mount_espresso_pose", "angled_mount_espresso_pose"),
        ("approach_pitcher", "angled_approach_pitcher"),
        ("pick_pitcher", "angled_pick_pitcher"),
        ("unmount_double", "angled_unmount_double"),
        ("unmount_single", "angled_unmount_single"),
        ("mount_double", "angled_mount_double"),
        ("mount_single", "angled_mount_single"),
        ("double_tamper", "angled_double_tamper"),
        ("single_tamper", "angled_single_tamper"),
        ("unmount", "angled_unmount"),
        ("mount", "angled_mount"),
        ("tamper", "angled_tamper"),
        ("grinder", "angled_grinder"),
        ("_is_valid_angles", "angled__is_valid_angles"),
    ]

    esp_block = replace_words(esp_src, esp_pairs)
    esp_header = (
        '\n"""\nangled_espresso.py\n\n'
        "Parallel definitions prefixed with angled_; same behavior as the espresso section above until customized.\n"
        '"""\n\n'
    )
    esp_out = esp_header + esp_block

    # Cleaning: invalidate_cleaning_cache through clean_portafilter
    m_cln = regex.search(
        r"(^def invalidate_cleaning_cache\(\):.*?^    return True\n)(?=\s*\n\"\"\"\nmilk_frothing)",
        s,
        regex.MULTILINE | regex.DOTALL,
    )
    if not m_cln:
        raise SystemExit("Could not find cleaning block")
    cln_src = m_cln.group(1)

    cln_pairs = [
        ("clean_portafilter", "angled_clean_portafilter"),
        ("invalidate_cleaning_cache", "angled_invalidate_cleaning_cache"),
        ("_hard_brush_clean_cache", "angled__hard_brush_clean_cache"),
        ("_soft_brush_clean_cache", "angled__soft_brush_clean_cache"),
        ("_capture_current_angles", "angled_cleaning_capture_current_angles"),
        ("_is_valid_angles", "angled_cleaning_is_valid_angles"),
        ("_normalize_espresso_shot", "angled__normalize_espresso_shot"),
    ]
    cln_block = replace_words(cln_src, cln_pairs)
    cln_header = (
        '\n"""\nangled_cleaning.py\n\n'
        "Parallel definitions prefixed with angled_; same behavior as cleaning above until customized.\n"
        '"""\n\n'
        "angled__hard_brush_clean_cache: Dict[str, Any] = {}\n"
        "angled__soft_brush_clean_cache: Dict[str, Any] = {}\n\n"
    )
    cln_out = cln_header + cln_block

    # Insert espresso angled block after double_return_cleaned (before cleaning.py docstring)
    anchor = 'def double_return_cleaned_espresso_pitcher(**params) -> bool:\n    params["port"] = "port_1"\n    return return_cleaned_espresso_pitcher(**params)\n\n"""\ncleaning.py'
    if anchor not in s:
        raise SystemExit("Anchor for espresso insert not found")
    s = s.replace(
        anchor,
        'def double_return_cleaned_espresso_pitcher(**params) -> bool:\n    params["port"] = "port_1"\n    return return_cleaned_espresso_pitcher(**params)\n'
        + esp_out
        + '\n"""\ncleaning.py',
    )

    # Insert cleaning angled after clean_portafilter return True, before milk_frothing docstring
    anchor2 = (
        "    if not ok(run_skill(\"gotoJ_deg\", *ESPRESSO_GRINDER_HOME)):\n"
        "        return False\n\n"
        "    return True\n \n\"\"\"\nmilk_frothing.py"
    )
    if anchor2 not in s:
        anchor2 = (
            "    if not ok(run_skill(\"gotoJ_deg\", *ESPRESSO_GRINDER_HOME)):\n"
            "        return False\n\n"
            "    return True\n\n\"\"\"\nmilk_frothing.py"
        )
    if anchor2 not in s:
        raise SystemExit("Anchor for cleaning insert not found")
    s = s.replace(
        anchor2,
        "    if not ok(run_skill(\"gotoJ_deg\", *ESPRESSO_GRINDER_HOME)):\n"
        "        return False\n\n"
        "    return True\n"
        + cln_out
        + '\n"""\nmilk_frothing.py',
    )

    PATH.write_text(s)
    print("Updated", PATH)


if __name__ == "__main__":
    main()