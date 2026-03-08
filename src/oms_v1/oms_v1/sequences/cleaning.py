"""
cleaning.py

Defines the cleaning routine for machine ports using parameterized configurations.
This module provides comprehensive portafilter cleaning functionality for the BARNS
coffee automation system, including hard brush and soft brush cleaning sequences
with precise positioning and error handling.
"""

import time
from typing import Dict, Any, Optional
from oms_v1.manipulate_node import run_skill
from oms_v1.params import (
    ESPRESSO_GRINDER_HOME, CLEANING_PARAMS, DEFAULT_PORT,
    DELAY_VERY_SHORT
)


def clean_portafilter(**params) -> bool:
    """
    Very simple cleaning flow:
      1) unmount
      2) grinder home
      3) hard brush: approach → adjust → mount → motion1 → motion2 → retreat_hard
      4) soft brush: approach → mount → motion1 → motion2 → retreat_soft
      5) grinder home
    """
    # Import here to avoid circular import with espresso.py
    from oms_v1.sequences.espresso import _normalize_espresso_shot
    
    # Normalize from espresso shot if provided
    # New format: {'espresso': {'espresso_shot_double': 2.0}}
    espresso_dict = params.get("espresso")
    shot_cfg = _normalize_espresso_shot(espresso_dict)

    # Extract and validate port parameter (derived from shot when not explicitly provided)
    port = params.get("port") or (shot_cfg.get("port") if shot_cfg else DEFAULT_PORT)

    def ok(r):  # minimal check: treat False/None as failure
        return r not in (False, None)

    # 2) Go to cleaning station home
    if not ok(run_skill("gotoJ_deg", -35.223076,-2.939468,-128.314575,-47.896400,-73.999352,1.973845)):
        return False

    # 3) Hard brush
    if not ok(run_skill("approach_machine", "portafilter_cleaner", "hard_brush")):
        return False
    if not ok(run_skill("gotoJ_deg", *CLEANING_PARAMS['hard_brush_adjust'])):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "hard_brush")):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", 0,0,50,0,0,0)):
        return False
    if not ok(run_skill("moveEE_movJ", 0,0,-35,-2.5,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", *CLEANING_PARAMS['retreat_hard'])):
        return False

    # 4) Soft brush
    if not ok(run_skill("approach_machine", "portafilter_cleaner", "soft_brush")):
        return False
    if not ok(run_skill("mount_machine", "portafilter_cleaner", "soft_brush")):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", 0,0,50,0,0,0)):
        return False
    if not ok(run_skill("moveEE_movJ", 0,0,-55,0,0,0)):
        return False
    if not ok(run_skill("moveEE_movJ", 0,10,0,0,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", 10,0,0,0,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", -20,0,0,0,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE_movJ", 0,-10,10,-2.5,0,0)):
        return False
    # time.sleep(DELAY_VERY_SHORT)
    if not ok(run_skill("moveEE", *CLEANING_PARAMS['retreat_soft'])):
        return False

    # 5) Return to cleaning station home
    if not ok(run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)):
        return False

    return True
    
# Register functions for CLI discovery and external access
SEQUENCES = {
    'clean_portafilter': clean_portafilter,
}
