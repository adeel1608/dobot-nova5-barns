# placer
"""
get_ice.py

Defines the ice handling sequence for coffee service automation.
This module provides simplified functions for moving to ice positions
in the BARNS coffee automation system.
"""

import time
from oms_v1.manipulate_node import run_skill

# Predefined joint targets for ice operations (degrees)
# Speed/acc hints from your CLI examples: SpeedJ=100, AccJ=100 (handled by controller defaults)
ice_home = (
    112.3456,   # j1
    -18.2169,   # j2
    -139.2699,  # j3
    -51.3612,   # j4
    -76.2103,   # j5
    -0.2569,    # j6
)

approach_ice = (
    109.3439,   # j1
    -103.6352,  # j2
    -88.2536,   # j3
    11.9293,    # j4
    -4.3764,    # j5
    0.7191,     # j6
)

grab_ice = (
    71.1132,    # j1
    -132.4976,  # j2
    -72.8472,   # j3
    20.6167,    # j4
    -76.2114,   # j5
    -0.3139,    # j6
)


def go_to_ice() -> bool:
    """
    Full ice grabbing workflow:
    1) Move to ice_home
    2) Move to approach_ice
    3) Move to grab_ice and grip
    """
    try:
        print("🧊 Starting full go_to_ice sequence")
        print("=" * 40)

        # Step 1: Move to ice_home
        print("🏠 Step 1/3: Moving to ice_home...")
        home_result = run_skill("gotoJ_deg", *ice_home)
        if home_result is False:
            print("[ERROR] Failed to move to ice_home")
            return False
        print("   ✅ Successfully moved to ice_home")

        # Step 2: Move to approach_ice
        print("📍 Step 2/3: Moving to approach_ice...")
        approach_result = run_skill("gotoJ_deg", *approach_ice)
        if approach_result is False:
            print("[ERROR] Failed to move to approach_ice")
            return False
        print("   ✅ Successfully moved to approach_ice")

        # Step 3: Move to grab_ice and grip
        print("🤏 Step 3/3: Moving to grab_ice and gripping...")
        grab_result = run_skill("gotoJ_deg", *grab_ice)
        if grab_result is False:
            print("[ERROR] Failed to move to grab_ice")
            return False

        run_skill("sync")
        grip_result = run_skill("set_gripper_position", 100, 80)
        if grip_result is False:
            print("[ERROR] Failed to grip ice")
            return False
        print("   ✅ Successfully gripped ice")

        print("=" * 40)
        print("✅ GO_TO_ICE SEQUENCE COMPLETED SUCCESSFULLY")
        return True

    except Exception as e:
        print(f"[ERROR] Unexpected error during go_to_ice: {e}")
        return False


def go_home_ice() -> bool:
    """
    Return workflow:
    1) Move to approach_ice
    2) Move to ice_home
    (No gripping actions)
    """
    try:
        print("🧊 Starting go_home_ice sequence")
        print("=" * 40)

        # Step 1: Move to approach_ice
        print("📍 Step 1/2: Moving to approach_ice...")
        approach_result = run_skill("gotoJ_deg", *approach_ice)
        if approach_result is False:
            print("[ERROR] Failed to move to approach_ice")
            return False
        print("   ✅ Successfully moved to approach_ice")

        # Step 2: Move to ice_home
        print("🏠 Step 2/2: Moving to ice_home...")
        home_result = run_skill("gotoJ_deg", *ice_home)
        if home_result is False:
            print("[ERROR] Failed to move to ice_home")
            return False
        print("   ✅ Successfully moved to ice_home")

        print("=" * 40)
        print("✅ GO_HOME_ICE SEQUENCE COMPLETED SUCCESSFULLY")
        return True

    except Exception as e:
        print(f"[ERROR] Unexpected error during go_home_ice: {e}")
        return False


# Register functions for CLI discovery
SEQUENCES = {
    "go_to_ice": go_to_ice,
    "go_home_ice": go_home_ice,
}
