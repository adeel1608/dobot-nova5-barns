"""
home.py

Defines the 'home' positioning routine using compass directions.
"""

import time
from oms_v1.params import HOME_ANGLES, Espresso_grinder_home, Espresso_home
from oms_v1.manipulate_node import run_skill


def home(position):
    """
    Move the robot to one of the predefined home orientations.
    
    Args:
        position (str): One of 'north', 'north_east', 'east',
                        'south_east', 'south', 'south_west',
                        'west', 'north_west'.
    """
    angles = HOME_ANGLES.get(position)
    if not angles:
        print(f"[ERROR] unknown home position: {position!r}")
        return

    print(f"--- refreshing and moving to {position} home ---")
    run_skill("refresh_position")
    time.sleep(2.0)
    run_skill("gotoJ_deg", *angles)
    print(f"--- arrived at {position} home ---")

def get_machine_position():
    """Machine position updates here (when starting from home position)."""
    run_skill("gotoJ_deg", Espresso_home)
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    for i in range(5):
        time.sleep(1.1)
        run_skill("move_to", "portafilter_cleaner", 0.12)
    run_skill("get_machine_position", "portafilter_cleaner")
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("gotoJ_deg", 0.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "espresso_grinder", 0.12)
    run_skill("get_machine_position", "espresso_grinder")
    run_skill("gotoJ_deg", 7.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379)
    run_skill("moveJ_deg", 35, 0, 0, 0, 0, 0)
    for i in range(5):
        time.sleep(1.0)
        run_skill("move_to", "three_group_espresso", 0.12)
    run_skill("get_machine_position", "three_group_espresso")
    run_skill("gotoJ_deg", Espresso_home)

# Register for CLI discovery
SEQUENCES = {
    'home': home,
    'get_machine_position' : get_machine_position,
}
