"""
test.py
"""

import time
from oms_v1.manipulate_node import run_skill
from oms_v1.params import PULL_ESPRESSO_PARAMS, POUR_PARAMS, HOME_ANGLES, GRAB_CUP_PARAMS, PLACE_CUP_PARAMS


def test(**params):
    """Machine position updates here (when starting from home position)."""
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso home 
    run_skill("gotoJ_deg", -62.837723, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    for i in range(5):
        time.sleep(1.0)
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
    run_skill("gotoJ_deg", 42.427441,13.883821,-133.648376,-81.024788,-49.533218,13.894379) #Espresso homeprint("Press ENTER to proceed...")
    run_skill("refresh_position")
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)
    run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
    

# Register for CLI discovery
SEQUENCES = {
    'test': test,
}
