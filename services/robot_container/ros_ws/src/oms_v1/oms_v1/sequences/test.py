"""
test.py
"""

import time
from oms_v1.manipulate_node import run_skill
from oms_v1.params import PULL_ESPRESSO_PARAMS, POUR_PARAMS, HOME_ANGLES, GRAB_CUP_PARAMS, PLACE_CUP_PARAMS


def test():
    print("Press ENTER to proceed...")
    input()
    run_skill("refresh_position")
    run_skill("gotoJ_deg", 88.718765,-27.610367,-135.543320,-14.489676,-5.211227,0.032554)
    run_skill("gotoJ_deg", 57.162277, -2.957932, -128.257645, -89.085014, -79.229942, 9.602360)
    run_skill("moveJ_deg", -90, 0, 0, 0, 0, 0)
    

# Register for CLI discovery
SEQUENCES = {
    'test': test,
}
