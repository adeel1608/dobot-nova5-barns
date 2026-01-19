"""
test.py
"""

import time
from oms_v1.manipulate_node import run_skill
from oms_v1.params import PULL_ESPRESSO_PARAMS, POUR_PARAMS, HOME_ANGLES, GRAB_CUP_PARAMS, PLACE_CUP_PARAMS, Espresso_home, Espresso_grinder_home


def test(port):
    run_skill("gotoJ_deg", *Espresso_home) #Espresso home
    run_skill("gotoJ_deg", *Espresso_grinder_home) #Espresso grinder home

# Register for CLI discovery
SEQUENCES = {
    'test': test,
}
