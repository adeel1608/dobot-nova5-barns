"""
cleaning.py

Defines the cleaning routine for machine ports using parameterized configurations.
"""

import time
from oms_v1.params import Espresso_grinder_home, Espresso_home
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.espresso import unmount, mount

def clean(port):
    """
    Perform the cleaning sequence on the specified port.

    Args:
        port (str): 'port_1', 'port_2', or 'port_3'.
    """
    unmount(port)  # Unmount the portafilter if necessary
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    run_skill("approach_machine", "portafilter_cleaner", "hard_brush", True)
    run_skill("moveEE", -88, 0, 0, 0, 0, -135)
    run_skill("mount_machine", "portafilter_cleaner", "hard_brush", True)
    run_skill("moveEE", 0, 0, 100, 0, 0, 0)
    run_skill("approach_machine", "portafilter_cleaner", "soft_brush", True)
    run_skill("mount_machine", "portafilter_cleaner", "soft_brush", True)
    run_skill("moveEE", 0, 0, 150, 0, 0, 0)
    run_skill("gotoJ_deg", *Espresso_grinder_home)
    mount(port)

# Register for CLI discovery
SEQUENCES = {
    'clean': clean,
}
