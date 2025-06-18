"""
cleaning.py

Defines the cleaning routine for machine ports using parameterized configurations.
"""

import time
from oms_v1.params import CLEANER_PARAMS
from oms_v1.manipulate_node import run_skill
from oms_v1.sequences.espresso import unmount, mount

def clean(port):
    """
    Perform the cleaning sequence on the specified port.

    Args:
        port (str): 'port_1', 'port_2', or 'port_3'.
    """
    params = CLEANER_PARAMS.get(port)
    if not params:
        print(f"[ERROR] unknown port: {port!r}")
        return
    # Prepare for cleaning
    print(f"--- preparing for cleaning on {port} ---")
    unmount(port)  # Unmount the portafilter if necessary
    # Move to the specified home position
    print(f"--- moving to specified_home for {port} ---")
    run_skill("gotoJ_deg", *params['grinder_home'])

    # Rotate brush or cleaner tool
    print(f"--- rotating cleaner by {params['rotate_amount']}° ---")
    run_skill("moveJ_deg", params['rotate_amount'], 0, 0, 0, 0, 0)

    # Move to cleaning point and execute linear push
    print(f"--- moving to clean_point for {port} ---")
    run_skill("gotoJ_deg", *params['clean_point'])
    run_skill("moveEE", *params['linear_push'])
    run_skill("moveJ_deg", *params['twist_around'])

    print(f"--- executing cleaning pattern for {port} ---")
    for offset in params['clean_pattern']:
        run_skill("moveEE", *offset)

    # Twist back after cleaning
    print(f"--- twisting back cutter for {port} ---")
    run_skill("moveJ_deg", *params['twist_back'])
    run_skill("moveEE", 0, -150, 0, 0, 0, 0)

    # Return to the designated return_home pose
    print(f"--- returning to return_home for {port} ---")
    run_skill("gotoJ_deg", *params['return_home'])
    run_skill("moveJ_deg", *params['final_offset'])
    
    # Mount the portafilter back
    print(f"--- mounting portafilter back for {port} ---")
    mount(port, skip_initial_move=True)

# Register for CLI discovery
SEQUENCES = {
    'clean': clean,
}
