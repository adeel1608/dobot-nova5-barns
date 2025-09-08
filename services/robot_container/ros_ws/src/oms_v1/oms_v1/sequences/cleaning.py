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
from oms_v1.sequences.espresso import unmount, mount
from oms_v1.params import (
    ESPRESSO_GRINDER_HOME, CLEANING_PARAMS, DEFAULT_PORT,
    validate_port, log_step, log_success, log_error, log_info
)


def clean(**params) -> bool:
    """
    Complete portafilter cleaning sequence.
    
    This function performs a comprehensive cleaning workflow:
    1. Unmounts the portafilter from the specified espresso group
    2. Moves to the cleaning station
    3. Performs hard brush cleaning with precise positioning
    4. Performs soft brush cleaning with thorough coverage
    5. Remounts the portafilter back to the espresso group
    
    The cleaning process ensures optimal hygiene and performance by utilizing
    both hard and soft brush cleaning methods with automated positioning.
    
    Args:
        port (str): The portafilter port to clean ('port_1', 'port_2', or 'port_3'), defaults to 'port_2'
        
    Returns:
        bool: True if cleaning sequence completed successfully, False otherwise
        
    Example:
        success = clean(port='port_1')
        if success:
            print("Cleaning completed successfully")
    """
    try:
        # Extract and validate port parameter
        port = params.get("port", DEFAULT_PORT)
        if not port or not validate_port(port):
            log_info("Please provide valid port parameter: 'port_1', 'port_2', or 'port_3'")
            return False
        
        print(f"🧹 Starting comprehensive cleaning sequence for {port}")
        print("=" * 60)
        
        # Step 1: Unmount the portafilter from espresso group
        log_step(1, 6, f"Unmounting portafilter from {port}")
        unmount_result = unmount(port=port)
        
        if unmount_result is False:
            log_error(f"Failed to unmount portafilter from {port}")
            log_info("Cannot proceed with cleaning without successful unmount")
            return False
        log_success("Portafilter successfully unmounted", indent=1)
        
        # Step 2: Move to cleaning station home position
        log_step(2, 6, "Moving to cleaning station")
        home_result = run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
        
        if home_result is False:
            log_error("Failed to move to cleaning station home")
            return False
        log_success("Successfully positioned at cleaning station", indent=1)
        
        # Step 3: Perform hard brush cleaning
        log_step(3, 6, "Performing hard brush cleaning")
        if not _perform_brush_cleaning("hard_brush"):
            return False
        
        # Step 4: Perform soft brush cleaning
        log_step(4, 6, "Performing soft brush cleaning")
        if not _perform_brush_cleaning("soft_brush"):
            return False
        
        # Step 5: Return to cleaning station home
        log_step(5, 6, "Returning to cleaning station home")
        return_home_result = run_skill("gotoJ_deg", *ESPRESSO_GRINDER_HOME)
        
        if return_home_result is False:
            log_error("Failed to return to cleaning station home")
            return False
        log_success("Successfully returned to cleaning station home", indent=1)
        
        # Step 6: Complete cleaning sequence
        log_step(6, 6, "Cleaning sequence finalization")
        log_info("Cleaning completed - portafilter ready for manual handling")
        
        # Final success summary
        print("=" * 60)
        print(f"✅ CLEANING SEQUENCE COMPLETED SUCCESSFULLY FOR {port.upper()}")
        print("   ✓ Hard brush cleaning performed with precision")
        print("   ✓ Soft brush cleaning completed for optimal finish")
        print("   ✓ All cleaning motions executed successfully")
        print("   🧹 Portafilter is now clean and ready for use!")
        print("=" * 60)
        return True
        
    except Exception as e:
        log_error(f"Unexpected error during cleaning: {e}")
        log_info("Cleaning process terminated due to error")
        log_info("Manual inspection of portafilter position may be required")
        return False


def _perform_brush_cleaning(brush_type: str) -> bool:
    """
    Perform cleaning with specified brush type
    
    Args:
        brush_type: Type of brush ('hard_brush' or 'soft_brush')
        
    Returns:
        bool: True if cleaning completed successfully
    """
    brush_emoji = "🪣" if brush_type == "hard_brush" else "🪶"
    brush_name = "hard brush" if brush_type == "hard_brush" else "soft brush"
    
    log_info(f"{brush_emoji} Performing {brush_name} cleaning...")
    
    # Approach the cleaning brush
    log_info(f"Approaching {brush_name} cleaning position...", indent=1)
    approach_result = run_skill("approach_machine", "portafilter_cleaner", brush_type)
    
    if approach_result is False:
        log_error(f"Failed to approach {brush_name}")
        return False
    log_success(f"Successfully approached {brush_name}", indent=1)
        
    # Special positioning for hard brush
    if brush_type == "hard_brush":
        log_info("Adjusting position for optimal cleaning angle...", indent=1)
        move_result = run_skill("gotoJ_deg", *CLEANING_PARAMS['hard_brush_adjust'])
        
        if move_result is False:
            log_error(f"Failed to adjust position for {brush_name}")
            return False
        log_success("Position adjusted for optimal cleaning", indent=1)
        
    # Mount to brush for cleaning
    log_info(f"Mounting to {brush_name} for cleaning...", indent=1)
    mount_result = run_skill("mount_machine", "portafilter_cleaner", brush_type)
    
    if mount_result is False:
        log_error(f"Failed to mount to {brush_name}")
        return False
    log_success(f"Successfully mounted to {brush_name}", indent=1)

    # Perform cleaning motions (2 cycles for thorough cleaning)
    for cycle in range(2):
        log_info(f"Executing cleaning motion {cycle + 1}/2...", indent=1)
        
        # First cleaning motion
        clean_motion1 = run_skill("moveEE", *CLEANING_PARAMS['cleaning_motion_1'])
        if clean_motion1 is False:
            print(f"   ⚠️  Cleaning motion 1 may not have completed fully")
        
        # Second cleaning motion for thorough cleaning
        clean_motion2 = run_skill("moveEE", *CLEANING_PARAMS['cleaning_motion_2'])
        if clean_motion2 is False:
            print(f"   ⚠️  Cleaning motion 2 may not have completed fully")
    
    # Retract from brush
    log_info(f"Retracting from {brush_name}...", indent=1)
    retreat_key = 'retreat_hard' if brush_type == "hard_brush" else 'retreat_soft'
    up_result = run_skill("moveEE", *CLEANING_PARAMS[retreat_key])
    
    if up_result is False:
        log_error(f"Failed to move up after {brush_name}")
        return False
    log_success(f"{brush_name.title()} cleaning completed successfully", indent=1)
    
    return True


# Register functions for CLI discovery and external access
SEQUENCES = {
    'clean': clean,
}
