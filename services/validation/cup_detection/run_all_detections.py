#!/usr/bin/env python3
"""
Script to run all detection methods from cup_detector.py
- detect_sauce: Sauce dispenser detection
- detect_milk: Milk dispenser detection  
- detect: Main cup detection for 4 positions

Usage:
    # Activate virtual environment first (from parent directory):
    # Windows: venv\Scripts\activate
    # Linux/Mac: source venv/bin/activate
    
    # Then run:
    python run_all_detections.py
"""

import os
import sys
import time
import logging

# Add parent directory to path if needed
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from cup_detector import RFDETRDetector

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger(__name__)


def main():
    """Main function to test all detection methods"""
    
    log.info("="*60)
    log.info("Starting All Detection Methods Test")
    log.info("="*60)
    
    try:
        # Initialize detector (loads config.py from current directory)
        log.info("\n[1/4] Initializing RFDETRDetector...")
        detector = RFDETRDetector(config_path="config.py")
        
        # Wait for camera connection
        log.info("\n[2/4] Waiting for camera connection...")
        max_wait = 10  # seconds
        start_time = time.time()
        while time.time() - start_time < max_wait:
            status = detector.get_connection_status()
            if status.get("connected"):
                log.info(f"✓ Camera connected! (attempts: {status.get('attempts', 0)})")
                break
            time.sleep(0.5)
        else:
            log.warning("⚠ Camera not connected within timeout, attempting detection anyway...")
        
        # Give detector time to capture frames
        time.sleep(1)
        
        log.info("\n" + "="*60)
        log.info("Running Detection Methods")
        log.info("="*60)
        
        # Call detect_sauce
        log.info("\n[3/4] Calling detect_sauce()...")
        sauce_result = detector.detect_cup_sauce_dispenser()
        log.info(f"Sauce Detection Result: {sauce_result}")
        if isinstance(sauce_result, dict) and "error" in sauce_result:
            log.error(f"  └─ Error: {sauce_result['error']}")
        elif sauce_result:
            log.info("  └─ ✓ Cup detected at sauce dispenser")
        else:
            log.info("  └─ ✗ No cup at sauce dispenser")
        
        time.sleep(0.5)  # Brief delay between detections
        
        # Call detect_milk
        log.info("\n[4/4] Calling detect_milk()...")
        milk_result = detector.detect_cup_milk_dispenser()
        log.info(f"Milk Detection Result: {milk_result}")
        if isinstance(milk_result, dict) and "error" in milk_result:
            log.error(f"  └─ Error: {milk_result['error']}")
        elif milk_result:
            log.info("  └─ ✓ Cup detected at milk dispenser")
        else:
            log.info("  └─ ✗ No cup at milk dispenser")
        
        time.sleep(0.5)  # Brief delay between detections
        
        # Call detect (main 4-position cup detection)
        log.info("\n[5/5] Calling detect() for 4 cup positions...")
        detect_result = detector.detect_cups_on_station()
        log.info(f"Cup Detection Result: {detect_result}")
        if isinstance(detect_result, dict) and "error" in detect_result:
            log.error(f"  └─ Error: {detect_result['error']}")
        else:
            for pos, present in detect_result.items():
                status = "✓ Present" if present else "✗ Empty"
                log.info(f"  └─ Position {pos}: {status}")
        
        # Summary
        log.info("\n" + "="*60)
        log.info("Detection Summary")
        log.info("="*60)
        log.info(f"Sauce Dispenser: {sauce_result}")
        log.info(f"Milk Dispenser:  {milk_result}")
        log.info(f"Cup Positions:   {detect_result}")
        log.info("="*60)
        
        # Cleanup
        log.info("\nCleaning up...")
        detector.release()
        log.info("✓ Detector released successfully")
        
    except FileNotFoundError as e:
        log.error(f"✗ Config file not found: {e}")
        log.error("  Make sure 'config.py' exists in the cup_detection directory")
        sys.exit(1)
    except Exception as e:
        log.exception(f"✗ Error during detection: {e}")
        sys.exit(1)
    
    log.info("\n✓ All detection methods completed successfully!")


if __name__ == "__main__":
    main()

