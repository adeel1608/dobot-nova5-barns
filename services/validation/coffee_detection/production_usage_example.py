"""
Simple production usage - runs detection every 10 minutes
"""

import time
from camera_worker_production import ProductionCoffeeDetector, load_config

def main():
    config = load_config("detection_config.json")
    detector = ProductionCoffeeDetector(config)
    
    try:
        while True:
            # Detect coffee
            t0 = time.time()
            result = detector.detect_coffee()
            print(result)
            
            if result["coffee"]:
                print("COFFEE DETECTED!")
            dt = time.time() - t0
            print(f"Detection time: {dt:.3f}s")
            # Wait 10 minutes
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        detector.cleanup()

if __name__ == "__main__":
    main()