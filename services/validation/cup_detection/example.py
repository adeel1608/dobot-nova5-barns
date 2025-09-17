"""
Simple Usage Example for Cup Detection System
"""

from cup_detector import CupDetector
import time

def main():
    """Simple example of how to use the cup detector"""
    
    # Create detector (automatically loads config.py)
    detector = CupDetector("config.py")
    
    try:
        print("Cup Detector Started!")
        print("Press Ctrl+C to stop")
        
        while True:
            # Detect cups
            result = detector.detect()
            
            # Check result
            if "error" in result:
                print(result)
                print(f"ERROR: {result['error']}")
            else:
                print(f"Result: {result}")
            
            # Wait a bit
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        detector.release()

if __name__ == "__main__":
    main()
