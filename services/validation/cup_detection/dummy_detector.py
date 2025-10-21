"""
Dummy Cup Detection System for Testing
Returns simulated cup detection results without requiring camera connection
"""

import time
import random
import logging
from typing import Dict, Union

log = logging.getLogger("dummy-cup-detector")

class DummyCupDetector:
    """
    Dummy cup detector that simulates RF-DETR cup detection results
    Useful for testing the validation service without camera hardware
    """
    
    def __init__(self, config_path: str = "config.py"):
        self.config_path = config_path
        log.info("Dummy Cup Detector initialized - no camera required!")
        
        # Simulate different test scenarios
        self.test_scenarios = [
            "all_empty",      # No cups detected
            "all_full",       # All 4 cups detected  
            "partial_1",      # Cups 0,1 detected
            "partial_2",      # Cups 2,3 detected
            "random"          # Random detection pattern
        ]
        
        self.current_scenario = "random"
        self.scenario_counter = 0
        
    def set_test_scenario(self, scenario: str):
        """Set a specific test scenario"""
        if scenario in self.test_scenarios:
            self.current_scenario = scenario
            log.info(f"Test scenario set to: {scenario}")
        else:
            log.warning(f"Unknown scenario: {scenario}. Available: {self.test_scenarios}")
    
    def get_connection_status(self):
        """Simulate camera connection status"""
        return {
            "status": "Connected (Dummy)",
            "connected": True, 
            "attempts": 1,
            "last_frame_age_s": 0.1
        }
    
    def _transform_result(self, result: dict):
        """
        Transform 0-indexed result to 1-indexed and reverse the values.
        Example: {0:True, 1:False, 2:False, 3:False} -> {1:False, 2:False, 3:False, 4:True}
        """
        # Reverse the values (position 0 -> 3, 1 -> 2, 2 -> 1, 3 -> 0)
        reversed_values = [result.get(3-i, False) for i in range(4)]
        # Convert to 1-indexed dict
        return {i+1: reversed_values[i] for i in range(4)}
    
    def detect(self) -> Union[Dict[int, bool], Dict[str, str]]:
        """
        Simulate cup detection results
        Returns: {1: bool, 2: bool, 3: bool, 4: bool} or {"error": "..."}
        Note: Results are reversed (position 0 -> 3, 1 -> 2, etc.) and 1-indexed
        """
        
        # Simulate processing time
        time.sleep(0.1)
        
        try:
            if self.current_scenario == "all_empty":
                result = {0: False, 1: False, 2: False, 3: False}
                
            elif self.current_scenario == "all_full":
                result = {0: True, 1: True, 2: True, 3: True}
                
            elif self.current_scenario == "partial_1":
                result = {0: True, 1: True, 2: False, 3: False}
                
            elif self.current_scenario == "partial_2":
                result = {0: False, 1: False, 2: True, 3: True}
                
            elif self.current_scenario == "random":
                # Generate random detection pattern
                result = {
                    0: random.choice([True, False]),
                    1: random.choice([True, False]), 
                    2: random.choice([True, False]),
                    3: random.choice([True, False])
                }
                
            else:
                # Cycle through scenarios automatically
                scenarios = ["all_empty", "partial_1", "all_full", "partial_2"]
                scenario = scenarios[self.scenario_counter % len(scenarios)]
                self.scenario_counter += 1
                
                if scenario == "all_empty":
                    result = {0: False, 1: False, 2: False, 3: False}
                elif scenario == "all_full":
                    result = {0: True, 1: True, 2: True, 3: True}
                elif scenario == "partial_1":
                    result = {0: True, 1: True, 2: False, 3: False}
                else:  # partial_2
                    result = {0: False, 1: False, 2: True, 3: True}
            
            # Transform result (reverse + 1-indexed)
            transformed = self._transform_result(result)
            
            # Log the result for debugging
            detected_count = sum(1 for present in transformed.values() if present)
            log.info(f"Dummy detection result: {transformed} ({detected_count}/4 cups detected)")
            
            return transformed
            
        except Exception as e:
            log.error(f"Dummy detector error: {e}")
            return {"error": f"Dummy detector failed: {str(e)}"}
    
    def release(self):
        """Cleanup - nothing to do for dummy detector"""
        log.info("Dummy Cup Detector released")
        pass

# Alias for compatibility
CupDetector = DummyCupDetector
