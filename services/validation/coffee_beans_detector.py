class CoffeeBeansDetector:
    """
    Test class for coffee beans detection with multiple dummy scenarios
    Comment/uncomment the return statements to test different cases
    """
    
    def detect_coffee_beans(self):
        """
        Returns dummy detection results for testing
        Comment/uncomment different return statements to test various scenarios
        """
        
        # =============================================================================
        # PERIODIC DETECTION TEST SCENARIOS (Case 1)
        # =============================================================================
        
        # Scenario 1.1: Normal successful detection with good percentage (>0)
        # Should update inventory with detected percentage
        return {
            "success": True,
            "percentage": 80,
            "confidence": 0.95,
            "message": "Coffee beans detected successfully"
        }
        
        # # Scenario 1.2: Successful detection with medium percentage
        # return {
        #     "success": True,
        #     "percentage": 45.2,
        #     "confidence": 0.88,
        #     "message": "Coffee beans detected successfully"
        # }
        
        # # Scenario 1.3: Successful detection with low but positive percentage
        # return {
        #     "success": True,
        #     "percentage": 5.1,
        #     "confidence": 0.72,
        #     "message": "Coffee beans detected successfully"
        # }
        
        # # Scenario 1.4: Detection successful but zero percentage (no beans visible)
        # # Should NOT update inventory
        # return {
        #     "success": True,
        #     "percentage": 0.0,
        #     "confidence": 0.85,
        #     "message": "No coffee beans detected in visible area"
        # }
        
        # # Scenario 1.5: Detection successful but negative percentage (below detection area)
        # # Should NOT update inventory
        # return {
        #     "success": True,
        #     "percentage": -1.0,
        #     "confidence": 0.60,
        #     "message": "Coffee beans below detection threshold"
        # }
        
        # # Scenario 1.6: Detection failed due to camera/CV error
        # # Should keep current inventory amount
        # raise Exception("Camera connection lost")
        
        # # Scenario 1.7: Detection failed due to image processing error
        # raise Exception("Image processing failed - corrupted frame")
        
        # # Scenario 1.8: Detection failed due to lighting issues
        # raise Exception("Insufficient lighting for detection")
        
        # =============================================================================
        # REFILL OPERATION TEST SCENARIOS (Case 4)
        # =============================================================================
        
        # # Scenario 4.1: Refill successful with high percentage
        # # Should update inventory and return success
        # return {
        #     "success": True,
        #     "percentage": 85.7,
        #     "confidence": 0.92,
        #     "message": "Coffee beans refill detected successfully"
        # }
        
        # # Scenario 4.2: Refill successful with medium percentage
        # return {
        #     "success": True,
        #     "percentage": 60.3,
        #     "confidence": 0.89,
        #     "message": "Coffee beans refill detected successfully"
        # }
        
        # # Scenario 4.3: Refill detection but beans still below visible area
        # # Should trigger "visibility_issue" alert
        # return {
        #     "success": True,
        #     "percentage": 0.0,
        #     "confidence": 0.75,
        #     "message": "Coffee beans still below detection area"
        # }
        
        # # Scenario 4.4: Refill detection with negative percentage 
        # # Should trigger "visibility_issue" alert
        # return {
        #     "success": True,
        #     "percentage": -2.5,
        #     "confidence": 0.68,
        #     "message": "Coffee beans below minimum detection level"
        # }
        
        # # Scenario 4.5: Refill detection failed - camera disconnected
        # # Should trigger "camera_reconnect" alert
        # raise Exception("Camera hardware disconnected")
        
        # # Scenario 4.6: Refill detection failed - network timeout
        # # Should trigger "camera_reconnect" alert
        # raise Exception("Network timeout connecting to camera")
        
        # # Scenario 4.7: Refill detection failed - CV algorithm error
        # # Should trigger "camera_reconnect" alert
        # raise Exception("Computer vision algorithm initialization failed")
