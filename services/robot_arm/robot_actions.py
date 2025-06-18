import time
import logging

logger = logging.getLogger(__name__)

def robot_test1(params: dict):
    """Test function 1 for robot arm service"""
    # Simulate robot operation time
    time.sleep(0.8)
    
    return {
        "success": True,
        "message": "robot_test1 passed successfully",
        "details": {
            "test_name": "robot_test1",
            "params_received": params,
            "duration_sec": 0.8,
            "service": "robot_arm"
        }
    }

def robot_test2(params: dict):
    """Test function 2 for robot arm service"""
    # Simulate robot operation time
    time.sleep(1.2)
    
    return {
        "success": True,
        "message": "robot_test2 passed successfully",
        "details": {
            "test_name": "robot_test2",
            "params_received": params,
            "duration_sec": 1.2,
            "service": "robot_arm"
        }
    }

# Robot action implementations mapping
ROBOT_ACTIONS = {
    "robot_test1": robot_test1,
    "robot_test2": robot_test2,
    # Add more robot actions as needed
}

def perform(action_name: str, params: dict):
    """
    Perform a robot action based on the action name and parameters.
    This is called by the gRPC server.
    """
    try:
        if action_name not in ROBOT_ACTIONS:
            logger.error(f"Unknown robot action: {action_name}")
            return {
                "success": False,
                "error": f"Unknown robot action: {action_name}"
            }
        
        logger.info(f"Executing robot action: {action_name} with params: {params}")
        result = ROBOT_ACTIONS[action_name](params)
        logger.info(f"Robot action {action_name} completed successfully")
        
        return result
        
    except Exception as e:
        logger.error(f"Error executing robot action {action_name}: {e}")
        return {
            "success": False,
            "error": str(e)
        }
