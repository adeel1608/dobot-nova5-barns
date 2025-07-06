import asyncio
import time
import logging
import os
import sys
from typing import Dict

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

logger = logging.getLogger(__name__)

async def robot_test1(params: dict):
    """Test function 1 for robot arm service"""
    arm_id = params.get("arm_id", 1)
    simulation_mode = params.get("simulation_mode", True)
    
    # Simulate robot operation time
    await asyncio.sleep(5)
    
    return {
        "success": True,
        "message": "robot_test1 passed successfully",
        "details": {
            "test_name": "robot_test1",
            "arm_id": arm_id,
            "params_received": params,
            "duration_sec": 0.8,
            "service": "robot_arm",
            "simulation_mode": simulation_mode
        }
    }

async def robot_test2(params: dict):
    """Test function 2 for robot arm service"""
    arm_id = params.get("arm_id", 1)
    simulation_mode = params.get("simulation_mode", True)
    
    # Simulate robot operation time
    await asyncio.sleep(5)
    
    return {
        "success": True,
        "message": "robot_test2 passed successfully",
        "details": {
            "test_name": "robot_test2",
            "arm_id": arm_id,
            "params_received": params,
            "duration_sec": 1.2,
            "service": "robot_arm",
            "simulation_mode": simulation_mode
        }
    }

async def call_robot_container(action_name: str, params: dict):
    """Call the actual robot container via RabbitMQ"""
    try:
        import uuid
        from shared.rabbitmq_client import RabbitMQClient
        
        # Create RabbitMQ client with unique name to avoid queue conflicts
        unique_client_name = f"robot_arm_bridge_{uuid.uuid4().hex[:8]}"
        robot_client = RabbitMQClient(unique_client_name)
        await robot_client.connect()
        
        try:
            # Determine which robot container to call based on arm_id
            arm_id = params.get("arm_id", 1)
            robot_service = f"robot_container_{arm_id}"  # robot_container_1 or robot_container_2
            
            logger.info(f"Calling robot container {robot_service} for action: {action_name}")
            logger.info(f"---\nParams: {params}\n---")
            response = await robot_client.send_request(
                target_service=robot_service,
                action="execute_action",
                data={
                    "action_name": action_name,
                    "params": params
                },
                timeout=300  # Robot actions can take longer
            )
            
            if response.get("error"):
                logger.error(f"Robot container error: {response['error']}")
                return {
                    "success": False,
                    "error": f"Robot container error: {response['error']}",
                    "message": "Failed to execute robot action"
                }
            
            return response
            
        finally:
            await robot_client.disconnect()
            
    except Exception as e:
        logger.error(f"Error calling robot container: {str(e)}")
        return {
            "success": False,
            "error": f"Error calling robot container: {str(e)}",
            "message": "Robot container communication failed"
        }

# Robot action implementations mapping
TEST_ACTIONS = {
    "robot_test1": robot_test1,
    "robot_test2": robot_test2,
}

# Combined actions - includes both test and real robot functions
ROBOT_ACTIONS = TEST_ACTIONS.copy()

# Add a function to get available actions from robot container
async def get_robot_container_actions():
    """Get available actions from robot container"""
    try:
        import uuid
        from shared.rabbitmq_client import RabbitMQClient
        
        unique_client_name = f"robot_arm_bridge_{uuid.uuid4().hex[:8]}"
        robot_client = RabbitMQClient(unique_client_name)
        await robot_client.connect()
        
        try:
            response = await robot_client.send_request(
                target_service="robot_container_1",  # Query first robot for available actions
                action="list_actions",
                data={},
                timeout=10
            )
            
            if response.get("success") and "actions" in response:
                return response["actions"]
            
        finally:
            await robot_client.disconnect()
            
    except Exception as e:
        logger.error(f"Error getting robot container actions: {e}")
    
    return []

# Legacy function for backwards compatibility
def perform(action_name: str, params: dict):
    """
    Legacy synchronous function for backwards compatibility.
    This is called by the old gRPC server.
    """
    try:
        if action_name not in TEST_ACTIONS:
            # For non-test actions, call robot container
            logger.info(f"Calling robot container for action: {action_name}")
            
            # Run the async function synchronously for legacy compatibility
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                result = loop.run_until_complete(call_robot_container(action_name, params))
            finally:
                loop.close()
            
            return result
        else:
            # For test actions, run locally
            logger.info(f"Executing test action locally: {action_name}")
            
            # Run the async function synchronously for legacy compatibility
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                result = loop.run_until_complete(TEST_ACTIONS[action_name](params))
            finally:
                loop.close()
            
            logger.info(f"Robot action {action_name} completed successfully")
            return result
        
    except Exception as e:
        logger.error(f"Error executing robot action {action_name}: {e}")
        return {
            "success": False,
            "error": str(e)
        }

# Async version for new RabbitMQ-based robot service
async def execute_robot_action(action_name: str, params: dict):
    """
    Async function to execute robot actions.
    Routes to robot container for non-test actions, executes locally for test actions.
    """
    try:
        if action_name in TEST_ACTIONS:
            # Execute test actions locally
            logger.info(f"Executing test action locally: {action_name}")
            return await TEST_ACTIONS[action_name](params)
        else:
            # Call robot container for real robot actions
            logger.info(f"Routing to robot container for action: {action_name}")
            return await call_robot_container(action_name, params)
        
    except Exception as e:
        logger.error(f"Error executing robot action {action_name}: {e}")
        return {
            "success": False,
            "error": str(e),
            "message": f"Failed to execute robot action: {action_name}"
        }
