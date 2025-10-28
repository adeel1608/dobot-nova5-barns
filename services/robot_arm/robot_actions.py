import asyncio
import time
import logging
import os
import sys
from typing import Dict

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.logger import log

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
    """Call the actual robot container via RabbitMQ with two-phase approach and retry logic"""
    try:
        import uuid
        from shared.logger import log
        from shared.rabbitmq_client import RabbitMQClient
        
        # Create RabbitMQ client with unique name to avoid queue conflicts
        unique_client_name = f"robot_arm_bridge_{uuid.uuid4().hex[:8]}"
        robot_client = RabbitMQClient(unique_client_name)
        await robot_client.connect()
        
        try:
            # Determine which robot container to call based on arm_id
            arm_id = params.get("arm_id", 1)
            robot_service = f"robot_container_{arm_id}"  # robot_container_1 or robot_container_2
            
            log("INFO", "Action", service="robot_arm")
            
            # PHASE 1: Robust health check with retry logic
            health_check_success = False
            max_health_retries = 3
            health_retry_delay = 2
            
            for health_attempt in range(max_health_retries):
                try:
                    log("INFO", "Action", service="robot_arm")
                    
                    health_response = await robot_client.send_request(
                        target_service=robot_service,
                        action="health",
                        data={},
                        timeout=15  # Increased timeout for health check
                    )
                    
                    log("INFO", "Action", service="robot_arm")
                    
                    # More flexible health check - accept if we get any valid response
                    if health_response and (
                        health_response.get("status") == "healthy" or 
                        health_response.get("robot_id") is not None or
                        health_response.get("service") is not None
                    ):
                        log("INFO", "Success", service="robot_arm")
                        health_check_success = True
                        break
                    else:
                        log("ERROR", "Warning", service="robot_arm")
                        if health_attempt < max_health_retries - 1:
                            log("INFO", "Retrying health check in {health_retry_delay} seconds...", service="robot_arm")
                            await asyncio.sleep(health_retry_delay)
                            continue
                        
                except asyncio.TimeoutError:
                    log("ERROR", "⏰ [PHASE 1] Health check timeout for {robot_service} (attempt {health_attempt + 1})", service="robot_arm")
                    if health_attempt < max_health_retries - 1:
                        log("INFO", "Retrying health check in {health_retry_delay} seconds...", service="robot_arm")
                        await asyncio.sleep(health_retry_delay)
                        continue
                        
                except Exception as health_error:
                    log("ERROR", "Warning", service="robot_arm")
                    if health_attempt < max_health_retries - 1:
                        log("INFO", "Retrying health check in {health_retry_delay} seconds...", service="robot_arm")
                        await asyncio.sleep(health_retry_delay)
                        continue
            
            if not health_check_success:
                log("ERROR", "Health check failed after retries", service="robot_arm", arm=arm_id, action=action_name)
                return {
                    "success": False,
                    "error": f"Robot container {robot_service} is not responding after {max_health_retries} attempts",
                    "message": "Robot container not connected or not responding",
                    "phase": "health_check_failed"
                }
            
            # PHASE 2: Execute the actual action with retry logic
            log("INFO", "Action", service="robot_arm")
            log("INFO", "---\nParams: {params}\n---", service="robot_arm")
            
            max_execution_retries = 2
            execution_retry_delay = 3
            
            for exec_attempt in range(max_execution_retries):
                try:
                    log("INFO", "Action", service="robot_arm")
                    
                    response = await robot_client.send_request(
                        target_service=robot_service,
                        action="execute_action",
                        data={
                            "action_name": action_name,
                            "params": params
                        },
                        timeout=300  # Robot actions can take longer, but we know it's responsive
                    )
                    
                    if response.get("error"):
                        log("ERROR", "Action execution returned error", service="robot_arm", arm=arm_id, action=action_name, error=response.get("error", "")[:50])
                        # Don't retry execution errors - these are likely action-specific failures
                        return {
                            "success": False,
                            "error": f"Robot execution error: {response['error']}",
                            "message": "Robot action execution failed",
                            "phase": "execution_failed"
                        }
                    
                    log("INFO", "Success", service="robot_arm")
                    return response
                    
                except asyncio.TimeoutError:
                    log("ERROR", "⏰ [PHASE 2] Action execution timeout for {action_name} (attempt {exec_attempt + 1})", service="robot_arm")
                    if exec_attempt < max_execution_retries - 1:
                        log("INFO", "Retrying action execution in {execution_retry_delay} seconds...", service="robot_arm")
                        await asyncio.sleep(execution_retry_delay)
                        continue
                    else:
                        return {
                            "success": False,
                            "error": f"Action {action_name} timed out after {max_execution_retries} attempts",
                            "message": "Robot action execution timed out",
                            "phase": "execution_timeout"
                        }
                        
                except Exception as exec_error:
                    log("ERROR", "Warning", service="robot_arm")
                    if exec_attempt < max_execution_retries - 1:
                        log("INFO", "Retrying action execution in {execution_retry_delay} seconds...", service="robot_arm")
                        await asyncio.sleep(execution_retry_delay)
                        continue
                    else:
                        return {
                            "success": False,
                            "error": f"Action execution failed: {str(exec_error)}",
                            "message": "Robot action execution failed with error",
                            "phase": "execution_error"
                        }
            
        finally:
            await robot_client.disconnect()
            
    except Exception as e:
        log("ERROR", "Error", service="robot_arm")
        return {
            "success": False,
            "error": f"Robot container communication error: {str(e)}",
            "message": "Robot container communication failed",
            "phase": "communication_error"
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
        from shared.logger import log
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
        log("ERROR", "Error getting robot container actions: {e}", service="robot_arm")
    
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
            log("INFO", "Calling robot container for action: {action_name}", service="robot_arm")
            
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
            log("INFO", "Executing test action locally: {action_name}", service="robot_arm")
            
            # Run the async function synchronously for legacy compatibility
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                result = loop.run_until_complete(TEST_ACTIONS[action_name](params))
            finally:
                loop.close()
            
            log("INFO", "Robot action {action_name} completed successfully", service="robot_arm")
            return result
        
    except Exception as e:
        log("ERROR", "Error executing robot action {action_name}: {e}", service="robot_arm")
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
    arm_id = params.get("arm_id", 1)
    
    try:
        if action_name in TEST_ACTIONS:
            # Execute test actions locally
            log("INFO", "🧪 [LOCAL-TEST] Executing test action locally: {action_name}", service="robot_arm")
            result = await TEST_ACTIONS[action_name](params)
            log("INFO", "Success", service="robot_arm")
            return result
        else:
            # Call robot container for real robot actions
            log("INFO", "Action", service="robot_arm")
            result = await call_robot_container(action_name, params)
            
            # Add more context to the result based on which phase failed
            if not result.get("success"):
                phase = result.get("phase", "unknown")
                error = result.get("error", "Unknown error")
                
                if phase == "health_check_timeout":
                    log("ERROR", "Health check timeout - robot not responding", service="robot_arm", arm=arm_id, action=action_name)
                    result["user_message"] = f"Robot Arm {arm_id} is not connected or responding"
                elif phase == "health_check_failed":
                    log("ERROR", "Health check failed - robot not ready", service="robot_arm", arm=arm_id, action=action_name)
                    result["user_message"] = f"Robot Arm {arm_id} is not ready for operations"
                elif phase == "execution_failed":
                    log("ERROR", "Robot action execution failed", service="robot_arm", arm=arm_id, action=action_name, error=error[:50])
                    result["user_message"] = f"Robot Arm {arm_id} failed to execute {action_name}"
                else:
                    log("ERROR", "Robot communication error", service="robot_arm", arm=arm_id, action=action_name, phase=phase)
                    result["user_message"] = f"Communication error with Robot Arm {arm_id}"
            else:
                log("INFO", "Success", service="robot_arm")
            
            return result
        
    except Exception as e:
        log("ERROR", "Error", service="robot_arm")
        return {
            "success": False,
            "error": str(e),
            "message": f"Failed to execute robot action: {action_name}",
            "user_message": f"Unexpected error with Robot Arm {arm_id}",
            "phase": "unexpected_error"
        }
