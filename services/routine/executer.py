# services/routine/executor.py
# from validation_client import call_validation
# from robot_client import call_robot
# from event_publisher import publish_event

import os
import sys
import logging
from datetime import datetime

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.rabbitmq_client import RabbitMQClient

# Configure logging
logger = logging.getLogger(__name__)

# Remove the global client - we'll use the one passed from the main service
# rabbitmq_client = RabbitMQClient("routine_executor")

async def call_validation(func_name: str, params: dict, rabbitmq_client: RabbitMQClient):
    """
    Calls the validation service with the given function name and parameters.
    """
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="validate",
            data={
                "function": func_name,
                "params": params
            },
            timeout=30
        )
        
        if response.get("error"):
            logger.error(f"Validation service error: {response['error']}")
            return {"passed": False, "details": f"Validation service error: {response['error']}"}
        
        return response
        
    except Exception as e:
        logger.error(f"Error calling validation service: {str(e)}")
        return {"passed": False, "details": f"Error calling validation service: {str(e)}"}

async def call_automation(func_name: str, params: dict, rabbitmq_client: RabbitMQClient):
    """
    Calls the automation service with the given function name and parameters.
    """
    try:
        response = await rabbitmq_client.send_request(
            target_service="automation",
            action="automate",
            data={
                "function": func_name,
                "params": params
            },
            timeout=60  # Automation might take longer
        )
        
        if response.get("error"):
            logger.error(f"Automation service error: {response['error']}")
            return {"success": False, "message": f"Automation service error: {response['error']}"}
        
        return response
        
    except Exception as e:
        logger.error(f"Error calling automation service: {str(e)}")
        return {"success": False, "message": f"Error calling automation service: {str(e)}"}

async def call_robot(func_name: str, params: dict, arm_id: int, rabbitmq_client: RabbitMQClient):
    """
    Calls the robot service with the given function name and parameters via RabbitMQ.
    """
    try:
        # Add arm_id to params for robot service
        robot_params = {**params, "arm_id": arm_id}
        
        response = await rabbitmq_client.send_request(
            target_service="robot_arm",
            action="robot_action",
            data={
                "function": func_name,
                "params": robot_params,
                "arm_id": arm_id
            },
            timeout=60  # Robot actions might take longer
        )
        
        if response.get("error"):
            logger.error(f"Robot service error: {response['error']}")
            return {"success": False, "message": f"Robot service error: {response['error']}"}
        
        return response
        
    except Exception as e:
        logger.error(f"Error calling robot service: {str(e)}")
        return {"success": False, "message": f"Error calling robot service: {str(e)}"}

async def publish_event(event_name: str, data: dict, rabbitmq_client: RabbitMQClient):
    """
    Publishes an event to the event bus via RabbitMQ.
    """
    try:
        await rabbitmq_client.send_event(event_name, data)
        logger.debug(f"Published event: {event_name}")
    except Exception as e:
        logger.error(f"Error publishing event {event_name}: {str(e)}")

async def send_feedback_to_scheduler(cup_id: str, action: str, success: bool, rabbitmq_client: RabbitMQClient, message: str = ""):
    """
    Sends feedback to the scheduler service about task completion.
    """
    try:
        response = await rabbitmq_client.send_request(
            target_service="scheduler",
            action="feedback",
            data={
                "cup_id": cup_id,
                "action": action,
                "success": success,
                "message": message,
                "timestamp": datetime.now().isoformat()
            },
            timeout=10
        )
        
        if response.get("success", True):
            logger.info(f"Feedback sent to scheduler for {action} on cup {cup_id}: {'success' if success else 'failed'}")
            return True
        else:
            logger.error(f"Failed to send feedback to scheduler: {response.get('error', 'Unknown error')}")
            return False
            
    except Exception as e:
        logger.error(f"Error sending feedback to scheduler: {str(e)}")
        return False

async def process_task(arm_id: int, task, configs: dict, rabbitmq_client: RabbitMQClient):
    """
    Executes the configured steps for a high-level function on a given arm.
    """
    cup_id = task.get("item", {}).get("cup_id", "unknown")
    function = task.get("function")
    success = True
    message = ""
    
    logger.info(f"Processing task: {function} for cup {cup_id} on arm {arm_id}")
    
    try:
        cfg = configs[function]
        for step in cfg["steps"]:
            step_type = step["type"]
            func_name = step["function"]
            # Merge static params + any item-specific params (if needed)
            params = {**step.get("params", {}), **task.get("item", {})}
            
            logger.info(f"Executing step: {func_name} ({step_type}) for cup {cup_id}")
            
            if step_type == "validation":
                res = await call_validation(func_name, params, rabbitmq_client)
                if not res.get("passed", False):
                    message = f"Validation failed: {res.get('details', '')}"
                    await publish_event("validation.failed", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "reason": res}, rabbitmq_client)
                    success = False
                    break  # abort on validation failure
                    
            elif step_type == "robot":
                res = await call_robot(func_name, params, arm_id=arm_id, rabbitmq_client=rabbitmq_client)
                if not res.get("success", False):
                    message = f"Robot error: {res.get('message', '')}"
                    await publish_event("robot.error", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "error": res.get("message", "")}, rabbitmq_client)
                    success = False
                    break  # abort on robot error
                    
            elif step_type == "automation":
                res = await call_automation(func_name, params, rabbitmq_client)
                if not res.get("success", False):
                    message = f"Automation error: {res.get('message', '')}"
                    await publish_event("automation.error", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "error": res.get("message", "")}, rabbitmq_client)
                    success = False
                    break  # abort on automation error
                    
            # publish a step-completed event
            await publish_event("routine.step_completed", 
                        {"arm": arm_id, "cup": cup_id,
                        "step": func_name}, rabbitmq_client)
                        
    except Exception as e:
        success = False
        message = f"Exception in routine: {str(e)}"
        logger.error(f"Error processing task: {e}")
    
    # Send feedback to scheduler
    logger.info(f"Sending feedback to scheduler: {function} for cup {cup_id} - {'SUCCESS' if success else 'FAILED'}")
    await send_feedback_to_scheduler(cup_id, function, success, rabbitmq_client, message)
    
    # all steps done
    if success:
        logger.info(f"Task completed successfully: {function} for cup {cup_id}")
        await publish_event("routine.completed", 
                    {"arm": arm_id, "cup": cup_id, "function": function}, rabbitmq_client)
    else:
        logger.error(f"Task failed: {function} for cup {cup_id} - {message}")
