# services/routine/executor.py
# from validation_client import call_validation
# from robot_client import call_robot
# from event_publisher import publish_event

import os
import sys
import logging
from datetime import datetime
import asyncio

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
    The function name is sent directly as the action to match validation service handlers.
    """
    try:
        # Prepare payload with metadata expected by validation service
        payload = {
            "request_id": f"routine-{datetime.now().timestamp()}",
            "client_type": "routine",
            **params  # Merge any additional params
        }
        
        logger.info(f"Calling validation service: action={func_name}, payload={payload}")
        
        # Send function name as action directly (e.g., "cup_detection", "check_coffee_beans")
        # The validation service has handlers registered for specific actions, not a generic "validate"
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action=func_name,  # Send function name directly as action
            data=payload,  # Send payload with metadata
            timeout=30
        )
        
        logger.info(f"Validation service response: {response}")
        
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
        logger.info(f"📞 [ROUTINE] Sending automation request: function='{func_name}', params={params}")
        
        response = await rabbitmq_client.send_request(
            target_service="automation",
            action="automate",
            data={
                "function": func_name,
                "params": params
            },
            timeout=80  # 80 second timeout for automation functions
        )
        
        logger.info(f"📨 [ROUTINE] Received automation response: {response}")
        
        if response.get("error"):
            logger.error(f"❌ [ROUTINE] Automation service error: {response['error']}")
            return {"success": False, "message": f"Automation service error: {response['error']}"}
        
        logger.info(f"✅ [ROUTINE] Automation request completed successfully")
        return response
        
    except Exception as e:
        logger.error(f"❌ [ROUTINE] Error calling automation service: {str(e)}")
        logger.error(f"❌ [ROUTINE] Exception details: {type(e).__name__}: {str(e)}")
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
            timeout=300  # Robot actions might take longer
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

def find_nearest_available_position(current_position: int, detection_result: dict) -> int:
    """
    Find the nearest available (False) cup position to the current position.
    
    Args:
        current_position: The originally intended cup position (e.g., 1)
        detection_result: Dict with position as key and occupancy as value 
                         (True = occupied, False = available)
    
    Returns:
        The nearest available position number
    """
     Ensure current_position is an integer
    current_position = int(current_position)
    
    # Get all available positions (False values) and ensure they're integers
    available_positions = [int(pos) for pos, occupied in detection_result.items() if not occupied]
    if not available_positions:
        logger.warning(f"No available cup positions found in detection result: {detection_result}")
        return current_position  # Return original if none available
    
    # If current position is available, use it
    if current_position in available_positions:
        logger.info(f"Current position {current_position} is available, no change needed")
        return current_position
    
    # Find nearest available position by calculating absolute distance
    nearest_position = min(available_positions, key=lambda pos: abs(pos - current_position))
    
    logger.info(f"Original position {current_position} is occupied. Using nearest available: {nearest_position}")
    logger.info(f"Available positions: {sorted(available_positions)}")
    
    return nearest_position

async def send_feedback_to_scheduler(cup_id: str, action: str, success: bool, rabbitmq_client: RabbitMQClient, message: str = ""):
    """Send feedback to scheduler with retry logic and fallback event notification."""
    logger.info(f"Sending feedback to scheduler: {action} for cup {cup_id} - {'SUCCESS' if success else 'FAILED'}")
    
    feedback_data = {
        "cup_id": cup_id,
        "action": action,
        "success": success,
        "message": message,
        "timestamp": datetime.now().isoformat()
    }
    
    max_retries = 3
    retry_delay = 2  # seconds
    
    for attempt in range(max_retries):
        try:
            logger.info(f"Sending feedback to scheduler (attempt {attempt + 1}/{max_retries}): {feedback_data}")
            
            # Send feedback to scheduler
            response = await rabbitmq_client.send_request(
                target_service="scheduler",
                action="feedback",
                data=feedback_data,
                timeout=10  # Increased timeout to 10 seconds
            )
            
            if response and response.get("success"):
                logger.info(f"Feedback sent to scheduler for {action} on cup {cup_id}: success")
                return True
            else:
                logger.warning(f"Scheduler returned error for feedback (attempt {attempt + 1}): {response}")
                if attempt < max_retries - 1:
                    logger.info(f"Retrying feedback in {retry_delay} seconds...")
                    await asyncio.sleep(retry_delay)
                    continue
                else:
                    logger.error(f"Failed to send feedback after {max_retries} attempts: {response}")
                    break
                    
        except asyncio.TimeoutError:
            logger.error(f"Timeout sending feedback to scheduler (attempt {attempt + 1}/{max_retries})")
            if attempt < max_retries - 1:
                logger.info(f"Retrying feedback in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
                continue
            else:
                logger.error(f"Failed to send feedback after {max_retries} timeout attempts")
                break
                
        except ConnectionError as e:
            logger.error(f"Connection error sending feedback to scheduler (attempt {attempt + 1}/{max_retries}): {e}")
            if attempt < max_retries - 1:
                logger.info(f"Retrying feedback in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
                continue
            else:
                logger.error(f"Failed to send feedback after {max_retries} connection error attempts")
                break
                
        except Exception as e:
            logger.error(f"Unexpected error sending feedback to scheduler (attempt {attempt + 1}/{max_retries}): {e}")
            if attempt < max_retries - 1:
                logger.info(f"Retrying feedback in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
                continue
            else:
                logger.error(f"Failed to send feedback after {max_retries} attempts due to error: {e}")
                break
    
    # If all retries failed, use event-based fallback notification
    logger.warning(f"All feedback retries failed. Using event-based fallback for {action} on cup {cup_id}")
    try:
        # Send event as fallback - this uses a different RabbitMQ mechanism that may be more resilient
        event_name = "routine.task_completed" if success else "routine.task_failed"
        event_data = {
            "cup_id": cup_id,
            "function": action,
            "arm_id": 1,  # This could be made dynamic if needed
            "timestamp": datetime.now().isoformat()
        }
        
        if not success:
            event_data["error"] = message or "Task execution failed"
            
        await rabbitmq_client.send_event(event_name, event_data)
        logger.info(f"Fallback event sent: {event_name} for {action} on cup {cup_id}")
        return True
        
    except Exception as fallback_error:
        logger.error(f"Fallback event notification also failed for {action} on cup {cup_id}: {fallback_error}")
        return False

async def process_task(arm_id: int, task, configs: dict, rabbitmq_client: RabbitMQClient):
    """
    Executes the configured steps for a high-level function on a given arm.
    """
    cup_id = task.get("item", {}).get("cup_id", "unknown")
    function = task.get("function")
    success = True
    message = ""
    
    # Add small staggered delay for Arm 2 to prevent RabbitMQ overload when both arms start simultaneously
    if arm_id == 2:
        logger.info(f"[ARM-2] Adding 1s stagger delay to prevent parallel connection overload")
        await asyncio.sleep(1)
    
    logger.info(f"Processing task: {function} for cup {cup_id} on arm {arm_id}")
    
    try:
        cfg = configs[function]
        # Get reference to task's ingredients (not a copy) so updates persist
        task_item = task.get("item", {})
        if "ingredients" not in task_item:
            task_item["ingredients"] = {}
        
        for step in cfg["steps"]:
            step_type = step["type"]
            func_name = step["function"]
            # Get reference to ingredients (not a copy) so updates persist across steps
            ingredients = task_item["ingredients"]
            # Create a copy for params to send to services
            params = dict(ingredients)
            
            logger.info(f"─────────────────────────────────────────────────")
            logger.info(f"Executing step: {func_name} ({step_type}) for cup {cup_id}")
            logger.info(f"📦 Step params (from task ingredients): {params}")
            
            # Log cup_position specifically for debugging
            if "position" in params and "cup_position" in params["position"]:
                logger.info(f"🎯 Cup position for this step: {params['position']['cup_position']}")
            elif "cup_position" in params:
                logger.info(f"🎯 Cup position for this step: {params['cup_position']}")
            else:
                logger.info(f"🎯 No cup_position found in params")
            
            if step_type == "validation":
                res = await call_validation(func_name, params, rabbitmq_client)
                
                # Special handling for cup_detection - update cup position based on availability
                if func_name == "cup_detection" and res.get("passed", False):
                    # Try to get detection_result from top level first, then from details
                    detection_result = res.get("detection_result") or res.get("details", {}).get("cups_detected", {})
                    if detection_result:
                        logger.info(f"🔍 Cup detection result: {detection_result}")
                        # Get current cup_position from task ingredients (already retrieved on line 277)
                        current_position = None
                        
                        # Look for cup_position in ingredients (check multiple possible locations)
                        # Option 1: Direct key 'cup_position'
                        if "cup_position" in ingredients:
                            cup_pos_data = ingredients["cup_position"]
                            if isinstance(cup_pos_data, dict):
                                # Extract position value from nested dict: {'cup_position': 1.0}
                                current_position = int(list(cup_pos_data.values())[0])
                            elif isinstance(cup_pos_data, (int, float)):
                                current_position = int(cup_pos_data)
                        # Option 2: Nested under 'position' key
                        elif "position" in ingredients and isinstance(ingredients["position"], dict):
                            if "cup_position" in ingredients["position"]:
                                cup_pos_value = ingredients["position"]["cup_position"]
                                if isinstance(cup_pos_value, (int, float)):
                                    current_position = int(cup_pos_value)
                        
                        if current_position:
                            logger.info(f"📍 Current cup position from task: {current_position}")
                            
                            # Find nearest available position
                            new_position = find_nearest_available_position(current_position, detection_result)
                            
                            # Update task params with new position if it changed
                            if new_position != current_position:
                                logger.info(f"🔄 Updating cup position from {current_position} to {new_position} for cup {cup_id}")
                                
                                # Update the task's ingredient data
                                # Update direct 'cup_position' key if exists
                                if "cup_position" in ingredients:
                                    if isinstance(ingredients["cup_position"], dict):
                                        # Update the nested dict format
                                        ingredients["cup_position"]["cup_position"] = float(new_position)
                                    else:
                                        ingredients["cup_position"] = float(new_position)
                                
                                # Update nested 'position.cup_position' if exists
                                if "position" in ingredients and isinstance(ingredients["position"], dict):
                                    if "cup_position" in ingredients["position"]:
                                        old_val = ingredients["position"]["cup_position"]
                                        ingredients["position"]["cup_position"] = float(new_position)
                                        logger.info(f"✅ Updated ingredients['position']['cup_position']: {old_val} → {new_position}")
                                
                                # Also update params for subsequent steps (will be used in line 276)
                                if "cup_position" in params:
                                    if isinstance(params["cup_position"], dict):
                                        params["cup_position"]["cup_position"] = float(new_position)
                                    else:
                                        params["cup_position"] = float(new_position)
                                
                                # Update nested params.position.cup_position if exists
                                if "position" in params and isinstance(params["position"], dict):
                                    if "cup_position" in params["position"]:
                                        params["position"]["cup_position"] = float(new_position)
                                        logger.info(f"✅ Updated params['position']['cup_position'] to {new_position}")
                                
                                logger.info(f"✅ Cup position updated successfully for cup {cup_id}")
                                logger.info(f"📦 Updated task_item['ingredients']: {task_item['ingredients']}")
                                logger.info(f"🔗 Ingredients reference updated - changes will persist to next step")
                                logger.info(f"📦 Current params after update: {params}")
                            else:
                                logger.info(f"✓ Cup position {current_position} is available, no change needed")
                        else:
                            logger.warning(f"⚠️ Could not extract cup_position from ingredients: {ingredients}")
                
                if not res.get("passed", False):
                    message = f"Validation failed: {res.get('details', '')}"
                    await publish_event("validation.failed", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "reason": res}, rabbitmq_client)
                    success = False
                    break  # abort on validation failure
                    
            elif step_type == "robot":
                # Add retry logic for robot actions to handle transient failures during parallel execution
                max_retries = 2
                retry_delay = 3  # seconds
                
                for attempt in range(max_retries):
                    res = await call_robot(func_name, params, arm_id=arm_id, rabbitmq_client=rabbitmq_client)
                    
                    if res.get("success", False):
                        break  # Success, exit retry loop
                    
                    # Check if it's a transient error (timeout, connection issues)
                    error_msg = res.get('message', '').lower()
                    is_transient = any(keyword in error_msg for keyword in ['timeout', 'connection', 'unhealthy', 'health check'])
                    
                    if is_transient and attempt < max_retries - 1:
                        logger.warning(f"[ARM-{arm_id}] Transient error on {func_name} (attempt {attempt + 1}/{max_retries}): {error_msg}")
                        logger.info(f"[ARM-{arm_id}] Retrying in {retry_delay}s...")
                        await asyncio.sleep(retry_delay)
                        continue
                    else:
                        # Non-transient error or final retry failed
                        message = f"Robot error: {res.get('message', '')}"
                        logger.error(f"[ARM-{arm_id}] Robot step failed after {attempt + 1} attempts: {func_name}")
                        await publish_event("robot.error", 
                                    {"arm": arm_id, "cup": cup_id,
                                    "step": func_name, "error": res.get("message", "")}, rabbitmq_client)
                        success = False
                        break  # abort on robot error
                
                if not res.get("success", False):
                    break  # Exit step loop if robot action ultimately failed
                    
            elif step_type == "automation":
                logger.info(f"🤖 [ROUTINE] Processing automation step: {func_name} for cup {cup_id}")
                res = await call_automation(func_name, params, rabbitmq_client)
                if not res.get("success", False):
                    message = f"Automation error: {res.get('message', '')}"
                    logger.error(f"❌ [ROUTINE] Automation step failed: {func_name} - {message}")
                    await publish_event("automation.error", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "error": res.get("message", "")}, rabbitmq_client)
                    success = False
                    break  # abort on automation error
                else:
                    logger.info(f"✅ [ROUTINE] Automation step completed: {func_name} for cup {cup_id}")
                    
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
