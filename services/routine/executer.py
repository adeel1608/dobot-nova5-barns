# services/routine/executor.py
# from validation_client import call_validation
# from robot_client import call_robot
# from event_publisher import publish_event

import os
import sys
import logging
from datetime import datetime
import asyncio
import json

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.logger import log
from shared.rabbitmq_client import RabbitMQClient
cup_station_lock = asyncio.Lock()
cup_station_lock_holder = None

# Configure logging
logger = logging.getLogger(__name__)

# Remove the global client - we'll use the one passed from the main service
# rabbitmq_client = RabbitMQClient("routine_executor")

async def call_validation(func_name: str, params: dict, rabbitmq_client: RabbitMQClient, cup_id: str = None):
    """
    Calls the validation service with the given function name and parameters.
    The function name is sent directly as the action to match validation service handlers.
    """
    try:
        # Build request_id with cup_id if provided for better tracking
        if cup_id:
            request_id = f"routine-{cup_id}-{datetime.now().timestamp()}"
        else:
            request_id = f"routine-{datetime.now().timestamp()}"
        
        # Prepare payload with metadata expected by validation service
        payload = {
            "request_id": request_id,
            "client_type": "routine",
            "cup_id": cup_id,  # Include cup_id in payload for validation service
            **params  # Merge any additional params
        }
        
        log("INFO", f"Calling validation service: action={func_name}, payload={payload}", service="routine")
        
        # Set timeout based on validation type
        # cup_detection needs longer timeout to allow for retries when all stations occupied
        # (validation retries every 10s, so we allow ~10 retries = 100s + buffer)
        timeout = 120 
        
        # Send function name as action directly (e.g., "cup_detection", "check_coffee_beans")
        # The validation service has handlers registered for specific actions, not a generic "validate"
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action=func_name,  # Send function name directly as action
            data=payload,  # Send payload with metadata
            timeout=timeout
        )
        
        log("INFO", f"Validation service response: {response}", service="routine")
        
        if response.get("error"):
            log("ERROR", f"Validation service error: {response['error']}", service="routine")
            return {"passed": False, "details": f"Validation service error: {response['error']}"}
        
        return response
        
    except Exception as e:
        log("ERROR", f"Error calling validation service: {str(e)}", service="routine")
        return {"passed": False, "details": f"Error calling validation service: {str(e)}"}

async def call_automation(func_name: str, params: dict, rabbitmq_client: RabbitMQClient):
    """
    Calls the automation service with the given function name and parameters.
    """
    try:
        log("INFO", f"Sending automation request: function='{func_name}', params={params}", service="routine")
        
        response = await rabbitmq_client.send_request(
            target_service="automation",
            action="automate",
            data={
                "function": func_name,
                "params": params
            },
            timeout=80  # 80 second timeout for automation functions
        )
        
        log("INFO", f"Automation response received for {func_name}", service="routine")
        
        if response.get("error"):
            log("ERROR", f"Automation returned error for {func_name}: {response.get('error', '')[:50]}", service="routine")
            return {"success": False, "message": f"Automation service error: {response['error']}"}
        
        log("INFO", f"Automation completed successfully: {func_name}", service="routine")
        return response
        
    except Exception as e:
        log("ERROR", f"Automation call exception for {func_name}: {str(e)[:100]}", service="routine")
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
            log("ERROR", f"Robot service error: {response['error']}", service="routine")
            return {"success": False, "message": f"Robot service error: {response['error']}"}
        
        return response
        
    except Exception as e:
        log("ERROR", f"Error calling robot service: {str(e)}", service="routine")
        return {"success": False, "message": f"Error calling robot service: {str(e)}"}

async def publish_event(event_name: str, data: dict, rabbitmq_client: RabbitMQClient):
    """
    Publishes an event to the event bus via RabbitMQ.
    """
    try:
        await rabbitmq_client.send_event(event_name, data)
        log("DEBUG", f"Published event: {event_name}", service="routine")
    except Exception as e:
        log("ERROR", f"Error publishing event {event_name}: {str(e)}", service="routine")

def find_nearest_available_position(current_position: int, detection_result: dict) -> int:
    """
    Find the nearest available (False) cup position to the current position.
    
    Args:
        current_position: The originally intended cup position in 0-indexed format (0, 1, 2, 3)
        detection_result: Dict with 0-indexed position as key and occupancy as value 
                         (True = occupied/cup present, False = available/no cup)
    
    Returns:
        The nearest available position number (0-indexed)
    """
    # Ensure current_position is an integer
    current_position = int(current_position)
    
    # Get all available positions (False values = no cup = available) and ensure they're integers
    available_positions = [int(pos) for pos, occupied in detection_result.items() if not occupied]
    
    # Build detection breakdown string (avoid nested f-string syntax issues)
    # Convert pos to int in case detection_result has string keys
    detection_breakdown = ', '.join([f'Pos {int(pos)} (Station {int(pos)+1}): {"OCCUPIED" if occ else "AVAILABLE"}' for pos, occ in sorted(detection_result.items(), key=lambda x: int(x[0]))])
    log("INFO", f"Detection result breakdown: {detection_breakdown}", service="routine")
    log("INFO", f"Available positions: {[f'Pos {p} (Station {p+1})' for p in sorted(available_positions)]}", service="routine")
    
    if not available_positions:
        log("ERROR", f"No available cup positions found! All stations occupied: {detection_result}", service="routine")
        return current_position  # Return original if none available
    
    # If current position is available, use it
    if current_position in available_positions:
        log("INFO", f"Position {current_position} (Station {current_position + 1}) is available, no change needed", service="routine")
        return current_position
    
    # Find nearest available position by calculating absolute distance
    nearest_position = min(available_positions, key=lambda pos: abs(pos - current_position))
    
    log("INFO", f"Position {current_position} (Station {current_position + 1}) is OCCUPIED. Using nearest available: Position {nearest_position} (Station {nearest_position + 1})", service="routine")
    
    return nearest_position

async def send_feedback_to_scheduler(cup_id: str, action: str, success: bool, rabbitmq_client: RabbitMQClient, message: str = ""):
    """Send feedback to scheduler with retry logic and fallback event notification."""
    log("INFO", f"Sending feedback to scheduler: {action} for cup {cup_id} - {'SUCCESS' if success else 'FAILED'}", service="routine")
    
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
            log("INFO", f"Sending feedback to scheduler (attempt {attempt + 1}/{max_retries}): {feedback_data}", service="routine")
            
            # Send feedback to scheduler
            response = await rabbitmq_client.send_request(
                target_service="scheduler",
                action="feedback",
                data=feedback_data,
                timeout=10  # Increased timeout to 10 seconds
            )
            
            if response and response.get("success"):
                log("INFO", f"Feedback sent to scheduler for {action} on cup {cup_id}: success", service="routine")
                return True
            else:
                log("ERROR", f"Scheduler returned error for feedback (attempt {attempt + 1}): {response}", service="routine")
                if attempt < max_retries - 1:
                    log("INFO", f"Retrying feedback in {retry_delay} seconds...", service="routine")
                    await asyncio.sleep(retry_delay)
                    continue
                else:
                    log("ERROR", f"Failed to send feedback after {max_retries} attempts: {response}", service="routine")
                    break
                    
        except asyncio.TimeoutError:
            log("ERROR", f"Timeout sending feedback to scheduler (attempt {attempt + 1}/{max_retries})", service="routine")
            if attempt < max_retries - 1:
                log("INFO", f"Retrying feedback in {retry_delay} seconds...", service="routine")
                await asyncio.sleep(retry_delay)
                continue
            else:
                log("ERROR", f"Failed to send feedback after {max_retries} timeout attempts", service="routine")
                break
                
        except ConnectionError as e:
            log("ERROR", f"Connection error sending feedback to scheduler (attempt {attempt + 1}/{max_retries}): {e}", service="routine")
            if attempt < max_retries - 1:
                log("INFO", f"Retrying feedback in {retry_delay} seconds...", service="routine")
                await asyncio.sleep(retry_delay)
                continue
            else:
                log("ERROR", f"Failed to send feedback after {max_retries} connection error attempts", service="routine")
                break
                
        except Exception as e:
            log("ERROR", f"Unexpected error sending feedback to scheduler (attempt {attempt + 1}/{max_retries}): {e}", service="routine")
            if attempt < max_retries - 1:
                log("INFO", f"Retrying feedback in {retry_delay} seconds...", service="routine")
                await asyncio.sleep(retry_delay)
                continue
            else:
                log("ERROR", f"Failed to send feedback after {max_retries} attempts due to error: {e}", service="routine")
                break
    
    # If all retries failed, use event-based fallback notification
    log("ERROR", f"All feedback retries failed. Using event-based fallback for {action} on cup {cup_id}", service="routine")
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
        log("INFO", f"Fallback event sent: {event_name} for {action} on cup {cup_id}", service="routine")
        return True
        
    except Exception as fallback_error:
        log("ERROR", f"Fallback event notification also failed for {action} on cup {cup_id}: {fallback_error}", service="routine")
        return False

async def process_task(arm_id: int, task, configs: dict, rabbitmq_client: RabbitMQClient):
    """
    Executes the configured steps for a high-level function on a given arm.
    """
    global cup_station_lock_holder
    cup_id = task.get("item", {}).get("cup_id", "unknown")
    function = task.get("function")
    success = True
    message = ""
    
    # Add small staggered delay for Arm 2 to prevent RabbitMQ overload when both arms start simultaneously
    if arm_id == 2:
        log("INFO", "[ARM-2] Adding 1s stagger delay to prevent parallel connection overload", service="routine")
        await asyncio.sleep(1)
    
    log("INFO", f"Processing task: {function} for cup {cup_id} on arm {arm_id}", service="routine")
    log("DEBUG", f"Task structure: {json.dumps(task, indent=2)}", service="routine")
    
    try:
        cfg = configs[function]
        # Get reference to task's ingredients (not a copy) so updates persist
        task_item = task.get("item", {})
        if "ingredients" not in task_item:
            task_item["ingredients"] = {}
        
        log("DEBUG", f"Task ingredients: {json.dumps(task_item.get('ingredients', {}), indent=2)}", service="routine")
        
        for step in cfg["steps"]:
            step_type = step["type"]
            func_name = step["function"]
            # Get reference to ingredients (not a copy) so updates persist across steps
            ingredients = task_item["ingredients"]
            # Create a copy for params to send to services
            params = dict(ingredients)
            
            log("INFO", f"Executing step: {func_name} ({step_type}) for cup {cup_id}", service="routine")
            
            # Log cup_position specifically for debugging
            if "position" in params and "cup_position" in params["position"]:
                log("INFO", f"Cup position for this step: {params['position']['cup_position']}", service="routine")
            elif "cup_position" in params:
                log("INFO", f"Cup position for this step: {params['cup_position']}", service="routine")
            else:
                log("INFO", "No cup_position found in params", service="routine")
            
            if step_type == "validation":
                log("INFO", f"Calling validation function '{func_name}' with params: {json.dumps(params)}", service="routine")
                res = await call_validation(func_name, params, rabbitmq_client, cup_id=cup_id)
                
                # Special handling for cup_detection - update cup position based on availability
                if func_name == "cup_detection" and res.get("passed", False):
                    # Check if all stations are occupied after retries
                    if res.get("all_stations_occupied") and res.get("retries_exhausted"):
                        log("ERROR", f"All cup stations occupied after retries for cup {cup_id}. Cannot proceed with task.", service="routine")
                        message = "All cup stations are occupied. Please remove cups and try again."
                        await publish_event("validation.failed", 
                                    {"arm": arm_id, "cup": cup_id,
                                    "step": func_name, "reason": "all_stations_occupied"}, rabbitmq_client)
                        success = False
                        break  # abort task
                    
                    # Try to get detection_result from top level first, then from details
                    detection_result = res.get("detection_result") or res.get("details", {}).get("cups_detected", {})
                    if detection_result:
                        log("INFO", f"Cup detection result: {detection_result}", service="routine")
                        # Get current cup_position from task ingredients (already retrieved on line 277)
                        current_position = None
                        
                        # Look for cup_position in ingredients (check multiple possible locations)
                        # Option 1: Direct key 'cup_position'
                        if "cup_position" in ingredients:
                            cup_pos_data = ingredients["cup_position"]
                            if isinstance(cup_pos_data, dict):
                                # Extract position value from nested dict: {'cup_position': 1.0}
                                # Handle case where nested value might be string, int, or float
                                nested_value = list(cup_pos_data.values())[0]
                                current_position = int(float(nested_value))  # float() handles strings like "1.0"
                            else:
                                # Handle any other type (int, float, str) with robust conversion
                                current_position = int(float(cup_pos_data))
                        # Option 2: Nested under 'position' key
                        elif "position" in ingredients and isinstance(ingredients["position"], dict):
                            if "cup_position" in ingredients["position"]:
                                cup_pos_value = ingredients["position"]["cup_position"]
                                # Handle any type (int, float, str) with robust conversion
                                current_position = int(float(cup_pos_value))
                        
                        if current_position:
                            log("INFO", f"Current cup position from task: {current_position} (1-indexed)", service="routine")
                            
                            # CRITICAL: Convert 1-indexed cup_position to 0-indexed for detection_result comparison
                            # Task uses: 1=Station1, 2=Station2, 3=Station3, 4=Station4 (1-indexed)
                            # Detection returns: 0=Station1, 1=Station2, 2=Station3, 3=Station4 (0-indexed)
                            current_position_0indexed = current_position - 1
                            log("INFO", f"Converted to 0-indexed for detection comparison: {current_position_0indexed}", service="routine")
                            
                            # Find nearest available position (using 0-indexed)
                            new_position_0indexed = find_nearest_available_position(current_position_0indexed, detection_result)
                            # Convert back to 1-indexed for task storage
                            new_position = new_position_0indexed + 1
                            
                            # Update task params with new position if it changed
                            if new_position != current_position:
                                log("INFO", f"📍 Position changed: Station {current_position} → Station {new_position} (task uses 1-indexed)", service="routine")
                                
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
                                        log("INFO", "Success", service="routine")
                                
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
                                        log("INFO", "Success", service="routine")
                                
                                log("INFO", "Success", service="routine")
                                log("INFO", f"Updated task_item['ingredients']: {json.dumps(task_item['ingredients'])}", service="routine")
                                log("DEBUG", "Linking", service="routine")
                                log("INFO", f"Current params after update: {json.dumps(params)}", service="routine")
                                
                                # CRITICAL: Notify scheduler about position change so ALL future tasks use updated position
                                try:
                                    log("INFO", f"Notifying scheduler about position change for cup {cup_id}: {current_position} → {new_position}", service="routine")
                                    position_update_response = await rabbitmq_client.send_request(
                                        target_service="scheduler",
                                        action="update_cup_position",
                                        data={
                                            "cup_id": cup_id,
                                            "new_position": float(new_position),
                                            "old_position": float(current_position),
                                            "timestamp": datetime.now().isoformat()
                                        },
                                        timeout=5
                                    )
                                    if position_update_response and position_update_response.get("success"):
                                        log("INFO", f"Scheduler acknowledged position update for cup {cup_id} to {new_position}", service="routine")
                                    else:
                                        log("ERROR", f"Scheduler failed to update position for cup {cup_id}: {position_update_response.get('error', 'Unknown')[:50]}", service="routine")
                                except Exception as e:
                                    log("ERROR", f"Scheduler position update exception for cup {cup_id}: {str(e)[:100]}", service="routine")
                            else:
                                log("INFO", f"Station {current_position} (1-indexed) is available for cup {cup_id}, no position change needed", service="routine")
                        else:
                            log("ERROR", f"Invalid position detected in validation response for cup {cup_id}", service="routine")
                
                if not res.get("passed", False):
                    message = f"Validation failed: {res.get('details', '')}"
                    await publish_event("validation.failed", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "reason": res}, rabbitmq_client)
                    success = False
                    break  # abort on validation failure
                    
            elif step_type == "robot":
                # Check if this is a cup_station function that requires mutual exclusion
                is_cup_station = "cup_station" in func_name
                
                # For cup_station functions, acquire lock with polling
                if is_cup_station:
                    # Poll every second until lock is available
                    lock_acquired = False
                    while not lock_acquired:
                        if not cup_station_lock.locked():
                            # Try to acquire the lock (non-blocking check, then acquire)
                            # Since we just checked it's not locked, acquire should succeed immediately
                            # but we use acquire() which will wait if another arm got it first
                            await cup_station_lock.acquire()
                            cup_station_lock_holder = arm_id
                            lock_acquired = True
                            log("INFO", f"[ARM-{arm_id}] Acquired cup_station lock for function: {func_name}", service="routine")
                        else:
                            # Lock is held by another arm, wait and check again
                            current_holder = cup_station_lock_holder
                            log("INFO", f"[ARM-{arm_id}] Waiting for cup_station lock (currently held by Arm {current_holder}) for function: {func_name}", service="routine")
                            await asyncio.sleep(1)  # Check every second
                
                try:
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
                            log("ERROR", f"[ARM-{arm_id}] Transient error on {func_name} (attempt {attempt + 1}/{max_retries}): {error_msg}", service="routine")
                            log("INFO", f"[ARM-{arm_id}] Retrying in {retry_delay}s...", service="routine")
                            await asyncio.sleep(retry_delay)
                            continue
                        else:
                            # Non-transient error or final retry failed
                            message = f"Robot error: {res.get('message', '')}"
                            log("ERROR", f"[ARM-{arm_id}] Robot step failed after {attempt + 1} attempts: {func_name}", service="routine")
                            await publish_event("robot.error", 
                                        {"arm": arm_id, "cup": cup_id,
                                        "step": func_name, "error": res.get("message", "")}, rabbitmq_client)
                            success = False
                            break  # abort on robot error
                    
                    if not res.get("success", False):
                        break  # Exit step loop if robot action ultimately failed
                finally:
                    # Always release the lock for cup_station functions (only release if we acquired it)
                    if is_cup_station and cup_station_lock.locked() and cup_station_lock_holder == arm_id:
                        cup_station_lock_holder = None
                        cup_station_lock.release()
                        log("INFO", f"[ARM-{arm_id}] Released cup_station lock for function: {func_name}", service="routine")
                    
            elif step_type == "automation":
                log("INFO", "Action", service="routine")
                res = await call_automation(func_name, params, rabbitmq_client)
                if not res.get("success", False):
                    message = f"Automation error: {res.get('message', '')}"
                    log("ERROR", f"Automation execution failed for {func_name} on cup {cup_id}: {res.get('message', '')[:50]}", service="routine")
                    await publish_event("automation.error", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "error": res.get("message", "")}, rabbitmq_client)
                    success = False
                    break  # abort on automation error
                else:
                    log("INFO", "Success", service="routine")
                    
            # publish a step-completed event
            await publish_event("routine.step_completed", 
                        {"arm": arm_id, "cup": cup_id,
                        "step": func_name}, rabbitmq_client)
                        
    except Exception as e:
        success = False
        message = f"Exception in routine: {str(e)}"
        log("ERROR", f"Error processing task: {e}", service="routine")
    
    # Send feedback to scheduler
    log("INFO", f"Sending feedback to scheduler: {function} for cup {cup_id} - {'SUCCESS' if success else 'FAILED'}", service="routine")
    await send_feedback_to_scheduler(cup_id, function, success, rabbitmq_client, message)
    
    # all steps done
    if success:
        log("INFO", f"Task completed successfully: {function} for cup {cup_id}", service="routine")
        await publish_event("routine.completed", 
                    {"arm": arm_id, "cup": cup_id, "function": function}, rabbitmq_client)
    else:
        log("ERROR", f"Task failed: {function} for cup {cup_id} - {message}", service="routine")
