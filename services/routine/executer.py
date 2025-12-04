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

# Resource lock for cup_station functions (prevents arm collisions)
cup_station_lock = asyncio.Lock()
cup_station_lock_holder = None

# RESOURCE LOCK RETRY CONFIGURATION
# These settings ensure strict queue adherence - arms will retry instead of skipping tasks
RESOURCE_LOCK_RETRY_INTERVAL = 1  # seconds between retry attempts
RESOURCE_LOCK_MAX_WAIT_TIME = 300  # maximum wait time (5 minutes) before timeout

# ORDER STOP TRACKING
# Track which orders have been stopped to allow graceful pausing of task execution
stopped_orders = set()  # Set of stopped order_ids
stopped_orders_lock = asyncio.Lock()  # Lock for thread-safe access to stopped_orders

# PARTIAL TASK PROGRESS TRACKING
# Track completed sub-steps for tasks to enable resume from exact sub-step
# Format: {f"{cup_id}-{function}": last_completed_step_index}
task_progress = {}  # Dict mapping task_key to last completed step index
task_progress_lock = asyncio.Lock()  # Lock for thread-safe access to task_progress

# VALIDATION FAILURE TRACKING
# Track which cup_id-function combinations have already triggered validation failure handling
# This prevents duplicate failure processing if validation is called multiple times
# Format: set of f"{cup_id}-{function}" strings
validation_failures_in_progress = set()
validation_failures_lock = asyncio.Lock()

# Configure logging
logger = logging.getLogger(__name__)

# Remove the global client - we'll use the one passed from the main service
# rabbitmq_client = RabbitMQClient("routine_executor")

async def mark_order_stopped(order_id: int):
    """Mark an order as stopped."""
    async with stopped_orders_lock:
        stopped_orders.add(order_id)
        log("INFO", f"Order {order_id} marked as stopped in routine", service="routine")

async def mark_order_resumed(order_id: int):
    """Mark an order as resumed (no longer stopped)."""
    async with stopped_orders_lock:
        stopped_orders.discard(order_id)
        log("INFO", f"Order {order_id} marked as resumed in routine", service="routine")
    
    # Clear validation failure markers so the order can retry validations
    await clear_order_validation_failures(order_id)

async def is_order_stopped(order_id: int) -> bool:
    """Check if an order is stopped."""
    async with stopped_orders_lock:
        return order_id in stopped_orders

async def get_task_progress(cup_id: str, function: str) -> int:
    """Get the last completed step index for a task. Returns -1 if no progress."""
    task_key = f"{cup_id}-{function}"
    async with task_progress_lock:
        return task_progress.get(task_key, -1)

async def set_task_progress(cup_id: str, function: str, step_index: int):
    """Store the last completed step index for a task."""
    task_key = f"{cup_id}-{function}"
    async with task_progress_lock:
        task_progress[task_key] = step_index
        log("DEBUG", f"Task progress saved: {task_key} -> step {step_index}", service="routine")

async def clear_task_progress(cup_id: str, function: str):
    """Clear task progress when task completes successfully or fails permanently."""
    task_key = f"{cup_id}-{function}"
    async with task_progress_lock:
        if task_key in task_progress:
            del task_progress[task_key]
            log("DEBUG", f"Task progress cleared: {task_key}", service="routine")

async def clear_order_progress(order_id: int):
    """Clear all task progress for an order (when order completes/cancels)."""
    async with task_progress_lock:
        keys_to_remove = [key for key in task_progress.keys() if key.startswith(f"{order_id}-")]
        for key in keys_to_remove:
            del task_progress[key]
        if keys_to_remove:
            log("INFO", f"Cleared progress for {len(keys_to_remove)} tasks in order {order_id}", service="routine")

async def mark_validation_failure_in_progress(cup_id: str, function: str) -> bool:
    """
    Mark that a validation failure is being processed for a cup_id-function combination.
    Returns True if this is the first failure (should process), False if already in progress (skip).
    """
    failure_key = f"{cup_id}-{function}"
    async with validation_failures_lock:
        if failure_key in validation_failures_in_progress:
            log("WARNING", f"Validation failure already in progress for {failure_key}, skipping duplicate processing", service="routine")
            return False
        validation_failures_in_progress.add(failure_key)
        log("INFO", f"Marked validation failure in progress for {failure_key}", service="routine")
        return True

async def clear_validation_failure_in_progress(cup_id: str, function: str):
    """Clear validation failure in-progress marker when task completes or order stops."""
    failure_key = f"{cup_id}-{function}"
    async with validation_failures_lock:
        if failure_key in validation_failures_in_progress:
            validation_failures_in_progress.discard(failure_key)
            log("DEBUG", f"Cleared validation failure marker for {failure_key}", service="routine")

async def clear_order_validation_failures(order_id: int):
    """Clear all validation failure markers for an order (when order is resumed or cancelled)."""
    async with validation_failures_lock:
        keys_to_remove = [key for key in validation_failures_in_progress if key.startswith(f"{order_id}-")]
        for key in keys_to_remove:
            validation_failures_in_progress.discard(key)
        if keys_to_remove:
            log("INFO", f"Cleared {len(keys_to_remove)} validation failure markers for order {order_id}", service="routine")

def extract_order_id_from_cup_id(cup_id: str) -> int:
    """Extract order_id from cup_id format (order_id-cup_index)."""
    try:
        if '-' in cup_id:
            return int(cup_id.split('-')[0])
    except (ValueError, IndexError):
        pass
    return None

async def send_validation_failure_to_dashboard(func_name: str, cup_id: str, rabbitmq_client: RabbitMQClient):
    """Send validation failure key to dashboard (message mapping handled by dashboard)."""
    try:
        await publish_event("validation.failed.dashboard", {
            "validation_function": func_name,
            "cup_id": cup_id,
            "timestamp": datetime.now().isoformat()
        }, rabbitmq_client)
        log("INFO", f"Sent validation failure key to dashboard: {func_name} (cup: {cup_id})", service="routine")
    except Exception as e:
        log("ERROR", f"Error sending validation failure to dashboard: {str(e)}", service="routine")

async def revert_previous_step_and_stop(cup_id: str, current_action: str, rabbitmq_client: RabbitMQClient):
    """
    Revert the previous step and stop the order when validation fails.
    
    This function calls OMS to stop the order (not scheduler directly) so that:
    1. Order status is properly updated in database (STOPPING -> STOPPED)
    2. WebSocket events are broadcast to dashboard
    3. The same flow is followed as when the UI stop button is clicked
    
    Args:
        cup_id: The cup ID
        current_action: The current validation action that failed
        rabbitmq_client: RabbitMQ client for communication
    
    Returns:
        dict with success status
    """
    try:
        log("INFO", f"[VALIDATION FAILURE HANDLER] Starting revert and stop process for cup {cup_id}", service="routine")
        log("INFO", f"[VALIDATION FAILURE HANDLER] Current action that failed: {current_action}", service="routine")
        
        # First, revert the previous step
        log("INFO", f"[REVERT STEP] Sending revert request to scheduler for cup {cup_id}", service="routine")
        revert_response = await rabbitmq_client.send_request(
            target_service="scheduler",
            action="revert_previous_step",
            data={
                "cup_id": cup_id,
                "current_action": current_action
            },
            timeout=10
        )
        
        log("INFO", f"[REVERT STEP] Received response from scheduler: {revert_response}", service="routine")
        
        if revert_response and revert_response.get("success"):
            reverted_action = revert_response.get('reverted_action')
            log("INFO", f"[REVERT STEP] Successfully reverted previous step: {reverted_action}", service="routine")
        else:
            error_msg = revert_response.get('error', 'Unknown error') if revert_response else 'No response'
            log("WARNING", f"[REVERT STEP] Failed to revert previous step: {error_msg}", service="routine")
        
        # Get order_id from cup_id (format: order_id-cup_index)
        order_id = None
        if '-' in cup_id:
            try:
                order_id = int(cup_id.split('-')[0])
            except ValueError:
                log("WARNING", f"Could not parse order_id from cup_id: {cup_id}", service="routine")
        
        if order_id:
            # Stop the order via OMS (NOT scheduler directly)
            # This ensures proper status updates and event broadcasts
            log("INFO", f"[STOP ORDER] Extracted order_id {order_id} from cup_id {cup_id}", service="routine")
            log("INFO", f"[STOP ORDER] Sending stop request to OMS for order {order_id}", service="routine")
            log("INFO", f"[STOP ORDER] OMS will handle status updates and call scheduler", service="routine")
            
            stop_response = await rabbitmq_client.send_request(
                target_service="oms",
                action="stop_order",
                data={
                    "order_id": order_id
                },
                timeout=30  # Reduced from 120s - stop process is now much faster with immediate task pause notifications
            )
            
            log("INFO", f"[STOP ORDER] Received response from OMS: {stop_response}", service="routine")
            
            if stop_response and stop_response.get("success"):
                log("INFO", f"[STOP ORDER] Successfully stopped order {order_id} via OMS", service="routine")
                log("INFO", f"[STOP ORDER] Order status updated in database, events broadcast to dashboard", service="routine")
            else:
                error_msg = stop_response.get('error', 'Unknown error') if stop_response else 'No response'
                log("WARNING", f"[STOP ORDER] Failed to stop order {order_id} via OMS: {error_msg}", service="routine")
        else:
            log("WARNING", f"[STOP ORDER] Could not extract order_id from cup_id: {cup_id}", service="routine")
        
        log("INFO", f"[VALIDATION FAILURE HANDLER] Completed revert and stop process for cup {cup_id}", service="routine")
        return {"success": True}
        
    except Exception as e:
        log("ERROR", f"[VALIDATION FAILURE HANDLER] Error reverting step and stopping order: {str(e)}", service="routine")
        return {"success": False, "error": str(e)}

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
    
    max_retries = 20
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
    
    Returns:
        dict: {
            "success": bool - Whether task completed successfully,
            "validation_failed_stopped": bool - Whether stopped due to validation failure,
            "order_stopped": bool - Whether stopped due to order stop request,
            "message": str - Error/status message
        }
    """
    global cup_station_lock_holder
    cup_id = task.get("item", {}).get("cup_id", "unknown")
    function = task.get("function")
    success = True
    message = ""
    validation_failed_stopped = False  # Flag to track if we stopped due to validation failure
    order_stopped = False  # Flag to track if order was stopped during execution
    
    # Extract order_id from cup_id for stop checking
    order_id = extract_order_id_from_cup_id(cup_id)
    if not order_id:
        log("WARNING", f"Could not extract order_id from cup_id {cup_id}, stop checking disabled", service="routine")
    
    # Add small staggered delay for Arm 2 to prevent RabbitMQ overload when both arms start simultaneously
    if arm_id == 2:
        log("INFO", "[ARM-2] Adding 1s stagger delay to prevent parallel connection overload", service="routine")
        await asyncio.sleep(1)
    
    # Check if order is stopped before starting
    if order_id and await is_order_stopped(order_id):
        log("INFO", f"Order {order_id} is stopped, task {function} for cup {cup_id} will not execute", service="routine")
        return {
            "success": False,
            "validation_failed_stopped": False,
            "order_stopped": True,
            "message": "Order stopped before task execution"
        }
    
    log("INFO", f"Processing task: {function} for cup {cup_id} on arm {arm_id}", service="routine")
    log("DEBUG", f"Task structure: {json.dumps(task, indent=2)}", service="routine")
    
    try:
        cfg = configs[function]
        # Get reference to task's ingredients (not a copy) so updates persist
        task_item = task.get("item", {})
        if "ingredients" not in task_item:
            task_item["ingredients"] = {}
        
        log("DEBUG", f"Task ingredients: {json.dumps(task_item.get('ingredients', {}), indent=2)}", service="routine")
        
        # Check for partial progress - resume from where we left off
        last_completed_step = await get_task_progress(cup_id, function)
        start_step_index = last_completed_step + 1  # Start from next step after last completed
        
        if start_step_index > 0:
            log("INFO", f"[RESUME] Found partial progress for {function} on cup {cup_id}, resuming from step {start_step_index}", service="routine")
        
        for step_index, step in enumerate(cfg["steps"]):
            # Skip steps that were already completed in a previous execution
            if step_index < start_step_index:
                func_name = step.get("function", "unknown")
                log("INFO", f"[RESUME] Skipping already completed step {step_index}: {func_name}", service="routine")
                continue
            # Check if order has been stopped before executing each step
            if order_id and await is_order_stopped(order_id):
                log("INFO", f"[STOP DETECTED] Order {order_id} stopped during task execution at step {step.get('function')}", service="routine")
                log("INFO", f"[STOP DETECTED] Progress saved - will resume from step {step_index} on order resume", service="routine")
                order_stopped = True
                success = False
                message = "Order stopped during task execution"
                # Progress is already saved from previous completed steps
                break  # Exit step loop - will skip feedback so task remains pending
            
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
                log("INFO", f"[VALIDATION STEP] Starting validation: {func_name} for cup {cup_id}", service="routine")
                log("DEBUG", f"[VALIDATION STEP] Validation params: {json.dumps(params)}", service="routine")
                res = await call_validation(func_name, params, rabbitmq_client, cup_id=cup_id)
                log("INFO", f"[VALIDATION STEP] Validation result for {func_name}: passed={res.get('passed', False)}", service="routine")
                
                # Debug: Log func_name and type for troubleshooting
                log("INFO", f"[DEBUG] Checking func_name='{func_name}' (type: {type(func_name).__name__}) against 'cup_detection'", service="routine")
                
                # Special handling for cup_detection - check station availability FIRST
                if func_name == "cup_detection":
                    log("INFO", f"[CUP DETECTION] ✅ Entered cup_detection with results=  {res}", service="routine")
                    # Get detection_result from top level first, then from details
                    detection_result = res.get("detection_result") or res.get("details", {}).get("cups_detected", {})
                    log("INFO", f"[CUP DETECTION] Detection result: {detection_result}", service="routine")
                    
                    if not detection_result:
                        # No detection result means detection failed
                        log("ERROR", f"[CUP DETECTION] No detection result returned for cup {cup_id}", service="routine")
                        res["passed"] = False
                        res["error"] = "Cup detection failed - no result"
                    else:
                        # Check if ALL stations are occupied (all values are True)
                        # detection_result format: {0: bool, 1: bool, 2: bool, 3: bool} where True = occupied, False = available
                        all_occupied = all(detection_result.values())
                        
                        if all_occupied:
                            log("ERROR", f"[ALL STATIONS OCCUPIED] All cup stations occupied for cup {cup_id}. Cannot proceed with task.", service="routine")
                            message = "All cup stations are occupied. Please remove cups and try again."
                            
                            # Send dashboard message for all stations occupied
                            log("INFO", f"[ALL STATIONS OCCUPIED] Sending dashboard notification for {func_name}", service="routine")
                            await send_validation_failure_to_dashboard(func_name, cup_id, rabbitmq_client)
                            
                            await publish_event("validation.failed", 
                                        {"arm": arm_id, "cup": cup_id,
                                        "step": func_name, "reason": "all_stations_occupied"}, rabbitmq_client)
                            
                            # Mark validation as failed
                            res["passed"] = False
                            res["error"] = "All cup stations occupied"
                        else:
                            # At least one station is available - update position to nearest available
                            log("INFO", f"[CUP DETECTION] Stations available. Proceeding with position update.", service="routine")
                            
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
                                log("ERROR", f"Could not extract cup_position from task ingredients for cup {cup_id}", service="routine")
                
                if not res.get("passed", False):
                    # Validation failed - check if we should process this failure or if it's already being handled
                    log("ERROR", f"[VALIDATION FAILED] Validation {func_name} failed for cup {cup_id}", service="routine")
                    log("ERROR", f"[VALIDATION FAILED] Failure details: {res.get('details', '')}", service="routine")
                    
                    # Use guard to prevent duplicate processing of same validation failure
                    should_process = await mark_validation_failure_in_progress(cup_id, function)
                    
                    if not should_process:
                        # This validation failure is already being processed, skip duplicate handling
                        log("WARNING", f"[VALIDATION FAILED] Duplicate validation failure detected for {function} on cup {cup_id}, skipping duplicate processing", service="routine")
                        validation_failed_stopped = True
                        break  # Exit without processing again
                    
                    log("INFO", f"[VALIDATION FAILED] Initiating validation failure handler sequence", service="routine")
                    
                    # Send dashboard message based on validation function mapping
                    log("INFO", f"[VALIDATION FAILED] Sending dashboard notification for {func_name}", service="routine")
                    await send_validation_failure_to_dashboard(func_name, cup_id, rabbitmq_client)
                    
                    # Publish validation failed event
                    log("INFO", f"[VALIDATION FAILED] Publishing validation.failed event", service="routine")
                    await publish_event("validation.failed", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "reason": res}, rabbitmq_client)
                    
                    # Stop the order and mark task for sub-step resume
                    # This calls scheduler to:
                    # 1. Mark CURRENT task (this one) as "pending" (will retry on resume)
                    # 2. Stop the order via OMS
                    # When order is resumed, scheduler will resubmit THIS task
                    # Routine will resume from the exact sub-step that failed (using saved progress)
                    log("INFO", f"[VALIDATION FAILED] Starting revert and stop process for task: {function}", service="routine")
                    await revert_previous_step_and_stop(cup_id, function, rabbitmq_client)
                    
                    # Set flag to prevent sending feedback at the end
                    # No feedback = scheduler keeps current task as "pending" (set by revert)
                    # Progress is preserved so we can resume from this exact sub-step
                    validation_failed_stopped = True
                    log("INFO", f"[VALIDATION FAILED] Set validation_failed_stopped flag to True", service="routine")
                    
                    # Break from step loop to stop execution
                    log("INFO", f"[VALIDATION FAILED] Breaking from step loop - recipe execution stopped for cup {cup_id}", service="routine")
                    log("INFO", f"[VALIDATION FAILED] Task {function} will resume from step {step_index} (checkpoint saved)", service="routine")
                    break  # Stop execution - no feedback sent, task remains pending for sub-step resume
                    
            elif step_type == "robot":
                # Check if this is a cup_station function that requires mutual exclusion
                is_cup_station = "cup_station" in func_name
                
                # For cup_station functions, acquire lock with polling and retry logic
                if is_cup_station:
                    # STRICT QUEUE ADHERENCE: Retry to maintain queue order
                    # The arm will NOT skip this task - it will wait until the resource is available
                    lock_acquired = False
                    retry_count = 0
                    start_wait_time = asyncio.get_event_loop().time()
                    
                    log("INFO", f"[ARM-{arm_id}] 🔒 Attempting to acquire cup_station lock for {func_name} (cup {cup_id})", service="routine")
                    
                    while not lock_acquired:
                        if not cup_station_lock.locked():
                            # Try to acquire the lock (non-blocking check, then acquire)
                            # Since we just checked it's not locked, acquire should succeed immediately
                            # but we use acquire() which will wait if another arm got it first
                            await cup_station_lock.acquire()
                            cup_station_lock_holder = arm_id
                            lock_acquired = True
                            
                            if retry_count > 0:
                                wait_duration = asyncio.get_event_loop().time() - start_wait_time
                                log("INFO", f"[ARM-{arm_id}] ✅ Acquired cup_station lock for {func_name} after {retry_count} retries ({wait_duration:.1f}s)", service="routine")
                            else:
                                log("INFO", f"[ARM-{arm_id}] ✅ Acquired cup_station lock for {func_name} (no wait)", service="routine")
                        else:
                            # Lock is held by another arm, retry with exponential backoff notification
                            retry_count += 1
                            current_holder = cup_station_lock_holder
                            elapsed_time = asyncio.get_event_loop().time() - start_wait_time
                            
                            # Log every 5 retries to avoid spam, but always log first retry
                            if retry_count == 1 or retry_count % 5 == 0:
                                log("INFO", f"[ARM-{arm_id}] ⏳ Retry {retry_count}: Waiting for cup_station lock (held by Arm {current_holder}) - {func_name} on cup {cup_id} ({elapsed_time:.1f}s elapsed)", service="routine")
                            
                            # Check if we've exceeded maximum wait time
                            if elapsed_time > RESOURCE_LOCK_MAX_WAIT_TIME:
                                log("ERROR", f"[ARM-{arm_id}] ❌ Cup station lock timeout after {retry_count} retries ({elapsed_time:.1f}s) for {func_name} on cup {cup_id}", service="routine")
                                message = f"Resource lock timeout: cup_station unavailable after {elapsed_time:.1f}s"
                                success = False
                                break  # Exit retry loop and fail this step
                            
                            # Wait before retry (strict queue order - keep retrying)
                            await asyncio.sleep(RESOURCE_LOCK_RETRY_INTERVAL)
                    
                    # If lock acquisition failed (timeout), skip robot execution
                    if not lock_acquired:
                        log("ERROR", f"[ARM-{arm_id}] Failed to acquire cup_station lock for {func_name}, skipping robot execution", service="routine")
                        await publish_event("robot.error", 
                                    {"arm": arm_id, "cup": cup_id,
                                    "step": func_name, "error": "Cup station lock timeout"}, rabbitmq_client)
                        break  # Exit step loop
                
                try:
                    # Add retry logic for robot actions to handle transient failures during parallel execution
                    max_retries = 20
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
            
            # Save progress after successful step completion
            # This allows resuming from this point if order is stopped
            await set_task_progress(cup_id, function, step_index)
                        
    except Exception as e:
        success = False
        message = f"Exception in routine: {str(e)}"
        log("ERROR", f"Error processing task: {e}", service="routine")
    
    # Send feedback to scheduler (skip if validation failed OR order stopped)
    if not validation_failed_stopped and not order_stopped:
        log("INFO", f"Sending feedback to scheduler: {function} for cup {cup_id} - {'SUCCESS' if success else 'FAILED'}", service="routine")
        await send_feedback_to_scheduler(cup_id, function, success, rabbitmq_client, message)
        
        # Clear progress on completion or permanent failure (task won't retry)
        await clear_task_progress(cup_id, function)
        
        # Clear validation failure marker on completion
        await clear_validation_failure_in_progress(cup_id, function)
        
        # all steps done
        if success:
            log("INFO", f"Task completed successfully: {function} for cup {cup_id}", service="routine")
            await publish_event("routine.completed", 
                        {"arm": arm_id, "cup": cup_id, "function": function}, rabbitmq_client)
        else:
            log("ERROR", f"Task failed: {function} for cup {cup_id} - {message}", service="routine")
    elif validation_failed_stopped:
        # Keep progress - scheduler will mark this task as pending for retry
        # When order resumes, THIS task will be retried from the saved checkpoint
        log("INFO", f"Skipping feedback for cup {cup_id} - validation failed, task will resume from checkpoint on order resume", service="routine")
        log("INFO", f"Progress preserved - next execution will skip completed sub-steps", service="routine")
        # NOTE: Don't clear validation failure marker here - it stays until order is resumed/cancelled
        
        # CRITICAL: Notify scheduler immediately that this task is paused (not actively running)
        # This prevents scheduler from waiting 90 seconds polling for this "submitted" task
        try:
            log("INFO", f"Notifying scheduler that task {function} is paused for cup {cup_id}", service="routine")
            await rabbitmq_client.send_request(
                target_service="scheduler",
                action="task_paused",
                data={
                    "cup_id": cup_id,
                    "function": function,
                    "reason": "validation_failed",
                    "timestamp": datetime.now().isoformat()
                },
                timeout=5
            )
        except Exception as e:
            log("WARNING", f"Failed to notify scheduler about paused task: {str(e)[:100]}", service="routine")
    elif order_stopped:
        # Keep progress - task will resume from where it stopped
        log("INFO", f"[STOP DETECTED] Skipping feedback for cup {cup_id} - order stopped, task remains in submitted state for resume", service="routine")
        log("INFO", f"[STOP DETECTED] Progress preserved - will resume from saved checkpoint on order resume", service="routine")
        # Clear validation failure marker when order stops normally (not due to validation failure)
        await clear_validation_failure_in_progress(cup_id, function)
        
        # CRITICAL: Notify scheduler immediately that this task is paused (not actively running)
        # This prevents scheduler from waiting 90 seconds polling for this "submitted" task
        try:
            log("INFO", f"Notifying scheduler that task {function} is paused for cup {cup_id}", service="routine")
            await rabbitmq_client.send_request(
                target_service="scheduler",
                action="task_paused",
                data={
                    "cup_id": cup_id,
                    "function": function,
                    "reason": "order_stopped",
                    "timestamp": datetime.now().isoformat()
                },
                timeout=5
            )
        except Exception as e:
            log("WARNING", f"Failed to notify scheduler about paused task: {str(e)[:100]}", service="routine")
    
    # Return task execution status
    return {
        "success": success,
        "validation_failed_stopped": validation_failed_stopped,
        "order_stopped": order_stopped,
        "message": message
    }
