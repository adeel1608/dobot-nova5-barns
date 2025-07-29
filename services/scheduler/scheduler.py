import asyncio
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Any, Tuple, Optional
import httpx
import sys
import os

# Import logger from the volume-mounted data directory
# sys.path.insert(0, '/app/data')
# import logger

import logging
logger = logging.getLogger(__name__)

# Shared data structures for scheduling
tasks = []           # All pending tasks across orders
tasks_by_cup = {}    # Map cup_id -> list of all tasks for that cup (for completion tracking)
completed = {}       # Map cup_id -> set of completed action names for that cup
failed_tasks = []    # List of failed tasks
tasks_total = 0      # Total number of tasks scheduled
completed_count = 0  # Counter for tasks completed successfully
failed_count = 0     # Counter for tasks failed
lock = threading.Lock()  # Lock to synchronize access to shared data
current_status = {"order_id": None, "cup_index": None, "step": None, "status": "idle"}
status_callback = None  # Callback function to notify about status updates
order_completion_notified = False  # Flag to prevent duplicate completion notifications

# Global RabbitMQ client reference for notifications
_global_rabbitmq_client = None

def set_rabbitmq_client(client):
    """Set the global RabbitMQ client for use in notifications."""
    global _global_rabbitmq_client
    _global_rabbitmq_client = client
    logger.info("🔧 [SCHEDULER] Global RabbitMQ client set for notifications")

# Configuration for the routine service
ROUTINE_SERVICE_URL = "http://routine:8000"  # Can be overridden via environment variable

def load_recipes(recipe_file: str):
    """Load drink recipes from a JSON file."""
    with open(recipe_file, 'r') as f:
        return json.load(f)

def parse_orders(order_file: str):
    """Read and parse orders from a text file (drink name and cup ID per line)."""
    orders = []
    with open(order_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue  # skip blank lines or comments
            parts = line.split()
            if len(parts) >= 2:
                drink = parts[0]
                cup_id = parts[1]
                orders.append((drink, cup_id))
    return orders

def setup_tasks(orders, recipes):
    """Create task entries for each order based on the recipes."""
    global tasks, tasks_by_cup, completed, tasks_total, failed_tasks, failed_count
    tasks = []
    tasks_by_cup = {}
    completed = {}
    failed_tasks = []
    failed_count = 0
    tasks_total = 0

    for drink, cup_id in orders:
        # Check that the recipe exists
        if drink not in recipes:
            raise Exception(f"Recipe for drink '{drink}' not found.")
        recipe = recipes[drink]
        completed[cup_id] = set()
        tasks_by_cup[cup_id] = []
        # Validate that dependencies in recipe refer to valid actions
        valid_actions = {step["action"] for step in recipe}
        for step in recipe:
            if "depends_on" in step:
                deps = step["depends_on"]
                # Ensure depends_on is a list for consistency
                dep_list = deps if isinstance(deps, list) else [deps]
                for dep in dep_list:
                    if dep not in valid_actions:
                        raise Exception(
                            f"Invalid dependency '{dep}' in recipe for {drink}: no such step"
                        )
        # Create task dicts for each step in the recipe
        for step in recipe:
            action_name = step["action"]
            assigned_arm = step["assigned_arm"]
            dep_list = []
            if "depends_on" in step:
                deps = step["depends_on"]
                dep_list = deps if isinstance(deps, list) else [deps]
            task = {
                "drink": drink,
                "cup": cup_id,
                "action": action_name,
                "assigned_arm": assigned_arm,
                "depends_on": dep_list,
                "status": "pending"
            }
            tasks.append(task)
            tasks_by_cup[cup_id].append(task)
            tasks_total += 1

async def submit_task_to_routine(arm_id: str, function: str, cup_id: str, drink_type: str):
    """Submit a task to the routine service via RabbitMQ."""
    client = None
    try:
        # Import RabbitMQ client here to avoid circular imports
        import sys
        import os
        sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
        from shared.rabbitmq_client import RabbitMQClient
        
        # Create a temporary RabbitMQ client for sending tasks with unique ID
        import uuid
        client = RabbitMQClient(f"scheduler_task_submitter_{uuid.uuid4().hex[:8]}")
        await client.connect()
        
        # Create the payload for the routine service
        payload = {
            "arm_id": int(arm_id.replace("Arm", "")),  # Convert "Arm1" to 1
            "function": function,
            "item": {
                "cup_id": cup_id,
                "addons": []  # No addons for now, could be made configurable
            }
        }
        
        # Send request to routine service via RabbitMQ
        response = await client.send_request(
            target_service="routine",
            action="submit_task",
            data=payload,
            timeout=10
        )
        
        if response.get("success"):
            logger.info(f"✅ Task submitted to routine: {function} on arm {arm_id} for cup {cup_id}")
            return True
        else:
            logger.error(f"❌ Failed to submit task to routine: {response.get('error', 'Unknown error')}")
            return False
            
    except Exception as e:
        logger.error(f"❌ Error submitting task to routine: {str(e)}")
        return False
    finally:
        # Ensure client is disconnected even if an exception occurs
        if client:
            try:
                await client.disconnect()
            except Exception as disconnect_error:
                logger.warning(f"⚠️ Error disconnecting RabbitMQ client: {disconnect_error}")

async def arm_worker(arm_name: str):
    """Worker thread for a robotic arm that executes tasks when they are ready."""
    global completed_count, failed_count
    logger.info(f"🤖 DEBUG: {arm_name} worker started. Total tasks to process: {tasks_total}")
    
    consecutive_no_work_count = 0
    max_consecutive_no_work = 300  # Increased timeout for better reliability
    
    while True:
        task = None
        should_exit = False
        
        # Find a pending task for this arm with all dependencies satisfied
        with lock:
            # Exit when all tasks are either completed or failed (NOT just submitted)
            total_finished = completed_count + failed_count
            if total_finished >= tasks_total:
                logger.info(f"🤖 DEBUG: {arm_name} worker finished. Completed: {completed_count}, Failed: {failed_count}, Total: {tasks_total}")
                should_exit = True
            else:
                # Check for pending tasks that can be executed
                for t in tasks:
                    if t["assigned_arm"] == arm_name and t["status"] == "pending":
                        # Check if all dependencies for this task are completed
                        cup_id = t["cup"]
                        deps = t["depends_on"]
                        if all(dep in completed[cup_id] for dep in deps):
                            # Take this task for execution
                            t["status"] = "in_progress"
                            task = t
                            break
        
        if should_exit:
            break
            
        if task:
            consecutive_no_work_count = 0  # Reset counter when we have work
            
            # Perform the task by sending it to the routine service
            action = task["action"]
            cup_id = task["cup"]
            drink = task["drink"]
            
            # Update status and notify
            await update_status(f"Executing {action} for {drink} (cup {cup_id})")
            
            # Submit the task to the routine service
            success = await submit_task_to_routine(
                task["assigned_arm"], 
                action, 
                cup_id, 
                drink
            )
            
            if success:
                # Mark the task as submitted - completion will be handled by callback
                with lock:
                    task["status"] = "submitted"
                    logger.info(f"✅ Task {action} for cup {cup_id} successfully submitted to routine")
            else:
                # Mark the task as failed immediately
                with lock:
                    task["status"] = "failed"
                    failed_tasks.append(task)
                    failed_count += 1
                    logger.error(f"❌ Failed to submit task: {action} for cup {cup_id}")
        else:
            # No available task for this arm right now
            consecutive_no_work_count += 1
            
            # Check current state for debugging
            with lock:
                pending_tasks = [t for t in tasks if t["status"] == "pending"]
                submitted_tasks = [t for t in tasks if t["status"] == "submitted"]
                completed_tasks = [t for t in tasks if t["status"] == "done"]
                failed_tasks_status = [t for t in tasks if t["status"] == "failed"]
                
                if consecutive_no_work_count % 50 == 0:  # Log every 5 seconds
                    logger.info(f"🤖 DEBUG: {arm_name} waiting - Pending: {len(pending_tasks)}, Submitted: {len(submitted_tasks)}, Completed: {len(completed_tasks)}, Failed: {len(failed_tasks_status)}")
            
            # Prevent infinite waiting - if we've been waiting too long, check if we should exit
            if consecutive_no_work_count >= max_consecutive_no_work:
                with lock:
                    total_finished = completed_count + failed_count
                    if total_finished >= tasks_total:
                        logger.info(f"🤖 DEBUG: {arm_name} worker exiting after waiting - all tasks done")
                        break
                    
                    # Check if there are any tasks left that could potentially be processed
                    pending_tasks = [t for t in tasks if t["status"] == "pending"]
                    submitted_tasks = [t for t in tasks if t["status"] == "submitted"]
                    
                    if not pending_tasks and not submitted_tasks:
                        logger.info(f"🤖 DEBUG: {arm_name} worker exiting - no pending or submitted tasks left")
                        break
                        
                    # If there are submitted tasks, continue waiting for them to complete
                    if submitted_tasks:
                        logger.info(f"🤖 DEBUG: {arm_name} continuing to wait for {len(submitted_tasks)} submitted tasks to complete")
                        consecutive_no_work_count = 0  # Reset counter and continue waiting
                    elif not any(all(dep in completed[t["cup"]] for dep in t["depends_on"]) for t in pending_tasks if t["assigned_arm"] == arm_name):
                        # No tasks for this arm can be executed due to dependencies
                        logger.info(f"🤖 DEBUG: {arm_name} worker exiting - no executable tasks for this arm")
                        break
                    else:
                        consecutive_no_work_count = 0  # Reset and continue
            
            # Small delay to avoid busy waiting and yield control to event loop
            await asyncio.sleep(0.1)

def run(order_file: str = 'data/orders.txt', recipe_file: str = 'data/recipes.json'):
    """Load orders and recipes, then run the simulation."""
    recipes = load_recipes(recipe_file)
    orders = parse_orders(order_file)
    if not orders:
        print("No orders to process.")
        return
    setup_tasks(orders, recipes)
    logger.info(f"Starting coffee order simulation for {len(orders)} orders...")
    
    # Run with asyncio since workers are now async
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_async())
    
    logger.info("All orders completed.")

async def run_async():
    """Async version of the run function for running arm workers."""
    # Create tasks for both arms
    arm1 = asyncio.create_task(arm_worker("Arm1"))
    arm2 = asyncio.create_task(arm_worker("Arm2"))
    
    # Wait for both arms to finish all tasks
    await asyncio.gather(arm1, arm2)

# New functions for API integration

async def update_status(message: str):
    """Update current status and call the callback if set."""
    global current_status
    logger.info(message)
    if status_callback:
        if asyncio.iscoroutinefunction(status_callback):
            await status_callback(message)
        else:
            status_callback(message)

def register_status_callback(callback):
    """Register a callback function to be called on status updates."""
    global status_callback
    status_callback = callback

def setup_tasks_from_order(order_id: int, drinks: List[Dict[str, Any]], recipes: Dict[str, List[Dict[str, Any]]]):
    """Create task entries for an order received through the API."""
    global tasks, tasks_by_cup, completed, tasks_total, current_status, failed_tasks, failed_count, order_completion_notified, completed_count
    
    logger.info(f"[SCHEDULER] Setting up tasks for order {order_id} with {len(drinks)} drinks")
    
    # Reset ALL global variables for the new order to prevent race conditions
    tasks = []
    tasks_by_cup = {}
    completed = {}
    failed_tasks = []
    failed_count = 0
    completed_count = 0  # Reset completed count
    tasks_total = 0
    order_completion_notified = False  # Critical: Reset completion notification flag for new order
    
    # Update current status with new order info
    current_status.update({
        "order_id": order_id, 
        "status": "in_progress", 
        "step": "preparing",
        "cup_index": None
    })
    
    # Convert API drink format to scheduler format
    orders = []
    for idx, cup in enumerate(drinks, start=1):
        drink_type = cup.get("type")
        cup_id = f"{order_id}-{idx}"  # Create a unique cup ID
        orders.append((drink_type, cup_id))
        
        # Check if recipe exists for this drink
        if drink_type not in recipes:
            logger.warning(f"[SCHEDULER] No recipe found for drink type '{drink_type}'")
            return False
        else:
            logger.info(f"[SCHEDULER] Found recipe for '{drink_type}' with {len(recipes[drink_type])} steps")
    
    # Use existing setup_tasks logic
    try:
        setup_tasks(orders, recipes)
        logger.info(f"[SCHEDULER] Created {tasks_total} tasks for order {order_id}")
        return True
    except Exception as e:
        logger.error(f"[SCHEDULER] Error setting up tasks: {str(e)}")
        current_status.update({"status": "error", "step": str(e)})
        return False

async def process_order_async(order_id: int, drinks: List[Dict[str, Any]], recipes: Dict[str, List[Dict[str, Any]]]):
    """Process an order asynchronously using the scheduler."""
    global current_status, completed_count, tasks_total, failed_count, order_completion_notified
    
    logger.info(f"🔍 DEBUG: process_order_async called for order {order_id} with {len(drinks)} drinks")
    logger.info(f"🔍 DEBUG: Available recipes: {list(recipes.keys()) if recipes else 'No recipes loaded'}")
    logger.info(f"🔍 DEBUG: Recipe count: {len(recipes) if recipes else 0}")
    logger.info(f"🔍 DEBUG: Drinks to process: {drinks}")
    
    # Check if recipes is empty
    if not recipes:
        logger.error(f"❌ CRITICAL: No recipes available for order {order_id}")
        await notify_oms_completion(order_id, False, "No recipes available")
        return False
    
    # Setup tasks for the order
    try:
        setup_result = setup_tasks_from_order(order_id, drinks, recipes)
        logger.info(f"🔍 DEBUG: setup_tasks_from_order returned: {setup_result}")
        
        if not setup_result:
            logger.error(f"❌ Failed to setup tasks for order {order_id}")
            # Notify OMS about order failure
            await notify_oms_completion(order_id, False, "Failed to setup tasks for order")
            return False
    except Exception as e:
        logger.error(f"❌ Exception in setup_tasks_from_order for order {order_id}: {e}")
        import traceback
        logger.error(f"❌ Setup tasks exception traceback: {traceback.format_exc()}")
        await notify_oms_completion(order_id, False, f"Setup tasks exception: {str(e)}")
        return False
    
    logger.info(f"✅ Tasks setup successfully for order {order_id}. Total tasks: {tasks_total}")
    
    # Reset counters
    completed_count = 0
    
    # Start worker threads
    await update_status(f"Starting to process order {order_id}")
    
    try:
        # Create and start arm workers
        arm1 = asyncio.create_task(arm_worker("Arm1"))
        arm2 = asyncio.create_task(arm_worker("Arm2"))
        
        # Calculate dynamic timeout based on number of cups
        # Base timeout (2 minutes) + per-cup timeout (1 minute per cup)
        base_timeout = 120.0  # 2 minutes base
        per_cup_timeout = 400.0  # 1 minute per cup
        num_cups = len(drinks)
        dynamic_timeout = base_timeout + (per_cup_timeout * num_cups)
        
        logger.info(f"🕐 Order {order_id} timeout set to {dynamic_timeout:.0f} seconds ({dynamic_timeout/60:.1f} minutes) for {num_cups} cups")
        
        # Wait for both arms to finish all tasks with dynamic timeout
        try:
            await asyncio.wait_for(
                asyncio.gather(arm1, arm2, return_exceptions=True),
                timeout=dynamic_timeout
            )
            logger.info(f"✅ Both arm workers completed for order {order_id}")
        except asyncio.TimeoutError:
            logger.error(f"❌ Order {order_id} timed out after {dynamic_timeout/60:.1f} minutes")
            # Cancel both arms
            arm1.cancel()
            arm2.cancel()
            await notify_oms_completion(order_id, False, f"Order processing timed out after {dynamic_timeout/60:.1f} minutes")
            return False
        
        # Check if order was successful or failed
        with lock:
            if order_completion_notified:
                # Completion already notified by feedback handler
                logger.info(f"✅ [SCHEDULER] Order {order_id} completion already notified by feedback handler")
                return completed_count == tasks_total and failed_count == 0
            elif failed_count > 0:
                # Some tasks failed - mark order as failed
                current_status.update({"status": "error", "step": f"{failed_count} tasks failed"})
                await update_status(f"Order {order_id} failed: {failed_count} out of {tasks_total} tasks failed")
                
                # Create failure reason listing failed tasks
                failed_task_names = [f"{task['action']} ({task['cup']})" for task in failed_tasks]
                reason = f"Failed tasks: {', '.join(failed_task_names)}"
                
                # Notify OMS about order failure
                await notify_oms_completion(order_id, False, reason)
                order_completion_notified = True
                return False
            elif completed_count == tasks_total:
                # All tasks completed successfully
                current_status.update({"status": "completed", "step": None, "cup_index": None})
                await update_status(f"Order {order_id} completed successfully")
                
                # Notify OMS about order completion
                await notify_oms_completion(order_id, True)
                order_completion_notified = True
                return True
            else:
                # This shouldn't happen, but handle it as a failure
                reason = f"Unexpected state: {completed_count} completed, {failed_count} failed out of {tasks_total} total"
                current_status.update({"status": "error", "step": reason})
                await update_status(f"Order {order_id} failed: {reason}")
                
                # Notify OMS about order failure
                await notify_oms_completion(order_id, False, reason)
                order_completion_notified = True
                return False
                
    except Exception as e:
        # Notify OMS about order failure
        await notify_oms_completion(order_id, False, f"Order processing failed: {str(e)}")
        current_status.update({"status": "error", "step": str(e)})
        await update_status(f"Order {order_id} failed: {e}")
        return False

def get_current_status():
    """Get the current processing status."""
    return current_status

async def handle_routine_feedback(cup_id: str, action: str, success: bool):
    """
    Handle feedback from the routine service about task completion.
    
    When a task fails, immediately notifies OMS and cancels remaining tasks
    instead of waiting for all tasks to complete.
    """
    global completed_count, completed, failed_count, failed_tasks, order_completion_notified
    
    logger.info(f"[SCHEDULER] Processing feedback: {action} for {cup_id} - {'SUCCESS' if success else 'FAILED'}")
    
    try:
        # Variables to track what needs to be done outside the lock
        update_message = None
        should_check_order_completion = False
        should_notify_immediate_failure = False
        failed_task_info = None
        
        with lock:
            # Find the first matching task that is not yet completed/failed
            task_found = False
            for task in tasks:
                if task["cup"] == cup_id and task["action"] == action and task["status"] not in ["done", "failed"]:
                    task_found = True
                    
                    if success:
                        # Mark task as completed
                        task["status"] = "done"
                        completed[cup_id].add(action)
                        completed_count += 1
                        logger.info(f"[SCHEDULER] Task completed: {action} for {cup_id}")
                        
                        # Check if this was the final task for this cup
                        if len(completed[cup_id]) == len(tasks_by_cup[cup_id]):
                            update_message = f"Order complete: {task['drink']} for {cup_id}"
                    else:
                        # Mark task as failed
                        task["status"] = "failed"
                        failed_tasks.append(task)
                        failed_count += 1
                        logger.error(f"[SCHEDULER] Task failed: {action} for {cup_id}")
                        
                        # Set flag for immediate failure notification
                        if not order_completion_notified:
                            should_notify_immediate_failure = True
                            failed_task_info = task.copy()
                    
                    # Check if we should evaluate order completion after this task update
                    should_check_order_completion = True
                    break
            
            if not task_found:
                # Check if task was already completed
                if cup_id in completed and action in completed[cup_id]:
                    logger.warning(f"[SCHEDULER] Duplicate feedback ignored for {action} on {cup_id}")
                    return
                else:
                    logger.warning(f"[SCHEDULER] Task not found: {action} for {cup_id}")
        
        # Call async operations outside the lock to prevent blocking
        if update_message:
            await update_status(update_message)
        
        # Handle immediate failure notification
        if should_notify_immediate_failure and failed_task_info:
            with lock:
                order_id = current_status.get("order_id")
            
            if order_id:
                # Mark as notified to prevent duplicate notifications
                with lock:
                    order_completion_notified = True
                
                failed_action = failed_task_info['action']
                failed_cup = failed_task_info['cup']
                reason = f"Task failed: {failed_action} for {failed_cup}"
                
                with lock:
                    current_status.update({"status": "error", "step": f"Task {failed_action} failed"})
                
                await update_status(f"Order {order_id} failed: {reason}")
                logger.info(f"[SCHEDULER] Notifying OMS of order {order_id} failure")
                await notify_oms_completion(order_id, False, reason)
                
                # Cancel remaining tasks
                with lock:
                    for task in tasks:
                        if task["status"] not in ["done", "failed"]:
                            task["status"] = "cancelled"
        
        # Check for order completion
        elif should_check_order_completion and not order_completion_notified:
            logger.info(f"🔍 DEBUG: Calling check_and_notify_order_completion for order_id={current_status.get('order_id')}, should_check={should_check_order_completion}, notified={order_completion_notified}")
            await check_and_notify_order_completion()
        elif not task_found and not order_completion_notified:
            logger.info(f"🔍 DEBUG: Task not found, calling check_and_notify_order_completion for order_id={current_status.get('order_id')}, task_found={task_found}, notified={order_completion_notified}")
            await check_and_notify_order_completion()
        else:
            logger.info(f"🔍 DEBUG: Skipping completion check - should_check={should_check_order_completion}, task_found={task_found}, notified={order_completion_notified}")
        
    except Exception as e:
        logger.error(f"[SCHEDULER] Error in feedback processing: {e}")
        raise

async def check_and_notify_order_completion():
    """Check if the current order is complete and notify OMS if so."""
    global completed_count, failed_count, tasks_total, current_status, order_completion_notified
    
    with lock:
        # Count actual task statuses instead of relying on counters (more reliable)
        completed_tasks = sum(1 for task in tasks if task["status"] == "done")
        failed_tasks_count = sum(1 for task in tasks if task["status"] == "failed")
        cancelled_tasks = sum(1 for task in tasks if task["status"] == "cancelled")
        total_finished = completed_tasks + failed_tasks_count + cancelled_tasks
        
        order_id = current_status.get("order_id")
        
        logger.info(f"🔍 DEBUG: check_and_notify_order_completion called - order_id={order_id}, completed={completed_tasks}, failed={failed_tasks_count}, cancelled={cancelled_tasks}, total={tasks_total}, notified={order_completion_notified}")
        
        # Only proceed if we have an order_id and not already notified
        if not order_id or order_completion_notified:
            logger.info(f"🔍 DEBUG: Early return - no order_id ({not order_id}) or already notified ({order_completion_notified})")
            return
            
        # If not all tasks are finished and no failures, continue waiting
        if total_finished < tasks_total and failed_tasks_count == 0:
            logger.info(f"🔍 DEBUG: Waiting for more tasks - finished({total_finished}) < total({tasks_total}) and no failures")
            return
        
        if failed_tasks_count > 0:
            # Some tasks failed - notify failure
            failed_task_list = [task for task in tasks if task["status"] == "failed"]
            failed_task_names = [f"{task['action']} ({task['cup']})" for task in failed_task_list]
            reason = f"Failed tasks: {', '.join(failed_task_names)}"
            
            current_status.update({"status": "error", "step": f"{failed_tasks_count} tasks failed"})
            await update_status(f"Order {order_id} failed: {failed_tasks_count} out of {tasks_total} tasks failed")
            
            logger.info(f"[SCHEDULER] Notifying OMS of order {order_id} failure")
            await notify_oms_completion(order_id, False, reason)
            order_completion_notified = True
            
        elif completed_tasks == tasks_total:
            # All tasks completed successfully
            current_status.update({"status": "completed", "step": None, "cup_index": None})
            await update_status(f"Order {order_id} completed successfully")
            
            logger.info(f"[SCHEDULER] Notifying OMS of order {order_id} completion")
            await notify_oms_completion(order_id, True)
            order_completion_notified = True
            
        else:
            # Unexpected state
            reason = f"Unexpected state: {completed_tasks} completed, {failed_tasks_count} failed, {cancelled_tasks} cancelled out of {tasks_total} total"
            current_status.update({"status": "error", "step": reason})
            await update_status(f"Order {order_id} failed: {reason}")
            
            logger.info(f"[SCHEDULER] Notifying OMS of order {order_id} unexpected failure")
            await notify_oms_completion(order_id, False, reason)
            order_completion_notified = True

# Helper function to notify OMS of order completion
async def notify_oms_completion(order_id: int, success: bool, reason: Optional[str] = None, rabbitmq_client=None):
    """Notify the OMS service that an order has completed or failed via RabbitMQ events."""
    global _global_rabbitmq_client
    
    try:
        # Use provided client or the global client
        client = rabbitmq_client or _global_rabbitmq_client
        
        # If no client available, try to get it from service instance as fallback
        if not client:
            try:
                # Try to import and get the service instance directly
                import importlib
                app_module = importlib.import_module('services.scheduler.app')
                if hasattr(app_module, '_scheduler_service_instance') and app_module._scheduler_service_instance:
                    service_instance = app_module._scheduler_service_instance
                    if hasattr(service_instance, 'rabbitmq_client'):
                        client = service_instance.rabbitmq_client
                        logger.info(f"🔧 [SCHEDULER] Using service instance RabbitMQ client as fallback for order {order_id}")
                    else:
                        logger.warning(f"⚠️ [SCHEDULER] Service instance has no rabbitmq_client, skipping notification for order {order_id}")
                        return
                else:
                    logger.warning(f"⚠️ [SCHEDULER] No service instance available, skipping notification for order {order_id}")
                    return
            except Exception as fallback_error:
                logger.error(f"❌ [SCHEDULER] Fallback client access failed for order {order_id}: {fallback_error}")
                return
        
        if not client:
            logger.warning(f"⚠️ [SCHEDULER] No RabbitMQ client available, skipping notification for order {order_id}")
            return
            
        logger.info(f"🔧 [SCHEDULER] Using RabbitMQ client for order {order_id} notification")
        
        if success:
            event_data = {
                "order_id": order_id,
                "timestamp": time.time()
            }
            try:
                await asyncio.wait_for(
                    client.send_event("scheduler.order_completed", event_data),
                    timeout=10.0
                )
                logger.info(f"✅ [SCHEDULER] Order {order_id} completion sent to OMS")
            except asyncio.TimeoutError:
                logger.error(f"⏰ [SCHEDULER] Timeout sending completion event for order {order_id}")
            except Exception as send_error:
                logger.error(f"❌ [SCHEDULER] Error sending completion event for order {order_id}: {send_error}")
        else:
            event_data = {
                "order_id": order_id, 
                "error": reason or "Processing failed",
                "timestamp": time.time()
            }
            try:
                await asyncio.wait_for(
                    client.send_event("scheduler.order_failed", event_data),
                    timeout=10.0
                )
                logger.error(f"❌ [SCHEDULER] Order {order_id} failure sent to OMS: {reason}")
            except asyncio.TimeoutError:
                logger.error(f"⏰ [SCHEDULER] Timeout sending failure event for order {order_id}")
            except Exception as send_error:
                logger.error(f"❌ [SCHEDULER] Error sending failure event for order {order_id}: {send_error}")
        
    except Exception as e:
        logger.error(f"⚠️ [SCHEDULER] Failed to notify OMS for order {order_id}: {e}")
        import traceback
        logger.error(f"⚠️ [SCHEDULER] Traceback: {traceback.format_exc()}")
