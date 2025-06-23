import asyncio
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Any, Tuple
import httpx

from .data import logger

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

async def submit_task_to_routine(arm_id: int, function: str, cup_id: str, drink_type: str):
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
                "cup_size": "regular",  # Default size, could be made configurable
                "drink": drink_type,
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
            logger.log(f"✅ Task submitted to routine: {function} on arm {arm_id} for cup {cup_id}")
            return True
        else:
            logger.log(f"❌ Failed to submit task to routine: {response.get('error', 'Unknown error')}")
            return False
            
    except Exception as e:
        logger.log(f"❌ Error submitting task to routine: {str(e)}")
        return False
    finally:
        # Ensure client is disconnected even if an exception occurs
        if client:
            try:
                await client.disconnect()
            except Exception as disconnect_error:
                logger.log(f"⚠️ Error disconnecting RabbitMQ client: {disconnect_error}")

async def arm_worker(arm_name: str):
    """Worker thread for a robotic arm that executes tasks when they are ready."""
    global completed_count, failed_count
    logger.log(f"🤖 DEBUG: {arm_name} worker started. Total tasks to process: {tasks_total}")
    
    consecutive_no_work_count = 0
    max_consecutive_no_work = 100  # Increased timeout for better reliability
    
    while True:
        task = None
        should_exit = False
        
        # Find a pending task for this arm with all dependencies satisfied
        with lock:
            # Exit when all tasks are either completed or failed (NOT just submitted)
            total_finished = completed_count + failed_count
            if total_finished >= tasks_total:
                logger.log(f"🤖 DEBUG: {arm_name} worker finished. Completed: {completed_count}, Failed: {failed_count}, Total: {tasks_total}")
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
                    logger.log(f"✅ Task {action} for cup {cup_id} successfully submitted to routine")
            else:
                # Mark the task as failed immediately
                with lock:
                    task["status"] = "failed"
                    failed_tasks.append(task)
                    failed_count += 1
                    logger.log(f"❌ Failed to submit task: {action} for cup {cup_id}")
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
                    logger.log(f"🤖 DEBUG: {arm_name} waiting - Pending: {len(pending_tasks)}, Submitted: {len(submitted_tasks)}, Completed: {len(completed_tasks)}, Failed: {len(failed_tasks_status)}")
            
            # Prevent infinite waiting - if we've been waiting too long, check if we should exit
            if consecutive_no_work_count >= max_consecutive_no_work:
                with lock:
                    total_finished = completed_count + failed_count
                    if total_finished >= tasks_total:
                        logger.log(f"🤖 DEBUG: {arm_name} worker exiting after waiting - all tasks done")
                        break
                    
                    # Check if there are any tasks left that could potentially be processed
                    pending_tasks = [t for t in tasks if t["status"] == "pending"]
                    submitted_tasks = [t for t in tasks if t["status"] == "submitted"]
                    
                    if not pending_tasks and not submitted_tasks:
                        logger.log(f"🤖 DEBUG: {arm_name} worker exiting - no pending or submitted tasks left")
                        break
                        
                    # If there are submitted tasks, continue waiting for them to complete
                    if submitted_tasks:
                        logger.log(f"🤖 DEBUG: {arm_name} continuing to wait for {len(submitted_tasks)} submitted tasks to complete")
                        consecutive_no_work_count = 0  # Reset counter and continue waiting
                    elif not any(all(dep in completed[t["cup"]] for dep in t["depends_on"]) for t in pending_tasks if t["assigned_arm"] == arm_name):
                        # No tasks for this arm can be executed due to dependencies
                        logger.log(f"🤖 DEBUG: {arm_name} worker exiting - no executable tasks for this arm")
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
    logger.log(f"Starting coffee order simulation for {len(orders)} orders...")
    
    # Run with asyncio since workers are now async
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_async())
    
    logger.log("All orders completed.")

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
    logger.log(message)
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
    global tasks, tasks_by_cup, completed, tasks_total, current_status, failed_tasks, failed_count
    
    logger.log(f"🔍 DEBUG: setup_tasks_from_order called for order {order_id}")
    logger.log(f"🔍 DEBUG: Received recipes: {list(recipes.keys()) if recipes else 'None/Empty'}")
    logger.log(f"🔍 DEBUG: Received drinks: {drinks}")
    
    # Reset or initialize the task structures
    tasks = []
    tasks_by_cup = {}
    completed = {}
    failed_tasks = []
    failed_count = 0
    tasks_total = 0
    
    current_status.update({"order_id": order_id, "status": "in_progress", "step": "preparing"})
    
    # Convert API drink format to scheduler format
    orders = []
    for idx, cup in enumerate(drinks, start=1):
        drink_type = cup.get("type")
        cup_id = f"{order_id}-{idx}"  # Create a unique cup ID
        orders.append((drink_type, cup_id))
        logger.log(f"🔍 DEBUG: Created order: drink={drink_type}, cup_id={cup_id}")
        
        # Check if recipe exists for this drink
        if drink_type not in recipes:
            logger.log(f"❌ ERROR: No recipe found for drink type '{drink_type}'")
            logger.log(f"❌ Available recipes: {list(recipes.keys())}")
            return False
        else:
            logger.log(f"✅ Found recipe for '{drink_type}' with {len(recipes[drink_type])} steps")
    
    logger.log(f"🔍 DEBUG: Processing {len(orders)} orders: {orders}")
    
    # Use existing setup_tasks logic
    try:
        setup_tasks(orders, recipes)
        logger.log(f"🔍 DEBUG: setup_tasks completed. Created {tasks_total} total tasks")
        return True
    except Exception as e:
        logger.log(f"❌ Error setting up tasks: {str(e)}")
        import traceback
        logger.log(f"❌ Setup tasks traceback: {traceback.format_exc()}")
        current_status.update({"status": "error", "step": str(e)})
        return False

async def process_order_async(order_id: int, drinks: List[Dict[str, Any]], recipes: Dict[str, List[Dict[str, Any]]]):
    """Process an order asynchronously using the scheduler."""
    global current_status, completed_count, tasks_total, failed_count
    
    logger.log(f"🔍 DEBUG: process_order_async called for order {order_id} with {len(drinks)} drinks")
    logger.log(f"🔍 DEBUG: Available recipes: {list(recipes.keys()) if recipes else 'No recipes loaded'}")
    logger.log(f"🔍 DEBUG: Recipe count: {len(recipes) if recipes else 0}")
    logger.log(f"🔍 DEBUG: Drinks to process: {drinks}")
    
    # Check if recipes is empty
    if not recipes:
        logger.log(f"❌ CRITICAL: No recipes available for order {order_id}")
        await notify_oms_completion(order_id, False, "No recipes available")
        return False
    
    # Setup tasks for the order
    try:
        setup_result = setup_tasks_from_order(order_id, drinks, recipes)
        logger.log(f"🔍 DEBUG: setup_tasks_from_order returned: {setup_result}")
        
        if not setup_result:
            logger.log(f"❌ Failed to setup tasks for order {order_id}")
            # Notify OMS about order failure
            await notify_oms_completion(order_id, False, "Failed to setup tasks for order")
            return False
    except Exception as e:
        logger.log(f"❌ Exception in setup_tasks_from_order for order {order_id}: {e}")
        import traceback
        logger.log(f"❌ Setup tasks exception traceback: {traceback.format_exc()}")
        await notify_oms_completion(order_id, False, f"Setup tasks exception: {str(e)}")
        return False
    
    logger.log(f"✅ Tasks setup successfully for order {order_id}. Total tasks: {tasks_total}")
    
    # Reset counters
    completed_count = 0
    
    # Start worker threads
    await update_status(f"Starting to process order {order_id}")
    
    try:
        # Create and start arm workers
        arm1 = asyncio.create_task(arm_worker("Arm1"))
        arm2 = asyncio.create_task(arm_worker("Arm2"))
        
        # Wait for both arms to finish all tasks with timeout to prevent hanging
        try:
            await asyncio.wait_for(
                asyncio.gather(arm1, arm2, return_exceptions=True),
                timeout=300.0  # 5 minute timeout for order processing
            )
            logger.log(f"✅ Both arm workers completed for order {order_id}")
        except asyncio.TimeoutError:
            logger.log(f"❌ Order {order_id} timed out after 5 minutes")
            # Cancel both arms
            arm1.cancel()
            arm2.cancel()
            await notify_oms_completion(order_id, False, "Order processing timed out after 5 minutes")
            return False
        
        # Check if order was successful or failed
        with lock:
            if failed_count > 0:
                # Some tasks failed - mark order as failed
                current_status.update({"status": "error", "step": f"{failed_count} tasks failed"})
                await update_status(f"Order {order_id} failed: {failed_count} out of {tasks_total} tasks failed")
                
                # Create failure reason listing failed tasks
                failed_task_names = [f"{task['action']} ({task['cup']})" for task in failed_tasks]
                reason = f"Failed tasks: {', '.join(failed_task_names)}"
                
                # Notify OMS about order failure
                await notify_oms_completion(order_id, False, reason)
                return False
            elif completed_count == tasks_total:
                # All tasks completed successfully
                current_status.update({"status": "completed", "step": None, "cup_index": None})
                await update_status(f"Order {order_id} completed successfully")
                
                # Notify OMS about order completion
                await notify_oms_completion(order_id, True)
                return True
            else:
                # This shouldn't happen, but handle it as a failure
                reason = f"Unexpected state: {completed_count} completed, {failed_count} failed out of {tasks_total} total"
                current_status.update({"status": "error", "step": reason})
                await update_status(f"Order {order_id} failed: {reason}")
                
                # Notify OMS about order failure
                await notify_oms_completion(order_id, False, reason)
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
    """Handle feedback from the routine service about task completion."""
    global completed_count, completed, failed_count, failed_tasks
    
    logger.log(f"🔄 [SCHEDULER] Processing feedback: cup_id={cup_id}, action={action}, success={success}")
    logger.log(f"🔍 [SCHEDULER] Current counts - completed: {completed_count}, failed: {failed_count}, total: {tasks_total}")
    
    with lock:
        # Find the first matching task that is not yet completed/failed
        task_found = False
        for task in tasks:
            if task["cup"] == cup_id and task["action"] == action and task["status"] not in ["done", "failed"]:
                task_found = True
                logger.log(f"✅ [SCHEDULER] Found matching task: {task}")
                
                # This task is ready to be processed
                
                if success:
                    # Mark task as completed
                    task["status"] = "done"
                    completed[cup_id].add(action)
                    completed_count += 1
                    logger.log(f"✅ [SCHEDULER] Task marked as completed. New completed_count: {completed_count}")
                    
                    # Check if this was the final task for this cup
                    if len(completed[cup_id]) == len(tasks_by_cup[cup_id]):
                        message = f"Order complete: {task['drink']} for {cup_id}"
                        logger.log(f" --- {message} ---")
                        await update_status(message)
                else:
                    # Mark task as failed and count in failed_count
                    task["status"] = "failed"
                    failed_tasks.append(task)
                    failed_count += 1
                    logger.log(f"❌ [SCHEDULER] Task failed: {action} for cup {cup_id}. New failed_count: {failed_count}")
                break
        
        if not task_found:
            logger.log(f"⚠️ [SCHEDULER] No matching task found for cup_id={cup_id}, action={action}")
            logger.log(f"🔍 [SCHEDULER] Available tasks: {[(t['cup'], t['action']) for t in tasks]}")

# Helper function to notify OMS of order completion
async def notify_oms_completion(order_id: int, success: bool, reason: str = None):
    """Notify the OMS service that an order has completed or failed via RabbitMQ events."""
    client = None
    try:
        # Import RabbitMQ client here to avoid circular imports
        import sys
        import os
        sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
        from shared.rabbitmq_client import RabbitMQClient
        
        # Create a temporary RabbitMQ client for sending events with unique ID
        import uuid
        client = RabbitMQClient(f"scheduler_notifier_{uuid.uuid4().hex[:8]}")
        await client.connect()
        
        if success:
            # Order completed successfully - send event
            await client.send_event("scheduler.order_completed", {
                "order_id": order_id,
                "timestamp": time.time()
            })
            logger.log(f"✅ Sent RabbitMQ event: Order {order_id} completed successfully")
        else:
            # Order failed - send event
            await client.send_event("scheduler.order_failed", {
                "order_id": order_id, 
                "error": reason or "Processing failed",
                "timestamp": time.time()
            })
            logger.log(f"❌ Sent RabbitMQ event: Order {order_id} failed - {reason}")
        
    except Exception as e:
        logger.log(f"⚠️ Failed to send RabbitMQ event for order {order_id}: {e}")
    finally:
        # Ensure client is disconnected even if an exception occurs
        if client:
            try:
                await client.disconnect()
            except Exception as disconnect_error:
                logger.log(f"⚠️ Error disconnecting RabbitMQ notifier client: {disconnect_error}")
