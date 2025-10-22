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
# Map cup_id -> full cup data (ingredients, addons, size, etc.)
cup_data_by_cup: Dict[str, Dict[str, Any]] = {}
status_callback = None  # Callback function to notify about status updates
order_completion_notified = False  # Flag to prevent duplicate completion notifications
order_stopped = False  # Flag to signal workers to stop processing

# Per-arm cup priority scheduling data structures
per_arm_current_cups = {}    # Map arm_name -> cup_id currently being worked on (one cup per arm)
cup_completion_status = {}   # Map cup_id -> "pending" | "in_progress" | "completed" | "failed"

# Global RabbitMQ client reference for notifications
_global_rabbitmq_client = None

def set_rabbitmq_client(client):
    """Set the global RabbitMQ client for use in notifications."""
    global _global_rabbitmq_client
    _global_rabbitmq_client = client
    logger.info("🔧 [SCHEDULER] Global RabbitMQ client set for notifications")

async def _emit_plan_built(order_id: int):
    """Emit current per-arm plan for the given order so UI can render immediately."""
    try:
        if _global_rabbitmq_client is None:
            return
        plan = get_per_arm_lists()
        await _global_rabbitmq_client.send_event("scheduler.plan_built", {
            "order_id": order_id,
            "plan": plan,
            "timestamp": time.time()
        })
    except Exception as e:
        logger.warning(f"[SCHEDULER] Failed to emit plan_built: {e}")

async def _emit_task_progress(cup_id: str, action: str):
    """Emit a task progress (submitted/in_progress) update."""
    try:
        if _global_rabbitmq_client is None:
            return
        await _global_rabbitmq_client.send_event("scheduler.feedback_processed", {
            "cup_id": cup_id,
            "action": action,
            "success": None,  # signifies in-progress
            "message": "submitted"
        })
    except Exception as e:
        logger.warning(f"[SCHEDULER] Failed to emit task progress: {e}")

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
    global tasks, tasks_by_cup, completed, tasks_total, failed_tasks, failed_count, per_arm_current_cups, cup_completion_status
    tasks = []
    tasks_by_cup = {}
    completed = {}
    failed_tasks = []
    failed_count = 0
    tasks_total = 0
    
    # Reset cup-priority scheduling data structures
    per_arm_current_cups = {"Arm1": None, "Arm2": None}
    cup_completion_status = {}

    for drink, cup_id in orders:
        # Check that the recipe exists
        if drink not in recipes:
            raise Exception(f"Recipe for drink '{drink}' not found.")
        recipe = recipes[drink]
        completed[cup_id] = set()
        tasks_by_cup[cup_id] = []
        cup_completion_status[cup_id] = "pending"  # Initialize cup status
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

    # After building all tasks, emit per-arm lists of [step, cup_id]
    try:
        lists_str = _format_per_arm_lists()
        logger.info(lists_str)
    except Exception as e:
        logger.warning(f"[SCHEDULER] Could not log per-arm lists: {e}")

def get_per_arm_plan() -> Dict[str, List[Tuple[str, str]]]:
    """Return a per-arm execution plan as lists of (action, cup_id) pairs.

    The plan is derived from the current task graph and preserves the insertion
    order of cups and in-recipe step order, grouped by arm assignment.
    """
    plan: Dict[str, List[Tuple[str, str]]] = {"Arm1": [], "Arm2": []}
    try:
        # Iterate cups in insertion order, then steps in recipe order
        for cup_id, cup_tasks in tasks_by_cup.items():
            for t in cup_tasks:
                arm = t.get("assigned_arm")
                if arm in plan:
                    plan[arm].append((t.get("action"), cup_id))
    except Exception:
        # In case of any unexpected structure, fall back to scanning flat task list
        for t in tasks:
            arm = t.get("assigned_arm")
            cup_id = t.get("cup")
            if arm in ("Arm1", "Arm2") and cup_id:
                plan[arm].append((t.get("action"), cup_id))
    return plan

def get_per_arm_lists() -> Dict[str, List[List[str]]]:
    """Return per-arm lists with [step, cup_id] pairs suitable for printing/JSON."""
    plan = get_per_arm_plan()
    return {
        "Arm1": [[action, cup_id] for (action, cup_id) in plan.get("Arm1", [])],
        "Arm2": [[action, cup_id] for (action, cup_id) in plan.get("Arm2", [])],
    }

def _format_per_arm_lists() -> str:
    """Formatted string with two lists (Arm1 and Arm2) where elements are [step, cup_id]."""
    try:
        lists = get_per_arm_lists()
        return (
            f"[SCHEDULER] Arm1: {lists.get('Arm1', [])}\n"
            f"[SCHEDULER] Arm2: {lists.get('Arm2', [])}"
        )
    except Exception as e:
        return f"[SCHEDULER] Failed to format per-arm lists: {e}"

def select_task_with_per_arm_cup_priority(arm_name: str):
    """
    Select a task for the given arm using per-arm cup-priority scheduling.
    
    This algorithm allows parallel processing across arms while ensuring each arm
    completes all its tasks for a cup before starting a new cup.
    
    Priority order:
    1. Tasks from the cup this arm is currently working on
    2. Tasks from new cups (only if arm has no current cup)
    
    Returns the selected task or None if no task is available.
    """
    global tasks, per_arm_current_cups, completed, cup_completion_status
    
    # Get all pending tasks for this arm
    pending_tasks = [t for t in tasks if t["assigned_arm"] == arm_name and t["status"] == "pending"]
    
    # Filter to tasks whose dependencies are satisfied
    available_tasks = []
    for task in pending_tasks:
        cup_id = task["cup"]
        deps = task["depends_on"]
        if all(dep in completed[cup_id] for dep in deps):
            available_tasks.append(task)
    
    if not available_tasks:
        return None
    
    # Priority 1: Tasks from cups this arm is currently working on
    current_cup_id = per_arm_current_cups[arm_name]
    current_cup_tasks = [t for t in available_tasks if t["cup"] == current_cup_id]
    
    if current_cup_tasks:
        # Select the first available task from current cups
        task = current_cup_tasks[0]
        task["status"] = "in_progress"
        logger.info(f"🎯 [CUP-PRIORITY] {arm_name} continuing work on cup {task['cup']} - {task['action']}")
        return task
    
    # Priority 2: Tasks from new cups (only if arm has no current cup)
    if current_cup_id is None:
        # Start working on a new cup - prefer pending cups, but also allow in-progress cups
        new_cup_tasks = [t for t in available_tasks if cup_completion_status[t["cup"]] in ["pending", "in_progress"]]
        
        if new_cup_tasks:
            task = new_cup_tasks[0]
            task["status"] = "in_progress"
            cup_id = task["cup"]
            
            # Mark this cup as being worked on by this arm
            per_arm_current_cups[arm_name] = cup_id
            if cup_completion_status[cup_id] == "pending":
                cup_completion_status[cup_id] = "in_progress"
            
            logger.info(f"🆕 [PER-ARM-CUP-PRIORITY] {arm_name} starting new cup {cup_id} - {task['action']}")
            return task
    
    # If we get here, the arm has a current cup but no available tasks for it
    # This means we're waiting for dependencies to be satisfied
    logger.debug(f"⏳ [PER-ARM-CUP-PRIORITY] {arm_name} waiting for dependencies on cup: {current_cup_id}")
    return None

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
        
        # Read cup data (ingredients/addons/size) captured during setup
        with lock:
            cup_data = cup_data_by_cup.get(cup_id, {})
        ingredients = cup_data.get("ingredients", {})
        addons = cup_data.get("addons", [])
        size = cup_data.get("size")

        # Create the payload for the routine service
        payload = {
            "arm_id": int(arm_id.replace("Arm", "")),  # Convert "Arm1" to 1
            "function": function,
            "item": {
                "cup_id": cup_id,
                "drink_type": drink_type,
                "size": size,
                "addons": addons,
                "ingredients": ingredients
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
    """Worker thread for a robotic arm that executes tasks when they are ready.
    
    This worker is persistent and handles multiple orders sequentially without exiting.
    It waits for new tasks when the current order completes.
    """
    global completed_count, failed_count, order_stopped, tasks_total
    logger.info(f"🤖 [SCHEDULER] {arm_name} worker started and ready for orders")
    
    consecutive_no_work_count = 0
    max_idle_before_check = 100  # Check every 10 seconds if idle
    
    while True:
        task = None
        current_tasks_total = 0
        
        # Find a pending task for this arm with all dependencies satisfied (Cup-Priority Algorithm)
        with lock:
            current_tasks_total = tasks_total
            
            # Check if order is stopped
            if order_stopped:
                # Don't pick up new tasks, but wait for submitted tasks to complete
                submitted_tasks = [t for t in tasks if t["status"] == "submitted"]
                if len(submitted_tasks) == 0:
                    logger.info(f"🛑 {arm_name} worker: order stopped, no tasks in progress - waiting for next order")
                    task = None
                else:
                    logger.debug(f"🛑 {arm_name} worker waiting for {len(submitted_tasks)} submitted tasks to complete before stopping")
                    task = None
            # Check if current order is complete (all tasks done or failed)
            elif current_tasks_total > 0 and completed_count + failed_count >= current_tasks_total:
                logger.info(f"🏁 {arm_name} worker: current order complete (Completed: {completed_count}, Failed: {failed_count}, Total: {current_tasks_total}) - waiting for next order")
                task = None
                consecutive_no_work_count = 0  # Reset counter when order completes
            # Active order with tasks to process
            elif current_tasks_total > 0:
                task = select_task_with_per_arm_cup_priority(arm_name)
            # No active order - wait for tasks
            else:
                task = None
        
        # Never exit - workers are persistent
        # if should_exit:
        #     break
            
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
                # Emit in-progress update so dashboard marks it
                try:
                    import asyncio as _asyncio
                    loop = _asyncio.get_event_loop()
                    if loop.is_running():
                        loop.create_task(_emit_task_progress(cup_id, action))
                    else:
                        loop.run_until_complete(_emit_task_progress(cup_id, action))
                except Exception as _e:
                    logger.warning(f"[SCHEDULER] Could not emit task progress: {_e}")
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
                
                if consecutive_no_work_count % max_idle_before_check == 0:  # Log every 10 seconds when idle
                    if current_tasks_total > 0:
                        logger.debug(f"🤖 {arm_name} waiting - Pending: {len(pending_tasks)}, Submitted: {len(submitted_tasks)}, Completed: {len(completed_tasks)}, Failed: {len(failed_tasks_status)}")
                    else:
                        logger.debug(f"🤖 {arm_name} idle - waiting for new orders")
            
            # Workers are persistent - reset counter periodically to prevent overflow
            if consecutive_no_work_count >= 10000:  # Reset after ~1000 seconds (16 minutes) of idling
                consecutive_no_work_count = 0
                logger.debug(f"🤖 {arm_name} worker still active and waiting for orders")
            
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
    global tasks, tasks_by_cup, completed, tasks_total, current_status, failed_tasks, failed_count, order_completion_notified, completed_count, per_arm_current_cups, cup_completion_status, order_stopped, cup_data_by_cup
    
    logger.info(f"[SCHEDULER] Setting up tasks for order {order_id} with {len(drinks)} drinks")
    
    # Reset ALL state at the START of new order setup to avoid race conditions
    # This is done synchronously and atomically
    with lock:
        reset_scheduler_state_sync()
    
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
        # Keep the full cup dict for downstream payloads (ingredients, addons, size, etc.)
        cup_data_by_cup[cup_id] = cup
        
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
        # Emit plan immediately after building tasks so dashboard renders sequence early
        try:
            import asyncio as _asyncio
            loop = _asyncio.get_event_loop()
            if loop.is_running():
                loop.create_task(_emit_plan_built(order_id))
            else:
                loop.run_until_complete(_emit_plan_built(order_id))
        except Exception as _e:
            logger.warning(f"[SCHEDULER] Could not schedule plan_built emission: {_e}")
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
    
    # Start heartbeat monitoring for this order
    heartbeat_task = None
    try:
        heartbeat_task = await start_order_heartbeat(order_id)
        logger.info(f"💓 [SCHEDULER] Started heartbeat monitoring for order {order_id}")
    except Exception as e:
        logger.warning(f"⚠️ [SCHEDULER] Failed to start heartbeat for order {order_id}: {e}")
    
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
                
                # Notify OMS about order failure with retry
                notification_success = False
                for attempt in range(3):
                    if attempt > 0:
                        delay = 2 ** attempt
                        logger.warning(f"[SCHEDULER] Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay")
                        await asyncio.sleep(delay)
                    
                    notification_success = await notify_oms_completion(order_id, False, reason)
                    if notification_success:
                        break
                
                if notification_success:
                    order_completion_notified = True
                    logger.info(f"✅ [SCHEDULER] Order {order_id} failure notification confirmed")
                else:
                    logger.error(f"❌ [SCHEDULER] Failed to notify OMS of order {order_id} failure after 3 attempts")
                
                return False
            elif completed_count == tasks_total:
                # All tasks completed successfully
                current_status.update({"status": "completed", "step": None, "cup_index": None})
                await update_status(f"Order {order_id} completed successfully")
                
                # Notify OMS about order completion with retry
                notification_success = False
                for attempt in range(3):
                    if attempt > 0:
                        delay = 2 ** attempt
                        logger.warning(f"[SCHEDULER] Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay")
                        await asyncio.sleep(delay)
                    
                    notification_success = await notify_oms_completion(order_id, True)
                    if notification_success:
                        break
                
                if notification_success:
                    order_completion_notified = True
                    logger.info(f"✅ [SCHEDULER] Order {order_id} completion notification confirmed")
                else:
                    logger.error(f"❌ [SCHEDULER] Failed to notify OMS of order {order_id} completion after 3 attempts")
                
                return True
            else:
                # This shouldn't happen, but handle it as a failure
                reason = f"Unexpected state: {completed_count} completed, {failed_count} failed out of {tasks_total} total"
                current_status.update({"status": "error", "step": reason})
                await update_status(f"Order {order_id} failed: {reason}")
                
                # Notify OMS about order failure with retry
                notification_success = False
                for attempt in range(3):
                    if attempt > 0:
                        delay = 2 ** attempt
                        logger.warning(f"[SCHEDULER] Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay")
                        await asyncio.sleep(delay)
                    
                    notification_success = await notify_oms_completion(order_id, False, reason)
                    if notification_success:
                        break
                
                if notification_success:
                    order_completion_notified = True
                    logger.info(f"✅ [SCHEDULER] Order {order_id} failure notification confirmed")
                else:
                    logger.error(f"❌ [SCHEDULER] Failed to notify OMS of order {order_id} failure after 3 attempts")
                
                return False
                
    except Exception as e:
        # Notify OMS about order failure with retry
        current_status.update({"status": "error", "step": str(e)})
        await update_status(f"Order {order_id} failed: {e}")
        
        notification_success = False
        for attempt in range(3):
            if attempt > 0:
                delay = 2 ** attempt
                logger.warning(f"[SCHEDULER] Retry attempt {attempt + 1}/3 for order {order_id} exception notification after {delay}s delay")
                await asyncio.sleep(delay)
            
            notification_success = await notify_oms_completion(order_id, False, f"Order processing failed: {str(e)}")
            if notification_success:
                break
        
        if notification_success:
            with lock:
                order_completion_notified = True
            logger.info(f"✅ [SCHEDULER] Order {order_id} exception notification confirmed")
        else:
            logger.error(f"❌ [SCHEDULER] Failed to notify OMS of order {order_id} exception after 3 attempts")
        
        return False
    finally:
        # Clean up heartbeat task
        if heartbeat_task and not heartbeat_task.done():
            heartbeat_task.cancel()
            try:
                await heartbeat_task
            except asyncio.CancelledError:
                pass
            logger.info(f"💓 [SCHEDULER] Stopped heartbeat monitoring for order {order_id}")

def get_current_status():
    """Get the current processing status."""
    return current_status

async def update_cup_position(cup_id: str, new_position: float) -> bool:
    """
    Update the cup position for all pending tasks of a given cup.
    Called when cup_detection finds a better position.
    
    Args:
        cup_id: The cup ID to update
        new_position: The new cup position (1-4)
    
    Returns:
        True if update successful, False if cup not found
    """
    global cup_data_by_cup, tasks
    
    logger.info(f"[SCHEDULER] 🔄 Updating cup position for {cup_id} to {new_position}")
    
    with lock:
        # Update in cup_data_by_cup (master data for this cup)
        if cup_id not in cup_data_by_cup:
            logger.warning(f"[SCHEDULER] Cup {cup_id} not found in cup_data_by_cup")
            return False
        
        cup_data = cup_data_by_cup[cup_id]
        
        # Update position in ingredients
        if "ingredients" in cup_data:
            if "position" in cup_data["ingredients"]:
                old_position = cup_data["ingredients"]["position"].get("cup_position")
                cup_data["ingredients"]["position"]["cup_position"] = new_position
                logger.info(f"[SCHEDULER] Updated cup_data_by_cup[{cup_id}]['ingredients']['position']['cup_position']: {old_position} → {new_position}")
            else:
                # Create position dict if it doesn't exist
                cup_data["ingredients"]["position"] = {"cup_position": new_position}
                logger.info(f"[SCHEDULER] Created position entry in cup_data_by_cup[{cup_id}]")
        else:
            logger.warning(f"[SCHEDULER] No ingredients found in cup_data for {cup_id}")
        
        # Update all pending tasks for this cup
        updated_task_count = 0
        for task in tasks:
            if task["cup"] == cup_id and task["status"] == "pending":
                # Update position in task's item data
                if "item" in task and "ingredients" in task["item"]:
                    if "position" in task["item"]["ingredients"]:
                        task["item"]["ingredients"]["position"]["cup_position"] = new_position
                        updated_task_count += 1
                    else:
                        task["item"]["ingredients"]["position"] = {"cup_position": new_position}
                        updated_task_count += 1
        
        logger.info(f"[SCHEDULER] ✅ Updated {updated_task_count} pending tasks for {cup_id}")
        logger.info(f"[SCHEDULER] 🎯 Future tasks for {cup_id} will use position {new_position}")
        
        return True

async def handle_routine_feedback(cup_id: str, action: str, success: bool):
    """
    Handle feedback from the routine service about task completion.
    
    When a task fails, immediately notifies OMS and cancels remaining tasks
    instead of waiting for all tasks to complete.
    """
    global completed_count, completed, failed_count, failed_tasks, order_completion_notified, per_arm_current_cups, cup_completion_status
    
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
                            
                            # Cup is now complete - remove from arm's current cup
                            arm_name = task["assigned_arm"]
                            if arm_name in per_arm_current_cups and per_arm_current_cups[arm_name] == cup_id:
                                per_arm_current_cups[arm_name] = None
                            cup_completion_status[cup_id] = "completed"
                            logger.info(f"🏁 [PER-ARM-CUP-PRIORITY] Cup {cup_id} completed by {arm_name}")
                    else:
                        # Mark task as failed
                        task["status"] = "failed"
                        failed_tasks.append(task)
                        failed_count += 1
                        logger.error(f"[SCHEDULER] Task failed: {action} for {cup_id}")
                        
                        # Mark cup as failed and remove from arm's current cup
                        arm_name = task["assigned_arm"]
                        if arm_name in per_arm_current_cups and per_arm_current_cups[arm_name] == cup_id:
                            per_arm_current_cups[arm_name] = None
                        cup_completion_status[cup_id] = "failed"
                        logger.error(f"❌ [PER-ARM-CUP-PRIORITY] Cup {cup_id} failed due to {action} task failure")
                        
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
                failed_action = failed_task_info['action']
                failed_cup = failed_task_info['cup']
                reason = f"Task failed: {failed_action} for {failed_cup}"
                
                with lock:
                    current_status.update({"status": "error", "step": f"Task {failed_action} failed"})
                
                await update_status(f"Order {order_id} failed: {reason}")
                logger.info(f"[SCHEDULER] Notifying OMS of order {order_id} failure")
                
                # Retry notification up to 3 times with exponential backoff
                notification_success = False
                for attempt in range(3):
                    if attempt > 0:
                        delay = 2 ** attempt  # 2, 4 seconds
                        logger.warning(f"[SCHEDULER] Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay")
                        await asyncio.sleep(delay)
                    
                    notification_success = await notify_oms_completion(order_id, False, reason)
                    if notification_success:
                        break
                
                # Only mark as notified if we successfully delivered the notification
                if notification_success:
                    with lock:
                        order_completion_notified = True
                    logger.info(f"✅ [SCHEDULER] Order {order_id} failure notification confirmed after {attempt + 1} attempt(s)")
                    # State will be reset when next order starts
                else:
                    logger.error(f"❌ [SCHEDULER] Failed to notify OMS of order {order_id} failure after 3 attempts")
                
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

def reset_scheduler_state_sync():
    """Reset scheduler state synchronously (called at start of new order setup).
    
    This function resets the scheduler state in preparation for a new order.
    It should be called from within setup_tasks_from_order to avoid race conditions.
    """
    global tasks, tasks_by_cup, completed, failed_tasks, failed_count, completed_count
    global tasks_total, order_completion_notified, order_stopped, per_arm_current_cups, cup_completion_status, cup_data_by_cup
    
    try:
        logger.info("🔄 [SCHEDULER] Resetting scheduler state for new order")
        # This function is called from within a lock in setup_tasks_from_order
        tasks.clear()
        tasks_by_cup.clear()
        completed.clear()
        failed_tasks.clear()
        failed_count = 0
        completed_count = 0
        tasks_total = 0
        order_completion_notified = False
        order_stopped = False
        per_arm_current_cups.clear()
        cup_completion_status.clear()
        cup_data_by_cup.clear()
        
        # Keep current_status for debugging, but mark as idle
        current_status.update({
            "order_id": None,
            "status": "idle",
            "step": None,
            "cup_index": None
        })
        logger.info("✅ [SCHEDULER] State reset complete - ready for new order")
    except Exception as e:
        logger.error(f"❌ [SCHEDULER] Error resetting state: {e}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")

async def check_and_notify_order_completion():
    """Check if the current order is complete and notify OMS if so."""
    global completed_count, failed_count, tasks_total, current_status, order_completion_notified
    
    # Read state snapshot while holding lock (minimize lock time)
    with lock:
        # Count actual task statuses instead of relying on counters (more reliable)
        completed_tasks = sum(1 for task in tasks if task["status"] == "done")
        failed_tasks_count = sum(1 for task in tasks if task["status"] == "failed")
        cancelled_tasks = sum(1 for task in tasks if task["status"] == "cancelled")
        total_finished = completed_tasks + failed_tasks_count + cancelled_tasks
        
        order_id = current_status.get("order_id")
        current_tasks_total = tasks_total
        already_notified = order_completion_notified
        
        logger.info(f"🔍 DEBUG: check_and_notify_order_completion called - order_id={order_id}, completed={completed_tasks}, failed={failed_tasks_count}, cancelled={cancelled_tasks}, total={current_tasks_total}, notified={already_notified}")
        
        # Only proceed if we have an order_id and not already notified
        if not order_id or already_notified:
            logger.info(f"🔍 DEBUG: Early return - no order_id ({not order_id}) or already notified ({already_notified})")
            return
            
        # If not all tasks are finished and no failures, continue waiting
        if total_finished < current_tasks_total and failed_tasks_count == 0:
            logger.info(f"🔍 DEBUG: Waiting for more tasks - finished({total_finished}) < total({current_tasks_total}) and no failures")
            return
        
        # Get failed task details if needed
        failed_task_list = [task.copy() for task in tasks if task["status"] == "failed"]
    
    # Release lock before doing async operations
    # Now handle the three cases outside the lock
    
    if failed_tasks_count > 0:
        # Some tasks failed - notify failure
        failed_task_names = [f"{task['action']} ({task['cup']})" for task in failed_task_list]
        reason = f"Failed tasks: {', '.join(failed_task_names)}"
        
        with lock:
            current_status.update({"status": "error", "step": f"{failed_tasks_count} tasks failed"})
        await update_status(f"Order {order_id} failed: {failed_tasks_count} out of {current_tasks_total} tasks failed")
        
        logger.info(f"[SCHEDULER] Notifying OMS of order {order_id} failure")
        
        # Retry notification up to 3 times with exponential backoff
        notification_success = False
        for attempt in range(3):
            if attempt > 0:
                delay = 2 ** attempt  # 2, 4 seconds
                logger.warning(f"[SCHEDULER] Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay")
                await asyncio.sleep(delay)
            
            notification_success = await notify_oms_completion(order_id, False, reason)
            if notification_success:
                break
        
        if notification_success:
            with lock:
                order_completion_notified = True
            logger.info(f"✅ [SCHEDULER] Order {order_id} failure notification confirmed after {attempt + 1} attempt(s)")
            # State will be reset when next order starts
        else:
            logger.error(f"❌ [SCHEDULER] Failed to notify OMS of order {order_id} failure after 3 attempts")
        
    elif completed_tasks == current_tasks_total:
        # All tasks completed successfully
        with lock:
            current_status.update({"status": "completed", "step": None, "cup_index": None})
        await update_status(f"Order {order_id} completed successfully")
        
        logger.info(f"[SCHEDULER] Notifying OMS of order {order_id} completion")
        
        # Retry notification up to 3 times with exponential backoff
        notification_success = False
        for attempt in range(3):
            if attempt > 0:
                delay = 2 ** attempt  # 2, 4 seconds
                logger.warning(f"[SCHEDULER] Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay")
                await asyncio.sleep(delay)
            
            notification_success = await notify_oms_completion(order_id, True)
            if notification_success:
                break
        
        if notification_success:
            with lock:
                order_completion_notified = True
            logger.info(f"✅ [SCHEDULER] Order {order_id} completion notification confirmed after {attempt + 1} attempt(s)")
            # State will be reset when next order starts
        else:
            logger.error(f"❌ [SCHEDULER] Failed to notify OMS of order {order_id} completion after 3 attempts")
        
    else:
        # Unexpected state
        reason = f"Unexpected state: {completed_tasks} completed, {failed_tasks_count} failed, {cancelled_tasks} cancelled out of {current_tasks_total} total"
        with lock:
            current_status.update({"status": "error", "step": reason})
        await update_status(f"Order {order_id} failed: {reason}")
        
        logger.info(f"[SCHEDULER] Notifying OMS of order {order_id} unexpected failure")
        
        # Retry notification up to 3 times with exponential backoff
        notification_success = False
        for attempt in range(3):
            if attempt > 0:
                delay = 2 ** attempt  # 2, 4 seconds
                logger.warning(f"[SCHEDULER] Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay")
                await asyncio.sleep(delay)
            
            notification_success = await notify_oms_completion(order_id, False, reason)
            if notification_success:
                break
        
        if notification_success:
            with lock:
                order_completion_notified = True
            logger.info(f"✅ [SCHEDULER] Order {order_id} failure notification confirmed after {attempt + 1} attempt(s)")
            # State will be reset when next order starts
        else:
            logger.error(f"❌ [SCHEDULER] Failed to notify OMS of order {order_id} failure after 3 attempts")

# Helper function to notify OMS of order completion
async def notify_oms_completion(order_id: int, success: bool, reason: Optional[str] = None, rabbitmq_client=None) -> bool:
    """Notify the OMS service that an order has completed or failed via RabbitMQ events.
    
    Returns:
        bool: True if notification was confirmed received by OMS, False otherwise
    """
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
                        return False
                else:
                    logger.warning(f"⚠️ [SCHEDULER] No service instance available, skipping notification for order {order_id}")
                    return False
            except Exception as fallback_error:
                logger.error(f"❌ [SCHEDULER] Fallback client access failed for order {order_id}: {fallback_error}")
                return False
        
        if not client:
            logger.warning(f"⚠️ [SCHEDULER] No RabbitMQ client available, skipping notification for order {order_id}")
            return False
            
        logger.info(f"🔧 [SCHEDULER] Using RabbitMQ client for order {order_id} notification")
        
        # Use send_event_with_ack for guaranteed delivery with acknowledgment
        if success:
            event_data = {
                "order_id": order_id,
                "timestamp": time.time()
            }
            event_type = "scheduler.order_completed"
        else:
            event_data = {
                "order_id": order_id, 
                "error": reason or "Processing failed",
                "timestamp": time.time()
            }
            event_type = "scheduler.order_failed"
        
        # Send event with acknowledgment (20 second timeout for DB writes)
        try:
            result = await client.send_event_with_ack(event_type, event_data, timeout=20.0)
            
            if result.get("success") and result.get("acknowledged"):
                logger.info(f"✅ [SCHEDULER] Order {order_id} {'completion' if success else 'failure'} confirmed by OMS")
                return True
            else:
                error_msg = result.get("error", "Unknown error")
                logger.error(f"❌ [SCHEDULER] OMS did not acknowledge order {order_id} notification: {error_msg}")
                return False
                
        except Exception as send_error:
            logger.error(f"❌ [SCHEDULER] Error sending acknowledged event for order {order_id}: {send_error}")
            import traceback
            logger.error(f"❌ [SCHEDULER] Traceback: {traceback.format_exc()}")
            return False
        
    except Exception as e:
        logger.error(f"⚠️ [SCHEDULER] Failed to notify OMS for order {order_id}: {e}")
        import traceback
        logger.error(f"⚠️ [SCHEDULER] Traceback: {traceback.format_exc()}")
        return False

async def send_order_heartbeat(order_id: int, status: str, progress: Dict[str, Any] = None):
    """Send periodic heartbeat updates for long-running orders"""
    global _global_rabbitmq_client
    
    try:
        client = _global_rabbitmq_client
        if not client:
            return
            
        heartbeat_data = {
            "order_id": order_id,
            "status": status,
            "timestamp": time.time(),
            "progress": progress or {}
        }
        
        # Use fire-and-forget event for heartbeats (less critical)
        await client.send_event("scheduler.order_heartbeat", heartbeat_data)
        logger.debug(f"💓 [SCHEDULER] Heartbeat sent for order {order_id}: {status}")
        
    except Exception as e:
        logger.warning(f"⚠️ [SCHEDULER] Failed to send heartbeat for order {order_id}: {e}")

async def start_order_heartbeat(order_id: int):
    """Start periodic heartbeat for an order"""
    global current_status
    
    async def heartbeat_loop():
        while True:
            try:
                with lock:
                    if current_status.get("order_id") != order_id:
                        # Order changed, stop heartbeat
                        break
                    
                    # Calculate progress
                    total_tasks = tasks_total
                    completed_tasks = completed_count
                    failed_tasks = failed_count
                    
                    progress = {
                        "total_tasks": total_tasks,
                        "completed_tasks": completed_tasks,
                        "failed_tasks": failed_tasks,
                        "completion_percentage": (completed_tasks / total_tasks * 100) if total_tasks > 0 else 0
                    }
                
                await send_order_heartbeat(order_id, "processing", progress)
                await asyncio.sleep(30)  # Send heartbeat every 30 seconds
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.warning(f"⚠️ [SCHEDULER] Heartbeat loop error for order {order_id}: {e}")
                await asyncio.sleep(30)
    
    # Start heartbeat task
    heartbeat_task = asyncio.create_task(heartbeat_loop())
    return heartbeat_task
