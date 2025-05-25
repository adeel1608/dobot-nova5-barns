import asyncio
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Any, Tuple
import httpx

from .data import logger
from .data import process_library

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
    """Submit a task to the routine service via HTTP."""
    url = f"{ROUTINE_SERVICE_URL}/task"
    
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
    
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(url, json=payload)
            if response.status_code == 200:
                logger.log(f"Task submitted to routine: {function} on arm {arm_id} for cup {cup_id}")
                return True
            else:
                logger.log(f"Failed to submit task to routine: {response.text}")
                return False
    except Exception as e:
        logger.log(f"Error submitting task to routine: {str(e)}")
        return False

async def arm_worker(arm_name: str):
    """Worker thread for a robotic arm that executes tasks when they are ready."""
    global completed_count, failed_count
    while True:
        task = None
        # Find a pending task for this arm with all dependencies satisfied
        with lock:
            # Exit when all tasks are either completed or failed
            if (completed_count + failed_count) >= tasks_total:
                break
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
        if task:
            # Perform the task by sending it to the routine service
            action = task["action"]
            cup_id = task["cup"]
            drink = task["drink"]
            
            # Update status and notify
            update_status(f"Executing {action} for {drink} (cup {cup_id})")
            
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
                    # Note: We don't increment completed_count here anymore
                    # It will be incremented when we receive feedback from routine
            else:
                # Mark the task as failed but DON'T count as completed
                with lock:
                    task["status"] = "failed"
                    failed_tasks.append(task)
                    failed_count += 1
                    logger.log(f"Failed to submit task: {action} for cup {cup_id}")
        else:
            # No available task for this arm right now; small delay to avoid busy waiting
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

def update_status(message: str):
    """Update current status and call the callback if set."""
    global current_status
    logger.log(message)
    if status_callback:
        status_callback(message)

def register_status_callback(callback):
    """Register a callback function to be called on status updates."""
    global status_callback
    status_callback = callback

def setup_tasks_from_order(order_id: int, drinks: List[Dict[str, Any]], recipes: Dict[str, List[Dict[str, Any]]]):
    """Create task entries for an order received through the API."""
    global tasks, tasks_by_cup, completed, tasks_total, current_status, failed_tasks, failed_count
    
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
    
    # Use existing setup_tasks logic
    try:
        setup_tasks(orders, recipes)
        return True
    except Exception as e:
        logger.log(f"Error setting up tasks: {str(e)}")
        current_status.update({"status": "error", "step": str(e)})
        return False

async def process_order_async(order_id: int, drinks: List[Dict[str, Any]], recipes: Dict[str, List[Dict[str, Any]]]):
    """Process an order asynchronously using the scheduler."""
    global current_status, completed_count, tasks_total, failed_count
    
    # Setup tasks for the order
    if not setup_tasks_from_order(order_id, drinks, recipes):
        # Notify OMS about order failure
        await notify_oms_completion(order_id, False, "Failed to setup tasks for order")
        return False
    
    # Reset counters
    completed_count = 0
    
    # Start worker threads
    update_status(f"Starting to process order {order_id}")
    
    try:
        # Create and start arm workers
        arm1 = asyncio.create_task(arm_worker("Arm1"))
        arm2 = asyncio.create_task(arm_worker("Arm2"))
        
        # Wait for both arms to finish all tasks (either completed or failed)
        done, pending = await asyncio.wait(
            [arm1, arm2], 
            return_when=asyncio.ALL_COMPLETED
        )
        
        # Check if order was successful or failed
        with lock:
            if failed_count > 0:
                # Some tasks failed - mark order as failed
                current_status.update({"status": "error", "step": f"{failed_count} tasks failed"})
                update_status(f"Order {order_id} failed: {failed_count} out of {tasks_total} tasks failed")
                
                # Create failure reason listing failed tasks
                failed_task_names = [f"{task['action']} ({task['cup']})" for task in failed_tasks]
                reason = f"Failed tasks: {', '.join(failed_task_names)}"
                
                # Notify OMS about order failure
                await notify_oms_completion(order_id, False, reason)
                return False
            elif completed_count == tasks_total:
                # All tasks completed successfully
                current_status.update({"status": "completed", "step": None, "cup_index": None})
                update_status(f"Order {order_id} completed successfully")
                
                # Notify OMS about order completion
                await notify_oms_completion(order_id, True)
                return True
            else:
                # This shouldn't happen, but handle it as a failure
                reason = f"Unexpected state: {completed_count} completed, {failed_count} failed out of {tasks_total} total"
                current_status.update({"status": "error", "step": reason})
                update_status(f"Order {order_id} failed: {reason}")
                
                # Notify OMS about order failure
                await notify_oms_completion(order_id, False, reason)
                return False
                
    except Exception as e:
        # Notify OMS about order failure
        await notify_oms_completion(order_id, False, f"Order processing failed: {str(e)}")
        current_status.update({"status": "error", "step": str(e)})
        update_status(f"Order {order_id} failed: {e}")
        return False

def get_current_status():
    """Get the current processing status."""
    return current_status

def handle_routine_feedback(cup_id: str, action: str, success: bool):
    """Handle feedback from the routine service about task completion."""
    global completed_count, completed, failed_count, failed_tasks
    
    with lock:
        # Find the task in our list
        for task in tasks:
            if task["cup"] == cup_id and task["action"] == action:
                if success:
                    # Mark task as completed
                    task["status"] = "done"
                    completed[cup_id].add(action)
                    completed_count += 1
                    # Check if this was the final task for this cup
                    if len(completed[cup_id]) == len(tasks_by_cup[cup_id]):
                        message = f"Order complete: {task['drink']} for {cup_id}"
                        logger.log(f" --- {message} ---")
                        update_status(message)
                else:
                    # Mark task as failed and count in failed_count
                    task["status"] = "failed"
                    failed_tasks.append(task)
                    failed_count += 1
                    logger.log(f"Task failed: {action} for cup {cup_id}")
                break

# Helper function to notify OMS of order completion
async def notify_oms_completion(order_id: int, success: bool, reason: str = None):
    """Notify the OMS service that an order has completed or failed."""
    try:
        oms_url = "http://oms:8000"  # OMS service URL
        
        if success:
            # Order completed successfully
            endpoint = f"{oms_url}/orders/{order_id}/complete"
            async with httpx.AsyncClient() as client:
                response = await client.post(endpoint)
                logger.log(f"✅ Notified OMS: Order {order_id} completed successfully")
        else:
            # Order failed
            endpoint = f"{oms_url}/orders/{order_id}/fail"
            params = {"reason": reason or "Processing failed"}
            async with httpx.AsyncClient() as client:
                response = await client.post(endpoint, params=params)
                logger.log(f"❌ Notified OMS: Order {order_id} failed - {reason}")
                
    except Exception as e:
        logger.log(f"⚠️ Failed to notify OMS about order {order_id}: {e}")
