import asyncio
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Any, Tuple, Optional
from datetime import datetime
import httpx
import sys
import os

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from shared.logger import log

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
order_completion_logged = False  # Flag to prevent spam logging order completion
order_stopped_logged = False  # Flag to prevent spam logging order stopped
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

# Cup validation tracking - validates ingredients before starting a cup
# This runs in scheduler so both arms can work in parallel without waiting for validation
cup_validation_status = {}   # Map cup_id -> "pending" | "validating" | "validated" | "failed"
cup_validation_lock = asyncio.Lock()  # Async lock for validation status updates

# Global RabbitMQ client reference for notifications
_global_rabbitmq_client = None

def set_rabbitmq_client(client):
    """Set the global RabbitMQ client for use in notifications."""
    global _global_rabbitmq_client
    _global_rabbitmq_client = client
    log("DEBUG", "RabbitMQ client set", service="scheduler")


async def validate_cup_ingredients(cup_id: str) -> dict:
    """
    Validate that ingredients are available for a cup before starting its tasks.
    
    This function is called by the scheduler before starting any cup's tasks,
    removing the need for routine service to do validation (which was causing
    Arm 2 to wait for Arm 1's validation).
    
    Args:
        cup_id: The cup ID to validate ingredients for
        
    Returns:
        dict: {"success": bool, "passed": bool, "details": str, "missing_ingredients": list}
    """
    global _global_rabbitmq_client, cup_data_by_cup
    
    log("INFO", f"[CUP VALIDATION] Starting ingredient validation for cup {cup_id}", service="scheduler")
    
    try:
        # Get cup data from stored cup information
        with lock:
            cup_data = cup_data_by_cup.get(cup_id, {})
        
        if not cup_data:
            log("ERROR", f"[CUP VALIDATION] No cup data found for cup {cup_id}", service="scheduler")
            return {"success": False, "passed": False, "details": "Cup data not found"}
        
        # Extract ingredients from cup data
        ingredients = cup_data.get("ingredients", {})
        
        if not ingredients:
            log("WARNING", f"[CUP VALIDATION] No ingredients found for cup {cup_id}, skipping validation", service="scheduler")
            return {"success": True, "passed": True, "details": "No ingredients to validate"}
        
        # Build validation payload matching routine service format
        request_id = f"scheduler-{cup_id}-{time.time()}"
        payload = {
            "request_id": request_id,
            "client_type": "scheduler",
            "cup_id": cup_id,
            **ingredients  # Include all ingredients (milk, syrups, cups, espresso, etc.)
        }
        
        log("INFO", f"[CUP VALIDATION] Sending validation request for cup {cup_id}", service="scheduler")
        log("DEBUG", f"[CUP VALIDATION] Payload keys: {list(payload.keys())}", service="scheduler")
        
        if not _global_rabbitmq_client:
            log("ERROR", f"[CUP VALIDATION] No RabbitMQ client available for validation", service="scheduler")
            return {"success": False, "passed": False, "details": "RabbitMQ client not available"}
        
        # Call validation service - same handler that routine uses
        response = await _global_rabbitmq_client.send_request(
            target_service="validation",
            action="validate_ingredients",
            data=payload,
            timeout=30  # 30 second timeout for validation
        )
        
        log("INFO", f"[CUP VALIDATION] Validation response for cup {cup_id}: passed={response.get('passed', False)}", service="scheduler")
        
        if response.get("error"):
            log("ERROR", f"[CUP VALIDATION] Validation service error for cup {cup_id}: {response['error']}", service="scheduler")
            return {
                "success": False,
                "passed": False,
                "details": f"Validation service error: {response['error']}"
            }
        
        return {
            "success": True,
            "passed": response.get("passed", False),
            "details": response.get("details", ""),
            "missing_ingredients": response.get("missing_ingredients", [])
        }
        
    except asyncio.TimeoutError:
        log("ERROR", f"[CUP VALIDATION] Validation timeout for cup {cup_id}", service="scheduler")
        return {"success": False, "passed": False, "details": "Validation timeout"}
    except Exception as e:
        log("ERROR", f"[CUP VALIDATION] Exception during validation for cup {cup_id}: {str(e)[:100]}", service="scheduler")
        return {"success": False, "passed": False, "details": f"Validation exception: {str(e)}"}


async def send_validation_failure_to_dashboard(cup_id: str, validation_result: dict):
    """
    Send validation failure notification to dashboard.
    
    This mirrors what routine service does when validation fails,
    allowing the dashboard to show appropriate error messages.
    
    Args:
        cup_id: The cup ID that failed validation
        validation_result: The validation result containing failure details
    """
    global _global_rabbitmq_client
    
    try:
        if not _global_rabbitmq_client:
            log("WARNING", f"[DASHBOARD NOTIFY] No RabbitMQ client for dashboard notification", service="scheduler")
            return
        
        # Send validation failure event to dashboard
        # This matches the format used by routine service
        await _global_rabbitmq_client.send_event("validation.failed.dashboard", {
            "validation_function": "validate_ingredients",
            "cup_id": cup_id,
            "details": validation_result.get("details", ""),
            "missing_ingredients": validation_result.get("missing_ingredients", []),
            "timestamp": time.time()
        })
        
        log("INFO", f"[DASHBOARD NOTIFY] Sent validation failure to dashboard for cup {cup_id}", service="scheduler")
        
    except Exception as e:
        log("ERROR", f"[DASHBOARD NOTIFY] Failed to notify dashboard for cup {cup_id}: {str(e)[:100]}", service="scheduler")


async def handle_cup_validation_failure(cup_id: str, validation_result: dict):
    """
    Handle a cup validation failure by stopping tasks for that cup
    and notifying the dashboard.
    
    This allows the previous cup's tasks to finish while stopping
    only the failed cup's tasks.
    
    Args:
        cup_id: The cup ID that failed validation
        validation_result: The validation result containing failure details
    """
    global tasks, cup_completion_status, order_stopped, _global_rabbitmq_client
    
    log("INFO", f"[VALIDATION FAILURE] Handling validation failure for cup {cup_id}", service="scheduler")
    
    # Send notification to dashboard first
    await send_validation_failure_to_dashboard(cup_id, validation_result)
    
    # Mark cup as failed
    with lock:
        cup_completion_status[cup_id] = "failed"
        
        # Cancel all pending tasks for this cup
        cancelled_count = 0
        for task in tasks:
            if task["cup"] == cup_id and task["status"] == "pending":
                task["status"] = "cancelled"
                cancelled_count += 1
        
        log("INFO", f"[VALIDATION FAILURE] Cancelled {cancelled_count} pending tasks for cup {cup_id}", service="scheduler")
    
    # Extract order_id from cup_id for order-level handling
    order_id = None
    if '-' in cup_id:
        try:
            order_id = int(cup_id.split('-')[0])
        except ValueError:
            log("WARNING", f"[VALIDATION FAILURE] Could not parse order_id from cup_id: {cup_id}", service="scheduler")
    
    if order_id:
        # Check if there are any cups still in progress
        cups_in_progress = []
        with lock:
            for cid, status in cup_completion_status.items():
                if cid != cup_id and status == "in_progress":
                    cups_in_progress.append(cid)
        
        if cups_in_progress:
            log("INFO", f"[VALIDATION FAILURE] Cups still in progress: {cups_in_progress}. Letting them finish.", service="scheduler")
            log("INFO", f"[VALIDATION FAILURE] Setting order_stopped flag to stop new tasks after current cups complete", service="scheduler")
            
            # Set order_stopped so no new cups start, but let current cups finish
            with lock:
                order_stopped = True
            
            # We won't stop the order via OMS yet - we'll do that when the in-progress cups finish
            # This is handled by check_and_notify_order_completion
        else:
            log("INFO", f"[VALIDATION FAILURE] No cups in progress. Stopping order {order_id} immediately.", service="scheduler")
            
            # Stop the order via OMS (same as routine does)
            try:
                if _global_rabbitmq_client:
                    stop_response = await _global_rabbitmq_client.send_request(
                        target_service="oms",
                        action="stop_order",
                        data={"order_id": order_id},
                        timeout=30
                    )
                    
                    if stop_response and stop_response.get("success"):
                        log("INFO", f"[VALIDATION FAILURE] Successfully stopped order {order_id} via OMS", service="scheduler")
                    else:
                        error_msg = stop_response.get('error', 'Unknown') if stop_response else 'No response'
                        log("WARNING", f"[VALIDATION FAILURE] Failed to stop order {order_id} via OMS: {error_msg[:50]}", service="scheduler")
            except Exception as e:
                log("ERROR", f"[VALIDATION FAILURE] Exception stopping order {order_id}: {str(e)[:100]}", service="scheduler")
    
    # Emit validation failure event for dashboard tracking
    try:
        if _global_rabbitmq_client:
            await _global_rabbitmq_client.send_event("scheduler.cup_validation_failed", {
                "cup_id": cup_id,
                "order_id": order_id,
                "details": validation_result.get("details", ""),
                "missing_ingredients": validation_result.get("missing_ingredients", []),
                "timestamp": time.time()
            })
    except Exception as e:
        log("ERROR", f"[VALIDATION FAILURE] Failed to emit validation failed event: {str(e)[:100]}", service="scheduler")


async def validate_and_start_cup(cup_id: str, arm_name: str) -> bool:
    """
    Validate a cup's ingredients before starting its tasks.
    
    This is called when an arm is about to start working on a new cup.
    If validation fails, the cup's tasks are cancelled and the order is stopped.
    
    Args:
        cup_id: The cup ID to validate
        arm_name: The arm that will work on this cup (for logging)
        
    Returns:
        bool: True if validation passed and cup can start, False otherwise
    """
    global cup_validation_status, cup_validation_lock
    
    async with cup_validation_lock:
        # Check if this cup has already been validated
        validation_status = cup_validation_status.get(cup_id)
        
        if validation_status == "validated":
            log("DEBUG", f"[CUP VALIDATION] Cup {cup_id} already validated, proceeding", service="scheduler")
            return True
        
        if validation_status == "failed":
            log("DEBUG", f"[CUP VALIDATION] Cup {cup_id} already failed validation, skipping", service="scheduler")
            return False
        
        if validation_status == "validating":
            # Another arm is validating this cup, wait for result
            log("DEBUG", f"[CUP VALIDATION] Cup {cup_id} is being validated by another arm, waiting", service="scheduler")
            # Release lock and wait for validation to complete
            # This shouldn't normally happen with proper cup assignment
    
    # Mark cup as validating
    async with cup_validation_lock:
        cup_validation_status[cup_id] = "validating"
    
    log("INFO", f"[CUP VALIDATION] {arm_name} starting validation for cup {cup_id}", service="scheduler")
    
    # Perform validation
    validation_result = await validate_cup_ingredients(cup_id)
    
    if validation_result.get("passed", False):
        # Validation passed
        async with cup_validation_lock:
            cup_validation_status[cup_id] = "validated"
        log("INFO", f"[CUP VALIDATION] Cup {cup_id} passed validation, {arm_name} can proceed", service="scheduler")
        return True
    else:
        # Validation failed
        async with cup_validation_lock:
            cup_validation_status[cup_id] = "failed"
        log("ERROR", f"[CUP VALIDATION] Cup {cup_id} failed validation: {validation_result.get('details', '')}", service="scheduler")
        
        # Handle the failure (notify dashboard, stop tasks, etc.)
        await handle_cup_validation_failure(cup_id, validation_result)
        
        return False

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
        log("ERROR", f"Plan built event emit failed for order {order_id}: {str(e)[:50]}", service="scheduler")

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
        log("ERROR", f"Task progress event emit failed for {action} on cup {cup_id}: {str(e)[:50]}", service="scheduler")

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
    """Create task entries for each order based on the recipes.
    
    IMPORTANT: Tasks are added to the global tasks list in STRICT QUEUE ORDER.
    The order in which tasks are appended here determines the execution sequence
    for each arm. The scheduler will select tasks in this exact order (respecting
    dependencies and cup-priority rules).
    """
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
        log("DEBUG", "Per-arm task list formatted", service="scheduler")
    except Exception as e:
        log("DEBUG", "Per-arm list formatting failed", service="scheduler", error=str(e)[:50])

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
    
    STRICT QUEUE ORDER: Tasks are selected in the exact order they were added to
    the queue (tasks list), respecting dependencies and cup-priority rules.
    
    Priority order:
    1. Tasks from the cup this arm is currently working on (in queue order)
    2. Tasks from new cups (only if arm has no current cup AND no earlier cups have pending tasks, in queue order)
    
    Returns the selected task or None if no task is available.
    """
    global tasks, per_arm_current_cups, completed, cup_completion_status
    
    current_cup_id = per_arm_current_cups[arm_name]
    
    # Track the first pending cup we encounter (for strict cup priority)
    first_pending_cup_for_arm = None
    
    # Iterate through tasks in their original queue order
    # This ensures we maintain strict sequence adherence
    for task in tasks:
        # Skip if not for this arm
        if task["assigned_arm"] != arm_name:
            continue
        
        # Skip if not pending
        if task["status"] != "pending":
            continue
        
        cup_id = task["cup"]
        
        # Track first pending cup encountered (for strict cup priority)
        if first_pending_cup_for_arm is None:
            first_pending_cup_for_arm = cup_id
        
        # Check if dependencies are satisfied
        deps = task["depends_on"]
        deps_satisfied = all(dep in completed[cup_id] for dep in deps)
        
        # Priority 1: Task from current cup (first in queue with deps satisfied)
        if cup_id == current_cup_id:
            if deps_satisfied:
                task["status"] = "in_progress"
                log("DEBUG", f"{arm_name} continuing cup {cup_id} - {task['action']} (strict queue order)", service="scheduler")
                return task
            else:
                # Dependencies not met for current cup - wait for them
                continue
        
        # Priority 2: Task from new cup (only if arm has no current cup)
        if current_cup_id is None and deps_satisfied:
            # STRICT CUP PRIORITY: Only pick up this cup if it's the first pending cup we encountered
            # This prevents skipping cups that have tasks with unmet dependencies
            if cup_id == first_pending_cup_for_arm:
                # Check if cup is available to start
                if cup_completion_status[cup_id] in ["pending", "in_progress"]:
                    task["status"] = "in_progress"
                    
                    # Mark this cup as being worked on by this arm
                    per_arm_current_cups[arm_name] = cup_id
                    if cup_completion_status[cup_id] == "pending":
                        cup_completion_status[cup_id] = "in_progress"
                    
                    log("DEBUG", f"{arm_name} starting cup {cup_id} - {task['action']} (strict queue order)", service="scheduler")
                    return task
            # If this is not the first pending cup, don't pick it up (maintain cup priority)
    
    # No available tasks found in queue order
    if current_cup_id is not None:
        log("DEBUG", f"{arm_name} waiting for dependencies on cup {current_cup_id}", service="scheduler")
    elif first_pending_cup_for_arm is not None:
        log("DEBUG", f"{arm_name} waiting for dependencies on first pending cup {first_pending_cup_for_arm}", service="scheduler")
    else:
        log("DEBUG", f"{arm_name} waiting for available tasks", service="scheduler")
    
    return None

async def submit_task_to_routine(arm_id: str, function: str, cup_id: str, drink_type: str):
    """Submit a task to the routine service via RabbitMQ."""
    global cup_validation_status
    
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
        
        # Check if scheduler has already validated this cup's ingredients
        # This allows routine to skip the validate_ingredients step
        scheduler_validated = cup_validation_status.get(cup_id) == "validated"

        # Create the payload for the routine service
        payload = {
            "arm_id": int(arm_id.replace("Arm", "")),  # Convert "Arm1" to 1
            "function": function,
            "item": {
                "cup_id": cup_id,
                "drink_type": drink_type,
                "size": size,
                "addons": addons,
                "ingredients": ingredients,
                "scheduler_validated": scheduler_validated  # Flag to skip validation in routine
            }
        }
        
        if scheduler_validated:
            log("DEBUG", f"Cup {cup_id} marked as scheduler_validated - routine will skip validate_ingredients", service="scheduler")
        
        # Send request to routine service via RabbitMQ
        response = await client.send_request(
            target_service="routine",
            action="submit_task",
            data=payload,
            timeout=10
        )
        
        if response.get("success"):
            log("DEBUG", f"Task {function} submitted to routine for cup {cup_id} on {arm_id}", service="scheduler")
            return True
        else:
            log("ERROR", f"Routine task submission failed for {function} on cup {cup_id} ({arm_id}): {response.get('error', 'Unknown')[:50]}", service="scheduler")
            return False
            
    except Exception as e:
        log("ERROR", f"Routine task submission exception for {function} on cup {cup_id} ({arm_id}): {str(e)[:100]}", service="scheduler")
        return False
    finally:
        # Ensure client is disconnected even if an exception occurs
        if client:
            try:
                await client.disconnect()
            except Exception as disconnect_error:
                log("ERROR", "Disconnect failed", service="scheduler")

async def arm_worker(arm_name: str):
    """Worker thread for a robotic arm that executes tasks when they are ready.
    
    This worker is persistent and handles multiple orders sequentially without exiting.
    It waits for new tasks when the current order completes.
    
    VALIDATION INTEGRATION:
    When starting a NEW cup (not continuing an existing one), this worker validates
    the cup's ingredients BEFORE starting any tasks. This allows both arms to work
    in parallel without Arm 2 waiting for Arm 1's validation.
    """
    global completed_count, failed_count, order_stopped, tasks_total, order_completion_logged, order_stopped_logged
    log("INFO", "Worker started", service="scheduler", arm=arm_name)
    
    consecutive_no_work_count = 0
    max_idle_before_check = 100  # Check every 10 seconds if idle
    
    while True:
        task = None
        current_tasks_total = 0
        is_new_cup = False  # Track if we're starting a new cup
        
        # Find a pending task for this arm with all dependencies satisfied (Cup-Priority Algorithm)
        with lock:
            current_tasks_total = tasks_total
            
            # Check if order is stopped
            if order_stopped:
                # Don't pick up new tasks, but wait for submitted tasks to complete
                submitted_tasks = [t for t in tasks if t["status"] == "submitted"]
                if len(submitted_tasks) == 0:
                    # Only log once when stopped with no tasks
                    if not order_stopped_logged:
                        log("INFO", f"{arm_name} worker stopped - waiting for resume or new order", service="scheduler")
                        order_stopped_logged = True
                    task = None
                    # Don't exit - wait for resume or new order
                    # The worker will continue and check again in the next iteration
                else:
                    log("DEBUG", f"{arm_name} waiting for {len(submitted_tasks)} submitted tasks to complete", service="scheduler")
                    task = None
            # Check if current order is complete (all tasks done or failed)
            elif current_tasks_total > 0 and completed_count + failed_count >= current_tasks_total:
                # Only log once per order completion
                if not order_completion_logged:
                    log("INFO", f"{arm_name} worker finished - order complete (completed: {completed_count}, failed: {failed_count})", service="scheduler")
                    order_completion_logged = True
                task = None
                consecutive_no_work_count = 0  # Reset counter when order completes
                # Exit the worker when order completes so process_order_async can finish
                break
            # Active order with tasks to process
            elif current_tasks_total > 0:
                # Get current cup for this arm before selection
                current_cup_before = per_arm_current_cups.get(arm_name)
                task = select_task_with_per_arm_cup_priority(arm_name)
                
                # Check if we're starting a new cup
                if task:
                    current_cup_after = per_arm_current_cups.get(arm_name)
                    is_new_cup = (current_cup_before != current_cup_after and current_cup_after is not None)
                    
                    if is_new_cup:
                        log("INFO", f"[CUP VALIDATION] {arm_name} detected new cup start: {current_cup_after}", service="scheduler")
            # No active order - wait for tasks
            else:
                task = None
        
        # VALIDATION CHECK FOR NEW CUP (outside lock to allow async validation)
        if task and is_new_cup:
            cup_id = task["cup"]
            
            log("INFO", f"[CUP VALIDATION] {arm_name} validating ingredients before starting cup {cup_id}", service="scheduler")
            
            # Validate cup ingredients before starting
            validation_passed = await validate_and_start_cup(cup_id, arm_name)
            
            if not validation_passed:
                log("ERROR", f"[CUP VALIDATION] Cup {cup_id} failed validation - {arm_name} cannot start tasks", service="scheduler")
                
                # Reset the task status and arm's current cup
                with lock:
                    # Reset task to pending (it was set to in_progress by select_task)
                    task["status"] = "pending"
                    # Clear this arm's current cup so it can pick up next available cup
                    per_arm_current_cups[arm_name] = None
                
                # Skip this task and continue to look for another cup
                task = None
                
                # Small delay before trying again
                await asyncio.sleep(0.5)
                continue
            
            log("INFO", f"[CUP VALIDATION] Cup {cup_id} passed validation - {arm_name} proceeding with tasks", service="scheduler")
            
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
                    log("DEBUG", f"Task {action} submitted and awaiting completion for cup {cup_id} on {arm_name}", service="scheduler")
                # Emit in-progress update so dashboard marks it
                try:
                    import asyncio as _asyncio
                    loop = _asyncio.get_event_loop()
                    if loop.is_running():
                        loop.create_task(_emit_task_progress(cup_id, action))
                    else:
                        loop.run_until_complete(_emit_task_progress(cup_id, action))
                except Exception as _e:
                    log("ERROR", f"Task progress emit failed in {arm_name} worker: {str(_e)[:50]}", service="scheduler")
            else:
                # Mark the task as failed immediately
                with lock:
                    task["status"] = "failed"
                    failed_tasks.append(task)
                    failed_count += 1
                    log("ERROR", f"Task {action} submission to routine failed immediately for cup {cup_id} on {arm_name}", service="scheduler")
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
                        log("DEBUG", f"{arm_name} waiting - Pending: {len(pending_tasks)}, Submitted: {len(submitted_tasks)}, Completed: {len(completed_tasks)}, Failed: {len(failed_tasks_status)}", service="scheduler")
                    else:
                        log("DEBUG", f"{arm_name} idle - waiting for new orders", service="scheduler")
            
            # Workers are persistent - reset counter periodically to prevent overflow
            if consecutive_no_work_count >= 10000:  # Reset after ~1000 seconds (16 minutes) of idling
                consecutive_no_work_count = 0
                log("DEBUG", f"{arm_name} worker still active and waiting for orders", service="scheduler")
            
            # Small delay to avoid busy waiting and yield control to event loop
            await asyncio.sleep(0.1)

def run(order_file: str = 'data/orders.txt', recipe_file: str = 'data/recipes.json'):
    """Load orders and recipes, then run the simulation."""
    recipes = load_recipes(recipe_file)
    orders = parse_orders(order_file)
    if not orders:
        log("INFO", "No orders to process", service="scheduler")
        return
    setup_tasks(orders, recipes)
    log("INFO", f"Starting coffee order simulation for {len(orders)} orders", service="scheduler")
    
    # Run with asyncio since workers are now async
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_async())
    
    log("INFO", "All orders completed", service="scheduler")

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
    log("INFO", message, service="scheduler")
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
    
    log("INFO", f"Setting up tasks for order {order_id} with {len(drinks)} drinks", service="scheduler")
    
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
            log("WARNING", f"No recipe found for drink type '{drink_type}' in order {order_id}", service="scheduler")
            return False
        else:
            log("INFO", f"Found recipe for '{drink_type}' with {len(recipes[drink_type])} steps in order {order_id}", service="scheduler")
    
    # Use existing setup_tasks logic
    try:
        setup_tasks(orders, recipes)
        log("INFO", f"Created {tasks_total} tasks for order {order_id}", service="scheduler")
        # Emit plan immediately after building tasks so dashboard renders sequence early
        try:
            import asyncio as _asyncio
            loop = _asyncio.get_event_loop()
            if loop.is_running():
                loop.create_task(_emit_plan_built(order_id))
            else:
                loop.run_until_complete(_emit_plan_built(order_id))
        except Exception as _e:
            log("WARNING", f"Could not schedule plan_built emission for order {order_id}: {str(_e)[:50]}", service="scheduler")
        return True
    except Exception as e:
        log("ERROR", f"Error setting up tasks for order {order_id}: {str(e)[:100]}", service="scheduler")
        current_status.update({"status": "error", "step": str(e)})
        return False

async def process_order_async(order_id: int, drinks: List[Dict[str, Any]], recipes: Dict[str, List[Dict[str, Any]]]):
    """Process an order asynchronously using the scheduler."""
    global current_status, completed_count, tasks_total, failed_count, order_completion_notified, order_completion_logged
    
    log("DEBUG", f"process_order_async called for order {order_id} with {len(drinks)} drinks", service="scheduler")
    log("DEBUG", f"Available recipes: {list(recipes.keys()) if recipes else 'No recipes loaded'}", service="scheduler")
    log("DEBUG", f"Recipe count: {len(recipes) if recipes else 0}", service="scheduler")
    log("DEBUG", f"Drinks to process: {drinks}", service="scheduler")
    
    # Check if recipes is empty
    if not recipes:
        log("ERROR", f"No recipes available for order {order_id}", service="scheduler")
        await notify_oms_completion(order_id, False, "No recipes available")
        return False
    
    # Setup tasks for the order
    try:
        setup_result = setup_tasks_from_order(order_id, drinks, recipes)
        log("DEBUG", f"setup_tasks_from_order returned: {setup_result} for order {order_id}", service="scheduler")
        
        if not setup_result:
            log("ERROR", f"Failed to setup tasks for order {order_id}", service="scheduler")
            # Notify OMS about order failure
            await notify_oms_completion(order_id, False, "Failed to setup tasks for order")
            return False
    except Exception as e:
        log("ERROR", f"Exception in setup_tasks_from_order for order {order_id}: {str(e)[:100]}", service="scheduler")
        import traceback
        log("ERROR", f"Setup tasks exception traceback for order {order_id}: {traceback.format_exc()[:200]}", service="scheduler")
        await notify_oms_completion(order_id, False, f"Setup tasks exception: {str(e)}")
        return False
    
    log("INFO", f"Tasks setup successfully for order {order_id}. Total tasks: {tasks_total}", service="scheduler")
    
    # Reset counters and flags
    completed_count = 0
    order_completion_logged = False
    
    # Start worker threads
    await update_status(f"Starting to process order {order_id}")
    
    # Start heartbeat monitoring for this order
    heartbeat_task = None
    try:
        heartbeat_task = await start_order_heartbeat(order_id)
        log("INFO", f"Started heartbeat monitoring for order {order_id}", service="scheduler")
    except Exception as e:
        log("WARNING", f"Failed to start heartbeat for order {order_id}: {str(e)[:50]}", service="scheduler")
    
    try:
        # Create and start arm workers
        arm1 = asyncio.create_task(arm_worker("Arm1"))
        arm2 = asyncio.create_task(arm_worker("Arm2"))
        
        # Calculate dynamic timeout based on number of cups
        # Base timeout (2 minutes) + per-cup timeout (6.67 minutes per cup)
        base_timeout = 60.0  # 1 minute base
        per_cup_timeout = 360.0  # 6 minutes per cup (360 seconds)
        num_cups = len(drinks)
        dynamic_timeout = base_timeout + (per_cup_timeout * num_cups)
        
        log("INFO", f"Order {order_id} timeout set to {dynamic_timeout:.0f} seconds ({dynamic_timeout/60:.1f} minutes) for {num_cups} cups", service="scheduler")
        
        # Wait for both arms to finish with a pause-aware timeout
        # This timeout doesn't count time when order is stopped
        try:
            elapsed_processing_time = 0.0
            check_interval = 0.5  # Check every 0.5 seconds
            last_check_time = time.time()
            workers_done = False
            last_stopped_log_time = 0.0
            stopped_log_interval = 10.0  # Log stopped status every 10 seconds
            
            while not workers_done:
                # Check if workers are done
                if arm1.done() and arm2.done():
                    workers_done = True
                    log("INFO", f"Both arm workers completed for order {order_id}", service="scheduler")
                    break
                
                # Check if order is stopped (don't count this time against timeout)
                with lock:
                    is_stopped = order_stopped
                    is_notified = order_completion_notified
                
                # If order was completed/failed by feedback handler, exit
                if is_notified:
                    log("INFO", f"Order {order_id} was completed/failed by feedback handler", service="scheduler")
                    workers_done = True
                    break
                
                # Wait for check interval
                await asyncio.sleep(check_interval)
                current_time = time.time()
                
                # Only count elapsed time if order is not stopped
                if not is_stopped:
                    elapsed_processing_time += (current_time - last_check_time)
                    last_stopped_log_time = 0.0  # Reset stopped log counter when resuming
                else:
                    # Log stopped status periodically (not every iteration)
                    if last_stopped_log_time == 0.0 or (current_time - last_stopped_log_time) >= stopped_log_interval:
                        log("INFO", f"Order {order_id} is stopped - timeout paused at {elapsed_processing_time:.1f}s", service="scheduler")
                        last_stopped_log_time = current_time
                
                last_check_time = current_time
                
                # Check if we've exceeded timeout (only counting non-stopped time)
                if elapsed_processing_time > dynamic_timeout:
                    log("ERROR", f"Order {order_id} timed out after {elapsed_processing_time/60:.1f} minutes of processing time", service="scheduler")
                    # Cancel both arms
                    arm1.cancel()
                    arm2.cancel()
                    await notify_oms_completion(order_id, False, f"Order processing timed out after {elapsed_processing_time/60:.1f} minutes")
                    return False
            
        except Exception as wait_error:
            log("ERROR", f"Error in wait loop for order {order_id}: {str(wait_error)[:100]}", service="scheduler")
            arm1.cancel()
            arm2.cancel()
            raise
        
        # Check if order was successful or failed
        with lock:
            if order_completion_notified:
                # Completion already notified by feedback handler
                log("INFO", f"Order {order_id} completion already notified by feedback handler", service="scheduler")
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
                        log("WARNING", f"Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay", service="scheduler")
                        await asyncio.sleep(delay)
                    
                    notification_success = await notify_oms_completion(order_id, False, reason)
                    if notification_success:
                        break
                
                if notification_success:
                    order_completion_notified = True
                    log("INFO", f"Order {order_id} failure notification confirmed", service="scheduler")
                else:
                    log("ERROR", f"Failed to notify OMS of order {order_id} failure after 3 attempts", service="scheduler")
                
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
                        log("WARNING", f"Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay", service="scheduler")
                        await asyncio.sleep(delay)
                    
                    notification_success = await notify_oms_completion(order_id, True)
                    if notification_success:
                        break
                
                if notification_success:
                    order_completion_notified = True
                    log("INFO", f"Order {order_id} completion notification confirmed", service="scheduler")
                else:
                    log("ERROR", f"Failed to notify OMS of order {order_id} completion after 3 attempts", service="scheduler")
                
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
                        log("WARNING", f"Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay", service="scheduler")
                        await asyncio.sleep(delay)
                    
                    notification_success = await notify_oms_completion(order_id, False, reason)
                    if notification_success:
                        break
                
                if notification_success:
                    order_completion_notified = True
                    log("INFO", f"Order {order_id} failure notification confirmed", service="scheduler")
                else:
                    log("ERROR", f"Failed to notify OMS of order {order_id} failure after 3 attempts", service="scheduler")
                
                return False
                
    except Exception as e:
        # Notify OMS about order failure with retry
        current_status.update({"status": "error", "step": str(e)})
        await update_status(f"Order {order_id} failed: {e}")
        
        notification_success = False
        for attempt in range(3):
            if attempt > 0:
                delay = 2 ** attempt
                log("WARNING", f"Retry attempt {attempt + 1}/3 for order {order_id} exception notification after {delay}s delay", service="scheduler")
                await asyncio.sleep(delay)
            
            notification_success = await notify_oms_completion(order_id, False, f"Order processing failed: {str(e)}")
            if notification_success:
                break
        
        if notification_success:
            with lock:
                order_completion_notified = True
            log("INFO", f"Order {order_id} exception notification confirmed", service="scheduler")
        else:
            log("ERROR", f"Failed to notify OMS of order {order_id} exception after 3 attempts", service="scheduler")
        
        return False
    finally:
        # Clean up heartbeat task
        if heartbeat_task and not heartbeat_task.done():
            heartbeat_task.cancel()
            try:
                await heartbeat_task
            except asyncio.CancelledError:
                pass
            log("INFO", f"Stopped heartbeat monitoring for order {order_id}", service="scheduler")

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
    
    log("INFO", f"Updating cup position for {cup_id} to {new_position}", service="scheduler")
    
    with lock:
        # Update in cup_data_by_cup (master data for this cup)
        if cup_id not in cup_data_by_cup:
            log("WARNING", f"Cup {cup_id} not found in cup_data_by_cup", service="scheduler")
            return False
        
        cup_data = cup_data_by_cup[cup_id]
        
        # Update position in ingredients
        if "ingredients" in cup_data:
            if "position" in cup_data["ingredients"]:
                old_position = cup_data["ingredients"]["position"].get("cup_position")
                cup_data["ingredients"]["position"]["cup_position"] = new_position
                log("INFO", f"Updated cup_data_by_cup[{cup_id}]['ingredients']['position']['cup_position']: {old_position} to {new_position}", service="scheduler")
            else:
                # Create position dict if it doesn't exist
                cup_data["ingredients"]["position"] = {"cup_position": new_position}
                log("INFO", f"Created position entry in cup_data_by_cup[{cup_id}]", service="scheduler")
        else:
            log("WARNING", f"No ingredients found in cup_data for {cup_id}", service="scheduler")
        
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
        
        log("INFO", f"Updated {updated_task_count} pending tasks for cup {cup_id}", service="scheduler")
        log("INFO", f"Future tasks for cup {cup_id} will use position {new_position}", service="scheduler")
        
        return True

async def handle_routine_feedback(cup_id: str, action: str, success: bool):
    """
    Handle feedback from the routine service about task completion.
    
    When a task fails, immediately notifies OMS and cancels remaining tasks
    instead of waiting for all tasks to complete.
    """
    global completed_count, completed, failed_count, failed_tasks, order_completion_notified, per_arm_current_cups, cup_completion_status
    
    log("INFO", f"Processing feedback: {action} for {cup_id} - {'SUCCESS' if success else 'FAILED'}", service="scheduler")
    
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
                        log("INFO", f"Task completed: {action} for cup {cup_id}", service="scheduler")
                        
                        arm_name = task["assigned_arm"]
                        
                        # Check if this arm has completed all its tasks for this cup
                        arm_tasks_for_cup = [t for t in tasks_by_cup[cup_id] if t["assigned_arm"] == arm_name]
                        # Count remaining tasks (any status other than done/failed/cancelled)
                        arm_remaining_tasks = [t for t in arm_tasks_for_cup if t["status"] not in ["done", "failed", "cancelled"]]
                        
                        # Only release arm if NO tasks remain (includes submitted tasks)
                        if len(arm_remaining_tasks) == 0:
                            # This arm has finished all its tasks for this cup - release it to work on next cup
                            if arm_name in per_arm_current_cups and per_arm_current_cups[arm_name] == cup_id:
                                per_arm_current_cups[arm_name] = None
                                log("INFO", f"{arm_name} completed all its tasks for cup {cup_id} - ready for next cup", service="scheduler")
                        
                        # Check if entire cup is complete (all tasks from both arms)
                        if len(completed[cup_id]) == len(tasks_by_cup[cup_id]):
                            update_message = f"Order complete: {task['drink']} for {cup_id}"
                            cup_completion_status[cup_id] = "completed"
                            log("INFO", f"Cup {cup_id} fully completed (all arms finished)", service="scheduler")
                    else:
                        # Mark task as failed
                        task["status"] = "failed"
                        failed_tasks.append(task)
                        failed_count += 1
                        log("ERROR", f"Task failed: {action} for cup {cup_id}", service="scheduler")
                        
                        # Mark cup as failed and remove from arm's current cup
                        arm_name = task["assigned_arm"]
                        if arm_name in per_arm_current_cups and per_arm_current_cups[arm_name] == cup_id:
                            per_arm_current_cups[arm_name] = None
                        cup_completion_status[cup_id] = "failed"
                        log("ERROR", f"Cup {cup_id} failed due to {action} task failure", service="scheduler")
                        
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
                    log("WARNING", f"Duplicate feedback ignored for {action} on cup {cup_id}", service="scheduler")
                    return
                else:
                    log("WARNING", f"Task not found: {action} for cup {cup_id}", service="scheduler")
        
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
                log("INFO", f"Notifying OMS of order {order_id} failure", service="scheduler")
                
                # Retry notification up to 3 times with exponential backoff
                notification_success = False
                for attempt in range(3):
                    if attempt > 0:
                        delay = 2 ** attempt  # 2, 4 seconds
                        log("WARNING", f"Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay", service="scheduler")
                        await asyncio.sleep(delay)
                    
                    notification_success = await notify_oms_completion(order_id, False, reason)
                    if notification_success:
                        break
                
                # Only mark as notified if we successfully delivered the notification
                if notification_success:
                    with lock:
                        order_completion_notified = True
                    log("INFO", f"Order {order_id} failure notification confirmed after {attempt + 1} attempt(s)", service="scheduler")
                    # State will be reset when next order starts
                else:
                    log("ERROR", f"Failed to notify OMS of order {order_id} failure after 3 attempts", service="scheduler")
                
                # Cancel remaining tasks
                with lock:
                    for task in tasks:
                        if task["status"] not in ["done", "failed"]:
                            task["status"] = "cancelled"
        
        # Check for order completion
        elif should_check_order_completion and not order_completion_notified:
            log("DEBUG", f"Calling check_and_notify_order_completion for order_id={current_status.get('order_id')}, should_check={should_check_order_completion}, notified={order_completion_notified}", service="scheduler")
            await check_and_notify_order_completion()
        elif not task_found and not order_completion_notified:
            log("DEBUG", f"Task not found, calling check_and_notify_order_completion for order_id={current_status.get('order_id')}, task_found={task_found}, notified={order_completion_notified}", service="scheduler")
            await check_and_notify_order_completion()
        else:
            log("DEBUG", f"Skipping completion check - should_check={should_check_order_completion}, task_found={task_found}, notified={order_completion_notified}", service="scheduler")
        
    except Exception as e:
        log("ERROR", f"Error in feedback processing: {str(e)[:100]}", service="scheduler")
        raise

def reset_scheduler_state_sync():
    """Reset scheduler state synchronously (called at start of new order setup).
    
    This function resets the scheduler state in preparation for a new order.
    It should be called from within setup_tasks_from_order to avoid race conditions.
    """
    global tasks, tasks_by_cup, completed, failed_tasks, failed_count, completed_count
    global tasks_total, order_completion_notified, order_stopped, order_stopped_logged, order_completion_logged
    global per_arm_current_cups, cup_completion_status, cup_data_by_cup, cup_validation_status
    
    try:
        log("INFO", "Resetting scheduler state for new order", service="scheduler")
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
        order_stopped_logged = False
        order_completion_logged = False
        per_arm_current_cups.clear()
        cup_completion_status.clear()
        cup_data_by_cup.clear()
        cup_validation_status.clear()  # Reset cup validation tracking for new order
        
        # Keep current_status for debugging, but mark as idle
        current_status.update({
            "order_id": None,
            "status": "idle",
            "step": None,
            "cup_index": None
        })
        log("INFO", "State reset complete - ready for new order", service="scheduler")
    except Exception as e:
        log("ERROR", f"Error resetting state: {str(e)[:100]}", service="scheduler")
        import traceback
        log("ERROR", f"Traceback: {traceback.format_exc()[:200]}", service="scheduler")

async def revert_previous_step_for_cup(cup_id: str, current_action: str):
    """
    Revert the previous completed step for a cup to pending status.
    This allows the recipe to retry from the previous step when validation fails.
    
    Args:
        cup_id: The cup ID to revert steps for
        current_action: The current action that failed (validation step)
    
    Returns:
        dict with success status and reverted action name if found
    """
    global completed, tasks, completed_count
    
    log("INFO", f"[REVERT STEP] Starting revert process for cup {cup_id}", service="scheduler")
    log("INFO", f"[REVERT STEP] Current action: {current_action}", service="scheduler")
    
    try:
        with lock:
            # Get all tasks for this cup in order
            cup_tasks = [t for t in tasks if t["cup"] == cup_id]
            log("INFO", f"[REVERT STEP] Found {len(cup_tasks)} tasks for cup {cup_id}", service="scheduler")
            
            # Find the current task (may be in submitted or in_progress status)
            current_task = None
            current_task_index = -1
            log("INFO", f"[REVERT STEP] Searching for current task: {current_action}", service="scheduler")
            
            for idx, task in enumerate(cup_tasks):
                if task["action"] == current_action and task["status"] in ["submitted", "in_progress", "pending"]:
                    current_task = task
                    current_task_index = idx
                    log("INFO", f"[REVERT STEP] Found current task at index {idx}, status: {task['status']}", service="scheduler")
                    break
            
            # If not found, try to find by action only (might be in different status)
            if current_task_index == -1:
                log("INFO", f"[REVERT STEP] Task not found with expected status, searching by action only", service="scheduler")
                for idx, task in enumerate(cup_tasks):
                    if task["action"] == current_action:
                        current_task = task
                        current_task_index = idx
                        log("INFO", f"[REVERT STEP] Found current task at index {idx}, status: {task['status']}", service="scheduler")
                        break
            
            if current_task_index == -1:
                log("WARNING", f"[REVERT STEP] Current task {current_action} not found for cup {cup_id}", service="scheduler")
                return {"success": False, "error": "Current task not found"}
            
            # Mark current task as pending so it will be retried on resume
            # Routine has saved progress of completed sub-steps, so it will resume from where it failed
            if current_task["status"] in ["submitted", "in_progress"]:
                log("INFO", f"[REVERT STEP] Resetting current task '{current_action}' to 'pending' for retry with sub-step resume", service="scheduler")
                current_task["status"] = "pending"
                log("INFO", f"[REVERT STEP] Routine will resume this task from the failed sub-step using saved progress", service="scheduler")
            else:
                log("INFO", f"[REVERT STEP] Current task '{current_action}' has status '{current_task['status']}', setting to pending", service="scheduler")
                current_task["status"] = "pending"
            
            # Don't revert previous tasks - just retry the current task from its saved checkpoint
            # This prevents unnecessary rework and ensures precise sub-step resumption
            log("INFO", f"[REVERT STEP] Task '{current_action}' will retry from saved checkpoint in routine", service="scheduler")
            
            return {
                "success": True,
                "reverted_action": None,
                "message": f"Task reset to pending, will resume from sub-step checkpoint"
            }
            
    except Exception as e:
        log("ERROR", f"Error reverting previous step for cup {cup_id}: {str(e)[:100]}", service="scheduler")
        return {"success": False, "error": str(e)}

async def check_and_notify_order_completion():
    """Check if the current order is complete and notify OMS if so."""
    global completed_count, failed_count, tasks_total, current_status, order_completion_notified, order_stopped
    
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
        is_stopped = order_stopped
        
        log("DEBUG", f"check_and_notify_order_completion called - order_id={order_id}, completed={completed_tasks}, failed={failed_tasks_count}, cancelled={cancelled_tasks}, total={current_tasks_total}, notified={already_notified}, stopped={is_stopped}", service="scheduler")
        
        # Only proceed if we have an order_id and not already notified
        if not order_id or already_notified:
            log("DEBUG", f"Early return - no order_id ({not order_id}) or already notified ({already_notified})", service="scheduler")
            return
        
        # If order was stopped, don't check for completion - it's handled separately
        if is_stopped:
            log("DEBUG", f"Order was stopped - skipping completion check", service="scheduler")
            return
            
        # If not all tasks are finished and no failures, continue waiting
        if total_finished < current_tasks_total and failed_tasks_count == 0:
            log("DEBUG", f"Waiting for more tasks - finished({total_finished}) < total({current_tasks_total}) and no failures", service="scheduler")
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
        
        log("INFO", f"Notifying OMS of order {order_id} failure", service="scheduler")
        
        # Retry notification up to 3 times with exponential backoff
        notification_success = False
        for attempt in range(3):
            if attempt > 0:
                delay = 2 ** attempt  # 2, 4 seconds
                log("WARNING", f"Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay", service="scheduler")
                await asyncio.sleep(delay)
            
            notification_success = await notify_oms_completion(order_id, False, reason)
            if notification_success:
                break
        
        if notification_success:
            with lock:
                order_completion_notified = True
            log("INFO", f"Order {order_id} failure notification confirmed after {attempt + 1} attempt(s)", service="scheduler")
            # State will be reset when next order starts
        else:
            log("ERROR", f"Failed to notify OMS of order {order_id} failure after 3 attempts", service="scheduler")
        
    elif completed_tasks == current_tasks_total:
        # All tasks completed successfully
        with lock:
            current_status.update({"status": "completed", "step": None, "cup_index": None})
        await update_status(f"Order {order_id} completed successfully")
        
        log("INFO", f"Notifying OMS of order {order_id} completion", service="scheduler")
        
        # Retry notification up to 3 times with exponential backoff
        notification_success = False
        for attempt in range(3):
            if attempt > 0:
                delay = 2 ** attempt  # 2, 4 seconds
                log("WARNING", f"Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay", service="scheduler")
                await asyncio.sleep(delay)
            
            notification_success = await notify_oms_completion(order_id, True)
            if notification_success:
                break
        
        if notification_success:
            with lock:
                order_completion_notified = True
            log("INFO", f"Order {order_id} completion notification confirmed after {attempt + 1} attempt(s)", service="scheduler")
            # State will be reset when next order starts
        else:
            log("ERROR", f"Failed to notify OMS of order {order_id} completion after 3 attempts", service="scheduler")
        
    else:
        # Unexpected state
        reason = f"Unexpected state: {completed_tasks} completed, {failed_tasks_count} failed, {cancelled_tasks} cancelled out of {current_tasks_total} total"
        with lock:
            current_status.update({"status": "error", "step": reason})
        await update_status(f"Order {order_id} failed: {reason}")
        
        log("INFO", f"Notifying OMS of order {order_id} unexpected failure", service="scheduler")
        
        # Retry notification up to 3 times with exponential backoff
        notification_success = False
        for attempt in range(3):
            if attempt > 0:
                delay = 2 ** attempt  # 2, 4 seconds
                log("WARNING", f"Retry attempt {attempt + 1}/3 for order {order_id} notification after {delay}s delay", service="scheduler")
                await asyncio.sleep(delay)
            
            notification_success = await notify_oms_completion(order_id, False, reason)
            if notification_success:
                break
        
        if notification_success:
            with lock:
                order_completion_notified = True
            log("INFO", f"Order {order_id} failure notification confirmed after {attempt + 1} attempt(s)", service="scheduler")
            # State will be reset when next order starts
        else:
            log("ERROR", f"Failed to notify OMS of order {order_id} failure after 3 attempts", service="scheduler")

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
                        log("INFO", f"Using service instance RabbitMQ client as fallback for order {order_id}", service="scheduler")
                    else:
                        log("WARNING", f"Service instance has no rabbitmq_client, skipping notification for order {order_id}", service="scheduler")
                        return False
                else:
                    log("WARNING", f"No service instance available, skipping notification for order {order_id}", service="scheduler")
                    return False
            except Exception as fallback_error:
                log("ERROR", f"Fallback client access failed for order {order_id}: {str(fallback_error)[:50]}", service="scheduler")
                return False
        
        if not client:
            log("WARNING", f"No RabbitMQ client available, skipping notification for order {order_id}", service="scheduler")
            return False
            
        log("INFO", f"Using RabbitMQ client for order {order_id} notification", service="scheduler")
        
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
                log("INFO", f"Order {order_id} {'completion' if success else 'failure'} confirmed by OMS", service="scheduler")
                return True
            else:
                error_msg = result.get("error", "Unknown error")
                log("ERROR", f"OMS did not acknowledge order {order_id} notification: {error_msg[:100]}", service="scheduler")
                return False
                
        except Exception as send_error:
            log("ERROR", f"Error sending acknowledged event for order {order_id}: {str(send_error)[:100]}", service="scheduler")
            import traceback
            log("ERROR", f"Traceback: {traceback.format_exc()[:200]}", service="scheduler")
            return False
        
    except Exception as e:
        log("ERROR", f"Failed to notify OMS for order {order_id}: {str(e)[:100]}", service="scheduler")
        import traceback
        log("ERROR", f"Traceback: {traceback.format_exc()[:200]}", service="scheduler")
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
        log("DEBUG", f"Heartbeat sent for order {order_id}: {status}", service="scheduler")
        
    except Exception as e:
        log("WARNING", f"Failed to send heartbeat for order {order_id}: {str(e)[:50]}", service="scheduler")

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
                    is_stopped = order_stopped
                    
                    progress = {
                        "total_tasks": total_tasks,
                        "completed_tasks": completed_tasks,
                        "failed_tasks": failed_tasks,
                        "completion_percentage": (completed_tasks / total_tasks * 100) if total_tasks > 0 else 0
                    }
                
                # Send appropriate status based on whether order is stopped
                status = "stopped" if is_stopped else "processing"
                await send_order_heartbeat(order_id, status, progress)
                await asyncio.sleep(30)  # Send heartbeat every 30 seconds
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                log("WARNING", f"Heartbeat loop error for order {order_id}: {str(e)[:50]}", service="scheduler")
                await asyncio.sleep(30)
    
    # Start heartbeat task
    heartbeat_task = asyncio.create_task(heartbeat_loop())
    return heartbeat_task
