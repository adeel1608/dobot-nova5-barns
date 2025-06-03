# services/oms/app.py
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, BackgroundTasks, HTTPException, Path, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from contextlib import asynccontextmanager
from . import db, queue, models  # hypothetical internal modules
import httpx

# Additional models for request/response data
class TaskCreate(BaseModel):
    order_id: int
    item_id: int
    arm_id: int
    function_name: str

class TaskStatusUpdate(BaseModel):
    status: str
    error_message: Optional[str] = None

class TaskStepCreate(BaseModel):
    task_id: int
    step_index: int
    step_type: str
    function_name: str
    params: Dict[str, Any]
    status: str = "pending"

class TaskStepStatusUpdate(BaseModel):
    status: str
    error_message: Optional[str] = None

class EventCreate(BaseModel):
    event_type: str
    payload: Dict[str, Any]

class AlertCreate(BaseModel):
    event_id: int
    alert_type: str
    severity: str

class SystemStopRequest(BaseModel):
    reason: Optional[str] = ""

class ThresholdWarning(BaseModel):
    ingredient: str
    severity: str = Field(..., regex="^(low|medium|high)$")

class InventoryRefill(BaseModel):
    ingredient: str

# Order status constants
ORDER_STATUS = {
    'QUEUED': 'queued',
    'PROCESSING': 'processing', 
    'COMPLETED': 'completed',
    'HALTED': 'halted',         # Paused due to validation/ingredient issues
    'STOPPED': 'stopped',       # Manually stopped
    'ERROR': 'error',           # Failed with error
    'CANCELLED': 'cancelled'    # Cancelled by user
}

# Task status constants  
TASK_STATUS = {
    'QUEUED': 'queued',
    'RUNNING': 'running',
    'COMPLETED': 'completed',
    'FAILED': 'failed',
    'HALTED': 'halted'         # Waiting for validation/manual intervention
}

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    db.connect()        # Connect to PostgreSQL
    queue.connect()     # Connect to Redis
    
    # Sync queue with database on startup
    print("Syncing queue with database on startup...")
    queue.sync_with_database()
    
    yield
    # Shutdown
    # Add any cleanup code here if needed

app = FastAPI(title="Order Management Service", lifespan=lifespan)

# Add CORS middleware to allow dashboard access
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Dashboard origin
        "http://127.0.0.1:3000",
        "http://localhost:3001",  # Allow alternative ports
        "http://127.0.0.1:3001"
    ],
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
)

# In-memory list of websocket connections for broadcasting (simple approach)
active_connections: list[WebSocket] = []
alert_connections: list[WebSocket] = []

# Order Endpoints

@app.post("/orders/")
def create_order(order: models.Order):
    """Add a new order to the queue."""
    order_id = db.save_order(order)            # save order to PostgreSQL (status = 'queued')
    queue.add_order(order_id)                  # add order ID to Redis queue list
    # Publish an event to RabbitMQ: "order.received"
    # ... (RabbitMQ publish code)
    broadcast({"event": "order_received", "order": order_id, "status": "queued"})
    return {"order_id": order_id, "status": "queued"}

@app.get("/orders/")
def list_orders(status: Optional[str] = None):
    """Retrieve orders, optionally filtered by status."""
    orders = db.get_orders(status=status)      # fetch from DB (joined with queue info for ordering)
    return {"orders": orders}

@app.get("/orders/{order_id}")
def get_order(order_id: int = Path(..., title="The ID of the order to retrieve")):
    """Retrieve a specific order by ID with detailed task and step information."""
    order = db.get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")
    
    # Get detailed task information for this order
    try:
        tasks = db.get_order_tasks(order_id)  # Get all tasks for this order
        
        # For each task, get the steps
        for task in tasks:
            task['steps'] = db.get_task_steps(task['id'])
            
        order['tasks'] = tasks
        order['task_summary'] = {
            'total_tasks': len(tasks),
            'completed_tasks': len([t for t in tasks if t['status'] == 'completed']),
            'failed_tasks': len([t for t in tasks if t['status'] == 'failed']),
            'running_tasks': len([t for t in tasks if t['status'] == 'running']),
            'halted_tasks': len([t for t in tasks if t['status'] == 'halted'])
        }
        
        # Add scheduling timeline
        order['timeline'] = []
        
        # Order received
        order['timeline'].append({
            'step': 'Order Received',
            'status': 'completed',
            'timestamp': order.get('created_at'),
            'description': 'Order was placed and queued for processing'
        })
        
        # Processing started
        if order.get('started_at'):
            order['timeline'].append({
                'step': 'Processing Started',
                'status': 'completed',
                'timestamp': order.get('started_at'),
                'description': 'Order processing began'
            })
        
        # Add task-level steps to timeline
        for task in tasks:
            task_desc = f"Task: {task['function_name']} (Arm {task['arm_id']})"
            task_status = 'completed' if task['status'] == 'completed' else 'in_progress' if task['status'] == 'running' else 'pending'
            
            if task['status'] in ['failed', 'halted']:
                task_status = 'error'
                task_desc += f" - {task.get('error_message', 'Error occurred')}"
            
            order['timeline'].append({
                'step': task_desc,
                'status': task_status,
                'timestamp': task.get('started_at') or task.get('queued_at'),
                'description': f"Status: {task['status']}"
            })
            
            # Add detailed steps if they exist
            for step in task.get('steps', []):
                step_status = 'completed' if step['status'] in ['passed', 'completed'] else 'error' if step['status'] == 'failed' else 'in_progress' if step['status'] == 'running' else 'pending'
                order['timeline'].append({
                    'step': f"  └─ {step['function_name']}",
                    'status': step_status,
                    'timestamp': step.get('started_at'),
                    'description': step.get('error_message') or f"Step {step['status']}"
                })
        
        # Order completed
        if order.get('completed_at'):
            final_status = 'completed' if order['status'] == 'completed' else 'error'
            order['timeline'].append({
                'step': 'Order Completed',
                'status': final_status,
                'timestamp': order.get('completed_at'),
                'description': f"Order finished with status: {order['status']}"
            })
            
    except Exception as e:
        print(f"Error fetching task details for order {order_id}: {e}")
        order['tasks'] = []
        order['task_summary'] = {'total_tasks': 0, 'completed_tasks': 0, 'failed_tasks': 0, 'running_tasks': 0, 'halted_tasks': 0}
        order['timeline'] = []
    
    return order

@app.put("/orders/{order_id}/reorder")
def reorder_queue(order_id: int, new_position: int):
    """Reorder an existing queued order."""
    queue.reorder(order_id, new_position)      # update order position in Redis queue
    # Broadcast the updated queue order to clients
    broadcast({"event": "queue_reordered", "order": order_id, "new_position": new_position})
    return {"msg": "reordered", "order": order_id, "position": new_position}

@app.put("/orders/reorder")
def bulk_reorder_queue(order_data: dict):
    """Bulk reorder the entire queue with new order IDs sequence."""
    try:
        order_ids = order_data.get("order_ids", [])
        if not order_ids:
            raise HTTPException(status_code=400, detail="order_ids list is required")
        
        # Update the queue order in Redis
        queue.bulk_reorder(order_ids)
        
        # Broadcast the updated queue order to clients
        broadcast({"event": "queue_bulk_reordered", "order_ids": order_ids})
        return {"msg": "queue_reordered", "order_count": len(order_ids)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to reorder queue: {str(e)}")

@app.patch("/orders/{order_id}/start")
def start_order(order_id: int, background_tasks: BackgroundTasks):
    """Mark order as processing and send it to Scheduler."""
    order = db.get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")
    db.update_order_status(order_id, "processing")
    queue.remove(order_id)
    # Notify via events and WebSocket
    broadcast({"event": "order_started", "order": order_id})
    # Send order to Scheduler for processing (async to avoid blocking response)
    background_tasks.add_task(send_to_scheduler, order)
    return {"msg": "order_sent_to_scheduler", "order": order_id}

@app.patch("/orders/{order_id}/status")
def update_order_status(
    order_id: int = Path(..., title="The ID of the order to update"),
    status: str = Query(..., title="The new status to set"),
    reason: Optional[str] = Query(None, title="Reason for status change")
):
    """Update the status of an order with optional reason."""
    order = db.get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")
    
    # Validate status
    valid_statuses = list(ORDER_STATUS.values())
    if status not in valid_statuses:
        raise HTTPException(status_code=400, detail=f"Invalid status. Must be one of: {valid_statuses}")
    
    # Update order status with reason
    db.update_order_status(order_id, status, reason)
    
    # Log the status change event
    db.log_event("order_status_changed", {
        "order_id": order_id,
        "old_status": order.get("status"),
        "new_status": status,
        "reason": reason,
        "timestamp": "now"
    })
    
    # Broadcast the status update
    broadcast({
        "event": "order_status_updated", 
        "order": order_id, 
        "status": status,
        "reason": reason
    })
    
    return {"msg": "status_updated", "order": order_id, "status": status, "reason": reason}

@app.post("/orders/{order_id}/halt")
def halt_order(
    order_id: int = Path(..., title="The ID of the order to halt"),
    reason: str = Query(..., title="Reason for halting the order")
):
    """Halt an order due to validation issues or manual intervention required."""
    order = db.get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")
    
    # Update status to halted
    db.update_order_status(order_id, ORDER_STATUS['HALTED'], reason)
    
    # Create an alert for the halted order
    event_id = db.log_event("order_halted", {
        "order_id": order_id,
        "reason": reason,
        "timestamp": "now"
    })
    alert_id = db.create_alert(event_id, "order_halted", "warning")
    
    # Broadcast the halt event
    broadcast({
        "event": "order_halted",
        "order": order_id,
        "reason": reason,
        "alert_id": alert_id
    })
    
    return {"msg": "order_halted", "order": order_id, "reason": reason, "alert_id": alert_id}

@app.post("/orders/{order_id}/resume")
def resume_order(order_id: int = Path(..., title="The ID of the order to resume")):
    """Resume a halted order."""
    order = db.get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")
    
    if order.get("status") != ORDER_STATUS['HALTED']:
        raise HTTPException(status_code=400, detail="Order is not in halted state")
    
    # Update status back to processing
    db.update_order_status(order_id, ORDER_STATUS['PROCESSING'])
    
    # Log the resume event
    db.log_event("order_resumed", {
        "order_id": order_id,
        "timestamp": "now"
    })
    
    # Broadcast the resume event
    broadcast({
        "event": "order_resumed",
        "order": order_id
    })
    
    return {"msg": "order_resumed", "order": order_id}

@app.post("/orders/{order_id}/complete")
def complete_order(order_id: int = Path(..., title="The ID of the order to complete")):
    """Mark an order as completed (called by Scheduler)."""
    order = db.get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")
    
    # Update status to completed
    db.update_order_status(order_id, ORDER_STATUS['COMPLETED'])
    
    # Log the completion event
    event_id = db.log_event("order_completed", {
        "order_id": order_id,
        "timestamp": "now"
    })
    
    # Broadcast the completion event
    broadcast({
        "event": "order_completed",
        "order": order_id,
        "timestamp": "now"
    })
    
    return {"msg": "order_completed", "order": order_id}

@app.delete("/orders/{order_id}")
def delete_order(order_id: int = Path(..., title="The ID of the order to delete")):
    """Delete an order from the system."""
    order = db.get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")
    
    # Remove from queue if it's still queued
    if order.get("status") == ORDER_STATUS['QUEUED']:
        queue.remove(order_id)
    
    # Delete from database (now allows deletion of all order types including processing)
    success = db.delete_order(order_id)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to delete order from database")
    
    # Log the deletion event
    event_id = db.log_event("order_deleted", {
        "order_id": order_id,
        "previous_status": order.get("status"),
        "timestamp": "now"
    })
    
    # Broadcast the deletion event
    broadcast({
        "event": "order_deleted",
        "order": order_id,
        "previous_status": order.get("status"),
        "timestamp": "now"
    })
    
    return {"msg": "order_deleted", "order": order_id, "previous_status": order.get("status")}

@app.post("/orders/{order_id}/fail")
def fail_order(
    order_id: int = Path(..., title="The ID of the order to fail"),
    reason: str = Query(..., title="Reason for order failure")
):
    """Mark an order as failed (called by Scheduler)."""
    order = db.get_order(order_id)
    if not order:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")
    
    # Update status to error with reason
    db.update_order_status(order_id, ORDER_STATUS['ERROR'], reason)
    
    # Log the failure event
    event_id = db.log_event("order_failed", {
        "order_id": order_id,
        "reason": reason,
        "timestamp": "now"
    })
    
    # Create an alert for the failed order
    alert_id = db.create_alert(event_id, "order_failed", "critical")
    
    # Broadcast the failure event
    broadcast({
        "event": "order_failed",
        "order": order_id,
        "reason": reason,
        "alert_id": alert_id
    })
    
    return {"msg": "order_failed", "order": order_id, "reason": reason, "alert_id": alert_id}

# Task Endpoints

@app.post("/tasks/", response_model=dict)
def create_task(task: TaskCreate):
    """Create a new task for an order."""
    task_id = db.save_task(
        task.order_id, 
        task.item_id, 
        task.arm_id, 
        task.function_name
    )
    return {"task_id": task_id, "status": "queued"}

@app.patch("/tasks/{task_id}/status")
def update_task_status(
    task_id: int = Path(..., title="The ID of the task to update"),
    update: TaskStatusUpdate = None
):
    """Update the status of a task."""
    db.update_task_status(task_id, update.status, update.error_message)
    return {"msg": "task_status_updated", "task_id": task_id, "status": update.status}

@app.post("/tasks/steps/", response_model=dict)
def create_task_step(step: TaskStepCreate):
    """Create a new task step."""
    step_id = db.save_task_step(
        step.task_id,
        step.step_index,
        step.step_type,
        step.function_name,
        step.params,
        step.status
    )
    return {"step_id": step_id, "status": step.status}

@app.patch("/tasks/steps/{step_id}/status")
def update_task_step_status(
    step_id: int = Path(..., title="The ID of the step to update"),
    update: TaskStepStatusUpdate = None
):
    """Update the status of a task step."""
    db.update_task_step_status(step_id, update.status, update.error_message)
    return {"msg": "step_status_updated", "step_id": step_id, "status": update.status}

# Event and Alert Endpoints

@app.post("/events/", response_model=dict)
def log_event(event: EventCreate):
    """Log an event."""
    event_id = db.log_event(event.event_type, event.payload)
    return {"event_id": event_id}

@app.post("/alerts/", response_model=dict)
def create_alert(alert: AlertCreate):
    """Create an alert."""
    alert_id = db.create_alert(alert.event_id, alert.alert_type, alert.severity)
    # Broadcast alert to clients
    broadcast({"event": "alert_created", "alert_id": alert_id, "type": alert.alert_type, "severity": alert.severity})
    return {"alert_id": alert_id}

@app.patch("/alerts/{alert_id}/acknowledge")
def acknowledge_alert(alert_id: int = Path(..., title="The ID of the alert to acknowledge")):
    """Acknowledge an alert."""
    db.acknowledge_alert(alert_id)
    # Broadcast acknowledgment
    broadcast({"event": "alert_acknowledged", "alert_id": alert_id})
    return {"msg": "alert_acknowledged", "alert_id": alert_id}

@app.get("/alerts/active")
def get_active_alerts():
    """Get all active (unacknowledged) alerts."""
    alerts = db.get_active_alerts()
    return {"alerts": alerts}

# System Control Endpoints

@app.post("/system/stop")
def stop_system(request: SystemStopRequest):
    """Emergency stop - halt all system operations."""
    try:
        # Log the stop event
        db.log_event("system_stopped", {"reason": request.reason})
        
        # Create an alert for the system stop
        event_id = db.log_event("emergency_stop", {"reason": request.reason, "timestamp": "now"})
        alert_id = db.create_alert(event_id, "emergency_stop", "critical")
        
        # Broadcast system stop event
        broadcast({
            "event": "system_stopped", 
            "reason": request.reason,
            "alert_id": alert_id,
            "timestamp": "now"
        })
        
        return {
            "status": "success",
            "message": "System stopped successfully",
            "reason": request.reason,
            "alert_id": alert_id
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to stop system: {str(e)}")

@app.post("/system/resume")
def resume_system():
    """Resume system operations after emergency stop."""
    try:
        # Log the resume event
        event_id = db.log_event("system_resumed", {"timestamp": "now"})
        
        # Broadcast system resume event
        broadcast({
            "event": "system_resumed",
            "timestamp": "now"
        })
        
        return {
            "status": "success",
            "message": "System operations resumed successfully"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to resume system: {str(e)}")

@app.get("/system/status")
def get_system_status():
    """Get current system status."""
    try:
        # Get recent system events to determine current status
        recent_events = db.get_recent_events(limit=10)
        
        # Simple logic to determine if system is stopped or running
        system_status = "running"  # Default
        for event in recent_events:
            if event.get("event_type") == "system_stopped":
                system_status = "stopped"
                break
            elif event.get("event_type") == "system_resumed":
                system_status = "running"
                break
        
        return {
            "status": system_status,
            "timestamp": "now"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get system status: {str(e)}")

@app.get("/queue/sync")
def sync_queue():
    """Manually sync the Redis queue with database orders."""
    try:
        success = queue.sync_with_database()
        if success:
            # Get current queue status
            current_queue = queue.get_queue()
            return {
                "status": "success",
                "message": "Queue synced with database",
                "queue_length": len(current_queue),
                "queue": current_queue
            }
        else:
            return {"status": "error", "message": "Failed to sync queue with database"}
    except Exception as e:
        return {"status": "error", "message": f"Sync error: {str(e)}"}

# Inventory Management Endpoints

@app.post("/inventory/threshold-warning")
def receive_threshold_warning(warning: ThresholdWarning):
    """Receive threshold warning from Validation Service."""
    try:
        # Map severity to alert severity
        alert_severity_map = {
            "low": "warning",
            "medium": "warning", 
            "high": "critical"
        }
        
        alert_severity = alert_severity_map.get(warning.severity.lower(), "warning")
        
        # Log the threshold warning event
        event_id = db.log_event("threshold_warning", {
            "ingredient": warning.ingredient,
            "severity": warning.severity.lower(),
            "timestamp": "now"
        })
        
        # Create an alert for the threshold warning
        alert_id = db.create_alert(event_id, "ingredient_threshold", alert_severity)
        
        # Broadcast the threshold warning to dashboard
        broadcast({
            "event": "threshold_warning",
            "ingredient": warning.ingredient,
            "severity": warning.severity.lower(),
            "alert_id": alert_id,
            "timestamp": "now"
        })
        
        return {
            "status": "success",
            "message": f"Threshold warning received for {warning.ingredient}",
            "ingredient": warning.ingredient,
            "severity": warning.severity.lower(),
            "alert_id": alert_id
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to process threshold warning: {str(e)}")

@app.post("/inventory/refill")
def refill_inventory(refill: InventoryRefill):
    """Manually refill a specific ingredient inventory."""
    try:
        # Validate ingredient
        valid_ingredients = ["milk", "cup", "beans", "syrup"]
        if refill.ingredient.lower() not in valid_ingredients:
            raise HTTPException(
                status_code=400, 
                detail=f"Invalid ingredient. Must be one of: {valid_ingredients}"
            )
        
        # Log the refill event
        event_id = db.log_event("inventory_refilled", {
            "ingredient": refill.ingredient.lower(),
            "timestamp": "now"
        })
        
        # Send refill request to Validation Service
        validation_url = "http://localhost:8003/inventory/refill"
        
        try:
            import httpx
            import asyncio
            
            async def send_refill_to_validation():
                async with httpx.AsyncClient() as client:
                    response = await client.post(validation_url, json={"ingredient": refill.ingredient.lower()})
                    response.raise_for_status()
                    return response.json()
            
            # Run the async function
            try:
                loop = asyncio.get_running_loop()
                # Create a task to run the async function
                task = loop.create_task(send_refill_to_validation())
                # Note: In a real implementation, you might want to await this properly
                # For now, we'll continue without waiting for the response
            except RuntimeError:
                # No running loop, create a new one
                validation_response = asyncio.run(send_refill_to_validation())
                print(f"✅ Validation Service response: {validation_response}")
            
        except Exception as validation_error:
            print(f"⚠️ Warning: Could not reach Validation Service: {validation_error}")
            # Continue anyway - the refill might be manual
        
        # Broadcast the refill event to dashboard
        broadcast({
            "event": "inventory_refilled",
            "ingredient": refill.ingredient.lower(),
            "timestamp": "now"
        })
        
        return {
            "status": "success",
            "message": f"Inventory refill initiated for {refill.ingredient}",
            "ingredient": refill.ingredient.lower()
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to process inventory refill: {str(e)}")

@app.get("/inventory/status")
def get_inventory_status():
    """Get current inventory status (placeholder - would typically query Validation Service)."""
    try:
        # This would typically query the Validation Service for current inventory levels
        # For now, return a placeholder response
        return {
            "status": "success",
            "inventory": {
                "milk": {"level": "medium", "last_refilled": "2024-01-15T10:30:00Z"},
                "cup": {"level": "high", "last_refilled": "2024-01-15T08:00:00Z"},
                "beans": {"level": "low", "last_refilled": "2024-01-14T16:45:00Z"},
                "syrup": {"level": "medium", "last_refilled": "2024-01-15T09:15:00Z"}
            },
            "timestamp": "now"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get inventory status: {str(e)}")

async def send_to_scheduler(order_data: dict):
    """Call Scheduler service to initiate processing of the order."""
    
    scheduler_url = "http://scheduler:8000/process"  # Corrected service name
    
    # Create a clean payload with only the data scheduler needs
    # Convert database format (drink_type) to scheduler format (type)
    cups = []
    for cup in order_data.get("cups", []):
        cups.append({
            "type": cup.get("drink_type"),  # Map drink_type to type
            "size": cup.get("cup_size"),    # Map cup_size to size
            "addons": cup.get("addons", [])
        })
    
    scheduler_payload = {
        "id": order_data.get("id"),
        "cups": cups
    }
    
    try:
        async with httpx.AsyncClient() as client:
            await client.post(scheduler_url, json=scheduler_payload)
    except Exception as e:
        # Handle error (e.g., log, retry, or publish an "order.failed" event)
        print(f"Error sending order to scheduler: {e}")
        # Update order status to error
        db.update_order_status(order_data["id"], "error")
        broadcast({"event": "order_failed", "order": order_data["id"], "error": str(e)})

# WebSocket endpoint for real-time order updates
@app.websocket("/ws/orders")
async def orders_ws(websocket: WebSocket):
    await websocket.accept()
    active_connections.append(websocket)
    try:
        while True:
            await websocket.receive_text()  # keep connection alive (no specific incoming messages)
    except WebSocketDisconnect:
        # Safe removal - only remove if still in list
        if websocket in active_connections:
            active_connections.remove(websocket)

# WebSocket endpoint for real-time alert updates
@app.websocket("/ws/alerts")
async def alerts_ws(websocket: WebSocket):
    await websocket.accept()
    alert_connections.append(websocket)
    try:
        while True:
            await websocket.receive_text()  # keep connection alive
    except WebSocketDisconnect:
        # Safe removal - only remove if still in list
        if websocket in alert_connections:
            alert_connections.remove(websocket)

def broadcast(message: dict):
    """Send a message to all connected WebSocket clients."""
    import json
    import asyncio
    
    # Create the message string once
    message_str = json.dumps(message)
    
    async def send_to_connections():
        # Broadcast to order connections
        disconnected_orders = []
        for ws in active_connections:
            try:
                await ws.send_text(message_str)
            except Exception as e:
                print(f"WebSocket send error: {e}")
                disconnected_orders.append(ws)
        
        # Remove disconnected WebSockets
        for ws in disconnected_orders:
            if ws in active_connections:
                active_connections.remove(ws)
        
        # Broadcast alerts to alert connections (if it's an alert event)
        if message.get("event") in ["alert_created", "alert_acknowledged"]:
            disconnected_alerts = []
            for ws in alert_connections:
                try:
                    await ws.send_text(message_str)
                except Exception as e:
                    print(f"Alert WebSocket send error: {e}")
                    disconnected_alerts.append(ws)
            
            # Remove disconnected alert WebSockets
            for ws in disconnected_alerts:
                if ws in alert_connections:
                    alert_connections.remove(ws)
    
    # Schedule the coroutine to run in the event loop
    try:
        loop = asyncio.get_running_loop()
        # Create a task to run the async function
        loop.create_task(send_to_connections())
        print(f"WebSocket broadcast scheduled: {message.get('event', 'unknown')}")
    except RuntimeError:
        # No running loop, create a new one
        asyncio.run(send_to_connections())
        print(f"WebSocket broadcast sent: {message.get('event', 'unknown')}")
