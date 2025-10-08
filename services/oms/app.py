# services/oms/app.py
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, BackgroundTasks, HTTPException, Path, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from contextlib import asynccontextmanager
from . import db, queue, models  # hypothetical internal modules
import httpx
import asyncio
import json
import logging
import os
import sys
from dataclasses import asdict

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.rabbitmq_client import RabbitMQClient, EventListener
from .pos_core import parse_transaction, load_reference_data_from_db

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global RabbitMQ clients
rabbitmq_client: Optional[RabbitMQClient] = None
event_listener: Optional[EventListener] = None

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
    severity: str = Field(..., pattern="^(low|medium|high)$")

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
    global rabbitmq_client, event_listener
    
    db.connect()        # Connect to PostgreSQL
    queue.connect()     # Connect to Redis
    
    # Initialize POS reference data
    try:
        pos_db_path = os.environ.get("POS_DB_PATH", "pos_reference.db")
        success = load_reference_data_from_db(pos_db_path)
        if not success:
            logger.warning("Could not load POS reference data. Running with empty references.")
        else:
            logger.info("POS reference data loaded successfully")
    except Exception as e:
        logger.error(f"Failed to load POS reference data: {e}")
    
    # Mark any processing orders as failed due to container restart
    print("Marking processing orders as failed due to container restart...")
    try:
        failed_count = db.mark_processing_orders_as_failed()
        if failed_count > 0:
            print(f"✅ Marked {failed_count} processing orders as failed due to container restart")
            # Broadcast the failure events to any connected clients
            for i in range(failed_count):
                broadcast({
                    "event": "container_restart_order_cleanup",
                    "message": f"Marked {failed_count} orders as failed due to container restart",
                    "timestamp": "now"
                })
        else:
            print("✅ No processing orders found to mark as failed")
    except Exception as e:
        print(f"⚠️ Error marking processing orders as failed: {e}")
    
    # Sync queue with database on startup
    print("Syncing queue with database on startup...")
    queue.sync_with_database()
    
    # Initialize RabbitMQ clients
    try:
        rabbitmq_client = RabbitMQClient("oms")
        await rabbitmq_client.connect()
        
        event_listener = EventListener("oms")
        await event_listener.connect()
        
        # Register RabbitMQ message handlers
        register_rabbitmq_handlers()
        
        # Subscribe to relevant events
        await event_listener.subscribe_to_events([
            "scheduler.*", "validation.*", "automation.*", "routine.*", "system.*"
        ])
        register_event_handlers()
        
        logger.info("OMS service started with RabbitMQ integration")
        
    except Exception as e:
        logger.error(f"Failed to initialize RabbitMQ: {e}")
        # Continue without RabbitMQ if it fails
    
    yield
    
    # Shutdown
    if rabbitmq_client:
        await rabbitmq_client.disconnect()
    if event_listener:
        await event_listener.disconnect()

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

def register_rabbitmq_handlers():
    """Register all RabbitMQ message handlers"""
    if not rabbitmq_client:
        return
        
    rabbitmq_client.register_handler("create_order", handle_create_order_mq)
    rabbitmq_client.register_handler("list_orders", handle_list_orders_mq)
    rabbitmq_client.register_handler("get_order", handle_get_order_mq)
    rabbitmq_client.register_handler("start_order", handle_start_order_mq)
    rabbitmq_client.register_handler("update_order_status", handle_update_order_status_mq)
    rabbitmq_client.register_handler("delete_order", handle_delete_order_mq)
    rabbitmq_client.register_handler("halt_order", handle_halt_order_mq)
    rabbitmq_client.register_handler("resume_order", handle_resume_order_mq)
    rabbitmq_client.register_handler("bulk_reorder_queue", handle_bulk_reorder_queue_mq)
    rabbitmq_client.register_handler("sync_queue", handle_sync_queue_mq)
    rabbitmq_client.register_handler("emergency_stop", handle_emergency_stop_mq)
    rabbitmq_client.register_handler("resume_operations", handle_resume_operations_mq)
    rabbitmq_client.register_handler("get_active_alerts", handle_get_active_alerts_mq)
    rabbitmq_client.register_handler("get_acknowledged_alerts", handle_get_acknowledged_alerts_mq)
    rabbitmq_client.register_handler("acknowledge_alert", handle_acknowledge_alert_mq)
    rabbitmq_client.register_handler("mark_processing_orders_failed", handle_mark_processing_orders_failed_mq)
    rabbitmq_client.register_handler("health", handle_health_mq)
    
    logger.info("Registered all RabbitMQ message handlers")

def register_event_handlers():
    """Register event handlers"""
    if not event_listener:
        return
        
    event_listener.register_event_handler("scheduler.order_completed", handle_order_completed_event)
    event_listener.register_event_handler("scheduler.order_failed", handle_order_failed_event)
    event_listener.register_event_handler("scheduler.order_heartbeat", handle_order_heartbeat_event)
    event_listener.register_event_handler("validation.threshold_warning", handle_threshold_warning_event)
    event_listener.register_event_handler("system.shutdown", handle_shutdown_event)
    
    logger.info("Registered all event handlers")

# RabbitMQ Message Handlers
async def handle_create_order_mq(data: Dict) -> Dict:
    logger.info(data, type(data))
    """Handle create order requests via RabbitMQ"""
    try:
        order_data = data.get("order", {})
        order = models.Order(**order_data)  # Validate with Pydantic model
        
        order_id = db.save_order(order)
        queue.add_order(order_id)
        
        # Send event
        if rabbitmq_client:
            await rabbitmq_client.send_event("oms.order_created", {
                "order_id": order_id,
                "timestamp": "now"
            })
        
        broadcast({"event": "order_received", "order": order_id, "status": "queued"})
        
        return {"success": True, "order_id": order_id, "status": "queued"}
        
    except Exception as e:
        logger.error(f"Error creating order via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_list_orders_mq(data: Dict) -> Dict:
    """Handle list orders requests via RabbitMQ"""
    try:
        status = data.get("status")
        orders = db.get_orders(status=status)
        return {"success": True, "orders": orders}
        
    except Exception as e:
        logger.error(f"Error listing orders via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_get_order_mq(data: Dict) -> Dict:
    """Handle get order requests via RabbitMQ"""
    try:
        order_id = data.get("order_id")
        if not order_id:
            return {"success": False, "error": "Missing order_id"}
            
        order = db.get_order(order_id)
        if not order:
            return {"success": False, "error": f"Order {order_id} not found"}
            
        return {"success": True, "order": order}
        
    except Exception as e:
        logger.error(f"Error getting order via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_start_order_mq(data: Dict) -> Dict:
    """Handle start order requests via RabbitMQ"""
    try:
        order_id = data.get("order_id")
        if not order_id:
            return {"success": False, "error": "Missing order_id"}
        
        logger.info(f"🚀 OMS received start_order request for order {order_id}")
        
        # Concurrency guard: allow only one processing order at a time
        try:
            processing = db.get_orders(status=ORDER_STATUS['PROCESSING'])
        except Exception as e:
            processing = []
            logger.error(f"Error checking processing orders: {e}")
        if processing and any(o.get('status') == ORDER_STATUS['PROCESSING'] for o in processing):
            logger.warning(f"🔒 OMS rejecting start_order for {order_id}: another order is already processing")
            return {"success": False, "error": "Another order is currently processing. Please wait."}
        
        order = db.get_order(order_id)
        if not order:
            return {"success": False, "error": f"Order {order_id} not found"}
        
        # Update order status to processing
        db.update_order_status(order_id, "processing")
        queue.remove(order_id)
        
        # Send event
        if rabbitmq_client:
            await rabbitmq_client.send_event("oms.order_started", {
                "order_id": order_id,
                "timestamp": "now"
            })
        
        # Broadcast to WebSocket clients
        broadcast({"event": "order_started", "order": order_id})
        
        # Send order to Scheduler for processing (async to avoid blocking response)
        asyncio.create_task(send_to_scheduler(order))
        
        logger.info(f"✅ OMS successfully started processing order {order_id}")
        return {"success": True, "message": "order_sent_to_scheduler", "order_id": order_id}
        
    except Exception as e:
        logger.error(f"💥 Error starting order via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_update_order_status_mq(data: Dict) -> Dict:
    """Handle update order status requests via RabbitMQ"""
    try:
        order_id = data.get("order_id")
        status = data.get("status")
        reason = data.get("reason")
        
        if not order_id or not status:
            return {"success": False, "error": "Missing order_id or status"}
            
        order = db.get_order(order_id)
        if not order:
            return {"success": False, "error": f"Order {order_id} not found"}
        
        # Validate status
        valid_statuses = list(ORDER_STATUS.values())
        if status not in valid_statuses:
            return {"success": False, "error": f"Invalid status. Must be one of: {valid_statuses}"}
        
        # Update order status
        db.update_order_status(order_id, status, reason)
        
        # Log event
        db.log_event("order_status_changed", {
            "order_id": order_id,
            "old_status": order.get("status"),
            "new_status": status,
            "reason": reason,
            "timestamp": "now"
        })
        
        # Broadcast update
        broadcast({
            "event": "order_status_updated", 
            "order": order_id, 
            "status": status,
            "reason": reason
        })
        
        return {"success": True, "message": "status_updated", "order_id": order_id, "status": status}
        
    except Exception as e:
        logger.error(f"Error updating order status via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_delete_order_mq(data: Dict) -> Dict:
    """Handle delete order requests via RabbitMQ"""
    try:
        order_id = data.get("order_id")
        if not order_id:
            return {"success": False, "error": "Missing order_id"}
            
        order = db.get_order(order_id)
        if not order:
            return {"success": False, "error": f"Order {order_id} not found"}
        
        # If the order is in progress, request cancellation in scheduler and routine first
        if order.get("status") == ORDER_STATUS['PROCESSING']:
            try:
                if rabbitmq_client:
                    # Ask scheduler to cancel the order (which also instructs routine)
                    cancel_resp = await rabbitmq_client.send_request(
                        target_service="scheduler",
                        action="cancel_order",
                        data={"order_id": order_id},
                        timeout=15
                    )
                    logger.info(f"OMS cancel request response (scheduler): {cancel_resp}")
            except Exception as e:
                logger.error(f"OMS failed to request scheduler cancel for order {order_id}: {e}")
        
        # Remove from queue if still queued
        if order.get("status") == ORDER_STATUS['QUEUED']:
            queue.remove(order_id)
        
        # Delete from database
        success = db.delete_order(order_id)
        if not success:
            return {"success": False, "error": "Failed to delete order from database"}
        
        # Log deletion event
        event_id = db.log_event("order_deleted", {
            "order_id": order_id,
            "previous_status": order.get("status"),
            "timestamp": "now"
        })
        
        # Broadcast deletion
        broadcast({
            "event": "order_deleted",
            "order": order_id,
            "previous_status": order.get("status"),
            "timestamp": "now"
        })
        
        return {"success": True, "message": "order_deleted", "order_id": order_id}
        
    except Exception as e:
        logger.error(f"Error deleting order via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_halt_order_mq(data: Dict) -> Dict:
    """Handle halt order requests via RabbitMQ"""
    try:
        order_id = data.get("order_id")
        reason = data.get("reason", "Manual halt")
        
        if not order_id:
            return {"success": False, "error": "Missing order_id"}
            
        order = db.get_order(order_id)
        if not order:
            return {"success": False, "error": f"Order {order_id} not found"}
        
        # Update status to halted
        db.update_order_status(order_id, ORDER_STATUS['HALTED'], reason)
        
        # Create alert
        event_id = db.log_event("order_halted", {
            "order_id": order_id,
            "reason": reason,
            "timestamp": "now"
        })
        alert_id = db.create_alert(event_id, "order_halted", "warning")
        
        # Broadcast halt event
        broadcast({
            "event": "order_halted",
            "order": order_id,
            "reason": reason,
            "alert_id": alert_id
        })
        
        return {"success": True, "message": "order_halted", "order_id": order_id, "alert_id": alert_id}
        
    except Exception as e:
        logger.error(f"Error halting order via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_resume_order_mq(data: Dict) -> Dict:
    """Handle resume order requests via RabbitMQ"""
    try:
        order_id = data.get("order_id")
        if not order_id:
            return {"success": False, "error": "Missing order_id"}
            
        order = db.get_order(order_id)
        if not order:
            return {"success": False, "error": f"Order {order_id} not found"}
        
        if order.get("status") != ORDER_STATUS['HALTED']:
            return {"success": False, "error": "Order is not in halted state"}
        
        # Update status back to processing
        db.update_order_status(order_id, ORDER_STATUS['PROCESSING'])
        
        # Log resume event
        db.log_event("order_resumed", {
            "order_id": order_id,
            "timestamp": "now"
        })
        
        # Broadcast resume event
        broadcast({
            "event": "order_resumed",
            "order": order_id
        })
        
        return {"success": True, "message": "order_resumed", "order_id": order_id}
        
    except Exception as e:
        logger.error(f"Error resuming order via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_bulk_reorder_queue_mq(data: Dict) -> Dict:
    """Handle bulk reorder queue requests via RabbitMQ"""
    try:
        order_ids = data.get("order_ids", [])
        if not order_ids:
            return {"success": False, "error": "order_ids list is required"}
        
        # Update the queue order in Redis
        queue.bulk_reorder(order_ids)
        
        # Broadcast the updated queue order to clients
        broadcast({"event": "queue_bulk_reordered", "order_ids": order_ids})
        
        return {"success": True, "message": "queue_reordered", "order_count": len(order_ids)}
        
    except Exception as e:
        logger.error(f"Error bulk reordering queue via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_sync_queue_mq(data: Dict) -> Dict:
    """Handle sync queue requests via RabbitMQ"""
    try:
        success = queue.sync_with_database()
        if success:
            current_queue = queue.get_queue()
            return {
                "success": True,
                "message": "Queue synced with database",
                "queue_length": len(current_queue),
                "queue": current_queue
            }
        else:
            return {"success": False, "message": "Failed to sync queue with database"}
            
    except Exception as e:
        logger.error(f"Error syncing queue via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_emergency_stop_mq(data: Dict) -> Dict:
    """Handle emergency stop requests via RabbitMQ"""
    try:
        reason = data.get("reason", "Emergency stop requested")
        
        # Log the stop event
        db.log_event("system_stopped", {"reason": reason})
        
        # Create an alert for the system stop
        event_id = db.log_event("emergency_stop", {"reason": reason, "timestamp": "now"})
        alert_id = db.create_alert(event_id, "emergency_stop", "critical")
        
        # Broadcast system stop event
        broadcast({
            "event": "system_stopped", 
            "reason": reason,
            "alert_id": alert_id,
            "timestamp": "now"
        })
        
        return {
            "success": True,
            "message": "System stopped successfully",
            "reason": reason,
            "alert_id": alert_id
        }
        
    except Exception as e:
        logger.error(f"Error stopping system via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_resume_operations_mq(data: Dict) -> Dict:
    """Handle resume operations requests via RabbitMQ"""
    try:
        # Log the resume event
        event_id = db.log_event("system_resumed", {"timestamp": "now"})
        
        # Broadcast system resume event
        broadcast({
            "event": "system_resumed",
            "timestamp": "now"
        })
        
        return {
            "success": True,
            "message": "System operations resumed successfully"
        }
        
    except Exception as e:
        logger.error(f"Error resuming system via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_get_active_alerts_mq(data: Dict) -> Dict:
    """Handle get active alerts requests via RabbitMQ"""
    try:
        alerts = db.get_active_alerts()
        return {"success": True, "alerts": alerts}
        
    except Exception as e:
        logger.error(f"Error getting active alerts via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_get_acknowledged_alerts_mq(data: Dict) -> Dict:
    """Handle get acknowledged alerts requests via RabbitMQ"""
    try:
        alerts = db.get_acknowledged_alerts()
        return {"success": True, "alerts": alerts}
        
    except Exception as e:
        logger.error(f"Error getting acknowledged alerts via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_acknowledge_alert_mq(data: Dict) -> Dict:
    """Handle acknowledge alert requests via RabbitMQ"""
    try:
        alert_id = data.get("alert_id")
        if not alert_id:
            return {"success": False, "error": "Missing alert_id"}
            
        db.acknowledge_alert(alert_id)
        
        # Broadcast acknowledgment
        broadcast({"event": "alert_acknowledged", "alert_id": alert_id})
        
        return {"success": True, "message": "alert_acknowledged", "alert_id": alert_id}
        
    except Exception as e:
        logger.error(f"Error acknowledging alert via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_mark_processing_orders_failed_mq(data: Dict) -> Dict:
    """Handle mark processing orders as failed requests via RabbitMQ"""
    try:
        reason = data.get("reason", "Processing interrupted via RabbitMQ request")
        
        failed_count = db.mark_processing_orders_as_failed(reason)
        
        if failed_count > 0:
            # Broadcast the failure events to any connected clients
            broadcast({
                "event": "processing_orders_marked_failed",
                "count": failed_count,
                "reason": reason,
                "timestamp": "now"
            })
            
            return {
                "success": True,
                "message": f"Marked {failed_count} processing orders as failed",
                "failed_count": failed_count,
                "reason": reason
            }
        else:
            return {
                "success": True,
                "message": "No processing orders found to mark as failed",
                "failed_count": 0,
                "reason": reason
            }
        
    except Exception as e:
        logger.error(f"Error marking processing orders as failed via MQ: {e}")
        return {"success": False, "error": str(e)}

async def handle_health_mq(data: Dict) -> Dict:
    """Handle health check requests via RabbitMQ"""
    try:
        return {
            "status": "healthy",
            "service": "oms",
            "timestamp": "now",
            "queue_length": len(queue.get_queue()) if queue else 0,
            "rabbitmq_connected": rabbitmq_client is not None
        }
    except Exception as e:
        logger.error(f"Error in health check via MQ: {e}")
        return {"success": False, "error": str(e)}

# Event Handlers
async def handle_order_completed_event(data: Dict):
    """Handle order completion events from scheduler"""
    order_id = data.get("order_id")
    logger.info(f"🎉 [OMS] Received order_completed event from scheduler for order {order_id}")
    
    if order_id:
        # Check if order is already completed to prevent duplicate processing
        order = db.get_order(order_id)
        if order and order.get("status") == ORDER_STATUS['COMPLETED']:
            logger.warning(f"⚠️ [OMS] Order {order_id} is already COMPLETED. Ignoring duplicate completion event.")
            return
        
        logger.info(f"✅ [OMS] Updating order {order_id} status to COMPLETED in database")
        db.update_order_status(order_id, ORDER_STATUS['COMPLETED'])
        
        logger.info(f"📡 [OMS] Broadcasting order_completed event to dashboard for order {order_id}")
        broadcast({
            "event": "order_completed",
            "order": order_id,
            "timestamp": "now"
        })
        logger.info(f"✅ [OMS] Successfully processed order completion for order {order_id}")
    else:
        logger.error(f"❌ [OMS] Received order_completed event but no order_id provided: {data}")

async def handle_order_failed_event(data: Dict):
    """Handle order failure events from scheduler"""
    order_id = data.get("order_id")
    error = data.get("error", "Unknown error")
    logger.info(f"❌ [OMS] Received order_failed event from scheduler for order {order_id}, error: {error}")
    
    if order_id:
        # Check if order is already in error state to prevent duplicate processing
        order = db.get_order(order_id)
        if order and order.get("status") == ORDER_STATUS['ERROR']:
            logger.warning(f"⚠️ [OMS] Order {order_id} is already in ERROR state. Ignoring duplicate failure event.")
            return
        
        logger.info(f"❌ [OMS] Updating order {order_id} status to ERROR in database")
        db.update_order_status(order_id, ORDER_STATUS['ERROR'], error)
        
        logger.info(f"📡 [OMS] Broadcasting order_failed event to dashboard for order {order_id}")
        broadcast({
            "event": "order_failed",
            "order": order_id,
            "error": error,
            "timestamp": "now"
        })
        logger.info(f"❌ [OMS] Successfully processed order failure for order {order_id}")
    else:
        logger.error(f"❌ [OMS] Received order_failed event but no order_id provided: {data}")

async def handle_order_heartbeat_event(data: Dict):
    """Handle order heartbeat events from scheduler"""
    order_id = data.get("order_id")
    status = data.get("status")
    logger.info(f"💓 [OMS] Received order_heartbeat event from scheduler for order {order_id}, status: {status}")
    
    if order_id:
        order = db.get_order(order_id)
        if order:
            # Update order status if it's not already completed or failed
            if order.get("status") not in [ORDER_STATUS['COMPLETED'], ORDER_STATUS['ERROR']]:
                db.update_order_status(order_id, status)
                logger.info(f"✅ [OMS] Updated order {order_id} status to {status} in database")
                broadcast({
                    "event": "order_heartbeat",
                    "order": order_id,
                    "status": status,
                    "timestamp": "now"
                })
            else:
                logger.warning(f"⚠️ [OMS] Order {order_id} is already in final state ({order['status']}). Ignoring heartbeat.")
        else:
            logger.warning(f"⚠️ [OMS] Order {order_id} not found in database. Ignoring heartbeat.")
    else:
        logger.error(f"❌ [OMS] Received order_heartbeat event but no order_id provided: {data}")

async def handle_threshold_warning_event(data: Dict):
    """Handle threshold warning events from validation service"""
    ingredient = data.get("ingredient")
    severity = data.get("severity")
    if ingredient and severity:
        # Create alert for threshold warning
        event_id = db.log_event("threshold_warning", {
            "ingredient": ingredient,
            "severity": severity,
            "timestamp": "now"
        })
        alert_id = db.create_alert(event_id, "ingredient_threshold", severity)
        
        broadcast({
            "event": "threshold_warning",
            "ingredient": ingredient,
            "severity": severity,
            "alert_id": alert_id,
            "timestamp": "now"
        })

async def handle_shutdown_event(data: Dict):
    """Handle system shutdown events"""
    logger.info("Received shutdown event, stopping OMS service...")

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
    # Concurrency guard: allow only one processing order at a time
    try:
        processing = db.get_orders(status=ORDER_STATUS['PROCESSING'])
    except Exception as e:
        processing = []
        logger.error(f"Error checking processing orders: {e}")
    if processing and any(o.get('status') == ORDER_STATUS['PROCESSING'] for o in processing):
        raise HTTPException(status_code=409, detail="Another order is currently processing. Please wait.")

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
    
    # If processing, best-effort cancel in scheduler/routine first (fire-and-forget)
    if order.get("status") == ORDER_STATUS['PROCESSING']:
        try:
            if rabbitmq_client:
                asyncio.create_task(rabbitmq_client.send_request(
                    target_service="scheduler",
                    action="cancel_order",
                    data={"order_id": order_id},
                    timeout=10
                ))
        except Exception as e:
            logger.error(f"Error requesting cancel before delete for order {order_id}: {e}")
    
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
    update: TaskStatusUpdate,
    task_id: int = Path(..., title="The ID of the task to update")
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
    update: TaskStepStatusUpdate,
    step_id: int = Path(..., title="The ID of the step to update")
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

@app.get("/alerts/acknowledged")
def get_acknowledged_alerts():
    """Get all acknowledged alerts."""
    alerts = db.get_acknowledged_alerts()
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
        
        # Get RabbitMQ client health status
        rabbitmq_health = {}
        if rabbitmq_client:
            rabbitmq_health = rabbitmq_client.get_health_status()
        
        return {
            "status": system_status,
            "timestamp": "now",
            "rabbitmq_health": rabbitmq_health,
            "event_listener_connected": event_listener is not None
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get system status: {str(e)}")

@app.get("/system/rabbitmq-health")
def get_rabbitmq_health():
    """Get detailed RabbitMQ client health status."""
    try:
        if not rabbitmq_client:
            return {
                "status": "error",
                "message": "RabbitMQ client not initialized",
                "timestamp": "now"
            }
        
        health_status = rabbitmq_client.get_health_status()
        return {
            "status": "success",
            "health": health_status,
            "timestamp": "now"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get RabbitMQ health: {str(e)}")

# POS Integration Endpoint
@app.post("/pos/process-order")
async def process_pos_order(order_data: dict):
    """Process POS order and return parsed transaction as JSON, printing the dataclass object."""
    try:
        # Validate required fields
        required_fields = [
            "transaction_id", "date", "time", "store_number", "pos_reg_id", "items"
        ]
        for field in required_fields:
            if field not in order_data:
                raise HTTPException(status_code=400, detail=f"Missing required field: {field}")
        if not isinstance(order_data.get("items"), list) or len(order_data["items"]) == 0:
            raise HTTPException(status_code=400, detail="Items must be a non-empty list")

        # Process the order via core
        parsed_order = parse_transaction(order_data)

        # Build cups honoring item quantity (ordered_qty)
        order = {"order": {"cups": []}}

        for item in parsed_order.get("items", []):
            # Group this item's ingredients by category -> ingredient_id -> amount
            grouped = {}
            for ing in item.ingredients:
                cat = getattr(ing, "category", None)
                ingredient_id = getattr(ing, "ingredient_id", None)
                amount = getattr(ing, "total_amount", 0)
                if cat not in grouped:
                    grouped[cat] = {}
                grouped[cat][ingredient_id] = amount

            # Determine size from grouped cups (if present)
            size = None
            if "cups" in grouped and len(grouped["cups"]) > 0:
                size = next(iter(grouped["cups"].keys()))

            # Number of identical cups to create for this item
            quantity = int(getattr(item, "ordered_qty", 1) or 1)

            # Append one cup entry per quantity
            for _ in range(max(1, quantity)):
                # Shallow copy per append to avoid shared references
                ingredients_copy = {k: dict(v) for k, v in grouped.items()}
                cup_entry = {
                    "type": item.recipe_id,
                    "size": size,
                    "addons": [],
                    "ingredients": ingredients_copy,
                }
                order["order"]["cups"].append(cup_entry)
        print(order, type(order))

        result = await handle_create_order_mq(order)
        # Result of results for 2 drinks (sample) = [{'espresso': {'regular': 1.0}, 'cups': {'H7': 1.0}, 'milk': {'almond': 70.0}}, {'espresso': {'regular': 2.0}, 'milk': {'almond': 260.0}, 'cups': {'H12': 1.0}}]
        return result

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing order: {str(e)}")

@app.get("/pos/menu-items")
async def get_menu_items():
    """Get all menu items with their default ingredients from the POS database."""
    try:
        from .pos_core import MENU_ITEMS, load_reference_data_from_db
        
        # Ensure reference data is loaded
        if not MENU_ITEMS:
            load_reference_data_from_db()
        
        # Convert to list format for frontend
        menu_items_list = []
        for item_id, item_data in MENU_ITEMS.items():
            menu_items_list.append({
                "item_id": item_id,
                "name": item_data["name"],
                "category": item_data["category"],
                "size": item_data["size"],
                "automation": item_data["automation"],
                "recipe": item_data["recipe"],
                "default_ingredients": item_data["default_ingredients"]
            })
        
        return {
            "success": True,
            "menu_items": menu_items_list
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching menu items: {str(e)}")

@app.get("/pos/ingredients")
async def get_ingredients():
    """Get all ingredients grouped by category from the POS database."""
    try:
        from .pos_core import INGREDIENT_DETAILS, INGREDIENTS, load_reference_data_from_db
        
        # Ensure reference data is loaded
        if not INGREDIENT_DETAILS:
            load_reference_data_from_db()
        
        # Group ingredients by category
        ingredients_by_category = {}
        for ingredient_id, details in INGREDIENT_DETAILS.items():
            category = details["category"]
            if category not in ingredients_by_category:
                ingredients_by_category[category] = []
            
            ingredients_by_category[category].append({
                "ingredient_id": ingredient_id,
                "name": INGREDIENTS.get(ingredient_id, ingredient_id),
                "type": details["type"],
                "category": category,
                "base_units": details["base_units"],
                "automated": details["automated"],
                "default_amount": details["default_amount"],
                "is_topping": details["is_topping"]
            })
        
        return {
            "success": True,
            "ingredients_by_category": ingredients_by_category
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching ingredients: {str(e)}")

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

@app.post("/orders/mark-processing-failed")
def mark_processing_orders_failed(reason: str = Query("Manual cleanup of processing orders", title="Reason for marking orders as failed")):
    """Manually mark all processing orders as failed."""
    try:
        failed_count = db.mark_processing_orders_as_failed(reason)
        
        if failed_count > 0:
            # Broadcast the failure events to any connected clients
            broadcast({
                "event": "processing_orders_marked_failed",
                "count": failed_count,
                "reason": reason,
                "timestamp": "now"
            })
            
            return {
                "status": "success",
                "message": f"Marked {failed_count} processing orders as failed",
                "failed_count": failed_count,
                "reason": reason
            }
        else:
            return {
                "status": "success", 
                "message": "No processing orders found to mark as failed",
                "failed_count": 0,
                "reason": reason
            }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to mark processing orders as failed: {str(e)}")

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
    """Send order to Scheduler service via RabbitMQ."""
    
    logger.info(f"🔍 DEBUG: send_to_scheduler called with order_data: {order_data}")
    
    # Create a clean payload with only the data scheduler needs
    # Convert database format (drink_type) to scheduler format (type)
    cups = []
    for cup in order_data.get("cups", []):
        cups.append({
            "type": cup.get("drink_type"),  # Map drink_type to type
            "size": cup.get("cup_size"),    # Map cup_size to size
            "addons": cup.get("addons", []),
            "ingredients": cup.get("ingredients", {})
        })
    
    scheduler_payload = {
        "id": order_data.get("id"),
        "cups": cups
    }
    
    logger.info(f"🔍 DEBUG: scheduler_payload: {scheduler_payload}")
    
    try:
        if rabbitmq_client:
            logger.info(f"📤 OMS sending order {order_data.get('id')} to scheduler via RabbitMQ")
            logger.info(f"🔍 DEBUG: RabbitMQ client available, sending request...")
            
            response = await rabbitmq_client.send_request(
                target_service="scheduler",
                action="process_order",
                data=scheduler_payload,
                timeout=180
            )
            
            logger.info(f"🔍 DEBUG: Scheduler response: {response}")
            
            if response.get("success"):
                logger.info(f"✅ Order {order_data.get('id')} successfully sent to scheduler")
            else:
                error_msg = f"Scheduler rejected order: {response.get('error', 'Unknown error')}"
                logger.error(f"❌ {error_msg}")
                raise Exception(error_msg)
        else:
            error_msg = "RabbitMQ client not available"
            logger.error(f"❌ {error_msg}")
            raise Exception(error_msg)
              
    except Exception as e:
        # Handle error (e.g., log, retry, or publish an "order.failed" event)
        logger.error(f"💥 Error sending order to scheduler: {e}")
        logger.error(f"🔍 DEBUG: Exception type: {type(e)}, args: {e.args}")
        # Update order status to error
        try:
            db.update_order_status(order_data["id"], "error", str(e))
            broadcast({"event": "order_failed", "order": order_data["id"], "error": str(e)})
        except Exception as db_error:
            logger.error(f"💥 Additional error updating order status: {db_error}")

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

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
