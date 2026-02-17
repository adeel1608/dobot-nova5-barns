"""
API Bridge Service for BARNS Dashboard
Translates HTTP requests to RabbitMQ messages and vice versa
"""

import asyncio
import json
import logging
import os
import sys
from datetime import datetime
from typing import Dict, Any, Optional, Set, Tuple
import uuid

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import socketio

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.logger import log
from shared.rabbitmq_client import RabbitMQClient, EventListener

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create Socket.IO server
sio = socketio.AsyncServer(
    cors_allowed_origins="*",
    async_mode='asgi',
    logger=False,
    engineio_logger=False
)

app = FastAPI(title="BARNS API Bridge Service")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",      # Create React App default
        "http://127.0.0.1:3000",
        "http://localhost:3001",      # Alternative React port
        "http://127.0.0.1:3001",
        "http://localhost:5173",      # Vite default port
        "http://127.0.0.1:5173",
        "http://192.168.200.129:30003",
        "http://192.168.200.129:30001",
        "http://localhost:30003",
        "http://localhost:30001"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global RabbitMQ clients
rabbitmq_client: Optional[RabbitMQClient] = None
event_listener: Optional[EventListener] = None

# WebSocket connections for real-time updates
active_websockets = []


# Track client subscriptions for Socket.IO
client_subscriptions: Dict[str, Set[str]] = {}

# Statistics
stats = {
    "total_connections": 0,
    "active_connections": 0,
    "total_events_sent": 0,
    "events_by_topic": {}
}

# Request/Response models
class OrderCreate(BaseModel):
    cups: list

class OrderUpdate(BaseModel):
    status: str
    reason: Optional[str] = None

class InventoryRefill(BaseModel):
    ingredient: str
    amount: Optional[int] = 100

@app.on_event("startup")
async def startup_event():
    """Initialize RabbitMQ connections on startup"""
    global rabbitmq_client, event_listener
    
    try:
        # Initialize RabbitMQ client
        rabbitmq_client = RabbitMQClient("api_bridge")
        await rabbitmq_client.connect()
        
        # Initialize event listener for real-time updates
        event_listener = EventListener("api_bridge")
        await event_listener.connect()
        
        # Subscribe to events for real-time dashboard updates
        # Note: Use # for multi-level matching (e.g., validation.failed.dashboard)
        # * matches single word, # matches zero or more words
        await event_listener.subscribe_to_events([
            "oms.#", "scheduler.*", "validation.#", "automation.*", "routine.*"
        ])
        
        # Register event handlers for all order-related events
        # OMS Events
        event_listener.register_event_handler("oms.order_created", handle_order_event)
        event_listener.register_event_handler("oms.order_started", handle_order_event)
        event_listener.register_event_handler("oms.order_status_updated", handle_order_event)
        event_listener.register_event_handler("oms.order_stopping", handle_order_event)
        event_listener.register_event_handler("oms.order_stopped", handle_order_event)
        event_listener.register_event_handler("oms.order_halted", handle_order_event)
        event_listener.register_event_handler("oms.order_resumed", handle_order_event)
        event_listener.register_event_handler("oms.order_completed", handle_order_event)
        event_listener.register_event_handler("oms.order_failed", handle_order_event)
        event_listener.register_event_handler("oms.order_deleted", handle_order_event)
        event_listener.register_event_handler("oms.alert_created", handle_alert_event)
        
        # Scheduler Events
        event_listener.register_event_handler("scheduler.order_received", handle_order_event)
        event_listener.register_event_handler("scheduler.order_processing_started", handle_order_event)
        event_listener.register_event_handler("scheduler.order_completed", handle_order_event)
        event_listener.register_event_handler("scheduler.order_failed", handle_order_event)
        event_listener.register_event_handler("scheduler.order_error", handle_order_event)
        # Detailed scheduler task events (plan and per-task updates)
        event_listener.register_event_handler("scheduler.plan_built", handle_scheduler_plan_built_event)
        event_listener.register_event_handler("scheduler.status_update", handle_scheduler_status_update_event)
        event_listener.register_event_handler("scheduler.feedback_processed", handle_scheduler_feedback_processed_event)
        
        # Inventory Events
        event_listener.register_event_handler("validation.inventory_updated", handle_inventory_updated_event)
        event_listener.register_event_handler("validation.all_inventory_updated", handle_inventory_updated_event_all)
        event_listener.register_event_handler("validation.stock_level_updated", handle_stock_level_event)
        event_listener.register_event_handler("validation.category_summary_updated", handle_category_summary_event)
        
        # Validation Alert Events
        event_listener.register_event_handler("validation.threshold_warning", handle_validation_alert_event)
        event_listener.register_event_handler("validation.all_stations_occupied", handle_validation_alert_event)
        event_listener.register_event_handler("validation.retry_status", handle_validation_alert_event)
        # validation.failed.dashboard is handled by OMS (creates alert and broadcasts)
        
        # Log all registered handlers for debugging
        log("INFO", f"Registered event handlers: {list(event_listener.event_handlers.keys())}", service="api_bridge")
        log("INFO", "API Bridge service started successfully", service="api_bridge")
        
    except Exception as e:
        log("ERROR", f"Failed to start API Bridge service: {e}", service="api_bridge")
        raise

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up RabbitMQ connections on shutdown"""
    global rabbitmq_client, event_listener
    
    if rabbitmq_client:
        await rabbitmq_client.disconnect()
    if event_listener:
        await event_listener.disconnect()
    
    log("INFO", "API Bridge service stopped", service="api_bridge")

# Event handlers for real-time updates
async def handle_order_event(data: Dict):
    """Handle order-related events and broadcast to WebSocket clients"""
    log("DEBUG", "Broadcasting", service="api_bridge")
    await broadcast_to_websockets({
        "type": "order_update",
        "event": data.get("event_type", "unknown"),
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_scheduler_plan_built_event(data: Dict):
    """Forward per-arm plan to WebSocket clients so UI can render task list."""
    await broadcast_to_websockets({
        "type": "order_update",
        "event": "scheduler.plan_built",
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_scheduler_status_update_event(data: Dict):
    await broadcast_to_websockets({
        "type": "order_update",
        "event": "scheduler.status_update",
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_scheduler_feedback_processed_event(data: Dict):
    await broadcast_to_websockets({
        "type": "order_update",
        "event": "scheduler.feedback_processed",
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_inventory_event(data: Dict):
    """Handle inventory-related events and broadcast to WebSocket clients"""
    log("DEBUG", "Broadcasting", service="api_bridge")
    
    message = {
        "type": "inventory_update",
        "event": data.get("event_type", "unknown"),
        "data": data,
        "timestamp": datetime.now().isoformat()
    }

    await broadcast_to_websockets({
        "type": "inventory_update",
        "event": data.get("event_type", "unknown"),
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def broadcast_to_websockets(message: Dict):
    """Broadcast message to all connected WebSocket clients"""
    if active_websockets:
        log("INFO", f"Broadcasting to {len(active_websockets)} WebSocket clients: {message.get('type', 'unknown')}/{message.get('event', 'unknown')}", service="api_bridge")
        disconnected = []
        for websocket in active_websockets:
            try:
                await websocket.send_text(json.dumps(message))
                log("DEBUG", f"Successfully sent message to WebSocket client", service="api_bridge")
            except Exception as e:
                log("ERROR", f"Failed to send WebSocket message: {e}", service="api_bridge")
                disconnected.append(websocket)
        
        # Remove disconnected clients
        for ws in disconnected:
            if ws in active_websockets:
                active_websockets.remove(ws)
                log("INFO", f"Removed disconnected WebSocket client. {len(active_websockets)} clients remaining.", service="api_bridge")
    else:
        log("WARNING", f"No active WebSocket clients to broadcast to. Message: {message.get('type', 'unknown')}/{message.get('event', 'unknown')}", service="api_bridge")

# HTTP API Endpoints (translating to RabbitMQ)

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "api_bridge",
        "timestamp": datetime.now().isoformat()
    }

@app.get("/health")
async def root_health_check():
    """Root health check endpoint"""
    return {
        "status": "healthy",
        "service": "api_bridge",
        "timestamp": datetime.now().isoformat()
    }

# Order Management Endpoints
@app.post("/api/orders")
async def create_order(order: OrderCreate):
    """Create a new order"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="create_order",
            data={"order": order.dict()},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to create order"))
            
    except Exception as e:
        log("ERROR", f"Error creating order: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/orders")
async def list_orders(
    status: Optional[str] = None,
    limit: Optional[int] = Query(None, ge=1, le=100),
    offset: int = Query(0, ge=0)
):
    """List orders with optional status filter"""
    try:
        request_data = {}
        if status:
            request_data["status"] = status
        if limit is not None:
            request_data["limit"] = limit
        if offset:
            request_data["offset"] = offset

        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="list_orders",
            data=request_data,
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to list orders"))
            
    except Exception as e:
        log("ERROR", f"Error listing orders: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/orders/stats/summary")
async def get_orders_statistics():
    """Get order statistics - proxied directly to OMS service"""
    try:
        import httpx
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.get("http://oms-service:8000/orders/stats/summary")
            response.raise_for_status()
            result = response.json()
            return {
                "success": True,
                "data": result.get("stats", {}),
                "timestamp": datetime.now().isoformat()
            }
    except httpx.HTTPError as e:
        log("ERROR", f"Error fetching order statistics from OMS: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Failed to fetch order statistics: {str(e)}")
    except Exception as e:
        log("ERROR", f"Unexpected error fetching order statistics: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/orders/{order_id}")
async def get_order(order_id: int):
    """Get a specific order by ID"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="get_order",
            data={"order_id": order_id},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=404, detail=response.get("error", "Order not found"))
            
    except Exception as e:
        log("ERROR", f"Error getting order {order_id}: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.patch("/api/orders/{order_id}/start")
async def start_order(order_id: int):
    """Start processing an order"""
    try:
        log("INFO", "Starting", service="api_bridge")
        
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="start_order",
            data={"order_id": order_id},
            timeout=30
        )
        
        log("INFO", f"Received response from OMS for order {order_id}: {response}", service="api_bridge")
        
        if response.get("success"):
            log("INFO", f"Order {order_id} start successful", service="api_bridge")
            return response
        else:
            log("ERROR", f"Order {order_id} start failed: {response.get('error', 'Unknown')[:50]}", service="api_bridge")
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to start order"))
            
    except Exception as e:
        log("ERROR", "Error", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.patch("/api/orders/{order_id}/status")
async def update_order_status(order_id: int, update: OrderUpdate):
    """Update order status"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="update_order_status",
            data={
                "order_id": order_id,
                "status": update.status,
                "reason": update.reason
            },
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to update order"))
            
    except Exception as e:
        log("ERROR", f"Error updating order {order_id}: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.delete("/api/orders/{order_id}")
async def delete_order(order_id: int):
    """Delete an order"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="delete_order",
            data={"order_id": order_id},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to delete order"))
            
    except Exception as e:
        log("ERROR", f"Error deleting order {order_id}: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/orders/{order_id}/halt")
async def halt_order(order_id: int, reason: str = None):
    """Halt an order with reason"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="halt_order",
            data={"order_id": order_id, "reason": reason},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to halt order"))
            
    except Exception as e:
        log("ERROR", f"Error halting order {order_id}: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/orders/{order_id}/stop")
async def stop_order(order_id: int):
    """Stop a processing order"""
    try:
        log("INFO", "Stop", service="api_bridge")
        
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="stop_order",
            data={"order_id": order_id},
            timeout=30
        )
        
        log("INFO", "Stop", service="api_bridge")
        
        if response.get("success"):
            log("INFO", f"Order {order_id} stop successful", service="api_bridge")
            return response
        else:
            log("ERROR", f"Order {order_id} stop failed: {response.get('error', 'Unknown')[:50]}", service="api_bridge")
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to stop order"))
            
    except Exception as e:
        log("ERROR", "Error", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/orders/{order_id}/resume")
async def resume_order(order_id: int):
    """Resume a stopped/halted order"""
    try:
        log("INFO", "Resume", service="api_bridge")
        
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="resume_order",
            data={"order_id": order_id},
            timeout=30
        )
        
        log("INFO", "Resume", service="api_bridge")
        
        if response.get("success"):
            log("INFO", f"Order {order_id} resume successful", service="api_bridge")
            return response
        else:
            log("ERROR", f"Order {order_id} resume failed: {response.get('error', 'Unknown')[:50]}", service="api_bridge")
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to resume order"))
            
    except Exception as e:
        log("ERROR", "Error", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

# Queue Management Endpoints
@app.get("/api/queue")
async def get_queue():
    """Get current order queue"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="sync_queue",
            data={},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to get queue"))
            
    except Exception as e:
        log("ERROR", f"Error getting queue: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.put("/api/queue/reorder")
async def reorder_queue(order_data: dict):
    """Reorder the queue"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="bulk_reorder_queue",
            data=order_data,
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to reorder queue"))
            
    except Exception as e:
        log("ERROR", f"Error reordering queue: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

# System Status Endpoints
@app.get("/api/system/status")
async def get_system_status():
    """Get overall system status"""
    try:
        # Get status from multiple services
        services = ["oms", "scheduler", "routine", "validation", "automation"]
        statuses = {}
        
        for service in services:
            try:
                response = await rabbitmq_client.send_request(
                    target_service=service,
                    action="health",
                    data={},
                    timeout=10
                )
                statuses[service] = response
            except Exception as e:
                statuses[service] = {"status": "error", "error": str(e)}
        
        return {
            "success": True,
            "services": statuses,
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        log("ERROR", f"Error getting system status: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/system/stop")
async def stop_system(request: dict):
    """Emergency stop system"""
    try:
        reason = request.get("reason", "Emergency stop requested")
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="emergency_stop",
            data={"reason": reason},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to stop system"))
            
    except Exception as e:
        log("ERROR", f"Error stopping system: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/system/resume")
async def resume_system():
    """Resume system operations"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="resume_operations",
            data={},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to resume system"))
            
    except Exception as e:
        log("ERROR", f"Error resuming system: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

# POS Integration Endpoints
@app.post("/api/pos/process-order")
async def process_pos_order(order_data: dict):
    """Proxy POS order processing to OMS service"""
    try:
        import httpx
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(
                "http://oms-service:8000/pos/process-order",
                json=order_data
            )
            response.raise_for_status()
            return response.json()
    except httpx.HTTPError as e:
        log("ERROR", f"Error proxying POS order to OMS: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Failed to process POS order: {str(e)}")
    except Exception as e:
        log("ERROR", f"Unexpected error processing POS order: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/pos/menu-items")
async def get_menu_items():
    """Proxy menu items request to OMS service"""
    try:
        import httpx
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.get("http://oms-service:8000/pos/menu-items")
            response.raise_for_status()
            return response.json()
    except httpx.HTTPError as e:
        log("ERROR", f"Error fetching menu items from OMS: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Failed to fetch menu items: {str(e)}")
    except Exception as e:
        log("ERROR", f"Unexpected error fetching menu items: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/pos/ingredients")
async def get_ingredients():
    """Proxy ingredients request to OMS service"""
    try:
        import httpx
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.get("http://oms-service:8000/pos/ingredients")
            response.raise_for_status()
            return response.json()
    except httpx.HTTPError as e:
        log("ERROR", f"Error fetching ingredients from OMS: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Failed to fetch ingredients: {str(e)}")
    except Exception as e:
        log("ERROR", f"Unexpected error fetching ingredients: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))

# Recipe Management Endpoints
@app.get("/api/recipes")
async def get_recipes():
    """Get available recipes from recipes.json file"""
    try:
        recipes_file = os.path.join('/app', 'data', 'recipes.json')
        
        if not os.path.exists(recipes_file):
            log("ERROR", f"Recipes file not found at {recipes_file}", service="api_bridge")
            raise HTTPException(status_code=500, detail="Recipes file not found")
        
        with open(recipes_file, 'r', encoding='utf-8') as f:
            recipes_data = json.load(f)
        
        # Extract recipe names and convert to title case for display
        recipe_names = []
        for recipe_name in recipes_data.keys():
            # Convert from lowercase to title case (e.g., "latte" -> "Latte")
            display_name = recipe_name.replace('_', ' ').title()
            recipe_names.append({
                "name": recipe_name,
                "display_name": display_name,
                "steps": len(recipes_data[recipe_name])
            })
        
        log("INFO", "Successfully loaded {len(recipe_names)} recipes from file", service="api_bridge")
        return {
            "success": True,
            "data": recipe_names,  # Use 'data' field for consistency with dashboard API client
            "message": f"Successfully loaded {len(recipe_names)} recipes",
            "count": len(recipe_names),
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        log("ERROR", f"Error loading recipes: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Failed to load recipes: {str(e)}")

# Inventory Management Endpoints
@app.post("/api/inventory/test_summary")
async def test_summary():
    """Test summary endpoint"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="category_summary",
            data={},
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
        # publish to socket io 
            await handle_category_summary_event(response.get("details", {}))
        else:
            error_msg = response.get("error", "Failed to get category summary from validation service")
            log("ERROR", f"Validation service returned error: {error_msg}", service="api_bridge")
            raise HTTPException(status_code=503, detail=error_msg)
            
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        log("ERROR", f"Error getting inventory category summary: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@app.get("/api/inventory/status")
async def get_inventory_status(ingredient_type: Optional[str] = None, subtype: Optional[str] = None):
    """Get inventory status - all, by type, or specific item"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="inventory_status",
            data={
                "ingredient_type": ingredient_type,
                "subtype": subtype
            },
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
            # Return hierarchical format as-is - no flattening
            return {
                "success": True,
                "inventory": response.get("details", {}),
                "timestamp": datetime.now().isoformat()
            }
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to get inventory status"))
            
    except Exception as e:
        log("ERROR", f"Error getting inventory status: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))
    
@app.get("/api/inventory/category-info")
async def get_inventory_category_info():
    """Get inventory category info"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="category_info",
            data={},
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
            return response.get("details", {})
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to get inventory category info"))
            
    except Exception as e:
        log("ERROR", f"Error getting inventory category info: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))


# Mapping from frontend ingredient names to database subtype names
# Based on validation_schema.sql - only items that exist in the database
FRONTEND_TO_DB_SUBTYPE = {
    # Milk mappings (frontend name -> database subtype)
    "whole_milk": "whole_fat_milk",
    "skim_milk": "low_fat_milk",  # skim_milk maps to low_fat_milk in DB
    "almond_milk": "almond_milk",  # matches
    "lactose_free_milk": "lactose_free_milk",  # matches
    "low_fat_milk": "low_fat_milk",  # matches
    "whole_fat_milk": "whole_fat_milk",  # matches
    # Note: soy_milk, oat_milk, coconut_milk, rice_milk, heavy_cream don't exist in DB
    
    # Syrup mappings (frontend name -> database subtype)
    "vanilla_syrup": "vanilla_syrup",  # matches
    "caramel_syrup": "caramel_syrup",  # matches
    "hazelnut_syrup": "hazelnut_syrup",  # matches
    "white_chocolate_sauce": "white_chocolate_sauce",  # matches
    "caramel_sauce": "caramel_sauce",  # matches
    "condense_milk_sauce": "condense_milk_sauce",  # matches
    "peached_iced_syrup": "peached_iced_syrup",  # matches
    "passion_fruit_iced_syrup": "passion_fruit_iced_syrup",  # matches
    "ice_tea_syrup": "ice_tea_syrup",  # matches
    # Note: cinnamon_syrup, peppermint_syrup, irish_cream_syrup, amaretto_syrup,
    # coconut_syrup, raspberry_syrup, lavender_syrup, maple_syrup don't exist in DB
    
    # Premixes (frontend name -> database subtype)
    "mocha_frappe": "mocha_frappe",  # matches
    "chocolate_frappe": "chocolate_frappe",  # matches
    "half_and_half": "half_and_half",  # matches
}

def parse_ingredient_string(ingredient: str) -> Tuple[Optional[str], Optional[str]]:
    """
    Parse ingredient string into ingredient_type and subtype.
    Maps frontend ingredient names to database subtype names.
    
    Handles formats like:
    - "coffee_beans_regular" -> ("coffee_beans", "regular")
    - "whole_milk" -> ("milk", "whole_fat_milk")  # mapped to DB name
    - "cup_H7" -> ("cups", "cup_H7")
    - "vanilla_syrup" -> ("syrups", "vanilla_syrup")
    - "milk" -> ("milk", None)  # refill all milk
    - "cup" or "cups" -> ("cups", None)  # refill all cups
    - "beans" -> ("coffee_beans", None)  # refill all coffee beans
    - "syrup" -> ("syrups", None)  # refill all syrups
    """
    if not ingredient:
        return None, None
    
    ingredient_lower = ingredient.lower().strip()
    
    # Handle simple category names (refill entire category)
    if ingredient_lower in ["milk"]:
        return "milk", None
    elif ingredient_lower in ["cup", "cups"]:
        return "cups", None
    elif ingredient_lower in ["beans", "coffee", "coffee_beans"]:
        return "coffee_beans", None
    elif ingredient_lower in ["syrup", "syrups"]:
        return "syrups", None
    elif ingredient_lower in ["premix", "premixes"]:
        return "premixes", None
    
    # Handle compound names with underscores
    parts = ingredient.split("_")
    
    # Coffee beans: "coffee_beans_regular" or "coffee_beans_decaf"
    if len(parts) >= 3 and parts[0] == "coffee" and parts[1] == "beans":
        subtype = "_".join(parts[2:])  # "regular" or "decaf"
        return "coffee_beans", subtype
    elif len(parts) == 2 and parts[0] == "coffee" and parts[1] == "beans":
        # "coffee_beans" without subtype - refill all coffee beans
        return "coffee_beans", None
    
    # Cups: "cup_H7", "cup_H9", "cup_C7", etc.
    if len(parts) >= 2 and parts[0] == "cup":
        subtype = ingredient  # Keep full name like "cup_H7"
        return "cups", subtype
    
    # Milk: Map frontend names to database subtypes
    if ingredient_lower.endswith("_milk") or ingredient_lower in ["heavy_cream"]:
        # Map frontend name to database subtype name
        db_subtype = FRONTEND_TO_DB_SUBTYPE.get(ingredient_lower)
        if db_subtype:
            return "milk", db_subtype
        else:
            # Ingredient doesn't exist in database - log warning and return None to skip
            log("WARNING", f"Ingredient '{ingredient}' (milk) not found in database, skipping refill", service="api_bridge")
            return None, None
    
    # Syrups: Map frontend names to database subtypes
    if ingredient_lower.endswith("_syrup") or ingredient_lower.endswith("_sauce"):
        # Map frontend name to database subtype name
        db_subtype = FRONTEND_TO_DB_SUBTYPE.get(ingredient_lower)
        if db_subtype:
            return "syrups", db_subtype
        else:
            # Ingredient doesn't exist in database - log warning and return None to skip
            log("WARNING", f"Ingredient '{ingredient}' (syrup) not found in database, skipping refill", service="api_bridge")
            return None, None
    
    # Premixes: Map frontend names to database subtypes
    if ingredient_lower.endswith("_frappe") or "premix" in ingredient_lower or ingredient_lower == "half_and_half":
        # Map frontend name to database subtype name
        db_subtype = FRONTEND_TO_DB_SUBTYPE.get(ingredient_lower)
        if db_subtype:
            return "premixes", db_subtype
        else:
            # Ingredient doesn't exist in database - log warning and return None to skip
            log("WARNING", f"Ingredient '{ingredient}' (premix) not found in database, skipping refill", service="api_bridge")
            return None, None
    
    # Default: assume it's a subtype of unknown category
    # Try to infer category from common patterns
    log("WARNING", f"Could not parse ingredient '{ingredient}', treating as subtype only", service="api_bridge")
    return None, ingredient

@app.post("/api/inventory/refill")
async def refill_inventory(request: InventoryRefill):
    """Refill inventory - accepts ingredient string and parses it into ingredient_type and subtype"""
    try:
        # Parse the ingredient string
        ingredient_type, subtype = parse_ingredient_string(request.ingredient)
        
        # If ingredient doesn't exist in database, return success (skip it)
        if ingredient_type is None and subtype is None:
            log("INFO", f"Ingredient '{request.ingredient}' not found in database, skipping refill", service="api_bridge")
            return {
                "passed": True,
                "details": {"message": f"Ingredient '{request.ingredient}' not found in database, skipped"},
                "request_id": uuid.uuid4(),
                "client_type": "api_bridge"
            }
        
        log("INFO", f"Refilling inventory: ingredient='{request.ingredient}' -> ingredient_type='{ingredient_type}', subtype='{subtype}'", service="api_bridge")
        
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="inventory_refill",
            data={
                "ingredient_type": ingredient_type, 
                "subtype": subtype
            },
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
            return response
        else:
            # Return success for mock data
            log("ERROR", "Validation service not available, simulating refill", service="api_bridge")
            return {
                "passed": False,
                "details": {},
                "request_id": uuid.uuid4(),
                "client_type": "api_bridge"
            }
            
    except Exception as e:
        log("ERROR", f"Error refilling inventory: {e}", service="api_bridge")
        # Return success for mock data
        return {
            "passed": False,
            "details": {},
            "request_id": uuid.uuid4(),
            "client_type": "api_bridge"
        }

@app.get("/api/inventory/category-summary")
async def get_inventory_category_summary():
    """Get inventory category summary with lowest levels per category"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="category_summary",
            data={},
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
            return {
                "success": True,
                "summary": response.get("details", {}),
                "timestamp": datetime.now().isoformat()
            }
        else:
            error_msg = response.get("error", "Failed to get category summary from validation service")
            log("ERROR", f"Validation service returned error: {error_msg}", service="api_bridge")
            raise HTTPException(status_code=503, detail=error_msg)
            
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        log("ERROR", f"Error getting inventory category summary: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.get("/api/inventory/stock-level")
async def get_inventory_stock_level():
    """Get inventory stock level statistics"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="stock_level",
            data={},
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
            return {
                "success": True,
                "stock_level": response.get("details", {}),
                "timestamp": datetime.now().isoformat()
            }
        else:
            raise HTTPException(status_code=400, detail="Failed to get severity statistics")
            
    except Exception as e:
        log("ERROR", f"Error getting inventory severity: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))
    
@app.get("/api/inventory/category-count")
async def get_inventory_category_count():
    """Get inventory category count"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="category_count",
            data={},
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
            return {
                "success": True,
                "request_id": response.get("request_id"),
                "client_type": response.get("client_type"),
                "details": response.get("details", {})
            }
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to get inventory category count"))
            
    except Exception as e:
        log("ERROR", f"Error getting inventory category count: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/inventory/by-stock-level/{stock_level}")
async def get_inventory_by_stock_level(stock_level: str):
    """Get inventory items filtered by stock level (high, medium, low, empty)"""
    try:
        # Validate stock level parameter
        valid_levels = ["high", "medium", "low", "empty"]
        if stock_level not in valid_levels:
            raise HTTPException(
                status_code=400, 
                detail=f"Invalid stock level. Must be one of: {', '.join(valid_levels)}"
            )
        
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="inventory_by_stock_level",
            data={
                "stock_level": stock_level
            },
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
            return {
                "success": True,
                "stock_level": stock_level,
                "ingredients": response.get("details", {}),
                "timestamp": datetime.now().isoformat()
            }
        else:
            raise HTTPException(
                status_code=400, 
                detail=response.get("error", f"Failed to get {stock_level} stock ingredients")
            )
            
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        log("ERROR", f"Error getting {stock_level} stock ingredients: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")           


@app.post("/api/inventory/update-limits")
async def update_inventory_limits(request: Request):
    """Update ingredient capacity limits (max_capacity, warning_threshold, critical_threshold)"""
    try:
        body = await request.json()
        updates = body.get("updates", [])
        
        if not updates:
            raise HTTPException(status_code=400, detail="No updates provided")
        
        # Validate updates format
        for update in updates:
            if not all(k in update for k in ['category', 'subtype', 'field', 'value']):
                raise HTTPException(status_code=400, detail="Invalid update format")
            
            if update['field'] not in ['max_capacity', 'warning_threshold', 'critical_threshold']:
                raise HTTPException(status_code=400, detail=f"Invalid field: {update['field']}")
        
        # Send request to validation service
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="update_limits",
            data={
                "updates": updates
            },
            timeout=30
        )
        
        if response.get("success") or response.get("passed"):
            return {
                "success": True,
                "message": "Inventory limits updated successfully",
                "details": response.get("details", {}),
                "timestamp": datetime.now().isoformat()
            }
        else:
            raise HTTPException(
                status_code=400, 
                detail=response.get("error", "Failed to update inventory limits")
            )
            
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        log("ERROR", f"Error updating inventory limits: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

# Alert Management Endpoints
@app.get("/api/alerts/active")
async def get_active_alerts():
    """Get active alerts"""
    try:
        # Check if rabbitmq_client is available
        if rabbitmq_client is None:
            log("ERROR", "RabbitMQ client not initialized, returning empty alerts", service="api_bridge")
            return {
                "success": True,
                "alerts": [],
                "timestamp": datetime.now().isoformat()
            }
        
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="get_active_alerts",
            data={},
            timeout=10
        )
        
        if response and response.get("success"):
            return response
        else:
            # Return empty alerts if OMS doesn't have this endpoint yet
            log("ERROR", f"OMS service doesn't have get_active_alerts endpoint or returned error: {response}", service="api_bridge")
            return {
                "success": True,
                "alerts": [],
                "timestamp": datetime.now().isoformat()
            }
            
    except TimeoutError as e:
        log("ERROR", f"Timeout getting active alerts from OMS: {e}", service="api_bridge")
        return {
            "success": True,
            "alerts": [],
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        log("ERROR", "Error getting active alerts: {type(e).__name__}: {str(e)}", service="api_bridge")
        # Return empty alerts on error
        return {
            "success": True,
            "alerts": [],
            "timestamp": datetime.now().isoformat()
        }

@app.get("/api/alerts/acknowledged")
async def get_acknowledged_alerts():
    """Get acknowledged alerts"""
    try:
        # Check if rabbitmq_client is available
        if rabbitmq_client is None:
            log("ERROR", "RabbitMQ client not initialized, returning empty acknowledged alerts", service="api_bridge")
            return {
                "success": True,
                "alerts": [],
                "timestamp": datetime.now().isoformat()
            }
        
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="get_acknowledged_alerts",
            data={},
            timeout=10
        )
        
        if response and response.get("success"):
            return response
        else:
            # Return empty alerts if OMS doesn't have this endpoint yet
            log("ERROR", f"OMS service doesn't have get_acknowledged_alerts endpoint or returned error: {response}", service="api_bridge")
            return {
                "success": True,
                "alerts": [],
                "timestamp": datetime.now().isoformat()
            }
            
    except TimeoutError as e:
        log("ERROR", f"Timeout getting acknowledged alerts from OMS: {e}", service="api_bridge")
        return {
            "success": True,
            "alerts": [],
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        log("ERROR", "Error getting acknowledged alerts: {type(e).__name__}: {str(e)}", service="api_bridge")
        # Return empty alerts on error
        return {
            "success": True,
            "alerts": [],
            "timestamp": datetime.now().isoformat()
        }

@app.post("/api/alerts/{alert_id}/acknowledge")
async def acknowledge_alert(alert_id: int):
    """Acknowledge an alert"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="acknowledge_alert",
            data={"alert_id": alert_id},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to acknowledge alert"))
            
    except Exception as e:
        log("ERROR", f"Error acknowledging alert {alert_id}: {e}", service="api_bridge")
        raise HTTPException(status_code=500, detail=str(e))



#------------------------------------------------------------------------
# socket io endpoints
#------------------------------------------------------------------------


# Socket.IO Event Handlers - SIMPLIFIED
@sio.event
async def connect(sid, environ):
    """Handle client connection"""
    stats["total_connections"] += 1
    stats["active_connections"] += 1
    
    log("DEBUG", "Connecting", service="api_bridge")
    
    # Send welcome message
    await sio.emit('connected', {
        "status": "connected",
        "client_id": sid,
        "message": "Socket.IO connection established",
        "timestamp": datetime.now().isoformat()
    }, room=sid)

@sio.event
async def disconnect(sid):
    """Handle client disconnection"""
    stats["active_connections"] -= 1
    log("DEBUG", "Connecting", service="api_bridge")

@sio.event
async def ping(sid):
    """Handle ping for heartbeat"""
    await sio.emit('pong', {
        "timestamp": datetime.now().isoformat()
    }, room=sid)

# Update emission functions to broadcast to ALL clients
async def emit_inventory_update(category: str, inventory_data: Dict):
    """Emit inventory update for specific category"""
    # Emit with specific event name
    await sio.emit(f'inventory.update.{category}', {
        "category": category,
        "inventory": inventory_data,
        "timestamp": datetime.now().isoformat()
    })
    
    # Also emit general update
    await sio.emit('inventory.update', {
        "category": category,
        "inventory": inventory_data,
        "timestamp": datetime.now().isoformat()
    })
    
    log("DEBUG", "Broadcasting", service="api_bridge")

async def emit_stock_level_update(stock_data: Dict):
    """Emit stock level statistics update"""
    await sio.emit('inventory.stock_level', {
        "success": True,
        "stock_levels": stock_data,
        "timestamp": datetime.now().isoformat()
    })
    
    log("DEBUG", "Broadcasting", service="api_bridge")

async def emit_inventory_summary(summary_data: Dict):
    """Emit inventory category summary update"""
    await sio.emit('inventory.summary', {
        "success": True,
        "summary": summary_data,
        "timestamp": datetime.now().isoformat()
    })
    
    log("DEBUG", "Broadcasting", service="api_bridge")

# Replace your existing event handlers with these:
async def handle_inventory_updated_event(data: Dict):
    """Handle category-specific inventory update events"""
    category = data.get("category")
    inventory_data = data.get("inventory", {})
    
    log("INFO", f"Received inventory update for category: {category}", service="api_bridge")    
    
    # Emit to Socket.IO clients
    await emit_inventory_update(category, inventory_data)
    
    # Also broadcast to WebSocket clients for real-time dashboard updates
    await broadcast_to_websockets({
        "type": "inventory_update",
        "event": "inventory_updated",
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_stock_level_event(data: Dict):
    """Handle stock level summary update events"""
    log("INFO", "Received stock level update", service="api_bridge")
    
    # Emit to Socket.IO clients
    await emit_stock_level_update(data)
    
    # Also broadcast to WebSocket clients for real-time dashboard updates
    await broadcast_to_websockets({
        "type": "inventory_update",
        "event": "stock_level_updated",
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_category_summary_event(data: Dict):
    """Handle category summary update events"""
    log("INFO", "Received category summary update", service="api_bridge")
    
    # Emit to Socket.IO clients
    await emit_inventory_summary(data)
    
    # Also broadcast to WebSocket clients for real-time dashboard updates
    await broadcast_to_websockets({
        "type": "inventory_update",
        "event": "category_summary_updated",
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_inventory_updated_event_all(data: Dict):
    """Handle all inventory update events"""
    log("INFO", "Received all inventory update", service="api_bridge")
    
    # Emit to Socket.IO clients
    await emit_inventory_update_all(data)
    
    # Also broadcast to WebSocket clients for real-time dashboard updates
    await broadcast_to_websockets({
        "type": "inventory_update",
        "event": "all_inventory_updated",
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def emit_inventory_update_all(data: Dict):
    """Emit all inventory update"""
    await sio.emit('inventory.status', {
        "success": True,
        "inventory": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_validation_alert_event(data: Dict):
    """Handle validation alert events (threshold_warning, all_stations_occupied, retry_status)"""
    event_type = data.get("_event_type", "validation_alert")  # Get the actual event type
    log("INFO", f"Received validation alert event: {event_type}", service="api_bridge")
    log("DEBUG", f"Alert data: {data}", service="api_bridge")
    
    # Broadcast to WebSocket clients
    await broadcast_to_websockets({
        "type": "alert",
        "event": event_type,
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_alert_event(data: Dict):
    """Handle alert events from OMS (alert_created, validation_failed, etc)"""
    log("INFO", f"Received alert from OMS: {data.get('event', 'unknown')}", service="api_bridge")
    # Forward alert directly to dashboard via WebSocket
    await broadcast_to_websockets(data)

# Note: validation.failed.dashboard is handled entirely by OMS service
# OMS creates the alert in database and broadcasts to all WebSocket clients
# No need for API Bridge to duplicate this functionality

# Add Socket.IO stats endpoint
@app.get("/api/socketio/stats")
async def get_socketio_stats():
    """Get Socket.IO connection statistics"""
    return {
        "success": True,
        "stats": stats,
        "active_topics": list(set().union(*client_subscriptions.values())) if client_subscriptions else [],
        "timestamp": datetime.now().isoformat()
    }

# WebSocket endpoint for real-time updates
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time dashboard updates"""
    await websocket.accept()
    active_websockets.append(websocket)
    client_id = f"{websocket.client.host}:{websocket.client.port}" if websocket.client else "unknown"
    log("DEBUG", "Connecting", service="api_bridge")
    
    # Send welcome message
    await websocket.send_text(json.dumps({
        "type": "connection",
        "status": "connected",
        "message": "Real-time updates enabled",
        "timestamp": datetime.now().isoformat()
    }))
    
    try:
        while True:
            # Keep connection alive and handle any incoming messages
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
                if message.get("type") == "ping":
                    # Respond to ping with pong
                    await websocket.send_text(json.dumps({
                        "type": "pong", 
                        "timestamp": datetime.now().isoformat()
                    }))
                else:
                    # Echo back other messages for debugging
                    await websocket.send_text(json.dumps({
                        "type": "echo", 
                        "received": message,
                        "timestamp": datetime.now().isoformat()
                    }))
            except json.JSONDecodeError:
                # Handle non-JSON messages
                await websocket.send_text(json.dumps({
                    "type": "error", 
                    "message": "Invalid JSON received",
                    "timestamp": datetime.now().isoformat()
                }))
            
    except WebSocketDisconnect:
        if websocket in active_websockets:
            active_websockets.remove(websocket)
        log("DEBUG", "Connecting", service="api_bridge")
    except Exception as e:
        log("ERROR", f"WebSocket error for {client_id}: {e}", service="api_bridge")
        if websocket in active_websockets:
            active_websockets.remove(websocket)


# Mount Socket.IO app
socket_app = socketio.ASGIApp(sio, app)

if __name__ == "__main__":
    uvicorn.run(socket_app, host="0.0.0.0", port=8000) 
