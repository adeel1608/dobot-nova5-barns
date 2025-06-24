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
from typing import Dict, Any, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.rabbitmq_client import RabbitMQClient, EventListener

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
        "http://127.0.0.1:5173"
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
        await event_listener.subscribe_to_events([
            "oms.*", "scheduler.*", "validation.*", "automation.*", "routine.*"
        ])
        
        # Register event handlers for all order-related events
        # OMS Events
        event_listener.register_event_handler("oms.order_created", handle_order_event)
        event_listener.register_event_handler("oms.order_started", handle_order_event)
        event_listener.register_event_handler("oms.order_status_updated", handle_order_event)
        event_listener.register_event_handler("oms.order_halted", handle_order_event)
        event_listener.register_event_handler("oms.order_resumed", handle_order_event)
        event_listener.register_event_handler("oms.order_completed", handle_order_event)
        event_listener.register_event_handler("oms.order_failed", handle_order_event)
        event_listener.register_event_handler("oms.order_deleted", handle_order_event)
        
        # Scheduler Events
        event_listener.register_event_handler("scheduler.order_received", handle_order_event)
        event_listener.register_event_handler("scheduler.order_processing_started", handle_order_event)
        event_listener.register_event_handler("scheduler.order_completed", handle_order_event)
        event_listener.register_event_handler("scheduler.order_failed", handle_order_event)
        event_listener.register_event_handler("scheduler.order_error", handle_order_event)
        
        # Inventory Events
        event_listener.register_event_handler("validation.threshold_warning", handle_inventory_event)
        event_listener.register_event_handler("inventory.refilled", handle_inventory_event)
        
        logger.info("API Bridge service started successfully")
        
    except Exception as e:
        logger.error(f"Failed to start API Bridge service: {e}")
        raise

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up RabbitMQ connections on shutdown"""
    global rabbitmq_client, event_listener
    
    if rabbitmq_client:
        await rabbitmq_client.disconnect()
    if event_listener:
        await event_listener.disconnect()
    
    logger.info("API Bridge service stopped")

# Event handlers for real-time updates
async def handle_order_event(data: Dict):
    """Handle order-related events and broadcast to WebSocket clients"""
    logger.info(f"📡 Broadcasting order event to {len(active_websockets)} WebSocket clients: {data}")
    
    await broadcast_to_websockets({
        "type": "order_update",
        "event": data.get("event_type", "unknown"),
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def handle_inventory_event(data: Dict):
    """Handle inventory-related events and broadcast to WebSocket clients"""
    logger.info(f"📡 Broadcasting inventory event to {len(active_websockets)} WebSocket clients: {data}")
    
    await broadcast_to_websockets({
        "type": "inventory_update",
        "event": data.get("event_type", "unknown"),
        "data": data,
        "timestamp": datetime.now().isoformat()
    })

async def broadcast_to_websockets(message: Dict):
    """Broadcast message to all connected WebSocket clients"""
    if active_websockets:
        logger.info(f"📡 Broadcasting to {len(active_websockets)} WebSocket clients: {message.get('type', 'unknown')}")
        disconnected = []
        for websocket in active_websockets:
            try:
                await websocket.send_text(json.dumps(message))
            except Exception as e:
                logger.warning(f"Failed to send WebSocket message: {e}")
                disconnected.append(websocket)
        
        # Remove disconnected clients
        for ws in disconnected:
            if ws in active_websockets:
                active_websockets.remove(ws)
                logger.info(f"Removed disconnected WebSocket client. {len(active_websockets)} clients remaining.")
    else:
        logger.debug("No active WebSocket clients to broadcast to")

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
        logger.error(f"Error creating order: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/orders")
async def list_orders(status: Optional[str] = None):
    """List orders with optional status filter"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="list_orders",
            data={"status": status} if status else {},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to list orders"))
            
    except Exception as e:
        logger.error(f"Error listing orders: {e}")
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
        logger.error(f"Error getting order {order_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.patch("/api/orders/{order_id}/start")
async def start_order(order_id: int):
    """Start processing an order"""
    try:
        logger.info(f"🚀 Starting order {order_id} - sending RabbitMQ request to OMS")
        
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="start_order",
            data={"order_id": order_id},
            timeout=30
        )
        
        logger.info(f"📨 Received response from OMS for order {order_id}: {response}")
        
        if response.get("success"):
            logger.info(f"✅ Order {order_id} started successfully")
            return response
        else:
            logger.error(f"❌ Order {order_id} start failed: {response.get('error')}")
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to start order"))
            
    except Exception as e:
        logger.error(f"💥 Exception starting order {order_id}: {e}")
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
        logger.error(f"Error updating order {order_id}: {e}")
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
        logger.error(f"Error deleting order {order_id}: {e}")
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
        logger.error(f"Error halting order {order_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/orders/{order_id}/resume")
async def resume_order(order_id: int):
    """Resume a halted order"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="oms",
            action="resume_order",
            data={"order_id": order_id},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            raise HTTPException(status_code=400, detail=response.get("error", "Failed to resume order"))
            
    except Exception as e:
        logger.error(f"Error resuming order {order_id}: {e}")
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
        logger.error(f"Error getting queue: {e}")
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
        logger.error(f"Error reordering queue: {e}")
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
        logger.error(f"Error getting system status: {e}")
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
        logger.error(f"Error stopping system: {e}")
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
        logger.error(f"Error resuming system: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# Recipe Management Endpoints
@app.get("/api/recipes")
async def get_recipes():
    """Get available recipes from recipes.json file"""
    try:
        recipes_file = os.path.join('/app', 'data', 'recipes.json')
        
        if not os.path.exists(recipes_file):
            logger.error(f"Recipes file not found at {recipes_file}")
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
        
        logger.info(f"Successfully loaded {len(recipe_names)} recipes from file")
        return {
            "success": True,
            "data": recipe_names,  # Use 'data' field for consistency with dashboard API client
            "message": f"Successfully loaded {len(recipe_names)} recipes",
            "count": len(recipe_names),
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        logger.error(f"Error loading recipes: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to load recipes: {str(e)}")

# Inventory Management Endpoints
@app.get("/api/inventory/status")
async def get_inventory_status():
    """Get inventory status"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="inventory_status",
            data={},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            # Return mock data if validation service is not available
            logger.warning("Validation service not available, returning mock inventory data")
            return {
                "success": True,
                "inventory": {
                    "milk": {
                        "level": "medium",
                        "numeric": 65,
                        "last_refilled": "2024-01-15T10:30:00Z"
                    },
                    "cup": {
                        "level": "high", 
                        "numeric": 85,
                        "last_refilled": "2024-01-15T08:00:00Z"
                    },
                    "beans": {
                        "level": "low",
                        "numeric": 20,
                        "last_refilled": "2024-01-14T16:45:00Z"
                    },
                    "syrup": {
                        "level": "medium",
                        "numeric": 55,
                        "last_refilled": "2024-01-15T09:15:00Z"
                    }
                },
                "timestamp": datetime.now().isoformat()
            }
            
    except Exception as e:
        logger.error(f"Error getting inventory status: {e}")
        # Return mock data on error
        return {
            "success": True,
            "inventory": {
                "milk": {
                    "level": "medium",
                    "numeric": 65,
                    "last_refilled": "2024-01-15T10:30:00Z"
                },
                "cup": {
                    "level": "high", 
                    "numeric": 85,
                    "last_refilled": "2024-01-15T08:00:00Z"
                },
                "beans": {
                    "level": "low",
                    "numeric": 20,
                    "last_refilled": "2024-01-14T16:45:00Z"
                },
                "syrup": {
                    "level": "medium",
                    "numeric": 55,
                    "last_refilled": "2024-01-15T09:15:00Z"
                }
            },
            "timestamp": datetime.now().isoformat()
        }

@app.post("/api/inventory/refill")
async def refill_inventory(request: dict):
    """Refill inventory"""
    try:
        ingredient = request.get("ingredient")
        if not ingredient:
            raise HTTPException(status_code=400, detail="Ingredient is required")
            
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="inventory_refill",
            data={"ingredient": ingredient},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            # Return success for mock data
            logger.warning(f"Validation service not available, simulating refill for {ingredient}")
            return {
                "success": True,
                "message": f"Refill initiated for {ingredient}",
                "ingredient": ingredient,
                "timestamp": datetime.now().isoformat()
            }
            
    except Exception as e:
        logger.error(f"Error refilling inventory: {e}")
        # Return success for mock data
        return {
            "success": True,
            "message": f"Refill simulated for {request.get('ingredient', 'unknown')}",
            "ingredient": request.get('ingredient', 'unknown'),
            "timestamp": datetime.now().isoformat()
        }

@app.get("/api/inventory/category-summary")
async def get_inventory_category_summary():
    """Get inventory category summary with lowest levels per category"""
    try:
        response = await rabbitmq_client.send_request(
            target_service="validation",
            action="inventory_category_summary",
            data={},
            timeout=30
        )
        
        if response.get("success"):
            return response
        else:
            # Return mock category summary if validation service is not available
            logger.warning("Validation service not available, returning mock category summary")
            return {
                "success": True,
                "category_summary": {
                    "milk": {
                        "level": "medium",
                        "numeric": 45,
                        "last_refilled": None
                    },
                    "beans": {
                        "level": "low", 
                        "numeric": 20,
                        "last_refilled": None
                    },
                    "syrups": {
                        "level": "high",
                        "numeric": 75,
                        "last_refilled": None
                    },
                    "cups": {
                        "level": "medium",
                        "numeric": 55,
                        "last_refilled": None
                    }
                },
                "timestamp": datetime.now().isoformat()
            }
            
    except Exception as e:
        logger.error(f"Error getting inventory category summary: {e}")
        # Return mock data on error
        return {
            "success": True,
            "category_summary": {
                "milk": {
                    "level": "low",
                    "numeric": 0,
                    "last_refilled": None
                },
                "beans": {
                    "level": "low", 
                    "numeric": 0,
                    "last_refilled": None
                },
                "syrups": {
                    "level": "low",
                    "numeric": 0,
                    "last_refilled": None
                },
                "cups": {
                    "level": "low",
                    "numeric": 0,
                    "last_refilled": None
                }
            },
            "timestamp": datetime.now().isoformat()
        }

# Alert Management Endpoints
@app.get("/api/alerts/active")
async def get_active_alerts():
    """Get active alerts"""
    try:
        # Check if rabbitmq_client is available
        if rabbitmq_client is None:
            logger.warning("RabbitMQ client not initialized, returning empty alerts")
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
            logger.warning(f"OMS service doesn't have get_active_alerts endpoint or returned error: {response}")
            return {
                "success": True,
                "alerts": [],
                "timestamp": datetime.now().isoformat()
            }
            
    except TimeoutError as e:
        logger.warning(f"Timeout getting active alerts from OMS: {e}")
        return {
            "success": True,
            "alerts": [],
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"Error getting active alerts: {type(e).__name__}: {str(e)}")
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
            logger.warning("RabbitMQ client not initialized, returning empty acknowledged alerts")
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
            logger.warning(f"OMS service doesn't have get_acknowledged_alerts endpoint or returned error: {response}")
            return {
                "success": True,
                "alerts": [],
                "timestamp": datetime.now().isoformat()
            }
            
    except TimeoutError as e:
        logger.warning(f"Timeout getting acknowledged alerts from OMS: {e}")
        return {
            "success": True,
            "alerts": [],
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"Error getting acknowledged alerts: {type(e).__name__}: {str(e)}")
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
        logger.error(f"Error acknowledging alert {alert_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# WebSocket endpoint for real-time updates
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time dashboard updates"""
    await websocket.accept()
    active_websockets.append(websocket)
    client_id = f"{websocket.client.host}:{websocket.client.port}" if websocket.client else "unknown"
    logger.info(f"🔌 WebSocket client connected: {client_id}. Total clients: {len(active_websockets)}")
    
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
        logger.info(f"🔌 WebSocket client disconnected: {client_id}. Total clients: {len(active_websockets)}")
    except Exception as e:
        logger.error(f"🔌 WebSocket error for {client_id}: {e}")
        if websocket in active_websockets:
            active_websockets.remove(websocket)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000) 