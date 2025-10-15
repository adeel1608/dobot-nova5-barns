import asyncio
import json
import logging
import os
import sys
from datetime import datetime
from typing import Dict, List, Optional

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.rabbitmq_client import RabbitMQClient, EventListener
from . import db, queue, models
from .pos_core import load_reference_data_from_db

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Order status constants
ORDER_STATUS = {
    'QUEUED': 'queued',
    'PROCESSING': 'processing', 
    'COMPLETED': 'completed',
    'HALTED': 'halted',
    'STOPPED': 'stopped',
    'ERROR': 'error',
    'CANCELLED': 'cancelled'
}

# Task status constants  
TASK_STATUS = {
    'QUEUED': 'queued',
    'RUNNING': 'running',
    'COMPLETED': 'completed',
    'FAILED': 'failed',
    'HALTED': 'halted'
}

class OMSService:
    def __init__(self):
        self.rabbitmq_client = RabbitMQClient("oms")
        self.event_listener = EventListener("oms")
        self.dashboard_subscribers = []  # Track dashboard connections
        
    async def start(self):
        """Start the OMS service"""
        # Initialize database and queue connections
        db.connect()
        queue.connect()
        
        # Initialize POS reference data
        try:
            pos_db_path = os.environ.get("POS_DB_PATH", "pos_reference.db")
            success = load_reference_data_from_db(pos_db_path)
            if not success:
                logger.warning("Could not load POS reference data. Running with empty references.")
            else:
                logger.info("POS reference data loaded successfully (MQ service)")
        except Exception as e:
            logger.error(f"Failed to load POS reference data (MQ service): {e}")
        
        # Sync queue with database on startup
        logger.info("Syncing queue with database on startup...")
        queue.sync_with_database()
        
        await self.rabbitmq_client.connect()
        await self.event_listener.connect()
        
        # Register message handlers
        self._register_handlers()
        
        # Subscribe to relevant events
        await self.event_listener.subscribe_to_events([
            "scheduler.*", "validation.*", "automation.*", "routine.*", "system.*"
        ])
        self._register_event_handlers()
        
        logger.info("OMS service started and listening for messages")
        
        # Keep the service running
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down OMS service...")
        finally:
            await self.stop()
    
    async def stop(self):
        """Stop the OMS service"""
        await self.rabbitmq_client.disconnect()
        await self.event_listener.disconnect()
        logger.info("OMS service stopped")
    
    def _register_handlers(self):
        """Register all message handlers"""
        # Order management
        self.rabbitmq_client.register_handler("create_order", self.handle_create_order)
        self.rabbitmq_client.register_handler("list_orders", self.handle_list_orders)
        self.rabbitmq_client.register_handler("get_order", self.handle_get_order)
        self.rabbitmq_client.register_handler("update_order_status", self.handle_update_order_status)
        self.rabbitmq_client.register_handler("start_order", self.handle_start_order)
        self.rabbitmq_client.register_handler("halt_order", self.handle_halt_order)
        self.rabbitmq_client.register_handler("resume_order", self.handle_resume_order)
        self.rabbitmq_client.register_handler("complete_order", self.handle_complete_order)
        self.rabbitmq_client.register_handler("fail_order", self.handle_fail_order)
        self.rabbitmq_client.register_handler("delete_order", self.handle_delete_order)
        
        # Queue management
        self.rabbitmq_client.register_handler("reorder_queue", self.handle_reorder_queue)
        self.rabbitmq_client.register_handler("bulk_reorder_queue", self.handle_bulk_reorder_queue)
        self.rabbitmq_client.register_handler("sync_queue", self.handle_sync_queue)
        
        # System management
        self.rabbitmq_client.register_handler("stop_system", self.handle_stop_system)
        self.rabbitmq_client.register_handler("resume_system", self.handle_resume_system)
        self.rabbitmq_client.register_handler("get_system_status", self.handle_get_system_status)
        
        # Inventory management
        self.rabbitmq_client.register_handler("threshold_warning", self.handle_threshold_warning)
        self.rabbitmq_client.register_handler("refill_inventory", self.handle_refill_inventory)
        self.rabbitmq_client.register_handler("get_inventory_status", self.handle_get_inventory_status)
        
        # Alert management
        self.rabbitmq_client.register_handler("get_active_alerts", self.handle_get_active_alerts)
        self.rabbitmq_client.register_handler("get_acknowledged_alerts", self.handle_get_acknowledged_alerts)
        self.rabbitmq_client.register_handler("acknowledge_alert", self.handle_acknowledge_alert)
        
        # Dashboard subscription
        self.rabbitmq_client.register_handler("subscribe_dashboard", self.handle_subscribe_dashboard)
        self.rabbitmq_client.register_handler("health", self.handle_health)
    
    def _register_event_handlers(self):
        """Register event handlers"""
        self.event_listener.register_event_handler("scheduler.order_completed", self.handle_order_completed_event)
        self.event_listener.register_event_handler("scheduler.order_failed", self.handle_order_failed_event)
        self.event_listener.register_event_handler("scheduler.order_heartbeat", self.handle_order_heartbeat_event)
        self.event_listener.register_event_handler("validation.threshold_warning", self.handle_threshold_warning_event)
        self.event_listener.register_event_handler("system.shutdown", self.handle_shutdown_event)
    
    # Order Management Handlers
    async def handle_create_order(self, data: Dict) -> Dict:
        """Handle order creation requests"""
        try:
            order_data = data.get("order")
            if not order_data:
                return {"success": False, "error": "Missing order data"}
            
            # Save order to database
            order_id = db.save_order(order_data)
            queue.add_order(order_id)
            
            # Send event
            await self.rabbitmq_client.send_event("oms.order_created", {
                "order_id": order_id,
                "status": "queued",
                "timestamp": datetime.now().isoformat()
            })
            
            # Broadcast to dashboard subscribers
            await self._broadcast_to_dashboard({
                "event": "order_received",
                "order": order_id,
                "status": "queued"
            })
            
            return {"success": True, "order_id": order_id, "status": "queued"}
            
        except Exception as e:
            logger.error(f"Error creating order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_list_orders(self, data: Dict) -> Dict:
        """Handle order listing requests"""
        try:
            status = data.get("status")
            limit = data.get("limit")
            offset = data.get("offset", 0)
            result = db.get_orders(status=status, limit=limit, offset=offset)
            return {"success": True, **result}
        except Exception as e:
            logger.error(f"Error listing orders: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_get_order(self, data: Dict) -> Dict:
        """Handle single order retrieval requests"""
        try:
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            order = db.get_order(order_id)
            if not order:
                return {"success": False, "error": f"Order {order_id} not found"}
            
            # Get detailed task information
            try:
                tasks = db.get_order_tasks(order_id)
                for task in tasks:
                    task['steps'] = db.get_task_steps(task['id'])
                order['tasks'] = tasks
            except Exception as e:
                logger.warning(f"Error fetching task details for order {order_id}: {e}")
                order['tasks'] = []
            
            return {"success": True, "order": order}
            
        except Exception as e:
            logger.error(f"Error getting order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_start_order(self, data: Dict) -> Dict:
        """Handle order start requests"""
        # CRITICAL DEBUG - This should always appear in logs
        print(f"OMS RECEIVED START_ORDER REQUEST FOR ORDER: {data.get('order_id')}")
        logger.error(f"OMS RECEIVED START_ORDER REQUEST FOR ORDER: {data.get('order_id')}")
        
        try:
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            # Concurrency guard: allow only one processing order at a time
            try:
                processing = db.get_orders(status=ORDER_STATUS['PROCESSING'])
            except Exception as e:
                processing = []
                logger.error(f"Error checking processing orders: {e}")
            if processing and any(o.get('status') == ORDER_STATUS['PROCESSING'] for o in processing):
                logger.warning(f"🔒 OMS (MQ) rejecting start_order for {order_id}: another order is already processing")
                return {"success": False, "error": "Another order is currently processing. Please wait."}

            order = db.get_order(order_id)
            if not order:
                return {"success": False, "error": f"Order {order_id} not found"}
            
            # Update status and remove from queue
            db.update_order_status(order_id, "processing")
            queue.remove(order_id)
            
            # Send order to scheduler
            await self._send_to_scheduler(order)
            
            # Send events
            await self.rabbitmq_client.send_event("oms.order_started", {
                "order_id": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            await self._broadcast_to_dashboard({
                "event": "order_started",
                "order": order_id
            })
            
            print(f"🔥🔥🔥 OMS SUCCESSFULLY PROCESSED START_ORDER FOR ORDER: {order_id} 🔥🔥🔥")
            logger.error(f"🔥🔥🔥 OMS SUCCESSFULLY PROCESSED START_ORDER FOR ORDER: {order_id} 🔥🔥🔥")
            
            return {"success": True, "message": "order_sent_to_scheduler", "order": order_id}
            
        except Exception as e:
            print(f"🔥🔥🔥 OMS ERROR PROCESSING START_ORDER: {e} 🔥🔥🔥")
            logger.error(f"🔥🔥🔥 OMS ERROR PROCESSING START_ORDER: {e} 🔥🔥🔥")
            logger.error(f"Error starting order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_update_order_status(self, data: Dict) -> Dict:
        """Handle order status update requests"""
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
            if status not in ORDER_STATUS.values():
                return {"success": False, "error": f"Invalid status. Must be one of: {list(ORDER_STATUS.values())}"}
            
            # Update status
            old_status = order.get("status")
            db.update_order_status(order_id, status, reason)
            
            # Log event
            db.log_event("order_status_changed", {
                "order_id": order_id,
                "old_status": old_status,
                "new_status": status,
                "reason": reason,
                "timestamp": datetime.now().isoformat()
            })
            
            # Send events
            await self.rabbitmq_client.send_event("oms.order_status_updated", {
                "order_id": order_id,
                "old_status": old_status,
                "new_status": status,
                "reason": reason,
                "timestamp": datetime.now().isoformat()
            })
            
            await self._broadcast_to_dashboard({
                "event": "order_status_updated",
                "order": order_id,
                "status": status,
                "reason": reason
            })
            
            return {"success": True, "message": "status_updated", "order": order_id, "status": status}
            
        except Exception as e:
            logger.error(f"Error updating order status: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_halt_order(self, data: Dict) -> Dict:
        """Handle order halt requests"""
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
                "timestamp": datetime.now().isoformat()
            })
            alert_id = db.create_alert(event_id, "order_halted", "warning")
            
            # Send events
            await self.rabbitmq_client.send_event("oms.order_halted", {
                "order_id": order_id,
                "reason": reason,
                "alert_id": alert_id,
                "timestamp": datetime.now().isoformat()
            })
            
            await self._broadcast_to_dashboard({
                "event": "order_halted",
                "order": order_id,
                "reason": reason,
                "alert_id": alert_id
            })
            
            return {"success": True, "message": "order_halted", "order": order_id, "alert_id": alert_id}
            
        except Exception as e:
            logger.error(f"Error halting order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_resume_order(self, data: Dict) -> Dict:
        """Handle order resume requests"""
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
            
            # Log event
            db.log_event("order_resumed", {
                "order_id": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            # Send events
            await self.rabbitmq_client.send_event("oms.order_resumed", {
                "order_id": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            await self._broadcast_to_dashboard({
                "event": "order_resumed",
                "order": order_id
            })
            
            return {"success": True, "message": "order_resumed", "order": order_id}
            
        except Exception as e:
            logger.error(f"Error resuming order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_complete_order(self, data: Dict) -> Dict:
        """Handle order completion requests"""
        try:
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            order = db.get_order(order_id)
            if not order:
                return {"success": False, "error": f"Order {order_id} not found"}
            
            # Update status to completed
            db.update_order_status(order_id, ORDER_STATUS['COMPLETED'])
            
            # Log event
            event_id = db.log_event("order_completed", {
                "order_id": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            # Send events
            await self.rabbitmq_client.send_event("oms.order_completed", {
                "order_id": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            await self._broadcast_to_dashboard({
                "event": "order_completed",
                "order": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            return {"success": True, "message": "order_completed", "order": order_id}
            
        except Exception as e:
            logger.error(f"Error completing order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_fail_order(self, data: Dict) -> Dict:
        """Handle order failure requests"""
        try:
            order_id = data.get("order_id")
            reason = data.get("reason", "Unknown error")
            
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            order = db.get_order(order_id)
            if not order:
                return {"success": False, "error": f"Order {order_id} not found"}
            
            # Update status to error
            db.update_order_status(order_id, ORDER_STATUS['ERROR'], reason)
            
            # Create alert
            event_id = db.log_event("order_failed", {
                "order_id": order_id,
                "reason": reason,
                "timestamp": datetime.now().isoformat()
            })
            alert_id = db.create_alert(event_id, "order_failed", "critical")
            
            # Send events
            await self.rabbitmq_client.send_event("oms.order_failed", {
                "order_id": order_id,
                "reason": reason,
                "alert_id": alert_id,
                "timestamp": datetime.now().isoformat()
            })
            
            await self._broadcast_to_dashboard({
                "event": "order_failed",
                "order": order_id,
                "reason": reason,
                "alert_id": alert_id
            })
            
            return {"success": True, "message": "order_failed", "order": order_id, "alert_id": alert_id}
            
        except Exception as e:
            logger.error(f"Error failing order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_delete_order(self, data: Dict) -> Dict:
        """Handle order deletion requests"""
        try:
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            order = db.get_order(order_id)
            if not order:
                return {"success": False, "error": f"Order {order_id} not found"}
            
            # Remove from queue if still queued
            if order.get("status") == ORDER_STATUS['QUEUED']:
                queue.remove(order_id)
            
            # Delete from database
            success = db.delete_order(order_id)
            if not success:
                return {"success": False, "error": "Failed to delete order from database"}
            
            # Log event
            event_id = db.log_event("order_deleted", {
                "order_id": order_id,
                "previous_status": order.get("status"),
                "timestamp": datetime.now().isoformat()
            })
            
            # Send events
            await self.rabbitmq_client.send_event("oms.order_deleted", {
                "order_id": order_id,
                "previous_status": order.get("status"),
                "timestamp": datetime.now().isoformat()
            })
            
            await self._broadcast_to_dashboard({
                "event": "order_deleted",
                "order": order_id,
                "previous_status": order.get("status")
            })
            
            return {"success": True, "message": "order_deleted", "order": order_id}
            
        except Exception as e:
            logger.error(f"Error deleting order: {e}")
            return {"success": False, "error": str(e)}
    
    # Queue Management Handlers
    async def handle_reorder_queue(self, data: Dict) -> Dict:
        """Handle queue reordering requests"""
        try:
            order_id = data.get("order_id")
            new_position = data.get("new_position")
            
            if order_id is None or new_position is None:
                return {"success": False, "error": "Missing order_id or new_position"}
            
            queue.reorder(order_id, new_position)
            
            await self._broadcast_to_dashboard({
                "event": "queue_reordered",
                "order": order_id,
                "new_position": new_position
            })
            
            return {"success": True, "message": "reordered", "order": order_id, "position": new_position}
            
        except Exception as e:
            logger.error(f"Error reordering queue: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_bulk_reorder_queue(self, data: Dict) -> Dict:
        """Handle bulk queue reordering requests"""
        try:
            order_ids = data.get("order_ids", [])
            if not order_ids:
                return {"success": False, "error": "order_ids list is required"}
            
            queue.bulk_reorder(order_ids)
            
            await self._broadcast_to_dashboard({
                "event": "queue_bulk_reordered",
                "order_ids": order_ids
            })
            
            return {"success": True, "message": "queue_reordered", "order_count": len(order_ids)}
            
        except Exception as e:
            logger.error(f"Error bulk reordering queue: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_sync_queue(self, data: Dict) -> Dict:
        """Handle queue sync requests"""
        try:
            success = queue.sync_with_database()
            if success:
                current_queue = queue.get_queue()
                return {
                    "success": True,
                    "status": "success",
                    "message": "Queue synced with database",
                    "queue_length": len(current_queue),
                    "queue": current_queue
                }
            else:
                return {"success": False, "message": "Failed to sync queue with database"}
                
        except Exception as e:
            logger.error(f"Error syncing queue: {e}")
            return {"success": False, "error": str(e)}
    
    # System Management Handlers
    async def handle_stop_system(self, data: Dict) -> Dict:
        """Handle system stop requests"""
        try:
            reason = data.get("reason", "Manual stop")
            
            # Send system stop event
            await self.rabbitmq_client.send_event("system.stop", {
                "reason": reason,
                "timestamp": datetime.now().isoformat()
            })
            
            return {"success": True, "message": "System stop initiated", "reason": reason}
            
        except Exception as e:
            logger.error(f"Error stopping system: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_resume_system(self, data: Dict) -> Dict:
        """Handle system resume requests"""
        try:
            # Send system resume event
            await self.rabbitmq_client.send_event("system.resume", {
                "timestamp": datetime.now().isoformat()
            })
            
            return {"success": True, "message": "System resume initiated"}
            
        except Exception as e:
            logger.error(f"Error resuming system: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_get_system_status(self, data: Dict) -> Dict:
        """Handle system status requests"""
        try:
            # Get current system status
            queue_status = queue.get_queue()
            
            return {
                "success": True,
                "status": "operational",
                "queue_length": len(queue_status),
                "timestamp": datetime.now().isoformat()
            }
            
        except Exception as e:
            logger.error(f"Error getting system status: {e}")
            return {"success": False, "error": str(e)}
    
    # Inventory Management Handlers
    async def handle_threshold_warning(self, data: Dict) -> Dict:
        """Handle inventory threshold warnings"""
        try:
            ingredient = data.get("ingredient")
            severity = data.get("severity", "medium")
            
            if not ingredient:
                return {"success": False, "error": "Missing ingredient"}
            
            # Log the warning
            event_id = db.log_event("inventory_threshold_warning", {
                "ingredient": ingredient,
                "severity": severity,
                "timestamp": datetime.now().isoformat()
            })
            
            # Create alert
            alert_id = db.create_alert(event_id, "inventory_low", severity)
            
            # Broadcast to dashboard
            await self._broadcast_to_dashboard({
                "event": "inventory_warning",
                "ingredient": ingredient,
                "severity": severity,
                "alert_id": alert_id
            })
            
            return {"success": True, "message": "Warning logged", "alert_id": alert_id}
            
        except Exception as e:
            logger.error(f"Error handling threshold warning: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_refill_inventory(self, data: Dict) -> Dict:
        """Handle inventory refill requests"""
        try:
            ingredient = data.get("ingredient")
            if not ingredient:
                return {"success": False, "error": "Missing ingredient"}
            
            # Send refill request to validation service
            response = await self.rabbitmq_client.send_request(
                target_service="validation",
                action="inventory_refill",
                data={"ingredient": ingredient, "amount": 100},
                timeout=10
            )
            
            if response.get("success"):
                await self._broadcast_to_dashboard({
                    "event": "inventory_refilled",
                    "ingredient": ingredient
                })
                
                return {"success": True, "message": f"Refill initiated for {ingredient}"}
            else:
                return {"success": False, "error": response.get("error", "Refill failed")}
                
        except Exception as e:
            logger.error(f"Error refilling inventory: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_get_inventory_status(self, data: Dict) -> Dict:
        """Handle inventory status requests"""
        try:
            # Get inventory status from validation service
            response = await self.rabbitmq_client.send_request(
                target_service="validation",
                action="inventory_status",
                data={},
                timeout=10
            )
            
            if response.get("success"):
                return response
            else:
                return {"success": False, "error": response.get("error", "Failed to get inventory status")}
                
        except Exception as e:
            logger.error(f"Error getting inventory status: {e}")
            return {"success": False, "error": str(e)}
    
    # Alert Management Handlers
    async def handle_get_active_alerts(self, data: Dict) -> Dict:
        """Handle get active alerts requests"""
        try:
            alerts = db.get_active_alerts()
            return {"success": True, "alerts": alerts}
        except Exception as e:
            logger.error(f"Error getting active alerts: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_get_acknowledged_alerts(self, data: Dict) -> Dict:
        """Handle get acknowledged alerts requests"""
        try:
            alerts = db.get_acknowledged_alerts()
            return {"success": True, "alerts": alerts}
        except Exception as e:
            logger.error(f"Error getting acknowledged alerts: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_acknowledge_alert(self, data: Dict) -> Dict:
        """Handle alert acknowledgment requests"""
        try:
            alert_id = data.get("alert_id")
            if not alert_id:
                return {"success": False, "error": "Missing alert_id"}
            
            db.acknowledge_alert(alert_id)
            
            # Log event
            db.log_event("alert_acknowledged", {
                "alert_id": alert_id,
                "timestamp": datetime.now().isoformat()
            })
            
            # Broadcast to dashboard
            await self._broadcast_to_dashboard({
                "event": "alert_acknowledged",
                "alert_id": alert_id
            })
            
            return {"success": True, "message": "alert_acknowledged", "alert_id": alert_id}
        except Exception as e:
            logger.error(f"Error acknowledging alert: {e}")
            return {"success": False, "error": str(e)}

    # Dashboard and Utility Handlers
    async def handle_subscribe_dashboard(self, data: Dict) -> Dict:
        """Handle dashboard subscription requests"""
        try:
            dashboard_id = data.get("dashboard_id", "default")
            if dashboard_id not in self.dashboard_subscribers:
                self.dashboard_subscribers.append(dashboard_id)
                logger.info(f"Dashboard {dashboard_id} subscribed to updates")
            
            return {"success": True, "subscribed": True}
            
        except Exception as e:
            logger.error(f"Error handling dashboard subscription: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_health(self, data: Dict) -> Dict:
        """Handle health check requests"""
        return {
            "status": "healthy",
            "service": "oms",
            "timestamp": datetime.now().isoformat(),
            "dashboard_subscribers": len(self.dashboard_subscribers)
        }
    
    # Event Handlers
    async def handle_order_completed_event(self, data: Dict):
        """Handle order completion events from scheduler"""
        order_id = data.get("order_id")
        if order_id:
            await self.handle_complete_order({"order_id": order_id})
    
    async def handle_order_failed_event(self, data: Dict):
        """Handle order failure events from scheduler"""
        order_id = data.get("order_id")
        error = data.get("error", "Unknown error")
        if order_id:
            await self.handle_fail_order({"order_id": order_id, "reason": error})
    
    async def handle_order_heartbeat_event(self, data: Dict):
        """Handle order heartbeat events from scheduler"""
        order_id = data.get("order_id")
        status = data.get("status", "processing")
        progress = data.get("progress", {})
        
        if order_id:
            # Log heartbeat for monitoring
            logger.info(f"💓 [OMS] Received heartbeat for order {order_id}: {status}")
            if progress:
                completion_pct = progress.get("completion_percentage", 0)
                logger.info(f"📊 [OMS] Order {order_id} progress: {completion_pct:.1f}% complete")
            
            # Broadcast heartbeat to dashboard subscribers
            await self._broadcast_to_dashboard({
                "event": "order_heartbeat",
                "order": order_id,
                "status": status,
                "progress": progress,
                "timestamp": datetime.now().isoformat()
            })
    
    async def handle_threshold_warning_event(self, data: Dict):
        """Handle threshold warning events from validation service"""
        ingredient = data.get("ingredient")
        threshold = data.get("threshold", "medium")
        if ingredient:
            await self.handle_threshold_warning({"ingredient": ingredient, "severity": threshold})
    
    async def handle_shutdown_event(self, data: Dict):
        """Handle system shutdown events"""
        logger.info("Received shutdown event, stopping OMS service...")
        await self.stop()
    
    # Helper Methods
    async def _send_to_scheduler(self, order_data: Dict):
        """Send order to scheduler for processing"""
        try:
            response = await self.rabbitmq_client.send_request(
                target_service="scheduler",
                action="process_order",
                data=order_data,
                timeout=180
            )
            
            if not response.get("success"):
                logger.error(f"Failed to send order to scheduler: {response.get('error')}")
                
        except Exception as e:
            logger.error(f"Error sending order to scheduler: {e}")
    
    async def _broadcast_to_dashboard(self, message: Dict):
        """Broadcast message to dashboard subscribers"""
        try:
            await self.rabbitmq_client.send_event("oms.dashboard_update", {
                "message": message,
                "timestamp": datetime.now().isoformat()
            })
            
        except Exception as e:
            logger.error(f"Error broadcasting to dashboard: {e}")

# Main execution
async def main():
    service = OMSService()
    await service.start()

if __name__ == "__main__":
    asyncio.run(main()) 