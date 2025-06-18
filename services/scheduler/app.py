"""BARNS Scheduler Service

Orchestrates order processing by breaking down drink orders into tasks 
and coordinating execution across robotic arms.
"""

import asyncio
import logging
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.rabbitmq_client import RabbitMQClient, EventListener
from . import scheduler

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load drink recipes
recipes = {}

class SchedulerService:
    """Scheduler service for BARNS order orchestration."""
    
    def __init__(self):
        self.rabbitmq_client = RabbitMQClient("scheduler")
        self.event_listener = EventListener("scheduler")
        self.status_subscribers = []
        
    async def start(self):
        """Start the scheduler service."""
        await self.rabbitmq_client.connect()
        await self.event_listener.connect()
        
        # Load recipes
        await self._load_recipes()
        
        # Register message handlers
        self.rabbitmq_client.register_handler("process_order", self.handle_process_order)
        self.rabbitmq_client.register_handler("feedback", self.handle_feedback)
        self.rabbitmq_client.register_handler("get_status", self.handle_get_status)
        self.rabbitmq_client.register_handler("subscribe_status", self.handle_subscribe_status)
        self.rabbitmq_client.register_handler("health", self.handle_health)
        
        # Subscribe to events
        await self.event_listener.subscribe_to_events(["routine.*", "system.*", "oms.*"])
        self.event_listener.register_event_handler("routine.task_completed", self.handle_task_completed_event)
        self.event_listener.register_event_handler("routine.task_failed", self.handle_task_failed_event)
        self.event_listener.register_event_handler("system.shutdown", self.handle_shutdown_event)
        
        # Register status callback with scheduler module
        scheduler.register_status_callback(self.notify_status)
        
        logger.info("Scheduler service started and listening for messages")
        
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down scheduler service...")
        finally:
            await self.stop()
    
    async def stop(self):
        """Stop the scheduler service."""
        await self.rabbitmq_client.disconnect()
        await self.event_listener.disconnect()
        logger.info("Scheduler service stopped")
    
    async def _load_recipes(self):
        """Load drink recipes from file."""
        global recipes
        try:
            recipe_file = Path(__file__).parent / "data" / "recipes.json"
            recipes = scheduler.load_recipes(str(recipe_file))
            logger.info(f"Loaded {len(recipes)} recipes: {list(recipes.keys())}")
                
        except Exception as e:
            logger.error(f"Failed to load recipes: {e}")
            recipes = {}
    
    async def handle_process_order(self, data: Dict) -> Dict:
        """Handle order processing requests from OMS."""
        try:
            order_id = data.get("id")
            drinks = data.get("cups", [])
            
            if not order_id:
                return {"success": False, "error": "Missing order ID"}
            
            # Send acknowledgment event
            await self.rabbitmq_client.send_event("scheduler.order_received", {
                "order_id": order_id,
                "drink_count": len(drinks),
                "timestamp": datetime.now().isoformat()
            })
            
            # Start processing order asynchronously
            asyncio.create_task(self._process_order_async(order_id, drinks))
            
            return {
                "success": True,
                "message": "Order accepted by scheduler",
                "order_id": order_id
            }
            
        except Exception as e:
            logger.error(f"Error processing order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_feedback(self, data: Dict) -> Dict:
        """Handle feedback from routine service about task completion."""
        try:
            cup_id = data.get("cup_id")
            action = data.get("action")
            success = data.get("success")
            message = data.get("message", "")
            
            # Process the feedback through the scheduler
            await scheduler.handle_routine_feedback(cup_id, action, success)
            
            # Notify status subscribers
            status_message = f"{action} for cup {cup_id} {'completed' if success else 'failed'}"
            if message:
                status_message += f": {message}"
            
            await self.notify_status(status_message)
            
            # Send feedback event
            await self.rabbitmq_client.send_event("scheduler.feedback_processed", {
                "cup_id": cup_id,
                "action": action,
                "success": success,
                "message": message,
                "timestamp": datetime.now().isoformat()
            })
            
            return {"success": True, "status": "received"}
            
        except Exception as e:
            logger.error(f"Error handling feedback: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_get_status(self, data: Dict) -> Dict:
        """Handle status requests."""
        try:
            status = scheduler.get_current_status()
            return {
                "success": True,
                "status": status,
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Error getting status: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_subscribe_status(self, data: Dict) -> Dict:
        """Handle status subscription requests."""
        try:
            service_name = data.get("service_name")
            if service_name and service_name not in self.status_subscribers:
                self.status_subscribers.append(service_name)
                logger.info(f"Service {service_name} subscribed to status updates")
            
            return {"success": True, "subscribed": True}
            
        except Exception as e:
            logger.error(f"Error handling status subscription: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_health(self, data: Dict) -> Dict:
        """Handle health check requests."""
        return {
            "status": "healthy",
            "service": "scheduler",
            "timestamp": datetime.now().isoformat(),
            "loaded_recipes": len(recipes),
            "status_subscribers": len(self.status_subscribers)
        }
    
    async def handle_task_completed_event(self, data: Dict):
        """Handle task completion events from routine service."""
        cup_id = data.get("cup_id")
        function = data.get("function")
        
        if cup_id and function:
            await scheduler.handle_routine_feedback(cup_id, function, True)
            await self.notify_status(f"Task {function} completed for cup {cup_id}")
    
    async def handle_task_failed_event(self, data: Dict):
        """Handle task failure events from routine service."""
        cup_id = data.get("cup_id")
        function = data.get("function")
        error = data.get("error", "Unknown error")
        
        if cup_id and function:
            await scheduler.handle_routine_feedback(cup_id, function, False)
            await self.notify_status(f"Task {function} failed for cup {cup_id}: {error}")
    
    async def handle_shutdown_event(self, data: Dict):
        """Handle system shutdown events."""
        logger.info("Received shutdown event, stopping scheduler service...")
        await self.stop()
    
    async def _process_order_async(self, order_id: int, drinks: List[Dict]):
        """Background coroutine to process each drink using the scheduler."""
        logger.info(f"Processing order {order_id} with {len(drinks)} drinks")
        
        try:
            # Send processing started event
            await self.rabbitmq_client.send_event("scheduler.order_processing_started", {
                "order_id": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            # Use the scheduler's process_order_async function
            success = await scheduler.process_order_async(order_id, drinks, recipes)
            
            # Send completion event
            if success:
                await self.rabbitmq_client.send_event("scheduler.order_completed", {
                    "order_id": order_id,
                    "timestamp": datetime.now().isoformat()
                })
                await self.notify_status(f"Order {order_id} completed successfully")
            else:
                await self.rabbitmq_client.send_event("scheduler.order_failed", {
                    "order_id": order_id,
                    "timestamp": datetime.now().isoformat()
                })
                await self.notify_status(f"Failed to process order {order_id}")
                
        except Exception as e:
            logger.error(f"Exception in order processing for order {order_id}: {e}")
            
            # Send error event
            await self.rabbitmq_client.send_event("scheduler.order_error", {
                "order_id": order_id,
                "error": str(e),
                "timestamp": datetime.now().isoformat()
            })
            
            scheduler.current_status.update({"status": "error", "step": str(e)})
            await self.notify_status(f"Error in order {order_id}: {e}")
    
    async def notify_status(self, message: str):
        """Send status updates to subscribed services."""
        try:
            # Send status event to all interested services
            await self.rabbitmq_client.send_event("scheduler.status_update", {
                "message": message,
                "status": scheduler.get_current_status(),
                "timestamp": datetime.now().isoformat()
            })
            
            logger.debug(f"Status update sent: {message}")
            
        except Exception as e:
            logger.error(f"Error sending status update: {e}")

async def main():
    """Main service entry point."""
    service = SchedulerService()
    await service.start()

if __name__ == "__main__":
    asyncio.run(main()) 