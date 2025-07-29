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

# Global service instance for access by scheduler module
_scheduler_service_instance = None

class SchedulerService:
    """Scheduler service for BARNS order orchestration."""
    
    def __init__(self):
        global _scheduler_service_instance
        self.rabbitmq_client = RabbitMQClient("scheduler")
        self.event_listener = EventListener("scheduler")
        self.status_subscribers = []
        _scheduler_service_instance = self  # Store reference for scheduler module
        
    async def start(self):
        """Start the scheduler service with automatic reconnection."""
        while True:
            try:
                await self._start_service()
                # If we get here, the service was interrupted
                break
            except KeyboardInterrupt:
                logger.info("Shutting down scheduler service...")
                await self.stop()
                break
            except Exception as e:
                logger.error(f"Service error: {e}")
                logger.info("Restarting service in 10 seconds...")
                await self._cleanup()
                await asyncio.sleep(10)

    async def _start_service(self):
        """Internal method to start the service components."""
        logger.info("🚀 [SCHEDULER] Starting scheduler service...")
        
        # Load recipes first
        logger.info("📖 [SCHEDULER] Loading recipes...")
        await self._load_recipes()
        
        # Register message handlers BEFORE connecting to prevent race conditions
        logger.info("🔧 [SCHEDULER] Registering RabbitMQ message handlers...")
        self.rabbitmq_client.register_handler("process_order", self.handle_process_order)
        self.rabbitmq_client.register_handler("feedback", self.handle_feedback)
        self.rabbitmq_client.register_handler("get_status", self.handle_get_status)
        self.rabbitmq_client.register_handler("subscribe_status", self.handle_subscribe_status)
        self.rabbitmq_client.register_handler("health", self.handle_health)
        logger.info("✅ [SCHEDULER] All RabbitMQ message handlers registered successfully")
        
        # Register event handlers for the event listener
        self.event_listener.register_event_handler("routine.task_completed", self.handle_task_completed_event)
        self.event_listener.register_event_handler("routine.task_failed", self.handle_task_failed_event)
        self.event_listener.register_event_handler("system.shutdown", self.handle_shutdown_event)
        
        # NOW connect to RabbitMQ (this starts consuming messages)
        # Retry connection logic for RabbitMQ
        while True:
            try:
                logger.info("🔌 [SCHEDULER] Attempting to connect to RabbitMQ...")
                await self.rabbitmq_client.connect()
                await self.event_listener.connect()
                logger.info("✅ [SCHEDULER] RabbitMQ connections established")
                break
            except Exception as e:
                logger.error(f"❌ [SCHEDULER] Failed to connect to RabbitMQ: {e}")
                logger.info("Retrying connection in 10 seconds...")
                await asyncio.sleep(10)
        
        # Subscribe to events AFTER connecting
        await self.event_listener.subscribe_to_events(["routine.*", "system.*", "oms.*"])
        
        # Register status callback with scheduler module
        logger.info("🔗 [SCHEDULER] Registering status callback with scheduler module...")
        scheduler.register_status_callback(self.notify_status)
        
        logger.info("Scheduler service ready and listening for messages")
        
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down scheduler service...")
            raise

    async def _cleanup(self):
        """Clean up connections and tasks."""
        try:
            # Disconnect from RabbitMQ
            try:
                await self.rabbitmq_client.disconnect()
            except:
                pass
            
            try:
                await self.event_listener.disconnect()
            except:
                pass
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

    async def stop(self):
        """Stop the scheduler service."""
        await self._cleanup()
        logger.info("Scheduler service stopped")
    
    async def _load_recipes(self):
        """Load drink recipes from file."""
        global recipes
        try:
            recipe_file = Path("/app/data/recipes.json")
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
            
            logger.info(f"[SCHEDULER] Processing order {order_id} with {len(drinks)} drinks")
            
            if not order_id:
                logger.error(f"[SCHEDULER] Missing order ID in request")
                return {"success": False, "error": "Missing order ID"}
            
            # Send acknowledgment event
            try:
                await self.rabbitmq_client.send_event("scheduler.order_received", {
                    "order_id": order_id,
                    "drink_count": len(drinks),
                    "timestamp": datetime.now().isoformat()
                })
            except ConnectionError as e:
                logger.error(f"Connection error sending order received event: {e}")
                # Still continue processing but note the connection issue
            
            # Start processing order asynchronously
            asyncio.create_task(self._process_order_async(order_id, drinks))
            
            response = {
                "success": True,
                "message": "Order accepted by scheduler",
                "order_id": order_id
            }
            
            logger.info(f"[SCHEDULER] Order {order_id} accepted for processing")
            return response
            
        except ConnectionError as e:
            logger.error(f"Connection error in handle_process_order: {e}")
            raise  # This will trigger service restart
        except Exception as e:
            logger.error(f"[SCHEDULER] Error processing order: {e}")
            return {"success": False, "error": str(e)}
    
    async def handle_feedback(self, data: Dict) -> Dict:
        """Handle feedback from routine service about task completion."""
        try:
            cup_id = data.get("cup_id")
            action = data.get("action")
            success = data.get("success")
            message = data.get("message", "")
            
            logger.info(f"[SCHEDULER] Feedback: {action} for {cup_id} - {'SUCCESS' if success else 'FAILED'}")
            
            # Process the feedback through the scheduler
            try:
                await scheduler.handle_routine_feedback(cup_id, action, success)
                logger.info(f"[SCHEDULER] Feedback processed for {action} on {cup_id}")
            except Exception as feedback_error:
                logger.error(f"[SCHEDULER] Error processing feedback: {feedback_error}")
                # Still continue to send response
            
            # Notify status subscribers
            status_message = f"{action} for {cup_id} {'completed' if success else 'failed'}"
            if message:
                status_message += f": {message}"
            
            try:
                await self.notify_status(status_message)
            except Exception as status_error:
                logger.error(f"Error notifying status subscribers: {status_error}")
            
            # Send feedback event
            try:
                await self.rabbitmq_client.send_event("scheduler.feedback_processed", {
                    "cup_id": cup_id,
                    "action": action,
                    "success": success,
                    "message": message,
                    "timestamp": datetime.now().isoformat()
                })
            except ConnectionError as e:
                logger.error(f"Connection error sending feedback processed event: {e}")
            except Exception as event_error:
                logger.error(f"Error sending feedback processed event: {event_error}")
            
            return {"success": True, "status": "received"}
            
        except ConnectionError as e:
            logger.error(f"Connection error in handle_feedback: {e}")
            raise  # This will trigger service restart
        except Exception as e:
            logger.error(f"[SCHEDULER] Feedback handler error: {e}")
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
        logger.info(f"🔄 [SCHEDULER] Background processing started for order {order_id} with {len(drinks)} drinks")
        print(f"🔥🔥🔥 SCHEDULER ASYNC PROCESSING STARTED FOR ORDER {order_id} 🔥🔥🔥")
        
        try:
            # Send processing started event
            await self.rabbitmq_client.send_event("scheduler.order_processing_started", {
                "order_id": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            # Use the scheduler's process_order_async function
            # Note: The scheduler module handles sending completion/failure events to OMS
            success = await scheduler.process_order_async(order_id, drinks, recipes)
            
            # Send status notifications (events are already sent by scheduler module)
            if success:
                logger.info(f"✅ [SCHEDULER] Order {order_id} completed successfully")
                await self.notify_status(f"Order {order_id} completed successfully")
            else:
                logger.error(f"❌ [SCHEDULER] Failed to process order {order_id}")
                await self.notify_status(f"Failed to process order {order_id}")
                
        except Exception as e:
            logger.error(f"🔥🔥🔥 SCHEDULER ASYNC PROCESSING ERROR FOR ORDER {order_id}: {e} 🔥🔥🔥")
            print(f"🔥🔥🔥 SCHEDULER ASYNC PROCESSING ERROR FOR ORDER {order_id}: {e} 🔥🔥🔥")
            
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
    try:
        await service.start()
    except KeyboardInterrupt:
        logger.info("Received interrupt signal, shutting down...")
        await service.stop()

if __name__ == "__main__":
    asyncio.run(main()) 