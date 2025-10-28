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
from shared.logger import log
from . import scheduler

# Configure logging (keep for any remaining stdlib logs)
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
                log("INFO", "Shutting down", service="scheduler")
                await self.stop()
                break
            except Exception as e:
                log("ERROR", "Service error", service="scheduler", error=str(e))
                log("DEBUG", "Restarting in 10s", service="scheduler")
                await self._cleanup()
                await asyncio.sleep(10)

    async def _start_service(self):
        """Internal method to start the service components."""
        log("INFO", "Scheduler starting", service="scheduler")
        
        # Load recipes first
        log("DEBUG", "Loading recipes", service="scheduler")
        await self._load_recipes()
        
        # Register message handlers BEFORE connecting to prevent race conditions
        log("DEBUG", "Registering handlers", service="scheduler")
        self.rabbitmq_client.register_handler("process_order", self.handle_process_order)
        self.rabbitmq_client.register_handler("feedback", self.handle_feedback)
        self.rabbitmq_client.register_handler("update_cup_position", self.handle_update_cup_position)
        self.rabbitmq_client.register_handler("get_status", self.handle_get_status)
        self.rabbitmq_client.register_handler("subscribe_status", self.handle_subscribe_status)
        self.rabbitmq_client.register_handler("health", self.handle_health)
        self.rabbitmq_client.register_handler("cancel_order", self.handle_cancel_order)
        self.rabbitmq_client.register_handler("stop_order", self.handle_stop_order)
        self.rabbitmq_client.register_handler("resume_order", self.handle_resume_order)
        log("DEBUG", "Handlers registered", service="scheduler")
        
        # Register event handlers for the event listener
        self.event_listener.register_event_handler("routine.task_completed", self.handle_task_completed_event)
        self.event_listener.register_event_handler("routine.task_failed", self.handle_task_failed_event)
        self.event_listener.register_event_handler("system.shutdown", self.handle_shutdown_event)
        
        # NOW connect to RabbitMQ (this starts consuming messages)
        # Retry connection logic for RabbitMQ
        while True:
            try:
                log("DEBUG", "Connecting to RabbitMQ", service="scheduler")
                await self.rabbitmq_client.connect()
                await self.event_listener.connect()
                log("INFO", "RabbitMQ connected", service="scheduler")
                
                # Set the global RabbitMQ client in the scheduler module
                log("DEBUG", "Setting RabbitMQ client", service="scheduler")
                scheduler.set_rabbitmq_client(self.rabbitmq_client)
                
                break
            except Exception as e:
                log("ERROR", "RabbitMQ connect failed", service="scheduler", error=str(e))
                log("DEBUG", "Retrying in 10s", service="scheduler")
                await asyncio.sleep(10)
        
        # Subscribe to events AFTER connecting
        await self.event_listener.subscribe_to_events(["routine.*", "system.*", "oms.*"])
        
        # Register status callback with scheduler module
        log("DEBUG", "Registering status callback", service="scheduler")
        scheduler.register_status_callback(self.notify_status)
        
        log("INFO", "Scheduler ready", service="scheduler")
        
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            log("INFO", "Shutting down", service="scheduler")
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
            log("ERROR", "Cleanup failed", service="scheduler", error=str(e))

    async def stop(self):
        """Stop the scheduler service."""
        await self._cleanup()
        log("INFO", "Scheduler stopped", service="scheduler")
    
    async def _load_recipes(self):
        """Load drink recipes from file."""
        global recipes
        try:
            recipe_file = Path("/app/data/recipes.json")
            recipes = scheduler.load_recipes(str(recipe_file))
            log("INFO", "Recipes loaded", service="scheduler", count=len(recipes))
                
        except Exception as e:
            log("ERROR", "Recipe load failed", service="scheduler", error=str(e))
            recipes = {}
    
    async def handle_process_order(self, data: Dict) -> Dict:
        """Handle order processing requests from OMS."""
        try:
            order_id = data.get("id")
            drinks = data.get("cups", [])
            
            log("INFO", f"Order {order_id} received with {len(drinks)} drinks", service="scheduler")
            
            if not order_id:
                log("ERROR", "Missing order ID", service="scheduler")
                return {"success": False, "error": "Missing order ID"}
            
            # Send acknowledgment event
            try:
                await self.rabbitmq_client.send_event("scheduler.order_received", {
                    "order_id": order_id,
                    "drink_count": len(drinks),
                    "timestamp": datetime.now().isoformat()
                })
            except ConnectionError as e:
                log("ERROR", "Order received event send failed", service="scheduler", error="CONNECTION")
                # Still continue processing but note the connection issue
            
            # Start processing order asynchronously
            asyncio.create_task(self._process_order_async(order_id, drinks))
            
            response = {
                "success": True,
                "message": "Order accepted by scheduler",
                "order_id": order_id
            }
            
            log("INFO", f"Order {order_id} accepted", service="scheduler")
            return response
            
        except ConnectionError as e:
            log("ERROR", f"RabbitMQ connection lost in process_order for order {order_id}", service="scheduler")
            raise  # This will trigger service restart
        except Exception as e:
            log("ERROR", f"Order {order_id} processing exception: {str(e)[:100]}", service="scheduler")
            return {"success": False, "error": str(e)}
    
    async def handle_feedback(self, data: Dict) -> Dict:
        """Handle feedback from routine service about task completion."""
        try:
            cup_id = data.get("cup_id")
            action = data.get("action")
            success = data.get("success")
            message = data.get("message", "")
            
            log("DEBUG", f"Feedback received: {action} for cup {cup_id} - {'success' if success else 'failed'}", service="scheduler")
            
            # Process the feedback through the scheduler
            try:
                await scheduler.handle_routine_feedback(cup_id, action, success)
                log("DEBUG", f"Feedback processed: {action} for cup {cup_id}", service="scheduler")
            except Exception as feedback_error:
                log("ERROR", "Feedback processing failed", service="scheduler", error=str(feedback_error))
                # Still continue to send response
            
            # Notify status subscribers
            status_message = f"{action} for {cup_id} {'completed' if success else 'failed'}"
            if message:
                status_message += f": {message}"
            
            try:
                await self.notify_status(status_message)
            except Exception as status_error:
                log("ERROR", "Status notify failed", service="scheduler", error=str(status_error))
            
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
                log("ERROR", "Feedback processed event send failed", service="scheduler", error="CONNECTION")
            except Exception as event_error:
                log("ERROR", "Feedback processed event error", service="scheduler", error=str(event_error)[:100])
            
            return {"success": True, "status": "received"}
            
        except ConnectionError as e:
            log("ERROR", f"RabbitMQ connection lost in handle_feedback for cup {cup_id}", service="scheduler")
            raise  # This will trigger service restart
        except Exception as e:
            log("ERROR", f"Feedback handler exception for cup {cup_id}: {str(e)[:100]}", service="scheduler")
            return {"success": False, "error": str(e)}
    
    async def handle_update_cup_position(self, data: Dict) -> Dict:
        """Handle cup position update from routine service (after cup_detection)."""
        try:
            cup_id = data.get("cup_id")
            new_position = data.get("new_position")
            old_position = data.get("old_position")
            
            log("DEBUG", f"Position update received for cup {cup_id}: {new_position}", service="scheduler")
            
            if not cup_id or new_position is None:
                log("ERROR", "Cup position update missing cup_id or position", service="scheduler")
                return {"success": False, "error": "Missing required parameters"}
            
            # Update position in scheduler's internal data structures
            updated = await scheduler.update_cup_position(cup_id, float(new_position))
            
            if updated:
                log("INFO", f"Position updated for cup {cup_id} to {new_position}", service="scheduler")
                log("DEBUG", f"All future tasks for cup {cup_id} will use position {new_position}", service="scheduler")
                
                # Send event to notify about position update
                try:
                    await self.rabbitmq_client.send_event("scheduler.cup_position_updated", {
                        "cup_id": cup_id,
                        "new_position": new_position,
                        "old_position": old_position,
                        "timestamp": datetime.now().isoformat()
                    })
                except Exception as e:
                    log("ERROR", "Cup position updated event send failed", service="scheduler", error=str(e)[:50])
                
                return {"success": True, "message": f"Position updated to {new_position}"}
            else:
                log("ERROR", f"Cup {cup_id} not found in active orders", service="scheduler")
                return {"success": False, "error": f"Cup {cup_id} not found in active orders"}
            
        except Exception as e:
            log("ERROR", f"Cup position update exception for cup {cup_id}: {str(e)[:100]}", service="scheduler")
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
            log("ERROR", "Status get failed", service="scheduler", error=str(e))
            return {"success": False, "error": str(e)}
    
    async def handle_subscribe_status(self, data: Dict) -> Dict:
        """Handle status subscription requests."""
        try:
            service_name = data.get("service_name")
            if service_name and service_name not in self.status_subscribers:
                self.status_subscribers.append(service_name)
                log("DEBUG", "Status subscription", service="scheduler")
            
            return {"success": True, "subscribed": True}
            
        except Exception as e:
            log("ERROR", "Subscription failed", service="scheduler", error=str(e))
            return {"success": False, "error": str(e)}
    
    async def handle_health(self, data: Dict) -> Dict:
        """Handle health check requests"""
        try:
            # Get RabbitMQ client health status
            rabbitmq_health = {}
            if self.rabbitmq_client:
                rabbitmq_health = self.rabbitmq_client.get_health_status()
            
            return {
                "status": "healthy",
                "service": "scheduler",
                "timestamp": datetime.now().isoformat(),
                "rabbitmq_health": rabbitmq_health,
                "event_listener_connected": self.event_listener is not None,
                "recipes_loaded": len(recipes) if recipes else 0
            }
        except Exception as e:
            log("ERROR", "Health check failed", service="scheduler", error=str(e))
            return {"success": False, "error": str(e)}
    
    async def handle_cancel_order(self, data: Dict) -> Dict:
        """Cancel current order processing: mark remaining tasks cancelled and instruct routine to drop queued items."""
        try:
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            # Mark remaining tasks as cancelled in scheduler state
            from . import scheduler as core
            cancelled = 0
            with core.lock:
                if core.current_status.get("order_id") != order_id:
                    # If different order is running, nothing to do
                    pass
                for task in core.tasks:
                    if task["status"] not in ["done", "failed", "cancelled"]:
                        task["status"] = "cancelled"
                        cancelled += 1
                # Allow workers to exit naturally based on status counts
            
            # Build cup_ids to cancel in routine queues
            cup_ids = []
            with core.lock:
                cup_ids = list(core.tasks_by_cup.keys())
            
            # Tell routine to drop any queued tasks for these cups
            try:
                resp = await self.rabbitmq_client.send_request(
                    target_service="routine",
                    action="cancel_order",
                    data={"order_id": order_id, "cup_ids": cup_ids},
                    timeout=10
                )
                log("DEBUG", "Cancel response", service="scheduler")
            except Exception as e:
                log("ERROR", f"Routine cancel request failed for order {order_id}: {str(e)[:100]}", service="scheduler")
            
            # Stop heartbeat by changing current_status order_id
            with core.lock:
                core.current_status.update({"status": "cancelled", "step": None})
            
            # Notify OMS with failure/cancel info
            try:
                await core.notify_oms_completion(order_id, False, "Order cancelled by user", self.rabbitmq_client)
            except Exception as e:
                log("ERROR", f"OMS cancellation notification failed for order {order_id}: {str(e)[:100]}", service="scheduler")
            
            return {"success": True, "cancelled_tasks": cancelled}
        except Exception as e:
            log("ERROR", f"Cancel order handler exception for order {order_id}: {str(e)[:100]}", service="scheduler")
            return {"success": False, "error": str(e)}
    
    async def handle_stop_order(self, data: Dict) -> Dict:
        """Handle stop order request - signals workers to halt processing gracefully."""
        try:
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            from . import scheduler as core
            
            with core.lock:
                if core.current_status.get("order_id") != order_id:
                    return {"success": False, "error": f"Order {order_id} is not currently processing"}
                
                # Set stop flag to signal workers (they'll finish current task then stop)
                core.order_stopped = True
                core.current_status["status"] = "stopping"
                log("INFO", f"Order {order_id} stopping signal sent", service="scheduler")
            
            # Send stopping status to dashboard
            try:
                await self.rabbitmq_client.send_event("scheduler.order_stopping", {
                    "order_id": order_id,
                    "timestamp": datetime.now().isoformat()
                })
            except Exception as e:
                log("ERROR", f"Order {order_id} stopping event send failed: {str(e)[:50]}", service="scheduler")
            
            # Wait for submitted tasks to complete (poll with timeout)
            max_wait_time = 90  # Maximum 90 seconds to wait
            wait_interval = 0.5  # Check every 0.5 seconds
            elapsed = 0
            
            while elapsed < max_wait_time:
                with core.lock:
                    submitted_tasks = [t for t in core.tasks if t["status"] == "submitted"]
                    if len(submitted_tasks) == 0:
                        log("INFO", f"All submitted tasks completed for stop of order {order_id}", service="scheduler")
                        break
                    else:
                        log("DEBUG", f"Waiting for {len(submitted_tasks)} submitted tasks to complete for order {order_id}", service="scheduler")
                
                await asyncio.sleep(wait_interval)
                elapsed += wait_interval
            
            # Check if we timed out
            with core.lock:
                submitted_tasks = [t for t in core.tasks if t["status"] == "submitted"]
                if len(submitted_tasks) > 0:
                    log("ERROR", f"Stop order {order_id} timed out with {len(submitted_tasks)} tasks still submitted", service="scheduler")
            
            # Send stopped event when actually stopped
            try:
                await self.rabbitmq_client.send_event("scheduler.order_stopped", {
                    "order_id": order_id,
                    "timestamp": datetime.now().isoformat()
                })
            except Exception as e:
                log("ERROR", f"Order {order_id} stopped event send failed: {str(e)[:50]}", service="scheduler")
            
            log("INFO", f"Order {order_id} stop complete", service="scheduler")
            return {"success": True, "message": "Order stopped - all tasks completed or halted"}
        except Exception as e:
            log("ERROR", f"Stop order handler exception for order {order_id}: {str(e)[:100]}", service="scheduler")
            return {"success": False, "error": str(e)}
    
    async def handle_resume_order(self, data: Dict) -> Dict:
        """Handle resume order request - clears stop flag."""
        try:
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            from . import scheduler as core
            
            with core.lock:
                if core.current_status.get("order_id") != order_id:
                    return {"success": False, "error": f"Order {order_id} was not stopped"}
                
                # Clear stop flag
                core.order_stopped = False
                core.current_status["status"] = "in_progress"
                log("INFO", f"Order {order_id} resumed", service="scheduler")
            
            return {"success": True, "message": "Order resumed"}
        except Exception as e:
            log("ERROR", f"Resume order handler exception for order {order_id}: {str(e)[:100]}", service="scheduler")
            return {"success": False, "error": str(e)}
    
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
        log("INFO", "Shutdown received", service="scheduler")
        await self.stop()
    
    async def _process_order_async(self, order_id: int, drinks: List[Dict]):
        """Background coroutine to process each drink using the scheduler."""
        log("INFO", f"Background processing started for order {order_id} with {len(drinks)} drinks", service="scheduler")
        
        
        try:
            # Send processing started event
            await self.rabbitmq_client.send_event("scheduler.order_processing_started", {
                "order_id": order_id,
                "timestamp": datetime.now().isoformat()
            })
            
            # Use the scheduler's process_order_async function
            # Note: The scheduler module handles sending completion/failure events to OMS
            # Log per-arm lists [[step, cup_id], ...] before processing
            try:
                lists = scheduler._format_per_arm_lists()
                log("DEBUG", f"Per-arm task lists for order {order_id}", service="scheduler")
            except Exception as e:
                log("DEBUG", f"Per-arm list format failed for order {order_id}: {str(e)[:50]}", service="scheduler")

            success = await scheduler.process_order_async(order_id, drinks, recipes)
            
            # Before processing, emit per-arm plan so UI can render task lists
            try:
                plan = scheduler.get_per_arm_lists()
                await self.rabbitmq_client.send_event("scheduler.plan_built", {
                    "order_id": order_id,
                    "plan": plan,
                    "timestamp": datetime.now().isoformat()
                })
            except Exception as plan_err:
                log("ERROR", f"Plan built event send failed for order {order_id}: {str(plan_err)[:50]}", service="scheduler")

            # Send status notifications (events are already sent by scheduler module)
            if success:
                log("INFO", f"Order {order_id} processing completed successfully", service="scheduler")
                await self.notify_status(f"Order {order_id} completed successfully")
                # Ensure order_stopped flag is reset after successful completion
                scheduler.order_stopped = False
            else:
                log("ERROR", f"Order {order_id} processing failed", service="scheduler")
                await self.notify_status(f"Failed to process order {order_id}")
                # Ensure order_stopped flag is reset after failure
                scheduler.order_stopped = False
                
        except Exception as e:
            log("ERROR", f"Async order processing exception for order {order_id}: {str(e)[:100]}", service="scheduler")
            
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
            
            log("DEBUG", "Status update event sent", service="scheduler")
            
        except Exception as e:
            log("ERROR", "Status update event send failed", service="scheduler", error=str(e)[:100])

async def main():
    """Main service entry point."""
    service = SchedulerService()
    try:
        await service.start()
    except KeyboardInterrupt:
        log("INFO", "Interrupt received", service="scheduler")
        await service.stop()

if __name__ == "__main__":
    asyncio.run(main()) 