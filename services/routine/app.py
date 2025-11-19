"""BARNS Routine Service

Orchestrates task execution for robotic arms by managing task queues,
coordinating with validation and automation services.
"""

import asyncio
import json
import logging
import os
import sys
from datetime import datetime
from typing import Dict
import time

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.logger import log
from shared.rabbitmq_client import RabbitMQClient, EventListener
from .executer import process_task

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Task queues for each arm
task_queues: dict[int, asyncio.Queue] = {
    1: asyncio.Queue(),
    2: asyncio.Queue()
}

task_configs = {}

# Shared lock for cup_station robot functions to prevent collisions between arms
cup_station_lock = asyncio.Lock()
cup_station_lock_holder: int | None = None  # Track which arm currently holds the lock

class RoutineService:
    """Routine service for BARNS task orchestration."""
    
    def __init__(self):
        self.rabbitmq_client = RabbitMQClient("routine")
        self.event_listener = EventListener("routine")
        self.worker_tasks = []
        
    async def start(self):
        """Start the routine service with automatic reconnection."""
        while True:
            try:
                await self._start_service()
            except Exception as e:
                log("ERROR", "Service error: {e}", service="routine")
                log("INFO", "Restarting service in 10 seconds...", service="routine")
                await asyncio.sleep(10)
                # Clean up any existing connections
                await self._cleanup()
    
    async def _start_service(self):
        """Internal method to start the service components."""
        # Retry connection logic for RabbitMQ
        while True:
            try:
                log("INFO", "Attempting to connect to RabbitMQ...", service="routine")
                await self.rabbitmq_client.connect()
                await self.event_listener.connect()
                log("INFO", "Successfully connected to RabbitMQ", service="routine")
                break
            except Exception as e:
                log("ERROR", "Failed to connect to RabbitMQ: {e}", service="routine")
                log("INFO", "Retrying connection in 10 seconds...", service="routine")
                await asyncio.sleep(10)
        
        # Load task configurations
        await self._load_task_configs()
        
        # Register message handlers
        self.rabbitmq_client.register_handler("submit_task", self.handle_submit_task)
        self.rabbitmq_client.register_handler("health", self.handle_health)
        self.rabbitmq_client.register_handler("get_queue_status", self.handle_get_queue_status)
        self.rabbitmq_client.register_handler("clear_queue", self.handle_clear_queue)
        self.rabbitmq_client.register_handler("cancel_order", self.handle_cancel_order)
        self.rabbitmq_client.register_handler("stop_order", self.handle_stop_order)
        self.rabbitmq_client.register_handler("resume_order", self.handle_resume_order)
        
        # Subscribe to events
        await self.event_listener.subscribe_to_events(["system.*", "scheduler.*"])
        self.event_listener.register_event_handler("system.shutdown", self.handle_shutdown_event)
        
        # Start worker coroutines for each arm
        for arm_id in [1, 2]:
            task = asyncio.create_task(self.worker(arm_id, task_queues[arm_id]))
            self.worker_tasks.append(task)
        
        log("INFO", "Routine service started and listening for messages", service="routine")
        
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            log("INFO", "Shutting down routine service...", service="routine")
            raise
    
    async def _cleanup(self):
        """Clean up connections and tasks."""
        try:
            # Cancel worker tasks
            for task in self.worker_tasks:
                if not task.done():
                    task.cancel()
            
            # Wait for tasks to complete
            if self.worker_tasks:
                await asyncio.gather(*self.worker_tasks, return_exceptions=True)
            
            # Clear worker tasks list
            self.worker_tasks.clear()
            
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
            log("ERROR", "Error during cleanup: {e}", service="routine")

    async def stop(self):
        """Stop the routine service."""
        await self._cleanup()
        log("INFO", "Routine service stopped", service="routine")
    
    async def _load_task_configs(self):
        """Load task configurations from file."""
        global task_configs
        cfg_path = os.getenv("ROUTINE_CONFIG_PATH", "/app/config/tasks.json")
        
        try:
            with open(cfg_path) as f:
                task_configs = json.load(f)
            log("INFO", "Loaded task configurations from {cfg_path}", service="routine")
        except FileNotFoundError:
            log("INFO", "Task Configs not found", service="routine")
    
    async def worker(self, arm_id: int, q: asyncio.Queue):
        """Continuously process tasks assigned to this arm."""
        log("INFO", "Worker started for Arm {arm_id}", service="routine")
        
        while True:
            try:
                task_data = await q.get()
                try:
                    # Process task and get execution status
                    log("INFO", f"[WORKER] Arm{arm_id} processing task: {task_data.get('function', 'unknown')}", service="routine")
                    result = await process_task(arm_id, task_data, task_configs, self.rabbitmq_client)
                    log("INFO", f"[WORKER] Arm{arm_id} task result: success={result.get('success')}, validation_failed_stopped={result.get('validation_failed_stopped')}, order_stopped={result.get('order_stopped')}", service="routine")
                    
                    # Only send events if task actually completed or failed
                    # Skip events if validation failed or order stopped (task remains pending for retry/resume)
                    if result.get("validation_failed_stopped", False):
                        log("INFO", f"[WORKER] Arm{arm_id} validation failure detected - skipping event emission", service="routine")
                        log("INFO", f"[WORKER] Arm{arm_id} task remains PENDING in scheduler for retry", service="routine")
                        # Don't send any event - task remains pending in scheduler
                    elif result.get("order_stopped", False):
                        log("INFO", f"[WORKER] Arm{arm_id} order stopped detected - skipping event emission", service="routine")
                        log("INFO", f"[WORKER] Arm{arm_id} task remains in SUBMITTED state in scheduler for resume", service="routine")
                        # Don't send any event - task remains submitted in scheduler
                    elif result.get("success", True):
                        # Task completed successfully
                        log("INFO", f"[WORKER] Arm{arm_id} task succeeded - sending completion event", service="routine")
                        await self._send_task_event("routine.task_completed", task_data, arm_id)
                    else:
                        # Task failed (not due to validation or order stop)
                        log("ERROR", f"[WORKER] Arm{arm_id} task failed - sending failure event", service="routine")
                        await self._send_task_event("routine.task_failed", task_data, arm_id, result.get("message", "Task failed"))
                    
                except Exception as e:
                    log("ERROR", "[Routine][Arm{arm_id}] error: {e}", service="routine")
                    
                    # Send error event for unexpected exceptions
                    await self._send_task_event("routine.task_failed", task_data, arm_id, str(e))
                finally:
                    q.task_done()
                    
            except asyncio.CancelledError:
                log("INFO", "Worker for Arm {arm_id} cancelled", service="routine")
                break
            except ConnectionError as e:
                log("ERROR", "[Routine][Arm{arm_id}] RabbitMQ connection error: {e}", service="routine")
                # This will trigger service restart
                raise
            except Exception as e:
                log("ERROR", "[Routine][Arm{arm_id}] unexpected error: {e}", service="routine")
    
    async def _send_task_event(self, event_type: str, task_data: Dict, arm_id: int, error: str = None):
        """Send task completion or failure event."""
        try:
            cup_id = task_data.get("item", {}).get("cup_id", "unknown")
            function = task_data.get("function", "unknown")
            
            event_data = {
                "cup_id": cup_id,
                "function": function,
                "arm_id": arm_id,
                "timestamp": datetime.now().isoformat()
            }
            
            if error:
                event_data["error"] = error
                
            await self.rabbitmq_client.send_event(event_type, event_data)
        except ConnectionError as e:
            log("ERROR", "Connection error sending event: {e}", service="routine")
            # Re-raise to trigger service restart
            raise
        except Exception as e:
            log("ERROR", "Error sending task event: {e}", service="routine")
            # Don't re-raise other exceptions as they shouldn't cause reconnection
    
    async def handle_submit_task(self, data: Dict) -> Dict:
        """Handle task submission requests."""
        try:
            arm_id = data.get("arm_id")
            function = data.get("function")
            item = data.get("item", {})
            
            if arm_id not in task_queues:
                return {"error": "Invalid arm_id", "success": False}
            
            if function not in task_configs:
                return {"error": f"Unknown function: {function}", "success": False}
            
            # Create task object
            task_data = {
                "function": function,
                "item": item,
                "arm_id": arm_id,
                "timestamp": datetime.now().isoformat()
            }
            
            # Add to queue
            await task_queues[arm_id].put(task_data)
            
            # Send event
            try:
                await self.rabbitmq_client.send_event("routine.task_queued", {
                    "arm_id": arm_id,
                    "function": function,
                    "queue_size": task_queues[arm_id].qsize()
                })
            except ConnectionError as e:
                log("ERROR", "Connection error sending task queued event: {e}", service="routine")
                # Still return success as task was queued, but log the connection issue
                
            return {
                "status": "queued",
                "arm_id": arm_id,
                "function": function,
                "queue_size": task_queues[arm_id].qsize(),
                "success": True
            }
            
        except ConnectionError as e:
            log("ERROR", "Connection error in handle_submit_task: {e}", service="routine")
            raise  # This will trigger service restart
        except Exception as e:
            log("ERROR", "Error submitting task: {e}", service="routine")
            return {"error": str(e), "success": False}
    
    async def handle_health(self, data: Dict) -> Dict:
        """Handle health check requests."""
        return {
            "status": "healthy",
            "service": "routine",
            "timestamp": datetime.now().isoformat(),
            "arm_queues": {
                arm_id: queue.qsize() for arm_id, queue in task_queues.items()
            },
            "available_functions": list(task_configs.keys())
        }
    
    async def handle_get_queue_status(self, data: Dict) -> Dict:
        """Handle queue status requests."""
        try:
            arm_id = data.get("arm_id")
            
            if arm_id and arm_id in task_queues:
                return {
                    "arm_id": arm_id,
                    "queue_size": task_queues[arm_id].qsize(),
                    "success": True
                }
            else:
                return {
                    "all_arms": {
                        arm_id: queue.qsize() for arm_id, queue in task_queues.items()
                    },
                    "success": True
                }
                
        except Exception as e:
            log("ERROR", "Error getting queue status: {e}", service="routine")
            return {"error": str(e), "success": False}
    
    async def handle_clear_queue(self, data: Dict) -> Dict:
        """Handle queue clearing requests."""
        try:
            arm_id = data.get("arm_id")
            
            if arm_id and arm_id in task_queues:
                # Clear specific arm queue
                while not task_queues[arm_id].empty():
                    try:
                        task_queues[arm_id].get_nowait()
                        task_queues[arm_id].task_done()
                    except asyncio.QueueEmpty:
                        break
                
                try:
                    await self.rabbitmq_client.send_event("routine.queue_cleared", {
                        "arm_id": arm_id,
                        "timestamp": datetime.now().isoformat()
                    })
                except ConnectionError as e:
                    log("ERROR", "Connection error sending queue cleared event: {e}", service="routine")
                
                return {"arm_id": arm_id, "cleared": True, "success": True}
            else:
                # Clear all queues
                cleared_arms = []
                for arm_id, queue in task_queues.items():
                    while not queue.empty():
                        try:
                            queue.get_nowait()
                            queue.task_done()
                        except asyncio.QueueEmpty:
                            break
                    cleared_arms.append(arm_id)
                
                try:
                    await self.rabbitmq_client.send_event("routine.all_queues_cleared", {
                        "cleared_arms": cleared_arms,
                        "timestamp": datetime.now().isoformat()
                    })
                except ConnectionError as e:
                    log("ERROR", "Connection error sending all queues cleared event: {e}", service="routine")
                
                return {"cleared_arms": cleared_arms, "success": True}
                
        except ConnectionError as e:
            log("ERROR", "Connection error in handle_clear_queue: {e}", service="routine")
            raise  # This will trigger service restart
        except Exception as e:
            log("ERROR", "Error clearing queue: {e}", service="routine")
            return {"error": str(e), "success": False}
    
    async def handle_cancel_order(self, data: Dict) -> Dict:
        """Cancel queued tasks for a given order_id (or specific cup_ids)."""
        try:
            order_id = data.get("order_id")
            cup_ids = set(data.get("cup_ids", []) or [])
            if not order_id and not cup_ids:
                return {"success": False, "error": "Missing order_id or cup_ids"}
            
            removed: Dict[int, int] = {}
            for arm_id, queue in task_queues.items():
                kept_items = []
                removed_count = 0
                while not queue.empty():
                    try:
                        item = queue.get_nowait()
                        cup_id = (item or {}).get("item", {}).get("cup_id")
                        match = False
                        if order_id and isinstance(cup_id, str) and cup_id.startswith(f"{order_id}-"):
                            match = True
                        if not match and cup_ids and cup_id in cup_ids:
                            match = True
                        
                        # Mark the dequeued item as done (to balance unfinished_tasks)
                        queue.task_done()
                        
                        if match:
                            removed_count += 1
                        else:
                            kept_items.append(item)
                    except asyncio.QueueEmpty:
                        break
                
                for item in kept_items:
                    await queue.put(item)
                removed[arm_id] = removed_count
            
            try:
                await self.rabbitmq_client.send_event("routine.order_cancelled", {
                    "order_id": order_id,
                    "cup_ids": list(cup_ids) if cup_ids else None,
                    "removed": removed,
                    "timestamp": datetime.now().isoformat()
                })
            except Exception as e:
                log("ERROR", "Error sending routine.order_cancelled event: {e}", service="routine")
            
            return {"success": True, "removed": removed}
        except Exception as e:
            log("ERROR", "Error cancelling order in routine: {e}", service="routine")
            return {"success": False, "error": str(e)}
    
    async def handle_stop_order(self, data: Dict) -> Dict:
        """Handle stop order requests - mark order as stopped to pause task execution."""
        try:
            from .executer import mark_order_stopped
            
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            log("INFO", f"[STOP ORDER] Received stop request for order {order_id}", service="routine")
            
            # Mark the order as stopped
            await mark_order_stopped(order_id)
            
            log("INFO", f"[STOP ORDER] Order {order_id} marked as stopped - tasks will pause gracefully", service="routine")
            
            # Send event
            try:
                await self.rabbitmq_client.send_event("routine.order_stopped", {
                    "order_id": order_id,
                    "timestamp": datetime.now().isoformat()
                })
            except Exception as e:
                log("ERROR", f"Error sending routine.order_stopped event: {e}", service="routine")
            
            return {"success": True, "order_id": order_id, "message": "Order marked as stopped"}
            
        except Exception as e:
            log("ERROR", f"Error stopping order in routine: {e}", service="routine")
            return {"success": False, "error": str(e)}
    
    async def handle_resume_order(self, data: Dict) -> Dict:
        """Handle resume order requests - clear stop flag to allow task execution to continue."""
        try:
            from .executer import mark_order_resumed
            
            order_id = data.get("order_id")
            if not order_id:
                return {"success": False, "error": "Missing order_id"}
            
            log("INFO", f"[RESUME ORDER] Received resume request for order {order_id}", service="routine")
            
            # Clear the stopped flag
            await mark_order_resumed(order_id)
            
            log("INFO", f"[RESUME ORDER] Order {order_id} resumed - tasks will continue execution", service="routine")
            
            # Send event
            try:
                await self.rabbitmq_client.send_event("routine.order_resumed", {
                    "order_id": order_id,
                    "timestamp": datetime.now().isoformat()
                })
            except Exception as e:
                log("ERROR", f"Error sending routine.order_resumed event: {e}", service="routine")
            
            return {"success": True, "order_id": order_id, "message": "Order resumed"}
            
        except Exception as e:
            log("ERROR", f"Error resuming order in routine: {e}", service="routine")
            return {"success": False, "error": str(e)}
    
    async def handle_shutdown_event(self, data: Dict):
        """Handle system shutdown events."""
        log("INFO", "Received shutdown event, stopping routine service...", service="routine")
        await self.stop()

async def main():
    """Main service entry point."""
    service = RoutineService()
    try:
        await service.start()
    except KeyboardInterrupt:
        log("INFO", "Received interrupt signal, shutting down...", service="routine")
        await service.stop()

if __name__ == "__main__":
    asyncio.run(main()) 