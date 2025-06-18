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

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

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

class RoutineService:
    """Routine service for BARNS task orchestration."""
    
    def __init__(self):
        self.rabbitmq_client = RabbitMQClient("routine")
        self.event_listener = EventListener("routine")
        self.worker_tasks = []
        
    async def start(self):
        """Start the routine service."""
        await self.rabbitmq_client.connect()
        await self.event_listener.connect()
        
        # Load task configurations
        await self._load_task_configs()
        
        # Register message handlers
        self.rabbitmq_client.register_handler("submit_task", self.handle_submit_task)
        self.rabbitmq_client.register_handler("health", self.handle_health)
        self.rabbitmq_client.register_handler("get_queue_status", self.handle_get_queue_status)
        self.rabbitmq_client.register_handler("clear_queue", self.handle_clear_queue)
        
        # Subscribe to events
        await self.event_listener.subscribe_to_events(["system.*", "scheduler.*"])
        self.event_listener.register_event_handler("system.shutdown", self.handle_shutdown_event)
        
        # Start worker coroutines for each arm
        for arm_id in [1, 2]:
            task = asyncio.create_task(self.worker(arm_id, task_queues[arm_id]))
            self.worker_tasks.append(task)
        
        logger.info("Routine service started and listening for messages")
        
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down routine service...")
        finally:
            await self.stop()
    
    async def stop(self):
        """Stop the routine service."""
        # Cancel worker tasks
        for task in self.worker_tasks:
            task.cancel()
        
        # Wait for tasks to complete
        await asyncio.gather(*self.worker_tasks, return_exceptions=True)
        
        await self.rabbitmq_client.disconnect()
        await self.event_listener.disconnect()
        logger.info("Routine service stopped")
    
    async def _load_task_configs(self):
        """Load task configurations from file."""
        global task_configs
        cfg_path = os.getenv("ROUTINE_CONFIG_PATH", "/app/config/tasks.json")
        
        try:
            with open(cfg_path) as f:
                task_configs = json.load(f)
            logger.info(f"Loaded task configurations from {cfg_path}")
        except FileNotFoundError:
            logger.info("Task Configs not found")
    
    async def worker(self, arm_id: int, q: asyncio.Queue):
        """Continuously process tasks assigned to this arm."""
        logger.info(f"Worker started for Arm {arm_id}")
        
        while True:
            try:
                task_data = await q.get()
                try:
                    await process_task(arm_id, task_data, task_configs, self.rabbitmq_client)
                    
                    # Send completion event
                    await self._send_task_event("routine.task_completed", task_data, arm_id)
                    
                except Exception as e:
                    logger.error(f"[Routine][Arm{arm_id}] error: {e}")
                    
                    # Send error event
                    await self._send_task_event("routine.task_failed", task_data, arm_id, str(e))
                finally:
                    q.task_done()
                    
            except asyncio.CancelledError:
                logger.info(f"Worker for Arm {arm_id} cancelled")
                break
            except Exception as e:
                logger.error(f"[Routine][Arm{arm_id}] unexpected error: {e}")
    
    async def _send_task_event(self, event_type: str, task_data: Dict, arm_id: int, error: str = None):
        """Send task completion or failure event."""
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
            await self.rabbitmq_client.send_event("routine.task_queued", {
                "arm_id": arm_id,
                "function": function,
                "queue_size": task_queues[arm_id].qsize()
            })
            
            return {
                "status": "queued",
                "arm_id": arm_id,
                "function": function,
                "queue_size": task_queues[arm_id].qsize(),
                "success": True
            }
            
        except Exception as e:
            logger.error(f"Error submitting task: {e}")
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
            logger.error(f"Error getting queue status: {e}")
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
                
                await self.rabbitmq_client.send_event("routine.queue_cleared", {
                    "arm_id": arm_id,
                    "timestamp": datetime.now().isoformat()
                })
                
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
                
                await self.rabbitmq_client.send_event("routine.all_queues_cleared", {
                    "cleared_arms": cleared_arms,
                    "timestamp": datetime.now().isoformat()
                })
                
                return {"cleared_arms": cleared_arms, "success": True}
                
        except Exception as e:
            logger.error(f"Error clearing queue: {e}")
            return {"error": str(e), "success": False}
    
    async def handle_shutdown_event(self, data: Dict):
        """Handle system shutdown events."""
        logger.info("Received shutdown event, stopping routine service...")
        await self.stop()

async def main():
    """Main service entry point."""
    service = RoutineService()
    await service.start()

if __name__ == "__main__":
    asyncio.run(main()) 