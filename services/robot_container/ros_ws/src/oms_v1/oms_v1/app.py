"""
Command-line interface for pickn_place sequences with JSON-driven recipes.
Enhanced with RabbitMQ service for remote robot action execution.
"""
import argparse
import json
import sys
import os
import asyncio
import logging
from pathlib import Path
from typing import Dict

# Add shared directory to path for RabbitMQ client
sys.path.append('/app/shared')
# Also add the current directory structure for local imports
sys.path.append('/root/ros_ws/src')

# Import sequence functions
try:
    from oms_v1.sequences.home      import SEQUENCES as HOME_SEQ
    from oms_v1.sequences.cups      import SEQUENCES as CUPS_SEQ
    from oms_v1.sequences.espresso import SEQUENCES as ESPR_SEQ
    from oms_v1.sequences.cleaning  import SEQUENCES as CLEAN_SEQ
    from oms_v1.sequences.test      import SEQUENCES as TEST_SEQ
except ImportError as e:
    print(f"[ERROR] Could not import sequences: {e}")
    sys.exit(1)

# Merge all sequence mappings
SEQUENCES = {}
SEQUENCES.update(HOME_SEQ)
SEQUENCES.update(CUPS_SEQ)
SEQUENCES.update(ESPR_SEQ)
SEQUENCES.update(CLEAN_SEQ)
SEQUENCES.update(TEST_SEQ)

# Map action names to actual callables
ACTION_MAP = {}
for name, fn in SEQUENCES.items():
    ACTION_MAP[name] = fn

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Function to just run a single action
def run_action(action_name: str, params: dict):
    """
    Run a single action with parameters.
    """
    fn = ACTION_MAP.get(action_name)
    if fn is None:
        print(f"[ERROR] No action found for name: {action_name}")
        return {"success": False, "error": f"No action found for name: {action_name}"}
    
    try:
        print(f"Running action: {action_name} with params: {params}")
        result = fn(**params)
        return {"success": True, "result": result, "message": f"Action {action_name} completed"}
    except Exception as e:
        error_msg = f"Error executing action {action_name}: {str(e)}"
        print(f"[ERROR] {error_msg}")
        return {"success": False, "error": error_msg}

class RobotContainerService:
    """RabbitMQ service for robot container."""
    
    def __init__(self, robot_id: int = 1):
        self.robot_id = robot_id
        self.service_name = f"robot_container_{robot_id}"
        self.rabbitmq_client = None
        
    async def start(self):
        """Start the robot container service."""
        try:
            # Import RabbitMQ client
            from rabbitmq_client import RabbitMQClient
            
            self.rabbitmq_client = RabbitMQClient(self.service_name)
            await self.rabbitmq_client.connect()
            
            # Register message handlers
            self.rabbitmq_client.register_handler("execute_action", self.handle_execute_action)
            self.rabbitmq_client.register_handler("list_actions", self.handle_list_actions)
            self.rabbitmq_client.register_handler("health", self.handle_health)
            
            logger.info(f"Robot Container {self.robot_id} service started and listening for messages")
            logger.info(f"Available actions: {list(ACTION_MAP.keys())}")
            
            try:
                await asyncio.Future()  # Run forever
            except KeyboardInterrupt:
                logger.info("Shutting down robot container service...")
            finally:
                await self.stop()
                
        except ImportError:
            logger.error("RabbitMQ client not available. Running in standalone mode.")
        except Exception as e:
            logger.error(f"Error starting robot container service: {e}")
    
    async def stop(self):
        """Stop the robot container service."""
        if self.rabbitmq_client:
            await self.rabbitmq_client.disconnect()
        logger.info(f"Robot container {self.robot_id} service stopped")
    
    async def handle_execute_action(self, data: Dict) -> Dict:
        """Handle action execution requests."""
        try:
            action_name = data.get("action_name")
            params = data.get("params", {})
            
            if not action_name:
                return {
                    "success": False,
                    "error": "No action_name provided"
                }
            
            if action_name not in ACTION_MAP:
                return {
                    "success": False,
                    "error": f"Unknown action: {action_name}",
                    "available_actions": list(ACTION_MAP.keys())
                }
            
            logger.info(f"Executing action: {action_name} with params: {params}")
            
            # Execute the action
            fn = ACTION_MAP[action_name]
            
            # Run in thread pool to avoid blocking
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, lambda: fn(**params))
            
            response = {
                "success": True,
                "result": result,
                "message": f"Action {action_name} completed successfully",
                "robot_id": self.robot_id,
                "action_name": action_name
            }
            
            logger.info(f"Action {action_name} completed successfully")
            return response
            
        except Exception as e:
            error_msg = f"Error executing action {action_name}: {str(e)}"
            logger.error(error_msg)
            return {
                "success": False,
                "error": error_msg,
                "robot_id": self.robot_id,
                "action_name": action_name
            }
    
    async def handle_list_actions(self, data: Dict) -> Dict:
        """Handle action listing requests."""
        return {
            "success": True,
            "actions": list(ACTION_MAP.keys()),
            "count": len(ACTION_MAP),
            "robot_id": self.robot_id
        }
    
    async def handle_health(self, data: Dict) -> Dict:
        """Handle health check requests."""
        return {
            "status": "healthy",
            "service": self.service_name,
            "robot_id": self.robot_id,
            "available_actions": len(ACTION_MAP),
            "timestamp": str(asyncio.get_event_loop().time())
        }

async def run_service_mode():
    """Run robot container as a RabbitMQ service."""
    # Get robot ID from environment
    robot_id = int(os.getenv("ROBOT_ID", "1"))
    
    service = RobotContainerService(robot_id)
    await service.start()

# CLI functionality remains the same
def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(description="Execute robot actions")
    parser.add_argument("--action", help="Action to execute")
    parser.add_argument("--params", help="JSON parameters for the action", default="{}")
    parser.add_argument("--service", action="store_true", help="Run as RabbitMQ service")
    
    args = parser.parse_args()
    
    if args.service:
        # Run as service
        asyncio.run(run_service_mode())
    elif args.action:
        # Run single action
        try:
            params = json.loads(args.params)
        except json.JSONDecodeError:
            print("[ERROR] Invalid JSON in params")
            return
        
        result = run_action(args.action, params)
        print(json.dumps(result, indent=2))
    else:
        print("Use --action <name> to run an action, or --service to run as RabbitMQ service")
        print(f"Available actions: {list(ACTION_MAP.keys())}")

if __name__ == "__main__":
    main()

