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
import time
from datetime import datetime

# PYTHONPATH is set via environment variables in docker-compose

# Import sequence functions
try:
    from oms_v1.sequences.home      import SEQUENCES as HOME_SEQ
    from oms_v1.sequences.espresso import SEQUENCES as ESPR_SEQ
    from oms_v1.sequences.cleaning  import SEQUENCES as CLEAN_SEQ
    from oms_v1.sequences.test      import SEQUENCES as TEST_SEQ
    from oms_v1.sequences.paper_cups import SEQUENCES as PAPER_SEQ
    from oms_v1.sequences.plastic_cups import SEQUENCES as PLASTIC_SEQ
    from oms_v1.sequences.slush import SEQUENCES as SLUSH_SEQ
    from oms_v1.sequences.milk_frothing import SEQUENCES as MILK_SEQ
    from oms_v1.sequence.computer_vision import SEQUENCES as COMPUTER_VISION


except ImportError as e:
    print(f"[ERROR] Could not import sequences: {e}")
    sys.exit(1)

# Merge all sequence mappings
SEQUENCES = {}
SEQUENCES.update(HOME_SEQ)
SEQUENCES.update(ESPR_SEQ)
SEQUENCES.update(CLEAN_SEQ)
SEQUENCES.update(TEST_SEQ)
SEQUENCES.update(PAPER_SEQ)
SEQUENCES.update(PLASTIC_SEQ)
SEQUENCES.update(SLUSH_SEQ)
SEQUENCES.update(MILK_SEQ)
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
        self.is_running = asyncio.Event()
        
    async def start(self):
        """Start the robot container service with automatic reconnection."""
        reconnect_attempts = 0
        while True:
            try:
                reconnect_attempts += 1
                if reconnect_attempts > 1:
                    logger.info(f"🔄 [ROBOT-{self.robot_id}] Reconnection attempt #{reconnect_attempts}")
                
                await self._start_service()
                # If we get here without exception, service was gracefully stopped
                logger.info(f"✅ [ROBOT-{self.robot_id}] Service stopped gracefully")
                break
                
            except KeyboardInterrupt:
                logger.info(f"⚠️ [ROBOT-{self.robot_id}] Shutting down robot container service...")
                await self.stop()
                break
                
            except Exception as e:
                logger.error(f"❌ [ROBOT-{self.robot_id}] Service error: {e}")
                logger.info(f"🔄 [ROBOT-{self.robot_id}] Restarting service in 10 seconds...")
                await self.stop()  # Clean up before retrying
                await asyncio.sleep(10)
                # Continue loop to reconnect

    async def _start_service(self):
        """Internal method to start service components with connection monitoring."""
        try:
            from shared.rabbitmq_client import RabbitMQClient
            self.rabbitmq_client = RabbitMQClient(self.service_name)

            # Retry connection logic for RabbitMQ with infinite retries
            connection_attempt = 0
            while True:
                try:
                    connection_attempt += 1
                    logger.info(f"🤖 [ROBOT-{self.robot_id}] Attempting to connect to RabbitMQ (attempt #{connection_attempt})...")
                    await self.rabbitmq_client.connect()
                    logger.info(f"✅ [ROBOT-{self.robot_id}] Successfully connected to RabbitMQ")
                    break
                except Exception as e:
                    logger.error(f"❌ [ROBOT-{self.robot_id}] Failed to connect to RabbitMQ: {e}")
                    logger.info(f"🔄 [ROBOT-{self.robot_id}] Retrying connection in 10 seconds...")
                    await asyncio.sleep(10)
                    # Continue loop - never give up!
            
            # Register handlers
            self.rabbitmq_client.register_handler("execute_action", self.handle_execute_action)
            self.rabbitmq_client.register_handler("list_actions", self.handle_list_actions)
            self.rabbitmq_client.register_handler("health", self.handle_health)

            logger.info(f"✅ [ROBOT-{self.robot_id}] Robot Container service started and listening for messages")
            logger.info(f"🎯 [ROBOT-{self.robot_id}] Available actions: {len(ACTION_MAP)} actions loaded")

            # Monitor connection health with periodic checks
            try:
                while True:
                    # Check if connection is still alive
                    if not self.rabbitmq_client or not hasattr(self.rabbitmq_client, 'connection'):
                        logger.error(f"❌ [ROBOT-{self.robot_id}] RabbitMQ connection lost!")
                        raise ConnectionError("RabbitMQ connection lost")
                    
                    # Wait a bit before next health check
                    await asyncio.sleep(30)  # Check every 30 seconds
                    
            except KeyboardInterrupt:
                logger.info(f"⚠️ [ROBOT-{self.robot_id}] Shutting down robot container service...")
                raise
            except ConnectionError:
                logger.error(f"❌ [ROBOT-{self.robot_id}] Connection error detected - triggering reconnection")
                raise  # Trigger outer loop reconnection
            except Exception as e:
                logger.error(f"❌ [ROBOT-{self.robot_id}] Unexpected error in service loop: {e}")
                raise  # Trigger outer loop reconnection

        except ImportError as e:
            logger.error(f"❌ [ROBOT-{self.robot_id}] RabbitMQ client not available: {e}")
            raise
        except KeyboardInterrupt:
            raise  # Pass through keyboard interrupt
        except Exception as e:
            logger.error(f"❌ [ROBOT-{self.robot_id}] Error in service: {e}")
            raise  # Re-raise to trigger restart in outer loop

    async def stop(self):
        """Stop the robot container service and clean up connections."""
        if self.rabbitmq_client:
            try:
                await self.rabbitmq_client.disconnect()
                logger.info(f"🛑 [ROBOT-{self.robot_id}] RabbitMQ connection closed")
            except Exception as e:
                logger.error(f"⚠️ [ROBOT-{self.robot_id}] Error closing RabbitMQ connection: {e}")
            finally:
                self.rabbitmq_client = None
        logger.info(f"🛑 [ROBOT-{self.robot_id}] Robot container service stopped")

    async def handle_execute_action(self, data: Dict) -> Dict:
        """Handle action execution requests."""
        action_name = data.get("action_name", "unknown")
        
        try:
            params = data.get("params", {})
            
            logger.info(f"🤖 [ROBOT-{self.robot_id}] Received action request: {action_name}")
            logger.info(f"🤖 [ROBOT-{self.robot_id}] Parameters: {params}")
            
            if not action_name or action_name == "unknown":
                logger.error(f"❌ [ROBOT-{self.robot_id}] No action_name provided")
                return {
                    "success": False,
                    "error": "No action_name provided",
                    "robot_id": self.robot_id
                }
            
            if action_name not in ACTION_MAP:
                logger.error(f"❌ [ROBOT-{self.robot_id}] Unknown action: {action_name}")
                logger.info(f"🤖 [ROBOT-{self.robot_id}] Available actions: {list(ACTION_MAP.keys())}")
                return {
                    "success": False,
                    "error": f"Unknown action: {action_name}",
                    "available_actions": list(ACTION_MAP.keys()),
                    "robot_id": self.robot_id,
                    "action_name": action_name
                }
            
            logger.info(f"✅ [ROBOT-{self.robot_id}] Action {action_name} accepted - starting execution...")
            
            # Execute the action
            fn = ACTION_MAP[action_name]
            
            # Run in thread pool to avoid blocking
            loop = asyncio.get_event_loop()
            start_time = asyncio.get_event_loop().time()
            
            try:
                result = await loop.run_in_executor(None, lambda: fn(**params))
                execution_time = asyncio.get_event_loop().time() - start_time
                
                if result == True:
                    response = {
                        "success": True,
                        "message": f"Action {action_name} completed successfully",
                        "robot_id": self.robot_id,
                        "action_name": action_name,
                        "execution_time": round(execution_time, 2)
                    }
                    
                    logger.info(f"✅ [ROBOT-{self.robot_id}] Action {action_name} completed successfully in {execution_time:.2f}s")
                    
                elif result == False:
                    response = {
                        "success": False,
                        "error": f"Action {action_name} returned False",
                        "message": f"Action {action_name} execution failed",
                        "robot_id": self.robot_id,
                        "action_name": action_name,
                        "execution_time": round(execution_time, 2)
                    }
                    
                    logger.error(f"❌ [ROBOT-{self.robot_id}] Action {action_name} failed (returned False) in {execution_time:.2f}s")
                    
                else:
                    # Handle other return types (strings, dicts, etc.)
                    response = {
                        "success": True,
                        "result": result,
                        "message": f"Action {action_name} completed",
                        "robot_id": self.robot_id,
                        "action_name": action_name,
                        "execution_time": round(execution_time, 2)
                    }
                    
                    logger.info(f"✅ [ROBOT-{self.robot_id}] Action {action_name} completed with result: {result} in {execution_time:.2f}s")
                
                return response
                
            except Exception as exec_error:
                execution_time = asyncio.get_event_loop().time() - start_time
                error_msg = f"Exception during action execution: {str(exec_error)}"
                logger.error(f"💥 [ROBOT-{self.robot_id}] {error_msg} (after {execution_time:.2f}s)")
                
                return {
                    "success": False,
                    "error": error_msg,
                    "message": f"Action {action_name} threw exception",
                    "robot_id": self.robot_id,
                    "action_name": action_name,
                    "execution_time": round(execution_time, 2)
                }
            
        except ConnectionError as e:
            logger.error(f"❌ [ROBOT-{self.robot_id}] Connection error in handle_execute_action: {e}")
            # This will trigger service restart
            raise  # Propagate to trigger reconnection
        except Exception as e:
            error_msg = f"Error processing action request: {str(e)}"
            logger.error(f"💥 [ROBOT-{self.robot_id}] {error_msg}")
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
            "actions_count": len(ACTION_MAP),
            "timestamp": datetime.now().isoformat(),
            "connection_status": "connected",
            "ready": True,
            "healthy": True  # Explicit boolean for easier checking
        }

async def run_service_mode():
    """Run robot container as a RabbitMQ service."""
    # Get robot ID from environment
    robot_id = int(os.getenv("ROBOT_ID", "1"))
    
    service = RobotContainerService(robot_id)
    try:
        await service.start()
    except KeyboardInterrupt:
        logger.info("Service interrupted by user. Exiting.")

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
        try:
            asyncio.run(run_service_mode())
        except KeyboardInterrupt:
            logger.info("Shutting down...")
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
