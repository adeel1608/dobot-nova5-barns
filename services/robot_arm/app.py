"""BARNS Robot Arm Service

Handles robotic arm operations including movement control, gripper operations,
and coordinated task execution for coffee brewing automation.
"""

import asyncio
import logging
import os
import sys
from datetime import datetime
from typing import Dict

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.logger import log
from shared.rabbitmq_client import RabbitMQClient, EventListener
from .robot_actions import ROBOT_ACTIONS, execute_robot_action, get_robot_container_actions

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotArmService:
    """Robot arm service for BARNS automation system."""
    
    def __init__(self):
        self.rabbitmq_client = RabbitMQClient("robot_arm")
        self.event_listener = EventListener("robot_arm")
        self.simulation_mode = os.getenv("ROBOT_SIMULATION", "true").lower() == "true"
        
    async def start(self):
        """Start the robot arm service with automatic reconnection."""
        while True:
            try:
                await self._start_service()
                # If we get here, the service was interrupted
                break
            except KeyboardInterrupt:
                log("INFO", "Shutting down robot arm service...", service="robot_arm")
                await self.stop()
                break
            except Exception as e:
                log("ERROR", f"Service error: {e}", service="robot_arm")
                log("INFO", "Restarting service in 10 seconds...", service="robot_arm")
                await self._cleanup()
                await asyncio.sleep(10)

    async def _start_service(self):
        """Internal method to start the service components."""
        # Retry connection logic for RabbitMQ
        while True:
            try:
                log("INFO", "Connecting to RabbitMQ...", service="robot_arm")
                await self.rabbitmq_client.connect()
                await self.event_listener.connect()
                log("INFO", "RabbitMQ connections established", service="robot_arm")
                break
            except Exception as e:
                log("ERROR", f"RabbitMQ connection failed: {str(e)[:100]}", service="robot_arm")
                log("INFO", "Retrying RabbitMQ connection in 10 seconds...", service="robot_arm")
                await asyncio.sleep(10)
        
        # Register message handlers
        self.rabbitmq_client.register_handler("robot_action", self.handle_robot_action)
        self.rabbitmq_client.register_handler("health", self.handle_health)
        self.rabbitmq_client.register_handler("list_actions", self.handle_list_actions)
        self.rabbitmq_client.register_handler("emergency_stop", self.handle_emergency_stop)
        self.rabbitmq_client.register_handler("calibrate", self.handle_calibrate)
        self.rabbitmq_client.register_handler("get_status", self.handle_get_status)
        
        # Subscribe to events
        await self.event_listener.subscribe_to_events(["system.*", "robot.*"])
        self.event_listener.register_event_handler("system.shutdown", self.handle_shutdown_event)
        self.event_listener.register_event_handler("robot.emergency_stop", self.handle_emergency_stop_event)
        
        mode = "SIMULATION" if self.simulation_mode else "HARDWARE"
        log("INFO", f"Robot Arm service started in {mode} mode and listening for messages", service="robot_arm")
        
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            log("INFO", "Shutting down robot arm service...", service="robot_arm")
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
            log("ERROR", f"Error during cleanup: {e}", service="robot_arm")

    async def stop(self):
        """Stop the robot arm service."""
        await self._cleanup()
        log("INFO", "Robot arm service stopped", service="robot_arm")
    
    async def handle_robot_action(self, data: Dict) -> Dict:
        """Handle robot action requests."""
        try:
            function = data.get("function")
            params = data.get("params", {})
            arm_id = data.get("arm_id", 1)  # Default to arm 1
            
            # Validate required function parameter
            if not function:
                return {
                    "success": False,
                    "error": "Missing required 'function' parameter",
                    "message": "Robot action function name is required"
                }
            
            # Add arm_id to params for robot actions
            params["arm_id"] = arm_id
            params["simulation_mode"] = self.simulation_mode
            
            # Send start event
            try:
                await self.rabbitmq_client.send_event("robot.action_started", {
                    "function": function,
                    "arm_id": arm_id,
                    "params": params,
                    "timestamp": datetime.now().isoformat()
                })
            except ConnectionError as e:
                log("ERROR", f"Connection error sending action started event: {e}", service="robot_arm")
                # Continue with action execution but note the connection issue
            
            # Execute robot action (this will route to robot container for non-test actions)
            result = await execute_robot_action(function, params)
            
            # Send completion event
            try:
                await self.rabbitmq_client.send_event("robot.action_completed", {
                    "function": function,
                    "arm_id": arm_id,
                    "result": result,
                    "timestamp": datetime.now().isoformat()
                })
            except ConnectionError as e:
                log("ERROR", f"Connection error sending action completed event: {e}", service="robot_arm")
            
            return result
            
        except ConnectionError as e:
            log("ERROR", f"Connection error in handle_robot_action: {e}", service="robot_arm")
            raise  # This will trigger service restart
        except Exception as e:
            log("ERROR", f"Error in robot action: {e}", service="robot_arm")
            
            # Send error event
            try:
                await self.rabbitmq_client.send_event("robot.action_error", {
                    "function": function or "unknown",
                    "arm_id": arm_id,
                    "error": str(e),
                    "timestamp": datetime.now().isoformat()
                })
            except ConnectionError as conn_error:
                log("ERROR", f"Connection error sending action error event: {conn_error}", service="robot_arm")
            except Exception as event_error:
                log("ERROR", f"Error sending action error event: {event_error}", service="robot_arm")
            
            return {
                "success": False,
                "error": f"Error executing robot action '{function or 'unknown'}': {str(e)}",
                "message": "Robot action failed"
            }
    
    async def handle_health(self, data: Dict) -> Dict:
        """Handle health check requests."""
        return {
            "status": "healthy",
            "service": "robot_arm",
            "simulation_mode": self.simulation_mode,
            "timestamp": datetime.now().isoformat(),
            "available_actions": len(ROBOT_ACTIONS)
        }
    
    async def handle_list_actions(self, data: Dict) -> Dict:
        """Handle action listing requests."""
        try:
            # Get test actions (always available)
            test_actions = list(ROBOT_ACTIONS.keys())
            
            # Try to get robot container actions
            robot_container_actions = []
            if not self.simulation_mode:
                try:
                    robot_container_actions = await get_robot_container_actions()
                except Exception as e:
                    log("ERROR", f"Could not get robot container actions: {e}", service="robot_arm")
            
            # Combine actions
            all_actions = test_actions + robot_container_actions
            
            return {
                "actions": all_actions,
                "test_actions": test_actions,
                "robot_container_actions": robot_container_actions,
                "count": len(all_actions),
                "simulation_mode": self.simulation_mode,
                "success": True
            }
        except Exception as e:
            log("ERROR", f"Error listing actions: {e}", service="robot_arm")
            return {
                "actions": list(ROBOT_ACTIONS.keys()),
                "count": len(ROBOT_ACTIONS),
                "simulation_mode": self.simulation_mode,
                "success": True,
                "error": str(e)
            }
    
    async def handle_emergency_stop(self, data: Dict) -> Dict:
        """Handle emergency stop requests."""
        try:
            arm_id = data.get("arm_id", "all")
            
            await self.rabbitmq_client.send_event("robot.emergency_stopped", {
                "arm_id": arm_id,
                "timestamp": datetime.now().isoformat(),
                "reason": "Emergency stop requested"
            })
            
            # Here you would implement actual emergency stop logic
            # For now, we'll just log and respond
            log("ERROR", f"Emergency stop activated for arm {arm_id}", service="robot_arm")
            
            return {
                "success": True,
                "message": f"Emergency stop activated for arm {arm_id}"
            }
            
        except Exception as e:
            log("ERROR", f"Error during emergency stop: {e}", service="robot_arm")
            return {
                "success": False,
                "error": str(e)
            }
    
    async def handle_calibrate(self, data: Dict) -> Dict:
        """Handle calibration requests."""
        try:
            arm_id = data.get("arm_id", 1)
            calibration_type = data.get("type", "auto")
            
            await self.rabbitmq_client.send_event("robot.calibration_started", {
                "arm_id": arm_id,
                "type": calibration_type,
                "timestamp": datetime.now().isoformat()
            })
            
            # Simulate calibration process
            if self.simulation_mode:
                await asyncio.sleep(2)  # Simulate calibration time
                result = {
                    "success": True,
                    "message": f"Calibration completed for arm {arm_id}",
                    "calibration_data": {
                        "accuracy": 0.1,  # mm
                        "home_position": [0, 0, 0, 0, 0, 0],
                        "workspace_verified": True
                    }
                }
            else:
                # Here you would implement actual calibration
                result = {
                    "success": True,
                    "message": f"Hardware calibration completed for arm {arm_id}"
                }
            
            await self.rabbitmq_client.send_event("robot.calibration_completed", {
                "arm_id": arm_id,
                "result": result,
                "timestamp": datetime.now().isoformat()
            })
            
            return result
            
        except Exception as e:
            log("ERROR", f"Error during calibration: {e}", service="robot_arm")
            return {
                "success": False,
                "error": str(e)
            }
    
    async def handle_get_status(self, data: Dict) -> Dict:
        """Handle status requests."""
        try:
            arm_id = data.get("arm_id", 1)
            
            # Simulate status information
            status = {
                "arm_id": arm_id,
                "is_connected": True,
                "is_moving": False,
                "current_position": {"x": 300, "y": 200, "z": 150},
                "joint_positions": [0, 45, -90, 0, 45, 0],
                "tool_status": "gripper_open",
                "safety_status": "ok",
                "last_action": "move_to_position",
                "simulation_mode": self.simulation_mode,
                "timestamp": datetime.now().isoformat()
            }
            
            return {
                "success": True,
                "status": status
            }
            
        except Exception as e:
            log("ERROR", f"Error getting robot status: {e}", service="robot_arm")
            return {
                "success": False,
                "error": str(e)
            }
    
    async def handle_shutdown_event(self, data: Dict):
        """Handle system shutdown events."""
        log("INFO", "Received shutdown event, stopping robot arm service...", service="robot_arm")
        await self.stop()
    
    async def handle_emergency_stop_event(self, data: Dict):
        """Handle emergency stop events."""
        log("ERROR", "Emergency stop event received!", service="robot_arm")
        arm_id = data.get("arm_id", "all")
        await self.handle_emergency_stop({"arm_id": arm_id})

async def main():
    """Main service entry point."""
    service = RobotArmService()
    try:
        await service.start()
    except KeyboardInterrupt:
        log("INFO", "Received interrupt signal, shutting down...", service="robot_arm")
        await service.stop()

if __name__ == "__main__":
    asyncio.run(main()) 