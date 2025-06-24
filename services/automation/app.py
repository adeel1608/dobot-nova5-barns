"""BARNS Automation Service

Handles automation functions for coffee brewing equipment including
dispensing, and control operations.
"""

import asyncio
import logging
import os
import sys
from datetime import datetime
from typing import Dict

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.rabbitmq_client import RabbitMQClient, EventListener
from .automation_functions import AUTOMATION_FUNCTIONS

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AutomationService:
    """Automation service for BARNS coffee brewing system."""
    
    def __init__(self):
        self.rabbitmq_client = RabbitMQClient("automation")
        self.event_listener = EventListener("automation")
        
    async def start(self):
        """Start the automation service."""
        await self.rabbitmq_client.connect()
        await self.event_listener.connect()
        
        # Register message handlers
        self.rabbitmq_client.register_handler("automate", self.handle_automate)
        self.rabbitmq_client.register_handler("health", self.handle_health)
        self.rabbitmq_client.register_handler("list_functions", self.handle_list_functions)
        self.rabbitmq_client.register_handler("stop_automation", self.handle_stop_automation)
        
        logger.info(f"🔧 [AUTOMATION] Registered handlers: automate, health, list_functions, stop_automation")
        logger.info(f"📋 [AUTOMATION] Available functions: {list(AUTOMATION_FUNCTIONS.keys())}")
        
        # Subscribe to events
        await self.event_listener.subscribe_to_events(["system.*", "automation.*"])
        self.event_listener.register_event_handler("system.shutdown", self.handle_shutdown_event)
        self.event_listener.register_event_handler("automation.emergency_stop", self.handle_emergency_stop)
        
        logger.info("🚀 [AUTOMATION] Service started and listening for messages")
        
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down automation service...")
        finally:
            await self.stop()
    
    async def stop(self):
        """Stop the automation service."""
        await self.rabbitmq_client.disconnect()
        await self.event_listener.disconnect()
        logger.info("Automation service stopped")
    
    async def handle_automate(self, data: Dict) -> Dict:
        """Handle automation requests."""
        try:
            function = data.get("function")
            params = data.get("params", {})
            
            logger.info(f"🤖 [AUTOMATION] Received automation request: function='{function}', params={params}")
            
            if function not in AUTOMATION_FUNCTIONS:
                logger.error(f"❌ [AUTOMATION] Unknown function '{function}'. Available: {list(AUTOMATION_FUNCTIONS.keys())}")
                return {
                    "success": False,
                    "error": f"No such automation function '{function}'",
                    "message": f"Available functions: {list(AUTOMATION_FUNCTIONS.keys())}"
                }
            
            logger.info(f"🚀 [AUTOMATION] Starting execution of '{function}'")
            
            # Send start event
            await self.rabbitmq_client.send_event("automation.started", {
                "function": function,
                "params": params,
                "timestamp": datetime.now().isoformat()
            })
            
            # Execute automation function
            result = await AUTOMATION_FUNCTIONS[function](params)
            
            logger.info(f"✅ [AUTOMATION] Function '{function}' completed successfully: {result}")
            
            # Send completion event
            await self.rabbitmq_client.send_event("automation.completed", {
                "function": function,
                "result": result,
                "timestamp": datetime.now().isoformat()
            })
            
            return result
            
        except Exception as e:
            logger.error(f"💥 [AUTOMATION] Error executing function '{function}': {e}")
            logger.error(f"💥 [AUTOMATION] Exception details: {type(e).__name__}: {str(e)}")
            
            # Send error event
            await self.rabbitmq_client.send_event("automation.error", {
                "function": function,
                "error": str(e),
                "timestamp": datetime.now().isoformat()
            })
            
            return {
                "success": False,
                "error": f"Error executing automation function '{function}': {str(e)}",
                "message": "Automation function failed"
            }
    
    async def handle_health(self, data: Dict) -> Dict:
        """Handle health check requests."""
        return {
            "status": "healthy",
            "service": "automation",
            "timestamp": datetime.now().isoformat(),
            "available_functions": len(AUTOMATION_FUNCTIONS)
        }
    
    async def handle_list_functions(self, data: Dict) -> Dict:
        """Handle function listing requests."""
        return {
            "functions": list(AUTOMATION_FUNCTIONS.keys()),
            "count": len(AUTOMATION_FUNCTIONS),
            "success": True
        }
    
    async def handle_stop_automation(self, data: Dict) -> Dict:
        """Handle automation stop requests."""
        try:
            await self.rabbitmq_client.send_event("automation.stopped", {
                "timestamp": datetime.now().isoformat(),
                "reason": "Manual stop requested"
            })
            
            return {
                "success": True,
                "message": "Automation processes stopped"
            }
            
        except Exception as e:
            logger.error(f"Error stopping automation: {e}")
            return {
                "success": False,
                "error": str(e)
            }
    
    async def handle_shutdown_event(self, data: Dict):
        """Handle system shutdown events."""
        logger.info("Received shutdown event, stopping automation service...")
        await self.stop()
    
    async def handle_emergency_stop(self, data: Dict):
        """Handle emergency stop events."""
        logger.warning("Emergency stop received!")
        await self.rabbitmq_client.send_event("automation.emergency_stopped", {
            "timestamp": datetime.now().isoformat(),
            "reason": data.get("reason", "Emergency stop triggered")
        })

async def main():
    """Main service entry point."""
    service = AutomationService()
    await service.start()

if __name__ == "__main__":
    asyncio.run(main()) 