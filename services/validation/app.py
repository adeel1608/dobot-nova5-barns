"""
BARNS Validation Service

Provides modular validation framework for inventory management, 
sensor validation, and quality control.
"""

import asyncio
import logging
import os
import sys
from datetime import datetime
from typing import Dict, Optional

# Add parent directory to path for shared imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.rabbitmq_client import RabbitMQClient, EventListener
from .validations import load_validation_functions
from .validations.inventory import get_inventory_levels, set_inventory_level, get_category_summary

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ValidationService:
    """Modular validation service for BARNS automation system."""
    
    def __init__(self):
        """Initialize the validation service with RabbitMQ clients and load validation functions."""
        self.rabbitmq_client = RabbitMQClient("validation")
        self.event_listener = EventListener("validation")
        self.validators = load_validation_functions()
        logger.info(f"Loaded {len(self.validators)} validation functions: {list(self.validators.keys())}")
        
    async def start(self):
        """Start the validation service and register handlers."""
        await self.rabbitmq_client.connect()
        await self.event_listener.connect()
        
        # Register message handlers
        self.rabbitmq_client.register_handler("validate", self.handle_validate)
        self.rabbitmq_client.register_handler("health", self.handle_health)
        self.rabbitmq_client.register_handler("inventory_status", self.handle_inventory_status)
        self.rabbitmq_client.register_handler("inventory_refill", self.handle_inventory_refill)
        self.rabbitmq_client.register_handler("inventory_category_summary", self.handle_inventory_category_summary)
        
        # Subscribe to events
        await self.event_listener.subscribe_to_events(["inventory.*", "system.*"])
        self.event_listener.register_event_handler("inventory.refill_requested", self.handle_refill_event)
        
        logger.info("Validation service started and listening for messages")
        
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down validation service...")
        finally:
            await self.stop()
    
    async def stop(self):
        """Gracefully stop the validation service."""
        await self.rabbitmq_client.disconnect()
        await self.event_listener.disconnect()
        logger.info("Validation service stopped")
    
    async def handle_validate(self, data: Dict) -> Dict:
        """Handle validation requests by routing to appropriate validation function."""
        try:
            function = data.get("function")
            params = data.get("params", {})
            
            if function not in self.validators:
                logger.warning(f"Unknown validation function requested: {function}")
                return {
                    "error": f"No such validation function '{function}'", 
                    "passed": False,
                    "available_functions": list(self.validators.keys())
                }
            
            # Execute validation function
            result = await self.validators[function](params)
            
            # Publish completion event
            await self.rabbitmq_client.send_event("validation.completed", {
                "function": function,
                "result": result,
                "params": params,
                "timestamp": datetime.now().isoformat()
            })
            
            return result
            
        except Exception as e:
            logger.error(f"Error in validation function '{function}': {e}", exc_info=True)
            return {
                "error": f"Validation function error: {str(e)}", 
                "passed": False,
                "function": function,
                "timestamp": datetime.now().isoformat()
            }
    
    async def handle_health(self, data: Dict) -> Dict:
        """Handle health check requests."""
        return {
            "status": "healthy",
            "service": "validation",
            "timestamp": datetime.now().isoformat(),
            "loaded_validators": len(self.validators),
            "available_functions": list(self.validators.keys())
        }
    
    async def handle_inventory_status(self, data: Dict) -> Dict:
        """Handle inventory status requests."""
        try:
            ingredient = data.get("ingredient")
            inventory_levels = get_inventory_levels()
            
            if ingredient:
                if ingredient in inventory_levels:
                    return {
                        "ingredient": ingredient,
                        "status": inventory_levels[ingredient],
                        "success": True
                    }
                else:
                    return {"error": f"Unknown ingredient: {ingredient}", "success": False}
            else:
                # Format inventory data for API response
                formatted_inventory = {}
                for ing, data in inventory_levels.items():
                    level = data["level"]
                    threshold_low = data["threshold_low"]
                    threshold_medium = data["threshold_medium"]
                    
                    if level <= threshold_low:
                        level_status = "low"
                    elif level <= threshold_medium:
                        level_status = "medium"
                    else:
                        level_status = "high"
                    
                    formatted_inventory[ing] = {
                        "level": level_status,
                        "numeric": level,
                        "last_refilled": data["last_refilled"]
                    }
                
                return {
                    "inventory": formatted_inventory,
                    "success": True
                }
        except Exception as e:
            logger.error(f"Error getting inventory status: {e}")
            return {"error": str(e), "success": False}
    
    async def handle_inventory_refill(self, data: Dict) -> Dict:
        """Handle inventory refill requests."""
        try:
            ingredient = data.get("ingredient")
            amount = data.get("amount", 100)
            
            if not set_inventory_level(ingredient, amount):
                return {"error": f"Unknown ingredient: {ingredient}", "success": False}
            
            # Publish refill event
            await self.rabbitmq_client.send_event("inventory.refilled", {
                "ingredient": ingredient,
                "new_level": amount,
                "timestamp": datetime.now().isoformat()
            })
            
            # Check thresholds
            await self._check_inventory_thresholds()
            
            return {
                "ingredient": ingredient,
                "new_level": amount,
                "success": True
            }
            
        except Exception as e:
            logger.error(f"Error refilling inventory: {e}")
            return {"error": str(e), "success": False}
    
    async def handle_inventory_category_summary(self, data: Dict) -> Dict:
        """Handle inventory category summary requests."""
        try:
            summary = get_category_summary()
            
            return {
                "category_summary": summary,
                "success": True
            }
            
        except Exception as e:
            logger.error(f"Error getting inventory category summary: {e}")
            return {"error": str(e), "success": False}
    
    async def handle_refill_event(self, data: Dict):
        """Handle inventory refill events from other services."""
        ingredient = data.get("ingredient")
        amount = data.get("amount", 100)
        
        if ingredient:
            await self.handle_inventory_refill({"ingredient": ingredient, "amount": amount})
    
    async def _check_inventory_thresholds(self):
        """Check inventory thresholds and send warnings if needed."""
        inventory_levels = get_inventory_levels()
        
        for ingredient, info in inventory_levels.items():
            level = info["level"]
            threshold_low = info["threshold_low"]
            threshold_medium = info["threshold_medium"]
            
            if level <= threshold_low:
                await self.rabbitmq_client.send_event("validation.threshold_warning", {
                    "ingredient": ingredient,
                    "level": level,
                    "threshold": "low",
                    "severity": "high",
                    "timestamp": datetime.now().isoformat()
                })
            elif level <= threshold_medium:
                await self.rabbitmq_client.send_event("validation.threshold_warning", {
                    "ingredient": ingredient,
                    "level": level,
                    "threshold": "medium",
                    "severity": "medium",
                    "timestamp": datetime.now().isoformat()
                })

async def main():
    """Main service entry point."""
    service = ValidationService()
    await service.start()

if __name__ == "__main__":
    asyncio.run(main())
