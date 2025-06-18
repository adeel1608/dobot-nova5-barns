import asyncio
import json
import logging
import uuid
from typing import Dict, Any, Callable, Optional
from datetime import datetime, timedelta
import aio_pika
from aio_pika import Message, DeliveryMode, ExchangeType
from aio_pika.abc import AbstractIncomingMessage
import os

logger = logging.getLogger(__name__)

class RabbitMQClient:
    def __init__(self, service_name: str):
        self.service_name = service_name
        self.connection = None
        self.channel = None
        self.exchange = None
        self.response_queue = None
        self.pending_requests = {}
        self.message_handlers = {}
        self.rabbitmq_url = os.getenv("RABBITMQ_URL", "amqp://guest:guest@localhost:5672/")
        
    async def connect(self):
        """Establish connection to RabbitMQ"""
        try:
            self.connection = await aio_pika.connect_robust(self.rabbitmq_url)
            self.channel = await self.connection.channel()
            
            # Declare main exchange for service communication
            self.exchange = await self.channel.declare_exchange(
                "barns_services", ExchangeType.TOPIC, durable=True
            )
            
            # Create response queue for RPC-style communication
            self.response_queue = await self.channel.declare_queue(
                f"{self.service_name}_responses", exclusive=True, auto_delete=True
            )
            await self.response_queue.consume(self._handle_response)
            
            # Create service-specific queue for incoming requests
            service_queue = await self.channel.declare_queue(
                f"{self.service_name}_requests", durable=True
            )
            await service_queue.bind(self.exchange, f"{self.service_name}.*")
            await service_queue.consume(self._handle_request)
            
            logger.info(f"RabbitMQ connected for service: {self.service_name}")
            
        except Exception as e:
            logger.error(f"Failed to connect to RabbitMQ: {e}")
            raise
    
    async def disconnect(self):
        """Close RabbitMQ connection"""
        if self.connection:
            await self.connection.close()
            logger.info(f"RabbitMQ disconnected for service: {self.service_name}")
    
    def register_handler(self, action: str, handler: Callable):
        """Register a message handler for a specific action"""
        self.message_handlers[action] = handler
        logger.info(f"Registered handler for action: {action}")
    
    async def send_request(self, target_service: str, action: str, data: Dict[Any, Any], timeout: int = 30) -> Dict[Any, Any]:
        """Send a request to another service and wait for response"""
        correlation_id = str(uuid.uuid4())
        routing_key = f"{target_service}.{action}"
        
        message_body = {
            "action": action,
            "data": data,
            "timestamp": datetime.now().isoformat(),
            "source_service": self.service_name
        }
        
        message = Message(
            json.dumps(message_body).encode(),
            correlation_id=correlation_id,
            reply_to=self.response_queue.name,
            delivery_mode=DeliveryMode.PERSISTENT
        )
        
        # Store pending request
        future = asyncio.Future()
        self.pending_requests[correlation_id] = future
        
        try:
            logger.info(f"🚀 {self.service_name} sending request to {routing_key} with correlation_id: {correlation_id}")
            await self.exchange.publish(message, routing_key=routing_key)
            logger.info(f"📤 {self.service_name} published message to {routing_key}, waiting for response...")
            
            # Wait for response with timeout
            response = await asyncio.wait_for(future, timeout=timeout)
            logger.info(f"✅ {self.service_name} received response for {routing_key}: {response}")
            return response
            
        except asyncio.TimeoutError:
            logger.error(f"⏰ {self.service_name} request timeout for {routing_key} after {timeout}s")
            return {"error": "Request timeout", "success": False}
        except Exception as e:
            logger.error(f"💥 {self.service_name} failed to send request to {routing_key}: {e}")
            return {"error": str(e), "success": False}
        finally:
            # Clean up pending request
            self.pending_requests.pop(correlation_id, None)
    
    async def send_event(self, event_type: str, data: Dict[Any, Any]):
        """Send an event (fire-and-forget)"""
        routing_key = f"events.{event_type}"
        
        message_body = {
            "event_type": event_type,
            "data": data,
            "timestamp": datetime.now().isoformat(),
            "source_service": self.service_name
        }
        
        message = Message(
            json.dumps(message_body).encode(),
            delivery_mode=DeliveryMode.PERSISTENT
        )
        
        await self.exchange.publish(message, routing_key=routing_key)
    
    async def _handle_request(self, message: AbstractIncomingMessage):
        """Handle incoming requests"""
        async with message.process():
            try:
                body = json.loads(message.body.decode())
                action = body.get("action")
                data = body.get("data", {})
                source_service = body.get("source_service")
                
                if action in self.message_handlers:
                    # Execute handler
                    handler = self.message_handlers[action]
                    if asyncio.iscoroutinefunction(handler):
                        result = await handler(data)
                    else:
                        result = handler(data)
                    
                    
                    # Send response if reply_to is specified
                    if message.reply_to:
                        response_message = Message(
                            json.dumps(result).encode(),
                            correlation_id=message.correlation_id,
                            delivery_mode=DeliveryMode.PERSISTENT
                        )
                        await self.channel.default_exchange.publish(
                            response_message, routing_key=message.reply_to
                        )
                else:
                    logger.warning(f"❌ {self.service_name} no handler registered for action: {action}")
                    
                    # Send error response
                    if message.reply_to:
                        error_response = {
                            "error": f"No handler for action: {action}",
                            "success": False
                        }
                        response_message = Message(
                            json.dumps(error_response).encode(),
                            correlation_id=message.correlation_id,
                            delivery_mode=DeliveryMode.PERSISTENT
                        )
                        await self.channel.default_exchange.publish(
                            response_message, routing_key=message.reply_to
                        )
                        logger.info(f"❌ {self.service_name} sent error response for correlation_id: {message.correlation_id}")
                        
            except Exception as e:
                logger.error(f"💥 {self.service_name} error handling request: {e}")
                
                # Send error response
                if message.reply_to:
                    error_response = {
                        "error": str(e),
                        "success": False
                    }
                    response_message = Message(
                        json.dumps(error_response).encode(),
                        correlation_id=message.correlation_id,
                        delivery_mode=DeliveryMode.PERSISTENT
                    )
                    await self.channel.default_exchange.publish(
                        response_message, routing_key=message.reply_to
                    )
                    logger.info(f"💥 {self.service_name} sent error response for exception: {e}")
    
    async def _handle_response(self, message: AbstractIncomingMessage):
        """Handle incoming responses"""
        async with message.process():
            try:
                correlation_id = message.correlation_id
                if correlation_id in self.pending_requests:
                    response_data = json.loads(message.body.decode())
                    future = self.pending_requests[correlation_id]
                    if not future.done():
                        future.set_result(response_data)
                else:
                    logger.warning(f"Received response for unknown correlation_id: {correlation_id}")
                    
            except Exception as e:
                logger.error(f"Error handling response: {e}")

# Event listener for services that need to listen to events
class EventListener:
    def __init__(self, service_name: str):
        self.service_name = service_name
        self.connection = None
        self.channel = None
        self.exchange = None
        self.event_handlers = {}
        self.rabbitmq_url = os.getenv("RABBITMQ_URL", "amqp://guest:guest@localhost:5672/")
    
    async def connect(self):
        """Connect to RabbitMQ for event listening"""
        try:
            self.connection = await aio_pika.connect_robust(self.rabbitmq_url)
            self.channel = await self.connection.channel()
            
            self.exchange = await self.channel.declare_exchange(
                "barns_services", ExchangeType.TOPIC, durable=True
            )
            
            # Create event queue
            event_queue = await self.channel.declare_queue(
                f"{self.service_name}_events", durable=True
            )
            
            logger.info(f"Event listener connected for service: {self.service_name}")
            
        except Exception as e:
            logger.error(f"Failed to connect event listener: {e}")
            raise
    
    async def subscribe_to_events(self, event_patterns: list):
        """Subscribe to specific event patterns"""
        event_queue = await self.channel.declare_queue(
            f"{self.service_name}_events", durable=True
        )
        
        for pattern in event_patterns:
            await event_queue.bind(self.exchange, f"events.{pattern}")
            logger.info(f"Subscribed to events: {pattern}")
        
        await event_queue.consume(self._handle_event)
    
    def register_event_handler(self, event_type: str, handler: Callable):
        """Register an event handler"""
        self.event_handlers[event_type] = handler
        logger.info(f"Registered event handler for: {event_type}")
    
    async def _handle_event(self, message: AbstractIncomingMessage):
        """Handle incoming events"""
        async with message.process():
            try:
                body = json.loads(message.body.decode())
                event_type = body.get("event_type")
                data = body.get("data", {})
                
                if event_type in self.event_handlers:
                    handler = self.event_handlers[event_type]
                    if asyncio.iscoroutinefunction(handler):
                        await handler(data)
                    else:
                        handler(data)
                        
            except Exception as e:
                logger.error(f"Error handling event: {e}")
    
    async def disconnect(self):
        """Close connection"""
        if self.connection:
            await self.connection.close()
            logger.info(f"Event listener disconnected for service: {self.service_name}") 