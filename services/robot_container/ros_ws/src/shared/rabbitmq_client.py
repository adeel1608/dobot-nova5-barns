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
import time
from enum import Enum

import sys
sys.tracebacklimit = 0

# logger = logging.getLogger(__name__)

class CircuitState(Enum):
    CLOSED = "closed"      # Normal operation
    OPEN = "open"          # Circuit breaker open, reject requests
    HALF_OPEN = "half_open"  # Testing if service is back

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
        
        # Circuit breaker configuration
        self.circuit_state = CircuitState.CLOSED
        self.failure_count = 0
        self.last_failure_time = 0
        self.circuit_open_time = 30  # seconds to keep circuit open
        self.failure_threshold = 5   # failures before opening circuit
        self.success_threshold = 3   # successes to close circuit
        self.success_count = 0
        
        # Retry configuration
        self.max_retries = 3
        self.retry_delay = 1.0  # seconds
        self.retry_backoff = 2.0  # exponential backoff multiplier

        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(self.__class__.__name__)

        # Silence Pika's verbose DEBUG logs
        logging.getLogger('pika').setLevel(logging.WARNING)

        # ADD THESE LINES to silence aio_pika debug logs
        logging.getLogger('aio_pika').setLevel(logging.WARNING)
        logging.getLogger('aiormq').setLevel(logging.WARNING)
        logging.getLogger('aiormq.connection').setLevel(logging.WARNING)
    
    def _should_allow_request(self) -> bool:
        """Check if circuit breaker allows the request"""
        if self.circuit_state == CircuitState.CLOSED:
            return True
        
        if self.circuit_state == CircuitState.OPEN:
            # Check if enough time has passed to try half-open
            if time.time() - self.last_failure_time > self.circuit_open_time:
                self.circuit_state = CircuitState.HALF_OPEN
                self.logger.info(f"🔄 {self.service_name} circuit breaker transitioning to HALF_OPEN")
                return True
            return False
        
        # HALF_OPEN state - allow limited requests
        return True
    
    def _record_success(self):
        """Record a successful operation"""
        self.failure_count = 0
        if self.circuit_state == CircuitState.HALF_OPEN:
            self.success_count += 1
            if self.success_count >= self.success_threshold:
                self.circuit_state = CircuitState.CLOSED
                self.success_count = 0
                self.logger.info(f"✅ {self.service_name} circuit breaker CLOSED - service recovered")
    
    def _record_failure(self):
        """Record a failed operation"""
        self.failure_count += 1
        self.last_failure_time = time.time()
        self.success_count = 0
        
        if self.failure_count >= self.failure_threshold and self.circuit_state == CircuitState.CLOSED:
            self.circuit_state = CircuitState.OPEN
            self.logger.warning(f"🚨 {self.service_name} circuit breaker OPEN - too many failures")
        
    async def _retry_operation(self, operation, *args, **kwargs):
        """Retry an operation with exponential backoff"""
        last_exception = None
        delay = self.retry_delay
        
        for attempt in range(self.max_retries + 1):
            try:
                if not self._should_allow_request():
                    raise Exception(f"Circuit breaker is {self.circuit_state.value}")
                
                result = await operation(*args, **kwargs)
                self._record_success()
                return result
                
            except Exception as e:
                last_exception = e
                self._record_failure()
                
                if attempt < self.max_retries:
                    self.logger.warning(f"⚠️ {self.service_name} operation failed (attempt {attempt + 1}/{self.max_retries + 1}): {e}")
                    await asyncio.sleep(delay)
                    delay *= self.retry_backoff
                else:
                    self.logger.error(f"❌ {self.service_name} operation failed after {self.max_retries + 1} attempts: {e}")
        
        raise last_exception
        
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
                f"{self.service_name}_responses", durable=True, auto_delete=False
            )
            await self.response_queue.consume(self._handle_response)
            
            # Create service-specific queue for incoming requests
            service_queue = await self.channel.declare_queue(
                f"{self.service_name}_requests", durable=True
            )
            await service_queue.bind(self.exchange, f"{self.service_name}.*")
            await service_queue.consume(self._handle_request)
            
            self.logger.info(f"RabbitMQ connected for service: {self.service_name}")
            
        except Exception as e:
            self.logger.error(f"Failed to connect to RabbitMQ: {e}")
            raise
    
    async def disconnect(self):
        """Close RabbitMQ connection"""
        if self.connection:
            await self.connection.close()
            self.logger.info(f"RabbitMQ disconnected for service: {self.service_name}")
    
    def register_handler(self, action: str, handler: Callable):
        """Register a message handler for a specific action"""
        self.message_handlers[action] = handler
        self.logger.info(f"Registered handler for action: {action}")
    
    async def send_request(self, target_service: str, action: str, data: Dict[Any, Any], timeout: int = 30) -> Dict[Any, Any]:
        """Send a request to another service and wait for response with retry logic"""
        
        async def _send_request_operation():
            correlation_id = str(uuid.uuid4())
            routing_key = f"{target_service}.{action}"
            
            message_body = {
                "action": action,
                "data": data,
                "timestamp": datetime.now().isoformat(),
                "source_service": self.service_name,
            }

            #val added this block to handle the validation service
            if target_service == "validation":
                message_body = {
                "function_name": action,
                "payload": data,
                "timestamp": datetime.now().isoformat(),
                "client_type": self.service_name,
                "request_id": correlation_id,
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
                self.logger.info(f"🚀 {self.service_name} sending request to {routing_key} with correlation_id: {correlation_id}")
                await self.exchange.publish(message, routing_key=routing_key)
                self.logger.info(f"📤 {self.service_name} published message to {routing_key}, waiting for response...")
                
                # Wait for response with timeout
                response = await asyncio.wait_for(future, timeout=timeout)
                self.logger.info(f"✅ {self.service_name} received response for {routing_key}: {response}")
                return response
                
            except asyncio.TimeoutError:
                self.logger.error(f"⏰ {self.service_name} request timeout for {routing_key} after {timeout}s")
                return {"error": "Request timeout", "success": False}
            except Exception as e:
                self.logger.error(f"💥 {self.service_name} failed to send request to {routing_key}: {e}")
                return {"error": str(e), "success": False}
            finally:
                # Clean up pending request
                self.pending_requests.pop(correlation_id, None)
        
        return await self._retry_operation(_send_request_operation)
    
    async def send_event(self, event_type: str, data: Dict[Any, Any]):
        """Send an event (fire-and-forget) with retry logic"""
        
        async def _send_event_operation():
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
            self.logger.info(f"📡 {self.service_name} sent event: {event_type}")
        
        return await self._retry_operation(_send_event_operation)
    
    async def send_event_with_ack(self, event_type: str, data: Dict[Any, Any], timeout: float = 10.0):
        """Send an event and wait for acknowledgment (for critical events)"""
        
        async def _send_event_with_ack_operation():
            routing_key = f"events.{event_type}"
            correlation_id = str(uuid.uuid4())
            
            message_body = {
                "event_type": event_type,
                "data": data,
                "timestamp": datetime.now().isoformat(),
                "source_service": self.service_name,
                "correlation_id": correlation_id
            }
            
            message = Message(
                json.dumps(message_body).encode(),
                correlation_id=correlation_id,
                reply_to=self.response_queue.name,
                delivery_mode=DeliveryMode.PERSISTENT
            )
            
            # Store pending acknowledgment
            future = asyncio.Future()
            self.pending_requests[correlation_id] = future
            
            try:
                await self.exchange.publish(message, routing_key=routing_key)
                self.logger.info(f"📡 {self.service_name} sent critical event: {event_type}")
                
                # Wait for acknowledgment
                ack = await asyncio.wait_for(future, timeout=timeout)
                self.logger.info(f"✅ {self.service_name} received ack for event: {event_type}")
                return ack
                
            except asyncio.TimeoutError:
                self.logger.error(f"⏰ {self.service_name} event ack timeout for {event_type}")
                return {"error": "Event acknowledgment timeout", "success": False}
            finally:
                self.pending_requests.pop(correlation_id, None)
        
        return await self._retry_operation(_send_event_with_ack_operation)
    
    def get_health_status(self) -> Dict[str, Any]:
        """Get the health status of the RabbitMQ client"""
        return {
            "service_name": self.service_name,
            "connected": self.connection is not None and not self.connection.is_closed,
            "circuit_state": self.circuit_state.value,
            "failure_count": self.failure_count,
            "success_count": self.success_count,
            "pending_requests": len(self.pending_requests),
            "last_failure_time": self.last_failure_time
        }
    
    async def _handle_request(self, message: AbstractIncomingMessage):
        """Handle incoming requests"""
        async with message.process():
            try:
                body = json.loads(message.body.decode())
                action = ""
                data = {}
                source_service = ""


                if self.service_name == "validation":
                    action = body.get("function_name")
                    data = body.get("payload", {})
                    source_service = body.get("client_type")
                else:
                    action = body.get("action")
                    data = body.get("data", {})
                    source_service = body.get("source_service")
                
                if action in self.message_handlers:
                    # Execute handler
                    handler = self.message_handlers[action]
                    handler_input = {}
                    
                    #val If this is the validation service, pass the complete body instead of just data
                    if self.service_name == "validation":
                        handler_input = body
                    else:
                        handler_input = data
                    
                    #val used handler_input instead of data 
                    if asyncio.iscoroutinefunction(handler):
                        result = await handler(handler_input)
                    else:
                        result = handler(handler_input)
                    
                    
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
                    self.logger.warning(f"❌ {self.service_name} no handler registered for action: {action}")
                    
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
                        self.logger.info(f"❌ {self.service_name} sent error response for correlation_id: {message.correlation_id}")
                        
            except Exception as e:
                self.logger.error(f"💥 {self.service_name} error handling request: {e}")
                
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
                    self.logger.info(f"💥 {self.service_name} sent error response for exception: {e}")
    
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
                    self.logger.warning(f"Received response for unknown correlation_id: {correlation_id}")
                    
            except Exception as e:
                self.logger.error(f"Error handling response: {e}")

# Event listener for services that need to listen to events
class EventListener:
    def __init__(self, service_name: str):
        self.service_name = service_name
        self.connection = None
        self.channel = None
        self.exchange = None
        self.event_handlers = {}
        self.rabbitmq_url = os.getenv("RABBITMQ_URL", "amqp://guest:guest@localhost:5672/")

        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(self.__class__.__name__)

        # Silence Pika's verbose DEBUG logs
        logging.getLogger('pika').setLevel(logging.WARNING)

        # ADD THESE LINES to silence aio_pika debug logs
        logging.getLogger('aio_pika').setLevel(logging.WARNING)
        logging.getLogger('aiormq').setLevel(logging.WARNING)
        logging.getLogger('aiormq.connection').setLevel(logging.WARNING)
    
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
            
            self.logger.info(f"Event listener connected for service: {self.service_name}")
            
        except Exception as e:
            self.logger.error(f"Failed to connect event listener: {e}")
            raise
    
    async def subscribe_to_events(self, event_patterns: list):
        """Subscribe to specific event patterns"""
        event_queue = await self.channel.declare_queue(
            f"{self.service_name}_events", durable=True
        )
        
        for pattern in event_patterns:
            routing_key_pattern = f"events.{pattern}"
            await event_queue.bind(self.exchange, routing_key_pattern)
            self.logger.info(f"📡 {self.service_name} subscribed to events: {pattern} (routing_key: {routing_key_pattern})")
        
        self.logger.info(f"📡 {self.service_name} consuming events from queue: {event_queue.name}")
        await event_queue.consume(self._handle_event)
    
    def register_event_handler(self, event_type: str, handler: Callable):
        """Register an event handler"""
        self.event_handlers[event_type] = handler
        self.logger.info(f"Registered event handler for: {event_type}")
    
    async def _handle_event(self, message: AbstractIncomingMessage):
        """Handle incoming events"""
        async with message.process():
            try:
                body = json.loads(message.body.decode())
                event_type = body.get("event_type")
                data = body.get("data", {})
                
                # Log all received events for debugging
                routing_key = message.routing_key if hasattr(message, 'routing_key') else 'unknown'
                self.logger.info(f"📨 {self.service_name} received event: {event_type} (routing_key: {routing_key})")
                self.logger.debug(f"Event data: {data}")
                self.logger.debug(f"Registered handlers: {list(self.event_handlers.keys())}")
                
                result = None
                handler_error = None
                
                if event_type in self.event_handlers:
                    handler = self.event_handlers[event_type]
                    self.logger.info(f"✅ {self.service_name} found handler for {event_type}, calling handler...")
                    try:
                        if asyncio.iscoroutinefunction(handler):
                            result = await handler(data)
                        else:
                            result = handler(data)
                        self.logger.info(f"✅ {self.service_name} handler for {event_type} completed successfully")
                    except Exception as handler_ex:
                        handler_error = handler_ex
                        self.logger.error(f"❌ Error in event handler for {event_type}: {handler_ex}")
                else:
                    self.logger.warning(f"⚠️ {self.service_name} received event {event_type} but no handler registered for it")
                    self.logger.warning(f"Available handlers: {list(self.event_handlers.keys())}")
                
                # Send acknowledgment if reply_to is specified (for send_event_with_ack)
                if message.reply_to and message.correlation_id:
                    if handler_error:
                        # Send error acknowledgment
                        ack_response = {
                            "success": False,
                            "acknowledged": False,
                            "error": str(handler_error)
                        }
                    elif result is not None:
                        # Send the result from the handler as acknowledgment
                        ack_response = result if isinstance(result, dict) else {"success": True, "acknowledged": True}
                    else:
                        # Send default success acknowledgment
                        ack_response = {"success": True, "acknowledged": True}
                    
                    ack_message = Message(
                        json.dumps(ack_response).encode(),
                        correlation_id=message.correlation_id,
                        delivery_mode=DeliveryMode.PERSISTENT
                    )
                    await self.channel.default_exchange.publish(
                        ack_message, routing_key=message.reply_to
                    )
                    self.logger.info(f"📨 {self.service_name} sent acknowledgment for event {event_type}")
                        
            except Exception as e:
                self.logger.error(f"Error handling event: {e}")
                
                # Try to send error acknowledgment if possible
                if message.reply_to and message.correlation_id:
                    try:
                        error_ack = {
                            "success": False,
                            "acknowledged": False,
                            "error": str(e)
                        }
                        ack_message = Message(
                            json.dumps(error_ack).encode(),
                            correlation_id=message.correlation_id,
                            delivery_mode=DeliveryMode.PERSISTENT
                        )
                        await self.channel.default_exchange.publish(
                            ack_message, routing_key=message.reply_to
                        )
                        self.logger.info(f"📨 {self.service_name} sent error acknowledgment")
                    except Exception as ack_error:
                        self.logger.error(f"Failed to send error acknowledgment: {ack_error}")
    
    async def disconnect(self):
        """Close connection"""
        if self.connection:
            await self.connection.close()
            self.logger.info(f"Event listener disconnected for service: {self.service_name}")
