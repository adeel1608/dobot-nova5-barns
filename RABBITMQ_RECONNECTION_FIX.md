# RabbitMQ Reconnection Fix

## Problem

When the RabbitMQ container restarted, services would appear to reconnect but would not receive any messages. The connection looked healthy but no messages were being processed.

## Root Cause

The issue had TWO main problems:

### 1. Missing Consumer Re-establishment

The `RabbitMQClient` was using `aio_pika.connect_robust()` which automatically reconnects the TCP connection when RabbitMQ restarts. However, **the message consumers were not being re-established** after reconnection.

When RabbitMQ restarted:
- The connection was automatically re-established by `connect_robust`
- The channel appeared to be open
- BUT the `.consume()` callbacks were NOT re-run
- Result: Messages were published to queues but no consumer was listening

### 2. Multiple Copies of Shared Module

There were **two copies** of the `rabbitmq_client.py` file:
- `/home/qss/BARNS/shared/rabbitmq_client.py` - Used by most services
- `/home/qss/BARNS/services/robot_container/ros_ws/src/shared/rabbitmq_client.py` - Used by robot container

Initially only the first copy was updated, so the robot container continued to use the old code without the fix.

## Solution

### Changes to RabbitMQClient

1. **Added reconnection callback registration**:
   ```python
   self.connection.reconnect_callbacks.add(self._on_reconnect)
   ```

2. **Extracted channel setup to separate method** `_setup_channel()`:
   - Creates channel
   - Declares exchange
   - Declares queues
   - **Binds queues to exchange**
   - **Re-establishes consumers with `.consume()` calls**

3. **Added reconnection callback** `_on_reconnect()`:
   - Called automatically when connection is re-established
   - Calls `_setup_channel()` to recreate everything
   - Logs success/failure of re-establishment

### Changes to EventListener

Similar changes were made to the `EventListener` class:

1. Added reconnection callback registration
2. Created `_setup_channel()` method
3. Added `_on_reconnect()` callback
4. Stored `event_patterns` to re-subscribe after reconnection

### Changes to app.py

Enhanced the health check in the robot container service to verify:
- RabbitMQ client exists
- Connection is not closed
- Channel is not closed
- Exchange exists
- Response queue exists

## Files Modified

1. `/home/qss/BARNS/shared/rabbitmq_client.py`
2. `/home/qss/BARNS/services/robot_container/ros_ws/src/shared/rabbitmq_client.py`
3. `/home/qss/BARNS/services/robot_container/ros_ws/src/oms_v1/oms_v1/app.py`

## Testing

To verify the fix:

1. Start all services: `docker-compose -f docker-compose.dev.yml up`
2. Verify services are connected and receiving messages
3. Restart RabbitMQ: `docker restart barns-rabbitmq`
4. Wait for reconnection (look for "Channel and consumers re-established successfully" logs)
5. Send a test message - it should be received and processed

## Key Takeaways

1. **`connect_robust` alone is not enough** - it reconnects the TCP connection but doesn't restore application-level state like consumers
2. **Use reconnection callbacks** - `aio_pika` provides `reconnect_callbacks` to handle re-initialization
3. **Check for duplicate modules** - Multiple copies of shared code can cause confusion
4. **Test failure scenarios** - Always test what happens when dependencies restart

## Connection Parameters

The fix also added explicit parameters to `connect_robust()`:
- `reconnect_interval=5` - Try to reconnect every 5 seconds
- `fail_fast=False` - Keep trying to reconnect indefinitely

These ensure the service never gives up trying to reconnect to RabbitMQ.

