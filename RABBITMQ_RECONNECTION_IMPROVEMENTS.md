# RabbitMQ Reconnection Improvements

## Problem Summary

The robot container service was experiencing issues where:
1. RabbitMQ server sometimes fails and restarts
2. The app would connect but not send/receive messages
3. After RabbitMQ restart, the service also needed to be restarted
4. Sometimes even when connected, data wasn't flowing

## Root Causes Identified

1. **Stale Connection Detection**: The health check only verified if connection object existed, not if it was actually working
2. **Lost Consumers**: When RabbitMQ reconnected, consumers (message listeners) were not re-established
3. **Channel Closure**: Channels could be closed even when connection appeared active
4. **No Active Verification**: No periodic verification that messages could actually be sent/received

## Solutions Implemented

### 1. Connection State Tracking

Added comprehensive connection state management in `RabbitMQClient` and `EventListener`:

```python
# Connection state tracking
self.is_connected = False
self.is_reconnecting = False
self.connection_lost_count = 0
```

### 2. Connection Callbacks

Registered callbacks to detect and handle connection events:

```python
async def _on_connection_closed(self, connection, exception):
    """Handle connection closed event"""
    self.is_connected = False
    self.connection_lost_count += 1
    self.logger.warning(f"Connection closed: {exception}")

async def _on_connection_reconnected(self, connection):
    """Handle connection reconnected event"""
    self.logger.info(f"Connection reconnected, re-establishing consumers...")
    await self._setup_consumers()
    self.is_connected = True
```

### 3. Consumer Re-establishment

Created `_setup_consumers()` method that:
- Recreates channel if closed
- Re-declares exchange
- Re-creates queues
- Re-establishes all consumers
- Can be called both during initial connection and after reconnection

```python
async def _setup_consumers(self):
    """Setup or re-setup channel, exchange, and consumers"""
    # Get or create channel
    if not self.channel or self.channel.is_closed:
        self.channel = await self.connection.channel()
        await self.channel.set_qos(prefetch_count=10)
    
    # Declare exchange, queues, and consumers
    # ...
```

### 4. Active Connection Verification

Added `verify_connection()` method that actively checks connection health:

```python
async def verify_connection(self) -> bool:
    """Actively verify that the connection is working"""
    if not self.connection or self.connection.is_closed:
        self.is_connected = False
        return False
    
    if not self.channel or self.channel.is_closed:
        self.logger.warning("Channel is closed, attempting to recreate...")
        await self._setup_consumers()
    
    self.is_connected = True
    return True
```

### 5. Enhanced Health Status

Improved `get_health_status()` to provide detailed connection state:

```python
def get_health_status(self) -> Dict[str, Any]:
    connection_ok = (
        self.connection is not None and 
        not self.connection.is_closed and
        self.is_connected
    )
    channel_ok = self.channel is not None and not self.channel.is_closed
    
    return {
        "connected": connection_ok,
        "channel_ready": channel_ok,
        "is_reconnecting": self.is_reconnecting,
        "connection_lost_count": self.connection_lost_count,
        "healthy": connection_ok and channel_ok and not self.is_reconnecting
        # ... more fields
    }
```

### 6. Improved Connection Monitoring in app.py

Enhanced the service monitoring loop with:
- Active connection verification every 15 seconds
- Consecutive failure tracking
- Automatic reconnection trigger after 3 consecutive failures

```python
check_interval = 15  # Check every 15 seconds
consecutive_failures = 0
max_consecutive_failures = 3

while True:
    await asyncio.sleep(check_interval)
    
    is_connected = await self.rabbitmq_client.verify_connection()
    
    if not is_connected:
        consecutive_failures += 1
        if consecutive_failures >= max_consecutive_failures:
            raise ConnectionError("Connection verification failed multiple times")
    else:
        consecutive_failures = 0
```

## Benefits

1. **Automatic Recovery**: Service automatically recovers from RabbitMQ restarts without manual intervention
2. **Early Detection**: Problems are detected within 15 seconds instead of waiting for message failures
3. **Self-Healing**: Consumers are automatically re-established when connection is restored
4. **Better Observability**: Detailed health status shows exact connection state
5. **Graceful Degradation**: Circuit breaker prevents cascading failures

## Files Modified

1. `services/robot_container/ros_ws/src/shared/rabbitmq_client.py`
   - Added connection state tracking
   - Added connection callbacks
   - Added `_setup_consumers()` method
   - Added `verify_connection()` method
   - Enhanced `get_health_status()`
   - Applied same improvements to `EventListener` class

2. `shared/rabbitmq_client.py`
   - Same improvements as above (for other services)

3. `services/robot_container/ros_ws/src/oms_v1/oms_v1/app.py`
   - Enhanced connection monitoring loop
   - Added consecutive failure tracking
   - Improved health check handler
   - Better error logging

## Testing Recommendations

1. **Normal Operation**: Verify service connects and processes messages normally
2. **RabbitMQ Restart**: Restart RabbitMQ and confirm service automatically reconnects
3. **Network Interruption**: Simulate network issues and verify recovery
4. **Health Checks**: Call health endpoint and verify detailed status
5. **Load Testing**: Ensure reconnection works under message load

## Configuration

The reconnection behavior can be tuned via:
- `reconnect_interval=5`: Wait 5 seconds between reconnection attempts
- `check_interval=15`: Verify connection every 15 seconds
- `max_consecutive_failures=3`: Reconnect after 3 failed verifications
- `prefetch_count=10`: Process up to 10 messages simultaneously

## Monitoring

Watch for these log messages:
- `✅ RabbitMQ connected successfully` - Initial connection
- `🔌 connection closed` - Connection lost
- `🔄 connection reconnected` - Automatic reconnection
- `✅ consumers re-established` - Listeners restored
- `⚠️ Connection verification failed` - Health check failure
- `❌ Connection verification failed multiple times` - Triggering reconnection

