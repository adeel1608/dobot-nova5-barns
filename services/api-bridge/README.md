# API Bridge Service

## Brief Overview

The API Bridge Service acts as the primary HTTP gateway between the BARNS Dashboard and all backend microservices. It translates synchronous HTTP requests into asynchronous RabbitMQ messages, manages real-time WebSocket/Socket.IO connections for live updates, and provides a unified REST API interface for the entire system.

## Key Features

- **HTTP to RabbitMQ Translation**: Converts REST API calls to RabbitMQ RPC messages
- **Dual Real-Time Support**: WebSocket and Socket.IO for dashboard updates
- **Event Broadcasting**: Receives RabbitMQ events and pushes to connected clients
- **Unified API Gateway**: Single entry point for dashboard to access all services
- **Request/Response Correlation**: Handles async message correlation with timeouts
- **CORS-Enabled**: Configured for local and production dashboard access
- **Comprehensive API**: Orders, inventory, recipes, POS integration, alerts, system control

## Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    BARNS Dashboard                         │
│              (React Web Application)                       │
└───────────────┬──────────────┬─────────────────────────────┘
                │              │
         HTTP/REST        WebSocket/Socket.IO
                │              │ (Real-time updates)
                ↓              ↓
┌───────────────────────────────────────────────────────────┐
│              API Bridge Service (FastAPI)                 │
│                                                            │
│  ┌─────────────────────┐      ┌──────────────────────┐   │
│  │  HTTP Endpoints     │      │  Event Listener      │   │
│  │  /api/orders        │      │  - Order events      │   │
│  │  /api/inventory     │      │  - Inventory events  │   │
│  │  /api/system        │      │  - Scheduler events  │   │
│  │  /api/alerts        │      │  - Validation events │   │
│  └──────────┬──────────┘      └─────────┬────────────┘   │
│             │                           │                 │
│             ↓                           ↓                 │
│  ┌──────────────────────────────────────────────────┐    │
│  │        RabbitMQ Client (Request/Response)        │    │
│  └──────────────────────────────────────────────────┘    │
└───────────────┬───────────────────────┬────────────────  ───┘
                │                       │
                ↓                       ↓
       ┌────────────────┐      ┌─────────────────┐
       │   RabbitMQ     │      │   RabbitMQ      │
       │   RPC Queue    │      │   Event Exchange│
       └────────┬───────┘      └────────┬────────┘
                │                       │
     ┌──────────┼───────────┬───────────┼──────────┐
     ↓          ↓           ↓           ↓          ↓
┌────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐ ┌──────────┐
│  OMS   │ │Scheduler │ │Validation│ │Routine │ │Automation│
└────────┘ └──────────┘ └──────────┘ └────────┘ └──────────┘
```

### Communication Flow

1. **Inbound HTTP Requests**:
   - Dashboard → HTTP POST/GET → API Bridge
   - API Bridge → RabbitMQ RPC → Target Service
   - Target Service → RabbitMQ Response → API Bridge
   - API Bridge → HTTP Response → Dashboard

2. **Outbound Real-time Events**:
   - Service → RabbitMQ Event Publish → Event Exchange
   - API Bridge subscribes → Receives Event
   - API Bridge → WebSocket/Socket.IO → Dashboard (real-time update)

## Setup & Installation

### Prerequisites

- Python 3.8+
- RabbitMQ server running
- Access to BARNS network
- Docker (for containerized deployment)

### Local Development

```bash
# Navigate to service directory
cd services/api-bridge

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export RABBITMQ_URL="amqp://admin:admin123@localhost:5672/"
export PYTHONPATH="/path/to/barns"

# Run service
uvicorn app:app --host 0.0.0.0 --port 8000 --reload
```

### Docker Deployment

Automatically deployed via `docker-compose.yml`:

```bash
# Start API Bridge and dependencies
docker-compose up -d rabbitmq api-bridge

# View logs
docker-compose logs -f api-bridge

# Access API documentation
curl http://localhost:8000/docs
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `RABBITMQ_URL` | `amqp://admin:admin123@rabbitmq:5672/` | RabbitMQ connection string |
| `PYTHONPATH` | `/app` | Python module search path |

### CORS Configuration

Configured in `app.py` to allow dashboard origins:

```python
allow_origins=[
    "http://localhost:3000",      # React dev server
    "http://localhost:5173",      # Vite dev server
    "http://127.0.0.1:3000",
    "http://127.0.0.1:5173"
]
```

### Docker Network Configuration

```yaml
networks:
  - barns-network
ports:
  - "8000:8000"  # HTTP API and WebSocket
depends_on:
  - rabbitmq (must be healthy)
  - validation-service
  - routine-service
  - scheduler-service
```

## API/Endpoints

### Health & Status

#### GET /health, /api/health
Health check endpoint.

**Response:**
```json
{
  "status": "healthy",
  "service": "api_bridge",
  "timestamp": "2025-01-15T10:30:00"
}
```

### Order Management

#### POST /api/orders
Create a new order.

**Request:**
```json
{
  "cups": [
    {"recipe": "latte", "size": "medium"}
  ]
}
```

**Response:**
```json
{
  "success": true,
  "order_id": 123,
  "status": "pending",
  "timestamp": "2025-01-15T10:30:00"
}
```

#### GET /api/orders
List all orders with optional status filter.

**Query Parameters:**
- `status` (optional): Filter by status (pending, processing, completed, failed)

**Response:**
```json
{
  "success": true,
  "orders": [
    {
      "order_id": 123,
      "status": "processing",
      "cups": [...]
    }
  ]
}
```

#### GET /api/orders/{order_id}
Get specific order details.

#### PATCH /api/orders/{order_id}/start
Start processing an order.

#### PATCH /api/orders/{order_id}/status
Update order status.

**Request:**
```json
{
  "status": "completed",
  "reason": "Order fulfilled successfully"
}
```

#### DELETE /api/orders/{order_id}
Delete an order.

#### POST /api/orders/{order_id}/halt
Halt order with reason.

**Request:**
```json
{
  "reason": "Ingredient shortage"
}
```

#### POST /api/orders/{order_id}/stop
Emergency stop an order.

#### POST /api/orders/{order_id}/resume
Resume a halted order.

### Queue Management

#### GET /api/queue
Get current order queue state.

**Response:**
```json
{
  "success": true,
  "queue": [
    {"order_id": 123, "position": 1, "status": "processing"},
    {"order_id": 124, "position": 2, "status": "pending"}
  ]
}
```

#### PUT /api/queue/reorder
Reorder the queue.

**Request:**
```json
{
  "order": [124, 123, 125]
}
```

### Inventory Management

#### GET /api/inventory/status
Get inventory status (all, by type, or specific item).

**Query Parameters:**
- `ingredient_type` (optional): Filter by ingredient type
- `subtype` (optional): Filter by subtype

**Response:**
```json
{
  "success": true,
  "inventory": {
    "coffee_beans": {
      "regular": {
        "current_amount": 750,
        "status": "high",
        "warning_threshold": 300,
        "critical_threshold": 100
      }
    }
  },
  "timestamp": "2025-01-15T10:30:00"
}
```

#### POST /api/inventory/refill
Refill inventory (triggers CV detection for coffee beans).

**Query Parameters:**
- `ingredient_type` (optional): Specific ingredient to refill
- `subtype` (optional): Specific subtype to refill

**Response:**
```json
{
  "passed": true,
  "details": {
    "coffee_beans_message": "Refilled with 85% detected",
    "coffee_beans_percentage": 85
  }
}
```

#### GET /api/inventory/category-summary
Get lowest stock level per category.

**Response:**
```json
{
  "success": true,
  "summary": {
    "coffee_beans": "high",
    "milk": "medium",
    "syrups": "low"
  }
}
```

#### GET /api/inventory/stock-level
Get stock level statistics (counts per level).

**Response:**
```json
{
  "success": true,
  "stock_level": {
    "high": 5,
    "medium": 3,
    "low": 2,
    "empty": 0
  }
}
```

#### GET /api/inventory/by-stock-level/{stock_level}
Get inventory items filtered by stock level.

**Parameters:**
- `stock_level`: high, medium, low, or empty

**Response:**
```json
{
  "success": true,
  "stock_level": "low",
  "ingredients": {
    "syrups": {
      "vanilla": {"current_amount": 50, "critical_threshold": 100}
    }
  }
}
```

#### GET /api/inventory/category-info
Get category metadata (unit types, capacities).

#### GET /api/inventory/category-count
Get count of items per category.

### Recipe Management

#### GET /api/recipes
Get available recipes from recipes.json.

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "name": "latte",
      "display_name": "Latte",
      "steps": 5
    }
  ],
  "count": 10
}
```

### POS Integration

#### POST /api/pos/process-order
Proxy POS order to OMS service.

**Request:**
```json
{
  "transaction_id": "POS-001",
  "items": [
    {"product_id": "latte_medium", "quantity": 1}
  ]
}
```

#### GET /api/pos/menu-items
Get POS menu items from OMS.

#### GET /api/pos/ingredients
Get ingredient list for POS.

### System Management

#### GET /api/system/status
Get health status of all services.

**Response:**
```json
{
  "success": true,
  "services": {
    "oms": {"status": "healthy"},
    "scheduler": {"status": "healthy"},
    "validation": {"status": "healthy"}
  }
}
```

#### POST /api/system/stop
Emergency stop all operations.

**Request:**
```json
{
  "reason": "Emergency maintenance required"
}
```

#### POST /api/system/resume
Resume system operations after emergency stop.

### Alert Management

#### GET /api/alerts/active
Get active alerts.

**Response:**
```json
{
  "success": true,
  "alerts": [
    {
      "alert_id": 1,
      "type": "inventory_low",
      "message": "Coffee beans running low",
      "severity": "warning",
      "timestamp": "2025-01-15T10:00:00"
    }
  ]
}
```

#### GET /api/alerts/acknowledged
Get acknowledged alerts.

#### POST /api/alerts/{alert_id}/acknowledge
Acknowledge an alert.

### Real-Time WebSocket

#### WebSocket /ws
Real-time event streaming endpoint.

**Connection:**
```javascript
const ws = new WebSocket('ws://localhost:8000/ws');

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Event:', data.type, data);
};
```

**Event Types:**
- `connection`: Initial connection confirmation
- `order_update`: Order status changes
- `inventory_update`: Inventory level changes
- `pong`: Heartbeat response

**Ping/Pong:**
```javascript
ws.send(JSON.stringify({type: 'ping'}));
```

### Socket.IO (Alternative Real-Time)

#### GET /api/socketio/stats
Get Socket.IO connection statistics.

**Events:**
- `inventory.update.{category}`: Category-specific updates
- `inventory.summary`: Category summary updates
- `inventory.stock_level`: Stock level statistics
- `inventory.status`: Full inventory status

## Usage Examples

### Dashboard API Client

```javascript
// services/barns-dashboard/src/api/orders.js
import api from './base';

// Create order
const createOrder = async (order) => {
  const response = await api.post('/api/orders', order);
  return response.data;
};

// Start order
const startOrder = async (orderId) => {
  const response = await api.patch(`/api/orders/${orderId}/start`);
  return response.data;
};

// Get queue
const getQueue = async () => {
  const response = await api.get('/api/queue');
  return response.data;
};
```

### Real-Time Updates

```javascript
// WebSocket connection
const connectWebSocket = () => {
  const ws = new WebSocket('ws://localhost:8000/ws');
  
  ws.onopen = () => console.log('Connected to API Bridge');
  
  ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    
    switch(message.type) {
      case 'order_update':
        updateOrderUI(message.data);
        break;
      case 'inventory_update':
        updateInventoryUI(message.data);
        break;
    }
  };
  
  // Heartbeat
  setInterval(() => {
    ws.send(JSON.stringify({type: 'ping'}));
  }, 30000);
};
```

### Python Client

```python
import httpx
import asyncio

async def create_and_start_order():
    async with httpx.AsyncClient() as client:
        # Create order
        response = await client.post(
            "http://localhost:8000/api/orders",
            json={"cups": [{"recipe": "latte"}]}
        )
        order = response.json()
        order_id = order["order_id"]
        
        # Start order
        response = await client.patch(
            f"http://localhost:8000/api/orders/{order_id}/start"
        )
        return response.json()

asyncio.run(create_and_start_order())
```

### cURL Examples

```bash
# Create order
curl -X POST http://localhost:8000/api/orders \
  -H "Content-Type: application/json" \
  -d '{"cups": [{"recipe": "latte", "size": "medium"}]}'

# Get inventory status
curl http://localhost:8000/api/inventory/status | jq

# Refill coffee beans
curl -X POST 'http://localhost:8000/api/inventory/refill?ingredient_type=coffee_beans&subtype=regular'

# Get system status
curl http://localhost:8000/api/system/status | jq
```

## Dependencies

### Core Dependencies

- **FastAPI** (0.104.1): Web framework for HTTP API
- **Uvicorn** (0.24.0): ASGI server
- **Pydantic** (2.5.0): Data validation and serialization
- **aio-pika** (9.3.1): Async RabbitMQ client
- **pika** (1.3.2): Sync RabbitMQ client (fallback)
- **python-socketio** (5.13.0): Socket.IO support
- **httpx** (0.25.0): Async HTTP client for POS proxying

### Shared Modules

- `shared.rabbitmq_client`: RabbitMQ client and event listener

## Integration Points

### Downstream Services (Calls To)

1. **OMS Service**
   - Order CRUD operations
   - Queue management
   - Alert management
   - Emergency stop/resume
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 30 seconds

2. **Validation Service**
   - Inventory status queries
   - Inventory refill operations
   - Category summaries
   - Stock level statistics
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 30 seconds

3. **Scheduler Service**
   - Health checks
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 10 seconds

4. **Routine Service**
   - Health checks
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 10 seconds

5. **Automation Service**
   - Health checks
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 10 seconds

6. **OMS Direct HTTP** (POS Proxy)
   - `/pos/process-order`
   - `/pos/menu-items`
   - `/pos/ingredients`
   - **Protocol**: HTTP
   - **Port**: Internal 8000

### Upstream Services (Receives From)

1. **Dashboard**
   - All HTTP REST requests
   - WebSocket connections for real-time updates
   - **Port**: 8000 (external)

### Event Subscriptions

Subscribes to RabbitMQ events from:
- `oms.*`: Order lifecycle events
- `scheduler.*`: Order processing, plan updates
- `validation.*`: Inventory updates
- `automation.*`: Automation events
- `routine.*`: Routine execution events

**Event Handlers:**
- Order events → Broadcast to WebSocket/Socket.IO
- Inventory events → Broadcast inventory updates
- Scheduler events → Broadcast task progress

## Troubleshooting

### RabbitMQ Connection Failed

**Issue**: API Bridge fails to start with RabbitMQ connection error

**Solutions:**
1. Verify RabbitMQ is running:
   ```bash
   docker-compose ps rabbitmq
   curl http://localhost:15672  # Management UI
   ```

2. Check RabbitMQ credentials:
   ```bash
   docker-compose logs rabbitmq | grep -i "default user"
   ```

3. Verify network connectivity:
   ```bash
   docker exec -it barns-api-bridge ping rabbitmq
   ```

### Timeout Errors on API Calls

**Issue**: API returns 500 error with timeout message

**Causes:**
- Target service not running
- Service overloaded
- RabbitMQ queue backed up

**Solutions:**
1. Check target service health:
   ```bash
   docker-compose ps
   docker-compose logs [service-name]
   ```

2. Increase timeout in code (default 30s):
   ```python
   response = await rabbitmq_client.send_request(
       target_service="oms",
       action="create_order",
       timeout=60  # Increase to 60 seconds
   )
   ```

### WebSocket Disconnects Frequently

**Issue**: Dashboard loses real-time connection

**Solutions:**
1. Implement automatic reconnection in dashboard:
   ```javascript
   const reconnectWebSocket = () => {
     const ws = new WebSocket('ws://localhost:8000/ws');
     ws.onclose = () => {
       setTimeout(reconnectWebSocket, 3000);
     };
   };
   ```

2. Check network stability and firewall rules

3. Enable WebSocket ping/pong heartbeat

### CORS Errors

**Issue**: Dashboard cannot connect due to CORS

**Solution**: Add dashboard origin to CORS configuration:
```python
allow_origins=[
    "http://your-dashboard-domain:port"
]
```

### Event Not Broadcasting

**Issue**: Real-time updates not reaching dashboard

**Debugging:**
1. Check event listener connection:
   ```bash
   docker-compose logs api-bridge | grep "Event"
   ```

2. Verify RabbitMQ exchange bindings:
   ```bash
   # Access RabbitMQ management UI
   http://localhost:15672
   # Check barns_events exchange bindings
   ```

3. Test WebSocket connection:
   ```javascript
   const ws = new WebSocket('ws://localhost:8000/ws');
   ws.onopen = () => console.log('Connected');
   ws.onmessage = (e) => console.log('Message:', e.data);
   ```

## Performance Considerations

- **Concurrent Requests**: Handles hundreds of concurrent HTTP requests
- **WebSocket Connections**: Supports 100+ simultaneous connections
- **RabbitMQ Connection Pooling**: Single persistent connection per client
- **Timeout Management**: 30s default, configurable per endpoint
- **Memory Usage**: ~100MB base + ~1MB per active WebSocket connection

## Security Notes

- No authentication implemented (internal network only)
- CORS restricted to known dashboard origins
- RabbitMQ credentials in environment variables
- All communication over internal Docker network
- External access only on port 8000

## Future Enhancements

- JWT authentication for API endpoints
- Rate limiting per client
- Request/response logging middleware
- Metrics and monitoring integration
- GraphQL API layer
- HTTP/2 support
- TLS/SSL for production deployments
