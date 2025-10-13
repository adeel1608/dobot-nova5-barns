# Order Management Service (OMS)

## Brief Overview

The OMS is the central order orchestration hub managing the complete order lifecycle from creation through completion, integrating POS systems, coordinating with Scheduler, managing queue priorities, persisting order data in PostgreSQL/Redis, and providing comprehensive HTTP and RabbitMQ APIs.

## Key Features

- **Order Lifecycle Management**: Create, start, stop, resume, complete, delete orders
- **Queue Management**: Priority-based order queue with Redis persistence
- **Database Persistence**: PostgreSQL for order history and audit trail
- **POS Integration**: Process orders from external POS systems
- **Alert System**: Low inventory and system alerts
- **Real-Time Updates**: Event broadcasting to dashboard
- **HTTP + RabbitMQ APIs**: Dual interface for flexibility
- **Order State Machine**: Robust state transitions with validation

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                   Order Management Service                    │
│                                                               │
│  ┌─────────────────────────────────────────────┐            │
│  │  FastAPI HTTP Server (app.py:2140 lines)    │            │
│  │  - REST API endpoints                       │            │
│  │  - POS integration                          │            │
│  │  - Order CRUD                               │            │
│  └────────────┬────────────────────────────────┘            │
│               │                                              │
│               ↓                                              │
│  ┌─────────────────────────────────────────────┐            │
│  │  RabbitMQ Handler                           │            │
│  │  - Async message processing                 │            │
│  │  - Event subscriptions                      │            │
│  └────────────┬────────────────────────────────┘            │
│               │                                              │
│     ┌─────────┼─────────┐                                   │
│     ↓         ↓         ↓                                   │
│ ┌────────┐ ┌──────┐ ┌──────────┐                           │
│ │Redis   │ │Postgres│ │Models  │                           │
│ │Queue   │ │Orders │ │(SQLAlch)│                           │
│ └────────┘ └──────┘ └──────────┘                           │
└────────┬──────────────────────┬────────────────────────────┘
         │                      │
         ↓                      ↓
┌────────────────┐     ┌────────────────┐
│  Scheduler     │     │  Dashboard     │
│  Service       │     │  (via API      │
│                │     │   Bridge)      │
└────────────────┘     └────────────────┘
```

## Setup & Installation

### Prerequisites

- PostgreSQL database
- Redis server
- RabbitMQ server
- Python 3.8+

### Docker Deployment

```bash
docker-compose up -d postgres redis rabbitmq oms-service
docker-compose logs -f oms-service
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `RABBITMQ_URL` | `amqp://admin:admin123@rabbitmq:5672/` | RabbitMQ connection |
| `REDIS_HOST` | `redis` | Redis host |
| `REDIS_PORT` | `6379` | Redis port |
| `POSTGRES_HOST` | `postgres` | PostgreSQL host |
| `POSTGRES_PORT` | `5432` | PostgreSQL port |
| `POSTGRES_DB` | `barns_oms` | Database name |
| `POSTGRES_USER` | `barns_user` | Database user |
| `POSTGRES_PASSWORD` | `barns_pass` | Database password |
| `PYTHONPATH` | `/app` | Python module path |

### Database Schema

Initialized from `schema.sql`:
- `orders` table: Order records with full history
- `order_items` table: Individual cups/drinks per order
- Indexes on status, created_at for performance

## API/Endpoints

### HTTP REST API (Port 8002)

#### POST /orders
Create new order.

**Request:**
```json
{
  "cups": [
    {
      "recipe": "latte",
      "size": "medium",
      "customizations": {}
    }
  ]
}
```

**Response:**
```json
{
  "success": true,
  "order_id": 123,
  "status": "pending",
  "created_at": "2025-01-15T10:30:00"
}
```

#### GET /orders
List all orders with optional status filter.

Query params: `?status=pending`

#### GET /orders/{order_id}
Get specific order details.

#### PATCH /orders/{order_id}/start
Start order processing.

#### POST /orders/{order_id}/stop
Emergency stop order.

#### POST /orders/{order_id}/resume
Resume stopped order.

#### DELETE /orders/{order_id}
Delete order.

#### GET /queue
Get current order queue.

#### PUT /queue/reorder
Reorder queue positions.

### POS Integration

#### POST /pos/process-order
Process order from POS system.

**Request:**
```json
{
  "transaction_id": "POS-001",
  "items": [
    {"product_id": "latte_medium", "quantity": 1}
  ],
  "customer_name": "John Doe"
}
```

#### GET /pos/menu-items
Get POS menu items.

#### GET /pos/ingredients
Get ingredient list for POS.

### RabbitMQ Actions

#### Action: `create_order`
Create order via RabbitMQ.

#### Action: `list_orders`
List orders via RabbitMQ.

#### Action: `get_order`
Get order details.

#### Action: `start_order`
Start order processing.

#### Action: `stop_order`
Stop order.

#### Action: `delete_order`
Delete order.

#### Action: `sync_queue`
Get queue status.

#### Action: `emergency_stop`
System-wide emergency stop.

## Order State Machine

```
pending → started → processing → completed
   ↓         ↓          ↓            
 deleted   halted    failed
             ↓
          resumed → processing
```

**Valid Transitions:**
- `pending` → `started`, `deleted`
- `started` → `processing`, `halted`, `failed`
- `processing` → `completed`, `halted`, `failed`
- `halted` → `resumed`, `deleted`
- `resumed` → `processing`

## Usage Examples

### Create and Start Order

```python
import httpx
import asyncio

async def create_order():
    async with httpx.AsyncClient() as client:
        # Create
        response = await client.post(
            "http://localhost:8002/orders",
            json={"cups": [{"recipe": "latte"}]}
        )
        order = response.json()
        order_id = order["order_id"]
        
        # Start
        response = await client.patch(
            f"http://localhost:8002/orders/{order_id}/start"
        )
        return response.json()

asyncio.run(create_order())
```

### POS Order Processing

```python
async def pos_order():
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "http://localhost:8002/pos/process-order",
            json={
                "transaction_id": "POS-123",
                "items": [
                    {"product_id": "latte_medium", "quantity": 2}
                ]
            }
        )
        return response.json()
```

### Queue Management

```python
# Get queue
response = await client.get("http://localhost:8002/queue")
queue = response.json()

# Reorder
response = await client.put(
    "http://localhost:8002/queue/reorder",
    json={"order": [124, 123, 125]}
)
```

## Dependencies

### Core Dependencies

- **FastAPI** (0.115.12): HTTP API framework
- **SQLAlchemy** (ORM for PostgreSQL)
- **psycopg2-binary** (2.9.9): PostgreSQL driver
- **redis** (5.0.1): Redis client
- **aio-pika** (9.3.1): RabbitMQ async client
- **uvicorn** (0.24.0): ASGI server

## Integration Points

### Downstream Services (Calls To)

1. **Scheduler Service**
   - `process_order`: Start order processing
   - `stop_order`, `resume_order`: Order control
   - **Protocol**: RabbitMQ RPC

2. **Validation Service**
   - (Indirectly via Scheduler)

### Upstream Services (Receives From)

1. **API Bridge**
   - All HTTP requests from dashboard
   - **Protocol**: HTTP REST

2. **POS Systems**
   - External order submissions
   - **Protocol**: HTTP REST

3. **Scheduler Service**
   - Order completion notifications
   - **Protocol**: RabbitMQ events

### Event Subscriptions

Listens to:
- `scheduler.order_completed`: Order finished
- `scheduler.order_failed`: Order failed
- `validation.threshold_warning`: Low inventory alerts

### Event Publications

Broadcasts to `barns_events` exchange:
- `oms.order_created`: New order
- `oms.order_started`: Order processing started
- `oms.order_completed`: Order finished
- `oms.order_failed`: Order failed
- `oms.order_status_updated`: Status changed
- `oms.order_deleted`: Order removed

## Troubleshooting

### Database Connection Failed

```bash
# Check PostgreSQL
docker-compose ps postgres
docker exec -it barns-postgres psql -U barns_user -d barns_oms -c "SELECT COUNT(*) FROM orders;"
```

### Redis Connection Failed

```bash
# Check Redis
docker-compose ps redis
docker exec -it barns-redis redis-cli ping
```

### Order Stuck in Processing

1. Check Scheduler service:
   ```bash
   docker-compose logs scheduler-service
   ```

2. Check order status:
   ```bash
   curl http://localhost:8002/orders/123
   ```

3. Force stop if needed:
   ```bash
   curl -X POST http://localhost:8002/orders/123/stop
   ```

### Queue Desynchronization

Redis and PostgreSQL out of sync:
```bash
# Restart OMS to rebuild queue from database
docker-compose restart oms-service
```

## Performance Considerations

- **Database**: Indexed on status and created_at
- **Redis Queue**: In-memory for fast access
- **Concurrent Requests**: Handles 100+ req/sec
- **Memory Usage**: ~200MB base + ~10KB per order
- **Order Throughput**: 50+ orders/minute

## Security Notes

- No authentication on HTTP endpoints (internal network)
- POS integration should use API keys (not implemented)
- Database credentials in environment variables
- All communication over internal Docker network

## Future Enhancements

- OAuth2 authentication for HTTP API
- Order analytics and reporting dashboard
- Customer notification system
- Payment integration
- Order modification support
- Batch order processing
- SLA tracking and monitoring
