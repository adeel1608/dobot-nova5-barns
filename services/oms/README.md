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

## Complete API Surface (In/Out)

This section is the canonical list of all current inbound and outbound APIs implemented by OMS (primarily `services/oms/app.py`).

### Inbound APIs (Into OMS)

#### HTTP + WebSocket endpoints exposed by OMS

Default container port is documented as **8002** in this README; paths below are as implemented.

| Method | Path | Purpose |
|---|---|---|
| POST | `/orders/` | Create new order (persists + queues) |
| GET | `/orders/stats/summary` | Order statistics summary |
| GET | `/orders/` | List orders (supports `status`, `limit`, `offset`) |
| GET | `/orders/{order_id}` | Get order details (includes tasks/steps summary when available) |
| PUT | `/orders/{order_id}/reorder` | Reorder a single queued order (expects `new_position`) |
| PUT | `/orders/reorder` | Bulk reorder queue (body includes `order_ids`) |
| PATCH | `/orders/{order_id}/start` | Start processing (sends to Scheduler asynchronously) |
| PATCH | `/orders/{order_id}/status` | Update status (query params: `status`, optional `reason`) |
| POST | `/orders/{order_id}/halt` | Halt an order (creates alert) |
| POST | `/orders/{order_id}/stop` | Stop an order (requests Scheduler stop) |
| POST | `/orders/{order_id}/resume` | Resume a stopped/halted order (requests Scheduler resume) |
| POST | `/orders/{order_id}/complete` | Mark order completed |
| POST | `/orders/{order_id}/fail` | Mark order failed |
| DELETE | `/orders/{order_id}` | Delete order (requests Scheduler cancel if processing) |
| PATCH | `/tasks/{task_id}/status` | Update a task status |
| PATCH | `/tasks/steps/{step_id}/status` | Update a task-step status |
| PATCH | `/alerts/{alert_id}/acknowledge` | Acknowledge alert |
| GET | `/alerts/active` | List active alerts |
| GET | `/alerts/acknowledged` | List acknowledged alerts |
| POST | `/system/stop` | System-wide stop |
| POST | `/system/resume` | System-wide resume |
| GET | `/system/status` | System status summary |
| GET | `/system/rabbitmq-health` | RabbitMQ connectivity status |
| POST | `/pos/process-order` | POS order processing |
| GET | `/pos/menu-items` | POS menu items |
| GET | `/pos/ingredients` | POS ingredients list |
| GET | `/queue/sync` | Sync/read queue snapshot |
| POST | `/orders/mark-processing-failed` | Mark stuck processing orders as failed |
| POST | `/inventory/threshold-warning` | Receive threshold warning (creates alert) |
| POST | `/inventory/refill` | Manual refill trigger (attempts to call Validation HTTP) |
| GET | `/inventory/status` | Inventory status placeholder response |
| WS | `/ws/orders` | WebSocket for order broadcasts |
| WS | `/ws/alerts` | WebSocket for alert broadcasts |

#### RabbitMQ RPC actions handled by OMS (service name: `oms`)

These are the RPC `action` names other services can call via `RabbitMQClient.send_request(target_service="oms", action=...)`:

| Action | Purpose |
|---|---|
| `health` | OMS health check |
| `create_order` | Create order |
| `list_orders` | List orders |
| `get_order` | Get order |
| `start_order` | Start order processing (sends to Scheduler) |
| `stop_order` | Stop order (requests Scheduler stop) |
| `resume_order` | Resume order (requests Scheduler resume) |
| `halt_order` | Halt order (creates alert) |
| `update_order_status` | Update order status |
| `delete_order` | Delete order (requests Scheduler cancel if processing) |
| `sync_queue` | Queue snapshot/sync |
| `bulk_reorder_queue` | Bulk reorder queue |
| `emergency_stop` | System-wide emergency stop |
| `resume_operations` | Resume after stop |
| `get_active_alerts` | List active alerts |
| `get_acknowledged_alerts` | List acknowledged alerts |
| `acknowledge_alert` | Acknowledge an alert |
| `mark_processing_orders_failed` | Mark processing orders as failed |

#### RabbitMQ event topics consumed by OMS

OMS subscribes to:
- `scheduler.#`
- `validation.#`
- `automation.*`
- `routine.*`
- `system.*`

And registers concrete handlers for:
- Scheduler: `scheduler.order_completed`, `scheduler.order_failed`, `scheduler.order_heartbeat`, `scheduler.plan_built`, `scheduler.status_update`, `scheduler.feedback_processed`
- Validation: `validation.threshold_warning`, `validation.all_stations_occupied`, `validation.retry_status`, `validation.failed.dashboard`
- System: `system.shutdown`

### Outbound APIs (From OMS)

#### RabbitMQ RPC calls made by OMS

| When | Target service | RPC action |
|---|---|---|
| Start order | `scheduler` | `process_order` |
| Stop order | `scheduler` | `stop_order` |
| Resume order | `scheduler` | `resume_order` |
| Delete processing order | `scheduler` | `cancel_order` |

#### Direct HTTP calls made by OMS

| Inbound endpoint | Outbound method/path | Notes |
|---|---|---|
| POST `/inventory/refill` | `POST http://localhost:8003/inventory/refill` | Best-effort call; OMS continues if Validation is unreachable |

#### Events published by OMS (RabbitMQ)

OMS publishes:
- `oms.order_created`
- `oms.order_started`
- `oms.order_stopping`
- `oms.order_stopped`
- `oms.alert_created`

OMS also broadcasts realtime updates to connected WebSocket clients (`/ws/orders`, `/ws/alerts`) using JSON payloads that include an `event` field (for example: `order_received`, `order_started`, `order_stopping`, `order_stopped`, `order_deleted`, `threshold_warning`, `inventory_refilled`).

### HTTP REST API (Port 8002)

#### POST /orders/
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

#### GET /orders/
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

#### GET /queue/sync
Get current order queue snapshot.

#### PUT /orders/reorder
Bulk reorder queue positions.

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

#### Action: `resume_order`
Resume order.

#### Action: `halt_order`
Halt order.

#### Action: `update_order_status`
Update order status.

#### Action: `delete_order`
Delete order.

#### Action: `sync_queue`
Get queue status.

#### Action: `emergency_stop`
System-wide emergency stop.

#### Action: `resume_operations`
Resume after emergency stop.

#### Action: `get_active_alerts`
Get active alerts.

#### Action: `get_acknowledged_alerts`
Get acknowledged alerts.

#### Action: `acknowledge_alert`
Acknowledge alert.

#### Action: `mark_processing_orders_failed`
Mark processing orders as failed.

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
- **psycopg2-binary** (2.9.9): PostgreSQL driver
- **redis** (5.0.1): Redis client
- **aio-pika** (9.3.1): RabbitMQ async client
- **uvicorn** (0.24.0): ASGI server
- **httpx** (0.25.2): HTTP client (Validation refill best-effort call)
- **websockets** (12.0): WebSocket support
- **python-socketio** (5.13.0): Socket.IO support (dependency present)
- **requests** (2.31.0): HTTP client (dependency present)

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
- `oms.order_stopping`: Order stopping initiated
- `oms.order_stopped`: Order stopped
- `oms.alert_created`: Alert created (used by API Bridge to push to dashboard)

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
