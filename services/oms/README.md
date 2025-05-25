# Order Management Service (OMS)

## Purpose and Workflow

The Order Management Service (OMS) is the central orchestrator of the BARNS (Business Automation & Robotics) system. It manages the complete lifecycle of coffee orders from creation to completion.

### Core Responsibilities
- **Order Queue Management**: Maintains and manages the order queue using Redis
- **Order Lifecycle Tracking**: Tracks orders through states: `queued` → `processing` → `completed/failed/halted`
- **Database Persistence**: Stores orders, tasks, steps, events, and alerts in PostgreSQL
- **Real-time Communication**: Provides WebSocket connections for live dashboard updates
- **System Coordination**: Interfaces with Scheduler service to initiate order processing
- **Alert Management**: Handles system alerts and notifications

### Workflow
1. **Order Creation**: Receives new orders via REST API, stores in database, adds to Redis queue
2. **Queue Management**: Allows reordering of queued orders via drag-and-drop interface
3. **Processing Initiation**: When "Start" is clicked, sends order to Scheduler service
4. **Status Tracking**: Receives completion/failure notifications from Scheduler
5. **Real-time Updates**: Broadcasts status changes to connected dashboard clients

## API Structure

### Order Endpoints

#### Create Order
```http
POST /orders/
Content-Type: application/json

{
  "status": "queued",
  "cups": [
    {
      "type": "Latte",
      "size": "regular", 
      "addons": ["extra_shot"]
    }
  ]
}
```

#### Get Orders
```http
GET /orders/                    # Get all orders
GET /orders/?status=queued      # Filter by status
GET /orders/{order_id}          # Get specific order with detailed task information
```

#### Order Actions
```http
PATCH /orders/{order_id}/start                    # Start processing order
POST /orders/{order_id}/complete                  # Mark order complete (called by Scheduler)
POST /orders/{order_id}/fail?reason=error_msg     # Mark order failed (called by Scheduler)
POST /orders/{order_id}/halt?reason=issue_desc    # Halt order for manual intervention
POST /orders/{order_id}/resume                    # Resume halted order
```

#### Queue Management
```http
PUT /orders/{order_id}/reorder         # Reorder single item
PUT /orders/reorder                    # Bulk reorder entire queue
```

### Task Management Endpoints
```http
POST /tasks/                           # Create task
PATCH /tasks/{task_id}/status          # Update task status
POST /tasks/steps/                     # Create task step
PATCH /tasks/steps/{step_id}/status    # Update step status
```

### System Control Endpoints
```http
POST /system/stop                      # Emergency stop
POST /system/resume                    # Resume operations
GET /system/status                     # Get system status
```

### Real-time Communication
```http
WebSocket /ws/orders                   # Order status updates
WebSocket /ws/alerts                   # System alerts
```

## Database Schema

### Core Tables
- **orders**: Order information and status
- **order_items**: Individual cups/items in orders
- **tasks**: Processing tasks for each order item
- **task_steps**: Individual steps within tasks
- **events**: System events log
- **alerts**: System alerts and notifications

### Key Status Values
- **Order Status**: `queued`, `processing`, `completed`, `halted`, `stopped`, `error`, `cancelled`
- **Task Status**: `queued`, `running`, `completed`, `failed`, `halted`

## Adding New Modules

### 1. Adding New Order Types

**Step 1**: Update the order model in `models.py`:
```python
# Add new cup types or drink options
class Cup:
    type: str  # Add new drink types here
    size: str  # Add new sizes
    addons: List[str]  # Add new addon options
```

**Step 2**: Update database schema if needed:
```sql
-- Add new columns to order_items table if required
ALTER TABLE order_items ADD COLUMN new_field VARCHAR(255);
```

### 2. Adding New API Endpoints

**Step 1**: Add endpoint to `app.py`:
```python
@app.get("/orders/analytics")
def get_order_analytics():
    """New analytics endpoint."""
    data = db.get_analytics_data()
    return {"analytics": data}
```

**Step 2**: Add corresponding database function in `db.py`:
```python
def get_analytics_data():
    """Fetch analytics data from database."""
    # Implementation here
    pass
```

### 3. Adding New Database Operations

**Step 1**: Add function to `db.py`:
```python
def new_database_operation(param1: str, param2: int) -> Dict[str, Any]:
    """New database operation."""
    conn = get_connection()
    try:
        with conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            cur.execute("SELECT * FROM table WHERE condition = %s", (param1,))
            return cur.fetchall()
    finally:
        release_connection(conn)
```

### 4. Adding New Event Types

**Step 1**: Define event type constants:
```python
# Add to app.py
EVENT_TYPES = {
    'NEW_EVENT': 'new_event_type',
    # ... existing events
}
```

**Step 2**: Create event logging function:
```python
def log_new_event(order_id: int, details: dict):
    """Log new event type."""
    event_id = db.log_event(EVENT_TYPES['NEW_EVENT'], {
        "order_id": order_id,
        "details": details,
        "timestamp": "now"
    })
    broadcast({"event": "new_event_occurred", "order": order_id, "event_id": event_id})
```

### 5. Adding New WebSocket Events

**Step 1**: Define broadcast message structure:
```python
def broadcast_new_event(data: dict):
    """Broadcast new event type to clients."""
    broadcast({
        "event": "new_event_name",
        "data": data,
        "timestamp": datetime.now().isoformat()
    })
```

## Environment Variables

```env
DB_NAME=barns_db
DB_USER=postgres  
DB_PASSWORD=postgres
DB_HOST=localhost
DB_PORT=5432
REDIS_HOST=localhost
REDIS_PORT=6379
```

## Development Setup

1. **Install Dependencies**:
   ```bash
   pip install fastapi uvicorn psycopg2-binary redis httpx
   ```

2. **Database Setup**:
   ```bash
   # Run database migrations
   python -m services.oms.migrations
   ```

3. **Run Service**:
   ```bash
   uvicorn services.oms.app:app --host 0.0.0.0 --port 8000 --reload
   ```

## Testing

```bash
# Unit tests
python -m pytest services/oms/tests/

# API testing
curl -X POST "http://localhost:8000/orders/" \
  -H "Content-Type: application/json" \
  -d '{"status": "queued", "cups": [{"type": "Latte", "size": "regular"}]}'
```

## Scalability Considerations

- **Database Connection Pooling**: Uses PostgreSQL connection pooling for concurrent requests
- **Redis Queue**: Scalable queue management with Redis
- **Async Operations**: FastAPI async support for high concurrency
- **Modular Design**: Clear separation between API, business logic, and data layers
- **Event-Driven Architecture**: Loose coupling through events and WebSocket broadcasting 