# Scheduler Service

## Brief Overview

The Scheduler Service is the orchestration brain of BARNS that transforms drink orders into executable tasks, manages dependencies, coordinates multi-arm parallel execution, validates ingredient availability, and tracks real-time progress throughout the drink-making process.

## Key Features

- **Recipe-to-Task Parsing**: Converts drink recipes into dependency-aware task graphs
- **Multi-Arm Coordination**: Intelligent scheduling across Robot Arm 1 and Arm 2
- **Dependency Management**: Ensures tasks execute only after prerequisites complete
- **Per-Cup Prioritization**: Completes one cup at a time per arm for efficiency
- **Inventory Pre-Check**: Validates ingredient availability before starting
- **Real-Time Progress Tracking**: Per-task status updates to dashboard
- **Failure Handling**: Task retry logic and error recovery
- **Order Control**: Start, stop, resume, and cancel operations
- **Event Broadcasting**: Live updates to OMS and dashboard

## Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    Scheduler Service                        │
│                                                             │
│  ┌──────────────────────────────────────────────────┐     │
│  │  SchedulerService (app.py)                       │     │
│  │  - RabbitMQ Message Handlers                     │     │
│  └───────────┬──────────────────────────────────────┘     │
│              │                                             │
│              ↓                                             │
│  ┌──────────────────────────────────────────────────┐     │
│  │  Scheduler Core (scheduler.py)                   │     │
│  │                                                   │     │
│  │  ┌────────────────────────────────────────┐     │     │
│  │  │  Recipe Parser                         │     │     │
│  │  │  - Load recipes.json                   │     │     │
│  │  │  - Convert to task graphs              │     │     │
│  │  └────────────────────────────────────────┘     │     │
│  │                                                   │     │
│  │  ┌────────────────────────────────────────┐     │     │
│  │  │  Task Scheduler                        │     │     │
│  │  │  - Dependency resolution               │     │     │
│  │  │  - Multi-arm coordination              │     │     │
│  │  │  - Per-cup prioritization              │     │     │
│  │  └────────────────────────────────────────┘     │     │
│  │                                                   │     │
│  │  ┌────────────────────────────────────────┐     │     │
│  │  │  Execution Coordinator                 │     │     │
│  │  │  - Submit tasks to Routine             │     │     │
│  │  │  - Track completion/failure            │     │     │
│  │  │  - Handle feedback events              │     │     │
│  │  └────────────────────────────────────────┘     │     │
│  └──────────┬────────────────────────────────────┬─┘     │
│             │                                    │        │
└─────────────┼────────────────────────────────────┼────────┘
              │                                    │
              ↓                                    ↓
     ┌────────────────┐                  ┌────────────────┐
     │  Routine       │                  │  Validation    │
     │  Service       │                  │  Service       │
     │  (Execute)     │                  │  (Pre-Check)   │
     └────────────────┘                  └────────────────┘
              │
              ↓
     ┌────────────────────┐
     │   RabbitMQ Events  │
     │   - Progress       │
     │   - Completion     │
     │   - Failures       │
     └────────────────────┘
```

### Execution Flow

1. **Order Received**: OMS sends `process_order` request
2. **Recipe Parsing**: Load recipe, create task graph with dependencies
3. **Inventory Validation**: Call Validation service to check ingredients
4. **Task Scheduling**: Build per-arm execution plan
5. **Task Execution**: Submit tasks to Routine service when dependencies satisfied
6. **Progress Tracking**: Listen for `routine.task_completed` events
7. **Completion**: Notify OMS when all tasks done

## Setup & Installation

### Prerequisites

- Python 3.8+
- RabbitMQ server
- Access to Validation and Routine services
- Recipe JSON file (`data/recipes.json`)
- Docker (for containerized deployment)

### Local Development

```bash
# Navigate to service directory
cd services/scheduler

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export RABBITMQ_URL="amqp://admin:admin123@localhost:5672/"
export PYTHONPATH="/path/to/barns"

# Ensure recipes.json exists
ls -l ../../data/recipes.json

# Run service
python app.py
```

### Docker Deployment

```bash
# Start scheduler and dependencies
docker-compose up -d rabbitmq validation-service routine-service scheduler-service

# View logs
docker-compose logs -f scheduler-service

# Check recipe loading
docker-compose logs scheduler-service | grep "Loaded.*recipes"
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `RABBITMQ_URL` | `amqp://admin:admin123@rabbitmq:5672/` | RabbitMQ connection |
| `PYTHONPATH` | `/app` | Python module search path |

### Recipe Format (`data/recipes.json`)

```json
{
  "latte": [
    {
      "action": "pick_cup_medium",
      "assigned_arm": "Arm1",
      "depends_on": []
    },
    {
      "action": "grind_coffee",
      "assigned_arm": "Arm2",
      "depends_on": []
    },
    {
      "action": "pull_espresso_shot",
      "assigned_arm": "Arm2",
      "depends_on": ["grind_coffee", "pick_cup_medium"]
    },
    {
      "action": "steam_milk",
      "assigned_arm": "Arm1",
      "depends_on": ["pull_espresso_shot"]
    },
    {
      "action": "pour_milk",
      "assigned_arm": "Arm1",
      "depends_on": ["steam_milk"]
    }
  ]
}
```

**Recipe Fields:**
- `action`: Task name (must match Routine service actions)
- `assigned_arm`: "Arm1" or "Arm2"
- `depends_on`: List of prerequisite actions (can be empty)

### Docker Volume Mounts

```yaml
volumes:
  - ./data:/app/data  # Recipes directory
```

## API/Endpoints

### Action: `process_order`
Start processing an order.

**Request:**
```json
{
  "request_id": "req-001",
  "order_id": 123,
  "cups": [
    {
      "drink": "latte",
      "cup_id": "cup_1",
      "size": "medium",
      "ingredients": {
        "coffee_beans": {"type": "regular", "amount": 2},
        "milk": {"type": "whole", "amount": 200}
      }
    }
  ]
}
```

**Response:**
```json
{
  "success": true,
  "order_id": 123,
  "message": "Order processing started",
  "tasks_count": 15,
  "validation_result": {
    "passed": true,
    "details": {}
  }
}
```

### Action: `feedback`
Process task completion/failure feedback from Routine.

**Request:**
```json
{
  "cup_id": "cup_1",
  "action": "pull_espresso_shot",
  "success": true,
  "message": "Espresso pulled successfully"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Feedback processed"
}
```

### Action: `get_status`
Get current order execution status.

**Response:**
```json
{
  "success": true,
  "status": {
    "order_id": 123,
    "cup_index": 0,
    "step": "pull_espresso_shot",
    "status": "in_progress"
  },
  "tasks": {
    "total": 15,
    "completed": 8,
    "failed": 0,
    "pending": 7
  },
  "per_arm": {
    "Arm1": {
      "current_cup": "cup_1",
      "pending_tasks": ["steam_milk", "pour_milk"],
      "completed_tasks": ["pick_cup_medium"]
    },
    "Arm2": {
      "current_cup": "cup_1",
      "pending_tasks": [],
      "completed_tasks": ["grind_coffee", "pull_espresso_shot"]
    }
  }
}
```

### Action: `stop_order`
Emergency stop current order.

**Request:**
```json
{
  "order_id": 123
}
```

**Response:**
```json
{
  "success": true,
  "message": "Order stopped",
  "tasks_completed": 8,
  "tasks_cancelled": 7
}
```

### Action: `resume_order`
Resume a stopped order.

**Request:**
```json
{
  "order_id": 123
}
```

**Response:**
```json
{
  "success": true,
  "message": "Order resumed",
  "tasks_remaining": 7
}
```

### Action: `cancel_order`
Cancel order entirely.

**Request:**
```json
{
  "order_id": 123
}
```

### Action: `health`
Health check.

**Response:**
```json
{
  "status": "healthy",
  "service": "scheduler",
  "recipes_loaded": 10
}
```

## Usage Examples

### Process Order from OMS

```python
from shared.rabbitmq_client import RabbitMQClient
import asyncio

async def process_order():
    client = RabbitMQClient("oms")
    await client.connect()
    
    response = await client.send_request(
        target_service="scheduler",
        action="process_order",
        data={
            "order_id": 123,
            "cups": [
                {
                    "drink": "latte",
                    "cup_id": "cup_1",
                    "size": "medium",
                    "ingredients": {
                        "coffee_beans": {"type": "regular", "amount": 2},
                        "milk": {"type": "whole", "amount": 200}
                    }
                }
            ]
        },
        timeout=30
    )
    
    print(f"Order started: {response}")
    await client.disconnect()

asyncio.run(process_order())
```

### Monitor Progress

```python
from shared.rabbitmq_client import EventListener

async def monitor_progress():
    listener = EventListener("monitor")
    await listener.connect()
    
    def handle_progress(data):
        print(f"Task progress: {data}")
    
    listener.register_event_handler("scheduler.status_update", handle_progress)
    listener.register_event_handler("scheduler.feedback_processed", handle_progress)
    
    await listener.subscribe_to_events(["scheduler.*"])
    
    # Run forever
    await asyncio.Future()

asyncio.run(monitor_progress())
```

### Emergency Stop

```python
async def emergency_stop(order_id):
    client = RabbitMQClient("control")
    await client.connect()
    
    response = await client.send_request(
        target_service="scheduler",
        action="stop_order",
        data={"order_id": order_id},
        timeout=10
    )
    
    print(f"Stop response: {response}")
    await client.disconnect()
```

## Dependencies

### Core Dependencies

- **aio-pika** (9.3.1): Async RabbitMQ communication
- **httpx** (0.25.0): HTTP client for potential REST calls

### Shared Modules

- `shared.rabbitmq_client`: RabbitMQ client and event listener

## Integration Points

### Downstream Services (Calls To)

1. **Validation Service**
   - `pre_check`: Validate ingredients before starting
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 30 seconds

2. **Routine Service**
   - Task execution submissions
   - **Protocol**: Direct function calls (same process in future: RabbitMQ)

### Upstream Services (Receives From)

1. **OMS Service**
   - `process_order`: Start order processing
   - `stop_order`, `resume_order`, `cancel_order`: Order control
   - **Protocol**: RabbitMQ RPC

### Event Subscriptions

Listens to:
- `routine.task_completed`: Task success
- `routine.task_failed`: Task failure
- `system.shutdown`: Graceful shutdown

### Event Publications

Broadcasts to `barns_events` exchange:
- `scheduler.plan_built`: Initial task plan created
- `scheduler.status_update`: Progress updates
- `scheduler.feedback_processed`: Task completion
- `scheduler.order_completed`: All tasks done
- `scheduler.order_failed`: Order failed
- `scheduler.order_processing_started`: Order started

## Troubleshooting

### Recipe Not Found

**Issue**: "Recipe for drink 'X' not found"

**Solutions:**
1. Check recipes.json exists:
   ```bash
   docker exec -it barns-scheduler cat /app/data/recipes.json
   ```

2. Verify recipe name matches exactly (case-sensitive)

3. Reload service:
   ```bash
   docker-compose restart scheduler-service
   ```

### Inventory Validation Failed

**Issue**: Order rejected due to insufficient ingredients

**Debugging:**
1. Check validation service:
   ```bash
   docker-compose ps validation-service
   docker-compose logs validation-service
   ```

2. Manually check inventory:
   ```bash
   curl http://localhost:8000/api/inventory/status
   ```

3. Refill ingredients:
   ```bash
   curl -X POST http://localhost:8000/api/inventory/refill
   ```

### Tasks Stuck in Pending

**Issue**: Tasks not executing

**Causes:**
- Dependency not satisfied
- Routine service not running
- RabbitMQ connection lost

**Solutions:**
1. Check task status:
   ```python
   response = await client.send_request("scheduler", "get_status", {})
   print(response["per_arm"])
   ```

2. Verify Routine service:
   ```bash
   docker-compose ps routine-service
   docker-compose logs routine-service
   ```

3. Check dependencies in recipe (circular dependencies cause deadlock)

### Order Never Completes

**Issue**: Some tasks remain pending indefinitely

**Debugging:**
1. Get detailed status:
   ```bash
   # Look for failed or stuck tasks
   docker-compose logs scheduler-service | grep -E "(failed|stuck|pending)"
   ```

2. Check for failed tasks that block dependencies

3. Restart order if safe:
   ```bash
   # Stop current order
   # Fix underlying issue
   # Resume or create new order
   ```

## Performance Considerations

- **Task Execution**: Parallel execution across 2 arms
- **Dependency Resolution**: O(n) per task check
- **Memory Usage**: ~50MB base + ~1KB per task
- **Recipe Parsing**: <100ms for typical recipes
- **RabbitMQ Throughput**: Handles 50+ orders/minute
- **Per-Cup Strategy**: Reduces context switching, faster completion

## Security Notes

- No authentication (internal network only)
- Recipe file is read-only after load
- RabbitMQ credentials in environment
- All communication over internal Docker network

## Future Enhancements

- Dynamic recipe editing via API
- Machine learning for optimal task scheduling
- Predictive failure detection
- Multi-order batch optimization
- Resource-aware scheduling (arm availability)
- Task priority levels
- Graceful degradation (single arm mode)
- Historical analytics and optimization
