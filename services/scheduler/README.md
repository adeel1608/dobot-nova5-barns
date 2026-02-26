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

## APIs (Inbound and Outbound)

The Scheduler service does **not** expose HTTP endpoints. All integration is via **RabbitMQ RPC actions** (request/response) and **RabbitMQ events** (fire-and-forget, some with ack).

### Inbound APIs (into Scheduler)

#### RabbitMQ RPC actions (request/response)

These actions are registered by `services/scheduler/app.py` on the Scheduler RPC consumer.

##### Action: `process_order`
Accept an order and start processing asynchronously.

- **Request**

```json
{
  "id": 123,
  "cups": [
    {
      "type": "latte",
      "size": "medium",
      "addons": [],
      "ingredients": {
        "coffee_beans": { "type": "regular", "amount": 2 },
        "milk": { "type": "whole", "amount": 200 },
        "position": { "cup_position": 2 }
      }
    }
  ]
}
```

- **Response**

```json
{
  "success": true,
  "message": "Order accepted by scheduler",
  "order_id": 123
}
```

Notes:
- Scheduler generates internal cup IDs as `"{order_id}-{cup_index}"` (1-based), e.g. `"123-1"`, `"123-2"`.
- If a cup’s `type` has no matching recipe in `/app/data/recipes.json`, the call fails during setup.

##### Action: `feedback`
Synchronous feedback path used by Routine to report completion/failure.

- **Request**

```json
{
  "cup_id": "123-1",
  "action": "pull_espresso_shot",
  "success": true,
  "message": "optional human-readable message"
}
```

- **Response**

```json
{
  "success": true,
  "status": "received"
}
```

##### Action: `update_cup_position`
Update the cup position used by future pending tasks (typically after cup detection).

- **Request**

```json
{
  "cup_id": "123-1",
  "new_position": 3,
  "old_position": 2
}
```

- **Response (success)**

```json
{
  "success": true,
  "message": "Position updated to 3"
}
```

##### Action: `get_status`
Return the current scheduler status object.

- **Response**

```json
{
  "success": true,
  "status": {
    "order_id": 123,
    "cup_index": null,
    "step": "preparing",
    "status": "in_progress"
  },
  "timestamp": "2026-02-26T12:34:56.789012"
}
```

##### Action: `subscribe_status`
Register a service name as a status subscriber (Scheduler still broadcasts status events regardless; this is currently tracked in-memory only).

- **Request**

```json
{ "service_name": "dashboard" }
```

- **Response**

```json
{ "success": true, "subscribed": true }
```

##### Action: `health`
Service health check, including RabbitMQ client health snapshot.

- **Response**

```json
{
  "status": "healthy",
  "service": "scheduler",
  "timestamp": "2026-02-26T12:34:56.789012",
  "rabbitmq_health": { "healthy": true },
  "event_listener_connected": true,
  "recipes_loaded": 10
}
```

##### Action: `cancel_order`
Cancel the active order in scheduler state and request Routine to drop queued work.

- **Request**

```json
{ "order_id": 123 }
```

- **Response**

```json
{ "success": true, "cancelled_tasks": 7 }
```

##### Action: `stop_order`
Stop the active order (graceful halt). Scheduler also instructs Routine to stop.

- **Request**

```json
{ "order_id": 123 }
```

- **Response**

```json
{ "success": true, "message": "Order stopped - all tasks completed or halted" }
```

##### Action: `resume_order`
Resume a stopped order. Scheduler converts paused/cancelled tasks back to pending and instructs Routine to resume.

- **Request**

```json
{ "order_id": 123 }
```

- **Response**

```json
{ "success": true, "message": "Order resumed" }
```

##### Action: `revert_previous_step`
Request that Scheduler revert a cup to a previous step (used by Routine-controlled recovery).

- **Request**

```json
{ "cup_id": "123-1", "current_action": "pour_milk" }
```

##### Action: `task_paused`
Notification from Routine that a task was paused so Scheduler can mark the task as `paused` and avoid waiting on it during stop flows.

- **Request**

```json
{ "cup_id": "123-1", "function": "validate_ingredients", "reason": "validation_failure" }
```

#### RabbitMQ events consumed (fire-and-forget)

Scheduler subscribes to `routine.*`, `system.*`, and `oms.*`, and handles:
- `routine.task_completed` (expects `cup_id`, `function`)
- `routine.task_failed` (expects `cup_id`, `function`, optional `error`)
- `system.shutdown`

### Outbound APIs (out of Scheduler)

#### RabbitMQ RPC calls (Scheduler -> other services)

##### To Validation service
- **Action `validate_ingredients`**: Pre-check a cup before starting work.
  - **Payload**: `{ "request_id", "client_type": "scheduler", "cup_id", ...ingredients }`
- **Action `update_ingredients`**: Subtract cup ingredients from inventory after validation passes.
  - **Payload**: `{ "request_id", "client_type": "scheduler", "cup_id", ...ingredients }`

##### To Routine service
- **Action `submit_task`**: Submit a task for execution.
  - **Payload**:

```json
{
  "arm_id": 1,
  "function": "pull_espresso_shot",
  "item": {
    "cup_id": "123-1",
    "drink_type": "latte",
    "size": "medium",
    "addons": [],
    "ingredients": {},
    "scheduler_validated": true
  }
}
```

- **Action `cancel_order`**: `{ "order_id": 123, "cup_ids": ["123-1", "123-2"] }`
- **Action `stop_order`**: `{ "order_id": 123 }`
- **Action `resume_order`**: `{ "order_id": 123 }`

##### To OMS service
- **Action `update_order_status`**: Used when stopping due to validation failures.
  - **Payload**: `{ "order_id": 123, "status": "stopped", "reason": "..." }`

#### RabbitMQ events published (Scheduler -> exchange)

These are emitted by `app.py` and `scheduler.py`:
- `scheduler.order_received`
- `scheduler.order_processing_started`
- `scheduler.plan_built` (includes `plan`; may be emitted more than once, treat as idempotent)
- `scheduler.status_update` (includes `message` and `status`)
- `scheduler.feedback_processed`
- `scheduler.cup_position_updated`
- `scheduler.order_stopping`
- `scheduler.order_stopped`
- `scheduler.order_resumed`
- `scheduler.order_error`
- `scheduler.order_heartbeat` (every 30s while order is active; fire-and-forget)

Order completion notifications to OMS (sent with acknowledgment):
- `scheduler.order_completed` (sent via `send_event_with_ack`, expects OMS acknowledgment)
- `scheduler.order_failed` (sent via `send_event_with_ack`, expects OMS acknowledgment)

Validation-related operator/dashboard events:
- `validation.failed.dashboard`
- `scheduler.refill_required`
- `scheduler.cup_validation_failed`
- `scheduler.order_stopped_validation`

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
            "id": 123,
            "cups": [
                {
                    "type": "latte",
                    "size": "medium",
                    "addons": [],
                    "ingredients": {
                        "coffee_beans": {"type": "regular", "amount": 2},
                        "milk": {"type": "whole", "amount": 200},
                        "position": {"cup_position": 2}
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
   - `validate_ingredients`: Validate cup ingredients before starting a new cup
   - `update_ingredients`: Subtract cup ingredients from inventory after validation passes
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 30 seconds

2. **Routine Service**
   - Task execution submissions
   - **Protocol**: RabbitMQ RPC (`submit_task`, `stop_order`, `resume_order`, `cancel_order`)

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

Publishes to the `barns_services` topic exchange using routing keys:
- **RPC**: `{service}.{action}` (example: `scheduler.process_order`)
- **Events**: `events.{event_type}` (example: `events.scheduler.status_update`)

Commonly consumed by dashboard/OMS:
- `scheduler.plan_built`
- `scheduler.status_update`
- `scheduler.order_processing_started`
- `scheduler.order_stopping`
- `scheduler.order_stopped`
- `scheduler.order_resumed`
- `scheduler.order_completed` (acknowledged by OMS)
- `scheduler.order_failed` (acknowledged by OMS)

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
   response = await client.send_request(target_service="scheduler", action="get_status", data={})
   print(response["status"])
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
