# Routine Service

## Brief Overview

The Routine Service is the task execution coordinator that manages per-arm task queues, orchestrates calls to Validation, Automation, and Robot Arm services, and provides real-time feedback to the Scheduler about task completion and failures.

## Key Features

- **Per-Arm Task Queues**: Separate asyncio queues for Arm 1 and Arm 2
- **Worker Pattern**: Dedicated worker coroutine per robotic arm
- **Service Orchestration**: Coordinates Validation, Automation, and Robot Arm services
- **Task Configuration**: JSON-based task definitions with validation/automation/robot steps
- **Event Publishing**: Real-time completion/failure notifications
- **Error Handling**: Graceful failure recovery and reporting
- **Queue Management**: Status queries, queue clearing, order cancellation
- **Resource Lock Retry**: Automatic retry for locked resources to maintain strict queue order
- **Strict Queue Adherence**: Arms never skip tasks - they retry until resources become available

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                 Routine Service                          │
│                                                          │
│  ┌──────────────────────────────────────────────┐      │
│  │  RoutineService (app.py)                     │      │
│  │  - Message Handlers                          │      │
│  │  - Queue Management                          │      │
│  └────────────┬─────────────┬───────────────────┘      │
│               │             │                           │
│       ┌───────┴──┐      ┌───┴───────┐                  │
│       │  Queue 1 │      │  Queue 2  │                  │
│       │  (Arm 1) │      │  (Arm 2)  │                  │
│       └─────┬────┘      └─────┬─────┘                  │
│             │                 │                         │
│       ┌─────▼────┐      ┌─────▼────┐                   │
│       │ Worker 1 │      │ Worker 2 │                   │
│       └─────┬────┘      └─────┬────┘                   │
│             │                 │                         │
│             └──────┬──────────┘                         │
│                    │                                    │
│        ┌───────────▼────────────┐                       │
│        │  Executer (executer.py)│                       │
│        │  - Task Processing     │                       │
│        │  - Service Calls       │                       │
│        └────────────────────────┘                       │
└─────────────┬──────────┬─────────┬──────────────────────┘
              │          │         │
              ↓          ↓         ↓
     ┌───────────┐ ┌──────────┐ ┌────────────┐
     │Validation │ │Automation│ │ Robot Arm  │
     │ Service   │ │ Service  │ │  Service   │
     └───────────┘ └──────────┘ └────────────┘
```

### Execution Flow

1. **Task Submission**: Scheduler sends `submit_task` to Routine
2. **Queue Assignment**: Task added to appropriate arm's queue (1 or 2)
3. **Worker Processing**: Dedicated worker picks up task from queue
4. **Task Execution**: Execute steps from task config:
   - Validation checks (if configured)
   - Automation hardware control (if configured)
   - Robot arm movements (if configured)
5. **Event Publishing**:
   - Step-level events: `routine.step_completed` (and error events like `validation.failed`, `robot.error`, `automation.error`)
   - Task-level outcome: `routine.task_completed` or `routine.task_failed`
6. **Next Task**: Worker continues to next task in queue

## Setup & Installation

### Prerequisites

- Python 3.8+
- RabbitMQ server
- Access to Validation, Automation, and Robot Arm services
- Task configuration file (`config/tasks.json`)
- Docker (for containerized deployment)

### Local Development

```bash
# Navigate to service directory
cd services/routine

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export RABBITMQ_URL="amqp://admin:admin123@localhost:5672/"
export ROUTINE_CONFIG_PATH="/path/to/config/tasks.json"
export PYTHONPATH="/path/to/barns"

# Run service
python app.py
```

### Docker Deployment

```bash
# Start routine service and dependencies
docker-compose up -d rabbitmq validation-service automation-service robot-arm-service routine-service

# View logs
docker-compose logs -f routine-service

# Check worker status
docker-compose logs routine-service | grep "Worker started"
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `RABBITMQ_URL` | `amqp://admin:admin123@rabbitmq:5672/` | RabbitMQ connection |
| `ROUTINE_CONFIG_PATH` | `/app/config/tasks.json` | Task configuration file path |
| `PYTHONPATH` | `/app` | Python module search path |

### Task Configuration (`config/tasks.json`)

Defines validation, automation, and robot steps for each task:

```json
{
  "pick_cup_medium": {
    "validate": false,
    "automate": false,
    "robot": {
      "function": "pick_cup",
      "params": {
        "position": "medium_cup_stack",
        "grip_force": 30
      }
    }
  },
  "grind_coffee": {
    "validate": {
      "function": "check_coffee_beans",
      "params": {
        "amount_needed": 18
      }
    },
    "automate": {
      "function": "activate_grinder",
      "params": {
        "duration": 15,
        "grind_size": "fine"
      }
    },
    "robot": false
  },
  "pull_espresso_shot": {
    "validate": false,
    "automate": {
      "function": "espresso_machine",
      "params": {
        "shots": 2,
        "temperature": 92
      }
    },
    "robot": {
      "function": "position_cup",
      "params": {
        "location": "espresso_outlet"
      }
    }
  }
}
```

**Configuration Fields:**
- `validate`: Validation service call (false or object with function/params)
- `automate`: Automation service call (false or object with function/params)
- `robot`: Robot arm service call (false or object with function/params)

### Docker Volume Mounts

```yaml
volumes:
  - ./config:/app/config  # Task configurations
```

## Retry Logic & Queue Adherence

### Resource Lock Retry (Cup Station)

The routine service implements **strict queue adherence** - robotic arms will **never skip tasks** in their queue, even when resources are temporarily unavailable.

**Cup Station Lock Behavior:**
- Cup station functions (`place_cup_at_station`, `pick_cup_from_station`, etc.) require mutual exclusion between arms
- When a cup station resource is locked by another arm, the requesting arm will:
  1. **Retry every 1 second** until the lock becomes available
  2. Continue retrying up to **5 minutes** (configurable via `RESOURCE_LOCK_MAX_WAIT_TIME`)
  3. Log retry attempts every 5 retries to avoid log spam
  4. Only timeout and fail after exceeding maximum wait time

**Configuration Constants (in `executer.py`):**
```python
RESOURCE_LOCK_RETRY_INTERVAL = 1    # seconds between retry attempts
RESOURCE_LOCK_MAX_WAIT_TIME = 300   # maximum 5 minutes before timeout
```

**Example Log Output:**
```
[ARM-1] Attempting to acquire cup_station lock for place_cup_at_station (cup 123-1)
[ARM-1] Acquired cup_station lock for place_cup_at_station (no wait)
...
[ARM-2] Attempting to acquire cup_station lock for place_cup_at_station (cup 123-2)
[ARM-2] Retry 1: Waiting for cup_station lock (held by Arm 1) - place_cup_at_station on cup 123-2 (1.0s elapsed)
[ARM-2] Retry 5: Waiting for cup_station lock (held by Arm 1) - place_cup_at_station on cup 123-2 (5.2s elapsed)
[ARM-2] Acquired cup_station lock for place_cup_at_station after 8 retries (8.1s)
```

### Validation Retry

The **validation service** handles its own internal retries for operations like:
- `cup_detection`: Retries when all cup stations are occupied, waiting for a station to become available
- Other validation checks with transient failures

The routine service respects validation retry outcomes and only fails tasks when validation explicitly reports `retries_exhausted`.

### Robot Action Retry

Robot actions automatically retry on **transient failures**:
- Connection timeouts
- Health check failures
- Temporary RabbitMQ issues

**Retry Configuration:**
- Maximum 20 retry attempts
- 3 second delay between retries
- Only retries on transient errors (not hardware/logic errors)

### Feedback Retry

Feedback to the scheduler is critical for maintaining system state. The routine service retries feedback delivery:
- Maximum 20 retry attempts
- 2 second delay between retries
- Falls back to event-based notification if all retries fail

This ensures the scheduler always knows about task completion/failure, even during temporary network issues.

## APIs In (Consumed by Routine Service)

This service does not expose HTTP endpoints. It consumes RabbitMQ actions/events.

### RabbitMQ Actions

| Action | Purpose | Request Shape (JSON) |
|--------|---------|----------------------|
| `submit_task` | Enqueue a task for an arm worker | `{ "arm_id": 1, "function": "<task_name>", "item": { ... } }` |
| `get_queue_status` | Queue size for one arm or all arms | `{}` or `{ "arm_id": 1 }` |
| `clear_queue` | Clear one arm queue or all queues | `{}` or `{ "arm_id": 1 }` |
| `cancel_order` | Remove queued tasks matching `order_id-*` cup ids (or explicit `cup_ids`) | `{ "order_id": 123 }` or `{ "cup_ids": ["123-1","123-2"] }` |
| `stop_order` | Mark an order as stopped (pause execution) | `{ "order_id": 123 }` |
| `resume_order` | Clear stopped flag for an order (resume execution) | `{ "order_id": 123 }` |
| `health` | Service health and queue sizes | `{}` |

### `submit_task` Request Example

```json
{
  "arm_id": 1,
  "function": "pick_cup_medium",
  "item": {
    "cup_id": "123-1",
    "drink": "latte",
    "size": "medium"
  }
}
```

### Event Subscriptions

- Subscription patterns: `system.*`, `scheduler.*`
- Handled events:
  - `system.shutdown`

## APIs Out (Produced by Routine Service)

### RabbitMQ Events Published

Queue management / orchestration events:
- `routine.task_queued`
- `routine.task_completed`
- `routine.task_failed`
- `routine.queue_cleared`
- `routine.all_queues_cleared`
- `routine.order_cancelled`
- `routine.order_stopped`
- `routine.order_resumed`

Execution-step events emitted by the task executor:
- `routine.step_completed`
- `routine.completed`
- `step.error`
- `robot.error`
- `automation.error`
- `validation.failed`
- `validation.failed.dashboard`

### Downstream RabbitMQ Requests (Service-to-Service)

| Target Service | Action | Purpose | Request Shape (JSON) |
|---------------|--------|---------|----------------------|
| `scheduler` | `feedback` | Report task result (with retry + fallback events) | `{ "cup_id": "...", "action": "<task_name>", "success": true, "message": "", "timestamp": "..." }` |
| `scheduler` | `revert_previous_step` | Revert prior scheduler step after validation failure | `{ "cup_id": "...", "current_action": "<task_name>" }` |
| `scheduler` | `update_cup_position` | Persist updated cup position when routine remaps stations | `{ "cup_id": "...", "old_position": 1, "new_position": 2, "timestamp": "..." }` |
| `scheduler` | `task_paused` | Immediately mark task as paused (validation failed or order stopped) | `{ "cup_id": "...", "function": "<task_name>", "reason": "validation_failed|order_stopped", "timestamp": "..." }` |
| `validation` | `<validation_function_name>` | Run a validation step (action name is dynamic) | `{ "request_id": "...", "client_type": "routine", "cup_id": "...", ...params }` |
| `automation` | `automate` | Run an automation function | `{ "function": "<automation_function>", "params": { ... } }` |
| `robot_arm` | `robot_action` | Run a robot-arm function on a specific arm | `{ "function": "<robot_function>", "params": { ... , "arm_id": 1 }, "arm_id": 1 }` |
| `oms` | `stop_order` | Stop an order via OMS (used on validation failure) | `{ "order_id": 123 }` |

## Usage Examples

### Submit Task from Scheduler

```python
from shared.rabbitmq_client import RabbitMQClient
import asyncio

async def submit_task():
    client = RabbitMQClient("scheduler")
    await client.connect()
    
    response = await client.send_request(
        target_service="routine",
        action="submit_task",
        data={
            "arm_id": 1,
            "function": "pick_cup_medium",
            "item": {
                "cup_id": "cup_1",
                "drink": "latte"
            }
        },
        timeout=5
    )
    
    print(f"Task submitted: {response}")
    await client.disconnect()

asyncio.run(submit_task())
```

### Monitor Task Completion

```python
from shared.rabbitmq_client import EventListener

async def monitor_tasks():
    listener = EventListener("monitor")
    await listener.connect()
    
    def handle_completion(data):
        print(f"Task completed: {data['function']} for {data['cup_id']}")
    
    def handle_failure(data):
        print(f"Task failed: {data['function']} - {data.get('error')}")
    
    listener.register_event_handler("routine.task_completed", handle_completion)
    listener.register_event_handler("routine.task_failed", handle_failure)
    
    await listener.subscribe_to_events(["routine.*"])
    await asyncio.Future()

asyncio.run(monitor_tasks())
```

### Clear Queue (Emergency)

```python
async def emergency_clear():
    client = RabbitMQClient("control")
    await client.connect()
    
    response = await client.send_request(
        target_service="routine",
        action="clear_queue",
        data={},
        timeout=5
    )
    
    print(f"Queue cleared: {response}")
    await client.disconnect()
```

### Stop / Resume Order

```python
async def stop_order(order_id: int):
    client = RabbitMQClient("control")
    await client.connect()
    response = await client.send_request(
        target_service="routine",
        action="stop_order",
        data={"order_id": order_id},
        timeout=5
    )
    print(response)
    await client.disconnect()

async def resume_order(order_id: int):
    client = RabbitMQClient("control")
    await client.connect()
    response = await client.send_request(
        target_service="routine",
        action="resume_order",
        data={"order_id": order_id},
        timeout=5
    )
    print(response)
    await client.disconnect()
```

## Dependencies

### Core Dependencies

- **aio-pika** (9.3.1): Async RabbitMQ client used by `shared.rabbitmq_client`
- **pika** (1.3.2): RabbitMQ client (used in some components/tools)
- **python-json-logger** (2.0.7): Structured logging support

### Shared Modules

- `shared.rabbitmq_client`: RabbitMQ client and event listener

## Integration Points

### Downstream Services (Calls To)

1. **Validation Service**
   - Pre-task ingredient checks
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 120 seconds (routine-side timeout; validation may retry internally)
   - **Action**: Dynamic; the validation function name is sent as the RabbitMQ action

2. **Automation Service**
   - Hardware control (grinder, espresso machine, etc.)
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 80 seconds

3. **Robot Arm Service**
   - Robotic arm movements and actions
   - **Protocol**: RabbitMQ RPC (routine calls `robot_arm` via RPC; the robot service may use gRPC internally)
   - **Timeout**: 300 seconds

4. **Scheduler Service**
   - Feedback and state coordination
   - **Protocol**: RabbitMQ RPC
   - **Actions**: `feedback`, `revert_previous_step`, `update_cup_position`, `task_paused`

5. **OMS Service**
   - Stop order flow (used on validation failure)
   - **Protocol**: RabbitMQ RPC
   - **Action**: `stop_order`

### Upstream Services (Receives From)

1. **Scheduler Service**
   - `submit_task`: Task execution requests
   - `clear_queue`: Emergency queue clearing
   - `cancel_order`: Order cancellation
   - **Protocol**: RabbitMQ RPC

### Event Subscriptions

Listens to:
- `system.shutdown`: Graceful shutdown signal

### Event Publications

Publishes to the event bus:
- `routine.task_completed`: Task-level completion (worker outcome)
- `routine.task_failed`: Task-level failure (worker outcome)
- `routine.step_completed`: Step-level completion (executor)
- `routine.completed`: Task-level completion (executor)
- `validation.failed`, `validation.failed.dashboard`, `robot.error`, `automation.error`, `step.error`

**Event Format:**
```json
{
  "cup_id": "cup_1",
  "function": "pull_espresso_shot",
  "arm_id": 2,
  "timestamp": "2025-01-15T10:30:00"
}
```

## Troubleshooting

### Worker Not Processing Tasks

**Issue**: Tasks submitted but not executing

**Solutions:**
1. Check worker logs:
   ```bash
   docker-compose logs routine-service | grep "Worker started"
   ```

2. Verify queue not empty:
   ```python
   response = await client.send_request("routine", "get_queue_status", {})
   print(response)
   ```

3. Restart service:
   ```bash
   docker-compose restart routine-service
   ```

### Task Timeout Errors

**Issue**: Tasks failing with timeout

**Causes:**
- Downstream service not responding
- Task taking longer than timeout allows

**Solutions:**
1. Increase timeout in `executer.py`:
   ```python
   response = await rabbitmq_client.send_request(
       target_service="automation",
       timeout=120  # Increase from 80
   )
   ```

2. Check downstream service health:
   ```bash
   docker-compose ps automation-service robot-arm-service
   ```

### Task Configuration Not Found

**Issue**: "Task config not found for function X"

**Solutions:**
1. Verify tasks.json exists:
   ```bash
   docker exec -it barns-routine cat /app/config/tasks.json
   ```

2. Add missing task to configuration

3. Reload service:
   ```bash
   docker-compose restart routine-service
   ```

### Automation Service Connection Failed

**Issue**: Tasks failing to call automation

**Debugging:**
1. Check automation service:
   ```bash
   docker-compose ps automation-service
   docker-compose logs automation-service
   ```

2. Test RabbitMQ connectivity:
   ```bash
   docker exec -it barns-routine ping rabbitmq
   ```

## Performance Considerations

- **Queue Capacity**: Unlimited (uses asyncio.Queue)
- **Worker Latency**: <10ms task pickup
- **Concurrent Execution**: 2 tasks (one per arm)
- **Memory Usage**: ~50MB base + ~5MB per 100 queued tasks
- **Task Throughput**: 20-30 tasks/minute per arm
- **Timeout Overhead**: Varies by downstream service (30s-300s)

## Security Notes

- No authentication (internal network only)
- RabbitMQ credentials in environment
- All communication over internal Docker network
- Task configurations are read-only after load

## Future Enhancements

- Dynamic task priority adjustment
- Task retry with exponential backoff
- Parallel task execution within single arm
- Task execution history and analytics
- Integrated monitoring panel with InfluxDB metrics
- Circuit breaker pattern for downstream services
- Task execution time prediction
