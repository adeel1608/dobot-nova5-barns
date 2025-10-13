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
5. **Event Publishing**: Send `routine.task_completed` or `routine.task_failed`
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

## API/Endpoints

### Action: `submit_task`
Submit a task to execution queue.

**Request:**
```json
{
  "arm_id": 1,
  "function": "pick_cup_medium",
  "item": {
    "cup_id": "cup_1",
    "drink": "latte",
    "size": "medium"
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Task submitted to Arm 1 queue",
  "queue_size": 3
}
```

### Action: `get_queue_status`
Get current queue status for all arms.

**Response:**
```json
{
  "success": true,
  "arm_1_queue_size": 2,
  "arm_2_queue_size": 1,
  "total_tasks": 3
}
```

### Action: `clear_queue`
Clear all pending tasks from queues.

**Request:**
```json
{
  "arm_id": 1  // Optional: specific arm, omit for all
}
```

**Response:**
```json
{
  "success": true,
  "message": "Cleared 5 tasks from queues"
}
```

### Action: `cancel_order`
Cancel all tasks for a specific order.

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
  "message": "Cancelled all tasks for order 123",
  "tasks_cancelled": 8
}
```

### Action: `health`
Health check.

**Response:**
```json
{
  "status": "healthy",
  "service": "routine",
  "arm_1_worker": "running",
  "arm_2_worker": "running",
  "queue_sizes": {
    "arm_1": 0,
    "arm_2": 0
  }
}
```

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
        print(f"✅ Task completed: {data['function']} for {data['cup_id']}")
    
    def handle_failure(data):
        print(f"❌ Task failed: {data['function']} - {data.get('error')}")
    
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

## Dependencies

### Core Dependencies

- **aio-pika** (9.3.1): Async RabbitMQ client
- **httpx** (0.25.2): HTTP client for service calls
- **grpcio** (1.60.0): gRPC for robot communication

### Shared Modules

- `shared.rabbitmq_client`: RabbitMQ client and event listener

## Integration Points

### Downstream Services (Calls To)

1. **Validation Service**
   - Pre-task ingredient checks
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 30 seconds

2. **Automation Service**
   - Hardware control (grinder, espresso machine, etc.)
   - **Protocol**: RabbitMQ RPC
   - **Timeout**: 80 seconds

3. **Robot Arm Service**
   - Robotic arm movements and actions
   - **Protocol**: RabbitMQ RPC / gRPC
   - **Timeout**: 300 seconds

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

Broadcasts to `barns_events` exchange:
- `routine.task_completed`: Task successfully executed
- `routine.task_failed`: Task execution failed

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
- Grafana/Prometheus metrics integration
- Circuit breaker pattern for downstream services
- Task execution time prediction
