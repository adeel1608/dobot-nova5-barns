# Routine Service

The Routine Service orchestrates task execution for robotic arms by managing task queues and coordinating with validation and automation services in the BARNS system.

## Features

- **Task Queue Management**: Manages separate queues for each robotic arm
- **Multi-Step Execution**: Executes complex tasks with validation and automation steps
- **Service Coordination**: Coordinates with validation and automation services
- **Event-Driven**: Publishes events for task progress and completion
- **Error Handling**: Comprehensive error handling with feedback to scheduler
- **Health Monitoring**: Built-in health checks and queue status monitoring

## File Structure

```
services/routine/
├── app.py                    # Main service application with queue management
├── executer.py              # Task execution logic and service coordination
├── Dockerfile.rabbitmq      # Container configuration
├── requirements.txt         # Python dependencies
└── README.md               # This documentation
```

## Core Workflow

1. **Task Reception**: Receives task requests from Scheduler service via RabbitMQ
2. **Queue Management**: Routes tasks to appropriate arm queues (Arm 1 or Arm 2)
3. **Task Execution**: Workers process tasks by executing configured steps
4. **Step Processing**: Executes validation and automation steps in sequence
5. **Service Coordination**: Calls validation and automation services via RabbitMQ
6. **Feedback**: Reports completion status back to Scheduler
7. **Event Publishing**: Publishes progress events for monitoring

## Available Task Configurations

### Default Tasks (when config file not found)

#### test1
```json
{
    "steps": [
        {"type": "validation", "function": "check_cup_present", "params": {}},
        {"type": "validation", "function": "validate_test1", "params": {}},
        {"type": "automation", "function": "automation_test1", "params": {}}
    ]
}
```

#### test2
```json
{
    "steps": [
        {"type": "validation", "function": "check_temperature", "params": {"target_temperature": 85}},
        {"type": "validation", "function": "validate_test2", "params": {}},
        {"type": "automation", "function": "automation_test2", "params": {}}
    ]
}
```

## API Endpoints (RabbitMQ)

### Submit Task
```python
# Request
{
    "arm_id": 1,
    "function": "test1",
    "item": {
        "cup_id": "123-1",
        "cup_size": "regular", 
        "drink": "test_drink",
        "addons": []
    }
}

# Response
{
    "status": "queued",
    "arm_id": 1,
    "function": "test1",
    "queue_size": 1,
    "success": True
}
```

### Health Check
```python
# Response
{
    "status": "healthy",
    "service": "routine",
    "timestamp": "2024-01-15T10:30:00Z",
    "arm_queues": {
        "1": 0,
        "2": 0
    },
    "available_functions": ["test1", "test2"]
}
```

### Get Queue Status
```python
# Request (specific arm)
{
    "arm_id": 1
}

# Request (all arms)
{}

# Response
{
    "all_arms": {
        "1": 0,
        "2": 1
    },
    "success": True
}
```

### Clear Queue
```python
# Request (specific arm)
{
    "arm_id": 1
}

# Request (all arms)
{}

# Response
{
    "cleared_arms": [1, 2],
    "success": True
}
```

## Task Configuration System

### Configuration File Location
- **Path**: `/app/config/tasks.json` (configurable via `ROUTINE_CONFIG_PATH`)
- **Format**: JSON with task definitions

### Step Types
- **validation**: Calls validation service for quality checks
- **automation**: Calls automation service for equipment operations

### Step Properties
- **type**: Step category ("validation" or "automation")
- **function**: Specific function to call in the target service
- **params**: Parameters to pass to the function

### Example Custom Configuration
```json
{
    "custom_task": {
        "steps": [
            {
                "type": "validation",
                "function": "check_ingredient_availability",
                "params": {
                    "ingredient": "milk",
                    "amount_needed": 2
                }
            },
            {
                "type": "automation", 
                "function": "dispense_milk",
                "params": {
                    "milk_type": "regular",
                    "amount": 120
                }
            },
            {
                "type": "validation",
                "function": "update_inventory",
                "params": {
                    "ingredient": "milk",
                    "amount_used": 2
                }
            }
        ]
    }
}
```

## Events Published

- `routine.task_queued`: When task is added to queue
- `routine.task_completed`: When task completes successfully  
- `routine.task_failed`: When task fails with error details
- `routine.queue_cleared`: When arm queue is cleared
- `routine.all_queues_cleared`: When all queues are cleared
- `routine.completed`: When individual task finishes (legacy)

## Integration with Other Services

### Scheduler Service
- **Receives**: Task submission requests
- **Sends**: Task completion/failure feedback

### Validation Service
- **Calls**: Validation functions for quality checks
- **Receives**: Pass/fail results with details

### Automation Service
- **Calls**: Automation functions for equipment operations
- **Receives**: Success/failure status with timing information

## Adding New Tasks

1. **Create task configuration** in `/app/config/tasks.json`:
```json
{
    "new_task": {
        "steps": [
            {
                "type": "validation",
                "function": "your_validation_function",
                "params": {"param1": "value1"}
            },
            {
                "type": "automation",
                "function": "your_automation_function", 
                "params": {"param2": "value2"}
            }
        ]
    }
}
```

2. **Ensure functions exist** in validation and automation services

3. **Restart service** to reload configuration

## Error Handling

The service provides comprehensive error handling:

- **Invalid Arm ID**: Returns error for unsupported arm IDs
- **Unknown Function**: Returns error when task configuration doesn't exist
- **Step Failures**: Aborts task execution on validation/automation failures
- **Service Communication**: Handles timeouts and connection errors
- **Queue Management**: Proper cleanup of failed tasks

## Testing

Test the service using available functions:

```bash
# Check service health
docker logs barns-routine

# Verify service is running and healthy
docker ps --filter name=barns-routine

# Check queue status
# (Use RabbitMQ management interface or API Bridge)
```

## Container Status

The routine service runs as a Docker container with:
- **Health checks**: Container health monitoring
- **Auto-restart**: Automatic restart on failure
- **RabbitMQ integration**: Event-driven communication
- **Worker management**: Asynchronous arm workers
- **Queue persistence**: In-memory task queues

Check status: `docker ps --filter name=barns-routine`

## Development Guidelines

1. **Keep tasks modular** - Break complex operations into simple steps
2. **Handle failures gracefully** - Provide detailed error information
3. **Use appropriate step types** - Validation for checks, automation for actions
4. **Test configurations** - Verify all referenced functions exist
5. **Monitor performance** - Track task execution times and success rates

## Environment Variables

- `ROUTINE_CONFIG_PATH`: Path to task configuration file (default: `/app/config/tasks.json`)

## Performance Considerations

- **Async Processing**: All service calls are asynchronous
- **Worker Isolation**: Each arm has dedicated worker to prevent blocking
- **Event Publishing**: Non-blocking event publishing
- **Resource Cleanup**: Proper task cleanup on completion/failure
- **Concurrent Execution**: Support for multiple simultaneous tasks 