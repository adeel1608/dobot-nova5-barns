# Scheduler Service

The Scheduler Service orchestrates order processing by breaking down drink orders into tasks and coordinating execution across robotic arms based on dependencies and resource availability.

## Features

- **Recipe Management**: Loads and manages drink recipes with task dependencies
- **Task Decomposition**: Breaks down orders into individual actionable tasks
- **Dependency Resolution**: Ensures tasks execute in the correct order
- **Resource Allocation**: Coordinates task execution across multiple robotic arms
- **Progress Tracking**: Monitors task completion and provides feedback
- **Event-Driven**: Communicates via RabbitMQ for reliable message delivery

## File Structure

```
services/scheduler/
├── app.py                    # Main service application
├── scheduler.py             # Task orchestration and dependency logic
├── data/
│   ├── recipes.json         # Drink recipes and task definitions
│   └── logger.py           # Logging utilities
├── Dockerfile.rabbitmq     # Container configuration
├── requirements.txt        # Python dependencies
└── README.md              # This documentation
```

## Core Workflow

1. **Order Reception**: Receives order from OMS via RabbitMQ
2. **Recipe Lookup**: Loads drink recipe from recipes.json
3. **Task Generation**: Creates individual tasks based on recipe steps
4. **Dependency Resolution**: Determines task execution order
5. **Arm Coordination**: Assigns tasks to available robotic arms
6. **Task Submission**: Sends tasks to Routine service for execution
7. **Progress Monitoring**: Tracks completion via feedback from Routine
8. **Order Completion**: Notifies OMS when all tasks complete

## Available Recipes

### Current Recipes (from data/recipes.json)

## API Endpoints (RabbitMQ)

### Process Order
```python
# Request from OMS
{
    "id": 123,
    "cups": [
        {
            "type": "pipelinetest",
            "size": "regular",
            "addons": []
        }
    ]
}

# Response
{
    "success": True,
    "message": "Order accepted by scheduler",
    "order_id": 123
}
```

### Task Feedback
```python
# Request from Routine Service
{
    "cup_id": "123-1",
    "action": "test1",
    "success": True,
    "message": "Task completed successfully"
}

# Response
{
    "success": True,
    "status": "received"
}
```

### Get Status
```python
# Response
{
    "success": True,
    "status": {
        "order_id": 123,
        "cup_index": 1,
        "step": "Executing test1 for pipelinetest (cup 123-1)",
        "status": "in_progress"
    },
    "timestamp": "2024-01-15T10:30:00Z"
}
```

### Subscribe to Status Updates
```python
# Request
{
    "service_name": "dashboard"
}

# Response
{
    "success": True,
    "subscribed": True
}
```

### Health Check
```python
# Response
{
    "status": "healthy",
    "service": "scheduler",
    "timestamp": "2024-01-15T10:30:00Z",
    "loaded_recipes": 3,
    "status_subscribers": 1
}
```

## Task Management

### Task States
- **pending**: Task is waiting for dependencies or arm availability
- **submitted**: Task has been sent to Routine service
- **completed**: Task completed successfully
- **failed**: Task failed during execution

### Dependency Resolution
The scheduler uses a dependency graph to ensure correct execution order:
1. Tasks with no dependencies start immediately
2. Tasks wait for all dependencies to complete
3. Failed tasks mark the entire order as failed
4. Parallel execution on multiple arms when dependencies allow

### Recipe Properties
- **action**: Task name to execute (must match Routine service functions)
- **assigned_arm**: Which robotic arm performs the task ("Arm1" or "Arm2")
- **depends_on**: Array of prerequisite tasks that must complete first

## Adding New Recipes

1. **Add recipe to `data/recipes.json`**:
```json
{
    "new_drink": [
        {
            "action": "step1",
            "assigned_arm": "Arm1",
            "depends_on": []
        },
        {
            "action": "step2",
            "assigned_arm": "Arm2",
            "depends_on": ["step1"]
        }
    ]
}
```

2. **Ensure functions exist** in Routine service configurations

3. **Restart service** to reload recipes

## Events Published

- `scheduler.order_received`: When order is received from OMS
- `scheduler.order_processing_started`: When order processing begins
- `scheduler.order_completed`: When order completes successfully
- `scheduler.order_failed`: When order fails
- `scheduler.order_error`: When order encounters errors
- `scheduler.feedback_processed`: When feedback is processed
- `scheduler.status_update`: Status updates for subscribers

## Integration with Other Services

### OMS Service
- **Receives**: Order processing requests
- **Sends**: Order completion/failure notifications

### Routine Service
- **Sends**: Individual task execution requests
- **Receives**: Task completion feedback

### Dashboard
- **Provides**: Real-time status updates via events

## Error Handling

The service provides comprehensive error handling:

- **Recipe Not Found**: Returns error when drink recipe doesn't exist
- **Task Failures**: Handles individual task failures and marks order as failed
- **Service Communication**: Handles timeouts and connection errors
- **Dependency Violations**: Ensures proper task ordering

### Order Failure Scenarios
1. **Invalid Recipe**: Drink type not found in recipes.json
2. **Task Submission Failure**: Cannot submit task to Routine service
3. **Task Execution Failure**: Task fails during execution
4. **Timeout**: Order processing exceeds time limits

## Testing

Test the service using available recipes:

```bash
# Check service health
docker logs barns-scheduler

# Verify service is running and healthy
docker ps --filter name=barns-scheduler

# Check recipe loading
# Service logs show: "Loaded 3 recipes: ['latte', 'americano', 'pipelinetest']"
```

## Container Status

The scheduler service runs as a Docker container with:
- **Health checks**: Container health monitoring
- **Auto-restart**: Automatic restart on failure
- **RabbitMQ integration**: Event-driven communication
- **Recipe loading**: Dynamic recipe loading from JSON
- **Status tracking**: Real-time order progress monitoring

Check status: `docker ps --filter name=barns-scheduler`

## Development Guidelines

1. **Keep recipes simple** - Break complex drinks into discrete steps
2. **Define clear dependencies** - Ensure proper task ordering
3. **Use appropriate arm assignments** - Balance workload across arms
4. **Test new recipes** - Verify all referenced actions exist in Routine service
5. **Monitor performance** - Track task completion times and success rates

## Configuration

### Recipe Configuration
- **Location**: `data/recipes.json`
- **Format**: JSON with drink definitions
- **Loading**: Automatic on service startup

### Arm Configuration
- **Arm1**: Primary arm for sequential operations
- **Arm2**: Secondary arm for parallel operations
- **Dependencies**: Define which tasks must complete before others

## Performance Considerations

- **Async Processing**: All task coordination is asynchronous
- **Parallel Execution**: Multiple arms work simultaneously when possible
- **Resource Optimization**: Dependency resolution maximizes parallelization
- **Fast Failure**: Orders fail quickly when tasks encounter errors
- **Memory Management**: Task state cleaned up after order completion 