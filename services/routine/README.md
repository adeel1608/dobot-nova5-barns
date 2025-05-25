# Routine Service

## Purpose and Workflow

The Routine Service is responsible for executing individual robotic tasks by coordinating validation checks and robot operations. It serves as the bridge between high-level task requests from the Scheduler and low-level robot/validation operations.

### Core Responsibilities
- **Task Execution**: Executes multi-step robotic routines based on predefined configurations
- **Validation Integration**: Coordinates with Validation service for quality checks
- **Robot Coordination**: Manages robot arm operations and movements
- **Event Publishing**: Publishes step-by-step execution events for monitoring
- **Feedback Reporting**: Reports task completion status back to Scheduler
- **Error Handling**: Manages failures and provides detailed error information

### Workflow
1. **Task Reception**: Receives task requests from Scheduler service
2. **Configuration Lookup**: Loads task configuration from `config/tasks.json`
3. **Step Execution**: Executes validation and robot steps in sequence
4. **Progress Monitoring**: Publishes events for each completed step
5. **Error Handling**: Aborts on validation/robot failures with detailed error reporting
6. **Completion Notification**: Sends success/failure feedback to Scheduler

### Execution Flow
```
Task Request → Config Lookup → Step Execution (Validation/Robot) → Event Publishing → Feedback to Scheduler
```

## API Structure

### Core Endpoints

#### Execute Task (Called by Scheduler)
```http
POST /task
Content-Type: application/json

{
  "arm_id": 1,
  "function": "steam_milk",
  "item": {
    "cup_id": "123-1",
    "cup_size": "regular",
    "drink": "Latte",
    "addons": []
  }
}
```

#### Health Check
```http
GET /health               # Service health status
```

### Response Structures

#### Task Execution Response
```json
{
  "status": "accepted",
  "message": "Task processing started",
  "arm_id": 1,
  "function": "steam_milk"
}
```

#### Error Response
```json
{
  "detail": "Unknown function",
  "function": "invalid_task",
  "available_functions": ["pull_espresso", "steam_milk", "pour_milk", "sprinkle_cocoa"]
}
```

## Task Configuration System

### Configuration File Structure (`config/tasks.json`)
```json
{
  "pull_espresso": {
    "steps": [
      {
        "type": "validation",
        "function": "check_cup_present",
        "params": {}
      },
      {
        "type": "validation",
        "function": "check_beans",
        "params": {"min_grams": 18}
      },
      {
        "type": "robot",
        "function": "move_to_espresso_head",
        "params": {}
      },
      {
        "type": "robot",
        "function": "activate_pump",
        "params": {"duration_sec": 30}
      },
      {
        "type": "validation",
        "function": "check_weight",
        "params": {"min_weight": 30}
      }
    ]
  }
}
```

### Step Types
- **validation**: Calls Validation service for quality checks
- **robot**: Calls Robot service for physical operations

### Step Properties
- **type**: Step category ("validation" or "robot")
- **function**: Specific function to call
- **params**: Parameters to pass to the function

## Event Publishing

### Event Types
- **routine.step_completed**: Individual step completion
- **routine.completed**: Full task completion
- **validation.failed**: Validation step failure
- **robot.error**: Robot operation failure

### Event Structure
```json
{
  "event_type": "routine.step_completed",
  "data": {
    "arm": 1,
    "cup": "123-1",
    "step": "check_cup_present"
  }
}
```

## Integration Points

### With Scheduler Service
- **Receives**: Task execution requests
- **Sends**: Task completion feedback

### With Validation Service
- **Calls**: Validation functions for quality checks
- **Receives**: Pass/fail results with details

### With Robot Service
- **Calls**: Robot movement and operation functions
- **Receives**: Success/failure status

## Adding New Modules

### 1. Adding New Task Configurations

**Step 1**: Add task definition to `config/tasks.json`:
```json
{
  "new_task": {
    "steps": [
      {
        "type": "validation",
        "function": "new_validation_check",
        "params": {"threshold": 10}
      },
      {
        "type": "robot",
        "function": "new_robot_action",
        "params": {"speed": "slow"}
      }
    ]
  }
}
```

**Step 2**: Ensure corresponding functions exist in Validation and Robot services

### 2. Adding New Step Types

**Step 1**: Update step execution logic in `executer.py`:
```python
async def process_task(arm_id: int, task, configs: dict):
    for step in cfg["steps"]:
        step_type = step["type"]
        if step_type == "validation":
            # Existing validation logic
        elif step_type == "robot":
            # Existing robot logic
        elif step_type == "new_step_type":
            # New step type logic
            res = await call_new_service(func_name, params)
            if not res.get("success", False):
                success = False
                break
```

**Step 2**: Implement corresponding service call function:
```python
async def call_new_service(func_name: str, params: dict):
    """Call new service type."""
    url = f"{NEW_SERVICE_URL}/{func_name}"
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(url, json=params)
            return response.json()
    except Exception as e:
        return {"success": False, "error": str(e)}
```

### 3. Adding Custom Event Publishers

**Step 1**: Create custom event publisher:
```python
# routine/custom_events.py
class CustomEventPublisher:
    def __init__(self):
        self.event_handlers = []
    
    def publish_custom_event(self, event_type: str, data: dict):
        """Publish custom events with additional processing."""
        # Custom processing logic
        enhanced_data = self.enhance_event_data(data)
        publish_event(event_type, enhanced_data)
    
    def enhance_event_data(self, data: dict) -> dict:
        """Add custom data enrichment."""
        return {**data, "timestamp": time.time(), "service": "routine"}
```

**Step 2**: Integrate custom publisher:
```python
# In executer.py
from .custom_events import CustomEventPublisher

custom_publisher = CustomEventPublisher()

async def process_task(...):
    # Use custom publisher
    custom_publisher.publish_custom_event("custom.task_started", {...})
```

### 4. Adding Task Preprocessing

**Step 1**: Create task preprocessor:
```python
# routine/preprocessor.py
class TaskPreprocessor:
    def __init__(self):
        self.optimization_rules = {}
    
    def preprocess_task(self, task_request: dict) -> dict:
        """Preprocess task before execution."""
        # Add preprocessing logic
        optimized_params = self.optimize_parameters(task_request)
        return {**task_request, **optimized_params}
    
    def optimize_parameters(self, request: dict) -> dict:
        """Optimize task parameters based on context."""
        # Optimization logic
        pass
```

**Step 2**: Integrate preprocessing:
```python
# In app.py
from .preprocessor import TaskPreprocessor

preprocessor = TaskPreprocessor()

@app.post("/task")
async def execute_task(task: TaskRequest):
    # Preprocess task
    processed_task = preprocessor.preprocess_task(task.dict())
    # Continue with execution
```

### 5. Adding Advanced Error Handling

**Step 1**: Create error handler:
```python
# routine/error_handler.py
class AdvancedErrorHandler:
    def __init__(self):
        self.retry_strategies = {}
        self.error_categories = {}
    
    def handle_error(self, error: Exception, context: dict) -> dict:
        """Handle errors with retry logic and categorization."""
        error_type = self.categorize_error(error)
        retry_action = self.get_retry_strategy(error_type)
        
        return {
            "should_retry": retry_action.should_retry,
            "retry_delay": retry_action.delay,
            "error_category": error_type,
            "recovery_action": retry_action.recovery_action
        }
```

### 6. Adding Performance Monitoring

**Step 1**: Create performance monitor:
```python
# routine/performance_monitor.py
class PerformanceMonitor:
    def __init__(self):
        self.metrics = {}
        self.timing_data = {}
    
    def start_task_timing(self, task_id: str):
        """Start timing a task."""
        self.timing_data[task_id] = {"start": time.time()}
    
    def end_task_timing(self, task_id: str, success: bool):
        """End timing and record metrics."""
        if task_id in self.timing_data:
            duration = time.time() - self.timing_data[task_id]["start"]
            self.record_metric(task_id, duration, success)
```

## Environment Variables

```env
VALIDATION_SERVICE_URL=http://validation:8000    # Validation service endpoint
ROBOT_SERVICE_URL=http://robot:8000             # Robot service endpoint
SCHEDULER_SERVICE_URL=http://scheduler:8000      # Scheduler service endpoint
```

## Configuration Files

### Task Configurations
- **Location**: `config/tasks.json`
- **Purpose**: Defines available tasks and their step sequences
- **Format**: JSON with task definitions

### Service Endpoints
- **Validation Service**: Configurable via environment variables
- **Robot Service**: Configurable via environment variables

## Development Setup

1. **Install Dependencies**:
   ```bash
   pip install fastapi uvicorn httpx asyncio
   ```

2. **Run Service**:
   ```bash
   uvicorn services.routine.app:app --host 0.0.0.0 --port 8000 --reload
   ```

3. **Test Task Configuration Loading**:
   ```python
   import json
   with open('services/routine/config/tasks.json') as f:
       tasks = json.load(f)
   print(tasks.keys())
   ```

## Error Handling

### Validation Failures
- Task aborts immediately on validation failure
- Detailed failure reason provided in feedback
- Event published for monitoring

### Robot Operation Failures
- Task aborts on robot operation failure
- Error details captured and reported
- Recovery actions can be configured

### Service Communication Failures
- Timeout handling for external service calls
- Retry logic for transient failures
- Fallback operations for critical failures

## Testing

### Unit Testing
```bash
# Test individual functions
python -m pytest services/routine/tests/test_executer.py

# Test configuration loading
python -m pytest services/routine/tests/test_config.py
```

### Integration Testing
```bash
# Test with validation service
curl -X POST "http://localhost:8000/task" \
  -H "Content-Type: application/json" \
  -d '{"arm_id": 1, "function": "steam_milk", "item": {"cup_id": "test-1"}}'
```

## Performance Considerations

- **Async Processing**: All external service calls are asynchronous
- **Event Publishing**: Non-blocking event publishing to avoid delays
- **Error Recovery**: Quick failure detection and reporting
- **Resource Management**: Proper cleanup of resources after task completion
- **Concurrent Execution**: Support for multiple simultaneous tasks on different arms 