# Scheduler Service

## Purpose and Workflow

The Scheduler Service is responsible for orchestrating the execution of coffee orders by breaking them down into individual tasks and coordinating their execution across multiple robotic arms based on dependencies and resource availability.

### Core Responsibilities
- **Recipe Management**: Loads and manages drink recipes with task dependencies
- **Task Decomposition**: Breaks down orders into individual actionable tasks
- **Dependency Resolution**: Ensures tasks execute in the correct order based on dependencies
- **Resource Allocation**: Coordinates task execution across multiple robotic arms (Arm1, Arm2)
- **Progress Tracking**: Monitors task completion and failure rates
- **OMS Integration**: Communicates order completion/failure status back to OMS

### Workflow
1. **Order Reception**: Receives order from OMS via `/process` endpoint
2. **Task Setup**: Converts order into individual tasks based on drink recipes
3. **Dependency Analysis**: Determines task execution order based on dependencies
4. **Parallel Execution**: Coordinates tasks across available robotic arms
5. **Progress Monitoring**: Tracks task completion via feedback from Routine service
6. **Order Completion**: Notifies OMS when all tasks complete (successfully or with failures)

### Task Execution Flow
```
Order → Recipe Lookup → Task Generation → Dependency Resolution → Arm Assignment → Routine Execution → Feedback Processing → OMS Notification
```

## API Structure

### Core Endpoints

#### Process Order (Called by OMS)
```http
POST /process
Content-Type: application/json

{
  "id": 123,
  "cups": [
    {
      "type": "Latte",
      "size": "regular",
      "addons": []
    }
  ]
}
```

#### Task Feedback (Called by Routine Service)
```http
POST /feedback
Content-Type: application/json

{
  "cup_id": "123-1",
  "action": "steam_milk",
  "success": true,
  "message": "Task completed successfully"
}
```

#### Status Monitoring
```http
GET /status                    # Get current processing status
WebSocket /ws/status          # Real-time status updates
```

### Response Structures

#### Process Order Response
```json
{
  "msg": "Order accepted by scheduler",
  "order_id": 123
}
```

#### Status Response
```json
{
  "order_id": 123,
  "cup_index": 1,
  "step": "Executing steam_milk for Latte (cup 123-1)",
  "status": "in_progress"
}
```

## Recipe System

### Recipe File Structure (`data/recipes.json`)
```json
{
  "Latte": [
    { 
      "action": "pick_cup", 
      "assigned_arm": "Arm1" 
    },
    { 
      "action": "pull_espresso", 
      "assigned_arm": "Arm1", 
      "depends_on": ["pick_cup"] 
    },
    { 
      "action": "steam_milk", 
      "assigned_arm": "Arm2", 
      "depends_on": ["pick_cup"] 
    },
    { 
      "action": "pour_milk", 
      "assigned_arm": "Arm2", 
      "depends_on": ["pull_espresso", "steam_milk"] 
    },
    { 
      "action": "serve", 
      "assigned_arm": "Arm1", 
      "depends_on": ["pour_milk"] 
    }
  ]
}
```

### Recipe Properties
- **action**: Task name to execute
- **assigned_arm**: Which robotic arm performs the task ("Arm1" or "Arm2")
- **depends_on**: Array of prerequisite tasks that must complete first

## Task Management

### Task States
- **pending**: Task is waiting for dependencies or arm availability
- **in_progress**: Task is currently being executed
- **submitted**: Task has been sent to Routine service
- **done**: Task completed successfully
- **failed**: Task failed during execution

### Dependency Resolution
The scheduler uses a dependency graph to ensure tasks execute in the correct order:
1. Tasks with no dependencies start immediately
2. Tasks wait for all dependencies to complete before starting
3. Failed tasks mark the entire order as failed
4. Parallel execution on multiple arms when dependencies allow

## Adding New Modules

### 1. Adding New Drink Recipes

**Step 1**: Add recipe to `data/recipes.json`:
```json
{
  "NewDrink": [
    { 
      "action": "new_action", 
      "assigned_arm": "Arm1" 
    },
    { 
      "action": "another_action", 
      "assigned_arm": "Arm2", 
      "depends_on": ["new_action"] 
    }
  ]
}
```

**Step 2**: Ensure corresponding tasks exist in Routine service

### 2. Adding New Task Actions

**Step 1**: Update recipe files with new action names
**Step 2**: Implement corresponding functionality in Routine service
**Step 3**: Update process library if using simulation mode:
```python
# Add to data/process_library.py
def new_action(cup_id: str, arm: str):
    """Simulate new action."""
    logger.log(f"{arm} is performing new action for {cup_id}")
    time.sleep(2)  # simulation delay
```

### 3. Adding New Arm Types

**Step 1**: Update arm worker initialization in `scheduler.py`:
```python
async def process_order_async(order_id: int, drinks: List[Dict[str, Any]], recipes: Dict[str, List[Dict[str, Any]]]):
    # Add new arm workers
    arm1 = asyncio.create_task(arm_worker("Arm1"))
    arm2 = asyncio.create_task(arm_worker("Arm2"))
    arm3 = asyncio.create_task(arm_worker("Arm3"))  # New arm
    
    await asyncio.wait([arm1, arm2, arm3], return_when=asyncio.ALL_COMPLETED)
```

**Step 2**: Update recipes to use new arm:
```json
{
  "action": "specialized_task",
  "assigned_arm": "Arm3"
}
```

### 4. Adding Advanced Scheduling Logic

**Step 1**: Create new scheduler module:
```python
# scheduler/advanced_scheduler.py
class AdvancedScheduler:
    def __init__(self):
        self.priority_queue = []
        self.resource_optimizer = ResourceOptimizer()
    
    def optimize_task_assignment(self, tasks):
        """Implement advanced scheduling algorithms."""
        pass
```

**Step 2**: Integrate with main scheduler:
```python
# In scheduler.py
from .advanced_scheduler import AdvancedScheduler

advanced_scheduler = AdvancedScheduler()

async def process_order_async(...):
    # Use advanced scheduling
    optimized_tasks = advanced_scheduler.optimize_task_assignment(tasks)
```

### 5. Adding Custom Feedback Handlers

**Step 1**: Create feedback processor:
```python
# scheduler/feedback_processor.py
class FeedbackProcessor:
    def __init__(self):
        self.analytics = AnalyticsCollector()
    
    def process_task_feedback(self, cup_id: str, action: str, success: bool, metrics: dict):
        """Process feedback with additional analytics."""
        # Custom processing logic
        self.analytics.record_task_performance(action, success, metrics)
```

**Step 2**: Update feedback handling:
```python
def handle_routine_feedback(cup_id: str, action: str, success: bool, metrics: dict = None):
    """Enhanced feedback handling."""
    feedback_processor.process_task_feedback(cup_id, action, success, metrics)
    # ... existing logic
```

### 6. Adding Status Monitoring Extensions

**Step 1**: Create status monitor:
```python
# scheduler/status_monitor.py
class StatusMonitor:
    def __init__(self):
        self.performance_metrics = {}
        self.alert_thresholds = {}
    
    def check_performance_alerts(self):
        """Monitor performance and trigger alerts."""
        pass
```

**Step 2**: Integrate monitoring:
```python
# Add to app.py
@app.get("/metrics")
def get_performance_metrics():
    """Get detailed performance metrics."""
    return status_monitor.get_metrics()
```

## Configuration

### Environment Variables
```env
ROUTINE_SERVICE_URL=http://routine:8000    # Routine service endpoint
OMS_SERVICE_URL=http://oms:8000           # OMS service endpoint
```

### Recipe Configuration
- Recipe files are loaded from `data/recipes.json`
- Recipes define task sequences and dependencies
- ARM assignments determine resource allocation

## Development Setup

1. **Install Dependencies**:
   ```bash
   pip install fastapi uvicorn httpx asyncio
   ```

2. **Run Service**:
   ```bash
   uvicorn services.scheduler.app:app --host 0.0.0.0 --port 8000 --reload
   ```

3. **Test Recipe Loading**:
   ```python
   from services.scheduler.scheduler import load_recipes
   recipes = load_recipes('services/scheduler/data/recipes.json')
   print(recipes)
   ```

## Integration Points

### With OMS Service
- **Receives**: Order processing requests
- **Sends**: Order completion/failure notifications

### With Routine Service  
- **Sends**: Individual task execution requests
- **Receives**: Task completion feedback

### With Dashboard
- **Provides**: Real-time status updates via WebSocket

## Error Handling

### Task Failure Scenarios
1. **Submission Failure**: Task fails to submit to Routine service
2. **Execution Failure**: Task fails during execution at Routine service
3. **Timeout**: Task doesn't complete within expected timeframe

### Order Failure Handling
- Any task failure marks the entire order as failed
- Detailed failure reasons are provided to OMS
- Failed task information is logged for debugging

## Performance Considerations

- **Async Processing**: All task coordination is asynchronous
- **Parallel Execution**: Multiple arms can work simultaneously
- **Resource Optimization**: Dependency resolution allows maximum parallelization
- **Failure Fast**: Orders fail quickly when tasks encounter errors
- **Memory Management**: Task state is cleaned up after order completion 