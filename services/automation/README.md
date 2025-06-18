# Automation Service

The Automation Service handles automated equipment control for coffee brewing operations in the BARNS system. It provides standardized automation functions for heating, dispensing, and testing operations.

## Features

- **Async Operations**: Non-blocking execution with proper await/response patterns
- **Error Handling**: Comprehensive error reporting and status tracking
- **Extensible Design**: Easy to add new automation functions
- **Health Monitoring**: Built-in health checks and status reporting
- **Event-Driven**: Publishes automation events for system coordination

## File Structure

```
services/automation/
├── app.py                    # Main service application
├── automation_functions.py   # Automation function implementations
├── Dockerfile.rabbitmq      # Container configuration
├── requirements.txt         # Python dependencies
└── README.md               # This documentation
```

## Available Automation Functions

### heat_water
Heat water to specified temperature.

**Parameters:**
- `target_temp_c` (number): Target temperature in Celsius - default: 93
- `volume_ml` (number): Volume in milliliters - default: 250

**Response:**
```json
{
  "success": true,
  "message": "Heated 250ml water to 93°C",
  "details": {
    "target_temperature": 93,
    "volume": 250,
    "actual_temperature": 93,
    "duration_sec": 3
  }
}
```

### dispense_milk
Dispense milk from automated milk system.

**Parameters:**
- `milk_type` (string): Type of milk - default: "regular"
- `amount` (number): Amount in milliliters - default: 120
- `temperature` (string): Temperature - default: "cold"

**Response:**
```json
{
  "success": true,
  "message": "Dispensed 120ml of regular milk",
  "details": {
    "milk_type": "regular",
    "amount_ml": 120,
    "temperature": "cold",
    "duration_sec": 1.5
  }
}
```

### automation_test1 / automation_test2
Test functions for system validation.

**Response:**
```json
{
  "success": true,
  "message": "automation_test1 passed successfully",
  "details": {
    "test_name": "automation_test1",
    "params_received": {},
    "duration_sec": 0.5,
    "service": "automation"
  }
}
```

## API Endpoints (RabbitMQ)

### Automation Request
```python
# Request
{
    "function": "heat_water",
    "params": {
        "target_temp_c": 93,
        "volume_ml": 250
    }
}

# Response  
{
    "success": true,
    "message": "Heated 250ml water to 93°C",
    "details": {...}
}
```

### Health Check
```python
# Response
{
    "status": "healthy",
    "service": "automation",
    "timestamp": "2024-01-15T10:30:00Z",
    "available_functions": 4
}
```

### List Functions
```python
# Response
{
    "functions": ["heat_water", "dispense_milk", "automation_test1", "automation_test2"],
    "count": 4,
    "success": true
}
```

### Stop Automation
```python
# Response
{
    "success": true,
    "message": "Automation processes stopped"
}
```

## Adding New Automation Functions

To add a new automation function:

1. **Define the function** in `automation_functions.py`:
```python
async def new_function(params: dict):
    """Description of the function."""
    param1 = params.get("param1", default_value)
    
    # Simulate or implement automation
    await asyncio.sleep(duration)
    
    return {
        "success": True,
        "message": "Operation completed",
        "details": {
            "param1": param1,
            "duration_sec": duration
        }
    }
```

2. **Add to function mapping**:
```python
AUTOMATION_FUNCTIONS = {
    # ... existing functions ...
    "new_function": new_function,
}
```

## Integration with Routine Service

Automation functions are called by the routine service through RabbitMQ:

```python
# In task configuration
{
    "type": "automation",
    "function": "heat_water",
    "params": {"target_temp_c": 85, "volume_ml": 200}
}
```

## Events Published

- `automation.started`: When automation function begins
- `automation.completed`: When automation function completes
- `automation.error`: When automation function fails
- `automation.stopped`: When automation is manually stopped
- `automation.emergency_stopped`: When emergency stop is triggered

## Error Handling

The service provides comprehensive error handling:

- **Function Not Found**: Returns error when requested function doesn't exist
- **Parameter Validation**: Validates input parameters  
- **Execution Errors**: Catches and reports runtime errors
- **Event Publishing**: Publishes error events for system coordination

## Testing

Test the service using the built-in test functions:

```bash
# Check service health
docker logs barns-automation

# Verify service is running
docker ps --filter name=barns-automation
```

## Container Status

The automation service runs as a Docker container with:
- **Health checks**: Container health monitoring
- **Auto-restart**: Automatic restart on failure  
- **RabbitMQ integration**: Event-driven communication
- **Async execution**: Non-blocking automation operations

Check status: `docker ps --filter name=barns-automation`

## Development Guidelines

1. **Keep operations realistic** - Simulate actual equipment timing
2. **Provide detailed responses** - Include operation details and timing
3. **Handle errors gracefully** - Return appropriate error responses
4. **Use descriptive messages** - Clear success/failure messages
5. **Include duration tracking** - For performance monitoring 