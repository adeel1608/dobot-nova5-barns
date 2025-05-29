# Automation Service

The Automation Service is responsible for controlling automated equipment during routine handling in the BARNS (Barista Automated Robotic Network System). This service provides a standardized API for interacting with various automation equipment such as grinders, water heaters, milk dispensers, cleaning systems, and conveyor belts.

## Overview

This service acts as a bridge between the routine execution system and physical automation equipment. It provides:

- **Standardized API**: Consistent interface for all automation functions
- **Async Operations**: Non-blocking execution with proper await/response patterns
- **Error Handling**: Comprehensive error reporting and status tracking
- **Extensible Design**: Easy to add new automation functions
- **Health Monitoring**: Built-in health checks and status reporting
- **Modular Structure**: Automation functions are separated in `automation_functions.py` for better organization

## File Structure

```
services/automation/
├── app.py                    # Main FastAPI application
├── automation_functions.py   # All automation function implementations
├── Dockerfile               # Container configuration
├── requirements.txt         # Python dependencies
└── README.md               # This documentation
```

## API Endpoints

### POST /automate
Execute an automation function with specified parameters.

**Request Body:**
```json
{
  "function": "function_name",
  "params": {
    "param1": "value1",
    "param2": "value2"
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Operation completed successfully",
  "details": {
    "duration_sec": 2.5,
    "additional_info": "..."
  }
}
```

### GET /health
Health check endpoint for service monitoring.

**Response:**
```json
{
  "status": "healthy",
  "service": "automation"
}
```

### GET /functions
List all available automation functions.

**Response:**
```json
{
  "functions": ["grind_beans", "heat_water", "dispense_milk", "clean_system", "activate_conveyor"],
  "count": 5
}
```

## Available Automation Functions

### grind_beans
Grind coffee beans with specified settings.

**Parameters:**
- `grind_size` (string): Grind size ("coarse", "medium", "fine") - default: "medium"
- `amount_g` (number): Amount in grams - default: 18

**Example:**
```json
{
  "function": "grind_beans",
  "params": {
    "grind_size": "fine",
    "amount_g": 18
  }
}
```

### heat_water
Heat water to specified temperature.

**Parameters:**
- `target_temp_c` (number): Target temperature in Celsius - default: 93
- `volume_ml` (number): Volume in milliliters - default: 250

**Example:**
```json
{
  "function": "heat_water",
  "params": {
    "target_temp_c": 93,
    "volume_ml": 250
  }
}
```

### dispense_milk
Dispense milk from automated milk system.

**Parameters:**
- `milk_type` (string): Type of milk ("regular", "soya", "almond", "oat") - default: "regular"
- `amount` (number): Amount in milliliters - default: 120
- `temperature` (string): Temperature ("cold", "warm") - default: "cold"

**Example:**
```json
{
  "function": "dispense_milk",
  "params": {
    "milk_type": "soya",
    "amount": 120
  }
}
```

### clean_system
Run automated cleaning cycle.

**Parameters:**
- `cycle_type` (string): Type of cleaning ("rinse", "full") - default: "rinse"

**Example:**
```json
{
  "function": "clean_system",
  "params": {
    "cycle_type": "full"
  }
}
```

### activate_conveyor
Control conveyor belt movement.

**Parameters:**
- `direction` (string): Direction ("forward", "backward") - default: "forward"
- `duration_sec` (number): Duration in seconds - default: 5
- `speed` (string): Speed ("slow", "normal", "fast") - default: "normal"

**Example:**
```json
{
  "function": "activate_conveyor",
  "params": {
    "direction": "forward",
    "duration_sec": 10,
    "speed": "slow"
  }
}
```

## Integration with Routine Service

The automation service is integrated into the routine execution system through the `executer.py` file. Automation steps can be added to task configurations using the following format:

```json
{
  "type": "automation",
  "function": "function_name",
  "params": {
    "param1": "value1",
    "param2": "value2"
  }
}
```

### Example Task Configuration

```json
{
  "soya_latte": {
    "steps": [
      {
        "type": "automation",
        "function": "grind_beans",
        "params": {"grind_size": "fine", "amount_g": 18}
      },
      {
        "type": "automation",
        "function": "heat_water",
        "params": {"target_temp_c": 93, "volume_ml": 250}
      },
      {
        "type": "automation",
        "function": "dispense_milk",
        "params": {"milk_type": "soya", "amount": 120}
      },
      {
        "type": "robot",
        "function": "move_to_steam_wand",
        "params": {}
      }
    ]
  }
}
```

## Adding New Automation Functions

To add a new automation function:

1. **Define the function** in `automation_functions.py`:
```python
async def new_automation_function(params: dict):
    """Description of the function"""
    # Extract parameters
    param1 = params.get("param1", default_value)
    
    # Simulate or implement actual automation
    await asyncio.sleep(duration)  # For simulation
    
    return {
        "success": True,
        "message": "Operation completed",
        "details": {
            "param1": param1,
            "duration_sec": duration
        }
    }
```

2. **Add to function mapping** in `automation_functions.py`:
```python
AUTOMATION_FUNCTIONS = {
    # ... existing functions ...
    "new_automation_function": new_automation_function,
}
```

3. **Update documentation** in this README file.

## Error Handling

The service provides comprehensive error handling:

- **Function Not Found**: Returns error when requested function doesn't exist
- **Parameter Validation**: Validates input parameters
- **Execution Errors**: Catches and reports runtime errors
- **Service Communication**: Handles network and communication errors

## Development and Testing

### Running Locally
```bash
cd services/automation
uvicorn app:app --host 0.0.0.0 --port 8000 --reload
```

### Testing with curl
```bash
# Test automation function with soya milk
curl -X POST http://localhost:8005/automate \
  -H "Content-Type: application/json" \
  -d '{"function": "dispense_milk", "params": {"milk_type": "soya", "amount": 120}}'

# Test grind beans function
curl -X POST http://localhost:8005/automate \
  -H "Content-Type: application/json" \
  -d '{"function": "grind_beans", "params": {"grind_size": "fine", "amount_g": 18}}'

# Health check
curl http://localhost:8005/health

# List functions
curl http://localhost:8005/functions
```

## Docker Deployment

The service is containerized and can be deployed using Docker Compose:

```bash
docker-compose up automation
```

The service will be available at `http://localhost:8005`.

## Environment Variables

- `AUTOMATION_SERVICE_URL`: URL for the automation service (default: "http://automation:8000")

## Future Enhancements

- **Real Hardware Integration**: Replace simulation with actual equipment drivers
- **Status Monitoring**: Real-time status reporting for equipment
- **Configuration Management**: Dynamic configuration for different equipment types
- **Logging and Metrics**: Enhanced logging and performance metrics
- **Safety Interlocks**: Safety checks and emergency stop functionality
- **Milk Type Validation**: Validate available milk types against inventory
- **Equipment Status**: Monitor milk dispenser levels and equipment health 