# BARNS Validation Service

A simplified validation service providing essential validation functions and inventory management for the BARNS coffee automation system.

## 🎯 Purpose

The Validation Service provides:
- **Test Validation Functions**: Used by routine service for testing workflows
- **Inventory Validation**: Ingredient availability checking and inventory updates
- **Inventory Management**: Real-time inventory tracking and refill handling
- **Event Publishing**: Threshold warnings and status updates

## 📁 Structure

```
services/validation/
├── app.py                    # Main service application
├── validations/              # Validation functions
│   ├── __init__.py          # Function loader
│   ├── tests.py             # Test validation functions  
│   └── inventory.py         # Inventory functions
├── requirements.txt
└── README.md
```

## 🔧 Core Functions

### Test Functions (Used by Routine Service)
- `validate_test1`: Basic integration test validation
- `validate_test2`: Comprehensive system test validation

### Inventory Functions (Used by Routine Service)
- `check_ingredient_availability`: Check if sufficient ingredients are available
- `update_inventory`: Update inventory levels after ingredient usage

## 📡 API Endpoints (RabbitMQ)

### 1. Validation Function Execution
**Handler:** `validate`

```python
# Request
{
    "function": "check_ingredient_availability",
    "params": {
        "ingredient": "whole_milk",
        "amount_needed": 2
    }
}

# Response
{
    "passed": true,
    "details": "Sufficient whole_milk available: 80 >= 2",
    "data": {
        "ingredient": "whole_milk",
        "available": 80,
        "needed": 2
    }
}
```

### 2. Health Check
**Handler:** `health`

```python
# Response
{
    "status": "healthy",
    "service": "validation",
    "timestamp": "2024-01-15T10:30:00Z",
    "loaded_validators": 4,
    "available_functions": ["validate_test1", "validate_test2", "check_ingredient_availability", "update_inventory"]
}
```

### 3. Inventory Status (Used by Dashboard & OMS)
**Handler:** `inventory_status`

```python
# Request - Get all inventory
{}

# Response
{
    "success": true,
    "inventory": {
        "whole_milk": {
            "level": "high",
            "numeric": 80,
            "last_refilled": "2024-01-15T08:30:00Z"
        },
        "coffee_beans": {
            "level": "medium",
            "numeric": 45,
            "last_refilled": "2024-01-14T10:15:00Z"
        },
        "paper_cups": {
            "level": "low",
            "numeric": 25,
            "last_refilled": "2024-01-12T14:20:00Z"
        }
    }
}

# Request - Get specific ingredient
{
    "ingredient": "whole_milk"
}

# Response
{
    "success": true,
    "ingredient": "whole_milk",
    "status": {
        "level": 80,
        "threshold_low": 20,
        "threshold_medium": 50,
        "last_refilled": "2024-01-15T08:30:00Z"
    }
}
```

### 4. Inventory Refill (Used by OMS)
**Handler:** `inventory_refill`

```python
# Request
{
    "ingredient": "whole_milk",
    "amount": 100
}

# Response
{
    "success": true,
    "ingredient": "whole_milk",
    "new_level": 100
}
```

### 5. Category Summary (Used by Dashboard)
**Handler:** `inventory_category_summary`

```python
# Response
{
    "success": true,
    "category_summary": {
        "milk": {
            "level": "medium",
            "numeric": 45,
            "last_refilled": null
        },
        "beans": {
            "level": "high",
            "numeric": 85,
            "last_refilled": null
        },
        "syrups": {
            "level": "low",
            "numeric": 15,
            "last_refilled": null
        },
        "cups": {
            "level": "medium",
            "numeric": 60,
            "last_refilled": null
        }
    }
}
```

## 📦 Current Inventory Items

**Essential ingredients tracked:**

```python
# Milk products
"whole_milk", "skim_milk", "almond_milk", "soy_milk"

# Coffee essentials  
"coffee_beans"

# Basic syrups
"vanilla_syrup", "caramel_syrup", "chocolate_syrup"

# Cups
"paper_cups", "plastic_cups"
```

**Threshold Levels:**
- **Low**: Red status, triggers high-severity warnings
- **Medium**: Yellow status, triggers medium-severity warnings  
- **High**: Green status, normal operation

## 📢 Events Published

### Validation Events
- `validation.completed`: When a validation function completes
- `validation.threshold_warning`: When inventory levels drop below thresholds

### Inventory Events
- `inventory.refilled`: When inventory is successfully refilled

## 🔄 Response Format

All validation functions return:

```python
{
    "passed": bool,      # Required: True if validation passed
    "details": str,      # Required: Human-readable description
    "data": dict        # Optional: Additional data
}
```

All API handlers return:

```python
{
    "success": bool,     # Required: True if request succeeded
    "error": str,       # Optional: Error message if failed
    "...": "data"       # Response-specific data
}
```

## 🚀 Development

### Running the Service
```bash
cd services/validation
python app.py
```

### Adding New Test Functions
1. Add function to `validations/tests.py`:
```python
def validate_test3(params: dict) -> dict:
    return {
        "passed": True,
        "details": "Test 3 passed",
        "data": {"test_name": "validate_test3"}
    }
```

2. Register in `validations/__init__.py`:
```python
return {
    "validate_test1": validate_test1,
    "validate_test2": validate_test2,
    "validate_test3": validate_test3,  # Add here
    "check_ingredient_availability": check_ingredient_availability,
    "update_inventory": update_inventory
}
```

### Adding New Inventory Items
1. Add to `INVENTORY_LEVELS` in `validations/inventory.py`:
```python
"new_ingredient": {
    "level": random.randint(20, 100), 
    "threshold_low": 20, 
    "threshold_medium": 50, 
    "last_refilled": None
}
```

2. Update category mapping in `get_category_summary()` if needed.

## 🧪 Testing

### Test Validation Functions
```bash
# Via RabbitMQ message (requires RabbitMQ running)
{
    "function": "validate_test1",
    "params": {"test": "integration"}
}
```

### Test Inventory Functions
```bash
# Check ingredient availability
{
    "function": "check_ingredient_availability", 
    "params": {"ingredient": "whole_milk", "amount_needed": 5}
}

# Update inventory usage
{
    "function": "update_inventory",
    "params": {"ingredient": "whole_milk", "amount_used": 3}
}
```

## 🐳 Container Status

Check validation service status:
```bash
docker ps --filter name=barns-validation
docker logs barns-validation
```

The service runs with:
- **Health monitoring**: Automatic health checks
- **Auto-restart**: Restart on failure  
- **Event integration**: Real-time communication with other services
- **Inventory persistence**: In-memory tracking (would be database in production)

---

**Key Benefits of Simplified Design:**
- ✅ **Focused**: Only essential functions used by other services
- ✅ **Maintainable**: Clear, simple codebase
- ✅ **Reliable**: Fewer components, fewer failure points
- ✅ **Fast**: Lightweight with minimal overhead 