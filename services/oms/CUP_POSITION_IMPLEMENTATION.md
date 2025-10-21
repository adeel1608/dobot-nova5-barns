# Cup Position Dynamic Assignment Implementation

## Overview
This document describes the implementation of dynamic cup position assignment based on real-time cup detection results. The system automatically assigns available docking station positions to cups by finding the nearest available position.

## Database Schema

### Cup Position in Database
The `cup_position` is stored in the `menu_item_default_ingredients` table:

```sql
INSERT INTO menu_item_default_ingredients (
    menu_item_id, ingredient_id, category, type, unit_amount, automated,
    temperature_sensitive, needs_validation, quantity, base_units,
    foam_sensitive, is_topping, cup_position
)
VALUES (
    'item_id', 'cup_position', 'position', 'cup_position', 1, 'full',
    0, 0, 1, 'position', 0, 0, NULL
);
```

**Key Points:**
- `ingredient_id`: 'cup_position'
- `category`: 'position'
- `type`: 'cup_position'
- `unit_amount`: The actual position number (1, 2, 3, 4, etc.)
- `cup_position` column: Optional dedicated column (can also use unit_amount)

## Data Flow

### 1. Order Creation (POS → OMS)

When a POS order is created, the system processes it through multiple layers:

```javascript
// Frontend: services/barns-dashboard/src/pages/dashboard/components/NewPOSOrderForm.jsx
// The form displays cup_position as read-only in the "Cup Position" section
```

**Backend Processing (`services/oms/app.py`):**

```python
# Input format
{
    'order': {
        'cups': [{
            'type': 'Cappuccino',
            'size': 'cup_H9',
            'addons': [],
            'ingredients': {
                'milk': {1: 110.0},
                'cups': {'cup_H9': 1.0},
                'temperature': {'regular_temperature': 73.0},
                'espresso': {'espresso_shot_single': 1.0},
                'position': {'cup_position': 1.0}  # Original position from database
            }
        }]
    }
}
```

### 2. Order Parsing (POS Core)

**File: `services/oms/pos_core.py`**

#### Loading from Database

```python
def load_reference_data_from_db(db_file_path: str = "pos_reference.db") -> bool:
    # Check if cup_position column exists
    cursor.execute("PRAGMA table_info(menu_item_default_ingredients)")
    columns = [col[1] for col in cursor.fetchall()]
    has_cup_position = 'cup_position' in columns
    
    # Query includes cup_position if available
    if has_cup_position:
        query = """
            SELECT menu_item_id, ingredient_id, category, type, unit_amount, ..., cup_position
            FROM menu_item_default_ingredients
        """
    
    # Store position value
    if cup_position is not None:
        ingredient_data["cup_position"] = cup_position
    elif category == "position" and ingredient_id == "cup_position":
        # Fallback: use unit_amount as position value
        ingredient_data["cup_position"] = unit_amount
```

#### Ingredient Creation

```python
def _create_ingredient_list(ingredients: List[Dict[str, Any]], kitchen_modifiers: Dict[str, Any]):
    # For position category, use unit_amount as the cup_position value
    cup_pos_value = ingredient.get("cup_position")
    if category == "position" and cup_pos_value is None:
        cup_pos_value = ingredient.get("unit_amount", 1)
    
    ingredient_obj = ParsedIngredient(
        category=ingredient["category"],
        type=ingredient["type"],
        ingredient_id=mapped_ingredient_id,
        ...,
        cup_position=cup_pos_value  # Stored in ParsedIngredient dataclass
    )
```

### 3. Cup Detection & Position Assignment (Routine Executor)

**File: `services/routine/executer.py`**

#### Detection Result Format

When `cup_detection` is called, it returns:

```python
{
    "passed": True,
    "detection_result": {
        1: False,  # Available
        2: False,  # Available
        3: True,   # Occupied
        4: True    # Occupied
    },
    "details": {
        "cups_detected": {1: False, 2: False, 3: True, 4: True},
        "total_positions": 4,
        "detected_count": 2,
        "message": "Detected 2 out of 4 cup positions"
    }
}
```

**True = Occupied, False = Available**

#### Position Reassignment Logic

```python
def find_nearest_available_position(current_position: int, detection_result: dict) -> int:
    """
    Find the nearest available (False) cup position to the current position.
    
    Example:
        current_position = 1
        detection_result = {1: True, 2: False, 3: True, 4: False}
        Returns: 2 (nearest available to position 1)
    """
    # Get all available positions (False values)
    available_positions = [pos for pos, occupied in detection_result.items() if not occupied]
    
    if not available_positions:
        return current_position  # Return original if none available
    
    # If current position is available, use it
    if current_position in available_positions:
        return current_position
    
    # Find nearest available position by calculating absolute distance
    nearest_position = min(available_positions, key=lambda pos: abs(pos - current_position))
    
    return nearest_position
```

#### Integration in Task Processing

```python
async def process_task(arm_id: int, task, configs: dict, rabbitmq_client: RabbitMQClient):
    for step in cfg["steps"]:
        if step_type == "validation":
            res = await call_validation(func_name, params, rabbitmq_client)
            
            # Special handling for cup_detection
            if func_name == "cup_detection" and res.get("passed", False):
                detection_result = res.get("detection_result") or res.get("details", {}).get("cups_detected", {})
                
                if detection_result:
                    # Get current cup_position from task ingredients
                    ingredients = task.get("item", {}).get("ingredients", {})
                    
                    if "cup_position" in ingredients:
                        cup_pos_data = ingredients["cup_position"]
                        if isinstance(cup_pos_data, dict):
                            # Extract position: {'cup_position': 1.0}
                            current_position = int(list(cup_pos_data.values())[0])
                        elif isinstance(cup_pos_data, (int, float)):
                            current_position = int(cup_pos_data)
                    
                    if current_position:
                        # Find nearest available position
                        new_position = find_nearest_available_position(current_position, detection_result)
                        
                        # Update task params with new position
                        if new_position != current_position:
                            logger.info(f"🔄 Updating cup position from {current_position} to {new_position}")
                            
                            # Update the task's ingredient data
                            if isinstance(ingredients["cup_position"], dict):
                                ingredients["cup_position"]["cup_position"] = float(new_position)
                            else:
                                ingredients["cup_position"] = float(new_position)
                            
                            # Also update params for subsequent steps
                            if "cup_position" in params:
                                if isinstance(params["cup_position"], dict):
                                    params["cup_position"]["cup_position"] = float(new_position)
                                else:
                                    params["cup_position"] = float(new_position)
```

## Frontend Display

**File: `services/barns-dashboard/src/pages/dashboard/components/NewPOSOrderForm.jsx`**

The cup position is displayed as a read-only field:

```jsx
{category === 'position' ? (
  <div key={category} className="bg-gray-50 p-3 rounded">
    <label className="block text-xs font-medium text-gray-700 mb-1 capitalize">
      Cup Position
    </label>
    {/* Show current cup position (from defaults) as read-only */}
    <div className="text-xs bg-white px-2 py-1 rounded">
      {(item.selectedMenuItem.default_ingredients || [])
        .filter(di => di.category === 'position')
        .map((di, idx) => (
          <div key={`position-${idx}`}>
            {di.cup_position || di.type || 'Default Position'} (fixed)
          </div>
        ))}
    </div>
  </div>
) : (
  // Other ingredient categories...
)}
```

## Example Workflow

### Scenario: Order with Position 1, but Position 1 is Occupied

1. **Order Created:**
   ```json
   {
     "type": "Cappuccino",
     "ingredients": {
       "position": {"cup_position": 1.0}
     }
   }
   ```

2. **Cup Detection Results:**
   ```python
   {1: True, 2: False, 3: True, 4: False}
   # Position 1 is occupied, 2 and 4 are available
   ```

3. **Position Reassignment:**
   ```
   Current: 1
   Available: [2, 4]
   Distance to 2: |2-1| = 1
   Distance to 4: |4-1| = 3
   Nearest: 2
   ```

4. **Updated Task:**
   ```json
   {
     "type": "Cappuccino",
     "ingredients": {
       "position": {"cup_position": 2.0}  // Updated from 1 to 2
     }
   }
   ```

5. **Robot uses position 2 for this cup**

## Key Files Modified

1. **services/oms/pos_core.py**
   - Added `cup_position` field to `ParsedIngredient` dataclass
   - Updated `load_reference_data_from_db()` to load cup_position from database
   - Updated `_create_ingredient_list()` to extract cup_position from unit_amount for position category

2. **services/oms/app.py**
   - Updated `/pos/process-order` endpoint to include position in ingredients
   - Ensured position category is properly grouped in order structure

3. **services/barns-dashboard/src/pages/dashboard/components/NewPOSOrderForm.jsx**
   - Added read-only display for cup_position in the customize section
   - Prevents modification of position category (fixed value)

4. **services/routine/executer.py**
   - Added `find_nearest_available_position()` helper function
   - Updated `process_task()` to dynamically reassign cup positions based on detection
   - Integrated cup detection result processing

5. **services/validation/main_validation.py**
   - Added `detection_result` at top level of response for easy access
   - Maintains backward compatibility with `details.cups_detected`

## Benefits

1. **Dynamic Position Assignment**: Automatically finds available docking stations
2. **Nearest Position Logic**: Minimizes robot movement by selecting nearest available position
3. **Fail-safe**: Falls back to original position if no detection results available
4. **Real-time Adaptation**: Responds to actual cup presence in docking stations
5. **Idempotent**: If original position is available, it uses that position

## Testing

To test the cup position assignment:

1. Create an order with a specific cup position (e.g., position 1)
2. Place cups in docking stations to simulate occupancy
3. Run cup detection during order processing
4. Verify that the system assigns the nearest available position
5. Check logs for position reassignment messages

## Logs to Monitor

```
🔍 Cup detection result: {1: False, 2: False, 3: True, 4: True}
Original position 1 is available, no change needed
```

Or if reassignment occurs:

```
🔍 Cup detection result: {1: True, 2: False, 3: True, 4: False}
Original position 1 is occupied. Using nearest available: 2
Available positions: [2, 4]
🔄 Updating cup position from 1 to 2 for cup cup_001
✅ Cup position updated successfully for cup cup_001
```

