# POS Integration Documentation

## Overview

The POS Integration system is a Flask API that processes Point of Sale (POS) orders and converts them into a standardized format for the coffee validation system. It handles ingredient modifications, kitchen notes, and generates flat ingredient lists with proper validation requirements.

## System Architecture

### Core Components

1. **Reference Data System**
   - `menu_items`: Complete drink recipes with default ingredients
   - `ingredients`: ID-to-name mapping for all ingredients  
   - `ingredient_details`: Detailed properties for each ingredient

2. **Processing Pipeline**
   - Order parsing and validation
   - Ingredient modification application
   - Kitchen modifier processing
   - Flat ingredient list generation
   - Validation requirement filtering

3. **API Endpoints**
   - `/process-order`: Main order processing endpoint
   - `/filter-for-validation`: Extract validation requirements
   - `/health`: System health check

## Reference Data Structure

### Menu Items
```json
{
  "menu_items": {
    "cappuccino_9oz": {
      "name": "cappuccino_9oz",
      "category": "coffee",
      "size": "9oz", 
      "automation": "full",
      "recipe": "Cappuccino",
      "default_ingredients": [
        {
          "ingredient_id": "espresso_shot_regular",
          "category": "espresso",
          "type": "regular",
          "quantity": 1,
          "base_units": "shots",
          "automated": "full",
          "temperature_sensitive": true,
          "needs_validation": true
        },
        {
          "ingredient_id": "213411",
          "category": "milk", 
          "type": "whole_fat",
          "quantity": 110,
          "base_units": "ml",
          "automated": "full",
          "temperature_sensitive": true,
          "foam_sensitive": true,
          "needs_validation": true
        },
        {
          "ingredient_id": "cup_H9",
          "category": "cups",
          "type": "H9", 
          "quantity": 1,
          "base_units": "units",
          "automated": "full",
          "needs_validation": true
        }
      ]
    }
  }
}
```

### Ingredient Details
```json
{
  "ingredient_details": {
    "213411": {
      "category": "milk",
      "type": "whole_fat",
      "base_units": "ml",
      "automated": "full",
      "temperature_sensitive": true,
      "foam_sensitive": true, 
      "needs_validation": true,
      "default_amount": 150
    },
    "vanilla_syrup": {
      "category": "syrups",
      "type": "vanilla",
      "base_units": "ml",
      "automated": "full", 
      "temperature_sensitive": false,
      "needs_validation": true,
      "default_amount": 8
    }
  }
}
```

## Order Processing Logic

### 1. Input Order Structure

```json
{
  "transaction_id": "TXN-12345",
  "date": "2024-06-12",
  "time": "10:30:00", 
  "store_number": "STORE-001",
  "pos_reg_id": "REG-01",
  "customer_id": "CUST-123",
  "items": [
    {
      "item_id": "cappuccino_9oz",
      "quantity": 1,
      "kitchen_notes": [
        {
          "type": "Temperature",
          "detail": "extra_hot",
          "qty": 0
        },
        {
          "type": "Foam", 
          "detail": "extra_foam",
          "qty": 0
        },
        {
          "type": "Sugar",
          "detail": "Brown Sugar",
          "qty": 2
        }
      ],
      "item_ingredients": [
        {
          "itemId": "213420",
          "initialItemId": "213411", 
          "qty": 1,
          "isAddon": false,
          "isModified": true
        },
        {
          "itemId": "vanilla_syrup",
          "qty": 1,
          "isAddon": true,
          "isModified": false
        }
      ]
    }
  ]
}
```

### 2. Processing Steps

#### Step A: Base Ingredient Loading
- Load default ingredients from `menu_items[item_id].default_ingredients`
- For "cappuccino_9oz": espresso shot, whole milk, H9 cup

#### Step B: Kitchen Notes Processing
Kitchen notes are processed into two categories:

**Automation Modifiers (qty = 0):**
- Temperature controls: "extra_hot", "less_hot", "normal" 
- Foam controls: "extra_foam", "light_foam", "no_foam"
- Ice controls: "extra_ice", "light_ice", "no_ice"

**Manual Notes (qty > 0):**
- Sugar additions, special instructions
- Displayed to barista, not automated

```python
kitchen_modifiers = {
    "temperature": "extra_hot",
    "foam": "extra_foam", 
    "ice_level": "normal"
}

manual_notes = [
    {
        "type": "Sugar",
        "quantity": 2,
        "detail": "Brown Sugar"
    }
]
```

#### Step C: Ingredient Modifications

**Replacement Modifications (isAddon=False, isModified=True):**
- Replace whole milk (213411) with oat milk (213420)
- Maintains original quantity but updates all other properties
- Gets default_amount from ingredient_details

**Addon Modifications (isAddon=True):**
- Add vanilla syrup to the drink
- Uses default_amount from ingredient_details
- Creates new ingredient entry

#### Step D: Flat Ingredient List Generation
Creates final ingredient list with:
- Applied kitchen modifiers
- Calculated total amounts
- Temperature/foam applications
- Validation flags

### 3. Example Processing Flow

**Input Order:** Cappuccino 9oz with Oat Milk + Vanilla Syrup + Extra Hot + Extra Foam

**Base Ingredients:**
```json
[
  {
    "ingredient_id": "espresso_shot_regular",
    "quantity": 1,
    "category": "espresso"
  },
  {
    "ingredient_id": "213411", 
    "quantity": 110,
    "category": "milk"
  },
  {
    "ingredient_id": "cup_H9",
    "quantity": 1, 
    "category": "cups"
  }
]
```

**After Modifications:**
```json
[
  {
    "ingredient_id": "espresso_shot_regular",
    "quantity": 1,
    "category": "espresso"
  },
  {
    "ingredient_id": "213420",
    "quantity": 110, 
    "category": "milk",
    "modified": true,
    "original_ingredient": "213411"
  },
  {
    "ingredient_id": "vanilla_syrup",
    "quantity": 1,
    "category": "syrups",
    "is_addon": true
  },
  {
    "ingredient_id": "cup_H9", 
    "quantity": 1,
    "category": "cups"
  }
]
```

**Final Parsed Output:**
```json
{
  "transaction_id": "TXN-12345",
  "date": "2024-06-12", 
  "time": "10:30:00",
  "store_number": "STORE-001",
  "pos_reg_id": "REG-01",
  "customer_id": "CUST-123",
  "items": [
    {
      "line": 1,
      "recipe_id": "cappuccino_9oz",
      "recipe_name": "cappuccino_9oz",
      "recipe": "Cappuccino",
      "category": "coffee",
      "size": "9oz",
      "ordered_qty": 1,
      "automation": "full",
      "manual_notes": [
        {
          "type": "Sugar",
          "quantity": 2,
          "detail": "Brown Sugar"
        }
      ],
      "ingredients": [
        {
          "category": "espresso",
          "type": "regular",
          "ingredient_id": "espresso_shot_regular", 
          "ingredient_name": "espresso_shot_regular",
          "quantity": 1,
          "unit_amount": 1,
          "total_amount": 1,
          "automated": "full",
          "needs_validation": true,
          "temperature": "extra_hot"
        },
        {
          "category": "milk",
          "type": "oat",
          "ingredient_id": "213420",
          "ingredient_name": "oat_milk",
          "quantity": 110,
          "unit_amount": 150,
          "total_amount": 16500,
          "automated": "full", 
          "needs_validation": true,
          "temperature": "extra_hot",
          "foam": "extra_foam",
          "modified": true,
          "original_ingredient": "213411"
        },
        {
          "category": "syrups",
          "type": "vanilla", 
          "ingredient_id": "vanilla_syrup",
          "ingredient_name": "vanilla_syrup",
          "quantity": 1,
          "unit_amount": 8,
          "total_amount": 8,
          "automated": "full",
          "needs_validation": true,
          "is_addon": true
        },
        {
          "category": "cups",
          "type": "H9",
          "ingredient_id": "cup_H9",
          "ingredient_name": "H9_cup",
          "quantity": 1, 
          "unit_amount": 1,
          "total_amount": 1,
          "automated": "full",
          "needs_validation": true
        }
      ],
      "validation_required": [
        {
          "category": "espresso", 
          "type": "regular",
          "amount": 1
        },
        {
          "category": "milk",
          "type": "oat", 
          "amount": 16500
        },
        {
          "category": "syrups",
          "type": "vanilla",
          "amount": 8
        },
        {
          "category": "cups",
          "type": "H9",
          "amount": 1
        }
      ]
    }
  ]
}
```

## Modification Types

### 1. Milk Substitutions
```json
{
  "itemId": "213420",
  "initialItemId": "213411",
  "qty": 1,
  "isAddon": false,
  "isModified": true
}
```
- Replaces whole milk with oat milk
- Maintains original quantity from recipe
- Updates ingredient properties from ingredient_details

### 2. Syrup/Sauce Additions
```json
{
  "itemId": "vanilla_syrup", 
  "qty": 2,
  "isAddon": true,
  "isModified": false
}
```
- Adds 2 units of vanilla syrup
- Uses default_amount (8ml) per unit = 16ml total

### 3. Extra Shots
```json
{
  "itemId": "espresso_shot_regular",
  "qty": 1, 
  "isAddon": true,
  "isModified": false
}
```
- Adds 1 extra espresso shot
- Uses default_amount (1 shot) from ingredient_details

## Kitchen Modifiers

### Temperature Control
- Applied to temperature_sensitive ingredients (espresso, milk, water)
- Options: "extra_hot", "less_hot", "normal"
- Affects automated equipment settings

### Foam Control  
- Applied to foam_sensitive ingredients (milk)
- Options: "extra_foam", "light_foam", "no_foam"
- Controls milk steaming process

### Ice Control
- Applied to ice category ingredients
- Options: "extra_ice", "light_ice", "no_ice"
- Adjusts ice quantity automatically

## Validation Requirements

The system extracts ingredients that need validation:

```json
"validation_required": [
  {
    "category": "milk",
    "type": "oat", 
    "amount": 16500
  },
  {
    "category": "syrups",
    "type": "vanilla",
    "amount": 8
  }
]
```

These are sent to the validation system for inventory checking and dispensing validation.

## API Usage

### Process Order
```bash
POST /process-order
Content-Type: application/json

{
  "transaction_id": "TXN-12345",
  "items": [...]
}
```

### Filter for Validation  
```bash
POST /filter-for-validation
Content-Type: application/json

{
  "parsed_order": {...}
}
```

## Error Handling

- Unknown menu items are skipped
- Missing ingredient details fall back to defaults
- Invalid modifications are logged but don't break processing
- Kitchen notes with invalid types are ignored

## Integration Points

1. **POS System**: Sends raw orders via /process-order
2. **Validation System**: Receives validation_required ingredients
3. **Dashboard**: Displays manual_notes for barista
4. **Equipment Control**: Uses automation flags and modifiers

This system provides a robust, flexible foundation for processing complex coffee orders with modifications while maintaining data integrity and validation requirements.
