# Sauce Category Split - Changelog

## Overview
This document outlines all changes made to split the "sauce" category from "syrups" into its own distinct category in the BARNS system.

## Date: 2026-01-25

## Summary
Previously, sauce items (white_chocolate_sauce, caramel_sauce, condense_milk_sauce) were grouped under the "syrups" category. They have now been split into their own "sauce" category for better organization and inventory management.

---

## Changes Made

### 1. Backend - POS Core (`services/oms/pos_core.py`)

**Modified:** `_map_ingredient_id()` function

Added sauce category mapping to convert sauce ingredient IDs to numeric pump numbers:

```python
elif category == "sauce":
    # Try to map using type or id
    mapped_id = SAUCE_MAPPINGS.get(ingredient_type) or SAUCE_MAPPINGS.get(ingredient_id)
    return mapped_id if mapped_id is not None else ingredient_id
```

**Impact:** Sauce ingredients are now properly mapped to their respective pump numbers (10, 11, 12).

---

### 2. Configuration - Tasks (`config/tasks.json`)

**Added:** New "Dispense Sauce" task definition

```json
"Dispense Sauce": {
  "steps": [
    {
      "type": "automation",
      "function": "dispense_sauce",
      "params": {}
    }
  ]
}
```

**Impact:** Recipes can now explicitly call the "Dispense Sauce" action, which routes to the `dispense_sauce()` automation function.

---

### 3. Frontend - Inventory Store (`services/barns-dashboard/src/store/inventoryStore.js`)

**Removed:** All filters that excluded the "sauce" category

**Modified locations:**
- Line ~81: `fetchCategoryInfoData()` - Removed sauce filter
- Line ~115: `fetchFullStockSummaryData()` - Removed sauce filter
- Line ~180: `fetchInventoryStatus()` - Removed sauce filter
- Line ~209: `updateCategorySummary()` - Removed sauce filter

**Added:** Sauce category to initial state

```javascript
categorySummary: {
  milk: { level: 'unknown', numeric: 0, last_refilled: null },
  coffee_beans: { level: 'unknown', numeric: 0, last_refilled: null },
  syrups: { level: 'unknown', numeric: 0, last_refilled: null },
  sauce: { level: 'unknown', numeric: 0, last_refilled: null },  // NEW
  cups: { level: 'unknown', numeric: 0, last_refilled: null },
  premixes: { level: 'unknown', numeric: 0, last_refilled: null }
}
```

**Impact:** Sauce inventory is now tracked and displayed separately in the dashboard.

---

### 4. Frontend - Order Customization (`services/barns-dashboard/src/pages/newOrder/components/ItemCustomization.jsx`)

**Modified:** Added 'sauce' to `additiveCategories` array

```javascript
const additiveCategories = ['syrups', 'sauce', 'toppings', 'extras'];
```

**Impact:** Users can now add sauce ingredients with quantity controls when customizing orders in the UI.

---

### 5. Validation - Inventory Rules (`services/validation/inventory_rules.json`)

**Moved:** Sauce items from "syrups" to new "sauce" category

**Before:** sauce items were nested under `"syrups": { "subtypes": { ... } }`

**After:** sauce items are now under their own category:

```json
"sauce": {
  "subtypes": {
    "white_chocolate_sauce": {
      "max_capacity": 2500,
      "warning_threshold": 300,
      "critical_threshold": 150
    },
    "caramel_sauce": {
      "max_capacity": 680,
      "warning_threshold": 300,
      "critical_threshold": 150
    },
    "condense_milk_sauce": {
      "max_capacity": 370,
      "warning_threshold": 200,
      "critical_threshold": 100
    }
  }
}
```

**Impact:** Sauce inventory is tracked separately with its own thresholds and capacities.

---

### 6. Validation - Main Validation (`services/validation/main_validation.py`)

**Added:** New `SAUCE_ID_TO_SUBTYPE` mapping

```python
SAUCE_ID_TO_SUBTYPE = {
    9: "white_chocolate_sauce",
    10: "caramel_sauce",
    11: "condense_milk_sauce",
}
```

**Removed:** Sauce IDs from `SYRUP_ID_TO_SUBTYPE` mapping

**Modified:** Two functions to handle sauce category:
1. `process_ingredient_validation_request()` - Added sauce validation logic
2. `process_ingredient_update_request()` - Added sauce deduction logic

**Impact:** Validation service now properly validates and deducts sauce ingredients separately from syrups.

---

### 7. Validation - App Handlers (`services/validation/app.py`)

**Added:** New `handle_update_sauce()` handler

```python
async def handle_update_sauce(self, data: Dict[Any, Any]) -> Dict[Any, Any]:
    """
    Handle sauce update requests from routine service - deducts ONLY sauces.
    """
    # Implementation similar to handle_update_syrup but for sauce category
```

**Registered:** New handler in RabbitMQ client

```python
self.rabbitmq_client.register_handler("update_sauce", self.handle_update_sauce)
```

**Impact:** Routine service can now send dedicated sauce update requests.

---

## Hardware Mapping

### Sauce Pumps (SAUCE_MAPPINGS in pos_core.py)
- `white_chocolate`: Pump 10
- `caramel`: Pump 11
- `condense_milk`: Pump 12

### Syrup Pumps (SYRUP_MAPPINGS in pos_core.py)
- `normal_water`: Pump 1
- `hazelnut`: Pump 14
- `vanilla`: Pump 7
- `peach_iced_tea`: Pump 9
- `passion_fruit_puree`: Pump 11
- `ice_tea`: Pump 13
- `caramel_syrup`: Pump 23

---

## Database Considerations

### Required Updates - CRITICAL

**For Existing Databases:**

Run the migration script to update sauce items from 'syrups' to 'sauce' category:

```bash
# Execute the migration script
psql -h localhost -U validation_user -d barns_validation -f db-validation-sauce-migration.sql
```

Or manually run:

```sql
-- Update existing sauce entries in validation database
\c barns_validation validation_user;

UPDATE public.inventory 
SET ingredient_type = 'sauce' 
WHERE ingredient_type = 'syrups' 
  AND subtype IN ('white_chocolate_sauce', 'caramel_sauce', 'condense_milk_sauce');
```

**For Fresh Installations:**

The updated `db-validation-schema.sql` already includes sauce items with the correct category.

### Validation Service Fix

Fixed error handling in `inventory_manager.py`:
- Added 'sauce' to `inventory_cache` initialization
- Fixed NoneType error when db_data is None (happens when database doesn't have records for new categories)
- Improved error handling in `load_inventory_data()` method

---

## Testing Checklist

### Database Migration
- [x] Run migration script on validation database (PostgreSQL)
- [x] Run migration script on OMS database (SQLite pos_reference.db)
- [x] Verify sauce items moved from 'syrups' to 'sauce' category
- [x] Check inventory counts are preserved

### Backend Testing  
- [ ] Test order creation with sauce ingredients in frontend
- [ ] Verify sauce inventory tracking in dashboard
- [ ] Test sauce dispensing automation function
- [ ] Verify sauce validation in validation service
- [ ] Test sauce inventory deduction after order completion
- [ ] Check sauce category appears in inventory status API
- [ ] Verify sauce thresholds trigger low inventory warnings
- [ ] Confirm no "NoneType has no attribute 'get'" errors in validation logs

### Frontend Testing
- [ ] Sauce category appears in inventory dashboard
- [ ] Can add sauce ingredients when customizing orders
- [ ] Sauce inventory levels display correctly
- [ ] Low sauce inventory triggers warnings

---

## Migration Notes

### For Existing Data
If you have existing orders or inventory data with sauces under "syrups":
1. Update database records to use "sauce" category
2. Refresh inventory status to reflect new category
3. Clear any cached inventory data in frontend

### Backward Compatibility
- Old recipes using "Dispense Syrup" will still work for syrup items
- Sauce items should be updated to use "Dispense Sauce" action
- The automation functions (`dispense_syrup` and `dispense_sauce`) are separate and independent

---

## Benefits

1. **Better Organization:** Clearer separation between syrups and sauces
2. **Independent Tracking:** Sauce and syrup inventory tracked separately
3. **Accurate Reporting:** Separate analytics for each category
4. **Hardware Alignment:** Matches physical pump configuration
5. **Scalability:** Easier to add new items to either category

---

## Files Modified

1. `services/oms/pos_core.py`
2. `config/tasks.json`
3. `services/barns-dashboard/src/store/inventoryStore.js`
4. `services/barns-dashboard/src/pages/newOrder/components/ItemCustomization.jsx`
5. `services/validation/inventory_rules.json`
6. `services/validation/main_validation.py`
7. `services/validation/app.py`
8. `services/validation/inventory_manager.py`
9. `db-validation-schema.sql`

## Files Created

1. `SAUCE_CATEGORY_SPLIT_CHANGELOG.md` (this file)
2. `db-validation-sauce-migration.sql` (migration script for existing databases)

---

## Related Documentation

- See `services/automation/automation_functions.py` for `dispense_sauce()` implementation
- See `data/recipes.json` for recipe actions using "Dispense Sauce"
- See `services/dispensing/src/main.cpp` for hardware pump configuration

---

## Contact

For questions or issues related to this change, refer to the system architecture documentation or contact the development team.
