# Cup Capacity Management System

## Overview

The Cup Capacity Management System intelligently adjusts milk quantities when customers customize their drinks with add-ons and temperature preferences. The system uses a three-phase approach to ensure drinks stay within cup capacity while maintaining quality.

## Architecture

### Three-Phase Capacity Logic

#### Phase 1: Temperature Foam Reduction
Temperature selection affects the foam content, which reduces the milk quantity upfront:

- **Kids Temperature (54°C)**: 0% foam - No milk reduction
- **Regular Temperature (71.5°C)**: 10% foam - 10% milk reduction
- **Extra Hot (83.5°C)**: 15% foam - 15% milk reduction

**Example**: A 260ml milk base with Extra Hot temperature becomes 221ml (260ml - 39ml foam).

#### Phase 2: Fill Free Cup Space
Add-ons first fill the remaining free space in the cup without affecting milk:

- Calculate: Free Space = Cup Volume - Recipe Volume
- Add-ons fill this space first
- No milk reduction occurs in this phase

**Example**: 355ml cup with 325ml recipe = 30ml free space. First 30ml of add-ons go here.

#### Phase 3: Milk Substitution
Only after free space is exhausted, add-ons start substituting the foam-reduced milk:

- Maximum substitution: 30% of foam-reduced milk
- System prevents exceeding this limit
- User sees "No more add-ons supported" message when limit is reached

## Implementation

### Frontend Components

#### Configuration Constants
**File**: `services/barns-dashboard/src/constants/cupCapacityConfig.js`

- Cup volumes for all sizes (H7, H9, H12, C7, C9, C12, C16)
- Ingredient densities (g/ml) for volume conversion
- Temperature foam percentages
- Maximum milk substitution percentage (30%)

#### Capacity Calculator Utility
**File**: `services/barns-dashboard/src/pages/newOrder/utils/cupCapacityCalculator.js`

Key functions:
- `applyFoamReduction()` - Phase 1 calculation
- `calculateFreeSpace()` - Phase 2 calculation
- `calculateMilkSubstitution()` - Phase 3 calculation
- `calculateAdjustedMilk()` - Main function combining all phases

#### UI Components
**File**: `services/barns-dashboard/src/pages/newOrder/components/ItemCustomization.jsx`

Features:
- Explicit temperature buttons (Kids, Regular, Extra Hot)
- Real-time capacity indicator showing:
  - Current milk substitution percentage
  - Visual progress bar
  - Remaining capacity
  - Foam reduction amount
- Add-on controls disabled when capacity limit reached
- Warning message when limit is hit

### Backend Processing

#### POS Core Module
**File**: `services/oms/pos_core.py`

Constants added:
- `CUP_VOLUMES` - Cup capacity in ml
- `INGREDIENT_DENSITIES` - Density values for weight-to-volume conversion
- `TEMPERATURE_FOAM_PERCENTAGES` - Foam percentages by temperature
- `MAX_MILK_SUBSTITUTION_PERCENT` - 30% limit

Functions:
- `_calculate_milk_adjustments()` - Implements three-phase logic
- Enhanced `_create_ingredient_list()` - Applies adjustments to milk ingredients

Enhanced `ParsedIngredient` dataclass with:
- `adjusted_amount` - Final milk amount after all reductions
- `milk_reduction_percent` - Percentage of milk substituted

## Density-Based Volume Conversion

Add-ons are measured by weight (grams) but occupy volume (ml) in the cup. The system uses density conversion:

```
Volume (ml) = Weight (grams) / Density (g/ml)
```

**Ingredient Densities**:
- Milk: 1.03 g/ml (baseline)
- Syrups: 1.32 g/ml (heavier due to sugar)
- Sauces: 1.35 g/ml (denser)
- Espresso: 1.02 g/ml (similar to water)
- Ice: 0.92 g/ml (floats)

## Example Calculation

### Scenario: 12oz Latte with Extra Hot and 4 Pumps Vanilla

**Given**:
- Cup: H12 (355ml capacity)
- Recipe: 60ml espresso + 260ml milk + 5ml foam = 325ml
- Free space: 355ml - 325ml = 30ml

**Phase 1 - Foam Reduction**:
- Extra Hot = 15% foam
- Milk reduction: 260ml × 15% = 39ml
- Foam-reduced milk: 221ml
- Max substitution allowed: 221ml × 30% = 66.3ml

**Phase 2 - Fill Free Space**:
- 4 pumps vanilla = 40g
- Volume: 40g ÷ 1.32 = 30.3ml
- Free space used: 30ml
- Overflow: 0.3ml

**Phase 3 - Milk Substitution**:
- Overflow requiring substitution: 0.3ml
- Substitution percentage: 0.3ml ÷ 221ml = 0.14%
- Well under 30% limit
- Can add more: YES

**Final Result**:
- Adjusted milk: 221ml - 0.3ml = 220.7ml
- Substitution used: 0.14% of 30%
- Remaining capacity: 66ml volume equivalent

## User Experience

### Temperature Selection
Users see three clear temperature options with:
- Temperature label and degrees Celsius
- Foam percentage indicator
- Active state visual feedback
- **Warning indicator (⚠)** if changing to that temperature would exceed capacity with current add-ons

### Temperature Change Validation
When attempting to change temperature with add-ons already added:
- **If new temperature stays within limit**: Change proceeds normally
- **If new temperature would exceed 30% limit**: 
  - User receives a confirmation dialog explaining the issue
  - Shows exact percentage that would be exceeded
  - Options:
    - Click OK to keep current temperature
    - Click Cancel for instructions to remove add-ons manually
  - Temperature buttons show warning icon (⚠) for temperatures that would exceed

**Example Dialog**:
```
Changing temperature from Regular to Extra Hot will exceed the capacity 
limit (33.2% of 30%).

You have 5 add-on(s) that need 73.4ml of space.

Options:
• Click OK to keep current temperature (Regular)
• Click Cancel to change temperature and remove some add-ons
```

### Capacity Indicator
Real-time visual feedback showing:
- **Stacked bar chart** displaying cup fill composition:
  - **Milk** (light blue): Adjusted milk amount after foam reduction
  - **Foam** (yellow): Foam portion created by temperature
  - **Espresso** (dark brown): Espresso shots
  - **Syrups** (pink): Syrup add-ons
  - **Sauces** (red): Sauce add-ons
  - **Toppings** (purple): Topping add-ons
  - **Extras** (green): Other add-ons
  - **Free Space** (striped gray): Remaining cup capacity
- **Color-coded legend** below the bar showing ml amounts for each component
- **Hover tooltips** on each segment showing precise volumes
- **Warning message** when maximum capacity is reached

### Limit Enforcement
When 30% limit is reached:
- Add-on dropdowns show "Capacity limit reached"
- Plus buttons are disabled
- Warning message: "No more add-ons supported - Maximum milk substitution reached"

## Benefits

1. **Quality Control**: Ensures drinks fit in the cup without overflow
2. **Customer Choice**: Maximizes customization within physical constraints
3. **Transparency**: Users understand capacity limits in real-time
4. **Consistency**: Same logic in frontend (UX) and backend (validation)
5. **Accurate Calculations**: Density-based volume conversion for precise measurements

## Technical Notes

### Frontend Validation
- Calculation happens in real-time as user customizes
- Prevents adding items that would exceed capacity
- Updates immediately on temperature change

### Backend Validation
- Recalculates adjustments during order processing
- Logs warnings if limits are exceeded
- Maintains data integrity

### Density Assumptions
- Default syrup weight: 10g per pump
- Default sauce weight: 10g per pump
- Toppings: 5g per unit
- Extras: 10g per unit

## Future Enhancements

Potential improvements:
1. Configurable maximum substitution percentage per drink type
2. Different foam percentages for different milk types
3. Temperature-specific cup capacity adjustments (thermal expansion)
4. Custom density values per specific ingredient
5. Visual cup fill simulation in UI

## Testing

Key test scenarios:
1. Temperature changes update capacity immediately
2. Adding multiple add-ons correctly fills free space first
3. 30% limit is enforced correctly
4. Removing add-ons frees up capacity
5. Different cup sizes have different capacities
6. Backend calculations match frontend
7. **NEW: Temperature change validation when at capacity**
   - Add maximum add-ons with Regular temp
   - Try to change to Extra Hot → Should show warning
   - Try to change to Kids → Should allow (more lenient)
8. **NEW: Warning indicators on temperature buttons**
   - Add add-ons near limit
   - Check that temperature buttons show warning icon (⚠) for stricter temps

## Support

For questions or issues related to the cup capacity system:
- Frontend: Check `cupCapacityCalculator.js` utility
- Backend: Check `pos_core.py` `_calculate_milk_adjustments()` function
- Configuration: Review `cupCapacityConfig.js` constants
