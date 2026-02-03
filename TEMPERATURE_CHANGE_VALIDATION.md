# Temperature Change Validation Enhancement

## Problem Identified

When a user adds maximum add-ons at one temperature and then changes to a stricter temperature (e.g., Regular → Extra Hot), the recalculation could result in exceeding the 30% milk substitution limit.

**Example Scenario**:
- User has 260ml milk base
- Selects Regular temp (10% foam) → 234ml foam-reduced milk
- Adds add-ons using 70.2ml (30% of 234ml) → **At limit**
- Changes to Extra Hot (15% foam) → 221ml foam-reduced milk
- Now 70.2ml is **31.7% of 221ml** → **Exceeds limit!**

## Solution Implemented

### 1. Temperature Change Validation

**File**: `services/barns-dashboard/src/pages/newOrder/components/ItemCustomization.jsx`

Added validation in `handleTemperatureChange()` function that:

1. **Simulates** the capacity calculation with the new temperature before applying it
2. **Checks** if the new temperature would exceed 30% milk substitution
3. **Prevents** the change and shows a confirmation dialog if limit would be exceeded
4. **Allows** the change if it stays within limits or makes capacity more lenient

### 2. Visual Warning Indicators

Added `wouldTemperatureExceedCapacity()` helper function that checks each temperature option and displays:

- **⚠ Warning icon** on temperature buttons that would exceed capacity
- **Yellow border** styling for buttons with warnings
- **Tooltip** explaining the issue on hover

### 3. User Flow

```
User clicks temperature button
    ↓
Has add-ons? → NO → Change temperature immediately
    ↓ YES
Calculate new capacity
    ↓
Would exceed 30%? → NO → Change temperature immediately
    ↓ YES
Show confirmation dialog:
    "Changing from X to Y will exceed limit (Z%)"
    Options:
    • OK: Keep current temperature
    • Cancel: Instructions to remove add-ons
    ↓
User choice → Temperature kept or user removes add-ons manually
```

## Code Changes

### `ItemCustomization.jsx`

**Added/Modified Functions**:
1. `handleTemperatureChange()` - Now async with validation logic
2. `wouldTemperatureExceedCapacity()` - New helper to check if temp change would exceed limit

**Updated JSX**:
- Temperature buttons now show warning icon and styling when they would exceed capacity
- Added `would-exceed` CSS class conditionally

### `styles.css`

**New Styles Added**:
```css
.temperature-btn.would-exceed          /* Yellow warning background */
.temperature-btn.would-exceed:hover    /* Darker yellow on hover */
.temp-warning-icon                     /* Warning icon styling */
.temperature-btn.active.would-exceed   /* Active state with warning */
```

## User Experience

### Before Enhancement
- User could change temperature freely
- Capacity indicator would update to show >100%
- No prevention or warning
- Order could be submitted with exceeded capacity

### After Enhancement
- **Visual warning** on temperature buttons that would exceed
- **Confirmation dialog** before problematic temperature change
- **Clear explanation** of the issue with specific percentages
- **User control** to either keep current temp or manually adjust add-ons
- **Prevents silent capacity violations**

## Example Dialog

```
Changing temperature from Regular to Extra Hot will exceed 
the capacity limit (31.7% of 30%).

You have 5 add-on(s) that need 70.2ml of space.

Options:
• Click OK to keep current temperature (Regular)
• Click Cancel to change temperature and remove some add-ons
```

If user clicks Cancel:
```
To change to Extra Hot, you need to free up 3.9ml.

Please remove some add-ons manually before changing temperature.
```

## Testing Scenarios

### Test 1: Regular → Extra Hot (Stricter)
1. Create 12oz Latte
2. Select Regular temp
3. Add 7 pumps of vanilla syrup (near 30% limit)
4. Try to change to Extra Hot
5. **Expected**: Warning icon appears, confirmation dialog shows

### Test 2: Extra Hot → Regular (More Lenient)
1. Create 12oz Latte  
2. Select Extra Hot temp
3. Add 6 pumps of vanilla syrup (at 30% limit)
4. Try to change to Regular
5. **Expected**: Change proceeds, capacity drops to ~27%, controls re-enabled

### Test 3: Kids → Extra Hot (Big Jump)
1. Create 12oz Latte
2. Select Kids temp (0% foam)
3. Add 8 pumps of vanilla syrup
4. Try to change to Extra Hot (15% foam)
5. **Expected**: Warning icon appears, confirmation dialog shows large overage

## Benefits

1. **Prevents Invalid Orders**: Users can't accidentally create orders exceeding capacity
2. **Clear Feedback**: Visual warnings before clicking
3. **User Control**: User decides whether to adjust or cancel
4. **Maintains UX**: Doesn't auto-remove add-ons without permission
5. **Educational**: Dialog explains the math behind the limitation

## Edge Cases Handled

- ✅ Changing to same temperature (no validation)
- ✅ No add-ons present (skip validation)
- ✅ No milk ingredient (skip validation)
- ✅ Calculation happens before state update (prevents flash of invalid state)
- ✅ Warning indicators update when add-ons added/removed

## Files Modified

1. `services/barns-dashboard/src/pages/newOrder/components/ItemCustomization.jsx` (+82 lines)
2. `services/barns-dashboard/src/pages/newOrder/styles.css` (+25 lines)
3. `CUP_CAPACITY_MANAGEMENT.md` (documentation updated)

## Future Enhancements

Potential improvements:
1. Auto-suggest which add-ons to remove to make temp change possible
2. Allow one-click "remove minimum add-ons" to enable temp change
3. Show preview of what capacity would be with each temperature
4. Remember user's preferred temperature per drink type
