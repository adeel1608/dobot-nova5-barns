# Validation Retry Fix - Prevent Multiple Validation Failures

## Problem
When a validation failed (e.g., `milk_cup_detection_absent`), the system was processing the same validation failure multiple times in quick succession (up to 6 times within 5 seconds). This caused:
- Duplicate error logs
- Unnecessary processing overhead
- Potential race conditions during order stop

Example from logs:
```
12/3/2025, 10:24:44 AM - [VALIDATION FAILED] Validation milk_cup_detection_absent failed for cup 133-1
12/3/2025, 10:24:43 AM - [VALIDATION FAILED] Validation milk_cup_detection_absent failed for cup 133-1
12/3/2025, 10:24:42 AM - [VALIDATION FAILED] Validation milk_cup_detection_absent failed for cup 133-1
12/3/2025, 10:24:41 AM - [VALIDATION FAILED] Validation milk_cup_detection_absent failed for cup 133-1
12/3/2025, 10:24:40 AM - [VALIDATION FAILED] Validation milk_cup_detection_absent failed for cup 133-1
12/3/2025, 10:24:39 AM - [VALIDATION FAILED] Validation milk_cup_detection_absent failed for cup 133-1
```

## Root Cause
The validation failure was being processed multiple times due to:
1. Tasks being resubmitted before order stop completed
2. Race conditions between validation failure handling and order stop
3. No mechanism to prevent duplicate failure processing

## Solution
Implemented a **validation failure guard mechanism** to ensure each validation failure is processed only once:

### Changes Made

#### 1. Added Validation Failure Tracking (`services/routine/executer.py`)
```python
# VALIDATION FAILURE TRACKING
# Track which cup_id-function combinations have already triggered validation failure handling
# This prevents duplicate failure processing if validation is called multiple times
# Format: set of f"{cup_id}-{function}" strings
validation_failures_in_progress = set()
validation_failures_lock = asyncio.Lock()
```

#### 2. Added Guard Functions
- **`mark_validation_failure_in_progress(cup_id, function)`**: Marks that a validation failure is being processed. Returns `True` if this is the first failure (should process), `False` if already in progress (skip).
- **`clear_validation_failure_in_progress(cup_id, function)`**: Clears the marker when task completes or order stops.
- **`clear_order_validation_failures(order_id)`**: Clears all validation failure markers for an order when it's resumed or cancelled.

#### 3. Updated Validation Failure Handling
When a validation fails, the system now:
1. Checks if this failure is already being processed
2. If already in progress, skips duplicate processing and exits
3. If first occurrence, processes the failure (sends notifications, stops order)
4. Marks the failure as "in progress" to block duplicates

```python
if not res.get("passed", False):
    # Validation failed - check if we should process this failure or if it's already being handled
    log("ERROR", f"[VALIDATION FAILED] Validation {func_name} failed for cup {cup_id}", service="routine")
    
    # Use guard to prevent duplicate processing of same validation failure
    should_process = await mark_validation_failure_in_progress(cup_id, function)
    
    if not should_process:
        # This validation failure is already being processed, skip duplicate handling
        log("WARNING", f"[VALIDATION FAILED] Duplicate validation failure detected for {function} on cup {cup_id}, skipping duplicate processing", service="routine")
        validation_failed_stopped = True
        break  # Exit without processing again
    
    # Process the failure (only if this is the first occurrence)
    ...
```

#### 4. Cleanup on Order Resume
When an order is resumed, all validation failure markers are cleared so validations can be retried:
```python
async def mark_order_resumed(order_id: int):
    """Mark an order as resumed (no longer stopped)."""
    async with stopped_orders_lock:
        stopped_orders.discard(order_id)
        log("INFO", f"Order {order_id} marked as resumed in routine", service="routine")
    
    # Clear validation failure markers so the order can retry validations
    await clear_order_validation_failures(order_id)
```

## Expected Behavior After Fix

### Scenario 1: Validation Fails Once
1. Validation executes and fails
2. System marks failure as "in progress"
3. Sends dashboard notification
4. Publishes validation.failed event
5. Stops the order via OMS
6. Logs appear only ONCE (no duplicates)

### Scenario 2: Duplicate Validation Attempts (Race Condition)
1. First validation fails and starts processing
2. Second validation attempt arrives (race condition)
3. System detects failure already in progress
4. Logs warning: `"Duplicate validation failure detected, skipping duplicate processing"`
5. Second attempt exits without processing
6. Only the first attempt processes the failure

### Scenario 3: Order Resume
1. Order is resumed
2. Validation failure markers are cleared
3. Validation can be retried fresh
4. If validation fails again, it will be processed (not blocked as duplicate)

## Testing Recommendations

1. **Test validation failure**: Trigger a validation failure (e.g., cup present when absent expected) and verify only ONE set of failure logs appears
2. **Test rapid retry**: If system tries validation multiple times quickly, verify only the first failure is processed
3. **Test order resume**: After validation failure and order stop, resume the order and verify validation can be retried
4. **Monitor logs**: Check for the new log messages:
   - `"Marked validation failure in progress for {cup_id}-{function}"`
   - `"Duplicate validation failure detected, skipping duplicate processing"` (if race condition occurs)
   - `"Cleared {N} validation failure markers for order {order_id}"` (on resume)

## Benefits

1. **Cleaner logs**: No more duplicate error messages
2. **Better performance**: Validation failure processing happens only once
3. **Prevents race conditions**: Guards against multiple concurrent failure handlers
4. **Maintains resume capability**: Validation failures can still be retried when order is resumed
5. **Thread-safe**: Uses asyncio locks to prevent concurrent access issues

