# Timeout Race Condition Fix

## Problem Description

Order 327 was successfully processed and marked as completed at 16:35:37, but then received a timeout error at 16:41:43 and 16:43:38 (approximately 6 and 8 minutes later), which incorrectly changed the order status from COMPLETED to ERROR.

### Timeline
1. **16:35:37** - Order 327 completed successfully
   - Scheduler notified OMS of completion
   - OMS marked order as COMPLETED
   - OMS confirmed receipt
2. **16:41:43** - First timeout error
   - Scheduler sent order_failed event (timeout after 8.7 minutes)
   - OMS updated order from COMPLETED to ERROR
3. **16:43:38** - Second timeout error
   - Scheduler sent another order_failed event
   - OMS rejected it (already in ERROR state)

## Root Cause

The issue had three components:

### 1. Arm Workers Never Exit
In `services/scheduler/scheduler.py`, the `arm_worker()` function was designed to be persistent and never exit. When an order completed, it would just continue looping and waiting for new tasks instead of exiting.

```python
# Check if current order is complete (all tasks done or failed)
elif current_tasks_total > 0 and completed_count + failed_count >= current_tasks_total:
    # Only log once per order completion
    if not order_completion_logged:
        log("INFO", f"{arm_name} worker finished - order complete")
        order_completion_logged = True
    task = None
    consecutive_no_work_count = 0
    # Workers never exited - just continued looping!
```

### 2. Timeout Always Fires
In `process_order_async()`, the function uses `asyncio.wait_for()` to wait for both arm workers with a timeout:

```python
await asyncio.wait_for(
    asyncio.gather(arm1, arm2, return_exceptions=True),
    timeout=dynamic_timeout
)
```

Since the arm workers never exit, the `wait_for` would always timeout, even if the order completed successfully. The timeout didn't check if the order was already completed before sending a failure notification.

### 3. OMS Overwrites Completed Status
In `services/oms/app.py`, the `handle_order_failed_event()` function only checked if an order was already in ERROR state, but didn't check if it was already COMPLETED:

```python
# Only checked for ERROR, not COMPLETED
if order and order.get("status") == ORDER_STATUS['ERROR']:
    return {"success": True, "acknowledged": True}

# This would overwrite COMPLETED with ERROR!
db.update_order_status(order_id, ORDER_STATUS['ERROR'], error)
```

## Solution

### Fix 1: Make Arm Workers Exit on Completion
Modified `arm_worker()` in `services/scheduler/scheduler.py` to exit when order completes:

```python
# Check if current order is complete (all tasks done or failed)
elif current_tasks_total > 0 and completed_count + failed_count >= current_tasks_total:
    if not order_completion_logged:
        log("INFO", f"{arm_name} worker finished - order complete")
        order_completion_logged = True
    task = None
    consecutive_no_work_count = 0
    # Exit the worker when order completes so process_order_async can finish
    break
```

Also made workers exit when stopped:

```python
if order_stopped:
    submitted_tasks = [t for t in tasks if t["status"] == "submitted"]
    if len(submitted_tasks) == 0:
        if not order_stopped_logged:
            log("INFO", f"{arm_name} worker stopped - no submitted tasks")
            order_stopped_logged = True
        task = None
        # Exit the worker when stopped and no tasks are pending
        break
```

### Fix 2: Check Completion Before Timeout Notification
Modified the timeout handler in `process_order_async()` to check if order was already completed:

```python
except asyncio.TimeoutError:
    # Check if order was already completed before sending timeout error
    with lock:
        if order_completion_notified:
            log("INFO", f"Order {order_id} was already completed/failed before timeout, ignoring timeout")
            return completed_count == tasks_total and failed_count == 0
    
    log("ERROR", f"Order {order_id} timed out after {dynamic_timeout/60:.1f} minutes")
    # ... send failure notification
```

### Fix 3: Prevent Overwriting Completed Status in OMS
Modified `handle_order_failed_event()` in `services/oms/app.py` to check for COMPLETED status:

```python
# Check if order is already in a final state
current_status = order.get("status", "").upper() if order else None

if order and current_status == ORDER_STATUS['ERROR']:
    log("ERROR", f"Order {order_id} already in error state")
    return {"success": True, "acknowledged": True}

# Don't overwrite completed orders with error state (race condition protection)
if order and current_status == ORDER_STATUS['COMPLETED']:
    log("ERROR", f"Order {order_id} already completed, ignoring failure event (likely timeout race condition)")
    return {"success": True, "acknowledged": True, "order_id": order_id, "note": "Already completed"}
```

### Bonus Fix: Corrected Misleading Comment
Fixed the timeout comment that said "1 minute per cup" but was actually 6.67 minutes per cup (400 seconds).

## Impact

These changes ensure that:
1. ✅ Arm workers properly exit when orders complete, allowing `asyncio.wait_for()` to complete successfully
2. ✅ Timeout errors won't be sent if the order already completed successfully
3. ✅ OMS won't overwrite COMPLETED status with ERROR status, even if a late timeout event arrives
4. ✅ The system is more resilient to race conditions between completion and timeout mechanisms

## Testing Recommendations

1. **Happy Path**: Run an order end-to-end and verify it completes without timeout errors
2. **Timeout Path**: Simulate a long-running order (or reduce timeout) and verify timeout works correctly
3. **Race Condition**: Test with very short timeouts to ensure completion events take precedence over timeout events
4. **Stop/Resume**: Test the stop order functionality to ensure workers exit properly when stopped

## Files Modified

1. `services/scheduler/scheduler.py` - Arm worker exit logic and timeout handler
2. `services/oms/app.py` - Order status validation in failure handler

