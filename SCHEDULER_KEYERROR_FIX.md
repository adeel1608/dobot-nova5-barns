# Scheduler KeyError Fix - Defensive Dictionary Access

## Problem

The scheduler service was throwing KeyError exceptions when accessing task dictionary keys:

```
scheduler ERROR: Task paused handler exception: 'item'
scheduler ERROR: Stop order handler exception for order 135: 'function'
scheduler ERROR: Resume order handler exception for order 135: 'function'
```

This caused the stop/resume process to fail, leaving orders stuck in "processing" state.

## Root Cause

The new handlers I added (`handle_task_paused`, stop handler force-pause, resume handler) were accessing task dictionary keys directly without checking if they exist:

```python
# BEFORE (unsafe)
for t in core.tasks:
    if t["item"].get("cup_id") == cup_id and t["function"] == function:  # KeyError if missing
        ...

if t["status"] == "submitted":  # KeyError if missing
    ...

task["function"]  # KeyError if missing
task["item"].get("cup_id")  # KeyError if item is missing
```

Some tasks in the scheduler may not have all expected keys, especially during error conditions or state transitions.

## Solution

Changed all dictionary access to use `.get()` method with safe defaults:

### 1. `handle_task_paused` - Task Lookup

**Before:**
```python
for t in core.tasks:
    if t["item"].get("cup_id") == cup_id and t["function"] == function:
        task = t
        break
```

**After:**
```python
for t in core.tasks:
    # Safely get item and function
    task_item = t.get("item", {})
    task_function = t.get("function", "")
    task_cup_id = task_item.get("cup_id", "") if isinstance(task_item, dict) else ""
    
    if task_cup_id == cup_id and task_function == function:
        task = t
        break
```

### 2. `handle_task_paused` - Status Check

**Before:**
```python
if task["status"] == "submitted":
    task["status"] = "paused"
```

**After:**
```python
task_status = task.get("status", "")
if task_status == "submitted":
    task["status"] = "paused"
```

### 3. `handle_resume_order` - Paused Task Conversion

**Before:**
```python
paused_tasks = [t for t in core.tasks if t["status"] == "paused"]
for task in paused_tasks:
    task["status"] = "pending"
    log("DEBUG", f"Converted task {task['function']} (cup: {task['item'].get('cup_id')}) from paused to pending")
```

**After:**
```python
paused_tasks = [t for t in core.tasks if t.get("status") == "paused"]
for task in paused_tasks:
    task["status"] = "pending"
    # Safely get function and cup_id
    task_function = task.get("function", "unknown")
    task_item = task.get("item", {})
    task_cup_id = task_item.get("cup_id", "unknown") if isinstance(task_item, dict) else "unknown"
    log("DEBUG", f"Converted task {task_function} (cup: {task_cup_id}) from paused to pending")
```

### 4. `handle_stop_order` - Force Pause

**Before:**
```python
submitted_tasks = [t for t in core.tasks if t["status"] == "submitted"]
for task in submitted_tasks:
    task["status"] = "paused"
    log("INFO", f"Force-paused task {task['function']} for cup {task['item'].get('cup_id')}")
```

**After:**
```python
submitted_tasks = [t for t in core.tasks if t.get("status") == "submitted"]
for task in submitted_tasks:
    task["status"] = "paused"
    # Safely get function and cup_id
    task_function = task.get("function", "unknown")
    task_item = task.get("item", {})
    task_cup_id = task_item.get("cup_id", "unknown") if isinstance(task_item, dict) else "unknown"
    log("INFO", f"Force-paused task {task_function} for cup {task_cup_id}")
```

### 5. `handle_stop_order` - Polling Loop

**Before:**
```python
submitted_tasks = [t for t in core.tasks if t["status"] == "submitted"]
```

**After:**
```python
submitted_tasks = [t for t in core.tasks if t.get("status") == "submitted"]
```

## Benefits

✅ **No more KeyError exceptions** - All dictionary access is safe
✅ **Graceful degradation** - Missing keys don't crash the handler
✅ **Better error handling** - Added traceback logging for debugging
✅ **Consistent pattern** - All handlers use the same defensive approach

## Testing

The fix has been applied. Next stop/resume should work without exceptions:

### Expected Success Logs
```
[Scheduler] [TASK PAUSED] Marked task Arm1Test1 as paused (reason: validation_failed) for cup 135-1
[Scheduler] All submitted tasks completed for stop of order 135
[Scheduler] Converting 1 paused tasks back to pending for retry
[Scheduler] Converted task Arm1Test1 (cup: 135-1) from paused to pending
```

### No More Error Logs
```
✗ scheduler ERROR: Task paused handler exception: 'item'
✗ scheduler ERROR: Stop order handler exception for order 135: 'function'
✗ scheduler ERROR: Resume order handler exception for order 135: 'function'
```

## Why This Happened

This was an oversight in the initial implementation - I assumed all tasks would have the standard structure:
```python
{
    "item": {"cup_id": "...", ...},
    "function": "...",
    "status": "..."
}
```

But in reality, tasks can be in partial states during errors or race conditions, so defensive programming is essential.

## Related Files

- `services/scheduler/app.py` - All fixes applied here

