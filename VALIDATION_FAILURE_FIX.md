# Validation Failure Stop/Resume Fix with Sub-Step Resumption

## Problem
When a validation failure occurred during order processing, there were two issues:

### Issue 1: Duplicate Execution
1. Task "Pour Espresso" would fail validation at some sub-step
2. Order would be stopped
3. When resumed, the ENTIRE task would execute again from the beginning
4. This caused duplicate work and "Duplicate feedback" warnings

### Issue 2: No Sub-Step Resumption
Routine tasks consist of multiple sub-steps (e.g., cup_detection → pour_espresso_pitcher_cup_station → return_espresso_pitcher). When validation failed partway through:
- All completed sub-steps would be re-executed on resume
- No mechanism existed to resume from the exact failed sub-step
- This wasted time and resources repeating already-completed work

## Example Scenario
```
Task: "Pour Espresso" has 5 sub-steps
1. ✅ cup_detection (completed)
2. ✅ grind_coffee (completed)
3. ❌ check_portafilter (validation failed) → ORDER STOPS
4. ⏸️ pour_espresso (not reached)
5. ⏸️ return_pitcher (not reached)

OLD BEHAVIOR on resume:
- Restarts from step 1 (duplicates work for steps 1-2)

NEW BEHAVIOR on resume:
- Resumes from step 3 (skips completed steps 1-2)
```

## Root Cause
1. Routine had no mechanism to track partial task progress
2. When a task was resubmitted, it always started from sub-step 0
3. The `revert_previous_step_for_cup` function was designed to revert to the PREVIOUS scheduler task, not handle sub-step resumption

## Solution

### Changes in `services/scheduler/scheduler.py`

**Modified `revert_previous_step_for_cup` function (lines 1104-1122)**:
- Resets the CURRENT task to "pending" (not cancelled)
- Does NOT revert to previous scheduler task
- Relies on routine's sub-step progress tracking for precise resumption
- Simplified logic: just reset current task and let routine handle the rest

### Changes in `services/routine/executer.py`

**1. Added Partial Task Progress Tracking (lines 36-93)**:
```python
# New global state tracking
task_progress = {}  # Maps "{cup_id}-{function}" to last completed step index
task_progress_lock = asyncio.Lock()

# New helper functions
async def get_task_progress(cup_id, function) -> int
async def set_task_progress(cup_id, function, step_index)
async def clear_task_progress(cup_id, function)
async def clear_order_progress(order_id)
```

**2. Modified `process_task` to Support Resume (lines 532-544)**:
- Checks for existing progress when task starts
- If progress exists, skips already-completed sub-steps
- Logs which steps are being skipped

**3. Save Progress After Each Step (line 868)**:
- After each successful sub-step, saves progress
- Allows resumption from that exact point if stopped

**4. Progress Lifecycle Management (lines 880-898)**:
- Clears progress on successful completion (no longer needed)
- Clears progress on permanent failure (task won't retry)
- **Preserves progress** on validation failure (enables sub-step resume)
- **Preserves progress** on order stop (enables sub-step resume)

**5. Updated Comments and Logs**:
- Clarified that validation failures preserve progress
- Added resume checkpoint logging
- Made it clear tasks resume from saved sub-step, not restart

## Flow After Fix

### When Validation Fails:
1. Routine detects validation failure at sub-step N
2. Progress is saved showing steps 0 to N-1 are complete
3. Calls `revert_previous_step_and_stop`:
   - Scheduler marks **CURRENT** task as "pending" (for sub-step retry)
   - OMS stops the order
4. Routine finishes processing **without** sending feedback
5. Routine **preserves** progress (doesn't clear it)
6. Task key: `{cup_id}-{function}` maps to last completed step

### When Order Resumes:
1. Scheduler worker wakes up
2. Looks for pending tasks
3. Finds the **SAME** task that had validation failure
4. Submits task to routine
5. Routine checks `task_progress[{cup_id}-{function}]`
6. Finds progress exists (e.g., step 2 was last completed)
7. **Skips steps 0-2** (already done)
8. **Resumes from step 3** (the validation failure point)
9. Continues forward from there

### On Successful Completion:
- Task completes all sub-steps
- Sends success feedback to scheduler
- **Clears progress** for this task (no longer needed)

## Benefits
- ✅ Eliminates duplicate sub-step execution
- ✅ Precise sub-step resumption on validation failure
- ✅ Precise sub-step resumption on order stop
- ✅ No unnecessary rework (skips completed sub-steps)
- ✅ Maintains strict task order and dependencies
- ✅ Clear logging for debugging ("Resuming from step X")
- ✅ Prevents "Duplicate feedback" warnings
- ✅ Efficient resource usage (no wasted robotic arm movements)

## Key Design Decisions

### Why Track Progress in Routine (not Scheduler)?
- Scheduler works with high-level tasks ("Pour Espresso")
- Routine knows about sub-steps (cup_detection, pour, return)
- Sub-step granularity is a routine concern, not scheduler concern
- Keeps separation of concerns clean

### Why Preserve Progress on Validation Failure?
- User may fix the issue (e.g., add cups to stations) and resume
- No need to redo already-completed sub-steps
- Validation can be retried without repeating prep work

### Why Clear Progress on Success?
- No longer needed once task completes
- Prevents stale state accumulation
- Clean slate for next order

## Testing Recommendations
1. Test validation failure at first sub-step (resume from step 0)
2. Test validation failure at middle sub-step (verify skip of completed steps)
3. Test validation failure at last sub-step
4. Test manual order stop mid-task (should resume from exact sub-step)
5. Verify logs show "Resuming from step X" and "Skipping already completed step Y"
6. Verify no duplicate arm movements on resume
7. Test multiple validation failures in same task (progress updates correctly)
8. Test order completion clears all progress for that order

