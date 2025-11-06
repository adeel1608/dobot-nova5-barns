# Camera Stream Redundancy Fix - Testing Checklist

## Quick Test (5 minutes)

### ✅ Test 1: Basic Page Load
1. Navigate to cameras page
2. **Wait**: Watch console for 1 second
3. **Expected**: See "Streams enabled after mount stabilization"
4. **Expected**: Video streams appear
5. **Pass if**: Only ONE connection per camera in logs

### ✅ Test 2: Page Refresh
1. On cameras page with active streams
2. Press **F5** to refresh
3. **Expected**: Streams restart after ~1 second
4. **Expected**: Clean disconnection messages in console
5. **Pass if**: No "Duplicate session detected" warnings

### ✅ Test 3: Rapid Tab Switching
1. Open cameras page
2. Quickly switch to another tab and back (< 1 second)
3. Repeat 3 times
4. **Expected**: Streams stay active
5. **Pass if**: No connection spam (check backend logs)

## Full Test Suite (15 minutes)

### ✅ Test 4: Tab Hidden (Long Duration)
1. Open cameras page
2. Switch to another tab
3. **Wait**: 6 seconds (past 5-second threshold)
4. Switch back to cameras tab
5. **Expected**: "Tab hidden - will stop streams if hidden for 5s"
6. **Expected**: Streams restart cleanly
7. **Pass if**: No errors, streams resume

### ✅ Test 5: Fullscreen Mode
1. Click fullscreen on any camera
2. **Expected**: Native fullscreen mode activates
3. Press **ESC** to exit
4. Re-enter fullscreen
5. **Expected**: No duplicate connections
6. **Pass if**: Smooth transition, no errors

### ✅ Test 6: Navigation Away and Back
1. Go to cameras page
2. Navigate to different page (e.g., Dashboard)
3. Navigate back to cameras page
4. **Expected**: Streams initialize after 1 second
5. **Pass if**: Clean startup, no orphaned connections

### ✅ Test 7: Hot Reload (Dev Mode Only)
1. Open cameras page in development
2. Make a trivial code change (add a space)
3. Save file to trigger hot reload
4. **Expected**: "Hot reload detected" in logs
5. **Expected**: Streams restart after stabilization
6. **Pass if**: No connection spam

### ✅ Test 8: Multiple Cameras
1. If you have multiple cameras configured
2. Open cameras page
3. **Expected**: One connection per camera
4. Switch to "All Cameras" view
5. **Pass if**: Correct number of active sessions

### ✅ Test 9: Error Recovery
1. Disconnect network/camera temporarily
2. **Expected**: "Stream Unavailable" message
3. Reconnect network/camera
4. **Wait**: 60 seconds for health check
5. **Expected**: Stream auto-recovers
6. **Pass if**: Stream resumes without manual refresh

### ✅ Test 10: Long-Running Stability
1. Open cameras page
2. **Wait**: 5 minutes
3. Check backend logs
4. **Expected**: Periodic "frames delivered" logs every 10s
5. **Expected**: Steady frame counts (~150 frames/10s)
6. **Pass if**: No disconnections, stable performance

## What to Look For

### ✅ Good Signs (Healthy System)
- Console shows "Streams enabled after mount stabilization"
- Only ONE connection per camera at any time
- "Stream naturally ended" on disconnections (no cooldown)
- Periodic frame count logs every 10 seconds
- Clean "Client disconnected" messages
- Frame counts steadily increasing

### ❌ Red Flags (Issues)
- "Duplicate session detected" warnings
- Multiple connections for same camera simultaneously
- Frequent "Short-lived connection detected" messages
- Streams not starting after page load
- "Cooldown active" messages during normal navigation
- Connection age < 0.5 seconds frequently

## Console Commands for Debugging

### Check Active Streams (Backend)
```bash
curl http://localhost:8001/stream/active
```

### Check Cooldown Status
```bash
curl http://localhost:8001/stream/cooldown/all
```

### View Camera List
```bash
curl http://localhost:8001/cameras
```

## Key Timing Parameters

| Event | Delay | Purpose |
|-------|-------|---------|
| Mount to Stream Enable | 1.0s | Component stabilization |
| Reconnection Debounce | 0.5s | Prevent connection spam |
| Tab Hidden Threshold | 5.0s | Avoid brief focus changes |
| Stream Health Check | 30s | Detect stalled streams |
| Stalled Stream Timeout | 60s | Force reconnection |

## Backend Log Patterns

### ✅ Normal Operation
```
[INFO] Stream started for ceiling, session: abc-123, active sessions: 1
[INFO] Stream generator started for ceiling, session: abc-123
[INFO] Stream ceiling session abc-123: 150 frames delivered
[INFO] Stream ceiling session abc-123: 300 frames delivered
```

### ✅ Clean Shutdown
```
[INFO] Client disconnected from ceiling (frames delivered: 450)
[INFO] Stream naturally ended for ceiling - no cooldown applied
[INFO] Stream generator ended for ceiling, total frames: 450
```

### ⚠️ Hot Reload (Expected)
```
[INFO] Short-lived connection detected (0.23s) - likely hot reload
[INFO] Hot reload detected
```

### ❌ Problem (Should NOT See)
```
[WARNING] Duplicate session detected for ceiling, session: abc-123
[WARNING] Cannot start stream - cooldown active for 1.5s
```

## Performance Metrics

### Expected Values
- **Connection Lifetime**: > 10 seconds (normal navigation)
- **Frame Rate**: ~15 FPS (150 frames per 10 seconds)
- **Active Sessions**: Equal to number of visible cameras
- **Short-lived Connections**: < 5% of total

### Troubleshooting

**Streams don't start after page load**
- Wait full 1 second for stabilization
- Check console for error messages
- Verify backend is running (port 8001)

**"Cooldown active" during normal use**
- This should ONLY happen after manual "stop-all" calls
- Not during page refresh or navigation
- If you see this, something is calling stop-all incorrectly

**Multiple connections for same camera**
- Check React DevTools for duplicate components
- Look for "Duplicate session detected" in backend
- This is now prevented by the fix

## Success Criteria

### All Tests Pass When:
1. ✅ Streams start reliably after page load
2. ✅ Page refresh doesn't create duplicate connections
3. ✅ Rapid navigation doesn't cause connection spam
4. ✅ Tab switching works smoothly
5. ✅ Fullscreen mode has no issues
6. ✅ Streams run stably for 5+ minutes
7. ✅ Backend logs show clean connection lifecycle
8. ✅ No "duplicate session" or unwanted "cooldown" warnings

## If Tests Fail

1. **Clear browser cache** and hard refresh (Ctrl+F5)
2. **Restart backend service** to clear any stuck state
3. **Check browser console** for JavaScript errors
4. **Check backend logs** for detailed error messages
5. **Verify network connectivity** to camera/backend
6. **Review this checklist** for expected vs actual behavior

## Report Issues

When reporting issues, include:
- ❌ Which test(s) failed
- 📋 Browser console output (with errors)
- 📋 Backend logs (last 50 lines)
- 🔄 Steps to reproduce
- 💻 Browser and OS version
- ⏱️ Timestamps of when issue occurred

---

**Quick Reference**: 
- Frontend: `services/barns-dashboard/src/pages/cameras/components/UnifiedCameraPanel.jsx`
- Backend: `services/video-stream/app.py`
- Detailed Docs: `services/video-stream/STREAM_REDUNDANCY_FIXES.md`

