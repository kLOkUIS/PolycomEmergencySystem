# TX Integration Plan: Sleep Baseline → Production Firmware

## Baseline Validation ✓
- **Achieved**: 25.25 µA (button wake only, LPM enabled, no busy-polling)
- **Source**: [Sleep design/sleep_button_only/](Sleep%20design/sleep_button_only/)
- **Status**: Ready for integration

## TX Architecture Analysis

### Current TX State Machine (from [firmwares/tx/tx.ino](firmwares/tx/tx.ino))

TX uses a **loop-based state machine** with polling:
```cpp
void loop() {
  switch (gState) {
    case STATE_IDLE:
      // Currently: delay(10) or busy polling
      break;
    case STATE_TRANSMIT:
      // Do work
      break;
    case STATE_WAIT_FEEDBACK:
      // Wait for callback
      break;
  }
}
```

### Integration Points

| Point | Current Behavior | To Change | Risk |
|-------|------------------|-----------|------|
| **STATE_IDLE** | `delay(10)` + poll | `api.system.sleep.all(ms)` with timer wake | **Medium** — Need timer callback for transitions |
| **Button wake** | Likely polling in loop | Wire to `registerWakeupCallback()` | **Low** — RUI3 API is straightforward |
| **TX send flow** | Synchronous blocking | Keep as-is (no idle sleep during TX) | **Low** — No change needed here |
| **Feedback timeout** | `delay()` in loop | `api.system.timer.create()` alternative | **Medium** — Timing must remain accurate |

## Phase 1: Safety Baseline (v1.0)

### Goal
Integrate sleep into TX **without** changing feedback/timing logic. Use timer wake to maintain TX schedule.

### Steps

1. **Replicate baseline structure**: Copy button + timer wake callbacks from sleep test
2. **Replace IDLE loop with sleep**: 
   ```cpp
   // OLD
   while (getState() == STATE_IDLE) {
     delay(10);
   }
   
   // NEW  
   if (getState() == STATE_IDLE) {
     api.system.sleep.all(10);  // Wake on button OR timer
   }
   ```
3. **Add timer wake for feedback timeout**:
   - Create persistent `api.system.timer` for feedback deadline
   - Calculate sleep duration as: `feedback_timeout_ms - elapsed_ms`
   - Replace `delay(feedback_timeout_ms)` with `sleep.all(remaining_ms)`

4. **Verify parity**:
   - TX should still transmit at the same schedule
   - Feedback timeout should still trigger
   - Button interrupt should still work
   - Power consumption in IDLE should drop to ~25 µA

### Code diff (rough outline)

```cpp
// In setup():
api.system.lpm.set(1);
api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
(void)api.system.sleep.registerWakeupCallback(wakeupCallback);
api.system.timer.create();  // For feedback timeout

// In loop():
if (gState == STATE_IDLE) {
  uint32_t sleep_dur = calculateIdleDuration();
  api.system.sleep.all(sleep_dur);
}

// Add callback
void wakeupCallback() {
  // Wake from sleep, check state transitions
}
```

## Phase 2: Feedback Integration (v1.1)

### Goal
Replace `delay()`-based feedback timeout with timer callback chain.

### Changes
- Swap `delay(feedback_timeout_ms)` → `api.system.timer.start(feedback_timeout_ms, timerCallback)`
- In `timerCallback`: Post feedback timeout event instead of returning from delay
- Keep sleep in STATE_WAIT_FEEDBACK if no timeout

### Risk
Medium — timing acuracyand callback sequencing must remain deterministic.

## Phase 3: Full Event-Driven Refactor (v2.0)

### Goal (Future)
Convert from loop-based polling to fully event-driven (no active loop).

### Scope
Deferred — focus on Phase 1 first.

---

## Testing Checklist (Phase 1)

- [ ] TX compiles with sleep baseline functions integrated
- [ ] Idle power drops from current ~mA to <50 µA baseline + TX overhead
- [ ] Button wakes TX from sleep correctly
- [ ] TX transmits at expected interval (timing not degraded)
- [ ] Feedback timeout still triggers correctly
- [ ] Multi-send cycle works (e.g., 3 sends over 60s)

## File References

- **Baseline**: [Sleep design/sleep_button_only/sleep_button_only.ino](Sleep%20design/sleep_button_only/sleep_button_only.ino)
- **TX Target**: [firmwares/tx/tx.ino](firmwares/tx/tx.ino)
- **Feedback Logic**: [firmwares/tx/Feedback.h](firmwares/tx/Feedback.h)
- **Tuning**: [firmwares/tx/FeedbackTuning.h](firmwares/tx/FeedbackTuning.h)

## Next Actions

1. ✓ Baseline validated (25.25 µA)
2. → Analyze TX state machine in detail
3. → Create TX integration candidate (v1.0 with sleep in IDLE only)
4. → Test and measure power savings
