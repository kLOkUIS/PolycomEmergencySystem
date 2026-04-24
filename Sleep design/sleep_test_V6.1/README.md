# sleep_test_V6.1

## Snapshot Date
2026-04-24 — Button pending flag fragility discovery.

## Key Finding: Button Pending Flag Trade-Off

### Discovery
The `gButtonWakePending` flag state has a direct impact on per-mode sleep behavior:

**Clearing flag before `startTxDiagStep()`:**
- Mode 0 (baseline button wake): ✅ Works (deep sleep)
- Mode 1 (LoRa init only): ✅ Works (deep sleep)
- Mode 2 (TX immediate sleep): ❌ Breaks (stays ~400 uA)
- Mode 3 (TX + callback): ❌ Breaks (stays ~400 uA)

**Not clearing flag (current V6.1 state):**
- Mode 0: ❌ Likely broken
- Mode 1: ❌ Likely broken
- Mode 2: ✅ Works (~25 uA deep sleep)
- Mode 3: ✅ Works (~25 uA deep sleep)

### Root Cause Hypothesis
Any extra CPU/interrupt bookkeeping after `psend()` but before the scheduler re-enters deep sleep disrupts the radio's post-TX shutdown sequence. The button-pending clear operation (GPIO read + `noInterrupts()/interrupts()`) happens to be significant enough to trigger this timing sensitivity.

### Architecture Notes

**Always-on wake path (current V6.1):**
- `api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN)` always runs.
- `registerWakeupCallback(buttonWakeupCallback)` always runs (required for RUI3 to return to deep sleep after TX-done).
- `TEST_BUTTON_WAKE_ENABLED` flag now only controls whether callback sets `gButtonWakePending`.

**Critical insight:**
Registering a wake callback even when you don't use button wakeup is **necessary** for TX mode sleep stability. RUI3 uses this to decide it's safe to return to deep sleep after events.

## Open Questions

1. Can modes 0/1 work with TX modes if we defer button-pending clear to a later time (not setup)?
2. Would a timer-based periodic check on BUTTON_PIN release be less disruptive?
3. Is there a way to signal the scheduler "ready for sleep now" explicitly after `psend()`?

## Next Steps

- Explore whether button-release detection should run from loop after enough idle time rather than setup.
- Consider state machine where TX modes skip button bookkeeping entirely but non-TX modes process it separately.
- Measure if a post-TX settle time (very short, <<1ms) allows TX modes to stabilize before return-to-sleep logic runs.

## Build Info
From repository root:
```sh
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build/sleep_test_V6.1 "./Sleep design/sleep_test_V6.1"
```

## Status
Archived snapshot with clear trade-off documented. Modes 2/3 working; requires separate solution for modes 0/1.
