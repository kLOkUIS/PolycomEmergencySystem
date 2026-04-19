# sleep_test_V3

## Goal
Capture a stable timer + sleep architecture that preserves deep sleep current while supporting periodic wake and button-triggered feedback.

## Status
- Archived on 2026-04-18 as V3 snapshot.
- This chapter is primarily about timer behavior and power residency.
- Recent feedback-pattern tweaks (debounce and long-press behavior) are secondary and not the main V3 conclusion.

## Key Findings (Timer and Sleep)

1. `api.system.timer` periodic timer path is not suitable for deep-sleep baseline on this stack.
- Pattern tested: `api.system.timer.create(..., RAK_TIMER_PERIODIC)` + `api.system.timer.start(...)`
- Observed result: ~405-406 uA during expected sleep
- Conclusion: Avoid RUI periodic system timer for the low-power idle path.

2. `api.system.sleep.all(timeoutMs)` is the safe periodic wake mechanism.
- Pattern tested: `api.system.sleep.all(TEST_TIMER_PERIOD_MS)`
- Observed result: ~24 uA sleep baseline with brief periodic wake pulses
- Conclusion: Use sleep timeout for periodic wake in low-power mode.

3. `millis()` can be used in active-state logic, but avoid busy-spin loop designs.
- A non-blocking phase loop experiment that repeatedly returned to `loop()` without sleeping caused high average current (around ~560 uA) because CPU stayed active/spinning.
- This was a scheduler structure issue, not proof that timestamp reads themselves are incompatible.

4. Active-state timing can be blocking without harming the power budget, if sleep is re-entered cleanly.
- Device is awake only briefly.
- Current implementation uses short active bursts, then returns to `sleep.all(...)`.

## Current V3 Behavior
- Wake sources:
  - Button falling edge interrupt
  - Sleep timeout (`TEST_TIMER_PERIOD_MS`)
- On button wake:
  - Run feedback pattern (LED blinks + motor pulse)
  - Return to idle/sleep path
- On timeout wake:
  - Increment timer wake counter
  - Return to sleep path

## Compile
From repository root:

```sh
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build/sleep_test_V3 "./Sleep design/sleep_test_V3"
```

## TX Firmware Direction
Use this V3 rule set when integrating into TX firmware:
- Keep periodic idle wake on `sleep.all(timeout)`.
- Avoid RUI periodic `api.system.timer` for deep-sleep cadence.
- Use active-state timing in short bursts and always return quickly to sleep.
- Treat non-blocking schedulers carefully: no idle spin loops between states.

## Design Note: Voltage Read Timing After Button Wake

With V3 timer findings, battery-voltage scheduling should be driven by `sleep.all(timeout)` only.

Recommended sequence after a button interrupt:
1. Wake on button and run all button-related work first (feedback/tasks).
2. Enter sleep for a fixed **2 minute settling window** to let battery voltage recover after heavy load.
3. Wake after 2 minutes and take battery reading.
4. Choose next sleep interval from that reading:
- If battery is healthy: schedule next voltage check in about **24 hours**.
- If battery is low / warning state: schedule next wake according to the battery-feedback policy (shorter interval).

Rationale:
- A reading taken immediately after motor/LED/radio activity can be biased low.
- Post-press settling gives a more representative battery value.
- This keeps the architecture simple and compatible with the proven low-power path (no RUI periodic timer backend).
