# sleep_test_V6

## Goal
Archive the current event-driven TX diagnostic sketch exactly as tested on 2026-04-20, including the latest findings and open questions.

## Status
- Archived on 2026-04-20 as V6 snapshot.
- Source copied from active working sketch `Sleep design/sleep_test/sleep_test.ino` without behavior changes.

## Test Architecture
- `setup()` performs one-time diagnostic setup.
- `loop()` performs indefinite sleep only:
  - `api.system.sleep.all(0xFFFFFFFF)`
- TX completion handling is callback-driven (`sendCallback`).
- No polling/wait loop exists in `loop()`.

## Important Findings (2026-04-20)

1. `loop()` must do nothing except sleep for valid low-current testing.
- This remains a hard requirement from the event-driven refactor.
- A fully state-driven implementation is still under consideration, not finalized.

2. Latest full retest matrix clarifies what changes with button wake.
- With `TEST_BUTTON_WAKE_ENABLED=1`:
  - mode 0 = 400 uA
  - mode 1 = 400 uA
  - mode 2 = 25 uA
  - mode 3 = 25 uA
  - mode 4 = 400 uA
  - mode 5 = 400 uA
  - mode 6 = 400 uA
  - mode 7 = 400 uA
- With `TEST_BUTTON_WAKE_ENABLED=0`:
  - mode 0 = 400 uA
  - mode 1 = 400 uA
  - mode 2 = 400 uA
  - mode 3 = 400 uA
  - mode 4 = 400 uA
  - mode 5 = 400 uA
  - mode 6 = 400 uA
  - mode 7 = 400 uA

3. The corrected interpretation is that modes 2 and 3 are the ones that break when button wake is disabled.
- It is not specifically mode 0/1 that regressed under button wake toggle.
- `TEST_BUTTON_WAKE_ENABLED=0` removes the previous mode 2/3 low-current recovery and forces those modes to ~400 uA.
- This strongly suggests wake/sleep configuration is interacting with TX completion path.

4. Modes 4/5/6/7 can still confound interpretation.
- These modes include settle delays and extra post-TX operations.
- Current evidence suggests those paths may be introducing additional residency/activity effects.
- The settle-time strategy may need replacement with explicit state/timer transitions.

## Mode Notes (Current Interpretation)

| Mode | Intent | Current interpretation |
|---|---|---|
| 0 | Baseline, no LoRa init | 400 uA in both button wake ON and OFF retests |
| 1 | LoRa config only, no TX | 400 uA in both button wake ON and OFF retests |
| 2 | TX then immediate sleep | 25 uA with button wake ON, 400 uA with button wake OFF |
| 3 | TX + callback handling then sleep | 25 uA with button wake ON, 400 uA with button wake OFF |
| 4-7 | TX + settle/post-TX actions | 400 uA in both button wake ON and OFF retests |

## Build
From repository root:

```sh
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build/sleep_test_V6 "./Sleep design/sleep_test_V6"
```

## Next Investigation Direction
- Focus first on why button wake OFF removes the mode 2/3 low-current recovery seen when button wake is ON.
- Treat modes 0/1 and 4-7 as currently fixed at ~400 uA in this snapshot.
- Replace settle-delay behavior with explicit non-blocking state/timer-driven transitions for modes 4+.
