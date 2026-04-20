# Sleep design

This file is only an index for the `Sleep design` folder.
Per-version details and measurements live in each version folder README.

## Folder layout

- `sleep_test/`
  - `sleep_test.ino`
  - Active working sketch (edit and test here before snapshotting a version).
- `sleep_test_V1/`
  - `sleep_test_V1.ino`
  - `README.md` (build changelog + measurement data).
- `sleep_test_V6/`
  - `sleep_test_V6.ino`
  - `README.md` (2026-04-20 snapshot with event-driven loop-only sleep findings and open issues).

## Current status

- `sleep_test_V1` stores the validated baseline (~25.25 uA).
- `sleep_test_V6` stores the latest checkpoint where loop behavior is constrained to sleep-only.
- `sleep_test` remains the active workspace for next experiments.

## Key finding (important)

- The ~400 uA issue came from a polling/hold loop that repeatedly woke the MCU:
  - `while (buttonPressedRaw()) { api.system.sleep.all(25); }`
- Removing that loop and relying on interrupt wake restored deep sleep (~25.25 uA).
- New finding (2026-04-20): disabling button wake can break expected low-current behavior in mode 0/1, so wake/sleep configuration path is still under investigation.

## Build commands

From repository root:

```sh
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build/sleep_test "./Sleep design/sleep_test"
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build/sleep_test_V1 "./Sleep design/sleep_test_V1"
```

## Measurement protocol

- Disconnect J-Link/debugger for power measurements.
- Power-cycle before each run.
- Keep meter range and wiring unchanged between runs.
- Record values in each version's local `README.md`.
