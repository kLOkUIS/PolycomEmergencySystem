# sleep_test_V1

## Goal
Capture the validated minimal baseline sleep behavior.

## Build Changelog
- 2026-04-17: Archived as V1 baseline snapshot.
- 2026-04-17: Button wake only, no timer wake logic.
- 2026-04-17: `api.system.lpm.set(1)` enabled.
- 2026-04-17: Root cause documented: removing button/knob polling loop dropped current from ~400 uA to ~25.25 uA.

## Compile
From repository root:

```sh
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build/sleep_test_V1 "./Sleep design/sleep_test_V1"
```

Optional flags:

```sh
# Disable LPM (comparison only)
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board \
  --build-property compiler.cpp.extra_flags="-DTEST_MODE_LPM=0" \
  --output-dir ./build/sleep_test_V1_no_lpm \
  "./Sleep design/sleep_test_V1"
```

## Measurement Data

| Date | Build | Timer Enabled | Timer Period (ms) | Avg Current (uA) | Notes |
|---|---|---:|---:|---:|---|
| 2026-04-17 | Baseline V1 | 0 | N/A | 25.25 | Validated minimal baseline |

## Root Cause History

- High idle current (~400 uA) was caused by repeated polling while a button was held:
  - `while (buttonPressedRaw()) { api.system.sleep.all(25); }`
- That pattern wakes every 25 ms and prevents deep sleep residency.
- Baseline V1 removes this loop and uses interrupt wake only.

## Debug Notes
- Always disconnect debugger while measuring baseline current.
- Use `sleep_test` as the active working area for new experiments before snapshotting to next version.
