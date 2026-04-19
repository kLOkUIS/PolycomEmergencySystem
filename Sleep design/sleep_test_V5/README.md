# sleep_test_V5

## Goal
Archive the transmit-boundary sleep investigation after V4, focused on what happens once `api.lora.psend(...)` is executed.

## Status
- Archived on 2026-04-19 as V5 snapshot.
- This version keeps the one-shot setup-time architecture and adds TX lifecycle variants.

## Test Architecture
- LPM enabled with `api.system.lpm.set(1)`.
- Diagnostic runs once in `setup()`.
- If mode includes TX, one packet (`"sleep-tx"`) is sent with `api.lora.psend(...)`.
- Optional post-TX steps are applied based on mode.
- Device then enters indefinite sleep with `api.system.sleep.all(0xFFFFFFFF)`.

## Mode Matrix and Measurements

| Mode | Sequence | Measured Sleep Current (uA) |
|---|---|---:|
| 0 | Baseline (no LoRa init) | 25 |
| 1 | Configure LoRa only, no send | 25 |
| 2 | `psend()` then sleep immediately | 400 |
| 3 | `psend()` then wait TX callback then sleep | 400 |
| 4 | Mode 3 + settle 100 ms | 400 |
| 5 | Mode 3 + settle 5000 ms | 400 (after ~5 s around ~4 mA) |
| 6 | Mode 5 + `precv(0)` | not recorded in this snapshot |
| 7 | Mode 5 + `precv(0)` + delay + `Radio.Sleep()` | 400 |

## Findings

1. LoRa P2P configuration alone still returns to low-current sleep.
- Modes 0 and 1 remain at ~25 uA.

2. The transition to ~400 uA starts as soon as one real TX occurs.
- Mode 2 already rises to ~400 uA.
- Waiting for callback or adding settle delays does not recover low-current sleep.

3. Additional post-TX radio calls did not recover low-current sleep.
- Mode 7 (`precv(0)` then `Radio.Sleep()`) still measures ~400 uA.

4. Current evidence points to a post-`psend()` stack/radio residency state.
- The issue appears linked to executing TX itself, not LoRa init alone.

## Compile
From repository root:

```sh
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build/sleep_test_V5 "./Sleep design/sleep_test_V5"
```

## Next Step Direction
Investigate recovery paths after TX that do more than local radio method calls, for example:
- controlled stack reset/re-init sequence,
- explicit reboot-to-idle measurement path,
- comparison with TX firmware operational flow and bounded receive windows.