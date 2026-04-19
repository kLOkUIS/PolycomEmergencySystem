# sleep_test_V4

## Goal
Archive the radio-sleep diagnostic matrix that isolates which RUI3/Radio calls preserve deep sleep current and which calls force the board into the ~400 uA state.

## Status
- Archived on 2026-04-19 as V4 snapshot.
- This version is the first sleep test that cleanly separates `Radio.Sleep()` behavior from `api.lora.precv(0)` behavior.
- The diagnostic action runs once in `setup()` and the device then returns to indefinite `sleep.all(0xFFFFFFFF)`.

## Test Architecture
- LPM enabled with `api.system.lpm.set(1)`.
- Button wake remains configured, but the radio diagnostic path is no longer triggered by button activity.
- The selected radio mode runs once during boot, then the board sleeps indefinitely.
- This removes wake/release and debounce artifacts from the current measurement.

## Diagnostic Modes

| Mode | Behavior | Measured Sleep Current (uA) | Conclusion |
|---|---|---:|---|
| 0 | No LoRa init | 25 | Clean baseline |
| 1 | Configure LoRa only | 25 | LoRa init/config alone does not break deep sleep |
| 2 | Configure LoRa + `Radio.Sleep()` only | 400 | `Radio.Sleep()` path on this stack does not return system to low-current sleep |
| 3 | Configure LoRa + `api.lora.precv(0)` only | 25 | `precv(0)` is not the source of the 400 uA state in this isolated boot-time test |
| 4 | Configure LoRa + `api.lora.precv(0)` + `Radio.Sleep()` | 400 | Adding `Radio.Sleep()` after `precv(0)` still lands in high-current state |
| 5 | Configure LoRa + repeated `api.lora.precv(0)` + `Radio.Sleep()` | 400 | Extra stop/sleep attempts do not recover deep sleep |

## Key Findings

1. `api.lora` configuration is not the problem.
- Mode 0 and mode 1 both measured ~25 uA.
- Entering P2P mode and setting frequency/modem parameters alone does not raise steady-state sleep current.

2. `Radio.Sleep()` is the transition that correlates with the ~400 uA state.
- Mode 2 rises to ~400 uA with no `precv(0)` involved.
- Modes 4 and 5 remain at ~400 uA even with additional receive-stop attempts.

3. `api.lora.precv(0)` is not sufficient evidence of a problem by itself in the current one-shot harness.
- In this archived setup-time test, mode 3 remains at ~25 uA.
- That means the earlier suspicion about `precv(0)` being the root cause is not supported by the cleaner V4 matrix.

4. The remaining open issue is how to cleanly return from an actual send/receive operation to the 25 uA sleep floor.
- The next experiment should transmit a real packet, wait for send completion, then compare sleep current with and without `Radio.Sleep()` or alternate stack shutdown patterns.

## Why V4 Matters
- Earlier button-triggered radio diagnostics were vulnerable to wake-path artifacts.
- V4 moves the experiment to a one-shot boot-time setup so the measurement reflects steady-state sleep after the radio action, not repeated wake handling.
- This makes the results suitable for deciding the next TX integration experiment.

## Compile
From repository root:

```sh
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build/sleep_test_V4 "./Sleep design/sleep_test_V4"
```

## Suggested Next Step
Create the next working test in `Sleep design/sleep_test/` that:
- initializes P2P,
- sends one packet,
- waits for send completion,
- optionally opens a bounded receive window,
- then returns to sleep using alternative shutdown sequences.

That will answer the practical TX question: what exact radio lifecycle leaves the board in low-current idle after a real transmission attempt.