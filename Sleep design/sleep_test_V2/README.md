# Sleep Test V2 — Timer Wake via Sleep Timeout

## Measurement Result

- **Sleep current**: 24.14 µA
- **Wake behavior**: Brief wake every 10 seconds (timer), plus on-demand wake via button press
- **LPM enabled**: Yes (`api.system.lpm.set(1)`)

## Key Finding: `api.system.timer` Prevents Deep Sleep

`api.system.timer.create(RAK_TIMER_0, cb, RAK_TIMER_PERIODIC)` + `api.system.timer.start(...)` was causing ~405 µA idle current. The RUI3 system timer subsystem keeps the chip active and prevents deep sleep during `sleep.all()`.

**Fix**: Use `sleep.all(periodMs)` with a finite timeout instead of a system timer object. On wake, check whether the button callback fired; if not, treat the wake as a timer timeout.

## Flags

| Flag | Default | Effect |
|------|---------|--------|
| `TEST_MODE_LPM` | 1 | Enables `api.system.lpm.set(1)` |
| `TEST_TIMER_ENABLED` | 1 | Enables periodic wake every `TEST_TIMER_PERIOD_MS` ms |
| `TEST_TIMER_PERIOD_MS` | 10000 | Timer period in milliseconds |

## Wake Logic

```
sleep.all(10000ms)
  → button press  → gButtonWakeCount++
  → timeout       → gTimerWakeCount++
```

## Lineage

- **V1** (25.25 µA) — Button wake only, no timer, established clean baseline
- **V2** (24.14 µA) — Added periodic timer wake via sleep timeout; confirmed system timer API must be avoided
