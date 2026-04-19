# TX Feedback Best Practice (RAK4631 / nRF52840)

This document defines the stable feedback architecture for `firmwares/tx`.

## Goals

- Deterministic user feedback with no overlapping motor ownership.
- Long-press timing that matches human expectation.
- No motor state leakage from PWM warning into later pulses.
- Keep radio state machine independent from haptic pattern rendering.

## Core Design Rules

1. Single output owner
- Only `Feedback.h` writes `LED_PIN` and `MOTOR_PIN` outputs.
- `tx.ino` must never directly call `analogWrite`/`digitalWrite` on the motor pin.

2. Event-driven feedback
- `tx.ino` emits feedback events via `setState(...)`.
- `feedbackUpdate(...)` renders patterns incrementally in the main loop.

3. Non-blocking runtime patterns
- Runtime feedback must be step-based and advanced by time.
- Avoid `delay(...)` in normal feedback paths.

4. PWM separation
- Use PWM only for warning crescendo (`FEEDBACK_MODE_WARNING`).
- Use fixed pulse patterns for confirmation/success/failure cues.

5. nRF52 PWM safety
- Motor OFF should explicitly clear duty (`analogWrite(pin, 0)`).
- Motor ON should use full-duty PWM (`analogWrite(pin, 255)`) to avoid mixed pin-mode edge cases.

## Current UX Timeline

1. Button press
- Immediate tactile tap + LED flash (`TX_STATE_PRESS_FEEDBACK`).

2. Hold tracking begins
- Start hold timing after tap completes.

3. Warning stage
- Short silence, then PWM crescendo (`TX_STATE_WAIT_LONG_PRESS`).

4. Long-press confirmed
- 400 ms silence, then short confirmation pulse (`TX_STATE_CONFIRM_LONG_PRESS`).

5. Result semantics
- In-call confirmed: `O0o` pattern (`TX_STATE_COMPLETE`).
- ACK only (no call): double pulse (`TX_STATE_COMPLETE_ACK_ONLY`).
- Failure after retries: one strong pulse (`TX_STATE_FAILED`).

## State Machine Contract

- `setState(...)` is the only place that starts/stops feedback patterns.
- Completion states (`COMPLETE`, `COMPLETE_ACK_ONLY`, `FAILED`) return to sleep only after `!feedbackBusy(...)`.
- Release in `WAIT_LONG_PRESS` must stop warning immediately before branch decision.

## Tuning Knobs (safe)

Edit in `Feedback.h`:

Prefer editing `FeedbackTuning.h` instead of `Feedback.h`:

- `WARNING_POST_TAP_SILENT_MS`
- `WARNING_RAMP_MS`
- `WARNING_PRE_CONFIRM_SILENT_MS`
- `WARNING_MIN_PWM`
- `WARNING_MAX_PWM`
- `BUTTON_TAP_ON_MS`
- `LONG_PRESS_CONFIRM_SILENCE_MS`
- `LONG_PRESS_CONFIRM_PULSE_MS`
- `CALL_ESTABLISHED_*`
- `ACK_ONLY_*`
- `FAILURE_PULSE_MS`

Edit in `tx.ino`:

- Radio timeout constants (`ACK_TIMEOUT_MS`, `CALL_TIMEOUT_MS`, etc.)

## Do Not Regress

- Do not reintroduce direct motor writes in `tx.ino`.
- Do not mix blocking feedback functions with asynchronous radio states.
- Do not mix digital motor pulses with PWM warning paths outside `feedbackApplyOutputs(...)`.

## Validation Checklist

1. Repeated short press/release:
- No residual vibration carry-over.

2. Hold-release-hold sequences:
- Warning always restarts from baseline.

3. Long-press threshold crossing:
- Distinct silence then confirm pulse.

4. SOS outcomes:
- `O0o` for in-call confirmed.
- Double pulse for ACK-only path.
- Single strong pulse for failure path.
