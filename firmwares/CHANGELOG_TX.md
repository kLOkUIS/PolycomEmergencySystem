# TX Firmware Changelog

Hardware: RAK4631 (nRF52840), CR2 cell on VDD, knob PCB  
LoRa: 868 MHz, SF7, BW125, CR4/5, preamble 8, +14 dBm — unchanged across all versions

---

## Tx V1 — Initial working firmware

- Long-press only (3000 ms threshold) triggers SOS
- Short press does nothing — no other press mode
- Pins hard-coded in constants: `P1_03` LED, `P1_01` BTN, `P1_04` MOTOR
- Debug serial output via `DEBUG_SERIAL` compile flag
- `feedbackButtonDetected` / `feedbackLongPressProgress` / `feedbackLongPressConfirmed` / `feedbackAckReceived` / `feedbackCallEstablished` / `feedbackAckOnly` / `feedbackFailure` — all called as free functions taking pin arguments
- `feedbackIdle()` called on sleep entry to idle the LED and motor
- SOS state machine: SLEEP → WAIT_LONG_PRESS → SEND_SOS → WAIT_TX_DONE → WAIT_ACK → WAIT_CALL → COMPLETE / COMPLETE_ACK_ONLY / FAILED / RETRY_DELAY
- Up to 4 SOS retry attempts (`MAX_SOS_ATTEMPTS = 4`)
- Retry delay 1200 ms
- ACK wait 3000 ms, CALL wait 4000 ms
- Sleep with indefinite `sleep.all(0xFFFFFFFF)`; button wakeup re-arms press from inside `enterSleepState`
- No watchdog, no recovery system, no battery monitoring

---

## Tx V2 — Short press sends END_CALL

- Added `SHORT_PRESS_MS = 200` threshold to distinguish short from long release
- Short press releases in a valid window → sends `PACKET_END_CALL` → `SEND_END_CALL` / `WAIT_END_CALL_TX_DONE` states
- Too-short releases (< 200 ms) silently ignored
- All other behaviour identical to V1

---

## Tx V3 — Short press becomes CALL_TOGGLE; debug serial removed

- `PACKET_END_CALL` replaced by `PACKET_CALL_TOGGLE` (protocol rename)
- `SHORT_PRESS_MS` lowered to 100 ms
- Debug serial (`DEBUG_SERIAL`, `DPRINT`, `DPRINTLN`) removed entirely — cleaner production build
- `sendSosMessage()` replaced by generic `sendMessage(PacketType)` used for both SOS and CALL_TOGGLE
- `scheduleRetry` simplified; no debug prints
- All feedback calls remain as free functions with pin arguments

---

## Tx V4 — Attempt hard-timeout and RX window margin guard

- Added `ATTEMPT_HARD_TIMEOUT_MS = 14000` — per-SOS-attempt absolute deadline regardless of state
- Added `RX_WINDOW_TIMEOUT_MARGIN_MS = 600` — software fallback after the LoRa RX timeout fires, guarding against callback latency
- `gAttemptStartedAtMs` and `gRxWindowStartedAtMs` tracking variables added
- `startReceiveWindow` now records `gRxWindowStartedAtMs` after arming the radio
- WAIT_TX_DONE, WAIT_ACK, WAIT_CALL all check both the hard attempt timeout and the per-window margin guard
- WAIT_ACK can now also accept a `PACKET_CALL` directly (skips WAIT_CALL immediately to COMPLETE)
- `setLowPowerEnabled()` helper added; LPM disabled while active, re-enabled on sleep entry
- `elapsedAtLeast()` helper introduced (overflow-safe elapsed check)
- Loop now calls `setLowPowerEnabled(gState == TX_STATE_SLEEP)` at top of every iteration
- `loop()` delay reduced from 5 ms to 2 ms

---

## Tx V5 — Software watchdog and flash recovery system

- **Software watchdog added**: `watchdogArm()` / `watchdogDisarm()` / `watchdogTouch()` / `watchdogService()`
  - Stall timeout: 15 000 ms (time between state changes or explicit touches)
  - Absolute timeout: 60 000 ms (total active session ceiling)
  - On trip: calls `rebootForRecovery()` → writes flash marker → reboots
- **Recovery flash system added**: packed 32-bit word in user flash
  - `RECOVERY_FLAG_SOFT_RECOVERY` (bit 0): set on every `scheduleRetry()` (SOS miss)
  - `RECOVERY_FLAG_REBOOT_RECOVERY` (bit 1): set by `rebootForRecovery()` (watchdog trip)
  - Boot countdown: 5 boots stored in bits [15:8]; marker auto-clears after 5 power cycles
  - `maybeShowRecoveryBootStrobe()` on boot: 3× 45 ms blinks for reboot recovery; 120 ms pause + 180 ms pulse for soft recovery
- `setupmaybeShowRecoveryBootStrobe()` called early in `setup()` before peripherals
- `startReceiveWindow()` now explicitly stops any prior RX window before re-arming (`api.lora.precv(0)` then `precv(timeout)`)
- `watchdogTouch()` called on successful radio events (TX done, RX window arm)
- `watchdogService()` called at the top of every `loop()` iteration

---

## Tx V6 — FeedbackController object; press-feedback state; RESETREAS-based recovery clear

- **FeedbackController introduced**: `gFeedback` object, all feedback calls now through `feedbackInit` / `feedbackStop` / `feedbackUpdate` / `feedbackBusy` / `feedbackPlay*` API
- **New press-feedback states**:
  - `TX_STATE_PRESS_FEEDBACK`: plays a short tap on button-down, waits for tap to finish before entering WAIT_LONG_PRESS
  - `TX_STATE_CONFIRM_LONG_PRESS`: plays the long-press confirmation haptic before sending SOS, waits for it to finish
- `gButtonPressedAtMs` renamed to `gPressStartedAtMs`
- `setState()` helper introduced — sets `gState` + `gStateStartedAtMs` and drives feedback on transition
- COMPLETE / COMPLETE_ACK_ONLY / FAILED now wait for `feedbackBusy()` to clear before returning to SLEEP (non-blocking feedback)
- **Recovery clear logic changed from boot-count to `RESETREAS` register**:
  - `shouldClearRecoveryMarkerThisBoot()` reads NRF52 reset-reason register; clears flags only on clean (non-software) reset
  - 5-boot packed countdown removed; flash state simplified back to flags-only word
  - `readRecoveryFlags()` / `writeRecoveryFlags()` functions replace `readRecoveryState()` / `writeRecoveryState()`
- `LONG_PRESS_MS` moved into `FeedbackTuning` namespace
- `SHORT_PRESS_MS` replaced by `FeedbackTuning::SHORT_PRESS_MS`
- `setLowPowerEnabled()` retained; `loop()` still calls it per iteration

---

## Tx V7 — Battery monitoring; alert strobes; 5-boot recovery countdown restored

- **Battery voltage monitoring added**:
  - `readBatteryVoltageV()` reads `api.system.bat.get()` (SysVolt, NRF_SAADC_INPUT_VDD) with median-of-3 sampling
  - `DAILY_BATTERY_CHECK_INTERVAL_MS = 24 h` absolute-period timer
  - `gLastBatteryCheckAttemptMs`: absolute timestamp; never reset by button activity — continues through deep sleep
  - `batteryCheckDue()` / `batteryCheckRemainingMs()` helpers drive sleep duration
- **Battery alert levels**: `BATTERY_ALERT_NONE` / `BATTERY_ALERT_WARNING` / `BATTERY_ALERT_CRITICAL`
  - Warning strobe: every 60 s
  - Critical strobe: every 10 s
  - Alert signals play from sleep state; `batteryAlertRemainingMs()` also constrains sleep duration
- **Recovery system**:
  - 5-boot packed countdown **restored** (was replaced by RESETREAS in V6 — reverted)
  - `RECOVERY_FLAG_SOFT_RECOVERY` and its boot strobe **removed** — SOS failure is a normal radio outcome, not a crash
  - Only `RECOVERY_FLAG_REBOOT_RECOVERY` (watchdog trip) now writes the flash marker and shows the 3-blink boot strobe
  - `markRecoveryFlag()` guards against duplicate writes; only writes if flags or countdown changed
- `TX_PIN` / `LED_PIN` / `BUTTON_PIN` now defined via `WB_IO1` / `WB_IO2` macros with `#ifndef` guards (compile-time overridable)
- `gAttemptStartedAtMs` tracking retained from V4+
- Sleep duration is now `min(batteryCheckRemainingMs(), batteryAlertRemainingMs())` instead of indefinite

---

## Tx V8 — Non-blocking feedback overhaul; always-awake idle; battery state service

### Sleep logic removed (clean awake baseline)
- All sleep/wake logic removed: `api.system.sleep.all()`, `api.system.sleep.setup()`, `registerWakeupCallback()`, `wakeupCallback`, `batteryServiceTimerCallback`, `startBatteryServiceTimer`, `RAK_TIMER_*` APIs
- `TX_STATE_SLEEP` renamed to `TX_STATE_IDLE`; idle is now a lightweight polling loop with `delay(10)` between button checks
- `TX_STATE_BATTERY_SERVICE` enum value and its switch case removed
- `setLowPowerEnabled()` and all calls to it removed; LPM is not used in awake firmware
- `gButtonWakePending`, `gBatteryServiceWakePending`, `gBatteryTimerCallbackCount`, `gBatterySleepDispatchCount`, `gLastObservedTimerCallbackCount` removed
- Debug wake-source stage-probe pulse calls removed
- `transitionToSleepState()` renamed to `transitionToIdleState()`; all call sites updated

### Non-blocking FeedbackController (from V6 architecture, applied here)
- `FeedbackController gFeedback` singleton replaces all free-function feedback calls taking pin arguments
- New `FeedbackTuning.h` namespace consolidates all timing and PWM constants (`SHORT_PRESS_MS`, `LONG_PRESS_TOTAL_MS`, `LONG_PRESS_MS`, warning ramp constants, cue durations)
- `feedbackInit()` / `feedbackStop()` / `feedbackUpdate()` / `feedbackBusy()` / `feedbackPlayPattern()` API
- `feedbackUpdate(gFeedback, millis())` called as the first statement every `loop()` tick
- All blocking feedback helpers removed: `feedbackIdle`, `feedbackButtonDetected`, `feedbackLongPressProgress`, `feedbackLongPressConfirmed`, `feedbackAckReceived`, `feedbackCallEstablished`, `feedbackAckOnly`, `feedbackFailure`, `motorRamp`
- `setState(TxState)` helper introduced: atomically sets `gState + gStateStartedAtMs` and dispatches feedback side-effects per state
- **Two new interaction states**:
  - `TX_STATE_PRESS_FEEDBACK`: entered on button-down, plays tap cue, waits for tap to finish before entering `WAIT_LONG_PRESS`
  - `TX_STATE_CONFIRM_LONG_PRESS`: entered after long-press threshold, plays confirmation haptic, waits for it to finish before sending SOS
- `COMPLETE` / `COMPLETE_ACK_ONLY` / `FAILED`: no longer call blocking feedback + immediate idle; each waits for `!feedbackBusy(gFeedback)` then calls `transitionToIdleState()`
- `gButtonPressedAtMs` renamed to `gPressStartedAtMs`; `gLongPressDebounced` removed

### Recovery system — 5-boot countdown restored (reverts V6 RESETREAS change)
- `readRecoveryState()` / `writeRecoveryState()` functions with packed 32-bit flash word (flags in low byte, boot count in bits [15:8])
- `RECOVERY_BOOT_COUNT_RESET_VALUE = 5`; marker auto-clears after 5 power cycles
- `RECOVERY_FLAG_SOFT_RECOVERY` (bit 0) retained alongside `RECOVERY_FLAG_REBOOT_RECOVERY` (bit 1)
- `maybeShowRecoveryBootStrobe()` shows 3 × 45 ms blinks on reboot recovery

### Battery state service (new in V8; no sleep coupling)
- `extern "C" void service_battery_get_SysVolt_level(float *)` ADC read via RUI3 system service (same path as V7)
- `readBatteryVoltageV()`: median-of-3 samples with 2 ms spacing and 5 ms settle delay before reading
- `BatteryAlertLevel` enum: `BATTERY_ALERT_NONE` / `BATTERY_ALERT_LOW` / `BATTERY_ALERT_CRITICAL`
- Thresholds: Low ≤ 2.80 V, Critical ≤ 2.75 V (higher than V7's 2.75/2.70 thresholds)
- **Two independent schedules in software** — no sleep timer required:
  - Voltage sample: every 24 h in normal state; every 60 min while Low or Critical
  - Reminder playback: every 60 s while Low or Critical
- Boot behaviour: first sample taken immediately on first safe idle window after boot, then on the 24-hour schedule
- On level change (including first detection): `gBatteryReminderPending = true` → immediate reminder on next idle opportunity
- On recovery to `BATTERY_ALERT_NONE`: reminder scheduling cleared, sample cadence returns to 24 h
- `batteryServiceUpdate(nowMs)` called each `loop()` tick after `watchdogService()`, before main `switch(gState)`
- **Safe-window gating**: sample and reminder only run when `gState == TX_STATE_IDLE`, button not pressed, and `!feedbackBusy(gFeedback)` — battery service never interrupts protocol flow or active feedback
- Battery alert feedback is **LED-only** (no motor): Low = 2 short flashes (28 ms on / 12 ms off / 90 ms gap), Critical = 5 short flashes same timing; both non-blocking through `FeedbackController`
- All battery tuning constants in `FeedbackTuning` namespace: `BATTERY_LOW_V`, `BATTERY_CRITICAL_V`, `BATTERY_CHECK_INTERVAL_MS`, `BATTERY_ALERT_REMINDER_INTERVAL_MS`, `BATTERY_ALERT_RECHECK_INTERVAL_MS`, `BATTERY_SAMPLE_COUNT`, `BATTERY_SAMPLE_SPACING_MS`, `BATTERY_SAMPLE_SETTLE_MS`, strobe timing/counts

---

## Active (tx/) — Same as Tx V8

The active `firmwares/tx/tx.ino` is an exact copy of Tx V7.
