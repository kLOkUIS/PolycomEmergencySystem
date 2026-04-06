# RX Firmware Changelog

Hardware: RAK4631 (nRF52840), dial PCB  
LoRa: 868 MHz, SF7, BW125, CR4/5, preamble 8, +14 dBm — unchanged across all versions

---

## RX V1 — Simulated test responder (not production hardware)

- Runs on the knob/dev board with a single LED on `P1_03`
- **Randomised ACK/CALL simulation** for bench-testing the TX:
  - `ACK_SUCCESS_PERCENT = 80` — 80 % chance of sending ACK
  - `CALL_SUCCESS_PERCENT = 95` — 95 % chance of sending CALL after ACK
  - `shouldSimulateSuccess(percent)` uses `random()` for probability gating
- Debug serial always on (no compile flag; `Serial.begin(115200)` in setup)
- All TX output logged with `Serial.print`
- State machine: IDLE → SENDING_ACK → WAITING_FOR_CALL → SENDING_CALL
- `CALL_DELAY_MS = 1000` — 1 s between ACK done and CALL send
- RX window 65534 ms, reopened after each message or timeout
- No hardware key control, no CALL_SENSE input
- `blink()` helper for visual feedback on packet receive

---

## RX V2 — Real dial PCB hardware; key sequencer; CALL_SENSE input

- Completely rewritten for the production dial PCB
- **Hardware pins**: `LED_RX_PIN = P0_24`, `LED_DIAL_CTL_PIN = P0_31`, `CALL_SENSE_PIN = P0_25`, KEY1–KEY6 on `P1_01`–`P1_04`, `P0_05`, `P0_04`
- **Key sequencer added**: timed PWM-free key presses via `KeyAction` table
  - `PLACE_CALL_ACTIONS`: KEY1×3 → KEY2×1 → KEY3×1
  - `END_CALL_ACTIONS`: KEY3×1 → KEY1×3
  - `KEY_PRESS_MS = 220`, `KEY_REPEAT_GAP_MS = 180`, `KEY_STEP_GAP_MS = 350`
- **CALL_SENSE debounce**: 180 ms; LED_RX mirrors call-sense state
- Session state: device ID + sequence tracked; repeated SOS retries from same TX get ACK without re-triggering dial
- SOS message: ACK queued immediately → `PLACE_CALL_ACTIONS` started → waits for CALL_SENSE → sends PACKET_CALL
- `PACKET_END_CALL` handled: triggers `END_CALL_ACTIONS` while call is active
- `queueReply()` stops RX before transmit, restarts RX on TX done
- `LED_DIAL_CTL_PIN` blinks during key action steps
- RX packet pulse on LED_DIAL_CTL: 140 ms flash on any received message
- LPM set to 1 (low-power enabled)
- Debug serial controlled by `DEBUG_SERIAL` compile flag
- No TX done timeout — waits indefinitely for `gTxDone`
- No session timeout — session stays open until CALL_SENSE arrives

---

## RX V3 — CALL_TOGGLE replaces END_CALL; ring-cadence hold; LPM disabled

- `PACKET_END_CALL` replaced by `PACKET_CALL_TOGGLE` (protocol rename matching TX V3)
  - `END_CALL_ACTIONS` removed; `CALL_TOGGLE_ACTIONS` introduced (same key sequence)
  - `ACTION_PLAN_END_CALL` → `ACTION_PLAN_CALL_TOGGLE`
- **Ring-cadence active-hold added**: measured ON ~879 ms / OFF ~4121 ms
  - `CALL_ACTIVE_HOLD_MS = 4600` — hold time after ring pulse ends, so CALL_TOGGLE commands received between ring bursts still work
  - `CALL_ACTIVE_SHORT_BURST_MAX_MS = 1700` — short pulses treated as ringing; long pulses treated as real call (no hold on hangup)
  - `callControlAllowed()` gating: CALL_TOGGLE only executes if currently active or within the hold window
- **LPM disabled** (`api.system.lpm.set(0)`) — keeps RX fully awake for deterministic radio callback handling
- `gCallSenseActiveStartedAtMs` added to track burst duration
- Debug serial retained via `DEBUG_SERIAL` flag
- No TX done timeout, no session timeout, no watchdog

---

## RX V4 — TX done timeout; session timeout; radio event watchdog; `clearSessionState()`

- **TX done timeout**: `TX_DONE_TIMEOUT_MS = 3500` — if TX callback never fires, release pending TX state and restart RX
- **Session timeout**: `SESSION_AWAIT_CALL_SENSE_TIMEOUT_MS = 45 000` ms — if CALL_SENSE never arrives, clear session so a new SOS can be accepted
  - `gSessionStartedAtMs` tracking added; `updateSessionTimeout()` called each loop
- **RX radio watchdog**: `RX_WATCHDOG_MS = 70 000` — if no radio event for 70 s, forcibly restart RX window
  - `gLastRadioEventAtMs` tracking; `markRadioEvent()` called on every TX/RX callback and on `restartReceive()`
- **`clearSessionState()`** helper introduced (was inline in V3)
- `startAction()` now records `gActionStartedAtMs`; `updateActionPlan()` checks `ACTION_PLAN_TIMEOUT_MS = 20 000` — stuck action plans self-cancel
- `gActionStartedAtMs` and `gPendingTxStartedAtMs` fields added
- Session management tightened: `queueReply()` only starts action plan if ACK queue succeeds; otherwise clears session
- `handleSosMessage()` no longer starts action plan on a session-matches retry — just re-ACKs
- `handleCallToggleMessage()` clears any stale session before starting action
- Debug serial removed; all `DPRINT`/`DPRINTLN` removed from serviceReceivedMessage, updatePendingTransmit, updateActionPlan
- `markRadioCallbackEvent()` alias added (same role, clearer name)

---

## RX V5 — Multi-stage stuck-radio watchdog; recovery flash system; 5-boot countdown

- **Multi-stage stuck radio watchdog**:
  - Stage 1 (`RX_WD_LISTEN_STUCK_MS = 420 000`): fires if `gLastRadioCallbackAtMs` not updated for 7 minutes → calls `recoverStuckReceiveWatched()`
  - Stage 2 (`RX_WD_FULL_RECOVERY_AFTER_FAILURES = 2`): after 2 restart failures, attempts a full `configureLoRaP2P()` re-init before giving up
  - Stage 3 (`RX_WD_MAX_RESTART_FAILURES = 5`): after 5 failures, calls `rebootForRecovery()` → writes recovery flash marker → reboots
  - All thresholds overridable at compile time via `#define` guards
- **Recovery flash system added** (mirrors TX V5):
  - `RECOVERY_FLAG_SOFT_RECOVERY` (bit 0): written by `restartReceiveWatched()` on RX restart failure
  - `RECOVERY_FLAG_REBOOT_RECOVERY` (bit 1): written by `rebootForRecovery()` (watchdog trip)
  - 5-boot countdown packed in bits [15:8]; marker auto-clears after 5 power cycles
  - `maybeShowRecoveryBootStrobe()` on boot: 3× 45 ms blinks for reboot recovery; 120 ms pause + 180 ms pulse for soft recovery
- **Idle RX refresh**: `RX_IDLE_REFRESH_MS = 60 000` — if no radio event for 60 s (but not stuck), silently restart RX window via `refreshReceiveWindowIdleWatchdog()` to prevent modem drift
- `gLastRadioCallbackAtMs` added separately from `gLastRadioEventAtMs`:
  - `gLastRadioEventAtMs`: updated on `restartReceive()` calls (any radio operation)
  - `gLastRadioCallbackAtMs`: updated only when `sendCallback` / `receiveCallback` fires (true modem feedback)
- `gRxRestartFailureCount` / `gLastStuckRecoveryAttemptAtMs` / `gLastFullRecoveryAtMs` fields added
- SOS de-duplication logic relaxed: if CALL_SENSE never arrived, new SOS replaces stale waiting session (no longer ignored while busy)
- `restartReceiveWatched()` used in all internal recovery paths; `restartReceive()` retained for direct initial arming
- `maybeShowRecoveryBootStrobe()` called before `allKeysLow()` in setup

---

## RX V6 — Bug-fix cleanup of V5 (no functional changes)

RX V6 and V5 produce identical binaries. The changes are:
- Removal of a duplicate `markRecoveryFlag(RECOVERY_FLAG_SOFT_RECOVERY)` call inside `restartReceiveWatched()` that appeared twice in V5 (before and inside the full-recovery block)
- Minor whitespace / indentation corrections
- `recoverStuckReceiveWatched()` call sequence confirmed correct (stop → restart → update callback timestamp)

---

## RX V7 — Same as RX V6

RX V7 is an identical copy of RX V6. No code changes.

---

## Active (rx/) — Same as RX V6/V7

The active `firmwares/rx/rx.ino` is identical to RX V6 and RX V7.
