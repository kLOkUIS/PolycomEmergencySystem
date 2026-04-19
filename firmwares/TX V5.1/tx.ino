#include <nrf_gpio.h>

#include "Protocol.h"
#include "Feedback.h"

static const uint32_t LED_PIN = P1_03;
static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t MOTOR_PIN = P1_04;

static const uint32_t TX_DEVICE_ID = 1;

static const uint32_t LORA_FREQ_HZ = 868000000;
static const uint16_t LORA_SF = 7;
static const uint16_t LORA_BW = 0;
static const uint16_t LORA_CR = 1;
static const uint16_t LORA_PREAMBLE = 8;
static const int16_t LORA_TX_POWER = 14;

static const uint32_t SHORT_PRESS_MS = 100;
static const uint32_t LONG_PRESS_MS = 3000;
static const uint32_t SEND_DONE_TIMEOUT_MS = 3000;
static const uint32_t ACK_TIMEOUT_MS = 3000;
static const uint32_t CALL_TIMEOUT_MS = 4000;
static const uint32_t RETRY_DELAY_MS = 1200;
static const uint32_t RX_WINDOW_TIMEOUT_MARGIN_MS = 600;
static const uint32_t ATTEMPT_HARD_TIMEOUT_MS = 14000;
static const uint32_t WAKE_BUTTON_DEBOUNCE_MS = 20;
static const uint8_t MAX_SOS_ATTEMPTS = 4;

// Short test interval; increase to a longer production value once timer wake is verified.
static const uint32_t BATTERY_SERVICE_INTERVAL_MS = 15000;
static const RAK_TIMER_ID BATTERY_SERVICE_TIMER_ID = RAK_TIMER_1;
static const uint32_t DEBUG_PULSE_ON_MS = 70;
static const uint32_t DEBUG_PULSE_OFF_MS = 90;
static const uint32_t DEBUG_AWAKE_TIMER_PROBE_MS = 20000;

// Active-session software watchdog tuning (TX only).
// Override at compile time with -DWD_STALL_TIMEOUT_MS=<ms> and -DWD_ABSOLUTE_TIMEOUT_MS=<ms>.
#ifndef WD_STALL_TIMEOUT_MS
#define WD_STALL_TIMEOUT_MS 15000UL
#endif

#ifndef WD_ABSOLUTE_TIMEOUT_MS
#define WD_ABSOLUTE_TIMEOUT_MS 60000UL
#endif

static const uint32_t RECOVERY_FLAG_SOFT_RECOVERY = (1u << 0);
static const uint32_t RECOVERY_FLAG_REBOOT_RECOVERY = (1u << 1);
static const uint32_t RECOVERY_BOOT_MARKER_FLASH_OFFSET = 0;
static const uint32_t RECOVERY_FLAG_VALID_MASK = RECOVERY_FLAG_SOFT_RECOVERY | RECOVERY_FLAG_REBOOT_RECOVERY;
static const uint8_t RECOVERY_BOOT_COUNT_RESET_VALUE = 5;
static const uint32_t RECOVERY_BOOT_COUNT_SHIFT = 8;
static const uint32_t RECOVERY_BOOT_COUNT_MASK = (0xFFu << RECOVERY_BOOT_COUNT_SHIFT);

enum TxState {
	TX_STATE_SLEEP = 0,
	TX_STATE_WAIT_LONG_PRESS,
	TX_STATE_SEND_CALL_TOGGLE,
	TX_STATE_WAIT_CALL_TOGGLE_TX_DONE,
	TX_STATE_SEND_SOS,
	TX_STATE_WAIT_TX_DONE,
	TX_STATE_WAIT_ACK,
	TX_STATE_WAIT_CALL,
	TX_STATE_RETRY_DELAY,
	TX_STATE_COMPLETE,
	TX_STATE_COMPLETE_ACK_ONLY,
	TX_STATE_FAILED,
	TX_STATE_BATTERY_SERVICE,
};

volatile bool gButtonWakePending = false;
volatile bool gBatteryServiceWakePending = false;
volatile bool gTxDone = false;
volatile bool gRxPending = false;
volatile bool gRxTimedOut = false;
volatile bool gRxError = false;

volatile uint32_t gBatteryTimerCallbackCount = 0;
uint32_t gBatterySleepDispatchCount = 0;

uint8_t gRxBuffer[255];
uint16_t gRxLength = 0;

TxState gState = TX_STATE_SLEEP;
uint32_t gStateStartedAtMs = 0;
uint32_t gBootedAtMs = 0;
uint32_t gLastObservedTimerCallbackCount = 0;
uint32_t gButtonPressedAtMs = 0;
uint32_t gSequence = 0;
uint8_t gAttemptCount = 0;
uint32_t gAttemptStartedAtMs = 0;
uint32_t gRxWindowStartedAtMs = 0;
uint32_t gLocalRadioFaultCount = 0;
bool gLongPressDebounced = false;
bool gWdActive = false;
uint32_t gWdArmedAtMs = 0;
uint32_t gWdLastProgressMs = 0;
TxState gWdLastState = TX_STATE_SLEEP;

static void readRecoveryState(uint32_t &flags, uint8_t &bootCount) {
	uint32_t rawState = 0;
	if (!api.system.flash.get(RECOVERY_BOOT_MARKER_FLASH_OFFSET, (uint8_t *)&rawState, sizeof(rawState))) {
		flags = 0;
		bootCount = 0;
		return;
	}

	// Erased flash defaults to 0xFFFFFFFF and should be treated as "no marker".
	if (rawState == 0xFFFFFFFFu) {
		flags = 0;
		bootCount = 0;
		return;
	}

	flags = rawState & RECOVERY_FLAG_VALID_MASK;
	bootCount = (uint8_t)((rawState & RECOVERY_BOOT_COUNT_MASK) >> RECOVERY_BOOT_COUNT_SHIFT);
}

static void writeRecoveryState(uint32_t flags, uint8_t bootCount) {
	const uint32_t sanitizedFlags = flags & RECOVERY_FLAG_VALID_MASK;
	const uint8_t sanitizedBootCount = (sanitizedFlags == 0) ? 0 : bootCount;
	const uint32_t packedState = sanitizedFlags | (((uint32_t)sanitizedBootCount) << RECOVERY_BOOT_COUNT_SHIFT);
	api.system.flash.set(RECOVERY_BOOT_MARKER_FLASH_OFFSET, (uint8_t *)&packedState, sizeof(packedState));
}

static void markRecoveryFlag(uint32_t flag) {
	uint32_t flags = 0;
	uint8_t bootCount = 0;
	readRecoveryState(flags, bootCount);
	const uint32_t updatedFlags = (flags | flag) & RECOVERY_FLAG_VALID_MASK;
	if (updatedFlags == flags && bootCount == RECOVERY_BOOT_COUNT_RESET_VALUE) {
		return;
	}
	writeRecoveryState(updatedFlags, RECOVERY_BOOT_COUNT_RESET_VALUE);
}

static void rebootForRecovery() {
	markRecoveryFlag(RECOVERY_FLAG_REBOOT_RECOVERY);
	api.system.reboot();
}

static void maybeShowRecoveryBootStrobe() {
	uint32_t flags = 0;
	uint8_t bootCount = 0;
	readRecoveryState(flags, bootCount);
	if (flags == 0) {
		return;
	}

	if ((flags & RECOVERY_FLAG_REBOOT_RECOVERY) != 0) {
		for (uint8_t i = 0; i < 3; i++) {
			digitalWrite(LED_PIN, HIGH);
			delay(45);
			digitalWrite(LED_PIN, LOW);
			delay(45);
		}
	}

	if (bootCount > 0) {
		bootCount--;
	}

	if (bootCount == 0) {
		writeRecoveryState(0, 0);
	} else {
		writeRecoveryState(flags, bootCount);
	}
}

static void watchdogArm() {
	if (gWdActive) {
		return;
	}
	const uint32_t nowMs = millis();
	gWdActive = true;
	gWdArmedAtMs = nowMs;
	gWdLastProgressMs = nowMs;
	gWdLastState = gState;
}

static void watchdogDisarm() {
	gWdActive = false;
}

static void watchdogTouch() {
	if (!gWdActive) {
		return;
	}
	gWdLastProgressMs = millis();
}

static void watchdogService() {
	if (gState == TX_STATE_SLEEP) {
		watchdogDisarm();
		gWdLastState = gState;
		return;
	}

	if (!gWdActive) {
		watchdogArm();
	}

	if (gState != gWdLastState) {
		gWdLastState = gState;
		watchdogTouch();
	}

	const uint32_t nowMs = millis();
	if ((uint32_t)(nowMs - gWdLastProgressMs) >= WD_STALL_TIMEOUT_MS || (uint32_t)(nowMs - gWdArmedAtMs) >= WD_ABSOLUTE_TIMEOUT_MS) {
		rebootForRecovery();
	}
}

static bool elapsedAtLeast(uint32_t sinceMs, uint32_t durationMs) {
	return (uint32_t)(millis() - sinceMs) >= durationMs;
}

static bool buttonPressedRaw() {
	return nrf_gpio_pin_read(BUTTON_PIN) == 0;
}

static void debugPulseCount(uint8_t count) {
	for (uint8_t i = 0; i < count; i++) {
		digitalWrite(LED_PIN, HIGH);
		delay(DEBUG_PULSE_ON_MS);
		digitalWrite(LED_PIN, LOW);
		delay(DEBUG_PULSE_OFF_MS);
	}
}

void wakeupCallback(void) {
	// Only latch a wake request; all debounce and press timing runs in the main loop.
	gButtonWakePending = true;
}

void batteryServiceTimerCallback(void *) {
	gBatteryServiceWakePending = true;
	gBatteryTimerCallbackCount++;
}

void sendCallback(void) {
	gTxDone = true;
}

void receiveCallback(rui_lora_p2p_recv_t data) {
	if (data.Status == LORA_P2P_RXDONE) {
		uint16_t copyLength = data.BufferSize;
		if (copyLength > sizeof(gRxBuffer)) {
			copyLength = sizeof(gRxBuffer);
		}

		memcpy(gRxBuffer, data.Buffer, copyLength);
		gRxLength = copyLength;
		gRxPending = true;
	} else if (data.Status == LORA_P2P_RXTIMEOUT) {
		gRxTimedOut = true;
	} else if (data.Status == LORA_P2P_RXERROR) {
		gRxError = true;
	}
}

static void resetRadioFlags() {
	gTxDone = false;
	gRxPending = false;
	gRxTimedOut = false;
	gRxError = false;
	gRxLength = 0;
}

static bool startReceiveWindow(uint32_t timeoutMs) {
	gRxPending = false;
	gRxTimedOut = false;
	gRxError = false;
	// Stop any lingering receive state before re-arming a new window.
	api.lora.precv(0);
	if (!api.lora.precv(timeoutMs)) {
		return false;
	}
	gRxWindowStartedAtMs = millis();
	watchdogTouch();
	return true;
}

static bool takeReceivedMessage(ProtocolMessage &message) {
	if (!gRxPending) {
		return false;
	}

	uint8_t localBuffer[255];
	uint16_t localLength = 0;

	noInterrupts();
	localLength = gRxLength;
	memcpy(localBuffer, gRxBuffer, localLength);
	gRxPending = false;
	interrupts();

	return protocolParseMessage(localBuffer, localLength, message);
}

static bool radioWindowTimedOut() {
	if (gRxTimedOut || gRxError) {
		gRxTimedOut = false;
		gRxError = false;
		return true;
	}

	return false;
}

static bool configureLoRaP2P() {
	if (api.lora.nwm.get() != 0) {
		api.lora.nwm.set();
		api.system.reboot();
	}

	api.lora.registerPSendCallback(sendCallback);
	api.lora.registerPRecvCallback(receiveCallback);

	bool ok = true;
	ok &= api.lora.pfreq.set(LORA_FREQ_HZ);
	ok &= api.lora.psf.set(LORA_SF);
	ok &= api.lora.pbw.set(LORA_BW);
	ok &= api.lora.pcr.set(LORA_CR);
	ok &= api.lora.ppl.set(LORA_PREAMBLE);
	ok &= api.lora.ptp.set(LORA_TX_POWER);
	return ok;
}

static bool sendMessage(PacketType type) {
	ProtocolMessage message;
	message.type = type;
	message.deviceId = TX_DEVICE_ID;
	message.sequence = gSequence;

	char payload[PROTOCOL_MAX_PAYLOAD];
	if (!protocolFormatMessage(message, payload, sizeof(payload))) {
		return false;
	}

	resetRadioFlags();
	return api.lora.psend(strlen(payload), (uint8_t *)payload);
}

static bool attemptHardTimeoutReached() {
	return elapsedAtLeast(gAttemptStartedAtMs, ATTEMPT_HARD_TIMEOUT_MS);
}

static bool rxWindowExpired(uint32_t timeoutMs) {
	return elapsedAtLeast(gRxWindowStartedAtMs, timeoutMs + RX_WINDOW_TIMEOUT_MARGIN_MS);
}

static void markLocalRadioFault() {
	// Local radio faults (send/receive-arm failures) mark soft recovery for diagnostics.
	gLocalRadioFaultCount++;
	markRecoveryFlag(RECOVERY_FLAG_SOFT_RECOVERY);
	watchdogTouch();
}

static void scheduleRetry() {
	watchdogTouch();
	if (gAttemptCount < MAX_SOS_ATTEMPTS) {
		gState = TX_STATE_RETRY_DELAY;
		gStateStartedAtMs = millis();
	} else {
		gState = TX_STATE_FAILED;
		gStateStartedAtMs = millis();
	}
}

static void handleLinkProtocolFailureRetry() {
	// Link/protocol failures (missing ACK, RX timeout) retry without marking a radio fault.
	scheduleRetry();
}

static void handleLinkProtocolFailureAckOnly() {
	// Post-ACK link/protocol miss degrades to ACK-only completion without marking a radio fault.
	gState = TX_STATE_COMPLETE_ACK_ONLY;
	gStateStartedAtMs = millis();
}

static void handleLocalRadioFaultRetry() {
	markLocalRadioFault();
	scheduleRetry();
}

static void handleLocalRadioFaultAckOnly() {
	markLocalRadioFault();
	handleLinkProtocolFailureAckOnly();
}

static void handleLocalRadioFaultTerminal() {
	markLocalRadioFault();
	gState = TX_STATE_FAILED;
	gStateStartedAtMs = millis();
}

static void transitionToSleepState() {
	// Stop radio RX before entering TX_STATE_SLEEP. The sleep state calls sleep.all() and
	// blocks there; radio must be idle before the MCU sleeps.
	api.lora.precv(0);
	gState = TX_STATE_SLEEP;
	gStateStartedAtMs = millis();
}

static void enterWaitLongPressState() {
	gLongPressDebounced = false;
	gButtonPressedAtMs = 0;
	gState = TX_STATE_WAIT_LONG_PRESS;
	gStateStartedAtMs = millis();
}

static void startBatteryServiceTimer(bool &createOk, bool &startOk) {
	createOk = api.system.timer.create(BATTERY_SERVICE_TIMER_ID, batteryServiceTimerCallback, RAK_TIMER_PERIODIC);
	startOk = false;
	if (createOk) {
		startOk = api.system.timer.start(BATTERY_SERVICE_TIMER_ID, BATTERY_SERVICE_INTERVAL_MS, NULL);
	}
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	pinMode(MOTOR_PIN, OUTPUT);
	nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
	maybeShowRecoveryBootStrobe();
	feedbackIdle(LED_PIN, MOTOR_PIN);

	api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
	api.system.sleep.registerWakeupCallback(wakeupCallback);

	if (!configureLoRaP2P()) {
		markLocalRadioFault();
		while (true) {
			feedbackFailure(LED_PIN, MOTOR_PIN);
			delay(200);
		}
	}

	bool timerCreateOk = false;
	bool timerStartOk = false;
	startBatteryServiceTimer(timerCreateOk, timerStartOk);
	// Debug stage 1: timer create/start result marker.
	// 1 pulse = create failed, 2 pulses = start failed, 3 pulses = both OK.
	if (!timerCreateOk) {
		debugPulseCount(1);
	} else if (!timerStartOk) {
		debugPulseCount(2);
	} else {
		debugPulseCount(3);
	}

	gBootedAtMs = millis();
}

void loop() {
	// Debug stage 2 probe: keep MCU awake briefly and show callback activity directly.
	// 4 pulses means the timer callback count advanced while awake.
	if (!elapsedAtLeast(gBootedAtMs, DEBUG_AWAKE_TIMER_PROBE_MS)) {
		uint32_t callbackCountSnapshot = 0;
		noInterrupts();
		callbackCountSnapshot = gBatteryTimerCallbackCount;
		interrupts();

		if (callbackCountSnapshot != gLastObservedTimerCallbackCount) {
			gLastObservedTimerCallbackCount = callbackCountSnapshot;
			debugPulseCount(4);
		}

		delay(10);
		return;
	}

	ProtocolMessage message;
	watchdogService();

	switch (gState) {
	case TX_STATE_SLEEP:
		// One single sleep point. Bounded timeout lets us distinguish true timer-flag wake
		// from timeout wake when sleep.all() does not resume on timer callbacks.
		// Radio RX is already stopped by transitionToSleepState() before entering this state.
		api.system.sleep.all(BATTERY_SERVICE_INTERVAL_MS);
		{
			// Atomically snapshot and clear wake flags set asynchronously by callbacks.
			bool buttonWake = false;
			bool batteryServiceWake = false;
			noInterrupts();
			buttonWake = gButtonWakePending;
			batteryServiceWake = gBatteryServiceWakePending;
			gButtonWakePending = false;
			gBatteryServiceWakePending = false;
			interrupts();

			// Button wake has priority over timer wake.
			if (buttonWake) {
				// Diagnostic marker: wake classified as button.
				debugPulseCount(6);
				enterWaitLongPressState();
			} else if (batteryServiceWake) {
				gBatterySleepDispatchCount++;
				// Debug stage 3 marker: sleep dispatch selected battery-service by timer flag.
				debugPulseCount(1);
				gState = TX_STATE_BATTERY_SERVICE;
				gStateStartedAtMs = millis();
			} else {
				// Deep-interaction marker: woke only by sleep timeout, not by timer callback flag.
				debugPulseCount(5);
				gState = TX_STATE_BATTERY_SERVICE;
				gStateStartedAtMs = millis();
			}
		}
		break;

	case TX_STATE_WAIT_LONG_PRESS:
		if (!gLongPressDebounced) {
			if (!buttonPressedRaw()) {
				// Diagnostic marker: button wake path selected, but no actual button press held.
				debugPulseCount(7);
				transitionToSleepState();
				break;
			}

			if (!elapsedAtLeast(gStateStartedAtMs, WAKE_BUTTON_DEBOUNCE_MS)) {
				break;
			}

			gLongPressDebounced = true;
			gButtonPressedAtMs = millis();
			feedbackButtonDetected(LED_PIN, MOTOR_PIN);
		}

		if (!buttonPressedRaw()) {
			const uint32_t pressDurationMs = millis() - gButtonPressedAtMs;
			if (pressDurationMs >= SHORT_PRESS_MS && pressDurationMs < LONG_PRESS_MS) {
				gSequence++;
				gState = TX_STATE_SEND_CALL_TOGGLE;
				gStateStartedAtMs = millis();
			} else {
				transitionToSleepState();
			}
			break;
		}

		feedbackLongPressProgress(LED_PIN, MOTOR_PIN, millis() - gButtonPressedAtMs, LONG_PRESS_MS);

		if (millis() - gButtonPressedAtMs >= LONG_PRESS_MS) {
			// Threshold reached: short dot confirmation, then start SOS.
			feedbackLongPressConfirmed(LED_PIN, MOTOR_PIN);
			gSequence++;
			gAttemptCount = 0;
			gState = TX_STATE_SEND_SOS;
			gStateStartedAtMs = millis();
		}
		break;

	case TX_STATE_SEND_CALL_TOGGLE:
		if (sendMessage(PACKET_CALL_TOGGLE)) {
			gState = TX_STATE_WAIT_CALL_TOGGLE_TX_DONE;
			gStateStartedAtMs = millis();
		} else {
			handleLocalRadioFaultTerminal();
		}
		break;

	case TX_STATE_WAIT_CALL_TOGGLE_TX_DONE:
		if (gTxDone) {
			gTxDone = false;
			watchdogTouch();
			transitionToSleepState();
		} else if (millis() - gStateStartedAtMs >= SEND_DONE_TIMEOUT_MS) {
			// TX-done timeout with no callback is a local radio fault.
			handleLocalRadioFaultTerminal();
		}
		break;

	case TX_STATE_SEND_SOS:
		gAttemptCount++;
		gAttemptStartedAtMs = millis();
		if (sendMessage(PACKET_SOS)) {
			gState = TX_STATE_WAIT_TX_DONE;
			gStateStartedAtMs = millis();
		} else {
			handleLocalRadioFaultRetry();
		}
		break;

	case TX_STATE_WAIT_TX_DONE:
		if (attemptHardTimeoutReached()) {
			handleLinkProtocolFailureRetry();
			break;
		}

		if (gTxDone) {
			gTxDone = false;
			watchdogTouch();
			if (startReceiveWindow(ACK_TIMEOUT_MS)) {
				gState = TX_STATE_WAIT_ACK;
				gStateStartedAtMs = millis();
			} else {
				handleLocalRadioFaultRetry();
			}
		} else if (elapsedAtLeast(gStateStartedAtMs, SEND_DONE_TIMEOUT_MS)) {
			// TX-done timeout with no callback is a local radio fault.
			handleLocalRadioFaultRetry();
		}
		break;

	case TX_STATE_WAIT_ACK:
		if (attemptHardTimeoutReached()) {
			handleLinkProtocolFailureRetry();
			break;
		}

		if (rxWindowExpired(ACK_TIMEOUT_MS)) {
			// Soft ACK window expiry is a link/protocol failure, not a local radio fault.
			handleLinkProtocolFailureRetry();
			break;
		}

		if (takeReceivedMessage(message)) {
			watchdogTouch();
			if (protocolMatchesResponse(message, PACKET_ACK, TX_DEVICE_ID, gSequence)) {
				feedbackAckReceived(LED_PIN);
				if (startReceiveWindow(CALL_TIMEOUT_MS)) {
					gState = TX_STATE_WAIT_CALL;
					gStateStartedAtMs = millis();
				} else {
					handleLocalRadioFaultRetry();
				}
			} else if (protocolMatchesResponse(message, PACKET_CALL, TX_DEVICE_ID, gSequence)) {
				gState = TX_STATE_COMPLETE;
				gStateStartedAtMs = millis();
			} else {
				if (!startReceiveWindow(ACK_TIMEOUT_MS)) {
					handleLocalRadioFaultRetry();
				}
			}
		} else if (radioWindowTimedOut()) {
			// RX timeout/error while awaiting ACK is a link/protocol failure.
			handleLinkProtocolFailureRetry();
		}
		break;

	case TX_STATE_WAIT_CALL:
		if (attemptHardTimeoutReached()) {
			handleLinkProtocolFailureAckOnly();
			break;
		}

		if (rxWindowExpired(CALL_TIMEOUT_MS)) {
			handleLinkProtocolFailureAckOnly();
			break;
		}

		if (takeReceivedMessage(message)) {
			watchdogTouch();
			if (protocolMatchesResponse(message, PACKET_CALL, TX_DEVICE_ID, gSequence)) {
				gState = TX_STATE_COMPLETE;
				gStateStartedAtMs = millis();
			} else {
				// Failing to re-open RX after an unexpected message is a local radio fault;
				// degrade to ACK-only rather than re-running the full SOS cycle.
				if (!startReceiveWindow(CALL_TIMEOUT_MS)) {
					handleLocalRadioFaultAckOnly();
				}
			}
		} else if (radioWindowTimedOut()) {
			handleLinkProtocolFailureAckOnly();
		}
		break;

	case TX_STATE_RETRY_DELAY:
		if (millis() - gStateStartedAtMs >= RETRY_DELAY_MS) {
			gState = TX_STATE_SEND_SOS;
			gStateStartedAtMs = millis();
		}
		break;

	case TX_STATE_COMPLETE:
		feedbackCallEstablished(LED_PIN, MOTOR_PIN);
		transitionToSleepState();
		break;

	case TX_STATE_COMPLETE_ACK_ONLY:
		feedbackAckOnly(LED_PIN, MOTOR_PIN);
		transitionToSleepState();
		break;

	case TX_STATE_FAILED:
		feedbackFailure(LED_PIN, MOTOR_PIN);
		transitionToSleepState();
		break;

	case TX_STATE_BATTERY_SERVICE:
		// Debug stage 2/3 proof: callback woke system and dispatch reached this state.
		// 2-pulse pattern is intentionally distinct from other debug markers.
		digitalWrite(LED_PIN, HIGH);
		delay(80);
		digitalWrite(LED_PIN, LOW);
		delay(80);
		digitalWrite(LED_PIN, HIGH);
		delay(80);
		digitalWrite(LED_PIN, LOW);
		transitionToSleepState();
		break;
	}
}