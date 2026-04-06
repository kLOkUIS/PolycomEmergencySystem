#include <nrf_gpio.h>

#include "Protocol.h"
#include "Feedback.h"

extern "C" void service_battery_get_SysVolt_level(float *sys_lvl);

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

static const uint32_t SEND_DONE_TIMEOUT_MS = 3000;
static const uint32_t ACK_TIMEOUT_MS = 3000;
static const uint32_t CALL_TIMEOUT_MS = 4000;
static const uint32_t RETRY_DELAY_MS = 1200;
static const uint32_t RX_WINDOW_TIMEOUT_MARGIN_MS = 600;
static const uint32_t ATTEMPT_HARD_TIMEOUT_MS = 14000;
static const uint8_t MAX_SOS_ATTEMPTS = 4;
static const uint32_t DAILY_BATTERY_CHECK_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL;
static const uint32_t BATTERY_SETTLE_AFTER_WAKE_MS = 5;
static const float BATTERY_WARNING_V = 2.75f;
static const float BATTERY_CRITICAL_V = 2.70f;
static const uint8_t BATTERY_SYSV_SAMPLE_COUNT = 3;
static const uint32_t BATTERY_SYSV_SAMPLE_SPACING_MS = 2;
static const uint32_t BATTERY_WARNING_STROBE_INTERVAL_MS = 60UL * 1000UL;
static const uint32_t BATTERY_CRITICAL_STROBE_INTERVAL_MS = 10UL * 1000UL;
static const uint32_t BATTERY_STROBE_ON_MS = 28;
static const uint32_t BATTERY_STROBE_OFF_MS = 12;
static const uint8_t BATTERY_WARNING_STROBE_COUNT = 2;
static const uint8_t BATTERY_CRITICAL_STROBE_COUNT = 5;
static const uint32_t BATTERY_STROBE_GAP_MS = 90;

// Active-session software watchdog tuning (TX only).
// Override at compile time with -DWD_STALL_TIMEOUT_MS=<ms> and -DWD_ABSOLUTE_TIMEOUT_MS=<ms>.
#ifndef WD_STALL_TIMEOUT_MS
#define WD_STALL_TIMEOUT_MS 15000UL
#endif

#ifndef WD_ABSOLUTE_TIMEOUT_MS
#define WD_ABSOLUTE_TIMEOUT_MS 60000UL
#endif

static const uint32_t RECOVERY_FLAG_REBOOT_RECOVERY = (1u << 1);
static const uint32_t RECOVERY_BOOT_MARKER_FLASH_OFFSET = 0;
static const uint32_t RECOVERY_FLAG_VALID_MASK = RECOVERY_FLAG_REBOOT_RECOVERY;
static const uint8_t RECOVERY_BOOT_COUNT_RESET_VALUE = 5;
static const uint32_t RECOVERY_BOOT_COUNT_SHIFT = 8;
static const uint32_t RECOVERY_BOOT_COUNT_MASK = (0xFFu << RECOVERY_BOOT_COUNT_SHIFT);

enum TxState {
	TX_STATE_SLEEP = 0,
	TX_STATE_PRESS_FEEDBACK,
	TX_STATE_WAIT_LONG_PRESS,
	TX_STATE_CONFIRM_LONG_PRESS,
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
};

enum BatteryAlertLevel {
	BATTERY_ALERT_NONE = 0,
	BATTERY_ALERT_WARNING,
	BATTERY_ALERT_CRITICAL,
};

volatile uint32_t gWakeCallbackCount = 0;
volatile bool gTxDone = false;
volatile bool gRxPending = false;
volatile bool gRxTimedOut = false;
volatile bool gRxError = false;

uint8_t gRxBuffer[255];
uint16_t gRxLength = 0;

FeedbackController gFeedback;
TxState gState = TX_STATE_SLEEP;
uint32_t gStateStartedAtMs = 0;
uint32_t gPressStartedAtMs = 0;
uint32_t gSequence = 0;
uint8_t gAttemptCount = 0;
uint32_t gAttemptStartedAtMs = 0;
uint32_t gRxWindowStartedAtMs = 0;
bool gRequireReleaseBeforeNextTrigger = false;
bool gLowPowerEnabled = true;
bool gWdActive = false;
uint32_t gWdArmedAtMs = 0;
uint32_t gWdLastProgressMs = 0;
TxState gWdLastState = TX_STATE_SLEEP;
float gLastBatteryVoltageV = 0.0f;
BatteryAlertLevel gBatteryAlertLevel = BATTERY_ALERT_NONE;
uint32_t gLastBatteryCheckAttemptMs = 0;
uint32_t gLastBatteryAlertSignalMs = 0;

static void setState(TxState newState) {
	gState = newState;
	gStateStartedAtMs = millis();

	switch (newState) {
	case TX_STATE_SLEEP:
		feedbackStop(gFeedback);
		break;
	case TX_STATE_PRESS_FEEDBACK:
		feedbackPlayButtonTap(gFeedback);
		break;
	case TX_STATE_WAIT_LONG_PRESS:
		feedbackStartLongPressWarning(gFeedback, gPressStartedAtMs, FeedbackTuning::LONG_PRESS_MS);
		break;
	case TX_STATE_CONFIRM_LONG_PRESS:
		feedbackPlayLongPressConfirmed(gFeedback);
		break;
	case TX_STATE_COMPLETE:
		feedbackPlayCallEstablished(gFeedback);
		break;
	case TX_STATE_COMPLETE_ACK_ONLY:
		feedbackPlayAckOnly(gFeedback);
		break;
	case TX_STATE_FAILED:
		feedbackPlayFailure(gFeedback);
		break;
	default:
		break;
	}
}

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

static bool batteryCheckDue() {
	return elapsedAtLeast(gLastBatteryCheckAttemptMs, DAILY_BATTERY_CHECK_INTERVAL_MS);
}

static uint32_t batteryCheckRemainingMs() {
	const uint32_t elapsedMs = (uint32_t)(millis() - gLastBatteryCheckAttemptMs);
	if (elapsedMs >= DAILY_BATTERY_CHECK_INTERVAL_MS) {
		return 0;
	}
	return DAILY_BATTERY_CHECK_INTERVAL_MS - elapsedMs;
}

static uint32_t batteryAlertIntervalMs(BatteryAlertLevel level) {
	switch (level) {
	case BATTERY_ALERT_CRITICAL:
		return BATTERY_CRITICAL_STROBE_INTERVAL_MS;
	case BATTERY_ALERT_WARNING:
		return BATTERY_WARNING_STROBE_INTERVAL_MS;
	default:
		return 0;
	}
}

static bool batteryAlertSignalDue() {
	const uint32_t intervalMs = batteryAlertIntervalMs(gBatteryAlertLevel);
	if (intervalMs == 0) {
		return false;
	}
	return elapsedAtLeast(gLastBatteryAlertSignalMs, intervalMs);
}

static uint32_t batteryAlertRemainingMs() {
	const uint32_t intervalMs = batteryAlertIntervalMs(gBatteryAlertLevel);
	if (intervalMs == 0) {
		return UINT32_MAX;
	}
	const uint32_t elapsedMs = (uint32_t)(millis() - gLastBatteryAlertSignalMs);
	if (elapsedMs >= intervalMs) {
		return 0;
	}
	return intervalMs - elapsedMs;
}

static void setLowPowerEnabled(bool enabled) {
	if (gLowPowerEnabled == enabled) {
		return;
	}
	api.system.lpm.set(enabled ? 1 : 0);
	gLowPowerEnabled = enabled;
}

static bool buttonPressedRaw() {
	return nrf_gpio_pin_read(BUTTON_PIN) == 0;
}

static float readBatteryVoltageV() {
	float samples[BATTERY_SYSV_SAMPLE_COUNT];
	for (uint8_t i = 0; i < BATTERY_SYSV_SAMPLE_COUNT; i++) {
		service_battery_get_SysVolt_level(&samples[i]);
		if (i + 1 < BATTERY_SYSV_SAMPLE_COUNT) {
			delay(BATTERY_SYSV_SAMPLE_SPACING_MS);
		}
	}

	const float v0 = samples[0], v1 = samples[1], v2 = samples[2];
	float median;
	if (v0 <= v1) {
		median = (v1 <= v2) ? v1 : (v0 <= v2 ? v2 : v0);
	} else {
		median = (v0 <= v2) ? v0 : (v1 <= v2 ? v2 : v1);
	}

	return median;
}

static BatteryAlertLevel getBatteryAlertLevel(float voltageV) {
	if (voltageV <= BATTERY_CRITICAL_V) {
		return BATTERY_ALERT_CRITICAL;
	}
	if (voltageV <= BATTERY_WARNING_V) {
		return BATTERY_ALERT_WARNING;
	}
	return BATTERY_ALERT_NONE;
}

static bool pulseBatteryAlertSingleStrobe() {
	if (buttonPressedRaw()) {
		return false;
	}

	digitalWrite(LED_PIN, HIGH);
	for (uint32_t elapsedMs = 0; elapsedMs < BATTERY_STROBE_ON_MS; elapsedMs++) {
		if (buttonPressedRaw()) {
			digitalWrite(LED_PIN, LOW);
			return false;
		}
		delay(1);
	}

	digitalWrite(LED_PIN, LOW);
	for (uint32_t elapsedMs = 0; elapsedMs < BATTERY_STROBE_OFF_MS; elapsedMs++) {
		if (buttonPressedRaw()) {
			return false;
		}
		delay(1);
	}
	return true;
}

static void playBatteryAlertSignal() {
	uint8_t strobeCount = 0;
	switch (gBatteryAlertLevel) {
	case BATTERY_ALERT_CRITICAL:
		strobeCount = BATTERY_CRITICAL_STROBE_COUNT;
		break;
	case BATTERY_ALERT_WARNING:
		strobeCount = BATTERY_WARNING_STROBE_COUNT;
		break;
	default:
		return;
	}

	for (uint8_t i = 0; i < strobeCount; i++) {
		if (!pulseBatteryAlertSingleStrobe()) {
			return;
		}
		if (i + 1 < strobeCount) {
			for (uint32_t elapsedMs = 0; elapsedMs < BATTERY_STROBE_GAP_MS; elapsedMs++) {
				if (buttonPressedRaw()) {
					return;
				}
				delay(1);
			}
		}
	}
}

void wakeupCallback(void) {
	gWakeCallbackCount++;
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

static void scheduleRetry() {
	watchdogTouch();
	if (gAttemptCount < MAX_SOS_ATTEMPTS) {
		setState(TX_STATE_RETRY_DELAY);
	} else {
		setState(TX_STATE_FAILED);
	}
}

static void enterSleepState() {
	if (gRequireReleaseBeforeNextTrigger) {
		if (!buttonPressedRaw()) {
			gRequireReleaseBeforeNextTrigger = false;
		} else {
			// Don't busy-poll while waiting for release; sleep shortly and re-check.
			api.system.sleep.all(100);
			return;
		}
	}

	if (buttonPressedRaw()) {
		gPressStartedAtMs = millis();
		setState(TX_STATE_PRESS_FEEDBACK);
		return;
	}

	// Stop the radio before sleeping — a lingering RX window keeps it powered.
	api.lora.precv(0);
	uint32_t wakeCountBeforeSleep = gWakeCallbackCount;
	const uint32_t batterySleepMs = batteryCheckRemainingMs();
	const uint32_t alertSleepMs = batteryAlertRemainingMs();
	uint32_t sleepMs = batterySleepMs < alertSleepMs ? batterySleepMs : alertSleepMs;
	if (sleepMs > 0) {
		api.system.sleep.all(sleepMs);
	}

	const bool wakeCallbackChanged = gWakeCallbackCount != wakeCountBeforeSleep;
	bool wokeByButtonEdge = false;
	if (wakeCallbackChanged) {
		for (uint8_t i = 0; i < 20; i++) {
			if (buttonPressedRaw()) {
				wokeByButtonEdge = true;
				break;
			}
			delay(1);
		}
	}

	if (!wokeByButtonEdge && batteryCheckDue()) {
		gLastBatteryCheckAttemptMs = millis();

		if (!buttonPressedRaw()) {
			delay(BATTERY_SETTLE_AFTER_WAKE_MS);
			if (!buttonPressedRaw()) {
				gLastBatteryVoltageV = readBatteryVoltageV();
				const BatteryAlertLevel previousAlertLevel = gBatteryAlertLevel;
				gBatteryAlertLevel = getBatteryAlertLevel(gLastBatteryVoltageV);
				if (gBatteryAlertLevel == BATTERY_ALERT_NONE) {
					gLastBatteryAlertSignalMs = millis();
				} else if (previousAlertLevel != gBatteryAlertLevel || previousAlertLevel == BATTERY_ALERT_NONE) {
					gLastBatteryAlertSignalMs = millis();
				}
			}
		}
	}

	if (!wokeByButtonEdge && batteryAlertSignalDue() && !buttonPressedRaw()) {
		gLastBatteryAlertSignalMs = millis();
		playBatteryAlertSignal();
	}

	// Re-arm long-press flow on any button-held wake condition.
	// This is more robust than relying only on wake callback counters.
	if (buttonPressedRaw()) {
		gPressStartedAtMs = millis();
		setState(TX_STATE_PRESS_FEEDBACK);
	}
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	pinMode(MOTOR_PIN, OUTPUT);
	feedbackInit(gFeedback, LED_PIN, MOTOR_PIN);
	nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
	maybeShowRecoveryBootStrobe();
	feedbackStop(gFeedback);

	setLowPowerEnabled(true);
	api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
	api.system.sleep.registerWakeupCallback(wakeupCallback);
	gLastBatteryCheckAttemptMs = millis();

	if (!configureLoRaP2P()) {
		while (true) {
			feedbackPlayFailure(gFeedback);
			while (feedbackBusy(gFeedback)) {
				feedbackUpdate(gFeedback, millis());
				delay(2);
			}
			delay(120);
		}
	}
}

void loop() {
	ProtocolMessage message;
	feedbackUpdate(gFeedback, millis());
	setLowPowerEnabled(gState == TX_STATE_SLEEP);
	watchdogService();

	switch (gState) {
	case TX_STATE_SLEEP:
		enterSleepState();
		break;

	case TX_STATE_PRESS_FEEDBACK:
		if (!buttonPressedRaw()) {
			const uint32_t pressDurationMs = millis() - gPressStartedAtMs;
			if (pressDurationMs >= FeedbackTuning::SHORT_PRESS_MS) {
				gSequence++;
				setState(TX_STATE_SEND_CALL_TOGGLE);
			} else {
				setState(TX_STATE_SLEEP);
			}
			break;
		}

		if (!feedbackBusy(gFeedback)) {
			setState(TX_STATE_WAIT_LONG_PRESS);
		}
		break;

	case TX_STATE_WAIT_LONG_PRESS:
		if (!buttonPressedRaw()) {
			feedbackStop(gFeedback);
			const uint32_t pressDurationMs = millis() - gPressStartedAtMs;
			if (pressDurationMs >= FeedbackTuning::SHORT_PRESS_MS && pressDurationMs < FeedbackTuning::LONG_PRESS_MS) {
				gSequence++;
				setState(TX_STATE_SEND_CALL_TOGGLE);
			} else {
				setState(TX_STATE_SLEEP);
			}
			break;
		}

		if (millis() - gPressStartedAtMs >= FeedbackTuning::LONG_PRESS_MS) {
			gRequireReleaseBeforeNextTrigger = true;
			gSequence++;
			gAttemptCount = 0;
			setState(TX_STATE_CONFIRM_LONG_PRESS);
		}
		break;

	case TX_STATE_CONFIRM_LONG_PRESS:
		if (!feedbackBusy(gFeedback)) {
			setState(TX_STATE_SEND_SOS);
		}
		break;

	case TX_STATE_SEND_CALL_TOGGLE:
		if (sendMessage(PACKET_CALL_TOGGLE)) {
			setState(TX_STATE_WAIT_CALL_TOGGLE_TX_DONE);
		} else {
			setState(TX_STATE_FAILED);
		}
		break;

	case TX_STATE_WAIT_CALL_TOGGLE_TX_DONE:
		if (gTxDone) {
			gTxDone = false;
			watchdogTouch();
			setState(TX_STATE_SLEEP);
		} else if (millis() - gStateStartedAtMs >= SEND_DONE_TIMEOUT_MS) {
			setState(TX_STATE_FAILED);
		}
		break;

	case TX_STATE_SEND_SOS:
		gAttemptCount++;
		gAttemptStartedAtMs = millis();
		if (sendMessage(PACKET_SOS)) {
			setState(TX_STATE_WAIT_TX_DONE);
		} else {
			scheduleRetry();
		}
		break;

	case TX_STATE_WAIT_TX_DONE:
		if (elapsedAtLeast(gAttemptStartedAtMs, ATTEMPT_HARD_TIMEOUT_MS)) {
			scheduleRetry();
			break;
		}

		if (gTxDone) {
			gTxDone = false;
			watchdogTouch();
			if (startReceiveWindow(ACK_TIMEOUT_MS)) {
				setState(TX_STATE_WAIT_ACK);
			} else {
				scheduleRetry();
			}
		} else if (elapsedAtLeast(gStateStartedAtMs, SEND_DONE_TIMEOUT_MS)) {
			scheduleRetry();
		}
		break;

	case TX_STATE_WAIT_ACK:
		if (elapsedAtLeast(gAttemptStartedAtMs, ATTEMPT_HARD_TIMEOUT_MS)) {
			scheduleRetry();
			break;
		}

		if (elapsedAtLeast(gRxWindowStartedAtMs, ACK_TIMEOUT_MS + RX_WINDOW_TIMEOUT_MARGIN_MS)) {
			scheduleRetry();
			break;
		}

		if (takeReceivedMessage(message)) {
			watchdogTouch();
			if (protocolMatchesResponse(message, PACKET_ACK, TX_DEVICE_ID, gSequence)) {
				if (startReceiveWindow(CALL_TIMEOUT_MS)) {
					setState(TX_STATE_WAIT_CALL);
				} else {
					scheduleRetry();
				}
			} else if (protocolMatchesResponse(message, PACKET_CALL, TX_DEVICE_ID, gSequence)) {
				setState(TX_STATE_COMPLETE);
			} else {
				if (!startReceiveWindow(ACK_TIMEOUT_MS)) {
					scheduleRetry();
				}
			}
		} else if (radioWindowTimedOut()) {
			scheduleRetry();
		}
		break;

	case TX_STATE_WAIT_CALL:
		if (elapsedAtLeast(gAttemptStartedAtMs, ATTEMPT_HARD_TIMEOUT_MS)) {
			setState(TX_STATE_COMPLETE_ACK_ONLY);
			break;
		}

		if (elapsedAtLeast(gRxWindowStartedAtMs, CALL_TIMEOUT_MS + RX_WINDOW_TIMEOUT_MARGIN_MS)) {
			setState(TX_STATE_COMPLETE_ACK_ONLY);
			break;
		}

		if (takeReceivedMessage(message)) {
			watchdogTouch();
			if (protocolMatchesResponse(message, PACKET_CALL, TX_DEVICE_ID, gSequence)) {
				setState(TX_STATE_COMPLETE);
			} else {
				if (!startReceiveWindow(CALL_TIMEOUT_MS)) {
					setState(TX_STATE_COMPLETE_ACK_ONLY);
				}
			}
		} else if (radioWindowTimedOut()) {
			setState(TX_STATE_COMPLETE_ACK_ONLY);
		}
		break;

	case TX_STATE_RETRY_DELAY:
		if (millis() - gStateStartedAtMs >= RETRY_DELAY_MS) {
			setState(TX_STATE_SEND_SOS);
		}
		break;

	case TX_STATE_COMPLETE:
		if (!feedbackBusy(gFeedback)) {
			setState(TX_STATE_SLEEP);
		}
		break;

	case TX_STATE_COMPLETE_ACK_ONLY:
		if (!feedbackBusy(gFeedback)) {
			setState(TX_STATE_SLEEP);
		}
		break;

	case TX_STATE_FAILED:
		if (!feedbackBusy(gFeedback)) {
			setState(TX_STATE_SLEEP);
		}
		break;
	}
	delay(2);
}