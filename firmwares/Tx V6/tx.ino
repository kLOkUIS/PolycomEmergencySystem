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

static const uint32_t SEND_DONE_TIMEOUT_MS = 3000;
static const uint32_t ACK_TIMEOUT_MS = 3000;
static const uint32_t CALL_TIMEOUT_MS = 4000;
static const uint32_t RETRY_DELAY_MS = 1200;
static const uint32_t RX_WINDOW_TIMEOUT_MARGIN_MS = 600;
static const uint32_t ATTEMPT_HARD_TIMEOUT_MS = 14000;
static const uint8_t MAX_SOS_ATTEMPTS = 4;

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

static uint32_t readRecoveryFlags() {
	uint32_t rawFlags = 0;
	if (!api.system.flash.get(RECOVERY_BOOT_MARKER_FLASH_OFFSET, (uint8_t *)&rawFlags, sizeof(rawFlags))) {
		return 0;
	}

	// Erased flash defaults to 0xFFFFFFFF and should be treated as "no marker".
	if (rawFlags == 0xFFFFFFFFu) {
		return 0;
	}

	return rawFlags & RECOVERY_FLAG_VALID_MASK;
}

static void writeRecoveryFlags(uint32_t flags) {
	const uint32_t sanitizedFlags = flags & RECOVERY_FLAG_VALID_MASK;
	api.system.flash.set(RECOVERY_BOOT_MARKER_FLASH_OFFSET, (uint8_t *)&sanitizedFlags, sizeof(sanitizedFlags));
}

static void markRecoveryFlag(uint32_t flag) {
	uint32_t flags = readRecoveryFlags();
	if ((flags & flag) != 0) {
		return;
	}
	flags |= flag;
	writeRecoveryFlags(flags);
}

static void rebootForRecovery() {
	markRecoveryFlag(RECOVERY_FLAG_REBOOT_RECOVERY);
	api.system.reboot();
}

static bool shouldClearRecoveryMarkerThisBoot() {
	const uint32_t resetReason = NRF_POWER->RESETREAS;
	const uint32_t softwareResetMask = POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_DOG_Msk | POWER_RESETREAS_LOCKUP_Msk;
	return (resetReason & softwareResetMask) == 0;
}

static void maybeShowRecoveryBootStrobe() {
	const uint32_t flags = readRecoveryFlags();
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

	if ((flags & RECOVERY_FLAG_SOFT_RECOVERY) != 0) {
		delay(120);
		digitalWrite(LED_PIN, HIGH);
		delay(180);
		digitalWrite(LED_PIN, LOW);
	}

	if (shouldClearRecoveryMarkerThisBoot()) {
		writeRecoveryFlags(0);
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
	markRecoveryFlag(RECOVERY_FLAG_SOFT_RECOVERY);
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
	api.system.sleep.all((uint32_t)0xFFFFFFFF);

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