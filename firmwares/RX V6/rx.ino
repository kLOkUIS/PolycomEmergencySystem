				#include <nrf_gpio.h>

				#include "Protocol.h"

static const uint32_t LED_RX_PIN = P0_24;
static const uint32_t LED_DIAL_CTL_PIN = P0_31;
static const uint32_t KEY1_CTL_PIN = P1_01;
static const uint32_t KEY2_CTL_PIN = P1_02;
static const uint32_t KEY3_CTL_PIN = P1_03;
static const uint32_t KEY4_CTL_PIN = P1_04;
static const uint32_t KEY5_CTL_PIN = P0_05;
static const uint32_t KEY6_CTL_PIN = P0_04;
static const uint32_t CALL_SENSE_PIN = P0_25;

static const uint32_t LORA_FREQ_HZ = 868000000;
static const uint16_t LORA_SF = 7;
static const uint16_t LORA_BW = 0;
static const uint16_t LORA_CR = 1;
static const uint16_t LORA_PREAMBLE = 8;
static const int16_t LORA_TX_POWER = 14;
static const uint32_t RX_WINDOW_MS = 65534;
static const uint32_t RX_IDLE_REFRESH_MS = 60000;

static const uint32_t KEY_PRESS_MS = 220;
static const uint32_t KEY_REPEAT_GAP_MS = 180;
static const uint32_t KEY_STEP_GAP_MS = 350;
static const uint32_t CALL_SENSE_DEBOUNCE_MS = 180;
// Measured incoming ring cadence: ON ~879 ms, OFF ~4121 ms.
// Hold for a full OFF window (+margin) so CALL_TOGGLE works any time during ringing.
static const uint32_t CALL_ACTIVE_HOLD_MS = 4600;
// Treat only short active bursts as "ringing". Longer active periods are real calls.
static const uint32_t CALL_ACTIVE_SHORT_BURST_MAX_MS = 1700;
static const uint32_t RX_PACKET_PULSE_MS = 140;
static const uint32_t TX_DONE_TIMEOUT_MS = 3500;
static const uint32_t ACTION_PLAN_TIMEOUT_MS = 20000;
static const uint32_t SESSION_AWAIT_CALL_SENSE_TIMEOUT_MS = 45000;

// RX watchdog tuning (always active).
// Stage 1: restart RX window.
// Stage 2: full LoRa reconfigure after repeated restart failures.
// Stage 3: reboot after too many failures.
#ifndef RX_WD_LISTEN_STUCK_MS
#define RX_WD_LISTEN_STUCK_MS 420000UL
#endif

#ifndef RX_WD_STUCK_RETRY_GAP_MS
#define RX_WD_STUCK_RETRY_GAP_MS 3000UL
#endif

#ifndef RX_WD_FULL_RECOVERY_AFTER_FAILURES
#define RX_WD_FULL_RECOVERY_AFTER_FAILURES 2
#endif

#ifndef RX_WD_MAX_RESTART_FAILURES
#define RX_WD_MAX_RESTART_FAILURES 5
#endif

static const uint32_t RECOVERY_FLAG_SOFT_RECOVERY = (1u << 0);
static const uint32_t RECOVERY_FLAG_REBOOT_RECOVERY = (1u << 1);
static const uint32_t RECOVERY_BOOT_MARKER_FLASH_OFFSET = 0;
static const uint32_t RECOVERY_FLAG_VALID_MASK = RECOVERY_FLAG_SOFT_RECOVERY | RECOVERY_FLAG_REBOOT_RECOVERY;

struct KeyAction {
	uint32_t pin;
	uint8_t presses;
};

static const KeyAction PLACE_CALL_ACTIONS[] = {
	{KEY1_CTL_PIN, 3},
	{KEY2_CTL_PIN, 1},
	{KEY3_CTL_PIN, 1},
};

static const KeyAction CALL_TOGGLE_ACTIONS[] = {
	{KEY3_CTL_PIN, 1},
	{KEY1_CTL_PIN, 3},
};

enum ActionPlan {
	ACTION_PLAN_NONE = 0,
	ACTION_PLAN_PLACE_CALL,
	ACTION_PLAN_CALL_TOGGLE,
};

enum ActionPhase {
	ACTION_PHASE_IDLE = 0,
	ACTION_PHASE_PRESS_HIGH,
	ACTION_PHASE_PRESS_LOW,
};

volatile bool gRxPending = false;
volatile bool gRxTimedOut = false;
volatile bool gRxError = false;
volatile bool gTxDone = false;

uint8_t gRxBuffer[255];
uint16_t gRxLength = 0;

PacketType gPendingTxType = PACKET_INVALID;
uint32_t gPendingTxDeviceId = 0;
uint32_t gPendingTxSequence = 0;

bool gSessionAwaitingCallSense = false;
uint32_t gSessionDeviceId = 0;
uint32_t gSessionSequence = 0;
uint32_t gSessionStartedAtMs = 0;

bool gCallSenseRawHigh = true;
bool gCallSenseStableHigh = true;
uint32_t gCallSenseRawChangedAtMs = 0;
uint32_t gCallActiveHoldUntilMs = 0;
uint32_t gCallSenseActiveStartedAtMs = 0;

ActionPlan gActionPlan = ACTION_PLAN_NONE;
const KeyAction *gActionTable = nullptr;
uint8_t gActionCount = 0;
uint8_t gActionIndex = 0;
uint8_t gActionPressCount = 0;
ActionPhase gActionPhase = ACTION_PHASE_IDLE;
uint32_t gActionDueAtMs = 0;
uint32_t gActionStartedAtMs = 0;
bool gActionLedOn = false;
uint32_t gRxPacketPulseUntilMs = 0;
uint32_t gPendingTxStartedAtMs = 0;
uint32_t gLastRadioEventAtMs = 0;
uint32_t gLastRadioCallbackAtMs = 0;
uint32_t gLastStuckRecoveryAttemptAtMs = 0;
uint32_t gLastFullRecoveryAtMs = 0;
uint8_t gRxRestartFailureCount = 0;

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
			digitalWrite(LED_RX_PIN, HIGH);
			delay(45);
			digitalWrite(LED_RX_PIN, LOW);
			delay(45);
		}
	}

	if ((flags & RECOVERY_FLAG_SOFT_RECOVERY) != 0) {
		delay(120);
		digitalWrite(LED_RX_PIN, HIGH);
		delay(180);
		digitalWrite(LED_RX_PIN, LOW);
	}

	if (shouldClearRecoveryMarkerThisBoot()) {
		writeRecoveryFlags(0);
	}
}

static bool elapsedAtLeast(uint32_t sinceMs, uint32_t durationMs) {
	return (uint32_t)(millis() - sinceMs) >= durationMs;
}

static void markRadioEvent() {
	gLastRadioEventAtMs = millis();
}

static void markRadioCallbackEvent() {
	const uint32_t nowMs = millis();
	gLastRadioEventAtMs = nowMs;
	gLastRadioCallbackAtMs = nowMs;
}

static void clearSessionState() {
	gSessionAwaitingCallSense = false;
	gSessionDeviceId = 0;
	gSessionSequence = 0;
	gSessionStartedAtMs = 0;
}

void sendCallback(void) {
	gTxDone = true;
	markRadioCallbackEvent();
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
		markRadioCallbackEvent();
	} else if (data.Status == LORA_P2P_RXTIMEOUT) {
		gRxTimedOut = true;
		markRadioCallbackEvent();
	} else if (data.Status == LORA_P2P_RXERROR) {
		gRxError = true;
		markRadioCallbackEvent();
	}
}

static void allKeysLow() {
	pinMode(KEY1_CTL_PIN, OUTPUT);
	digitalWrite(KEY1_CTL_PIN, LOW);
	pinMode(KEY2_CTL_PIN, OUTPUT);
	digitalWrite(KEY2_CTL_PIN, LOW);
	pinMode(KEY3_CTL_PIN, OUTPUT);
	digitalWrite(KEY3_CTL_PIN, LOW);
	pinMode(KEY4_CTL_PIN, OUTPUT);
	digitalWrite(KEY4_CTL_PIN, LOW);
	pinMode(KEY5_CTL_PIN, OUTPUT);
	digitalWrite(KEY5_CTL_PIN, LOW);
	pinMode(KEY6_CTL_PIN, OUTPUT);
	digitalWrite(KEY6_CTL_PIN, LOW);
}

static bool restartReceive() {
	gRxPending = false;
	gRxTimedOut = false;
	gRxError = false;
	if (!api.lora.precv(RX_WINDOW_MS)) {
		return false;
	}
	markRadioEvent();
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
	ok &= restartReceive();
	return ok;
}

static bool refreshReceiveWindowIdleWatchdog() {
	api.lora.precv(0);
	if (!restartReceive()) {
		return false;
	}

	gRxRestartFailureCount = 0;
	markRadioEvent();
	gLastRadioCallbackAtMs = millis();
	return true;
}

static bool restartReceiveWatched() {
	if (restartReceive()) {
		gRxRestartFailureCount = 0;
		return true;
	}

	markRecoveryFlag(RECOVERY_FLAG_SOFT_RECOVERY);

	if (gRxRestartFailureCount < 255) {
		gRxRestartFailureCount++;
	}

	if (gRxRestartFailureCount >= RX_WD_FULL_RECOVERY_AFTER_FAILURES) {
		markRecoveryFlag(RECOVERY_FLAG_SOFT_RECOVERY);
		// Try a full LoRa stack reconfigure before escalating to reboot.
		if ((uint32_t)(millis() - gLastFullRecoveryAtMs) >= RX_WD_STUCK_RETRY_GAP_MS) {
			gLastFullRecoveryAtMs = millis();
			api.lora.precv(0);
			if (configureLoRaP2P()) {
				gRxRestartFailureCount = 0;
				markRadioEvent();
				gLastRadioCallbackAtMs = millis();
				return true;
			}
		}
	}

	if (gRxRestartFailureCount >= RX_WD_MAX_RESTART_FAILURES) {
		rebootForRecovery();
	}

	return false;
}

static bool queueReply(PacketType type, uint32_t deviceId, uint32_t sequence) {
	if (gPendingTxType != PACKET_INVALID) {
		return false;
	}

	ProtocolMessage message;
	message.type = type;
	message.deviceId = deviceId;
	message.sequence = sequence;

	char payload[PROTOCOL_MAX_PAYLOAD];
	if (!protocolFormatMessage(message, payload, sizeof(payload))) {
		return false;
	}

	api.lora.precv(0);
	gTxDone = false;
	gPendingTxType = type;
	gPendingTxDeviceId = deviceId;
	gPendingTxSequence = sequence;
	gPendingTxStartedAtMs = millis();
	if (!api.lora.psend(strlen(payload), (uint8_t *)payload)) {
		gPendingTxType = PACKET_INVALID;
		gPendingTxStartedAtMs = 0;
		return restartReceiveWatched();
	}
	markRadioEvent();

	return true;
}

static bool callSenseActive() {
	return !gCallSenseStableHigh;
}

static bool callControlAllowed() {
	if (callSenseActive()) {
		return true;
	}

	return (long)(millis() - gCallActiveHoldUntilMs) < 0;
}

static void updateCallSense() {
	const bool rawHigh = digitalRead(CALL_SENSE_PIN) == HIGH;
	const uint32_t nowMs = millis();
	const bool wasActive = callSenseActive();

	if (rawHigh != gCallSenseRawHigh) {
		gCallSenseRawHigh = rawHigh;
		gCallSenseRawChangedAtMs = nowMs;
	}

	if ((nowMs - gCallSenseRawChangedAtMs) >= CALL_SENSE_DEBOUNCE_MS) {
		gCallSenseStableHigh = gCallSenseRawHigh;
	}

	const bool isActive = callSenseActive();
	if (!wasActive && isActive) {
		gCallSenseActiveStartedAtMs = nowMs;
	}

	if (wasActive && !isActive) {
		const uint32_t activeDurationMs = nowMs - gCallSenseActiveStartedAtMs;
		if (activeDurationMs <= CALL_ACTIVE_SHORT_BURST_MAX_MS) {
			// Keep call-control commands valid across ringing blink gaps.
			gCallActiveHoldUntilMs = nowMs + CALL_ACTIVE_HOLD_MS;
		} else {
			// Long active period likely means real call session; avoid post-hangup reopen window.
			gCallActiveHoldUntilMs = 0;
		}
	}

	digitalWrite(LED_RX_PIN, isActive ? HIGH : LOW);
}

static void clearActionState() {
	gActionPlan = ACTION_PLAN_NONE;
	gActionTable = nullptr;
	gActionCount = 0;
	gActionIndex = 0;
	gActionPressCount = 0;
	gActionPhase = ACTION_PHASE_IDLE;
	gActionDueAtMs = 0;
	gActionStartedAtMs = 0;
	gActionLedOn = false;
	allKeysLow();
}

static bool actionBusy() {
	return gActionPlan != ACTION_PLAN_NONE;
}

static void startAction(ActionPlan plan, const KeyAction *actions, uint8_t actionCount) {
	clearActionState();
	gActionPlan = plan;
	gActionTable = actions;
	gActionCount = actionCount;
	gActionPhase = ACTION_PHASE_PRESS_HIGH;
	gActionDueAtMs = millis();
	gActionStartedAtMs = gActionDueAtMs;
}

static void updateActionPlan() {
	if (!actionBusy()) {
		return;
	}

	if (elapsedAtLeast(gActionStartedAtMs, ACTION_PLAN_TIMEOUT_MS)) {
		clearActionState();
		return;
	}

	const uint32_t nowMs = millis();
	if ((long)(nowMs - gActionDueAtMs) < 0) {
		return;
	}

	if (gActionIndex >= gActionCount) {
		clearActionState();
		return;
	}

	const KeyAction &action = gActionTable[gActionIndex];

	switch (gActionPhase) {
	case ACTION_PHASE_PRESS_HIGH:
		gActionLedOn = true;
		digitalWrite(action.pin, HIGH);
		gActionPhase = ACTION_PHASE_PRESS_LOW;
		gActionDueAtMs = nowMs + KEY_PRESS_MS;
		break;

	case ACTION_PHASE_PRESS_LOW:
		digitalWrite(action.pin, LOW);
		gActionLedOn = false;
		gActionPressCount++;
		if (gActionPressCount < action.presses) {
			gActionPhase = ACTION_PHASE_PRESS_HIGH;
			gActionDueAtMs = nowMs + KEY_REPEAT_GAP_MS;
		} else {
			gActionPressCount = 0;
			gActionIndex++;
			if (gActionIndex >= gActionCount) {
				clearActionState();
			} else {
				gActionPhase = ACTION_PHASE_PRESS_HIGH;
				gActionDueAtMs = nowMs + KEY_STEP_GAP_MS;
			}
		}
		break;

	default:
		clearActionState();
		break;
	}
}

static void updateDialCtlLed() {
	const uint32_t nowMs = millis();
	const bool rxPulseOn = (long)(nowMs - gRxPacketPulseUntilMs) < 0;
	digitalWrite(LED_DIAL_CTL_PIN, (gActionLedOn || rxPulseOn) ? HIGH : LOW);
}

static bool currentSessionMatches(uint32_t deviceId, uint32_t sequence) {
	return gSessionAwaitingCallSense && gSessionDeviceId == deviceId && gSessionSequence == sequence;
}

static void updateSessionTimeout() {
	if (!gSessionAwaitingCallSense) {
		return;
	}

	if (elapsedAtLeast(gSessionStartedAtMs, SESSION_AWAIT_CALL_SENSE_TIMEOUT_MS)) {
		clearSessionState();
	}
}

static void handleSosMessage(const ProtocolMessage &message) {
	if (currentSessionMatches(message.deviceId, message.sequence)) {
		queueReply(PACKET_ACK, message.deviceId, message.sequence);
		return;
	}

	if (actionBusy()) {
		return;
	}

	// If CALL_SENSE never arrived, allow a new SOS to replace the stale waiting session.
	if (gSessionAwaitingCallSense) {
		clearSessionState();
	}

	gSessionDeviceId = message.deviceId;
	gSessionSequence = message.sequence;
	gSessionAwaitingCallSense = true;
	gSessionStartedAtMs = millis();
	if (queueReply(PACKET_ACK, message.deviceId, message.sequence)) {
		startAction(ACTION_PLAN_PLACE_CALL, PLACE_CALL_ACTIONS, sizeof(PLACE_CALL_ACTIONS) / sizeof(PLACE_CALL_ACTIONS[0]));
	} else {
		clearSessionState();
	}
}

static void handleCallToggleMessage(const ProtocolMessage &message) {
	(void)message;
	if (!callControlAllowed()) {
		return;
	}

	if (actionBusy()) {
		return;
	}

	// User manually toggled call state; don't keep stale pending session.
	clearSessionState();

	startAction(
		ACTION_PLAN_CALL_TOGGLE,
		CALL_TOGGLE_ACTIONS,
		sizeof(CALL_TOGGLE_ACTIONS) / sizeof(CALL_TOGGLE_ACTIONS[0]));
}

static void serviceReceivedMessage(const ProtocolMessage &message) {
	gRxPacketPulseUntilMs = millis() + RX_PACKET_PULSE_MS;

	switch (message.type) {
	case PACKET_SOS:
		handleSosMessage(message);
		break;

	case PACKET_CALL_TOGGLE:
		handleCallToggleMessage(message);
		break;

	default:
		break;
	}
}

static void updatePendingTransmit() {
	if (gPendingTxType != PACKET_INVALID && elapsedAtLeast(gPendingTxStartedAtMs, TX_DONE_TIMEOUT_MS)) {
		gPendingTxType = PACKET_INVALID;
		gPendingTxStartedAtMs = 0;
		restartReceiveWatched();
		return;
	}

	if (gPendingTxType == PACKET_INVALID || !gTxDone) {
		return;
	}

	gTxDone = false;
	gPendingTxType = PACKET_INVALID;
	gPendingTxStartedAtMs = 0;
	restartReceiveWatched();
}

static void maybeSendCallConfirmation() {
	if (!gSessionAwaitingCallSense) {
		return;
	}

	if (actionBusy()) {
		return;
	}

	if (!callSenseActive()) {
		return;
	}

	if (queueReply(PACKET_CALL, gSessionDeviceId, gSessionSequence)) {
		clearSessionState();
	}
}

void setup() {
	pinMode(LED_RX_PIN, OUTPUT);
	digitalWrite(LED_RX_PIN, LOW);
	pinMode(LED_DIAL_CTL_PIN, OUTPUT);
	digitalWrite(LED_DIAL_CTL_PIN, LOW);
	maybeShowRecoveryBootStrobe();
	pinMode(CALL_SENSE_PIN, INPUT);
	allKeysLow();

	// Keep RX fully awake for deterministic radio callback handling.
	api.system.lpm.set(0);
	gCallSenseRawHigh = digitalRead(CALL_SENSE_PIN) == HIGH;
	gCallSenseStableHigh = gCallSenseRawHigh;
	gCallSenseRawChangedAtMs = millis();
	gCallActiveHoldUntilMs = 0;
	gCallSenseActiveStartedAtMs = gCallSenseStableHigh ? 0 : millis();
	clearSessionState();
	markRadioEvent();
	gLastRadioCallbackAtMs = millis();
	gLastStuckRecoveryAttemptAtMs = 0;
	gLastFullRecoveryAtMs = 0;
	gRxRestartFailureCount = 0;

	if (!configureLoRaP2P()) {
		while (true) {
			digitalWrite(LED_RX_PIN, HIGH);
			delay(80);
			digitalWrite(LED_RX_PIN, LOW);
			delay(80);
		}
	}

}

void loop() {
	ProtocolMessage message;

	updateCallSense();
	updatePendingTransmit();
	updateActionPlan();
	updateDialCtlLed();
	maybeSendCallConfirmation();
	updateSessionTimeout();

	if (gRxTimedOut || gRxError) {
		gRxTimedOut = false;
		gRxError = false;
		restartReceiveWatched();
	}

	if (gPendingTxType == PACKET_INVALID && elapsedAtLeast(gLastRadioEventAtMs, RX_IDLE_REFRESH_MS)) {
		if (!refreshReceiveWindowIdleWatchdog()) {
			restartReceiveWatched();
		}
	}

	if (elapsedAtLeast(gLastRadioCallbackAtMs, RX_WD_LISTEN_STUCK_MS)) {
		if ((uint32_t)(millis() - gLastStuckRecoveryAttemptAtMs) >= RX_WD_STUCK_RETRY_GAP_MS) {
			gLastStuckRecoveryAttemptAtMs = millis();
			restartReceiveWatched();
		}
	}

	if (takeReceivedMessage(message)) {
		serviceReceivedMessage(message);
	}

	delay(2);
}