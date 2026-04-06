#include <nrf_gpio.h>

#include "Protocol.h"
#include "Feedback.h"

// Set to 1 to enable serial debug output.
// Keep 0 for production: Serial.begin() keeps the UART peripheral clocked
// and prevents the nRF52840 from reaching full deep sleep.
#define DEBUG_SERIAL 0

#if DEBUG_SERIAL
#define DBEGIN(baud) Serial.begin(baud)
#define DPRINT(x)    Serial.print(x)
#define DPRINTLN(x)  Serial.println(x)
#else
#define DBEGIN(baud)
#define DPRINT(x)
#define DPRINTLN(x)
#endif

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

static const uint32_t SHORT_PRESS_MS = 200;
static const uint32_t LONG_PRESS_MS = 3000;
static const uint32_t SEND_DONE_TIMEOUT_MS = 3000;
static const uint32_t ACK_TIMEOUT_MS = 3000;
static const uint32_t CALL_TIMEOUT_MS = 4000;
static const uint32_t RETRY_DELAY_MS = 1200;
static const uint8_t MAX_SOS_ATTEMPTS = 4;

enum TxState {
	TX_STATE_SLEEP = 0,
	TX_STATE_WAIT_LONG_PRESS,
	TX_STATE_SEND_END_CALL,
	TX_STATE_WAIT_END_CALL_TX_DONE,
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

TxState gState = TX_STATE_SLEEP;
uint32_t gStateStartedAtMs = 0;
uint32_t gButtonPressedAtMs = 0;
uint32_t gSequence = 0;
uint8_t gAttemptCount = 0;
bool gRequireReleaseBeforeNextTrigger = false;

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
	return api.lora.precv(timeoutMs);
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
		DPRINTLN("Switching to LoRa P2P mode and rebooting...");
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

	DPRINT("TX ");
	DPRINT(protocolTypeName(type));
	DPRINT(" seq=");
	DPRINTLN(gSequence);

	resetRadioFlags();
	return api.lora.psend(strlen(payload), (uint8_t *)payload);
}

static void scheduleRetry() {
	if (gAttemptCount < MAX_SOS_ATTEMPTS) {
		gState = TX_STATE_RETRY_DELAY;
		gStateStartedAtMs = millis();
		DPRINTLN("Retry scheduled");
	} else {
		gState = TX_STATE_FAILED;
		gStateStartedAtMs = millis();
		DPRINTLN("Max attempts reached");
	}
}

static void enterSleepState() {
	feedbackIdle(LED_PIN, MOTOR_PIN);

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
		gButtonPressedAtMs = millis();
		feedbackButtonDetected(LED_PIN, MOTOR_PIN);
		gState = TX_STATE_WAIT_LONG_PRESS;
		gStateStartedAtMs = millis();
		return;
	}

	// Stop the radio before sleeping — a lingering RX window keeps it powered.
	api.lora.precv(0);
	api.system.sleep.all((uint32_t)0xFFFFFFFF);

	// Re-arm long-press flow on any button-held wake condition.
	// This is more robust than relying only on wake callback counters.
	if (buttonPressedRaw()) {
		gButtonPressedAtMs = millis();
		feedbackButtonDetected(LED_PIN, MOTOR_PIN);
		gState = TX_STATE_WAIT_LONG_PRESS;
		gStateStartedAtMs = millis();
	}
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	pinMode(MOTOR_PIN, OUTPUT);
	nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
	feedbackIdle(LED_PIN, MOTOR_PIN);

	DBEGIN(115200);
#if DEBUG_SERIAL
	delay(300);
#endif
	DPRINTLN("TX firmware boot");

	api.system.lpm.set(1);
	api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
	api.system.sleep.registerWakeupCallback(wakeupCallback);

	if (!configureLoRaP2P()) {
		DPRINTLN("LoRa P2P config failed");
		while (true) {
			feedbackFailure(LED_PIN, MOTOR_PIN);
			delay(200);
		}
	}

	DPRINTLN("TX firmware ready");
}

void loop() {
	ProtocolMessage message;

	switch (gState) {
	case TX_STATE_SLEEP:
		enterSleepState();
		break;

	case TX_STATE_WAIT_LONG_PRESS:
		if (!buttonPressedRaw()) {
			const uint32_t pressDurationMs = millis() - gButtonPressedAtMs;
			if (pressDurationMs >= SHORT_PRESS_MS && pressDurationMs < LONG_PRESS_MS) {
				DPRINTLN("Short press released: send END_CALL");
				gSequence++;
				gState = TX_STATE_SEND_END_CALL;
				gStateStartedAtMs = millis();
			} else {
				DPRINTLN("Press too short, ignoring");
				gState = TX_STATE_SLEEP;
			}
			break;
		}

		feedbackLongPressProgress(LED_PIN, MOTOR_PIN, millis() - gButtonPressedAtMs, LONG_PRESS_MS);

		if (millis() - gButtonPressedAtMs >= LONG_PRESS_MS) {
			// Threshold reached: short dot confirmation, then start SOS.
			feedbackLongPressConfirmed(LED_PIN, MOTOR_PIN);
			gRequireReleaseBeforeNextTrigger = true;
			gSequence++;
			gAttemptCount = 0;
			gState = TX_STATE_SEND_SOS;
			gStateStartedAtMs = millis();
		}
		break;

	case TX_STATE_SEND_END_CALL:
		if (sendMessage(PACKET_END_CALL)) {
			gState = TX_STATE_WAIT_END_CALL_TX_DONE;
			gStateStartedAtMs = millis();
		} else {
			DPRINTLN("Failed to queue END_CALL packet");
			gState = TX_STATE_FAILED;
			gStateStartedAtMs = millis();
		}
		break;

	case TX_STATE_WAIT_END_CALL_TX_DONE:
		if (gTxDone) {
			gTxDone = false;
			gState = TX_STATE_SLEEP;
		} else if (millis() - gStateStartedAtMs >= SEND_DONE_TIMEOUT_MS) {
			DPRINTLN("END_CALL TX done timeout");
			gState = TX_STATE_FAILED;
			gStateStartedAtMs = millis();
		}
		break;

	case TX_STATE_SEND_SOS:
		gAttemptCount++;
		if (sendMessage(PACKET_SOS)) {
			gState = TX_STATE_WAIT_TX_DONE;
			gStateStartedAtMs = millis();
		} else {
			DPRINTLN("Failed to queue SOS packet");
			scheduleRetry();
		}
		break;

	case TX_STATE_WAIT_TX_DONE:
		if (gTxDone) {
			gTxDone = false;
			if (startReceiveWindow(ACK_TIMEOUT_MS)) {
				gState = TX_STATE_WAIT_ACK;
				gStateStartedAtMs = millis();
				DPRINTLN("Waiting for ACK");
			} else {
				DPRINTLN("Failed to enter ACK receive window");
				scheduleRetry();
			}
		} else if (millis() - gStateStartedAtMs >= SEND_DONE_TIMEOUT_MS) {
			DPRINTLN("TX done timeout");
			scheduleRetry();
		}
		break;

	case TX_STATE_WAIT_ACK:
		if (takeReceivedMessage(message)) {
			DPRINT("RX while waiting ACK: ");
			DPRINTLN(protocolTypeName(message.type));

			if (protocolMatchesResponse(message, PACKET_ACK, TX_DEVICE_ID, gSequence)) {
				feedbackAckReceived(LED_PIN);
				if (startReceiveWindow(CALL_TIMEOUT_MS)) {
					gState = TX_STATE_WAIT_CALL;
					gStateStartedAtMs = millis();
					DPRINTLN("ACK received, waiting for CALL");
				} else {
					DPRINTLN("Failed to enter CALL receive window");
					scheduleRetry();
				}
			} else {
				startReceiveWindow(ACK_TIMEOUT_MS);
			}
		} else if (radioWindowTimedOut()) {
			DPRINTLN("ACK timeout");
			scheduleRetry();
		}
		break;

	case TX_STATE_WAIT_CALL:
		if (takeReceivedMessage(message)) {
			DPRINT("RX while waiting CALL: ");
			DPRINTLN(protocolTypeName(message.type));

			if (protocolMatchesResponse(message, PACKET_CALL, TX_DEVICE_ID, gSequence)) {
				gState = TX_STATE_COMPLETE;
				gStateStartedAtMs = millis();
			} else {
				startReceiveWindow(CALL_TIMEOUT_MS);
			}
		} else if (radioWindowTimedOut()) {
			DPRINTLN("CALL timeout");
			gState = TX_STATE_COMPLETE_ACK_ONLY;
			gStateStartedAtMs = millis();
		}
		break;

	case TX_STATE_RETRY_DELAY:
		if (millis() - gStateStartedAtMs >= RETRY_DELAY_MS) {
			gState = TX_STATE_SEND_SOS;
			gStateStartedAtMs = millis();
		}
		break;

	case TX_STATE_COMPLETE:
		DPRINTLN("CALL confirmed");
		feedbackCallEstablished(LED_PIN, MOTOR_PIN);
		gState = TX_STATE_SLEEP;
		break;

	case TX_STATE_COMPLETE_ACK_ONLY:
		DPRINTLN("ACK received, CALL not confirmed within timeout");
		feedbackAckOnly(LED_PIN, MOTOR_PIN);
		gState = TX_STATE_SLEEP;
		break;

	case TX_STATE_FAILED:
		DPRINTLN("Request finished without confirmation");
		feedbackFailure(LED_PIN, MOTOR_PIN);
		gState = TX_STATE_SLEEP;
		break;
	}

	delay(5);
}