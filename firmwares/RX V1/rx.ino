#include "Protocol.h"

static const uint32_t LED_PIN = P1_03;

static const uint32_t LORA_FREQ_HZ = 868000000;
static const uint16_t LORA_SF = 7;
static const uint16_t LORA_BW = 0;
static const uint16_t LORA_CR = 1;
static const uint16_t LORA_PREAMBLE = 8;
static const int16_t LORA_TX_POWER = 14;

static const uint32_t RX_WINDOW_MS = 65534;
static const uint32_t CALL_DELAY_MS = 1000;
static const uint8_t ACK_SUCCESS_PERCENT = 80;
static const uint8_t CALL_SUCCESS_PERCENT = 95;

enum ReplyState {
	REPLY_IDLE = 0,
	REPLY_SENDING_ACK,
	REPLY_WAITING_FOR_CALL,
	REPLY_SENDING_CALL,
};

volatile bool gRxPending = false;
volatile bool gRxTimedOut = false;
volatile bool gRxError = false;
volatile bool gTxDone = false;

uint8_t gRxBuffer[255];
uint16_t gRxLength = 0;
ReplyState gReplyState = REPLY_IDLE;
uint32_t gReplyDeviceId = 0;
uint32_t gReplySequence = 0;
uint32_t gCallDueAtMs = 0;

static void blink(uint32_t onMs, uint32_t offMs) {
	digitalWrite(LED_PIN, HIGH);
	delay(onMs);
	digitalWrite(LED_PIN, LOW);
	delay(offMs);
}

static bool shouldSimulateSuccess(uint8_t percent) {
	if (percent >= 100) {
		return true;
	}

	return (uint8_t)random(0, 100) < percent;
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

static bool restartReceive() {
	gRxPending = false;
	gRxTimedOut = false;
	gRxError = false;
	return api.lora.precv(RX_WINDOW_MS);
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

static bool sendReply(PacketType type, uint32_t deviceId, uint32_t sequence) {
	ProtocolMessage message;
	message.type = type;
	message.deviceId = deviceId;
	message.sequence = sequence;

	char payload[PROTOCOL_MAX_PAYLOAD];
	if (!protocolFormatMessage(message, payload, sizeof(payload))) {
		return false;
	}

	api.lora.precv(0);  // stop RX before transmitting
	gTxDone = false;
	Serial.print("TX ");
	Serial.println(payload);
	return api.lora.psend(strlen(payload), (uint8_t *)payload);
}

static bool configureLoRaP2P() {
	if (api.lora.nwm.get() != 0) {
		Serial.println("Switching to LoRa P2P mode and rebooting...");
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

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	Serial.begin(115200);
	delay(300);
	randomSeed(millis());

	Serial.println("RX firmware boot");
	Serial.println("Randomized ACK/CALL test responder");

	if (!configureLoRaP2P()) {
		Serial.println("LoRa P2P RX config failed");
		while (true) {
			blink(80, 80);
		}
	}

	Serial.println("RX firmware ready");
}

void loop() {
	ProtocolMessage message;

	if (gRxTimedOut || gRxError) {
		gRxTimedOut = false;
		gRxError = false;
		restartReceive();
	}

	if (takeReceivedMessage(message)) {
		Serial.print("RX ");
		Serial.print(protocolTypeName(message.type));
		Serial.print(" device=");
		Serial.print(message.deviceId);
		Serial.print(" seq=");
		Serial.println(message.sequence);

		blink(60, 20);

		if (message.type == PACKET_SOS && gReplyState == REPLY_IDLE) {
			gReplyDeviceId = message.deviceId;
			gReplySequence = message.sequence;

			if (shouldSimulateSuccess(ACK_SUCCESS_PERCENT) && sendReply(PACKET_ACK, gReplyDeviceId, gReplySequence)) {
				gReplyState = REPLY_SENDING_ACK;
			} else {
				Serial.println("ACK intentionally skipped or failed");
				restartReceive();
			}
		} else {
			restartReceive();
		}
	}

	switch (gReplyState) {
	case REPLY_IDLE:
		break;

	case REPLY_SENDING_ACK:
		if (gTxDone) {
			gTxDone = false;
			gCallDueAtMs = millis() + CALL_DELAY_MS;
			gReplyState = REPLY_WAITING_FOR_CALL;
		}
		break;

	case REPLY_WAITING_FOR_CALL:
		if (millis() >= gCallDueAtMs) {
			if (shouldSimulateSuccess(CALL_SUCCESS_PERCENT) && sendReply(PACKET_CALL, gReplyDeviceId, gReplySequence)) {
				gReplyState = REPLY_SENDING_CALL;
			} else {
				Serial.println("CALL intentionally skipped or failed");
				gReplyState = REPLY_IDLE;
				restartReceive();
			}
		}
		break;

	case REPLY_SENDING_CALL:
		if (gTxDone) {
			gTxDone = false;
			gReplyState = REPLY_IDLE;
			restartReceive();
		}
		break;
	}

	delay(5);
}