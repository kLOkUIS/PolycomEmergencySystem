static const uint32_t LED_PIN = P1_03;

// LoRa P2P settings (adjust for your region if needed)
static const uint32_t LORA_FREQ_HZ = 868000000;
static const uint16_t LORA_SF = 7;
static const uint16_t LORA_BW = 0;      // 0 = 125 kHz
static const uint16_t LORA_CR = 1;      // 1 = 4/6
static const uint16_t LORA_PREAMBLE = 8;
static const int16_t LORA_TX_POWER = 14;

static const uint32_t SOS_INTERVAL_MS = 5000;

unsigned long lastSendMs = 0;

void send_cb(void) {
	Serial.println("LoRa TX done");
}

static void blink(uint32_t onMs, uint32_t offMs) {
	digitalWrite(LED_PIN, HIGH);
	delay(onMs);
	digitalWrite(LED_PIN, LOW);
	delay(offMs);
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	Serial.begin(115200);
	delay(1500);

	Serial.println("RAK4631 LoRa P2P SOS sender");

	// Ensure LoRa P2P mode (0). If changed, reboot to apply.
	if (api.lora.nwm.get() != 0) {
		Serial.println("Switching to LoRa P2P mode and rebooting...");
		api.lora.nwm.set();
		api.system.reboot();
	}

	api.lora.registerPSendCallback(send_cb);

	bool ok = true;
	ok &= api.lora.pfreq.set(LORA_FREQ_HZ);
	ok &= api.lora.psf.set(LORA_SF);
	ok &= api.lora.pbw.set(LORA_BW);
	ok &= api.lora.pcr.set(LORA_CR);
	ok &= api.lora.ppl.set(LORA_PREAMBLE);
	ok &= api.lora.ptp.set(LORA_TX_POWER);

	if (!ok) {
		Serial.println("LoRa P2P config failed");
		// Fast blink forever on config error
		while (true) {
			blink(80, 80);
		}
	}

	Serial.print("Freq: ");
	Serial.println(LORA_FREQ_HZ);
	Serial.println("LoRa P2P ready");
}

void loop() {
	if (millis() - lastSendMs < SOS_INTERVAL_MS) {
		return;
	}
	lastSendMs = millis();

	static const uint8_t msg[] = "SOS";
	bool sent = api.lora.psend(sizeof(msg) - 1, (uint8_t *)msg);

	if (sent) {
		Serial.println("Sent: SOS");
		blink(120, 40);
		blink(120, 40);
		blink(120, 40);
	} else {
		Serial.println("Send failed");
		blink(400, 100);
	}
}
