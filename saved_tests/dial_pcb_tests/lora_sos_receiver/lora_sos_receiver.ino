static const uint32_t LED_PIN = P1_03;

// Must match sender settings
static const uint32_t LORA_FREQ_HZ = 868000000;
static const uint16_t LORA_SF = 7;
static const uint16_t LORA_BW = 0;      // 0 = 125 kHz
static const uint16_t LORA_CR = 1;      // 1 = 4/6
static const uint16_t LORA_PREAMBLE = 8;

volatile bool rxPending = false;
uint8_t rxBuf[255];
uint16_t rxLen = 0;
int16_t rxRssi = 0;
int8_t rxSnr = 0;

static void blink(uint32_t onMs, uint32_t offMs) {
  digitalWrite(LED_PIN, HIGH);
  delay(onMs);
  digitalWrite(LED_PIN, LOW);
  delay(offMs);
}

void recv_cb(rui_lora_p2p_recv_t data) {
  if (data.Status == LORA_P2P_RXDONE) {
    uint16_t copyLen = data.BufferSize;
    if (copyLen > sizeof(rxBuf)) {
      copyLen = sizeof(rxBuf);
    }

    memcpy(rxBuf, data.Buffer, copyLen);
    rxLen = copyLen;
    rxRssi = data.Rssi;
    rxSnr = data.Snr;
    rxPending = true;
  } else if (data.Status == LORA_P2P_RXTIMEOUT) {
    // Keep receiver alive in long-running tests.
    api.lora.precv(65534);
  } else if (data.Status == LORA_P2P_RXERROR) {
    Serial.println("RX CRC error");
    api.lora.precv(65534);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(1500);
  Serial.println("RAK4631 LoRa P2P SOS receiver");

  // Ensure LoRa P2P mode (0). If changed, reboot to apply.
  if (api.lora.nwm.get() != 0) {
    Serial.println("Switching to LoRa P2P mode and rebooting...");
    api.lora.nwm.set();
    api.system.reboot();
  }

  bool ok = true;
  ok &= api.lora.pfreq.set(LORA_FREQ_HZ);
  ok &= api.lora.psf.set(LORA_SF);
  ok &= api.lora.pbw.set(LORA_BW);
  ok &= api.lora.pcr.set(LORA_CR);
  ok &= api.lora.ppl.set(LORA_PREAMBLE);

  api.lora.registerPRecvCallback(recv_cb);
  ok &= api.lora.precv(65534); // continuous receive

  if (!ok) {
    Serial.println("LoRa P2P RX config failed");
    while (true) {
      blink(80, 80);
    }
  }

  Serial.print("Listening at ");
  Serial.print(LORA_FREQ_HZ);
  Serial.println(" Hz");
}

void loop() {
  if (!rxPending) {
    return;
  }

  noInterrupts();
  uint16_t len = rxLen;
  int16_t rssi = rxRssi;
  int8_t snr = rxSnr;
  uint8_t local[255];
  memcpy(local, rxBuf, len);
  rxPending = false;
  interrupts();

  Serial.print("RX ");
  Serial.print(len);
  Serial.print(" B, RSSI=");
  Serial.print(rssi);
  Serial.print(", SNR=");
  Serial.println(snr);

  Serial.print("HEX: ");
  for (uint16_t i = 0; i < len; i++) {
    if (local[i] < 16) Serial.print('0');
    Serial.print(local[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("ASCII: ");
  for (uint16_t i = 0; i < len; i++) {
    char c = (char)local[i];
    if (c >= 32 && c <= 126) {
      Serial.print(c);
    } else {
      Serial.print('.');
    }
  }
  Serial.println();

  blink(120, 40);
}
