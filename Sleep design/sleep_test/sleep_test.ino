#include <nrf_gpio.h>
extern "C" {
#include <radio.h>
}

static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t LED_PIN    = P1_03;
static const uint32_t MOTOR_PIN  = P1_04;

#ifndef TEST_MODE_LPM
#define TEST_MODE_LPM 1
#endif

// TX diagnostic mode (selected at compile time):
// 0 = disabled (baseline button wake only)
// 1 = configure LoRa only, no send
// 2 = send one packet, then return to sleep immediately
// 3 = send one packet, wait for send callback, then sleep
// 4 = send one packet, wait for send callback, then settle briefly
// 5 = send one packet, wait for send callback, then settle longer
// 6 = send one packet, wait for callback, settle longer, then precv(0)
// 7 = send one packet, wait for callback, settle longer, precv(0), delay, Radio.Sleep()
#ifndef TEST_TX_DIAG_MODE
#define TEST_TX_DIAG_MODE 7
#endif

#ifndef TEST_LORA_SHUTDOWN_DELAY_MS
#define TEST_LORA_SHUTDOWN_DELAY_MS 10
#endif

#ifndef TEST_TX_DONE_TIMEOUT_MS
#define TEST_TX_DONE_TIMEOUT_MS 3000
#endif

#ifndef TEST_TX_SETTLE_SHORT_MS
#define TEST_TX_SETTLE_SHORT_MS 100
#endif

#ifndef TEST_TX_SETTLE_LONG_MS
#define TEST_TX_SETTLE_LONG_MS 5000
#endif

static const uint32_t LORA_FREQ_HZ  = 868000000;
static const uint16_t LORA_SF       = 7;
static const uint16_t LORA_BW       = 0;
static const uint16_t LORA_CR       = 1;
static const uint16_t LORA_PREAMBLE = 8;
static const int16_t  LORA_TX_POWER = 14;

static uint8_t gTxPayload[] = "sleep-tx";

volatile bool     gButtonWakePending = false;
volatile uint32_t gButtonWakeCount   = 0;
volatile uint32_t gOtherWakeCount    = 0;
volatile bool     gTxDone            = false;
volatile uint32_t gTxDoneCount       = 0;
static   uint32_t gDiagAttemptCount  = 0;
static   uint32_t gDiagInitFailCount = 0;
static   uint32_t gDiagSendFailCount = 0;
static   uint32_t gDiagTimeoutCount  = 0;
static   uint32_t gDiagStepFailCount = 0;
static   bool     gDiagRan           = false;

enum TxDiagMode {
  TX_DIAG_DISABLED = 0,
  TX_DIAG_INIT_ONLY = 1,
  TX_DIAG_SEND_IMMEDIATE_SLEEP = 2,
  TX_DIAG_SEND_WAIT_DONE = 3,
  TX_DIAG_SEND_WAIT_DONE_SETTLE_SHORT = 4,
  TX_DIAG_SEND_WAIT_DONE_SETTLE_LONG = 5,
  TX_DIAG_SEND_WAIT_DONE_SETTLE_LONG_PRECV = 6,
  TX_DIAG_SEND_WAIT_DONE_SETTLE_LONG_PRECV_SLEEP = 7,
};

void buttonWakeupCallback(void) {
  gButtonWakePending = true;
}

void sendCallback(void) {
  gTxDone = true;
  gTxDoneCount++;
}

static bool configureLoRaP2P(void) {
  if (api.lora.nwm.get() != 0) {
    api.lora.nwm.set();
    api.system.reboot();
    return false;
  }

  api.lora.registerPSendCallback(sendCallback);

  bool ok = true;
  ok &= api.lora.pfreq.set(LORA_FREQ_HZ);
  ok &= api.lora.psf.set(LORA_SF);
  ok &= api.lora.pbw.set(LORA_BW);
  ok &= api.lora.pcr.set(LORA_CR);
  ok &= api.lora.ppl.set(LORA_PREAMBLE);
  ok &= api.lora.ptp.set(LORA_TX_POWER);
  return ok;
}

static bool sendTestPacket(void) {
  gTxDone = false;
  if (!api.lora.psend(sizeof(gTxPayload) - 1, gTxPayload)) {
    gDiagSendFailCount++;
    return false;
  }

  return true;
}

static bool waitForSendDone(void) {
  if (gTxDone) {
    return true;
  }

  const uint32_t startedAtMs = millis();
  while (!gTxDone) {
    if ((uint32_t)(millis() - startedAtMs) >= TEST_TX_DONE_TIMEOUT_MS) {
      gDiagTimeoutCount++;
      return false;
    }
    delay(1);
  }

  return true;
}

static bool runTxDiagStep(void) {
  const uint32_t mode = (uint32_t)TEST_TX_DIAG_MODE;

  if (mode == TX_DIAG_DISABLED) {
    return true;
  }

  if (!configureLoRaP2P()) {
    gDiagInitFailCount++;
    return false;
  }

  if (mode == TX_DIAG_INIT_ONLY) {
    return true;
  }

  if (!sendTestPacket()) {
    return false;
  }

  if (mode == TX_DIAG_SEND_IMMEDIATE_SLEEP) {
    return true;
  }

  if (!waitForSendDone()) {
    return false;
  }

  if (mode == TX_DIAG_SEND_WAIT_DONE) {
    return true;
  }

  if (mode == TX_DIAG_SEND_WAIT_DONE_SETTLE_SHORT) {
    delay(TEST_TX_SETTLE_SHORT_MS);
    return true;
  }

  if (mode == TX_DIAG_SEND_WAIT_DONE_SETTLE_LONG) {
    delay(TEST_TX_SETTLE_LONG_MS);
    return true;
  }

  if (mode == TX_DIAG_SEND_WAIT_DONE_SETTLE_LONG_PRECV) {
    delay(TEST_TX_SETTLE_LONG_MS);
    api.lora.precv(0);
    return true;
  }

  if (mode == TX_DIAG_SEND_WAIT_DONE_SETTLE_LONG_PRECV_SLEEP) {
    delay(TEST_TX_SETTLE_LONG_MS);
    api.lora.precv(0);
    delay(TEST_LORA_SHUTDOWN_DELAY_MS);
    Radio.Sleep();
    return true;
  }

  gDiagStepFailCount++;
  return false;
}

void setup() {
  pinMode(LED_PIN,   OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(LED_PIN,   LOW);
  digitalWrite(MOTOR_PIN, LOW);

#if TEST_MODE_LPM
  api.system.lpm.set(1);
#endif

  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
  (void)api.system.sleep.registerWakeupCallback(buttonWakeupCallback);

  gDiagAttemptCount++;
  gDiagRan = runTxDiagStep();
}

void loop() {
  api.system.sleep.all((uint32_t)0xFFFFFFFF);

  if (gButtonWakePending) {
    noInterrupts();
    gButtonWakePending = false;
    gButtonWakeCount++;
    interrupts();
  } else {
    gOtherWakeCount++;
  }
}
