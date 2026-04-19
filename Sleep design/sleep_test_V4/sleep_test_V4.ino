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

// Radio diagnostic mode (selected at compile time):
// 0 = disabled (baseline button wake only)
// 1 = configure LoRa only, no shutdown
// 2 = configure LoRa + Radio.Sleep only
// 3 = configure LoRa + precv(0) only
// 4 = configure LoRa + precv(0) + Radio.Sleep
// 5 = configure LoRa + repeated precv(0) + Radio.Sleep sequence
#ifndef TEST_LORA_DIAG_MODE
#define TEST_LORA_DIAG_MODE 5
#endif

#ifndef TEST_LORA_SHUTDOWN_DELAY_MS
#define TEST_LORA_SHUTDOWN_DELAY_MS 10
#endif

#ifndef TEST_LORA_EXTRA_SLEEP_GAP_MS
#define TEST_LORA_EXTRA_SLEEP_GAP_MS 3
#endif

static const uint32_t LORA_FREQ_HZ  = 868000000;
static const uint16_t LORA_SF       = 7;
static const uint16_t LORA_BW       = 0;
static const uint16_t LORA_CR       = 1;
static const uint16_t LORA_PREAMBLE = 8;
static const int16_t  LORA_TX_POWER = 14;

volatile bool     gButtonWakePending = false;
volatile uint32_t gButtonWakeCount   = 0;
volatile uint32_t gOtherWakeCount    = 0;
static   uint32_t gDiagAttemptCount  = 0;
static   uint32_t gDiagInitFailCount = 0;
static   uint32_t gDiagStepFailCount = 0;
static   bool     gRadioDiagRan      = false;

enum RadioDiagMode {
  RADIO_DIAG_DISABLED = 0,
  RADIO_DIAG_INIT_ONLY = 1,
  RADIO_DIAG_INIT_RADIO_SLEEP_ONLY = 2,
  RADIO_DIAG_INIT_PRECV_ONLY = 3,
  RADIO_DIAG_INIT_PRECV_RADIO_SLEEP = 4,
  RADIO_DIAG_INIT_PRECV_RADIO_SLEEP_STRICT = 5,
};

void buttonWakeupCallback(void) {
  gButtonWakePending = true;
}

static bool configureLoRaP2P(void) {
  if (api.lora.nwm.get() != 0) {
    api.lora.nwm.set();
    api.system.reboot();
    return false;
  }

  bool ok = true;
  ok &= api.lora.pfreq.set(LORA_FREQ_HZ);
  ok &= api.lora.psf.set(LORA_SF);
  ok &= api.lora.pbw.set(LORA_BW);
  ok &= api.lora.pcr.set(LORA_CR);
  ok &= api.lora.ppl.set(LORA_PREAMBLE);
  ok &= api.lora.ptp.set(LORA_TX_POWER);
  return ok;
}

static void radioSleepOnly(void) {
  delay(TEST_LORA_SHUTDOWN_DELAY_MS);
  Radio.Sleep();
}

static void startReceiveOnly(void) {
  api.lora.precv(0);
}

static void startReceiveThenRadioSleep(void) {
  api.lora.precv(0);
  delay(TEST_LORA_SHUTDOWN_DELAY_MS);
  Radio.Sleep();
}

static void startReceiveThenRadioSleepStrict(void) {
  api.lora.precv(0);
  delay(TEST_LORA_SHUTDOWN_DELAY_MS);
  Radio.Sleep();

  delay(TEST_LORA_EXTRA_SLEEP_GAP_MS);
  api.lora.precv(0);
  Radio.Sleep();

  delay(TEST_LORA_EXTRA_SLEEP_GAP_MS);
  Radio.Sleep();
}

static bool runRadioDiagStep(void) {
  const uint32_t mode = (uint32_t)TEST_LORA_DIAG_MODE;

  if (mode == RADIO_DIAG_DISABLED) {
    return true;
  }

  if (!configureLoRaP2P()) {
    gDiagInitFailCount++;
    return false;
  }

  if (mode == RADIO_DIAG_INIT_ONLY) {
    return true;
  }

  if (mode == RADIO_DIAG_INIT_RADIO_SLEEP_ONLY) {
    radioSleepOnly();
    return true;
  }

  if (mode == RADIO_DIAG_INIT_PRECV_ONLY) {
    startReceiveOnly();
    return true;
  }

  if (mode == RADIO_DIAG_INIT_PRECV_RADIO_SLEEP) {
    startReceiveThenRadioSleep();
    return true;
  }

  if (mode == RADIO_DIAG_INIT_PRECV_RADIO_SLEEP_STRICT) {
    startReceiveThenRadioSleepStrict();
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
  gRadioDiagRan = runRadioDiagStep();
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