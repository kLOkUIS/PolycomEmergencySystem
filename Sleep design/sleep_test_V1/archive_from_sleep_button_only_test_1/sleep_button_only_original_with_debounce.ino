#include <nrf_gpio.h>
extern "C" {
#include <radio.h>
}

static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t LED_PIN = P1_03;
static const uint32_t MOTOR_PIN = P1_04;

#ifndef TEST_MODE_LPM
#define TEST_MODE_LPM 1
#endif

// Diagnostic matrix (set via -D flag during compile):
// Variant A: TEST_MODE_SKIP_ALL_LORA=1, others=0 → NO LoRa calls (pure nRF52 baseline)
// Variant B: TEST_MODE_LORA_PRE_SLEEP=1, others=0 → only api.lora.precv(0)
// Variant C: TEST_MODE_DIRECT_RADIO_SLEEP=1, others=0 → only Radio.Sleep()
// Variant D: both=1 → both calls (current behavior)

#ifndef TEST_MODE_SKIP_ALL_LORA
#define TEST_MODE_SKIP_ALL_LORA 0
#endif

#ifndef TEST_MODE_LORA_PRE_SLEEP
#define TEST_MODE_LORA_PRE_SLEEP 0
#endif

#ifndef TEST_MODE_DIRECT_RADIO_SLEEP
#define TEST_MODE_DIRECT_RADIO_SLEEP 0
#endif

volatile bool gButtonWakePending = false;
volatile uint32_t gButtonWakeCount = 0;

void wakeupCallback(void) {
  gButtonWakePending = true;
}

static bool buttonPressedRaw() {
  return nrf_gpio_pin_read(BUTTON_PIN) == 0;
}

static void applySleepPreparation() {
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

#if !TEST_MODE_SKIP_ALL_LORA
  #if TEST_MODE_LORA_PRE_SLEEP
    api.lora.precv(0);
  #endif

  #if TEST_MODE_DIRECT_RADIO_SLEEP
    Radio.Sleep();
  #endif
#endif
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

  nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

#if TEST_MODE_LPM
  api.system.lpm.set(1);
#endif

  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
  (void)api.system.sleep.registerWakeupCallback(wakeupCallback);

  while (buttonPressedRaw()) {
    api.system.sleep.all(25);
  }
}

void loop() {
  applySleepPreparation();
  api.system.sleep.all((uint32_t)0xFFFFFFFF);

  if (gButtonWakePending) {
    noInterrupts();
    gButtonWakePending = false;
    gButtonWakeCount++;
    interrupts();

    while (buttonPressedRaw()) {
      api.system.sleep.all(25);
    }
  }
}
