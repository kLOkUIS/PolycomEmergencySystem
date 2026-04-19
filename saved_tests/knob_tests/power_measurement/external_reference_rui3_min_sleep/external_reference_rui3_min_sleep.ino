#include <nrf_gpio.h>

// RUI3 minimal low-power reference sketch for RAK4631.
// Goal: remove app complexity and measure the practical floor on this board.

static const uint32_t LED_PIN = P1_03;
static const uint32_t MOTOR_PIN = P1_04;
static const uint32_t BUTTON_PIN = P1_01;

// Toggle to compare lpm off/on in a minimal RUI3-only setup.
#ifndef TEST_MODE_LPM
#define TEST_MODE_LPM 1
#endif

// Toggle to stop LoRa RX and let radio idle before sleep.
#ifndef TEST_MODE_RADIO_IDLE
#define TEST_MODE_RADIO_IDLE 1
#endif

volatile bool gWakePending = false;

void wakeupCallback(void) {
  gWakePending = true;
}

static void prepareLowPower() {
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

#if TEST_MODE_LPM
  api.system.lpm.set(1);
#endif

#if TEST_MODE_RADIO_IDLE
  api.lora.precv(0);
#endif

  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
  api.system.sleep.registerWakeupCallback(wakeupCallback);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

  prepareLowPower();

  // Wait button release once so we do not immediately wake from a held press.
  while (nrf_gpio_pin_read(BUTTON_PIN) == 0) {
    delay(5);
  }
  delay(25);
}

void loop() {
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

  api.system.sleep.all((uint32_t)0xFFFFFFFF);

  // Keep wake path tiny. No LED/motor activity to avoid measurement disturbance.
  noInterrupts();
  gWakePending = false;
  interrupts();
}
