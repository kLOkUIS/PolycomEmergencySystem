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

volatile bool gButtonWakePending = false;
volatile uint32_t gButtonWakeCount = 0;

void wakeupCallback(void) {
  gButtonWakePending = true;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

#if TEST_MODE_LPM
  api.system.lpm.set(1);
#endif

  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
  (void)api.system.sleep.registerWakeupCallback(wakeupCallback);
}

void loop() {
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);
  api.system.sleep.all((uint32_t)0xFFFFFFFF);

  noInterrupts();
  gButtonWakePending = false;
  gButtonWakeCount++;
  interrupts();
}
