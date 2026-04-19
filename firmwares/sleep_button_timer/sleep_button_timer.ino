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

#ifndef TEST_TIMER_ENABLED
#define TEST_TIMER_ENABLED 1
#endif

#ifndef TEST_TIMER_WAKE_PERIOD_MS
#define TEST_TIMER_WAKE_PERIOD_MS 30000
#endif

volatile bool gButtonWakePending = false;
volatile bool gTimerWakePending = false;
volatile uint32_t gButtonWakeCount = 0;
volatile uint32_t gTimerWakeCount = 0;

void wakeupCallback(void) {
  gButtonWakePending = true;
}

void timerCallback(void *) {
  gTimerWakePending = true;
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

#if TEST_TIMER_ENABLED
  api.system.timer.create(RAK_TIMER_0, timerCallback, RAK_TIMER_PERIODIC);
  api.system.timer.start(RAK_TIMER_0, TEST_TIMER_WAKE_PERIOD_MS, NULL);
#endif
}

void loop() {
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);
  api.system.sleep.all((uint32_t)0xFFFFFFFF);

  if (gButtonWakePending) {
    noInterrupts();
    gButtonWakePending = false;
    gButtonWakeCount++;
    interrupts();
  }

#if TEST_TIMER_ENABLED
  if (gTimerWakePending) {
    noInterrupts();
    gTimerWakePending = false;
    gTimerWakeCount++;
    interrupts();
  }
#endif
}
