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

#ifndef TEST_TIMER_PERIOD_MS
#define TEST_TIMER_PERIOD_MS 10000
#endif

volatile bool gButtonWakePending = false;
volatile uint32_t gButtonWakeCount = 0;
volatile uint32_t gTimerWakeCount = 0;

void buttonWakeupCallback(void) {
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

  // Button wake setup
  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
  (void)api.system.sleep.registerWakeupCallback(buttonWakeupCallback);
}

void loop() {
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

  uint32_t sleepDurationMs = (uint32_t)0xFFFFFFFF;
#if TEST_TIMER_ENABLED
  sleepDurationMs = TEST_TIMER_PERIOD_MS;
#endif
  api.system.sleep.all(sleepDurationMs);

  // Handle button wake
  if (gButtonWakePending) {
    noInterrupts();
    gButtonWakePending = false;
    gButtonWakeCount++;
    interrupts();
  } else {
#if TEST_TIMER_ENABLED
    // If no button wake callback fired, treat wake as timer timeout.
    gTimerWakeCount++;
#endif
  }
}
