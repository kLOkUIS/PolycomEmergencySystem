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

#ifndef TEST_TIMER_ENABLED
#define TEST_TIMER_ENABLED 0
#endif

#ifndef TEST_TIMER_PERIOD_MS
#define TEST_TIMER_PERIOD_MS 10000
#endif

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum State {
  STATE_IDLE,
  STATE_ACTIVE
};

static State gState = STATE_IDLE;

// ---------------------------------------------------------------------------
// Wake counters / flags
// ---------------------------------------------------------------------------
static const uint32_t DEBOUNCE_MS = 500;

volatile bool     gButtonWakePending = false;
volatile uint32_t gButtonWakeCount   = 0;
volatile uint32_t gTimerWakeCount    = 0;
static   uint32_t gLastButtonMs      = 0;

void buttonWakeupCallback(void) {
  gButtonWakePending = true;
}

// ---------------------------------------------------------------------------
// Active pattern: LED blink + motor pulse, then outputs off
// ---------------------------------------------------------------------------
static void runActivePattern(void) {
  // 3 quick LED blinks.
  // delay() is safe during active state: RUI3 source shows it stops app timers
  // before its RTC-based wait loop. millis() is a plain RTC register read.
  // Neither leaves anything running that would prevent deep sleep afterward.
  uint32_t t0 = millis();
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

  // One short motor pulse
  digitalWrite(MOTOR_PIN, HIGH);
  delay(200);
  digitalWrite(MOTOR_PIN, LOW);

  // Record total active duration for diagnostic purposes
  (void)(millis() - t0); // t0 available here if needed later
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
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
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
  // --- ACTIVE state: run pattern, then fall back to IDLE ---
  if (gState == STATE_ACTIVE) {
    runActivePattern();
    digitalWrite(LED_PIN,   LOW);
    digitalWrite(MOTOR_PIN, LOW);
    gLastButtonMs = millis(); // debounce window starts after pattern completes
    gState = STATE_IDLE;
  }

  // --- IDLE: go to sleep ---
  uint32_t sleepDurationMs = (uint32_t)0xFFFFFFFF;
#if TEST_TIMER_ENABLED
  sleepDurationMs = TEST_TIMER_PERIOD_MS;
#endif
  api.system.sleep.all(sleepDurationMs);

  // --- On wake: determine source ---
  if (gButtonWakePending) {
    noInterrupts();
    gButtonWakePending = false;
    interrupts();
    uint32_t now = millis();
    if ((uint32_t)(now - gLastButtonMs) >= DEBOUNCE_MS) {
      gButtonWakeCount++;
      gState = STATE_ACTIVE;
    }
  } else {
#if TEST_TIMER_ENABLED
    gTimerWakeCount++;
    // gState stays IDLE — timer wake does nothing visible yet
#endif
  }
}
