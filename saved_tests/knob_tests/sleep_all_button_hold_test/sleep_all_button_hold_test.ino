static const uint32_t LED_PIN = P1_03;
static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t MOTOR_PIN = P1_04;

static const uint32_t DEBOUNCE_MS = 25;
static const uint32_t WAKE_FEEDBACK_MS = 90;
static const uint32_t HOLD_CONFIRM_MS = 3000;
static const uint32_t CONFIRM_PULSE_MS = 140;
static const uint32_t SLEEP_PULSE_ON_MS = 60;
static const uint32_t SLEEP_PULSE_OFF_MS = 70;

volatile bool wakeEvent = false;

void wakeupCallback(void) {
  wakeEvent = true;
}

static void ledOn() { digitalWrite(LED_PIN, HIGH); }
static void ledOff() { digitalWrite(LED_PIN, LOW); }
static void motorOn() { digitalWrite(MOTOR_PIN, HIGH); }
static void motorOff() { digitalWrite(MOTOR_PIN, LOW); }

static void ledPulse(uint32_t onMs, uint32_t offMs) {
  ledOn();
  delay(onMs);
  ledOff();
  delay(offMs);
}

static void wakeFeedback() {
  ledOn();
  motorOn();
  delay(WAKE_FEEDBACK_MS);
  ledOff();
  motorOff();
}

static void preSleepFeedback() {
  ledPulse(SLEEP_PULSE_ON_MS, SLEEP_PULSE_OFF_MS);
  ledPulse(SLEEP_PULSE_ON_MS, 0);
}

static bool isPressedDebounced() {
  if (digitalRead(BUTTON_PIN) != LOW) return false;
  delay(DEBOUNCE_MS);
  return digitalRead(BUTTON_PIN) == LOW;
}

static void waitRelease() {
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(5);
  }
  delay(DEBOUNCE_MS);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  ledOff();
  motorOff();

  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
  api.system.sleep.registerWakeupCallback(wakeupCallback);
}

void loop() {
  ledOff();
  motorOff();

  if (digitalRead(BUTTON_PIN) == LOW) {
    waitRelease();
  }

  wakeEvent = false;
  preSleepFeedback();
  api.system.sleep.all();

  if (!wakeEvent) {
    return;
  }

  wakeFeedback();

  uint32_t t0 = millis();
  bool heldLongEnough = true;
  while ((millis() - t0) < HOLD_CONFIRM_MS) {
    if (digitalRead(BUTTON_PIN) != LOW) {
      heldLongEnough = false;
      break;
    }
    delay(10);
  }

  if (heldLongEnough && isPressedDebounced()) {
    ledPulse(CONFIRM_PULSE_MS, 80);
  }

  waitRelease();
}
