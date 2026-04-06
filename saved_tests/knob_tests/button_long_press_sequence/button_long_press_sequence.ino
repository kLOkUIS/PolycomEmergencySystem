static const uint32_t LED_PIN   = P1_03;
static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t MOTOR_PIN  = P1_04;
static const uint32_t BUZZER_PIN = P1_04;   // same pin — buzzer driven via MOSFET

static const uint32_t DEBOUNCE_MS      = 25;
static const uint32_t LONG_PRESS_MS    = 3000;  // hold time required

// --- helpers ---

static void ledBlink(uint32_t onMs = 80, uint32_t offMs = 80) {
  digitalWrite(LED_PIN, HIGH);
  delay(onMs);
  digitalWrite(LED_PIN, LOW);
  delay(offMs);
}

static void buzz(uint32_t freq, uint32_t durationMs) {
  tone(BUZZER_PIN, freq, durationMs);
  delay(durationMs);
  noTone(BUZZER_PIN);
  // tone() hands the pin to a timer; reclaim it for digitalWrite()
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
}

// Plays after confirmed long press — LED blinks + ascending tones
static void runRewardSequence() {
  const uint32_t freqs[]    = { 880, 1175 };   // A5, D6
  const uint32_t durations[] = { 120, 180 };

  for (int i = 0; i < 2; i++) {
    ledBlink(100, 60);
    buzz(freqs[i], durations[i]);
    delay(80);
  }
}

void setup() {
  pinMode(LED_PIN,   OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN,   LOW);
  digitalWrite(MOTOR_PIN, LOW);
}

void loop() {
  // Wait for initial press with debounce
  if (digitalRead(BUTTON_PIN) != LOW) return;
  delay(DEBOUNCE_MS);
  if (digitalRead(BUTTON_PIN) != LOW) return;

  // Acknowledge: instant tactile + LED (motor fires at t=0)
  digitalWrite(MOTOR_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(20);
  digitalWrite(MOTOR_PIN, LOW);

  // Wait up to LONG_PRESS_MS for the button to still be held
  uint32_t held = DEBOUNCE_MS + 120;   // time already spent
  bool longPress = false;
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(10);
    held += 10;
    if (held >= LONG_PRESS_MS) {
      longPress = true;
      break;
    }
  }

  if (longPress) {
    // Confirmation: short rising chirp
    buzz(600, 60);
    buzz(900, 80);
    delay(300);

    // Reward sequence
    runRewardSequence();

    // Wait for release
    while (digitalRead(BUTTON_PIN) == LOW) delay(1);
  }

  delay(DEBOUNCE_MS);
}
