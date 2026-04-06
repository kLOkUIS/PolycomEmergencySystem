#include <SEGGER_RTT.h>

// Archived hardware validation sketch for button + LED + motor driver.
// Pins:
// - LED: P1_03
// - Button: P1_01 (INPUT_PULLUP, active LOW)
// - Motor gate: P1_04 (low-side NMOS)

static const uint32_t LED_PIN = P1_03;
static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t MOTOR_PIN = P1_04;
static const bool MOTOR_ACTIVE_HIGH = true;  // Low-side NMOS gate is usually active-high
static const uint32_t MOTOR_MASK = (1u << 4);  // P1.04

static const uint32_t LED_PULSE_ON_MS = 80;
static const uint32_t LED_PULSE_OFF_MS = 80;
static const uint32_t MOTOR_PULSE_ON_MS = 220;
static const uint32_t MOTOR_PULSE_OFF_MS = 120;
static const uint32_t DEBOUNCE_MS = 25;
static const uint32_t GATE_HOLD_MS = 5000;

static void motorOn() {
  if (MOTOR_ACTIVE_HIGH) {
    NRF_P1->OUTSET = MOTOR_MASK;
  } else {
    NRF_P1->OUTCLR = MOTOR_MASK;
  }
}

static void motorOff() {
  if (MOTOR_ACTIVE_HIGH) {
    NRF_P1->OUTCLR = MOTOR_MASK;
  } else {
    NRF_P1->OUTSET = MOTOR_MASK;
  }
}

static void runSequence() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(LED_PULSE_ON_MS);
    digitalWrite(LED_PIN, LOW);
    delay(LED_PULSE_OFF_MS);
  }

  SEGGER_RTT_WriteString(0, "Diag: gate HIGH hold 5s\r\n");
  motorOn();
  delay(GATE_HOLD_MS);
  motorOff();
  SEGGER_RTT_WriteString(0, "Diag: gate LOW hold 5s\r\n");
  delay(GATE_HOLD_MS);

  // Longer first pulse helps overcome motor startup friction.
  motorOn();
  delay(MOTOR_PULSE_ON_MS + 120);
  motorOff();
  delay(MOTOR_PULSE_OFF_MS);

  for (int i = 0; i < 2; i++) {
    motorOn();
    delay(MOTOR_PULSE_ON_MS);
    motorOff();
    delay(MOTOR_PULSE_OFF_MS);
  }
}

void setup() {
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "RAK4631 button/motor test start\r\n");

  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  // Force P1.04 to output + high-drive (H0H1) for robust NMOS gate control.
  NRF_P1->PIN_CNF[4] = (1u << 0) | (6u << 8);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  motorOff();

  SEGGER_RTT_WriteString(0, "Pins: LED=P1_03, BUTTON=P1_01, MOTOR=P1_04\r\n");
  SEGGER_RTT_WriteString(0, "Motor config: low-side NMOS, active-high gate\r\n");
  SEGGER_RTT_WriteString(0, "Waiting for button press...\r\n");
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(DEBOUNCE_MS);
    if (digitalRead(BUTTON_PIN) == LOW) {
      SEGGER_RTT_WriteString(0, "Button pressed -> running sequence\r\n");
      runSequence();
      SEGGER_RTT_WriteString(0, "Sequence complete\r\n");

      while (digitalRead(BUTTON_PIN) == LOW) {
        delay(1);
      }
      delay(DEBOUNCE_MS);
      SEGGER_RTT_WriteString(0, "Waiting for next press...\r\n");
    }
  }
}
