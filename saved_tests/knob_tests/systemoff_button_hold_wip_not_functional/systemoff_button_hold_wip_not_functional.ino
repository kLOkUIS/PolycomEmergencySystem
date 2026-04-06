#include <nrf_gpio.h>
#include <nrf_power.h>
#include <nrf_soc.h>

// WIP / NOT FUNCTIONAL YET:
// System-off wake-on-button concept test for current-consumption comparison.

static const uint32_t LED_PIN = P1_03;
static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t MOTOR_PIN = P1_04;

static const uint32_t DEBOUNCE_MS = 25;
static const uint32_t WAKE_FEEDBACK_MS = 90;
static const uint32_t HOLD_CONFIRM_MS = 3000;
static const uint32_t CONFIRM_PULSE_MS = 140;
static const uint32_t SLEEP_PULSE_ON_MS = 60;
static const uint32_t SLEEP_PULSE_OFF_MS = 70;
static const uint8_t SYSTEMOFF_MAGIC = 0xA5;

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

static void sleepFeedback() {
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

static void enterSystemOff() {
  // Use hardware pin name for nRF HAL calls.
  uint32_t buttonHwPin = digitalPinToPinName(BUTTON_PIN);

  // Wake when button goes low.
  nrf_gpio_cfg_sense_input(buttonHwPin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

  ledOff();
  motorOff();
  delay(2);

  // Mark intentional system-off entry so wake detection is robust across reset-reason quirks.
  NRF_POWER->GPREGRET = SYSTEMOFF_MAGIC;

  // On RUI3 + SoftDevice builds, enter system-off through SD API.
#ifdef SOFTDEVICE_PRESENT
  (void)sd_power_system_off();
#else
  nrf_power_system_off();
#endif

  // Safety fallback; execution should never continue past system-off call.
  NRF_POWER->SYSTEMOFF = 1;
  __DSB();
  while (true) {
    __WFE();
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  ledOff();
  motorOff();

  bool wokeFromSystemOff = (NRF_POWER->GPREGRET == SYSTEMOFF_MAGIC);
  NRF_POWER->GPREGRET = 0;

  // Prevent immediate sleep/reboot loops if button is still held at boot/wake.
  if (digitalRead(BUTTON_PIN) == LOW) {
    waitRelease();
  }

  if (wokeFromSystemOff) {
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
}

void loop() {
  sleepFeedback();
  enterSystemOff();
}
