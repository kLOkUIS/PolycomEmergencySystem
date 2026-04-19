/*
  external_reference_adafruit_systemoff.ino

  Source inspiration:
  - Adafruit nRF52 "blink_sleep" example by Pierre Constantineau
    https://github.com/adafruit/Adafruit_nRF52_Arduino/
      blob/master/libraries/Bluefruit52Lib/examples/Hardware/blink_sleep/blink_sleep.ino

  Goal:
  - Reproduce a known low-power pattern from outside this project.
  - Enter nRF52 System OFF and wake from button pin sense.

  Hardware target:
  - RAK4631 on WisBlock base
  - Button pin: P1_01 (active low)
*/

#include <Arduino.h>
#include <nrf_gpio.h>
#include <nrf_soc.h>

static const uint32_t LED_PIN = P1_03;
static const uint32_t MOTOR_PIN = P1_04;
static const uint32_t BUTTON_PIN = P1_01;

static void pulseLed(uint32_t onMs, uint32_t offMs) {
  digitalWrite(LED_PIN, HIGH);
  delay(onMs);
  digitalWrite(LED_PIN, LOW);
  delay(offMs);
}

static void enterSystemOffOnButtonLow() {
  // Ensure outputs are not consuming power before entering System OFF.
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

  // Configure wake-up sense directly on the button pin.
  // Wake when pin is pulled low (button press).
  nrf_gpio_cfg_sense_input(
    BUTTON_PIN,
    NRF_GPIO_PIN_PULLUP,
    NRF_GPIO_PIN_SENSE_LOW
  );

  // Enter deepest sleep state. Device resets on wake.
  (void) sd_power_system_off();

  // Should never execute unless System OFF failed.
  while (true) {
    delay(1000);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);

  // Boot marker: one short pulse, then enter System OFF.
  pulseLed(80, 100);

  // Wait for button release so we do not instantly wake again.
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(5);
  }
  delay(25);

  enterSystemOffOnButtonLow();
}

void loop() {
  // Not used: device enters System OFF from setup().
}
