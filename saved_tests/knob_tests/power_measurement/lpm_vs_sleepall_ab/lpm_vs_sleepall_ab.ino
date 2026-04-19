#include <nrf_gpio.h>

// -----------------------------------------------------------------------------
// LPM vs sleep.all A/B current test (RAK4631)
// -----------------------------------------------------------------------------
// Goal:
//   Measure low-power current with only one variable changed:
//   A) sleep.all only
//   B) lpm.set(1) + sleep.all
//
// How to use:
//   1) Flash with TEST_MODE_LPM_PLUS_SLEEPALL = 0 and measure sleep current.
//   2) Flash with TEST_MODE_LPM_PLUS_SLEEPALL = 1 and measure sleep current.
//   3) Keep all hardware/probe/scope settings identical between runs.
//
// Notes:
//   - No timer wake is used.
//   - No radio activity.
//   - Button wake only.
//   - LED/MOTOR pulses happen only after a real button wake (optional).
// -----------------------------------------------------------------------------

static const uint32_t LED_PIN = P1_03;
static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t MOTOR_PIN = P1_04;

// A/B selector:
//   0 = sleep.all only
//   1 = lpm.set(1) + sleep.all
#ifndef TEST_MODE_LPM_PLUS_SLEEPALL
#define TEST_MODE_LPM_PLUS_SLEEPALL 1
#endif

// Radio sleep selector:
//   0 = radio left in default state
//   1 = api.lora.precv(0) called before sleep (stops RX / idles radio)
#ifndef TEST_MODE_RADIO_SLEEP
#define TEST_MODE_RADIO_SLEEP 1
#endif

// Set to 0 for absolutely minimal post-wake disturbance during measurements.
#ifndef WAKE_FEEDBACK_ENABLED
#define WAKE_FEEDBACK_ENABLED 1
#endif

static const uint32_t BUTTON_RELEASE_DEBOUNCE_MS = 25;

volatile bool gButtonWakePending = false;

static bool buttonPressedRaw() {
	return nrf_gpio_pin_read(BUTTON_PIN) == 0;
}

static void waitButtonRelease() {
	while (buttonPressedRaw()) {
		delay(5);
	}
	delay(BUTTON_RELEASE_DEBOUNCE_MS);
}

static void pulseLed(uint32_t onMs, uint32_t offMs) {
	digitalWrite(LED_PIN, HIGH);
	delay(onMs);
	digitalWrite(LED_PIN, LOW);
	delay(offMs);
}

static void pulseMotor(uint32_t onMs) {
	digitalWrite(MOTOR_PIN, HIGH);
	delay(onMs);
	digitalWrite(MOTOR_PIN, LOW);
}

void wakeupCallback(void) {
	// Callback stays tiny: only latch wake reason.
	gButtonWakePending = true;
}

static void configureSleepModeAB() {
#if TEST_MODE_LPM_PLUS_SLEEPALL
	api.system.lpm.set(1);
#endif

#if TEST_MODE_RADIO_SLEEP
	// Stop LoRa RX window and idle the SX1262 before sleep.
	api.lora.precv(0);
#endif

	api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
	api.system.sleep.registerWakeupCallback(wakeupCallback);
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	pinMode(MOTOR_PIN, OUTPUT);
	nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

	digitalWrite(LED_PIN, LOW);
	digitalWrite(MOTOR_PIN, LOW);

	configureSleepModeAB();

	// Boot marker indicates selected mode:
	// Group 1 — LPM:    1 pulse = off,  2 pulses = lpm.set(1)
	// Group 2 — Radio:  gap only = off, 1 pulse  = radio sleep
	pulseLed(60, 80);
#if TEST_MODE_LPM_PLUS_SLEEPALL
	pulseLed(60, 80);
#endif
	delay(300); // gap between groups
#if TEST_MODE_RADIO_SLEEP
	pulseLed(60, 140);
#endif
}

void loop() {
	// Keep outputs low before entering sleep measurement window.
	digitalWrite(LED_PIN, LOW);
	digitalWrite(MOTOR_PIN, LOW);

	if (buttonPressedRaw()) {
		waitButtonRelease();
	}

	api.system.sleep.all((uint32_t)0xFFFFFFFF);

	bool buttonWake = false;
	noInterrupts();
	buttonWake = gButtonWakePending;
	gButtonWakePending = false;
	interrupts();

#if WAKE_FEEDBACK_ENABLED
	if (buttonWake) {
		pulseLed(70, 20);
		pulseMotor(70);
	}
#else
	(void)buttonWake;
#endif
}
