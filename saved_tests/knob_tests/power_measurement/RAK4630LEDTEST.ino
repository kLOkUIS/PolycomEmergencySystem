#include <nrf_gpio.h>

static const uint32_t LED_PIN = P1_03;
static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t MOTOR_PIN = P1_04;

static const uint32_t DEBOUNCE_MS = 25;
// Current measurement mode: no timer wake, no pre-sleep blink.
// Device sleeps indefinitely. Button wakes it for a brief pulse then
// it returns to sleep. Read DC voltage across 100Ohm shunt during sleep.

volatile uint32_t gWakeCallbackCount = 0;

static bool buttonPressedRaw() {
	return nrf_gpio_pin_read(BUTTON_PIN) == 0;
}

static void ledPulse(uint32_t onMs, uint32_t offMs) {
	digitalWrite(LED_PIN, HIGH);
	delay(onMs);
	digitalWrite(LED_PIN, LOW);
	delay(offMs);
}

static void motorPulse(uint32_t ms) {
	digitalWrite(MOTOR_PIN, HIGH);
	delay(ms);
	digitalWrite(MOTOR_PIN, LOW);
}

static void waitButtonRelease() {
	while (buttonPressedRaw()) {
		delay(5);
	}
	delay(DEBOUNCE_MS);
}

void wakeupCallback(void) {
	gWakeCallbackCount++;
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	pinMode(MOTOR_PIN, OUTPUT);
	nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

	digitalWrite(LED_PIN, LOW);
	digitalWrite(MOTOR_PIN, LOW);

	api.system.lpm.set(1);
	api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
	api.system.sleep.registerWakeupCallback(wakeupCallback);
}

void loop() {
	// Ensure outputs are low before sleeping
	digitalWrite(LED_PIN, LOW);
	digitalWrite(MOTOR_PIN, LOW);

	if (buttonPressedRaw()) {
		waitButtonRelease();
	}

	// Sleep for a very long time - effectively indefinite.
	// Button (falling edge) will wake it early. 0 means 0ms in RUI3, so use max uint32.
	// No timer wake, no pre-sleep blink, so the shunt reads pure system sleep current.
	uint32_t wakeCountBefore = gWakeCallbackCount;
	api.system.sleep.all((uint32_t)0xFFFFFFFF);  // ~49 days, woken by button only

	// Brief feedback so we know the button wake worked, then loop back to sleep.
	if (gWakeCallbackCount != wakeCountBefore) {
		ledPulse(80, 20);
		motorPulse(80);
	}
}
