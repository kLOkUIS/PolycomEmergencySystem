#include <nrf_gpio.h>

static const uint32_t LED_PIN = P1_03;
static const uint32_t BUTTON_PIN = P1_01;
static const uint32_t TIMER_PERIOD_MS = 15000;

volatile bool gButtonWakePending = false;
volatile bool gTimerWakePending = false;

static bool buttonPressedRaw() {
	return nrf_gpio_pin_read(BUTTON_PIN) == 0;
}

static void pulse(uint32_t onMs, uint32_t offMs) {
	digitalWrite(LED_PIN, HIGH);
	delay(onMs);
	digitalWrite(LED_PIN, LOW);
	delay(offMs);
}

void wakeupCallback(void) {
	gButtonWakePending = true;
}

void timerCallback(void *) {
	gTimerWakePending = true;
}

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

	// Boot marker
	pulse(80, 80);
	pulse(80, 200);

	api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, BUTTON_PIN);
	api.system.sleep.registerWakeupCallback(wakeupCallback);

	api.system.timer.create(RAK_TIMER_0, timerCallback, RAK_TIMER_PERIODIC);
	api.system.timer.start(RAK_TIMER_0, TIMER_PERIOD_MS, NULL);
}

void loop() {
	api.system.sleep.all((uint32_t)0xFFFFFFFF);

	if (gButtonWakePending) {
		noInterrupts();
		gButtonWakePending = false;
		interrupts();

		// 1 blink for button wake
		pulse(120, 120);

		// Wait for button release to avoid confusing retriggers
		while (buttonPressedRaw()) {
			delay(10);
		}
		delay(30);
	}

	if (gTimerWakePending) {
		noInterrupts();
		gTimerWakePending = false;
		interrupts();

		// 2 blinks for timer wake, then go straight back to sleep
		pulse(80, 80);
		pulse(80, 120);
	}
}
