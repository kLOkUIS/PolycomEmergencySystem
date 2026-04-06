// Watchdog recovery/strobe validation test (single-file, no helper folders).
// Purpose:
// - Simulate "stuck" conditions with a configurable probability.
// - Reboot through a recovery path that sets a persistent marker.
// - On next boot, strobe LED quickly if previous reset was recovery-triggered.

#include <Arduino.h>

// ------------------------------
// Target selection
// ------------------------------
// Set to 1 to test TX LED pin behavior, 0 to test RX LED pin behavior.
#ifndef TEST_TARGET_TX
#define TEST_TARGET_TX 1
#endif

#if TEST_TARGET_TX
static const uint32_t TEST_LED_PIN = P1_03;
#else
static const uint32_t TEST_LED_PIN = P0_24;
#endif

// ------------------------------
// Fault injection tuning
// ------------------------------
// 40% chance to simulate a stuck condition each evaluation window.
#ifndef TEST_STUCK_CHANCE_PERCENT
#define TEST_STUCK_CHANCE_PERCENT 40
#endif

// Evaluate for fault injection every this many milliseconds.
#ifndef TEST_INJECT_PERIOD_MS
#define TEST_INJECT_PERIOD_MS 5000UL
#endif

// If "stuck" is active for this long without progress, trigger recovery reboot.
#ifndef TEST_STALL_TIMEOUT_MS
#define TEST_STALL_TIMEOUT_MS 3000UL
#endif

// Hard cap for active session runtime before recovery reboot.
#ifndef TEST_ABSOLUTE_TIMEOUT_MS
#define TEST_ABSOLUTE_TIMEOUT_MS 20000UL
#endif

// Persistent marker for "recovery reboot happened last session".
static const uint32_t RECOVERY_BOOT_MAGIC = 0xC0DEF00Du;
static uint32_t gRecoveryBootMarker __attribute__((section(".noinit")));

bool gWatchdogActive = false;
bool gSimulatedStuck = false;
uint32_t gWatchdogArmedAtMs = 0;
uint32_t gLastProgressAtMs = 0;
uint32_t gLastInjectCheckAtMs = 0;

static void rebootForRecovery() {
	gRecoveryBootMarker = RECOVERY_BOOT_MAGIC;
	api.system.reboot();
}

static void maybeShowRecoveryBootStrobe() {
	if (gRecoveryBootMarker != RECOVERY_BOOT_MAGIC) {
		return;
	}

	gRecoveryBootMarker = 0;
	for (uint8_t i = 0; i < 3; i++) {
		digitalWrite(TEST_LED_PIN, HIGH);
		delay(45);
		digitalWrite(TEST_LED_PIN, LOW);
		delay(45);
	}
}

static void watchdogArmIfNeeded() {
	if (gWatchdogActive) {
		return;
	}
	const uint32_t nowMs = millis();
	gWatchdogActive = true;
	gWatchdogArmedAtMs = nowMs;
	gLastProgressAtMs = nowMs;
}

static void watchdogTouchProgress() {
	if (!gWatchdogActive) {
		return;
	}
	gLastProgressAtMs = millis();
}

static void watchdogService() {
	if (!gWatchdogActive) {
		return;
	}

	const uint32_t nowMs = millis();
	if ((uint32_t)(nowMs - gLastProgressAtMs) >= TEST_STALL_TIMEOUT_MS ||
		(uint32_t)(nowMs - gWatchdogArmedAtMs) >= TEST_ABSOLUTE_TIMEOUT_MS) {
		rebootForRecovery();
	}
}

static void maybeInjectStuckFault() {
	const uint32_t nowMs = millis();
	if ((uint32_t)(nowMs - gLastInjectCheckAtMs) < TEST_INJECT_PERIOD_MS) {
		return;
	}
	gLastInjectCheckAtMs = nowMs;

	if (gSimulatedStuck) {
		return;
	}

	const uint8_t roll = (uint8_t)random(100);
	if (roll < TEST_STUCK_CHANCE_PERCENT) {
		gSimulatedStuck = true;
	}
}

void setup() {
	pinMode(TEST_LED_PIN, OUTPUT);
	digitalWrite(TEST_LED_PIN, LOW);

	maybeShowRecoveryBootStrobe();

	randomSeed((uint32_t)millis() ^ (uint32_t)analogRead(A0));
	api.system.lpm.set(0);

	gWatchdogActive = false;
	gSimulatedStuck = false;
	gWatchdogArmedAtMs = 0;
	gLastProgressAtMs = 0;
	gLastInjectCheckAtMs = millis();
}

void loop() {
	watchdogArmIfNeeded();

	maybeInjectStuckFault();

	if (!gSimulatedStuck) {
		// Healthy behavior: blink heartbeat and refresh watchdog progress.
		digitalWrite(TEST_LED_PIN, HIGH);
		delay(70);
		digitalWrite(TEST_LED_PIN, LOW);
		delay(130);
		watchdogTouchProgress();
	} else {
		// Simulated stuck: intentionally stop touching progress while loop still runs.
		delay(50);
	}

	watchdogService();
}
