// Ring Cadence Detector (RTT-based)
// Measures incoming call LED/CALL_SENSE blink pattern and outputs timing via RTT.
// Hook up J-Link and use Segger J-Link RTT viewer to read debug output.
//
// Detects three consecutive on/off cycles with similar durations,
// then outputs the measured pattern to RTT for capture.
//
// CALL_SENSE: P0_25 (active=LOW, inactive=HIGH)
// LED_RX: P0_24 (mirrors debounced CALL_SENSE for visual feedback)

#include <stdint.h>
#include <stdio.h>

// Segger RTT printf macro (minimal RTT setup, may need RUI integration)
extern "C" {
	void SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
	#define RTT_PRINTF(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
}

static const uint32_t CALL_SENSE_PIN = P0_25;
static const uint32_t LED_RX_PIN = P0_24;

static const uint32_t DEBOUNCE_MS = 8;
static const uint32_t LOOP_DELAY_MS = 2;
static const uint32_t SIMILAR_THRESHOLD_PCT = 20;  // Allow +/-20% variance in timing
static const uint32_t MIN_CYCLE_MS = 100;  // Ignore bounces shorter than 100 ms
static const uint32_t MAX_CYCLE_MS = 5000;  // Ignore delays longer than 5 seconds
static const uint32_t CALL_ENDED_OFF_GAP_MS = 7000;  // Long OFF gap means ringing stopped

enum CadenceState {
	CADENCE_IDLE = 0,
	CADENCE_TRACKING,
	CADENCE_COLLECTED,
};

struct CycleTiming {
	uint32_t onDurationMs;
	uint32_t offDurationMs;
};

CadenceState gCadenceState = CADENCE_IDLE;
uint8_t gCycleCount = 0;
CycleTiming gCycles[3] = {};

bool gCallSenseRawHigh = true;
bool gCallSenseStableHigh = true;
uint32_t gCallSenseRawChangedAtMs = 0;
bool gPrevCadenceActive = false;

uint32_t gCurrentCycleStartMs = 0;
bool gCurrentPhaseIsOn = false;  // true = on (LOW), false = off (HIGH)

void allKeysLow() {
	const uint32_t keyPins[] = {P1_01, P1_02, P1_03, P1_04, P0_05, P0_04};
	for (size_t i = 0; i < sizeof(keyPins)/sizeof(keyPins[0]); i++) {
		pinMode(keyPins[i], OUTPUT);
		digitalWrite(keyPins[i], LOW);
	}
}

static bool callSenseActive() {
	return !gCallSenseStableHigh;
}

static void updateCallSense() {
	const bool rawHigh = digitalRead(CALL_SENSE_PIN) == HIGH;
	const uint32_t nowMs = millis();

	if (rawHigh != gCallSenseRawHigh) {
		gCallSenseRawHigh = rawHigh;
		gCallSenseRawChangedAtMs = nowMs;
	}

	if ((nowMs - gCallSenseRawChangedAtMs) >= DEBOUNCE_MS) {
		gCallSenseStableHigh = gCallSenseRawHigh;
	}

	digitalWrite(LED_RX_PIN, callSenseActive() ? HIGH : LOW);
}

static bool isSimilar(uint32_t a, uint32_t b) {
	if (a == 0 || b == 0) return false;
	uint32_t minVal = a < b ? a : b;
	uint32_t maxVal = a > b ? a : b;
	uint32_t percentDiff = ((maxVal - minVal) * 100) / minVal;
	return percentDiff <= SIMILAR_THRESHOLD_PCT;
}

static void reportDetectedCadence(const char *reason, uint32_t onMs, uint32_t offMs) {
	RTT_PRINTF("\n=== RING CADENCE DETECTED (%s) ===\n", reason);
	RTT_PRINTF("ON duration:  %lu ms\n", onMs);
	RTT_PRINTF("OFF duration: %lu ms\n", offMs);
	RTT_PRINTF("===================================\n\n");
}

static void updateCadenceTracking() {
	const uint32_t nowMs = millis();
	const bool isActive = callSenseActive();

	if (gCadenceState == CADENCE_IDLE && isActive && !gPrevCadenceActive) {
		// Start tracking when call goes active.
		gCadenceState = CADENCE_TRACKING;
		gCycleCount = 0;
		gCurrentCycleStartMs = nowMs;
		gCurrentPhaseIsOn = true;
		RTT_PRINTF("Cadence tracking started\n");
		gPrevCadenceActive = isActive;
		return;
	}

	if (gCadenceState != CADENCE_TRACKING) {
		gPrevCadenceActive = isActive;
		return;
	}

	// If OFF stays long, ringing likely ended. Finalize using stable cycles only.
	if (!isActive && !gCurrentPhaseIsOn && (nowMs - gCurrentCycleStartMs) >= CALL_ENDED_OFF_GAP_MS) {
		if (gCycleCount >= 2 &&
			isSimilar(gCycles[0].onDurationMs, gCycles[1].onDurationMs) &&
			isSimilar(gCycles[0].offDurationMs, gCycles[1].offDurationMs)) {
			reportDetectedCadence("Call ended", gCycles[0].onDurationMs, gCycles[0].offDurationMs);
			gCadenceState = CADENCE_COLLECTED;
		} else {
			RTT_PRINTF("Call ended without stable cadence; discarding\n");
			gCadenceState = CADENCE_IDLE;
		}

		gCycleCount = 0;
		gCurrentPhaseIsOn = false;
		gPrevCadenceActive = isActive;
		return;
	}

	if (isActive == gPrevCadenceActive) {
		return;
	}

    // Stable edge detected: record prior phase duration and switch phase.
	const uint32_t phaseDurationMs = nowMs - gCurrentCycleStartMs;

	if (gPrevCadenceActive) {
		// Ending ON phase, starting OFF phase.
		if (phaseDurationMs < MIN_CYCLE_MS || phaseDurationMs > MAX_CYCLE_MS) {
			// Invalid duration, reset.
			gCadenceState = CADENCE_IDLE;
			RTT_PRINTF("Reset (invalid ON duration: %lu ms)\n", phaseDurationMs);
			gCycleCount = 0;
			gCurrentPhaseIsOn = false;
			gPrevCadenceActive = isActive;
			return;
		}
		gCycles[gCycleCount].onDurationMs = phaseDurationMs;
		gCurrentPhaseIsOn = false;
	} else {
		// Ending OFF phase, starting ON phase.
		if (phaseDurationMs < MIN_CYCLE_MS || phaseDurationMs > MAX_CYCLE_MS) {
			// Invalid duration, reset.
			gCadenceState = CADENCE_IDLE;
			RTT_PRINTF("Reset (invalid OFF duration: %lu ms)\n", phaseDurationMs);
			gCycleCount = 0;
			gCurrentPhaseIsOn = false;
			gPrevCadenceActive = isActive;
			return;
		}
		gCycles[gCycleCount].offDurationMs = phaseDurationMs;
		gCycleCount++;

		// Check if we have 3 complete cycles and they're ALL similar.
		// This way we only report once we're confident in the pattern.
		if (gCycleCount >= 3) {
			bool cycle0Similar1 =
				isSimilar(gCycles[0].onDurationMs, gCycles[1].onDurationMs) &&
				isSimilar(gCycles[0].offDurationMs, gCycles[1].offDurationMs);

			bool cycle1Similar2 =
				isSimilar(gCycles[1].onDurationMs, gCycles[2].onDurationMs) &&
				isSimilar(gCycles[1].offDurationMs, gCycles[2].offDurationMs);

			if (cycle0Similar1 && cycle1Similar2) {
				// Pattern confirmed during stable ringing!
				gCadenceState = CADENCE_COLLECTED;
				reportDetectedCadence("3 cycles stable", gCycles[0].onDurationMs, gCycles[0].offDurationMs);
				return;
			}

			// Keep a rolling 2-cycle history if not stable yet.
			gCycles[0] = gCycles[1];
			gCycles[1] = gCycles[2];
			gCycleCount = 2;
		}

		gCurrentPhaseIsOn = true;
	}

	gCurrentCycleStartMs = nowMs;
	gPrevCadenceActive = isActive;
}

void setup() {
	pinMode(CALL_SENSE_PIN, INPUT);
	pinMode(LED_RX_PIN, OUTPUT);
	digitalWrite(LED_RX_PIN, LOW);
	allKeysLow();

	gCallSenseRawHigh = digitalRead(CALL_SENSE_PIN) == HIGH;
	gCallSenseStableHigh = gCallSenseRawHigh;
	gCallSenseRawChangedAtMs = millis();
	gCurrentCycleStartMs = millis();
	gPrevCadenceActive = callSenseActive();

	// Initialize Segger RTT (note: RUI may handle this automatically).
	api.system.lpm.set(1);

	RTT_PRINTF("\n\n=== Ring Cadence Detector ===\n");
	RTT_PRINTF("Waiting for incoming call...\n");
	RTT_PRINTF("Threshold: +/- %u%% variance\n", SIMILAR_THRESHOLD_PCT);
	RTT_PRINTF("============================\n\n");
}

void loop() {
	updateCallSense();
	updateCadenceTracking();

	delay(LOOP_DELAY_MS);
}
