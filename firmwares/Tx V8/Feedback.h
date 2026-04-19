#pragma once

#include <Arduino.h>
#include "FeedbackTuning.h"

// Feedback controller design:
// - Single owner for LED + motor pin writes.
// - Non-blocking update loop (no delay() inside runtime patterns).
// - Button-warning uses PWM; all other cues are explicit step patterns.
// - Motor OFF/ON are issued through analogWrite-domain first to avoid stale PWM duty on nRF52.

enum FeedbackMode : uint8_t {
	FEEDBACK_MODE_IDLE = 0,
	FEEDBACK_MODE_WARNING,
	FEEDBACK_MODE_PATTERN,
};

enum FeedbackMotorMode : uint8_t {
	FEEDBACK_MOTOR_OFF = 0,
	FEEDBACK_MOTOR_ON,
	FEEDBACK_MOTOR_PWM,
};

struct FeedbackStep {
	uint16_t durationMs;
	bool ledOn;
	FeedbackMotorMode motorMode;
	uint8_t motorValue;
};

struct FeedbackController {
	uint32_t ledPin;
	uint32_t motorPin;
	FeedbackMode mode;
	uint32_t warningStartedAtMs;
	uint32_t warningTargetMs;
	const FeedbackStep *patternSteps;
	uint8_t patternStepCount;
	uint8_t patternStepIndex;
	uint32_t patternStepStartedAtMs;
};

static const uint32_t FEEDBACK_WARNING_POST_TAP_SILENT_MS = FeedbackTuning::WARNING_POST_TAP_SILENT_MS;
static const uint32_t FEEDBACK_WARNING_PRE_CONFIRM_SILENT_MS = FeedbackTuning::WARNING_PRE_CONFIRM_SILENT_MS;
static const uint8_t FEEDBACK_WARNING_MIN_PWM = FeedbackTuning::WARNING_MIN_PWM;
static const uint8_t FEEDBACK_WARNING_MAX_PWM = FeedbackTuning::WARNING_MAX_PWM;

static const FeedbackStep FEEDBACK_PATTERN_BUTTON_TAP[] = {
	{FeedbackTuning::BUTTON_TAP_ON_MS, true, FEEDBACK_MOTOR_ON, 0},
};

static const FeedbackStep FEEDBACK_PATTERN_LONG_PRESS_CONFIRMED[] = {
	{FeedbackTuning::LONG_PRESS_CONFIRM_SILENCE_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::LONG_PRESS_CONFIRM_PULSE_MS, true, FEEDBACK_MOTOR_ON, 0},
};

static const FeedbackStep FEEDBACK_PATTERN_CALL_ESTABLISHED[] = {
	{FeedbackTuning::CALL_ESTABLISHED_PULSE1_MS, true, FEEDBACK_MOTOR_ON, 0},
	{FeedbackTuning::CALL_ESTABLISHED_GAP1_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::CALL_ESTABLISHED_PULSE2_MS, true, FEEDBACK_MOTOR_ON, 0},
	{FeedbackTuning::CALL_ESTABLISHED_GAP2_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::CALL_ESTABLISHED_PULSE3_MS, true, FEEDBACK_MOTOR_ON, 0},
};

static const FeedbackStep FEEDBACK_PATTERN_ACK_ONLY[] = {
	{FeedbackTuning::ACK_ONLY_PULSE1_MS, true, FEEDBACK_MOTOR_ON, 0},
	{FeedbackTuning::ACK_ONLY_GAP_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::ACK_ONLY_PULSE2_MS, true, FEEDBACK_MOTOR_ON, 0},
};

static const FeedbackStep FEEDBACK_PATTERN_FAILURE[] = {
	{FeedbackTuning::FAILURE_PULSE_MS, true, FEEDBACK_MOTOR_ON, 0},
};

static const FeedbackStep FEEDBACK_PATTERN_BATTERY_LOW[] = {
	{FeedbackTuning::BATTERY_STROBE_ON_MS, true, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_OFF_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_GAP_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_ON_MS, true, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_OFF_MS, false, FEEDBACK_MOTOR_OFF, 0},
};

static const FeedbackStep FEEDBACK_PATTERN_BATTERY_CRITICAL[] = {
	{FeedbackTuning::BATTERY_STROBE_ON_MS, true, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_OFF_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_GAP_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_ON_MS, true, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_OFF_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_GAP_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_ON_MS, true, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_OFF_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_GAP_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_ON_MS, true, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_OFF_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_GAP_MS, false, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_ON_MS, true, FEEDBACK_MOTOR_OFF, 0},
	{FeedbackTuning::BATTERY_STROBE_OFF_MS, false, FEEDBACK_MOTOR_OFF, 0},
};

static inline void feedbackApplyOutputs(FeedbackController &controller, bool ledOn, FeedbackMotorMode motorMode, uint8_t motorValue) {
	digitalWrite(controller.ledPin, ledOn ? HIGH : LOW);
	switch (motorMode) {
	case FEEDBACK_MOTOR_OFF:
		// Keep motor pin control in PWM domain to avoid stale duty carry-over on nRF52.
		analogWrite(controller.motorPin, 0);
		digitalWrite(controller.motorPin, LOW);
		break;
	case FEEDBACK_MOTOR_ON:
		analogWrite(controller.motorPin, 255);
		break;
	case FEEDBACK_MOTOR_PWM:
		analogWrite(controller.motorPin, motorValue);
		break;
	}
}

static inline void feedbackStop(FeedbackController &controller) {
	controller.mode = FEEDBACK_MODE_IDLE;
	controller.patternSteps = nullptr;
	controller.patternStepCount = 0;
	controller.patternStepIndex = 0;
	controller.patternStepStartedAtMs = 0;
	feedbackApplyOutputs(controller, false, FEEDBACK_MOTOR_OFF, 0);
}

static inline void feedbackInit(FeedbackController &controller, uint32_t ledPin, uint32_t motorPin) {
	controller.ledPin = ledPin;
	controller.motorPin = motorPin;
	controller.mode = FEEDBACK_MODE_IDLE;
	controller.warningStartedAtMs = 0;
	controller.warningTargetMs = 1;
	controller.patternSteps = nullptr;
	controller.patternStepCount = 0;
	controller.patternStepIndex = 0;
	controller.patternStepStartedAtMs = 0;
	feedbackStop(controller);
}

static inline void feedbackStartLongPressWarning(FeedbackController &controller, uint32_t startedAtMs, uint32_t targetMs) {
	controller.mode = FEEDBACK_MODE_WARNING;
	controller.warningStartedAtMs = startedAtMs;
	controller.warningTargetMs = targetMs > 0 ? targetMs : 1;
	controller.patternSteps = nullptr;
	controller.patternStepCount = 0;
	controller.patternStepIndex = 0;
	controller.patternStepStartedAtMs = 0;
}

static inline void feedbackPlayPattern(FeedbackController &controller, const FeedbackStep *steps, uint8_t stepCount) {
	if (steps == nullptr || stepCount == 0) {
		feedbackStop(controller);
		return;
	}

	controller.mode = FEEDBACK_MODE_PATTERN;
	controller.patternSteps = steps;
	controller.patternStepCount = stepCount;
	controller.patternStepIndex = 0;
	controller.patternStepStartedAtMs = millis();
	feedbackApplyOutputs(controller, steps[0].ledOn, steps[0].motorMode, steps[0].motorValue);
}

static inline void feedbackPlayLongPressConfirmed(FeedbackController &controller) {
	feedbackPlayPattern(
		controller,
		FEEDBACK_PATTERN_LONG_PRESS_CONFIRMED,
		sizeof(FEEDBACK_PATTERN_LONG_PRESS_CONFIRMED) / sizeof(FEEDBACK_PATTERN_LONG_PRESS_CONFIRMED[0]));
}

static inline void feedbackPlayButtonTap(FeedbackController &controller) {
	feedbackPlayPattern(
		controller,
		FEEDBACK_PATTERN_BUTTON_TAP,
		sizeof(FEEDBACK_PATTERN_BUTTON_TAP) / sizeof(FEEDBACK_PATTERN_BUTTON_TAP[0]));
}

static inline void feedbackPlayCallEstablished(FeedbackController &controller) {
	feedbackPlayPattern(
		controller,
		FEEDBACK_PATTERN_CALL_ESTABLISHED,
		sizeof(FEEDBACK_PATTERN_CALL_ESTABLISHED) / sizeof(FEEDBACK_PATTERN_CALL_ESTABLISHED[0]));
}

static inline void feedbackPlayAckOnly(FeedbackController &controller) {
	feedbackPlayPattern(
		controller,
		FEEDBACK_PATTERN_ACK_ONLY,
		sizeof(FEEDBACK_PATTERN_ACK_ONLY) / sizeof(FEEDBACK_PATTERN_ACK_ONLY[0]));
}

static inline void feedbackPlayFailure(FeedbackController &controller) {
	feedbackPlayPattern(
		controller,
		FEEDBACK_PATTERN_FAILURE,
		sizeof(FEEDBACK_PATTERN_FAILURE) / sizeof(FEEDBACK_PATTERN_FAILURE[0]));
}

static inline void feedbackPlayBatteryLow(FeedbackController &controller) {
	feedbackPlayPattern(
		controller,
		FEEDBACK_PATTERN_BATTERY_LOW,
		sizeof(FEEDBACK_PATTERN_BATTERY_LOW) / sizeof(FEEDBACK_PATTERN_BATTERY_LOW[0]));
}

static inline void feedbackPlayBatteryCritical(FeedbackController &controller) {
	feedbackPlayPattern(
		controller,
		FEEDBACK_PATTERN_BATTERY_CRITICAL,
		sizeof(FEEDBACK_PATTERN_BATTERY_CRITICAL) / sizeof(FEEDBACK_PATTERN_BATTERY_CRITICAL[0]));
}

static inline bool feedbackBusy(const FeedbackController &controller) {
	return controller.mode == FEEDBACK_MODE_PATTERN;
}

static inline void feedbackUpdate(FeedbackController &controller, uint32_t nowMs) {
	if (controller.mode == FEEDBACK_MODE_IDLE) {
		return;
	}

	if (controller.mode == FEEDBACK_MODE_WARNING) {
		feedbackApplyOutputs(controller, false, FEEDBACK_MOTOR_OFF, 0);
		const uint32_t elapsedMs = nowMs - controller.warningStartedAtMs;
		if (elapsedMs <= FEEDBACK_WARNING_POST_TAP_SILENT_MS) {
			return;
		}

		const uint32_t preConfirmStartMs = controller.warningTargetMs > FEEDBACK_WARNING_PRE_CONFIRM_SILENT_MS
			? controller.warningTargetMs - FEEDBACK_WARNING_PRE_CONFIRM_SILENT_MS
			: 0;

		if (elapsedMs >= preConfirmStartMs) {
			return;
		}

		if (elapsedMs >= controller.warningTargetMs) {
			feedbackApplyOutputs(controller, false, FEEDBACK_MOTOR_PWM, FEEDBACK_WARNING_MAX_PWM);
			return;
		}

		const uint32_t rampElapsedMs = elapsedMs - FEEDBACK_WARNING_POST_TAP_SILENT_MS;
		const uint32_t rampEndMs = preConfirmStartMs > FEEDBACK_WARNING_POST_TAP_SILENT_MS ? preConfirmStartMs : FEEDBACK_WARNING_POST_TAP_SILENT_MS;
		const uint32_t rampDurationMs = rampEndMs > FEEDBACK_WARNING_POST_TAP_SILENT_MS ? rampEndMs - FEEDBACK_WARNING_POST_TAP_SILENT_MS : 1;
		const int32_t pwmLevel = FEEDBACK_WARNING_MIN_PWM + ((int32_t)(FEEDBACK_WARNING_MAX_PWM - FEEDBACK_WARNING_MIN_PWM) * (int32_t)rampElapsedMs) / (int32_t)rampDurationMs;
		feedbackApplyOutputs(controller, false, FEEDBACK_MOTOR_PWM, (uint8_t)constrain(pwmLevel, FEEDBACK_WARNING_MIN_PWM, FEEDBACK_WARNING_MAX_PWM));
		return;
	}

	if (controller.mode != FEEDBACK_MODE_PATTERN || controller.patternSteps == nullptr || controller.patternStepCount == 0) {
		feedbackStop(controller);
		return;
	}

	const FeedbackStep &step = controller.patternSteps[controller.patternStepIndex];
	if ((uint32_t)(nowMs - controller.patternStepStartedAtMs) < step.durationMs) {
		return;
	}

	controller.patternStepIndex++;
	if (controller.patternStepIndex >= controller.patternStepCount) {
		feedbackStop(controller);
		return;
	}

	controller.patternStepStartedAtMs = nowMs;
	const FeedbackStep &nextStep = controller.patternSteps[controller.patternStepIndex];
	feedbackApplyOutputs(controller, nextStep.ledOn, nextStep.motorMode, nextStep.motorValue);
}