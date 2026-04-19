#pragma once

#include <Arduino.h>

namespace FeedbackTuning {

// Press classification threshold from initial physical press to release.
static constexpr uint32_t SHORT_PRESS_MS = 100;
static constexpr uint32_t LONG_PRESS_TOTAL_MS = 3000;

// Warning (hold) profile from the moment tap feedback has completed.
// Timeline:
// 1) post-tap silence
// 2) PWM ramp
// 3) pre-confirm silence
// 4) long-press confirmed immediately
static constexpr uint32_t WARNING_POST_TAP_SILENT_MS = 500;
static constexpr uint32_t WARNING_PRE_CONFIRM_SILENT_MS = 50;
static constexpr uint32_t WARNING_RAMP_MS =
	LONG_PRESS_TOTAL_MS - WARNING_POST_TAP_SILENT_MS - WARNING_PRE_CONFIRM_SILENT_MS;
static constexpr uint8_t WARNING_MIN_PWM = 60;
static constexpr uint8_t WARNING_MAX_PWM = 240;

// Derived hold threshold after tap completion.
static constexpr uint32_t LONG_PRESS_MS = LONG_PRESS_TOTAL_MS;

// Button tap cue.
static constexpr uint16_t BUTTON_TAP_ON_MS = 120;

// Long-press confirmation cue.
// Any pre-confirmation silence should be represented in the warning hold profile,
// not after the long-press threshold has already been reached.
static constexpr uint16_t LONG_PRESS_CONFIRM_SILENCE_MS = 180;
static constexpr uint16_t LONG_PRESS_CONFIRM_PULSE_MS = 140;

// In-call confirmed cue (O0o).
static constexpr uint16_t CALL_ESTABLISHED_PULSE1_MS = 120;
static constexpr uint16_t CALL_ESTABLISHED_GAP1_MS = 130;
static constexpr uint16_t CALL_ESTABLISHED_PULSE2_MS = 100;
static constexpr uint16_t CALL_ESTABLISHED_GAP2_MS = 130;
static constexpr uint16_t CALL_ESTABLISHED_PULSE3_MS = 80;

// ACK-only cue (double pulse).
static constexpr uint16_t ACK_ONLY_PULSE1_MS = 90;
static constexpr uint16_t ACK_ONLY_GAP_MS = 130;
static constexpr uint16_t ACK_ONLY_PULSE2_MS = 90;

// Failure cue (single strong pulse).
static constexpr uint16_t FAILURE_PULSE_MS = 220;

// Battery service policy.
static constexpr float BATTERY_LOW_V = 2.80f;
static constexpr float BATTERY_CRITICAL_V = 2.75f;
static constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL;
static constexpr uint32_t BATTERY_ALERT_REMINDER_INTERVAL_MS = 60UL * 1000UL;
static constexpr uint32_t BATTERY_CRITICAL_REMINDER_INTERVAL_MS = 15UL * 1000UL;       // CRITICAL reminder. TEST (prod: 15UL * 1000UL)
static constexpr uint32_t BATTERY_ALERT_RECHECK_INTERVAL_MS = 60UL * 60UL * 1000UL;
static constexpr uint8_t BATTERY_SAMPLE_COUNT = 3;
static constexpr uint32_t BATTERY_SAMPLE_SPACING_MS = 2;
static constexpr uint32_t BATTERY_SAMPLE_SETTLE_MS = 5;

// Battery LOW pattern: 2 slow prominent blinks, LED only.
static constexpr uint16_t BATTERY_LOW_ON_MS = 80;
static constexpr uint16_t BATTERY_LOW_OFF_MS = 40;
static constexpr uint16_t BATTERY_LOW_GAP_MS = 150;

// Battery CRITICAL pattern: 3 groups of 3 rapid LED+motor pulses.
static constexpr uint16_t BATTERY_CRITICAL_PULSE_ON_MS = 45;
static constexpr uint16_t BATTERY_CRITICAL_PULSE_OFF_MS = 25;  // within-group gap, motor stays on
static constexpr uint16_t BATTERY_CRITICAL_GROUP_GAP_MS = 120; // between-group pause, motor off

static_assert(WARNING_MAX_PWM > WARNING_MIN_PWM, "WARNING_MAX_PWM must be greater than WARNING_MIN_PWM");
static_assert(
	LONG_PRESS_TOTAL_MS > WARNING_POST_TAP_SILENT_MS + WARNING_PRE_CONFIRM_SILENT_MS,
	"LONG_PRESS_TOTAL_MS must be greater than WARNING_POST_TAP_SILENT_MS + WARNING_PRE_CONFIRM_SILENT_MS");
static_assert(LONG_PRESS_MS > SHORT_PRESS_MS, "LONG_PRESS_MS must be greater than SHORT_PRESS_MS");
static_assert(BATTERY_LOW_V > BATTERY_CRITICAL_V, "BATTERY_LOW_V must be greater than BATTERY_CRITICAL_V");
static_assert(BATTERY_SAMPLE_COUNT >= 3, "BATTERY_SAMPLE_COUNT must be at least 3");

} // namespace FeedbackTuning
