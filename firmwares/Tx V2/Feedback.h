#pragma once

#include <Arduino.h>

// Ramp motor PWM from fromPwm to toPwm linearly over durationMs (5ms steps).
// Uses int16_t params so decrescendo (high→low) math doesn't underflow.
static inline void motorRamp(uint32_t motorPin, int16_t fromPwm, int16_t toPwm, uint32_t durationMs) {
	uint32_t steps = durationMs / 5;
	if (steps < 1) {
		analogWrite(motorPin, (uint8_t)constrain(toPwm, 0, 255));
		return;
	}
	for (uint32_t i = 0; i <= steps; i++) {
		int32_t level = fromPwm + ((int32_t)(toPwm - fromPwm) * (int32_t)i) / (int32_t)steps;
		analogWrite(motorPin, (uint8_t)constrain(level, 0, 255));
		delay(5);
	}
}

inline void feedbackIdle(uint32_t ledPin, uint32_t motorPin) {
	digitalWrite(ledPin, LOW);
	// analogWrite keeps the nRF52 PWM peripheral clocked even at 0%.
	// digitalWrite fully disables PWM on the pin and returns it to GPIO.
	digitalWrite(motorPin, LOW);
}

// Button press: LED on + motor decrescendo 255→0 over 100ms
// Feels like a solid tap that melts away
inline void feedbackButtonDetected(uint32_t ledPin, uint32_t motorPin) {
	digitalWrite(ledPin, HIGH);
	motorRamp(motorPin, 255, 0, 100);
	digitalWrite(ledPin, LOW);
}

// Long-press hold progress: gentle crescendo while user keeps holding.
// This runs continuously in the WAIT_LONG_PRESS state and has no blocking delays.
inline void feedbackLongPressProgress(uint32_t ledPin, uint32_t motorPin, uint32_t elapsedMs, uint32_t targetMs) {
	digitalWrite(ledPin, LOW);
	if (elapsedMs <= 1000) {
		analogWrite(motorPin, 0);
		return;
	}
	if (elapsedMs >= targetMs) {
		analogWrite(motorPin, 210);
		return;
	}
	uint32_t rampElapsedMs = elapsedMs - 1000;
	uint32_t rampDurationMs = targetMs > 1000 ? targetMs - 1000 : 1;
	int32_t level = 20 + ((int32_t)190 * (int32_t)rampElapsedMs) / (int32_t)rampDurationMs;
	analogWrite(motorPin, (uint8_t)constrain(level, 20, 210));
}

// Long press confirmed: short dot confirmation right at threshold.
// User can still cancel up to this exact moment by releasing early.
inline void feedbackLongPressConfirmed(uint32_t ledPin, uint32_t motorPin) {
	analogWrite(motorPin, 0);
	digitalWrite(ledPin, LOW);
	delay(180);
	digitalWrite(ledPin, HIGH);
	analogWrite(motorPin, 255);
	delay(70);
	motorRamp(motorPin, 255, 0, 70);
	digitalWrite(ledPin, LOW);
}

// ACK received: calm LED-only double blink — "you've been heard"
inline void feedbackAckReceived(uint32_t ledPin) {
	digitalWrite(ledPin, HIGH);
	delay(200);
	digitalWrite(ledPin, LOW);
	delay(150);
	digitalWrite(ledPin, HIGH);
	delay(200);
	digitalWrite(ledPin, LOW);
}

// Call confirmed: calming diminuendo  O>  0>  0>
// Each pulse softer than the last, fading into peace
inline void feedbackCallEstablished(uint32_t ledPin, uint32_t motorPin) {
	// O> big pulse with slow fade
	digitalWrite(ledPin, HIGH);
	analogWrite(motorPin, 255);
	delay(220);
	digitalWrite(ledPin, LOW);
	motorRamp(motorPin, 230, 0, 180);
	delay(220);
	// 0> medium pulse
	digitalWrite(ledPin, HIGH);
	analogWrite(motorPin, 170);
	delay(180);
	digitalWrite(ledPin, LOW);
	motorRamp(motorPin, 155, 0, 150);
	delay(220);
	// 0> soft pulse
	digitalWrite(ledPin, HIGH);
	analogWrite(motorPin, 100);
	delay(150);
	digitalWrite(ledPin, LOW);
	motorRamp(motorPin, 90, 0, 130);
}

// ACK received but CALL never came: two medium equal pulses
inline void feedbackAckOnly(uint32_t ledPin, uint32_t motorPin) {
	analogWrite(motorPin, 200);
	delay(130);
	motorRamp(motorPin, 200, 0, 90);
	delay(130);
	analogWrite(motorPin, 200);
	delay(130);
	motorRamp(motorPin, 200, 0, 90);
}

inline void feedbackFailure(uint32_t ledPin, uint32_t motorPin) {
	analogWrite(motorPin, 255);
	delay(300);
	motorRamp(motorPin, 255, 0, 120);
}