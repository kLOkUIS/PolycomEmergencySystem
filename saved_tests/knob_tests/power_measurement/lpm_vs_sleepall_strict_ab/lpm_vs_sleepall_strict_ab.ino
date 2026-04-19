/***
 *  lpm_vs_sleepall_strict_ab.ino
 *
 *  Strict apples-to-apples A/B test:
 *    0 = sleep.all only
 *    1 = lpm.set(1) + sleep.all
 *
 *  All other behavior is held constant:
 *    - no Serial
 *    - no boot/wake pulses
 *    - no timer wake
 *    - no LoRa API calls
 *    - button wake only
 */

static const uint32_t WAKEUP_PIN = P1_01;
static const uint32_t LED_PIN = P1_03;
static const uint32_t MOTOR_PIN = P1_04;

#ifndef TEST_MODE_LPM_PLUS_SLEEPALL
#define TEST_MODE_LPM_PLUS_SLEEPALL 0
#endif

volatile bool gWakePending = false;

void WakeupCallback()
{
    gWakePending = true;
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);

#if TEST_MODE_LPM_PLUS_SLEEPALL
    api.system.lpm.set(1);
#endif

    api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, WAKEUP_PIN);
    (void)api.system.sleep.registerWakeupCallback(WakeupCallback);
}

void loop()
{
    digitalWrite(LED_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    api.system.sleep.all((uint32_t)0xFFFFFFFF);

    noInterrupts();
    gWakePending = false;
    interrupts();
}
