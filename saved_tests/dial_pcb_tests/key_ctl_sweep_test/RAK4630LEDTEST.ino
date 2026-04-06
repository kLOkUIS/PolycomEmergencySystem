// KEY_CTL Switch Test
// Activates each KEY_CTL line one at a time.
// LED_DIAL_CTL lights up the exact moment the key line goes HIGH.
// LED_RX blinks once for Key1, twice for Key2, ... six times for Key6.
// Each key stays active for 600ms, then all lines go LOW for 1s before the next key.

static const uint32_t LED_PIN          = P0_24;
static const uint32_t LED_DIAL_CTL_PIN = P0_31;

static const uint32_t KEY_PINS[6] = {
    P1_01,  // Key1_CTL  (Back-Up)
    P1_02,  // Key2_CTL  (1)
    P1_03,  // Key3_CTL  (Talk/End)
    P1_04,  // Key4_CTL  (Vol+)
    P0_05,  // Key5_CTL  (2)
    P0_04   // Key6_CTL  (3)
};

void blinkLED(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(LED_DIAL_CTL_PIN, OUTPUT);
    digitalWrite(LED_DIAL_CTL_PIN, LOW);

    for (int i = 0; i < 6; i++) {
        pinMode(KEY_PINS[i], OUTPUT);
        digitalWrite(KEY_PINS[i], LOW);
    }
}

void loop() {
    for (int key = 0; key < 6; key++) {
        // Activate this key line; light up LED_DIAL_CTL at the same moment
        digitalWrite(LED_DIAL_CTL_PIN, HIGH);
        digitalWrite(KEY_PINS[key], HIGH);
        delay(600);
        digitalWrite(KEY_PINS[key], LOW);
        digitalWrite(LED_DIAL_CTL_PIN, LOW);

        // Blink LED (key + 1) times to indicate which key was activated
        blinkLED(key + 1);

        // Pause before the next key
        delay(1000);
    }
}
