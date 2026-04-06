// CALL_SENSE Activity Test (diagnostic)
// LED_RX shows debounced CALL_SENSE level (ON when LOW/active).
// LED_DIAL_CTL flashes briefly on each detected edge.
// This makes it obvious whether the line is stuck or actually toggling.

static const uint32_t LED_RX_PIN       = P0_24;
static const uint32_t LED_DIAL_CTL_PIN = P0_31;
static const uint32_t CALL_SENSE_PIN   = P0_25;

static const unsigned long EDGE_FLASH_MS    = 180;
static const unsigned long DEBOUNCE_MS      = 8;
static const unsigned long LOOP_DELAY_MS    = 2;

unsigned long edgeFlashUntilMs = 0;
unsigned long rawChangedAtMs = 0;
bool lastRawState = true;
bool debouncedState = true;

void allKeysLow() {
    pinMode(P1_01, OUTPUT);
    digitalWrite(P1_01, LOW);
    pinMode(P1_02, OUTPUT);
    digitalWrite(P1_02, LOW);
    pinMode(P1_03, OUTPUT);
    digitalWrite(P1_03, LOW);
    pinMode(P1_04, OUTPUT);
    digitalWrite(P1_04, LOW);
    pinMode(P0_05, OUTPUT);
    digitalWrite(P0_05, LOW);
    pinMode(P0_04, OUTPUT);
    digitalWrite(P0_04, LOW);
}

bool callSenseRawState() {
    return digitalRead(CALL_SENSE_PIN) == HIGH;
}

void setup() {
    pinMode(LED_RX_PIN, OUTPUT);
    digitalWrite(LED_RX_PIN, LOW);

    pinMode(LED_DIAL_CTL_PIN, OUTPUT);
    digitalWrite(LED_DIAL_CTL_PIN, LOW);

    pinMode(CALL_SENSE_PIN, INPUT);

    allKeysLow();

    lastRawState = callSenseRawState();
    debouncedState = lastRawState;
    rawChangedAtMs = millis();
}

void loop() {
    const bool rawState = callSenseRawState();
    const unsigned long nowMs = millis();

    if (rawState != lastRawState) {
        lastRawState = rawState;
        rawChangedAtMs = nowMs;
    }

    // Debounce before accepting a new stable level.
    if ((nowMs - rawChangedAtMs) >= DEBOUNCE_MS && debouncedState != rawState) {
        debouncedState = rawState;
        edgeFlashUntilMs = nowMs + EDGE_FLASH_MS;
    }

    // LOW on CALL_SENSE means active.
    const bool callActive = !debouncedState;
    digitalWrite(LED_RX_PIN, callActive ? HIGH : LOW);

    // Flash activity LED only on detected edges.
    const bool edgeFlashActive = (long)(nowMs - edgeFlashUntilMs) < 0;
    digitalWrite(LED_DIAL_CTL_PIN, edgeFlashActive ? HIGH : LOW);

    delay(LOOP_DELAY_MS);
}
