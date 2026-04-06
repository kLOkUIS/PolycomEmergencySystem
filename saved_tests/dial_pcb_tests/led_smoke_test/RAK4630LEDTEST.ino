// Dial PCB LED smoke test
// Purpose: verify MCU is alive and both board LEDs are connected correctly.

static const uint32_t LED_RX_PIN = P0_24;
static const uint32_t LED_DIAL_CTL_PIN = P0_31;

// KEY control lines go to TMUX1511 control inputs.
// Keep all LOW so no emulated key is pressed during boot or test runtime.
static const uint32_t KEY1_CTL_PIN = P1_01;
static const uint32_t KEY2_CTL_PIN = P1_02;
static const uint32_t KEY3_CTL_PIN = P1_03;
static const uint32_t KEY4_CTL_PIN = P1_04;
static const uint32_t KEY5_CTL_PIN = P0_05;
static const uint32_t KEY6_CTL_PIN = P0_04;

static void allLedsOff() {
	digitalWrite(LED_RX_PIN, LOW);
	digitalWrite(LED_DIAL_CTL_PIN, LOW);
}

static void setupKeySafetyDefaults() {
	pinMode(KEY1_CTL_PIN, OUTPUT);
	pinMode(KEY2_CTL_PIN, OUTPUT);
	pinMode(KEY3_CTL_PIN, OUTPUT);
	pinMode(KEY4_CTL_PIN, OUTPUT);
	pinMode(KEY5_CTL_PIN, OUTPUT);
	pinMode(KEY6_CTL_PIN, OUTPUT);

	digitalWrite(KEY1_CTL_PIN, LOW);
	digitalWrite(KEY2_CTL_PIN, LOW);
	digitalWrite(KEY3_CTL_PIN, LOW);
	digitalWrite(KEY4_CTL_PIN, LOW);
	digitalWrite(KEY5_CTL_PIN, LOW);
	digitalWrite(KEY6_CTL_PIN, LOW);
}

void setup() {
	pinMode(LED_RX_PIN, OUTPUT);
	pinMode(LED_DIAL_CTL_PIN, OUTPUT);
	allLedsOff();

	setupKeySafetyDefaults();
}

void loop() {
	// Step 1: LED_RX only
	digitalWrite(LED_RX_PIN, HIGH);
	digitalWrite(LED_DIAL_CTL_PIN, LOW);
	delay(400);

	// Step 2: LED_DIAL_CTL only
	digitalWrite(LED_RX_PIN, LOW);
	digitalWrite(LED_DIAL_CTL_PIN, HIGH);
	delay(400);

	// Step 3: both on
	digitalWrite(LED_RX_PIN, HIGH);
	digitalWrite(LED_DIAL_CTL_PIN, HIGH);
	delay(400);

	// Step 4: both off
	allLedsOff();
	delay(400);
}
