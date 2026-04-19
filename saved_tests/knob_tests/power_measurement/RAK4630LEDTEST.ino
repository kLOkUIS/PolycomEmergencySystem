/***
 *  This example shows powersave function.
***/

static const uint32_t WAKEUP_PIN = P1_01;

void WakeupCallback()
{
    Serial.printf("This is Wakeup Callback\r\n");
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("RAKwireless System Powersave Example");
    Serial.println("------------------------------------------------------");

    api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, WAKEUP_PIN);

    if (api.system.sleep.registerWakeupCallback(WakeupCallback) == false)
    {
        Serial.println("Create Wakeup Callback failed.");
    }
}

void loop()
{
    Serial.print("The timestamp before sleeping: ");
    Serial.print(millis());
    Serial.println(" ms");
    Serial.println("(Wait 10 seconds or press button to wakeup)");
    api.system.sleep.all(10000);
    Serial.print("The timestamp after sleeping: ");
    Serial.print(millis());
    Serial.println(" ms");
}
