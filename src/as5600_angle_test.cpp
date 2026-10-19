#include <Arduino.h>
#include <AS5600.h>

namespace
{
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr uint32_t REPORT_INTERVAL_MS = 250;

AsyncTWI twi;
AS5600 encoder(AS5600_ADDRESS, twi, 40000);
uint32_t lastReportMs = 0;
}

void setup()
{
    Serial.begin(115200);
    twi.begin(100000UL);
    encoder.begin();
    Serial.println(F("AS5600 angle test: reading through AS5600 driver"));
}

void loop()
{
    encoder.update();

    const uint32_t now = millis();
    if (now - lastReportMs >= REPORT_INTERVAL_MS)
    {
        lastReportMs = now;
        Serial.print(F("valid="));
        Serial.print(encoder.hasValidData() ? 1 : 0);
        Serial.print(F(" angle_deg="));
        Serial.print(encoder.getAngleDegrees(), 2);
        Serial.print(F(" raw="));
        Serial.print(encoder.getFilteredAngle());
        Serial.print(F(" result="));
        Serial.print(static_cast<uint8_t>(encoder.getLastResult()));
        Serial.print(F(" err_state="));
        Serial.print(twi.getLastErrorState());
        Serial.print(F(" err_mstatus=0x"));
        Serial.println(twi.getLastErrorStatus(), HEX);
    }
}
