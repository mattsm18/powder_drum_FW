/*
    AS5600.h

    Lightweight AS5600 driver for closed-loop motor control.

    Features:
    - Raw angle reading
    - Filtered angle reading (degrees/radians)
    - Magnet detection
    - Configurable internal digital filters

    Matthew Smith
*/

#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

class AS5600
{
public:

    // Filter Configuration
    enum class SlowFilter : uint8_t
    {
        X16 = 0b00,   // Lowest noise, highest latency
        X8  = 0b01,
        X4  = 0b10,
        X2  = 0b11    // Lowest latency, highest noise
    };

    /// Fast filter threshold
    enum class FastFilterThreshold : uint8_t
    {
        Disabled = 0b000,
        LSB6     = 0b001,
        LSB7     = 0b010,
        LSB9     = 0b011,
        LSB18    = 0b100,
        LSB21    = 0b101,
        LSB24    = 0b110,
        LSB10    = 0b111
    };

    // Construction
    AS5600(uint8_t address = 0x36, TwoWire& wire = Wire);
    bool begin();

    void configureFilters(SlowFilter slow, FastFilterThreshold fast);

    // Accessors
    uint16_t getRawAngle();
    uint16_t getFilteredAngle();
    float getAngleDegrees();
    float getAngleRadians();

    // Diagnostics
    bool isMagnetDetected();

private:

    // Fast lookups for conversion
    static constexpr float DEG_PER_COUNT = 360.0f / 4096.0f;
    static constexpr float RAD_PER_COUNT = (2.0f * PI) / 4096.0f;

    // Register Map
    static constexpr uint8_t REG_CONF       = 0x07;
    static constexpr uint8_t REG_STATUS     = 0x0B;
    static constexpr uint8_t REG_RAW_ANGLE  = 0x0C;
    static constexpr uint8_t REG_ANGLE      = 0x0E;

    // I2C
    TwoWire* _wire;
    uint8_t _address;

    // Helpers
    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    void write16(uint8_t reg, uint16_t value);
};

#endif