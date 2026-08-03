#include "AS5600.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
AS5600::AS5600(uint8_t address, TwoWire& wire)
{
    _address = address;
    _wire = &wire;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Initialise sensor
bool AS5600::begin()
{
    _wire->begin();
    configureFilters(SlowFilter::X16, FastFilterThreshold::Disabled);
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Configure internal digital filters
void AS5600::configureFilters(SlowFilter slow, FastFilterThreshold fast)
{
    uint16_t conf = read16(REG_CONF);

    // Clear FTH[12:10] and SF[9:8]
    conf &= ~((0x07 << 10) | (0x03 << 8));

    // Set new values
    conf |= (static_cast<uint16_t>(fast) << 10);
    conf |= (static_cast<uint16_t>(slow) << 8);

    write16(REG_CONF, conf);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t AS5600::getRawAngle(){ return read16(REG_RAW_ANGLE) & 0x0FFF; }
uint16_t AS5600::getFilteredAngle(){ return read16(REG_ANGLE) & 0x0FFF; }

float AS5600::getAngleDegrees(){ return -(float)getFilteredAngle() * DEG_PER_COUNT; }
float AS5600::getAngleRadians(){ return -(float)getFilteredAngle() * RAD_PER_COUNT; }

bool AS5600::isMagnetDetected(){ return read8(REG_STATUS) & (1 << 5); }

///////////////////////////////////////////////////////////////////////////////////////////////////
// Read 8-bit register
uint8_t AS5600::read8(uint8_t reg)
{
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission();
    _wire->requestFrom(_address, (uint8_t)1);
    if (_wire->available()) return _wire->read();
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Read 16-bit register
uint16_t AS5600::read16(uint8_t reg)
{
    _wire->beginTransmission(_address);
    _wire->write(reg);

    _wire->endTransmission();
    _wire->requestFrom(_address, (uint8_t)2);
    if (_wire->available() < 2) return 0;

    uint16_t value = _wire->read();
    value <<= 8;
    value |= _wire->read();

    return value;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Write 16-bit register
void AS5600::write16(uint8_t reg, uint16_t value)
{
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value >> 8);
    _wire->write(value & 0xFF);
    _wire->endTransmission();
}