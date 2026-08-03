#include "AS5600.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
AS5600::AS5600(uint8_t address, TwoWire& wire, uint32_t velocityWindow_us)
{
    _address = address;
    _wire = &wire;
    _velocityWindow_us = velocityWindow_us;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Initialise sensor
bool AS5600::begin()
{
    _wire->begin();
    configureFilters(SlowFilter::X8, FastFilterThreshold::Disabled);
    reset();
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Configure internal digital filters
void AS5600::configureFilters(SlowFilter slow, FastFilterThreshold fast)
{
    uint16_t conf = read16(REG_CONF);

    conf &= ~((0x07 << 10) | (0x03 << 8));
    conf |= (static_cast<uint16_t>(fast) << 10);
    conf |= (static_cast<uint16_t>(slow) << 8);

    write16(REG_CONF, conf);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool AS5600::isMagnetDetected() { return read8(REG_STATUS) & (1 << 5); }

uint16_t AS5600::getRawAngle()      { return read16(REG_RAW_ANGLE) & 0x0FFF; }
uint16_t AS5600::getFilteredAngle() { return read16(REG_ANGLE) & 0x0FFF; }

float AS5600::getAngleDegrees() { return -(float)getFilteredAngle() * DEG_PER_COUNT; }
float AS5600::getAngleRadians() { return -(float)getFilteredAngle() * RAD_PER_COUNT; }

float AS5600::getAngularVelocityRadS() { return _angularVelocityRadS; }
float AS5600::getAngularVelocityDegS() { return _angularVelocityRadS * (180.0f / PI); }

///////////////////////////////////////////////////////////////////////////////////////////////////
// Update velocity estimate.
//
// Call this as often as you want (every tick, every loop) — it's cheap and it
// unwraps the angle on EVERY call, so wraparound is always handled correctly
// no matter how wide the velocity window is.
//
// The actual rad/s number is only recomputed once _velocityWindow_us has
// elapsed, using the SUM of the small per-call deltas over that window and
// the ACTUAL measured elapsed time (not an assumed dt). Widening the window
// trades latency for a much better signal-to-noise ratio on the estimate,
// since the ±0.5 count quantization error stays fixed while the accumulated
// count delta grows with the window.
void AS5600::update()
{
    uint16_t counts = getFilteredAngle();
    uint32_t now = micros(); // captured immediately after the read completes

    if (!_velocityInit)
    {
        _prevCounts               = counts;
        _windowStartTimestamp_us  = now;
        _accumCounts               = 0;
        _velocityInit              = true;
        _angularVelocityRadS       = 0.0f;
        return;
    }

    // Per-call delta with wraparound correction — always safe, independent of window width
    int32_t deltaCounts = (int32_t)counts - (int32_t)_prevCounts;
    if (deltaCounts >  2048) deltaCounts -= 4096;
    if (deltaCounts < -2048) deltaCounts += 4096;

    _accumCounts += deltaCounts;
    _prevCounts   = counts;

    uint32_t elapsed = now - _windowStartTimestamp_us;
    if (elapsed < _velocityWindow_us) return; // still accumulating — hold last estimate

    float dt = elapsed * 1e-6f;
    if (dt > 0.0f) {
        _angularVelocityRadS = -((float)_accumCounts * RAD_PER_COUNT) / dt;
    }

    // Reset window
    _accumCounts              = 0;
    _windowStartTimestamp_us  = now;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void AS5600::reset()
{
    _velocityInit             = false;
    _accumCounts               = 0;
    _windowStartTimestamp_us  = 0;
    _angularVelocityRadS       = 0.0f;
}

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