#include "AS5600.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////////////////////////////////////////////

AS5600::AS5600(uint8_t address, AsyncTWI& twi, uint32_t velocityWindow_us)
{
    _address = address;
    _twi = &twi;
    _velocityWindow_us = velocityWindow_us;

    reset();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Initialise sensor
///////////////////////////////////////////////////////////////////////////////////////////////////

bool AS5600::begin()
{
    reset();
    _ready = true;

    // Request the default filter configuration.
    configureFilters(SlowFilter::X8, FastFilterThreshold::Disabled);

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Main asynchronous update
///////////////////////////////////////////////////////////////////////////////////////////////////

void AS5600::update()
{
    // Advance the TWI state machine.
    _twi->update();

    // Process a completed transaction.
    processTransaction();

    // If the TWI driver is available, start the next operation.
    startNextTransaction();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Start the next required transaction
///////////////////////////////////////////////////////////////////////////////////////////////////

void AS5600::startNextTransaction()
{
    if (!_ready){ return; }
    if (_transaction != Transaction::NONE){ return; }
    if (_twi->isBusy()){ return; }

    // Filter configuration requested
    if (_configureFiltersRequested)
    {
        _transaction = Transaction::READ_CONF;

        if (!_twi->readRegisters(_address, REG_CONF, _confBuffer, 2))
        {
            _transaction = Transaction::NONE;
        }

        return;
    }

    // Normal operation, Read angle continuously.
    _transaction = Transaction::READ_ANGLE;

    if (!_twi->readRegisters(_address, REG_ANGLE, _angleBuffer, 2))
    {
        _transaction = Transaction::NONE;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Process completed asynchronous transaction
///////////////////////////////////////////////////////////////////////////////////////////////////

void AS5600::processTransaction()
{
    if (_transaction == Transaction::NONE){ return; }
    if (!_twi->isDone()){ return; }
    _lastResult = _twi->getResult();

    // Transaction failed
    if (_lastResult != AsyncTWI::Result::OK)
    {
        _validData = false;
        _transaction = Transaction::NONE;
        _twi->clearDone();
        return;
    }

    // Read angle
    if (_transaction == Transaction::READ_ANGLE)
    {
        _rawAngle =(static_cast<uint16_t>(_angleBuffer[0]) << 8) | _angleBuffer[1];
        _filteredAngle = _rawAngle & 0x0FFF;
        _validData = true;

        processAngle(_filteredAngle);
    }

    // Read configuration
    else if (_transaction == Transaction::READ_CONF)
    {
        uint16_t conf =(static_cast<uint16_t>(_confBuffer[0]) << 8) | _confBuffer[1];

        conf = buildFilterConfig(conf, _requestedSlowFilter, _requestedFastFilter);
        _writeConfBuffer[0] = static_cast<uint8_t>(conf >> 8);
        _writeConfBuffer[1] = static_cast<uint8_t>(conf & 0xFF);
        _transaction = Transaction::WRITE_CONF;

        _twi->clearDone();

        if (!_twi->writeRegisters(_address, REG_CONF, _writeConfBuffer, 2))
        {
            _transaction = Transaction::NONE;
        }

        return;
    }

    // Write configuration
    else if (_transaction == Transaction::WRITE_CONF){ _configureFiltersRequested = false; }
    _transaction = Transaction::NONE;
    _twi->clearDone();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Configure AS5600 internal filters
///////////////////////////////////////////////////////////////////////////////////////////////////

void AS5600::configureFilters(SlowFilter slow, FastFilterThreshold fast)
{
    _requestedSlowFilter = slow;
    _requestedFastFilter = fast;

    _configureFiltersRequested = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Build new CONF register value
///////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t AS5600::buildFilterConfig(uint16_t conf, SlowFilter slow, FastFilterThreshold fast) const
{
    // Slow filter bits: 9:8
    conf &= ~(0x03 << 8);

    // Fast filter threshold bits: 11:10
    conf &= ~(0x03 << 10);
    conf |= static_cast<uint16_t>(slow) << 8;
    conf |= static_cast<uint16_t>(fast) << 10;

    return conf;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Process angle and calculate velocity
///////////////////////////////////////////////////////////////////////////////////////////////////

void AS5600::processAngle(uint16_t counts)
{
    uint32_t now = micros();

    // First sample
    if (!_velocityInit)
    {
        _prevCounts = counts;
        _windowStartTimestamp_us = now;
        _accumCounts = 0;
        _velocityInit = true;
        _angularVelocityRadS = 0.0f;
        return;
    }

    // Calculate wrapped delta
    int32_t deltaCounts =
        static_cast<int32_t>(counts) -
        static_cast<int32_t>(_prevCounts);

    // Handle wraparound
    if (deltaCounts > 2048){ deltaCounts -= 4096; } //cw
    if (deltaCounts < -2048){ deltaCounts += 4096; } //ccw

    // Accumulate movement
    _accumCounts += deltaCounts;
    _prevCounts = counts;

    // Check velocity window
    uint32_t elapsed = now - _windowStartTimestamp_us;
    if (elapsed < _velocityWindow_us){ return; }

    // Calculate velocity using actual measured time
    float dt = elapsed * 1e-6f;

    if (dt > 0.0f)
    {
        _angularVelocityRadS = -(static_cast<float>(_accumCounts) * RAD_PER_COUNT) / dt;
    }

    // Reset velocity window
    _accumCounts = 0;
    _windowStartTimestamp_us = now;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Public API
///////////////////////////////////////////////////////////////////////////////////////////////////

bool AS5600::isReady()                   const { return _ready; }
bool AS5600::hasValidData()              const { return _validData;}

AsyncTWI::Result AS5600::getLastResult() const { return _lastResult;}
uint16_t AS5600::getRawAngle()           const { return _rawAngle; }
uint16_t AS5600::getFilteredAngle()      const { return _filteredAngle; }

float AS5600::getAngleDegrees()          const { return -static_cast<float>(_filteredAngle) * DEG_PER_COUNT; }
float AS5600::getAngleRadians()          const { return -static_cast<float>(_filteredAngle) * RAD_PER_COUNT; }
float AS5600::getAngularVelocityRadS()   const { return _angularVelocityRadS; }
float AS5600::getAngularVelocityDegS()   const { return _angularVelocityRadS * (180.0f / PI); }

///////////////////////////////////////////////////////////////////////////////////////////////////
// Reset
///////////////////////////////////////////////////////////////////////////////////////////////////

void AS5600::reset()
{
    _ready = false;
    _validData = false;
    _transaction = Transaction::NONE;
    _lastResult = AsyncTWI::Result::NONE;
    _rawAngle = 0;
    _filteredAngle = 0;
    _velocityInit = false;
    _prevCounts = 0;
    _accumCounts = 0;
    _windowStartTimestamp_us = 0; 
    _angularVelocityRadS = 0.0f;
    _configureFiltersRequested = false;
}