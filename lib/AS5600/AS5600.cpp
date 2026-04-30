#include "AS5600.h"

///////////////////////////////////////
// Constructor
//
AS5600::AS5600(const uint8_t address, TwoWire &wirePort){
    I2C_ADDRESS = address;
    wire = &wirePort;
    wire->begin();
    wire->setClock(400000); //i2c fast mode
}

///////////////////////////////////////
// Read single byte register over i2c
//
uint8_t AS5600::readRegister(uint8_t reg){  
    
    // Transmit register
    wire->beginTransmission(I2C_ADDRESS);
    wire->write(reg);
    wire->endTransmission();
    
    // Request single byte response register
    wire->requestFrom(I2C_ADDRESS, (uint8_t)(1));
    return wire->available() ? wire->read() : 0;
}

///////////////////////////////////////
// Read double byte register over i2c
//
uint16_t AS5600::readRegister16(uint8_t reg){

    // Transmit register
    wire->beginTransmission(I2C_ADDRESS);
    wire->write(reg);
    wire->endTransmission();

    // Request two bytes response register
    wire->requestFrom(I2C_ADDRESS, (uint8_t)(2));
    
    // Stitch two uint8_t into one uint16_t
    uint16_t value = 0;
    if(wire->available() >= 2){
        value = (wire->read() << 8) | wire->read();
    }

    // Return uint_16_t
    return value;
}

void AS5600::update()
{
    static const float TAU = 0.1f;

    uint16_t raw = readRegister16(0x0C);
    
    // Only process fresh data
    if (raw == _lastRaw) return;
    _lastRaw = raw;

    uint32_t now = micros();
    float dt = (now - _lastMicros) * 1e-6f;
    _lastMicros = now;

    float theta = raw * (2.0f * M_PI / 4096.0f);
    float dtheta = theta - _thetaPrev;

    if (dtheta >  M_PI) dtheta -= 2.0f * M_PI;
    if (dtheta < -M_PI) dtheta += 2.0f * M_PI;

    float omegaRaw = dtheta / dt;  // dt is real elapsed time since last fresh read

    float alpha = dt / (TAU + dt);
    _omega = alpha * omegaRaw + (1.0f - alpha) * _omega;
    _angularVelocity = _omega;
    _thetaPrev = theta;
}

///////////////////////////////////////
// Public API
//

// Accessors
uint16_t AS5600::getRawAngle()      { return readRegister16(0x0C);}
float AS5600::getAngleDegrees()     { return getRawAngle() * 360.0 / 4096.0; }
float AS5600::getAngleRadians()     { return getRawAngle() * (2.0f * M_PI) / 4096.0f; }
float AS5600::getAngularVelocity()  { return _angularVelocity; }

// Queries
bool AS5600::isMagnetDetected() { return (readRegister(0x0B) & 0x20) != 0; }