#include "AS5600.h"

///////////////////////////////////////
// Constructor
//
AS5600::AS5600(const uint8_t address, TwoWire &wirePort){
    I2C_ADDRESS = address;
    wire = &wirePort;
    wire->begin();
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
    
    float angle = getAngleRadians();

    // Get dt (change in time) from the difference between now and the last iteration
    unsigned long now = micros();
    float dt = (now - _lastMicros) / 1e6f;
    if (dt <= 0.0001f) return;

    // Check dt is valid
    if(dt > 0){
        
        // Get delta (change in angle)
        float delta = angle - _lastAngle;

        // Handle wraparound (raw angle is 0-4095)
        if(delta > M_PI) delta -= 2.0f * M_PI;
        if(delta < -M_PI) delta += 2.0f * M_PI;

        // Calculate the angular velocity as [ change in angle / change in time ] (rad / s)
        float omega = delta / dt; 

        // SIGNAL DEGRADATION!!!!!
        _angularVelocity = _lowPassAlpha * omega + (1.0f - _lowPassAlpha) * _angularVelocity;

        
    }

    _lastAngle = angle;
    _lastMicros = now;
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