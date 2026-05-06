#include "AS5600.h"

///////////////////////////////////////
// Constructor
//
AS5600::AS5600
( 
    float sampleRate_us, 
    float EMATimeConst,
    uint8_t address, 
    TwoWire &wirePort
)
{
    _sampleRate_us = sampleRate_us;
    _EMATimeConst = EMATimeConst;
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

// Encoder update function, used to measure the angular velocity
// 
// - Called in main() at clock speed
// - Fires at _sampleRate_us which is passed on construction
// 
// omega = delta theta / delta time
// Angular velocity = change in angle / change in time
//
// Applies a light EMA (Exponential Moving Average) Filter to reduce quantisation

void AS5600::update()
{
    // Get angle as filtered angle from AS5600
    uint16_t angle = readFilteredAngleReg();

    // exit on stale data -- arbitrary....
    if (angle == _lastAngle) { return; } 

    // Solve for dt (change in time)
    unsigned long now = micros();
    float dt = (now - _lastMicros) * 1e-6f;
    _lastMicros = now;

    // Solve for d(theta) (change in angle)
    float theta = angle * (2.0f * M_PI / 4096.0f);
    float d_theta = theta - _thetaPrev;

    // Handle wraparound (jump from 4095 -> 0 and 0 -> 4095)
    if (d_theta >  M_PI) d_theta -= 2.0f * M_PI;
    if (d_theta < -M_PI) d_theta += 2.0f * M_PI;

    // calculate Angular Velocity
    float angularVelocityRaw = d_theta / dt;

    // Apply 1st order discrete-time filter (Low-Pass Filter)
    float alpha = dt / (_EMATimeConst + dt); // BAD - remove from loop
    _angularVelocity = alpha * angularVelocityRaw + (1.0f - alpha) * _angularVelocity;
    _thetaPrev = theta;
}

///////////////////////////////////////
// Public API
//

// Accessors
uint16_t AS5600::readAngleReg()         { return readRegister16(0x0C); }
uint16_t AS5600::readFilteredAngleReg() { return readRegister16(0x0E); }
float AS5600::getAngleDegrees()         { return readFilteredAngleReg() * 360.0 / 4096.0; }
float AS5600::getAngleRadians()         { return readFilteredAngleReg() * (2.0f * M_PI) / 4096.0f; }
float AS5600::getAngularVelocity()      { return -_angularVelocity; } // negative direction to match stepper direction convention