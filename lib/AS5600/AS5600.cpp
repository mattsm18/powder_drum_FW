#include "AS5600.h"

///////////////////////////////////////
// Constructor
//
AS5600::AS5600
( 
    uint8_t address, 
    TwoWire &wirePort
)
{
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

///////////////////////////////////////
// Public API
//

// Accessors
uint16_t AS5600::readAngleReg()         { return readRegister16(0x0C); }
uint16_t AS5600::readFilteredAngleReg() { return readRegister16(0x0E); }
float AS5600::getAngleDegrees()         { return readFilteredAngleReg() * 360.0 / 4096.0; }
float AS5600::getAngleRadians()         { return readFilteredAngleReg() * (2.0f * M_PI) / 4096.0f; }