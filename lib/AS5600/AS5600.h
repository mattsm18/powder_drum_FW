/*
Title: AS5600
Author: Matthew Smith
Date: 29/04/26
Purpose:
- Wrap AS5600 hardware function into a simple class library
- Provide Public API for easy access of encoder measurements
*/

#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

class AS5600 {
    public:

        //*** Constructor ***//
        AS5600
        (
            uint8_t address = 0x36, 
            TwoWire &wire = Wire
        );

        //*** Accessors ***//
        uint16_t readAngleReg();
        uint16_t readFilteredAngleReg();
        float getAngleDegrees();
        float getAngleRadians();
        
    private:

        //*** Internal Attributes ***/

        // I2C
        TwoWire *wire;
        uint8_t I2C_ADDRESS;

        //*** Register accessing functions ***//
        uint8_t readRegister(uint8_t reg);
        uint16_t readRegister16(uint8_t reg);
        
};

#endif