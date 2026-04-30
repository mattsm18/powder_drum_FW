/*
Title: AS5600
Author: Matthew Smith
Date: 29/04/26
Purpose:
- Wrap AS5600 functionality into a managed class library

*/

#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

class AS5600 {
    public:
        //*** Constructor ***//
        AS5600(const uint8_t I2C_ADDRESS = 0x36, TwoWire &wire = Wire);

        //*** Accessors ***//
        uint16_t getRawAngle();
        float getAngleDegrees();
        float getAngleRadians();
        float getAngularVelocity();

        //*** Queries ***//
        bool isMagnetDetected();
        
        //*** Update ***/
        void update();
        
    private:

        //*** Internal Attributes ***/

        // I2C
        TwoWire *wire = nullptr;
        uint8_t I2C_ADDRESS = 0x36;

        // Angular velocity calculation attributes
        unsigned long _lastMicros = 0;
        float _lastAngle = 0;
        float _angularVelocity = 0;
        float _lowPassAlpha = 0.5f;

        //*** Register accessing functions ***//
        uint8_t readRegister(uint8_t reg);
        uint16_t readRegister16(uint8_t reg);

        
};

#endif