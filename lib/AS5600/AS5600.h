/*
Title: AS5600
Author: Matthew Smith
Date: 29/04/26
Purpose:
- Wrap AS5600 functionality into a managed class library
- Estimate Angular Velocity with EMA Filter
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
            float sampleRate_us = 1000,
            float EMATimeConst = 0.02,
            uint8_t I2C_ADDRESS = 0x36, 
            TwoWire &wire = Wire
        );

        //*** Accessors ***//
        uint16_t readAngleReg();
        uint16_t readFilteredAngleReg();
        float getAngleDegrees();
        float getAngleRadians();
        float getAngularVelocity();
        float getFiltered();
        
        //*** Update ***/
        void update();
        
    private:

        //*** Internal Attributes ***/

        // I2C
        TwoWire *wire;
        uint8_t I2C_ADDRESS;

        // Angular Velocity Measurement and Filtering
        float _sampleRate_us;
        float _EMATimeConst;
        unsigned long _lastMicros;
        float _lastAngle;
        float _thetaPrev;

        // Internal storage
        float _angularVelocity;

        //*** Register accessing functions ***//
        uint8_t readRegister(uint8_t reg);
        uint16_t readRegister16(uint8_t reg);

        
};

#endif