/*
Title: StepperMotor
Author: Matthew Smith
Date: 1/05/26
Purpose:
- Wrap StepperMotor driving functionality into a class
- Provide smooth, time-accurate stepping and speed control
- Allow dynamic microstepping rates

*/

#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>

// Enumerations
enum Direction { CW = 0, CCW = 1 };
enum Microstep { FULL = 1, HALF = 2, QUARTER = 4, EIGHTH = 8, SIXTEENTH = 16, THIRTY_SECOND = 32 };

// Class definition
class StepperMotor {
public:
    //*** Constructor ***/
    StepperMotor
    (
        uint8_t stepPin, 
        uint8_t dirPin,
        Microstep microstep = FULL,
        uint8_t stepsPerRev = 200,
        uint32_t isrFreq = 20000
    );

    // Accessors
    float getAngularVelocity() const;

    // Methods
    void setAngularVelocity(float radsPerSec);
    static void tick();

private:
    static StepperMotor* _instance;

    // Hardware
    uint8_t _stepPin;
    uint8_t _dirPin;

    // Timing
    const uint16_t _stepsPerRev;
    const uint32_t _isrFreq;

    // Motion
    volatile float _angularVelocity = 0.0f;
    volatile float _phase = 0.0f;
    volatile float _phaseIncrement = 0.0f;

    // Pulse state machine
    volatile bool _stepHigh = false;

    Direction _direction = CW;

    void updateStepTiming();
    void stepIfDue();
};

#endif