/*
Title: PIController
Author: Matthew Smith
Date: 29/04/26
Purpose:
- Handle pure PI calculations

*/

#include "PIController.h"

// Bumpless transfer
// 

PIController::PIController
    (
        float kP, 
        float kI, 
        float integralLimit,
        float deadband
    )
        : 
        _kP(kP), 
        _kI(kI), 
        _integralLimit(integralLimit),
        _deadband(deadband), 
        _integral(0.0f) 
    {}

// Accumulate integral with anti-windup clamp
float PIController::update(float error, float dt) {

    if (fabsf(error) < _deadband) error = 0.0f;

    _integral += error * dt;
    _integral = constrain(_integral, -_integralLimit, _integralLimit);

    return (_kP * error) + (_kI * _integral);
}

void PIController::reset() { _integral = 0.0f; }

void PIController::setKp(float kP) { _kP = kP; }
void PIController::setKi(float kI) { _kI = kI; }
void PIController::setIntegralLimit(float intLimit) { _integralLimit = intLimit; }
void PIController::setDeadband(float deadband) { _deadband = deadband; }
