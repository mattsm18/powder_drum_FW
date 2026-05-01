/*
Title: PIController
Author: Matthew Smith
Date: 29/04/26
Purpose:
- Handle pure PI calculations

*/

#include "PIController.h"


PIController::PIController
    (
        float kP, 
        float kI, 
        float integralLimit
    )
        : 
        _kP(kP), 
        _kI(kI), 
        _integralLimit(integralLimit), 
        _integral(0.0f) 
    {}

// Accumulate integral with anti-windup clamp
float PIController::update(float error, float dt) {
        
    _integral += error * dt;
    _integral = constrain(_integral, -_integralLimit, _integralLimit);

    return (_kP * error) + (_kI * _integral);
}

void PIController::reset() { _integral = 0.0f; }