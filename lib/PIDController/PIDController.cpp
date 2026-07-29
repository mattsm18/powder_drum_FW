/*
Title: PIDController
Author: Matthew Smith
Date: 29/04/26
Purpose:
- Handle pure PID calculations

*/

#include "PIDController.h"

PIDController::PIDController
    (
        float kP, 
        float kI, 
        float kD
    )
        : 
        _kP(kP), 
        _kI(kI), 
        _kD(kD),
        _integral(0.0f),
        _prevError(0.0f)
    {}

float PIDController::update(float error, float dt) {

    _integral += error * dt;

    float derivative = 0.0f;
    if (dt > 0.0f) { derivative = (error - _prevError) / dt; }
    _prevError = error;

    return (_kP * error) + (_kI * _integral) + (_kD * derivative);
}

void PIDController::reset() { 
    _integral = 0.0f; 
    _prevError = 0.0f;
}

void PIDController::setKp(float kP) { _kP = kP; }
void PIDController::setKi(float kI) { _kI = kI; }
void PIDController::setKd(float kD) { _kD = kD; }