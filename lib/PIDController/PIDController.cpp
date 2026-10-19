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
        float kD,
        float integralLimit,
        float outputLimit
    )
        : 
        _kP(kP), 
        _kI(kI), 
        _kD(kD),
        _integral(0.0f),
        _prevError(0.0f),
        _integralLimit(integralLimit),
        _outputLimit(outputLimit)
    {}

///////////////////////////////////////////////////////////////////////////////////////////////////

float PIDController::update(float error, float dt) {

    float derivative = 0.0f;
    if (dt > 0.0f) { derivative = (error - _prevError) / dt; }
    _prevError = error;

    // Provisional output using the *current* (pre-update) integral,
    // to decide whether integrating this step would push us further into saturation
    float provisionalOutput = (_kP * error) + (_kI * _integral) + (_kD * derivative);

    bool saturatedHigh = provisionalOutput > _outputLimit;
    bool saturatedLow  = provisionalOutput < -_outputLimit;

    // Conditional integration: only accumulate if doing so won't push us
    // further past a limit we're already at (anti-windup)
    bool integratingWouldWorsenSaturation =
        (saturatedHigh && error > 0.0f) ||
        (saturatedLow  && error < 0.0f);

    if (!integratingWouldWorsenSaturation) {
        _integral += error * dt;
        _integral = constrain(_integral, -_integralLimit, _integralLimit);
    }

    float output = (_kP * error) + (_kI * _integral) + (_kD * derivative);
    return constrain(output, -_outputLimit, _outputLimit);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void PIDController::reset() { 
    _integral = 0.0f; 
    _prevError = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void PIDController::setKp(float kP) { _kP = kP; }
void PIDController::setKi(float kI) { _kI = kI; }
void PIDController::setKd(float kD) { _kD = kD; }
void PIDController::setIntegralLimit(float integralLimit) { _integralLimit = integralLimit; }
void PIDController::setOutputLimit(float outputLimit) { _outputLimit = outputLimit; }