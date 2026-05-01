#include "StepperMotor.h"

StepperMotor* StepperMotor::_instance = nullptr;

StepperMotor::StepperMotor(uint8_t stepPin, uint8_t dirPin, Microstep microstep, uint8_t stepsPerRev, uint32_t isrFreq)
    : _stepPin(stepPin), _dirPin(dirPin), _stepsPerRev(stepsPerRev * microstep), _isrFreq(isrFreq)
{
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    _instance = this;
}

void StepperMotor::setAngularVelocity(float radsPerSec) {
    _angularVelocity = radsPerSec;
    _direction = (radsPerSec >= 0.0f) ? CW : CCW;
    digitalWrite(_dirPin, _direction);
    updateStepTiming();
}

float StepperMotor::getAngularVelocity() const {
    return _angularVelocity;
}

void StepperMotor::updateStepTiming() {
    float stepsPerSec = (fabsf(_angularVelocity) / (2.0f * M_PI)) * _stepsPerRev;
    _phaseIncrement = stepsPerSec / _isrFreq;
}

// Entry point for ISR
void StepperMotor::tick() { if (_instance) _instance->stepIfDue(); }

void StepperMotor::stepIfDue() {
    // Finish pulse (LOW phase)
    if (_stepHigh) {
        PORTE.OUTCLR = (1 << 1);
        _stepHigh = false;
    }

    // Accumulate phase
    _phase += _phaseIncrement;

    // Start pulse if needed
    if (_phase >= 1.0f) {
        _phase -= 1.0f;

        PORTE.OUTSET = (1 << 1);
        _stepHigh = true;
    }
}