// StepperMotor.cpp
#include "StepperMotor.h"

StepperMotor* StepperMotor::_instances[StepperMotor::MAX_INSTANCES] = {};
uint8_t StepperMotor::_instanceCount = 0;

StepperMotor::StepperMotor
(
    uint8_t pulPin, uint8_t dirPin, uint8_t enaPin,
    Microstep microstep, uint16_t fullStepsPerRev,
    uint32_t tickFreqHz
): 
    _pulPin(pulPin), _dirPin(dirPin), _enaPin(enaPin),
    _fullStepsPerRev(fullStepsPerRev), _microstep(microstep),
     _tickFreqHz(tickFreqHz)
{
    _microstepsPerRev = (uint32_t)_fullStepsPerRev * (uint8_t)_microstep;

    pinMode(_pulPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_enaPin, OUTPUT);
    digitalWrite(_pulPin, LOW);

    if (_instanceCount < MAX_INSTANCES) { _instances[_instanceCount++] = this; }
    // else: silently drops registration — consider an assert/error hook here
}

////////////////////////////////////////////////////////////////////////////////////////////

void StepperMotor::enable() {
    digitalWrite(_enaPin, LOW);   
    _enabled = true;
}

////////////////////////////////////////////////////////////////////////////////////////////

void StepperMotor::disable() {
    digitalWrite(_enaPin, HIGH);
    _enabled = false;
}

////////////////////////////////////////////////////////////////////////////////////////////

void StepperMotor::setAngularVelocity(float radPerSec) { applyStepRate(radPerSec); }

////////////////////////////////////////////////////////////////////////////////////////////

void StepperMotor::applyStepRate(float radPerSec) {
    _commandedVelRadS = radPerSec;
    _direction = (radPerSec >= 0.0f) ? 1 : -1;
    digitalWrite(_dirPin, _direction > 0 ? HIGH : LOW);

    float stepsPerSec = fabsf(radPerSec) * (float)_microstepsPerRev / (2.0f * PI);

    // Fraction of full 32-bit accumulator range added per tick to hit stepsPerSec
    // stepsPerSec / tickFreqHz = fraction of one full accumulator wrap per tick
    float fraction = stepsPerSec / (float)_tickFreqHz;
    _accumulatorStep = (uint32_t)(fraction * 4294967296.0); // 2^32
}

////////////////////////////////////////////////////////////////////////////////////////////

void StepperMotor::tick() {

    uint32_t prev = _accumulator;
    _accumulator += _accumulatorStep;

    // overflow occurred -> emit a step pulse
    if (_accumulator < prev) { 
        digitalWrite(_pulPin, HIGH);
        digitalWrite(_pulPin, LOW);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////

void StepperMotor::tickAll() {
    for (uint8_t i = 0; i < _instanceCount; i++) {
        _instances[i]->tick();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////