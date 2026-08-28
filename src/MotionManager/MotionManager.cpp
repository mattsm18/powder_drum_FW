#include "MotionManager.h"
#include <AsyncTWI.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

MotionManager::MotionManager()
    :
    _encoder(0x36, twi, 40000),
    _motor(TB6600_DRIVER_A_PUL, TB6600_DRIVER_A_DIR, TB6600_DRIVER_A_ENA, Microstep::THIRTY_SECOND, 200, STEPPER_TICK_ISR_FREQ_HZ),
    _controller(KP, KI, KD, INTEGRAL_LIMIT, OUTPUT_LIMIT)
{}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MotionManager::begin()
{
    enableMotor();
    twi.begin(100000);
    _encoder.begin();
    _controller.reset();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MotionManager::update()
{
    // update the Control Loop slowly (at 200Hz)
    uint32_t now = micros();
    if (now - _lastControlMicros < MOTION_UPDATE_PERIOD_US) return;
    _lastControlMicros = now;

    //_encoder.update(); 
    updateControlLoop();
}

void MotionManager::updateControlLoop()
{
    _measuredVelocity = -1.0f * _encoder.getAngularVelocityRadS();  // Sign flip
    _measuredAngleDegrees = _encoder.getAngleDegrees();
    
    // Ramp toward the commanded setpoint rather than stepping onto it directly
    float delta = _setpointVelocity - _rampedSetpoint;
    float maxStep = _accelRate * MOTION_DT;
    _rampedSetpoint += constrain(delta, -maxStep, maxStep);

    // Closed-loop
    float error = _rampedSetpoint - _measuredVelocity;
    float correction = _controller.update(error, MOTION_DT);

    // Feed-forward (commanded = ramped setpoint + correction)
    _controlVelocity = _rampedSetpoint + correction;

    // Command motor to move at specified velocity
    _motor.setAngularVelocity(_controlVelocity);
}
