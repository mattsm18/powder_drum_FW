#include "MotionManager.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

MotionManager::MotionManager()
    :
    _encoder(0x36, Wire, 40000),
    _motor(TB6600_DRIVER_A_PUL, TB6600_DRIVER_A_DIR, TB6600_DRIVER_A_ENA, Microstep::THIRTY_SECOND, 200, STEPPER_TICK_ISR_FREQ_HZ),
    _controller(KP, KI, KD, INTEGRAL_LIMIT, OUTPUT_LIMIT)
{}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MotionManager::begin()
{
    enableMotor();
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

    _encoder.update(); 
    updateControlLoop();
}

void MotionManager::updateControlLoop()
{
    _measuredVelocity = _encoder.getAngularVelocityRadS();

    // Closed-loop
    float error = _setpointVelocity - _measuredVelocity;
    float correction = _controller.update(error, MOTION_DT);

    // Feed-forward (commanded = setpoint + correction)
    _controlVelocity = _setpointVelocity + correction;

    // Command motor to move at specified velocity
    _motor.setAngularVelocity(_controlVelocity);
}
