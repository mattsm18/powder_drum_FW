#include "MotionManager.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

MotionManager::MotionManager()
    :
    _encoder(),
    _motor(
        TB6600_DRIVER_A_PUL, TB6600_DRIVER_A_DIR, TB6600_DRIVER_A_ENA,
        Microstep::THIRTY_SECOND, 200, ISR_FREQ_HZ
    ),
    _filter(VELOCITY_FILTER_TIME_CONST),
    _controller(KP, KI, KD)
{}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MotionManager::begin() {

    // Ensure a clean starting state -- no stale integral, no stale dt/theta seed
    _controller.reset();
    _filter.reset();
    _estimatorInitialised = false;
    _lastMicros = 0;

    enableMotor();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MotionManager::update() {

    uint32_t now = micros();

    // First call (or just re-enabled) -- seed timing + encoder state, nothing to
    // differentiate against yet
    if (!_estimatorInitialised) {
        _thetaPrev = _encoder.getAngleRadians();
        _estimatorInitialised = true;
        _lastMicros = now;
        return;
    }

    float dt = (now - _lastMicros) * 1e-6f;
    _lastMicros = now;

    if (dt <= 0.0f) { return; } // guard against a degenerate/duplicate timer read

    // --- Velocity Estimation ---
    float theta = _encoder.getAngleRadians();
    float dTheta = theta - _thetaPrev;

    // Handle wraparound (jump from 2*PI -> 0 and 0 -> 2*PI)
    if (dTheta >  PI) dTheta -= 2.0f * PI;
    if (dTheta < -PI) dTheta += 2.0f * PI;

    float rawVelocity = dTheta / dt;
    _thetaPrev = theta;

    float measuredVelocity = _filter.update(rawVelocity, dt);

    // --- Setpoint Ramp ---
    float delta = _setpoint - _rampedSetpoint;
    float maxStep = _accelRate * dt;
    _rampedSetpoint += constrain(delta, -maxStep, maxStep);

    // --- Closed-Loop Control ---
    float error = _rampedSetpoint - measuredVelocity;
    float output = _controller.update(error, dt);

    _motor.setAngularVelocity(output);
}