#include "MotionManager.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

MotionManager::MotionManager()
    :
    _encoder(),
    _motor(
        TB6600_DRIVER_A_PUL,
        TB6600_DRIVER_A_DIR,
        TB6600_DRIVER_A_ENA,
        Microstep::THIRTY_SECOND,
        200,
        STEPPER_TICK_ISR_FREQ_HZ
    ),
    _controller(KP, KI, KD, INTEGRAL_LIMIT, OUTPUT_LIMIT),
    _filter(EMA_FILTER_TIME_CONST)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MotionManager::begin()
{
    _controller.reset();
    _filter.reset();
    _thetaIndex = 0;
    _thetaBufferFilled = false;

    for (uint8_t i = 0; i < VELOCITY_ESTIMATION_WINDOW; i++)
    {
        _thetaBuffer[i] = 0.0f;
    }

    _estimatorInitialised = false;
    _measuredVelocity = 0.0f;
    _lastControlMicros = 0;
    _encoder.begin();

    enableMotor();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// Update the motion loop at MOTION_LOOP_FREQ_HZ
void MotionManager::update()
{   
    //_motor.setAngularVelocity(20.0);
    // Timing (Enter loop only at MOTION_LOOP_PERIOD_US)
    uint32_t now = micros();
    if (now - _lastControlMicros < MOTION_LOOP_PERIOD_US) return;
    _lastControlMicros = now;
    
    // Initialise estimator on first run
    if (!_estimatorInitialised)
    {
        _thetaPrev = _encoder.getAngleRadians();
        _estimatorInitialised = true;
        _measuredVelocity = 0.0f;
        return;
    }
    
    _measuredVelocity = _estimateEncoderVelocity();

    // Setpoint ramp
    float delta = _setpoint - _rampedSetpoint;
    float maxStep = _accelRate * MOTION_LOOP_DT;
    _rampedSetpoint += constrain(delta, -maxStep, maxStep);

    // Closed-loop control
    float error = _rampedSetpoint - _measuredVelocity;
    float correction = _controller.update(error, MOTION_LOOP_DT);
    float commandedVelocity = _rampedSetpoint + correction;
    
   
}

float MotionManager::_estimateEncoderVelocity()
{
    static const float MAX_PLAUSIBLE_VELOCITY = 60.0f;

    float theta = _encoder.getAngleRadians();

    // Not enough history yet
    if (!_thetaBufferFilled)
    {
        _thetaBuffer[_thetaIndex++] = theta;

        if (_thetaIndex >= VELOCITY_ESTIMATION_WINDOW)
        {
            _thetaIndex = 0;
            _thetaBufferFilled = true;
        }

        return 0.0f;
    }

    float thetaOld = _thetaBuffer[_thetaIndex];

    // Replace oldest with newest
    _thetaBuffer[_thetaIndex] = theta;
    _thetaIndex = (_thetaIndex + 1) % VELOCITY_ESTIMATION_WINDOW;

    float dTheta = theta - thetaOld;

    // Wraparound correction
    if (dTheta > PI)  dTheta -= 2.0f * PI;
    if (dTheta < -PI) dTheta += 2.0f * PI;

    float velocity = dTheta / (VELOCITY_ESTIMATION_WINDOW * MOTION_LOOP_DT);

    if (fabsf(velocity) > MAX_PLAUSIBLE_VELOCITY){ return _measuredVelocity; }
    return _filter.update(velocity, MOTION_LOOP_DT);
}