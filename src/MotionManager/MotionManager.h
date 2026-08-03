#ifndef MOTIONMANAGER_H
#define MOTIONMANAGER_H

/*
Title: MotionManager.h
Author: Matthew Smith 22173112
Date: 2/08/26

Purpose:
- Handle closed-loop velocity control of stepper motor
- Wires together the business logic of the powder drum application with custom single-responsibility libraries.

Note:
- The ISR should call StepperMotor::tickAll() directly, not through this class --
  StepperMotor self-registers its instances, so MotionManager doesn't need to
  expose _motor to give the ISR access to tick().
*/

#include <Arduino.h>
#include <SignalProcessing.h>
#include <PIDController.h>
#include <StepperMotor.h>
#include <AS5600.h>

#include "pins.h"

#define STEPPER_TICK_ISR_FREQ_HZ 32000
#define MOTION_LOOP_FREQ_HZ 200
#define MOTION_LOOP_PERIOD_US (1000000UL / MOTION_LOOP_FREQ_HZ)
#define MOTION_LOOP_DT        (1.0f / MOTION_LOOP_FREQ_HZ)

// Default Controller Gains
#define KP 5.0f
#define KI 1.0f
#define KD 0.0f
#define INTEGRAL_LIMIT 20.0f   
#define OUTPUT_LIMIT   30.0f   // real max achievable motor rad/s

// Velocity Estimation Filtration
#define VELOCITY_ESTIMATION_WINDOW 10
#define EMA_FILTER_TIME_CONST 0.02f

class MotionManager
{
public:

    MotionManager();

    void begin();
    void update();

    void enableMotor()  { _motor.enable(); }
    void disableMotor() { _motor.disable(); }

    // Setpoint / Motion Profile
    void setSetpointAngularVelocity(float radPerSec) { _setpoint = radPerSec; }
    void setAccelRate(float radPerSec2) { _accelRate = radPerSec2; }

    float getSetpointAngularVelocity() const { return _setpoint; }
    float getRampedAngularVelocity() const { return _rampedSetpoint; }
    float getAccelRate() const { return _accelRate; }

    // Controller Tuning
    void setKp(float kP) { _controller.setKp(kP); }
    void setKi(float kI) { _controller.setKi(kI); }
    void setKd(float kD) { _controller.setKd(kD); }

    float getKp() { return _controller.getKp(); }
    float getKi() { return _controller.getKi(); }
    float getKd() { return _controller.getKd(); }

    // Accessors
    float getEncoderAngularVelocity() { return _measuredVelocity; }
    float getEncoderAngleRadians()    { return _encoder.getAngleRadians(); }
    float getEncoderAngleDegrees()    { return _encoder.getAngleDegrees(); }

    float getMotorAngularVelocity()   { return _motor.getAngularVelocity(); }
    void setMotorAngularVelocity(float value) { _motor.setAngularVelocity(value); }

private:

    // Hardware
    AS5600 _encoder;
    StepperMotor _motor;
    EMAFilter _filter;

    // Signal Processing / Control
    float _thetaBuffer[VELOCITY_ESTIMATION_WINDOW] = {0.0f};
    uint8_t _thetaIndex = 0;
    bool _thetaBufferFilled = false;

    PIDController _controller;

    // Setpoint / Motion Profile
    float _setpoint = 0.0f;
    float _rampedSetpoint = 0.0f;
    float _accelRate = 5.0f;

    // Velocity Estimation
    float _thetaPrev = 0.0f;
    float _measuredVelocity = 0.0f;
    bool _estimatorInitialised = false;

    // Timing
    uint32_t _lastControlMicros = 0;

    // Helper functions
    float _estimateEncoderVelocity();
};

#endif