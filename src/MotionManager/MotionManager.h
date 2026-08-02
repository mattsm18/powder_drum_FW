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

// Timing
#define ISR_FREQ_HZ 32000

// Default Controller Gains
#define KP 1.0f
#define KI 2.0f
#define KD 0.0f

// Velocity Estimation
#define VELOCITY_FILTER_TIME_CONST 0.02f

class MotionManager
{
    public:
        // Constructor
        MotionManager();

        // Public API
        void begin();
        void update();
        void enableMotor()  { _motor.enable(); }
        void disableMotor() { _motor.disable(); }

        // Setpoint / Motion Profile
        void setTargetVelocity(float radPerSec) { _setpoint = radPerSec; }
        void setAccelRate(float radPerSec2)     { _accelRate = radPerSec2; }

        float getTargetVelocity() const { return _setpoint; }
        float getRampedVelocity() const { return _rampedSetpoint; }
        float getAccelRate()      const { return _accelRate; }

        // Controller Tuning
        void setKp(float kP) { _controller.setKp(kP); }
        void setKi(float kI) { _controller.setKi(kI); }
        void setKd(float kD) { _controller.setKd(kD); }

        float getKp() { return _controller.getKp(); }
        float getKi() { return _controller.getKi(); }
        float getKd() { return _controller.getKd(); }

        // Accessors
        float getEncoderAngularVelocity() { return _filter.getValue(); }
        float getEncoderAngleRadians()    { return _encoder.getAngleRadians(); }
        float getEncoderAngleDegrees()    { return _encoder.getAngleDegrees(); }
        float getMotorAngularVelocity()   { return _motor.getAngularVelocity(); }

    private:

        // Private attributes
        AS5600 _encoder;
        StepperMotor _motor;
        EMAFilter _filter;
        PIDController _controller;

        // Setpoint / Motion Profile
        float _setpoint = 0.0f;
        float _rampedSetpoint = 0.0f;
        float _accelRate = 5.0f;

        // Velocity estimation state (differentiation, wraparound handling)
        float _thetaPrev = 0.0f;
        bool _estimatorInitialised = false;

        // Timing
        uint32_t _lastMicros = 0;

};

#endif