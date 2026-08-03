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
#define MOTION_UPDATE_FREQ_HZ 200
#define MOTION_UPDATE_PERIOD_US (1000000UL / MOTION_UPDATE_FREQ_HZ)
#define MOTION_DT               (1.0f / MOTION_UPDATE_FREQ_HZ)

// Default Controller Gains
#define KP 0.2f
#define KI 0.1f
#define KD 0.0f
#define INTEGRAL_LIMIT 20.0f   
#define OUTPUT_LIMIT   40.0f   // real max achievable motor rad/s

class MotionManager
{
public:
    
    MotionManager();
    
    /////*** PUBLIC METHODS ***/////
    void begin();
    void update();
    void updateControlLoop();

    void enableMotor()  { _motor.enable(); }
    void disableMotor() { _motor.disable(); }

    /////*** PUBLIC ACCESSORS ***/////
    // Setters
    void setSetpoint(float setpoint) { _setpointVelocity = setpoint; }
    void setAccelRate(float accel)  { _accelRate = accel; }

    void setKp(float kP) { _controller.setKp(kP); }
    void setKi(float kI) { _controller.setKi(kI); }
    void setKd(float kD) { _controller.setKd(kD); }

    // Getters
    float getKp() { return _controller.getKp(); }
    float getKi() { return _controller.getKi(); }
    float getKd() { return _controller.getKd(); }
    
    float getSetpoint()               { return _setpointVelocity; }
    float getRampedSetpoint()         { return _rampedSetpoint; }
    float getMeasuredVelocity()       { return _measuredVelocity; }
    float getControlVelocity()        { return _controlVelocity; }
    float getAccelRate()              { return _accelRate; }
    
private:

    // Hardware
    AS5600 _encoder;
    StepperMotor _motor;
    PIDController _controller;

    // Setpoint / Motion Profile
    float _setpointVelocity = 0.0f; 
    float _rampedSetpoint = 0.0f;
    float _measuredVelocity = 0.0f;
    float _controlVelocity = 0.0f;

    float _accelRate = 5.0f;

    // Dt calculation
    uint32_t _lastControlMicros = 0;
};

#endif