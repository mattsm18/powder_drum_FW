/*
Title: main.cpp
Author: Matthew Smith
Date: 29/04/26
Purpose:
-  Entry point for powder drum FW
-  Pass relevant data between objects

*/

#include <Arduino.h>
#include <PowderDrumProtocol.h>
#include <StepperMotor.h>
#include "SerialManager/SerialManager.h"
#include "MotionManager/MotionManager.h"
#include "pins.h"

// Setup
void setupISR();
void mapSerialParameters();

//*** Instantiate Objects ***//
SerialManager serial;
MotionManager motion;

//*** Global State ***//
float lightState = 0.0f; // 0 = off, anything > 0 = on

//*** Main Program ***/
void setup() {

    pinMode(RELAY_OUTPUT_B, OUTPUT);
    digitalWrite(RELAY_OUTPUT_B, LOW);

    setupISR();
    mapSerialParameters();

    serial.begin(115200);
    motion.begin();
}

void loop() {
    serial.update();
    motion.update();

}

void mapSerialParameters()
{
    serial.onSet([](uint8_t parameterID, float value) {
        switch (parameterID) {
            case ParamID::SETPOINT:                 motion.setSetpoint(value);   break;
            case ParamID::ACCELERATIONRATE:         motion.setAccelRate(value);  break;
            case ParamID::PIDPROPORTIONALGAIN:      motion.setKp(value);            break;
            case ParamID::PIDINTEGRALGAIN:          motion.setKi(value);            break;
            case ParamID::PIDDERIVATIVEGAIN:        motion.setKd(value);            break;
            case ParamID::TOGGLELIGHTS:
                lightState = value;
                digitalWrite(RELAY_OUTPUT_B, lightState > 0.0f ? HIGH : LOW);
                break;
        }
    });

    serial.onGet([](uint8_t parameterID) -> float {
        switch (parameterID) {
            case ParamID::PROTOCOLVERSION:        return SERIAL_PROTOCOL_VERSION;
            case ParamID::SETPOINT:               return motion.getSetpoint();
            case ParamID::RAMPEDSETPOINT:         return motion.getRampedSetpoint();
            case ParamID::ACCELERATIONRATE:       return motion.getAccelRate();
            case ParamID::PIDPROPORTIONALGAIN:    return motion.getKp();
            case ParamID::PIDINTEGRALGAIN:        return motion.getKi();
            case ParamID::PIDDERIVATIVEGAIN:      return motion.getKd();
            case ParamID::MEASUREDANGLEDEGREES:   return motion.getMeasuredAngleDegrees();
            case ParamID::MEASUREDVELOCITY:       return motion.getMeasuredVelocity();
            case ParamID::CONTROLVELOCITY:        return motion.getControlVelocity();
            case ParamID::TOGGLELIGHTS:           return lightState;
            default: return 0.0f;
        }
    });
}

// HARDWARE INTERRUPT SERVICE ROUTINE (ISR) -> Fixed clock for motor updates
// Setup function for Hardware interrupts, used to drive StepperMotors atomically
void setupISR() 
{
    cli();  
    TCB0.CTRLB   = TCB_CNTMODE_INT_gc;                      // Periodic interrupt mode (CTC-equivalent)
    TCB0.CCMP    = (F_CPU / STEPPER_TICK_ISR_FREQ_HZ) - 1;  // Period/compare value
    TCB0.INTCTRL = TCB_CAPT_bm;                             // Enable compare/capture interrupt
    TCB0.CTRLA   = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;   // Start timer, no prescale (clk/1)
    sei();
}

// Attach ISR to StepperMotor step function
ISR(TCB0_INT_vect) 
{
    StepperMotor::tickAll();
    TCB0.INTFLAGS = TCB_CAPT_bm;   // Must clear flag manually — write-1-to-clear
}