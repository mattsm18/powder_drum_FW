/*
Title: main.cpp
Author: Matthew Smith
Date: 29/04/26
Purpose:
-  Entry point for powder drum FW
-  Call all higher level functionality
-  Pass relevant data between objects

*/
#include <avr/interrupt.h>

#include <Arduino.h>
#include <AS5600.h>                      // Encoder Internal Library
#include <StepperMotor.h>                // StepperMotor Internal Library
#include <PIDController.h>               // PI Controller Internal Libaray
#include <PowderDrumProtocol.h>
#include <SerialManager.h>
#include "pins.h"

//*** Definitions ***//

// Timing
#define ISR_FREQ_HZ 32000

// Serial Comms
#define SERIAL_BAUD_RATE 115200

// Setup
void setupISR();
void mapSerialParameters();

//*** Global Variables ***/
float setpoint = 0;
float rampedSetpoint = 0.0f;
float accelRate = 5.0f;
float lightState = 255.0f; //0 = off, anything > 0 = on

//*** Instantiate Objects ***//
AS5600 encoderA;
StepperMotor motorA(TB6600_DRIVER_A_PUL, TB6600_DRIVER_A_DIR, TB6600_DRIVER_A_ENA, Microstep::THIRTY_SECOND, 200, ISR_FREQ_HZ);
SerialManager serialManager;

//*** Main Program ***/
void setup() {

    setupISR();
    
    motorA.enable();
    serialManager.begin(SERIAL_BAUD_RATE);
    mapSerialParameters();
}

void loop() {
    serialManager.update();
    motorA.setAngularVelocity(10.0);
}

void mapSerialParameters()
{
    serialManager.onSet([](uint8_t parameterID, float value) {
        switch (parameterID) {
            case ParamID::SETPOINT:setpoint  = value; break;
            case ParamID::ACCELRATE:accelRate = value; break;
            //case ParamID::KP: kP = value; break;
            //case ParamID::KI: kI = value; break;
            //case ParamID::KD: kD = value; break;
            case ParamID::LIGHTS: lightState = value; break;
        }
    });

    serialManager.onGet([](uint8_t parameterID) -> float {
        switch (parameterID) {
            case ParamID::PROTOCOLVERSION:        return SERIAL_PROTOCOL_VERSION;
            case ParamID::SETPOINT:               return setpoint;
            case ParamID::ACCELRATE:              return accelRate;
            case ParamID::RAMPEDSETPOINT:         return rampedSetpoint;
            //case ParamID::ENCODERANGULARVELOCITY: return encoderVelocity;
            //case ParamID::KP: return kP;
            //case ParamID::KI: return kI;
            //case ParamID::KD: return kD;
            case ParamID::LIGHTS: return lightState;
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
    TCB0.CCMP    = (F_CPU / ISR_FREQ_HZ) - 1;               // Period/compare value
    TCB0.INTCTRL = TCB_CAPT_bm;                             // Enable compare/capture interrupt
    TCB0.CTRLA   = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;   // Start timer, no prescale (clk/1)
    sei();
}

// Attach ISR to StepperMotor step function
ISR(TCB0_INT_vect) 
{
    motorA.tick();
    TCB0.INTFLAGS = TCB_CAPT_bm;   // Must clear flag manually — write-1-to-clear
}