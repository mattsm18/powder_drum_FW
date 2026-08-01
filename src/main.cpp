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
#include "SerialManager/SerialManager.h"
#include "MotionManager/MotionManager.h"
#include "pins.h"

//*** Definitions ***//

// Timing
#define ISR_FREQ_HZ 32000

// Serial Comms
#define SERIAL_BAUD_RATE 115200

// Setup
void setupISR();
void mapSerialParameters();

//*** Instantiate Objects ***//
SerialManager serialManager;

//*** Main Program ***/
void setup() {

    setupISR();
    mapSerialParameters();

    serialManager.begin(SERIAL_BAUD_RATE);
}

void loop() {
    serialManager.update();
}

void mapSerialParameters()
{
    serialManager.onSet([](uint8_t parameterID, float value) {
        switch (parameterID) {
            case ParamID::TARGETANGULARVELOCITY: break;
            case ParamID::ACCELERATIONRATE:      break;
            //case ParamID::KP: kP = value; break;
            //case ParamID::KI: kI = value; break;
            //case ParamID::KD: kD = value; break;
            case ParamID::TOGGLELIGHTS: break;
        }
    });

    serialManager.onGet([](uint8_t parameterID) -> float {
        switch (parameterID) {
            case ParamID::PROTOCOLVERSION:        return SERIAL_PROTOCOL_VERSION;
            case ParamID::TARGETANGULARVELOCITY:  return 0;
            case ParamID::ACCELERATIONRATE:       return 0;
            case ParamID::ENCODERANGULARVELOCITY: return 0;
            //case ParamID::KP: return kP;
            //case ParamID::KI: return kI;
            //case ParamID::KD: return kD;
            case ParamID::TOGGLELIGHTS: return 0;
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