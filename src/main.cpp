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

#include "CommsManager/SerialManager.h"  // Serial Comms Implementation
#include "pins.h"

//*** Definitions ***//

// Timing
#define ISR_FREQ_HZ 32000

// Serial Comms
#define SERIAL_BAUD_RATE 115200

// Setup
void setupISR();
void mapSerialParameters();

// Run
void runSpeedControl();

//*** Global Variables ***/
float setpoint = 0;
float rampedSetpoint = 0.0f;
float accelRate = 5.0f;
float lightState = 255.0f; //0 = off, anything > 0 = on

//*** Instantiate Objects ***//
AS5600 encoderA;

StepperMotor motorA(
    TB6600_DRIVER_A_PUL, TB6600_DRIVER_A_DIR, TB6600_DRIVER_A_ENA, 
    Microstep::THIRTY_SECOND, 200, ISR_FREQ_HZ
);

#define KP 1.0
#define KI 2.0
#define KD 0.5

PIDController controller(KP, KI, KD);

SerialHandler serialComms;

//*** Main Program ***/
void setup() {

    // Start Serial Comms
    serialComms.begin(SERIAL_BAUD_RATE);
    mapSerialParameters(); 

    // Setup light relay pin
    pinMode(RELAY_OUTPUT_B, OUTPUT);

    // Setup ISR to Fire StepperMotor ticks atomically
    setupISR();
    
    motorA.enable();
}

void loop() {
    motorA.setAngularVelocity(10.0);

    //runSpeedControl();
    //serialComms.update();

    // Update the light relay
    //(lightState > 0) ? digitalWrite(RELAY_OUTPUT_B, HIGH) : digitalWrite(RELAY_OUTPUT_B, LOW);
}

//////////////////////////////////////////////////////////////////////////////////
// HARDWARE INTERRUPT SERVICE ROUTINE (ISR) -> Fixed clock for motor updates
//////////////////////////////////////////////////////////////////////////////////

// Setup function for Hardware interrupts, used to drive StepperMotors atomically
void setupISR() 
{
    cli();
    TCB0.CTRLB   = TCB_CNTMODE_INT_gc;               // Periodic interrupt mode (CTC-equivalent)
    TCB0.CCMP    = (F_CPU / ISR_FREQ_HZ) - 1;         // Period/compare value
    TCB0.INTCTRL = TCB_CAPT_bm;                       // Enable compare/capture interrupt
    TCB0.CTRLA   = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Start timer, no prescale (clk/1)
    sei();
}

// Attach ISR to StepperMotor step function
ISR(TCB0_INT_vect) 
{
    motorA.tick();
    TCB0.INTFLAGS = TCB_CAPT_bm;   // Must clear flag manually — write-1-to-clear
}

//////////////////////////////////////////////////////////////////////////////////
// Plumb Serial Parameters to corrosponding data and/or functions
//////////////////////////////////////////////////////////////////////////////////

void mapSerialParameters()
{
    // SET — write only
    serialComms.onSet([](uint8_t parameter_id, float value) {
        switch (parameter_id) {
            case 0x01: setpoint = value;                    break;
            case 0x02: accelRate = value;                   break;
            case 0x20: controller.setKp(value);             break;
            case 0x21: controller.setKi(value);             break;
            case 0x22: controller.setKd(value);             break;
            case 0x30: lightState = value;                  break;
        }
    });

    // GET — read only
    serialComms.onGet([](uint8_t parameter_id) -> float {
        switch (parameter_id) {
            case 0x00: return SERIAL_PROTOCOL_VERSION;
            case 0x01: return setpoint;
            case 0x02: return accelRate;
            case 0x03: return rampedSetpoint;
            case 0x20: return controller.getKp();
            case 0x21: return controller.getKi();
            case 0x22: return controller.getKd();
            case 0x30: return lightState;
            default:   return 0.0f;
        }
    });
}