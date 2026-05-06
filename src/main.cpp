
/*
Title: main.cpp
Author: Matthew Smith
Date: 29/04/26
Purpose:
-  Entry point for powder drum FW
-  Call all higher level functionality
-  Pass relevant data between objects

*/

#include <Arduino.h>
#include <AS5600.h>                      // Encoder Internal Library
#include <StepperMotor.h>                // StepperMotor Internal Library
#include <PIController.h>                // PI Controller Internal Libaray
#include "SerialHandler/SerialHandler.h" // Serial Comms Implementation
#include "pins.h"

//*** Definitions ***//

// Timing
#define ISR_FREQ_HZ 20000
#define ENCODER_SAMPLE_RATE_US 5000
#define ENCODER_EMA_FILTER_TIME_CONST 0.5f // Low-Pass Noise filter

// PI Controller
#define KP 1.0f
#define KI 2.0f
#define INTEGRAL_LIMIT 20   // Stops integral windup
#define DEADBAND 0.0f       // Helps to limit noise at steady-state

// Serial Comms
#define SERIAL_BAUD_RATE 115200

// Setup
void setupISR();
void mapSerialParameters();

// Run
void runSpeedControl();

//*** Instantiate Objects ***//
AS5600 encoder(ENCODER_SAMPLE_RATE_US, ENCODER_EMA_FILTER_TIME_CONST);
StepperMotor motor(DRIVER_STEP_PIN, DRIVER_DIR_PIN, Microstep::SIXTEENTH, 200, ISR_FREQ_HZ);
PIController piController(KP, KI, INTEGRAL_LIMIT, DEADBAND);
SerialHandler serialComms;

//*** Global Variables ***/
float setpoint = 0;
float rampedSetpoint = 0.0f;
float accelRate = 5.0f;

void setup() {

    // Start Serial Comms
    serialComms.begin(SERIAL_BAUD_RATE);
    mapSerialParameters();

    // Setup ISR to Fire StepperMotor ticks atomically
    setupISR();

}

void loop() {
    
    runSpeedControl();
    serialComms.update();
}


//////////////////////////////////////////////////////////////////////////////////
// HARDWARE INTERRUPT SERVICE ROUTINE (ISR) -> Fixed clock for motor updates
//////////////////////////////////////////////////////////////////////////////////

// Setup function for Hardware interrupts, used to drive StepperMotors atomically
void setupISR() 
{
    cli();
    TCB0.CTRLA   = 0;
    TCB0.CTRLB   = TCB_CNTMODE_INT_gc;
    TCB0.CCMP = (F_CPU / ISR_FREQ_HZ) - 1;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CTRLA   = TCB_ENABLE_bm | TCB_CLKSEL_CLKDIV1_gc;
    sei();
}

// Attach ISR to StepperMotor step function
ISR(TCB0_INT_vect) 
{
    motor.tick();
    TCB0.INTFLAGS = TCB_CAPT_bm;
}

//////////////////////////////////////////////////////////////////////////////////
// Plumb Serial Parameters to corrosponding data and/or functions
//////////////////////////////////////////////////////////////////////////////////

void mapSerialParameters()
{
    // SET — write only
    serialComms.onSet([](uint8_t parameter_id, float value) {
        switch (parameter_id) {
            case 0x01: setpoint   = value; break;
            case 0x02: accelRate  = value; break;
        }
    });

    // GET — read only, version reserved at 0x00
    serialComms.onGet([](uint8_t parameter_id) -> float {
        switch (parameter_id) {
            case 0x00: return SERIAL_PROTOCOL_VERSION;
            case 0x01: return encoder.getAngularVelocity();
            case 0x02: return rampedSetpoint;
            case 0x03: return setpoint;
            case 0x04: return accelRate;
            default:   return 0.0f;
        }
    });
}

//////////////////////////////////////////////////////////////////////////////////
// Motor Speed Control Handler -> Plumb closed-loop feedback
//////////////////////////////////////////////////////////////////////////////////

void runSpeedControl()
{
    encoder.update();

    static uint32_t lastTime = 0;
    uint32_t now = micros();

    if (lastTime == 0) { lastTime = now; return; }

    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    // Ramp commanded setpoint toward target
    float delta = setpoint - rampedSetpoint;
    float maxStep = accelRate * dt;
    rampedSetpoint += constrain(delta, -maxStep, maxStep);

    // PI tracks the ramp, not the target directly
    float error = rampedSetpoint - (-encoder.getAngularVelocity());
    float output = piController.update(error, dt);
    motor.setAngularVelocity(output);
}