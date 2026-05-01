
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
#include <AS5600.h>         // Encoder Library
#include <StepperMotor.h>   // StepperMotor Library
#include <PIController.h>   // PI Controller Libaray
#include "pins.h"

//*** Definitions ***//

// Timing
#define ISR_FREQ_HZ 50000
#define ENCODER_SAMPLE_RATE_US 5000
#define ENCODER_EMA_FILTER_TIME_CONST 0.15f

// PI Controller
#define KP 1
#define KI 10
#define INTEGRAL_LIMIT 20

// Serial Comms
#define SERIAL_BAUD_RATE 115200
#define SERIAL_LOG_RATE_US 50000

//*** Function Declarations ***//
// Setup
void setupISR();

void runSpeedControl();
void runSerialPrinter();
void runSerialListener();

//*** Instantiate Objects ***//
AS5600 encoder(ENCODER_SAMPLE_RATE_US, ENCODER_EMA_FILTER_TIME_CONST);
StepperMotor motor(DRIVER_STEP_PIN, DRIVER_DIR_PIN, Microstep::QUARTER, 200, ISR_FREQ_HZ);
PIController piController(KP, KI, INTEGRAL_LIMIT);

//*** Global Variables ***/
float setpointAngularVelocity = 0;

void setup() {

  Serial.begin(SERIAL_BAUD_RATE);
  setupISR();

}

void loop() {

    runSpeedControl();
    runSerialPrinter();
    runSerialListener();
    
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
// Motor Speed Control Handler -> Plumb closed-loop feedback
//////////////////////////////////////////////////////////////////////////////////

void runSpeedControl()
{
    encoder.update();

    // Define timing
    static uint32_t lastTime = 0;
    uint32_t now = micros();

    // Guard against inital case
    if (lastTime == 0) { lastTime = now; return; }
    
    // Calculate dt
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    // Run PI Controller
    float error = setpointAngularVelocity - (-encoder.getAngularVelocity());
    float output = piController.update(error, dt);

    // refresh encoder and motor
    motor.setAngularVelocity(output);

    //motor.setAngularVelocity(setpointAngularVelocity);
}

//////////////////////////////////////////////////////////////////////////////////
// Serial Comms Handlers -> Listens to incoming packets, send response packets
//////////////////////////////////////////////////////////////////////////////////

void runSerialPrinter()
{
    static uint32_t lastPrint = 0;
    if (micros() - lastPrint >= SERIAL_LOG_RATE_US) {
        lastPrint += SERIAL_LOG_RATE_US;
        Serial.println(-encoder.getAngularVelocity());
    }
}

void runSerialListener()
{
    // Return if not available
    if(!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) setpointAngularVelocity = input.toFloat();
}