
/*
Title: main.cpp
Author: Matthew Smith
Date: 29/04/26
Purpose:
-  Entry point for powder drum FW
-  Initialise pins
-  Handle primary function calls for all components in the FW
*/

#include <Arduino.h>
#include <AS5600.h>
#include <StepperMotor.h>
#include "pins.h"

// Define sampling and filtering parameters
#define ISR_FREQ_HZ 50000
#define ENCODER_SAMPLE_RATE_US 5000
#define ENCODER_EMA_FILTER_TIME_CONST 0.05

// Function Declaration
void setupISR();

// Instantiate Objects with timing parameters
AS5600 encoder(ENCODER_SAMPLE_RATE_US, ENCODER_EMA_FILTER_TIME_CONST);
StepperMotor motor(DRIVER_STEP_PIN, DRIVER_DIR_PIN, Microstep::QUARTER, 200, ISR_FREQ_HZ);

void setup() {

  // Start serial comms -> baud: 115200
  Serial.begin(115200);
  
  // Init 5V Relay module pin
  pinMode(LED_PANEL_RELAY_PIN, OUTPUT);

  // Setup Timer Interrupt 
  setupISR();

}

void loop() {
    encoder.update();

    // Serial print — rate-limited separately (every 50ms)
    static uint32_t lastPrint = 0;
    if (micros() - lastPrint >= 50000) {
        lastPrint += 50000;
        Serial.println(-encoder.getAngularVelocity());
    }

    // Serial input
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0)
            motor.setAngularVelocity(input.toFloat());
    }
}


//////////////////////////////////////////////////////////////////////////////////
// HARDWARE INTERRUPT SERVICE ROUTINE (ISR) -> Fixed clock for motor updates
//////////////////////////////////////////////////////////////////////////////////

// Setup function for Hardware interrupts, used to drive StepperMotors atomically
void setupISR() {
    cli();
    TCB0.CTRLA   = 0;
    TCB0.CTRLB   = TCB_CNTMODE_INT_gc;
    TCB0.CCMP = (F_CPU / ISR_FREQ_HZ) - 1;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CTRLA   = TCB_ENABLE_bm | TCB_CLKSEL_CLKDIV1_gc;
    sei();
}

// Attach ISR to StepperMotor step function
ISR(TCB0_INT_vect) {
    motor.tick();
    TCB0.INTFLAGS = TCB_CAPT_bm;

}
