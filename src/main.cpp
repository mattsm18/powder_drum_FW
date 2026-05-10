
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
#define ISR_FREQ_HZ 32000
#define ENCODER_SAMPLE_RATE_US 5000
#define ENCODER_EMA_FILTER_TIME_CONST 0.5f // Low-Pass Noise filter

// Serial Comms
#define SERIAL_BAUD_RATE 115200

// Default Controller
#define KP 1.0f
#define KI 2.0f
#define INTEGRAL_LIMIT 20.0f
#define DEADBAND 0.0f

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
AS5600 encoder(ENCODER_SAMPLE_RATE_US, ENCODER_EMA_FILTER_TIME_CONST);
StepperMotor motor(DRIVER_STEP_PIN, DRIVER_DIR_PIN, Microstep::THIRTY_SECOND, 200, ISR_FREQ_HZ);
PIController controller(KP, KI, INTEGRAL_LIMIT, DEADBAND);
SerialHandler serialComms;

//*** Main Program ***/
void setup() {

    // Start Serial Comms
    serialComms.begin(SERIAL_BAUD_RATE);
    mapSerialParameters(); 

    // Setup light relay pin
    pinMode(LED_PANEL_RELAY_PIN, OUTPUT);

    // Setup ISR to Fire StepperMotor ticks atomically
    setupISR();
}

void loop() {

    runSpeedControl();
    serialComms.update();

    // Update the light relay
    (lightState > 0) ? digitalWrite(LED_PANEL_RELAY_PIN, HIGH) : digitalWrite(LED_PANEL_RELAY_PIN, LOW);
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
            case 0x01: setpoint = value;                    break;
            case 0x02: accelRate = value;                   break;
            case 0x20: controller.setKp(value);             break;
            case 0x21: controller.setKi(value);             break;
            case 0x23: controller.setIntegralLimit(value);  break;
            case 0x24: controller.setDeadband(value);       break;
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
            case 0x10: return encoder.getAngularVelocity();
            case 0x20: return controller.getKp();
            case 0x21: return controller.getKi();
            case 0x23: return controller.getIntegralLimit();
            case 0x24: return controller.getDeadband();
            case 0x30: return lightState;
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
    float error = rampedSetpoint - encoder.getAngularVelocity();
    float output = controller.update(error, dt);
    motor.setAngularVelocity(output);
}