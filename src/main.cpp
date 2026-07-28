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
#define ENCODER_FILTER_TIME_CONST 0.5f // Low-Pass Noise filter

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
AS5600 encoder(ENCODER_SAMPLE_RATE_US, ENCODER_FILTER_TIME_CONST);

StepperMotor motorA(
    TB6600_DRIVER_A_PUL, TB6600_DRIVER_A_DIR, TB6600_DRIVER_A_ENA, 
    Microstep::THIRTY_SECOND, 200, 
    ISR_FREQ_HZ
);

PIController controller(KP, KI, INTEGRAL_LIMIT, DEADBAND);

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
    motorA.setAngularVelocity(30.0);

    //runSpeedControl();
    //serialComms.update();

    // Update the light relay
    //(lightState > 0) ? digitalWrite(RELAY_OUTPUT_B, HIGH) : digitalWrite(RELAY_OUTPUT_B, LOW);
}

//////////////////////////////////////////////////////////////////////////////////
// HARDWARE INTERRUPT SERVICE ROUTINE (ISR) -> Fixed clock for motor updates
//////////////////////////////////////////////////////////////////////////////////

// Setup function for Hardware interrupts, used to drive StepperMotors atomically
// Uses Timer1 (16-bit) in CTC mode, no prescaling.
// OCR1A = (F_CPU / ISR_FREQ_HZ) - 1 must be <= 65535 to fit in the 16-bit register.
void setupISR() 
{
    cli();
    TCCR1A = 0;                          // Normal port operation, CTC handled via TCCR1B
    TCCR1B = (1 << WGM12);                // CTC mode (OCR1A as TOP)
    OCR1A  = (F_CPU / ISR_FREQ_HZ) - 1;   // Compare match value
    TIMSK1 = (1 << OCIE1A);               // Enable Timer1 Compare A interrupt
    TCCR1B |= (1 << CS10);                // Start timer, no prescaling (clk/1)
    sei();
}

// Attach ISR to StepperMotor step function
ISR(TIMER1_COMPA_vect) 
{
    motorA.tick();
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
    motorA.setAngularVelocity(output);
}