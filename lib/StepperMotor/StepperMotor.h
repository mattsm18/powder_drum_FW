#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>

enum Direction { CW = 0, CCW = 1 };
enum Microstep { FULL = 1, HALF = 2, QUARTER = 4, EIGHTH = 8, SIXTEENTH = 16, THIRTY_SECOND = 32 };

class StepperMotor {
public:
    StepperMotor(uint8_t stepPin, uint8_t dirPin,
                 Microstep microstep = FULL,
                 uint8_t stepsPerRev = 200,
                 uint32_t isrFreq = 20000);

    void setAngularVelocity(float radsPerSec);
    float getAngularVelocity() const;

    static void tick();
    
    //temp debug
    float getP(){ return _phaseIncrement; }

private:
    static StepperMotor* _instance;

    // Hardware
    uint8_t _stepPin;
    uint8_t _dirPin;

    const uint16_t _stepsPerRev;
    const uint32_t _isrFreq;

    // Motion
    volatile float _angularVelocity = 0.0f;
    volatile float _phase = 0.0f;
    volatile float _phaseIncrement = 0.0f;

    // Pulse state machine
    volatile bool _stepHigh = false;

    Direction _direction = CW;

    void updateStepTiming();
    void stepIfDue();
};

#endif