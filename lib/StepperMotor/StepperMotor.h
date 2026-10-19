// StepperMotor.h
#pragma once
#include <Arduino.h>
#include <stdint.h>

enum class Microstep : uint8_t {
    FULL       = 1,
    HALF       = 2,
    QUARTER    = 4,
    EIGHTH     = 8,
    SIXTEENTH  = 16,
    THIRTY_SECOND = 32
};

class StepperMotor {
public:

    StepperMotor
    (
        uint8_t pulPin, uint8_t dirPin, uint8_t enaPin,
        Microstep microstep, uint16_t stepsPerRev,
        uint32_t tickFreqHz
    );

    void enable();
    void disable();
    bool isEnabled() const { return _enabled; }

    // --- Velocity control ---
    void setAngularVelocity(float radPerSec);
    float getAngularVelocity() const { return _setpointVelocity; }

    // --- ISR-side ---
    static void tickAll();   // call this from your hardware ISR
    void tick();             // per-instance tick, called by tickAll()

private:
    void applyStepRate(float radPerSec);

    // Pins
    uint8_t _pulPin, _dirPin, _enaPin;

    // Geometry
    uint16_t _stepsPerRev;
    Microstep _microstep;
    uint32_t _microstepsPerRev;

    // Timing
    uint32_t _tickFreqHz;

    // State
    volatile bool _enabled = false;
    volatile int8_t _direction = 1;          // +1 / -1
    volatile float _setpointVelocity = 0.0f;

    // Phase accumulator (Q32 fixed point, wraps naturally on overflow)
    volatile uint32_t _accumulator = 0;
    volatile uint32_t _accumulatorStep = 0;  // added every tick

    // Registry for multi-instance ISR dispatch
    static constexpr uint8_t MAX_INSTANCES = 4;
    static StepperMotor* _instances[MAX_INSTANCES];
    static uint8_t _instanceCount;
};