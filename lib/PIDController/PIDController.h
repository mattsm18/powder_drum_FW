#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Arduino.h"

class PIDController {
public:
    //*** Constructor ***/
    PIDController
    (
        float kP, 
        float kI, 
        float kD,
        float integralLimit,
        float outputLimit
    );

    //*** Methods ***/
    float update(float error, float dt);
    void reset();

    // Setters
    void setKp(float kP);
    void setKi(float kI);
    void setKd(float kD);
    void setIntegralLimit(float integralLimit);
    void setOutputLimit(float outputLimit);

    // Getters
    float getKp()             { return _kP; }
    float getKi()             { return _kI; }
    float getKd()             { return _kD; }
    float getIntegralLimit()  { return _integralLimit; }
    float getOutputLimit()    { return _outputLimit; }

private:
    float _kP;
    float _kI;
    float _kD;
    float _integral;
    float _prevError;
    float _integralLimit;
    float _outputLimit;
};

#endif