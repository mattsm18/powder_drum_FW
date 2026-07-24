#ifndef PICONTROLLER_H
#define PICONTROLLER_H

#include "Arduino.h"

class PIController {
public:
    //*** Constructor ***/
    PIController
    (
        float kP, 
        float kI, 
        float integralLimit,
        float deadband
    );

    //*** Methods ***/
    float update(float error, float dt);
    void reset();

    // Setters
    void setKp(float kP);
    void setKi(float kI);
    void setIntegralLimit(float intLimit);
    void setDeadband(float deadband);

    // Getters
    float getKp()             { return _kP; }
    float getKi()             { return _kI; }
    float getIntegralLimit()  { return _integralLimit; }
    float getDeadband()       { return _deadband; }

private:
    float _kP;
    float _kI;
    float _integralLimit;
    float _integral;
    float _deadband;
};

#endif