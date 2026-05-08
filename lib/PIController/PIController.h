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
    void PIController::setKp(float kP);
    void PIController::setKi(float kI);
    void PIController::setIntegralLimit(float intLimit);
    void PIController::setDeadband(float deadband);

    // Getters
    float PIController::getKp()             { return _kP; }
    float PIController::getKi()             { return _kI; }
    float PIController::getIntegralLimit()  { return _integralLimit; }
    float PIController::getDeadband()       { return _deadband; }

private:
    float _kP;
    float _kI;
    float _integralLimit;
    float _integral;
    float _deadband;
};


#endif