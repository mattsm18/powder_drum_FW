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
        float kDq
    );

    //*** Methods ***/
    float update(float error, float dt);
    void reset();

    // Setters
    void setKp(float kP);
    void setKi(float kI);
    void setKd(float kD);

    // Getters
    float getKp()             { return _kP; }
    float getKi()             { return _kI; }
    float getKd()             { return _kD; }

private:
    float _kP;
    float _kI;
    float _kD;
    float _integral;
    float _prevError;
};

#endif