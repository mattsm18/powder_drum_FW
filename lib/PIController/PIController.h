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
        float integralLimit
    );

    //*** Methods ***/
    float update(float error, float dt);
    void reset();

private:
    float _kP;
    float _kI;
    float _integralLimit;
    float _integral;
};


#endif