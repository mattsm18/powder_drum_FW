#include "EMAFilter.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

EMAFilter::EMAFilter(float timeConstant){ _timeConst = timeConstant; };

///////////////////////////////////////////////////////////////////////////////////////////////////

float EMAFilter::update(float rawValue, float dt)
{   
    // First sample -> seed the filter to avoid a startup transient
    if(!_init){
        _value = rawValue;
        _init = true;
        return _value;
    }

    float alpha = dt / ( _timeConst + dt );
    _value = alpha * rawValue + (1.0f - alpha) * _value;

    return _value;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

void EMAFilter::reset(float value)
{ 
    _value = value; 
    _init = true;
}
