/*
Title: EMAFilter.h
Author: Matthew Smith 22173112
Date: 2/08/26

Purpose:
- Exponential Moving Average Filter -> look at README for specific details
*/


#ifndef EMAFILTER_H
#define EMAFILTER_H

class EMAFilter
{
    public:
        // Constructor
        EMAFilter(float timeConstant);

        // Methods
        float update(float rawValue, float dt);
        void reset(float value = 0.0f);

        // Getters
        float getValue() const { return _value; }
        float getTimeConstant() const { return _timeConst; }

        // Setters
        void setTimeConstant(float timeConstant) { _timeConst = timeConstant; }

    private:
        // Attributes
        float _timeConst;
        float _value;
        bool _init;
};

#endif