/*
Title: MovingAverage.h
Author: Matthew Smith 22173112
Date: 03/08/26

Purpose:
- Fixed-size moving window average filter
*/

#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H

#include <stdint.h>

class MovingAverage
{
public:
    // Constructor
    explicit MovingAverage(uint16_t windowSize);

    // Destructor
    ~MovingAverage();

    // Methods
    float update(float sample);
    void reset(float value = 0.0f);
    void resize(uint16_t newWindowSize);
    
    // Getters
    float getValue() const { return _average; }

private:
    // Configuration
    uint16_t _windowSize;

    // Circular buffer
    float* _buffer;
    uint16_t _index;
    uint16_t _count;

    // Running statistics
    float _sum;
    float _average;
};

#endif