/*
Title: MovingAverage.cpp
Author: Matthew Smith 22173112
Date: 03/08/26

Purpose:
- Fixed-size moving window average filter
*/

#include "MovingAverage.h"

MovingAverage::MovingAverage(uint16_t windowSize)
    : _windowSize(windowSize),
      _buffer(nullptr),
      _index(0),
      _count(0),
      _sum(0.0f),
      _average(0.0f)
{
    if (_windowSize == 0) { _windowSize = 1; }

    _buffer = new float[_windowSize];

    for (uint16_t i = 0; i < _windowSize; i++) { _buffer[i] = 0.0f; }
}

MovingAverage::~MovingAverage(){ delete[] _buffer; }

float MovingAverage::update(float sample)
{
    // Remove oldest sample once buffer is full
    if (_count == _windowSize) { _sum -= _buffer[_index]; } 
    else { _count++; }

    // Store new sample
    _buffer[_index] = sample;
    _sum += sample;

    // Advance circular buffer
    _index++;
    if (_index >= _windowSize) { _index = 0; }

    // Compute average
    _average = _sum / static_cast<float>(_count);

    return _average;
}

void MovingAverage::reset(float value)
{
    _sum = value * _windowSize;
    _average = value;
    _count = _windowSize;
    _index = 0;

    for (uint16_t i = 0; i < _windowSize; i++) {
        _buffer[i] = value;
    }
}

void MovingAverage::resize(uint16_t newWindowSize)
{
    if (newWindowSize == 0 || newWindowSize == _windowSize)
        return;

    delete[] _buffer;

    _windowSize = newWindowSize;
    _buffer = new float[_windowSize];

    reset();
}