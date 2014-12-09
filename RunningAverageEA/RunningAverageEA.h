#ifndef RunningAverageEA_h
#define RunningAverageEA_h
//
//    FILE: RunningAverageEA.h
//  AUTHOR: Rob dot Tillaart at gmail dot com
// PURPOSE: RunningAverageEA library for Arduino
//     URL: http://arduino.cc/playground/Main/RunningAverageEA
// HISTORY: See RunningAverageEA.cpp
//
// Released to the public domain
//

// backwards compatibility
// clr() clear()
// add(x) addValue(x)
// avg() getAverage()

#define RunningAverageEA_LIB_VERSION "0.2.04"

#include "Arduino.h"

class RunningAverageEA
{
public:
    RunningAverageEA(void);
    RunningAverageEA(int);
    ~RunningAverageEA();

    void clear();
    void addValue(float);
    void fillValue(float, int);

    float getAverage();
    float getStandardDeviation();

    float getElement(uint8_t idx);
    uint8_t getSize() { return _size; }
    uint8_t getCount() { return _cnt; }

protected:
    uint8_t _size;
    uint8_t _cnt;
    uint8_t _idx;
    float   _sum;
    float * _ar;
};

#endif
// END OF FILE