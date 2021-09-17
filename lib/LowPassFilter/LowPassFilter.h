
// One-pole recursive low-pass filter
/*
 *
 * Based on the original code from Introduction to Haptics -- an online
 * course from Stanford University.
 *
 * Original copyright:
 * --------------------------------------------------------------------------
 * Tania Morimoto and Allison Okamura, Stanford University
 * 11.16.13 - 10.16.14
  * --------------------------------------------------------------------------
 */
 // One-pole recursive low-pass filter

#ifndef LPASS_h
#define LPASS_h

#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif 


class LowPassFilter {

     float  y1 = 0.0;
     float  b1 = 0.0;
     float  a0 = 1.0;

public:
    LowPassFilter() { }
    inline void begin(float band_freq, float sampling_freq) {
        setFc(band_freq / sampling_freq);
    }
    inline void begin(float fc) {
        setFc(fc);
    }
    inline void reset() {
        y1 = 0.0;
    }
    inline void setFc(float fc) {
        
        if (fc >= 13.5)            fc = 13;
        y1 = 0.0;
        b1 = exp(-2.0 * M_PI * fc);
        a0 = 1.0 - b1;
    }

    inline float filter(float x)    {
        return y1 = x * a0 + y1 * b1;
    }
};

#endif 