#ifndef RUNAVG
#define RUNAVG

#include <Arduino.h>
#define MAXNUMPOINTS 40

class runAVG {
public:
    int dataPointsCount;
    float values[MAXNUMPOINTS];
    float out = 0;
    uint8_t k;  // k stores the index of the current array read to create a circular memory through the array

    runAVG() {};

    void begin(int maxNumberValues = MAXNUMPOINTS) {

        if (maxNumberValues == MAXNUMPOINTS) {
            dataPointsCount = MAXNUMPOINTS;
        }
        else {
            dataPointsCount = maxNumberValues;
        }

        k = 0;  //initialize so that we start to write at index 0 for the MAF

        // fill array with values
        for (int i = 0; i < dataPointsCount; i++) {
            values[i] = 0;  // fill the array readings from the sensor
        }
    }

    float smoothReading(float newValue) {
        out = 0;

        // increase index when reach the end loop to 0
        values[k] = newValue;
        k = (k + 1) % dataPointsCount;

        for (int i = 0; i < dataPointsCount; i++) {
            out += values[i];
        }
        out /= dataPointsCount;

        return out;
    }
};

#endif
