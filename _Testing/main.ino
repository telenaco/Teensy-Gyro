#include <Arduino.h>
#include <Blink.h>
#include <GyroDevice.h>
#include <HX711.h>
#include <elapsedMillis.h>
#include <math.h>
#include <runAVG.h>

elapsedMillis tm;
elapsedMillis sw;
unsigned long oldSw;

float tmp;

HX711 loadCell;
runAVG filterLoad;
runAVG filterAnalog;

GyroDevice gyro;
IntervalTimer timer;

float twopi = 3.14159 * 2;
unsigned long lastTime;
unsigned long SampleTime;
float phase = 0.0f;
elapsedMicros usec = 0;

bool even = false;

float _freq = 0.0002F;
float _amplitude = 0.5F;

float manualInput = 0;

void setup() {
    analogWriteFrequency(20, 23437.5);  // Teensy 3.0 pin 3 also changes to 375 kHz

    pinMode(17, INPUT);

    filterAnalog.begin();
    Serial.begin(2000000);

    gyro.begin();
    delay(2000);
    gyro.setSpeed(40);
    sw = 0;
}

void loop() {

    // if (sw > 2) {
    //     Serial.println((String) "<," +
    //                    tm + "," +               // 1
    //                    "0" + "," +              // 2
    //                    gyro.getPitch() + "," +  // 3
    //                    gyro.getYaw() + "," +    // 4
    //                    "0" + "," +              // 5
    //                    "0" + "," +              // 6
    //                    "0" + "," +              // 7
    //                    ">");
    //     sw = 0;
    // }



}

double mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
    return (double)(x - in_min) * (out_max - out_min) / (double)(in_max - in_min) + out_min;
}

// // float updateSine() {
// //     phase += micros();
// //     float val = sinf(phase * _freq) * _amplitude;
// //     if (phase >= twopi) phase = 0;
// //     return val;
// // }
