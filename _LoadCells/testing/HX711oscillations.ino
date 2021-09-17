#include <Arduino.h>
#include <Blink.h>
#include <GyroDevice.h>
#include <HX711.h>
#include <math.h>
#include <runAVG.h>

// blink led object
Blink blinky;

// 3 load cells
HX711 scale1;

// hold reading variables and filter values
runAVG filter1;
runAVG filterAnalog;
float tmp1;

// timers
elapsedMillis sw;  // data timer
elapsedMillis tm;  // loop control

// controller
GyroDevice gyro;
//IntervalTimer timer;

float twopi = 3.14159 * 2;
unsigned long lastTime;
unsigned long SampleTime;
float phase = 0.0f;
elapsedMicros usec = 0;

bool even = false;

float _freq = 0.0002F;
float _amplitude = 0.5F;

double manualInput = 0;

//cheap moving average filter
int dataPointsCount = 15;
double values[15];
double out = 0;
uint8_t k;  // k stores the index of the current array read to create a circular memory through the array

void setup() {
    analogWriteFrequency(20, 23437.5);  // Teensy 3.0 pin 3 also changes to 375 kHz

    blinky.begin(13, 500);
    gyro.begin();
    Serial.begin(115200);

    scale1.begin(loadDAT, loadCLK);
    scale1.set_scale(106146);

    scale1.tare(20);
    filter1.begin();
    filterAnalog.begin();

    // use potentiometer to manually adjust position
    pinMode(17, INPUT);
}

void loop() {
    blinky.update();

    manualInput = filterAnalog.smoothReading(analogRead(17));
    // 1440 == 360° * 4, full potentimeter rotation == 1440 degrees
    manualInput = mapfloat(manualInput, 0, 1023, 0, 1440);

    // used for calibration
    gyro.updatePitch(manualInput);
    gyro.updateGyro();

    if (scale1.is_ready() && (sw > 5)) {
        tmp1 = filter1.smoothReading(scale1.get_units());

        Serial.println((String)"<," +
            tm + "," +    // 1
            tmp1 + "," +  // 2
            "0" + "," +   // 3
            "0" + "," +   // 4
            "0" + "," +   // 5
            "0" + "," +   // 6
            "0" + "," +   // 7
            ">");
        sw = 0;
    }
}

double mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
    return (double)(x - in_min) * (out_max - out_min) / (double)(in_max - in_min) + out_min;
}

float updateSine() {
    phase += micros();
    float val = sinf(phase * _freq) * _amplitude;
    if (phase >= twopi) phase = 0;
    return val;
}
