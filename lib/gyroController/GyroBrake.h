#ifndef GYRO_BRAKE_h
#define GYRO_BRAKE_h

#include <Arduino.h>
#include <Servo.h>

class GyroBrake {

private:

    Servo _brake;

    int8_t  _pin;
    int16_t _timeOff;
    int16_t _timeOn;

    bool _isActive = false;
    bool _isClosing = false;
    bool _isOpening = false;
    bool _closeSmooth = false;

    uint32_t _closeInterval = 0;         // time till we change state
    uint32_t _openInterval = 0;          // time till we change state
    uint32_t _closingTimer = 0;          // when we last changed state
    uint32_t _openingTimer = 0;          // when we last changed state

    float openPos = 70.0f;
    float closePos = 100.0f;


public:


    GyroBrake() {};
    virtual ~GyroBrake() {};

    void begin(uint8_t pin);
    void update();
    void open(uint32_t duration = 400);
    void close(uint32_t duration = 500);
    void closeSmooth(uint32_t duration = 1000);
};

#endif /* GYRO_BRAKE_h */