#ifndef GYRO_FLYWHEEL_h
#define GYRO_FLYWHEEL_h

#include <Arduino.h>
#include <ESC.h>

class GyroFlywheel
{
private:

    ESC     gyroESC;
    int16_t  _speed;                // speed range 0 to 100 %
    int8_t  _maxSpeed = 100;        // %
    int8_t  _minSpeed = 0;          // %
    bool    _isRunning = false;  

public:

    GyroFlywheel() {};
    virtual ~GyroFlywheel() {};

    void    begin(int8_t pin);
    void    setSpeed(int8_t speed); 
    int8_t  getSpeed();
};

#endif /* GYRO_FLYWHEEL_h */