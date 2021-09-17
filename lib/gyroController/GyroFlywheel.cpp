#include <GyroFlywheel.h>
#include <Arduino.h>

#define SPEED_MIN 1000                                               
#define SPEED_MAX 2000


void GyroFlywheel::begin(int8_t pin) {
    gyroESC.begin(pin);
    gyroESC.arm();                                          // Send the Arm value so the ESC will be ready to take commands
    gyroESC.stop();
}

void GyroFlywheel::setSpeed(int8_t speed) {

    _speed = map(speed, _minSpeed, _maxSpeed, 1000, 2000);  // 1000 - 2000 ms are hard coded on the ESC, see ESC specs
    _speed = constrain(_speed, 1000, 2000);

    if (_speed <= 1000) {                                   // stop the motor if speed is 0
        gyroESC.speed(0);
        gyroESC.stop();
        _isRunning = false;
    }
    else if (!_isRunning && _speed > 1000) {
        /* if motor is on rest and speed if over 1400 ms the motor used wont be able to start
           1400 is the limit (via trial and error) that will get the flywheel started */
        gyroESC.speed(1400);
        delayMicroseconds(1000);
        gyroESC.speed(_speed);
    }
    else if (_isRunning && _speed > 1000) {             
        gyroESC.speed(_speed);                            
        _isRunning = true;
    }
}

int8_t GyroFlywheel::getSpeed(){
    uint8_t outSpeed = map(_speed, 1000, 2000, _minSpeed, _maxSpeed);
    return outSpeed;        
}
