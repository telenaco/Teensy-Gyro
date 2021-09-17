#include <GyroBrake.h>
#include <Arduino.h>
#include <Servo.h>


void GyroBrake::begin(uint8_t pin) {
    _pin = pin;
    _brake.attach(_pin);
    delay(100);
    _isClosing = false;
    _closeSmooth = false;
}

void GyroBrake::update() {
    if (!_isActive)
        return;                                      // if brake is not active do nothing 

    uint32_t now = micros();
    if (_isClosing) {
        if ((now - _closingTimer <= _closeInterval) && _closeSmooth) this->closeSmooth();
        else if (now - _closingTimer <= _closeInterval && !_closeSmooth ) this->close();
        else if (now - _closingTimer >= _closeInterval) this->open();
    }
    else if (!_isClosing) {
        if (now - _openingTimer <= _openInterval) {
            this->open();
        }
        else if (now - _openingTimer >= _openInterval) {
            _isActive = false;
            _closeSmooth = false;
            _brake.detach();
        }
    }
}

/**
 * @brief close the brake for the duration specified
 * 
 * @param duration microseconds that the brake is to stay close for
 */
void GyroBrake::close(uint32_t duration) {
    if (!_isClosing) {
        _brake.attach(_pin);
        _closingTimer = micros();
        _closeInterval = duration;
        _isActive = true;
        _isClosing = true;
    }
    _brake.write(closePos);
}

/**
 * @brief close the brake over the period of time defined by the duration
 * 
 * @param duration time it takes to close the brake
 */
void GyroBrake::closeSmooth(uint32_t duration) {
     static float curBrakePos = 0;                   // close initial position
     static float closingStep = 0;
    if (!_isClosing) {
        _closeSmooth = true;
        _closingTimer = micros();
        _closeInterval = duration;
        _isActive = true;
        _isClosing = true;
        curBrakePos = 0;
        closingStep = closePos / duration;
    }
    curBrakePos +=  closingStep;
    _brake.write((int)curBrakePos);
}

void GyroBrake::open(uint32_t duration) {
    if (_isClosing) {
        _openingTimer = micros();
        _openInterval = duration;
        _isClosing = false;
    }
    _brake.write(openPos);
}