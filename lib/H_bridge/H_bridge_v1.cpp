#include "H_bridge_v1.h"

H_bridge::H_bridge() {}
H_bridge::~H_bridge() {}

void H_bridge::begin(uint8_t enable, uint8_t in1, uint8_t in2) {
    pinMode(enable, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    _pinEnable = enable;
    _pinIn1 = in1;
    _pinIn2 = in2;

}

void H_bridge::run(uint8_t direction, uint8_t speed) {
    _speed = map(speed, 0, 100, 0, 255);

    switch (direction) {
    case BACKWARD:
        this->backward(_speed);
        break;
    case FORWARD:
        Serial.println(_speed);
        this->forward(_speed);
        break;
    case STOP:
        this->stop();
        break;
    }
}

void H_bridge::stop() {
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);

    analogWrite(_pinEnable, 0);

    _direction = STOP;
    _isMoving = false;
}

void H_bridge::forward(uint8_t speed) {
    _speed = map(speed, 0, 100, 0, 255);
    //_speed = constrain(_speed, 0, 255);

    digitalWrite(_pinIn1, HIGH);
    digitalWrite(_pinIn2, LOW);

    analogWrite(_pinEnable, _speed);

    _direction = FORWARD;
    _isMoving = true;
}

void H_bridge::backward(uint8_t speed) {

    _speed = map(speed, 0, 100, 0, 255);
    //_speed = constrain(_speed, 0, 255);

    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, HIGH);

    analogWrite(_pinEnable, _speed);

    _direction = BACKWARD;
    _isMoving = true;
}

void H_bridge::invert() {
    digitalWrite(_pinIn1, digitalRead(_pinIn1) ? 0 : 1);
    digitalWrite(_pinIn2, digitalRead(_pinIn2) ? 0 : 1);
}

uint8_t H_bridge::getSpeed() {
    return _speed;
}

bool H_bridge::isMoving() {
    return _isMoving;
}