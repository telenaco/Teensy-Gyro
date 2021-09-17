#include "GyroAxle.h"

GyroAxle::GyroAxle() {}

GyroAxle::~GyroAxle() {}

void GyroAxle::begin(int8_t IN1, int8_t IN2, int8_t DATA, int8_t CLK, int8_t CS, float ratio) {

    _IN1 = IN1;
    _IN2 = IN2;

    pinMode(_IN1, OUTPUT);
    pinMode(_IN2, OUTPUT);

    encoder.begin(CS, CLK, DATA);
    encoder.setZero();

    _ratio = ratio;

    this->reset();
    /* adjust the values below to filter position data */
    float _refreshLoop = 500.0f;                                                        // update loop, ms

    _velFilter.begin(1.0f, _refreshLoop);
    _accFilter.begin(1.0f, _refreshLoop);

    filterPosition = new RunningMedian(10);
    filterVelocity = new RunningMedian(60);
    filterAcceleration = new RunningMedian(60);
}

void GyroAxle::reset() {

    _velFilter.reset();
    _accFilter.reset();

    readings.time = 0;
    readings.pos = 0;
    readings.angle = 0;
    readings.vel = 0;
    readings.acc = 0;
    previousReadings.time = 0;
    previousReadings.pos = 0;
    previousReadings.angle = 0;
    previousReadings.vel = 0;
    previousReadings.acc = 0;
}

void GyroAxle::updateReadings() {

    readings.time = micros();                                                             // update time and delta time 
    _deltaTime = ((float)(readings.time - previousReadings.time));

    readings.pos = encoder.readEnc();                                                     // refresh position

    filterPosition->add(readings.pos);
    readings.pos = filterPosition->getAverage(3);

    readings.angle = (readings.pos / _ratio);                                             // ratio value is given at the class init, ratio == encoder steps per degree                                                                   
    readings.radians = ((readings.pos / _ratio) * (PI / 180.0f));                         // radians 

    readings.vel = (((readings.radians - previousReadings.radians)) / _deltaTime) * 1e6;  // deg/s                                                                  
    filterVelocity->add(readings.vel);
    readings.vel = filterVelocity->getAverage(10);
    readings.vel = _velFilter.filter(readings.vel);
    if ((readings.vel <= 0.1f && readings.vel >= 0) ||                                   // if there is any noise when the axis is not moving remove it 0.1 rad/s ~= 1 rpm
        (readings.vel >= -0.1f && readings.vel <= 0)) {
        readings.vel = 0;
    }

    readings.acc = ((readings.vel - previousReadings.vel) / _deltaTime) * 1e6;            // deg/s^2
    filterAcceleration->add(readings.acc);
    readings.acc = filterAcceleration->getAverage(10);
    readings.acc = _accFilter.filter(readings.acc);
    if ((readings.acc <= 0.1f && readings.acc >= 0) ||
        (readings.acc >= -0.1f && readings.acc <= 0)) {
        readings.acc = 0;
    }

    /* update previous readings with current readings for next time the loop runs */
    previousReadings.time = readings.time;
    previousReadings.pos = readings.pos;
    previousReadings.angle = readings.angle;
    previousReadings.radians = readings.radians;
    previousReadings.vel = readings.vel;
    previousReadings.acc = readings.acc;

    /* proportional  */

    _error = fabs(_targetAngle - readings.angle);

    /* integral */
    errorIntegral = errorIntegral + _error * _deltaTime;
    if (_error < _deadband) errorIntegral = 0.0f;

    /* derivative */
    float dedt = (_error - prevError) / (_deltaTime);

    prevError = _error;

    _speed = (kp * _error)+(ki * errorIntegral) + (kd * dedt);   
}

void GyroAxle::updatePosition() {
    this->updateReadings();

    // Serial.print("target angle -> ");
    // Serial.print(_targetAngle);
    // Serial.print(" reading angle -> ");
    // Serial.print(readings.angle);
    // Serial.print(" error -> ");
    // Serial.print(_error);

    if (_error <= 1.0) {
        digitalWrite(_IN1, LOW);
        digitalWrite(_IN2, LOW);
        //Serial.println(" zero speed");
    }

    else if (_targetAngle >= readings.angle) {
        // backwards
        _speed += (minSpeed - 6);
        _speed = (_speed >= maxSpeed) ? maxSpeed : _speed;  // if speed is above max, cap it to max value

        // Serial.print(" speed is -> ");
        // Serial.print(_speed);
        // Serial.println(" backwards ");

        _speed = map(_speed, 0, 100, 0, 255);

        analogWrite(_IN1, 0);
        analogWrite(_IN2, _speed);
    }
    else if (_targetAngle <= readings.angle) {
        // forwards
        _speed += (minSpeed - 6);

        _speed = (_speed >= maxSpeed) ? maxSpeed : _speed;  // if speed is above max, cap it to max value

        // Serial.print(" speed is -> ");
        // Serial.print(_speed);
        // Serial.println(" forward ");

        _speed = map(_speed, 0, 100, 0, 255);

        analogWrite(_IN1, _speed);
        analogWrite(_IN2, 0);
    }
}

void GyroAxle::setMaxVel(int16_t _vel) {
    maxSpeed = _vel;
}
void GyroAxle::setMinVel(int16_t _vel) {
    minSpeed = _vel;
}

void GyroAxle::setPidVal(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void GyroAxle::targetAngle(double pos) {
    reachedTarget = false;
    _targetAngle = pos;
}

void GyroAxle::printAngle() {
    Serial.println(readings.angle);
}

void GyroAxle::printSpeed() {
    Serial.println(_speed);
}

bool GyroAxle::reachTarget() {
    if (_error <= 2) return true;
    else return false;
}

axleReadings GyroAxle::getReadings() {
    return readings;
}
