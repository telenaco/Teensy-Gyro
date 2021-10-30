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
    float _refreshLoop = 500.0f;         // update loop, ms 

    _velFilter.begin(1.0f, _refreshLoop);
    _accFilter.begin(1.0f, _refreshLoop);

    filterPosition = new RunningMedian(5);
    filterVelocity = new RunningMedian(3);
    filterAcceleration = new RunningMedian(3);
}

void GyroAxle::reset() {

    _velFilter.reset();
    _accFilter.reset();

    readings.time  = 0;
    readings.pos   = 0;
    readings.angle = 0;
    readings.vel   = 0;
    readings.acc   = 0;
    previousReadings.time  = 0;
    previousReadings.pos   = 0;
    previousReadings.angle = 0;
    previousReadings.vel   = 0;
    previousReadings.acc   = 0;
}

void GyroAxle::updateReadings() {

    // update a data package
    // - Time
    readings.time = micros();
    _deltaTime = ((float)(readings.time - previousReadings.time));
    // - Current encoder position (encoder steps per turn)
    readings.pos = encoder.readEnc();
    filterPosition->add(readings.pos);
    readings.pos = filterPosition->getMedian();
    // - Current angle 
    readings.angle = (readings.pos / _ratio);
    // - Current radians
    readings.radians = radians(readings.angle);
    // - Current velocity
    readings.vel = (((readings.radians - previousReadings.radians)) / _deltaTime) * 1e6;                                                 
    filterVelocity->add(readings.vel);
    readings.vel = filterVelocity->getAverage(10);
    readings.vel = _velFilter.filter(readings.vel);
    if ((readings.vel <= 0.1f && readings.vel >= 0) ||                                   // if there is any noise when the axis is not moving remove it 0.1 rad/s ~= 1 rpm
        (readings.vel >= -0.1f && readings.vel <= 0)) {
        readings.vel = 0;
    }
    // - Current acceleration
    readings.acc = ((readings.vel - previousReadings.vel) / _deltaTime) * 1e6;           // rad/s^2
    filterAcceleration->add(readings.acc);
    readings.acc = filterAcceleration->getAverage(10);
    readings.acc = _accFilter.filter(readings.acc);
    if ((readings.acc <= 0.1f && readings.acc >= 0) ||
        (readings.acc >= -0.1f && readings.acc <= 0)) {
        readings.acc = 0;
    }

    /* update previous readings with current readings for next time the loop runs */
    previousReadings.time = readings.time;
    previousReadings.radians = readings.radians;
    previousReadings.vel = readings.vel;

    // Serial1.print(_targetAngle);
    // Serial1.print(',');
    // Serial1.print(readings.pos);
    // Serial1.print(',');
    // Serial1.print(readings.radians);
    // Serial1.print(',');
    // Serial1.print(readings.angle);
    // Serial1.print(',');
    // Serial1.print(readings.vel);
    // //Serial1.print(_error);
    // Serial1.print(',');
    // Serial1.print(readings.acc);
    // Serial1.print(',');
    // Serial1.print(millis());
    // Serial1.println("");

    /* proportional  */
    // calculate the shorter path to the desire angle
    // https://math.stackexchange.com/questions/110080/shortest-way-to-achieve-target-angle
    float tmp[3];
    // fmod (float modulus)
    float curPos;
    curPos = readings.angle < 0 ? 360 + fmodf(readings.angle, 360.0f) : fmodf(readings.angle, 360.0f);

    tmp[0] = _targetAngle - curPos;
    tmp[1] = (_targetAngle - curPos) + 360;
    tmp[2] = (_targetAngle - curPos) - 360;

    // the lowest value of the array above determines the shortest path
    // if negative value is rotate CW, if positive CCW
    _error = 720;
    for (int i = 0; i <= 2; i++) {
        if (_error > fabs(tmp[i])) {
            _error = fabs(tmp[i]);
            // determine direction of min value
            (tmp[i] < 0)? _direction = true : _direction = false;
            //(tmp[i] < 0)? Serial1.print("clockwise") : Serial1.print("Cclockwise") ;
        }     
    }

  


    float static _prevError = 0;
    float static _errorIntegral = 0;

    /* integral */
    _errorIntegral = _errorIntegral + _error * _deltaTime;
    // the below solve some inestability on the pid at 0
    if (_error < _deadband) _errorIntegral = 0.0f;

    /* derivative */
    float dedt = (_error - _prevError) / (_deltaTime);

    //             P                I                       D 
    _speed = (_kp * _error) + (_ki * _errorIntegral) + (_kd * dedt);
    
    _prevError = _error;

    Serial1.print("target angle ->");
    Serial1.print(_targetAngle);
    Serial1.print("  curPos  -> ");
    Serial1.print(curPos);
    Serial1.print("  error  -> ");
    Serial1.print(_error);
    Serial1.print("  Kp Ki Kd ");
    Serial1.print((String)_kp + " , " + _ki + " , " + _kd);
    Serial1.print("  minSpeed  -> ");
    Serial1.print(maxSpeed);
    Serial1.print("  speed  -> ");


    // give it a kick to overcome friction if gimbal is stop
    if (_speed <= minSpeed) {
        _speed += minSpeed;
    }
    else if (_speed >= maxSpeed)
    {
        _speed = maxSpeed;
    }
    


    //constrain(_speed, minSpeed, maxSpeed);
    Serial1.print(_speed);
    Serial1.println("");


    // convert motor speed % to 0-255
    _speed = map(_speed, 0, 100, 0, 255);

        
   
    

    // // deadband speed, override PID
    if (fabs(_error) <= _deadband) _speed = 0;
    Serial1.print(_speed);
    Serial1.println("");
}



void GyroAxle::updatePosition() {

    // error too small 
    if (_error <= _deadband) {
        digitalWrite(_IN1, LOW);
        digitalWrite(_IN2, LOW);
        Serial1.println("zero");
        Serial1.println("");
        return;
    }
    
    if (!_direction) {
        // CCW looking from the Z axis down
        analogWrite(_IN1, 0);
        analogWrite(_IN2, _speed);
        Serial1.println("CCW");
        Serial1.println("");
    }
    else if (_direction) {
        // CW looking from the Z axis down
        analogWrite(_IN1, _speed);
        analogWrite(_IN2, 0);
        Serial1.println("CW");
        Serial1.println("");
    }
}

void GyroAxle::targetAngle(double pos) {
    _reachedTarget = false;
    _targetAngle = pos;
}

bool GyroAxle::reachTarget() {
    if (_error <= _deadband) return true;
    else return false;
}
