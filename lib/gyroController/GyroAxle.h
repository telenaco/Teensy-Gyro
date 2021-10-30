#ifndef GYROAXLE_H
#define GYROAXLE_H

#include <Arduino.h>
#include <AS5045.h>                 // encoder  
#include <LowPassFilter.h>          // readings filter
#include <RunningMedian.h>

typedef struct axleReadings {
    double  time    = 0;            // micros reading
    double  pos     = 0;            // encoder steps
    double  angle   = 0;            // conversion from steps to angle
    double  radians = 0;            // conversion from steps to radians
    double  vel     = 0;            // radians /s
    double  acc     = 0;            // angular velocity radians/s^2
}axleReadings;

class GyroAxle {
private:

    RunningMedian* filterPosition;
    RunningMedian* filterVelocity;
    RunningMedian* filterAcceleration;

    AS5045  encoder;                // paired encoder on shaft
    
    uint8_t _IN1;
    uint8_t _IN2;

    float  minSpeed = 0;
    float  maxSpeed = 100;

    // Input;(readings.pos) analogue value 
    // Output;(speed) PWM motor actuation
    // Setpoint;(desiredPosition) desired position 
    double  _targetAngle = 0.0;
    double  _speed = 0.0;
    double  _error       = 0.0;
    double  _deltaTime   = 0;
    bool    _direction   = false;

    // Kp:how aggressively the PID reacts to the current amount of error 
    // Ki:how aggressively the PID reacts to error over time 
    // Kd:how aggressively the PID reacts to the change in error 
    double _kp = 1.0f, _ki = 0.0f, _kd = 0.0f;

    LowPassFilter _velFilter;
    LowPassFilter _accFilter;

    float   _ratio = 0;            // ratio steps/degree
    double  _deadband = 1.5f;      // deadband degrees, avoids jitter.
    bool    _reachedTarget = true;

public:


    GyroAxle();
    virtual ~GyroAxle();
    axleReadings readings;                   // data containers (time, pos, angle, radians,vel, acc)
    axleReadings previousReadings;
    axleReadings getReadings() { return readings; };   // return  axleReadings 

    void begin(int8_t IN1, int8_t IN2,              // h-bridge pins
               int8_t DATA, int8_t CLK, int8_t CS,  // encoder pins
               float ratio);                        // pulley ratio refresh loop   

    void         reset();                    // set readings and filter to 0
    void         updateReadings();           // read encoder and update pos, vel and acc
    void         updatePosition();           // move axle to desire position 
    void         setMaxVel(float _vel) { maxSpeed = _vel; };     // limits max speed of gimbal 
    void         setMinVel(float _vel) { minSpeed = _vel; };   // min speed (below 40PWM doesnt run)
    void         setPidVal(double kp, double ki, double kd) { _kp = kp; _ki = ki; _kd = kd;};
    void         targetAngle(double _pos);
    bool         reachTarget();
};

#endif // GYROAXLE_H
