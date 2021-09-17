#ifndef GYROAXLE_H
#define GYROAXLE_H

#include <Arduino.h>
#include <AS5045.h>                                                                 // encoder  
#include <LowPassFilter.h>                                                          // readings filter
#include <RunningMedian.h>

typedef struct axleReadings {
    double  time = 0;                                                                // micros reading
    double  pos = 0;                                                                 // encoder steps
    double  angle = 0;                                                               // conversion from steps to angle
    double  radians = 0;                                                             // conversion from steps to radians
    double  vel = 0;                                                                 // radians /s
    double  acc = 0;                                                                 // angular velocity radians/s^2
}axleReadings;

class GyroAxle {
private:

    RunningMedian* filterPosition;
    RunningMedian* filterVelocity;
    RunningMedian* filterAcceleration;

    //H_bridge motorHbridge;                                                          // axel actuation motor 
    AS5045   encoder;                                                               // with paired encoder on shaft
    //PID* speedcontrol;

    uint8_t _IN1;
    uint8_t _IN2;

    int16_t  minSpeed = 0;   // min speed needed to overcome the friction of the gimbal, use method 
    int16_t  maxSpeed = 100; // constrain the max speed to avoid braking it 😓, 🤞 I finish the user studies 👍🙏

    /* Input;(readings.pos) the analogue value read into the mController
       Output;(speed) the pulse width modulation output to a pin
       Setpoint;(desiredPosition) the value that Output should work towards, the ideal value */
    double _targetAngle = 0.0;
    double _speed = 0.0;
    double _error = 0.0;
    double _deltaTime = 0;

    double errorIntegral;
    double prevError;
    /* Kp:Determines how aggressively the PID reacts to the current amount of error (Proportional)
       Ki:Determines how aggressively the PID reacts to error over time (Integral)
       Kd:Determines how aggressively the PID reacts to the change in error (Derivative)  */
    double kp = 1.0f, ki = 0.0f, kd = 0.0f;

    // LowPassFilter _posFilter;                                                     
    // LowPassFilter _angFilter;
    LowPassFilter _velFilter;
    LowPassFilter _accFilter;
    // LowPassFilter _speedFilter;



    double  _ratio = 0;                                                            // ration of steps per degree of rotation, constant dependent of the gear ratio size
    double  _deadband = 5.0f;                                                      // deadband degrees for the axle, to avoid jitter when desire position is reached.
    bool reachedTarget = true;

public:


    GyroAxle();
    virtual ~GyroAxle();
    axleReadings readings;                                                          // data containers for readings (time, pos, angle, radians,vel, acc)
    axleReadings previousReadings;
    axleReadings getReadings();                                                     // return current axleReadings from sensor

    void begin(int8_t IN1, int8_t IN2,                                   // h-bridge pins
        int8_t DATA, int8_t CLK, int8_t CS,                                         // encoder pins
        float ratio);                                                              // pulley ratio refresh loop   

    void         reset();                                                           // reset  readings and filter values, set them to 0
    void         updateReadings();                                                  // read encoder and update calculation of angle,velocity and acceleration
    void         updatePosition();                                                  // maintain the axes at the desire position 
    void         setMaxVel(int16_t _vel);                                               // speed control to limit max speed of gimbal actuation
    void         setMinVel(int16_t _vel);
    void         setPidVal(double _kp, double _ki, double _kd);
    void         targetAngle(double _pos);
    void         printAngle();
    void         printSpeed();
    bool         reachTarget();
};

#endif // GYROAXLE_H
