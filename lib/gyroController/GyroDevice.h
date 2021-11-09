#ifndef GYRO_DEVICE_h
#define GYRO_DEVICE_h

#include <Arduino.h>
#include <Geometry.h>
#include <PinDef.h>
//#include <GyroCom.h>
#include <GyroAxle.h>               
#include <GyroBrake.h>
#include <GyroFlywheel.h>

class GyroDevice {
private:

    GyroFlywheel flywheel;
    GyroBrake brake;
    GyroAxle yawAxle;                                 // list of components per controller
    GyroAxle pitchAxle;
    //GyroCom com;

    float diskMass = 0.096;                          //    96g
    float radious = 0.050f;                           //  10cm diameter
    float diskAngVel = 753.0f;                       // max speed at constant v (rad/s),standard disk at 7200 rpm

    float yawPos = 0;
    float pitchPos = 0;

    const float inertiaX = 0.25f * diskMass * pow(radious, 2);
    const float inertiaY = 0.25f * diskMass * pow(radious, 2);
    const float inertiaZ = 0.5f * diskMass * pow(radious, 2);

    Matrix<3, 3> I;                                   // Diagonal Matrix

    Point omegaDisk, omegaDotDisk, omegaGimbal;// vectors

    Point Mflywheel;
    Matrix<3, 3> rot_a_to_d;                           // rotation matrix

    Point longEqTorque;
    Point shortEqTorque;

    void plotResult(float* x, float* y, float* z);

    bool returnHome = true;
    float returnHomeSpeed = 20.0f;
    void retHome();


public:

    GyroDevice();
    virtual ~GyroDevice();

    void begin();                                     // initialize controller
    void refreshReading();                            // update position values for encoder and brake 
    void updatePosition();                            // actuation the gimbals to move to desire position
    void calculateTorque();                           // update the calculation of the output torque
    void calculateDisplacemnet();
    bool reachTargetAngles();

    void setGimbalSpeed(float _vel);
    // close instantanly for a period of time in milliseconds
    void brakeOn(uint32_t duration) { brake.close(duration); };
    // close the brake gradually over a period of time (input in milliseconds)
    void brakeSmooth(uint32_t duration) { brake.closeSmooth(duration); };
    // disk speed 0 - 100
    void flywheelSpeed(int8_t speed) { flywheel.setSpeed(speed); };
    // desired target angle per axle
    void pitchTargetAngle(double angle) { pitchAxle.targetAngle(angle); };
    void yawTargetAngle(double angle) { yawAxle.targetAngle(angle); };
    // return yaw value
    int getYaw() { return (float)yawAxle.readings.angle; };
    int getPitch() { return (float)pitchAxle.readings.angle; };

};
#endif                                                // GYRO_DEVICE_h

