#ifndef GYRO_DEVICE_h
#define GYRO_DEVICE_h

#include <Arduino.h>
#include <Geometry.h>
#include <PinDef.h>
#include <GyroCom.h>
#include <GyroAxle.h>               
#include <GyroBrake.h>
#include <GyroFlywheel.h>

class GyroDevice {
private:

    GyroFlywheel flywheel;
    GyroBrake brake;
    GyroAxle yawAxle;                                 // list of components per controller
    GyroAxle pitchAxle;
    GyroCom com;

    float diskMass = 0.192f;                          //    96g
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
    void updatePosition();                            // actuation of the gimbals
    void calculateTorque();                           // update the calculation of the output torque
    void sendTorque();
    void sendData();
    void sendFormulaTwo();
    void printData();
    void setGimbalSpeed(int _vel);
    bool reachTargetAngles();

    /* debug methods */
    void brakeOn(uint32_t duration);                  // 1           
    void brakeSmooth(uint32_t duration);              // 2
    void flywheelSpeed(int8_t speed);                 // 5
    void pitchTargetAngle(double angle);               // 6
    void yawTargetAngle(double angle);                 // 7

    /* print/plot methods */
    void printPitchPos();
    void printYawPos();
    void plotYawReadings();
    void plotPitchReadings();
    void printSpeeds();
    int getYaw();
    int getPitch();

};
#endif                                                // GYRO_DEVICE_h

