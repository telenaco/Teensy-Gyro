#include <elapsedmillis.h>
#include <BasicLinearAlgebra.h>
#include <GyroDevice.h>
#include <blink.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <SLIPEncodedUSBSerial.h>
#include <SLIPEncodedSerial.h>
#include <gyroPinDef.h>

// encoding to send and receive message to UNITY via serial com
SLIPEncodedUSBSerial SLIPSerial(Serial);

// for debugging purposes, keep LED blinking 
Blink blinky;

// controller class
GyroDevice gyroController;

bool impact = false;

// data received on a message, new goal position
struct DataPackage {
    float RotX = 0.0f;
    float RotY = 0.0f;
    float RotZ = 0.0f;
};

DataPackage dataPackage;
float increment = 0.0f;

void setup() {
    SLIPSerial.begin(2000000);
    // debug, echo data via serial 1
    Serial1.begin(115200);
    Serial1.println("done setup");

    // initialize controller (PID values, encoder steps...) declared inside the class
    gyroController.begin();
    // keep blinking at a rate of 0.5s
    blinky.begin(13, 500);

    // on start start moving the flywheel
    gyroController.flywheelSpeed(22);
}

void loop() {
    // show that the controller is alive and running
    blinky.update();

    // receive msg from unity and call function for the message  
    updateUnityMsg();
    
    // calculate the required acceleration

    // motor velocity is always same 

    gyroController.refreshReading();
    gyroController.updatePosition();
    updatePosition();
    

    increment += 0.03;
    Serial1.println(increment);

}

void updatePosition() {
         gyroController.pitchTargetAngle(increment);
         //gyroController.pitchTargetAngle(dataPackage.RotY);
         gyroController.yawTargetAngle(increment);
         //gyroController.yawTargetAngle(dataPackage.RotZ);
}

/**
 * @brief read a bynary stream encoded using SLIP and OSC.
 */
void updateUnityMsg() {


    OSCMessage msgIn;
    int size;
    unsigned long start = millis();
    unsigned long timeout = 12;

    while (!SLIPSerial.endofPacket()) {
        if ((size = SLIPSerial.available()) > 0) {
            while (size--) {
                msgIn.fill(SLIPSerial.read());
            }
        }
        unsigned long now = millis();
        unsigned long elapsed = now - start;
        // if there is no new message return without updating data
        if (elapsed > timeout) return;
    }

    if (!msgIn.hasError()) {
        msgIn.dispatch("/yawI",     yawIncrement);
        msgIn.dispatch("/pitchI",   pitchIncrement);
        msgIn.dispatch("/yawS",     yawSetAngle);
        msgIn.dispatch("/pitchS",   pitchSetAngle);
        msgIn.dispatch("/G_vel",    setGimbalVel);
        msgIn.dispatch("/F_vel",    setFlywheelVel);
        msgIn.dispatch("/data",     refreshData);
        msgIn.dispatch("/sPID",     setPidValues);
    }     
}

// TODO
void yawIncrement(OSCMessage& msg) {
    dataPackage.RotZ += msg.getFloat(0);
}

void pitchIncrement(OSCMessage& msg) {
    dataPackage.RotY += msg.getFloat(0);    
}

void yawSetAngle(OSCMessage& msg) {
    dataPackage.RotZ = msg.getFloat(0);
    Serial1.print("yaw set -> ");
    Serial1.println(dataPackage.RotZ);
}

void pitchSetAngle(OSCMessage& msg) {
    dataPackage.RotY = msg.getFloat(0);    
    Serial1.print("pitch set -> ");
    Serial1.println(dataPackage.RotY);
}

void setGimbalVel(OSCMessage& msg) {
    float vel = msg.getFloat(0);
    gyroController.setGimbalSpeed(vel);
    Serial1.print("gimbal vel -> ");
    Serial1.println(vel);
}

void setPidValues(OSCMessage& msg) {
    int check = msg.getInt(0);
    if (check) {
        gyroController.setPidImpact();
    }
    else {
        gyroController.setPidMove();
    }
    Serial1.print("pid settings -> ");
    Serial1.println(check);
}

void setFlywheelVel(OSCMessage& msg) {
    float vel = msg.getFloat(0);
    gyroController.flywheelSpeed(vel);
    Serial1.print("flywheel vel -> ");
    Serial1.println(vel);
}

void refreshData(OSCMessage& msg) {
        dataPackage.RotX =  msg.getFloat(0);
        dataPackage.RotY =  msg.getFloat(1);
        dataPackage.RotZ = msg.getFloat(2);
}


