#include <elapsedmillis.h>
#include <BasicLinearAlgebra.h>
#include <GyroDevice.h>
#include <blink.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <SLIPEncodedUSBSerial.h>
#include <SLIPEncodedSerial.h>
#include <gyroPinDef.h>

// encoding to send and receive message to UNITY
SLIPEncodedUSBSerial SLIPSerial(Serial);


// for debugging, keep blinking 
Blink blinky;
int32_t counter = 0;

// controller 
GyroDevice gyroController;
// filter to remove the odd value out of range, use the median out of 3 values
RunningMedian* inputPositionX;
RunningMedian* inputPositionY;
RunningMedian* inputPositionZ;

bool impact = false;

unsigned long prevMillis = 0;
const long interval = 2;
int position = 0;

// data received on a message, torque, Rotation and velocity to actuate the gyro
struct DataPackage {
    float TorqX = 0.0f;
    float TorqY = 0.0f;
    float TorqZ = 0.0f;
    float RotX = 0.0f;
    float RotY = 0.0f;
    float RotZ = 0.0f;
    float gVel = 0.0f;
};

DataPackage _dataPackage;

void setup() {
    SLIPSerial.begin(2000000);
    // debug, echo data via serial 1
    Serial1.begin(115200);
    

    // all the initialization data (PID values, encoder steps...) declared in the class
    gyroController.begin();
    blinky.begin(13, 500);

    // inputPositionX = new RunningMedian(3);
    // inputPositionY = new RunningMedian(5);
    // inputPositionZ = new RunningMedian(5);

    gyroController.setGimbalSpeed(25);
    gyroController.flywheelSpeed(20);
}


void loop() {

    // show that the controller is alive and running, if no flash|ing might be stuck somewhere 
    blinky.update();
    
    // receive msg from unity    
    updateUnityMsg();
    // // calculate the required acceleration

    // // motor velocity is always same 

    gyroController.refreshReading();
    gyroController.updatePosition();
    syncPositions();

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
        msgIn.dispatch("/yaw", yawIncrement);
        msgIn.dispatch("/pitch", pitchIncrement);
        msgIn.dispatch("/G_vel", setGimbalVel);
        msgIn.dispatch("/F_vel", setFlywheelVel);
        msgIn.dispatch("/data", refreshData);
        msgIn.dispatch("/impact", actuation);
    }

        // if ((_dataPackage.TorqX || _dataPackage.TorqY || _dataPackage.TorqZ) > 0) {
        //     impact = true;        
}

void calcTorque() {
    // calculate how much to move the disk
}

void syncPositions() {

    // Ajust the gimbal speed to actuate the disk
    // rotate the yaw (z) first then the pitch (y) 
    gyroController.pitchTargetAngle(_dataPackage.RotY);
    gyroController.yawTargetAngle(_dataPackage.RotZ);
    //Serial1.println(position);

    //gyroController.yawTargetAngle(90);
}

void impactRoutine() {

    // timer variables
    static unsigned long previousMillis = 0;
    static unsigned long currentMillis = 0;
    const long interval = 2000;

    // speed for impact
    gyroController.setGimbalSpeed(100);

    gyroController.calculateDisplacemnet();

        
        while (currentMillis - previousMillis >= interval) {
            // save the last time you blinked the LED
      previousMillis = currentMillis;
  }

  impact = false;
}


void yawIncrement(OSCMessage& msg) {
    _dataPackage.RotZ += msg.getFloat(0);
    //Serial1.println(_dataPackage.RotZ);    
    //Serial1.print("Match: '/yawIncrement' - ");
}

void pitchIncrement(OSCMessage& msg) {
    _dataPackage.RotY += msg.getFloat(0);
    //Serial1.print("Match: '/pitchincrement' - ");
    //Serial1.println(_dataPackage.RotY);    
}

void setGimbalVel(OSCMessage& msg) {
    gyroController.setGimbalSpeed(msg.getFloat(0));
    //Serial1.print("Match: '/setGimbalVel'");
    //Serial1.println(msg.getFloat(0));    
}


void setFlywheelVel(OSCMessage& msg) {
    gyroController.flywheelSpeed(0);
    gyroController.flywheelSpeed(msg.getFloat(0));
    //Serial1.print("Match: '/setFlywheelVel'");
    //Serial1.println(msg.getFloat(0));
}

void refreshData(OSCMessage& msg) {
    
        _dataPackage.TorqX = msg.getFloat(0);
        _dataPackage.TorqY = msg.getFloat(1);
        _dataPackage.TorqZ = msg.getFloat(2);
        _dataPackage.RotX =  msg.getFloat(3);
        _dataPackage.RotY =  msg.getFloat(4);
        _dataPackage.RotZ =  msg.getFloat(5);
        _dataPackage.gVel =  msg.getFloat(6);

    //Serial1.println("Match: '/refreshData'");
}

void actuation(OSCMessage& msg) {
    //Serial1.println("Match: '/actuation'");
}


