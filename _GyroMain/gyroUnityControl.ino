#include <elapsedmillis.h>
#include <BasicLinearAlgebra.h>
#include <GyroDevice.h>
#include <blink.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <SLIPEncodedUSBSerial.h>
#include <SLIPEncodedSerial.h>

// encoding to send and receive message to UNITY
SLIPEncodedUSBSerial SLIPSerial(Serial);

// for debugging, keep blinking 
Blink blinky;

// controller 
GyroDevice gyroController;
// filter to remove the odd value out of range, use the median out of 3 values
RunningMedian* inputPositionX;
RunningMedian* inputPositionY;
RunningMedian* inputPositionZ;

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
    SLIPSerial.begin(115200);
    // debug, echo data via serial 1
    Serial1.begin(115200);

    // all the initialization data (PID values, encoder steps...) declared in the class
    gyroController.begin();
    blinky.begin(13, 500);

    inputPositionX = new RunningMedian(3);
    inputPositionY = new RunningMedian(5);
    inputPositionZ = new RunningMedian(5);

    gyroController.flywheelSpeed(60);
}

void loop() {
    // show that the controller is alive and running, if no flashing might be stuck somewhere 
    blinky.update();
    // receive msg from unity


    //updateUnityMsg();
    // calculate the required acceleration

    // motor velocity is always same 

    gyroController.refreshReading();
    gyroController.updatePosition();

    if (Serial.available()) {
        static int angle = 0;
        // read the incoming byte:
        char incomingByte = Serial.read();
        if (incomingByte == 'a') {
            Serial.println(incomingByte);
            gyroController.yawTargetAngle(angle += 10);
        }
        if (incomingByte == 'b') {
            Serial.println(incomingByte);
            gyroController.yawTargetAngle(angle -= 10);
        }
    }
}

/**
 * @brief read the serial stream, messages are encoded using OSC.
 *
 */
void updateUnityMsg() {

    OSCMessage msgIN;
    int size;
    unsigned long start = millis();
    unsigned long timeout = 12;

    while (!SLIPSerial.endofPacket()) {
        if ((size = SLIPSerial.available()) > 0) {
            while (size--) {
                msgIN.fill(SLIPSerial.read());
            }
        }
        unsigned long now = millis();
        unsigned long elapsed = now - start;
        if (elapsed > timeout) return;
    }

    if (!msgIN.hasError()) {

        // SLIPSerial.beginPacket();
        // msgIN.send(SLIPSerial);
        // SLIPSerial.endPacket();

        _dataPackage.TorqX = msgIN.getFloat(0);
        _dataPackage.TorqY = msgIN.getFloat(1);
        _dataPackage.TorqZ = msgIN.getFloat(2);
        _dataPackage.RotX = msgIN.getFloat(3);
        _dataPackage.RotY = msgIN.getFloat(4);

        inputPositionZ->add(msgIN.getFloat(5));
        _dataPackage.RotZ = inputPositionZ->getMedian();

        _dataPackage.gVel = msgIN.getFloat(6);

        syncPositions();
    }
}

void calcTorque() {
    // calculate how much to move the disk
}

void syncPositions() {

    // Ajust the gimbal speed to actuate the disk
    // rotate the yaw (z) first then the pitch (y) 
    //gyroController.pitchTargetAngle(_dataPackage.RotY);
    //gyroController.yawTargetAngle(_dataPackage.RotZ);
    gyroController.yawTargetAngle(90);
}

void impact() {
    // we move to the calculated desired positoin and inmediatly lower the speed to min for a given time to avoid opposite feedback
}
