#include <elapsedmillis.h>
#include <BasicLinearAlgebra.h>
#include <GyroDevice.h>
#include <blink.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <SLIPEncodedSerial.h>


//IntervalTimer updateTimer;
Blink blinky;

//GyroDevice gyroController;

//receive a bundle
struct DataPackage
{
    float recTorqX = 0.0f;
    float recTorqY = 0.0f;
    float recTorqZ = 0.0f;
    float recRotX = 0.0f;
    float recRotY = 0.0f;
    float recRotZ = 0.0f;
    float gVel = 0.0f;
};

DataPackage _dataPackage; 

void setup() {    
    Serial.begin(115200);
    Serial1.begin(115200);
    blinky.begin(13, 100);
}

void loop() {
    blinky.update();
    //readOscMes();
    String a;

    while (Serial1.available() > 0) {
        Serial.println(Serial1.readStringUntil('\n'));
    }
}

