#include <Arduino.h>
#include <elapsedMillis.h>
#include <runAVG.h>
#include "VernierLib.h"  

// timers
elapsedMillis sw;
elapsedMillis tm;

// filter
runAVG filter;

// vernier force sensor
VernierLib Vernier;   //create an instance of the VernierLib library

void setup() {
    Serial.begin(115200);  //setup communication to display
    filter.begin();
    Vernier.autoID();  //identify the sensor being used
}

void loop() {

    float tmp = filter.smoothReading(Vernier.readSensor());

    Serial.println((String)"<," +
        tm + "," +
        tmp + "," +
        "0,0,0,0,0,>");



}


