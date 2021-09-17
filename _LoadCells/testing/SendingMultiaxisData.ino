/**
 * @file SendingMultiaxisData.ino
 * @author your name (you@domain.com)
 * @brief sample use to debut the multiaxis.ino sketch, it sends data in the same format that the gyroController does so
 * @version 0.1
 * @date 2021-05-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <Arduino.h>
#include <elapsedMillis.h>
#include <blink.h>

#define HWSERIAL Serial1

elapsedMillis sw;
Blink blinky;

int32_t x = 0, y = 0, z = 0;

void setup() {
    HWSERIAL.begin(2000000);
    blinky.begin(13, 500);
}

void loop() {
    blinky.update();

    if (sw >= 20) {
        x += 1;
        y += 2;
        z += 3;
        
        HWSerialMessage();
        sw = 0;
    }
}

void HWSerialMessage() {
        HWSERIAL.print("<");
        HWSERIAL.print(x);
        HWSERIAL.print(',');
        HWSERIAL.print(y);
        HWSERIAL.print(',');
        HWSERIAL.print(z);
        HWSERIAL.println(">");
}