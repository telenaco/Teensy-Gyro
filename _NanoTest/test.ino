#include <Arduino.h>
#include <Blink.h>
#include <OSCBundle.h>
#include <OSCBoards.h>

#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial( thisBoardsSerialUSB );
#else
#include <SLIPEncodedSerial.h>
 SLIPEncodedSerial SLIPSerial(Serial);
#endif
 

Blink blinky;

void setup() {

    SLIPSerial.begin(115200);
    Serial.begin(115200);
    Serial.println("test");

    blinky.begin(13, 100);
}

void loop() {
    Serial.println("test print");
    blinky.update();

}