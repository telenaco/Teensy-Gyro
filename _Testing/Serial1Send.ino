/**                   sender                **/

#include <elapsedMillis.h>
#include <Arduino.h>

#define HWSERIAL Serial1

elapsedMillis sw;
int32_t counter = 0;

void setup() {
    HWSERIAL.begin(2000000);
}

void loop() {


    if (sw >= 100)
    {
        counter++;
        HWSERIAL.println(counter);
        sw = 0;
    }
}


/**                     receiver                 **/


#include <elapsedMillis.h>
#include <Arduino.h>

#define HWSERIAL Serial1

void setup() {
    Serial.begin(2000000);
    HWSERIAL.begin(2000000);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    int incomingByte;

    if (HWSERIAL.available() > 0) {
        incomingByte = HWSERIAL.read();
        Serial.print("UART received: ");
        Serial.println(incomingByte, DEC);
    }
}