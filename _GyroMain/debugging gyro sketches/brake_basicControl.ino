#include <Servo.h>
#include <Blink.h>
#include <gyroPinDef.h>

Servo brake;
Blink blinky;

int pos = 0;

void setup() {
  Serial.begin(115200);
  blinky.begin(13, 500);
  brake.attach(brakePin);


}

void loop() {
  blinky.update();
  brake.write(0);
  delay(5000);
  brake.write(180); // tell servo to go to position in variable 'pos'
  delay(5000); // waits 15ms for the servo to reach the position
}