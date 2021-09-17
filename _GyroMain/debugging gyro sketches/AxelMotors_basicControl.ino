#include <H_bridge_v1.h>
#include <Blink.h>
#include <gyroPinDef.h>

H_bridge yawMotor, pitchMotor;
Blink blinky;

void setup() {
  Serial.begin(115200);
  blinky.begin(13, 500);
  yawMotor.begin(enableYaw, yawA, yawB);
  pitchMotor.begin(enablePitch, pitchA, pitchB);
}

void loop() {
  blinky.update();

  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    Serial.print("current speed -> ");
    Serial.println(i);
    yawMotor.forward(i);
    pitchMotor.forward(i);
    delay(20);
  }

  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    Serial.print("current speed -> ");
    Serial.println(i);
    yawMotor.forward(i);
    pitchMotor.forward(i);
    delay(20);
  }
  yawMotor.stop();
  pitchMotor.stop();
  delay(5000);
}