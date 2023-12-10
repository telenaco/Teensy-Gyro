#include <GyroDevice.h>
#include <AS5045.h>
#include <Blink.h>
#include <gyroPinDef.h>
#include <elapsedMillis.h>

AS5045 encoderYaw, encoderPitch;
Blink blinky;
elapsedMillis sw;


void setup() {
  Serial.begin(115200);
  blinky.begin(13, 500);
  encoderYaw.begin(yawCs, yawClk, yawData);                     //AS5045 needs three pins for basic position operation
  encoderPitch.begin(pitchCs, pitchClk, pitchData);             //data, clock and select
}

void loop() {
  
  blinky.update();
  int currentYaw = encoderYaw.read_bias();
  int currentPitch = encoderPitch.read_bias ();

  if (sw > 10) {
    Serial.print("millis() -> "
    );
    Serial.print(millis());
    Serial.print("  <->  ");
     Serial.print(currentPitch);
     Serial.print("  <->  ");
    Serial.println(currentYaw);
    sw = 0;
  }

}