#include <elapsedmillis.h>
#include <pinDef.h>
#include <GyroDevice.h>
#include <AS5045.h>
#include <GyroAxle.h>

elapsedMillis sw;
// AS5045 yaw, pitch;
// GyroAxle yaw, pitch;
GyroDevice controller;
String command;

// read direcltly from encoder 
// void setup() {
//   yaw.begin(yawCs, yawClk, yawData);
//   pitch.begin(pitchCs, pitchClk, pitchData);

//   Serial.begin(115200);
// }

// void loop() {
//   yaw.read_bias();
//   pitch.read_bias();

//   if (sw > 20) {
//     Serial.print(yaw.read_bias());
//     Serial.print(" <-- --->");
//     Serial.println(pitch.read_bias());

//     sw = 0;
//   }
// }

// read directly from the Axle class using telemetry to test v0.7 http://www.farrellf.com/TelemetryViewer/
// void setup() {
//     yaw.begin(yawA, yawB, yawEnable, yawData, yawClk, yawCs, 109.16);
//     pitch.begin(pitchA, pitchB, pitchEnable, pitchData, pitchClk, pitchCs, 98.12);

//     Serial.begin(115200);
//     Serial.println("end of setup");
// }

// void loop() {
//     yaw.refreshEncoder();
//     pitch.refreshEncoder();

//     if (sw > 10) {

//         float yaw_data = yaw.getGimbalAngle();
//         float pitch_data = pitch.getGimbalAngle();

//         char yaw_data_text[30];
//         char pitch_data_text[30];

//         dtostrf(yaw_data, 10, 10, yaw_data_text);
//         dtostrf(pitch_data, 10, 10, pitch_data_text);

//         char text[63];
//         snprintf(text, 63, "%s,%s", yaw_data_text, pitch_data_text);
//         Serial.println(text);

//         sw = 0;
//     }
// }

// // read directly from the Axle class
void setup() {
    // pins are assign at the gyroDevice.h file
    controller.begin();
    // Serial is initialized as part of the GyroCom.h
}

void loop() {

    controller.refreshReadings();
    controller.calibratePID();

    if (Serial.available()) {
        command = Serial.readStringUntil('\n');

        if (command.equals("brake")) {
            controller.brakeOn();
        }
        else if (command.equals("open")) {
            controller.brakeOff();
        }
        else if (command.startsWith("speed")) {
            String speed = command.substring(6);
            controller.setSpeed(speed.toInt());
        }
        else if (command.startsWith("yaw")) {
            String speed = command.substring(4);
            controller.updateYaw(speed.toInt());
        }
        else if (command.startsWith("pitch")) {
            String speed = command.substring(6);
            controller.updatePitch(speed.toInt());
        }
        else {
            Serial.println("Invalid command");
        }
    }

    // print the axle position values to telemetry 0.7
    if (sw > 20) {
        float yaw = controller.getYaw();
        float pitch = controller.getPitch();

        char yaw_text[30];
        char pitch_text[30];

        dtostrf(yaw, 10, 10, yaw_text);
        dtostrf(pitch, 10, 10, pitch_text);

        char text[63];
        snprintf(text, 63, "%s,%s", yaw_text, pitch_text);

        Serial.println(text);

        sw = 0;
    }
}






