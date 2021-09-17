#include <elapsedmillis.h>
#include <BasicLinearAlgebra.h>
#include <GyroDevice.h>
#include <blink.h>

#define REFRESH_LOOP 1000

IntervalTimer updateTimer;
Blink blinky;
elapsedMillis sw;               // run the functions for a time
String command;
uint8_t functionNumber = 0;

GyroDevice gyroController;

void setup() {
    gyroController.begin();
    blinky.begin(13, 500);
    updateTimer.begin(gyroUpdate, REFRESH_LOOP);

    while (!Serial);

    Serial.println("enter the function name and value");
    Serial.println("1 - brake + duration");
    Serial.println("2 - brakeSmooth + duration");
    Serial.println("3 - encoderPos");
    Serial.println("4 - encoderAngle");
    Serial.println("5 - flywheelOn + duration");
    Serial.println("6 - pitch + angle");
    Serial.println("7 - yaw + angle");
    Serial.println("8 - reset");
}

void gyroUpdate() {
    gyroController.update();
}

void loop() {
    blinky.update();

    if (Serial.available()) {
        command = Serial.readStringUntil('\n');

        if (command.startsWith("brakeSmooth")) {
            int brakeDuration = command.substring(12).toInt();
            Serial.print("braking flywheel for (ms) -> ");
            Serial.println(brakeDuration);
            gyroController.brakeSmooth(brakeDuration);
        }
        else if (command.startsWith("brake")) {
            int brakeDuration = command.substring(6).toInt();
            Serial.print("braking for -> ");
            Serial.println(brakeDuration);
            gyroController.brakeOn(brakeDuration);
        }
        else if (command.equalsIgnoreCase("encoderPos")) {
            functionNumber = 1;
        }
        else if (command.equalsIgnoreCase("encoderAngle")) {
            functionNumber = 2;
        }
        else if (command.startsWith("flywheel")) {
            int speed = command.substring(8).toInt();
            Serial.print("flywheel speed -> ");
            Serial.println(speed);
            gyroController.flywheelStart(speed);
        }
        else if (command.startsWith("pitch")) {
            int angle = command.substring(5).toInt();
            Serial.print("move pitch to -> ");
            Serial.println(angle);
            gyroController.movePitchTo(angle);
        }
        else if (command.startsWith("yaw")) {
            int angle = command.substring(3).toInt();
            Serial.print("move yaw to -> ");
            Serial.println(angle);
            gyroController.moveYawTo(angle);
        }
        else if (command.equalsIgnoreCase("reset")) {
            functionNumber = 3;
        }
        else {
            Serial.println("Invalid command");
        }
    }

    if (sw >= 10)
    {
        switch (functionNumber)
        {
        case 1:
            gyroController.encoderPosition();
            break;
        case 2:
            gyroController.encoderAngle();
            break;
        case 3:
            functionNumber = 0;
            break;
        default:
            break;
        }
        sw = 0;
    }
}

