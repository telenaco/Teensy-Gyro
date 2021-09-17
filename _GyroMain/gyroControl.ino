// control code for debugging the gyro 04/05/2021

#include <elapsedmillis.h>
#include <BasicLinearAlgebra.h>
#include <GyroDevice.h>
#include <blink.h>

#define SEND_SERIAL

//IntervalTimer updateTimer;
Blink blinky;
elapsedMicros sw;
elapsedMicros sw1;
elapsedMicros sw2;

GyroDevice gyroController;

bool movingRoutine = false;
bool activateFeedback = false;
unsigned long previousMillis = 0;     // will store last time LED was updated


void setup() {

    Serial.begin(2000000);
    Serial.println("Gyro Controller");              // to identify the com port when multiple connected

    gyroController.setGimbalSpeed(25);              // slow speed to avoid vibrations

    gyroController.begin();
    blinky.begin(13, 500);
}

void loop() {

    blinky.update();
    debugController();

    gyroController.refreshReading();
    gyroController.updatePosition();
    gyroController.calculateTorque();
    //gyroController.sendFormulaTwo();


    // if (sw1 >= 1000) {                         // actuate the motor at 1KHhz 
    //     sw1 = 0;
    // }

    // if (sw >= 4000) {                         // send data over serial at 250Hz, every 4 ms
    //     sw = 0;
    // }

    // -------------- Debug ----------------------
    if (movingRoutine) gyroRoutine();
    if (activateFeedback) feedbackOuput();
}


/**
 * @brief move to to controller to a series of pre determine positions, press 0 on the keyboard to start this routine
 *
 */
void gyroRoutine() {

    static int Index = 0;
    static int preIndex = 0;
    static bool hasIncremented = false;

    //static int angleYaw = 0;
    //static int anglePitch = 0;

    if (gyroController.reachTargetAngles() && !hasIncremented) {
        Index++;
        hasIncremented = true;
        previousMillis = millis();
    }

    switch (Index) {
    case 1:
        // move pitch to 360 * 3
        Serial.println("case 1");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.pitchTargetAngle(360 * 3);
        hasIncremented = false;
        break;
    case 2:
        // move pitch to 0
        Serial.println("case 2");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.pitchTargetAngle(0);
        break;
    case 3:
        // move yaw to 90
        Serial.println("case 3");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.yawTargetAngle(90);
        break;
    case 4:
        // move pitch to 360 * 3
        Serial.println("case 4");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.pitchTargetAngle(360 * 3);
        break;
    case 5:
        // move pitch to 360 * 3
        Serial.println("case 5");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.pitchTargetAngle(90);
        break;
    case 6:
        // move pitch to 0
        Serial.println("case 6");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.yawTargetAngle(360 * 2);
        break;
    case 7:
        // move yaw to 90
        Serial.println("case 7");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.yawTargetAngle(90);
        break;
    case 8:
        // move pitch and yaw to 0 
        Serial.println("case 8");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.yawTargetAngle(0);
        gyroController.pitchTargetAngle(0);
        break;
    case 9:
        // move pitch and yaw to 360 * 5
        Serial.println("case 9");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.yawTargetAngle(360 * 2);
        gyroController.pitchTargetAngle(360 * 2);
        break;
    case 10:
        // move pitch and yaw to 0
        Serial.println("case 10");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.yawTargetAngle(0);
        gyroController.pitchTargetAngle(0);
        break;
    case 11:
        //
        Serial.println("case 11");
        if (!feedbackLoopDelay()) return;
        hasIncremented = false;
        gyroController.setGimbalSpeed(40);
        activateFeedback = true;
        movingRoutine = false;
        Index = 0;
        break;
    default:
        break;
    }
}

bool feedbackLoopDelay() {

    unsigned long currentMillis = millis();
    static const long interval = 2000;           // interval at which to blink (milliseconds)

    if (currentMillis - previousMillis >= interval) {
        // previuosMillis se actualliza cuando se incrementa el index 
        return true;
    }
    Serial.println(currentMillis - previousMillis);
    return false;
}

/**
 * @brief last of the gyroRoutine, swing the controller head back and forth to 45 degress and 0, repeat 10 times.
 *
 */
void feedbackOuput() {
    static bool homeposition = true;
    static long interval = 3000;
    static unsigned long prevMilis = 0;
    static int count = 0;

    unsigned long currentMillis = millis();

    if (currentMillis - prevMilis >= interval) {
        count++;
        Serial.println((String)"count is ->" + count);
        prevMilis = currentMillis;

        if (homeposition) {
            gyroController.pitchTargetAngle(45);
            homeposition = false;
        }
        else {
            gyroController.pitchTargetAngle(0);
            homeposition = true;
        }

        if (count >= 10) {
            activateFeedback = false;
            count = 0;
            gyroController.pitchTargetAngle(0);
            gyroController.yawTargetAngle(0);
            gyroController.setGimbalSpeed(22);
        }

    }
}



/**
 * @brief read any messages in the serial port and the action (eg. brake, increment angle, return to home)
 *
 */
void debugController() {

    static int currentAngleYaw = 0;
    static int currentAnglePitch = 0;

    String command;
    int increment = 90;

    if (Serial.available()) {
        command = Serial.readStringUntil('\n');

        if (command.startsWith("brakeSmooth")) {
            int brakeDuration = command.substring(12).toInt();
            gyroController.brakeSmooth(brakeDuration);
        }
        else if (command.startsWith("brake")) {
            int brakeDuration = command.substring(6).toInt();
            constrain(brakeDuration, 200, 5000);
            gyroController.brakeOn(brakeDuration);
        }
        // change flywheel speed
        else if (command.startsWith("flywheel")) {
            int speed = command.substring(8).toInt();
            speed = (speed < 0) ? 0 : ((speed > 100) ? 100 : speed);
            gyroController.flywheelSpeed(speed);
        }
        // incremement the angle 
        else if (command.equalsIgnoreCase("1")) {
            currentAngleYaw += increment;
            gyroController.yawTargetAngle(currentAngleYaw);
        }
        // decrement the angle
        else if (command.equalsIgnoreCase("2")) {
            currentAngleYaw -= increment;
            gyroController.yawTargetAngle(currentAngleYaw);
        }
        else if (command.equalsIgnoreCase("3")) {
            currentAnglePitch += increment;
            gyroController.pitchTargetAngle(currentAnglePitch);
        }
        else if (command.equalsIgnoreCase("4")) {
            currentAnglePitch -= increment;
            gyroController.pitchTargetAngle(currentAnglePitch);
        }
        else if (command.equalsIgnoreCase("5")) {
            currentAnglePitch += 360 * 10;
            gyroController.pitchTargetAngle(currentAnglePitch);
        }
        else if (command.equalsIgnoreCase("6")) {
            currentAngleYaw += 360 * 10;
            gyroController.yawTargetAngle(currentAngleYaw);
        }
        // decrement the angle
        else if (command.equalsIgnoreCase("7")) {
            currentAnglePitch = 0;
            currentAngleYaw = 0;
            gyroController.pitchTargetAngle(currentAnglePitch);
            gyroController.yawTargetAngle(currentAngleYaw);
        }
        else if (command.equalsIgnoreCase("0")) {
            previousMillis = millis();
            movingRoutine = !movingRoutine;
        }
        else if (command.equalsIgnoreCase("reset")) {
            // reset function
        }
        else {
            Serial.println("Invalid command");
        }
    }
}
