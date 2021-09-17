#include <Arduino.h>
#include <Blink.h>
//#include <LowPassFilter.h>
#include <RunningMedian.h>
#include <HX711.h>
#include "HX711PinDef.h"
//#include <SLIPEncodedSerial.h>


// data is send from Gyro µC to load cell µC via Serial1, 
// data from Gyro and LoadCell is forwarded from load cell µC to PC
// GyroController data (Serial) -> HX711 multiaxis (Serial) -> Computer 

// only used to communicated directly with second board
// SLIPEncodedSerial SLIPSerial(Serial1); // SERIAL 1 uses pins 0,1 not USB port 

/** for initial calibration uncomnent the follwing line **/
//#define CALIBRATE_CELLS

struct axisReading {
    float reading, scale, force, crossover;
    float Xcontribution, Ycontribution, Zcontribution;
};

RunningMedian filterForceX(9);
RunningMedian filterForceY(9);
RunningMedian filterForceZ(9);

// RunningMedian filterGyroX(5);
// RunningMedian filterGyroY(5);
// RunningMedian filterGyroZ(5);

// RunningMedian filterPitchPos(5);
// RunningMedian filterPitchVel(5);
// RunningMedian filterPitchAcc(5);

// RunningMedian filterYawPos(5);
// RunningMedian filterYawVel(5);
// RunningMedian filterYawAcc(5);


Blink blinky;
HX711 scaleX, scaleY, scaleZ;
axisReading axisX, axisY, axisZ;

// timer
elapsedMillis sw;  // data timer

// const byte numBytes = 40;
// byte receivedBytes[numBytes];
// byte numReceived = 0;

// boolean newData = false;

// float gyroTorqueX, gyroTorqueY, gyroTorqueZ;
// float gyroPitchPos, gyroPitchVel, gyroPitchAcc;
// float gyroYawPos, gyroYawVel, gyroYawAcc;

// typedef union {
//     byte val_b[4];
//     float val_f;
// } ufloat;

// uint32_t timerClock;
// elapsedMillis tm;

// int oldTime = 0;
// int counter = 0;

void setup() {

    blinky.begin(13, 50);

    // choose the serial port to send, for debugging 
    //SLIPSerial.begin(2000000);
    Serial.begin(115200);
    
    scaleX.begin(dat3, clk3);
    scaleY.begin(dat2, clk2);
    scaleZ.begin(dat1, clk1);

    axisX.scale = 857.72;
    axisY.scale = 784.89;
    axisZ.scale = 1052;
    // Percentage of the axis that affects the others 
    axisX.Ycontribution = -0.0215f;
    axisX.Zcontribution = 0.0123f;
    axisY.Xcontribution = -0.0187f;
    axisY.Zcontribution = -0.04f;
    axisZ.Xcontribution = 0.0117;
    axisZ.Ycontribution = -0.0204;

    scaleX.tare(500);  // number of time to averate the current value
    scaleY.tare(500);
    scaleZ.tare(500);
}

void loop() {

    // show that we are alive
    blinky.update();
    // update the reading values from the load cell 
    updateCellReadings();
    // convert the readings to Newtons and account for the crossover from the other cells
    // adjust on this function the direction of the force
    calculateNewtons();
    // update reading of the controller from the SLIPSerial
    // send the data to the computer every 1ms (adjust sw)
    //updateControllerReadings();

    sendReadings();
}

void calibrateCell() {
    axisX.reading = (scaleX.get_value(200));
    axisY.reading = (scaleY.get_value(200));
    axisZ.reading = (scaleZ.get_value(200));

    Serial.println((String)
        axisX.reading + "," +
        axisY.reading + "," +
        axisZ.reading);
    delay(1);
    loop();
}

void updateCellReadings() {
    axisX.reading = (scaleX.get_value());
    axisY.reading = (scaleY.get_value());
    axisZ.reading = (scaleZ.get_value());
}

void calculateNewtons() {

    static float readingX = 0;
    static float readingY = 0;
    static float readingZ = 0;
    
    axisX.crossover = (axisY.reading * axisY.Xcontribution) + (axisZ.reading * axisZ.Xcontribution);
    readingX = axisX.reading + 296.88;
    readingX -= axisX.crossover;
    readingX /= axisX.scale;

    axisY.crossover = (axisX.reading * axisX.Ycontribution) + (axisZ.reading * axisZ.Ycontribution);
    readingY = axisY.reading - 1837;
    readingY -= axisY.crossover;
    readingY /= axisY.scale;

    axisZ.crossover = (axisX.reading * axisX.Zcontribution) + (axisY.reading * axisY.Zcontribution);
    readingZ = axisZ.reading + 172;
    readingZ -= axisZ.crossover;
    readingZ /= axisZ.scale;

    filterForceX.add(readingY);
    axisX.force = filterForceX.getAverage(5);
    filterForceY.add(readingX);
    axisY.force = filterForceY.getAverage(5);
    filterForceZ.add(readingZ);
    axisZ.force = filterForceZ.getAverage(5);

    // depending of the fram of reference invert direction
    //axisX.force *= -1.0f;
    axisY.force *= -1.0f;
    axisZ.force *= -1.0f;    
}
/*
void updateControllerReadings() {

    // receive 9 floats split on 4 bytes each

    recvWithStartEndMarkers();

    if (newData == true) {

        byte x[4], y[4], z[4];
        byte pitchPos[4], pitchVel[4], pitchAcc[4];
        byte yawPos[4], yawVel[4], yawAcc[4];      

        ArrayCopy(receivedBytes, 0, x, 0, 4);
        ArrayCopy(receivedBytes, 4, y, 0, 4);
        ArrayCopy(receivedBytes, 8, z, 0, 4);
        ArrayCopy(receivedBytes, 12, pitchPos, 0, 4);
        ArrayCopy(receivedBytes, 16, pitchVel, 0, 4);
        ArrayCopy(receivedBytes, 20, pitchAcc, 0, 4);
        ArrayCopy(receivedBytes, 24, yawPos, 0, 4);
        ArrayCopy(receivedBytes, 28, yawVel, 0, 4);
        ArrayCopy(receivedBytes, 32, yawAcc, 0, 4);

        float tmp = 0;

        tmp = BytesToFloat(x);
        filterGyroX.add(tmp);
        gyroTorqueX = filterGyroX.getMedian();
        
        tmp = BytesToFloat(y);
        filterGyroY.add(tmp);
        gyroTorqueY = filterGyroY.getMedian();

        tmp = BytesToFloat(z);
        filterGyroZ.add(tmp);
        gyroTorqueZ = filterGyroZ.getMedian();
        
        tmp = BytesToFloat(pitchPos);
        filterPitchPos.add(tmp);
        gyroPitchPos = filterPitchPos.getMedian();

        tmp = BytesToFloat(pitchVel);
        filterPitchVel.add(tmp);
        gyroPitchVel = filterPitchVel.getMedian();

        tmp = BytesToFloat(pitchAcc);
        filterPitchAcc.add(tmp);
        gyroPitchAcc = filterPitchAcc.getMedian();

        tmp = BytesToFloat(yawPos);
        filterYawPos.add(tmp);
        gyroYawPos = filterYawPos.getMedian();

        tmp = BytesToFloat(yawVel);
        filterYawVel.add(tmp);
        gyroYawVel = filterYawVel.getMedian();

        tmp = BytesToFloat(yawAcc);
        filterYawAcc.add(tmp);
        gyroYawAcc = filterYawAcc.getMedian();        

        newData = false;

        // Debuging - received message and converted back to float
        // Serial.print(gyroTorqueX);
        // Serial.print("\t");
        // Serial.print(gyroTorqueY);
        // Serial.print("\t");
        // Serial.print(gyroTorqueZ);
        // Serial.print("\t");
        // Serial.print(gyroPitchPos);
        // Serial.print("\t");
        // Serial.print(gyroPitchVel);
        // Serial.print("\t");
        // Serial.print(gyroPitchAcc);
        // Serial.print("\t");
        // Serial.print(gyroYawPos);
        // Serial.print("\t");
        // Serial.print(gyroYawVel);
        // Serial.print("\t");
        // Serial.print(gyroYawAcc);
        // Serial.print("\t");
        // Serial.println("");
    }
}

void recvWithStartEndMarkers() {
    byte ndx = 0;
    byte rb;
    int size;
    Serial1.clear();  // clear the buffer before reading, only interested on the most recent data

    while (!SLIPSerial.endofPacket()) {
        if ((size = SLIPSerial.available()) > 0) {
            while (size--) {
                 rb = SLIPSerial.read();
                 receivedBytes[ndx] = rb;
                 ndx++;
                 newData = true;
            }
        }
    }
}
*/
void sendReadings() {

    Serial.print(axisX.force);
    Serial.print(',');
    Serial.print(axisY.force);
    Serial.print(',');
    Serial.print(axisZ.force);
    Serial.print(',');
    Serial.println(millis());    
    // Serial.print(',');
    // Serial.print(gyroTorqueX, 5);
    // Serial.print(',');
    // Serial.print(gyroTorqueY, 5);
    // Serial.print(',');
    // Serial.print(gyroTorqueZ, 5);
    // Serial.print(',');
    // Serial.print(gyroPitchPos, 5);
    // Serial.print(',');
    // Serial.print(gyroPitchVel, 5);
    // Serial.print(',');
    // Serial.print(gyroPitchAcc, 5);
    // Serial.print(',');
    // Serial.print(gyroYawPos, 5);
    // Serial.print(',');
    // Serial.print(gyroYawVel, 5);
    // Serial.print(',');
    // Serial.print(gyroYawAcc, 5);
    // Serial.println("");

}

// void FloatToBytes(float val, byte segments[]) {
//     ufloat temp;

//     temp.val_f = val;

//     segments[0] = temp.val_b[0];
//     segments[1] = temp.val_b[1];
//     segments[2] = temp.val_b[2];
//     segments[3] = temp.val_b[3];
// }

// float BytesToFloat(byte segments[]) {
//     ufloat temp;

//     temp.val_b[3] = segments[3];
//     temp.val_b[2] = segments[2];
//     temp.val_b[1] = segments[1];
//     temp.val_b[0] = segments[0];

//     return temp.val_f;
// }

// void ArrayCopy(byte src[], int src_index, byte dest[], int dest_index, int len) {
//     for (int i = 0; i < len; i++) {
//         dest[dest_index + i] = src[src_index + i];
//     }
// }