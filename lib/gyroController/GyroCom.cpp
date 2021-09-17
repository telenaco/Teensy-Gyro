#include <GyroCom.h>
#include <GyroAxle.h>

/**
 * @brief the controller had bidirectinal com with the PC with connected to the multiload cell
 * the controller forwards the force calculation to the other mController and waits for moving instructions.
 * Serial is then declared on main with the debug function.
 */

    #include <SLIPEncodedUSBSerial.h>
    SLIPEncodedUSBSerial SLIPSerial(Serial);

    // #include <SLIPEncodedSerial.h>
    // SLIPEncodedSerial SLIPSerial(Serial1);


void GyroCom::begin() {
    SLIPSerial.begin(2000000);
}

void GyroCom::sendThreeValues(float* x, float* y, float* z)
{
    if (x == NULL ||
        y == NULL ||
        z == NULL)
        return;

    byte tempX[4]; // temporal containers to hold float 
    byte tempY[4];
    byte tempZ[4];

    FloatToBytes(*x, tempX);
    FloatToBytes(*y, tempY);
    FloatToBytes(*z, tempZ);

    ArrayCopy(tempX, 0, _message, 0, 4);   
    ArrayCopy(tempY, 0, _message, 4, 4);
    ArrayCopy(tempZ, 0, _message, 8, 4);
    _message[12] = 0x3c;   
}

void GyroCom::sendTwoFormulas(float* x1, float* y1, float* z1, float* x2, float* y2, float* z2, axleReadings* pitch, axleReadings* yaw) {

    Serial.print(*x1, 5);
    Serial.print(',');
    Serial.print(*y1, 5);
    Serial.print(',');
    Serial.print(*z1, 5);
    Serial.print(',');
    Serial.print(*x2, 5);
    Serial.print(',');
    Serial.print(*y2, 5);
    Serial.print(',');
    Serial.print(*z2, 5);
    Serial.print(',');
    Serial.print(pitch->radians, 3);
    Serial.print(',');
    Serial.print(pitch->vel, 3);
    Serial.print(',');
    Serial.print(pitch->acc, 3);
    Serial.print(',');
    Serial.print(yaw->radians, 3);
    Serial.print(',');
    Serial.print(yaw->vel, 3);
    Serial.print(',');
    Serial.print(yaw->acc, 3);
    Serial.print(',');
    Serial.print(millis());
    Serial.println();
}

    
    void GyroCom::sendData(float* x, float* y, float* z, axleReadings* pitch, axleReadings* yaw) {

    if (x == NULL ||
        y == NULL ||
        z == NULL ||
        pitch == NULL ||
        yaw == NULL)
        return;

    byte tempX[4]; // temporal containers to hold float 
    byte tempY[4];
    byte tempZ[4];
    byte tempPitchPos[4];
    byte tempPitchVel[4];
    byte tempPitchAcc[4];
    byte tempYawPos[4];
    byte tempYawVel[4];
    byte tempYawAcc[4];

    FloatToBytes(*x, tempX);
    FloatToBytes(*y, tempY);
    FloatToBytes(*z, tempZ);
    FloatToBytes((float)pitch->radians, tempPitchPos);
    FloatToBytes((float)pitch->vel, tempPitchVel);
    FloatToBytes((float)pitch->acc, tempPitchAcc);
    FloatToBytes((float)yaw->radians, tempYawPos);
    FloatToBytes((float)yaw->vel, tempYawVel);
    FloatToBytes((float)yaw->acc, tempYawAcc);

    ArrayCopy(tempX, 0, _message, 0, 4);
    ArrayCopy(tempY, 0, _message, 4, 4);
    ArrayCopy(tempZ, 0, _message, 8, 4);
    ArrayCopy(tempPitchPos, 0, _message, 12, 4);
    ArrayCopy(tempPitchVel, 0, _message, 16, 4);
    ArrayCopy(tempPitchAcc, 0, _message, 20, 4);
    ArrayCopy(tempYawPos, 0, _message, 24, 4);
    ArrayCopy(tempYawVel, 0, _message, 28, 4);
    ArrayCopy(tempYawAcc, 0, _message, 32, 4);
    _message[36] = 0x3c;

    SLIPSerial.beginPacket();
    SLIPSerial.write(_message, 37);
    SLIPSerial.endPacket();

}

void GyroCom::FloatToBytes(float val, byte segments[]) {
    ufloat temp;

    temp.val_f = val;

    segments[0] = temp.val_b[0];
    segments[1] = temp.val_b[1];
    segments[2] = temp.val_b[2];
    segments[3] = temp.val_b[3];
}

float GyroCom::BytesToFloat(byte segments[]) {
    ufloat temp;

    temp.val_b[3] = segments[3] ;
    temp.val_b[2] = segments[2] ;
    temp.val_b[1] = segments[1] ;
    temp.val_b[0] = segments[0] ;

    return temp.val_f;
}

void GyroCom::ArrayCopy(byte src[], int src_index, byte dest[], int dest_index, int len) {
    for (int i = 0; i < len; i++) {
        dest[dest_index + i] = src[src_index + i];
    }
}
