/**
 * @file GyroCom.h
 * @brief communication between the µController and Unity
 * - using < and > for open and close message
 * send data to unity:
 *  - angle ψ
 *  - velocity ψ
 *  - acceleration ψ
 *  - angle Θ
 *  - velocity Θ
 *  - acceleration Θ
 *  - current millis
 *  - brake state
 *
 *  Curently "BUFFER_LENGTH 32" to change it modify WireKinetis.h
 *
 *  intended baudrate   error at 96MHz
 *      500,000             0.0%
 *
 *  int8_t velocity, acceleration range -100 / 100 (negative value = counterclock wise rotation)
 *  int8_t brake position / 0 or 1
 *
 */

#ifndef GYRO_COM
#define GYRO_COM


#include <Arduino.h>
#include <GyroAxle.h>       // to access the axle data struct

class GyroCom {

private:

    const int16_t numChars = 32;
    char receivedChars[32];
    boolean newData = false;
    byte _message[40];

    typedef union {
	byte val_b[4];
	float val_f;
    } ufloat;
    
    void recvWithStartEndMarkers();


public:
    int readingValues[6] = {};

    GyroCom() {}
    virtual ~GyroCom() {}

    void begin();
    // void sendMessage(String message);
    // bool readMessage(int*, int*, int*, int*, int*);
    // bool readTerminal();
    // bool telemetryMessage(axleReadings* yaw, axleReadings* pitch);
    // void sendTorque(double* x, double* y, double* z);
    // void sendTorque(float* x, float* y, float* z);
    // void printEncoderSteps(double* yawSteps, double* pitchSteps);
    // void printEncoderAngles(double* yawAngle, double* pitchAngle);
    // void axelValues(double* a, double* b, double* c, double* d, double* e);

    void sendThreeValues(float* x, float* y, float* z);
    void sendTwoFormulas(float* x1, float* y1, float* z1, float* x2, float* y2, float* z2, axleReadings* pitch, axleReadings* yaw);
    void sendData(float* x, float* y, float* z, axleReadings* pitch, axleReadings* yaw);
    //void serialWriteAll(byte buffer[], int size);

    void FloatToBytes(float val, byte segments[]);
    float BytesToFloat(byte segments[]);
    void ArrayCopy(byte src[], int src_index, byte dest[], int dest_index, int len);
};
#endif