#ifndef gyroscope_h
#define gyroscope_h

typedef struct dcMotors {
    volatile int IN1;
    volatile int IN2;
    volatile int EN;
    volatile int goalPosition;
    volatile int angle;
    volatile int prevAngle;
    volatile int dir;
    volatile int speedVal;
    volatile int startPos;
    volatile int endPos;
} dc;

typedef struct brake {
    volatile int pin;
    volatile long pulse;
    volatile long pulseInterval;
    unsigned long previousMicros;
    unsigned long currentMicros;
    int range[2];
    volatile bool state;
    volatile bool closed;
    volatile int duration;
    volatile long startMillis;
} b;

typedef struct ESC {
    volatile int pin;
    volatile int value;
    volatile int speedVal;
    volatile bool dir;
} e;

class Gyroscope {
public:
    /**
 * @brief
 *
 * @param IN1
 * @param IN2
 * @param IN3
 * @param IN4
 * @param ENA
 * @param ENB
 * @param flywheel
 * @param yawCLK
 * @param yawCS
 * @param yawData
 * @param pitchCLK
 * @param pitchCS
 * @param pitchData
 * @param brake
 */
    void initGyro(int IN1, int IN2, int IN3, int IN4, int ENA, int ENB,
        int flywheel,
        int yawCLK, int yawCS, int yawData,
        int pitchCLK, int pitchCS, int pitchData,
        int brake);
    void flywheelSetSpeed(int);
    void brakeFlywheel(int);
    void brakeInterval(int pulseIncrement, int pulseStep);
    void receiveDataFromUnity();
    void updateAxes();
    void setAngles(int, int);
    void getAngles();
    void calibrate(int axis);

private:
    e esc;
    b brake;
    dc motorX;
    dc motorY;
};

#endif
