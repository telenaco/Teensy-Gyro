#ifndef AS5047D_h
#define AS5047D_h

#define AS5047D_ADDRESS 0x3FFF

class AS5047D {
public:
    void init(int C);
    int read();

private:
    int value;
    int angle;
    int prevAngle;
    int RERotations;
    bool CW;
    int CSN;
};

#endif
