#ifndef HBRIDGE_h
#define HBRIDGE_h

/**
 * sets the speed as a % (0-100)
 */

#include <Arduino.h>

class H_bridge {

private:

    enum Direction {
        STOP = 0,
        FORWARD = 1,   //Clockwise
        BACKWARD = 2   //CCW
    };

    uint8_t    _pinEnable;
    uint8_t    _pinIn1;
    uint8_t    _pinIn2;
    uint8_t    _speed = 0;
    bool       _isMoving = false;
    Direction  _direction;

public:

    H_bridge();
    virtual ~H_bridge();

    /**
     * @brief initialize the h-Bridge
     *
     * @param enable pin connected to the enable channel of the h-bridge
     * @param in1 pin connected to in1 or in3
     * @param in2 pin connected to in2 or in4
     */
    void begin(uint8_t enable, uint8_t in1, uint8_t in2);


    void    run(uint8_t direcction = 1, uint8_t speed = 255);
    bool    isMoving();
    void    forward(uint8_t speed);
    void    backward(uint8_t speed);
    void    invert();
    void    stop();
    uint8_t getSpeed();
};

#endif
