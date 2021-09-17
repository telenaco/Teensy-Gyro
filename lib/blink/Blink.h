#ifndef BLINK_H
#define BLINK_H

#include <Arduino.h>
#include <inttypes.h>

class Blink {
    uint32_t _interval;
    uint32_t _lastTime;
    uint8_t _pin;

public:
    Blink() {};

    virtual ~Blink() {};

    /**
     * @brief Method to blink and LED 
     * 
     * @param pin Pin connected to the LED
     * @param time interval time
     */
    void begin(uint8_t pin, uint16_t time) {
        pinMode(pin, OUTPUT);
        _interval = time;
        _pin = pin;
    }

    /**
     * @brief update the state of the LED, if time has pass toggle state
     * 
     * @return true toggle LED
     * @return false LED condition has not changed
     */
    bool update() {
        uint32_t time = millis();

        if (time > (_lastTime + _interval)) {
            _lastTime = time;
            digitalWrite(_pin, !digitalRead(_pin));
            return true;
        }
        return false;
    }
};

#endif
