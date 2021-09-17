/**
 * @file AS5045.h
 * @brief returns the current reading, 12 bit resolution (360 === 4096)
 *  - encoder position on boot is set to 0
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef AS5045_h
#define AS5045_h

#include <Arduino.h>
#include <RunningMedian.h>

class AS5045
{
public:
  AS5045();
  virtual ~AS5045();

  boolean begin(byte pinCS, byte pinCLK, byte pinDO);
  int32_t readEnc();                                      // read the encoder value on steps (4096 steps, 12 bit resolution per rotation)
  void setZero();                                         // substract initial reading as offset to set initial position as 0

private:
  int32_t read();
  void reset();

  RunningMedian* outlierFilter;

  byte _pinCLK;
  byte _pinCS;
  byte _pinDO;

  volatile int32_t _turns, _reading, _lastReading, _startOffset;
};
#endif /* AS5045_h */