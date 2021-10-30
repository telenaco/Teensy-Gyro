#include "AS5045.h"


AS5045::AS5045() {}

AS5045::~AS5045() {}

boolean AS5045::begin(byte pinCS, byte pinCLK, byte pinDO) {
  _pinCS = pinCS;
  _pinCLK = pinCLK;
  _pinDO = pinDO;

  pinMode(_pinCLK, OUTPUT);
  pinMode(_pinCS, OUTPUT);
  pinMode(_pinDO, INPUT_PULLUP);

  digitalWrite(_pinCLK, LOW);
  digitalWrite(_pinCS, LOW);

  outlierFilter = new RunningMedian(3);

  this->reset();

  return true;
}

void AS5045::reset() {
  _turns = 0;
  _reading = 0;
  _lastReading = 0;
  _startOffset = 0;
  this->setZero();
}

int32_t AS5045::read() {
  noInterrupts();
  digitalWrite(_pinCS, LOW);
  int32_t value = 0;

  for (byte i = 0; i < 12; i++) {
    digitalWrite(_pinCLK, LOW);
    delayMicroseconds(1);
    digitalWrite(_pinCLK, HIGH);
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(_pinDO);
  }

  byte status = 0;
  for (byte i = 0; i < 6; i++) {
    digitalWrite(_pinCLK, LOW);
    delayMicroseconds(1);
    digitalWrite(_pinCLK, HIGH);
    delayMicroseconds(1);
    status = (status << 1) | digitalRead(_pinDO);
  }
  digitalWrite(_pinCS, HIGH);
  interrupts();

  return value;
}

int32_t AS5045::readEnc() {
  _reading = read();

  int16_t q2min = 4096 / 4;                                         // 1st quadrant && 4th quadrant
  int16_t q3max = q2min * 3;                                        // keep track of full rotations

  if ((_reading < q2min) && (_lastReading > q3max)) _turns++;       // If transition from quadrant 4 to quadrant 1, increase _turns count.
  else if ((_lastReading < q2min) && (_reading > q3max)) _turns--;  // If transition from quadrant 1 to quadrant 4, decrease _turns count.

  _lastReading = _reading;

  _reading = (_turns * 4096) + _reading;
  _reading -= _startOffset;
  outlierFilter->add(_reading);
  _reading = outlierFilter->getMedian();

  return _reading;
}

void AS5045::setZero() {
  _startOffset = read();
  _lastReading = _startOffset;
  _reading = read();
}