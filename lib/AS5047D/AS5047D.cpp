#include "AS5047D.h"
#include <SPI.h>


void AS5047D::init(int C) {
  SPI.begin();                      /* SPI is the communication protocol used by the 25LC256 */
  this->CSN = C;
  this->angle = 0;
  this->prevAngle = 0;
  this->value = 0;
  this->RERotations = 0;
  pinMode(this->CSN, OUTPUT);
  digitalWrite(this->CSN, HIGH);
}



int AS5047D::read() {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

  digitalWrite(this->CSN, LOW);
  delayMicroseconds(1);
  this->value = SPI.transfer16(AS5047D_ADDRESS);
  this->value = (this->value & AS5047D_ADDRESS);
  digitalWrite(this->CSN, HIGH);

  SPI.endTransaction();

  this->angle = (((unsigned long)this->value) * 4096UL / 16384UL) + (this->RERotations * 4096);

  if (this->angle < this->prevAngle - 3400) {
    this->RERotations++;
    this->angle += 4096;
  }
  else if (this->angle > this->prevAngle + 3400) {
    this->RERotations--;
    this->angle -= 4096;
  }
  this->prevAngle = this->angle;
  return this->angle;
}
