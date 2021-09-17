#include "gyroscope.h"

#include <AS5045.h>
#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

Servo flywheel;

AS5045 encoderX;
AS5045 encoderY;

const int period = 20000;

void Gyroscope::initGyro(int IN1, int IN2, int IN3, int IN4, int ENA, int ENB,
    int esc,
    int yawCLK, int yawCS, int yawData,
    int pitchCLK, int pitchCS, int pitchData,
    int brake) {
    this->motorX.IN1 = IN1;
    this->motorX.IN2 = IN2;
    this->motorY.IN1 = IN3;
    this->motorY.IN2 = IN4;
    this->motorX.EN = ENA;
    this->motorY.EN = ENB;
    this->motorX.goalPosition = 0;
    this->motorY.goalPosition = 0;
    this->motorX.startPos = -140;
    this->motorY.endPos = 490;
    this->motorY.startPos = -35;
    this->motorY.endPos = 390;

    this->esc.pin = esc;
    this->esc.value = 1500;
    this->esc.speedVal = 0;

    flywheel.attach(this->esc.pin);
    flywheel.writeMicroseconds(this->esc.value);

    delay(7000);

    pinMode(this->motorX.IN1, OUTPUT);
    pinMode(this->motorX.IN2, OUTPUT);
    pinMode(this->motorX.EN, OUTPUT);

    pinMode(this->motorY.IN1, OUTPUT);
    pinMode(this->motorY.IN2, OUTPUT);
    pinMode(this->motorY.EN, OUTPUT);

    digitalWrite(this->motorX.IN1, HIGH);
    digitalWrite(this->motorX.IN2, LOW);
    analogWrite(this->motorX.EN, 0);

    digitalWrite(this->motorY.IN1, HIGH);
    digitalWrite(this->motorY.IN2, LOW);
    analogWrite(this->motorY.EN, 0);

    encoderX.begin(yawData, yawCLK, yawCS);
    encoderY.begin(pitchData, pitchCLK, pitchCS);

    this->motorX.angle = encoderX.getReadingOffset() / 95;
    this->motorY.angle = encoderY.getReadingOffset() / 146;

    this->brake.pin = brake;
    this->brake.pulse = 700;
    this->brake.pulseInterval = 700;
    this->brake.previousMicros = 0;
    this->brake.currentMicros = 0;
    this->brake.range[0] = 1100;
    this->brake.range[1] = 1500;
    this->brake.state = LOW;
    this->brake.closed = false;
    this->brake.duration = 0;

    pinMode(this->brake.pin, OUTPUT);
}

////////////////////////////////////////////////////////////////////////////////
/* flywheel */
/* 1100 ccw and 1900 cw, stop at 1500 */
////////////////////////////////////////////////////////////////////////////////

void Gyroscope::flywheelSetSpeed(int value) {
    Serial.println(value);
    if ((this->esc.value > 1500 && value < 1500) || (this->esc.value < 1500 && value > 1500) || (value == 1500 && this->esc.value != 1500)) {
        flywheel.writeMicroseconds(1500);
        this->esc.value = 1500;
        this->brake.duration = 500;
        this->brakeFlywheel(1);
        if (value != 1500) {
            this->flywheelSetSpeed(value);
        }
    }
    else if (value != 1500) {
        this->esc.dir = value >= 1500 ? true : false;
        if (value < this->esc.value) {
            flywheel.writeMicroseconds(value);
            this->brake.duration = 250;
            this->brakeFlywheel(1);

        }
        else if (value > this->esc.value) {
            flywheel.writeMicroseconds(value);
        }
        this->esc.value = value;
    }
}

////////////////////////////////////////////////////////////////////////////////
/* axis */
////////////////////////////////////////////////////////////////////////////////

void Gyroscope::setAngles(int xAxis, int yAxis) {
    const int mappedXvalue = map(xAxis, -85, 445, this->motorX.startPos, this->motorX.endPos);
    const int mappedYvalue = map(yAxis, -82, 442, this->motorY.startPos, this->motorY.endPos);

    Serial.println((String)mappedXvalue + ' ' + mappedYvalue);

    this->motorX.goalPosition = mappedXvalue;
    this->motorY.goalPosition = mappedYvalue;
}

void Gyroscope::getAngles() {
    this->motorX.angle = encoderX.getReadingOffset() / 96;
    this->motorY.angle = encoderY.getReadingOffset() / 146;

    Serial.println((String)this->motorX.angle + ' ' + this->motorY.angle);
}

void Gyroscope::updateAxes() {
    if (this->brake.closed) {
        if (this->brake.startMillis - millis() > this->brake.duration) {
            this->brakeFlywheel(0);
        }
    }

    this->motorX.angle = encoderX.getReadingOffset() / 95;
    this->motorY.angle = encoderY.getReadingOffset() / 146;

    int distanceX = this->motorX.goalPosition - this->motorX.angle;
    int distanceY = this->motorY.goalPosition - this->motorY.angle;

    /* need to tweak these values */
    this->motorX.speedVal = map(abs(distanceX), 3, 1000, 150, 250);
    this->motorY.speedVal = map(abs(distanceY), 3, 1000, 150, 250);

    this->motorX.speedVal = constrain(this->motorX.speedVal, 150, 250);
    this->motorY.speedVal = constrain(this->motorY.speedVal, 150, 250);

    if (distanceX < -3) {
        digitalWrite(motorX.IN1, LOW);
        digitalWrite(motorX.IN2, HIGH);
    }
    else if (distanceX > 3) {
        digitalWrite(motorX.IN1, HIGH);
        digitalWrite(motorX.IN2, LOW);
    }
    else {
        this->motorX.speedVal = 0;
    }

    if (distanceY < 3) {
        digitalWrite(motorY.IN1, LOW);
        digitalWrite(motorY.IN2, HIGH);
    }
    else if (distanceY > -3) {
        digitalWrite(motorY.IN1, HIGH);
        digitalWrite(motorY.IN2, LOW);
    }
    else {
        this->motorY.speedVal = 0;
    }

    //  Serial.println((String) this->motorX.speedVal + ' ' + distanceX);
    //  Serial.println((String) this->motorY.speedVal + ' ' + distanceY);

    analogWrite(this->motorX.EN, this->motorX.speedVal);
    analogWrite(this->motorY.EN, this->motorY.speedVal);
}

////////////////////////////////////////////////////////////////////////////////
/* brake */
////////////////////////////////////////////////////////////////////////////////

void Gyroscope::brakeInterval(int pulseIncrement, int pulseStep) {
    this->brake.currentMicros = micros();

    if (this->brake.currentMicros - this->brake.previousMicros >= this->brake.pulseInterval) {
        this->brake.previousMicros = this->brake.currentMicros;

        if (this->brake.state == LOW) {
            this->brake.state = HIGH;
        }
        else {
            this->brake.state = LOW;
        }

        this->brake.pulse += (pulseStep * pulseIncrement);

        if (this->brake.pulseInterval != this->brake.pulse) {
            this->brake.pulseInterval = this->brake.pulse;
        }
        else {
            this->brake.pulseInterval = period - this->brake.pulse;
        }
        digitalWrite(this->brake.pin, this->brake.state);
    }
}

void Gyroscope::brakeFlywheel(int brakeWheel) {
    this->brake.startMillis = millis();

    if (this->brake.pulse > this->brake.range[brakeWheel]) {
        while (this->brake.pulse > this->brake.range[brakeWheel]) {
            this->brakeInterval(-1, 10);
        }
    }
    else if (this->brake.pulse < this->brake.range[brakeWheel]) {
        while (this->brake.pulse < this->brake.range[brakeWheel]) {
            this->brakeInterval(1, 10);
        }
    }
    this->brake.closed = brakeWheel == 1 ? true : false;
}

////////////////////////////////////////////////////////////////////////////////
/* communication */
////////////////////////////////////////////////////////////////////////////////

void Gyroscope::receiveDataFromUnity() {
    if (Serial.available() > 0) {
        String serialResponse = Serial.readStringUntil('\r\n', 20);

        char buf[20];
        serialResponse.toCharArray(buf, 20);

        int _speed, _angleX, _angleY;

        if (sscanf(buf, "%d,%d,%d", &_speed, &_angleX, &_angleY) == 3) {
            this->esc.speedVal = map(_speed, -100, 100, 1100, 1900);
            Serial.println(this->esc.speedVal);
            this->flywheelSetSpeed(this->esc.speedVal);
            this->setAngles(_angleX, _angleY);
        }

        Serial.println((String)_speed + " , " + _angleX + " , " + _angleY);
    }
}

////////////////////////////////////////////////////////////////////////////////
/* calibrate axes */
////////////////////////////////////////////////////////////////////////////////

void Gyroscope::calibrate(int axis) {
    struct dcMotors motor = axis == 0 ? this->motorX : this->motorY;

    boolean setStart = false;
    boolean setEnd = false;

    motor.angle = axis == 0 ? encoderX.getReadingOffset() / 95 : encoderY.getReadingOffset() / 146;

    int startValue = motor.angle;
    int minRotation = axis == 0 ? 43000 : 50000;

    motor.prevAngle = motor.angle + 200;

    //rotate clockwise
    digitalWrite(motor.IN1, LOW);
    digitalWrite(motor.IN2, HIGH);

    while (setEnd == false) {
        motor.angle = axis == 0 ? encoderX.getReadingOffset() / 95 : encoderY.getReadingOffset() / 146;

        if (!setStart) {
            if (motor.prevAngle - motor.angle >= -1) {
                if (startValue - motor.angle < 100) {
                    motor.speedVal = axis == 0 ? 160 : 80;
                }
                else {
                    motor.speedVal = axis == 0 ? map(motor.prevAngle - motor.angle, -1, 150, 120, 160) : map(motor.prevAngle - motor.angle, -1, 400, 65, 70);
                }

            }
            else if (motor.prevAngle - motor.angle < -1) {
                motor.startPos = motor.angle;
                setStart = true;
                delay(1000);
                digitalWrite(motor.IN1, HIGH);  // change direction
                digitalWrite(motor.IN2, LOW);
                motor.speedVal = axis == 0 ? 160 : 80;
            }

        }
        else if (setStart) {
            if (motor.angle - motor.prevAngle > 0 || abs(motor.startPos - motor.angle) < 1000) {
                if (abs(motor.startPos - motor.angle) < 100) {
                    motor.speedVal = axis == 0 ? 160 : 80;
                }
                else {
                    motor.speedVal = axis == 0 ? map(motor.angle - motor.prevAngle, 0, 150, 120, 160) : map(motor.angle - motor.prevAngle, 0, 400, 65, 70);
                }
            }
            else if (motor.angle - motor.prevAngle <= 0) {
                if (abs(motor.startPos - motor.angle) < minRotation) {
                    delay(500);
                    this->calibrate(axis);
                    break;
                }
                else {
                    motor.endPos = motor.angle;
                    setEnd = true;
                    motor.speedVal = axis == 0 ? 80 : 60;
                    analogWrite(motor.EN, motor.speedVal);
                    Serial.println("done");
                    if (axis == 0) {
                        this->motorX.startPos = motor.startPos;
                        this->motorX.endPos = motor.endPos;
                        this->motorX.angle = encoderX.getReadingOffset() / 95;
                    }
                    else {
                        this->motorY.startPos = motor.startPos;
                        this->motorY.endPos = motor.endPos;
                        this->motorY.angle = encoderY.getReadingOffset() / 146;
                    }
                    break;
                }
            }
        }

        motor.speedVal = constrain(motor.speedVal, 0, 255);
        analogWrite(motor.EN, motor.speedVal);
        motor.prevAngle = motor.angle;

        delayMicroseconds(1500);
    }

    Serial.println((String)motor.startPos + " , " + motor.endPos);
    Serial.println((String)"calibrated " + axis + ", " + (motor.endPos - motor.startPos));
}

/*
void Gyroscope::calibrate() {

   boolean setStart = false;
   boolean setEnd = false;

  this->motorX.angle = encoderX.read();
  this->motorY.angle = encoderY.read();

   int xStart = this->motorX.angle;
   int yStart = this->motorY.angle;

  this->motorX.prevAngle = this->motorX.angle + 200;
  this->motorY.prevAngle = this->motorY.angle + 200;

  //rotate clockwise
  digitalWrite(this->motorY.IN1, LOW);
  digitalWrite(this->motorY.IN2, HIGH);
  digitalWrite(this->motorX.IN1, LOW);
  digitalWrite(this->motorX.IN2, HIGH);

  Serial.println((String) "X " + this->motorX.angle + " , " + this->motorX.prevAngle);

  while (!setEnd) {

    this->motorX.angle = encoderX.read();

    if (!setStart) {
      Serial.println(this->motorX.prevAngle - this->motorX.angle);
      if (this->motorX.prevAngle - this->motorX.angle >= -1) {
        if (xStart - this->motorX.angle < 100) {
          this->motorX.speedVal = 180;
        } else {
          this->motorX.speedVal = map(this->motorX.prevAngle - this->motorX.angle, -1, 120, 125, 180);
        }

      } else if (this->motorX.prevAngle - this->motorX.angle < -1) {
        this->motorX.startPos = this->motorX.angle;
        setStart = true;
        Serial.println("change direction X");
        analogWrite(this->motorX.EN, 70);
        delay(1000);
        digitalWrite(this->motorX.IN1, HIGH);
        digitalWrite(this->motorX.IN2, LOW);
        this->motorX.speedVal = 160;
      }

//      Serial.println((String) "X " + this->motorX.speedVal + ", " + (this->motorX.prevAngle - this->motorX.angle));

    } else if (setStart) {

      Serial.println((String) "X " + this->motorX.angle + " , " + this->motorX.prevAngle);

      if (this->motorX.angle - this->motorX.prevAngle > 0 || abs(this->motorX.startPos - this->motorX.angle) < 1000) {

        if (abs(this->motorX.startPos - this->motorX.angle) < 100) {
          this->motorX.speedVal = 180;
        } else {
          this->motorX.speedVal = map(this->motorX.angle - this->motorX.prevAngle, 0, 180, 125, 160);
        }
      } else if (this->motorX.angle - this->motorX.prevAngle <= 0) {
        delay(1000);
        Serial.println((String) "Total width " + abs(this->motorX.startPos - this->motorX.angle));

        if (abs(this->motorX.startPos - this->motorX.angle) > 40000) {
          this->motorX.endPos = this->motorX.angle;
          this->motorX.speedVal = 70;
          setEnd = true;
        } else {
          this->calibrate();
        }
      }
//      Serial.println((String) "X " + this->motorX.speedVal + ", " + (this->motorX.angle - this->motorX.prevAngle));

    }
    this->motorX.speedVal = constrain(this->motorX.speedVal, 0, 255);

    analogWrite(this->motorX.EN, this->motorX.speedVal);
    this->motorX.prevAngle = this->motorX.angle;

    delayMicroseconds(1500);
  }

  Serial.println((String) "X " + this->motorX.startPos + " , " + this->motorX.endPos + " , " + (this->motorX.endPos - this->motorX.startPos));
  Serial.println((String) "calibrated X" + (this->motorX.endPos - this->motorX.startPos) + " , Y" + (this->motorY.endPos - this->motorY.startPos));
}
*/
