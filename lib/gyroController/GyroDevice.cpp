#ifndef __GYRODEVICE_H__
#define __GYRODEVICE_H__

#include <GyroDevice.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;   				// basic linea algebra 

GyroDevice::GyroDevice() {}

GyroDevice::~GyroDevice() {}

void GyroDevice::begin() {


	analogWriteFrequency(pitchA, 23437.5f); // PWM range remains 0 - 255, any speed on the controller are adjusted by % (0-100)
	//https://www.pjrc.com/teensy/td_pulse.html
	// the magic number below is the resolution points of the encoder per degree of rotation
	const float ratioYaw = 78.04f;
	const float ratioPitch = 97.2f;

	// initialize both axles 

	yawAxle.begin(yawA, yawB, yawData, yawClk, yawCs, ratioYaw);
	yawAxle.setMinVel(5.0f);
	yawAxle.setMaxVel(24.0f);

	pitchAxle.begin(pitchA, pitchB, pitchData, pitchClk, pitchCs, ratioPitch);
	pitchAxle.setMinVel(5.0f);
	pitchAxle.setMaxVel(24.0f);

	this->setPidMove();

	flywheel.begin(flywheelPin);
	flywheel.setSpeed(0);
	brake.begin(brakePin);
	//com.begin();

	/******************************* Initialize torque formula **********************************/

	omegaDisk.Fill(0);
	omegaDotDisk.Fill(0);
	omegaGimbal.Fill(0);

	rot_a_to_d <<
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0;

	I <<
		inertiaX, 0, 0,
		0, inertiaY, 0,
		0, 0, inertiaZ;

}

void GyroDevice::setPidMove() {
	yawAxle.setPidVal(1.0, 0.0, 10000.0);
	pitchAxle.setPidVal(3.0, 0.0, 18000.0);
}

void GyroDevice::setPidImpact() {
	yawAxle.setPidVal(1.0, 0.0, 1000.0);
	pitchAxle.setPidVal(1.0, 0.0, 1000.0);
}

/**
 * @brief call the encoders and update the reading of current position for both axles
 */
void GyroDevice::refreshReading() {
	yawAxle.updateReadings();
	pitchAxle.updateReadings();
}

/**
 * @brief update the target angle for both axles and move towards target
 */
void GyroDevice::updatePosition() {
	yawAxle.updatePosition();
	pitchAxle.updatePosition();
	brake.update();
}

/**
 * @brief adjust the maximun angular velocity for the gimbals
 *
 * @param _vel value from 0-100
 */
void GyroDevice::setGimbalSpeed(float _vel) {
	yawAxle.setMaxVel(_vel);
	pitchAxle.setMaxVel(_vel);
}

/**
 * @brief flag to indicate the gimbal has reached target position
 */
bool GyroDevice::reachTargetAngles() {
	if (yawAxle.reachTarget() && pitchAxle.reachTarget()) return true;
	else return false;
}

/******** DEBUG ********/

void GyroDevice::printData() {
	//int32_t tmpMillis = millis();
	static int32_t oldMillis = 0;
	Serial1.print(pitchAxle.readings.pos);
	Serial1.print(',');
	Serial1.print(pitchAxle.readings.radians);
	Serial1.print(',');
	Serial1.print(pitchAxle.readings.angle);
	Serial1.print(',');
	Serial1.print(pitchAxle.readings.vel);
	Serial1.print(',');
	Serial1.print(pitchAxle.readings.acc);
	Serial1.print(',');
	Serial1.print(millis() - oldMillis);
	Serial1.print(',');
	Serial1.print(pitchAxle.readings.acc);
	Serial1.println("");
	oldMillis = millis();
	// Serial1.print(',');
	// Serial1.print(yawAxle.readings.pos);
	// Serial1.print(',');
	// Serial1.print(yawAxle.readings.radians);
	// Serial1.print(',');
	// Serial1.print(yawAxle.readings.angle);
	// Serial1.print(',');
	// Serial1.print(yawAxle.readings.vel);
	// Serial1.print(',');
	// Serial1.println(yawAxle.readings.acc);
}


/******** MODEL ********/

// @brief Calculate the output torque based on the existing variables(position, angular velocity and delta time)

void GyroDevice::calculateTorque() {

	float pitchPos = pitchAxle.readings.radians;
	float yawPos = yawAxle.readings.radians;

	// update vector components
	omegaDotDisk.X() = (-yawAxle.readings.acc * sin(pitchPos)) - (yawAxle.readings.vel * pitchAxle.readings.vel * sin(pitchPos));
	omegaDotDisk.Y() = pitchAxle.readings.acc;
	// [\dot\psi] disk angular acceleration is omited as the disk spins at a constant velocity
	omegaDotDisk.Z() = (yawAxle.readings.acc * cos(pitchPos)) - (yawAxle.readings.vel * pitchAxle.readings.vel * sin(pitchPos));

	// left side of the equation 
	Point tmp1 = I * omegaDotDisk;

	omegaDisk.X() = -yawAxle.readings.vel * sin(pitchPos);
	omegaDisk.Y() = pitchAxle.readings.vel;
	omegaDisk.Z() = diskAngVel + (yawAxle.readings.vel * cos(pitchPos));

	// right side of the equation
	Point tmp2 = I * omegaDisk;

	omegaGimbal.X() = -yawAxle.readings.vel * sin(pitchPos);
	omegaGimbal.Y() = pitchAxle.readings.vel;
	omegaGimbal.Z() = yawAxle.readings.vel * cos(pitchPos);

	Point tmp3 = omegaGimbal.CrossProduct(tmp2);

	longEqTorque = tmp1 + tmp3;

	shortEqTorque.X() = (inertiaZ * 2) * (((pitchAxle.readings.vel * diskAngVel) / 2) - (0.25 * yawAxle.readings.acc * sin(pitchAxle.readings.pos)));

	shortEqTorque.Y() = (inertiaX) * (pitchAxle.readings.vel * sin(pitchAxle.readings.pos) *
		((pitchAxle.readings.vel * cos(yawAxle.readings.pos)) + (2 * diskAngVel))
		+ pitchAxle.readings.acc);

	shortEqTorque.Z() = inertiaZ *
		((pitchAxle.readings.acc * cos(pitchAxle.readings.pos)) -
			(pitchAxle.readings.vel * yawAxle.readings.vel * sin(pitchAxle.readings.pos)));

	// 	//R_a_d
	rot_a_to_d <<
		cos(pitchPos) * cos(yawPos), -sin(yawPos), sin(pitchPos)* cos(yawPos),
		cos(pitchPos)* sin(yawPos), cos(yawPos), sin(pitchPos)* sin(yawPos),
		sin(pitchPos), 0, cos(pitchPos);

	//M_d
	longEqTorque = rot_a_to_d * longEqTorque;
	shortEqTorque = rot_a_to_d * shortEqTorque;
}

#endif // __GYRODEVICE_H__