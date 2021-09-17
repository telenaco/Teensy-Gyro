#ifndef __GYRODEVICE_H__
#define __GYRODEVICE_H__

#include <GyroDevice.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;   				// basic linea algebra 

GyroDevice::GyroDevice() {
}

GyroDevice::~GyroDevice() {
}

void GyroDevice::begin() {

	analogWriteFrequency(pitchA, 23437.5); // PWM range remains 0 - 255, any speed on the controller are adjusted by % (0-100)
	//https://www.pjrc.com/teensy/td_pulse.html
	// the magic number below is the resolution points of the encoder per degree of rotation
	const float ratioYaw = 81.14;
	const float ratioPitch = 97.2;

	// initialize both axles
	yawAxle.begin(yawA, yawB, yawData, yawClk, yawCs, ratioYaw);
	yawAxle.setMinVel(17); // mminimu value requried to overcome friction for the axle to start rotating
	yawAxle.setPidVal(1.8, 0.0, 1200.0);
	pitchAxle.begin(pitchA, pitchB, pitchData, pitchClk, pitchCs, ratioPitch);
	pitchAxle.setMinVel(15);  // mminimu value requried to overcome friction for the axle to start rotating
	pitchAxle.setPidVal(3.0, 0.0, 1000.0);

	flywheel.begin(flywheelPin);
	flywheel.setSpeed(0);
	brake.begin(brakePin);
	com.begin();

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

/**
 * @brief call the encoders and update the current position of both axles 
 */
void GyroDevice::refreshReading() {
	yawAxle.updateReadings();
	pitchAxle.updateReadings();
}

/**
 * @brief update the target angle per axle and move towards target
 */
void GyroDevice::updatePosition() {
	yawAxle.updatePosition();
	pitchAxle.updatePosition();
	brake.update();
}

/**
 * @brief Calculate the output torque based on the existing variables(position, angular velocity and delta time)
 */
void GyroDevice::calculateTorque() {

	float pitchPos = pitchAxle.readings.radians;
	float yawPos = yawAxle.readings.radians;

	// update vector components
	omegaDotDisk.X() = (-yawAxle.readings.acc * sin(pitchPos)) - (yawAxle.readings.vel * pitchAxle.readings.vel * sin(pitchPos));
	omegaDotDisk.Y() = pitchAxle.readings.acc;
	// [\dot\psi] disk angular acceleration is omited as the disk spins at a constant velocity
	omegaDotDisk.Z() = (yawAxle.readings.acc * cos(pitchPos)) - (yawAxle.readings.vel * pitchAxle.readings.vel * sin(pitchPos));

	// com.sendTorque(&omegaDotDisk.X(), &omegaDotDisk.Y(), &omegaDotDisk.Z());
	// plotResult(&omegaDotDisk.X(), &omegaDotDisk.Y(), &omegaDotDisk.Z());

	// left side of the equation 
	Point tmp1 = I * omegaDotDisk;

	// com.sendTorque(tmp1.X(), tmp1.Y(), tmp1.Z());
	// plotResult(&tmp1.X(), &tmp1.Y(),&tmp1.Z());

	omegaDisk.X() = -yawAxle.readings.vel * sin(pitchPos);
	omegaDisk.Y() = pitchAxle.readings.vel;
	omegaDisk.Z() = diskAngVel + (yawAxle.readings.vel * cos(pitchPos));

	// com.sendTorque(&omegaDisk.X(), &omegaDisk.Y(), &omegaDisk.Z());
	// plotResult(&omegaDisk.X(), &omegaDisk.Y(), &omegaDisk.Z());

	// right side of the equation
	Point tmp2 = I * omegaDisk;

	// com.sendTorque(tmp2.X(), tmp2.Y(), tmp2.Z());
	// plotResult(&tmp2.X(), &tmp2.Y(), &tmp2.Z());

	omegaGimbal.X() = -yawAxle.readings.vel * sin(pitchPos);
	omegaGimbal.Y() = pitchAxle.readings.vel;
	omegaGimbal.Z() = yawAxle.readings.vel * cos(pitchPos);

	// com.sendTorque(&omegaGimbal.X(), &omegaGimbal.Y(), &omegaGimbal.Z());	
	// plotResult(&omegaGimbal.X(), &omegaGimbal.Y(), &omegaGimbal.Z());	

	Point tmp3 = omegaGimbal.CrossProduct(tmp2);

	longEqTorque = tmp1 + tmp3;

	// com.sendTorque(tmp3.X(), tmp3.Y(), tmp3.Z());
	// plotResult(&tmp3.X(), &tmp3.Y(), &tmp3.Z());
	// static double I = (diskMass * pow(radious, 2));

	shortEqTorque.X() = (inertiaZ*2) * (((pitchAxle.readings.vel * diskAngVel) / 2) - (0.25 * yawAxle.readings.acc * sin(pitchAxle.readings.pos)));
	
	shortEqTorque.Y() = (inertiaX) * (pitchAxle.readings.vel * sin(pitchAxle.readings.pos) *
		 ((pitchAxle.readings.vel * cos(yawAxle.readings.pos)) + (2 * diskAngVel))
		 + pitchAxle.readings.acc);

	shortEqTorque.Z() = inertiaZ *
		 ((pitchAxle.readings.acc * cos(pitchAxle.readings.pos)) -
			 (pitchAxle.readings.vel * yawAxle.readings.vel * sin(pitchAxle.readings.pos)));


	// 	//R_a_d
	rot_a_to_d <<
		cos(pitchPos)*cos(yawPos),		-sin(yawPos),	sin(pitchPos)*cos(yawPos),
		cos(pitchPos)*sin(yawPos),		cos(yawPos),	sin(pitchPos)*sin(yawPos),
		-sin(pitchPos),				 	0,				cos(pitchPos);

	//M_d
	longEqTorque = rot_a_to_d * longEqTorque;
	shortEqTorque = rot_a_to_d * shortEqTorque;

}

void GyroDevice::sendFormulaTwo() {
	com.sendTwoFormulas(
		&longEqTorque.X(),
		&longEqTorque.Y(),
		&longEqTorque.Z(),
		&shortEqTorque.X(),
		&shortEqTorque.Y(),
		&shortEqTorque.Z(),
		&pitchAxle.readings,
		&yawAxle.readings);
}

void GyroDevice::plotResult(float* x, float* y, float* z) {
	Serial.print(*x, 6);
	Serial.print(",");
	Serial.print(*y, 6);
	Serial.print(",");
	Serial.print(*z, 6);
	Serial.println("");
	Serial.send_now();
}

void GyroDevice::sendTorque() {
	com.sendThreeValues(&longEqTorque.X(), &longEqTorque.Y(), &longEqTorque.Z());
}

void GyroDevice::sendData() {
	com.sendData(
		&longEqTorque.X(),
		&longEqTorque.Y(),
		&longEqTorque.Z(),
		&pitchAxle.readings,
		&yawAxle.readings);
}


void GyroDevice::printData() {
	Serial.print(longEqTorque.X());
	Serial.print(" <-> ");	
	Serial.print(longEqTorque.Y());
	Serial.print(" <-> ");
	Serial.print(longEqTorque.Z());
	Serial.print(" <-> ");
	Serial.print(pitchAxle.readings.pos);
	Serial.print(" <-> ");
	Serial.print(pitchAxle.readings.angle);
	Serial.print(" <-> ");
	Serial.print(pitchAxle.readings.vel);
	Serial.print(" <-> ");
	Serial.print(pitchAxle.readings.acc);
	Serial.print(" <-> ");
	Serial.print(yawAxle.readings.pos);
	Serial.print(" <-> ");
	Serial.print(yawAxle.readings.angle);
	Serial.print(" <-> ");
	Serial.print(yawAxle.readings.vel);
	Serial.print(" <-> ");
	Serial.print(yawAxle.readings.acc);
	Serial.println("");
}

void GyroDevice::setGimbalSpeed(int _vel) {
	yawAxle.setMaxVel(_vel);
	pitchAxle.setMaxVel(_vel);
}

bool GyroDevice::reachTargetAngles() 
{
	if (yawAxle.reachTarget() && pitchAxle.reachTarget()) return true;
	else return false;
}

void GyroDevice::brakeOn(uint32_t duration) {
	brake.close(duration);
}

void GyroDevice::brakeSmooth(uint32_t duration) {
	brake.closeSmooth(duration);
}

void GyroDevice::flywheelSpeed(int8_t speed) {
	flywheel.setSpeed(speed);
}

void GyroDevice::pitchTargetAngle(double angle) {
	pitchAxle.targetAngle(angle);
}

void GyroDevice::yawTargetAngle(double angle) {
	yawAxle.targetAngle(angle);
}

void GyroDevice::printPitchPos()
{
	Serial.println(pitchAxle.readings.angle);
}

void GyroDevice::plotYawReadings() {
	Serial.print(yawAxle.readings.pos);
	Serial.print(",");
	Serial.print(yawAxle.readings.angle);
	Serial.print(",");
	Serial.print(yawAxle.readings.radians);
	Serial.print(",");
	Serial.print(yawAxle.readings.vel);
	Serial.print(",");
	Serial.println(yawAxle.readings.acc);
}
void GyroDevice::plotPitchReadings() {
	Serial.print(pitchAxle.readings.pos);
	Serial.print(",");
	Serial.print(pitchAxle.readings.angle);
	Serial.print(",");
	Serial.print(pitchAxle.readings.radians);
	Serial.print(",");
	Serial.print(pitchAxle.readings.vel);
	Serial.print(",");
	Serial.println(pitchAxle.readings.acc);
}

int GyroDevice::getYaw() {
	return (int)yawAxle.readings.angle;
}

int GyroDevice::getPitch() {
	return (int)pitchAxle.readings.angle;
}

#endif // __GYRODEVICE_H__