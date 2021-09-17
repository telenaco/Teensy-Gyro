/**
 * @file PID_v1.cpp
 *
 * @brief modified version of
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * This Library is licensed under the MIT License
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_v1.h"

 /**
  * @brief Construct a new empty PID::PID object
  *
  */
PID::PID() {}

PID::PID(double* Input, double* Output, double* Setpoint,
    double Kp, double Ki, double Kd, int POn, int ControllerDirection) {
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);  //default output limit corresponds to
        //the arduino pwm limits

    // *************** chande millis() to micros, the control loop for gyro runs under a ms  ****************

    SampleTime = 1000;  //edited to run on micros instead of millis 

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = micros() - SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input, double* Output, double* Setpoint,
    double Kp, double Ki, double Kd, int ControllerDirection)
    : PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {
}

/**
 * @brief Construct a new PID::PID object
 * links the PID to the Input, Output, and setpoint.
 * set also initial parameters
 *
 * @param Input
 * @param Output
 * @param Setpoint
 * @param Kp - proportional
 * @param Ki - integral
 * @param Kd - derivative
 * @param POn for specifiying proportional mode
 * @param ControllerDirection
 */

 /*Constructor (...)*********************************************************
  *    To allow backwards compatability for v1.1, or for people that just want
  *    to use Proportional on Error without explicitly saying so
  ***************************************************************************/


/**
 * @brief This function should be called every time "void loop()" executes.
 * The function will decide for itself whether a new
 * Pid Output needs to be computed.  returns true when the output is computed,
 * false when nothing has been done.
*/
bool PID::Compute() {

    //if (!inAuto) return false;

    unsigned long now = micros();
    unsigned long timeChange = (now - lastTime);

    if (timeChange >= SampleTime) {
        /*Compute all the working error variables*/
        double input = *myInput;
        double error = (*mySetpoint - input);
        double dInput = (input - lastInput);
        double output;
        outputSum += (ki * error);

        /*Add Proportional on Measurement, if P_ON_M is specified*/

        if (!pOnE) outputSum -= kp * dInput;
        if (outputSum > outMax)
            outputSum = outMax;
        else if (outputSum < outMin)
            outputSum = outMin;
        /*Add Proportional on Error, if P_ON_E is specified*/
        if (pOnE)
            output = kp * error;
        else
            output = 0;
        /*Compute Rest of PID Output*/
        output += outputSum - kd * dInput;
        if (output > outMax)
            output = outMax;
        else if (output < outMin)
            output = outMin;
        *myOutput = output;
        /*Remember some variables for next time*/
        lastInput = input;
        lastTime = now;
        return true;
    }
    else
        return false;
}

/**
* @brief This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
* @param kp - proportional value
* @param Ki - integral value
* @param kd - derivative value
* @param POn
*/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn) {
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

    pOn = POn;
    pOnE = POn == P_ON_E;

    dispKp = Kp;
    dispKi = Ki;
    dispKd = Kd;

    double SampleTimeInSec = ((double)SampleTime) / 1000000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;

    if (controllerDirection == REVERSE) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

/**
 * @brief Set Tunings using the last-rembered POn setting
 *
 * @param Kp
 * @param Ki
 * @param Kd
*/
void PID::SetTunings(double Kp, double Ki, double Kd) {
    SetTunings(Kp, Ki, Kd, pOn);
}

/**
 * @brief  sets the period, in Milliseconds, at which the calculation is performed
 *
 * @param NewSampleTime
 */
void PID::SetSampleTime(int NewSampleTime) {
    if (NewSampleTime > 0) {
        double ratio = (double)NewSampleTime / (double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min,
    double Max) {
    if (Min >= Max) return;
    outMin = Min;
    outMax = Max;

    if (inAuto) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin)
            *myOutput = outMin;

        if (outputSum > outMax) outputSum = outMax;
        else if (outputSum < outMin)
            outputSum = outMin;
    }
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
void PID::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto) { /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}
/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/
void PID::Initialize() {
    outputSum = *myOutput;
    lastInput = *myInput;
    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin)
        outputSum = outMin;
}

/* @
SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing.  This is called from the constructor.
******************************************************************************/
void PID::SetControllerDirection(int Direction) {
    if (inAuto && Direction != controllerDirection) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
    controllerDirection = Direction;
}

/* @brief these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp() { return dispKp; }
double PID::GetKi() { return dispKi; }
double PID::GetKd() { return dispKd; }
int PID::GetMode() { return inAuto ? AUTOMATIC : MANUAL; }
int PID::GetDirection() { return controllerDirection; }
