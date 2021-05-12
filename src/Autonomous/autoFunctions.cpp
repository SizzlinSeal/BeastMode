#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"

using namespace pros;

double kP = 0.09;
double kI = 0.0;
double kD = 0.09;

double turnKP = 0.046;
double turnKI = 0.0;
double turnKD = 0.05;

int error; // sensorValues - desiredValue = Position
int prevError = 0; // Position 20 milliseconds ago
int derivative; // error - prevError = speed
int totalError = 0; // totalError = totalError + error

int turnError; // sensorValues - desiredValue = Position
int turnPrevError = 0; // Position 20 milliseconds ago
int turnDerivative; // error - prevError = speed
int turnTotalError = 0; // totalError = totalError + error

//Autonomous settings
int desiredValue = 0; // motor ticks I think, bot 200rpm about 900 ticks / rev,
int desiredTurnValue = 0; // should work same as above

// variables used in slew
bool startSlew = false;
bool turning = false;

// variable that is part of stopping the PID
bool resetDriveSensors = false;

// variables modified for use
bool enableDrivePID = true;

// variables that have to be created outside thread
double lateralMotorPower = 0.0;
double turnMotorPower = 0.0;
double slew = 0.0;
double turnSlew = 0.0;
double slewPercent = 0.0;
double slewTurnPercent = 0.0;

int drivePID() {
  while(enableDrivePID) {
    if(resetDriveSensors) {
      resetDriveSensors = false;
      LF.tare_position();
      LB.tare_position();
      RF.tare_position();
      RB.tare_position();
    }

    // get the position of the motors
    int leftMotorAPosition = LF.get_position();
    int leftMotorBPosition = LB.get_position();
    int rightMotorAPosition = RF.get_position();
    int rightMotorBPosition = RB.get_position();

    /////////////////////////////////////////////////////////
    // Lateral Movement PID
    ////////////////////////////////////////////////////////////////////////////////

    // get average of the four motors
    int averagePosition = (leftMotorAPosition + leftMotorBPosition + rightMotorAPosition + rightMotorBPosition)/4;

    // Potential
    error = desiredValue - averagePosition;

    // Derivative
    derivative = error - prevError;

    // velocity
    totalError += error;

    // calculate motor power
    lateralMotorPower = error * kP + derivative * kD;

    /////////////////////////////////////////////////////////
    // Turning Movement PID
    ////////////////////////////////////////////////////////////////////////////////

    // get average of the four motors
    int turnDifference = ((leftMotorAPosition + leftMotorBPosition)/2) - ((rightMotorAPosition + rightMotorBPosition)/2);

    // Potential
    turnError = desiredTurnValue - turnDifference;

    // Derivative
    turnDerivative = turnError - turnPrevError;

    // velocity
    turnTotalError += turnError;

    // calculate power for turning
    turnMotorPower = turnError * turnKP + turnDerivative * turnKD;

    /////////////////////////////////////////////////////////////////////////////////
    // slew
    ////////////////////////////////////

    if(startSlew) {
      // lateral Slew
      if (turning == false) {
          slew = (lateralMotorPower / 100) * slewPercent;
          if(slewPercent < 100) {
             slewPercent += 10;
          }
      } else if (turning) {
          turnSlew = (turnMotorPower / 100) * slewTurnPercent;
          if(slewTurnPercent < 100) {
              slewTurnPercent += 10;
          }
      }
    }

    // spin the motors
    LF.move(slew + turnSlew);
    LB.move(slew + turnSlew);
    RF.move(slew - turnSlew);
    RB.move(slew - turnSlew);

    // set errors
    prevError = error;
    turnPrevError = turnError;

    // sleep
    delay(20);
  }
  return 0;
}


void setPID(bool isTurning, double val1, double val2) {
  resetDriveSensors = true;
  startSlew = true;
  slewPercent = 0;
  slewTurnPercent = 0;

  if (isTurning) {
    turning = true;
    turnKP = val2;
    desiredValue = 0;
    desiredTurnValue = val1;
  } else if (isTurning == false) {
      turning = false;
      kP = val2;
      desiredTurnValue = 0;
      desiredValue = val1;
    }
}

void setPID(bool isTurning, double val1) {
  resetDriveSensors = true;
  startSlew = true;
  slewPercent = 0;
  slewTurnPercent = 0;

  if (isTurning) {
    turning = true;
    desiredValue = 0;
    desiredTurnValue = val1;
  } else if (isTurning == false) {
      turning = false;
      desiredTurnValue = 0;
      desiredValue = val1;
    }
}



///////////////////////////////////////////////////////////////////////////////////
// Motion profiling
///////////////////////////////////
