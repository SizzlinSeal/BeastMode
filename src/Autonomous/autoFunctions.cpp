#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"

using namespace pros;



//////////////////////////////////////////////////////////////////////
// PID thread
///////////////////////////////////

// lateral movement PID values
double kP = 0.09; // P
double kI = 0.0; // I (Unused)
double kD = 0.09; // D

// Turning movement PID values
double turnKP = 0.046; // P
double turnKI = 0.0; // I (Unused)
double turnKD = 0.05; // D

// Variables required by the funciton
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

// main function
int drivePID() {
  // run forever
  while (true) {
    // only run when wanted (set a variable)
    while (enableDrivePID) {

      // reset encoder positions
      if(resetDriveSensors) {
        resetDriveSensors = false;
        LF.tare_position(); // Left front motor
        LB.tare_position(); // Left Back motor
        RF.tare_position(); // Right front motor
        RB.tare_position(); // Right back motor
      }

      // get the position of the motors
      int leftMotorAPosition = LF.get_position(); // Left front motor position
      int leftMotorBPosition = LB.get_position(); // left back motor position
      int rightMotorAPosition = RF.get_position(); // Right front motor position
      int rightMotorBPosition = RB.get_position(); // Right back motor position

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
        // lateral slew
        if (!turning) {
          // set lateral motor power
            slew = (lateralMotorPower / 100) * slewPercent;
            // if slewPercent is less than 100 increase its value
            if(slewPercent < 100) {
               slewPercent += 10;
            }
            // turning slew
        } else if (turning) {
          // set turn motor power
            turnSlew = (turnMotorPower / 100) * slewTurnPercent;
            // if slewTurnPercent is less than 100 increase its value
            if(slewTurnPercent < 100) {
                slewTurnPercent += 10;
            }
        }
      }

      // spin the motors
      LF.move(slew + turnSlew); // Left front motor
      LB.move(slew + turnSlew); // Left back motor
      RF.move(slew - turnSlew); // Right front motor
      RB.move(slew - turnSlew); // Right back motor

      // set errors
      prevError = error;
      turnPrevError = turnError;

      // sleep
      delay(10);
    }
    // sleep
    delay(20);
  }
  return 0;
}

 // setPID function to make using PID easier
void setPID(bool isTurning, double val1, double val2) {
  // set variables
  resetDriveSensors = true; // reset the encoders
  startSlew = true; // start slew
  slewPercent = 0; // reset slew percent
  slewTurnPercent = 0; // reset slew turn persent

  // if turning is desired
  if (isTurning) {
    turning = true; // enable turning mode
    turnKP = val2; // val2 parameter = P value for turning
    desiredValue = 0; // set desired lateral movement to 0
    desiredTurnValue = val1; // set desired turning movement to val1 parameter

    // if lateral movement is desired
  } else {
      turning = false; // set turning mode to false
      kP = val2; // val2 parameter = lateral P value
      desiredTurnValue = 0; // set desider turn value to 0
      desiredValue = val1; // set desired lateral movement to val1 parameter
    }
}

// setPID function to make using PID easier
void setPID(bool isTurning, double val1) {
  // set variables
  resetDriveSensors = true; // reset the encoders
  startSlew = true; // start slew
  slewPercent = 0; // reset slew percent
  slewTurnPercent = 0; // reset slew turn percent

  // if turning movement is desired
  if (isTurning) {
    turning = true; // enable turning mode
    desiredValue = 0; // set desired lateral movement to 0
    desiredTurnValue = val1; // set desired turning movement to val1 parameter

    // if lateral movement is desired
  } else if (!isTurning) {
      turning = false; // set turning mode to false
      desiredTurnValue = 0; // set desider turn value to 0
      desiredValue = val1; // set desired lateral movement to val1 parameter
    }
}




///////////////////////////////////////////////////////////
// Ball detection system
//////////////////////////////////]

// variable for ball detection system
bool ballDetectorToggle = false;

// thread that can be toggled during autonomous and driver control that automatically shuts the intakes whenever a ball is detected
int ballDetector() {
  // firstTime variable to run code every time this function is disabled and enabled
  bool firstTime = true;

    while (true) {
      // only run loop if it is toggled
      if (ballDetectorToggle) {

        // run this code every time the task is reactivated
        if (firstTime) {
          // first time "initialization" of the distance sensors
          lDist.get();
          // move intakes to open position
          lIntake.move_velocity(-200);
          rIntake.move_velocity(-200);
          // set first time to false as this function already ran
          firstTime = false;
          // delay for 100 msec so distance sensors dont detect the intakes
          pros::delay(100);
        }

          // print out distance sensor values for debugging
          pros::lcd::print(0, "Left Dist: %d\n", lDist.get());

          // if the ball is in the range
          if (lDist.get() > 50 && lDist.get() < 170) {
            // intake balls
            lIntake.move_velocity(200);
            rIntake.move_velocity(200);
            bIndexer.move_velocity(600);
            // wait 1 second
            pros::delay(1000);
            // stop bottom indexer and move intakes to open position
            bIndexer.move(0);
            rIntake.move_velocity(-200);
            lIntake.move_velocity(-200);

            // wait 500 msecs to prevent distance sensors detecting intakes
            pros::delay(500);

          } else {
            // if no balls are in the range, keep intakes in open position
            lIntake.move_velocity(-200);
            rIntake.move_velocity(-200);
          }
    } else {
      // set first time variable to true whenever thread gets disabled
      firstTime = true;
    }
    // delay so the cpu does not have a stroke
    pros::delay(20);
  }
  return 0;
}
