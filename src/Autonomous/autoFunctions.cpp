#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"

using namespace pros;



//////////////////////////////////////////////////////////////////////
// PID fuunctions
///////////////////////////////////

void DriveBreak() {
  LF.move(0);
  LB.move(0);
  RF.move(0);
  RB.move(0);
}

void resetEnc() {
  lTrackingWheel.reset();
  rTrackingWheel.reset();
  mTrackingWheel.reset();
}

double avgEnc() {
  return (lTrackingWheel.get_value() + rTrackingWheel.get_value())/2;
}





///////////////////////////////////////////////////////////////////////
// Lateral PID
/////////////////////////////////


// lateralPID variables, set to default values in future
double lateralGoal = 0; // goal for lateral movement
double lateralP = 0; // P for lateral movement
double lateralI = 0; // I for lateral movement
double lateralD = 0; // D for lateral movement
double lateralSlewOffset = 0; // slew offset for lateral movement
double lateralSlewMaxChange = 0; // max change before slew kicks in of lateral movement

// main thread
void lateralPIDThread() {

  // reset encoders
  resetEnc();

  // error
  double error = lateralGoal - avgEnc();
  // previous error
  double prevError = 0;
  // derivative
  double derivative;
  // total error
  double totalError;
  // motor power
  double motorPower;
  // previous motor power
  double prevMotorPower = 0;
  // difference in motor power
  double powerChange = 0;

  // while the target has not been reached
  while (error != 0) {

    error = lateralGoal - avgEnc(); // update current error
    derivative = error - prevError; // update current derivative
    totalError += error; // update total error

    motorPower = (error * lateralP + totalError * lateralI + derivative * lateralD); // calculate motor power

    powerChange = motorPower - prevMotorPower; // calculate the derivative of change in motor power

    // if the motor power exceeds slew limits, reduce the power
    if (powerChange > lateralSlewMaxChange) {
      motorPower = prevMotorPower + lateralSlewOffset; // calculate new motor power
    }

    // spin the motors
    LB.move(motorPower); // Left back motor
    LF.move(motorPower); // Left front motor
    RB.move(motorPower); // Right back motor
    RF.move(motorPower); // Right front motor

    prevError = error; // update last error
    motorPower = prevMotorPower; // update last motor power

    pros::delay(10); // delay so the CPU does not go monke mode

  }
  DriveBreak(); // stop drivetrain
}


void lateralPID(double goal) {
  lateralGoal = goal;

  pros::Task lateralPIDTask(lateralPIDThread);
}

void lateralPID(double goal, double kP) {
  lateralGoal = goal;
  lateralP = kP;

  pros::Task lateralPIDTask(lateralPIDThread);
}

void lateralPID(double goal, double kP, double kI) {
  lateralGoal = goal;
  if (kP != 0) {
    lateralP = kP;
  }
  lateralI = kI;

  pros::Task lateralPIDTask(lateralPIDThread);
}

void lateralPID(double goal, double kP, double kI, double kD) {
  lateralGoal = goal;

  if (kP != 0) {
    lateralP = kP;
  }

  if (kI != 0) {
    lateralI = kI;
  }

  lateralD = kD;

  pros::Task lateralPIDTask(lateralPIDThread);
}

void lateralPID(double goal, double kP, double kI, double kD, double slewOffset) {
  lateralGoal = goal;

  if (kP != 0) {
    lateralP = kP;
  }

  if (kI != 0) {
    lateralI = kI;
  }

  if (kD != 0) {
    lateralD = kD;
  }

  lateralSlewOffset = slewOffset;

  pros::Task lateralPIDTask(lateralPIDThread);
}

void lateralPID(double goal, double kP, double kI, double kD, double slewOffset, double slewMaxChange) {
  lateralGoal = goal;

  if (kP != 0) {
    lateralP = kP;
  }

  if (kI != 0) {
    lateralI = kI;
  }

  if (kD != 0) {
    lateralD = kD;
  }

  if (slewOffset != 0) {
    lateralSlewOffset = slewOffset;
  }

  lateralSlewMaxChange = slewMaxChange;

  pros::Task lateralPIDTask(lateralPIDThread);
}





























///////////////////////////////////////////////////////////////////////
// Turning PID
/////////////////////////////////


// Turning PID variables, set to default values in future
double turningGoal = 0; // goal for turning movement
double turningP = 0; // P for turning movement
double turningI = 0; // I for turning movement
double turningD = 0; // D for turning movement
double turningSlewOffset = 0; // slew offset for turning movement
double turningSlewMaxChange = 0; // max change before slew kicks in for turning movement

// main thread
void turningPIDThread() {

  // reset encoders
  resetEnc();

  // turn difference
  double turnDifference = lTrackingWheel.get_value() - rTrackingWheel.get_value();
  // error
  double error = turningGoal - turnDifference;
  // previous error
  double prevError = 0;
  // derivative
  double derivative;
  // total error
  double totalError;
  // motor power
  double motorPower;
  // previous motor power
  double prevMotorPower = 0;
  // difference in motor power
  double powerChange = 0;

  // while the target has not been reached
  while (error != 0) {



    error = turningGoal - turnDifference; // update current error
    derivative = error - prevError; // update current derivative
    totalError += error; // update total error

    motorPower = (error * turningP + totalError * turningI + derivative * turningD); // calculate motor power

    powerChange = motorPower - prevMotorPower; // calculate the derivative of change in motor power

    // if the motor power exceeds slew limits, reduce the power
    if (powerChange > turningSlewMaxChange) {
      motorPower = prevMotorPower + turningSlewOffset; // calculate new motor power
    }

    // spin the motors
    LB.move(motorPower); // Left back motor
    LF.move(motorPower); // Left front motor
    RB.move(-motorPower); // Right back motor
    RF.move(-motorPower); // Right front motor

    prevError = error; // update last error
    motorPower = prevMotorPower; // update last motor power

    pros::delay(10); // delay so the CPU does not go monke mode

  }
  DriveBreak(); // stop drivetrain
}


void turningPID(double goal) {
  turningGoal = goal;

  pros::Task turningPIDTask(turningPIDThread);
}

void turningPID(double goal, double kP) {
  turningGoal = goal;
  turningP = kP;

  pros::Task turningPIDTask(turningPIDThread);
}

void turningPID(double goal, double kP, double kI) {
  turningGoal = goal;
  if (kP != 0) {
    turningP = kP;
  }
  turningI = kI;

  pros::Task turningPIDTask(turningPIDThread);
}

void turningPID(double goal, double kP, double kI, double kD) {
  turningGoal = goal;

  if (kP != 0) {
    turningP = kP;
  }

  if (kI != 0) {
    turningI = kI;
  }

  turningD = kD;

  pros::Task turningPIDTask(turningPIDThread);
}

void turningPID(double goal, double kP, double kI, double kD, double slewOffset) {
  turningGoal = goal;

  if (kP != 0) {
    turningP = kP;
  }

  if (kI != 0) {
    turningI = kI;
  }

  if (kD != 0) {
    turningD = kD;
  }

  turningSlewOffset = slewOffset;

  pros::Task turningPIDTask(turningPIDThread);
}

void turningPID(double goal, double kP, double kI, double kD, double slewOffset, double slewMaxChange) {
  turningGoal = goal;

  if (kP != 0) {
    turningP = kP;
  }

  if (kI != 0) {
    turningI = kI;
  }

  if (kD != 0) {
    turningD = kD;
  }

  if (slewOffset != 0) {
    turningSlewOffset = slewOffset;
  }

  turningSlewMaxChange = slewMaxChange;

  pros::Task turningPIDTask(turningPIDThread);
}
































//////////////////////////////////////////////////////////////////////
// Strafing PID
/////////////////////////////////


// Strafing PID variables, set to default values in future
double strafingGoal = 0; // goal for strafing movement
double strafingP = 0; // P for strafing movement
double strafingI = 0; // I for strafing movement
double strafingD = 0; // D for strafing movement
double strafingSlewOffset = 0; // slew offset for strafing movement
double strafingSlewMaxChange = 0; // max change before slew kicks in for strafing movement

// main thread
void strafingPIDThread() {

  // reset encoders
  resetEnc();

  // turn difference
  double mTrackingWheelPos = mTrackingWheel.get_value();
  // error
  double error = strafingGoal - mTrackingWheelPos;
  // previous error
  double prevError = 0;
  // derivative
  double derivative;
  // total error
  double totalError;
  // motor power
  double motorPower;
  // previous motor power
  double prevMotorPower = 0;
  // difference in motor power
  double powerChange = 0;

  // while the target has not been reached
  while (error != 0) {



    error = strafingGoal - mTrackingWheelPos; // update current error
    derivative = error - prevError; // update current derivative
    totalError += error; // update total error

    motorPower = (error * strafingP + totalError * strafingI + derivative * strafingD); // calculate motor power

    powerChange = motorPower - prevMotorPower; // calculate the derivative of change in motor power

    // if the motor power exceeds slew limits, reduce the power
    if (powerChange > strafingSlewMaxChange) {
      motorPower = prevMotorPower + strafingSlewOffset; // calculate new motor power
    }

    // spin the motors
    LB.move(motorPower); // Left back motor
    LF.move(-motorPower); // Left front motor
    RB.move(-motorPower); // Right back motor
    RF.move(motorPower); // Right front motor

    prevError = error; // update last error
    motorPower = prevMotorPower; // update last motor power

    pros::delay(10); // delay so the CPU does not go monke mode

  }
  DriveBreak(); // stop drivetrain
}


void strafingPID(double goal) {
  strafingGoal = goal;

  pros::Task strafingPIDTask(strafingPIDThread);
}

void strafingPID(double goal, double kP) {
  strafingGoal = goal;
  strafingP = kP;

  pros::Task strafingPIDTask(strafingPIDThread);
}

void strafingPID(double goal, double kP, double kI) {
  strafingGoal = goal;
  if (kP != 0) {
    strafingP = kP;
  }
  strafingI = kI;

  pros::Task strafingPIDTask(strafingPIDThread);
}

void strafingPID(double goal, double kP, double kI, double kD) {
  strafingGoal = goal;

  if (kP != 0) {
    strafingP = kP;
  }

  if (kI != 0) {
    strafingI = kI;
  }

  strafingD = kD;

  pros::Task strafingPIDTask(strafingPIDThread);
}

void strafingPID(double goal, double kP, double kI, double kD, double slewOffset) {
  strafingGoal = goal;

  if (kP != 0) {
    strafingP = kP;
  }

  if (kI != 0) {
    strafingI = kI;
  }

  if (kD != 0) {
    strafingD = kD;
  }

  strafingSlewOffset = slewOffset;

  pros::Task strafingPIDTask(strafingPIDThread);
}

void strafingPID(double goal, double kP, double kI, double kD, double slewOffset, double slewMaxChange) {
  strafingGoal = goal;

  if (kP != 0) {
    strafingP = kP;
  }

  if (kI != 0) {
    strafingI = kI;
  }

  if (kD != 0) {
    strafingD = kD;
  }

  if (slewOffset != 0) {
    strafingSlewOffset = slewOffset;
  }

  strafingSlewMaxChange = slewMaxChange;

  pros::Task strafingPIDTask(strafingPIDThread);
}








///////////////////////////////////////////////////////////
// Ball detection system
//////////////////////////////////

// variable for ball detection system
bool ballDetectorToggle = false;

// thread that can be toggled during autonomous and driver control that automatically shuts the intakes whenever a ball is detected
int ballDetector() {
  // firstTime variable to run code every time this function is disabled and enabled
  bool firstTime = true;

    while (true) {
      // only run loop if it is toggled
      if (ballDetectorToggle) {

          // print out distance sensor values for debugging
          //pros::lcd::print(0, "Left Dist: %d\n", lDist.get());

          // if the ball is in the range
          if ((lDist.get() > 50 && lDist.get() < 170 && !firstTime) || (rDist.get() > 50 && rDist.get() < 170 && !firstTime)) {
            // intake balls
            lIntake.move_velocity(200);
            rIntake.move_velocity(200);
            bIndexer.move_velocity(600);
            // wait 1 second
            pros::delay(1000); // should be replaced by optical sensor, waitUntil its detected in future
            // stop bottom indexer and move intakes to open position
            bIndexer.move(0);
            rIntake.move_velocity(-200);
            lIntake.move_velocity(-200);

            // wait until intakes are in the out position
            waitUntil(lIntake.get_efficiency() < 2 && rIntake.get_efficiency() < 2);

          } else {
            // if no balls are in the range, keep intakes in open position
            lIntake.move_velocity(-200);
            rIntake.move_velocity(-200);
            firstTime = false;
          }
    } else {
      // set first time variable to true whenever thread gets disabled
      firstTime = true;
    }
    // delay so the cpu does not have a stroke
    pros::delay(10);
  }
  return 0;
}
