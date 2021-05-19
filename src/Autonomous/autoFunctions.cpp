#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"

using namespace pros;



//////////////////////////////////////////////////////////////////////
// PID thread
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

// variables
double goal = 0.0;
float kP = 0.0;
float kI = 0.0;
float kD = 0.0;



int ForwardIntakePD() { // revert for skills

  resetEnc(); // resets the Enc
  //Error//
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0;
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;



  while (error > 3) {
      /*if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0 ) {
      IntakeL.stop();
      IntakeR.stop();
      } else {*/
        error = goal - avgEnc();
      derivative = error - prevError;
      totalerror += error;

      lateralmotorpower = (error * kP + totalerror * kI + derivative * kD);

      LB.move(lateralmotorpower);
      LF.move(lateralmotorpower);
      RB.move(lateralmotorpower);
      RF.move(lateralmotorpower);

      prevError = error;
      pros::delay(10);
      }

  DriveBreak();
  return 0;
}

// set PID variables
void setPID(double goalD, double kPD, double kID, double kDD) {
  pros::Task pidD(ForwardIntakePD);
  goal = goalD;
  kP = kPD;
  kI = kID;
  kD = kDD;
}



///////////////////////////////////////////////////////////
// Ball detection system
//////////////////////////////////

// variable for ball detection system
bool ballDetectorToggle = false;

// thread that can be toggled during autonomous and driver control that automatically shuts the intakes whenever a ball is detected
int ballDetector() {

  bool firstTime = true;

    while (true) {

      // set variables to save time
      double lDistance = lDist.get();
      double rDistance = rDist.get();

      // print distance sensor values to save time
      pros::lcd::print(3, "Left distance sensor: %d\n", lDistance);
      pros::lcd::print(4, "Right distance sensor: %d\n", rDistance);

      if (firstTime) {
        lIntake.move_velocity(-200);
        rIntake.move_velocity(-200);
        waitUntil(lIntake.get_efficiency() < 2 && rIntake.get_efficiency() < 2);
        lDistance = lDist.get();
        rDistance = rDist.get();
        firstTime = false;
      }

      // only run loop if it is toggled
      if (ballDetectorToggle) {

          // if the ball is in the range
          if ((lDistance > 50 && lDistance < 170) || (rDistance > 50 && rDistance < 170)) {
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
          }
    } else {
      firstTime = true; // set the first time to true
    }

    // delay so the cpu does not have a stroke
    pros::delay(10);
  }
  return 0;
}
