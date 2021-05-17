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

void ForwardIntakePD(double goal, float KP,float KI,float KD){ // revert for skills

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

      lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

      LB.move(lateralmotorpower);
      LF.move(lateralmotorpower);
      RB.move(lateralmotorpower);
      RF.move(lateralmotorpower);

      prevError = error;
      pros::delay(10);
      }


  DriveBreak();
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

          // print out distance sensor values for debugging
          pros::lcd::print(0, "Left Dist: %d\n", lDist.get());

          // if the ball is in the range
          if (lDist.get() > 50 && lDist.get() < 170 && !firstTime) {
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
