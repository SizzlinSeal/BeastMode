#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"

using namespace pros;


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




//////////////////////////////////////////////////////////////
// Async Lateral P with slew
///////////////////////////////////



// structures for task argument
typedef struct {
  double goal;
  double kP;
  double maxMotorChange;
} lateralAsyncPSlewArguments;


// variable that indicates whether the PID is still running
bool asyncLateralPSlewRunning = false;

// main function
void lateralAsyncPSlewThread(double goal, double kP, double maxMotorChange) {

  // set ar

  // initialize variables
  double avgPos = (lTrackingWheel.get_value() + rTrackingWheel.get_value())/2; // the average position of the left and right tracking wheels
  double error = 1; // the error, set to 1 so the while loop runs at leat once
  double lateralMotorPower = error * kP; // voltage to be sent to all motors
  double prevLateralMotorPower = 0; // previous motor voltage
  // reset tracking wheel encoders
  lTrackingWheel.reset();
  rTrackingWheel.reset();
  // set boolean to true if it is needed to know if the P loop is running in autonomous
  asyncLateralPSlewRunning = true;

  // while loop, runs P loop until the error is within the error range
  while (error < 1 || error > 1) {

    // update variables
    avgPos = (lTrackingWheel.get_value() + rTrackingWheel.get_value())/2;
    error = goal - avgPos;
    lateralMotorPower = error * kP;

    if ((lateralMotorPower - prevLateralMotorPower) > maxMotorChange) {
      lateralMotorPower = prevLateralMotorPower + maxMotorChange;
    }

    // move motors
    LF.move(lateralMotorPower);
    LB.move(lateralMotorPower);
    RF.move(lateralMotorPower);
    RB.move(lateralMotorPower);

    // update prevLateralMotorPower
    prevLateralMotorPower = lateralMotorPower;

    // delay so CPU does not go full monke
    pros::delay(10);
  }
  // set boolean to false because the P loop is no longer running
  asyncLateralPSlewRunning = false;
}

/*
// function to easily run the P loop
void asyncLateralPSlew(double goal, double kP, double maxMotorChange) {
  // create the argument as a type
  lateralAsyncPSlewArguments* lateralAsyncPSlewArgument = new lateralAsyncPSlewArguments();

  // set parameters
  lateralAsyncPSlewArgument->goal = goal; // goal value
  lateralAsyncPSlewArgument->kP = kP; // kP value
  lateralAsyncPSlewArgument->maxMotorChange = maxMotorChange; // max motor voltage change
  // run thread
  pros::Task asyncLateralPSlewTask(lateralAsyncPSlewThread, lateralAsyncPSlewArgument);
}*/
