#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"


void pid(double goal, double kP) {
  double error = 1;
  double motorpower = 0;
  double avgEnc = 0;

  while(error != 0) {

    avgEnc = (lTrackingWheel.get_value() + rTrackingWheel.get_value())/2;
    error = goal - avgEnc;
    motorpower = error * kP;

    LF.move(motorpower);
    RF.move(motorpower);
    LB.move(motorpower);
    RB.move(motorpower);

    pros::delay(10);
  }
}






// in-person autonomous #1
void inPerson1() {
  // basic P loop
  pid(500, 0.09);

}
