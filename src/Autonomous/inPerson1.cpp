#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"


void pid(double goal, double kP, double kD, double slewMaxChange) {
  double error = 0;
  double motorpower = 0;
  double avgEnc = 0;
  double prevError = 0;
  double derivative = 0;
  double prevMotorSpeed = 0;
  int a = 0;
  double run = false;

  while(!run) {

    avgEnc = (lTrackingWheel.get_value() + rTrackingWheel.get_value())/2.0;
    error = goal - avgEnc;
    derivative = error - prevError;
    motorpower = (error * kP + derivative * kD);

    if (motorpower - prevMotorSpeed > slewMaxChange) {
      motorpower = prevMotorSpeed + slewMaxChange;
    }


    LF.move(motorpower);
    RF.move(motorpower);
    LB.move(motorpower);
    RB.move(motorpower);

    prevMotorSpeed = motorpower;
    prevError = error;

    pros::delay(10);

    pros::lcd::print(0, "Iterations: %d\n", a++);

    if(error ==0){
      run = true;
    }

  }

}



void TurnLeft(double degree, float kP) {
    imu1.tare_rotation();
    imu2.tare_rotation();
    imu3.tare_rotation();

    double error = 3;

    int range = 1;
    while (error != 0) {

      error = ((imu1.get_rotation() + imu2.get_rotation() + imu3.get_rotation())/3.0 - degree);

      LF.move(-error * kP);
      RF.move(error * kP);
      LB.move(-error * kP);
      RB.move(error * kP);

      pros::delay(10);
    }
}




// in-person autonomous #1
void inPerson1() {
  // basic P loop
  pid(2800, 0.15, 0.05, 10);
  pros::delay(200);
  TurnLeft(45,0.3);
  //TurnLeft(45,0.4);
}
