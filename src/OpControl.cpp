#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"

using namespace pros;

// variables
bool opControlActivation = false; // can be disabled and enabled easily


// base control functions (arcade contol)
int baseControl() {
  while (true) {
    // only run if opControlActivation is true
    if (opControlActivation) {

      // configure deadzones
      double strafeDeadzone = 2;
      double turnDeadzone = 2;
      double deadzone = 2;


      // get lateral and turning variables from controller joystick positions
      double power = Master.get_analog(ANALOG_LEFT_Y);
      double turn = Master.get_analog(ANALOG_RIGHT_X);
      double strafe = Master.get_analog(ANALOG_LEFT_X);
      double left = power + turn;
      double right = power - turn;

      // raw strafing power for each motor
      double sLF = 0;
      double sLB = 0;
      double sRF = 0;
      double sRB = 0;

      // final power for each motor
      double powerLF = 0;
      double powerLB = 0;
      double powerRF = 0;
      double powerRB = 0;

      // strafe deadzone
      if (std::fabs(strafe) > strafeDeadzone) { // if it is not within the strafe deadzone the strafe
        sLF = strafe;
        sLB = -strafe;
        sRF = -strafe;
        sRB = strafe;

      } else { // else dont deliver strafing power
        sLF = 0;
        sLB = 0;
        sRF = 0;
        sRB = 0;
      }

      // if the turn deadzone is not reached do not deliver turn power
      if (std::fabs(turn) <  turnDeadzone) {
        turn = 0;
      }

      // if the forward/backward deadzone is not reached do not deliver turn power
      if (std::fabs(power) < deadzone) {
        power = 0;
      }

      // set total motor powers
      powerLF = left + sLF;
      powerLB = left + sLB;
      powerRF = right + sRF;
      powerRB = right + sRB;


      // move motors
      // LF motor
      if (std::fabs(powerLF) > 600) {  // if statement so the motor does not always go max speed
        LF.move_velocity((power/powerLF + turn/powerLF + sLF/powerLF)*600); // balance out values
      } else {
        LF.move_velocity(powerLF * 6);  // else move normally
      }

      // LB motor
      if (std::fabs(powerLB) > 600) {
        LB.move_velocity((power/powerLB + turn/powerLB + sLB/powerLB)*600);
      } else {
        LB.move_velocity(powerLB * 6);
      }

      // RF motor
      if (std::fabs(powerRF) > 600) {
        RF.move_velocity((power/powerRF - turn/powerRF + sRF/powerRF)*600);
      } else {
        RF.move_velocity(powerRF * 6);
      }

      // RB motor
      if (std::fabs(powerRB) > 600) {
        RB.move_velocity((power/powerRB - turn/powerRB + sRB/powerRB)*600);
      } else {
        RB.move_velocity(powerRB * 6);
      }


    }
    delay(10); // don't waste resources
  }
  return 0;
}

// left top bumper
int buttonL1() {
  // run code over and over again till the end of time
  while (true) {
    // if button L1 is pressed, do stuff, else do nothing
    if (Master.get_digital(E_CONTROLLER_DIGITAL_L1)) {

      // spin indexers downward
      bIndexer.move_velocity(-600);
      tIndexer.move_velocity(-600);

      // spin intakes outward
      lIntake.move_velocity(-200);
      rIntake.move_velocity(-200);

      // wait until buttonL1 is not pressing, blocking
      waitUntil(!Master.get_digital(E_CONTROLLER_DIGITAL_L1));

      // stop stuff
      bIndexer.move(0);
      tIndexer.move(0);
      lIntake.move(0);
      rIntake.move(0);
    }
    // a delay so that the brain does not commit suicide
    delay(10);
  }

  return 0;
}


// left bottom bumper
int buttonL2() {
  // loop forever
  while(true) {
    // if button L2 pressed
    if (Master.get_digital(E_CONTROLLER_DIGITAL_L2)) {

      // spin bottom indexer upwards
      bIndexer.move_velocity(600);

      // spin intakes inward
      lIntake.move_velocity(200);
      rIntake.move_velocity(200);

      // wait until L2 is released
      waitUntil(!Master.get_digital(E_CONTROLLER_DIGITAL_L2));

      // stop stuff
      bIndexer.move(0);
      lIntake.move(0);
      rIntake.move(0);
    }
    // delay so resources arent wasted
    delay(10);
  }
  return 0;
}


// Right top bumper
int buttonR1() {
  // loop forever
  while(true) {
    // if button R1 pressed
    if (Master.get_digital(E_CONTROLLER_DIGITAL_R1)) {

      // spin bottom indexer upwards
      bIndexer.move_velocity(-600);
      tIndexer.move_velocity(-600);
      lIntake.move_velocity(-200);
      rIntake.move_velocity(-200);

      // wait until L2 is released
      waitUntil(!Master.get_digital(E_CONTROLLER_DIGITAL_R1));

      // stop stuff
      bIndexer.move(0);
      tIndexer.move(0);
      lIntake.move(0);
      rIntake.move(0);
    }
    // delay so resources arent wasted
    delay(10);
  }
  return 0;
}


// Right bottom bumper
int buttonR2() {
  // loop forever
  while(true) {
    // if button R2 pressed
    if (Master.get_digital(E_CONTROLLER_DIGITAL_R2)) {

      // spin bottom indexer upwards
      bIndexer.move_velocity(600);
      tIndexer.move_velocity(600);

      // wait until L2 is released
      waitUntil(!Master.get_digital(E_CONTROLLER_DIGITAL_R2));

      // stop stuff
      bIndexer.move(0);
      tIndexer.move(0);
    }
    // delay so resources arent wasted
    delay(10);
  }
  return 0;
}

// button Y
int buttonY() {
  // loop forever
  while (true) {
    // if button Y is pressed
    if (Master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
      // toggle the ballDetection thread
      ballDetectorToggle = !ballDetectorToggle;
      // wait until the button is released so the toggle doesnt get spammed
      waitUntil(!Master.get_digital(E_CONTROLLER_DIGITAL_Y));
    }
    // delay so resources dont get wasted
    delay(10);
  }
  return 0;
}

// create tasks for all the buttons
void opControl() {
  Task l1(buttonL1); // L1 button
  Task l2(buttonL2); // L2 button
  Task r1(buttonR1); // R1 button
  Task r2(buttonR2); // R2 button
  Task y(buttonY); // Y button
  Task drivetrainControl(baseControl); // base control
}
