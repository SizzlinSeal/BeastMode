#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"

using namespace pros;

// variables
bool opControlActivation = false; // can be disabled and enabled easily

// base control functions (arcade contol)
int opControlMain() {
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


      ////////////////////////////////////////////////////
      // Controller Buttons
      //////////////////////////////


      // if Master button L1 is pressed and nothing else
      if (Master.get_digital(E_CONTROLLER_DIGITAL_L1) && !Master.get_digital(E_CONTROLLER_DIGITAL_L2) && !Master.get_digital(E_CONTROLLER_DIGITAL_R1) && !Master.get_digital(E_CONTROLLER_DIGITAL_R2)) {

        // if ball detector thread is NOT running
        if (!ballDetectorToggle) {
          // spin indexers downward
          tIndexer.move_velocity(-600);
          bIndexer.move_velocity(-600);
          // spin intakes outward
          lIntake.move_velocity(-200);
          rIntake.move_velocity(-200);

          // if it is running
        } else {
          // spin top indexer
          tIndexer.move_velocity(-600);
        }


        // if Master button L2 is pressed and nothing else
      } else if (Master.get_digital(E_CONTROLLER_DIGITAL_L2) && !Master.get_digital(E_CONTROLLER_DIGITAL_L1) && !Master.get_digital(E_CONTROLLER_DIGITAL_R1) && !Master.get_digital(E_CONTROLLER_DIGITAL_R2)) {


        // if ball detector thread is NOT running
        if (!ballDetectorToggle) {
          // spin bottom indexer upwards
          bIndexer.move_velocity(600);
          // stop top indexer
          tIndexer.move(0);
          // spin intakes inward
          lIntake.move_velocity(200);
          rIntake.move_velocity(200);

          // if it is running
        } else {
          tIndexer.move(0);
        }


        // if Master button R1 is pressed and nothing else
      } else if (Master.get_digital(E_CONTROLLER_DIGITAL_R1) && !Master.get_digital(E_CONTROLLER_DIGITAL_L1) && !Master.get_digital(E_CONTROLLER_DIGITAL_L2) && !Master.get_digital(E_CONTROLLER_DIGITAL_R2)) {

        if (!ballDetectorToggle) {
          // spin bottom indexer upwards
          bIndexer.move_velocity(-600);
          tIndexer.move_velocity(-600);
          lIntake.move_velocity(-200);
          rIntake.move_velocity(-200);

        } else {
          tIndexer.move(-600);
        }


        // if bMaster utton R2 is pressed and nothing else
      } else if (Master.get_digital(E_CONTROLLER_DIGITAL_R2) && !Master.get_digital(E_CONTROLLER_DIGITAL_L1) && !Master.get_digital(E_CONTROLLER_DIGITAL_L2) && !Master.get_digital(E_CONTROLLER_DIGITAL_R1)) {

        if (!ballDetectorToggle) {
          // spin indexer upwards
          bIndexer.move_velocity(600);
          tIndexer.move_velocity(600);
          // stop intakes
          lIntake.move(0);
          rIntake.move(0);

        } else {
          tIndexer.move(0);
        }


        // if Master button L2 and Master button R2 are pressing and nothing else
      } else if (Master.get_digital(E_CONTROLLER_DIGITAL_L2) && Master.get_digital(E_CONTROLLER_DIGITAL_R2) && !Master.get_digital(E_CONTROLLER_DIGITAL_L1) && !Master.get_digital(E_CONTROLLER_DIGITAL_R1)) {

        if (!ballDetectorToggle) {
          // cycle
          bIndexer.move(600);
          tIndexer.move(600);
          lIntake.move(200);
          rIntake.move(200);

        } else {
          tIndexer.move(0);
        }

        // if nothing is pressed stop everything
      } else if (!Master.get_digital(E_CONTROLLER_DIGITAL_L1) && !Master.get_digital(E_CONTROLLER_DIGITAL_L2) && !Master.get_digital(E_CONTROLLER_DIGITAL_R1) && !Master.get_digital(E_CONTROLLER_DIGITAL_R2)) {

        if (!ballDetectorToggle) {
          // stop everything
          bIndexer.move(0);
          tIndexer.move(0);
          lIntake.move(0);
          rIntake.move(0);

        } else {
          tIndexer.move(0);
        }
      }


      // partner controls

      // if buttonY was pressed
      if (Partner.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
        ballDetectorToggle = !ballDetectorToggle;
      }

    }
    delay(10); // don't waste resources
  }
  return 0;
}



// create tasks for all the buttons
void opControl() {
  Task operatorControl(opControlMain); // base control
}
