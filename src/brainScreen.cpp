#include "main.h"
#include "robot-config.h"

int brainScreen() {
  while (true) {
    // print sensor values
    pros::lcd::print(0, "Left tracking wheel: %d\n", lTrackingWheel.get_value());
    pros::lcd::print(1, "Right tracking wheel: %d\n", rTrackingWheel.get_value());
    pros::lcd::print(2, "Middle tracking wheel: %d\n", mTrackingWheel.get_value());
    pros::delay(100);
  }
  return 0;
}
