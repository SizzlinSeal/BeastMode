#include "main.h"
#include "robot-config.h"
#include "autoFunctions.h"

// in-person autonomous #1
void inPerson1() {
  lateralPID(600, 0.2, 0, 0);
  pros::delay(500);
  lateralPID(600, 0.2, 0, 0);
}
