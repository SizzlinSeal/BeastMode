#pragma once

#include "main.h"

// Main PID functions and variables
// set PID variables
void setPID(double goalD, double kPD, double kID, double kDD);

// Automatic ball detector functions and variables
extern bool ballDetectorToggle;
int ballDetector();
