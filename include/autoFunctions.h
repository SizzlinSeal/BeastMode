#pragma once

#include "main.h"


// Main PID functions and variables
extern bool enableDrivePID;
int drivePID();
void setPID(bool isTurning, double val1, double val2);
void setPID(bool isTurning, double val1);

// Automatic ball detector functions and variables
extern bool ballDetectorToggle;
int ballDetector();
