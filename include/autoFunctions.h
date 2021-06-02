#pragma once

#include "main.h"

// Async P with slew functions and variables
void lateralAsyncPSlewThread(double goal, double kP, double maxMotorChange);
extern bool asyncLateralPSlewRunning;

// Automatic ball detector functions and variables
extern bool ballDetectorToggle;
int ballDetector();
