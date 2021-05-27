#pragma once

#include "main.h"

// Main PID functions and variables
// forwardPID functions and variables
void forwardP(double goal, double kP);

// Async P functions and variables
void asyncLateralP(double goal, double kP);
extern bool asyncLateralPRunning;

// Automatic ball detector functions and variables
extern bool ballDetectorToggle;
int ballDetector();
