#pragma once

#include "main.h"


// Main PID functions and variables
void ForwardIntakePD(double goal, float KP,float KI,float KD);

// Automatic ball detector functions and variables
extern bool ballDetectorToggle;
int ballDetector();
