#pragma once

#include "main.h"

// Main PID functions and variables
// set PID variables
void lateralPID(double goal);
void lateralPID(double goal, double kP);
void lateralPID(double goal, double kP, double kI);
void lateralPID(double goal, double kP, double kI, double kD);
void lateralPID(double goal, double kP, double kI, double kD, double slewOffset);
void lateralPID(double goal, double kP, double kI, double kD, double slewOffset, double slewMaxChange);

void turningPID(double goal);
void turningPID(double goal, double kP);
void turningPID(double goal, double kP, double kI);
void turningPID(double goal, double kP, double kI, double kD);
void turningPID(double goal, double kP, double kI, double kD, double slewOffset);
void turningPID(double goal, double kP, double kI, double kD, double slewOffset, double slewMaxChange);

void strafingPID(double goal);
void strafingPID(double goal, double kP);
void strafingPID(double goal, double kP, double kI);
void strafingPID(double goal, double kP, double kI, double kD);
void strafingPID(double goal, double kP, double kI, double kD, double slewOffset);
void strafingPID(double goal, double kP, double kI, double kD, double slewOffset, double slewMaxChange);


// Automatic ball detector functions and variables
extern bool ballDetectorToggle;
int ballDetector();
