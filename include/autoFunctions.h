#pragma once

#include "main.h"

int drivePID();
void setPID(bool isTurning, double val1, double val2);
void setPID(bool isTurning, double val1);
extern bool enableDrivePID;
