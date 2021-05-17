#pragma once

#include "main.h"


////////////////////////////////////////////////////
// Okapi constructors
////////////////////////

extern std::shared_ptr<okapi::ChassisController> okapiChassis;
extern std::shared_ptr<okapi::AsyncMotionProfileController> profileController;
extern std::shared_ptr<okapi::OdomChassisController> chassis;
////////////////////////////////////////////////////////
// pros constructors
////////////////////////

// controllers
extern pros::Controller Master; // master controller

// drivetrain motors
extern pros::Motor LF; // Left front motor
extern pros::Motor LB; // Left back motor
extern pros::Motor RF; // Right front motor
extern pros::Motor RB; // Right back motor

// intakes
extern pros::Motor lIntake; // Left intake
extern pros::Motor rIntake; // Right intake

// indexers
extern pros::Motor bIndexer; // Bottom indexer
extern pros::Motor tIndexer; // Top indexer

// tracking wheels
extern pros::ADIEncoder lTrackingWheel;
extern pros::ADIEncoder rTrackingWheel;
extern pros::ADIEncoder mTrackingWheel;

// sensors
extern pros::Distance lDist; // left distance sensor
extern pros::Distance rDist; // Right distance sensor
