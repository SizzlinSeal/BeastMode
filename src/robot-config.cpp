#include "main.h"
#include "robot-config.h"

using namespace okapi;

/////////////////////////////////////////////////////////////
// okapi constructors
////////////////////////////

// define chasis
std::shared_ptr<ChassisController> okapiChassis = ChassisControllerBuilder()
                        .withMotors({11, 16}, {19, 17})
                        .withGains(
                          {0.1, 0, 0.0001}, // Distance controller gains
                          {0.1, 0, 0.0001}, // Turn controller gains
                          {0.1, 0, 0.0001})  // Angle controller gains (helps drive straight)
                        .withDimensions (okapi::AbstractMotor::gearset::blue, {{4_in, 12_in}, okapi::imev5BlueTPR / 0.6}) // redo this, this is wrong
                        .build(); // add encoders when i get the chance

std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
            3.0, //Maximum linear velocity of the chassis
            1.0, //Maximum linear acceleration
            8.0  //Maximum linear jerk of the chassis
        })
        .withOutput(okapiChassis) // output values to chassiscontroller
        .buildMotionProfileController();
        std::shared_ptr<OdomChassisController> chassis =
          ChassisControllerBuilder()
            .withMotors({11, 16}, {19, 17}) // left motor is 1, right motor is 2 (reversed)
            // green gearset, 4 inch wheel diameter, 11.5 inch wheel track
            .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
            // left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
            .withSensors(ADIEncoder{'A', 'B'}, ADIEncoder{'C', 'D', true})
            // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
            .withOdometry({{2.75_in, 12_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
            .buildOdometry();

/////////////////////////////////////////////////////////
// Pros constructors
//////////////////////////////////

// controllers
pros::Controller Master (pros::E_CONTROLLER_MASTER); // master controller

// drivetrain motors
pros::Motor LF (11, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // Left Front Port 11
pros::Motor LB (16, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // Left Back Port 16
pros::Motor RF (19, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // Right Front Port 19
pros::Motor RB (9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // Right Back Port 17

// intakes
pros::Motor lIntake (20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES); // Left intake Port 20
pros::Motor rIntake (18, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES); // right intake port 18

// indexers
pros::Motor bIndexer (1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // bottom indexer Port 1
pros::Motor tIndexer (2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // top indexer Port 2

// tracking wheels
pros::ADIEncoder lTrackingWheel('C', 'D', false); // Left tracking wheel 
pros::ADIEncoder rTrackingWheel('A', 'B', true);
pros::ADIEncoder mTrackingWheel('E', 'F', false);

// sensors
pros::Distance lDist (6); // left intake distance sensor Port 5
pros::Distance rDist (5); // Right intake distance sensor Port 5
