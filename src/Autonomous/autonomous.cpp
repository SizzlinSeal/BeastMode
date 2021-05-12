#include "main.h"
#include "autonomous.h"
#include "robot-config.h"

using namespace okapi;


/////////////////////////////////////////////////////////////////
// Main autonomous functions
///////////////////////////////////////

void pre_auton() {

}



//////////////////////////////////////////////////////////////////
// Test functions
///////////////////////////////////////

void test_sTurn()
{
    Master.clear();
    Master.print(0, 0, "s turn ran");
    profileController->generatePath({{0_ft, 0_ft, 0_deg},
                                     {1_ft,0_ft,0_deg},
                                     {0_ft,3_ft,150_deg},
                                   {0_ft,4_ft,170_deg},
                                 {1_ft,0_ft,0_deg}},
                                    "A");
    Master.clear();
    Master.print(0, 0, "path generated");
    profileController->setTarget("A");
    Master.clear();
    Master.print(0, 0, "target set");
    profileController->waitUntilSettled();
}
void test_ODOM(){
chassis->setState({0_in, 0_in, 0_deg});
// turn 45 degrees and drive approximately 1.4 ft
chassis->driveToPoint({4_ft, 0_ft});
// turn approximately 45 degrees to end up at 90 degrees
chassis->turnToAngle(90_deg);
}
