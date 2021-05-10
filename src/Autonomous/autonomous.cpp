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
    profileController->generatePath({{0_ft, 0_ft, 0_deg},
                                     {5_ft, 5_ft, 90_deg}},
                                    "A");
    profileController->setTarget("A");
    profileController->waitUntilSettled();
}
