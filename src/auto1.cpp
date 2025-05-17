#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    // std::cout << "auto1" << std::endl;

    ChassisControlSTS.FollowTarget(500, 6000, Red);
    wait(100, msec);
}