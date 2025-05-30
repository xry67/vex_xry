#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto2(){
    ChassisControlSTS.MoveToDistance(2000, 1000, "back", true);
    wait(1000, msec);
    ChassisControlSTS.MoveBySmallWheel(2000, 1000, 0, 'n');
}