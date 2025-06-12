#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto4(){

    ChassisControlSTS.TurnTo(10, 90, 0);
    // wait(500, msec);
    ChassisControlSTS.TurnTo(10, 150, 0);
    // wait(500, msec);
    ChassisControlSTS.TurnTo(10, 180, 0);
    // wait(500, msec);
    ChassisControlSTS.TurnTo(10, 225, 0);
    // wait(500, msec);
    ChassisControlSTS.TurnTo(10, 270, 0);
}