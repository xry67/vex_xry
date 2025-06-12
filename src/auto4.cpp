#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto4(){

    ChassisControlSTS.TurnTo(7, 90, 0);
    wait(500, msec);
    ChassisControlSTS.TurnTo(7, 150, 0);
    wait(500, msec);
    ChassisControlSTS.TurnTo(7, 180, 0);
    wait(500, msec);
    ChassisControlSTS.TurnTo(7, 225, 0);
    wait(500, msec);
    ChassisControlSTS.TurnTo(7, 270, 0);
}