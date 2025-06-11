#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto4(){

    ChassisControlSTS.Move(20, 2000, 0.0, 'n', false);
    wait(500, msec);
    ChassisControlSTS.Move(20, 2000, 0.0, 'u', false);

}