#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto3(){
    // while(true){
    //     ChassisControlSTS.Turn(1000, 180, 'l');
    //     wait(1000, msec);
    // }

    ChassisControlSTS.MoveBySmallWheel(2000, 2000, 0.0, 'n', false);
    wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(2000, 2000, 0.0, 'u', false);
}