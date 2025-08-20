#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

void auto7(){
    // ChassisControlSTS.Turn(10, 10, 'r');

    for (int i : {5, 10, 20, 45, 90, 120, 180}){
        ChassisControlSTS.Turn_PPS = {0.02, 0.4, 6};
        ChassisControlSTS.Turn(10, i, 'r');
        wait(1000, msec);
    }
}