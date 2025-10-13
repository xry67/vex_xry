#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "ToolControl.h"

void auto7(){
    digitalH.set(1);

    // ChassisControlSTS.Turn(10, 10, 'r');

    for (int i : {20, 45, 90, 120, 180}){
        ChassisControlSTS.Turn_PPS = {0.025, 0.5, 5};
        ChassisControlSTS.Turn(10, i, 'r');
        ChassisControlSTS.Turn_PPS = {0.025, 0.5, 5};
        ChassisControlSTS.Turn(10, i, 'l');
        wait(1000, msec);
    }

    // for (int i : {200, 600, 1200, 1800}){
    //     ChassisControlSTS.maxinum_speed_acceleration = 0.2;
    //     ChassisControlSTS.Move_PPS = {0.00750, 0.50, 2};
    //     ChassisControlSTS.Move(10, i, 'n');
    //     ChassisControlSTS.Move(10, i, 'u');

    //     wait(500, msec);
    // }
}