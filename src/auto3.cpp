#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

// 定位轮测试
void auto3(){
    // while(true){
    //     ChassisControlSTS.Turn(1000, 180, 'l');
    //     wait(1000, msec);
    // }

    ChassisControlSTS.MoveBySmallWheel(8, 2000, 0.0, 'n', false);
    // wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 2000, 0.0, 'u', false);
    // wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 1500, 0.0, 'n', false);
    // wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 1500, 0.0, 'u', false);
       //  wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 1000, 0.0, 'n', false);
    // wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 1000, 0.0, 'u', false);
       //  wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 500, 0.0, 'n', false);
    // wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 500, 0.0, 'u', false);
       //      wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 200, 0.0, 'n', false);
    // wait(500, msec);
    ChassisControlSTS.MoveBySmallWheel(8, 200, 0.0, 'u', false);
}