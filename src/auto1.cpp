#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    
    ChassisInertial.resetRotation();  // 陀螺仪初始化
    ChassisControlSTS.MoveBySmallWheel(2000, 1000, 0.0, 'n');
    ChassisControlSTS.Turn(800, 90, 'l');
    ChassisControlSTS.MoveBySmallWheel(2000, 1000, 0.0, 'n');
    ChassisControlSTS.Turn(800, 90, 'r');
    ChassisControlSTS.MoveBySmallWheel(2000, 1000, 0.0, 'n');
    ChassisControlSTS.Turn(800, 30, 'l');
    ChassisControlSTS.MoveBySmallWheel(2000, 500, 0.0, 'n');

    // ChassisControlSTS.MoveBySmallWheel(2000, 2000, 0.0, 'n');
    // ChassisControlSTS.Turn(800, 130, 'r');
    // ChassisControlSTS.MoveBySmallWheel(2000, 1000, 0.0, 'n');
    // ChassisControlSTS.Turn(800, 30, 'l');
    // ChassisControlSTS.MoveBySmallWheel(2000, 1000, 0.0, 'u');
    wait(100, msec);
}