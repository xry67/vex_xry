#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto1(){
    
    ChassisInertial.resetRotation();  // 陀螺仪初始化
    ChassisControlSTS.MoveBySmallWheel(1000, 2000, 0.0, 'n');
    ChassisControlSTS.Turn(8000, 90, 'l');
    ChassisControlSTS.MoveBySmallWheel(1000, 1000, 0.0, 'n');
    ChassisControlSTS.Turn(8000, 45, 'r');
    ChassisControlSTS.MoveBySmallWheel(1000, 1000, 0.0, 'n');
    ChassisControlSTS.Turn(8000, 45, 'r');
    ChassisControlSTS.MoveBySmallWheel(1000, 1000, 0.0, 'n');
    ChassisControlSTS.WaitMoveFinish();

    // ChassisControlSTS.ChassisStoppingCoast();
    wait(2000, msec);
}