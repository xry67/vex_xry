#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto2()
{
    ChassisControlSTS.Move(9, 1600, 0, 'n', true);
    ChassisControlSTS.WaitPositionArrive(200);
    // 跳跃
    ChassisControlSTS.WaitPositionArrive(400);
    // 开炮
    ChassisControlSTS.WaitPositionArrive(800);
    // 开炮！
    ChassisControlSTS.WaitMoveFinish(); // 等待多线程运动执行完成
}