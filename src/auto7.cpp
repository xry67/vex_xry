#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

void auto7()
{

    // 前进吸球
    ChassisControlSTS.Move(10, 1300, 'n', true);
    SimulatedRemoteControl("R1");
    ChassisControlSTS.WaitPositionArrive(400);
    ChassisControlSTS.ResetMaxSpeed(3);
    ChassisControlSTS.WaitPositionArrive(850);
    digitalH.set(1);
    ChassisControlSTS.WaitMoveFinish();
    digitalH.set(0);
    // 转前进吐球
    ChassisControlSTS.TurnTo(16, -45, 0);
    ChassisControlSTS.Move(7, 700, 'n', true);
    ChassisControlSTS.WaitPositionArrive(500);
    SimulatedRemoteControl("R2");
    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.WaitPositionArrive(600);
    ChassisControlSTS.StopMoving();
    ChassisControlSTS.ChassisRightControl(3);
    ChassisControlSTS.ChassisLeftControl(3);
    wait(2000, msec);
    SimulatedRemoteControl("R1");
    ChassisControlSTS.SetStopModHold();

    // 倒退转向
    ChassisControlSTS.Move(7, 800, 'u');
    ChassisControlSTS.TurnTo(16, -90, 0);
    // 前进吸
    ChassisControlSTS.Move(10, 2300, 'n', true);
    ChassisControlSTS.WaitPositionArrive(1000);
    ChassisControlSTS.ResetMaxSpeed(3);
    ChassisControlSTS.WaitMoveFinish();
    // 转向中桥吐球
    wait(200, msec);
    ChassisControlSTS.TurnTo(16, -127, 0);
    ChassisControlSTS.Move(7, 600, 'u', true);
    ChassisControlSTS.WaitPositionArrive(550);
    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.StopMoving();
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(200, msec);
    SimulatedRemoteControl("L2");
    
    wait(2500, msec);
}