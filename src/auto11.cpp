#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto11(){
    // 倒车钩子
    ChassisInertial.setRotation(-180, deg);
    ChassisControlSTS.Move(5, 1000, -140, 'n', true); // 1550 
    ChassisControlSTS.WaitPositionArrive(800);
    ChassisControlSTS.ResetStraightAngle(-180);
    ChassisControlSTS.WaitMoveFinish();
    ChassisControlSTS.TurnTo(10, 180, false);

    ChassisControlSTS.Move(12, 1700, -180, 'u', true); // 直线钩
    ChassisControlSTS.WaitPositionArrive(1500);
    ChassisControlSTS.SetStopModHold();

    ChassisControlSTS.StopMoving();
    // 急刹车
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(100, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);

    // 正钩
    ChassisControlSTS.CurveMove(3, 295, 1100, 'q', true);   // 曲线
    ChassisControlSTS.WaitPositionArrive(1070);
    ChassisControlSTS.StopMoving();   // 提前结束，不需要精准

    ChassisControlSTS.Move(12, 1500, -360, 'n', true); // 直线钩
    ChassisControlSTS.WaitPositionArrive(1300);
    ChassisControlSTS.SetStopModHold();

    ChassisControlSTS.StopMoving();
    // 急刹车
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(100, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);
}