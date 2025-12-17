#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"

void auto4(){
    ChassisControlSTS.Over_count = 1;
    ChassisInertial.setRotation(-180, deg);
    ChassisControlSTS.Move_PPS = {0.0182, 0.5, 2};
    ChassisControlSTS.maxinum_speed_acceleration = 0.5;
    ChassisControlSTS.Move(16, 400, 'n'); // 1550

    ChassisControlSTS.maxinum_speed_acceleration_turn = 0.1;
    ChassisControlSTS.AutoPPSTurnTo(10, -226);

    ChassisControlSTS.CurveMove(10, 780, 800, 'd', true);
    ChassisControlSTS.WaitPositionArrive(750);
    ChassisControlSTS.StopMoving();

    ChassisControlSTS.Move(10, 1000, -180, 'u'); // 1550
    ChassisControlSTS.WaitPositionArrive(750);
    ChassisControlSTS.SetStopModHold();
    ChassisControlSTS.StopMoving();
    ToolMotor1.spin(forward, 0, voltageUnits::volt);
    ToolMotor2.spin(forward, 0, voltageUnits::volt);
    ToolMotor3.spin(forward, 0, voltageUnits::volt);
    // 急刹车
    ChassisControlSTS.ChassisLeftControl(-2);
    ChassisControlSTS.ChassisRightControl(-2);
    wait(100, msec);
    ChassisControlSTS.ChassisLeftControl(0);
    ChassisControlSTS.ChassisRightControl(0);



}