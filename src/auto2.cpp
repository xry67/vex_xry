#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto2()
{
    ToolMotor3.spin(forward, -5, volt);
    wait(20, msec);
    wait(1000, msec);
    ToolMotor3.spin(forward, 0, volt);

    ChassisControlSTS.Move(6, 1100, 0, 'n');
    Claw.LiftMove(50);
    Claw.changeMod();
    wait(500, msec);
    ChassisControlSTS.Turn_PID(6, -90, 0.8, 0.035, 0.05, 2);
    Claw.LiftMove(60);

    ChassisControlSTS.ChassisForwardAngle(-90);
    for (int i = 0; i < 60; i++)
    {
        ChassisControlSTS.ChassisForward(4);
        if (i > 30 && std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
        {
            break;
        }
        wait(10, msec);
    }
    ToolMotor6.spin(forward, 5, volt);
    wait(1000, msec);
    ToolMotor6.spin(forward, 0, volt);

    ChassisControlSTS.Move(7, 500, -90, 'u', true);
    ChassisControlSTS.WaitPositionArrive(260);

    ChassisControlSTS.WaitMoveFinish();
    ChassisControlSTS.Turn_PID(8, -37, 0.8, 0.035, 0.05, 2);
    Claw.reversal();
    Claw.changeMod();
    // 去找第二个
    Claw.LiftMove(44);
    ChassisControlSTS.Move(6, 1700, -37, 'n', true);
    ChassisControlSTS.WaitPositionArrive(600);
    ToolMotor6.spin(forward, -13, volt);
    ChassisControlSTS.ResetMaxSpeed(2);
    ChassisControlSTS.WaitMoveFinish();
    ToolMotor6.spin(forward, -2, volt);

    ChassisControlSTS.Turn_PID(6, -175, 0.8, 0.035, 0.05, 2);
    Claw.LiftMove(70);

    ChassisControlSTS.ChassisForwardAngle(-175);
    for (int i = 0; i < 120; i++)
    {
        ChassisControlSTS.ChassisForward(4);
        if (i > 30 && std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
        {
            break;
        }
        wait(10, msec);
    }
    Claw.LiftMove(50);
    ToolMotor6.spin(forward, 2, volt);
    ChassisControlSTS.Move(4, 600, 'u', true);
    ChassisControlSTS.WaitPositionArrive(400);
    ChassisControlSTS.ResetStraightAngle(-90);
    ChassisControlSTS.WaitMoveFinish();
    ToolMotor6.spin(forward, 0, volt);
}