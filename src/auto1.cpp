#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto1()
{
    ToolMotor3.spin(forward, -5, volt);
    wait(20, msec);
    wait(1000, msec);
    ToolMotor3.spin(forward, 0, volt);

    ChassisControlSTS.Move(8, 370, 0, 'n', true);
    ChassisControlSTS.WaitMoveFinish();
    Claw.LiftMove(50);
    Claw.changeMod();
    ChassisControlSTS.Turn_PID(10, -60, 0.8, 0.035, 0.05, 2);

    Claw.LiftMove(60);

    ChassisControlSTS.ChassisForwardAngle(-60);
    for (int i = 0; i < 100; i++)
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
    // 放完了预装

    ChassisControlSTS.Move(7, 500, -60, 'u', true);
    ChassisControlSTS.WaitPositionArrive(260);
    Claw.reversal();
    Claw.changeMod();
    ChassisControlSTS.WaitMoveFinish();
    ChassisControlSTS.Turn_PID(10, -90, 0.6, 0.035, 0.05, 2);
    ToolMotor6.spin(forward, 0, volt);

    // 去找第二个
    ChassisControlSTS.Move(8, 1600, -90, 'n', true);
    Claw.LiftMove(-2000);
    ChassisControlSTS.WaitMoveFinish();
    ChassisControlSTS.Turn_PID(10, -48, 0.8, 0.035, 0.05, 2);

    ToolMotor6.spin(forward, -13, volt);
    ChassisControlSTS.Move(8, 650, -48, 'n', true);
    ChassisControlSTS.WaitPositionArrive(200);
    ChassisControlSTS.ResetMaxSpeed(1.7);
    ChassisControlSTS.WaitMoveFinish();
    ToolMotor6.spin(forward, -2, volt);

    ChassisControlSTS.Turn_PID(10, 70, 0.8, 0.035, 0.05, 2);
    Claw.LiftMove(70);

    ChassisControlSTS.ChassisForwardAngle(70);
    for (int i = 0; i < 80; i++)
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
    wait(500, msec);
    ChassisControlSTS.Move(4, 1600, 90, 'u', true);
    ChassisControlSTS.WaitPositionArrive(400);
    ChassisControlSTS.ReSetToEncoderWheel(500, 180 + 90, 'n');

    ChassisControlSTS.WaitMoveFinish();
    ToolMotor6.spin(forward, 0, volt);
}
