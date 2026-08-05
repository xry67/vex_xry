#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

// 侧扇区自动，近方
void auto4()
{
    ChassisForward(0, 5, 300);

    ChassisControlSTS.ChassisForwardAngle(0);
    for (int i = 0; i < 100; i++)
    {
        ChassisControlSTS.ChassisForward(-5);
        if (i > 30 && std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
        {
            break;
        }
        wait(10, msec);
    }
    ChassisControlSTS.ChassisStop();
    // (0, 5, 600);

    ChassisControlSTS.Move(8, 370, 0, 'n');
    ChassisControlSTS.Turn_PID(10, -64, 0.8, 0.035, 0.05, 2);

    ChassisControlSTS.ChassisForwardAngle(-64);
    for (int i = 0; i < 60; i++)
    {
        ChassisControlSTS.ChassisForward(4);
        if (i == 30)
        {
            Claw.LiftMove(400);
        }
        if (i > 30 && std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
        {
            break;
        }
        wait(10, msec);
    }
    Claw.LiftMove(-2000);

    Claw.mod_move_flag = true;
    Claw.mod_motor.spin(forward, -8, volt);
    wait(500, msec);
    Claw.mod_move_flag = false;

    ToolMotor6.spin(forward, 1.5, volt);

    ChassisControlSTS.Move(7, 600, -64, 'u', true);
    ChassisControlSTS.WaitPositionArrive(260);
    Claw.claw_mod = 0;
    Claw.changeMod(1);
    ChassisControlSTS.WaitMoveFinish();
    ChassisControlSTS.Turn_PID(10, -30, 0.8, 0.035, 0.05, 2);
    ToolMotor6.spin(forward, 0, volt);

    // 去找第二个

    ChassisControlSTS.Move(8, 1800, -30, 'n', true);
    Claw.LiftMove(-2000);
    ToolMotor6.spin(forward, -13, volt);
    ChassisControlSTS.WaitPositionArrive(1000);
    ChassisControlSTS.ResetMaxSpeed(1.5);
    ChassisControlSTS.WaitMoveFinish();
    ToolMotor6.spin(forward, -2, volt);

    ChassisControlSTS.Turn_PID(10, -170, 0.8, 0.035, 0.05, 2);
    Claw.LiftMove(2000);

    ChassisControlSTS.ChassisForwardAngle(-170);
    for (int i = 0; i < 80; i++)
    {
        ChassisControlSTS.ChassisForward(4);
        if (i > 30 && std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
        {
            break;
        }
        wait(10, msec);
    }
    Claw.LiftMove(500);
    ToolMotor6.spin(forward, 2, volt);
    ChassisControlSTS.Move(4, 600, 'u', true);
    ChassisControlSTS.WaitPositionArrive(400);
    ChassisControlSTS.ResetStraightAngle(-90);
    ChassisControlSTS.WaitMoveFinish();
    ToolMotor6.spin(forward, 0, volt);
}
