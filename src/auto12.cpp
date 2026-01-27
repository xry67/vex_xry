#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "RobotControl.h"
#include "RemoteControl.h"
#include "AutoTools.h"

void auto12()
{
    ChassisControlSTS.Over_count = 4;
    SimulatedRemoteControl("R1");
    wait(1600, msec);
    ChassisControlSTS.ChassisForwardAngle(0);
    // for (int i = 0; i < 10; i++)
    // {
    //     ChassisControlSTS.ChassisForward(-4);
    //     wait(10, msec);
    // }
    for (int i = 0; i < 40; i++)
    {
        ChassisControlSTS.ChassisLeftControl(8);
        ChassisControlSTS.ChassisRightControl(8);

        wait(10, msec);
    }
    for (int i = 0; i < 15; i++)
    {
        ChassisControlSTS.ChassisForward(6);
        wait(50, msec);
        ChassisControlSTS.ChassisForward(-2);
        wait(50, msec);
    }
    // for (int i = 0; i < 12; i++)
    // {
    //     ChassisControlSTS.ChassisForward(-4);
    //     wait(10, msec);
    // }10

    for (int i = 0; i < 4; i++)
    {

        ChassisControlSTS.ChassisLeftControl(-5);
        ChassisControlSTS.ChassisRightControl(7);
        wait(60, msec);
        ChassisControlSTS.ChassisLeftControl(7);
        ChassisControlSTS.ChassisRightControl(-5);
        wait(60, msec);
    }

    wait(10, msec);
    for (int i = 0; i < 55; i++)
    {
        ChassisControlSTS.ChassisForward(-10);
        wait(10, msec);
    }
    ChassisControlSTS.MoveToDistance(10, 600, 0, 'f');
    ChassisControlSTS.TurnTo(10, -90, 0);
    wait(200, msec);
    ChassisControlSTS.MoveToDistance(10, 1230, -90, 'f');
    ChassisControlSTS.TurnTo(10, -180, 0);

    ChassisControlSTS.MoveToDistance(7, 1010, -180, 'b', 1);
    wait(200, msec);
    while (ChassisControlSTS.IsMoving())
    {
        if (GetDistance_back() > 800)
        {
            ChassisControlSTS.ResetMaxSpeed(2);
            break;
        }
        wait(10, msec);
    }
    ChassisControlSTS.WaitMoveFinish();

    ChassisControlSTS.TurnTo(10, -180 - 45, 0);
    ChassisControlSTS.Move(6, 800, -180 - 45, 'n', 1);
    wait(500, msec);
    while (ChassisControlSTS.IsMoving())
    {
        if (ChassisControlSTS.GetVelocity_pct_mean() <= 3)
        {
            ChassisControlSTS.StopMoving();
            break;
        }
        wait(10, msec);
    }
    ChassisControlSTS.WaitMoveFinish();
    SimulatedRemoteControl("R2");
    wait(3000, msec);

    //  第一个导入桶----------------------------
    ChassisControlSTS.Move(7, 2800, -180 - 45, 'u', 1);
    ChassisControlSTS.WaitPositionArrive(1350);
    ChassisControlSTS.ResetMaxSpeed(2);
    ChassisControlSTS.WaitPositionArrive(1500);
    // 重新对其至-90度
    ChassisControlSTS.ResetMaxSpeed(2);
    ChassisControlSTS.ResetStraightAngle(-180 - 90);
    ChassisControlSTS.WaitPositionArrive(1900);
    ChassisControlSTS.ResetMaxSpeed(5);
    ChassisControlSTS.ReSetToDistance(450, -180 - 90, 'b');
    ChassisControlSTS.WaitMoveFinish();

    digitalE.set(1);
    wait(300, msec);
    // 转至180
    ChassisControlSTS.TurnTo(10, 0, 0);

    SimulatedRemoteControl("R1");

    ChassisControlSTS.Move(5, 900, -360, 'n', true); // 1550

    // 使用距离传感器判断是否到达恒力区域
    while (GetDistance_front() > 372)
    {
        wait(10, msec);
    }
    ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
    ChassisControlSTS.StopMoving();
    wait(100, msec);

    move_intake_time(-360, 2000);

    // 去第二个导入桶----------------------------------------------------------
    ChassisControlSTS.Move(8, 6000, -360 + 50, 'u', 1);
    ChassisControlSTS.WaitPositionArrive(600);
    ChassisControlSTS.ResetStraightAngle(-358);
    digitalE.set(0);
    ChassisControlSTS.WaitPositionArrive(4000);
    while (ChassisControlSTS.IsMoving())
    {
        if (GetDistance_back() < 950)
        {
            break;
        }
        wait(10, msec);
    }
    ChassisControlSTS.ResetMaxSpeed(3);
    ChassisControlSTS.ResetStraightAngle(-90 - 360);
    while (1)
    {
        if (ChassisInertial.rotation() < -45 - 360)
        {
            break;
        }
        wait(10, msec);
    }
    ChassisControlSTS.ReSetToDistance(470, -90 - 360, 'f');

    ChassisControlSTS.WaitMoveFinish();
    wait(300, msec);

    ChassisControlSTS.TurnTo(10, 180, 0);

    // 倒车 打球-===========================-====-=
    Move_u_Long_r(-180 - 360);
    SimulatedRemoteControl("L1");

    // 努力进入对位
    ChassisControlSTS.ChassisForwardAngle(-180 - 360);
    ChassisControlSTS.ChassisForward(-2);
    digitalE.set(1);
    wait(1500, msec);
    intake_and_pushback_new_r(-180 - 360, 2500, 1500);

    ChassisInertial.setRotation(-360 - 180, deg);
    // 去停泊
    ChassisControlSTS.Move(10, 1000, -180 - 30 - 360, 'n', true); // 1550
    ChassisControlSTS.WaitPositionArrive(800);
    ChassisControlSTS.ResetMaxSpeed(5);
    ChassisControlSTS.ReSetToCurveMove_Position(700, 500, 'q');
    ChassisControlSTS.WaitPositionArrive(450);
    ChassisControlSTS.ReSetToEncoderWheel(1500, -360 - 180 - 65, 'n');
    ChassisControlSTS.WaitPositionArrive(600);
    ChassisControlSTS.ResetStraightAngle(-180 - 75 - 360);
    ChassisControlSTS.ResetMaxSpeed(3);
    SimulatedRemoteControl("R1");
    ChassisControlSTS.ReSetToEncoderWheel(1100, -180 - 80 - 360, 'n');

    // while (ChassisControlSTS.IsMoving())
    // {
    //     if (GetDistance_back() > 900)
    //     {
    //         ChassisControlSTS.ResetMaxSpeed(2);
    //         break;
    //     }
    //     wait(10, msec);
    // }
    while (ChassisControlSTS.IsMoving())
    {
        if (std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 5)
        {

            // ChassisControlSTS.StopMoving();
            digitalE.set(1);
            ChassisControlSTS.ReSetToEncoderWheel(110, -180 - 80 - 360, 'u');
            break;
        }
        wait(10, msec);
    }

    ChassisControlSTS.WaitMoveFinish();
    digitalE.set(1);

    ChassisControlSTS.ChassisForwardAngle(-180 - 92 - 360);
    for (int i = 0; i < 200; i++)
    {
        if (i > 150)
        {
            digitalE.set(0);
        }

        if (i > 80)
        {
            ChassisControlSTS.ChassisForwardAngle(-180 - 100 - 360);
        }
        ChassisControlSTS.ChassisForward(6.5);
        wait(10, msec);
    }

    // SimulatedRemoteControl("stop");

    // 去第三个导入桶-----------------------
    ChassisControlSTS.Move(6, 4000, -180 - 170 - 360, 'n', 1);
    ChassisControlSTS.WaitPositionArrive(800);
    ChassisControlSTS.ResetStraightAngle(-180 - 90 - 360);
    ChassisControlSTS.WaitPositionArrive(1500);
    ChassisControlSTS.ResetMaxSpeed(3);
    ChassisControlSTS.ReSetToDistance(450, -180 - 90 - 360, 'f');

    ChassisControlSTS.WaitMoveFinish();
    wait(100, msec);
    digitalE.set(1);
    ChassisControlSTS.TurnTo(10, 180, 0);

    // 吸导入
    SimulatedRemoteControl("R1");

    ChassisControlSTS.Move(5, 900, -180 - 360, 'n', true); // 1550

    // 使用距离传感器判断是否到达恒力区域
    while (GetDistance_front() > 372)
    {
        wait(10, msec);
    }
    ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
    ChassisControlSTS.StopMoving();
    wait(100, msec);

    move_intake_time(-180 - 360, 2500);

    // 去第四个导入桶
    ChassisControlSTS.Move(8, 6000, -180 + 50 - 360, 'u', 1);
    ChassisControlSTS.WaitPositionArrive(600);
    ChassisControlSTS.ResetStraightAngle(-180 + 2 - 360);
    digitalE.set(0);
    ChassisControlSTS.WaitPositionArrive(4000);
    while (ChassisControlSTS.IsMoving())
    {
        if (GetDistance_back() < 900)
        {
            break;
        }
        wait(10, msec);
    }
    ChassisControlSTS.ResetMaxSpeed(3);
    ChassisControlSTS.ResetStraightAngle(-180 - 90 - 360);
    while (1)
    {
        if (ChassisInertial.rotation() < -45 - 180 - 360)
        {
            break;
        }
        wait(10, msec);
    }
    ChassisControlSTS.ReSetToDistance(480, -90 - 180 - 360, 'f');

    ChassisControlSTS.WaitMoveFinish();
    wait(300, msec);

    ChassisControlSTS.TurnTo(10, 0, 0);

    // 倒车 打球-===========================-====-=
    Move_u_Long_r(-360 - 360);
    SimulatedRemoteControl("L1");
    // 努力进入对位
    ChassisControlSTS.ChassisForwardAngle(-360 - 360);
    ChassisControlSTS.ChassisForward(-2);
    digitalE.set(1);
    wait(1500, msec);

    intake_and_pushback_new_r(-360 - 360, 2500, 1500);

    // 去停泊
    ChassisControlSTS.Move(10, 1000, -180 - 30 - 360 - 180, 'n', true); // 1550
    ChassisControlSTS.WaitPositionArrive(800);
    ChassisControlSTS.ResetMaxSpeed(5);
    ChassisControlSTS.ReSetToCurveMove_Position(700, 500, 'q');
    ChassisControlSTS.WaitPositionArrive(450);
    ChassisControlSTS.ReSetToEncoderWheel(1500, -360 - 180 - 65 - 180, 'n');
    ChassisControlSTS.WaitPositionArrive(600);
    ChassisControlSTS.ResetStraightAngle(-180 - 75 - 360 - 180);
    SimulatedRemoteControl("R1");
    ChassisControlSTS.SetStopModCoast();
    ChassisControlSTS.StopMoving();
    ChassisControlSTS.ChassisForwardAngle(-180 - 92 - 360 - 180);
    for (int i = 0; i < 150; i++)
    {
        if (i > 150)
        {
            digitalE.set(0);
        }

        if (i > 80)
        {
            ChassisControlSTS.ChassisForwardAngle(-180 - 100 - 360 - 180);
        }
        ChassisControlSTS.ChassisForward(8);
        wait(10, msec);
    }
}
