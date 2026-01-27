#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"
#include <vector>
#include <deque>

void auto14()
{
  // 倒车 打球-===========================-====-=
  ChassisInertial.setRotation(-360 - 360, deg);
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
  ChassisControlSTS.ResetMaxSpeed(3);
  SimulatedRemoteControl("R1");
  ChassisControlSTS.ReSetToEncoderWheel(1100, -180 - 80 - 360 - 180, 'n');

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
      ChassisControlSTS.ReSetToEncoderWheel(110, -180 - 80 - 360 - 180, 'u');
      break;
    }
    wait(10, msec);
  }

  ChassisControlSTS.WaitMoveFinish();
  digitalE.set(1);

  ChassisControlSTS.ChassisForwardAngle(-180 - 92 - 360 - 180);
  for (int i = 0; i < 100; i++)
  {
    if (i > 80)
    {
      ChassisControlSTS.ChassisForwardAngle(-180 - 100 - 360 - 180);
    }
    ChassisControlSTS.ChassisForward(6.5);
    wait(10, msec);
  }
}