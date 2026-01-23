#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"
#include <vector>
#include <deque>

bool IsAllInScope(std::deque<double> list, double mininum, double maxinum)
{

  for (double num : list)
  {
    if (num < mininum || num > maxinum)
    {
      return false;
    }
  }
  return true;
}

void auto14()
{
  // 去停泊
  ChassisControlSTS.Move(10, 1000, 30, 'n', true); // 1550
  ChassisControlSTS.WaitPositionArrive(800);
  ChassisControlSTS.ResetMaxSpeed(5);
  ChassisControlSTS.ReSetToCurveMove_Position(700, 500, 'p');
  ChassisControlSTS.WaitPositionArrive(450);
  ChassisControlSTS.ReSetToEncoderWheel(1500, 65, 'n');
  ChassisControlSTS.WaitPositionArrive(600);
  ChassisControlSTS.ResetStraightAngle(75);
  ChassisControlSTS.ResetMaxSpeed(6);
  SimulatedRemoteControl("R1");
  ChassisControlSTS.ReSetToEncoderWheel(1100, 80, 'n');

  while (ChassisControlSTS.IsMoving())
  {
    if (GetDistance_back() > 900)
    {
      ChassisControlSTS.ResetMaxSpeed(2);
      break;
    }
    wait(10, msec);
  }
  while (ChassisControlSTS.IsMoving())
  {
    if (std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 3)
    {

      // ChassisControlSTS.StopMoving();
      digitalE.set(1);
      ChassisControlSTS.ReSetToEncoderWheel(100, 80, 'u');
      break;
    }
    wait(10, msec);
  }

  ChassisControlSTS.WaitMoveFinish();
  digitalE.set(1);

  ChassisControlSTS.ChassisForwardAngle(85);
  for (int i = 0; i < 200; i++)
  {
    if (i > 100)
    {
      digitalE.set(0);
    }

    ChassisControlSTS.ChassisForward(6);
    wait(10, msec);
  }

  ChassisControlSTS.MoveToDistance(8, 765, 100, 'f');
  SimulatedRemoteControl("stop");
  ChassisControlSTS.Move(10, 400, 90, 'u', 1);
  ChassisControlSTS.WaitPositionArrive(300);
  ChassisControlSTS.ResetStraightAngle(0);
}