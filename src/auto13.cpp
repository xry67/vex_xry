#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"
#include <vector>

// 半场长度：1790 mm
// 管道半长 280 mm
double w = 340; // 宽度
double h = 450; //  长度
double f_h = h / 2 - 100;
double b_h = h / 2 - 70;
double l_w = w / 2 - 30;
double r_w = w / 2 - 30 - 95;

double GetMoveLen(double Right_d, double Front_d)
{
  return sqrt(2) * (1790 - Right_d);
}

void GoToMiddleBridge(double angle)
{

  ChassisControlSTS.Over_count = 10;
  ChassisControlSTS.TurnTo(10, angle, 0);
  ChassisControlSTS.Over_count = 5;
  // 735
  // 1208

  double Right_d = GetDistance_right() + r_w;
  double Front_d = GetDistance_front() + f_h;

  ChassisControlSTS.MoveToDistance(5, Right_d, angle, 'f');

  ChassisControlSTS.TurnTo(10, angle + 45, 0);

  digitalB.set(0);
  digitalC.set(0);

  ChassisControlSTS.Move(10, GetMoveLen(Right_d, Front_d) * 735 / 1208 * 1.5, angle + 45, 'u', 1);
  ChassisControlSTS.WaitPositionArrive(GetMoveLen(Right_d, Front_d) * 735 / 1208 * 0.7);
  ChassisControlSTS.ResetMaxSpeed(3);
  ChassisControlSTS.ReSetToEncoderWheel(1500, angle + 45, 'u');
  while (ChassisControlSTS.IsMoving())
  {
    if (ChassisControlSTS.GetVelocity_pct_mean() <= 3)
    {
      ChassisControlSTS.StopMoving();
      break;
    }
    wait(10, msec);
  }
  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.WaitMoveFinish();
}

void FromGoToMiddleBridge_To_Intake(double angle)
{
  // 去导入
  ChassisControlSTS.Move(12, 2800, angle + 45, 'n', true); // 1550
  ChassisControlSTS.WaitPositionArrive(1200);
  ChassisControlSTS.ResetMaxSpeed(2);
  ChassisControlSTS.WaitPositionArrive(1500);

  // 重新对其至-90度
  ChassisControlSTS.ResetMaxSpeed(2);
  ChassisControlSTS.ResetStraightAngle(angle + 90);
  ChassisControlSTS.WaitPositionArrive(1900);
  ChassisControlSTS.ResetMaxSpeed(5);
  ChassisControlSTS.ReSetToDistance(470, angle + 90, 'f');
  ChassisControlSTS.WaitMoveFinish();
  digitalE.set(1);
  wait(300, msec);
  // 转至180
  ChassisControlSTS.TurnTo(10, angle, 0);
}

void auto13()
{
  // GoToMiddleBridge(0);
  // return;
  ChassisControlSTS.Over_count = 5;
  digitalH.set(0);
  digitalF.set(0);
  SimulatedRemoteControl("R1");
  ChassisControlSTS.Move(6, 1300, 'n', 1);
  ChassisControlSTS.WaitPositionArrive(1000);
  SimulatedRemoteControl("stop");
  ToolMotor2.spin(forward, -16, voltageUnits::volt);
  ChassisControlSTS.WaitMoveFinish();

  // 中桥
  digitalG.set(0);
  ChassisControlSTS.TurnTo(16, -90 - 45, 0);
  ToolMotor1.spin(forward, -16, voltageUnits::volt);
  ChassisControlSTS.Move(16, 700, -90 - 45, 'u', 1); // 1550
  ChassisControlSTS.WaitPositionArrive(600);
  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  ChassisControlSTS.ChassisLeftControl(-2);
  ChassisControlSTS.ChassisRightControl(-2);
  SimulatedRemoteControl("L2");

  wait(800, msec);
  ChassisControlSTS.SetStopModHold();
  digitalG.set(0);

  // 去导入
  FromGoToMiddleBridge_To_Intake(-180);

  digitalE.set(1);
  // 吸导入
  SimulatedRemoteControl("R1");

  ChassisControlSTS.Move(5, 900, -180, 'n', true); // 1550

  // 使用距离传感器判断是否到达恒力区域
  while (GetDistance_front() > 372)
  {
    wait(10, msec);
  }
  ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
  ChassisControlSTS.StopMoving();
  wait(100, msec);

  move_intake_time(-180, 2000);

  // 去第二个导入桶----------------------------------------------------------
  ChassisControlSTS.Move(8, 6000, -230, 'u', 1);
  ChassisControlSTS.WaitPositionArrive(600);
  ChassisControlSTS.ResetStraightAngle(-182);
  digitalE.set(0);
  ChassisControlSTS.WaitPositionArrive(4000);
  while (ChassisControlSTS.IsMoving())
  {
    if (GetDistance_back() < 1000)
    {
      break;
    }
    wait(10, msec);
  }
  ChassisControlSTS.ResetMaxSpeed(3);
  ChassisControlSTS.ResetStraightAngle(-90);
  ChassisControlSTS.WaitPositionArrive(ChassisControlSTS.GetPositions_mean() + 300);
  ChassisControlSTS.ReSetToDistance(450, -90, 'f');

  ChassisControlSTS.WaitMoveFinish();
  wait(300, msec);

  ChassisControlSTS.TurnTo(10, 0, 0);
  ChassisControlSTS.WaitMoveFinish();

  // 倒车 打球-===========================-====-=
  Move_u_Long(0);
  SimulatedRemoteControl("L1");

  // 努力进入对位
  ChassisControlSTS.ChassisForwardAngle(0);
  ChassisControlSTS.ChassisForward(-2);
  digitalE.set(1);
  wait(1500, msec);
  intake_and_pushback_new(0, 2500, 1500);

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
    if (std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 5)
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
  // wait(2000, msec);

  GoToMiddleBridge(0);
  SimulatedRemoteControl("L2-");
  wait(2000, msec);

  // 第三个技能赛
  FromGoToMiddleBridge_To_Intake(0);

  digitalG.set(0);
  // 吸导入
  SimulatedRemoteControl("R1");

  ChassisControlSTS.Move(5, 900, 0, 'n', true); // 1550

  // 使用距离传感器判断是否到达恒力区域
  while (GetDistance_front() > 372)
  {
    wait(10, msec);
  }
  ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
  ChassisControlSTS.StopMoving();
  wait(100, msec);

  move_intake_time(0, 2500);

  // 去第四个导入桶
  ChassisControlSTS.Move(8, 6000, -50, 'u', 1);
  ChassisControlSTS.WaitPositionArrive(600);
  ChassisControlSTS.ResetStraightAngle(-2);
  digitalE.set(0);
  ChassisControlSTS.WaitPositionArrive(4000);
  while (ChassisControlSTS.IsMoving())
  {
    if (GetDistance_back() < 1000)
    {
      break;
    }
    wait(10, msec);
  }
  ChassisControlSTS.ResetMaxSpeed(3);
  ChassisControlSTS.ResetStraightAngle(90);
  ChassisControlSTS.WaitPositionArrive(ChassisControlSTS.GetPositions_mean() + 300);
  ChassisControlSTS.ReSetToDistance(480, 90, 'f');

  ChassisControlSTS.WaitMoveFinish();
  wait(100, msec);

  ChassisControlSTS.TurnTo(10, 180, 0);
  ChassisControlSTS.WaitMoveFinish();

  // 倒车 打球-===========================-====-=
  Move_u_Long(180);
  SimulatedRemoteControl("L1");
  // 努力进入对位
  ChassisControlSTS.ChassisForwardAngle(180);
  ChassisControlSTS.ChassisForward(-2);
  digitalE.set(1);
  wait(1500, msec);

  intake_and_pushback_new(180, 2500, 1500);

  // 去停泊
  ChassisControlSTS.Move(10, 1000, 30 + 180, 'n', true); // 1550
  ChassisControlSTS.WaitPositionArrive(800);
  ChassisControlSTS.ResetMaxSpeed(5);
  ChassisControlSTS.ReSetToCurveMove_Position(600, 500, 'p');
  ChassisControlSTS.WaitPositionArrive(450);
  ChassisControlSTS.ReSetToEncoderWheel(1500, 65 + 180, 'n');
  ChassisControlSTS.WaitPositionArrive(600);
  ChassisControlSTS.ResetStraightAngle(75 + 180);
  ChassisControlSTS.ResetMaxSpeed(8);
  SimulatedRemoteControl("R2");
  ChassisControlSTS.ReSetToEncoderWheel(1100, 80 + 180, 'n');
  while (ChassisControlSTS.IsMoving())
  {
    if (GetDistance_back() >= 900)
    {
      ChassisControlSTS.ResetMaxSpeed(2);
      digitalE.set(1);
      break;
    }
    wait(10, msec);
  }
  while (ChassisControlSTS.IsMoving())
  {
    if (std::abs(ChassisControlSTS.GetVelocity_pct_mean()) < 5)
    {

      // ChassisControlSTS.StopMoving();
      digitalE.set(1);
      ChassisControlSTS.ReSetToEncoderWheel(100, 80 + 180, 'u');
      break;
    }
    wait(10, msec);
  }
  ChassisControlSTS.WaitMoveFinish();
  digitalE.set(1);

  ChassisControlSTS.ChassisForwardAngle(85 + 180);
  for (int i = 0; i < 100; i++)
  {
    if (i > 50)
    {
      digitalE.set(0);
    }
    ChassisControlSTS.ChassisForward(6);
    wait(10, msec);
  }

  ChassisControlSTS.ChassisForward(6);
}