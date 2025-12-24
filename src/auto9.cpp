#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

// 左6+1+钩 队牌摆位

// 一号：正：吸球，后吐球
// 二号：正：分球方向
// 三号：正：中高方向
void auto9()
{
  ChassisControlSTS.Over_count = 1;
  ChassisInertial.setRotation(-22, deg);
  SimulatedRemoteControl("R1");

  // 前进吸球
  ChassisControlSTS.Move(10, 1300, 'n', true);

  ChassisControlSTS.WaitPositionArrive(400);
  ChassisControlSTS.ResetMaxSpeed(3);
  ChassisControlSTS.WaitPositionArrive(1000);
  digitalH.set(1);
  ChassisControlSTS.WaitMoveFinish();

  ChassisControlSTS.AutoPPSTurnTo(16, -43 - 80);

  // 运球
  ToolMotor1.spin(forward, -16, voltageUnits::volt);
  ToolMotor2.spin(forward, 10, pct);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);

  ChassisControlSTS.Move_PPS = {0.01, 0, 2};
  ChassisControlSTS.Move(16, 600, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(550);
  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  ChassisControlSTS.ChassisLeftControl(-2);
  ChassisControlSTS.ChassisRightControl(-2);
  wait(300, msec);
  SimulatedRemoteControl("L2");
  wait(1000, msec);
  ToolMotor1.spin(forward, -16, voltageUnits::volt);
  ToolMotor2.spin(forward, 0, voltageUnits::volt);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);
  ChassisControlSTS.SetStopModHold();
  digitalH.set(0);

  // 高阶==================================================
  // 斜线到达判定距离传感器位置
  ChassisControlSTS.Move(12, 2800, -135, 'n', true); // 1550
  ChassisControlSTS.WaitPositionArrive(1300);

  // 重新对其至-90度
  ChassisControlSTS.ResetMaxSpeed(2);
  ChassisControlSTS.ResetStraightAngle(-90);
  ChassisControlSTS.WaitPositionArrive(1600);

  // 等待到达指定距离
  while (GetDistance_front() > 420)
  {
    // std::cout << GetDistance_front() << std::endl;
    if (!ChassisControlSTS.IsMoving())
    {
      ChassisControlSTS.ChassisLeftControl(2);
      ChassisControlSTS.ChassisRightControl(2);
    }
    wait(10, msec);
  }

  // 刹车
  ChassisControlSTS.ChassisLeftControl(0);
  ChassisControlSTS.ChassisRightControl(0);
  ChassisControlSTS.StopMoving();
  wait(200, msec); // 需要保留等待时间，否则自转的旋转中心会由于惯性发生改变

  digitalH.set(1);

  // 转至180
   ChassisControlSTS.Over_count = 5;
  ChassisControlSTS.AutoPPSTurnTo(10, -180);
   ChassisControlSTS.Over_count = 1;

  // 吸导入
  ToolMotor1.spin(forward, -16, voltageUnits::volt);
  ToolMotor2.spin(forward, 10, pct);
  // ToolMotor3.spin(forward, 25, pct);
  // 运球

  ChassisControlSTS.Move(5, 900, -180, 'n', true); // 1550

  // 使用距离传感器判断是否到达恒力区域
  while (GetDistance_front() > 372)
  {
    wait(10, msec);
  }
  ChassisControlSTS.SetStopModCoast(); // 需要保留，否则容易插不进去
  ChassisControlSTS.StopMoving();
  wait(100, msec);

  AutoIntake();

  // 轻微的蠕动
  for (int i = 0; i < 50; i++)
  {
    if (GetDistance_front() > 137)
    {
      ChassisControlSTS.ChassisLeftControl(5);
      ChassisControlSTS.ChassisRightControl(5);
    }
    else
    {
      ChassisControlSTS.ChassisLeftControl(0);
      ChassisControlSTS.ChassisRightControl(0);
    }
    if ((stance == 'r' && OpticalHaveBlue(Optical1)) || (stance == 'b' && OpticalHaveRed(Optical1)))
    {
      break;
    }
    wait(10, msec);
  }

  StopAutoIntake();

  // 倒车 打球
  ChassisControlSTS.Move(7, 1500, -180, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(500);
  // 运球
  ToolMotor2.spin(forward, 10, pct);
  ToolMotor3.spin(forward, 25, pct);
  ToolMotor1.spin(forward, -2, voltageUnits::volt);

  // 提前打球
  ChassisControlSTS.WaitPositionArrive(1200);
  digitalH.set(0);

  // 打球
  SimulatedRemoteControl("L1");

  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  // 努力进入对位
  ChassisControlSTS.ChassisLeftControl(-2);
  ChassisControlSTS.ChassisRightControl(-2);
  wait(1000, msec);


  ChassisControlSTS.AutoPPSTurnTo(10, -180);
  // 钩子
  ChassisControlSTS.Move_PPS = {0.0182, 0.5, 2};
  ChassisControlSTS.maxinum_speed_acceleration = 0.5;
  ChassisControlSTS.Move(16, 400, 'n'); // 1550

  ChassisControlSTS.maxinum_speed_acceleration_turn = 0.1;
  ChassisControlSTS.AutoPPSTurnTo(10, -220);

  ChassisControlSTS.CurveMove(10, 700, 800, 'd', true);
  ChassisControlSTS.WaitPositionArrive(750);
  ChassisControlSTS.StopMoving();

  ChassisControlSTS.Move(10, 1000, -180, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(550);
  ChassisControlSTS.SetStopModHold();
  ChassisControlSTS.StopMoving();
  ToolMotor1.spin(forward, 0, voltageUnits::volt);
  ToolMotor2.spin(forward, 0, voltageUnits::volt);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);
  // 急刹车
  ChassisControlSTS.ChassisLeftControl(-6);
  ChassisControlSTS.ChassisRightControl(6);
  wait(500, msec);
  ChassisControlSTS.ChassisLeftControl(0);
  ChassisControlSTS.ChassisRightControl(0);
  wait(4000, msec);
}