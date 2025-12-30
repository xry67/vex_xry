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

  ChassisControlSTS.AutoPPSTurnTo(16, -43-85);

  // 运球
  ToolMotor1.spin(forward, -16, voltageUnits::volt);
  ToolMotor2.spin(forward, 10, pct);
  ToolMotor3.spin(forward, 0, voltageUnits::volt);

  ChassisControlSTS.Move_PPS = {0.01, 0, 2};
  ChassisControlSTS.Move(16, 600, 'u', true); // 1550
  ChassisControlSTS.WaitPositionArrive(450);
  ChassisControlSTS.SetStopModCoast();
  ChassisControlSTS.StopMoving();
  ChassisControlSTS.ChassisLeftControl(-2);
  ChassisControlSTS.ChassisRightControl(-2);
  wait(500, msec);
  SimulatedRemoteControl("L2");
  wait(1000, msec);
  SimulatedRemoteControl("R1");
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

  intake_and_pushback_new(-180, 450, 1500);

  // 钩子
  Hook(-180);
}