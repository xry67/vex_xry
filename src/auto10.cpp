#include "vex.h"
#include "robot-config.h"
#include "ChassisControl.h"
#include "AutoTools.h"

// 右6+1+钩 队牌摆位，记得回收钩子
void auto10()
{
  ChassisControlSTS.Over_count = 1;
  ChassisInertial.setRotation(22, deg);
  SimulatedRemoteControl("R1");

  // 前进吸球
  ChassisControlSTS.Move(10, 1300, 'n', true);

  ChassisControlSTS.WaitPositionArrive(400);
  ChassisControlSTS.ResetMaxSpeed(3);
  ChassisControlSTS.WaitPositionArrive(1000);
  digitalH.set(1);
  ChassisControlSTS.WaitMoveFinish();

  ChassisControlSTS.Turn_PPS = {0.025, 0.5, 5};
  ChassisControlSTS.min_turn_speed = 1.6;
  ChassisControlSTS.TurnTo(16, -45, 0);
  digitalH.set(0);

  ChassisControlSTS.Move(10, 700, 'n', true);
  ChassisControlSTS.WaitPositionArrive(400);
  SimulatedRemoteControl("R2");
  ChassisControlSTS.WaitMoveFinish();
  wait(1000, msec);
  SimulatedRemoteControl("R1");

  // 高阶==================================================
  // 斜线到达判定距离传感器位置
  ChassisControlSTS.Move(12, 2800, -45,'u', true);
  ChassisControlSTS.WaitPositionArrive(1000);

  // 重新对齐至-90度
  ChassisControlSTS.ResetMaxSpeed(2);
  ChassisControlSTS.ResetStraightAngle(-90);
  ChassisControlSTS.WaitPositionArrive(1300);

  // 等待到达指定距离
  while (GetDistance_back() > 435)
  {
    std::cout << GetDistance_back() << std::endl;
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
  ChassisControlSTS.AutoPPSTurnTo(10, 180);
  ChassisControlSTS.Over_count = 1;

  intake_and_pushback_new(-180, 450, 2000);

  // 钩子
  Hook(-180);
}